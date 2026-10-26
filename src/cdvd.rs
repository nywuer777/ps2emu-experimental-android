//! Read-only cooked-sector reader for ISO 9660 images used by PS2 discs.

use std::{
    error::Error,
    fmt,
    fs::File,
    io::{Read, Seek, SeekFrom},
    path::Path,
    sync::{Arc, Mutex},
};

pub const ISO_SECTOR_SIZE: usize = 2048;
const RAW_SECTOR_SIZE: usize = 2352;
const PVD_LBA: u32 = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SectorFormat {
    Cooked2048,
    Raw2352Mode1,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IsoEntry {
    pub name: String,
    pub extent_lba: u32,
    pub size: u32,
    pub is_directory: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CdvdError {
    InvalidImage,
    InvalidVolumeDescriptor,
    InvalidDirectoryRecord,
    UnsupportedInterleave,
    NotFound(String),
    NotDirectory(String),
    RangeOverflow,
    Io(String),
}

impl fmt::Display for CdvdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidImage => write!(f, "image is not a supported 2048-byte ISO or 2352-byte Mode 1 disc"),
            Self::InvalidVolumeDescriptor => write!(f, "ISO 9660 primary volume descriptor is invalid"),
            Self::InvalidDirectoryRecord => write!(f, "ISO 9660 directory record is invalid"),
            Self::UnsupportedInterleave => write!(f, "interleaved ISO 9660 file sections are unsupported"),
            Self::NotFound(path) => write!(f, "ISO path not found: {path}"),
            Self::NotDirectory(path) => write!(f, "ISO path is not a directory: {path}"),
            Self::RangeOverflow => write!(f, "ISO extent exceeds image bounds"),
            Self::Io(message) => write!(f, "disc image I/O failed: {message}"),
        }
    }
}

impl Error for CdvdError {}

#[derive(Debug, Clone)]
pub struct CdvdImage {
    source: ImageSource,
    image_len: u64,
    format: SectorFormat,
    volume_blocks: u32,
    volume_id: String,
    root: IsoEntry,
}

#[derive(Debug, Clone)]
enum ImageSource {
    Memory(Vec<u8>),
    File(Arc<Mutex<File>>),
}

impl CdvdImage {
    pub fn from_bytes(image: Vec<u8>) -> Result<Self, CdvdError> {
        let image_len = image.len() as u64;
        let format = if image.len() % ISO_SECTOR_SIZE == 0
            && has_pvd(&image, ISO_SECTOR_SIZE, 0)
        {
            SectorFormat::Cooked2048
        } else if image.len() % RAW_SECTOR_SIZE == 0
            && has_pvd(&image, RAW_SECTOR_SIZE, 16)
        {
            SectorFormat::Raw2352Mode1
        } else {
            return Err(CdvdError::InvalidImage);
        };

        let mut disc = Self {
            source: ImageSource::Memory(image),
            image_len,
            format,
            volume_blocks: 0,
            volume_id: String::new(),
            root: IsoEntry { name: String::from("/"), extent_lba: 0, size: 0, is_directory: true },
        };
        disc.read_volume_descriptor()?;
        Ok(disc)
    }

    /// Open an ISO image without loading its bytes into memory. Sector reads
    /// are serialized on the file handle and allocate only the requested data.
    pub fn open(path: impl AsRef<Path>) -> Result<Self, CdvdError> {
        let mut file = File::open(path).map_err(|error| CdvdError::Io(error.to_string()))?;
        let image_len = file.metadata().map_err(|error| CdvdError::Io(error.to_string()))?.len();
        let cooked = image_len % ISO_SECTOR_SIZE as u64 == 0
            && has_pvd_file(&mut file, ISO_SECTOR_SIZE as u64, 0)?;
        let raw = !cooked
            && image_len % RAW_SECTOR_SIZE as u64 == 0
            && has_pvd_file(&mut file, RAW_SECTOR_SIZE as u64, 16)?;
        let format = if cooked {
            SectorFormat::Cooked2048
        } else if raw {
            SectorFormat::Raw2352Mode1
        } else {
            return Err(CdvdError::InvalidImage);
        };

        let mut disc = Self {
            source: ImageSource::File(Arc::new(Mutex::new(file))),
            image_len,
            format,
            volume_blocks: 0,
            volume_id: String::new(),
            root: IsoEntry { name: String::from("/"), extent_lba: 0, size: 0, is_directory: true },
        };
        disc.read_volume_descriptor()?;
        Ok(disc)
    }

    fn read_volume_descriptor(&mut self) -> Result<(), CdvdError> {
        let pvd = self.read_sector(PVD_LBA)?;
        if pvd[0] != 1 || &pvd[1..6] != b"CD001" || pvd[6] != 1 {
            return Err(CdvdError::InvalidVolumeDescriptor);
        }
        self.volume_blocks = read_both_u32(&pvd, 80)?;
        if read_both_u16(&pvd, 128)? as usize != ISO_SECTOR_SIZE {
            return Err(CdvdError::InvalidVolumeDescriptor);
        }
        self.volume_id = String::from_utf8_lossy(&pvd[40..72]).trim().to_string();
        let root_len = pvd[156] as usize;
        if root_len < 34 || 156 + root_len > pvd.len() {
            return Err(CdvdError::InvalidVolumeDescriptor);
        }
        self.root = parse_directory_record(&pvd[156..156 + root_len])?;
        if !self.root.is_directory {
            return Err(CdvdError::InvalidVolumeDescriptor);
        }
        Ok(())
    }

    pub fn format(&self) -> SectorFormat { self.format }
    pub fn volume_id(&self) -> &str { &self.volume_id }
    pub fn volume_blocks(&self) -> u32 { self.volume_blocks }

    pub fn read_sector(&self, lba: u32) -> Result<[u8; ISO_SECTOR_SIZE], CdvdError> {
        // The primary volume descriptor itself lives at LBA 16 and supplies
        // `volume_blocks`. Until that descriptor has been read, enforce only
        // the physical image length; afterward enforce the declared volume.
        if self.volume_blocks != 0 && lba >= self.volume_blocks {
            return Err(CdvdError::RangeOverflow);
        }
        let (stride, payload_offset) = match self.format {
            SectorFormat::Cooked2048 => (ISO_SECTOR_SIZE, 0),
            SectorFormat::Raw2352Mode1 => (RAW_SECTOR_SIZE, 16),
        };
        let start = (lba as u64)
            .checked_mul(stride as u64)
            .and_then(|value| value.checked_add(payload_offset as u64))
            .ok_or(CdvdError::RangeOverflow)?;
        let end = start.checked_add(ISO_SECTOR_SIZE as u64).ok_or(CdvdError::RangeOverflow)?;
        if end > self.image_len { return Err(CdvdError::RangeOverflow); }
        let mut sector = [0; ISO_SECTOR_SIZE];
        match &self.source {
            ImageSource::Memory(image) => {
                let start = usize::try_from(start).map_err(|_| CdvdError::RangeOverflow)?;
                let end = start.checked_add(ISO_SECTOR_SIZE).ok_or(CdvdError::RangeOverflow)?;
                sector.copy_from_slice(image.get(start..end).ok_or(CdvdError::RangeOverflow)?);
            }
            ImageSource::File(file) => {
                let mut file = file.lock().map_err(|_| CdvdError::Io(String::from("image lock poisoned")))?;
                file.seek(SeekFrom::Start(start)).map_err(|error| CdvdError::Io(error.to_string()))?;
                file.read_exact(&mut sector).map_err(|error| CdvdError::Io(error.to_string()))?;
            }
        }
        Ok(sector)
    }

    pub fn lookup(&self, path: &str) -> Result<IsoEntry, CdvdError> {
        let mut current = self.root.clone();
        let mut walked = String::from("/");
        for part in path.split(['/', '\\']).filter(|part| !part.is_empty()) {
            if !current.is_directory { return Err(CdvdError::NotDirectory(walked)); }
            let entries = self.directory_entries(&current)?;
            let wanted = normalize_name(part);
            let next = entries.into_iter().find(|entry| normalize_name(&entry.name) == wanted)
                .ok_or_else(|| CdvdError::NotFound(path.to_string()))?;
            walked.push_str(&next.name);
            if next.is_directory { walked.push('/'); }
            current = next;
        }
        Ok(current)
    }

    pub fn read_file(&self, path: &str) -> Result<Vec<u8>, CdvdError> {
        let entry = self.lookup(path)?;
        if entry.is_directory { return Err(CdvdError::NotDirectory(path.to_string())); }
        self.read_extent(entry.extent_lba, entry.size)
    }

    pub fn boot_executable(&self) -> Result<Option<String>, CdvdError> {
        let config = match self.read_file("SYSTEM.CNF") {
            Ok(bytes) => bytes,
            Err(CdvdError::NotFound(_)) => return Ok(None),
            Err(error) => return Err(error),
        };
        let text = String::from_utf8_lossy(&config);
        for line in text.lines() {
            let Some((key, value)) = line.split_once('=') else { continue };
            if key.trim().eq_ignore_ascii_case("BOOT2") || key.trim().eq_ignore_ascii_case("BOOT") {
                return Ok(Some(value.trim().trim_matches('"').to_string()));
            }
        }
        Ok(None)
    }

    /// Resolve the executable named by SYSTEM.CNF and return its ISO path and
    /// bytes. Device prefixes (for example `cdrom0:`) and surrounding quotes
    /// are accepted; ISO version suffixes are handled by normal name matching.
    pub fn read_boot_executable(&self) -> Result<Option<(String, Vec<u8>)>, CdvdError> {
        let Some(boot_path) = self.boot_executable()? else { return Ok(None) };
        let path = boot_path
            .split_once(':')
            .map(|(_, path)| path)
            .unwrap_or(&boot_path)
            .trim_start_matches(['/', '\\']);
        let bytes = self.read_file(path)?;
        Ok(Some((boot_path, bytes)))
    }

    fn directory_entries(&self, directory: &IsoEntry) -> Result<Vec<IsoEntry>, CdvdError> {
        let bytes = self.read_extent(directory.extent_lba, directory.size)?;
        let mut entries = Vec::new();
        let mut cursor = 0usize;
        while cursor < bytes.len() {
            let record_len = bytes[cursor] as usize;
            if record_len == 0 {
                cursor = ((cursor / ISO_SECTOR_SIZE) + 1) * ISO_SECTOR_SIZE;
                continue;
            }
            let end = cursor.checked_add(record_len).ok_or(CdvdError::RangeOverflow)?;
            let record = bytes.get(cursor..end).ok_or(CdvdError::InvalidDirectoryRecord)?;
            let name_len = *record.get(32).ok_or(CdvdError::InvalidDirectoryRecord)? as usize;
            let name_start = 33usize;
            if name_len == 1 && record.get(name_start).is_some_and(|id| *id <= 1) {
                cursor = end;
                continue;
            }
            entries.push(parse_directory_record(record)?);
            cursor = end;
        }
        Ok(entries)
    }

    fn read_extent(&self, lba: u32, size: u32) -> Result<Vec<u8>, CdvdError> {
        let size = size as usize;
        let mut result = Vec::with_capacity(size);
        let mut remaining = size;
        let mut sector_lba = lba;
        while remaining > 0 {
            let sector = self.read_sector(sector_lba)?;
            let take = remaining.min(ISO_SECTOR_SIZE);
            result.extend_from_slice(&sector[..take]);
            remaining -= take;
            sector_lba = sector_lba.checked_add(1).ok_or(CdvdError::RangeOverflow)?;
        }
        Ok(result)
    }
}

fn has_pvd(image: &[u8], stride: usize, payload: usize) -> bool {
    let Some(start) = (PVD_LBA as usize).checked_mul(stride).and_then(|v| v.checked_add(payload)) else { return false; };
    image.get(start..start + 7).is_some_and(|pvd| pvd[0] == 1 && &pvd[1..6] == b"CD001" && pvd[6] == 1)
}

fn has_pvd_file(file: &mut File, stride: u64, payload: u64) -> Result<bool, CdvdError> {
    let offset = (PVD_LBA as u64)
        .checked_mul(stride)
        .and_then(|value| value.checked_add(payload))
        .ok_or(CdvdError::RangeOverflow)?;
    let file_len = file
        .metadata()
        .map_err(|error| CdvdError::Io(error.to_string()))?
        .len();
    if offset.checked_add(7).map_or(true, |end| end > file_len) {
        return Ok(false);
    }
    let mut descriptor = [0; 7];
    file.seek(SeekFrom::Start(offset)).map_err(|error| CdvdError::Io(error.to_string()))?;
    file.read_exact(&mut descriptor).map_err(|error| CdvdError::Io(error.to_string()))?;
    Ok(descriptor[0] == 1 && &descriptor[1..6] == b"CD001" && descriptor[6] == 1)
}

fn read_both_u16(data: &[u8], offset: usize) -> Result<u16, CdvdError> {
    let le = u16::from_le_bytes(data.get(offset..offset + 2).ok_or(CdvdError::InvalidVolumeDescriptor)?.try_into().unwrap());
    let be = u16::from_be_bytes(data.get(offset + 2..offset + 4).ok_or(CdvdError::InvalidVolumeDescriptor)?.try_into().unwrap());
    if le != be { return Err(CdvdError::InvalidVolumeDescriptor); }
    Ok(le)
}

fn read_both_u32(data: &[u8], offset: usize) -> Result<u32, CdvdError> {
    let le = u32::from_le_bytes(data.get(offset..offset + 4).ok_or(CdvdError::InvalidVolumeDescriptor)?.try_into().unwrap());
    let be = u32::from_be_bytes(data.get(offset + 4..offset + 8).ok_or(CdvdError::InvalidVolumeDescriptor)?.try_into().unwrap());
    if le != be { return Err(CdvdError::InvalidVolumeDescriptor); }
    Ok(le)
}

fn parse_directory_record(record: &[u8]) -> Result<IsoEntry, CdvdError> {
    if record.len() < 34 || record[0] as usize > record.len() { return Err(CdvdError::InvalidDirectoryRecord); }
    let name_len = record[32] as usize;
    let name_end = 33usize.checked_add(name_len).ok_or(CdvdError::RangeOverflow)?;
    if name_end > record.len() { return Err(CdvdError::InvalidDirectoryRecord); }
    let extent = read_both_u32(record, 2).map_err(|_| CdvdError::InvalidDirectoryRecord)?;
    let size = read_both_u32(record, 10).map_err(|_| CdvdError::InvalidDirectoryRecord)?;
    let flags = record[25];
    if record[26] != 0 || record[27] != 0 || flags & 0x80 != 0 { return Err(CdvdError::UnsupportedInterleave); }
    let identifier = &record[33..name_end];
    let name = match identifier {
        [0] => String::from("."),
        [1] => String::from(".."),
        bytes => String::from_utf8_lossy(bytes).into_owned(),
    };
    Ok(IsoEntry { name, extent_lba: extent + record[1] as u32, size, is_directory: flags & 2 != 0 })
}

fn normalize_name(name: &str) -> String {
    let clean = name.trim_end_matches('/');
    clean.split_once(';').map_or(clean, |(base, _)| base).to_ascii_uppercase()
}
