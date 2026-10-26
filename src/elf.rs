//! Bounds-checked ELF32 little-endian MIPS PT_LOAD loader.

use crate::{iop::{IopMemory, IopMemoryError}, memory::{Memory, MemoryError}};
use std::fmt;

const ELF_HEADER_SIZE: usize = 52;
const ELF32_PHDR_SIZE: usize = 32;
const PT_LOAD: u32 = 1;
const EM_MIPS: u16 = 8;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoadedSegment {
    pub guest_address: u32,
    pub file_size: u32,
    pub memory_size: u32,
    pub flags: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoadedElf {
    pub entry: u32,
    pub segments: Vec<LoadedSegment>,
}

#[derive(Debug)]
pub enum ElfLoadError {
    Truncated(&'static str),
    BadMagic,
    UnsupportedClass(u8),
    UnsupportedEncoding(u8),
    UnsupportedMachine(u16),
    InvalidHeader(&'static str),
    Memory(MemoryError),
    IopMemory(IopMemoryError),
}

impl fmt::Display for ElfLoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Truncated(part) => write!(f, "truncated ELF {part}"),
            Self::BadMagic => write!(f, "input is not an ELF file"),
            Self::UnsupportedClass(class) => write!(f, "unsupported ELF class {class}"),
            Self::UnsupportedEncoding(encoding) => write!(f, "unsupported ELF encoding {encoding}"),
            Self::UnsupportedMachine(machine) => write!(f, "ELF machine {machine} is not MIPS"),
            Self::InvalidHeader(reason) => write!(f, "invalid ELF header: {reason}"),
            Self::Memory(error) => write!(f, "cannot map ELF segment: {error}"),
            Self::IopMemory(error) => write!(f, "cannot map ELF segment into IOP: {error}"),
        }
    }
}

impl std::error::Error for ElfLoadError {}

impl From<MemoryError> for ElfLoadError {
    fn from(value: MemoryError) -> Self {
        Self::Memory(value)
    }
}

impl From<IopMemoryError> for ElfLoadError {
    fn from(value: IopMemoryError) -> Self { Self::IopMemory(value) }
}

fn read_u16(data: &[u8], offset: usize, part: &'static str) -> Result<u16, ElfLoadError> {
    let bytes = data.get(offset..offset + 2).ok_or(ElfLoadError::Truncated(part))?;
    Ok(u16::from_le_bytes(bytes.try_into().unwrap()))
}

fn read_u32(data: &[u8], offset: usize, part: &'static str) -> Result<u32, ElfLoadError> {
    let bytes = data.get(offset..offset + 4).ok_or(ElfLoadError::Truncated(part))?;
    Ok(u32::from_le_bytes(bytes.try_into().unwrap()))
}

fn load_elf32_mips_with(
    data: &[u8],
    mut write_bytes: impl FnMut(u32, &[u8]) -> Result<(), ElfLoadError>,
) -> Result<LoadedElf, ElfLoadError> {
    if data.get(0..4) != Some(b"\x7fELF") {
        return Err(ElfLoadError::BadMagic);
    }
    let class = *data.get(4).ok_or(ElfLoadError::Truncated("ident"))?;
    if class != 1 {
        return Err(ElfLoadError::UnsupportedClass(class));
    }
    let encoding = *data.get(5).ok_or(ElfLoadError::Truncated("ident"))?;
    if encoding != 1 {
        return Err(ElfLoadError::UnsupportedEncoding(encoding));
    }
    if data.len() < ELF_HEADER_SIZE {
        return Err(ElfLoadError::Truncated("header"));
    }
    let machine = read_u16(data, 18, "header")?;
    if machine != EM_MIPS {
        return Err(ElfLoadError::UnsupportedMachine(machine));
    }
    let entry = read_u32(data, 24, "header")?;
    let phoff = read_u32(data, 28, "header")? as usize;
    let ehsize = read_u16(data, 40, "header")? as usize;
    let phentsize = read_u16(data, 42, "header")? as usize;
    let phnum = read_u16(data, 44, "header")? as usize;
    if ehsize < ELF_HEADER_SIZE {
        return Err(ElfLoadError::InvalidHeader("e_ehsize is too small"));
    }
    if phentsize < ELF32_PHDR_SIZE {
        return Err(ElfLoadError::InvalidHeader("e_phentsize is too small"));
    }
    let table_size = phentsize.checked_mul(phnum)
        .and_then(|size| phoff.checked_add(size))
        .ok_or(ElfLoadError::InvalidHeader("program header table overflow"))?;
    if table_size > data.len() {
        return Err(ElfLoadError::Truncated("program header table"));
    }

    let mut segments = Vec::new();
    for index in 0..phnum {
        let offset = phoff + index * phentsize;
        if read_u32(data, offset, "program header")? != PT_LOAD {
            continue;
        }
        let file_offset = read_u32(data, offset + 4, "program header")? as usize;
        let virtual_address = read_u32(data, offset + 8, "program header")?;
        let physical_address = read_u32(data, offset + 12, "program header")?;
        let file_size = read_u32(data, offset + 16, "program header")?;
        let memory_size = read_u32(data, offset + 20, "program header")?;
        let flags = read_u32(data, offset + 24, "program header")?;
        if file_size > memory_size {
            return Err(ElfLoadError::InvalidHeader("p_filesz exceeds p_memsz"));
        }
        let file_end = file_offset.checked_add(file_size as usize)
            .ok_or(ElfLoadError::InvalidHeader("segment file range overflow"))?;
        let file_bytes = data.get(file_offset..file_end).ok_or(ElfLoadError::Truncated("load segment"))?;
        let guest_address = if physical_address != 0 { physical_address } else { virtual_address };
        write_bytes(guest_address, file_bytes)?;
        let zero_size = (memory_size - file_size) as usize;
        if zero_size != 0 {
            let zeros = vec![0; zero_size];
            write_bytes(guest_address.checked_add(file_size)
                .ok_or(ElfLoadError::InvalidHeader("guest segment address overflow"))?, &zeros)?;
        }
        segments.push(LoadedSegment { guest_address, file_size, memory_size, flags });
    }
    if segments.is_empty() {
        return Err(ElfLoadError::InvalidHeader("no PT_LOAD segments"));
    }
    Ok(LoadedElf { entry, segments })
}

pub fn load_elf32_mips(data: &[u8], memory: &mut Memory) -> Result<LoadedElf, ElfLoadError> {
    load_elf32_mips_with(data, |address, bytes| memory.write_bytes(address, bytes).map_err(ElfLoadError::from))
}

pub fn load_elf32_mips_iop(data: &[u8], memory: &mut IopMemory) -> Result<LoadedElf, ElfLoadError> {
    load_elf32_mips_with(data, |address, bytes| memory.write_bytes(address, bytes).map_err(ElfLoadError::from))
}
