//! VIF command-stream decoder and initial VU memory upload path.

use std::fmt;

use crate::memory::Memory;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum VifError {
    InvalidUnit(u8),
    Truncated { offset: usize, needed: usize, available: usize },
    UnsupportedUnpack(u8),
    UnsupportedCommand(u8),
    Gif(String),
}

impl fmt::Display for VifError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidUnit(unit) => write!(f, "invalid VIF unit {unit}"),
            Self::Truncated { offset, needed, available } => write!(f, "truncated VIF stream at {offset:#x}: needs {needed} bytes, has {available}"),
            Self::UnsupportedUnpack(format) => write!(f, "unsupported VIF UNPACK format {format:#x}"),
            Self::UnsupportedCommand(command) => write!(f, "unsupported VIF command {command:#x}"),
            Self::Gif(error) => write!(f, "VIF DIRECT GIF transfer failed: {error}"),
        }
    }
}

impl std::error::Error for VifError {}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum VifEvent {
    StartMicroProgram { address: u16, flush: bool, top: u16, itop: u16 },
    ContinueMicroProgram { top: u16, itop: u16 },
    Mark(u16),
}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct VifReport {
    pub bytes_consumed: usize,
    pub unpack_vectors: usize,
    pub micro_bytes: usize,
    pub direct_qwords: usize,
    pub events: Vec<VifEvent>,
}

/// Stateful command decoder for VIF0 or VIF1. The caller supplies each DMA
/// stream in order and handles returned MSCAL events with the selected VU.
#[derive(Debug, Clone)]
pub struct Vif {
    unit: u8,
    cycle_cl: usize,
    cycle_wl: usize,
    mode: u8,
    base: u32,
    offset: u32,
    tops: u32,
    itop: u32,
    double_buffer_flag: bool,
    mask: u32,
    row: [u32; 4],
    col: [u32; 4],
}

impl Vif {
    pub fn new(unit: u8) -> Result<Self, VifError> {
        if unit > 1 { return Err(VifError::InvalidUnit(unit)); }
        Ok(Self {
            unit,
            cycle_cl: 1,
            cycle_wl: 1,
            mode: 0,
            base: 0,
            offset: 0,
            tops: 0,
            itop: 0,
            double_buffer_flag: false,
            mask: 0,
            row: [0; 4],
            col: [0; 4],
        })
    }

    pub fn unit(&self) -> u8 { self.unit }

    /// Process an ordered stream of VIF command words and their payloads.
    /// UNPACK writes VU data memory; MPG writes VU micro memory; VIF1 DIRECT
    /// forwards its quadwords to the existing GIF/GS path.
    pub fn process_stream(&mut self, stream: &[u8], memory: &mut Memory) -> Result<VifReport, VifError> {
        let mut report = VifReport::default();
        let mut cursor = 0usize;
        while cursor < stream.len() {
            let command = read_u32(stream, &mut cursor)?;
            let opcode = ((command >> 24) & 0x7f) as u8;
            let num = ((command >> 16) & 0xff) as usize;
            let immediate = command as u16;
            match opcode {
                0x00 | 0x10 | 0x11 | 0x13 | 0x06 => {}
                0x07 => report.events.push(VifEvent::Mark(immediate)),
                0x01 => {
                    self.cycle_cl = match (immediate & 0xff) as usize { 0 => 256, n => n };
                    self.cycle_wl = match ((immediate >> 8) & 0xff) as usize { 0 => 256, n => n };
                }
                0x02 => {
                    if self.unit == 1 {
                        self.offset = (immediate & 0x3ff) as u32;
                        self.base = self.tops;
                        self.double_buffer_flag = false;
                    }
                }
                0x03 if self.unit == 1 => self.base = (immediate & 0x3ff) as u32,
                0x04 => self.itop = (immediate & if self.unit == 0 { 0xff } else { 0x3ff }) as u32,
                0x05 => self.mode = (immediate & 3) as u8,
                0x14 | 0x15 => report.events.push(VifEvent::StartMicroProgram {
                    address: immediate & 0x3ff,
                    flush: opcode == 0x15,
                    top: self.tops as u16,
                    itop: self.itop as u16,
                }),
                0x17 => report.events.push(VifEvent::ContinueMicroProgram {
                    top: self.tops as u16,
                    itop: self.itop as u16,
                }),
                0x20 => self.mask = read_u32(stream, &mut cursor)?,
                0x30 => {
                    for lane in 0..4 { self.row[lane] = read_u32(stream, &mut cursor)?; }
                }
                0x31 => {
                    for lane in 0..4 { self.col[lane] = read_u32(stream, &mut cursor)?; }
                }
                0x4a => {
                    let count = if num == 0 { 256 } else { num };
                    let size = count * 8;
                    let payload = take(stream, &mut cursor, size)?;
                    if !memory.write_vu_micro_bytes(self.unit, (immediate as usize & 0x3ff) * 8, payload) {
                        return Err(VifError::InvalidUnit(self.unit));
                    }
                    report.micro_bytes += size;
                }
                0x50 | 0x51 if self.unit == 1 => {
                    let count = if immediate == 0 { 65_536 } else { immediate as usize };
                    let size = count.checked_mul(16).ok_or(VifError::Truncated { offset: cursor, needed: usize::MAX, available: stream.len().saturating_sub(cursor) })?;
                    let payload = take(stream, &mut cursor, size)?;
                    let qwords = payload.chunks_exact(16).map(|q| u128::from_le_bytes(q.try_into().expect("16-byte GIF qword"))).collect::<Vec<_>>();
                    memory.gs_mut().submit_gif(&qwords).map_err(|e| VifError::Gif(e.to_string()))?;
                    memory.record_vif_direct_qwords(count as u64);
                    report.direct_qwords += count;
                }
                0x60..=0x7f => {
                    let format = opcode & 0x0f;
                    let masked = opcode & 0x10 != 0;
                    let count = if num == 0 { 256 } else { num };
                    let bits = format_bits(format).ok_or(VifError::UnsupportedUnpack(format))?;
                    let components = format_components(format);
                    let input_size = (count * components * bits).div_ceil(32) * 4;
                    let payload = take(stream, &mut cursor, input_size)?;
                    let mut unpack_address = (immediate as u32) & 0x3ff;
                    if immediate & (1 << 15) != 0 && self.unit == 1 {
                        unpack_address = unpack_address.wrapping_add(self.tops);
                    }
                    self.unpack(payload, count, format, immediate & (1 << 14) != 0, masked, &mut unpack_address, memory, &mut report)?;
                }
                _ => return Err(VifError::UnsupportedCommand(opcode)),
            }
            if matches!(opcode, 0x14 | 0x15 | 0x17) && self.unit == 1 {
                // TOP is latched for the program being started, then TOPS
                // advances to the opposite double buffer for later UNPACKs.
                if self.double_buffer_flag {
                    self.tops = self.base;
                    self.double_buffer_flag = false;
                } else {
                    self.tops = self.base.wrapping_add(self.offset) & 0x3ff;
                    self.double_buffer_flag = true;
                }
            }
        }
        report.bytes_consumed = cursor;
        Ok(report)
    }

    fn unpack(
        &mut self,
        data: &[u8],
        count: usize,
        format: u8,
        unsigned: bool,
        masked: bool,
        destination: &mut u32,
        memory: &mut Memory,
        report: &mut VifReport,
    ) -> Result<(), VifError> {
        let bits = format_bits(format).ok_or(VifError::UnsupportedUnpack(format))?;
        let components = format_components(format);
        let mut bit_cursor = 0usize;
        for vector_index in 0..count {
            let input_cycle = vector_index % self.cycle_cl;
            let write_vector = self.cycle_wl >= self.cycle_cl || input_cycle < self.cycle_wl;
            let mut vector = [0u32; 4];
            for component in 0..components {
                let raw = read_bits(data, bit_cursor, bits);
                bit_cursor += bits;
                let value = extend(raw, bits, unsigned);
                vector[component] = match self.mode {
                    1 => value.wrapping_add(self.row[component]),
                    2 => {
                        let sum = value.wrapping_add(self.row[component]);
                        self.row[component] = sum;
                        sum
                    }
                    _ => value,
                };
            }
            if !write_vector { continue; }
            let old = memory.read_vu_data_vector(self.unit, *destination).ok_or(VifError::InvalidUnit(self.unit))?;
            if masked {
                for component in 0..4 {
                    let action = (self.mask >> (((vector_index & 3) * 8 + component * 2) as u32)) & 3;
                    vector[component] = match action {
                        0 => vector[component],
                        1 => self.row[component],
                        2 => self.col[component],
                        _ => old[component],
                    };
                }
            }
            memory.write_vu_data_vector(self.unit, *destination, vector);
            *destination = destination.wrapping_add(1);
            if self.cycle_wl > self.cycle_cl && input_cycle == self.cycle_cl - 1 {
                *destination = destination.wrapping_add((self.cycle_wl - self.cycle_cl) as u32);
            }
            report.unpack_vectors += 1;
        }
        Ok(())
    }
}

fn read_u32(stream: &[u8], cursor: &mut usize) -> Result<u32, VifError> {
    let bytes = take(stream, cursor, 4)?;
    Ok(u32::from_le_bytes(bytes.try_into().expect("four-byte VIF word")))
}

fn take<'a>(stream: &'a [u8], cursor: &mut usize, size: usize) -> Result<&'a [u8], VifError> {
    let end = cursor.checked_add(size).ok_or(VifError::Truncated { offset: *cursor, needed: size, available: stream.len().saturating_sub(*cursor) })?;
    if end > stream.len() {
        return Err(VifError::Truncated { offset: *cursor, needed: size, available: stream.len().saturating_sub(*cursor) });
    }
    let data = &stream[*cursor..end];
    *cursor = end;
    Ok(data)
}

fn format_components(format: u8) -> usize {
    match format { 0..=2 => 1, 4..=6 => 2, 8..=10 => 3, 12..=14 => 4, _ => 0 }
}

fn format_bits(format: u8) -> Option<usize> {
    match format {
        0 | 4 | 8 | 12 => Some(32),
        1 | 5 | 9 | 13 => Some(16),
        2 | 6 | 10 | 14 => Some(8),
        _ => None,
    }
}

fn read_bits(data: &[u8], offset: usize, bits: usize) -> u32 {
    let mut value = 0u32;
    for bit in 0..bits {
        let source = offset + bit;
        value |= (((data[source / 8] >> (source & 7)) & 1) as u32) << bit;
    }
    value
}

fn extend(value: u32, bits: usize, unsigned: bool) -> u32 {
    if unsigned || bits == 32 { return value; }
    let shift = 32 - bits;
    (((value << shift) as i32) >> shift) as u32
}
