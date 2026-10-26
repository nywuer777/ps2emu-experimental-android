//! Shared SIF mailbox and flag registers for the EE and IOP memory buses.

use std::sync::{Arc, Mutex};

const EE_SIF_BASE: u32 = 0x1000_f000;
const IOP_SIF_BASE: u32 = 0x1d00_0000;
pub const SIF_FIFO_CAPACITY_WORDS: usize = 128;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SifChannel {
    Sif0,
    Sif1,
}

#[derive(Clone, Default)]
pub struct SifBus {
    registers: Arc<Mutex<SifRegisters>>,
}

#[derive(Default)]
struct SifRegisters {
    ee_command: u32,
    iop_command: u32,
    ee_flags: u32,
    iop_flags: u32,
    control: u32,
    bus_directory: u32,
    fifos: [SifFifo; 2],
}

#[derive(Clone)]
struct SifFifo {
    words: [u32; SIF_FIFO_CAPACITY_WORDS],
    read_pos: usize,
    write_pos: usize,
    len: usize,
}

impl Default for SifFifo {
    fn default() -> Self {
        Self { words: [0; SIF_FIFO_CAPACITY_WORDS], read_pos: 0, write_pos: 0, len: 0 }
    }
}

impl SifBus {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn fifo_len(&self, channel: SifChannel) -> usize {
        self.registers.lock().map(|state| state.fifos[Self::fifo_index(channel)].len).unwrap_or(0)
    }

    pub fn fifo_free(&self, channel: SifChannel) -> usize {
        SIF_FIFO_CAPACITY_WORDS - self.fifo_len(channel)
    }

    pub fn write_fifo(&self, channel: SifChannel, words: &[u32]) -> usize {
        let Ok(mut state) = self.registers.lock() else { return 0; };
        let fifo = &mut state.fifos[Self::fifo_index(channel)];
        let count = words.len().min(SIF_FIFO_CAPACITY_WORDS - fifo.len);
        for word in &words[..count] {
            let position = fifo.write_pos;
            fifo.words[position] = *word;
            fifo.write_pos = (position + 1) % SIF_FIFO_CAPACITY_WORDS;
        }
        fifo.len += count;
        count
    }

    pub fn read_fifo(&self, channel: SifChannel, output: &mut [u32]) -> usize {
        let Ok(mut state) = self.registers.lock() else { return 0; };
        let fifo = &mut state.fifos[Self::fifo_index(channel)];
        let count = output.len().min(fifo.len);
        for word in &mut output[..count] {
            let position = fifo.read_pos;
            *word = fifo.words[position];
            fifo.read_pos = (position + 1) % SIF_FIFO_CAPACITY_WORDS;
        }
        fifo.len -= count;
        count
    }

    /// Transfer a whole DMA unit or leave the FIFO untouched.
    pub fn read_fifo_exact(&self, channel: SifChannel, output: &mut [u32]) -> bool {
        let Ok(mut state) = self.registers.lock() else { return false; };
        let fifo = &mut state.fifos[Self::fifo_index(channel)];
        if fifo.len < output.len() { return false; }
        for word in output.iter_mut() {
            *word = fifo.words[fifo.read_pos];
            fifo.read_pos = (fifo.read_pos + 1) % SIF_FIFO_CAPACITY_WORDS;
        }
        fifo.len -= output.len();
        true
    }

    pub fn write_fifo_exact(&self, channel: SifChannel, words: &[u32]) -> bool {
        let Ok(mut state) = self.registers.lock() else { return false; };
        let fifo = &mut state.fifos[Self::fifo_index(channel)];
        if SIF_FIFO_CAPACITY_WORDS - fifo.len < words.len() { return false; }
        for word in words {
            fifo.words[fifo.write_pos] = *word;
            fifo.write_pos = (fifo.write_pos + 1) % SIF_FIFO_CAPACITY_WORDS;
        }
        fifo.len += words.len();
        true
    }

    fn fifo_index(channel: SifChannel) -> usize {
        match channel { SifChannel::Sif0 => 0, SifChannel::Sif1 => 1 }
    }

    pub(crate) fn read_ee(&self, address: u32) -> Option<u32> {
        let registers = self.registers.lock().ok()?;
        let offset = address.checked_sub(EE_SIF_BASE)?;
        match offset {
            0x200 => Some(registers.ee_command),
            0x210 => Some(registers.iop_command),
            0x220 => Some(registers.ee_flags),
            0x230 => Some(registers.iop_flags),
            0x240 => Some(registers.control | 0xf000_0102),
            0x250 => Some(0),
            0x260 => Some(registers.bus_directory),
            _ => None,
        }
    }

    pub(crate) fn write_ee(&self, address: u32, value: u32) -> bool {
        let Some(offset) = address.checked_sub(EE_SIF_BASE) else { return false; };
        let Ok(mut registers) = self.registers.lock() else { return false; };
        match offset {
            0x200 => registers.ee_command = value,
            0x210 => registers.iop_command = value,
            0x220 => registers.ee_flags |= value,
            0x230 => registers.iop_flags &= !value,
            0x240 => {
                if value & 0x100 != 0 { registers.control |= 0x100; }
                else { registers.control &= !0x100; }
            }
            0x250 => {}
            0x260 => registers.bus_directory = value,
            _ => return false,
        }
        true
    }

    pub(crate) fn read_iop(&self, address: u32) -> Option<u32> {
        let registers = self.registers.lock().ok()?;
        let raw_offset = address.checked_sub(IOP_SIF_BASE)?;
        if raw_offset >= 0x100 { return None; }
        let offset = raw_offset & 0x8f0;
        match offset {
            0x000 => Some(registers.ee_command),
            0x010 => Some(registers.iop_command),
            0x020 => Some(registers.ee_flags),
            0x030 => Some(registers.iop_flags),
            0x040 => Some(registers.control | 0xf000_0002),
            0x060 => Some(0),
            _ => None,
        }
    }

    pub(crate) fn write_iop(&self, address: u32, value: u32) -> bool {
        let Some(raw_offset) = address.checked_sub(IOP_SIF_BASE) else { return false; };
        if raw_offset >= 0x100 { return false; }
        let offset = raw_offset & 0x8f0;
        let Ok(mut registers) = self.registers.lock() else { return false; };
        match offset {
            0x000 => {}
            0x010 => registers.iop_command = value,
            0x020 => registers.ee_flags &= !value,
            0x030 => registers.iop_flags |= value,
            0x040 => {
                if value & (0x20 | 0x80) != 0 {
                    registers.control = (registers.control & !0xf000) | 0x2000;
                }
                let toggles = value & 0xf0;
                registers.control ^= toggles;
            }
            0x060 => registers.bus_directory = 0,
            _ => return false,
        }
        true
    }
}

pub(crate) fn aligned_register(address: u32, base: u32, iop: bool) -> Option<u32> {
    let aligned = address & !3;
    if iop {
        let raw_offset = aligned.checked_sub(base)?;
        if raw_offset >= 0x100 { return None; }
        let offset = raw_offset & 0x8f0;
        if matches!(offset, 0x000 | 0x010 | 0x020 | 0x030 | 0x040 | 0x060) {
            Some(aligned)
        } else {
            None
        }
    } else {
        let offset = aligned.checked_sub(base)?;
        if matches!(offset, 0x200 | 0x210 | 0x220 | 0x230 | 0x240 | 0x250 | 0x260) {
            Some(aligned)
        } else {
            None
        }
    }
}

pub(crate) fn write_partial(bus: &SifBus, address: u32, width: usize, value: u32, iop: bool) -> bool {
    let base = if iop { IOP_SIF_BASE } else { EE_SIF_BASE };
    let Some(register_address) = aligned_register(address, base, iop) else { return false; };
    let shift = (address & 3) * 8;
    let mask = match width { 1 => 0xff, 2 => 0xffff, _ => u32::MAX };
    let shifted_mask = mask.checked_shl(shift).unwrap_or(0);
    let shifted_value = value.wrapping_shl(shift) & shifted_mask;
    let offset = if iop { register_address.wrapping_sub(IOP_SIF_BASE) & 0x8f0 } else { register_address.wrapping_sub(EE_SIF_BASE) };
    let write_one_action = if iop { matches!(offset, 0x020 | 0x030) } else { matches!(offset, 0x220 | 0x230) };
    let merged = if write_one_action {
        shifted_value
    } else {
        let current = if iop { bus.read_iop(register_address) } else { bus.read_ee(register_address) }.unwrap_or(0);
        (current & !shifted_mask) | shifted_value
    };
    if iop { bus.write_iop(register_address, merged) } else { bus.write_ee(register_address, merged) }
}
