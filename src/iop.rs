//! Early IOP R3000A interpreter and memory slice.

use std::collections::VecDeque;
use std::fmt;
use crate::cdvd::{CdvdImage, ISO_SECTOR_SIZE};
use crate::memory::{BIOS_ROM_BASE, BIOS_ROM_SIZE};
use crate::sif::SifBus;
use crate::spu2::{Spu2, SPU2_BASE, SPU2_REGISTER_SIZE};
use crate::sio2::{Sio2, SIO2_BASE, SIO2_REGISTER_SIZE};

pub const IOP_RAM_SIZE: usize = 2 * 1024 * 1024;
pub const IOP_SCRATCH_BASE: u32 = 0x1f80_0000;
pub const IOP_SCRATCH_SIZE: usize = 1024;
const CDVD_BASE: u32 = 0x1f40_2000;
const DMA3_MADR: u32 = 0x1f80_10b0;
const DMA3_BCR: u32 = 0x1f80_10b4;
const DMA3_CHCR: u32 = 0x1f80_10b8;
const DMA7_MADR: u32 = 0x1f80_1500;
const DMA7_BCR: u32 = 0x1f80_1504;
const DMA7_CHCR: u32 = 0x1f80_1508;
const DMA11_MADR: u32 = 0x1f80_1540;
const DMA11_BCR: u32 = 0x1f80_1544;
const DMA11_CHCR: u32 = 0x1f80_1548;
const DMA12_MADR: u32 = 0x1f80_1550;
const DMA12_BCR: u32 = 0x1f80_1554;
const DMA12_CHCR: u32 = 0x1f80_1558;
const DMA_ICR: u32 = 0x1f80_10f4;
const DMA_ICR2: u32 = 0x1f80_1574;
const DMA9_MADR: u32 = 0x1f80_1520;
const DMA9_BCR: u32 = 0x1f80_1524;
const DMA9_CHCR: u32 = 0x1f80_1528;
const DMA9_TADR: u32 = 0x1f80_152c;
const DMA10_MADR: u32 = 0x1f80_1530;
const DMA10_BCR: u32 = 0x1f80_1534;
const DMA10_CHCR: u32 = 0x1f80_1538;
const DMA10_TADR: u32 = 0x1f80_153c;
const IOP_I_STAT: u32 = 0x1f80_1070;
const IOP_I_MASK: u32 = 0x1f80_1074;
const COUNTER_MODE_WRITE_MASK: u32 = 0x63ff;
const COUNTER_FLAG_MASK: u32 = (1 << 11) | (1 << 12);
const COUNTER_TARGET_IRQ: u32 = 1 << 4;
const COUNTER_OVERFLOW_IRQ: u32 = 1 << 5;
const COUNTER_REPEAT_IRQ: u32 = 1 << 6;
const COUNTER_TOGGLE_IRQ: u32 = 1 << 7;
const COUNTER_EXTERNAL_CLOCK: u32 = 1 << 8;
const COUNTER_TIMER2_PRESCALE: u32 = 1 << 9;
const COUNTER_IRQ_ENABLED: u32 = 1 << 10;
const COUNTER_ZERO_RETURN: u32 = 1 << 3;
pub const IOP_CLOCK_HZ: u64 = 33_868_800;
const CDVD_INITIAL_SEEK_CYCLES: u64 = IOP_CLOCK_HZ / 10;
const SIF0_CHAIN_MAX_TAGS: usize = 4096;
const SIF0_CHAIN_MAX_WORDS: usize = 1 << 20;

const COUNTER_REGISTERS: [(u32, u32, u32); 6] = [
    (0x1f80_1100, 0x1f80_1104, 0x1f80_1108),
    (0x1f80_1110, 0x1f80_1114, 0x1f80_1118),
    (0x1f80_1120, 0x1f80_1124, 0x1f80_1128),
    (0x1f80_1480, 0x1f80_1484, 0x1f80_1488),
    (0x1f80_1490, 0x1f80_1494, 0x1f80_1498),
    (0x1f80_14a0, 0x1f80_14a4, 0x1f80_14a8),
];

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IopMemoryError {
    Unmapped { address: u32, width: usize },
    Misaligned { address: u32, alignment: usize },
    OutOfBounds { address: u32, length: usize },
    ReadOnly { address: u32 },
    Cdvd(String),
}

impl fmt::Display for IopMemoryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unmapped { address, width } => write!(f, "unmapped IOP address {address:#010x} ({width} bytes)"),
            Self::Misaligned { address, alignment } => write!(f, "unaligned IOP address {address:#010x}, alignment {alignment}"),
            Self::OutOfBounds { address, length } => write!(f, "IOP range {address:#010x}+{length:#x} exceeds mapped memory"),
            Self::ReadOnly { address } => write!(f, "write to read-only IOP BIOS ROM at {address:#010x}"),
            Self::Cdvd(message) => write!(f, "CD/DVD device: {message}"),
        }
    }
}

impl std::error::Error for IopMemoryError {}

#[derive(Clone)]
pub struct IopMemory {
    ram: Vec<u8>,
    scratchpad: Vec<u8>,
    bios_rom: Vec<u8>,
    cdvd: CdvdDevice,
    dma3_madr: u32,
    dma3_bcr: u32,
    dma3_chcr: u32,
    dma7_madr: u32,
    dma7_bcr: u32,
    dma7_chcr: u32,
    dma7_words_remaining: u32,
    dma11_madr: u32,
    dma11_bcr: u32,
    dma11_chcr: u32,
    dma11_words_remaining: u32,
    dma12_madr: u32,
    dma12_bcr: u32,
    dma12_chcr: u32,
    dma12_words_remaining: u32,
    dma_icr: u32,
    dma_icr2: u32,
    dma9_madr: u32,
    dma9_bcr: u32,
    dma9_chcr: u32,
    dma9_tadr: u32,
    dma9_words_remaining: u32,
    dma9_chain_words: VecDeque<u32>,
    dma9_chain_loaded: bool,
    dma9_chain_final_madr: u32,
    dma9_chain_final_tadr: u32,
    dma10_madr: u32,
    dma10_bcr: u32,
    dma10_chcr: u32,
    dma10_tadr: u32,
    dma10_words_remaining: u32,
    dma10_chain_end: bool,
    i_stat: u32,
    i_mask: u32,
    iop_cycles: u64,
    cdvd_event_deadline: Option<u64>,
    counters: [IopCounter; 6],
    spu2: Spu2,
    sio2: Sio2,
    sif: SifBus,
}

#[derive(Clone, Copy)]
struct IopCounter {
    count: u64,
    mode: u32,
    target: u64,
    prescaler: u32,
    prescale_phase: u32,
    target_armed: bool,
}

impl IopCounter {
    fn new(index: usize) -> Self {
        Self {
            count: 0,
            mode: if index < 6 { COUNTER_IRQ_ENABLED } else { 0 },
            target: if index < 3 { 0x1_0000 } else { 0x1_0000_0000 },
            prescaler: 1,
            prescale_phase: 0,
            target_armed: false,
        }
    }

    fn mask(index: usize) -> u64 {
        if index < 3 { 0xffff } else { 0xffff_ffff }
    }
}

impl Default for IopMemory {
    fn default() -> Self { Self::new() }
}

impl IopMemory {
    pub fn new() -> Self {
        Self::with_sif(SifBus::new())
    }

    pub fn with_sif(sif: SifBus) -> Self {
        Self {
            ram: vec![0; IOP_RAM_SIZE], scratchpad: vec![0; IOP_SCRATCH_SIZE], bios_rom: vec![0xff; BIOS_ROM_SIZE],
            cdvd: CdvdDevice::new(), dma3_madr: 0, dma3_bcr: 0, dma3_chcr: 0,
            dma7_madr: 0, dma7_bcr: 0, dma7_chcr: 0, dma7_words_remaining: 0,
            dma11_madr: 0, dma11_bcr: 0, dma11_chcr: 0, dma11_words_remaining: 0,
            dma12_madr: 0, dma12_bcr: 0, dma12_chcr: 0, dma12_words_remaining: 0,
            dma_icr: 0,
            dma_icr2: 0,
            dma9_madr: 0, dma9_bcr: 0, dma9_chcr: 0, dma9_tadr: 0, dma9_words_remaining: 0,
            dma9_chain_words: VecDeque::new(), dma9_chain_loaded: false,
            dma9_chain_final_madr: 0, dma9_chain_final_tadr: 0,
            dma10_madr: 0, dma10_bcr: 0, dma10_chcr: 0, dma10_tadr: 0, dma10_words_remaining: 0,
            dma10_chain_end: false,
            i_stat: 0, i_mask: 0,
            iop_cycles: 0, cdvd_event_deadline: None,
            counters: std::array::from_fn(IopCounter::new),
            spu2: Spu2::new(),
            sio2: Sio2::new(),
            sif,
        }
    }

    pub fn mount_cdvd(&mut self, image: CdvdImage) { self.cdvd.mount(image); }

    pub fn cdvd_mounted(&self) -> bool { self.cdvd.media.is_some() }

    pub fn iop_cycles(&self) -> u64 { self.iop_cycles }

    /// Drain interleaved stereo PCM samples produced by the experimental SPU2.
    pub fn take_audio_samples(&mut self) -> Vec<i16> { self.spu2.take_audio_samples() }

    /// Set DualShock button bits for a controller port. Zero bits are pressed.
    pub fn set_gamepad_buttons(&mut self, port: usize, buttons: u16) { self.sio2.set_buttons(port, buttons); }

    /// Set right-X, right-Y, left-X, left-Y stick bytes; 0x80 is centered.
    pub fn set_gamepad_sticks(&mut self, port: usize, sticks: [u8; 4]) { self.sio2.set_sticks(port, sticks); }

    fn iop_irq_pending(&self) -> bool { self.i_stat & self.i_mask != 0 }

    fn advance_iop_cycle(&mut self) -> Result<(), IopMemoryError> {
        self.iop_cycles = self.iop_cycles.wrapping_add(1);
        self.tick_counters();
        self.spu2.tick(1);
        self.service_spu2_dma()?;
        self.service_sio2_dma()?;
        self.service_sif_dma()?;
        if self.cdvd_event_deadline.is_some_and(|deadline| self.iop_cycles >= deadline) {
            self.cdvd_event_deadline = None;
            if self.cdvd.read_active && !self.cdvd.data_ready {
                self.cdvd.data_ready = true;
                self.cdvd.intr_status |= 1;
                self.i_stat |= 1 << 2;
                if self.dma3_chcr & (1 << 24) != 0 { self.execute_cdvd_dma()?; }
            }
        }
        Ok(())
    }

    fn tick_counters(&mut self) {
        for index in 0..self.counters.len() {
            let mut pending_irq = false;
            {
                let counter = &mut self.counters[index];
                // External pixel/HBlank clocks and display gates need a GS timing
                // source. Until that exists, these modes remain stopped.
                if counter.mode & (COUNTER_EXTERNAL_CLOCK | 1) != 0 { continue; }
                counter.prescale_phase += 1;
                if counter.prescale_phase < counter.prescaler { continue; }
                counter.prescale_phase = 0;
                counter.count += 1;

                let mask = IopCounter::mask(index);
                let target_hit = counter.target_armed && counter.count >= counter.target;
                let overflow_hit = counter.count > mask;
                if target_hit {
                    counter.mode |= 1 << 11;
                    pending_irq |= Self::update_counter_irq(counter, false);
                    if counter.mode & COUNTER_ZERO_RETURN != 0 {
                        counter.count -= counter.target;
                    } else {
                        counter.target_armed = false;
                    }
                }
                if overflow_hit {
                    counter.mode |= 1 << 12;
                    pending_irq |= Self::update_counter_irq(counter, true);
                    counter.count &= mask;
                    counter.target_armed = counter.target <= mask;
                }
            }
            if pending_irq { self.i_stat |= 1 << [4, 5, 6, 14, 15, 16][index]; }
        }
    }

    fn update_counter_irq(counter: &mut IopCounter, overflow: bool) -> bool {
        let irq_select = if overflow { COUNTER_OVERFLOW_IRQ } else { COUNTER_TARGET_IRQ };
        if counter.mode & irq_select == 0 || counter.mode & COUNTER_IRQ_ENABLED == 0 { return false; }
        if counter.mode & COUNTER_TOGGLE_IRQ != 0 {
            counter.mode ^= COUNTER_IRQ_ENABLED;
        } else if counter.mode & COUNTER_REPEAT_IRQ == 0 {
            counter.mode &= !COUNTER_IRQ_ENABLED;
        }
        true
    }

    fn counter_register(address: u32) -> Option<(usize, u8)> {
        COUNTER_REGISTERS.iter().enumerate().find_map(|(index, regs)| {
            [regs.0, regs.1, regs.2].iter().position(|&register| register == address).map(|kind| (index, kind as u8))
        })
    }

    fn read_counter_register(&self, address: u32) -> Option<u32> {
        let (index, register) = Self::counter_register(address)?;
        let counter = self.counters[index];
        Some(match register {
            0 => counter.count as u32,
            1 => counter.mode,
            _ => counter.target as u32,
        })
    }

    fn write_counter_register(&mut self, address: u32, value: u32) -> bool {
        let Some((index, register)) = Self::counter_register(address) else { return false; };
        let counter = &mut self.counters[index];
        match register {
            0 => counter.count = u64::from(value) & IopCounter::mask(index),
            1 => {
                let flags = counter.mode & COUNTER_FLAG_MASK & !(value & COUNTER_FLAG_MASK);
                counter.mode = (value & COUNTER_MODE_WRITE_MASK) | flags;
                counter.mode |= COUNTER_IRQ_ENABLED;
                counter.count = 0;
                counter.prescale_phase = 0;
                counter.target_armed = counter.target <= IopCounter::mask(index);
                counter.prescaler = match index {
                    2 if value & COUNTER_TIMER2_PRESCALE != 0 => 8,
                    4 | 5 => match (value >> 13) & 3 { 1 => 8, 2 => 16, 3 => 256, _ => 1 },
                    _ => 1,
                };
            }
            _ => {
                counter.target = u64::from(value) & IopCounter::mask(index);
                counter.target_armed = counter.count < counter.target;
            }
        }
        true
    }

    fn write_counter_part(&mut self, address: u32, width: usize, value: u32) -> bool {
        let register_address = address & !3;
        let Some((index, register)) = Self::counter_register(register_address) else { return false; };
        let shift = (address & 3) * 8;
        let access_mask = match width { 1 => 0xff, 2 => 0xffff, _ => u32::MAX };
        let shifted_mask = access_mask.checked_shl(shift).unwrap_or(0);
        let shifted_value = value.wrapping_shl(shift) & shifted_mask;
        let counter = &mut self.counters[index];
        match register {
            0 => {
                let merged = ((counter.count as u32) & !shifted_mask) | shifted_value;
                counter.count = u64::from(merged) & IopCounter::mask(index);
            }
            1 if shift < 16 => {
                let old_mode = counter.mode;
                let control_mask = shifted_mask & COUNTER_MODE_WRITE_MASK;
                let flags = old_mode & COUNTER_FLAG_MASK & !(shifted_value & COUNTER_FLAG_MASK);
                counter.mode = ((old_mode & !control_mask) | (shifted_value & control_mask)) & COUNTER_MODE_WRITE_MASK;
                counter.mode |= flags | COUNTER_IRQ_ENABLED;
                counter.count = 0;
                counter.prescale_phase = 0;
                counter.target_armed = counter.target <= IopCounter::mask(index);
                counter.prescaler = match index {
                    2 if counter.mode & COUNTER_TIMER2_PRESCALE != 0 => 8,
                    4 | 5 => match (counter.mode >> 13) & 3 { 1 => 8, 2 => 16, 3 => 256, _ => 1 },
                    _ => 1,
                };
            }
            1 => {}
            _ => {
                let merged = ((counter.target as u32) & !shifted_mask) | shifted_value;
                counter.target = u64::from(merged) & IopCounter::mask(index);
                counter.target_armed = counter.count < counter.target;
            }
        }
        true
    }

    fn schedule_next_cdvd_sector(&mut self) {
        let cycles_per_sector = if self.cdvd.disc_type() == 0x12 {
            IOP_CLOCK_HZ / 75
        } else {
            IOP_CLOCK_HZ / 675
        };
        self.cdvd_event_deadline = Some(self.iop_cycles.wrapping_add(cycles_per_sector));
    }

    pub fn load_bios_rom(&mut self, image: &[u8]) -> Result<(), IopMemoryError> {
        if image.is_empty() || image.len() > BIOS_ROM_SIZE || BIOS_ROM_SIZE % image.len() != 0 {
            return Err(IopMemoryError::OutOfBounds { address: BIOS_ROM_BASE, length: image.len() });
        }
        for (index, byte) in self.bios_rom.iter_mut().enumerate() {
            *byte = image[index % image.len()];
        }
        Ok(())
    }

    fn physical(address: u32) -> u32 {
        if (0x8000_0000..0xc000_0000).contains(&address) { address & 0x1fff_ffff } else { address }
    }

    fn is_spu2_register(physical: u32) -> bool {
        (SPU2_BASE..SPU2_BASE + SPU2_REGISTER_SIZE).contains(&physical)
    }

    fn is_sio2_register(physical: u32) -> bool {
        (SIO2_BASE..SIO2_BASE + SIO2_REGISTER_SIZE).contains(&physical)
    }

    fn range(&self, address: u32, length: usize) -> Result<(bool, usize), IopMemoryError> {
        let physical = Self::physical(address);
        if (IOP_SCRATCH_BASE..IOP_SCRATCH_BASE + IOP_SCRATCH_SIZE as u32).contains(&physical) {
            let offset = (physical - IOP_SCRATCH_BASE) as usize;
            if offset.checked_add(length).map_or(true, |end| end > self.scratchpad.len()) {
                return Err(IopMemoryError::OutOfBounds { address, length });
            }
            return Ok((true, offset));
        }
        // The IOP's 2 MiB RAM is mirrored in its low address window and at
        // the EE-visible IOP RAM aperture.
        if physical < 0x0200_0000 || (0x1c00_0000..0x1c20_0000).contains(&physical) {
            let offset = if physical >= 0x1c00_0000 {
                ((physical - 0x1c00_0000) as usize) & (IOP_RAM_SIZE - 1)
            } else {
                (physical as usize) & (IOP_RAM_SIZE - 1)
            };
            if offset.checked_add(length).map_or(true, |end| end > self.ram.len()) {
                return Err(IopMemoryError::OutOfBounds { address, length });
            }
            return Ok((false, offset));
        }
        Err(IopMemoryError::Unmapped { address, width: length })
    }

    pub fn read_bytes(&self, address: u32, length: usize) -> Result<&[u8], IopMemoryError> {
        let physical = Self::physical(address);
        if (BIOS_ROM_BASE..BIOS_ROM_BASE + BIOS_ROM_SIZE as u32).contains(&physical) {
            let offset = (physical - BIOS_ROM_BASE) as usize;
            if offset.checked_add(length).map_or(true, |end| end > self.bios_rom.len()) {
                return Err(IopMemoryError::OutOfBounds { address, length });
            }
            return Ok(&self.bios_rom[offset..offset + length]);
        }
        let (scratch, offset) = self.range(address, length)?;
        let region = if scratch { &self.scratchpad } else { &self.ram };
        Ok(&region[offset..offset + length])
    }

    pub fn write_bytes(&mut self, address: u32, bytes: &[u8]) -> Result<(), IopMemoryError> {
        let physical = Self::physical(address);
        if (BIOS_ROM_BASE..BIOS_ROM_BASE + BIOS_ROM_SIZE as u32).contains(&physical) {
            return Err(IopMemoryError::ReadOnly { address });
        }
        let (scratch, offset) = self.range(address, bytes.len())?;
        let region = if scratch { &mut self.scratchpad } else { &mut self.ram };
        region[offset..offset + bytes.len()].copy_from_slice(bytes);
        Ok(())
    }

    pub fn read_u8(&mut self, address: u32) -> Result<u8, IopMemoryError> {
        let physical = Self::physical(address);
        if Self::is_spu2_register(physical) {
            return Ok(self.spu2.read_u8(physical - SPU2_BASE));
        }
        if Self::is_sio2_register(physical) { return Ok(self.sio2.read_u8(physical - SIO2_BASE)); }
        if let Some(value) = self.sif.read_iop(physical & !3) {
            return Ok((value >> ((physical & 3) * 8)) as u8);
        }
        if (CDVD_BASE..CDVD_BASE + 0x100).contains(&physical) {
            return Ok(self.cdvd.read((physical - CDVD_BASE) as u8));
        }
        let register = physical & !3;
        if let Some(value) = self.read_counter_register(register) {
            return Ok((value >> ((physical & 3) * 8)) as u8);
        }
        Ok(self.read_bytes(address, 1)?[0])
    }
    pub fn read_u16(&mut self, address: u32) -> Result<u16, IopMemoryError> {
        if address & 1 != 0 { return Err(IopMemoryError::Misaligned { address, alignment: 2 }); }
        let physical = Self::physical(address);
        if Self::is_spu2_register(physical) {
            return Ok(self.spu2.read_u16(physical - SPU2_BASE));
        }
        if Self::is_sio2_register(physical) { return Ok(self.sio2.read_u16(physical - SIO2_BASE)); }
        if let Some(value) = self.sif.read_iop(physical & !3) {
            return Ok((value >> ((physical & 2) * 8)) as u16);
        }
        if (CDVD_BASE..CDVD_BASE + 0x100).contains(&physical) {
            return Ok(u16::from_le_bytes([self.read_u8(address)?, self.read_u8(address + 1)?]));
        }
        let register = physical & !3;
        if let Some(value) = self.read_counter_register(register) {
            return Ok((value >> ((physical & 2) * 8)) as u16);
        }
        Ok(u16::from_le_bytes(self.read_bytes(address, 2)?.try_into().unwrap()))
    }
    pub fn read_u32(&mut self, address: u32) -> Result<u32, IopMemoryError> {
        if address & 3 != 0 { return Err(IopMemoryError::Misaligned { address, alignment: 4 }); }
        let physical = Self::physical(address);
        if Self::is_spu2_register(physical) {
            return Ok(self.spu2.read_u32(physical - SPU2_BASE));
        }
        if Self::is_sio2_register(physical) { return Ok(self.sio2.read_u32(physical - SIO2_BASE)); }
        if let Some(value) = self.sif.read_iop(physical) { return Ok(value); }
        if (CDVD_BASE..CDVD_BASE + 0x100).contains(&physical) {
            return Ok(u32::from_le_bytes([
                self.read_u8(address)?, self.read_u8(address + 1)?,
                self.read_u8(address + 2)?, self.read_u8(address + 3)?,
            ]));
        }
        if matches!(physical, DMA3_MADR | DMA3_BCR | DMA3_CHCR | DMA7_MADR | DMA7_BCR | DMA7_CHCR | DMA11_MADR | DMA11_BCR | DMA11_CHCR | DMA12_MADR | DMA12_BCR | DMA12_CHCR | DMA_ICR | DMA_ICR2 | DMA9_MADR | DMA9_BCR | DMA9_CHCR | DMA9_TADR | DMA10_MADR | DMA10_BCR | DMA10_CHCR | DMA10_TADR | IOP_I_STAT | IOP_I_MASK) {
            return Ok(match physical {
                DMA3_MADR => self.dma3_madr,
                DMA3_BCR => self.dma3_bcr,
                DMA3_CHCR => self.dma3_chcr,
                DMA7_MADR => self.dma7_madr,
                DMA7_BCR => self.dma7_bcr,
                DMA7_CHCR => self.dma7_chcr,
                DMA11_MADR => self.dma11_madr,
                DMA11_BCR => self.dma11_bcr,
                DMA11_CHCR => self.dma11_chcr,
                DMA12_MADR => self.dma12_madr,
                DMA12_BCR => self.dma12_bcr,
                DMA12_CHCR => self.dma12_chcr,
                DMA_ICR => self.dma_icr,
                DMA_ICR2 => self.dma_icr2,
                DMA9_MADR => self.dma9_madr,
                DMA9_BCR => self.dma9_bcr,
                DMA9_CHCR => self.dma9_chcr,
                DMA9_TADR => self.dma9_tadr,
                DMA10_MADR => self.dma10_madr,
                DMA10_BCR => self.dma10_bcr,
                DMA10_CHCR => self.dma10_chcr,
                DMA10_TADR => self.dma10_tadr,
                IOP_I_STAT => self.i_stat,
                _ => self.i_mask,
            });
        }
        if let Some(value) = self.read_counter_register(physical) { return Ok(value); }
        Ok(u32::from_le_bytes(self.read_bytes(address, 4)?.try_into().unwrap()))
    }
    pub fn write_u8(&mut self, address: u32, value: u8) -> Result<(), IopMemoryError> {
        let physical = Self::physical(address);
        if Self::is_spu2_register(physical) {
            self.spu2.write_u8(physical - SPU2_BASE, value);
            return Ok(());
        }
        if Self::is_sio2_register(physical) { self.sio2.write_u8(physical - SIO2_BASE, value); return Ok(()); }
        if (CDVD_BASE..CDVD_BASE + 0x100).contains(&physical) {
            let previous_status = self.cdvd.intr_status;
            let register = (physical - CDVD_BASE) as u8;
            self.cdvd.write(register, value)?;
            if self.cdvd.intr_status != previous_status { self.i_stat |= 1 << 2; }
            if register == 0x04 {
                if self.cdvd.read_active {
                    self.cdvd.data_ready = false;
                    self.cdvd_event_deadline = Some(self.iop_cycles.wrapping_add(CDVD_INITIAL_SEEK_CYCLES));
                } else {
                    self.cdvd_event_deadline = None;
                }
            }
            return Ok(());
        }
        if crate::sif::write_partial(&self.sif, physical, 1, u32::from(value), true) { return Ok(()); }
        if self.write_counter_part(physical, 1, u32::from(value)) { return Ok(()); }
        self.write_bytes(address, &[value])
    }
    pub fn write_u16(&mut self, address: u32, value: u16) -> Result<(), IopMemoryError> {
        if address & 1 != 0 { return Err(IopMemoryError::Misaligned { address, alignment: 2 }); }
        let physical = Self::physical(address);
        if Self::is_spu2_register(physical) {
            self.spu2.write_u16(physical - SPU2_BASE, value);
            return Ok(());
        }
        if Self::is_sio2_register(physical) { self.sio2.write_u16(physical - SIO2_BASE, value); return Ok(()); }
        if (CDVD_BASE..CDVD_BASE + 0x100).contains(&physical) {
            let bytes = value.to_le_bytes();
            self.write_u8(address, bytes[0])?;
            return self.write_u8(address + 1, bytes[1]);
        }
        if crate::sif::write_partial(&self.sif, physical, 2, u32::from(value), true) { return Ok(()); }
        if self.write_counter_part(physical, 2, u32::from(value)) { return Ok(()); }
        self.write_bytes(address, &value.to_le_bytes())
    }
    pub fn write_u32(&mut self, address: u32, value: u32) -> Result<(), IopMemoryError> {
        if address & 3 != 0 { return Err(IopMemoryError::Misaligned { address, alignment: 4 }); }
        let physical = Self::physical(address);
        if Self::is_spu2_register(physical) {
            self.spu2.write_u32(physical - SPU2_BASE, value);
            return Ok(());
        }
        if Self::is_sio2_register(physical) {
            self.sio2.write_u32(physical - SIO2_BASE, value);
            if physical == SIO2_BASE + 0x68 && value & 1 != 0 { self.i_stat |= 1 << 17; }
            return Ok(());
        }
        if self.sif.write_iop(physical, value) { return Ok(()); }
        if self.write_counter_register(physical, value) { return Ok(()); }
        match physical {
            IOP_I_STAT => self.i_stat &= !value,
            IOP_I_MASK => self.i_mask ^= value,
            DMA_ICR => {
                self.dma_icr &= !(value & 0x7f00_0000);
                self.dma_icr = (self.dma_icr & !0x00bf_0000) | (value & 0x00bf_0000);
                self.update_dma_master_flag();
            }
            DMA_ICR2 => {
                self.dma_icr2 &= !(value & 0x7f00_0000);
                self.dma_icr2 = (self.dma_icr2 & !0x00bf_0000) | (value & 0x00bf_0000);
                self.update_dma_icr2_master_flag();
            }
            DMA3_MADR => self.dma3_madr = value & 0x00ff_ffff,
            DMA3_BCR => self.dma3_bcr = value,
            DMA3_CHCR => {
                self.dma3_chcr = value;
                if value & (1 << 24) != 0 { self.execute_cdvd_dma()?; }
            }
            DMA7_MADR => self.dma7_madr = value & 0x00ff_fffc,
            DMA7_BCR => self.dma7_bcr = value,
            DMA7_CHCR => {
                self.dma7_chcr = value;
                self.dma7_words_remaining = if value & (1 << 24) != 0 && ((value >> 9) & 3) == 0 {
                    match self.dma7_bcr & 0xffff { 0 => 0x1_0000, count => count }
                } else { 0 };
            }
            DMA11_MADR => self.dma11_madr = value & 0x00ff_fffc,
            DMA11_BCR => self.dma11_bcr = value,
            DMA11_CHCR => {
                self.dma11_chcr = value;
                self.dma11_words_remaining = if value & (1 << 24) != 0 && value & 1 != 0 && ((value >> 9) & 3) == 0 {
                    match self.dma11_bcr & 0xffff { 0 => 0x1_0000, count => count }
                } else { 0 };
            }
            DMA12_MADR => self.dma12_madr = value & 0x00ff_fffc,
            DMA12_BCR => self.dma12_bcr = value,
            DMA12_CHCR => {
                self.dma12_chcr = value;
                self.dma12_words_remaining = if value & (1 << 24) != 0 && value & 1 == 0 && ((value >> 9) & 3) == 0 {
                    match self.dma12_bcr & 0xffff { 0 => 0x1_0000, count => count }
                } else { 0 };
            }
            DMA9_MADR => self.dma9_madr = value & 0x00ff_fffc,
            DMA9_BCR => self.dma9_bcr = value,
            DMA9_CHCR => {
                self.dma9_chcr = value;
                self.dma9_chain_words.clear();
                self.dma9_chain_loaded = false;
                self.dma9_words_remaining = if value & (1 << 24) != 0 && ((value >> 9) & 3) == 0 {
                    match self.dma9_bcr & 0xffff { 0 => 0x1_0000, count => count }
                } else { 0 };
            }
            DMA9_TADR => self.dma9_tadr = value & 0x00ff_fffc,
            DMA10_MADR => self.dma10_madr = value & 0x00ff_fffc,
            DMA10_BCR => self.dma10_bcr = value,
            DMA10_CHCR => {
                self.dma10_chcr = value;
                self.dma10_chain_end = false;
                self.dma10_words_remaining = if value & (1 << 24) != 0 && ((value >> 9) & 3) == 0 {
                    match self.dma10_bcr & 0xffff { 0 => 0x1_0000, count => count }
                } else { 0 };
            }
            DMA10_TADR => self.dma10_tadr = value & 0x00ff_fffc,
            _ if (CDVD_BASE..CDVD_BASE + 0x100).contains(&physical) => {
                for (index, byte) in value.to_le_bytes().into_iter().enumerate() {
                    self.write_u8(address + index as u32, byte)?;
                }
            }
            _ => return self.write_bytes(address, &value.to_le_bytes()),
        }
        Ok(())
    }

    fn execute_cdvd_dma(&mut self) -> Result<(), IopMemoryError> {
        let word_count = (self.dma3_bcr & 0xffff).saturating_mul(self.dma3_bcr >> 16);
        let transfer_bytes = word_count.saturating_mul(4) as usize;
        let sector_capacity = transfer_bytes / ISO_SECTOR_SIZE;
        if sector_capacity == 0 {
            self.dma3_chcr &= !(1 << 24);
            return Ok(());
        }
        if !self.cdvd.read_active || !self.cdvd.data_ready { return Ok(()); }
        let sectors = 1.min(sector_capacity).min(self.cdvd.sectors_remaining as usize);
        let mut destination = self.dma3_madr;
        for _ in 0..sectors {
            let sector_result = self.cdvd.media.as_ref()
                .ok_or_else(|| IopMemoryError::Cdvd(String::from("disc removed during transfer")))?
                .read_sector(self.cdvd.current_lba);
            let sector = match sector_result {
                Ok(sector) => sector,
                Err(_) => {
                    self.cdvd.error = 0x30;
                    self.cdvd.ready = 0x41;
                    self.cdvd.intr_status |= 1;
                    self.i_stat |= 1 << 2;
                    self.cdvd.read_active = false;
                    self.dma3_chcr &= !(1 << 24);
                    return Ok(());
                }
            };
            self.write_bytes(destination, &sector)?;
            destination = destination.wrapping_add(ISO_SECTOR_SIZE as u32);
            self.cdvd.current_lba = self.cdvd.current_lba.wrapping_add(1);
            self.cdvd.sectors_remaining -= 1;
        }
        self.dma3_madr = destination & 0x00ff_ffff;
        self.dma3_chcr &= !(1 << 24);
        self.cdvd.intr_status |= 1;
        self.i_stat |= 1 << 2;
        if self.dma_icr & (1 << 19) != 0 {
            self.dma_icr |= 1 << 27;
            if self.dma_icr & (1 << 23) != 0 { self.dma_icr |= 1 << 31; }
            self.i_stat |= 1 << 3;
        }
        if self.cdvd.sectors_remaining == 0 {
            self.cdvd.read_active = false;
            self.cdvd.data_ready = false;
            self.cdvd.ready = 0x40;
            self.cdvd.status = 0x0a;
            self.cdvd_event_deadline = None;
        } else {
            self.cdvd.data_ready = false;
            self.schedule_next_cdvd_sector();
        }
        Ok(())
    }

    fn update_dma_master_flag(&mut self) {
        let channel_enable = (self.dma_icr >> 16) & 0x3f;
        let channel_flags = (self.dma_icr >> 24) & 0x3f;
        let master = self.dma_icr & (1 << 23) != 0 && channel_enable & channel_flags != 0;
        if master { self.dma_icr |= 1 << 31; } else { self.dma_icr &= !(1 << 31); }
    }

    fn service_sif_dma(&mut self) -> Result<(), IopMemoryError> {
        use crate::sif::SifChannel;

        if self.dma9_chcr & (1 << 24) != 0 && ((self.dma9_chcr >> 9) & 3) == 3 {
            if !self.dma9_chain_loaded {
                let (words, final_madr, final_tadr) = self.collect_sif0_source_chain()?;
                self.dma9_chain_words = words.into();
                self.dma9_chain_final_madr = final_madr;
                self.dma9_chain_final_tadr = final_tadr;
                self.dma9_chain_loaded = true;
            }
            if self.sif.fifo_free(SifChannel::Sif0) != 0 {
                if let Some(word) = self.dma9_chain_words.pop_front() {
                    if self.sif.write_fifo(SifChannel::Sif0, &[word]) == 1 {
                        if self.dma9_chain_words.is_empty() {
                            self.dma9_madr = self.dma9_chain_final_madr;
                            self.dma9_tadr = self.dma9_chain_final_tadr;
                            self.dma9_chcr &= !(1 << 24);
                            self.dma9_chain_loaded = false;
                            self.complete_sif_dma(2);
                        }
                    }
                }
            }
        }

        // IOP channel 9 pushes one word into SIF0 per IOP cycle. If the EE
        // side has not drained the FIFO yet, the active transfer waits here.
        if self.dma9_words_remaining != 0 && self.dma9_chcr & (1 << 24) != 0 {
            if self.sif.fifo_free(SifChannel::Sif0) != 0 {
                let bytes = self.read_bytes(self.dma9_madr, 4)?;
                let word = u32::from_le_bytes(bytes.try_into().unwrap());
                if self.sif.write_fifo(SifChannel::Sif0, &[word]) == 1 {
                    Self::advance_sif_dma_address(&mut self.dma9_madr, self.dma9_chcr);
                    self.dma9_words_remaining -= 1;
                    self.dma9_bcr = (self.dma9_bcr & 0xffff_0000) | (self.dma9_words_remaining as u16 as u32);
                    if self.dma9_words_remaining == 0 {
                        self.dma9_chcr &= !(1 << 24);
                        self.complete_sif_dma(2);
                    }
                }
            }
        }

        // IOP channel 10 pops one word from SIF1 into IOP RAM. An empty FIFO
        // leaves CHCR active, so a later EE transfer can resume it.
        if self.dma10_words_remaining != 0 && self.dma10_chcr & (1 << 24) != 0 {
            self.range(self.dma10_madr, 4)?;
            let mut word = [0u32; 1];
            if self.sif.read_fifo(SifChannel::Sif1, &mut word) == 1 {
                self.write_bytes(self.dma10_madr, &word[0].to_le_bytes())?;
                Self::advance_sif_dma_address(&mut self.dma10_madr, self.dma10_chcr);
                self.dma10_words_remaining -= 1;
                self.dma10_bcr = (self.dma10_bcr & 0xffff_0000) | (self.dma10_words_remaining as u16 as u32);
                if self.dma10_words_remaining == 0 {
                    self.dma10_chcr &= !(1 << 24);
                    self.complete_sif_dma(3);
                }
            }
        } else if self.dma10_chcr & (1 << 24) != 0 && ((self.dma10_chcr >> 9) & 3) == 3 {
            if self.dma10_words_remaining == 0 {
                if self.dma10_chain_end {
                    self.dma10_chcr &= !(1 << 24);
                    self.dma10_chain_end = false;
                    self.complete_sif_dma(3);
                } else if self.sif.fifo_len(SifChannel::Sif1) >= 4 {
                    let mut tag = [0u32; 4];
                    if self.sif.read_fifo(SifChannel::Sif1, &mut tag) == 4 {
                        self.dma10_madr = tag[0] & 0x00ff_fffc;
                        self.dma10_words_remaining = tag[1] & 0x000f_fffc;
                        self.dma10_chain_end = tag[0] & ((1 << 30) | (1 << 31)) != 0;
                    }
                }
            }
            if self.dma10_words_remaining != 0 && self.sif.fifo_len(SifChannel::Sif1) != 0 {
                self.range(self.dma10_madr, 4)?;
                let mut word = [0u32; 1];
                if self.sif.read_fifo(SifChannel::Sif1, &mut word) == 1 {
                    self.write_bytes(self.dma10_madr, &word[0].to_le_bytes())?;
                    Self::advance_sif_dma_address(&mut self.dma10_madr, self.dma10_chcr);
                    self.dma10_words_remaining -= 1;
                    self.dma10_bcr = (self.dma10_bcr & 0xffff_0000) | (self.dma10_words_remaining as u16 as u32);
                }
            }
        }
        Ok(())
    }

    fn service_spu2_dma(&mut self) -> Result<(), IopMemoryError> {
        if self.dma7_words_remaining == 0 || self.dma7_chcr & (1 << 24) == 0 {
            return Ok(());
        }
        let address = self.dma7_madr & 0x00ff_fffc;
        if self.dma7_chcr & 1 != 0 {
            let bytes = self.read_bytes(address, 4)?;
            let halves = [u16::from_le_bytes([bytes[0], bytes[1]]), u16::from_le_bytes([bytes[2], bytes[3]])];
            self.spu2.write_u16(0x1ac, halves[0]);
            self.spu2.write_u16(0x1ac, halves[1]);
        } else {
            let low = self.spu2.read_u16(0x1ac).to_le_bytes();
            let high = self.spu2.read_u16(0x1ac).to_le_bytes();
            self.write_bytes(address, &[low[0], low[1], high[0], high[1]])?;
        }
        let delta = if self.dma7_chcr & 2 != 0 { 0xffff_fffc } else { 4 };
        self.dma7_madr = self.dma7_madr.wrapping_add(delta) & 0x00ff_fffc;
        self.dma7_words_remaining -= 1;
        if self.dma7_words_remaining == 0 {
            self.dma7_chcr &= !(1 << 24);
            self.dma_icr2 |= 1 << 24;
            self.update_dma_icr2_master_flag();
            if self.dma_icr2 & (1 << 31) != 0 { self.i_stat |= 1 << 3; }
        }
        Ok(())
    }

    fn service_sio2_dma(&mut self) -> Result<(), IopMemoryError> {
        if self.dma11_words_remaining != 0 && self.dma11_chcr & (1 << 24) != 0 {
            let bytes = self.read_bytes(self.dma11_madr, 4)?;
            let word = u32::from_le_bytes(bytes.try_into().unwrap());
            self.sio2.dma_push_word(word);
            let delta = if self.dma11_chcr & 2 != 0 { 0xffff_fffc } else { 4 };
            self.dma11_madr = self.dma11_madr.wrapping_add(delta) & 0x00ff_fffc;
            self.dma11_words_remaining -= 1;
            self.dma11_bcr = (self.dma11_bcr & 0xffff_0000) | self.dma11_words_remaining as u16 as u32;
            if self.dma11_words_remaining == 0 {
                self.dma11_chcr &= !(1 << 24);
                self.dma_icr2 |= 1 << 28;
                self.update_dma_icr2_master_flag();
                if self.dma_icr2 & (1 << 31) != 0 { self.i_stat |= 1 << 3; }
            }
        }

        if self.dma12_words_remaining != 0 && self.dma12_chcr & (1 << 24) != 0 {
            if let Some(word) = self.sio2.dma_pop_word() {
                self.write_bytes(self.dma12_madr, &word.to_le_bytes())?;
                let delta = if self.dma12_chcr & 2 != 0 { 0xffff_fffc } else { 4 };
                self.dma12_madr = self.dma12_madr.wrapping_add(delta) & 0x00ff_fffc;
                self.dma12_words_remaining -= 1;
                self.dma12_bcr = (self.dma12_bcr & 0xffff_0000) | self.dma12_words_remaining as u16 as u32;
                if self.dma12_words_remaining == 0 {
                    self.dma12_chcr &= !(1 << 24);
                    self.dma_icr2 |= 1 << 29;
                    self.update_dma_icr2_master_flag();
                    if self.dma_icr2 & (1 << 31) != 0 { self.i_stat |= 1 << 3; }
                }
            }
        }
        Ok(())
    }

    fn collect_sif0_source_chain(&self) -> Result<(Vec<u32>, u32, u32), IopMemoryError> {
        let transfer_tag_payload = self.dma9_chcr & (1 << 8) != 0;
        let mut output = Vec::new();
        let mut tadr = self.dma9_tadr & 0x00ff_fffc;

        for _ in 0..SIF0_CHAIN_MAX_TAGS {
            let tag_bytes = self.read_bytes(tadr, 8)?;
            let address_word = u32::from_le_bytes(tag_bytes[0..4].try_into().unwrap());
            let control_word = u32::from_le_bytes(tag_bytes[4..8].try_into().unwrap());
            let address = address_word & 0x00ff_fffc;
            let word_count = control_word & 0x00ff_ffff;
            let irq = address_word & (1 << 30) != 0;
            let end = address_word & (1 << 31) != 0;
            if word_count % 4 != 0 || word_count as usize > SIF0_CHAIN_MAX_WORDS {
                return Err(IopMemoryError::OutOfBounds { address: tadr, length: word_count as usize * 4 });
            }
            let qwc = word_count / 4;
            if qwc > 0xffff {
                return Err(IopMemoryError::OutOfBounds { address: tadr, length: word_count as usize * 4 });
            }
            let ee_id = if end { 7u32 } else { 0u32 };
            let ee_tag_low = (u64::from(address & 0x00ff_fff0) << 32) | u64::from(qwc | (ee_id << 28) | (u32::from(irq) << 31));
            if output.len().saturating_add(4 + word_count as usize) > SIF0_CHAIN_MAX_WORDS {
                return Err(IopMemoryError::OutOfBounds { address: tadr, length: output.len().saturating_add(word_count as usize * 4) });
            }
            output.try_reserve(4 + word_count as usize).map_err(|_| IopMemoryError::OutOfBounds { address: tadr, length: word_count as usize * 4 })?;
            output.push(ee_tag_low as u32);
            output.push((ee_tag_low >> 32) as u32);
            if transfer_tag_payload {
                let payload = self.read_bytes(tadr.wrapping_add(8), 8)?;
                output.push(u32::from_le_bytes(payload[0..4].try_into().unwrap()));
                output.push(u32::from_le_bytes(payload[4..8].try_into().unwrap()));
                tadr = tadr.wrapping_add(16) & 0x00ff_fffc;
            } else {
                output.extend([0, 0]);
                tadr = tadr.wrapping_add(8) & 0x00ff_fffc;
            }
            let data = self.read_bytes(address, word_count as usize * 4)?;
            for chunk in data.chunks_exact(4) {
                output.push(u32::from_le_bytes(chunk.try_into().unwrap()));
            }
            let final_madr = address.wrapping_add(word_count.wrapping_mul(4)) & 0x00ff_ffff;
            if end { return Ok((output, final_madr, tadr)); }
        }
        Err(IopMemoryError::OutOfBounds { address: tadr, length: SIF0_CHAIN_MAX_TAGS })
    }

    fn advance_sif_dma_address(address: &mut u32, chcr: u32) {
        let delta = if chcr & 2 != 0 { 0xffff_fffc } else { 4 };
        *address = address.wrapping_add(delta) & 0x00ff_fffc;
    }

    fn complete_sif_dma(&mut self, icr2_channel: u32) {
        self.dma_icr2 |= 1 << (24 + icr2_channel);
        self.update_dma_icr2_master_flag();
        if self.dma_icr2 & (1 << 31) != 0 {
            self.i_stat |= 1 << 3;
        }
    }

    fn update_dma_icr2_master_flag(&mut self) {
        let channel_enable = (self.dma_icr2 >> 16) & 0x3f;
        let channel_flags = (self.dma_icr2 >> 24) & 0x3f;
        let master = self.dma_icr2 & (1 << 23) != 0 && channel_enable & channel_flags != 0;
        if master { self.dma_icr2 |= 1 << 31; } else { self.dma_icr2 &= !(1 << 31); }
    }
}

#[derive(Clone)]
struct CdvdDevice {
    media: Option<CdvdImage>,
    n_command: u8,
    n_parameters: [u8; 16],
    n_parameter_count: usize,
    ready: u8,
    error: u8,
    intr_status: u8,
    status: u8,
    s_command: u8,
    s_parameters: [u8; 16],
    s_parameter_count: usize,
    s_results: [u8; 16],
    s_result_count: usize,
    s_result_position: usize,
    current_lba: u32,
    sectors_remaining: u32,
    read_active: bool,
    data_ready: bool,
}

impl CdvdDevice {
    fn new() -> Self {
        Self {
            media: None,
            n_command: 0,
            n_parameters: [0; 16],
            n_parameter_count: 0,
            ready: 0,
            error: 0,
            intr_status: 0,
            status: 1,
            s_command: 0,
            s_parameters: [0; 16],
            s_parameter_count: 0,
            s_results: [0; 16],
            s_result_count: 0,
            s_result_position: 0,
            current_lba: 0,
            sectors_remaining: 0,
            read_active: false,
            data_ready: false,
        }
    }

    fn mount(&mut self, image: CdvdImage) {
        self.media = Some(image);
        self.ready = 0x40;
        self.status = 0x0a;
        self.error = 0;
    }

    fn read(&mut self, register: u8) -> u8 {
        match register {
            0x04 => self.n_command,
            0x05 => self.ready,
            0x06 => std::mem::take(&mut self.error),
            0x08 => self.intr_status,
            0x0a => self.status,
            0x0b => self.status & 1,
            0x0f => if self.media.is_some() { self.disc_type() } else { 0 },
            0x16 => self.s_command,
            0x17 => if self.s_result_position >= self.s_result_count { 0x40 } else { 0 },
            0x18 => {
                if self.s_result_position < self.s_result_count {
                    let value = self.s_results[self.s_result_position];
                    self.s_result_position += 1;
                    value
                } else {
                    0
                }
            }
            _ => 0xff,
        }
    }

    fn write(&mut self, register: u8, value: u8) -> Result<(), IopMemoryError> {
        match register {
            0x04 => self.issue_n_command(value),
            0x05 => {
                if self.n_parameter_count < self.n_parameters.len() {
                    self.n_parameters[self.n_parameter_count] = value;
                    self.n_parameter_count += 1;
                }
            }
            0x08 => self.intr_status &= !value,
            0x16 => self.issue_s_command(value),
            0x17 => {
                if self.s_parameter_count < self.s_parameters.len() {
                    self.s_parameters[self.s_parameter_count] = value;
                    self.s_parameter_count += 1;
                }
            }
            _ => {}
        }
        Ok(())
    }

    fn issue_n_command(&mut self, command: u8) {
        self.n_command = command;
        self.error = 0;
        let parameter_count = self.n_parameter_count;
        let result = match command {
            0x00 => Ok(()),
            0x01 => {
                self.status = 0;
                self.read_active = false;
                Ok(())
            }
            0x04 => {
                self.read_active = false;
                self.status = 0x0a;
                Ok(())
            }
            0x06 | 0x08 if parameter_count == 11 => {
                if self.media.is_none() {
                    Err(0x12)
                } else {
                    self.current_lba = u32::from_le_bytes(self.n_parameters[0..4].try_into().unwrap());
                    self.sectors_remaining = u32::from_le_bytes(self.n_parameters[4..8].try_into().unwrap());
                    if self.sectors_remaining == 0 {
                        Err(0x21)
                    } else {
                        self.read_active = true;
                        self.data_ready = false;
                        self.status = 0x06;
                        self.ready = 0x80;
                        Ok(())
                    }
                }
            }
            0x06 | 0x08 => Err(0x22),
            _ => Err(0x10),
        };
        if let Err(error) = result {
            self.error = error;
            self.ready = 0x41;
            self.intr_status |= 1;
        } else if !self.read_active {
            self.ready = 0x40;
            self.intr_status |= 1;
        }
        self.n_parameter_count = 0;
    }

    fn issue_s_command(&mut self, command: u8) {
        self.s_command = command;
        self.s_results = [0; 16];
        self.s_result_position = 0;
        let (count, result) = match command {
            0x01 => (1, if self.media.is_some() { self.disc_type() } else { 0 }),
            0x05 => (1, 0),
            0x08 => {
                self.s_results[..8].copy_from_slice(&[0, 0, 0, 0, 0, 1, 1, 0]);
                (8, 0)
            }
            0x1a => (1, if self.media.is_some() { 1 } else { 0 }),
            _ => (1, 0x81),
        };
        if command != 0x08 { self.s_results[0] = result; }
        self.s_result_count = count;
        self.s_parameter_count = 0;
    }

    fn disc_type(&self) -> u8 {
        let Some(media) = &self.media else { return 0 };
        if media.volume_blocks() > 400_000 { 0x14 } else { 0x12 }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IopFault {
    Memory(IopMemoryError),
    ReservedInstruction { pc: u32, instruction: u32 },
    Syscall { pc: u32, code: u32 },
    Break { pc: u32, code: u32 },
    ArithmeticOverflow { pc: u32, instruction: u32 },
}

impl fmt::Display for IopFault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Memory(error) => write!(f, "{error}"),
            Self::ReservedInstruction { pc, instruction } => write!(f, "unsupported IOP instruction {instruction:#010x} at {pc:#010x}"),
            Self::Syscall { pc, code } => write!(f, "unhandled IOP syscall {code:#x} at {pc:#010x}"),
            Self::Break { pc, code } => write!(f, "IOP break {code:#x} at {pc:#010x}"),
            Self::ArithmeticOverflow { pc, instruction } => write!(f, "IOP integer overflow at {pc:#010x}, instruction {instruction:#010x}"),
        }
    }
}

impl std::error::Error for IopFault {}
impl From<IopMemoryError> for IopFault { fn from(value: IopMemoryError) -> Self { Self::Memory(value) } }

#[derive(Debug, Clone, Copy)]
struct Branch { target: u32, origin: u32 }

#[derive(Debug, Clone)]
pub struct IopCpu {
    pub gpr: [u32; 32],
    pub hi: u32,
    pub lo: u32,
    pub cp0: [u32; 16],
    pub pc: u32,
    pending_branch: Option<Branch>,
    pub retired_instructions: u64,
    pub exceptions_taken: u64,
}

impl IopCpu {
    pub fn new(entry: u32) -> Self {
        Self { gpr: [0; 32], hi: 0, lo: 0, cp0: [0; 16], pc: entry, pending_branch: None, retired_instructions: 0, exceptions_taken: 0 }
    }

    pub fn step(&mut self, memory: &mut IopMemory) -> Result<(), IopFault> {
        memory.advance_iop_cycle()?;
        let pc = self.pc;
        let irq_pending = memory.iop_irq_pending();
        if irq_pending { self.cp0[13] |= 1 << 10; } else { self.cp0[13] &= !(1 << 10); }
        let status = self.cp0[12];
        if irq_pending && status & 1 != 0 && status & (1 << 10) != 0 {
            self.enter_exception(0, pc, None);
            return Ok(());
        }
        if pc & 3 != 0 {
            self.enter_exception(4, pc, Some(pc));
            return Ok(());
        }
        let ins = match memory.read_u32(pc) {
            Ok(ins) => ins,
            Err(IopMemoryError::Misaligned { address, .. }) => {
                self.enter_exception(4, pc, Some(address));
                return Ok(());
            }
            Err(_) => {
                self.enter_exception(6, pc, None);
                return Ok(());
            }
        };
        let op = ins >> 26;
        let rs = ((ins >> 21) & 31) as usize;
        let rt = ((ins >> 16) & 31) as usize;
        let rd = ((ins >> 11) & 31) as usize;
        let sa = (ins >> 6) & 31;
        let imm = ins as u16 as i16 as i32;
        let a = self.gpr[rs];
        let b = self.gpr[rt];
        let address = a.wrapping_add(imm as u32);
        let mut branch = None;
        let execution = (|| -> Result<(), IopFault> {
        match op {
            0 => match ins & 63 {
                0x00 => self.gpr[rd] = b.wrapping_shl(sa),
                0x02 => self.gpr[rd] = b >> sa,
                0x03 => self.gpr[rd] = ((b as i32) >> sa) as u32,
                0x04 => self.gpr[rd] = b.wrapping_shl(a & 31),
                0x06 => self.gpr[rd] = b >> (a & 31),
                0x07 => self.gpr[rd] = ((b as i32) >> (a & 31)) as u32,
                0x08 => branch = Some(Branch { target: a, origin: pc }),
                0x09 => { self.gpr[rd] = pc.wrapping_add(8); branch = Some(Branch { target: a, origin: pc }); }
                0x0c => return Err(IopFault::Syscall { pc, code: (ins >> 6) & 0xfffff }),
                0x0d => return Err(IopFault::Break { pc, code: (ins >> 6) & 0xfffff }),
                0x10 => self.gpr[rd] = self.hi,
                0x11 => self.hi = a,
                0x12 => self.gpr[rd] = self.lo,
                0x13 => self.lo = a,
                0x18 => { let p = (a as i32 as i64) * (b as i32 as i64); self.lo = p as u32; self.hi = (p >> 32) as u32; }
                0x19 => { let p = (a as u64) * (b as u64); self.lo = p as u32; self.hi = (p >> 32) as u32; }
                0x1a => { let (q, r) = if b == 0 { (if (a as i32) < 0 { 1 } else { u32::MAX }, a) } else if a == 0x8000_0000 && b == u32::MAX { (a, 0) } else { ((a as i32 / b as i32) as u32, (a as i32 % b as i32) as u32) }; self.lo = q; self.hi = r; }
                0x1b => { self.lo = if b == 0 { u32::MAX } else { a / b }; self.hi = if b == 0 { a } else { a % b }; }
                0x20 | 0x21 => { let (v, overflow) = (a as i32).overflowing_add(b as i32); if ins & 63 == 0x20 && overflow { return Err(IopFault::ArithmeticOverflow { pc, instruction: ins }); } self.gpr[rd] = v as u32; }
                0x22 | 0x23 => { let (v, overflow) = (a as i32).overflowing_sub(b as i32); if ins & 63 == 0x22 && overflow { return Err(IopFault::ArithmeticOverflow { pc, instruction: ins }); } self.gpr[rd] = v as u32; }
                0x24 => self.gpr[rd] = a & b,
                0x25 => self.gpr[rd] = a | b,
                0x26 => self.gpr[rd] = a ^ b,
                0x27 => self.gpr[rd] = !(a | b),
                0x2a => self.gpr[rd] = ((a as i32) < (b as i32)) as u32,
                0x2b => self.gpr[rd] = (a < b) as u32,
                _ => return Err(IopFault::ReservedInstruction { pc, instruction: ins }),
            },
            0x02 | 0x03 => { if op == 3 { self.gpr[31] = pc.wrapping_add(8); } branch = Some(Branch { target: ((pc.wrapping_add(4)) & 0xf000_0000) | ((ins & 0x03ff_ffff) << 2), origin: pc }); }
            0x04 | 0x05 | 0x06 | 0x07 => {
                let take = match op { 4 => a == b, 5 => a != b, 6 => (a as i32) <= 0, _ => (a as i32) > 0 };
                branch = Some(Branch { target: if take { pc.wrapping_add(4).wrapping_add((imm << 2) as u32) } else { pc.wrapping_add(8) }, origin: pc });
            }
            0x01 => {
                let signed = a as i32;
                let (valid, take, link) = match rt as u32 {
                    0x00 => (true, signed < 0, false),
                    0x01 => (true, signed >= 0, false),
                    0x10 => (true, signed < 0, true),
                    0x11 => (true, signed >= 0, true),
                    _ => (false, false, false),
                };
                if !valid { return Err(IopFault::ReservedInstruction { pc, instruction: ins }); }
                if link { self.gpr[31] = pc.wrapping_add(8); }
                branch = Some(Branch { target: if take { pc.wrapping_add(4).wrapping_add((imm << 2) as u32) } else { pc.wrapping_add(8) }, origin: pc });
            }
            0x08 | 0x09 => { let (v, overflow) = (a as i32).overflowing_add(imm); if op == 8 && overflow { return Err(IopFault::ArithmeticOverflow { pc, instruction: ins }); } self.gpr[rt] = v as u32; }
            0x0a => self.gpr[rt] = ((a as i32) < imm) as u32,
            0x0b => self.gpr[rt] = (a < imm as u32) as u32,
            0x0c => self.gpr[rt] = a & (ins & 0xffff),
            0x0d => self.gpr[rt] = a | (ins & 0xffff),
            0x0e => self.gpr[rt] = a ^ (ins & 0xffff),
            0x0f => self.gpr[rt] = (ins & 0xffff) << 16,
            0x10 => match rs as u32 {
                0 => self.gpr[rt] = self.cp0[rd],
                4 => self.cp0[rd] = a,
                16 if ins & 63 == 0x10 => {
                    let status = self.cp0[12];
                    self.cp0[12] = (status & !0x0f) | ((status >> 2) & 0x0f);
                }
                _ => return Err(IopFault::ReservedInstruction { pc, instruction: ins }),
            },
            0x20 => self.gpr[rt] = memory.read_u8(address)? as i8 as i32 as u32,
            0x21 => self.gpr[rt] = memory.read_u16(address)? as i16 as i32 as u32,
            0x23 => self.gpr[rt] = memory.read_u32(address)?,
            0x22 => {
                const KEEP: [u32; 4] = [0x00ff_ffff, 0x0000_ffff, 0x0000_00ff, 0];
                const SHIFT: [u32; 4] = [24, 16, 8, 0];
                let lane = (address & 3) as usize;
                let word = memory.read_u32(address & !3)?;
                self.gpr[rt] = (self.gpr[rt] & KEEP[lane]) | word.wrapping_shl(SHIFT[lane]);
            }
            0x26 => {
                const KEEP: [u32; 4] = [0, 0xff00_0000, 0xffff_0000, 0xffff_ff00];
                const SHIFT: [u32; 4] = [0, 8, 16, 24];
                let lane = (address & 3) as usize;
                let word = memory.read_u32(address & !3)?;
                self.gpr[rt] = (self.gpr[rt] & KEEP[lane]) | (word >> SHIFT[lane]);
            }
            0x24 => self.gpr[rt] = memory.read_u8(address)? as u32,
            0x25 => self.gpr[rt] = memory.read_u16(address)? as u32,
            0x28 => memory.write_u8(address, b as u8)?,
            0x29 => memory.write_u16(address, b as u16)?,
            0x2b => memory.write_u32(address, b)?,
            0x2a => {
                const KEEP: [u32; 4] = [0xffff_ff00, 0xffff_0000, 0xff00_0000, 0];
                const SHIFT: [u32; 4] = [24, 16, 8, 0];
                let lane = (address & 3) as usize;
                let aligned = address & !3;
                let prior = memory.read_u32(aligned)?;
                memory.write_u32(aligned, ((b >> SHIFT[lane]) & !KEEP[lane]) | (prior & KEEP[lane]))?;
            }
            0x2e => {
                const KEEP: [u32; 4] = [0, 0x0000_00ff, 0x0000_ffff, 0x00ff_ffff];
                const SHIFT: [u32; 4] = [0, 8, 16, 24];
                let lane = (address & 3) as usize;
                let aligned = address & !3;
                let prior = memory.read_u32(aligned)?;
                memory.write_u32(aligned, (b.wrapping_shl(SHIFT[lane]) & !KEEP[lane]) | (prior & KEEP[lane]))?;
            }
            _ => return Err(IopFault::ReservedInstruction { pc, instruction: ins }),
        }
        Ok(())
        })();
        if let Err(fault) = execution {
            let (code, bad_vaddr) = match &fault {
                IopFault::Memory(IopMemoryError::Misaligned { address, .. }) => (4, Some(*address)),
                IopFault::Memory(_) => (7, None),
                IopFault::Syscall { .. } => (8, None),
                IopFault::Break { .. } => (9, None),
                IopFault::ReservedInstruction { .. } => (10, None),
                IopFault::ArithmeticOverflow { .. } => (12, None),
            };
            self.enter_exception(code, pc, bad_vaddr);
            return Ok(());
        }
        let old_branch = self.pending_branch.take();
        self.pc = old_branch.map_or(pc.wrapping_add(4), |pending| pending.target);
        // Branches take effect after the next instruction. Preserve a new
        // branch only when the current instruction was not itself a delay slot.
        self.pending_branch = if old_branch.is_none() { branch } else { None };
        self.gpr[0] = 0;
        self.retired_instructions = self.retired_instructions.wrapping_add(1);
        Ok(())
    }

    pub fn run(&mut self, memory: &mut IopMemory, instruction_budget: u64) -> Result<(), IopFault> {
        for _ in 0..instruction_budget { self.step(memory)?; }
        Ok(())
    }

    fn enter_exception(&mut self, code: u32, pc: u32, bad_vaddr: Option<u32>) {
        let delay_branch = self.pending_branch.take();
        let in_delay_slot = delay_branch.is_some();
        let exception_pc = delay_branch.map_or(pc, |branch| branch.origin);
        let mut cause = self.cp0[13] & !((0x1f << 2) | (1 << 31));
        cause |= (code & 0x1f) << 2;
        if in_delay_slot { cause |= 1 << 31; }
        self.cp0[13] = cause;
        self.cp0[14] = exception_pc;
        if let Some(address) = bad_vaddr { self.cp0[8] = address; }
        let status = self.cp0[12];
        self.cp0[12] = (status & !0x3f) | ((status << 2) & 0x3f);
        self.pc = if status & (1 << 22) != 0 { 0xbfc0_0180 } else { 0x8000_0080 };
        self.exceptions_taken = self.exceptions_taken.wrapping_add(1);
    }
}
