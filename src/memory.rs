//! PS2 guest memory regions used by the first executable EE slice.

use std::collections::VecDeque;
use std::fmt;
use crate::gs::Gs;
use crate::sif::{SifBus, SifChannel};
use crate::vif::{Vif, VifError, VifReport};

pub const MAIN_RAM_SIZE: usize = 32 * 1024 * 1024;
pub const BIOS_ROM_BASE: u32 = 0x1fc0_0000;
pub const BIOS_ROM_SIZE: usize = 4 * 1024 * 1024;
pub const SCRATCHPAD_BASE: u32 = 0x7000_0000;
pub const SCRATCHPAD_SIZE: usize = 16 * 1024;
pub const VU0_MICRO_BASE: u32 = 0x1100_0000;
pub const VU0_DATA_BASE: u32 = 0x1100_4000;
pub const VU0_MEMORY_SIZE: usize = 4 * 1024;
pub const VU1_MICRO_BASE: u32 = 0x1100_8000;
pub const VU1_DATA_BASE: u32 = 0x1100_c000;
pub const VU1_MEMORY_SIZE: usize = 16 * 1024;
pub const EE_TLB_ENTRIES: usize = 48;
pub const INTC_STAT: u32 = 0x1000_f000;
pub const INTC_MASK: u32 = 0x1000_f010;
pub const DMAC_EXT_BASE: u32 = 0x1000_f500;
pub const DMAC_EXT_SIZE: usize = 0x100;
pub const EE_TIMER_BASE: u32 = 0x1000_0000;
pub const EE_TIMER_STRIDE: u32 = 0x800;
pub const EE_TIMER_COUNT: u32 = 0x00;
pub const EE_TIMER_MODE: u32 = 0x10;
pub const EE_TIMER_COMP: u32 = 0x20;
pub const EE_TIMER_HOLD: u32 = 0x30;
const EE_SIF0_CHCR: u32 = 0x1000_c000;
const EE_SIF0_MADR: u32 = 0x1000_c010;
const EE_SIF0_QWC: u32 = 0x1000_c020;
const EE_SIF0_TADR: u32 = 0x1000_c030;
const EE_SIF1_CHCR: u32 = 0x1000_c400;
const EE_SIF1_MADR: u32 = 0x1000_c410;
const EE_SIF1_QWC: u32 = 0x1000_c420;
const EE_SIF1_TADR: u32 = 0x1000_c430;
const EE_SPR_FROM_CHCR: u32 = 0x1000_d000;
const EE_SPR_FROM_MADR: u32 = 0x1000_d010;
const EE_SPR_FROM_QWC: u32 = 0x1000_d020;
const EE_SPR_FROM_SADR: u32 = 0x1000_d080;
const EE_SPR_TO_CHCR: u32 = 0x1000_d400;
const EE_SPR_TO_MADR: u32 = 0x1000_d410;
const EE_SPR_TO_QWC: u32 = 0x1000_d420;
const EE_SPR_TO_TADR: u32 = 0x1000_d430;
const EE_SPR_TO_SADR: u32 = 0x1000_d480;
const EE_GIF_CHCR: u32 = 0x1000_a000;
const EE_GIF_MADR: u32 = 0x1000_a010;
const EE_GIF_QWC: u32 = 0x1000_a020;
const EE_GIF_TADR: u32 = 0x1000_a030;
const EE_GS_TO_EE_CHCR: u32 = 0x1000_b000;
const EE_GS_TO_EE_MADR: u32 = 0x1000_b010;
const EE_GS_TO_EE_QWC: u32 = 0x1000_b020;
const EE_GS_TO_EE_TADR: u32 = 0x1000_b030;
const GS_BUSDIR: u32 = 0x1200_1040;
const EE_VIF0_CHCR: u32 = 0x1000_8000;
const EE_VIF0_MADR: u32 = 0x1000_8010;
const EE_VIF0_QWC: u32 = 0x1000_8020;
const EE_VIF0_TADR: u32 = 0x1000_8030;
const EE_VIF1_CHCR: u32 = 0x1000_9000;
const EE_VIF1_MADR: u32 = 0x1000_9010;
const EE_VIF1_QWC: u32 = 0x1000_9020;
const EE_VIF1_TADR: u32 = 0x1000_9030;
const EE_D_STAT: u32 = 0x1000_e010;
const GIF_DMA_MAX_QWORDS: usize = 1_048_576;
const GIF_DMA_MAX_TAGS: usize = 65_536;
const VIF_DMA_MAX_QWORDS: usize = 1_048_576;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TlbEntry {
    pub page_mask: u32,
    pub entry_hi: u32,
    pub entry_lo0: u32,
    pub entry_lo1: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MemoryError {
    Unmapped { address: u32, width: usize },
    Misaligned { address: u32, alignment: usize },
    OutOfBounds { address: u32, length: usize },
    TlbMiss { address: u32, store: bool },
    TlbInvalid { address: u32, store: bool },
    TlbModified { address: u32 },
    Privilege { address: u32, store: bool },
    ReadOnly { address: u32 },
}

impl fmt::Display for MemoryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Unmapped { address, width } => {
                write!(f, "unmapped guest address {address:#010x} ({width} bytes)")
            }
            Self::Misaligned { address, alignment } => {
                write!(f, "unaligned guest address {address:#010x}, alignment {alignment}")
            }
            Self::OutOfBounds { address, length } => {
                write!(f, "guest range {address:#010x}+{length:#x} exceeds mapped memory")
            }
            Self::TlbMiss { address, store } => write!(f, "EE TLB miss on {} at {address:#010x}", if *store { "store" } else { "load" }),
            Self::TlbInvalid { address, store } => write!(f, "invalid EE TLB entry on {} at {address:#010x}", if *store { "store" } else { "load" }),
            Self::TlbModified { address } => write!(f, "EE TLB write to clean page at {address:#010x}"),
            Self::Privilege { address, store } => write!(f, "privileged address {address:#010x} accessed by user-mode {}", if *store { "store" } else { "load" }),
            Self::ReadOnly { address } => write!(f, "write to read-only BIOS ROM at {address:#010x}"),
        }
    }
}

impl std::error::Error for MemoryError {}

#[derive(Clone)]
pub struct Memory {
    main_ram: Vec<u8>,
    scratchpad: Vec<u8>,
    bios_rom: Vec<u8>,
    vu0_micro: Vec<u8>,
    vu0_data: Vec<u8>,
    vu1_micro: Vec<u8>,
    vu1_data: Vec<u8>,
    tlb: [TlbEntry; EE_TLB_ENTRIES],
    user_mode: bool,
    tlb_asid_hint: u32,
    intc: Intc,
    d_stat: u32,
    dmac_ext: [u32; DMAC_EXT_SIZE / 4],
    gs: Gs,
    gif_chcr: u32,
    gif_madr: u32,
    gif_qwc: u32,
    gif_tadr: u32,
    gif_dma_error: Option<String>,
    gs_to_ee_chcr: u32,
    gs_to_ee_madr: u32,
    gs_to_ee_qwc: u32,
    gs_to_ee_tadr: u32,
    gs_to_ee_dma_error: Option<String>,
    gs_busdir: u64,
    vif0: Vif,
    vif1: Vif,
    vif0_chcr: u32,
    vif0_madr: u32,
    vif0_qwc: u32,
    vif0_tadr: u32,
    vif1_chcr: u32,
    vif1_madr: u32,
    vif1_qwc: u32,
    vif1_tadr: u32,
    vif_dma_errors: [Option<String>; 2],
    vif_events: Vec<(u8, crate::vif::VifEvent)>,
    diagnostic_counters: [u64; 6],
    timers: EeTimers,
    sif: SifBus,
    sif0_chcr: u32,
    sif0_madr: u32,
    sif0_qwc: u32,
    sif0_tadr: u32,
    sif0_chain_end: bool,
    sif1_chcr: u32,
    sif1_madr: u32,
    sif1_qwc: u32,
    sif1_tadr: u32,
    sif1_chain_words: VecDeque<u32>,
    sif1_chain_loaded: bool,
    sif1_chain_final_madr: u32,
    sif1_chain_final_tadr: u32,
    sif_dma_error: Option<String>,
    spr_from_chcr: u32,
    spr_from_madr: u32,
    spr_from_qwc: u32,
    spr_from_sadr: u32,
    spr_to_chcr: u32,
    spr_to_madr: u32,
    spr_to_qwc: u32,
    spr_to_tadr: u32,
    spr_to_sadr: u32,
    scratchpad_dma_error: Option<String>,
}

#[derive(Clone, Copy, Default)]
struct Intc {
    status: u32,
    mask: u32,
}

impl Intc {
    fn read(&self, address: u32) -> Option<u32> {
        match address {
            INTC_STAT => Some(self.status),
            INTC_MASK => Some(self.mask),
            _ => None,
        }
    }

    fn write(&mut self, address: u32, value: u32) -> bool {
        match address {
            INTC_STAT => self.status &= !value,
            INTC_MASK => self.mask ^= value,
            _ => return false,
        }
        true
    }
}

#[derive(Clone, Copy, Default)]
struct EeTimer {
    count: u16,
    mode: u32,
    compare: u16,
    hold: u16,
    prescaler_remainder: u32,
}

#[derive(Clone, Copy, Default)]
struct EeTimers {
    channels: [EeTimer; 4],
}

impl EeTimers {
    fn channel_register(address: u32) -> Option<(usize, u32)> {
        let offset = address.checked_sub(EE_TIMER_BASE)?;
        let channel = (offset / EE_TIMER_STRIDE) as usize;
        let register = offset % EE_TIMER_STRIDE;
        if channel < 4 && matches!(register, EE_TIMER_COUNT | EE_TIMER_MODE | EE_TIMER_COMP | EE_TIMER_HOLD) {
            Some((channel, register))
        } else {
            None
        }
    }

    fn read(&self, address: u32) -> Option<u32> {
        let (channel, register) = Self::channel_register(address)?;
        let timer = self.channels[channel];
        match register {
            EE_TIMER_COUNT => Some(timer.count as u32),
            EE_TIMER_MODE => Some(timer.mode),
            EE_TIMER_COMP => Some(timer.compare as u32),
            EE_TIMER_HOLD if channel < 2 => Some(timer.hold as u32),
            _ => None,
        }
    }

    fn write(&mut self, address: u32, value: u32) -> Option<(usize, bool)> {
        let (channel, register) = Self::channel_register(address)?;
        let timer = &mut self.channels[channel];
        match register {
            EE_TIMER_COUNT => timer.count = value as u16,
            EE_TIMER_MODE => {
                // Bits 10 and 11 are write-one-to-clear event flags. The lower
                // ten bits are writable control; reserved bits read as zero.
                let old_flags = timer.mode & 0x0c00;
                let cleared_flags = old_flags & !(value & 0x0c00);
                timer.mode = (value & 0x03ff) | cleared_flags;
                timer.prescaler_remainder = 0;
            }
            EE_TIMER_COMP => timer.compare = value as u16,
            EE_TIMER_HOLD if channel < 2 => timer.hold = value as u16,
            _ => return None,
        }
        Some((channel, false))
    }

    fn advance(&mut self, cycles: u64) -> u8 {
        let mut irq_sources = 0u8;
        for (index, timer) in self.channels.iter_mut().enumerate() {
            // Clock selector 3 is HBLANK-driven and waits for the GS timing
            // device, which is not part of this bring-up slice.
            if timer.mode & (1 << 7) == 0 || timer.mode & 3 == 3 {
                continue;
            }
            let divider = 1u64 << (((timer.mode & 3) * 4) as u32);
            let accumulated = timer.prescaler_remainder as u64 + cycles;
            let ticks = accumulated / divider;
            timer.prescaler_remainder = (accumulated % divider) as u32;
            for _ in 0..ticks {
                let old_count = timer.count;
                timer.count = timer.count.wrapping_add(1);
                if timer.count == timer.compare {
                    if timer.mode & (1 << 10) == 0 {
                        timer.mode |= 1 << 10;
                        if timer.mode & (1 << 8) != 0 {
                            irq_sources |= 1 << index;
                        }
                    }
                    if timer.mode & (1 << 6) != 0 {
                        timer.count = 0;
                    }
                }
                if old_count == u16::MAX && timer.mode & (1 << 11) == 0 {
                    timer.mode |= 1 << 11;
                    if timer.mode & (1 << 9) != 0 {
                        irq_sources |= 1 << index;
                    }
                }
            }
        }
        irq_sources
    }
}

impl Default for Memory {
    fn default() -> Self {
        Self::new()
    }
}

impl Memory {
    pub fn new() -> Self {
        Self::with_sif(SifBus::new())
    }

    pub fn with_sif(sif: SifBus) -> Self {
        Self {
            main_ram: vec![0; MAIN_RAM_SIZE],
            scratchpad: vec![0; SCRATCHPAD_SIZE],
            bios_rom: vec![0xff; BIOS_ROM_SIZE],
            vu0_micro: vec![0; VU0_MEMORY_SIZE],
            vu0_data: vec![0; VU0_MEMORY_SIZE],
            vu1_micro: vec![0; VU1_MEMORY_SIZE],
            vu1_data: vec![0; VU1_MEMORY_SIZE],
            tlb: [TlbEntry::default(); EE_TLB_ENTRIES],
            user_mode: false,
            tlb_asid_hint: 0,
            intc: Intc::default(),
            d_stat: 0,
            dmac_ext: [0; DMAC_EXT_SIZE / 4],
            gs: Gs::default(),
            gif_chcr: 0,
            gif_madr: 0,
            gif_qwc: 0,
            gif_tadr: 0,
            gif_dma_error: None,
            gs_to_ee_chcr: 0,
            gs_to_ee_madr: 0,
            gs_to_ee_qwc: 0,
            gs_to_ee_tadr: 0,
            gs_to_ee_dma_error: None,
            gs_busdir: 0,
            vif0: Vif::new(0).expect("valid VIF0 unit"),
            vif1: Vif::new(1).expect("valid VIF1 unit"),
            vif0_chcr: 0,
            vif0_madr: 0,
            vif0_qwc: 0,
            vif0_tadr: 0,
            vif1_chcr: 0,
            vif1_madr: 0,
            vif1_qwc: 0,
            vif1_tadr: 0,
            vif_dma_errors: [None, None],
            vif_events: Vec::new(),
            diagnostic_counters: [0; 6],
            timers: EeTimers::default(),
            sif,
            sif0_chcr: 0,
            sif0_madr: 0,
            sif0_qwc: 0,
            sif0_tadr: 0,
            sif0_chain_end: false,
            sif1_chcr: 0,
            sif1_madr: 0,
            sif1_qwc: 0,
            sif1_tadr: 0,
            sif1_chain_words: VecDeque::new(),
            sif1_chain_loaded: false,
            sif1_chain_final_madr: 0,
            sif1_chain_final_tadr: 0,
            sif_dma_error: None,
            spr_from_chcr: 0,
            spr_from_madr: 0,
            spr_from_qwc: 0,
            spr_from_sadr: 0,
            spr_to_chcr: 0,
            spr_to_madr: 0,
            spr_to_qwc: 0,
            spr_to_tadr: 0,
            spr_to_sadr: 0,
            scratchpad_dma_error: None,
        }
    }

    pub fn set_user_mode(&mut self, user_mode: bool) {
        self.user_mode = user_mode;
    }

    /// Install a BIOS image in the EE's 4 MiB boot-ROM window. Smaller
    /// supported dumps are mirrored across the window; writes stay blocked.
    pub fn load_bios_rom(&mut self, image: &[u8]) -> Result<(), MemoryError> {
        if image.is_empty() || image.len() > BIOS_ROM_SIZE || BIOS_ROM_SIZE % image.len() != 0 {
            return Err(MemoryError::OutOfBounds { address: BIOS_ROM_BASE, length: image.len() });
        }
        for (index, byte) in self.bios_rom.iter_mut().enumerate() {
            *byte = image[index % image.len()];
        }
        Ok(())
    }

    pub fn raise_intc_irq(&mut self, source: u8) -> bool {
        if source >= 15 {
            return false;
        }
        self.intc.status |= 1u32 << source;
        true
    }

    pub fn intc_irq_pending(&self) -> bool {
        self.intc.status & self.intc.mask != 0
    }

    pub fn gs(&self) -> &Gs { &self.gs }

    pub fn gs_mut(&mut self) -> &mut Gs { &mut self.gs }

    pub(crate) fn vu_micro_word(&self, unit: u8, offset: u32) -> u32 {
        let memory = if unit == 0 { &self.vu0_micro } else { &self.vu1_micro };
        let start = (offset as usize) & (memory.len() - 4);
        u32::from_le_bytes(memory[start..start + 4].try_into().expect("fixed VU word slice"))
    }

    pub(crate) fn vu_data_halfword(&self, unit: u8, offset: u32) -> u16 {
        let memory = if unit == 0 { &self.vu0_data } else { &self.vu1_data };
        let start = (offset as usize) & (memory.len() - 2);
        u16::from_le_bytes(memory[start..start + 2].try_into().expect("fixed VU halfword slice"))
    }

    pub(crate) fn set_vu_data_halfword(&mut self, unit: u8, offset: u32, value: u16) {
        let memory = if unit == 0 { &mut self.vu0_data } else { &mut self.vu1_data };
        let start = (offset as usize) & (memory.len() - 2);
        memory[start..start + 2].copy_from_slice(&value.to_le_bytes());
    }

    /// Write one VU data-memory vector. `unit` is 0 or 1 and the address is in
    /// 128-bit vector units, as used by VIF UNPACK.
    pub fn write_vu_data_vector(&mut self, unit: u8, address: u32, vector: [u32; 4]) -> bool {
        let (memory, size) = match unit {
            0 => (&mut self.vu0_data, VU0_MEMORY_SIZE),
            1 => (&mut self.vu1_data, VU1_MEMORY_SIZE),
            _ => return false,
        };
        let offset = (address as usize % (size / 16)) * 16;
        for (component, value) in vector.into_iter().enumerate() {
            let start = offset + component * 4;
            memory[start..start + 4].copy_from_slice(&value.to_le_bytes());
        }
        true
    }

    /// Read one VU data-memory vector in X,Y,Z,W order.
    pub fn read_vu_data_vector(&self, unit: u8, address: u32) -> Option<[u32; 4]> {
        let (memory, size) = match unit {
            0 => (&self.vu0_data, VU0_MEMORY_SIZE),
            1 => (&self.vu1_data, VU1_MEMORY_SIZE),
            _ => return None,
        };
        let offset = (address as usize % (size / 16)) * 16;
        Some(std::array::from_fn(|i| {
            u32::from_le_bytes(memory[offset + i * 4..offset + i * 4 + 4].try_into().expect("VU vector lane"))
        }))
    }

    /// Load VIF MPG bytes into VU micro memory, wrapping at the unit's size.
    pub fn write_vu_micro_bytes(&mut self, unit: u8, address: usize, bytes: &[u8]) -> bool {
        let memory = match unit {
            0 => &mut self.vu0_micro,
            1 => &mut self.vu1_micro,
            _ => return false,
        };
        let length = memory.len();
        for (index, byte) in bytes.iter().copied().enumerate() {
            memory[(address + index) % length] = byte;
        }
        true
    }

    pub fn take_gif_dma_error(&mut self) -> Option<String> { self.gif_dma_error.take() }
    pub fn take_gs_to_ee_dma_error(&mut self) -> Option<String> { self.gs_to_ee_dma_error.take() }
    pub fn take_sif_dma_error(&mut self) -> Option<String> { self.sif_dma_error.take() }

    pub fn take_scratchpad_dma_error(&mut self) -> Option<String> { self.scratchpad_dma_error.take() }

    pub fn take_vif_dma_error(&mut self, unit: u8) -> Option<String> {
        self.vif_dma_errors.get_mut(unit as usize).and_then(Option::take)
    }

    pub fn take_vif_events(&mut self) -> Vec<(u8, crate::vif::VifEvent)> {
        std::mem::take(&mut self.vif_events)
    }

    /// VIF0, VIF1, unpack vectors, VU micro bytes, VIF DIRECT qwords, GIF DMA qwords.
    pub fn diagnostic_counters(&self) -> [u64; 6] { self.diagnostic_counters }

    pub fn reset_diagnostic_counters(&mut self) { self.diagnostic_counters = [0; 6]; }

    pub(crate) fn record_vif_direct_qwords(&mut self, count: u64) {
        self.diagnostic_counters[4] = self.diagnostic_counters[4].wrapping_add(count);
    }

    /// Advance the coarse EE bus-clock model and raise timer INTC sources.
    /// Clock source 3 (HBLANK) remains idle until a GS timing device is added.
    pub fn advance_cycles(&mut self, cycles: u64) {
        self.service_vif_dma();
        self.service_gif_dma();
        self.service_gs_to_ee_dma();
        self.service_sif_dma();
        self.service_scratchpad_dma();
        let sources = self.timers.advance(cycles);
        for timer in 0..4 {
            if sources & (1 << timer) != 0 {
                let _ = self.raise_intc_irq(9 + timer as u8);
            }
        }
    }

    fn service_vif_dma(&mut self) {
        for unit in 0..2u8 {
            let (chcr, qwc) = if unit == 0 { (self.vif0_chcr, self.vif0_qwc) } else { (self.vif1_chcr, self.vif1_qwc) };
            if chcr & (1 << 8) == 0 { continue; }
            let mode = (chcr >> 2) & 3;
            if mode > 1 {
                self.vif_dma_errors[unit as usize] = Some(format!("unsupported VIF DMA mode {mode}"));
                self.set_vif_dma_str(unit, false);
                continue;
            }
            let chain = mode == 1;
            let address = if chain {
                if unit == 0 { self.vif0_tadr } else { self.vif1_tadr }
            } else if unit == 0 { self.vif0_madr } else { self.vif1_madr };
            let (packet, final_address, final_madr) = if chain {
                self.collect_vif_chain(unit, address, chcr)
            } else {
                let (packet, final_address) = self.collect_vif_normal(address, qwc as usize);
                (packet, final_address, final_address)
            };
            let packet = match packet {
                Ok(packet) => packet,
                Err(error) => {
                    self.vif_dma_errors[unit as usize] = Some(error);
                    self.set_vif_dma_str(unit, false);
                    continue;
                }
            };
            let result = self.execute_vif_stream(unit, &packet);
            match result {
                Ok(report) => {
                    self.diagnostic_counters[unit as usize] = self.diagnostic_counters[unit as usize].wrapping_add(1);
                    self.diagnostic_counters[2] = self.diagnostic_counters[2].wrapping_add(report.unpack_vectors as u64);
                    self.diagnostic_counters[3] = self.diagnostic_counters[3].wrapping_add(report.micro_bytes as u64);
                    self.vif_events.extend(report.events.into_iter().map(|event| (unit, event)));
                    if unit == 0 {
                        self.vif0_madr = final_madr;
                        self.vif0_tadr = final_address;
                        self.vif0_qwc = 0;
                    } else {
                        self.vif1_madr = final_madr;
                        self.vif1_tadr = final_address;
                        self.vif1_qwc = 0;
                    }
                    self.set_vif_dma_str(unit, false);
                    self.complete_dmac_channel(unit as u32);
                }
                Err(error) => {
                    self.vif_dma_errors[unit as usize] = Some(error.to_string());
                    self.set_vif_dma_str(unit, false);
                }
            }
        }
    }

    fn collect_vif_normal(&self, start: u32, qwc: usize) -> (Result<Vec<u8>, String>, u32) {
        if qwc > VIF_DMA_MAX_QWORDS {
            return (Err(String::from("VIF normal DMA exceeds the packet limit")), start);
        }
        let Some(byte_count) = qwc.checked_mul(16) else {
            return (Err(String::from("VIF normal DMA byte count overflow")), start);
        };
        let mut bytes = Vec::new();
        if bytes.try_reserve_exact(byte_count).is_err() {
            return (Err(String::from("VIF DMA packet allocation failed")), start);
        }
        let mut address = start;
        for _ in 0..qwc {
            match self.read_gif_qword(address) {
                Ok(qword) => bytes.extend_from_slice(&qword.to_le_bytes()),
                Err(error) => return (Err(error), address),
            }
            address = address.wrapping_add(16);
        }
        (Ok(bytes), address)
    }

    fn collect_vif_chain(&self, unit: u8, start: u32, chcr: u32) -> (Result<Vec<u8>, String>, u32, u32) {
        let mut packet = Vec::new();
        let mut tag_address = start & 0x7fff_fff0;
        let mut current_madr = if unit == 0 { self.vif0_madr } else { self.vif1_madr };
        let mut call_stack = [0u32; 2];
        let mut stack_depth = 0usize;
        let transfer_tag = chcr & (1 << 6) != 0;
        for _ in 0..GIF_DMA_MAX_TAGS {
            let tag = match self.read_gif_qword(tag_address) {
                Ok(tag) => tag,
                Err(error) => return (Err(error), tag_address, current_madr),
            };
            let tag_low = tag as u64;
            let qwc = (tag_low & 0xffff) as usize;
            let id = ((tag_low >> 28) & 7) as u8;
            let irq = tag_low & (1 << 31) != 0;
            let target = ((tag_low >> 32) as u32) & 0x7fff_fff0;
            let inline_address = tag_address.wrapping_add(16) & 0x7fff_fff0;
            if packet.len().saturating_add(qwc).saturating_add(usize::from(transfer_tag)) > VIF_DMA_MAX_QWORDS {
                return (Err(String::from("VIF chain exceeds the packet limit")), tag_address, current_madr);
            }
            if transfer_tag {
                packet.extend_from_slice(&((tag >> 64) as u64).to_le_bytes());
            }
            let data_address = if matches!(id, 0 | 3 | 4) { target } else { inline_address };
            for index in 0..qwc {
                let address = data_address.wrapping_add((index as u32).wrapping_mul(16));
                match self.read_gif_qword(address) {
                    Ok(qword) => packet.extend_from_slice(&qword.to_le_bytes()),
                    Err(error) => return (Err(error), tag_address, current_madr),
                }
            }
            let after_data = data_address.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0;
            current_madr = after_data;
            let mut end = false;
            tag_address = match id {
                0 => { end = true; after_data } // REFE
                1 => inline_address.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0, // CNT
                2 => target, // NEXT
                3 | 4 => inline_address, // REF / REFS
                5 => {
                    if stack_depth == call_stack.len() {
                        return (Err(String::from("VIF chain CALL stack overflow")), tag_address, current_madr);
                    }
                    call_stack[stack_depth] = inline_address.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0;
                    stack_depth += 1;
                    target
                }
                6 => {
                    if stack_depth == 0 {
                        return (Err(String::from("VIF chain RET with an empty stack")), tag_address, current_madr);
                    }
                    stack_depth -= 1;
                    call_stack[stack_depth]
                }
                7 => { end = true; inline_address.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0 } // END
                _ => unreachable!(),
            };
            if chcr & (1 << 7) != 0 && irq { end = true; }
            if end { return (Ok(packet), tag_address, current_madr); }
        }
        (Err(String::from("VIF chain exceeded the tag limit")), tag_address, current_madr)
    }

    fn execute_vif_stream(&mut self, unit: u8, bytes: &[u8]) -> Result<VifReport, VifError> {
        let mut vif = if unit == 0 {
            std::mem::replace(&mut self.vif0, Vif::new(0).expect("valid VIF0 unit"))
        } else {
            std::mem::replace(&mut self.vif1, Vif::new(1).expect("valid VIF1 unit"))
        };
        let result = vif.process_stream(bytes, self);
        if unit == 0 { self.vif0 = vif; } else { self.vif1 = vif; }
        result
    }

    fn set_vif_dma_str(&mut self, unit: u8, running: bool) {
        let register = if unit == 0 { &mut self.vif0_chcr } else { &mut self.vif1_chcr };
        if running { *register |= 1 << 8; } else { *register &= !(1 << 8); }
    }

    fn service_gif_dma(&mut self) {
        if self.gif_chcr & (1 << 8) == 0 { return; }
        let (packet, final_address) = match (self.gif_chcr >> 2) & 3 {
            0 => self.collect_gif_normal(),
            1 => self.collect_gif_chain(),
            _ => return,
        };
        let (packet, final_address) = match (packet, final_address) {
            (Ok(packet), address) => (packet, address),
            (Err(error), _) => {
                self.gif_dma_error = Some(error);
                self.gif_chcr &= !(1 << 8);
                return;
            }
        };
        if let Err(error) = self.gs.submit_gif(&packet) {
            self.gif_dma_error = Some(error.to_string());
            self.gif_chcr &= !(1 << 8);
            return;
        }
        self.diagnostic_counters[5] = self.diagnostic_counters[5].wrapping_add(packet.len() as u64);
        if (self.gif_chcr >> 2) & 3 == 0 {
            self.gif_madr = final_address & 0x7fff_fff0;
            self.gif_qwc = 0;
        } else {
            self.gif_tadr = final_address & 0x7fff_fff0;
        }
        self.gif_chcr &= !(1 << 8);
        self.complete_dmac_channel(2);
    }

    fn service_gs_to_ee_dma(&mut self) {
        if self.gs_to_ee_chcr & (1 << 8) == 0 { return; }
        if (self.gs_to_ee_chcr >> 2) & 3 != 0 {
            self.gs_to_ee_dma_error = Some(String::from("GS-to-EE DMA currently supports normal mode only"));
            self.gs_to_ee_chcr &= !(1 << 8);
            return;
        }
        if self.gs_busdir & 1 == 0 {
            self.gs_to_ee_dma_error = Some(String::from("GS-to-EE DMA requires BUSDIR=1"));
            self.gs_to_ee_chcr &= !(1 << 8);
            return;
        }
        let qwc = self.gs_to_ee_qwc as usize;
        if qwc > GIF_DMA_MAX_QWORDS {
            self.gs_to_ee_dma_error = Some(String::from("GS-to-EE DMA exceeds the qword limit"));
            self.gs_to_ee_chcr &= !(1 << 8);
            return;
        }
        if qwc == 0 {
            self.gs_to_ee_chcr &= !(1 << 8);
            self.complete_dmac_channel(3);
            return;
        }
        let Some(image) = self.gs.readback_image() else {
            self.gs_to_ee_dma_error = Some(String::from("GS has no supported local-to-host transfer configured"));
            self.gs_to_ee_chcr &= !(1 << 8);
            return;
        };
        if image.len() != qwc {
            self.gs_to_ee_dma_error = Some(format!("GS readback has {} qwords, DMA requested {qwc}", image.len()));
            self.gs_to_ee_chcr &= !(1 << 8);
            return;
        }
        let start = self.gs_to_ee_madr;
        let Some(byte_count) = qwc.checked_mul(16) else {
            self.gs_to_ee_dma_error = Some(String::from("GS-to-EE DMA byte count overflow"));
            self.gs_to_ee_chcr &= !(1 << 8);
            return;
        };
        let mut bytes = Vec::new();
        if bytes.try_reserve_exact(byte_count).is_err() {
            self.gs_to_ee_dma_error = Some(String::from("GS-to-EE DMA allocation failed"));
            self.gs_to_ee_chcr &= !(1 << 8);
            return;
        }
        for qword in image { bytes.extend_from_slice(&qword.to_le_bytes()); }
        if let Err(error) = self.write_bytes(start, &bytes) {
            self.gs_to_ee_dma_error = Some(error.to_string());
            self.gs_to_ee_chcr &= !(1 << 8);
            return;
        }
        self.gs_to_ee_madr = start.wrapping_add(byte_count as u32) & 0x7fff_fff0;
        self.gs_to_ee_qwc = 0;
        self.gs_to_ee_chcr &= !(1 << 8);
        self.complete_dmac_channel(3);
    }

    fn collect_gif_normal(&self) -> (Result<Vec<u128>, String>, u32) {
        let qwc = self.gif_qwc as usize;
        if qwc > GIF_DMA_MAX_QWORDS {
            return (Err(String::from("GIF normal DMA exceeds the packet limit")), self.gif_madr);
        }
        let mut packet = Vec::new();
        if packet.try_reserve_exact(qwc).is_err() {
            return (Err(String::from("GIF DMA packet allocation failed")), self.gif_madr);
        }
        let mut address = self.gif_madr;
        for _ in 0..qwc {
            match self.read_gif_qword(address) {
                Ok(qword) => packet.push(qword),
                Err(error) => return (Err(error), address),
            }
            address = address.wrapping_add(16);
        }
        (Ok(packet), address)
    }

    fn collect_gif_chain(&self) -> (Result<Vec<u128>, String>, u32) {
        let mut packet = Vec::new();
        let mut tag_address = self.gif_tadr & 0x7fff_fff0;
        let mut call_stack = [0u32; 2];
        let mut stack_depth = 0usize;
        for _ in 0..GIF_DMA_MAX_TAGS {
            let tag = match self.read_gif_qword(tag_address) {
                Ok(tag) => tag,
                Err(error) => return (Err(error), tag_address),
            };
            let tag_low = tag as u64;
            let qwc = (tag_low & 0xffff) as usize;
            let id = ((tag_low >> 28) & 7) as u8;
            let irq = tag_low & (1 << 31) != 0;
            let target = ((tag_low >> 32) as u32) & 0x7fff_fff0;
            let inline_address = tag_address.wrapping_add(16) & 0x7fff_fff0;
            let transfer_tag = self.gif_chcr & (1 << 6) != 0;
            if packet.len().saturating_add(qwc).saturating_add(usize::from(transfer_tag)) > GIF_DMA_MAX_QWORDS {
                return (Err(String::from("GIF chain DMA exceeds the packet limit")), tag_address);
            }
            if transfer_tag {
                packet.push((tag >> 64) as u128);
            }
            let data_address = if matches!(id, 0 | 3 | 4) { target } else { inline_address };
            if let Err(error) = self.append_gif_qwords(&mut packet, data_address, qwc) {
                return (Err(error), tag_address);
            }
            let after_data = inline_address.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0;
            let mut end = false;
            tag_address = match id {
                0 => { end = true; after_data } // REFE
                1 => after_data, // CNT
                2 => target, // NEXT
                3 | 4 => inline_address, // REF / REFS
                5 => {
                    if stack_depth == call_stack.len() {
                        return (Err(String::from("GIF chain CALL stack overflow")), tag_address);
                    }
                    call_stack[stack_depth] = after_data;
                    stack_depth += 1;
                    target
                }
                6 => {
                    if stack_depth == 0 {
                        return (Err(String::from("GIF chain RET with an empty stack")), tag_address);
                    }
                    stack_depth -= 1;
                    call_stack[stack_depth]
                }
                7 => { end = true; after_data } // END
                _ => unreachable!(),
            };
            if self.gif_chcr & (1 << 7) != 0 && irq { end = true; }
            if end { return (Ok(packet), tag_address); }
        }
        (Err(String::from("GIF chain exceeded the tag limit")), tag_address)
    }

    fn append_gif_qwords(&self, packet: &mut Vec<u128>, address: u32, count: usize) -> Result<(), String> {
        if packet.try_reserve(count).is_err() {
            return Err(String::from("GIF DMA packet allocation failed"));
        }
        let mut cursor = address;
        for _ in 0..count {
            packet.push(self.read_gif_qword(cursor)?);
            cursor = cursor.wrapping_add(16);
        }
        Ok(())
    }

    fn read_gif_qword(&self, address: u32) -> Result<u128, String> {
        self.read_bytes(address, 16)
            .map(|bytes| u128::from_le_bytes(bytes.try_into().unwrap()))
            .map_err(|error| error.to_string())
    }

    fn service_sif_dma(&mut self) {
        if self.sif0_chcr & (1 << 8) != 0 && (self.sif0_chcr >> 2) & 3 == 0 {
            while self.sif0_qwc != 0 {
                if self.sif.fifo_len(SifChannel::Sif0) < 4 { break; }
                if let Err(error) = self.region_mut(self.sif0_madr, 16) {
                    self.sif_dma_error = Some(format!("SIF0 destination {:#010x}: {error}", self.sif0_madr));
                    self.sif0_chcr &= !(1 << 8);
                    break;
                }
                let mut words = [0u32; 4];
                if !self.sif.read_fifo_exact(SifChannel::Sif0, &mut words) { break; }
                let mut bytes = [0u8; 16];
                for (chunk, word) in bytes.chunks_exact_mut(4).zip(words) {
                    chunk.copy_from_slice(&word.to_le_bytes());
                }
                if self.write_bytes(self.sif0_madr, &bytes).is_err() {
                    self.sif0_chcr &= !(1 << 8);
                    break;
                }
                self.sif0_madr = self.sif0_madr.wrapping_add(16);
                self.sif0_qwc -= 1;
            }
            if self.sif0_qwc == 0 {
                self.sif0_chcr &= !(1 << 8);
                self.complete_dmac_channel(5);
            }
        } else if self.sif0_chcr & (1 << 8) != 0 && (self.sif0_chcr >> 2) & 3 == 1 {
            self.service_sif0_destination_chain();
        } else if self.sif0_chcr & (1 << 8) != 0 {
            self.sif_dma_error = Some(format!("unsupported SIF0 DMA mode {}", (self.sif0_chcr >> 2) & 3));
            self.sif0_chcr &= !(1 << 8);
        }

        if self.sif1_chcr & (1 << 8) != 0 && (self.sif1_chcr >> 2) & 3 == 0 {
            while self.sif1_qwc != 0 {
                let bytes = match self.read_bytes(self.sif1_madr, 16) {
                    Ok(bytes) => bytes,
                    Err(error) => {
                        self.sif_dma_error = Some(format!("SIF1 source {:#010x}: {error}", self.sif1_madr));
                        self.sif1_chcr &= !(1 << 8);
                        break;
                    }
                };
                let words: [u32; 4] = std::array::from_fn(|index| {
                    u32::from_le_bytes(bytes[index * 4..index * 4 + 4].try_into().unwrap())
                });
                if !self.sif.write_fifo_exact(SifChannel::Sif1, &words) { break; }
                self.sif1_madr = self.sif1_madr.wrapping_add(16);
                self.sif1_qwc -= 1;
            }
            if self.sif1_qwc == 0 {
                self.sif1_chcr &= !(1 << 8);
                self.complete_dmac_channel(6);
            }
        } else if self.sif1_chcr & (1 << 8) != 0 && (self.sif1_chcr >> 2) & 3 == 1 {
            if !self.sif1_chain_loaded {
                match self.collect_sif1_chain(self.sif1_tadr, self.sif1_chcr) {
                    Ok((words, final_madr, final_tadr)) => {
                        self.sif1_chain_words = words.into();
                        self.sif1_chain_final_madr = final_madr;
                        self.sif1_chain_final_tadr = final_tadr;
                        self.sif1_chain_loaded = true;
                    }
                    Err(error) => {
                        self.sif_dma_error = Some(error);
                        self.sif1_chcr &= !(1 << 8);
                    }
                }
            }
            if self.sif1_chain_loaded {
                let count = self.sif1_chain_words.len().min(self.sif.fifo_free(SifChannel::Sif1));
                let words: Vec<u32> = self.sif1_chain_words.iter().take(count).copied().collect();
                let written = self.sif.write_fifo(SifChannel::Sif1, &words);
                for _ in 0..written { self.sif1_chain_words.pop_front(); }
                if self.sif1_chain_words.is_empty() {
                    self.sif1_madr = self.sif1_chain_final_madr;
                    self.sif1_tadr = self.sif1_chain_final_tadr;
                    self.sif1_qwc = 0;
                    self.sif1_chcr &= !(1 << 8);
                    self.sif1_chain_loaded = false;
                    self.complete_dmac_channel(6);
                }
            }
        } else if self.sif1_chcr & (1 << 8) != 0 {
            self.sif_dma_error = Some(format!("unsupported SIF1 DMA mode {}", (self.sif1_chcr >> 2) & 3));
            self.sif1_chcr &= !(1 << 8);
        }
    }

    fn service_sif0_destination_chain(&mut self) {
        // In destination-chain mode, SIF0 receives each four-word DMAtag
        // from the IOP FIFO, then routes the following QWC to its ADDR.
        const MAX_CHAIN_QWORDS: usize = 1 << 20;
        const MAX_CHAIN_TAGS_PER_SERVICE: usize = 4096;
        let mut tags = 0usize;

        while self.sif0_chcr & (1 << 8) != 0 && tags < MAX_CHAIN_TAGS_PER_SERVICE {
            if self.sif0_qwc == 0 {
                if self.sif0_chain_end {
                    self.sif0_chcr &= !(1 << 8);
                    self.complete_dmac_channel(5);
                    self.sif0_chain_end = false;
                    return;
                }
                if self.sif.fifo_len(SifChannel::Sif0) < 4 { return; }
                let mut tag_words = [0u32; 4];
                if self.sif.read_fifo(SifChannel::Sif0, &mut tag_words) != 4 { return; }
                let low = u64::from(tag_words[0]) | (u64::from(tag_words[1]) << 32);
                let qwc = (low & 0xffff) as usize;
                let id = ((low >> 28) & 7) as u8;
                let irq = low & (1 << 31) != 0;
                let address = ((low >> 32) as u32) & 0x7fff_fff0;
                if qwc > MAX_CHAIN_QWORDS {
                    self.sif_dma_error = Some(String::from("SIF0 destination chain exceeds the qword limit"));
                    self.sif0_chcr &= !(1 << 8);
                    return;
                }
                if !matches!(id, 0 | 1 | 7) {
                    self.sif_dma_error = Some(format!("unsupported SIF0 destination-chain tag ID {id}"));
                    self.sif0_chcr &= !(1 << 8);
                    return;
                }
                self.sif0_madr = address;
                self.sif0_qwc = qwc as u32;
                self.sif0_chain_end = id == 7 || (self.sif0_chcr & (1 << 7) != 0 && irq);
                tags += 1;
                if self.sif0_qwc == 0 { continue; }
            }

            // SIF transfers complete qwords atomically and remain active
            // while the producer has not supplied the next complete qword.
            if self.sif.fifo_len(SifChannel::Sif0) < 4 { return; }
            if let Err(error) = self.region_mut(self.sif0_madr, 16) {
                self.sif_dma_error = Some(format!("SIF0 destination {:#010x}: {error}", self.sif0_madr));
                self.sif0_chcr &= !(1 << 8);
                return;
            }
            let mut words = [0u32; 4];
            if !self.sif.read_fifo_exact(SifChannel::Sif0, &mut words) { return; }
            let mut bytes = [0u8; 16];
            for (chunk, word) in bytes.chunks_exact_mut(4).zip(words) {
                chunk.copy_from_slice(&word.to_le_bytes());
            }
            if let Err(error) = self.write_bytes(self.sif0_madr, &bytes) {
                self.sif_dma_error = Some(error.to_string());
                self.sif0_chcr &= !(1 << 8);
                return;
            }
            self.sif0_madr = self.sif0_madr.wrapping_add(16) & 0x7fff_fff0;
            self.sif0_qwc -= 1;
        }
    }

    fn collect_sif1_chain(&self, start: u32, chcr: u32) -> Result<(Vec<u32>, u32, u32), String> {
        let mut words = Vec::new();
        let mut tadr = start & 0x7fff_fff0;
        let mut call_stack = [0u32; 2];
        let mut stack_depth = 0usize;
        let mut transferred_qwords = 0usize;
        let transfer_tag = chcr & (1 << 6) != 0;
        let tie = chcr & (1 << 7) != 0;

        for _ in 0..GIF_DMA_MAX_TAGS {
            let tag = self.read_gif_qword(tadr)?;
            let low = tag as u64;
            let qwc = (low & 0xffff) as usize;
            let id = ((low >> 28) & 7) as u8;
            let irq = low & (1 << 31) != 0;
            let target = ((low >> 32) as u32) & 0x7fff_fff0;
            let inline = tadr.wrapping_add(16) & 0x7fff_fff0;
            transferred_qwords = transferred_qwords.saturating_add(qwc + usize::from(transfer_tag));
            if transferred_qwords > GIF_DMA_MAX_QWORDS {
                return Err(String::from("SIF1 chain exceeds the qword limit"));
            }
            if words.try_reserve(qwc.saturating_mul(4) + usize::from(transfer_tag) * 2).is_err() {
                return Err(String::from("SIF1 chain allocation failed"));
            }
            if transfer_tag {
                let payload = (tag >> 64) as u64;
                words.push(payload as u32);
                words.push((payload >> 32) as u32);
            }

            let data_address = if matches!(id, 0 | 3 | 4) { target } else { inline };
            for index in 0..qwc {
                let address = data_address.wrapping_add((index as u32).wrapping_mul(16));
                let qword = self.read_gif_qword(address)?;
                words.extend((0..4).map(|lane| (qword >> (lane * 32)) as u32));
            }
            let after_data = data_address.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0;
            let mut end = false;
            tadr = match id {
                0 => { end = true; after_data }
                1 => inline.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0,
                2 => target,
                3 | 4 => inline,
                5 => {
                    if stack_depth == call_stack.len() { return Err(String::from("SIF1 chain CALL stack overflow")); }
                    call_stack[stack_depth] = inline.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0;
                    stack_depth += 1;
                    target
                }
                6 => {
                    if stack_depth == 0 { return Err(String::from("SIF1 chain RET with an empty stack")); }
                    stack_depth -= 1;
                    call_stack[stack_depth]
                }
                7 => { end = true; inline.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0 }
                _ => unreachable!(),
            };
            if tie && irq { end = true; }
            if end { return Ok((words, after_data, tadr)); }
        }
        Err(String::from("SIF1 chain exceeded the tag limit"))
    }

    fn service_scratchpad_dma(&mut self) {
        // Channel 8 copies SPR to RAM; channel 9 copies RAM to SPR.
        if self.spr_from_chcr & (1 << 8) != 0 && (self.spr_from_chcr >> 2) & 3 == 0 {
            while self.spr_from_qwc != 0 {
                let source = self.spr_from_sadr as usize & (SCRATCHPAD_SIZE - 1);
                let mut bytes = [0u8; 16];
                for (index, byte) in bytes.iter_mut().enumerate() {
                    *byte = self.scratchpad[(source + index) & (SCRATCHPAD_SIZE - 1)];
                }
                if self.write_bytes(self.spr_from_madr, &bytes).is_err() {
                    self.spr_from_chcr &= !(1 << 8);
                    break;
                }
                self.spr_from_madr = self.spr_from_madr.wrapping_add(16) & 0x7fff_fff0;
                self.spr_from_sadr = self.spr_from_sadr.wrapping_add(16) & 0x3ff0;
                self.spr_from_qwc -= 1;
            }
            if self.spr_from_qwc == 0 {
                self.spr_from_chcr &= !(1 << 8);
                self.complete_dmac_channel(8);
            }
        }

        if self.spr_to_chcr & (1 << 8) != 0 && (self.spr_to_chcr >> 2) & 3 == 0 {
            while self.spr_to_qwc != 0 {
                let Ok(source_bytes) = self.read_bytes(self.spr_to_madr, 16) else {
                    self.spr_to_chcr &= !(1 << 8);
                    break;
                };
                let bytes: [u8; 16] = source_bytes.try_into().expect("16-byte scratchpad DMA read");
                let target = self.spr_to_sadr as usize & (SCRATCHPAD_SIZE - 1);
                for (index, byte) in bytes.into_iter().enumerate() {
                    let offset = (target + index) & (SCRATCHPAD_SIZE - 1);
                    self.scratchpad[offset] = byte;
                }
                self.spr_to_madr = self.spr_to_madr.wrapping_add(16) & 0x7fff_fff0;
                self.spr_to_sadr = self.spr_to_sadr.wrapping_add(16) & 0x3ff0;
                self.spr_to_qwc -= 1;
            }
            if self.spr_to_qwc == 0 {
                self.spr_to_chcr &= !(1 << 8);
                self.complete_dmac_channel(9);
            }
        } else if self.spr_to_chcr & (1 << 8) != 0 && (self.spr_to_chcr >> 2) & 3 == 1 {
            self.service_spr_to_chain();
        }
    }

    fn service_spr_to_chain(&mut self) {
        let mut tadr = self.spr_to_tadr & 0x7fff_fff0;
        let mut madr = self.spr_to_madr;
        let mut sadr = self.spr_to_sadr;
        let mut qwc_left = 0usize;
        let mut transferred = 0usize;
        let mut call_stack = [0u32; 2];
        let mut stack_depth = 0usize;
        let transfer_tag = self.spr_to_chcr & (1 << 6) != 0;
        let tie = self.spr_to_chcr & (1 << 7) != 0;

        let result = (|| -> Result<(), String> {
            for _ in 0..GIF_DMA_MAX_TAGS {
                let tag = self.read_gif_qword(tadr)?;
                let low = tag as u64;
                let qwc = (low & 0xffff) as usize;
                let id = ((low >> 28) & 7) as u8;
                let irq = low & (1 << 31) != 0;
                let target = ((low >> 32) as u32) & 0x7fff_fff0;
                let inline = tadr.wrapping_add(16) & 0x7fff_fff0;
                transferred = transferred.saturating_add(qwc);
                if transferred > GIF_DMA_MAX_QWORDS {
                    return Err(String::from("SPR_TO chain exceeds the qword limit"));
                }

                if transfer_tag {
                    let payload = ((tag >> 64) as u64).to_le_bytes();
                    for (index, byte) in payload.into_iter().enumerate() {
                        self.scratchpad[(sadr as usize + index) & (SCRATCHPAD_SIZE - 1)] = byte;
                    }
                    sadr = sadr.wrapping_add(8) & 0x3fff;
                }

                let data_address = if matches!(id, 0 | 3 | 4) { target } else { inline };
                madr = data_address;
                qwc_left = qwc;
                while qwc_left != 0 {
                    let source = self.read_bytes(madr, 16).map_err(|error| error.to_string())?;
                    let bytes: [u8; 16] = source.try_into().expect("16-byte SPR_TO chain read");
                    let target_offset = sadr as usize & (SCRATCHPAD_SIZE - 1);
                    for (index, byte) in bytes.into_iter().enumerate() {
                        self.scratchpad[(target_offset + index) & (SCRATCHPAD_SIZE - 1)] = byte;
                    }
                    madr = madr.wrapping_add(16) & 0x7fff_fff0;
                    sadr = sadr.wrapping_add(16) & 0x3ff0;
                    qwc_left -= 1;
                    self.spr_to_madr = madr;
                    self.spr_to_sadr = sadr;
                    self.spr_to_qwc = qwc_left as u32;
                }

                let after_data = data_address.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0;
                let mut end = false;
                tadr = match id {
                    0 => { end = true; after_data }
                    1 => inline.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0,
                    2 => target,
                    3 | 4 => inline,
                    5 => {
                        if stack_depth == call_stack.len() { return Err(String::from("SPR_TO chain CALL stack overflow")); }
                        call_stack[stack_depth] = inline.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0;
                        stack_depth += 1;
                        target
                    }
                    6 => {
                        if stack_depth == 0 { return Err(String::from("SPR_TO chain RET with an empty stack")); }
                        stack_depth -= 1;
                        call_stack[stack_depth]
                    }
                    7 => { end = true; inline.wrapping_add((qwc as u32).wrapping_mul(16)) & 0x7fff_fff0 }
                    _ => unreachable!(),
                };
                self.spr_to_tadr = tadr;
                self.spr_to_madr = after_data;
                self.spr_to_qwc = 0;
                self.spr_to_sadr = sadr;
                if tie && irq { end = true; }
                if end { return Ok(()); }
            }
            Err(String::from("SPR_TO chain exceeded the tag limit"))
        })();

        self.spr_to_tadr = tadr;
        self.spr_to_madr = madr;
        self.spr_to_sadr = sadr;
        self.spr_to_qwc = qwc_left as u32;
        self.spr_to_chcr &= !(1 << 8);
        match result {
            Ok(()) => self.complete_dmac_channel(9),
            Err(error) => self.scratchpad_dma_error = Some(error),
        }
    }

    fn complete_dmac_channel(&mut self, channel: u32) {
        self.d_stat |= 1 << channel;
        self.update_d_stat_master_flag();
    }

    fn update_d_stat_master_flag(&mut self) {
        let pending = self.d_stat & 0x7fff;
        let enabled = (self.d_stat >> 16) & 0x7fff;
        if pending & enabled != 0 {
            self.d_stat |= 1 << 31;
            let _ = self.raise_intc_irq(3);
        } else {
            self.d_stat &= !(1 << 31);
        }
    }

    fn write_d_stat(&mut self, value: u32) {
        self.d_stat &= !(value & 0x7fff);
        self.d_stat ^= value & 0x7fff_0000;
        self.update_d_stat_master_flag();
    }

    pub fn tlb_entry(&self, index: usize) -> Option<TlbEntry> {
        self.tlb.get(index).copied()
    }

    pub fn write_tlb_entry(&mut self, index: usize, entry: TlbEntry) -> bool {
        if let Some(slot) = self.tlb.get_mut(index) {
            *slot = entry;
            true
        } else {
            false
        }
    }

    pub fn probe_tlb(&self, entry_hi: u32) -> Option<usize> {
        self.tlb.iter().position(|entry| {
            let page_size = tlb_page_size(entry.page_mask);
            let pair_mask = !(page_size.saturating_mul(2).wrapping_sub(1));
            let global = entry.entry_lo0 & entry.entry_lo1 & 1 != 0;
            (entry.entry_hi & pair_mask) == (entry_hi & pair_mask)
                && (global || (entry.entry_hi & 0xff) == (entry_hi & 0xff))
        })
    }

    fn translate(&self, address: u32, store: bool) -> Result<u32, MemoryError> {
        if !self.user_mode {
            return Ok(if address >= 0x8000_0000 { address & 0x1fff_ffff } else { address });
        }
        if (0x8000_0000..0xc000_0000).contains(&address) {
            return Err(MemoryError::Privilege { address, store });
        }
        let Some((index, entry)) = self.tlb.iter().copied().enumerate().find(|(_, entry)| {
            let page_size = tlb_page_size(entry.page_mask);
            let pair_mask = !(page_size.saturating_mul(2).wrapping_sub(1));
            let global = entry.entry_lo0 & entry.entry_lo1 & 1 != 0;
            (entry.entry_hi & pair_mask) == (address & pair_mask)
                && (global || (entry.entry_hi & 0xff) == (self.tlb_asid_hint & 0xff))
        }) else {
            return Err(MemoryError::TlbMiss { address, store });
        };
        let _ = index;
        let page_size = tlb_page_size(entry.page_mask);
        let odd = address & page_size != 0;
        let lo = if odd { entry.entry_lo1 } else { entry.entry_lo0 };
        if lo & 2 == 0 {
            return Err(MemoryError::TlbInvalid { address, store });
        }
        if store && lo & 4 == 0 {
            return Err(MemoryError::TlbModified { address });
        }
        let frame = ((lo >> 6) & 0x000f_ffff) << 12;
        let physical_page = frame & !(page_size - 1);
        Ok(physical_page | (address & (page_size - 1)))
    }

    pub(crate) fn set_tlb_asid(&mut self, asid: u32) {
        self.tlb_asid_hint = asid & 0xff;
    }

    fn region(&self, address: u32, length: usize) -> Result<(&[u8], usize), MemoryError> {
        let physical = self.translate(address, false)?;
        if (SCRATCHPAD_BASE..SCRATCHPAD_BASE + SCRATCHPAD_SIZE as u32).contains(&physical) {
            let offset = (physical - SCRATCHPAD_BASE) as usize;
            if offset.checked_add(length).map_or(true, |end| end > self.scratchpad.len()) {
                return Err(MemoryError::OutOfBounds { address, length });
            }
            return Ok((&self.scratchpad, offset));
        }

        if (BIOS_ROM_BASE..BIOS_ROM_BASE + BIOS_ROM_SIZE as u32).contains(&physical) {
            let offset = (physical - BIOS_ROM_BASE) as usize;
            if offset.checked_add(length).map_or(true, |end| end > self.bios_rom.len()) {
                return Err(MemoryError::OutOfBounds { address, length });
            }
            return Ok((&self.bios_rom, offset));
        }

        if (VU0_MICRO_BASE..VU0_DATA_BASE).contains(&physical) {
            let offset = ((physical - VU0_MICRO_BASE) as usize) % VU0_MEMORY_SIZE;
            if offset.checked_add(length).map_or(true, |end| end > self.vu0_micro.len()) {
                return Err(MemoryError::OutOfBounds { address, length });
            }
            return Ok((&self.vu0_micro, offset));
        }
        if (VU0_DATA_BASE..VU0_DATA_BASE + 0x4000).contains(&physical) {
            let offset = ((physical - VU0_DATA_BASE) as usize) % VU0_MEMORY_SIZE;
            if offset.checked_add(length).map_or(true, |end| end > self.vu0_data.len()) {
                return Err(MemoryError::OutOfBounds { address, length });
            }
            return Ok((&self.vu0_data, offset));
        }
        if (VU1_MICRO_BASE..VU1_MICRO_BASE + VU1_MEMORY_SIZE as u32).contains(&physical) {
            let offset = ((physical - VU1_MICRO_BASE) as usize) % VU1_MEMORY_SIZE;
            if offset.checked_add(length).map_or(true, |end| end > self.vu1_micro.len()) {
                return Err(MemoryError::OutOfBounds { address, length });
            }
            return Ok((&self.vu1_micro, offset));
        }
        if (VU1_DATA_BASE..VU1_DATA_BASE + VU1_MEMORY_SIZE as u32).contains(&physical) {
            let offset = ((physical - VU1_DATA_BASE) as usize) % VU1_MEMORY_SIZE;
            if offset.checked_add(length).map_or(true, |end| end > self.vu1_data.len()) {
                return Err(MemoryError::OutOfBounds { address, length });
            }
            return Ok((&self.vu1_data, offset));
        }

        let offset = physical as usize;
        if offset.checked_add(length).map_or(true, |end| end > self.main_ram.len()) {
            return Err(MemoryError::Unmapped { address, width: length });
        }
        Ok((&self.main_ram, offset))
    }

    fn region_mut(&mut self, address: u32, length: usize) -> Result<(&mut [u8], usize), MemoryError> {
        let physical = self.translate(address, true)?;
        if (SCRATCHPAD_BASE..SCRATCHPAD_BASE + SCRATCHPAD_SIZE as u32).contains(&physical) {
            let offset = (physical - SCRATCHPAD_BASE) as usize;
            if offset.checked_add(length).map_or(true, |end| end > self.scratchpad.len()) {
                return Err(MemoryError::OutOfBounds { address, length });
            }
            return Ok((&mut self.scratchpad, offset));
        }

        if (BIOS_ROM_BASE..BIOS_ROM_BASE + BIOS_ROM_SIZE as u32).contains(&physical) {
            return Err(MemoryError::ReadOnly { address });
        }

        if (VU0_MICRO_BASE..VU0_DATA_BASE).contains(&physical) {
            let offset = ((physical - VU0_MICRO_BASE) as usize) % VU0_MEMORY_SIZE;
            if offset.checked_add(length).map_or(true, |end| end > self.vu0_micro.len()) {
                return Err(MemoryError::OutOfBounds { address, length });
            }
            return Ok((&mut self.vu0_micro, offset));
        }
        if (VU0_DATA_BASE..VU0_DATA_BASE + 0x4000).contains(&physical) {
            let offset = ((physical - VU0_DATA_BASE) as usize) % VU0_MEMORY_SIZE;
            if offset.checked_add(length).map_or(true, |end| end > self.vu0_data.len()) {
                return Err(MemoryError::OutOfBounds { address, length });
            }
            return Ok((&mut self.vu0_data, offset));
        }
        if (VU1_MICRO_BASE..VU1_MICRO_BASE + VU1_MEMORY_SIZE as u32).contains(&physical) {
            let offset = ((physical - VU1_MICRO_BASE) as usize) % VU1_MEMORY_SIZE;
            if offset.checked_add(length).map_or(true, |end| end > self.vu1_micro.len()) {
                return Err(MemoryError::OutOfBounds { address, length });
            }
            return Ok((&mut self.vu1_micro, offset));
        }
        if (VU1_DATA_BASE..VU1_DATA_BASE + VU1_MEMORY_SIZE as u32).contains(&physical) {
            let offset = ((physical - VU1_DATA_BASE) as usize) % VU1_MEMORY_SIZE;
            if offset.checked_add(length).map_or(true, |end| end > self.vu1_data.len()) {
                return Err(MemoryError::OutOfBounds { address, length });
            }
            return Ok((&mut self.vu1_data, offset));
        }

        let offset = physical as usize;
        if offset.checked_add(length).map_or(true, |end| end > self.main_ram.len()) {
            return Err(MemoryError::Unmapped { address, width: length });
        }
        Ok((&mut self.main_ram, offset))
    }

    pub fn read_bytes(&self, address: u32, length: usize) -> Result<&[u8], MemoryError> {
        let (region, offset) = self.region(address, length)?;
        Ok(&region[offset..offset + length])
    }

    pub fn write_bytes(&mut self, address: u32, bytes: &[u8]) -> Result<(), MemoryError> {
        let (region, offset) = self.region_mut(address, bytes.len())?;
        region[offset..offset + bytes.len()].copy_from_slice(bytes);
        Ok(())
    }

    fn check_alignment(address: u32, alignment: usize) -> Result<(), MemoryError> {
        if address as usize % alignment != 0 {
            return Err(MemoryError::Misaligned { address, alignment });
        }
        Ok(())
    }

    pub fn read_u8(&self, address: u32) -> Result<u8, MemoryError> {
        let physical = self.translate(address, false)?;
        if let Some(value) = self.sif.read_ee(physical & !3) {
            return Ok((value >> ((physical & 3) * 8)) as u8);
        }
        Ok(self.read_bytes(address, 1)?[0])
    }

    pub fn read_u16(&self, address: u32) -> Result<u16, MemoryError> {
        Self::check_alignment(address, 2)?;
        let physical = self.translate(address, false)?;
        if let Some(value) = self.sif.read_ee(physical & !3) {
            return Ok((value >> ((physical & 2) * 8)) as u16);
        }
        Ok(u16::from_le_bytes(self.read_bytes(address, 2)?.try_into().unwrap()))
    }

    pub fn read_u32(&self, address: u32) -> Result<u32, MemoryError> {
        Self::check_alignment(address, 4)?;
        let physical = self.translate(address, false)?;
        if physical == EE_D_STAT { return Ok(self.d_stat); }
        if let Some(value) = self.read_dmac_ext(physical) { return Ok(value); }
        if physical == GS_BUSDIR { return Ok(self.gs_busdir as u32); }
        if let Some(value) = self.intc.read(physical) {
            return Ok(value);
        }
        if let Some(value) = self.timers.read(physical) {
            return Ok(value);
        }
        if let Some(value) = self.sif.read_ee(physical) {
            return Ok(value);
        }
        if let Some(value) = self.read_sif_dma_register(physical) { return Ok(value); }
        Ok(u32::from_le_bytes(self.read_bytes(address, 4)?.try_into().unwrap()))
    }

    pub fn read_u64(&self, address: u32) -> Result<u64, MemoryError> {
        Self::check_alignment(address, 8)?;
        let physical = self.translate(address, false)?;
        if physical == GS_BUSDIR { return Ok(self.gs_busdir); }
        Ok(u64::from_le_bytes(self.read_bytes(address, 8)?.try_into().unwrap()))
    }

    pub fn read_u128(&self, address: u32) -> Result<u128, MemoryError> {
        Self::check_alignment(address, 16)?;
        Ok(u128::from_le_bytes(self.read_bytes(address, 16)?.try_into().unwrap()))
    }

    pub fn write_u8(&mut self, address: u32, value: u8) -> Result<(), MemoryError> {
        let physical = self.translate(address, true)?;
        if crate::sif::write_partial(&self.sif, physical, 1, u32::from(value), false) { return Ok(()); }
        self.write_bytes(address, &[value])
    }

    pub fn write_u16(&mut self, address: u32, value: u16) -> Result<(), MemoryError> {
        Self::check_alignment(address, 2)?;
        let physical = self.translate(address, true)?;
        if crate::sif::write_partial(&self.sif, physical, 2, u32::from(value), false) { return Ok(()); }
        self.write_bytes(address, &value.to_le_bytes())
    }

    pub fn write_u32(&mut self, address: u32, value: u32) -> Result<(), MemoryError> {
        Self::check_alignment(address, 4)?;
        let physical = self.translate(address, true)?;
        if physical == EE_D_STAT {
            self.write_d_stat(value);
            return Ok(());
        }
        if physical == GS_BUSDIR {
            self.gs_busdir = u64::from(value & 1);
            return Ok(());
        }
        if self.write_dmac_ext(physical, value) { return Ok(()); }
        if self.intc.write(physical, value) {
            return Ok(());
        }
        if self.timers.write(physical, value).is_some() {
            return Ok(());
        }
        if self.sif.write_ee(physical, value) { return Ok(()); }
        if self.write_sif_dma_register(physical, value) { return Ok(()); }
        self.write_bytes(address, &value.to_le_bytes())
    }

    fn read_dmac_ext(&self, address: u32) -> Option<u32> {
        let offset = address.checked_sub(DMAC_EXT_BASE)? as usize;
        if offset >= DMAC_EXT_SIZE || offset % 4 != 0 {
            return None;
        }
        Some(self.dmac_ext[offset / 4])
    }

    fn write_dmac_ext(&mut self, address: u32, value: u32) -> bool {
        let Some(offset) = address.checked_sub(DMAC_EXT_BASE).map(|value| value as usize) else {
            return false;
        };
        if offset >= DMAC_EXT_SIZE || offset % 4 != 0 {
            return false;
        }
        self.dmac_ext[offset / 4] = value;
        true
    }

    fn read_sif_dma_register(&self, address: u32) -> Option<u32> {
        Some(match address {
            EE_VIF0_CHCR => self.vif0_chcr,
            EE_VIF0_MADR => self.vif0_madr,
            EE_VIF0_QWC => self.vif0_qwc,
            EE_VIF0_TADR => self.vif0_tadr,
            EE_VIF1_CHCR => self.vif1_chcr,
            EE_VIF1_MADR => self.vif1_madr,
            EE_VIF1_QWC => self.vif1_qwc,
            EE_VIF1_TADR => self.vif1_tadr,
            EE_GIF_CHCR => self.gif_chcr,
            EE_GIF_MADR => self.gif_madr,
            EE_GIF_QWC => self.gif_qwc,
            EE_GIF_TADR => self.gif_tadr,
            EE_GS_TO_EE_CHCR => self.gs_to_ee_chcr,
            EE_GS_TO_EE_MADR => self.gs_to_ee_madr,
            EE_GS_TO_EE_QWC => self.gs_to_ee_qwc,
            EE_GS_TO_EE_TADR => self.gs_to_ee_tadr,
            EE_SIF0_CHCR => self.sif0_chcr,
            EE_SIF0_MADR => self.sif0_madr,
            EE_SIF0_QWC => self.sif0_qwc,
            EE_SIF0_TADR => self.sif0_tadr,
            EE_SIF1_CHCR => self.sif1_chcr,
            EE_SIF1_MADR => self.sif1_madr,
            EE_SIF1_QWC => self.sif1_qwc,
            EE_SIF1_TADR => self.sif1_tadr,
            EE_SPR_FROM_CHCR => self.spr_from_chcr,
            EE_SPR_FROM_MADR => self.spr_from_madr,
            EE_SPR_FROM_QWC => self.spr_from_qwc,
            EE_SPR_FROM_SADR => self.spr_from_sadr,
            EE_SPR_TO_CHCR => self.spr_to_chcr,
            EE_SPR_TO_MADR => self.spr_to_madr,
            EE_SPR_TO_QWC => self.spr_to_qwc,
            EE_SPR_TO_TADR => self.spr_to_tadr,
            EE_SPR_TO_SADR => self.spr_to_sadr,
            _ => return None,
        })
    }

    fn write_sif_dma_register(&mut self, address: u32, value: u32) -> bool {
        match address {
            EE_VIF0_CHCR => self.vif0_chcr = value,
            EE_VIF0_MADR => self.vif0_madr = value & 0x7fff_fff0,
            EE_VIF0_QWC => self.vif0_qwc = value,
            EE_VIF0_TADR => self.vif0_tadr = value & 0x7fff_fff0,
            EE_VIF1_CHCR => self.vif1_chcr = value,
            EE_VIF1_MADR => self.vif1_madr = value & 0x7fff_fff0,
            EE_VIF1_QWC => self.vif1_qwc = value,
            EE_VIF1_TADR => self.vif1_tadr = value & 0x7fff_fff0,
            EE_GIF_CHCR => self.gif_chcr = value,
            EE_GIF_MADR => self.gif_madr = value & 0x7fff_fff0,
            EE_GIF_QWC => self.gif_qwc = value,
            EE_GIF_TADR => self.gif_tadr = value & 0x7fff_fff0,
            EE_GS_TO_EE_CHCR => self.gs_to_ee_chcr = value,
            EE_GS_TO_EE_MADR => self.gs_to_ee_madr = value & 0x7fff_fff0,
            EE_GS_TO_EE_QWC => self.gs_to_ee_qwc = value,
            EE_GS_TO_EE_TADR => self.gs_to_ee_tadr = value & 0x7fff_fff0,
            EE_SIF0_CHCR => {
                self.sif0_chcr = value;
                if value & (1 << 8) != 0 {
                    self.sif0_chain_end = false;
                    if (value >> 2) & 3 == 1 { self.sif0_qwc = 0; }
                }
            }
            EE_SIF0_MADR => self.sif0_madr = value & 0x7fff_fff0,
            EE_SIF0_QWC => self.sif0_qwc = value,
            EE_SIF0_TADR => self.sif0_tadr = value & 0x7fff_fff0,
            EE_SIF1_CHCR => {
                self.sif1_chcr = value;
                if value & (1 << 8) != 0 {
                    self.sif1_chain_words.clear();
                    self.sif1_chain_loaded = false;
                }
            }
            EE_SIF1_MADR => self.sif1_madr = value & 0x7fff_fff0,
            EE_SIF1_QWC => self.sif1_qwc = value,
            EE_SIF1_TADR => self.sif1_tadr = value & 0x7fff_fff0,
            EE_SPR_FROM_CHCR => self.spr_from_chcr = value,
            EE_SPR_FROM_MADR => self.spr_from_madr = value & 0x7fff_fff0,
            EE_SPR_FROM_QWC => self.spr_from_qwc = value & 0xffff,
            EE_SPR_FROM_SADR => self.spr_from_sadr = value & 0x3ff0,
            EE_SPR_TO_CHCR => self.spr_to_chcr = value,
            EE_SPR_TO_MADR => self.spr_to_madr = value & 0x7fff_fff0,
            EE_SPR_TO_QWC => self.spr_to_qwc = value & 0xffff,
            EE_SPR_TO_TADR => self.spr_to_tadr = value & 0x7fff_fff0,
            EE_SPR_TO_SADR => self.spr_to_sadr = value & 0x3ff0,
            _ => return false,
        }
        true
    }

    pub fn write_u64(&mut self, address: u32, value: u64) -> Result<(), MemoryError> {
        Self::check_alignment(address, 8)?;
        let physical = self.translate(address, true)?;
        if physical == GS_BUSDIR {
            self.gs_busdir = value & 1;
            return Ok(());
        }
        self.write_bytes(address, &value.to_le_bytes())
    }

    pub fn write_u128(&mut self, address: u32, value: u128) -> Result<(), MemoryError> {
        Self::check_alignment(address, 16)?;
        self.write_bytes(address, &value.to_le_bytes())
    }
}

fn tlb_page_size(page_mask: u32) -> u32 {
    (((page_mask & 0x01ff_e000) | 0x1fff).wrapping_add(1) / 2).max(0x1000)
}
