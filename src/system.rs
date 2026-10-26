//! Shared EE/IOP machine wrapper with a deterministic bring-up scheduler.

use crate::cdvd::{CdvdError, CdvdImage};
use crate::cpu::{Cpu, CpuFault};
use crate::elf::{load_elf32_mips, load_elf32_mips_iop, ElfLoadError, LoadedElf};
use crate::gs::{DEFAULT_GS_HEIGHT, DEFAULT_GS_WIDTH};
use crate::iop::{IopCpu, IopFault, IopMemory, IopMemoryError, IOP_CLOCK_HZ};
use crate::memory::{Memory, MemoryError, BIOS_ROM_SIZE};
use crate::sif::SifBus;
use std::fmt;

const EE_CLOCK_HZ: u64 = 294_912_000;

#[derive(Debug)]
pub enum Ps2Error {
    EeMemory(MemoryError),
    IopMemory(IopMemoryError),
    Elf(ElfLoadError),
    EeFault(CpuFault),
    IopFault(IopFault),
    Cdvd(CdvdError),
    Dma(String),
    InvalidBiosLength { processor: &'static str, length: usize },
}

impl fmt::Display for Ps2Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EeMemory(error) => write!(f, "EE memory: {error}"),
            Self::IopMemory(error) => write!(f, "IOP memory: {error}"),
            Self::Elf(error) => write!(f, "ELF load: {error}"),
            Self::EeFault(error) => write!(f, "EE stopped: {error}"),
            Self::IopFault(error) => write!(f, "IOP stopped: {error}"),
            Self::Dma(error) => write!(f, "EE DMA stopped: {error}"),
            Self::Cdvd(error) => write!(f, "CD/DVD image: {error}"),
            Self::InvalidBiosLength { processor, length } => write!(f, "invalid {processor} BIOS length: {length} bytes"),
        }
    }
}

impl std::error::Error for Ps2Error {}

impl From<MemoryError> for Ps2Error {
    fn from(value: MemoryError) -> Self { Self::EeMemory(value) }
}

impl From<IopMemoryError> for Ps2Error {
    fn from(value: IopMemoryError) -> Self { Self::IopMemory(value) }
}

impl From<ElfLoadError> for Ps2Error {
    fn from(value: ElfLoadError) -> Self { Self::Elf(value) }
}

impl From<CdvdError> for Ps2Error {
    fn from(value: CdvdError) -> Self { Self::Cdvd(value) }
}

/// EE/IOP state owner. It shares the SIF bus and advances the slower IOP at a
/// rational rate against the EE instruction-cycle approximation.
pub struct Ps2System {
    ee: Cpu,
    iop: IopCpu,
    ee_memory: Memory,
    iop_memory: IopMemory,
    iop_clock_phase: u64,
    scheduler_steps: u64,
    ee_pc_trace: [u32; 16],
    ee_instruction_trace: [u32; 16],
    ee_pc_trace_len: usize,
    ee_pc_trace_next: usize,
}

impl Default for Ps2System {
    fn default() -> Self { Self::new() }
}

impl Ps2System {
    pub fn new() -> Self {
        let sif = SifBus::new();
        Self {
            ee: Cpu::new(0xbfc0_0000),
            iop: IopCpu::new(0xbfc0_0000),
            ee_memory: Memory::with_sif(sif.clone()),
            iop_memory: IopMemory::with_sif(sif),
            iop_clock_phase: 0,
            scheduler_steps: 0,
            ee_pc_trace: [0; 16],
            ee_instruction_trace: [0; 16],
            ee_pc_trace_len: 0,
            ee_pc_trace_next: 0,
        }
    }

    /// Install separate EE and IOP firmware images. Smaller supported dumps
    /// are mirrored by their memory buses, matching the existing ROM model.
    pub fn load_bios(&mut self, ee_bios: &[u8], iop_bios: &[u8]) -> Result<(), Ps2Error> {
        Self::validate_bios("EE", ee_bios)?;
        Self::validate_bios("IOP", iop_bios)?;
        self.ee_memory.load_bios_rom(ee_bios)?;
        self.iop_memory.load_bios_rom(iop_bios)?;
        self.reset_cpus();
        Ok(())
    }

    pub fn mount_cdvd(&mut self, image: CdvdImage) {
        self.iop_memory.mount_cdvd(image);
    }

    /// Load the executable selected by SYSTEM.CNF and attach the same disc to
    /// the IOP CD/DVD device. This is a direct ELF bring-up path, not BIOS boot.
    pub fn load_disc_boot_elf(
        &mut self,
        image: CdvdImage,
    ) -> Result<Option<(String, LoadedElf)>, Ps2Error> {
        let boot = image.read_boot_executable()?;
        let loaded = match boot {
            Some((path, executable)) => Some((path, self.load_ee_elf(&executable)?)),
            None => None,
        };
        self.mount_cdvd(image);
        Ok(loaded)
    }

    pub fn load_ee_elf(&mut self, image: &[u8]) -> Result<LoadedElf, Ps2Error> {
        let loaded = load_elf32_mips(image, &mut self.ee_memory)?;
        self.ee_memory.reset_diagnostic_counters();
        self.ee = Cpu::new(loaded.entry);
        self.reset_ee_pc_trace();
        Ok(loaded)
    }

    pub fn load_iop_elf(&mut self, image: &[u8]) -> Result<LoadedElf, Ps2Error> {
        let loaded = load_elf32_mips_iop(image, &mut self.iop_memory)?;
        self.iop = IopCpu::new(loaded.entry);
        Ok(loaded)
    }

    /// Execute one EE instruction and any IOP instruction due at this clock point.
    pub fn step(&mut self) -> Result<(), Ps2Error> {
        let pc = self.ee.pc;
        let instruction = self.ee_memory.read_u32(pc).unwrap_or(0);
        self.ee_pc_trace[self.ee_pc_trace_next] = pc;
        self.ee_instruction_trace[self.ee_pc_trace_next] = instruction;
        self.ee_pc_trace_next = (self.ee_pc_trace_next + 1) % self.ee_pc_trace.len();
        self.ee_pc_trace_len = (self.ee_pc_trace_len + 1).min(self.ee_pc_trace.len());
        let cycles_before = self.ee.cpu_cycles;
        self.ee.step(&mut self.ee_memory).map_err(Ps2Error::EeFault)?;
        let elapsed = self.ee.cpu_cycles.wrapping_sub(cycles_before).max(1);
        self.iop_clock_phase = self.iop_clock_phase.saturating_add(elapsed.saturating_mul(IOP_CLOCK_HZ));
        while self.iop_clock_phase >= EE_CLOCK_HZ {
            self.iop.step(&mut self.iop_memory).map_err(Ps2Error::IopFault)?;
            self.iop_clock_phase -= EE_CLOCK_HZ;
        }
        self.scheduler_steps = self.scheduler_steps.wrapping_add(1);
        if let Some(error) = self.ee_memory.take_sif_dma_error() { return Err(Ps2Error::Dma(error)); }
        Ok(())
    }

    pub fn run_ee_steps(&mut self, count: u64) -> Result<(), Ps2Error> {
        for _ in 0..count { self.step()?; }
        Ok(())
    }

    /// Run a diagnostic budget, stopping after the first newly taken EE exception.
    pub fn run_ee_steps_until_exception(&mut self, count: u64) -> Result<u64, Ps2Error> {
        let initial_exceptions = self.ee.exceptions_taken;
        for completed in 0..count {
            self.step()?;
            if self.ee.exceptions_taken != initial_exceptions {
                return Ok(completed + 1);
            }
        }
        Ok(count)
    }

    /// Stop before executing the target EE instruction. Returns false if the
    /// budget expires; both CPUs and devices remain available for inspection.
    pub fn run_until_ee_pc(&mut self, target: u32, budget: u64) -> Result<bool, Ps2Error> {
        for _ in 0..budget {
            if self.ee.pc == target { return Ok(true); }
            self.step()?;
        }
        Ok(self.ee.pc == target)
    }

    pub fn reset_cpus(&mut self) {
        self.ee = Cpu::new(0xbfc0_0000);
        self.iop = IopCpu::new(0xbfc0_0000);
        self.iop_clock_phase = 0;
        self.scheduler_steps = 0;
        self.reset_ee_pc_trace();
    }

    pub fn ee(&self) -> &Cpu { &self.ee }
    pub fn iop(&self) -> &IopCpu { &self.iop }
    pub fn ee_memory(&self) -> &Memory { &self.ee_memory }
    pub fn ee_memory_mut(&mut self) -> &mut Memory { &mut self.ee_memory }
    pub fn iop_memory(&self) -> &IopMemory { &self.iop_memory }
    pub fn iop_memory_mut(&mut self) -> &mut IopMemory { &mut self.iop_memory }

    pub fn frame_rgba(&self) -> &[u32] { self.ee_memory.gs().framebuffer() }
    pub fn frame_dimensions(&self) -> (usize, usize) { (DEFAULT_GS_WIDTH, DEFAULT_GS_HEIGHT) }
    pub fn take_audio_samples(&mut self) -> Vec<i16> { self.iop_memory.take_audio_samples() }
    pub fn set_gamepad_buttons(&mut self, port: usize, buttons: u16) {
        self.iop_memory.set_gamepad_buttons(port, buttons);
    }
    pub fn set_gamepad_sticks(&mut self, port: usize, sticks: [u8; 4]) {
        self.iop_memory.set_gamepad_sticks(port, sticks);
    }
    pub fn scheduler_steps(&self) -> u64 { self.scheduler_steps }

    /// Most recently executed EE PCs, ordered oldest to newest.
    pub fn recent_ee_pcs(&self) -> Vec<u32> {
        let start = (self.ee_pc_trace_next + self.ee_pc_trace.len() - self.ee_pc_trace_len)
            % self.ee_pc_trace.len();
        (0..self.ee_pc_trace_len)
            .map(|index| self.ee_pc_trace[(start + index) % self.ee_pc_trace.len()])
            .collect()
    }

    /// Most recently fetched EE instructions, paired with their guest PCs.
    pub fn recent_ee_trace(&self) -> Vec<(u32, u32)> {
        let start = (self.ee_pc_trace_next + self.ee_pc_trace.len() - self.ee_pc_trace_len)
            % self.ee_pc_trace.len();
        (0..self.ee_pc_trace_len)
            .map(|index| {
                let slot = (start + index) % self.ee_pc_trace.len();
                (self.ee_pc_trace[slot], self.ee_instruction_trace[slot])
            })
            .collect()
    }

    fn reset_ee_pc_trace(&mut self) {
        self.ee_pc_trace = [0; 16];
        self.ee_instruction_trace = [0; 16];
        self.ee_pc_trace_len = 0;
        self.ee_pc_trace_next = 0;
    }

    fn validate_bios(processor: &'static str, image: &[u8]) -> Result<(), Ps2Error> {
        if image.is_empty() || image.len() > BIOS_ROM_SIZE || BIOS_ROM_SIZE % image.len() != 0 {
            return Err(Ps2Error::InvalidBiosLength { processor, length: image.len() });
        }
        Ok(())
    }
}
