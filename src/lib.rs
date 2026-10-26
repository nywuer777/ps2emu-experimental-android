//! Minimal, explicit PS2 Emotion Engine research core.
//!
//! This crate currently loads little-endian ELF32/MIPS executables into a
//! 32 MiB EE memory model and an independent 2 MiB IOP memory slice. It
//! interprets documented scalar subsets for the R5900 and R3000A, including
//! EE quadword transfers and optional read-only BIOS ROM mappings. The GS
//! path is an initial GIF-to-software-framebuffer implementation. COP2 has a
//! partial VU0 macro-mode and VU0/VU1 micro-mode slices with VIF command/DMA
//! paths; SPU2 output is a bring-up model. `Ps2System` wires EE/IOP state
//! together with a shared SIF bus and an approximate instruction scheduler.

pub mod cpu;
pub mod cdvd;
pub mod elf;
pub mod gs;
pub mod iop;
pub mod memory;
pub mod sif;
pub mod sio2;
pub mod spu2;
pub mod system;
pub mod vif;
#[cfg(target_os = "android")]
pub mod android_ffi;

pub use cpu::{Cpu, CpuFault, CpuSnapshot};
pub use cdvd::{CdvdError, CdvdImage, IsoEntry, SectorFormat, ISO_SECTOR_SIZE};
pub use elf::{load_elf32_mips, load_elf32_mips_iop, ElfLoadError, LoadedElf};
pub use gs::{Gs, GsError, DEFAULT_GS_HEIGHT, DEFAULT_GS_WIDTH, GS_VRAM_SIZE};
pub use iop::{IopCpu, IopFault, IopMemory, IopMemoryError, IOP_RAM_SIZE, IOP_SCRATCH_BASE, IOP_SCRATCH_SIZE};
pub use memory::{Memory, MemoryError, TlbEntry, BIOS_ROM_BASE, BIOS_ROM_SIZE, EE_TLB_ENTRIES, INTC_MASK, INTC_STAT, MAIN_RAM_SIZE};
pub use sif::{SifBus, SifChannel, SIF_FIFO_CAPACITY_WORDS};
pub use sio2::{Sio2, SIO2_BASE, SIO2_REGISTER_SIZE};
pub use spu2::{Spu2, SPU2_BASE, SPU2_RAM_SIZE, SPU2_REGISTER_SIZE};
pub use system::{Ps2Error, Ps2System};
pub use vif::{Vif, VifError, VifEvent, VifReport};

