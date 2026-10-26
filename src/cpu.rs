//! R5900 scalar interpreter slice with architectural branch delay slots.

use crate::memory::{Memory, MemoryError, TlbEntry, EE_TLB_ENTRIES, VU0_MEMORY_SIZE, VU1_MEMORY_SIZE};
use crate::vif::VifEvent;
use std::fmt;

#[derive(Debug, Clone, Copy)]
struct PendingBranch {
    target: u32,
}

#[derive(Debug, Clone)]
pub struct Cpu {
    /// PS2 GPRs are 128-bit. This initial scalar slice models the low 64-bit
    /// lane used by the implemented instructions and preserves the upper lane.
    pub gpr: [u128; 32],
    pub hi: u128,
    pub lo: u128,
    /// R5900 shift amount register used by QFSRV and MMI byte alignment.
    pub sa: u32,
    /// COP0 control registers, represented by their architecturally visible
    /// low 32-bit values in this initial system-control slice.
    pub cp0: [u32; 32],
    /// Scalar FPU registers store the IEEE-754 single-precision bit pattern.
    pub fpr: [u32; 32],
    pub fcr: [u32; 32],
    /// VU0 vector data registers, exposed for COP2 moves and quadword loads.
    pub vu0_data: [u128; 32],
    pub vu0_acc: u128,
    pub vu0_integer: [u16; 16],
    pub vu0_control: [u32; 32],
    vu_top: [u16; 2],
    vu_itop: [u16; 2],
    pub vu1_data: [u128; 32],
    pub vu1_acc: u128,
    pub vu1_integer: [u16; 16],
    pub vu1_control: [u32; 32],
    pub pc: u32,
    pending_branch: Option<PendingBranch>,
    pub retired_instructions: u64,
    pub exceptions_taken: u64,
    pub last_exception_instruction: u32,
    pub cpu_cycles: u64,
    count_subcycle: u8,
    pending_async_fault: Option<CpuFault>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CpuSnapshot {
    pub pc: u32,
    pub gpr: [u128; 32],
    pub hi: u128,
    pub lo: u128,
    pub sa: u32,
    pub cp0: [u32; 32],
    pub fpr: [u32; 32],
    pub fcr: [u32; 32],
    pub vu0_data: [u128; 32],
    pub vu0_acc: u128,
    pub vu0_integer: [u16; 16],
    pub vu0_control: [u32; 32],
    pub vu_top: [u16; 2],
    pub vu_itop: [u16; 2],
    pub vu1_data: [u128; 32],
    pub vu1_acc: u128,
    pub vu1_integer: [u16; 16],
    pub vu1_control: [u32; 32],
    pub retired_instructions: u64,
    pub exceptions_taken: u64,
    pub last_exception_instruction: u32,
    pub cpu_cycles: u64,
}

#[derive(Debug, Clone)]
pub enum CpuFault {
    Memory(MemoryError),
    AddressError { pc: u32, address: u32, store: bool },
    ArithmeticOverflow { pc: u32, instruction: u32 },
    Syscall { pc: u32, code: u32 },
    Break { pc: u32, code: u32 },
    BranchInDelaySlot { pc: u32, instruction: u32 },
    VuMicroBudgetExceeded { unit: u8, pc: u32, start_pc: u32, budget: usize },
    Graphics(String),
    ReservedInstruction { pc: u32, instruction: u32 },
}

impl fmt::Display for CpuFault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Memory(error) => write!(f, "{error}"),
            Self::AddressError { pc, address, store } => write!(f, "{} address error at {pc:#010x}: {address:#010x}", if *store { "store" } else { "load" }),
            Self::ArithmeticOverflow { pc, instruction } => write!(f, "integer overflow at {pc:#010x}, instruction {instruction:#010x}"),
            Self::Syscall { pc, code } => write!(f, "unhandled syscall {code:#x} at {pc:#010x}"),
            Self::Break { pc, code } => write!(f, "break {code:#x} at {pc:#010x}"),
            Self::BranchInDelaySlot { pc, instruction } => write!(f, "branch in delay slot at {pc:#010x}, instruction {instruction:#010x}"),
            Self::VuMicroBudgetExceeded { unit, pc, start_pc, budget } => write!(f, "VU{unit} microprogram at {start_pc:#x} exceeded {budget} bundles at EE {pc:#010x}"),
            Self::Graphics(message) => write!(f, "VU graphics transfer failed: {message}"),
            Self::ReservedInstruction { pc, instruction } => write!(f, "unsupported instruction {instruction:#010x} at {pc:#010x}"),
        }
    }
}

impl std::error::Error for CpuFault {}

impl From<MemoryError> for CpuFault {
    fn from(value: MemoryError) -> Self {
        Self::Memory(value)
    }
}

#[derive(Default)]
struct Effects {
    branch_target: Option<u32>,
    annul_delay_slot: bool,
    direct_pc: Option<u32>,
}

#[derive(Default)]
struct MmiEffects {
    gpr: Option<u128>,
    hi: Option<u128>,
    lo: Option<u128>,
}

impl MmiEffects {
    fn gpr(value: u128) -> Self {
        Self { gpr: Some(value), ..Self::default() }
    }
}

fn map_packed_lanes(a: u128, b: u128, lane_bits: u32, mut op: impl FnMut(u64, u64) -> u64) -> u128 {
    let lane_mask = (1u128 << lane_bits) - 1;
    let lane_count = 128 / lane_bits;
    let mut result = 0u128;
    for lane in 0..lane_count {
        let shift = lane * lane_bits;
        let left = ((a >> shift) & lane_mask) as u64;
        let right = ((b >> shift) & lane_mask) as u64;
        result |= ((op(left, right) as u128) & lane_mask) << shift;
    }
    result
}

fn accumulate_word_products(
    rs: u128,
    rt: u128,
    hi: u128,
    lo: u128,
    signed: bool,
    subtract: bool,
) -> MmiEffects {
    let mut result = 0u128;
    let mut next_hi = 0u128;
    let mut next_lo = 0u128;

    for lane in 0..2 {
        let word = lane * 2;
        let shift = word * 32;
        let lhs = (rs >> shift) as u32;
        let rhs = (rt >> shift) as u32;
        let product = if signed {
            ((lhs as i32 as i64).wrapping_mul(rhs as i32 as i64)) as u64
        } else {
            (lhs as u64).wrapping_mul(rhs as u64)
        };
        let accumulator = ((((hi >> (lane * 64)) as u32 as u64) << 32)
            | ((lo >> (lane * 64)) as u32 as u64)) as u64;
        let value = if subtract {
            accumulator.wrapping_sub(product)
        } else {
            accumulator.wrapping_add(product)
        };
        result |= (value as u128) << (lane * 64);
        next_lo |= (value as u32 as i32 as i64 as u64 as u128) << (lane * 64);
        next_hi |= ((value >> 32) as u32 as i32 as i64 as u64 as u128) << (lane * 64);
    }

    MmiEffects { gpr: Some(result), hi: Some(next_hi), lo: Some(next_lo) }
}

fn replace_word(value: u128, lane: usize, word: u32) -> u128 {
    let shift = lane * 32;
    (value & !((u32::MAX as u128) << shift)) | ((word as u128) << shift)
}

fn get_word(value: u128, lane: usize) -> u32 {
    (value >> (lane * 32)) as u32
}

fn halfword_accumulator_result(hi: u128, lo: u128) -> u128 {
    get_word(lo, 0) as u128
        | ((get_word(hi, 0) as u128) << 32)
        | ((get_word(lo, 2) as u128) << 64)
        | ((get_word(hi, 2) as u128) << 96)
}

fn execute_mmi2_half_accumulate(
    subop: u32,
    rs: u128,
    rt: u128,
    mut hi: u128,
    mut lo: u128,
) -> Option<MmiEffects> {
    match subop {
        16 | 20 => {
            for half in 0..8 {
                let shift = half * 16;
                let lhs = (rs >> shift) as u16 as i16 as i32;
                let rhs = (rt >> shift) as u16 as i16 as i32;
                let product = lhs.wrapping_mul(rhs) as u32;
                let group = half / 2;
                let word = (group / 2) * 2 + half % 2;
                if group % 2 == 0 {
                    let old = get_word(lo, word);
                    let value = if subop == 16 { old.wrapping_add(product) } else { old.wrapping_sub(product) };
                    lo = replace_word(lo, word, value);
                } else {
                    let old = get_word(hi, word);
                    let value = if subop == 16 { old.wrapping_add(product) } else { old.wrapping_sub(product) };
                    hi = replace_word(hi, word, value);
                }
            }
            Some(MmiEffects {
                gpr: Some(halfword_accumulator_result(hi, lo)),
                hi: Some(hi),
                lo: Some(lo),
            })
        }
        17 | 21 => {
            for pair in 0..4 {
                let half = pair * 2;
                let first_shift = half * 16;
                let second_shift = (half + 1) * 16;
                let first = (rs >> first_shift) as u16 as i16 as i32
                    * (rt >> first_shift) as u16 as i16 as i32;
                let second = (rs >> second_shift) as u16 as i16 as i32
                    * (rt >> second_shift) as u16 as i16 as i32;
                let group = pair;
                let word = (group / 2) * 2;
                let combined = if subop == 17 { second.wrapping_add(first) } else { second.wrapping_sub(first) };
                let trailing = if subop == 17 { second as u32 } else { !(second as u32) };
                if group % 2 == 0 {
                    lo = replace_word(lo, word, combined as u32);
                    lo = replace_word(lo, word + 1, trailing);
                } else {
                    hi = replace_word(hi, word, combined as u32);
                    hi = replace_word(hi, word + 1, trailing);
                }
            }
            Some(MmiEffects {
                gpr: Some(halfword_accumulator_result(hi, lo)),
                hi: Some(hi),
                lo: Some(lo),
            })
        }
        28 => {
            hi = 0;
            lo = 0;
            for half in 0..8 {
                let shift = half * 16;
                let lhs = (rs >> shift) as u16 as i16 as i32;
                let rhs = (rt >> shift) as u16 as i16 as i32;
                let product = lhs.wrapping_mul(rhs) as u32;
                let group = half / 2;
                let word = (group / 2) * 2 + half % 2;
                if group % 2 == 0 {
                    lo = replace_word(lo, word, product);
                } else {
                    hi = replace_word(hi, word, product);
                }
            }
            Some(MmiEffects {
                gpr: Some(halfword_accumulator_result(hi, lo)),
                hi: Some(hi),
                lo: Some(lo),
            })
        }
        _ => None,
    }
}

fn execute_mmi0(subop: u32, rs: u128, rt: u128) -> Option<u128> {
    Some(match subop {
        0 => map_packed_lanes(rs, rt, 32, |a, b| a.wrapping_add(b)), // PADDW
        1 => map_packed_lanes(rs, rt, 32, |a, b| a.wrapping_sub(b)), // PSUBW
        2 => map_packed_lanes(rs, rt, 32, |a, b| if (a as u32 as i32) > (b as u32 as i32) { u32::MAX as u64 } else { 0 }), // PCGTW
        3 => map_packed_lanes(rs, rt, 32, |a, b| (a as u32 as i32).max(b as u32 as i32) as u32 as u64), // PMAXW
        4 => map_packed_lanes(rs, rt, 16, |a, b| a.wrapping_add(b)), // PADDH
        5 => map_packed_lanes(rs, rt, 16, |a, b| a.wrapping_sub(b)), // PSUBH
        6 => map_packed_lanes(rs, rt, 16, |a, b| if (a as u16 as i16) > (b as u16 as i16) { u16::MAX as u64 } else { 0 }), // PCGTH
        7 => map_packed_lanes(rs, rt, 16, |a, b| (a as u16 as i16).max(b as u16 as i16) as u16 as u64), // PMAXH
        8 => map_packed_lanes(rs, rt, 8, |a, b| a.wrapping_add(b)), // PADDB
        9 => map_packed_lanes(rs, rt, 8, |a, b| a.wrapping_sub(b)), // PSUBB
        10 => map_packed_lanes(rs, rt, 8, |a, b| if (a as u8 as i8) > (b as u8 as i8) { u8::MAX as u64 } else { 0 }), // PCGTB
        16 => map_packed_lanes(rs, rt, 32, |a, b| (a as u32 as i32).saturating_add(b as u32 as i32) as u32 as u64), // PADDSW
        17 => map_packed_lanes(rs, rt, 32, |a, b| (a as u32 as i32).saturating_sub(b as u32 as i32) as u32 as u64), // PSUBSW
        18 => interleave_low(rs, rt, 32),
        19 => pack_even(rs, rt, 32),
        20 => map_packed_lanes(rs, rt, 16, |a, b| (a as u16 as i16).saturating_add(b as u16 as i16) as u16 as u64), // PADDSH
        21 => map_packed_lanes(rs, rt, 16, |a, b| (a as u16 as i16).saturating_sub(b as u16 as i16) as u16 as u64), // PSUBSH
        22 => interleave_low(rs, rt, 16),
        23 => pack_even(rs, rt, 16),
        24 => map_packed_lanes(rs, rt, 8, |a, b| (a as u8 as i8).saturating_add(b as u8 as i8) as u8 as u64), // PADDSB
        25 => map_packed_lanes(rs, rt, 8, |a, b| (a as u8 as i8).saturating_sub(b as u8 as i8) as u8 as u64), // PSUBSB
        26 => interleave_low(rs, rt, 8),
        27 => pack_even(rs, rt, 8),
        30 => map_packed_lanes(rt, 0, 32, |value, _| {
            let pixel = value as u32;
            (((pixel & 0x001f) << 3)
                | ((pixel & 0x03e0) << 6)
                | ((pixel & 0x7c00) << 9)
                | ((pixel & 0x8000) << 16)) as u64
        }), // PEXT5: expand PS2 RGBA5551 channels into spaced 8-bit lanes
        31 => map_packed_lanes(rt, 0, 32, |value, _| {
            let pixel = value as u32;
            (((pixel >> 3) & 0x001f)
                | ((pixel >> 6) & 0x03e0)
                | ((pixel >> 9) & 0x7c00)
                | ((pixel >> 16) & 0x8000)) as u64
        }), // PPAC5: pack spaced channels into PS2 RGBA5551
        _ => return None,
    })
}

fn interleave_segment(rs: u128, rt: u128, lane_bits: u32, start: u32, count: u32) -> u128 {
    let mask = (1u128 << lane_bits) - 1;
    let mut result = 0u128;
    for lane in 0..count {
        let source_shift = (start + lane) * lane_bits;
        let out_shift = lane * lane_bits * 2;
        result |= ((rt >> source_shift) & mask) << out_shift;
        result |= ((rs >> source_shift) & mask) << (out_shift + lane_bits);
    }
    result
}

fn permute_lanes(value: u128, lane_bits: u32, indexes: &[usize]) -> u128 {
    let mask = (1u128 << lane_bits) - 1;
    indexes.iter().enumerate().fold(0, |result, (out, &src)| {
        result | (((value >> (src as u32 * lane_bits)) & mask) << (out as u32 * lane_bits))
    })
}

fn execute_mmi1(subop: u32, rs: u128, rt: u128, sa: u32) -> Option<u128> {
    Some(match subop {
        1 => map_packed_lanes(rt, rt, 32, |a, _| {
            let value = a as u32 as i32;
            value.saturating_abs() as u32 as u64
        }), // PABSW
        2 => map_packed_lanes(rs, rt, 32, |a, b| if a as u32 == b as u32 { u32::MAX as u64 } else { 0 }), // PCEQW
        3 => map_packed_lanes(rs, rt, 32, |a, b| (a as u32 as i32).min(b as u32 as i32) as u32 as u64), // PMINW
        4 => {
            let mut result = 0u128;
            for lane in 0..8 {
                let a = ((rs >> (lane * 16)) & 0xffff) as u16 as i16;
                let b = ((rt >> (lane * 16)) & 0xffff) as u16 as i16;
                let value = if lane < 4 { a.wrapping_sub(b) as u16 } else { a.wrapping_add(b) as u16 };
                result |= (value as u128) << (lane * 16);
            }
            result
        } // PADSBH
        5 => map_packed_lanes(rt, rt, 16, |a, _| (a as u16 as i16).saturating_abs() as u16 as u64), // PABSH
        6 => map_packed_lanes(rs, rt, 16, |a, b| if a as u16 == b as u16 { u16::MAX as u64 } else { 0 }), // PCEQH
        7 => map_packed_lanes(rs, rt, 16, |a, b| (a as u16 as i16).min(b as u16 as i16) as u16 as u64), // PMINH
        10 => map_packed_lanes(rs, rt, 8, |a, b| if a as u8 == b as u8 { u8::MAX as u64 } else { 0 }), // PCEQB
        16 => map_packed_lanes(rs, rt, 32, |a, b| (a as u32).saturating_add(b as u32) as u64), // PADDUW
        17 => map_packed_lanes(rs, rt, 32, |a, b| (a as u32).saturating_sub(b as u32) as u64), // PSUBUW
        18 => interleave_segment(rs, rt, 32, 2, 2), // PEXTUW
        20 => map_packed_lanes(rs, rt, 16, |a, b| (a as u16).saturating_add(b as u16) as u64), // PADDUH
        21 => map_packed_lanes(rs, rt, 16, |a, b| (a as u16).saturating_sub(b as u16) as u64), // PSUBUH
        22 => interleave_segment(rs, rt, 16, 4, 4), // PEXTUH
        24 => map_packed_lanes(rs, rt, 8, |a, b| (a as u8).saturating_add(b as u8) as u64), // PADDUB
        25 => map_packed_lanes(rs, rt, 8, |a, b| (a as u8).saturating_sub(b as u8) as u64), // PSUBUB
        26 => interleave_segment(rs, rt, 8, 8, 8), // PEXTUB
        27 => qfsrv(rs, rt, sa),
        _ => return None,
    })
}

fn execute_mmi2(subop: u32, rs: u128, rt: u128, hi: u128, lo: u128) -> Option<MmiEffects> {
    if matches!(subop, 16 | 17 | 20 | 21 | 28) {
        return execute_mmi2_half_accumulate(subop, rs, rt, hi, lo);
    }
    Some(match subop {
        0 => return Some(accumulate_word_products(rs, rt, hi, lo, true, false)), // PMADDW
        4 => return Some(accumulate_word_products(rs, rt, hi, lo, true, true)), // PMSUBW
        2 => map_packed_lanes(rt, rs, 32, |value, shift| value.wrapping_shl((shift as u32) & 31)), // PSLLVW
        3 => map_packed_lanes(rt, rs, 32, |value, shift| value >> ((shift as u32) & 31)), // PSRLVW
        10 => {
            let mut result = 0;
            for lane in 0..4 {
                result |= ((rt >> (lane * 16)) & 0xffff) << (lane * 32);
                result |= ((rs >> ((lane + 4) * 16)) & 0xffff) << (lane * 32 + 16);
            }
            result
        } // PINTH
        8 => return Some(MmiEffects::gpr(hi)), // PMFHI
        9 => return Some(MmiEffects::gpr(lo)), // PMFLO
        12 => {
            let mut next_hi = 0u128;
            let mut next_lo = 0u128;
            let mut result = 0u128;
            for lane in 0..2 {
                let word = lane * 2;
                let a = (rs >> (word * 32)) as u32 as i32 as i64;
                let b = (rt >> (word * 32)) as u32 as i32 as i64;
                let product = a.wrapping_mul(b) as u64;
                next_lo |= (product as u32 as i32 as i64 as u64 as u128) << (lane * 64);
                next_hi |= ((product >> 32) as u32 as i32 as i64 as u64 as u128) << (lane * 64);
                result |= (product as u128) << (lane * 64);
            }
            return Some(MmiEffects { gpr: Some(result), hi: Some(next_hi), lo: Some(next_lo) });
        } // PMULTW
        13 => {
            let mut next_hi = 0u128;
            let mut next_lo = 0u128;
            for lane in 0..2 {
                let word = lane * 2;
                let dividend = (rs >> (word * 32)) as u32 as i32;
                let divisor = (rt >> (word * 32)) as u32 as i32;
                let (quotient, remainder) = if dividend == i32::MIN && divisor == -1 {
                    (i32::MIN, 0)
                } else if divisor == 0 {
                    (if dividend < 0 { 1 } else { -1 }, dividend)
                } else {
                    (dividend / divisor, dividend % divisor)
                };
                next_lo |= (quotient as i64 as u64 as u128) << (lane * 64);
                next_hi |= (remainder as i64 as u64 as u128) << (lane * 64);
            }
            return Some(MmiEffects { hi: Some(next_hi), lo: Some(next_lo), ..MmiEffects::default() });
        } // PDIVW
        29 => {
            let divisor = rt as u16 as i16 as i32;
            let mut next_hi = hi;
            let mut next_lo = lo;
            for lane in 0..4 {
                let dividend = get_word(rs, lane) as i32;
                let (quotient, remainder) = if dividend == i32::MIN && divisor == -1 {
                    (i32::MIN, 0)
                } else if divisor == 0 {
                    (if dividend < 0 { 1 } else { -1 }, dividend)
                } else {
                    (dividend / divisor, dividend % divisor)
                };
                next_lo = replace_word(next_lo, lane, quotient as u32);
                next_hi = replace_word(next_hi, lane, remainder as u32);
            }
            return Some(MmiEffects { hi: Some(next_hi), lo: Some(next_lo), ..MmiEffects::default() });
        } // PDIVBW
        14 => ((rs as u64 as u128) << 64) | rt as u64 as u128, // PCPYLD
        18 => rs & rt, // PAND
        19 => rs ^ rt, // PXOR
        26 => permute_lanes(rt, 16, &[2, 1, 0, 3, 6, 5, 4, 7]), // PEXEH
        27 => permute_lanes(rt, 16, &[3, 2, 1, 0, 7, 6, 5, 4]), // PREVH
        30 => permute_lanes(rt, 32, &[2, 1, 0, 3]), // PEXEW
        31 => permute_lanes(rt, 32, &[1, 2, 0, 3]), // PROT3W
        _ => return None,
    })
    .map(MmiEffects::gpr)
}

fn execute_mmi3(subop: u32, rs: u128, rt: u128, hi: u128, lo: u128) -> Option<MmiEffects> {
    Some(match subop {
        0 => return Some(accumulate_word_products(rs, rt, hi, lo, false, false)), // PMADDUW
        3 => map_packed_lanes(rt, rs, 32, |value, shift| {
            ((value as u32 as i32) >> ((shift as u32) & 31)) as u32 as u64
        }), // PSRAVW
        10 => {
            let mut result = 0;
            for lane in 0..4 {
                result |= ((rt >> (lane * 32)) & 0xffff) << (lane * 32);
                result |= ((rs >> (lane * 32)) & 0xffff) << (lane * 32 + 16);
            }
            result
        } // PINTEH
        8 => return Some(MmiEffects { hi: Some(rs), ..MmiEffects::default() }), // PMTHI
        9 => return Some(MmiEffects { lo: Some(rs), ..MmiEffects::default() }), // PMTLO
        12 => {
            let mut next_hi = 0u128;
            let mut next_lo = 0u128;
            let mut result = 0u128;
            for lane in 0..2 {
                let word = lane * 2;
                let product = ((rs >> (word * 32)) as u32 as u64)
                    .wrapping_mul((rt >> (word * 32)) as u32 as u64);
                next_lo |= (product as u32 as i32 as i64 as u64 as u128) << (lane * 64);
                next_hi |= ((product >> 32) as u32 as i32 as i64 as u64 as u128) << (lane * 64);
                result |= (product as u128) << (lane * 64);
            }
            return Some(MmiEffects { gpr: Some(result), hi: Some(next_hi), lo: Some(next_lo) });
        } // PMULTUW
        13 => {
            let mut next_hi = 0u128;
            let mut next_lo = 0u128;
            for lane in 0..2 {
                let word = lane * 2;
                let dividend = (rs >> (word * 32)) as u32;
                let divisor = (rt >> (word * 32)) as u32;
                let (quotient, remainder) = if divisor == 0 {
                    (u32::MAX, dividend)
                } else {
                    (dividend / divisor, dividend % divisor)
                };
                next_lo |= (quotient as i32 as i64 as u64 as u128) << (lane * 64);
                next_hi |= (remainder as i32 as i64 as u64 as u128) << (lane * 64);
            }
            return Some(MmiEffects { hi: Some(next_hi), lo: Some(next_lo), ..MmiEffects::default() });
        } // PDIVUW
        14 => ((rt >> 64) << 64) | (rs >> 64), // PCPYUD
        18 => rs | rt, // POR
        19 => !(rs | rt), // PNOR
        26 => permute_lanes(rt, 16, &[0, 2, 1, 3, 4, 6, 5, 7]), // PEXCH
        27 => {
            let low = (rt & 0xffff) as u128;
            let high = ((rt >> 64) & 0xffff) as u128;
            (0..4).fold(0, |result, lane| {
                result | (low << (lane * 16)) | (high << ((lane + 4) * 16))
            })
        } // PCPYH
        30 => permute_lanes(rt, 32, &[0, 2, 1, 3]), // PEXCW
        _ => return None,
    })
    .map(MmiEffects::gpr)
}

fn execute_mmi_shift_or_count(funct: u32, rs: u128, rt: u128, shift: u32) -> Option<u128> {
    Some(match funct {
        0x04 => map_packed_lanes(rs, 0, 32, |value, _| {
            let word = value as u32;
            let sign_fill = if word & 0x8000_0000 == 0 { 0 } else { u32::MAX };
            (word ^ sign_fill).leading_zeros().saturating_sub(1) as u64
        }), // PLZCW
        0x14 => map_packed_lanes(rt, 0, 16, |value, _| if shift >= 16 { 0 } else { (value as u16).wrapping_shl(shift) as u64 }), // PSLLH
        0x16 => map_packed_lanes(rt, 0, 16, |value, _| if shift >= 16 { 0 } else { ((value as u16) >> shift) as u64 }), // PSRLH
        0x17 => map_packed_lanes(rt, 0, 16, |value, _| ((value as u16 as i16) >> shift.min(15)) as u16 as u64), // PSRAH
        0x1c => map_packed_lanes(rt, 0, 32, |value, _| (value as u32).wrapping_shl(shift) as u64), // PSLLW
        0x1e => map_packed_lanes(rt, 0, 32, |value, _| ((value as u32) >> shift) as u64), // PSRLW
        0x1f => map_packed_lanes(rt, 0, 32, |value, _| ((value as u32 as i32) >> shift) as u32 as u64), // PSRAW
        _ => return None,
    })
}

fn execute_mmi_outer(
    funct: u32,
    rs: u128,
    rt: u128,
    rd_value: u128,
    shift: u32,
    hi: u128,
    lo: u128,
) -> Option<MmiEffects> {
    let low_64_mask = u64::MAX as u128;
    let hi_bank = (hi >> 64) as u64;
    let lo_bank = (lo >> 64) as u64;
    match funct {
        0x00 | 0x01 => {
            let accumulator = (((hi as u32) as u64) << 32) | lo as u32 as u64;
            let lhs = rs as u32;
            let rhs = rt as u32;
            let sum = if funct == 0x00 {
                accumulator.wrapping_add((lhs as i32 as i64).wrapping_mul(rhs as i32 as i64) as u64)
            } else {
                accumulator.wrapping_add((lhs as u64).wrapping_mul(rhs as u64))
            };
            let low_word = sum as u32;
            let high_word = (sum >> 32) as u32;
            let next_hi = (hi & !low_64_mask)
                | ((high_word as i32 as i64 as u64) as u128);
            let next_lo = (lo & !low_64_mask)
                | ((low_word as i32 as i64 as u64) as u128);
            let gpr_value = (rd_value & !low_64_mask)
                | (low_word as i32 as i64 as u64 as u128);
            Some(MmiEffects { gpr: Some(gpr_value), hi: Some(next_hi), lo: Some(next_lo) })
        } // MADD / MADDU
        0x10 => Some(MmiEffects::gpr((rd_value & !low_64_mask) | hi_bank as u128)), // MFHI1
        0x11 => Some(MmiEffects {
            hi: Some((hi & low_64_mask) | ((rs as u64 as u128) << 64)),
            ..MmiEffects::default()
        }), // MTHI1
        0x12 => Some(MmiEffects::gpr((rd_value & !low_64_mask) | lo_bank as u128)), // MFLO1
        0x13 => Some(MmiEffects {
            lo: Some((lo & low_64_mask) | ((rs as u64 as u128) << 64)),
            ..MmiEffects::default()
        }), // MTLO1
        0x18 | 0x19 | 0x1a | 0x1b | 0x20 | 0x21 => {
            let lhs = rs as u32;
            let rhs = rt as u32;
            let (next_hi_word, next_lo_word) = match funct {
                0x18 => {
                    let product = (lhs as i32 as i64).wrapping_mul(rhs as i32 as i64);
                    ((product >> 32) as u32, product as u32)
                } // MULT1
                0x19 => {
                    let product = (lhs as u64).wrapping_mul(rhs as u64);
                    ((product >> 32) as u32, product as u32)
                } // MULTU1
                0x1a => {
                    let dividend = lhs as i32;
                    let divisor = rhs as i32;
                    let (quotient, remainder) = if divisor == 0 {
                        (if dividend < 0 { 1 } else { -1 }, dividend)
                    } else if dividend == i32::MIN && divisor == -1 {
                        (i32::MIN, 0)
                    } else {
                        (dividend / divisor, dividend % divisor)
                    };
                    (remainder as u32, quotient as u32)
                } // DIV1
                0x1b => {
                    let (quotient, remainder) = if rhs == 0 {
                        (u32::MAX, lhs as i32 as u32)
                    } else {
                        (lhs / rhs, lhs % rhs)
                    };
                    (remainder, quotient)
                } // DIVU1
                0x20 | 0x21 => {
                    let accumulator = ((hi_bank as u32 as u64) << 32) | (lo_bank as u32 as u64);
                    let sum = if funct == 0x20 {
                        accumulator.wrapping_add((lhs as i32 as i64).wrapping_mul(rhs as i32 as i64) as u64)
                    } else {
                        accumulator.wrapping_add((lhs as u64).wrapping_mul(rhs as u64))
                    };
                    ((sum >> 32) as u32, sum as u32)
                } // MADD1 / MADDU1
                _ => unreachable!(),
            };
            let next_hi = (hi & low_64_mask)
                | (((next_hi_word as i32 as i64 as u64) as u128) << 64);
            let next_lo = (lo & low_64_mask)
                | (((next_lo_word as i32 as i64 as u64) as u128) << 64);
            let gpr_value = if matches!(funct, 0x18 | 0x19 | 0x20 | 0x21) {
                (rd_value & !low_64_mask) | next_lo_word as i32 as i64 as u64 as u128
            } else {
                rd_value
            };
            Some(MmiEffects { gpr: Some(gpr_value), hi: Some(next_hi), lo: Some(next_lo) })
        }
        0x30 => {
            let value = match shift {
                0 => {
                    let mut value = 0;
                    for (out, source) in [(0, (lo, 0)), (1, (hi, 0)), (2, (lo, 2)), (3, (hi, 2))] {
                        value = replace_word(value, out, get_word(source.0, source.1));
                    }
                    value
                } // PMFHL.LW
                1 => {
                    let mut value = 0;
                    for (out, source) in [(0, (lo, 1)), (1, (hi, 1)), (2, (lo, 3)), (3, (hi, 3))] {
                        value = replace_word(value, out, get_word(source.0, source.1));
                    }
                    value
                } // PMFHL.UW
                2 => {
                    let mut value = 0u128;
                    for lane in 0..2 {
                        let word = lane * 2;
                        let wide = ((get_word(hi, word) as u64) << 32) | get_word(lo, word) as u64;
                        let narrowed = (wide as i64).clamp(i32::MIN as i64, i32::MAX as i64) as i32 as i64 as u64;
                        value |= (narrowed as u128) << (lane * 64);
                    }
                    value
                } // PMFHL.SLW
                3 => {
                    let words = [get_word(lo, 0), get_word(lo, 1), get_word(hi, 0), get_word(hi, 1),
                        get_word(lo, 2), get_word(lo, 3), get_word(hi, 2), get_word(hi, 3)];
                    words.iter().enumerate().fold(0u128, |acc, (lane, word)| acc | ((*word as u16 as u128) << (lane * 16)))
                } // PMFHL.LH
                4 => {
                    let words = [get_word(lo, 0), get_word(lo, 1), get_word(hi, 0), get_word(hi, 1),
                        get_word(lo, 2), get_word(lo, 3), get_word(hi, 2), get_word(hi, 3)];
                    words.iter().enumerate().fold(0u128, |acc, (lane, word)| {
                        let narrowed = (*word as i32).clamp(i16::MIN as i32, i16::MAX as i32) as i16 as u16;
                        acc | ((narrowed as u128) << (lane * 16))
                    })
                } // PMFHL.SH
                _ => return None,
            };
            Some(MmiEffects::gpr(value))
        }
        0x31 if shift == 0 => {
            let mut next_hi = hi;
            let mut next_lo = lo;
            next_lo = replace_word(next_lo, 0, get_word(rs, 0));
            next_hi = replace_word(next_hi, 0, get_word(rs, 1));
            next_lo = replace_word(next_lo, 2, get_word(rs, 2));
            next_hi = replace_word(next_hi, 2, get_word(rs, 3));
            Some(MmiEffects { hi: Some(next_hi), lo: Some(next_lo), ..MmiEffects::default() })
        } // PMTHL.LW
        _ => execute_mmi_shift_or_count(funct, rs, rt, shift).map(MmiEffects::gpr),
    }
}

fn qfsrv(rs: u128, rt: u128, sa: u32) -> u128 {
    let shift = (sa & 0x0f) * 8;
    let words = [rt as u64, (rt >> 64) as u64, rs as u64, (rs >> 64) as u64];
    let index = (shift / 64) as usize;
    let bit_shift = shift % 64;
    let low = if bit_shift == 0 { words[index] } else { (words[index] >> bit_shift) | (words[index + 1] << (64 - bit_shift)) };
    let high = if bit_shift == 0 { words[index + 1] } else { (words[index + 1] >> bit_shift) | (words[index + 2] << (64 - bit_shift)) };
    ((high as u128) << 64) | low as u128
}

fn interleave_low(rs: u128, rt: u128, lane_bits: u32) -> u128 {
    let lane_mask = (1u128 << lane_bits) - 1;
    let lanes = 128 / lane_bits;
    let out_lanes = lanes / 2;
    let mut result = 0u128;
    for lane in 0..out_lanes {
        let shift = lane * lane_bits;
        let t = (rt >> shift) & lane_mask;
        let s = (rs >> shift) & lane_mask;
        result |= t << (shift * 2);
        result |= s << (shift * 2 + lane_bits);
    }
    result
}

fn pack_even(rs: u128, rt: u128, lane_bits: u32) -> u128 {
    let lane_mask = (1u128 << lane_bits) - 1;
    let lanes = 128 / lane_bits;
    let out_lanes = lanes / 2;
    let mut result = 0u128;
    for lane in 0..out_lanes {
        let t = (rt >> (lane * 2 * lane_bits)) & lane_mask;
        let s = (rs >> (lane * 2 * lane_bits)) & lane_mask;
        result |= t << (lane * lane_bits);
        result |= s << ((lane + out_lanes) * lane_bits);
    }
    result
}

impl Default for Cpu {
    fn default() -> Self {
        Self::new(0)
    }
}

impl Cpu {
    pub fn new(entry: u32) -> Self {
        let mut cp0 = [0; 32];
        cp0[1] = 47; // Random starts at the final R5900 TLB slot.
        cp0[12] = 0x7040_0004; // CP0 enabled, BEV set, reset/error level.
        cp0[15] = 0x0000_2e20; // R5900 processor ID.
        cp0[16] = 0x0000_0440; // R5900 configuration.
        let mut vu0_data = [0; 32];
        vu0_data[0] = 0x3f80_0000u128 << 96; // VF0 is the architectural (0, 0, 0, 1) constant.
        let mut vu1_data = [0; 32];
        vu1_data[0] = 0x3f80_0000u128 << 96;
        Self {
            gpr: [0; 32],
            hi: 0,
            lo: 0,
            sa: 0,
            cp0,
            fpr: [0; 32],
            fcr: [0; 32],
            vu0_data,
            vu0_acc: 0,
            vu0_integer: [0; 16],
            vu0_control: [0; 32],
            vu_top: [0; 2],
            vu_itop: [0; 2],
            vu1_data,
            vu1_acc: 0,
            vu1_integer: [0; 16],
            vu1_control: [0; 32],
            pc: entry,
            pending_branch: None,
            retired_instructions: 0,
            exceptions_taken: 0,
            last_exception_instruction: 0,
            cpu_cycles: 0,
            count_subcycle: 0,
            pending_async_fault: None,
        }
    }

    pub fn snapshot(&self) -> CpuSnapshot {
        CpuSnapshot {
            pc: self.pc,
            gpr: self.gpr,
            hi: self.hi,
            lo: self.lo,
            sa: self.sa,
            cp0: self.cp0,
            fpr: self.fpr,
            fcr: self.fcr,
            vu0_data: self.vu0_data,
            vu0_acc: self.vu0_acc,
            vu0_integer: self.vu0_integer,
            vu0_control: self.vu0_control,
            vu_top: self.vu_top,
            vu_itop: self.vu_itop,
            vu1_data: self.vu1_data,
            vu1_acc: self.vu1_acc,
            vu1_integer: self.vu1_integer,
            vu1_control: self.vu1_control,
            retired_instructions: self.retired_instructions,
            exceptions_taken: self.exceptions_taken,
            last_exception_instruction: self.last_exception_instruction,
            cpu_cycles: self.cpu_cycles,
        }
    }

    fn reg64(&self, index: u32) -> u64 {
        if index == 0 { 0 } else { self.gpr[index as usize] as u64 }
    }

    fn write64(&mut self, index: u32, value: u64) {
        if index != 0 {
            let upper = self.gpr[index as usize] & (u64::MAX as u128) << 64;
            self.gpr[index as usize] = upper | value as u128;
        }
        self.gpr[0] = 0;
    }

    fn write128(&mut self, index: u32, value: u128) {
        if index != 0 {
            self.gpr[index as usize] = value;
        }
        self.gpr[0] = 0;
    }

    fn write32_signed(&mut self, index: u32, value: u32) {
        self.write64(index, (value as i32 as i64) as u64);
    }

    pub fn step(&mut self, memory: &mut Memory) -> Result<(), CpuFault> {
        if let Some(fault) = self.pending_async_fault.take() { return Err(fault); }
        let pc = self.pc;
        let status = self.cp0[12];
        let wired = (self.cp0[6] & 0x3f).min((EE_TLB_ENTRIES - 1) as u32);
        let random = self.cp0[1] & 0x3f;
        self.cp0[1] = if random <= wired { (EE_TLB_ENTRIES - 1) as u32 } else { random - 1 };
        memory.set_user_mode(status & 0x6 == 0 && (status >> 3) & 3 == 2);
        memory.set_tlb_asid(self.cp0[10]);
        if memory.intc_irq_pending() {
            self.cp0[13] |= 1 << 10; // INTC INT0 is EE Cause.IP2.
        } else {
            self.cp0[13] &= !(1 << 10);
        }
        if self.interrupt_enabled() {
            self.enter_exception(0, pc, false, None);
            return Ok(());
        }
        if pc & 3 != 0 {
            self.enter_exception(4, pc, false, Some(pc));
            self.advance_cycles(1, memory);
            return Ok(());
        }
        let instruction = match memory.read_u32(pc) {
            Ok(instruction) => instruction,
            Err(error) => {
                let (code, bad_vaddr) = exception_for_memory_error(&error, false);
                self.enter_exception(code, pc, false, bad_vaddr);
                self.advance_cycles(1, memory);
                return Ok(());
            }
        };
        if self.pending_branch.is_some() && is_control_transfer(instruction) {
            self.enter_exception(10, pc, true, None);
            self.advance_cycles(1, memory);
            return Ok(());
        }
        let old_branch = self.pending_branch.take();
        let effects = match self.execute(instruction, pc, memory) {
            Ok(effects) => effects,
            Err(fault) => {
                let delay_slot = old_branch.is_some();
                let (code, bad_vaddr) = exception_for_fault(&fault, instruction);
                self.last_exception_instruction = instruction;
                self.enter_exception(code, pc, delay_slot, bad_vaddr);
                self.advance_cycles(1, memory);
                return Ok(());
            }
        };
        self.pc = if let Some(target) = effects.direct_pc {
            target
        } else if let Some(branch) = old_branch {
            branch.target
        } else if effects.annul_delay_slot {
            pc.wrapping_add(8)
        } else {
            pc.wrapping_add(4)
        };
        if old_branch.is_none() && effects.direct_pc.is_none() && !effects.annul_delay_slot {
            self.pending_branch = effects.branch_target.map(|target| PendingBranch { target });
        }
        self.gpr[0] = 0;
        self.retired_instructions = self.retired_instructions.wrapping_add(1);
        self.advance_cycles(1, memory);
        Ok(())
    }

    /// Assert or clear five external interrupt-pending inputs in Cause.IP2..IP6.
    pub fn set_interrupt_pending(&mut self, pending: u8) {
        self.cp0[13] = (self.cp0[13] & !0x0000_7c00) | (((pending & 0x1f) as u32) << 10);
    }

    fn interrupt_enabled(&self) -> bool {
        let status = self.cp0[12];
        status & 1 != 0
            && status & 0x6 == 0
            && self.cp0[13] & status & 0x0000_ff00 != 0
    }

    fn advance_cycles(&mut self, cycles: u8, memory: &mut Memory) {
        self.cpu_cycles = self.cpu_cycles.wrapping_add(cycles as u64);
        memory.advance_cycles(cycles as u64);
        for (unit, event) in memory.take_vif_events() {
            if unit > 1 { continue; }
            let (start, top, itop) = match event {
                VifEvent::StartMicroProgram { address, top, itop, .. } => (Some(address as u32), top, itop),
                VifEvent::ContinueMicroProgram { top, itop } => (Some(self.vu_control(unit, 26)), top, itop),
                VifEvent::Mark(_) => (None, 0, 0),
            };
            if let Some(start) = start {
                self.vu_top[unit as usize] = top;
                self.vu_itop[unit as usize] = itop;
                if let Err(fault) = self.execute_vu_micro(unit, memory, start, self.pc) {
                    if self.pending_async_fault.is_none() { self.pending_async_fault = Some(fault); }
                }
            }
        }
        self.count_subcycle = self.count_subcycle.saturating_add(cycles);
        while self.count_subcycle >= 2 {
            self.count_subcycle -= 2;
            self.cp0[9] = self.cp0[9].wrapping_add(1);
            if self.cp0[9] == self.cp0[11] {
                self.cp0[13] |= 1 << 15; // timer interrupt, Cause.IP7
            }
        }
    }

    pub fn run(&mut self, memory: &mut Memory, instruction_budget: u64) -> Result<(), CpuFault> {
        for _ in 0..instruction_budget {
            self.step(memory)?;
        }
        Ok(())
    }

    fn execute(&mut self, ins: u32, pc: u32, memory: &mut Memory) -> Result<Effects, CpuFault> {
        let op = ins >> 26;
        let rs = (ins >> 21) & 31;
        let rt = (ins >> 16) & 31;
        let rd = (ins >> 11) & 31;
        let sa = (ins >> 6) & 31;
        let imm = ins as u16 as i16 as i64;
        let imm_u = ins as u16 as u64;
        let mut effects = Effects::default();
        let a = self.reg64(rs);
        let b = self.reg64(rt);
        let address = a.wrapping_add(imm as u64) as u32;

        match op {
            0x11 => self.execute_cop1(ins, pc, &mut effects)?,
            0x10 => { // COP0: MFC0, MTC0, and ERET.
                let cop_rs = rs;
                match cop_rs {
                    0x00 => { // MFC0: 32-bit control register, sign-extended to GPR.
                        self.write32_signed(rt, self.cp0[rd as usize]);
                    }
                    0x04 => { // MTC0
                        let value = a as u32;
                        match rd {
                            0 => self.cp0[0] = value & 0x8000_003f,
                            1 | 8 | 15 => {}, // Random, BadVAddr and PRid are read-only.
                            4 => self.cp0[4] = (self.cp0[4] & 0x007f_fff0) | (value & 0xff80_000f),
                            5 => self.cp0[5] = value & 0x01ff_e000,
                            6 => {
                                self.cp0[6] = value & 0x3f;
                                self.cp0[1] = (EE_TLB_ENTRIES - 1) as u32;
                            }
                            10 => self.cp0[10] = value,
                            12 => self.cp0[12] = value,
                            13 => self.cp0[13] = (self.cp0[13] & !0x0000_0300) | (value & 0x0000_0300),
                            9 => self.cp0[9] = value,
                            11 => {
                                self.cp0[11] = value;
                                self.cp0[13] &= !(1 << 15);
                            }
                            14 | 30 => self.cp0[rd as usize] = value,
                            _ => self.cp0[rd as usize] = value,
                        }
                    }
                    0x10 if ins & 0x3f == 0x18 => { // ERET
                        let status = self.cp0[12];
                        if status & (1 << 2) != 0 { // ERL: ErrorEPC
                            effects.direct_pc = Some(self.cp0[30]);
                            self.cp0[12] = status & !(1 << 2);
                        } else { // EXL: EPC
                            effects.direct_pc = Some(self.cp0[14]);
                            self.cp0[12] = status & !(1 << 1);
                        }
                    }
                    0x10 => match ins & 0x3f {
                        0x01 => { // TLBR
                            let index = (self.cp0[0] & 0x3f) as usize;
                            if let Some(entry) = memory.tlb_entry(index) {
                                self.cp0[5] = entry.page_mask;
                                self.cp0[10] = entry.entry_hi;
                                self.cp0[2] = entry.entry_lo0;
                                self.cp0[3] = entry.entry_lo1;
                            }
                        }
                        0x02 | 0x06 => { // TLBWI / TLBWR
                            let index = if ins & 0x3f == 0x02 {
                                (self.cp0[0] & 0x3f) as usize
                            } else {
                                (self.cp0[1] & 0x3f) as usize
                            };
                            if index < EE_TLB_ENTRIES {
                                memory.write_tlb_entry(index, TlbEntry {
                                    page_mask: self.cp0[5],
                                    entry_hi: self.cp0[10],
                                    entry_lo0: self.cp0[2],
                                    entry_lo1: self.cp0[3],
                                });
                            }
                        }
                        0x08 => { // TLBP
                            self.cp0[0] = memory.probe_tlb(self.cp0[10])
                                .map(|index| index as u32)
                                .unwrap_or(0x8000_0000);
                        }
                        _ => return Err(CpuFault::ReservedInstruction { pc, instruction: ins }),
                    },
                    _ => return Err(CpuFault::ReservedInstruction { pc, instruction: ins }),
                }
            }
            0x00 => {
                match ins & 0x3f {
                    0x00 => self.write32_signed(rd, (b as u32).wrapping_shl(sa)), // SLL / NOP
                    0x02 => self.write32_signed(rd, (b as u32) >> sa),
                    0x03 => self.write32_signed(rd, ((b as u32 as i32) >> sa) as u32),
                    0x04 => self.write32_signed(rd, (b as u32).wrapping_shl((a as u32) & 31)),
                    0x06 => self.write32_signed(rd, (b as u32) >> ((a as u32) & 31)),
                    0x07 => self.write32_signed(rd, ((b as u32 as i32) >> ((a as u32) & 31)) as u32),
                    0x08 => effects.branch_target = Some(a as u32), // JR
                    0x09 => { // JALR
                        self.write64(rd, pc.wrapping_add(8) as u64);
                        effects.branch_target = Some(a as u32);
                    }
                    0x0a => { // MOVZ
                        if b == 0 { self.write64(rd, a); }
                    }
                    0x0b => { // MOVN
                        if b != 0 { self.write64(rd, a); }
                    }
                    0x0c => return Err(CpuFault::Syscall { pc, code: (ins >> 6) & 0xfffff }),
                    0x0d => return Err(CpuFault::Break { pc, code: (ins >> 6) & 0xfffff }),
                    0x0f => {} // SYNC is a no-op in this single-threaded slice.
                    0x28 => self.write64(rd, self.sa as u64), // MFSA
                    0x29 => self.sa = a as u32 & 0x7f, // MTSA
                    0x10 => self.write64(rd, self.hi as u64), // MFHI
                    0x11 => self.write_special_low64(true, a), // MTHI
                    0x12 => self.write64(rd, self.lo as u64), // MFLO
                    0x13 => self.write_special_low64(false, a), // MTLO
                    0x14 => self.write64(rd, b.wrapping_shl((a as u32) & 63)),
                    0x16 => self.write64(rd, b >> ((a as u32) & 63)),
                    0x17 => self.write64(rd, ((b as i64) >> ((a as u32) & 63)) as u64),
                    0x18 => { // MULT
                        let product = (a as u32 as i32 as i64) * (b as u32 as i32 as i64);
                        let lo = (product as u32 as i32 as i64) as u64;
                        let hi = ((product >> 32) as u32 as i32 as i64) as u64;
                        self.write_special_low64(false, lo);
                        self.write_special_low64(true, hi);
                        self.write64(rd, lo);
                    }
                    0x19 => { // MULTU, EE result lanes sign-extend from 32 bits.
                        let product = (a as u32 as u64) * (b as u32 as u64);
                        let lo = (product as u32 as i32 as i64) as u64;
                        let hi = ((product >> 32) as u32 as i32 as i64) as u64;
                        self.write_special_low64(false, lo);
                        self.write_special_low64(true, hi);
                        self.write64(rd, lo);
                    }
                    0x1a => { // DIV
                        let lhs = a as u32 as i32;
                        let rhs = b as u32 as i32;
                        let (quotient, remainder) = if rhs == 0 {
                            (if lhs < 0 { 1i32 } else { -1i32 }, lhs)
                        } else if lhs == i32::MIN && rhs == -1 {
                            (i32::MIN, 0)
                        } else {
                            (lhs / rhs, lhs % rhs)
                        };
                        self.write_special_low64(false, quotient as i64 as u64);
                        self.write_special_low64(true, remainder as i64 as u64);
                    }
                    0x1b => { // DIVU
                        let lhs = a as u32;
                        let rhs = b as u32;
                        let (quotient, remainder) = if rhs == 0 { (u32::MAX, lhs) } else { (lhs / rhs, lhs % rhs) };
                        self.write_special_low64(false, quotient as i32 as i64 as u64);
                        self.write_special_low64(true, remainder as i32 as i64 as u64);
                    }
                    0x20 | 0x21 => {
                        let result = (a as u32).wrapping_add(b as u32);
                        if ins & 0x3f == 0x20 {
                            if (a as u32 as i32).checked_add(b as u32 as i32).is_none() {
                                return Err(CpuFault::ArithmeticOverflow { pc, instruction: ins });
                            }
                        }
                        self.write32_signed(rd, result);
                    }
                    0x22 | 0x23 => {
                        let result = (a as u32).wrapping_sub(b as u32);
                        if ins & 0x3f == 0x22 && (a as u32 as i32).checked_sub(b as u32 as i32).is_none() {
                            return Err(CpuFault::ArithmeticOverflow { pc, instruction: ins });
                        }
                        self.write32_signed(rd, result);
                    }
                    0x24 => self.write64(rd, a & b),
                    0x25 => self.write64(rd, a | b),
                    0x26 => self.write64(rd, a ^ b),
                    0x27 => self.write64(rd, !(a | b)),
                    0x2a => self.write64(rd, ((a as i64) < (b as i64)) as u64),
                    0x2b => self.write64(rd, (a < b) as u64),
                    0x2c => {
                        if (a as i64).checked_add(b as i64).is_none() {
                            return Err(CpuFault::ArithmeticOverflow { pc, instruction: ins });
                        }
                        self.write64(rd, a.wrapping_add(b));
                    }
                    0x2d => self.write64(rd, a.wrapping_add(b)),
                    0x2e => {
                        if (a as i64).checked_sub(b as i64).is_none() {
                            return Err(CpuFault::ArithmeticOverflow { pc, instruction: ins });
                        }
                        self.write64(rd, a.wrapping_sub(b));
                    }
                    0x2f => self.write64(rd, a.wrapping_sub(b)),
                    0x38 => self.write64(rd, b.wrapping_shl(sa)),
                    0x3a => self.write64(rd, b >> sa),
                    0x3b => self.write64(rd, ((b as i64) >> sa) as u64),
                    0x3c => self.write64(rd, b.wrapping_shl(sa + 32)),
                    0x3e => self.write64(rd, b >> (sa + 32)),
                    0x3f => self.write64(rd, ((b as i64) >> (sa + 32)) as u64),
                    _ => return Err(CpuFault::ReservedInstruction { pc, instruction: ins }),
                }
            }
            0x02 | 0x03 => {
                if op == 0x03 {
                    self.write64(31, pc.wrapping_add(8) as u64);
                }
                effects.branch_target = Some(((pc.wrapping_add(4)) & 0xf000_0000) | ((ins & 0x03ff_ffff) << 2));
            }
            0x01 => { // REGIMM: signed branches and link variants.
                if rt == 0x18 { // MTSAB: byte-alignment shift amount.
                    self.sa = (a as u32).wrapping_add(imm as u32) & 0x0f;
                    return Ok(effects);
                }
                if rt == 0x19 { // MTSAH: halfword-alignment shift amount.
                    self.sa = ((a as u32).wrapping_add(imm as u32) & 0x07) << 1;
                    return Ok(effects);
                }
                let value = a as i64;
                let (valid, taken, likely, link) = match rt {
                    0x00 => (true, value < 0, false, false), // BLTZ
                    0x01 => (true, value >= 0, false, false), // BGEZ
                    0x02 => (true, value < 0, true, false), // BLTZL
                    0x03 => (true, value >= 0, true, false), // BGEZL
                    0x10 => (true, value < 0, false, true), // BLTZAL
                    0x11 => (true, value >= 0, false, true), // BGEZAL
                    0x12 => (true, value < 0, true, true), // BLTZALL
                    0x13 => (true, value >= 0, true, true), // BGEZALL
                    _ => (false, false, false, false),
                };
                if !valid {
                    return Err(CpuFault::ReservedInstruction { pc, instruction: ins });
                }
                // PCSX2's R5900 interpreter writes the link before testing
                // the predicate, including the likely-and-link forms.
                if link {
                    self.write64(31, pc.wrapping_add(8) as u64);
                }
                if likely && !taken {
                    effects.annul_delay_slot = true;
                } else {
                    let target = if taken {
                        pc.wrapping_add(4).wrapping_add(((imm as i32) << 2) as u32)
                    } else {
                        pc.wrapping_add(8)
                    };
                    effects.branch_target = Some(target);
                }
            }
            0x04 | 0x05 => {
                let taken = if op == 0x04 { a == b } else { a != b };
                let target = if taken {
                    pc.wrapping_add(4).wrapping_add(((imm as i32) << 2) as u32)
                } else {
                    pc.wrapping_add(8)
                };
                effects.branch_target = Some(target);
            }
            0x14 | 0x15 | 0x16 | 0x17 => {
                let taken = match op {
                    0x14 => a == b, // BEQL
                    0x15 => a != b, // BNEL
                    0x16 => (a as i64) <= 0, // BLEZL
                    _ => (a as i64) > 0, // BGTZL
                };
                if taken {
                    effects.branch_target = Some(pc.wrapping_add(4).wrapping_add(((imm as i32) << 2) as u32));
                } else {
                    effects.annul_delay_slot = true;
                }
            }
            0x06 | 0x07 => {
                let taken = if op == 0x06 { (a as i64) <= 0 } else { (a as i64) > 0 };
                let target = if taken {
                    pc.wrapping_add(4).wrapping_add(((imm as i32) << 2) as u32)
                } else {
                    pc.wrapping_add(8)
                };
                effects.branch_target = Some(target);
            }
            0x08 | 0x09 => {
                let lhs = a as i32;
                let rhs = imm as i32;
                let result = lhs.wrapping_add(rhs);
                if op == 0x08 && lhs.checked_add(rhs).is_none() {
                    return Err(CpuFault::ArithmeticOverflow { pc, instruction: ins });
                }
                self.write32_signed(rt, result as u32);
            }
            0x0a => self.write64(rt, ((a as i64) < imm) as u64),
            0x0b => self.write64(rt, (a < imm as u64) as u64),
            0x0c => self.write64(rt, a & imm_u),
            0x0d => self.write64(rt, a | imm_u),
            0x0e => self.write64(rt, a ^ imm_u),
            0x0f => self.write32_signed(rt, (ins & 0xffff) << 16),
            0x18 | 0x19 => {
                let result = a.wrapping_add(imm as u64);
                if op == 0x18 && (a as i64).checked_add(imm).is_none() {
                    return Err(CpuFault::ArithmeticOverflow { pc, instruction: ins });
                }
                self.write64(rt, result);
            }
            0x1c => { // MMI packed integer groups.
                let source_s = self.gpr[rs as usize];
                let source_t = self.gpr[rt as usize];
                let result = match ins & 0x3f {
                    0x08 => execute_mmi0(sa, source_s, source_t).map(MmiEffects::gpr),
                    0x09 => execute_mmi2(sa, source_s, source_t, self.hi, self.lo),
                    0x28 => execute_mmi1(sa, source_s, source_t, self.sa).map(MmiEffects::gpr),
                    0x29 => execute_mmi3(sa, source_s, source_t, self.hi, self.lo),
                    funct => execute_mmi_outer(
                        funct,
                        source_s,
                        source_t,
                        self.gpr[rd as usize],
                        sa,
                        self.hi,
                        self.lo,
                    ),
                }.ok_or(CpuFault::ReservedInstruction { pc, instruction: ins })?;
                if let Some(value) = result.gpr {
                    self.write128(rd, value);
                }
                if let Some(value) = result.hi {
                    self.hi = value;
                }
                if let Some(value) = result.lo {
                    self.lo = value;
                }
            }
            0x1e => { // LQ, EE quadword load.
                let value = memory.read_u128(address).map_err(|e| self.address_fault(e, pc, address, false))?;
                self.write128(rt, value);
            }
            0x12 => self.execute_cop2(ins, pc, &mut effects, memory)?,
            0x1a => { // LDL, little-endian merge into the high portion of a doubleword.
                let aligned = address & !7;
                let lane = address & 7;
                let shift = (7 - lane) * 8;
                let word = memory.read_u64(aligned)?;
                let old = self.reg64(rt);
                let preserve = if shift == 0 { 0 } else { (1u64 << shift) - 1 };
                self.write64(rt, (old & preserve) | word.wrapping_shl(shift));
            }
            0x1b => { // LDR, little-endian merge into the low portion of a doubleword.
                let aligned = address & !7;
                let lane = address & 7;
                let shift = lane * 8;
                let word = memory.read_u64(aligned)?;
                let preserve_shift = (8 - lane) * 8;
                let preserve = if preserve_shift == 64 { 0 } else { u64::MAX << preserve_shift };
                self.write64(rt, (self.reg64(rt) & preserve) | (word >> shift));
            }
            0x20 => { // LB
                let value = memory.read_u8(address)? as i8 as i64 as u64;
                self.write64(rt, value);
            }
            0x22 => { // LWL, little-endian merge into the high portion of the word.
                const MASK: [u32; 4] = [0x00ff_ffff, 0x0000_ffff, 0x0000_00ff, 0];
                const SHIFT: [u32; 4] = [24, 16, 8, 0];
                let aligned = address & !3;
                let lane = (address & 3) as usize;
                let word = memory.read_u32(aligned)?;
                let merged = ((self.reg64(rt) as u32) & MASK[lane]) | word.wrapping_shl(SHIFT[lane]);
                self.write32_signed(rt, merged);
            }
            0x21 => { // LH
                let value = memory.read_u16(address).map_err(|e| self.address_fault(e, pc, address, false))? as i16 as i64 as u64;
                self.write64(rt, value);
            }
            0x23 => { // LW
                let value = memory.read_u32(address).map_err(|e| self.address_fault(e, pc, address, false))? as i32 as i64 as u64;
                self.write64(rt, value);
            }
            0x26 => { // LWR, little-endian merge into the low portion of the word.
                const MASK: [u32; 4] = [0, 0xff00_0000, 0xffff_0000, 0xffff_ff00];
                const SHIFT: [u32; 4] = [0, 8, 16, 24];
                let aligned = address & !3;
                let lane = (address & 3) as usize;
                let word = memory.read_u32(aligned)?;
                let merged = ((self.reg64(rt) as u32) & MASK[lane]) | (word >> SHIFT[lane]);
                if lane == 0 {
                    self.write32_signed(rt, merged);
                } else {
                    self.write_low32_preserve_upper(rt, merged);
                }
            }
            0x24 => self.write64(rt, memory.read_u8(address)? as u64),
            0x25 => self.write64(rt, memory.read_u16(address).map_err(|e| self.address_fault(e, pc, address, false))? as u64),
            0x27 => self.write64(rt, memory.read_u32(address).map_err(|e| self.address_fault(e, pc, address, false))? as u64),
            0x37 => self.write64(rt, memory.read_u64(address).map_err(|e| self.address_fault(e, pc, address, false))?),
            0x36 => {
                let value = memory.read_u128(address)
                    .map_err(|e| self.address_fault(e, pc, address, false))?;
                if rt != 0 { self.vu0_data[rt as usize] = value; } // LQC2, VF0 stays constant
            }
            0x28 => memory.write_u8(address, b as u8)?,
            0x29 => memory.write_u16(address, b as u16).map_err(|e| self.address_fault(e, pc, address, true))?,
            0x2a => { // SWL
                const MASK: [u32; 4] = [0xffff_ff00, 0xffff_0000, 0xff00_0000, 0];
                const SHIFT: [u32; 4] = [24, 16, 8, 0];
                let aligned = address & !3;
                let lane = (address & 3) as usize;
                let old = memory.read_u32(aligned)?;
                let value = ((b as u32) >> SHIFT[lane]) | (old & MASK[lane]);
                memory.write_u32(aligned, value)?;
            }
            0x2b => memory.write_u32(address, b as u32).map_err(|e| self.address_fault(e, pc, address, true))?,
            0x2c => { // SDL, little-endian merge from the high portion of a doubleword.
                let aligned = address & !7;
                let lane = address & 7;
                let shift = (7 - lane) * 8;
                let old = memory.read_u64(aligned)?;
                let write_bits = (lane + 1) * 8;
                let write_mask = if write_bits == 64 { u64::MAX } else { (1u64 << write_bits) - 1 };
                let value = (old & !write_mask) | ((b >> shift) & write_mask);
                memory.write_u64(aligned, value).map_err(|e| self.address_fault(e, pc, address, true))?;
            }
            0x2d => { // SDR, little-endian merge from the low portion of a doubleword.
                let aligned = address & !7;
                let lane = address & 7;
                let shift = lane * 8;
                let old = memory.read_u64(aligned)?;
                let preserve_bits = lane * 8;
                let preserve_mask = if preserve_bits == 0 { 0 } else { (1u64 << preserve_bits) - 1 };
                let value = (old & preserve_mask) | (b.wrapping_shl(shift) & !preserve_mask);
                memory.write_u64(aligned, value).map_err(|e| self.address_fault(e, pc, address, true))?;
            }
            0x2e => { // SWR
                const MASK: [u32; 4] = [0, 0x0000_00ff, 0x0000_ffff, 0x00ff_ffff];
                const SHIFT: [u32; 4] = [0, 8, 16, 24];
                let aligned = address & !3;
                let lane = (address & 3) as usize;
                let old = memory.read_u32(aligned)?;
                let value = ((b as u32).wrapping_shl(SHIFT[lane])) | (old & MASK[lane]);
                memory.write_u32(aligned, value)?;
            }
            0x1f => memory.write_u128(address, self.gpr[rt as usize])
                .map_err(|e| self.address_fault(e, pc, address, true))?, // SQ
            0x3e => memory.write_u128(address, self.vu0_data[rt as usize])
                .map_err(|e| self.address_fault(e, pc, address, true))?, // SQC2
            0x3f => memory.write_u64(address, b).map_err(|e| self.address_fault(e, pc, address, true))?,
            _ => return Err(CpuFault::ReservedInstruction { pc, instruction: ins }),
        }
        Ok(effects)
    }

    fn execute_cop1(&mut self, ins: u32, pc: u32, effects: &mut Effects) -> Result<(), CpuFault> {
        let format = (ins >> 21) & 31;
        let rt = (ins >> 16) & 31;
        let fs = (ins >> 11) & 31;
        let fd = (ins >> 6) & 31;
        match format {
            0 => self.write32_signed(rt, self.fpr[fs as usize]), // MFC1
            2 => self.write32_signed(rt, self.fcr[fs as usize]), // CFC1
            4 => self.fpr[fs as usize] = self.reg64(rt) as u32, // MTC1
            6 => {
                if fs == 31 { self.fcr[31] = (self.reg64(rt) as u32) & 0x0183_ffff; }
            }
            8 => {
                let condition = self.fcr[31] & (1 << 23) != 0;
                let branch_on_true = rt & 1 != 0;
                let taken = condition == branch_on_true;
                let likely = rt & 2 != 0;
                if likely && !taken {
                    effects.annul_delay_slot = true;
                } else {
                    let offset = (ins as u16 as i16 as i32).wrapping_shl(2) as u32;
                    effects.branch_target = Some(if taken { pc.wrapping_add(4).wrapping_add(offset) } else { pc.wrapping_add(8) });
                }
            }
            16 => {
                let lhs = f32::from_bits(self.fpr[fs as usize]);
                let rhs = f32::from_bits(self.fpr[rt as usize]);
                let function = ins & 0x3f;
                match function {
                    0x00 => self.fpr[fd as usize] = (lhs + rhs).to_bits(),
                    0x01 => self.fpr[fd as usize] = (lhs - rhs).to_bits(),
                    0x02 => self.fpr[fd as usize] = (lhs * rhs).to_bits(),
                    0x03 => self.fpr[fd as usize] = (lhs / rhs).to_bits(),
                    0x04 => self.fpr[fd as usize] = lhs.sqrt().to_bits(),
                    0x05 => self.fpr[fd as usize] = lhs.abs().to_bits(),
                    0x06 => self.fpr[fd as usize] = self.fpr[fs as usize],
                    0x07 => self.fpr[fd as usize] = (-lhs).to_bits(),
                    0x0c..=0x0f | 0x24 => {
                        let rounded = match function {
                            0x0c => lhs.round(),
                            0x0d => lhs.trunc(),
                            0x0e => lhs.ceil(),
                            0x0f => lhs.floor(),
                            _ => match self.fcr[31] & 3 {
                                1 => lhs.trunc(),
                                2 => lhs.ceil(),
                                3 => lhs.floor(),
                                _ => lhs.round(),
                            },
                        };
                        let integer = if rounded.is_nan() { i32::MIN } else { rounded.clamp(i32::MIN as f32, i32::MAX as f32) as i32 };
                        self.fpr[fd as usize] = integer as u32;
                    }
                    0x30 | 0x32 | 0x3c | 0x3e => {
                        let condition = match function {
                            0x30 => false,
                            0x32 => lhs == rhs,
                            0x3c => lhs < rhs,
                            _ => lhs <= rhs,
                        };
                        if condition { self.fcr[31] |= 1 << 23; } else { self.fcr[31] &= !(1 << 23); }
                    }
                    _ => return Err(CpuFault::ReservedInstruction { pc, instruction: ins }),
                }
            }
            _ => return Err(CpuFault::ReservedInstruction { pc, instruction: ins }),
        }
        Ok(())
    }

    fn execute_cop2(&mut self, ins: u32, pc: u32, effects: &mut Effects, memory: &mut Memory) -> Result<(), CpuFault> {
        let operation = (ins >> 21) & 31;
        let rt = (ins >> 16) & 31;
        let rd = (ins >> 11) & 31;
        let element = ((ins >> 6) & 3) as usize;
        match operation {
            0 => {
                let value = (self.vu0_data[rd as usize] >> (element * 32)) as u32;
                self.write32_signed(rt, value);
            }
            1 => self.write128(rt, self.vu0_data[rd as usize]),
            2 => self.write32_signed(rt, self.vu0_control[rd as usize]),
            4 => {
                let shift = element * 32;
                let mask = !(u128::from(u32::MAX) << shift);
                if rd != 0 {
                    self.vu0_data[rd as usize] = (self.vu0_data[rd as usize] & mask)
                        | (u128::from(self.reg64(rt) as u32) << shift);
                }
            }
            5 => {
                if rd != 0 { self.vu0_data[rd as usize] = self.gpr[rt as usize]; }
            }
            6 => self.vu0_control[rd as usize] = self.reg64(rt) as u32,
            8 => {
                let condition = self.vu0_control[29] & (1 << 8) != 0;
                let branch_on_true = rt & 1 != 0;
                let taken = condition == branch_on_true;
                if rt & 2 != 0 && !taken {
                    effects.annul_delay_slot = true;
                } else {
                    let offset = (ins as u16 as i16 as i32).wrapping_shl(2) as u32;
                    effects.branch_target = Some(if taken { pc.wrapping_add(4).wrapping_add(offset) } else { pc.wrapping_add(8) });
                }
            }
            16..=31 => self.execute_vu_macro(0, ins, pc, memory)?,
            _ => return Err(CpuFault::ReservedInstruction { pc, instruction: ins }),
        }
        Ok(())
    }

    fn execute_vu_macro(&mut self, unit: u8, ins: u32, pc: u32, memory: &mut Memory) -> Result<(), CpuFault> {
        let function = ins & 0x3f;
        if function == 56 || function == 57 {
            let start = if function == 56 { (ins >> 6) & 0x7fff } else { self.vu_control(unit, 27) & 0x7fff };
            return self.execute_vu_micro(unit, memory, start, pc);
        }
        let fd = ((ins >> 6) & 0x1f) as usize;
        let fs = ((ins >> 11) & 0x1f) as usize;
        let ft = ((ins >> 16) & 0x1f) as usize;
        let id = fd & 0xf;
        let is = fs & 0xf;
        let it = ft & 0xf;
        match function {
            48 => self.write_vu_integer(unit, id, self.vu_integer(unit, is).wrapping_add(self.vu_integer(unit, it))),
            49 => self.write_vu_integer(unit, id, self.vu_integer(unit, is).wrapping_sub(self.vu_integer(unit, it))),
            50 => {
                let imm5 = ((ins >> 16) & 0x1f) as i16;
                let immediate = (imm5 << 11) >> 11;
                self.write_vu_integer(unit, id, self.vu_integer(unit, is).wrapping_add(immediate as u16));
            }
            52 => self.write_vu_integer(unit, id, self.vu_integer(unit, is) & self.vu_integer(unit, it)),
            53 => self.write_vu_integer(unit, id, self.vu_integer(unit, is) | self.vu_integer(unit, it)),
            _ => {}
        }
        if matches!(function, 48 | 49 | 50 | 52 | 53) { return Ok(()); }

        let mask = (ins >> 21) & 0xf;
        let source_s = self.vu_data(unit, fs);
        let source_t = self.vu_data(unit, ft);
        let mut destination = self.vu_data(unit, fd);
        let broadcast = match function {
            0..=3 | 8..=11 | 12..=15 | 16..=23 | 24..=27 => Some((function & 3) as usize),
            _ => None,
        };
        let base_operation = match function {
            0..=3 | 40 => Some(0), // VADD
            4..=7 | 44 => Some(1), // VSUB
            8..=11 | 41 => Some(2), // VMADD
            12..=15 | 45 => Some(3), // VMSUB
            16..=19 => Some(5), // VMAXx/y/z/w
            20..=23 => Some(6), // VMINIx/y/z/w
            24..=27 | 42 => Some(4), // VMUL
            43 => Some(5), // VMAX
            47 => Some(6), // VMINI
            _ => None,
        }.ok_or(CpuFault::ReservedInstruction { pc, instruction: ins })?;

        for lane in 0..4 {
            if mask & (1 << (3 - lane)) == 0 { continue; }
            let left = f32::from_bits((source_s >> (lane * 32)) as u32);
            let right_lane = broadcast.unwrap_or(lane);
            let right = f32::from_bits((source_t >> (right_lane * 32)) as u32);
            let acc = f32::from_bits((self.vu_acc(unit) >> (lane * 32)) as u32);
            let result = match base_operation {
                0 => left + right,
                1 => left - right,
                2 => acc + left * right,
                3 => acc - left * right,
                4 => left * right,
                5 => left.max(right),
                6 => left.min(right),
                _ => unreachable!(),
            };
            let shift = lane * 32;
            let lane_mask = u128::from(u32::MAX) << shift;
            destination = (destination & !lane_mask) | (u128::from(result.to_bits()) << shift);
        }
        self.set_vu_data(unit, fd, destination);
        Ok(())
    }

    /// Run the VU0 micro interpreter for a bounded number of 64-bit bundles.
    /// The VU has independent upper and lower pipelines; this first slice
    /// executes both in a deterministic order and models one branch delay slot.
    fn execute_vu_micro(&mut self, unit: u8, memory: &mut Memory, start_pc: u32, ee_pc: u32) -> Result<(), CpuFault> {
        let memory_size = if unit == 0 { VU0_MEMORY_SIZE as u32 } else { VU1_MEMORY_SIZE as u32 };
        let tpc_mask = (memory_size / 8) - 1;
        const BUDGET: usize = 4096;

        self.set_vu_control(unit, 26, start_pc & tpc_mask);
        let mut pending_branch: Option<u32> = None;
        let mut end_delay = false;

        for _ in 0..BUDGET {
            let micro_pc = self.vu_control(unit, 26) & tpc_mask;
            let lower = memory.vu_micro_word(unit, micro_pc * 8);
            let upper = memory.vu_micro_word(unit, micro_pc * 8 + 4);
            self.set_vu_control(unit, 26, micro_pc.wrapping_add(1) & tpc_mask);

            let branch_from_previous = pending_branch.take();
            if upper & (1 << 31) != 0 {
                // I-bit: execute the upper instruction, put the lower word in VI[I].
                self.execute_vu_micro_upper(unit, upper, ee_pc, memory)?;
                self.write_vu_integer(unit, 21, lower as u16);
            } else {
                self.execute_vu_micro_upper(unit, upper, ee_pc, memory)?;
                pending_branch = self.execute_vu_micro_lower(unit, lower, micro_pc, ee_pc, memory)?;
            }

            self.cpu_cycles = self.cpu_cycles.wrapping_add(1);

            // A prior branch takes effect after this bundle, preserving its delay slot.
            if let Some(target) = branch_from_previous {
                self.set_vu_control(unit, 26, target & tpc_mask);
            }

            if end_delay {
                return Ok(());
            }
            if upper & (1 << 30) != 0 {
                // E-bit ends after the following bundle.
                end_delay = true;
            }
        }

        Err(CpuFault::VuMicroBudgetExceeded { unit, pc: ee_pc, start_pc, budget: BUDGET })
    }

    fn execute_vu_micro_upper(&mut self, unit: u8, upper: u32, ee_pc: u32, memory: &mut Memory) -> Result<(), CpuFault> {
        let function = upper & 0x3f;
        // Micro upper words use the same vector operation fields as COP2 macros.
        if upper == 0 || upper == 0x8000_033c || function == 0x3c {
            return Ok(());
        }
        if matches!(function, 56 | 57) {
            return Err(CpuFault::ReservedInstruction { pc: ee_pc, instruction: upper });
        }
        self.execute_vu_macro(unit, upper, ee_pc, memory)
    }

    fn execute_vu_micro_lower(&mut self, unit: u8, lower: u32, micro_pc: u32, ee_pc: u32, memory: &mut Memory) -> Result<Option<u32>, CpuFault> {
        let opcode = lower >> 25;
        let id = ((lower >> 6) & 0x1f) as usize & 0xf;
        let is = ((lower >> 11) & 0x1f) as usize & 0xf;
        let it = ((lower >> 16) & 0x1f) as usize & 0xf;
        let imm11 = (((lower & 0x7ff) as i16) << 5 >> 5) as i32;
        let memory_size = if unit == 0 { VU0_MEMORY_SIZE as u32 } else { VU1_MEMORY_SIZE as u32 };
        let branch_target = || micro_pc.wrapping_add(1).wrapping_add(imm11 as u32) & ((memory_size / 8) - 1);
        let lane_mask = (lower >> 21) & 0xf;

        if lower == 0 || lower == 0x8000_033c {
            return Ok(None);
        }

        match opcode {
            0x00 | 0x01 => {
                let vector_register = it;
                let vector_mask = (lower >> 21) & 0xf;
                let vector_address = self.vu_integer(unit, is).wrapping_add(imm11 as u16) as u32;
                let memory_vector = memory
                    .read_vu_data_vector(unit, vector_address)
                    .ok_or(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower })?;
                if opcode == 0x00 {
                    let mut destination = self.vu_data(unit, vector_register);
                    for lane in 0..4 {
                        if vector_mask & (1 << (3 - lane)) != 0 {
                            let shift = lane * 32;
                            let lane_mask = u128::from(u32::MAX) << shift;
                            destination = (destination & !lane_mask)
                                | (u128::from(memory_vector[lane]) << shift);
                        }
                    }
                    self.set_vu_data(unit, vector_register, destination);
                } else {
                    let source = self.vu_data(unit, vector_register);
                    let mut stored = memory_vector;
                    for lane in 0..4 {
                        if vector_mask & (1 << (3 - lane)) != 0 {
                            stored[lane] = (source >> (lane * 32)) as u32;
                        }
                    }
                    memory.write_vu_data_vector(unit, vector_address, stored);
                }
            }
            // Lower-op field type 1: VI integer ALU operations.
            0x40 => match lower & 0x3f {
                0x3c if ((lower >> 6) & 0x1f) == 0x1a => {
                    if unit != 1 {
                        return Err(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower });
                    }
                    self.write_vu_integer(unit, it, self.vu_top[unit as usize]);
                }
                0x3d if ((lower >> 6) & 0x1f) == 0x1a => {
                    if unit != 1 {
                        return Err(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower });
                    }
                    self.write_vu_integer(unit, it, self.vu_itop[unit as usize]);
                }
                0x3c if ((lower >> 6) & 0x1f) == 0x0d => {
                    let address = u32::from(self.vu_integer(unit, is));
                    let loaded = memory
                        .read_vu_data_vector(unit, address)
                        .ok_or(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower })?;
                    let mut destination = self.vu_data(unit, it);
                    for component in 0..4 {
                        if lane_mask & (1 << (3 - component)) != 0 {
                            let shift = component * 32;
                            let field_mask = u128::from(u32::MAX) << shift;
                            destination = (destination & !field_mask)
                                | (u128::from(loaded[component]) << shift);
                        }
                    }
                    self.set_vu_data(unit, it, destination);
                    if is != 0 {
                        self.write_vu_integer(unit, is, self.vu_integer(unit, is).wrapping_add(1));
                    }
                }
                0x3e if ((lower >> 6) & 0x1f) == 0x0d => {
                    if is != 0 {
                        self.write_vu_integer(unit, is, self.vu_integer(unit, is).wrapping_sub(1));
                    }
                    let address = u32::from(self.vu_integer(unit, is));
                    let loaded = memory
                        .read_vu_data_vector(unit, address)
                        .ok_or(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower })?;
                    let mut destination = self.vu_data(unit, it);
                    for component in 0..4 {
                        if lane_mask & (1 << (3 - component)) != 0 {
                            let shift = component * 32;
                            let field_mask = u128::from(u32::MAX) << shift;
                            destination = (destination & !field_mask)
                                | (u128::from(loaded[component]) << shift);
                        }
                    }
                    self.set_vu_data(unit, it, destination);
                }
                0x3d if ((lower >> 6) & 0x1f) == 0x0d => {
                    let base = it;
                    let address = u32::from(self.vu_integer(unit, base));
                    let mut stored = memory
                        .read_vu_data_vector(unit, address)
                        .ok_or(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower })?;
                    let source = self.vu_data(unit, is);
                    for component in 0..4 {
                        if lane_mask & (1 << (3 - component)) != 0 {
                            stored[component] = (source >> (component * 32)) as u32;
                        }
                    }
                    memory.write_vu_data_vector(unit, address, stored);
                    if base != 0 {
                        self.write_vu_integer(unit, base, self.vu_integer(unit, base).wrapping_add(1));
                    }
                }
                0x3f if ((lower >> 6) & 0x1f) == 0x0d => {
                    let base = it;
                    if base != 0 {
                        self.write_vu_integer(unit, base, self.vu_integer(unit, base).wrapping_sub(1));
                    }
                    let address = u32::from(self.vu_integer(unit, base));
                    let mut stored = memory
                        .read_vu_data_vector(unit, address)
                        .ok_or(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower })?;
                    let source = self.vu_data(unit, is);
                    for component in 0..4 {
                        if lane_mask & (1 << (3 - component)) != 0 {
                            stored[component] = (source >> (component * 32)) as u32;
                        }
                    }
                    memory.write_vu_data_vector(unit, address, stored);
                }
                0x3c if ((lower >> 6) & 0x1f) == 0x1b => {
                    if unit != 1 {
                        return Err(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower });
                    }
                    submit_vu1_xgkick(memory, self.vu_integer(unit, is) as u32)?;
                }
                0x3e if ((lower >> 6) & 0x1f) == 0x0f => {
                    if it != 0 {
                        let component = selected_vu_component(lane_mask)
                            .ok_or(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower })?;
                        let address = u32::from(self.vu_integer(unit, is)) * 16 + component as u32 * 4;
                        self.write_vu_integer(unit, it, memory.vu_data_halfword(unit, address));
                    }
                }
                0x3f if ((lower >> 6) & 0x1f) == 0x0f => {
                    let component = selected_vu_component(lane_mask)
                        .ok_or(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower })?;
                    let address = u32::from(self.vu_integer(unit, is)) * 16;
                    let mut vector = memory
                        .read_vu_data_vector(unit, self.vu_integer(unit, is) as u32)
                        .ok_or(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower })?;
                    vector[component] = ((self.vu_data(unit, it) >> (component * 32)) as u16) as u32;
                    memory.write_vu_data_vector(unit, address / 16, vector);
                }
                0x3d if ((lower >> 6) & 0x1f) == 0x0f => {
                    let value = (self.vu_integer(unit, is) as i16 as i32) as u32;
                    let mut vector = self.vu_data(unit, it);
                    for component in 0..4 {
                        if lane_mask & (1 << (3 - component)) != 0 {
                            let shift = component * 32;
                            let field_mask = u128::from(u32::MAX) << shift;
                            vector = (vector & !field_mask) | (u128::from(value) << shift);
                        }
                    }
                    self.set_vu_data(unit, it, vector);
                }
                0x3c if ((lower >> 6) & 0x1f) == 0x0f => {
                    if (lower >> 23) & 3 != 0 {
                        return Err(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower });
                    }
                    let component = ((lower >> 21) & 3) as usize;
                    let value = (self.vu_data(unit, is) >> (component * 32)) as u16;
                    self.write_vu_integer(unit, it, value);
                }
                0x30 => self.write_vu_integer(unit, id, self.vu_integer(unit, is).wrapping_add(self.vu_integer(unit, it))),
                0x31 => self.write_vu_integer(unit, id, self.vu_integer(unit, is).wrapping_sub(self.vu_integer(unit, it))),
                0x32 => {
                    let immediate = ((((lower >> 6) & 0x1f) as i16) << 11) >> 11;
                    let source = ((lower >> 11) & 0x1f) as usize & 0xf;
                    let destination = ((lower >> 16) & 0x1f) as usize & 0xf;
                    self.write_vu_integer(unit, destination, self.vu_integer(unit, source).wrapping_add(immediate as u16));
                }
                0x34 => self.write_vu_integer(unit, id, self.vu_integer(unit, is) & self.vu_integer(unit, it)),
                0x35 => self.write_vu_integer(unit, id, self.vu_integer(unit, is) | self.vu_integer(unit, it)),
                _ => return Err(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower }),
            },
            // Lower-op field type 7: direct and integer conditional branches.
            0x20 => return Ok(Some(branch_target())), // B
            0x21 => {
                self.write_vu_integer(unit, it, micro_pc.wrapping_mul(8).wrapping_add(16) as u16);
                return Ok(Some(branch_target()));
            }
            0x04 | 0x05 => {
                let lane = selected_vu_component(lane_mask)
                    .ok_or(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower })?;
                let address = ((self.vu_integer(unit, is) as i32 + imm11).wrapping_mul(16) as u32)
                    .wrapping_add(lane as u32 * 4);
                if opcode == 0x04 {
                    self.write_vu_integer(unit, it, memory.vu_data_halfword(unit, address));
                } else {
                    memory.set_vu_data_halfword(unit, address, self.vu_integer(unit, it));
                }
            }
            0x28 if self.vu_integer(unit, is) == self.vu_integer(unit, it) => return Ok(Some(branch_target())), // IBEQ
            0x29 if self.vu_integer(unit, is) != self.vu_integer(unit, it) => return Ok(Some(branch_target())), // IBNE
            0x2c if (self.vu_integer(unit, is) as i16) <= 0 => return Ok(Some(branch_target())), // IBLEZ
            0x2d if (self.vu_integer(unit, is) as i16) < 0 => return Ok(Some(branch_target())), // IBLTZ
            0x2e if (self.vu_integer(unit, is) as i16) > 0 => return Ok(Some(branch_target())), // IBGTZ
            0x2f if (self.vu_integer(unit, is) as i16) >= 0 => return Ok(Some(branch_target())), // IBGEZ
            _ => return Err(CpuFault::ReservedInstruction { pc: ee_pc, instruction: lower }),
        }
        Ok(None)
    }

    fn vu_data(&self, unit: u8, index: usize) -> u128 {
        if unit == 0 { self.vu0_data[index] } else { self.vu1_data[index] }
    }

    fn set_vu_data(&mut self, unit: u8, index: usize, value: u128) {
        let registers = if unit == 0 { &mut self.vu0_data } else { &mut self.vu1_data };
        if index != 0 { registers[index] = value; }
        registers[0] = 0x3f80_0000u128 << 96;
    }

    fn vu_acc(&self, unit: u8) -> u128 {
        if unit == 0 { self.vu0_acc } else { self.vu1_acc }
    }

    fn vu_integer(&self, unit: u8, index: usize) -> u16 {
        if unit == 0 { self.vu0_integer[index] } else { self.vu1_integer[index] }
    }

    fn vu_control(&self, unit: u8, index: usize) -> u32 {
        if unit == 0 { self.vu0_control[index] } else { self.vu1_control[index] }
    }

    fn set_vu_control(&mut self, unit: u8, index: usize, value: u32) {
        if unit == 0 { self.vu0_control[index] = value; } else { self.vu1_control[index] = value; }
    }

    fn write_vu_integer(&mut self, unit: u8, index: usize, value: u16) {
        let registers = if unit == 0 { &mut self.vu0_integer } else { &mut self.vu1_integer };
        if index != 0 { registers[index] = value; }
        registers[0] = 0;
    }

    fn address_fault(&self, error: MemoryError, pc: u32, address: u32, store: bool) -> CpuFault {
        match error {
            MemoryError::Misaligned { .. } => CpuFault::AddressError { pc, address, store },
            other => CpuFault::Memory(other),
        }
    }

    fn write_low32_preserve_upper(&mut self, index: u32, value: u32) {
        if index != 0 {
            let upper = self.gpr[index as usize] & !(u32::MAX as u128);
            self.gpr[index as usize] = upper | value as u128;
        }
        self.gpr[0] = 0;
    }

    fn write_special_low64(&mut self, high: bool, value: u64) {
        let register = if high { &mut self.hi } else { &mut self.lo };
        *register = (*register & ((u64::MAX as u128) << 64)) | value as u128;
    }

    fn enter_exception(&mut self, code: u32, pc: u32, delay_slot: bool, bad_vaddr: Option<u32>) {
        let status = self.cp0[12];
        if let Some(address) = bad_vaddr {
            self.cp0[8] = address;
            if matches!(code, 1 | 2 | 3) {
                self.cp0[4] = (self.cp0[4] & 0xff80_000f) | ((address >> 9) & 0x007f_fff0);
                self.cp0[10] = (address & 0xffff_e000) | (self.cp0[10] & 0x1fff);
            }
        }
        let mut cause = self.cp0[13] & !(0x8000_007c);
        cause |= (code & 0x1f) << 2;
        if delay_slot {
            cause |= 1 << 31;
        }
        self.cp0[13] = cause;

        if status & (1 << 1) == 0 {
            self.cp0[14] = if delay_slot { pc.wrapping_sub(4) } else { pc };
        }
        self.cp0[12] = status | (1 << 1); // EXL
        let offset = if status & (1 << 1) != 0 {
            0x180 // Nested exception uses the general vector.
        } else if code == 0 {
            0x200 // Interrupt vector.
        } else if matches!(code, 2 | 3) {
            0 // TLB refill vector.
        } else {
            0x180
        };
        let base = if status & (1 << 22) != 0 { 0xbfc0_0200 } else { 0x8000_0000 };
        self.pc = base + offset;
        self.pending_branch = None;
        self.exceptions_taken = self.exceptions_taken.wrapping_add(1);
    }
}

fn exception_for_fault(fault: &CpuFault, instruction: u32) -> (u32, Option<u32>) {
    match fault {
        CpuFault::AddressError { address, store, .. } => (if *store { 5 } else { 4 }, Some(*address)),
        CpuFault::ArithmeticOverflow { .. } => (12, None),
        CpuFault::Syscall { .. } => (8, None),
        CpuFault::Break { .. } => (9, None),
        CpuFault::BranchInDelaySlot { .. }
        | CpuFault::VuMicroBudgetExceeded { .. }
        | CpuFault::Graphics(_)
        | CpuFault::ReservedInstruction { .. } => (10, None),
        CpuFault::Memory(error) => {
            let op = instruction >> 26;
            let store = matches!(op, 0x28..=0x2b | 0x1f | 0x3f);
            exception_for_memory_error(error, store)
        }
    }
}

fn exception_for_memory_error(error: &MemoryError, store: bool) -> (u32, Option<u32>) {
    match error {
        MemoryError::TlbMiss { address, store } | MemoryError::TlbInvalid { address, store } => {
            (if *store { 3 } else { 2 }, Some(*address))
        }
        MemoryError::TlbModified { address } => (1, Some(*address)),
        MemoryError::Privilege { address, store } => (if *store { 5 } else { 4 }, Some(*address)),
        MemoryError::Unmapped { address, .. }
        | MemoryError::Misaligned { address, .. }
        | MemoryError::OutOfBounds { address, .. } => (if store { 5 } else { 4 }, Some(*address)),
        MemoryError::ReadOnly { address } => (5, Some(*address)),
    }
}

/// Forward one VU1 XGKICK stream from VU data memory to the shared GIF/GS
/// decoder. Addresses and packet lengths are measured in 128-bit qwords.
fn submit_vu1_xgkick(memory: &mut Memory, start_qword: u32) -> Result<(), CpuFault> {
    const VU1_QWORDS: usize = VU1_MEMORY_SIZE / 16;
    let mut packet = Vec::new();
    let mut cursor = 0usize;
    let mut ended = false;

    while cursor < VU1_QWORDS {
        let tag = memory
            .read_vu_data_vector(1, start_qword.wrapping_add(cursor as u32))
            .ok_or_else(|| CpuFault::Graphics(String::from("VU1 data memory is unavailable")))?;
        let tag = u128::from(tag[0])
            | (u128::from(tag[1]) << 32)
            | (u128::from(tag[2]) << 64)
            | (u128::from(tag[3]) << 96);
        packet.push(tag);
        cursor += 1;

        let loops = (tag & 0x7fff) as usize;
        let mode = ((tag >> 58) & 3) as usize;
        let registers = match ((tag >> 60) & 0xf) as usize { 0 => 16, count => count };
        let payload_qwords = match mode {
            0 => loops.checked_mul(registers),
            1 => loops.checked_mul(registers).map(|items| items.div_ceil(2)),
            2 | 3 => Some(loops),
            _ => unreachable!(),
        }
        .ok_or_else(|| CpuFault::Graphics(String::from("GIF tag payload length overflow")))?;
        let end = cursor.checked_add(payload_qwords)
            .ok_or_else(|| CpuFault::Graphics(String::from("GIF tag payload length overflow")))?;
        if end > VU1_QWORDS {
            return Err(CpuFault::Graphics(String::from("XGKICK packet exceeds VU1 data memory")));
        }
        while cursor < end {
            let words = memory
                .read_vu_data_vector(1, start_qword.wrapping_add(cursor as u32))
                .ok_or_else(|| CpuFault::Graphics(String::from("VU1 data memory is unavailable")))?;
            packet.push(
                u128::from(words[0])
                    | (u128::from(words[1]) << 32)
                    | (u128::from(words[2]) << 64)
                    | (u128::from(words[3]) << 96),
            );
            cursor += 1;
        }
        if (tag >> 15) & 1 != 0 {
            ended = true;
            break;
        }
    }

    if !ended {
        return Err(CpuFault::Graphics(String::from("XGKICK packet has no GIF end-of-packet tag")));
    }
    memory.gs_mut().submit_gif(&packet)
        .map_err(|error| CpuFault::Graphics(error.to_string()))?;
    Ok(())
}

fn selected_vu_component(mask: u32) -> Option<usize> {
    if mask.count_ones() != 1 { return None; }
    Some(3 - mask.trailing_zeros() as usize)
}

fn is_control_transfer(ins: u32) -> bool {
    let op = ins >> 26;
    match op {
        0x01..=0x07 | 0x14..=0x17 => true,
        0x00 => matches!(ins & 0x3f, 0x08 | 0x09), // JR / JALR
        0x10 => ((ins >> 21) & 31) == 0x10 && (ins & 0x3f) == 0x18, // ERET
        0x11 | 0x12 => ((ins >> 21) & 31) == 0x08, // BC1 / BC2
        _ => false,
    }
}
