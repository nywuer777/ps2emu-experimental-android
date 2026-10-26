//! Small C ABI used by the experimental Android diagnostic shell.
//!
//! This exposes the experimental BIOS and CDVD bring-up path.

use crate::{CdvdImage, Ps2System};
use std::{
    ffi::CStr,
    fs,
    os::raw::{c_char, c_void},
    ptr,
};

struct AndroidCore {
    system: Ps2System,
    bios_loaded: bool,
    disc_loaded: bool,
    bios_crc32: Option<u32>,
    bios_sha1: Option<[u8; 20]>,
    status: String,
}

fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = u32::MAX;
    for byte in bytes {
        crc ^= *byte as u32;
        for _ in 0..8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xedb8_8320 } else { crc >> 1 };
        }
    }
    !crc
}

fn sha1(bytes: &[u8]) -> [u8; 20] {
    let mut data = bytes.to_vec();
    let bit_len = (data.len() as u64).wrapping_mul(8);
    data.push(0x80);
    while data.len() % 64 != 56 {
        data.push(0);
    }
    data.extend_from_slice(&bit_len.to_be_bytes());

    let mut h = [
        0x6745_2301u32,
        0xefcd_ab89,
        0x98ba_dcfe,
        0x1032_5476,
        0xc3d2_e1f0,
    ];
    for chunk in data.chunks_exact(64) {
        let mut w = [0u32; 80];
        for (i, word) in chunk.chunks_exact(4).enumerate() {
            w[i] = u32::from_be_bytes(word.try_into().unwrap());
        }
        for i in 16..80 {
            w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
        }
        let [mut a, mut b, mut c, mut d, mut e] = h;
        for (i, word) in w.iter().enumerate() {
            let (f, k) = match i {
                0..=19 => ((b & c) | ((!b) & d), 0x5a82_7999),
                20..=39 => (b ^ c ^ d, 0x6ed9_eba1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8f1b_bcdc),
                _ => (b ^ c ^ d, 0xca62_c1d6),
            };
            let next = a.rotate_left(5)
                .wrapping_add(f)
                .wrapping_add(e)
                .wrapping_add(k)
                .wrapping_add(*word);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = next;
        }
        h[0] = h[0].wrapping_add(a);
        h[1] = h[1].wrapping_add(b);
        h[2] = h[2].wrapping_add(c);
        h[3] = h[3].wrapping_add(d);
        h[4] = h[4].wrapping_add(e);
    }
    let mut digest = [0u8; 20];
    for (word, out) in h.iter().zip(digest.chunks_exact_mut(4)) {
        out.copy_from_slice(&word.to_be_bytes());
    }
    digest
}

fn hex_sha1(digest: &[u8; 20]) -> String {
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn set_status(core: &mut AndroidCore, message: impl AsRef<str>) {
    let snapshot = core.system.ee().snapshot();
    let iop = core.system.iop();
    let gs = core.system.ee_memory().gs();
    let dma = core.system.ee_memory().diagnostic_counters();
    let exception_epc_instruction = core.system.ee_memory()
        .read_u32(snapshot.cp0[14])
        .unwrap_or(0);
    let exception_opcode = exception_epc_instruction >> 26;
    let exception_mmi_function = exception_epc_instruction & 0x3f;
    let exception_code = (snapshot.cp0[13] >> 2) & 0x1f;
    let exception_name = match exception_code {
        0 => "interrupt",
        1 => "TLB modified",
        2 => "TLB load",
        3 => "TLB store",
        4 => "address error on load",
        5 => "address error on store",
        6 => "bus error on instruction fetch",
        7 => "bus error on data access",
        8 => "syscall",
        9 => "breakpoint",
        10 => "reserved instruction",
        11 => "coprocessor unusable",
        12 => "arithmetic overflow",
        13 => "trap",
        14 => "virtual coherency",
        _ => "processor exception",
    };
    let fault_opcode = snapshot.last_exception_instruction >> 26;
    let fault_mmi_function = snapshot.last_exception_instruction & 0x3f;
    let opcode_diagnostic = if exception_opcode == 0x1c {
        format!("MMI function {exception_mmi_function:#04x}")
    } else {
        format!("primary opcode {exception_opcode:#04x}")
    };
    let fault_diagnostic = if fault_opcode == 0x1c {
        format!("MMI function {fault_mmi_function:#04x}")
    } else {
        format!("primary opcode {fault_opcode:#04x}")
    };
    let recent_trace = core.system.recent_ee_trace();
    let recent_trace = recent_trace.iter().rev().take(8).rev()
        .map(|(pc, instruction)| format!("{pc:08x}:{instruction:08x}"))
        .collect::<Vec<_>>()
        .join(" ");
    let bios_fingerprint = match (core.bios_crc32, core.bios_sha1.as_ref()) {
        (Some(crc), Some(sha1)) => format!("\nBIOS CRC32: {crc:08x}\nBIOS SHA-1: {}", hex_sha1(sha1)),
        (Some(crc), None) => format!("\nBIOS CRC32: {crc:08x}"),
        _ => String::new(),
    };
    core.status = format!(
        "{}{}\nEE PC: {:#010x}\nEE instructions: {}\nEE exceptions: {}\nEE exception: {} (ExcCode {})\nEE cause/EPC/BadVAddr: {:#x}/{:#010x}/{:#010x}\nEE EPC instruction/BD: {:#010x}/{} ({})\nEE last fault PC/instruction: {:#010x}/{:#010x} ({})\nEE loop r1/r2/r3: {:#010x}/{:#010x}/{:#010x}\nEE trace PC/opcode: {}\nIOP PC: {:#010x}\nIOP instructions/exceptions: {}/{}\nVIF DMA: {}/{}; unpack vectors: {}; VU micro bytes: {}\nVU starts: {}/{}; GIF qwords: {}; VIF direct qwords: {}\nGS vertices/ignored writes: {}/{}\nScheduler steps: {}",
        message.as_ref(),
        bios_fingerprint,
        snapshot.pc,
        snapshot.retired_instructions,
        snapshot.exceptions_taken,
        exception_name,
        exception_code,
        snapshot.cp0[13],
        snapshot.cp0[14],
        snapshot.cp0[8],
        exception_epc_instruction,
        snapshot.cp0[13] >> 31 != 0,
        opcode_diagnostic,
        snapshot.last_exception_pc,
        snapshot.last_exception_instruction,
        fault_diagnostic,
        snapshot.gpr[1] as u32,
        snapshot.gpr[2] as u32,
        snapshot.gpr[3] as u32,
        recent_trace,
        iop.pc,
        iop.retired_instructions,
        iop.exceptions_taken,
        dma[0],
        dma[1],
        dma[2],
        dma[3],
        snapshot.vu_microprograms_started[0],
        snapshot.vu_microprograms_started[1],
        dma[5],
        dma[4],
        gs.submitted_vertices(),
        gs.ignored_register_writes(),
        core.system.scheduler_steps(),
    );
}

unsafe fn path_from_ptr<'a>(path: *const c_char) -> Result<&'a str, i32> {
    if path.is_null() {
        return Err(-1);
    }
    CStr::from_ptr(path).to_str().map_err(|_| -2)
}

#[no_mangle]
pub extern "C" fn ps2emu_create() -> *mut c_void {
    let mut core = Box::new(AndroidCore {
        system: Ps2System::new(),
        bios_loaded: false,
        disc_loaded: false,
        bios_crc32: None,
        bios_sha1: None,
        status: String::new(),
    });
    set_status(&mut core, "Núcleo inicializado; selecione BIOS e ISO.");
    Box::into_raw(core).cast()
}

#[no_mangle]
pub unsafe extern "C" fn ps2emu_destroy(handle: *mut c_void) {
    if !handle.is_null() {
        drop(Box::from_raw(handle.cast::<AndroidCore>()));
    }
}

#[no_mangle]
pub unsafe extern "C" fn ps2emu_load_bios(
    handle: *mut c_void,
    path: *const c_char,
) -> i32 {
    let Some(core) = handle.cast::<AndroidCore>().as_mut() else {
        return -1;
    };
    let path = match path_from_ptr(path) {
        Ok(path) => path,
        Err(code) => return code,
    };
    let bytes = match fs::read(path) {
        Ok(bytes) => bytes,
        Err(error) => {
            set_status(core, format!("Falha ao ler BIOS: {error}"));
            return -3;
        }
    };
    match core.system.load_bios(&bytes, &bytes) {
        Ok(()) => {
            core.bios_loaded = true;
            core.disc_loaded = false;
            core.bios_crc32 = Some(crc32(&bytes));
            core.bios_sha1 = Some(sha1(&bytes));
            set_status(core, format!("BIOS aceita, {} bytes; mesma imagem usada nos mapas EE e IOP.", bytes.len()));
            0
        }
        Err(error) => {
            set_status(core, error.to_string());
            -4
        }
    }
}

#[no_mangle]
pub unsafe extern "C" fn ps2emu_load_disc(
    handle: *mut c_void,
    path: *const c_char,
) -> i32 {
    let Some(core) = handle.cast::<AndroidCore>().as_mut() else {
        return -1;
    };
    let path = match path_from_ptr(path) {
        Ok(path) => path,
        Err(code) => return code,
    };
    let image = match CdvdImage::open(path) {
        Ok(image) => image,
        Err(error) => {
            set_status(core, error.to_string());
            return -3;
        }
    };
    core.system.mount_cdvd(image);
    core.disc_loaded = true;
    set_status(core, "ISO montada no CDVD; execução iniciará pelo vetor da BIOS.");
    0
}

#[no_mangle]
pub unsafe extern "C" fn ps2emu_run(handle: *mut c_void, steps: u64) -> i32 {
    let Some(core) = handle.cast::<AndroidCore>().as_mut() else {
        return -1;
    };
    if !core.bios_loaded || !core.disc_loaded {
        set_status(core, "Selecione uma BIOS aceita e uma ISO com ELF inicializável.");
        return -2;
    }
    let initial_exceptions = core.system.ee().exceptions_taken;
    match core.system.run_ee_steps_until_exception(steps) {
        Ok(completed_steps) => {
            let new_exceptions = core.system.ee().exceptions_taken.saturating_sub(initial_exceptions);
            set_status(core, format!(
                "Probe parou após {completed_steps} de {steps} passos; novas exceções do EE: {new_exceptions}."
            ));
            0
        }
        Err(error) => {
            set_status(core, format!("Probe interrompido: {error}"));
            -3
        }
    }
}

#[no_mangle]
pub unsafe extern "C" fn ps2emu_status(
    handle: *const c_void,
    output: *mut c_char,
    capacity: usize,
) -> usize {
    if output.is_null() || capacity == 0 {
        return 0;
    }
    let Some(core) = handle.cast::<AndroidCore>().as_ref() else {
        *output = 0;
        return 0;
    };
    let bytes = core.status.as_bytes();
    let count = bytes.len().min(capacity - 1);
    ptr::copy_nonoverlapping(bytes.as_ptr(), output.cast::<u8>(), count);
    *output.add(count) = 0;
    count
}
