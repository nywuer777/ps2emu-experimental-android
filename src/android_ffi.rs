//! Small C ABI used by the experimental Android diagnostic shell.
//!
//! This exposes the existing Rust bring-up path. It does not add a PS2 BIOS
//! boot sequence or claim compatibility with commercial games.

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
    status: String,
}

fn set_status(core: &mut AndroidCore, message: impl AsRef<str>) {
    let snapshot = core.system.ee().snapshot();
    let iop = core.system.iop();
    let gs = core.system.ee_memory().gs();
    let dma = core.system.ee_memory().diagnostic_counters();
    let recent_trace = core.system.recent_ee_trace();
    let recent_trace = recent_trace.iter().rev().take(8).rev()
        .map(|(pc, instruction)| format!("{pc:08x}:{instruction:08x}"))
        .collect::<Vec<_>>()
        .join(" ");
    core.status = format!(
        "{}\nEE PC: {:#010x}\nEE instructions: {}\nEE exceptions: {}\nEE cause/EPC/BadVAddr: {:#x}/{:#010x}/{:#010x}\nEE fault instruction: {:#010x}\nEE trace PC/opcode: {}\nIOP PC: {:#010x}\nIOP instructions/exceptions: {}/{}\nVIF DMA: {}/{}; unpack vectors: {}; VU micro bytes: {}\nVU starts: {}/{}; GIF qwords: {}; VIF direct qwords: {}\nGS vertices/ignored writes: {}/{}\nScheduler steps: {}",
        message.as_ref(),
        snapshot.pc,
        snapshot.retired_instructions,
        snapshot.exceptions_taken,
        (snapshot.cp0[13] >> 2) & 0x1f,
        snapshot.cp0[14],
        snapshot.cp0[8],
        snapshot.last_exception_instruction,
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
    match core.system.load_disc_boot_elf(image) {
        Ok(Some((boot_path, loaded))) => {
            core.disc_loaded = true;
            set_status(core, format!(
                "ELF carregado diretamente: {boot_path}; entrada {:#010x}; {} segmentos. Boot de BIOS não implementado.",
                loaded.entry,
                loaded.segments.len(),
            ));
            0
        }
        Ok(None) => {
            core.disc_loaded = false;
            set_status(core, "ISO montada, mas SYSTEM.CNF não apontou para um ELF carregável.");
            -4
        }
        Err(error) => {
            core.disc_loaded = false;
            set_status(core, error.to_string());
            -5
        }
    }
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
        Ok(completed) => {
            let message = if core.system.ee().exceptions_taken > initial_exceptions {
                format!("Probe pausado na primeira exceção do EE após {completed} passos.")
            } else {
                format!("Probe concluído: {completed} passos solicitados sem nova exceção do EE.")
            };
            set_status(core, message);
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
