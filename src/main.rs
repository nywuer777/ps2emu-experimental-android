//! language: Rust, file: src/main.rs, runtime: stable Rust, target: host probe

use ps2emu_core::{load_elf32_mips, load_elf32_mips_iop, CdvdImage, Cpu, IopCpu, IopMemory, Memory, Ps2System};
use std::{env, fs, process::ExitCode};

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = env::args_os().skip(1);
    // A separate parser keeps the integrated machine mode independent of the
    // legacy single-processor probe positional arguments.
    if env::args_os().nth(1).as_deref() == Some(std::ffi::OsStr::new("--system")) {
        return run_system(env::args_os().skip(2));
    }
    let first = args.next().ok_or_else(|| String::from("usage: ps2emu-probe [--iop] <mips-elf> [instruction-budget] | --bios <bios-rom> [instruction-budget] | --iop-bios <bios-rom> [--cdvd <disc.iso>] [instruction-budget] | --inspect-iso <disc.iso> | --extract-boot <disc.iso> <output.elf> | --probe-disc <disc.iso> [instruction-budget]"))?;
    if first == "--probe-disc" {
        let disc_path = args.next().ok_or_else(|| String::from("usage: ps2emu-probe --probe-disc <disc.iso> [instruction-budget] [--frame-ppm <output.ppm>]"))?;
        let budget = args.next().map(|value| value.to_string_lossy().parse::<u64>())
            .transpose()?.unwrap_or(100_000);
        let dump_path = match args.next() {
            None => None,
            Some(flag) if flag == "--frame-ppm" => Some(args.next().ok_or_else(|| String::from("--frame-ppm requires an output path"))?),
            Some(_) => return Err(String::from("usage: ps2emu-probe --probe-disc <disc.iso> [instruction-budget] [--frame-ppm <output.ppm>]").into()),
        };
        if args.next().is_some() { return Err(String::from("usage: ps2emu-probe --probe-disc <disc.iso> [instruction-budget] [--frame-ppm <output.ppm>]").into()); }
        let disc = CdvdImage::open(&disc_path)?;
        let Some((boot_path, executable)) = disc.read_boot_executable()? else {
            return Err(String::from("SYSTEM.CNF does not name a boot executable").into());
        };
        let mut memory = Memory::new();
        let loaded = load_elf32_mips(&executable, &mut memory)?;
        let mut cpu = Cpu::new(loaded.entry);
        println!("disc boot ELF: {boot_path}, entry={:#010x}, segments={}", loaded.entry, loaded.segments.len());
        println!("running EE scalar slice without BIOS initialization for {budget} instructions.");
        match cpu.run(&mut memory, budget) {
            Ok(()) => println!("budget reached: pc={:#010x}, retired={}, exceptions={}, ee_cycles={}", cpu.pc, cpu.retired_instructions, cpu.exceptions_taken, cpu.cpu_cycles),
            Err(error) => println!("ELF probe stopped: {error}; pc={:#010x}, retired={}, exceptions={}", cpu.pc, cpu.retired_instructions, cpu.exceptions_taken),
        }
        if let Some(path) = dump_path {
            fs::write(&path, memory.gs().encode_ppm())?;
            println!("GS framebuffer written: {}", path.to_string_lossy());
        }
        return Ok(());
    }
    if first == "--extract-boot" {
        let disc_path = args.next().ok_or_else(|| String::from("usage: ps2emu-probe --extract-boot <disc.iso> <output.elf>"))?;
        let output_path = args.next().ok_or_else(|| String::from("usage: ps2emu-probe --extract-boot <disc.iso> <output.elf>"))?;
        if args.next().is_some() { return Err(String::from("usage: ps2emu-probe --extract-boot <disc.iso> <output.elf>").into()); }
        let disc = CdvdImage::open(&disc_path)?;
        let Some((boot_path, executable)) = disc.read_boot_executable()? else {
            return Err(String::from("SYSTEM.CNF does not name a boot executable").into());
        };
        fs::write(&output_path, &executable)?;
        println!("extracted {boot_path}: {} bytes → {}", executable.len(), output_path.to_string_lossy());
        return Ok(());
    }
    if first == "--inspect-iso" {
        let path = args.next().ok_or_else(|| String::from("usage: ps2emu-probe --inspect-iso <disc.iso>"))?;
        if args.next().is_some() { return Err(String::from("usage: ps2emu-probe --inspect-iso <disc.iso>").into()); }
        let disc = CdvdImage::open(&path)?;
        let boot = disc.boot_executable()?;
        println!("ISO 9660: volume={:?}, format={:?}, sectors={}", disc.volume_id(), disc.format(), disc.volume_blocks());
        match boot {
            Some(executable) => println!("boot executable: {executable}"),
            None => println!("boot executable: SYSTEM.CNF or BOOT entry not found"),
        }
        return Ok(());
    }
    if first == "--iop-bios" {
        let path = args.next().ok_or_else(|| String::from("usage: ps2emu-probe --iop-bios <bios-rom> [instruction-budget]"))?;
        let argument = args.next();
        let (disc_path, budget_argument) = if argument.as_deref() == Some(std::ffi::OsStr::new("--cdvd")) {
            (Some(args.next().ok_or_else(|| String::from("--cdvd requires a disc image path"))?), args.next())
        } else {
            (None, argument)
        };
        let budget = budget_argument.map(|value| value.to_string_lossy().parse::<u64>())
            .transpose()?.unwrap_or(100_000);
        if args.next().is_some() {
            return Err(String::from("usage: ps2emu-probe --iop-bios <bios-rom> [instruction-budget]").into());
        }
        let image = fs::read(&path)?;
        let mut memory = IopMemory::new();
        memory.load_bios_rom(&image)?;
        if let Some(disc_path) = disc_path {
            memory.mount_cdvd(CdvdImage::open(disc_path)?);
        }
        let mut cpu = IopCpu::new(0xbfc0_0000);
        println!("IOP BIOS ROM mapped: {} bytes, reset vector={:#010x}", image.len(), cpu.pc);
        println!("CD/DVD image mounted: {}", memory.cdvd_mounted());
        println!("Running IOP R3000A slice for {budget} instructions.");
        match cpu.run(&mut memory, budget) {
            Ok(()) => println!("budget reached: pc={:#010x}, retired={}, exceptions={}, iop_cycles={}", cpu.pc, cpu.retired_instructions, cpu.exceptions_taken, memory.iop_cycles()),
            Err(error) => println!("IOP BIOS execution stopped: {error}; pc={:#010x}, retired={}, iop_cycles={}", cpu.pc, cpu.retired_instructions, memory.iop_cycles()),
        }
        return Ok(());
    }
    if first == "--bios" {
        let path = args.next().ok_or_else(|| String::from("usage: ps2emu-probe --bios <bios-rom> [instruction-budget]"))?;
        let budget = args.next().map(|value| value.to_string_lossy().parse::<u64>())
            .transpose()?.unwrap_or(100_000);
        if args.next().is_some() {
            return Err(String::from("usage: ps2emu-probe --bios <bios-rom> [instruction-budget]").into());
        }
        let image = fs::read(&path)?;
        let mut memory = Memory::new();
        memory.load_bios_rom(&image)?;
        let mut cpu = Cpu::new(0xbfc0_0000);
        println!("EE BIOS ROM mapped: {} bytes, reset vector={:#010x}", image.len(), cpu.pc);
        println!("Running EE scalar slice for {budget} instructions.");
        match cpu.run(&mut memory, budget) {
            Ok(()) => println!("budget reached: pc={:#010x}, retired={}, exceptions={}, ee_cycles={}", cpu.pc, cpu.retired_instructions, cpu.exceptions_taken, cpu.cpu_cycles),
            Err(error) => println!("BIOS execution stopped: {error}; pc={:#010x}, retired={}", cpu.pc, cpu.retired_instructions),
        }
        return Ok(());
    }
    let iop_mode = first == "--iop";
    let path = if iop_mode { args.next().ok_or_else(|| String::from("usage: ps2emu-probe [--iop] <mips-elf> [instruction-budget]"))? } else { first };
    let budget = args.next().map(|value| value.to_string_lossy().parse::<u64>())
        .transpose()?.unwrap_or(100_000);
    if args.next().is_some() {
        return Err(String::from("usage: ps2emu-probe [--iop] <mips-elf> [instruction-budget]").into());
    }

    let image = fs::read(&path)?;
    if iop_mode {
        let mut memory = IopMemory::new();
        let loaded = load_elf32_mips_iop(&image, &mut memory)?;
        let mut cpu = IopCpu::new(loaded.entry);
        println!("IOP ELF loaded: entry={:#010x}, segments={}", loaded.entry, loaded.segments.len());
        println!("Running IOP R3000A slice for {budget} instructions.");
        match cpu.run(&mut memory, budget) {
            Ok(()) => println!("budget reached: pc={:#010x}, retired={}, exceptions={}", cpu.pc, cpu.retired_instructions, cpu.exceptions_taken),
            Err(error) => println!("IOP execution stopped: {error}; pc={:#010x}, retired={}, exceptions={}", cpu.pc, cpu.retired_instructions, cpu.exceptions_taken),
        }
    } else {
        let mut memory = Memory::new();
        let loaded = load_elf32_mips(&image, &mut memory)?;
        let mut cpu = Cpu::new(loaded.entry);
        println!("EE ELF loaded: entry={:#010x}, segments={}", loaded.entry, loaded.segments.len());
        println!("Running EE scalar slice for {budget} instructions.");
        match cpu.run(&mut memory, budget) {
            Ok(()) => println!("budget reached: pc={:#010x}, retired={}, exceptions={}, ee_cycles={}", cpu.pc, cpu.retired_instructions, cpu.exceptions_taken, cpu.cpu_cycles),
            Err(error) => println!("execution stopped: {error}; pc={:#010x}, retired={}", cpu.pc, cpu.retired_instructions),
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("ps2emu-probe: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run_system(mut args: impl Iterator<Item = std::ffi::OsString>) -> Result<(), Box<dyn std::error::Error>> {
    let mut ee_bios = None;
    let mut iop_bios = None;
    let mut disc = None;
    let mut frame = None;
    let mut budget = 100_000u64;
    let mut stop_pc = None;
    while let Some(flag) = args.next() {
        let value = args.next().ok_or("system options require a value")?;
        match flag.to_str() {
            Some("--ee-bios") => ee_bios = Some(value),
            Some("--iop-bios") => iop_bios = Some(value),
            Some("--disc") => disc = Some(value),
            Some("--frame-ppm") => frame = Some(value),
            Some("--stop-ee-pc") => {
                let text = value.to_str().ok_or("invalid breakpoint address")?;
                stop_pc = Some(if let Some(hex) = text.strip_prefix("0x") {
                    u32::from_str_radix(hex, 16)?
                } else { text.parse()? });
            }
            Some("--steps") => budget = value.to_str().ok_or("invalid step budget")?.parse()?,
            _ => return Err(format!("unknown system option: {}", flag.to_string_lossy()).into()),
        }
    }
    let ee_rom = fs::read(ee_bios.ok_or("--system requires --ee-bios <rom>")?)?;
    let iop_rom = fs::read(iop_bios.ok_or("--system requires --iop-bios <rom>")?)?;
    let mut system = Ps2System::new();
    system.load_bios(&ee_rom, &iop_rom)?;
    if let Some(path) = disc { system.mount_cdvd(CdvdImage::open(path)?); }
    println!("integrated BIOS probe: EE/IOP reset vectors, budget={budget}; approximate timing");
    let outcome = if let Some(target) = stop_pc {
        system.run_until_ee_pc(target, budget).map(|hit| {
            println!("EE breakpoint {target:#010x}: {}", if hit { "hit" } else { "budget exhausted" });
        })
    } else { system.run_ee_steps(budget) };
    println!("EE pc={:#010x}, retired={}, cycles={}, exceptions={}", system.ee().pc,
        system.ee().retired_instructions, system.ee().cpu_cycles, system.ee().exceptions_taken);
    println!("IOP pc={:#010x}, retired={}, cycles={}, exceptions={}", system.iop().pc,
        system.iop().retired_instructions, system.iop_memory().iop_cycles(), system.iop().exceptions_taken);
    println!("scheduler steps={}, disc mounted={}", system.scheduler_steps(), system.iop_memory().cdvd_mounted());
    if let Some(path) = frame { fs::write(path, system.ee_memory().gs().encode_ppm())?; }
    outcome.map_err(|error| error.into())
}
