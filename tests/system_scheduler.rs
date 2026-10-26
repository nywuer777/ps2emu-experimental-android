use ps2emu_core::Ps2System;

#[test]
fn nop_firmware_advances_both_processors_at_scheduler_ratio() {
    let mut system = Ps2System::new();
    system.load_bios(&[0; 4], &[0; 4]).unwrap();
    system.run_ee_steps(1000).unwrap();
    let iop_steps = 1000 * 33_868_800u64 / 294_912_000;
    assert_eq!(system.scheduler_steps(), 1000);
    assert_eq!(system.ee().retired_instructions, 1000);
    assert_eq!(system.iop().retired_instructions, iop_steps);
    assert_eq!(system.iop_memory().iop_cycles(), iop_steps);
    assert_eq!(system.ee().pc, 0xbfc0_0000 + 4000);
    assert_eq!(system.iop().pc, 0xbfc0_0000 + iop_steps as u32 * 4);
    system.reset_cpus();
    system.run_ee_steps(1000).unwrap();
    assert_eq!(system.iop().retired_instructions, iop_steps);
}

#[test]
fn rejected_firmware_pair_does_not_reset_running_processors() {
    let mut system = Ps2System::new();
    system.load_bios(&[0; 4], &[0; 4]).unwrap();
    system.run_ee_steps(100).unwrap();
    let ee_pc = system.ee().pc;
    let iop_pc = system.iop().pc;
    assert!(system.load_bios(&[0; 4], &[0; 3]).is_err());
    assert_eq!(system.ee().pc, ee_pc);
    assert_eq!(system.iop().pc, iop_pc);
    assert_eq!(system.scheduler_steps(), 100);
}

fn firmware(words: &[u32]) -> Vec<u8> {
    let mut rom = vec![0u8; 4096];
    for (destination, word) in rom.chunks_exact_mut(4).zip(words) {
        destination.copy_from_slice(&word.to_le_bytes());
    }
    rom
}

#[test]
fn guest_cpus_exchange_sif_mailbox_under_shared_scheduler() {
    // EE publishes 42 and polls the reply; IOP polls the request, increments
    // it and publishes 43. No host writes to either mailbox are used.
    let ee = firmware(&[
        0x3c08_1000, // lui t0, 0x1000
        0x3508_f200, // ori t0, t0, 0xf200
        0x2409_002a, // addiu t1, zero, 42
        0xad09_0000, // sw t1, 0(t0)
        0x8d0a_0010, // poll: lw t2, 16(t0)
        0x1140_fffe, // beq t2, zero, poll
        0,          // delay slot
        0,          // success breakpoint
    ]);
    let iop = firmware(&[
        0x3c08_1d00, // lui t0, 0x1d00
        0x8d09_0000, // poll: lw t1, 0(t0)
        0,          // load-delay compatibility
        0x1120_fffd, // beq t1, zero, poll
        0,
        0x2529_0001, // addiu t1, t1, 1
        0xad09_0010, // sw t1, 16(t0)
    ]);
    let mut system = Ps2System::new();
    system.load_bios(&ee, &iop).unwrap();
    assert!(system.run_until_ee_pc(0xbfc0_001c, 1000).unwrap());
    assert_eq!(system.ee_memory_mut().read_u32(0x1000_f210).unwrap(), 43);
    assert_eq!(system.iop_memory_mut().read_u32(0x1d00_0000).unwrap(), 42);
    assert_eq!(system.ee().exceptions_taken, 0);
    assert_eq!(system.iop().exceptions_taken, 0);
    let steps = system.scheduler_steps();
    assert!(system.run_until_ee_pc(0xbfc0_001c, 0).unwrap());
    assert_eq!(system.scheduler_steps(), steps);
}

#[test]
fn breakpoint_budget_expires_without_claiming_a_hit() {
    let mut system = Ps2System::new();
    system.load_bios(&[0; 4], &[0; 4]).unwrap();
    assert!(!system.run_until_ee_pc(0xbfc0_1000, 10).unwrap());
    assert_eq!(system.scheduler_steps(), 10);
    assert_eq!(system.ee().pc, 0xbfc0_0028);
}
