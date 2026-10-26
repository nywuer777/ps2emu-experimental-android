use ps2emu_core::Ps2System;

fn machine() -> Ps2System {
    let mut system = Ps2System::new();
    system.load_bios(&[0; 4], &[0; 4]).unwrap();
    system
}

#[test]
fn ee_to_iop_dma_resumes_after_fifo_backpressure() {
    let mut system = machine();
    let bytes: Vec<u8> = (0..1024u32).map(|i| (i.wrapping_mul(17) ^ (i >> 3)) as u8).collect();
    system.ee_memory_mut().write_bytes(0x1000, &bytes).unwrap();
    for (address, value) in [(0x1000_c410, 0x1000), (0x1000_c420, 64), (0x1000_c400, 0x101)] {
        system.ee_memory_mut().write_u32(address, value).unwrap();
    }
    // Let the producer fill its 128-word FIFO before enabling the consumer.
    system.run_ee_steps(100).unwrap();
    assert_ne!(system.ee_memory_mut().read_u32(0x1000_c400).unwrap() & 0x100, 0);
    for (address, value) in [(0x1f80_1530, 0x2000), (0x1f80_1534, 256), (0x1f80_1538, 1 << 24)] {
        system.iop_memory_mut().write_u32(address, value).unwrap();
    }
    system.ee_memory_mut().write_u32(0x1000_e010, 1 << 22).unwrap();
    system.iop_memory_mut().write_u32(0x1f80_1574, (1 << 23) | (1 << 19)).unwrap();
    system.run_ee_steps(3000).unwrap();
    assert_eq!(system.iop_memory().read_bytes(0x2000, bytes.len()).unwrap(), bytes.as_slice());
    assert_eq!(system.ee_memory_mut().read_u32(0x1000_c400).unwrap() & 0x100, 0);
    assert_eq!(system.iop_memory_mut().read_u32(0x1f80_1538).unwrap() & (1 << 24), 0);
    assert_ne!(system.ee_memory_mut().read_u32(0x1000_e010).unwrap() & (1 << 31), 0);
    assert_ne!(system.ee_memory_mut().read_u32(0x1000_f000).unwrap() & (1 << 3), 0);
    assert_ne!(system.iop_memory_mut().read_u32(0x1f80_1574).unwrap() & (1 << 31), 0);
    assert_ne!(system.iop_memory_mut().read_u32(0x1f80_1070).unwrap() & (1 << 3), 0);
}

#[test]
fn iop_to_ee_dma_waits_for_receiver_then_completes() {
    let mut system = machine();
    let bytes: Vec<u8> = (0..1024u32).map(|i| (i.wrapping_mul(29) ^ (i >> 2)) as u8).collect();
    system.iop_memory_mut().write_bytes(0x1000, &bytes).unwrap();
    for (address, value) in [(0x1f80_1520, 0x1000), (0x1f80_1524, 256), (0x1f80_1528, (1 << 24) | 1)] {
        system.iop_memory_mut().write_u32(address, value).unwrap();
    }
    system.run_ee_steps(1500).unwrap();
    assert_ne!(system.iop_memory_mut().read_u32(0x1f80_1528).unwrap() & (1 << 24), 0);
    for (address, value) in [(0x1000_c010, 0x2000), (0x1000_c020, 64), (0x1000_c000, 0x100)] {
        system.ee_memory_mut().write_u32(address, value).unwrap();
    }
    system.ee_memory_mut().write_u32(0x1000_e010, 1 << 21).unwrap();
    system.iop_memory_mut().write_u32(0x1f80_1574, (1 << 23) | (1 << 18)).unwrap();
    system.run_ee_steps(3000).unwrap();
    assert_eq!(system.ee_memory().read_bytes(0x2000, bytes.len()).unwrap(), bytes.as_slice());
    assert_eq!(system.ee_memory_mut().read_u32(0x1000_c000).unwrap() & 0x100, 0);
    assert_eq!(system.iop_memory_mut().read_u32(0x1f80_1528).unwrap() & (1 << 24), 0);
    assert_ne!(system.ee_memory_mut().read_u32(0x1000_e010).unwrap() & (1 << 31), 0);
    assert_ne!(system.iop_memory_mut().read_u32(0x1f80_1574).unwrap() & (1 << 31), 0);
    assert_ne!(system.iop_memory_mut().read_u32(0x1f80_1070).unwrap() & (1 << 3), 0);
}

#[test]
fn invalid_destination_reports_fault_and_preserves_queued_qword() {
    let mut system = machine();
    let payload = [0x5au8; 16];
    system.iop_memory_mut().write_bytes(0x1000, &payload).unwrap();
    for (a, v) in [(0x1f80_1520, 0x1000), (0x1f80_1524, 4), (0x1f80_1528, (1 << 24) | 1)] {
        system.iop_memory_mut().write_u32(a, v).unwrap();
    }
    system.run_ee_steps(100).unwrap();
    for (a, v) in [(0x1000_c010, 0x0200_0000), (0x1000_c020, 1), (0x1000_c000, 0x100)] {
        system.ee_memory_mut().write_u32(a, v).unwrap();
    }
    let error = system.run_ee_steps(1).unwrap_err();
    assert!(matches!(error, ps2emu_core::Ps2Error::Dma(_)));
    assert_eq!(system.ee_memory_mut().read_u32(0x1000_c020).unwrap(), 1);
    assert_eq!(system.ee_memory_mut().read_u32(0x1000_c000).unwrap() & 0x100, 0);
    system.ee_memory_mut().write_u32(0x1000_c010, 0x2000).unwrap();
    system.ee_memory_mut().write_u32(0x1000_c000, 0x100).unwrap();
    system.run_ee_steps(1).unwrap();
    assert_eq!(system.ee_memory().read_bytes(0x2000, 16).unwrap(), &payload);
}

#[test]
fn invalid_source_is_reported_instead_of_silent_completion() {
    let mut system = machine();
    for (a, v) in [(0x1000_c410, 0x0200_0000), (0x1000_c420, 1), (0x1000_c400, 0x101)] {
        system.ee_memory_mut().write_u32(a, v).unwrap();
    }
    let error = system.run_ee_steps(1).unwrap_err().to_string();
    assert!(error.contains("SIF1 source 0x02000000"));
    assert_eq!(system.ee_memory_mut().read_u32(0x1000_c420).unwrap(), 1);
    assert_eq!(system.ee_memory_mut().read_u32(0x1000_c400).unwrap() & 0x100, 0);
}
