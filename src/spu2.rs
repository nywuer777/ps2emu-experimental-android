//! Experimental SPU2 voice decoder and stereo mixer.
//!
//! This is an IOP-clocked bring-up model. It decodes the common 4-bit ADPCM
//! voice format and exposes interleaved 48 kHz stereo samples to a host audio
//! backend; ADSR, reverb, DMA arbitration, and the two-core effect buses remain
//! approximations or unimplemented.

use std::collections::VecDeque;

pub const SPU2_BASE: u32 = 0x1f90_0000;
pub const SPU2_REGISTER_SIZE: u32 = 0x800;
pub const SPU2_RAM_SIZE: usize = 2 * 1024 * 1024;
const IOP_CLOCK_HZ: u64 = 33_868_800;
const OUTPUT_RATE: u64 = 48_000;
const MAX_QUEUED_SAMPLES: usize = 96_000;
const VOICES_PER_CORE: usize = 24;
const CORE_STRIDE: usize = 0x400;
const VOICE_BASE: usize = 0x000;
const KEY_ON: usize = 0x1a0;
const KEY_OFF: usize = 0x1a4;
const TRANSFER_ADDRESS_HIGH: usize = 0x1a8;
const TRANSFER_ADDRESS_LOW: usize = 0x1aa;
const TRANSFER_DATA: usize = 0x1ac;
const VOICE_ADDRESS: usize = 0x1c0;

#[derive(Clone)]
struct Voice {
    active: bool,
    releasing: bool,
    start_address: usize,
    block_address: usize,
    sample_index: usize,
    pitch_phase: u32,
    samples: [i16; 28],
    block_flags: u8,
    previous: [i32; 2],
    envelope: i32,
    loop_address: Option<usize>,
    block_loaded: bool,
}

impl Default for Voice {
    fn default() -> Self {
        Self {
            active: false,
            releasing: false,
            start_address: 0,
            block_address: 0,
            sample_index: 0,
            pitch_phase: 0,
            samples: [0; 28],
            block_flags: 0,
            previous: [0; 2],
            envelope: 0,
            loop_address: None,
            block_loaded: false,
        }
    }
}

#[derive(Clone)]
pub struct Spu2 {
    registers: Vec<u16>,
    sound_ram: Vec<u8>,
    voices: [Voice; 48],
    transfer_word_address: [u32; 2],
    sample_clock: u64,
    output: VecDeque<i16>,
}

impl Default for Spu2 {
    fn default() -> Self { Self::new() }
}

impl Spu2 {
    pub fn new() -> Self {
        Self {
            registers: vec![0; SPU2_REGISTER_SIZE as usize / 2],
            sound_ram: vec![0; SPU2_RAM_SIZE],
            voices: std::array::from_fn(|_| Voice::default()),
            transfer_word_address: [0; 2],
            sample_clock: 0,
            output: VecDeque::new(),
        }
    }

    pub fn take_audio_samples(&mut self) -> Vec<i16> {
        self.output.drain(..).collect()
    }

    pub fn sound_ram(&self) -> &[u8] { &self.sound_ram }

    pub fn tick(&mut self, cycles: u64) {
        self.sample_clock = self.sample_clock.saturating_add(cycles.saturating_mul(OUTPUT_RATE));
        while self.sample_clock >= IOP_CLOCK_HZ {
            self.sample_clock -= IOP_CLOCK_HZ;
            self.mix_frame();
        }
    }

    pub fn read_u8(&mut self, offset: u32) -> u8 {
        let offset = (offset as usize) & 0x7ff;
        let word = self.read_word(offset & !1);
        (word >> ((offset & 1) * 8)) as u8
    }

    pub fn read_u16(&mut self, offset: u32) -> u16 {
        self.read_word((offset as usize) & 0x7fe)
    }

    pub fn read_u32(&mut self, offset: u32) -> u32 {
        let offset = (offset as usize) & 0x7fc;
        u32::from(self.read_word(offset)) | (u32::from(self.read_word(offset + 2)) << 16)
    }

    pub fn write_u8(&mut self, offset: u32, value: u8) {
        let offset = (offset as usize) & 0x7ff;
        let word_offset = offset & !1;
        let old = self.peek_word(word_offset);
        let shift = (offset & 1) * 8;
        let merged = (old & !(0xff << shift)) | (u16::from(value) << shift);
        self.write_word(word_offset, merged);
    }

    pub fn write_u16(&mut self, offset: u32, value: u16) {
        self.write_word((offset as usize) & 0x7fe, value);
    }

    pub fn write_u32(&mut self, offset: u32, value: u32) {
        let offset = (offset as usize) & 0x7fc;
        self.write_word(offset, value as u16);
        self.write_word(offset + 2, (value >> 16) as u16);
    }

    fn read_word(&mut self, offset: usize) -> u16 {
        if offset % CORE_STRIDE == TRANSFER_DATA {
            let core = offset / CORE_STRIDE;
            let word_address = self.transfer_word_address[core] as usize;
            let ram_offset = (word_address * 2) & (SPU2_RAM_SIZE - 1);
            let value = u16::from_le_bytes([self.sound_ram[ram_offset], self.sound_ram[(ram_offset + 1) & (SPU2_RAM_SIZE - 1)]]);
            self.transfer_word_address[core] = self.transfer_word_address[core].wrapping_add(1);
            return value;
        }
        if let Some(voice_index) = Self::voice_register_index(offset, 0x0a) {
            return self.voices[voice_index].envelope.clamp(0, 0x7fff) as u16;
        }
        self.peek_word(offset)
    }

    fn peek_word(&self, offset: usize) -> u16 {
        self.registers[(offset & 0x7fe) / 2]
    }

    fn write_word(&mut self, offset: usize, value: u16) {
        let offset = offset & 0x7fe;
        let core = offset / CORE_STRIDE;
        let core_offset = offset % CORE_STRIDE;
        if core_offset == TRANSFER_ADDRESS_HIGH || core_offset == TRANSFER_ADDRESS_LOW {
            let old = self.transfer_word_address[core];
            self.transfer_word_address[core] = if core_offset == TRANSFER_ADDRESS_HIGH {
                (old & 0x0000_ffff) | (u32::from(value) << 16)
            } else {
                (old & 0xffff_0000) | u32::from(value)
            };
        } else if core_offset == TRANSFER_DATA {
            let ram_offset = ((self.transfer_word_address[core] as usize) * 2) & (SPU2_RAM_SIZE - 1);
            let bytes = value.to_le_bytes();
            self.sound_ram[ram_offset] = bytes[0];
            self.sound_ram[(ram_offset + 1) & (SPU2_RAM_SIZE - 1)] = bytes[1];
            self.transfer_word_address[core] = self.transfer_word_address[core].wrapping_add(1);
        }
        self.registers[offset / 2] = value;

        if core_offset == KEY_ON || core_offset == KEY_ON + 2 {
            let mask = if core_offset == KEY_ON { u32::from(value) } else { u32::from(value) << 16 };
            self.key_on(core, mask);
        } else if core_offset == KEY_OFF || core_offset == KEY_OFF + 2 {
            let mask = if core_offset == KEY_OFF { u32::from(value) } else { u32::from(value) << 16 };
            self.key_off(core, mask);
        }
    }

    fn voice_register_index(offset: usize, register: usize) -> Option<usize> {
        let core = offset / CORE_STRIDE;
        let local = offset % CORE_STRIDE;
        if local >= VOICE_BASE && local < VOICE_BASE + VOICES_PER_CORE * 0x10 && local % 0x10 == register {
            Some(core * VOICES_PER_CORE + local / 0x10)
        } else {
            None
        }
    }

    fn voice_register(&self, voice_index: usize, register: usize) -> u16 {
        let core = voice_index / VOICES_PER_CORE;
        let voice = voice_index % VOICES_PER_CORE;
        self.peek_word(core * CORE_STRIDE + VOICE_BASE + voice * 0x10 + register)
    }

    fn key_on(&mut self, core: usize, mask: u32) {
        for local_voice in 0..VOICES_PER_CORE {
            if mask & (1 << local_voice) == 0 { continue; }
            let index = core * VOICES_PER_CORE + local_voice;
            let address_offset = core * CORE_STRIDE + VOICE_ADDRESS + local_voice * 12;
            let start = ((u32::from(self.peek_word(address_offset)) << 16)
                | u32::from(self.peek_word(address_offset + 2))) as usize;
            let voice = &mut self.voices[index];
            voice.active = true;
            voice.releasing = false;
            voice.start_address = (start * 2) & (SPU2_RAM_SIZE - 1);
            voice.block_address = voice.start_address;
            voice.sample_index = 0;
            voice.pitch_phase = 0;
            voice.samples = [0; 28];
            voice.block_flags = 0;
            voice.previous = [0; 2];
            voice.envelope = 0;
            voice.loop_address = None;
            voice.block_loaded = false;
        }
    }

    fn key_off(&mut self, core: usize, mask: u32) {
        for local_voice in 0..VOICES_PER_CORE {
            if mask & (1 << local_voice) != 0 {
                self.voices[core * VOICES_PER_CORE + local_voice].releasing = true;
            }
        }
    }

    fn mix_frame(&mut self) {
        let mut left = 0i64;
        let mut right = 0i64;
        for index in 0..self.voices.len() {
            let sample = self.next_voice_sample(index);
            if sample == 0 { continue; }
            let volume_left = i64::from(self.voice_register(index, 0x00) & 0x7fff);
            let volume_right = i64::from(self.voice_register(index, 0x02) & 0x7fff);
            let envelope = i64::from(self.voices[index].envelope);
            left += i64::from(sample) * envelope * volume_left / (0x7fff * 0x7fff);
            right += i64::from(sample) * envelope * volume_right / (0x7fff * 0x7fff);
        }
        self.output.push_back(left.clamp(i16::MIN as i64, i16::MAX as i64) as i16);
        self.output.push_back(right.clamp(i16::MIN as i64, i16::MAX as i64) as i16);
        while self.output.len() > MAX_QUEUED_SAMPLES { self.output.pop_front(); }
    }

    fn next_voice_sample(&mut self, index: usize) -> i16 {
        if !self.voices[index].active { return 0; }
        let pitch = u32::from(self.voice_register(index, 0x04) & 0x3fff).max(1);
        let voice = &mut self.voices[index];
        if voice.releasing {
            voice.envelope = (voice.envelope - 96).max(0);
            if voice.envelope == 0 { voice.active = false; return 0; }
        } else {
            voice.envelope = (voice.envelope + 96).min(0x7fff);
        }
        if !voice.block_loaded { Self::decode_block(voice, &self.sound_ram); }
        let sample = voice.samples[voice.sample_index.min(27)];
        voice.pitch_phase = voice.pitch_phase.saturating_add(pitch);
        while voice.pitch_phase >= 0x1000 && voice.active {
            voice.pitch_phase -= 0x1000;
            voice.sample_index += 1;
            if voice.sample_index >= 28 {
                if voice.block_flags & 1 != 0 {
                    if voice.block_flags & 4 != 0 {
                        voice.block_address = voice.loop_address.unwrap_or(voice.start_address);
                    } else {
                        voice.active = false;
                        break;
                    }
                } else {
                    voice.block_address = (voice.block_address + 16) & (SPU2_RAM_SIZE - 1);
                }
                voice.sample_index = 0;
                voice.block_loaded = false;
                Self::decode_block(voice, &self.sound_ram);
            }
        }
        sample
    }

    fn decode_block(voice: &mut Voice, sound_ram: &[u8]) {
        let address = voice.block_address & (SPU2_RAM_SIZE - 1);
        let header = sound_ram[address];
        let flags = sound_ram[(address + 1) & (SPU2_RAM_SIZE - 1)];
        let shift = (header & 0x0f).min(12);
        let filter = ((header >> 4) as usize).min(4);
        const COEFFICIENTS: [(i32, i32); 5] = [(0, 0), (60, 0), (115, -52), (98, -55), (122, -60)];
        let (c0, c1) = COEFFICIENTS[filter];
        for sample_index in 0..28 {
            let packed = sound_ram[(address + 2 + sample_index / 2) & (SPU2_RAM_SIZE - 1)];
            let nibble = if sample_index & 1 == 0 { packed & 0x0f } else { packed >> 4 };
            let signed = if nibble & 8 != 0 { i32::from(nibble) - 16 } else { i32::from(nibble) };
            let residual = (signed << 12) >> shift;
            let predicted = (voice.previous[0] * c0 + voice.previous[1] * c1 + 32) >> 6;
            let decoded = (residual + predicted).clamp(i16::MIN as i32, i16::MAX as i32);
            voice.samples[sample_index] = decoded as i16;
            voice.previous[1] = voice.previous[0];
            voice.previous[0] = decoded;
        }
        voice.block_flags = flags;
        if flags & 2 != 0 { voice.loop_address = Some(address); }
        voice.block_loaded = true;
    }
}
