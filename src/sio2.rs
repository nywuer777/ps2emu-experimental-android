//! Experimental PIO path for the PS2 SIO2 digital/analog controller protocol.

use std::collections::VecDeque;

pub const SIO2_BASE: u32 = 0x1f80_8200;
pub const SIO2_REGISTER_SIZE: u32 = 0x80;
const SEND3_BASE: usize = 0x00;
const SEND3_COUNT: usize = 16;
const FIFO_IN: usize = 0x60;
const FIFO_OUT: usize = 0x64;
const CONTROL: usize = 0x68;
const RECV1: usize = 0x6c;
const RECV2: usize = 0x70;
const RECV3: usize = 0x74;

#[derive(Clone)]
struct PadState {
    buttons: u16,
    sticks: [u8; 4],
    config_mode: bool,
    analog_mode: bool,
}

impl Default for PadState {
    fn default() -> Self {
        Self { buttons: 0xffff, sticks: [0x80; 4], config_mode: false, analog_mode: true }
    }
}

#[derive(Clone)]
pub struct Sio2 {
    send3: [u32; SEND3_COUNT],
    tx: VecDeque<u8>,
    rx: VecDeque<u8>,
    pads: [PadState; 2],
    recv1: u32,
    recv2: u32,
    recv3: u32,
    control: u32,
}

impl Default for Sio2 {
    fn default() -> Self { Self::new() }
}

impl Sio2 {
    pub fn new() -> Self {
        Self {
            send3: [0; SEND3_COUNT], tx: VecDeque::new(), rx: VecDeque::new(),
            pads: std::array::from_fn(|_| PadState::default()),
            recv1: 0x1d100, recv2: 0x0f, recv3: 0, control: 0,
        }
    }

    /// Buttons use the controller wire format: zero means pressed, one released.
    pub fn set_buttons(&mut self, port: usize, buttons: u16) {
        if let Some(pad) = self.pads.get_mut(port) { pad.buttons = buttons; }
    }

    /// Set right-X, right-Y, left-X, left-Y; 0x80 is centered.
    pub fn set_sticks(&mut self, port: usize, sticks: [u8; 4]) {
        if let Some(pad) = self.pads.get_mut(port) { pad.sticks = sticks; }
    }

    pub(crate) fn dma_push_word(&mut self, word: u32) {
        self.tx.extend(word.to_le_bytes());
    }

    pub(crate) fn dma_pop_word(&mut self) -> Option<u32> {
        if self.rx.is_empty() { return None; }
        let mut bytes = [0; 4];
        for byte in &mut bytes { *byte = self.rx.pop_front().unwrap_or(0); }
        Some(u32::from_le_bytes(bytes))
    }

    pub fn read_u8(&mut self, offset: u32) -> u8 {
        let offset = offset as usize;
        match offset {
            FIFO_OUT..=0x67 => self.rx.pop_front().unwrap_or(0),
            _ => self.read_u32((offset & !3) as u32).to_le_bytes()[offset & 3],
        }
    }

    pub fn read_u16(&mut self, offset: u32) -> u16 {
        let bytes = [self.read_u8(offset), self.read_u8(offset + 1)];
        u16::from_le_bytes(bytes)
    }

    pub fn read_u32(&mut self, offset: u32) -> u32 {
        let offset = offset as usize;
        match offset {
            FIFO_OUT => {
                let b = [self.rx.pop_front().unwrap_or(0), self.rx.pop_front().unwrap_or(0),
                    self.rx.pop_front().unwrap_or(0), self.rx.pop_front().unwrap_or(0)];
                u32::from_le_bytes(b)
            }
            CONTROL => self.control,
            RECV1 => self.recv1,
            RECV2 => self.recv2,
            RECV3 => self.recv3,
            SEND3_BASE..=0x3c if offset % 4 == 0 => self.send3[offset / 4],
            _ => 0,
        }
    }

    pub fn write_u8(&mut self, offset: u32, value: u8) {
        let offset = offset as usize;
        if offset == FIFO_IN { self.tx.push_back(value); }
        else if offset >= CONTROL && offset < CONTROL + 4 {
            let shift = (offset - CONTROL) * 8;
            let old = self.control;
            self.write_u32(CONTROL as u32, (old & !(0xff << shift)) | (u32::from(value) << shift));
        } else if (SEND3_BASE..SEND3_BASE + SEND3_COUNT * 4).contains(&offset) {
            let register = offset & !3;
            let shift = (offset & 3) * 8;
            let old = self.send3[register / 4];
            self.send3[register / 4] = (old & !(0xff << shift)) | (u32::from(value) << shift);
        }
    }

    pub fn write_u16(&mut self, offset: u32, value: u16) {
        let [a, b] = value.to_le_bytes();
        self.write_u8(offset, a);
        self.write_u8(offset + 1, b);
    }

    pub fn write_u32(&mut self, offset: u32, value: u32) {
        let offset = offset as usize;
        match offset {
            FIFO_IN => self.tx.extend(value.to_le_bytes()),
            CONTROL => {
                self.control = value & !0x0c;
                if value & 0x0c != 0 { self.tx.clear(); self.rx.clear(); }
                if value & 1 != 0 { self.execute(); }
            }
            SEND3_BASE..=0x3c if offset % 4 == 0 => self.send3[offset / 4] = value,
            _ => {}
        }
    }

    fn execute(&mut self) {
        self.rx.clear();
        self.recv1 = 0x1d100;
        self.recv2 = 0x0f;
        self.recv3 = 0;
        for descriptor in self.send3 {
            let length = ((descriptor >> 18) & 0x7f) as usize;
            if length == 0 { continue; }
            let port = (descriptor & 3) as usize;
            if self.tx.len() < length { break; }
            let packet: Vec<u8> = self.tx.drain(..length).collect();
            if port >= self.pads.len() { continue; }
            let response = Self::transact(&mut self.pads[port], &packet);
            self.recv1 = 0x1100;
            self.recv3 = self.recv3.wrapping_add(response.len() as u32);
            self.rx.extend(response);
        }
        self.tx.clear();
    }

    fn transact(pad: &mut PadState, packet: &[u8]) -> Vec<u8> {
        let length = packet.len();
        if length == 0 || packet[0] != 0x01 { return vec![0xff; length]; }
        let command = packet.get(1).copied().unwrap_or(0);
        let mut response = vec![0xff; length];
        if length > 2 { response[2] = 0x5a; }
        match command {
            0x42 => {
                if length > 1 { response[1] = Self::mode_byte(pad); }
                if length > 3 { response[3] = pad.buttons as u8; }
                if length > 4 { response[4] = (pad.buttons >> 8) as u8; }
                if pad.analog_mode && !pad.config_mode {
                    let protocol_sticks = [pad.sticks[1], pad.sticks[0], pad.sticks[3], pad.sticks[2]];
                    for (index, value) in protocol_sticks.into_iter().enumerate() {
                        if length > 5 + index { response[5 + index] = value; }
                    }
                    const PRESSURE_BITS: [u8; 12] = [5, 7, 4, 6, 12, 13, 14, 15, 10, 11, 8, 9];
                    for (index, bit) in PRESSURE_BITS.into_iter().enumerate() {
                        if length > 9 + index { response[9 + index] = if pad.buttons & (1 << bit) == 0 { 0xff } else { 0 }; }
                    }
                }
            }
            0x43 => {
                if pad.config_mode {
                    if length > 1 { response[1] = 0xf3; }
                    if packet.get(3).copied().unwrap_or(0) == 0 {
                        pad.config_mode = false;
                        for byte in response.iter_mut().skip(3) { *byte = 0; }
                    }
                } else {
                    pad.config_mode = packet.get(3).copied().unwrap_or(0) != 0;
                    if length > 1 { response[1] = Self::mode_byte(pad); }
                    if length > 3 { response[3] = pad.buttons as u8; }
                    if length > 4 { response[4] = (pad.buttons >> 8) as u8; }
                    let protocol_sticks = [pad.sticks[1], pad.sticks[0], pad.sticks[3], pad.sticks[2]];
                    for (index, value) in protocol_sticks.into_iter().enumerate() {
                        if length > 5 + index { response[5 + index] = value; }
                    }
                }
            }
            0x44 if pad.config_mode => {
                if packet.len() > 3 { pad.analog_mode = packet[3] != 0; }
                if length > 1 { response[1] = 0xf3; }
                for byte in response.iter_mut().skip(3) { *byte = 0; }
            }
            0x45 if pad.config_mode => {
                let values = [0x03, 0x02, u8::from(pad.analog_mode), 0x02, 0x01, 0x00];
                if length > 1 { response[1] = 0xf3; }
                for (index, value) in values.into_iter().enumerate() { if length > 3 + index { response[3 + index] = value; } }
            }
            _ => {
                if length > 1 { response[1] = Self::mode_byte(pad); }
            }
        }
        response
    }

    fn mode_byte(pad: &PadState) -> u8 {
        if pad.config_mode { 0xf3 } else if pad.analog_mode { 0x79 } else { 0x41 }
    }

}
