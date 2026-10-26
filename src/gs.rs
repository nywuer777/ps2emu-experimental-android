//! Initial software GS path: GIF packet decoding and a clipped RGBA framebuffer.

use std::fmt;

pub const GS_VRAM_SIZE: usize = 4 * 1024 * 1024;
pub const DEFAULT_GS_WIDTH: usize = 640;
pub const DEFAULT_GS_HEIGHT: usize = 448;

const REG_PRIM: u8 = 0x00;
const REG_RGBAQ: u8 = 0x01;
const REG_ST: u8 = 0x02;
const REG_UV: u8 = 0x03;
const REG_XYZ2: u8 = 0x05;
const REG_TEX0_1: u8 = 0x06;
const REG_CLAMP_1: u8 = 0x08;
const REG_TEX1_1: u8 = 0x14;
const REG_TEXA: u8 = 0x3b;
const REG_ALPHA_1: u8 = 0x42;
const REG_COLCLAMP: u8 = 0x46;
const REG_TEST_1: u8 = 0x47;
const REG_XYOFFSET_1: u8 = 0x18;
const REG_SCISSOR_1: u8 = 0x40;
const REG_FRAME_1: u8 = 0x4c;
const REG_ZBUF_1: u8 = 0x4e;
const REG_BITBLTBUF: u8 = 0x50;
const REG_TRXPOS: u8 = 0x51;
const REG_TRXREG: u8 = 0x52;
const REG_TRXDIR: u8 = 0x53;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GsError {
    InvalidDimensions { width: usize, height: usize },
    TruncatedGif { qword: usize, required: usize, available: usize },
}

impl fmt::Display for GsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidDimensions { width, height } => write!(f, "invalid GS framebuffer dimensions {width}x{height}"),
            Self::TruncatedGif { qword, required, available } => write!(f, "truncated GIF packet at qword {qword}: needs {required}, has {available}"),
        }
    }
}

impl std::error::Error for GsError {}

#[derive(Clone, Copy, Default)]
struct Vertex {
    x: i32,
    y: i32,
    z: u32,
    color: u32,
    u: i32,
    v: i32,
    s: f32,
    t: f32,
    q: f32,
}

/// Small software GS model for bring-up and deterministic framebuffer tests.
/// It is not a full GS: local memory implements PSMCT32, PSMCT16 and PSMCT16S swizzles.
/// Screen-space depth testing and ALPHA blending cover a small GS
/// state subset; VRAM Z formats, other PSMs, and field timing remain absent.
#[derive(Clone)]
pub struct Gs {
    width: usize,
    height: usize,
    pixels: Vec<u32>,
    depth: Vec<u32>,
    vram: Vec<u8>,
    primitive: u64,
    rgbaq: u32,
    q: f32,
    st: [f32; 2],
    uv: [i32; 2],
    tex0: u64,
    tex1: u64,
    texa: u64,
    clamp: u64,
    alpha: u64,
    colclamp: bool,
    test: u64,
    z_write_mask: bool,
    xyoffset_x: i32,
    xyoffset_y: i32,
    scissor: [usize; 4],
    frame_psm: u8,
    frame_mask: u32,
    depth_base: u32,
    depth_psm: u8,
    blit_sbp: u32,
    blit_sbw: u32,
    blit_spsm: u8,
    blit_dbp: u32,
    blit_dbw: u32,
    blit_dpsm: u8,
    trx_dsax: u32,
    trx_dsay: u32,
    trx_ssax: u32,
    trx_ssay: u32,
    trx_position_direction: u8,
    trx_width: u32,
    trx_height: u32,
    trx_direction: u8,
    vertices: Vec<Vertex>,
    primitive_vertices: u64,
    ignored_register_writes: u64,
    frame_base: u32,
    frame_buffer_width: u32,
}

impl Default for Gs {
    fn default() -> Self {
        Self::new(DEFAULT_GS_WIDTH, DEFAULT_GS_HEIGHT).expect("default GS dimensions are valid")
    }
}

impl Gs {
    pub fn new(width: usize, height: usize) -> Result<Self, GsError> {
        if width == 0 || height == 0 || width > 2048 || height > 2048 {
            return Err(GsError::InvalidDimensions { width, height });
        }
        Ok(Self {
            width,
            height,
            pixels: vec![0xff00_0000; width * height],
            depth: vec![0; width * height],
            vram: vec![0; GS_VRAM_SIZE],
            primitive: 0,
            rgbaq: 0xffff_ffff,
            q: 1.0,
            st: [0.0; 2],
            uv: [0; 2],
            tex0: 0,
            tex1: 0,
            texa: 0,
            clamp: 0,
            alpha: 0,
            colclamp: false,
            test: 0,
            z_write_mask: false,
            xyoffset_x: 0,
            xyoffset_y: 0,
            scissor: [0, 0, width - 1, height - 1],
            frame_psm: 0,
            frame_mask: 0,
            depth_base: 0,
            depth_psm: 48,
            blit_sbp: 0,
            blit_sbw: 1,
            blit_spsm: 0,
            blit_dbp: 0,
            blit_dbw: 1,
            blit_dpsm: 0,
            trx_dsax: 0,
            trx_dsay: 0,
            trx_ssax: 0,
            trx_ssay: 0,
            trx_position_direction: 0,
            trx_width: 0,
            trx_height: 0,
            trx_direction: 0,
            vertices: Vec::with_capacity(4),
            primitive_vertices: 0,
            ignored_register_writes: 0,
            frame_base: 0,
            frame_buffer_width: 1,
        })
    }

    pub fn dimensions(&self) -> (usize, usize) { (self.width, self.height) }

    /// Pixels are packed as `0xAABBGGRR`; rows are top-to-bottom.
    pub fn framebuffer(&self) -> &[u32] { &self.pixels }

    /// Experimental GS local-memory bytes, including PSMCT32 swizzling.
    pub fn vram(&self) -> &[u8] { &self.vram }

    /// Export the configured local-to-host rectangle as IMAGE qwords.
    /// Pixels are packed in transfer order, least-significant lane first.
    /// Only CT32/24/16/16S and T8/T4 source formats are modeled.
    pub fn readback_image(&self) -> Option<Vec<u128>> {
        if self.trx_direction != 1 || self.trx_width == 0 || self.trx_height == 0 {
            return None;
        }
        let bits_per_pixel = match self.blit_spsm {
            0 | 1 => 32,
            2 | 10 => 16,
            19 => 8,
            20 => 4,
            _ => return None,
        };
        let pixel_count = (self.trx_width as usize).checked_mul(self.trx_height as usize)?;
        let bits = pixel_count.checked_mul(bits_per_pixel)?;
        let qword_count = bits.checked_add(127)? / 128;
        let mut output = Vec::new();
        output.try_reserve_exact(qword_count).ok()?;
        output.resize(qword_count, 0u128);
        let pixels_per_row = self.trx_width as usize;
        for index in 0..pixel_count {
            let x = index % pixels_per_row;
            let y = index / pixels_per_row;
            let source_x = Self::transfer_coordinate(self.trx_ssax, x, self.trx_position_direction & 1 != 0);
            let source_y = Self::transfer_coordinate(self.trx_ssay, y, self.trx_position_direction & 2 != 0);
            let Some(pixel) = self.read_transfer_pixel(source_x, source_y) else { return None; };
            let bit_offset = index * bits_per_pixel;
            output[bit_offset / 128] |= (pixel as u128) << (bit_offset % 128);
        }
        Some(output)
    }

    fn read_transfer_pixel(&self, x: usize, y: usize) -> Option<u32> {
        let pixel = match self.blit_spsm {
            0 | 1 => {
                let address = Self::psmct32_address(self.blit_sbp, self.blit_sbw, x, y);
                let raw = u32::from_le_bytes(self.vram.get(address..address + 4)?.try_into().ok()?);
                if self.blit_spsm == 1 { raw & 0x00ff_ffff } else { raw }
            }
            2 | 10 => {
                let address = Self::psmct16_address(self.blit_sbp, self.blit_sbw, x, y, self.blit_spsm == 10);
                u16::from_le_bytes(self.vram.get(address..address + 2)?.try_into().ok()?) as u32
            }
            19 => self.vram.get(Self::psmt8_address(self.blit_sbp, self.blit_sbw, x, y)).copied()? as u32,
            20 => {
                let (address, high_nibble) = Self::psmt4_address(self.blit_sbp, self.blit_sbw, x, y);
                let byte = *self.vram.get(address)?;
                if high_nibble { (byte >> 4) as u32 } else { (byte & 0xf) as u32 }
            }
            _ => return None,
        };
        Some(pixel)
    }

    fn transfer_coordinate(start: u32, offset: usize, reverse: bool) -> usize {
        let start = start as usize;
        if reverse { start.wrapping_sub(offset) & 0x7ff } else { start.wrapping_add(offset) & 0x7ff }
    }

    /// Encode the current surface as binary PPM (P6), which can be opened by
    /// common image tools without adding an image codec dependency.
    pub fn encode_ppm(&self) -> Vec<u8> {
        let mut output = format!("P6\n{} {}\n255\n", self.width, self.height).into_bytes();
        output.reserve(self.pixels.len().saturating_mul(3));
        for pixel in &self.pixels {
            output.push(*pixel as u8);
            output.push((*pixel >> 8) as u8);
            output.push((*pixel >> 16) as u8);
        }
        output
    }

    pub fn framebuffer_mut(&mut self) -> &mut [u32] { &mut self.pixels }

    pub fn clear(&mut self, rgba: u32) { self.pixels.fill(rgba); }

    pub fn ignored_register_writes(&self) -> u64 { self.ignored_register_writes }

    /// Consume complete GIF tags from a stream of little-endian 128-bit qwords.
    /// Returns the number of qwords consumed. PACKED, REGLIST, and image-mode
    /// payload lengths are decoded; IMAGE writes support swizzled PSMCT32 and
    /// PSMCT16 host-to-local transfers.
    pub fn submit_gif(&mut self, qwords: &[u128]) -> Result<usize, GsError> {
        let mut cursor = 0usize;
        while cursor < qwords.len() {
            let tag_index = cursor;
            let tag = qwords[cursor];
            cursor += 1;
            let loops = (tag & 0x7fff) as usize;
            let end_of_packet = (tag >> 15) & 1 != 0;
            let pre = (tag >> 46) & 1 != 0;
            let prim = (tag >> 47) & 0x7ff;
            let mode = ((tag >> 58) & 3) as u8;
            let register_count = match ((tag >> 60) & 0xf) as usize { 0 => 16, n => n };
            let descriptors = (tag >> 64) as u64;

            if pre { self.write_register(REG_PRIM, prim as u64); }
            let required = match mode {
                0 => loops.saturating_mul(register_count),
                1 => loops.saturating_mul(register_count).div_ceil(2),
                2 | 3 => loops,
                _ => unreachable!(),
            };
            if cursor.saturating_add(required) > qwords.len() {
                return Err(GsError::TruncatedGif { qword: tag_index, required, available: qwords.len() - cursor });
            }

            match mode {
                0 => {
                    for item in 0..required {
                        let packed = qwords[cursor];
                        cursor += 1;
                        let descriptor = ((descriptors >> ((item % register_count) * 4)) & 0xf) as u8;
                        if descriptor == 0xe {
                            self.write_register((packed >> 64) as u8, packed as u64);
                        } else {
                            self.write_register(descriptor, packed as u64);
                        }
                    }
                }
                1 => {
                    let total = loops * register_count;
                    for item in 0..total {
                        let packed = qwords[cursor + item / 2];
                        let value = if item & 1 == 0 { packed as u64 } else { (packed >> 64) as u64 };
                        let descriptor = ((descriptors >> ((item % register_count) * 4)) & 0xf) as u8;
                        self.write_register(descriptor, value);
                    }
                    cursor += required;
                }
                2 | 3 => {
                    self.transfer_image(&qwords[cursor..cursor + required]);
                    cursor += required;
                }
                _ => unreachable!(),
            }
            if end_of_packet { break; }
        }
        Ok(cursor)
    }

    pub fn write_register(&mut self, register: u8, value: u64) {
        match register {
            REG_PRIM => {
                self.primitive = value & 0x7ff;
                self.vertices.clear();
                self.primitive_vertices = 0;
            }
            REG_RGBAQ => {
                self.rgbaq = value as u32;
                self.q = f32::from_bits((value >> 32) as u32);
            }
            REG_ST => {
                self.st = [
                    f32::from_bits(value as u32),
                    f32::from_bits((value >> 32) as u32),
                ];
            }
            REG_UV => {
                self.uv = [
                    (value & 0x3fff) as i32,
                    ((value >> 16) & 0x3fff) as i32,
                ];
            }
            REG_TEX0_1 => self.tex0 = value,
            REG_TEX1_1 => self.tex1 = value,
            REG_CLAMP_1 => self.clamp = value,
            REG_TEXA => self.texa = value,
            REG_ALPHA_1 => self.alpha = value,
            REG_COLCLAMP => self.colclamp = value & 1 != 0,
            REG_TEST_1 => self.test = value,
            REG_XYZ2 => self.submit_vertex(value),
            REG_XYOFFSET_1 => {
                self.xyoffset_x = (value & 0xffff) as i32;
                self.xyoffset_y = ((value >> 32) & 0xffff) as i32;
            }
            REG_SCISSOR_1 => {
                let x0 = (value & 0x7ff) as usize;
                let x1 = ((value >> 16) & 0x7ff) as usize;
                let y0 = ((value >> 32) & 0x7ff) as usize;
                let y1 = ((value >> 48) & 0x7ff) as usize;
                self.scissor = [x0, y0, x1, y1];
            }
            REG_FRAME_1 => {
                self.frame_base = (value & 0x1ff) as u32;
                self.frame_buffer_width = (((value >> 16) & 0x3f) as u32).max(1);
                self.frame_psm = ((value >> 24) & 0x3f) as u8;
                self.frame_mask = (value >> 32) as u32;
            }
            REG_ZBUF_1 => {
                self.depth_base = (value & 0x1ff) as u32;
                self.depth_psm = ((value >> 24) & 0x3f) as u8;
                self.z_write_mask = value & (1 << 32) != 0;
            }
            REG_BITBLTBUF => {
                self.blit_sbp = (value & 0x3fff) as u32;
                self.blit_sbw = ((value >> 16) & 0x3f) as u32;
                self.blit_spsm = ((value >> 24) & 0x3f) as u8;
                self.blit_dbp = ((value >> 32) & 0x3fff) as u32;
                self.blit_dbw = ((value >> 48) & 0x3f) as u32;
                self.blit_dpsm = ((value >> 56) & 0x3f) as u8;
            }
            REG_TRXPOS => {
                self.trx_ssax = (value & 0x7ff) as u32;
                self.trx_ssay = ((value >> 16) & 0x7ff) as u32;
                self.trx_dsax = ((value >> 32) & 0x7ff) as u32;
                self.trx_dsay = ((value >> 48) & 0x7ff) as u32;
                self.trx_position_direction = ((value >> 59) & 3) as u8;
            }
            REG_TRXREG => {
                self.trx_width = (value & 0xfff) as u32;
                self.trx_height = ((value >> 32) & 0xfff) as u32;
            }
            REG_TRXDIR => {
                self.trx_direction = (value & 3) as u8;
                if self.trx_direction == 2 { self.transfer_local_to_local(); }
            }
            _ => self.ignored_register_writes = self.ignored_register_writes.wrapping_add(1),
        }
    }

    fn transfer_image(&mut self, qwords: &[u128]) {
        // Host-to-local IMAGE transfers support CT32, CT24, CT16, CT16S, T8 and T4.
        if self.trx_direction != 0 || !matches!(self.blit_dpsm, 0 | 1 | 2 | 10 | 19 | 20) || self.trx_width == 0 || self.trx_height == 0 {
            return;
        }
        let pixel_count = (self.trx_width as usize).saturating_mul(self.trx_height as usize);
        let mut pixel_index = 0usize;
        for qword in qwords {
            if self.blit_dpsm == 19 {
                for lane in 0..16 {
                    if pixel_index >= pixel_count { return; }
                    let x = pixel_index % self.trx_width as usize;
                    let y = pixel_index / self.trx_width as usize;
                    let address = Self::psmt8_address(
                        self.blit_dbp,
                        self.blit_dbw,
                        Self::transfer_coordinate(self.trx_dsax, x, self.trx_position_direction & 1 != 0),
                        Self::transfer_coordinate(self.trx_dsay, y, self.trx_position_direction & 2 != 0),
                    );
                    self.vram[address] = (*qword >> (lane * 8)) as u8;
                    pixel_index += 1;
                }
            } else if self.blit_dpsm == 20 {
                for lane in 0..32 {
                    if pixel_index >= pixel_count { return; }
                    let x = pixel_index % self.trx_width as usize;
                    let y = pixel_index / self.trx_width as usize;
                    let (address, high_nibble) = Self::psmt4_address(
                        self.blit_dbp,
                        self.blit_dbw,
                        Self::transfer_coordinate(self.trx_dsax, x, self.trx_position_direction & 1 != 0),
                        Self::transfer_coordinate(self.trx_dsay, y, self.trx_position_direction & 2 != 0),
                    );
                    let index = ((*qword >> (lane * 4)) & 0xf) as u8;
                    let old = self.vram[address];
                    self.vram[address] = if high_nibble {
                        (old & 0x0f) | (index << 4)
                    } else {
                        (old & 0xf0) | index
                    };
                    pixel_index += 1;
                }
            } else if !matches!(self.blit_dpsm, 2 | 10) {
                for lane in 0..4 {
                    if pixel_index >= pixel_count { return; }
                    let x = pixel_index % self.trx_width as usize;
                    let y = pixel_index / self.trx_width as usize;
                    let address = Self::psmct32_address(
                        self.blit_dbp,
                        self.blit_dbw,
                        Self::transfer_coordinate(self.trx_dsax, x, self.trx_position_direction & 1 != 0),
                        Self::transfer_coordinate(self.trx_dsay, y, self.trx_position_direction & 2 != 0),
                    );
                    let pixel = (*qword >> (lane * 32)) as u32;
                    if self.blit_dpsm == 1 {
                        let old = u32::from_le_bytes(self.vram[address..address + 4].try_into().expect("wrapped GS image address"));
                        let pixel = (old & 0xff00_0000) | (pixel & 0x00ff_ffff);
                        self.vram[address..address + 4].copy_from_slice(&pixel.to_le_bytes());
                    } else {
                        self.vram[address..address + 4].copy_from_slice(&pixel.to_le_bytes());
                    }
                    pixel_index += 1;
                }
            } else {
                for lane in 0..8 {
                    if pixel_index >= pixel_count { return; }
                    let x = pixel_index % self.trx_width as usize;
                    let y = pixel_index / self.trx_width as usize;
                    let address = Self::psmct16_address(
                        self.blit_dbp,
                        self.blit_dbw,
                        Self::transfer_coordinate(self.trx_dsax, x, self.trx_position_direction & 1 != 0),
                        Self::transfer_coordinate(self.trx_dsay, y, self.trx_position_direction & 2 != 0),
                        self.blit_dpsm == 10,
                    );
                    let pixel = (*qword >> (lane * 16)) as u16;
                    self.vram[address..address + 2].copy_from_slice(&pixel.to_le_bytes());
                    pixel_index += 1;
                }
            }
        }
    }

    fn transfer_local_to_local(&mut self) {
        if self.blit_spsm != self.blit_dpsm
            || !matches!(self.blit_spsm, 0 | 1 | 2 | 10 | 19 | 20)
            || self.trx_width == 0
            || self.trx_height == 0
        {
            return;
        }
        let Some(pixel_count) = (self.trx_width as usize).checked_mul(self.trx_height as usize) else { return; };
        let mut pixels = Vec::new();
        if pixels.try_reserve_exact(pixel_count).is_err() { return; }
        let width = self.trx_width as usize;
        for index in 0..pixel_count {
            let x = index % width;
            let y = index / width;
            let sx = Self::transfer_coordinate(self.trx_ssax, x, self.trx_position_direction & 1 != 0);
            let sy = Self::transfer_coordinate(self.trx_ssay, y, self.trx_position_direction & 2 != 0);
            let Some(pixel) = self.read_transfer_pixel(sx, sy) else { return; };
            pixels.push(pixel);
        }
        for (index, pixel) in pixels.into_iter().enumerate() {
            let x = index % width;
            let y = index / width;
            let dx = Self::transfer_coordinate(self.trx_dsax, x, self.trx_position_direction & 1 != 0);
            let dy = Self::transfer_coordinate(self.trx_dsay, y, self.trx_position_direction & 2 != 0);
            if !self.write_transfer_pixel(dx, dy, pixel) { return; }
        }
    }

    fn write_transfer_pixel(&mut self, x: usize, y: usize, pixel: u32) -> bool {
        match self.blit_dpsm {
            0 | 1 => {
                let address = Self::psmct32_address(self.blit_dbp, self.blit_dbw, x, y);
                let old = u32::from_le_bytes(self.vram[address..address + 4].try_into().expect("wrapped GS transfer address"));
                let value = if self.blit_dpsm == 1 { (old & 0xff00_0000) | (pixel & 0x00ff_ffff) } else { pixel };
                self.vram[address..address + 4].copy_from_slice(&value.to_le_bytes());
            }
            2 | 10 => {
                let address = Self::psmct16_address(self.blit_dbp, self.blit_dbw, x, y, self.blit_dpsm == 10);
                self.vram[address..address + 2].copy_from_slice(&(pixel as u16).to_le_bytes());
            }
            19 => {
                let address = Self::psmt8_address(self.blit_dbp, self.blit_dbw, x, y);
                self.vram[address] = pixel as u8;
            }
            20 => {
                let (address, high_nibble) = Self::psmt4_address(self.blit_dbp, self.blit_dbw, x, y);
                let old = self.vram[address];
                self.vram[address] = if high_nibble { (old & 0x0f) | (((pixel as u8) & 0xf) << 4) } else { (old & 0xf0) | ((pixel as u8) & 0xf) };
            }
            _ => return false,
        }
        true
    }

    fn submit_vertex(&mut self, value: u64) {
        let vertex = Vertex {
            x: ((value & 0xffff) as i16 as i32) - self.xyoffset_x,
            y: (((value >> 16) & 0xffff) as i16 as i32) - self.xyoffset_y,
            z: ((value >> 32) & 0x00ff_ffff) as u32,
            color: self.rgbaq,
            u: self.uv[0],
            v: self.uv[1],
            s: self.st[0],
            t: self.st[1],
            q: self.q,
        };
        match (self.primitive & 7) as u8 {
            0 => self.draw_point(vertex),
            1 => {
                self.vertices.push(vertex);
                if self.vertices.len() == 2 {
                    self.draw_line(self.vertices[0], self.vertices[1]);
                    self.vertices.clear();
                }
            }
            2 => {
                if let Some(previous) = self.vertices.last().copied() { self.draw_line(previous, vertex); }
                self.vertices.clear();
                self.vertices.push(vertex);
            }
            3 => {
                self.vertices.push(vertex);
                if self.vertices.len() == 3 {
                    let vertices = [self.vertices[0], self.vertices[1], self.vertices[2]];
                    self.draw_triangle(vertices);
                    self.vertices.clear();
                }
            }
            4 => {
                self.vertices.push(vertex);
                if self.vertices.len() >= 3 {
                    let len = self.vertices.len();
                    let mut triangle = [self.vertices[len - 3], self.vertices[len - 2], self.vertices[len - 1]];
                    if self.primitive_vertices & 1 != 0 { triangle.swap(0, 1); }
                    self.draw_triangle(triangle);
                    self.vertices.drain(..len - 2);
                }
            }
            5 => {
                self.vertices.push(vertex);
                if self.vertices.len() >= 3 {
                    let len = self.vertices.len();
                    let anchor = self.vertices[0];
                    let previous = self.vertices[len - 1];
                    self.draw_triangle([anchor, self.vertices[len - 2], previous]);
                    self.vertices.clear();
                    self.vertices.push(anchor);
                    self.vertices.push(previous);
                }
            }
            6 => {
                self.vertices.push(vertex);
                if self.vertices.len() == 2 {
                    self.draw_sprite(self.vertices[0], self.vertices[1]);
                    self.vertices.clear();
                }
            }
            _ => {}
        }
        self.primitive_vertices = self.primitive_vertices.wrapping_add(1);
    }

    fn clip_bounds(&self) -> Option<(usize, usize, usize, usize)> {
        let [x0, y0, x1, y1] = self.scissor;
        let left = x0.min(self.width);
        let top = y0.min(self.height);
        let right = x1.min(self.width - 1);
        let bottom = y1.min(self.height - 1);
        (left <= right && top <= bottom).then_some((left, top, right, bottom))
    }

    fn draw_point(&mut self, vertex: Vertex) {
        let x = vertex.x.div_euclid(16);
        let y = vertex.y.div_euclid(16);
        if x >= 0 && y >= 0 { self.put_pixel(x as usize, y as usize, vertex.color, vertex.z); }
    }

    fn draw_line(&mut self, a: Vertex, b: Vertex) {
        let (mut x0, mut y0) = (a.x.div_euclid(16), a.y.div_euclid(16));
        let (x1, y1) = (b.x.div_euclid(16), b.y.div_euclid(16));
        let dx = (x1 - x0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let dy = -(y1 - y0).abs();
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut error = dx + dy;
        let steps = dx.max(-dy).max(1);
        let mut step = 0;
        loop {
            let z = (i64::from(a.z)
                + (i64::from(b.z) - i64::from(a.z)) * i64::from(step) / i64::from(steps)) as u32;
            if x0 >= 0 && y0 >= 0 { self.put_pixel(x0 as usize, y0 as usize, a.color, z); }
            if x0 == x1 && y0 == y1 { break; }
            let twice = error * 2;
            if twice >= dy { error += dy; x0 += sx; }
            if twice <= dx { error += dx; y0 += sy; }
            step += 1;
        }
    }

    fn draw_sprite(&mut self, a: Vertex, b: Vertex) {
        let Some((clip_x0, clip_y0, clip_x1, clip_y1)) = self.clip_bounds() else { return; };
        let left = a.x.min(b.x).div_euclid(16).max(clip_x0 as i32);
        let top = a.y.min(b.y).div_euclid(16).max(clip_y0 as i32);
        let right = a.x.max(b.x).div_euclid(16).min(clip_x1 as i32);
        let bottom = a.y.max(b.y).div_euclid(16).min(clip_y1 as i32);
        if left > right || top > bottom { return; }
        let (x0, y0, x1, y1) = (left as usize, top as usize, right as usize, bottom as usize);
        for y in y0..=y1 {
            for x in x0..=x1 {
                let color = if self.texturing_enabled() {
                    let (u, v) = self.sprite_texture_coordinates(a, b, x as i32 * 16 + 8, y as i32 * 16 + 8);
                    self.texture_color(u, v, a.color)
                } else { a.color };
                let z = Self::interpolate_axis(a.z as i32, b.z as i32, x as i32 * 16 + 8, a.x, b.x);
                self.put_pixel_unchecked(x, y, color, z.max(0) as u32);
            }
        }
    }

    fn draw_triangle(&mut self, mut vertices: [Vertex; 3]) {
        let mut area = Self::edge(vertices[0], vertices[1], vertices[2].x, vertices[2].y);
        if area == 0 { return; }
        if area < 0 {
            vertices.swap(1, 2);
            area = -area;
        }
        let Some((clip_x0, clip_y0, clip_x1, clip_y1)) = self.clip_bounds() else { return; };
        let min_x = vertices.iter().map(|v| v.x).min().unwrap().div_euclid(16).max(clip_x0 as i32);
        let min_y = vertices.iter().map(|v| v.y).min().unwrap().div_euclid(16).max(clip_y0 as i32);
        let max_x = vertices.iter().map(|v| v.x).max().unwrap().div_euclid(16).min(clip_x1 as i32);
        let max_y = vertices.iter().map(|v| v.y).max().unwrap().div_euclid(16).min(clip_y1 as i32);
        if min_x > max_x || min_y > max_y { return; }
        let gouraud = self.primitive & (1 << 3) != 0;
        let flat_color = vertices[0].color;
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let px = x * 16 + 8;
                let py = y * 16 + 8;
                let w0 = Self::edge(vertices[1], vertices[2], px, py);
                let w1 = Self::edge(vertices[2], vertices[0], px, py);
                let w2 = Self::edge(vertices[0], vertices[1], px, py);
                if w0 < 0 || w1 < 0 || w2 < 0 { continue; }
                let color = if gouraud { Self::interpolate_color(vertices.map(|v| v.color), [w0, w1, w2], area) } else { flat_color };
                let color = if self.texturing_enabled() {
                    let (u, v) = self.triangle_texture_coordinates(vertices, [w0, w1, w2], area);
                    self.texture_color(u, v, color)
                } else { color };
                let z = Self::interpolate_scalar(vertices.map(|v| v.z as i32), [w0, w1, w2], area).max(0) as u32;
                self.put_pixel_unchecked(x as usize, y as usize, color, z);
            }
        }
    }

    fn edge(a: Vertex, b: Vertex, x: i32, y: i32) -> i64 {
        i64::from(x - a.x) * i64::from(b.y - a.y) - i64::from(y - a.y) * i64::from(b.x - a.x)
    }

    fn psmct32_address(base_pointer: u32, buffer_width: u32, x: usize, y: usize) -> usize {
        // PSMCT32 local memory uses 64x32-pixel pages, 8x8-pixel blocks,
        // and Morton ordering inside both a page and a block.
        let page_width = buffer_width.max(1) as usize;
        let page = (y / 32) * page_width + (x / 64);
        let page_x = x & 63;
        let page_y = y & 31;
        let block = Self::morton(page_x / 8, page_y / 8, 3, 2);
        let pixel = Self::morton(page_x & 7, page_y & 7, 3, 3);
        ((base_pointer as usize * 256) + page * 8192 + block * 256 + pixel * 4)
            & (GS_VRAM_SIZE - 1)
    }

    fn psmct16_address(base_pointer: u32, buffer_width: u32, x: usize, y: usize, signed: bool) -> usize {
        // PSMCT16 uses 64x64 pages, 16x8 blocks, then four 16x2 columns.
        const BLOCK: [usize; 32] = [
            0, 2, 8, 10, 1, 3, 9, 11, 4, 6, 12, 14, 5, 7, 13, 15,
            16, 18, 24, 26, 17, 19, 25, 27, 20, 22, 28, 30, 21, 23, 29, 31,
        ];
        const BLOCK_SIGNED: [usize; 32] = [
            0, 2, 16, 18, 1, 3, 17, 19, 8, 10, 24, 26, 9, 11, 25, 27,
            4, 6, 20, 22, 5, 7, 21, 23, 12, 14, 28, 30, 13, 15, 29, 31,
        ];
        const COLUMN_WORD: [usize; 32] = [
            0, 1, 4, 5, 8, 9, 12, 13, 0, 1, 4, 5, 8, 9, 12, 13,
            2, 3, 6, 7, 10, 11, 14, 15, 2, 3, 6, 7, 10, 11, 14, 15,
        ];
        const COLUMN_HALF: [usize; 32] = [
            0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
        ];
        let page_width = buffer_width.max(1) as usize;
        let page = (y / 64) * page_width + (x / 64);
        let page_x = x & 63;
        let page_y = y & 63;
        let block_index = (page_y / 8) * 4 + page_x / 16;
        let block = if signed { BLOCK_SIGNED[block_index] } else { BLOCK[block_index] };
        let in_column = ((page_y & 1) * 16) + (page_x & 15);
        let column = (page_y & 7) / 2;
        let byte_address = (base_pointer as usize * 256)
            + page * 8192
            + block * 256
            + column * 64
            + COLUMN_WORD[in_column] * 4
            + COLUMN_HALF[in_column] * 2;
        byte_address & (GS_VRAM_SIZE - 1)
    }

    fn psmz32_address(base_pointer: u32, buffer_width: u32, x: usize, y: usize) -> usize {
        // PSMZ32/24 share the 64x32 page and 8x8 block shape, with Z-order tables.
        const BLOCK_Z32: [usize; 32] = [
            24, 25, 28, 29, 8, 9, 12, 13,
            26, 27, 30, 31, 10, 11, 14, 15,
            16, 17, 20, 21, 0, 1, 4, 5,
            18, 19, 22, 23, 2, 3, 6, 7,
        ];
        const COLUMN_WORD_Z32: [usize; 16] = [
            0, 1, 4, 5, 8, 9, 12, 13,
            2, 3, 6, 7, 10, 11, 14, 15,
        ];
        let page_width = buffer_width.max(1) as usize;
        let page = (y / 32) * page_width + (x / 64);
        let page_x = x & 63;
        let page_y = y & 31;
        let block_x = page_x / 8;
        let block_y = page_y / 8;
        let block = BLOCK_Z32[block_x + block_y * 8];
        let in_block_x = page_x & 7;
        let in_block_y = page_y & 7;
        let column = in_block_y / 2;
        let column_word = COLUMN_WORD_Z32[in_block_x + (in_block_y & 1) * 8];
        ((base_pointer as usize * 256) + page * 8192 + block * 256 + column * 64 + column_word * 4)
            & (GS_VRAM_SIZE - 1)
    }

    fn psmz16_address(base_pointer: u32, buffer_width: u32, x: usize, y: usize, signed: bool) -> usize {
        const BLOCK_Z16: [usize; 32] = [
            24, 26, 16, 18, 25, 27, 17, 19, 28, 30, 20, 22, 29, 31, 21, 23,
            8, 10, 0, 2, 9, 11, 1, 3, 12, 14, 4, 6, 13, 15, 5, 7,
        ];
        const BLOCK_Z16S: [usize; 32] = [
            24, 26, 8, 10, 25, 27, 9, 11, 16, 18, 0, 2, 17, 19, 1, 3,
            28, 30, 12, 14, 29, 31, 13, 15, 20, 22, 4, 6, 21, 23, 5, 7,
        ];
        const COLUMN_WORD: [usize; 32] = [
            0, 1, 4, 5, 8, 9, 12, 13, 0, 1, 4, 5, 8, 9, 12, 13,
            2, 3, 6, 7, 10, 11, 14, 15, 2, 3, 6, 7, 10, 11, 14, 15,
        ];
        const COLUMN_HALF: [usize; 32] = [
            0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
            0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1,
        ];
        let page_width = buffer_width.max(1) as usize;
        let page = (y / 64) * page_width + (x / 64);
        let page_x = x & 63;
        let page_y = y & 63;
        let block_index = (page_y / 8) * 4 + page_x / 16;
        let block = if signed { BLOCK_Z16S[block_index] } else { BLOCK_Z16[block_index] };
        let in_column = ((page_y & 1) * 16) + (page_x & 15);
        let column = (page_y & 7) / 2;
        let byte_address = (base_pointer as usize * 256)
            + page * 8192
            + block * 256
            + column * 64
            + COLUMN_WORD[in_column] * 4
            + COLUMN_HALF[in_column] * 2;
        byte_address & (GS_VRAM_SIZE - 1)
    }

    fn psmt8_address(base_pointer: u32, buffer_width: u32, x: usize, y: usize) -> usize {
        const BLOCK: [usize; 32] = [
            0, 1, 4, 5, 16, 17, 20, 21,
            2, 3, 6, 7, 18, 19, 22, 23,
            8, 9, 12, 13, 24, 25, 28, 29,
            10, 11, 14, 15, 26, 27, 30, 31,
        ];
        const COLUMN_WORD_EVEN: [usize; 64] = [
            0, 1, 4, 5, 8, 9, 12, 13, 0, 1, 4, 5, 8, 9, 12, 13,
            2, 3, 6, 7, 10, 11, 14, 15, 2, 3, 6, 7, 10, 11, 14, 15,
            8, 9, 12, 13, 0, 1, 4, 5, 8, 9, 12, 13, 0, 1, 4, 5,
            10, 11, 14, 15, 2, 3, 6, 7, 10, 11, 14, 15, 2, 3, 6, 7,
        ];
        const COLUMN_WORD_ODD: [usize; 64] = [
            8, 9, 12, 13, 0, 1, 4, 5, 8, 9, 12, 13, 0, 1, 4, 5,
            10, 11, 14, 15, 2, 3, 6, 7, 10, 11, 14, 15, 2, 3, 6, 7,
            0, 1, 4, 5, 8, 9, 12, 13, 0, 1, 4, 5, 8, 9, 12, 13,
            2, 3, 6, 7, 10, 11, 14, 15, 2, 3, 6, 7, 10, 11, 14, 15,
        ];
        const COLUMN_BYTE: [usize; 64] = [
            0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2,
            0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2,
            1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3,
            1, 1, 1, 1, 1, 1, 1, 1, 3, 3, 3, 3, 3, 3, 3, 3,
        ];
        let page_width = (buffer_width as usize / 2).max(1);
        let page = (y / 64) * page_width + (x / 128);
        let page_x = x & 127;
        let page_y = y & 63;
        let block_x = page_x / 16;
        let block_y = page_y / 16;
        let block = BLOCK[block_x + block_y * 8];
        let in_block_x = page_x & 15;
        let in_block_y = page_y & 15;
        let column = in_block_y / 4;
        let table_index = in_block_x + (in_block_y & 3) * 16;
        let column_word = if column & 1 == 0 { COLUMN_WORD_EVEN[table_index] } else { COLUMN_WORD_ODD[table_index] };
        let byte_address = base_pointer as usize * 256
            + page * 8192
            + block * 256
            + column * 64
            + column_word * 4
            + COLUMN_BYTE[table_index];
        byte_address & (GS_VRAM_SIZE - 1)
    }

    fn psmt4_address(base_pointer: u32, buffer_width: u32, x: usize, y: usize) -> (usize, bool) {
        const BLOCK: [usize; 32] = [
            0, 2, 8, 10, 1, 3, 9, 11,
            4, 6, 12, 14, 5, 7, 13, 15,
            16, 18, 24, 26, 17, 19, 25, 27,
            20, 22, 28, 30, 21, 23, 29, 31,
        ];
        const A: [usize; 8] = [0, 1, 4, 5, 8, 9, 12, 13];
        const B: [usize; 8] = [2, 3, 6, 7, 10, 11, 14, 15];
        const C: [usize; 8] = [8, 9, 12, 13, 0, 1, 4, 5];
        const D: [usize; 8] = [10, 11, 14, 15, 2, 3, 6, 7];

        let page_width = (buffer_width as usize / 2).max(1);
        let page = (y / 128) * page_width + (x / 128);
        let page_x = x & 127;
        let page_y = y & 127;
        let block_x = page_x / 32;
        let block_y = page_y / 16;
        let block = BLOCK[block_x + block_y * 4];
        let in_block_x = page_x & 31;
        let in_block_y = page_y & 15;
        let column = in_block_y / 4;
        let table_x = in_block_x;
        let table_y = in_block_y & 3;
        let row = match (column & 1, table_y) {
            (0, 0) | (1, 2) => A[table_x & 7],
            (0, 1) | (1, 3) => B[table_x & 7],
            (0, 2) | (1, 0) => C[table_x & 7],
            (0, 3) | (1, 1) => D[table_x & 7],
            _ => unreachable!(),
        };
        let column_byte = (table_x / 8) * 2 + (table_y & 1);
        let byte_address = base_pointer as usize * 256
            + page * 8192
            + block * 256
            + column * 64
            + row * 4
            + column_byte / 2;
        (byte_address & (GS_VRAM_SIZE - 1), column_byte & 1 != 0)
    }

    fn morton(x: usize, y: usize, x_bits: usize, y_bits: usize) -> usize {
        let mut result = 0usize;
        for bit in 0..x_bits.max(y_bits) {
            if bit < x_bits { result |= ((x >> bit) & 1) << (bit * 2); }
            if bit < y_bits { result |= ((y >> bit) & 1) << (bit * 2 + 1); }
        }
        result
    }

    fn interpolate_color(colors: [u32; 3], weights: [i64; 3], area: i64) -> u32 {
        let mut result = 0u32;
        for channel in 0..4 {
            let shift = channel * 8;
            let sum = (0..3).map(|i| i64::from((colors[i] >> shift) & 0xff) * weights[i]).sum::<i64>();
            let value = ((sum + area / 2) / area).clamp(0, 255) as u32;
            result |= value << shift;
        }
        result
    }

    fn interpolate_scalar(values: [i32; 3], weights: [i64; 3], area: i64) -> i32 {
        let sum = (0..3).map(|i| i64::from(values[i]) * weights[i]).sum::<i64>();
        ((sum + area / 2) / area).clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32
    }

    fn interpolate_axis(a_value: i32, b_value: i32, position: i32, a_pos: i32, b_pos: i32) -> i32 {
        let delta = i64::from(b_pos) - i64::from(a_pos);
        if delta == 0 { return a_value; }
        let offset = i64::from(position) - i64::from(a_pos);
        (i64::from(a_value) + (i64::from(b_value - a_value) * offset / delta)) as i32
    }

    fn texturing_enabled(&self) -> bool {
        self.primitive & (1 << 4) != 0
    }

    fn triangle_texture_coordinates(&self, vertices: [Vertex; 3], weights: [i64; 3], area: i64) -> (i32, i32) {
        if self.primitive & (1 << 8) != 0 {
            return (
                Self::interpolate_scalar(vertices.map(|v| v.u), weights, area),
                Self::interpolate_scalar(vertices.map(|v| v.v), weights, area),
            );
        }
        let mut s_over_q = 0.0f64;
        let mut t_over_q = 0.0f64;
        let mut one_over_q = 0.0f64;
        for i in 0..3 {
            let q = f64::from(vertices[i].q);
            if !q.is_finite() || q.abs() < 1.0e-12 { continue; }
            let weight = weights[i] as f64 / area as f64;
            s_over_q += f64::from(vertices[i].s) / q * weight;
            t_over_q += f64::from(vertices[i].t) / q * weight;
            one_over_q += 1.0 / q * weight;
        }
        if !one_over_q.is_finite() || one_over_q.abs() < 1.0e-12 { return (0, 0); }
        let scale = 16.0 * self.texture_dimensions().0 as f64;
        let s = (s_over_q / one_over_q * scale).round();
        let t = (t_over_q / one_over_q * 16.0 * self.texture_dimensions().1 as f64).round();
        (s.clamp(i32::MIN as f64, i32::MAX as f64) as i32, t.clamp(i32::MIN as f64, i32::MAX as f64) as i32)
    }

    fn sprite_texture_coordinates(&self, a: Vertex, b: Vertex, px: i32, py: i32) -> (i32, i32) {
        if self.primitive & (1 << 8) != 0 {
            return (
                Self::interpolate_axis(a.u, b.u, px, a.x, b.x),
                Self::interpolate_axis(a.v, b.v, py, a.y, b.y),
            );
        }
        let s = Self::interpolate_axis((a.s * 65536.0) as i32, (b.s * 65536.0) as i32, px, a.x, b.x) as f64 / 65536.0;
        let t = Self::interpolate_axis((a.t * 65536.0) as i32, (b.t * 65536.0) as i32, py, a.y, b.y) as f64 / 65536.0;
        let q = Self::interpolate_axis((a.q * 65536.0) as i32, (b.q * 65536.0) as i32, px, a.x, b.x) as f64 / 65536.0;
        if !q.is_finite() || q.abs() < 1.0e-12 { return (0, 0); }
        let (width, height) = self.texture_dimensions();
        let u = (s / q * width as f64 * 16.0).round();
        let v = (t / q * height as f64 * 16.0).round();
        (u.clamp(i32::MIN as f64, i32::MAX as f64) as i32, v.clamp(i32::MIN as f64, i32::MAX as f64) as i32)
    }

    fn texture_dimensions(&self) -> (usize, usize) {
        (1usize << ((self.tex0 >> 26) & 0xf), 1usize << ((self.tex0 >> 30) & 0xf))
    }

    fn texture_color(&self, u_fixed: i32, v_fixed: i32, vertex_color: u32) -> u32 {
        let psm = ((self.tex0 >> 20) & 0x3f) as u8;
        if !self.texturing_enabled() || !matches!(psm, 0 | 1 | 2 | 10 | 19 | 20) { return vertex_color; }
        let min_filter = ((self.tex1 >> 6) & 7) as u8;
        let linear = self.tex1 & (1 << 5) != 0 || matches!(min_filter, 1 | 4 | 5);
        let texel = if linear {
            let x0 = u_fixed.div_euclid(16);
            let y0 = v_fixed.div_euclid(16);
            let fx = u_fixed.rem_euclid(16) as i64;
            let fy = v_fixed.rem_euclid(16) as i64;
            let Some(c00) = self.texture_texel(psm, x0, y0) else { return vertex_color; };
            let Some(c10) = self.texture_texel(psm, x0 + 1, y0) else { return vertex_color; };
            let Some(c01) = self.texture_texel(psm, x0, y0 + 1) else { return vertex_color; };
            let Some(c11) = self.texture_texel(psm, x0 + 1, y0 + 1) else { return vertex_color; };
            Self::bilinear_color([c00, c10, c01, c11], fx, fy)
        } else {
            let Some(texel) = self.texture_texel(psm, u_fixed.div_euclid(16), v_fixed.div_euclid(16)) else { return vertex_color; };
            texel
        };
        match (self.tex0 >> 35) & 3 {
            0 => Self::modulate(texel, vertex_color),
            1 if self.tex0 & (1 << 34) != 0 => texel,
            1 => (texel & 0x00ff_ffff) | (vertex_color & 0xff00_0000),
            _ => vertex_color,
        }
    }

    fn texture_texel(&self, psm: u8, u_coord: i32, v_coord: i32) -> Option<u32> {
        let (width, height) = self.texture_dimensions();
        let buffer_width = (((self.tex0 >> 14) & 0x3f) as usize).max(1) * 64;
        let u = Self::texture_axis(
            u_coord, width, (self.clamp & 3) as u8,
            ((self.clamp >> 4) & 0x3ff) as i32,
            ((self.clamp >> 14) & 0x3ff) as i32,
        );
        let v = Self::texture_axis(
            v_coord, height, ((self.clamp >> 2) & 3) as u8,
            ((self.clamp >> 24) & 0x3ff) as i32,
            ((self.clamp >> 34) & 0x3ff) as i32,
        );
        let base_pointer = (self.tex0 & 0x3fff) as u32;
        Some(if psm == 0 {
            let address = Self::psmct32_address(base_pointer, buffer_width as u32 / 64, u, v);
            u32::from_le_bytes(self.vram[address..address + 4].try_into().expect("wrapped GS texture address"))
        } else if psm == 19 {
            let address = Self::psmt8_address(base_pointer, ((self.tex0 >> 14) & 0x3f) as u32, u, v);
            let index = self.vram[address] as usize;
            let cbp = ((self.tex0 >> 37) & 0x3fff) as u32;
            let csm2 = self.tex0 & (1 << 55) != 0;
            let palette_index = if csm2 { index } else { (index & !0x18) | ((index & 0x08) << 1) | ((index & 0x10) >> 1) };
            let cpsm = ((self.tex0 >> 51) & 0xf) as u8;
            let palette = match cpsm {
                0 => {
                    let address = if csm2 {
                        (cbp as usize * 256 + palette_index * 4) & (GS_VRAM_SIZE - 1)
                    } else {
                        Self::psmct32_address(cbp, 1, palette_index & 63, palette_index >> 6)
                    };
                    u32::from_le_bytes(self.vram[address..address + 4].try_into().expect("wrapped GS CLUT address"))
                }
                2 => {
                    let address = if csm2 {
                        (cbp as usize * 256 + palette_index * 2) & (GS_VRAM_SIZE - 1)
                    } else {
                        Self::psmct16_address(cbp, 1, palette_index & 63, palette_index >> 6, false)
                    };
                    Self::expand_rgba5551(u16::from_le_bytes(self.vram[address..address + 2].try_into().expect("wrapped GS CLUT address")), self.texa)
                }
                _ => return None,
            };
            palette
        } else if psm == 20 {
            let (address, high_nibble) = Self::psmt4_address(
                base_pointer,
                ((self.tex0 >> 14) & 0x3f) as u32,
                u,
                v,
            );
            let packed = self.vram[address];
            let index = usize::from(if high_nibble { packed >> 4 } else { packed & 0x0f });
            let csa = ((self.tex0 >> 56) & 0x1f) as usize;
            let csm2 = self.tex0 & (1 << 55) != 0;
            let palette_index = csa * 16 + index;
            let palette_index = if csm2 { palette_index } else {
                (palette_index & !0x18) | ((palette_index & 0x08) << 1) | ((palette_index & 0x10) >> 1)
            };
            let cbp = ((self.tex0 >> 37) & 0x3fff) as u32;
            match ((self.tex0 >> 51) & 0xf) as u8 {
                0 => {
                    let address = if csm2 {
                        (cbp as usize * 256 + palette_index * 4) & (GS_VRAM_SIZE - 1)
                    } else {
                        Self::psmct32_address(cbp, 1, palette_index & 63, palette_index >> 6)
                    };
                    u32::from_le_bytes(self.vram[address..address + 4].try_into().expect("wrapped GS CLUT address"))
                }
                2 => {
                    let address = if csm2 {
                        (cbp as usize * 256 + palette_index * 2) & (GS_VRAM_SIZE - 1)
                    } else {
                        Self::psmct16_address(cbp, 1, palette_index & 63, palette_index >> 6, false)
                    };
                    Self::expand_rgba5551(u16::from_le_bytes(self.vram[address..address + 2].try_into().expect("wrapped GS CLUT address")), self.texa)
                }
                _ => return None,
            }
        } else if psm == 1 {
            let address = Self::psmct32_address(base_pointer, buffer_width as u32 / 64, u, v);
            let raw = u32::from_le_bytes(self.vram[address..address + 4].try_into().expect("wrapped GS texture address"));
            Self::expand_rgb24(raw & 0x00ff_ffff, self.texa)
        } else {
            let address = Self::psmct16_address(base_pointer, buffer_width as u32 / 64, u, v, psm == 10);
            Self::expand_rgba5551(u16::from_le_bytes(self.vram[address..address + 2].try_into().expect("wrapped GS texture address")), self.texa)
        })
    }

    fn bilinear_color(colors: [u32; 4], fx: i64, fy: i64) -> u32 {
        let mut result = 0u32;
        for shift in [0, 8, 16, 24] {
            let c00 = i64::from((colors[0] >> shift) & 0xff);
            let c10 = i64::from((colors[1] >> shift) & 0xff);
            let c01 = i64::from((colors[2] >> shift) & 0xff);
            let c11 = i64::from((colors[3] >> shift) & 0xff);
            let top = c00 * (16 - fx) + c10 * fx;
            let bottom = c01 * (16 - fx) + c11 * fx;
            let channel = (top * (16 - fy) + bottom * fy + 128) / 256;
            result |= (channel.clamp(0, 255) as u32) << shift;
        }
        result
    }

    fn expand_rgba5551(pixel: u16, texa: u64) -> u32 {
        let expand = |channel: u32| (channel << 3) | (channel >> 2);
        let r = expand(u32::from(pixel & 0x1f));
        let g = expand(u32::from((pixel >> 5) & 0x1f));
        let b = expand(u32::from((pixel >> 10) & 0x1f));
        let rgb_zero = pixel & 0x7fff == 0;
        let a = if texa & (1 << 15) != 0 && rgb_zero {
            0
        } else if pixel & 0x8000 != 0 {
            ((texa >> 32) & 0xff) as u32
        } else {
            (texa & 0xff) as u32
        };
        r | (g << 8) | (b << 16) | (a << 24)
    }

    fn expand_rgb24(pixel: u32, texa: u64) -> u32 {
        let alpha = if texa & (1 << 15) != 0 && pixel & 0x00ff_ffff == 0 {
            0
        } else {
            (texa & 0xff) as u32
        };
        (pixel & 0x00ff_ffff) | (alpha << 24)
    }

    fn pack_rgba5551(color: u32) -> u16 {
        let r = (color & 0xff) >> 3;
        let g = ((color >> 8) & 0xff) >> 3;
        let b = ((color >> 16) & 0xff) >> 3;
        let a = u16::from(color & 0xff00_0000 != 0);
        (r as u16) | ((g as u16) << 5) | ((b as u16) << 10) | (a << 15)
    }

    fn texture_axis(coord: i32, size: usize, mode: u8, min: i32, max: i32) -> usize {
        let size_i32 = size.min(i32::MAX as usize) as i32;
        let coord = match mode {
            0 => coord.rem_euclid(size_i32),
            1 => coord.clamp(0, size_i32 - 1),
            2 => coord.clamp(min.min(max), max.max(min)),
            3 => (coord & min) | max,
            _ => unreachable!(),
        };
        coord.rem_euclid(size_i32) as usize
    }

    fn modulate(texel: u32, color: u32) -> u32 {
        let mut out = 0u32;
        for shift in [0, 8, 16, 24] {
            let t = (texel >> shift) & 0xff;
            let c = (color >> shift) & 0xff;
            out |= ((t * c / 128).min(255)) << shift;
        }
        out
    }

    fn put_pixel(&mut self, x: usize, y: usize, color: u32, z: u32) {
        if let Some((x0, y0, x1, y1)) = self.clip_bounds() {
            if x >= x0 && x <= x1 && y >= y0 && y <= y1 { self.put_pixel_unchecked(x, y, color, z); }
        }
    }

    fn put_pixel_unchecked(&mut self, x: usize, y: usize, mut color: u32, z: u32) {
        if !matches!(self.frame_psm, 0 | 1 | 2 | 10) { return; }
        let index = y * self.width + x;
        let old = self.pixels[index];
        let mut write_frame = true;
        let mut write_depth = true;
        let mut rgb_only = false;
        if self.test & 1 != 0 && !Self::alpha_test_pass((color >> 24) as u8, self.test) {
            match (self.test >> 12) & 3 {
                0 => return,
                1 => write_depth = false,
                2 => write_frame = false,
                _ => { write_depth = false; rgb_only = true; }
            }
        }
        // PSMCT24 has no alpha bit, so its destination-alpha test passes.
        if self.frame_psm != 1 && self.test & (1 << 14) != 0 {
            let destination_alpha = old & 0x8000_0000 != 0;
            let required_alpha = self.test & (1 << 15) != 0;
            if destination_alpha != required_alpha { return; }
        }
        let z_test_enabled = self.test & (1 << 16) != 0;
        let old_depth = self.depth_value(index, x, y);
        if z_test_enabled {
            let test_z = if matches!(self.depth_psm, 50 | 58) { z >> 8 } else { z };
            let passes = match (self.test >> 17) & 3 {
                0 => false,
                1 => true,
                2 => test_z >= old_depth,
                _ => test_z > old_depth,
            };
            if !passes { return; }
        }
        if write_frame {
            if self.primitive & (1 << 6) != 0 { color = Self::alpha_blend(color, old, self.alpha, self.colclamp); }
            if rgb_only { color = (color & 0x00ff_ffff) | (old & 0xff00_0000); }
        }
        let color = if matches!(self.frame_psm, 2 | 10) {
            Self::expand_rgba5551(Self::pack_rgba5551(color), 0x0000_00ff_0000_0000)
        } else { color };
        if write_frame {
            let merged = (color & !self.frame_mask) | (old & self.frame_mask);
            self.pixels[index] = merged;
            let base_pointer = self.frame_base * 32; // FRAME FBP pages -> 256-byte blocks.
            if matches!(self.frame_psm, 2 | 10) {
                let address = Self::psmct16_address(base_pointer, self.frame_buffer_width, x, y, self.frame_psm == 10);
                let packed = Self::pack_rgba5551(merged);
                self.vram[address..address + 2].copy_from_slice(&packed.to_le_bytes());
            } else {
                let address = Self::psmct32_address(base_pointer, self.frame_buffer_width, x, y);
                if self.frame_psm == 1 {
                    let old = u32::from_le_bytes(self.vram[address..address + 4].try_into().expect("wrapped GS framebuffer address"));
                    let stored = (old & 0xff00_0000) | (merged & 0x00ff_ffff);
                    self.vram[address..address + 4].copy_from_slice(&stored.to_le_bytes());
                } else {
                    self.vram[address..address + 4].copy_from_slice(&merged.to_le_bytes());
                }
            }
        }
        if write_depth && !self.z_write_mask {
            self.depth[index] = z & 0x00ff_ffff;
            self.write_depth_value(x, y, z);
        }
    }

    fn depth_value(&self, index: usize, x: usize, y: usize) -> u32 {
        match self.depth_psm {
            48 | 49 => {
                let base_pointer = self.depth_base * 32;
                let address = Self::psmz32_address(base_pointer, self.frame_buffer_width, x, y);
                let word = u32::from_le_bytes(self.vram[address..address + 4].try_into().expect("wrapped GS depth address"));
                if self.depth_psm == 49 { word & 0x00ff_ffff } else { word }
            }
            50 | 58 => {
                let address = Self::psmz16_address(
                    self.depth_base * 32,
                    self.frame_buffer_width,
                    x,
                    y,
                    self.depth_psm == 58,
                );
                u32::from(u16::from_le_bytes(self.vram[address..address + 2].try_into().expect("wrapped GS depth address")))
            }
            _ => self.depth[index],
        }
    }

    fn write_depth_value(&mut self, x: usize, y: usize, z: u32) {
        match self.depth_psm {
            48 => {
                let address = Self::psmz32_address(self.depth_base * 32, self.frame_buffer_width, x, y);
                self.vram[address..address + 4].copy_from_slice(&z.to_le_bytes());
            }
            49 => {
                let address = Self::psmz32_address(self.depth_base * 32, self.frame_buffer_width, x, y);
                let old = u32::from_le_bytes(self.vram[address..address + 4].try_into().expect("wrapped GS depth address"));
                let value = (old & 0xff00_0000) | (z & 0x00ff_ffff);
                self.vram[address..address + 4].copy_from_slice(&value.to_le_bytes());
            }
            50 | 58 => {
                let address = Self::psmz16_address(
                    self.depth_base * 32,
                    self.frame_buffer_width,
                    x,
                    y,
                    self.depth_psm == 58,
                );
                self.vram[address..address + 2].copy_from_slice(&((z >> 8) as u16).to_le_bytes());
            }
            _ => {}
        }
    }

    fn alpha_test_pass(alpha: u8, test: u64) -> bool {
        let reference = ((test >> 4) & 0xff) as u8;
        match (test >> 1) & 7 {
            0 => false,
            1 => true,
            2 => alpha < reference,
            3 => alpha <= reference,
            4 => alpha == reference,
            5 => alpha >= reference,
            6 => alpha > reference,
            _ => alpha != reference,
        }
    }

    fn alpha_blend(source: u32, destination: u32, alpha: u64, colclamp: bool) -> u32 {
        let a = (alpha & 3) as u8;
        let b = ((alpha >> 2) & 3) as u8;
        let c = ((alpha >> 4) & 3) as u8;
        let d = ((alpha >> 6) & 3) as u8;
        let fixed = ((alpha >> 32) & 0xff) as i32;
        let lhs = match a { 0 => source, 1 => destination, _ => 0 };
        let rhs = match b { 0 => source, 1 => destination, _ => 0 };
        let addend = match d { 0 => source, 1 => destination, _ => 0 };
        let blend_factor = match c {
            0 => ((source >> 24) & 0xff) as i32,
            1 => ((destination >> 24) & 0xff) as i32,
            2 => fixed,
            _ => 0,
        };
        let mut result = source & 0xff00_0000;
        for shift in [0, 8, 16] {
            let left = ((lhs >> shift) & 0xff) as i32;
            let right = ((rhs >> shift) & 0xff) as i32;
            let base = ((addend >> shift) & 0xff) as i32;
            let value = (((left - right) * blend_factor) >> 7) + base;
            let channel = if colclamp { value.clamp(0, 255) as u32 } else { value as u32 & 0xff };
            result |= channel << shift;
        }
        result
    }
}
