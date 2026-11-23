//! Explore Air 2 wire protocol: framing, checksums, AES, key derivation and reply parsing.
//!
//! Everything here is pure and deterministic, so it can be tested without hardware.
//!
//! ```text
//! Plaintext frame:  [len][op][payload]                 len = op + payload bytes, no checksum
//! Encrypted frame:  [0x80 | encLen][AES-256-ECB(inner)]
//!                   inner = [len][op][payload] zero-padded to the next 16-byte multiple that
//!                   leaves room for a trailing byte; the 8-bit checksum of the core sits in the
//!                   LAST byte. encLen = 128 wraps to 0x00, so the header 0x80 means 128 bytes.
//! Replies:          [hdr][body]; hdr & 0x80 = encrypted body. 0x5E 0xA2 un-stuffs to 0x24.
//! ```

use aes::Aes256;
use aes::cipher::{BlockDecrypt, BlockEncrypt, KeyInit, generic_array::GenericArray};

use crate::error::{Error, Result};

/// Opcodes used by this crate.
pub mod op {
    /// Encrypted: end a draw section. Payload `00 00 00`.
    pub const END_DRAW: u8 = 0x22;
    /// Encrypted: begin a draw section. Payload `00 00 00`.
    pub const BEGIN_DRAW: u8 = 0x21;
    /// Encrypted: tool state; pressure in the last two bytes (big-endian).
    pub const SET_MACHINE_STATE: u8 = 0x30;
    /// Encrypted: per-job geometry block.
    pub const GEOMETRY: u8 = 0x31;
    /// Encrypted: speed/acceleration registers (RAM only).
    pub const SPEED_REGISTER: u8 = 0xA1;
    /// Encrypted: orient the blade (takes ~1.2 s).
    pub const ORIENT_BLADE: u8 = 0x76;
    /// Encrypted: up to 13 move/line vectors.
    pub const MULTI_DRAW_VECTOR: u8 = 0x43;

    /// Plaintext: machine status / identity.
    pub const MACHINE_STATUS: u8 = 0x60;
    /// Plaintext: lights, buttons, progress and dial.
    pub const INTERFACE_STATE: u8 = 0x73;
    /// Plaintext: material dial position.
    pub const MATERIAL_DIAL: u8 = 0x38;
    /// Plaintext: set the button lights.
    pub const SET_LIGHTS: u8 = 0x72;
    /// Plaintext: clear latched button presses.
    pub const CLEAR_BUTTONS: u8 = 0x71;
    /// Plaintext: eject the mat.
    pub const UNLOAD_MAT: u8 = 0x84;
    /// Plaintext: abort / reset the cut state.
    pub const ABORT: u8 = 0x68;
    /// Plaintext: read the carriage position (two big-endian f32, inches).
    pub const READ_POSITION: u8 = 0xB9;
    /// Plaintext: reset the AES state before `NewAES`.
    pub const RESET_AES: u8 = 0xCF;
    /// Plaintext: `NewAES` session-key request.
    pub const NEW_AES: u8 = 0xC7;
}

/// Opcodes that must never be sent: firmware/bootloader and persistent-memory writes.
pub const FORBIDDEN_OPCODES: [u8; 6] = [0x04, 0x05, 0x08, 0x12, 0xA3, 0xFF];

/// Returns an error if `opcode` is on the do-not-send list.
pub fn check_opcode(opcode: u8) -> Result<()> {
    if FORBIDDEN_OPCODES.contains(&opcode) {
        Err(Error::ForbiddenOpcode(opcode))
    } else {
        Ok(())
    }
}

/// Supported machine types.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum MachineType {
    ExploreAir2,
}

impl MachineType {
    /// The machine's internal type id.
    pub fn id(self) -> u8 {
        match self {
            MachineType::ExploreAir2 => 10,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            MachineType::ExploreAir2 => "Explore Air 2",
        }
    }

    /// The 64-byte base buffer the key-encryption key is derived from. It originates in
    /// Cricut's own software; see the opencutter NOTICE for the distribution implications.
    pub fn base_buffer(self) -> [u8; 64] {
        match self {
            MachineType::ExploreAir2 => hex_array(concat!(
                "204c63cd413760ad33e2142121165655728372dd44c351db15947136336078aa",
                "21d913388505788b568a3787112c2500011e64e570b231cb56c9868605401618"
            )),
        }
    }

    /// The key-encryption key for this machine type.
    pub fn kek(self) -> [u8; 32] {
        kek_from_base(&self.base_buffer())
    }
}

/// `KEK[i] = base[63 - 2i]`: the odd-indexed bytes of the base buffer, reversed.
pub fn kek_from_base(base: &[u8; 64]) -> [u8; 32] {
    std::array::from_fn(|i| base[63 - 2 * i])
}

/// 8-bit additive checksum.
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |acc, b| acc.wrapping_add(*b))
}

/// `[len][op][payload]`.
pub fn plain_frame(opcode: u8, payload: &[u8]) -> Result<Vec<u8>> {
    check_opcode(opcode)?;
    let len = 1 + payload.len();
    let len_byte = u8::try_from(len)
        .ok()
        .filter(|l| *l <= 0x7F)
        .ok_or(Error::FrameTooLarge(len))?;
    let mut f = Vec::with_capacity(1 + len);
    f.push(len_byte);
    f.push(opcode);
    f.extend_from_slice(payload);
    Ok(f)
}

/// `[0x80 | encLen][AES-256-ECB(inner)]`.
pub fn enc_frame(opcode: u8, payload: &[u8], key: &[u8; 32]) -> Result<Vec<u8>> {
    let core = plain_frame(opcode, payload)?;
    let padded = (core.len() / 16 + 1) * 16;
    // Header byte: 0x80 | (encrypted length & 0x7F); a 128-byte body wraps to 0x80.
    let header = match u8::try_from(padded) {
        Ok(n) if n <= 128 => 0x80 | (n & 0x7F),
        _ => return Err(Error::FrameTooLarge(padded)),
    };
    let mut inner = vec![0u8; padded];
    inner[..core.len()].copy_from_slice(&core);
    inner[padded - 1] = checksum(&core);
    aes_encrypt(&mut inner, key);
    let mut out = Vec::with_capacity(1 + padded);
    out.push(header);
    out.extend_from_slice(&inner);
    Ok(out)
}

/// Body length encoded by a reply header byte (an encrypted 128-byte body wraps to `0x80`).
pub fn body_len(hdr: u8) -> usize {
    if hdr == 0x80 {
        128
    } else {
        (hdr & 0x7F) as usize
    }
}

/// Undo the machine's byte stuffing: `0x5E 0xA2` → `0x24`.
pub fn unescape(input: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(input.len());
    let mut i = 0;
    while i < input.len() {
        if input[i] == 0x5E && input.get(i + 1) == Some(&0xA2) {
            out.push(0x24);
            i += 2;
        } else {
            out.push(input[i]);
            i += 1;
        }
    }
    out
}

/// In-place AES-256-ECB encryption; `data.len()` must be a multiple of 16.
pub fn aes_encrypt(data: &mut [u8], key: &[u8; 32]) {
    let cipher = Aes256::new(GenericArray::from_slice(key));
    for block in data.as_chunks_mut::<16>().0 {
        cipher.encrypt_block(GenericArray::from_mut_slice(block));
    }
}

/// In-place AES-256-ECB decryption; `data.len()` must be a multiple of 16.
pub fn aes_decrypt(data: &mut [u8], key: &[u8; 32]) {
    let cipher = Aes256::new(GenericArray::from_slice(key));
    for block in data.as_chunks_mut::<16>().0 {
        cipher.decrypt_block(GenericArray::from_mut_slice(block));
    }
}

/// The `NewAES` request: `31 C7 <slot> <47 random bytes>`. (`0x31` = length of op + 48 bytes.)
pub fn new_aes_request(slot: u8, random: &[u8; 47]) -> Vec<u8> {
    let mut req = Vec::with_capacity(50);
    req.extend_from_slice(&[0x31, op::NEW_AES, slot]);
    req.extend_from_slice(random);
    req
}

/// Find a `NewAES` reply (`0x30` + 48 bytes) in an accumulated, unescaped buffer and decrypt the
/// session key from it. Stray bytes before the reply are skipped.
pub fn find_session_key(unescaped: &[u8], kek: &[u8; 32]) -> Option<[u8; 32]> {
    for (i, _) in unescaped.iter().enumerate().filter(|(_, b)| **b == 0x30) {
        let Some(&encrypted) = unescaped[i + 1..].first_chunk::<48>() else {
            break; // not enough bytes after this (or any later) 0x30
        };
        let mut block = encrypted;
        aes_decrypt(&mut block, kek);
        if let [0x01, rest @ ..] = &block {
            return rest.first_chunk::<32>().copied();
        }
    }
    None
}

/// A parsed reply frame. `body` excludes the header and is decrypted if it was encrypted.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Reply {
    pub header: u8,
    pub body: Vec<u8>,
}

impl Reply {
    /// Parse a raw reply, un-stuffing and (if flagged and a key is given) decrypting it.
    pub fn parse(raw: &[u8], key: Option<&[u8; 32]>) -> Option<Reply> {
        let u = unescape(raw);
        let header = *u.first()?;
        let n = body_len(header);
        let mut body = u[1..u.len().min(1 + n)].to_vec();
        if header & 0x80 != 0
            && !body.is_empty()
            && body.len().is_multiple_of(16)
            && let Some(k) = key
        {
            aes_decrypt(&mut body, k);
        }
        Some(Reply { header, body })
    }

    /// For an encrypted command's reply: `<len> 01 <op> …` means the machine dropped it.
    pub fn is_drop_nak(&self) -> bool {
        self.body.get(1) == Some(&0x01)
    }

    /// For a `MultiDrawVector` ACK: byte 5 is the machine's free FIFO space.
    pub fn free_fifo(&self) -> Option<u8> {
        self.body.get(5).copied()
    }
}

/// Lights / buttons / dial snapshot from `GetInterfaceState` (0x73).
///
/// Body layout: `[0x73][lights][buttons][0x03][progress][dial]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InterfaceState {
    pub lights: u8,
    pub buttons: u8,
    pub progress: u8,
    pub dial: u8,
}

impl InterfaceState {
    pub const LIGHT_LOAD: u8 = 0x20;
    pub const LIGHT_MAT_LOADED: u8 = 0x10;
    pub const LIGHT_GO_BLINK: u8 = 0x08;
    pub const LIGHT_GO_SOLID: u8 = 0x04;
    pub const BUTTON_LOAD: u8 = 0x04;
    pub const BUTTON_GO: u8 = 0x02;

    pub fn parse(body: &[u8]) -> Option<Self> {
        if body.len() < 6 || body[0] != op::INTERFACE_STATE {
            return None;
        }
        Some(Self {
            lights: body[1],
            buttons: body[2],
            progress: body[4],
            dial: body[5],
        })
    }

    pub fn mat_loaded(&self) -> bool {
        self.lights & Self::LIGHT_MAT_LOADED != 0
    }

    pub fn go_pressed(&self) -> bool {
        self.buttons & Self::BUTTON_GO != 0
    }

    pub fn load_pressed(&self) -> bool {
        self.buttons & Self::BUTTON_LOAD != 0
    }
}

/// Parse a `ReadPositionFloat` (0xB9) reply body: `b9 <x f32 BE> <y f32 BE>`, inches.
pub fn parse_position(body: &[u8]) -> Option<(f32, f32)> {
    let [op::READ_POSITION, rest @ ..] = body else {
        return None;
    };
    let (x, rest) = rest.split_first_chunk::<4>()?;
    let y = rest.first_chunk::<4>()?;
    Some((f32::from_be_bytes(*x), f32::from_be_bytes(*y)))
}

pub(crate) fn hex_array<const N: usize>(s: &str) -> [u8; N] {
    let v = hex(s);
    v.try_into().expect("hex literal has the wrong length")
}

/// Decode a hex string (whitespace ignored); for compile-time-known constants and tests.
///
/// # Panics
///
/// Panics if `s` contains a non-hex digit or an odd number of digits: that is a bug in the
/// literal, not a runtime condition.
pub fn hex(s: &str) -> Vec<u8> {
    let s: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).expect("invalid hex"))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const KEY: [u8; 32] = [7u8; 32];

    #[test]
    fn checksum_wraps() {
        assert_eq!(checksum(&[]), 0x00);
        assert_eq!(checksum(&hex("ff")), 0xFF);
        assert_eq!(checksum(&hex("ff02")), 0x01);
    }

    #[test]
    fn plain_frames() {
        assert_eq!(
            plain_frame(0x60, &hex("000000")).unwrap(),
            hex("04 60 00 00 00")
        );
        assert_eq!(plain_frame(0x73, &[]).unwrap(), hex("01 73"));
    }

    #[test]
    fn enc_frame_small() {
        let f = enc_frame(0x22, &hex("000000"), &KEY).unwrap();
        assert_eq!(f.len(), 17);
        assert_eq!(f[0], 0x90);
        let mut inner = f[1..].to_vec();
        aes_decrypt(&mut inner, &KEY);
        assert_eq!(&inner[..5], &hex("04 22 00 00 00")[..]);
        assert!(inner[5..15].iter().all(|b| *b == 0));
        assert_eq!(inner[15], checksum(&hex("0422000000")));
    }

    #[test]
    fn enc_frame_exact_block_gets_extra_block() {
        // core of 16 bytes must pad to 32 so the checksum has its own trailing byte
        let f = enc_frame(0x31, &[0u8; 14], &KEY).unwrap();
        assert_eq!(f.len(), 33);
        assert_eq!(f[0], 0xA0);
    }

    #[test]
    fn enc_frame_128_wraps_header() {
        // 13 vectors = 117 bytes + len + op = 119 -> 128
        let f = enc_frame(0x43, &[0u8; 117], &KEY).unwrap();
        assert_eq!(f.len(), 129);
        assert_eq!(f[0], 0x80);
        assert_eq!(body_len(f[0]), 128);
        assert!(enc_frame(0x43, &[0u8; 126], &KEY).is_err());
    }

    #[test]
    fn body_lengths() {
        assert_eq!(body_len(0x09), 9);
        assert_eq!(body_len(0x90), 16);
        assert_eq!(body_len(0x80), 128);
    }

    #[test]
    fn unescaping() {
        assert_eq!(unescape(&hex("5ea2")), hex("24"));
        assert_eq!(unescape(&hex("015ea2ff5e")), hex("0124ff5e"));
        assert_eq!(unescape(&hex("5e01")), hex("5e01"));
    }

    #[test]
    fn kek_derivation() {
        let base: [u8; 64] = std::array::from_fn(|i| u8::try_from(i).unwrap());
        let kek = kek_from_base(&base);
        assert_eq!(kek[0], 63);
        assert_eq!(kek[1], 61);
        assert_eq!(kek[31], 1);
        assert_eq!(MachineType::ExploreAir2.kek()[0], 0x18);
        assert_eq!(MachineType::ExploreAir2.kek()[31], 0x4c);
    }

    #[test]
    fn forbidden_opcodes_rejected() {
        for o in FORBIDDEN_OPCODES {
            assert!(matches!(
                plain_frame(o, &[]),
                Err(Error::ForbiddenOpcode(_))
            ));
            assert!(enc_frame(o, &[], &KEY).is_err());
        }
    }

    #[test]
    fn session_key_roundtrip_with_stray_bytes() {
        let kek = MachineType::ExploreAir2.kek();
        let session: [u8; 32] = std::array::from_fn(|i| u8::try_from(i * 3).unwrap());
        let mut block = [0u8; 48];
        block[0] = 0x01;
        block[1..33].copy_from_slice(&session);
        aes_encrypt(&mut block, &kek);
        let mut stream = hex("09 60 6a 00 30 11");
        stream.push(0x30);
        stream.extend_from_slice(&block);
        assert_eq!(find_session_key(&stream, &kek), Some(session));
    }

    #[test]
    fn interface_and_position() {
        let st = InterfaceState::parse(&hex("73 10 02 03 00 03")).unwrap();
        assert!(st.mat_loaded() && st.go_pressed());
        assert_eq!(st.dial, 3);
        let mut b = vec![0xB9];
        b.extend_from_slice(&4.06f32.to_be_bytes());
        b.extend_from_slice(&0f32.to_be_bytes());
        assert_eq!(parse_position(&b), Some((4.06, 0.0)));
        assert_eq!(parse_position(&hex("b9 00")), None);
    }
}
