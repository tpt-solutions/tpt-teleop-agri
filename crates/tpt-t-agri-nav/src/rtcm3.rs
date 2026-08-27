//! Zero-copy RTCM3 frame detection, CRC24Q validation, and observation decode.
//!
//! RTCM3 messages are variable-length: a `0xD3` preamble, a 12-bit length, the
//! payload, then a 24-bit CRC. The CRC (CRC24Q, polynomial `0x1864CFB`) is
//! computed over the length field and payload. Two decoders are provided:
//! [`decode_1005`] (reference-station ECEF ARP) and [`decode_msm7_header`]
//! (GPS MSM7 satellite/signal masks) — sufficient to drive RTK base
//! positioning and to enumerate tracked satellites for 100 Hz correction
//! ingestion. Full per-signal carrier-phase block decoding is a documented
//! follow-up (see `Phase 3` in `todo.md`).

/// RTCM3 sync preamble.
pub const RTCM3_PREAMBLE: u8 = 0xD3;

/// A CRC24Q-validated RTCM3 message frame.
#[derive(Debug, Clone, Copy)]
pub struct RtcmFrame<'a> {
    /// RTCM message number (e.g. 1005, 1077).
    pub msg_type: u16,
    /// Message payload (length bytes), excluding preamble/length/CRC.
    pub data: &'a [u8],
}

/// Compute the RTCM3 CRC24Q over `data`.
///
/// MSB-first, polynomial `0x1864CFB`, no reflection, init `0`, no final XOR.
pub fn crc24q(data: &[u8]) -> u32 {
    let mut crc: u32 = 0;
    for &b in data {
        crc ^= (b as u32) << 16;
        for _ in 0..8 {
            crc <<= 1;
            if crc & 0x800000 != 0 {
                crc ^= 0x1864CFB;
            }
        }
        crc &= 0xFFFFFF;
    }
    crc & 0xFFFFFF
}

/// Locate the next valid RTCM3 frame in `buf`.
///
/// Returns the frame (borrowing `buf`) and the number of bytes it consumed, or
/// `None` if no complete, CRC-valid frame is present. The search skips past an
/// invalid preamble and resumes at the next `0xD3`.
pub fn next_frame(buf: &[u8]) -> Option<(RtcmFrame<'_>, usize)> {
    let start = buf.iter().position(|&b| b == RTCM3_PREAMBLE)?;
    if start + 3 > buf.len() {
        return None;
    }
    let len = (((buf[start + 1] & 0x03) as usize) << 8) | (buf[start + 2] as usize);
    let total = 3 + len + 3;
    if start + total > buf.len() {
        return None;
    }
    let body = &buf[start + 1..start + 3 + len];
    let crc_calc = crc24q(body);
    let off = start + 3 + len;
    let crc_recv = ((buf[off] as u32) << 16) | ((buf[off + 1] as u32) << 8) | (buf[off + 2] as u32);
    if crc_calc != crc_recv {
        return None;
    }
    let data = &buf[start + 3..start + 3 + len];
    let msg_type = (((data[0] as u16) << 4) | ((data[1] >> 4) as u16)) & 0x0FFF;
    Some((RtcmFrame { msg_type, data }, total))
}

/// A bit-oriented reader over a byte slice (MSB-first).
pub struct BitReader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> BitReader<'a> {
    /// Create a reader at bit position 0.
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Read `n` bits MSB-first into a `u64`.
    pub fn read(&mut self, n: usize) -> u64 {
        let mut v = 0u64;
        for _ in 0..n {
            let byte = self.buf[self.pos / 8];
            let bit = (byte >> (7 - (self.pos % 8))) & 1;
            v = (v << 1) | bit as u64;
            self.pos += 1;
        }
        v
    }

    /// Remaining bits available.
    pub fn remaining(&self) -> usize {
        self.buf.len() * 8 - self.pos
    }
}

/// A bit-oriented writer (MSB-first) used to build RTCM3 messages for tests and
/// for generating correction streams.
pub struct BitWriter {
    buf: Vec<u8>,
    pos: usize,
}

impl Default for BitWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl BitWriter {
    /// Create an empty writer.
    pub fn new() -> Self {
        Self {
            buf: Vec::new(),
            pos: 0,
        }
    }

    /// Append `n` low bits of `value` (MSB-first).
    pub fn write(&mut self, value: u64, n: usize) {
        for i in (0..n).rev() {
            let bit = ((value >> i) & 1) as u8;
            let byte_idx = self.pos / 8;
            if byte_idx >= self.buf.len() {
                self.buf.push(0);
            }
            self.buf[byte_idx] |= bit << (7 - (self.pos % 8));
            self.pos += 1;
        }
    }

    /// Bytes written so far.
    pub fn bytes(&self) -> &[u8] {
        &self.buf
    }

    /// Total bits written.
    pub fn len_bits(&self) -> usize {
        self.pos
    }
}

fn s38(v: u64) -> f64 {
    if v & (1u64 << 37) != 0 {
        (v as i64 - (1i64 << 38)) as f64
    } else {
        v as f64
    }
}

#[cfg(test)]
fn to_u38(v: f64) -> u64 {
    (v as i64 as u64) & ((1u64 << 38) - 1)
}

/// Reference-station ECEF antenna reference point (messages 1005 / 1006).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StationEcef {
    /// Reference station id (DF003).
    pub station_id: u16,
    /// ECEF X (metres).
    pub x: f64,
    /// ECEF Y (metres).
    pub y: f64,
    /// ECEF Z (metres).
    pub z: f64,
}

/// Decode a 1005/1006 reference-station ARP message.
///
/// Layout (RTCM 3.3 §3.5.13): message number (12), station id (12), 4 indicator
/// bits, then three 38-bit signed ECEF coordinates in 0.1 mm units.
pub fn decode_1005(data: &[u8]) -> Option<StationEcef> {
    if data.len() < 18 {
        return None;
    }
    let mut br = BitReader::new(data);
    let msg = br.read(12);
    if msg != 1005 && msg != 1006 {
        return None;
    }
    let station_id = br.read(12) as u16;
    let _ind = br.read(4);
    let x = s38(br.read(38)) * 1e-4;
    let y = s38(br.read(38)) * 1e-4;
    let z = s38(br.read(38)) * 1e-4;
    Some(StationEcef {
        station_id,
        x,
        y,
        z,
    })
}

/// GPS MSM7 header (message 1077): epoch, masks enumerating tracked
/// satellites and signals for carrier-phase extraction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MsmHeader {
    /// Reference station id.
    pub station_id: u16,
    /// GPS epoch time (DF001, 30 bits).
    pub epoch: u32,
    /// Multiple-message flag (DF393).
    pub multiple_message: bool,
    /// Satellite mask (DF394, 64 bits).
    pub satellite_mask: u64,
    /// Signal mask (DF395, 32 bits).
    pub signal_mask: u32,
}

/// Decode the header of a GPS MSM7 (1071–1077) message.
///
/// Layout (RTCM 3.3 MSM): message number (12), station id (12), GPS epoch (30),
/// multiple-message flag (1), 7 indicator bits (IODS 3 + clock steering 2 +
/// external clock 2), satellite mask (64), signal mask (32).
pub fn decode_msm7_header(data: &[u8]) -> Option<MsmHeader> {
    if data.len() < 18 {
        return None;
    }
    let mut br = BitReader::new(data);
    let msg = br.read(12);
    if !(1071..=1077).contains(&msg) {
        return None;
    }
    let station_id = br.read(12) as u16;
    let epoch = br.read(30) as u32;
    let multiple_message = br.read(1) == 1;
    let _ind = br.read(7);
    let satellite_mask = br.read(64);
    let signal_mask = br.read(32) as u32;
    Some(MsmHeader {
        station_id,
        epoch,
        multiple_message,
        satellite_mask,
        signal_mask,
    })
}

/// Encode a minimal valid RTCM3 message (preamble + length + payload + CRC).
///
/// The CRC24Q is computed over the length field and payload, matching the
/// validation performed by [`next_frame`].
pub fn encode_frame(payload: &[u8]) -> Vec<u8> {
    let len_hi = ((payload.len() >> 8) & 0x03) as u8;
    let len_lo = (payload.len() & 0xFF) as u8;
    let mut body = Vec::with_capacity(payload.len() + 2);
    body.push(len_hi);
    body.push(len_lo);
    body.extend_from_slice(payload);
    let crc = crc24q(&body);
    let mut out = Vec::with_capacity(payload.len() + 6);
    out.push(RTCM3_PREAMBLE);
    out.extend_from_slice(&body);
    out.push((crc >> 16) as u8);
    out.push((crc >> 8) as u8);
    out.push(crc as u8);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_is_deterministic() {
        let a = crc24q(&[0xD3, 0x00, 0x13]);
        let b = crc24q(&[0xD3, 0x00, 0x13]);
        assert_eq!(a, b);
        assert_eq!(a & !0xFFFFFF, 0);
    }

    #[test]
    fn frame_round_trips() {
        let payload = [0x10, 0x50, 0x00, 0xFF, 0xAB];
        let frame = encode_frame(&payload);
        let (f, consumed) = next_frame(&frame).unwrap();
        assert_eq!(f.data, &payload[..]);
        assert_eq!(consumed, frame.len());
        assert_eq!(f.msg_type, 0x105);
    }

    #[test]
    fn corrupt_frame_rejected() {
        let mut frame = encode_frame(&[0x10, 0x05, 0x00]);
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;
        assert!(next_frame(&frame).is_none());
    }

    #[test]
    fn station_ecef_round_trips() {
        let mut w = BitWriter::new();
        w.write(1005, 12);
        w.write(1234, 12);
        w.write(0, 4);
        w.write(to_u38(1_234_567.0 * 1e4), 38);
        w.write(to_u38(-987_654.0 * 1e4), 38);
        w.write(to_u38(2_345_678.0 * 1e4), 38);
        let payload = w.bytes().to_vec();
        let frame = encode_frame(&payload);
        let (f, _) = next_frame(&frame).unwrap();
        let s = decode_1005(f.data).unwrap();
        assert_eq!(s.station_id, 1234);
        assert!((s.x - 1_234_567.0).abs() < 1e-6);
        assert!((s.y + 987_654.0).abs() < 1e-6);
        assert!((s.z - 2_345_678.0).abs() < 1e-6);
    }

    #[test]
    fn msm7_header_round_trips() {
        let mut w = BitWriter::new();
        w.write(1077, 12);
        w.write(99, 12);
        w.write(1_234_567, 30);
        w.write(1, 1);
        w.write(0, 7);
        w.write(0xAAAA_AAAA_AAAA_AAAA, 64);
        w.write(0x5555_5555, 32);
        let frame = encode_frame(w.bytes());
        let (f, _) = next_frame(&frame).unwrap();
        let h = decode_msm7_header(f.data).unwrap();
        assert_eq!(h.station_id, 99);
        assert_eq!(h.epoch, 1_234_567);
        assert!(h.multiple_message);
        assert_eq!(h.satellite_mask, 0xAAAA_AAAA_AAAA_AAAA);
        assert_eq!(h.signal_mask, 0x5555_5555);
    }
}
