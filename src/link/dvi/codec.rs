//! OBDX Pro DVI codec (DV0 — pure, no I/O). Frames are `CMD LEN DATA… YY`
//! (LEN = data-byte count, excludes the checksum; `09`/`11` carry a 2-byte
//! LEN), responses are `CMD+0x10`, errors are `7F LEN CMD ERR YY`, and the
//! checksum is `0xFF - (sum of every prior byte & 0xFF)`. Every constant and
//! test vector below is from the OBDX Pro Developers Reference Manual v3.00
//! (§3.2 checksum, §3.3 frame format, §3.4/3.5 RX, §3.6/3.7 TX, §3.8 identity,
//! §3.9 settings, §3.11 protocol/comm, §3.14 CAN filters, §3.16.3 example).

/// Command bytes (request); the reply is `cmd + 0x10`.
pub mod cmd {
    pub const RX_NORMAL: u8 = 0x08;
    pub const RX_LARGE: u8 = 0x09;
    pub const TX_NORMAL: u8 = 0x10;
    pub const TX_LARGE: u8 = 0x11;
    pub const INFO: u8 = 0x22;
    pub const SETTINGS: u8 = 0x24;
    pub const RESET: u8 = 0x25;
    pub const PROTOCOL: u8 = 0x31;
    pub const CAN: u8 = 0x34;
    pub const ERROR: u8 = 0x7F;
}

/// `22` sub-commands.
pub mod info {
    pub const HARDWARE_VERSION: u8 = 0x00;
    pub const FIRMWARE_VERSION: u8 = 0x01;
    pub const MODEL: u8 = 0x02;
    pub const NAME: u8 = 0x03;
    pub const SERIAL: u8 = 0x04;
    pub const OBD_PROTOCOLS: u8 = 0x05;
    pub const PC_PROTOCOLS: u8 = 0x06;
}

/// `31 02 01 XX` protocols.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ObdProtocol {
    Aldl = 0x00,
    Vpw = 0x01,
    HsCan = 0x02,
    GmCan = 0x03,
    MsCan = 0x04,
}

/// `31 02 02 XX` network communication.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Comm {
    Off = 0x00,
    On = 0x01,
    ListenOnly = 0x02,
}

/// `34 11 00` filter type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FilterType {
    Pass = 0x00,
    Flow = 0x01,
    Block = 0x02,
}

/// Configuration error codes (`7F LEN CMD ERR`).
pub fn error_name(code: u8) -> &'static str {
    match code {
        1 => "InvalidCommand",
        2 => "RecvTooLong",
        3 => "ByteWaitTimeout",
        4 => "InvalidSerialChksum",
        5 => "SubCommandIncorrectSize",
        6 => "InvalidSubCommand",
        7 => "SubCommandInvalidData",
        _ => "Unknown",
    }
}

/// `0xFF - (sum & 0xFF)` over every byte before the checksum.
pub fn checksum(bytes: &[u8]) -> u8 {
    let sum: u32 = bytes.iter().map(|&b| b as u32).sum();
    0xFF - (sum & 0xFF) as u8
}

/// Build `CMD LEN DATA… YY` (1-byte LEN; the large forms `09`/`11` take 2).
pub fn frame(cmd: u8, data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() + 4);
    out.push(cmd);
    if cmd == cmd::RX_LARGE || cmd == cmd::TX_LARGE {
        out.push((data.len() >> 8) as u8);
        out.push((data.len() & 0xFF) as u8);
    } else {
        debug_assert!(data.len() <= 0xFF);
        out.push(data.len() as u8);
    }
    out.extend_from_slice(data);
    let yy = checksum(&out);
    out.push(yy);
    out
}

/// A decoded frame from the tool.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DviFrame {
    /// A network frame (`08`/`09`), CAN layout decoded.
    Rx(RxFrame),
    /// `7F LEN CMD ERR`: `cmd` = the failing command (`0xFF` = a bus read fault).
    Error { cmd: u8, code: u8 },
    /// Any other reply (`CMD+0x10`): raw data bytes.
    Reply { cmd: u8, data: Vec<u8> },
}

/// A received CAN frame: `[ts_us?] ID(4) data…`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RxFrame {
    pub id: u32,
    /// Microsecond timestamp when enabled (`24 02 03 01`).
    pub ts_us: Option<u32>,
    pub data: Vec<u8>,
}

/// Incremental frame splitter over a byte stream. Validates checksums; a
/// bad checksum resyncs by dropping one byte.
#[derive(Default)]
pub struct FrameParser {
    buf: Vec<u8>,
    /// RX frames carry a 4-byte µs timestamp (set once `24 02 03 01` took).
    pub timestamps: bool,
}

impl FrameParser {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn reset(&mut self) {
        self.buf.clear();
    }

    /// Command bytes that can start a frame — the tool's (replies `cmd +
    /// 0x10`, RX, error) and the PC's requests (so a fake device / loopback
    /// can parse with the same splitter). Anything else at the head of the
    /// buffer is garbage to resync past — a stray byte must never be read
    /// as a length and stall the parser waiting for bytes that will not come.
    fn known_head(b: u8) -> bool {
        matches!(
            b,
            0x08 | 0x09
                | 0x10
                | 0x11
                | 0x20
                | 0x21
                | 0x22
                | 0x24
                | 0x25
                | 0x31
                | 0x32
                | 0x33
                | 0x34
                | 0x35
                | 0x41
                | 0x42
                | 0x43
                | 0x44
                | 0x7F
        )
    }

    /// Feed bytes; returns every complete frame in order.
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<DviFrame> {
        self.buf.extend_from_slice(bytes);
        let mut out = Vec::new();
        loop {
            match self.buf.first() {
                None => break,
                Some(&b) if !Self::known_head(b) => {
                    self.buf.remove(0);
                    continue;
                }
                _ => {}
            }
            let Some(total) = self.frame_len() else { break };
            if self.buf.len() < total {
                break;
            }
            let raw: Vec<u8> = self.buf.drain(..total).collect();
            let (body, yy) = raw.split_at(total - 1);
            if checksum(body) != yy[0] {
                // Bad checksum: resync one byte past this command byte.
                let mut rest = raw[1..].to_vec();
                rest.extend_from_slice(&self.buf);
                self.buf = rest;
                continue;
            }
            if let Some(f) = self.decode(body) {
                out.push(f);
            }
        }
        out
    }

    /// Total frame length (incl. checksum) for the frame at the buffer head.
    fn frame_len(&self) -> Option<usize> {
        let cmd = *self.buf.first()?;
        if cmd == cmd::RX_LARGE || cmd == cmd::TX_LARGE || cmd == cmd::TX_LARGE + 0x10 {
            if self.buf.len() < 3 {
                return None;
            }
            let len = ((self.buf[1] as usize) << 8) | self.buf[2] as usize;
            Some(3 + len + 1)
        } else {
            let len = *self.buf.get(1)? as usize;
            Some(2 + len + 1)
        }
    }

    fn decode(&self, body: &[u8]) -> Option<DviFrame> {
        let cmd = body[0];
        let data = if cmd == cmd::RX_LARGE || cmd == cmd::TX_LARGE + 0x10 {
            &body[3..]
        } else {
            &body[2..]
        };
        match cmd {
            cmd::ERROR => Some(DviFrame::Error {
                cmd: *data.first()?,
                code: *data.get(1)?,
            }),
            cmd::RX_NORMAL | cmd::RX_LARGE => {
                let (ts_us, rest) = if self.timestamps && data.len() >= 4 {
                    (
                        Some(u32::from_be_bytes([data[0], data[1], data[2], data[3]])),
                        &data[4..],
                    )
                } else {
                    (None, data)
                };
                if rest.len() < 4 {
                    return None;
                }
                let id = u32::from_be_bytes([rest[0], rest[1], rest[2], rest[3]]);
                Some(DviFrame::Rx(RxFrame {
                    id,
                    ts_us,
                    data: rest[4..].to_vec(),
                }))
            }
            _ => Some(DviFrame::Reply {
                cmd,
                data: data.to_vec(),
            }),
        }
    }
}

// ---- request builders -------------------------------------------------------

/// `22 01 XX` scantool information.
pub fn info_request(sub: u8) -> Vec<u8> {
    frame(cmd::INFO, &[sub])
}

/// `31 02 01 XX` set OBD protocol.
pub fn set_protocol(p: ObdProtocol) -> Vec<u8> {
    frame(cmd::PROTOCOL, &[0x01, p as u8])
}

/// `31 02 02 XX` network communication.
pub fn set_comm(c: Comm) -> Vec<u8> {
    frame(cmd::PROTOCOL, &[0x02, c as u8])
}

/// `31 02 06 01` stay in DVI (`00` would return to ELM — never sent).
pub fn set_api_dvi() -> Vec<u8> {
    frame(cmd::PROTOCOL, &[0x06, 0x01])
}

/// `24 02 03 XX` RX timestamps on/off.
pub fn set_timestamps(on: bool) -> Vec<u8> {
    frame(cmd::SETTINGS, &[0x03, on as u8])
}

/// `34 02 0E XX` CAN padding on/off (§3.14.15 — every written frame padded
/// to 8 bytes with the §3.14.14 pad byte, as an ELM does by default; many
/// ECUs ignore a DLC-3 request).
pub fn set_padding(on: bool) -> Vec<u8> {
    frame(cmd::CAN, &[0x0E, on as u8])
}

/// `34 02 06 XX` hardware (00) or software (01) filters (§3.14.7 — hardware
/// = 28 11-bit + 8 29-bit slots; software = the full 32 their J2534 uses).
pub fn set_software_filters(on: bool) -> Vec<u8> {
    frame(cmd::CAN, &[0x06, on as u8])
}

/// `34 02 15 XX` predefined CAN baud rate (§3.14.22): 6 = 500 kbps
/// (HS-CAN default), 5 = 250 kbps.
pub fn set_baud(code: u8) -> Vec<u8> {
    frame(cmd::CAN, &[0x15, code])
}
pub const BAUD_500K: u8 = 6;
pub const BAUD_250K: u8 = 5;

/// `34 04 1A slot NN XX` periodic-frame interval in ms (§3.14.26.1, 8 slots).
pub fn periodic_interval(slot: u8, ms: u16) -> Vec<u8> {
    let b = ms.to_be_bytes();
    frame(cmd::CAN, &[0x1A, slot, b[0], b[1]])
}

/// `34 LEN 1B slot <4-byte id> <payload ≤ 8>` periodic-frame data
/// (§3.14.26.2 — header + data bytes; the length byte is auto-formatted
/// like a `10` write).
pub fn periodic_data(slot: u8, id: u32, payload: &[u8]) -> Vec<u8> {
    let mut d = vec![0x1B, slot];
    d.extend_from_slice(&id.to_be_bytes());
    d.extend_from_slice(payload);
    frame(cmd::CAN, &d)
}

/// `34 03 1C slot XX` periodic-frame enable (§3.14.26.3).
pub fn periodic_enable(slot: u8, on: bool) -> Vec<u8> {
    frame(cmd::CAN, &[0x1C, slot, on as u8])
}

/// `34 11 00 …` entire CAN filter.
pub fn can_filter(
    number: u8,
    extended: bool,
    kind: FilterType,
    on: bool,
    id: u32,
    mask: u32,
    flow_id: u32,
) -> Vec<u8> {
    // The manual's layout: `34 11 00 MM NN XX ZZ ID MASK FLOW` (LEN 0x11 = 17 data bytes).
    let mut d = Vec::with_capacity(17);
    d.push(0x00);
    d.push(number);
    d.push(extended as u8);
    d.push(kind as u8);
    d.push(on as u8);
    d.extend_from_slice(&id.to_be_bytes());
    d.extend_from_slice(&mask.to_be_bytes());
    d.extend_from_slice(&flow_id.to_be_bytes());
    frame(cmd::CAN, &d)
}

/// `34 04 05 MM NN XX` filter enable status.
pub fn can_filter_status(number: u8, extended: bool, on: bool) -> Vec<u8> {
    frame(cmd::CAN, &[0x05, number, extended as u8, on as u8])
}

/// `10 …` send one CAN frame: 4-byte id + payload (the tool adds the CAN
/// length byte). >255 bytes uses `11`.
pub fn tx_can(id: u32, extended: bool, payload: &[u8]) -> Vec<u8> {
    let _ = extended; // 29-bit ids are simply the 4-byte value
    let mut d = id.to_be_bytes().to_vec();
    d.extend_from_slice(payload);
    if d.len() > 0xFF {
        frame(cmd::TX_LARGE, &d)
    } else {
        frame(cmd::TX_NORMAL, &d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        s.split_whitespace()
            .map(|t| u8::from_str_radix(t, 16).unwrap())
            .collect()
    }

    /// §3.2 — the manual's three hand-computed checksums.
    #[test]
    fn checksum_matches_the_manual() {
        assert_eq!(checksum(&hex("22 01 00")), 0xDC);
        assert_eq!(checksum(&hex("32 05 00 01 00 00 00")), 0xC7);
        assert_eq!(checksum(&hex("32 05 00 FF FE FC FD")), 0xD2);
    }

    /// §3.16.3 — every frame of the worked HS-CAN example, byte for byte.
    #[test]
    fn hs_can_example_frames() {
        assert_eq!(set_protocol(ObdProtocol::HsCan), hex("31 02 01 02 C9"));
        assert_eq!(set_protocol(ObdProtocol::GmCan), hex("31 02 01 03 C8"));
        assert_eq!(
            can_filter(0, false, FilterType::Flow, true, 0x7E8, 0x7FF, 0x7E0),
            hex("34 11 00 00 00 01 01 00 00 07 E8 00 00 07 FF 00 00 07 E0 DC")
        );
        assert_eq!(set_comm(Comm::On), hex("31 02 02 01 C9"));
        assert_eq!(
            tx_can(0x7E0, false, &[0x20]),
            hex("10 05 00 00 07 E0 20 E3")
        );
        // §3.6.3 example 1 (YY left as a placeholder in the manual → computed)
        let mut ex = hex("10 06 00 00 07 E0 01 00");
        ex.push(checksum(&ex));
        assert_eq!(tx_can(0x7E0, false, &[0x01, 0x00]), ex);
        assert_eq!(info_request(info::HARDWARE_VERSION), hex("22 01 00 DC"));
        assert_eq!(info_request(info::MODEL), hex("22 01 02 DA"));
        assert_eq!(set_api_dvi(), hex("31 02 06 01 C5"));
        // §3.14.26.2 — the manual's periodic-data example (YY computed).
        let mut pd = hex("34 08 1B 00 00 00 07 E0 01 3E");
        pd.push(checksum(&pd));
        assert_eq!(periodic_data(0, 0x7E0, &[0x01, 0x3E]), pd);
        let mut pi = hex("34 04 1A 03 00 64");
        pi.push(checksum(&pi));
        assert_eq!(periodic_interval(3, 100), pi);
        let mut pe = hex("34 03 1C 03 01");
        pe.push(checksum(&pe));
        assert_eq!(periodic_enable(3, true), pe);
        let mut baud = hex("34 02 15 05");
        baud.push(checksum(&baud));
        assert_eq!(set_baud(BAUD_250K), baud);
        let mut pad = hex("34 02 0E 01");
        pad.push(checksum(&pad));
        assert_eq!(set_padding(true), pad);
        assert_eq!(set_comm(Comm::ListenOnly), hex("31 02 02 02 C8"));
    }

    #[test]
    fn parses_rx_reply_and_error_frames() {
        let mut p = FrameParser::new();
        // §3.16.3 step 5: RX 7E8 01 60 ; step 4 ack ; §3.3 error ; §3.4.3 example 1
        let frames = p.feed(&hex(
            "08 06 00 00 07 E8 01 60 A1 20 01 00 DE 7F 02 22 05 57",
        ));
        assert_eq!(frames.len(), 3, "{frames:?}");
        assert_eq!(
            frames[0],
            DviFrame::Rx(RxFrame {
                id: 0x7E8,
                ts_us: None,
                data: vec![0x01, 0x60]
            })
        );
        assert_eq!(
            frames[1],
            DviFrame::Reply {
                cmd: 0x20,
                data: vec![0x00]
            }
        );
        assert_eq!(frames[2], DviFrame::Error { cmd: 0x22, code: 5 });
        let f = p.feed(&frame(cmd::RX_NORMAL, &hex("00 00 07 E8 04 41 0C 09 C4")));
        assert_eq!(
            f,
            vec![DviFrame::Rx(RxFrame {
                id: 0x7E8,
                ts_us: None,
                data: hex("04 41 0C 09 C4")
            })]
        );
    }

    #[test]
    fn parser_handles_split_chunks_and_resyncs_on_bad_checksum() {
        let mut p = FrameParser::new();
        let whole = hex("08 06 00 00 07 E8 01 60 A1");
        assert!(p.feed(&whole[..4]).is_empty());
        let f = p.feed(&whole[4..]);
        assert_eq!(f.len(), 1);
        // Corrupt checksum, then a good frame: the bad one is dropped, the good one survives.
        let mut bad = whole.clone();
        *bad.last_mut().unwrap() ^= 0xFF;
        bad.extend_from_slice(&whole);
        let f = p.feed(&bad);
        assert_eq!(f.len(), 1);
    }

    #[test]
    fn timestamped_rx_and_large_frames() {
        let mut p = FrameParser::new();
        p.timestamps = true;
        let f = p.feed(&frame(
            cmd::RX_NORMAL,
            &hex("00 00 03 E8 00 00 07 E8 04 41 0C 09 C4"),
        ));
        assert_eq!(
            f,
            vec![DviFrame::Rx(RxFrame {
                id: 0x7E8,
                ts_us: Some(1000),
                data: hex("04 41 0C 09 C4")
            })]
        );
        p.timestamps = false;
        // §3.5.3 example 1 (large form; YY computed)
        let f = p.feed(&frame(cmd::RX_LARGE, &hex("00 00 07 E8 04 41 0C 09 C4")));
        assert_eq!(
            f,
            vec![DviFrame::Rx(RxFrame {
                id: 0x7E8,
                ts_us: None,
                data: hex("04 41 0C 09 C4")
            })]
        );
        // 29-bit id
        let f = p.feed(&frame(
            cmd::RX_NORMAL,
            &hex("18 DA F1 10 06 41 00 BF FE B9 93"),
        ));
        assert_eq!(
            f[0],
            DviFrame::Rx(RxFrame {
                id: 0x18DAF110,
                ts_us: None,
                data: hex("06 41 00 BF FE B9 93")
            })
        );
    }
}
