//! CAN addressing model: 11-bit vs 29-bit (ISO 15765-4).
//!
//! Two types plus a wrapper own every wire-format conversion, so the rest of the stack
//! (subscription/registry/session-monitor/FFI) holds a protocol-neutral [`Controller`] and
//! never touches header strings:
//!
//! - [`Addressing`] — the protocol descriptor negotiated for a session.
//! - [`Controller`] — a uniform, structured ECU identity (the ECU address byte).
//! - [`Bus`] — bundles the addressing and does `read` (response header → `Controller`) and
//!   `request_header`/`functional_header` (`Controller` → `ATSH`), plus `from_target`.
//!
//! Payload is header-independent: once [`Bus::read`] strips the header, an 11-bit and a 29-bit
//! message are byte-identical, so all downstream parsing is shared. See
//! `docs/29bit_CAN_Support_Plan.md`.

/// CAN addressing negotiated for a session. One arm per protocol (add future protocols here
/// plus the matching [`Bus`] arms). `tester` is the external-test-equipment address — learned
/// from the wire, not hardcoded (decision A).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Addressing {
    /// 11-bit CAN: functional `7DF`, request `7E0-7E7`, response `7E8-7EF`.
    Can11,
    /// 29-bit CAN (ISO 15765-4): functional `18DB33F1`, request `18DAxxF1`, response `18DAF1xx`.
    Can29 { tester: u8 },
}

impl Addressing {
    /// Map an `ATDPN` protocol number to an addressing mode. Returns `None` for non-CAN
    /// protocols (1–5: J1850 / ISO 9141 / KWP) and anything unrecognized — the caller fails the
    /// connection with an "unsupported protocol" error (decision C). `tester` is seeded to the
    /// conventional `0xF1`; the real value is learned from the first response (see [`Addressing::sniff`]).
    pub fn from_atdpn(n: &str) -> Option<Addressing> {
        match n.trim() {
            "6" | "8" => Some(Addressing::Can11), // 500k / 250k, 11-bit
            "7" | "9" => Some(Addressing::Can29 { tester: 0xF1 }), // 500k / 250k, 29-bit
            _ => None,
        }
    }

    /// Detect the addressing from a real response's leading tokens (decision B — primary path):
    /// a 3-char `7Ex` → 11-bit; a 4-token `18 DA <tester> <src>` (physical response, format byte
    /// `DA`) → 29-bit, learning `tester` from `byte[2]` (decision A). Returns `None` if the tokens
    /// aren't a recognizable CAN response header.
    pub fn sniff(tokens: &[&str]) -> Option<Addressing> {
        let first = tokens.first()?;
        // 11-bit: single 3-char token starting with '7' (7E8, 7EA, …).
        if first.len() == 3 && first.starts_with('7') && is_hex(first) {
            return Some(Addressing::Can11);
        }
        // 29-bit response: 18 DA <tester> <src>, all one-byte hex tokens, format byte DA.
        let b: Vec<u8> = tokens
            .iter()
            .take(4)
            .filter_map(|t| u8::from_str_radix(t, 16).ok())
            .collect();
        if b.len() == 4 && b[1] == 0xDA {
            return Some(Addressing::Can29 { tester: b[2] });
        }
        None
    }

    /// Human-readable protocol string for `VehicleInfo.protocol` (decision 6). The 250k variants
    /// share the model but aren't distinguished here yet; refine when a 250k vehicle surfaces.
    pub fn display(&self) -> String {
        match self {
            Addressing::Can11 => "ISO 15765-4 CAN 11-bit (500k)".to_string(),
            Addressing::Can29 { .. } => "ISO 15765-4 CAN 29-bit (500k)".to_string(),
        }
    }
}

/// A logical ECU identity, protocol-scoped (GB14 widening, clean break):
/// - **29-bit**: the diagnostic address byte `aa` (`18DAF110`/`14DAF141` → `0x10`/`0x41`).
/// - **11-bit**: the FULL 11-bit request CAN id (`7E8` → `0x7E0`; low-range `654` → `0x254`) —
///   the old 3-bit ecu index couldn't represent the GM low-range body block.
/// Serialized key = `"{:02X}"` (grows naturally: `"10"`, `"7E0"`, `"254"`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Controller {
    pub id: u16,
}

impl Controller {
    pub fn new(id: u16) -> Self {
        Controller { id }
    }

    /// Stable key for maps / JSON / FFI (`"10"`, `"7E0"`, `"254"`).
    pub fn key(&self) -> String {
        format!("{:02X}", self.id)
    }
}

/// A session's addressing context. Owns every wire-format conversion so callers never thread
/// `addressing` or touch header strings.
#[derive(Clone, Copy, Debug)]
pub struct Bus {
    pub addressing: Addressing,
}

impl Bus {
    pub fn new(addressing: Addressing) -> Self {
        Bus { addressing }
    }

    /// Recognize a controller header at the start of `tokens` (generalizes the old
    /// `is_controller_id`). Returns `(Controller, tokens_consumed)`, or `None` if there's no
    /// header here (headerless payload).
    ///
    /// Matches by *structure*, not the literal `18 DA F1`: a 29-bit response is 4 one-byte hex
    /// tokens whose format byte (`byte[1]`) is `DA` and whose destination (`byte[2]`) is our learned
    /// tester — so it survives a non-`F1` tester or an odd priority byte.
    pub fn read(&self, tokens: &[&str]) -> Option<(Controller, usize)> {
        match self.addressing {
            Addressing::Can11 => {
                let t = tokens.first()?;
                if t.len() != 3 || !is_hex(t) {
                    return None;
                }
                let id = u16::from_str_radix(t, 16).ok()?;
                // Attribute a response to its REQUEST id (the identity). GB14 ranges:
                // standard block 7E8-7EF → -8 (7E0-7E7 request-form echoes keep as-is);
                // FS0: the Ford diagnostic universe 700-7EF answers request+8
                // (728→720, 738→730, 768→760, 7D8→7D0 — busscan.txt's
                // ATCR709-7EF listen window), so 708-7DF attributes -8 too.
                // NOTE the asymmetry with from_target: REQUEST headers also
                // live in 708-7DF ("720", "730"), so only RESPONSE attribution
                // generalizes — target strings stay verbatim.
                // GM low-range 641-65F → -0x400 (controllers.md). 5xx UUDT lines and
                // anything else are not physical responses we attribute → None.
                let req = match id {
                    0x7E8..=0x7EF => id - 8,
                    0x7E0..=0x7E7 => id,
                    0x708..=0x7DF => id - 8,
                    0x641..=0x65F => id - 0x400,
                    _ => return None,
                };
                Some((Controller { id: req }, 1))
            }
            Addressing::Can29 { tester } => {
                let b: Vec<u8> = tokens
                    .iter()
                    .take(4)
                    .filter_map(|t| u8::from_str_radix(t, 16).ok())
                    .collect();
                if b.len() == 4 && b[1] == 0xDA && b[2] == tester {
                    Some((Controller { id: b[3] as u16 }, 4))
                } else {
                    None
                }
            }
        }
    }

    /// Functional (broadcast) request header for `ATSH`.
    pub fn functional_header(&self) -> String {
        match self.addressing {
            Addressing::Can11 => "7DF".to_string(),
            Addressing::Can29 { tester } => format!("18DB33{:02X}", tester),
        }
    }

    /// Physical `ATSH` request header targeting one ECU.
    pub fn request_header(&self, c: Controller) -> String {
        match self.addressing {
            Addressing::Can11 => format!("{:03X}", c.id), // 0x7E0 → 7E0, 0x254 → 254
            Addressing::Can29 { tester } => format!("18DA{:02X}{:02X}", c.id, tester), // 0x10 → 18DA10F1
        }
    }

    /// Normalize an inbound target string — an imported Torque `Header` / PID `preferredController`
    /// (`"7DF"`/`"7E0"`/`"7E8"`), or a bare ECU key (`"10"`) — to a [`Controller`] in the *active*
    /// protocol. `None` means "no specific controller" → caller uses [`Bus::functional_header`].
    ///
    /// An 11-bit header has no equivalent in 29-bit's separate address space (ECUs `0x10/0x28/0x40…`,
    /// not `0-7`), so on `Can29` a `7Ex`/`7DF` string returns `None` → functional fallback; the
    /// extended scan then attributes the response by its source header (C1/C3).
    pub fn from_target(&self, s: &str) -> Option<Controller> {
        let s = s.trim().to_uppercase();
        // Functional broadcast is never a specific controller.
        if s == "7DF" || s == "18DB33F1" {
            return None;
        }
        match self.addressing {
            Addressing::Can11 => {
                // Full 11-bit ids verbatim (GB14): "7E0"-"7E7", low-range "254", any ≤0x7FF.
                // Response forms normalize to the request id: 7E8-7EF → -8, 641-65F → -0x400.
                // Legacy bare ecu-index keys "00"-"07" → the 7E0 block. Other bare bytes
                // (0x08-0xFF) are ambiguous → None → functional.
                let id = u16::from_str_radix(&s, 16).ok()?;
                let id = match id {
                    0x7E8..=0x7EF => id - 8,
                    0x641..=0x65F => id - 0x400,
                    0x000..=0x007 => 0x7E0 + id,
                    0x008..=0x0FF => return None,
                    0x100..=0x7FF => id,
                    _ => return None,
                };
                Some(Controller { id })
            }
            Addressing::Can29 { .. } => {
                // GB13: a full 8-hex header states priority+format+target+source itself
                // ("14DA41F1" GM-enhanced / "18DA10F1" ISO). Format DA = physical → the
                // target byte is the ECU. DB (functional) and DC (UUDT, module→tester)
                // are not per-ECU request targets → None → functional fallback.
                if s.len() == 8 && is_hex(&s) {
                    if &s[2..4] == "DA" {
                        return u8::from_str_radix(&s[4..6], 16)
                            .ok()
                            .map(|aa| Controller { id: aa as u16 });
                    }
                    return None;
                }
                // An 11-bit header can't map to 29-bit → functional fallback.
                if s.starts_with("7E") {
                    return None;
                }
                // A bare ECU byte (a discovered 29-bit source, or our own key "10").
                u8::from_str_radix(&s, 16)
                    .ok()
                    .map(|aa| Controller { id: aa as u16 })
            }
        }
    }

    /// Resolve an inbound target string to the `ATSH` header to send: the ECU's physical request
    /// header, or the functional broadcast header when the target can't apply to the active
    /// protocol (an imported 11-bit header on a 29-bit bus). The C3 targeting contract.
    ///
    /// GB13: a full 8-hex physical header string ("14DA41F1") is ATSH-ready and passes through
    /// VERBATIM — the data states its own priority (GM enhanced `14` vs ISO `18`), and
    /// re-rendering via [`Bus::request_header`] would clobber it with the `18` default.
    pub fn target_header(&self, target: &str) -> String {
        let s = target.trim().to_uppercase();
        if matches!(self.addressing, Addressing::Can29 { .. })
            && s.len() == 8
            && is_hex(&s)
            && &s[2..4] == "DA"
        {
            return s;
        }
        match self.from_target(&s) {
            Some(c) => self.request_header(c),
            None => self.functional_header(),
        }
    }

    /// Leading CAN id of a monitor-mode line, normalized to the compact form
    /// SP decoders/filters key on (`"7E8"`, `"18DAF110"`). 11-bit: the first
    /// token. 29-bit: monitor lines print the id as four spaced one-byte
    /// tokens (the same shape [`Bus::read`] consumes) — join them. Falls back
    /// to the first token so non-frame lines (acks, `STOPPED`) still yield a
    /// key that simply matches no decoder.
    pub fn monitor_id(&self, line: &str) -> Option<String> {
        let mut tokens = line.split_whitespace();
        match self.addressing {
            Addressing::Can11 => tokens.next().map(str::to_uppercase),
            Addressing::Can29 { .. } => {
                let four: Vec<&str> = line.split_whitespace().take(4).collect();
                if four.len() == 4 && four.iter().all(|t| t.len() == 2 && is_hex(t)) {
                    Some(four.join("").to_uppercase())
                } else {
                    tokens.next().map(str::to_uppercase)
                }
            }
        }
    }
}

fn is_hex(s: &str) -> bool {
    !s.is_empty() && s.chars().all(|c| c.is_ascii_hexdigit())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn atdpn_maps_can_protocols_and_rejects_non_can() {
        assert_eq!(Addressing::from_atdpn("6"), Some(Addressing::Can11));
        assert_eq!(Addressing::from_atdpn("8"), Some(Addressing::Can11));
        assert_eq!(
            Addressing::from_atdpn("7"),
            Some(Addressing::Can29 { tester: 0xF1 })
        );
        assert_eq!(
            Addressing::from_atdpn("9"),
            Some(Addressing::Can29 { tester: 0xF1 })
        );
        // Non-CAN (J1850 / ISO 9141 / KWP) and junk → None → caller fails the connection.
        for n in ["1", "2", "3", "4", "5", "0", "A", ""] {
            assert_eq!(
                Addressing::from_atdpn(n),
                None,
                "protocol {n} should be non-CAN"
            );
        }
    }

    #[test]
    fn sniff_detects_mode_and_learns_tester() {
        // 11-bit response.
        let t: Vec<&str> = "7E8 04 41 0C 1A F8".split_whitespace().collect();
        assert_eq!(Addressing::sniff(&t), Some(Addressing::Can11));

        // 29-bit response (Jeep 0100 from ECU 0x10) — learns tester = F1 from byte[2].
        let t: Vec<&str> = "18 DA F1 10 06 41 00 BF FE B9 93"
            .split_whitespace()
            .collect();
        assert_eq!(
            Addressing::sniff(&t),
            Some(Addressing::Can29 { tester: 0xF1 })
        );

        // Non-default tester is learned, not assumed.
        let t: Vec<&str> = "18 DA F0 11 06 41 00 80 00 00 01"
            .split_whitespace()
            .collect();
        assert_eq!(
            Addressing::sniff(&t),
            Some(Addressing::Can29 { tester: 0xF0 })
        );

        // Not a CAN header.
        assert_eq!(Addressing::sniff(&["NO", "DATA"]), None);
        assert_eq!(Addressing::sniff(&[]), None);
    }

    #[test]
    fn controller_key_is_uniform() {
        assert_eq!(Controller::new(0x7E0).key(), "7E0");
        assert_eq!(Controller::new(0x10).key(), "10");
        assert_eq!(Controller::new(0x28).key(), "28");
    }

    #[test]
    fn read_11bit_response_headers() {
        let bus = Bus::new(Addressing::Can11);
        let t: Vec<&str> = "7E8 04 41 0C 1A F8".split_whitespace().collect();
        assert_eq!(bus.read(&t), Some((Controller::new(0x7E0), 1)));
        let t: Vec<&str> = "7E9 06 41 00 80 00 00 01".split_whitespace().collect();
        assert_eq!(bus.read(&t), Some((Controller::new(0x7E1), 1)));
        // GB14: low-range body responses attribute at -0x400 (654 → request 254 = EPB).
        let t: Vec<&str> = "654 04 62 41 1E 01".split_whitespace().collect();
        assert_eq!(bus.read(&t), Some((Controller::new(0x254), 1)));
        // FS0: Ford 700-7EF universe — response = request + 8.
        let t: Vec<&str> = "728 05 62 F1 11 49".split_whitespace().collect();
        assert_eq!(
            bus.read(&t),
            Some((Controller::new(0x720), 1)),
            "IPC 728 → 720"
        );
        let t: Vec<&str> = "768 05 62 F1 11 41".split_whitespace().collect();
        assert_eq!(
            bus.read(&t),
            Some((Controller::new(0x760), 1)),
            "ABS 768 → 760"
        );
        let t: Vec<&str> = "7D8 05 62 F1 11 53".split_whitespace().collect();
        assert_eq!(
            bus.read(&t),
            Some((Controller::new(0x7D0), 1)),
            "APIM 7D8 → 7D0"
        );
        let t: Vec<&str> = "70E 05 62 F1 11 49".split_whitespace().collect();
        assert_eq!(
            bus.read(&t),
            Some((Controller::new(0x706), 1)),
            "IPMA 70E → 706"
        );
        // 5xx UUDT lines are not physical responses.
        assert_eq!(bus.read(&["554", "04", "62", "41", "1E", "01"]), None);
        // Headerless payload → None.
        assert_eq!(bus.read(&["41", "0C", "1A", "F8"]), None);
    }

    #[test]
    fn read_29bit_response_headers_from_captured_frames() {
        let bus = Bus::new(Addressing::Can29 { tester: 0xF1 });
        // Jeep ECUs.
        let t: Vec<&str> = "18 DA F1 10 06 41 00 BF FE B9 93"
            .split_whitespace()
            .collect();
        assert_eq!(bus.read(&t), Some((Controller::new(0x10), 4)));
        let t: Vec<&str> = "18 DA F1 18 06 41 00 98 18 00 01"
            .split_whitespace()
            .collect();
        assert_eq!(bus.read(&t), Some((Controller::new(0x18), 4)));
        // Tahoe non-contiguous ECUs.
        let t: Vec<&str> = "18 DA F1 28 06 41 01 00 04 00 00"
            .split_whitespace()
            .collect();
        assert_eq!(bus.read(&t), Some((Controller::new(0x28), 4)));
        let t: Vec<&str> = "18 DA F1 40 06 41 00 80 00 00 01"
            .split_whitespace()
            .collect();
        assert_eq!(bus.read(&t), Some((Controller::new(0x40), 4)));
    }

    #[test]
    fn read_29bit_rejects_wrong_tester() {
        // A response addressed to a different tester isn't ours.
        let bus = Bus::new(Addressing::Can29 { tester: 0xF1 });
        let t: Vec<&str> = "18 DA F0 10 06 41 00 BF FE B9 93"
            .split_whitespace()
            .collect();
        assert_eq!(bus.read(&t), None);
    }

    #[test]
    fn request_and_functional_headers_both_protocols() {
        let bus11 = Bus::new(Addressing::Can11);
        assert_eq!(bus11.functional_header(), "7DF");
        assert_eq!(bus11.request_header(Controller::new(0x7E0)), "7E0");
        assert_eq!(bus11.request_header(Controller::new(0x7E1)), "7E1");
        assert_eq!(bus11.request_header(Controller::new(0x254)), "254"); // GB14 low range

        let bus29 = Bus::new(Addressing::Can29 { tester: 0xF1 });
        assert_eq!(bus29.functional_header(), "18DB33F1");
        assert_eq!(bus29.request_header(Controller::new(0x10)), "18DA10F1");
        assert_eq!(bus29.request_header(Controller::new(0x28)), "18DA28F1");
    }

    #[test]
    fn request_response_round_trip_29bit() {
        // The physical request header we send, and the response header the ECU replies with,
        // must resolve back to the same Controller.
        let bus = Bus::new(Addressing::Can29 { tester: 0xF1 });
        let ecu = Controller::new(0x11);
        assert_eq!(bus.request_header(ecu), "18DA11F1"); // request: target=ECU, source=tester
        let resp: Vec<&str> = "18 DA F1 11 10 14 49 0A 01 45".split_whitespace().collect();
        assert_eq!(bus.read(&resp).map(|(c, _)| c), Some(ecu)); // response: target=tester, source=ECU
    }

    #[test]
    fn from_target_11bit_imported_torque_headers() {
        let bus = Bus::new(Addressing::Can11);
        // Torque request header "7E0", response form "7E8", bare key "00" — all → ecu 0.
        assert_eq!(bus.from_target("7E0"), Some(Controller::new(0x7E0)));
        assert_eq!(bus.from_target("7E8"), Some(Controller::new(0x7E0)));
        assert_eq!(bus.from_target("00"), Some(Controller::new(0x7E0))); // legacy ecu-index key
        assert_eq!(bus.from_target("7E1"), Some(Controller::new(0x7E1)));
        // GB14: low-range request, response form, and UUDT-never.
        assert_eq!(bus.from_target("254"), Some(Controller::new(0x254)));
        assert_eq!(bus.from_target("654"), Some(Controller::new(0x254)));
        // Functional broadcast → None (caller uses functional_header).
        assert_eq!(bus.from_target("7DF"), None);
    }

    #[test]
    fn target_header_resolution_c3_contract() {
        // 11-bit: imported Torque targets resolve to the same ATSH as before the refactor.
        let b11 = Bus::new(Addressing::Can11);
        assert_eq!(b11.target_header("7E0"), "7E0"); // physical request, unchanged
        assert_eq!(b11.target_header("7E8"), "7E0"); // response form → same ecu → request 7E0
        assert_eq!(b11.target_header("7DF"), "7DF"); // functional
                                                     // 29-bit: a discovered ECU targets physically; an imported 11-bit header → functional.
        let b29 = Bus::new(Addressing::Can29 { tester: 0xF1 });
        assert_eq!(b29.target_header("10"), "18DA10F1"); // physical to discovered ecu 0x10
        assert_eq!(b29.target_header("28"), "18DA28F1");
        assert_eq!(b29.target_header("7E0"), "18DB33F1"); // 11-bit header → functional fallback
    }

    #[test]
    fn monitor_id_normalizes_both_protocols() {
        // 11-bit monitor line → first token.
        let b11 = Bus::new(Addressing::Can11);
        assert_eq!(
            b11.monitor_id("7E8 06 41 0C 3F E3 0D 73"),
            Some("7E8".into())
        );
        assert_eq!(b11.monitor_id("STOPPED"), Some("STOPPED".into()));
        assert_eq!(b11.monitor_id(""), None);

        // 29-bit monitor line → first four byte-tokens joined (captured Jeep shape).
        let b29 = Bus::new(Addressing::Can29 { tester: 0xF1 });
        assert_eq!(
            b29.monitor_id("18 DA F1 10 06 41 00 BF FE B9 93"),
            Some("18DAF110".into())
        );
        assert_eq!(
            b29.monitor_id("18 DA F1 18 05 41 05 7F 00"),
            Some("18DAF118".into())
        );
        // Non-frame lines fall back to the first token (matches no decoder).
        assert_eq!(b29.monitor_id("STOPPED"), Some("STOPPED".into()));
        assert_eq!(b29.monitor_id("OK"), Some("OK".into()));
    }

    #[test]
    fn from_target_29bit_11bit_header_falls_back_to_functional() {
        let bus = Bus::new(Addressing::Can29 { tester: 0xF1 });
        // An imported 11-bit Torque header has no 29-bit equivalent → None → functional fallback.
        assert_eq!(bus.from_target("7E0"), None);
        assert_eq!(bus.from_target("7E8"), None);
        assert_eq!(bus.from_target("7DF"), None);
        assert_eq!(bus.from_target("18DB33F1"), None);
        // A discovered 29-bit ECU byte does resolve.
        assert_eq!(bus.from_target("10"), Some(Controller::new(0x10)));
        assert_eq!(bus.from_target("28"), Some(Controller::new(0x28)));
    }

    // MARK: GB13 — the priority prefix comes from the data, per target.

    #[test]
    fn from_target_29bit_full_physical_headers_both_priorities() {
        let bus = Bus::new(Addressing::Can29 { tester: 0xF1 });
        // GM-enhanced priority 14 (headers.json shapes) and ISO 18 both parse; the
        // target byte is the ECU identity.
        assert_eq!(bus.from_target("14DA41F1"), Some(Controller::new(0x41))); // LCM
        assert_eq!(bus.from_target("14DA80F1"), Some(Controller::new(0x80))); // RADIO
        assert_eq!(bus.from_target("18DA10F1"), Some(Controller::new(0x10)));
        assert_eq!(bus.from_target("14da11f1"), Some(Controller::new(0x11))); // case-blind
                                                                              // Non-physical formats are not per-ECU targets: DB functional, DC UUDT
                                                                              // (module→tester response channel — never something we address).
        assert_eq!(bus.from_target("14DB33F1"), None);
        assert_eq!(bus.from_target("14DCF141"), None);
    }

    #[test]
    fn target_header_29bit_full_headers_pass_through_verbatim() {
        let bus = Bus::new(Addressing::Can29 { tester: 0xF1 });
        // The stated priority survives — no re-render through the 18 default.
        assert_eq!(bus.target_header("14DA41F1"), "14DA41F1");
        assert_eq!(bus.target_header("14da80f1"), "14DA80F1");
        assert_eq!(bus.target_header("18DA10F1"), "18DA10F1");
        // Bare ECU keys still render with the ISO default (roster / standard surface).
        assert_eq!(bus.target_header("41"), "18DA41F1");
        // Non-physical forms → functional fallback, unchanged.
        assert_eq!(bus.target_header("14DCF141"), "18DB33F1");
        // 11-bit bus: an 8-hex 29-bit header still can't apply → functional.
        let b11 = Bus::new(Addressing::Can11);
        assert_eq!(b11.target_header("14DA41F1"), "7DF");
    }

    #[test]
    fn read_29bit_accepts_gm_priority_responses() {
        // Receive side was already priority-blind (format byte + tester matching);
        // pin it: a 14-priority physical response attributes to its source module.
        let bus = Bus::new(Addressing::Can29 { tester: 0xF1 });
        let t: Vec<&str> = "14 DA F1 41 03 59 02 8D".split_whitespace().collect();
        assert_eq!(bus.read(&t), Some((Controller::new(0x41), 4)));
        // A UUDT line (format DC) is NOT a physical response — read() skips it;
        // monitor_id still keys it for stream decoders.
        let t: Vec<&str> = "14 DC F1 41 05 62 F2 FE 00".split_whitespace().collect();
        assert_eq!(bus.read(&t), None);
        assert_eq!(
            bus.monitor_id("14 DC F1 41 05 62 F2 FE 00"),
            Some("14DCF141".into())
        );
        assert_eq!(
            bus.monitor_id("14 DA F1 80 03 7F 19 11"),
            Some("14DAF180".into())
        );
    }
}

/// Physical response id for a request header (SP / STPX filters). 11-bit:
/// `7E0`→`7E8`, `73B`→`743` (+8); GB14 GM low-range body block `241..=25F`
/// answers at +0x400. 29-bit: `18DA<ecu><tester>`→`18DA<tester><ecu>` (the
/// target/source byte swap, priority preserved — GB13 `14DA…` alike). Anything
/// else falls back to the header unchanged.
pub fn periodic_response_id(header: &str) -> String {
    let h = header.trim().to_uppercase();
    if h.len() <= 3 {
        if let Ok(n) = u16::from_str_radix(&h, 16) {
            if (0x241..=0x25F).contains(&n) {
                return format!("{:03X}", n + 0x400);
            }
            if n <= 0x7FF {
                return format!("{:03X}", n + 8);
            }
        }
    }
    if h.len() == 8 && &h[2..4] == "DA" && h.chars().all(|c| c.is_ascii_hexdigit()) {
        return format!("{}DA{}{}", &h[0..2], &h[6..8], &h[4..6]);
    }
    h
}
