//! Tier S — UDS periodic streaming, pure functions (S0).
//!
//! The nGauge method (docs/research/nGauge_S197_Protocol.md): `10 03` →
//! `2C 03` → N × `2C 01 F2 0x <srcDID> <pos> <len>` → `2A 03 00…` → consume
//! UUDT frames on the profile's response ids at ~25 Hz.
//!
//! Everything platform-specific is DATA (`udsprofiles.json`, keyed by the
//! vinrules capability tag); the UDS mechanics here are ISO 14229 standard:
//! `F2xx` periodic-DID range, 7 data bytes per slot (8-byte frame minus the
//! periodic-ID byte), `2A` transmission modes slow/medium/fast = 01/02/03.
//!
//! This module is PURE — no wire, no state. The stream plugin (S1+) drives
//! it; the tests replay the captured S197 byte streams.

use std::collections::HashMap;

/// Data bytes available per periodic slot (8-byte frame minus the DID byte).
pub const SLOT_DATA_BYTES: usize = 7;

/// One platform's streaming parameters (a `udsprofiles.json` entry).
#[derive(Debug, Clone, PartialEq)]
pub struct StreamProfile {
    pub name: String,
    /// Request target ECU (e.g. "7E0").
    pub target: String,
    /// UUDT ids the periodic frames arrive on (e.g. ["6A0", "6A1"]).
    pub response_ids: Vec<String>,
    /// Periodic DIDs available (F200..F200+slot_count-1).
    pub slot_count: usize,
    /// `2A` transmission mode byte: slow=01, medium=02, fast=03.
    pub rate_mode: u8,
    pub keepalive_interval_ms: u64,
    /// Post-keepalive settle before in-gap work / re-arm (`keepaliveSettleMs`,
    /// default 200 — the bench-era constant, now tunable per profile: at 30
    /// gauges the beat's feed hole is the visible "all gauges freeze" stall).
    pub keepalive_settle_ms: u64,
}

/// Parse `udsprofiles.json`: `{ "<tag>": { name, target, responseIds,
/// slotCount, periodicRate, keepaliveIntervalMs } }`. A malformed ENTRY is
/// dropped (that tag degrades to untagged → poll) — same degrade-safe
/// posture as the VIN rules; a malformed FILE yields an empty map.
pub fn parse_stream_profiles(json: &str) -> HashMap<String, StreamProfile> {
    let mut out = HashMap::new();
    let Ok(root) = serde_json::from_str::<serde_json::Value>(json) else {
        return out;
    };
    let Some(obj) = root.as_object() else {
        return out;
    };
    for (tag, v) in obj {
        if tag.starts_with('_') {
            continue; // _comment etc.
        }
        let (Some(name), Some(target)) = (
            v.get("name").and_then(|x| x.as_str()),
            v.get("target").and_then(|x| x.as_str()),
        ) else {
            continue;
        };
        let Some(ids) = v.get("responseIds").and_then(|x| x.as_array()) else {
            continue;
        };
        let response_ids: Vec<String> = ids
            .iter()
            .filter_map(|x| x.as_str())
            .map(|s| s.trim().to_uppercase())
            .filter(|s| !s.is_empty())
            .collect();
        let Some(slot_count) = v.get("slotCount").and_then(|x| x.as_u64()) else {
            continue;
        };
        // 256 is the PROTOCOL bound (periodic DIDs span F200-F2FF), not a
        // policy: how many slots a platform gets is the FILE's decision.
        if response_ids.is_empty() || slot_count == 0 || slot_count > 256 {
            continue;
        }
        let rate_mode = match v.get("periodicRate").and_then(|x| x.as_str()) {
            Some("slow") => 0x01,
            Some("medium") => 0x02,
            Some("fast") | None => 0x03,
            Some(_) => continue, // unknown rate word = malformed entry
        };
        let keepalive_settle_ms = v
            .get("keepaliveSettleMs")
            .and_then(|x| x.as_u64())
            .unwrap_or(200);
        let keepalive_interval_ms = v
            .get("keepaliveIntervalMs")
            .and_then(|x| x.as_u64())
            .unwrap_or(4000);
        out.insert(
            tag.clone(),
            StreamProfile {
                name: name.to_string(),
                target: target.to_uppercase(),
                response_ids,
                slot_count: slot_count as usize,
                rate_mode,
                keepalive_interval_ms,
                keepalive_settle_ms,
            },
        );
    }
    out
}

/// Map a subscription pid to its UDS source DID.
/// Custom Mode 22 pids ARE DIDs (`221E1A` → `1E1A`); standard Mode 01 pids
/// ride the `F4xx` mirror convention (`010C` → `F40C` — Ford implements it).
/// Anything else can't stream.
pub fn source_did(pid: &str) -> Option<String> {
    let p = pid.trim().to_uppercase();
    if p.len() == 6 && p.starts_with("22") && p[2..].chars().all(|c| c.is_ascii_hexdigit()) {
        return Some(p[2..].to_string());
    }
    if p.len() == 4 && p.starts_with("01") && p[2..].chars().all(|c| c.is_ascii_hexdigit()) {
        return Some(format!("F4{}", &p[2..]));
    }
    None
}

/// One packed signal inside a periodic DID.
#[derive(Debug, Clone, PartialEq)]
pub struct PackedSignal {
    /// Subscription pid this slice belongs to (completion key).
    pub pid: String,
    /// 0-based byte offset within the slot's 7 data bytes.
    pub offset: usize,
    pub len: usize,
}

/// Periodic-DID low byte (0x00..slot_count) → packed signals in order.
/// Bytes past the last packed signal (the rolling counters on the wire) are
/// simply never sliced — dropped by construction.
pub type UnpackMap = HashMap<u8, Vec<PackedSignal>>;

/// The `2C 01` define set for a subscription.
#[derive(Debug, Clone, PartialEq)]
pub struct DefineSet {
    /// Wire commands, in send order: one `2C01F20x<src><pos><len>` per signal.
    pub define_commands: Vec<String>,
    /// (slot, pid) for each define command, same order — the driver uses it
    /// to DROP a signal whose define the vehicle refuses (Mustang: F40C is a
    /// definable source, F40D gets `7F 2C 31`) and stream the survivors.
    pub record_pids: Vec<(u8, String)>,
    /// The `2A <mode> 00 01 …` start command for the used slots.
    pub start_command: String,
    pub unpack: UnpackMap,
    pub slots_used: usize,
    /// Signals that didn't fit slot_count (one signal per slot) — they stay
    /// on the poll side; a streamed subset beats failing the whole set.
    pub dropped: Vec<String>,
}

/// Why a set can't stream (v1 all-or-nothing → the whole set demotes to poll).
#[derive(Debug, Clone, PartialEq)]
pub enum BuildDefinesError {
    /// A subscribed signal has no source DID (not Mode 22 / Mode 01).
    Unmappable(String),
    /// A signal has no confirmed length (observe-then-batch applies here too).
    UnknownLength(String),
    /// A single signal wider than one slot's 7 data bytes.
    TooWide(String),
    /// Total bytes exceed slot_count × 7.
    CapacityExceeded {
        needed_slots: usize,
        slot_count: usize,
    },
    /// The pids span more than one target ECU (one session = one target).
    MixedTarget,
}

/// Pack `(pid, confirmed_len, target)` signals into periodic DIDs, first-fit
/// in order, ≤7 data bytes per slot, ≤ profile.slot_count slots. The `2C 01`
/// position byte is always 01 — position-in-SOURCE-record, whole source (see
/// the block comment inside; the dynamic DID concatenates in define order).
pub fn build_defines(
    signals: &[(String, u8, Option<String>)],
    profile: &StreamProfile,
) -> Result<DefineSet, BuildDefinesError> {
    // One target per periodic session: every explicit target must agree
    // (None rides the profile target).
    let mut seen_target: Option<String> = None;
    for (pid, _, target) in signals {
        if let Some(t) = target {
            let t = t.to_uppercase();
            match &seen_target {
                None => seen_target = Some(t),
                Some(prev) if *prev == t => {}
                Some(_) => return Err(BuildDefinesError::MixedTarget),
            }
            let _ = pid;
        }
    }
    if let Some(t) = &seen_target {
        if *t != profile.target {
            return Err(BuildDefinesError::MixedTarget);
        }
    }

    // MULTI-RECORD FIRST-FIT, position byte ALWAYS 01. Per ISO 14229 a
    // `2C 01` record is `<srcDID> <positionInSOURCErecord> <memorySize>` —
    // the position indexes the SOURCE DID's data (1 = from its first byte),
    // NOT the dynamic DID; the dynamic DID is the CONCATENATION of records
    // in define order. The Mustang bench refusals (`7F 2C 31` on every
    // append, round 3 2026-07-31) were OUR bug, not a Ford limit: v1 sent
    // cumulative dynamic-DID positions, so every append asked for bytes
    // past its source's end (`F40C` pos2 len2 on a 2-byte source) —
    // requestOutOfRange, exactly as specified. Position 1 + full source
    // length is always in range. Signals beyond slot_count×7 bytes overflow
    // into `dropped` (a streamed subset beats failing the whole set); they
    // stay on the paused poll side until S3.
    let mut define_commands = Vec::new();
    let mut record_pids: Vec<(u8, String)> = Vec::new();
    let mut unpack: UnpackMap = HashMap::new();
    let mut dropped: Vec<String> = Vec::new();
    let mut fill = vec![0usize; profile.slot_count];

    for (pid, len, _) in signals {
        let len = *len as usize;
        // Per-signal problems DROP the signal, never the set (a streamed
        // subset beats no stream): unknown length (not observed yet — e.g.
        // a gauge added moments before a re-define), unmappable (Mode 09
        // etc.), wider than a slot, or capacity overflow. Dropped signals
        // stay on the poll side.
        if len == 0 || len > SLOT_DATA_BYTES || source_did(pid).is_none() {
            dropped.push(pid.to_uppercase());
            continue;
        }
        let src = source_did(pid).unwrap();
        // First-fit: the lowest slot with room for the whole record.
        let Some(slot) = (0..profile.slot_count).find(|&s| fill[s] + len <= SLOT_DATA_BYTES) else {
            dropped.push(pid.to_uppercase());
            continue;
        };
        // 2C 01 F2 0x <srcDID> 01 <len> — whole source, appended to the
        // slot; its bytes land at the slot's current fill.
        define_commands.push(format!("2C01F2{slot:02X}{src}01{len:02X}"));
        unpack.entry(slot as u8).or_default().push(PackedSignal {
            pid: pid.to_uppercase(),
            offset: fill[slot],
            len,
        });
        record_pids.push((slot as u8, pid.to_uppercase()));
        fill[slot] += len;
    }

    let slots_used = fill.iter().filter(|&&f| f > 0).count();
    let start_command = {
        let mut s = format!("2A{:02X}", profile.rate_mode);
        for i in 0..slots_used {
            s.push_str(&format!("{i:02X}"));
        }
        s
    };
    Ok(DefineSet {
        define_commands,
        record_pids,
        start_command,
        unpack,
        slots_used,
        dropped,
    })
}

/// Remove a REFUSED record's signal from the unpack map and close the gap:
/// the record never joined the wire, so every LATER record in the same slot
/// (concatenation order) shifts left by the refused record's length. Removes
/// the slot entirely when its last signal goes. Returns false if absent.
pub fn drop_refused(unpack: &mut UnpackMap, slot: u8, pid: &str) -> bool {
    let Some(signals) = unpack.get_mut(&slot) else {
        return false;
    };
    let Some(i) = signals.iter().position(|s| s.pid == pid) else {
        return false;
    };
    let removed = signals.remove(i);
    for s in signals.iter_mut() {
        if s.offset > removed.offset {
            s.offset -= removed.len;
        }
    }
    if signals.is_empty() {
        unpack.remove(&slot);
    }
    true
}

/// Decode one monitor line (`6A0 00 00 80 01 00 0B 38 4F`) into per-pid data
/// bytes via the unpack map. Returns empty for non-periodic lines (other CAN
/// ids, noise, `STOPPED`). Trailing bytes past the packed signals — the
/// per-DID rolling counters seen on DIDs 03/04/06 — are never sliced.
/// A frame shorter than a signal's slice yields nothing for that signal
/// (torn line) but still decodes earlier complete signals.
pub fn decode_periodic_line(
    line: &str,
    response_ids: &[String],
    unpack: &UnpackMap,
) -> Vec<(String, Vec<u8>)> {
    let tokens: Vec<&str> = line.split_whitespace().collect();
    if tokens.len() < 2 {
        return Vec::new();
    }
    let bytes: Vec<u8> = tokens[1..]
        .iter()
        .filter_map(|t| u8::from_str_radix(t, 16).ok())
        .collect();
    if bytes.len() + 1 != tokens.len() {
        return Vec::new(); // a non-hex slot byte made the old decoder bail
    }
    decode_periodic_frame(&tokens[0].to_uppercase(), &bytes, response_ids, unpack)
}

/// Frame-native decode (LH3): `id` is the normalized CAN id, `data` the
/// payload after the header — slot/DID byte first, packed signals after.
pub fn decode_periodic_frame(
    id: &str,
    data: &[u8],
    response_ids: &[String],
    unpack: &UnpackMap,
) -> Vec<(String, Vec<u8>)> {
    if !response_ids.iter().any(|r| r == id) {
        return Vec::new();
    }
    let Some((&did, data)) = data.split_first() else {
        return Vec::new();
    };
    let Some(signals) = unpack.get(&did) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for sig in signals {
        if sig.offset + sig.len <= data.len() {
            out.push((
                sig.pid.clone(),
                data[sig.offset..sig.offset + sig.len].to_vec(),
            ));
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s197_profile() -> StreamProfile {
        StreamProfile {
            name: "Ford S197 Mustang 2011–2014".to_string(),
            target: "7E0".to_string(),
            response_ids: vec!["6A0".to_string(), "6A1".to_string()],
            slot_count: 10,
            rate_mode: 0x03,
            keepalive_interval_ms: 4000,
            keepalive_settle_ms: 200,
        }
    }

    // ---- profile parsing ----

    #[test]
    fn parse_profiles_happy_path() {
        let json = r#"{
          "_comment": "ignored",
          "s197": {
            "name": "Ford S197 Mustang 2011–2014",
            "target": "7e0",
            "responseIds": ["6a0", "6A1"],
            "slotCount": 10,
            "periodicRate": "fast",
            "keepaliveIntervalMs": 4000
          }
        }"#;
        let profiles = parse_stream_profiles(json);
        assert_eq!(profiles.len(), 1);
        assert_eq!(profiles["s197"], s197_profile());
    }

    /// `keepaliveSettleMs` is optional (default 200 — covered by the happy
    /// path above, whose JSON omits it); an explicit value is honored.
    #[test]
    fn parse_profiles_explicit_keepalive_settle() {
        let json = r#"{
          "mock": { "name": "x", "target": "7E0", "responseIds": ["6A0"],
                    "slotCount": 4, "keepaliveSettleMs": 25 }
        }"#;
        let profiles = parse_stream_profiles(json);
        assert_eq!(profiles["mock"].keepalive_settle_ms, 25);
        assert_eq!(
            profiles["mock"].keepalive_interval_ms, 4000,
            "interval still defaults"
        );
    }

    /// Malformed ENTRIES drop (tag degrades to untagged → poll); the rest of
    /// the file still parses. A malformed FILE yields an empty map.
    #[test]
    fn parse_profiles_malformed_degrades_to_untagged() {
        let json = r#"{
          "no_ids":   { "name": "x", "target": "7E0", "slotCount": 10 },
          "no_slots": { "name": "x", "target": "7E0", "responseIds": ["6A0"] },
          "bad_rate": { "name": "x", "target": "7E0", "responseIds": ["6A0"], "slotCount": 10, "periodicRate": "ludicrous" },
          "zero":     { "name": "x", "target": "7E0", "responseIds": ["6A0"], "slotCount": 0 },
          "ok":       { "name": "x", "target": "7E0", "responseIds": ["6A0"], "slotCount": 4 }
        }"#;
        let profiles = parse_stream_profiles(json);
        assert_eq!(profiles.len(), 1, "only the well-formed entry survives");
        assert!(profiles.contains_key("ok"));
        assert_eq!(profiles["ok"].rate_mode, 0x03, "rate defaults to fast");
        assert!(parse_stream_profiles("not json at all").is_empty());
        assert!(parse_stream_profiles("[1,2,3]").is_empty());
    }

    // ---- source DID mapping ----

    #[test]
    fn source_did_mapping() {
        // Custom Mode 22: the DID is the source.
        assert_eq!(source_did("221E1A").as_deref(), Some("1E1A"));
        assert_eq!(source_did("22f40c").as_deref(), Some("F40C"));
        // Mode 01 rides the F4xx mirror.
        assert_eq!(source_did("010C").as_deref(), Some("F40C"));
        assert_eq!(source_did("0105").as_deref(), Some("F405"));
        // Everything else can't stream.
        assert_eq!(source_did("0902"), None);
        assert_eq!(source_did("03"), None);
        assert_eq!(source_did("010C|010D"), None);
        assert_eq!(source_did("ATSP 0"), None);
    }

    // ---- build_defines ----

    /// The captured F200 shape: 0700 (4B) + F40C RPM (2B) + F40F ACT (1B)
    /// pack into ONE slot exactly like the nGauge frame (`6A0 00 00 80 01 00
    /// 0B 38 4F`). Every define carries position byte 01 — position-in-
    /// SOURCE-record per ISO 14229 (the cumulative-position appends v1 sent
    /// were source overruns, the real cause of the bench `7F 2C 31`s).
    #[test]
    fn defines_pack_one_slot_like_the_capture() {
        let signals = vec![
            ("220700".to_string(), 4u8, None),
            ("22F40C".to_string(), 2u8, None),
            ("22F40F".to_string(), 1u8, None),
        ];
        let set = build_defines(&signals, &s197_profile()).unwrap();
        assert_eq!(
            set.define_commands,
            vec!["2C01F20007000104", "2C01F200F40C0102", "2C01F200F40F0101"]
        );
        assert_eq!(set.slots_used, 1, "4+2+1 = 7 bytes, one full slot");
        assert_eq!(set.start_command, "2A0300");
        assert!(set.dropped.is_empty());
        // Offsets are concatenation order — RPM at bytes 5-6 of the frame,
        // matching the capture (0B 38 → 718 rpm).
        assert_eq!(
            set.unpack[&0][0],
            PackedSignal {
                pid: "220700".into(),
                offset: 0,
                len: 4
            }
        );
        assert_eq!(
            set.unpack[&0][1],
            PackedSignal {
                pid: "22F40C".into(),
                offset: 4,
                len: 2
            }
        );
        assert_eq!(
            set.unpack[&0][2],
            PackedSignal {
                pid: "22F40F".into(),
                offset: 6,
                len: 1
            }
        );
    }

    /// First-fit overflows into the next slot at the 7-byte boundary; the
    /// start command lists every used slot.
    #[test]
    fn defines_overflow_and_mixed_lengths() {
        let signals = vec![
            ("22AAAA".to_string(), 4u8, None),
            ("22BBBB".to_string(), 2u8, None),
            ("22CCCC".to_string(), 2u8, None), // 4+2+2 > 7 → slot 1
            ("010C".to_string(), 2u8, None),   // F40C mirror, fits slot 1
        ];
        let set = build_defines(&signals, &s197_profile()).unwrap();
        assert_eq!(set.slots_used, 2);
        assert_eq!(set.start_command, "2A030001");
        assert_eq!(set.unpack[&0].len(), 2);
        assert_eq!(
            set.unpack[&1][0],
            PackedSignal {
                pid: "22CCCC".into(),
                offset: 0,
                len: 2
            }
        );
        assert_eq!(
            set.unpack[&1][1],
            PackedSignal {
                pid: "010C".into(),
                offset: 2,
                len: 2
            }
        );
        assert_eq!(set.define_commands[3], "2C01F201F40C0102");
    }

    /// A refused record drops out and later records in the slot shift left —
    /// they concatenate on the wire without the refused source's bytes.
    #[test]
    fn drop_refused_shifts_later_offsets() {
        let signals = vec![
            ("220700".to_string(), 4u8, None),
            ("22F40C".to_string(), 2u8, None),
            ("22F40F".to_string(), 1u8, None),
        ];
        let mut unpack = build_defines(&signals, &s197_profile()).unwrap().unpack;
        assert!(drop_refused(&mut unpack, 0, "220700"));
        assert_eq!(
            unpack[&0][0],
            PackedSignal {
                pid: "22F40C".into(),
                offset: 0,
                len: 2
            }
        );
        assert_eq!(
            unpack[&0][1],
            PackedSignal {
                pid: "22F40F".into(),
                offset: 2,
                len: 1
            }
        );
        // Unknown pid: no-op. Last signal out removes the slot.
        assert!(!drop_refused(&mut unpack, 0, "22ZZZZ"));
        assert!(drop_refused(&mut unpack, 0, "22F40C"));
        assert!(drop_refused(&mut unpack, 0, "22F40F"));
        assert!(unpack.is_empty());
    }

    #[test]
    fn defines_demote_reasons() {
        let p = s197_profile();
        // Per-signal problems DROP the signal, never the set: unmappable
        // (Mode 09), unknown length (not yet observed), wider than a slot.
        for bad in [("0902", 2u8), ("22F40C", 0), ("221111", 8)] {
            let set = build_defines(&[(bad.0.into(), bad.1, None)], &p).unwrap();
            assert_eq!(set.dropped, vec![bad.0.to_uppercase()], "{bad:?}");
            assert!(set.define_commands.is_empty());
        }
        // Capacity: overflow signals are DROPPED (streamed subset beats
        // failing the whole set), not an error.
        let small = StreamProfile {
            slot_count: 2,
            ..p.clone()
        };
        let signals: Vec<_> = (0..3)
            .map(|i| (format!("22AA{i:02X}"), 7u8, None))
            .collect();
        let set = build_defines(&signals, &small).unwrap();
        assert_eq!(set.slots_used, 2);
        assert_eq!(set.dropped, vec!["22AA02".to_string()]);
        // Mixed targets → demote (one periodic session = one target ECU).
        let e = build_defines(
            &[
                ("22AAAA".into(), 2, Some("7E0".into())),
                ("22BBBB".into(), 2, Some("7E9".into())),
            ],
            &p,
        )
        .unwrap_err();
        assert_eq!(e, BuildDefinesError::MixedTarget);
        // A target that isn't the profile's target also demotes.
        let e = build_defines(&[("22AAAA".into(), 2, Some("726".into()))], &p).unwrap_err();
        assert_eq!(e, BuildDefinesError::MixedTarget);
    }

    // ---- decode: the captured rev sweep (Source Data D) ----

    /// Build an unpack map matching the CAPTURED S197 packing for the DIDs
    /// the sweep exercises (F200 RPM, F201/6A1-01 AFR, 6A1-03 MAF Hz,
    /// F204 MAF), then replay capture lines VERBATIM and assert the decoded
    /// values reproduce the documented numbers.
    fn capture_unpack() -> UnpackMap {
        let mut m: UnpackMap = HashMap::new();
        // 6A0 00: 0700 (4B: 00 80 01 00) · RPM word · ACT byte — 4+2+1 = 7.
        m.insert(
            0,
            vec![
                PackedSignal {
                    pid: "220700".into(),
                    offset: 0,
                    len: 4,
                },
                PackedSignal {
                    pid: "22F40C".into(),
                    offset: 4,
                    len: 2,
                },
                PackedSignal {
                    pid: "22F40F".into(),
                    offset: 6,
                    len: 1,
                },
            ],
        );
        // 6A1 01: AFR1 word · AFR2 word · F434ish word (counterless)
        m.insert(
            1,
            vec![
                PackedSignal {
                    pid: "22F434".into(),
                    offset: 0,
                    len: 2,
                },
                PackedSignal {
                    pid: "22F435".into(),
                    offset: 2,
                    len: 2,
                },
            ],
        );
        // 6A1 03: MAF-Hz word · word · "00 24" — byte 7 is a rolling counter
        // and is NOT in the map (dropped by construction).
        m.insert(
            3,
            vec![PackedSignal {
                pid: "22116B".into(),
                offset: 0,
                len: 2,
            }],
        );
        // 6A0 04: speed word · pedal word · MAF word · counter (dropped)
        m.insert(
            4,
            vec![
                PackedSignal {
                    pid: "221505".into(),
                    offset: 0,
                    len: 2,
                },
                PackedSignal {
                    pid: "220914".into(),
                    offset: 2,
                    len: 2,
                },
                PackedSignal {
                    pid: "22F408".into(),
                    offset: 4,
                    len: 2,
                },
            ],
        );
        m
    }

    fn rpm_of(decoded: &[(String, Vec<u8>)]) -> Option<u32> {
        decoded
            .iter()
            .find(|(p, _)| p == "22F40C")
            .map(|(_, b)| ((b[0] as u32) << 8 | b[1] as u32) / 4)
    }

    #[test]
    fn decode_reproduces_captured_rev_sweep() {
        let ids = s197_profile().response_ids;
        let unpack = capture_unpack();

        // Idle line from Source Data D: RPM 0B4A/4 = 722.
        let idle = decode_periodic_line("6A0 00 00 80 01 00 0B 4A 4F", &ids, &unpack);
        assert_eq!(rpm_of(&idle), Some(722));
        assert!(idle
            .iter()
            .any(|(p, b)| p == "220700" && b == &vec![0x00, 0x80, 0x01, 0x00]));
        assert!(idle.iter().any(|(p, b)| p == "22F40F" && b == &vec![0x4F]));

        // Sweep endpoints: 0B07 → 705 rpm, 0BA7 → 745 rpm (the doc's
        // "707–745" rounded loosely; 0x0B07/4 = 705.75 — bytes don't lie).
        let low = decode_periodic_line("6A0 00 00 80 01 00 0B 07 4F", &ids, &unpack);
        let high = decode_periodic_line("6A0 00 00 80 01 00 0B A7 4F", &ids, &unpack);
        assert_eq!(rpm_of(&low), Some(705));
        assert_eq!(rpm_of(&high), Some(745));

        // AFR moving under throttle: 7CC3 → 8386.
        let afr1 = decode_periodic_line("6A1 01 7C C3 7F E0 16 45 00", &ids, &unpack);
        let afr2 = decode_periodic_line("6A1 01 83 86 7F E0 16 45 00", &ids, &unpack);
        let afr_word = |d: &[(String, Vec<u8>)]| {
            d.iter()
                .find(|(p, _)| p == "22F434")
                .map(|(_, b)| (b[0] as u32) << 8 | b[1] as u32)
        };
        assert_eq!(afr_word(&afr1), Some(0x7CC3));
        assert_eq!(afr_word(&afr2), Some(0x8386));

        // MAF rising: 1D50 → 1E84 (6A0 04); byte-7 counter NOT decoded.
        let maf1 = decode_periodic_line("6A0 04 00 00 03 05 1D 50 83", &ids, &unpack);
        let maf2 = decode_periodic_line("6A0 04 00 00 03 05 1E 84 84", &ids, &unpack);
        let maf_word = |d: &[(String, Vec<u8>)]| {
            d.iter()
                .find(|(p, _)| p == "22F408")
                .map(|(_, b)| (b[0] as u32) << 8 | b[1] as u32)
        };
        assert_eq!(maf_word(&maf1), Some(0x1D50));
        assert_eq!(maf_word(&maf2), Some(0x1E84));
        assert_eq!(maf1.len(), 3, "counter byte is dropped, not a signal");

        // MAF-Hz on the ODD id (6A1 03): 064E → 0993 across the sweep.
        let hz1 = decode_periodic_line("6A1 03 06 4E 02 0D 00 24 83", &ids, &unpack);
        let hz2 = decode_periodic_line("6A1 03 09 93 02 0D 00 24 84", &ids, &unpack);
        let hz_word = |d: &[(String, Vec<u8>)]| {
            d.iter()
                .find(|(p, _)| p == "22116B")
                .map(|(_, b)| (b[0] as u32) << 8 | b[1] as u32)
        };
        assert_eq!(hz_word(&hz1), Some(0x064E));
        assert_eq!(hz_word(&hz2), Some(0x0993));
    }

    #[test]
    fn decode_ignores_non_periodic_lines() {
        let ids = s197_profile().response_ids;
        let unpack = capture_unpack();
        // Other CAN ids (the passive broadcast), noise, prompts: nothing.
        assert!(decode_periodic_line("201 0B 38 00 00", &ids, &unpack).is_empty());
        assert!(decode_periodic_line("7E8 06 41 0C 3F E3 0D 73", &ids, &unpack).is_empty());
        assert!(decode_periodic_line("STOPPED", &ids, &unpack).is_empty());
        assert!(decode_periodic_line("", &ids, &unpack).is_empty());
        // Unmapped DID on a periodic id: nothing (defines don't cover it).
        assert!(decode_periodic_line("6A0 08 01 13 00 00 0F F0 00", &ids, &unpack).is_empty());
        // Torn line: earlier signals still decode, the sliced-off one doesn't.
        let torn = decode_periodic_line("6A0 04 00 00 03 05 1D", &ids, &unpack);
        assert_eq!(torn.len(), 2, "speed+pedal decode, truncated MAF doesn't");
    }

    // ---- probe acks (LH7: typed — the ack check is "a payload opens with
    // the positive SID", over the handler's framing) ----

    #[test]
    fn probe_ack_parsing() {
        use crate::addressing::{Addressing, Bus};
        let opens = |cmd: &str, text: &str, prefix: &[u8]| {
            crate::response_parser::text_payloads(cmd, text, Bus::new(Addressing::Can11))
                .iter()
                .any(|p| p.bytes.starts_with(prefix))
        };
        // With and without header/PCI; the S3 parameter tail is fine.
        assert!(opens("1003", "50 03 00 32 01 F4", &[0x50, 0x03]));
        assert!(opens("1003", "7E8 06 50 03 00 32 01 F4", &[0x50, 0x03]));
        assert!(opens("2C03", "6C 03", &[0x6C, 0x03]));
        assert!(opens("2C03", "7E8 02 6C 03", &[0x6C, 0x03]));
        // NRC / noise / timeout-ish → not ok.
        assert!(!opens("1003", "7F 10 11", &[0x50, 0x03]));
        assert!(!opens("1003", "7E8 03 7F 10 11", &[0x50, 0x03]));
        assert!(opens("1003", "7E8 03 7F 10 11", &[0x7F]));
        assert!(!opens("2C03", "7F 2C 31", &[0x6C, 0x03]));
        assert!(!opens("1003", "NO DATA", &[0x50, 0x03]));
        assert!(!opens("1003", "", &[0x50, 0x03]));
    }
}
