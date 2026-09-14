//! Per-VIN learned PID byte lengths (High_Rate_Data_Plan, common-ground #1).
//!
//! Lengths are LEARNED, not authored: every plain single-PID response is
//! self-delimiting, and its observed data-byte count is recorded here. Batch
//! admission (B1) and `F2xx` packing (Tier S) require a confirmed length; a
//! PID without one rides solo — which is also how a stale entry heals after
//! an echo-validation failure invalidates it (demote → solo → re-observe).
//!
//! Semantics:
//! - `record` is last-write-wins ("re-observe replaces a stale length") —
//!   write-once would trap a stale value forever after a reflash.
//! - Observation reads the FIRST controller line only: a PID's data length
//!   is spec-fixed across ECUs, and any oddball disagreement is caught by
//!   echo validation at slice time.
//! - Persisted as `{cache_path}/{VIN}.lengths` (JSON pid → len) beside the
//!   existing `.vehicle_info` / `.cache` files; per-VIN because Mode 22 DID
//!   lengths vary per calibration.

use std::collections::HashMap;
use std::path::PathBuf;

#[derive(Default)]
pub struct LengthCache {
    lengths: HashMap<String, u8>,
    /// Persistence target — set once the VIN is known (`load_for_vin`).
    file: Option<PathBuf>,
}

impl LengthCache {
    pub fn new() -> Self {
        Self::default()
    }

    /// Bind to `{cache_path}/{vin}.lengths` and load any persisted entries.
    /// Clears previous state first — a new vehicle must not inherit the last
    /// car's lengths.
    pub fn load_for_vin(&mut self, cache_path: &str, vin: &str) {
        self.lengths.clear();
        let file = PathBuf::from(cache_path).join(format!("{}.lengths", vin));
        if let Ok(data) = std::fs::read_to_string(&file) {
            if let Ok(map) = serde_json::from_str::<HashMap<String, u8>>(&data) {
                self.lengths = map;
            }
        }
        self.file = Some(file);
    }

    /// Confirmed data-byte length for a PID, if observed on this vehicle.
    pub fn get(&self, pid: &str) -> Option<u8> {
        self.lengths.get(&pid.to_uppercase()).copied()
    }

    /// SP-LEN: seed a DERIVED length (from the pid's equation byte usage) —
    /// only fills a hole; a learned observation always wins and overwrites.
    pub fn seed(&mut self, pid: &str, len: u8) {
        let key = pid.to_uppercase();
        if self.lengths.contains_key(&key) {
            return;
        }
        self.lengths.insert(key, len);
        self.persist();
    }

    /// Record an observation (last-write-wins) and persist on change.
    pub fn record(&mut self, pid: &str, len: u8) {
        let key = pid.to_uppercase();
        if self.lengths.get(&key) == Some(&len) {
            return; // unchanged — skip the disk write
        }
        self.lengths.insert(key, len);
        self.persist();
    }

    /// Drop a length that failed echo validation — the owner rides solo until
    /// re-observed (demotion IS this invalidation; there is no flag).
    pub fn invalidate(&mut self, pid: &str) {
        if self.lengths.remove(&pid.to_uppercase()).is_some() {
            self.persist();
        }
    }

    fn persist(&self) {
        if let Some(ref file) = self.file {
            if let Ok(json) = serde_json::to_string(&self.lengths) {
                let _ = std::fs::write(file, json);
            }
        }
    }
}

/// LH7: observed data-byte length of a plain single-PID member from its
/// typed payloads — the first controller whose payload opens with the pid's
/// echo (`41 0C` / `62 F4 0C`): payload length minus the echo. Scope: modes
/// 01 and 22 (the batching/streaming-relevant modes). None for anything
/// unlearnable — NO DATA (no payloads), a negative response (`7F …`), an
/// echo mismatch, other modes, a suffixed/piped pid — nothing gets recorded.
/// An assembled multi-frame payload carries its full length; a lone first
/// frame (no CFs framed) teaches nothing.
pub fn observed_len(pid: &str, payloads: &[crate::link::Payload]) -> Option<u8> {
    let pid = pid.to_uppercase();
    let hex = |r: std::ops::Range<usize>| u8::from_str_radix(&pid[r], 16).ok();
    let echo: Vec<u8> = match (pid.get(0..2), pid.len()) {
        (Some("01"), 4) => vec![0x41, hex(2..4)?],
        (Some("22"), 6) => vec![0x62, hex(2..4)?, hex(4..6)?],
        _ => return None,
    };
    let p = payloads.iter().find(|p| p.bytes.starts_with(&echo))?;
    let data_len = p.bytes.len() - echo.len();
    if data_len == 0 || data_len > 255 {
        return None;
    }
    Some(data_len as u8)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The pre-LH7 text entry, for the bench fixtures below: frame the text
    /// in the addressing its first line sniffs, then learn.
    fn observed_data_len(pid: &str, response: &str) -> Option<u8> {
        use crate::addressing::{Addressing, Bus};
        let sniffed = response
            .replace('\r', "\n")
            .lines()
            .map(str::trim)
            .find(|l| !l.is_empty())
            .and_then(|line| Addressing::sniff(&line.split_whitespace().collect::<Vec<_>>()))
            .unwrap_or(Addressing::Can11);
        observed_len(
            pid,
            &crate::response_parser::text_payloads(pid, response, Bus::new(sniffed)),
        )
    }

    // ---- observed_data_len (fixtures from the 2026-07-30 CX/ECUsim bench) ----

    #[test]
    fn mode01_single_frame() {
        assert_eq!(observed_data_len("010C", "7E8 04 41 0C 3F F3"), Some(2));
        assert_eq!(observed_data_len("010D", "7E8 03 41 0D 73"), Some(1));
        assert_eq!(
            observed_data_len("0100", "7E8 06 41 00 BE 1B 30 13"),
            Some(4)
        );
    }

    #[test]
    fn mode22_single_frame() {
        assert_eq!(
            observed_data_len("22F40C", "7E8 05 62 F4 0C 0B 62"),
            Some(2)
        );
        assert_eq!(
            observed_data_len("221E35", "7E8 05 62 1E 35 0F F0"),
            Some(2)
        );
        assert_eq!(observed_data_len("22F40F", "7E8 04 62 F4 0F 4A"), Some(1));
    }

    #[test]
    fn first_controller_line_wins() {
        // Multi-ECU response: length read from the FIRST line only.
        assert_eq!(
            observed_data_len("0105", "7E8 03 41 05 4A \r7E9 03 41 05 4A"),
            Some(1)
        );
    }

    #[test]
    fn can29_header() {
        assert_eq!(
            observed_data_len("0100", "18 DA F1 10 06 41 00 BF FE B9 93"),
            Some(4)
        );
    }

    #[test]
    fn multi_frame_length_from_assembled_payload() {
        // LH7: the assembled reply teaches (total 0x0B = 11 payload bytes,
        // minus the 2-byte echo = 9); a lone first frame with no CFs framed
        // teaches nothing (its declared total is not a payload).
        assert_eq!(
            observed_data_len(
                "0178",
                "7E8 10 0B 41 78 0D 0A ED 09\r7E8 21 1A F8 00 00 00 00 00"
            ),
            Some(9)
        );
        assert_eq!(
            observed_data_len("22DE00", "7E8 10 83 62 DE 00 31 5A 56"),
            None
        );
    }

    #[test]
    fn rejects_everything_unlearnable() {
        // NO DATA / negative response / echo mismatch / other modes.
        assert_eq!(observed_data_len("010C", "NO DATA"), None);
        assert_eq!(observed_data_len("22F452", "7E8 03 7F 22 31"), None);
        assert_eq!(observed_data_len("010C", "7E8 03 41 0D 73"), None); // wrong pid's line
        assert_eq!(
            observed_data_len("0902", "7E8 10 14 49 02 01 31 47 31"),
            None
        ); // mode 09 out of scope
        assert_eq!(observed_data_len("010C 1", "7E8 04 41 0C 3F F3"), None); // suffixed ≠ plain
        assert_eq!(observed_data_len("010C|010D", "x|y"), None); // piped ≠ plain
    }

    // ---- LengthCache semantics ----

    #[test]
    fn record_get_overwrite() {
        let mut cache = LengthCache::new();
        assert_eq!(cache.get("22F40C"), None);
        cache.record("22F40C", 2);
        assert_eq!(cache.get("22f40c"), Some(2)); // case-insensitive
                                                  // Last-write-wins: the reflash scenario ("re-observe replaces").
        cache.record("22F40C", 3);
        assert_eq!(cache.get("22F40C"), Some(3));
    }

    #[test]
    fn invalidate_drops_entry() {
        let mut cache = LengthCache::new();
        cache.record("010C", 2);
        cache.invalidate("010C");
        assert_eq!(cache.get("010C"), None);
    }

    #[test]
    fn per_vin_roundtrip_and_isolation() {
        let dir = std::env::temp_dir().join(format!("lc_test_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let dir_str = dir.to_str().unwrap();

        // VIN A learns a length; it persists.
        let mut cache = LengthCache::new();
        cache.load_for_vin(dir_str, "VIN_A");
        cache.record("22F40C", 2);

        // Fresh instance for VIN A sees it (roundtrip).
        let mut cache_a = LengthCache::new();
        cache_a.load_for_vin(dir_str, "VIN_A");
        assert_eq!(cache_a.get("22F40C"), Some(2));

        // VIN B is isolated — and switching to it clears in-memory state.
        cache_a.load_for_vin(dir_str, "VIN_B");
        assert_eq!(cache_a.get("22F40C"), None);

        let _ = std::fs::remove_dir_all(&dir);
    }
}
