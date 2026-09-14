//! SP — STN adapter-periodic acquisition (STPPMA), pure builder (SP1).
//!
//! Bench-proven 2026-08-01: the OBDLink STN firmware fires a request every
//! N ms on its own (`STPPMA period,header,data`) and the host just monitors
//! the responses — no round-trip per sample. This module turns the active,
//! chunk-grouped, tier-sorted acquisition into the STPPMA command set:
//! - each message's `data` is a J1979/Mode-22 CHUNK (the bare OBD payload,
//!   NO ISO-TP PCI byte — the adapter frames it; we double-PCI'd once and the
//!   ECU ignored it);
//! - a signal's TIER sets the PERIOD (priority = rate);
//! - PHYSICAL header, not functional, to avoid the multi-ECU flood. The
//!   caller renders headers for the active protocol via `Bus` (SP29):
//!   `7E0` on 11-bit, `18DA10F1` on 29-bit.
//!
//! Reuses the poll plugin's chunk grouping (`Admission`) verbatim: this is
//! "emit the already-computed chunked+tiered acquisition as STPPMA."

use crate::acquisition::{join_chunk_wire, Admission, DuePick};
use crate::subscription::RefreshTier;
use std::collections::BTreeMap;

/// Tier → STPPMA period (ms), host-configurable (the host's `stnperiodic.json`
/// → obd_set_stn_periods). Priority sets the rate. Ceilings (ECU internal
/// refresh + BLE downlink) make Fast ~40 Hz the useful max — 100/400 Hz just
/// re-reads stale values and saturates BLE.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TierPeriods {
    pub fast_ms: u32,
    pub medium_ms: u32,
    pub slow_ms: u32,
}

impl Default for TierPeriods {
    fn default() -> Self {
        Self {
            fast_ms: 25,
            medium_ms: 100,
            slow_ms: 500,
        }
    }
}

impl TierPeriods {
    pub fn period_for(&self, tier: RefreshTier) -> u32 {
        match tier {
            RefreshTier::Fast => self.fast_ms,
            RefreshTier::Medium => self.medium_ms,
            RefreshTier::Slow => self.slow_ms,
        }
    }
}

/// Default periods (tests / fallback).
pub fn tier_period_ms(tier: RefreshTier) -> u32 {
    TierPeriods::default().period_for(tier)
}

/// One periodic message to install via `STPPMA period,header,data`.
#[derive(Debug, Clone, PartialEq)]
pub struct PeriodicMessage {
    pub period_ms: u32,
    /// Physical request header, protocol-rendered ("7E0" / "18DA10F1").
    pub header: String,
    /// Bare OBD payload — mode prefix once + member suffixes, e.g.
    /// "01050C0D" (RPM+speed+coolant) or "22F405F40C" (two DIDs). No PCI.
    pub data: String,
    /// Subscription pids this message covers (decode routing).
    pub pids: Vec<String>,
}

impl PeriodicMessage {
    /// The wire the console/bridge sends. Handle is captured from the reply.
    pub fn command(&self) -> String {
        format!("STPPMA {}, {}, {}", self.period_ms, self.header, self.data)
    }
}

/// SP-EDIT: one message for an existing tier/header from explicit members
/// (delete-by-handle then re-add the remainder or a merged set).
pub fn message_for(period_ms: u32, header: &str, mut pids: Vec<String>) -> PeriodicMessage {
    if pids.len() > 1 {
        pids.sort();
    }
    PeriodicMessage {
        period_ms,
        header: header.to_string(),
        data: join_chunk_wire(&pids),
        pids,
    }
}

/// Build the STPPMA set from `(pid, controller, tier)` signals under the live
/// chunk policy. Signals group BY TIER (own period each), then chunk within
/// tier via `admission.group`; each chunk → one `PeriodicMessage`. `default_
/// header` is the physical target for target-free (Mode 01) chunks.
pub fn build_periodic_set(
    signals: &[(String, Option<String>, RefreshTier)],
    admission: &Admission,
    default_header: &str,
    periods: &TierPeriods,
    single_frame_reply: bool,
) -> (Vec<PeriodicMessage>, Vec<String>) {
    // Bucket by tier (BTreeMap on the divisor → Fast(1) < Medium(3) < Slow(10)
    // for deterministic order).
    let mut by_tier: BTreeMap<u32, (RefreshTier, Vec<DuePick>)> = BTreeMap::new();
    for (pid, controller, tier) in signals {
        by_tier
            .entry(tier.divisor())
            .or_insert_with(|| (*tier, Vec::new()))
            .1
            .push(DuePick {
                pid: pid.clone(),
                controller: controller.clone(),
                timeout_ms: None,
            });
    }

    let get_len = |pid: &str| -> Option<u8> {
        admission
            .length_cache
            .as_ref()
            .and_then(|c| c.lock().unwrap().get(pid))
    };

    let mut out = Vec::new();
    let mut excluded = Vec::new();
    for (_, (tier, picks)) in by_tier {
        let period = periods.period_for(tier);
        // Partition by (mode, header): a chunk shares one request wire.
        let mut buckets: BTreeMap<(String, String), Vec<(String, u8)>> = BTreeMap::new();
        for p in picks {
            let pid = p.pid.to_uppercase();
            // HARDWARE 2026-08-02 (adapter log 0037): the RESPONSE must fit
            // ONE CAN frame. Monitor mode has no tester flow control — a
            // multi-frame answer is a FirstFrame the ECU spams forever and
            // the splitter can never decode (adding 0104 to 010C0D pushed
            // the reply to 8 bytes and every gauge went dark). A pid with
            // no learned/seeded length can't be budgeted → excluded, and a
            // pid whose response can't fit even alone (Mode 09 strings) →
            // excluded.
            let Some(len) = get_len(&pid) else {
                excluded.push(pid);
                continue;
            };
            // `single_frame_reply` = false on a link whose adapter assembles
            // multi-frame slot replies (DVI): only the request (count cap)
            // bounds a message then.
            if single_frame_reply && 1 + member_cost(&pid, len) > 7 {
                excluded.push(pid);
                continue;
            }
            let mode = pid[..2].to_string();
            let header = p
                .controller
                .clone()
                .unwrap_or_else(|| default_header.to_string());
            buckets.entry((mode, header)).or_default().push((pid, len));
        }
        for ((mode, header), mut members) in buckets {
            members.sort(); // deterministic wire
            let cap = if mode == "22" {
                admission.chunk22_limit.max(1)
            } else {
                admission.chunk_limit.max(1)
            };
            // Greedy pack under the 7-byte response budget + count cap.
            let reply_budget: u8 = if single_frame_reply { 7 - 1 } else { u8::MAX }; // mode echo byte
            let mut current: Vec<String> = Vec::new();
            let mut budget = reply_budget;
            for (pid, len) in members {
                let cost = member_cost(&pid, len);
                if !current.is_empty() && (current.len() >= cap || cost > budget) {
                    out.push(PeriodicMessage {
                        period_ms: period,
                        header: header.clone(),
                        data: join_chunk_wire(&current),
                        pids: current.clone(),
                    });
                    current.clear();
                    budget = reply_budget;
                }
                budget -= cost.min(budget);
                current.push(pid);
            }
            if !current.is_empty() {
                out.push(PeriodicMessage {
                    period_ms: period,
                    header,
                    data: join_chunk_wire(&current),
                    pids: current,
                });
            }
        }
    }
    (out, excluded)
}

/// Response bytes one member costs inside a chunk: its pid-suffix echo
/// (Mode 01 = 1 byte, Mode 22 DID = 2) plus its data length.
pub fn member_cost(pid: &str, len: u8) -> u8 {
    let echo = ((pid.len().saturating_sub(2)) / 2).max(1) as u8;
    echo + len
}

/// SP-EDIT merge feasibility: all members (with known lengths) fit ONE
/// single-frame response under the count cap.
pub fn fits_single_frame(members: &[(String, u8)], count_cap: usize) -> bool {
    if members.is_empty() || members.len() > count_cap.max(1) {
        return false;
    }
    let total: u16 = 1 + members
        .iter()
        .map(|(p, l)| member_cost(p, *l) as u16)
        .sum::<u16>();
    total <= 7
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Admission with a length cache pre-seeded per pid (the budget packer
    /// needs a length for every member — unknown lengths are left out).
    fn admission_with_lens(chunk: usize, chunk22: usize, lens: &[(&str, u8)]) -> Admission {
        let cache = std::sync::Arc::new(std::sync::Mutex::new(
            crate::length_cache::LengthCache::new(),
        ));
        {
            let mut c = cache.lock().unwrap();
            for (pid, len) in lens {
                c.record(pid, *len);
            }
        }
        Admission {
            max_segments: 12,
            chunk_limit: chunk,
            chunk22_limit: chunk22,
            length_cache: Some(cache),
            chunk_excluded: std::collections::HashSet::new(),
        }
    }

    fn admission_with(chunk: usize, chunk22: usize, pids: &[&str]) -> Admission {
        let lens: Vec<(&str, u8)> = pids.iter().map(|p| (*p, 1)).collect();
        admission_with_lens(chunk, chunk22, &lens)
    }

    fn sig(pid: &str, tier: RefreshTier) -> (String, Option<String>, RefreshTier) {
        (pid.to_string(), None, tier)
    }

    #[test]
    fn tiers_map_to_periods() {
        assert_eq!(tier_period_ms(RefreshTier::Fast), 25);
        assert_eq!(tier_period_ms(RefreshTier::Medium), 100);
        assert_eq!(tier_period_ms(RefreshTier::Slow), 500);
    }

    #[test]
    fn fast_mode01_signals_chunk_into_one_periodic() {
        // RPM + speed + throttle, all Fast → one 25 ms STPPMA, bare payload.
        let signals = vec![
            sig("010C", RefreshTier::Fast),
            sig("010D", RefreshTier::Fast),
            sig("0111", RefreshTier::Fast),
        ];
        let (set, left) = build_periodic_set(
            &signals,
            &admission_with(6, 1, &["010C", "010D", "0111", "0105"]),
            "7E0",
            &TierPeriods::default(),
            true,
        );
        assert!(left.is_empty());
        assert_eq!(set.len(), 1);
        assert_eq!(set[0].period_ms, 25);
        assert_eq!(set[0].header, "7E0");
        assert_eq!(
            set[0].data, "010C0D11",
            "mode prefix once + sorted suffixes"
        );
        assert_eq!(set[0].command(), "STPPMA 25, 7E0, 010C0D11");
    }

    #[test]
    fn tiers_split_into_separate_periodics() {
        // Fast RPM + Slow coolant → two messages at their own rates.
        let signals = vec![
            sig("010C", RefreshTier::Fast),
            sig("0105", RefreshTier::Slow),
        ];
        let (set, left) = build_periodic_set(
            &signals,
            &admission_with(6, 1, &["010C", "010D", "0111", "0105"]),
            "7E0",
            &TierPeriods::default(),
            true,
        );
        assert!(left.is_empty());
        assert_eq!(set.len(), 2);
        // Fast bucket first (divisor 1 < 10).
        assert_eq!((set[0].period_ms, set[0].data.as_str()), (25, "010C"));
        assert_eq!((set[1].period_ms, set[1].data.as_str()), (500, "0105"));
    }

    #[test]
    fn mode22_customs_chunk_and_keep_controller_header() {
        let signals = vec![
            ("22F40C".into(), Some("7E0".into()), RefreshTier::Fast),
            ("22F40D".into(), Some("7E0".into()), RefreshTier::Fast),
        ];
        let (set, left) = build_periodic_set(
            &signals,
            &admission_with(1, 2, &["22F40C", "22F40D"]),
            "7E0",
            &TierPeriods::default(),
            true,
        );
        assert!(left.is_empty());
        assert_eq!(set.len(), 1);
        assert_eq!(set[0].data, "22F40CF40D");
        assert_eq!(set[0].header, "7E0");
        assert_eq!(set[0].pids, vec!["22F40C", "22F40D"]);
    }

    #[test]
    fn chunk_limit_splits_overflow_into_multiple_periodics() {
        // 3 fast pids, chunk_limit 2 → two periodics (2 + 1).
        let signals = vec![
            sig("010C", RefreshTier::Fast),
            sig("010D", RefreshTier::Fast),
            sig("0111", RefreshTier::Fast),
        ];
        let (set, _) = build_periodic_set(
            &signals,
            &admission_with(2, 1, &["010C", "010D", "0111"]),
            "7E0",
            &TierPeriods::default(),
            true,
        );
        assert_eq!(set.len(), 2, "2-cap splits 3 into 2+1");
        assert!(set.iter().all(|m| m.period_ms == 25));
    }

    #[test]
    fn response_budget_splits_the_mustang_case() {
        // HARDWARE 2026-08-02: 0104(1B) + 010C(2B) + 010D(1B) responses cost
        // 1 + (1+1) + (1+2) + (1+1) = 8 bytes → one frame can't carry it →
        // the ECU answered with an ISO-TP FirstFrame that monitor mode can
        // never complete. The packer must SPLIT, never emit an 8-byte chunk.
        let signals = vec![
            sig("0104", RefreshTier::Fast),
            sig("010C", RefreshTier::Fast),
            sig("010D", RefreshTier::Fast),
        ];
        let adm = admission_with_lens(6, 1, &[("0104", 1), ("010C", 2), ("010D", 1)]);
        let (set, left) = build_periodic_set(&signals, &adm, "7E0", &TierPeriods::default(), true);
        assert!(left.is_empty());
        assert!(set.len() >= 2, "8-byte response must split: {set:?}");
        for m in &set {
            let cost: u16 = 1 + m
                .pids
                .iter()
                .map(|p| {
                    let len = match p.as_str() {
                        "010C" => 2,
                        _ => 1,
                    };
                    member_cost(p, len) as u16
                })
                .sum::<u16>();
            assert!(cost <= 7, "chunk {m:?} response = {cost} bytes");
        }
        // Every pid still acquired — split, not dropped.
        let all: Vec<&String> = set.iter().flat_map(|m| m.pids.iter()).collect();
        assert_eq!(all.len(), 3);
    }

    #[test]
    fn unknown_length_and_fat_pids_are_left_out() {
        let signals = vec![
            sig("010C", RefreshTier::Fast), // known, fits
            sig("0110", RefreshTier::Fast), // NOT in cache
            sig("090A", RefreshTier::Slow), // len 20 — can never fit
        ];
        let adm = admission_with_lens(6, 1, &[("010C", 2), ("090A", 20)]);
        let (set, left) = build_periodic_set(&signals, &adm, "7E0", &TierPeriods::default(), true);
        assert_eq!(set.len(), 1);
        assert_eq!(set[0].pids, vec!["010C"]);
        assert!(
            left.contains(&"0110".to_string()),
            "unknown length left out"
        );
        assert!(left.contains(&"090A".to_string()), "multi-frame left out");
    }

    #[test]
    fn fits_single_frame_budget() {
        assert!(fits_single_frame(
            &[("010C".into(), 2), ("010D".into(), 1)],
            6
        ));
        // 1 + 2 + 3 + 2 = 8 → no.
        assert!(!fits_single_frame(
            &[("0104".into(), 1), ("010C".into(), 2), ("010D".into(), 1)],
            6
        ));
        // Count cap respected.
        assert!(!fits_single_frame(
            &[("010C".into(), 1), ("010D".into(), 1)],
            1
        ));
        // Mode 22 echo is 2 bytes: 1 + (2+2) + (2+1) = 8 → no.
        assert!(!fits_single_frame(
            &[("22F40C".into(), 2), ("22F40D".into(), 1)],
            2
        ));
    }

    #[test]
    fn no_chunking_one_periodic_per_signal() {
        let signals = vec![
            sig("010C", RefreshTier::Fast),
            sig("010D", RefreshTier::Fast),
        ];
        let (set, _) = build_periodic_set(
            &signals,
            &admission_with(1, 1, &["010C", "010D"]),
            "7E0",
            &TierPeriods::default(),
            true,
        );
        assert_eq!(set.len(), 2, "chunk_limit 1 = solo each");
        assert_eq!(set[0].data, "010C");
        assert_eq!(set[1].data, "010D");
    }
}
