//! Discovery Ownership (docs/1.2.0/Discovery_Ownership_Plan.md): the
//! connector scan LOOP and the prune policy live HERE, session-side. The
//! platform is a stateless sensor — `scan_connectors_round` runs one
//! bounded round and answers with a snapshot; this loop owns cadence,
//! accumulation, RB2-A pruning, the merge with Rust's own transports
//! (USB serial, WiFi probe), emission, and lifecycle.

use super::*;
use crate::platform::ConnectorInfo;
use std::time::{Duration, Instant};

/// Tuning knobs, injectable so crate tests run the REAL loop at test speed
/// with a scripted probe (no live WiFi SYNs out of `cargo test`).
pub(super) struct DiscoveryCfg {
    /// A new round STARTS this long after the previous one started.
    pub interval: Duration,
    /// …but never sooner than this after it ENDED. A round can outlast the
    /// interval (a BLE scan window, a classic inquiry); without a floor the
    /// rounds run back to back and the radio is scanning 100% of the time —
    /// including the moment the user picks an adapter and we must connect.
    pub min_gap: Duration,
    /// A host round that hasn't answered by now is "no data" for this
    /// round — keep the previous list, never fabricate an empty snapshot.
    pub watchdog: Duration,
    /// Rust-owned connectors (USB serial, WiFi probe). Runs every round,
    /// concurrent with the host round; internally bounded (~2 s worst).
    pub probe: Arc<dyn Fn() -> Vec<ConnectorInfo> + Send + Sync>,
}

impl Default for DiscoveryCfg {
    fn default() -> Self {
        DiscoveryCfg {
            interval: Duration::from_secs(3),
            min_gap: Duration::from_secs(1),
            watchdog: Duration::from_secs(10),
            probe: Arc::new(crate::transport::router::discover_rust_connectors),
        }
    }
}

/// RB2-A policy, moved verbatim from Swift's `knownConnectors`: accumulate
/// in first-seen order; `ble` entries are advertising-based, so absence
/// from the CURRENT round's snapshot drops them; everything else the host
/// reports (classic paired list, mocks, demo) is sticky — those don't
/// advertise, absence isn't a signal.
fn integrate_host_round(entries: &mut Vec<ConnectorInfo>, snapshot: Vec<ConnectorInfo>) {
    let current_ble: std::collections::HashSet<String> = snapshot
        .iter()
        .filter(|c| c.connector_type == "ble")
        .map(|c| c.id.clone())
        .collect();
    entries.retain(|e| e.connector_type != "ble" || current_ble.contains(&e.id));
    for c in snapshot {
        if !entries.iter().any(|e| e.id == c.id) {
            entries.push(c);
        }
    }
}

/// How long to idle after a round that took `elapsed`: out to the interval,
/// and at least `min_gap` when the round overran it.
fn round_pause(elapsed: Duration, cfg: &DiscoveryCfg) -> Duration {
    cfg.interval.saturating_sub(elapsed).max(cfg.min_gap)
}

/// Host entries ∪ rust entries, host order first, deduped by id.
fn merge(host: &[ConnectorInfo], rust: Vec<ConnectorInfo>) -> Vec<ConnectorInfo> {
    let mut out = host.to_vec();
    for c in rust {
        if !out.iter().any(|m| m.id == c.id) {
            out.push(c);
        }
    }
    out
}

impl SessionAPIHandle {
    pub(super) fn spawn_discovery_loop(&self, generation: u64, cfg: DiscoveryCfg) {
        let api = self.clone();
        let _ = std::thread::Builder::new()
            .name("obd-discovery".into())
            .spawn(move || api.discovery_loop(generation, cfg));
    }

    /// True while this attempt's scan session should keep rounding.
    /// Round 1 runs BEFORE `awaiting_selection` is set; after that the flag
    /// dropping means a selection (or cancel/disconnect) ended the episode.
    fn discovery_should_run(&self, generation: u64, first: bool) -> bool {
        if !self.is_connect_current(generation) {
            return false;
        }
        let state = self.shared_state.lock().unwrap();
        state.running && (first || state.awaiting_selection)
    }

    fn discovery_loop(&self, generation: u64, cfg: DiscoveryCfg) {
        use crate::connection_msg::ConnectionMsg;
        let mut host_entries: Vec<ConnectorInfo> = Vec::new();
        let mut last_emitted: Vec<ConnectorInfo> = Vec::new();
        let mut first = true;
        loop {
            if !self.discovery_should_run(generation, first) {
                return;
            }
            let started = Instant::now();
            // Round activity ping — the host UI shows a scanning indicator
            // between "start" and "done" (rail session-status icon).
            self.send_status(ConnectionMsg::SCAN_ROUND, Some("start"), None);

            // Rust probe rides concurrent with the host round; joining after
            // the host answers barely blocks (probe ≤ ~2 s, host round ≥ 2 s).
            let probe_fn = Arc::clone(&cfg.probe);
            let probe = std::thread::Builder::new()
                .name("obd-discovery-probe".into())
                .spawn(move || probe_fn())
                .ok();

            let platform = Arc::clone(&self.shared_state.lock().unwrap().platform);
            let (tx, rx) = std::sync::mpsc::channel::<Vec<ConnectorInfo>>();
            platform.scan_connectors_round(Box::new(move |snapshot| {
                let _ = tx.send(snapshot);
            }));
            let host_round = rx.recv_timeout(cfg.watchdog);
            let rust_entries = probe.and_then(|h| h.join().ok()).unwrap_or_default();

            match host_round {
                Ok(snapshot) => integrate_host_round(&mut host_entries, snapshot),
                // Missed round ≠ empty round: keep the list as it stands.
                Err(_) => eprintln!(
                    "[DISCOVERY] host scan round unanswered after {:?} — keeping previous list",
                    cfg.watchdog
                ),
            }
            self.send_status(ConnectionMsg::SCAN_ROUND, Some("done"), None);

            if !self.is_connect_current(generation) {
                return; // cancelled / superseded / selected while rounding
            }

            let merged = merge(&host_entries, rust_entries);
            if first {
                first = false;
                if merged.is_empty() {
                    self.send_status(
                        ConnectionMsg::CONNECTION_FAILED,
                        Some(ConnectionMsg::REASON_NO_CONNECTORS),
                        None,
                    );
                    return;
                }
                // The UI ALWAYS gets the picker — even for a single connector
                // (user direction 2026-07-31: seeing and choosing the adapter
                // beats a silent auto-connect).
                self.shared_state.lock().unwrap().awaiting_selection = true;
                // SO2: the ONE waiting_for_selection per attempt — the host
                // runs its selection logic exactly once off this.
                self.store_and_emit_connectors(&merged, ConnectionMsg::WAITING_FOR_SELECTION);
                last_emitted = merged;
            } else if merged != last_emitted {
                // SO2: later list changes are a data-carrier refresh, NOT a
                // second waiting_for_selection (emit on change only —
                // identical lists are pure UI churn).
                self.store_and_emit_connectors(&merged, ConnectionMsg::CONNECTOR_LIST);
                last_emitted = merged;
            }

            std::thread::sleep(round_pause(started.elapsed(), &cfg));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::{
        ConnectionCallback, ConnectionStatus, OBDDataCallback, OBDPlatformInterface,
    };
    use std::collections::VecDeque;
    use std::sync::mpsc;

    fn c(id: &str, kind: &str) -> ConnectorInfo {
        ConnectorInfo {
            id: id.into(),
            name: id.to_uppercase(),
            connector_type: kind.into(),
        }
    }

    // ── cadence (pure) ──

    #[test]
    fn round_pause_fills_the_interval_and_floors_an_overrun() {
        let cfg = DiscoveryCfg {
            interval: Duration::from_secs(3),
            min_gap: Duration::from_secs(1),
            ..DiscoveryCfg::default()
        };
        // Quick round: idle out to the interval.
        assert_eq!(round_pause(Duration::from_millis(500), &cfg), Duration::from_millis(2500));
        // Round nearly filled the interval: the floor wins.
        assert_eq!(round_pause(Duration::from_millis(2600), &cfg), Duration::from_secs(1));
        // Round overran (BLE window + classic inquiry): still a real gap,
        // never back-to-back scanning.
        assert_eq!(round_pause(Duration::from_secs(11), &cfg), Duration::from_secs(1));
    }

    // ── prune policy (pure) ──

    #[test]
    fn prune_ble_absent_from_round_sticky_rest() {
        let mut entries = vec![
            c("bleA", "ble"),
            c("mockM", "mock"),
            c("classicC", "classic"),
        ];
        // bleA stops advertising; a new bleB appears.
        integrate_host_round(&mut entries, vec![c("bleB", "ble")]);
        let ids: Vec<&str> = entries.iter().map(|e| e.id.as_str()).collect();
        assert_eq!(
            ids,
            vec!["mockM", "classicC", "bleB"],
            "ble pruned, sticky kept, first-seen order"
        );
        // Same round again: idempotent, no dupes.
        integrate_host_round(&mut entries, vec![c("bleB", "ble")]);
        assert_eq!(entries.len(), 3);
    }

    #[test]
    fn merge_appends_rust_without_dupes() {
        let host = vec![c("mockM", "mock"), c("wifi:1", "wifi")];
        let out = merge(&host, vec![c("wifi:1", "wifi"), c("usb:0", "usb")]);
        let ids: Vec<&str> = out.iter().map(|e| e.id.as_str()).collect();
        assert_eq!(ids, vec!["mockM", "wifi:1", "usb:0"]);
    }

    // ── loop-level, against a scripted platform ──

    /// Scripted sensor: each `scan_connectors_round` pops one round —
    /// Some(list) answers immediately, None swallows the callback (the
    /// watchdog path), an exhausted script answers empty forever.
    struct ScriptedScanPlatform {
        rounds: Mutex<VecDeque<Option<Vec<ConnectorInfo>>>>,
    }

    impl ScriptedScanPlatform {
        fn new(rounds: Vec<Option<Vec<ConnectorInfo>>>) -> Arc<Self> {
            Arc::new(Self {
                rounds: Mutex::new(rounds.into()),
            })
        }
    }

    impl OBDPlatformInterface for ScriptedScanPlatform {
        fn send_command(&self, _command: &str, _timeout_ms: u32) {}
        fn set_completion_callback(&self, _callback: Option<Box<dyn Fn(String) + Send + Sync>>) {}
        fn set_data_callback(&self, _callback: Option<OBDDataCallback>) {}
        fn set_connection_callback(&self, _callback: Option<ConnectionCallback>) {}
        fn is_connected(&self) -> bool {
            false
        }
        fn connection_status(&self) -> ConnectionStatus {
            ConnectionStatus::Disconnected
        }
        fn scan_connectors_round(&self, callback: Box<dyn FnOnce(Vec<ConnectorInfo>) + Send>) {
            match self.rounds.lock().unwrap().pop_front() {
                Some(Some(list)) => callback(list),
                Some(None) => {} // never answers — watchdog covers it
                None => callback(Vec::new()),
            }
        }
    }

    fn boot(
        rounds: Vec<Option<Vec<ConnectorInfo>>>,
    ) -> (OBDSessionManager, SessionAPIHandle, mpsc::Receiver<String>) {
        let platform = ScriptedScanPlatform::new(rounds);
        let (tx, rx) = mpsc::channel::<String>();
        let session = OBDSessionManager::new(
            platform as Arc<dyn OBDPlatformInterface>,
            OBDSessionConfig::default(),
            move |msg| {
                let _ = tx.send(msg);
            },
        )
        .unwrap();
        let api = session.api_handle();
        (session, api, rx)
    }

    fn collect(rx: &mpsc::Receiver<String>, window: Duration) -> Vec<String> {
        let deadline = Instant::now() + window;
        let mut out = Vec::new();
        while Instant::now() < deadline {
            match rx.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
                Ok(m) => out.push(m),
                Err(_) => break,
            }
        }
        out
    }

    fn cfg(probe: Vec<ConnectorInfo>) -> DiscoveryCfg {
        DiscoveryCfg {
            interval: Duration::from_millis(60),
            min_gap: Duration::from_millis(10),
            watchdog: Duration::from_millis(150),
            probe: Arc::new(move || probe.clone()),
        }
    }

    fn waiting_count(msgs: &[String]) -> usize {
        msgs.iter()
            .filter(|m| m.contains("\"waiting_for_selection\""))
            .count()
    }

    fn connector_list_count(msgs: &[String]) -> usize {
        msgs.iter()
            .filter(|m| m.contains("\"connector_list\""))
            .count()
    }

    #[test]
    fn loop_prunes_merges_and_emits_on_change_only() {
        // SO2: R1 → ONE waiting_for_selection. R2 (ble gone → prune) → a
        // connector_list refresh, NOT a second waiting_for_selection. R3+
        // unchanged → silent.
        let (session, api, rx) = boot(vec![
            Some(vec![c("bleA", "ble"), c("mockM", "mock")]),
            Some(vec![c("mockM", "mock")]),
            Some(vec![c("mockM", "mock")]),
            Some(vec![c("mockM", "mock")]),
        ]);
        let generation = api.begin_connect_attempt();
        api.spawn_discovery_loop(generation, cfg(vec![c("wifi:1", "wifi")]));

        let msgs = collect(&rx, Duration::from_millis(400));
        assert_eq!(
            waiting_count(&msgs),
            1,
            "exactly one waiting_for_selection per attempt: {msgs:?}"
        );
        assert_eq!(
            connector_list_count(&msgs),
            1,
            "the prune change rides connector_list: {msgs:?}"
        );
        assert!(
            msgs.iter()
                .any(|m| m.contains("\"scan_round\"") && m.contains("start"))
                && msgs
                    .iter()
                    .any(|m| m.contains("\"scan_round\"") && m.contains("done")),
            "rounds ping start/done for the UI indicator: {msgs:?}"
        );
        let first = msgs
            .iter()
            .find(|m| m.contains("waiting_for_selection"))
            .unwrap();
        assert!(
            first.contains("bleA") && first.contains("mockM") && first.contains("wifi:1"),
            "round 1 carries host ∪ rust: {first}"
        );
        {
            let list = &api.shared_state.lock().unwrap().discovered_connectors;
            let ids: Vec<&str> = list.iter().map(|e| e.id.as_str()).collect();
            assert_eq!(ids, vec!["mockM", "wifi:1"], "stored truth pruned + merged");
        }
        api.cancel_connect_attempt(); // ends the loop (generation bump)
        drop(session);
    }

    #[test]
    fn watchdog_keeps_list_and_stays_silent() {
        // R1 answers; R2 never answers (watchdog); R3 answers same → no emit.
        let (session, api, rx) = boot(vec![
            Some(vec![c("mockM", "mock")]),
            None,
            Some(vec![c("mockM", "mock")]),
        ]);
        let generation = api.begin_connect_attempt();
        api.spawn_discovery_loop(generation, cfg(vec![]));

        let msgs = collect(&rx, Duration::from_millis(600));
        assert_eq!(waiting_count(&msgs), 1, "only round 1 emitted: {msgs:?}");
        assert_eq!(
            api.shared_state.lock().unwrap().discovered_connectors.len(),
            1,
            "missed round kept the list"
        );
        api.cancel_connect_attempt();
        drop(session);
    }

    #[test]
    fn stale_generation_stops_the_loop_silently() {
        let (session, api, rx) = boot(vec![
            Some(vec![c("mockM", "mock")]),
            Some(vec![c("mockM", "mock"), c("bleZ", "ble")]),
        ]);
        let generation = api.begin_connect_attempt();
        api.spawn_discovery_loop(generation, cfg(vec![]));
        // Let round 1 land, then supersede the attempt (what select does).
        let _ = collect(&rx, Duration::from_millis(90));
        api.begin_connect_attempt();
        let after = collect(&rx, Duration::from_millis(300));
        assert_eq!(
            waiting_count(&after),
            0,
            "no emissions after supersede: {after:?}"
        );
        drop(session);
    }

    #[test]
    fn empty_first_round_fails_with_no_connectors() {
        let (session, api, rx) = boot(vec![Some(vec![])]);
        let generation = api.begin_connect_attempt();
        api.spawn_discovery_loop(generation, cfg(vec![]));
        let msgs = collect(&rx, Duration::from_millis(300));
        assert!(
            msgs.iter()
                .any(|m| m.contains("connection_failed") && m.contains("no_connectors")),
            "empty round 1 fails the attempt: {msgs:?}"
        );
        assert_eq!(waiting_count(&msgs), 0);
        drop(session);
    }
}
