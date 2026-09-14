//! Session acquisition health (SH1) — per-window transfer stats + summary +
//! conservative anomaly detection.
//!
//! A **window** is a contiguous span on one acquisition tier
//! (`poll` / `uds-stream` / `stn-periodic`); inside a window signals are
//! bucketed into completed 5 s **intervals** (the same cadence as the
//! `stream_rx` counters). The tracker is pure: every entry point takes
//! `now_ms` (epoch millis, same time source `session_logger` stamps `ts`
//! with) so tests are deterministic; it never touches the wall clock, locks,
//! or I/O. The session-manager wiring feeds it counters at existing sites
//! and emits the returned events to the session jsonl + the Swift response
//! callback.
//!
//! Anomalies (log-only, conservative; each fires at most once per window):
//! * `tier_silent` — an ENGAGED tier (uds-stream / stn-periodic) window has
//!   ≥2 consecutive completed 5 s intervals with 0 signals while some
//!   earlier window this session delivered signals (the 29-bit stale-cache
//!   signature: healthy poll, dead stream).
//! * `rate_degraded` — after a tier has ≥3 completed nonzero intervals of
//!   baseline this session, ≥2 consecutive intervals below 50 % of that
//!   tier's baseline (mean of its completed nonzero intervals).
//! * `ladder_churn` — ≥4 tier switches within 10 minutes with no
//!   subscription edit between them.

/// Interval bucket length — matches the `stream_rx` 5 s counters.
pub const INTERVAL_MS: u64 = 5_000;

/// Rolling-window cap: close + reopen the current window on the SAME tier
/// once it reaches this age. Without it, a single-tier session (a plain
/// ELM327/Dragy never leaves poll) emits NOTHING until disconnect — no
/// live chart, and one giant window in the permanent record.
pub const MAX_WINDOW_MS: u64 = 60_000;
/// Ladder-churn observation span.
const CHURN_WINDOW_MS: u64 = 10 * 60 * 1000;
/// Ladder-churn switch threshold.
const CHURN_SWITCHES: usize = 4;

/// Acquisition tier a window runs on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tier {
    Poll,
    UdsStream,
    StnPeriodic,
    /// The same adapter-periodic rung as `StnPeriodic`, run by the OBDX
    /// Pro's DVI slot engine — a distinct NAME on the wire/logs/UI only
    /// (David 2026-08-29: "should be dvi-periodic and dvi").
    DviPeriodic,
}

impl Tier {
    pub fn as_str(self) -> &'static str {
        match self {
            Tier::Poll => "poll",
            Tier::UdsStream => "uds-stream",
            Tier::StnPeriodic => "stn-periodic",
            Tier::DviPeriodic => "dvi-periodic",
        }
    }

    /// Parse the wire/log spelling ("poll" | "uds-stream" | "stn-periodic" |
    /// "dvi-periodic").
    pub fn parse(s: &str) -> Option<Tier> {
        match s.trim() {
            "poll" => Some(Tier::Poll),
            "uds-stream" => Some(Tier::UdsStream),
            "stn-periodic" => Some(Tier::StnPeriodic),
            "dvi-periodic" => Some(Tier::DviPeriodic),
            _ => None,
        }
    }

    /// Engaged tiers are the ones the ladder brought up (not plain polling).
    fn engaged(self) -> bool {
        !matches!(self, Tier::Poll)
    }

    fn index(self) -> usize {
        match self {
            Tier::Poll => 0,
            Tier::UdsStream => 1,
            Tier::StnPeriodic | Tier::DviPeriodic => 2,
        }
    }
}

/// A finished acquisition window. `rtt_avg_ms` is supplied by the caller at
/// close time (command-processor running sum/count delta across the window).
#[derive(Debug, Clone)]
pub struct Window {
    pub tier: Tier,
    pub start_ms: u64,
    pub end_ms: u64,
    /// Delivered member signals inside the window.
    pub signals: u64,
    /// Completed 5 s intervals inside the window.
    pub batches: u64,
    pub rtt_avg_ms: Option<f64>,
}

impl Window {
    pub fn duration_s(&self) -> f64 {
        (self.end_ms.saturating_sub(self.start_ms)) as f64 / 1000.0
    }

    pub fn sig_per_s(&self) -> f64 {
        let d = self.duration_s();
        if d > 0.0 {
            self.signals as f64 / d
        } else {
            0.0
        }
    }
}

/// A fired anomaly (full form, for live emission).
#[derive(Debug, Clone)]
pub struct Anomaly {
    pub code: &'static str,
    pub tier: Tier,
    pub detail: String,
    pub at_ms: u64,
}

/// Compact anomaly reference kept for the session summary.
#[derive(Debug, Clone)]
pub struct AnomalyBrief {
    pub code: &'static str,
    pub tier: Tier,
    pub at_ms: u64,
}

/// What a tracker call produced — the caller emits these.
#[derive(Debug)]
pub enum HealthEvent {
    WindowClosed(Window),
    Anomaly(Anomaly),
}

/// End-of-session rollup.
#[derive(Debug, Clone)]
pub struct SessionSummary {
    pub started_ms: u64,
    pub ended_ms: u64,
    pub windows: Vec<Window>,
    pub anomalies: Vec<AnomalyBrief>,
    pub total_signals: u64,
}

/// The window currently being filled.
struct CurrentWindow {
    tier: Tier,
    start_ms: u64,
    signals: u64,
    batches: u64,
    interval_start_ms: u64,
    interval_signals: u64,
    zero_streak: u32,
    below_streak: u32,
    tier_silent_fired: bool,
    rate_degraded_fired: bool,
}

impl CurrentWindow {
    fn open(tier: Tier, now_ms: u64) -> Self {
        Self {
            tier,
            start_ms: now_ms,
            signals: 0,
            batches: 0,
            interval_start_ms: now_ms,
            interval_signals: 0,
            zero_streak: 0,
            below_streak: 0,
            tier_silent_fired: false,
            rate_degraded_fired: false,
        }
    }

    fn close(self, end_ms: u64, rtt_avg_ms: Option<f64>) -> Window {
        Window {
            tier: self.tier,
            start_ms: self.start_ms,
            end_ms,
            signals: self.signals,
            batches: self.batches,
            rtt_avg_ms,
        }
    }
}

/// Pure per-session acquisition-health tracker. See module docs.
pub struct HealthTracker {
    started_ms: Option<u64>,
    finished: bool,
    current: Option<CurrentWindow>,
    windows: Vec<Window>,
    anomalies: Vec<AnomalyBrief>,
    /// Per-tier (by `Tier::index`) sum/count of completed NONZERO interval
    /// signal counts — the session baseline for `rate_degraded`.
    interval_sum: [u64; 3],
    interval_cnt: [u64; 3],
    /// Some earlier (closed) window this session delivered signals — gates
    /// `tier_silent` on a healthy start.
    ever_window_with_signals: bool,
    /// Tier-switch timestamps since the last subscription edit (pruned to
    /// the churn span).
    switch_times: Vec<u64>,
}

impl Default for HealthTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl HealthTracker {
    pub fn new() -> Self {
        Self {
            started_ms: None,
            finished: false,
            current: None,
            windows: Vec::new(),
            anomalies: Vec::new(),
            interval_sum: [0; 3],
            interval_cnt: [0; 3],
            ever_window_with_signals: false,
            switch_times: Vec::new(),
        }
    }

    /// Complete any 5 s intervals `now_ms` has passed and run the per-interval
    /// anomaly checks. Never closes windows.
    fn roll_intervals(&mut self, now_ms: u64) -> Vec<HealthEvent> {
        let mut events = Vec::new();
        let Some(mut cur) = self.current.take() else {
            return events;
        };
        while now_ms >= cur.interval_start_ms + INTERVAL_MS {
            let n = cur.interval_signals;
            cur.interval_signals = 0;
            cur.interval_start_ms += INTERVAL_MS;
            cur.batches += 1;
            let ti = cur.tier.index();

            if n == 0 {
                cur.zero_streak += 1;
            } else {
                cur.zero_streak = 0;
            }

            // rate_degraded — compare against the baseline BEFORE this
            // interval joins it (a degraded interval must not dilute the
            // reference it is judged against).
            if self.interval_cnt[ti] >= 3 {
                let baseline = self.interval_sum[ti] as f64 / self.interval_cnt[ti] as f64;
                if (n as f64) < 0.5 * baseline {
                    cur.below_streak += 1;
                } else {
                    cur.below_streak = 0;
                }
                if cur.below_streak >= 2 && !cur.rate_degraded_fired {
                    cur.rate_degraded_fired = true;
                    let at_ms = cur.interval_start_ms;
                    events.push(self.record_anomaly(
                        "rate_degraded",
                        cur.tier,
                        format!(
                            "{} rate below 50% of session baseline ({:.1} sig per 5s) \
                             for 2+ consecutive intervals (latest {} signals)",
                            cur.tier.as_str(),
                            baseline,
                            n
                        ),
                        at_ms,
                    ));
                }
            }
            if n > 0 {
                self.interval_sum[ti] += n;
                self.interval_cnt[ti] += 1;
            }

            // tier_silent — an engaged tier gone quiet after a healthy start.
            if cur.tier.engaged()
                && cur.zero_streak >= 2
                && self.ever_window_with_signals
                && !cur.tier_silent_fired
            {
                cur.tier_silent_fired = true;
                let at_ms = cur.interval_start_ms;
                events.push(self.record_anomaly(
                    "tier_silent",
                    cur.tier,
                    format!(
                        "{} delivered 0 signals for 2+ consecutive 5s intervals \
                         after earlier windows had data",
                        cur.tier.as_str()
                    ),
                    at_ms,
                ));
            }
        }
        self.current = Some(cur);
        events
    }

    fn record_anomaly(
        &mut self,
        code: &'static str,
        tier: Tier,
        detail: String,
        at_ms: u64,
    ) -> HealthEvent {
        self.anomalies.push(AnomalyBrief { code, tier, at_ms });
        HealthEvent::Anomaly(Anomaly {
            code,
            tier,
            detail,
            at_ms,
        })
    }

    fn close_current(&mut self, now_ms: u64, rtt_avg_ms: Option<f64>) -> Option<Window> {
        let cur = self.current.take()?;
        let window = cur.close(now_ms, rtt_avg_ms);
        if window.signals > 0 {
            self.ever_window_with_signals = true;
        }
        self.windows.push(window.clone());
        Some(window)
    }

    /// The session is now on `tier`. Closes the current window on a tier
    /// change (the caller supplies that window's `rtt_avg_ms`, from its
    /// command-processor sum/count snapshot delta) and opens the next one.
    /// After `finish()` a call here starts a fresh session (reconnect).
    pub fn on_tier(
        &mut self,
        tier: Tier,
        now_ms: u64,
        rtt_avg_ms: Option<f64>,
    ) -> Vec<HealthEvent> {
        if self.finished {
            *self = Self::new();
        }
        self.started_ms.get_or_insert(now_ms);
        match &self.current {
            None => {
                self.current = Some(CurrentWindow::open(tier, now_ms));
                Vec::new()
            }
            Some(cur) if cur.tier == tier => self.roll_intervals(now_ms),
            Some(_) => {
                let mut events = self.roll_intervals(now_ms);
                if let Some(window) = self.close_current(now_ms, rtt_avg_ms) {
                    events.push(HealthEvent::WindowClosed(window));
                }
                self.current = Some(CurrentWindow::open(tier, now_ms));
                // ladder_churn — switches since the last subscription edit,
                // inside the rolling 10-minute span.
                self.switch_times
                    .retain(|t| now_ms.saturating_sub(*t) <= CHURN_WINDOW_MS);
                self.switch_times.push(now_ms);
                if self.switch_times.len() >= CHURN_SWITCHES {
                    let count = self.switch_times.len();
                    self.switch_times.clear(); // re-arm: 4 fresh switches to fire again
                    events.push(self.record_anomaly(
                        "ladder_churn",
                        tier,
                        format!(
                            "{count} tier switches within 10 minutes with no subscription edit"
                        ),
                        now_ms,
                    ));
                }
                events
            }
        }
    }

    /// CONNECTED was just emitted: window 1 starts NOW. Discards any window
    /// opened lazily during adapter init/identify (`on_signals` opens on the
    /// first delivered signal — field logs show that's `ATWS`, 8-9 s before
    /// CONNECTED — so window 1 otherwise blends protocol-search/VIN/support
    /// scans into its duration, sig/s, and RTT: measured 420 ms vs 74 ms real
    /// polling on the Mustang). The discarded pre-window is init traffic, not
    /// acquisition — it is dropped, not emitted. The caller must also reset
    /// its RTT stats baseline (HealthState::window_rtt_ms consume).
    pub fn reset_for_connected(&mut self, now_ms: u64) {
        if self.finished {
            *self = Self::new();
        }
        self.started_ms = Some(now_ms);
        self.current = Some(CurrentWindow::open(Tier::Poll, now_ms));
    }

    /// `n` member signals were just delivered. Opens a poll window lazily if
    /// none is open (polling can precede any explicit tier call). No-op
    /// after `finish()` — drain stragglers must not start a ghost session.
    pub fn on_signals(&mut self, n: u64, now_ms: u64) -> Vec<HealthEvent> {
        if self.finished {
            return Vec::new();
        }
        if self.current.is_none() {
            self.started_ms.get_or_insert(now_ms);
            self.current = Some(CurrentWindow::open(Tier::Poll, now_ms));
        }
        let events = self.roll_intervals(now_ms);
        if let Some(cur) = self.current.as_mut() {
            cur.signals += n;
            cur.interval_signals += n;
        }
        events
    }

    /// Clock tick with no data — completes intervals while a tier is silent
    /// (`tier_silent` needs time to pass without any signal callback).
    pub fn on_tick(&mut self, now_ms: u64) -> Vec<HealthEvent> {
        if self.finished {
            return Vec::new();
        }
        self.roll_intervals(now_ms)
    }

    /// Cheap predicate for the caller: is the current window old enough to
    /// roll? Split from [`Self::roll_if_due`] because computing the closing
    /// RTT (`window_rtt_ms`) CONSUMES the caller's stats-snapshot baseline —
    /// it must not run on ticks that won't roll.
    pub fn window_due_to_roll(&self, now_ms: u64) -> bool {
        !self.finished
            && matches!(&self.current,
                        Some(c) if now_ms.saturating_sub(c.start_ms) >= MAX_WINDOW_MS)
    }

    /// Rolling windows: close + reopen the current window on the SAME tier
    /// once it reaches [`MAX_WINDOW_MS`]. NOT a tier switch — no churn
    /// bookkeeping. Caller (the ~1 s maintenance tick) supplies the closing
    /// window's `rtt_avg_ms` exactly like `on_tier` callers do.
    pub fn roll_if_due(&mut self, now_ms: u64, rtt_avg_ms: Option<f64>) -> Vec<HealthEvent> {
        if self.finished {
            return Vec::new();
        }
        let due_tier = match &self.current {
            Some(c) if now_ms.saturating_sub(c.start_ms) >= MAX_WINDOW_MS => c.tier,
            _ => return Vec::new(),
        };
        let mut events = self.roll_intervals(now_ms);
        if let Some(window) = self.close_current(now_ms, rtt_avg_ms) {
            events.push(HealthEvent::WindowClosed(window));
        }
        self.current = Some(CurrentWindow::open(due_tier, now_ms));
        events
    }

    /// A subscription add/remove happened — engage/teardown around it is
    /// user-driven, not churn.
    pub fn on_subscription_edit(&mut self, _now_ms: u64) {
        self.switch_times.clear();
    }

    /// End of session: closes the open window (caller supplies its
    /// `rtt_avg_ms`) and returns the trailing events (final window close +
    /// any last-interval anomalies — every window gets its
    /// `acquisition_window` emission, the final one included) plus the
    /// summary. Returns `None` when the session never started or the
    /// summary was already taken (the double-emit guard for disconnect vs
    /// shutdown).
    pub fn finish(
        &mut self,
        now_ms: u64,
        rtt_avg_ms: Option<f64>,
    ) -> Option<(Vec<HealthEvent>, SessionSummary)> {
        if self.finished {
            return None;
        }
        let started_ms = self.started_ms?;
        let mut events = self.roll_intervals(now_ms);
        if let Some(window) = self.close_current(now_ms, rtt_avg_ms) {
            events.push(HealthEvent::WindowClosed(window));
        }
        self.finished = true;
        let summary = SessionSummary {
            started_ms,
            ended_ms: now_ms,
            windows: self.windows.clone(),
            anomalies: self.anomalies.clone(),
            total_signals: self.windows.iter().map(|w| w.signals).sum(),
        };
        Some((events, summary))
    }
}

// ---------------------------------------------------------------------------
// Payload builders — the FROZEN wire contract shared with the Swift consumer.
// Field names must not change.
// ---------------------------------------------------------------------------

/// `{"kind":"acquisition_window",...}` payload.
pub fn window_payload(w: &Window) -> serde_json::Value {
    serde_json::json!({
        "kind": "acquisition_window",
        "tier": w.tier.as_str(),
        "start_ms": w.start_ms,
        "end_ms": w.end_ms,
        "duration_s": w.duration_s(),
        "signals": w.signals,
        "sig_per_s": w.sig_per_s(),
        "batches": w.batches,
        "rtt_avg_ms": w.rtt_avg_ms,
    })
}

/// `{"kind":"anomaly",...}` payload.
pub fn anomaly_payload(a: &Anomaly) -> serde_json::Value {
    serde_json::json!({
        "kind": "anomaly",
        "code": a.code,
        "tier": a.tier.as_str(),
        "detail": a.detail,
        "at_ms": a.at_ms,
    })
}

/// `{"kind":"session_summary",...}` payload.
pub fn summary_payload(s: &SessionSummary) -> serde_json::Value {
    serde_json::json!({
        "kind": "session_summary",
        "started_ms": s.started_ms,
        "ended_ms": s.ended_ms,
        "duration_s": (s.ended_ms.saturating_sub(s.started_ms)) as f64 / 1000.0,
        "windows": s.windows.iter().map(window_payload).collect::<Vec<_>>(),
        "anomalies": s.anomalies.iter().map(|a| serde_json::json!({
            "code": a.code,
            "tier": a.tier.as_str(),
            "at_ms": a.at_ms,
        })).collect::<Vec<_>>(),
        "totals": { "signals": s.total_signals },
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const T0: u64 = 1_000_000;

    fn windows_of(events: &[HealthEvent]) -> Vec<&Window> {
        events
            .iter()
            .filter_map(|e| match e {
                HealthEvent::WindowClosed(w) => Some(w),
                _ => None,
            })
            .collect()
    }

    fn anomalies_of(events: &[HealthEvent]) -> Vec<&Anomaly> {
        events
            .iter()
            .filter_map(|e| match e {
                HealthEvent::Anomaly(a) => Some(a),
                _ => None,
            })
            .collect()
    }

    #[test]
    fn window_rollover_on_tier_change() {
        let mut t = HealthTracker::new();
        assert!(t.on_tier(Tier::Poll, T0, None).is_empty());
        // 3 intervals of signals on poll.
        for i in 0..3u64 {
            assert!(anomalies_of(&t.on_signals(100, T0 + i * INTERVAL_MS + 100)).is_empty());
        }
        // Switch to uds-stream at t=16s → poll window closes.
        let events = t.on_tier(Tier::UdsStream, T0 + 16_000, Some(42.5));
        let ws = windows_of(&events);
        assert_eq!(ws.len(), 1);
        let w = ws[0];
        assert_eq!(w.tier, Tier::Poll);
        assert_eq!(w.start_ms, T0);
        assert_eq!(w.end_ms, T0 + 16_000);
        assert_eq!(w.signals, 300);
        assert_eq!(w.batches, 3); // 3 completed 5s intervals in 16s
        assert!((w.duration_s() - 16.0).abs() < 1e-9);
        assert!((w.sig_per_s() - 300.0 / 16.0).abs() < 1e-9);
        assert_eq!(w.rtt_avg_ms, Some(42.5));
        assert!(anomalies_of(&events).is_empty());
    }

    #[test]
    fn same_tier_call_does_not_close_window() {
        let mut t = HealthTracker::new();
        t.on_tier(Tier::Poll, T0, None);
        t.on_signals(10, T0 + 100);
        let events = t.on_tier(Tier::Poll, T0 + 200, None);
        assert!(windows_of(&events).is_empty());
        let (events, summary) = t.finish(T0 + 300, None).unwrap();
        assert_eq!(
            windows_of(&events).len(),
            1,
            "finish closes the open window"
        );
        assert_eq!(summary.windows.len(), 1);
        assert_eq!(summary.total_signals, 10);
    }

    #[test]
    fn tier_silent_fires_once_per_window() {
        let mut t = HealthTracker::new();
        // Healthy poll window first (the "earlier window had signals" gate).
        t.on_tier(Tier::Poll, T0, None);
        t.on_signals(500, T0 + 1_000);
        t.on_tier(Tier::UdsStream, T0 + 10_000, None);
        // Two silent 5s intervals → fires exactly once.
        let e1 = t.on_tick(T0 + 10_000 + INTERVAL_MS);
        assert!(
            anomalies_of(&e1).is_empty(),
            "one zero interval is not enough"
        );
        let e2 = t.on_tick(T0 + 10_000 + 2 * INTERVAL_MS);
        let a = anomalies_of(&e2);
        assert_eq!(a.len(), 1);
        assert_eq!(a[0].code, "tier_silent");
        assert_eq!(a[0].tier, Tier::UdsStream);
        // More silence: no re-fire inside the same window.
        let e3 = t.on_tick(T0 + 10_000 + 5 * INTERVAL_MS);
        assert!(anomalies_of(&e3).is_empty());
    }

    #[test]
    fn tier_silent_does_not_fire_without_healthy_history_or_on_poll() {
        // No earlier window with signals → engaged silence is not anomalous.
        let mut t = HealthTracker::new();
        t.on_tier(Tier::UdsStream, T0, None);
        let e = t.on_tick(T0 + 3 * INTERVAL_MS);
        assert!(anomalies_of(&e).is_empty());

        // Poll is not an engaged tier — silence never fires tier_silent.
        let mut t = HealthTracker::new();
        t.on_tier(Tier::Poll, T0, None);
        t.on_signals(100, T0 + 100);
        t.on_tier(Tier::UdsStream, T0 + 6_000, None);
        t.on_tier(Tier::Poll, T0 + 7_000, None);
        let e = t.on_tick(T0 + 7_000 + 4 * INTERVAL_MS);
        assert!(anomalies_of(&e).is_empty());
    }

    #[test]
    fn rate_degraded_fires_after_baseline_and_two_low_intervals() {
        let mut t = HealthTracker::new();
        t.on_tier(Tier::Poll, T0, None);
        // 3 baseline intervals at 100 signals each.
        for i in 0..3u64 {
            t.on_signals(100, T0 + i * INTERVAL_MS + 100);
        }
        // Two consecutive intervals at 30 (<50% of 100).
        t.on_signals(30, T0 + 3 * INTERVAL_MS + 100);
        let e4 = t.on_tick(T0 + 4 * INTERVAL_MS);
        assert!(
            anomalies_of(&e4).is_empty(),
            "one low interval is not enough"
        );
        t.on_signals(30, T0 + 4 * INTERVAL_MS + 100);
        let e5 = t.on_tick(T0 + 5 * INTERVAL_MS);
        let a = anomalies_of(&e5);
        assert_eq!(a.len(), 1);
        assert_eq!(a[0].code, "rate_degraded");
        assert_eq!(a[0].tier, Tier::Poll);
        // Continued low rate: no re-fire in the same window.
        t.on_signals(30, T0 + 5 * INTERVAL_MS + 100);
        let e6 = t.on_tick(T0 + 6 * INTERVAL_MS);
        assert!(anomalies_of(&e6).is_empty());
    }

    #[test]
    fn rate_degraded_does_not_fire_without_baseline_or_after_recovery() {
        // Only 2 baseline intervals → never fires.
        let mut t = HealthTracker::new();
        t.on_tier(Tier::Poll, T0, None);
        for i in 0..2u64 {
            t.on_signals(100, T0 + i * INTERVAL_MS + 100);
        }
        t.on_signals(10, T0 + 2 * INTERVAL_MS + 100);
        t.on_signals(10, T0 + 3 * INTERVAL_MS + 100);
        let e = t.on_tick(T0 + 4 * INTERVAL_MS);
        assert!(anomalies_of(&e).is_empty());

        // Recovery between low intervals resets the streak.
        let mut t = HealthTracker::new();
        t.on_tier(Tier::Poll, T0, None);
        for i in 0..3u64 {
            t.on_signals(100, T0 + i * INTERVAL_MS + 100);
        }
        t.on_signals(30, T0 + 3 * INTERVAL_MS + 100); // low
        t.on_signals(100, T0 + 4 * INTERVAL_MS + 100); // recovered
        t.on_signals(30, T0 + 5 * INTERVAL_MS + 100); // low again (streak=1)
        let e = t.on_tick(T0 + 6 * INTERVAL_MS);
        assert!(anomalies_of(&e).is_empty());
    }

    #[test]
    fn ladder_churn_fires_on_four_switches_without_edit() {
        let mut t = HealthTracker::new();
        t.on_tier(Tier::Poll, T0, None);
        let mut fired = Vec::new();
        for (i, tier) in [Tier::UdsStream, Tier::Poll, Tier::UdsStream, Tier::Poll]
            .into_iter()
            .enumerate()
        {
            let events = t.on_tier(tier, T0 + (i as u64 + 1) * 30_000, None);
            fired.extend(
                anomalies_of(&events)
                    .iter()
                    .map(|a| (a.code, a.at_ms))
                    .collect::<Vec<_>>(),
            );
        }
        assert_eq!(fired.len(), 1);
        assert_eq!(fired[0].0, "ladder_churn");
        assert_eq!(fired[0].1, T0 + 4 * 30_000);
        // Re-armed: the 5th switch alone does not fire again.
        let events = t.on_tier(Tier::UdsStream, T0 + 5 * 30_000, None);
        assert!(anomalies_of(&events).is_empty());
    }

    #[test]
    fn ladder_churn_suppressed_by_subscription_edit_and_time() {
        // An edit between switches resets the count.
        let mut t = HealthTracker::new();
        t.on_tier(Tier::Poll, T0, None);
        t.on_tier(Tier::UdsStream, T0 + 30_000, None);
        t.on_tier(Tier::Poll, T0 + 60_000, None);
        t.on_subscription_edit(T0 + 70_000);
        let e3 = t.on_tier(Tier::UdsStream, T0 + 90_000, None);
        let e4 = t.on_tier(Tier::Poll, T0 + 120_000, None);
        assert!(anomalies_of(&e3).is_empty());
        assert!(anomalies_of(&e4).is_empty());

        // Switches spread beyond 10 minutes never accumulate to 4.
        let mut t = HealthTracker::new();
        t.on_tier(Tier::Poll, T0, None);
        for i in 0..6u64 {
            let tier = if i % 2 == 0 {
                Tier::UdsStream
            } else {
                Tier::Poll
            };
            let events = t.on_tier(tier, T0 + (i + 1) * 6 * 60_000, None);
            assert!(anomalies_of(&events).is_empty(), "switch {i} must not fire");
        }
    }

    #[test]
    fn finish_summarizes_and_guards_double_emit() {
        let mut t = HealthTracker::new();
        t.on_tier(Tier::Poll, T0, None);
        t.on_signals(120, T0 + 1_000);
        t.on_tier(Tier::UdsStream, T0 + 10_000, Some(50.0));
        t.on_signals(400, T0 + 11_000);
        let (events, summary) = t
            .finish(T0 + 20_000, None)
            .expect("first finish yields summary");
        // The final (uds-stream) window closes AT finish and is emitted.
        let ws = windows_of(&events);
        assert_eq!(ws.len(), 1);
        assert_eq!(ws[0].tier, Tier::UdsStream);
        assert_eq!(summary.started_ms, T0);
        assert_eq!(summary.ended_ms, T0 + 20_000);
        assert_eq!(summary.windows.len(), 2);
        assert_eq!(summary.windows[0].tier, Tier::Poll);
        assert_eq!(summary.windows[0].signals, 120);
        assert_eq!(summary.windows[0].rtt_avg_ms, Some(50.0));
        assert_eq!(summary.windows[1].tier, Tier::UdsStream);
        assert_eq!(summary.windows[1].signals, 400);
        assert_eq!(summary.total_signals, 520);
        assert!(summary.anomalies.is_empty());
        // Double-emit guard.
        assert!(t.finish(T0 + 21_000, None).is_none());
        // Signals after finish are ignored (drain stragglers).
        assert!(t.on_signals(5, T0 + 22_000).is_empty());
        assert!(t.finish(T0 + 23_000, None).is_none());
    }

    #[test]
    fn finish_before_start_returns_none_and_on_tier_restarts_after_finish() {
        let mut t = HealthTracker::new();
        assert!(t.finish(T0, None).is_none());

        t.on_tier(Tier::Poll, T0, None);
        t.on_signals(10, T0 + 100);
        assert!(t.finish(T0 + 1_000, None).is_some());
        // Reconnect: a fresh session begins.
        t.on_tier(Tier::Poll, T0 + 60_000, None);
        t.on_signals(7, T0 + 60_100);
        let (_, s) = t.finish(T0 + 61_000, None).unwrap();
        assert_eq!(s.started_ms, T0 + 60_000);
        assert_eq!(s.total_signals, 7);
        assert_eq!(s.windows.len(), 1);
    }

    /// Window 1 restarts at CONNECTED: the lazily-opened identify window
    /// (first signal arrives during adapter init) is discarded — never
    /// emitted — and the first REAL window starts at connect time, so its
    /// duration/signals/RTT cover dashboard traffic only.
    #[test]
    fn reset_for_connected_discards_identify_window() {
        let mut t = HealthTracker::new();
        // Identify phase: a support-scan response opens a poll window at 1s.
        let events = t.on_signals(5, 1_000);
        assert!(events.is_empty());

        // CONNECTED at 9s.
        t.reset_for_connected(9_000);

        // Dashboard polls, then engage closes window 1 at 15s.
        t.on_signals(60, 12_000);
        let events = t.on_tier(Tier::UdsStream, 15_000, Some(74.0));
        let windows: Vec<_> = events
            .iter()
            .filter_map(|e| match e {
                HealthEvent::WindowClosed(w) => Some(w),
                _ => None,
            })
            .collect();
        assert_eq!(windows.len(), 1, "identify pre-window must not be emitted");
        let w = windows[0];
        assert_eq!(w.start_ms, 9_000, "window 1 starts at CONNECTED");
        assert_eq!(w.end_ms, 15_000);
        assert_eq!(w.signals, 60, "identify-phase signals excluded");
        assert_eq!(w.rtt_avg_ms, Some(74.0));
        let (_, summary) = t.finish(20_000, None).expect("summary");
        assert_eq!(
            summary.started_ms, 9_000,
            "session start anchors to CONNECTED"
        );
    }

    /// Single-tier sessions (Dragy/plain ELM: poll forever) must still emit
    /// windows via the rolling cap — and rolls are NOT churn switches.
    #[test]
    fn rolling_cap_closes_same_tier_windows() {
        let mut t = HealthTracker::new();
        t.on_tier(Tier::Poll, T0, None);
        t.on_signals(100, T0 + 1_000);
        // Not due before the cap.
        assert!(!t.window_due_to_roll(T0 + MAX_WINDOW_MS - 1));
        assert!(t.roll_if_due(T0 + MAX_WINDOW_MS - 1, Some(10.0)).is_empty());
        // Due at the cap: closes window 1, reopens poll.
        assert!(t.window_due_to_roll(T0 + MAX_WINDOW_MS));
        let events = t.roll_if_due(T0 + MAX_WINDOW_MS, Some(10.0));
        let closed: Vec<_> = events
            .iter()
            .filter(|e| matches!(e, HealthEvent::WindowClosed(_)))
            .collect();
        assert_eq!(closed.len(), 1);
        // Second stretch also rolls; still poll, still no ladder_churn.
        t.on_signals(50, T0 + MAX_WINDOW_MS + 1_000);
        let events2 = t.roll_if_due(T0 + 2 * MAX_WINDOW_MS, Some(11.0));
        assert!(events2
            .iter()
            .any(|e| matches!(e, HealthEvent::WindowClosed(w) if w.tier == Tier::Poll)));
        assert!(!events2
            .iter()
            .any(|e| matches!(e, HealthEvent::Anomaly(a) if a.code == "ladder_churn")));
        let (_, s) = t
            .finish(T0 + 2 * MAX_WINDOW_MS + 5_000, Some(12.0))
            .unwrap();
        assert_eq!(s.windows.len(), 3, "two rolled + final");
        assert_eq!(s.total_signals, 150);
    }

    #[test]
    fn on_signals_opens_poll_window_lazily() {
        let mut t = HealthTracker::new();
        t.on_signals(42, T0);
        let (_, s) = t.finish(T0 + 2_000, Some(12.0)).unwrap();
        assert_eq!(s.windows.len(), 1);
        assert_eq!(s.windows[0].tier, Tier::Poll);
        assert_eq!(s.windows[0].signals, 42);
        assert_eq!(s.windows[0].rtt_avg_ms, Some(12.0));
        assert_eq!(s.total_signals, 42);
    }

    #[test]
    fn payloads_match_frozen_contract() {
        let w = Window {
            tier: Tier::UdsStream,
            start_ms: 1_000,
            end_ms: 11_000,
            signals: 250,
            batches: 2,
            rtt_avg_ms: None,
        };
        let p = window_payload(&w);
        assert_eq!(p["kind"], "acquisition_window");
        assert_eq!(p["tier"], "uds-stream");
        assert_eq!(p["start_ms"], 1_000);
        assert_eq!(p["end_ms"], 11_000);
        assert_eq!(p["duration_s"], 10.0);
        assert_eq!(p["signals"], 250);
        assert_eq!(p["sig_per_s"], 25.0);
        assert_eq!(p["batches"], 2);
        assert!(p["rtt_avg_ms"].is_null());

        let a = Anomaly {
            code: "tier_silent",
            tier: Tier::StnPeriodic,
            detail: "example".to_string(),
            at_ms: 5_000,
        };
        let p = anomaly_payload(&a);
        assert_eq!(p["kind"], "anomaly");
        assert_eq!(p["code"], "tier_silent");
        assert_eq!(p["tier"], "stn-periodic");
        assert_eq!(p["detail"], "example");
        assert_eq!(p["at_ms"], 5_000);

        let s = SessionSummary {
            started_ms: 0,
            ended_ms: 20_000,
            windows: vec![w],
            anomalies: vec![AnomalyBrief {
                code: "tier_silent",
                tier: Tier::UdsStream,
                at_ms: 9_000,
            }],
            total_signals: 250,
        };
        let p = summary_payload(&s);
        assert_eq!(p["kind"], "session_summary");
        assert_eq!(p["started_ms"], 0);
        assert_eq!(p["ended_ms"], 20_000);
        assert_eq!(p["duration_s"], 20.0);
        assert_eq!(p["windows"].as_array().unwrap().len(), 1);
        assert_eq!(p["windows"][0]["kind"], "acquisition_window");
        assert_eq!(p["anomalies"][0]["code"], "tier_silent");
        assert_eq!(p["anomalies"][0]["tier"], "uds-stream");
        assert_eq!(p["anomalies"][0]["at_ms"], 9_000);
        assert_eq!(p["totals"]["signals"], 250);
    }
}
