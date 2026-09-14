//! Subscription Manager for PID monitoring
//!
//! Manages PID subscriptions with deduplication, lifecycle control,
//! and efficient polling coordination.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};
use uuid::Uuid;

use crate::error::SubscriptionError;
use crate::pid_registry::GlobalPIDRegistry;

/// Refresh rate tier for PID polling priority.
/// Fast PIDs are polled every cycle, Medium every 3rd, Slow every 10th.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RefreshTier {
    Fast = 1,
    Medium = 3,
    Slow = 10,
}

impl RefreshTier {
    pub fn divisor(&self) -> u32 {
        *self as u32
    }

    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => RefreshTier::Fast,
            10 => RefreshTier::Slow,
            _ => RefreshTier::Medium,
        }
    }
}

/// Layered tier resolution (#18), most-specific wins:
/// 1. an explicit entry in `tiers` (curated Mode 01 map seeded per
///    subscription + any app-sent overrides via set_pid_tiers)
/// 2. `F4xx` mirror inherit: `22F4XX` is Mode 01 PID `01XX` read over UDS —
///    it inherits that pid's tier (so `22F40C` RPM is Fast, not Medium)
/// 3. Medium fallback.
/// (The unit heuristic runs app-side — units live on DataPoints — and
/// arrives through `tiers` like any other override.)
pub fn resolve_tier(pid: &str, tiers: &HashMap<String, RefreshTier>) -> RefreshTier {
    if let Some(t) = tiers.get(pid) {
        return *t;
    }
    let p = pid.trim().to_uppercase();
    if p.len() == 6 && p.starts_with("22F4") {
        if let Some(t) = tiers.get(&format!("01{}", &p[4..])) {
            return *t;
        }
    }
    RefreshTier::Medium
}

/// Returns the default tier mapping for standard OBD-II PIDs.
/// PIDs not in this map default to Medium.
pub fn default_pid_tiers() -> HashMap<String, RefreshTier> {
    let mut map = HashMap::new();
    // Fast — values that change rapidly while driving
    for pid in &["010C", "010D", "0111", "0110", "0104"] {
        map.insert(pid.to_string(), RefreshTier::Fast);
    }
    // Slow — values that change gradually or never
    for pid in &[
        "0105", "015C", "0133", "012F", "011F", "0131", "0902", "090A", "0142",
    ] {
        map.insert(pid.to_string(), RefreshTier::Slow);
    }
    // Everything else defaults to Medium via .unwrap_or()
    map
}

/// Subscription states
#[derive(Debug, Clone, PartialEq)]
pub enum SubscriptionState {
    /// Actively polling
    Active,
    /// Temporarily paused
    Paused,
    /// Permanently cancelled
    Cancelled,
    /// All run-once PIDs completed (no longer polling)
    Completed,
    /// Error state with message
    Error(String),
}

/// Individual PID subscription
#[derive(Debug, Clone)]
pub struct Subscription {
    /// Unique subscription ID
    pub id: Uuid,
    /// Human-readable name (e.g., "Dashboard", "Discovery")
    pub name: Option<String>,
    /// Set of PIDs being monitored
    pub pids: std::collections::HashSet<String>,
    /// Target controller (e.g., "7E0")
    pub target_controller: Option<String>,
    /// Current state
    pub state: SubscriptionState,
    /// Creation timestamp (Unix timestamp in seconds)
    pub created_at: f64,
    /// Last activity timestamp (Unix timestamp in seconds)
    pub last_active: Option<f64>,
    /// The HOST paused this subscription (dashboard hidden, user pause) —
    /// distinct from an acquisition pause (stream/sink engage pauses subs it
    /// will resume at teardown). `resume_many` — the acquisition-side resume —
    /// must NOT resurrect a host-paused subscription: field bug 2026-08-06,
    /// leaving the dashboard paused it, the stream teardown resumed it, and
    /// the ladder re-engaged streaming while the user sat in the console.
    pub host_paused: bool,
    /// Run count for each PID (None = continuous, Some(n) = run n times)
    pub pid_run_counts: std::collections::HashMap<String, Option<u32>>,
    /// Execution count for each PID
    pub pid_execution_counts: std::collections::HashMap<String, u32>,
    /// Last response for each PID (for subscription_complete message)
    pub pid_responses: std::collections::HashMap<String, (String, f64)>, // (data, timestamp)
    /// Whether this subscription is run-once only (no continuous PIDs)
    pub is_run_once_only: bool,
    /// Optional timeout override in ms (None = use default from config)
    pub timeout_ms: Option<u32>,
    /// Refresh rate tier per PID (controls polling frequency)
    pub pid_tiers: HashMap<String, RefreshTier>,
}

/// Subscription information for API responses
#[derive(Debug, Clone)]
pub struct SubscriptionInfo {
    pub id: Uuid,
    pub name: Option<String>,
    pub pids: Vec<String>,
    pub target_controller: Option<String>,
    pub state: SubscriptionState,
    pub created_at: f64,
    pub last_active: Option<f64>,
}

impl From<&Subscription> for SubscriptionInfo {
    fn from(sub: &Subscription) -> Self {
        Self {
            id: sub.id,
            name: sub.name.clone(),
            pids: sub.pids.iter().cloned().collect(),
            target_controller: sub.target_controller.clone(),
            state: sub.state.clone(),
            created_at: sub.created_at,
            last_active: sub.last_active,
        }
    }
}

/// Audit log entry for PID add/remove operations on subscriptions.
#[derive(Debug, Clone, serde::Serialize)]
pub struct SubscriptionAuditEntry {
    /// "add" or "remove"
    pub action: String,
    /// Subscription ID
    pub subscription_id: String,
    /// Human-readable subscription name
    pub subscription_name: Option<String>,
    /// PIDs affected
    pub pids: Vec<String>,
    /// Target controller (if any)
    pub target_controller: Option<String>,
    /// Unix timestamp
    pub timestamp: f64,
}

/// Callback fired on every subscription change, receiving the audit entry.
pub type SubscriptionChangeCallback = Box<dyn Fn(&SubscriptionAuditEntry) + Send>;

/// Result from `command_completed()` with subscription completion and mismatch info.
#[derive(Debug, Clone)]
pub struct CommandCompletedResult {
    /// If a run-once subscription just completed, its ID
    pub completed_subscription: Option<Uuid>,
    /// Whether the response PID (from 41 XX bytes) didn't match the sent PID
    pub pid_mismatch: bool,
    /// The PID parsed from the response bytes (None if unparseable or AT command)
    pub response_pid: Option<String>,
}

/// Subscription manager
pub struct SubscriptionManager {
    /// Active subscriptions
    subscriptions: HashMap<Uuid, Subscription>,
    /// Global PID registry for deduplication
    pid_registry: Arc<Mutex<GlobalPIDRegistry>>,
    /// Maximum allowed subscriptions
    max_subscriptions: usize,
    /// PIDs currently in-flight (sent but not yet received response)
    in_flight_pids: std::collections::HashSet<String>,
    /// DS0/DV3: pids a live adapter-side stream serves while polling keeps
    /// running (`concurrent_rx` links) — `peek_next_eligible` skips them.
    streamed_pids: std::collections::HashSet<String>,
    /// Per-PID virtual timeline ticks for weighted fair scheduling.
    /// Lower tick = more overdue for polling. After polling, tick advances by the tier divisor.
    pid_schedule_ticks: HashMap<String, u64>,
    /// Count of responses where the PID in the response bytes (41 XX) didn't match
    /// the PID that was sent. Indicates BLE fragmentation or adapter buffering issues.
    pub response_mismatch_count: u64,
    /// Total responses processed (for mismatch rate calculation)
    pub response_total_count: u64,
    /// Audit log of PID add/remove operations
    audit_log: std::collections::VecDeque<SubscriptionAuditEntry>,
    /// Maximum audit log entries
    max_audit_entries: usize,
    /// Callback fired on every subscription change (add/remove PIDs, start/pause/cancel).
    /// Receives the audit entry as argument.
    subscription_change_callback: Option<SubscriptionChangeCallback>,
}

impl SubscriptionManager {
    /// Create a new subscription manager
    pub fn new(pid_registry: Arc<Mutex<GlobalPIDRegistry>>, max_subscriptions: usize) -> Self {
        Self {
            subscriptions: HashMap::new(),
            pid_registry,
            max_subscriptions,
            in_flight_pids: std::collections::HashSet::new(),
            streamed_pids: std::collections::HashSet::new(),
            pid_schedule_ticks: HashMap::new(),
            response_mismatch_count: 0,
            response_total_count: 0,
            audit_log: std::collections::VecDeque::with_capacity(200),
            max_audit_entries: 200,
            subscription_change_callback: None,
        }
    }

    /// Create a new subscription with an optional human-readable name.
    pub fn create_subscription(&mut self, name: Option<String>) -> Result<Uuid, SubscriptionError> {
        if self.subscriptions.len() >= self.max_subscriptions {
            return Err(SubscriptionError::AlreadyExists);
        }

        let id = Uuid::new_v4();
        let created_at = Self::now_secs();

        let subscription = Subscription {
            id,
            name,
            pids: std::collections::HashSet::new(),
            target_controller: None,
            state: SubscriptionState::Paused, // Start paused, activate with add_pids
            host_paused: false,
            created_at,
            last_active: None,
            pid_run_counts: std::collections::HashMap::new(),
            pid_execution_counts: std::collections::HashMap::new(),
            pid_responses: std::collections::HashMap::new(),
            is_run_once_only: false,
            timeout_ms: None,
            pid_tiers: default_pid_tiers(),
        };

        self.subscriptions.insert(id, subscription);
        Ok(id)
    }

    /// Set timeout for a subscription
    pub fn set_subscription_timeout(
        &mut self,
        subscription_id: Uuid,
        timeout_ms: u32,
    ) -> Result<(), SubscriptionError> {
        let subscription = self
            .subscriptions
            .get_mut(&subscription_id)
            .ok_or(SubscriptionError::SubscriptionNotFound(subscription_id))?;
        subscription.timeout_ms = Some(timeout_ms);
        Ok(())
    }

    /// Get timeout for a subscription (returns None to use default)
    pub fn get_subscription_timeout(&self, subscription_id: Uuid) -> Option<u32> {
        self.subscriptions
            .get(&subscription_id)
            .and_then(|s| s.timeout_ms)
    }

    /// Add PIDs to a subscription
    pub fn add_pids_to_subscription(
        &mut self,
        subscription_id: Uuid,
        pids: Vec<String>,
        target_controller: Option<String>,
    ) -> Result<(), SubscriptionError> {
        self.add_pids_to_subscription_with_run_counts(
            subscription_id,
            pids,
            target_controller,
            None,
        )
    }

    /// Add PIDs to subscription with run count control
    /// run_counts: None = continuous, Some(count) = run specified number of times
    pub fn add_pids_to_subscription_with_run_counts(
        &mut self,
        subscription_id: Uuid,
        pids: Vec<String>,
        target_controller: Option<String>,
        run_counts: Option<std::collections::HashMap<String, Option<u32>>>,
    ) -> Result<(), SubscriptionError> {
        // Seed added PIDs at the schedule's current floor (min tick across PIDs active in any
        // subscription): the new PID ties with the most-overdue PID and is polled within a slot
        // or two — a freshly dragged gauge fills in immediately — without the sustained monopoly
        // tick-0 seeding caused, and without waiting a whole rotation as max-tick seeding did.
        // (Computed before the mutable borrow below.)
        let seed_tick = {
            let active: std::collections::HashSet<&String> = self
                .subscriptions
                .values()
                .flat_map(|s| s.pids.iter())
                .collect();
            self.pid_schedule_ticks
                .iter()
                .filter(|(p, _)| active.contains(p))
                .map(|(_, t)| *t)
                .min()
                .unwrap_or(0)
        };

        let subscription = self
            .subscriptions
            .get_mut(&subscription_id)
            .ok_or(SubscriptionError::SubscriptionNotFound(subscription_id))?;

        if subscription.state == SubscriptionState::Cancelled {
            return Err(SubscriptionError::NotActive);
        }

        // Validate PIDs (basic format check)
        for pid in &pids {
            if !Self::is_valid_pid_static(pid) {
                return Err(SubscriptionError::InvalidPID(pid.clone()));
            }
        }

        // Add PIDs to subscription with run counts
        let mut all_have_run_count = true;

        for pid in &pids {
            subscription.pids.insert(pid.clone());

            // Set run count (default to continuous if not specified)
            let run_count = run_counts
                .as_ref()
                .and_then(|counts| counts.get(pid))
                .copied()
                .flatten(); // None = continuous

            subscription.pid_run_counts.insert(pid.clone(), run_count);
            subscription.pid_execution_counts.insert(pid.clone(), 0);

            // Seed unconditionally: a removed-then-re-added PID must not inherit its stale old
            // tick (far below the pack → it would monopolize polling just like tick-0 seeding).
            self.pid_schedule_ticks.insert(pid.clone(), seed_tick);

            // Track if any PIDs are continuous
            if run_count.is_none() {
                all_have_run_count = false;
            }
        }

        // Mark subscription as run-once-only if ALL PIDs have run counts
        if all_have_run_count && !pids.is_empty() {
            subscription.is_run_once_only = true;
        }

        // Set target controller if provided
        if let Some(ref controller) = target_controller {
            subscription.target_controller = Some(controller.clone());
        }

        // Register with PID registry
        {
            let mut registry = self.pid_registry.lock().unwrap();
            for pid in &pids {
                registry.register_pid_subscription(pid, subscription_id, target_controller.clone());
            }
        }

        self.touch_and_audit(subscription_id, "add", pids);
        Ok(())
    }

    /// Remove PIDs from a subscription
    pub fn remove_pids_from_subscription(
        &mut self,
        subscription_id: Uuid,
        pids: Vec<String>,
    ) -> Result<(), SubscriptionError> {
        let subscription = self
            .subscriptions
            .get_mut(&subscription_id)
            .ok_or(SubscriptionError::SubscriptionNotFound(subscription_id))?;

        if subscription.state == SubscriptionState::Cancelled {
            return Err(SubscriptionError::NotActive);
        }

        // Remove PIDs from subscription and registry
        {
            let mut registry = self.pid_registry.lock().unwrap();
            for pid in &pids {
                subscription.pids.remove(pid);
                registry.unregister_pid_subscription(pid, &subscription_id);
            }
        }

        self.touch_and_audit(subscription_id, "remove", pids);
        Ok(())
    }

    /// Start a subscription (begin polling)
    pub fn start_subscription(&mut self, subscription_id: Uuid) -> Result<(), SubscriptionError> {
        let subscription = self
            .subscriptions
            .get_mut(&subscription_id)
            .ok_or(SubscriptionError::SubscriptionNotFound(subscription_id))?;

        // Explicit host start always clears the host-pause mark, whatever
        // state the subscription is in.
        subscription.host_paused = false;
        match subscription.state {
            SubscriptionState::Active => return Ok(()), // Already active
            SubscriptionState::Cancelled => return Err(SubscriptionError::NotActive),
            SubscriptionState::Completed => return Err(SubscriptionError::NotActive), // Can't restart completed
            SubscriptionState::Error(_) => return Err(SubscriptionError::NotActive),
            SubscriptionState::Paused => {
                subscription.state = SubscriptionState::Active;
                subscription.last_active = Some(Self::now_secs());
                Ok(())
            }
        }
    }

    /// Pause a subscription (stop polling)
    pub fn pause_subscription(&mut self, subscription_id: Uuid) -> Result<(), SubscriptionError> {
        let subscription = self
            .subscriptions
            .get_mut(&subscription_id)
            .ok_or(SubscriptionError::SubscriptionNotFound(subscription_id))?;

        if subscription.state == SubscriptionState::Cancelled {
            return Err(SubscriptionError::NotActive);
        }

        subscription.state = SubscriptionState::Paused;
        subscription.host_paused = true;
        Ok(())
    }

    /// Pause every Active subscription, returning the ids that were paused
    /// (plan M7: sink_start silences polling; sink_stop resumes exactly these).
    pub fn pause_all_active(&mut self) -> Vec<Uuid> {
        let mut paused = Vec::new();
        for (id, sub) in self.subscriptions.iter_mut() {
            if sub.state == SubscriptionState::Active {
                sub.state = SubscriptionState::Paused;
                paused.push(*id);
            }
        }
        paused
    }

    /// Resume a specific set of previously paused subscriptions — the
    /// acquisition-side resume (stream/SP teardown, sink_stop). Skips
    /// host-paused subscriptions: the host paused them for its own reasons
    /// (dashboard hidden, user pause) and owns their resume.
    pub fn resume_many(&mut self, ids: &[Uuid]) {
        for id in ids {
            if let Some(sub) = self.subscriptions.get_mut(id) {
                if sub.state == SubscriptionState::Paused && !sub.host_paused {
                    sub.state = SubscriptionState::Active;
                }
            }
        }
    }

    /// Cancel a subscription permanently
    pub fn cancel_subscription(&mut self, subscription_id: Uuid) -> Result<(), SubscriptionError> {
        let _subscription = self
            .subscriptions
            .remove(&subscription_id)
            .ok_or(SubscriptionError::SubscriptionNotFound(subscription_id))?;

        // Unregister all PIDs from global registry
        let mut registry = self.pid_registry.lock().unwrap();
        registry.unregister_subscription(&subscription_id);

        Ok(())
    }

    /// Get subscription information
    pub fn get_subscription_info(&self, subscription_id: Uuid) -> Option<SubscriptionInfo> {
        self.subscriptions
            .get(&subscription_id)
            .map(|sub| sub.into())
    }

    /// List all subscriptions
    pub fn list_subscriptions(&self) -> Vec<SubscriptionInfo> {
        self.subscriptions.values().map(|sub| sub.into()).collect()
    }

    /// Get active subscriptions (those that should be polled)
    pub fn get_active_subscriptions(&self) -> Vec<&Subscription> {
        self.subscriptions
            .values()
            .filter(|sub| sub.state == SubscriptionState::Active)
            .collect()
    }

    /// Check if a run-once-only subscription is complete (all PIDs executed)
    /// If complete, marks it as Completed and returns the subscription_id
    /// Returns Some(subscription_id) if just completed, None otherwise
    pub fn check_and_mark_subscription_complete(&mut self) -> Option<Uuid> {
        for (id, subscription) in self.subscriptions.iter_mut() {
            if subscription.is_run_once_only
                && subscription.state == SubscriptionState::Active
                && subscription.pids.is_empty()
            {
                // All PIDs have been removed (executed their run count)
                // Mark as completed BEFORE returning (stops polling immediately)
                subscription.state = SubscriptionState::Completed;
                return Some(*id);
            }
        }
        None
    }

    /// Get the collected responses for a completed subscription
    /// Returns Vec of (pid, data, timestamp) tuples, sorted by timestamp (order received)
    pub fn get_subscription_responses(&self, subscription_id: Uuid) -> Vec<(String, String, f64)> {
        if let Some(subscription) = self.subscriptions.get(&subscription_id) {
            let mut responses: Vec<_> = subscription
                .pid_responses
                .iter()
                .map(|(pid, (data, ts))| (pid.clone(), data.clone(), *ts))
                .collect();
            // Sort by timestamp to preserve order received
            responses.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));
            responses
        } else {
            Vec::new()
        }
    }

    /// Check if subscription is run-once only
    pub fn is_run_once_only(&self, subscription_id: Uuid) -> bool {
        self.subscriptions
            .get(&subscription_id)
            .map(|s| s.is_run_once_only)
            .unwrap_or(false)
    }

    /// Validate PID format (basic check)
    fn is_valid_pid_static(cmd: &str) -> bool {
        let cmd_upper = cmd.to_uppercase();
        let cmd_trimmed = cmd_upper.trim();

        // OBDLink batched command (STN §16): pipe-separated sub-commands in one
        // prompt, e.g. "010C|010D|ATSH 7E0|22F40C". Valid iff every segment is.
        if cmd_trimmed.contains('|') {
            return cmd_trimmed
                .split('|')
                .all(|seg| !seg.trim().is_empty() && Self::is_valid_pid_static(seg));
        }

        // AT commands (ELM327): ATZ, ATE0, ATH1, ATSP 0, ATL0, ATS0, etc.
        if cmd_trimmed.starts_with("AT") {
            return cmd_trimmed.len() >= 3; // At least "ATx"
        }

        // ST commands (STN chips): STDI, STFAC, etc.
        if cmd_trimmed.starts_with("ST") {
            return cmd_trimmed.len() >= 3; // At least "STx"
        }

        // OBD PID, optionally followed by an ELM "expected responses" digit
        // (e.g. "010C 1" = return after 1 response). Split that off first.
        let head = match cmd_trimmed.split_once(' ') {
            Some((h, tail)) if tail.len() <= 2 && tail.chars().all(|c| c.is_ascii_digit()) => h,
            Some(_) => return false, // a space with a non-count tail isn't a PID
            None => cmd_trimmed,
        };

        // OBD requests: even-length hex, 2–26 chars.
        // 2:  mode only ("01", "09")
        // 4:  mode + PID ("010C")
        // 6:  Mode 22 DID ("220104")
        // 8–14: J1979 multi-PID ("010C0D05" … "010C0D050F1104" = 6 PIDs)
        // up to 26: Mode 22 multi-DID ("22F40CF40D…" = 6 DIDs)
        if head.len() < 2 || head.len() > 26 || head.len() % 2 != 0 {
            return false;
        }

        head.chars().all(|c| c.is_ascii_hexdigit())
    }

    /// Seconds since UNIX epoch as f64 (shared timestamp helper).
    fn now_secs() -> f64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs_f64()
    }

    /// Snapshot of unique (pid, controller) pairs from the registry
    /// (lock, copy, drop — never hold the registry lock across a scan).
    fn unique_pids(&self) -> Vec<(String, Option<String>)> {
        self.pid_registry.lock().unwrap().get_unique_pids()
    }

    /// First ACTIVE subscription containing `pid` — the owner for
    /// scheduling/eligibility purposes.
    fn owning_active_sub(&self, pid: &str) -> Option<&Subscription> {
        self.subscriptions
            .values()
            .find(|sub| sub.pids.contains(pid) && sub.state == SubscriptionState::Active)
    }

    /// Whether `pid` has exhausted its run limit in `sub`
    /// (continuous pids never do).
    fn run_limit_reached(sub: &Subscription, pid: &str) -> bool {
        match sub.pid_run_counts.get(pid) {
            Some(Some(limit)) => sub.pid_execution_counts.get(pid).unwrap_or(&0) >= limit,
            _ => false,
        }
    }

    /// Shared add/remove tail: touch `last_active` and push the audit entry.
    fn touch_and_audit(&mut self, subscription_id: Uuid, action: &str, pids: Vec<String>) {
        let now = Self::now_secs();
        let (sub_name, target_ctrl) = match self.subscriptions.get_mut(&subscription_id) {
            Some(sub) => {
                sub.last_active = Some(now);
                (sub.name.clone(), sub.target_controller.clone())
            }
            None => return,
        };
        self.push_audit(SubscriptionAuditEntry {
            action: action.to_string(),
            subscription_id: subscription_id.to_string(),
            subscription_name: sub_name,
            pids,
            target_controller: target_ctrl,
            timestamp: now,
        });
    }

    // ==================== EVENT-DRIVEN POLLING API ====================
    // These methods support the event-driven architecture where each command
    // completion triggers the next command, rather than polling on a timer.

    /// Get the next command to send (event-driven API)
    ///
    /// Returns (pid, controller, timeout_ms) for the next command to send.
    /// Marks the PID as in-flight to prevent duplicate sends.
    /// Uses round-robin across subscriptions for fairness.
    /// Returns None if no commands are available.
    pub fn get_next_command(&mut self) -> Option<(String, Option<String>, Option<u32>)> {
        let (pid, controller, timeout, _tick) = self.peek_next_eligible()?;
        self.commit_pick(&pid);
        Some((pid, controller, timeout))
    }

    /// Multi-pick variant for pipelined exchanges (BP1): up to `max` due picks,
    /// most-overdue first, all SHARING the first pick's target controller (one
    /// exchange = one header). Same eligibility/tier scheduling as
    /// `get_next_command` — pipes carry more of the due list per round-trip,
    /// they never promote a not-yet-due PID past its tier.
    ///
    /// One exchange = one step of the tier virtual clock: the first (most
    /// overdue) pick's tick T defines the sweep, and ONLY pids sitting at T
    /// join it. Without this cutoff every pid is always "the next minimum"
    /// eventually, so a 12-slot pipe would drain Fast/Medium/Slow alike into
    /// every sweep and collapse the tier ratios to 1:1 (seen on the CX:
    /// `010C|010D|010F|0105` every exchange). With it, Fast pids advance T by
    /// 1 per sweep, so Medium (+3) joins every 3rd sweep and Slow (+10) every
    /// 10th — exactly the serial tier semantics, just more per round-trip.
    pub fn get_next_commands(
        &mut self,
        max: usize,
        admission: &crate::acquisition::Admission,
    ) -> Vec<(String, Option<String>, Option<u32>)> {
        let mut picks: Vec<(String, Option<String>, Option<u32>)> = Vec::new();
        let mut picked: Vec<crate::acquisition::DuePick> = Vec::new();
        let mut target: Option<Option<String>> = None;
        let mut sweep_tick: Option<u64> = None;
        for _ in 0..max.max(1) {
            // Re-run the single-pick scan each round: in-flight marking from
            // the previous pick excludes it, so this walks the due list in
            // eligibility order without duplicating the scheduler.
            let candidate = self.peek_next_eligible();
            match candidate {
                Some((pid, controller, timeout, tick)) => {
                    match sweep_tick {
                        None => sweep_tick = Some(tick),
                        Some(t) if tick > t => break, // not due this sweep
                        Some(_) => {}
                    }
                    match &target {
                        None => target = Some(controller.clone()),
                        Some(t) if *t == controller => {}
                        Some(_) => break, // different target — next exchange's problem
                    }
                    // Plugin admission (B1): would this pick still fit the
                    // exchange (pipe segments × chunk capacity)? A refused
                    // pick stays due — it rides the NEXT exchange.
                    let due = crate::acquisition::DuePick {
                        pid: pid.clone(),
                        controller: controller.clone(),
                        timeout_ms: timeout,
                    };
                    if !picked.is_empty() && !admission.admit(&picked, &due) {
                        break;
                    }
                    self.commit_pick(&pid);
                    picked.push(due);
                    picks.push((pid, controller, timeout));
                }
                None => break,
            }
        }
        picks
    }

    /// The most-overdue eligible (pid, controller, timeout, tick) WITHOUT
    /// committing it — shared scan for the single- and multi-pick paths.
    fn peek_next_eligible(&mut self) -> Option<(String, Option<String>, Option<u32>, u64)> {
        let all_pids = self.unique_pids();

        let mut best: Option<(String, Option<String>, Option<u32>, u64)> = None;
        for (pid, controller) in &all_pids {
            if self.in_flight_pids.contains(pid) {
                continue;
            }
            // DS0/DV3: served by a live adapter-side stream on a concurrent
            // link — the stream delivers it, polling skips it.
            if self.streamed_pids.contains(pid) {
                continue;
            }
            if let Some(subscription) = self.owning_active_sub(pid) {
                if !Self::run_limit_reached(subscription, pid) {
                    let tick = self.pid_schedule_ticks.get(pid).copied().unwrap_or(0);
                    if best.as_ref().map_or(true, |(_, _, _, t)| tick < *t) {
                        best = Some((
                            pid.clone(),
                            controller.clone(),
                            subscription.timeout_ms,
                            tick,
                        ));
                    }
                }
            }
        }
        best
    }

    /// BP2/#17: projected polling load and fast-gauge rate for the CURRENT
    /// active selection under the live acquisition policy.
    ///
    /// Load L = F + M/3 + S/10 fast-equivalent SIGNALS per sweep cycle (a
    /// Fast pid rides every sweep, Medium every 3rd, Slow every 10th).
    /// Chunk axis (#17): pids the admission would chunk share wire segments
    /// — segment load = ceil(chunkable_load / chunk_limit) + solo_load,
    /// because a chunk costs the SAME flat RTT as a single request (measured:
    /// `01050C0D0F` ≈ `010C` ≈ 89 ms on the CX+Malibu). Pipe axis: a sweep
    /// of s segments costs RTT + (s−1)·SEG ms; averaged per fast cycle
    /// exchanges = max(1, segs/pipe_limit),
    /// T = exchanges·RTT + (segs−exchanges)·SEG, projected fast Hz = 1000/T.
    /// Run-once pids (console one-shots) are transient and excluded. This is
    /// a MODEL — Session Stats shows it beside the measured rate;
    /// measured ≪ projected flags a sick session.
    pub fn acquisition_projection(&self, admission: &crate::acquisition::Admission) -> (f64, f64) {
        self.acquisition_projection_with_candidate(admission, None, None)
    }

    /// Projection using the session's MEASURED per-exchange latency instead
    /// of the pessimistic 149 ms model constant — so the displayed rate
    /// tracks the actual adapter (a 64 ms CX reads ~11 Hz, not the model's
    /// 6.7). Falls back to the model when no measurement exists yet.
    pub fn acquisition_projection_measured(
        &self,
        admission: &crate::acquisition::Admission,
        measured_rtt_ms: Option<f64>,
    ) -> (f64, f64) {
        self.acquisition_projection_with_candidate(admission, None, measured_rtt_ms)
    }

    /// Same model, optionally simulating ONE extra pid in the selection —
    /// the quality-floor gate (visible addition #2) asks "what would the
    /// rate be WITH this gauge?". The candidate's tier resolves through the
    /// standard layers (curated map + F4xx inherit); its chunkability is the
    /// STEADY STATE (plain Mode 01 on a chunk-capable session chunks after
    /// one observation — gating on the transient solo poll would refuse
    /// gauges that are fine two sweeps later).
    pub fn acquisition_projection_with_candidate(
        &self,
        admission: &crate::acquisition::Admission,
        candidate: Option<&str>,
        measured_rtt_ms: Option<f64>,
    ) -> (f64, f64) {
        let all_pids = self.unique_pids();

        let mut load = 0.0_f64; // signals per fast cycle
        let mut chunkable_load = 0.0_f64; // Mode 01 chunks (B1)
        let mut chunkable22_load = 0.0_f64; // Mode 22 multi-DID (B3)
        let mut solo_load = 0.0_f64;
        for (pid, _controller) in &all_pids {
            if let Some(subscription) = self.owning_active_sub(pid) {
                // Run-limited pids are one-shots, not steady-state load.
                let run_once = matches!(subscription.pid_run_counts.get(pid), Some(Some(_)));
                if !run_once {
                    let divisor = resolve_tier(pid, &subscription.pid_tiers).divisor() as f64;
                    load += 1.0 / divisor;
                    match admission.chunk_class(pid) {
                        Some(crate::acquisition::ChunkClass::Mode01) => {
                            chunkable_load += 1.0 / divisor
                        }
                        Some(crate::acquisition::ChunkClass::Mode22) => {
                            chunkable22_load += 1.0 / divisor
                        }
                        None => solo_load += 1.0 / divisor,
                    }
                }
            }
        }
        if let Some(pid) = candidate {
            let divisor = resolve_tier(pid, &default_pid_tiers()).divisor() as f64;
            load += 1.0 / divisor;
            // Steady state: on a capable session the candidate chunks after
            // one solo observation.
            if admission.chunk_limit > 1 && crate::acquisition::is_plain_mode01_data_pid(pid) {
                chunkable_load += 1.0 / divisor;
            } else if admission.chunk22_limit > 1
                && crate::acquisition::is_plain_mode22_data_pid(pid)
            {
                chunkable22_load += 1.0 / divisor;
            } else {
                solo_load += 1.0 / divisor;
            }
        }
        if load <= 0.0 {
            return (0.0, 0.0);
        }
        let ceil_segments = |l: f64, limit: usize| {
            if l > 0.0 {
                (l / limit.max(1) as f64).ceil()
            } else {
                0.0
            }
        };
        let segs = ceil_segments(chunkable_load, admission.chunk_limit)
            + ceil_segments(chunkable22_load, admission.chunk22_limit)
            + solo_load;
        let exchanges = (segs / admission.max_segments.max(1) as f64).max(1.0);
        // Prefer the session's REAL measured latency (a sane sample only —
        // ignore absurd values from a stalled first exchange); scale the
        // pipe-segment term by the same ratio so the pipe/RTT relationship
        // holds on a faster adapter.
        let model_rtt = crate::acquisition::MEASURED_RTT_MS;
        let rtt = measured_rtt_ms
            .filter(|ms| *ms >= 5.0 && *ms <= 2000.0)
            .unwrap_or(model_rtt);
        let seg = crate::acquisition::MEASURED_PIPE_SEGMENT_MS * (rtt / model_rtt);
        let t_ms = exchanges * rtt + (segs - exchanges).max(0.0) * seg;
        (load, 1000.0 / t_ms)
    }

    /// Tier S: the steady-state signal set — pids of ACTIVE subscriptions
    /// without run limits (one-shots are transient; they ride the keepalive
    /// gaps, never the stream defines).
    pub fn active_continuous_pids(&self) -> Vec<(String, Option<String>)> {
        let mut out = Vec::new();
        for sub in self.subscriptions.values() {
            if sub.state != SubscriptionState::Active {
                continue;
            }
            for pid in &sub.pids {
                let run_once = matches!(sub.pid_run_counts.get(pid), Some(Some(_)));
                if !run_once {
                    out.push((pid.clone(), sub.target_controller.clone()));
                }
            }
        }
        out
    }

    /// Commit a pick: advance its tier tick and mark it in-flight.
    fn commit_pick(&mut self, pid: &str) {
        // Schedule next poll: current tick + divisor (fast=1, medium=3, slow=10)
        let tier_divisor = self
            .subscriptions
            .values()
            .find(|sub| sub.pids.contains(pid))
            .map(|sub| resolve_tier(pid, &sub.pid_tiers))
            .unwrap_or(RefreshTier::Medium)
            .divisor() as u64;

        let current_tick = self.pid_schedule_ticks.get(pid).copied().unwrap_or(0);
        self.pid_schedule_ticks
            .insert(pid.to_string(), current_tick + tier_divisor);

        // Mark as in-flight
        self.in_flight_pids.insert(pid.to_string());
    }

    /// Extract the PID from a raw OBD response string.
    /// Parses the service response byte to determine which PID actually responded,
    /// rather than trusting the command that was sent.
    ///
    /// Supports:
    /// - Mode 01 responses: `41 XX`       -> `01XX`
    /// - Mode 09 responses: `49 XX`       -> `09XX`
    /// - Mode 22 responses: `62 XX YY`    -> `22XXYY`
    /// - Mode 24 responses: `64 XX YY`    -> `24XXYY`
    ///
    /// Returns the PID string (e.g., "010C", "22DF71") or None if not parseable.
    pub fn extract_pid_from_response(response: &str) -> Option<String> {
        // Response formats:
        //   "7E8 04 41 0C 1A F8"            — Mode 01 single-frame
        //   "7E8 05 62 DF 71 00 6F"         — Mode 22 single-frame
        //   "41 0C 1A F8"                   — no controller prefix
        //   "7E8 10 0B 41 78 ..."           — ISO-TP multi-frame
        let tokens: Vec<&str> = response.split_whitespace().collect();
        for i in 0..tokens.len().saturating_sub(1) {
            let byte = tokens[i];
            match byte {
                // Mode 01 response (41 XX) and Mode 09 response (49 XX): 1-byte PID
                "41" | "49" => {
                    let mode = if byte == "41" { "01" } else { "09" };
                    let pid_byte = tokens[i + 1];
                    if pid_byte.len() == 2 && pid_byte.chars().all(|c| c.is_ascii_hexdigit()) {
                        return Some(format!("{}{}", mode, pid_byte).to_uppercase());
                    }
                }
                // Mode 22 response (62 XX YY) and Mode 24 response (64 XX YY): 2-byte PID
                "62" | "64" => {
                    let mode = if byte == "62" { "22" } else { "24" };
                    if i + 2 < tokens.len() {
                        let pid_hi = tokens[i + 1];
                        let pid_lo = tokens[i + 2];
                        if pid_hi.len() == 2
                            && pid_hi.chars().all(|c| c.is_ascii_hexdigit())
                            && pid_lo.len() == 2
                            && pid_lo.chars().all(|c| c.is_ascii_hexdigit())
                        {
                            return Some(format!("{}{}{}", mode, pid_hi, pid_lo).to_uppercase());
                        }
                    }
                }
                _ => {}
            }
        }
        None
    }

    /// Called when a command completes (success or failure)
    ///
    /// Removes the PID from in-flight tracking, increments execution count,
    /// records the response, and checks for subscription completion.
    ///
    /// Validates that the response PID (from "41 XX" bytes) matches the
    /// command PID that was sent. If they don't match, routes to the correct PID.
    /// Returns a `CommandCompletedResult` with subscription completion and mismatch info.
    pub fn command_completed(
        &mut self,
        pid: &str,
        response: Option<String>,
        timestamp: f64,
    ) -> CommandCompletedResult {
        // Cross-check response PID against sent PID.
        // If the response contains a "41 XX" that doesn't match, use the response PID instead.
        // Track mismatch rate to surface BLE fragmentation / adapter buffering issues.
        //
        // Only applies when the SENT command's mode is one the extractor understands
        // (01/09/22/24) AND the extracted PID is from the same mode. Payload bytes of
        // other modes routinely contain false markers — e.g. a Mode 06 multi-frame
        // response carrying literal `41 00` data bytes would otherwise re-attribute the
        // completion to "0100", leaving the sent PID's run count stuck and a run-once
        // subscription polling it forever.
        let sent_mode = pid.get(0..2).unwrap_or("");
        // The cross-check exists for BLE fragmentation on PLAIN single-PID
        // requests — anything compound completes under its FULL sent string,
        // because its response legitimately opens with a pid echo that differs
        // from the sent string, and re-attribution would account the completion
        // to a pid no subscription contains → the batch re-polls forever.
        // Compound = piped batch ("010C|010D"), J1979 multi-PID ("010C0D05"),
        // or an expected-responses suffix ("010C 1").
        let plain_single_len = match sent_mode {
            "01" | "09" => pid.len() == 4, // mode + one PID byte
            "22" | "24" => pid.len() == 6, // mode + one 2-byte DID
            _ => false,
        };
        let cross_checkable = !pid.contains('|') && !pid.contains(' ') && plain_single_len;
        let (actual_pid, pid_mismatch, response_pid_parsed) = if let Some(ref data) = response {
            match Self::extract_pid_from_response(data) {
                Some(response_pid)
                    if cross_checkable && response_pid.get(0..2) == Some(sent_mode) =>
                {
                    self.response_total_count += 1;
                    if response_pid != pid {
                        self.response_mismatch_count += 1;
                        let rp = response_pid.clone();
                        (response_pid, true, Some(rp))
                    } else {
                        (pid.to_string(), false, Some(response_pid))
                    }
                }
                _ => (pid.to_string(), false, None),
            }
        } else {
            (pid.to_string(), false, None)
        };

        // Remove the SENT pid from in-flight (that's the one we're done waiting for)
        self.in_flight_pids.remove(pid);

        // Account the completion to EVERY active subscription that contains this PID.
        // A PID is polled once on the wire but can be held by several subscriptions at
        // once (e.g. a run-once batch export overlapping the continuous dashboard) — the
        // data already fans out to all of them, so the execution/run-count accounting
        // must too. Matching only the first (old behavior) left the run-once
        // subscription's pids set forever non-empty and it never completed.
        // FIX: Use actual_pid (validated from response bytes) instead of blindly trusting sent PID
        for (sub_id, subscription) in self.subscriptions.iter_mut() {
            // Only match subscriptions where the PID is still active
            if subscription.pids.contains(&actual_pid)
                && subscription.state == SubscriptionState::Active
            {
                // Increment execution count
                let current_count = subscription
                    .pid_execution_counts
                    .entry(actual_pid.clone())
                    .or_insert(0);
                *current_count += 1;
                let current = *current_count;

                // Record response if provided
                if let Some(ref data) = response {
                    subscription
                        .pid_responses
                        .insert(actual_pid.clone(), (data.clone(), timestamp));
                }

                // Check if we've reached the limit for this PID
                if let Some(max_runs) = subscription.pid_run_counts.get(&actual_pid) {
                    if let Some(limit) = max_runs {
                        if current >= *limit {
                            // Remove PID from subscription completely
                            subscription.pids.remove(&actual_pid);
                            let mut registry = self.pid_registry.lock().unwrap();
                            registry.unregister_pid_subscription(&actual_pid, sub_id);
                        }
                    }
                }

                // Clean up run_counts for completed PIDs (prevents matching old subscriptions)
                if !subscription.pids.contains(&actual_pid) {
                    subscription.pid_run_counts.remove(&actual_pid);
                    subscription.pid_execution_counts.remove(&actual_pid);
                }
            }
        }

        // Check if any run-once subscription is complete
        CommandCompletedResult {
            completed_subscription: self.check_and_mark_subscription_complete(),
            pid_mismatch,
            response_pid: response_pid_parsed,
        }
    }

    /// Check if any commands are currently in-flight
    pub fn has_in_flight_commands(&self) -> bool {
        !self.in_flight_pids.is_empty()
    }

    /// Get count of in-flight commands
    pub fn in_flight_count(&self) -> usize {
        self.in_flight_pids.len()
    }

    /// Clear all in-flight tracking (used on disconnect/reset)
    pub fn clear_in_flight(&mut self) {
        self.in_flight_pids.clear();
    }

    /// Get the refresh tier name for a PID (e.g., "Fast", "Medium", "Slow").
    /// Returns None if the PID isn't in any active subscription.
    pub fn get_pid_tier(&self, pid: &str) -> Option<String> {
        for subscription in self.subscriptions.values() {
            if subscription.pids.contains(pid) {
                let tier = resolve_tier(pid, &subscription.pid_tiers);
                return Some(format!("{:?}", tier));
            }
        }
        None
    }

    /// Log an external event to the audit log (e.g., vehicle discovery).
    /// Use this to record non-subscription activity so it shows up in
    /// `obd_get_subscription_audit_log` and the change callback.
    pub fn log_audit_event(&mut self, action: &str, name: Option<String>, pids: Vec<String>) {
        let now = Self::now_secs();
        self.push_audit(SubscriptionAuditEntry {
            action: action.to_string(),
            subscription_id: "system".to_string(),
            subscription_name: name,
            pids,
            target_controller: None,
            timestamp: now,
        });
    }

    /// Push an audit log entry, evicting oldest if at capacity.
    /// Fires the subscription change callback if installed.
    fn push_audit(&mut self, entry: SubscriptionAuditEntry) {
        if let Some(ref cb) = self.subscription_change_callback {
            cb(&entry);
        }
        if self.audit_log.len() >= self.max_audit_entries {
            self.audit_log.pop_front();
        }
        self.audit_log.push_back(entry);
    }

    /// Install a callback that fires on every subscription change.
    pub fn set_subscription_change_callback(&mut self, callback: SubscriptionChangeCallback) {
        self.subscription_change_callback = Some(callback);
    }

    /// Remove the subscription change callback.
    pub fn remove_subscription_change_callback(&mut self) {
        self.subscription_change_callback = None;
    }

    /// Get the audit log of PID add/remove operations.
    /// Returns most recent `count` entries (newest first).
    pub fn get_audit_log(&self, count: usize) -> Vec<&SubscriptionAuditEntry> {
        self.audit_log.iter().rev().take(count).collect()
    }

    /// Get a snapshot of all subscriptions with their PIDs and tiers.
    /// Returns a JSON-serializable structure.
    pub fn get_subscriptions_snapshot(&self) -> Vec<serde_json::Value> {
        self.subscriptions
            .values()
            .map(|sub| {
                let pids_with_tiers: Vec<serde_json::Value> = sub.pids.iter().map(|pid| {
                let tier = resolve_tier(pid, &sub.pid_tiers);
                serde_json::json!({
                    "pid": pid,
                    "tier": format!("{:?}", tier),
                    "run_count": sub.pid_run_counts.get(pid).copied().flatten(),
                    "execution_count": sub.pid_execution_counts.get(pid).copied().unwrap_or(0),
                    "in_flight": self.in_flight_pids.contains(pid),
                })
            }).collect();

                serde_json::json!({
                    "id": sub.id.to_string(),
                    "subscription_name": sub.name,
                    "state": format!("{:?}", sub.state),
                    "target_controller": sub.target_controller,
                    "created_at": sub.created_at,
                    "last_active": sub.last_active,
                    "pid_count": sub.pids.len(),
                    "pids": pids_with_tiers,
                    "is_run_once_only": sub.is_run_once_only,
                })
            })
            .collect()
    }

    /// Get response mismatch metrics.
    /// Returns (mismatch_count, total_count, mismatch_rate).
    /// A non-zero mismatch rate indicates BLE fragmentation or adapter buffering issues.
    pub fn response_mismatch_metrics(&self) -> (u64, u64, f64) {
        let rate = if self.response_total_count > 0 {
            self.response_mismatch_count as f64 / self.response_total_count as f64
        } else {
            0.0
        };
        (
            self.response_mismatch_count,
            self.response_total_count,
            rate,
        )
    }

    /// Set refresh rate tiers for PIDs in a subscription.
    /// Tiers map PID strings to RefreshTier values.
    /// PIDs not in the map keep their current tier (defaults apply).
    /// DS0/DV3: the pids a live stream serves (empty = none).
    pub fn set_streamed_pids(&mut self, pids: std::collections::HashSet<String>) {
        self.streamed_pids = pids;
    }

    pub fn set_pid_tiers(
        &mut self,
        subscription_id: Uuid,
        tiers: HashMap<String, RefreshTier>,
    ) -> Result<(), SubscriptionError> {
        let subscription = self
            .subscriptions
            .get_mut(&subscription_id)
            .ok_or(SubscriptionError::SubscriptionNotFound(subscription_id))?;
        // Merge provided tiers on top of existing (defaults stay for unspecified PIDs)
        for (pid, tier) in tiers {
            subscription.pid_tiers.insert(pid, tier);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    #[test]
    fn test_create_subscription() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();
        assert!(manager.get_subscription_info(id).is_some());

        let info = manager.get_subscription_info(id).unwrap();
        assert_eq!(info.state, SubscriptionState::Paused);
        assert!(info.pids.is_empty());
    }

    #[test]
    fn test_add_pids_to_subscription() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();

        // Add PIDs
        manager
            .add_pids_to_subscription(
                id,
                vec!["010C".to_string(), "010D".to_string()],
                Some("7E0".to_string()),
            )
            .unwrap();

        let info = manager.get_subscription_info(id).unwrap();
        assert_eq!(info.pids.len(), 2);
        assert!(info.pids.contains(&"010C".to_string()));
        assert!(info.pids.contains(&"010D".to_string()));
        assert_eq!(info.target_controller, Some("7E0".to_string()));
    }

    #[test]
    fn test_subscription_lifecycle() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();

        // Start subscription
        manager.start_subscription(id).unwrap();
        assert_eq!(
            manager.get_subscription_info(id).unwrap().state,
            SubscriptionState::Active
        );

        // Pause subscription
        manager.pause_subscription(id).unwrap();
        assert_eq!(
            manager.get_subscription_info(id).unwrap().state,
            SubscriptionState::Paused
        );

        // Cancel subscription
        manager.cancel_subscription(id).unwrap();
        assert!(manager.get_subscription_info(id).is_none());
    }

    #[test]
    fn test_invalid_pid() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();

        // Try to add invalid PID
        let result = manager.add_pids_to_subscription(id, vec!["INVALID".to_string()], None);
        assert!(matches!(result, Err(SubscriptionError::InvalidPID(_))));
    }

    #[test]
    fn test_valid_pid_accepts_batches_and_response_count() {
        // Plain PIDs / AT / ST still valid.
        assert!(SubscriptionManager::is_valid_pid_static("010C"));
        assert!(SubscriptionManager::is_valid_pid_static("ATSP 0"));
        // Expected-responses suffix (ELM "return after N").
        assert!(SubscriptionManager::is_valid_pid_static("010C 1"));
        assert!(SubscriptionManager::is_valid_pid_static("22F40C 1"));
        // OBDLink pipe batch — each segment valid.
        assert!(SubscriptionManager::is_valid_pid_static("010C|010D|0105"));
        assert!(SubscriptionManager::is_valid_pid_static(
            "ATSH 7E0|22F40C 1|221E35 1"
        ));
        // J1979 multi-PID requests (alone and inside a pipe batch).
        assert!(SubscriptionManager::is_valid_pid_static("010C0D05"));
        assert!(SubscriptionManager::is_valid_pid_static("010C0D050F1104"));
        assert!(SubscriptionManager::is_valid_pid_static(
            "010C0D050F1104|011F"
        ));
        assert!(!SubscriptionManager::is_valid_pid_static("010C0")); // odd length
                                                                     // Still rejects genuine garbage.
        assert!(!SubscriptionManager::is_valid_pid_static("INVALID"));
        assert!(!SubscriptionManager::is_valid_pid_static("010C XYZ")); // space, non-count tail
        assert!(!SubscriptionManager::is_valid_pid_static("010C||010D")); // empty segment
        assert!(!SubscriptionManager::is_valid_pid_static("010C|ZZZZ")); // bad segment
    }

    /// Pipe-shaped admission for engine tests: n segments, no chunking.
    fn pipes(n: usize) -> crate::acquisition::Admission {
        crate::acquisition::Admission {
            max_segments: n,
            chunk_limit: 1,
            chunk22_limit: 1,
            length_cache: None,
            chunk_excluded: std::collections::HashSet::new(),
        }
    }

    /// BP1 multi-pick: returns up to K due picks, most-overdue first, marks
    /// each in-flight (so a re-scan can't duplicate), and a later call picks
    /// up the remainder.
    #[test]
    fn test_get_next_commands_multi_pick() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);
        let id = manager.create_subscription(None).unwrap();
        manager
            .add_pids_to_subscription(id, vec!["010C".into(), "010D".into(), "0105".into()], None)
            .unwrap();
        manager.start_subscription(id).unwrap();

        let picks = manager.get_next_commands(2, &pipes(2));
        assert_eq!(picks.len(), 2, "respects the pick budget");
        let picked: std::collections::HashSet<String> =
            picks.iter().map(|(p, _, _)| p.clone()).collect();
        assert_eq!(picked.len(), 2, "no duplicate picks");

        // The remaining pid arrives on the next call; the two in-flight ones
        // are excluded.
        let rest = manager.get_next_commands(3, &pipes(3));
        assert_eq!(rest.len(), 1);
        assert!(
            !picked.contains(&rest[0].0),
            "in-flight pids must be excluded"
        );
    }

    /// BP1 regression (seen live on the CX): a 12-slot pipe budget must NOT
    /// drain every tier into every sweep — only pids at the sweep's virtual
    /// tick join. Fast rides every sweep, Medium every 3rd, Slow every 10th.
    #[test]
    fn test_get_next_commands_preserves_tier_ratios() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);
        let id = manager.create_subscription(None).unwrap();
        // Default tiers: 010C Fast, 010F Medium (unmapped), 0105 Slow.
        manager
            .add_pids_to_subscription(id, vec!["010C".into(), "010F".into(), "0105".into()], None)
            .unwrap();
        manager.start_subscription(id).unwrap();

        let mut counts: std::collections::HashMap<String, u32> = std::collections::HashMap::new();
        for _ in 0..30 {
            let picks = manager.get_next_commands(12, &pipes(12));
            assert!(!picks.is_empty());
            assert!(
                picks.iter().any(|(p, _, _)| p == "010C"),
                "fast pid rides every sweep"
            );
            for (pid, _, _) in &picks {
                *counts.entry(pid.clone()).or_insert(0) += 1;
            }
            manager.clear_in_flight(); // exchange completed
        }
        assert_eq!(counts.get("010C"), Some(&30), "fast: every sweep");
        assert_eq!(counts.get("010F"), Some(&10), "medium: every 3rd sweep");
        assert_eq!(counts.get("0105"), Some(&3), "slow: every 10th sweep");
    }

    /// #18 layered tier resolution: explicit entry > F4xx inherit > Medium.
    #[test]
    fn test_resolve_tier_layers() {
        let tiers = default_pid_tiers();
        // Layer 1: curated map.
        assert_eq!(resolve_tier("010C", &tiers), RefreshTier::Fast);
        assert_eq!(resolve_tier("0105", &tiers), RefreshTier::Slow);
        // Layer 2: F4xx mirror inherits its Mode 01 twin.
        assert_eq!(resolve_tier("22F40C", &tiers), RefreshTier::Fast, "UDS RPM");
        assert_eq!(
            resolve_tier("22F40D", &tiers),
            RefreshTier::Fast,
            "UDS speed"
        );
        assert_eq!(
            resolve_tier("22F405", &tiers),
            RefreshTier::Slow,
            "UDS coolant"
        );
        assert_eq!(
            resolve_tier("22f40c", &tiers),
            RefreshTier::Fast,
            "case-blind"
        );
        // Unmapped twin → Medium; non-F4xx Mode 22 → Medium.
        assert_eq!(resolve_tier("22F4AB", &tiers), RefreshTier::Medium);
        assert_eq!(resolve_tier("221E35", &tiers), RefreshTier::Medium);
        // Precedence: an explicit override on the F4xx pid itself wins.
        let mut with_override = tiers.clone();
        with_override.insert("22F40C".to_string(), RefreshTier::Slow);
        assert_eq!(resolve_tier("22F40C", &with_override), RefreshTier::Slow);
    }

    /// BP2 projection math: load = F + M/3 + S/10; piped sweep costs
    /// RTT + (L−1)·SEG, serial costs L·RTT (model constants — asserted via
    /// the constants so re-tuning RTT doesn't break the SHAPE these pin).
    #[test]
    fn test_acquisition_projection_pipe_on_off() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);
        let id = manager.create_subscription(None).unwrap();
        // Default tiers: 010C/010D Fast, 010F Medium (unmapped), 0105 Slow.
        manager
            .add_pids_to_subscription(
                id,
                vec!["010C".into(), "010D".into(), "010F".into(), "0105".into()],
                None,
            )
            .unwrap();
        manager.start_subscription(id).unwrap();

        let expected_load = 2.0 + 1.0 / 3.0 + 1.0 / 10.0; // 2.4333

        use crate::acquisition::{MEASURED_PIPE_SEGMENT_MS as SEG, MEASURED_RTT_MS as RTT};
        let (load, hz_piped) = manager.acquisition_projection(&pipes(12));
        assert!((load - expected_load).abs() < 1e-9);
        // One exchange fits: T = RTT + (L−1)·SEG.
        let t_piped = RTT + (expected_load - 1.0) * SEG;
        assert!(
            (hz_piped - 1000.0 / t_piped).abs() < 0.01,
            "piped {hz_piped}"
        );

        let (load_serial, hz_serial) = manager.acquisition_projection(&pipes(1));
        assert!((load_serial - expected_load).abs() < 1e-9);
        // Serial: T = L·RTT.
        assert!(
            (hz_serial - 1000.0 / (expected_load * RTT)).abs() < 0.01,
            "serial {hz_serial}"
        );
        assert!(hz_piped > hz_serial);
    }

    /// #17 chunk-aware projection: chunkable pids share ONE flat-cost
    /// segment (the Malibu shape → one RTT per sweep); unconfirmed lengths
    /// fall back to the solo model; a chunkable overflow adds segments.
    #[test]
    fn test_acquisition_projection_chunk_aware() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);
        let id = manager.create_subscription(None).unwrap();
        // Malibu shape: 010C/010D Fast, 010F Medium, 0105 Slow.
        manager
            .add_pids_to_subscription(
                id,
                vec!["010C".into(), "010D".into(), "010F".into(), "0105".into()],
                None,
            )
            .unwrap();
        manager.start_subscription(id).unwrap();

        let cache = Arc::new(Mutex::new(crate::length_cache::LengthCache::new()));
        let chunked =
            |cache: &Arc<Mutex<crate::length_cache::LengthCache>>| crate::acquisition::Admission {
                max_segments: 12,
                chunk_limit: 6,
                chunk22_limit: 1,
                length_cache: Some(Arc::clone(cache)),
                chunk_excluded: std::collections::HashSet::new(),
            };

        use crate::acquisition::{MEASURED_PIPE_SEGMENT_MS as SEG, MEASURED_RTT_MS as RTT};
        let expected_load = 2.0 + 1.0 / 3.0 + 1.0 / 10.0;
        // No lengths learned yet → nothing chunks → identical to pipe model.
        let (_, hz_unconfirmed) = manager.acquisition_projection(&chunked(&cache));
        let t_pipe = RTT + (expected_load - 1.0) * SEG;
        assert!(
            (hz_unconfirmed - 1000.0 / t_pipe).abs() < 0.01,
            "{hz_unconfirmed}"
        );

        // All confirmed → one flat segment per sweep: T = one RTT (the
        // hardware-verified Malibu shape: a chunk costs a single round-trip).
        for (pid, len) in [("010C", 2), ("010D", 1), ("010F", 1), ("0105", 1)] {
            cache.lock().unwrap().record(pid, len);
        }
        let (load, hz_chunked) = manager.acquisition_projection(&chunked(&cache));
        assert!((load - expected_load).abs() < 1e-9);
        assert!((hz_chunked - 1000.0 / RTT).abs() < 0.01, "{hz_chunked}");

        // Excluding a member moves it to the solo load: segments = 1 chunk +
        // 1/3 solo (010F Medium) → exch = max(1, 1.3333/12) = 1 →
        // T = RTT + 0.3333·SEG.
        let mut a = chunked(&cache);
        a.chunk_excluded.insert("010F".to_string());
        let (_, hz_excluded) = manager.acquisition_projection(&a);
        let t_excl = RTT + (1.0 / 3.0) * SEG;
        assert!(
            (hz_excluded - 1000.0 / t_excl).abs() < 0.01,
            "{hz_excluded}"
        );
        assert!(hz_excluded < hz_chunked);
    }

    /// #17: more than chunk_limit chunkable fast pids overflow into a second
    /// segment — 8 Fast confirmed @ chunk 6 → 2 segments → RTT + SEG sweeps.
    #[test]
    fn test_acquisition_projection_chunk_overflow_segments() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);
        let id = manager.create_subscription(None).unwrap();
        let pids: Vec<String> = [
            "0104", "0105", "010B", "010C", "010D", "010F", "0110", "0111",
        ]
        .iter()
        .map(|p| p.to_string())
        .collect();
        manager
            .add_pids_to_subscription(id, pids.clone(), None)
            .unwrap();
        // Force everything Fast so the load is exactly 8 signals/sweep.
        let tiers: std::collections::HashMap<String, RefreshTier> = pids
            .iter()
            .map(|p| (p.clone(), RefreshTier::Fast))
            .collect();
        manager.set_pid_tiers(id, tiers).unwrap();
        manager.start_subscription(id).unwrap();

        let cache = Arc::new(Mutex::new(crate::length_cache::LengthCache::new()));
        for p in &pids {
            cache.lock().unwrap().record(p, 1);
        }
        let admission = crate::acquisition::Admission {
            max_segments: 12,
            chunk_limit: 6,
            chunk22_limit: 1,
            length_cache: Some(Arc::clone(&cache)),
            chunk_excluded: std::collections::HashSet::new(),
        };
        let (load, hz) = manager.acquisition_projection(&admission);
        assert!((load - 8.0).abs() < 1e-9);
        // ceil(8/6) = 2 segments → T = RTT + (2−1)·SEG.
        use crate::acquisition::{MEASURED_PIPE_SEGMENT_MS as SEG, MEASURED_RTT_MS as RTT};
        assert!((hz - 1000.0 / (RTT + SEG)).abs() < 0.01, "{hz}");
    }

    /// Run-once pids (console one-shots) are not steady-state load; an empty
    /// selection projects to zero.
    #[test]
    fn test_acquisition_projection_excludes_run_once() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);
        assert_eq!(manager.acquisition_projection(&pipes(12)), (0.0, 0.0));

        let id = manager.create_subscription(None).unwrap();
        let mut run_counts = std::collections::HashMap::new();
        run_counts.insert("0902".to_string(), Some(1u32));
        manager
            .add_pids_to_subscription_with_run_counts(
                id,
                vec!["0902".into()],
                None,
                Some(run_counts),
            )
            .unwrap();
        manager.start_subscription(id).unwrap();
        assert_eq!(manager.acquisition_projection(&pipes(12)), (0.0, 0.0));
    }

    /// B1 demotion routing at the ENGINE level: with chunk admission, the
    /// pick loop stops at the first pick that no longer fits — an
    /// unconfirmed-length pid is refused from the chunk exchange and rides
    /// the NEXT exchange solo (observe-then-batch, no flags anywhere).
    #[test]
    fn test_get_next_commands_chunk_admission_routes_unconfirmed_solo() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);
        let id = manager.create_subscription(None).unwrap();
        manager
            .add_pids_to_subscription(id, vec!["010C".into(), "010D".into()], None)
            .unwrap();
        manager.start_subscription(id).unwrap();

        let cache = Arc::new(Mutex::new(crate::length_cache::LengthCache::new()));
        cache.lock().unwrap().record("010C", 2);
        cache.lock().unwrap().record("010D", 1);
        let chunk_admission = crate::acquisition::Admission {
            max_segments: 1,
            chunk_limit: 6,
            chunk22_limit: 1,
            length_cache: Some(Arc::clone(&cache)),
            chunk_excluded: std::collections::HashSet::new(),
        };

        // Both confirmed → both picked for one chunk exchange.
        let picks = manager.get_next_commands(6, &chunk_admission);
        assert_eq!(picks.len(), 2);
        manager.clear_in_flight();

        // Invalidate 010D (demotion) → it no longer joins; 010C chunks alone
        // this exchange, 010D solos the next one.
        cache.lock().unwrap().invalidate("010D");
        let picks = manager.get_next_commands(6, &chunk_admission);
        assert_eq!(picks.len(), 1, "mixed chunk/solo can't share one segment");
        let picks2 = manager.get_next_commands(6, &chunk_admission);
        assert_eq!(picks2.len(), 1);
        let got: std::collections::HashSet<String> =
            [picks[0].0.clone(), picks2[0].0.clone()].into();
        assert_eq!(got.len(), 2, "both pids ride, in separate exchanges");
    }

    /// One exchange = one target: picks stop at a target-controller boundary.
    #[test]
    fn test_get_next_commands_same_target_only() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);
        let a = manager.create_subscription(None).unwrap();
        manager
            .add_pids_to_subscription(a, vec!["22F40C".into()], Some("7E0".into()))
            .unwrap();
        manager.start_subscription(a).unwrap();
        let b = manager.create_subscription(None).unwrap();
        manager
            .add_pids_to_subscription(b, vec!["221E35".into()], Some("7E9".into()))
            .unwrap();
        manager.start_subscription(b).unwrap();

        let picks = manager.get_next_commands(4, &pipes(4));
        assert_eq!(picks.len(), 1, "second pick targets a different controller");
        let picks2 = manager.get_next_commands(4, &pipes(4));
        assert_eq!(picks2.len(), 1, "other target rides the NEXT exchange");
        assert_ne!(picks[0].1, picks2[0].1);
    }

    /// A piped batch completes under its FULL sent string: the response's
    /// leading `41 0C` echo must NOT re-attribute completion to "010C" (which
    /// isn't in the subscription) — that left run-once batches polling forever.
    #[test]
    fn test_piped_batch_completes_run_once() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let batch = "010C|010D|0105".to_string();
        let id = manager.create_subscription(None).unwrap();
        let mut run_counts = std::collections::HashMap::new();
        run_counts.insert(batch.clone(), Some(1u32));
        manager
            .add_pids_to_subscription_with_run_counts(
                id,
                vec![batch.clone()],
                None,
                Some(run_counts),
            )
            .unwrap();
        manager.start_subscription(id).unwrap();

        let result = manager.command_completed(
            &batch,
            Some("41 0C 1A F8|41 0D 00|41 05 5A".to_string()),
            0.0,
        );
        assert!(
            !result.pid_mismatch,
            "piped batch must not be cross-checked"
        );
        assert_eq!(
            result.completed_subscription,
            Some(id),
            "run-once piped batch must complete its subscription"
        );
    }

    /// Same re-attribution loop, other compound forms: a J1979 multi-PID
    /// request ("010C0D") and a count-suffixed request ("010C 1") both answer
    /// with a leading "41 0C" echo — the cross-check must not steal their
    /// completion (accounted to "010C", which no subscription holds → forever
    /// re-poll, seen live on the CX bench 2026-07-30).
    #[test]
    fn test_compound_requests_complete_run_once() {
        for (sent, response) in [("010C0D", "41 0C 1A F8 0D 42"), ("010C 1", "41 0C 1A F8")] {
            let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
            let mut manager = SubscriptionManager::new(registry, 10);
            let id = manager.create_subscription(None).unwrap();
            let mut run_counts = std::collections::HashMap::new();
            run_counts.insert(sent.to_string(), Some(1u32));
            manager
                .add_pids_to_subscription_with_run_counts(
                    id,
                    vec![sent.to_string()],
                    None,
                    Some(run_counts),
                )
                .unwrap();
            manager.start_subscription(id).unwrap();

            let result = manager.command_completed(sent, Some(response.to_string()), 0.0);
            assert!(!result.pid_mismatch, "{sent}: must not be cross-checked");
            assert_eq!(
                result.completed_subscription,
                Some(id),
                "{sent}: run-once must complete"
            );
        }
    }

    #[test]
    fn test_subscription_not_found() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let fake_id = Uuid::new_v4();

        assert!(matches!(
            manager.add_pids_to_subscription(fake_id, vec!["010C".to_string()], None),
            Err(SubscriptionError::SubscriptionNotFound(_))
        ));
    }

    #[test]
    fn test_remove_pids() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();
        manager
            .add_pids_to_subscription(id, vec!["010C".to_string(), "010D".to_string()], None)
            .unwrap();

        // Remove one PID
        manager
            .remove_pids_from_subscription(id, vec!["010C".to_string()])
            .unwrap();

        let info = manager.get_subscription_info(id).unwrap();
        assert_eq!(info.pids.len(), 1);
        assert!(info.pids.contains(&"010D".to_string()));
        assert!(!info.pids.contains(&"010C".to_string()));
    }

    #[test]
    fn test_max_subscriptions() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 2);

        // Create max subscriptions
        let _id1 = manager.create_subscription(None).unwrap();
        let _id2 = manager.create_subscription(None).unwrap();

        // Third should fail
        assert!(matches!(
            manager.create_subscription(None),
            Err(SubscriptionError::AlreadyExists)
        ));
    }

    #[test]
    fn test_event_driven_run_once_subscription() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();

        // Add PIDs that ALL run once (pure run-once subscription)
        let mut run_counts = std::collections::HashMap::new();
        run_counts.insert("0906".to_string(), Some(1u32)); // Calibration data
        run_counts.insert("090A".to_string(), Some(1u32)); // VIN

        manager
            .add_pids_to_subscription_with_run_counts(
                id,
                vec!["0906".to_string(), "090A".to_string()],
                None,
                Some(run_counts),
            )
            .unwrap();

        // Should be marked as run-once-only
        assert!(
            manager.is_run_once_only(id),
            "Subscription should be run-once-only"
        );

        // Start subscription
        manager.start_subscription(id).unwrap();

        // Get first command (event-driven)
        let cmd1 = manager.get_next_command();
        assert!(cmd1.is_some(), "Should get first command");
        let (pid1, _, _) = cmd1.unwrap();

        // Complete first command - should NOT trigger subscription_complete yet
        let completed = manager.command_completed(&pid1, Some("response1".to_string()), 1.0);
        assert!(
            completed.completed_subscription.is_none(),
            "Subscription should not complete after first PID"
        );

        // Get second command
        let cmd2 = manager.get_next_command();
        assert!(cmd2.is_some(), "Should get second command");
        let (pid2, _, _) = cmd2.unwrap();

        // Complete second command - should trigger subscription_complete
        let completed = manager.command_completed(&pid2, Some("response2".to_string()), 2.0);
        assert_eq!(
            completed.completed_subscription,
            Some(id),
            "Subscription should complete after all PIDs done"
        );

        // No more commands should be available
        let cmd3 = manager.get_next_command();
        assert!(cmd3.is_none(), "No more commands after completion");

        // Verify subscription state
        let info = manager.get_subscription_info(id).unwrap();
        assert_eq!(info.state, SubscriptionState::Completed);

        // Verify responses were collected
        let responses = manager.get_subscription_responses(id);
        assert_eq!(responses.len(), 2, "Should have both responses");
    }

    #[test]
    fn test_event_driven_continuous_subscription() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();

        // Add continuous PIDs (no run counts)
        manager
            .add_pids_to_subscription(id, vec!["010C".to_string(), "010D".to_string()], None)
            .unwrap();

        // Should NOT be run-once-only
        assert!(
            !manager.is_run_once_only(id),
            "Subscription should be continuous"
        );

        // Start subscription
        manager.start_subscription(id).unwrap();

        // Get and complete multiple commands - never triggers subscription_complete
        for _ in 0..5 {
            let cmd = manager.get_next_command();
            assert!(
                cmd.is_some(),
                "Should always have commands for continuous subscription"
            );
            let (pid, _, _) = cmd.unwrap();

            let completed = manager.command_completed(&pid, Some("response".to_string()), 1.0);
            assert!(
                completed.completed_subscription.is_none(),
                "Continuous subscription never completes"
            );
        }

        // Subscription should still be active
        let info = manager.get_subscription_info(id).unwrap();
        assert_eq!(info.state, SubscriptionState::Active);
    }

    /// Regression: a run-once subscription must complete even when its PIDs overlap an
    /// active continuous subscription (e.g. the car-mock export batch requesting Mode 01
    /// PIDs the live dashboard is already polling). Completion accounting used to stop at
    /// the first subscription containing the PID, so the run-once set never emptied and
    /// subscription_complete never fired.
    #[test]
    fn test_run_once_completes_while_overlapping_continuous_subscription() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        // Continuous "dashboard" subscription holding 010C
        let dash = manager.create_subscription(None).unwrap();
        manager
            .add_pids_to_subscription(dash, vec!["010C".to_string()], None)
            .unwrap();
        manager.start_subscription(dash).unwrap();

        // Run-once "export" subscription sharing 010C, plus its own 010D
        let export = manager.create_subscription(None).unwrap();
        let mut run_counts = std::collections::HashMap::new();
        run_counts.insert("010C".to_string(), Some(1u32));
        run_counts.insert("010D".to_string(), Some(1u32));
        manager
            .add_pids_to_subscription_with_run_counts(
                export,
                vec!["010C".to_string(), "010D".to_string()],
                None,
                Some(run_counts),
            )
            .unwrap();
        assert!(
            manager.is_run_once_only(export),
            "export subscription should be run-once-only"
        );
        manager.start_subscription(export).unwrap();

        // Drive the event loop; the export must complete within a bounded number of cycles
        let mut completed = None;
        for i in 0..20 {
            let Some((pid, _, _)) = manager.get_next_command() else {
                break;
            };
            let response = format!("7E8 03 41 {} 50", &pid[2..4]);
            let result = manager.command_completed(&pid, Some(response), i as f64);
            if let Some(id) = result.completed_subscription {
                completed = Some(id);
                break;
            }
        }
        assert_eq!(completed, Some(export),
            "run-once subscription must complete even when its PIDs overlap a continuous subscription");

        // The export collected responses for BOTH of its PIDs (data fans out to all subscribers)
        let responses = manager.get_subscription_responses(export);
        assert_eq!(
            responses.len(),
            2,
            "export should have responses for both PIDs"
        );

        // The continuous subscription is untouched: still active, still polling the shared PID
        assert_eq!(
            manager.get_subscription_info(dash).unwrap().state,
            SubscriptionState::Active
        );
        let mut saw_shared_pid = false;
        for _ in 0..5 {
            if let Some((pid, _, _)) = manager.get_next_command() {
                if pid == "010C" {
                    saw_shared_pid = true;
                }
                manager.command_completed(&pid, Some("7E8 03 41 0C 50".to_string()), 99.0);
            }
        }
        assert!(
            saw_shared_pid,
            "continuous subscription must keep polling the shared PID after the run-once completes"
        );
    }

    /// Regression: the response-PID cross-check must not misattribute completions when a
    /// non-Mode-01 payload happens to contain bytes that look like a Mode 01 marker.
    /// A real Mode 06 multi-frame response can carry literal `41 00` data bytes; the old
    /// scan re-attributed the completion to "0100", so the sent PID's run count never
    /// incremented and a run-once subscription re-polled it forever (car-mock export hang).
    #[test]
    fn test_mode06_payload_with_false_mode01_marker_completes() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();
        let mut run_counts = std::collections::HashMap::new();
        run_counts.insert("0606".to_string(), Some(1u32));
        manager
            .add_pids_to_subscription_with_run_counts(
                id,
                vec!["0606".to_string()],
                None,
                Some(run_counts),
            )
            .unwrap();
        manager.start_subscription(id).unwrap();

        let (pid, _, _) = manager.get_next_command().expect("0606 should be polled");
        assert_eq!(pid, "0606");

        // Real-shaped Mode 06 multi-frame response whose payload contains "41 00"
        let response = "7E8 10 2E 46 06 07 0B 02 73 7E8 21 00 00 02 87 06 08 0B \
                        7E8 24 FF 06 82 01 00 41 00 7E8 25 00 01 90 06 83 8A 04";
        let result = manager.command_completed(&pid, Some(response.to_string()), 1.0);

        assert!(
            !result.pid_mismatch,
            "Mode 06 payload must not trigger the Mode 01 cross-check"
        );
        assert_eq!(result.completed_subscription, Some(id),
            "run-once must complete: the completion belongs to the sent PID 0606, not a false '0100'");
        assert!(
            manager.get_next_command().is_none(),
            "0606 must not be re-polled"
        );
    }

    /// Benchmark test: measures per-PID polling distribution.
    /// Run with: cargo test test_pid_polling_distribution_benchmark -- --nocapture
    ///
    /// Run BEFORE tiering changes to capture the uniform baseline,
    /// then AFTER to see fast PIDs getting more bandwidth.
    #[test]
    fn test_pid_polling_distribution_benchmark() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();

        // Typical dashboard: mix of fast-changing and slow-changing PIDs
        let pids = vec![
            "010C".to_string(), // RPM          (should be fast)
            "010D".to_string(), // Speed         (should be fast)
            "0111".to_string(), // Throttle      (should be fast)
            "0110".to_string(), // MAF           (should be fast)
            "0104".to_string(), // Engine Load   (should be fast)
            "0105".to_string(), // Coolant Temp  (should be slow)
            "012F".to_string(), // Fuel Level    (should be slow)
            "011F".to_string(), // Runtime       (should be slow)
        ];

        manager.add_pids_to_subscription(id, pids, None).unwrap();
        manager.start_subscription(id).unwrap();

        // Simulate 300 command cycles, tracking timing
        let total_cycles = 300u32;
        let mut poll_counts: std::collections::HashMap<String, u32> =
            std::collections::HashMap::new();
        let mut completion_times: Vec<std::time::Duration> = Vec::new();
        let bench_start = std::time::Instant::now();

        for _ in 0..total_cycles {
            if let Some((pid, _, _)) = manager.get_next_command() {
                let cmd_start = std::time::Instant::now();
                *poll_counts.entry(pid.clone()).or_insert(0) += 1;
                manager.command_completed(&pid, Some("OK".to_string()), 1.0);
                completion_times.push(cmd_start.elapsed());
            }
        }

        let bench_elapsed = bench_start.elapsed();

        // Print results for comparison
        let labels = [
            ("010C", "RPM"),
            ("010D", "Speed"),
            ("0111", "Throttle"),
            ("0110", "MAF"),
            ("0104", "Load"),
            ("0105", "Coolant"),
            ("012F", "Fuel Lvl"),
            ("011F", "Runtime"),
        ];

        println!(
            "\n=== PID Polling Distribution ({} cycles) ===",
            total_cycles
        );
        let mut total_polls = 0u32;
        for (pid, name) in &labels {
            let count = poll_counts.get(*pid).copied().unwrap_or(0);
            total_polls += count;
            println!("  {} ({:>8}): {:>4} polls", pid, name, count);
        }
        println!("  Total polls: {}", total_polls);
        println!("--- Timing ---");
        let cmds_per_sec = total_polls as f64 / bench_elapsed.as_secs_f64();
        let avg_completion = if !completion_times.is_empty() {
            let total: std::time::Duration = completion_times.iter().sum();
            total / completion_times.len() as u32
        } else {
            std::time::Duration::ZERO
        };
        println!("  Wall time:          {:.2?}", bench_elapsed);
        println!("  Commands/sec:       {:.1}", cmds_per_sec);
        println!("  Avg completion:     {:.2?}", avg_completion);
        println!("================================================\n");

        // Sanity check: every cycle should produce a command
        assert_eq!(
            total_polls, total_cycles,
            "Every cycle should produce a poll"
        );
    }

    /// Rate-limited benchmark: simulates real adapter timing at 20 cmd/sec.
    /// Each command has a simulated 50ms round-trip (send + ECU response + adapter processing),
    /// matching typical OBDLink MX+ on CAN behavior.
    ///
    /// Run with: cargo test test_pid_polling_rate_limited_benchmark -- --nocapture
    /// Note: takes ~15 seconds to run (300 commands at 20 cmd/sec).
    #[test]
    fn test_pid_polling_rate_limited_benchmark() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();

        let pids = vec![
            "010C".to_string(), // RPM          (fast)
            "010D".to_string(), // Speed         (fast)
            "0111".to_string(), // Throttle      (fast)
            "0110".to_string(), // MAF           (fast)
            "0104".to_string(), // Engine Load   (fast)
            "0105".to_string(), // Coolant Temp  (slow)
            "012F".to_string(), // Fuel Level    (slow)
            "011F".to_string(), // Runtime       (slow)
        ];

        manager.add_pids_to_subscription(id, pids, None).unwrap();
        manager.start_subscription(id).unwrap();

        let target_cmd_per_sec: f64 = 20.0;
        let simulated_response_time = std::time::Duration::from_millis(10); // ECU response time
        let inter_command_delay = std::time::Duration::from_secs_f64(1.0 / target_cmd_per_sec);
        let run_duration = std::time::Duration::from_secs(10);

        let mut poll_counts: std::collections::HashMap<String, u32> =
            std::collections::HashMap::new();
        let mut completion_times: Vec<std::time::Duration> = Vec::new();
        let bench_start = std::time::Instant::now();
        let mut total_polls = 0u32;

        while bench_start.elapsed() < run_duration {
            // Rate limit: wait between commands
            let cmd_start = std::time::Instant::now();

            if let Some((pid, _, _)) = manager.get_next_command() {
                // Simulate adapter + ECU round-trip
                std::thread::sleep(simulated_response_time);

                manager.command_completed(&pid, Some("OK".to_string()), 1.0);
                *poll_counts.entry(pid.clone()).or_insert(0) += 1;
                total_polls += 1;
                completion_times.push(cmd_start.elapsed());
            }

            // Enforce rate limit — sleep remaining time in this command slot
            let elapsed = cmd_start.elapsed();
            if elapsed < inter_command_delay {
                std::thread::sleep(inter_command_delay - elapsed);
            }
        }

        let bench_elapsed = bench_start.elapsed();

        let labels = [
            ("010C", "RPM"),
            ("010D", "Speed"),
            ("0111", "Throttle"),
            ("0110", "MAF"),
            ("0104", "Load"),
            ("0105", "Coolant"),
            ("012F", "Fuel Lvl"),
            ("011F", "Runtime"),
        ];

        println!(
            "\n=== Rate-Limited Benchmark ({:.0} cmd/sec target, {:.1}s run) ===",
            target_cmd_per_sec,
            run_duration.as_secs_f64()
        );
        for (pid, name) in &labels {
            let count = poll_counts.get(*pid).copied().unwrap_or(0);
            let hz = count as f64 / bench_elapsed.as_secs_f64();
            println!(
                "  {} ({:>8}): {:>4} polls  ({:.1} Hz)",
                pid, name, count, hz
            );
        }

        let actual_cmds_per_sec = total_polls as f64 / bench_elapsed.as_secs_f64();
        let avg_completion = if !completion_times.is_empty() {
            let total: std::time::Duration = completion_times.iter().sum();
            total / completion_times.len() as u32
        } else {
            std::time::Duration::ZERO
        };

        println!("--- Totals ---");
        println!("  Total polls:        {}", total_polls);
        println!("  Wall time:          {:.2?}", bench_elapsed);
        println!("  Actual cmd/sec:     {:.1}", actual_cmds_per_sec);
        println!(
            "  Avg completion:     {:.2?} (includes {}ms simulated response)",
            avg_completion,
            simulated_response_time.as_millis()
        );
        println!(
            "  Per-PID Hz (fast):  {:.1}",
            poll_counts.get("010C").copied().unwrap_or(0) as f64 / bench_elapsed.as_secs_f64()
        );
        println!(
            "  Per-PID Hz (slow):  {:.1}",
            poll_counts.get("0105").copied().unwrap_or(0) as f64 / bench_elapsed.as_secs_f64()
        );
        println!("================================================================\n");

        // Verify we're close to the target rate
        assert!(
            (actual_cmds_per_sec - target_cmd_per_sec).abs() < 5.0,
            "Should be close to {} cmd/sec, got {:.1}",
            target_cmd_per_sec,
            actual_cmds_per_sec
        );
    }

    /// Regression test: dynamically adding a PID to an active subscription must not
    /// cause response desync. This reproduces the bug where:
    ///   1. New PIDs got schedule_tick=0, starving existing PIDs (60x spam)
    ///   2. Responses were tagged with the wrong PID (41 XX mismatch)
    #[test]
    fn test_dynamic_pid_add_does_not_desync() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();

        // Start with 3 PIDs
        let initial_pids = vec![
            "0105".to_string(), // Coolant (slow)
            "010C".to_string(), // RPM (fast)
            "010D".to_string(), // Speed (fast)
        ];
        manager
            .add_pids_to_subscription(id, initial_pids, None)
            .unwrap();
        manager.start_subscription(id).unwrap();

        // Run 20 cycles to establish schedule ticks
        let mut poll_counts: HashMap<String, u32> = HashMap::new();
        for _ in 0..20 {
            if let Some((pid, _, _)) = manager.get_next_command() {
                *poll_counts.entry(pid.clone()).or_insert(0) += 1;
                manager.command_completed(&pid, Some(format!("7E8 03 41 {} 50", &pid[2..4])), 1.0);
            }
        }

        // Record the schedule floor BEFORE adding the new PID
        let min_tick_before = manager
            .pid_schedule_ticks
            .values()
            .copied()
            .min()
            .unwrap_or(0);

        // === DYNAMIC ADD: add PID 0104 mid-cycle ===
        manager
            .add_pids_to_subscription(id, vec!["0104".to_string()], None)
            .unwrap();

        // FIX VERIFICATION 1 (U2): the new PID is seeded at the schedule's current floor —
        // prompt first poll (ties with the most-overdue PID) but NOT tick 0, which would
        // starve existing PIDs until it caught up with the pack.
        let new_pid_tick = manager.pid_schedule_ticks.get("0104").copied().unwrap_or(0);
        assert_eq!(
            new_pid_tick, min_tick_before,
            "New PID should be seeded at the schedule floor ({}), got {}",
            min_tick_before, new_pid_tick
        );
        assert!(
            new_pid_tick > 0,
            "after 20 cycles the floor must be above 0 — seed isn't tick-0"
        );

        // Run 30 more cycles after the dynamic add
        let mut post_add_counts: HashMap<String, u32> = HashMap::new();
        for _ in 0..30 {
            if let Some((pid, _, _)) = manager.get_next_command() {
                *post_add_counts.entry(pid.clone()).or_insert(0) += 1;
                manager.command_completed(&pid, Some(format!("7E8 03 41 {} 50", &pid[2..4])), 1.0);
            }
        }

        // FIX VERIFICATION 2: The new PID should NOT dominate polling
        // (Before the fix, it would get ~all 30 polls due to tick=0)
        let new_pid_polls = post_add_counts.get("0104").copied().unwrap_or(0);
        let total_polls: u32 = post_add_counts.values().sum();
        assert!(
            new_pid_polls < total_polls / 2,
            "New PID 0104 got {} out of {} polls — should not dominate after dynamic add",
            new_pid_polls,
            total_polls
        );

        // FIX VERIFICATION 3: All 4 PIDs should have been polled at least once
        assert!(
            post_add_counts.contains_key("0104"),
            "New PID 0104 should be polled"
        );
        assert!(
            post_add_counts.contains_key("010C"),
            "Existing PID 010C should still be polled"
        );
        assert!(
            post_add_counts.contains_key("010D"),
            "Existing PID 010D should still be polled"
        );
        // 0105 is slow-tier so it may not appear in only 30 cycles — that's OK
    }

    /// Test that response PID validation catches mismatched responses.
    /// Simulates the scenario where a response for PID 010D arrives but was
    /// tagged as PID 010C (the command that was sent).
    #[test]
    fn test_response_pid_validation_catches_mismatch() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();
        manager
            .add_pids_to_subscription(id, vec!["010C".to_string(), "010D".to_string()], None)
            .unwrap();
        manager.start_subscription(id).unwrap();

        // Send command for 010C
        let cmd = manager.get_next_command();
        assert!(cmd.is_some());
        let (sent_pid, _, _) = cmd.unwrap();

        // But the response contains "41 0D" (Speed) — a MISMATCH!
        // This simulates the desync where the adapter returns a response
        // for a different PID than what was sent.
        let mismatched_response = "7E8 03 41 0D 5C"; // 41 0D = Speed, not RPM

        // command_completed should route this to 010D based on response bytes,
        // NOT to whatever PID was sent
        manager.command_completed(&sent_pid, Some(mismatched_response.to_string()), 1.0);

        // Verify that the response was recorded under 010D (the actual PID from response bytes)
        let sub = manager.subscriptions.get(&id).unwrap();
        if let Some((data, _)) = sub.pid_responses.get("010D") {
            assert_eq!(
                data, mismatched_response,
                "Response should be stored under actual PID 010D"
            );
        }
        // The sent PID should NOT have this response
        if sent_pid != "010D" {
            assert!(
                !sub.pid_responses.contains_key(&sent_pid)
                    || sub.pid_responses.get(&sent_pid).map(|(d, _)| d.as_str())
                        != Some(mismatched_response),
                "Mismatched response should NOT be stored under sent PID {}",
                sent_pid
            );
        }
    }

    /// Test the extract_pid_from_response helper with various response formats.
    #[test]
    fn test_extract_pid_from_response() {
        // Standard single-frame response
        assert_eq!(
            SubscriptionManager::extract_pid_from_response("7E8 04 41 0C 1A F8"),
            Some("010C".to_string())
        );
        // No controller prefix
        assert_eq!(
            SubscriptionManager::extract_pid_from_response("41 0D 5C"),
            Some("010D".to_string())
        );
        // Service 09 (VIN request)
        assert_eq!(
            SubscriptionManager::extract_pid_from_response("7E8 10 14 49 02 01 57 44"),
            Some("0902".to_string())
        );
        // ISO-TP multi-frame first frame
        assert_eq!(
            SubscriptionManager::extract_pid_from_response("7E8 10 0B 41 78 0D 0A ED 09"),
            Some("0178".to_string()) // mode 01 PID 78
        );
        // Non-data response
        assert_eq!(
            SubscriptionManager::extract_pid_from_response("NO DATA"),
            None
        );
        // Empty
        assert_eq!(SubscriptionManager::extract_pid_from_response(""), None);
        // Mode 22 extended PID (62 XX YY)
        assert_eq!(
            SubscriptionManager::extract_pid_from_response("7E8 05 62 DF 71 00 6F"),
            Some("22DF71".to_string())
        );
        // Mode 22 no controller prefix
        assert_eq!(
            SubscriptionManager::extract_pid_from_response("62 04 04 00 6F"),
            Some("220404".to_string())
        );
        // Mode 24 (64 XX YY)
        assert_eq!(
            SubscriptionManager::extract_pid_from_response("7E8 06 64 DF 71 80 00 FF"),
            Some("24DF71".to_string())
        );
    }

    /// U2 — a PID added to a live subscription is polled promptly (within a couple of slots),
    /// but does not monopolize the schedule, and a removed-then-re-added PID doesn't inherit a
    /// stale tick.
    /// Field bug 2026-08-06: leaving the dashboard host-paused the
    /// subscription, then the stream teardown's `resume_many` resurrected it
    /// and the ladder re-engaged streaming with nobody on the dashboard. The
    /// acquisition-side resume must skip host-paused subs; an explicit host
    /// start clears the mark.
    #[test]
    fn host_pause_survives_acquisition_resume() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();
        manager
            .add_pids_to_subscription(id, vec!["010C".to_string()], None)
            .unwrap();
        manager.start_subscription(id).unwrap();

        // Stream engage pauses the sub; teardown resumes it — normal cycle.
        let paused = manager.pause_all_active();
        assert_eq!(paused, vec![id]);
        manager.resume_many(&paused);
        assert_eq!(
            manager.get_subscription_info(id).unwrap().state,
            SubscriptionState::Active
        );

        // Host pause (dashboard hidden) → stream engage/teardown cycle must
        // NOT resurrect it.
        manager.pause_subscription(id).unwrap();
        let paused = manager.pause_all_active(); // nothing active now
        assert!(paused.is_empty());
        manager.resume_many(&[id]); // teardown resuming its (stale) list
        assert_eq!(
            manager.get_subscription_info(id).unwrap().state,
            SubscriptionState::Paused,
            "host-paused sub must stay paused through acquisition resume"
        );

        // Explicit host start clears the mark and reactivates.
        manager.start_subscription(id).unwrap();
        assert_eq!(
            manager.get_subscription_info(id).unwrap().state,
            SubscriptionState::Active
        );
        let paused = manager.pause_all_active();
        manager.resume_many(&paused);
        assert_eq!(
            manager.get_subscription_info(id).unwrap().state,
            SubscriptionState::Active
        );
    }

    #[test]
    fn test_late_added_pid_polls_promptly_without_monopoly() {
        let registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(5000)));
        let mut manager = SubscriptionManager::new(registry, 10);

        let id = manager.create_subscription(None).unwrap();
        manager
            .add_pids_to_subscription(id, vec!["010C".to_string(), "010D".to_string()], None)
            .unwrap();
        manager.start_subscription(id).unwrap();

        // Let the dashboard "run" for a while so existing ticks advance well past zero.
        for i in 0..30 {
            let (pid, _, _) = manager.get_next_command().expect("command available");
            manager.command_completed(&pid, Some("ok".to_string()), i as f64);
        }

        // Drag a new gauge on: its PID must be polled within the next 2 slots.
        manager
            .add_pids_to_subscription(id, vec!["0105".to_string()], None)
            .unwrap();
        let mut first_seen_at = None;
        let mut polls: Vec<String> = Vec::new();
        for i in 0..10 {
            let (pid, _, _) = manager.get_next_command().expect("command available");
            manager.command_completed(&pid, Some("ok".to_string()), 100.0 + i as f64);
            if pid == "0105" && first_seen_at.is_none() {
                first_seen_at = Some(i);
            }
            polls.push(pid);
        }
        let first = first_seen_at.expect("added PID was polled");
        assert!(
            first <= 1,
            "added PID should be polled within 2 slots, was at {}",
            first
        );
        let added_share = polls.iter().filter(|p| *p == "0105").count();
        assert!(
            added_share <= 5,
            "added PID must not monopolize the schedule ({}/10)",
            added_share
        );

        // Remove and re-add much later: same promptness, no stale-tick monopoly.
        manager
            .remove_pids_from_subscription(id, vec!["0105".to_string()])
            .unwrap();
        for i in 0..30 {
            let (pid, _, _) = manager.get_next_command().expect("command available");
            manager.command_completed(&pid, Some("ok".to_string()), 200.0 + i as f64);
        }
        manager
            .add_pids_to_subscription(id, vec!["0105".to_string()], None)
            .unwrap();
        let mut polls: Vec<String> = Vec::new();
        for i in 0..10 {
            let (pid, _, _) = manager.get_next_command().expect("command available");
            manager.command_completed(&pid, Some("ok".to_string()), 300.0 + i as f64);
            polls.push(pid);
        }
        assert!(
            polls[..2].iter().any(|p| p == "0105"),
            "re-added PID polled within 2 slots"
        );
        let readd_share = polls.iter().filter(|p| *p == "0105").count();
        assert!(
            readd_share <= 5,
            "re-added PID must not monopolize ({}/10)",
            readd_share
        );
    }
}
