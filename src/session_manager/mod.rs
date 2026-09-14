//! OBD Session Manager - Main orchestrator
//!
//! The central component that manages the background thread, coordinates
//! all services, and provides the synchronous API for Swift integration.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Duration;
use uuid::Uuid;

use crate::command_processor::{CommandProcessor, CommandProcessorBuilder};
use crate::config::OBDSessionConfig;
use crate::error::SessionError;
use crate::json_api::{message_builder, MessageProcessor};
use crate::pid_registry::GlobalPIDRegistry;
use crate::platform::OBDPlatformInterface;
use crate::session_logger::SessionLogger;
use crate::subscription::{SubscriptionInfo, SubscriptionManager};

/// One-line optional session log: expands to the
/// `if let Some(ref log) = <logger> { log.log_callback(key, detail) }`
/// boilerplate. ONLY for sites whose body is that single call.
macro_rules! log_cb {
    ($logger:expr, $key:expr, $detail:expr $(,)?) => {
        if let Some(ref log) = $logger {
            log.log_callback($key, $detail);
        }
    };
}

/// SH1 wiring wrapper: the pure `HealthTracker` plus the command-processor
/// RTT snapshot taken at the last window boundary (running sum/count delta
/// → per-window `rtt_avg_ms`). Lives in `SharedState`; never locked while
/// `shared_state` is being acquired (order is shared_state → health).
struct HealthState {
    tracker: crate::session_health::HealthTracker,
    rtt_total_ms: f64,
    rtt_count: u64,
}

impl HealthState {
    fn new() -> Self {
        Self {
            tracker: crate::session_health::HealthTracker::new(),
            rtt_total_ms: 0.0,
            rtt_count: 0,
        }
    }

    /// Advance the RTT snapshot to `stats` and return the average over the
    /// window that just ended (None when no commands completed inside it).
    fn window_rtt_ms(
        &mut self,
        stats: Option<crate::command_processor::CommandProcessorStats>,
    ) -> Option<f64> {
        let stats = stats?;
        let total_ms = stats.completion_time_total.as_secs_f64() * 1000.0;
        let count = stats.completion_time_count;
        let d_count = count.saturating_sub(self.rtt_count);
        let d_total = total_ms - self.rtt_total_ms;
        self.rtt_total_ms = total_ms;
        self.rtt_count = count;
        if d_count == 0 || d_total <= 0.0 {
            None
        } else {
            Some(d_total / d_count as f64)
        }
    }
}

/// SH1: epoch-millis now — the same time base `SessionLogger` stamps `ts` with.
fn health_now_ms() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// SH1: emit tracker events to the session jsonl (`acquisition_window` /
/// `anomaly` callback lines) and to Swift through the response callback as
/// `{"type":"session_health","payload":{...}}`.
fn emit_health_events(
    events: &[crate::session_health::HealthEvent],
    logger: &Option<Arc<SessionLogger>>,
    callback: &Arc<dyn Fn(String) + Send + Sync>,
) {
    use crate::session_health::HealthEvent;
    for event in events {
        let (key, payload) = match event {
            HealthEvent::WindowClosed(w) => (
                "acquisition_window",
                crate::session_health::window_payload(w),
            ),
            HealthEvent::Anomaly(a) => ("anomaly", crate::session_health::anomaly_payload(a)),
        };
        log_cb!(logger, key, &payload.to_string());
        callback(serde_json::json!({ "type": "session_health", "payload": payload }).to_string());
    }
}

/// SH1: one counter call from the signal-delivery paths (poll data callback,
/// UDS-stream ingest, STN-periodic decode) — count delivered member signals
/// and emit whatever the tracker surfaced (interval anomalies).
fn health_note_signals(
    health: &Arc<Mutex<HealthState>>,
    n: u64,
    logger: &Option<Arc<SessionLogger>>,
    callback: &Arc<dyn Fn(String) + Send + Sync>,
) {
    let events = health
        .lock()
        .unwrap()
        .tracker
        .on_signals(n, health_now_ms());
    if !events.is_empty() {
        emit_health_events(&events, logger, callback);
    }
}

/// SH1: what the health emitters pull out of the shared-state lock
/// (tracker, logger, response callback, command processor).
type HealthParts = (
    Arc<Mutex<HealthState>>,
    Option<Arc<SessionLogger>>,
    Arc<dyn Fn(String) + Send + Sync>,
    Option<Arc<CommandProcessor>>,
);

/// In-flight wire-command → member PIDs per pipe segment (P0/B1): written
/// when the engine sends a plugin-built transaction, consumed by the data
/// callback (segment j of the response ↔ segments[j]).
type InFlightMembers = Arc<Mutex<HashMap<String, Vec<Vec<String>>>>>;

/// Main OBD Session Manager
///
/// This is the primary interface for Swift applications. It spawns and manages
/// a background thread for continuous operations while providing a synchronous
/// API for immediate operations.
pub struct OBDSessionManager {
    /// API handle for synchronous operations
    api_handle: SessionAPIHandle,
    /// Background thread handle
    background_thread: Option<JoinHandle<()>>,
    /// SL3: lifecycle worker thread handle (joined at shutdown).
    lifecycle_thread: Option<JoinHandle<()>>,
    /// Shared state between API and background thread
    shared_state: Arc<Mutex<SharedState>>,
}

/// API Handle for synchronous operations
///
/// This provides thread-safe access to session management operations
/// that can be called from Swift without blocking.
#[derive(Clone)]
pub struct SessionAPIHandle {
    shared_state: Arc<Mutex<SharedState>>,
}

/// Shared state between API thread and background thread
/// Tier S: an active periodic stream's runtime state (S1/S2).
/// SP — active STN adapter-periodic acquisition. Simpler than ActiveStream:
/// the adapter runs the poll loop (STPPMA), so there is NO keepalive beat and
/// NO re-define — once STM is sent, frames just stream in and decode.
/// SP engage outcome — see engage_stn_periodic.
enum PeriodicEngage {
    Live,
    Retry,
    StayOnPoll,
}

/// LH4: the monitor-handshake mechanics live in the link layer now
/// (`MonitorHandshake`); the UDS stream keeps using them under this name.
pub(super) use crate::link::MonitorHandshake as AcquisitionRuntime;

/// SP: the session's record of a live adapter-periodic run (LH4). The wire
/// state — installed handles, decode map, monitor handshake — lives in the
/// handler's `PeriodicEngine`; the session keeps only what is session
/// policy: which poll subscriptions it paused (a user pause/cancel edits this
/// list so the teardown never resumes it).
struct ActivePeriodic {
    /// Poll subscriptions paused for the periodic run; resumed on stop.
    paused_subs: Vec<Uuid>,
    /// The engine running it.
    engine: Arc<dyn crate::link::PeriodicEngine>,
    /// DV3 (concurrent links): pids the slots serve while polling keeps
    /// running — pausing a subscription that owns one of them tears the
    /// plan down, exactly as a paused sub does on a quiesced link.
    served_pids: Vec<String>,
}

struct ActiveStream {
    /// Poll subscriptions paused for the stream; resumed on stop.
    paused_subs: Vec<Uuid>,
    /// CAN ids carrying the periodic frames (filter targets).
    response_ids: Vec<String>,
    /// Keepalive cadence from the profile.
    keepalive_interval_ms: u64,
    keepalive_settle_ms: u64,
    /// The keepalive wire: `STPX H:<target>,D:3E80,R:0` — R:0 returns to the
    /// prompt immediately instead of waiting out the (suppressed) response
    /// timeout. A plain `3E80` left the adapter busy when the `STM` re-arm
    /// arrived 150 ms later; ELM-class chips treat bytes-while-busy as an
    /// interrupt AND DISCARD THEM, so the re-arm became a garbled `TM` and
    /// the monitor never resumed (bench round 10: full-rate gauges for
    /// exactly one beat interval, then silence).
    keepalive_wire: String,
    /// Loop stop / STOPPED handshake / monitor liveness + break-window ack
    /// capture — shared mechanics (RS7). (Beat clears monitor_active during
    /// its break, sets after re-arm; stop skips the break when already at
    /// the prompt instead of paying the 1.5 s STOPPED timeout.)
    runtime: AcquisitionRuntime,
    /// Serializes transport operations between the beat thread and stop —
    /// a beat mid-handshake must finish before stop's own handshake.
    transport_op: Arc<Mutex<()>>,
    /// S3: the LIVE decode map — shared with the ingest so mid-stream signal
    /// adds/removes take effect without restarting the stream.
    unpack: Arc<Mutex<crate::periodic_stream::UnpackMap>>,
    /// S3: signal changes queued for the NEXT keepalive break (the ~1 s
    /// prompt window every beat): adds get a fresh slot defined there;
    /// removes just leave the decode map.
    pending: Arc<Mutex<StreamPending>>,
    /// Wire bytes DEFINED per slot (acked records only — refused ones never
    /// joined). Removes don't decrement: an undefined-on-removal record
    /// stays on the wire, its bytes stay occupied. First-fit adds pack into
    /// the first slot with `fill + len <= 7`.
    slot_fill: Arc<Mutex<std::collections::HashMap<u8, usize>>>,
    /// Profile facts the break-window work needs.
    slot_count: usize,
    stpx_header: String,
    rate_mode: u8,
}

/// Snapshot the keepalive beat thread runs on — everything
/// `arm_stream_transport` clones out of SharedState/ActiveStream under the
/// lock before spawning (was a 17-element tuple).
struct BeatCtx {
    runtime: AcquisitionRuntime,
    transport_op: Arc<Mutex<()>>,
    response_ids: Vec<String>,
    interval_ms: u64,
    settle_ms: u64,
    keepalive_wire: String,
    logger: Option<Arc<SessionLogger>>,
    pending: Arc<Mutex<StreamPending>>,
    unpack: Arc<Mutex<crate::periodic_stream::UnpackMap>>,
    slot_fill: Arc<Mutex<std::collections::HashMap<u8, usize>>>,
    slot_count: usize,
    stpx_header: String,
    rate_mode: u8,
    length_cache: Arc<Mutex<crate::length_cache::LengthCache>>,
    event_callback: Arc<dyn Fn(String) + Send + Sync>,
}

/// S3: queued mid-stream signal changes.
#[derive(Default)]
struct StreamPending {
    adds: Vec<String>,
    removes: Vec<String>,
}

/// What the host needs to RUN a started stream (S2): the transport-level
/// monitor loop — pass filters, `STMA`, line push, and the keepalive/monitor
/// window alternation — is owned by the HOST (it owns the transport; TX from
/// this side would break monitor mode invisibly). Returned by `start_stream`.
#[derive(Debug, Clone, serde::Serialize)]
pub struct StreamStartInfo {
    /// CAN ids carrying the periodic frames (pass-filter these, then STMA).
    pub response_ids: Vec<String>,
    /// Send `3E 80` at this cadence, inside a monitor break (the S3 timeout
    /// on the car is ~5 s — the whole break must fit inside it).
    pub keepalive_interval_ms: u64,
    /// Dynamic-DID slots in use (diagnostics).
    pub slots_used: usize,
    /// Subscribed pids NOT in the stream (define refused / dropped by the
    /// builder) — they sit on the PAUSED poll side while the stream runs,
    /// so their gauges must show "no data", not a stale polled value.
    pub excluded_pids: Vec<String>,
}

/// D7: RAII marker for an acquisition teardown in flight. Increments on
/// creation, decrements on EVERY exit path (drop) — saturating, so a
/// disconnect reset racing a late drop can never underflow/wedge the gate.
struct TeardownGuard(Arc<std::sync::atomic::AtomicUsize>);
impl TeardownGuard {
    fn new(counter: &Arc<std::sync::atomic::AtomicUsize>) -> Self {
        counter.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        Self(Arc::clone(counter))
    }
}
impl Drop for TeardownGuard {
    fn drop(&mut self) {
        let _ = self.0.fetch_update(
            std::sync::atomic::Ordering::SeqCst,
            std::sync::atomic::Ordering::SeqCst,
            |v| Some(v.saturating_sub(1)),
        );
    }
}

struct SharedState {
    /// The transport platform — held so the session can drive stream
    /// engage/teardown ITSELF (S4: streaming behind the subscription API).
    platform: Arc<dyn OBDPlatformInterface>,
    /// S4: vinrules `udsPeriodic` tag, handed over once by the host when the
    /// VIN publishes (vinrules is host-side data). None = poll-only vehicle.
    stream_tag: Mutex<Option<String>>,
    /// DR4: the session-config blob (datasets.json, parsed session slice), handed once
    /// by the host at setup. The session resolves the attached dataset itself at
    /// identify (VIN WMI × detected addressing) into `attached_dataset`.
    dataset_registry: Mutex<Option<crate::dataset_registry::SessionConfig>>,
    /// DR4: id of the dataset resolved for the CURRENT car (cleared implicitly by the
    /// next identify). Consumers: module walk (ED2), $19 read, stream self-select.
    attached_dataset: Mutex<Option<String>>,
    /// S4: monitor_control handshake — the host acks tap attach/detach here
    /// (obd_monitor_ready) so Rust never proceeds against a mis-wired tap.
    monitor_ack: Arc<(Mutex<bool>, std::sync::Condvar)>,
    /// SL3: engage request-dedup + sink stand-down handshake — true while an
    /// `Engage` message is queued or the ladder is running on the worker.
    /// NOT the mode authority (that's `phase`); it exists so N engage
    /// triggers collapse to one queued ladder run, and so `sink_start` can
    /// wait (bounded) for a cancelled ladder to stand down.
    engage_pending: Arc<std::sync::atomic::AtomicBool>,
    /// S4 arbitration: set by sink_start (console sniff / MCP / wizard) —
    /// the engage task stands down instead of fighting for the transport.
    engage_cancel: Arc<std::sync::atomic::AtomicBool>,
    /// Command processor
    command_processor: Option<Arc<CommandProcessor>>,
    /// PID registry
    pid_registry: Arc<Mutex<GlobalPIDRegistry>>,
    /// Subscription manager
    subscription_manager: Arc<Mutex<SubscriptionManager>>,
    /// Session configuration
    config: OBDSessionConfig,
    /// Running flag
    running: bool,
    /// Response callback for JSON API responses
    response_callback: Option<Box<dyn Fn(String) + Send + Sync>>,
    /// Default command timeout
    default_timeout_ms: u32,
    /// Session logger (optional)
    logger: Option<Arc<SessionLogger>>,
    /// True while waiting for user to select a connector (guards polling updates)
    awaiting_selection: bool,
    /// LH4: host-fed adapter facts (adapters.json `maxChunk` /
    /// `supportsPeriodic`, stnperiodic.json tier periods) — a live cell
    /// shared with the link handler, which snapshots it into `AdapterFacts`.
    host_facts: Arc<Mutex<crate::link::HostFacts>>,
    /// LH5: the adapter catalog (adapters.json) the host handed over.
    adapter_catalog: Option<crate::catalog::Catalog>,
    /// User tapped the STN/UDS badge OFF — pin plain polling; the engage
    /// ladder stands down until they tap it back on. Session-transient
    /// (cleared on disconnect), distinct from the persisted Settings gates.
    user_pinned_poll: Arc<std::sync::atomic::AtomicBool>,
    /// D7: acquisition teardowns currently in flight (UDS stop / STN periodic
    /// stop). Incremented when a teardown task starts, decremented when its
    /// wire drain completes. The engage beat DEFERS while nonzero — a fresh
    /// engage must never queue commands while teardown replies can still be
    /// in the correlator (Mustang log 153856: a 1003 paired with a stale
    /// poll-pipe reply + stray STOPPED, "succeeded", and the 5 s no-prompt
    /// watchdog dropped the connection).
    teardowns_in_flight: Arc<std::sync::atomic::AtomicUsize>,
    /// User preference gates (Settings → Data Acquisition): default ON.
    /// Disabled = that plugin never engages; polling is always available.
    uds_stream_enabled: bool,
    stn_periodic_enabled: bool,
    /// SR2b: Settings "Attempt calibration read on connect" (default ON).
    /// Off = no Ford `$23` strategy probe, no GM `F189` read; the strategy
    /// stays nil and downstream behaves fail-closed. Evaluated per connect.
    calibration_read_enabled: bool,
    /// Most recent discovered-connector list — lets select_connector() recover
    /// the full ConnectorInfo (esp. connector_type) from a bare id.
    discovered_connectors: Vec<crate::platform::ConnectorInfo>,
    /// Bumped on every new connect attempt AND on user cancel/disconnect; an
    /// in-flight connect flow compares its captured value and aborts silently
    /// when stale (P1: user cancel must actually stop the connect sequence).
    /// SL1: THE lifecycle fence — P1's `connect_generation` promoted to an atomic so
    /// terminal intent can bump it without the mutex (iron rule 1: safe in callbacks).
    /// Bumped by `begin_connect_attempt` (new attempt supersedes) and by the FIRST
    /// terminal claimant (claim-then-bump via `terminal_claimed`); consumed by equality.
    session_epoch: Arc<std::sync::atomic::AtomicU64>,
    /// SL1: terminal-intent claim (CAS). The first terminal source of a session (user
    /// disconnect, cancel-connect, fatal drop, quit) wins the claim and bumps the epoch;
    /// losers no-op — our own transport-close echo must NOT re-bump mid-drain, or the
    /// disconnect handler's own epoch-checked waits would see "intent moved" and
    /// truncate the drain. Reset by `begin_connect_attempt` (new life).
    terminal_claimed: Arc<std::sync::atomic::AtomicBool>,
    /// LH1: the protocol handler the session drives the wire through
    /// (`ElmHandler` today; `DviHandler` in LH6). Owns the adapter handshake,
    /// bus negotiation, wire encoding and status classification.
    link: Arc<dyn crate::link::LinkHandler>,
    /// LH7: per-command typed reply slots — filled by the data callback,
    /// awaited by the link handler's `exchange` (identify, chunk probe,
    /// send_control), uds_stream and the STN periodic engine.
    reply_waiters: Arc<crate::link::ReplyWaiters>,
    /// Currently active controller header on the adapter (e.g., "7E0", "7DF").
    /// Shared with data callback closure for controller switching in the event-driven loop.
    current_controller: Arc<Mutex<Option<String>>>,
    /// Negotiated CAN addressing. 11-bit until detection runs during identify (C0/C2).
    addressing: Arc<Mutex<crate::addressing::Addressing>>,
    /// SP29: primary (engine) ECU learned at identify — the physical STPPMA
    /// target for pids with no controller of their own (renders `7E0` on
    /// 11-bit, `18DA<ecu>F1` on 29-bit). None until identify runs.
    primary_ecu: Arc<Mutex<Option<crate::addressing::Controller>>>,
    /// Session monitor ring buffer for real-time command/response tracing.
    session_monitor: Arc<Mutex<crate::session_monitor::SessionMonitorBuffer>>,
    /// Last-connected connector + platform (no auto-reconnect — this feeds
    /// the disconnect callback's stale-drop check; cleared on disconnect).
    reconnect: Arc<crate::reconnect::ReconnectState>,
    /// Session-owned passive-listen buffer (plan M7) — shared by every transport.
    sink: Arc<crate::sink::SinkBuffer>,
    /// Subscriptions paused by sink_start, resumed by sink_stop.
    sink_paused_subs: Vec<Uuid>,
    /// DV3 silent-slot watchdog (2026-08-30): ms clock of the last
    /// slot-decoded batch (0 = none since arm) and of the arm itself; the
    /// background loop tears the periodic tier down after
    /// `PERIODIC_SILENT_MS` without frames and holds re-engage for
    /// `PERIODIC_BACKOFF_MS` (`tier_silent` in the health tracker only
    /// OBSERVED this — nothing reacted).
    periodic_last_rx_ms: Arc<std::sync::atomic::AtomicU64>,
    periodic_armed_ms: Arc<std::sync::atomic::AtomicU64>,
    periodic_backoff_until_ms: Arc<std::sync::atomic::AtomicU64>,
    /// LH0: the sniffer's STOPPED handshake (was the Swift `sinkStop` tap).
    /// While `sink_stop_flag` is set, the sink tap treats a STOPPED line as
    /// the ack (signalled through `sink_stop_ack`) instead of a frame.
    sink_stop_flag: Arc<std::sync::atomic::AtomicBool>,
    sink_stop_ack: Arc<(Mutex<bool>, std::sync::Condvar)>,
    /// Active acquisition plugin (P0: always `poll` at chunk = 1). The engine
    /// talks to the wire ONLY through this.
    acquisition: Arc<Mutex<Box<dyn crate::acquisition::AcquisitionPlugin>>>,
    /// Wire-command → member PIDs per pipe segment for in-flight plugin
    /// transactions. The data callback resolves completions here
    /// (common-ground #4: never re-derive members by string-parsing the
    /// wire); commands not in the map (init, direct API sends) fall back to
    /// the legacy pid extraction. Production paths hold pre-built Arc clones
    /// (captured before SharedState is assembled); this field is the
    /// test-seam the unit tests use to assert the map drains.
    #[allow(dead_code)]
    in_flight_members: InFlightMembers,
    /// Tier S: parsed stream profiles (udsprofiles.json, keyed by the
    /// vinrules capability tag) — handed in by the app at connect.
    stream_profiles: HashMap<String, crate::periodic_stream::StreamProfile>,
    /// Tier S: the active stream's teardown state (None = not streaming).
    stream_state: Arc<Mutex<Option<ActiveStream>>>,
    /// SP: active STN adapter-periodic acquisition (independent of streaming).
    periodic_state: Arc<Mutex<Option<ActivePeriodic>>>,
    /// Arc twin of `response_callback` — the stream ingest path emits
    /// obd_data from monitor lines outside the data-callback closure.
    response_callback_shared: Arc<dyn Fn(String) + Send + Sync>,
    /// SH1: session acquisition-health tracker + RTT snapshot (additive
    /// counters/emissions only; see `session_health.rs`).
    health: Arc<Mutex<HealthState>>,
    /// Per-VIN learned PID lengths (observe-then-batch): recorded by the data
    /// callback from plain single-PID responses, bound to `{VIN}.lengths`
    /// once identify learns the VIN, consumed via `PluginCtx::pid_len`.
    length_cache: Arc<Mutex<crate::length_cache::LengthCache>>,
    /// SL3: sender side of the lifecycle mailbox — the ONLY way lifecycle
    /// transitions reach the worker. Cloned (never locked over) by API
    /// enqueuers; the connection callback holds its own pre-cloned copy.
    lifecycle_tx: std::sync::mpsc::Sender<worker::LifecycleMsg>,
    /// SL3: the phase enum — sole source of truth for the session's mode
    /// (see `worker::Phase`). Tiny lock, held only for read/swap.
    phase: Arc<Mutex<worker::Phase>>,
}

impl SessionAPIHandle {
    /// Session-log a plugin transition with its wall-clock cost (bench
    /// visibility: how long the user rides poll rates around an edit).
    fn log_acquisition_switch(&self, transition: &str, t0: std::time::Instant) {
        let logger = {
            let state = self.shared_state.lock().unwrap();
            state.logger.clone()
        };
        log_cb!(
            logger,
            "acquisition_switch",
            &format!("{transition} in {} ms", t0.elapsed().as_millis()),
        );
        // SH1: the same transitions define the health-window boundaries —
        // the target tier is the suffix after the arrow.
        if let Some(tier) = transition
            .rsplit('→')
            .next()
            .and_then(crate::session_health::Tier::parse)
        {
            self.health_on_tier(tier);
        }
    }

    // MARK: - SH1 session health (additive wiring; tracker in session_health.rs)

    /// Copy the Arcs the health emitters need out of the shared-state lock.
    fn health_parts(&self) -> HealthParts {
        // Poison-tolerant: these run on disconnect/shutdown paths too.
        let state = self.shared_state.lock().unwrap_or_else(|p| p.into_inner());
        (
            Arc::clone(&state.health),
            state.logger.clone(),
            Arc::clone(&state.response_callback_shared),
            state.command_processor.as_ref().map(Arc::clone),
        )
    }

    /// CONNECTED just went out: restart health window 1 at this instant and
    /// consume the RTT stats baseline, so the first window measures dashboard
    /// traffic only — not the init/identify commands that ran before it
    /// (window 1 otherwise opens lazily at the first delivered signal, during
    /// adapter init).
    fn health_on_connected(&self) {
        let (health, _logger, _callback, processor) = self.health_parts();
        let mut hs = health.lock().unwrap();
        let _ = hs.window_rtt_ms(processor.map(|p| p.get_stats()));
        hs.tracker.reset_for_connected(health_now_ms());
    }

    /// The session is acquiring on `tier` from now on (window boundary).
    fn health_on_tier(&self, tier: crate::session_health::Tier) {
        let (health, logger, callback, processor) = self.health_parts();
        let events = {
            let mut hs = health.lock().unwrap();
            let rtt = hs.window_rtt_ms(processor.map(|p| p.get_stats()));
            hs.tracker.on_tier(tier, health_now_ms(), rtt)
        };
        emit_health_events(&events, &logger, &callback);
    }

    /// A subscription add/remove happened — tier churn around user edits is
    /// expected, not anomalous.
    fn health_on_subscription_edit(&self) {
        let (health, _logger, _callback, _processor) = self.health_parts();
        health
            .lock()
            .unwrap()
            .tracker
            .on_subscription_edit(health_now_ms());
    }

    /// End of session: emit the `session_summary` exactly once (disconnect
    /// paths and shutdown all call this; the tracker guards the double emit).
    fn health_finish_and_emit(&self) {
        let (health, logger, callback, processor) = self.health_parts();
        let finished = {
            let mut hs = health.lock().unwrap();
            let rtt = hs.window_rtt_ms(processor.map(|p| p.get_stats()));
            hs.tracker.finish(health_now_ms(), rtt)
        };
        if let Some((events, summary)) = finished {
            // The final window's acquisition_window line, then the summary.
            emit_health_events(&events, &logger, &callback);
            let payload = crate::session_health::summary_payload(&summary);
            log_cb!(logger, "session_summary", &payload.to_string());
            callback(
                serde_json::json!({ "type": "session_health", "payload": payload }).to_string(),
            );
        }
    }
}

mod api;
mod callbacks;
mod commands;
mod connect;
mod discovery;
mod engage;
mod sink;
mod sp_runtime;
mod tier_facts;
mod uds_stream;
mod worker;

#[cfg(test)]
use sp_runtime::periodic_response_id;
#[cfg(test)]
mod golden;
#[cfg(test)]
mod sl0_harness;
#[cfg(test)]
mod tests;
