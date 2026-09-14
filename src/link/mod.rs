//! LH1 — the seam between "the wire's protocol" and the engine.
//!
//! Everything above this module speaks in PIDs, segments and `ParsedResponse`;
//! everything below it speaks the adapter's dialect (ELM/STN text today, the
//! OBDX DVI binary protocol in LH6). The engine reaches the wire ONLY through
//! a [`LinkHandler`], which owns:
//!   * the adapter handshake (init list + hardware-won quirks + capability
//!     probes) → [`AdapterFacts`],
//!   * bus negotiation (11/29-bit sniff, primary-ECU learn),
//!   * wire encoding of a request group (STN `|` pipes) and the positional
//!     split of its reply,
//!   * the ONE classification of adapter status text → [`LinkStatus`]
//!     (previously string-sniffed in five places).
//!
//! Scope notes (Link_Handler_Plan.md): LH2 — identify consumes `LinkReply`
//! (status + per-controller parse); `request_raw` remains only for the
//! console-style `send_control` (text goes to the host verbatim). LH3 turns
//! the monitor path into `Frame`s. LH7 — the poll chain is event-driven, so
//! the handler never blocks inside it: it `encode`s the wire on the way out
//! and the platform hands the engine a typed `WireReply` (`Payload`s per
//! segment) on the way back; `exchange` is the connect-thread-only blocking
//! form (identify, probes).

pub mod dvi;
pub mod elm;
pub mod stn_periodic_engine;

use std::collections::HashMap;
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use crate::addressing::{Addressing, Controller};
use crate::response_parser::ParsedResponse;
use crate::stn_periodic::{PeriodicMessage, TierPeriods};

/// One controller's reply within a segment (LH7) — the ISO-TP payload after
/// framing: reply byte, echo, data. PCI stripped, multi-frame assembled. THE
/// unit both dialects hand the engine: ELM builds it from text lines
/// (`response_parser::text_payloads`), DVI straight from RX frames. Nothing
/// above the seam parses adapter text any more.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Payload {
    /// `Controller::key()` of the responder — the request id on 11-bit
    /// (`7E0` for a `7E8` reply), the source byte on 29-bit (`10`).
    pub controller: String,
    pub bytes: Vec<u8>,
}

/// One wire exchange's reply as the dialect delivers it to the engine (LH7):
/// the whole reply's display text (host `data` / console / Session Monitor —
/// never parsed) plus one `LinkReply` per pipe segment, positional (segment
/// j answers wire segment j; exactly one on DVI).
#[derive(Debug, Clone)]
pub struct WireReply {
    pub raw: String,
    pub segments: Vec<LinkReply>,
}

/// The typed outcome of one queued command: the dialect's reply, or a
/// transport/adapter failure code (`TIMEOUT`, `DISCONNECTED`, `WRITE_FAILED:
/// …`, `DVI_ERROR …`) — the codes the host's `error_severity` table keys on.
pub type LinkOutcome = Result<WireReply, String>;

/// A one-shot slot for one command's `LinkOutcome`, filled by the session's
/// data callback (LH7's replacement for the `captured_responses` text map).
pub struct ReplySlot {
    state: Mutex<Option<LinkOutcome>>,
    cvar: Condvar,
}

impl ReplySlot {
    /// Block until the outcome lands (or `timeout`); takes it.
    pub fn wait(&self, timeout: Duration) -> Option<LinkOutcome> {
        let deadline = Instant::now() + timeout;
        let mut g = self.state.lock().unwrap();
        loop {
            if g.is_some() {
                return g.take();
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return None;
            }
            let (ng, _) = self.cvar.wait_timeout(g, left).unwrap();
            g = ng;
        }
    }
}

/// Per-command reply slots keyed by the exact (uppercased) command string —
/// the correlation key the processor already uses. A caller arms a slot
/// BEFORE queueing so a reply landing while the poll chain is live is caught
/// without waiting for processor idleness (uds_stream, the STN periodic
/// engine); the data callback fills and removes it.
#[derive(Default)]
pub struct ReplyWaiters {
    slots: Mutex<HashMap<String, Arc<ReplySlot>>>,
}

impl ReplyWaiters {
    pub fn arm(&self, command: &str) -> Arc<ReplySlot> {
        let slot = Arc::new(ReplySlot {
            state: Mutex::new(None),
            cvar: Condvar::new(),
        });
        self.slots
            .lock()
            .unwrap()
            .insert(command.to_uppercase(), Arc::clone(&slot));
        slot
    }

    pub fn fill(&self, command: &str, outcome: &LinkOutcome) {
        let slot = self.slots.lock().unwrap().remove(&command.to_uppercase());
        if let Some(slot) = slot {
            *slot.state.lock().unwrap() = Some(outcome.clone());
            slot.cvar.notify_all();
        }
    }

    /// Drop a slot whose command never got queued.
    pub fn disarm(&self, command: &str) {
        self.slots.lock().unwrap().remove(&command.to_uppercase());
    }
}

/// One exchange as the engine sees it (LH2): the classified status and, for
/// data replies, the per-controller parse (`ParsedResponse.all_controllers`
/// keeps EVERY module that answered — an NRC-only module is present with
/// `is_valid == false`, which is how identify tells "absent" from "answered
/// but refused"). LH7: the typed `payloads` are what the engine consumes;
/// `raw` is the dialect's display text only — nothing above the seam
/// parses it.
#[derive(Debug, Clone)]
pub struct LinkReply {
    /// Verdict of the FIRST non-noise line (RB1's no-response classes hang
    /// off this — see `ElmHandler::detect_addressing`).
    pub status: LinkStatus,
    pub parsed: Option<ParsedResponse>,
    /// The dialect's whole-reply error veto: ANY line carries an error token
    /// (ELM: `NO DATA` / `*ERROR`). Identify's presence/name/cal verdicts
    /// were gated on this before LH2 and still are — a data frame followed by
    /// a trailing `CAN ERROR` is refused, not half-parsed.
    pub faulted: bool,
    /// LH7: the typed reply — one payload per responding controller (empty
    /// for acks, status tokens, no data). `parsed` is derived from these
    /// for the wire command; `parsed_for` re-derives for a member pid.
    pub payloads: Vec<Payload>,
    /// The dialect's display rendering (ELM: the text as received; DVI: the
    /// frames rendered `7E8 04 41 0C 1A F8`-style). Host `data`, console,
    /// Session Monitor — never parsed above the seam.
    pub raw: String,
}

impl LinkReply {
    /// Build a reply for `command` from typed payloads (the parse is derived).
    pub fn from_payloads(
        command: &str,
        status: LinkStatus,
        faulted: bool,
        payloads: Vec<Payload>,
        raw: String,
    ) -> Self {
        let parsed = crate::response_parser::from_payloads(command, &payloads);
        LinkReply {
            status,
            parsed,
            faulted,
            payloads,
            raw,
        }
    }
    /// No reply within the deadline.
    pub fn timeout() -> Self {
        LinkReply {
            status: LinkStatus::Timeout,
            parsed: None,
            faulted: false,
            payloads: Vec::new(),
            raw: String::new(),
        }
    }
    /// An adapter acknowledgement (`OK`) with nothing to parse.
    pub fn ack() -> Self {
        LinkReply {
            status: LinkStatus::Chatter,
            parsed: None,
            faulted: false,
            payloads: Vec::new(),
            raw: "OK".to_string(),
        }
    }
    /// The parse for a member pid (a chunk/pipe member differs from the
    /// wire string the reply was built for).
    pub fn parsed_for(&self, command: &str) -> Option<ParsedResponse> {
        crate::response_parser::from_payloads(command, &self.payloads)
    }
    /// The command got some reply from the adapter (data OR an acknowledgement).
    pub fn answered(&self) -> bool {
        self.status != LinkStatus::Timeout
    }
    /// A data reply on the first line (an NRC `7F xx` line counts — it is data).
    pub fn is_data(&self) -> bool {
        self.status == LinkStatus::Ok
    }
    /// The pre-LH2 identify gate, exactly: some reply and no error token on
    /// any line (a bare `OK`/`?`/`UNABLE TO CONNECT` passes — they never
    /// contained the veto words; the parse then decides).
    pub fn is_clean(&self) -> bool {
        self.answered() && !self.faulted
    }
    /// The parse, unless the reply is vetoed.
    pub fn clean_parsed(&self) -> Option<&ParsedResponse> {
        if self.faulted {
            None
        } else {
            self.parsed.as_ref()
        }
    }
}

/// Which dialect drives the link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkKind {
    /// ELM327-compatible text (incl. STN chips, which add `ST*` commands).
    Elm,
    /// OBDX Pro DVI binary (LH6).
    Dvi,
}

/// One CAN frame as received while monitoring (LH3).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// Protocol-normalized, uppercase CAN id — `7E8` (11-bit) or `18DAF110`
    /// (29-bit), the key `Bus::monitor_id` produces and the decoders match.
    pub id: String,
    /// 29-bit source.
    pub extended: bool,
    /// Microseconds since the Unix epoch, stamped at line arrival (no adapter
    /// timestamp exists on ELM/STN; DVI supplies one in LH6).
    pub ts_us: u64,
    /// Payload after the header tokens — PCI / slot byte first.
    pub data: Vec<u8>,
    /// The line as the link delivered it (prompt stripped) — the sniffer's
    /// `raw`, and the text the SP splitter re-frames (its 29-bit header must
    /// stay four spaced tokens).
    pub raw: String,
}

/// What the wire said while monitoring — parsed ONCE by the handler.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LinkEvent {
    Frame(Frame),
    /// The chip left monitor mode (`STOPPED`) — the stop/break handshakes'
    /// ack. Delivered synchronously on the transport thread, never queued.
    Stopped,
    /// Anything else: `OK` / `?` setup echoes, `BUFFER FULL`, `SEARCHING…`,
    /// and break-window replies that are not frames.
    Text(String),
}

/// What the wire said, classified once by the handler.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LinkStatus {
    /// Data lines (or at least something that isn't a status token).
    Ok,
    /// Adapter acknowledgement only (`OK`, `?`, an `ELM327`/`STN` banner) —
    /// the command was taken, there is nothing to parse.
    Chatter,
    /// Nothing came back from the bus (`NO DATA`, empty, all `SEARCHING…`).
    NoData,
    /// Monitor/stream interrupted (`STOPPED`).
    Stopped,
    /// No reply within the link's deadline (the transport's 5 s fallback).
    Timeout,
    /// Any other adapter error token, as the standardized code from
    /// `error::map_adapter_response_to_error` (`CAN_ERROR`, `UNABLE_TO_CONNECT`,
    /// `BUS_ERROR`, …) or `ERROR` for the bare/unknown variants. Every
    /// `*ERROR` flavour lands here — identify's verdicts treat them as one.
    AdapterError(String),
}

impl LinkStatus {
    /// "The bus did not answer" — the retryable class RB1 distinguishes from
    /// a real-but-wrong reply (ignition off / flaky link vs non-CAN vehicle).
    pub fn is_no_response(&self) -> bool {
        !matches!(self, LinkStatus::Ok | LinkStatus::Chatter)
    }
}

/// Facts the HOST feeds about the connected adapter (LH4): `adapters.json`
/// (`supportsPeriodic`, `maxChunk`) and `stnperiodic.json` (tier periods),
/// via `obd_set_stn_periodic_capable` / `obd_set_adapter_max_chunk` /
/// `obd_set_stn_periods`. A live shared cell — `set_stn_periods` can land
/// after connect.
#[derive(Debug, Clone, PartialEq)]
pub struct HostFacts {
    /// Which dialect drives the link (adapters.json `protocol`, DV2).
    pub kind: LinkKind,
    /// The STN chip runs STPPMA periodics (SP tier).
    pub periodic_capable: bool,
    /// Per-adapter J1979 chunk cap (≤1 disables the vehicle probe).
    pub max_chunk: usize,
    /// STPPMA tier → period.
    pub tier_periods: TierPeriods,
}

impl Default for HostFacts {
    fn default() -> Self {
        Self {
            kind: LinkKind::Elm,
            periodic_capable: false,
            max_chunk: crate::acquisition::CHUNK_MAX_PIDS,
            tier_periods: TierPeriods::default(),
        }
    }
}

/// Adapter-side facts: the handshake's (pipe probe, addressing sniff) plus a
/// snapshot of the host-fed ones.
#[derive(Debug, Clone, PartialEq)]
pub struct AdapterFacts {
    pub kind: LinkKind,
    /// OBDLink Batched Commands accepted (`STBC 1` + `STBCOF 0` → OK).
    pub pipe_capable: bool,
    /// Negotiated CAN addressing (11-bit until `detect_addressing` runs).
    pub addressing: Addressing,
    /// First `0100` responder — the engine ECU (SP29 primary target).
    pub primary_ecu: Option<Controller>,
    /// Host-fed (see `HostFacts`).
    pub periodic_capable: bool,
    pub max_chunk: usize,
    pub tier_periods: TierPeriods,
    /// DS0 (2026-08-29): request/response keeps running while an adapter-
    /// side stream is live — DVI (no monitor mode; slot replies and poll
    /// replies share the link, routed by echo). ELM/STN `false`: a live
    /// monitor owns the adapter, polling is quiesced.
    pub concurrent_rx: bool,
}

impl AdapterFacts {
    pub fn unknown(kind: LinkKind) -> Self {
        Self {
            kind,
            pipe_capable: false,
            addressing: Addressing::Can11,
            primary_ecu: None,
            periodic_capable: false,
            max_chunk: crate::acquisition::CHUNK_MAX_PIDS,
            tier_periods: TierPeriods::default(),
            concurrent_rx: false,
        }
    }
}

/// Monitor-mode handshake mechanics shared by the acquisition runtimes (the
/// session's UDS stream and the handler's periodic engine — RS7). MECHANICS
/// ONLY: timeouts stay per-call parameters, and each side keeps its OWN
/// ack-wait loop on `ack_slot` (the stream's token match + 7F-78 extension
/// vs SP's any-line ack).
#[derive(Clone)]
pub struct MonitorHandshake {
    /// Stops the monitor/beat loop thread.
    pub loop_stop: Arc<std::sync::atomic::AtomicBool>,
    /// STOPPED-handshake signal: set by the tap when the adapter acks a
    /// monitor break, waited on via `wait_stopped` with a bounded timeout.
    pub stopped_signal: Arc<(Mutex<bool>, std::sync::Condvar)>,
    /// Is the adapter currently in monitor mode?
    pub monitor_active: Arc<std::sync::atomic::AtomicBool>,
    /// Break-window ack capture: non-frame, non-STOPPED lines land here
    /// while a tap owns the transport.
    pub ack_slot: Arc<(Mutex<Option<String>>, std::sync::Condvar)>,
}

impl Default for MonitorHandshake {
    fn default() -> Self {
        Self::new()
    }
}

impl MonitorHandshake {
    pub fn new() -> Self {
        Self {
            loop_stop: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            stopped_signal: Arc::new((Mutex::new(false), std::sync::Condvar::new())),
            monitor_active: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            ack_slot: Arc::new((Mutex::new(None), std::sync::Condvar::new())),
        }
    }

    /// Clear the STOPPED flag ahead of a break/arm exchange.
    pub fn clear_stopped(&self) {
        *self.stopped_signal.0.lock().unwrap() = false;
    }

    /// Wait until the STOPPED flag is set or `timeout` elapses; returns the
    /// flag's final state (true = adapter acked the break / STM aborted).
    pub fn wait_stopped(&self, timeout: std::time::Duration) -> bool {
        let (lock, cvar) = &*self.stopped_signal;
        let seen = lock.lock().unwrap();
        let (seen, _) = cvar.wait_timeout_while(seen, timeout, |s| !*s).unwrap();
        *seen
    }
}

/// What the session asks the adapter to run periodically (LH4). Built by
/// the PURE `stn_periodic::build_periodic_set` under the session's
/// admission + length cache + bus — none of which crosses the seam.
pub struct PeriodicPlan {
    pub messages: Vec<PeriodicMessage>,
    /// pid → learned response length, for the echo-keyed splitter.
    pub lengths: std::collections::HashMap<String, u8>,
}

/// What actually installed.
#[derive(Debug, Clone)]
pub struct PeriodicInstall {
    /// Messages the adapter accepted (a partial install after OUT OF MEMORY
    /// keeps what took).
    pub accepted: usize,
    pub response_ids: Vec<String>,
    pub fast_period_ms: u32,
    /// The pids the accepted messages serve — on a `concurrent_rx` link the
    /// poll picker skips exactly these while the plan is live.
    pub served_pids: Vec<String>,
}

#[derive(Debug, Clone, Copy)]
pub struct PeriodicStats {
    /// Signals in the live decode map.
    pub members: usize,
    pub fast_period_ms: u32,
}

#[derive(Debug, Clone)]
pub enum PeriodicError {
    /// Not one STPPMA took.
    NoneAccepted,
    /// The link refused monitor mode.
    MonitorFailed(String),
    /// STM would not hold (the chip aborted every attempt).
    ArmAborted { attempts: u32 },
    /// Engine not started / already live.
    NotStarted,
}

impl std::fmt::Display for PeriodicError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PeriodicError::NoneAccepted => write!(f, "no STPPMA accepted — staying on poll"),
            PeriodicError::MonitorFailed(e) => write!(f, "monitor mode failed: {e}"),
            PeriodicError::ArmAborted { attempts } => {
                write!(f, "STM would not hold (aborted {attempts}×)")
            }
            PeriodicError::NotStarted => write!(f, "periodic engine not started"),
        }
    }
}

/// Decoded periodic signals: (pid, parsed) — one call per frame batch.
pub type PeriodicSink = Arc<dyn Fn(Vec<(String, ParsedResponse)>) + Send + Sync>;

/// Adapter-side periodic acquisition (LH4): the chip fires the poll set on
/// its own and the engine monitors the replies. `ElmHandler` supplies the
/// STN (STPPMA) engine; DVI has none by design (poll over DVI is streaming-
/// quality). Every method runs on the session's lifecycle worker — never
/// from a callback.
pub trait PeriodicEngine: Send + Sync {
    fn is_live(&self) -> bool;
    fn stats(&self) -> Option<PeriodicStats>;
    /// Install the plan (STCMM 1, one STPPMA per message). The caller has
    /// ALREADY quiesced polling. `NoneAccepted` = nothing installed, the
    /// caller unwinds.
    fn start(&self, plan: PeriodicPlan) -> Result<PeriodicInstall, PeriodicError>;
    /// Enter monitor (tap → STPO → STFAC → STFPA per id → STM verified);
    /// decoded signals reach `sink` off the transport thread.
    fn arm(&self, sink: PeriodicSink) -> Result<(), PeriodicError>;
    /// Stop phase 1: break monitor (space → STOPPED, STFAC), leave monitor
    /// mode — back at the prompt.
    fn break_monitor(&self);
    /// Stop phase 2: AT flush, STPPMC (verified, retried once), then the
    /// pinned protocol reopen (STPR → STP n → 0100; fallback STPC → 0100).
    fn teardown(&self);
    /// The link is gone (drop / fresh connect): forget state, touch NO wire.
    fn abandon(&self);
}

/// Why the adapter handshake failed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectFail {
    /// The command queue refused an init command (session shutting down).
    QueueRefused(String),
    /// Init commands did not all complete inside the deadline.
    InitTimeout,
}

/// Why bus negotiation failed (RB1 classes — the engine maps them to
/// `vehicle_not_responding` vs `unsupported_protocol`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddressingFail {
    /// Bus didn't answer (`UNABLE TO CONNECT` / `NO DATA` / all-`SEARCHING` /
    /// empty / timeout) — retryable.
    NoResponse,
    /// A real but non-CAN response.
    NonCan,
}

/// The protocol handler the session drives the wire through.
/// DV4: one capture id filter (`7E8`, `7E8/7FF`, `18DAF110`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CaptureFilter {
    pub id: u32,
    pub mask: u32,
    pub extended: bool,
}

pub trait LinkHandler: Send + Sync {
    fn kind(&self) -> LinkKind;

    /// Adapter handshake — vehicle-independent: init list, warm-start guard,
    /// stale periodic-slot purge, pipe probe. Result also cached in `facts()`.
    fn connect(&self) -> Result<AdapterFacts, ConnectFail>;

    /// Bus negotiation — needs the vehicle awake: sniff `0100` (learning the
    /// primary ECU), fall back to `ATDPN`. Separate from `connect` because the
    /// engine emits status + checks cancellation between the two.
    fn detect_addressing(&self) -> Result<(Addressing, Option<Controller>), AddressingFail>;

    /// Current snapshot of the adapter facts.
    fn facts(&self) -> AdapterFacts;

    /// One synchronous exchange (LH2): status + per-controller parse in the
    /// negotiated addressing. Connect-thread only: it waits for the whole
    /// processor to go idle, so it must never run inside the poll chain's
    /// data callback.
    fn request(&self, command: &str, timeout_ms: u32) -> LinkReply {
        self.exchange(command, timeout_ms)
            .and_then(|r| r.segments.into_iter().next())
            .unwrap_or_else(LinkReply::timeout)
    }

    /// One synchronous exchange, typed (LH7): the whole wire's reply. Same
    /// thread contract as `request` — `request`/`request_raw` derive from it.
    fn exchange(&self, command: &str, timeout_ms: u32) -> Option<WireReply>;

    /// Raw-text variant for the console-style path that hands the adapter's
    /// text to the host verbatim (`send_control`). Same thread contract.
    fn request_raw(&self, command: &str, timeout_ms: u32) -> Option<String> {
        self.exchange(command, timeout_ms).map(|r| r.raw)
    }

    /// Encode a request group (member PIDs per segment, wire order) into the
    /// exact wire string. len 1 = plain; >1 = one wire transaction.
    fn encode(&self, segments: &[Vec<String>]) -> String;

    /// Console / sniffer escape hatch — bypasses the correlator.
    fn raw_write(&self, line: &str);

    /// Passive listen (LH3): stop request/response operation and deliver
    /// every received line, parsed once, to `on` until `stop_monitor`. The
    /// caller still drives the adapter (filters + `STM`/`STMA`) through
    /// `raw_write` — filter application is not the handler's yet.
    fn monitor(&self, on: Box<dyn Fn(LinkEvent) + Send + Sync>) -> Result<(), String>;

    /// Leave passive listen and restore request/response operation.
    fn stop_monitor(&self);

    /// DV4: passive capture with adapter-side id filters. Default = plain
    /// `monitor` (ELM/STN: the host applies its own STFAC/STFPA filters);
    /// DVI installs PASS filters, goes listen-only and timestamps frames.
    fn capture(
        &self,
        filters: &[CaptureFilter],
        on: Box<dyn Fn(LinkEvent) + Send + Sync>,
    ) -> Result<(), String> {
        let _ = filters;
        self.monitor(on)
    }

    /// The adapter-side periodic engine, when the adapter has one (LH4).
    fn periodic(&self) -> Option<Arc<dyn PeriodicEngine>>;
}

/// Classify one trimmed line of adapter output. `None` = looks like data.
/// This is THE status-token table (was `response_parser::is_non_data_response`
/// + `error::map_adapter_response_to_error` + ad-hoc `contains` checks).
pub fn classify_line(line: &str) -> Option<LinkStatus> {
    let t = line.trim();
    if t.is_empty() {
        return Some(LinkStatus::NoData);
    }
    if t.starts_with("SEARCHING") {
        return Some(LinkStatus::NoData);
    }
    if t == "OK" || t == "?" || t.starts_with("ELM327") || t.starts_with("STN") {
        return Some(LinkStatus::Chatter);
    }
    if t == "NO DATA" {
        return Some(LinkStatus::NoData);
    }
    if t == "STOPPED" {
        return Some(LinkStatus::Stopped);
    }
    if let Some(code) = crate::error::map_adapter_response_to_error(t) {
        return Some(LinkStatus::AdapterError(code.to_string()));
    }
    // Bare / unmapped error flavours (`ERROR`, `BUS INIT: ...ERROR`, `FB ERROR`).
    if t == "ERROR" || t == "FB ERROR" || t == "BUS INIT: ...ERROR" {
        return Some(LinkStatus::AdapterError("ERROR".to_string()));
    }
    None
}

/// Classify a whole reply: the first line that is not `SEARCHING…`/blank
/// decides — data → `Ok`; otherwise its token. Nothing but noise → `NoData`.
pub fn classify_text(raw: Option<&str>) -> LinkStatus {
    let Some(raw) = raw else {
        return LinkStatus::Timeout;
    };
    raw.replace('\r', "\n")
        .lines()
        .map(str::trim)
        .find(|t| !t.is_empty() && !t.starts_with("SEARCHING"))
        .map(|t| classify_line(t).unwrap_or(LinkStatus::Ok))
        .unwrap_or(LinkStatus::NoData)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classify_tokens() {
        assert_eq!(classify_text(None), LinkStatus::Timeout);
        assert_eq!(classify_text(Some("")), LinkStatus::NoData);
        assert_eq!(classify_text(Some("SEARCHING...\r")), LinkStatus::NoData);
        assert_eq!(classify_text(Some("NO DATA")), LinkStatus::NoData);
        assert_eq!(
            classify_text(Some("SEARCHING...\rUNABLE TO CONNECT")),
            LinkStatus::AdapterError("UNABLE_TO_CONNECT".into())
        );
        assert_eq!(
            classify_text(Some("CAN ERROR")),
            LinkStatus::AdapterError("CAN_ERROR".into())
        );
        assert_eq!(
            classify_text(Some("ERROR")),
            LinkStatus::AdapterError("ERROR".into())
        );
        assert_eq!(classify_text(Some("STOPPED")), LinkStatus::Stopped);
        assert_eq!(classify_text(Some("OK")), LinkStatus::Chatter);
        assert_eq!(classify_text(Some("?")), LinkStatus::Chatter);
        assert_eq!(classify_text(Some("ELM327 v1.5")), LinkStatus::Chatter);
        assert_eq!(
            classify_text(Some("SEARCHING...\r7E8 06 41 00 BE 3F A8 13")),
            LinkStatus::Ok
        );
        assert!(LinkStatus::NoData.is_no_response());
        assert!(!LinkStatus::Ok.is_no_response());
    }

    /// The parser's non-data guard and the classifier agree on every token.
    #[test]
    fn classify_line_covers_parser_table() {
        for tok in [
            "OK",
            "?",
            "NO DATA",
            "UNABLE TO CONNECT",
            "CAN ERROR",
            "BUS BUSY",
            "BUS ERROR",
            "DATA ERROR",
            "<DATA ERROR",
            "<RX ERROR",
            "FB ERROR",
            "FC RX TIMEOUT",
            "BUFFER FULL",
            "OUT OF MEMORY",
            "STOPPED",
            "UART RX OVERFLOW",
            "LV RESET",
            "ACT ALERT",
            "LP ALERT",
            "BUS INIT: ...ERROR",
            "ERROR",
            "ELM327 v1.5",
            "STN2120 v4.2.1",
        ] {
            assert!(
                classify_line(tok).is_some(),
                "{tok} must classify as non-data"
            );
        }
        assert!(classify_line("7E8 04 41 0C 1A F8").is_none());
    }
}
