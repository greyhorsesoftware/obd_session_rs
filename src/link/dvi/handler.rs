//! `DviHandler` (LH6 = DV1) — the OBDX Pro DVI dialect behind [`LinkHandler`],
//! over ANY byte link (Rust TCP/serial, or the host's BLE via the byte
//! writer). Bootstrap = the one ELM string `DX DP 1`; DVI-only thereafter
//! (`31 02 06 01` pins it; a reset re-bootstraps). Engine contract kept
//! typed (LH7): every command the engine queues is encoded into a `10` TX
//! frame here, and the RX frames answering it are framed (ISO-TP assembled,
//! PCI dropped) straight into per-controller `Payload`s — the engine never
//! sees text from this adapter; the ELM-style line (`7E8 04 41 0C 1A F8`)
//! is rendered for DISPLAY only (host `data`, sniffer `raw`). No periodic
//! engine (poll over DVI is streaming-quality — plan §Scope calls); no
//! `STOPPED`, no monitor exclusivity.

use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use super::codec::{self, Comm, DviFrame, FilterType, ObdProtocol, RxFrame};
use super::periodic::{DviPeriodicEngine, SLOTS};
use crate::addressing::{Addressing, Bus, Controller};
use crate::command_processor::CommandProcessor;
use crate::link::{
    AdapterFacts, AddressingFail, ConnectFail, Frame, HostFacts, LinkEvent, LinkHandler, LinkKind,
    LinkReply, LinkStatus, Payload, PeriodicEngine, PeriodicError, PeriodicInstall, PeriodicPlan,
    PeriodicSink, PeriodicStats, ReplyWaiters, WireReply,
};
use crate::platform::OBDPlatformInterface;
use crate::session_logger::SessionLogger;

/// Functional request ids.
const FUNCTIONAL_11: u32 = 0x7DF;
const FUNCTIONAL_29: u32 = 0x18DB33F1;
/// Quiet window after the last frame of a functional (multi-ECU) reply.
const FUNCTIONAL_QUIET: Duration = Duration::from_millis(60);
/// No reply at all inside this window = `NO DATA` — the ISO-TP response
/// timeout an ELM adapter applies (ATST default ~200 ms); the tool itself
/// never says "nobody answered".
const NO_DATA_TIMEOUT: Duration = Duration::from_millis(250);
/// Setup exchange wait.
const SETUP_TIMEOUT: Duration = Duration::from_millis(1500);
/// Filter slots: 11-bit FLOW pairs 0–7, PASS-all 8, 29-bit FLOW pairs from 9
/// (all inside the hardware table's 28 11-bit slots too, should the
/// software-filter switch be refused).
/// Slot 8 holds the normal-operation OBD receive filter. Historically a
/// wide-open pass-all (mask 0) — replaced 2026-09-02/04 with the 0x700-0x7FF
/// diagnostic range (see `obd_pass_filter`) because pass-all forwarded the
/// ENTIRE live bus over BLE.
const OBD_PASS_SLOT: u8 = 8;
const FLOW_29_BASE: usize = 9;
/// DV3 leftover: the one FLOW filter that follows an `ATSH` target outside
/// the connect-time pairs (rewritten per target change).
const DYN_FLOW_SLOT: u8 = 26;
/// DV4: PASS filters for a capture live in slots 16–25.
const CAPTURE_SLOT_BASE: u8 = 16;
const CAPTURE_SLOTS: usize = 10;
/// A `7F xx 78` (response pending) from the target: wait this long for the
/// real answer (UDS P2*; uds_stream's break-window uses the same ~3 s).
const PENDING_WINDOW: Duration = Duration::from_millis(3000);

struct Outstanding {
    command: String,
    /// Physical request → the one response id that completes it.
    expected: Option<u32>,
    frames: Vec<RxFrame>,
    /// The target answered `7F xx 78` — its real reply is still coming.
    pending: bool,
    /// Stray `InvalidCommand` errors the tool sent for OUR bytes while this
    /// request was out — the tool's parser lost frame sync (bench
    /// 2026-08-29, WiFi: one error per byte of every TX).
    stray_errors: u32,
    started: Instant,
    last_frame: Instant,
    generation: u64,
}

struct State {
    /// `ATSH` target (None = functional).
    target: Option<(u32, bool)>,
    outstanding: Option<Outstanding>,
    generation: u64,
    tap: Option<Arc<dyn Fn(LinkEvent) + Send + Sync>>,
    /// CAN ids of the frames that answered the last completed command —
    /// `detect_addressing` reads the bus width off them.
    last_reply_ids: Vec<u32>,
    /// Last parser resync (throttle).
    last_resync: Option<Instant>,
    /// DV3: the live periodic-slot plan (None = not started).
    periodic: Option<PeriodicLive>,
    /// FLOW pairs installed at connect (response id, request id) — targets
    /// covered by them need no dynamic filter.
    flow_pairs: Vec<(u32, u32)>,
    /// The dynamic FLOW filter in `DYN_FLOW_SLOT`: (response id, request id).
    dyn_flow: Option<(u32, u32)>,
    /// DV4: capture PASS filters installed (slot, extended) + listen-only.
    capture_slots: Vec<(u8, bool)>,
    listen_only: bool,
    /// DV4: PASS-all (slot 8) was disabled for a filtered capture.
    pass_all_disabled: bool,
    /// Weak self for the periodic engine (set right after construction).
    self_weak: std::sync::Weak<DviHandler>,
}

/// What `periodic_start` installed: per slot the response id its replies
/// arrive on and the members (with learned lengths) the echo-keyed splitter
/// slices them with; `tx` is the decode channel `periodic_arm` opens.
struct PeriodicLive {
    /// (slot, response id, members[(pid, len)])
    /// (slot, response id, members, EF5-precomputed uppercase pid set —
    /// the `outstanding_wants` comparison key, built once at install)
    decoders: Vec<(
        u8,
        u32,
        Vec<(String, u8)>,
        std::collections::BTreeSet<String>,
    )>,
    fast_period_ms: u32,
    tx: Option<std::sync::mpsc::Sender<(u32, Payload)>>,
}

pub struct DviHandler {
    platform: Arc<dyn OBDPlatformInterface>,
    processor: Arc<CommandProcessor>,
    /// LH7: per-command reply slots the session's data callback fills.
    waiters: Arc<ReplyWaiters>,
    logger: Option<Arc<SessionLogger>>,
    facts: Arc<Mutex<AdapterFacts>>,
    host: Arc<Mutex<HostFacts>>,
    state: Arc<Mutex<State>>,
    /// Wakes the per-request collector on every counted frame (bench
    /// 2026-08-29, Jeep: without it the collector slept its whole 250 ms
    /// no-data window and every functional request cost ~254 ms).
    frame_cv: Condvar,
    /// Non-RX replies during setup exchanges.
    setup_slot: Arc<(Mutex<Option<DviFrame>>, Condvar)>,
    /// DV3: the slot engine, built on first use (holds a Weak to us).
    periodic_engine: std::sync::OnceLock<Arc<DviPeriodicEngine>>,
    /// BT0 (DVI_BLE_TX_Plan): TX/reply accounting, surfaced as a
    /// `dvi_tx_audit` jsonl line at most every 5 s while traffic flows.
    audit: TxAudit,
    /// EF5: mirrors `state.tap.is_some()` so `on_rx` skips the state lock
    /// entirely on the common (no sniffer/monitor) path.
    tap_armed: std::sync::atomic::AtomicBool,
}

/// BT0 counters — cumulative for the handler's lifetime; the audit line
/// prints totals, so storm rate reads from line spacing. `delivered` counts
/// real wire outcomes handed to the correlator (synthetic ELM-shim acks are
/// excluded) — replies_seen far above delivered is the H5 (routing loss)
/// signature; tx_sent far above tx_acked is H1/H2 (wire loss).
#[derive(Default)]
struct TxAudit {
    tx_sent: std::sync::atomic::AtomicU64,
    tx_acked: std::sync::atomic::AtomicU64,
    replies_seen: std::sync::atomic::AtomicU64,
    errors_seen: std::sync::atomic::AtomicU64,
    delivered: std::sync::atomic::AtomicU64,
    /// Positive replies whose echo named an earlier request — side-delivered
    /// under their own command, not counted in `delivered` (the outstanding
    /// request still produces its own completion or timeout).
    lagged: std::sync::atomic::AtomicU64,
    setup_timeouts: std::sync::atomic::AtomicU64,
    last_emit: Mutex<Option<Instant>>,
}

impl DviHandler {
    /// The base receive filter for NORMAL operation: pass the 11-bit
    /// DIAGNOSTIC range 0x700-0x7FF (id 0x700, mask 0x700), never the whole
    /// bus. A wide-open pass-all (mask 0) forwards every CAN frame over BLE —
    /// on a gateway-less car (2012 Mustang, ~300 frames/s of live broadcast,
    /// bench 2026-09-02) that saturates the BLE link. Every broadcast id on
    /// that bus floods BELOW 0x700 (measured), while every diagnostic
    /// response — legislated 7E8-7EF AND the physical modules the walk visits
    /// (720→728 … 7D0→7D8) — sits inside 0x700-0x7FF, so ONE range filter
    /// covers them all (David 2026-09-04: "one filter should cover all") and
    /// the per-target companion pass (old DYN_PASS_SLOT churn, half the
    /// walk's filter TX) is deleted. 29-bit cars receive via their own flow
    /// filters. The Console sniff/capture path installs its own wide filters
    /// when the user explicitly wants the whole bus.
    fn obd_pass_filter(&self, on: bool) -> Vec<u8> {
        codec::can_filter(OBD_PASS_SLOT, false, FilterType::Pass, on, 0x700, 0x700, 0)
    }

    /// Blind 29-bit RECEIVE pass: 29-bit diagnostic replies to tester F1 land
    /// on `18DAF1xx`, which the 11-bit `0x700` pass can't match (different id
    /// width). Without it the identify probe's 29-bit `0100` gets NO reply —
    /// the per-responder 29-bit FLOW filters (`install_flow_filters`) only go
    /// in AFTER a reply reveals the responders, so a 29-bit car can never
    /// receive its first frame and fails `vehicle_not_responding` over DVI
    /// (bench 2026-09-06, 29-bit Jeep — connected fine over the OBDLink/ELM,
    /// dead over the OBDX/DVI). Installed on `FLOW_29_BASE` at connect;
    /// `install_flow_filters(Can29)` overwrites it with the real flow pairs
    /// once the responders are known. `18DAF1xx` is response-only (nothing
    /// sends to the tester unsolicited), so it can't leak bus chatter.
    fn obd_pass_filter_29(&self, on: bool) -> Vec<u8> {
        codec::can_filter(
            FLOW_29_BASE as u8,
            true,
            FilterType::Pass,
            on,
            0x18DAF100,
            0x1FFFFF00,
            0,
        )
    }

    /// One `0100` liveness probe at (addressing, baud). We discard the
    /// payload — a reply's RESPONDER ID gives bus width (11- vs 29-bit),
    /// the answering baud, and the primary ECU. `Ok` installs the facts
    /// (+ 29-bit flow filters) and is the caller's cue to stop; `Err` carries
    /// the reply (if any) for the final NonCan-vs-NoResponse verdict.
    fn probe_addressing_combo(
        &self,
        a: Addressing,
        b: u8,
        baud: &mut u8,
        log_miss: bool,
    ) -> Result<(Addressing, Option<Controller>), Option<LinkReply>> {
        if b != *baud {
            let _ = self.exchange(&codec::set_baud(b), codec::cmd::CAN + 0x10);
            // A protocol-level change disables communication — re-enable.
            let _ = self.exchange(&codec::set_comm(Comm::On), codec::cmd::PROTOCOL + 0x10);
            *baud = b;
        }
        self.facts.lock().unwrap().addressing = a;
        self.state.lock().unwrap().target = None;
        let reply =
            LinkHandler::exchange(self, "0100", 5000).and_then(|r| r.segments.into_iter().next());
        let first_id = self.state.lock().unwrap().last_reply_ids.first().copied();
        if let (Some(r), Some(id)) = (&reply, first_id) {
            if r.status == LinkStatus::Ok {
                // 29-bit physical reply `18 DA <tester> <src>`: the tester
                // is learned from the id (as `Addressing::sniff` does).
                let found = if id > 0x7FF {
                    Addressing::Can29 {
                        tester: ((id >> 8) & 0xFF) as u8,
                    }
                } else {
                    Addressing::Can11
                };
                let toks = header_tokens(id);
                let refs: Vec<&str> = toks.iter().map(String::as_str).collect();
                let primary = Bus::new(found).read(&refs).map(|(c, _)| c);
                {
                    let mut f = self.facts.lock().unwrap();
                    f.addressing = found;
                    if primary.is_some() {
                        f.primary_ecu = primary;
                    }
                }
                self.log(
                    "dvi_setup",
                    &format!("bus {found:?} at baud code {baud} (0100 responder {id:X})"),
                );
                if matches!(found, Addressing::Can29 { .. }) {
                    let responders = self.state.lock().unwrap().last_reply_ids.clone();
                    self.install_flow_filters(found, &responders);
                }
                return Ok((found, primary));
            }
        }
        if log_miss {
            self.log(
                "dvi_setup",
                &format!(
                    "0100 at {a:?}/baud {b}: {:?}",
                    reply.as_ref().map(|r| r.status.clone())
                ),
            );
        }
        Err(reply)
    }

    pub fn new(
        platform: Arc<dyn OBDPlatformInterface>,
        processor: Arc<CommandProcessor>,
        waiters: Arc<ReplyWaiters>,
        logger: Option<Arc<SessionLogger>>,
        host: Arc<Mutex<HostFacts>>,
    ) -> Arc<Self> {
        let h = Arc::new(Self {
            platform,
            processor,
            waiters,
            logger,
            facts: Arc::new(Mutex::new(AdapterFacts::unknown(LinkKind::Dvi))),
            host,
            state: Arc::new(Mutex::new(State {
                target: None,
                outstanding: None,
                generation: 0,
                tap: None,
                flow_pairs: Vec::new(),
                dyn_flow: None,
                capture_slots: Vec::new(),
                listen_only: false,
                pass_all_disabled: false,
                last_reply_ids: Vec::new(),
                last_resync: None,
                periodic: None,
                self_weak: std::sync::Weak::new(),
            })),
            periodic_engine: std::sync::OnceLock::new(),
            audit: TxAudit::default(),
            tap_armed: std::sync::atomic::AtomicBool::new(false),
            frame_cv: Condvar::new(),
            setup_slot: Arc::new((Mutex::new(None), Condvar::new())),
        });
        h.state.lock().unwrap().self_weak = Arc::downgrade(&h);
        h.install_hooks();
        h
    }

    fn log(&self, key: &str, detail: &str) {
        if let Some(ref l) = self.logger {
            l.log_callback(key, detail);
        }
    }

    /// BT0: bump one audit counter.
    fn audit_bump(&self, c: &std::sync::atomic::AtomicU64) {
        c.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }

    /// BT0: emit the audit line if 5 s have passed since the last one.
    /// Called from the hot paths — the gate keeps it to one line / 5 s.
    fn maybe_emit_audit(&self) {
        {
            let mut last = self.audit.last_emit.lock().unwrap();
            match *last {
                Some(t) if t.elapsed() < Duration::from_secs(5) => return,
                _ => *last = Some(Instant::now()),
            }
        }
        use std::sync::atomic::Ordering::Relaxed;
        self.log(
            "dvi_tx_audit",
            &format!(
                "tx_sent={} tx_acked={} replies_seen={} errors_seen={} delivered={} lagged={} setup_timeouts={}",
                self.audit.tx_sent.load(Relaxed),
                self.audit.tx_acked.load(Relaxed),
                self.audit.replies_seen.load(Relaxed),
                self.audit.errors_seen.load(Relaxed),
                self.audit.delivered.load(Relaxed),
                self.audit.lagged.load(Relaxed),
                self.audit.setup_timeouts.load(Relaxed),
            ),
        );
    }

    /// Wire the platform's DVI hooks to this handler (weak — the platform
    /// outlives handlers; a rebuilt handler re-installs).
    fn install_hooks(self: &Arc<Self>) {
        // A binary dialect completes typed — no text decoder.
        self.platform.set_reply_decoder(None);
        let weak = Arc::downgrade(self);
        self.platform.set_frame_sink(Some(Arc::new(move |frame| {
            if let Some(h) = weak.upgrade() {
                h.on_frame(frame);
            }
        })));
        let weak = Arc::downgrade(self);
        self.platform
            .set_command_encoder(Some(Arc::new(move |cmd: &str| match weak.upgrade() {
                Some(h) => h.encode(cmd),
                None => Err("handler gone".to_string()),
            })));
    }

    fn bus(&self) -> Bus {
        Bus::new(self.facts.lock().unwrap().addressing)
    }

    fn functional(&self) -> (u32, bool) {
        match self.facts.lock().unwrap().addressing {
            Addressing::Can11 => (FUNCTIONAL_11, false),
            Addressing::Can29 { .. } => (FUNCTIONAL_29, true),
        }
    }

    // ---- command encoding (engine text → TX frame) ---------------------------

    fn encode(self: &Arc<Self>, command: &str) -> Result<Option<Vec<u8>>, String> {
        let cmd = command.trim().to_uppercase();
        if let Some(hdr) = cmd.strip_prefix("ATSH") {
            let hdr = hdr.trim();
            let id = u32::from_str_radix(hdr, 16).map_err(|_| format!("bad ATSH header {hdr}"))?;
            let extended = hdr.len() > 3;
            let is_functional = id == FUNCTIONAL_11 || id == FUNCTIONAL_29;
            self.state.lock().unwrap().target = if is_functional {
                None
            } else {
                Some((id, extended))
            };
            if !is_functional {
                self.ensure_flow_filter(id, extended);
            }
            self.platform.complete_command(command, Ok(ack_reply()));
            return Ok(None);
        }
        let compact: String = cmd.chars().filter(|c| !c.is_whitespace()).collect();
        let is_hex = !compact.is_empty()
            && compact.len() % 2 == 0
            && compact.chars().all(|c| c.is_ascii_hexdigit());
        if !is_hex || cmd.starts_with("AT") || cmd.starts_with("ST") || cmd.starts_with("DX") {
            // ELM/STN housekeeping has no DVI equivalent — answer as the
            // adapter would ("OK") so init-style callers keep flowing.
            self.platform.complete_command(command, Ok(ack_reply()));
            return Ok(None);
        }
        if compact.contains('|') {
            return Err("pipes are an STN encoding — not on DVI".to_string());
        }
        let payload: Vec<u8> = (0..compact.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&compact[i..i + 2], 16).unwrap())
            .collect();
        let (id, extended, expected) = {
            let st = self.state.lock().unwrap();
            match st.target {
                Some((id, ext)) => (id, ext, Some(response_id(id, ext))),
                None => {
                    let (f, ext) = self.functional();
                    (f, ext, None)
                }
            }
        };
        let generation = {
            let mut st = self.state.lock().unwrap();
            st.generation += 1;
            let g = st.generation;
            st.outstanding = Some(Outstanding {
                command: command.to_string(),
                expected,
                frames: Vec::new(),
                pending: false,
                stray_errors: 0,
                started: Instant::now(),
                last_frame: Instant::now(),
                generation: g,
            });
            g
        };
        self.spawn_collector(generation);
        self.audit_bump(&self.audit.tx_sent);
        self.maybe_emit_audit();
        Ok(Some(codec::tx_can(id, extended, &payload)))
    }

    /// Per-request timer: no frame inside `NO_DATA_TIMEOUT` → `NO DATA`;
    /// a functional request completes `FUNCTIONAL_QUIET` after its last
    /// frame (a physical one completes on its reply in `on_rx`).
    fn spawn_collector(self: &Arc<Self>, generation: u64) {
        let weak = Arc::downgrade(self);
        std::thread::spawn(move || loop {
            let Some(h) = weak.upgrade() else { return };
            let st = h.state.lock().unwrap();
            let wait = match st.outstanding.as_ref() {
                Some(o) if o.generation == generation => {
                    if o.frames.is_empty() {
                        let left = NO_DATA_TIMEOUT.saturating_sub(o.started.elapsed());
                        if left.is_zero() {
                            None
                        } else {
                            Some(left)
                        }
                    } else if o.expected.is_none() {
                        let left = FUNCTIONAL_QUIET.saturating_sub(o.last_frame.elapsed());
                        if left.is_zero() {
                            None
                        } else {
                            Some(left)
                        }
                    } else {
                        // Physical with frames from OTHER ids only (or a
                        // `7F xx 78` from the target): keep the clock
                        // relative to the last frame.
                        let window = if o.pending {
                            PENDING_WINDOW
                        } else {
                            NO_DATA_TIMEOUT
                        };
                        let left = window.saturating_sub(o.last_frame.elapsed());
                        if left.is_zero() {
                            None
                        } else {
                            Some(left)
                        }
                    }
                }
                _ => return,
            };
            match wait {
                // Sleep at most until the deadline — and wake early on every
                // frame (`on_rx` notifies) so the quiet window is measured
                // from the frame's arrival, not from the next timer tick.
                Some(d) => {
                    let (g, _) = h
                        .frame_cv
                        .wait_timeout(st, d.max(Duration::from_millis(1)))
                        .unwrap();
                    drop(g);
                }
                None => {
                    // A stray-error burst while this request was out = the
                    // tool's parser is misaligned (bench 2026-08-29, WiFi /
                    // classic bridges): realign it BEFORE completing, so the
                    // chain's next request lands on a clean parser. Runs on
                    // this collector thread (the setup exchanges wait on
                    // frames delivered by the worker thread).
                    let strays = st.outstanding.as_ref().map(|o| o.stray_errors).unwrap_or(0);
                    drop(st);
                    if strays >= 3 {
                        h.resync();
                    }
                    h.finish(generation, None);
                    return;
                }
            }
        });
    }

    /// Complete the outstanding command, typed (`err` = a DVI error code).
    /// No frame inside the window = `NO DATA` — the ISO-TP response timeout
    /// an ELM adapter reports; the tool itself never says "nobody answered".
    fn finish(&self, generation: u64, err: Option<String>) {
        let done = {
            let mut st = self.state.lock().unwrap();
            match st.outstanding.as_ref() {
                Some(o) if o.generation == generation => {
                    let o = st.outstanding.take();
                    if let Some(o) = &o {
                        st.last_reply_ids = o.frames.iter().map(|f| f.id).collect();
                    }
                    o
                }
                _ => None,
            }
        };
        if let Some(o) = done {
            if o.stray_errors > 0 {
                self.log(
                    "dvi_desync",
                    &format!("{} stray DVI error(s) while {} was out (tool parsing our frame byte by byte?)", o.stray_errors, o.command),
                );
            }
            let outcome = match err {
                Some(e) => Err(e),
                None if o.frames.is_empty() => {
                    let reply = LinkReply {
                        status: LinkStatus::NoData,
                        parsed: None,
                        faulted: true,
                        payloads: Vec::new(),
                        raw: "NO DATA".to_string(),
                    };
                    Ok(WireReply {
                        raw: reply.raw.clone(),
                        segments: vec![reply],
                    })
                }
                None => {
                    // Incomplete assemblies (a CF never came) are not payloads.
                    let payloads = frame_payloads(&o.frames, self.bus())
                        .into_iter()
                        .filter(|(_, _, done)| *done)
                        .map(|(_, p, _)| p)
                        .collect();
                    let raw = o
                        .frames
                        .iter()
                        .map(render_frame)
                        .collect::<Vec<_>>()
                        .join("\r");
                    let reply = LinkReply::from_payloads(
                        &o.command,
                        LinkStatus::Ok,
                        false,
                        payloads,
                        raw.clone(),
                    );
                    Ok(WireReply {
                        raw,
                        segments: vec![reply],
                    })
                }
            };
            self.audit_bump(&self.audit.delivered);
            self.platform.complete_command(&o.command, outcome);
        }
    }

    // ---- frames from the tool ---------------------------------------------

    fn on_frame(self: &Arc<Self>, frame: DviFrame) {
        // BT0: reply/error accounting (before routing, so a diverted or
        // dropped reply still counts as SEEN — that gap is the H5 signal).
        match &frame {
            DviFrame::Reply { .. } => self.audit_bump(&self.audit.replies_seen),
            DviFrame::Error { .. } => self.audit_bump(&self.audit.errors_seen),
            _ => {}
        }
        self.maybe_emit_audit();
        match frame {
            DviFrame::Rx(rx) => self.on_rx(rx),
            DviFrame::Reply { cmd, .. }
                if cmd == codec::cmd::TX_NORMAL + 0x10 || cmd == codec::cmd::TX_LARGE + 0x10 =>
            {
                // TX acknowledged — the audit is the only consumer (BT0).
                self.audit_bump(&self.audit.tx_acked);
            }
            DviFrame::Error { cmd, code }
                if cmd == codec::cmd::TX_NORMAL || cmd == codec::cmd::TX_LARGE =>
            {
                let gen = self
                    .state
                    .lock()
                    .unwrap()
                    .outstanding
                    .as_ref()
                    .map(|o| o.generation);
                if let Some(g) = gen {
                    self.finish(g, Some(format!("DVI_ERROR {}", codec::error_name(code))));
                }
            }
            DviFrame::Error { cmd, code } => {
                // An error for a command we never sent while a TX is out =
                // the tool parsing our frame byte by byte. Count it on the
                // outstanding request (reported by `finish`) and keep the
                // setup slot in the loop for the setup exchanges.
                let counted = {
                    let mut st = self.state.lock().unwrap();
                    match st.outstanding.as_mut() {
                        Some(o) => {
                            o.stray_errors += 1;
                            true
                        }
                        None => false,
                    }
                };
                if !counted {
                    let (lock, cvar) = &*self.setup_slot;
                    *lock.lock().unwrap() = Some(DviFrame::Error { cmd, code });
                    cvar.notify_all();
                }
            }
            other => {
                let (lock, cvar) = &*self.setup_slot;
                *lock.lock().unwrap() = Some(other);
                cvar.notify_all();
            }
        }
    }

    fn on_rx(self: &Arc<Self>, rx: RxFrame) {
        let extended = rx.id > 0x7FF;
        // Sniffer / stream tap sees every frame (DVI has no monitor
        // exclusivity). EF5: `tap_armed` keeps the common (no sniffer) path
        // out of the state lock here.
        if self.tap_armed.load(std::sync::atomic::Ordering::Relaxed) {
            let tap = self.state.lock().unwrap().tap.clone();
            if let Some(tap) = tap {
                tap(LinkEvent::Frame(Frame {
                    id: if extended {
                        format!("{:08X}", rx.id)
                    } else {
                        format!("{:03X}", rx.id)
                    },
                    extended,
                    ts_us: rx.ts_us.map(|t| t as u64).unwrap_or_else(now_us),
                    data: rx.data.clone(),
                    // DV4: a capture line carries the tool's µs stamp (on only
                    // while monitoring) — `12.345678 7E8 04 41 0C …`.
                    raw: match rx.ts_us {
                        Some(t) => format!("{:.6} {}", f64::from(t) / 1e6, render_frame(&rx)),
                        None => render_frame(&rx),
                    },
                }));
            }
        }
        let bus = self.bus();
        // DV3: polling keeps running while slots are live (`concurrent_rx`),
        // so a frame on a slot's response id may be a slot reply OR the
        // answer to an outstanding poll — the echo decides: it goes to the
        // stream decoder only if a slot's members slice it cleanly (the
        // poll picker never polls a slot-served pid, so a poll reply's echo
        // never matches a slot). Anything else falls through to the request.
        // ECHO GUARD (branch 2 below): on a shared response id (7E8) every
        // one of the PCM's answers looks alike by CAN id, so a reply for a
        // PREVIOUS request (ECU slower than the poll cadence) would
        // otherwise complete the CURRENT one → off-by-one mismatch cascade
        // that never resyncs (Mustang OBDX 2026-09-04). A plain single-pid
        // poll may only complete on a positive reply that echoes ITS pid; a
        // positive reply with a different echo is the lagged answer to an
        // earlier poll — not garbage, DATA. Deliver it upstream under the
        // command its echo names (sent == response, no mismatch) and leave
        // the outstanding request in place: completing a command that is
        // not outstanding is a chain no-op (`mark_command_completed`), so
        // the current request still completes on ITS reply. Only a whole
        // single frame can be re-attributed — a lagged FIRST frame has no
        // consecutive frames coming (nobody flow-controls a dead request),
        // so it is dropped rather than mis-assembled. A 7F negative carries
        // no pid (falls through, accepted); compound / AT commands have no
        // simple echo (guard skipped).
        //
        // EF5: ONE lock section decides routed / lagged / completion (was
        // three takes per frame); the lagged side-delivery still runs
        // OUTSIDE the lock — `complete_command` re-enters the upper layers.
        enum Act {
            Routed,
            Lagged(Option<(String, WireReply)>),
            Complete(u64),
            Nothing,
        }
        let act = {
            let mut st = self.state.lock().unwrap();
            // DV3 routing: does a slot's member set slice this frame cleanly?
            let routed = match st.periodic.as_ref() {
                Some(p) if p.decoders.iter().any(|(_, id, _, _)| *id == rx.id) => {
                    match frame_payloads(std::slice::from_ref(&rx), bus)
                        .into_iter()
                        .next()
                    {
                        Some((_, payload, true)) => {
                            let slot_match = p.decoders.iter().find(|(_, id, members, _)| {
                                *id == rx.id
                                    && crate::response_parser::split_multi_pid(
                                        members,
                                        std::slice::from_ref(&payload),
                                    )
                                    .map_or(false, |m| !m.is_empty())
                            });
                            // A request still out for EXACTLY this pid set
                            // (the sweep queued before the install draining
                            // — same wire as the slot) is answered first, or
                            // it would sit until timeout while the stream
                            // ate its reply (bench 20:31: 250–535 ms rtt).
                            // EF5: compared against the pid set PRECOMPUTED
                            // at install (was an uppercase collect per frame).
                            let outstanding_wants = slot_match.is_some()
                                && st.outstanding.as_ref().map_or(false, |o| {
                                    (o.expected == Some(rx.id)
                                        || (o.expected.is_none() && is_obd_responder(rx.id)))
                                        && slot_match.map_or(false, |(_, _, _, pidset)| {
                                            command_pid_set(&o.command) == *pidset
                                        })
                                });
                            let is_slot_reply = slot_match.is_some() && !outstanding_wants;
                            if is_slot_reply {
                                if let Some(tx) = p.tx.as_ref() {
                                    let _ = tx.send((rx.id, payload));
                                }
                            }
                            is_slot_reply
                        }
                        _ => false,
                    }
                }
                _ => false,
            };
            if routed {
                Act::Routed
            } else {
                let lagged = st.outstanding.as_ref().map_or(false, |o| {
                    (o.expected == Some(rx.id) || (o.expected.is_none() && is_obd_responder(rx.id)))
                        && expected_echo(&o.command).map_or(false, |echo| {
                            let resp = frame_response_bytes(&rx.data);
                            resp.first().map_or(false, |&b| b & 0x40 != 0 && b != 0x7F)
                                && !resp.starts_with(&echo)
                        })
                });
                if lagged {
                    let deliver = frame_payloads(std::slice::from_ref(&rx), bus)
                        .into_iter()
                        .next()
                        .and_then(|(_, payload, done)| {
                            if !done {
                                return None;
                            }
                            let cmd = command_for_reply(&payload.bytes)?;
                            let raw = render_frame(&rx);
                            let reply = LinkReply::from_payloads(
                                &cmd,
                                LinkStatus::Ok,
                                false,
                                vec![payload],
                                raw.clone(),
                            );
                            Some((
                                cmd,
                                WireReply {
                                    raw,
                                    segments: vec![reply],
                                },
                            ))
                        });
                    Act::Lagged(deliver)
                } else {
                    match st.outstanding.as_mut() {
                        None => Act::Nothing,
                        // Only frames that can answer the request count toward it:
                        // the physical target's reply id, or an OBD responder
                        // (`7E8–7EF` / `18DAxxyy` physical replies) for a functional
                        // one. Anything else on the bus (broadcast chatter) went to
                        // the tap above and must neither reset the quiet/no-data
                        // clocks nor become a controller key.
                        Some(o)
                            if o.expected == Some(rx.id)
                                || (o.expected.is_none() && is_obd_responder(rx.id)) =>
                        {
                            let id = rx.id;
                            let nrc_pending = rx.data.len() >= 4
                                && rx.data[0] == 0x03
                                && rx.data[1] == 0x7F
                                && rx.data[3] == 0x78;
                            o.frames.push(rx);
                            o.last_frame = Instant::now();
                            if nrc_pending && o.expected == Some(id) {
                                o.pending = true;
                            }
                            match o.expected {
                                // A physical request completes on its reply — once
                                // the reply is whole (a multi-frame answer waits for
                                // its consecutive frames; `7F xx 78` waits for the
                                // real answer).
                                Some(want)
                                    if want == id
                                        && !nrc_pending
                                        && reply_complete(&o.frames, id, bus) =>
                                {
                                    Act::Complete(o.generation)
                                }
                                _ => Act::Nothing,
                            }
                        }
                        Some(_) => Act::Nothing,
                    }
                }
            }
        };
        match act {
            // A slot reply never wakes the collector (unchanged behavior).
            Act::Routed => {}
            Act::Lagged(deliver) => {
                self.audit_bump(&self.audit.lagged);
                if let Some((cmd, wire)) = deliver {
                    self.platform.complete_command(&cmd, Ok(wire));
                }
                self.frame_cv.notify_all();
            }
            Act::Complete(g) => {
                self.frame_cv.notify_all();
                self.finish(g, None);
            }
            Act::Nothing => self.frame_cv.notify_all(),
        }
    }

    // ---- setup exchanges (bootstrap / identity / protocol) ------------------

    fn exchange(&self, tx: &[u8], expect_cmd: u8) -> Result<Vec<u8>, String> {
        self.exchange_within(tx, expect_cmd, SETUP_TIMEOUT)
    }

    /// An error frame counts only when it answers THIS command (`cmd ==
    /// tx[0]`); errors provoked by earlier garbage (an ELM string that hit
    /// a tool already in DVI mode) are logged and skipped.
    fn exchange_within(
        &self,
        tx: &[u8],
        expect_cmd: u8,
        timeout: Duration,
    ) -> Result<Vec<u8>, String> {
        {
            let (lock, _) = &*self.setup_slot;
            *lock.lock().unwrap() = None;
        }
        self.platform.write_bytes(tx)?;
        self.audit_bump(&self.audit.tx_sent);
        let (lock, cvar) = &*self.setup_slot;
        let deadline = Instant::now() + timeout;
        let mut g = lock.lock().unwrap();
        loop {
            match g.take() {
                Some(DviFrame::Reply { cmd, data }) if cmd == expect_cmd => return Ok(data),
                Some(DviFrame::Error { cmd, code }) if cmd == tx[0] => {
                    return Err(format!(
                        "DVI error on {cmd:02X}: {}",
                        codec::error_name(code)
                    ))
                }
                Some(DviFrame::Error { cmd, code }) => {
                    self.log(
                        "dvi_setup",
                        &format!(
                            "stray DVI error on {cmd:02X}: {} (ignored)",
                            codec::error_name(code)
                        ),
                    );
                }
                Some(_) => {}
                None => {}
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                self.audit_bump(&self.audit.setup_timeouts);
                return Err(format!("no DVI reply to {:02X}", tx[0]));
            }
            let (ng, _) = cvar.wait_timeout(g, left).unwrap();
            g = ng;
        }
    }

    /// Flow control on DVI is filter-driven: the tool sends `30 00 00` only
    /// for a responder matching a FLOW-type filter (its Flow ID is where the
    /// FC goes — §3.14.1/§3.14.10), so a multi-frame reply (VIN) needs one.
    /// 11-bit: the eight standard pairs `7E8+n ← 7E0+n`; 29-bit: one per
    /// `0100` responder (`18DAF1xx ← 18DAxxF1`). Slot 31 is a PASS-all
    /// filter (mask 0) so replies from any other id still arrive (physical
    /// targets outside the standard set get no FC — a follow-up: a dynamic
    /// FLOW filter per `ATSH` target).
    fn install_flow_filters(&self, a: Addressing, responders: &[u32]) {
        let mut pairs: Vec<(u32, u32)> = Vec::new(); // (response id, request id)
        let extended = match a {
            Addressing::Can11 => {
                for n in 0..8u32 {
                    pairs.push((0x7E8 + n, 0x7E0 + n));
                }
                false
            }
            Addressing::Can29 { tester } => {
                for &rx in responders {
                    if rx > 0x7FF && !pairs.iter().any(|(r, _)| *r == rx) {
                        let src = rx & 0xFF;
                        let tx = (rx & 0xFFFF_0000) | (src << 8) | tester as u32;
                        pairs.push((rx, tx));
                    }
                }
                true
            }
        };
        let mask = if extended { 0x1FFF_FFFF } else { 0x7FF };
        // Slots 0–7 = the 11-bit pairs (connect); 29-bit pairs from FLOW_29_BASE.
        let base = if extended { FLOW_29_BASE } else { 0 };
        for (i, (rx, tx)) in pairs.iter().enumerate() {
            let n = (base + i).min(27);
            let f = codec::can_filter(n as u8, extended, FilterType::Flow, true, *rx, mask, *tx);
            if let Err(e) = self.exchange(&f, codec::cmd::CAN + 0x10) {
                self.log(
                    "dvi_setup",
                    &format!("flow filter {n} ({rx:X}←{tx:X}) refused: {e}"),
                );
            }
        }
        self.log(
            "dvi_setup",
            &format!(
                "{} {} flow filters installed",
                pairs.len(),
                if extended { "29-bit" } else { "11-bit" }
            ),
        );
        self.state.lock().unwrap().flow_pairs.extend(pairs);
    }

    /// DV3 leftover (2026-08-30): a physical target outside the connect-time
    /// FLOW pairs gets ONE FLOW filter of its own (`response_id(target) ←
    /// target`) in `DYN_FLOW_SLOT`, rewritten when the target moves to
    /// another uncovered module — without it the tool never sends `30 00
    /// 00` and a multi-frame reply from that module stops at frame one.
    /// Runs on the `ATSH` path (once per target change, the engine's own
    /// switch cadence), before the `ATSH` ack, so the first request already
    /// has flow control.
    fn ensure_flow_filter(&self, id: u32, extended: bool) {
        let covered = {
            let st = self.state.lock().unwrap();
            st.flow_pairs.iter().any(|(_, tx)| *tx == id)
                || st.dyn_flow.map_or(false, |(_, tx)| tx == id)
        };
        if covered {
            return;
        }
        let rx = response_id(id, extended);
        let mask = if extended { 0x1FFF_FFFF } else { 0x7FF };
        let f = codec::can_filter(
            DYN_FLOW_SLOT,
            extended,
            FilterType::Flow,
            true,
            rx,
            mask,
            id,
        );
        match self.exchange(&f, codec::cmd::CAN + 0x10) {
            Ok(_) => {
                self.state.lock().unwrap().dyn_flow = Some((rx, id));
                self.log(
                    "dvi_flow_filter",
                    &format!("slot {DYN_FLOW_SLOT}: {rx:X} ← {id:X}"),
                );
            }
            Err(e) => self.log("dvi_flow_filter", &format!("{rx:X} ← {id:X} refused: {e}")),
        }
    }

    /// Realign the tool's DVI parser after a stray-error burst. The parser
    /// waits indefinitely for the rest of a truncated frame (its byte-wait
    /// timer only runs while bytes flow — the probe script showed a clean
    /// `ByteWaitTimeout` when bytes DO flow), so every later frame lost its
    /// first byte to the stale one and errored byte by byte. Filler zeros
    /// complete whatever is pending (each surplus zero is one harmless
    /// `InvalidCommand`), then `31 02 06 01` confirms alignment. Throttled.
    fn resync(&self) {
        {
            let mut st = self.state.lock().unwrap();
            if st
                .last_resync
                .map_or(false, |t| t.elapsed() < Duration::from_secs(2))
            {
                return;
            }
            st.last_resync = Some(Instant::now());
        }
        let _ = self.platform.write_bytes(&[0u8; 24]);
        self.audit_bump(&self.audit.tx_sent);
        std::thread::sleep(Duration::from_millis(60));
        match self.exchange(&codec::set_api_dvi(), codec::cmd::PROTOCOL + 0x10) {
            Ok(r) if r == [0x06, 0x01] => self.log(
                "dvi_resync",
                "parser realigned (filler + 31 02 06 01 confirmed)",
            ),
            Ok(r) => self.log("dvi_resync", &format!("unexpected confirm reply {r:02X?}")),
            Err(e) => self.log("dvi_resync", &format!("confirm failed: {e}")),
        }
    }

    /// Idempotent bootstrap (bench 2026-08-29: the tool STAYS in DVI across
    /// a BLE disconnect, and an ELM string into a DVI parser is garbage —
    /// `D`/`X` read as cmd 0x44 with length 88, swallowing what follows).
    /// So: probe DVI first with the valid `31 02 06 01` (+ a CR so an
    /// ELM-mode tool just answers `?`); a `41 02 06 01` means we are already
    /// there. No reply → the one ELM string `DX DP 1`, a short settle, then
    /// the binary confirm.
    fn bootstrap(&self) -> Result<(), String> {
        self.platform.set_byte_mode(true);
        let mut probe = codec::set_api_dvi();
        probe.push(b'\r');
        if let Ok(r) = self.exchange_within(
            &probe,
            codec::cmd::PROTOCOL + 0x10,
            Duration::from_millis(600),
        ) {
            if r == [0x06, 0x01] {
                self.log("dvi_setup", "already in DVI mode");
                return Ok(());
            }
        }
        self.platform.set_byte_mode(false);
        self.platform.write_raw("DX DP 1");
        std::thread::sleep(Duration::from_millis(250));
        self.platform.set_byte_mode(true);
        let r = self.exchange(&codec::set_api_dvi(), codec::cmd::PROTOCOL + 0x10)?;
        if r != [0x06, 0x01] {
            return Err(format!("DVI mode not confirmed: {r:02X?}"));
        }
        self.log("dvi_setup", "DX DP 1 → DVI confirmed");
        Ok(())
    }
}

/// The positive-response echo a plain single-pid command expects: `41 <pid>`
/// for mode 01, `62 <DID hi> <DID lo>` for mode 22. None for compound
/// (piped / multi-pid / expected-count suffix) or non-01/22 commands — those
/// legitimately don't open with a simple echo, so they skip the echo guard.
fn expected_echo(command: &str) -> Option<Vec<u8>> {
    let c = command.trim().to_uppercase();
    if c.contains('|') || c.contains(' ') {
        return None;
    }
    let hex = |r: std::ops::Range<usize>| u8::from_str_radix(c.get(r)?, 16).ok();
    match (c.get(0..2), c.len()) {
        (Some("01"), 4) => Some(vec![0x41, hex(2..4)?]),
        (Some("22"), 6) => Some(vec![0x62, hex(2..4)?, hex(4..6)?]),
        _ => None,
    }
}

/// The response bytes of a frame, past its ISO-TP PCI (single-frame = 1-byte
/// PCI, first-frame = 2-byte). Used only to peek the leading service+echo.
fn frame_response_bytes(data: &[u8]) -> &[u8] {
    match data.first().map(|b| b >> 4) {
        Some(0) => data.get(1..).unwrap_or(&[]),  // single frame
        Some(1) if data.len() >= 2 => &data[2..], // first frame
        _ => data,
    }
}

/// The poll command a positive reply's echo names — the inverse of
/// `expected_echo`: `62 F4 0C …` → `"22F40C"`, `41 0C …` → `"010C"`. Used to
/// re-attribute a lagged reply to the request it actually answers; other
/// service bytes return None (the lagged frame is then dropped).
fn command_for_reply(payload: &[u8]) -> Option<String> {
    match payload {
        [0x41, pid, ..] => Some(format!("01{:02X}", pid)),
        [0x62, hi, lo, ..] => Some(format!("22{:02X}{:02X}", hi, lo)),
        _ => None,
    }
}

/// Physical response id for a request id: 11-bit `+8` (GM low range
/// `+0x400`), 29-bit `xxDA<tgt><src>` byte swap.
fn response_id(id: u32, extended: bool) -> u32 {
    // `periodic_response_id` returns a HEX string — parse it as hex. The old
    // primary used `.parse::<u32>()` (DECIMAL): harmless for a response id
    // with a hex letter (it fails decimal-parse and fell through to the hex
    // path), but an ALL-NUMERIC response id like 728/738/768 parsed to the
    // wrong CAN id (0x2D8/0x2E2/0x300), so the per-target filters for
    // IPC(720)/PSCM(730)/ABS(760) listened on the wrong id and those modules'
    // replies were dropped → they vanished from the walk (Mustang 2026-09-04).
    let hdr = if extended {
        format!("{id:08X}")
    } else {
        format!("{id:03X}")
    };
    let fallback = if extended { id } else { id + 8 };
    u32::from_str_radix(&crate::addressing::periodic_response_id(&hdr), 16).unwrap_or(fallback)
}

/// The adapter's ack for a command with no bus exchange (`ATSH`, ELM/STN
/// housekeeping the engine still sends).
fn ack_reply() -> WireReply {
    let reply = LinkReply::ack();
    WireReply {
        raw: reply.raw.clone(),
        segments: vec![reply],
    }
}

/// Header tokens for a CAN id, as the addressing layer reads them: 11-bit
/// one 3-hex token, 29-bit four bytes.
fn header_tokens(id: u32) -> Vec<String> {
    if id > 0x7FF {
        id.to_be_bytes()
            .iter()
            .map(|b| format!("{b:02X}"))
            .collect()
    } else {
        vec![format!("{id:03X}")]
    }
}

/// `Controller::key()` for a responder id in the negotiated addressing
/// (the request id on 11-bit, the source byte on 29-bit); an id the
/// addressing does not recognize keys as its hex.
fn controller_key(id: u32, bus: Bus) -> String {
    let toks = header_tokens(id);
    let refs: Vec<&str> = toks.iter().map(String::as_str).collect();
    bus.read(&refs)
        .map(|(c, _)| c.key())
        .unwrap_or_else(|| toks.concat())
}

/// Render an RX frame for DISPLAY (host `data`, sniffer `raw`): the ELM
/// `ATH1` shape — 11-bit id as one 3-hex token, 29-bit as four spaced bytes,
/// payload bytes verbatim (PCI included, as on the wire).
fn render_frame(rx: &RxFrame) -> String {
    let mut out = header_tokens(rx.id).join(" ");
    for b in &rx.data {
        out.push_str(&format!(" {b:02X}"));
    }
    out
}

/// LH7: RX frames → per-controller payloads, ISO-TP framed in arrival
/// order: single frame `0N` → its N bytes; first frame `1X LL` opens an
/// assembly (the tool's auto frame processing may deliver it already whole —
/// more than a CAN frame's worth of bytes); consecutive frames `2N` extend
/// the open assembly for that id; flow control `3X` is skipped; a payload
/// with no PCI passes verbatim. Returns (id, payload, complete).
fn frame_payloads(frames: &[RxFrame], bus: Bus) -> Vec<(u32, Payload, bool)> {
    struct Acc {
        id: u32,
        payload: Payload,
        total: usize,
        done: bool,
    }
    let mut accs: Vec<Acc> = Vec::new();
    for f in frames {
        let d = &f.data;
        let Some(&pci) = d.first() else { continue };
        let key = || controller_key(f.id, bus);
        match pci >> 4 {
            0 if pci > 0 => {
                let n = (pci & 0x0F) as usize;
                let end = (1 + n).min(d.len());
                accs.push(Acc {
                    id: f.id,
                    payload: Payload {
                        controller: key(),
                        bytes: d[1..end].to_vec(),
                    },
                    total: n,
                    done: true,
                });
            }
            1 if d.len() >= 2 => {
                let total = (((pci & 0x0F) as usize) << 8) | d[1] as usize;
                let mut bytes = d[2..].to_vec();
                if bytes.len() > total {
                    bytes.truncate(total);
                }
                let done = bytes.len() >= total;
                accs.push(Acc {
                    id: f.id,
                    payload: Payload {
                        controller: key(),
                        bytes,
                    },
                    total,
                    done,
                });
            }
            2 => {
                if let Some(acc) = accs.iter_mut().rev().find(|a| a.id == f.id && !a.done) {
                    acc.payload.bytes.extend_from_slice(&d[1..]);
                    if acc.payload.bytes.len() >= acc.total {
                        acc.payload.bytes.truncate(acc.total);
                        acc.done = true;
                    }
                }
                // A stray consecutive frame with no open assembly is noise.
            }
            3 => {} // flow control
            _ => accs.push(Acc {
                id: f.id,
                payload: Payload {
                    controller: key(),
                    bytes: d.clone(),
                },
                total: d.len(),
                done: true,
            }),
        }
    }
    accs.into_iter()
        .map(|a| (a.id, a.payload, a.done))
        .collect()
}

/// The pid set a poll wire carries: `010C0D` → {010C, 010D}; `22F190F191`
/// → {22F190, 22F191} (mode 22 = 2-byte ids). Empty for non-data wires.
fn command_pid_set(command: &str) -> std::collections::BTreeSet<String> {
    let c = command.trim().to_uppercase();
    let mut out = std::collections::BTreeSet::new();
    if c.len() < 4 || !c.bytes().all(|b| b.is_ascii_hexdigit()) {
        return out;
    }
    let (mode, rest) = c.split_at(2);
    let width = if mode == "22" { 4 } else { 2 };
    if rest.len() % width != 0 {
        return out;
    }
    for i in (0..rest.len()).step_by(width) {
        out.insert(format!("{mode}{}", &rest[i..i + width]));
    }
    out
}

/// An id that can carry an OBD reply: 11-bit `7E8–7EF`, 29-bit `18DAxxyy`
/// (a physical response to any tester).
fn is_obd_responder(id: u32) -> bool {
    (0x7E8..=0x7EF).contains(&id) || (id >> 16) == 0x18DA
}

/// Is the latest reply from `id` whole (single frame, or an assembled
/// multi-frame)?
fn reply_complete(frames: &[RxFrame], id: u32, bus: Bus) -> bool {
    frame_payloads(frames, bus)
        .iter()
        .rev()
        .find(|(fid, _, _)| *fid == id)
        .map(|(_, _, done)| *done)
        .unwrap_or(false)
}

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

impl LinkHandler for DviHandler {
    fn kind(&self) -> LinkKind {
        LinkKind::Dvi
    }

    /// Bootstrap → identity → HS-CAN 500k, comm ON, no filters (every frame
    /// passes; the engine attributes by id).
    fn connect(&self) -> Result<AdapterFacts, ConnectFail> {
        {
            let mut st = self.state.lock().unwrap();
            st.target = None;
            st.flow_pairs.clear();
            st.dyn_flow = None;
            st.capture_slots.clear();
            st.listen_only = false;
            st.pass_all_disabled = false;
        }
        self.bootstrap().map_err(ConnectFail::QueueRefused)?;
        // The model reply (`22 01 02` → "FT"/"GT"/"VX") is the identity
        // PROBE only — the product is "OBDX Pro" whatever the variant (David
        // 2026-08-29: "no need to mention FT, all the same stuff").
        let _model = self
            .exchange(
                &codec::info_request(codec::info::MODEL),
                codec::cmd::INFO + 0x10,
            )
            .unwrap_or_default();
        let serial = self
            .exchange(
                &codec::info_request(codec::info::SERIAL),
                codec::cmd::INFO + 0x10,
            )
            .unwrap_or_default();
        let serial_txt: String = serial.iter().skip(1).map(|b| format!("{b:02X}")).collect();
        self.log("dvi_identity", &format!("OBDX Pro sn={serial_txt}"));
        self.exchange(
            &codec::set_protocol(ObdProtocol::HsCan),
            codec::cmd::PROTOCOL + 0x10,
        )
        .map_err(ConnectFail::QueueRefused)?;
        // Pad every written frame to 8 bytes (what an ELM does by default —
        // bench 2026-08-29: a DLC-3 `7DF 02 01 00` may be ignored). The
        // manual leaves the default blank; set it and log the tool's answer.
        match self.exchange(&codec::set_padding(true), codec::cmd::CAN + 0x10) {
            Ok(r) => self.log("dvi_setup", &format!("padding on → {r:02X?}")),
            Err(e) => self.log("dvi_setup", &format!("padding on refused: {e}")),
        }
        // Software filters: the hardware table is 28 11-bit + 8 29-bit slots
        // (Note 1) — software gives the full 32 (what their J2534 runs on).
        match self.exchange(&codec::set_software_filters(true), codec::cmd::CAN + 0x10) {
            Ok(r) => self.log("dvi_setup", &format!("software filters → {r:02X?}")),
            Err(e) => self.log("dvi_setup", &format!("software filters refused: {e}")),
        }
        // The manual's worked recipe (§3.16.3: a FLOW filter `7E8/7FF/7E0`
        // before enabling; §3.4: "set at least one filter before enabling
        // the network") for all eight standard 11-bit pairs, plus a PASS-all
        // — so an ECUsim/any 11-bit car answers with flow control from the
        // first request; 29-bit pairs are added by `detect_addressing` when
        // a 29-bit responder shows up.
        self.install_flow_filters(Addressing::Can11, &[]);
        match self.exchange(&self.obd_pass_filter(true), codec::cmd::CAN + 0x10) {
            Ok(r) => self.log(
                "dvi_setup",
                &format!("OBD pass filter 7E8-7EF installed → {r:02X?}"),
            ),
            Err(e) => self.log("dvi_setup", &format!("OBD pass filter refused: {e}")),
        }
        // Companion 29-bit receive pass so the blind Can29 identify probe can
        // hear a 29-bit car's first reply (18DAF1xx) — see obd_pass_filter_29.
        match self.exchange(&self.obd_pass_filter_29(true), codec::cmd::CAN + 0x10) {
            Ok(r) => self.log(
                "dvi_setup",
                &format!("29-bit OBD pass filter 18DAF1xx installed → {r:02X?}"),
            ),
            Err(e) => self.log("dvi_setup", &format!("29-bit OBD pass filter refused: {e}")),
        }
        self.exchange(&codec::set_comm(Comm::On), codec::cmd::PROTOCOL + 0x10)
            .map_err(ConnectFail::QueueRefused)?;
        // Timestamps off.
        let _ = self.exchange(&codec::set_timestamps(false), codec::cmd::SETTINGS + 0x10);
        self.platform.set_frame_timestamps(false);
        {
            let mut f = self.facts.lock().unwrap();
            f.pipe_capable = false;
        }
        Ok(self.facts())
    }

    /// Functional `0100`: the responders' ids say 11- vs 29-bit; the first
    /// responder is the primary ECU.
    /// The ELM auto-search, on DVI: a functional `0100` at 11-bit/500k,
    /// then 29-bit/500k (`18DB33F1` — a 29-bit car never answers `7DF`;
    /// bench 2026-08-29), then both at 250k. The responders' ids fix the
    /// addressing (tester learned from a 29-bit reply id) and the primary
    /// ECU; the FLOW filters go in for the bus found.
    fn detect_addressing(&self) -> Result<(Addressing, Option<Controller>), AddressingFail> {
        let mut baud = codec::BAUD_500K;
        let mut last: Option<LinkReply> = None;
        // 500k is every DVI target (Ford/GM), so retry the two 500k
        // addressings to match the ELM's persistent auto-search before the
        // 250k fallback: a slow-waking PCM (2012 Mustang, bench 2026-09-02)
        // answered the CX's FIRST 0100 only after ~3.4s of SEARCHING retries,
        // while a single NO_DATA_TIMEOUT (250ms) shot per combo gave up ~13x
        // too early → the OBDX reported `vehicle_not_responding` on a car the
        // CX read fine seconds later. Retry 500k until ~4s elapsed, logging
        // only the first pass's misses; then the 250k pair once.
        let deadline = Instant::now() + Duration::from_millis(4000);
        let mut pass = 0u32;
        loop {
            for a in [Addressing::Can11, Addressing::Can29 { tester: 0xF1 }] {
                match self.probe_addressing_combo(a, codec::BAUD_500K, &mut baud, pass == 0) {
                    Ok(hit) => return Ok(hit),
                    Err(r) => last = r,
                }
            }
            pass += 1;
            if Instant::now() >= deadline {
                break;
            }
        }
        for a in [Addressing::Can11, Addressing::Can29 { tester: 0xF1 }] {
            match self.probe_addressing_combo(a, codec::BAUD_250K, &mut baud, true) {
                Ok(hit) => return Ok(hit),
                Err(r) => last = r,
            }
        }
        // Nothing answered anywhere: back to the defaults.
        if baud != codec::BAUD_500K {
            let _ = self.exchange(&codec::set_baud(codec::BAUD_500K), codec::cmd::CAN + 0x10);
            let _ = self.exchange(&codec::set_comm(Comm::On), codec::cmd::PROTOCOL + 0x10);
        }
        self.facts.lock().unwrap().addressing = Addressing::Can11;
        match last {
            Some(r) if !r.status.is_no_response() => Err(AddressingFail::NonCan),
            _ => Err(AddressingFail::NoResponse),
        }
    }

    fn facts(&self) -> AdapterFacts {
        let mut f = self.facts.lock().unwrap().clone();
        let host = self.host.lock().unwrap();
        // DV3: the tool's 8 periodic slots — advertised by the catalog
        // (`supportsPeriodic`), like the STN chip's STPPMA.
        f.periodic_capable = host.periodic_capable;
        f.concurrent_rx = true; // no monitor mode: slots and polls share the link
        f.max_chunk = host.max_chunk;
        f.tier_periods = host.tier_periods;
        f
    }

    /// Same queue/wait contract as ElmHandler — the encoder above turns the
    /// queued text into a TX frame, `finish` completes it typed.
    fn exchange(&self, command: &str, timeout_ms: u32) -> Option<WireReply> {
        let slot = self.waiters.arm(command);
        if self
            .processor
            .queue_command(command.to_string(), timeout_ms)
            .is_err()
        {
            return None;
        }
        let _ = self
            .processor
            .wait_for_idle(Duration::from_millis(timeout_ms as u64 + 2000));
        slot.wait(Duration::ZERO)?.ok()
    }

    fn encode(&self, segments: &[Vec<String>]) -> String {
        // No pipes on DVI: one segment = one wire; the poll plugin never
        // builds more (pipe_capable is false).
        segments
            .first()
            .map(|s| crate::link::elm::join_chunk_wire(s))
            .unwrap_or_default()
    }

    /// Console escape hatch: hex bytes → one DVI frame written as-is.
    fn raw_write(&self, line: &str) {
        let compact: String = line.chars().filter(|c| c.is_ascii_hexdigit()).collect();
        if compact.len() >= 2 && compact.len() % 2 == 0 {
            let bytes: Vec<u8> = (0..compact.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&compact[i..i + 2], 16).unwrap())
                .collect();
            let _ = self.platform.write_bytes(&bytes);
            self.audit_bump(&self.audit.tx_sent);
        }
    }

    /// Frames flow to the tap alongside request/response (no exclusivity);
    /// timestamps on for the capture.
    fn monitor(&self, on: Box<dyn Fn(LinkEvent) + Send + Sync>) -> Result<(), String> {
        self.state.lock().unwrap().tap = Some(Arc::from(on));
        self.tap_armed
            .store(true, std::sync::atomic::Ordering::Relaxed);
        if self
            .exchange(&codec::set_timestamps(true), codec::cmd::SETTINGS + 0x10)
            .is_ok()
        {
            self.platform.set_frame_timestamps(true);
        }
        Ok(())
    }

    /// DV4 capture: PASS filters for the id list (slots 16–25; PASS-all
    /// parked while a filter list is in force), the bus in LISTEN-ONLY
    /// (`31 02 02 02` — nothing transmitted, no flow control injected while
    /// another tool flashes), then `monitor` (tap + µs timestamps).
    fn capture(
        &self,
        filters: &[crate::link::CaptureFilter],
        on: Box<dyn Fn(LinkEvent) + Send + Sync>,
    ) -> Result<(), String> {
        let mut slots: Vec<(u8, bool)> = Vec::new();
        for (i, f) in filters.iter().enumerate() {
            if i >= CAPTURE_SLOTS {
                self.log(
                    "dvi_capture",
                    &format!(
                        "{} filter(s) beyond the {CAPTURE_SLOTS}-slot budget skipped",
                        filters.len() - CAPTURE_SLOTS
                    ),
                );
                break;
            }
            let n = CAPTURE_SLOT_BASE + i as u8;
            match self.exchange(
                &pass_filter_frame(n, f.id, f.mask, f.extended),
                codec::cmd::CAN + 0x10,
            ) {
                Ok(_) => slots.push((n, f.extended)),
                Err(e) => self.log(
                    "dvi_capture",
                    &format!("pass filter {:X}/{:X} refused: {e}", f.id, f.mask),
                ),
            }
        }
        let filtered = !slots.is_empty();
        if filtered {
            let off = codec::can_filter(OBD_PASS_SLOT, false, FilterType::Pass, false, 0, 0, 0);
            if let Err(e) = self.exchange(&off, codec::cmd::CAN + 0x10) {
                self.log("dvi_capture", &format!("OBD pass filter park refused: {e}"));
            }
        }
        let listen = match self.exchange(
            &codec::set_comm(codec::Comm::ListenOnly),
            codec::cmd::PROTOCOL + 0x10,
        ) {
            Ok(_) => true,
            Err(e) => {
                self.log(
                    "dvi_capture",
                    &format!("listen-only refused: {e} — capturing with the bus in normal mode"),
                );
                false
            }
        };
        {
            let mut st = self.state.lock().unwrap();
            st.capture_slots = slots.clone();
            st.listen_only = listen;
            st.pass_all_disabled = filtered;
        }
        self.log(
            "dvi_capture",
            &format!(
                "listen-only={listen}, {}",
                if filtered {
                    format!("{} pass filter(s)", slots.len())
                } else {
                    "PASS-all".to_string()
                }
            ),
        );
        self.monitor(on)
    }

    fn stop_monitor(&self) {
        let (slots, listen, parked) = {
            let mut st = self.state.lock().unwrap();
            st.tap = None;
            self.tap_armed
                .store(false, std::sync::atomic::Ordering::Relaxed);
            (
                std::mem::take(&mut st.capture_slots),
                std::mem::take(&mut st.listen_only),
                std::mem::take(&mut st.pass_all_disabled),
            )
        };
        let _ = self.exchange(&codec::set_timestamps(false), codec::cmd::SETTINGS + 0x10);
        self.platform.set_frame_timestamps(false);
        // DV4 unwind: bus back to normal, capture filters off, PASS-all back.
        if listen {
            if let Err(e) = self.exchange(
                &codec::set_comm(codec::Comm::On),
                codec::cmd::PROTOCOL + 0x10,
            ) {
                self.log(
                    "dvi_capture",
                    &format!("comm ON after capture refused: {e}"),
                );
            }
        }
        for (n, extended) in slots {
            let _ = self.exchange(
                &codec::can_filter(n, extended, FilterType::Pass, false, 0, 0, 0),
                codec::cmd::CAN + 0x10,
            );
        }
        if parked {
            let _ = self.exchange(&self.obd_pass_filter(true), codec::cmd::CAN + 0x10);
        }
        if listen || parked {
            self.log("dvi_capture", "stopped — comm ON, capture filters cleared");
        }
    }

    /// The slot engine — only when the catalog marks the adapter periodic-
    /// capable (every OBDX Pro is; a `supportsPeriodic: false` entry keeps
    /// the dashboard on the poll tier).
    fn periodic(&self) -> Option<Arc<dyn PeriodicEngine>> {
        if !self.host.lock().unwrap().periodic_capable {
            return None;
        }
        let engine = self
            .periodic_engine
            .get_or_init(|| Arc::new(DviPeriodicEngine::new(self.self_weak())));
        Some(Arc::clone(engine) as Arc<dyn PeriodicEngine>)
    }
}

// ---- DV3: periodic slots (the `DviPeriodicEngine` delegates here) -----------

impl DviHandler {
    fn self_weak(&self) -> std::sync::Weak<DviHandler> {
        self.state.lock().unwrap().self_weak.clone()
    }

    pub(super) fn periodic_is_live(&self) -> bool {
        self.state.lock().unwrap().periodic.is_some()
    }

    pub(super) fn periodic_stats(&self) -> Option<PeriodicStats> {
        let st = self.state.lock().unwrap();
        st.periodic.as_ref().map(|p| PeriodicStats {
            members: p.decoders.iter().map(|(_, _, m, _)| m.len()).sum(),
            fast_period_ms: p.fast_period_ms,
        })
    }

    /// Load the plan into the tool's slots: data → interval → enable, one
    /// slot per message, Fast tier first (the plan is tier-ordered). Past
    /// 8 messages the rest are logged and left on the paused poll set.
    pub(super) fn periodic_start(
        &self,
        plan: PeriodicPlan,
    ) -> Result<PeriodicInstall, PeriodicError> {
        if self.periodic_is_live() {
            return Err(PeriodicError::NotStarted);
        }
        let mut decoders: Vec<(
            u8,
            u32,
            Vec<(String, u8)>,
            std::collections::BTreeSet<String>,
        )> = Vec::new();
        let mut to_enable: Vec<(u8, String, u32)> = Vec::new();
        let mut response_ids: Vec<String> = Vec::new();
        let mut served_pids: Vec<String> = Vec::new();
        let mut fast_period_ms = u32::MAX;
        let mut capped = 0usize;
        for m in &plan.messages {
            let slot = decoders.len() as u8;
            if slot >= SLOTS {
                capped += 1;
                continue;
            }
            let Ok(id) = u32::from_str_radix(m.header.trim(), 16) else {
                self.log("dvi_periodic_skip", &format!("bad header {}", m.header));
                continue;
            };
            let obd: Vec<u8> = (0..m.data.len())
                .step_by(2)
                .filter_map(|i| u8::from_str_radix(m.data.get(i..i + 2)?, 16).ok())
                .collect();
            // Periodic frames are RAW — no auto-format (§3.12.22.2: the
            // length "will not be automatically calculated"): the CAN
            // payload carries its own ISO-TP length byte. `02 01 0C`.
            if obd.is_empty() || obd.len() > 7 {
                self.log(
                    "dvi_periodic_skip",
                    &format!("payload {} does not fit one frame", m.data),
                );
                continue;
            }
            let mut payload = vec![obd.len() as u8];
            payload.extend_from_slice(&obd);
            // HARDWARE 2026-08-29 (Jeep, WiFi probe): a slot whose data is
            // a full 8-byte CAN payload fires exactly on its period (6-PID
            // `07 01 05 0C 0D 0F 11 04` @50 ms → 20.3/s, assembled reply);
            // a short frame (`02 01 0C`) is stored and ACKed but NEVER
            // transmitted, and 9+ bytes is `7F 02 34 05`. So: always pad
            // to 8 — the same shape the tool's own padding gives a poll.
            payload.resize(8, 0);
            // Manual order: interval → data → enable (§3.14.26.1–3). The
            // enable is deferred until every decoder is REGISTERED below —
            // bench 2026-08-29 20:31: the first slot reply arrived before
            // `state.periodic` existed, fell through to the outstanding poll
            // (`chunk_validation_failure unknown echo 0A`, a Session Monitor
            // mismatch) and chunk-excluded that poll's pids.
            let loaded = self
                .exchange(
                    &codec::periodic_interval(slot, m.period_ms.min(u16::MAX as u32) as u16),
                    codec::cmd::CAN + 0x10,
                )
                .and_then(|_| {
                    self.exchange(
                        &codec::periodic_data(slot, id, &payload),
                        codec::cmd::CAN + 0x10,
                    )
                });
            if let Err(e) = loaded {
                self.log(
                    "dvi_periodic_skip",
                    &format!("slot {slot} {} @{} ms refused: {e}", m.data, m.period_ms),
                );
                continue;
            }
            let resp = response_id(id, id > 0x7FF);
            let members: Vec<(String, u8)> = m
                .pids
                .iter()
                .map(|p| (p.clone(), plan.lengths.get(p).copied().unwrap_or(1)))
                .collect();
            let pidset: std::collections::BTreeSet<String> =
                members.iter().map(|(p, _)| p.to_uppercase()).collect();
            decoders.push((slot, resp, members, pidset));
            to_enable.push((slot, m.data.clone(), m.period_ms));
            fast_period_ms = fast_period_ms.min(m.period_ms);
        }
        if decoders.is_empty() {
            return Err(PeriodicError::NoneAccepted);
        }
        let fast_period_ms = if fast_period_ms == u32::MAX {
            25
        } else {
            fast_period_ms
        };
        // Decoders live BEFORE the first slot fires: `on_rx` routes slot
        // replies away from the poll path from the very first frame (the
        // decode channel opens at `arm`; until then routed frames drop).
        self.state.lock().unwrap().periodic = Some(PeriodicLive {
            decoders: decoders.clone(),
            fast_period_ms,
            tx: None,
        });
        let mut enabled: Vec<u8> = Vec::new();
        for (slot, data, period_ms) in &to_enable {
            match self.exchange(&codec::periodic_enable(*slot, true), codec::cmd::CAN + 0x10) {
                Ok(_) => enabled.push(*slot),
                Err(e) => self.log(
                    "dvi_periodic_skip",
                    &format!("slot {slot} {data} @{period_ms} ms enable refused: {e}"),
                ),
            }
        }
        decoders.retain(|(slot, _, _, _)| enabled.contains(slot));
        if decoders.is_empty() {
            self.state.lock().unwrap().periodic = None;
            return Err(PeriodicError::NoneAccepted);
        }
        for (_, resp, members, _) in &decoders {
            served_pids.extend(members.iter().map(|(p, _)| p.clone()));
            let key = format!("{resp:X}");
            if !response_ids.contains(&key) {
                response_ids.push(key);
            }
        }
        let accepted = decoders.len();
        self.state.lock().unwrap().periodic = Some(PeriodicLive {
            decoders,
            fast_period_ms,
            tx: None,
        });
        self.log(
            "dvi_periodic_start",
            &format!(
                "{accepted} slot(s) @{fast_period_ms} ms fast, frames padded to 8{}",
                if capped > 0 {
                    format!(", {capped} message(s) capped")
                } else {
                    String::new()
                }
            ),
        );
        Ok(PeriodicInstall {
            accepted,
            response_ids,
            fast_period_ms,
            served_pids,
        })
    }

    /// Open the decode channel: slot replies routed by `on_rx` are sliced
    /// by the echo-keyed splitter off the transport thread (freshest frame
    /// per response id per wake) and handed to the sink as poll-shaped
    /// parses.
    pub(super) fn periodic_arm(&self, sink: PeriodicSink) -> Result<(), PeriodicError> {
        let (tx, rx) = std::sync::mpsc::channel::<(u32, Payload)>();
        let decoders: Vec<(
            u8,
            u32,
            Vec<(String, u8)>,
            std::collections::BTreeSet<String>,
        )> = {
            let mut st = self.state.lock().unwrap();
            let Some(p) = st.periodic.as_mut() else {
                return Err(PeriodicError::NotStarted);
            };
            p.tx = Some(tx);
            p.decoders.clone()
        };
        let logger = self.logger.clone();
        std::thread::Builder::new()
            .name("obd-dvi-periodic".into())
            .spawn(move || {
                let mut rx_frames = 0u64;
                let mut rx_signals = 0u64;
                let mut last_log = Instant::now();
                loop {
                    let first = match rx.recv() {
                        Ok(f) => f,
                        Err(_) => break,
                    };
                    let mut latest: Vec<(u32, Payload)> = vec![first];
                    while let Ok((id, p)) = rx.try_recv() {
                        match latest.iter_mut().find(|(k, _)| *k == id) {
                            Some(e) => e.1 = p,
                            None => latest.push((id, p)),
                        }
                    }
                    rx_frames += latest.len() as u64;
                    for (id, payload) in latest {
                        for (_, _resp, members, _) in
                            decoders.iter().filter(|(_, r, _, _)| *r == id)
                        {
                            let Ok(map) = crate::response_parser::split_multi_pid(
                                members,
                                std::slice::from_ref(&payload),
                            ) else {
                                continue;
                            };
                            let batch: Vec<(String, crate::response_parser::ParsedResponse)> =
                                map.into_iter().collect();
                            rx_signals += batch.len() as u64;
                            if !batch.is_empty() {
                                sink(batch);
                            }
                            break;
                        }
                    }
                    if last_log.elapsed() >= Duration::from_secs(5) {
                        last_log = Instant::now();
                        if let Some(ref l) = logger {
                            l.log_callback(
                                "stream_rx",
                                &format!("dvi frames={rx_frames} signals={rx_signals}"),
                            );
                        }
                        rx_frames = 0;
                        rx_signals = 0;
                    }
                }
            })
            .map_err(|e| PeriodicError::MonitorFailed(e.to_string()))?;
        self.log("dvi_periodic_armed", "slot replies → decode channel");
        Ok(())
    }

    /// Close the decode channel (no monitor mode to leave on DVI).
    pub(super) fn periodic_break(&self) {
        if let Some(p) = self.state.lock().unwrap().periodic.as_mut() {
            p.tx = None;
        }
    }

    /// Disable every installed slot, forget the plan.
    pub(super) fn periodic_teardown(&self) {
        let live = self.state.lock().unwrap().periodic.take();
        if let Some(p) = live {
            for (slot, _, _, _) in &p.decoders {
                if let Err(e) = self.exchange(
                    &codec::periodic_enable(*slot, false),
                    codec::cmd::CAN + 0x10,
                ) {
                    self.log(
                        "dvi_periodic_stop",
                        &format!("slot {slot} disable refused: {e}"),
                    );
                }
            }
            self.log(
                "dvi_periodic_stop",
                &format!("{} slot(s) disabled", p.decoders.len()),
            );
        }
    }

    /// The link is gone: forget, touch no wire.
    pub(super) fn periodic_abandon(&self) {
        self.state.lock().unwrap().periodic = None;
    }
}

/// PASS filter for a sniffer id list (LH3's adapter-agnostic filters → DVI).
pub fn pass_filter_frame(number: u8, id: u32, mask: u32, extended: bool) -> Vec<u8> {
    codec::can_filter(number, extended, FilterType::Pass, true, id, mask, 0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_pid_set_parses_mode01_and_mode22_wires() {
        let set = |v: &[&str]| {
            v.iter()
                .map(|s| s.to_string())
                .collect::<std::collections::BTreeSet<_>>()
        };
        assert_eq!(command_pid_set("010C0D"), set(&["010C", "010D"]));
        assert_eq!(command_pid_set("0105"), set(&["0105"]));
        assert_eq!(command_pid_set("22F190F191"), set(&["22F190", "22F191"]));
        assert!(command_pid_set("ATSH7E0").is_empty());
        assert!(command_pid_set("010C0").is_empty());
    }

    #[test]
    fn frames_become_payloads() {
        let b11 = Bus::new(Addressing::Can11);
        let b29 = Bus::new(Addressing::Can29 { tester: 0xF1 });
        let sf = RxFrame {
            id: 0x7E8,
            ts_us: None,
            data: vec![0x04, 0x41, 0x0C, 0x1A, 0xF8, 0x00, 0x00, 0x00],
        };
        let p = frame_payloads(&[sf.clone()], b11);
        assert_eq!(p.len(), 1);
        assert_eq!(
            p[0].1,
            Payload {
                controller: "7E0".into(),
                bytes: vec![0x41, 0x0C, 0x1A, 0xF8]
            }
        );
        assert!(p[0].2);
        assert_eq!(render_frame(&sf), "7E8 04 41 0C 1A F8 00 00 00");

        let f29 = RxFrame {
            id: 0x18DAF110,
            ts_us: None,
            data: vec![0x06, 0x41, 0x00, 0xBF, 0xFE, 0xB9, 0x93],
        };
        let p = frame_payloads(&[f29.clone()], b29);
        assert_eq!(
            p[0].1,
            Payload {
                controller: "10".into(),
                bytes: vec![0x41, 0x00, 0xBF, 0xFE, 0xB9, 0x93]
            }
        );
        assert_eq!(render_frame(&f29), "18 DA F1 10 06 41 00 BF FE B9 93");

        // Raw multi-frame: FF + CFs assemble to the declared 0x14 = 20 bytes.
        let ff = RxFrame {
            id: 0x7E8,
            ts_us: None,
            data: vec![0x10, 0x14, 0x49, 0x02, 0x01, b'1', b'C', b'4'],
        };
        let cf1 = RxFrame {
            id: 0x7E8,
            ts_us: None,
            data: vec![0x21, b'M', b'O', b'C', b'K', b'2', b'9', b'S'],
        };
        let cf2 = RxFrame {
            id: 0x7E8,
            ts_us: None,
            data: vec![0x22, b'B', b'I', b'T', b'0', b'0', b'0', b'1'],
        };
        assert!(!reply_complete(&[ff.clone(), cf1.clone()], 0x7E8, b11));
        let p = frame_payloads(&[ff, cf1, cf2], b11);
        assert_eq!(p.len(), 1);
        assert!(p[0].2);
        let mut want = vec![0x49, 0x02, 0x01];
        want.extend_from_slice(b"1C4MOCK29SBIT0001");
        assert_eq!(p[0].1.bytes, want);

        // Tool-reassembled multi-frame (auto frame processing): whole in one RX.
        let mut d = vec![0x10, 0x14, 0x49, 0x02, 0x01];
        d.extend_from_slice(b"1C4MOCK29SBIT0001");
        let p = frame_payloads(
            &[RxFrame {
                id: 0x7E8,
                ts_us: None,
                data: d,
            }],
            b11,
        );
        assert!(p[0].2);
        assert_eq!(p[0].1.bytes, want);

        assert!(is_obd_responder(0x7E8) && is_obd_responder(0x7EF) && is_obd_responder(0x18DAF110));
        assert!(
            !is_obd_responder(0x7E0) && !is_obd_responder(0x0A8) && !is_obd_responder(0x18DB33F1)
        );

        // Two ECUs answering a functional request: one payload each, in order.
        let a = RxFrame {
            id: 0x7E8,
            ts_us: None,
            data: vec![0x06, 0x41, 0x00, 0xBE, 0x1B, 0x30, 0x13],
        };
        let b = RxFrame {
            id: 0x7E9,
            ts_us: None,
            data: vec![0x06, 0x41, 0x00, 0x80, 0x00, 0x00, 0x00],
        };
        let p = frame_payloads(&[a, b], b11);
        assert_eq!(
            p.iter()
                .map(|(_, p, _)| p.controller.as_str())
                .collect::<Vec<_>>(),
            vec!["7E0", "7E1"]
        );
    }

    #[test]
    fn echo_guard_pins_reply_to_its_pid() {
        // Plain single-pid commands expose their positive echo…
        assert_eq!(expected_echo("22F40C"), Some(vec![0x62, 0xF4, 0x0C]));
        assert_eq!(expected_echo("010C"), Some(vec![0x41, 0x0C]));
        // …compound / AT commands skip the guard.
        assert_eq!(expected_echo("010C0D"), None);
        assert_eq!(expected_echo("010C|010D"), None);
        assert_eq!(expected_echo("ATSH7E0"), None);
        // response-byte extraction past the ISO-TP PCI (single + first frame).
        assert_eq!(
            frame_response_bytes(&[0x05, 0x62, 0xF4, 0x0C, 0x0B, 0xEC]),
            &[0x62, 0xF4, 0x0C, 0x0B, 0xEC]
        );
        assert_eq!(
            frame_response_bytes(&[0x10, 0x09, 0x62, 0xF4, 0x0C]),
            &[0x62, 0xF4, 0x0C]
        );
        // The guard's core predicate: a POSITIVE reply with the wrong echo
        // is lagged (side-delivered under its own command, never completes
        // the outstanding one); the right echo, or a 7F negative, completes.
        let is_lagged = |cmd: &str, data: &[u8]| {
            expected_echo(cmd).map_or(false, |e| {
                let r = frame_response_bytes(data);
                r.first().map_or(false, |&b| b & 0x40 != 0 && b != 0x7F) && !r.starts_with(&e)
            })
        };
        assert!(
            is_lagged("22F410", &[0x05, 0x62, 0xF4, 0x0C, 0x0B, 0xEC]),
            "62 F40C is not F410's reply"
        );
        assert!(
            is_lagged("010C", &[0x05, 0x62, 0xF4, 0x0C, 0x0B, 0xEC]),
            "cross-family: 62 F40C is not 010C's reply"
        );
        assert!(
            !is_lagged("22F40C", &[0x05, 0x62, 0xF4, 0x0C, 0x0B, 0xEC]),
            "62 F40C IS F40C's reply"
        );
        assert!(
            !is_lagged("22F40C", &[0x03, 0x7F, 0x22, 0x31]),
            "7F negative is accepted (no DID)"
        );
        // Re-attribution: the lagged reply delivers under the command its
        // echo names (`command_for_reply` is `expected_echo`'s inverse).
        assert_eq!(
            command_for_reply(&[0x62, 0xF4, 0x0C, 0x0B, 0xEC]),
            Some("22F40C".to_string())
        );
        assert_eq!(
            command_for_reply(&[0x41, 0x0C, 0x0B, 0xEC]),
            Some("010C".to_string())
        );
        assert_eq!(
            command_for_reply(&[0x7F, 0x22, 0x31]),
            None,
            "negatives are not re-attributable"
        );
    }

    #[test]
    fn response_ids() {
        assert_eq!(response_id(0x7E0, false), 0x7E8);
        assert_eq!(response_id(0x18DA10F1, true), 0x18DAF110);
        // All-numeric response ids MUST parse as hex, not decimal (the walk
        // regression that hid IPC/PSCM/ABS — Mustang 2026-09-04).
        assert_eq!(response_id(0x720, false), 0x728);
        assert_eq!(response_id(0x730, false), 0x738);
        assert_eq!(response_id(0x760, false), 0x768);
        assert_eq!(response_id(0x727, false), 0x72F);
    }
}
