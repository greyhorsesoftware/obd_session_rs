//! `StnPeriodicEngine` — the OBDLink STN chip's adapter-periodic acquisition
//! (STPPMA) behind [`PeriodicEngine`] (LH4). Relocated verbatim from the
//! session's `sp_runtime` (bench-won sequences 2026-08-01…03): STCMM 1 +
//! STPPMA install with OUT OF MEMORY partial-install; the PACED arm (STPO,
//! STFAC, STFPA per id, STM verified over 5 tries); the break (space →
//! STOPPED, STFAC); the teardown (AT flush, verified STPPMC, pinned reopen
//! STPR → STP n → 0100, fallback STPC → 0100). The session keeps the policy
//! bracket around it: gate, plan building, quiesce, pause/resume, host tap
//! attach/detach, JSON/health delivery.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::elm::monitor_over;
use super::{
    AdapterFacts, LinkEvent, MonitorHandshake, PeriodicEngine, PeriodicError, PeriodicInstall,
    PeriodicPlan, PeriodicSink, PeriodicStats, ReplyWaiters,
};
use crate::addressing::{periodic_response_id, Addressing, Bus};
use crate::command_processor::CommandProcessor;
use crate::platform::OBDPlatformInterface;
use crate::session_logger::SessionLogger;

/// (response id, members[(pid, len)]) — a frame on the response id splits
/// against these members (echo-keyed; try-each).
type Decoders = Arc<Mutex<Vec<(String, Vec<(String, u8)>)>>>;

struct Live {
    decoders: Decoders,
    frame_ids: Arc<Mutex<Vec<String>>>,
    handshake: MonitorHandshake,
    fast_period_ms: u32,
}

pub struct StnPeriodicEngine {
    platform: Arc<dyn OBDPlatformInterface>,
    processor: Arc<CommandProcessor>,
    /// LH7: per-command reply slots the session's data callback fills.
    waiters: Arc<ReplyWaiters>,
    logger: Option<Arc<SessionLogger>>,
    facts: Arc<Mutex<AdapterFacts>>,
    live: Mutex<Option<Live>>,
}

impl StnPeriodicEngine {
    pub fn new(
        platform: Arc<dyn OBDPlatformInterface>,
        processor: Arc<CommandProcessor>,
        waiters: Arc<ReplyWaiters>,
        logger: Option<Arc<SessionLogger>>,
        facts: Arc<Mutex<AdapterFacts>>,
    ) -> Self {
        Self {
            platform,
            processor,
            waiters,
            logger,
            facts,
            live: Mutex::new(None),
        }
    }

    fn log(&self, key: &str, detail: &str) {
        if let Some(ref l) = self.logger {
            l.log_callback(key, detail);
        }
    }

    fn bus(&self) -> Bus {
        Bus::new(self.facts.lock().unwrap().addressing)
    }

    /// Request/response on the quiet bus: wait for THIS command's reply
    /// (the STN text, this being the STN dialect's own engine). An error
    /// outcome is None.
    fn exchange(&self, cmd: &str, timeout_ms: u32, wait_ms: u64) -> Option<String> {
        let slot = self.waiters.arm(cmd);
        if self
            .processor
            .queue_command(cmd.to_string(), timeout_ms)
            .is_err()
        {
            self.waiters.disarm(cmd);
            return None;
        }
        slot.wait(Duration::from_millis(wait_ms))?
            .ok()
            .map(|r| r.raw)
    }

    /// Clone the live handshake + tables out (never hold `live` across the wire).
    fn snapshot(&self) -> Option<(MonitorHandshake, Decoders, Arc<Mutex<Vec<String>>>)> {
        let g = self.live.lock().unwrap();
        g.as_ref().map(|l| {
            (
                l.handshake.clone(),
                Arc::clone(&l.decoders),
                Arc::clone(&l.frame_ids),
            )
        })
    }
}

impl PeriodicEngine for StnPeriodicEngine {
    fn is_live(&self) -> bool {
        self.live.lock().unwrap().is_some()
    }

    fn stats(&self) -> Option<PeriodicStats> {
        let g = self.live.lock().unwrap();
        g.as_ref().map(|l| PeriodicStats {
            members: l
                .decoders
                .lock()
                .unwrap()
                .iter()
                .map(|(_, m)| m.len())
                .sum(),
            fast_period_ms: l.fast_period_ms,
        })
    }

    fn start(&self, plan: PeriodicPlan) -> Result<PeriodicInstall, PeriodicError> {
        if self.is_live() {
            return Err(PeriodicError::NotStarted);
        }
        // STCMM 1: transmit-during-monitor so the periodics actually fire
        // (bench 2026-08-01: without it STM sits silent). STCMM monitoring
        // mode (OBDLink FRPM): 0 = receive-only, no CAN ACKs (DEFAULT —
        // passive, so STPPMA periodics never transmit); 1 = NORMAL NODE with
        // CAN ACKs (active bus participant → periodics fire); 2 = receive-all
        // incl errors, no ACK. We need 1. Bench-verified 2026-08-01.
        let _ = self.exchange("STCMM 1", 5000, 5000);

        // Fire each STPPMA, capture the returned handle; build the decoder
        // list (response id = physical header + 8 for 11-bit; members carry
        // learned lengths for the echo-keyed splitter).
        let mut decoders: Vec<(String, Vec<(String, u8)>)> = Vec::new();
        let mut resp_ids: Vec<String> = Vec::new();
        let mut accepted = 0usize;
        let mut fast_period_ms = u32::MAX;
        let mut served_pids: Vec<String> = Vec::new();
        for m in &plan.messages {
            let reply = self.exchange(&m.command(), 5000, 5000).unwrap_or_default();
            let r = reply.to_uppercase();
            if r.contains("OUT OF MEMORY") {
                // SP4 (partial): stop firing more, keep what installed.
                self.log("stn_periodic_oom", &m.data);
                break;
            }
            // Handle = the hex token in the reply (STN returns a bare hex).
            let handle = r
                .split_whitespace()
                .find(|t| u16::from_str_radix(t, 16).is_ok());
            if handle.is_none() {
                continue;
            }
            let resp = periodic_response_id(&m.header);
            if !resp_ids.contains(&resp) {
                resp_ids.push(resp.clone());
            }
            let members: Vec<(String, u8)> = m
                .pids
                .iter()
                .map(|p| (p.clone(), plan.lengths.get(p).copied().unwrap_or(1)))
                .collect();
            decoders.push((resp, members));
            served_pids.extend(m.pids.iter().cloned());
            accepted += 1;
            fast_period_ms = fast_period_ms.min(m.period_ms);
        }
        if accepted == 0 {
            return Err(PeriodicError::NoneAccepted);
        }
        let fast_period_ms = if fast_period_ms == u32::MAX {
            25
        } else {
            fast_period_ms
        };
        *self.live.lock().unwrap() = Some(Live {
            decoders: Arc::new(Mutex::new(decoders)),
            frame_ids: Arc::new(Mutex::new(resp_ids.clone())),
            handshake: MonitorHandshake::new(),
            fast_period_ms,
        });
        self.log("stn_periodic_start", &format!("{} message(s)", accepted));
        eprintln!("STN-periodic: STARTED ({} STPPMA message(s))", accepted);
        Ok(PeriodicInstall {
            accepted,
            response_ids: resp_ids,
            fast_period_ms,
            served_pids,
        })
    }

    fn arm(&self, sink: PeriodicSink) -> Result<(), PeriodicError> {
        let Some((handshake, decoders, frame_ids)) = self.snapshot() else {
            return Err(PeriodicError::NotStarted);
        };
        let response_ids: Vec<String> = frame_ids.lock().unwrap().clone();
        let bus = self.bus();

        // Decode channel + coalescing consumer (freshest per CAN-id key).
        // DECODE OFF THE TRANSPORT THREAD: the receive thread must do nothing
        // but enqueue. The thread exits when the channel closes (stop_monitor
        // drops the sender closure).
        let (line_tx, line_rx) = std::sync::mpsc::channel::<super::Frame>();
        let rx_lines = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let rx_signals = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let rx_last_log = Arc::new(Mutex::new(std::time::Instant::now()));
        let logger_for_rx = self.logger.clone();
        let decoders_for_rx = Arc::clone(&decoders);
        std::thread::spawn(move || {
            loop {
                let first = match line_rx.recv() {
                    Ok(l) => l,
                    Err(_) => break,
                };
                let mut latest: Vec<(String, super::Frame)> = Vec::new();
                let mut push = |frame: super::Frame| {
                    // SP29: keyed by the protocol-normalized CAN id ("7E8" /
                    // "18DAF110") so coalescing and decoder routing work on
                    // both header shapes (`Frame.id` IS that key).
                    let key = frame.id.clone();
                    match latest.iter_mut().find(|(k, _)| *k == key) {
                        Some(e) => e.1 = frame,
                        None => latest.push((key, frame)),
                    }
                };
                push(first);
                while let Ok(l) = line_rx.try_recv() {
                    push(l);
                }
                rx_lines.fetch_add(latest.len() as u64, std::sync::atomic::Ordering::Relaxed);
                {
                    let mut last = rx_last_log.lock().unwrap();
                    if last.elapsed() >= Duration::from_secs(5) {
                        *last = std::time::Instant::now();
                        if let Some(ref l) = logger_for_rx {
                            l.log_callback(
                                "stream_rx",
                                &format!(
                                    "stn lines={} signals={}",
                                    rx_lines.swap(0, std::sync::atomic::Ordering::Relaxed),
                                    rx_signals.swap(0, std::sync::atomic::Ordering::Relaxed)
                                ),
                            );
                        }
                    }
                }
                // Decoders live behind a mutex so edits could swap the decode
                // map without restarting the monitor.
                let decoders_now = decoders_for_rx.lock().unwrap().clone();
                for (id, frame) in latest {
                    // Route: try each decoder whose response id matches; the
                    // echo-keyed splitter cleanly parses only its own frame.
                    // The splitter is text-in: `frame.raw` keeps the header
                    // in the bus's own token shape (four tokens on 29-bit).
                    for (_resp, members) in decoders_now.iter().filter(|(r, _)| *r == id) {
                        let res = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            crate::response_parser::split_multi_pid_response(
                                members, &frame.raw, bus,
                            )
                        }));
                        let Ok(Ok(parsed_map)) = res else { continue };
                        let batch: Vec<(String, crate::response_parser::ParsedResponse)> =
                            parsed_map.into_iter().collect();
                        rx_signals
                            .fetch_add(batch.len() as u64, std::sync::atomic::Ordering::Relaxed);
                        if !batch.is_empty() {
                            sink(batch);
                        }
                        break;
                    }
                }
            }
        });

        // Enter monitor: tap routes STOPPED (stop/edit handshake), data
        // frames (→ decode), and everything else into the ack slot (SP's
        // ack slot deliberately takes ANY non-frame line — STFPA/STPO answer
        // OK — and any frame outside the periodic set).
        let stopped_for_tap = Arc::clone(&handshake.stopped_signal);
        let frame_ids_tap = Arc::clone(&frame_ids);
        let ack_for_tap = Arc::clone(&handshake.ack_slot);
        if let Err(e) = monitor_over(
            self.platform.as_ref(),
            bus,
            Box::new(move |ev: LinkEvent| {
                let stash_ack = |text: String| {
                    let (lock, cvar) = &*ack_for_tap;
                    *lock.lock().unwrap() = Some(text);
                    cvar.notify_all();
                };
                match ev {
                    LinkEvent::Stopped => {
                        let (lock, cvar) = &*stopped_for_tap;
                        *lock.lock().unwrap() = true;
                        cvar.notify_all();
                    }
                    LinkEvent::Frame(frame) => {
                        let is_ours = frame_ids_tap
                            .lock()
                            .unwrap()
                            .iter()
                            .any(|r| r.eq_ignore_ascii_case(&frame.id));
                        if is_ours {
                            let _ = line_tx.send(frame);
                        } else {
                            stash_ack(frame.raw);
                        }
                    }
                    LinkEvent::Text(text) => stash_ack(text.trim().to_string()),
                }
            }),
        ) {
            return Err(PeriodicError::MonitorFailed(e));
        }

        // Open the protocol right before monitoring: STPPMA periodics only
        // send while the protocol is OPEN, and "exiting a monitoring session
        // closes the protocol" (OBDLink FRPM §8.14) — so re-open here in case
        // the pause/drain window let it lapse. STPO opens the already-selected
        // (auto-detected at connect) protocol.
        // PACED arm (adapter log 0130): blasting STPO/STFAC/STFPA/STM as
        // one burst races the just-installed periodics' replies inside the
        // chip's parser — STM answers `>STOPPED` within ~20 ms and the
        // session sits deaf while Rust believes it armed. Send each setup
        // command alone and wait for its ack through the tap.
        // (SP's ack wait is deliberately NOT the stream's `wait_ack` — any
        // line acks here, no token match, no 7F-78 extension.)
        let ack_slot = Arc::clone(&handshake.ack_slot);
        let raw_ack = |cmd: &str| {
            {
                let (lock, _c) = &*ack_slot;
                *lock.lock().unwrap() = None;
            }
            self.platform.write_raw(cmd);
            let (lock, cvar) = &*ack_slot;
            let g = lock.lock().unwrap();
            let _ = cvar
                .wait_timeout_while(g, Duration::from_millis(1000), |st| st.is_none())
                .unwrap();
        };
        raw_ack("STPO");
        // Filters then STM (STM honors pass filters; STMA floods). Mask
        // covers every id bit of the active protocol (SP29): 11-bit `7FF`,
        // 29-bit `1FFFFFFF`.
        let mask = match bus.addressing {
            Addressing::Can11 => "7FF",
            Addressing::Can29 { .. } => "1FFFFFFF",
        };
        raw_ack("STFAC");
        for id in &response_ids {
            raw_ack(&format!("STFPA {id},{mask}"));
        }
        // STM alone — then VERIFY it stuck. The chip aborts a racing STM
        // with STOPPED almost immediately; watch the stopped signal and
        // retry until it holds. "Armed" must mean actually monitoring.
        let mut armed = false;
        for attempt in 1..=5u32 {
            handshake.clear_stopped();
            self.platform.write_raw("STM");
            // stopped seen inside 400 ms = the chip aborted the STM.
            let aborted = handshake.wait_stopped(Duration::from_millis(400));
            if !aborted {
                armed = true;
                if attempt > 1 {
                    self.log("stn_periodic_arm_retry", &attempt.to_string());
                }
                break;
            }
            std::thread::sleep(Duration::from_millis(150));
        }
        if !armed {
            return Err(PeriodicError::ArmAborted { attempts: 5 });
        }
        handshake
            .monitor_active
            .store(true, std::sync::atomic::Ordering::SeqCst);
        self.log("stn_periodic_armed", &response_ids.join(","));
        eprintln!(
            "STN-periodic: ARMED (monitoring {})",
            response_ids.join(",")
        );
        Ok(())
    }

    fn break_monitor(&self) {
        let Some((handshake, _, _)) = self.snapshot() else {
            return;
        };
        handshake
            .loop_stop
            .store(true, std::sync::atomic::Ordering::SeqCst);
        if handshake
            .monitor_active
            .load(std::sync::atomic::Ordering::SeqCst)
        {
            handshake.clear_stopped();
            self.platform.write_raw(" ");
            let _ = handshake.wait_stopped(Duration::from_millis(1500));
            self.platform.write_raw("STFAC");
            std::thread::sleep(Duration::from_millis(100));
            handshake
                .monitor_active
                .store(false, std::sync::atomic::Ordering::SeqCst);
        }
        // Leaves monitor mode → the tap closure (and the decode channel's
        // sender) drop → the decode thread exits.
        self.platform.exit_monitor_mode();
    }

    fn teardown(&self) {
        // Forget the live tables first (the wire below is the adapter's
        // cleanup, not ours).
        self.live.lock().unwrap().take();
        // Reopen the protocol before polling resumes. Hardware truth
        // (logs 0105 + 0116): after monitor mode the chip's
        // request/response receive path stays dead until a protocol
        // close→open cycle — STPO alone reports OK but polls get NO
        // DATA forever. The AUTO search costs ~4.9 s (log 020715),
        // so PIN the detected protocol first: STPR → STP <n> →
        // 0100 probe opens directly (~200 ms). If anything about
        // that is off (STPR unparsable, probe unanswered), fall
        // back to the proven STPC → 0100 full search.
        //
        // QUIESCE, then ONE verified clear (David, bench 2026-08-03
        // log 170701 + STN doc: exiting a monitoring session closes
        // the protocol; STPPMC clears every periodic slot at once —
        // the per-slot STPPMD loop ran ~2 s per delete against
        // still-draining residue and a TIMEOUT leaked a slot). The AT
        // flush is BEST-EFFORT with a short window (log 172223: a
        // sulking CX never answers it — that must not stall the stop);
        // the clear is verified and retried once (a lost clear =
        // adapter transmits forever).
        let _ = self.exchange("AT", 1000, 1200);
        std::thread::sleep(Duration::from_millis(250));
        if self.exchange("STPPMC", 2000, 2500).is_none() {
            let _ = self.exchange("STPPMC", 2000, 2500);
        }
        let mut reopened = false;
        if let Some(pr) = self.exchange("STPR", 2000, 2500) {
            // STPR's reply shape depends on how the protocol was
            // selected (bench 2026-08-02): AT/auto state reports the
            // ELM number tagged "[AT]" ("A6 [AT]"); an ST-pinned
            // state reports the STN number bare ("33"). Handle both.
            // ELM→STN per the FRPM High Speed CAN table: 33 = ISO
            // 15765 11-bit/500k, 34 = 29/500, 35 = 11/250,
            // 36 = 29/250 (31/32 = raw ISO 11898, not OBD; `STP 6`
            // answers `?` — bench 022330).
            let tok = pr.split_whitespace().next().unwrap_or("").to_uppercase();
            let elm = tok.strip_prefix('A').unwrap_or(&tok);
            let stn_proto: Option<String> =
                if tok.len() == 2 && tok.chars().all(|c| c.is_ascii_digit()) {
                    Some(tok.clone()) // already an STN number
                } else {
                    match elm {
                        "6" => Some("33".into()),
                        "7" => Some("34".into()),
                        "8" => Some("35".into()),
                        "9" => Some("36".into()),
                        _ => None,
                    }
                };
            if let Some(n) = stn_proto {
                let reply = self
                    .exchange(&format!("STP {n}"), 2000, 2500)
                    .unwrap_or_default()
                    .to_uppercase();
                if reply.contains("OK") {
                    self.log("stn_periodic_stop", &format!("STP {n} accepted"));
                }
                if let Some(resp) = self.exchange("0100", 10000, 8000) {
                    if resp.to_uppercase().contains("41 00") {
                        reopened = true;
                        self.log(
                            "stn_periodic_stop",
                            &format!("reopened pinned to STP {n} (no search)"),
                        );
                    }
                }
            }
        }
        if !reopened {
            let _ = self.exchange("STPC", 5000, 2500);
            let _ = self.exchange("0100", 10000, 8000);
        }
        let _ = self.processor.wait_for_idle(Duration::from_secs(3));
    }

    fn abandon(&self) {
        if let Some(l) = self.live.lock().unwrap().take() {
            l.handshake
                .loop_stop
                .store(true, std::sync::atomic::Ordering::SeqCst);
            l.handshake
                .monitor_active
                .store(false, std::sync::atomic::Ordering::SeqCst);
        }
    }
}
