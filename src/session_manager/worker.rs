//! SL3 — the lifecycle worker: ONE thread owns every lifecycle transition.
//!
//! The mailbox is the only way in. API calls and platform callbacks ENQUEUE
//! (cheap, lock-light, <50ms under any wedge — the SL0 beachball net); the
//! worker executes the hardware-proven bodies (engage ladder, stop sequences,
//! disconnect drain) one at a time, so "which teardown ran?" has exactly one
//! answer. The epoch (SL1) stays the only fence: enqueuers of user-terminal
//! intent claim/bump synchronously at the call site, and every in-handler wait
//! is bounded (`wait_timeout`/deadline + epoch check per iteration — handler
//! discipline: no bare `recv()`/`wait()` anywhere on this thread).
//!
//! A fatal transport drop is the one terminal source whose claim happens IN
//! the handler, not at enqueue: the claim must follow the stale-drop check
//! (our own close echo / a superseded attempt must not claim the NEW
//! session's terminal), and that check reads platform state — unsafe inside
//! the platform callback (iron rule 1). Queue latency is ms; the old
//! spawned-thread path had the same scheduling window.

use super::*;
use std::sync::mpsc;

/// SL3 phase enum — ONE source of truth for "what mode is the session in",
/// replacing the boolean derivations five threads used to re-derive from
/// flags. Written by the lifecycle worker's handlers (plus the connect flow
/// until the `Connect` migration lands); every transition logs a
/// `lifecycle_phase` line to the session jsonl. `stream_state` /
/// `periodic_state` remain PLUGIN-owned teardown handles (data, not
/// authority): the enum answers "what mode is the session in"; they answer
/// "what does this plugin need to tear down."
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Phase {
    Idle,
    Connecting {
        epoch: u64,
    },
    Connected {
        epoch: u64,
    },
    Engaging {
        epoch: u64,
        tier: crate::session_health::Tier,
    },
    Live {
        epoch: u64,
        tier: crate::session_health::Tier,
    },
    Stopping {
        epoch: u64,
        tier: crate::session_health::Tier,
    },
    Disconnecting {
        epoch: u64,
    },
}

/// SL3 mailbox — the full set. Discovery/picker stays host-interactive
/// OUTSIDE the mailbox; `Connect` is post-selection onward.
pub(super) enum LifecycleMsg {
    /// Post-selection connect: transport open → adapter init → identify →
    /// CONNECTED. The hardware-proven body (RB1/RB2) is CALLED, not
    /// rewritten; cancel/disconnect-mid-identify are ordinary epoch bails.
    Connect {
        connector: crate::platform::ConnectorInfo,
        generation: u64,
    },
    /// Run the engage ladder (poll → SP/UDS upgrade decision).
    Engage,
    /// Stop the LIVE acquisition plugin (stream or adapter-periodic), then
    /// optionally re-run the ladder (dashboard edits downgrade→upgrade).
    Stop { reengage: bool },
    /// User disconnect: bounded drain, transport close, terminal sequence if
    /// this caller won the SL1 claim (`owner`).
    Disconnect {
        generation: u64,
        owner: bool,
        platform: Arc<dyn OBDPlatformInterface>,
    },
    /// Platform reported the transport dropped (fatal drop or the echo of our
    /// own close). Stale-check → claim → local cleanup.
    Drop {
        platform_check: Option<Arc<dyn OBDPlatformInterface>>,
    },
    /// Session shutdown — exit the worker loop.
    Quit,
}

/// Spawn the lifecycle worker. Owns `rx` for the session's lifetime; exits on
/// `Quit`, a closed channel, or `running == false`.
pub(super) fn spawn_lifecycle_worker(
    api: SessionAPIHandle,
    rx: mpsc::Receiver<LifecycleMsg>,
) -> JoinHandle<()> {
    thread::Builder::new()
        .name("obd-lifecycle".into())
        .spawn(move || loop {
            // Bounded recv (handler discipline) — a dropped channel or a
            // shutdown flag can never park this thread forever.
            match rx.recv_timeout(Duration::from_millis(200)) {
                Ok(LifecycleMsg::Quit) | Err(mpsc::RecvTimeoutError::Disconnected) => break,
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    let running = api
                        .shared_state
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .running;
                    if !running {
                        break;
                    }
                }
                Ok(msg) => api.handle_lifecycle_msg(msg),
            }
        })
        .expect("spawn obd-lifecycle worker")
}

impl SessionAPIHandle {
    /// Current lifecycle phase (snapshot).
    pub(super) fn phase(&self) -> Phase {
        let state = self.shared_state.lock().unwrap_or_else(|p| p.into_inner());
        let p = *state.phase.lock().unwrap();
        p
    }

    /// Transition the phase enum (no-op when unchanged); every real
    /// transition lands in the session jsonl as `lifecycle_phase`.
    pub(super) fn set_phase(&self, next: Phase) {
        let (phase, logger) = {
            let state = self.shared_state.lock().unwrap_or_else(|p| p.into_inner());
            (Arc::clone(&state.phase), state.logger.clone())
        };
        let mut current = phase.lock().unwrap();
        if *current != next {
            log_cb!(
                logger,
                "lifecycle_phase",
                &format!("{:?} → {:?}", *current, next)
            );
            *current = next;
        }
    }

    /// The current session epoch (for phase payloads).
    pub(super) fn epoch_now(&self) -> u64 {
        let state = self.shared_state.lock().unwrap_or_else(|p| p.into_inner());
        let e = state
            .session_epoch
            .load(std::sync::atomic::Ordering::SeqCst);
        e
    }

    /// Enqueue a lifecycle message (never blocks; a dead worker means the
    /// session is shutting down and the message is moot).
    pub(super) fn send_lifecycle(&self, msg: LifecycleMsg) {
        let tx = {
            let state = self.shared_state.lock().unwrap_or_else(|p| p.into_inner());
            state.lifecycle_tx.clone()
        };
        let _ = tx.send(msg);
    }

    fn handle_lifecycle_msg(&self, msg: LifecycleMsg) {
        match msg {
            LifecycleMsg::Connect {
                connector,
                generation,
            } => self.connect_body(connector, generation),
            LifecycleMsg::Engage => self.engage_ladder(),
            LifecycleMsg::Stop { reengage } => self.stop_live_acquisition(reengage),
            LifecycleMsg::Disconnect {
                generation,
                owner,
                platform,
            } => self.disconnect_drain_body(generation, owner, platform),
            LifecycleMsg::Drop { platform_check } => self.drop_cleanup_body(platform_check),
            LifecycleMsg::Quit => {}
        }
    }

    /// SL3 `Drop` handler — runs ONLY on the lifecycle worker. The platform
    /// reported a transport drop; this was the callback's spawned thread.
    pub(super) fn drop_cleanup_body(&self, platform_check: Option<Arc<dyn OBDPlatformInterface>>) {
        use crate::connection_msg::ConnectionMsg;
        // Queue latency can land us AFTER a new connect attempt has taken the
        // transport (drop → instant reconnect). A stale DISCONNECTED would
        // stomp that attempt's UI and the abort would kill its in-flight
        // init/identify command — stand down.
        if let Some(ref p) = platform_check {
            if p.connection_status() != crate::platform::ConnectionStatus::Disconnected {
                return;
            }
        }
        // SL1: a genuine fatal drop is terminal intent — claim-then-bump.
        // The echo of OUR OWN close during a user disconnect loses the CAS
        // (the disconnect already claimed) and does not re-bump, so the
        // drain's epoch-checked waits stay intact.
        // SL2: only the claim WINNER emits — summary first, then the
        // terminal DISCONNECTED. A loser (user disconnect owns the
        // teardown) does the local cleanup below silently.
        let (_, owner) = self.claim_terminal();
        if owner {
            self.health_finish_and_emit();
            self.send_status(ConnectionMsg::DISCONNECTED, None, None);
            self.set_phase(Phase::Idle);
        }
        // S4: a drop mid-stream leaves no adapter to handshake
        // with — clear the stream state/tag locally so the NEXT
        // connect starts clean (defines on the car self-clear via
        // S3 once keepalives stop).
        {
            let state = self.shared_state.lock().unwrap();
            let stale = state.stream_state.lock().unwrap().take();
            if let Some(active) = stale {
                active
                    .runtime
                    .loop_stop
                    .store(true, std::sync::atomic::Ordering::SeqCst);
                if let Some(p) = state.command_processor.as_ref() {
                    p.set_hold_dispatch(false);
                }
            }
            *state.stream_tag.lock().unwrap() = None;
            state
                .user_pinned_poll
                .store(false, std::sync::atomic::Ordering::SeqCst); // fresh each connection
            state
                .teardowns_in_flight
                .store(0, std::sync::atomic::Ordering::SeqCst); // D7: no stale gate
                                                                // SP: same for a live adapter-periodic session — the
                                                                // adapter is gone (its ATZ on next connect clears the
                                                                // STPPMA slots), but hold_dispatch/decode loop must
                                                                // not survive into the next connect (bench
                                                                // 2026-08-01: user disconnected mid-periodic).
            let stale_p = state.periodic_state.lock().unwrap().take();
            if let Some(active) = stale_p {
                active.engine.abandon(); // no wire — the link is gone
                if let Some(p) = state.command_processor.as_ref() {
                    p.set_hold_dispatch(false);
                }
            }
        }
        self.emit_stream_state("stopped", serde_json::json!({}));
        let processor = self
            .shared_state
            .lock()
            .unwrap()
            .command_processor
            .as_ref()
            .map(Arc::clone);
        if let Some(processor) = processor {
            processor.abort_outstanding_and_clear();
        }
    }
}
