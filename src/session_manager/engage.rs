use super::*;

impl SessionAPIHandle {
    /// S4 engage owner: the DECISION runs on the lifecycle worker (SL3)
    /// because BOTH prerequisites land asynchronously after CONNECTED (VIN →
    /// tag handoff; dashboard pids). This entry point stays cheap: refusals
    /// and the one-at-a-time dedup happen synchronously (the SL0 harness
    /// observes `engage_pending` right after the call), then the ladder body
    /// is enqueued. Every mechanism the ladder drives is the hardware-proven
    /// S2 chain.
    pub fn maybe_engage_stream(&self) {
        // SL1: a beat arriving after terminal intent refuses AT ENTRY — the claim is
        // the fence. (A stale queued beat post-disconnect must not even enqueue;
        // equality checks in the ladder cover intent arriving mid-beat.)
        if self.terminal_intent_stands() {
            return;
        }
        let (engage_flag, engage_cancel) = {
            let state = self.shared_state.lock().unwrap();
            (
                Arc::clone(&state.engage_pending),
                Arc::clone(&state.engage_cancel),
            )
        };
        // Accepted-at-enqueue: the flag flips HERE (dedup + the harness's
        // synchronous observation), and the worker's ladder body clears it on
        // every exit path.
        if engage_flag.swap(true, std::sync::atomic::Ordering::SeqCst) {
            return; // an engage request is already queued/running
        }
        engage_cancel.store(false, std::sync::atomic::Ordering::SeqCst);
        // Badge pinned to poll → do not engage at all.
        if self
            .shared_state
            .lock()
            .unwrap()
            .user_pinned_poll
            .load(std::sync::atomic::Ordering::SeqCst)
        {
            engage_flag.store(false, std::sync::atomic::Ordering::SeqCst);
            return;
        }
        self.send_lifecycle(super::worker::LifecycleMsg::Engage);
    }

    /// The engage ladder body — SL3: runs ONLY on the lifecycle worker.
    /// `engage_pending` was set by the enqueuer; every exit path clears it.
    pub(super) fn engage_ladder(&self) {
        let (engage_flag, engage_cancel, platform) = {
            let state = self.shared_state.lock().unwrap();
            (
                Arc::clone(&state.engage_pending),
                Arc::clone(&state.engage_cancel),
                Arc::clone(&state.platform),
            )
        };
        let api = self;
        {
            let done = || engage_flag.store(false, std::sync::atomic::Ordering::SeqCst);
            // UDS attempts before the adapter-periodic fallback. Was 5
            // (~15 s of polling-speed dashboard when a car never takes
            // `10 03`); David 2026-08-30: "instead of sitting there failing
            // 5 times, drop the user to STN/DVI — every car that supports
            // UDS can do those." Two keeps the Mustang case (PCM shrugs off
            // the FIRST `10 03` right after connect, takes the next one a
            // beat later — bench: NO DATA at +3.7 s, success after) and
            // gets everyone else onto slots within ~6 s. The next trigger
            // (gauge add, sink release, pin change) re-runs the ladder, so a
            // late-blooming UDS car still gets its stream.
            const UDS_ATTEMPTS: u32 = 2;
            for attempt in 1..=UDS_ATTEMPTS {
                if api
                    .shared_state
                    .lock()
                    .unwrap()
                    .user_pinned_poll
                    .load(std::sync::atomic::Ordering::SeqCst)
                {
                    done();
                    return;
                }
                // Early bail BEFORE the settle sleep when this beat is already
                // doomed by the silent-fallback backoff: the periodic path is
                // held and there is no UDS alternative, so a 3 s settle would
                // just end in the same bail. Otherwise every re-triggered beat
                // during the 30 s backoff burns 3 s of the SINGLE lifecycle
                // worker, starving a queued Disconnect (Jeep OBDX BLE bench
                // 2026-09-03: disconnect stuck ~72 s behind doomed engage
                // beats). Narrow on purpose — a later trigger re-runs the
                // ladder, and the not-eligible / teardown-draining cases keep
                // their existing defer-then-recheck behavior below.
                {
                    let uds_ok = super::tier_facts::eligibility(&api.tier_facts())
                        .iter()
                        .any(|(t, v)| *t == crate::session_health::Tier::UdsStream && v.is_ok());
                    let held = api
                        .shared_state
                        .lock()
                        .unwrap()
                        .periodic_backoff_until_ms
                        .load(std::sync::atomic::Ordering::Relaxed)
                        > health_now_ms();
                    if held && !uds_ok {
                        done();
                        return;
                    }
                }
                // Sliced sleep: a sink (sniff/wizard/MCP) canceling us must
                // not wait out a full 3 s beat to take the transport.
                for _ in 0..20 {
                    thread::sleep(std::time::Duration::from_millis(150));
                    if engage_cancel.load(std::sync::atomic::Ordering::SeqCst) {
                        done();
                        return;
                    }
                    // SL1: terminal intent mid-beat — bail on the next tick.
                    if api.terminal_intent_stands() {
                        done();
                        return;
                    }
                }
                // D7: a teardown's wires may still be draining through the
                // correlator — engaging now pairs our first command with a
                // stale reply (garbage arm → watchdog drop). Defer a beat.
                {
                    // One acquisition for the teardown count + logger.
                    let (pending, logger) = {
                        let state = api.shared_state.lock().unwrap();
                        (
                            state
                                .teardowns_in_flight
                                .load(std::sync::atomic::Ordering::SeqCst),
                            state.logger.clone(),
                        )
                    };
                    if pending > 0 {
                        log_cb!(
                            logger,
                            "engage_deferred",
                            &format!("{pending} teardown(s) draining — waiting a beat")
                        );
                        continue;
                    }
                }
                {
                    let state = api.shared_state.lock().unwrap();
                    if state.stream_state.lock().unwrap().is_some() {
                        done();
                        return; // already live
                    }
                }
                if platform.connection_status() != crate::platform::ConnectionStatus::Connected {
                    done();
                    return;
                }
                // SL3 TierFacts: ONE snapshot per beat, PURE eligibility —
                // the tag is only worth carrying when the UDS tier is
                // eligible (pin/user-gate/no-profile all filter here; the
                // erasure idiom "disabled = no profile" is structural).
                let facts = api.tier_facts();
                let elig = super::tier_facts::eligibility(&facts);
                let uds_eligible = elig
                    .iter()
                    .any(|(t, v)| *t == crate::session_health::Tier::UdsStream && v.is_ok());
                let tag = if uds_eligible {
                    facts.stream_tag.clone()
                } else {
                    None
                };
                let has_pids = {
                    let state = api.shared_state.lock().unwrap();
                    let mgr = state.subscription_manager.lock().unwrap();
                    !mgr.active_continuous_pids().is_empty()
                };
                if !has_pids {
                    continue; // dashboard not populated yet
                }
                // ARM-COMMIT GATE (field 2026-08-07, third bite of this
                // class): pin/cancel/pause landing DURING the sliced sleep
                // went unseen — the checks above run before it — and the
                // task armed a doomed 0-signal stream against a transport a
                // takeover had just quiesced (MCP session_create racing the
                // previous bracket's release-spawned re-engage). Re-check
                // EVERYTHING cheap immediately before touching the wire;
                // has_pids again too — a host pause between the check above
                // and here empties the active set without setting any flag.
                if api.engage_commit_blocked(&engage_cancel) {
                    done();
                    return;
                }
                // SP fold: no UDS profile tag → this car polls. If the
                // adapter can run the loop itself (STPPMA), engage that;
                // otherwise there is nothing to engage — stay on poll.
                let Some(tag) = tag else {
                    // SP availability from the SAME snapshot; liveness is
                    // READINESS, checked at the arm point as ever.
                    let sp_eligible = elig
                        .iter()
                        .any(|(t, v)| *t == crate::session_health::Tier::StnPeriodic && v.is_ok());
                    let capable = {
                        let state = api.shared_state.lock().unwrap();
                        let live = state.periodic_state.lock().unwrap().is_some();
                        let held = state
                            .periodic_backoff_until_ms
                            .load(std::sync::atomic::Ordering::Relaxed)
                            > health_now_ms();
                        sp_eligible && !live && !held
                    };
                    if !capable {
                        done();
                        return;
                    }
                    match api.engage_stn_periodic(Arc::clone(&platform)) {
                        PeriodicEngage::Live | PeriodicEngage::StayOnPoll => {
                            done();
                            return;
                        }
                        PeriodicEngage::Retry => continue, // next beat
                    }
                };
                api.set_phase(super::worker::Phase::Engaging {
                    epoch: api.epoch_now(),
                    tier: crate::session_health::Tier::UdsStream,
                });
                api.emit_stream_state("engaging", serde_json::json!({}));
                let switch_t0 = std::time::Instant::now();
                match api.start_stream(Arc::clone(&platform), &tag) {
                    Ok(info) => {
                        // Tap attach handshake — never arm against a
                        // mis-wired tap (monitor lines would go nowhere and
                        // the first break would hang the beat).
                        if !api.monitor_control("attach", std::time::Duration::from_secs(3)) {
                            api.emit_stream_state(
                                "failed",
                                serde_json::json!({ "reason": "host tap attach timeout" }),
                            );
                            api.stop_stream(Arc::clone(&platform));
                            continue;
                        }
                        if let Err(e) = api.arm_stream_transport(Arc::clone(&platform)) {
                            api.emit_stream_state("failed", serde_json::json!({ "reason": e }));
                            api.stop_stream(Arc::clone(&platform));
                            continue;
                        }
                        api.emit_stream_state(
                            "live",
                            serde_json::json!({
                                "slotsUsed": info.slots_used,
                                "responseIds": info.response_ids,
                                "keepaliveIntervalMs": info.keepalive_interval_ms,
                                "excludedPids": info.excluded_pids,
                            }),
                        );
                        api.set_phase(super::worker::Phase::Live {
                            epoch: api.epoch_now(),
                            tier: crate::session_health::Tier::UdsStream,
                        });
                        api.log_acquisition_switch("poll→uds-stream", switch_t0);
                        done();
                        return;
                    }
                    Err(reason) => {
                        api.emit_stream_state(
                            "failed",
                            serde_json::json!({ "reason": reason, "attempt": attempt, "attempts": UDS_ATTEMPTS }),
                        );
                        // retry once — the PCM shrugging off an early 10 03
                        // is normal (bench: NO DATA at +3.7 s, success later)
                    }
                }
            }
            // Three-way fallback: the UDS stream never came up — before
            // settling for poll, try the adapter-periodic tier once.
            let facts = api.tier_facts();
            let sp_eligible = super::tier_facts::eligibility(&facts)
                .iter()
                .any(|(t, v)| *t == crate::session_health::Tier::StnPeriodic && v.is_ok());
            let capable = {
                let state = api.shared_state.lock().unwrap();
                let live = state.periodic_state.lock().unwrap().is_some()
                    || state.stream_state.lock().unwrap().is_some()
                    || state
                        .periodic_backoff_until_ms
                        .load(std::sync::atomic::Ordering::Relaxed)
                        > health_now_ms();
                sp_eligible && !live
            };
            if capable
                && platform.connection_status() == crate::platform::ConnectionStatus::Connected
                && !api.engage_commit_blocked(&engage_cancel)
            {
                let _ = api.engage_stn_periodic(Arc::clone(&platform));
            }
            if matches!(api.phase(), super::worker::Phase::Engaging { .. }) {
                api.set_phase(super::worker::Phase::Connected {
                    epoch: api.epoch_now(),
                });
            }
            done();
        }
    }

    /// Arm-commit gate: TRUE when an engage attempt must abort instead of
    /// touching the wire — the badge/session pinned poll, a takeover
    /// cancelled us, or every continuous subscription got paused since the
    /// attempt's entry checks (host-pause takeovers don't pin). Checked
    /// immediately before BOTH arm paths (UDS + SP fallback).
    fn engage_commit_blocked(&self, engage_cancel: &Arc<std::sync::atomic::AtomicBool>) -> bool {
        // SL1: the epoch check at the COMMIT point — intent that landed between the
        // beat's checks and the wire touch blocks the commit, observably.
        if self.terminal_intent_stands() {
            let logger = self.shared_state.lock().unwrap().logger.clone();
            log_cb!(logger, "engage_commit_blocked", "reason=epoch_stale");
            return true;
        }
        if engage_cancel.load(std::sync::atomic::Ordering::SeqCst) {
            return true;
        }
        let sub_mgr = {
            let state = self.shared_state.lock().unwrap();
            if state
                .user_pinned_poll
                .load(std::sync::atomic::Ordering::SeqCst)
            {
                return true;
            }
            Arc::clone(&state.subscription_manager)
        };
        let empty = sub_mgr.lock().unwrap().active_continuous_pids().is_empty();
        empty
    }

    /// SP: one full periodic engage attempt — setup, host tap handshake,
    /// arm. True = live (stream_state events emitted); false = unwound,
    /// polling untouched or resumed.
    fn engage_stn_periodic(
        &self,
        platform: Arc<dyn crate::platform::OBDPlatformInterface>,
    ) -> PeriodicEngage {
        // Nothing to acquire → Retry SILENTLY (no "engaging"/"failed"
        // events) — an empty active set is a normal transient during
        // dashboard edits and on the post-loop fallback.
        {
            let state = self.shared_state.lock().unwrap();
            let mgr = state.subscription_manager.lock().unwrap();
            if mgr.active_continuous_pids().is_empty() {
                return PeriodicEngage::Retry;
            }
        }
        let switch_t0 = std::time::Instant::now();
        match self.start_stn_periodic(Arc::clone(&platform)) {
            Ok(count) => {
                self.set_phase(super::worker::Phase::Engaging {
                    epoch: self.epoch_now(),
                    tier: crate::session_health::Tier::StnPeriodic,
                });
                self.emit_stream_state(
                    "engaging",
                    serde_json::json!({ "plugin": self.periodic_tier_name() }),
                );
                // Same tap handshake as the UDS stream — never arm against
                // a mis-wired tap (frames would go nowhere and the first
                // break would hang).
                if !self.monitor_control("attach", std::time::Duration::from_secs(3)) {
                    self.emit_stream_state(
                        "failed",
                        serde_json::json!({
                            "plugin": self.periodic_tier_name(),
                            "reason": "host tap attach timeout",
                        }),
                    );
                    self.stop_stn_periodic(Arc::clone(&platform));
                    self.set_phase(super::worker::Phase::Connected {
                        epoch: self.epoch_now(),
                    });
                    return PeriodicEngage::Retry;
                }
                if let Err(e) = self.arm_periodic_transport(Arc::clone(&platform)) {
                    self.emit_stream_state(
                        "failed",
                        serde_json::json!({ "plugin": self.periodic_tier_name(), "reason": e }),
                    );
                    self.stop_stn_periodic(Arc::clone(&platform));
                    self.set_phase(super::worker::Phase::Connected {
                        epoch: self.epoch_now(),
                    });
                    return PeriodicEngage::Retry;
                }
                self.emit_stream_state(
                    "live",
                    serde_json::json!({
                        "plugin": self.periodic_tier_name(),
                        "messages": count,
                    }),
                );
                self.set_phase(super::worker::Phase::Live {
                    epoch: self.epoch_now(),
                    tier: crate::session_health::Tier::StnPeriodic,
                });
                self.log_acquisition_switch(
                    &format!("poll→{}", self.periodic_tier_name()),
                    switch_t0,
                );
                PeriodicEngage::Live
            }
            Err(reason) if reason.starts_with("misfit:") => {
                // Loud once — diagnosable — then the ladder stands down.
                self.emit_stream_state(
                    "failed",
                    serde_json::json!({ "plugin": self.periodic_tier_name(), "reason": reason }),
                );
                PeriodicEngage::StayOnPoll
            }
            Err(_) => PeriodicEngage::Retry, // lengths teaching / transient — silent
        }
    }

    /// SP managed stop — enqueued to the lifecycle worker (SL3). `reengage`
    /// re-runs the engage ladder afterwards (used by live dashboard edits:
    /// the STPPMA set is adapter-side state, so an edit = stop → rebuild →
    /// re-arm). The unified `Stop` handler stops whichever plugin is live.
    pub fn request_stop_stn_periodic(&self, reengage: bool) {
        self.send_lifecycle(super::worker::LifecycleMsg::Stop { reengage });
    }

    /// S4 managed stop: break → host detaches the tap (handshake) →
    /// teardown → resume. Enqueued to the lifecycle worker (SL3); the host
    /// follows along via stream_state events. A live adapter-periodic session
    /// stops instead (one host-facing stop).
    pub fn request_stop_stream(&self) {
        self.send_lifecycle(super::worker::LifecycleMsg::Stop { reengage: false });
    }

    /// SL3 `Stop` handler — runs ONLY on the lifecycle worker. Stop is
    /// per-PLUGIN and reversible: run the LIVE plugin's stop body (session and
    /// link survive; no epoch bump), then optionally re-enqueue the ladder.
    /// Nothing live + `reengage` still re-runs the ladder (the SP edit path's
    /// not-live fast path).
    pub(super) fn stop_live_acquisition(&self, reengage: bool) {
        let (stream_live, periodic_live, platform) = {
            let state = self.shared_state.lock().unwrap();
            let stream_live = state.stream_state.lock().unwrap().is_some();
            let periodic_live = state.periodic_state.lock().unwrap().is_some();
            (stream_live, periodic_live, Arc::clone(&state.platform))
        };
        if stream_live {
            // D7: mark the teardown for the engage gate — held to the end of
            // the branch so every exit path (acked or local-only) releases.
            let _teardown =
                TeardownGuard::new(&self.shared_state.lock().unwrap().teardowns_in_flight);
            self.set_phase(super::worker::Phase::Stopping {
                epoch: self.epoch_now(),
                tier: crate::session_health::Tier::UdsStream,
            });
            self.emit_stream_state("stopping", serde_json::json!({}));
            let switch_t0 = std::time::Instant::now();
            self.break_stream_transport(Arc::clone(&platform));
            let detached = self.monitor_control("detach", std::time::Duration::from_secs(2));
            if detached {
                self.stop_stream(Arc::clone(&platform));
            } else {
                // If the host never acks the detach, skip the wire teardown
                // (the tap would swallow the replies and trip the 5 s
                // no-prompt drop) — no keepalives means S3 (~5 s) stops the
                // car side anyway — and still resume polling.
                let (stream_state, sub_mgr) = {
                    let state = self.shared_state.lock().unwrap();
                    (
                        Arc::clone(&state.stream_state),
                        Arc::clone(&state.subscription_manager),
                    )
                };
                let taken = stream_state.lock().unwrap().take();
                if let Some(active) = taken {
                    sub_mgr.lock().unwrap().resume_many(&active.paused_subs);
                    for id in &active.paused_subs {
                        self.kick_pipeline_if_idle(*id);
                    }
                }
            }
            self.emit_stream_state("stopped", serde_json::json!({}));
            self.log_acquisition_switch("uds-stream→poll", switch_t0);
            // Session and link survive a Stop — back to Connected, unless a
            // terminal path owns the ending (Disconnecting/Idle are theirs).
            if !self.terminal_intent_stands() {
                self.set_phase(super::worker::Phase::Connected {
                    epoch: self.epoch_now(),
                });
            }
        } else if periodic_live {
            // D7: RAII teardown marker — released once the wire drain completes.
            let _teardown =
                TeardownGuard::new(&self.shared_state.lock().unwrap().teardowns_in_flight);
            self.set_phase(super::worker::Phase::Stopping {
                epoch: self.epoch_now(),
                tier: crate::session_health::Tier::StnPeriodic,
            });
            // "stopping" FIRST — the host drops the badge and shows the
            // switching toast for the seconds the teardown takes (the
            // stream stop path already did this; the periodic path never
            // emitted it, so pause looked dead — bench 2026-08-02).
            self.emit_stream_state(
                "stopping",
                serde_json::json!({ "plugin": self.periodic_tier_name() }),
            );
            self.stop_stn_periodic(Arc::clone(&platform));
            self.emit_stream_state(
                "stopped",
                serde_json::json!({ "plugin": self.periodic_tier_name() }),
            );
            if !self.terminal_intent_stands() {
                self.set_phase(super::worker::Phase::Connected {
                    epoch: self.epoch_now(),
                });
            }
        }
        if reengage {
            self.maybe_engage_stream();
        }
    }

    pub fn stop_stream(&self, platform: Arc<dyn crate::platform::OBDPlatformInterface>) {
        // Belt-and-braces: if the host forgot phase 1, do it now (mock e2es
        // call stop_stream directly — their monitor mode is a closure, no
        // tap-vs-correlator distinction).
        self.break_stream_transport(Arc::clone(&platform));
        let (stream_state, processor, sub_mgr, logger) = {
            let state = self.shared_state.lock().unwrap();
            (
                Arc::clone(&state.stream_state),
                state.command_processor.as_ref().map(Arc::clone),
                Arc::clone(&state.subscription_manager),
                state.logger.clone(),
            )
        };
        let Some(active) = stream_state.lock().unwrap().take() else {
            return;
        };
        if let Some(processor) = processor {
            for cmd in ["2A04", "2C03", "1001"] {
                let _ = processor.queue_command(cmd.to_string(), 5000);
            }
            // BOUNDED wait — resumed poll wires queue FIFO behind the
            // teardown anyway, and even a fully failed teardown is
            // self-cleaning: no keepalive → S3 (~5 s) drops the session and
            // the schedule with it. A 10 s wait here held the stopping task
            // hostage for nothing.
            let _ = processor.wait_for_idle(std::time::Duration::from_secs(3));
        }
        sub_mgr.lock().unwrap().resume_many(&active.paused_subs);
        // FB1 lesson: resuming subscriptions does NOT revive the event chain —
        // it starved out while everything was paused. Kick each resumed sub
        // (guarded: only fires if the processor is truly idle).
        for id in &active.paused_subs {
            self.kick_pipeline_if_idle(*id);
        }
        log_cb!(logger, "stream_stop", "teardown complete");
        eprintln!("Stream: STOPPED (teardown complete, polling resumed)");
    }
}
