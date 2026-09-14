use super::*;

/// LH4: the response-id rule lives in `addressing`; kept reachable here for
/// the stream's STPX filter and the unit tests.
pub(super) use crate::addressing::periodic_response_id;

impl SessionAPIHandle {
    // MARK: - SP: STN adapter-periodic acquisition (STPPMA)
    //
    // LH4: the wire lives in the handler's `PeriodicEngine`
    // (`link::stn_periodic_engine`). This file is the SESSION bracket around
    // it — gate, plan, quiesce, pause/resume, host tap, delivery.

    /// Start adapter-periodic acquisition: the STN chip fires the poll set on
    /// its own (STPPMA) and we monitor the responses — no host round-trip per
    /// sample. Gated on the adapter's periodic engine. Setup only; call
    /// `arm_periodic_transport` after to begin monitoring.
    pub fn start_stn_periodic(
        &self,
        _platform: Arc<dyn crate::platform::OBDPlatformInterface>,
    ) -> Result<usize, String> {
        let (
            engine,
            processor,
            sub_mgr,
            lengths,
            periodic_state,
            stream_state,
            logger,
            bus,
            primary_ecu,
            admission,
            tier_periods,
            concurrent,
        ) = {
            let state = self.shared_state.lock().unwrap();
            let Some(processor) = state.command_processor.as_ref().map(Arc::clone) else {
                return Err("no command processor".to_string());
            };
            let bus = crate::addressing::Bus::new(*state.addressing.lock().unwrap());
            let admission = state.acquisition.lock().unwrap().admission();
            let primary = *state.primary_ecu.lock().unwrap();
            let tier_periods = state.host_facts.lock().unwrap().tier_periods;
            let concurrent = state.link.facts().concurrent_rx;
            (
                state.link.periodic(),
                processor,
                Arc::clone(&state.subscription_manager),
                Arc::clone(&state.length_cache),
                Arc::clone(&state.periodic_state),
                Arc::clone(&state.stream_state),
                state.logger.clone(),
                bus,
                primary,
                admission,
                tier_periods,
                concurrent,
            )
        };
        let Some(engine) = engine else {
            return Err("adapter does not support STPPMA — staying on poll".to_string());
        };
        if stream_state.lock().unwrap().is_some() {
            return Err("a UDS stream is live".to_string());
        }
        if periodic_state.lock().unwrap().is_some() {
            return Err("periodic already active".to_string());
        }

        // Snapshot tiered, length-confirmed signals BEFORE pausing (paused
        // subs stop reporting active).
        let signals: Vec<(String, Option<String>, crate::subscription::RefreshTier)> = {
            let mgr = sub_mgr.lock().unwrap();
            mgr.active_continuous_pids()
                .into_iter()
                .map(|(pid, target)| {
                    let tier = match mgr.get_pid_tier(&pid).as_deref() {
                        Some("Fast") => crate::subscription::RefreshTier::Fast,
                        Some("Slow") => crate::subscription::RefreshTier::Slow,
                        _ => crate::subscription::RefreshTier::Medium,
                    };
                    (pid, target, tier)
                })
                .collect()
        };
        if signals.is_empty() {
            return Err("no active pids to poll".to_string());
        }
        // DV3 (David 2026-08-29): on a concurrent link EVERY tier rides the
        // adapter's slots — Slow at its 500 ms period costs nothing and, with
        // no continuous pid left to poll, the completion-driven poll chain
        // idles instead of hammering the lone Slow pid (bench 20:06: `0105`
        // polled 17×/s on BLE, competing with the slot downlink). Pids past
        // the slot cap stay polled (`served_pids` excludes them).
        // POLL-FIRST GATE (design 2026-08-02): polling needs no lengths and
        // TEACHES them — streaming engages only once every active pid has a
        // learned/seeded length AND fits a single-frame response. Unknown
        // length = normal startup (the ladder retries silently; the first
        // poll sweep satisfies it). A true misfit can NEVER stream — the
        // dashboard stays on polling until that gauge is removed. (Mode 09
        // is filtered out of the datapoint store entirely — not a gauge.)
        let mut plan_lengths: std::collections::HashMap<String, u8> =
            std::collections::HashMap::new();
        {
            let cache = lengths.lock().unwrap();
            for (pid, _, _) in &signals {
                let Some(len) = cache.get(pid) else {
                    return Err(format!("waiting for lengths: {pid}"));
                };
                // Single-frame reply budget applies to DVI periodic TOO, not
                // just STN monitor: neither does tester flow control for the
                // autonomous slot responses, so a multi-frame slot reply
                // stalls at the FirstFrame — the ECU sends `10 09 …` and the
                // consecutive frame never comes (no FC), so the splitter hits
                // a misaligned echo and drops the whole frame (Mustang OBDX
                // 2026-09-04: 2 mode-22 DIDs/slot → 9-byte reply → 0 decoded,
                // silent-watchdog drop). The earlier "DVI assembles it" note
                // held for one mode-01 case but is not reliable. A pid whose
                // SOLO reply can't fit one frame stays on poll on both.
                if 1 + crate::stn_periodic::member_cost(pid, len) > 7 {
                    return Err(format!(
                        "misfit: {pid} response cannot fit one frame — staying on poll"
                    ));
                }
                plan_lengths.insert(pid.clone(), len);
            }
        }
        // SP29: render request headers for the ACTIVE protocol. Targets
        // resolve through Bus exactly like polling (C3 contract); the
        // target-free default is the primary (engine) ECU — `7E0` on 11-bit,
        // `18DA10F1` on 29-bit. A target with no physical mapping in this
        // protocol (an imported 11-bit CSV header on a 29-bit bus) would
        // have to go functional — a periodic multi-ECU flood — so it fails
        // the engage instead and the dashboard stays on poll. (Pre-pause:
        // erring here leaves polling untouched.)
        let default_ctrl = primary_ecu.unwrap_or(match bus.addressing {
            crate::addressing::Addressing::Can11 => crate::addressing::Controller::new(0x7E0),
            crate::addressing::Addressing::Can29 { .. } => crate::addressing::Controller::new(0x10),
        });
        let default_header = bus.request_header(default_ctrl);
        let mut resolved: Vec<(String, Option<String>, crate::subscription::RefreshTier)> =
            Vec::with_capacity(signals.len());
        for (pid, target, tier) in signals {
            let header = match target {
                None => None,
                Some(t) => match bus.from_target(&t) {
                    Some(c) => Some(bus.request_header(c)),
                    None => {
                        return Err(format!(
                            "target {t} has no physical address in this protocol — staying on poll"
                        ));
                    }
                },
            };
            resolved.push((pid, header, tier));
        }
        // single_frame_reply = true for BOTH tiers: DVI periodic has no
        // tester flow control for slot replies either, so each slot's packed
        // reply must fit one CAN frame (Mustang 2026-09-04 — see the misfit
        // gate above). This packs fewer DIDs per slot (mode-22: ~1/slot) but
        // every slot reply is then single-frame and decodes.
        let (messages, leftovers) = crate::stn_periodic::build_periodic_set(
            &resolved,
            &admission,
            &default_header,
            &tier_periods,
            true,
        );
        debug_assert!(leftovers.is_empty(), "gate admits only budgetable pids");
        if messages.is_empty() {
            return Err("no periodic-eligible signals".to_string());
        }
        let planned = messages.len();

        // QUIESCE FIRST (bench 2026-08-01): pause + drain BEFORE any STPPMA
        // exchange. A poll wire firing during setup (e.g. 010C0D) would land
        // its response after the tap attaches → no `>` prompt → 5 s fallback
        // DROP (same round-6 hazard the stream engage fixed by quiescing
        // first). Signals were snapshot above, before this pause.
        // DV3: a concurrent link (DVI) doesn't PAUSE the poll subscriptions —
        // slots and polling coexist at steady state (the picker skips served
        // pids). BUT it must not let physical polls INTERLEAVE with the
        // slot-install exchange itself: on BLE, `10 07` polls fired between
        // the `34 04 1A` / `34 0E 1B` / `34 03 1C` setup frames contend for
        // the write buffer and desync the tool's parser (Mustang OBDX bench
        // 2026-09-04). So HOLD command dispatch across the install (the
        // engine's own setup writes bypass the processor, so they still go)
        // and release it once slots are armed. Non-concurrent (STN) keeps the
        // full quiesce-then-pause below.
        let paused = if concurrent {
            processor.set_hold_dispatch(true);
            Vec::new()
        } else {
            let paused = sub_mgr.lock().unwrap().pause_all_active();
            processor.clear_queue();
            sub_mgr.lock().unwrap().clear_in_flight();
            let _ = processor.wait_for_idle(std::time::Duration::from_secs(3));
            paused
        };

        let served_pids = match engine.start(crate::link::PeriodicPlan {
            messages,
            lengths: plan_lengths,
        }) {
            Ok(install) => {
                if concurrent {
                    let served: std::collections::HashSet<String> =
                        install.served_pids.iter().cloned().collect();
                    log_cb!(
                        logger,
                        "periodic_coexist",
                        &format!("{} pid(s) on slots, rest polled", served.len())
                    );
                    sub_mgr.lock().unwrap().set_streamed_pids(served);
                    install.served_pids
                } else {
                    Vec::new()
                }
            }
            Err(e) => {
                // Post-pause failure MUST resume polling (quiesce-first pause
                // happened above) or the dashboard stays dead on a poll-only
                // adapter/car.
                if concurrent {
                    processor.set_hold_dispatch(false); // release the install hold
                } else {
                    sub_mgr.lock().unwrap().resume_many(&paused);
                    for id in &paused {
                        self.kick_pipeline_if_idle(*id);
                    }
                }
                return Err(e.to_string());
            }
        };

        *periodic_state.lock().unwrap() = Some(ActivePeriodic {
            paused_subs: paused,
            engine,
            served_pids,
        });
        {
            // Watchdog clocks: nothing counts until the engine is ARMED (the
            // STN arm is paced — STPO/STFAC/STFPA/STM — and the host tap
            // handshake sits in between; an install-time stamp would burn
            // the 3 s budget before a frame could possibly arrive).
            let state = self.shared_state.lock().unwrap();
            state
                .periodic_last_rx_ms
                .store(0, std::sync::atomic::Ordering::Relaxed);
            state
                .periodic_armed_ms
                .store(0, std::sync::atomic::Ordering::Relaxed);
        }
        if concurrent {
            // Slots installed — resume physical polling alongside the live
            // slots (steady-state coexistence). Kicks the queued polls that
            // piled up behind the hold.
            processor.set_hold_dispatch(false);
        } else {
            // Monitor owns the transport — hold command dispatch (released at
            // stop). Not on a concurrent link: nothing owns the transport.
            processor.set_hold_dispatch(true);
        }
        let _ = &logger;
        Ok(planned)
    }

    /// SP: begin monitoring the periodic responses. Call AFTER the host has
    /// attached its line tap. No keepalive beat — the adapter fires the
    /// periodics; we just filter (STFPA response ids) + STM + decode.
    pub fn arm_periodic_transport(
        &self,
        _platform: Arc<dyn crate::platform::OBDPlatformInterface>,
    ) -> Result<(), String> {
        let (engine, callback, processor, logger, health) = {
            let state = self.shared_state.lock().unwrap();
            let guard = state.periodic_state.lock().unwrap();
            let Some(active) = guard.as_ref() else {
                return Err("no periodic to arm".to_string());
            };
            (
                Arc::clone(&active.engine),
                Arc::clone(&state.response_callback_shared),
                state.command_processor.as_ref().map(Arc::clone),
                state.logger.clone(),
                Arc::clone(&state.health),
            )
        };
        // Delivery: each decoded signal becomes an `obd_data` message (the
        // parsed carries raw_hex), counts as a measured delivery for Session
        // Stats, and feeds the health tracker (stn-periodic tier signals).
        let last_rx = Arc::clone(&self.shared_state.lock().unwrap().periodic_last_rx_ms);
        let sink: crate::link::PeriodicSink = Arc::new(move |batch| {
            last_rx.store(health_now_ms(), std::sync::atomic::Ordering::Relaxed);
            let mut delivered = 0u32;
            for (pid, parsed) in batch {
                let raw = format!("{:?}", &parsed);
                let message = message_builder::obd_data_message(
                    pid,
                    raw,
                    None,
                    serde_json::to_value(&parsed).ok(),
                );
                if let Ok(json) = MessageProcessor::serialize_message(&message) {
                    callback(json);
                    delivered += 1;
                }
            }
            if let Some(ref p) = processor {
                p.record_signal_completions(delivered as u64);
            }
            health_note_signals(&health, delivered as u64, &logger, &callback);
        });
        engine.arm(sink).map_err(|e| e.to_string())?;
        // Armed: the silent-slot watchdog's clock starts now.
        self.shared_state
            .lock()
            .unwrap()
            .periodic_armed_ms
            .store(health_now_ms(), std::sync::atomic::Ordering::Relaxed);
        Ok(())
    }

    /// SP: stop periodic acquisition — break monitor, clear periodics
    /// (STPPMC), resume polling. Mirrors stop_stream's break→teardown→resume.
    pub fn stop_stn_periodic(&self, _platform: Arc<dyn crate::platform::OBDPlatformInterface>) {
        let switch_t0 = std::time::Instant::now();
        let (periodic_state, processor, sub_mgr, logger) = {
            let state = self.shared_state.lock().unwrap();
            (
                Arc::clone(&state.periodic_state),
                state.command_processor.as_ref().map(Arc::clone),
                Arc::clone(&state.subscription_manager),
                state.logger.clone(),
            )
        };
        let engine = {
            let guard = periodic_state.lock().unwrap();
            let Some(active) = guard.as_ref() else { return };
            Arc::clone(&active.engine)
        };
        // Break monitor (space → STOPPED, STFAC) — back at the prompt.
        engine.break_monitor();

        // HOST TAP DETACH (hardware 2026-08-02, log 002537): the fold's
        // engage attached the Swift line tap via monitor_control("attach");
        // without the paired detach every post-stop response kept flowing
        // into the dead tap instead of the correlator — STPPMD/STPPMC/AT all
        // TIMEOUT → frozen dash → 5 s no-prompt DROP. (The mock never caught
        // it: mock command replies bypass the line tap.) If the host never
        // acks, SKIP the wire teardown — those commands would just time out;
        // the adapter clears its STPPMA slots on the next connect's ATZ.
        let tap_detached = self.monitor_control("detach", std::time::Duration::from_secs(3));

        let Some(active) = periodic_state.lock().unwrap().take() else {
            return;
        };
        if let Some(ref processor) = processor {
            processor.set_hold_dispatch(false);
            if tap_detached {
                engine.teardown();
            } else {
                engine.abandon();
                log_cb!(
                    logger,
                    "stn_periodic_stop",
                    "host tap never detached — skipping wire teardown"
                );
            }
        } else {
            engine.abandon();
        }
        sub_mgr.lock().unwrap().resume_many(&active.paused_subs);
        for id in &active.paused_subs {
            self.kick_pipeline_if_idle(*id);
        }
        // DV3 (concurrent link): the slot-served pids return to polling —
        // clear the skip set and kick the chain in case every active pid
        // had been on a slot (chain idle with nothing to pick).
        let kick: Vec<uuid::Uuid> = {
            let mut mgr = sub_mgr.lock().unwrap();
            mgr.set_streamed_pids(std::collections::HashSet::new());
            mgr.get_active_subscriptions()
                .iter()
                .map(|s| s.id)
                .collect()
        };
        for id in kick {
            self.kick_pipeline_if_idle(id);
        }
        log_cb!(logger, "stn_periodic_stop", "teardown complete");
        self.log_acquisition_switch(&format!("{}→poll", self.periodic_tier_name()), switch_t0);
        eprintln!("STN-periodic: STOPPED (periodics cleared, polling resumed)");
    }
}
