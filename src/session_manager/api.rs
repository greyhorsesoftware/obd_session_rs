use super::*;

impl SessionAPIHandle {
    /// Send a single command
    ///
    /// Returns immediately. Response is delivered via the configured callback.
    pub fn send_command(&self, command: &str, timeout_ms: u32) -> Result<(), SessionError> {
        let state = self.shared_state.lock().unwrap();

        // Log direct command
        if let Some(ref logger) = state.logger {
            logger.log_command_sent(command, timeout_ms);
        }

        if let Some(ref processor) = state.command_processor {
            processor.queue_command(command.to_string(), timeout_ms)
        } else {
            Err(SessionError::InternalError(
                "Command processor not available".to_string(),
            ))
        }
    }

    /// Send batch commands
    ///
    /// Returns immediately. Responses are delivered via the configured callback.
    pub fn send_batch_commands(
        &self,
        commands: Vec<String>,
        timeout_ms: u32,
    ) -> Result<(), SessionError> {
        let state = self.shared_state.lock().unwrap();

        if let Some(ref processor) = state.command_processor {
            for command in commands {
                processor.queue_command(command, timeout_ms)?;
            }
            Ok(())
        } else {
            Err(SessionError::InternalError(
                "Command processor not available".to_string(),
            ))
        }
    }

    /// Create new subscription
    pub fn create_subscription(
        &self,
        name: Option<String>,
        pids: Vec<String>,
        target_controller: Option<String>,
    ) -> Result<Uuid, SessionError> {
        self.create_subscription_with_run_counts(name, pids, target_controller, None)
    }

    /// Create subscription with run count control
    /// run_counts: None = continuous, Some(count) = run specified number of times per PID
    pub fn create_subscription_with_run_counts(
        &self,
        name: Option<String>,
        pids: Vec<String>,
        target_controller: Option<String>,
        run_counts: Option<std::collections::HashMap<String, Option<u32>>>,
    ) -> Result<Uuid, SessionError> {
        let shared_state = self.shared_state.lock().unwrap();
        let mut subscription_manager = shared_state.subscription_manager.lock().unwrap();
        let subscription_id = subscription_manager.create_subscription(name)?;

        // 6b (DVI_BLE_TX_Plan): add BEFORE logging, unwind on failure — a
        // rejected pid list must leave NO registered subscription and NO
        // `create` in the log. The old create→log→add order left an empty
        // zombie plus a create event that looked successful, which masked
        // the mustang50 probe abort (2026-08-31).
        let create_details = {
            let run_counts_str = run_counts
                .as_ref()
                .map(|rc| format!("{:?}", rc))
                .unwrap_or_else(|| "continuous".to_string());
            format!(
                "pids={:?} controller={:?} run_counts={}",
                pids, target_controller, run_counts_str
            )
        };
        if let Err(e) = subscription_manager.add_pids_to_subscription_with_run_counts(
            subscription_id,
            pids,
            target_controller,
            run_counts,
        ) {
            let _ = subscription_manager.cancel_subscription(subscription_id);
            return Err(SessionError::InternalError(format!(
                "Failed to add PIDs: {:?}",
                e
            )));
        }

        if let Some(ref logger) = shared_state.logger {
            logger.log_subscription("create", &subscription_id.to_string(), &create_details);
        }

        Ok(subscription_id)
    }

    /// FB1: revive the polling chain when work is added to a QUIET session.
    /// The completion-driven chain dies when `get_next_command` returns None
    /// (e.g. every PID was removed); adding PIDs alone never restarts it —
    /// the dashboard stayed dead after remove-all → re-add. Same guarded
    /// kick as `start_subscription`: only when the processor is idle, and an
    /// idle processor means any in-flight markers are stale (their response
    /// never arrived) — clear them or the scheduler starves those PIDs.
    /// `only_if_active`: skip the kick unless the subscription is Active
    /// (adding to a paused sub — e.g. during sink mode — must stay quiet).
    pub(super) fn kick_pipeline_if_idle(&self, subscription_id: Uuid) {
        let (processor, sub_mgr, timeout) = {
            let state = self.shared_state.lock().unwrap();
            (
                state.command_processor.as_ref().map(Arc::clone),
                Arc::clone(&state.subscription_manager),
                state.default_timeout_ms,
            )
        };
        let is_active = sub_mgr
            .lock()
            .unwrap()
            .get_subscription_info(subscription_id)
            .map(|info| info.state == crate::subscription::SubscriptionState::Active)
            .unwrap_or(false);
        if !is_active {
            return;
        }
        if let Some(processor) = processor {
            // CAS on the chain-liveness flag, NOT a momentary idle probe: a
            // live chain looks idle between completing one command and
            // queueing the next (that window includes the whole data
            // callback), and a kick landing there birthed a second chain —
            // pipeline permanently 2 deep. Claiming liveness is race-free;
            // when the claim fails the live chain picks up this
            // subscription's PIDs on its next completion.
            if processor.try_claim_chain() {
                sub_mgr.lock().unwrap().clear_in_flight();
                let _ = processor.queue_command("AT".to_string(), timeout);
            }
        }
    }

    /// Shared head of add/remove_pids: under the state lock — log the
    /// action, snapshot which acquisitions are live, run `op` on the
    /// subscription manager — then release every lock before returning.
    /// Returns (stream_live, periodic_live, op result).
    fn pid_change_locked(
        &self,
        subscription_id: Uuid,
        log_action: &str,
        log_detail: &str,
        err_label: &str,
        op: impl FnOnce(&mut SubscriptionManager) -> Result<(), crate::error::SubscriptionError>,
    ) -> (bool, bool, Result<(), SessionError>) {
        let shared_state = self.shared_state.lock().unwrap();
        if let Some(ref logger) = shared_state.logger {
            logger.log_subscription(log_action, &subscription_id.to_string(), log_detail);
        }
        let stream_live = shared_state.stream_state.lock().unwrap().is_some();
        let periodic_live = shared_state.periodic_state.lock().unwrap().is_some();
        let result = {
            let mut subscription_manager = shared_state.subscription_manager.lock().unwrap();
            op(&mut subscription_manager).map_err(|e| {
                SessionError::InternalError(format!("Failed to {}: {:?}", err_label, e))
            })
        };
        (stream_live, periodic_live, result)
    }

    /// Add PIDs to subscription
    pub fn add_pids_to_subscription(
        &self,
        subscription_id: Uuid,
        pids: Vec<String>,
        target_controller: Option<String>,
    ) -> Result<(), SessionError> {
        let (stream_live, periodic_live, result) = self.pid_change_locked(
            subscription_id,
            "add_pids",
            &format!("pids={:?} controller={:?}", pids, target_controller),
            "add PIDs",
            |m| {
                m.add_pids_to_subscription(subscription_id, pids.clone(), target_controller.clone())
            },
        );
        result?;
        // SH1: user edit — engage/teardown around it is not ladder churn.
        self.health_on_subscription_edit();
        // FB1: a dead chain (all PIDs previously removed) must be revived.
        self.kick_pipeline_if_idle(subscription_id);
        // S4: while streaming, an added pid is an in-gap define — callers
        // never touch the stream surface.
        if stream_live {
            for pid in &pids {
                self.stream_change_signal(pid, true);
            }
        }
        // SP: any selection change DOWNGRADES to polling, then the
        // poll-first ladder upgrades again (David's design 2026-08-02).
        // The surgical STPPMD edit is RETIRED on hardware evidence: the STN
        // refuses to re-enter monitor mode mid-session (adapter log 0116:
        // full re-arm wire answered `>STOPPED` 14 ms after STM) — only a
        // from-scratch engage is honored.
        if periodic_live {
            self.request_stop_stn_periodic(true);
        }
        Ok(())
    }

    /// Remove PIDs from subscription
    pub fn remove_pids_from_subscription(
        &self,
        subscription_id: Uuid,
        pids: Vec<String>,
    ) -> Result<(), SessionError> {
        let (stream_live, periodic_live, result) = self.pid_change_locked(
            subscription_id,
            "remove_pids",
            &format!("pids={:?}", pids),
            "remove PIDs",
            |m| m.remove_pids_from_subscription(subscription_id, pids.clone()),
        );
        // SH1: user edit — engage/teardown around it is not ladder churn.
        if result.is_ok() {
            self.health_on_subscription_edit();
        }
        // S4: while streaming, a removed pid drops from the decode map.
        if stream_live {
            for pid in &pids {
                self.stream_change_signal(pid, false);
            }
        }
        // SP: downgrade → re-engage on any selection change (see
        // add_pids_to_subscription — surgical edit retired).
        if periodic_live {
            self.request_stop_stn_periodic(true);
        }
        result
    }

    /// Start subscription polling
    ///
    /// With event-driven architecture, this triggers the first command.
    pub fn start_subscription(&self, subscription_id: Uuid) -> Result<(), SessionError> {
        let shared_state = self.shared_state.lock().unwrap();

        // Log start
        if let Some(ref logger) = shared_state.logger {
            logger.log_subscription("start", &subscription_id.to_string(), "");
        }

        {
            let mut subscription_manager = shared_state.subscription_manager.lock().unwrap();
            subscription_manager
                .start_subscription(subscription_id)
                .map_err(|e| {
                    SessionError::InternalError(format!("Failed to start subscription: {:?}", e))
                })?;
        }

        drop(shared_state);

        // Send AT\r first to flush adapter, then trigger the first PID command.
        // The AT\r response ("OK") flows through the data callback which
        // calls get_next_command, starting the real polling loop.
        //
        // Kick ONLY when the pipeline is dry: every kick births an independent
        // completion-driven chain, and chains never die — they keep pulling the
        // next eligible PID from the shared scheduler even after their own
        // subscription completes. Starting dashboard while enable-tests was
        // still polling left the pipeline permanently 2 deep (two "in-flight"
        // PIDs in Session Stats, extra queue latency on every poll). A live
        // chain picks up this subscription's PIDs on its next completion.
        self.kick_pipeline_if_idle(subscription_id);

        // S4: a started subscription is the engage trigger — streaming is an
        // ACQUISITION choice now, not a host-driven feature. No-op unless the
        // tag/pids/connection prerequisites hold (checked in the task).
        // A (re)start is also the user's "try again" after a silent-slot
        // teardown (David 2026-08-30): lift the watchdog's re-engage hold —
        // it exists to stop AUTOMATIC flapping, and the watchdog's own stop
        // path never comes through here.
        self.shared_state
            .lock()
            .unwrap()
            .periodic_backoff_until_ms
            .store(0, std::sync::atomic::Ordering::Relaxed);
        self.maybe_engage_stream();

        Ok(())
    }

    /// Pull `subscription_id` out of any live stream/periodic resume list.
    /// USER PAUSE/CANCEL WINS (bench 2026-08-02, log 015453: ~3000 poll
    /// events over 40 s while the UI said paused): the plugin teardown
    /// resumes every sub IT paused at engage — pull this one out of that
    /// resume list so the stop leaves it paused/cancelled.
    /// Returns (stream_involved, periodic_involved). Caller holds the
    /// shared_state lock; the inner acquisition locks are taken and released
    /// here.
    fn detach_from_acquisition(shared_state: &SharedState, subscription_id: Uuid) -> (bool, bool) {
        let stream_involved = {
            let mut guard = shared_state.stream_state.lock().unwrap();
            match guard.as_mut() {
                Some(a) if a.paused_subs.contains(&subscription_id) => {
                    a.paused_subs.retain(|x| *x != subscription_id);
                    true
                }
                _ => false,
            }
        };
        let periodic_involved = {
            let mut guard = shared_state.periodic_state.lock().unwrap();
            match guard.as_mut() {
                Some(a) if a.paused_subs.contains(&subscription_id) => {
                    a.paused_subs.retain(|x| *x != subscription_id);
                    true
                }
                // DV3 (concurrent link): nothing was paused — the plan is
                // this subscription's acquisition if a slot serves one of
                // its pids.
                Some(a) if !a.served_pids.is_empty() => {
                    let mgr = shared_state.subscription_manager.lock().unwrap();
                    mgr.list_subscriptions()
                        .iter()
                        .find(|s| s.id == subscription_id)
                        .map_or(false, |s| s.pids.iter().any(|p| a.served_pids.contains(p)))
                }
                _ => false,
            }
        };
        (stream_involved, periodic_involved)
    }

    /// Shared post-drop tail of pause/cancel: stop whichever acquisition the
    /// subscription was driving. MUST be called with NO locks held (the
    /// request_stop paths take their own).
    fn stop_detached_acquisition(&self, stream_involved: bool, periodic_involved: bool) {
        if stream_involved {
            self.request_stop_stream();
        }
        if periodic_involved {
            self.request_stop_stn_periodic(false);
        }
    }

    /// Pause subscription
    pub fn pause_subscription(&self, subscription_id: Uuid) -> Result<(), SessionError> {
        let shared_state = self.shared_state.lock().unwrap();

        // Log pause
        if let Some(ref logger) = shared_state.logger {
            logger.log_subscription("pause", &subscription_id.to_string(), "");
        }

        // S4: pausing a subscription the stream PAUSED tears the stream
        // down (it IS that subscription's acquisition). Unrelated subs
        // (run-once batches, MCP) must not touch it.
        let (stream_involved, periodic_involved) =
            Self::detach_from_acquisition(&shared_state, subscription_id);
        let mut subscription_manager = shared_state.subscription_manager.lock().unwrap();
        let result = subscription_manager
            .pause_subscription(subscription_id)
            .map_err(|e| {
                SessionError::InternalError(format!("Failed to pause subscription: {:?}", e))
            });
        drop(subscription_manager);
        drop(shared_state);
        self.stop_detached_acquisition(stream_involved, periodic_involved);
        result
    }

    /// Cancel subscription
    pub fn cancel_subscription(&self, subscription_id: Uuid) -> Result<(), SessionError> {
        let shared_state = self.shared_state.lock().unwrap();

        // Log cancel
        if let Some(ref logger) = shared_state.logger {
            logger.log_subscription("cancel", &subscription_id.to_string(), "");
        }

        // Cancelled subs must not be resumed by the teardown either (see
        // pause_subscription — user intent wins over the plugin's bookkeeping).
        let (stream_involved, periodic_involved) =
            Self::detach_from_acquisition(&shared_state, subscription_id);
        let mut subscription_manager = shared_state.subscription_manager.lock().unwrap();
        let result = subscription_manager
            .cancel_subscription(subscription_id)
            .map_err(|e| {
                SessionError::InternalError(format!("Failed to cancel subscription: {:?}", e))
            });
        drop(subscription_manager);
        drop(shared_state);
        self.stop_detached_acquisition(stream_involved, periodic_involved);
        result
    }

    /// Set timeout for a subscription
    ///
    /// Override the default timeout for commands in this subscription.
    /// Useful for run-once subscriptions with multi-frame responses (e.g., VIN).
    pub fn set_subscription_timeout(
        &self,
        subscription_id: Uuid,
        timeout_ms: u32,
    ) -> Result<(), SessionError> {
        let shared_state = self.shared_state.lock().unwrap();
        let mut subscription_manager = shared_state.subscription_manager.lock().unwrap();
        subscription_manager
            .set_subscription_timeout(subscription_id, timeout_ms)
            .map_err(|e| SessionError::InternalError(format!("Failed to set timeout: {:?}", e)))
    }

    /// Get command processor statistics
    pub fn get_command_processor_stats(
        &self,
    ) -> Result<crate::command_processor::CommandProcessorStats, SessionError> {
        let shared_state = self.shared_state.lock().unwrap();
        if let Some(ref processor) = shared_state.command_processor {
            Ok(processor.get_stats())
        } else {
            Err(SessionError::InternalError(
                "Command processor not available".to_string(),
            ))
        }
    }

    /// Get response mismatch metrics from the subscription manager.
    /// Returns (mismatch_count, total_count, mismatch_rate).
    pub fn get_response_mismatch_metrics(&self) -> Result<(u64, u64, f64), SessionError> {
        let shared_state = self.shared_state.lock().unwrap();
        let sub_manager = shared_state.subscription_manager.lock().unwrap();
        Ok(sub_manager.response_mismatch_metrics())
    }

    /// Per-adapter chunk cap (adapters.json `maxChunk`) — set by the app at
    /// connect time, BEFORE identify runs the vehicle probe. Clamped to
    /// 1..=CHUNK_MAX_PIDS; ≤1 disables chunking (probe skipped).
    /// SP0: mark the connected adapter STPPMA-capable (STN chip).
    pub fn set_stn_periodic_capable(&self, capable: bool) {
        {
            let state = self.shared_state.lock().unwrap();
            state.host_facts.lock().unwrap().periodic_capable = capable;
        }
        self.emit_tier_availability(); // adapter fact changed
    }

    /// LH5: adapters.json → the session (connector facts resolve here for
    /// every connector, Rust-owned ones included).
    pub fn set_adapter_catalog(&self, json: &str) -> Result<(), String> {
        let catalog = crate::catalog::Catalog::parse(json)?;
        self.shared_state.lock().unwrap().adapter_catalog = Some(catalog);
        Ok(())
    }

    /// LH5 / DV2: which link handler drives the next connect (`"elm"` |
    /// `"dvi"`, from adapters.json `protocol`). Applied by `connect_body`
    /// before init — a live session is never swapped underneath.
    pub fn set_link_protocol(&self, protocol: &str) -> bool {
        let kind = match protocol.trim().to_ascii_lowercase().as_str() {
            "elm" | "" => crate::link::LinkKind::Elm,
            "dvi" => crate::link::LinkKind::Dvi,
            _ => return false,
        };
        let state = self.shared_state.lock().unwrap();
        state.host_facts.lock().unwrap().kind = kind;
        true
    }

    pub fn set_adapter_max_chunk(&self, cap: usize) {
        let cap = cap.clamp(1, crate::acquisition::CHUNK_MAX_PIDS);
        let state = self.shared_state.lock().unwrap();
        state.host_facts.lock().unwrap().max_chunk = cap;
        eprintln!("Session: adapter max chunk = {cap}");
    }

    /// Quality-floor gate (visible addition #2): simulate adding `pid` to
    /// the current selection and check the projected fast-gauge rate against
    /// the active tier's floor — serial ≥ 3 Hz, batched (pipes OR chunks)
    /// ≥ 6 Hz. LIVE: a pre-probe session gates at the serial floor and
    /// loosens when a probe upgrades the tier. Gating applies to ADDS only —
    /// existing selections are never trimmed.
    /// Returns (allowed, projected_fast_hz, floor_hz).
    ///
    /// RS7: the FFI export (`obd_simulate_dashboard_add`) was dead — zero
    /// Swift callers — and was removed; the floor-gating tests in
    /// `session_manager/tests.rs` still exercise this, so it is test-only.
    #[cfg(test)]
    pub fn simulate_dashboard_add(&self, pid: &str) -> (bool, f64, f64) {
        let shared_state = self.shared_state.lock().unwrap();
        let admission = shared_state.acquisition.lock().unwrap().admission();
        let (_, hz) = {
            let sub_manager = shared_state.subscription_manager.lock().unwrap();
            sub_manager.acquisition_projection_with_candidate(&admission, Some(pid), None)
        };
        let batched = admission.max_segments > 1 || admission.chunk_limit > 1;
        let floor = if batched { 6.0 } else { 3.0 };
        (hz >= floor, hz, floor)
    }

    /// BP2/B2/#17: acquisition line + projected rate for Session Stats.
    /// Returns (plugin_id, pipe_limit, chunk_limit, projected_load,
    /// projected_fast_hz). The projection models BOTH axes: chunkable pids
    /// (per the live admission — capability, learned lengths, exclusions)
    /// share flat-cost segments, solos ride their own.
    pub fn get_acquisition_stats(&self) -> (String, usize, usize, f64, f64) {
        let shared_state = self.shared_state.lock().unwrap();
        // While a UDS periodic stream is live, the acquisition IS the stream:
        // the projection is the flat push rate, not the poll model.
        if shared_state.stream_state.lock().unwrap().is_some() {
            let load = {
                let sub_manager = shared_state.subscription_manager.lock().unwrap();
                sub_manager.active_continuous_pids().len() as f64
            };
            return ("uds-stream".to_string(), 0, 0, load, 25.0);
        }
        // SP: while adapter-periodic is live the ADAPTER runs the loop — the
        // poll round-trip model is meaningless. Projected fast rate is
        // period-driven (1000/fastest period); measured ≪ projected reads as
        // BLE downlink saturation, not round-trip latency.
        if let Some(active) = shared_state.periodic_state.lock().unwrap().as_ref() {
            let stats = active.engine.stats();
            let load = stats.map(|s| s.members).unwrap_or(0) as f64;
            let fast_hz = 1000.0 / f64::from(stats.map(|s| s.fast_period_ms).unwrap_or(25).max(1));
            let kind = shared_state.host_facts.lock().unwrap().kind;
            return (
                super::tier_facts::periodic_tier_for(kind)
                    .as_str()
                    .to_string(),
                0,
                0,
                load,
                fast_hz,
            );
        }
        let (id, admission) = {
            let plugin = shared_state.acquisition.lock().unwrap();
            (plugin.id().to_string(), plugin.admission())
        };
        // Real per-exchange latency so the projection tracks THIS adapter.
        let measured_rtt_ms = shared_state
            .command_processor
            .as_ref()
            .map(|p| p.get_stats())
            .and_then(|s| s.average_completion_time())
            .map(|d| d.as_secs_f64() * 1000.0);
        let (load, fast_hz) = {
            let sub_manager = shared_state.subscription_manager.lock().unwrap();
            sub_manager.acquisition_projection_measured(&admission, measured_rtt_ms)
        };
        (
            id,
            admission.max_segments,
            admission.chunk_limit,
            load,
            fast_hz,
        )
    }

    /// Stream facts for Session Stats — `(slots_used, slot_count,
    /// response_ids)`; None when no stream is live. Kept separate from
    /// get_acquisition_stats so its tuple shape stays stable.
    pub fn get_stream_stats(&self) -> Option<(usize, usize, String)> {
        let stream_state = {
            let shared_state = self.shared_state.lock().unwrap();
            Arc::clone(&shared_state.stream_state)
        };
        let guard = stream_state.lock().unwrap();
        let active = guard.as_ref()?;
        let slots_used = active.slot_fill.lock().unwrap().len();
        Some((
            slots_used,
            active.slot_count,
            active.response_ids.join(", "),
        ))
    }

    /// Install a session monitor callback. Called for every command/response pair.
    pub fn install_session_monitor_callback(
        &self,
        callback: Box<dyn Fn(&crate::session_monitor::SessionMonitorEntry) + Send>,
    ) {
        let shared_state = self.shared_state.lock().unwrap();
        let mut monitor = shared_state.session_monitor.lock().unwrap();
        monitor.set_callback(callback);
    }

    /// Remove the session monitor callback.
    pub fn remove_session_monitor_callback(&self) {
        let shared_state = self.shared_state.lock().unwrap();
        let mut monitor = shared_state.session_monitor.lock().unwrap();
        monitor.remove_callback();
    }

    /// Get the last `count` session monitor entries as a JSON string.
    pub fn get_session_history(&self, count: usize) -> String {
        let shared_state = self.shared_state.lock().unwrap();
        let monitor = shared_state.session_monitor.lock().unwrap();
        let entries = monitor.get_last(count);
        serde_json::to_string(&entries).unwrap_or_else(|_| "[]".to_string())
    }

    /// Clear all session monitor history.
    pub fn clear_session_history(&self) {
        let shared_state = self.shared_state.lock().unwrap();
        let mut monitor = shared_state.session_monitor.lock().unwrap();
        monitor.clear();
    }

    /// Install a callback that fires on every subscription change (add/remove PIDs).
    pub fn install_subscription_change_callback(
        &self,
        callback: Box<dyn Fn(&crate::subscription::SubscriptionAuditEntry) + Send>,
    ) {
        let shared_state = self.shared_state.lock().unwrap();
        let mut sub_manager = shared_state.subscription_manager.lock().unwrap();
        sub_manager.set_subscription_change_callback(callback);
    }

    /// Remove the subscription change callback.
    pub fn remove_subscription_change_callback(&self) {
        let shared_state = self.shared_state.lock().unwrap();
        let mut sub_manager = shared_state.subscription_manager.lock().unwrap();
        sub_manager.remove_subscription_change_callback();
    }

    /// Get a snapshot of all subscriptions with their PIDs, tiers, and state.
    /// Returns a JSON string.
    pub fn get_subscriptions_snapshot(&self) -> String {
        let shared_state = self.shared_state.lock().unwrap();
        let sub_manager = shared_state.subscription_manager.lock().unwrap();
        let snapshot = sub_manager.get_subscriptions_snapshot();
        serde_json::to_string(&snapshot).unwrap_or_else(|_| "[]".to_string())
    }

    /// Get the audit log of PID add/remove operations.
    /// Returns a JSON string of the last `count` entries (newest first).
    pub fn get_subscription_audit_log(&self, count: usize) -> String {
        let shared_state = self.shared_state.lock().unwrap();
        let sub_manager = shared_state.subscription_manager.lock().unwrap();
        let entries = sub_manager.get_audit_log(count);
        serde_json::to_string(&entries).unwrap_or_else(|_| "[]".to_string())
    }

    /// Reset command processor statistics
    pub fn reset_command_processor_stats(&self) -> Result<(), SessionError> {
        let shared_state = self.shared_state.lock().unwrap();
        if let Some(ref processor) = shared_state.command_processor {
            processor.reset_stats();
            Ok(())
        } else {
            Err(SessionError::InternalError(
                "Command processor not available".to_string(),
            ))
        }
    }

    /// Get subscription info
    pub fn get_subscription_info(&self, subscription_id: Uuid) -> Option<SubscriptionInfo> {
        let shared_state = self.shared_state.lock().unwrap();
        let subscription_manager = shared_state.subscription_manager.lock().unwrap();
        subscription_manager.get_subscription_info(subscription_id)
    }

    /// List all subscriptions
    pub fn list_subscriptions(&self) -> Vec<SubscriptionInfo> {
        let shared_state = self.shared_state.lock().unwrap();
        let subscription_manager = shared_state.subscription_manager.lock().unwrap();
        subscription_manager.list_subscriptions()
    }

    /// Set refresh rate tiers for PIDs in a subscription.
    /// Tiers is a map of PID string → RefreshTier.
    /// PIDs not in the map keep their default tier.
    pub fn set_subscription_tiers(
        &self,
        subscription_id: Uuid,
        tiers: std::collections::HashMap<String, crate::subscription::RefreshTier>,
    ) -> Result<(), SessionError> {
        let shared_state = self.shared_state.lock().unwrap();
        let mut subscription_manager = shared_state.subscription_manager.lock().unwrap();
        subscription_manager.set_pid_tiers(subscription_id, tiers)?;
        Ok(())
    }

    /// Update rate limiting
    pub fn set_command_rate(&self, commands_per_second: f64) {
        let state = self.shared_state.lock().unwrap();
        if let Some(ref processor) = state.command_processor {
            processor.update_rate_limit(
                commands_per_second,
                state.config.command_processor.max_burst_tokens,
            );
        }
    }

    /// Get current configuration
    pub fn get_config(&self) -> OBDSessionConfig {
        self.shared_state.lock().unwrap().config.clone()
    }

    /// Reset all subscriptions
    pub fn reset_all_subscriptions(&self) -> Result<(), SessionError> {
        let shared_state = self.shared_state.lock().unwrap();
        // Reset controller state so next session starts with broadcast
        *shared_state.current_controller.lock().unwrap() = None;
        let mut subscription_manager = shared_state.subscription_manager.lock().unwrap();

        // Get all subscription IDs
        let subscription_ids: Vec<Uuid> = subscription_manager
            .list_subscriptions()
            .into_iter()
            .map(|info| info.id)
            .collect();

        // Cancel all subscriptions
        for id in subscription_ids {
            subscription_manager.cancel_subscription(id)?;
        }

        Ok(())
    }

    // ========================================================================
    // Connection Management
    // ========================================================================

    /// Send a connection progress status message via the response callback.
    pub(super) fn send_status(
        &self,
        message_key: &str,
        detail: Option<&str>,
        connectors: Option<&[crate::platform::ConnectorInfo]>,
    ) {
        use crate::connection_msg::ConnectionMsg;

        let phase = ConnectionMsg::phase_for(message_key);

        let payload = serde_json::json!({
            "phase": phase,
            "message": message_key,
            "detail": detail,
            "connectors": connectors,
        });

        let message = serde_json::json!({
            "type": "connection_progress",
            "id": Uuid::new_v4().to_string(),
            "payload": payload,
        });

        if let Ok(json) = serde_json::to_string(&message) {
            let state = self.shared_state.lock().unwrap();
            if let Some(ref log) = state.logger {
                let data = match detail {
                    Some(d) => format!("{} {}", message_key, d),
                    None => message_key.to_string(),
                };
                log.log_callback("connection_progress", &data);
            }
            if let Some(ref callback) = state.response_callback {
                callback(json);
            }
        }
    }

    /// Emit an incremental discovery fact during identify (C7 live scan panel). Each call carries
    /// a partial `discovery` object (`{protocol?, vin?, ecu_count?, pid_count?}`) on a
    /// connection_progress message; the UI accumulates them so the scan panel fills in as each
    /// identify sub-step lands, rather than waiting for the final vehicle_info payload.
    pub(super) fn send_discovery(&self, discovery: serde_json::Value) {
        let payload = serde_json::json!({
            "phase": "identifying_vehicle",
            "message": "discovery",
            "discovery": discovery,
        });

        let message = serde_json::json!({
            "type": "connection_progress",
            "id": Uuid::new_v4().to_string(),
            "payload": payload,
        });

        if let Ok(json) = serde_json::to_string(&message) {
            let state = self.shared_state.lock().unwrap();
            log_cb!(
                state.logger,
                "connection_progress",
                &format!("discovery {}", discovery)
            );
            if let Some(ref callback) = state.response_callback {
                callback(json);
            }
        }
    }

    /// Send vehicle info to Swift as a connection_progress event with type "vehicle_info".
    pub(super) fn send_vehicle_info(&self, info: &crate::vehicle_info::VehicleInfo) {
        let payload = serde_json::json!({
            "phase": "identifying_vehicle",
            "message": "vehicle_info",
            "vehicle_info": info,
        });

        let message = serde_json::json!({
            "type": "connection_progress",
            "id": Uuid::new_v4().to_string(),
            "payload": payload,
        });

        if let Ok(json) = serde_json::to_string(&message) {
            let state = self.shared_state.lock().unwrap();
            // Marker + VIN only — the full payload (supported_pids, ecus) is large and its
            // inputs are already in the log as raw responses.
            log_cb!(
                state.logger,
                "connection_progress",
                &format!("vehicle_info vin={}", info.vin)
            );
            if let Some(ref callback) = state.response_callback {
                callback(json);
            }
        }
    }
}
