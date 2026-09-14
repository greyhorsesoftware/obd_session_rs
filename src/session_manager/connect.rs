use super::*;

/// SR1: which strategy read this connect gets (Strategy_Read_Plan).
enum StrategyGate {
    /// No read: pref off, no dataset, or a non-covered make (fail-closed).
    None,
    /// Ford identify-override dataset — the `$23` RAM probe.
    Ford,
    /// gm_global_a — the `22 F189` DID read.
    GmGlobalA,
}

/// SR1: clean one 12-byte probe window into a cal name
/// (Ford_Strategy_CAN.md §5): printable ASCII up to the first NUL, junk
/// markers stripped (`HEX*`/`HEX`/`H32`/`VBF`), a `DEVICE ONLY` string
/// truncated to its first 7 chars, then a plausibility check — a cal name is
/// a short bare token (alphanumeric/underscore, 3–20 chars).
fn clean_strategy(buf: &[u8]) -> Option<String> {
    let text: String = buf
        .iter()
        .take_while(|&&b| b != 0x00)
        .filter(|&&b| b > 0x1F && b < 0x7F)
        .map(|&b| b as char)
        .collect();
    let mut s = text.trim().to_string();
    for marker in ["HEX*", "HEX", "H32", "VBF"] {
        s = s.replace(marker, "");
    }
    if s.contains("DEVICE ONLY") && s.len() > 7 {
        s.truncate(7);
    }
    let s: String = s
        .trim_matches(|c: char| c == '.' || c == '*' || c.is_whitespace())
        .to_string();
    let ok =
        (3..=20).contains(&s.len()) && s.chars().all(|c| c.is_ascii_alphanumeric() || c == '_');
    if ok {
        Some(s)
    } else {
        None
    }
}

/// RB1: outcome of the initial CAN-addressing probe (0100 / ATDPN).
enum AddressingDetect {
    /// CAN addressing detected — proceed with identify.
    Detected,
    /// Bus didn't answer (UNABLE TO CONNECT / NO DATA / all-SEARCHING /
    /// empty) — retryable, reported as `vehicle_not_responding`.
    NoResponse,
    /// A real but non-CAN response — reported as `unsupported_protocol`.
    NonCan,
}

impl SessionAPIHandle {
    /// Initiate connection to an OBD controller.
    /// Rust drives: discover → select → connect → AT init → vehicle info → connected.
    /// Start a new connect attempt: any older in-flight attempt becomes stale.
    pub(super) fn begin_connect_attempt(&self) -> u64 {
        let (epoch, claimed) = self.fence();
        // New life: the terminal claim resets so this session can be terminated once.
        claimed.store(false, std::sync::atomic::Ordering::SeqCst);
        let generation = epoch.fetch_add(1, std::sync::atomic::Ordering::SeqCst) + 1;
        self.set_phase(super::worker::Phase::Connecting { epoch: generation });
        generation
    }

    /// True while `generation` is still the active session epoch.
    pub(super) fn is_connect_current(&self, generation: u64) -> bool {
        self.fence().0.load(std::sync::atomic::Ordering::SeqCst) == generation
    }

    /// SL1 claim-then-bump: the FIRST terminal source of a session claims (CAS) and
    /// bumps the epoch; every later source no-ops and just reads the current epoch.
    /// Returns `(epoch, owner)` — SL2: the claim WINNER is the sole terminal emitter
    /// (`disconnecting → session_summary → disconnected`); losers do their teardown
    /// work silently.
    pub(super) fn claim_terminal(&self) -> (u64, bool) {
        let (epoch, claimed) = self.fence();
        if claimed
            .compare_exchange(
                false,
                true,
                std::sync::atomic::Ordering::SeqCst,
                std::sync::atomic::Ordering::SeqCst,
            )
            .is_ok()
        {
            (
                epoch.fetch_add(1, std::sync::atomic::Ordering::SeqCst) + 1,
                true,
            )
        } else {
            (epoch.load(std::sync::atomic::Ordering::SeqCst), false)
        }
    }

    /// True once terminal intent has been claimed for the CURRENT session — the
    /// engage ladder's entry refusal (SL1). Cleared by `begin_connect_attempt`.
    pub(super) fn terminal_intent_stands(&self) -> bool {
        self.fence().1.load(std::sync::atomic::Ordering::SeqCst)
    }

    fn fence(
        &self,
    ) -> (
        Arc<std::sync::atomic::AtomicU64>,
        Arc<std::sync::atomic::AtomicBool>,
    ) {
        let state = self.shared_state.lock().unwrap();
        (
            Arc::clone(&state.session_epoch),
            Arc::clone(&state.terminal_claimed),
        )
    }

    /// A failed attempt CLOSES WHAT IT OPENED (Mustang bench 2026-09-02: a
    /// vehicle_not_responding failure left the BLE link up — a connected
    /// adapter never advertises, so it vanished from every later scan round
    /// until app quit; on single-client WiFi the leaked socket would block
    /// the next connect outright). Claim the terminal FIRST so the transport
    /// drop's echo (host connection callback → cancel_connect_attempt) loses
    /// the SL1 claim and stays silent instead of stomping the failure UI
    /// with a DISCONNECTED.
    pub(super) fn teardown_failed_attempt(&self) {
        let _ = self.claim_terminal();
        let platform = Arc::clone(&self.shared_state.lock().unwrap().platform);
        platform.disconnect_from();
    }

    /// User cancelled / disconnected: invalidate any in-flight connect attempt
    /// and emit the terminal DISCONNECTED phase so the UI resets even if the
    /// attempt was mid-identify. The aborted flow itself stays silent (it may
    /// have been superseded by a newer attempt, whose UI must not be stomped).
    pub fn cancel_connect_attempt(&self) {
        use crate::connection_msg::ConnectionMsg;
        let (_, owner) = self.claim_terminal(); // SL1: cancel is terminal intent
        let processor = {
            let mut state = self.shared_state.lock().unwrap();
            state.awaiting_selection = false;
            state.command_processor.as_ref().map(Arc::clone)
        };
        // Abandon any command left outstanding at disconnect (e.g. a
        // subscription poll). Otherwise the processing loop blocks on it and
        // the NEXT connect's init commands time out.
        if let Some(processor) = processor {
            processor.abort_outstanding_and_clear();
        }
        // SL2: only the claim winner emits terminal status.
        if owner {
            self.send_status(ConnectionMsg::DISCONNECTED, None, None);
            self.set_phase(super::worker::Phase::Idle);
        }
    }

    /// How long a disconnect waits for the in-flight command's response
    /// before abandoning it. Normal polls complete in tens of ms; only a
    /// dead adapter runs out the clock.
    const DRAIN_TIMEOUT_MS: u64 = 2000;

    /// User disconnect with a graceful drain (real hardware courtesy).
    ///
    /// Yanking the link mid-command leaves the ELM/STN with a response it
    /// writes into a dead link — on cheap clones those bytes can arrive as
    /// garbage at the START of the next connection. Instead: stop feeding
    /// the processor, give the in-flight command a bounded window to finish
    /// so the adapter ends at its '>' prompt, THEN close the transport.
    ///
    /// Returns immediately: the fence (generation bump) and the DISCONNECTING
    /// status are synchronous; the drain + teardown + terminal DISCONNECTED
    /// run on a background thread. Every destructive step there is guarded by
    /// the captured generation, so if a newer connect attempt has started it
    /// backs off silently and the new attempt owns the processor/transport.
    pub fn disconnect_and_drain(&self, platform: Arc<dyn crate::platform::OBDPlatformInterface>) {
        use crate::connection_msg::ConnectionMsg;
        let (generation, owner) = self.claim_terminal(); // SL1: user disconnect is terminal intent
        let (processor, sub_mgr) = {
            let mut state = self.shared_state.lock().unwrap();
            state.awaiting_selection = false;
            (
                state.command_processor.as_ref().map(Arc::clone),
                Arc::clone(&state.subscription_manager),
            )
        };
        // Polling is completion-driven (each response enqueues the next PID),
        // so pause subscriptions FIRST or the queue refills itself and idle is
        // never reached. User disconnect deletes the dashboard subscription
        // Swift-side anyway; this also covers MCP watches and stragglers.
        // Also drop in-flight markers: the drain may abandon a command whose
        // response never comes, and a stale marker starves that PID (the
        // scheduler skips in-flight PIDs) in the next session.
        {
            let mut mgr = sub_mgr.lock().unwrap();
            mgr.pause_all_active();
            mgr.clear_in_flight();
        }
        if let Some(ref processor) = processor {
            processor.clear_queue(); // stop sending; only the in-flight remains
        }
        // SL2: only the claim winner emits the terminal sequence; a loser (a fatal
        // drop claimed first) still drains silently.
        if owner {
            self.send_status(ConnectionMsg::DISCONNECTING, None, None);
            self.set_phase(super::worker::Phase::Disconnecting { epoch: generation });
        }

        // SL3: the drain + teardown + terminal DISCONNECTED run on the
        // lifecycle worker (claim + DISCONNECTING already happened above,
        // synchronously — terminal intent bumps at ENQUEUE time).
        self.send_lifecycle(super::worker::LifecycleMsg::Disconnect {
            generation,
            owner,
            platform,
        });
    }

    /// SL3 `Disconnect` handler — runs ONLY on the lifecycle worker. The
    /// bounded drain (≤2 s), the transport close, and the terminal sequence
    /// (summary → DISCONNECTED, owner only). Every destructive step is guarded
    /// by the captured generation: superseded by a newer connect → hands off.
    pub(super) fn disconnect_drain_body(
        &self,
        generation: u64,
        owner: bool,
        platform: Arc<dyn OBDPlatformInterface>,
    ) {
        use crate::connection_msg::ConnectionMsg;
        let processor = {
            let state = self.shared_state.lock().unwrap();
            state.command_processor.as_ref().map(Arc::clone)
        };
        if let Some(ref processor) = processor {
            // Bounded drain — the response is the only thing that clears
            // the outstanding slot, and a dead adapter never sends one.
            processor.wait_for_idle(std::time::Duration::from_millis(Self::DRAIN_TIMEOUT_MS));
            if !self.is_connect_current(generation) {
                return; // superseded by a newer connect — hands off
            }
            // Backstop (dead adapter) / no-op when the drain succeeded.
            processor.abort_outstanding_and_clear();
        }
        if !self.is_connect_current(generation) {
            return;
        }
        platform.disconnect_from();
        // SL2: summary BEFORE the terminal DISCONNECTED — structural, not a race.
        // Only the claim winner emits either.
        if owner {
            self.health_finish_and_emit();
            self.send_status(ConnectionMsg::DISCONNECTED, None, None);
            self.set_phase(super::worker::Phase::Idle);
        }
    }

    pub fn connect_to_controller(&self) {
        use crate::connection_msg::ConnectionMsg;

        let generation = self.begin_connect_attempt();
        // SH1: a new connect closes any dangling health session (covers a
        // reconnect that superseded the previous disconnect's drain thread).
        self.health_finish_and_emit();
        self.send_status(ConnectionMsg::DISCOVERING_CONNECTORS, None, None);
        // Discovery Ownership: the session's engine owns the scan loop,
        // prune policy, and merge from here until selection/cancel.
        self.spawn_discovery_loop(generation, super::discovery::DiscoveryCfg::default());
    }

    /// Store + emit the connector list while in waiting_for_selection.
    /// Called by the discovery engine only — the list is already the merged
    /// host ∪ Rust-transport view. `message` is WAITING_FOR_SELECTION on the
    /// first emit (SO2: exactly once per attempt) or CONNECTOR_LIST for the
    /// later on-change refreshes.
    pub(super) fn store_and_emit_connectors(
        &self,
        connectors: &[crate::platform::ConnectorInfo],
        message: &str,
    ) {
        {
            let mut state = self.shared_state.lock().unwrap();
            // A selection can land mid-round; its list is then frozen
            // (select_connector recovers the ConnectorInfo from it).
            if !state.awaiting_selection {
                return;
            }
            state.discovered_connectors = connectors.to_vec();
        }
        self.send_status(message, None, Some(connectors));
    }

    /// User selected a connector from the waiting_for_selection list.
    pub fn select_connector(
        &self,
        platform: Arc<dyn crate::platform::OBDPlatformInterface>,
        connector_id: &str,
    ) {
        {
            let mut state = self.shared_state.lock().unwrap();
            // SO2: the single owner of "a selection is accepted". A second
            // select for the same picker session (user double-tap, a stray
            // re-select) finds awaiting_selection already false and is
            // ignored — WITHOUT this, the duplicate would `begin_connect_
            // attempt` again, bump the epoch, and supersede/kill the connect
            // already in flight. Replaces the Swift bridge dedupe (FB5).
            if !state.awaiting_selection {
                return;
            }
            state.awaiting_selection = false;
        }
        // A selection is a fresh user intent — it owns the connect state from here.
        let generation = self.begin_connect_attempt();
        // Recover the full ConnectorInfo (name, connector_type) from the last
        // discovered list — the UI selects by bare id.
        let connector = self
            .shared_state
            .lock()
            .unwrap()
            .discovered_connectors
            .iter()
            .find(|c| c.id == connector_id || c.name == connector_id)
            .cloned()
            .unwrap_or_else(|| crate::platform::ConnectorInfo {
                id: connector_id.to_string(),
                name: connector_id.to_string(),
                connector_type: String::new(),
            });
        self.initiate_connection(platform, &connector, generation);
    }

    /// Internal: connect to a specific connector, then run AT init + vehicle
    /// info. SL3: the CONNECTING status goes out synchronously (instant UI),
    /// then the flow is enqueued — the lifecycle worker runs the body, so
    /// adapter init + identify no longer execute on a platform callback
    /// thread. Aborts silently at each step boundary if `generation` is no
    /// longer the active connect attempt (user cancel or a newer attempt).
    fn initiate_connection(
        &self,
        _platform: Arc<dyn crate::platform::OBDPlatformInterface>,
        connector: &crate::platform::ConnectorInfo,
        generation: u64,
    ) {
        use crate::connection_msg::ConnectionMsg;
        // Detail = connector TYPE ("ble" | "classic" | "mock"), not the name —
        // the UI renders "Opening BLE/Bluetooth connection..." from it.
        self.send_status(
            ConnectionMsg::CONNECTING,
            Some(&connector.connector_type),
            None,
        );
        self.send_lifecycle(super::worker::LifecycleMsg::Connect {
            connector: connector.clone(),
            generation,
        });
    }

    /// SL3 `Connect` handler — runs ONLY on the lifecycle worker. Opens the
    /// transport (bounded wait on the platform's async result — 100 ms ticks
    /// with an epoch check each, 60 s ceiling), then runs the RB1/RB2-proven
    /// init + identify body inline. A queued Disconnect waits at most one
    /// probe boundary: identify's own epoch checks bail it out.
    pub(super) fn connect_body(&self, connector: crate::platform::ConnectorInfo, generation: u64) {
        use crate::connection_msg::ConnectionMsg;
        if !self.is_connect_current(generation) {
            return; // superseded before the worker got to it
        }
        let platform = {
            let state = self.shared_state.lock().unwrap();
            Arc::clone(&state.platform)
        };
        let platform_for_remember = Arc::clone(&platform);
        let connector_id_for_remember = connector.id.clone();

        // Transport open: the platform reports asynchronously; park the result
        // in a slot and wait BOUNDED (handler discipline — the host's own
        // connect deadline normally fires the callback long before ours).
        let slot: Arc<(
            Mutex<Option<crate::platform::ConnectResult>>,
            std::sync::Condvar,
        )> = Arc::new((Mutex::new(None), std::sync::Condvar::new()));
        let slot_for_cb = Arc::clone(&slot);
        platform.connect_to(
            &connector.id,
            Box::new(move |result| {
                let (m, cv) = &*slot_for_cb;
                *m.lock().unwrap() = Some(result);
                cv.notify_all();
            }),
        );
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(60);
        let result = loop {
            {
                let (m, cv) = &*slot;
                let guard = m.lock().unwrap();
                let (mut guard, _) = cv
                    .wait_timeout(guard, std::time::Duration::from_millis(100))
                    .unwrap();
                if let Some(r) = guard.take() {
                    break r;
                }
            }
            if !self.is_connect_current(generation) {
                return; // cancelled while the transport was connecting
            }
            if std::time::Instant::now() > deadline {
                // Old behavior parity: a connect result that never arrives
                // produced no status. Log it and hand the worker back.
                let logger = self.shared_state.lock().unwrap().logger.clone();
                log_cb!(logger, "connect_result_timeout", &connector.id);
                return;
            }
        };
        if !self.is_connect_current(generation) {
            return; // cancelled while the transport was connecting
        }
        let api = self;
        {
            match result {
                crate::platform::ConnectResult::Connected => {
                    // WIRE-LOG: `# connect <transport> <adapter>` header —
                    // unconditional (every connect, catalog hit or not), so
                    // multi-connect launches stay attributable and the
                    // WiFi-lines-labeled-"Bluetooth" misrouting class dies.
                    {
                        let logger = api.shared_state.lock().unwrap().logger.clone();
                        if let Some(l) = logger {
                            l.log_connect_header(&connector.connector_type, &connector.name);
                        }
                    }
                    api.apply_catalog_facts(&connector);
                    api.rebuild_link_for_connect();
                    api.send_status(ConnectionMsg::INITIALIZING_PROTOCOL, None, None);

                    if api.send_init_commands() {
                        if !api.is_connect_current(generation) {
                            return; // cancelled during adapter init
                        }
                        api.send_status(ConnectionMsg::IDENTIFYING_VEHICLE, None, None);

                        let vehicle_info = api.gather_vehicle_info();
                        if !api.is_connect_current(generation) {
                            return; // cancelled during identify
                        }
                        // RB1: identify is the single terminal gate. On failure
                        // it already emitted CONNECTION_FAILED — do NOT send
                        // CONNECTED (that was the double-terminal bug) and do NOT
                        // remember the connector.
                        match vehicle_info {
                            Ok(info) => {
                                // B2: vehicle chunk probe — after identify
                                // (either path, cache-hit included), before
                                // CONNECTED so the dashboard's first sweep
                                // already runs at the negotiated rate.
                                api.probe_chunk_capability();
                                if !api.is_connect_current(generation) {
                                    return;
                                }
                                api.send_vehicle_info(&info);
                                api.remember_for_reconnect(
                                    Arc::clone(&platform_for_remember),
                                    &connector_id_for_remember,
                                );
                                // S4: make connection_status() truthful —
                                // the engage guard reads it.
                                platform_for_remember.note_connection_status(
                                    crate::platform::ConnectionStatus::Connected,
                                );
                                api.send_status(ConnectionMsg::CONNECTED, None, None);
                                api.set_phase(super::worker::Phase::Connected {
                                    epoch: generation,
                                });
                                // SH1: the session starts acquiring on poll —
                                // window 1 (and the RTT baseline) restart HERE
                                // so identify's slow commands stay out of it.
                                api.health_on_connected();
                            }
                            Err(()) => {
                                // terminal CONNECTION_FAILED already sent by identify
                                api.teardown_failed_attempt();
                            }
                        }
                    } else {
                        api.send_status(
                            ConnectionMsg::CONNECTION_FAILED,
                            Some(ConnectionMsg::REASON_ADAPTER_INIT_TIMEOUT),
                            None,
                        );
                        api.teardown_failed_attempt();
                    }
                }
                crate::platform::ConnectResult::Failed { reason } => {
                    api.send_status(ConnectionMsg::CONNECTION_FAILED, Some(&reason), None);
                    // The host may have opened the link and failed later
                    // (BLE service probe, say) — close whatever is up.
                    api.teardown_failed_attempt();
                }
                crate::platform::ConnectResult::Cancelled => {
                    // SL2: if a terminal claim stands, cancel_connect_attempt (the
                    // winner) already emitted DISCONNECTED — a second one here was
                    // the dup. A supersede-cancel (new attempt, claim reset) keeps
                    // today's UI-reset emission.
                    if !api.terminal_intent_stands() {
                        api.send_status(ConnectionMsg::DISCONNECTED, None, None);
                    }
                    // S4: a drop mid-stream leaves no adapter to handshake
                    // with — clear the stream state/tag locally so the NEXT
                    // connect starts clean (defines on the car self-clear via
                    // S3 once keepalives stop).
                    {
                        let state = api.shared_state.lock().unwrap();
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
                    api.emit_stream_state("stopped", serde_json::json!({}));
                }
            }
        }
    }

    /// Remember the connector + platform after a successful connect. Despite
    /// the name there is no auto-reconnect: this feeds the disconnect
    /// callback's stale-drop check (it needs the platform handle to detect a
    /// superseding connect) and is cleared by `clear_and_cancel` on drops.
    fn remember_for_reconnect(
        &self,
        platform: Arc<dyn crate::platform::OBDPlatformInterface>,
        connector_id: &str,
    ) {
        let reconnect = { self.shared_state.lock().unwrap().reconnect.clone() };
        reconnect.remember(connector_id, platform);
    }

    /// One identify exchange (LH2): status + per-controller parse from the
    /// link handler. Connect-thread only.
    fn request(&self, command: &str, timeout_ms: u32) -> crate::link::LinkReply {
        self.link().request(command, timeout_ms)
    }

    /// LH1: the session's protocol handler.
    pub(super) fn link(&self) -> Arc<dyn crate::link::LinkHandler> {
        Arc::clone(&self.shared_state.lock().unwrap().link)
    }

    /// LH5: resolve the connector against the adapter catalog and fold what
    /// it says into the host facts. A connector no entry matches (mocks,
    /// unknown adapters) leaves the facts exactly as the host set them.
    pub(super) fn apply_catalog_facts(&self, connector: &crate::platform::ConnectorInfo) {
        let state = self.shared_state.lock().unwrap();
        let Some(catalog) = state.adapter_catalog.as_ref() else {
            return;
        };
        let Some(facts) = catalog.resolve(&connector.name, &connector.connector_type) else {
            return;
        };
        let mut host = state.host_facts.lock().unwrap();
        if let Some(p) = facts.supports_periodic {
            host.periodic_capable = p;
        }
        if let Some(c) = facts.max_chunk {
            host.max_chunk = c.clamp(1, crate::acquisition::CHUNK_MAX_PIDS);
        }
        host.kind = match facts.protocol.as_deref().map(|p| p.to_ascii_lowercase()) {
            Some(ref p) if p == "dvi" => crate::link::LinkKind::Dvi,
            _ => crate::link::LinkKind::Elm,
        };
        log_cb!(
            state.logger,
            "adapter_catalog",
            &format!(
                "{} → periodic={} chunk={} kind={:?}",
                facts.adapter_name, host.periodic_capable, host.max_chunk, host.kind
            )
        );
        // The host learns the dialect here (Console prompt/guards, sniff
        // panel transport gate) — emitted outside the locks.
        let kind = if host.kind == crate::link::LinkKind::Dvi {
            "dvi"
        } else {
            "elm"
        };
        let callback = Arc::clone(&state.response_callback_shared);
        drop(host);
        drop(state);
        callback(serde_json::json!({ "type": "link_handler", "kind": kind }).to_string());
    }

    /// LH5: the handler is built at session creation; the catalog's dialect
    /// (`HostFacts.kind`) is only known at connect. Rebuild when it differs —
    /// on the lifecycle worker, before init, with nothing live.
    pub(super) fn rebuild_link_for_connect(&self) {
        let mut state = self.shared_state.lock().unwrap();
        let want = state.host_facts.lock().unwrap().kind;
        if state.link.facts().kind == want {
            return;
        }
        let Some(processor) = state.command_processor.as_ref().map(Arc::clone) else {
            return;
        };
        let link = super::callbacks::build_link(
            want,
            &state.platform,
            &processor,
            &state.reply_waiters,
            &state.logger,
            &state.config,
            &state.host_facts,
        );
        log_cb!(state.logger, "link_handler", &format!("{want:?}"));
        state.link = link;
    }

    /// The session's negotiated addressing as a `Bus` for parsing/targeting.
    pub(super) fn bus(&self) -> crate::addressing::Bus {
        let a = *self.shared_state.lock().unwrap().addressing.lock().unwrap();
        crate::addressing::Bus::new(a)
    }

    /// Detect 11-bit vs 29-bit addressing and store it on the session (decisions A/B/C).
    /// LH1: the sniff (`0100` header, `ATDPN` fallback, SP29 primary-ECU learn)
    /// lives in the link handler; the engine keeps the session copy.
    fn detect_addressing(&self) -> AddressingDetect {
        match self.link().detect_addressing() {
            Ok((a, primary)) => {
                let state = self.shared_state.lock().unwrap();
                *state.addressing.lock().unwrap() = a;
                if let Some(c) = primary {
                    *state.primary_ecu.lock().unwrap() = Some(c);
                }
                AddressingDetect::Detected
            }
            Err(crate::link::AddressingFail::NoResponse) => AddressingDetect::NoResponse,
            Err(crate::link::AddressingFail::NonCan) => AddressingDetect::NonCan,
        }
    }

    /// Gather vehicle identity information after AT init completes.
    ///
    /// Sends OBD commands to collect VIN, engine type, and supported PIDs.
    /// Returns VehicleInfo on success, None on failure.
    /// Run identify. `Ok(info)` on success; `Err(())` means a terminal
    /// `CONNECTION_FAILED` was already emitted and the caller must NOT send
    /// `CONNECTED` (RB1 — one terminal state, and "connected" requires a VIN).
    pub(super) fn gather_vehicle_info(&self) -> Result<crate::vehicle_info::VehicleInfo, ()> {
        use crate::connection_msg::ConnectionMsg;
        use crate::vehicle_info::VehicleInfo;
        use crate::wmi_data;

        // Check cache first
        let (cache_path,) = {
            let state = self.shared_state.lock().unwrap();
            (state.config.cache_path.clone(),)
        };

        // Log discovery start to subscription audit
        {
            let state = self.shared_state.lock().unwrap();
            let mut sub_mgr = state.subscription_manager.lock().unwrap();
            sub_mgr.log_audit_event(
                "discovery_start",
                Some("Vehicle Discovery".to_string()),
                vec![
                    // VIN
                    "0902".into(),
                    // Engine type (monitor status)
                    "0101".into(),
                    // ECU discovery (per controller 7E0-7E7)
                    "090A".into(),
                    "0904".into(),
                    // Mode 01 support bitmaps
                    "0100".into(),
                    "0120".into(),
                    "0140".into(),
                    "0160".into(),
                    "0180".into(),
                    "01A0".into(),
                    "01C0".into(),
                    // Mode 06 support bitmaps
                    "0600".into(),
                    "0620".into(),
                    "0640".into(),
                    "0660".into(),
                    // Mode 09 support bitmap
                    "0900".into(),
                ],
            );
        }

        // 0. Detect addressing (11-bit vs 29-bit) before any parsing. RB1/RB3:
        //    distinguish a non-answering bus (retryable) from a non-CAN vehicle,
        //    and emit a reason CODE (Swift localizes).
        match self.detect_addressing() {
            AddressingDetect::Detected => {}
            AddressingDetect::NoResponse => {
                self.send_status(
                    ConnectionMsg::CONNECTION_FAILED,
                    Some(ConnectionMsg::REASON_VEHICLE_NOT_RESPONDING),
                    None,
                );
                return Err(());
            }
            AddressingDetect::NonCan => {
                self.send_status(
                    ConnectionMsg::CONNECTION_FAILED,
                    Some(ConnectionMsg::REASON_UNSUPPORTED_PROTOCOL),
                    None,
                );
                return Err(());
            }
        }

        // Protocol/addressing is known the moment detection succeeds — emit it first (C7).
        self.send_discovery(serde_json::json!({ "protocol": self.bus().addressing.display() }));

        // 1. Get VIN (Mode 09 PID 02) — multi-frame ISO-TP
        self.send_status(ConnectionMsg::QUERYING_VIN, None, None);
        let vin = self.gather_vin().unwrap_or_default();

        // Log VIN result
        {
            let state = self.shared_state.lock().unwrap();
            let mut sub_mgr = state.subscription_manager.lock().unwrap();
            let vin_entry = if vin.is_empty() {
                "UNKNOWN".to_string()
            } else {
                vin.clone()
            };
            sub_mgr.log_audit_event(
                "discovery_vin",
                Some("Vehicle Discovery".to_string()),
                vec![vin_entry],
            );
        }

        // DR4/ED2: resolve the attached dataset the moment VIN + addressing are both
        // known — BEFORE ECU discovery, so the module walk (ED2) can read it. The GB2
        // match rule, session-side; attachment is a session fact, not a host callback.
        {
            let state = self.shared_state.lock().unwrap();
            let addressing = *state.addressing.lock().unwrap();
            let registry = state.dataset_registry.lock().unwrap();
            let attached = registry
                .as_ref()
                .and_then(|cfg| cfg.resolve(&vin, &addressing))
                .map(|d| d.id.clone());
            if let Some(id) = &attached {
                log_cb!(state.logger, "dataset_attached", id);
            }
            *state.attached_dataset.lock().unwrap() = attached;
        }

        // RB1: VIN is strictly required. A real CAN vehicle answers 0902, so an
        // empty VIN means the bus wasn't actually talking (ignition off / flaky
        // link) — a completed identify with no VIN is a failure, not a hollow
        // "connected". (Distinct from the LB4 timing race, where the VIN DOES
        // arrive; here it never did.)
        if vin.is_empty() {
            self.send_status(
                ConnectionMsg::CONNECTION_FAILED,
                Some(ConnectionMsg::REASON_VEHICLE_NOT_RESPONDING),
                None,
            );
            return Err(());
        }

        // Surface the VIN as soon as it's read (C7) — don't wait for the full payload.
        if !vin.is_empty() {
            self.send_discovery(serde_json::json!({ "vin": vin }));
        }

        // Bind the learned-length cache to this VIN (loads any persisted
        // lengths; clears the previous car's). Must happen BEFORE the
        // vehicle-info cache-hit early return below — a cache-hit connect
        // still needs its lengths.
        if !vin.is_empty() {
            if let Some(ref path) = cache_path {
                let state = self.shared_state.lock().unwrap();
                state.length_cache.lock().unwrap().load_for_vin(path, &vin);
            }
        }

        // 1b. Check cache for this VIN
        if !vin.is_empty() {
            if let Some(ref path) = cache_path {
                self.send_status(ConnectionMsg::CHECKING_CACHE, None, None);
                let cache_file =
                    std::path::PathBuf::from(path).join(format!("{}.vehicle_info", &vin));
                if let Ok(data) = std::fs::read_to_string(&cache_file) {
                    if let Ok(mut cached_info) = serde_json::from_str::<VehicleInfo>(&data) {
                        self.send_status(ConnectionMsg::CACHE_HIT, None, None);
                        // Fill the scan panel from cache too (C7) — counts the live path emits later.
                        self.send_discovery(serde_json::json!({
                            "pid_count": cached_info.supported_pids.len(),
                            "ecu_count": cached_info.ecus.len(),
                        }));
                        // Log cache hit to audit
                        {
                            let state = self.shared_state.lock().unwrap();
                            let mut sub_mgr = state.subscription_manager.lock().unwrap();
                            sub_mgr.log_audit_event(
                                "discovery_cache_hit",
                                Some("Vehicle Discovery (Cache)".to_string()),
                                cached_info.supported_pids.clone(),
                            );
                        }
                        // SP29: primary_ecu is NOT seeded from cache — cached
                        // ECU keys belong to the addressing the cache was
                        // written under (an 11-bit "00" is garbage on a 29-bit
                        // connect). detect_addressing already learned it from
                        // the live 0100 response this connect.
                        //
                        // SR2b: a cache written while the calibration-read
                        // pref was off (or before SR shipped) carries no
                        // strategy — a pref-off nil must NOT persist as
                        // "probed and failed". Backfill once now that the
                        // gates allow it, and persist the result.
                        if self.backfill_strategy(&mut cached_info) {
                            let json =
                                serde_json::to_string_pretty(&cached_info).unwrap_or_default();
                            let _ = std::fs::write(&cache_file, json);
                        }
                        return Ok(cached_info);
                    }
                }
            }
        }

        // 2. Get engine type from monitor status (0101)
        self.send_status(ConnectionMsg::QUERYING_ENGINE_TYPE, None, None);
        let engine_type = self.gather_engine_type();

        // Log engine type
        {
            let state = self.shared_state.lock().unwrap();
            let mut sub_mgr = state.subscription_manager.lock().unwrap();
            sub_mgr.log_audit_event(
                "discovery_engine",
                Some("Vehicle Discovery".to_string()),
                vec![format!("{:?}", engine_type)],
            );
        }

        // 3. Get supported PIDs (functional broadcast) + the passive ECU roster — enumeration is
        //    free from the broadcast response headers, both protocols.
        self.send_status(ConnectionMsg::QUERYING_SUPPORT_PIDS, None, None);
        let (supported_pids, roster) = self.gather_supported_pids();
        self.send_discovery(serde_json::json!({ "pid_count": supported_pids.len() }));
        // SP29: primary ECU = first functional responder (the engine, by
        // convention and by response order on both protocols).
        if let Some(b) = roster.first().and_then(|k| u16::from_str_radix(k, 16).ok()) {
            *self
                .shared_state
                .lock()
                .unwrap()
                .primary_ecu
                .lock()
                .unwrap() = Some(crate::addressing::Controller::new(b));
        }

        // RB1: zero supported PIDs is the same "bus not answering" condition as
        // an empty VIN — a connection with nothing readable is a failure, not a
        // silent blank dashboard.
        if supported_pids.is_empty() {
            self.send_status(
                ConnectionMsg::CONNECTION_FAILED,
                Some(ConnectionMsg::REASON_VEHICLE_NOT_RESPONDING),
                None,
            );
            return Err(());
        }

        // 4. Detail each observed ECU (name + cal-IDs), physically targeted, both protocols.
        self.send_status(ConnectionMsg::DISCOVERING_ECUS, None, None);
        let ecus = self.gather_ecu_details(&roster);
        self.send_discovery(serde_json::json!({ "ecu_count": ecus.len() }));

        // Log ECUs found
        {
            let state = self.shared_state.lock().unwrap();
            let mut sub_mgr = state.subscription_manager.lock().unwrap();
            let ecu_addrs: Vec<String> = ecus
                .iter()
                .map(|e| format!("{} ({})", e.controller_id, e.name))
                .collect();
            sub_mgr.log_audit_event(
                "discovery_ecus",
                Some("Vehicle Discovery".to_string()),
                ecu_addrs,
            );
        }

        // 5. WMI lookup
        let wmi = wmi_data::extract_wmi(&vin).unwrap_or("").to_string();
        let manufacturer = wmi_data::lookup_manufacturer(&wmi).map(|s| s.to_string());

        // Log manufacturer
        {
            let state = self.shared_state.lock().unwrap();
            let mut sub_mgr = state.subscription_manager.lock().unwrap();
            let mfr = manufacturer
                .clone()
                .unwrap_or_else(|| "Unknown".to_string());
            sub_mgr.log_audit_event(
                "discovery_manufacturer",
                Some("Vehicle Discovery".to_string()),
                vec![wmi.clone(), mfr],
            );
        }

        // FS1c: identity enrichment. Year char = VIN position 10; the PCM
        // strategy read (`22 F188` on the primary ECU) runs ONLY when the
        // attached dataset carries an identify override (Ford tables) — one
        // bounded probe; census = which of the generation-telling headers
        // answered the walk (726 joins when MS-CAN lands).
        let year_char = vin.chars().nth(9).map(|c| c.to_string());
        // FS1c: the session-level strategy is the PCM row's F188 read (the
        // walk's identity cluster) — no separate probe.
        let pcm_strategy = ecus
            .iter()
            .find(|e| e.controller_id == "7E0")
            .and_then(|e| e.strategy.clone());
        let module_census: Vec<String> = ecus
            .iter()
            .map(|e| e.controller_id.clone())
            .filter(|id| matches!(id.as_str(), "730" | "716" | "724" | "706" | "726"))
            .collect();

        // SR1/SR2 (Strategy_Read_Plan): the strategy read. Ford (identify-
        // override dataset): `$23` RAM probe on the PCM — runs HERE, after the
        // walk and before any streaming engage (`2C 03` clears dynamic DIDs).
        // gm_global_a: one `22 F189` read on the ECM (the only module whose
        // cal matters). Pref-gated (SR2b); None = fail-closed downstream.
        let strategy_probed = match self.strategy_gate() {
            StrategyGate::Ford => self.probe_strategy_ford(),
            StrategyGate::GmGlobalA => self.read_f189_ecm(),
            StrategyGate::None => None,
        };

        let info = VehicleInfo {
            vin: vin.clone(),
            wmi,
            manufacturer,
            engine_type,
            ecus,
            supported_pids,
            protocol: self.bus().addressing.display(),
            year_char,
            pcm_strategy,
            module_census,
            strategy_probed,
        };

        // 5. Cache results if we have a VIN and cache path
        if !vin.is_empty() {
            if let Some(ref path) = cache_path {
                self.send_status(ConnectionMsg::SAVING_CACHE, None, None);
                if let Ok(_) = crate::cache::CacheManager::new(path) {
                    let json = serde_json::to_string_pretty(&info).unwrap_or_default();
                    let cache_file =
                        std::path::PathBuf::from(path).join(format!("{}.vehicle_info", vin));
                    let _ = std::fs::write(cache_file, json);
                }
            }
        }

        // Log discovery complete to subscription audit
        {
            let state = self.shared_state.lock().unwrap();
            let mut sub_mgr = state.subscription_manager.lock().unwrap();
            sub_mgr.log_audit_event(
                "discovery_complete",
                Some("Vehicle Discovery".to_string()),
                info.supported_pids.clone(),
            );
        }

        Ok(info)
    }

    /// Extract VIN from 0902 response
    fn gather_vin(&self) -> Option<String> {
        let parsed = self.request("0902", 10000).parsed?;

        // Pick the primary responder (lowest ECU — the powertrain module).
        let ctrl = crate::response_parser::select_controller(&parsed, None, &[])?;

        // data_bytes after echo stripping: [count, VIN bytes...]
        // First byte is number of data items (usually 01), rest is ASCII VIN
        if ctrl.data_bytes.len() < 2 {
            return None;
        }

        let vin: String = ctrl.data_bytes[1..]
            .iter()
            .filter(|&&b| b > 0x1F && b < 0x7F) // printable ASCII only
            .map(|&b| b as char)
            .collect();

        if vin.len() >= 11 {
            Some(vin)
        } else {
            None
        }
    }

    /// Detect engine type from 0101 monitor status
    fn gather_engine_type(&self) -> crate::vehicle_info::EngineType {
        use crate::vehicle_info::EngineType;

        let parsed = match self.request("0101", 5000).parsed {
            Some(p) => p,
            None => return EngineType::Unknown,
        };

        let ctrl = match crate::response_parser::select_controller(&parsed, None, &[]) {
            Some(c) => c,
            None => return EngineType::Unknown,
        };

        match crate::monitor_status_parser::parse_monitor_status(&ctrl.data_bytes) {
            Some(status) => {
                if status.engine_type == "diesel" {
                    EngineType::Diesel
                } else {
                    EngineType::Gasoline
                }
            }
            None => EngineType::Unknown,
        }
    }

    /// Enumerate + detail ECUs, protocol-appropriately. For each candidate, physically address it
    /// and query `090A` (name) + `0904` (cal IDs), one ECU at a time (clean single-responder
    /// multi-frame). Restores functional addressing when done.
    ///
    /// Candidate set differs by protocol because the address spaces differ:
    /// - **11-bit:** the standard `7E0-7E7` physical block (small, fixed) ∪ the passive roster.
    ///   The sweep is required — non-emissions ECUs (TCM/ABS/…) answer `090A` but not the
    ///   functional Mode 01/09 support scan, so the roster alone misses them.
    /// - **29-bit:** the passively-observed roster only — its source space (`0x00-0xFF`) is too
    ///   large to blind-sweep; ECUs are enumerated from the functional-broadcast response headers.
    fn gather_ecu_details(&self, roster: &[String]) -> Vec<crate::vehicle_info::ECUInfo> {
        use crate::addressing::{Addressing, Controller};
        use crate::vehicle_info::ECUInfo;
        use std::collections::BTreeSet;

        let bus = self.bus();

        // ED2 (Headers_ECU_Discovery_Plan): a matched dataset's module table OVERRIDES
        // enumeration entirely — walk exactly those addresses. No table → the legacy
        // J1979 discovery below, byte-identical.
        if let Some(ecus) = self.gather_table_ecu_details() {
            let _ = self.request(&format!("ATSH{}", bus.functional_header()), 2000);
            return ecus;
        }

        let mut candidates: BTreeSet<u16> = roster
            .iter()
            .filter_map(|k| u16::from_str_radix(k, 16).ok())
            .collect();
        if matches!(bus.addressing, Addressing::Can11) {
            // GB14 identity: 11-bit controllers are full CAN ids — sweep the standard block.
            candidates.extend(0x7E0..=0x7E7u16);
        }

        let mut ecus = Vec::new();
        for ecu in candidates {
            let key = format!("{:02X}", ecu);
            let in_roster = roster.iter().any(|k| k == &key);

            let atsh = format!("ATSH{}", bus.request_header(Controller::new(ecu)));
            let atsh_ok = self.request(&atsh, 2000).answered();
            if !atsh_ok && !in_roster {
                continue;
            }

            // Query ECU name (090A) — optional per ECU; a blank name is fine.
            let name = self
                .request("090A", 5000)
                .clean_parsed()
                .and_then(|p| Self::parse_090x_ascii(p, &key));

            // A sweep-only candidate (not in the roster) is a real ECU only if it answered 090A.
            if !in_roster && name.is_none() {
                continue;
            }

            // Query calibration IDs (0904).
            let calibration_ids = self
                .request("0904", 5000)
                .clean_parsed()
                .and_then(|p| Self::parse_090x_ascii(p, &key))
                .filter(|cal| !cal.is_empty())
                .map(|cal| vec![cal])
                .unwrap_or_default();

            ecus.push(ECUInfo {
                controller_id: key,
                name: name.unwrap_or_default(),
                calibration_ids,
                hardware: None,
                strategy: None,
                serial: None,
                module_vin: None,
            });
        }

        // Restore functional (broadcast) addressing for the active protocol.
        let _ = self.request(&format!("ATSH{}", bus.functional_header()), 2000);

        ecus
    }

    /// ED2: walk the attached dataset's module table (override enumeration). None when
    /// no dataset/table is attached → caller falls through to legacy discovery.
    ///
    /// The table decides WHO; the address range decides HOW (wire-fact rule, plan ED2):
    /// `7E0`-block → mode-09 `090A`/`0904` (cal IDs survive); 11-bit low range → GMLAN
    /// `$1A B0`; 29-bit routes → UDS `$22 F197`. Wire name text wins; the table name is
    /// the fallback for a module that answers but doesn't parse; a module that answers
    /// NOTHING attributable is absent and skipped silently (self-flagging).
    fn gather_table_ecu_details(&self) -> Option<Vec<crate::vehicle_info::ECUInfo>> {
        use crate::vehicle_info::ECUInfo;
        let (modules, identify) = {
            let state = self.shared_state.lock().unwrap();
            let attached = state.attached_dataset.lock().unwrap().clone()?;
            let registry = state.dataset_registry.lock().unwrap();
            let ds = registry
                .as_ref()?
                .datasets
                .iter()
                .find(|d| d.id == attached)?
                .clone();
            if ds.modules.is_empty() {
                return None;
            }
            (ds.modules, ds.identify)
        };
        // FS1: the dataset identify override, decomposed once — (send wire,
        // expected positive-echo bytes). Ford: 22F111 / 62 F1 11.
        let override_probe: Option<(String, Vec<u8>)> = identify.as_ref().and_then(|i| {
            let bytes: Vec<u8> = (0..i.expect.len())
                .step_by(2)
                .filter_map(|j| u8::from_str_radix(i.expect.get(j..j + 2)?, 16).ok())
                .collect();
            if bytes.is_empty() {
                None
            } else {
                Some((i.send.clone(), bytes))
            }
        });
        let bus = self.bus();
        let mut ecus = Vec::new();
        let mut ms_skipped = 0usize;
        for m in &modules {
            // FS1: MS-CAN rows are data-only until the MS phase — skip, count.
            if m.bus.as_deref() == Some("ms") {
                ms_skipped += 1;
                continue;
            }
            let Some(header) = m.request_header() else {
                continue;
            };
            let Some(ctrl) = bus.from_target(&header) else {
                continue;
            };
            let key = ctrl.key();
            if !self.request(&format!("ATSH{header}"), 2000).answered() {
                continue;
            }
            let is_7e_block = header.len() == 3
                && (0x7E0..=0x7E7).contains(&u16::from_str_radix(&header, 16).unwrap_or(0));
            if is_7e_block {
                // Standard-surface entry: exactly today's per-candidate treatment.
                let name = self
                    .request("090A", 5000)
                    .clean_parsed()
                    .and_then(|p| Self::parse_090x_ascii(p, &key));
                let calibration_ids = self
                    .request("0904", 5000)
                    .clean_parsed()
                    .and_then(|p| Self::parse_090x_ascii(p, &key))
                    .filter(|cal| !cal.is_empty())
                    .map(|cal| vec![cal])
                    .unwrap_or_default();
                if name.is_none() && calibration_ids.is_empty() {
                    continue; // absent
                }
                // FS2 identity cluster (override datasets only): F188/F18C/F190.
                let (strategy, serial, module_vin) = if override_probe.is_some() {
                    self.read_identity_cluster(ctrl)
                } else {
                    (None, None, None)
                };
                // The 7E0 block names itself via 090A, but Ford PCMs also carry
                // the hardware part number on `22 F111` — read it so the PCM row
                // gets a Hardware value like the walked modules do.
                let hardware = override_probe
                    .as_ref()
                    .and_then(|(send, exp)| self.probe_ascii(send, exp, ctrl).flatten());
                ecus.push(ECUInfo {
                    controller_id: key,
                    name: name.unwrap_or_else(|| m.name.clone()),
                    calibration_ids,
                    hardware,
                    strategy,
                    serial,
                    module_vin,
                });
            } else {
                // FS1: the dataset's identify override wins for every
                // non-7E0-block module (Ford `22 F111`); otherwise the
                // built-in range rules (29-bit $22 F197 / GMLAN $1A B0).
                let (cmd, expect): (&str, &[u8]) = if let Some((send, exp)) = &override_probe {
                    (send.as_str(), exp.as_slice())
                } else if header.len() == 8 {
                    ("22F197", &[0x62, 0xF1, 0x97])
                } else {
                    ("1AB0", &[0x5A, 0xB0])
                };
                // Presence is decided by the RAW reply, not name attribution:
                // a NO DATA / timeout means the module is absent, but ANY data
                // reply — a positive echo OR a 7F negative ("present, this DID
                // unsupported") — proves it is on the bus. Ford body modules
                // (IPC/ACM/PSCM/ABS) answer 7F to `22 F111` yet ARE fitted;
                // include them with the table name (Mustang bench 2026-09-02:
                // the walk found 4 of 9 HS modules because it required a
                // positive echo — the other 4 were dropped despite replying).
                let reply = self.request(cmd, 5000);
                if !reply.is_data() {
                    continue; // NO DATA / timeout — truly absent
                }
                // A positive echo yields the hardware/name; a 7F yields None.
                let parsed_name = decode_probe_ascii(&reply, expect, ctrl).flatten();
                // FS2 identity cluster (override datasets only) — the module
                // is PRESENT; three more bounded probes on the same ATSH.
                let (strategy, serial, module_vin) = if override_probe.is_some() {
                    self.read_identity_cluster(ctrl)
                } else {
                    (None, None, None)
                };
                // FS3 field finding: F111 on real Fords is the HARDWARE part
                // number, not a friendly name — the table name IS the display
                // name; F111 rides its own column.
                ecus.push(ECUInfo {
                    controller_id: key,
                    name: m.name.clone(),
                    calibration_ids: Vec::new(),
                    hardware: parsed_name,
                    strategy,
                    serial,
                    module_vin,
                });
            }
        }
        if ms_skipped > 0 {
            let logger = self.shared_state.lock().unwrap().logger.clone();
            log_cb!(
                logger,
                "module_walk",
                &format!("ms_modules_skipped={ms_skipped}")
            );
        }
        Some(ecus)
    }

    /// FS2 identity cluster — three bounded probes on the CURRENT (already
    /// ATSH'd) module: `22 F188` strategy, `22 F18C` serial, `22 F190` the
    /// VIN this module carries (a used-parts module keeps its donor's —
    /// anti-swap tell). NRC/silence per DID → that field stays None.
    fn read_identity_cluster(
        &self,
        ctrl: crate::addressing::Controller,
    ) -> (Option<String>, Option<String>, Option<String>) {
        let strategy = self
            .probe_ascii("22F188", &[0x62, 0xF1, 0x88], ctrl)
            .flatten();
        let serial = self
            .probe_ascii("22F18C", &[0x62, 0xF1, 0x8C], ctrl)
            .flatten();
        let module_vin = self
            .probe_ascii("22F190", &[0x62, 0xF1, 0x90], ctrl)
            .flatten();
        (strategy, serial, module_vin)
    }

    /// ED2: one probe on an already-`ATSH`'d module. Outer `None` = nothing attributable
    /// answered (absent). Inner = the ASCII name when the positive response (`expect`
    /// prefix, `5A B0` / `62 F1 97`) parsed; an attributable NRC or unparsable answer
    /// still proves presence (caller falls back to the table name).
    fn probe_ascii(
        &self,
        cmd: &str,
        expect: &[u8],
        want: crate::addressing::Controller,
    ) -> Option<Option<String>> {
        decode_probe_ascii(&self.request(cmd, 5000), expect, want)
    }

    /// SR1: which strategy read (if any) this connect gets. Pref-gated (SR2b).
    /// Ford = the attached dataset carries an identify override (the Ford
    /// tables); gm_global_a = the GM Global A dataset. Everything else: none —
    /// GMLAN also implements `$23`, so probing a non-Ford at Ford addresses
    /// risks a false-ASCII strategy (fail-closed by construction).
    fn strategy_gate(&self) -> StrategyGate {
        let state = self.shared_state.lock().unwrap();
        if !state.calibration_read_enabled {
            return StrategyGate::None;
        }
        let Some(attached) = state.attached_dataset.lock().unwrap().clone() else {
            return StrategyGate::None;
        };
        if attached == "gm_global_a" {
            return StrategyGate::GmGlobalA;
        }
        let registry = state.dataset_registry.lock().unwrap();
        let has_override = registry
            .as_ref()
            .and_then(|cfg| cfg.datasets.iter().find(|d| d.id == attached))
            .map(|d| d.identify.is_some())
            .unwrap_or(false);
        if has_override {
            StrategyGate::Ford
        } else {
            StrategyGate::None
        }
    }

    /// SR1: the Ford strategy probe. PCM
    /// only: extended session (`10 03`), clear DDDI (`2C 03`), then up to four
    /// fixed RAM windows of 3×4-byte `23 14 <addr BE> 04` reads; the 12 bytes
    /// clean up to the cal name (`GMBM2` style). Explicit
    /// `10 01` teardown + functional-header restore either way. Must run
    /// before any streaming engage — `2C 03` clears dynamic DIDs.
    fn probe_strategy_ford(&self) -> Option<String> {
        const WINDOWS: [[u32; 3]; 4] = [
            [0x101C0, 0x101C4, 0x101C8],
            [0x10043, 0x10047, 0x1004B],
            [0x10052, 0x10056, 0x1005A],
            [0x10240, 0x10244, 0x10248],
        ];
        let bus = self.bus();
        let logger = self.shared_state.lock().unwrap().logger.clone();
        let Some(ctrl) = bus.from_target("7E0") else {
            log_cb!(
                logger,
                "strategy_probe",
                "skipped: no 7E0 route on this addressing"
            );
            return None;
        };
        if !self.request("ATSH7E0", 2000).answered() {
            return None;
        }
        // Pre-LH2 gate: any reply without an error token opened the session
        // (an `OK`-class ack counted; the window reads decide).
        let sess_ok = self.request("1003", 2000).is_clean();
        let mut result = None;
        let mut window_used = 0usize;
        if sess_ok {
            let _ = self.request("2C03", 2000);
            'windows: for (wi, window) in WINDOWS.iter().enumerate() {
                let mut buf: Vec<u8> = Vec::new();
                for addr in window {
                    match self.read_memory_4(*addr, ctrl) {
                        Some(bytes) => buf.extend_from_slice(&bytes),
                        None => continue 'windows, // NRC 31 / silence — next window
                    }
                }
                if let Some(name) = clean_strategy(&buf) {
                    result = Some(name);
                    window_used = wi + 1;
                    break;
                }
            }
            // Return to the default session — don't rely on the 5 s S3 lapse.
            let _ = self.request("1001", 2000);
        }
        let _ = self.request(&format!("ATSH{}", bus.functional_header()), 2000);
        match &result {
            Some(name) => {
                log_cb!(
                    logger,
                    "strategy_probe",
                    &format!("window={window_used} strategy={name}")
                )
            }
            None => log_cb!(
                logger,
                "strategy_probe",
                &format!("no_result session_ok={sess_ok}")
            ),
        }
        result
    }

    /// SR1: one `23 14 <addr BE> 04` read on the already-ATSH'd PCM — the 4
    /// data bytes after a positive `63` (an address echo, if the ECU sends
    /// one, is skipped), or None on NRC/silence.
    fn read_memory_4(&self, addr: u32, want: crate::addressing::Controller) -> Option<[u8; 4]> {
        let cmd = format!("2314{addr:08X}04");
        decode_memory_4(&self.request(&cmd, 1000), addr, want)
    }

    /// SR2: the GM strategy-equivalent — one bounded `22 F189` read on the
    /// ECM (`7E0`); the only module whose cal matters. Restores the
    /// functional header either way.
    fn read_f189_ecm(&self) -> Option<String> {
        let bus = self.bus();
        let ctrl = bus.from_target("7E0")?;
        if !self.request("ATSH7E0", 2000).answered() {
            return None;
        }
        let v = self
            .probe_ascii("22F189", &[0x62, 0xF1, 0x89], ctrl)
            .flatten();
        let _ = self.request(&format!("ATSH{}", bus.functional_header()), 2000);
        v
    }

    /// SR2b cache backfill: a cached VehicleInfo with no strategy gets one
    /// read now (gates permitting) — a connect made while the pref was off
    /// must not permanently look like "probed and failed". Returns true when
    /// the info changed (caller persists).
    fn backfill_strategy(&self, info: &mut crate::vehicle_info::VehicleInfo) -> bool {
        if info.strategy_probed.is_some() {
            return false;
        }
        match self.strategy_gate() {
            StrategyGate::Ford => {
                if let Some(s) = self.probe_strategy_ford() {
                    info.strategy_probed = Some(s);
                    return true;
                }
            }
            StrategyGate::GmGlobalA => {
                if let Some(v) = self.read_f189_ecm() {
                    info.strategy_probed = Some(v);
                    return true;
                }
            }
            StrategyGate::None => {}
        }
        false
    }

    /// Parse a 090x multi-frame ISO-TP response into an ASCII string.
    /// Format mirrors gather_vin(): first data byte is count, rest is ASCII.
    fn parse_090x_ascii(
        parsed: &crate::response_parser::ParsedResponse,
        expected_controller: &str,
    ) -> Option<String> {
        // Find the response from the expected controller
        let ctrl = parsed.all_controllers.get(expected_controller)?;
        if ctrl.data_bytes.len() < 2 {
            return None;
        }

        // Skip first byte (number of data items), extract printable ASCII
        let text: String = ctrl.data_bytes[1..]
            .iter()
            .filter(|&&b| b > 0x1F && b < 0x7F)
            .map(|&b| b as char)
            .collect();

        if text.is_empty() {
            None
        } else {
            Some(text)
        }
    }

    /// Gather supported PIDs across Mode 01, Mode 06, and Mode 09 support ranges.
    /// Queries all ranges unconditionally — vehicles often don't set continuation bits correctly.
    /// Returns (supported PIDs, passively-observed ECU roster). The roster is every distinct
    /// controller that answered a functional broadcast — free enumeration, both protocols (C2/C5).
    fn gather_supported_pids(&self) -> (Vec<String>, Vec<String>) {
        use std::collections::BTreeSet;
        let mut all_supported = Vec::new();
        let mut roster: BTreeSet<String> = BTreeSet::new();

        self.gather_support_all(
            "01",
            &["0100", "0120", "0140", "0160", "0180", "01A0", "01C0"],
            &mut all_supported,
            &mut roster,
        );
        self.gather_support_all(
            "06",
            &["0600", "0620", "0640", "0660"],
            &mut all_supported,
            &mut roster,
        );
        self.gather_support_all("09", &["0900"], &mut all_supported, &mut roster);

        (all_supported, roster.into_iter().collect())
    }

    /// Query all support bitmap PIDs for a given mode without chaining. Unions the supported bits
    /// from ALL responding controllers (multi-ECU on 29-bit) and records each in `roster`.
    /// Skips commands that get no response but continues to the next range.
    fn gather_support_all(
        &self,
        mode: &str,
        commands: &[&str],
        out: &mut Vec<String>,
        roster: &mut std::collections::BTreeSet<String>,
    ) {
        let base_of = |cmd: &str| u16::from_str_radix(&cmd[2..], 16).unwrap_or(0);
        for &cmd in commands {
            let Some(parsed) = self.request(cmd, 5000).parsed else {
                continue; // Skip this range but try the next
            };

            for (ctrl_id, ctrl) in &parsed.all_controllers {
                if !ctrl.is_valid {
                    continue;
                }
                if ctrl_id != "DEFAULT" {
                    roster.insert(ctrl_id.clone());
                }
                let result = crate::pid_support_parser::parse_supported_pids(
                    &ctrl.data_bytes,
                    mode,
                    base_of(cmd),
                );
                out.extend(result.supported_pids);
            }
        }
    }

    /// Send AT initialization commands and wait for them all to complete.
    ///
    /// Uses the init_commands list from OBDSessionConfig (provided by the UI layer).
    /// Returns true if all commands were queued and completed within timeout.
    /// B2: post-identify vehicle probe for J1979 multi-PID support. Sends
    /// `010C0D` (RPM+speed — universally supported solo, standard lengths)
    /// and enables the chunk axis iff BOTH pids come back. Runs every
    /// connect on both identify paths (cache-hit included) — no per-VIN
    /// persistence: one ~90 ms exchange isn't worth a cache that can only
    /// go stale (same reasoning as the adapter-side STBC probe). Capability
    /// was already reset to OFF at init.
    pub(super) fn probe_chunk_capability(&self) {
        let (link, acquisition, logger, enabled, adapter_cap) = {
            let state = self.shared_state.lock().unwrap();
            let max_chunk = state.host_facts.lock().unwrap().max_chunk;
            (
                Arc::clone(&state.link),
                Arc::clone(&state.acquisition),
                state.logger.clone(),
                state.config.enable_j1979_chunking,
                max_chunk,
            )
        };
        if !enabled {
            return;
        }
        // Adapter registry says this adapter can't carry chunks (maxChunk ≤ 1)
        // — don't even spend the probe wire.
        if adapter_cap <= 1 {
            eprintln!("Identify: J1979 chunking off (adapter maxChunk {adapter_cap})");
            log_cb!(logger, "chunk_probe", "adapter_capped");
            return;
        }
        // 5 s command timeout + the handler's 2 s idle grace ≈ the old 10 s wait.
        let capable =
            crate::response_parser::chunk_probe_capable(&link.request("010C0D", 5000).payloads);
        {
            let mut plugin = acquisition.lock().unwrap();
            plugin.set_chunk_capable(capable);
            if capable {
                plugin.set_chunk_cap(adapter_cap);
            }
            // B3: arm the OPPORTUNISTIC Mode 22 probe — its capability is
            // independent of the Mode 01 result (a car can accept one and
            // refuse the other), so it arms whenever chunking is configured
            // and the adapter can carry a multi-pid wire at all; the first
            // composed pair of Mode 22 pids decides it.
            plugin.set_mode22_enabled(true);
        }
        eprintln!(
            "Identify: J1979 chunking {}",
            if capable {
                format!("ENABLED (010C0D answered both, cap {adapter_cap})")
            } else {
                "off (vehicle declined)".to_string()
            }
        );
        log_cb!(
            logger,
            "chunk_probe",
            if capable { "capable" } else { "unsupported" }
        );
    }

    /// Adapter handshake (LH1: `LinkHandler::connect` — init list, ATE0
    /// warm-start guard, STPPMC purge, STBC pipe probe). The engine applies
    /// the learned pipe capability to the poll plugin; chunk capability
    /// resets with it — it's per-VEHICLE and the post-identify probe
    /// re-enables it.
    pub(super) fn send_init_commands(&self) -> bool {
        let (link, acquisition, has_processor) = {
            let state = self.shared_state.lock().unwrap();
            (
                Arc::clone(&state.link),
                Arc::clone(&state.acquisition),
                state.command_processor.is_some(),
            )
        };
        if !has_processor {
            return false;
        }
        {
            let mut plugin = acquisition.lock().unwrap();
            plugin.set_pipe_capable(false);
            plugin.set_chunk_capable(false);
        }
        match link.connect() {
            Ok(facts) => {
                acquisition
                    .lock()
                    .unwrap()
                    .set_pipe_capable(facts.pipe_capable);
                true
            }
            Err(_) => false,
        }
    }
}

/// ED2/FS2 probe decoder (pure, LH2: over the parsed reply). Outer `None` =
/// nothing attributable answered (absent — vetoed reply, status token, or no
/// frame from `want`). Inner = the ASCII payload after the positive-response
/// prefix `expect`; `Some(None)` = the module answered (present — an NRC
/// keeps its controller entry in `all_controllers`) but nothing parsable.
///
/// Uses `raw_hex` (tokens after header + PCI, ISO-TP reassembled, BEFORE the
/// echo strip) rather than `data_bytes`: the parser's echo count for these
/// commands is not the positive-response prefix we validate here.
fn decode_probe_ascii(
    reply: &crate::link::LinkReply,
    expect: &[u8],
    want: crate::addressing::Controller,
) -> Option<Option<String>> {
    let parsed = reply.clean_parsed()?;
    let ctrl = parsed.all_controllers.get(&want.key())?;
    let data = strip_lone_pci(hex_tokens(&ctrl.raw_hex));
    if data.len() <= expect.len() || !data.starts_with(expect) {
        return Some(None); // present (it answered), but no parsable positive name
    }
    let text: String = data[expect.len()..]
        .iter()
        .take_while(|&&b| b != 0x00)
        .filter(|&&b| b > 0x1F && b < 0x7F)
        .map(|&b| b as char)
        .collect();
    let trimmed = text.trim().to_string();
    Some(if trimmed.is_empty() {
        None
    } else {
        Some(trimmed)
    })
}

/// SR1 `23 14 <addr BE> 04` decoder (pure, LH2): the 4 data bytes after a
/// positive `63` (an address echo, if the ECU sends one, is skipped); None
/// on NRC / silence / vetoed reply / nothing from `want`.
fn decode_memory_4(
    reply: &crate::link::LinkReply,
    addr: u32,
    want: crate::addressing::Controller,
) -> Option<[u8; 4]> {
    let parsed = reply.clean_parsed()?;
    let ctrl = parsed.all_controllers.get(&want.key())?;
    let data = strip_lone_pci(hex_tokens(&ctrl.raw_hex));
    if data.first() != Some(&0x63) {
        return None; // 7F 23 xx or unparsable
    }
    let addr_be = addr.to_be_bytes();
    let rest = &data[1..];
    let payload = if rest.len() >= 8 && rest[..4] == addr_be {
        &rest[4..8]
    } else {
        rest.get(..4)?
    };
    Some([payload[0], payload[1], payload[2], payload[3]])
}

/// The parser strips only single-frame PCIs (`0X`). A LONE first frame
/// (`10 len …` with its consecutive frames missing) or a stray consecutive
/// frame (`2X …`) keeps its PCI in `raw_hex`; the pre-LH2 per-line decoders
/// stripped those too (2 bytes / 1 byte) — same here, so a truncated
/// multi-frame reply still decodes to the bytes it did carry.
fn strip_lone_pci(mut bytes: Vec<u8>) -> Vec<u8> {
    match bytes.first().map(|b| b >> 4) {
        Some(1) if bytes.len() >= 2 => {
            bytes.drain(..2);
        }
        Some(2) => {
            bytes.remove(0);
        }
        _ => {}
    }
    bytes
}

fn hex_tokens(raw_hex: &str) -> Vec<u8> {
    raw_hex
        .split_whitespace()
        .filter_map(|t| u8::from_str_radix(t, 16).ok())
        .collect()
}

/// LH2 equivalence corpus for the two probe decoders: the verdicts were
/// pinned against the pre-LH2 raw-text decoders (present / absent /
/// NRC-present / multi-frame / 29-bit / address echo / multi-ECU / trailing
/// error veto / lone frames) and the parsed-reply implementation must
/// reproduce them. Each fixture is turned into a `LinkReply` exactly the way
/// `ElmHandler::request` does. Three deliberate deviations are pinned as
/// such below.
#[cfg(test)]
mod probe_decoder_corpus {
    use crate::addressing::{Addressing, Bus, Controller};
    use crate::link::LinkReply;

    fn reply(raw: &str, command: &str, bus: Bus) -> LinkReply {
        crate::link::elm::decode_segment(command, raw, bus)
    }
    fn decode_probe_ascii(
        raw: &str,
        expect: &[u8],
        want: Controller,
        bus: Bus,
    ) -> Option<Option<String>> {
        let cmd = match expect[0] {
            0x5A => "1AB0",
            _ => "22F188",
        };
        super::decode_probe_ascii(&reply(raw, cmd, bus), expect, want)
    }
    fn decode_memory_4(raw: &str, addr: u32, want: Controller, bus: Bus) -> Option<[u8; 4]> {
        super::decode_memory_4(&reply(raw, &format!("2314{addr:08X}04"), bus), addr, want)
    }

    fn b11() -> Bus {
        Bus::new(Addressing::Can11)
    }
    fn b29() -> Bus {
        Bus::new(Addressing::Can29 { tester: 0xF1 })
    }
    const F188: &[u8] = &[0x62, 0xF1, 0x88];
    const AB0: &[u8] = &[0x5A, 0xB0];

    #[test]
    fn ascii_single_frame_present() {
        let raw = "7E8 08 62 F1 88 47 4D 42 4D 32";
        assert_eq!(
            decode_probe_ascii(raw, F188, Controller::new(0x7E0), b11()),
            Some(Some("GMBM2".into()))
        );
    }

    #[test]
    fn ascii_multi_frame_bare_cr() {
        let raw = "7E8 10 13 62 F1 88 48 55 38 4C\r7E8 21 2D 31 34 4B 32 41 2D 4C\r7E8 22 41 00 00 00 00 00 00 00";
        assert_eq!(
            decode_probe_ascii(raw, F188, Controller::new(0x7E0), b11()),
            Some(Some("HU8L-14K2A-LA".into()))
        );
    }

    #[test]
    fn ascii_nrc_is_present_but_unparsed() {
        assert_eq!(
            decode_probe_ascii("7E8 03 7F 22 31", F188, Controller::new(0x7E0), b11()),
            Some(None)
        );
    }

    #[test]
    fn ascii_other_module_only_is_absent() {
        assert_eq!(
            decode_probe_ascii(
                "7E9 08 62 F1 88 41 42 43 44 45",
                F188,
                Controller::new(0x7E0),
                b11()
            ),
            None
        );
    }

    #[test]
    fn ascii_multi_ecu_picks_wanted() {
        let raw = "7E9 08 62 F1 88 41 42 43 44 45\r7E8 08 62 F1 88 47 4D 42 4D 32";
        assert_eq!(
            decode_probe_ascii(raw, F188, Controller::new(0x7E0), b11()),
            Some(Some("GMBM2".into()))
        );
        assert_eq!(
            decode_probe_ascii(raw, F188, Controller::new(0x7E1), b11()),
            Some(Some("ABCDE".into()))
        );
    }

    #[test]
    fn ascii_no_data_and_error_are_absent() {
        for raw in [
            "NO DATA",
            "CAN ERROR",
            "SEARCHING...\rUNABLE TO CONNECT",
            "BUS ERROR",
            "ERROR",
        ] {
            assert_eq!(
                decode_probe_ascii(raw, F188, Controller::new(0x7E0), b11()),
                None,
                "{raw}"
            );
        }
    }

    #[test]
    fn ascii_empty_or_prompt_only_is_absent() {
        for raw in ["", "\r", ">", "OK", "?"] {
            assert_eq!(
                decode_probe_ascii(raw, F188, Controller::new(0x7E0), b11()),
                None,
                "{raw:?}"
            );
        }
    }

    #[test]
    fn ascii_gmlan_1ab0_low_range() {
        let raw = "644 07 5A B0 42 43 4D 20 41";
        assert_eq!(
            decode_probe_ascii(raw, AB0, Controller::new(0x244), b11()),
            Some(Some("BCM A".into()))
        );
    }

    #[test]
    fn ascii_29bit_multi_frame() {
        let raw = "18 DA F1 10 10 0B 62 F1 97 50 43 4D\r18 DA F1 10 21 2D 41 42 43 00 00 00";
        assert_eq!(
            decode_probe_ascii(raw, &[0x62, 0xF1, 0x97], Controller::new(0x10), b29()),
            Some(Some("PCM-ABC".into()))
        );
    }

    #[test]
    fn ascii_short_positive_is_present_unparsed() {
        assert_eq!(
            decode_probe_ascii("7E8 03 62 F1 88", F188, Controller::new(0x7E0), b11()),
            Some(None)
        );
    }

    #[test]
    fn ascii_payload_all_unprintable_is_present_unnamed() {
        assert_eq!(
            decode_probe_ascii(
                "7E8 06 62 F1 88 00 01 02",
                F188,
                Controller::new(0x7E0),
                b11()
            ),
            Some(None)
        );
    }

    #[test]
    fn ascii_trailing_error_line_vetoes_whole_reply() {
        // Pre-LH2: `raw.contains("ERROR")` refused the reply even though a
        // data frame preceded the token.
        for raw in [
            "7E8 08 62 F1 88 47 4D 42 4D 32\rCAN ERROR",
            "7E8 08 62 F1 88 47 4D 42 4D 32\r<DATA ERROR",
            "7E8 08 62 F1 88 47 4D 42 4D 32\rNO DATA",
        ] {
            assert_eq!(
                decode_probe_ascii(raw, F188, Controller::new(0x7E0), b11()),
                None,
                "{raw}"
            );
        }
    }

    #[test]
    fn ascii_trailing_chatter_or_non_veto_token_is_harmless() {
        // `OK` / `UNABLE TO CONNECT` were never veto words — the data frame still named the module.
        for raw in [
            "7E8 08 62 F1 88 47 4D 42 4D 32\rOK",
            "7E8 08 62 F1 88 47 4D 42 4D 32\rUNABLE TO CONNECT",
        ] {
            assert_eq!(
                decode_probe_ascii(raw, F188, Controller::new(0x7E0), b11()),
                Some(Some("GMBM2".into())),
                "{raw}"
            );
        }
    }

    #[test]
    fn ascii_after_research_still_names() {
        // ATSP0 adapter re-searched mid-identify and then got data (the raw
        // decoder skipped the SEARCHING line; the parser must too).
        let raw = "SEARCHING...\r7E8 08 62 F1 88 47 4D 42 4D 32";
        assert_eq!(
            decode_probe_ascii(raw, F188, Controller::new(0x7E0), b11()),
            Some(Some("GMBM2".into()))
        );
        assert_eq!(
            decode_memory_4(
                "SEARCHING...\r7E8 05 63 47 4D 42 4D",
                0x101C0,
                Controller::new(0x7E0),
                b11()
            ),
            Some([0x47, 0x4D, 0x42, 0x4D])
        );
    }

    #[test]
    fn ascii_lone_first_frame_decodes_what_it_carries() {
        let raw = "7E8 10 13 62 F1 88 48 55 38 4C";
        assert_eq!(
            decode_probe_ascii(raw, F188, Controller::new(0x7E0), b11()),
            Some(Some("HU8L".into()))
        );
    }

    #[test]
    fn ascii_nrc_then_data_on_other_module_is_present_unparsed() {
        let raw = "7E8 03 7F 22 31\r7E9 08 62 F1 88 41 42 43 44 45";
        assert_eq!(
            decode_probe_ascii(raw, F188, Controller::new(0x7E0), b11()),
            Some(None)
        );
    }

    /// Deliberate deviation #2: `7F xx 78` (response pending) followed by the
    /// positive frame from the SAME module — raw concatenated both lines and
    /// reported present-but-unnamed; the parsed reply keeps the module's last
    /// single frame, so the name comes through.
    #[test]
    fn ascii_response_pending_then_positive_now_names() {
        let raw = "7E8 03 7F 22 78\r7E8 08 62 F1 88 47 4D 42 4D 32";
        assert_eq!(
            decode_probe_ascii(raw, F188, Controller::new(0x7E0), b11()),
            Some(Some("GMBM2".into()))
        );
    }

    /// Deliberate deviation #3: single-frame pad bytes — the raw decoder read
    /// every byte after the PCI, so printable pad leaked into the name; the
    /// parser truncates a genuine single frame to its PCI length.
    #[test]
    fn ascii_single_frame_pad_no_longer_leaks() {
        assert_eq!(
            decode_probe_ascii(
                "7E8 05 62 F1 88 41 42 43 44",
                F188,
                Controller::new(0x7E0),
                b11()
            ),
            Some(Some("AB".into()))
        );
    }

    #[test]
    fn memory_trailing_error_vetoes() {
        assert_eq!(
            decode_memory_4(
                "7E8 05 63 47 4D 42 4D\rBUS ERROR",
                0x101C0,
                Controller::new(0x7E0),
                b11()
            ),
            None
        );
    }

    #[test]
    fn memory_positive_without_echo() {
        assert_eq!(
            decode_memory_4(
                "7E8 05 63 47 4D 42 4D",
                0x101C0,
                Controller::new(0x7E0),
                b11()
            ),
            Some([0x47, 0x4D, 0x42, 0x4D])
        );
    }

    #[test]
    fn memory_positive_with_address_echo_single_line() {
        // 63 <addr BE> <4 data> behind a length byte (a pseudo-frame — >8
        // bytes can't be one CAN frame, so the echo branch is only reachable
        // this way): the echo is skipped.
        let raw = "7E8 09 63 00 01 01 C0 47 4D 42 4D";
        assert_eq!(
            decode_memory_4(raw, 0x101C0, Controller::new(0x7E0), b11()),
            Some([0x47, 0x4D, 0x42, 0x4D])
        );
    }

    /// Deliberate deviation #1: the raw decoder read only the FIRST line from
    /// the wanted module and never reassembled ISO-TP, so a 9-byte reply
    /// (address echo + data) split across frames yielded the ADDRESS bytes.
    /// The parsed reply is reassembled, so the data bytes come back. Real
    /// PCMs answer the 4-byte window in a single frame without an echo.
    #[test]
    fn memory_positive_with_address_echo_multi_frame() {
        let raw = "7E8 10 09 63 00 01 01 C0 47 4D\r7E8 21 42 4D 00 00 00 00 00 00";
        assert_eq!(
            decode_memory_4(raw, 0x101C0, Controller::new(0x7E0), b11()),
            Some([0x47, 0x4D, 0x42, 0x4D])
        );
    }

    #[test]
    fn memory_nrc_and_silence() {
        assert_eq!(
            decode_memory_4("7E8 03 7F 23 31", 0x101C0, Controller::new(0x7E0), b11()),
            None
        );
        assert_eq!(
            decode_memory_4("NO DATA", 0x101C0, Controller::new(0x7E0), b11()),
            None
        );
        assert_eq!(
            decode_memory_4(
                "7E9 05 63 47 4D 42 4D",
                0x101C0,
                Controller::new(0x7E0),
                b11()
            ),
            None
        );
    }

    #[test]
    fn memory_short_payload_is_none() {
        assert_eq!(
            decode_memory_4("7E8 03 63 47 4D", 0x101C0, Controller::new(0x7E0), b11()),
            None
        );
    }
}

#[cfg(test)]
mod strategy_probe_tests {
    use super::clean_strategy;

    fn window(parts: &[&[u8]]) -> Vec<u8> {
        parts.iter().flat_map(|p| p.iter().copied()).collect()
    }

    #[test]
    fn cleans_plain_cal_name() {
        // "GMBM" + "2\0\0\0" + "\0\0\0\0" → GMBM2 (Ford_Strategy_CAN.md §5 example)
        let buf = window(&[b"GMBM", &[0x32, 0, 0, 0], &[0, 0, 0, 0]]);
        assert_eq!(clean_strategy(&buf), Some("GMBM2".to_string()));
    }

    #[test]
    fn strips_junk_markers() {
        assert_eq!(
            clean_strategy(b"BSTK1.H32\0\0\0"),
            Some("BSTK1".to_string())
        );
        assert_eq!(
            clean_strategy(b"TXAP4.HEX\0\0\0"),
            Some("TXAP4".to_string())
        );
        assert_eq!(
            clean_strategy(b"FPDM2VBF\0\0\0\0"),
            Some("FPDM2".to_string())
        );
    }

    #[test]
    fn device_only_takes_first_seven() {
        assert_eq!(
            clean_strategy(b"GMBM2  DEVICE ONLY  "),
            Some("GMBM2".to_string())
        );
    }

    #[test]
    fn underscore_names_pass() {
        assert_eq!(
            clean_strategy(b"FPDM2_X01\0\0\0"),
            Some("FPDM2_X01".to_string())
        );
    }

    #[test]
    fn rejects_empty_and_junk_only() {
        assert_eq!(clean_strategy(&[0u8; 12]), None);
        assert_eq!(clean_strategy(b"HEX\0\0\0\0\0\0\0\0\0"), None);
        assert_eq!(clean_strategy(b"..\0\0\0\0\0\0\0\0\0\0"), None);
        // Embedded space (not a bare token) — implausible cal name.
        assert_eq!(clean_strategy(b"AB CD EF GH!"), None);
    }
}
