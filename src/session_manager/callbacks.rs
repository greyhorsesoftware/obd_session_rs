use super::*;

/// One member's resolution out of the demux: (emission raw, parsed,
/// completion data, learned length).
struct Resolved {
    member: String,
    raw: Option<String>,
    parsed: Option<serde_json::Value>,
    completion: Option<String>,
    /// LH7: the member's observed data length (plain single-PID replies
    /// teach the length cache; chunk members never do).
    learned: Option<u8>,
}

/// Everything `demux_segments` learned this exchange; applied by
/// `apply_chunk_outcomes` right after the loop.
struct DemuxOutcome {
    resolved: Vec<Resolved>,
    /// Members whose learned lengths must be invalidated (chunk demotion).
    invalidate: Vec<String>,
    /// Members the vehicle silently omitted from a chunk (route solo).
    omitted: Vec<String>,
    /// B3: outcome of a Mode 22 multi-DID segment this exchange —
    /// (ok, wire size) drives the capability size ladder.
    mode22_outcome: Option<(bool, usize)>,
}

/// Resolve the completed members. Plugin-built transactions
/// registered their member PIDs under the exact wire string
/// (common-ground #4: never re-derive by parsing the command);
/// commands outside the map (init, direct API sends, ATSH
/// switches) fall back to the legacy extraction: ONLY the targeted
/// compound form ("ATSH7E0 010C") takes the last token; any other
/// spaced/piped command IS the pid as sent (the "STFPA 7E8,7FF"
/// bug family).
fn resolve_members(in_flight_members: &InFlightMembers, command: &str) -> Vec<Vec<String>> {
    in_flight_members
        .lock()
        .unwrap()
        .remove(command)
        .unwrap_or_else(|| {
            vec![vec![if command.contains('|') {
                command.to_string()
            } else {
                let mut parts = command.split_whitespace();
                match (parts.next(), parts.next()) {
                    (Some(first), Some(last)) if first.to_uppercase().starts_with("ATSH") => {
                        last.to_string()
                    }
                    _ => command.to_string(),
                }
            }]]
        })
}

/// BP1 demux (LH7: typed): the wire's reply arrives as one `LinkReply` per
/// pipe segment, split by POSITION by the handler (STBCOF 0 mirrors input
/// pipes 1:1) so segment j belongs to seg_members[j]. A single-member
/// transaction whose reply the handler split anyway (a console piped
/// one-shot — ONE member holding the full string) gets the segments merged.
fn demux_segments(
    seg_members: &[Vec<String>],
    reply: Option<&crate::link::WireReply>,
    error: &Option<String>,
    length_cache: &Arc<Mutex<crate::length_cache::LengthCache>>,
    logger: &Option<Arc<SessionLogger>>,
    command: &str,
) -> DemuxOutcome {
    let segments: Vec<crate::link::LinkReply> = match reply {
        Some(r) if error.is_none() => {
            if seg_members.len() == 1 && r.segments.len() > 1 {
                let first = &r.segments[0];
                let payloads = r
                    .segments
                    .iter()
                    .flat_map(|s| s.payloads.iter().cloned())
                    .collect();
                vec![crate::link::LinkReply::from_payloads(
                    &seg_members[0][0],
                    first.status.clone(),
                    r.segments.iter().any(|s| s.faulted),
                    payloads,
                    r.raw.clone(),
                )]
            } else {
                r.segments.clone()
            }
        }
        _ => Vec::new(),
    };
    if error.is_none() && segments.len() < seg_members.len() {
        eprintln!(
            "[PIPE] batch aborted: {} segments for {} (cmd={})",
            segments.len(),
            seg_members.len(),
            command
        );
    }

    let mut resolved: Vec<Resolved> = Vec::new();
    let mut invalidate: Vec<String> = Vec::new();
    let mut omitted: Vec<String> = Vec::new();
    // B3: outcome of a Mode 22 multi-DID segment this exchange —
    // (ok, wire size) drives the capability size ladder.
    let mut mode22_outcome: Option<(bool, usize)> = None;
    for (j, seg_m) in seg_members.iter().enumerate() {
        let seg = if error.is_some() {
            None
        } else {
            segments.get(j)
        };
        match (seg, seg_m.len()) {
            (None, _) => {
                for m in seg_m {
                    resolved.push(Resolved {
                        member: m.clone(),
                        raw: None,
                        parsed: None,
                        completion: None,
                        learned: None,
                    });
                }
            }
            (Some(seg), 1) => {
                // Length learning (observe-then-batch): every plain
                // single-PID reply — including each piped segment — is
                // self-delimiting; observed_len rejects the unlearnable.
                resolved.push(Resolved {
                    member: seg_m[0].clone(),
                    raw: Some(seg.raw.clone()),
                    parsed: seg
                        .parsed_for(&seg_m[0])
                        .and_then(|r| serde_json::to_value(&r).ok()),
                    completion: Some(seg.raw.clone()),
                    learned: crate::length_cache::observed_len(&seg_m[0], &seg.payloads),
                });
            }
            (Some(seg), _) => {
                let text = &seg.raw;
                let lens: Option<Vec<(String, u8)>> = {
                    let cache = length_cache.lock().unwrap();
                    seg_m
                        .iter()
                        .map(|m| cache.get(m).map(|l| (m.clone(), l)))
                        .collect()
                };
                let split = match lens {
                    Some(pairs) => crate::response_parser::split_multi_pid(&pairs, &seg.payloads),
                    // Length evaporated between build and demux —
                    // treat as validation failure (re-observe all).
                    None => Err(crate::response_parser::ChunkSplitError::Validation(
                        "member length missing at demux".to_string(),
                    )),
                };
                let is_mode22 = seg_m[0].starts_with("22");
                match split {
                    Ok(map) => {
                        if is_mode22 {
                            mode22_outcome = Some((true, seg_m.len()));
                        }
                        for m in seg_m {
                            match map.get(m) {
                                Some(parsed) => {
                                    // Completion carries the member's own
                                    // echo (lowest ECU) so the response
                                    // cross-check attributes it correctly.
                                    let completion = parsed
                                        .all_controllers
                                        .values()
                                        .min_by(|a, b| a.controller_id.cmp(&b.controller_id))
                                        .map(|c| c.raw_hex.clone());
                                    resolved.push(Resolved {
                                        member: m.clone(),
                                        raw: Some(text.clone()),
                                        parsed: serde_json::to_value(parsed).ok(),
                                        completion,
                                        learned: None,
                                    });
                                }
                                // Silent omission (B2): the vehicle
                                // answered the chunk but left this
                                // member out — it would starve if
                                // re-chunked, so it solos from here.
                                None => {
                                    omitted.push(m.clone());
                                    resolved.push(Resolved {
                                        member: m.clone(),
                                        raw: None,
                                        parsed: None,
                                        completion: None,
                                        learned: None,
                                    });
                                }
                            }
                        }
                    }
                    Err(e) => {
                        // B3: NO DATA on a multi-DID wire is ALSO a
                        // shape rejection — the members answer solo,
                        // so silence means the request form was
                        // refused (Mustang: 3-DID got NO DATA twice
                        // before the 7F 22 31 landed).
                        if is_mode22 {
                            let reason = match &e {
                                crate::response_parser::ChunkSplitError::Validation(r) => r.clone(),
                                crate::response_parser::ChunkSplitError::NonData => {
                                    "NO DATA".to_string()
                                }
                            };
                            eprintln!(
                                "[CHUNK22] size {} failed (cmd={command}): {reason}",
                                seg_m.len()
                            );
                            log_cb!(logger, "chunk22_refused", &reason);
                            mode22_outcome = Some((false, seg_m.len()));
                        } else if let crate::response_parser::ChunkSplitError::Validation(reason) =
                            &e
                        {
                            {
                                eprintln!(
                                    "[CHUNK] validation failure (cmd={command}): {reason} — demoting {seg_m:?}"
                                );
                                log_cb!(logger, "chunk_validation_failure", reason);
                                invalidate.extend(seg_m.iter().cloned());
                            }
                        }
                        for m in seg_m {
                            resolved.push(Resolved {
                                member: m.clone(),
                                raw: None,
                                parsed: None,
                                completion: None,
                                learned: None,
                            });
                        }
                    }
                }
            }
        }
    }
    DemuxOutcome {
        resolved,
        invalidate,
        omitted,
        mode22_outcome,
    }
}

/// Apply what the demux learned — length invalidation (chunk demotion),
/// silent-omission solo routing, the Mode 22 size ladder, and the B2
/// transport clamp. Returns `chunk_transport_error`: a TRANSPORT failure on
/// a chunk wire (recoverable — the chain continues instead of pausing).
fn apply_chunk_outcomes(
    outcome: &DemuxOutcome,
    seg_members: &[Vec<String>],
    error: &Option<String>,
    length_cache: &Arc<Mutex<crate::length_cache::LengthCache>>,
    acquisition: &Arc<Mutex<Box<dyn crate::acquisition::AcquisitionPlugin>>>,
    logger: &Option<Arc<SessionLogger>>,
    command: &str,
) -> bool {
    let DemuxOutcome {
        invalidate,
        omitted,
        mode22_outcome,
        ..
    } = outcome;
    if !invalidate.is_empty() {
        let mut cache = length_cache.lock().unwrap();
        for m in invalidate {
            cache.invalidate(m);
        }
    }
    if !omitted.is_empty() {
        eprintln!("[CHUNK] members omitted by vehicle (cmd={command}): {omitted:?} — routing solo");
        let mut plugin = acquisition.lock().unwrap();
        for m in omitted {
            plugin.exclude_from_chunks(m);
        }
    }
    if let Some((ok, size)) = *mode22_outcome {
        let changed = acquisition
            .lock()
            .unwrap()
            .note_mode22_chunk_outcome(ok, size);
        if changed && ok {
            eprintln!("[CHUNK22] probe capable (cmd={command}) — Mode 22 pairing enabled");
            log_cb!(logger, "chunk22_probe", "capable");
        }
    }
    // Adapter demotion (B2): a TRANSPORT failure on a chunk wire —
    // e.g. a clone adapter dropping the prompt on a 6-PID request the
    // vehicle probe said was fine — halves the chunk size. Members
    // stay due (completed without data below) and the next sweep
    // rebuilds smaller, so one bad size costs one timeout, not a
    // wedged dashboard.
    let chunk_transport_error = error.is_some() && seg_members.iter().any(|s| s.len() > 1);
    if chunk_transport_error {
        let new_limit = acquisition.lock().unwrap().clamp_chunk();
        eprintln!(
            "[CHUNK] transport failure on chunk wire (cmd={command}) — clamping chunk size to {new_limit}"
        );
        log_cb!(logger, "chunk_clamp", &new_limit.to_string());
    }
    chunk_transport_error
}

/// Emit obd_data + learn lengths — PER MEMBER (Swift routes by pid and
/// stays completely unaware of pipes/chunks). Returns the delivered-member
/// count (signal counter input).
fn emit_member_data(
    resolved: &[Resolved],
    error: &Option<String>,
    length_cache: &Arc<Mutex<crate::length_cache::LengthCache>>,
    logger: &Option<Arc<SessionLogger>>,
    response_callback: &Arc<dyn Fn(String) + Send + Sync>,
) -> u64 {
    let mut signals_delivered: u64 = 0;
    for r in resolved {
        let Some(raw) = &r.raw else { continue };
        signals_delivered += 1;
        if let Some(len) = r.learned {
            length_cache.lock().unwrap().record(&r.member, len);
        }
        let message = message_builder::obd_data_message(
            r.member.clone(),
            raw.clone(),
            error.clone(),
            r.parsed.clone(),
        );
        if let Ok(json) = MessageProcessor::serialize_message(&message) {
            log_cb!(logger, "obd_data", &r.member);
            response_callback(json);
        }
    }
    signals_delivered
}

impl OBDSessionManager {
    /// Create a new OBD session manager
    ///
    /// This spawns a background thread and initializes all services.
    /// Returns immediately after setup is complete.
    pub fn new<F>(
        platform: Arc<dyn OBDPlatformInterface>,
        config: OBDSessionConfig,
        response_callback: F,
    ) -> Result<Self, SessionError>
    where
        F: Fn(String) + Send + Sync + 'static,
    {
        // Wrap response callback in Arc so it can be shared between data callback and SharedState
        let response_callback_arc: Arc<dyn Fn(String) + Send + Sync> = Arc::new(response_callback);

        // Last-connected memory — the connect flow remembers the connector +
        // platform on success; the connection callback uses the platform handle
        // for its stale-disconnect check and clears the memory on drops.
        // (Auto-reconnect itself was removed — drops land on the connect screen.)
        let reconnect_state = Arc::new(crate::reconnect::ReconnectState::new());

        // Create session logger if log path is configured (early, so callbacks can use it)
        let logger: Option<Arc<SessionLogger>> = if let Some(ref log_path) = config.log_base_path {
            let session_name = config
                .session_name
                .clone()
                .unwrap_or_else(|| Uuid::new_v4().to_string());
            match SessionLogger::new(log_path, &session_name) {
                Ok(l) => {
                    let l = Arc::new(l);
                    // WIRE-LOG: the platform tees wire lines into the
                    // logger's adapter_<ts>.log from here on.
                    platform.set_wire_log(Arc::clone(&l));
                    Some(l)
                }
                Err(e) => {
                    eprintln!("Failed to create session logger: {:?}", e);
                    None
                }
            }
        } else {
            None
        };

        // Create shared components
        let pid_registry = Arc::new(Mutex::new(GlobalPIDRegistry::new(config.pid_cache_ttl_ms)));
        let subscription_manager = Arc::new(Mutex::new(SubscriptionManager::new(
            Arc::clone(&pid_registry),
            config.max_subscriptions,
        )));

        // Create command processor
        let command_processor = Arc::new(
            CommandProcessorBuilder::new()
                .commands_per_second(config.command_processor.commands_per_second)
                .max_burst_tokens(config.command_processor.max_burst_tokens)
                .max_queue_size(config.command_processor.max_queue_size)
                .build(Arc::clone(&platform)),
        );

        // Set completion callback for serial command processing.
        // Note: mark_command_completed is also called in the data callback (which fires first)
        // to capture timing for the session monitor. This callback is a safe no-op in that case
        // since the outstanding command is already cleared.
        let command_processor_clone = Arc::clone(&command_processor);
        platform.set_completion_callback(Some(Box::new(move |command: String| {
            command_processor_clone.mark_command_completed(&command);
        })));

        // LH7: per-command reply slots — the data callback fills, the link
        // handler / uds_stream / STN periodic engine wait.
        let reply_waiters: Arc<crate::link::ReplyWaiters> =
            Arc::new(crate::link::ReplyWaiters::default());

        // LH4: host-fed adapter facts — one live cell for the setters and the handler.
        let host_facts: Arc<Mutex<crate::link::HostFacts>> =
            Arc::new(Mutex::new(crate::link::HostFacts::default()));
        // LH1: the ELM handler over this session's processor + capture map.
        // LH5: rebuilt per connect by `build_link` when the catalog says the
        // adapter speaks another dialect (DVI, LH6).
        let link: Arc<dyn crate::link::LinkHandler> = build_link(
            crate::link::LinkKind::Elm,
            &platform,
            &command_processor,
            &reply_waiters,
            &logger,
            &config,
            &host_facts,
        );

        // In-flight wire-command → member PIDs per pipe segment (P0/B1):
        // written when the engine sends a plugin-built transaction, consumed
        // by the data callback (segment j of the response ↔ segments[j]).
        let in_flight_members: InFlightMembers = Arc::new(Mutex::new(HashMap::new()));

        // Per-VIN learned lengths (observe-then-batch) — recorded by the data
        // callback, bound to a VIN file when identify learns the VIN.
        let length_cache: Arc<Mutex<crate::length_cache::LengthCache>> =
            Arc::new(Mutex::new(crate::length_cache::LengthCache::new()));

        // Active acquisition plugin. Shared between the event-driven send in
        // the data callback and the background trigger; owns the length cache
        // handle for chunk admission (B1).
        let acquisition: Arc<Mutex<Box<dyn crate::acquisition::AcquisitionPlugin>>> =
            Arc::new(Mutex::new(Box::new(crate::acquisition::PollPlugin::new(
                crate::acquisition::PluginCtx {
                    length_cache: Arc::clone(&length_cache),
                },
            ))));

        // Session monitor ring buffer
        let session_monitor = Arc::new(Mutex::new(
            crate::session_monitor::SessionMonitorBuffer::new(500),
        ));

        // Set data callback to forward OBD responses through session callback
        // Uses event-driven architecture: each command completion triggers the next
        let current_controller: Arc<Mutex<Option<String>>> = Arc::new(Mutex::new(None));
        let addressing: Arc<Mutex<crate::addressing::Addressing>> =
            Arc::new(Mutex::new(crate::addressing::Addressing::Can11));

        // SH1: session acquisition-health tracker (pure; fed at existing
        // counter sites, emitted through logger + response callback).
        let health = Arc::new(Mutex::new(HealthState::new()));
        let health_for_data = Arc::clone(&health);

        let response_callback_for_data = Arc::clone(&response_callback_arc);
        let subscription_manager_for_data = Arc::clone(&subscription_manager);
        let command_processor_for_data = Arc::clone(&command_processor);
        let logger_for_data = logger.clone();
        let waiters_for_data = Arc::clone(&reply_waiters);
        let current_controller_for_data = Arc::clone(&current_controller);
        let addressing_for_data = Arc::clone(&addressing);
        // Poll-chain default target (2026-08-29, David: "use physical
        // addressing — most mode 01/22 pids are on 7E8"): once identify has
        // learned the primary ECU, a DVI link polls it PHYSICALLY unless the
        // datapoint names another controller — a physical request completes
        // on its reply (no functional quiet window). Identify stays
        // functional (the 0100 roster is the responder COUNT). ELM links
        // keep functional polling for now (their adaptive timing already
        // bounds the wait; switching would re-record the goldens).
        let primary_ecu: Arc<Mutex<Option<crate::addressing::Controller>>> =
            Arc::new(Mutex::new(None));
        let primary_ecu_for_data = Arc::clone(&primary_ecu);
        let host_facts_for_data = Arc::clone(&host_facts);
        let session_monitor_for_data = Arc::clone(&session_monitor);
        let in_flight_members_for_data = Arc::clone(&in_flight_members);
        let acquisition_for_data = Arc::clone(&acquisition);
        let length_cache_for_data = Arc::clone(&length_cache);
        let default_timeout = config.command_processor.command_timeout_ms;
        platform.set_data_callback(Some(Box::new(
            move |command: String, outcome: crate::link::LinkOutcome| {
                use std::time::{SystemTime, UNIX_EPOCH};
                // Held for the whole callback: between mark_command_completed
                // (below) and the next-command queueing (end of callback) the
                // processor would otherwise look idle, and a concurrent
                // start_subscription/add_pids kick would birth a SECOND polling
                // chain (pipeline permanently 2 deep).
                let _completion_guard = command_processor_for_data.begin_completion();
                let timestamp = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_secs_f64();

                // LH7: hand the typed outcome to whoever armed a slot for this
                // command (identify / uds_stream / STN engine exchanges).
                waiters_for_data.fill(&command, &outcome);

                // The dialect's reply (typed) and, for logging / the host's
                // `data` string / the Session Monitor, its display text.
                let (reply, error) = match outcome {
                    Ok(r) => (Some(r), None),
                    Err(e) => (None, Some(e)),
                };
                let data: String = reply.as_ref().map(|r| r.raw.clone()).unwrap_or_default();

                // Log response received
                if let Some(ref log) = logger_for_data {
                    if let Some(ref err) = error {
                        log.log_error_received(&command, err);
                    } else {
                        log.log_response_received(&command, &data);
                    }
                }

                // Resolve the completed members (wire-string map first, legacy
                // extraction fallback — see resolve_members).
                let seg_members: Vec<Vec<String>> =
                    resolve_members(&in_flight_members_for_data, &command);
                let members: Vec<String> = seg_members.iter().flatten().cloned().collect();
                // Representative pid for single-pid plumbing below (context lookup,
                // monitor entry).
                let pid = members[0].clone();

                // BP1 demux + per-member resolution (see demux_segments), then
                // apply what it learned: length demotions, silent-omission solo
                // routing, the Mode 22 ladder, and the B2 transport clamp.
                let outcome = demux_segments(
                    &seg_members,
                    reply.as_ref(),
                    &error,
                    &length_cache_for_data,
                    &logger_for_data,
                    &command,
                );
                let chunk_transport_error = apply_chunk_outcomes(
                    &outcome,
                    &seg_members,
                    &error,
                    &length_cache_for_data,
                    &acquisition_for_data,
                    &logger_for_data,
                    &command,
                );
                let resolved = outcome.resolved;

                // Emit obd_data + capture + learn lengths — PER MEMBER (Swift
                // routes by pid and stays completely unaware of pipes/chunks;
                // see emit_member_data).
                {
                    let signals_delivered = emit_member_data(
                        &resolved,
                        &error,
                        &length_cache_for_data,
                        &logger_for_data,
                        &response_callback_for_data,
                    );
                    command_processor_for_data.record_signal_completions(signals_delivered);
                    // SH1: the same delivered-member count feeds the health
                    // tracker (poll-tier signal counting).
                    health_note_signals(
                        &health_for_data,
                        signals_delivered,
                        &logger_for_data,
                        &response_callback_for_data,
                    );

                    // Errors still surface one obd_data for the wire command (no
                    // member got data above when error.is_some()).
                    if error.is_some() {
                        let message = message_builder::obd_data_message(
                            command.clone(),
                            data.clone(),
                            error.clone(),
                            None,
                        );
                        if let Ok(json) = MessageProcessor::serialize_message(&message) {
                            log_cb!(logger_for_data, "obd_data", &command);
                            response_callback_for_data(json);
                        }
                    }
                }

                // Get timing from command processor (data callback fires before completion callback,
                // so this captures timing and clears the outstanding command)
                let timing = command_processor_for_data.mark_command_completed(&command);

                let is_at_cmd = {
                    let upper = pid.to_uppercase();
                    upper.starts_with("AT") || upper.starts_with("ST")
                };

                // Event-driven: complete EVERY member, collecting mismatch info and
                // any subscription completions (a multi-member transaction may
                // finish several run-once subscriptions).
                let mut sub_manager = subscription_manager_for_data.lock().unwrap();
                let mut completed_subscriptions: Vec<uuid::Uuid> = Vec::new();
                let mut pid_mismatch = false;
                let mut response_pid: Option<String> = None;
                for r in &resolved {
                    let c =
                        sub_manager.command_completed(&r.member, r.completion.clone(), timestamp);
                    if let Some(id) = c.completed_subscription {
                        completed_subscriptions.push(id);
                    }
                    pid_mismatch |= c.pid_mismatch;
                    if response_pid.is_none() {
                        response_pid = c.response_pid;
                    }
                }

                // Get subscription context and tier while we hold the lock
                let (sub_id_str, sub_name_str, target_ctrl, tier_str) = if !is_at_cmd {
                    let mut found: (
                        Option<String>,
                        Option<String>,
                        Option<String>,
                        Option<String>,
                    ) = (None, None, None, None);
                    for sub_info in sub_manager.list_subscriptions() {
                        if sub_info.pids.contains(&pid) {
                            found.0 = Some(sub_info.id.to_string());
                            found.1 = sub_info.name.clone();
                            found.2 = sub_info.target_controller.clone();
                            break;
                        }
                    }
                    found.3 = sub_manager.get_pid_tier(&pid);
                    found
                } else {
                    (None, None, None, None)
                };

                // Send subscription_complete notifications if needed
                for completed_id in &completed_subscriptions {
                    let complete_message =
                        message_builder::subscription_complete_message(*completed_id, vec![]);
                    if let Ok(json) = MessageProcessor::serialize_message(&complete_message) {
                        log_cb!(
                            logger_for_data,
                            "subscription_complete",
                            &completed_id.to_string()
                        );
                        response_callback_for_data(json);
                    }
                }

                // Push session monitor entry (mismatch info comes from command_completed)
                {
                    let (q_ms, r_ms, t_ms) = if let Some(ref t) = timing {
                        (t.queue_time_ms, t.response_time_ms, t.total_time_ms)
                    } else {
                        (0.0, 0.0, 0.0)
                    };

                    let entry = crate::session_monitor::SessionMonitorEntry {
                        command: command.clone(),
                        subscription_id: sub_id_str,
                        subscription_name: sub_name_str,
                        target_controller: target_ctrl,
                        response: data.clone(),
                        queue_time_ms: q_ms,
                        response_time_ms: r_ms,
                        total_time_ms: t_ms,
                        success: error.is_none(),
                        pid_mismatch,
                        response_pid,
                        timestamp,
                        is_at_command: is_at_cmd,
                        tier: tier_str,
                    };

                    let mut monitor = session_monitor_for_data.lock().unwrap();
                    monitor.push(entry);
                }

                // On error: pause all subscriptions, don't send next command.
                // App will call start_subscription to resume (which sends AT\r first).
                // A transport failure on a multi-member wire is RECOVERABLE — the
                // clamp above already shrank the batch and the members stay due —
                // so the chain continues instead of pausing the dashboard (the
                // Dragy incident: one choked 6-PID wire froze polling for good).
                // Solo-wire errors keep the conservative pause (disconnect noise).
                // 2026-08-29 (iPad rotation stopped discovery): a solo-wire error
                // whose members are owned by NO continuous subscription — a run-once
                // probe (the extended-PID scan, one-shot reads) — is a per-member
                // outcome, not a verdict on the link. `command_completed` already
                // consumed the member's run (errors count), so the batch completes
                // normally; pausing everything here instead left the discovery
                // overlay stuck forever with no resume path.
                let probe_only = error.is_some() && !chunk_transport_error && {
                    let members: Vec<&String> = resolved.iter().map(|r| &r.member).collect();
                    !members.is_empty()
                        && !sub_manager.get_active_subscriptions().iter().any(|s| {
                            !s.is_run_once_only && members.iter().any(|m| s.pids.contains(*m))
                        })
                };
                if probe_only {
                    eprintln!(
                        "[SESSION] error on run-once probe cmd={} — member failed, chain continues",
                        command
                    );
                }
                if error.is_some() && !chunk_transport_error && !probe_only {
                    eprintln!(
                        "[SESSION] error on cmd={} — pausing all subscriptions",
                        command
                    );
                    // Pause all active subscriptions
                    let sub_ids: Vec<uuid::Uuid> = sub_manager
                        .list_subscriptions()
                        .iter()
                        .filter(|s| s.state == crate::subscription::SubscriptionState::Active)
                        .map(|s| s.id)
                        .collect();
                    for id in sub_ids {
                        let _ = sub_manager.pause_subscription(id);
                    }
                    sub_manager.clear_in_flight();
                    in_flight_members_for_data.lock().unwrap().clear();
                    // Don't trigger next command — pipeline stops
                    command_processor_for_data.set_chain_alive(false);
                } else {
                    // Event-driven: trigger the next exchange — each completion
                    // picks its successor (a subscription start kicks an idle
                    // chain the same way via kick_pipeline_if_idle). Locks are
                    // taken strictly sequentially (sub → drop → plugin → sub →
                    // plugin) to keep the ordering deadlock-free.
                    drop(sub_manager);
                    let (k, admission) = {
                        let plugin = acquisition_for_data.lock().unwrap();
                        (plugin.max_picks(), plugin.admission())
                    };
                    let picks = subscription_manager_for_data
                        .lock()
                        .unwrap()
                        .get_next_commands(k, &admission);
                    let mut queued_next = false;
                    if !picks.is_empty() {
                        let default_target: Option<String> =
                            if host_facts_for_data.lock().unwrap().kind
                                == crate::link::LinkKind::Dvi
                            {
                                primary_ecu_for_data.lock().unwrap().map(|c| c.key())
                            } else {
                                None
                            };
                        let due: Vec<crate::acquisition::DuePick> = picks
                            .into_iter()
                            .map(
                                |(pid, controller, timeout_ms)| crate::acquisition::DuePick {
                                    pid,
                                    controller: controller.or_else(|| default_target.clone()),
                                    timeout_ms,
                                },
                            )
                            .collect();
                        let cmd = acquisition_for_data.lock().unwrap().next_command(&due);
                        if let Some(cmd) = cmd {
                            let timeout = cmd.timeout_ms.unwrap_or(default_timeout);

                            let bus =
                                crate::addressing::Bus::new(*addressing_for_data.lock().unwrap());
                            Self::send_wire_with_controller_switch(
                                &cmd,
                                timeout,
                                &current_controller_for_data,
                                Some(&command_processor_for_data),
                                logger_for_data.as_ref(),
                                bus,
                                &in_flight_members_for_data,
                            );
                            queued_next = true;
                        }
                    }
                    // The chain lives if this completion queued a successor — or
                    // if other work is still pending (e.g. this was a one-shot
                    // completing while the chain's own wire is queued/in flight;
                    // its pids were in-flight-marked so our pick came up empty,
                    // but the chain is NOT dead).
                    if queued_next {
                        command_processor_for_data.set_chain_alive(true);
                    } else if !command_processor_for_data.has_pending_work() {
                        command_processor_for_data.set_chain_alive(false);
                    }
                }
            },
        )));

        // (Connection callback is set after the API handle is created — see below —
        // so it can emit status and clean up through the session.)

        // Start command processor background thread
        let _command_thread = Arc::clone(&command_processor).start_processing_thread()?;

        // SL3: the lifecycle mailbox — created before the connection callback
        // so the callback can hold a pre-cloned Sender (send is lock-free wrt
        // SharedState; iron rule 1 keeps lock work out of callbacks).
        let (lifecycle_tx, lifecycle_rx) = std::sync::mpsc::channel::<worker::LifecycleMsg>();

        // Create shared state with Arc'd callback for internal use
        let response_callback_for_state = Arc::clone(&response_callback_arc);
        let shared_state = Arc::new(Mutex::new(SharedState {
            platform: Arc::clone(&platform),
            stream_tag: Mutex::new(None),
            dataset_registry: Mutex::new(None),
            attached_dataset: Mutex::new(None),
            monitor_ack: Arc::new((Mutex::new(false), std::sync::Condvar::new())),
            engage_pending: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            engage_cancel: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            command_processor: Some(command_processor),
            pid_registry,
            subscription_manager,
            config: config.clone(),
            running: true,
            response_callback: Some(Box::new(move |msg| response_callback_for_state(msg))),
            default_timeout_ms: config.command_processor.command_timeout_ms,
            logger: logger.clone(),
            awaiting_selection: false,
            host_facts,
            adapter_catalog: None,
            user_pinned_poll: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            teardowns_in_flight: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
            uds_stream_enabled: true,
            stn_periodic_enabled: true,
            calibration_read_enabled: true,
            discovered_connectors: Vec::new(),
            session_epoch: Arc::new(std::sync::atomic::AtomicU64::new(0)),
            terminal_claimed: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            link,
            reply_waiters,
            current_controller: Arc::clone(&current_controller),
            addressing: Arc::clone(&addressing),
            primary_ecu,
            session_monitor,
            reconnect: Arc::clone(&reconnect_state),
            sink: Arc::new(crate::sink::SinkBuffer::new()),
            sink_paused_subs: Vec::new(),
            periodic_last_rx_ms: Arc::new(std::sync::atomic::AtomicU64::new(0)),
            periodic_armed_ms: Arc::new(std::sync::atomic::AtomicU64::new(0)),
            periodic_backoff_until_ms: Arc::new(std::sync::atomic::AtomicU64::new(0)),
            sink_stop_flag: Arc::new(std::sync::atomic::AtomicBool::new(false)),
            sink_stop_ack: Arc::new((Mutex::new(false), std::sync::Condvar::new())),
            acquisition: Arc::clone(&acquisition),
            in_flight_members: Arc::clone(&in_flight_members),
            length_cache: Arc::clone(&length_cache),
            stream_profiles: HashMap::new(),
            stream_state: Arc::new(Mutex::new(None)),
            periodic_state: Arc::new(Mutex::new(None)),
            response_callback_shared: Arc::clone(&response_callback_arc),
            health: Arc::clone(&health),
            lifecycle_tx: lifecycle_tx.clone(),
            phase: Arc::new(Mutex::new(worker::Phase::Idle)),
        }));

        // Create API handle
        let api_handle = SessionAPIHandle {
            shared_state: Arc::clone(&shared_state),
        };

        // SL3: spawn the lifecycle worker — the sole executor of lifecycle
        // transitions (engage ladder, stops, disconnect drain, drop cleanup).
        let lifecycle_thread = worker::spawn_lifecycle_worker(api_handle.clone(), lifecycle_rx);

        // Connection callback: forward status to the UI, and on any disconnect free
        // the in-flight command and clear reconnect memory so the UI drops straight
        // to the connect screen. No auto-reconnect: on a half-open BLE link the
        // reconnect loop could stall minutes with no per-step deadline; a clean
        // drop-to-connect is simpler and honest.
        let response_callback_for_connection = Arc::clone(&response_callback_arc);
        let reconnect_for_cb = Arc::clone(&reconnect_state);
        let lifecycle_tx_for_cb = lifecycle_tx.clone();
        platform.set_connection_callback(Some(Box::new(move |status, reason| {
            if status == crate::platform::ConnectionStatus::Disconnected {
                // Capture the transport handle BEFORE clear_and_cancel wipes it —
                // the Drop handler uses it to detect a superseding connect.
                let platform_for_check = reconnect_for_cb.platform();
                reconnect_for_cb.clear_and_cancel();
                // Do NOTHING else inline. This callback is invoked synchronously
                // (often on the app main thread) while the platform's state lock is
                // held — both lock work AND the response-callback forward are unsafe
                // here (hardware-verified: an inline DISCONNECTED forward never
                // reaches the UI, and inline lock work wedged the app ~60s; the old
                // reconnect code's spawn-only discipline worked every time).
                // SL3: enqueue-only now — the lifecycle worker runs the drop
                // cleanup (stale check → claim → DISCONNECTED emit → local
                // teardown → abort outstanding). The claim happens IN the
                // handler because the stale-drop check must precede it and
                // reads platform state (unsafe here).
                let _ = lifecycle_tx_for_cb.send(worker::LifecycleMsg::Drop {
                    platform_check: platform_for_check,
                });
                return; // the worker owns the DISCONNECTED emit
            }

            // Forward status to the UI.
            let message = message_builder::connection_status_message(status, reason);
            if let Ok(json) = MessageProcessor::serialize_message(&message) {
                response_callback_for_connection(json);
            }
        })));

        // Start main background thread
        let shared_state_clone = Arc::clone(&shared_state);
        let background_thread = thread::spawn(move || {
            Self::background_main_loop(shared_state_clone, config);
        });

        Ok(Self {
            api_handle,
            background_thread: Some(background_thread),
            lifecycle_thread: Some(lifecycle_thread),
            shared_state,
        })
    }

    /// Get API handle for synchronous operations
    pub fn api_handle(&self) -> SessionAPIHandle {
        self.api_handle.clone()
    }

    /// Shutdown the session manager
    ///
    /// Signals the background thread to stop and waits for clean shutdown.
    pub fn shutdown(self) -> Result<(), SessionError> {
        // SL1: app quit is terminal intent (claim-then-bump; loses harmlessly if a
        // disconnect already claimed). Quit emits no terminal status regardless of
        // ownership — the app is exiting; the health emit below is once-guarded.
        let (_, _owner) = self.api_handle().claim_terminal();
        // SH1: emit the session summary if disconnect didn't already
        // (tracker guards the double emit).
        self.api_handle().health_finish_and_emit();

        // Signal shutdown
        {
            // Recover from a poisoned lock: shutdown is a simple state write and
            // must not panic across the FFI boundary.
            let mut state = self.shared_state.lock().unwrap_or_else(|p| p.into_inner());
            state.running = false;

            // SL3: wake the lifecycle worker so it exits promptly (it also
            // notices `running == false` on its bounded-recv tick).
            let _ = state.lifecycle_tx.send(worker::LifecycleMsg::Quit);

            // Stop command processor
            if let Some(ref processor) = state.command_processor {
                processor.stop();
            }
        }

        // Wait for background thread to finish
        if let Some(thread_handle) = self.background_thread {
            thread_handle.join().map_err(|_| {
                SessionError::InternalError("Background thread panicked".to_string())
            })?;
        }

        // SL3: join the lifecycle worker. Its handlers are bounded (handler
        // discipline), so this join is too.
        if let Some(thread_handle) = self.lifecycle_thread {
            thread_handle.join().map_err(|_| {
                SessionError::InternalError("Lifecycle worker panicked".to_string())
            })?;
        }

        Ok(())
    }

    /// Main background thread loop
    ///
    /// With event-driven architecture, this only handles cache cleanup.
    /// Subscription polling is triggered by command completions, not a timer.
    fn background_main_loop(shared_state: Arc<Mutex<SharedState>>, config: OBDSessionConfig) {
        // Cache cleanup interval (less frequent than old polling)
        let cleanup_interval = Duration::from_millis(config.poll_interval_ms * 10);

        while shared_state.lock().unwrap().running {
            // Clean up expired PID cache
            {
                let state = shared_state.lock().unwrap();
                let mut registry = state.pid_registry.lock().unwrap();
                registry.clear_expired_cache();
            }

            // SH1: tick the health tracker so 5 s intervals complete even
            // while an engaged tier is silent (tier_silent needs a clock,
            // not a data callback). Arcs copied out first; emission runs
            // outside the shared-state lock.
            {
                let (health, logger, callback, processor) = {
                    let state = shared_state.lock().unwrap();
                    (
                        Arc::clone(&state.health),
                        state.logger.clone(),
                        Arc::clone(&state.response_callback_shared),
                        state.command_processor.as_ref().map(Arc::clone),
                    )
                };
                let events = {
                    let mut hs = health.lock().unwrap();
                    let now = health_now_ms();
                    let mut events = hs.tracker.on_tick(now);
                    // Rolling windows (SH1 Dragy fix): a single-tier session
                    // must still emit windows — roll at MAX_WINDOW_MS with a
                    // proper RTT snapshot, same as tier-switch closes.
                    if hs.tracker.window_due_to_roll(now) {
                        let rtt = hs.window_rtt_ms(processor.map(|p| p.get_stats()));
                        events.extend(hs.tracker.roll_if_due(now, rtt));
                    }
                    events
                };
                if !events.is_empty() {
                    emit_health_events(&events, &logger, &callback);
                }
            }

            // DV3 silent-slot watchdog (1 s cadence).
            periodic_silence_check(&shared_state);

            // Sleep until next cleanup cycle
            thread::sleep(cleanup_interval);
        }
    }

    /// Sends a plugin-built wire transaction, prepending an ATSH controller
    /// switch if needed. Tracks current_controller to avoid redundant
    /// switches, and registers the transaction's members for completion.
    fn send_wire_with_controller_switch(
        cmd: &crate::acquisition::NextCommand,
        timeout: u32,
        current_controller: &Arc<Mutex<Option<String>>>,
        processor: Option<&Arc<CommandProcessor>>,
        logger: Option<&Arc<SessionLogger>>,
        bus: crate::addressing::Bus,
        in_flight_members: &InFlightMembers,
    ) {
        if let Some(target) = cmd.target.as_deref() {
            // Resolve the target (imported "7E0"/"7E8" or a discovered ecu key) to the actual
            // ATSH header in the active protocol. An 11-bit header that can't map onto 29-bit
            // falls back to functional addressing (C3, decision 1).
            let header = bus.target_header(target);

            let needs_switch = {
                let current = current_controller.lock().unwrap();
                current.as_deref() != Some(header.as_str())
            };

            if needs_switch {
                let atsh_command = format!("ATSH{}", header);

                if let Some(log) = logger {
                    log.log_command_sent(&atsh_command, timeout);
                }
                if let Some(proc) = processor {
                    let _ = proc.queue_command(atsh_command, timeout);
                }

                *current_controller.lock().unwrap() = Some(header);
            }
        }

        // Register members under the exact wire string BEFORE queueing, so the
        // data callback can resolve completion without parsing the command.
        in_flight_members
            .lock()
            .unwrap()
            .insert(cmd.wire.clone(), cmd.segments.clone());

        // Send the wire command
        if let Some(log) = logger {
            log.log_command_sent(&cmd.wire, timeout);
        }
        if let Some(proc) = processor {
            let _ = proc.queue_command(cmd.wire.clone(), timeout);
        }
    }
}

/// LH5: the link-handler factory — one per dialect. `Dvi` arrives with LH6;
/// until then it falls back to ELM (the catalog never emits it yet).
pub(super) fn build_link(
    kind: crate::link::LinkKind,
    platform: &Arc<dyn crate::platform::OBDPlatformInterface>,
    processor: &Arc<CommandProcessor>,
    waiters: &Arc<crate::link::ReplyWaiters>,
    logger: &Option<Arc<SessionLogger>>,
    config: &OBDSessionConfig,
    host_facts: &Arc<Mutex<crate::link::HostFacts>>,
) -> Arc<dyn crate::link::LinkHandler> {
    match kind {
        crate::link::LinkKind::Elm => {
            // An ELM link never uses the DVI hooks — clear any a prior DVI handler left.
            platform.set_frame_sink(None);
            platform.set_command_encoder(None);
            platform.set_byte_mode(false);
            crate::link::elm::ElmHandler::new(
                Arc::clone(platform),
                Arc::clone(processor),
                Arc::clone(waiters),
                logger.clone(),
                config.init_commands.clone(),
                config.enable_pipe_batching,
                Arc::clone(host_facts),
            )
        }
        crate::link::LinkKind::Dvi => crate::link::dvi::handler::DviHandler::new(
            Arc::clone(platform),
            Arc::clone(processor),
            Arc::clone(waiters),
            logger.clone(),
            Arc::clone(host_facts),
        ),
    }
}

/// DV3 silent-slot watchdog: a live adapter-periodic tier that has delivered
/// no slot batch for `PERIODIC_SILENT_MS` (measured from the arm or the last
/// batch) is torn down through the normal stop path — polling then either
/// recovers the dashboard or errors visibly — and the ladder is held off for
/// `PERIODIC_BACKOFF_MS` so a dead tool does not flap. The health tracker's
/// `tier_silent` (2 × 5 s) stays as the observability record.
pub(super) const PERIODIC_SILENT_MS: u64 = 3_000;
pub(super) const PERIODIC_BACKOFF_MS: u64 = 30_000;

fn periodic_silence_check(shared_state: &Arc<Mutex<SharedState>>) {
    use std::sync::atomic::Ordering;
    let (live, armed, last, logger, platform, name) = {
        let state = shared_state.lock().unwrap();
        let live = state.periodic_state.lock().unwrap().is_some();
        let kind = state.host_facts.lock().unwrap().kind;
        (
            live,
            state.periodic_armed_ms.load(Ordering::Relaxed),
            state.periodic_last_rx_ms.load(Ordering::Relaxed),
            state.logger.clone(),
            Arc::clone(&state.platform),
            super::tier_facts::periodic_tier_for(kind).as_str(),
        )
    };
    if !live || armed == 0 {
        return;
    }
    let now = health_now_ms();
    let since = now.saturating_sub(last.max(armed));
    if since < PERIODIC_SILENT_MS {
        return;
    }
    log_cb!(
        logger,
        "periodic_silent",
        &format!("{name}: no slot frames for {since} ms — falling back to polling (re-engage held {PERIODIC_BACKOFF_MS} ms)")
    );
    {
        let state = shared_state.lock().unwrap();
        state
            .periodic_backoff_until_ms
            .store(now + PERIODIC_BACKOFF_MS, Ordering::Relaxed);
        state.periodic_armed_ms.store(0, Ordering::Relaxed);
    }
    let api = SessionAPIHandle {
        shared_state: Arc::clone(shared_state),
    };
    api.emit_stream_state("stopping", serde_json::json!({ "plugin": name }));
    api.stop_stn_periodic(platform);
    api.emit_stream_state(
        "stopped",
        serde_json::json!({ "plugin": name, "reason": "silent" }),
    );
    if !api.terminal_intent_stands() {
        api.set_phase(super::worker::Phase::Connected {
            epoch: api.epoch_now(),
        });
    }
}
