use super::sp_runtime::periodic_response_id;
use super::*;

/// GS2: mid-stream signal outcomes go to the HOST too (typed event on
/// the response callback) — the jsonl-only log left refused adds
/// discoverable only by timeout.
fn signal_event(cb: &Arc<dyn Fn(String) + Send + Sync>, pid: &str, event: &str) {
    cb(format!(
        r#"{{"type":"stream_signal","payload":{{"pid":"{}","event":"{}"}}}}"#,
        pid, event
    ));
}

/// S3: mid-stream signal changes execute in the keepalive break — the
/// prompt window the alternation already pays for. Adds get a fresh
/// slot (Ford refuses appends within a slot but takes a new slot at
/// position 1); `2A` with the full live slot list is cumulative-safe.
/// Exchange replies come back through the tap into ack_slot.
///
/// Called from the beat loop while the transport op guard is held, AFTER
/// the keepalive and BEFORE the STM re-arm. The internal sequencing is
/// hardware law: `2A 04` stop before any define, and — whenever defines
/// ran — the `2A` re-arm is UNCONDITIONAL (see the HARDWARE TRUTH note
/// inside), so the schedule can't be left dead.
fn process_pending_signals(
    platform: &Arc<dyn crate::platform::OBDPlatformInterface>,
    ctx: &BeatCtx,
) {
    let sleep = |ms: u64| std::thread::sleep(std::time::Duration::from_millis(ms));
    // Locals mirror the beat closure's old captures (same names — the body
    // below is the cut-and-pasted in-gap block, unchanged).
    let logger = &ctx.logger;
    let pending = &ctx.pending;
    let unpack = &ctx.unpack;
    let ack_slot = Arc::clone(&ctx.runtime.ack_slot);
    let slot_fill = &ctx.slot_fill;
    let slot_count = ctx.slot_count;
    let stpx_header = &ctx.stpx_header;
    let rate_mode = ctx.rate_mode;
    let length_cache = &ctx.length_cache;
    let event_callback = &ctx.event_callback;

    let wait_ack = |token: &str| -> Option<String> {
        let (lock, cvar) = &*ack_slot;
        let mut deadline = std::time::Instant::now() + std::time::Duration::from_millis(800);
        let mut pending_seen = false;
        let mut guard = lock.lock().unwrap();
        loop {
            if let Some(line) = guard.as_ref() {
                let flat = line.to_uppercase().replace(' ', "");
                if flat.contains(token) {
                    return guard.take();
                }
                // UDS NRC handling (D2, 2026-08-03): `7F xx 78`
                // = ResponsePending — the PCM took the request
                // and will answer within P2* (seconds). A PCM
                // mid-broadcast says 78 routinely; treating it
                // as silence made EVERY hardware add "refused"
                // while the mock (instant acks) stayed green.
                // Extend once to ~3 s — inside the S3 keepalive
                // tolerance. Any OTHER 7F is a definitive NRC:
                // fail fast (the trace records the line).
                if flat.contains("7F") {
                    if flat.contains("78") {
                        if !pending_seen {
                            pending_seen = true;
                            deadline =
                                std::time::Instant::now() + std::time::Duration::from_millis(3000);
                        }
                        *guard = None; // await the real answer
                    } else {
                        return None; // negative response — done
                    }
                }
            }
            let left = deadline.saturating_duration_since(std::time::Instant::now());
            if left.is_zero() {
                return None;
            }
            let (g, _) = cvar.wait_timeout(guard, left).unwrap();
            guard = g;
        }
    };
    let (adds, removes) = {
        let mut p = pending.lock().unwrap();
        (std::mem::take(&mut p.adds), std::mem::take(&mut p.removes))
    };
    if !removes.is_empty() {
        let mut u = unpack.lock().unwrap();
        for pid in &removes {
            for signals in u.values_mut() {
                signals.retain(|s| s.pid != *pid);
            }
        }
        u.retain(|_, v| !v.is_empty());
        log_cb!(logger, "stream_signal_removed", &removes.join(","));
    }
    // HARDWARE TRUTH (Mustang 23:10 bench, adapter log): a
    // `2C 01` define while the periodic scheduler is RUNNING is
    // never acked — and it KILLS the broadcast (beats kept
    // cycling around a silent car; gauges froze). Defines are
    // only safe against a stopped scheduler: `2A 04` first,
    // then define, then ALWAYS re-arm — even if every define
    // refused — so the schedule can't be left dead.
    let doing_defines = !adds.is_empty();
    // Per-wire trace into the session log — the adapter-log ring
    // wipes every ~4 s at stream rates, so break-window forensics
    // must self-record (bench 2026-08-01: two silent refusals).
    let mut trace: Vec<String> = Vec::new();
    let peek_ack = |ack_slot: &Arc<(Mutex<Option<String>>, std::sync::Condvar)>| {
        ack_slot
            .0
            .lock()
            .unwrap()
            .clone()
            .unwrap_or_else(|| "TIMEOUT".into())
    };
    if doing_defines {
        *ack_slot.0.lock().unwrap() = None;
        platform.write_raw(&format!("STPX H:{stpx_header},D:2A04,R:1"));
        // "016A" (PCI+SID of the positive 2A response) — a bare
        // "6A" token matched broadcast frames ("6A0 …", trace
        // 155347) and reported the scheduler stopped while it
        // was still firing; the defines then hit the running
        // scheduler and the PCM ignored them (NO DATA).
        let stop_ack = wait_ack("016A"); // tolerate timeout — re-arm below restores either way
        trace.push(format!(
            "2A04→{}",
            stop_ack.unwrap_or_else(|| peek_ack(&ack_slot))
        ));
    }
    for pid in adds {
        // Byte length: learned cache first, else a solo probe in
        // this window (first frame's PCI carries the total).
        let mut len = length_cache.lock().unwrap().get(&pid).unwrap_or(0) as usize;
        let echo_bytes = if pid.len() >= 6 { 3 } else { 2 };
        if len == 0 {
            *ack_slot.0.lock().unwrap() = None;
            platform.write_raw(&format!("STPX H:{stpx_header},D:{pid},R:1"));
            // Echo-precise token (e.g. "6203D9") — a bare SID
            // matches random frame data bytes.
            let probe_token = if pid.len() >= 6 {
                format!("62{}", &pid[2..])
            } else {
                format!("41{}", &pid[2..])
            };
            let probe_ack = wait_ack(&probe_token);
            trace.push(format!(
                "probe {pid}→{}",
                probe_ack.clone().unwrap_or_else(|| peek_ack(&ack_slot))
            ));
            if let Some(line) = probe_ack {
                let toks: Vec<&str> = line.split_whitespace().collect();
                // [hdr] [PCI] … — FF (1x xx) carries a 12-bit total.
                let total = toks
                    .get(1)
                    .and_then(|t| u8::from_str_radix(t, 16).ok())
                    .map(|p| {
                        if p >> 4 == 1 {
                            (((p & 0x0F) as usize) << 8)
                                | toks
                                    .get(2)
                                    .and_then(|t| usize::from_str_radix(t, 16).ok())
                                    .unwrap_or(0)
                        } else {
                            p as usize
                        }
                    })
                    .unwrap_or(0);
                len = total.saturating_sub(echo_bytes);
                if len > 0 {
                    length_cache.lock().unwrap().record(&pid, len as u8);
                }
            }
        }
        let src = crate::periodic_stream::source_did(&pid);
        // First-fit: the lowest slot with room for the record
        // (fill counts every ACKED define this stream, including
        // removed-but-still-defined ones — their wire bytes stay
        // occupied). The new record appends at the slot's fill.
        let slot_and_offset = {
            let fill = slot_fill.lock().unwrap();
            (0..slot_count as u8)
                .map(|s| (s, fill.get(&s).copied().unwrap_or(0)))
                .find(|(_, f)| f + len <= crate::periodic_stream::SLOT_DATA_BYTES)
        };
        let (Some(src), Some((slot, offset)), true) = (
            src,
            slot_and_offset,
            len > 0 && len <= crate::periodic_stream::SLOT_DATA_BYTES,
        ) else {
            eprintln!("Stream: cannot add {pid} (len {len}, no slot/source) — stays on poll side");
            log_cb!(logger, "stream_signal_refused", &pid);
            signal_event(event_callback, &pid, "refused");
            continue;
        };
        let define = format!("2C01F2{slot:02X}{src}01{len:02X}");
        *ack_slot.0.lock().unwrap() = None;
        platform.write_raw(&format!("STPX H:{stpx_header},D:{define},R:1"));
        let define_ack = wait_ack("6C01F2");
        trace.push(format!(
            "define {define}→{}",
            define_ack.clone().unwrap_or_else(|| peek_ack(&ack_slot))
        ));
        if define_ack.is_some() {
            unpack.lock().unwrap().entry(slot).or_default().push(
                crate::periodic_stream::PackedSignal {
                    pid: pid.clone(),
                    offset,
                    len,
                },
            );
            *slot_fill.lock().unwrap().entry(slot).or_insert(0) += len;
            log_cb!(
                logger,
                "stream_signal_added",
                &format!("{pid}→slot{slot:02X}")
            );
            signal_event(event_callback, &pid, "added");
            eprintln!("Stream: added {pid} to slot {slot:02X} mid-stream");
        } else {
            log_cb!(logger, "stream_signal_refused", &pid);
            signal_event(event_callback, &pid, "refused");
            eprintln!("Stream: define for {pid} not acked — stays on poll side");
        }
    }
    if doing_defines {
        log_cb!(logger, "stream_add_trace", &trace.join(" | "));
        // Re-arm UNCONDITIONALLY — the scheduler was stopped
        // above; skipping this on all-refused leaves it dead.
        let mut slots: Vec<u8> = slot_fill.lock().unwrap().keys().copied().collect();
        slots.sort_unstable();
        let mut wire = format!("2A{rate_mode:02X}");
        for sl in slots {
            wire.push_str(&format!("{sl:02X}"));
        }
        platform.write_raw(&format!("STPX H:{stpx_header},D:{wire},R:0"));
        sleep(150);
    }
}

impl SessionAPIHandle {
    // MARK: - Tier S: UDS periodic streaming (S1)

    /// Store the app's parsed udsprofiles.json (malformed entries already
    /// dropped by the parser — a missing tag degrades to poll).
    pub fn set_stream_profiles(&self, json: &str) {
        let profiles = crate::periodic_stream::parse_stream_profiles(json);
        eprintln!("Session: {} stream profile(s) loaded", profiles.len());
        self.shared_state.lock().unwrap().stream_profiles = profiles;
    }

    /// Start UDS periodic streaming for the ACTIVE subscription set using the
    /// profile under `tag`. Sequence (the nGauge method): probe `10 03` /
    /// `2C 03` → `2C 01` defines from the subscription pids (lengths from the
    /// learned cache) → `2A <mode>` start → pause polling → monitor mode with
    /// the ingest closure → keepalive thread. ANY failure returns Err with
    /// the demotion reason and leaves polling untouched (fallback posture).
    pub fn start_stream(
        &self,
        // LH3: the monitor tap goes through the session's link handler; the
        // platform handle is no longer needed here. Kept in the signature
        // for the callers' shape until LH5 (`MultiPlatform`) reshapes them.
        _platform: Arc<dyn crate::platform::OBDPlatformInterface>,
        tag: &str,
    ) -> Result<StreamStartInfo, String> {
        let (
            profile,
            processor,
            waiters,
            sub_mgr,
            lengths,
            stream_state,
            callback,
            logger,
            bus,
            current_controller,
            health,
        ) = {
            let state = self.shared_state.lock().unwrap();
            let Some(profile) = state.stream_profiles.get(tag).cloned() else {
                return Err(format!(
                    "no stream profile for tag '{tag}' — staying on poll"
                ));
            };
            let Some(processor) = state.command_processor.as_ref().map(Arc::clone) else {
                return Err("no command processor".to_string());
            };
            let bus = crate::addressing::Bus::new(*state.addressing.lock().unwrap());
            (
                profile,
                processor,
                Arc::clone(&state.reply_waiters),
                Arc::clone(&state.subscription_manager),
                Arc::clone(&state.length_cache),
                Arc::clone(&state.stream_state),
                Arc::clone(&state.response_callback_shared),
                state.logger.clone(),
                bus,
                Arc::clone(&state.current_controller),
                Arc::clone(&state.health),
            )
        };
        if stream_state.lock().unwrap().is_some() {
            return Err("stream already active".to_string());
        }

        // Snapshot the streamable signal set BEFORE quiescing — paused
        // subs no longer report as active.
        let mut signals: Vec<(String, u8, Option<String>)> = {
            let mgr = sub_mgr.lock().unwrap();
            let cache = lengths.lock().unwrap();
            mgr.active_continuous_pids()
                .into_iter()
                .map(|(pid, target)| {
                    let len = cache.get(&pid).unwrap_or(0);
                    (pid, len, target)
                })
                .collect()
        };
        if signals.is_empty() {
            return Err("no active subscription pids to stream".to_string());
        }
        // DETERMINISTIC slot layout, FAST TIER FIRST: the pid set comes out
        // of a HashMap; unsorted it shuffled per connect, and plain
        // alphabetical order let a trans sensor take RPM's slot when the
        // dashboard exceeded slot_count (Mustang: 14 gauges, 10 slots —
        // 221E3A alphabetically beat 010C-adjacent pids out). Fast gauges
        // win slots; ties break by pid for stability. Overflow drops to the
        // (paused) poll side.
        {
            let mgr = sub_mgr.lock().unwrap();
            let rank = |pid: &str| -> u8 {
                match mgr.get_pid_tier(pid).as_deref() {
                    Some("Fast") => 0,
                    Some("Medium") => 1,
                    Some("Slow") => 2,
                    _ => 3,
                }
            };
            signals.sort_by(|a, b| rank(&a.0).cmp(&rank(&b.0)).then_with(|| a.0.cmp(&b.0)));
        }

        // QUIESCE POLLING BEFORE ANY SETUP WIRE (bench 2026-08-01): the
        // whole UDS setup — ATSH, probes AND defines — runs on a QUIET bus.
        // Two failing sessions showed `2C 03` → `7F 2C 7F` with poll wires
        // interleaved into the probe window while identically-shaped
        // sessions succeeded; interleaving is the variable we can remove.
        // (Quiesce must also precede the ATSH: clear_queue would wipe a
        // queued header switch.) Every failure exit below MUST resume +
        // kick (bail).
        let paused = sub_mgr.lock().unwrap().pause_all_active();
        // NO clear_queue here: the queue can hold RUN-ONCE wires (a batch, a
        // console command) whose completions must fire — clearing them
        // silently killed a post-sink batch (SinkMonitorTests caught it once
        // engage started running after sinks). Paused subs stop NEW picks;
        // whatever is queued drains through the correlator during this wait
        // (and FIFO + hold_dispatch protect the setup/monitor phases).
        sub_mgr.lock().unwrap().clear_in_flight();
        let _ = processor.wait_for_idle(std::time::Duration::from_secs(3));
        let bail = |reason: String, probe_tag: Option<&str>| -> String {
            if let Some(tag) = probe_tag {
                log_cb!(logger, "stream_probe", tag);
            }
            sub_mgr.lock().unwrap().resume_many(&paused);
            // FB1 lesson: resuming does NOT revive the starved event chain.
            for id in &paused {
                self.kick_pipeline_if_idle(*id);
            }
            reason
        };

        // The nGauge method is PHYSICAL: every setup command targets the
        // profile's ECU. Functionally-broadcast UDS is a trap — the S197 PCM
        // answers `10 03` on 7DF but silently ignores `2C` (the wizard hit
        // exactly this on the Mustang bench). Same switch the poll chain
        // uses: Bus renders the per-protocol header (11-bit `7E0`, 29-bit
        // `18DA10F1`), current_controller is updated so later poll wires
        // re-switch as needed. Commands are serial FIFO, so the ATSH lands
        // before the probes.
        {
            let header = bus.target_header(&profile.target);
            let needs_switch =
                current_controller.lock().unwrap().as_deref() != Some(header.as_str());
            if needs_switch {
                let atsh = format!("ATSH{header}");
                if let Some(ref log) = logger {
                    log.log_command_sent(&atsh, 5000);
                }
                if processor.queue_command(atsh, 5000).is_err() {
                    return Err("could not queue ATSH for stream target".to_string());
                }
                *current_controller.lock().unwrap() = Some(header);
            }
        }

        // Request/response with the poll chain still running: wait for THIS
        // command's reply slot, not for processor idleness (a live dashboard
        // keeps the queue busy forever). LH7: typed — the acks are checked
        // on payload bytes, never on text.
        let exchange = |cmd: &str| -> Option<crate::link::LinkReply> {
            let slot = waiters.arm(cmd);
            if processor.queue_command(cmd.to_string(), 5000).is_err() {
                waiters.disarm(cmd);
                return None;
            }
            slot.wait(std::time::Duration::from_secs(10))?
                .ok()?
                .segments
                .into_iter()
                .next()
        };
        // Any responder's payload opens with `prefix`?
        let opens_with = |r: &Option<crate::link::LinkReply>, prefix: &[u8]| -> bool {
            r.as_ref().map_or(false, |r| {
                r.payloads.iter().any(|p| p.bytes.starts_with(prefix))
            })
        };
        let shown = |r: &Option<crate::link::LinkReply>| -> String {
            r.as_ref().map(|r| r.raw.clone()).unwrap_or_default()
        };

        // Probe gates (cached failure = the caller just doesn't call again
        // this connect; nothing to persist — same posture as the chunk probe).
        let session_ack = exchange("1003");
        if !opens_with(&session_ack, &[0x50, 0x03]) {
            return Err(bail(
                format!(
                    "10 03 refused ({:?}) — staying on poll",
                    shown(&session_ack)
                ),
                Some("session_refused"),
            ));
        }
        let mut defines_ack = exchange("2C03");
        if !opens_with(&defines_ack, &[0x6C, 0x03]) && opens_with(&defines_ack, &[0x7F]) {
            // `7F 2C 7F` (serviceNotSupportedInActiveSession) seen on the
            // Mustang with the PCM answering `50 03` moments earlier — a
            // wedged/leftover session state (defines persist across session
            // exits per ISO 14229; every kill-while-streaming leaves them).
            // Force a clean default→extended transition and retry once.
            log_cb!(logger, "stream_probe", "defines_7F_retry_via_1001");
            let _ = exchange("1001");
            std::thread::sleep(std::time::Duration::from_millis(300));
            let re_session = exchange("1003");
            if opens_with(&re_session, &[0x50, 0x03]) {
                defines_ack = exchange("2C03");
            }
        }
        if !opens_with(&defines_ack, &[0x6C, 0x03]) {
            // The `7F 2C` shape: platform cannot stream.
            return Err(bail(
                format!(
                    "2C 03 refused ({:?}) — staying on poll",
                    shown(&defines_ack)
                ),
                Some("defines_refused"),
            ));
        }

        // Build defines from the pre-quiesce signal snapshot.
        let set = match crate::periodic_stream::build_defines(&signals, &profile) {
            Ok(set) => set,
            Err(e) => return Err(bail(format!("defines demoted to poll: {e:?}"), None)),
        };
        if set.define_commands.is_empty() {
            return Err(bail(
                format!(
                    "no streamable signals (all dropped: {:?}) — staying on poll",
                    set.dropped
                ),
                None,
            ));
        }
        if !set.dropped.is_empty() {
            eprintln!(
                "Stream: {} signal(s) beyond slot capacity stay on poll: {:?}",
                set.dropped.len(),
                set.dropped
            );
            log_cb!(logger, "stream_slots_dropped", &set.dropped.join(","));
        }

        // Every dynamic define is ≥ 8 data bytes (2C 01 F2 xx + 4-byte source
        // records) — past the ELM-class 7-byte single-frame TX limit, so a
        // plain hex command gets "?" from the ADAPTER, never reaching the bus
        // (Mustang bench 2026-07-31). STN chips segment it themselves via
        // STPX (H:header, D:data, R:1 = return after one response); pure-ELM
        // clones answer "?" to STPX too and correctly stay on poll.
        //
        // A REFUSED record drops just that signal, not the stream: the PCM
        // whitelists definable sources (Mustang: F40C fine, F40D → 7F 2C 31)
        // and positions are explicit per record, so survivors' slices are
        // unaffected. The dropped signal simply stays on the paused poll set
        // (stale while streaming — S3 coexistence picks this up properly).
        let stpx_header = bus.target_header(&profile.target);
        // Any UDS request past 7 data bytes (14 hex chars) needs the STN's
        // segmented transmit — that includes the START command once one-
        // signal-per-slot pushes the slot list past 5 entries (bench round 4:
        // "2A0300010203..." got "?" from the adapter).
        let stpx_wrap = |cmd: &str| -> String {
            if cmd.len() > 14 {
                format!("STPX H:{stpx_header},D:{cmd},R:1")
            } else {
                cmd.to_string()
            }
        };
        let mut refused: Vec<(u8, String)> = Vec::new();
        for (i, cmd) in set.define_commands.iter().enumerate() {
            let wire = stpx_wrap(cmd);
            let ack = exchange(&wire);
            if !opens_with(&ack, &[0x6C, 0x01]) {
                let (slot, pid) = set.record_pids[i].clone();
                eprintln!(
                    "Stream: define for {pid} refused ({:?}) — dropping it from the stream",
                    shown(&ack)
                );
                log_cb!(logger, "stream_define_skipped", &pid);
                refused.push((slot, pid));
            }
        }
        let mut unpack = set.unpack.clone();
        for (slot, pid) in &refused {
            // Removes the signal AND left-shifts later records in the slot —
            // the refused source's bytes never join the concatenation.
            crate::periodic_stream::drop_refused(&mut unpack, *slot, pid);
        }
        if unpack.is_empty() {
            return Err(bail(
                "every define refused — staying on poll".to_string(),
                Some("define_refused"),
            ));
        }
        // Start only the surviving slots.
        let mut live_slots: Vec<u8> = unpack.keys().copied().collect();
        live_slots.sort_unstable();
        // Wire bytes per slot (acked records only) — seeds the first-fit
        // accounting for mid-stream adds.
        let slot_fill: std::collections::HashMap<u8, usize> = unpack
            .iter()
            .map(|(slot, sigs)| (*slot, sigs.iter().map(|s| s.len).sum()))
            .collect();
        let start_command = {
            let mut s = format!("2A{:02X}", profile.rate_mode);
            for slot in &live_slots {
                s.push_str(&format!("{slot:02X}"));
            }
            s
        };
        let start_ack = exchange(&stpx_wrap(&start_command));
        if !opens_with(&start_ack, &[0x6A]) {
            return Err(bail(
                format!(
                    "2A start refused ({:?}) — staying on poll",
                    shown(&start_ack)
                ),
                None,
            ));
        }

        // Polling was quiesced BEFORE the probes (see above) — the round-6
        // tap-attach drain hazard (a queued wire losing its response to the
        // tap → 5 s no-prompt DROP) is covered by the quiesce + this final
        // drain (no clear: run-once wires keep their completions).
        let _ = processor.wait_for_idle(std::time::Duration::from_secs(2));
        // SHARED decode map (S3): the ingest reads it per line; the beat
        // loop mutates it when signals are added/removed mid-stream.
        let unpack: Arc<Mutex<crate::periodic_stream::UnpackMap>> = Arc::new(Mutex::new(unpack));
        let unpack_for_ingest = Arc::clone(&unpack);
        // RS7: stop/ack mechanics live in the shared runtime — created here
        // so its ack_slot / stopped_signal can be cloned into the ingest and
        // transport closures before ActiveStream is constructed.
        let runtime = AcquisitionRuntime::new();
        let response_ids = profile.response_ids.clone();
        let ingest_callback = Arc::clone(&callback);
        let processor_for_ingest = Arc::clone(&processor);
        // RX visibility (Mustang 153856: 51 s of healthy keepalives over a
        // frozen dash, and the jsonl couldn't say whether frames ever
        // arrived). Counters, logged every ~5 s from the ingest itself:
        // lines seen / signals delivered. Zero lines = the CAR is silent;
        // lines without signals = decode is dropping them.
        let rx_lines = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let rx_signals = Arc::new(std::sync::atomic::AtomicU64::new(0));
        let rx_last_log = Arc::new(Mutex::new(std::time::Instant::now()));
        let rx_lines_i = Arc::clone(&rx_lines);
        let rx_signals_i = Arc::clone(&rx_signals);
        let rx_last_log_i = Arc::clone(&rx_last_log);
        let logger_for_rx = logger.clone();
        // SH1: streamed signal deliveries feed the health tracker too.
        let health_for_ingest = Arc::clone(&health);
        let logger_for_health = logger.clone();
        // LH3: only frames whose id is in the profile reach the ingest; the
        // tap acks STOPPED inline and routes chatter / break replies itself.
        let ingest = move |frame: crate::link::Frame| {
            rx_lines_i.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            {
                let mut last = rx_last_log_i.lock().unwrap();
                if last.elapsed() >= std::time::Duration::from_secs(5) {
                    *last = std::time::Instant::now();
                    log_cb!(
                        logger_for_rx,
                        "stream_rx",
                        &format!(
                            "lines={} signals={}",
                            rx_lines_i.swap(0, std::sync::atomic::Ordering::Relaxed),
                            rx_signals_i.swap(0, std::sync::atomic::Ordering::Relaxed)
                        )
                    );
                }
            }
            let decoded = {
                let unpack = unpack_for_ingest.lock().unwrap();
                crate::periodic_stream::decode_periodic_frame(
                    &frame.id,
                    &frame.data,
                    &response_ids,
                    &unpack,
                )
            };
            rx_signals_i.fetch_add(decoded.len() as u64, std::sync::atomic::Ordering::Relaxed);
            // SH1: same count → health tracker (uds-stream tier signals).
            health_note_signals(
                &health_for_ingest,
                decoded.len() as u64,
                &logger_for_health,
                &ingest_callback,
            );
            for (pid, bytes) in decoded {
                // Same shape the poller emits for THIS pid: Mode 01 pids get
                // a solo `41 XX …`, Mode 22 pids a solo `62 XX YY …`.
                // (Indexing pid[4..6] unconditionally PANICKED on 4-char
                // Mode 01 pids — slot 0 was 0105 on the Mustang, so the
                // first decoded frame killed the ingest every round; under
                // the old in-lock delivery the panic even poisoned the
                // platform mutex and wedged the transport.)
                let raw = {
                    let hex: Vec<String> = bytes.iter().map(|b| format!("{b:02X}")).collect();
                    let echo = if pid.len() >= 6 {
                        format!("62 {} {}", &pid[2..4], &pid[4..6])
                    } else {
                        format!("41 {}", &pid[2..4])
                    };
                    format!("{} {}", echo, hex.join(" "))
                };
                let mut controllers = std::collections::HashMap::new();
                controllers.insert(
                    "00".to_string(),
                    crate::response_parser::ControllerResponse {
                        raw_hex: raw.clone(),
                        data_bytes: bytes,
                        controller_id: "00".to_string(),
                        is_valid: true,
                    },
                );
                let parsed = crate::response_parser::ParsedResponse {
                    parse_type: crate::response_parser::ParseType::PidResponse,
                    all_controllers: controllers,
                };
                let message = message_builder::obd_data_message(
                    pid,
                    raw,
                    None,
                    serde_json::to_value(&parsed).ok(),
                );
                if let Ok(json) = MessageProcessor::serialize_message(&message) {
                    ingest_callback(json);
                    // Session Stats: a streamed signal IS a measured
                    // delivery — without this the rolling rates zero out
                    // the moment polling pauses.
                    processor_for_ingest.record_signal_completions(1);
                }
            }
        };
        // DECODE OFF THE TRANSPORT THREAD (bench round 7): at stream rates
        // the receive thread must do nothing but enqueue — running decode +
        // the host data callback inline wedged the BLE thread once one
        // callback stalled, which starved every platform-lock user (the
        // keepalive thread included) and let the S3 timeout kill the stream.
        // A dedicated thread consumes the channel; it exits when the channel
        // closes (exit_monitor_mode drops the sender closure).
        let (line_tx, line_rx) = std::sync::mpsc::channel::<crate::link::Frame>();
        std::thread::spawn(move || {
            // COALESCING consumer: at ~300 lines/s the decode+emit path can
            // run slightly behind arrival — an unbounded backlog made the
            // STOPPED acks miss by seconds (bench round 9, beats 2+). Each
            // wake drains everything pending and keeps only the FRESHEST
            // frame per (CAN id, slot) — gauges want latest values, and the
            // work per cycle is bounded by the slot count.
            loop {
                let first = match line_rx.recv() {
                    Ok(l) => l,
                    Err(_) => break, // sender dropped = monitor exited
                };
                let mut latest: Vec<((String, Option<u8>), crate::link::Frame)> = Vec::new();
                let mut push = |frame: crate::link::Frame| {
                    let key = (frame.id.clone(), frame.data.first().copied());
                    match latest.iter_mut().find(|(k, _)| *k == key) {
                        Some(entry) => entry.1 = frame,
                        None => latest.push((key, frame)),
                    }
                };
                push(first);
                while let Ok(l) = line_rx.try_recv() {
                    push(l);
                }
                for (_, line) in latest {
                    // A decode panic must never kill the stream (one did —
                    // see the echo-shape note in the ingest).
                    let res =
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| ingest(line)));
                    if res.is_err() {
                        eprintln!("Stream: ingest panicked on a line — dropped");
                    }
                }
            }
        });
        let stopped_for_transport = Arc::clone(&runtime.stopped_signal);
        let ack_for_transport = Arc::clone(&runtime.ack_slot);
        let frame_ids: Vec<String> = profile.response_ids.clone();
        let link = Arc::clone(&self.shared_state.lock().unwrap().link);
        // LH3: the handler parses each line once; the tap only routes.
        if let Err(e) = link.monitor(Box::new(move |ev: crate::link::LinkEvent| {
            use crate::link::LinkEvent;
            let stash_ack = |text: String| {
                let (lock, cvar) = &*ack_for_transport;
                *lock.lock().unwrap() = Some(text);
                cvar.notify_all();
            };
            match ev {
                // STOPPED is transport chatter, not data — ack it INLINE so
                // the keepalive/stop handshakes never wait behind the decode
                // queue (round 9: backlogged acks cost 1.5 s per beat/stop).
                LinkEvent::Stopped => {
                    let (lock, cvar) = &*stopped_for_transport;
                    *lock.lock().unwrap() = true;
                    cvar.notify_all();
                }
                // A frame whose id is in the profile → decode. Any OTHER
                // frame (the physical 7E8 replies to break-window probes /
                // defines / 2A) is an exchange reply for the beat thread.
                LinkEvent::Frame(frame) => {
                    if frame_ids.iter().any(|r| r.eq_ignore_ascii_case(&frame.id)) {
                        let _ = line_tx.send(frame);
                    } else {
                        stash_ack(frame.raw);
                    }
                }
                LinkEvent::Text(text) => {
                    let t = text.trim();
                    if !t.is_empty() && t != "OK" && t != "?" {
                        stash_ack(text);
                    }
                }
            }
        })) {
            return Err(bail(
                format!("monitor mode failed: {e} — staying on poll"),
                None,
            ));
        }

        // The transport loop (filters → STMA → keepalive alternation) is a
        // SEPARATE phase: `arm_stream_transport` — the host attaches its
        // line tap between start and arm, so no monitor byte goes out
        // before lines can flow back in.
        *stream_state.lock().unwrap() = Some(ActiveStream {
            paused_subs: paused,
            response_ids: profile.response_ids.clone(),
            keepalive_interval_ms: profile.keepalive_interval_ms,
            keepalive_settle_ms: profile.keepalive_settle_ms,
            keepalive_wire: format!("STPX H:{stpx_header},D:3E80,R:0"),
            runtime,
            transport_op: Arc::new(Mutex::new(())),
            unpack,
            pending: Arc::new(Mutex::new(StreamPending::default())),
            slot_fill: Arc::new(Mutex::new(slot_fill)),
            slot_count: profile.slot_count,
            stpx_header: stpx_header.clone(),
            rate_mode: profile.rate_mode,
        });
        // The monitor owns the transport from here: HOLD command dispatch —
        // a command sent during monitor loses its response to the tap and
        // the transport's no-prompt fallback drops the connection (e.g. the
        // pipeline kick when the dashboard returns, console one-shots).
        // Queued work waits; released at stream break/stop.
        processor.set_hold_dispatch(true);
        log_cb!(
            logger,
            "stream_start",
            &format!("{tag}: {} slot(s)", live_slots.len())
        );
        eprintln!(
            "Stream: STARTED ({tag}, {} slot(s), {} signal(s), {} dropped)",
            live_slots.len(),
            set.define_commands.len() - refused.len(),
            refused.len()
        );
        Ok(StreamStartInfo {
            response_ids: profile.response_ids.clone(),
            keepalive_interval_ms: profile.keepalive_interval_ms,
            slots_used: live_slots.len(),
            excluded_pids: {
                let mut excluded = set.dropped.clone();
                excluded.extend(refused.iter().map(|(_, pid)| pid.clone()));
                excluded.sort();
                excluded.dedup();
                excluded
            },
        })
    }

    /// S2: run the transport loop for a started stream. Call AFTER the host
    /// has attached its line tap (lines must be able to flow before the
    /// first monitor byte goes out). Everything here goes via `write_raw` —
    /// monitor traffic must never touch the command correlator:
    /// arm = `STFAC` → `STFPA <id>,7FF` per response id (filters only work
    /// at the prompt — hardware-verified) → `STMA`; then every
    /// `keepaliveIntervalMs`: break (space) → STOPPED handshake → `3E 80`
    /// (response suppressed) → re-arm `STMA`. The whole beat is well inside
    /// the car's ~5 s S3 timeout.
    pub fn arm_stream_transport(
        &self,
        platform: Arc<dyn crate::platform::OBDPlatformInterface>,
    ) -> Result<(), String> {
        let ctx = {
            let state = self.shared_state.lock().unwrap();
            let guard = state.stream_state.lock().unwrap();
            let Some(active) = guard.as_ref() else {
                return Err("no active stream to arm".to_string());
            };
            BeatCtx {
                runtime: active.runtime.clone(),
                transport_op: Arc::clone(&active.transport_op),
                response_ids: active.response_ids.clone(),
                interval_ms: active.keepalive_interval_ms,
                settle_ms: active.keepalive_settle_ms,
                keepalive_wire: active.keepalive_wire.clone(),
                logger: state.logger.clone(),
                pending: Arc::clone(&active.pending),
                unpack: Arc::clone(&active.unpack),
                slot_fill: Arc::clone(&active.slot_fill),
                slot_count: active.slot_count,
                stpx_header: active.stpx_header.clone(),
                rate_mode: active.rate_mode,
                length_cache: Arc::clone(&state.length_cache),
                event_callback: Arc::clone(&state.response_callback_shared),
            }
        };
        std::thread::spawn(move || {
            use std::sync::atomic::Ordering;
            // The beat thread owns the WHOLE snapshot — the in-gap signal
            // work reads it through process_pending_signals; the loop's own
            // fields are bound by reference here (same names as before).
            let BeatCtx {
                ref runtime,
                ref transport_op,
                ref response_ids,
                interval_ms,
                settle_ms,
                ref keepalive_wire,
                ref logger,
                ref stpx_header,
                ..
            } = ctx;
            // Same names as before — the loop body below is untouched (RS7:
            // only the storage moved into the shared AcquisitionRuntime).
            let loop_stop = &runtime.loop_stop;
            let stopped_signal = &runtime.stopped_signal;
            let monitor_active = &runtime.monitor_active;
            let sleep = |ms: u64| std::thread::sleep(std::time::Duration::from_millis(ms));
            let wait_stopped = || runtime.wait_stopped(std::time::Duration::from_millis(1500));
            // Arm.
            {
                let _op = transport_op.lock().unwrap();
                platform.write_raw("STFAC");
                sleep(100);
                for id in response_ids {
                    platform.write_raw(&format!("STFPA {id},7FF"));
                    sleep(100);
                }
                // D2 ROOT CAUSE (Mustang trace 160547): break-window STPX
                // R:1 exchanges (2A04 stop, probes, defines) get their reply
                // on the PHYSICAL response id (7E8) — which these pass
                // filters BLOCKED. Every break-window request read NO DATA /
                // TIMEOUT on hardware while broadcasts flowed on, so every
                // in-gap add died against the still-running scheduler. Admit
                // the physical id; mid-monitor it is silent (keepalive is
                // response-suppressed), so it costs nothing.
                let phys = periodic_response_id(stpx_header);
                platform.write_raw(&format!("STFPA {phys},7FF"));
                sleep(100);
                // STM, NOT STMA: STMA means "monitor ALL" and IGNORES the
                // pass filters — the Mustang bench flooded the BLE link with
                // the whole bus (BUFFER FULL → dropped prompt). STM honors
                // the filter set.
                platform.write_raw("STM");
                monitor_active.store(true, Ordering::SeqCst);
            }
            log_cb!(logger, "stream_armed", &response_ids.join(","));
            eprintln!("Stream: monitor armed (filters {})", response_ids.join(","));

            // Keepalive/monitor alternation. The FIRST beat fires at half
            // the interval: the S3 clock started at the `2A` start command,
            // a beat's worth of time before the monitor armed — a full first
            // interval left ~0.4 s of margin on a 5 s S3 (bench round 7).
            let mut next_wait = interval_ms / 2;
            loop {
                let mut waited = 0u64;
                while waited < next_wait && !loop_stop.load(Ordering::SeqCst) {
                    sleep(50);
                    waited += 50;
                }
                next_wait = interval_ms;
                if loop_stop.load(Ordering::SeqCst) {
                    break;
                }
                let _op = transport_op.lock().unwrap();
                if loop_stop.load(Ordering::SeqCst) {
                    break;
                }
                log_cb!(logger, "stream_keepalive", "begin");
                // Break → STOPPED → keepalive → (pending signal work) → re-arm.
                *stopped_signal.0.lock().unwrap() = false;
                monitor_active.store(false, Ordering::SeqCst);
                platform.write_raw(" ");
                let acked = wait_stopped();
                platform.write_raw(keepalive_wire);
                // Post-keepalive settle — profile-tunable (keepaliveSettleMs,
                // default 200 = the bench-era constant). This is the beat's
                // feed hole: at 30 gauges it IS the visible global stall, so
                // the mock runs it near-zero and the hardware floor gets
                // benched per-profile (Roadmap 1.1).
                sleep(settle_ms);

                // S3: mid-stream signal changes execute HERE — the prompt
                // window the alternation already pays for (see
                // process_pending_signals; its 2A04-stop → define →
                // unconditional 2A re-arm sequencing is hardware law).
                process_pending_signals(&platform, &ctx);

                if loop_stop.load(Ordering::SeqCst) {
                    break; // stop owns the adapter now — leave it at the prompt
                }
                platform.write_raw("STM");
                monitor_active.store(true, Ordering::SeqCst);
                log_cb!(
                    logger,
                    "stream_keepalive",
                    if acked {
                        "beat"
                    } else {
                        "beat (no STOPPED ack)"
                    }
                );
            }
        });
        Ok(())
    }

    /// S2 stop, phase 1: halt the transport loop and break out of monitor
    /// mode (STOPPED handshake through the still-attached line tap), clear
    /// the pass filters, exit monitor. The host detaches its tap AFTER this
    /// returns and BEFORE `stop_stream` — the teardown commands are ordinary
    /// request/response traffic and their replies must reach the command
    /// correlator, not the tap (bench round 5: teardown replies swallowed by
    /// the tap → commands never completed → the transport's no-prompt
    /// fallback dropped the whole connection).
    pub fn break_stream_transport(&self, platform: Arc<dyn crate::platform::OBDPlatformInterface>) {
        let stream_state = {
            let state = self.shared_state.lock().unwrap();
            Arc::clone(&state.stream_state)
        };
        // Clone the handles OUT and DROP the stream_state lock before any
        // transport work: holding it across the break (write + 1.5 s
        // STOPPED wait + exit_monitor_mode) parked every other FFI that
        // touches stream_state — the Session Stats timer and gauge toggles
        // run on the MAIN thread, so a slow exit read as a BEACHBALL
        // (hardware 2026-08-01).
        let (runtime, transport_op) = {
            let guard = stream_state.lock().unwrap();
            let Some(active) = guard.as_ref() else { return };
            (active.runtime.clone(), Arc::clone(&active.transport_op))
        };
        runtime
            .loop_stop
            .store(true, std::sync::atomic::Ordering::SeqCst);
        {
            let _op = transport_op.lock().unwrap();
            if runtime
                .monitor_active
                .load(std::sync::atomic::Ordering::SeqCst)
            {
                runtime.clear_stopped();
                platform.write_raw(" ");
                let _ = runtime.wait_stopped(std::time::Duration::from_millis(1500));
                // Leftover pass filters would HIDE 7E8 poll responses —
                // clear them while we still own the transport.
                platform.write_raw("STFAC");
                std::thread::sleep(std::time::Duration::from_millis(100));
                runtime
                    .monitor_active
                    .store(false, std::sync::atomic::Ordering::SeqCst);
            }
        }
        platform.exit_monitor_mode();
        // Transport back at the prompt — release held dispatch (teardown
        // commands + anything queued during the stream).
        let processor = {
            let state = self.shared_state.lock().unwrap();
            state.command_processor.as_ref().map(Arc::clone)
        };
        if let Some(processor) = processor {
            processor.set_hold_dispatch(false);
        }
    }

    /// S3: queue a signal add/remove for a LIVE stream — executed in the
    /// next keepalive break (≤ one beat interval away), no restart. Returns
    /// false when no stream is active (polling handles the change normally).
    pub fn stream_change_signal(&self, pid: &str, add: bool) -> bool {
        let state = self.shared_state.lock().unwrap();
        let guard = state.stream_state.lock().unwrap();
        let Some(active) = guard.as_ref() else {
            return false;
        };
        let pid = pid.trim().to_uppercase();
        let mut p = active.pending.lock().unwrap();
        if add {
            p.removes.retain(|x| *x != pid);
            if !p.adds.contains(&pid) {
                p.adds.push(pid);
            }
        } else {
            p.adds.retain(|x| *x != pid);
            if !p.removes.contains(&pid) {
                p.removes.push(pid);
            }
        }
        true
    }

    /// Tear the stream down: `2A 04` stop · `2C 03` clear · `10 01` default
    /// session, resume polling. Call AFTER `break_stream_transport` and the
    /// host's tap detach (the replies must reach the correlator). Safe to
    /// call when idle; runs the break itself if the host skipped it.
    // MARK: - S4: streaming behind the subscription API

    /// Host hands over the vinrules `udsPeriodic` tag once the VIN is known
    /// (vinrules is host-side data; this is the single handoff). None/empty
    /// clears it (poll-only vehicle).
    pub fn set_stream_tag(&self, tag: Option<String>) {
        {
            let state = self.shared_state.lock().unwrap();
            let mut slot = state.stream_tag.lock().unwrap();
            *slot = tag.filter(|t| !t.is_empty());
            log_cb!(
                state.logger,
                "stream_tag",
                slot.as_deref().unwrap_or("(cleared)")
            );
        }
        self.emit_tier_availability(); // vehicle fact changed
    }

    /// DR4: hand the session-config blob (datasets.json, raw) to the session ONCE at
    /// setup. Parses the session slice; a malformed blob clears the registry (the
    /// session then behaves dataset-less — fail open, like every config surface).
    pub fn set_dataset_registry(&self, json: Option<String>) {
        let state = self.shared_state.lock().unwrap();
        let mut slot = state.dataset_registry.lock().unwrap();
        *slot = match json.as_deref().filter(|j| !j.is_empty()) {
            None => None,
            Some(j) => match crate::dataset_registry::SessionConfig::parse(j) {
                Ok(cfg) => {
                    log_cb!(
                        state.logger,
                        "dataset_registry",
                        &format!("{} dataset(s)", cfg.datasets.len())
                    );
                    Some(cfg)
                }
                Err(e) => {
                    log_cb!(state.logger, "dataset_registry_error", &e);
                    None
                }
            },
        };
    }

    /// Host acks a monitor_control attach/detach (the tap is wired/unwired).
    pub fn monitor_ready(&self) {
        let ack = {
            let state = self.shared_state.lock().unwrap();
            Arc::clone(&state.monitor_ack)
        };
        *ack.0.lock().unwrap() = true;
        ack.1.notify_all();
    }

    fn emit_json(&self, json: String) {
        let cb = {
            let state = self.shared_state.lock().unwrap();
            Arc::clone(&state.response_callback_shared)
        };
        cb(json);
    }

    pub(super) fn emit_stream_state(&self, state_name: &str, extra: serde_json::Value) {
        let mut payload = serde_json::json!({ "state": state_name });
        if let (Some(obj), Some(ex)) = (payload.as_object_mut(), extra.as_object()) {
            for (k, v) in ex {
                obj.insert(k.clone(), v.clone());
            }
        }
        self.emit_json(
            serde_json::json!({ "type": "stream_state", "payload": payload }).to_string(),
        );
    }

    /// Ask the host to attach/detach the monitor tap and wait for the ack.
    /// Badge toggle: pin/unpin plain polling. Pinning stops any live
    /// stream/periodic and holds poll; unpinning re-runs the engage ladder.
    pub fn set_user_pinned_poll(&self, pinned: bool) {
        {
            let st = self.shared_state.lock().unwrap();
            st.user_pinned_poll
                .store(pinned, std::sync::atomic::Ordering::SeqCst);
        }
        self.emit_tier_availability(); // user pin changed
        if pinned {
            // Drop whatever is live back to poll (proven downgrade paths).
            let stream_live = {
                let st = self.shared_state.lock().unwrap();
                let live = st.stream_state.lock().unwrap().is_some();
                live
            };
            let periodic_live = {
                let st = self.shared_state.lock().unwrap();
                let live = st.periodic_state.lock().unwrap().is_some();
                live
            };
            if stream_live {
                self.request_stop_stream();
            }
            if periodic_live {
                self.request_stop_stn_periodic(false);
            }
        } else {
            self.maybe_engage_stream();
        }
    }

    /// STPPMA tier periods from the host config (clamped 5..=10000 ms).
    pub fn set_stn_periods(&self, fast_ms: u32, medium_ms: u32, slow_ms: u32) {
        let clamp = |v: u32| v.clamp(5, 10_000);
        let state = self.shared_state.lock().unwrap();
        state.host_facts.lock().unwrap().tier_periods = crate::stn_periodic::TierPeriods {
            fast_ms: clamp(fast_ms),
            medium_ms: clamp(medium_ms),
            slow_ms: clamp(slow_ms),
        };
    }

    /// SR2b: Settings "Attempt calibration read on connect" (default ON).
    /// Consulted by identify's strategy read (Ford `$23` probe / GM `F189`).
    pub fn set_calibration_read(&self, enabled: bool) {
        self.shared_state.lock().unwrap().calibration_read_enabled = enabled;
    }

    /// Settings → Data Acquisition: enable/disable the upgrade plugins.
    /// Stored on the session; the engage ladder consults them per attempt.
    pub fn set_acquisition_plugins(&self, uds_enabled: bool, stn_enabled: bool) {
        {
            let mut state = self.shared_state.lock().unwrap();
            state.uds_stream_enabled = uds_enabled;
            state.stn_periodic_enabled = stn_enabled;
        }
        self.emit_tier_availability(); // user gates changed
    }

    /// SP-LEN: host-derived length seed (fills holes only).
    pub fn seed_pid_length(&self, pid: &str, len: u8) {
        let cache = {
            let state = self.shared_state.lock().unwrap();
            Arc::clone(&state.length_cache)
        };
        cache.lock().unwrap().seed(pid, len);
    }

    /// LH0: the line tap is Rust-side now — `enter_monitor_mode` /
    /// `exit_monitor_mode` flip the accumulator themselves, so there is no
    /// host tap to wire and nothing to wait for. The `monitor_control` event
    /// is still EMITTED (the host's event stream / session log stay
    /// identical — the LH0 gate); the host just no longer has to answer it.
    /// `monitor_ack` / `obd_monitor_ready` stay inert until LH1 removes them.
    pub(super) fn monitor_control(&self, action: &str, _timeout: std::time::Duration) -> bool {
        self.emit_json(
            serde_json::json!({ "type": "monitor_control", "payload": { "action": action } })
                .to_string(),
        );
        true
    }
}
