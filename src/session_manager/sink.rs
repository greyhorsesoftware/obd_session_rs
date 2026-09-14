use super::*;

impl SessionAPIHandle {
    // MARK: - Sink (passive listen) — plan M7
    // The buffer is session-owned; ANY platform that implements
    // enter_monitor_mode participates. sink_start also silences subscription
    // polling at the session level (previously only the Swift-side MCP
    // takeover guaranteed this).

    /// Enter sink state on the given platform. Fails cleanly when the
    /// platform doesn't support passive monitoring.
    pub fn sink_start(
        &self,
        platform: Arc<dyn crate::platform::OBDPlatformInterface>,
        filter: Option<String>,
        monitor_command: Option<String>,
    ) -> Result<(), String> {
        // LH0: the monitor command Rust re-issues on BUFFER FULL (STN-native,
        // per user direction: STMA, not ELM ATMA).
        let monitor = monitor_command
            .filter(|m| !m.trim().is_empty())
            .unwrap_or_else(|| {
                if filter.as_deref().map(|f| !f.is_empty()).unwrap_or(false) {
                    "STM"
                } else {
                    "STMA"
                }
                .to_string()
            });
        // Copy what we need out of the lock; never call platform/manager
        // methods while holding shared state.
        let (buffer, sub_mgr) = {
            let state = self.shared_state.lock().unwrap();
            // S4 arbitration (Mustang bench 2026-08-01: sniff-vs-engage
            // fights swallowed define responses → 5 s no-prompt DROP, over
            // and over). One transport owner at a time: a LIVE stream
            // refuses outright; an idling engage task is CANCELLED and
            // given a moment to stand down (sliced sleep notices ≤150 ms;
            // only a task mid-setup-on-the-wire keeps us out).
            state
                .engage_cancel
                .store(true, std::sync::atomic::Ordering::SeqCst);
            if state.stream_state.lock().unwrap().is_some() {
                return Err("stream is live — stop streaming before monitoring".to_string());
            }
            if state.periodic_state.lock().unwrap().is_some() {
                return Err(
                    "adapter periodic acquisition is live — stop it before monitoring".to_string(),
                );
            }
            (
                Arc::clone(&state.sink),
                Arc::clone(&state.subscription_manager),
            )
        };
        {
            let engage_pending = {
                let state = self.shared_state.lock().unwrap();
                Arc::clone(&state.engage_pending)
            };
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
            while engage_pending.load(std::sync::atomic::Ordering::SeqCst) {
                if std::time::Instant::now() > deadline {
                    return Err(
                        "stream engage in progress — try again in a few seconds".to_string()
                    );
                }
                std::thread::sleep(std::time::Duration::from_millis(50));
            }
        }

        buffer.start();
        let paused = sub_mgr.lock().unwrap().pause_all_active();
        {
            let mut state = self.shared_state.lock().unwrap();
            state.sink_paused_subs = paused;
        }

        // Drain the in-flight command BEFORE the tap takes the transport
        // (bounded — same courtesy as disconnect's drain). Pausing stops NEW
        // picks, but a response already owed (e.g. the post-connect
        // enable-tests batch) would otherwise be swallowed by monitor mode:
        // the correlator wedges its full timeout and the error path then
        // pauses every subscription — including ones created after the sink.
        let processor = {
            let state = self.shared_state.lock().unwrap();
            state.command_processor.as_ref().map(Arc::clone)
        };
        if let Some(processor) = processor {
            if !processor.wait_for_idle(std::time::Duration::from_secs(2)) {
                // The in-flight command didn't drain — monitor mode would
                // starve it into a timeout error whose pause-all freezes even
                // subscriptions created AFTER the sink (the post-monitor
                // one-shot included; SinkMonitorTests flake 2026-08-16, 4/5
                // at exactly 26s). Abort it instead: the aborted callback
                // never runs so no error propagates, and the member is still
                // due on its paused subscription — sink_stop's resume + kick
                // re-picks it.
                processor.abort_outstanding_and_clear();
            }
        }

        let push_target = Arc::clone(&buffer);
        // LH0: the line filter that lived in the Swift bridge's sink closure
        // (OBDSessionBridge.sinkStart, 2026-08-22) — setup chatter is not
        // bus traffic; BUFFER FULL means the chip EXITED monitor: re-arm
        // immediately (pass filters persist across the restart) and mark the
        // gap in the capture so the hole is visible instead of silent.
        let (stop_flag, stop_ack) = {
            let state = self.shared_state.lock().unwrap();
            (
                Arc::clone(&state.sink_stop_flag),
                Arc::clone(&state.sink_stop_ack),
            )
        };
        stop_flag.store(false, std::sync::atomic::Ordering::SeqCst);
        let rearm_platform = Arc::clone(&platform);
        // LH3: lines arrive parsed once (`LinkEvent`); the tap only routes.
        // `filter` only picks the default monitor command above — the host
        // applies it with its own STFAC/STFPA writes.
        let bus = self.bus();
        let on: Box<dyn Fn(crate::link::LinkEvent) + Send + Sync> =
            Box::new(move |ev: crate::link::LinkEvent| {
                use crate::link::LinkEvent;
                match ev {
                    LinkEvent::Stopped => {
                        if stop_flag.load(std::sync::atomic::Ordering::SeqCst) {
                            let (lock, cvar) = &*stop_ack;
                            *lock.lock().unwrap() = true;
                            cvar.notify_all();
                        } else {
                            // Not our handshake — the chip stopped on its own;
                            // keep it visible in the capture as before.
                            push_target.push("STOPPED".to_string());
                        }
                    }
                    LinkEvent::Frame(frame) => push_target.push_frame(frame),
                    LinkEvent::Text(text) => {
                        let up = text.to_uppercase();
                        if up == "OK" || up == "?" {
                            return;
                        }
                        if up.contains("BUFFER FULL") {
                            eprintln!("sink: BUFFER FULL — re-arming monitor ({monitor})");
                            push_target.push(
                                "# BUFFER FULL — monitor re-armed, frames lost here".to_string(),
                            );
                            rearm_platform.write_raw(&monitor);
                            return;
                        }
                        push_target.push(text)
                    }
                }
            });
        // DV4: on a DVI link the handler owns the capture (PASS filters for
        // the id list, listen-only bus, µs timestamps, frames via its tap);
        // ELM/STN keeps the text monitor + host-applied filters.
        let link = self.link();
        let result = if link.facts().kind == crate::link::LinkKind::Dvi {
            link.capture(&parse_capture_filters(filter.as_deref()), on)
        } else {
            crate::link::elm::monitor_over(platform.as_ref(), bus, on)
        };
        if let Err(e) = result {
            // Roll back: resume polling, deactivate the buffer.
            buffer.stop();
            let paused = {
                let mut state = self.shared_state.lock().unwrap();
                std::mem::take(&mut state.sink_paused_subs)
            };
            sub_mgr.lock().unwrap().resume_many(&paused);
            return Err(e);
        }
        Ok(())
    }

    /// Drain up to `max` captured frames.
    pub fn sink_read(&self, max: usize) -> Vec<crate::sink::SinkFrame> {
        let buffer = Arc::clone(&self.shared_state.lock().unwrap().sink);
        buffer.read(max)
    }

    /// Counters snapshot.
    pub fn sink_stats(&self) -> crate::sink::SinkStats {
        let buffer = Arc::clone(&self.shared_state.lock().unwrap().sink);
        buffer.stats()
    }

    /// Leave sink state: platform exits monitor mode, buffer deactivates
    /// (retaining the tail for a final read), paused subscriptions resume.
    pub fn sink_stop(&self, platform: Arc<dyn crate::platform::OBDPlatformInterface>) {
        // DV4: a DVI capture unwinds in the handler (tap off, comm ON,
        // filters cleared) — no STN handshake, no monitor-exit dead zone.
        let link = self.link();
        if link.facts().kind == crate::link::LinkKind::Dvi {
            link.stop_monitor();
        } else {
            // LH0: leave monitor per the STN handshake (was Swift `sinkStop`):
            // one interrupt byte (the chip discards it), WAIT for the STOPPED
            // acknowledgment through the still-attached tap, then a tiny grace
            // so the trailing prompt flushes before traffic resumes — resuming
            // early races the STOPPED/prompt flush and corrupts the next
            // command's response. Bounded: 1.5 s if no STOPPED lands (monitor
            // already down).
            {
                let (stop_flag, stop_ack) = {
                    let state = self.shared_state.lock().unwrap();
                    (
                        Arc::clone(&state.sink_stop_flag),
                        Arc::clone(&state.sink_stop_ack),
                    )
                };
                *stop_ack.0.lock().unwrap() = false;
                stop_flag.store(true, std::sync::atomic::Ordering::SeqCst);
                platform.write_raw(" ");
                {
                    let guard = stop_ack.0.lock().unwrap();
                    let _ = stop_ack
                        .1
                        .wait_timeout_while(
                            guard,
                            std::time::Duration::from_millis(1500),
                            |acked| !*acked,
                        )
                        .unwrap();
                }
                stop_flag.store(false, std::sync::atomic::Ordering::SeqCst);
                std::thread::sleep(std::time::Duration::from_millis(50));
            }
            platform.exit_monitor_mode();
            // Monitor-exit DEAD ZONE (SinkMonitorTests flake 2026-08-16, mock +
            // real STN alike): the first command written right after the STOPPED
            // handshake can be swallowed while the adapter settles back to
            // prompt mode. If polling resumes first, that swallowed command
            // times out and the error path pauses EVERY subscription — including
            // ones created after the sink. Absorb the dead zone with a
            // sacrificial sync exchange (ATE0 re-assert, response "OK") BEFORE
            // resuming: if it gets eaten, only the sync suffers — and its error,
            // if any, fires before the resume below un-pauses the world.
            {
                let processor = {
                    let state = self.shared_state.lock().unwrap();
                    state.command_processor.as_ref().map(Arc::clone)
                };
                if let Some(processor) = processor {
                    let _ = processor.queue_command("ATE0".to_string(), 2000);
                    if !processor.wait_for_idle(std::time::Duration::from_secs(3)) {
                        // Still wedged — drop it silently so no error propagates
                        // into the freshly resumed subscriptions.
                        processor.abort_outstanding_and_clear();
                    }
                }
            }
        }
        let (buffer, sub_mgr, paused) = {
            let mut state = self.shared_state.lock().unwrap();
            (
                Arc::clone(&state.sink),
                Arc::clone(&state.subscription_manager),
                std::mem::take(&mut state.sink_paused_subs),
            )
        };
        buffer.stop();
        sub_mgr.lock().unwrap().resume_many(&paused);
        // FB1 lesson (same as stop_stream): resuming does NOT revive the
        // starved event chain — kick each resumed sub.
        for id in &paused {
            self.kick_pipeline_if_idle(*id);
        }
        // S4: the sink released the transport — streaming may re-engage
        // (no-op unless tag/pids/connection hold).
        self.maybe_engage_stream();
    }
}

/// DV4: `"7E8,7A8/7FF,18DAF110"` → capture filters (default mask = every id
/// bit; 8 hex digits = 29-bit). Bad entries are skipped.
fn parse_capture_filters(list: Option<&str>) -> Vec<crate::link::CaptureFilter> {
    let mut out = Vec::new();
    for entry in list
        .unwrap_or("")
        .split(|c: char| c == ',' || c.is_whitespace())
    {
        let entry = entry.trim();
        if entry.is_empty() {
            continue;
        }
        let mut parts = entry.split('/');
        let id_s = parts.next().unwrap_or("");
        let Ok(id) = u32::from_str_radix(id_s, 16) else {
            continue;
        };
        let extended = id_s.len() > 3;
        let default_mask = if extended { 0x1FFF_FFFF } else { 0x7FF };
        let mask = parts
            .next()
            .and_then(|m| u32::from_str_radix(m, 16).ok())
            .unwrap_or(default_mask);
        out.push(crate::link::CaptureFilter { id, mask, extended });
    }
    out
}

#[cfg(test)]
mod capture_filter_tests {
    use super::parse_capture_filters;

    #[test]
    fn parses_ids_masks_and_widths() {
        let f = parse_capture_filters(Some("7E8,7A8/7F0 18DAF110,,zz"));
        assert_eq!(f.len(), 3);
        assert_eq!((f[0].id, f[0].mask, f[0].extended), (0x7E8, 0x7FF, false));
        assert_eq!((f[1].id, f[1].mask, f[1].extended), (0x7A8, 0x7F0, false));
        assert_eq!(
            (f[2].id, f[2].mask, f[2].extended),
            (0x18DAF110, 0x1FFF_FFFF, true)
        );
        assert!(parse_capture_filters(None).is_empty());
    }
}
