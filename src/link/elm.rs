//! `ElmHandler` — the ELM327 / STN text dialect behind [`LinkHandler`].
//!
//! Relocated code (LH1): every hardware-won quirk moved here with the code it
//! lived in — the ATE0 warm-start guard, the STPPMC stale-slot purge, the
//! STBC pipe probe, the `SEARCHING`-tolerant 11/29-bit sniff. Wire behaviour
//! is byte-identical to the pre-LH1 engine (gated by `session_manager/golden.rs`).
//!
//! The handler sits ON TOP of the session's `CommandProcessor` (one
//! outstanding command, exact-string correlation) and the session's reply
//! slots the data callback fills (LH7); it does not own a transport of its
//! own — LH0 made every transport a byte pipe into the platform, and the
//! platform stays the sender.
//!
//! LH7: this handler is the ONLY place ELM text is decoded. Every text
//! completion runs through [`decode_wire`] (installed on the platform as its
//! reply decoder) before the engine sees it as a `WireReply` — pipe segments
//! split by wire position, each framed into per-controller payloads.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::stn_periodic_engine::StnPeriodicEngine;
use super::{
    AdapterFacts, AddressingFail, ConnectFail, Frame, HostFacts, LinkEvent, LinkHandler, LinkKind,
    LinkOutcome, LinkReply, PeriodicEngine, ReplyWaiters, WireReply,
};
use crate::addressing::{Addressing, Bus, Controller};
use crate::command_processor::CommandProcessor;
use crate::platform::OBDPlatformInterface;
use crate::session_logger::SessionLogger;

pub struct ElmHandler {
    platform: Arc<dyn OBDPlatformInterface>,
    processor: Arc<CommandProcessor>,
    /// LH7: per-command reply slots the session's data callback fills.
    waiters: Arc<ReplyWaiters>,
    logger: Option<Arc<SessionLogger>>,
    init_commands: Vec<String>,
    pipe_enabled: bool,
    facts: Arc<Mutex<AdapterFacts>>,
    /// Host-fed facts (live cell shared with the session's setters).
    host: Arc<Mutex<HostFacts>>,
    /// The STN periodic engine, built on first use (LH4).
    periodic_engine: std::sync::OnceLock<Arc<StnPeriodicEngine>>,
}

impl ElmHandler {
    pub fn new(
        platform: Arc<dyn OBDPlatformInterface>,
        processor: Arc<CommandProcessor>,
        waiters: Arc<ReplyWaiters>,
        logger: Option<Arc<SessionLogger>>,
        init_commands: Vec<String>,
        pipe_enabled: bool,
        host: Arc<Mutex<HostFacts>>,
    ) -> Arc<Self> {
        let h = Arc::new(Self {
            platform,
            processor,
            waiters,
            logger,
            init_commands,
            pipe_enabled,
            facts: Arc::new(Mutex::new(AdapterFacts::unknown(LinkKind::Elm))),
            host,
            periodic_engine: std::sync::OnceLock::new(),
        });
        // LH7: the platform decodes every text completion through this
        // handler, in the addressing it negotiated (weak — the platform
        // outlives handlers; a rebuilt handler re-installs).
        let weak = Arc::downgrade(&h);
        h.platform
            .set_reply_decoder(Some(Arc::new(move |command: &str, text: &str| {
                let bus = weak
                    .upgrade()
                    .map(|h| Bus::new(h.facts.lock().unwrap().addressing))
                    .unwrap_or_else(|| Bus::new(Addressing::Can11));
                decode_wire(command, text, bus)
            })));
        h
    }

    fn log(&self, key: &str, detail: &str) {
        if let Some(ref log) = self.logger {
            log.log_callback(key, detail);
        }
    }

    /// Queue one command, wait for the processor to drain, take its
    /// outcome. None = could not queue / never went idle and nothing landed.
    fn run(&self, cmd: &str, timeout_ms: u32, idle_wait: Duration) -> Option<LinkOutcome> {
        let slot = self.waiters.arm(cmd);
        if self
            .processor
            .queue_command(cmd.to_string(), timeout_ms)
            .is_err()
        {
            self.waiters.disarm(cmd);
            return None;
        }
        let idle = self.processor.wait_for_idle(idle_wait);
        match slot.wait(Duration::ZERO) {
            Some(o) => Some(o),
            None if idle => Some(Err("TIMEOUT".to_string())),
            None => None,
        }
    }

    /// The adapter acknowledged (`OK` anywhere in the reply text).
    fn acked(outcome: &Option<LinkOutcome>) -> bool {
        matches!(outcome, Some(Ok(r)) if r.raw.to_uppercase().contains("OK"))
    }
}

/// Parse one monitor-mode line into a [`LinkEvent`] (LH3) — the ONE place
/// ELM monitor text is interpreted. `None` = nothing to deliver (blank).
/// Owns the `>`-glued-frame strip (trace 155347: the prompt can arrive glued
/// to a frame, `>6A0 04 …`) and the `STOPPED` fast-ack. A line is a `Frame`
/// when it starts with a CAN id in the bus's shape (11-bit: one 3-hex token,
/// or an 8-hex glued 29-bit id; 29-bit: four 2-hex tokens) followed by hex
/// bytes only; everything else is `Text`.
pub fn parse_monitor_line(line: &str, bus: Bus) -> Option<LinkEvent> {
    let line = line.trim().trim_start_matches('>').trim();
    if line.is_empty() {
        return None;
    }
    if line.to_uppercase().contains("STOPPED") {
        return Some(LinkEvent::Stopped);
    }
    let tokens: Vec<&str> = line.split_whitespace().collect();
    let is_hex = |t: &str| !t.is_empty() && t.chars().all(|c| c.is_ascii_hexdigit());
    let (id, extended, header_len) = match bus.addressing {
        Addressing::Can29 { .. }
            if tokens.len() >= 4 && tokens[..4].iter().all(|t| t.len() == 2 && is_hex(t)) =>
        {
            (tokens[..4].concat().to_uppercase(), true, 4)
        }
        _ if tokens[0].len() == 3 && is_hex(tokens[0]) => (tokens[0].to_uppercase(), false, 1),
        _ if tokens[0].len() == 8 && is_hex(tokens[0]) => (tokens[0].to_uppercase(), true, 1),
        _ => return Some(LinkEvent::Text(line.to_string())),
    };
    let rest = &tokens[header_len..];
    if rest.is_empty() || !rest.iter().all(|t| t.len() == 2 && is_hex(t)) {
        return Some(LinkEvent::Text(line.to_string()));
    }
    let data = rest
        .iter()
        .filter_map(|t| u8::from_str_radix(t, 16).ok())
        .collect();
    Some(LinkEvent::Frame(Frame {
        id,
        extended,
        ts_us: now_us(),
        data,
        raw: line.to_string(),
    }))
}

fn now_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_micros() as u64)
        .unwrap_or(0)
}

/// Put `platform` into monitor mode with every line parsed through
/// [`parse_monitor_line`]. The session's sink uses this directly (it may be
/// handed a platform other than the session's — the canned native platform
/// in tests); `ElmHandler::monitor` is this over the handler's own platform.
pub fn monitor_over(
    platform: &dyn OBDPlatformInterface,
    bus: Bus,
    on: Box<dyn Fn(LinkEvent) + Send + Sync>,
) -> Result<(), String> {
    platform.enter_monitor_mode(
        None,
        Box::new(move |line: String| {
            if let Some(ev) = parse_monitor_line(&line, bus) {
                on(ev);
            }
        }),
    )
}

/// Assemble one chunk's wire payload from its member pids: single member =
/// itself; multi-member = mode prefix once + member suffixes (the grouping
/// never mixes modes in one segment). Shared with stn_periodic's STPPMA
/// `data` — same shape on the poll wire and the adapter-periodic wire.
pub fn join_chunk_wire(members: &[String]) -> String {
    if members.len() == 1 {
        return members[0].clone();
    }
    let mode = members[0][..2].to_string();
    format!(
        "{mode}{}",
        members
            .iter()
            // strip_prefix, NOT trim_start_matches: a DID
            // like 2222 would be eaten by repeated trimming.
            .map(|p| p.strip_prefix(&mode).unwrap_or(p.as_str()))
            .collect::<String>()
    )
}

/// Segments → wire: each segment is a chunk payload, segments join with the
/// STN Batched Commands pipe. A single one-member segment is the bare pid,
/// byte-identical to the legacy serial path (load-bearing degeneracy).
pub fn encode_wire(segments: &[Vec<String>]) -> String {
    segments
        .iter()
        .map(|seg| join_chunk_wire(seg))
        .collect::<Vec<_>>()
        .join("|")
}

/// Split a piped response blob into per-segment strings by pipe POSITION
/// (`STBCOF 0` guarantees output pipes mirror input pipes 1:1). Segments keep
/// their internal `\r` line separators (multi-ECU lines pass through to the
/// per-controller parse); surrounding CR/whitespace is trimmed. `|` cannot
/// appear inside OBD hex output, so a bare split is exact.
pub fn split_pipe_segments(blob: &str) -> Vec<String> {
    blob.split('|')
        .map(|seg| {
            seg.trim_matches(|c: char| c == '\r' || c == '\n' || c == ' ')
                .to_string()
        })
        .collect()
}

/// LH7: one segment of ELM text → the typed reply. `wire` is the segment's
/// wire string (its echo line is dropped by the framing). Status = the
/// first non-noise line's verdict; `faulted` = the whole-reply veto identify
/// used before LH2 (`NO DATA` / any `*ERROR` line, incl. a trailing `CAN
/// ERROR` after data); payloads = the framed controller replies (empty for
/// status tokens / acks).
pub fn decode_segment(wire: &str, text: &str, bus: Bus) -> LinkReply {
    let status = super::classify_text(Some(text));
    let faulted = text.contains("NO DATA") || text.contains("ERROR");
    let payloads = crate::response_parser::text_payloads(wire, text, bus);
    LinkReply::from_payloads(wire, status, faulted, payloads, text.to_string())
}

/// LH7: a whole wire's text → `WireReply`. A piped wire (`010C|010D`) is
/// split by pipe POSITION (`STBCOF 0` mirrors input pipes 1:1) — segment j
/// is decoded for wire segment j; fewer reply segments than wire segments
/// are delivered as-is (the engine aborts the batch). Anything else is one
/// segment for the whole wire.
pub fn decode_wire(command: &str, text: &str, bus: Bus) -> WireReply {
    let segments = if command.contains('|') {
        split_pipe_segments(text)
            .into_iter()
            .zip(command.split('|'))
            .map(|(seg, wire)| decode_segment(wire, &seg, bus))
            .collect()
    } else {
        vec![decode_segment(command, text, bus)]
    };
    WireReply {
        raw: text.to_string(),
        segments,
    }
}

impl LinkHandler for ElmHandler {
    fn kind(&self) -> LinkKind {
        LinkKind::Elm
    }

    fn connect(&self) -> Result<AdapterFacts, ConnectFail> {
        // Capability resets FIRST so a reconnect on a different adapter can
        // never inherit the last one's pipes.
        self.facts.lock().unwrap().pipe_capable = false;

        if self.init_commands.is_empty() {
            return Ok(self.facts()); // Nothing to send — skip init
        }

        // Arm the ATE0 slot BEFORE the list goes out so the verification
        // below can only see THIS init's response.
        let ate0_slot = self
            .init_commands
            .iter()
            .any(|c| c.eq_ignore_ascii_case("ATE0"))
            .then(|| self.waiters.arm("ATE0"));

        for cmd in &self.init_commands {
            if let Err(e) = self.processor.queue_command(cmd.clone(), 5000) {
                self.waiters.disarm("ATE0");
                eprintln!("Failed to queue init command {}: {}", cmd, e);
                return Err(ConnectFail::QueueRefused(cmd.clone()));
            }
        }

        // Wait for all init commands to complete (10s per command, generous)
        let timeout = Duration::from_secs((self.init_commands.len() as u64) * 10);
        if !self.processor.wait_for_idle(timeout) {
            return Err(ConnectFail::InitTimeout);
        }

        // Warm-start race guard (seen on the bench 2026-07-30): the ATWS the
        // UI sends before init resets the adapter, and an ATE0 arriving
        // while it's still rebooting is SWALLOWED — the late reset banner
        // gets correlated as ATE0's response, echo stays ON, and ATDPN's
        // echo-prefixed reply later fails protocol detection ("unsupported
        // protocol" on a healthy car). Verify ATE0 really answered OK and
        // resend it once if not.
        let ate0 = ate0_slot.map(|slot| slot.wait(Duration::ZERO)).flatten();
        if self
            .init_commands
            .iter()
            .any(|c| c.eq_ignore_ascii_case("ATE0"))
            && !Self::acked(&ate0)
        {
            let seen = match &ate0 {
                Some(Ok(r)) => r.raw.clone(),
                Some(Err(e)) => e.clone(),
                None => String::new(),
            };
            eprintln!(
                "Init: ATE0 did not answer OK (got {:?}) — retrying once (warm-start race)",
                seen
            );
            self.log("init_retry_ate0", &seen);
            let retry = self.run("ATE0", 5000, Duration::from_secs(10));
            if retry.is_none() {
                return Err(ConnectFail::InitTimeout);
            }
            if !Self::acked(&retry) {
                // Echo may still be on; identify can misread ATDPN.
                // Log loudly but keep the previous behavior (connect on).
                eprintln!("Init: ATE0 retry still did not answer OK — echo may be enabled");
            }
        }

        // STALE-SLOT PURGE (bench 2026-08-03, CX): a periodic slot leaked
        // by a prior session (STPPMD timeout, app kill — and ATWS is not
        // proven to clear the periodic table) leaves the adapter spamming
        // the bus forever: ST commands crawl (~2 s each), engages take
        // 15 s, streams arrive dead, the BLE link eventually drops. One
        // idempotent STPPMC at every connect guarantees a clean table;
        // non-STN adapters answer "?" and move on.
        let _ = self.run("STPPMC", 5000, Duration::from_secs(10));

        // BP1: probe OBDLink Batched Commands. The setting is VOLATILE
        // (doesn't survive the ATWS the UI sends before init), so this
        // runs every connect — and it IS the capability probe: OK =
        // pipe-capable (CX/MX+), "?" = ELM clone (Dragy) → stay serial.
        if self.pipe_enabled {
            let probe_ok = Self::acked(&self.run("STBC 1", 5000, Duration::from_secs(10)));
            let mut capable = false;
            if probe_ok {
                // Verbose output format: output pipes mirror input pipes
                // 1:1 — position demux depends on it.
                capable = Self::acked(&self.run("STBCOF 0", 5000, Duration::from_secs(10)));
            }
            self.facts.lock().unwrap().pipe_capable = capable;
            eprintln!(
                "Init: pipe batching {}",
                if capable {
                    "ENABLED (STBC 1 OK)"
                } else {
                    "off (probe declined)"
                }
            );
            self.log(
                "pipe_probe",
                if capable { "capable" } else { "unsupported" },
            );
        }
        Ok(self.facts())
    }

    /// Primary: sniff a functional `0100` response header (learns the tester
    /// too). Fallback: `ATDPN`.
    fn detect_addressing(&self) -> Result<(Addressing, Option<Controller>), AddressingFail> {
        let raw0100 = self.request_raw("0100", 5000);
        let detected = raw0100.as_ref().and_then(|raw| {
            raw.replace('\r', "\n")
                .lines()
                .map(str::trim)
                .find(|l| !l.is_empty() && !l.starts_with("SEARCHING"))
                .and_then(|line| {
                    let tokens: Vec<&str> = line.split_whitespace().collect();
                    let a = Addressing::sniff(&tokens)?;
                    // SP29: the first 0100 responder IS the engine — learn it
                    // live in the just-detected protocol. Wire truth beats any
                    // cached roster: a VIN cached under 11-bit carries ECU
                    // keys meaningless on a 29-bit connect (log 230220: stale
                    // key 00 → STPPMA 18DA00F1 → periodics into the void
                    // while polling worked fine).
                    let primary = Bus::new(a).read(&tokens).map(|(c, _)| c);
                    Some((a, primary))
                })
        });

        let detected = detected.or_else(|| {
            self.request_raw("ATDPN", 2000)
                .and_then(|n| Addressing::from_atdpn(&n))
                .map(|a| (a, None))
        });

        if let Some((a, primary)) = detected {
            let mut facts = self.facts.lock().unwrap();
            facts.addressing = a;
            if primary.is_some() {
                facts.primary_ecu = primary;
            }
            return Ok((a, primary));
        }

        // RB1: distinguish "bus wasn't answering" (retryable — ignition off /
        // flaky adapter: 0100 came back UNABLE TO CONNECT / NO DATA / all
        // SEARCHING / empty) from a genuine non-CAN vehicle (a real but
        // non-CAN response). The former must NOT read as "unsupported".
        if super::classify_text(raw0100.as_deref()).is_no_response() {
            Err(AddressingFail::NoResponse)
        } else {
            Err(AddressingFail::NonCan)
        }
    }

    fn facts(&self) -> AdapterFacts {
        let mut f = self.facts.lock().unwrap().clone();
        let host = self.host.lock().unwrap();
        f.periodic_capable = host.periodic_capable;
        f.max_chunk = host.max_chunk;
        f.tier_periods = host.tier_periods;
        f
    }

    /// Connect-thread exchange: queue, drain the processor, take the typed
    /// outcome (an error outcome — TIMEOUT etc. — is None, as the pre-LH7
    /// capture map never held one).
    fn exchange(&self, command: &str, timeout_ms: u32) -> Option<WireReply> {
        self.run(
            command,
            timeout_ms,
            Duration::from_millis(timeout_ms as u64 + 2000),
        )?
        .ok()
    }

    fn encode(&self, segments: &[Vec<String>]) -> String {
        encode_wire(segments)
    }

    fn raw_write(&self, line: &str) {
        self.platform.write_raw(line);
    }

    fn monitor(&self, on: Box<dyn Fn(LinkEvent) + Send + Sync>) -> Result<(), String> {
        let bus = Bus::new(self.facts.lock().unwrap().addressing);
        monitor_over(self.platform.as_ref(), bus, on)
    }

    fn stop_monitor(&self) {
        self.platform.exit_monitor_mode();
    }

    /// The STPPMA engine — only on adapters the host marked periodic-capable
    /// (STN chips); ELM clones get none.
    fn periodic(&self) -> Option<Arc<dyn PeriodicEngine>> {
        if !self.host.lock().unwrap().periodic_capable {
            return None;
        }
        let engine = self.periodic_engine.get_or_init(|| {
            Arc::new(StnPeriodicEngine::new(
                Arc::clone(&self.platform),
                Arc::clone(&self.processor),
                Arc::clone(&self.waiters),
                self.logger.clone(),
                Arc::clone(&self.facts),
            ))
        });
        Some(Arc::clone(engine) as Arc<dyn PeriodicEngine>)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_is_bare_pid_for_single_member() {
        assert_eq!(encode_wire(&[vec!["010C".into()]]), "010C");
    }

    #[test]
    fn encode_joins_chunks_and_pipes() {
        assert_eq!(
            encode_wire(&[vec!["010C".into(), "010D".into()], vec!["0105".into()]]),
            "010C0D|0105"
        );
        assert_eq!(
            join_chunk_wire(&["222222".into(), "222223".into()]),
            "2222222223"
        );
    }

    #[test]
    fn monitor_line_parse_table() {
        use crate::addressing::Addressing;
        let b11 = Bus::new(Addressing::Can11);
        let b29 = Bus::new(Addressing::Can29 { tester: 0xF1 });
        let frame = |ev: Option<LinkEvent>| match ev {
            Some(LinkEvent::Frame(f)) => f,
            other => panic!("{other:?}"),
        };
        let f = frame(parse_monitor_line("6A0 00 0B 4A 4F", b11));
        assert_eq!(
            (f.id.as_str(), f.extended, f.data.as_slice(), f.raw.as_str()),
            (
                "6A0",
                false,
                &[0x00, 0x0B, 0x4A, 0x4F][..],
                "6A0 00 0B 4A 4F"
            )
        );
        // Glued prompt: stripped from id AND raw.
        let f = frame(parse_monitor_line(">6A0 04 11 22 33 44", b11));
        assert_eq!(
            (f.id.as_str(), f.raw.as_str()),
            ("6A0", "6A0 04 11 22 33 44")
        );
        // 29-bit: four spaced tokens → joined id; raw keeps the four tokens.
        let f = frame(parse_monitor_line("18 DA F1 10 06 41 00 BF FE B9 93", b29));
        assert_eq!(
            (f.id.as_str(), f.extended, f.data.len(), f.raw.as_str()),
            ("18DAF110", true, 7, "18 DA F1 10 06 41 00 BF FE B9 93")
        );
        // Glued 8-hex 29-bit id (mock shape) is a frame on either bus.
        assert_eq!(
            frame(parse_monitor_line("18DAF110 06 41 00 BF FE B9 93", b11)).id,
            "18DAF110"
        );
        assert_eq!(parse_monitor_line("STOPPED", b11), Some(LinkEvent::Stopped));
        assert_eq!(
            parse_monitor_line(">stopped", b11),
            Some(LinkEvent::Stopped)
        );
        assert_eq!(
            parse_monitor_line("OK", b11),
            Some(LinkEvent::Text("OK".into()))
        );
        assert_eq!(
            parse_monitor_line("BUFFER FULL", b11),
            Some(LinkEvent::Text("BUFFER FULL".into()))
        );
        assert_eq!(
            parse_monitor_line("SEARCHING...", b11),
            Some(LinkEvent::Text("SEARCHING...".into()))
        );
        // Header with no bytes, or non-hex bytes → text, not a frame.
        assert_eq!(
            parse_monitor_line("7E8", b11),
            Some(LinkEvent::Text("7E8".into()))
        );
        assert_eq!(
            parse_monitor_line("7E8 0G", b11),
            Some(LinkEvent::Text("7E8 0G".into()))
        );
        assert_eq!(parse_monitor_line("", b11), None);
        assert_eq!(parse_monitor_line(">", b11), None);
        assert_eq!(parse_monitor_line("   ", b11), None);
    }

    /// SP on 29-bit: the frame's `raw` keeps the four-token header, so the
    /// text-in multi-pid splitter (the SP decoder) still attributes it.
    #[test]
    fn sp_29bit_frame_raw_reframes_through_splitter() {
        use crate::addressing::Addressing;
        let b29 = Bus::new(Addressing::Can29 { tester: 0xF1 });
        let Some(LinkEvent::Frame(f)) = parse_monitor_line("18 DA F1 10 06 41 0C 1A F8 0D 45", b29)
        else {
            panic!()
        };
        assert_eq!(f.id, "18DAF110");
        let members = vec![("010C".to_string(), 2u8), ("010D".to_string(), 1u8)];
        let parsed = crate::response_parser::split_multi_pid_response(&members, &f.raw, b29)
            .expect("splits");
        assert_eq!(
            parsed["010C"].all_controllers["10"].data_bytes,
            vec![0x1A, 0xF8]
        );
        assert_eq!(parsed["010D"].all_controllers["10"].data_bytes, vec![0x45]);
    }

    #[test]
    fn split_is_positional_and_trims() {
        assert_eq!(
            split_pipe_segments("7E8 04 41 0C 3F F3 \r\r|7E8 03 41 0D 73"),
            vec!["7E8 04 41 0C 3F F3", "7E8 03 41 0D 73"]
        );
    }
}
