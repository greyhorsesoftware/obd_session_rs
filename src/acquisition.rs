//! Acquisition plugins (High_Rate_Data_Plan P0 + BP1).
//!
//! The subscription engine owns scheduling, due computation, and completion
//! fan-out; a plugin ONLY translates due picks into wire bytes and back.
//! Exactly one plugin owns the persistent subscription set at a time; `poll`
//! additionally serves all run-once traffic regardless of the active owner.
//!
//! `poll` carries two orthogonal policy axes — never new plugins:
//! - **pipe** (per-ADAPTER, BP1): OBDLink Batched Commands — join several
//!   due picks into one prompt exchange (`010C|010D|0105`). Probed via
//!   `STBC 1` at init; 1 (off) on clones like the Dragy.
//! - **chunk** (per-VEHICLE, B1, future): J1979 multi-PID requests.
//!
//! At (chunk = 1, pipe = 1) the emitted wire is byte-identical to the legacy
//! serial path — proven by the golden tests below and the unchanged suite.

/// One due pick from the engine's scheduler.
#[derive(Debug, Clone, PartialEq)]
pub struct DuePick {
    pub pid: String,
    pub controller: Option<String>,
    pub timeout_ms: Option<u32>,
}

/// One wire transaction with explicit member accounting.
///
/// `segments` mirror the wire's pipe structure: segment j of the response
/// belongs to `segments[j]`. A 1-member segment is an ordinary single-PID
/// request; a multi-member segment is a J1979 chunk (B1) whose response is
/// sliced by the echo-driven splitter. The completion path accounts every
/// member and NEVER string-parses `wire` to recover them (common-ground #4:
/// the `STFPA 7E8,7FF` bug family).
#[derive(Debug, Clone, PartialEq)]
pub struct NextCommand {
    /// Exact string to put on the wire.
    pub wire: String,
    /// Member PIDs per pipe segment, in wire order.
    pub segments: Vec<Vec<String>>,
    /// Optional target controller (resolved to an ATSH header by the engine).
    pub target: Option<String>,
    /// Optional per-command timeout override.
    pub timeout_ms: Option<u32>,
}

impl NextCommand {
    /// All member PIDs, flattened in wire order.
    pub fn members(&self) -> Vec<String> {
        self.segments.iter().flatten().cloned().collect()
    }
}

/// Shared services injected into plugins — plugins own no global state.
/// First resident: the per-VIN learned length cache (common-ground #1),
/// which B1's batch builder consults for chunk admission and Tier S's
/// `build_defines` for `F2xx` packing.
pub struct PluginCtx {
    pub length_cache: std::sync::Arc<std::sync::Mutex<crate::length_cache::LengthCache>>,
}

impl PluginCtx {
    /// Confirmed data-byte length for a PID on the current vehicle. `None`
    /// means unconfirmed → the PID rides solo (observe-then-batch).
    pub fn pid_len(&self, pid: &str) -> Option<u8> {
        self.length_cache.lock().unwrap().get(pid)
    }
}

/// A strategy for getting subscription data on and off the wire.
///
/// Compile-time trait objects only — nothing user-installable sits on the
/// wire path. Further capabilities (probe/start/ingest/stop for the
/// `uds-stream` plugin) join this trait when Tier S lands.
pub trait AcquisitionPlugin: Send {
    /// Stable identifier ("poll" | "uds-stream") — surfaced by the
    /// acquisition getter and the session log.
    fn id(&self) -> &'static str;

    /// How many due picks the engine should gather for the next exchange
    /// (an upper bound — [`Self::admission`] does the per-pick gating).
    fn max_picks(&self) -> usize {
        1
    }

    /// Adapter pipe capability (the `STBC 1` init probe result). Default
    /// no-op — only `poll` cares.
    fn set_pipe_capable(&mut self, _capable: bool) {}

    /// Vehicle J1979 multi-PID capability (B2's post-identify probe).
    /// Default no-op — only `poll` cares.
    fn set_chunk_capable(&mut self, _capable: bool) {}

    /// B2 member demotion: this vehicle silently omits `pid` from chunk
    /// responses — route it solo from now on (set dies with the capability
    /// reset). Default no-op — only `poll` cares.
    fn exclude_from_chunks(&mut self, _pid: &str) {}

    /// B2 adapter demotion: a chunk wire failed at the TRANSPORT level
    /// (timeout / dropped prompt — a clone adapter choking on a request the
    /// vehicle itself accepts). Halve the chunk size (BOTH modes — an adapter
    /// that chokes on long wires chokes regardless of mode) and retry
    /// smaller; returns the new Mode 01 limit for logging. Default no-op —
    /// only `poll` cares.
    fn clamp_chunk(&mut self) -> usize {
        1
    }

    /// B3: enable/disable Mode 22 multi-DID chunking for this connect.
    /// `true` arms the opportunistic probe (first composed wire IS the
    /// probe); `false` (config off / adapter-capped) forces solos. Default
    /// no-op — only `poll` cares.
    fn set_mode22_enabled(&mut self, _enabled: bool) {}

    /// B3 outcome feedback from the demux: the first composed PAIR decides
    /// the capability (the cap is 2 — Mustang refuses 3, see
    /// MODE22_CHUNK_MAX_DIDS). NO DATA on a multi-DID wire counts as a
    /// failure (the members answer solo, so silence means the SHAPE was
    /// rejected). `size` is the wire's member count, for logging. Returns
    /// true iff the state CHANGED. Default no-op — only `poll` cares.
    fn note_mode22_chunk_outcome(&mut self, _ok: bool, _size: usize) -> bool {
        false
    }

    /// Per-ADAPTER starting cap from the app's adapter registry
    /// (adapters.json `maxChunk`) — applied after the vehicle probe enables
    /// chunking, so a known-fragile clone starts at a size it can handle
    /// instead of paying clamp timeouts. Default no-op — only `poll` cares.
    fn set_chunk_cap(&mut self, _cap: usize) {}

    /// Pick-time admission policy for the engine's multi-pick loop —
    /// a self-contained value so the engine never re-enters the plugin
    /// (lock ordering) while picking.
    fn admission(&self) -> Admission {
        Admission::single()
    }

    /// Translate due picks (all sharing one target, all admitted by
    /// [`Self::admission`]) into a wire transaction. `None` iff `due` is empty.
    fn next_command(&mut self, due: &[DuePick]) -> Option<NextCommand>;
}

/// Max PIDs per J1979 chunk (SAE J1979 allows up to 6 per request).
pub const CHUNK_MAX_PIDS: usize = 6;

/// Max DIDs per Mode 22 multi-DID request (B3). Hardware truth (Mustang
/// bench 2026-07-31): the S197 PCM answers 2-DID requests cleanly but
/// REFUSES 3 (NO DATA → `7F 22 31`) — so the cap is the proven pair, per
/// user direction. A pair still halves the Mode 22 wire count; revisit only
/// if some platform is proven to take more.
pub const MODE22_CHUNK_MAX_DIDS: usize = 2;

/// B3 Mode 22 multi-DID capability — decided by the first composed pair
/// (opportunistic probing; there is no universal DID pair to ask every
/// vehicle). The size IS the cap (2) — no ladder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mode22ChunkState {
    /// Off until the session arms it post-identify (or config/adapter says no).
    Disabled,
    /// Armed: the next composed pair is the probe.
    Untried,
    /// Pair proven — keep pairing.
    Capable,
    /// Pair refused — solos for the rest of the connect.
    Refused,
}

/// A plain Mode 01 DATA pid — the only shape admitted to a chunk. Excludes
/// support queries (00/20/40/60), monitor status (01), and anything that
/// isn't exactly `01XX`.
pub fn is_plain_mode01_data_pid(pid: &str) -> bool {
    let p = pid.trim().to_uppercase();
    p.len() == 4
        && p.starts_with("01")
        && u8::from_str_radix(&p[2..], 16).is_ok()
        && !matches!(&p[2..], "00" | "20" | "40" | "60" | "01")
}

/// A plain Mode 22 DATA pid — the only shape admitted to a Mode 22 chunk
/// (B3). Excludes the `F2xx` dynamic-DID range (owned by Tier S streaming
/// defines) and DIDs with a `00` high byte (indistinguishable from ISO-TP
/// padding in the echo scan).
pub fn is_plain_mode22_data_pid(pid: &str) -> bool {
    let p = pid.trim().to_uppercase();
    p.len() == 6
        && p.starts_with("22")
        && u16::from_str_radix(&p[2..], 16).is_ok()
        && !p[2..].starts_with("F2")
        && !p[2..].starts_with("00")
}

/// Which chunk a pid may join (B1 Mode 01 / B3 Mode 22) — modes NEVER mix
/// in one wire.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChunkClass {
    Mode01,
    Mode22,
}

/// Pick-time admission policy (extracted from the active plugin so the
/// engine's pick loop holds no plugin lock). A candidate is admitted iff the
/// greedy segment grouping of (already-picked + candidate) still fits
/// `max_segments` — this one rule covers serial (1×1), pipes (N×1), chunks
/// (1×6), and their composition.
pub struct Admission {
    /// Pipe segments available per exchange (1 = no pipes).
    pub max_segments: usize,
    /// PIDs per J1979 chunk segment (1 = no chunking).
    pub chunk_limit: usize,
    /// DIDs per Mode 22 multi-DID segment (1 = no Mode 22 chunking; 2 while
    /// the opportunistic probe is untried; MODE22_CHUNK_MAX_DIDS once
    /// capable) — B3.
    pub chunk22_limit: usize,
    /// Learned lengths — the standing admission rule: only length-confirmed
    /// members join a chunk (observe-then-batch; also how demotion routes
    /// invalidated members solo).
    pub length_cache: Option<std::sync::Arc<std::sync::Mutex<crate::length_cache::LengthCache>>>,
    /// B2 member demotion: pids this VEHICLE silently omits from chunk
    /// responses. An omitted member would starve (re-chunked → omitted →
    /// no data, forever) — it solos instead; the rest of the set survives.
    /// Snapshot of the plugin's per-connect set; cleared on capability reset.
    pub chunk_excluded: std::collections::HashSet<String>,
}

impl Admission {
    pub fn single() -> Self {
        Self {
            max_segments: 1,
            chunk_limit: 1,
            chunk22_limit: 1,
            length_cache: None,
            chunk_excluded: std::collections::HashSet::new(),
        }
    }

    /// Is this pid admissible INTO A CHUNK (vs riding its own segment)?
    /// Public for the Session Stats projection — the model must count
    /// chunkable pids exactly the way the wire will group them.
    pub fn is_chunk_member(&self, pid: &str) -> bool {
        self.chunk_class(pid).is_some()
    }

    /// Which chunk (if any) this pid may join. The standing rules apply to
    /// both modes: length-confirmed only (observe-then-batch) and not on the
    /// vehicle's omission list; the mode limits gate independently (a car
    /// can chunk Mode 01 yet refuse multi-DID Mode 22 and vice versa).
    pub fn chunk_class(&self, pid: &str) -> Option<ChunkClass> {
        let class = if self.chunk_limit > 1 && is_plain_mode01_data_pid(pid) {
            ChunkClass::Mode01
        } else if self.chunk22_limit > 1 && is_plain_mode22_data_pid(pid) {
            ChunkClass::Mode22
        } else {
            return None;
        };
        let admitted = !self.chunk_excluded.contains(&pid.trim().to_uppercase())
            && self
                .length_cache
                .as_ref()
                .is_some_and(|c| c.lock().unwrap().get(pid).is_some());
        admitted.then_some(class)
    }

    fn class_limit(&self, class: ChunkClass) -> usize {
        match class {
            ChunkClass::Mode01 => self.chunk_limit,
            ChunkClass::Mode22 => self.chunk22_limit,
        }
    }

    /// Greedy segment grouping of a pick list under this policy: chunkable
    /// picks merge into per-mode chunks of ≤ that mode's limit (Mode 22
    /// additionally per target controller — a multi-DID request goes to ONE
    /// ECU); everything else rides its own segment. Deterministic: chunks
    /// first (fill order), then solos.
    pub fn group<'a>(&self, due: &'a [DuePick]) -> Vec<Vec<&'a DuePick>> {
        let mut chunks: Vec<(ChunkClass, Option<&'a String>, Vec<&'a DuePick>)> = Vec::new();
        let mut solos: Vec<&'a DuePick> = Vec::new();
        for p in due {
            match self.chunk_class(&p.pid) {
                Some(class) => {
                    let ctrl = match class {
                        ChunkClass::Mode01 => None, // functional — target-free
                        ChunkClass::Mode22 => p.controller.as_ref(),
                    };
                    match chunks.iter_mut().find(|(c, t, m)| {
                        *c == class && *t == ctrl && m.len() < self.class_limit(class)
                    }) {
                        Some((_, _, members)) => members.push(p),
                        None => chunks.push((class, ctrl, vec![p])),
                    }
                }
                None => solos.push(p),
            }
        }
        let mut out: Vec<Vec<&'a DuePick>> = chunks.into_iter().map(|(_, _, m)| m).collect();
        out.extend(solos.into_iter().map(|p| vec![p]));
        out
    }

    /// Would adding `candidate` to `already` still fit the exchange?
    pub fn admit(&self, already: &[DuePick], candidate: &DuePick) -> bool {
        let mut all = already.to_vec();
        all.push(candidate.clone());
        self.group(&all).len() <= self.max_segments
    }
}

/// Max segments joined into one piped exchange. The 1024-char adapter buffer
/// allows far more; 12 keeps exchanges ≤ ~450 ms (measured 89 + 30/segment)
/// so one-shots and pause aren't starved behind a mega-sweep.
pub const PIPE_MAX_SEGMENTS: usize = 12;

/// Measured wire model: one prompt round-trip plus each piped segment after
/// the first. RTT is a MIDDLE-OF-ROAD figure chosen 2026-07-31: the CX+Malibu
/// benched ~89 ms flat (single-frame replies), the Mustang session averaged
/// ~211 ms (multi-frame replies = more BT rounds + the adapter's
/// end-of-response silence wait) — 149 splits the difference so projections
/// aren't wildly optimistic on real cars. Used by the Session Stats
/// projection and the quality-floor gates — model, not measurement.
pub const MEASURED_RTT_MS: f64 = 149.0;
pub const MEASURED_PIPE_SEGMENT_MS: f64 = 30.0;

/// Request/response acquisition — today's serial path plus the two policy
/// axes: pipe (per-adapter, BP1) and chunk (per-vehicle, B1).
pub struct PollPlugin {
    /// Segments per exchange: 1 until the `STBC 1` probe confirms support.
    pipe_limit: usize,
    /// PIDs per J1979 chunk: 1 until B2's vehicle probe confirms support.
    chunk_limit: usize,
    /// Learned lengths (shared with the session) — chunk admission input.
    length_cache: std::sync::Arc<std::sync::Mutex<crate::length_cache::LengthCache>>,
    /// Pids this vehicle silently omits from chunk responses (B2 member
    /// demotion) — they solo; cleared with the capability reset.
    chunk_excluded: std::collections::HashSet<String>,
    /// Mode 22 multi-DID capability state (B3, opportunistic probe).
    chunk22: Mode22ChunkState,
    /// Adapter starting cap applied to Mode 22 chunks too (adapters.json).
    chunk22_cap: usize,
}

impl PollPlugin {
    pub fn new(ctx: PluginCtx) -> Self {
        Self {
            pipe_limit: 1,
            chunk_limit: 1,
            length_cache: ctx.length_cache,
            chunk_excluded: std::collections::HashSet::new(),
            chunk22: Mode22ChunkState::Disabled,
            chunk22_cap: MODE22_CHUNK_MAX_DIDS,
        }
    }

    fn chunk22_limit(&self) -> usize {
        let limit = match self.chunk22 {
            Mode22ChunkState::Disabled | Mode22ChunkState::Refused => 1,
            Mode22ChunkState::Untried | Mode22ChunkState::Capable => MODE22_CHUNK_MAX_DIDS,
        };
        limit.min(self.chunk22_cap.max(1))
    }
}

impl AcquisitionPlugin for PollPlugin {
    fn id(&self) -> &'static str {
        "poll"
    }

    fn max_picks(&self) -> usize {
        // Upper bound on members per exchange: segments × the biggest chunk
        // either mode can fill (admission does the real per-pick gating).
        self.pipe_limit * self.chunk_limit.max(self.chunk22_limit())
    }

    fn set_pipe_capable(&mut self, capable: bool) {
        self.pipe_limit = if capable { PIPE_MAX_SEGMENTS } else { 1 };
    }

    fn set_chunk_capable(&mut self, capable: bool) {
        self.chunk_limit = if capable { CHUNK_MAX_PIDS } else { 1 };
        // Capability reset = new vehicle/connect — omission memory dies too,
        // and Mode 22 goes back to Disabled until the session re-arms it.
        if !capable {
            self.chunk_excluded.clear();
            self.chunk22 = Mode22ChunkState::Disabled;
            self.chunk22_cap = MODE22_CHUNK_MAX_DIDS;
        }
    }

    fn exclude_from_chunks(&mut self, pid: &str) {
        self.chunk_excluded.insert(pid.trim().to_uppercase());
    }

    fn clamp_chunk(&mut self) -> usize {
        // 6 → 3 → 1: each failed size costs one command timeout, then the
        // sweep rebuilds smaller. At 1 chunking is effectively off for the
        // rest of the connection (capability reset restores 1 anyway).
        // Mode 22 shares the clamp — a transport-level choke is about wire
        // length, not mode.
        self.chunk_limit = (self.chunk_limit / 2).max(1);
        self.chunk22_cap = (self.chunk22_cap / 2).max(1);
        self.chunk_limit
    }

    fn set_chunk_cap(&mut self, cap: usize) {
        self.chunk_limit = self.chunk_limit.min(cap.max(1));
        self.chunk22_cap = self.chunk22_cap.min(cap.max(1));
    }

    fn set_mode22_enabled(&mut self, enabled: bool) {
        self.chunk22 = if enabled {
            Mode22ChunkState::Untried
        } else {
            Mode22ChunkState::Disabled
        };
    }

    fn note_mode22_chunk_outcome(&mut self, ok: bool, _size: usize) -> bool {
        let next = if ok {
            Mode22ChunkState::Capable
        } else {
            Mode22ChunkState::Refused
        };
        let changed = self.chunk22 != next;
        self.chunk22 = next;
        changed
    }

    fn admission(&self) -> Admission {
        Admission {
            max_segments: self.pipe_limit,
            chunk_limit: self.chunk_limit,
            chunk22_limit: self.chunk22_limit(),
            length_cache: Some(std::sync::Arc::clone(&self.length_cache)),
            chunk_excluded: self.chunk_excluded.clone(),
        }
    }

    fn next_command(&mut self, due: &[DuePick]) -> Option<NextCommand> {
        if due.is_empty() {
            return None;
        }
        // Single pick, no chunking possible: byte-identical pass-through
        // (the legacy serial path — load-bearing degeneracy).
        if due.len() == 1 {
            return Some(NextCommand {
                wire: due[0].pid.clone(),
                segments: vec![vec![due[0].pid.clone()]],
                target: due[0].controller.clone(),
                timeout_ms: due[0].timeout_ms,
            });
        }
        // Group under the same policy that admitted these picks: chunkable
        // picks merge into `01`+suffix chunks, everything else rides solo;
        // multiple segments join with `|` (admission guarantees they fit —
        // and that >1 segment only happens on a pipe-capable adapter).
        let admission = self.admission();
        let groups = admission.group(due);
        let segments: Vec<Vec<String>> = groups
            .iter()
            .map(|g| {
                let mut seg: Vec<String> = g.iter().map(|p| p.pid.clone()).collect();
                // Chunk member order is semantically free (the splitter is
                // echo-keyed) — sort for DETERMINISTIC wires: stable logs,
                // stable mock keys, and a repeated sweep re-arms the STN
                // batch-replay buffer instead of looking like a new batch.
                if seg.len() > 1 {
                    seg.sort();
                }
                seg
            })
            .collect();
        // LH1: the wire encoding (chunk payload + STN pipes) is the ELM
        // handler's; the plugin's contract is `segments`.
        let wire = crate::link::elm::encode_wire(&segments);
        Some(NextCommand {
            wire,
            segments,
            target: due[0].controller.clone(),
            timeout_ms: due.iter().filter_map(|p| p.timeout_ms).max(),
        })
    }
}

/// LH1: wire encoding moved to the ELM handler; re-exported for the
/// STPPMA builder (same chunk shape on the adapter-periodic wire).
pub(crate) use crate::link::elm::join_chunk_wire;
pub use crate::link::elm::split_pipe_segments;

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    fn pick(pid: &str) -> DuePick {
        DuePick {
            pid: pid.to_string(),
            controller: None,
            timeout_ms: None,
        }
    }

    fn test_plugin() -> (PollPlugin, Arc<Mutex<crate::length_cache::LengthCache>>) {
        let cache = Arc::new(Mutex::new(crate::length_cache::LengthCache::new()));
        let plugin = PollPlugin::new(PluginCtx {
            length_cache: Arc::clone(&cache),
        });
        (plugin, cache)
    }

    /// Golden serial equality: at pipe = 1 the plugin's wire output is
    /// byte-identical to the pid the engine picked — for every command shape
    /// the engine emits today.
    #[test]
    fn poll_plugin_wire_is_byte_identical_to_pid() {
        let (mut plugin, _cache) = test_plugin();
        for pid in [
            "010C",
            "221E35",
            "0902",
            "ATSP 0",
            "STFPA 7E8,7FF",
            "010C 1",
            "010C|010D|0105",
            "010C0D050F1104",
        ] {
            let cmd = plugin.next_command(&[pick(pid)]).unwrap();
            assert_eq!(cmd.wire, pid);
            assert_eq!(cmd.members(), vec![pid.to_string()]);
            assert_eq!(cmd.target, None);
            assert_eq!(cmd.timeout_ms, None);
        }
        assert!(plugin.next_command(&[]).is_none());
    }

    /// Target and timeout pass through untouched on the single path.
    #[test]
    fn poll_plugin_passes_target_and_timeout() {
        let (mut plugin, _cache) = test_plugin();
        let cmd = plugin
            .next_command(&[DuePick {
                pid: "22F40C".to_string(),
                controller: Some("7E0".to_string()),
                timeout_ms: Some(2500),
            }])
            .unwrap();
        assert_eq!(cmd.wire, "22F40C");
        assert_eq!(cmd.target.as_deref(), Some("7E0"));
        assert_eq!(cmd.timeout_ms, Some(2500));
    }

    /// Pipe join: members in wire order, shared target, max timeout override.
    #[test]
    fn poll_plugin_joins_piped_exchange() {
        let (mut plugin, _cache) = test_plugin();
        plugin.set_pipe_capable(true);
        assert_eq!(plugin.max_picks(), PIPE_MAX_SEGMENTS);

        let due = vec![
            DuePick {
                pid: "010C".into(),
                controller: Some("7E0".into()),
                timeout_ms: Some(1000),
            },
            DuePick {
                pid: "010D".into(),
                controller: Some("7E0".into()),
                timeout_ms: None,
            },
            DuePick {
                pid: "0105".into(),
                controller: Some("7E0".into()),
                timeout_ms: Some(3000),
            },
        ];
        let cmd = plugin.next_command(&due).unwrap();
        assert_eq!(cmd.wire, "010C|010D|0105");
        assert_eq!(cmd.members(), vec!["010C", "010D", "0105"]);
        assert_eq!(cmd.segments.len(), 3, "no chunking without capability");
        assert_eq!(cmd.target.as_deref(), Some("7E0"));
        assert_eq!(cmd.timeout_ms, Some(3000));
    }

    /// Capability off (probe said "?") → max_picks 1 → never joins.
    #[test]
    fn poll_plugin_pipe_off_stays_serial() {
        let (mut plugin, _cache) = test_plugin();
        assert_eq!(plugin.max_picks(), 1);
        plugin.set_pipe_capable(true);
        plugin.set_pipe_capable(false); // demotion (e.g. reconnect on a clone)
        assert_eq!(plugin.max_picks(), 1);
    }

    #[test]
    fn poll_plugin_id() {
        assert_eq!(test_plugin().0.id(), "poll");
    }

    // ---- B1: chunk axis ----

    /// Chunk on, pipes off (the Dragy shape): length-confirmed Mode 01 picks
    /// merge into ONE `01`-multi wire; members ride in wire order.
    #[test]
    fn poll_plugin_chunks_confirmed_mode01() {
        let (mut plugin, cache) = test_plugin();
        plugin.set_chunk_capable(true);
        assert_eq!(plugin.max_picks(), CHUNK_MAX_PIDS);
        for (pid, len) in [("010C", 2), ("010D", 1), ("0105", 1)] {
            cache.lock().unwrap().record(pid, len);
        }
        let due = vec![pick("010C"), pick("010D"), pick("0105")];
        let cmd = plugin.next_command(&due).unwrap();
        // Chunk members are sorted → deterministic wire regardless of pick order.
        assert_eq!(cmd.wire, "01050C0D");
        assert_eq!(cmd.segments, vec![vec!["0105", "010C", "010D"]]);
    }

    /// Admission: unconfirmed lengths ride solo, confirmed chunk; a
    /// non-Mode-01 pid never chunks. (Admission is what the ENGINE consults
    /// pick-by-pick — verified here at the policy level.)
    #[test]
    fn admission_routes_unconfirmed_solo() {
        let (mut plugin, cache) = test_plugin();
        plugin.set_chunk_capable(true);
        cache.lock().unwrap().record("010C", 2);
        let a = plugin.admission();

        // Confirmed chunk member admits alongside another confirmed one…
        cache.lock().unwrap().record("010D", 1);
        assert!(a.admit(&[pick("010C")], &pick("010D")));
        // …but an UNCONFIRMED pid needs its own segment → refused (pipe=1).
        assert!(!a.admit(&[pick("010C")], &pick("010F")));
        // Mode 22 never chunks, support/monitor pids never chunk.
        assert!(!a.admit(&[pick("010C")], &pick("22F40C")));
        assert!(!a.admit(&[pick("010C")], &pick("0100")));
        assert!(!a.admit(&[pick("010C")], &pick("0101")));
        // Chunk capacity: 6 confirmed members max.
        for (i, p) in ["0104", "0105", "010B", "010D", "010F"].iter().enumerate() {
            cache.lock().unwrap().record(p, (i + 1) as u8);
        }
        let six: Vec<DuePick> = ["010C", "0104", "0105", "010B", "010D", "010F"]
            .iter()
            .map(|p| pick(p))
            .collect();
        cache.lock().unwrap().record("0111", 1);
        assert!(
            !a.admit(&six, &pick("0111")),
            "7th member exceeds J1979's 6"
        );
    }

    /// Composition (BP × B): chunks fill pipe segments; a non-chunkable pick
    /// rides its own segment in the same exchange.
    #[test]
    fn poll_plugin_composes_chunks_with_pipes() {
        let (mut plugin, cache) = test_plugin();
        plugin.set_pipe_capable(true);
        plugin.set_chunk_capable(true);
        for (pid, len) in [("010C", 2), ("010D", 1)] {
            cache.lock().unwrap().record(pid, len);
        }
        // 0105 unconfirmed → solo segment; the benched composed shape.
        let due = vec![pick("010C"), pick("010D"), pick("0105")];
        let cmd = plugin.next_command(&due).unwrap();
        assert_eq!(cmd.wire, "010C0D|0105");
        assert_eq!(
            cmd.segments,
            vec![vec!["010C".to_string(), "010D".into()], vec!["0105".into()]]
        );
    }

    /// Adapter cap (adapters.json maxChunk): applied on top of the vehicle
    /// capability; clamp halves from the capped size.
    #[test]
    fn poll_plugin_adapter_chunk_cap() {
        let (mut plugin, _cache) = test_plugin();
        plugin.set_chunk_capable(true);
        plugin.set_chunk_cap(3);
        assert_eq!(plugin.max_picks(), 3);
        assert_eq!(plugin.admission().chunk_limit, 3);
        assert_eq!(plugin.clamp_chunk(), 1, "3 → 1 (integer halving)");
        // Cap below 1 is sanitized.
        plugin.set_chunk_capable(true);
        plugin.set_chunk_cap(0);
        assert_eq!(plugin.admission().chunk_limit, 1);
    }

    /// Step 13 capacity: 36 confirmed Fast pids at pipe 12 × chunk 6 →
    /// SIX pipe segments of SIX-pid chunks in one exchange; a 37th pick is
    /// still admissible (7th segment, 12 available) but a mix that would
    /// need a 13th segment is refused.
    #[test]
    fn composed_capacity_six_chunks_of_six() {
        let (mut plugin, cache) = test_plugin();
        plugin.set_pipe_capable(true);
        plugin.set_chunk_capable(true);
        assert_eq!(plugin.max_picks(), PIPE_MAX_SEGMENTS * CHUNK_MAX_PIDS);

        // 36 distinct plain Mode 01 pids, all length-confirmed.
        let pids: Vec<String> = (0x02..=0x60)
            .filter(|b| !matches!(b, 0x20 | 0x40 | 0x60))
            .take(36)
            .map(|b| format!("01{b:02X}"))
            .collect();
        assert_eq!(pids.len(), 36);
        for p in &pids {
            cache.lock().unwrap().record(p, 1);
        }
        let due: Vec<DuePick> = pids.iter().map(|p| pick(p)).collect();
        let cmd = plugin.next_command(&due).unwrap();
        assert_eq!(cmd.segments.len(), 6, "6 chunks");
        assert!(cmd.segments.iter().all(|s| s.len() == 6), "6 pids each");
        assert_eq!(cmd.wire.matches('|').count(), 5);
        assert_eq!(cmd.members().len(), 36);
        // Every segment's wire piece is `01` + 12 hex chars.
        for piece in cmd.wire.split('|') {
            assert_eq!(piece.len(), 14);
            assert!(piece.starts_with("01"));
        }

        // Admission at the boundary: the 36 chunkables + 11 solos = 12
        // segments (6 chunks + wait, 36 chunkables = 6 segments, so 6 solo
        // segments still fit); a 7th solo would need a 13th segment.
        let mut mixed = due.clone();
        for i in 0..6 {
            mixed.push(pick(&format!("22AA{i:02X}"))); // non-chunkable solos
        }
        let a = plugin.admission();
        let extra_solo = pick("22AAFF");
        assert!(
            !a.admit(&mixed, &extra_solo),
            "13th segment must be refused"
        );
        let last = mixed.pop().unwrap();
        assert!(a.admit(&mixed, &last), "12th segment fits");
    }

    // ---- B3: Mode 22 multi-DID ----

    fn pick_ctrl(pid: &str, ctrl: Option<&str>) -> DuePick {
        DuePick {
            pid: pid.to_string(),
            controller: ctrl.map(String::from),
            timeout_ms: None,
        }
    }

    /// The opportunistic probe ladder: Disabled → Untried (pair) → Capable
    /// (full size) / Refused (solos), with the builder emitting the Mustang
    /// bench wire shape (`22` + concatenated DIDs, members sorted).
    #[test]
    fn mode22_chunk_state_ladder_and_wire() {
        let (mut plugin, cache) = test_plugin();
        cache.lock().unwrap().record("22F40C", 2);
        cache.lock().unwrap().record("22F405", 1);
        cache.lock().unwrap().record("22038F", 1);
        let due = vec![pick("22F40C"), pick("22F405"), pick("22038F")];

        // Disabled (default): all solo, single pick at a time.
        assert_eq!(plugin.max_picks(), 1);
        assert!(plugin.admission().chunk_class("22F40C").is_none());

        // Untried: probe pair — exactly 2 merge, the third solos.
        plugin.set_mode22_enabled(true);
        assert_eq!(plugin.max_picks(), 2);
        let groups = plugin.admission().group(&due);
        assert_eq!(groups.len(), 2, "pair + solo: {groups:?}");
        assert_eq!(groups[0].len(), 2);

        // Probe success → Capable: the cap IS the pair (Mustang refuses 3),
        // so three due pids build a pair chunk + a solo segment.
        assert!(plugin.note_mode22_chunk_outcome(true, 2));
        assert!(
            !plugin.note_mode22_chunk_outcome(true, 2),
            "no change second time"
        );
        let cmd = plugin.next_command(&due).unwrap();
        assert_eq!(cmd.wire, "22F405F40C|22038F");
        assert_eq!(
            cmd.segments,
            vec![
                vec!["22F405".to_string(), "22F40C".to_string()],
                vec!["22038F".to_string()],
            ]
        );

        // Refused → solos forever (no thrash).
        assert!(plugin.note_mode22_chunk_outcome(false, 2));
        assert!(plugin.admission().chunk_class("22F40C").is_none());
        let cmd = plugin.next_command(&[pick("22F40C")]).unwrap();
        assert_eq!(cmd.wire, "22F40C", "solo pass-through untouched");
    }

    /// Modes never mix in one segment; Mode 22 chunks additionally require a
    /// shared target controller.
    #[test]
    fn mode22_chunks_never_mix_modes_or_controllers() {
        let (mut plugin, cache) = test_plugin();
        plugin.set_pipe_capable(true);
        plugin.set_chunk_capable(true);
        plugin.set_mode22_enabled(true);
        plugin.note_mode22_chunk_outcome(true, 2);
        for (p, l) in [("010C", 2), ("010D", 1), ("22F40C", 2), ("22F405", 1)] {
            cache.lock().unwrap().record(p, l);
        }
        let due = vec![
            pick("010C"),
            pick("010D"),
            pick_ctrl("22F40C", Some("7E0")),
            pick_ctrl("22F405", Some("7E0")),
        ];
        let cmd = plugin.next_command(&due).unwrap();
        assert_eq!(cmd.wire, "010C0D|22F405F40C", "one chunk per mode, piped");

        // Different targets: the Mode 22 picks must NOT merge.
        let split_targets = vec![
            pick_ctrl("22F40C", Some("7E0")),
            pick_ctrl("22F405", Some("7E1")),
        ];
        let groups = plugin.admission().group(&split_targets);
        assert_eq!(groups.len(), 2, "cross-ECU multi-DID is not a thing");
    }

    #[test]
    fn plain_mode22_data_pid_filter() {
        assert!(is_plain_mode22_data_pid("22F40C"));
        assert!(is_plain_mode22_data_pid("221E12"));
        assert!(is_plain_mode22_data_pid("222222"));
        // F2xx = Tier S dynamic DIDs; 00xx = padding-ambiguous; wrong shapes.
        for bad in ["22F200", "220012", "010C", "22F40C05", "22", "22F4 0C"] {
            assert!(!is_plain_mode22_data_pid(bad), "{bad}");
        }
    }

    /// Adapter clamp and cap apply to Mode 22 too (Dragy lesson: transport
    /// chokes are about wire length, not mode).
    #[test]
    fn mode22_respects_adapter_cap_and_clamp() {
        let (mut plugin, cache) = test_plugin();
        cache.lock().unwrap().record("22F40C", 2);
        cache.lock().unwrap().record("22F405", 1);
        plugin.set_mode22_enabled(true);
        plugin.note_mode22_chunk_outcome(true, 2); // Capable → limit 3
        plugin.set_chunk_cap(2);
        assert_eq!(plugin.admission().chunk22_limit, 2);
        plugin.clamp_chunk(); // halves → 1: effectively solo
        assert!(plugin.admission().chunk_class("22F40C").is_none());
    }

    #[test]
    fn plain_mode01_data_pid_filter() {
        assert!(is_plain_mode01_data_pid("010C"));
        assert!(is_plain_mode01_data_pid("015C"));
        for bad in [
            "0100", "0120", "0140", "0160", "0101", "22F40C", "0902", "010C0D", "01", "010C 1",
        ] {
            assert!(!is_plain_mode01_data_pid(bad), "{bad}");
        }
    }

    /// PluginCtx::pid_len reads the shared learned-length cache — None until
    /// observed (the "rides solo" signal), Some after.
    #[test]
    fn plugin_ctx_pid_len() {
        use std::sync::{Arc, Mutex};
        let cache = Arc::new(Mutex::new(crate::length_cache::LengthCache::new()));
        let ctx = PluginCtx {
            length_cache: Arc::clone(&cache),
        };
        assert_eq!(ctx.pid_len("22F40C"), None);
        cache.lock().unwrap().record("22F40C", 2);
        assert_eq!(ctx.pid_len("22f40c"), Some(2));
    }

    // ---- split_pipe_segments (fixtures from the 2026-07-30 bench) ----

    #[test]
    fn split_bench_three_segment_blob() {
        let blob = "7E8 04 41 0C 3F F3 \r7E9 04 41 0C 3F F3 \r\r|7E8 03 41 0D 73 \r7E9 03 41 0D 73 \r7EA 03 41 0D 73 \r\r|7E8 03 41 05 43 \r7E9 03 41 05 43";
        let segs = split_pipe_segments(blob);
        assert_eq!(segs.len(), 3);
        // Multi-ECU lines survive INSIDE a segment (internal \r kept).
        assert_eq!(segs[0], "7E8 04 41 0C 3F F3 \r7E9 04 41 0C 3F F3");
        assert_eq!(segs[2], "7E8 03 41 05 43 \r7E9 03 41 05 43");
    }

    #[test]
    fn split_keeps_no_data_positions() {
        // Bench 9-sweep shape: NO DATA slots hold their pipe position.
        let blob = "7E8 03 41 04 32 \r\r|NO DATA\r\r|7E8 03 41 0F 41";
        let segs = split_pipe_segments(blob);
        assert_eq!(segs, vec!["7E8 03 41 04 32", "NO DATA", "7E8 03 41 0F 41"]);
    }

    #[test]
    fn split_single_segment_is_identity() {
        assert_eq!(
            split_pipe_segments("7E8 04 41 0C 3F F3"),
            vec!["7E8 04 41 0C 3F F3"]
        );
    }

    /// Abort-tail: fewer output pipes than input — caller sees the shortfall.
    #[test]
    fn split_abort_tail_yields_fewer_segments() {
        let blob = "7E8 04 41 0C 3F F3 \r\r|?";
        let segs = split_pipe_segments(blob);
        assert_eq!(segs.len(), 2);
        assert_eq!(segs[1], "?");
    }
}
