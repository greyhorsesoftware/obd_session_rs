//! SL0 (Session_Lifecycle_Plan) — the lifecycle race harness.
//!
//! Regression net for the interleaving-bug class, built BEFORE the SL1/SL2 refactor.
//! Assertions marked `RED until SL1/SL2` are EXPECTED to fail against current code —
//! that failure is the proof of coverage; they carry `#[ignore]` with the reason and
//! get un-ignored as each SL phase lands. The rest (fence-no-leak, walk-stops,
//! API latency) are the permanent nets that must stay green through SL3/SL4.
//!
//! Chassis: the same full-session mock machinery the identify/drain tests use
//! (mock ExternalPlatform + scripted MockResponder + mpsc status capture).

use super::*;
use crate::external_platform::{
    create_mock_external_platform, create_mock_external_platform_with_delay,
};
use std::sync::mpsc;
use std::time::{Duration, Instant};

/// Boot a full mock session with a captured status stream.
fn boot() -> (
    OBDSessionManager,
    SessionAPIHandle,
    mpsc::Receiver<String>,
    &'static crate::mock_responder::MockExternalContext,
) {
    let (platform, context) = create_mock_external_platform();
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(
        std::sync::Arc::clone(&platform) as _,
        OBDSessionConfig::default(),
        move |msg| {
            let _ = tx.send(msg);
        },
    )
    .unwrap();
    let api = session.api_handle();
    (session, api, rx, context)
}

/// Drain the status stream for `window`, returning every message.
fn collect(rx: &mpsc::Receiver<String>, window: Duration) -> Vec<String> {
    let deadline = Instant::now() + window;
    let mut out = Vec::new();
    while Instant::now() < deadline {
        match rx.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
            Ok(m) => out.push(m),
            Err(_) => break,
        }
    }
    out
}

fn count_phase(msgs: &[String], phase: &str) -> usize {
    let needle = format!("\"{phase}\"");
    msgs.iter().filter(|m| m.contains(&needle)).count()
}

// ── RED until SL1: the engage ladder consults no fence after terminal intent ──

/// The re-engage race, distilled: after the user's terminal intent (disconnect —
/// which bumps the generation), a queued engage beat must REFUSE at entry.
/// Today `maybe_engage_stream` consults no fence at all — the beat starts and
/// sleeps toward engagement. SL1's target: refuse before the first beat sleep.
#[test] // SL1 landed 2026-08-15 — un-ignored, must stay green
fn sl0_engage_refuses_after_terminal_intent() {
    let (session, api, rx, _ctx) = boot();
    let (platform, engage_pending) = {
        let st = api.shared_state.lock().unwrap();
        (
            std::sync::Arc::clone(&st.platform),
            std::sync::Arc::clone(&st.engage_pending),
        )
    };

    // Terminal intent + full drain to DISCONNECTED.
    api.disconnect_and_drain(platform);
    let msgs = collect(&rx, Duration::from_secs(3));
    assert!(
        count_phase(&msgs, "disconnected") >= 1,
        "harness precondition: drain completed"
    );

    // A stale queued beat arrives after the intent.
    api.maybe_engage_stream();
    std::thread::sleep(Duration::from_millis(500));
    assert!(
        !engage_pending.load(std::sync::atomic::Ordering::SeqCst),
        "SL1: a beat after terminal intent must refuse at entry (epoch_stale), not sleep toward engagement"
    );

    session.shutdown().unwrap();
}

// ── RED until SL2: exactly one terminal sequence per session ──

/// Overlapping teardown paths (the FB9 shape: quit-driven disconnect racing the
/// user's button) must produce EXACTLY ONE `disconnecting → disconnected`.
/// Today each caller spawns its own drain and both emit.
#[test]
// SL2 landed 2026-08-15 — un-ignored, must stay green
fn sl0_terminal_sequence_exactly_once() {
    let (session, api, rx, _ctx) = boot();
    let platform = {
        let st = api.shared_state.lock().unwrap();
        std::sync::Arc::clone(&st.platform)
    };

    api.disconnect_and_drain(std::sync::Arc::clone(&platform));
    api.disconnect_and_drain(platform); // the second driver
    let msgs = collect(&rx, Duration::from_secs(4));

    assert_eq!(
        count_phase(&msgs, "disconnecting"),
        1,
        "SL2: exactly one DISCONNECTING regardless of how many paths requested teardown"
    );
    assert_eq!(
        count_phase(&msgs, "disconnected"),
        1,
        "SL2: exactly one DISCONNECTED regardless of how many paths requested teardown"
    );

    session.shutdown().unwrap();
}

/// `session_summary` must be emitted, BEFORE the terminal DISCONNECTED — the
/// SH1 residual SL2 makes structural. (The SessionHealthStore synthesized-line
/// fallback exists because this ordering is currently a race we sometimes lose.)
#[test]
// SL2 landed 2026-08-15 — un-ignored, must stay green
fn sl0_summary_before_disconnected() {
    let (session, api, rx, _ctx) = boot();
    let platform = {
        let st = api.shared_state.lock().unwrap();
        std::sync::Arc::clone(&st.platform)
    };

    // Precondition: a health session exists only once CONNECTED went out
    // (tracker.finish needs started_ms). Same call the real connect flow makes.
    api.health_on_connected();

    api.disconnect_and_drain(platform);
    let msgs = collect(&rx, Duration::from_secs(3));

    let summary_at = msgs.iter().position(|m| m.contains("session_summary"));
    let disconnected_at = msgs.iter().position(|m| m.contains("\"disconnected\""));
    let (Some(s), Some(d)) = (summary_at, disconnected_at) else {
        panic!(
            "SL2: summary present={} disconnected present={} — both must be emitted",
            summary_at.is_some(),
            disconnected_at.is_some()
        );
    };
    assert!(
        s < d,
        "SL2: session_summary (idx {s}) must precede DISCONNECTED (idx {d})"
    );

    session.shutdown().unwrap();
}

// ── Permanent nets (green today; must STAY green through SL1-SL4) ──

/// The fence must not leak across sessions: a FRESH session engages normally.
#[test]
fn sl0_fence_does_not_leak_across_sessions() {
    let (session, api, _rx, _ctx) = boot();
    let engage_pending = {
        let st = api.shared_state.lock().unwrap();
        std::sync::Arc::clone(&st.engage_pending)
    };
    api.maybe_engage_stream();
    assert!(
        engage_pending.load(std::sync::atomic::Ordering::SeqCst),
        "a fresh session's first beat must be accepted"
    );
    session.shutdown().unwrap();
}

/// Terminal intent mid-identify (the ED2 walk case): after disconnect fires
/// during the module walk, at most the in-flight probe reaches the wire —
/// nothing new is issued. (Today the drain kills the processor, which stops
/// the wire; SL1 makes the walk bail explicitly. Either way this must hold.)
#[test]
fn sl0_no_new_probes_after_intent_mid_identify() {
    let (platform, context) = create_mock_external_platform_with_delay(Duration::from_millis(100));
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(
        std::sync::Arc::clone(&platform) as _,
        OBDSessionConfig::default(),
        move |msg| {
            let _ = tx.send(msg);
        },
    )
    .unwrap();
    let api = session.api_handle();

    // Registry matching the default mock VIN's WMI ("2F0") with a table of
    // low-range modules that answer nothing — a long, slow walk.
    let blob = r#"{"version":1,"datasets":[{"id":"sl0","displayName":"sl0",
        "match":{"wmis":["2F0"]},
        "modules":[{"name":"M1","id":"24A"},{"name":"M2","id":"24B"},
                   {"name":"M3","id":"24C"},{"name":"M4","id":"24D"},
                   {"name":"M5","id":"24E"},{"name":"M6","id":"24F"},
                   {"name":"M7","id":"251"},{"name":"M8","id":"252"}]}]}"#;
    api.set_dataset_registry(Some(blob.to_string()));

    let api2 = api.clone();
    let gather = std::thread::spawn(move || {
        let _ = api2.gather_vehicle_info();
    });

    // Let identify get past VIN and into/near the walk, then fire the intent.
    std::thread::sleep(Duration::from_millis(2500));
    let before = context.responder.get_command_history().len();
    api.disconnect_and_drain(platform);
    let _ = collect(&rx, Duration::from_secs(3));
    let _ = gather.join();
    let after = context.responder.get_command_history().len();

    assert!(
        after.saturating_sub(before) <= 2,
        "terminal intent mid-identify: at most the in-flight probe (+ATSH echo) may land; \
         {} commands reached the wire after intent",
        after - before
    );
    session.shutdown().unwrap();
}

/// SL3 phase-enum net: transitions follow the lifecycle — Idle at boot,
/// Connecting on a fresh attempt, Disconnecting synchronously with the
/// owner's disconnect call, Idle again once the worker's drain completes.
#[test]
fn sl0_phase_follows_lifecycle_transitions() {
    let (session, api, rx, _ctx) = boot();
    assert_eq!(api.phase(), worker::Phase::Idle, "fresh session boots Idle");

    let generation = api.begin_connect_attempt();
    assert_eq!(
        api.phase(),
        worker::Phase::Connecting { epoch: generation },
        "a fresh attempt is Connecting at its own epoch"
    );

    let platform = {
        let st = api.shared_state.lock().unwrap();
        std::sync::Arc::clone(&st.platform)
    };
    api.disconnect_and_drain(platform);
    // Synchronous prefix: the owner set Disconnecting before returning.
    assert!(
        matches!(
            api.phase(),
            worker::Phase::Disconnecting { .. } | worker::Phase::Idle
        ),
        "disconnect sets Disconnecting synchronously (Idle if the worker already finished)"
    );
    let msgs = collect(&rx, Duration::from_secs(3));
    assert!(count_phase(&msgs, "disconnected") >= 1, "drain completed");
    assert_eq!(
        api.phase(),
        worker::Phase::Idle,
        "terminal sequence lands back at Idle"
    );

    session.shutdown().unwrap();
}

/// The beachball net: with a WEDGED drain in progress (60s-stuck in-flight
/// command), every host-callable entry point must return in <50ms — commands
/// are enqueue/spawn-shaped, never waiting on lifecycle work.
#[test]
fn sl0_api_calls_return_fast_under_wedged_drain() {
    let (platform, _context) = create_mock_external_platform_with_delay(Duration::from_secs(60));
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(
        std::sync::Arc::clone(&platform) as _,
        OBDSessionConfig::default(),
        move |msg| {
            let _ = tx.send(msg);
        },
    )
    .unwrap();
    let api = session.api_handle();

    api.send_command("010C", 60_000).unwrap();
    std::thread::sleep(Duration::from_millis(100)); // in-flight, will wedge

    let mut worst = Duration::ZERO;
    let mut check = |label: &str, f: &mut dyn FnMut()| {
        let t = Instant::now();
        f();
        let took = t.elapsed();
        assert!(
            took < Duration::from_millis(50),
            "{label} took {took:?} under a wedged drain — beachball risk"
        );
        if took > worst {
            worst = took;
        }
    };

    check("disconnect_and_drain", &mut || {
        api.disconnect_and_drain(std::sync::Arc::clone(&platform) as _)
    });
    check("second disconnect during wedged drain", &mut || {
        api.disconnect_and_drain(std::sync::Arc::clone(&platform) as _)
    });
    check("get_command_processor_stats", &mut || {
        let _ = api.get_command_processor_stats();
    });
    check("set_user_pinned_poll", &mut || {
        api.set_user_pinned_poll(true)
    });
    check("set_stream_tag", &mut || api.set_stream_tag(None));
    check("request_stop_stream", &mut || api.request_stop_stream());

    let _ = collect(&rx, Duration::from_secs(1));
    session.shutdown().unwrap();
}
