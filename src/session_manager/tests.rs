use super::*;
use crate::external_platform::create_mock_external_platform;

#[test]
fn test_session_manager_creation() {
    let (platform, _context) = create_mock_external_platform();
    let config = OBDSessionConfig::default();

    let session = OBDSessionManager::new(platform, config, |_response| {
        // Callback for responses
    })
    .unwrap();

    // Should be able to get API handle
    let _api = session.api_handle();

    // Shutdown
    session.shutdown().unwrap();
}

/// C2 end-to-end: a 29-bit vehicle (captured Jeep frames) is detected, its VIN decoded, and
/// supported PIDs unioned across ECUs — driving the real identify flow through a mock platform.
#[test]
fn identify_29bit_vehicle_end_to_end() {
    use crate::addressing::Addressing;

    let (platform, context) = create_mock_external_platform();
    // Captured Jeep 29-bit frames (18 DA F1 <ecu>).
    context.responder.set_mock_response(
        "0100",
        "18 DA F1 10 06 41 00 BF FE B9 93\r18 DA F1 18 06 41 00 98 18 00 01",
    );
    context.responder.set_mock_response(
        "0902",
        "18 DA F1 10 10 14 49 02 01 31 43 34\r\
             18 DA F1 10 21 4D 4F 43 4B 32 39 53\r\
             18 DA F1 10 22 42 49 54 30 30 30 31",
    );
    context
        .responder
        .set_mock_response("0101", "18 DA F1 10 06 41 01 00 07 E5 00");
    context
        .responder
        .set_mock_response("0120", "18 DA F1 10 06 41 20 80 00 00 01");
    // ECU name for the powertrain module (0x10): "ECM-Engine", multi-frame 090A.
    context.responder.set_mock_response(
        "090A",
        "18 DA F1 10 10 0D 49 0A 01 45 43 4D\r18 DA F1 10 21 2D 45 6E 67 69 6E 65",
    );

    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();

    let api = session.api_handle();
    let info = api
        .gather_vehicle_info()
        .expect("identify should succeed on 29-bit");

    // Addressing detected as 29-bit, tester learned as F1.
    assert_eq!(
        *api.shared_state.lock().unwrap().addressing.lock().unwrap(),
        Addressing::Can29 { tester: 0xF1 }
    );
    // VIN decoded from the multi-frame 0902.
    assert_eq!(info.vin, "1C4MOCK29SBIT0001");
    // Supported PIDs came through (unioned across the responding ECUs).
    assert!(
        !info.supported_pids.is_empty(),
        "supported PIDs should be non-empty"
    );
    // ECUs enumerated passively from the functional responses, then detailed per-ECU.
    let mut ecus: Vec<&str> = info.ecus.iter().map(|e| e.controller_id.as_str()).collect();
    ecus.sort();
    assert_eq!(ecus, vec!["10", "18"]);
    // Per-ECU detail (C5): the physically-targeted 090A name is attributed to ECU 0x10 only.
    let ecu10 = info.ecus.iter().find(|e| e.controller_id == "10").unwrap();
    assert_eq!(ecu10.name, "ECM-Engine");
    let ecu18 = info.ecus.iter().find(|e| e.controller_id == "18").unwrap();
    assert_eq!(ecu18.name, "", "ECU 0x18 didn't answer 090A → blank name");

    session.shutdown().unwrap();
}

/// C5 regression (bmw/gladiator fixtures): on 11-bit, an ECU that answers `090A` but NOT the
/// functional support scan must still be enumerated — the `7E0-7E7` sweep catches it, where
/// the passive roster alone would miss it.
#[test]
fn identify_11bit_enumerates_ecus_only_seen_in_090a() {
    use crate::addressing::Addressing;

    let (platform, context) = create_mock_external_platform();
    // Only the engine ECU (7E8) answers the functional support bitmap...
    context
        .responder
        .set_mock_response("0100", "7E8 06 41 00 BF FE B9 93");
    // ...but 090A reveals BOTH the engine (7E8 → "00") and transmission (7E9 → "01").
    context
        .responder
        .set_mock_response("090A", "7E8 06 49 0A 01 45 43 4D\r7E9 06 49 0A 01 54 43 4D");

    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();
    let info = api
        .gather_vehicle_info()
        .expect("11-bit identify should succeed");

    assert_eq!(
        *api.shared_state.lock().unwrap().addressing.lock().unwrap(),
        Addressing::Can11
    );
    // Both ECUs enumerated with names, even though only the engine answered 0100.
    let mut ecus: Vec<(&str, &str)> = info
        .ecus
        .iter()
        .map(|e| (e.controller_id.as_str(), e.name.as_str()))
        .collect();
    ecus.sort();
    assert_eq!(ecus, vec![("7E0", "ECM"), ("7E1", "TCM")]);

    session.shutdown().unwrap();
}

/// Repro (2018 Jaguar capture): interleaved multi-frame 090A from two ECUs, names embedding
/// a NUL ("ECM\0-EngineControl"). The real capture produced ecu 00 with an EMPTY name while
/// ecu 01 parsed — this drives the same frames through identify to find where 00's name dies.
#[test]
fn identify_11bit_jag_multiframe_ecu_names() {
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("0100", "7E8 06 41 00 BF FE B9 93");
    // Verbatim 090A frames from a 2018 Jaguar capture (ECU names only, no VIN).
    context.responder.set_mock_response(
        "090A",
        "7E8 10 17 49 0A 01 45 43 4D\r\
             7E9 10 17 49 0A 01 54 43 4D\r\
             7E8 21 00 2D 45 6E 67 69 6E\r\
             7E8 22 65 43 6F 6E 74 72 6F\r\
             7E9 21 00 2D 54 72 61 6E 73\r\
             7E8 23 6C 00 00 00 00 00 00\r\
             7E9 22 6D 69 73 43 74 72 6C\r\
             7E9 23 00 00 00 00 00 00 00",
    );

    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();
    let info = api.gather_vehicle_info().expect("identify should succeed");

    let mut ecus: Vec<(&str, &str)> = info
        .ecus
        .iter()
        .map(|e| (e.controller_id.as_str(), e.name.as_str()))
        .collect();
    ecus.sort();
    assert_eq!(
        ecus,
        vec![("7E0", "ECM-EngineControl"), ("7E1", "TCM-TransmisCtrl")],
        "both multi-frame names must assemble and attribute correctly"
    );

    session.shutdown().unwrap();
}

/// Repro (OBD console): a run-once subscription created while a continuous subscription is
/// paused (console open → dashboard hidden) must still emit `subscription_complete` after its
/// PID answers — the console's batch completion depends on it.
/// 2026-08-29 (iPad rotation stalled the EA link mid-scan): a transport
/// error on a member owned ONLY by a run-once subscription (the
/// extended-PID discovery scan) must not pause the world — the member's
/// run is consumed by the error, the chain continues, and the batch still
/// emits `subscription_complete`. Before the fix the solo-wire policy
/// paused every subscription, the scan included, with no resume path.
#[test]
fn run_once_probe_error_does_not_pause_the_scan() {
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    let (platform, context) = create_mock_external_platform();
    context.responder.set_mock_error("22F40C"); // the "stalled" probe
    context
        .responder
        .set_mock_response("22F40D", "7E8 05 62 F4 0D 00 10");

    let messages: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let messages_cb = messages.clone();
    let session = OBDSessionManager::new(platform, config_for_test(), move |json| {
        messages_cb.lock().unwrap().push(json);
    })
    .unwrap();
    let api = session.api_handle();
    context
        .platform
        .update_connection_status(crate::platform::ConnectionStatus::Connected, None);

    let mut rc = std::collections::HashMap::new();
    rc.insert("22F40C".to_string(), Some(1u32));
    rc.insert("22F40D".to_string(), Some(1u32));
    let scan = api
        .create_subscription_with_run_counts(
            Some("custom-pid-scan".to_string()),
            vec!["22F40C".to_string(), "22F40D".to_string()],
            None,
            Some(rc),
        )
        .unwrap();
    api.start_subscription(scan).unwrap();

    let deadline = std::time::Instant::now() + Duration::from_secs(8);
    let mut completed = false;
    while std::time::Instant::now() < deadline {
        if messages
            .lock()
            .unwrap()
            .iter()
            .any(|m| m.contains("subscription_complete") && m.contains(&scan.to_string()))
        {
            completed = true;
            break;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    assert!(
        completed,
        "run-once scan must complete despite one member erroring; got: {:#?}",
        messages
            .lock()
            .unwrap()
            .iter()
            .map(|m| &m[..m.len().min(140)])
            .collect::<Vec<_>>()
    );
    let history = context.responder.get_command_history();
    assert!(
        history.iter().any(|c| c == "22F40C") && history.iter().any(|c| c == "22F40D"),
        "both probes must have gone out: {history:?}"
    );
}

#[test]
fn run_once_sub_completes_while_other_sub_paused() {
    use std::sync::{Arc, Mutex};
    use std::time::Duration;

    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("0100", "7E8 41 00 BE 7F B0 13");
    context
        .responder
        .set_mock_response("010C", "7E8 04 41 0C 1A F8");

    let messages: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let messages_cb = messages.clone();
    let session = OBDSessionManager::new(platform, config_for_test(), move |json| {
        messages_cb.lock().unwrap().push(json);
    })
    .unwrap();
    let api = session.api_handle();

    // Continuous "dashboard" subscription, started then paused (console visible).
    let dash = api
        .create_subscription(None, vec!["010C".to_string()], None)
        .unwrap();
    api.start_subscription(dash).unwrap();
    std::thread::sleep(Duration::from_millis(300));
    api.pause_subscription(dash).unwrap();

    // Run-once "console" subscription for 0100.
    let mut rc = std::collections::HashMap::new();
    rc.insert("0100".to_string(), Some(1u32));
    let console = api
        .create_subscription_with_run_counts(None, vec!["0100".to_string()], None, Some(rc))
        .unwrap();
    api.start_subscription(console).unwrap();

    // Wait for the completion message (event-driven — arrives shortly after the response).
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let mut completed = false;
    while std::time::Instant::now() < deadline {
        {
            let msgs = messages.lock().unwrap();
            if msgs
                .iter()
                .any(|m| m.contains("subscription_complete") && m.contains(&console.to_string()))
            {
                completed = true;
                break;
            }
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    assert!(
        completed,
        "run-once console subscription must emit subscription_complete; got messages: {:#?}",
        messages
            .lock()
            .unwrap()
            .iter()
            .map(|m| &m[..m.len().min(120)])
            .collect::<Vec<_>>()
    );

    session.shutdown().unwrap();
}

/// A positively non-CAN protocol fails identify (decision C).
#[test]
fn identify_non_can_fails() {
    let (platform, context) = create_mock_external_platform();
    // No CAN-shaped response; ATDPN reports ISO 9141 (protocol 3).
    context.responder.set_mock_response("0100", "NO DATA");
    context.responder.set_mock_response("ATDPN", "3");

    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    assert!(
        session.api_handle().gather_vehicle_info().is_err(),
        "non-CAN protocol must fail identify"
    );
    session.shutdown().unwrap();
}

/// RB1: capture the failure reason CODE emitted on the connection callback.
fn identify_and_capture_failure(
    setup: impl FnOnce(&crate::mock_responder::MockExternalContext),
) -> Option<String> {
    use std::sync::{Arc, Mutex};
    let (platform, context) = create_mock_external_platform();
    setup(&context);
    let messages: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let cb = messages.clone();
    let session = OBDSessionManager::new(platform, config_for_test(), move |json| {
        cb.lock().unwrap().push(json);
    })
    .unwrap();
    let result = session.api_handle().gather_vehicle_info();
    assert!(result.is_err(), "identify should have failed");
    // Find the connection_failed message and pull its detail (reason code).
    let msgs = messages.lock().unwrap().clone();
    session.shutdown().unwrap();
    for m in &msgs {
        if m.contains("\"connection_failed\"") {
            let v: serde_json::Value = serde_json::from_str(m).unwrap();
            return v["payload"]["detail"].as_str().map(String::from);
        }
    }
    None
}

#[test]
fn identify_no_bus_response_is_vehicle_not_responding() {
    // 0100 comes back UNABLE TO CONNECT (ignition off / bus not ready) — a
    // retryable "vehicle not responding", NOT "unsupported protocol".
    let reason = identify_and_capture_failure(|ctx| {
        ctx.responder
            .set_mock_response("0100", "SEARCHING...UNABLE TO CONNECT");
        ctx.responder.set_mock_response("ATDPN", "A0");
    });
    assert_eq!(
        reason.as_deref(),
        Some("vehicle_not_responding"),
        "no bus response"
    );
}

#[test]
fn identify_empty_vin_is_vehicle_not_responding() {
    // CAN detected, but 0902 (VIN) returns nothing — a completed identify
    // with no VIN is a failure (RB1: VIN strictly required).
    let reason = identify_and_capture_failure(|ctx| {
        ctx.responder
            .set_mock_response("0100", "7E8 41 00 BE 7F B0 13");
        ctx.responder.set_mock_response("0902", "NO DATA");
        ctx.responder.set_mock_response("0900", "NO DATA");
    });
    assert_eq!(
        reason.as_deref(),
        Some("vehicle_not_responding"),
        "empty VIN"
    );
}

/// 6b (DVI_BLE_TX_Plan): a rejected pid list must unwind — no zombie
/// subscription registered, and the success path still registers one.
/// ("COMP:Fixture Live" is the literal string that aborted the mustang50
/// extended scan, 2026-08-31.)
#[test]
fn create_with_invalid_pid_leaves_no_zombie() {
    let (platform, _context) = create_mock_external_platform();
    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();

    let result = api.create_subscription(
        None,
        vec!["010C".to_string(), "COMP:Fixture Live".to_string()],
        None,
    );
    assert!(result.is_err(), "an invalid pid must fail the whole create");
    assert!(
        api.list_subscriptions().is_empty(),
        "a failed create must leave NO subscription behind: {:?}",
        api.list_subscriptions()
    );

    let ok = api.create_subscription(None, vec!["010C".to_string()], None);
    assert!(ok.is_ok(), "valid create still succeeds after the unwind");
    assert_eq!(
        api.list_subscriptions().len(),
        1,
        "the valid subscription registers"
    );
}

fn config_for_test() -> OBDSessionConfig {
    OBDSessionConfig::default()
}

/// SH1 end-to-end: a mock session that polls, then disconnects, leaves
/// `acquisition_window` + `session_summary` callback lines in the
/// session jsonl — and exactly ONE summary (the disconnect drain emits
/// it; shutdown's backstop must stay a no-op).
#[test]
fn session_health_windows_and_summary_reach_the_jsonl() {
    use std::time::Duration;

    let temp_dir = std::env::temp_dir().join("obd_session_health_e2e");
    let _ = std::fs::remove_dir_all(&temp_dir);

    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("010C", "7E8 04 41 0C 1A F8");

    let mut config = config_for_test();
    config.log_base_path = Some(temp_dir.to_string_lossy().into_owned());
    config.session_name = Some("health_e2e".to_string());

    let platform_for_disconnect = Arc::clone(&platform);
    let session = OBDSessionManager::new(platform, config, |_| {}).unwrap();
    let api = session.api_handle();

    // Poll a while — the first delivered signal opens the poll window.
    let sub = api
        .create_subscription(None, vec!["010C".to_string()], None)
        .unwrap();
    api.start_subscription(sub).unwrap();
    std::thread::sleep(Duration::from_millis(400));

    // Disconnect: the drain thread sends DISCONNECTED then emits the
    // final window + session summary.
    api.disconnect_and_drain(platform_for_disconnect);
    std::thread::sleep(Duration::from_millis(800));

    let log_path = temp_dir.join("health_e2e.jsonl");
    session.shutdown().unwrap();

    let content = std::fs::read_to_string(&log_path).expect("session jsonl exists");
    let mut window_payloads: Vec<serde_json::Value> = Vec::new();
    let mut summary_payloads: Vec<serde_json::Value> = Vec::new();
    for line in content.lines() {
        let v: serde_json::Value = serde_json::from_str(line).expect("valid jsonl line");
        if v["event"] != "callback" {
            continue;
        }
        match v["type"].as_str() {
            Some("acquisition_window") => window_payloads
                .push(serde_json::from_str(v["data"].as_str().unwrap()).expect("window payload")),
            Some("session_summary") => summary_payloads
                .push(serde_json::from_str(v["data"].as_str().unwrap()).expect("summary payload")),
            _ => {}
        }
    }

    assert!(
        !window_payloads.is_empty(),
        "the poll window must be logged as acquisition_window"
    );
    let w = &window_payloads[0];
    assert_eq!(w["kind"], "acquisition_window");
    assert_eq!(w["tier"], "poll");
    assert!(
        w["signals"].as_u64().unwrap() > 0,
        "mock polling delivered signals"
    );
    assert!(w["end_ms"].as_u64().unwrap() >= w["start_ms"].as_u64().unwrap());

    assert_eq!(
        summary_payloads.len(),
        1,
        "summary must emit exactly once (disconnect emitted; shutdown no-op)"
    );
    let s = &summary_payloads[0];
    assert_eq!(s["kind"], "session_summary");
    assert!(s["totals"]["signals"].as_u64().unwrap() > 0);
    assert_eq!(s["windows"][0]["tier"], "poll");
    assert!(s["ended_ms"].as_u64().unwrap() >= s["started_ms"].as_u64().unwrap());

    let _ = std::fs::remove_dir_all(&temp_dir);
}

/// P0 golden equality: with the engine routed through the poll plugin at
/// chunk = 1, a run-once subscription's wire traffic and completion are
/// byte-identical to the legacy serial path — the exact command goes out
/// once, the subscription completes, and the member map is drained.
#[test]
fn p0_poll_plugin_serial_equality_end_to_end() {
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("010C", "7E8 04 41 0C 1A F8");

    use std::sync::mpsc;
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();

    let mut run_counts = std::collections::HashMap::new();
    run_counts.insert("010C".to_string(), Some(1u32));
    let sub_id = api
        .create_subscription_with_run_counts(
            Some("p0-test".to_string()),
            vec!["010C".to_string()],
            None,
            Some(run_counts),
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    // Wait for the subscription_complete message (run-once finished).
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let mut completed = false;
    while std::time::Instant::now() < deadline {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(200)) {
            if msg.contains("subscription_complete") {
                completed = true;
                break;
            }
        }
    }
    assert!(
        completed,
        "run-once subscription must complete through the plugin path"
    );

    // Wire golden: exactly the bare pid went out (no rewriting).
    let sent_010c = context
        .responder
        .get_command_history()
        .iter()
        .filter(|c| c.as_str() == "010C")
        .count();
    assert_eq!(
        sent_010c, 1,
        "run-once pid must be sent exactly once, verbatim"
    );

    // Member map drained (no leaks).
    assert!(
        api.shared_state
            .lock()
            .unwrap()
            .in_flight_members
            .lock()
            .unwrap()
            .is_empty(),
        "in-flight member map must drain on completion"
    );

    // Length learning rode along: "7E8 04 41 0C 1A F8" → 2 data bytes
    // observed for 010C (observe-then-batch, recorded in the data
    // callback with no extra wire traffic).
    assert_eq!(
        api.shared_state
            .lock()
            .unwrap()
            .length_cache
            .lock()
            .unwrap()
            .get("010C"),
        Some(2),
        "single-PID response must record its observed data length"
    );

    session.shutdown().unwrap();
}

/// Warm-start race guard (seen on the bench 2026-07-30): an ATE0 swallowed by
/// the adapter's ATWS reboot gets the late reset banner correlated as its
/// response — init must detect the missing OK and resend ATE0 exactly once
/// (and still proceed, so a genuinely broken echo doesn't block connects).
#[test]
fn init_retries_swallowed_ate0_once() {
    let (platform, context) = create_mock_external_platform();
    context.responder.set_mock_response("ATE0", "ELM327 v1.4b");

    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();
    assert!(
        api.send_init_commands(),
        "init should proceed despite the retry"
    );

    let ate0_sends = context
        .responder
        .get_command_history()
        .iter()
        .filter(|c| c.eq_ignore_ascii_case("ATE0"))
        .count();
    assert_eq!(ate0_sends, 2, "ATE0 should be retried exactly once");

    session.shutdown().unwrap();
}

/// BP1 end-to-end: a pipe-capable adapter (STBC 1 → OK) gets a PIPED
/// exchange for two due pids; the demux emits per-member obd_data with
/// each member's segment, both members learn their lengths, and the
/// run-once subscription completes.
#[test]
fn bp1_piped_exchange_end_to_end() {
    let (platform, context) = create_mock_external_platform();
    // Opt the mock into pipe capability (default mock answers "?").
    context.responder.set_mock_response("STBC 1", "OK");
    context.responder.set_mock_response("STBCOF 0", "OK");
    // Both pick orders (equal ticks → registry iteration order varies).
    context
        .responder
        .set_mock_response("010C|010D", "7E8 04 41 0C 3F F3 \r\r|7E8 03 41 0D 73");
    context
        .responder
        .set_mock_response("010D|010C", "7E8 03 41 0D 73 \r\r|7E8 04 41 0C 3F F3");

    use std::sync::mpsc;
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();

    // Init runs the STBC probe → poll plugin becomes pipe-capable.
    assert!(api.send_init_commands());

    let mut run_counts = std::collections::HashMap::new();
    run_counts.insert("010C".to_string(), Some(1u32));
    run_counts.insert("010D".to_string(), Some(1u32));
    let sub_id = api
        .create_subscription_with_run_counts(
            Some("bp1-test".to_string()),
            vec!["010C".to_string(), "010D".to_string()],
            None,
            Some(run_counts),
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    // Collect per-member obd_data + the completion.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let (mut saw_c, mut saw_d, mut completed) = (false, false, false);
    while std::time::Instant::now() < deadline && !(saw_c && saw_d && completed) {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(200)) {
            if msg.contains("subscription_complete") {
                completed = true;
            }
            // LH7: the host decodes ONLY parsed.all_controllers[*].data_bytes —
            // pin that the typed path still emits it (the substring on
            // `data` alone would pass on the raw text).
            if msg.contains("\"command\":\"010C\"") && msg.contains("41 0C 3F F3") {
                assert!(msg.contains("\"data_bytes\":[63,243]"), "{msg}");
                saw_c = true;
            }
            if msg.contains("\"command\":\"010D\"") && msg.contains("41 0D 73") {
                assert!(msg.contains("\"data_bytes\":[115]"), "{msg}");
                saw_d = true;
            }
        }
    }
    assert!(
        saw_c && saw_d,
        "each member must get its own obd_data segment"
    );
    assert!(completed, "run-once must complete off one piped exchange");

    // Exactly one piped wire went out — not two serial commands.
    let history = context.responder.get_command_history();
    assert!(
        history.iter().any(|c| c == "010C|010D" || c == "010D|010C"),
        "wire must be the piped exchange, got {:?}",
        history
    );
    assert!(
        !history.iter().any(|c| c == "010C" || c == "010D"),
        "members must not ALSO go out serially"
    );

    // Length learning per segment.
    let state = api.shared_state.lock().unwrap();
    let lengths = state.length_cache.lock().unwrap();
    assert_eq!(lengths.get("010C"), Some(2));
    assert_eq!(lengths.get("010D"), Some(1));
    drop(lengths);
    drop(state);

    // BP2: the piped exchange counted ONE command but TWO signals — the
    // measured signals metric must not under-report by the batch factor.
    let stats = api.get_command_processor_stats().unwrap();
    assert!(
        stats.completed_signals >= stats.completed_commands + 1,
        "signals ({}) must exceed wire commands ({}) after a piped exchange",
        stats.completed_signals,
        stats.completed_commands
    );

    session.shutdown().unwrap();
}

/// Equality invariant: on a NON-capable adapter (default mock: STBC → ?)
/// the same subscription produces plain serial wires — no pipes anywhere.
#[test]
fn bp1_pipe_incapable_adapter_stays_serial() {
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("010C", "7E8 04 41 0C 3F F3");
    context
        .responder
        .set_mock_response("010D", "7E8 03 41 0D 73");

    use std::sync::mpsc;
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands()); // probe declines → pipe stays 1

    let mut run_counts = std::collections::HashMap::new();
    run_counts.insert("010C".to_string(), Some(1u32));
    run_counts.insert("010D".to_string(), Some(1u32));
    let sub_id = api
        .create_subscription_with_run_counts(
            Some("bp1-serial".to_string()),
            vec!["010C".to_string(), "010D".to_string()],
            None,
            Some(run_counts),
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let mut completed = false;
    while std::time::Instant::now() < deadline && !completed {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(200)) {
            if msg.contains("subscription_complete") {
                completed = true;
            }
        }
    }
    assert!(completed);

    let history = context.responder.get_command_history();
    assert!(
        !history.iter().any(|c| c.contains('|')),
        "no piped wires on a non-capable adapter, got {:?}",
        history
    );
    assert!(history.iter().any(|c| c == "010C"));
    assert!(history.iter().any(|c| c == "010D"));

    session.shutdown().unwrap();
}

/// B1 e2e: chunk-capable session merges two length-confirmed Mode 01
/// pids into ONE `010C0D` wire; each member gets its own obd_data (with
/// its own parsed slice) and run-once completes off the single exchange.
#[test]
fn b1_chunked_exchange_end_to_end() {
    let (platform, context) = create_mock_external_platform();
    // Pipe probe declines (mock default "?") — chunk works without pipes.
    context
        .responder
        .set_mock_response("010C0D", "7E8 06 41 0C 3F F3 0D 73");

    use std::sync::mpsc;
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());

    // B2's vehicle probe will do this on real connects; tests opt in.
    {
        let state = api.shared_state.lock().unwrap();
        state.acquisition.lock().unwrap().set_chunk_capable(true);
        let mut cache = state.length_cache.lock().unwrap();
        cache.record("010C", 2);
        cache.record("010D", 1);
    }

    let mut run_counts = std::collections::HashMap::new();
    run_counts.insert("010C".to_string(), Some(1u32));
    run_counts.insert("010D".to_string(), Some(1u32));
    let sub_id = api
        .create_subscription_with_run_counts(
            Some("b1-chunk".to_string()),
            vec!["010C".to_string(), "010D".to_string()],
            None,
            Some(run_counts),
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let (mut saw_c, mut saw_d, mut completed) = (false, false, false);
    while std::time::Instant::now() < deadline && !(saw_c && saw_d && completed) {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(200)) {
            if msg.contains("subscription_complete") {
                completed = true;
            }
            // Each member's obd_data carries ITS parsed slice ("41 0C 3F F3"
            // raw_hex inside parsed), independent of the shared chunk blob.
            if msg.contains("\"command\":\"010C\"") && msg.contains("41 0C 3F F3") {
                saw_c = true;
            }
            if msg.contains("\"command\":\"010D\"") && msg.contains("41 0D 73") {
                saw_d = true;
            }
        }
    }
    assert!(
        saw_c && saw_d,
        "each member must get its own sliced obd_data"
    );
    assert!(completed, "run-once must complete off one chunked exchange");

    let history = context.responder.get_command_history();
    assert!(
        history.iter().any(|c| c == "010C0D"),
        "wire must be the J1979 chunk, got {:?}",
        history
    );
    assert!(
        !history.iter().any(|c| c == "010C" || c == "010D"),
        "members must not ALSO go out solo"
    );

    // Healthy chunk → lengths intact.
    let state = api.shared_state.lock().unwrap();
    let lengths = state.length_cache.lock().unwrap();
    assert_eq!(lengths.get("010C"), Some(2));
    assert_eq!(lengths.get("010D"), Some(1));
    drop(lengths);
    drop(state);

    session.shutdown().unwrap();
}

/// B1 demotion loop e2e: a STALE learned length mis-slices the chunk →
/// validation failure → members stay due + lengths invalidated → the
/// admission rule routes them SOLO → solo responses re-teach the lengths
/// and the members complete. No flags anywhere — demotion and first
/// observation are the same code path.
#[test]
fn b1_chunk_validation_failure_demotes_and_recovers() {
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("010C0D", "7E8 06 41 0C 3F F3 0D 73");
    context
        .responder
        .set_mock_response("010C", "7E8 04 41 0C 3F F3");
    context
        .responder
        .set_mock_response("010D", "7E8 03 41 0D 73");

    use std::sync::mpsc;
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());

    {
        let state = api.shared_state.lock().unwrap();
        state.acquisition.lock().unwrap().set_chunk_capable(true);
        let mut cache = state.length_cache.lock().unwrap();
        cache.record("010C", 1); // STALE — real slice is 2 bytes
        cache.record("010D", 1);
    }

    // CONTINUOUS subscription — the dashboard shape demotion serves.
    // (Run-once members account even on a failed exchange, by design —
    // the NO DATA semantics console one-shots rely on.)
    let sub_id = api
        .create_subscription(
            Some("b1-demote".to_string()),
            vec!["010C".to_string(), "010D".to_string()],
            None,
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();
    drop(rx); // history + lengths are the observables here

    // Expected wire sequence: 010C0D (mis-slice → demote) → solo 010C +
    // solo 010D (re-observe true lengths) → 010C0D again (re-admitted).
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let mut history: Vec<String> = Vec::new();
    while std::time::Instant::now() < deadline {
        history = context.responder.get_command_history();
        if history.iter().filter(|c| *c == "010C0D").count() >= 2 {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    let first_chunk = history.iter().position(|c| c == "010C0D").unwrap();
    let solo_c = history.iter().position(|c| c == "010C");
    let solo_d = history.iter().position(|c| c == "010D");
    assert!(
        solo_c.is_some_and(|i| i > first_chunk) && solo_d.is_some_and(|i| i > first_chunk),
        "demoted members must re-poll SOLO after the failed chunk, got {:?}",
        history
    );
    assert!(
        history.iter().filter(|c| *c == "010C0D").count() >= 2,
        "re-learned lengths must RE-ADMIT the members to a chunk, got {:?}",
        history
    );

    // Solo responses re-taught the true lengths.
    let state = api.shared_state.lock().unwrap();
    let lengths = state.length_cache.lock().unwrap();
    assert_eq!(lengths.get("010C"), Some(2), "re-observed true length");
    assert_eq!(lengths.get("010D"), Some(1));
    drop(lengths);
    drop(state);

    session.shutdown().unwrap();
}

/// B2 e2e: the vehicle probe (`010C0D` answers both) enables the chunk
/// axis — a subsequent subscription rides ONE chunk wire, no manual
/// capability flipping anywhere.
#[test]
fn b2_probe_enables_chunking() {
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("010C0D", "7E8 06 41 0C 3F F3 0D 73");

    use std::sync::mpsc;
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    api.probe_chunk_capability();

    let (_, pipe_limit, chunk_limit, _, _) = api.get_acquisition_stats();
    assert_eq!(pipe_limit, 1, "mock adapter declines pipes");
    assert_eq!(chunk_limit, 6, "probe must enable the chunk axis");

    // Lengths confirmed → the pair chunks.
    {
        let state = api.shared_state.lock().unwrap();
        let mut cache = state.length_cache.lock().unwrap();
        cache.record("010C", 2);
        cache.record("010D", 1);
    }
    let mut run_counts = std::collections::HashMap::new();
    run_counts.insert("010C".to_string(), Some(1u32));
    run_counts.insert("010D".to_string(), Some(1u32));
    let sub_id = api
        .create_subscription_with_run_counts(
            Some("b2-probe".to_string()),
            vec!["010C".to_string(), "010D".to_string()],
            None,
            Some(run_counts),
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let mut completed = false;
    while std::time::Instant::now() < deadline && !completed {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(200)) {
            if msg.contains("subscription_complete") {
                completed = true;
            }
        }
    }
    assert!(completed);
    let history = context.responder.get_command_history();
    // Probe + subscription both used the chunk wire; members never solo.
    assert!(history.iter().filter(|c| *c == "010C0D").count() >= 2);
    assert!(!history.iter().any(|c| c == "010C" || c == "010D"));

    session.shutdown().unwrap();
}

/// B2: a vehicle that doesn't answer multi-PID (probe → NO DATA, the
/// fault-mock/Dragy-clone shape) stays serial end-to-end.
#[test]
fn b2_refusing_vehicle_stays_serial() {
    let (platform, context) = create_mock_external_platform();
    // No "010C0D" mock entry → responder answers NO DATA.
    context
        .responder
        .set_mock_response("010C", "7E8 04 41 0C 3F F3");
    context
        .responder
        .set_mock_response("010D", "7E8 03 41 0D 73");

    use std::sync::mpsc;
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    api.probe_chunk_capability();

    let (_, _, chunk_limit, _, _) = api.get_acquisition_stats();
    assert_eq!(chunk_limit, 1, "NO DATA probe must leave chunking off");

    // Even with confirmed lengths, the members go out solo.
    {
        let state = api.shared_state.lock().unwrap();
        let mut cache = state.length_cache.lock().unwrap();
        cache.record("010C", 2);
        cache.record("010D", 1);
    }
    let mut run_counts = std::collections::HashMap::new();
    run_counts.insert("010C".to_string(), Some(1u32));
    run_counts.insert("010D".to_string(), Some(1u32));
    let sub_id = api
        .create_subscription_with_run_counts(
            Some("b2-serial".to_string()),
            vec!["010C".to_string(), "010D".to_string()],
            None,
            Some(run_counts),
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let mut completed = false;
    while std::time::Instant::now() < deadline && !completed {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(200)) {
            if msg.contains("subscription_complete") {
                completed = true;
            }
        }
    }
    assert!(completed);
    let history = context.responder.get_command_history();
    assert!(history.iter().any(|c| c == "010C"));
    assert!(history.iter().any(|c| c == "010D"));
    assert_eq!(
        history.iter().filter(|c| *c == "010C0D").count(),
        1,
        "only the probe itself, never a subscription chunk: {:?}",
        history
    );

    session.shutdown().unwrap();
}

/// B2 member demotion: the vehicle answers the chunk but silently OMITS
/// one member → that member is excluded and solos; the REST of the set
/// keeps chunking (no whole-set demotion, no starvation loop).
#[test]
fn b2_omitted_member_solos_set_survives() {
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("010C0D", "7E8 06 41 0C 3F F3 0D 73");
    // The 3-PID chunk answers only 0C and 0D — 05 silently omitted.
    // (Chunk wires are member-sorted → deterministic mock key.)
    context
        .responder
        .set_mock_response("01050C0D", "7E8 06 41 0C 3F F3 0D 73");
    context
        .responder
        .set_mock_response("0105", "7E8 03 41 05 4A");

    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    api.probe_chunk_capability();

    {
        let state = api.shared_state.lock().unwrap();
        let mut cache = state.length_cache.lock().unwrap();
        cache.record("010C", 2);
        cache.record("010D", 1);
        cache.record("0105", 1);
    }
    // Continuous dashboard shape.
    let sub_id = api
        .create_subscription(
            Some("b2-omit".to_string()),
            vec!["010C".to_string(), "010D".to_string(), "0105".to_string()],
            None,
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    // Wait until the set has re-formed WITHOUT the omitted member.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let mut history: Vec<String> = Vec::new();
    while std::time::Instant::now() < deadline {
        history = context.responder.get_command_history();
        if history.iter().any(|c| c == "010C0D") && history.iter().any(|c| c == "0105") {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    assert!(
        history.iter().any(|c| c == "01050C0D"),
        "first sweep tried the full chunk: {:?}",
        history
    );
    assert!(
        history.iter().any(|c| c == "0105"),
        "omitted member must re-poll SOLO: {:?}",
        history
    );
    assert!(
        history.iter().any(|c| c == "010C0D"),
        "the surviving pair must KEEP chunking: {:?}",
        history
    );
    // 0105's length was never invalidated — exclusion, not demotion.
    let state = api.shared_state.lock().unwrap();
    assert_eq!(state.length_cache.lock().unwrap().get("0105"), Some(1));
    drop(state);

    session.shutdown().unwrap();
}

/// B2 adapter demotion (the Dragy incident): the VEHICLE accepts chunks
/// (2-PID probe answers) but the clone ADAPTER drops the prompt on a
/// bigger chunk wire. Each transport failure halves the chunk size —
/// 6 → 3 → 1 — until the sweep fits; members keep flowing solo instead
/// of wedging the dashboard on a 5 s timeout every sweep.
#[test]
fn b2_transport_failure_clamps_chunk_size() {
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("010C0D", "7E8 06 41 0C 3F F3 0D 73");
    // The 3-member chunk (sorted wire) chokes the adapter — at limit 6
    // AND at limit 3 the sweep builds the same wire, so it errors twice.
    context.responder.set_mock_error("01050C0D");
    context
        .responder
        .set_mock_response("010C", "7E8 04 41 0C 3F F3");
    context
        .responder
        .set_mock_response("010D", "7E8 03 41 0D 73");
    context
        .responder
        .set_mock_response("0105", "7E8 03 41 05 4A");

    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    api.probe_chunk_capability();
    assert_eq!(api.get_acquisition_stats().2, 6, "probe enabled chunking");

    {
        let state = api.shared_state.lock().unwrap();
        let mut cache = state.length_cache.lock().unwrap();
        cache.record("010C", 2);
        cache.record("010D", 1);
        cache.record("0105", 1);
    }
    // All Fast so every sweep wants all three members together.
    let sub_id = api
        .create_subscription(
            Some("b2-clamp".to_string()),
            vec!["010C".to_string(), "010D".to_string(), "0105".to_string()],
            None,
        )
        .unwrap();
    {
        let state = api.shared_state.lock().unwrap();
        let mut sub_manager = state.subscription_manager.lock().unwrap();
        use crate::subscription::RefreshTier;
        let tiers: std::collections::HashMap<String, RefreshTier> = [
            ("010C", RefreshTier::Fast),
            ("010D", RefreshTier::Fast),
            ("0105", RefreshTier::Fast),
        ]
        .iter()
        .map(|(p, t)| (p.to_string(), *t))
        .collect();
        sub_manager.set_pid_tiers(sub_id, tiers).unwrap();
    }
    api.start_subscription(sub_id).unwrap();

    // Wait for the clamp to bottom out and solo wires to flow.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(15);
    let mut history: Vec<String> = Vec::new();
    while std::time::Instant::now() < deadline {
        history = context.responder.get_command_history();
        if history.iter().any(|c| c == "0105") {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    assert!(
        history.iter().filter(|c| *c == "01050C0D").count() >= 2,
        "the failing wire went out at limit 6 AND limit 3: {:?}",
        history
    );
    assert!(
        history.iter().any(|c| c == "0105"),
        "after clamping to 1 the members flow solo: {:?}",
        history
    );
    assert_eq!(api.get_acquisition_stats().2, 1, "chunk limit bottomed out");

    session.shutdown().unwrap();
}

/// B3 e2e — the Mustang path: two length-confirmed Mode 22 pids compose
/// into the probe pair (`22F405F40C`), the golden multi-frame `62` blob
/// comes back, both members deliver solo-shaped obd_data, the plugin
/// flips Capable, and the next sweep chunks again.
#[test]
fn b3_mode22_chunk_probe_and_steady_state() {
    use std::sync::mpsc;
    let (platform, context) = create_mock_external_platform();
    // Verbatim Mustang bench response (2026-07-31, idling).
    context.responder.set_mock_response(
        "22F405F40C",
        "7E8 10 08 62 F4 0C 0B 73 F4 \r7E8 21 05 7A 00 00 00 00 00",
    );

    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    {
        let state = api.shared_state.lock().unwrap();
        state.acquisition.lock().unwrap().set_mode22_enabled(true);
        let mut cache = state.length_cache.lock().unwrap();
        cache.record("22F40C", 2);
        cache.record("22F405", 1);
    }
    let sub_id = api
        .create_subscription(
            Some("b3".to_string()),
            vec!["22F40C".to_string(), "22F405".to_string()],
            None,
        )
        .unwrap();
    {
        let state = api.shared_state.lock().unwrap();
        let mut sub_manager = state.subscription_manager.lock().unwrap();
        use crate::subscription::RefreshTier;
        let tiers: std::collections::HashMap<String, RefreshTier> =
            [("22F40C", RefreshTier::Fast), ("22F405", RefreshTier::Fast)]
                .iter()
                .map(|(p, t)| (p.to_string(), *t))
                .collect();
        sub_manager.set_pid_tiers(sub_id, tiers).unwrap();
    }
    api.start_subscription(sub_id).unwrap();

    // Both members must deliver from the sliced chunk.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
    let (mut saw_rpm, mut saw_ect) = (false, false);
    while std::time::Instant::now() < deadline && !(saw_rpm && saw_ect) {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(200)) {
            if msg.contains("\"command\":\"22F40C\"") && msg.contains("62 F4 0C 0B 73") {
                saw_rpm = true;
            }
            if msg.contains("\"command\":\"22F405\"") && msg.contains("62 F4 05 7A") {
                saw_ect = true;
            }
        }
    }
    assert!(saw_rpm && saw_ect, "chunk members deliver solo-shaped data");

    // Steady state: the composed wire repeats (probe → Capable → chunk).
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    loop {
        let n = context
            .responder
            .get_command_history()
            .iter()
            .filter(|c| *c == "22F405F40C")
            .count();
        if n >= 2 {
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "composed wire must repeat, history: {:?}",
            context.responder.get_command_history()
        );
        std::thread::sleep(std::time::Duration::from_millis(50));
    }

    session.shutdown().unwrap();
}

/// B3 e2e — refusal: the vehicle answers the multi-DID probe with
/// `7F 22 31`. The pair goes Refused (ONE composed attempt, ever — no
/// re-chunk thrash), lengths stay intact, and both members flow solo.
#[test]
fn b3_mode22_refusal_stays_solo() {
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("22F405F40C", "7E8 03 7F 22 31");
    context
        .responder
        .set_mock_response("22F40C", "7E8 05 62 F4 0C 0B 73");
    context
        .responder
        .set_mock_response("22F405", "7E8 04 62 F4 05 7A");

    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();
    {
        let state = api.shared_state.lock().unwrap();
        state.acquisition.lock().unwrap().set_mode22_enabled(true);
        let mut cache = state.length_cache.lock().unwrap();
        cache.record("22F40C", 2);
        cache.record("22F405", 1);
    }
    let sub_id = api
        .create_subscription(
            Some("b3-refused".to_string()),
            vec!["22F40C".to_string(), "22F405".to_string()],
            None,
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    // Wait until several solo polls of BOTH members have flowed.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
    let mut history: Vec<String> = Vec::new();
    while std::time::Instant::now() < deadline {
        history = context.responder.get_command_history();
        if history.iter().filter(|c| *c == "22F40C").count() >= 2
            && history.iter().filter(|c| *c == "22F405").count() >= 2
        {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    assert_eq!(
        history.iter().filter(|c| *c == "22F405F40C").count(),
        1,
        "exactly one composed attempt — Refused must not thrash: {history:?}"
    );
    assert!(
        history.iter().filter(|c| *c == "22F40C").count() >= 2,
        "members flow solo after refusal: {history:?}"
    );
    // Lengths were NOT invalidated (they came from solo observations).
    let state = api.shared_state.lock().unwrap();
    assert_eq!(state.length_cache.lock().unwrap().get("22F40C"), Some(2));
    drop(state);

    session.shutdown().unwrap();
}

/// Step 13 — composed golden replay of the 19:02 bench: pipe-capable
/// adapter + chunk-capable vehicle, confirmed pair chunks while the
/// unconfirmed member rides its own pipe segment (`010C0D|0105` — the
/// blob is the bench capture verbatim). The solo segment then TEACHES
/// 0105's length, so the NEXT sweep composes down to one chunk
/// (`01050C0D`) — observe-then-batch across the composition boundary.
#[test]
fn composed_pipe_chunk_golden_replay() {
    let (platform, context) = create_mock_external_platform();
    context.responder.set_mock_response("STBC 1", "OK");
    context.responder.set_mock_response("STBCOF 0", "OK");
    context
        .responder
        .set_mock_response("010C0D", "7E8 06 41 0C 3F E3 0D 73");
    // Bench blob verbatim (19:02:46 exchange).
    context.responder.set_mock_response(
            "010C0D|0105",
            "7E8 06 41 0C 3F E3 0D 73 \r7E9 06 41 0C 3F E3 0D 73 \r7EA 03 41 0D 73 \r\r|7E8 03 41 05 4A \r7E9 03 41 05 4A",
        );
    context.responder.set_mock_response(
        "01050C0D",
        "7E8 10 08 41 05 4A 0C 3F E3 \r7E8 21 0D 73 00 00 00 00 00",
    );

    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    {
        let state = api.shared_state.lock().unwrap();
        state.acquisition.lock().unwrap().set_chunk_capable(true);
        let mut cache = state.length_cache.lock().unwrap();
        cache.record("010C", 2);
        cache.record("010D", 1);
        // 0105 deliberately UNCONFIRMED → solo pipe segment first.
    }
    let sub_id = api
        .create_subscription(
            Some("composed".to_string()),
            vec!["010C".to_string(), "010D".to_string(), "0105".to_string()],
            None,
        )
        .unwrap();
    {
        let state = api.shared_state.lock().unwrap();
        let mut sub_manager = state.subscription_manager.lock().unwrap();
        use crate::subscription::RefreshTier;
        let tiers: std::collections::HashMap<String, RefreshTier> = [
            ("010C", RefreshTier::Fast),
            ("010D", RefreshTier::Fast),
            ("0105", RefreshTier::Fast),
        ]
        .iter()
        .map(|(p, t)| (p.to_string(), *t))
        .collect();
        sub_manager.set_pid_tiers(sub_id, tiers).unwrap();
    }
    api.start_subscription(sub_id).unwrap();

    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let mut history: Vec<String> = Vec::new();
    while std::time::Instant::now() < deadline {
        history = context.responder.get_command_history();
        if history.iter().any(|c| c == "01050C0D") {
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    let composed = history.iter().position(|c| c == "010C0D|0105");
    let full_chunk = history.iter().position(|c| c == "01050C0D");
    assert!(
        composed.is_some(),
        "composed wire must go out first: {:?}",
        history
    );
    assert!(
        full_chunk.is_some_and(|f| f > composed.unwrap()),
        "solo segment teaches 0105 → next sweep is ONE chunk: {:?}",
        history
    );
    // The solo pipe segment taught the length.
    let state = api.shared_state.lock().unwrap();
    assert_eq!(state.length_cache.lock().unwrap().get("0105"), Some(1));
    drop(state);

    session.shutdown().unwrap();
}

/// Step 13 — a NO DATA pipe segment inside a composed sweep: the chunk
/// segment's members deliver normally, the NO DATA member just stays due
/// — and CRITICALLY nothing is invalidated or clamped (NO DATA is a car
/// answer, not a mis-slice or a transport failure).
#[test]
fn composed_sweep_with_no_data_segment() {
    let (platform, context) = create_mock_external_platform();
    context.responder.set_mock_response("STBC 1", "OK");
    context.responder.set_mock_response("STBCOF 0", "OK");
    context
        .responder
        .set_mock_response("010C0D", "7E8 06 41 0C 3F E3 0D 73");
    context
        .responder
        .set_mock_response("010C0D|0111", "7E8 06 41 0C 3F E3 0D 73 \r\r|NO DATA");

    use std::sync::mpsc;
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    {
        let state = api.shared_state.lock().unwrap();
        state.acquisition.lock().unwrap().set_chunk_capable(true);
        let mut cache = state.length_cache.lock().unwrap();
        cache.record("010C", 2);
        cache.record("010D", 1);
        // 0111 unconfirmed → solo segment; the car answers NO DATA.
    }
    let sub_id = api
        .create_subscription(
            Some("nodata-seg".to_string()),
            vec!["010C".to_string(), "010D".to_string(), "0111".to_string()],
            None,
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    // Chunk members must deliver obd_data; wait for one.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    let mut saw_c = false;
    while std::time::Instant::now() < deadline && !saw_c {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(200)) {
            if msg.contains("\"command\":\"010C\"") && msg.contains("41 0C 3F E3") {
                saw_c = true;
            }
        }
    }
    assert!(
        saw_c,
        "chunk members deliver despite the NO DATA sibling segment"
    );

    let history = context.responder.get_command_history();
    assert!(history.iter().any(|c| c == "010C0D|0111"), "{:?}", history);
    // NO DATA didn't demote anything: lengths intact, chunking intact.
    let state = api.shared_state.lock().unwrap();
    assert_eq!(state.length_cache.lock().unwrap().get("010C"), Some(2));
    drop(state);
    assert_eq!(api.get_acquisition_stats().2, 6, "no clamp on NO DATA");

    session.shutdown().unwrap();
}

/// Quality-floor gate on serial (floor 3 Hz): "allowed" means the rate
/// WITH the candidate stays ≥ floor. All-fast on serial at the 149 ms
/// model RTT: 2 gauges → 1000/(2·149) = 3.36 Hz (allowed), 3 gauges →
/// 2.24 Hz (refused).
#[test]
fn gate_serial_floor() {
    let (platform, _context) = create_mock_external_platform();
    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands()); // mock: no pipes, no chunk probe run

    let sub_id = api
        .create_subscription(Some("gate".to_string()), vec!["010C".to_string()], None)
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    // 1 fast + candidate fast (010D is Fast in the curated map) = 2 fast
    // → 3.36 Hz ≥ 3 → allowed.
    let (allowed, hz, floor) = api.simulate_dashboard_add("010D");
    assert_eq!(floor, 3.0);
    assert!(allowed, "2nd fast gauge on serial projects {hz:.2} Hz");
    // Grow the selection: 2 fast + a 3rd fast candidate → 2.24 Hz → refused.
    api.add_pids_to_subscription(sub_id, vec!["010D".to_string()], None)
        .unwrap();
    let (allowed, hz, _) = api.simulate_dashboard_add("0111");
    assert!(!allowed, "3rd fast gauge on serial projects {hz:.2} Hz < 3");

    session.shutdown().unwrap();
}

/// Quality-floor gate goes LIVE with the tier: chunk-capable sessions
/// gate at 6 Hz, and a plain Mode 01 candidate is modeled at its steady
/// state (it will chunk) — so a whole extra fast gauge costs ~nothing
/// and passes where serial would refuse.
#[test]
fn gate_batched_floor_models_steady_state_chunking() {
    let (platform, _context) = create_mock_external_platform();
    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    {
        let state = api.shared_state.lock().unwrap();
        state.acquisition.lock().unwrap().set_chunk_capable(true);
        let mut cache = state.length_cache.lock().unwrap();
        for (p, l) in [("010C", 2), ("010D", 1), ("0111", 1), ("0104", 1)] {
            cache.record(p, l);
        }
    }
    let sub_id = api
        .create_subscription(
            Some("gate-chunk".to_string()),
            vec!["010C".to_string(), "010D".to_string(), "0111".to_string()],
            None,
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    // 3 fast chunked + a 4th fast candidate: still ONE chunk segment →
    // one 149 ms RTT → 6.71 Hz ≥ 6 → allowed (serial would refuse this).
    let (allowed, hz, floor) = api.simulate_dashboard_add("0104");
    assert_eq!(floor, 6.0);
    assert!(allowed, "chunked 4th gauge projects {hz:.2} Hz");
    assert!(hz > 6.5, "one chunk segment stays at the flat RTT: {hz:.2}");

    // With pipes, extra segments cost SEG (30 ms) instead of a full RTT:
    // a MEDIUM Mode 22 candidate (221E35) solos every 3rd sweep →
    // T = 149 + 30/3 = 159 ms → 6.29 Hz ≥ 6 → allowed.
    {
        let state = api.shared_state.lock().unwrap();
        state.acquisition.lock().unwrap().set_pipe_capable(true);
    }
    let (allowed, hz, _) = api.simulate_dashboard_add("221E35");
    assert!(allowed, "medium Mode 22 with pipes projects {hz:.2} Hz ≥ 6");
    // A FAST Mode 22 candidate (22F40C inherits Fast via F4xx) with
    // Mode 22 chunking unavailable rides its own pipe segment EVERY
    // sweep → 179 ms → 5.59 Hz < 6 → refused, with the honest why.
    let (allowed, hz, _) = api.simulate_dashboard_add("22F40C");
    assert!(!allowed, "fast solo Mode 22 projects {hz:.2} Hz < 6");

    session.shutdown().unwrap();
}

/// S2 e2e: the Rust transport loop against a scripted raw-writer rig —
/// the arm sequence goes out in order (STFAC → STFPA per id → STMA),
/// keepalive beats run the break/STOPPED/3E80/re-arm dance on a fast
/// cadence, frames decode between beats, and stop leaves the adapter at
/// the prompt with filters cleared before the teardown commands.
#[test]
fn s2_transport_loop_alternation_end_to_end() {
    use std::sync::mpsc;
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("1003", "50 03 00 32 01 F4");
    context.responder.set_mock_response("2C03", "6C 03");
    context
        .responder
        .set_mock_response("STPX H:7E0,D:2C01F200F40C0102,R:1", "6C 01 F2 00");
    context.responder.set_mock_response("2A0300", "6A");
    context.responder.set_mock_response("2A04", "6A");
    context.responder.set_mock_response("1001", "50 01");
    context
        .responder
        .set_mock_response("22F40C", "7E8 05 62 F4 0C 0B 4A");

    // Raw-writer rig: record every raw line; the break byte gets the
    // scripted STOPPED ack (like a real STN dropping out of monitor).
    let raw_log: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    {
        let log = Arc::clone(&raw_log);
        let plat = Arc::clone(&context.platform);
        context.platform.set_raw_writer_rust(Box::new(move |line| {
            log.lock().unwrap().push(line.clone());
            if line == " " {
                plat.sink_deliver("STOPPED".to_string());
            }
        }));
    }

    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    // Same platform facts as the s197 profile, but a 300 ms keepalive so
    // beats happen inside the test window.
    api.set_stream_profiles(
        r#"{"s197": {"name":"S197 fast-beat","target":"7E0",
                 "responseIds":["6A0","6A1"],"slotCount":10,
                 "periodicRate":"fast","keepaliveIntervalMs":300}}"#,
    );
    {
        let state = api.shared_state.lock().unwrap();
        state.length_cache.lock().unwrap().record("010C", 2);
    }
    // A MODE 01 pid on purpose: its 4-char id crashed the ingest's echo
    // formatting on hardware (pid[4..6] out of bounds) — this pins the
    // fix. source_did(010C) = F40C, so the define wire is unchanged.
    let sub_id = api
        .create_subscription(Some("s2".to_string()), vec!["010C".to_string()], None)
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    let platform_arc = {
        let w = &context.platform;
        Arc::clone(w) as Arc<dyn crate::platform::OBDPlatformInterface>
    };
    api.start_stream(Arc::clone(&platform_arc), "s197")
        .expect("stream must start");
    api.arm_stream_transport(Arc::clone(&platform_arc))
        .expect("transport loop must arm");

    // Arm sequence in exact order.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    loop {
        let log = raw_log.lock().unwrap().clone();
        if log.len() >= 5 {
            assert_eq!(
                &log[..5],
                &[
                    "STFAC".to_string(),
                    "STFPA 6A0,7FF".to_string(),
                    "STFPA 6A1,7FF".to_string(),
                    // D2: physical response id admitted so break-window
                    // STPX R:1 replies (2A04/probe/define) pass.
                    "STFPA 7E8,7FF".to_string(),
                    "STM".to_string(),
                ],
                "arm sequence"
            );
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "arm never went out: {log:?}"
        );
        std::thread::sleep(std::time::Duration::from_millis(20));
    }

    // A frame decodes into per-pid obd_data while armed — shaped like a
    // SOLO MODE 01 response (gauges route/decode it like a poll).
    context.platform.sink_deliver("6A0 00 0B 4A 4F".to_string());
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    let mut saw_frame = false;
    while std::time::Instant::now() < deadline && !saw_frame {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(100)) {
            saw_frame = msg.contains("\"command\":\"010C\"") && msg.contains("41 0C 0B 4A");
        }
    }
    assert!(saw_frame, "stream frame must decode while armed");

    // At least one full keepalive beat: break → 3E80 → re-arm.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    loop {
        let log = raw_log.lock().unwrap().clone();
        let beat = log
            .windows(3)
            .any(|w| w[0] == " " && w[1] == "STPX H:7E0,D:3E80,R:0" && w[2] == "STM");
        if beat {
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "no beat observed: {log:?}"
        );
        std::thread::sleep(std::time::Duration::from_millis(50));
    }

    // Stop: loop ends, monitor broken, FILTERS CLEARED (leftovers would
    // hide 7E8 poll responses), teardown commands out, polling resumes.
    api.stop_stream(Arc::clone(&platform_arc));
    {
        let log = raw_log.lock().unwrap().clone();
        let last_stfac = log.iter().rposition(|l| l == "STFAC").unwrap();
        assert!(
            log[last_stfac..].iter().all(|l| l != "STM"),
            "no re-arm after stop's filter clear: {log:?}"
        );
    }
    let history = context.responder.get_command_history();
    assert!(history.iter().any(|c| c == "2A04"), "{history:?}");
    assert!(history.iter().any(|c| c == "1001"), "{history:?}");
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    let mut polled_again = false;
    while std::time::Instant::now() < deadline && !polled_again {
        polled_again = context
            .responder
            .get_command_history()
            .iter()
            .rev()
            .take_while(|c| *c != "1001")
            .any(|c| c == "010C");
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    assert!(polled_again, "polling must resume after stream teardown");

    session.shutdown().unwrap();
}

/// S3: adding a gauge to a LIVE stream defines it into a fresh slot
/// during the next keepalive break — length probe (uncached pid) →
/// `2C 01` define → cumulative `2A` — with NO stream restart; the new
/// signal decodes right after.
#[test]
fn s4_subscription_driven_engage_and_pause_teardown() {
    // S4 e2e: NO direct stream calls — set the tag, start a subscription,
    // and the session engages itself: engaging event → setup chain →
    // monitor_control attach → (host acks) → arm → live event. Then
    // pause_subscription tears it down: stopping → detach → stopped,
    // polling resumed.
    use std::sync::mpsc;
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("1003", "50 03 00 32 01 F4");
    context.responder.set_mock_response("2C03", "6C 03");
    context
        .responder
        .set_mock_response("STPX H:7E0,D:2C01F200F40C0102,R:1", "6C 01 F2 00");
    context.responder.set_mock_response("2A0300", "6A");
    context.responder.set_mock_response("2A04", "6A");
    context.responder.set_mock_response("1001", "50 01");
    context
        .responder
        .set_mock_response("22F40C", "7E8 05 62 F4 0C 0B 4A");

    let raw_log: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    {
        let log = Arc::clone(&raw_log);
        let plat = Arc::clone(&context.platform);
        context.platform.set_raw_writer_rust(Box::new(move |line| {
            log.lock().unwrap().push(line.clone());
            if line == " " {
                plat.sink_deliver("STOPPED".to_string());
            }
        }));
    }

    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    // Engage guards on connection state — the mock starts Disconnected.
    context
        .platform
        .update_connection_status(crate::platform::ConnectionStatus::Connected, None);
    api.set_stream_profiles(S197_PROFILE_JSON);
    api.set_stream_tag(Some("s197".to_string()));
    {
        let state = api.shared_state.lock().unwrap();
        state.length_cache.lock().unwrap().record("22F40C", 2);
    }
    let sub_id = api
        .create_subscription(Some("s4".to_string()), vec!["22F40C".to_string()], None)
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    // The session engages on its own: wait for the attach request, ack
    // it (the "host" wiring its tap), then expect live.
    let wait_for = |needle: &str, rx: &mpsc::Receiver<String>, secs: u64| -> String {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(secs);
        while std::time::Instant::now() < deadline {
            if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(100)) {
                if msg.contains(needle) {
                    return msg;
                }
            }
        }
        panic!("never saw {needle:?}");
    };
    wait_for("\"state\":\"engaging\"", &rx, 10);
    wait_for("\"action\":\"attach\"", &rx, 10);
    api.monitor_ready();
    let live = wait_for("\"state\":\"live\"", &rx, 10);
    assert!(live.contains("\"slotsUsed\":1"), "{live}");

    // Streamed frame decodes to poll-shaped data.
    context.platform.sink_deliver("6A0 00 0B 4A".to_string());
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    let mut saw = false;
    while std::time::Instant::now() < deadline && !saw {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(100)) {
            saw = msg.contains("\"command\":\"22F40C\"");
        }
    }
    assert!(saw, "streamed frame must decode");

    // Pause the subscription → managed teardown with detach handshake.
    api.pause_subscription(sub_id).unwrap();
    wait_for("\"state\":\"stopping\"", &rx, 5);
    wait_for("\"action\":\"detach\"", &rx, 5);
    api.monitor_ready();
    wait_for("\"state\":\"stopped\"", &rx, 10);
    {
        let state = api.shared_state.lock().unwrap();
        assert!(
            state.stream_state.lock().unwrap().is_none(),
            "stream cleared"
        );
    }
    let history = context.responder.get_command_history();
    assert!(
        history.iter().any(|c| c == "2A04"),
        "teardown ran: {history:?}"
    );

    session.shutdown().unwrap();
}

#[test]
fn s3_add_then_remove_same_gap_nets_nothing() {
    // GS5/S3: an add CANCELLED by a remove before the beat fires must
    // send NO define — and the beats keep running (stream unharmed).
    use std::sync::mpsc;
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("1003", "50 03 00 32 01 F4");
    context.responder.set_mock_response("2C03", "6C 03");
    context
        .responder
        .set_mock_response("STPX H:7E0,D:2C01F200F40C0102,R:1", "6C 01 F2 00");
    context.responder.set_mock_response("2A0300", "6A");
    context.responder.set_mock_response("2A04", "6A");
    context.responder.set_mock_response("1001", "50 01");
    context
        .responder
        .set_mock_response("22F40C", "7E8 05 62 F4 0C 0B 4A");

    let raw_log: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    {
        let log = Arc::clone(&raw_log);
        let plat = Arc::clone(&context.platform);
        context.platform.set_raw_writer_rust(Box::new(move |line| {
            log.lock().unwrap().push(line.clone());
            if line == " " {
                plat.sink_deliver("STOPPED".to_string());
            } else if line.contains("D:2A04") {
                plat.sink_deliver("7E8 02 6A 04".to_string());
            }
        }));
    }

    let (tx, _rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    api.set_stream_profiles(
        r#"{"s197": {"name":"S197 fast-beat","target":"7E0",
                 "responseIds":["6A0","6A1"],"slotCount":10,
                 "periodicRate":"fast","keepaliveIntervalMs":300}}"#,
    );
    {
        let state = api.shared_state.lock().unwrap();
        state.length_cache.lock().unwrap().record("010C", 2);
        state.length_cache.lock().unwrap().record("22F405", 1);
    }
    let sub_id = api
        .create_subscription(Some("s3".to_string()), vec!["010C".to_string()], None)
        .unwrap();
    api.start_subscription(sub_id).unwrap();
    let platform_arc = {
        let w = &context.platform;
        Arc::clone(w) as Arc<dyn crate::platform::OBDPlatformInterface>
    };
    api.start_stream(Arc::clone(&platform_arc), "s197")
        .expect("stream must start");
    api.arm_stream_transport(Arc::clone(&platform_arc))
        .expect("transport loop must arm");

    // Add then IMMEDIATELY remove — the remove cancels the queued add.
    assert!(api.stream_change_signal("22F405", true));
    assert!(api.stream_change_signal("22F405", false));

    // Let a few beats run, then assert: beats happened, no define went
    // out for the cancelled signal (nor a 2A04 — no defines pending).
    std::thread::sleep(std::time::Duration::from_millis(1200));
    let log = raw_log.lock().unwrap().clone();
    let beats = log.iter().filter(|l| l.contains("D:3E80")).count();
    assert!(beats >= 2, "beats must keep running, saw {beats}: {log:?}");
    assert!(
        !log.iter().any(|l| l.contains("F405")),
        "cancelled add must not touch the wire: {log:?}"
    );

    api.stop_stream(Arc::clone(&platform_arc));
    session.shutdown().unwrap();
}

/// S3: stopping with an UNPROCESSED pending add must tear down cleanly —
/// no define after the stop, teardown completes, polling resumes.
#[test]
fn s3_stop_with_pending_add_is_clean() {
    use std::sync::mpsc;
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("1003", "50 03 00 32 01 F4");
    context.responder.set_mock_response("2C03", "6C 03");
    context
        .responder
        .set_mock_response("STPX H:7E0,D:2C01F200F40C0102,R:1", "6C 01 F2 00");
    context.responder.set_mock_response("2A0300", "6A");
    context.responder.set_mock_response("2A04", "6A");
    context.responder.set_mock_response("1001", "50 01");
    context
        .responder
        .set_mock_response("22F40C", "7E8 05 62 F4 0C 0B 4A");

    let raw_log: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    {
        let log = Arc::clone(&raw_log);
        let plat = Arc::clone(&context.platform);
        context.platform.set_raw_writer_rust(Box::new(move |line| {
            log.lock().unwrap().push(line.clone());
            if line == " " {
                plat.sink_deliver("STOPPED".to_string());
            }
        }));
    }

    let (tx, _rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    // LONG keepalive: the beat never fires before the stop — the pending
    // add must die with the stream, not execute during teardown.
    api.set_stream_profiles(
        r#"{"s197": {"name":"S197 slow-beat","target":"7E0",
                 "responseIds":["6A0","6A1"],"slotCount":10,
                 "periodicRate":"fast","keepaliveIntervalMs":30000}}"#,
    );
    {
        let state = api.shared_state.lock().unwrap();
        state.length_cache.lock().unwrap().record("010C", 2);
        state.length_cache.lock().unwrap().record("22F405", 1);
    }
    let sub_id = api
        .create_subscription(Some("s3".to_string()), vec!["010C".to_string()], None)
        .unwrap();
    api.start_subscription(sub_id).unwrap();
    let platform_arc = {
        let w = &context.platform;
        Arc::clone(w) as Arc<dyn crate::platform::OBDPlatformInterface>
    };
    api.start_stream(Arc::clone(&platform_arc), "s197")
        .expect("stream must start");
    api.arm_stream_transport(Arc::clone(&platform_arc))
        .expect("transport loop must arm");

    assert!(
        api.stream_change_signal("22F405", true),
        "queued while live"
    );
    api.stop_stream(Arc::clone(&platform_arc));

    let log = raw_log.lock().unwrap().clone();
    assert!(
        !log.iter().any(|l| l.contains("F405")),
        "pending add must not execute during/after stop: {log:?}"
    );
    let history = context.responder.get_command_history();
    assert!(
        history.iter().any(|c| c == "2A04"),
        "teardown 2A04: {history:?}"
    );
    assert!(
        history.iter().any(|c| c == "1001"),
        "teardown 1001: {history:?}"
    );
    // Stream state cleared → a second stop is a no-op and signals refuse.
    assert!(!api.stream_change_signal("22F405", true), "stream is gone");

    api.stop_stream(Arc::clone(&platform_arc));
    session.shutdown().unwrap();
}

#[test]
fn s3_mid_stream_add_defines_in_the_break() {
    use std::sync::mpsc;
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("1003", "50 03 00 32 01 F4");
    context.responder.set_mock_response("2C03", "6C 03");
    context
        .responder
        .set_mock_response("STPX H:7E0,D:2C01F200F40C0102,R:1", "6C 01 F2 00");
    context.responder.set_mock_response("2A0300", "6A");
    context.responder.set_mock_response("2A04", "6A");
    context.responder.set_mock_response("1001", "50 01");
    context
        .responder
        .set_mock_response("22F40C", "7E8 05 62 F4 0C 0B 4A");

    // Raw-writer rig: record lines; script the break-window exchanges —
    // STOPPED for the break, a `62` for the length probe, a `6C` for the
    // mid-stream define.
    let raw_log: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    {
        let log = Arc::clone(&raw_log);
        let plat = Arc::clone(&context.platform);
        context.platform.set_raw_writer_rust(Box::new(move |line| {
            log.lock().unwrap().push(line.clone());
            if line == " " {
                plat.sink_deliver("STOPPED".to_string());
            } else if line == "STPX H:7E0,D:22F405,R:1" {
                // Length probe: PCI 04 → 4 bytes total − 3 echo = len 1.
                plat.sink_deliver("7E8 04 62 F4 05 7A".to_string());
            } else if line == "STPX H:7E0,D:2C01F200F4050101,R:1" {
                plat.sink_deliver("7E8 04 6C 01 F2 00".to_string());
            } else if line == "STPX H:7E0,D:2A04,R:1" {
                plat.sink_deliver("7E8 02 6A 04".to_string());
            }
        }));
    }

    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    api.set_stream_profiles(
        r#"{"s197": {"name":"S197 fast-beat","target":"7E0",
                 "responseIds":["6A0","6A1"],"slotCount":10,
                 "periodicRate":"fast","keepaliveIntervalMs":300}}"#,
    );
    {
        let state = api.shared_state.lock().unwrap();
        state.length_cache.lock().unwrap().record("010C", 2);
    }
    let sub_id = api
        .create_subscription(Some("s3".to_string()), vec!["010C".to_string()], None)
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    let platform_arc = {
        let w = &context.platform;
        Arc::clone(w) as Arc<dyn crate::platform::OBDPlatformInterface>
    };
    api.start_stream(Arc::clone(&platform_arc), "s197")
        .expect("stream must start");
    api.arm_stream_transport(Arc::clone(&platform_arc))
        .expect("transport loop must arm");

    // Queue the mid-stream add (an UNCACHED Mode 22 pid → probes first).
    assert!(api.stream_change_signal("22F405", true), "stream is live");

    // The break-window work happens on the next beat: probe → STOP the
    // scheduler (2A04 — a define against a RUNNING scheduler is never
    // acked and kills the broadcast, Mustang 23:10 bench) → define →
    // re-arm. First-fit APPENDS into slot 0's free bytes (F40C fills
    // 2 of 7) — position byte 01 (whole source), landing at offset 2.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
    loop {
        let log = raw_log.lock().unwrap().clone();
        let stop_at = log.iter().position(|l| l == "STPX H:7E0,D:2A04,R:1");
        let define_at = log
            .iter()
            .position(|l| l == "STPX H:7E0,D:2C01F200F4050101,R:1");
        let probed = log.iter().any(|l| l == "STPX H:7E0,D:22F405,R:1");
        let restarted = log.iter().any(|l| l == "STPX H:7E0,D:2A0300,R:0");
        if let (true, Some(st), Some(df), true) = (probed, stop_at, define_at, restarted) {
            assert!(st < df, "scheduler must STOP before the define");
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "break-window add incomplete: {log:?}"
        );
        std::thread::sleep(std::time::Duration::from_millis(50));
    }

    // The new signal decodes from its appended bytes (offset 2, after
    // F40C's word).
    context
        .platform
        .sink_deliver("6A0 00 0B 4A 7A 00 00 00 00".to_string());
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    let mut saw = false;
    while std::time::Instant::now() < deadline && !saw {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(100)) {
            saw = msg.contains("\"command\":\"22F405\"") && msg.contains("62 F4 05 7A");
        }
    }
    assert!(saw, "mid-stream-added signal must decode");

    api.stop_stream(Arc::clone(&platform_arc));
    session.shutdown().unwrap();
}

/// S2 bench regression (Mustang round 3): the PCM whitelists definable
/// sources — F40C acks, F40D gets `7F 2C 31`. A refused record must drop
/// ONLY that signal: the stream starts with the survivor, and the start
/// command lists only surviving slots.
#[test]
fn s2_refused_define_drops_signal_not_stream() {
    use std::sync::mpsc;
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("1003", "50 03 00 32 01 F4");
    context.responder.set_mock_response("2C03", "6C 03");
    context
        .responder
        .set_mock_response("STPX H:7E0,D:2C01F200F40C0102,R:1", "6C 01 F2 00");
    // Speed's define refused by the vehicle (source not definable).
    context
        .responder
        .set_mock_response("STPX H:7E0,D:2C01F200F40D0101,R:1", "7E8 03 7F 2C 31");
    context.responder.set_mock_response("2A0300", "6A");
    context.responder.set_mock_response("2A04", "6A");
    context.responder.set_mock_response("1001", "50 01");
    context
        .responder
        .set_mock_response("22F40C", "7E8 05 62 F4 0C 0B 4A");
    context
        .responder
        .set_mock_response("22F40D", "7E8 04 62 F4 0D 45");

    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    api.set_stream_profiles(S197_PROFILE_JSON);
    {
        let state = api.shared_state.lock().unwrap();
        let mut cache = state.length_cache.lock().unwrap();
        cache.record("22F40C", 2);
        cache.record("22F40D", 1);
    }
    let sub_id = api
        .create_subscription(
            Some("partial".to_string()),
            vec!["22F40C".to_string(), "22F40D".to_string()],
            None,
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    let platform_arc = {
        let w = &context.platform;
        Arc::clone(w) as Arc<dyn crate::platform::OBDPlatformInterface>
    };
    let info = api
        .start_stream(Arc::clone(&platform_arc), "s197")
        .expect("stream must start with the surviving signal");
    assert_eq!(info.slots_used, 1);

    // The survivor decodes; the refused signal never appears.
    context.platform.sink_deliver("6A0 00 0B 4A 4F".to_string());
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    let mut saw_rpm = false;
    while std::time::Instant::now() < deadline && !saw_rpm {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(100)) {
            assert!(
                !msg.contains("\"command\":\"22F40D\""),
                "refused signal must not decode: {msg}"
            );
            saw_rpm = msg.contains("\"command\":\"22F40C\"") && msg.contains("62 F4 0C");
        }
    }
    assert!(saw_rpm, "surviving signal streams");

    let history = context.responder.get_command_history();
    assert!(
        history.iter().any(|c| c == "2A0300"),
        "start lists only the surviving slot: {history:?}"
    );

    api.stop_stream(Arc::clone(&platform_arc));
    session.shutdown().unwrap();
}

const S197_PROFILE_JSON: &str = r#"{
      "s197": {
        "name": "Ford S197 Mustang 2011-2014",
        "target": "7E0",
        "responseIds": ["6A0", "6A1"],
        "slotCount": 10,
        "periodicRate": "fast",
        "keepaliveIntervalMs": 4000
      }
    }"#;

/// S1 e2e: mock UDS responder acks the whole setup chain (10 03 → 2C 03
/// → 2C 01 defines → 2A 03), then pushes captured-shape periodic frames
/// at ~25 Hz through monitor mode — per-pid obd_data must arrive at
/// >10 Hz, polling is paused while streaming and resumes on stop.
#[test]
fn s1_mock_stream_end_to_end_over_10hz() {
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("1003", "50 03 00 32 01 F4");
    context.responder.set_mock_response("2C03", "6C 03");
    context
        .responder
        .set_mock_response("STPX H:7E0,D:2C01F200F40C0102,R:1", "6C 01 F2 00");
    context.responder.set_mock_response("2A0300", "6A");
    context.responder.set_mock_response("2A04", "6A");
    context.responder.set_mock_response("1001", "50 01");
    context
        .responder
        .set_mock_response("22F40C", "7E8 05 62 F4 0C 0B 4A");

    use std::sync::mpsc;
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    api.set_stream_profiles(S197_PROFILE_JSON);
    {
        let state = api.shared_state.lock().unwrap();
        state.length_cache.lock().unwrap().record("22F40C", 2);
    }

    let sub_id = api
        .create_subscription(Some("stream".to_string()), vec!["22F40C".to_string()], None)
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    let platform_arc = {
        let w = &context.platform;
        Arc::clone(w) as Arc<dyn crate::platform::OBDPlatformInterface>
    };
    let info = api
        .start_stream(Arc::clone(&platform_arc), "s197")
        .expect("stream must start against the capable mock");
    // S2 contract: the host's monitor loop gets what it needs.
    assert_eq!(
        info.response_ids,
        vec!["6A0".to_string(), "6A1".to_string()]
    );
    assert_eq!(info.keepalive_interval_ms, 4000);

    // The "adapter": push RPM frames at 25 Hz for ~1 s (sweep values).
    let pusher = {
        let p = Arc::clone(&context.platform);
        std::thread::spawn(move || {
            for i in 0..25u32 {
                let rpm_word = 0x0B07 + i * 8; // 707 → ~745 rpm sweep
                p.sink_deliver(format!(
                    "6A0 00 {:02X} {:02X} 4F",
                    (rpm_word >> 8) & 0xFF,
                    rpm_word & 0xFF
                ));
                std::thread::sleep(std::time::Duration::from_millis(40));
            }
        })
    };

    // Count per-pid obd_data updates for ~1 s of stream time.
    let started = std::time::Instant::now();
    let mut updates = 0u32;
    while started.elapsed() < std::time::Duration::from_millis(1300) {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(100)) {
            if msg.contains("\"command\":\"22F40C\"") && msg.contains("62 F4 0C") {
                updates += 1;
            }
        }
    }
    pusher.join().unwrap();
    let hz = updates as f64 / 1.3;
    assert!(
        hz > 10.0,
        "stream must beat polling: {updates} updates = {hz:.1} Hz"
    );

    // While streaming, the poll subscription is PAUSED — the mock saw the
    // setup chain but no 22F40C polling wires after the stream started.
    let history = context.responder.get_command_history();
    // PHYSICAL addressing: the setup chain targets the profile ECU and
    // the ATSH lands BEFORE the session probe (Mustang bench: broadcast
    // `2C` is silently ignored — the wizard's original failure).
    let atsh_pos = history.iter().position(|c| c == "ATSH7E0");
    let probe_pos = history.iter().position(|c| c == "1003");
    assert!(
        atsh_pos.is_some_and(|a| probe_pos.is_some_and(|p| a < p)),
        "ATSH7E0 must precede 1003: {history:?}"
    );
    assert!(history.iter().any(|c| c == "1003"));
    assert!(
        history
            .iter()
            .any(|c| c == "STPX H:7E0,D:2C01F200F40C0102,R:1"),
        "define goes out via STPX (>7-byte TX needs STN segmentation)"
    );
    assert!(history.iter().any(|c| c == "2A0300"));

    // Teardown resumes polling.
    api.stop_stream(Arc::clone(&platform_arc));
    let history = context.responder.get_command_history();
    assert!(history.iter().any(|c| c == "2A04"), "{history:?}");
    assert!(history.iter().any(|c| c == "1001"), "{history:?}");
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    let mut polled_again = false;
    while std::time::Instant::now() < deadline && !polled_again {
        polled_again = context
            .responder
            .get_command_history()
            .iter()
            .rev()
            .take_while(|c| *c != "1001")
            .any(|c| c == "22F40C");
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    assert!(polled_again, "polling must resume after stream teardown");

    session.shutdown().unwrap();
}

/// S1 negative: the vehicle refuses dynamic defines (`7F 2C 11`) — the
/// stream never starts, nothing is paused, polling continues untouched.
#[test]
fn s1_stream_7f2c_negative_stays_poll() {
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .set_mock_response("1003", "50 03 00 32 01 F4");
    context.responder.set_mock_response("2C03", "7F 2C 11");
    context
        .responder
        .set_mock_response("22F40C", "7E8 05 62 F4 0C 0B 4A");

    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    api.set_stream_profiles(S197_PROFILE_JSON);
    {
        let state = api.shared_state.lock().unwrap();
        state.length_cache.lock().unwrap().record("22F40C", 2);
    }
    let sub_id = api
        .create_subscription(
            Some("no-stream".to_string()),
            vec!["22F40C".to_string()],
            None,
        )
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    let platform_arc =
        Arc::clone(&context.platform) as Arc<dyn crate::platform::OBDPlatformInterface>;
    let err = api.start_stream(platform_arc, "s197").unwrap_err();
    assert!(err.contains("2C 03 refused"), "{err}");

    // Polling keeps flowing (subscription was never paused).
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    let mut polls = 0;
    while std::time::Instant::now() < deadline && polls < 3 {
        polls = context
            .responder
            .get_command_history()
            .iter()
            .filter(|c| *c == "22F40C")
            .count();
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    assert!(polls >= 3, "polling must continue after a refused probe");

    // An unknown tag degrades the same way.
    let platform_arc =
        Arc::clone(&context.platform) as Arc<dyn crate::platform::OBDPlatformInterface>;
    let err = api.start_stream(platform_arc, "s550").unwrap_err();
    assert!(err.contains("no stream profile"), "{err}");

    session.shutdown().unwrap();
}

/// A healthy ATE0 (answers OK) must NOT be resent.
#[test]
fn init_does_not_retry_healthy_ate0() {
    let (platform, context) = create_mock_external_platform();
    // Mock default: ATE0 → "OK".

    let session = OBDSessionManager::new(platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());

    let ate0_sends = context
        .responder
        .get_command_history()
        .iter()
        .filter(|c| c.eq_ignore_ascii_case("ATE0"))
        .count();
    assert_eq!(ate0_sends, 1, "healthy ATE0 must be sent exactly once");

    session.shutdown().unwrap();
}

#[test]
fn test_api_handle_operations() {
    let (platform, _context) = create_mock_external_platform();
    let config = OBDSessionConfig::default();

    let session = OBDSessionManager::new(platform, config, |_response| {
        // Callback for responses
    })
    .unwrap();

    let api = session.api_handle();

    // Test command sending
    api.send_command("ATZ", 1000).unwrap();

    // Test subscription creation
    let subscription_id = api
        .create_subscription(None, vec!["010C".to_string()], None)
        .unwrap();
    assert!(!subscription_id.to_string().is_empty());

    // Test subscription listing
    let subscriptions = api.list_subscriptions();
    assert_eq!(subscriptions.len(), 1);
    assert_eq!(subscriptions[0].id, subscription_id);

    // Shutdown
    session.shutdown().unwrap();
}

/// P1: user cancel must invalidate the in-flight connect attempt (so the
/// flow's step-boundary guards abort it) and emit the terminal
/// DISCONNECTED phase so the UI resets.
#[test]
fn cancel_invalidates_connect_attempt_and_emits_disconnected() {
    use std::sync::mpsc;

    let (platform, _context) = create_mock_external_platform();
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();

    let g1 = api.begin_connect_attempt();
    assert!(api.is_connect_current(g1));

    api.cancel_connect_attempt();
    assert!(
        !api.is_connect_current(g1),
        "cancel must invalidate the in-flight attempt"
    );

    // The cancel emits a connection_progress with the disconnected phase.
    let mut saw_disconnected = false;
    while let Ok(msg) = rx.recv_timeout(std::time::Duration::from_secs(2)) {
        if msg.contains("connection_progress") && msg.contains("disconnected") {
            saw_disconnected = true;
            break;
        }
    }
    assert!(saw_disconnected, "cancel must emit the DISCONNECTED phase");

    // A fresh attempt (e.g. the user clicks Connect again) owns the state.
    let g2 = api.begin_connect_attempt();
    assert!(api.is_connect_current(g2));
    assert!(!api.is_connect_current(g1));

    session.shutdown().unwrap();
}

/// One polling chain, not one per subscription: starting a second
/// subscription while the first still polls must NOT queue another AT
/// kick — each kick births an independent completion-driven chain and the
/// pipeline sits permanently 2 deep (two in-flight PIDs in Session Stats).
#[test]
fn second_subscription_start_does_not_double_the_pipeline() {
    use std::time::Duration;

    let (platform, context) = create_mock_external_platform();
    let session =
        OBDSessionManager::new(Arc::clone(&platform) as _, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();

    let sub1 = api
        .create_subscription(
            Some("dashboard".into()),
            vec!["0104".into(), "0105".into()],
            None,
        )
        .unwrap();
    api.start_subscription(sub1).unwrap();
    std::thread::sleep(Duration::from_millis(300)); // chain running

    let sub2 = api
        .create_subscription(Some("enable-tests".into()), vec!["010C".into()], None)
        .unwrap();
    api.start_subscription(sub2).unwrap();
    std::thread::sleep(Duration::from_millis(300));

    // Exactly one AT kick — the live chain absorbed the second subscription.
    let kicks = context
        .responder
        .get_command_history()
        .iter()
        .filter(|c| c.as_str() == "AT")
        .count();
    assert_eq!(kicks, 1, "second start must not queue another AT kick");

    // And the second subscription's PID is being polled by that one chain.
    assert!(
        context
            .responder
            .get_command_history()
            .iter()
            .any(|c| c == "010C"),
        "the live chain picks up the new subscription's PIDs"
    );

    session.shutdown().unwrap();
}

/// The lone chain dies when everything goes quiet (all subs paused →
/// get_next_command returns None, nothing re-queues), and the NEXT
/// start_subscription finds the processor idle and kicks a fresh chain.
/// Without the re-kick, resume would never poll again.
#[test]
fn chain_dies_when_quiet_and_restart_kicks_a_fresh_one() {
    use std::time::Duration;

    let (platform, context) = create_mock_external_platform();
    let session =
        OBDSessionManager::new(Arc::clone(&platform) as _, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();

    let sub = api
        .create_subscription(Some("dashboard".into()), vec!["0104".into()], None)
        .unwrap();
    api.start_subscription(sub).unwrap();
    std::thread::sleep(Duration::from_millis(300)); // chain running
    assert!(
        context
            .responder
            .get_command_history()
            .iter()
            .any(|c| c == "0104"),
        "chain polls while the subscription is active"
    );

    // Quiet: pause the only subscription. The in-flight command completes,
    // get_next_command returns None, and the chain starves out.
    api.pause_subscription(sub).unwrap();
    std::thread::sleep(Duration::from_millis(200)); // let the in-flight drain
    let after_pause = context.responder.get_command_history().len();
    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(
        context.responder.get_command_history().len(),
        after_pause,
        "no commands may be issued while everything is quiet — the chain is dead"
    );

    // Restart: processor is idle, so start_subscription kicks a FRESH
    // chain (second AT) and polling resumes.
    api.start_subscription(sub).unwrap();
    std::thread::sleep(Duration::from_millis(300));
    let history = context.responder.get_command_history();
    let kicks = history.iter().filter(|c| c.as_str() == "AT").count();
    assert_eq!(kicks, 2, "restart after quiet must kick a fresh chain");
    assert!(
        history.len() > after_pause + 1,
        "polling resumed after the restart"
    );

    session.shutdown().unwrap();
}

/// FB1: removing every PID starves the chain; ADDING a PID back must
/// revive it (add_pids kicks when the processor is idle). Without the
/// kick the dashboard stayed dead after remove-all → re-add.
#[test]
fn add_pids_to_quiet_subscription_revives_polling() {
    use std::time::Duration;

    let (platform, context) = create_mock_external_platform();
    let session =
        OBDSessionManager::new(Arc::clone(&platform) as _, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();

    let sub = api
        .create_subscription(Some("dashboard".into()), vec!["0104".into()], None)
        .unwrap();
    api.start_subscription(sub).unwrap();
    std::thread::sleep(Duration::from_millis(300));
    assert!(context
        .responder
        .get_command_history()
        .iter()
        .any(|c| c == "0104"));

    // Remove-all: get_next_command returns None → chain starves out.
    api.remove_pids_from_subscription(sub, vec!["0104".into()])
        .unwrap();
    std::thread::sleep(Duration::from_millis(200)); // drain in-flight
    let quiet_len = context.responder.get_command_history().len();
    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(
        context.responder.get_command_history().len(),
        quiet_len,
        "chain must be dead after all PIDs are removed"
    );

    // Re-add: must kick a fresh chain and resume polling.
    api.add_pids_to_subscription(sub, vec!["0105".into()], None)
        .unwrap();
    std::thread::sleep(Duration::from_millis(300));
    let history = context.responder.get_command_history();
    assert!(
        history.iter().skip(quiet_len).any(|c| c == "0105"),
        "polling must resume after PIDs are re-added"
    );
    let kicks = history.iter().filter(|c| c.as_str() == "AT").count();
    assert_eq!(kicks, 2, "start + revive = exactly two kicks");

    session.shutdown().unwrap();
}

/// FB1 guard: adding PIDs to a PAUSED subscription must NOT kick — a
/// paused session (e.g. sink/monitor mode) has to stay quiet.
#[test]
fn add_pids_to_paused_subscription_stays_quiet() {
    use std::time::Duration;

    let (platform, context) = create_mock_external_platform();
    let session =
        OBDSessionManager::new(Arc::clone(&platform) as _, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();

    let sub = api
        .create_subscription(Some("dashboard".into()), vec!["0104".into()], None)
        .unwrap();
    api.start_subscription(sub).unwrap();
    std::thread::sleep(Duration::from_millis(200));
    api.pause_subscription(sub).unwrap();
    std::thread::sleep(Duration::from_millis(200)); // chain starves

    let quiet_len = context.responder.get_command_history().len();
    api.add_pids_to_subscription(sub, vec!["0105".into()], None)
        .unwrap();
    std::thread::sleep(Duration::from_millis(300));
    assert_eq!(
        context.responder.get_command_history().len(),
        quiet_len,
        "adding to a paused subscription must not send anything"
    );

    session.shutdown().unwrap();
}

/// Graceful drain: disconnect lets a fast in-flight command finish (the
/// adapter ends at its prompt), and DISCONNECTING precedes the terminal
/// DISCONNECTED.
#[test]
fn disconnect_and_drain_completes_fast_inflight_command() {
    use std::sync::mpsc;
    use std::time::Duration;

    let (platform, _context) = create_mock_external_platform();
    let (tx, rx) = mpsc::channel::<String>();
    let session =
        OBDSessionManager::new(Arc::clone(&platform) as _, config_for_test(), move |msg| {
            let _ = tx.send(msg);
        })
        .unwrap();
    let api = session.api_handle();

    api.send_command("010C", 5000).unwrap();
    std::thread::sleep(Duration::from_millis(50)); // let it go in-flight

    api.disconnect_and_drain(platform);

    let mut saw_disconnecting = false;
    let mut saw_disconnected = false;
    while let Ok(msg) = rx.recv_timeout(Duration::from_secs(3)) {
        if msg.contains("\"disconnecting\"") {
            saw_disconnecting = true;
        }
        if msg.contains("\"disconnected\"") {
            saw_disconnected = true;
            break;
        }
    }
    assert!(saw_disconnecting, "drain must emit DISCONNECTING first");
    assert!(saw_disconnected, "drain must end in DISCONNECTED");
    // The fast command was drained, not abandoned.
    let stats = api.get_command_processor_stats().unwrap();
    assert_eq!(
        stats.in_progress_commands, 0,
        "processor must be idle after drain"
    );
    assert_eq!(
        stats.completed_commands, 1,
        "the in-flight command completed, not abandoned"
    );

    session.shutdown().unwrap();
}

/// Bounded drain: a STUCK in-flight command (dead adapter — its response
/// never comes) must not hang the disconnect. The backstop abandons it
/// and DISCONNECTED still arrives within the bound.
#[test]
fn disconnect_and_drain_is_bounded_with_stuck_command() {
    use std::sync::mpsc;
    use std::time::{Duration, Instant};

    // Response delayed far beyond the drain bound = stuck command.
    let (platform, _context) =
        crate::external_platform::create_mock_external_platform_with_delay(Duration::from_secs(60));
    let (tx, rx) = mpsc::channel::<String>();
    let session =
        OBDSessionManager::new(Arc::clone(&platform) as _, config_for_test(), move |msg| {
            let _ = tx.send(msg);
        })
        .unwrap();
    let api = session.api_handle();

    api.send_command("010C", 60_000).unwrap();
    std::thread::sleep(Duration::from_millis(100)); // let it go in-flight

    let start = Instant::now();
    api.disconnect_and_drain(platform);

    let mut saw_disconnected = false;
    while let Ok(msg) = rx.recv_timeout(Duration::from_secs(5)) {
        if msg.contains("\"disconnected\"") {
            saw_disconnected = true;
            break;
        }
    }
    assert!(
        saw_disconnected,
        "stuck command must not prevent DISCONNECTED"
    );
    assert!(
        start.elapsed() < Duration::from_secs(4),
        "drain must stay bounded, took {:?}",
        start.elapsed()
    );

    // The processor is usable again: a fresh attempt's command goes through.
    api.begin_connect_attempt();
    assert!(api.send_command("ATE0", 1000).is_ok());

    session.shutdown().unwrap();
}

/// M7: sink works with ANY platform that implements enter_monitor_mode —
/// proven with a minimal canned platform (no ExternalPlatform involved).
#[test]
fn sink_generic_path_with_canned_native_platform() {
    use crate::platform::{
        ConnectionCallback, ConnectionStatus, OBDDataCallback, OBDPlatformInterface,
    };

    /// A native-style platform: enter_monitor_mode streams canned frames
    /// synchronously through the session-provided closure.
    struct CannedMonitorPlatform;
    impl OBDPlatformInterface for CannedMonitorPlatform {
        fn send_command(&self, _c: &str, _t: u32) {}
        fn set_completion_callback(&self, _cb: Option<Box<dyn Fn(String) + Send + Sync>>) {}
        fn set_data_callback(&self, _cb: Option<OBDDataCallback>) {}
        fn set_connection_callback(&self, _cb: Option<ConnectionCallback>) {}
        fn is_connected(&self) -> bool {
            true
        }
        fn connection_status(&self) -> ConnectionStatus {
            ConnectionStatus::Connected
        }
        fn enter_monitor_mode(
            &self,
            _filter: Option<&str>,
            on_line: Box<dyn Fn(String) + Send + Sync>,
        ) -> Result<(), String> {
            for frame in [
                "18DAF110 06 41 00 BF FE B9 93",
                "7E8 03 41 0C 1A F8",
                "7E8 03 41 0D 00",
            ] {
                on_line(frame.to_string());
            }
            Ok(())
        }
    }

    let (ext_platform, _context) = create_mock_external_platform();
    let session = OBDSessionManager::new(ext_platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();

    let native: Arc<dyn crate::platform::OBDPlatformInterface> = Arc::new(CannedMonitorPlatform);
    api.sink_start(Arc::clone(&native), None, None)
        .expect("canned platform supports monitoring");

    let stats = api.sink_stats();
    assert!(stats.active);
    assert_eq!(stats.received_total, 3);

    let frames = api.sink_read(10);
    assert_eq!(frames.len(), 3);
    assert_eq!(frames[0].raw, "18DAF110 06 41 00 BF FE B9 93");

    api.sink_stop(native);
    assert!(!api.sink_stats().active);

    session.shutdown().unwrap();
}

/// LH4 e2e for the STN periodic engine (there was no Rust gate for SP —
/// only the Swift StnPeriodicIntegrationTests): capability set → a
/// subscription engages SP on its own → STCMM 1 + STPPMA install (RPM is
/// Fast tier → 25 ms) →
/// attach → the PACED arm (STPO, STFAC, STFPA 7E8,7FF, STM) → live → a
/// monitor frame decodes to poll-shaped data → pause tears down: break
/// (space → STOPPED, STFAC), detach, verified STPPMC + pinned reopen
/// (STPR → STP 33 → 0100), polling resumed.
/// Ladder attempt budget (David 2026-08-30): a car that refuses `10 03`
/// costs exactly TWO UDS attempts (3 s beats), then the adapter-periodic
/// tier engages — not five rounds of polling-speed dashboard. Each
/// `failed` event is tagged attempt/attempts.
#[test]
fn ladder_uds_refused_twice_then_adapter_periodic() {
    use std::sync::mpsc;
    let (platform, context) = create_mock_external_platform();
    context.responder.set_mock_response("1003", "7F 10 22");
    context
        .responder
        .set_mock_response("010C", "7E8 04 41 0C 1A F8");
    context.responder.set_mock_response("STCMM 1", "OK");
    context
        .responder
        .set_mock_response("STPPMA 25, 7E0, 010C", "1");
    context.responder.set_mock_response("AT", "OK");
    context.responder.set_mock_response("STPPMC", "OK");
    context.responder.set_mock_response("STPR", "A6 [AT]");
    context.responder.set_mock_response("STP 33", "OK");
    context
        .responder
        .set_mock_response("0100", "7E8 06 41 00 BE 3F A8 13");
    {
        let plat = Arc::clone(&context.platform);
        context.platform.set_raw_writer_rust(Box::new(move |line| {
            if line == " " {
                plat.sink_deliver("STOPPED".to_string());
            }
        }));
    }
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    context
        .platform
        .update_connection_status(crate::platform::ConnectionStatus::Connected, None);
    api.set_stn_periodic_capable(true);
    api.set_stream_profiles(S197_PROFILE_JSON);
    api.set_stream_tag(Some("s197".to_string()));
    {
        let state = api.shared_state.lock().unwrap();
        state.length_cache.lock().unwrap().record("010C", 2);
    }
    let sub_id = api
        .create_subscription(Some("uds-fail".to_string()), vec!["010C".to_string()], None)
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    let t0 = std::time::Instant::now();
    let mut failed: Vec<String> = Vec::new();
    let mut sp_engaging = false;
    let deadline = t0 + std::time::Duration::from_secs(30);
    while std::time::Instant::now() < deadline && !sp_engaging {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(100)) {
            if msg.contains("\"state\":\"failed\"") {
                failed.push(msg.clone());
            }
            if msg.contains("\"state\":\"engaging\"") && msg.contains("\"plugin\":\"stn-periodic\"")
            {
                sp_engaging = true;
            }
        }
    }
    assert!(
        sp_engaging,
        "adapter-periodic must engage after the UDS attempts: failed={failed:?}"
    );
    assert_eq!(
        failed.len(),
        2,
        "exactly two UDS attempts before the fallback: {failed:?}"
    );
    assert!(
        failed[0].contains("\"attempt\":1") && failed[0].contains("\"attempts\":2"),
        "{}",
        failed[0]
    );
    assert!(
        failed[1].contains("\"attempt\":2") && failed[1].contains("\"attempts\":2"),
        "{}",
        failed[1]
    );
    assert!(
        t0.elapsed() < std::time::Duration::from_secs(12),
        "fallback must come within a few beats, took {:?}",
        t0.elapsed()
    );

    // Finish the SP arm so the session is live on the fallback tier.
    let wait_for = |needle: &str, rx: &mpsc::Receiver<String>, secs: u64| -> String {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(secs);
        while std::time::Instant::now() < deadline {
            if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(100)) {
                if msg.contains(needle) {
                    return msg;
                }
            }
        }
        panic!("never saw {needle:?}");
    };
    wait_for("\"action\":\"attach\"", &rx, 10);
    api.monitor_ready();
    let live = wait_for("\"state\":\"live\"", &rx, 10);
    assert!(live.contains("\"plugin\":\"stn-periodic\""), "{live}");
    // Feed the slot (the mock delivers nothing on its own — the silent
    // watchdog would rightly tear an unfed tier down after 3 s).
    context
        .platform
        .sink_deliver("7E8 04 41 0C 1A F8".to_string());
    std::thread::sleep(std::time::Duration::from_millis(300));
    assert_eq!(api.get_acquisition_stats().0, "stn-periodic");
    session.shutdown().unwrap();
}

#[test]
fn sp_engine_end_to_end_through_the_ladder() {
    use std::sync::mpsc;
    let (platform, context) = create_mock_external_platform();
    context.responder.set_mock_response("STCMM 1", "OK");
    context
        .responder
        .set_mock_response("STPPMA 25, 7E0, 010C", "1");
    context.responder.set_mock_response("AT", "OK");
    context.responder.set_mock_response("STPPMC", "OK");
    context.responder.set_mock_response("STPR", "A6 [AT]");
    context.responder.set_mock_response("STP 33", "OK");
    context
        .responder
        .set_mock_response("0100", "7E8 06 41 00 BE 3F A8 13");

    let raw_log: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    {
        let log = Arc::clone(&raw_log);
        let plat = Arc::clone(&context.platform);
        context.platform.set_raw_writer_rust(Box::new(move |line| {
            log.lock().unwrap().push(line.clone());
            if line == " " {
                plat.sink_deliver("STOPPED".to_string());
            }
        }));
    }

    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();
    assert!(api.send_init_commands());
    context
        .platform
        .update_connection_status(crate::platform::ConnectionStatus::Connected, None);
    api.set_stn_periodic_capable(true);
    {
        let state = api.shared_state.lock().unwrap();
        state.length_cache.lock().unwrap().record("010C", 2);
    }
    let sub_id = api
        .create_subscription(Some("sp".to_string()), vec!["010C".to_string()], None)
        .unwrap();
    api.start_subscription(sub_id).unwrap();

    let wait_for = |needle: &str, rx: &mpsc::Receiver<String>, secs: u64| -> String {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(secs);
        while std::time::Instant::now() < deadline {
            if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(100)) {
                if msg.contains(needle) {
                    return msg;
                }
            }
        }
        panic!("never saw {needle:?}");
    };
    wait_for("\"plugin\":\"stn-periodic\"", &rx, 10);
    wait_for("\"action\":\"attach\"", &rx, 10);
    api.monitor_ready();
    let live = wait_for("\"state\":\"live\"", &rx, 10);
    assert!(live.contains("\"messages\":1"), "{live}");

    // The install went out on the correlator; the arm went out raw, paced.
    let history = context.responder.get_command_history();
    assert!(history.iter().any(|c| c == "STCMM 1"), "{history:?}");
    assert!(
        history.iter().any(|c| c == "STPPMA 25, 7E0, 010C"),
        "{history:?}"
    );
    {
        let raw = raw_log.lock().unwrap().clone();
        let pos = |c: &str| {
            raw.iter()
                .position(|x| x == c)
                .unwrap_or_else(|| panic!("{c} not written: {raw:?}"))
        };
        assert!(
            pos("STPO") < pos("STFAC")
                && pos("STFAC") < pos("STFPA 7E8,7FF")
                && pos("STFPA 7E8,7FF") < pos("STM"),
            "{raw:?}"
        );
    }

    // A periodic frame decodes to the pid's poll-shaped data.
    context
        .platform
        .sink_deliver("7E8 04 41 0C 1A F8".to_string());
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    let mut saw = false;
    while std::time::Instant::now() < deadline && !saw {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(100)) {
            saw = msg.contains("\"command\":\"010C\"") && msg.contains("1A F8");
        }
    }
    assert!(saw, "periodic frame must decode");
    assert_eq!(api.get_acquisition_stats().0, "stn-periodic");

    // Pause → break, detach, verified teardown, polling resumed.
    api.pause_subscription(sub_id).unwrap();
    wait_for("\"state\":\"stopping\"", &rx, 5);
    wait_for("\"action\":\"detach\"", &rx, 5);
    api.monitor_ready();
    wait_for("\"state\":\"stopped\"", &rx, 20);
    {
        let raw = raw_log.lock().unwrap().clone();
        let stm = raw.iter().position(|x| x == "STM").unwrap();
        assert!(
            raw[stm + 1..].iter().any(|x| x == " "),
            "break byte after STM: {raw:?}"
        );
        assert!(
            raw[stm + 1..].iter().any(|x| x == "STFAC"),
            "filters cleared on break: {raw:?}"
        );
    }
    let history = context.responder.get_command_history();
    let after = history
        .iter()
        .rposition(|c| c == "STPPMA 25, 7E0, 010C")
        .unwrap();
    let tail = &history[after + 1..];
    let pos = |c: &str| {
        tail.iter()
            .position(|x| x == c)
            .unwrap_or_else(|| panic!("{c} not in teardown: {tail:?}"))
    };
    assert!(
        pos("STPPMC") < pos("STPR") && pos("STPR") < pos("STP 33") && pos("STP 33") < pos("0100"),
        "{tail:?}"
    );
    assert!(api
        .shared_state
        .lock()
        .unwrap()
        .periodic_state
        .lock()
        .unwrap()
        .is_none());
    assert_eq!(api.get_acquisition_stats().0, "poll");

    session.shutdown().unwrap();
}

/// LH6 / DV1 gate: a fake OBDX Pro on a loopback TCP socket. The session
/// connects the `wifi:` connector (LH5), the catalog says `dvi`, the
/// handler is rebuilt, and identify runs ENTIRELY over DVI: one ELM
/// string (`DX DP 1`), then binary `31 02 06 01`, `22 01 02/04`, `31 02
/// 01 02`, `31 02 02 01`, and `10` TX frames for `0100` / `0902` / `0101`
/// / the support scan / `090A`+`0904` walk. VIN decodes from a
/// tool-reassembled multi-frame; a physical `010C` completes on its one
/// reply; zero ELM bytes after the bootstrap.

/// A fake OBDX Pro on loopback TCP: ELM until `DX DP 1`, then DVI —
/// echoes `31`/`24`/`34` settings, answers identity, replies to `10`
/// writes from a canned vehicle (`fake_reply`), and runs the 8 periodic
/// slots (§3.14.26): `34 1B` data, `34 1A` interval, `34 1C` enable
/// fires the slot's canned reply every interval until disabled.
struct FakeObdx {
    tx_log: Arc<Mutex<Vec<(u32, Vec<u8>)>>>,
    elm_after_bootstrap: Arc<std::sync::atomic::AtomicUsize>,
    slot_log: Arc<Mutex<Vec<String>>>,
}

fn fake_reply(id: u32, payload: &[u8]) -> Vec<Vec<u8>> {
    use crate::link::dvi::codec;
    let rx = |ecu: u32, d: &[u8]| {
        let mut v = ecu.to_be_bytes().to_vec();
        v.extend_from_slice(d);
        codec::frame(0x08, &v)
    };
    match payload {
        [0x01, 0x00] => vec![
            rx(0x7E8, &[0x06, 0x41, 0x00, 0xBE, 0x3F, 0xA8, 0x13]),
            rx(0x7E9, &[0x06, 0x41, 0x00, 0x80, 0x00, 0x00, 0x00]),
        ],
        [0x01, 0x20]
        | [0x01, 0x40]
        | [0x01, 0x60]
        | [0x01, 0x80]
        | [0x01, 0xA0]
        | [0x01, 0xC0]
        | [0x06, ..]
        | [0x09, 0x00] => vec![],
        [0x01, 0x01] => vec![rx(0x7E8, &[0x06, 0x41, 0x01, 0x00, 0x07, 0xE5, 0x00])],
        [0x01, 0x0C] => vec![rx(0x7E8, &[0x04, 0x41, 0x0C, 0x1A, 0xF8])],
        [0x01, 0x05] => vec![rx(0x7E8, &[0x03, 0x41, 0x05, 0x4A])],
        // Polls get an answer; a SLOT carrying 0110 deliberately never
        // fires (silent-slot watchdog test).
        [0x01, 0x10] => vec![rx(0x7E8, &[0x04, 0x41, 0x10, 0x00, 0x10])],
        [0x01, 0x0C, 0x0D] => vec![rx(0x7E8, &[0x06, 0x41, 0x0C, 0x1A, 0xF8, 0x0D, 0x32])],
        [0x09, 0x02] => {
            let mut d = vec![0x10, 0x14, 0x49, 0x02, 0x01];
            d.extend_from_slice(b"1C4MOCK29SBIT0001");
            vec![rx(0x7E8, &d)]
        }
        [0x09, 0x0A] if id == 0x7E0 => {
            let mut d = vec![0x10, 0x0D, 0x49, 0x0A, 0x01];
            d.extend_from_slice(b"ECM-Engine");
            vec![rx(0x7E8, &d)]
        }
        _ => vec![],
    }
}

fn spawn_fake_obdx(listener: std::net::TcpListener) -> FakeObdx {
    use crate::link::dvi::codec::{self, DviFrame, FrameParser};
    use std::io::{Read, Write};
    let elm_after_bootstrap = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    let tx_log: Arc<Mutex<Vec<(u32, Vec<u8>)>>> = Arc::new(Mutex::new(Vec::new()));
    let slot_log: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
    let (elm_seen, tx_seen, slots_seen) = (
        Arc::clone(&elm_after_bootstrap),
        Arc::clone(&tx_log),
        Arc::clone(&slot_log),
    );
    std::thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        let mut reader = stream.try_clone().unwrap();
        let writer = Arc::new(Mutex::new(stream));
        let mut parser = FrameParser::new();
        let mut dvi = false;
        // slot → (id, payload, interval ms, enabled flag)
        let mut slots: std::collections::HashMap<
            u8,
            (u32, Vec<u8>, u64, Arc<std::sync::atomic::AtomicBool>),
        > = std::collections::HashMap::new();
        let mut buf = [0u8; 1024];
        loop {
            let n = match reader.read(&mut buf) {
                Ok(0) | Err(_) => break,
                Ok(n) => n,
            };
            if !dvi {
                let text = String::from_utf8_lossy(&buf[..n]).to_uppercase();
                if text.contains("DX DP 1") {
                    writer.lock().unwrap().write_all(b"OK\r\r>").unwrap();
                    dvi = true;
                }
                continue;
            }
            if buf[..n]
                .iter()
                .all(|b| b.is_ascii_alphanumeric() || *b == b' ' || *b == b'\r')
                && n >= 3
            {
                elm_seen.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            }
            for f in parser.feed(&buf[..n]) {
                let mut reply: Vec<Vec<u8>> = Vec::new();
                if let DviFrame::Reply { cmd, data } = f {
                    match cmd {
                        0x31 => {
                            if let [0x02, mode] = data.as_slice() {
                                slots_seen.lock().unwrap().push(format!("comm {mode}"));
                            }
                            reply.push(codec::frame(0x41, &data));
                        }
                        0x22 => match data.first().copied() {
                            Some(0x02) => reply.push(codec::frame(0x32, &[0x02, b'G', b'T'])),
                            Some(0x04) => {
                                let mut d = vec![0x04];
                                d.extend_from_slice(&[1u8; 12]);
                                reply.push(codec::frame(0x32, &d));
                            }
                            Some(sub) => reply.push(codec::frame(0x32, &[sub, 0x0F])),
                            None => {}
                        },
                        0x24 => reply.push(codec::frame(0x34, &data)),
                        0x34 => {
                            match data.as_slice() {
                                [0x1A, slot, hi, lo] => {
                                    let e = slots.entry(*slot).or_insert((
                                        0,
                                        Vec::new(),
                                        100,
                                        Arc::new(std::sync::atomic::AtomicBool::new(false)),
                                    ));
                                    e.2 = u16::from_be_bytes([*hi, *lo]) as u64;
                                }
                                [0x1B, slot, i0, i1, i2, i3, payload @ ..] => {
                                    let e = slots.entry(*slot).or_insert((
                                        0,
                                        Vec::new(),
                                        100,
                                        Arc::new(std::sync::atomic::AtomicBool::new(false)),
                                    ));
                                    e.0 = u32::from_be_bytes([*i0, *i1, *i2, *i3]);
                                    e.1 = payload.to_vec();
                                }
                                [0x00, slot, _ext, ty, on, rest @ ..] if rest.len() == 12 => {
                                    let id =
                                        u32::from_be_bytes([rest[0], rest[1], rest[2], rest[3]]);
                                    let fc =
                                        u32::from_be_bytes([rest[8], rest[9], rest[10], rest[11]]);
                                    slots_seen.lock().unwrap().push(format!(
                                        "filter {slot} ty={ty} on={on} {id:X}<-{fc:X}"
                                    ));
                                }
                                [0x1C, slot, on] => {
                                    let e = slots.entry(*slot).or_insert((
                                        0,
                                        Vec::new(),
                                        100,
                                        Arc::new(std::sync::atomic::AtomicBool::new(false)),
                                    ));
                                    slots_seen.lock().unwrap().push(format!(
                                        "{} {slot}",
                                        if *on == 1 { "enable" } else { "disable" }
                                    ));
                                    e.3.store(false, std::sync::atomic::Ordering::SeqCst);
                                    if *on == 1 {
                                        let flag =
                                            Arc::new(std::sync::atomic::AtomicBool::new(true));
                                        e.3 = Arc::clone(&flag);
                                        // Periodic frames are raw: the payload carries its ISO-TP
                                        // length byte and is padded to a full 8-byte CAN payload
                                        // (hardware 2026-08-29: short frames never fire) — strip
                                        // both before the canned-vehicle lookup.
                                        let raw = e.1.clone();
                                        let obd = match raw.first() {
                                            Some(&n) if (n as usize) + 1 <= raw.len() => {
                                                raw[1..1 + n as usize].to_vec()
                                            }
                                            _ => raw,
                                        };
                                        let silent_slot = obd == [0x01, 0x10];
                                        let (id, payload, ms, w) =
                                            (e.0, obd, e.2, Arc::clone(&writer));
                                        std::thread::spawn(move || {
                                            while flag.load(std::sync::atomic::Ordering::SeqCst) {
                                                std::thread::sleep(
                                                    std::time::Duration::from_millis(ms.max(5)),
                                                );
                                                if !flag.load(std::sync::atomic::Ordering::SeqCst)
                                                    || silent_slot
                                                {
                                                    continue;
                                                }
                                                for r in fake_reply(id, &payload) {
                                                    if w.lock().unwrap().write_all(&r).is_err() {
                                                        return;
                                                    }
                                                }
                                            }
                                        });
                                    }
                                }
                                _ => {}
                            }
                            reply.push(codec::frame(0x44, &data));
                        }
                        0x10 => {
                            let id = u32::from_be_bytes([data[0], data[1], data[2], data[3]]);
                            let payload = data[4..].to_vec();
                            tx_seen.lock().unwrap().push((id, payload.clone()));
                            reply.push(codec::frame(0x20, &[0x00]));
                            reply.extend(fake_reply(id, &payload));
                        }
                        _ => {}
                    }
                }
                let mut w = writer.lock().unwrap();
                for r in reply {
                    if w.write_all(&r).is_err() {
                        return;
                    }
                }
            }
        }
        for (_, (_, _, _, flag)) in slots {
            flag.store(false, std::sync::atomic::Ordering::SeqCst);
        }
    });
    FakeObdx {
        tx_log,
        elm_after_bootstrap,
        slot_log,
    }
}

/// DV3 stream half: the periodic-slot engine over the loopback fake —
/// the ladder engages the "stn-periodic" tier on a DVI link, slot 0 is
/// loaded with the dashboard's 010C and enabled, its replies decode to
/// poll-shaped obd_data, and pausing disables the slot.
/// Shared DVI loopback bring-up for the DV3/DV4 tests: fake tool on a
/// local TCP port, catalog → DVI handler, bootstrap + identify.
fn dvi_loopback_session() -> (
    FakeObdx,
    OBDSessionManager,
    SessionAPIHandle,
    std::sync::mpsc::Receiver<String>,
) {
    use std::sync::mpsc;
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let fake = spawn_fake_obdx(listener);
    let (platform, _ctx) = create_mock_external_platform();
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |m| {
        let _ = tx.send(m);
    })
    .unwrap();
    let api = session.api_handle();
    api.set_adapter_catalog(r#"{"adapters":[{"name":"OBDX Pro","protocol":"dvi","supportsPeriodic":true,"wifi":{"namePatterns":["OBDX","127.0.0.1"]}}]}"#).unwrap();
    let connector = crate::platform::ConnectorInfo {
        id: format!("wifi:127.0.0.1:{port}"),
        name: "OBDX 127.0.0.1".into(),
        connector_type: "wifi".into(),
    };
    let (ctx_tx, ctx_rx) = mpsc::channel();
    let plat = Arc::clone(&api.shared_state.lock().unwrap().platform);
    plat.connect_to(
        &connector.id,
        Box::new(move |r| {
            let _ = ctx_tx.send(r);
        }),
    );
    assert!(matches!(
        ctx_rx
            .recv_timeout(std::time::Duration::from_secs(5))
            .unwrap(),
        crate::platform::ConnectResult::Connected
    ));
    api.apply_catalog_facts(&connector);
    api.rebuild_link_for_connect();
    assert!(api.send_init_commands());
    api.gather_vehicle_info().expect("identify over DVI");
    // `_ctx` is a reference — `mem::forget` on it was a no-op; the owned
    // context lives in `fake`/`session` returned below. Just ignore it.
    let _ = _ctx;
    (fake, session, api, rx)
}

fn wait_until(deadline_s: u64, mut cond: impl FnMut() -> bool) -> bool {
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(deadline_s);
    while std::time::Instant::now() < deadline {
        if cond() {
            return true;
        }
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    cond()
}

/// DV3 leftover: an `ATSH` to a module outside the connect-time FLOW
/// pairs installs ONE FLOW filter (slot 26, response id ← target) — RX
/// admission is the base 0x700-0x7FF range pass, so no companion pass
/// (2026-09-04). The same target again is free; another uncovered target
/// rewrites slot 26; a standard target (7E0–7E7) installs nothing.
#[test]
fn dv3_dynamic_flow_filter_follows_atsh_target() {
    let (fake, session, api, _rx) = dvi_loopback_session();
    let link = api.link();
    let flow = || {
        fake.slot_log
            .lock()
            .unwrap()
            .iter()
            .filter(|e| e.starts_with("filter 26"))
            .cloned()
            .collect::<Vec<_>>()
    };
    let pass28 = || {
        fake.slot_log
            .lock()
            .unwrap()
            .iter()
            .filter(|e| e.starts_with("filter 28"))
            .count()
    };
    let before_flow = flow().len();
    link.request("ATSH7A0", 2000);
    assert!(
        wait_until(3, || flow().len() == before_flow + 1),
        "{:?}",
        fake.slot_log.lock().unwrap()
    );
    assert_eq!(flow().last().unwrap(), "filter 26 ty=1 on=1 7A8<-7A0");
    link.request("ATSH7A0", 2000);
    link.request("ATSH7E0", 2000);
    std::thread::sleep(std::time::Duration::from_millis(300));
    assert_eq!(
        flow().len(),
        before_flow + 1,
        "repeat / standard target must not re-install flow: {:?}",
        flow()
    );
    link.request("ATSH7B0", 2000);
    assert!(
        wait_until(3, || flow().len() == before_flow + 2),
        "{:?}",
        fake.slot_log.lock().unwrap()
    );
    assert_eq!(flow().last().unwrap(), "filter 26 ty=1 on=1 7B8<-7B0");
    assert_eq!(
        pass28(),
        0,
        "no per-target pass filters any more (range pass covers RX)"
    );
    session.shutdown().unwrap();
}

/// DV3 leftover: a slot tier that delivers nothing is torn down by the
/// watchdog (≈3 s + the 1 s tick) through the normal stop path — the
/// host sees stopping/stopped(reason silent), the slot is disabled — and
/// the ladder does not re-engage during the backoff.
#[test]
fn dv3_silent_slots_fall_back_to_poll() {
    let (fake, session, api, rx) = dvi_loopback_session();
    {
        let state = api.shared_state.lock().unwrap();
        state.length_cache.lock().unwrap().record("0110", 2);
    }
    let sub = api
        .create_subscription(Some("silent".to_string()), vec!["0110".to_string()], None)
        .unwrap();
    api.start_subscription(sub).unwrap();
    let mut saw_live = false;
    let mut saw_silent = false;
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(15);
    while std::time::Instant::now() < deadline && !saw_silent {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(200)) {
            if msg.contains("\"state\":\"live\"") && msg.contains("dvi-periodic") {
                saw_live = true;
            }
            if msg.contains("\"state\":\"stopped\"") && msg.contains("\"reason\":\"silent\"") {
                saw_silent = true;
            }
        }
    }
    assert!(saw_live, "slot tier must engage first");
    assert!(
        saw_silent,
        "watchdog must stop a silent slot tier: {:?}",
        fake.slot_log.lock().unwrap()
    );
    assert!(
        fake.slot_log
            .lock()
            .unwrap()
            .iter()
            .any(|e| e == "disable 0"),
        "{:?}",
        fake.slot_log.lock().unwrap()
    );
    let enables = || {
        fake.slot_log
            .lock()
            .unwrap()
            .iter()
            .filter(|e| e.starts_with("enable"))
            .count()
    };
    let n = enables();
    std::thread::sleep(std::time::Duration::from_secs(3));
    assert_eq!(
        enables(),
        n,
        "ladder must hold off re-engage during the backoff"
    );
    // The user's Resume (a subscription start) is "try again": the hold
    // lifts and the ladder re-engages the slots.
    api.start_subscription(sub).unwrap();
    assert!(
        wait_until(8, || enables() > n),
        "start_subscription must lift the backoff and re-engage: {:?}",
        fake.slot_log.lock().unwrap()
    );
    session.shutdown().unwrap();
}

/// DV4: a capture on a DVI link = PASS filters for the id list (slots
/// 16+), PASS-all parked, bus LISTEN-ONLY; stop unwinds all three.
#[test]
fn dv4_capture_over_dvi_sets_pass_filters_and_listen_only() {
    let (fake, session, api, _rx) = dvi_loopback_session();
    let plat = Arc::clone(&api.shared_state.lock().unwrap().platform);
    api.sink_start(Arc::clone(&plat), Some("7E8,7A8/7FF".to_string()), None)
        .expect("DVI capture starts");
    let log = || fake.slot_log.lock().unwrap().clone();
    assert!(
        wait_until(3, || log().iter().any(|e| e == "comm 2")),
        "{:?}",
        log()
    );
    let l = log();
    assert!(l.iter().any(|e| e == "filter 16 ty=0 on=1 7E8<-0"), "{l:?}");
    assert!(l.iter().any(|e| e == "filter 17 ty=0 on=1 7A8<-0"), "{l:?}");
    assert!(
        l.iter().any(|e| e == "filter 8 ty=0 on=0 0<-0"),
        "PASS-all parked: {l:?}"
    );
    assert!(api.sink_stats().active);
    api.sink_stop(plat);
    let l = log();
    assert!(l.iter().any(|e| e == "comm 1"), "comm ON restored: {l:?}");
    assert!(
        l.iter().any(|e| e == "filter 16 ty=0 on=0 0<-0"),
        "capture filter cleared: {l:?}"
    );
    assert!(
        l.iter().any(|e| e == "filter 8 ty=0 on=1 700<-0"),
        "diagnostic-range pass filter restored: {l:?}"
    );
    assert!(!api.sink_stats().active);
    session.shutdown().unwrap();
}

#[test]
fn dv3_periodic_slots_over_dvi_loopback() {
    use std::sync::mpsc;
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let fake = spawn_fake_obdx(listener);

    let (platform, _ctx) = create_mock_external_platform();
    let (tx, rx) = mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |m| {
        let _ = tx.send(m);
    })
    .unwrap();
    let api = session.api_handle();
    api.set_adapter_catalog(r#"{"adapters":[{"name":"OBDX Pro","protocol":"dvi","supportsPeriodic":true,"wifi":{"namePatterns":["OBDX","127.0.0.1"]}}]}"#).unwrap();
    let connector = crate::platform::ConnectorInfo {
        id: format!("wifi:127.0.0.1:{port}"),
        name: "OBDX 127.0.0.1".into(),
        connector_type: "wifi".into(),
    };
    let (ctx_tx, ctx_rx) = mpsc::channel();
    let plat = Arc::clone(&api.shared_state.lock().unwrap().platform);
    plat.connect_to(
        &connector.id,
        Box::new(move |r| {
            let _ = ctx_tx.send(r);
        }),
    );
    assert!(matches!(
        ctx_rx
            .recv_timeout(std::time::Duration::from_secs(5))
            .unwrap(),
        crate::platform::ConnectResult::Connected
    ));
    api.apply_catalog_facts(&connector);
    api.rebuild_link_for_connect();
    assert!(
        api.link().facts().periodic_capable,
        "catalog supportsPeriodic → DVI slot engine"
    );
    assert!(api.send_init_commands());
    api.gather_vehicle_info().expect("identify over DVI");
    {
        let state = api.shared_state.lock().unwrap();
        state.length_cache.lock().unwrap().record("010C", 2);
        state.length_cache.lock().unwrap().record("0105", 1);
    }
    // 010C (default Medium) → slot 0 @100 ms; 0105 Slow → slot 1 @500 ms
    // (every tier rides the slots on a concurrent link); polling keeps
    // running for run-once work but has no continuous pid left.
    let sub = api
        .create_subscription(
            Some("dvi".to_string()),
            vec!["010C".to_string(), "0105".to_string()],
            None,
        )
        .unwrap();
    {
        let state = api.shared_state.lock().unwrap();
        let mut tiers = std::collections::HashMap::new();
        tiers.insert("0105".to_string(), crate::subscription::RefreshTier::Slow);
        state
            .subscription_manager
            .lock()
            .unwrap()
            .set_pid_tiers(sub, tiers)
            .unwrap();
    }
    api.start_subscription(sub).unwrap();

    let wait_for = |needle: &str, rx: &mpsc::Receiver<String>, secs: u64| -> String {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(secs);
        while std::time::Instant::now() < deadline {
            if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(100)) {
                if msg.contains(needle) {
                    return msg;
                }
            }
        }
        panic!("never saw {needle:?}");
    };
    wait_for("\"plugin\":\"dvi-periodic\"", &rx, 10);
    wait_for("\"action\":\"attach\"", &rx, 10);
    api.monitor_ready();
    let live = wait_for("\"state\":\"live\"", &rx, 10);
    assert!(live.contains("\"messages\":2"), "{live}");
    assert!(
        fake.slot_log
            .lock()
            .unwrap()
            .iter()
            .any(|e| e == "enable 0"),
        "{:?}",
        fake.slot_log.lock().unwrap()
    );
    assert!(
        fake.slot_log
            .lock()
            .unwrap()
            .iter()
            .any(|e| e == "enable 1"),
        "Slow pid takes a slot too: {:?}",
        fake.slot_log.lock().unwrap()
    );
    let tx_at_live = fake.tx_log.lock().unwrap().len();

    // Slot replies decode to the pid's poll-shaped data.
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(3);
    let mut saw = false;
    while std::time::Instant::now() < deadline && !saw {
        if let Ok(msg) = rx.recv_timeout(std::time::Duration::from_millis(100)) {
            saw = msg.contains("\"command\":\"010C\"") && msg.contains("\"data_bytes\":[26,248]");
        }
    }
    assert!(saw, "periodic slot reply must decode");
    assert_eq!(api.get_acquisition_stats().0, "dvi-periodic");

    // Coexistence: with every continuous pid on a slot the poll chain
    // idles — once the sweep queued before the install drains (≤ one
    // exchange per pid), neither slot-served pid is polled again.
    let _ = tx_at_live;
    std::thread::sleep(std::time::Duration::from_millis(300));
    let tx_settled = fake.tx_log.lock().unwrap().len();
    std::thread::sleep(std::time::Duration::from_millis(700));
    {
        let txs = fake.tx_log.lock().unwrap();
        let since = &txs[tx_settled.min(txs.len())..];
        assert!(
            !since
                .iter()
                .any(|(_, p)| p == &[0x01, 0x0C] || p == &[0x01, 0x05]),
            "slot-served pid was polled: {since:?}"
        );
    }

    // Pause → teardown disables the slot.
    api.pause_subscription(sub).unwrap();
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(6);
    while std::time::Instant::now() < deadline
        && !fake
            .slot_log
            .lock()
            .unwrap()
            .iter()
            .any(|e| e == "disable 0")
    {
        std::thread::sleep(std::time::Duration::from_millis(50));
    }
    assert!(
        fake.slot_log
            .lock()
            .unwrap()
            .iter()
            .any(|e| e == "disable 0"),
        "{:?}",
        fake.slot_log.lock().unwrap()
    );
    session.shutdown().unwrap();
}

#[test]
fn dv1_identify_over_dvi_loopback() {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    let port = listener.local_addr().unwrap().port();
    let fake = spawn_fake_obdx(listener);
    let elm_after_bootstrap = Arc::clone(&fake.elm_after_bootstrap);
    let tx_log = Arc::clone(&fake.tx_log);

    let (platform, _ctx) = create_mock_external_platform();
    let (tx, _rx) = std::sync::mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, config_for_test(), move |m| {
        let _ = tx.send(m);
    })
    .unwrap();
    let api = session.api_handle();
    api.set_adapter_catalog(r#"{"adapters":[{"name":"OBDX Pro","protocol":"dvi","wifi":{"namePatterns":["OBDX","127.0.0.1"]}}]}"#).unwrap();

    // Connect the Rust transport the way connect_body does, then the
    // identify sequence with the catalog-chosen handler.
    let connector = crate::platform::ConnectorInfo {
        id: format!("wifi:127.0.0.1:{port}"),
        name: "OBDX 127.0.0.1".into(),
        connector_type: "wifi".into(),
    };
    let (ctx_tx, ctx_rx) = std::sync::mpsc::channel();
    let plat = Arc::clone(&api.shared_state.lock().unwrap().platform);
    plat.connect_to(
        &connector.id,
        Box::new(move |r| {
            let _ = ctx_tx.send(r);
        }),
    );
    assert!(matches!(
        ctx_rx
            .recv_timeout(std::time::Duration::from_secs(5))
            .unwrap(),
        crate::platform::ConnectResult::Connected
    ));
    api.apply_catalog_facts(&connector);
    api.rebuild_link_for_connect();
    assert_eq!(api.link().facts().kind, crate::link::LinkKind::Dvi);

    assert!(
        api.send_init_commands(),
        "DVI bootstrap + identity + protocol"
    );
    let info = api.gather_vehicle_info().expect("identify over DVI");
    assert_eq!(info.vin, "1C4MOCK29SBIT0001");
    assert!(
        info.supported_pids.contains(&"010C".to_string()),
        "{:?}",
        info.supported_pids
    );
    let ecm = info
        .ecus
        .iter()
        .find(|e| e.controller_id == "7E0")
        .expect("ECM walked");
    assert_eq!(ecm.name, "ECM-Engine");

    // A physical request completes on its one reply (no quiet wait).
    let t0 = std::time::Instant::now();
    let reply = api.link().request("010C", 2000);
    assert!(
        t0.elapsed() < std::time::Duration::from_millis(500),
        "physical reply must not wait for the quiet window"
    );
    assert_eq!(
        reply.parsed.unwrap().all_controllers["7E0"].data_bytes,
        vec![0x1A, 0xF8]
    );

    // The poll chain targets the primary ECU physically on DVI (2026-08-29):
    // a dashboard subscription's 010C goes to 7E0, not 7DF.
    let dash = api
        .create_subscription(None, vec!["010C".to_string()], None)
        .unwrap();
    api.start_subscription(dash).unwrap();
    let t0 = std::time::Instant::now();
    loop {
        let physical = tx_log
            .lock()
            .unwrap()
            .iter()
            .any(|(id, p)| *id == 0x7E0 && p == &[0x01, 0x0C]);
        if physical {
            break;
        }
        assert!(
            t0.elapsed() < std::time::Duration::from_secs(3),
            "poll chain never sent 010C physically to 7E0: {:?}",
            tx_log.lock().unwrap()
        );
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    let _ = api.cancel_subscription(dash);

    assert_eq!(
        elm_after_bootstrap.load(std::sync::atomic::Ordering::SeqCst),
        0,
        "ELM text after bootstrap"
    );
    let txs = tx_log.lock().unwrap().clone();
    assert!(
        txs.iter().any(|(id, p)| *id == 0x7DF && p == &[0x01, 0x00]),
        "functional 0100 went out: {txs:?}"
    );
    assert!(
        txs.iter().any(|(id, p)| *id == 0x7E0 && p == &[0x09, 0x0A]),
        "physical 090A to 7E0: {txs:?}"
    );
    session.shutdown().unwrap();
}

/// M7: a platform without monitor support fails sink_start with a clear
/// error and leaves the session fully operational (buffer inactive,
/// nothing paused).
#[test]
fn sink_start_fails_cleanly_on_unsupported_platform() {
    use crate::platform::{
        ConnectionCallback, ConnectionStatus, OBDDataCallback, OBDPlatformInterface,
    };

    /// Uses the trait DEFAULTS — no monitor-mode support.
    struct NoMonitorPlatform;
    impl OBDPlatformInterface for NoMonitorPlatform {
        fn send_command(&self, _c: &str, _t: u32) {}
        fn set_completion_callback(&self, _cb: Option<Box<dyn Fn(String) + Send + Sync>>) {}
        fn set_data_callback(&self, _cb: Option<OBDDataCallback>) {}
        fn set_connection_callback(&self, _cb: Option<ConnectionCallback>) {}
        fn is_connected(&self) -> bool {
            true
        }
        fn connection_status(&self) -> ConnectionStatus {
            ConnectionStatus::Connected
        }
    }

    let (ext_platform, _context) = create_mock_external_platform();
    let session = OBDSessionManager::new(ext_platform, config_for_test(), |_| {}).unwrap();
    let api = session.api_handle();

    let plain: Arc<dyn crate::platform::OBDPlatformInterface> = Arc::new(NoMonitorPlatform);
    let err = api
        .sink_start(plain, None, None)
        .expect_err("defaults must refuse");
    assert!(err.contains("not supported"), "clear reason: {err}");
    assert!(
        !api.sink_stats().active,
        "failed start leaves the buffer inactive"
    );

    session.shutdown().unwrap();
}

// ── D7: teardown-in-flight gate ──────────────────────────────────────

#[test]
fn test_teardown_guard_raii_and_saturation() {
    let counter = Arc::new(std::sync::atomic::AtomicUsize::new(0));
    {
        let _a = TeardownGuard::new(&counter);
        assert_eq!(counter.load(std::sync::atomic::Ordering::SeqCst), 1);
        {
            let _b = TeardownGuard::new(&counter);
            assert_eq!(counter.load(std::sync::atomic::Ordering::SeqCst), 2);
        }
        assert_eq!(counter.load(std::sync::atomic::Ordering::SeqCst), 1);
        // Disconnect reset racing a live guard: drop must saturate, not wrap.
        counter.store(0, std::sync::atomic::Ordering::SeqCst);
    }
    assert_eq!(
        counter.load(std::sync::atomic::Ordering::SeqCst),
        0,
        "saturating drop"
    );
}

#[test]
fn test_stop_requests_without_live_acquisition_never_mark_teardown() {
    let (platform, _context) = create_mock_external_platform();
    let session = OBDSessionManager::new(platform, OBDSessionConfig::default(), |_| {}).unwrap();
    let api = session.api_handle();
    // Nothing is streaming: both stops take their early-return paths.
    api.request_stop_stream();
    api.request_stop_stn_periodic(false);
    std::thread::sleep(std::time::Duration::from_millis(400));
    let pending = api
        .shared_state
        .lock()
        .unwrap()
        .teardowns_in_flight
        .load(std::sync::atomic::Ordering::SeqCst);
    assert_eq!(
        pending, 0,
        "early returns are not teardowns and must not hold the gate"
    );
    session.shutdown().unwrap();
}

/// The engage beat DEFERS (consumes beats without engaging or exiting)
/// while a teardown is marked, and proceeds the first beat after release.
/// Control: with the gate open, the disconnected-mock bail exits the
/// engage thread within the first beat — so staying alive PAST one beat
/// is only possible via the deferral path.
#[test]
fn test_engage_beat_defers_while_teardown_in_flight() {
    let (platform, _context) = create_mock_external_platform();
    let session = OBDSessionManager::new(platform, OBDSessionConfig::default(), |_| {}).unwrap();
    let api = session.api_handle();
    let (engage_pending, teardowns) = {
        let st = api.shared_state.lock().unwrap();
        (
            Arc::clone(&st.engage_pending),
            Arc::clone(&st.teardowns_in_flight),
        )
    };
    let running =
        |f: &Arc<std::sync::atomic::AtomicBool>| f.load(std::sync::atomic::Ordering::SeqCst);

    // Control: gate open → first beat hits the disconnected bail and exits.
    api.maybe_engage_stream();
    assert!(running(&engage_pending), "engage thread started");
    std::thread::sleep(std::time::Duration::from_millis(3600));
    assert!(
        !running(&engage_pending),
        "control: exits within one beat when gate open"
    );

    // Deferral: hold the gate → the thread must still be alive after that
    // same interval (it consumed the beat with `continue`, not an exit).
    teardowns.store(1, std::sync::atomic::Ordering::SeqCst);
    api.maybe_engage_stream();
    std::thread::sleep(std::time::Duration::from_millis(3600));
    assert!(
        running(&engage_pending),
        "beat deferred while teardown drains"
    );

    // Release → the next beat passes the gate, hits the disconnected
    // bail, and the thread exits.
    teardowns.store(0, std::sync::atomic::Ordering::SeqCst);
    std::thread::sleep(std::time::Duration::from_millis(3600));
    assert!(
        !running(&engage_pending),
        "proceeds (and exits via bail) after release"
    );

    session.shutdown().unwrap();
}

/// SP29: response-id derivation covers both protocols.
#[test]
fn periodic_response_id_both_protocols() {
    // 11-bit: +8 rule, unchanged.
    assert_eq!(super::periodic_response_id("7E0"), "7E8");
    assert_eq!(super::periodic_response_id("7E1"), "7E9");
    assert_eq!(super::periodic_response_id("73B"), "743");
    // 29-bit physical request → response is the target/source byte swap.
    assert_eq!(super::periodic_response_id("18DA10F1"), "18DAF110");
    assert_eq!(super::periodic_response_id("18DA18F1"), "18DAF118");
    assert_eq!(super::periodic_response_id("18da10f1"), "18DAF110");
    // GB13: GM-enhanced priority 14 swaps the same way, priority preserved.
    assert_eq!(super::periodic_response_id("14DA41F1"), "14DAF141");
    assert_eq!(super::periodic_response_id("14DA11F1"), "14DAF111");
    // Unrecognized → unchanged (filter still narrows).
    assert_eq!(super::periodic_response_id("18DB33F1"), "18DB33F1");
    assert_eq!(super::periodic_response_id("XYZ9"), "XYZ9");
}
