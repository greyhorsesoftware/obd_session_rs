//! LH1 gate — wire-trace golden for the ELM handshake + identify + poll path.
//!
//! For every crate mock (`mock_data/*.json`, incl. the 29-bit Jeep copy) this drives the SAME sequence
//! `connect_body` runs (init → identify → chunk probe) and then one run-once
//! poll of the first six supported PIDs, and compares:
//!   * the exact ordered command history through the chunk probe,
//!   * the poll wires as a SORTED multiset (pick order at equal ticks follows
//!     registry iteration order — documented in `bp1_piped_exchange_end_to_end`),
//!   * the `VehicleInfo` JSON.
//! against `tests/fixtures/golden/<mock>.golden`. Recorded on the pre-LH1
//! baseline; `UPDATE_GOLDEN=1 cargo test golden_` rewrites the fixtures.
//! The pipe / chunk encodings are covered by the bp1 / b1 / b2 goldens in
//! `tests.rs` (the default mock answers `?` to STBC, so these traces are serial).

use super::*;
use crate::external_platform::create_mock_external_platform;
use std::time::{Duration, Instant};

fn fixture_path(mock: &str) -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/golden")
        .join(format!("{mock}.golden"))
}

/// The responder has no support-bitmap synthesis and most mocks carry no
/// `0100` entry, so build one from the mock's own Mode-01 PID list: bits for
/// PIDs 01–20 behind the header of its first `0902` line (11-bit `7E8` or
/// 29-bit `18 DA F1 xx`). Mocks that already answer `0100` are left alone.
fn synthesize_0100(path: &str) -> Option<String> {
    let json: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(path).ok()?).ok()?;
    let mode01 = json.get("01")?.as_object()?;
    if mode01.contains_key("00") {
        return None;
    }
    let first_0902 = json.get("09")?.get("02")?.as_array()?.first()?.as_str()?;
    let tokens: Vec<&str> = first_0902.split_whitespace().collect();
    let header_len = if tokens[0].len() == 3 { 1 } else { 4 };
    let header = tokens[..header_len].join(" ");
    let mut bits: u32 = 0;
    for pid in mode01.keys() {
        if let Ok(n) = u32::from_str_radix(pid, 16) {
            if (1..=0x20).contains(&n) {
                bits |= 1 << (32 - n);
            }
        }
    }
    let b = bits.to_be_bytes();
    Some(format!(
        "{header} 06 41 00 {:02X} {:02X} {:02X} {:02X}",
        b[0], b[1], b[2], b[3]
    ))
}

fn run_mock(mock: &str, path: &str) -> String {
    let (platform, context) = create_mock_external_platform();
    context
        .responder
        .load_car_scan_data(path)
        .expect("mock loads");
    if let Some(r) = synthesize_0100(path) {
        context.responder.set_mock_response("0100", &r);
    }
    context.responder.set_delay(Duration::from_millis(1));
    context.responder.clear_command_history();

    let (tx, rx) = std::sync::mpsc::channel::<String>();
    let session = OBDSessionManager::new(platform, OBDSessionConfig::default(), move |msg| {
        let _ = tx.send(msg);
    })
    .unwrap();
    let api = session.api_handle();

    // The connect_body sequence, minus the lifecycle plumbing.
    let init_ok = api.send_init_commands();
    // `supported_pids` is a HashSet union → Vec: order is not stable, so sort
    // it for the fixture (and for the deterministic poll pick below).
    let info = api.gather_vehicle_info().map(|mut v| {
        v.supported_pids.sort();
        v
    });
    api.probe_chunk_capability();
    let identify_history = context.responder.get_command_history();
    context.responder.clear_command_history();

    // One run-once poll of the first six supported PIDs (deterministic pick).
    let mut poll_wires: Vec<String> = Vec::new();
    if let Ok(info) = &info {
        let pids: Vec<String> = info.supported_pids.iter().take(6).cloned().collect();
        if !pids.is_empty() {
            let run_counts = pids.iter().map(|p| (p.clone(), Some(1u32))).collect();
            let sub = api
                .create_subscription_with_run_counts(
                    Some("golden".into()),
                    pids,
                    None,
                    Some(run_counts),
                )
                .unwrap();
            api.start_subscription(sub).unwrap();
            let deadline = Instant::now() + Duration::from_secs(10);
            while Instant::now() < deadline {
                match rx.recv_timeout(Duration::from_millis(200)) {
                    Ok(msg) if msg.contains("subscription_complete") => break,
                    Ok(_) => {}
                    Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
                    Err(_) => break,
                }
            }
            poll_wires = context.responder.get_command_history();
            poll_wires.sort();
        }
    }
    session.shutdown().unwrap();

    let mut out = String::new();
    out.push_str(&format!(
        "# mock: {mock}\n# init_ok: {init_ok}\n--- identify (ordered) ---\n"
    ));
    for c in &identify_history {
        out.push_str(c);
        out.push('\n');
    }
    out.push_str("--- poll (sorted) ---\n");
    for c in &poll_wires {
        out.push_str(c);
        out.push('\n');
    }
    out.push_str("--- vehicle_info ---\n");
    match info {
        Ok(v) => out.push_str(&serde_json::to_string_pretty(&v).unwrap()),
        Err(()) => out.push_str("Err"),
    }
    out.push('\n');
    out
}

fn check(mock: &str, path: &str) {
    let actual = run_mock(mock, path);
    let path = fixture_path(mock);
    if std::env::var_os("UPDATE_GOLDEN").is_some() {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(&path, &actual).unwrap();
        return;
    }
    let expected = std::fs::read_to_string(&path).unwrap_or_else(|_| {
        panic!(
            "missing golden {} — run with UPDATE_GOLDEN=1",
            path.display()
        )
    });
    if expected != actual {
        let tmp = std::env::temp_dir().join(format!("{mock}.golden.actual"));
        std::fs::write(&tmp, &actual).unwrap();
        panic!(
            "wire trace for `{mock}` differs from {}\n(actual written to {})\n\n--- expected ---\n{expected}\n--- actual ---\n{actual}",
            path.display(),
            tmp.display()
        );
    }
}

#[test]
fn golden_default() {
    check("default", "mock_data/default.json");
}
#[test]
fn golden_bmw428i() {
    check("bmw428i", "mock_data/bmw428i.json");
}
#[test]
fn golden_gladiator() {
    check("gladiator", "mock_data/gladiator.json");
}
#[test]
fn golden_mustang50() {
    check("mustang50", "mock_data/mustang50.json");
}
#[test]
fn golden_rangerover() {
    check("rangerover", "mock_data/rangerover.json");
}
/// 29-bit addressing path (captured Jeep frames with a synthetic VIN and
/// synthetic calibration ids).
#[test]
fn golden_29bit_jeep() {
    check("29bitJeep", "mock_data/29bitJeep.json");
}
