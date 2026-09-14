//! Integration test that exercises the library the way Swift apps will use it
//!
//! This test uses the session callback to receive OBD data as JSON messages,
//! which is the same pattern that Swift/FFI consumers will use.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use obd_session_rs::external_platform::create_mock_external_platform;
use obd_session_rs::{
    platform, session_manager::SessionAPIHandle, OBDSessionConfig, OBDSessionManager,
};

/// Test PIDs with their descriptions
fn get_test_pids() -> Vec<(&'static str, &'static str)> {
    vec![
        // Mode 01 - Powertrain Diagnostic Data (all 25 PIDs with data in default.json)
        ("0101", "Monitor Status"),
        ("0103", "Fuel System Status"),
        ("0104", "Calculated Engine Load"),
        ("0105", "Engine Coolant Temperature"),
        ("0106", "Short Term Fuel Trim - Bank 1"),
        ("0107", "Long Term Fuel Trim - Bank 1"),
        ("010C", "Engine RPM"),
        ("010D", "Vehicle Speed"),
        ("010F", "Intake Air Temperature"),
        ("0110", "Mass Air Flow Rate"),
        ("0113", "Oxygen Sensors Present"),
        ("0114", "Oxygen Sensor 1"),
        ("011C", "OBD Standards Compliance"),
        ("011F", "Run Time Since Engine Start"),
        ("0120", "PIDs supported [21-40]"),
        ("0121", "Distance Traveled with MIL on"),
        ("012F", "Fuel Level Input"),
        ("0133", "Absolute Barometric Pressure"),
        ("0140", "PIDs supported [41-60]"),
        ("0142", "Control Module Voltage"),
        ("0146", "Ambient Air Temperature"),
        ("0151", "Fuel Type"),
        ("0165", "EGR Error"),
        ("0178", "Exhaust Gas Temperature Bank 1 Sensor 2"),
        ("01B1", "B1 PID (custom)"),
        // Mode 06 - On-board Test Results (confirmed working)
        ("0620", "O2 Sensor Test Results"),
        // Mode 09 - Vehicle Information (confirmed working)
        ("0906", "Calibration Verification Numbers"),
        ("090A", "Vehicle Identification Number"),
    ]
}

/// Expected responses from default.json (based on actual responses)
fn get_expected_responses() -> HashMap<String, String> {
    let mut expected_responses = HashMap::new();

    // Mode 01 responses (actual format from debug output)
    expected_responses.insert(
        "0101".to_string(),
        "7E8 06 41 01 86 07 EF 80\r7E9 06 41 01 81 00 00 00".to_string(),
    );
    expected_responses.insert("0103".to_string(), "7E8 04 41 03 02 01".to_string());
    expected_responses.insert("0104".to_string(), "7E8 03 41 04 32".to_string());
    expected_responses.insert(
        "0105".to_string(),
        "7E8 03 41 05 FF\r7E9 03 41 05 FF".to_string(),
    );
    expected_responses.insert("0106".to_string(), "7E8 03 41 06 3C".to_string());
    expected_responses.insert("0107".to_string(), "7E8 03 41 07 46".to_string());
    expected_responses.insert(
        "010C".to_string(),
        "7E8 04 41 0C FF FF\r7E9 04 41 0C FF FF".to_string(),
    );
    expected_responses.insert(
        "010D".to_string(),
        "7E8 03 41 0D 6B\r7E9 03 41 0D 6B\r7EA 03 41 0D 6B".to_string(),
    );
    expected_responses.insert("010F".to_string(), "7E8 03 41 0F 41".to_string());
    expected_responses.insert("0110".to_string(), "7E8 04 41 10 FF FF".to_string());
    expected_responses.insert("0113".to_string(), "7E8 03 41 13 01".to_string());
    expected_responses.insert("0114".to_string(), "7E8 04 41 14 FF 80".to_string());
    expected_responses.insert(
        "011C".to_string(),
        "7E8 03 41 1C 01\r7E9 03 41 1C 01\r7EA 03 41 1C 01".to_string(),
    );
    expected_responses.insert("011F".to_string(), "7E8 04 41 1F 02 58".to_string());
    expected_responses.insert("0120".to_string(), "7E8 06 41 20 80 02 20 01".to_string());
    expected_responses.insert("0121".to_string(), "7E8 04 41 21 03 E8".to_string());
    expected_responses.insert("012F".to_string(), "7E8 03 41 2F 80".to_string());
    expected_responses.insert("0133".to_string(), "7E8 03 41 33 64".to_string());
    expected_responses.insert("0140".to_string(), "7E8 06 41 40 44 00 00 00".to_string());
    expected_responses.insert("0142".to_string(), "7E8 04 41 42 2E E0".to_string());
    expected_responses.insert("0146".to_string(), "7E8 03 41 46 3C".to_string());
    expected_responses.insert("0151".to_string(), "7E8 03 41 51 01".to_string());
    expected_responses.insert("0165".to_string(), "7E8 0B 41 65 15 3F".to_string());
    expected_responses.insert(
        "0178".to_string(),
        "7E8 10 0B 41 78 0D 0A ED 09\r7E8 21 28 09 E6 0A A8 00 00".to_string(),
    );
    expected_responses.insert(
        "01B1".to_string(),
        "7E8 10 0B 41 B1 05 0A ED 09\r7E8 21 28 09 E6 0A A8 00 00".to_string(),
    );

    // Mode 06 responses
    expected_responses.insert("0620".to_string(), "7E8 06 46 20 C0 00 8C D9".to_string());

    // Mode 09 responses
    expected_responses.insert(
        "0906".to_string(),
        "7E8 07 49 06 01 17 91 BC 82".to_string(),
    );
    expected_responses.insert("090A".to_string(),  "7E8 10 17 49 0A 01 45 43 55\r7E8 21 31 2D 45 6E 67 69 6E\r7E8 22 65 43 6F 6E 74 72 6F\r7E8 23 6C 00 00 00 00 00 00".to_string());

    expected_responses
}

/// OBD data received through session callback
#[derive(Debug, Clone)]
struct OBDDataMessage {
    command: String,
    data: String,
    error: Option<String>,
}

/// Shared state for tracking PID completion via session callback
#[derive(Clone)]
struct TestState {
    completed_pids: Arc<Mutex<HashMap<String, bool>>>,
    received_responses: Arc<Mutex<HashMap<String, String>>>,
}

impl TestState {
    fn new(expected_pids: Vec<String>) -> Self {
        let mut completed = HashMap::new();
        for pid in &expected_pids {
            completed.insert(pid.clone(), false);
        }

        Self {
            completed_pids: Arc::new(Mutex::new(completed)),
            received_responses: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    fn process_obd_data(&self, data: OBDDataMessage) {
        // Extract PID from command (e.g., "ATSH7E0 010C" -> "010C", or just "010C" -> "010C")
        let pid = if data.command.contains(' ') {
            data.command
                .split_whitespace()
                .last()
                .unwrap_or(&data.command)
                .to_string()
        } else {
            data.command.clone()
        };

        // Store response and mark complete
        if let Ok(mut responses) = self.received_responses.lock() {
            if data.error.is_none() {
                responses.insert(pid.clone(), data.data);
            }
        }

        if let Ok(mut completed) = self.completed_pids.lock() {
            if completed.contains_key(&pid) {
                completed.insert(pid, true);
            }
        }
    }

    fn all_complete(&self) -> bool {
        if let Ok(completed) = self.completed_pids.lock() {
            completed.values().all(|&complete| complete)
        } else {
            false
        }
    }

    fn completed_count(&self) -> usize {
        if let Ok(completed) = self.completed_pids.lock() {
            completed.values().filter(|&&complete| complete).count()
        } else {
            0
        }
    }

    fn get_received_responses(&self) -> HashMap<String, String> {
        if let Ok(responses) = self.received_responses.lock() {
            responses.clone()
        } else {
            HashMap::new()
        }
    }
}

/// Parse JSON message from session callback to extract OBD data
fn parse_obd_data_message(json: &str) -> Option<OBDDataMessage> {
    // Parse the JSON to extract obd_data messages
    let value: serde_json::Value = serde_json::from_str(json).ok()?;

    // Check if this is an obd_data message
    if value.get("type")?.as_str()? != "obd_data" {
        return None;
    }

    let payload = value.get("payload")?;
    Some(OBDDataMessage {
        command: payload.get("command")?.as_str()?.to_string(),
        data: payload.get("data")?.as_str()?.to_string(),
        error: payload
            .get("error")
            .and_then(|e| e.as_str())
            .map(|s| s.to_string()),
    })
}

/// Context holder pairing the session with its mock context (the context
/// itself is `&'static` — leaked by create_mock_external_platform)
struct SessionWithContext {
    session: OBDSessionManager,
    #[allow(dead_code)]
    context: &'static obd_session_rs::mock_responder::MockExternalContext,
}

/// Create and setup the OBD session using the session callback pattern
/// This mirrors how Swift apps will use the library
///
/// Uses ExternalPlatform with MockResponder - validates the EXACT same code path
/// that production Swift would use with real Bluetooth.
fn setup_session_with_callback(test_state: TestState) -> SessionWithContext {
    let test_state_clone = test_state.clone();

    // Create mock external platform - tests the same path as production Swift
    let (platform, context) = create_mock_external_platform();

    // Create session config
    let config = OBDSessionConfig::default();

    // Create OBD session manager with session callback that receives JSON messages
    // THIS IS THE KEY: Swift apps use this same pattern
    let session = OBDSessionManager::new(
        platform as Arc<dyn platform::OBDPlatformInterface>,
        config,
        move |json_response: String| {
            // This callback receives ALL data as JSON - just like Swift will
            if let Some(obd_data) = parse_obd_data_message(&json_response) {
                test_state_clone.process_obd_data(obd_data);
            }
        },
    )
    .expect("Failed to create session");

    SessionWithContext { session, context }
}

/// Create and start a subscription for the given PIDs
fn create_and_start_subscription(api: &SessionAPIHandle, pids: &[(&str, &str)]) -> uuid::Uuid {
    // Convert PIDs to strings
    let pid_strings: Vec<String> = pids.iter().map(|(pid, _)| pid.to_string()).collect();

    // Create subscription with run counts (most continuous, some run once)
    let mut run_counts = std::collections::HashMap::new();

    // Make some PIDs run only once (like VIN or calibration data)
    for (pid, _) in pids {
        if *pid == "0906" {
            // Calibration data - run once
            run_counts.insert(pid.to_string(), Some(1u32));
        }
        // Others default to continuous (None)
    }

    let run_counts_option = if run_counts.is_empty() {
        None
    } else {
        Some(run_counts)
    };

    // Create subscription
    let subscription_id = api
        .create_subscription_with_run_counts(None, pid_strings, None, run_counts_option)
        .expect("Failed to create subscription");

    // Start the subscription
    api.start_subscription(subscription_id)
        .expect("Failed to start subscription");

    subscription_id
}

/// Verify that responses match expected data
fn verify_responses(
    pids_to_test: &[(&str, &str)],
    expected_responses: &HashMap<String, String>,
    received_responses: &HashMap<String, String>,
) -> usize {
    println!("\nVerifying responses...");
    let mut verified_count = 0;

    for (pid, description) in pids_to_test {
        if let Some(expected) = expected_responses.get(&pid.to_string()) {
            if let Some(received) = received_responses.get(&pid.to_string()) {
                // Compare responses directly (both should have same \r formatting)
                if received == expected {
                    println!(
                        "✅ {} ({}): Response matches expected data",
                        pid, description
                    );
                    verified_count += 1;
                } else {
                    println!("❌ {} ({}): Response mismatch", pid, description);
                    println!("  Expected: {}", expected);
                    println!("  Received: {}", received);
                }
            } else {
                println!("❌ {} ({}): No response received", pid, description);
            }
        } else {
            println!(
                "⚠️  {} ({}): No expected response defined in test",
                pid, description
            );
        }
    }

    verified_count
}

#[test]
fn test_obd_session_with_mock_platform() {
    println!("\n🧪 Integration Test: Using Session Callback Pattern (like Swift apps)");
    println!("{}", "=".repeat(60));

    // Get test data
    let pids_to_test = get_test_pids();
    let expected_responses = get_expected_responses();
    let pid_strings: Vec<String> = pids_to_test
        .iter()
        .map(|(pid, _)| pid.to_string())
        .collect();

    // Create test state to track completion
    let test_state = TestState::new(pid_strings);

    // Setup session using session callback pattern (like Swift apps will use)
    // Uses ExternalPlatform + MockResponder - validates production code path
    let session_with_context = setup_session_with_callback(test_state.clone());
    let api = session_with_context.session.api_handle();

    // Create and start subscription
    let _subscription_id = create_and_start_subscription(&api, &pids_to_test);

    println!(
        "📡 Subscribed to {} PIDs, waiting for responses via session callback...",
        pids_to_test.len()
    );

    // Wait for all PIDs to complete (instead of fixed timeout)
    let mut wait_count = 0u32;
    loop {
        if test_state.all_complete() {
            println!("✅ All PIDs have completed!");
            break;
        }

        // Small delay to avoid busy waiting
        std::thread::sleep(Duration::from_millis(10));

        // Safety check - don't wait forever
        wait_count += 1;
        if wait_count > 10000 {
            // ~100 seconds max wait
            println!(
                "⚠️  Completed {} out of {} PIDs before timeout",
                test_state.completed_count(),
                pids_to_test.len()
            );
            break;
        }
    }

    // Get responses collected via session callback
    let received_responses = test_state.get_received_responses();

    // Verify responses
    let verified_count = verify_responses(&pids_to_test, &expected_responses, &received_responses);
    let completed_count = test_state.completed_count();

    // Print summary and assert
    println!("\n📊 Test Summary:");
    println!("   Subscribed to {} PIDs", pids_to_test.len());
    println!("   Completed {} PIDs", completed_count);
    println!("   Verified {} responses", verified_count);
    println!(
        "   Expected responses defined for {} PIDs",
        expected_responses.len()
    );

    // Assert that we got responses for all PIDs
    assert!(
        verified_count == pids_to_test.len(),
        "Only {} out of {} PIDs returned correct responses",
        verified_count,
        pids_to_test.len()
    );
    assert!(
        completed_count == pids_to_test.len(),
        "Only {} out of {} PIDs completed",
        completed_count,
        pids_to_test.len()
    );

    // Get and display command processor statistics
    match api.get_command_processor_stats() {
        Ok(stats) => {
            println!("\n📈 Command Processor Statistics:");
            println!("   Total Commands: {}", stats.total_commands);
            println!("   Completed Commands: {}", stats.completed_commands);
            println!("   Failed Commands: {}", stats.failed_commands);
            println!("   PIDs per Second: {:.2}", stats.pids_per_second());

            if let Some(avg_time) = stats.average_completion_time() {
                println!("   Average Completion Time: {:.2}ms", avg_time.as_millis());
            } else {
                println!("   Average Completion Time: N/A");
            }

            println!("   Currently In Progress: {}", stats.in_progress_commands);
        }
        Err(e) => {
            println!("⚠️  Failed to get command processor stats: {:?}", e);
        }
    }

    println!("\n✅ Integration test passed!");
    println!("   Data flowed through session callback → JSON → test verification");
    println!("   This matches how Swift apps will receive OBD data!");
}
