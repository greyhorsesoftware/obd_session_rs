//! Tests for mock data file compatibility
//!
//! These tests verify that mock car scan data files can be loaded and used
//! by the obd_session_rs library. Uses MockResponder which can be paired
//! with ExternalPlatform for full integration testing.
//!
//! Standard format:
//! ```json
//! {
//!   "01": {           // OBD-II Mode (service)
//!     "0C": [         // PID (parameter identifier)
//!       "41 0C 1A F8" // Array of hex response strings
//!     ]
//!   }
//! }
//! ```
//!
//! Special handling for mode 0A (no PID byte):
//! ```json
//! {
//!   "0A": {
//!     "data": [       // Uses "data" key instead of PID
//!       "7E8 02 4A 00"
//!     ]
//!   }
//! }
//! ```
//!
//! The library can handle additional fields like "vehicle_info", "all", etc.

use obd_session_rs::mock_responder::MockResponder;

// Note: prius.json was created for testing but doesn't exist in original mock_data
// #[test]
// fn test_prius_mock_data() {
//     test_mock_data_file("mock_data/prius.json");
// }

#[allow(dead_code)]
fn test_mock_data_file(file_path: &str) {
    // Create mock responder
    let responder = MockResponder::new();

    // Load the mock data file
    responder
        .load_car_scan_data(file_path)
        .expect(&format!("Failed to load JSON from {}", file_path));

    // Test some common commands
    let test_commands = vec!["010C", "010D", "0105", "0902"];

    for cmd in &test_commands {
        let response = responder.generate_response(cmd);
        assert!(
            !response.is_empty(),
            "Empty response for {} in {}",
            cmd,
            file_path
        );
    }

    println!("✅ {} mock data verified successfully", file_path);
}

#[test]
fn test_mock_data_structure() {
    // Create mock responder
    let responder = MockResponder::new();

    // Load test data
    responder
        .load_car_scan_data("mock_data/gladiator.json")
        .unwrap();

    // Test that we can query PIDs
    let rpm_response = responder.generate_response("010C"); // Engine RPM
    let speed_response = responder.generate_response("010D"); // Vehicle Speed
    let vin_response = responder.generate_response("0902"); // VIN

    assert!(!rpm_response.is_empty());
    assert!(!speed_response.is_empty());
    assert!(!vin_response.is_empty());
}

#[test]
fn test_mock_data_pid_responses() {
    // Create mock responder
    let responder = MockResponder::new();

    // Load test data
    responder
        .load_car_scan_data("mock_data/gladiator.json")
        .unwrap();

    // Test specific PID responses
    let response = responder.generate_response("010C"); // Should get RPM response

    // Should contain "41 0C" (mode 1, PID 0C response)
    assert!(
        response.contains("41") && response.contains("0C"),
        "Unexpected response format: {}",
        response
    );
}
