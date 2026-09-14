//! BLE Response Tests
//!
//! Tests that responses pass through the external platform correctly.

use std::sync::{Arc, Mutex};

use obd_session_rs::external_platform::ExternalPlatform;
use obd_session_rs::platform::OBDPlatformInterface;

#[derive(Clone)]
struct ResponseCapture {
    entries: Arc<Mutex<Vec<(String, String)>>>,
}

impl ResponseCapture {
    fn new() -> Self {
        Self {
            entries: Arc::new(Mutex::new(Vec::new())),
        }
    }
    fn get_entries(&self) -> Vec<(String, String)> {
        self.entries.lock().unwrap().clone()
    }
}

fn setup_platform() -> (Arc<ExternalPlatform>, ResponseCapture) {
    extern "C" fn noop_send(
        _cmd: *const std::os::raw::c_char,
        _timeout: u32,
        _ctx: *mut std::ffi::c_void,
    ) {
    }

    let platform = Arc::new(ExternalPlatform::new(noop_send, std::ptr::null_mut()));
    let capture = ResponseCapture::new();
    let capture_for_cb = capture.clone();

    platform.set_data_callback(Some(Box::new(move |command: String, result| {
        let data = match result {
            Ok(r) => r.raw,
            Err(e) => format!("ERROR: {:?}", e),
        };
        capture_for_cb.entries.lock().unwrap().push((command, data));
    })));
    platform.set_completion_callback(Some(Box::new(|_cmd: String| {})));

    (platform, capture)
}

/// Responses pass through to the data callback.
#[test]
fn test_responses_pass_through() {
    let (platform, capture) = setup_platform();

    platform.receive_response("010C".to_string(), "7E8 04 41 0C 1A F8".to_string());
    platform.receive_response("010D".to_string(), "7E8 03 41 0D 5C".to_string());
    platform.receive_response("0105".to_string(), "7E8 03 41 05 4B".to_string());

    let entries = capture.get_entries();
    assert_eq!(entries.len(), 3);
    assert!(entries[0].1.contains("41 0C"));
    assert!(entries[1].1.contains("41 0D"));
    assert!(entries[2].1.contains("41 05"));
}

/// AT commands pass through.
#[test]
fn test_at_commands_pass_through() {
    let (platform, capture) = setup_platform();

    platform.receive_response("ATE0".to_string(), "OK".to_string());
    platform.receive_response("ATSH7E0".to_string(), "OK".to_string());

    assert_eq!(capture.get_entries().len(), 2);
}

/// Error responses pass through.
#[test]
fn test_error_responses_pass_through() {
    let (platform, capture) = setup_platform();

    platform.receive_response("010C".to_string(), "NO DATA".to_string());
    platform.receive_response("010D".to_string(), "CAN ERROR".to_string());

    let entries = capture.get_entries();
    assert_eq!(entries.len(), 2);
    assert_eq!(entries[0].1, "NO DATA");
    assert_eq!(entries[1].1, "CAN ERROR");
}

/// Multi-controller responses pass through as one response.
#[test]
fn test_multi_controller_passes() {
    let (platform, capture) = setup_platform();

    platform.receive_response(
        "0101".to_string(),
        "7E8 06 41 01 86 07 EF 80 7E9 06 41 01 81 00 00 00".to_string(),
    );

    let entries = capture.get_entries();
    assert_eq!(entries.len(), 1);
    assert!(entries[0].1.contains("7E8"));
    assert!(entries[0].1.contains("7E9"));
}
