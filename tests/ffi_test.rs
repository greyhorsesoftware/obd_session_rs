// RS7: the mock-platform trio (obd_create_session / obd_get_api_handle /
// obd_destroy_session) is compiled only under the `ffi-test` feature —
// run these tests with `cargo test --test ffi_test --features ffi-test`.
#![cfg(feature = "ffi-test")]

use std::ffi::CStr;
use std::os::raw::{c_char, c_int};
use std::time::Duration;

use obd_session_rs::ffi::*;

// Helper functions for FFI testing
fn c_string_to_string(cstr: *const c_char) -> Option<String> {
    if cstr.is_null() {
        return None;
    }
    unsafe { CStr::from_ptr(cstr).to_str().ok().map(|s| s.to_string()) }
}

#[test]
fn test_ffi_interface() {
    // Initialize library
    let init_result = obd_initialize();
    assert_eq!(init_result.code, OBDResult::Success);

    // Create session with callback
    // Mock sessions now return JSON responses through this callback
    static mut RESPONSE_COUNT: usize = 0;
    static mut LAST_RESPONSE: Option<String> = None;

    extern "C" fn test_callback(_cmd: *const c_char, success: c_int, response: *const c_char) {
        unsafe {
            RESPONSE_COUNT += 1;
            if success != 0 {
                if let Some(resp_str) = c_string_to_string(response) {
                    LAST_RESPONSE = Some(resp_str);
                }
            }
            // The strings are borrowed for the duration of this call (scoped
            // CStrings owned by Rust) — copy them if needed, but do NOT free them.
        }
    }

    let session = obd_create_session(test_callback, std::ptr::null(), std::ptr::null());
    assert!(!session.is_null());

    let api_handle = obd_get_api_handle(session);
    assert!(!api_handle.is_null());

    // Send a test command
    let cmd_result = obd_send_command(api_handle, b"010C\0".as_ptr() as *const c_char, 1000);
    assert_eq!(cmd_result.code, OBDResult::Success);

    // Wait a bit for response (mock platform has ~50ms delay)
    std::thread::sleep(Duration::from_millis(200));

    // Check that we got a response (now JSON format)
    #[allow(static_mut_refs)]
    unsafe {
        assert!(
            RESPONSE_COUNT > 0,
            "Should have received at least one response"
        );
        assert!(LAST_RESPONSE.is_some(), "Should have received a response");

        // Response should be valid JSON containing the command
        let response = LAST_RESPONSE.as_ref().unwrap();
        assert!(
            response.contains("\"type\""),
            "Response should be valid JSON: {}",
            response
        );
        assert!(
            response.contains("010C") || response.contains("obd_data"),
            "Response should contain command or be obd_data type: {}",
            response
        );
    }

    // Get stats
    let stats_json = obd_get_stats(api_handle);
    assert!(!stats_json.is_null());
    let stats_str = unsafe { CStr::from_ptr(stats_json).to_str().unwrap() };
    assert!(stats_str.contains("total_commands"));
    obd_free_string(stats_json);

    // Cleanup
    let destroy_api_result = obd_destroy_api_handle(api_handle);
    assert_eq!(destroy_api_result.code, OBDResult::Success);

    let destroy_session_result = obd_destroy_session(session);
    assert_eq!(destroy_session_result.code, OBDResult::Success);
}

// --- Wrong-type session pointer handling (pointer tagging) ---

extern "C" fn noop_response(_cmd: *const c_char, _success: c_int, _resp: *const c_char) {}
extern "C" fn noop_send(_cmd: *const c_char, _timeout: u32, _ctx: *mut std::ffi::c_void) {}
extern "C" fn noop_discover(_json: *const c_char, _ctx: *mut std::ffi::c_void) {}
extern "C" fn noop_connect(_success: c_int, _err: *const c_char, _ctx: *mut std::ffi::c_void) {}
extern "C" fn noop_disconnect(_ctx: *mut std::ffi::c_void) {}

fn make_external_session() -> *mut OBDSession {
    obd_create_session_with_platform(
        noop_send,
        noop_response,
        std::ptr::null_mut(),
        std::ptr::null(),
        std::ptr::null(),
        std::ptr::null(),
        noop_discover,
        noop_connect,
        noop_disconnect,
        std::ptr::null(),
    )
}

#[test]
fn external_accessor_on_mock_session_errors_no_crash() {
    let session = obd_create_session(noop_response, std::ptr::null(), std::ptr::null());
    assert!(!session.is_null());

    // Calling the external-only accessor on a mock session must fail loudly (null),
    // not blindly cast into the wrong wrapper type.
    let handle = obd_get_external_api_handle(session);
    assert!(
        handle.is_null(),
        "external accessor must reject a mock session"
    );

    let msg = unsafe { CStr::from_ptr(obd_get_last_error()).to_str().unwrap() };
    assert!(msg.contains("mismatch"), "last error: {msg}");

    // Destroying with the wrong destructor must also fail (magic mismatch)...
    let wrong = obd_destroy_external_session(session);
    assert_eq!(wrong.code, OBDResult::Error);

    // ...and the correct destructor still works (the session was not consumed).
    let ok = obd_destroy_session(session);
    assert_eq!(ok.code, OBDResult::Success);
}

#[test]
fn mock_accessor_on_external_session_errors_no_crash() {
    let session = make_external_session();
    assert!(!session.is_null());

    // Calling the mock-only accessor on an external session must fail (null).
    let handle = obd_get_api_handle(session);
    assert!(
        handle.is_null(),
        "mock accessor must reject an external session"
    );

    // Wrong destructor fails, correct one succeeds.
    let wrong = obd_destroy_session(session);
    assert_eq!(wrong.code, OBDResult::Error);

    let ok = obd_destroy_external_session(session);
    assert_eq!(ok.code, OBDResult::Success);
}
