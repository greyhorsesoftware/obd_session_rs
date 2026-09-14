//! C-compatible Foreign Function Interface (FFI) for Swift integration
//!
//! This module provides a C-compatible interface that Swift applications can use
//! to interact with the obd_session_rs library. All functions use C calling conventions
//! and handle memory management appropriately for cross-language usage.

use std::cell::RefCell;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_void};
use std::ptr;
use std::sync::Arc;
use uuid::Uuid;

use serde::Deserialize;

#[cfg(feature = "ffi-test")]
use crate::external_platform::create_mock_external_platform;
use crate::external_platform::{
    ConnectResultCallback, DisconnectRequestCallback, DiscoverConnectorsCallback, ExternalPlatform,
};
#[cfg(feature = "ffi-test")]
use crate::mock_responder::MockExternalContext;
use crate::session_manager::{OBDSessionManager, SessionAPIHandle};
use crate::{config::OBDSessionConfig, platform::ConnectionStatus};

// Opaque types for Swift to hold pointers to Rust objects
// These are never dereferenced in Swift, just passed back to Rust functions

/// Opaque type representing an OBD session
#[repr(C)]
pub struct OBDSession {
    _private: *mut c_void,
}

/// Opaque type representing a session API handle
#[repr(C)]
pub struct OBDAPIHandle {
    _private: *mut c_void,
}

/// Result codes for FFI operations
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OBDResult {
    Success = 0,
    Error = 1,
}

/// Error information structure
#[repr(C)]
pub struct OBDError {
    pub code: OBDResult,
    pub message: *mut c_char,
}

// Helper functions for memory management

/// Convert Rust String to C string (caller must free with obd_free_string)
fn string_to_c_string(s: String) -> *mut c_char {
    match CString::new(s) {
        Ok(cstr) => cstr.into_raw(),
        Err(_) => ptr::null_mut(), // String contained null bytes
    }
}

/// Convert C string to Rust String
fn c_string_to_string(cstr: *const c_char) -> Option<String> {
    if cstr.is_null() {
        return None;
    }
    unsafe { CStr::from_ptr(cstr).to_str().ok().map(|s| s.to_string()) }
}

/// Create error structure
fn create_error(message: String) -> OBDError {
    OBDError {
        code: OBDResult::Error,
        message: string_to_c_string(message),
    }
}

/// Create success result
fn create_success() -> OBDError {
    OBDError {
        code: OBDResult::Success,
        message: ptr::null_mut(),
    }
}

// ============================================================================
// Panic containment + last-error reporting for the FFI boundary
// ============================================================================

thread_local! {
    /// Last error message for the current thread, surfaced by `obd_get_last_error`.
    static LAST_ERROR: RefCell<Option<CString>> = const { RefCell::new(None) };
}

/// Record a message retrievable via `obd_get_last_error`.
fn set_last_error(msg: String) {
    LAST_ERROR.with(|e| {
        *e.borrow_mut() = CString::new(msg).ok();
    });
}

/// Extract a human-readable message from a panic payload.
fn panic_message(payload: &(dyn std::any::Any + Send)) -> String {
    if let Some(s) = payload.downcast_ref::<&str>() {
        (*s).to_string()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "unknown panic".to_string()
    }
}

/// Run an FFI body with panic containment. On panic, record the message for
/// `obd_get_last_error` and return `on_panic` (the function's error value)
/// instead of unwinding across the C ABI (which is undefined behavior).
fn ffi_guard<R>(on_panic: R, f: impl FnOnce() -> R) -> R {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(r) => r,
        Err(payload) => {
            set_last_error(format!(
                "panic in FFI call: {}",
                panic_message(payload.as_ref())
            ));
            on_panic
        }
    }
}

/// `OBDError` value returned when a call is aborted by a contained panic or a
/// pointer-type mismatch. The message (if any) is available via `obd_get_last_error`.
fn ffi_error_result() -> OBDError {
    OBDError {
        code: OBDResult::Error,
        message: ptr::null_mut(),
    }
}

// Magic tags for the two wrapper types that share the opaque `*mut OBDSession`.
// Stored as the first field of each `#[repr(C)]` wrapper and checked on every
// cast, so a mock pointer can never be treated as an external one (or vice
// versa) — doing so would be heap type-confusion undefined behavior.
#[cfg(feature = "ffi-test")]
const MOCK_SESSION_MAGIC: u32 = 0x4D4F_434B; // "MOCK"
const EXTERNAL_SESSION_MAGIC: u32 = 0x4558_5450; // "EXTP"

/// Borrow the pointer as a mock-session wrapper, or record an error and return
/// `None` if it is null or tagged as a different session type.
#[cfg(feature = "ffi-test")]
unsafe fn as_mock_session<'a>(session: *mut OBDSession) -> Option<&'a MockSessionWrapper> {
    if session.is_null() {
        return None;
    }
    if *(session as *const u32) != MOCK_SESSION_MAGIC {
        set_last_error("session pointer type mismatch: expected a mock session".to_string());
        return None;
    }
    Some(&*(session as *const MockSessionWrapper))
}

/// Borrow the pointer as an external-session wrapper, or record an error and
/// return `None` if it is null or tagged as a different session type.
unsafe fn as_external_session<'a>(session: *mut OBDSession) -> Option<&'a ExternalSessionWrapper> {
    if session.is_null() {
        return None;
    }
    if *(session as *const u32) != EXTERNAL_SESSION_MAGIC {
        set_last_error("session pointer type mismatch: expected an external session".to_string());
        return None;
    }
    Some(&*(session as *const ExternalSessionWrapper))
}

// ============================================================================
// Marshalling helpers (RS3) — the shared per-export skeleton: panic guard,
// null-arg check, pointer tag check / deref, and the common result tails.
// Each caller supplies its exact pre-existing null-arg error value, so
// converted exports return byte-identical outputs. Deliberately private:
// they take raw pointers, and keeping them non-pub means clippy's
// `not_unsafe_ptr_arg_deref` does not apply.
// ============================================================================

/// Parse a UUID argument passed as a C string (null/invalid → `None`).
fn parse_uuid_arg(ptr: *const c_char) -> Option<Uuid> {
    c_string_to_string(ptr).and_then(|s| Uuid::parse_str(&s).ok())
}

/// The common `Result` tail: `Ok` → success, `Err` → `create_error` with the
/// error's `Display` text.
fn result_to_obd_error<T, E: std::fmt::Display>(result: Result<T, E>) -> OBDError {
    match result {
        Ok(_) => create_success(),
        Err(e) => create_error(format!("{}", e)),
    }
}

/// Guard + null check + deref for exports taking a `*mut OBDAPIHandle` and
/// returning `OBDError`. `args_null` extends the null check to the export's
/// other required pointer args; `null_msg` is the exact message the export
/// returns in that case.
fn with_api_error(
    api_handle: *mut OBDAPIHandle,
    args_null: bool,
    null_msg: &str,
    f: impl FnOnce(&SessionAPIHandle) -> OBDError,
) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if api_handle.is_null() || args_null {
            return create_error(null_msg.to_string());
        }
        f(unsafe { &*(api_handle as *mut SessionAPIHandle) })
    })
}

/// Same skeleton for API-handle exports returning `*mut c_char`;
/// `null_value` is the exact string returned when the null check fails.
fn with_api_str(
    api_handle: *mut OBDAPIHandle,
    args_null: bool,
    null_value: &str,
    f: impl FnOnce(&SessionAPIHandle) -> *mut c_char,
) -> *mut c_char {
    ffi_guard(ptr::null_mut(), || {
        if api_handle.is_null() || args_null {
            return string_to_c_string(null_value.to_string());
        }
        f(unsafe { &*(api_handle as *mut SessionAPIHandle) })
    })
}

/// The subscription call-through skeleton: null handle/args → "Invalid
/// parameters", unparseable subscription id → "Invalid subscription ID",
/// then `f(handle, id)`.
fn with_api_sub(
    api_handle: *mut OBDAPIHandle,
    subscription_id_str: *const c_char,
    args_null: bool,
    f: impl FnOnce(&SessionAPIHandle, Uuid) -> OBDError,
) -> OBDError {
    with_api_error(
        api_handle,
        subscription_id_str.is_null() || args_null,
        "Invalid parameters",
        |handle_ref| match parse_uuid_arg(subscription_id_str) {
            Some(id) => f(handle_ref, id),
            None => create_error("Invalid subscription ID".to_string()),
        },
    )
}

/// Tag-checked borrow of the external-session wrapper: run `f`, or return
/// `err` when the pointer is tagged as a different session type (the
/// mismatch message is recorded by `as_external_session`). Callers must
/// null-check `session` first, exactly as the open-coded version did.
fn external_or<R>(
    session: *mut OBDSession,
    err: R,
    f: impl FnOnce(&ExternalSessionWrapper) -> R,
) -> R {
    match unsafe { as_external_session(session) } {
        Some(wrapper) => f(wrapper),
        None => err,
    }
}

/// Guard + null check + tag check for external-session exports returning
/// `OBDError`. Tag mismatch returns the same `ffi_error_result()` every
/// open-coded export used.
fn with_external_error(
    session: *mut OBDSession,
    args_null: bool,
    null_msg: &str,
    f: impl FnOnce(&ExternalSessionWrapper) -> OBDError,
) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if session.is_null() || args_null {
            return create_error(null_msg.to_string());
        }
        external_or(session, ffi_error_result(), f)
    })
}

/// Same skeleton for the `c_int` convention (0 = error / no-op).
fn with_external_int(
    session: *mut OBDSession,
    args_null: bool,
    f: impl FnOnce(&ExternalSessionWrapper) -> c_int,
) -> c_int {
    ffi_guard(0, || {
        if session.is_null() || args_null {
            return 0;
        }
        external_or(session, 0, f)
    })
}

/// Same skeleton for pointer-returning exports (NULL = error).
fn with_external_ptr<T>(
    session: *mut OBDSession,
    args_null: bool,
    f: impl FnOnce(&ExternalSessionWrapper) -> *mut T,
) -> *mut T {
    ffi_guard(ptr::null_mut(), || {
        if session.is_null() || args_null {
            return ptr::null_mut();
        }
        external_or(session, ptr::null_mut(), f)
    })
}

/// Get the last error recorded on the calling thread (e.g. a contained panic
/// message or a pointer-type mismatch), or a generic string if none.
///
/// The returned pointer is valid until the next FFI call on the same thread;
/// callers should copy it immediately. Do NOT free it.
#[no_mangle]
pub extern "C" fn obd_get_last_error() -> *const c_char {
    LAST_ERROR.with(|e| match &*e.borrow() {
        Some(cstr) => cstr.as_ptr(),
        None => c"No error".as_ptr(),
    })
}

// FFI Functions

/// Initialize the OBD session library
/// Call this once at application startup
#[no_mangle]
pub extern "C" fn obd_initialize() -> OBDError {
    ffi_guard(ffi_error_result(), || {
        // Any global initialization can go here
        create_success()
    })
}

/// Parse a raw OBD response for a command into per-controller data bytes — standalone (no session).
///
/// # Parameters
/// * `command` - The request PID/command (e.g. "220307", "010C") — used to strip echo bytes.
/// * `response` - The raw adapter response (may be multi-controller / multi-frame ISO-TP).
///
/// # Returns
/// JSON object keyed by controller ID (caller must free with `obd_free_string`), e.g.
/// `{"7E0":{"raw_hex":"...","data_bytes":[43,159],"controller_id":"7E0","is_valid":true}}`
/// or `"{}"` if the response can't be parsed.
#[no_mangle]
pub extern "C" fn obd_parse_response(
    command: *const c_char,
    response: *const c_char,
) -> *mut c_char {
    ffi_guard(ptr::null_mut(), || {
        let Some(command) = c_string_to_string(command) else {
            return string_to_c_string("{}".to_string());
        };
        let Some(response) = c_string_to_string(response) else {
            return string_to_c_string("{}".to_string());
        };
        // Sniffed variant: session-less callers (PID editor test-data preview) may paste samples
        // from either an 11-bit or a 29-bit adapter log — detect per call instead of assuming 11-bit.
        match crate::response_parser::parse_response_sniffed(&command, &response) {
            Some(parsed) => {
                let json = serde_json::to_string(&parsed.all_controllers)
                    .unwrap_or_else(|_| "{}".to_string());
                string_to_c_string(json)
            }
            None => string_to_c_string("{}".to_string()),
        }
    })
}

#[cfg(test)]
mod parse_response_ffi_tests {
    use super::*;
    use std::ffi::CString;

    #[test]
    fn obd_parse_response_extracts_mode22_data_bytes() {
        let cmd = CString::new("220307").unwrap();
        let resp = CString::new("7E8 05 62 03 07 2B 9F").unwrap();
        let ptr = obd_parse_response(cmd.as_ptr(), resp.as_ptr());
        let json = unsafe { CStr::from_ptr(ptr).to_str().unwrap().to_string() };
        obd_free_string(ptr);
        // 0x2B = 43, 0x9F = 159, after stripping 7E8 / PCI / 62 / 0307. Controller key is the
        // uniform request-id key: 7E8 → "7E0" (GB14).
        assert!(json.contains("\"7E0\""), "json: {json}");
        assert!(json.contains("\"data_bytes\":[43,159]"), "json: {json}");
        assert!(json.contains("\"is_valid\":true"), "json: {json}");
    }

    #[test]
    fn obd_parse_response_handles_garbage() {
        let cmd = CString::new("220307").unwrap();
        let resp = CString::new("NO DATA").unwrap();
        let ptr = obd_parse_response(cmd.as_ptr(), resp.as_ptr());
        let json = unsafe { CStr::from_ptr(ptr).to_str().unwrap().to_string() };
        obd_free_string(ptr);
        // Never a crash; always a JSON object.
        assert!(json.starts_with('{'), "json: {json}");
    }
}

/// Wrapper struct for mock session
///
/// `#[repr(C)]` with `magic` first so the tag can be read from a `*mut OBDSession`
/// regardless of which wrapper type the pointer actually holds. The mock
/// context itself is `&'static` (leaked by `create_mock_external_platform` —
/// see the safety note there), so the wrapper no longer owns it.
#[cfg(feature = "ffi-test")]
#[repr(C)]
struct MockSessionWrapper {
    magic: u32,
    #[allow(dead_code)]
    session: OBDSessionManager,
    #[allow(dead_code)]
    context: &'static MockExternalContext,
}

/// Create a new OBD session with mock platform (for testing)
///
/// # Parameters
/// * `response_callback` - Function called with (command, success, response_or_error)
/// * `log_base_path` - Base path for session logs (NULL to disable logging)
/// * `session_name` - Name for log file: {log_base_path}/{session_name}.log (NULL = auto-generate UUID)
///
/// # Returns
/// Opaque session pointer or null on error
///
/// # Note
/// This uses the mock external platform which validates the same code path
/// that production Swift would use with real Bluetooth.
///
/// RS7: test-only export — compiled only with `--features ffi-test`
/// (tests/ffi_test.rs); never shipped in the app-linked library.
#[cfg(feature = "ffi-test")]
#[no_mangle]
pub extern "C" fn obd_create_session(
    response_callback: extern "C" fn(*const c_char, c_int, *const c_char),
    log_base_path: *const c_char,
    session_name: *const c_char,
) -> *mut OBDSession {
    ffi_guard(ptr::null_mut(), || {
        // Create mock external platform
        let (platform, context) = create_mock_external_platform();

        // Create session config with optional logging
        let mut config = OBDSessionConfig::default();
        if !log_base_path.is_null() {
            config.log_base_path = c_string_to_string(log_base_path);
            config.session_name = if session_name.is_null() {
                None
            } else {
                c_string_to_string(session_name)
            };
        }

        // Create session manager with JSON callback that forwards to C callback
        let session = match OBDSessionManager::new(
            platform as Arc<dyn crate::platform::OBDPlatformInterface>,
            config,
            move |json_response| {
                // Scoped CStrings — the callback only borrows the pointers, so they
                // are freed when this closure returns (mirrors the platform path).
                if let Ok(resp_cstr) = CString::new(json_response) {
                    let empty_cmd = CString::new("").unwrap();
                    response_callback(empty_cmd.as_ptr(), 1, resp_cstr.as_ptr());
                }
            },
        ) {
            Ok(session) => session,
            Err(e) => {
                eprintln!("Failed to create session: {:?}", e);
                return ptr::null_mut();
            }
        };

        // Wrap session and context together so context stays alive
        let wrapper = Box::new(MockSessionWrapper {
            magic: MOCK_SESSION_MAGIC,
            session,
            context,
        });
        Box::into_raw(wrapper) as *mut OBDSession
    })
}

/// Destroy an OBD session and free resources
/// Use this for sessions created with obd_create_session (mock platform)
#[cfg(feature = "ffi-test")]
#[no_mangle]
pub extern "C" fn obd_destroy_session(session: *mut OBDSession) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if session.is_null() {
            return create_error("Session pointer is null".to_string());
        }

        unsafe {
            if *(session as *const u32) != MOCK_SESSION_MAGIC {
                return create_error(
                    "session pointer type mismatch: expected a mock session".to_string(),
                );
            }
            let _ = Box::from_raw(session as *mut MockSessionWrapper);
        }

        create_success()
    })
}

/// Get API handle from session (for mock sessions created with obd_create_session)
#[cfg(feature = "ffi-test")]
#[no_mangle]
pub extern "C" fn obd_get_api_handle(session: *mut OBDSession) -> *mut OBDAPIHandle {
    ffi_guard(ptr::null_mut(), || {
        if session.is_null() {
            return ptr::null_mut();
        }

        unsafe {
            let wrapper_ref = match as_mock_session(session) {
                Some(w) => w,
                None => return ptr::null_mut(),
            };
            let api_handle = wrapper_ref.session.api_handle();

            // Wrap in opaque type
            let boxed_handle = Box::new(api_handle);
            Box::into_raw(boxed_handle) as *mut OBDAPIHandle
        }
    })
}

/// Destroy API handle
#[no_mangle]
pub extern "C" fn obd_destroy_api_handle(handle: *mut OBDAPIHandle) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if handle.is_null() {
            return create_error("API handle pointer is null".to_string());
        }

        unsafe {
            let _ = Box::from_raw(handle as *mut SessionAPIHandle);
        }

        create_success()
    })
}

/// Create a subscription for PIDs (all continuous)
/// Returns subscription ID as string (caller must free with obd_free_string)
///
/// For run-once PIDs, use obd_create_subscription_with_run_once instead.
#[no_mangle]
pub extern "C" fn obd_create_subscription(
    api_handle: *mut OBDAPIHandle,
    name: *const c_char,        // Optional human-readable name, null for none
    pids: *const *const c_char, // Array of C strings
    pid_count: usize,
    target_controller: *const c_char, // Optional controller, null for default
) -> *mut c_char {
    ffi_guard(ptr::null_mut(), || {
        // Delegate to the full version with no run-once PIDs
        obd_create_subscription_with_run_once(
            api_handle,
            name,
            pids,
            pid_count,
            target_controller,
            std::ptr::null(),
            0,
        )
    })
}

/// Create a subscription for PIDs with run-once support
///
/// PIDs in run_once_pids array will execute exactly once then stop.
/// PIDs not in run_once_pids will run continuously.
///
/// Returns subscription ID as string (caller must free with obd_free_string)
///
/// # Parameters
/// * `api_handle` - API handle from obd_get_api_handle
/// * `pids` - Array of PID strings to subscribe to
/// * `pid_count` - Number of PIDs in array
/// * `target_controller` - Optional ECU controller (null for default)
/// * `run_once_pids` - Array of PID strings that should only run once (can be null)
/// * `run_once_count` - Number of run-once PIDs (0 if run_once_pids is null)
#[no_mangle]
pub extern "C" fn obd_create_subscription_with_run_once(
    api_handle: *mut OBDAPIHandle,
    name: *const c_char,
    pids: *const *const c_char,
    pid_count: usize,
    target_controller: *const c_char,
    run_once_pids: *const *const c_char,
    run_once_count: usize,
) -> *mut c_char {
    with_api_str(
        api_handle,
        pids.is_null(),
        "ERROR: Invalid parameters",
        |handle_ref| {
            // Convert C string array to Rust Vec<String>
            let mut pid_vec = Vec::new();
            for i in 0..pid_count {
                unsafe {
                    let pid_ptr = *pids.add(i);
                    if let Some(pid_str) = c_string_to_string(pid_ptr) {
                        pid_vec.push(pid_str);
                    }
                }
            }

            // Convert run-once PIDs to a set for quick lookup
            let mut run_once_set = std::collections::HashSet::new();
            if !run_once_pids.is_null() && run_once_count > 0 {
                for i in 0..run_once_count {
                    unsafe {
                        let pid_ptr = *run_once_pids.add(i);
                        if let Some(pid_str) = c_string_to_string(pid_ptr) {
                            run_once_set.insert(pid_str);
                        }
                    }
                }
            }

            // Build run_counts HashMap: run-once PIDs get Some(1), others get None (continuous)
            let run_counts: Option<std::collections::HashMap<String, Option<u32>>> =
                if run_once_set.is_empty() {
                    None
                } else {
                    let mut counts = std::collections::HashMap::new();
                    for pid in &pid_vec {
                        if run_once_set.contains(pid) {
                            counts.insert(pid.clone(), Some(1));
                        }
                    }
                    Some(counts)
                };

            // Convert target controller
            let controller = if target_controller.is_null() {
                None
            } else {
                c_string_to_string(target_controller)
            };

            let sub_name = if name.is_null() {
                None
            } else {
                c_string_to_string(name)
            };
            match handle_ref
                .create_subscription_with_run_counts(sub_name, pid_vec, controller, run_counts)
            {
                Ok(subscription_id) => string_to_c_string(subscription_id.to_string()),
                Err(e) => string_to_c_string(format!("ERROR: {:?}", e)),
            }
        },
    )
}

/// Start a subscription
#[no_mangle]
pub extern "C" fn obd_start_subscription(
    api_handle: *mut OBDAPIHandle,
    subscription_id_str: *const c_char,
) -> OBDError {
    with_api_sub(
        api_handle,
        subscription_id_str,
        false,
        |handle_ref, subscription_id| {
            result_to_obd_error(handle_ref.start_subscription(subscription_id))
        },
    )
}

/// Stop a subscription
#[no_mangle]
pub extern "C" fn obd_pause_subscription(
    api_handle: *mut OBDAPIHandle,
    subscription_id_str: *const c_char,
) -> OBDError {
    with_api_sub(
        api_handle,
        subscription_id_str,
        false,
        |handle_ref, subscription_id| {
            result_to_obd_error(handle_ref.pause_subscription(subscription_id))
        },
    )
}

/// Cancel a subscription
#[no_mangle]
pub extern "C" fn obd_cancel_subscription(
    api_handle: *mut OBDAPIHandle,
    subscription_id_str: *const c_char,
) -> OBDError {
    with_api_sub(
        api_handle,
        subscription_id_str,
        false,
        |handle_ref, subscription_id| {
            result_to_obd_error(handle_ref.cancel_subscription(subscription_id))
        },
    )
}

/// Add PIDs to an existing subscription
///
/// # Parameters
/// * `api_handle` - API handle
/// * `subscription_id_str` - Subscription ID
/// * `pids_json` - JSON array of PIDs to add, e.g., "[\"010C\", \"010D\"]"
/// * `target_controller` - Optional target controller (nullable)
#[no_mangle]
pub extern "C" fn obd_add_pids_to_subscription(
    api_handle: *mut OBDAPIHandle,
    subscription_id_str: *const c_char,
    pids_json: *const c_char,
    target_controller: *const c_char,
) -> OBDError {
    with_api_sub(
        api_handle,
        subscription_id_str,
        pids_json.is_null(),
        |handle_ref, subscription_id| {
            let Some(pids_str) = c_string_to_string(pids_json) else {
                return create_error("Invalid PIDs JSON".to_string());
            };
            let pids: Vec<String> = match serde_json::from_str(&pids_str) {
                Ok(p) => p,
                Err(e) => return create_error(format!("Failed to parse PIDs JSON: {}", e)),
            };
            let controller = if target_controller.is_null() {
                None
            } else {
                c_string_to_string(target_controller)
            };
            result_to_obd_error(handle_ref.add_pids_to_subscription(
                subscription_id,
                pids,
                controller,
            ))
        },
    )
}

/// Remove PIDs from an existing subscription
///
/// # Parameters
/// * `api_handle` - API handle
/// * `subscription_id_str` - Subscription ID
/// * `pids_json` - JSON array of PIDs to remove, e.g., "[\"010C\"]"
#[no_mangle]
pub extern "C" fn obd_remove_pids_from_subscription(
    api_handle: *mut OBDAPIHandle,
    subscription_id_str: *const c_char,
    pids_json: *const c_char,
) -> OBDError {
    with_api_sub(
        api_handle,
        subscription_id_str,
        pids_json.is_null(),
        |handle_ref, subscription_id| {
            let Some(pids_str) = c_string_to_string(pids_json) else {
                return create_error("Invalid PIDs JSON".to_string());
            };
            let pids: Vec<String> = match serde_json::from_str(&pids_str) {
                Ok(p) => p,
                Err(e) => return create_error(format!("Failed to parse PIDs JSON: {}", e)),
            };
            result_to_obd_error(handle_ref.remove_pids_from_subscription(subscription_id, pids))
        },
    )
}

/// Set timeout for a subscription
///
/// Override the default timeout for commands in this subscription.
/// Useful for run-once subscriptions with multi-frame responses (e.g., VIN).
///
/// # Parameters
/// * `api_handle` - API handle
/// * `subscription_id_str` - Subscription ID
/// * `timeout_ms` - Timeout in milliseconds (e.g., 10000 for 10 seconds)
#[no_mangle]
pub extern "C" fn obd_set_subscription_timeout(
    api_handle: *mut OBDAPIHandle,
    subscription_id_str: *const c_char,
    timeout_ms: u32,
) -> OBDError {
    with_api_sub(
        api_handle,
        subscription_id_str,
        false,
        |handle_ref, subscription_id| {
            result_to_obd_error(handle_ref.set_subscription_timeout(subscription_id, timeout_ms))
        },
    )
}

/// Send a single command
#[no_mangle]
pub extern "C" fn obd_send_command(
    api_handle: *mut OBDAPIHandle,
    command: *const c_char,
    timeout_ms: u32,
) -> OBDError {
    with_api_error(
        api_handle,
        command.is_null(),
        "Invalid parameters",
        |handle_ref| {
            let Some(cmd_str) = c_string_to_string(command) else {
                return create_error("Invalid command string".to_string());
            };
            result_to_obd_error(handle_ref.send_command(&cmd_str, timeout_ms))
        },
    )
}

/// Get command processor statistics as JSON string
/// Returns JSON string that caller must free with obd_free_string
#[no_mangle]
pub extern "C" fn obd_get_stats(api_handle: *mut OBDAPIHandle) -> *mut c_char {
    with_api_str(
        api_handle,
        false,
        "ERROR: Invalid API handle",
        |handle_ref| {
            match handle_ref.get_command_processor_stats() {
                Ok(stats) => {
                    // Include response mismatch metrics from subscription manager
                    let (mismatch_count, mismatch_total, mismatch_rate) = handle_ref
                        .get_response_mismatch_metrics()
                        .unwrap_or((0, 0, 0.0));

                    // BP2/B2: acquisition line + projected rate (model, not measurement)
                    let (plugin_id, pipe_limit, chunk_limit, projected_load, projected_fast_hz) =
                        handle_ref.get_acquisition_stats();
                    // Tier S: slot usage + broadcast ids while a stream is live.
                    let (stream_slots_used, stream_slot_count, stream_ids) = handle_ref
                        .get_stream_stats()
                        .unwrap_or((0, 0, String::new()));

                    let json = format!(
                        r#"{{"total_commands":{},"completed_commands":{},"failed_commands":{},"pids_per_second":{},"signals_per_second":{:.2},"average_completion_time_ms":{},"in_progress_commands":{},"response_mismatch_count":{},"response_total_count":{},"response_mismatch_rate":{:.6},"acquisition_plugin":"{}","pipe_limit":{},"chunk_limit":{},"projected_load":{:.3},"projected_fast_hz":{:.2},"stream_slots_used":{},"stream_slot_count":{},"stream_response_ids":"{}"}}"#,
                        stats.total_commands,
                        stats.completed_commands,
                        stats.failed_commands,
                        stats.pids_per_second(),
                        stats.signals_per_second(),
                        stats
                            .average_completion_time()
                            .map(|d| d.as_millis())
                            .unwrap_or(0),
                        stats.in_progress_commands,
                        mismatch_count,
                        mismatch_total,
                        mismatch_rate,
                        plugin_id,
                        pipe_limit,
                        chunk_limit,
                        projected_load,
                        projected_fast_hz,
                        stream_slots_used,
                        stream_slot_count,
                        stream_ids
                    );
                    string_to_c_string(json)
                }
                Err(e) => string_to_c_string(format!("ERROR: {:?}", e)),
            }
        },
    )
}

/// Reset command processor statistics
#[no_mangle]
pub extern "C" fn obd_reset_stats(api_handle: *mut OBDAPIHandle) -> OBDError {
    with_api_error(api_handle, false, "Invalid API handle", |handle_ref| {
        result_to_obd_error(handle_ref.reset_command_processor_stats())
    })
}

/// Set command rate limit (commands per second)
///
/// # Parameters
/// * `api_handle` - API handle
/// * `commands_per_second` - Maximum commands per second (e.g., 10.0 for 10 cmd/sec)
///
/// # Example
/// ```c
/// obd_set_rate_limit(handle, 20.0);  // 20 commands per second
/// obd_set_rate_limit(handle, 5.0);   // Slow down to 5 cmd/sec for stability
/// ```
#[no_mangle]
pub extern "C" fn obd_set_rate_limit(
    api_handle: *mut OBDAPIHandle,
    commands_per_second: f64,
) -> OBDError {
    with_api_error(api_handle, false, "Invalid API handle", |handle_ref| {
        if commands_per_second <= 0.0 {
            return create_error("Rate must be positive".to_string());
        }
        handle_ref.set_command_rate(commands_per_second);
        create_success()
    })
}

/// Set refresh rate tiers for PIDs in a subscription
///
/// Tiers control how often each PID is polled relative to others:
/// - 1 = Fast (every cycle) — RPM, speed, throttle
/// - 3 = Medium (every 3rd cycle) — fuel trims, intake temp
/// - 10 = Slow (every 10th cycle) — coolant temp, fuel level
///
/// # Parameters
/// * `api_handle` - API handle
/// * `subscription_id_str` - Subscription ID
/// * `tiers_json` - JSON object mapping PID strings to divisor integers,
///   e.g., `{"010C": 1, "0105": 10}`. Pass null to use built-in defaults.
///
/// # Example
/// ```c
/// obd_set_subscription_tiers(handle, sub_id, "{\"010C\": 1, \"0105\": 10}");
/// obd_set_subscription_tiers(handle, sub_id, NULL);  // use defaults
/// ```
#[no_mangle]
pub extern "C" fn obd_set_subscription_tiers(
    api_handle: *mut OBDAPIHandle,
    subscription_id_str: *const c_char,
    tiers_json: *const c_char,
) -> OBDError {
    with_api_sub(
        api_handle,
        subscription_id_str,
        false,
        |handle_ref, subscription_id| {
            // Parse tiers from JSON, or use defaults if null
            let tiers = if tiers_json.is_null() {
                crate::subscription::default_pid_tiers()
            } else {
                let Some(tiers_str) = c_string_to_string(tiers_json) else {
                    return create_error("Invalid tiers JSON".to_string());
                };

                // Parse as {"PID": divisor_int, ...}
                let raw_tiers: std::collections::HashMap<String, u8> =
                    match serde_json::from_str(&tiers_str) {
                        Ok(t) => t,
                        Err(e) => {
                            return create_error(format!("Failed to parse tiers JSON: {}", e))
                        }
                    };

                // Empty JSON object means use defaults
                if raw_tiers.is_empty() {
                    crate::subscription::default_pid_tiers()
                } else {
                    raw_tiers
                        .into_iter()
                        .map(|(pid, divisor)| {
                            (pid, crate::subscription::RefreshTier::from_u8(divisor))
                        })
                        .collect()
                }
            };

            result_to_obd_error(handle_ref.set_subscription_tiers(subscription_id, tiers))
        },
    )
}

/// Install a session monitor callback that receives JSON for each command/response.
///
/// The callback receives a JSON-serialized SessionMonitorEntry for each command completion.
/// Only one callback can be active at a time — calling again replaces the previous one.
///
/// # Parameters
/// * `api_handle` - API handle
/// * `callback` - Function pointer receiving a null-terminated JSON string
#[no_mangle]
pub extern "C" fn obd_install_session_monitor(
    api_handle: *mut OBDAPIHandle,
    callback: extern "C" fn(*const c_char),
) -> OBDError {
    with_api_error(api_handle, false, "Invalid API handle", |handle_ref| {
        handle_ref.install_session_monitor_callback(Box::new(move |entry| {
            if let Ok(json) = serde_json::to_string(entry) {
                if let Ok(c_str) = std::ffi::CString::new(json) {
                    callback(c_str.as_ptr());
                }
            }
        }));
        create_success()
    })
}

/// Remove the session monitor callback.
#[no_mangle]
pub extern "C" fn obd_remove_session_monitor(api_handle: *mut OBDAPIHandle) -> OBDError {
    with_api_error(api_handle, false, "Invalid API handle", |handle_ref| {
        handle_ref.remove_session_monitor_callback();
        create_success()
    })
}

/// Get the last `count` session monitor entries as a JSON array string.
/// Caller must free the returned string with `obd_free_string`.
#[no_mangle]
pub extern "C" fn obd_get_session_history(
    api_handle: *mut OBDAPIHandle,
    count: u32,
) -> *mut c_char {
    with_api_str(api_handle, false, "[]", |handle_ref| {
        string_to_c_string(handle_ref.get_session_history(count as usize))
    })
}

/// Clear all session monitor history.
#[no_mangle]
pub extern "C" fn obd_clear_session_history(api_handle: *mut OBDAPIHandle) -> OBDError {
    with_api_error(api_handle, false, "Invalid API handle", |handle_ref| {
        handle_ref.clear_session_history();
        create_success()
    })
}

/// Install a callback that fires on every subscription change (PID add/remove).
/// The callback receives a JSON-serialized audit entry.
#[no_mangle]
pub extern "C" fn obd_install_subscription_change_callback(
    api_handle: *mut OBDAPIHandle,
    callback: extern "C" fn(*const c_char),
) -> OBDError {
    with_api_error(api_handle, false, "Invalid API handle", |handle_ref| {
        handle_ref.install_subscription_change_callback(Box::new(move |entry| {
            if let Ok(json) = serde_json::to_string(entry) {
                if let Ok(c_str) = std::ffi::CString::new(json) {
                    callback(c_str.as_ptr());
                }
            }
        }));
        create_success()
    })
}

/// Remove the subscription change callback.
#[no_mangle]
pub extern "C" fn obd_remove_subscription_change_callback(
    api_handle: *mut OBDAPIHandle,
) -> OBDError {
    with_api_error(api_handle, false, "Invalid API handle", |handle_ref| {
        handle_ref.remove_subscription_change_callback();
        create_success()
    })
}

/// Get a snapshot of all subscriptions with their PIDs, tiers, and state.
/// Returns a JSON array string. Caller must free with `obd_free_string`.
#[no_mangle]
pub extern "C" fn obd_get_subscriptions(api_handle: *mut OBDAPIHandle) -> *mut c_char {
    with_api_str(api_handle, false, "[]", |handle_ref| {
        string_to_c_string(handle_ref.get_subscriptions_snapshot())
    })
}

/// Get the audit log of PID add/remove operations.
/// Returns a JSON array of the last `count` entries (newest first), each with
/// action, subscription_id, pids, target_controller, and timestamp.
/// Caller must free with `obd_free_string`.
#[no_mangle]
pub extern "C" fn obd_get_subscription_audit_log(
    api_handle: *mut OBDAPIHandle,
    count: u32,
) -> *mut c_char {
    with_api_str(api_handle, false, "[]", |handle_ref| {
        string_to_c_string(handle_ref.get_subscription_audit_log(count as usize))
    })
}

/// Free a string allocated by the FFI functions
#[no_mangle]
pub extern "C" fn obd_free_string(s: *mut c_char) {
    ffi_guard((), || {
        if !s.is_null() {
            unsafe {
                let _ = CString::from_raw(s);
            }
        }
    })
}

/// Free an error structure
#[no_mangle]
pub extern "C" fn obd_free_error(error: OBDError) {
    ffi_guard((), || {
        obd_free_string(error.message);
    })
}

/// Get all known adapter error codes and their severity classification.
/// Returns JSON: {"errors": {"TIMEOUT": "transient", "DISCONNECTED": "fatal", ...}}
/// Swift calls this once at startup and uses these codes as the only error strings
/// passed to obd_external_receive_error.
/// Caller must free with obd_free_string.
#[no_mangle]
pub extern "C" fn obd_get_error_definitions() -> *mut c_char {
    ffi_guard(ptr::null_mut(), || {
        use crate::error::ADAPTER_ERRORS;
        let map: std::collections::HashMap<&str, &str> = ADAPTER_ERRORS
            .iter()
            .map(|e| (e.code, e.severity))
            .collect();
        let wrapper = serde_json::json!({ "errors": map });
        let json =
            serde_json::to_string(&wrapper).unwrap_or_else(|_| r#"{"errors":{}}"#.to_string());
        string_to_c_string(json)
    })
}

/// Get library version
#[no_mangle]
pub extern "C" fn obd_get_version() -> *mut c_char {
    ffi_guard(ptr::null_mut(), || {
        string_to_c_string(crate::VERSION.to_string())
    })
}

// ============================================================================
// External Platform FFI - For Swift/native OBD communication
// ============================================================================

/// Callback type for sending OBD commands to external platform (Swift)
///
/// # Parameters
/// * `command` - The OBD command to send (e.g., "010C")
/// * `timeout_ms` - Maximum time to wait for response
/// * `context` - User-provided context pointer (passed back from create)
pub type OBDSendCallback = extern "C" fn(*const c_char, u32, *mut c_void);

/// Callback type for receiving responses (same as before)
pub type OBDResponseCallback = extern "C" fn(*const c_char, c_int, *const c_char);

/// Wrapper struct to hold external platform reference
///
/// `#[repr(C)]` with `magic` first so the tag can be read from a `*mut OBDSession`
/// regardless of which wrapper type the pointer actually holds.
#[repr(C)]
struct ExternalSessionWrapper {
    magic: u32,
    #[allow(dead_code)] // Kept alive to maintain session lifecycle
    session: OBDSessionManager,
    platform: Arc<ExternalPlatform>,
    cache_path: Option<String>,
}

/// Create a new OBD session with an external platform implementation
///
/// Use this when Swift handles the actual OBD communication (e.g., Bluetooth).
/// Rust handles command queuing, subscriptions, rate limiting, and response processing.
///
/// # Parameters
/// * `send_callback` - Function Rust calls when a command needs to be sent
/// * `response_callback` - Function Rust calls to deliver responses back to Swift
/// * `context` - Opaque pointer passed to send_callback (e.g., Swift object reference)
///
/// # Returns
/// Opaque session pointer or null on error
///
/// # Example Flow
/// 1. Swift creates session with callbacks
/// 2. Swift calls obd_send_command or obd_create_subscription
/// 3. Rust calls send_callback with the OBD command
/// 4. Swift sends command over Bluetooth
/// 5. Swift receives Bluetooth response
/// 6. Swift calls obd_external_receive_response
/// 7. Rust processes response and calls response_callback
///
/// # Parameters
/// * `send_callback` - Function Rust calls when it needs to send a command
/// * `response_callback` - Function Rust calls to deliver JSON responses
/// * `context` - Opaque pointer passed to send_callback (e.g., your Swift object)
/// * `log_base_path` - Base path for session logs (NULL to disable logging)
/// * `session_name` - Name for log file: {log_base_path}/{session_name}.log (NULL = auto-generate UUID)
#[no_mangle]
pub extern "C" fn obd_create_session_with_platform(
    send_callback: OBDSendCallback,
    response_callback: OBDResponseCallback,
    context: *mut c_void,
    log_base_path: *const c_char,
    session_name: *const c_char,
    cache_path: *const c_char,
    discover_callback: DiscoverConnectorsCallback,
    connect_callback: ConnectResultCallback,
    disconnect_callback: DisconnectRequestCallback,
    init_commands_json: *const c_char,
) -> *mut OBDSession {
    ffi_guard(ptr::null_mut(), || {
        // Create external platform with connection callbacks
        let platform = Arc::new(ExternalPlatform::new_with_connection_callbacks(
            send_callback,
            Some(discover_callback),
            Some(connect_callback),
            Some(disconnect_callback),
            context,
        ));
        platform.attach_self(); // LH5: Rust transports feed the platform through this
        let platform_clone = Arc::clone(&platform);

        // Create session config with optional logging
        let mut config = OBDSessionConfig::default();
        if !log_base_path.is_null() {
            config.log_base_path = c_string_to_string(log_base_path);
            config.session_name = if session_name.is_null() {
                None
            } else {
                c_string_to_string(session_name)
            };
        }
        if !cache_path.is_null() {
            config.cache_path = c_string_to_string(cache_path);
        }

        // Parse init commands from JSON array (e.g., ["ATE0","ATH1","STDI"])
        if !init_commands_json.is_null() {
            if let Some(json_str) = c_string_to_string(init_commands_json) {
                if let Ok(commands) = serde_json::from_str::<Vec<String>>(&json_str) {
                    config.init_commands = commands;
                }
            }
        }

        // Create session manager with JSON callback that forwards to C callback
        let session = match OBDSessionManager::new(
            platform_clone as Arc<dyn crate::platform::OBDPlatformInterface>,
            config,
            move |json_response| {
                // Forward JSON response to Swift
                if let Some(resp_cstr) = CString::new(json_response).ok() {
                    // For JSON responses, command is empty, success=1
                    let empty_cmd = CString::new("").unwrap();
                    response_callback(empty_cmd.as_ptr(), 1, resp_cstr.as_ptr());
                }
            },
        ) {
            Ok(session) => session,
            Err(e) => {
                eprintln!("Failed to create session: {:?}", e);
                return ptr::null_mut();
            }
        };

        // Wrap session and platform together so we can access platform later
        let cache_path_str = if !cache_path.is_null() {
            c_string_to_string(cache_path)
        } else {
            None
        };
        let wrapper = Box::new(ExternalSessionWrapper {
            magic: EXTERNAL_SESSION_MAGIC,
            session,
            platform,
            cache_path: cache_path_str,
        });
        Box::into_raw(wrapper) as *mut OBDSession
    })
}

/// Get API handle from an external platform session
/// Use this instead of obd_get_api_handle for sessions created with obd_create_session_with_platform.
#[no_mangle]
pub extern "C" fn obd_get_external_api_handle(session: *mut OBDSession) -> *mut OBDAPIHandle {
    with_external_ptr(session, false, |wrapper_ref| {
        let api_handle = wrapper_ref.session.api_handle();

        let boxed_handle = Box::new(api_handle);
        Box::into_raw(boxed_handle) as *mut OBDAPIHandle
    })
}

/// Notify the session that OBD response data was received
///
/// Swift calls this when Bluetooth data arrives from the OBD device.
///
/// # Parameters
/// * `session` - Session pointer from obd_create_session_with_platform
/// * `command` - The command that was sent (for correlation)
/// * `response` - The response data received from OBD device
#[no_mangle]
pub extern "C" fn obd_external_receive_response(
    session: *mut OBDSession,
    command: *const c_char,
    response: *const c_char,
) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if session.is_null() || command.is_null() || response.is_null() {
            return create_error("Invalid parameters".to_string());
        }

        let Some(cmd) = c_string_to_string(command) else {
            return create_error("Invalid command string".to_string());
        };

        let Some(resp) = c_string_to_string(response) else {
            return create_error("Invalid response string".to_string());
        };

        external_or(session, ffi_error_result(), |wrapper| {
            wrapper.platform.receive_response(cmd, resp);
            create_success()
        })
    })
}

/// LH0: raw transport bytes from the host (BLE notify / RFCOMM read / EA
/// stream / mock-replay output). Framing — the `>` prompt, monitor lines, the
/// 5 s no-prompt fallback — happens in Rust; this call only appends and
/// returns, so it is safe on the host's I/O thread.
///
/// # Parameters
/// * `session` - Session pointer from obd_create_session_with_platform
/// * `bytes` / `len` - The chunk exactly as the transport delivered it
#[no_mangle]
pub extern "C" fn obd_external_receive_bytes(
    session: *mut OBDSession,
    bytes: *const u8,
    len: usize,
) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if session.is_null() || (bytes.is_null() && len > 0) {
            return create_error("Invalid parameters".to_string());
        }
        let data: &[u8] = if len == 0 {
            &[]
        } else {
            unsafe { std::slice::from_raw_parts(bytes, len) }
        };
        external_or(session, ffi_error_result(), |wrapper| {
            wrapper.platform.receive_bytes(data);
            create_success()
        })
    })
}

/// LH0: `simulateTimeout` / `simulateFatal` — the next completed response is
/// dropped and `code` (an obd_get_error_definitions key, e.g. "TIMEOUT" or
/// "UNABLE TO CONNECT") is delivered as the command's error. One-shot.
#[no_mangle]
pub extern "C" fn obd_external_inject_error(
    session: *mut OBDSession,
    code: *const c_char,
) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if session.is_null() || code.is_null() {
            return create_error("Invalid parameters".to_string());
        }
        let Some(code) = c_string_to_string(code) else {
            return create_error("Invalid error code string".to_string());
        };
        external_or(session, ffi_error_result(), |wrapper| {
            wrapper.platform.inject_error(&code);
            create_success()
        })
    })
}

/// LH0: register the host's Adapter Log sink. Rust reports what it framed
/// from the byte stream: `kind` ∈ "rx" (a completed response), "line" (a
/// monitor line), "dropped" (stale prompt / fallback / injected error).
#[no_mangle]
pub extern "C" fn obd_set_transport_log_cb(
    session: *mut OBDSession,
    cb: crate::external_platform::TransportLogCallback,
) -> OBDError {
    with_external_error(session, false, "Invalid session", |w| {
        w.platform.set_transport_log(cb);
        create_success()
    })
}

/// WIRE-LOG.c: host-side wire annotation — one NOTE row in the wire file
/// and the live-monitor ring (BLE canSend stalls, chunk splits, …).
#[no_mangle]
pub extern "C" fn obd_log_transport_note(
    session: *mut OBDSession,
    kind: *const c_char,
    text: *const c_char,
) -> OBDError {
    with_external_error(session, false, "Invalid session", |w| {
        let kind = c_string_to_string(kind).unwrap_or_default();
        let text = c_string_to_string(text).unwrap_or_default();
        w.platform.log_transport_note(&kind, &text);
        create_success()
    })
}

/// WIRE-LOG.c: flush the session + wire logs to disk now (resignActive /
/// will-terminate — a jetsam must not eat the buffered tail).
#[no_mangle]
pub extern "C" fn obd_flush_logs(session: *mut OBDSession) -> OBDError {
    with_external_error(session, false, "Invalid session", |w| {
        w.platform.flush_wire_log();
        create_success()
    })
}

/// Notify the session that a command failed
///
/// Swift calls this when an adapter or transport error occurs.
///
/// # Parameters
/// * `session` - Session pointer from obd_create_session_with_platform
/// * `command` - The command that failed
/// * `error_code` - Standardized error code string from obd_get_error_definitions()
///                  (e.g., "TIMEOUT", "NO_DATA", "DISCONNECTED")
#[no_mangle]
pub extern "C" fn obd_external_receive_error(
    session: *mut OBDSession,
    command: *const c_char,
    error_code: *const c_char,
) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if session.is_null() || command.is_null() || error_code.is_null() {
            return create_error("Invalid parameters".to_string());
        }

        let Some(cmd) = c_string_to_string(command) else {
            return create_error("Invalid command string".to_string());
        };

        let Some(code) = c_string_to_string(error_code) else {
            return create_error("Invalid error code string".to_string());
        };

        external_or(session, ffi_error_result(), |wrapper| {
            wrapper.platform.receive_error(cmd, code);
            create_success()
        })
    })
}

/// Notify the session that connection status changed
///
/// Swift calls this when Bluetooth connection state changes.
///
/// # Parameters
/// * `session` - Session pointer from obd_create_session_with_platform
/// * `status` - Connection status (0=connected, 1=disconnected, 2=connecting, 3=failed)
/// * `reason` - Optional reason string (can be null)
#[no_mangle]
pub extern "C" fn obd_external_update_connection(
    session: *mut OBDSession,
    status: c_int,
    reason: *const c_char,
) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if session.is_null() {
            return create_error("Invalid session".to_string());
        }

        let conn_status = match status {
            0 => ConnectionStatus::Connected,
            1 => ConnectionStatus::Disconnected,
            2 => ConnectionStatus::Connecting,
            3 => ConnectionStatus::ConnectionFailed,
            4 => ConnectionStatus::Reconnecting,
            _ => return create_error("Invalid status code".to_string()),
        };

        let reason_str = c_string_to_string(reason);

        external_or(session, ffi_error_result(), |wrapper| {
            wrapper
                .platform
                .update_connection_status(conn_status, reason_str);
            create_success()
        })
    })
}

// ============================================================================
// Connection Management FFI
// ============================================================================

/// Swift calls this to deliver ONE scan round's snapshot (the answer to a
/// `scan_connectors_round` request). Consumes the pending round callback
/// stored in ExternalPlatform; the discovery engine prunes/merges/emits.
///
/// # Parameters
/// * `session` - Session pointer
/// * `connectors_json` - JSON array of ConnectorInfo, e.g.
///   `[{"id":"AA:BB:CC","name":"OBDLink MX+","connector_type":"classic"}]`
#[no_mangle]
pub extern "C" fn obd_external_discover_result(
    session: *mut OBDSession,
    connectors_json: *const c_char,
) -> OBDError {
    with_external_error(session, false, "Invalid session", |wrapper| {
        let json = c_string_to_string(connectors_json).unwrap_or_else(|| "[]".to_string());
        wrapper.platform.receive_discover_result(&json);
        create_success()
    })
}

/// Swift calls this to deliver connect-to-device results.
///
/// # Parameters
/// * `session` - Session pointer
/// * `success` - 1=connected, 0=failed, -1=cancelled
/// * `error_message` - Error string on failure, NULL on success
#[no_mangle]
pub extern "C" fn obd_external_connect_result(
    session: *mut OBDSession,
    success: c_int,
    error_message: *const c_char,
) -> OBDError {
    with_external_error(session, false, "Invalid session", |wrapper| {
        let error_msg = c_string_to_string(error_message);
        wrapper.platform.receive_connect_result(success, error_msg);
        create_success()
    })
}

/// Initiate connection to an OBD controller.
/// Rust drives the flow: discover → select → connect → AT init → vehicle info.
/// The session's discovery engine owns the scan loop from here; the host is
/// asked for one bounded scan round at a time and the picker is the ONLY
/// selection path (the saved-preference feature never existed — removed
/// 2026-09-01).
#[no_mangle]
pub extern "C" fn obd_connect_to_controller(session: *mut OBDSession) -> OBDError {
    with_external_error(session, false, "Invalid session", |wrapper| {
        wrapper.session.api_handle().connect_to_controller();
        create_success()
    })
}

/// Tier S: hand the app's udsprofiles.json to the session (parsed
/// Rust-side; malformed entries degrade to untagged → poll). Call once at
/// connect — pure data plumbing.
#[no_mangle]
pub extern "C" fn obd_set_stream_profiles(
    session: *mut OBDSession,
    json: *const c_char,
) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if session.is_null() || json.is_null() {
            return create_error("Invalid parameters".to_string());
        }
        let Some(json) = c_string_to_string(json) else {
            return create_error("Invalid json".to_string());
        };
        external_or(session, ffi_error_result(), |wrapper| {
            wrapper.session.api_handle().set_stream_profiles(&json);
            create_success()
        })
    })
}

/// Per-adapter J1979 chunk cap from the app's adapter registry
/// (adapters.json `maxChunk`). Call at connect time, before identify; the
/// vehicle probe applies it. 1 disables chunking for this adapter; values
/// are clamped to 1..=6.
/// SP0: mark whether the connected adapter's STN chip supports STPPMA
/// periodic messaging (adapters.json `supportsPeriodic`). Call at connect.
#[no_mangle]
pub extern "C" fn obd_set_stn_periodic_capable(session: *mut OBDSession, capable: c_int) -> c_int {
    with_external_int(session, false, |w| {
        w.session
            .api_handle()
            .set_stn_periodic_capable(capable != 0);
        1
    })
}

/// LH6: register the host's raw BYTE writer — how a DVI frame reaches a
/// host-owned link (Swift BLE / classic / EA). Pattern of obd_set_raw_writer.
#[no_mangle]
pub extern "C" fn obd_set_bytes_writer(
    session: *mut OBDSession,
    writer: crate::external_platform::BytesWriteCallback,
) -> c_int {
    with_external_int(session, false, |w| {
        w.platform.set_bytes_writer(Some(writer));
        1
    })
}

/// LH5: hand the adapter catalog (adapters.json text) to the session. With
/// it, a connector's facts (supportsPeriodic / maxChunk / protocol) resolve
/// in Rust at connect for host AND Rust-owned connectors. Returns 1 on
/// success, 0 on a parse error.
#[no_mangle]
pub extern "C" fn obd_set_adapter_catalog(session: *mut OBDSession, json: *const c_char) -> c_int {
    let text = if json.is_null() {
        String::new()
    } else {
        c_string_to_string(json).unwrap_or_default()
    };
    with_external_int(session, false, |w| {
        match w.session.api_handle().set_adapter_catalog(&text) {
            Ok(()) => 1,
            Err(e) => {
                eprintln!("adapter catalog: {e}");
                0
            }
        }
    })
}

/// LH5 / DV2: the link dialect for the NEXT connect — `"elm"` (default) or
/// `"dvi"` — from adapters.json `protocol`. Call before
/// obd_external_connect_result, next to obd_set_stn_periodic_capable.
/// Returns 1 on success, 0 for an unknown protocol.
#[no_mangle]
pub extern "C" fn obd_set_link_protocol(
    session: *mut OBDSession,
    protocol: *const c_char,
) -> c_int {
    let proto = if protocol.is_null() {
        String::new()
    } else {
        c_string_to_string(protocol).unwrap_or_default()
    };
    with_external_int(session, false, |w| {
        if w.session.api_handle().set_link_protocol(&proto) {
            1
        } else {
            0
        }
    })
}

#[no_mangle]
pub extern "C" fn obd_set_adapter_max_chunk(session: *mut OBDSession, max_chunk: u32) -> OBDError {
    with_external_error(session, false, "Invalid parameters", |wrapper| {
        wrapper
            .session
            .api_handle()
            .set_adapter_max_chunk(max_chunk as usize);
        create_success()
    })
}

/// User selected a connector from the waiting_for_selection list.
///
/// # Parameters
/// * `session` - Session pointer
/// * `connector_id` - The selected connector's ID
#[no_mangle]
pub extern "C" fn obd_select_connector(
    session: *mut OBDSession,
    connector_id: *const c_char,
) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if session.is_null() || connector_id.is_null() {
            return create_error("Invalid parameters".to_string());
        }

        let Some(id) = c_string_to_string(connector_id) else {
            return create_error("Invalid connector ID".to_string());
        };

        external_or(session, ffi_error_result(), |wrapper| {
            wrapper.session.api_handle().select_connector(
                Arc::clone(&wrapper.platform) as Arc<dyn crate::platform::OBDPlatformInterface>,
                &id,
            );
            create_success()
        })
    })
}

/// User explicitly requested disconnect — clears the last-connected memory
/// (there is no auto-reconnect) and tears the session down cleanly.
#[no_mangle]
pub extern "C" fn obd_disconnect(session: *mut OBDSession) -> OBDError {
    with_external_error(session, false, "Invalid session", |wrapper| {
        // Invalidate any in-flight connect attempt (P1), then drain-and-
        // teardown on a background thread: stop sending, let the in-flight
        // command finish (bounded) so the adapter ends at its '>' prompt,
        // close the transport, emit the terminal DISCONNECTED. Returns
        // immediately; a DISCONNECTING status is emitted synchronously.
        let platform = std::sync::Arc::clone(&wrapper.platform)
            as std::sync::Arc<dyn crate::platform::OBDPlatformInterface>;
        wrapper.session.api_handle().disconnect_and_drain(platform);
        create_success()
    })
}

// MARK: - Sink (passive listen) FFI — plan M6

/// Enter sink state (plan M7: session-owned buffer, works with any platform
/// that implements enter_monitor_mode). `filter` may be NULL. For external
/// platforms the HOST puts the adapter into monitor mode and routes lines to
/// obd_sink_push; native platforms drive their own transport.
#[no_mangle]
/// `monitor_command` (LH0): the STN monitor command the host will write
/// (`STM` / `STMA` / a console override) — Rust re-issues it itself when the
/// chip drops out of monitor on `BUFFER FULL`. NULL → `STM` when a filter is
/// set, else `STMA`.
pub extern "C" fn obd_sink_start(
    session: *mut OBDSession,
    filter: *const c_char,
    monitor_command: *const c_char,
) -> OBDError {
    with_external_error(session, false, "Invalid session", |w| {
        let filter = if filter.is_null() {
            None
        } else {
            c_string_to_string(filter)
        };
        let monitor = if monitor_command.is_null() {
            None
        } else {
            c_string_to_string(monitor_command)
        };
        let platform = Arc::clone(&w.platform) as Arc<dyn crate::platform::OBDPlatformInterface>;
        result_to_obd_error(w.session.api_handle().sink_start(platform, filter, monitor))
    })
}

/// Push a captured transport line while sinking. LH0: the host no longer
/// splits lines (obd_external_receive_bytes does) — kept for one phase so
/// the bridge diff stays reviewable; removed in LH1.
#[no_mangle]
pub extern "C" fn obd_sink_push(session: *mut OBDSession, line: *const c_char) -> OBDError {
    with_external_error(session, line.is_null(), "Invalid arg", |w| {
        let line = c_string_to_string(line).unwrap_or_default();
        w.platform.sink_deliver(line);
        create_success()
    })
}

/// CMD4b: send one GM control exchange. `target` = dataset module NAME
/// ("EPB"/"ECM"/…); `service_hex` = the already-assembled service bytes
/// ("AE1A800000…" for A $AE, "2F11C90300" for B $2F). Rust resolves the
/// module's physical header from the attached dataset and sends. Returns JSON
/// {"ok":true,"raw":"7E8 03 EE 1A 00"} or {"ok":false,"error":"…"} — caller
/// frees with obd_free_string.
#[no_mangle]
pub extern "C" fn obd_send_control(
    session: *mut OBDSession,
    target: *const c_char,
    service_hex: *const c_char,
) -> *mut c_char {
    with_external_ptr(session, target.is_null() || service_hex.is_null(), |w| {
        let target = c_string_to_string(target).unwrap_or_default();
        let service = c_string_to_string(service_hex).unwrap_or_default();
        let obj = match w.session.api_handle().send_control(&target, &service) {
            Ok(raw) => serde_json::json!({ "ok": true, "raw": raw }),
            Err(e) => serde_json::json!({ "ok": false, "error": e }),
        };
        string_to_c_string(obj.to_string())
    })
}

/// Drain up to `max` queued frames as JSON:
/// {"frames":[{"t_ms":..,"raw":"..","id":"7E8"|null,"ts_us":..,"data":[..]}],"dropped":N}.
/// `raw` is the line verbatim (the sniffer/MCP contract); `id`/`ts_us`/`data`
/// are the LH3 parse (null id = a non-frame line such as STOPPED / BUFFER FULL).
/// Caller must free with obd_free_string. Returns NULL on error.
#[no_mangle]
pub extern "C" fn obd_sink_read(session: *mut OBDSession, max: u32) -> *mut c_char {
    with_external_ptr(session, false, |w| {
        let api = w.session.api_handle();
        let frames = api.sink_read(max as usize);
        let dropped = api.sink_stats().dropped;
        let arr: Vec<serde_json::Value> = frames.iter().map(|f| {
            serde_json::json!({ "t_ms": f.t_ms, "raw": f.raw, "id": f.id, "ts_us": f.ts_us, "data": f.data })
        }).collect();
        let obj = serde_json::json!({ "frames": arr, "dropped": dropped });
        string_to_c_string(obj.to_string())
    })
}

/// Sink counters as JSON:
/// {"active":bool,"started_at_ms":N,"received_total":N,"read_total":N,"queued":N,"dropped":N}.
/// Caller must free with obd_free_string.
#[no_mangle]
pub extern "C" fn obd_sink_stats(session: *mut OBDSession) -> *mut c_char {
    with_external_ptr(session, false, |w| {
        let s = w.session.api_handle().sink_stats();
        let obj = serde_json::json!({
            "active": s.active, "started_at_ms": s.started_at_ms,
            "received_total": s.received_total, "read_total": s.read_total,
            "queued": s.queued, "dropped": s.dropped
        });
        string_to_c_string(obj.to_string())
    })
}

/// Leave sink state (queue is retained for a final read).
#[no_mangle]
pub extern "C" fn obd_sink_stop(session: *mut OBDSession) -> OBDError {
    with_external_error(session, false, "Invalid session", |w| {
        let platform = Arc::clone(&w.platform) as Arc<dyn crate::platform::OBDPlatformInterface>;
        w.session.api_handle().sink_stop(platform);
        create_success()
    })
}

/// S2: start UDS periodic streaming using the profile under `tag`.
/// Runs the whole setup chain (session/defines gates → `2C 01` defines →
/// `2A` start), pauses polling, and arms the ingest — the HOST then owns the
/// transport-level monitor loop. Returns a JSON object the host needs
/// (caller frees with obd_free_string):
///   {"ok":true,"responseIds":["6A0","6A1"],"keepaliveIntervalMs":4000,
///    "slotsUsed":N}
///   {"ok":false,"error":"…"}  — polling untouched on failure.
/// SL4: legacy host-driven stream kit — mock-e2e only (`ffi-test`),
/// never shipped. The session drives streaming itself (S4 `obd_stream_engage`
/// / `obd_stream_request_stop`); every extra host-callable stop was another
/// Swift sandwich racing the ladder.
#[cfg(feature = "ffi-test")]
#[no_mangle]
pub extern "C" fn obd_start_stream(session: *mut OBDSession, tag: *const c_char) -> *mut c_char {
    with_external_ptr(session, tag.is_null(), |w| {
        let tag = c_string_to_string(tag).unwrap_or_default();
        let platform = Arc::clone(&w.platform) as Arc<dyn crate::platform::OBDPlatformInterface>;
        let result = match w.session.api_handle().start_stream(platform, &tag) {
            Ok(info) => serde_json::json!({
                "ok": true,
                "responseIds": info.response_ids,
                "keepaliveIntervalMs": info.keepalive_interval_ms,
                "slotsUsed": info.slots_used,
                "excludedPids": info.excluded_pids,
            }),
            Err(e) => serde_json::json!({ "ok": false, "error": e }),
        };
        CString::new(result.to_string())
            .map(|c| c.into_raw())
            .unwrap_or(ptr::null_mut())
    })
}

/// S2: register the host's raw transport writer. The Rust stream loop uses
/// it for all monitor-mode traffic (filters, STMA, break byte, keepalive) —
/// these must bypass the command correlator (a monitor command never
/// "completes" and would wedge the pipeline). The callback receives
/// (line, context) with the SAME context pointer given at session creation;
/// it must write the line to the adapter verbatim (plus terminator).
#[no_mangle]
pub extern "C" fn obd_set_raw_writer(
    session: *mut OBDSession,
    writer: crate::external_platform::RawWriteCallback,
) -> OBDError {
    with_external_error(session, false, "Invalid session", |w| {
        w.platform.set_raw_writer(writer);
        create_success()
    })
}

/// S2: start the stream's transport loop (filters → STMA → keepalive
/// alternation), all driven from Rust through the registered raw writer.
/// Call AFTER obd_start_stream succeeded AND the host's line tap is
/// attached (monitor lines flow back via obd_sink_push).
/// SL4: legacy host-driven stream kit — mock-e2e only (`ffi-test`),
/// never shipped. The session drives streaming itself (S4 `obd_stream_engage`
/// / `obd_stream_request_stop`); every extra host-callable stop was another
/// Swift sandwich racing the ladder.
#[cfg(feature = "ffi-test")]
#[no_mangle]
pub extern "C" fn obd_stream_arm(session: *mut OBDSession) -> OBDError {
    with_external_error(session, false, "Invalid session", |w| {
        let platform = Arc::clone(&w.platform) as Arc<dyn crate::platform::OBDPlatformInterface>;
        result_to_obd_error(w.session.api_handle().arm_stream_transport(platform))
    })
}

/// S2 stop phase 1: halt the loop, break monitor mode (STOPPED handshake
/// rides the still-attached tap), clear filters, exit monitor. Sequence for
/// the host: obd_stream_break → detach the line tap → obd_stop_stream (the
/// teardown replies must reach the command correlator, not the tap).
/// SL4: legacy host-driven stream kit — mock-e2e only (`ffi-test`),
/// never shipped. The session drives streaming itself (S4 `obd_stream_engage`
/// / `obd_stream_request_stop`); every extra host-callable stop was another
/// Swift sandwich racing the ladder.
#[cfg(feature = "ffi-test")]
#[no_mangle]
pub extern "C" fn obd_stream_break(session: *mut OBDSession) -> OBDError {
    with_external_error(session, false, "Invalid session", |w| {
        let platform = Arc::clone(&w.platform) as Arc<dyn crate::platform::OBDPlatformInterface>;
        w.session.api_handle().break_stream_transport(platform);
        create_success()
    })
}

/// S2 stop phase 2: `2A 04` · `2C 03` · `10 01` → resume polling with the
/// pipeline kick. Call AFTER obd_stream_break + tap detach. Safe when no
/// stream is active. Blocks until done — call off the main thread.
/// SL4: legacy host-driven stream kit — mock-e2e only (`ffi-test`),
/// never shipped. The session drives streaming itself (S4 `obd_stream_engage`
/// / `obd_stream_request_stop`); every extra host-callable stop was another
/// Swift sandwich racing the ladder.
#[cfg(feature = "ffi-test")]
#[no_mangle]
pub extern "C" fn obd_stop_stream(session: *mut OBDSession) -> OBDError {
    with_external_error(session, false, "Invalid session", |w| {
        let platform = Arc::clone(&w.platform) as Arc<dyn crate::platform::OBDPlatformInterface>;
        w.session.api_handle().stop_stream(platform);
        create_success()
    })
}

/// S3: queue a signal add (add=1) / remove (add=0) for a live stream —
/// executed in the next keepalive break, no restart. Returns 1 if queued,
/// 0 when no stream is active (the poll path handles the change normally).
#[no_mangle]
pub extern "C" fn obd_stream_change_signal(
    session: *mut OBDSession,
    pid: *const c_char,
    add: c_int,
) -> c_int {
    with_external_int(session, pid.is_null(), |w| {
        let pid = c_string_to_string(pid).unwrap_or_default();
        if w.session.api_handle().stream_change_signal(&pid, add != 0) {
            1
        } else {
            0
        }
    })
}

/// S4: hand the vinrules `udsPeriodic` capability tag to the session once
/// the VIN is known. NULL/empty clears it. From here the session decides
/// poll-vs-stream itself (start_subscription is the engage trigger).
#[no_mangle]
pub extern "C" fn obd_set_stream_tag(session: *mut OBDSession, tag: *const c_char) -> c_int {
    with_external_int(session, false, |w| {
        let tag = if tag.is_null() {
            None
        } else {
            c_string_to_string(tag).filter(|t| !t.is_empty())
        };
        w.session.api_handle().set_stream_tag(tag);
        1
    })
}

/// DR4: hand the session-config blob (the generated datasets.json, verbatim) to the
/// session once at setup. The session parses the session slice and resolves the
/// attached dataset itself at identify. NULL/empty clears it.
#[no_mangle]
pub extern "C" fn obd_set_dataset_registry(session: *mut OBDSession, json: *const c_char) -> c_int {
    with_external_int(session, false, |w| {
        let json = if json.is_null() {
            None
        } else {
            c_string_to_string(json).filter(|j| !j.is_empty())
        };
        w.session.api_handle().set_dataset_registry(json);
        1
    })
}

/// S4: host acks a monitor_control attach/detach request (the tap is wired).
#[no_mangle]
pub extern "C" fn obd_monitor_ready(session: *mut OBDSession) -> c_int {
    with_external_int(session, false, |w| {
        w.session.api_handle().monitor_ready();
        1
    })
}

/// S4 debug override: kick an engage attempt now (the normal trigger is
/// start_subscription). No-op when prerequisites don't hold.
#[no_mangle]
pub extern "C" fn obd_stream_engage(session: *mut OBDSession) -> c_int {
    with_external_int(session, false, |w| {
        w.session.api_handle().maybe_engage_stream();
        1
    })
}

/// S4: non-blocking managed stop (break → detach handshake → teardown →
/// resume); host follows along via stream_state events.
#[no_mangle]
pub extern "C" fn obd_stream_request_stop(session: *mut OBDSession) -> c_int {
    with_external_int(session, false, |w| {
        w.session.api_handle().request_stop_stream();
        1
    })
}

/// Use this instead of obd_destroy_session for sessions created with
/// obd_create_session_with_platform.
#[no_mangle]
pub extern "C" fn obd_destroy_external_session(session: *mut OBDSession) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if session.is_null() {
            return create_error("Session pointer is null".to_string());
        }

        unsafe {
            if *(session as *const u32) != EXTERNAL_SESSION_MAGIC {
                return create_error(
                    "session pointer type mismatch: expected an external session".to_string(),
                );
            }
            let _ = Box::from_raw(session as *mut ExternalSessionWrapper);
        }

        create_success()
    })
}

// ============================================================================
// Cache FFI
// ============================================================================

/// Probe PIDs using the cache — returns cached results for known PIDs,
/// flags uncached PIDs for the caller to probe over OBD.
///
/// This is a synchronous, non-blocking call — it only does disk I/O (no OBD).
/// Swift sends all PIDs it wants to discover, and this function returns
/// which are already cached (with their availability) and which need probing.
///
/// # Parameters
/// * `session` - Session pointer (used to read cache_path)
/// * `vin` - Vehicle Identification Number (cache key)
/// * `pids_json` - JSON array of PID strings, e.g. `["0100","0120","22DDBC"]`
///
/// # Returns
/// JSON string (caller must free with obd_free_string):
/// ```json
/// {
///   "cached_available": ["010C","010D"],
///   "cached_unavailable": ["22FFFF"],
///   "uncached": ["0140","22DDBC"]
/// }
/// ```
#[no_mangle]
pub extern "C" fn obd_probe_pids_cached(
    session: *mut OBDSession,
    vin: *const c_char,
    pids_json: *const c_char,
) -> *mut c_char {
    ffi_guard(ptr::null_mut(), || {
        if session.is_null() || vin.is_null() || pids_json.is_null() {
            return string_to_c_string(r#"{"error":"Invalid parameters"}"#.to_string());
        }

        let Some(vin_str) = c_string_to_string(vin) else {
            return string_to_c_string(r#"{"error":"Invalid VIN"}"#.to_string());
        };

        let Some(pids_str) = c_string_to_string(pids_json) else {
            return string_to_c_string(r#"{"error":"Invalid PIDs JSON"}"#.to_string());
        };

        let pids: Vec<String> = match serde_json::from_str(&pids_str) {
            Ok(p) => p,
            Err(e) => {
                return string_to_c_string(format!(r#"{{"error":"Failed to parse PIDs: {}"}}"#, e))
            }
        };

        // Get cache_path from session wrapper
        let cache_path = unsafe {
            match as_external_session(session) {
                Some(w) => w.cache_path.clone(),
                None => return string_to_c_string(r#"{"error":"Invalid session"}"#.to_string()),
            }
        };

        let cache_path = match cache_path {
            Some(p) => p,
            None => {
                // No cache configured — all PIDs are uncached
                let result = serde_json::json!({
                    "cached_available": [],
                    "cached_unavailable": [],
                    "uncached": pids
                });
                return string_to_c_string(result.to_string());
            }
        };

        let mgr = match crate::cache::CacheManager::new(&cache_path) {
            Ok(m) => m,
            Err(e) => return string_to_c_string(format!(r#"{{"error":"Cache error: {}"}}"#, e)),
        };

        match mgr.load(&vin_str) {
            Ok(Some(cache)) => {
                let mut cached_available = Vec::new();
                let mut cached_unavailable = Vec::new();
                let mut uncached = Vec::new();

                for pid in &pids {
                    if let Some(entry) = cache.entries.get(pid) {
                        if entry.available {
                            cached_available.push(pid.clone());
                        } else {
                            cached_unavailable.push(pid.clone());
                        }
                    } else {
                        uncached.push(pid.clone());
                    }
                }

                let result = serde_json::json!({
                    "cached_available": cached_available,
                    "cached_unavailable": cached_unavailable,
                    "uncached": uncached
                });
                string_to_c_string(result.to_string())
            }
            Ok(None) => {
                // No cache for this VIN — all PIDs are uncached
                let result = serde_json::json!({
                    "cached_available": [],
                    "cached_unavailable": [],
                    "uncached": pids
                });
                string_to_c_string(result.to_string())
            }
            Err(e) => string_to_c_string(format!(r#"{{"error":"Cache load error: {}"}}"#, e)),
        }
    })
}

/// Save probe results to cache after Swift has probed uncached PIDs over OBD.
///
/// # Parameters
/// * `session` - Session pointer (used to read cache_path)
/// * `vin` - Vehicle Identification Number (cache key)
/// * `results_json` - JSON object mapping PID to probe result:
///   ```json
///   {
///     "0100": {"raw_response": "41 00 BE 3E B8 13", "available": true},
///     "22DDBC": {"raw_response": "", "available": false}
///   }
///   ```
/// * `available_pids_json` - JSON array of derived available PIDs, e.g. `["010C","010D"]`
///
/// # Returns
/// OBDError with success/failure
#[no_mangle]
pub extern "C" fn obd_save_probe_results(
    session: *mut OBDSession,
    vin: *const c_char,
    results_json: *const c_char,
    available_pids_json: *const c_char,
) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        if session.is_null()
            || vin.is_null()
            || results_json.is_null()
            || available_pids_json.is_null()
        {
            return create_error("Invalid parameters".to_string());
        }

        let Some(vin_str) = c_string_to_string(vin) else {
            return create_error("Invalid VIN".to_string());
        };

        let Some(results_str) = c_string_to_string(results_json) else {
            return create_error("Invalid results JSON".to_string());
        };

        let Some(available_str) = c_string_to_string(available_pids_json) else {
            return create_error("Invalid available PIDs JSON".to_string());
        };

        // Parse results
        #[derive(Deserialize)]
        struct ProbeResult {
            raw_response: String,
            available: bool,
        }

        let results: std::collections::HashMap<String, ProbeResult> =
            match serde_json::from_str(&results_str) {
                Ok(r) => r,
                Err(e) => return create_error(format!("Failed to parse results: {}", e)),
            };

        let available_pids: Vec<String> = match serde_json::from_str(&available_str) {
            Ok(p) => p,
            Err(e) => return create_error(format!("Failed to parse available PIDs: {}", e)),
        };

        // Get cache_path from session wrapper
        let cache_path = unsafe {
            match as_external_session(session) {
                Some(w) => w.cache_path.clone(),
                None => return ffi_error_result(),
            }
        };

        let cache_path = match cache_path {
            Some(p) => p,
            None => return create_success(), // No cache configured — silently succeed
        };

        let mgr = match crate::cache::CacheManager::new(&cache_path) {
            Ok(m) => m,
            Err(e) => return create_error(format!("Cache error: {}", e)),
        };

        // Convert to CacheEntry map
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();

        let entries: std::collections::HashMap<String, crate::cache::CacheEntry> = results
            .into_iter()
            .map(|(pid, result)| {
                (
                    pid,
                    crate::cache::CacheEntry {
                        raw_response: result.raw_response,
                        available: result.available,
                        cached_at: now,
                    },
                )
            })
            .collect();

        match mgr.update_cache(&vin_str, entries, available_pids) {
            Ok(_) => create_success(),
            Err(e) => create_error(format!("Cache update error: {}", e)),
        }
    })
}

/// Clear the cache for a specific VIN
///
/// # Parameters
/// * `cache_path` - Base cache directory path
/// * `vin` - VIN to clear cache for (NULL to clear all caches)
#[no_mangle]
pub extern "C" fn obd_clear_cache(cache_path: *const c_char, vin: *const c_char) -> OBDError {
    ffi_guard(ffi_error_result(), || {
        let Some(path) = c_string_to_string(cache_path) else {
            return create_error("Invalid cache path".to_string());
        };

        let mgr = match crate::cache::CacheManager::new(&path) {
            Ok(m) => m,
            Err(e) => return create_error(format!("Cache error: {}", e)),
        };

        if vin.is_null() {
            match mgr.clear_all() {
                Ok(_) => create_success(),
                Err(e) => create_error(format!("Clear all error: {}", e)),
            }
        } else {
            let Some(vin_str) = c_string_to_string(vin) else {
                return create_error("Invalid VIN".to_string());
            };
            match mgr.clear(&vin_str) {
                Ok(_) => create_success(),
                Err(e) => create_error(format!("Clear error: {}", e)),
            }
        }
    })
}

/// Test-only entry point that panics inside the FFI guard, to prove the guard
/// contains the unwind at the C boundary and records the message.
#[cfg(test)]
extern "C" fn obd_test_panic() -> OBDError {
    ffi_guard(ffi_error_result(), || {
        panic!("deliberate test panic");
    })
}

#[cfg(test)]
mod ffi_safety_tests {
    use super::*;

    #[test]
    fn panic_is_contained_and_recorded() {
        // A panic on a C-boundary path must return the error value, not unwind.
        let result = obd_test_panic();
        assert_eq!(result.code, OBDResult::Error);

        // ...and the message must be retrievable via the last-error mechanism.
        let msg = unsafe {
            CStr::from_ptr(obd_get_last_error())
                .to_str()
                .unwrap()
                .to_string()
        };
        assert!(msg.contains("deliberate test panic"), "msg: {msg}");
    }
}

/// SP-LEN: seed a derived response length for a pid (equation byte usage).
/// Learned observations always override; seeding never overwrites.
#[no_mangle]
pub extern "C" fn obd_seed_pid_length(
    session: *mut OBDSession,
    pid: *const c_char,
    len: u32,
) -> OBDError {
    with_external_error(
        session,
        pid.is_null() || len == 0 || len > 64,
        "Invalid arg",
        |w| {
            let pid = c_string_to_string(pid).unwrap_or_default();
            w.session.api_handle().seed_pid_length(&pid, len as u8);
            create_success()
        },
    )
}

/// Badge toggle: pin (1) / unpin (0) plain polling. Pin stops any live
/// stream/periodic; unpin re-runs the engage ladder. Session-transient.
#[no_mangle]
pub extern "C" fn obd_set_user_pinned_poll(session: *mut OBDSession, pinned: c_int) -> OBDError {
    with_external_error(session, false, "Invalid arg", |w| {
        w.session.api_handle().set_user_pinned_poll(pinned != 0);
        create_success()
    })
}

/// STPPMA tier→period config from the host's `stnperiodic.json` (ms; clamped).
#[no_mangle]
pub extern "C" fn obd_set_stn_periods(
    session: *mut OBDSession,
    fast_ms: u32,
    medium_ms: u32,
    slow_ms: u32,
) -> OBDError {
    with_external_error(session, false, "Invalid arg", |w| {
        w.session
            .api_handle()
            .set_stn_periods(fast_ms, medium_ms, slow_ms);
        create_success()
    })
}

/// Settings: enable/disable the acquisition upgrade plugins (1 = enabled).
/// Both default enabled; disabling never touches plain polling.
#[no_mangle]
pub extern "C" fn obd_set_acquisition_plugins(
    session: *mut OBDSession,
    uds_enabled: c_int,
    stn_enabled: c_int,
) -> OBDError {
    with_external_error(session, false, "Invalid arg", |w| {
        w.session
            .api_handle()
            .set_acquisition_plugins(uds_enabled != 0, stn_enabled != 0);
        create_success()
    })
}

/// SR2b: Settings "Attempt calibration read on connect" (1 = enabled, the
/// default). Off = identify skips the Ford `$23` strategy probe and the GM
/// `F189` read entirely; the strategy stays unset (fail-closed downstream).
#[no_mangle]
pub extern "C" fn obd_set_calibration_read(session: *mut OBDSession, enabled: c_int) -> OBDError {
    with_external_error(session, false, "Invalid arg", |w| {
        w.session.api_handle().set_calibration_read(enabled != 0);
        create_success()
    })
}

/// SP: NON-BLOCKING periodic stop (background teardown; stream_state events
/// follow). The selector-open downgrade — returns immediately so the UI pops.
#[no_mangle]
pub extern "C" fn obd_stn_periodic_request_stop(session: *mut OBDSession) -> c_int {
    with_external_int(session, false, |w| {
        w.session.api_handle().request_stop_stn_periodic(false);
        1
    })
}
