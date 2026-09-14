//! Mock response generator for OBD commands
//!
//! This module provides response generation logic that can be used with
//! ExternalPlatform to create a fully mock OBD environment for testing.
//! This validates the same code path that production Swift would use.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Car scan data structure for mock responses
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CarScanData {
    /// Mode data (e.g., "01", "06", "09") and other fields
    #[serde(flatten)]
    pub data: serde_json::Value,
}

/// Mock response generator
///
/// Contains all the logic for generating OBD responses from JSON data files.
/// Can be used with ExternalPlatform to test the full external platform path.
pub struct MockResponder {
    /// Custom mock responses for specific commands
    mock_responses: Mutex<HashMap<String, String>>,
    /// Car scan data loaded from JSON
    car_scan_data: Mutex<Option<CarScanData>>,
    /// Current ECU controller for ATSH commands
    current_controller: Mutex<String>,
    /// Whether to return only the first ECU response
    only_return_ecu: Mutex<bool>,
    /// Artificial delay for responses
    pub response_delay: Mutex<Duration>,
    /// Command history for debugging
    command_history: Mutex<Vec<String>>,
    /// Commands that should fail at the TRANSPORT level (timeout/no prompt)
    /// instead of answering — models a clone adapter choking on a wire.
    error_commands: Mutex<std::collections::HashSet<String>>,
}

impl MockResponder {
    /// Create a new mock responder with default settings
    pub fn new() -> Self {
        let responder = Self {
            mock_responses: Mutex::new(HashMap::new()),
            car_scan_data: Mutex::new(None),
            current_controller: Mutex::new("7DF".to_string()),
            only_return_ecu: Mutex::new(false),
            response_delay: Mutex::new(Duration::from_millis(50)),
            command_history: Mutex::new(Vec::new()),
            error_commands: Mutex::new(std::collections::HashSet::new()),
        };

        // Load default car scan data
        let _ = responder.load_car_scan_data("mock_data/default.json");

        responder
    }

    /// Create a new mock responder with custom delay
    pub fn with_delay(delay: Duration) -> Self {
        let responder = Self::new();
        *responder.response_delay.lock().unwrap() = delay;
        responder
    }

    /// Load car scan data from JSON file
    pub fn load_car_scan_data(&self, file_path: &str) -> Result<(), String> {
        let file_content = std::fs::read_to_string(file_path)
            .map_err(|e| format!("Failed to read file {}: {}", file_path, e))?;

        let data: CarScanData = serde_json::from_str(&file_content)
            .map_err(|e| format!("Failed to parse JSON: {}", e))?;

        *self.car_scan_data.lock().unwrap() = Some(data);
        Ok(())
    }

    /// Set a custom mock response for a specific command
    pub fn set_mock_response(&self, command: &str, response: &str) {
        self.mock_responses
            .lock()
            .unwrap()
            .insert(command.to_string(), response.to_string());
    }

    /// Make `command` fail at the transport level (timeout / no `>` prompt) —
    /// models a clone adapter choking on a wire it can't handle.
    pub fn set_mock_error(&self, command: &str) {
        self.error_commands
            .lock()
            .unwrap()
            .insert(command.to_string());
    }

    /// Should this command error instead of answering? (Records history.)
    pub fn should_error(&self, command: &str) -> bool {
        let errors = self.error_commands.lock().unwrap().contains(command);
        if errors {
            self.record_command(command);
        }
        errors
    }

    /// Get command history
    pub fn get_command_history(&self) -> Vec<String> {
        self.command_history.lock().unwrap().clone()
    }

    /// Clear command history
    pub fn clear_command_history(&self) {
        self.command_history.lock().unwrap().clear();
    }

    /// Record a command in history
    pub fn record_command(&self, command: &str) {
        self.command_history
            .lock()
            .unwrap()
            .push(command.to_string());
    }

    /// Get response delay
    pub fn get_delay(&self) -> Duration {
        *self.response_delay.lock().unwrap()
    }

    /// Set response delay
    pub fn set_delay(&self, delay: Duration) {
        *self.response_delay.lock().unwrap() = delay;
    }

    /// Generate a mock response for a command
    pub fn generate_response(&self, command: &str) -> String {
        // Record command
        self.record_command(command);

        // Check custom mock responses first
        if let Some(response) = self.mock_responses.lock().unwrap().get(command) {
            return response.clone();
        }

        // Check car scan data
        if let Some(response) = self.get_response_from_car_scan(command) {
            return response;
        }

        // Default responses for common commands
        self.get_default_response(command)
    }

    /// Get response from car scan data
    fn get_response_from_car_scan(&self, command: &str) -> Option<String> {
        let car_data = self.car_scan_data.lock().unwrap();
        let car_data = car_data.as_ref()?;

        // Normalize command (remove spaces)
        let normalized = command.replace(" ", "");

        // Handle controller switching. 7DF = functional broadcast (all ECUs
        // answer); any physical ATSH 7Ex target filters to that ECU.
        if normalized == "ATSH7DF" {
            *self.only_return_ecu.lock().unwrap() = false;
            *self.current_controller.lock().unwrap() = "7DF".to_string();
            return Some("OK".to_string());
        }
        if normalized.starts_with("ATSH7E") {
            *self.only_return_ecu.lock().unwrap() = true;
            *self.current_controller.lock().unwrap() = normalized[4..].to_string();
            return Some("OK".to_string());
        }

        // Look up mode → pid → response array in the car-scan JSON.
        let lookup = |mode: &str, pid: &str| -> Option<&Vec<serde_json::Value>> {
            car_data
                .data
                .get(mode)
                .and_then(|m| m.as_object())
                .and_then(|o| o.get(pid))
                .and_then(|r| r.as_array())
        };

        // Handle targeted Mode 09 requests (090A, 0904, 0906) — filter JSON data by controller
        if normalized.starts_with("09") && normalized.len() == 4 {
            let controller = self.current_controller.lock().unwrap();
            if *controller != "7DF" {
                let pid = &normalized[2..];
                let (Some(responses), Ok(req)) =
                    (lookup("09", pid), u32::from_str_radix(&controller, 16))
                else {
                    return Some("NO DATA".to_string());
                };
                let response_addr = format!("{:03X}", req + 8);
                let filtered: Vec<String> = responses
                    .iter()
                    .filter_map(|v| v.as_str())
                    .filter(|s| s.starts_with(&response_addr))
                    .map(|s| s.to_string())
                    .collect();
                if filtered.is_empty() {
                    return Some("NO DATA".to_string());
                }
                return Some(filtered.join("\r"));
            }
        }

        // Extract mode and PID
        if normalized.len() >= 2 {
            let mode = &normalized[0..2].to_uppercase();
            let pid = if normalized.len() > 2 {
                &normalized[2..].to_uppercase()
            } else {
                ""
            };

            // Handle commands with only mode (no PID) like 03, 07, 0A
            if normalized.len() == 2 {
                let Some(data_array) = lookup(mode, "data") else {
                    return Some("NO DATA".to_string());
                };
                let response_strings: Vec<String> = data_array
                    .iter()
                    .filter_map(|v| v.as_str())
                    .map(|s| s.to_string())
                    .collect();
                return Some(response_strings.join("\r"));
            }

            // Handle regular PID requests
            if normalized.len() == 4 && pid.len() == 2 {
                let responses = lookup(mode, pid)?;
                let response_strings: Vec<String> = responses
                    .iter()
                    .filter_map(|v| v.as_str())
                    .map(|s| s.to_string())
                    .collect();

                if !*self.only_return_ecu.lock().unwrap() {
                    return Some(response_strings.join("\r"));
                }
                // Filter to lines from the targeted controller's response address
                let controller = self.current_controller.lock().unwrap();
                let Ok(req) = u32::from_str_radix(&controller, 16) else {
                    return response_strings.first().cloned();
                };
                let response_addr = format!("{:03X}", req + 8);
                let filtered: Vec<&String> = response_strings
                    .iter()
                    .filter(|s| s.starts_with(&response_addr))
                    .collect();
                if filtered.is_empty() {
                    return Some("NO DATA".to_string());
                }
                return Some(
                    filtered
                        .iter()
                        .map(|s| s.as_str())
                        .collect::<Vec<_>>()
                        .join("\r"),
                );
            }
        }

        None
    }

    /// Get default response for common commands
    fn get_default_response(&self, command: &str) -> String {
        let normalized = command.replace(" ", "");
        match normalized.as_str() {
            "ATZ" => "ELM327 v1.5".to_string(),
            "ATE0" => "OK".to_string(),
            "ATH1" => "OK".to_string(),
            "ATL0" => "OK".to_string(),
            "ATS0" => "OK".to_string(),
            "ATH0" => "OK".to_string(),
            "ATSP0" => "OK".to_string(),
            "ATV0" => "OK".to_string(),
            "STDI" => "OK".to_string(),
            // Batched-commands probe: the default mock is an ELM-class clone —
            // it must NOT appear pipe-capable, or every mock flow would start
            // emitting piped wires its canned entries can't match. Pipe tests
            // opt in with set_mock_response("STBC 1", "OK").
            "STBC1" => "?".to_string(),
            "STBCOF0" => "?".to_string(),
            "010C" => "41 0C 1A F8".to_string(),
            "010D" => "41 0D 00".to_string(),
            "0105" => "41 05 7B".to_string(),
            "ATRV" => "13.2V".to_string(),
            "0902" => "49 02 01 32 46 30 35 55 57 4A 46 00 00 00 00 00".to_string(),
            _ => "OK".to_string(),
        }
    }
}

impl Default for MockResponder {
    fn default() -> Self {
        Self::new()
    }
}

/// Context passed to mock send callback
/// Contains references to responder and platform for generating responses
pub struct MockExternalContext {
    pub responder: Arc<MockResponder>,
    pub platform: Arc<crate::external_platform::ExternalPlatform>,
}

/// Create a mock send callback for use with ExternalPlatform
///
/// This callback:
/// 1. Receives command from Rust
/// 2. Spawns a thread to simulate async response
/// 3. Generates mock response using MockResponder
/// 4. Calls platform.receive_response() to deliver response
///
/// This tests the exact same code path that production Swift would use.
pub extern "C" fn mock_send_callback(
    cmd_ptr: *const std::os::raw::c_char,
    _timeout_ms: u32,
    context: *mut std::ffi::c_void,
) {
    if cmd_ptr.is_null() || context.is_null() {
        return;
    }

    // Get context
    let ctx = unsafe { &*(context as *const MockExternalContext) };

    // Convert command
    let command = unsafe {
        std::ffi::CStr::from_ptr(cmd_ptr)
            .to_str()
            .unwrap_or("")
            .to_string()
    };

    // Clone what we need for the thread
    let responder = Arc::clone(&ctx.responder);
    let platform = Arc::clone(&ctx.platform);
    let delay = responder.get_delay();

    // Spawn thread to generate response (non-blocking, like real BLE)
    std::thread::spawn(move || {
        // Simulate communication delay
        std::thread::sleep(delay);

        // Transport-level failure injection (clone chokes, no prompt)
        if responder.should_error(&command) {
            platform.receive_error(command, "timeout: no > prompt".to_string());
            return;
        }

        // Generate response
        let response = responder.generate_response(&command);

        // Log response
        crate::platform::logging::log_data_received(&command, &response);

        // Deliver response through ExternalPlatform
        platform.receive_response(command, response);
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_responder_default_responses() {
        let responder = MockResponder::new();

        // AT commands have hardcoded responses
        assert_eq!(responder.generate_response("ATZ"), "ELM327 v1.5");
        assert_eq!(responder.generate_response("ATE0"), "OK");

        // PID responses come from JSON file if loaded
        let response = responder.generate_response("010C");
        assert!(!response.is_empty());
        // Response should contain mode 41 and PID 0C
        assert!(response.contains("41") && response.contains("0C"));
    }

    #[test]
    fn test_mock_responder_custom_response() {
        let responder = MockResponder::new();
        responder.set_mock_response("CUSTOM", "CUSTOM_RESPONSE");

        assert_eq!(responder.generate_response("CUSTOM"), "CUSTOM_RESPONSE");
    }

    #[test]
    fn test_mock_responder_command_history() {
        let responder = MockResponder::new();

        responder.generate_response("ATZ");
        responder.generate_response("010C");

        let history = responder.get_command_history();
        assert_eq!(history.len(), 2);
        assert_eq!(history[0], "ATZ");
        assert_eq!(history[1], "010C");
    }

    #[test]
    fn test_mock_responder_car_scan_data() {
        let responder = MockResponder::new();

        // Should load default.json and have real responses
        let response = responder.generate_response("0100");
        // Response should come from JSON file if loaded, otherwise default
        assert!(!response.is_empty());
    }
}
