//! Platform interface abstraction for OBD communication
//!
//! This module defines the trait that all OBD platforms must implement,
//! providing a clean abstraction over different communication methods
//! (Bluetooth, USB, WiFi, etc.).

use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// Connection status for OBD device
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ConnectionStatus {
    /// Device is connected and ready
    Connected,
    /// Device is disconnected
    Disconnected,
    /// Connection attempt in progress
    Connecting,
    /// Connection failed
    ConnectionFailed,
    /// Legacy status — auto-reconnect was removed; only constructed via the
    /// FFI status mapping (`4 =>`). Kept until the Swift side is verified
    /// not to send/expect it.
    Reconnecting,
}

impl std::fmt::Display for ConnectionStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConnectionStatus::Connected => write!(f, "connected"),
            ConnectionStatus::Disconnected => write!(f, "disconnected"),
            ConnectionStatus::Connecting => write!(f, "connecting"),
            ConnectionStatus::ConnectionFailed => write!(f, "connection_failed"),
            ConnectionStatus::Reconnecting => write!(f, "reconnecting"),
        }
    }
}

/// Descriptor for an available OBD-II connector
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConnectorInfo {
    /// Unique identifier (MAC address, UUID, "mock:gladiator")
    pub id: String,
    /// Display name ("OBDLink MX+")
    pub name: String,
    /// Connector type ("ble", "classic", "mock")
    pub connector_type: String,
}

/// Result of a connect attempt
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ConnectResult {
    /// Successfully connected
    Connected,
    /// Connection failed
    Failed { reason: String },
    /// Connection cancelled by user
    Cancelled,
}

/// Callback type for OBD data responses
/// LH7: (command, typed outcome) — the dialect's `WireReply` or a failure code.
pub type OBDDataCallback = Box<dyn Fn(String, crate::link::LinkOutcome) + Send + Sync>;

/// LH7: text → typed reply for a text dialect (the ElmHandler installs it;
/// the platform decodes every text completion through it before the data
/// callback). Arguments: (command, reply text).
pub type ReplyDecoder = Arc<dyn Fn(&str, &str) -> crate::link::WireReply + Send + Sync>;

/// Callback type for connection status changes
/// Parameters: (new_status, optional_reason)
pub type ConnectionCallback = Box<dyn Fn(ConnectionStatus, Option<String>) + Send + Sync>;

/// Trait that all OBD platform implementations must satisfy
pub trait OBDPlatformInterface: Send + Sync {
    /// WIRE-LOG: hand the platform the session logger so wire tees
    /// (TX/RX/DROP lines) reach `adapter_<ts>.log`. Default no-op — mock
    /// and test platforms may ignore it.
    fn set_wire_log(&self, _logger: std::sync::Arc<crate::session_logger::SessionLogger>) {}

    /// Send a command to the OBD device
    ///
    /// **CRITICAL**: This method MUST return immediately after queuing the command
    /// for asynchronous processing. The configured callback will be invoked when
    /// the response is received or timeout occurs.
    ///
    /// # Arguments
    /// * `command` - The OBD command to send (e.g., "010C")
    /// * `timeout_ms` - Maximum time to wait for response in milliseconds
    fn send_command(&self, command: &str, timeout_ms: u32);

    /// Set completion callback (called when command response is received)
    /// This enables serial command processing - only one outstanding command at a time
    fn set_completion_callback(&self, callback: Option<Box<dyn Fn(String) + Send + Sync>>);

    /// Set data callback for receiving OBD response data
    /// This is called with (command, LinkOutcome) when responses are received
    /// Session manager uses this to forward data through its callback
    fn set_data_callback(&self, callback: Option<OBDDataCallback>);

    /// Set connection status callback
    /// Called when connection status changes (connected, disconnected, etc.)
    fn set_connection_callback(&self, callback: Option<ConnectionCallback>);

    /// Check if the platform is currently connected to an OBD device
    fn is_connected(&self) -> bool;

    /// Get current connection status
    fn connection_status(&self) -> ConnectionStatus;

    /// Record a transport-level connection state change (S4: the session's
    /// own connect flow marks Connected here so `connection_status()` is
    /// truthful for engage guards). Default no-op for platforms that track
    /// state themselves.
    fn note_connection_status(&self, _status: ConnectionStatus) {}

    /// Run ONE bounded scan round (host defines the round length — e.g. the
    /// 2 s BLE window) and call back exactly once with a SNAPSHOT of what the
    /// host sees NOW. No accumulation, no pruning, no memory between rounds —
    /// cadence and list policy are the session's (session_manager/discovery).
    /// The host must self-terminate the round; there is no cancel.
    /// Default: answers immediately with no connectors.
    fn scan_connectors_round(&self, callback: Box<dyn FnOnce(Vec<ConnectorInfo>) + Send>) {
        callback(Vec::new());
    }

    /// Ask platform to connect to a specific connector by ID.
    /// Platform calls the provided callback with the result.
    /// Default: returns Cancelled (not supported).
    fn connect_to(&self, _connector_id: &str, _callback: Box<dyn FnOnce(ConnectResult) + Send>) {
        // Default: not supported
    }

    /// The session gave up on a `connect_to` it had asked for (user cancel
    /// while the transport was still opening). A platform that opens links
    /// asynchronously must drop the late result — and close a link that had
    /// just come up — WITHOUT publishing a status change: the cancel already
    /// told the UI, and a newer attempt may own the platform by then.
    /// Default: no-op (a host-owned transport runs its own connect deadline).
    fn abandon_pending_connect(&self) {}

    /// Ask platform to disconnect from the current device.
    /// Default: no-op.
    fn disconnect_from(&self) {
        // Default: no-op
    }

    /// Enter passive monitor mode (plan M7): stop request/response operation,
    /// put the adapter in monitor-all (honoring `filter` where supported), and
    /// deliver every received line to `on_line` until `exit_monitor_mode`.
    /// The session owns the queue — platforms only capture and forward.
    /// Default: unsupported, so `sink_start` fails cleanly on platforms that
    /// can't monitor.
    fn enter_monitor_mode(
        &self,
        _filter: Option<&str>,
        _on_line: Box<dyn Fn(String) + Send + Sync>,
    ) -> Result<(), String> {
        Err("passive monitoring not supported by this platform".to_string())
    }

    /// Leave passive monitor mode and restore request/response operation.
    /// Default: no-op.
    fn exit_monitor_mode(&self) {}

    /// S2: write a line RAW to the adapter, bypassing the command processor
    /// and its response correlator entirely. Monitor-mode traffic (`STMA`,
    /// filter setup, the keepalive `3E 80`, the break byte) MUST use this —
    /// a monitor command never "completes", so sending it as a command
    /// wedges the serial pipeline (hardware-verified). Default: no-op
    /// (platforms without raw transport access can't stream).
    fn write_raw(&self, _line: &str) {}

    // ---- LH6: binary-dialect hooks (OBDX DVI). Defaults = "not a byte link".

    /// Switch the link's framing between ELM text and DVI binary frames.
    fn set_byte_mode(&self, _on: bool) {}

    /// Write raw bytes to the wire (a DVI frame). Default: unsupported.
    fn write_bytes(&self, _bytes: &[u8]) -> Result<(), String> {
        Err("byte writes not supported by this platform".to_string())
    }

    /// Install the consumer of decoded DVI frames (delivered off the I/O
    /// thread, never under the platform's state lock).
    fn set_frame_sink(
        &self,
        _sink: Option<Arc<dyn Fn(crate::link::dvi::codec::DviFrame) + Send + Sync>>,
    ) {
    }

    /// Install the encoder that turns an engine command (`"010C"`, `"ATSH7E0"`)
    /// into wire bytes; `Ok(None)` = handled locally (the handler completes
    /// it itself), `Ok(Some(bytes))` = write these, `Err` = fail the command.
    fn set_command_encoder(
        &self,
        _enc: Option<Arc<dyn Fn(&str) -> Result<Option<Vec<u8>>, String> + Send + Sync>>,
    ) {
    }

    /// Complete the outstanding command with a typed outcome — how a binary
    /// dialect (DVI) answers the engine without ever rendering text.
    fn complete_command(&self, _command: &str, _outcome: crate::link::LinkOutcome) {}

    /// LH7: install the text → typed decoder a text dialect owns (None =
    /// the platform's built-in ELM 11-bit decode, for handler-less rigs).
    fn set_reply_decoder(&self, _dec: Option<ReplyDecoder>) {}

    /// DVI RX frames carry a µs timestamp once enabled on the tool.
    fn set_frame_timestamps(&self, _on: bool) {}
}

/// Background thread logging utilities.
///
/// Console (stdout) mirroring of command traffic — **off by default**: the session audit log
/// (`obd_logs/*.jsonl`, see `session_logger`) already records every command/response, so this
/// is pure console noise in normal runs. Set the `OBD_CONSOLE_LOG` environment variable to
/// re-enable for bench debugging.
pub mod logging {
    use std::sync::{Mutex, OnceLock};

    // Global lock to prevent interleaved output from multiple threads
    static LOG_MUTEX: Mutex<()> = Mutex::new(());

    fn console_enabled() -> bool {
        static ENABLED: OnceLock<bool> = OnceLock::new();
        *ENABLED.get_or_init(|| std::env::var_os("OBD_CONSOLE_LOG").is_some())
    }

    /// Log that a command was sent (for debugging background thread activity)
    pub fn log_command_sent(command: &str) {
        if !console_enabled() {
            return;
        }
        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_millis();

        // Lock to prevent interleaved output
        let _lock = LOG_MUTEX.lock().unwrap();
        println!("[{}] COMMAND_SENT: {}", timestamp, command);
    }

    /// Log that data was received (for debugging background thread activity)
    pub fn log_data_received(command: &str, response: &str) {
        if !console_enabled() {
            return;
        }
        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_millis();

        // Replace \r with space for compact single-line logging
        let clean_response = response.replace('\r', " ");

        // Lock to prevent interleaved output
        let _lock = LOG_MUTEX.lock().unwrap();
        println!(
            "[{}] DATA_RECEIVED: {} -> {}",
            timestamp, command, clean_response
        );
    }
}

/// Configuration for OBD platform with response callback
pub struct OBDPlatformConfig {
    /// Callback function invoked when command responses are received
    /// Parameters: (command, result)
    pub command_response_callback: Box<dyn Fn(String, Result<String, String>) + Send + 'static>,
}

// Re-export types from mock_responder module (used for testing)
pub use crate::mock_responder::CarScanData;
