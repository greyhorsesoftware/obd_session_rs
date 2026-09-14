//! JSON API for Rust → Swift communication
//!
//! Defines the outbound message types and serialization for the JSON-based
//! interface. Swift enters Rust only via the C exports in `ffi.rs`; this
//! module builds the JSON messages Rust pushes back through the response
//! callback (OBD data, connection status, subscription completion).

use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// Top-level message envelope
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Message {
    /// Message type discriminator
    #[serde(rename = "type")]
    pub message_type: String,
    /// Unique message ID
    pub id: Uuid,
    /// Message payload
    pub payload: MessagePayload,
}

/// Message payload variants
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(untagged)]
pub enum MessagePayload {
    /// Subscription complete notification (all run-once PIDs done)
    SubscriptionComplete(SubscriptionCompletePayload),
    /// Raw OBD data response (command + response from device)
    OBDData(OBDDataPayload),
    /// Connection status change notification
    ConnectionStatus(ConnectionStatusPayload),
}

/// OBD data response payload - raw command/response from device
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OBDDataPayload {
    /// The OBD command that was sent
    pub command: String,
    /// The response data from the device
    pub data: String,
    /// Timestamp of the response
    pub timestamp: f64,
    /// Error message if command failed (None if successful)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// Error severity: "transient" (auto-retry) or "fatal" (requires reconnect).
    /// Only present when error is set.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error_severity: Option<String>,
    /// Pre-parsed response data (None for AT commands, errors, unrecognized)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parsed: Option<serde_json::Value>,
}

/// Connection status payload
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConnectionStatusPayload {
    /// Current connection status
    pub status: String,
    /// Whether device is connected (convenience field)
    pub connected: bool,
    /// Optional reason for status change (e.g., "device_disconnected", "bluetooth_off")
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
    /// Timestamp of the status change
    pub timestamp: f64,
}

/// Subscription complete payload - sent when all run-once PIDs are done
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SubscriptionCompletePayload {
    /// Subscription ID
    pub subscription_id: Uuid,
    /// Final PID updates (all collected data)
    pub updates: Vec<PIDUpdate>,
}

/// Individual PID update
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PIDUpdate {
    /// PID identifier
    pub pid: String,
    /// Response data
    pub data: String,
    /// Timestamp of the update
    pub timestamp: f64,
    /// Error message (null if successful)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// Pre-parsed response data (None for AT commands, errors, unrecognized)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub parsed: Option<serde_json::Value>,
}

/// Message type constants
pub mod message_types {
    pub const SUBSCRIPTION_COMPLETE: &str = "subscription_complete";
    pub const OBD_DATA: &str = "obd_data";
    pub const CONNECTION_STATUS: &str = "connection_status";
}

/// Helper functions for creating messages
pub mod message_builder {
    use super::*;

    /// Create a subscription complete message
    /// Sent when all run-once PIDs in a subscription have been polled and no continuous PIDs remain
    pub fn subscription_complete_message(
        subscription_id: Uuid,
        updates: Vec<PIDUpdate>,
    ) -> Message {
        Message {
            message_type: message_types::SUBSCRIPTION_COMPLETE.to_string(),
            id: Uuid::new_v4(),
            payload: MessagePayload::SubscriptionComplete(super::SubscriptionCompletePayload {
                subscription_id,
                updates,
            }),
        }
    }

    /// Create an OBD data message for raw command/response data
    pub fn obd_data_message(
        command: String,
        data: String,
        error: Option<String>,
        parsed: Option<serde_json::Value>,
    ) -> Message {
        use std::time::{SystemTime, UNIX_EPOCH};
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        let error_severity = error
            .as_ref()
            .map(|e| crate::error::error_severity(e).to_string());
        Message {
            message_type: message_types::OBD_DATA.to_string(),
            id: Uuid::new_v4(),
            payload: MessagePayload::OBDData(super::OBDDataPayload {
                command,
                data,
                timestamp,
                error,
                error_severity,
                parsed,
            }),
        }
    }

    /// Create a connection status message
    pub fn connection_status_message(
        status: crate::platform::ConnectionStatus,
        reason: Option<String>,
    ) -> Message {
        use std::time::{SystemTime, UNIX_EPOCH};
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        let connected = status == crate::platform::ConnectionStatus::Connected;
        Message {
            message_type: message_types::CONNECTION_STATUS.to_string(),
            id: Uuid::new_v4(),
            payload: MessagePayload::ConnectionStatus(super::ConnectionStatusPayload {
                status: status.to_string(),
                connected,
                reason,
                timestamp,
            }),
        }
    }
}

/// Message processing utilities
pub struct MessageProcessor;

impl MessageProcessor {
    /// Serialize a Message to JSON string
    pub fn serialize_message(message: &Message) -> Result<String, serde_json::Error> {
        serde_json::to_string(message)
    }
}
