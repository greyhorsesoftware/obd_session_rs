//! Error types for OBD session management

use std::fmt;

/// Adapter error definition — code + severity classification.
pub struct AdapterErrorDef {
    pub code: &'static str,
    pub severity: &'static str, // "transient" or "fatal"
}

/// Single source of truth for all known adapter/transport error codes.
/// Swift calls `obd_get_error_definitions()` at startup to get this list.
/// Both sides use these codes — no hardcoded error strings.
pub const ADAPTER_ERRORS: &[AdapterErrorDef] = &[
    // Transient — adapter still connected, temporary failure
    AdapterErrorDef {
        code: "TIMEOUT",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "NO_DATA",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "CAN_ERROR",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "DATA_ERROR",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "BUS_BUSY",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "BUS_ERROR",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "BUFFER_FULL",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "STOPPED",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "RX_ERROR",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "FC_RX_TIMEOUT",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "NO_PERIPHERAL",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "ENCODING_ERROR",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "WRITE_FAILED",
        severity: "transient",
    },
    AdapterErrorDef {
        code: "EMPTY_COMMAND",
        severity: "transient",
    },
    // Fatal — connection lost or unrecoverable
    AdapterErrorDef {
        code: "NOT_CONNECTED",
        severity: "fatal",
    },
    AdapterErrorDef {
        code: "UNABLE_TO_CONNECT",
        severity: "fatal",
    },
    AdapterErrorDef {
        code: "LV_RESET",
        severity: "fatal",
    },
    AdapterErrorDef {
        code: "ACT_ALERT",
        severity: "fatal",
    },
    AdapterErrorDef {
        code: "LP_ALERT",
        severity: "fatal",
    },
    AdapterErrorDef {
        code: "UART_RX_OVERFLOW",
        severity: "fatal",
    },
    AdapterErrorDef {
        code: "OUT_OF_MEMORY",
        severity: "fatal",
    },
    AdapterErrorDef {
        code: "DISCONNECTED",
        severity: "fatal",
    },
];

/// Look up severity for an error code. Unknown/novel codes default to "fatal": a code we don't
/// recognize is treated as needs-attention rather than transient, so it does not drive the
/// auto-retry path forever (A3).
pub fn error_severity(error: &str) -> &'static str {
    ADAPTER_ERRORS
        .iter()
        .find(|e| e.code == error)
        .map(|e| e.severity)
        .unwrap_or("fatal")
}

/// Map raw adapter response strings to standardized error codes.
/// Returns None if the response is not an error.
pub fn map_adapter_response_to_error(response: &str) -> Option<&'static str> {
    let trimmed = response.trim().to_uppercase();
    match trimmed.as_str() {
        "NO DATA" => Some("NO_DATA"),
        "CAN ERROR" => Some("CAN_ERROR"),
        "DATA ERROR" | "<DATA ERROR" => Some("DATA_ERROR"),
        "<RX ERROR" => Some("RX_ERROR"),
        "BUS BUSY" => Some("BUS_BUSY"),
        "BUS ERROR" => Some("BUS_ERROR"),
        "BUFFER FULL" => Some("BUFFER_FULL"),
        "STOPPED" => Some("STOPPED"),
        "FC RX TIMEOUT" => Some("FC_RX_TIMEOUT"),
        "UNABLE TO CONNECT" => Some("UNABLE_TO_CONNECT"),
        "LV RESET" => Some("LV_RESET"),
        "ACT ALERT" => Some("ACT_ALERT"),
        "LP ALERT" => Some("LP_ALERT"),
        "UART RX OVERFLOW" => Some("UART_RX_OVERFLOW"),
        "OUT OF MEMORY" => Some("OUT_OF_MEMORY"),
        _ => None,
    }
}

/// Errors that can occur during subscription management
#[derive(Debug, Clone, PartialEq)]
pub enum SubscriptionError {
    /// Subscription with given ID not found
    SubscriptionNotFound(uuid::Uuid),
    /// Invalid PID format or unsupported PID
    InvalidPID(String),
    /// Target controller not available
    ControllerNotAvailable(String),
    /// Subscription already exists
    AlreadyExists,
    /// Operation not allowed on inactive subscription
    NotActive,
    /// Platform-specific subscription error
    PlatformError(String),
}

impl fmt::Display for SubscriptionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SubscriptionError::SubscriptionNotFound(id) => {
                write!(f, "Subscription not found: {}", id)
            }
            SubscriptionError::InvalidPID(pid) => write!(f, "Invalid PID: {}", pid),
            SubscriptionError::ControllerNotAvailable(controller) => {
                write!(f, "Controller not available: {}", controller)
            }
            SubscriptionError::AlreadyExists => write!(f, "Subscription already exists"),
            SubscriptionError::NotActive => {
                write!(f, "Operation not allowed on inactive subscription")
            }
            SubscriptionError::PlatformError(msg) => {
                write!(f, "Subscription platform error: {}", msg)
            }
        }
    }
}

impl std::error::Error for SubscriptionError {}

/// Errors that can occur during session management
#[derive(Debug)]
pub enum SessionError {
    /// Platform initialization failed
    PlatformInitError(String),
    /// Thread spawning failed
    ThreadSpawnError(std::io::Error),
    /// Session is shutting down
    ShuttingDown,
    /// Operation timed out
    Timeout(String),
    /// Internal error
    InternalError(String),
}

impl fmt::Display for SessionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SessionError::PlatformInitError(msg) => {
                write!(f, "Platform initialization error: {}", msg)
            }
            SessionError::ThreadSpawnError(err) => write!(f, "Thread spawn error: {}", err),
            SessionError::ShuttingDown => write!(f, "Session is shutting down"),
            SessionError::Timeout(msg) => write!(f, "Operation timed out: {}", msg),
            SessionError::InternalError(msg) => write!(f, "Internal error: {}", msg),
        }
    }
}

impl std::error::Error for SessionError {}

impl From<std::io::Error> for SessionError {
    fn from(err: std::io::Error) -> Self {
        SessionError::ThreadSpawnError(err)
    }
}

impl From<SubscriptionError> for SessionError {
    fn from(err: SubscriptionError) -> Self {
        SessionError::InternalError(format!("Subscription error: {:?}", err))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_transient_code_is_transient() {
        assert_eq!(error_severity("TIMEOUT"), "transient");
        assert_eq!(error_severity("NO_DATA"), "transient");
    }

    #[test]
    fn known_fatal_code_is_fatal() {
        assert_eq!(error_severity("DISCONNECTED"), "fatal");
        assert_eq!(error_severity("NOT_CONNECTED"), "fatal");
    }

    #[test]
    fn unknown_code_is_not_transient() {
        // Novel/unrecognized codes must not drive the infinite auto-retry path (A3).
        let severity = error_severity("SOME_BRAND_NEW_ADAPTER_ERROR");
        assert_ne!(severity, "transient");
        assert_eq!(severity, "fatal");
    }
}
