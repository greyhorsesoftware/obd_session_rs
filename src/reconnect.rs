//! Last-connected connector/platform memory.
//!
//! There is NO auto-reconnect (removed — an unexpected drop lands the UI on
//! the connect screen). This module survives because the disconnect callback
//! needs the last-connected platform handle for its stale-drop check (a
//! late-scheduled DISCONNECTED must stand down if a new connect already took
//! the transport), and `clear_and_cancel` is the user-disconnect reset.

use std::sync::{Arc, Mutex};

use crate::platform::OBDPlatformInterface;

/// Reason string Rust uses for a user-initiated disconnect (set by
/// `ExternalPlatform::disconnect_from`). Distinguishes a deliberate
/// disconnect from an unexpected drop in status messages.
pub const USER_DISCONNECT_REASON: &str = "User disconnected";

/// Shared last-connected state.
pub struct ReconnectState {
    last_connector_id: Mutex<Option<String>>,
    platform: Mutex<Option<Arc<dyn OBDPlatformInterface>>>,
}

impl ReconnectState {
    pub fn new() -> Self {
        Self {
            last_connector_id: Mutex::new(None),
            platform: Mutex::new(None),
        }
    }

    /// Remember the connector + platform after a successful connect
    /// (feeds the disconnect callback's stale-drop check).
    pub fn remember(&self, connector_id: &str, platform: Arc<dyn OBDPlatformInterface>) {
        *self.last_connector_id.lock().unwrap() = Some(connector_id.to_string());
        *self.platform.lock().unwrap() = Some(platform);
    }

    pub fn platform(&self) -> Option<Arc<dyn OBDPlatformInterface>> {
        self.platform.lock().unwrap().clone()
    }

    /// Forget the remembered connector. Called on user-initiated disconnect
    /// and on drops.
    pub fn clear_and_cancel(&self) {
        *self.last_connector_id.lock().unwrap() = None;
    }
}

impl Default for ReconnectState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clear_forgets_connector() {
        let rs = ReconnectState::new();
        // Verify clear resets the remembered id (direct field access — the
        // public surface has no id getter; only the platform handle is read).
        *rs.last_connector_id.lock().unwrap() = Some("AA:BB".to_string());
        assert_eq!(
            rs.last_connector_id.lock().unwrap().clone(),
            Some("AA:BB".to_string())
        );
        rs.clear_and_cancel();
        assert_eq!(rs.last_connector_id.lock().unwrap().clone(), None);
    }
}
