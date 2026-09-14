//! Connection status message constants
//!
//! Typed constants for status message keys sent during the connection lifecycle.
//! Swift mirrors these as `ConnectionMsg` static constants for compile-time safety.

/// Connection status message constants — used as keys in status callbacks.
/// Swift mirrors these as ConnectionMsg static constants.
pub struct ConnectionMsg;

impl ConnectionMsg {
    // Connection phases
    pub const DISCOVERING_CONNECTORS: &'static str = "discovering_connectors";
    /// Discovery engine round activity (detail "start" | "done") — a UI
    /// blip for the rail's scanning indicator, NOT a phase transition.
    pub const SCAN_ROUND: &'static str = "scan_round";
    pub const WAITING_FOR_SELECTION: &'static str = "waiting_for_selection";
    /// SO2: emitted ONCE per attempt when discovery first enters selection.
    /// Later list changes ride `CONNECTOR_LIST` (a data carrier, no phase
    /// change) so the host refreshes the picker without re-running the
    /// selection logic — the reason the FB5 re-select guards existed.
    pub const CONNECTOR_LIST: &'static str = "connector_list";
    pub const CONNECTING: &'static str = "connecting";
    pub const INITIALIZING_PROTOCOL: &'static str = "initializing_protocol";
    pub const IDENTIFYING_VEHICLE: &'static str = "identifying_vehicle";
    pub const CONNECTED: &'static str = "connected";
    pub const RECONNECTING: &'static str = "reconnecting";
    pub const CONNECTION_FAILED: &'static str = "connection_failed";
    /// Drain in progress: new sends stopped, in-flight command finishing
    /// before the transport closes. Terminal DISCONNECTED follows.
    pub const DISCONNECTING: &'static str = "disconnecting";
    pub const DISCONNECTED: &'static str = "disconnected";

    // Failure reason CODES (RB3) — sent as the `detail` of a CONNECTION_FAILED
    // status. The Rust layer never emits human-readable failure prose; Swift
    // maps these codes to localized strings (ConnectionMsg.swift). Dynamic
    // data (device names, counts) rides in separate fields, never baked in.
    pub const REASON_NO_CONNECTORS: &'static str = "no_connectors";
    pub const REASON_ADAPTER_INIT_TIMEOUT: &'static str = "adapter_init_timeout";
    pub const REASON_BT_CONNECT_FAILED: &'static str = "bt_connect_failed";
    pub const REASON_RECONNECT_FAILED: &'static str = "reconnect_failed";
    pub const REASON_UNSUPPORTED_PROTOCOL: &'static str = "unsupported_protocol";
    /// Bus wasn't answering — no VIN / UNABLE TO CONNECT / no supported PIDs
    /// (RB1). Retryable (ignition off, adapter flaky, half-open link).
    pub const REASON_VEHICLE_NOT_RESPONDING: &'static str = "vehicle_not_responding";

    // Vehicle identity sub-phases (sent during IDENTIFYING_VEHICLE)
    pub const QUERYING_VIN: &'static str = "querying_vin";
    pub const CHECKING_CACHE: &'static str = "checking_cache";
    pub const CACHE_HIT: &'static str = "cache_hit";
    pub const QUERYING_ENGINE_TYPE: &'static str = "querying_engine_type";
    pub const DISCOVERING_ECUS: &'static str = "discovering_ecus";
    pub const QUERYING_SUPPORT_PIDS: &'static str = "querying_support_pids";
    pub const SAVING_CACHE: &'static str = "saving_cache";

    /// Derive the connection phase from a message constant.
    /// Vehicle identity sub-phases map to IDENTIFYING_VEHICLE;
    /// everything else is its own phase (returned as-is).
    pub fn phase_for<'a>(message: &'a str) -> &'a str {
        match message {
            Self::QUERYING_VIN
            | Self::CHECKING_CACHE
            | Self::CACHE_HIT
            | Self::QUERYING_ENGINE_TYPE
            | Self::DISCOVERING_ECUS
            | Self::QUERYING_SUPPORT_PIDS
            | Self::SAVING_CACHE => Self::IDENTIFYING_VEHICLE,
            other => other,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_phase_for_top_level() {
        assert_eq!(
            ConnectionMsg::phase_for(ConnectionMsg::DISCOVERING_CONNECTORS),
            ConnectionMsg::DISCOVERING_CONNECTORS
        );
        assert_eq!(
            ConnectionMsg::phase_for(ConnectionMsg::CONNECTING),
            ConnectionMsg::CONNECTING
        );
        assert_eq!(
            ConnectionMsg::phase_for(ConnectionMsg::CONNECTED),
            ConnectionMsg::CONNECTED
        );
        assert_eq!(
            ConnectionMsg::phase_for(ConnectionMsg::CONNECTION_FAILED),
            ConnectionMsg::CONNECTION_FAILED
        );
    }

    #[test]
    fn test_phase_for_vehicle_identity() {
        assert_eq!(
            ConnectionMsg::phase_for(ConnectionMsg::QUERYING_VIN),
            ConnectionMsg::IDENTIFYING_VEHICLE
        );
        assert_eq!(
            ConnectionMsg::phase_for(ConnectionMsg::CHECKING_CACHE),
            ConnectionMsg::IDENTIFYING_VEHICLE
        );
        assert_eq!(
            ConnectionMsg::phase_for(ConnectionMsg::CACHE_HIT),
            ConnectionMsg::IDENTIFYING_VEHICLE
        );
        assert_eq!(
            ConnectionMsg::phase_for(ConnectionMsg::DISCOVERING_ECUS),
            ConnectionMsg::IDENTIFYING_VEHICLE
        );
        assert_eq!(
            ConnectionMsg::phase_for(ConnectionMsg::QUERYING_SUPPORT_PIDS),
            ConnectionMsg::IDENTIFYING_VEHICLE
        );
        assert_eq!(
            ConnectionMsg::phase_for(ConnectionMsg::SAVING_CACHE),
            ConnectionMsg::IDENTIFYING_VEHICLE
        );
    }
}
