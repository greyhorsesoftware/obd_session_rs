//! Connector routing (LH5 — the plan's `MultiPlatform`, folded into
//! `ExternalPlatform`): connector ids the HOST minted (`mock:`, bare BLE
//! UUIDs, classic names, `ea:` serials) stay on the host's wire; ids Rust
//! minted carry a prefix — `usb:` (serial) / `wifi:` (TCP) — and open a
//! Rust-owned [`ByteTransport`]. Only NEW ids get prefixes, so every saved
//! preference and `knownConnectors` lookup keeps working unchanged.

use std::sync::Arc;
use std::time::Duration;

use super::byte::{ByteSink, ByteTransport, LinkDropSink};
use crate::platform::ConnectorInfo;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Route {
    /// The host (Swift) owns the wire.
    Host,
    /// A Rust transport owns the wire.
    Rust,
}

pub fn route(connector_id: &str) -> Route {
    let rust_prefix = connector_id.starts_with("usb:") || connector_id.starts_with("wifi:");
    #[cfg(feature = "ble")]
    let rust_prefix = rust_prefix || connector_id.starts_with("ble:");
    #[cfg(feature = "bluetooth-classic")]
    let rust_prefix =
        rust_prefix || connector_id.starts_with("rfcomm:") || connector_id.starts_with("com:");
    if rust_prefix {
        Route::Rust
    } else {
        Route::Host
    }
}

/// Rust-side discovery: USB serial ports + the WiFi adapter probe. Fast and
/// bounded (the WiFi probe is one connect attempt with a short timeout).
pub fn discover_rust_connectors() -> Vec<ConnectorInfo> {
    let mut out = Vec::new();
    #[cfg(any(target_os = "macos", target_os = "linux"))]
    out.extend(super::serial::SerialTransport::discover());
    out.extend(super::tcp::TcpTransport::discover(Duration::from_millis(
        800,
    )));
    #[cfg(feature = "ble")]
    out.extend(super::ble::discover(Duration::from_secs(5)));
    #[cfg(all(feature = "bluetooth-classic", target_os = "linux"))]
    out.extend(super::classic_linux::discover(Duration::from_secs(10)));
    #[cfg(all(feature = "bluetooth-classic", target_os = "windows"))]
    out.extend(super::classic_windows::discover(Duration::from_secs(1)));
    out
}

/// Open the Rust transport a prefixed connector id names.
pub fn open_rust_transport(
    connector_id: &str,
    on_bytes: ByteSink,
    on_drop: LinkDropSink,
) -> Result<Arc<dyn ByteTransport>, String> {
    if let Some(addr) = connector_id.strip_prefix("wifi:") {
        let t = super::tcp::TcpTransport::connect(addr, Duration::from_secs(5), on_bytes, on_drop)?;
        return Ok(Arc::new(t));
    }
    #[cfg(any(target_os = "macos", target_os = "linux"))]
    if let Some(path) = connector_id.strip_prefix("usb:") {
        let t = super::serial::SerialTransport::open(path, 115_200, on_bytes, on_drop)?;
        return Ok(Arc::new(t));
    }
    #[cfg(feature = "ble")]
    if connector_id.starts_with("ble:") {
        return Ok(Arc::new(super::ble::BleTransport::connect(
            connector_id,
            on_bytes,
            on_drop,
        )?));
    }
    #[cfg(all(feature = "bluetooth-classic", target_os = "linux"))]
    if connector_id.starts_with("rfcomm:") {
        return Ok(Arc::new(
            super::classic_linux::LinuxClassicTransport::connect(connector_id, on_bytes, on_drop)?,
        ));
    }
    #[cfg(all(feature = "bluetooth-classic", target_os = "windows"))]
    if connector_id.starts_with("com:") {
        return Ok(Arc::new(
            super::classic_windows::WindowsClassicTransport::connect(
                connector_id,
                on_bytes,
                on_drop,
            )?,
        ));
    }
    Err(format!("no Rust transport for connector {connector_id}"))
}
