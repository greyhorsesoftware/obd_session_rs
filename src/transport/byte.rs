//! `ByteTransport` (LH5) — every link is a byte pipe. The host-fed one
//! (Swift CoreBluetooth / IOBluetooth / ExternalAccessory) writes through a C
//! callback; the Rust-owned ones (TCP, serial, native BLE/RFCOMM) implement
//! this trait. Bytes coming IN always go to the platform's accumulator via a
//! [`ByteSink`] handed to the transport at construction — one framing path,
//! one delivery worker, whoever owns the wire.

use std::sync::Arc;

/// Where a Rust transport delivers received bytes (`ExternalPlatform::receive_bytes`).
pub type ByteSink = Arc<dyn Fn(&[u8]) + Send + Sync>;

/// Called once when the transport's reader ends (peer closed / read error).
pub type LinkDropSink = Arc<dyn Fn(String) + Send + Sync>;

pub trait ByteTransport: Send + Sync {
    /// Write bytes to the wire (blocking is fine — callers are the command
    /// processor thread and the monitor-mode raw writers).
    fn write(&self, bytes: &[u8]) -> Result<(), String>;
    /// Close the link; idempotent. Stops the reader thread.
    fn close(&self);
}
