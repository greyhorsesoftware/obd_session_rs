//! Transports (LH5): every link is a [`byte::ByteTransport`]. Always
//! compiled: TCP (WiFi adapters) and serial (USB CDC, macOS/Linux) plus the
//! connector [`router`]. Feature-gated natives for shells without a host
//! Bluetooth stack (Linux / Windows): btleplug BLE (`ble`) and RFCOMM / COM
//! Classic (`bluetooth-classic`). On Apple the host owns Bluetooth.

pub mod byte;
pub mod router;
#[cfg(any(target_os = "macos", target_os = "linux"))]
pub mod serial;
pub mod tcp;

#[cfg(feature = "ble")]
pub mod ble;
#[cfg(all(feature = "ble", target_os = "linux"))]
pub mod ble_agent;
#[cfg(all(feature = "bluetooth-classic", target_os = "linux"))]
pub mod classic_linux;
#[cfg(all(feature = "bluetooth-classic", target_os = "windows"))]
pub mod classic_windows;
