//! # obd_session_rs
//!
//! An OBD-II session engine: it owns the adapter link (ELM327 / STN / OBDX
//! DVI), identifies the vehicle on 11- and 29-bit CAN, and polls PIDs from
//! named subscriptions with tiered refresh, delivering every parsed reply to
//! the host as JSON. A C FFI (`ffi`) wraps the same engine for non-Rust hosts.
//!
//! ## Key features
//!
//! - **Self-managed background thread** — all API calls return immediately;
//!   replies arrive through the JSON callback.
//! - **Two platform routes** — the host feeds bytes from its own transport
//!   through [`external_platform::ExternalPlatform`], or the crate's native
//!   transports (`transport`: TCP, serial, and feature-gated BLE / Bluetooth
//!   Classic) drive the adapter directly.
//! - **PID deduplication and token-bucket rate limiting** across subscriptions.
//! - **Identify** — VIN, calibration ids, ECU roster and supported-PID map,
//!   cached per VIN (`cache`).
//! - **Session monitor, health and wire log** for diagnosing a live link.
//!
//! ## Architecture
//!
//! ```text
//! Host (Rust or C FFI) → OBDSessionManager → background worker
//!     ↑                                            ↓
//! JSON callback ← response parser ← link handler ← subscription poller
//! ```
//!
//! ## Usage with the mock platform
//!
//! ```rust,no_run
//! use obd_session_rs::{OBDSessionManager, OBDSessionConfig};
//! use obd_session_rs::external_platform::create_mock_external_platform;
//!
//! // The mock platform answers from mock_data/ — same code path as production.
//! let (platform, _context) = create_mock_external_platform();
//! let config = OBDSessionConfig::default();
//!
//! let session = OBDSessionManager::new(platform, config, |json| {
//!     println!("Received: {json}");
//! })?;
//!
//! let api = session.api_handle();
//! api.send_command("010C", 1000)?;
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```

pub mod acquisition;
pub mod addressing;
pub mod cache;
pub mod command_processor;
pub mod config;
pub mod connection_msg;
pub mod dataset_registry;
pub mod error;
pub mod external_platform;
pub mod ffi;
pub mod json_api;
pub mod length_cache;
pub mod mock_responder;
pub mod mode06_parser;
pub mod monitor_status_parser;
pub mod periodic_stream;
pub mod pid_registry;
pub mod pid_support_parser;
pub mod platform;
pub mod rate_limit;
pub mod reconnect;
pub mod response_parser;
pub mod session_health;
pub mod session_logger;
pub mod session_manager;
pub mod session_monitor;
pub mod sink;
pub mod stn_periodic;
pub mod subscription;
pub mod trouble_code_parser;
pub mod vehicle_info;
pub mod wmi_data;
// LH0: byte → ELM-text framing shared by EVERY transport (host-fed and native).
pub mod accumulator;
// LH5: adapters.json handed to Rust — connector → facts.
pub mod catalog;
// LH1: the protocol seam (LinkHandler + ElmHandler).
pub mod link;

// LH5: byte transports Rust owns (TCP / serial) + the connector router;
// native Bluetooth stays feature-gated inside.
pub mod transport;

// Re-exports for public API
pub use config::{CommandProcessorConfig, OBDSessionConfig};
pub use connection_msg::ConnectionMsg;
pub use error::{SessionError, SubscriptionError};
pub use json_api::{message_builder, Message, MessageProcessor};
pub use platform::{ConnectResult, ConnectorInfo, OBDPlatformConfig, OBDPlatformInterface};
pub use session_manager::OBDSessionManager;
pub use subscription::{SubscriptionInfo, SubscriptionManager};
pub use vehicle_info::VehicleInfo;

// FFI types (only for FFI usage, not direct Rust API)
pub use ffi::{OBDAPIHandle, OBDError, OBDResult, OBDSession};

// Version info
pub const VERSION: &str = env!("CARGO_PKG_VERSION");
