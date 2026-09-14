//! Native BLE transport via btleplug (LH5: a `ByteTransport`). For shells
//! without a host Bluetooth stack (Linux / Windows — a Tauri build); on Apple
//! the host owns CoreBluetooth and this module is not compiled in.
//!
//! Connector ids are minted as `ble:<peripheral id>`; the reader is the
//! notification stream, delivered byte-for-byte to the platform's
//! accumulator through the sink handed in at connect.

#![allow(dead_code)]

use std::sync::Mutex;
use std::time::Duration;

use btleplug::api::{
    Central, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::StreamExt;
use tokio::runtime::Runtime;

use super::byte::{ByteSink, ByteTransport, LinkDropSink};
use crate::platform::ConnectorInfo;

/// Known OBD BLE service UUIDs
const OBD_SERVICE_UUIDS: &[&str] = &[
    "fff0",                                 // Generic ELM327 BLE
    "e7810a71-73ae-499d-8c15-faa9aef0c3f2", // OBDLink CX / STN
    "6e400001-b5a3-f393-e0a9-e50e24dcca9e", // Nordic UART (OBDX Pro)
];
/// Write characteristic candidates
const TX_CHAR_UUIDS: &[&str] = &[
    "fff2",
    "bef8d6c9-9c21-4c9e-b632-bd58c1009f9f",
    "6e400002-b5a3-f393-e0a9-e50e24dcca9e",
];
/// Notify characteristic candidates
const RX_CHAR_UUIDS: &[&str] = &[
    "fff1",
    "bef8d6c9-9c21-4c9e-b632-bd58c1009f9f",
    "6e400003-b5a3-f393-e0a9-e50e24dcca9e",
];

/// One shared runtime + adapter for scans and links.
struct BleCentral {
    runtime: Runtime,
    adapter: Adapter,
}

/// The process-wide BLE central. Only a SUCCESS is cached: a failure (no
/// adapter yet, bluetoothd not up, D-Bus refused) is retried on the next
/// discovery round instead of poisoning BLE for the life of the process.
fn central() -> Result<&'static BleCentral, String> {
    static CENTRAL: std::sync::OnceLock<BleCentral> = std::sync::OnceLock::new();
    if let Some(c) = CENTRAL.get() {
        return Ok(c);
    }
    let runtime = Runtime::new().map_err(|e| format!("tokio runtime: {e}"))?;
    let adapter = runtime.block_on(async {
        let manager = Manager::new()
            .await
            .map_err(|e| format!("BLE manager: {e}"))?;
        manager
            .adapters()
            .await
            .map_err(|e| format!("BLE adapters: {e}"))?
            .into_iter()
            .next()
            .ok_or_else(|| "No BLE adapter found".to_string())
    })?;
    Ok(CENTRAL.get_or_init(|| BleCentral { runtime, adapter }))
}

/// Scan for OBD BLE adapters (bounded) → `ble:` connectors.
pub fn discover(duration: Duration) -> Vec<ConnectorInfo> {
    // Log like the WiFi probe does ([WIFI] …) — a silent empty list hid
    // "no adapter / bluetoothd down" from users (2026-09-12).
    let c = match central() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("[BLE] unavailable: {e}");
            return vec![];
        }
    };
    let result = c.runtime.block_on(async {
        c.adapter
            .start_scan(ScanFilter::default())
            .await
            .map_err(|e| format!("start_scan: {e}"))?;
        tokio::time::sleep(duration).await;
        let _ = c.adapter.stop_scan().await;
        let mut out = Vec::new();
        let mut seen = 0usize;
        for p in c
            .adapter
            .peripherals()
            .await
            .map_err(|e| format!("peripherals: {e}"))?
        {
            seen += 1;
            let Ok(Some(props)) = p.properties().await else {
                continue;
            };
            let name = props.local_name.unwrap_or_else(|| "Unknown".to_string());
            let has_obd_service = props.services.iter().any(|u| {
                let s = u.to_string().to_lowercase();
                OBD_SERVICE_UUIDS.iter().any(|obd| s.contains(obd))
            });
            if has_obd_service || name.to_lowercase().contains("obd") {
                out.push(ConnectorInfo {
                    id: format!("ble:{}", p.id()),
                    name,
                    connector_type: "ble".to_string(),
                });
            }
        }
        eprintln!("[BLE] scan: {seen} peripheral(s) seen, {} OBD", out.len());
        Ok::<_, String>(out)
    });
    match result {
        Ok(out) => out,
        Err(e) => {
            eprintln!("[BLE] scan failed: {e}");
            vec![]
        }
    }
}

pub struct BleTransport {
    peripheral: Mutex<Option<Peripheral>>,
    tx_char: Characteristic,
}

impl BleTransport {
    /// Connect to a scanned peripheral by its `ble:<id>` connector id.
    pub fn connect(
        connector_id: &str,
        on_bytes: ByteSink,
        on_drop: LinkDropSink,
    ) -> Result<Self, String> {
        let want = connector_id.strip_prefix("ble:").unwrap_or(connector_id);
        let c = central()?;
        c.runtime.block_on(async {
            let peripheral = c
                .adapter
                .peripherals()
                .await
                .map_err(|e| e.to_string())?
                .into_iter()
                .find(|p| p.id().to_string() == want)
                .ok_or_else(|| format!("Device {want} not found in scan results"))?;
            peripheral
                .connect()
                .await
                .map_err(|e| format!("Connect failed: {e}"))?;
            peripheral
                .discover_services()
                .await
                .map_err(|e| format!("Service discovery failed: {e}"))?;
            let chars = peripheral.characteristics();
            let find = |set: &[&str]| {
                chars
                    .iter()
                    .find(|ch| {
                        let s = ch.uuid.to_string().to_lowercase();
                        set.iter().any(|u| s.contains(u))
                    })
                    .cloned()
            };
            let tx_char = find(TX_CHAR_UUIDS).ok_or("TX characteristic not found")?;
            let rx_char = find(RX_CHAR_UUIDS).ok_or("RX characteristic not found")?;
            peripheral
                .subscribe(&rx_char)
                .await
                .map_err(|e| format!("Subscribe failed: {e}"))?;
            let notifier = peripheral.clone();
            c.runtime.spawn(async move {
                let Ok(mut notifications) = notifier.notifications().await else {
                    on_drop("notifications unavailable".to_string());
                    return;
                };
                while let Some(n) = notifications.next().await {
                    on_bytes(&n.value);
                }
                on_drop("peer closed".to_string());
            });
            Ok(Self {
                peripheral: Mutex::new(Some(peripheral)),
                tx_char,
            })
        })
    }
}

impl ByteTransport for BleTransport {
    fn write(&self, bytes: &[u8]) -> Result<(), String> {
        let p = self
            .peripheral
            .lock()
            .unwrap()
            .clone()
            .ok_or("not connected")?;
        let c = central()?;
        c.runtime.block_on(async {
            // MTU-sized chunks: BLE writes past the link MTU are truncated silently.
            for chunk in bytes.chunks(20) {
                p.write(&self.tx_char, chunk, WriteType::WithoutResponse)
                    .await
                    .map_err(|e| format!("Write failed: {e}"))?;
            }
            Ok(())
        })
    }

    fn close(&self) {
        if let Some(p) = self.peripheral.lock().unwrap().take() {
            if let Ok(c) = central() {
                c.runtime.block_on(async {
                    let _ = p.disconnect().await;
                });
            }
        }
    }
}
