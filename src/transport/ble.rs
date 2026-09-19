//! Native BLE transport via btleplug (LH5: a `ByteTransport`). For shells
//! without a host Bluetooth stack (Linux / Windows — a Tauri build); on Apple
//! the host owns CoreBluetooth and this module is not compiled in.
//!
//! Connector ids are minted as `ble:<peripheral id>`; the reader is the
//! notification stream, delivered byte-for-byte to the platform's
//! accumulator through the sink handed in at connect.
//!
//! Every stage of connect is BOUNDED and names itself on failure
//! (`ble_connect_timeout`, `ble_subscribe_failed: …`): a peripheral that
//! never answers must fail the attempt, not wedge the session worker
//! (OBDLink CX bench 2026-09-19 — the CCCD write sat unanswered forever).

#![allow(dead_code)]

use std::fmt::Display;
use std::future::Future;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::Duration;

use btleplug::api::{
    Central, CharPropFlags, Characteristic, Manager as _, Peripheral as _, ScanFilter, WriteType,
};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::StreamExt;
use tokio::runtime::Runtime;
use uuid::Uuid;

use super::byte::{ByteSink, ByteTransport, LinkDropSink};
use crate::platform::ConnectorInfo;

/// Stage budgets. Their sum (plus the 5 s cleanup disconnect) stays under the
/// session's 60 s connect deadline, so the stage error is what the user sees.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(15);
const DISCOVER_TIMEOUT: Duration = Duration::from_secs(10);
/// Includes Just Works pairing when the adapter demands a bonded link.
const SUBSCRIBE_TIMEOUT: Duration = Duration::from_secs(15);
const DISCONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// A 16-bit SIG UUID on the Bluetooth base UUID.
const fn short(u: u16) -> Uuid {
    Uuid::from_u128(((u as u128) << 96) | 0x0000_0000_0000_1000_8000_0080_5f9b_34fb)
}

/// One serial-over-GATT layout: TX and RX are looked up INSIDE `service`
/// only, by exact UUID, and must carry the properties the role needs.
struct SerialProfile {
    label: &'static str,
    service: Uuid,
    tx: Uuid,
    rx: Uuid,
}

const PROFILES: &[SerialProfile] = &[
    // Generic ELM327 BLE — and the OBDLink CX (fw 5.13.0 exposes only this
    // plus Dialog SUOTA 0xfef5; verified on hardware 2026-09-19).
    SerialProfile {
        label: "fff0 serial",
        service: short(0xfff0),
        tx: short(0xfff2),
        rx: short(0xfff1),
    },
    // LELink-style: ONE characteristic is both write and notify, so tx == rx
    // is deliberate. Not verified on hardware here.
    SerialProfile {
        label: "e7810a71 serial",
        service: Uuid::from_u128(0xe7810a71_73ae_499d_8c15_faa9aef0c3f2),
        tx: Uuid::from_u128(0xbef8d6c9_9c21_4c9e_b632_bd58c1009f9f),
        rx: Uuid::from_u128(0xbef8d6c9_9c21_4c9e_b632_bd58c1009f9f),
    },
    // Nordic UART (OBDX Pro)
    SerialProfile {
        label: "nordic uart",
        service: Uuid::from_u128(0x6e400001_b5a3_f393_e0a9_e50e24dcca9e),
        tx: Uuid::from_u128(0x6e400002_b5a3_f393_e0a9_e50e24dcca9e),
        rx: Uuid::from_u128(0x6e400003_b5a3_f393_e0a9_e50e24dcca9e),
    },
];

/// Picker-worthy: advertises a known serial service, or is named like an OBD adapter.
fn is_obd_candidate(name: &str, advertised: &[Uuid]) -> bool {
    advertised
        .iter()
        .any(|u| PROFILES.iter().any(|p| p.service == *u))
        || name.to_lowercase().contains("obd")
}

/// First profile whose service offers a writable TX and a notifying RX.
fn pick_chars<'a>(
    chars: impl IntoIterator<Item = &'a Characteristic> + Clone,
) -> Result<(Characteristic, Characteristic), String> {
    let writable = CharPropFlags::WRITE | CharPropFlags::WRITE_WITHOUT_RESPONSE;
    let notifying = CharPropFlags::NOTIFY | CharPropFlags::INDICATE;
    for p in PROFILES {
        let find = |uuid: Uuid, need: CharPropFlags| {
            chars
                .clone()
                .into_iter()
                .find(|ch| {
                    ch.service_uuid == p.service
                        && ch.uuid == uuid
                        && ch.properties.intersects(need)
                })
                .cloned()
        };
        if let (Some(tx), Some(rx)) = (find(p.tx, writable), find(p.rx, notifying)) {
            return Ok((tx, rx));
        }
    }
    Err("ble_no_serial_service".to_string())
}

/// Run one connect stage under a deadline; failures carry the stage name.
async fn stage<T, E: Display>(
    name: &str,
    budget: Duration,
    fut: impl Future<Output = Result<T, E>>,
) -> Result<T, String> {
    match tokio::time::timeout(budget, fut).await {
        Ok(Ok(v)) => Ok(v),
        Ok(Err(e)) => Err(format!("ble_{name}_failed: {e}")),
        Err(_) => Err(format!("ble_{name}_timeout")),
    }
}

/// One shared runtime + adapter for scans and links.
struct BleCentral {
    runtime: Runtime,
    adapter: Adapter,
    /// Scan and connect never overlap: BlueZ starves a `Connect()` issued
    /// while discovery is running.
    op: tokio::sync::Mutex<()>,
    /// A connect is waiting for `op` — the in-flight scan ends its window now.
    connect_pending: AtomicBool,
    abort_scan: tokio::sync::Notify,
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
    Ok(CENTRAL.get_or_init(|| BleCentral {
        runtime,
        adapter,
        op: tokio::sync::Mutex::new(()),
        connect_pending: AtomicBool::new(false),
        abort_scan: tokio::sync::Notify::new(),
    }))
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
        let _op = c.op.lock().await;
        let started = c
            .adapter
            .start_scan(ScanFilter::default())
            .await
            .map_err(|e| format!("start_scan: {e}"));
        let listed = match started {
            Ok(()) => scan_window(c, duration).await,
            Err(e) => Err(e),
        };
        // ALWAYS stop — a failed start can still leave a BlueZ discovery
        // session behind, and a running scan starves the next connect.
        if let Err(e) = c.adapter.stop_scan().await {
            if listed.is_ok() {
                eprintln!("[BLE] stop_scan failed: {e}");
            }
        }
        listed
    });
    match result {
        Ok(out) => out,
        Err(e) => {
            eprintln!("[BLE] scan failed: {e}");
            vec![]
        }
    }
}

/// Hold the scan open for `duration` (or until a connect asks for the
/// radio), then list the OBD candidates. NOTE: BlueZ's device list is a cache
/// of everything it has met, so an adapter that is connected elsewhere or
/// unplugged can linger here; filtering on per-window RSSI was tried and
/// dropped (bench 2026-09-19: a CX advertising at -44 dBm showed no RSSI for
/// four rounds running). The bounded connect reports such a stale entry
/// instead (`ble_connect_timeout`).
async fn scan_window(c: &BleCentral, duration: Duration) -> Result<Vec<ConnectorInfo>, String> {
    {
        let aborted = c.abort_scan.notified();
        let mut aborted = std::pin::pin!(aborted);
        aborted.as_mut().enable();
        if !c.connect_pending.load(Ordering::SeqCst) {
            let _ = tokio::time::timeout(duration, aborted).await;
        }
    }
    let mut out = Vec::new();
    let mut seen = 0usize;
    for p in c
        .adapter
        .peripherals()
        .await
        .map_err(|e| format!("peripherals: {e}"))?
    {
        let Ok(Some(props)) = p.properties().await else {
            continue;
        };
        seen += 1;
        let name = props.local_name.unwrap_or_else(|| "Unknown".to_string());
        if is_obd_candidate(&name, &props.services) {
            out.push(ConnectorInfo {
                id: format!("ble:{}", p.id()),
                name,
                connector_type: "ble".to_string(),
            });
        }
    }
    eprintln!("[BLE] scan: {seen} peripheral(s) seen, {} OBD", out.len());
    Ok(out)
}

pub struct BleTransport {
    peripheral: Mutex<Option<Peripheral>>,
    tx_char: Characteristic,
    write_type: WriteType,
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
        // Ask an in-flight scan round to give up the radio, then queue behind it.
        c.connect_pending.store(true, Ordering::SeqCst);
        c.abort_scan.notify_waiters();
        c.runtime.block_on(async {
            let _op = c.op.lock().await;
            c.connect_pending.store(false, Ordering::SeqCst);
            let peripheral = c
                .adapter
                .peripherals()
                .await
                .map_err(|e| e.to_string())?
                .into_iter()
                .find(|p| p.id().to_string() == want)
                .ok_or_else(|| format!("Device {want} not found in scan results"))?;

            // Linux: approve Just Works pairing for THIS device while the
            // link comes up (see ble_agent). Non-fatal — adapters with open
            // characteristics never pair at all.
            #[cfg(target_os = "linux")]
            let _agent =
                match super::ble_agent::PairingAgent::register(&format!("/org/bluez/{want}")) {
                    Ok(a) => Some(a),
                    Err(e) => {
                        eprintln!("[BLE] pairing agent unavailable: {e}");
                        None
                    }
                };

            stage("connect", CONNECT_TIMEOUT, peripheral.connect()).await?;
            let link = async {
                stage("discover", DISCOVER_TIMEOUT, peripheral.discover_services()).await?;
                let chars = peripheral.characteristics();
                let (tx_char, rx_char) = pick_chars(&chars)?;
                stage(
                    "subscribe",
                    SUBSCRIBE_TIMEOUT,
                    peripheral.subscribe(&rx_char),
                )
                .await?;
                Ok::<_, String>(tx_char)
            }
            .await;
            let tx_char = match link {
                Ok(tx) => tx,
                Err(e) => {
                    // Close what we opened: a connected adapter stops
                    // advertising and would vanish from every later scan.
                    let _ = tokio::time::timeout(DISCONNECT_TIMEOUT, peripheral.disconnect()).await;
                    return Err(e);
                }
            };
            let write_type = if tx_char
                .properties
                .contains(CharPropFlags::WRITE_WITHOUT_RESPONSE)
            {
                WriteType::WithoutResponse
            } else {
                WriteType::WithResponse
            };
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
                write_type,
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
                p.write(&self.tx_char, chunk, self.write_type)
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
                    let _ = tokio::time::timeout(DISCONNECT_TIMEOUT, p.disconnect()).await;
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    fn ch(service: Uuid, uuid: Uuid, properties: CharPropFlags) -> Characteristic {
        Characteristic {
            uuid,
            service_uuid: service,
            properties,
            descriptors: BTreeSet::new(),
        }
    }

    const SUOTA: Uuid = short(0xfef5);

    #[test]
    fn short_uuid_sits_on_the_bluetooth_base() {
        assert_eq!(
            short(0xfff1).to_string(),
            "0000fff1-0000-1000-8000-00805f9b34fb"
        );
    }

    /// The OBDLink CX table as read off hardware: a notify characteristic in
    /// the SUOTA service comes first and must NOT be picked.
    #[test]
    fn cx_layout_picks_fff2_and_fff1() {
        let chars = vec![
            ch(
                SUOTA,
                Uuid::from_u128(0x5f78df94_798c_46f5_990a_b3eb6a065c88),
                CharPropFlags::READ | CharPropFlags::NOTIFY,
            ),
            ch(
                SUOTA,
                Uuid::from_u128(0x457871e8_d516_4ca1_9116_57d0b17b9cb2),
                CharPropFlags::WRITE | CharPropFlags::WRITE_WITHOUT_RESPONSE,
            ),
            ch(short(0xfff0), short(0xfff1), CharPropFlags::NOTIFY),
            ch(
                short(0xfff0),
                short(0xfff2),
                CharPropFlags::WRITE | CharPropFlags::WRITE_WITHOUT_RESPONSE,
            ),
        ];
        let (tx, rx) = pick_chars(&chars).unwrap();
        assert_eq!(tx.uuid, short(0xfff2));
        assert_eq!(rx.uuid, short(0xfff1));
    }

    #[test]
    fn single_characteristic_profile_resolves_tx_and_rx_to_it() {
        let p = &PROFILES[1];
        let chars = vec![ch(
            p.service,
            p.tx,
            CharPropFlags::WRITE | CharPropFlags::NOTIFY,
        )];
        let (tx, rx) = pick_chars(&chars).unwrap();
        assert_eq!(tx.uuid, rx.uuid);
    }

    #[test]
    fn right_uuid_in_the_wrong_service_is_ignored() {
        let chars = vec![
            ch(SUOTA, short(0xfff1), CharPropFlags::NOTIFY),
            ch(SUOTA, short(0xfff2), CharPropFlags::WRITE),
        ];
        assert_eq!(pick_chars(&chars).unwrap_err(), "ble_no_serial_service");
    }

    #[test]
    fn roles_need_their_properties() {
        // fff1 readable but not notifying → no usable RX → no profile.
        let chars = vec![
            ch(short(0xfff0), short(0xfff1), CharPropFlags::READ),
            ch(short(0xfff0), short(0xfff2), CharPropFlags::WRITE),
        ];
        assert!(pick_chars(&chars).is_err());
    }

    #[test]
    fn candidates_by_advertised_service_or_name() {
        assert!(is_obd_candidate("Unknown", &[short(0x180a), short(0xfff0)]));
        assert!(is_obd_candidate("OBDLink CX", &[]));
        assert!(!is_obd_candidate("Core200S", &[short(0x180a)]));
    }

    #[test]
    fn stage_names_timeouts_and_failures() {
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .build()
            .unwrap();
        rt.block_on(async {
            let never = std::future::pending::<Result<(), String>>();
            assert_eq!(
                stage("subscribe", Duration::from_millis(20), never)
                    .await
                    .unwrap_err(),
                "ble_subscribe_timeout"
            );
            let failed = async { Err::<(), _>("boom") };
            assert_eq!(
                stage("connect", Duration::from_secs(1), failed)
                    .await
                    .unwrap_err(),
                "ble_connect_failed: boom"
            );
            let fine = async { Ok::<_, String>(7) };
            assert_eq!(
                stage("discover", Duration::from_secs(1), fine)
                    .await
                    .unwrap(),
                7
            );
        });
    }
}
