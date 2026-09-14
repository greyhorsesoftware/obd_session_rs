//! `TcpTransport` (LH5) — WiFi adapters (ELM327 WiFi clones, OBDX Pro
//! WiFi) speak the same byte stream over `192.168.4.1:23`. `std::net` only;
//! works on iOS too (needs `network.client`, and on iOS 14+ the local-network
//! usage description).

use std::io::{Read, Write};
use std::net::{Shutdown, SocketAddr, TcpStream, ToSocketAddrs};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::byte::{ByteSink, ByteTransport, LinkDropSink};
use crate::platform::ConnectorInfo;

pub const DEFAULT_WIFI_ADDR: &str = "192.168.4.1:23";

/// The WiFi adapters we know how to find — each is its own access point,
/// so the address is fixed per vendor. Probed in parallel; one connector
/// per hit, named so the picker (and the catalog) can tell them apart.
pub const WIFI_ADAPTERS: &[(&str, &str)] = &[
    ("192.168.4.1:23", "OBDX Pro (WiFi)"), // OBDX Pro Wireless Connections doc
    ("192.168.0.10:35000", "OBDLink MX WiFi"), // OBDLink MX WiFi (STN)
];

pub struct TcpTransport {
    writer: Mutex<Option<TcpStream>>,
    reader_stop: Arc<std::sync::atomic::AtomicBool>,
}

impl TcpTransport {
    /// Connect (bounded) and start the reader thread. `on_bytes` gets every
    /// chunk as it arrives; `on_drop` fires once when the peer goes away.
    pub fn connect(
        addr: &str,
        timeout: Duration,
        on_bytes: ByteSink,
        on_drop: LinkDropSink,
    ) -> Result<Self, String> {
        let sock: SocketAddr = addr
            .to_socket_addrs()
            .map_err(|e| format!("bad address {addr}: {e}"))?
            .next()
            .ok_or_else(|| format!("bad address {addr}"))?;
        let stream = TcpStream::connect_timeout(&sock, timeout)
            .map_err(|e| format!("connect {addr}: {e}"))?;
        stream.set_nodelay(true).ok();
        let reader = stream.try_clone().map_err(|e| format!("clone: {e}"))?;
        let reader_stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        spawn_reader(reader, Arc::clone(&reader_stop), on_bytes, on_drop);
        Ok(Self {
            writer: Mutex::new(Some(stream)),
            reader_stop,
        })
    }

    /// Discovery: a quick bounded probe of the well-known WiFi adapter
    /// address — present only when the Mac/iPad is on the adapter's network.
    pub fn discover(timeout: Duration) -> Vec<ConnectorInfo> {
        // One thread per known adapter address; two attempts each — on iOS
        // the FIRST socket to a local address is what raises the Local
        // Network permission prompt and fails meanwhile (bench 2026-08-29,
        // iPad + OBDX Pro WiFi never listed).
        let handles: Vec<_> = WIFI_ADAPTERS
            .iter()
            .map(|&(addr, name)| {
                std::thread::spawn(move || {
                    let Ok(mut addrs) = addr.to_socket_addrs() else {
                        return None;
                    };
                    let sock = addrs.next()?;
                    for attempt in 0..2 {
                        match TcpStream::connect_timeout(&sock, timeout) {
                            Ok(s) => {
                                let _ = s.shutdown(Shutdown::Both);
                                return Some(ConnectorInfo {
                                    id: format!("wifi:{addr}"),
                                    name: format!("{name} ({addr})"),
                                    connector_type: "wifi".to_string(),
                                });
                            }
                            Err(e) => {
                                eprintln!(
                                    "[WIFI] probe {addr} attempt {} failed: {e}",
                                    attempt + 1
                                );
                                if attempt == 0 {
                                    std::thread::sleep(Duration::from_millis(300));
                                }
                            }
                        }
                    }
                    None
                })
            })
            .collect();
        handles
            .into_iter()
            .filter_map(|h| h.join().ok().flatten())
            .collect()
    }
}

fn spawn_reader(
    mut reader: TcpStream,
    stop: Arc<std::sync::atomic::AtomicBool>,
    on_bytes: ByteSink,
    on_drop: LinkDropSink,
) {
    let _ = std::thread::Builder::new()
        .name("obd-tcp-reader".into())
        .spawn(move || {
            let mut buf = [0u8; 2048];
            let reason = loop {
                match reader.read(&mut buf) {
                    Ok(0) => break "peer closed".to_string(),
                    Ok(n) => on_bytes(&buf[..n]),
                    Err(e) => {
                        if stop.load(std::sync::atomic::Ordering::SeqCst) {
                            break "closed".to_string();
                        }
                        break format!("read error: {e}");
                    }
                }
            };
            if !stop.load(std::sync::atomic::Ordering::SeqCst) {
                on_drop(reason);
            }
        });
}

impl ByteTransport for TcpTransport {
    fn write(&self, bytes: &[u8]) -> Result<(), String> {
        let mut g = self.writer.lock().unwrap();
        let Some(s) = g.as_mut() else {
            return Err("not connected".to_string());
        };
        s.write_all(bytes).map_err(|e| format!("write: {e}"))
    }

    fn close(&self) {
        self.reader_stop
            .store(true, std::sync::atomic::Ordering::SeqCst);
        if let Some(s) = self.writer.lock().unwrap().take() {
            let _ = s.shutdown(Shutdown::Both);
        }
    }
}
