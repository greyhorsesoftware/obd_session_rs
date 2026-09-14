//! Bluetooth Classic (SPP/RFCOMM) transport for Linux (LH5: a `ByteTransport`).
//! BlueZ RFCOMM socket; discovery via `hcitool scan`. Connector ids are
//! `rfcomm:<AA:BB:CC:DD:EE:FF>`; the reader thread feeds the platform's
//! accumulator through the sink handed in at connect.

#![allow(dead_code)]

use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::byte::{ByteSink, ByteTransport, LinkDropSink};
use crate::platform::ConnectorInfo;

/// RFCOMM channel commonly used by ELM327 adapters
const DEFAULT_RFCOMM_CHANNEL: u8 = 1;

/// Scan for paired/visible Classic devices → `rfcomm:` connectors.
pub fn discover(_duration: Duration) -> Vec<ConnectorInfo> {
    use std::process::Command;
    let Ok(output) = Command::new("hcitool").args(["scan", "--flush"]).output() else {
        return vec![];
    };
    if !output.status.success() {
        return vec![];
    }
    String::from_utf8_lossy(&output.stdout)
        .lines()
        .skip(1)
        .filter_map(|line| {
            let mut parts = line.trim().splitn(2, char::is_whitespace);
            let addr = parts.next()?.trim();
            let name = parts.next().unwrap_or("Unknown").trim();
            if addr.split(':').count() != 6 {
                return None;
            }
            Some(ConnectorInfo {
                id: format!("rfcomm:{addr}"),
                name: name.to_string(),
                connector_type: "classic".to_string(),
            })
        })
        .collect()
}

pub struct LinuxClassicTransport {
    fd: Mutex<Option<i32>>,
    reader_stop: Arc<std::sync::atomic::AtomicBool>,
}

impl LinuxClassicTransport {
    pub fn connect(
        connector_id: &str,
        on_bytes: ByteSink,
        on_drop: LinkDropSink,
    ) -> Result<Self, String> {
        use std::mem;
        use std::os::raw::c_int;
        const AF_BLUETOOTH: c_int = 31;
        const BTPROTO_RFCOMM: c_int = 3;
        let address = connector_id.strip_prefix("rfcomm:").unwrap_or(connector_id);
        let fd = unsafe { libc::socket(AF_BLUETOOTH, libc::SOCK_STREAM, BTPROTO_RFCOMM) };
        if fd < 0 {
            return Err(format!(
                "RFCOMM socket: {}",
                std::io::Error::last_os_error()
            ));
        }
        #[repr(C)]
        struct SockaddrRc {
            rc_family: u16,
            rc_bdaddr: [u8; 6],
            rc_channel: u8,
        }
        let addr = SockaddrRc {
            rc_family: AF_BLUETOOTH as u16,
            rc_bdaddr: parse_bt_address(address)?,
            rc_channel: DEFAULT_RFCOMM_CHANNEL,
        };
        let r = unsafe {
            libc::connect(
                fd,
                &addr as *const SockaddrRc as *const libc::sockaddr,
                mem::size_of::<SockaddrRc>() as u32,
            )
        };
        if r < 0 {
            unsafe { libc::close(fd) };
            return Err(format!(
                "RFCOMM connect {address}: {}",
                std::io::Error::last_os_error()
            ));
        }
        // Bounded reads so `close()` can stop the reader.
        let tv = libc::timeval {
            tv_sec: 0,
            tv_usec: 200_000,
        };
        unsafe {
            libc::setsockopt(
                fd,
                libc::SOL_SOCKET,
                libc::SO_RCVTIMEO,
                &tv as *const _ as *const libc::c_void,
                mem::size_of::<libc::timeval>() as u32,
            );
        }
        let reader_stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        {
            let stop = Arc::clone(&reader_stop);
            let _ = std::thread::Builder::new()
                .name("obd-rfcomm-reader".into())
                .spawn(move || {
                    let mut buf = [0u8; 2048];
                    let reason = loop {
                        if stop.load(std::sync::atomic::Ordering::SeqCst) {
                            break "closed".to_string();
                        }
                        let n = unsafe {
                            libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len())
                        };
                        if n > 0 {
                            on_bytes(&buf[..n as usize]);
                        } else if n == 0 {
                            break "peer closed".to_string();
                        } else {
                            let err = std::io::Error::last_os_error();
                            if matches!(
                                err.kind(),
                                std::io::ErrorKind::WouldBlock
                                    | std::io::ErrorKind::TimedOut
                                    | std::io::ErrorKind::Interrupted
                            ) {
                                continue;
                            }
                            break format!("read error: {err}");
                        }
                    };
                    if !stop.load(std::sync::atomic::Ordering::SeqCst) {
                        on_drop(reason);
                    }
                });
        }
        Ok(Self {
            fd: Mutex::new(Some(fd)),
            reader_stop,
        })
    }
}

/// Parse "AA:BB:CC:DD:EE:FF" to [u8; 6] (BlueZ bdaddr is little-endian).
fn parse_bt_address(address: &str) -> Result<[u8; 6], String> {
    let parts: Vec<&str> = address.split(':').collect();
    if parts.len() != 6 {
        return Err(format!("Invalid Bluetooth address: {address}"));
    }
    let mut bytes = [0u8; 6];
    for (i, part) in parts.iter().enumerate() {
        bytes[5 - i] =
            u8::from_str_radix(part, 16).map_err(|_| format!("Invalid hex in address: {part}"))?;
    }
    Ok(bytes)
}

impl ByteTransport for LinuxClassicTransport {
    fn write(&self, bytes: &[u8]) -> Result<(), String> {
        let g = self.fd.lock().unwrap();
        let Some(fd) = *g else {
            return Err("not connected".to_string());
        };
        let n = unsafe { libc::write(fd, bytes.as_ptr() as *const libc::c_void, bytes.len()) };
        if n < 0 {
            Err(format!("write: {}", std::io::Error::last_os_error()))
        } else {
            Ok(())
        }
    }

    fn close(&self) {
        self.reader_stop
            .store(true, std::sync::atomic::Ordering::SeqCst);
        if let Some(fd) = self.fd.lock().unwrap().take() {
            unsafe { libc::close(fd) };
        }
    }
}
