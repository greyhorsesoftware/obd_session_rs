//! `SerialTransport` (LH5) — USB adapters that enumerate as a CDC serial
//! port (`/dev/cu.usbmodem*` on macOS: OBDX Pro `0483:5740`, STN USB, …).
//! Raw termios (`libc`), 115200 8N1 by default; a reader thread feeds the
//! platform's accumulator. macOS / Linux only — iOS has no serial devices.

#![cfg(any(target_os = "macos", target_os = "linux"))]

use std::sync::{Arc, Mutex};

use super::byte::{ByteSink, ByteTransport, LinkDropSink};
use crate::platform::ConnectorInfo;

pub struct SerialTransport {
    fd: Mutex<Option<i32>>,
    reader_stop: Arc<std::sync::atomic::AtomicBool>,
}

impl SerialTransport {
    pub fn open(
        path: &str,
        baud: u32,
        on_bytes: ByteSink,
        on_drop: LinkDropSink,
    ) -> Result<Self, String> {
        let cpath = std::ffi::CString::new(path).map_err(|_| "bad path".to_string())?;
        // O_NONBLOCK for the open only (a modem-control line can block open
        // forever); cleared right after so reads block with a timeout.
        let fd = unsafe {
            libc::open(
                cpath.as_ptr(),
                libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK,
            )
        };
        if fd < 0 {
            return Err(format!("open {path}: {}", std::io::Error::last_os_error()));
        }
        unsafe {
            let flags = libc::fcntl(fd, libc::F_GETFL);
            libc::fcntl(fd, libc::F_SETFL, flags & !libc::O_NONBLOCK);
            let mut tio: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(fd, &mut tio) != 0 {
                libc::close(fd);
                return Err(format!(
                    "tcgetattr {path}: {}",
                    std::io::Error::last_os_error()
                ));
            }
            libc::cfmakeraw(&mut tio);
            tio.c_cflag |= libc::CLOCAL | libc::CREAD;
            tio.c_cflag &= !(libc::PARENB | libc::CSTOPB | libc::CSIZE);
            tio.c_cflag |= libc::CS8;
            // Blocking read: return as soon as ≥1 byte, or after 100 ms.
            tio.c_cc[libc::VMIN] = 0;
            tio.c_cc[libc::VTIME] = 1;
            let speed = baud_constant(baud);
            libc::cfsetispeed(&mut tio, speed);
            libc::cfsetospeed(&mut tio, speed);
            if libc::tcsetattr(fd, libc::TCSANOW, &tio) != 0 {
                libc::close(fd);
                return Err(format!(
                    "tcsetattr {path}: {}",
                    std::io::Error::last_os_error()
                ));
            }
            libc::tcflush(fd, libc::TCIOFLUSH);
        }
        let reader_stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        spawn_reader(fd, Arc::clone(&reader_stop), on_bytes, on_drop);
        Ok(Self {
            fd: Mutex::new(Some(fd)),
            reader_stop,
        })
    }

    /// Discovery: every `/dev/cu.usbmodem*` / `/dev/cu.usbserial*` (macOS)
    /// or `/dev/ttyACM*` / `/dev/ttyUSB*` (Linux).
    pub fn discover() -> Vec<ConnectorInfo> {
        let mut out = Vec::new();
        let Ok(entries) = std::fs::read_dir("/dev") else {
            return out;
        };
        let mut names: Vec<String> = entries
            .filter_map(|e| e.ok())
            .map(|e| e.file_name().to_string_lossy().to_string())
            .filter(|n| {
                n.starts_with("cu.usbmodem")
                    || n.starts_with("cu.usbserial")
                    || n.starts_with("ttyACM")
                    || n.starts_with("ttyUSB")
            })
            .collect();
        names.sort();
        for n in names {
            // A CDC-ACM modem port (`cu.usbmodem*` / `ttyACM*`) is the OBDX
            // Pro's STM32 USB — name it as the adapter (David 2026-08-29:
            // "just OBDX Pro, type usb"); FTDI-style `usbserial`/`ttyUSB`
            // ports stay generic. The id keeps the device path either way.
            let name = if n.starts_with("cu.usbmodem") || n.starts_with("ttyACM") {
                "OBDX Pro".to_string()
            } else {
                format!("USB serial adapter ({n})")
            };
            out.push(ConnectorInfo {
                id: format!("usb:/dev/{n}"),
                name,
                connector_type: "usb".to_string(),
            });
        }
        out
    }
}

fn baud_constant(baud: u32) -> libc::speed_t {
    match baud {
        9600 => libc::B9600,
        19200 => libc::B19200,
        38400 => libc::B38400,
        57600 => libc::B57600,
        230400 => libc::B230400,
        _ => libc::B115200,
    }
}

fn spawn_reader(
    fd: i32,
    stop: Arc<std::sync::atomic::AtomicBool>,
    on_bytes: ByteSink,
    on_drop: LinkDropSink,
) {
    let _ = std::thread::Builder::new()
        .name("obd-serial-reader".into())
        .spawn(move || {
            let mut buf = [0u8; 2048];
            let reason = loop {
                if stop.load(std::sync::atomic::Ordering::SeqCst) {
                    break "closed".to_string();
                }
                let n = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
                if n > 0 {
                    on_bytes(&buf[..n as usize]);
                } else if n < 0 {
                    let err = std::io::Error::last_os_error();
                    if err.kind() == std::io::ErrorKind::Interrupted {
                        continue;
                    }
                    if stop.load(std::sync::atomic::Ordering::SeqCst) {
                        break "closed".to_string();
                    }
                    break format!("read error: {err}");
                }
                // n == 0: VTIME expired with nothing — loop (checks `stop`).
            };
            if !stop.load(std::sync::atomic::Ordering::SeqCst) {
                on_drop(reason);
            }
        });
}

impl ByteTransport for SerialTransport {
    fn write(&self, bytes: &[u8]) -> Result<(), String> {
        let g = self.fd.lock().unwrap();
        let Some(fd) = *g else {
            return Err("not connected".to_string());
        };
        let mut off = 0usize;
        while off < bytes.len() {
            let n = unsafe {
                libc::write(
                    fd,
                    bytes[off..].as_ptr() as *const libc::c_void,
                    bytes.len() - off,
                )
            };
            if n < 0 {
                return Err(format!("write: {}", std::io::Error::last_os_error()));
            }
            off += n as usize;
        }
        Ok(())
    }

    fn close(&self) {
        self.reader_stop
            .store(true, std::sync::atomic::Ordering::SeqCst);
        if let Some(fd) = self.fd.lock().unwrap().take() {
            unsafe { libc::close(fd) };
        }
    }
}
