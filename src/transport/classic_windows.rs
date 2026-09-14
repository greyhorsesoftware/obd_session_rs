//! Bluetooth Classic (SPP) transport for Windows (LH5: a `ByteTransport`).
//! A paired SPP device appears as a COM port; the port is opened as a serial
//! device. Connector ids are `com:COMn`; a reader thread feeds the platform's
//! accumulator through the sink handed in at connect.

#![allow(dead_code)]
#![cfg(target_os = "windows")]

use std::sync::{Arc, Mutex};
use std::time::Duration;

use super::byte::{ByteSink, ByteTransport, LinkDropSink};
use crate::platform::ConnectorInfo;

/// Enumerate Bluetooth SPP COM ports (PowerShell) → `com:` connectors.
pub fn discover(_duration: Duration) -> Vec<ConnectorInfo> {
    use std::process::Command;
    let script = "Get-WmiObject Win32_PnPEntity | Where-Object { $_.Name -match 'Bluetooth' -and $_.Name -match 'COM\\d+' } | ForEach-Object { $_.Name }";
    let Ok(output) = Command::new("powershell")
        .args(["-NoProfile", "-Command", script])
        .output()
    else {
        return vec![];
    };
    String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter_map(|line| {
            let start = line.find("(COM")?;
            let end = line[start..].find(')')? + start;
            let port = &line[start + 1..end];
            Some(ConnectorInfo {
                id: format!("com:{port}"),
                name: line.trim().to_string(),
                connector_type: "classic".to_string(),
            })
        })
        .collect()
}

pub struct WindowsClassicTransport {
    handle: Mutex<Option<usize>>, // HANDLE as usize (Send)
    reader_stop: Arc<std::sync::atomic::AtomicBool>,
}

impl WindowsClassicTransport {
    pub fn connect(
        connector_id: &str,
        on_bytes: ByteSink,
        on_drop: LinkDropSink,
    ) -> Result<Self, String> {
        let port = connector_id.strip_prefix("com:").unwrap_or(connector_id);
        let path =
            std::ffi::CString::new(format!("\\\\.\\{port}")).map_err(|_| "bad port".to_string())?;
        const GENERIC_READ: u32 = 0x8000_0000;
        const GENERIC_WRITE: u32 = 0x4000_0000;
        const OPEN_EXISTING: u32 = 3;
        let handle = unsafe {
            kernel32::CreateFileA(
                path.as_ptr(),
                GENERIC_READ | GENERIC_WRITE,
                0,
                std::ptr::null_mut(),
                OPEN_EXISTING,
                0,
                std::ptr::null_mut(),
            )
        };
        if handle as isize == -1 {
            return Err(format!("open {port} failed"));
        }
        let h = handle as usize;
        let reader_stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        {
            let stop = Arc::clone(&reader_stop);
            let _ = std::thread::Builder::new()
                .name("obd-com-reader".into())
                .spawn(move || {
                    let mut buf = [0u8; 2048];
                    loop {
                        if stop.load(std::sync::atomic::Ordering::SeqCst) {
                            return;
                        }
                        let mut n: u32 = 0;
                        let ok = unsafe {
                            kernel32::ReadFile(
                                h as *mut _,
                                buf.as_mut_ptr() as *mut _,
                                buf.len() as u32,
                                &mut n,
                                std::ptr::null_mut(),
                            )
                        };
                        if ok == 0 {
                            if !stop.load(std::sync::atomic::Ordering::SeqCst) {
                                on_drop("read error".to_string());
                            }
                            return;
                        }
                        if n > 0 {
                            on_bytes(&buf[..n as usize]);
                        } else {
                            std::thread::sleep(Duration::from_millis(10));
                        }
                    }
                });
        }
        Ok(Self {
            handle: Mutex::new(Some(h)),
            reader_stop,
        })
    }
}

impl ByteTransport for WindowsClassicTransport {
    fn write(&self, bytes: &[u8]) -> Result<(), String> {
        let g = self.handle.lock().unwrap();
        let Some(h) = *g else {
            return Err("not connected".to_string());
        };
        let mut written: u32 = 0;
        let ok = unsafe {
            kernel32::WriteFile(
                h as *mut _,
                bytes.as_ptr() as *const _,
                bytes.len() as u32,
                &mut written,
                std::ptr::null_mut(),
            )
        };
        if ok == 0 {
            Err("WRITE_FAILED".to_string())
        } else {
            Ok(())
        }
    }

    fn close(&self) {
        self.reader_stop
            .store(true, std::sync::atomic::Ordering::SeqCst);
        if let Some(h) = self.handle.lock().unwrap().take() {
            unsafe { kernel32::CloseHandle(h as *mut _) };
        }
    }
}

mod kernel32 {
    use std::ffi::c_void;
    use std::os::raw::c_char;
    extern "system" {
        pub fn CreateFileA(
            lpFileName: *const c_char,
            dwDesiredAccess: u32,
            dwShareMode: u32,
            lpSecurityAttributes: *mut c_void,
            dwCreationDisposition: u32,
            dwFlagsAndAttributes: u32,
            hTemplateFile: *mut c_void,
        ) -> *mut c_void;
        pub fn WriteFile(
            hFile: *mut c_void,
            lpBuffer: *const c_void,
            nNumberOfBytesToWrite: u32,
            lpNumberOfBytesWritten: *mut u32,
            lpOverlapped: *mut c_void,
        ) -> i32;
        pub fn ReadFile(
            hFile: *mut c_void,
            lpBuffer: *mut c_void,
            nNumberOfBytesToRead: u32,
            lpNumberOfBytesRead: *mut u32,
            lpOverlapped: *mut c_void,
        ) -> i32;
        pub fn CloseHandle(hObject: *mut c_void) -> i32;
    }
}
