//! Session audit logger + wire log (WIRE-LOG, DVI_BLE_TX_Plan.md)
//!
//! Writes TWO append-only files per app launch, stem-paired in the same
//! directory, from ONE dedicated logger thread:
//!
//! - `<session_id>.jsonl` — the audit log (commands, responses, callbacks,
//!   subscription lifecycle) as JSON Lines. Append-only so a crash
//!   mid-session never leaves an unparseable file.
//! - `adapter_<ts>.log` — the wire log: every TX/RX/DROP on every transport
//!   (USB, WiFi, classic BT, BLE, EA, Mock) plus host NOTE annotations,
//!   `# connect` headers per connection, 25 MB two-half rotation.
//!
//! THREADING CONTRACT (WIRE-LOG.a): callers NEVER touch a file. Every log
//! call formats its line and does one non-blocking channel send; the logger
//! thread owns both files behind BufWriters. Flushes: ~1 s tick, IMMEDIATE
//! on fault-class events, on explicit `flush_sync`, and at shutdown. If the
//! channel fills (disk stall), lines are DROPPED AND COUNTED — acquisition
//! is never blocked by logging — and the drop count is written as a NOTE
//! once the channel drains.

use serde_json::json;
use std::fs::{create_dir_all, File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, RecvTimeoutError, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Wire-line direction.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum WireDir {
    Tx,
    Rx,
    Drop,
    Note,
}

impl WireDir {
    fn label(self) -> &'static str {
        match self {
            WireDir::Tx => "TX",
            WireDir::Rx => "RX",
            WireDir::Drop => "DROP",
            WireDir::Note => "NOTE",
        }
    }
}

enum Msg {
    /// Preformatted jsonl line; `fault` forces an immediate flush of both files.
    Jsonl {
        line: String,
        fault: bool,
    },
    Wire {
        ts_ms: u64,
        dir: WireDir,
        text: String,
    },
    Connect {
        ts_ms: u64,
        transport: String,
        adapter: String,
    },
    /// Flush both files and ack.
    Flush(SyncSender<()>),
    Shutdown,
}

/// Default wire-file rotation threshold: rotate current → `.old` at 25 MB,
/// keeping the newest 25–50 MB on disk (WIRE-LOG.b).
const WIRE_ROTATE_BYTES: u64 = 25 * 1024 * 1024;
/// Channel capacity — at wire rate (~250 lines/s) this is ~30 s of headroom.
const CHANNEL_CAP: usize = 8192;

/// Event/callback kinds whose appearance must be on disk immediately — a
/// crash must not eat the second that mattered (post-mortem rule).
fn is_fault_kind(kind: &str) -> bool {
    matches!(
        kind,
        "error_received"
            | "anomaly"
            | "dvi_desync"
            | "dvi_resync"
            | "chunk_validation_failure"
            | "session_end"
    )
}

/// Session logger — public API unchanged from the synchronous version;
/// everything now rides the logger thread.
pub struct SessionLogger {
    tx: SyncSender<Msg>,
    /// Lines dropped because the channel was full (disk stall). Reported as
    /// a NOTE by the logger thread when it next drains.
    dropped: Arc<AtomicU64>,
    jsonl_path: PathBuf,
    wire_path: PathBuf,
    thread: Mutex<Option<std::thread::JoinHandle<()>>>,
}

impl SessionLogger {
    /// Create a new session logger.
    ///
    /// * `base_path` — directory for both files (created if needed)
    /// * `session_id` — jsonl filename stem; the wire filename derives from
    ///   it (`obd_session_<ts>` → `adapter_<ts>.log`) so the pair is
    ///   stem-matched for the FB7 diagnostic archive and retention.
    pub fn new(base_path: &str, session_id: &str) -> Result<Self, std::io::Error> {
        Self::new_with_rotate(base_path, session_id, WIRE_ROTATE_BYTES)
    }

    /// Rotation threshold injected for tests.
    pub(crate) fn new_with_rotate(
        base_path: &str,
        session_id: &str,
        rotate_bytes: u64,
    ) -> Result<Self, std::io::Error> {
        let dir = PathBuf::from(base_path);
        create_dir_all(&dir)?;

        let jsonl_path = dir.join(format!("{}.jsonl", session_id));
        let wire_stem = session_id
            .strip_prefix("obd_session_")
            .map(|rest| format!("adapter_{}", rest))
            .unwrap_or_else(|| format!("adapter_{}", session_id));
        let wire_path = dir.join(format!("{}.log", wire_stem));

        // The jsonl is opened up front (session_start is the first line);
        // the wire file is created LAZILY on its first line, so a launch
        // that never connects leaves no empty adapter file.
        let jsonl_file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&jsonl_path)?;

        let (tx, rx) = sync_channel::<Msg>(CHANNEL_CAP);
        let dropped = Arc::new(AtomicU64::new(0));
        let thread = spawn_writer(
            rx,
            jsonl_file,
            wire_path.clone(),
            rotate_bytes,
            Arc::clone(&dropped),
        );

        let logger = Self {
            tx,
            dropped,
            jsonl_path,
            wire_path,
            thread: Mutex::new(Some(thread)),
        };

        logger.log_event("session_start", json!({}));
        Ok(logger)
    }

    /// Current timestamp in epoch milliseconds.
    fn timestamp_ms() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64
    }

    /// Non-blocking send; a full channel increments the drop counter instead
    /// of ever blocking the caller (acquisition hot paths log through here).
    fn send(&self, msg: Msg) {
        match self.tx.try_send(msg) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => {
                self.dropped.fetch_add(1, Ordering::Relaxed);
            }
            Err(TrySendError::Disconnected(_)) => {}
        }
    }

    /// Append one audit record: `fields` plus the common `ts` + `event`
    /// keys, as a single JSON line. Formatting happens on the caller; file
    /// I/O happens on the logger thread.
    fn log_event(&self, event: &str, mut fields: serde_json::Value) {
        let mut fault = is_fault_kind(event);
        if let Some(obj) = fields.as_object_mut() {
            obj.insert("ts".to_string(), json!(Self::timestamp_ms()));
            obj.insert("event".to_string(), json!(event));
            if let Some(t) = obj.get("type").and_then(|t| t.as_str()) {
                fault = fault || is_fault_kind(t);
            }
        }
        self.send(Msg::Jsonl {
            line: fields.to_string(),
            fault,
        });
    }

    /// Log a command being sent
    pub fn log_command_sent(&self, command: &str, timeout_ms: u32) {
        self.log_event(
            "command_sent",
            json!({ "cmd": command, "timeout_ms": timeout_ms }),
        );
    }

    /// Log a response received
    pub fn log_response_received(&self, command: &str, response: &str) {
        self.log_event(
            "response_received",
            json!({ "cmd": command, "response": response }),
        );
    }

    /// Log an error response
    pub fn log_error_received(&self, command: &str, error: &str) {
        self.log_event("error_received", json!({ "cmd": command, "error": error }));
    }

    /// Log callback invocation
    pub fn log_callback(&self, callback_type: &str, data: &str) {
        self.log_event("callback", json!({ "type": callback_type, "data": data }));
    }

    /// Log subscription event
    pub fn log_subscription(&self, event: &str, subscription_id: &str, details: &str) {
        self.log_event(
            "subscription",
            json!({ "sub_event": event, "id": subscription_id, "details": details }),
        );
    }

    /// Log session end
    pub fn log_session_end(&self) {
        self.log_event("session_end", json!({}));
    }

    // --- WIRE-LOG.b: the wire file ---------------------------------------

    /// One wire line: `<epoch_ms> <TX|RX|DROP|NOTE> <bytes> <rtt|-> <raw>`.
    /// RTT is computed on the logger thread (first RX after a TX).
    pub fn log_wire(&self, dir: WireDir, text: &str) {
        self.send(Msg::Wire {
            ts_ms: Self::timestamp_ms(),
            dir,
            text: text.to_string(),
        });
    }

    /// `# connect <epoch_ms> <transport> <adapter>` header — written at each
    /// connection so multi-connect launches stay attributable (and the
    /// WiFi-lines-labeled-"Bluetooth" misrouting dies).
    pub fn log_connect_header(&self, transport: &str, adapter: &str) {
        self.send(Msg::Connect {
            ts_ms: Self::timestamp_ms(),
            transport: transport.to_string(),
            adapter: adapter.to_string(),
        });
    }

    /// Host-side annotation (the `obd_log_transport_note` FFI lands here):
    /// BLE canSend stalls, chunk splits, delay waits — things only the host
    /// write path can see, interleaved inline with the wire rows.
    pub fn log_note(&self, kind: &str, text: &str) {
        self.log_wire(WireDir::Note, &format!("{kind}: {text}"));
    }

    /// Flush both files and wait for the ack (bounded). Used by tests, the
    /// host's resignActive flush, and shutdown.
    pub fn flush_sync(&self) {
        let (ack_tx, ack_rx) = sync_channel::<()>(1);
        // Flush must get through even when the data channel is saturated —
        // a blocking send is bounded in practice (the thread drains 8k lines
        // in well under a second) and returns Err instantly if the thread is
        // gone (channel disconnected).
        if self.tx.send(Msg::Flush(ack_tx)).is_ok() {
            let _ = ack_rx.recv_timeout(Duration::from_secs(2));
        }
    }

    /// Get the jsonl log file path
    pub fn path(&self) -> &PathBuf {
        &self.jsonl_path
    }

    /// The wire-log path (`adapter_<ts>.log`).
    pub fn wire_path(&self) -> &PathBuf {
        &self.wire_path
    }
}

impl Drop for SessionLogger {
    fn drop(&mut self) {
        self.log_session_end();
        let _ = self.tx.send(Msg::Shutdown);
        if let Ok(mut guard) = self.thread.lock() {
            if let Some(handle) = guard.take() {
                let _ = handle.join();
            }
        }
    }
}

// --- the logger thread ----------------------------------------------------

struct WireFile {
    path: PathBuf,
    rotate_bytes: u64,
    writer: Option<BufWriter<File>>,
    bytes_written: u64,
    /// Last TX instant — the next RX line carries `now - last_tx` as RTT,
    /// then the marker clears (matches the ring's first-response-only rule).
    last_tx: Option<Instant>,
}

impl WireFile {
    fn ensure_open(&mut self) -> Option<&mut BufWriter<File>> {
        if self.writer.is_none() {
            let file = OpenOptions::new()
                .create(true)
                .append(true)
                .open(&self.path)
                .ok()?;
            self.bytes_written = file.metadata().map(|m| m.len()).unwrap_or(0);
            self.writer = Some(BufWriter::new(file));
        }
        self.writer.as_mut()
    }

    fn write_line(&mut self, line: &str) {
        let len = line.len() as u64 + 1;
        if let Some(w) = self.ensure_open() {
            let _ = writeln!(w, "{}", line);
        }
        self.bytes_written += len;
        if self.bytes_written >= self.rotate_bytes {
            self.rotate();
        }
    }

    /// Two-half rotation: flush + close, current → `.old` (replacing any
    /// previous `.old`), start fresh. The newest window is always on disk.
    fn rotate(&mut self) {
        if let Some(mut w) = self.writer.take() {
            let _ = w.flush();
        }
        let old = self.path.with_extension("log.old");
        let _ = std::fs::remove_file(&old);
        let _ = std::fs::rename(&self.path, &old);
        self.bytes_written = 0;
        // Reopened lazily on the next line.
    }

    fn flush(&mut self) {
        if let Some(w) = self.writer.as_mut() {
            let _ = w.flush();
        }
    }
}

fn spawn_writer(
    rx: Receiver<Msg>,
    jsonl_file: File,
    wire_path: PathBuf,
    rotate_bytes: u64,
    dropped: Arc<AtomicU64>,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .name("obd-session-logger".into())
        .spawn(move || {
            let mut jsonl = BufWriter::new(jsonl_file);
            let mut wire = WireFile {
                path: wire_path,
                rotate_bytes,
                writer: None,
                bytes_written: 0,
                last_tx: None,
            };
            let mut dirty = false;
            let mut last_flush = Instant::now();

            loop {
                match rx.recv_timeout(Duration::from_millis(250)) {
                    Ok(msg) => {
                        // Surface any drop streak before the next line, so
                        // the gap is visible where it happened.
                        let lost = dropped.swap(0, Ordering::Relaxed);
                        if lost > 0 {
                            wire.write_line(&format!(
                                "{} NOTE      0      - logger channel full: {} line(s) dropped",
                                SessionLogger::timestamp_ms(),
                                lost
                            ));
                        }
                        match msg {
                            Msg::Jsonl { line, fault } => {
                                let _ = writeln!(jsonl, "{}", line);
                                dirty = true;
                                if fault {
                                    let _ = jsonl.flush();
                                    wire.flush();
                                    dirty = false;
                                    last_flush = Instant::now();
                                }
                            }
                            Msg::Wire { ts_ms, dir, text } => {
                                let rtt = match dir {
                                    WireDir::Tx => {
                                        wire.last_tx = Some(Instant::now());
                                        None
                                    }
                                    WireDir::Rx => {
                                        wire.last_tx.take().map(|t| t.elapsed().as_millis() as u64)
                                    }
                                    _ => None,
                                };
                                let rtt_col = rtt
                                    .map(|ms| format!("{}ms", ms))
                                    .unwrap_or_else(|| "-".to_string());
                                wire.write_line(&format!(
                                    "{} {:<4} {:>6} {:>7} {}",
                                    ts_ms,
                                    dir.label(),
                                    text.len(),
                                    rtt_col,
                                    text
                                ));
                                dirty = true;
                            }
                            Msg::Connect {
                                ts_ms,
                                transport,
                                adapter,
                            } => {
                                wire.write_line(&format!(
                                    "# connect {} {} {}",
                                    ts_ms, transport, adapter
                                ));
                                wire.last_tx = None;
                                dirty = true;
                            }
                            Msg::Flush(ack) => {
                                let _ = jsonl.flush();
                                wire.flush();
                                dirty = false;
                                last_flush = Instant::now();
                                let _ = ack.try_send(());
                            }
                            Msg::Shutdown => break,
                        }
                    }
                    Err(RecvTimeoutError::Timeout) => {}
                    Err(RecvTimeoutError::Disconnected) => break,
                }

                if dirty && last_flush.elapsed() >= Duration::from_secs(1) {
                    let _ = jsonl.flush();
                    wire.flush();
                    dirty = false;
                    last_flush = Instant::now();
                }
            }

            let _ = jsonl.flush();
            wire.flush();
        })
        .expect("spawn session logger thread")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn temp_dir(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(name);
        let _ = fs::remove_dir_all(&dir);
        dir
    }

    #[test]
    fn test_session_logger() {
        let dir = temp_dir("obd_test_logs");

        let logger = SessionLogger::new(dir.to_str().unwrap(), "test_session").unwrap();

        logger.log_command_sent("010C", 5000);
        logger.log_response_received("010C", "7E8 04 41 0C 1A F8");
        logger.log_callback("obd_data", "{...}");
        logger.log_subscription("created", "abc-123", "pids=[010C, 010D]");
        logger.flush_sync();

        // Verify file exists, with the .jsonl extension
        assert!(logger.path().exists());
        assert_eq!(
            logger.path().extension().and_then(|e| e.to_str()),
            Some("jsonl")
        );

        // Every line must parse as a JSON object with ts + event
        let content = fs::read_to_string(logger.path()).unwrap();
        let records: Vec<serde_json::Value> = content
            .lines()
            .map(|l| serde_json::from_str(l).expect("each line is valid JSON"))
            .collect();
        assert!(records
            .iter()
            .all(|r| r.get("ts").is_some() && r.get("event").is_some()));

        let events: Vec<&str> = records
            .iter()
            .map(|r| r["event"].as_str().unwrap())
            .collect();
        assert_eq!(
            events,
            vec![
                "session_start",
                "command_sent",
                "response_received",
                "callback",
                "subscription"
            ]
        );

        // Field-level checks
        assert_eq!(records[1]["cmd"], "010C");
        assert_eq!(records[1]["timeout_ms"], 5000);
        assert_eq!(records[2]["response"], "7E8 04 41 0C 1A F8");
        assert_eq!(records[4]["sub_event"], "created");
        assert_eq!(records[4]["id"], "abc-123");

        // No wire lines were logged — no adapter file may exist (lazy open).
        assert!(!logger.wire_path().exists());

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn wire_log_lines_headers_rtt_and_pairing() {
        let dir = temp_dir("obd_test_wire");
        let logger =
            SessionLogger::new(dir.to_str().unwrap(), "obd_session_20260831_0101").unwrap();

        // Stem pairing: obd_session_<ts>.jsonl ↔ adapter_<ts>.log
        assert_eq!(
            logger.wire_path().file_name().unwrap().to_str().unwrap(),
            "adapter_20260831_0101.log"
        );

        logger.log_connect_header("wifi", "OBDX Pro");
        logger.log_wire(WireDir::Tx, "31 02 06 01");
        logger.log_wire(WireDir::Rx, "44 02 06 01");
        logger.log_wire(WireDir::Rx, "08 06 00 00 07 E8 41 0C");
        logger.log_note("ble_stall", "canSend false for 12ms");
        logger.flush_sync();

        let content = fs::read_to_string(logger.wire_path()).unwrap();
        let lines: Vec<&str> = content.lines().collect();
        assert_eq!(lines.len(), 5, "{content}");
        assert!(lines[0].starts_with("# connect ") && lines[0].ends_with("wifi OBDX Pro"));
        assert!(lines[1].contains(" TX "));
        // First RX after the TX carries an RTT; the second does not.
        assert!(lines[2].contains("ms "), "rtt on first rx: {}", lines[2]);
        assert!(
            lines[3].contains(" - "),
            "no rtt on pushed rx: {}",
            lines[3]
        );
        assert!(lines[4].contains("NOTE") && lines[4].contains("ble_stall"));

        let _ = fs::remove_dir_all(&dir);
    }

    #[test]
    fn wire_log_rotates_two_halves() {
        let dir = temp_dir("obd_test_wire_rotate");
        let logger = SessionLogger::new_with_rotate(
            dir.to_str().unwrap(),
            "obd_session_rot",
            400, // rotate every ~400 bytes for the test
        )
        .unwrap();

        for i in 0..40 {
            logger.log_wire(
                WireDir::Rx,
                &format!("frame {:02} payload payload payload", i),
            );
        }
        logger.flush_sync();

        let current = fs::read_to_string(logger.wire_path()).unwrap();
        let old_path = logger.wire_path().with_extension("log.old");
        assert!(old_path.exists(), "rotation must leave an .old half");
        let old = fs::read_to_string(&old_path).unwrap();
        // Newest line is in the current half; both halves are non-empty.
        assert!(current.contains("frame 39"));
        assert!(!old.is_empty());

        let _ = fs::remove_dir_all(&dir);
    }
}
