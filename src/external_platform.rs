//! External platform implementation for Swift/native integration
//!
//! This module provides a platform that delegates actual OBD communication
//! to an external implementation (e.g., Swift handling Bluetooth).
//! Rust handles command queuing, subscriptions, and response processing.

use std::os::raw::{c_char, c_int};
use std::sync::{mpsc, Arc, Mutex};
use std::time::{Duration, Instant};

use crate::accumulator::{AccEvent, Mode, ResponseAccumulator};

use crate::platform::{
    ConnectResult, ConnectionCallback, ConnectionStatus, ConnectorInfo, OBDDataCallback,
    OBDPlatformInterface,
};

/// Callback type for sending commands to external platform (Swift)
/// Parameters: (command, timeout_ms, context)
/// RS7: unified with the FFI-exported typedef — `OBDSendCallback` is the one
/// name (it is what the generated header emits and Swift binds against).
use crate::ffi::OBDSendCallback;

/// Callback type for connector discovery results from Swift
/// Parameters: (connectors_json, context)
/// connectors_json is a JSON array of ConnectorInfo objects
pub type DiscoverConnectorsCallback = extern "C" fn(*const c_char, *mut std::ffi::c_void);

/// Callback type for connect result from Swift
/// Parameters: (success, error_message, context)
/// success: 1 = connected, 0 = failed, -1 = cancelled
/// error_message: NULL on success, error string on failure
pub type ConnectResultCallback = extern "C" fn(c_int, *const c_char, *mut std::ffi::c_void);

/// Callback type for disconnect requests to Swift
/// Parameters: (context)
/// The session drives disconnects: Rust invokes this so the platform
/// implementation closes the underlying transport (BLE / RFCOMM / mock).
pub type DisconnectRequestCallback = extern "C" fn(*mut std::ffi::c_void);

/// LH0: framed transport events for the host's Adapter Log — the host only
/// sees byte chunks now, so Rust reports what it made of them.
/// Parameters: (kind, text, context) with kind ∈ "rx" | "line" | "dropped".
pub type TransportLogCallback = extern "C" fn(*const c_char, *const c_char, *mut std::ffi::c_void);

/// LH0 fallback when the `>` prompt never arrives — ONE constant, the value
/// Swift's `ResponseProcessor` used (`Constants.Bluetooth.responseTimeout`),
/// deliberately NOT the per-command `timeout_ms` (Link_Handler_Plan finding 10).
pub const FALLBACK_TIMEOUT: Duration = Duration::from_millis(5000);

/// Work for the delivery thread. Byte ingress only ever enqueues; the engine's
/// callbacks run on the worker — never on the host's I/O thread (finding 9).
enum WorkerMsg {
    Event(AccEvent),
    /// A command went out: start the fallback clock for it.
    Armed(String),
    /// The command completed through another path (string API / disconnect).
    Disarm,
    Shutdown,
}

/// The host context pointer, made movable into the worker thread.
#[derive(Clone, Copy)]
struct HostCtx(*mut std::ffi::c_void);
unsafe impl Send for HostCtx {}

/// Internal state for the external platform
struct ExternalPlatformState {
    /// Connection status
    connection_status: ConnectionStatus,
    /// Callback to notify when command completes (for serial processing).
    /// EF1: stored as `Arc` (converted from the trait's `Box` at set time) so
    /// delivery can clone the handle and invoke OUTSIDE the state lock —
    /// these callbacks run arbitrary host code (the whole Swift delivery
    /// chain); holding the platform mutex through them serialized transport
    /// threads behind UI work and made host re-entry a deadlock.
    completion_callback: Option<Arc<dyn Fn(String) + Send + Sync>>,
    /// Callback to deliver OBD data (EF1: Arc, see above)
    data_callback: Option<Arc<dyn Fn(String, crate::link::LinkOutcome) + Send + Sync>>,
    /// Callback for connection status changes (EF1: Arc, see above)
    connection_callback: Option<Arc<dyn Fn(ConnectionStatus, Option<String>) + Send + Sync>>,
    /// Current pending command (if any)
    pending_command: Option<String>,
    /// Pending scan-round callback (set by scan_connectors_round, consumed by receive_discover_result)
    pending_discover_callback: Option<Box<dyn FnOnce(Vec<ConnectorInfo>) + Send>>,
    /// Pending connect callback (set by connect_to, consumed by receive_connect_result)
    pending_connect_callback: Option<Box<dyn FnOnce(ConnectResult) + Send>>,
    /// While sinking (plan M7): the session-provided line consumer. Set by
    /// enter_monitor_mode, cleared by exit_monitor_mode; obd_sink_push
    /// delivers through it. Arc so delivery can invoke it OUTSIDE the state
    /// lock — holding the lock through a consumer at stream rates starves
    /// every other state-lock user (bench round 7: the keepalive thread
    /// wedged at write_raw and the S3 timeout killed the stream).
    sink_on_line: Option<Arc<dyn Fn(String) + Send + Sync>>,
    /// S2: raw transport writer registered by the host (obd_set_raw_writer)
    /// — bypasses the command correlator for monitor-mode traffic.
    raw_writer: Option<RawWriteCallback>,
    /// Test hook: a Rust closure raw writer (mock rigs record the writes and
    /// script STOPPED handshakes). Preferred over the C callback when set.
    /// Arc so write_raw can clone-out-then-call outside the state lock
    /// (same pattern as sink_on_line).
    raw_writer_rust: Option<Arc<dyn Fn(String) + Send + Sync>>,
    /// LH0: host Adapter Log sink for framed rx/line/dropped events.
    transport_log: Option<TransportLogCallback>,
    /// WIRE-LOG: the session logger's wire file — every tlog line and TX
    /// tee is forwarded here (one non-blocking channel send; the logger
    /// thread does the disk I/O).
    wire: Option<std::sync::Arc<crate::session_logger::SessionLogger>>,
    /// LH6: host byte writer (DVI frames over a host-owned link).
    bytes_writer: Option<BytesWriteCallback>,
    /// LH6: consumer of decoded DVI frames (the DviHandler).
    frame_sink: Option<Arc<dyn Fn(crate::link::dvi::codec::DviFrame) + Send + Sync>>,
    /// LH6: command → wire-bytes encoder (the DviHandler); None = ELM text.
    command_encoder: Option<Arc<dyn Fn(&str) -> Result<Option<Vec<u8>>, String> + Send + Sync>>,
    /// LH7: text → typed reply (the ElmHandler); None = built-in ELM 11-bit.
    reply_decoder: Option<crate::platform::ReplyDecoder>,
    /// LH0: `simulateTimeout` / `simulateFatal` — drop the next completed
    /// response and deliver this error instead (one-shot).
    injected_error: Option<String>,
}

/// S2: host-registered raw transport write (line, context).
pub type RawWriteCallback = extern "C" fn(*const c_char, *mut std::ffi::c_void);

/// LH6: host-registered raw BYTE write (bytes, len, context) — a DVI frame
/// over a host-owned link (Swift BLE / classic / EA).
pub type BytesWriteCallback = extern "C" fn(*const u8, usize, *mut std::ffi::c_void);

/// LH5: who owns the wire. The host path (Swift callback) is today's; a
/// Rust transport (TCP / serial) is set by `connect_to` for `usb:`/`wifi:`
/// connectors and cleared on disconnect. Bytes IN always take the same road
/// (`receive_bytes` → accumulator → delivery worker), so the engine never
/// knows which.
enum Writer {
    Host,
    Rust(Arc<dyn crate::transport::byte::ByteTransport>),
}

/// External platform that delegates to Swift/native code
///
/// This platform calls out to Swift when commands need to be sent,
/// and Swift calls back when responses are received.
pub struct ExternalPlatform {
    /// LH5: the active writer (host callback or a Rust transport).
    writer: Mutex<Writer>,
    /// LH5: `Arc<Self>` handle for the Rust transports' sinks (set once by
    /// the session creator via `attach_self`; the mock factory does too).
    self_weak: Mutex<Option<std::sync::Weak<ExternalPlatform>>>,
    /// Fence for Rust-route connects, which open on their own thread: bumped
    /// by `disconnect_from` / `abandon_pending_connect`. A connect (or a link
    /// drop) that finds the epoch moved on belongs to an abandoned attempt —
    /// its transport is closed instead of installed, its drop stays silent.
    connect_epoch: std::sync::atomic::AtomicU64,
    /// Internal state protected by mutex (Arc: the delivery worker shares it)
    state: Arc<Mutex<ExternalPlatformState>>,
    /// LH0: bytes → responses/lines. Its OWN lock, never held while any
    /// callback runs, so the host's I/O thread can always append and return.
    acc: Arc<Mutex<ResponseAccumulator>>,
    worker_tx: Mutex<Option<mpsc::Sender<WorkerMsg>>>,
    /// Test hook: shorten the fallback (worker reads it at arm time).
    fallback: Arc<Mutex<Duration>>,
    /// Callback to send commands to external implementation
    send_callback: OBDSendCallback,
    /// Callback for connector discovery (optional — set during session creation)
    discover_callback: Option<DiscoverConnectorsCallback>,
    /// Callback for connect-to-device (optional — set during session creation)
    connect_callback: Option<ConnectResultCallback>,
    /// Callback for disconnect requests (optional — set during session creation)
    disconnect_callback: Option<DisconnectRequestCallback>,
    /// Context pointer passed to callbacks (typically Swift object pointer)
    context: *mut std::ffi::c_void,
}

// Safety: We ensure thread-safe access through Mutex
// The context pointer is managed by Swift and must remain valid
unsafe impl Send for ExternalPlatform {}
unsafe impl Sync for ExternalPlatform {}

impl Drop for ExternalPlatform {
    fn drop(&mut self) {
        if let Some(tx) = self.worker_tx.lock().unwrap().take() {
            let _ = tx.send(WorkerMsg::Shutdown);
        }
    }
}

/// Shared delivery path: notify the data callback with the result, then the
/// completion callback (which enables the next command). Runs on whichever
/// thread calls it — the worker for the byte path, the caller for the
/// string path (tests / mock responder), exactly as before LH0.
fn deliver_with(
    state: &Arc<Mutex<ExternalPlatformState>>,
    command: String,
    result: Result<String, String>,
) {
    // LH7: the text is decoded into the typed reply BEFORE the engine sees
    // it — by the text dialect's decoder, or the built-in ELM 11-bit decode
    // for rigs without a handler. Outside the state lock (the decoder reads
    // the handler's facts).
    let outcome: crate::link::LinkOutcome = match result {
        Ok(text) => {
            let decoder = state.lock().unwrap().reply_decoder.clone();
            Ok(match decoder {
                Some(d) => d(&command, &text),
                None => crate::link::elm::decode_wire(
                    &command,
                    &text,
                    crate::addressing::Bus::new(crate::addressing::Addressing::Can11),
                ),
            })
        }
        Err(e) => Err(e),
    };
    deliver_typed(state, command, outcome);
}

/// LH7: the typed delivery — data callback, then completion callback.
fn deliver_typed(
    state: &Arc<Mutex<ExternalPlatformState>>,
    command: String,
    outcome: crate::link::LinkOutcome,
) {
    // EF1: clone the handles under the lock, invoke outside it.
    let (data_cb, completion_cb) = {
        let state = state.lock().unwrap();
        (
            state.data_callback.clone(),
            state.completion_callback.clone(),
        )
    };
    if let Some(data_cb) = data_cb {
        data_cb(command.clone(), outcome);
    }
    if let Some(completion_cb) = completion_cb {
        completion_cb(command);
    }
}

fn tlog(state: &Arc<Mutex<ExternalPlatformState>>, ctx: HostCtx, kind: &str, text: &str) {
    let (cb, wire) = {
        let s = state.lock().unwrap();
        (s.transport_log, s.wire.clone())
    };
    if let Some(w) = wire {
        use crate::session_logger::WireDir;
        let dir = match kind {
            "rx" | "line" => WireDir::Rx,
            "tx" => WireDir::Tx,
            "dropped" => WireDir::Drop,
            _ => WireDir::Note,
        };
        w.log_wire(dir, text);
    }
    if let Some(cb) = cb {
        let text: String = text.chars().filter(|c| *c != '\0').collect();
        if let (Ok(k), Ok(t)) = (std::ffi::CString::new(kind), std::ffi::CString::new(text)) {
            cb(k.as_ptr(), t.as_ptr(), ctx.0);
        }
    }
}

/// TX tee — ring + wire file (WIRE-LOG.c: Rust is the ONLY wire logger;
/// the Swift managers' logSent calls are gone, so TX reaches the live
/// monitor through the same callback as RX).
fn twire_tx(state: &Arc<Mutex<ExternalPlatformState>>, ctx: HostCtx, text: &str) {
    tlog(state, ctx, "tx", text);
}

fn head40(text: &str) -> String {
    text.chars().take(40).collect()
}

/// The delivery worker — the Rust replacement for Swift's serial
/// `processingQueue`. FIFO; one outstanding command; the fallback clock is
/// armed by `send_command` and cleared by completion or `Disarm`.
fn worker_main(
    rx: mpsc::Receiver<WorkerMsg>,
    state: Arc<Mutex<ExternalPlatformState>>,
    acc: Arc<Mutex<ResponseAccumulator>>,
    fallback: Arc<Mutex<Duration>>,
    ctx: HostCtx,
) {
    let mut outstanding: Option<(String, Instant)> = None;
    loop {
        let msg = match outstanding {
            Some((_, deadline)) => {
                match rx.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
                    Ok(m) => m,
                    Err(mpsc::RecvTimeoutError::Timeout) => {
                        // No `>` inside the fallback window: clear the
                        // half-formed buffer and fail the command — Swift's
                        // `fallbackWorkItem`, relocated.
                        let (command, _) = outstanding.take().unwrap();
                        let partial = {
                            let mut a = acc.lock().unwrap();
                            let p = a.flush().unwrap_or_default();
                            a.reset();
                            p
                        };
                        let ms = fallback.lock().unwrap().as_millis();
                        tlog(
                            &state,
                            ctx,
                            "dropped",
                            &format!("FALLBACK ({ms}ms) (no > prompt): {}", head40(&partial)),
                        );
                        deliver_with(&state, command, Err("TIMEOUT".to_string()));
                        continue;
                    }
                    Err(mpsc::RecvTimeoutError::Disconnected) => return,
                }
            }
            None => match rx.recv() {
                Ok(m) => m,
                Err(_) => return,
            },
        };
        match msg {
            WorkerMsg::Shutdown => return,
            WorkerMsg::Armed(command) => {
                let d = *fallback.lock().unwrap();
                outstanding = Some((command, Instant::now() + d));
            }
            WorkerMsg::Disarm => outstanding = None,
            WorkerMsg::Event(AccEvent::Dropped(reason)) => tlog(&state, ctx, "dropped", &reason),
            WorkerMsg::Event(AccEvent::Dvi(frame)) => {
                tlog(&state, ctx, "rx", &format!("dvi {frame:?}"));
                let sink = state.lock().unwrap().frame_sink.clone();
                if let Some(sink) = sink {
                    sink(frame);
                }
            }
            WorkerMsg::Event(AccEvent::Line(line)) => {
                tlog(&state, ctx, "line", &line);
                // Clone out, release, invoke — never run the tap under the lock.
                let on_line = state.lock().unwrap().sink_on_line.clone();
                if let Some(on_line) = on_line {
                    on_line(line);
                }
            }
            WorkerMsg::Event(AccEvent::Response(text)) => {
                let injected = state.lock().unwrap().injected_error.take();
                match outstanding.take() {
                    None => {
                        // Nothing waiting (late reply after the fallback, or
                        // unsolicited) — Swift ignored it too.
                        tlog(
                            &state,
                            ctx,
                            "dropped",
                            &format!("no command waiting: {}", head40(&text)),
                        );
                    }
                    Some((command, _)) => {
                        if let Some(code) = injected {
                            tlog(
                                &state,
                                ctx,
                                "dropped",
                                &format!("SIMULATED {code}: {}", head40(&text)),
                            );
                            deliver_with(&state, command, Err(code));
                        } else {
                            tlog(&state, ctx, "rx", &text);
                            deliver_with(&state, command, Ok(text));
                        }
                    }
                }
            }
        }
    }
}

impl ExternalPlatform {
    /// Create a new external platform
    ///
    /// # Parameters
    /// * `send_callback` - Function called when Rust needs to send a command
    /// * `context` - Opaque pointer passed to callbacks (e.g., Swift object)
    ///
    /// # Safety
    /// The context pointer must remain valid for the lifetime of the platform
    pub fn new(send_callback: OBDSendCallback, context: *mut std::ffi::c_void) -> Self {
        Self::new_with_connection_callbacks(send_callback, None, None, None, context)
    }

    /// Create a new external platform with discover/connect/disconnect callbacks
    pub fn new_with_connection_callbacks(
        send_callback: OBDSendCallback,
        discover_callback: Option<DiscoverConnectorsCallback>,
        connect_callback: Option<ConnectResultCallback>,
        disconnect_callback: Option<DisconnectRequestCallback>,
        context: *mut std::ffi::c_void,
    ) -> Self {
        let state = Arc::new(Mutex::new(ExternalPlatformState {
            connection_status: ConnectionStatus::Disconnected,
            completion_callback: None,
            data_callback: None,
            connection_callback: None,
            pending_command: None,
            pending_discover_callback: None,
            pending_connect_callback: None,
            sink_on_line: None,
            raw_writer: None,
            raw_writer_rust: None,
            transport_log: None,
            wire: None,
            injected_error: None,
            bytes_writer: None,
            frame_sink: None,
            command_encoder: None,
            reply_decoder: None,
        }));
        let acc = Arc::new(Mutex::new(ResponseAccumulator::new()));
        let fallback = Arc::new(Mutex::new(FALLBACK_TIMEOUT));
        let (tx, rx) = mpsc::channel::<WorkerMsg>();
        {
            let state = Arc::clone(&state);
            let acc = Arc::clone(&acc);
            let fallback = Arc::clone(&fallback);
            let ctx = HostCtx(context);
            let _ = std::thread::Builder::new()
                .name("obd-ext-delivery".into())
                .spawn(move || worker_main(rx, state, acc, fallback, ctx));
        }
        ExternalPlatform {
            writer: Mutex::new(Writer::Host),
            self_weak: Mutex::new(None),
            connect_epoch: std::sync::atomic::AtomicU64::new(0),
            state,
            acc,
            worker_tx: Mutex::new(Some(tx)),
            fallback,
            send_callback,
            discover_callback,
            connect_callback,
            disconnect_callback,
            context,
        }
    }

    /// LH5: give the platform a weak handle to itself so Rust transports can
    /// feed `receive_bytes` / report a drop without a reference cycle.
    pub fn attach_self(self: &Arc<Self>) {
        *self.self_weak.lock().unwrap() = Some(Arc::downgrade(self));
    }

    /// LH6: register the host's byte writer (obd_set_bytes_writer).
    pub fn set_bytes_writer(&self, cb: Option<BytesWriteCallback>) {
        self.state.lock().unwrap().bytes_writer = cb;
    }

    /// LH6: the DVI handler completes a command it correlated itself.
    pub fn complete_pending(&self, command: &str, outcome: crate::link::LinkOutcome) {
        self.post(WorkerMsg::Disarm);
        deliver_typed(&self.state, command.to_string(), outcome);
    }

    fn rust_writer(&self) -> Option<Arc<dyn crate::transport::byte::ByteTransport>> {
        match &*self.writer.lock().unwrap() {
            Writer::Rust(t) => Some(Arc::clone(t)),
            Writer::Host => None,
        }
    }

    /// LH5: open the Rust transport a prefixed connector names and make it
    /// the writer. Reader → `receive_bytes`; a drop → Disconnected. `epoch`
    /// is the connect epoch the attempt started under: if it has moved on by
    /// the time the link is up, the attempt was abandoned — close, don't install.
    fn connect_rust(&self, connector_id: &str, epoch: u64) -> Result<(), String> {
        use std::sync::atomic::Ordering;
        let weak =
            self.self_weak.lock().unwrap().clone().ok_or_else(|| {
                "platform has no self handle (attach_self not called)".to_string()
            })?;
        let weak_bytes = weak.clone();
        let on_bytes: crate::transport::byte::ByteSink = Arc::new(move |bytes: &[u8]| {
            if let Some(p) = weak_bytes.upgrade() {
                p.receive_bytes(bytes);
            }
        });
        let on_drop: crate::transport::byte::LinkDropSink = Arc::new(move |reason: String| {
            if let Some(p) = weak.upgrade() {
                // A link we closed ourselves (disconnect / abandoned attempt)
                // already had its say — its echo must not touch the writer
                // or the status of whatever attempt owns the platform now.
                if p.connect_epoch.load(Ordering::SeqCst) != epoch {
                    return;
                }
                *p.writer.lock().unwrap() = Writer::Host;
                p.update_connection_status(ConnectionStatus::Disconnected, Some(reason));
            }
        });
        let transport =
            crate::transport::router::open_rust_transport(connector_id, on_bytes, on_drop)?;
        let mut writer = self.writer.lock().unwrap();
        if self.connect_epoch.load(Ordering::SeqCst) != epoch {
            drop(writer);
            transport.close();
            return Err("connect abandoned".to_string());
        }
        self.acc.lock().unwrap().reset();
        *writer = Writer::Rust(transport);
        Ok(())
    }

    fn post(&self, msg: WorkerMsg) {
        if let Some(tx) = self.worker_tx.lock().unwrap().as_ref() {
            let _ = tx.send(msg);
        }
    }

    // MARK: - LH0 byte ingress

    /// Feed raw transport bytes (obd_external_receive_bytes). Append-and-
    /// return: framing happens here under the accumulator's own lock, every
    /// resulting event is handed to the delivery worker. Never blocks on the
    /// engine.
    pub fn receive_bytes(&self, data: &[u8]) {
        let events = self.acc.lock().unwrap().feed_bytes(data);
        for event in events {
            self.post(WorkerMsg::Event(event));
        }
    }

    /// `simulateTimeout` / `simulateFatal`: the next completed response is
    /// dropped and `code` delivered as the command's error.
    pub fn inject_error(&self, code: &str) {
        self.state.lock().unwrap().injected_error = Some(code.to_string());
    }

    /// Register the host's Adapter Log sink (obd_set_transport_log_cb).
    pub fn set_transport_log(&self, cb: TransportLogCallback) {
        self.state.lock().unwrap().transport_log = Some(cb);
    }

    /// WIRE-LOG: install the wire-file sink (trait impl forwards here).
    pub fn install_wire_log(&self, logger: std::sync::Arc<crate::session_logger::SessionLogger>) {
        self.state.lock().unwrap().wire = Some(logger);
    }

    /// WIRE-LOG.c: host-side annotation (`obd_log_transport_note`) — the
    /// things only the host write path can see (BLE canSend stalls, chunk
    /// splits, delay waits). Lands as a NOTE row in the wire file AND in
    /// the live-monitor ring via the same callback.
    pub fn log_transport_note(&self, kind: &str, text: &str) {
        tlog(
            &self.state,
            HostCtx(self.context),
            "note",
            &format!("{kind}: {text}"),
        );
    }

    /// WIRE-LOG.c: flush both log files now (host calls this on
    /// resignActive / will-terminate so a jetsam or quit never eats the
    /// buffered tail).
    pub fn flush_wire_log(&self) {
        let wire = self.state.lock().unwrap().wire.clone();
        if let Some(w) = wire {
            w.flush_sync();
        }
    }

    /// Test hook: shorten the no-prompt fallback.
    pub fn set_fallback_timeout(&self, timeout: Duration) {
        *self.fallback.lock().unwrap() = timeout;
    }

    /// Called by Swift when OBD response data is received
    ///
    /// # Parameters
    /// * `command` - The command that was sent
    /// * `response` - The response data received
    pub fn receive_response(&self, command: String, response: String) {
        self.post(WorkerMsg::Disarm);
        self.deliver(command, Ok(response));
    }

    /// Shared delivery path for receive_response/receive_error: notify the
    /// data callback with the result, then the completion callback (which
    /// enables the next command).
    fn deliver(&self, command: String, result: Result<String, String>) {
        deliver_with(&self.state, command, result);
    }

    // MARK: - Raw transport writes (S2)

    /// Register the host's raw transport writer (obd_set_raw_writer).
    pub fn set_raw_writer(&self, cb: RawWriteCallback) {
        self.state.lock().unwrap().raw_writer = Some(cb);
    }

    /// Test hook: a Rust closure writer (mock rigs record writes and script
    /// the STOPPED handshake). Preferred over the C callback when set.
    pub fn set_raw_writer_rust(&self, f: Box<dyn Fn(String) + Send + Sync>) {
        self.state.lock().unwrap().raw_writer_rust = Some(Arc::from(f));
    }

    // MARK: - Sink (plan M7)

    /// Deliver a captured transport line while sinking. Called by
    /// obd_sink_push (the Swift host routes monitor-mode lines here); forwards
    /// into the session's SinkBuffer via the closure enter_monitor_mode stored.
    /// Copy the invocation out from under the lock is unnecessary — the
    /// closure only pushes into the buffer's own mutex, and the state lock is
    /// never taken inside it.
    pub fn sink_deliver(&self, line: String) {
        // Clone out, release, invoke — NEVER run the consumer under the
        // state lock (see sink_on_line).
        let on_line = self.state.lock().unwrap().sink_on_line.clone();
        if let Some(on_line) = on_line {
            on_line(line);
        }
    }

    /// Called by Swift when an error occurs
    ///
    /// # Parameters
    /// * `command` - The command that failed
    /// * `error` - Error description
    pub fn receive_error(&self, command: String, error: String) {
        self.post(WorkerMsg::Disarm);
        self.deliver(command, Err(error));
    }

    /// Called by Swift when connector discovery completes.
    /// Invokes and consumes the pending discover callback.
    pub fn receive_discover_result(&self, connectors_json: &str) {
        let callback = {
            let mut state = self.state.lock().unwrap();
            state.pending_discover_callback.take()
        };
        if let Some(cb) = callback {
            let connectors: Vec<ConnectorInfo> =
                serde_json::from_str(connectors_json).unwrap_or_default();
            cb(connectors);
        }
    }
    /// Called by Swift when connect-to-device completes.
    /// Invokes and consumes the pending connect callback.
    pub fn receive_connect_result(&self, success: i32, error_message: Option<String>) {
        let callback = {
            let mut state = self.state.lock().unwrap();
            state.pending_connect_callback.take()
        };
        if let Some(cb) = callback {
            let result = match success {
                1 => ConnectResult::Connected,
                0 => ConnectResult::Failed {
                    reason: error_message.unwrap_or_else(|| "Unknown error".to_string()),
                },
                _ => ConnectResult::Cancelled,
            };
            cb(result);
        }
    }

    /// Called by Swift when connection status changes
    pub fn update_connection_status(&self, status: ConnectionStatus, reason: Option<String>) {
        if status == ConnectionStatus::Disconnected {
            self.post(WorkerMsg::Disarm);
            self.acc.lock().unwrap().reset();
        }
        // EF1: mutate under the lock, invoke callbacks outside it (the
        // connection callback's job is teardown that re-enters the platform).
        let (pending, data_cb, completion_cb, conn_cb) = {
            let mut state = self.state.lock().unwrap();
            let old_status = state.connection_status;
            state.connection_status = status;

            // If disconnecting and there's a pending command, fail it — including a
            // cancel mid-connect (old == Connecting), where an in-flight init
            // command would otherwise sit out its full timeout.
            let pending = if status == ConnectionStatus::Disconnected
                && (old_status == ConnectionStatus::Connected
                    || old_status == ConnectionStatus::Connecting)
            {
                state.pending_command.take()
            } else {
                None
            };
            (
                pending,
                state.data_callback.clone(),
                state.completion_callback.clone(),
                state.connection_callback.clone(),
            )
        };
        if let Some(cmd) = pending {
            if let Some(data_cb) = data_cb {
                data_cb(cmd.clone(), Err("DISCONNECTED".to_string()));
            }
            if let Some(completion_cb) = completion_cb {
                completion_cb(cmd);
            }
        }

        // Notify connection callback
        if let Some(conn_cb) = conn_cb {
            conn_cb(status, reason);
        }
    }
}

impl OBDPlatformInterface for ExternalPlatform {
    fn set_wire_log(&self, logger: std::sync::Arc<crate::session_logger::SessionLogger>) {
        self.install_wire_log(logger);
    }

    fn send_command(&self, command: &str, timeout_ms: u32) {
        // Store pending command
        {
            let mut state = self.state.lock().unwrap();
            state.pending_command = Some(command.to_string());
        }
        // Arm the fallback BEFORE the write (Swift: prepareForCommand, then write).
        self.post(WorkerMsg::Armed(command.to_string()));

        // LH6: a binary dialect encodes the command itself (and may answer
        // it locally — e.g. an ATSH that only moves the handler's target).
        let encoder = self.state.lock().unwrap().command_encoder.clone();
        if let Some(enc) = encoder {
            match enc(command) {
                Ok(None) => return, // handled by the handler (it completes it)
                Ok(Some(bytes)) => {
                    if let Err(e) = self.write_bytes(&bytes) {
                        self.receive_error(command.to_string(), format!("WRITE_FAILED: {e}"));
                    }
                    return;
                }
                Err(e) => {
                    self.receive_error(command.to_string(), e);
                    return;
                }
            }
        }

        // LH5: a Rust transport writes the ELM line itself (`\r` terminated);
        // the host path keeps today's callback (Swift appends the terminator).
        if let Some(t) = self.rust_writer() {
            twire_tx(&self.state, HostCtx(self.context), command);
            let mut line = command.as_bytes().to_vec();
            line.push(b'\r');
            if let Err(e) = t.write(&line) {
                self.receive_error(command.to_string(), format!("WRITE_FAILED: {e}"));
            }
            return;
        }
        // Convert command to C string and call external callback
        if let Ok(cmd_cstr) = std::ffi::CString::new(command) {
            twire_tx(&self.state, HostCtx(self.context), command);
            (self.send_callback)(cmd_cstr.as_ptr(), timeout_ms, self.context);
        } else {
            // Command contained null bytes - fail immediately
            self.receive_error(command.to_string(), "ENCODING_ERROR".to_string());
        }
    }

    fn set_completion_callback(&self, callback: Option<Box<dyn Fn(String) + Send + Sync>>) {
        let callback = callback.map(|c| -> Arc<dyn Fn(String) + Send + Sync> { Arc::from(c) });
        let mut state = self.state.lock().unwrap();
        state.completion_callback = callback;
    }

    fn set_data_callback(&self, callback: Option<OBDDataCallback>) {
        let callback = callback.map(
            |c| -> Arc<dyn Fn(String, crate::link::LinkOutcome) + Send + Sync> { Arc::from(c) },
        );
        let mut state = self.state.lock().unwrap();
        state.data_callback = callback;
    }

    fn set_connection_callback(&self, callback: Option<ConnectionCallback>) {
        let callback = callback.map(
            |c| -> Arc<dyn Fn(ConnectionStatus, Option<String>) + Send + Sync> { Arc::from(c) },
        );
        let mut state = self.state.lock().unwrap();
        state.connection_callback = callback;
    }

    fn is_connected(&self) -> bool {
        let state = self.state.lock().unwrap();
        state.connection_status == ConnectionStatus::Connected
    }

    fn connection_status(&self) -> ConnectionStatus {
        let state = self.state.lock().unwrap();
        state.connection_status
    }

    fn note_connection_status(&self, status: ConnectionStatus) {
        self.update_connection_status(status, None);
    }

    fn scan_connectors_round(&self, callback: Box<dyn FnOnce(Vec<ConnectorInfo>) + Send>) {
        // One bounded host round: park the callback, poke the host; the
        // answer arrives via obd_external_discover_result (round complete).
        // Rust's own connectors (USB serial, WiFi) are NOT merged here —
        // the discovery engine owns that (session_manager/discovery).
        if let Some(discover_cb) = self.discover_callback {
            {
                let mut state = self.state.lock().unwrap();
                state.pending_discover_callback = Some(callback);
            }
            discover_cb(std::ptr::null(), self.context);
        } else {
            // No host scanning available — an empty round, immediately.
            callback(Vec::new());
        }
    }

    fn connect_to(&self, connector_id: &str, callback: Box<dyn FnOnce(ConnectResult) + Send>) {
        // LH5: Rust-owned connectors never involve the host.
        if crate::transport::router::route(connector_id) == crate::transport::router::Route::Rust {
            // Open on a thread of our own and answer through the callback,
            // like the host route does: a transport open can block for tens
            // of seconds (BLE connect + pairing), and the caller's bounded
            // wait and cancel checks only work once this has returned.
            let Some(me) = self.self_weak.lock().unwrap().as_ref().and_then(|w| w.upgrade()) else {
                callback(ConnectResult::Failed {
                    reason: "platform has no self handle (attach_self not called)".to_string(),
                });
                return;
            };
            let epoch = self.connect_epoch.load(std::sync::atomic::Ordering::SeqCst);
            let id = connector_id.to_string();
            // The callback rides in a shared slot so a failed spawn can still answer.
            let slot = Arc::new(Mutex::new(Some(callback)));
            let slot_t = Arc::clone(&slot);
            let spawned = std::thread::Builder::new()
                .name("obd-rust-connect".into())
                .spawn(move || {
                    let result = me.connect_rust(&id, epoch);
                    let Some(callback) = slot_t.lock().unwrap().take() else {
                        return;
                    };
                    match result {
                        Ok(()) => {
                            me.update_connection_status(ConnectionStatus::Connected, None);
                            callback(ConnectResult::Connected);
                        }
                        Err(reason) => callback(ConnectResult::Failed { reason }),
                    }
                });
            if let Err(e) = spawned {
                if let Some(callback) = slot.lock().unwrap().take() {
                    callback(ConnectResult::Failed { reason: format!("connect thread: {e}") });
                }
            }
            return;
        }
        if let Some(connect_cb) = self.connect_callback {
            // Store callback — will be consumed when Swift calls obd_external_connect_result
            {
                let mut state = self.state.lock().unwrap();
                state.pending_connect_callback = Some(callback);
            }
            // Tell Swift to connect to this connector ID
            if let Ok(id_cstr) = std::ffi::CString::new(connector_id) {
                // success=0 means "this is a request, not a result"
                // error_message contains the connector_id
                connect_cb(0, id_cstr.as_ptr(), self.context);
            }
        } else {
            callback(ConnectResult::Failed {
                reason: "No connect callback configured".to_string(),
            });
        }
    }

    fn abandon_pending_connect(&self) {
        // Fence first, so neither a late open nor the closed link's drop
        // echo can reach the attempt that comes next.
        let rust = {
            let mut writer = self.writer.lock().unwrap();
            self.connect_epoch.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            std::mem::replace(&mut *writer, Writer::Host)
        };
        if let Writer::Rust(t) = rust {
            t.close();
            // Keep connection_status() truthful, but tell no one.
            self.state.lock().unwrap().connection_status = ConnectionStatus::Disconnected;
        }
    }

    fn disconnect_from(&self) {
        // LH5: a Rust transport closes here; the host path asks Swift.
        let rust = {
            let mut writer = self.writer.lock().unwrap();
            self.connect_epoch.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
            std::mem::replace(&mut *writer, Writer::Host)
        };
        if let Writer::Rust(t) = rust {
            t.close();
            self.update_connection_status(
                ConnectionStatus::Disconnected,
                Some("User disconnected".to_string()),
            );
            return;
        }
        // Session-driven teardown: tell the platform implementation to close
        // the underlying transport first, then publish the status change.
        if let Some(disconnect_cb) = self.disconnect_callback {
            disconnect_cb(self.context);
        }
        self.update_connection_status(
            ConnectionStatus::Disconnected,
            Some("User disconnected".to_string()),
        );
    }

    /// External platforms store the session's line consumer; the HOST drives
    /// the adapter into monitor mode (it owns the transport — the Swift bridge
    /// writes ATH1 + ATMA/STM after obd_sink_start returns) and feeds lines
    /// back via obd_sink_push → sink_deliver.
    fn enter_monitor_mode(
        &self,
        _filter: Option<&str>,
        on_line: Box<dyn Fn(String) + Send + Sync>,
    ) -> Result<(), String> {
        self.state.lock().unwrap().sink_on_line = Some(Arc::from(on_line));
        // LH0: Rust splits the lines now — the host just keeps writing bytes.
        self.acc.lock().unwrap().set_mode(Mode::Line);
        Ok(())
    }

    fn exit_monitor_mode(&self) {
        self.state.lock().unwrap().sink_on_line = None;
        self.acc.lock().unwrap().set_mode(Mode::Prompt);
    }

    fn set_byte_mode(&self, on: bool) {
        self.acc
            .lock()
            .unwrap()
            .set_mode(if on { Mode::Dvi } else { Mode::Prompt });
    }

    /// LH6: bytes to the wire — a Rust transport, else the host's byte
    /// writer (invoked outside the state lock, like write_raw).
    fn write_bytes(&self, bytes: &[u8]) -> Result<(), String> {
        {
            let hex: String = bytes.iter().map(|b| format!("{b:02X} ")).collect();
            twire_tx(&self.state, HostCtx(self.context), hex.trim_end());
        }
        if let Some(t) = self.rust_writer() {
            return t.write(bytes);
        }
        let cb = self.state.lock().unwrap().bytes_writer;
        match cb {
            Some(cb) => {
                cb(bytes.as_ptr(), bytes.len(), self.context);
                Ok(())
            }
            None => Err("no byte writer registered".to_string()),
        }
    }

    fn set_frame_sink(
        &self,
        sink: Option<Arc<dyn Fn(crate::link::dvi::codec::DviFrame) + Send + Sync>>,
    ) {
        self.state.lock().unwrap().frame_sink = sink;
    }

    fn set_command_encoder(
        &self,
        enc: Option<Arc<dyn Fn(&str) -> Result<Option<Vec<u8>>, String> + Send + Sync>>,
    ) {
        self.state.lock().unwrap().command_encoder = enc;
    }

    fn complete_command(&self, command: &str, outcome: crate::link::LinkOutcome) {
        self.complete_pending(command, outcome);
    }

    fn set_reply_decoder(&self, dec: Option<crate::platform::ReplyDecoder>) {
        self.state.lock().unwrap().reply_decoder = dec;
    }

    fn set_frame_timestamps(&self, on: bool) {
        self.acc.lock().unwrap().set_dvi_timestamps(on);
    }

    /// S2: raw write via the registered host callback (or the Rust test
    /// hook). The closure is invoked OUTSIDE the state lock — a mock writer
    /// may synchronously sink_deliver a scripted STOPPED, which re-locks.
    fn write_raw(&self, line: &str) {
        // Clone out, release, invoke — same pattern as sink_deliver (never
        // run a writer under the state lock).
        let (rust_writer, c_writer) = {
            let state = self.state.lock().unwrap();
            (state.raw_writer_rust.clone(), state.raw_writer)
        };
        twire_tx(&self.state, HostCtx(self.context), line);
        if let Some(f) = rust_writer {
            f(line.to_string());
            return;
        }
        // LH5: a Rust transport takes raw lines directly (`\r` terminated).
        if let Some(t) = self.rust_writer() {
            let mut bytes = line.as_bytes().to_vec();
            bytes.push(b'\r');
            let _ = t.write(&bytes);
            return;
        }
        if let Some(cb) = c_writer {
            if let Ok(c) = std::ffi::CString::new(line) {
                cb(c.as_ptr(), self.context);
            }
        }
    }
}

/// Create an ExternalPlatform configured with mock response generation
///
/// This creates a platform that uses MockResponder for response generation,
/// allowing tests to validate the full external platform code path.
///
/// The context is INTENTIONALLY LEAKED (`&'static`): the platform stores a raw
/// pointer to it and dereferences it on every `send_command`, from worker /
/// keepalive / responder threads that can outlive the caller's scope. When the
/// context was a caller-owned Box, a test that returned (or unwound on a failed
/// assertion) while any of those threads was mid-send hit a use-after-free —
/// the load-dependent SIGSEGV/SIGBUS that killed whole suite runs. A leaked
/// context can never dangle; the cost is a few dozen bytes per mock session,
/// which only tests and the debug mock-replay path ever create.
pub fn create_mock_external_platform() -> (
    Arc<ExternalPlatform>,
    &'static crate::mock_responder::MockExternalContext,
) {
    use crate::mock_responder::{mock_send_callback, MockExternalContext, MockResponder};

    let responder = Arc::new(MockResponder::new());

    // Create a placeholder platform first (we'll update context after)
    let placeholder = Arc::new(ExternalPlatform::new(
        mock_send_callback,
        std::ptr::null_mut(), // Will be set after context creation
    ));

    // Leak: the pointer below must stay valid for the process lifetime.
    let context_ptr = Box::into_raw(Box::new(MockExternalContext {
        responder,
        platform: placeholder,
    }));

    // Create the real platform with the (immortal) context pointer
    let platform = Arc::new(ExternalPlatform::new(
        mock_send_callback,
        context_ptr as *mut std::ffi::c_void,
    ));
    unsafe { (*context_ptr).platform = Arc::clone(&platform) };
    platform.attach_self();

    (platform, unsafe { &*context_ptr })
}

/// Create an ExternalPlatform with mock responses and custom delay
pub fn create_mock_external_platform_with_delay(
    delay: std::time::Duration,
) -> (
    Arc<ExternalPlatform>,
    &'static crate::mock_responder::MockExternalContext,
) {
    let (platform, context) = create_mock_external_platform();
    context.responder.set_delay(delay);
    (platform, context)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::sync::mpsc;
    use std::time::Duration;

    // Test send callback that tracks calls
    static SEND_COUNT: AtomicU32 = AtomicU32::new(0);

    extern "C" fn test_send_callback(
        _cmd: *const c_char,
        _timeout: u32,
        _ctx: *mut std::ffi::c_void,
    ) {
        SEND_COUNT.fetch_add(1, Ordering::SeqCst);
    }

    /// LH0 rig: a platform whose data callback records (command, result).
    fn byte_rig() -> (
        Arc<ExternalPlatform>,
        mpsc::Receiver<(String, crate::link::LinkOutcome)>,
    ) {
        extern "C" fn send_cb(_c: *const c_char, _t: u32, _ctx: *mut std::ffi::c_void) {}
        let platform = Arc::new(ExternalPlatform::new(send_cb, std::ptr::null_mut()));
        let (tx, rx) = mpsc::channel();
        platform.set_data_callback(Some(Box::new(move |cmd, res| {
            let _ = tx.send((cmd, res));
        })));
        (platform, rx)
    }

    #[test]
    fn lh0_bytes_complete_the_pending_command_in_order() {
        let (p, rx) = byte_rig();
        p.send_command("010C", 1000);
        p.receive_bytes(b"7E8 04 41");
        p.receive_bytes(b" 0C 1A F8\r\r>");
        let (cmd, res) = rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(cmd, "010C");
        assert_eq!(res.map(|r| r.raw), Ok("7E8 04 41 0C 1A F8".to_string()));
        p.send_command("010D", 1000);
        p.receive_bytes(b"7E8 03 41 0D 00\r>");
        let (cmd, res) = rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(cmd, "010D");
        assert_eq!(res.map(|r| r.raw), Ok("7E8 03 41 0D 00".to_string()));
    }

    #[test]
    fn lh0_no_prompt_fails_with_timeout_and_clears_buffer() {
        let (p, rx) = byte_rig();
        p.set_fallback_timeout(Duration::from_millis(150));
        p.send_command("0100", 1000);
        p.receive_bytes(b"SEARCHING...\r");
        let (cmd, res) = rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(cmd, "0100");
        assert_eq!(res.map(|r| r.raw), Err("TIMEOUT".to_string()));
        p.send_command("ATE0", 1000);
        p.receive_bytes(b"OK\r>");
        let (_, res) = rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(res.map(|r| r.raw), Ok("OK".to_string()));
    }

    #[test]
    fn lh0_string_path_disarms_the_fallback() {
        let (p, rx) = byte_rig();
        p.set_fallback_timeout(Duration::from_millis(100));
        p.send_command("010C", 1000);
        p.receive_response("010C".into(), "7E8 04 41 0C 1A F8".into());
        let _ = rx.recv_timeout(Duration::from_secs(1)).unwrap();
        assert!(
            rx.recv_timeout(Duration::from_millis(400)).is_err(),
            "no TIMEOUT after completion"
        );
    }

    #[test]
    fn lh0_injected_error_replaces_next_response() {
        let (p, rx) = byte_rig();
        p.inject_error("UNABLE TO CONNECT");
        p.send_command("010C", 1000);
        p.receive_bytes(b"7E8 04 41 0C 1A F8\r>");
        let (_, res) = rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(res.map(|r| r.raw), Err("UNABLE TO CONNECT".to_string()));
        p.send_command("010C", 1000);
        p.receive_bytes(b"7E8 04 41 0C 1A F8\r>");
        let (_, res) = rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert!(res.is_ok());
    }

    #[test]
    fn lh0_monitor_mode_routes_lines_then_returns_to_prompt() {
        let (p, rx) = byte_rig();
        let (ltx, lrx) = mpsc::channel::<String>();
        p.enter_monitor_mode(
            None,
            Box::new(move |l| {
                let _ = ltx.send(l);
            }),
        )
        .unwrap();
        p.receive_bytes(b"6A0 04 11 22 33 44\r>6A1 04 55 66");
        p.receive_bytes(b" 77 88\rSTOPPED\r");
        assert_eq!(
            lrx.recv_timeout(Duration::from_secs(1)).unwrap(),
            "6A0 04 11 22 33 44"
        );
        assert_eq!(
            lrx.recv_timeout(Duration::from_secs(1)).unwrap(),
            ">6A1 04 55 66 77 88"
        );
        assert_eq!(lrx.recv_timeout(Duration::from_secs(1)).unwrap(), "STOPPED");
        p.exit_monitor_mode();
        p.send_command("ATE0", 1000);
        p.receive_bytes(b"OK\r>");
        let (cmd, res) = rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(
            (cmd.as_str(), res.map(|r| r.raw)),
            ("ATE0", Ok("OK".to_string()))
        );
        assert!(lrx.try_recv().is_err());
    }

    #[test]
    fn test_external_platform_creation() {
        let platform = ExternalPlatform::new(test_send_callback, std::ptr::null_mut());
        assert!(!platform.is_connected());
        assert_eq!(platform.connection_status(), ConnectionStatus::Disconnected);
    }

    #[test]
    fn test_connection_status_update() {
        let platform = ExternalPlatform::new(test_send_callback, std::ptr::null_mut());

        platform.update_connection_status(ConnectionStatus::Connected, None);
        assert!(platform.is_connected());

        platform.update_connection_status(ConnectionStatus::Disconnected, Some("Test".to_string()));
        assert!(!platform.is_connected());
    }

    #[test]
    fn test_send_command_calls_callback() {
        SEND_COUNT.store(0, Ordering::SeqCst);

        let platform = ExternalPlatform::new(test_send_callback, std::ptr::null_mut());
        platform.send_command("010C", 1000);

        assert_eq!(SEND_COUNT.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn test_mock_external_platform_integration() {
        // Create mock external platform
        let (platform, _context) = create_mock_external_platform();

        // Set up channel to receive responses
        let (tx, rx) = mpsc::channel();
        let tx_clone = tx.clone();

        // Set data callback
        platform.set_data_callback(Some(Box::new(move |cmd, result| {
            tx_clone.send((cmd, result)).unwrap();
        })));

        // Set completion callback
        let (complete_tx, complete_rx) = mpsc::channel();
        platform.set_completion_callback(Some(Box::new(move |cmd| {
            complete_tx.send(cmd).unwrap();
        })));

        // Send a command - this will trigger mock_send_callback
        platform.send_command("ATZ", 1000);

        // Wait for response (mock has 50ms delay by default)
        let (cmd, result) = rx
            .recv_timeout(Duration::from_secs(2))
            .expect("Should receive response");
        assert_eq!(cmd, "ATZ");
        assert!(result.is_ok());
        assert_eq!(result.unwrap().raw, "ELM327 v1.5");

        // Completion callback should also fire
        let completed_cmd = complete_rx
            .recv_timeout(Duration::from_secs(1))
            .expect("Should receive completion");
        assert_eq!(completed_cmd, "ATZ");
    }

    #[test]
    fn test_mock_external_platform_pid_response() {
        let (platform, _context) = create_mock_external_platform();

        let (tx, rx) = mpsc::channel();
        platform.set_data_callback(Some(Box::new(move |cmd, result| {
            tx.send((cmd, result)).unwrap();
        })));

        // Send PID command
        platform.send_command("010C", 1000);

        let (cmd, result) = rx
            .recv_timeout(Duration::from_secs(2))
            .expect("Should receive response");
        assert_eq!(cmd, "010C");
        assert!(result.is_ok());
        // Response should contain mode 41 0C (PID response format)
        let response = result.unwrap().raw;
        assert!(response.contains("41") && response.contains("0C"));
    }

    /// Loopback listener that reports (over the channel) when its one client hangs up.
    #[cfg(test)]
    fn hangup_probe() -> (u16, std::sync::mpsc::Receiver<()>) {
        use std::io::Read;
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let (mut s, _) = listener.accept().unwrap();
            let mut buf = [0u8; 64];
            while !matches!(s.read(&mut buf), Ok(0) | Err(_)) {}
            let _ = tx.send(());
        });
        (port, rx)
    }

    /// A Rust-route connect opens on its own thread: `connect_to` returns at
    /// once and the result arrives through the callback, so the session's
    /// bounded wait and cancel checks cover BLE / USB / WiFi too.
    #[test]
    fn rust_connect_answers_off_the_calling_thread() {
        let (port, _hangup) = hangup_probe();
        let (platform, _ctx) = create_mock_external_platform();
        let (tx, rx) = std::sync::mpsc::channel();
        platform.connect_to(
            &format!("wifi:127.0.0.1:{port}"),
            Box::new(move |r| {
                let _ = tx.send((r, std::thread::current().id()));
            }),
        );
        let (result, thread) = rx.recv_timeout(Duration::from_secs(5)).unwrap();
        assert!(matches!(result, ConnectResult::Connected));
        assert_ne!(thread, std::thread::current().id());
        platform.disconnect_from();
    }

    /// Cancel lands right after the link came up: the link is closed, and
    /// its drop echo publishes NOTHING (a newer attempt may own the platform).
    #[test]
    fn abandoned_connect_closes_the_link_silently() {
        let (port, hangup) = hangup_probe();
        let (platform, _ctx) = create_mock_external_platform();
        let statuses: Arc<Mutex<Vec<ConnectionStatus>>> = Arc::new(Mutex::new(Vec::new()));
        {
            let st = Arc::clone(&statuses);
            platform
                .set_connection_callback(Some(Box::new(move |s, _| st.lock().unwrap().push(s))));
        }
        let (tx, rx) = std::sync::mpsc::channel();
        platform.connect_to(
            &format!("wifi:127.0.0.1:{port}"),
            Box::new(move |r| {
                let _ = tx.send(r);
            }),
        );
        assert!(matches!(
            rx.recv_timeout(Duration::from_secs(5)).unwrap(),
            ConnectResult::Connected
        ));

        platform.abandon_pending_connect();
        hangup
            .recv_timeout(Duration::from_secs(5))
            .expect("abandoned link is closed");
        std::thread::sleep(Duration::from_millis(150)); // let the drop echo run
        assert!(!platform.is_connected());
        assert!(
            !statuses.lock().unwrap().contains(&ConnectionStatus::Disconnected),
            "abandon is silent: {:?}",
            statuses.lock().unwrap()
        );
    }

    /// The open finishes AFTER the attempt was abandoned (epoch moved on):
    /// the late transport is closed, never installed as the writer.
    #[test]
    fn late_connect_under_a_stale_epoch_is_dropped() {
        let (port, hangup) = hangup_probe();
        let (platform, _ctx) = create_mock_external_platform();
        let stale = platform.connect_epoch.load(std::sync::atomic::Ordering::SeqCst);
        platform.abandon_pending_connect();
        let err = platform
            .connect_rust(&format!("wifi:127.0.0.1:{port}"), stale)
            .unwrap_err();
        assert_eq!(err, "connect abandoned");
        hangup
            .recv_timeout(Duration::from_secs(5))
            .expect("late link is closed");
        assert!(platform.rust_writer().is_none());
    }

    /// The session drives transport teardown: disconnect_from must invoke the
    /// registered disconnect callback (Swift closes BLE/RFCOMM there) before
    /// publishing the Disconnected status.
    /// LH5: a Rust-owned TCP transport (loopback fake ELM) rides the SAME
    /// accumulator + delivery worker as the host-fed bytes — request/response,
    /// monitor lines, and a peer close reported as Disconnected.
    #[test]
    fn lh5_loopback_tcp_transport_end_to_end() {
        use std::io::{Read, Write};
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        // Fake ELM: answers 0100 and ATE0 at the prompt, streams two frames on STMA, quits on "Q".
        std::thread::spawn(move || {
            let (mut s, _) = listener.accept().unwrap();
            let mut buf = [0u8; 256];
            loop {
                let n = match s.read(&mut buf) {
                    Ok(0) | Err(_) => return,
                    Ok(n) => n,
                };
                let line = String::from_utf8_lossy(&buf[..n]).trim().to_uppercase();
                match line.as_str() {
                    "0100" => {
                        s.write_all(b"7E8 06 41 00 BE 3F A8 13\r\r>").unwrap();
                    }
                    "ATE0" => {
                        s.write_all(b"OK\r\r>").unwrap();
                    }
                    "STMA" => {
                        s.write_all(b"7E8 03 41 0D 45\r6A0 00 0B 4A\r").unwrap();
                    }
                    "Q" => return,
                    _ => {
                        s.write_all(b"?\r\r>").unwrap();
                    }
                }
            }
        });

        let (platform, _ctx) = create_mock_external_platform();
        let got: Arc<Mutex<Vec<(String, crate::link::LinkOutcome)>>> =
            Arc::new(Mutex::new(Vec::new()));
        {
            let got = Arc::clone(&got);
            platform.set_data_callback(Some(Box::new(move |cmd, res| {
                got.lock().unwrap().push((cmd, res))
            })));
        }
        let statuses: Arc<Mutex<Vec<ConnectionStatus>>> = Arc::new(Mutex::new(Vec::new()));
        {
            let st = Arc::clone(&statuses);
            platform
                .set_connection_callback(Some(Box::new(move |s, _| st.lock().unwrap().push(s))));
        }

        // A scan round with no host callback answers immediately and empty —
        // Rust connectors are the discovery engine's to merge, not the platform's.
        let (dtx, drx) = std::sync::mpsc::channel();
        platform.scan_connectors_round(Box::new(move |list| {
            let _ = dtx.send(list);
        }));
        let list = drx
            .recv_timeout(Duration::from_secs(5))
            .expect("round callback fires once");
        assert!(list.is_empty(), "hostless platform reports an empty round");

        let (ctx_tx, ctx_rx) = std::sync::mpsc::channel();
        platform.connect_to(
            &format!("wifi:127.0.0.1:{port}"),
            Box::new(move |r| {
                let _ = ctx_tx.send(r);
            }),
        );
        assert!(matches!(
            ctx_rx.recv_timeout(Duration::from_secs(5)).unwrap(),
            ConnectResult::Connected
        ));
        assert!(platform.is_connected());
        assert_eq!(
            statuses.lock().unwrap().last(),
            Some(&ConnectionStatus::Connected)
        );

        // Request/response through the Rust writer.
        platform.send_command("0100", 2000);
        let deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < deadline && got.lock().unwrap().is_empty() {
            std::thread::sleep(Duration::from_millis(10));
        }
        let first = got.lock().unwrap().first().cloned().expect("0100 answered");
        assert_eq!(first.0, "0100");
        assert_eq!(
            first.1.as_ref().unwrap().raw.trim(),
            "7E8 06 41 00 BE 3F A8 13"
        );

        // Monitor mode: raw write STMA, lines flow to the sink.
        let lines: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        {
            let l = Arc::clone(&lines);
            platform
                .enter_monitor_mode(None, Box::new(move |line| l.lock().unwrap().push(line)))
                .unwrap();
        }
        platform.write_raw("STMA");
        let deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < deadline && lines.lock().unwrap().len() < 2 {
            std::thread::sleep(Duration::from_millis(10));
        }
        assert_eq!(
            lines.lock().unwrap().clone(),
            vec!["7E8 03 41 0D 45", "6A0 00 0B 4A"]
        );
        platform.exit_monitor_mode();

        // Peer closes → Disconnected surfaces through the same status path.
        platform.write_raw("Q");
        let deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < deadline && platform.is_connected() {
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(!platform.is_connected(), "peer close must disconnect");
        assert_eq!(
            statuses.lock().unwrap().last(),
            Some(&ConnectionStatus::Disconnected)
        );
    }

    #[test]
    fn test_disconnect_from_invokes_disconnect_callback() {
        use std::sync::atomic::{AtomicUsize, Ordering};

        static CALLS: AtomicUsize = AtomicUsize::new(0);
        extern "C" fn disconnect_cb(_context: *mut std::ffi::c_void) {
            CALLS.fetch_add(1, Ordering::SeqCst);
        }
        extern "C" fn send_cb(_c: *const c_char, _t: u32, _ctx: *mut std::ffi::c_void) {}

        let platform = ExternalPlatform::new_with_connection_callbacks(
            send_cb,
            None,
            None,
            Some(disconnect_cb),
            std::ptr::null_mut(),
        );

        let before = CALLS.load(Ordering::SeqCst);
        platform.disconnect_from();
        assert_eq!(
            CALLS.load(Ordering::SeqCst),
            before + 1,
            "transport close requested"
        );
        assert_eq!(platform.connection_status(), ConnectionStatus::Disconnected);
    }

    #[test]
    fn test_enter_monitor_mode_stores_and_delivers_lines() {
        use std::sync::{Arc, Mutex as StdMutex};
        extern "C" fn send_cb(_c: *const c_char, _t: u32, _ctx: *mut std::ffi::c_void) {}
        let platform = ExternalPlatform::new(send_cb, std::ptr::null_mut());

        let captured: Arc<StdMutex<Vec<String>>> = Arc::new(StdMutex::new(Vec::new()));
        let sink = Arc::clone(&captured);

        // Before entering monitor mode: deliveries are dropped.
        platform.sink_deliver("dropped".to_string());
        assert!(captured.lock().unwrap().is_empty());

        platform
            .enter_monitor_mode(None, Box::new(move |line| sink.lock().unwrap().push(line)))
            .expect("external platform supports monitor mode");
        platform.sink_deliver("18DAF110 06 41 00".to_string());
        platform.sink_deliver("7E8 03 41 0C 1A F8".to_string());
        assert_eq!(captured.lock().unwrap().len(), 2);
        assert_eq!(captured.lock().unwrap()[0], "18DAF110 06 41 00");

        // After exit: deliveries are dropped again.
        platform.exit_monitor_mode();
        platform.sink_deliver("late".to_string());
        assert_eq!(captured.lock().unwrap().len(), 2);
    }
}
