//! BlueZ pairing agent for the native BLE transport (Linux only).
//!
//! Some adapters (OBDLink CX) refuse the notify CCCD write with
//! `Insufficient Authentication` until the link is bonded. The kernel then
//! starts Just Works pairing on its own, and bluetoothd asks the DEFAULT
//! agent to authorize it — with no agent registered it answers "no" within
//! milliseconds and the subscribe hangs (bench 2026-09-19). macOS approves
//! the same pairing silently, which is why the host-owned path never saw it.
//!
//! So: while a connect is in flight we are the default agent, and we approve
//! pairing for THE ONE device the user selected — everything else is
//! rejected. We never call `Device1.Pair()`: connect + subscribe, and the
//! adapter's own refusal starts the pairing. Dropping the agent unregisters
//! it, handing the default back to whoever held it before.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::Duration;

use dbus::blocking::Connection;
use dbus::channel::{MatchingReceiver, Sender};
use dbus::message::MatchRule;
use dbus::{Message, Path};

const AGENT_PATH: &str = "/obd_session/ble_agent";
const AGENT_IFACE: &str = "org.bluez.Agent1";
/// No display, no keyboard → Just Works; bluetoothd asks `RequestAuthorization`.
const CAPABILITY: &str = "NoInputNoOutput";

pub struct PairingAgent {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

/// What the agent answers for one incoming `org.bluez.Agent1` call.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum Verdict {
    Accept,
    Reject,
}

/// Pure policy: approve pairing-time requests for the selected device only.
/// `Cancel` / `Release` carry no device and are always acknowledged.
pub(crate) fn verdict(member: &str, device: Option<&str>, allowed: &str) -> Verdict {
    match member {
        "Cancel" | "Release" => Verdict::Accept,
        "RequestAuthorization" | "RequestConfirmation" | "AuthorizeService" => {
            if device == Some(allowed) {
                Verdict::Accept
            } else {
                Verdict::Reject
            }
        }
        // PIN / passkey entry: we have neither, and no OBD BLE adapter asks.
        _ => Verdict::Reject,
    }
}

impl PairingAgent {
    /// Become the default agent, approving pairing for `device_path` only
    /// (`/org/bluez/hci0/dev_XX_…`). Registration runs on its own thread with
    /// its own D-Bus connection; the result is reported back before returning.
    pub fn register(device_path: &str) -> Result<Self, String> {
        let stop = Arc::new(AtomicBool::new(false));
        let stop_t = Arc::clone(&stop);
        let allowed = device_path.to_string();
        let (tx, rx) = std::sync::mpsc::channel::<Result<(), String>>();
        let thread = std::thread::Builder::new()
            .name("obd-ble-agent".into())
            .spawn(move || serve(allowed, stop_t, tx))
            .map_err(|e| format!("agent thread: {e}"))?;
        match rx.recv_timeout(Duration::from_secs(5)) {
            Ok(Ok(())) => Ok(Self {
                stop,
                thread: Some(thread),
            }),
            Ok(Err(e)) => Err(e),
            Err(_) => {
                stop.store(true, Ordering::SeqCst);
                Err("agent registration timed out".to_string())
            }
        }
    }
}

impl Drop for PairingAgent {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::SeqCst);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn serve(
    allowed: String,
    stop: Arc<AtomicBool>,
    ready: std::sync::mpsc::Sender<Result<(), String>>,
) {
    let conn = match Connection::new_system() {
        Ok(c) => c,
        Err(e) => {
            let _ = ready.send(Err(format!("system bus: {e}")));
            return;
        }
    };
    let mut rule = MatchRule::new_method_call();
    rule.path = Some(Path::from(AGENT_PATH));
    conn.start_receive(
        rule,
        Box::new(move |msg: Message, c: &Connection| {
            let member = msg.member().map(|m| m.to_string()).unwrap_or_default();
            let device: Option<Path> = msg.get1();
            let v = if msg.interface().as_deref() == Some(AGENT_IFACE) {
                verdict(&member, device.as_deref(), &allowed)
            } else {
                Verdict::Reject
            };
            eprintln!(
                "[BLE] agent: {member} {} -> {v:?}",
                device.as_deref().unwrap_or("-")
            );
            let reply = match v {
                Verdict::Accept => msg.method_return(),
                Verdict::Reject => {
                    let text = std::ffi::CString::new("not the selected adapter").unwrap();
                    msg.error(&"org.bluez.Error.Rejected".into(), &text)
                }
            };
            let _ = c.send(reply);
            true
        }),
    );

    let manager = conn.with_proxy("org.bluez", "/org/bluez", Duration::from_secs(5));
    let path = Path::from(AGENT_PATH);
    let registered: Result<(), dbus::Error> = manager
        .method_call(
            "org.bluez.AgentManager1",
            "RegisterAgent",
            (path.clone(), CAPABILITY),
        )
        .and_then(|()| {
            manager.method_call(
                "org.bluez.AgentManager1",
                "RequestDefaultAgent",
                (path.clone(),),
            )
        });
    if let Err(e) = registered {
        let _ = ready.send(Err(format!("register agent: {e}")));
        return;
    }
    let _ = ready.send(Ok(()));

    while !stop.load(Ordering::SeqCst) {
        let _ = conn.process(Duration::from_millis(100));
    }
    let _: Result<(), dbus::Error> =
        manager.method_call("org.bluez.AgentManager1", "UnregisterAgent", (path,));
}

#[cfg(test)]
mod tests {
    use super::*;

    const CX: &str = "/org/bluez/hci0/dev_48_23_35_34_0E_CD";

    #[test]
    fn approves_only_the_selected_device() {
        assert_eq!(
            verdict("RequestAuthorization", Some(CX), CX),
            Verdict::Accept
        );
        assert_eq!(
            verdict("RequestConfirmation", Some(CX), CX),
            Verdict::Accept
        );
        assert_eq!(
            verdict("RequestAuthorization", Some("/org/bluez/hci0/dev_AA"), CX),
            Verdict::Reject
        );
        assert_eq!(verdict("RequestAuthorization", None, CX), Verdict::Reject);
    }

    #[test]
    fn no_pin_or_passkey_entry() {
        assert_eq!(verdict("RequestPinCode", Some(CX), CX), Verdict::Reject);
        assert_eq!(verdict("RequestPasskey", Some(CX), CX), Verdict::Reject);
    }

    #[test]
    fn lifecycle_calls_are_acknowledged() {
        assert_eq!(verdict("Cancel", None, CX), Verdict::Accept);
        assert_eq!(verdict("Release", None, CX), Verdict::Accept);
    }
}
