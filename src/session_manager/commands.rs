//! CMD4b (GM_Commands_Plan) — the `obd_send_control` verb.
//!
//! Swift owns control SEMANTICS (it decoded the vault catalog: type, cpid,
//! payload) and assembles the UDS/GMLAN service bytes — `AE 1A 80 00 …` for a
//! Global A device control, `2F 11 C9 03 00` for a Global B IOCBI. Rust owns
//! bytes-on-wire ADDRESSING: resolve the target module's physical header from
//! the attached dataset (GB13/GB14 rendering, already proven by the ED2 walk),
//! send the service bytes through the serial command processor, return the raw
//! response for Swift to decode (`EE`/`6F` ok, `AE E3`/`7F` NRC).
//!
//! Safety is layered ABOVE this verb (Swift): the safe-subset filter (no
//! latch, no numeric fields, no `$2E` writes), explicit-tap-only, and the
//! authored `flow` warnings. This verb is the one door; SL1's epoch fence
//! guards it for free, and a live stream/periodic refuses (one transport
//! owner — same arbitration `sink_start` respects).

use super::*;

impl SessionAPIHandle {
    /// Send one control exchange to `target` (a dataset module NAME, e.g.
    /// "EPB" / "ECM") carrying the already-assembled `service_hex`
    /// (`"AE1A800000…"`). Returns the raw adapter response, or an `Err`
    /// message the host surfaces verbatim.
    pub fn send_control(&self, target: &str, service_hex: &str) -> Result<String, String> {
        // One transport owner: a live stream/periodic refuses (stop it first),
        // matching sink_start. A control is a wire write; it must not collide
        // with monitor-mode traffic.
        {
            let state = self.shared_state.lock().unwrap();
            if state.stream_state.lock().unwrap().is_some() {
                return Err("stream is live — stop streaming before sending a control".into());
            }
            if state.periodic_state.lock().unwrap().is_some() {
                return Err("adapter periodic acquisition is live — stop it first".into());
            }
        }
        // SL1: a control after terminal intent refuses (the same fence the
        // engage ladder honors — a send racing a disconnect is exactly the
        // lingering-write class).
        if self.terminal_intent_stands() {
            return Err("session is tearing down — control refused".into());
        }

        // Resolve the target module's physical request header from the
        // attached dataset (Swift never decodes the module table — DR boundary).
        let header = {
            let state = self.shared_state.lock().unwrap();
            let attached = state
                .attached_dataset
                .lock()
                .unwrap()
                .clone()
                .ok_or("no dataset attached")?;
            let registry = state.dataset_registry.lock().unwrap();
            let ds = registry
                .as_ref()
                .and_then(|r| r.datasets.iter().find(|d| d.id == attached))
                .ok_or("attached dataset not in registry")?;
            let module = ds
                .modules
                .iter()
                .find(|m| m.name.eq_ignore_ascii_case(target))
                .ok_or_else(|| format!("target '{target}' not in dataset module table"))?;
            module
                .request_header()
                .ok_or_else(|| format!("target '{target}' has no address"))?
        };

        // Quiesce polling for the exchange: pause active subscriptions so a
        // poll response can't land between our ATSH and the control reply, then
        // resume + kick after (FB1 — resuming alone won't revive the chain).
        let sub_mgr = {
            let state = self.shared_state.lock().unwrap();
            Arc::clone(&state.subscription_manager)
        };
        let paused = sub_mgr.lock().unwrap().pause_all_active();

        let bus = self.bus();
        let atsh = format!("ATSH{}", bus.target_header(&header));
        let result = (|| {
            let link = self.link();
            link.request_raw(&atsh, 2000)
                .ok_or("adapter did not accept the controller switch (ATSH)")?;
            link.request_raw(service_hex, 5000)
                .ok_or("no response to the control command")
        })();

        sub_mgr.lock().unwrap().resume_many(&paused);
        for id in &paused {
            self.kick_pipeline_if_idle(*id);
        }
        result.map_err(|e: &str| e.to_string())
    }
}
