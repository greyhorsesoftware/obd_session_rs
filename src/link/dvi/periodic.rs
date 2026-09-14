//! `DviPeriodicEngine` — the OBDX Pro's adapter-periodic acquisition (DV3
//! stream half, 2026-08-29): the tool has 8 periodic-frame slots (manual
//! §3.14.26 — one CAN frame each, ms interval), the direct analogue of the
//! STN chip's `STPPMA`. The session's bracket is unchanged (gate, plan,
//! quiesce, pause/resume, JSON/health); this engine only maps the
//! [`PeriodicEngine`] contract onto the three slot commands and routes the
//! slot replies, which arrive as ordinary RX frames, to the sink.
//!
//! Differences from the STN engine, by construction of DVI: no monitor
//! mode (replies flow alongside request/response — `arm` just opens the
//! decode channel, `break_monitor` closes it), no `STOPPED` handshake, no
//! protocol reopen at teardown (disabling the slots is the whole cleanup),
//! and a hard cap of 8 messages — the plan's tail past 8 stays un-installed
//! (logged) and those pids wait on the paused poll set, like an STN
//! `OUT OF MEMORY` partial install.
//!
//! The engine holds only a `Weak` to the handler: all state lives in
//! `DviHandler::State.periodic` so `on_rx` can route without a second lock.

use std::sync::{Arc, Weak};

use super::handler::DviHandler;
use crate::link::{
    PeriodicEngine, PeriodicError, PeriodicInstall, PeriodicPlan, PeriodicSink, PeriodicStats,
};

/// Slots the tool offers (§3.14.26: "Up to 8 periodic messages").
pub const SLOTS: u8 = 8;

pub struct DviPeriodicEngine {
    handler: Weak<DviHandler>,
}

impl DviPeriodicEngine {
    pub fn new(handler: Weak<DviHandler>) -> Self {
        Self { handler }
    }

    fn handler(&self) -> Option<Arc<DviHandler>> {
        self.handler.upgrade()
    }
}

impl PeriodicEngine for DviPeriodicEngine {
    fn is_live(&self) -> bool {
        self.handler().map_or(false, |h| h.periodic_is_live())
    }

    fn stats(&self) -> Option<PeriodicStats> {
        self.handler().and_then(|h| h.periodic_stats())
    }

    fn start(&self, plan: PeriodicPlan) -> Result<PeriodicInstall, PeriodicError> {
        let h = self.handler().ok_or(PeriodicError::NotStarted)?;
        h.periodic_start(plan)
    }

    fn arm(&self, sink: PeriodicSink) -> Result<(), PeriodicError> {
        let h = self.handler().ok_or(PeriodicError::NotStarted)?;
        h.periodic_arm(sink)
    }

    fn break_monitor(&self) {
        if let Some(h) = self.handler() {
            h.periodic_break();
        }
    }

    fn teardown(&self) {
        if let Some(h) = self.handler() {
            h.periodic_teardown();
        }
    }

    fn abandon(&self) {
        if let Some(h) = self.handler() {
            h.periodic_abandon();
        }
    }
}
