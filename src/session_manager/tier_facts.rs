//! SL3 — tier availability as data: one facts SNAPSHOT + one PURE function.
//!
//! The ladder used to re-derive eligibility inline every beat from 4–5
//! separate lock takes; facts sampled at different instants went mutually
//! stale (part of why the ARM-COMMIT GATE re-checks everything). Now a beat
//! takes ONE snapshot (`SessionAPIHandle::tier_facts`, single lock
//! acquisition) and asks `eligibility()` — no locks, no I/O, truth-table
//! testable. Availability says "worth trying"; the wire keeps the last word
//! (2C refusals, STPPMA budget, live-state readiness at the arm point).
//!
//! Facts change → `emit_tier_availability` publishes per-tier
//! eligible/blocked (+ typed reason) to the host and the session jsonl —
//! the badge can say WHY a tier is off; the log answers it after the fact.

use super::*;
use crate::session_health::Tier;

/// The slow-changing facts a ladder beat decides from — one struct, one
/// sampling instant. Adapter facts land at connect, vehicle facts at
/// identify, user gates whenever the host flips them.
#[derive(Debug, Clone)]
pub(super) struct TierFacts {
    /// Adapter: the connected chip can run STPPMA periodic messaging.
    pub stn_periodic_capable: bool,
    /// User gates (Settings → Data Acquisition; default ON).
    pub uds_stream_enabled: bool,
    pub stn_periodic_enabled: bool,
    /// Session-transient badge pin (cleared each connection).
    pub user_pinned_poll: bool,
    /// Vehicle: the vinrules `udsPeriodic` tag (None = poll-only vehicle).
    pub stream_tag: Option<String>,
    /// Adapter: the connected link is DVI (OBDX Pro). DVI has no UDS-stream
    /// engine — its stream tier is dvi-periodic — so uds-stream is blocked
    /// regardless of the vehicle profile (Mustang OBDX bench 2026-09-02:
    /// uds-stream engaged then wasted ~5s falling back).
    pub is_dvi_link: bool,
}

/// Why a tier is not worth attempting. The erasure idiom ("disabled = car
/// has no profile") becomes structural: every gate is a filter with a name.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum TierBlock {
    AdapterIncapable,
    NoStreamProfile,
    UserDisabled,
    PinnedPoll,
}

impl TierBlock {
    pub(super) fn as_str(self) -> &'static str {
        match self {
            TierBlock::AdapterIncapable => "adapter_incapable",
            TierBlock::NoStreamProfile => "no_stream_profile",
            TierBlock::UserDisabled => "user_disabled",
            TierBlock::PinnedPoll => "pinned_poll",
        }
    }
}

/// PURE availability policy, ordered high→low. Reason precedence within a
/// tier: the user's explicit pin wins, then the user's settings gate, then
/// the capability/profile fact — the badge should name the user's own action
/// before blaming hardware.
pub(super) fn eligibility(f: &TierFacts) -> [(Tier, Result<(), TierBlock>); 3] {
    let uds = if f.user_pinned_poll {
        Err(TierBlock::PinnedPoll)
    } else if !f.uds_stream_enabled {
        Err(TierBlock::UserDisabled)
    } else if f.is_dvi_link {
        // DVI's stream tier is dvi-periodic; there is no UDS-stream engine.
        Err(TierBlock::AdapterIncapable)
    } else if f.stream_tag.is_none() {
        Err(TierBlock::NoStreamProfile)
    } else {
        Ok(())
    };
    let stn = if f.user_pinned_poll {
        Err(TierBlock::PinnedPoll)
    } else if !f.stn_periodic_enabled {
        Err(TierBlock::UserDisabled)
    } else if !f.stn_periodic_capable {
        Err(TierBlock::AdapterIncapable)
    } else {
        Ok(())
    };
    [
        (Tier::UdsStream, uds),
        (Tier::StnPeriodic, stn),
        (Tier::Poll, Ok(())), // polling is always available
    ]
}

/// The adapter-periodic rung's public name for this link: the ladder and
/// the phase machine use `Tier::StnPeriodic` internally for both engines;
/// everything EMITTED (tier_availability, stream_state plugin, Session
/// Stats, health windows, switch logs) says `dvi-periodic` on a DVI link.
pub(super) fn periodic_tier_for(kind: crate::link::LinkKind) -> crate::session_health::Tier {
    if kind == crate::link::LinkKind::Dvi {
        crate::session_health::Tier::DviPeriodic
    } else {
        crate::session_health::Tier::StnPeriodic
    }
}

impl SessionAPIHandle {
    /// The emitted adapter-periodic tier for the connected link (locks
    /// shared state — do not call with it held).
    pub(super) fn periodic_tier(&self) -> crate::session_health::Tier {
        let kind = {
            let state = self.shared_state.lock().unwrap_or_else(|p| p.into_inner());
            let k = state.host_facts.lock().unwrap().kind;
            k
        };
        periodic_tier_for(kind)
    }

    pub(super) fn periodic_tier_name(&self) -> &'static str {
        self.periodic_tier().as_str()
    }

    /// ONE snapshot of the tier facts — a single shared-state acquisition, so
    /// a beat never mixes facts from different instants.
    pub(super) fn tier_facts(&self) -> TierFacts {
        let state = self.shared_state.lock().unwrap_or_else(|p| p.into_inner());
        let (periodic_capable, is_dvi_link) = {
            let hf = state.host_facts.lock().unwrap();
            (hf.periodic_capable, hf.kind == crate::link::LinkKind::Dvi)
        };
        let facts = TierFacts {
            stn_periodic_capable: periodic_capable,
            uds_stream_enabled: state.uds_stream_enabled,
            stn_periodic_enabled: state.stn_periodic_enabled,
            user_pinned_poll: state
                .user_pinned_poll
                .load(std::sync::atomic::Ordering::SeqCst),
            stream_tag: state.stream_tag.lock().unwrap().clone(),
            is_dvi_link,
        };
        facts
    }

    /// Facts changed → publish per-tier availability (host event + jsonl).
    /// Same observability move as `engage_commit_blocked`.
    pub(super) fn emit_tier_availability(&self) {
        let facts = self.tier_facts();
        let periodic = self.periodic_tier();
        let tiers: Vec<serde_json::Value> = eligibility(&facts)
            .iter()
            .map(|(tier, verdict)| {
                let tier = if *tier == Tier::StnPeriodic {
                    periodic
                } else {
                    *tier
                };
                (tier, verdict)
            })
            .map(|(tier, verdict)| match verdict {
                Ok(()) => serde_json::json!({ "tier": tier.as_str(), "eligible": true }),
                Err(block) => serde_json::json!({
                    "tier": tier.as_str(),
                    "eligible": false,
                    "reason": block.as_str(),
                }),
            })
            .collect();
        let payload = serde_json::json!({ "tiers": tiers });
        let (logger, callback) = {
            let state = self.shared_state.lock().unwrap_or_else(|p| p.into_inner());
            (
                state.logger.clone(),
                Arc::clone(&state.response_callback_shared),
            )
        };
        log_cb!(logger, "tier_availability", &payload.to_string());
        callback(
            serde_json::json!({ "type": "tier_availability", "payload": payload }).to_string(),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn facts(
        capable: bool,
        uds_on: bool,
        stn_on: bool,
        pinned: bool,
        tag: bool,
        is_dvi: bool,
    ) -> TierFacts {
        TierFacts {
            stn_periodic_capable: capable,
            uds_stream_enabled: uds_on,
            stn_periodic_enabled: stn_on,
            user_pinned_poll: pinned,
            stream_tag: tag.then(|| "s197".to_string()),
            is_dvi_link: is_dvi,
        }
    }

    fn verdict(f: &TierFacts, tier: Tier) -> Result<(), TierBlock> {
        eligibility(f)
            .iter()
            .find(|(t, _)| *t == tier)
            .expect("tier present")
            .1
    }

    /// Full truth table over the facts space — the entire availability policy
    /// in one place. Expected values re-derive the policy independently.
    #[test]
    fn eligibility_truth_table() {
        for capable in [false, true] {
            for uds_on in [false, true] {
                for stn_on in [false, true] {
                    for pinned in [false, true] {
                        for tag in [false, true] {
                            for is_dvi in [false, true] {
                                let f = facts(capable, uds_on, stn_on, pinned, tag, is_dvi);

                                let expect_uds = if pinned {
                                    Err(TierBlock::PinnedPoll)
                                } else if !uds_on {
                                    Err(TierBlock::UserDisabled)
                                } else if is_dvi {
                                    Err(TierBlock::AdapterIncapable)
                                } else if !tag {
                                    Err(TierBlock::NoStreamProfile)
                                } else {
                                    Ok(())
                                };
                                let expect_stn = if pinned {
                                    Err(TierBlock::PinnedPoll)
                                } else if !stn_on {
                                    Err(TierBlock::UserDisabled)
                                } else if !capable {
                                    Err(TierBlock::AdapterIncapable)
                                } else {
                                    Ok(())
                                };

                                assert_eq!(verdict(&f, Tier::UdsStream), expect_uds, "{f:?}");
                                assert_eq!(verdict(&f, Tier::StnPeriodic), expect_stn, "{f:?}");
                                assert_eq!(
                                    verdict(&f, Tier::Poll),
                                    Ok(()),
                                    "poll always available"
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    /// Ordering contract: high → low, poll last.
    #[test]
    fn eligibility_is_ordered_high_to_low() {
        let order: Vec<Tier> = eligibility(&facts(true, true, true, false, true, false))
            .iter()
            .map(|(t, _)| *t)
            .collect();
        assert_eq!(order, vec![Tier::UdsStream, Tier::StnPeriodic, Tier::Poll]);
    }
}
