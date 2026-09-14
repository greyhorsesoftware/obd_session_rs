//! DR4 (Dataset_Registry_Plan) — the session-config blob.
//!
//! The host hands down the generated `datasets.json` ONCE (raw, verbatim — the vault
//! decrypt stays host-side); this module parses the SESSION SLICE and resolves the
//! attached dataset at identify (VIN's WMI × detected addressing — the same GB2 match
//! rule the app uses). App-slice fields (displayName, capabilities, dtcFormat, …) are
//! ignored here by serde's default unknown-field tolerance.
//!
//! Consumers grow into this: the module walk (ED2) reads `modules`, the `$19` DTC read
//! selects on `dtc_read`, streaming self-selection reads `rules`/`stream_profiles`
//! (retiring `obd_set_stream_tag` eventually). Today the session resolves + logs the
//! attachment; nothing else changes.

use serde::Deserialize;

use crate::addressing::Addressing;

#[derive(Debug, Clone, Deserialize)]
pub struct SessionConfig {
    #[allow(dead_code)]
    pub version: u32,
    pub datasets: Vec<DatasetCfg>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct DatasetCfg {
    pub id: String,
    #[serde(rename = "match")]
    pub match_cfg: Option<MatchCfg>,
    /// FS1: per-dataset identify override for walked modules OUTSIDE the
    /// 7E0 block (Ford: `22 F111` name read). Absent = built-in range rules.
    pub identify: Option<IdentifyCfg>,
    #[serde(default)]
    pub modules: Vec<ModuleCfg>,
    #[serde(rename = "dtcRead")]
    pub dtc_read: Option<String>,
    #[serde(default)]
    pub rules: Vec<RuleCfg>,
    /// Kept as raw JSON until the streaming consumer parses it (own plan).
    #[serde(rename = "streamProfiles")]
    pub stream_profiles: Option<serde_json::Value>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct MatchCfg {
    pub wmis: Vec<String>,
    pub addressing: Option<String>,
    /// FS1b: optional VIN-position constraints (1-based char index →
    /// allowed chars, the vinrules shape) — how Mustang generations
    /// sharing `1FA` split (year char 10). More constraints = more specific.
    #[serde(default)]
    pub positions: std::collections::HashMap<String, Vec<String>>,
}

/// FS1: the identify probe a dataset's walked modules answer
/// (`send` = wire hex, `expect` = positive-echo prefix bytes as hex).
#[derive(Debug, Clone, Deserialize)]
pub struct IdentifyCfg {
    pub send: String,
    pub expect: String,
}

/// One module-table entry. 11-bit tables carry `id` ("7E0", "254"); 29-bit tables carry
/// the decomposed route (`priority`/`format`/`target`/`source`, e.g. 14/DA/41/F1).
#[derive(Debug, Clone, Deserialize)]
pub struct ModuleCfg {
    pub name: String,
    pub id: Option<String>,
    pub priority: Option<String>,
    pub format: Option<String>,
    pub target: Option<String>,
    pub source: Option<String>,
    /// FS1: bus marker — `"ms"` rows are DATA-ONLY until the MS-CAN plan
    /// lands (the walk skips them; absent = HS, today's default).
    pub bus: Option<String>,
}

impl ModuleCfg {
    /// The ATSH-ready physical request header for this module — flat 11-bit id, or the
    /// assembled 29-bit route (which `Bus::target_header` passes through verbatim, GB13).
    pub fn request_header(&self) -> Option<String> {
        if let Some(id) = &self.id {
            return Some(id.clone());
        }
        match (&self.priority, &self.format, &self.target, &self.source) {
            (Some(p), Some(f), Some(t), Some(s)) => Some(format!("{p}{f}{t}{s}")),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct RuleCfg {
    pub wmi: Vec<String>,
    #[serde(default)]
    pub positions: std::collections::HashMap<String, Vec<String>>,
    #[serde(default)]
    pub capabilities: std::collections::HashMap<String, String>,
}

impl SessionConfig {
    pub fn parse(json: &str) -> Result<SessionConfig, String> {
        serde_json::from_str(json).map_err(|e| format!("datasets.json: {e}"))
    }

    /// The GB2 match rule, session-side, extended for FS1b: WMI ∈ match.wmis
    /// (exact 3-char, else the 2-char + trailing-space family key) ∧ declared
    /// addressing matches the detected bus (undeclared = any) ∧ every declared
    /// VIN-position constraint holds. Among the candidates, MOST-SPECIFIC WINS:
    /// most position constraints first (Mustang generations sharing `1FA`),
    /// then exact-WMI over family (a positions-less dataset like Ford Extended
    /// is the natural fallback). A malformed position key fails the dataset —
    /// a partial constraint must never attach.
    pub fn resolve(&self, vin: &str, addressing: &Addressing) -> Option<&DatasetCfg> {
        if vin.len() < 3 {
            return None;
        }
        let vin_upper = vin.to_uppercase();
        let chars: Vec<char> = vin_upper.chars().collect();
        let wmi = &vin_upper[..3];
        let family = format!("{} ", &wmi[..2]);
        let bus = match addressing {
            Addressing::Can11 => "11bit",
            Addressing::Can29 { .. } => "29bit",
        };
        self.datasets
            .iter()
            .filter_map(|d| {
                let m = d.match_cfg.as_ref()?;
                let exact = m.wmis.iter().any(|w| w == wmi);
                if !exact && !m.wmis.iter().any(|w| w == &family) {
                    return None;
                }
                if !m.addressing.as_deref().map_or(true, |req| req == bus) {
                    return None;
                }
                for (key, allowed) in &m.positions {
                    let index: usize = key.parse().ok().filter(|i| *i >= 1)?;
                    let ch = chars.get(index - 1)?;
                    let ch = ch.to_string();
                    if !allowed.iter().any(|a| a.eq_ignore_ascii_case(&ch)) {
                        return None;
                    }
                }
                // Specificity: (position-constraint count, exact-WMI) — max wins.
                Some((m.positions.len(), exact, d))
            })
            .max_by_key(|(npos, exact, _)| (*npos, *exact))
            .map(|(_, _, d)| d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BLOB: &str = r#"{
        "version": 1,
        "datasets": [
            { "id": "Ford Extended", "displayName": "Ford Extended",
              "match": { "wmis": ["1FA", "1ZV"] }, "hasPids": true },
            { "id": "gm_global_a", "displayName": "GM Global A",
              "match": { "wmis": ["1G1", "1GC", "1G "], "addressing": "11bit" },
              "capabilities": { "gmGlobal": "true" }, "dtcRead": "gmlanA9",
              "modules": [ { "name": "ECM", "id": "7E0" }, { "name": "EPB", "id": "254" } ] },
            { "id": "gm_global_b",
              "match": { "wmis": ["1G1", "1GC", "1G "], "addressing": "29bit" },
              "dtcRead": "uds19",
              "modules": [ { "name": "ECM", "priority": "14", "format": "DA",
                             "target": "11", "source": "F1" },
                           { "name": "LCM", "priority": "14", "format": "DA",
                             "target": "41", "source": "F1" } ] }
        ]
    }"#;

    #[test]
    fn parses_session_slice_and_ignores_app_fields() {
        let cfg = SessionConfig::parse(BLOB).unwrap();
        assert_eq!(cfg.datasets.len(), 3);
        assert_eq!(cfg.datasets[1].dtc_read.as_deref(), Some("gmlanA9"));
        assert_eq!(cfg.datasets[2].modules.len(), 2);
    }

    #[test]
    fn resolves_gm_variant_by_addressing() {
        let cfg = SessionConfig::parse(BLOB).unwrap();
        let b = cfg
            .resolve("1GCUYDED4LZ123456", &Addressing::Can29 { tester: 0xF1 })
            .unwrap();
        assert_eq!(b.id, "gm_global_b");
        assert_eq!(b.dtc_read.as_deref(), Some("uds19"));
        assert_eq!(b.modules[1].request_header().as_deref(), Some("14DA41F1"));

        let a = cfg
            .resolve("1GCUYDED4LZ123456", &Addressing::Can11)
            .unwrap();
        assert_eq!(a.id, "gm_global_a");
        assert_eq!(a.modules[1].request_header().as_deref(), Some("254"));
    }

    #[test]
    fn family_key_and_undeclared_addressing() {
        let cfg = SessionConfig::parse(BLOB).unwrap();
        // 1GZ has no exact entry → the "1G " family key routes it.
        assert_eq!(
            cfg.resolve("1GZ1234567F123456", &Addressing::Can11)
                .unwrap()
                .id,
            "gm_global_a"
        );
        // Ford declares no addressing → any bus attaches it.
        assert_eq!(
            cfg.resolve("1ZVSYNTH0CMSTNG50", &Addressing::Can11)
                .unwrap()
                .id,
            "Ford Extended"
        );
        assert_eq!(
            cfg.resolve("1ZVSYNTH0CMSTNG50", &Addressing::Can29 { tester: 0xF1 })
                .unwrap()
                .id,
            "Ford Extended"
        );
        // Unknown make → none.
        assert!(cfg
            .resolve("WVWZZZ3CZWE123456", &Addressing::Can11)
            .is_none());
    }

    const FORD_BLOB: &str = r#"{
        "version": 1,
        "datasets": [
            { "id": "Ford Extended", "match": { "wmis": ["1FA", "1FT", "1ZV", "3FA", "WF0"] },
              "identify": { "send": "22F111", "expect": "62F111" },
              "modules": [ { "name": "PCM", "id": "7E0" },
                           { "name": "SJB", "id": "726", "bus": "ms" } ] },
            { "id": "Ford S197 GT 2011-2014",
              "match": { "wmis": ["1ZV"], "positions": { "10": ["B","C","D","E"] } },
              "identify": { "send": "22F111", "expect": "62F111" },
              "modules": [ { "name": "PCM", "id": "7E0" }, { "name": "ABS", "id": "760" } ] },
            { "id": "Ford S550 2015-2019",
              "match": { "wmis": ["1FA", "3FA", "WF0"],
                         "positions": { "10": ["F","G","H","J","K","L","M","N","P"] } } },
            { "id": "Ford S650",
              "match": { "wmis": ["1FA", "3FA", "WF0"],
                         "positions": { "10": ["R","S","T","V","W","X","Y"] } } }
        ]
    }"#;

    /// FS1b — the Mustang generation decision tree, session-side.
    #[test]
    fn positions_select_mustang_generation() {
        let cfg = SessionConfig::parse(FORD_BLOB).unwrap();
        let can11 = Addressing::Can11;
        // 2012 S197 GT (year char C at position 10).
        assert_eq!(
            cfg.resolve("1ZVBP8CF5C5299999", &can11).unwrap().id,
            "Ford S197 GT 2011-2014"
        );
        // 2008 pre-facelift (year 8) → positions fail → Ford Extended fallback.
        assert_eq!(
            cfg.resolve("1ZVHT82H485123456", &can11).unwrap().id,
            "Ford Extended"
        );
        // 2016 S550 vs 2024 S650, same WMI 1FA — year char decides.
        assert_eq!(
            cfg.resolve("1FA6P8CF0G5299999", &can11).unwrap().id,
            "Ford S550 2015-2019"
        );
        assert_eq!(
            cfg.resolve("1FA6P8CF0R5299999", &can11).unwrap().id,
            "Ford S650"
        );
        // Mexico-built S550 routes the same.
        assert_eq!(
            cfg.resolve("3FA6P8CF0G5299999", &can11).unwrap().id,
            "Ford S550 2015-2019"
        );
        // F-150 → Ford Extended (no generation matches).
        assert_eq!(
            cfg.resolve("1FTFW1ET5DF123456", &can11).unwrap().id,
            "Ford Extended"
        );
    }

    /// FS1 — identify override + bus marker parse.
    #[test]
    fn identify_and_bus_fields_parse() {
        let cfg = SessionConfig::parse(FORD_BLOB).unwrap();
        let ford = &cfg.datasets[0];
        assert_eq!(ford.identify.as_ref().unwrap().send, "22F111");
        assert_eq!(ford.identify.as_ref().unwrap().expect, "62F111");
        assert_eq!(ford.modules[1].bus.as_deref(), Some("ms"));
        assert!(ford.modules[0].bus.is_none());
        // GM blob (no identify/bus) still parses — fields optional.
        let gm = SessionConfig::parse(BLOB).unwrap();
        assert!(gm.datasets[1].identify.is_none());
    }
}
