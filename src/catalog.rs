//! Adapter catalog (LH5): the app's `adapters.json`, handed to Rust at session
//! creation (`obd_set_adapter_catalog`) so the facts a connector implies —
//! `supportsPeriodic`, `maxChunk`, `protocol` — resolve in ONE place for
//! host-owned AND Rust-owned connectors alike (a USB/WiFi connector never
//! passes through the Swift bridge's per-connect setters). Matching mirrors
//! `OBDAdapterCatalog.swift`: BLE names `contains` (case-insensitive),
//! classic names `hasPrefix`; `usb`/`wifi` patterns match the connector
//! name the same `contains` way. A connector no entry matches leaves the
//! facts as the host set them.

use serde::Deserialize;

#[derive(Debug, Clone, Deserialize, Default)]
pub struct Catalog {
    #[serde(default)]
    pub adapters: Vec<Adapter>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct Adapter {
    pub name: String,
    #[serde(default, rename = "maxChunk")]
    pub max_chunk: Option<usize>,
    #[serde(default, rename = "supportsPeriodic")]
    pub supports_periodic: Option<bool>,
    /// `"elm"` (default) | `"dvi"`.
    #[serde(default)]
    pub protocol: Option<String>,
    #[serde(default)]
    pub ble: Option<Patterns>,
    #[serde(default)]
    pub classic: Option<Patterns>,
    #[serde(default)]
    pub usb: Option<Patterns>,
    #[serde(default)]
    pub wifi: Option<Patterns>,
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct Patterns {
    #[serde(default, rename = "namePatterns")]
    pub name_patterns: Vec<String>,
}

/// What a matched entry says about the adapter.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CatalogFacts {
    pub adapter_name: String,
    pub supports_periodic: Option<bool>,
    pub max_chunk: Option<usize>,
    pub protocol: Option<String>,
}

impl Catalog {
    pub fn parse(json: &str) -> Result<Self, String> {
        serde_json::from_str(json).map_err(|e| e.to_string())
    }

    /// Resolve the facts for a connector by (name, connector_type).
    pub fn resolve(&self, name: &str, connector_type: &str) -> Option<CatalogFacts> {
        let lname = name.to_lowercase();
        let matches = |p: &Option<Patterns>, prefix_match: bool| -> bool {
            p.as_ref().map_or(false, |p| {
                p.name_patterns.iter().any(|pat| {
                    let lp = pat.to_lowercase();
                    if prefix_match {
                        lname.starts_with(&lp)
                    } else {
                        lname.contains(&lp)
                    }
                })
            })
        };
        let hit = self.adapters.iter().find(|a| match connector_type {
            "ble" => matches(&a.ble, false),
            "classic" => matches(&a.classic, true),
            "usb" => matches(&a.usb, false),
            "wifi" => matches(&a.wifi, false),
            _ => false,
        })?;
        Some(CatalogFacts {
            adapter_name: hit.name.clone(),
            supports_periodic: hit.supports_periodic,
            max_chunk: hit.max_chunk,
            protocol: hit.protocol.clone(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const JSON: &str = r#"{ "adapters": [
        { "name": "OBDLink CX", "supportsPeriodic": true, "ble": { "namePatterns": ["OBDLink CX"] } },
        { "name": "OBDLink MX+", "supportsPeriodic": true, "classic": { "namePatterns": ["OBDLink MX"] } },
        { "name": "Dragy OBD", "maxChunk": 3, "ble": { "namePatterns": ["Dragy"] } },
        { "name": "OBDX Pro VX", "protocol": "dvi", "ble": { "namePatterns": ["OBDX"] }, "usb": { "namePatterns": ["OBDX", "usbmodem"] }, "wifi": { "namePatterns": ["OBDX", "192.168.4.1"] } },
        { "name": "OBDLink MX WiFi", "supportsPeriodic": true, "wifi": { "namePatterns": ["OBDLink MX WiFi", "192.168.0.10"] } }
    ] }"#;

    #[test]
    fn resolves_like_the_swift_catalog() {
        let c = Catalog::parse(JSON).unwrap();
        let cx = c.resolve("OBDLink CX 12345", "ble").unwrap();
        assert_eq!(
            (
                cx.adapter_name.as_str(),
                cx.supports_periodic,
                cx.protocol.as_deref()
            ),
            ("OBDLink CX", Some(true), None)
        );
        assert_eq!(
            c.resolve("OBDLink MX+ 4711", "classic")
                .unwrap()
                .adapter_name,
            "OBDLink MX+"
        );
        assert!(
            c.resolve("Something OBDLink MX", "classic").is_none(),
            "classic = prefix match"
        );
        assert_eq!(c.resolve("dragy obd", "ble").unwrap().max_chunk, Some(3));
        let vx = c
            .resolve("USB serial adapter (cu.usbmodem1234)", "usb")
            .unwrap();
        assert_eq!(vx.protocol.as_deref(), Some("dvi"));
        assert_eq!(
            c.resolve("OBDX Pro", "usb").unwrap().protocol.as_deref(),
            Some("dvi")
        );
        assert!(c.resolve("mock:default", "mock").is_none());
        // WiFi (2026-08-29): each vendor is its own access point — the
        // probe's name/address decide the dialect.
        let wx = c
            .resolve("OBDX Pro (WiFi) (192.168.4.1:23)", "wifi")
            .unwrap();
        assert_eq!(
            (wx.adapter_name.as_str(), wx.protocol.as_deref()),
            ("OBDX Pro VX", Some("dvi"))
        );
        let mx = c
            .resolve("OBDLink MX WiFi (192.168.0.10:35000)", "wifi")
            .unwrap();
        assert_eq!(
            (
                mx.adapter_name.as_str(),
                mx.supports_periodic,
                mx.protocol.as_deref()
            ),
            ("OBDLink MX WiFi", Some(true), None)
        );
    }
}
