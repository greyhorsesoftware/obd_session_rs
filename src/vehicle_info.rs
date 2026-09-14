//! Vehicle identity information
//!
//! Gathers static vehicle data during connection: VIN, WMI, manufacturer,
//! engine type, ECU list with calibration IDs, and supported PIDs.
//! Results are cached to `{VIN}.json` so repeat connections skip OBD probing.

use serde::{Deserialize, Serialize};

/// Engine type derived from PID 0101 bit 12
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum EngineType {
    Unknown = 0,
    Gasoline = 1,
    Diesel = 2,
}

/// Information about a single ECU (Electronic Control Unit)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ECUInfo {
    /// Controller ID (e.g. "00")
    pub controller_id: String,
    /// ECU name (e.g. "ECM-Engine Control Module")
    pub name: String,
    /// Calibration IDs for this ECU (from PID 0904)
    pub calibration_ids: Vec<String>,
    /// FS2/FS3: the module's HARDWARE part number (`22 F111` ASCII — on real
    /// Fords this is a part number like "BR33-14F094-BB", not a friendly
    /// name; field-confirmed 2026-08-16). The display NAME stays the table's.
    #[serde(default)]
    pub hardware: Option<String>,
    /// FS2 identity cluster (Ford `22 F188`) — module strategy / part-number
    /// family (what the reference tool keys profiles on). Read only on
    /// identify-override datasets; serde-default for old caches.
    #[serde(default)]
    pub strategy: Option<String>,
    /// FS2 identity cluster (`22 F18C`) — module serial number.
    #[serde(default)]
    pub serial: Option<String>,
    /// FS2 identity cluster (`22 F190`) — the VIN THIS MODULE carries. A
    /// used-parts module still carries its donor car's VIN (anti-swap tell).
    #[serde(default)]
    pub module_vin: Option<String>,
}

/// Complete vehicle identity information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleInfo {
    /// Vehicle Identification Number
    pub vin: String,
    /// World Manufacturer Identifier (first 3 chars of VIN)
    pub wmi: String,
    /// Manufacturer name from WMI lookup (None if unknown)
    pub manufacturer: Option<String>,
    /// Engine type (gasoline/diesel/unknown)
    pub engine_type: EngineType,
    /// List of discovered ECUs with their calibration IDs
    pub ecus: Vec<ECUInfo>,
    /// List of supported standard PIDs (e.g. ["010C", "010D"])
    pub supported_pids: Vec<String>,
    /// Human-readable negotiated protocol, e.g. "ISO 15765-4 CAN 29-bit (500k)".
    /// `#[serde(default)]` so a cache file written before this field still deserializes (→ "",
    /// which the app shows as unknown); fresh identify always sets it.
    #[serde(default)]
    pub protocol: String,
    /// FS1c: VIN model-year character (position 10) — the generation
    /// discriminator, cached explicitly so the cache is self-describing.
    #[serde(default)]
    pub year_char: Option<String>,
    /// FS1c: PCM strategy string (Ford `22 F188` ASCII) — PCM family /
    /// GTD hint. Read only on identify-override datasets; None elsewhere.
    #[serde(default)]
    pub pcm_strategy: Option<String>,
    /// FS1c: which census headers answered during the walk (subset of
    /// 730/716/724/706 today; 726 joins when MS-CAN lands) — catches a
    /// bad VIN / swapped PCM across sessions.
    #[serde(default)]
    pub module_census: Vec<String>,
    /// SR1/SR2: the strategy read (Strategy_Read_Plan). Ford identify-override
    /// datasets: the PCM calibration name from the `$23` RAM probe (`GMBM2`
    /// style). gm_global_a: the ECM's `22 F189` value.
    /// None = probe off / failed / non-covered make (fail-closed).
    #[serde(default)]
    pub strategy_probed: Option<String>,
}

/// FFI-compatible vehicle info struct
#[repr(C)]
pub struct VehicleInfoFFI {
    pub vin: *mut std::os::raw::c_char,
    pub wmi: *mut std::os::raw::c_char,
    pub manufacturer: *mut std::os::raw::c_char, // NULL if unknown
    pub engine_type: u8,                         // 0=unknown, 1=gasoline, 2=diesel
    pub ecus: *mut ECUInfoFFI,
    pub ecu_count: u32,
    pub supported_pids: *mut u32,
    pub supported_pid_count: u32,
}

/// FFI-compatible ECU info struct
#[repr(C)]
pub struct ECUInfoFFI {
    pub controller_id: *mut std::os::raw::c_char,
    pub name: *mut std::os::raw::c_char,
    pub calibration_ids: *mut *mut std::os::raw::c_char,
    pub calibration_id_count: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vehicle_info_serialization() {
        let info = VehicleInfo {
            vin: "1FADP3F29JL123456".to_string(),
            wmi: "1FA".to_string(),
            manufacturer: Some("Ford".to_string()),
            engine_type: EngineType::Gasoline,
            ecus: vec![ECUInfo {
                controller_id: "00".to_string(),
                name: "ECM-Engine Control Module".to_string(),
                calibration_ids: vec!["F150_ECM_V2.3".to_string()],
                hardware: None,
                strategy: None,
                serial: None,
                module_vin: None,
            }],
            supported_pids: vec!["010C".to_string(), "010D".to_string()],
            year_char: None,
            pcm_strategy: None,
            module_census: Vec::new(),
            strategy_probed: None,
            protocol: "ISO 15765-4 CAN 11-bit (500k)".to_string(),
        };

        let json = serde_json::to_string(&info).unwrap();
        let deserialized: VehicleInfo = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.vin, "1FADP3F29JL123456");
        assert_eq!(deserialized.manufacturer, Some("Ford".to_string()));
        assert_eq!(deserialized.ecus.len(), 1);
        assert_eq!(deserialized.ecus[0].calibration_ids.len(), 1);
        assert_eq!(deserialized.protocol, "ISO 15765-4 CAN 11-bit (500k)");
    }

    #[test]
    fn deserialize_tolerates_missing_protocol() {
        // A cache file written before `protocol` existed still decodes (→ "").
        let json = r#"{"vin":"X","wmi":"XXX","manufacturer":null,"engine_type":"Unknown",
                       "ecus":[],"supported_pids":[]}"#;
        let info: VehicleInfo = serde_json::from_str(json).unwrap();
        assert_eq!(info.protocol, "");
    }
}
