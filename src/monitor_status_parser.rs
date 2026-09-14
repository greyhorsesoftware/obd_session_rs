//! Monitor status parser (Mode 01 PID 01)
//!
//! Decodes the 4-byte monitor status response into MIL status, DTC count,
//! engine type, and readiness monitor statuses.

use serde::Serialize;

/// Parsed monitor status result
#[derive(Debug, Clone, Serialize)]
pub struct MonitorStatusResult {
    /// Malfunction Indicator Lamp (check engine light) is on
    pub mil_on: bool,
    /// Number of confirmed DTCs
    pub dtc_count: u8,
    /// Engine type: "gasoline" (spark ignition) or "diesel" (compression ignition)
    pub engine_type: String,
    /// Continuous monitors (always applicable)
    pub continuous_monitors: ContinuousMonitors,
    /// Non-continuous monitor availability (which monitors the vehicle supports)
    pub non_continuous_availability: NonContinuousMonitors,
    /// Non-continuous monitor completeness (which monitors have completed)
    pub non_continuous_completeness: NonContinuousMonitors,
}

/// Continuous monitor statuses (available + complete in one struct)
#[derive(Debug, Clone, Serialize)]
pub struct ContinuousMonitors {
    pub misfire: bool,
    pub fuel_system: bool,
    pub component_monitor: bool,
}

/// Non-continuous monitor flags — interpretation depends on engine type
#[derive(Debug, Clone, Serialize)]
pub struct NonContinuousMonitors {
    // Gasoline (spark ignition) monitors
    pub catalyst: Option<bool>,
    pub heated_catalyst: Option<bool>,
    pub evap_system: Option<bool>,
    pub secondary_air: Option<bool>,
    pub oxygen_sensor: Option<bool>,
    pub oxygen_sensor_heater: Option<bool>,
    pub egr_system: Option<bool>,
    // Diesel (compression ignition) monitors
    pub nmhc_catalyst: Option<bool>,
    pub nox_aftertreatment: Option<bool>,
    pub boost_pressure: Option<bool>,
    pub exhaust_sensor: Option<bool>,
    pub pm_filter: Option<bool>,
    pub egr_vvt: Option<bool>,
}

/// Parse gasoline non-continuous monitors from a byte
fn parse_gasoline_monitors(byte: u8) -> NonContinuousMonitors {
    NonContinuousMonitors {
        catalyst: Some(byte & 0x01 != 0),
        heated_catalyst: Some(byte & 0x02 != 0),
        evap_system: Some(byte & 0x04 != 0),
        secondary_air: Some(byte & 0x08 != 0),
        oxygen_sensor: Some(byte & 0x20 != 0),
        oxygen_sensor_heater: Some(byte & 0x40 != 0),
        egr_system: Some(byte & 0x80 != 0),
        // Diesel fields are None for gasoline
        nmhc_catalyst: None,
        nox_aftertreatment: None,
        boost_pressure: None,
        exhaust_sensor: None,
        pm_filter: None,
        egr_vvt: None,
    }
}

/// Parse diesel non-continuous monitors from a byte
fn parse_diesel_monitors(byte: u8) -> NonContinuousMonitors {
    NonContinuousMonitors {
        // Gasoline fields are None for diesel
        catalyst: None,
        heated_catalyst: None,
        evap_system: None,
        secondary_air: None,
        oxygen_sensor: None,
        oxygen_sensor_heater: None,
        egr_system: None,
        // Diesel monitors
        nmhc_catalyst: Some(byte & 0x01 != 0),
        nox_aftertreatment: Some(byte & 0x02 != 0),
        boost_pressure: Some(byte & 0x08 != 0),
        exhaust_sensor: Some(byte & 0x20 != 0),
        pm_filter: Some(byte & 0x40 != 0),
        egr_vvt: Some(byte & 0x80 != 0),
    }
}

/// Parse monitor status from the 4 data bytes of a 0101 response.
///
/// `data_bytes`: 4 bytes [A, B, C, D] after echo stripping
///
/// Byte A: MIL status (bit 7) + DTC count (bits 6-0)
/// Byte B: Monitor availability — continuous (bits 2-0) + engine type (bit 3)
///         Monitor completeness — continuous (bits 6-4)
/// Byte C: Non-continuous monitor availability
/// Byte D: Non-continuous monitor completeness
pub fn parse_monitor_status(data_bytes: &[u8]) -> Option<MonitorStatusResult> {
    if data_bytes.len() < 4 {
        return None;
    }

    let byte_a = data_bytes[0];
    let byte_b = data_bytes[1];
    let byte_c = data_bytes[2];
    let byte_d = data_bytes[3];

    let mil_on = byte_a & 0x80 != 0;
    let dtc_count = byte_a & 0x7F;

    // Bit 3 of byte B: 0 = spark ignition (gasoline), 1 = compression ignition (diesel)
    let is_diesel = byte_b & 0x08 != 0;
    let engine_type = if is_diesel { "diesel" } else { "gasoline" };

    // Continuous monitors — availability from bits 2-0, completeness from bits 6-4
    // Note: for completeness, 0 = complete, 1 = incomplete (inverted logic)
    let continuous_monitors = ContinuousMonitors {
        misfire: byte_b & 0x01 != 0,
        fuel_system: byte_b & 0x02 != 0,
        component_monitor: byte_b & 0x04 != 0,
    };

    // Non-continuous monitors
    let (availability, completeness) = if is_diesel {
        (parse_diesel_monitors(byte_c), parse_diesel_monitors(byte_d))
    } else {
        (
            parse_gasoline_monitors(byte_c),
            parse_gasoline_monitors(byte_d),
        )
    };

    Some(MonitorStatusResult {
        mil_on,
        dtc_count,
        engine_type: engine_type.to_string(),
        continuous_monitors,
        non_continuous_availability: availability,
        non_continuous_completeness: completeness,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn load_mock_data() -> serde_json::Value {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/mock_data/default.json");
        let contents = std::fs::read_to_string(path).expect("failed to read mock data");
        serde_json::from_str(&contents).expect("failed to parse mock data")
    }

    fn mock_lines(arr: &serde_json::Value) -> String {
        arr.as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_str().unwrap())
            .collect::<Vec<_>>()
            .join("\n")
    }

    #[test]
    fn test_monitor_status_from_mock_7e8() {
        // "0101" -> 7E8 data_bytes = [0x86, 0x07, 0xEF, 0x80]
        // Byte A (0x86): MIL = 1 (bit 7), DTC count = 0x06 = 6
        // Byte B (0x07): engine type = gasoline (bit 3 = 0)
        //   continuous: misfire=1, fuel_system=1, component=1
        // Byte C (0xEF): non-continuous availability
        // Byte D (0x80): non-continuous completeness
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["01"]);
        let parsed = crate::response_parser::parse_response("0101", &data).unwrap();

        let ctrl = &parsed.all_controllers["7E0"];
        let result = parse_monitor_status(&ctrl.data_bytes).unwrap();

        assert!(result.mil_on);
        assert_eq!(result.dtc_count, 6);
        assert_eq!(result.engine_type, "gasoline");
        assert!(result.continuous_monitors.misfire);
        assert!(result.continuous_monitors.fuel_system);
        assert!(result.continuous_monitors.component_monitor);
        // Byte C = 0xEF = 1110_1111: all gasoline monitors available
        assert_eq!(result.non_continuous_availability.catalyst, Some(true));
        assert_eq!(result.non_continuous_availability.evap_system, Some(true));
        assert_eq!(result.non_continuous_availability.egr_system, Some(true));
    }

    #[test]
    fn test_monitor_status_from_mock_7e9() {
        // 7E9 data_bytes = [0x81, 0x00, 0x00, 0x00]
        // MIL = 1, DTC count = 1
        // No monitors available
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["01"]);
        let parsed = crate::response_parser::parse_response("0101", &data).unwrap();

        let ctrl = &parsed.all_controllers["7E1"];
        let result = parse_monitor_status(&ctrl.data_bytes).unwrap();

        assert!(result.mil_on);
        assert_eq!(result.dtc_count, 1);
        assert_eq!(result.engine_type, "gasoline");
        assert!(!result.continuous_monitors.misfire);
    }

    #[test]
    fn test_no_mil_no_dtcs() {
        let result = parse_monitor_status(&[0x00, 0x00, 0x00, 0x00]).unwrap();
        assert!(!result.mil_on);
        assert_eq!(result.dtc_count, 0);
    }

    #[test]
    fn test_diesel_engine_type() {
        // Byte B bit 3 = 1 → diesel
        let result = parse_monitor_status(&[0x00, 0x08, 0x01, 0x00]).unwrap();
        assert_eq!(result.engine_type, "diesel");
        assert_eq!(result.non_continuous_availability.nmhc_catalyst, Some(true));
        // Gasoline fields should be None
        assert_eq!(result.non_continuous_availability.catalyst, None);
    }

    #[test]
    fn test_too_short() {
        assert!(parse_monitor_status(&[0x00, 0x00]).is_none());
        assert!(parse_monitor_status(&[]).is_none());
    }
}
