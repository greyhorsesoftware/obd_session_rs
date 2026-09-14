//! Trouble code (DTC) parser
//!
//! Decodes Mode 03/07/0A response bytes into standard DTC strings (e.g., "P0101").
//! Each DTC is encoded as 2 bytes per SAE J1979.

use serde::Serialize;

/// Result of parsing a trouble code response
#[derive(Debug, Clone, Serialize)]
pub struct TroubleCodeResult {
    /// Number of DTCs reported
    pub dtc_count: usize,
    /// Decoded DTC strings (e.g., ["P0101", "C0300"])
    pub codes: Vec<String>,
}

/// DTC type prefix from bits 7-6 of the first byte
const DTC_PREFIXES: [char; 4] = ['P', 'C', 'B', 'U'];

/// Decode a single DTC from a 2-byte pair.
///
/// Byte A bits 7-6: type prefix (P/C/B/U)
/// Byte A bits 5-4: second character (0-3)
/// Byte A bits 3-0: third character (0-F)
/// Byte B bits 7-4: fourth character (0-F)
/// Byte B bits 3-0: fifth character (0-F)
fn decode_dtc(byte_a: u8, byte_b: u8) -> Option<String> {
    if byte_a == 0 && byte_b == 0 {
        return None; // padding
    }

    let prefix = DTC_PREFIXES[(byte_a >> 6) as usize];
    let second = (byte_a >> 4) & 0x03;
    let third = byte_a & 0x0F;
    let fourth = byte_b >> 4;
    let fifth = byte_b & 0x0F;

    Some(format!(
        "{}{}{:X}{:X}{:X}",
        prefix, second, third, fourth, fifth
    ))
}

/// Parse DTC response bytes into a list of trouble code strings.
///
/// `data_bytes`: bytes after echo stripping. First byte is DTC count,
/// followed by 2-byte DTC pairs.
pub fn parse_dtcs(data_bytes: &[u8]) -> TroubleCodeResult {
    if data_bytes.is_empty() {
        return TroubleCodeResult {
            dtc_count: 0,
            codes: Vec::new(),
        };
    }

    let reported_count = data_bytes[0] as usize;
    let mut codes = Vec::new();

    // Process byte pairs after the count byte
    let pairs = &data_bytes[1..];
    for chunk in pairs.chunks(2) {
        if chunk.len() < 2 {
            break;
        }
        if let Some(dtc) = decode_dtc(chunk[0], chunk[1]) {
            codes.push(dtc);
        }
    }

    TroubleCodeResult {
        dtc_count: reported_count,
        codes,
    }
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
    fn test_decode_dtc_p_type() {
        assert_eq!(decode_dtc(0x01, 0x01), Some("P0101".to_string()));
        assert_eq!(decode_dtc(0x01, 0x00), Some("P0100".to_string()));
        assert_eq!(decode_dtc(0x02, 0x00), Some("P0200".to_string()));
    }

    #[test]
    fn test_decode_dtc_c_type() {
        // 0x43 = 01_00_0011 → C (01), 0 (00), 3 (0011)
        assert_eq!(decode_dtc(0x43, 0x00), Some("C0300".to_string()));
    }

    #[test]
    fn test_decode_dtc_b_type() {
        // 0x82 = 10_00_0010 → B (10), 0 (00), 2 (0010)
        assert_eq!(decode_dtc(0x82, 0x00), Some("B0200".to_string()));
    }

    #[test]
    fn test_decode_dtc_u_type() {
        // 0xC1 = 11_00_0001 → U (11), 0 (00), 1 (0001)
        assert_eq!(decode_dtc(0xC1, 0x00), Some("U0100".to_string()));
    }

    #[test]
    fn test_decode_dtc_padding() {
        assert_eq!(decode_dtc(0x00, 0x00), None);
    }

    #[test]
    fn test_mode03_from_mock_7e8() {
        // Mode 03 multi-frame from 7E8:
        // After echo strip: count=6, then 6 DTC pairs
        let mock = load_mock_data();
        let data = mock_lines(&mock["03"]["data"]);
        let parsed = crate::response_parser::parse_response("03", &data).unwrap();

        let ctrl = &parsed.all_controllers["7E0"];
        let result = parse_dtcs(&ctrl.data_bytes);

        assert_eq!(result.dtc_count, 6);
        assert_eq!(result.codes.len(), 6);
        assert_eq!(result.codes[0], "P0100");
        assert_eq!(result.codes[1], "P0200");
        assert_eq!(result.codes[2], "P0300");
        assert_eq!(result.codes[3], "C0300");
        assert_eq!(result.codes[4], "B0200");
        assert_eq!(result.codes[5], "U0100");
    }

    #[test]
    fn test_mode07_from_mock_7e8() {
        // Mode 07 (pending DTCs) from 7E8
        let mock = load_mock_data();
        let data = mock_lines(&mock["07"]["data"]);
        let parsed = crate::response_parser::parse_response("07", &data).unwrap();

        let ctrl = &parsed.all_controllers["7E0"];
        let result = parse_dtcs(&ctrl.data_bytes);

        assert_eq!(result.dtc_count, 4);
        assert_eq!(result.codes.len(), 4);
        assert_eq!(result.codes[0], "P0107");
        assert_eq!(result.codes[1], "P0207");
        assert_eq!(result.codes[2], "P0307");
        assert_eq!(result.codes[3], "C0307");
    }

    #[test]
    fn test_mode07_from_mock_7ea() {
        // Mode 07 from 7EA: "7EA 04 47 01 A2 45"
        // After echo (47): count=1, pair (A2, 45)
        let mock = load_mock_data();
        let data = mock_lines(&mock["07"]["data"]);
        let parsed = crate::response_parser::parse_response("07", &data).unwrap();

        let ctrl = &parsed.all_controllers["7E2"];
        let result = parse_dtcs(&ctrl.data_bytes);

        assert_eq!(result.dtc_count, 1);
        assert_eq!(result.codes.len(), 1);
        // 0xA2 = 10_10_0010 → B (10), 2 (10), 2 (0010) = B2
        // 0x45 = 0100_0101 → 4, 5
        assert_eq!(result.codes[0], "B2245");
    }

    #[test]
    fn test_empty_data() {
        let result = parse_dtcs(&[]);
        assert_eq!(result.dtc_count, 0);
        assert!(result.codes.is_empty());
    }

    #[test]
    fn test_count_only_no_pairs() {
        let result = parse_dtcs(&[0x00]);
        assert_eq!(result.dtc_count, 0);
        assert!(result.codes.is_empty());
    }
}
