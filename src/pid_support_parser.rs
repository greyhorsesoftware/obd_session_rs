//! PID support bitmap parser
//!
//! Decodes the 4-byte bitmap from PID support queries (0100, 0120, 0140, 0160,
//! 0600, 0620, 0900) into a list of supported PID strings.

use serde::Serialize;

/// Result of parsing a PID support response
#[derive(Debug, Clone, Serialize)]
pub struct PidSupportResult {
    /// OBD mode (e.g., "01", "06", "09")
    pub mode: String,
    /// Base offset of this support range (e.g., 0x00, 0x20, 0x40, 0x60)
    pub base_offset: u16,
    /// List of supported PID strings (e.g., ["0101", "0105", "010C"])
    pub supported_pids: Vec<String>,
}

/// Parse a PID support bitmap into a list of supported PID strings.
///
/// `data_bytes`: 4-byte bitmap from the response (after echo stripping)
/// `mode`: OBD mode string (e.g., "01", "06", "09")
/// `base_offset`: base PID offset (e.g., 0x00 for 0100, 0x20 for 0120)
pub fn parse_supported_pids(data_bytes: &[u8], mode: &str, base_offset: u16) -> PidSupportResult {
    let mut supported = Vec::new();

    for (byte_idx, &byte) in data_bytes.iter().take(4).enumerate() {
        for bit in 0..8u16 {
            if byte & (0x80 >> bit) != 0 {
                let pid = base_offset + (byte_idx as u16) * 8 + bit + 1;
                supported.push(format!("{}{:02X}", mode, pid));
            }
        }
    }

    PidSupportResult {
        mode: mode.to_string(),
        base_offset,
        supported_pids: supported,
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
    fn test_pid_support_0120() {
        // "0120" -> data_bytes [0x80, 0x02, 0x20, 0x01]
        // 0x80 = 10000000 → PID 0x21
        // 0x02 = 00000010 → PID 0x2F
        // 0x20 = 00100000 → PID 0x33
        // 0x01 = 00000001 → PID 0x40
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["20"]);
        let parsed = crate::response_parser::parse_response("0120", &data).unwrap();

        let result = parse_supported_pids(&parsed.all_controllers["7E0"].data_bytes, "01", 0x20);
        assert_eq!(result.mode, "01");
        assert_eq!(result.base_offset, 0x20);
        assert_eq!(result.supported_pids, vec!["0121", "012F", "0133", "0140"]);
    }

    #[test]
    fn test_pid_support_0140() {
        // "0140" -> data_bytes [0x44, 0x00, 0x00, 0x00]
        // 0x44 = 01000100 → bits 6 and 2
        // bit 6: PID = 0x40 + 0*8 + 1 + 1 = 0x42
        // bit 2: PID = 0x40 + 0*8 + 5 + 1 = 0x46
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["40"]);
        let parsed = crate::response_parser::parse_response("0140", &data).unwrap();

        let result = parse_supported_pids(&parsed.all_controllers["7E0"].data_bytes, "01", 0x40);
        assert_eq!(result.supported_pids, vec!["0142", "0146"]);
    }

    #[test]
    fn test_pid_support_0620() {
        // "0620" -> data_bytes [0xC0, 0x00, 0x8C, 0xD9]
        let mock = load_mock_data();
        let data = mock_lines(&mock["06"]["20"]);
        let parsed = crate::response_parser::parse_response("0620", &data).unwrap();

        let result = parse_supported_pids(&parsed.all_controllers["7E0"].data_bytes, "06", 0x20);
        assert_eq!(result.mode, "06");
        // 0xC0 = 11000000 → 0x21, 0x22
        assert!(result.supported_pids.contains(&"0621".to_string()));
        assert!(result.supported_pids.contains(&"0622".to_string()));
    }

    #[test]
    fn test_all_zeros() {
        let result = parse_supported_pids(&[0x00, 0x00, 0x00, 0x00], "01", 0x00);
        assert!(result.supported_pids.is_empty());
    }

    #[test]
    fn test_all_ones() {
        let result = parse_supported_pids(&[0xFF, 0xFF, 0xFF, 0xFF], "01", 0x00);
        assert_eq!(result.supported_pids.len(), 32);
        assert_eq!(result.supported_pids[0], "0101");
        assert_eq!(result.supported_pids[31], "0120");
    }

    #[test]
    fn test_short_data() {
        // Fewer than 4 bytes — only process what's available
        let result = parse_supported_pids(&[0x80], "01", 0x00);
        assert_eq!(result.supported_pids, vec!["0101"]);
    }
}
