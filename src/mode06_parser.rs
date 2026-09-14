//! Mode 06 test result parser
//!
//! Decodes Mode 06 response bytes into structured test results.
//! Each test result is a 9-byte block: MID(1) + TID(1) + UASID(1) + TV(2) + MIN(2) + MAX(2)

use serde::Serialize;

/// A single Mode 06 test result
#[derive(Debug, Clone, Serialize)]
pub struct Mode06TestResult {
    /// On-Board Diagnostic Monitor ID
    pub mid: u8,
    /// Standardized Test ID
    pub tid: u8,
    /// Unit and Scaling ID
    pub uasid: u8,
    /// Test value (raw, unscaled)
    pub value: u16,
    /// Minimum test limit (raw)
    pub min_limit: u16,
    /// Maximum test limit (raw)
    pub max_limit: u16,
}

/// Result of parsing a Mode 06 response
#[derive(Debug, Clone, Serialize)]
pub struct Mode06Result {
    /// All test results from the response
    pub test_results: Vec<Mode06TestResult>,
}

/// Parse Mode 06 response data into test results.
///
/// `data_bytes`: bytes after echo stripping (echo is just "46" for Mode 06 test queries).
/// Data is 9-byte blocks: MID(1) TID(1) UASID(1) TestValue(2) MinLimit(2) MaxLimit(2)
pub fn parse_mode06(data_bytes: &[u8]) -> Mode06Result {
    let mut results = Vec::new();

    for chunk in data_bytes.chunks(9) {
        if chunk.len() < 9 {
            break; // Incomplete block — stop
        }

        let mid = chunk[0];
        let tid = chunk[1];
        let uasid = chunk[2];
        let value = u16::from_be_bytes([chunk[3], chunk[4]]);
        let min_limit = u16::from_be_bytes([chunk[5], chunk[6]]);
        let max_limit = u16::from_be_bytes([chunk[7], chunk[8]]);

        results.push(Mode06TestResult {
            mid,
            tid,
            uasid,
            value,
            min_limit,
            max_limit,
        });
    }

    Mode06Result {
        test_results: results,
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
    fn test_mode06_0601_single_block() {
        // "0601" -> after echo "46": 9 bytes = 1 block
        // MID=01, TID=0x91, UASID=0x2F, TV=0x518A, MIN=0x32C8, MAX=0xFFFF
        let mock = load_mock_data();
        let data = mock_lines(&mock["06"]["01"]);
        let parsed = crate::response_parser::parse_response("0601", &data).unwrap();

        let result = parse_mode06(&parsed.all_controllers["7E0"].data_bytes);
        assert_eq!(result.test_results.len(), 1);

        let tr = &result.test_results[0];
        assert_eq!(tr.mid, 0x01);
        assert_eq!(tr.tid, 0x91);
        assert_eq!(tr.uasid, 0x2F);
        assert_eq!(tr.value, 0x518A);
        assert_eq!(tr.min_limit, 0x32C8);
        assert_eq!(tr.max_limit, 0xFFFF);
    }

    #[test]
    fn test_mode06_0631_six_blocks() {
        // "0631" -> 54 data bytes = 6 blocks, all MID=0x31
        let mock = load_mock_data();
        let data = mock_lines(&mock["06"]["31"]);
        let parsed = crate::response_parser::parse_response("0631", &data).unwrap();

        let result = parse_mode06(&parsed.all_controllers["7E0"].data_bytes);
        assert_eq!(result.test_results.len(), 6);

        // All blocks should have MID=0x31
        for tr in &result.test_results {
            assert_eq!(tr.mid, 0x31);
        }

        // First block: TID=0x93, UASID=0x24
        assert_eq!(result.test_results[0].tid, 0x93);
        assert_eq!(result.test_results[0].uasid, 0x24);
        assert_eq!(result.test_results[0].value, 0x0000);
        assert_eq!(result.test_results[0].min_limit, 0x0000);
        assert_eq!(result.test_results[0].max_limit, 0x0032);

        // Last block: TID=0x99, UASID=0x0A
        assert_eq!(result.test_results[5].tid, 0x99);
        assert_eq!(result.test_results[5].value, 0x0000);
        assert_eq!(result.test_results[5].max_limit, 0x03E8);
    }

    #[test]
    fn test_mode06_0602_five_blocks() {
        // "0602" -> 46 total bytes, after echo "46" = 45 bytes = 5 blocks
        let mock = load_mock_data();
        let data = mock_lines(&mock["06"]["02"]);
        let parsed = crate::response_parser::parse_response("0602", &data).unwrap();

        let result = parse_mode06(&parsed.all_controllers["7E0"].data_bytes);
        assert_eq!(result.test_results.len(), 5);

        for tr in &result.test_results {
            assert_eq!(tr.mid, 0x02);
        }

        // First block: TID=0x07, UASID=0x0B
        assert_eq!(result.test_results[0].tid, 0x07);
        assert_eq!(result.test_results[0].uasid, 0x0B);
        assert_eq!(result.test_results[0].value, 0x0273);
        assert_eq!(result.test_results[0].min_limit, 0x0000);
        assert_eq!(result.test_results[0].max_limit, 0x0287);
    }

    #[test]
    fn test_empty_data() {
        let result = parse_mode06(&[]);
        assert!(result.test_results.is_empty());
    }

    #[test]
    fn test_incomplete_block() {
        // Less than 9 bytes — no complete blocks
        let result = parse_mode06(&[0x01, 0x02, 0x03, 0x04]);
        assert!(result.test_results.is_empty());
    }
}
