//! WMI (World Manufacturer Identifier) lookup
//!
//! Embeds wmis.csv at compile time and provides a lookup from
//! 3-character WMI code to manufacturer name.

use std::collections::HashMap;
use std::sync::OnceLock;

/// Embedded WMI CSV data
const WMI_CSV: &str = include_str!("data/wmis.csv");

/// Global WMI lookup table (initialized once on first access)
static WMI_MAP: OnceLock<HashMap<&'static str, &'static str>> = OnceLock::new();

fn build_wmi_map() -> HashMap<&'static str, &'static str> {
    let mut map = HashMap::new();
    for line in WMI_CSV.lines().skip(1) {
        // Format: WMI,Manufacturer
        if let Some((wmi, manufacturer)) = line.split_once(',') {
            let wmi = wmi.trim();
            let manufacturer = manufacturer.trim();
            if !wmi.is_empty() && !manufacturer.is_empty() {
                map.insert(wmi, manufacturer);
            }
        }
    }
    map
}

/// Look up manufacturer name from a 3-character WMI code.
/// Returns None if the WMI is not found.
pub fn lookup_manufacturer(wmi: &str) -> Option<&'static str> {
    let map = WMI_MAP.get_or_init(build_wmi_map);
    map.get(wmi).copied()
}

/// Extract WMI (first 3 characters) from a VIN.
/// Returns None if VIN is too short.
pub fn extract_wmi(vin: &str) -> Option<&str> {
    if vin.len() >= 3 {
        Some(&vin[..3])
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lookup_known_wmi() {
        // Ford
        assert!(lookup_manufacturer("1FA").is_some());
    }

    #[test]
    fn test_lookup_unknown_wmi() {
        assert!(lookup_manufacturer("ZZZ").is_none());
    }

    #[test]
    fn test_extract_wmi() {
        assert_eq!(extract_wmi("1FADP3F29JL123456"), Some("1FA"));
        assert_eq!(extract_wmi("AB"), None);
    }

    #[test]
    fn test_map_not_empty() {
        let map = WMI_MAP.get_or_init(build_wmi_map);
        assert!(map.len() > 100, "WMI map should have hundreds of entries");
    }

    #[test]
    fn test_gm_truck_suv_block_present() {
        // GB0: the GM truck/SUV/Korea codes appended 2026-08-15 (display names only —
        // routing/capabilities never read this table). Sample across the regions.
        assert_eq!(lookup_manufacturer("1GN"), Some("Chevrolet USA")); // Tahoe/Suburban
        assert_eq!(lookup_manufacturer("1GK"), Some("GMC USA")); // Yukon
        assert_eq!(lookup_manufacturer("3GC"), Some("Chevrolet Truck Mexico")); // Silverado MX
        assert_eq!(lookup_manufacturer("2GN"), Some("Chevrolet Canada")); // Equinox
        assert_eq!(lookup_manufacturer("KL8"), Some("Chevrolet South Korea")); // Spark
        assert_eq!(lookup_manufacturer("W0V"), Some("Opel Spain"));
    }

    #[test]
    fn test_1ht_not_gm() {
        // 1HT is International Harvester/Navistar in standard registries — deliberately
        // NOT added as GM (GB0 caution).
        assert!(
            lookup_manufacturer("1HT").map_or(true, |m| !m.contains("GM")
                && !m.contains("General Motors")
                && !m.contains("Chevrolet"))
        );
    }
}
