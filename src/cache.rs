//! PID Discovery Cache
//!
//! Caches PID discovery results (support bitmaps and derived PID lists) to disk
//! so repeat connections to the same vehicle skip redundant OBD probing.
//!
//! Cache files are stored at `{cache_path}/{VIN}.cache` as JSON.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

/// A single cached PID probe result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CacheEntry {
    /// The raw OBD response string for this PID
    pub raw_response: String,
    /// Whether this PID is available/supported
    pub available: bool,
    /// Timestamp when this entry was cached (seconds since epoch)
    pub cached_at: u64,
}

/// Discovery cache for a single vehicle (identified by VIN)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleCache {
    /// VIN this cache belongs to
    pub vin: String,
    /// Cache format version (for future migrations)
    pub version: u32,
    /// When this cache was last updated (seconds since epoch)
    pub updated_at: u64,
    /// Cached PID results keyed by command (e.g. "0100", "22DDBC")
    pub entries: HashMap<String, CacheEntry>,
    /// Derived list of available/supported PIDs
    pub available_pids: Vec<String>,
}

impl VehicleCache {
    /// Create a new empty cache for a VIN
    pub fn new(vin: String) -> Self {
        Self {
            vin,
            version: 1,
            updated_at: now_epoch_secs(),
            entries: HashMap::new(),
            available_pids: Vec::new(),
        }
    }
}

/// Manages reading/writing cache files from disk
pub struct CacheManager {
    /// Base directory for cache files
    cache_dir: PathBuf,
}

impl CacheManager {
    /// Create a new CacheManager with the given cache directory
    pub fn new(cache_path: &str) -> Result<Self, CacheError> {
        let cache_dir = PathBuf::from(cache_path);
        // Create directory if it doesn't exist
        if !cache_dir.exists() {
            fs::create_dir_all(&cache_dir).map_err(|e| CacheError::IoError(e.to_string()))?;
        }
        Ok(Self { cache_dir })
    }

    /// Load cache for a VIN, returns None if no cache exists
    pub fn load(&self, vin: &str) -> Result<Option<VehicleCache>, CacheError> {
        let path = self.cache_file_path(vin);
        if !path.exists() {
            return Ok(None);
        }
        let data = fs::read_to_string(&path).map_err(|e| CacheError::IoError(e.to_string()))?;
        let cache: VehicleCache =
            serde_json::from_str(&data).map_err(|e| CacheError::ParseError(e.to_string()))?;
        Ok(Some(cache))
    }

    /// Save cache for a VIN
    pub fn save(&self, cache: &VehicleCache) -> Result<(), CacheError> {
        let path = self.cache_file_path(&cache.vin);
        let data = serde_json::to_string_pretty(cache)
            .map_err(|e| CacheError::ParseError(e.to_string()))?;
        fs::write(&path, data).map_err(|e| CacheError::IoError(e.to_string()))?;
        Ok(())
    }

    /// Delete cache for a VIN
    pub fn clear(&self, vin: &str) -> Result<(), CacheError> {
        let path = self.cache_file_path(vin);
        if path.exists() {
            fs::remove_file(&path).map_err(|e| CacheError::IoError(e.to_string()))?;
        }
        Ok(())
    }

    /// Delete all cache files
    pub fn clear_all(&self) -> Result<(), CacheError> {
        if self.cache_dir.exists() {
            for entry in
                fs::read_dir(&self.cache_dir).map_err(|e| CacheError::IoError(e.to_string()))?
            {
                let entry = entry.map_err(|e| CacheError::IoError(e.to_string()))?;
                let path = entry.path();
                if path.extension().and_then(|e| e.to_str()) == Some("cache") {
                    fs::remove_file(&path).map_err(|e| CacheError::IoError(e.to_string()))?;
                }
            }
        }
        Ok(())
    }

    /// Check which PIDs from the requested list are already cached
    pub fn get_cached_pids(
        &self,
        vin: &str,
        requested_pids: &[String],
    ) -> Result<(Vec<String>, Vec<String>), CacheError> {
        let cache = self.load(vin)?;
        match cache {
            Some(c) => {
                let mut cached = Vec::new();
                let mut uncached = Vec::new();
                for pid in requested_pids {
                    if c.entries.contains_key(pid) {
                        cached.push(pid.clone());
                    } else {
                        uncached.push(pid.clone());
                    }
                }
                Ok((cached, uncached))
            }
            None => Ok((Vec::new(), requested_pids.to_vec())),
        }
    }

    /// Update cache with new probe results and return merged available PIDs
    pub fn update_cache(
        &self,
        vin: &str,
        new_entries: HashMap<String, CacheEntry>,
        new_available_pids: Vec<String>,
    ) -> Result<VehicleCache, CacheError> {
        let mut cache = self
            .load(vin)?
            .unwrap_or_else(|| VehicleCache::new(vin.to_string()));

        // Merge new entries into existing cache
        for (pid, entry) in new_entries {
            cache.entries.insert(pid, entry);
        }

        // Rebuild available_pids: merge existing + new, deduplicate
        let mut all_available: Vec<String> = cache.available_pids.clone();
        for pid in new_available_pids {
            if !all_available.contains(&pid) {
                all_available.push(pid);
            }
        }
        all_available.sort();
        cache.available_pids = all_available;
        cache.updated_at = now_epoch_secs();

        self.save(&cache)?;
        Ok(cache)
    }

    fn cache_file_path(&self, vin: &str) -> PathBuf {
        // Sanitize VIN for filename safety
        let safe_vin: String = vin
            .chars()
            .map(|c| if c.is_alphanumeric() { c } else { '_' })
            .collect();
        self.cache_dir.join(format!("{}.cache", safe_vin))
    }
}

/// Cache errors
#[derive(Debug)]
pub enum CacheError {
    IoError(String),
    ParseError(String),
}

impl std::fmt::Display for CacheError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CacheError::IoError(msg) => write!(f, "Cache I/O error: {}", msg),
            CacheError::ParseError(msg) => write!(f, "Cache parse error: {}", msg),
        }
    }
}

impl std::error::Error for CacheError {}

fn now_epoch_secs() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cache_roundtrip() {
        let dir = tempfile::tempdir().unwrap();
        let mgr = CacheManager::new(dir.path().to_str().unwrap()).unwrap();

        let mut cache = VehicleCache::new("WVWZZZ3CZWE123456".to_string());
        cache.entries.insert(
            "0100".to_string(),
            CacheEntry {
                raw_response: "41 00 BE 3E B8 13".to_string(),
                available: true,
                cached_at: 1000,
            },
        );
        cache.available_pids = vec!["010C".to_string(), "010D".to_string()];

        mgr.save(&cache).unwrap();
        let loaded = mgr.load("WVWZZZ3CZWE123456").unwrap().unwrap();

        assert_eq!(loaded.vin, "WVWZZZ3CZWE123456");
        assert_eq!(loaded.entries.len(), 1);
        assert_eq!(loaded.available_pids.len(), 2);
        assert!(loaded.entries.contains_key("0100"));
    }

    #[test]
    fn test_cache_miss() {
        let dir = tempfile::tempdir().unwrap();
        let mgr = CacheManager::new(dir.path().to_str().unwrap()).unwrap();
        assert!(mgr.load("NONEXISTENT").unwrap().is_none());
    }

    #[test]
    fn test_get_cached_pids() {
        let dir = tempfile::tempdir().unwrap();
        let mgr = CacheManager::new(dir.path().to_str().unwrap()).unwrap();

        let mut cache = VehicleCache::new("VIN123".to_string());
        cache.entries.insert(
            "0100".to_string(),
            CacheEntry {
                raw_response: "41 00 FF FF FF FF".to_string(),
                available: true,
                cached_at: 1000,
            },
        );
        mgr.save(&cache).unwrap();

        let (cached, uncached) = mgr
            .get_cached_pids("VIN123", &["0100".to_string(), "0120".to_string()])
            .unwrap();
        assert_eq!(cached, vec!["0100"]);
        assert_eq!(uncached, vec!["0120"]);
    }

    #[test]
    fn test_update_cache_merges() {
        let dir = tempfile::tempdir().unwrap();
        let mgr = CacheManager::new(dir.path().to_str().unwrap()).unwrap();

        // Initial cache
        let mut initial_entries = HashMap::new();
        initial_entries.insert(
            "0100".to_string(),
            CacheEntry {
                raw_response: "resp1".to_string(),
                available: true,
                cached_at: 1000,
            },
        );
        mgr.update_cache("VIN1", initial_entries, vec!["010C".to_string()])
            .unwrap();

        // Update with new entries
        let mut new_entries = HashMap::new();
        new_entries.insert(
            "0120".to_string(),
            CacheEntry {
                raw_response: "resp2".to_string(),
                available: true,
                cached_at: 2000,
            },
        );
        let merged = mgr
            .update_cache("VIN1", new_entries, vec!["010D".to_string()])
            .unwrap();

        assert_eq!(merged.entries.len(), 2);
        assert_eq!(merged.available_pids, vec!["010C", "010D"]);
    }

    #[test]
    fn test_clear_cache() {
        let dir = tempfile::tempdir().unwrap();
        let mgr = CacheManager::new(dir.path().to_str().unwrap()).unwrap();

        let cache = VehicleCache::new("VIN_CLEAR".to_string());
        mgr.save(&cache).unwrap();
        assert!(mgr.load("VIN_CLEAR").unwrap().is_some());

        mgr.clear("VIN_CLEAR").unwrap();
        assert!(mgr.load("VIN_CLEAR").unwrap().is_none());
    }
}
