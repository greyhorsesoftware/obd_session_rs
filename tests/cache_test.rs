//! Cache integration tests
//!
//! Tests the PID discovery cache lifecycle:
//! 1. First probe — all PIDs uncached
//! 2. Simulate PID discovery (mock OBD responses)
//! 3. Save probe results to cache
//! 4. Second probe — PIDs now cached
//! 5. Incremental updates, multi-VIN isolation, clear

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use obd_session_rs::cache::{CacheEntry, CacheManager};

#[test]
fn test_cache_lifecycle() {
    let tmp = tempfile::tempdir().unwrap();
    let cache_dir = tmp.path().join("cache");
    let mgr = CacheManager::new(cache_dir.to_str().unwrap()).unwrap();
    let vin = "WVWZZZ3CZWE999999";

    // =========================================================================
    // Step 1: First probe — nothing cached yet
    // =========================================================================
    let pids_to_discover = vec![
        "0100".to_string(),
        "0120".to_string(),
        "0140".to_string(),
        "22DDBC".to_string(),
    ];

    let (cached, uncached) = mgr.get_cached_pids(vin, &pids_to_discover).unwrap();
    assert!(cached.is_empty(), "No PIDs should be cached on first probe");
    assert_eq!(uncached.len(), 4, "All PIDs should be uncached");

    // =========================================================================
    // Step 2: Simulate PID discovery — construct probe results
    // =========================================================================
    let mut probe_results = HashMap::new();

    probe_results.insert(
        "0100".to_string(),
        CacheEntry {
            raw_response: "41 00 BE 3E B8 13".to_string(),
            available: true,
            cached_at: 1000,
        },
    );
    probe_results.insert(
        "0120".to_string(),
        CacheEntry {
            raw_response: "41 20 80 02 20 01".to_string(),
            available: true,
            cached_at: 1000,
        },
    );
    probe_results.insert(
        "0140".to_string(),
        CacheEntry {
            raw_response: "41 40 44 00 00 00".to_string(),
            available: true,
            cached_at: 1000,
        },
    );
    // Extended PID — not supported
    probe_results.insert(
        "22DDBC".to_string(),
        CacheEntry {
            raw_response: "".to_string(),
            available: false,
            cached_at: 1000,
        },
    );

    let available_pids = vec![
        "0101".to_string(),
        "0103".to_string(),
        "0104".to_string(),
        "0105".to_string(),
        "0106".to_string(),
        "0107".to_string(),
        "010C".to_string(),
        "010D".to_string(),
        "010F".to_string(),
        "0110".to_string(),
        "0121".to_string(),
        "012F".to_string(),
        "0142".to_string(),
        "0146".to_string(),
    ];

    // =========================================================================
    // Step 3: Save to cache
    // =========================================================================
    let saved = mgr
        .update_cache(vin, probe_results, available_pids.clone())
        .unwrap();

    assert_eq!(saved.vin, vin);
    assert_eq!(saved.entries.len(), 4);
    assert_eq!(saved.available_pids.len(), 14);

    // Verify file on disk
    let cache_file = cache_dir.join("WVWZZZ3CZWE999999.cache");
    assert!(cache_file.exists(), "Cache file should exist on disk");

    // =========================================================================
    // Step 4: Second probe — all cached
    // =========================================================================
    let (cached, uncached) = mgr.get_cached_pids(vin, &pids_to_discover).unwrap();
    assert_eq!(cached.len(), 4, "All PIDs should be cached now");
    assert!(uncached.is_empty());

    // Verify available vs unavailable
    let loaded = mgr.load(vin).unwrap().unwrap();
    assert!(loaded.entries["0100"].available);
    assert!(loaded.entries["0120"].available);
    assert!(loaded.entries["0140"].available);
    assert!(!loaded.entries["22DDBC"].available);
    assert_eq!(loaded.entries["0100"].raw_response, "41 00 BE 3E B8 13");
    assert!(loaded.available_pids.contains(&"010C".to_string()));
    assert!(loaded.available_pids.contains(&"0142".to_string()));

    // =========================================================================
    // Step 5: Mixed probe — some cached, some new
    // =========================================================================
    let mixed = vec![
        "0100".to_string(),   // cached (available)
        "0160".to_string(),   // new
        "22DDBC".to_string(), // cached (unavailable)
        "22AABB".to_string(), // new
    ];
    let (cached, uncached) = mgr.get_cached_pids(vin, &mixed).unwrap();
    assert_eq!(cached, vec!["0100", "22DDBC"]);
    assert_eq!(uncached, vec!["0160", "22AABB"]);

    // =========================================================================
    // Step 6: Incremental update — merge new results
    // =========================================================================
    let mut new_results = HashMap::new();
    new_results.insert(
        "0160".to_string(),
        CacheEntry {
            raw_response: "41 60 00 00 00 00".to_string(),
            available: true,
            cached_at: 2000,
        },
    );
    new_results.insert(
        "22AABB".to_string(),
        CacheEntry {
            raw_response: "".to_string(),
            available: false,
            cached_at: 2000,
        },
    );

    let updated = mgr
        .update_cache(vin, new_results, vec!["0165".to_string()])
        .unwrap();

    assert_eq!(updated.entries.len(), 6, "Should have 6 entries total");
    assert_eq!(updated.available_pids.len(), 15);
    assert!(updated.available_pids.contains(&"0165".to_string()));
    // Original data preserved
    assert_eq!(updated.entries["0100"].raw_response, "41 00 BE 3E B8 13");

    // =========================================================================
    // Step 7: Different VIN gets separate cache
    // =========================================================================
    let vin2 = "1HGBH41JXMN109186";
    let (cached, uncached) = mgr.get_cached_pids(vin2, &pids_to_discover).unwrap();
    assert!(cached.is_empty(), "Different VIN should have no cache");
    assert_eq!(uncached.len(), 4);

    // =========================================================================
    // Step 8: Clear single VIN
    // =========================================================================
    mgr.clear(vin).unwrap();
    assert!(mgr.load(vin).unwrap().is_none());
    let (cached, _) = mgr.get_cached_pids(vin, &pids_to_discover).unwrap();
    assert!(cached.is_empty());

    // =========================================================================
    // Step 9: Clear all
    // =========================================================================
    let mut vin2_results = HashMap::new();
    vin2_results.insert(
        "0100".to_string(),
        CacheEntry {
            raw_response: "41 00 FF FF FF FF".to_string(),
            available: true,
            cached_at: 3000,
        },
    );
    mgr.update_cache(vin2, vin2_results, vec![]).unwrap();
    assert!(mgr.load(vin2).unwrap().is_some());

    mgr.clear_all().unwrap();
    assert!(mgr.load(vin2).unwrap().is_none());
}

/// End-to-end test: create a mock OBD session, discover PIDs through it,
/// then save the real responses to cache and verify on reload.
#[test]
fn test_cache_with_mock_session_discovery() {
    use obd_session_rs::external_platform::create_mock_external_platform;
    use obd_session_rs::platform;
    use obd_session_rs::{OBDSessionConfig, OBDSessionManager};

    let tmp = tempfile::tempdir().unwrap();
    let cache_dir = tmp.path().join("cache");
    let vin = "WVWZZZ3CZWE123456";

    // Collect OBD responses from mock session callback
    let responses: Arc<Mutex<HashMap<String, String>>> = Arc::new(Mutex::new(HashMap::new()));
    let responses_clone = responses.clone();

    let (mock_platform, _context) = create_mock_external_platform();

    let mut config = OBDSessionConfig::default();
    config.cache_path = Some(cache_dir.to_str().unwrap().to_string());

    let session = OBDSessionManager::new(
        mock_platform as Arc<dyn platform::OBDPlatformInterface>,
        config,
        move |json_response: String| {
            if let Ok(value) = serde_json::from_str::<serde_json::Value>(&json_response) {
                if value.get("type").and_then(|t| t.as_str()) == Some("obd_data") {
                    if let Some(payload) = value.get("payload") {
                        let cmd = payload
                            .get("command")
                            .and_then(|c| c.as_str())
                            .unwrap_or("")
                            .to_string();
                        let data = payload
                            .get("data")
                            .and_then(|d| d.as_str())
                            .unwrap_or("")
                            .to_string();
                        if !cmd.is_empty() && !data.is_empty() && !cmd.starts_with("AT") {
                            if let Ok(mut map) = responses_clone.lock() {
                                map.insert(cmd, data);
                            }
                        }
                    }
                }
            }
        },
    )
    .expect("Failed to create session");

    let api = session.api_handle();

    // Send PID support bitmap commands as run-once subscription
    let discovery_pids = vec!["0100".to_string(), "0120".to_string(), "0140".to_string()];

    let mut run_counts = HashMap::new();
    for pid in &discovery_pids {
        run_counts.insert(pid.clone(), Some(1u32));
    }

    let sub_id = api
        .create_subscription_with_run_counts(None, discovery_pids.clone(), None, Some(run_counts))
        .expect("Failed to create subscription");

    api.start_subscription(sub_id)
        .expect("Failed to start subscription");

    // Wait for all 3 responses
    let mut attempts = 0;
    loop {
        std::thread::sleep(Duration::from_millis(50));
        let count = responses.lock().unwrap().len();
        if count >= 3 {
            break;
        }
        attempts += 1;
        assert!(
            attempts <= 100,
            "Timed out waiting for discovery responses (got {}/3)",
            count
        );
    }

    let received = responses.lock().unwrap().clone();
    println!(
        "Received {} discovery responses from mock session",
        received.len()
    );
    for (pid, resp) in &received {
        println!("  {} → {}", pid, resp);
    }

    // Save responses to cache
    let mgr = CacheManager::new(cache_dir.to_str().unwrap()).unwrap();

    let mut entries = HashMap::new();
    for (pid, raw_response) in &received {
        entries.insert(
            pid.clone(),
            CacheEntry {
                raw_response: raw_response.clone(),
                available: true,
                cached_at: 1000,
            },
        );
    }

    let cache = mgr.update_cache(vin, entries, vec![]).unwrap();
    assert_eq!(cache.entries.len(), 3);

    // Verify cache hit
    let (cached, uncached) = mgr.get_cached_pids(vin, &discovery_pids).unwrap();
    assert_eq!(cached.len(), 3, "All discovery PIDs should be cached");
    assert!(uncached.is_empty());

    // Verify raw responses match what the mock returned
    let loaded = mgr.load(vin).unwrap().unwrap();
    for (pid, raw_response) in &received {
        assert_eq!(
            loaded.entries[pid].raw_response, *raw_response,
            "Cached response for {} should match mock response",
            pid
        );
    }

    api.cancel_subscription(sub_id).ok();
}
