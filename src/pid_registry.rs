//! Global PID Registry for deduplication and caching
//!
//! Manages PID subscriptions across multiple consumers to prevent redundant
//! queries and provides intelligent caching of responses.

use std::collections::{HashMap, HashSet};
#[allow(unused_imports)]
use std::time::Duration;
use std::time::{SystemTime, UNIX_EPOCH};
use uuid::Uuid;

/// Cached PID response with TTL
#[derive(Debug, Clone)]
pub struct CachedPIDResponse {
    /// The response data
    pub data: String,
    /// Timestamp when response was received (Unix timestamp in seconds)
    pub timestamp: f64,
    /// Time-to-live in milliseconds
    pub ttl_ms: u32,
}

impl CachedPIDResponse {
    /// Check if this cached response is still valid
    pub fn is_valid(&self) -> bool {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        let age_seconds = now - self.timestamp;
        let age_ms = age_seconds * 1000.0;
        age_ms < self.ttl_ms as f64
    }
}

/// Global PID registry for deduplication
#[derive(Debug)]
pub struct GlobalPIDRegistry {
    /// Maps PID ID to list of subscriptions requesting it
    pid_subscriptions: HashMap<String, HashSet<Uuid>>,
    /// Maps PID ID to cached last response
    pid_cache: HashMap<String, CachedPIDResponse>,
    /// Maps PID ID to target controller
    pid_controllers: HashMap<String, String>,
    /// Default cache TTL
    default_ttl_ms: u32,
}

impl GlobalPIDRegistry {
    /// Create a new PID registry
    pub fn new(default_ttl_ms: u32) -> Self {
        Self {
            pid_subscriptions: HashMap::new(),
            pid_cache: HashMap::new(),
            pid_controllers: HashMap::new(),
            default_ttl_ms,
        }
    }

    /// Register a PID subscription
    ///
    /// # Arguments
    /// * `pid` - The PID to subscribe to
    /// * `subscription_id` - ID of the subscription
    /// * `target_controller` - Optional target controller (e.g., "7E0")
    pub fn register_pid_subscription(
        &mut self,
        pid: &str,
        subscription_id: Uuid,
        target_controller: Option<String>,
    ) {
        // Add to subscription map
        self.pid_subscriptions
            .entry(pid.to_string())
            .or_insert_with(HashSet::new)
            .insert(subscription_id);

        // Set controller if provided
        if let Some(controller) = target_controller {
            self.pid_controllers.insert(pid.to_string(), controller);
        }
    }

    /// Unregister a PID subscription
    ///
    /// # Arguments
    /// * `pid` - The PID to unsubscribe from
    /// * `subscription_id` - ID of the subscription
    pub fn unregister_pid_subscription(&mut self, pid: &str, subscription_id: &Uuid) {
        if let Some(subscriptions) = self.pid_subscriptions.get_mut(pid) {
            subscriptions.remove(subscription_id);
            // Remove PID entirely if no more subscriptions
            if subscriptions.is_empty() {
                self.pid_subscriptions.remove(pid);
                self.pid_cache.remove(pid);
                self.pid_controllers.remove(pid);
            }
        }
    }

    /// Unregister all PIDs for a subscription
    ///
    /// # Arguments
    /// * `subscription_id` - ID of the subscription to remove
    pub fn unregister_subscription(&mut self, subscription_id: &Uuid) {
        let mut pids_to_remove = Vec::new();

        // Find all PIDs for this subscription
        for (pid, subscriptions) in &mut self.pid_subscriptions {
            subscriptions.remove(subscription_id);
            if subscriptions.is_empty() {
                pids_to_remove.push(pid.clone());
            }
        }

        // Clean up empty PIDs
        for pid in pids_to_remove {
            self.pid_subscriptions.remove(&pid);
            self.pid_cache.remove(&pid);
            self.pid_controllers.remove(&pid);
        }
    }

    /// Get all unique PIDs that need to be queried
    ///
    /// Returns PIDs sorted by controller for efficient batching
    pub fn get_unique_pids(&self) -> Vec<(String, Option<String>)> {
        let mut pids_by_controller: HashMap<Option<String>, Vec<String>> = HashMap::new();

        for pid in self.pid_subscriptions.keys() {
            let controller = self.pid_controllers.get(pid).cloned();
            pids_by_controller
                .entry(controller)
                .or_insert_with(Vec::new)
                .push(pid.clone());
        }

        let mut result = Vec::new();
        for (controller, mut pids) in pids_by_controller {
            pids.sort(); // Consistent ordering
            for pid in pids {
                result.push((pid, controller.clone()));
            }
        }

        result
    }

    /// Get subscriptions for a PID
    pub fn get_pid_subscriptions(&self, pid: &str) -> Option<&HashSet<Uuid>> {
        self.pid_subscriptions.get(pid)
    }

    /// Cache a PID response
    ///
    /// # Arguments
    /// * `pid` - The PID that was queried
    /// * `data` - The response data
    /// * `ttl_ms` - Optional TTL override, uses default if None
    pub fn cache_pid_response(&mut self, pid: &str, data: String, ttl_ms: Option<u32>) {
        let ttl = ttl_ms.unwrap_or(self.default_ttl_ms);
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        let cached = CachedPIDResponse {
            data,
            timestamp,
            ttl_ms: ttl,
        };
        self.pid_cache.insert(pid.to_string(), cached);
    }

    /// Get cached PID response if valid
    pub fn get_cached_pid_response(&self, pid: &str) -> Option<&CachedPIDResponse> {
        self.pid_cache.get(pid).filter(|cached| cached.is_valid())
    }

    /// Clear expired cache entries
    pub fn clear_expired_cache(&mut self) {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs_f64();
        self.pid_cache.retain(|_, cached| {
            let age_seconds = now - cached.timestamp;
            let age_ms = age_seconds * 1000.0;
            age_ms < cached.ttl_ms as f64
        });
    }

    /// Get controller for a PID
    pub fn get_pid_controller(&self, pid: &str) -> Option<&String> {
        self.pid_controllers.get(pid)
    }

    /// Check if a PID is subscribed to
    pub fn is_pid_subscribed(&self, pid: &str) -> bool {
        self.pid_subscriptions.contains_key(pid)
    }

    /// Get subscription count for a PID
    pub fn get_pid_subscription_count(&self, pid: &str) -> usize {
        self.pid_subscriptions
            .get(pid)
            .map(|s| s.len())
            .unwrap_or(0)
    }

    /// Get total number of unique PIDs
    pub fn unique_pid_count(&self) -> usize {
        self.pid_subscriptions.len()
    }

    /// Get total number of subscriptions
    pub fn total_subscription_count(&self) -> usize {
        self.pid_subscriptions.values().map(|s| s.len()).sum()
    }

    /// Get all PIDs for debugging
    pub fn get_all_pids(&self) -> Vec<String> {
        let mut pids: Vec<_> = self.pid_subscriptions.keys().cloned().collect();
        pids.sort();
        pids
    }

    /// Clear all cached responses
    pub fn clear_cache(&mut self) {
        self.pid_cache.clear();
    }

    /// Set default cache TTL
    pub fn set_default_ttl(&mut self, ttl_ms: u32) {
        self.default_ttl_ms = ttl_ms;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_pid_registration() {
        let mut registry = GlobalPIDRegistry::new(5000);

        let sub1 = Uuid::new_v4();
        let sub2 = Uuid::new_v4();

        // Register PIDs
        registry.register_pid_subscription("010C", sub1, Some("7E0".to_string()));
        registry.register_pid_subscription("010D", sub1, None);
        registry.register_pid_subscription("010C", sub2, Some("7E0".to_string())); // Duplicate

        // Check registrations
        assert_eq!(registry.get_pid_subscription_count("010C"), 2);
        assert_eq!(registry.get_pid_subscription_count("010D"), 1);
        assert_eq!(registry.unique_pid_count(), 2);
        assert_eq!(registry.total_subscription_count(), 3);

        // Check controller mapping
        assert_eq!(
            registry.get_pid_controller("010C"),
            Some(&"7E0".to_string())
        );
        assert_eq!(registry.get_pid_controller("010D"), None);
    }

    #[test]
    fn test_pid_unregistration() {
        let mut registry = GlobalPIDRegistry::new(5000);

        let sub1 = Uuid::new_v4();
        let sub2 = Uuid::new_v4();

        // Register PIDs
        registry.register_pid_subscription("010C", sub1, None);
        registry.register_pid_subscription("010D", sub1, None);
        registry.register_pid_subscription("010C", sub2, None);

        // Unregister one subscription from 010C
        registry.unregister_pid_subscription("010C", &sub1);
        assert_eq!(registry.get_pid_subscription_count("010C"), 1);

        // Unregister entire subscription
        registry.unregister_subscription(&sub1);
        assert_eq!(registry.get_pid_subscription_count("010C"), 1); // sub2 still has it
        assert_eq!(registry.get_pid_subscription_count("010D"), 0); // sub1 was only subscriber
        assert!(!registry.is_pid_subscribed("010D"));
    }

    #[test]
    fn test_unique_pids() {
        let mut registry = GlobalPIDRegistry::new(5000);

        let sub1 = Uuid::new_v4();

        registry.register_pid_subscription("010C", sub1, Some("7E0".to_string()));
        registry.register_pid_subscription("010D", sub1, Some("7E0".to_string()));
        registry.register_pid_subscription("0105", sub1, None);

        let unique_pids = registry.get_unique_pids();
        assert_eq!(unique_pids.len(), 3);

        // Should be grouped by controller
        let controller_7e0: Vec<_> = unique_pids
            .iter()
            .filter(|(_, controller)| controller.as_ref() == Some(&"7E0".to_string()))
            .collect();
        assert_eq!(controller_7e0.len(), 2);
    }

    #[test]
    fn test_pid_caching() {
        let mut registry = GlobalPIDRegistry::new(1000); // 1 second TTL

        // Cache a response
        registry.cache_pid_response("010C", "41 0C 1A F8".to_string(), None);

        // Should be retrievable immediately
        let cached = registry.get_cached_pid_response("010C");
        assert!(cached.is_some());
        assert_eq!(cached.unwrap().data, "41 0C 1A F8");

        // Should still be valid after short wait
        thread::sleep(Duration::from_millis(500));
        assert!(registry.get_cached_pid_response("010C").is_some());

        // Should expire after TTL
        thread::sleep(Duration::from_millis(600));
        assert!(registry.get_cached_pid_response("010C").is_none());
    }

    #[test]
    fn test_cache_cleanup() {
        let mut registry = GlobalPIDRegistry::new(100); // Very short TTL

        registry.cache_pid_response("010C", "data".to_string(), None);
        assert!(registry.get_cached_pid_response("010C").is_some());

        // Wait for expiry
        thread::sleep(Duration::from_millis(150));

        // Cache should still exist but be expired
        assert!(registry.pid_cache.contains_key("010C"));

        // Clear expired entries
        registry.clear_expired_cache();
        assert!(!registry.pid_cache.contains_key("010C"));
    }
}
