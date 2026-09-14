//! Session Monitor — real-time command/response tracing
//!
//! Captures every command that flows through the command processor with timing,
//! subscription context, and PID mismatch detection. Entries are stored in a
//! fixed-size ring buffer and optionally forwarded via a callback.

#![allow(dead_code)]

use serde::Serialize;
use std::collections::VecDeque;

/// A single captured command/response pair with timing and context.
#[derive(Debug, Clone, Serialize)]
pub struct SessionMonitorEntry {
    /// The command sent (e.g., "0105", "ATSH7E0")
    pub command: String,
    /// Which subscription triggered this (None for manual/AT commands)
    pub subscription_id: Option<String>,
    /// Human-readable subscription name
    pub subscription_name: Option<String>,
    /// Target controller header if set (e.g., "7E0")
    pub target_controller: Option<String>,
    /// Raw response from the adapter
    pub response: String,
    /// Time from queue entry to send (queue wait time)
    pub queue_time_ms: f64,
    /// Time from send to response (BLE round-trip)
    pub response_time_ms: f64,
    /// Total time from queue entry to response
    pub total_time_ms: f64,
    /// Whether the response was successful
    pub success: bool,
    /// If the response PID didn't match the sent command
    pub pid_mismatch: bool,
    /// The PID parsed from the response bytes (if applicable)
    pub response_pid: Option<String>,
    /// Unix timestamp
    pub timestamp: f64,
    /// Whether this was an AT command (not a PID query)
    pub is_at_command: bool,
    /// Refresh tier (Fast/Medium/Slow) if from a subscription
    pub tier: Option<String>,
}

/// Real-time callback fired for each captured entry.
pub type SessionMonitorCallback = Box<dyn Fn(&SessionMonitorEntry) + Send>;

/// Ring buffer for session monitor entries with optional real-time callback.
pub struct SessionMonitorBuffer {
    entries: VecDeque<SessionMonitorEntry>,
    max_entries: usize,
    callback: Option<SessionMonitorCallback>,
}

impl SessionMonitorBuffer {
    /// Create a new buffer with the given capacity.
    pub fn new(max_entries: usize) -> Self {
        Self {
            entries: VecDeque::with_capacity(max_entries),
            max_entries,
            callback: None,
        }
    }

    /// Push an entry, evicting the oldest if at capacity.
    /// Fires the callback (if installed) before storing.
    pub fn push(&mut self, entry: SessionMonitorEntry) {
        if let Some(cb) = &self.callback {
            cb(&entry);
        }
        self.entries.push_back(entry);
        if self.entries.len() > self.max_entries {
            self.entries.pop_front();
        }
    }

    /// Return the last `count` entries (most recent first).
    pub fn get_last(&self, count: usize) -> Vec<&SessionMonitorEntry> {
        self.entries.iter().rev().take(count).collect()
    }

    /// Return total number of entries currently stored.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether the buffer is empty.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Clear all entries.
    pub fn clear(&mut self) {
        self.entries.clear();
    }

    /// Install a real-time callback. Replaces any existing callback.
    pub fn set_callback(&mut self, callback: SessionMonitorCallback) {
        self.callback = Some(callback);
    }

    /// Remove the callback.
    pub fn remove_callback(&mut self) {
        self.callback = None;
    }

    /// Check if a callback is installed.
    pub fn has_callback(&self) -> bool {
        self.callback.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_entry(command: &str, response: &str) -> SessionMonitorEntry {
        SessionMonitorEntry {
            command: command.to_string(),
            subscription_id: None,
            subscription_name: None,
            target_controller: None,
            response: response.to_string(),
            queue_time_ms: 1.0,
            response_time_ms: 5.0,
            total_time_ms: 6.0,
            success: true,
            pid_mismatch: false,
            response_pid: None,
            timestamp: 1000.0,
            is_at_command: command.starts_with("AT"),
            tier: None,
        }
    }

    #[test]
    fn test_monitor_captures_entries() {
        let mut buffer = SessionMonitorBuffer::new(500);
        buffer.push(make_entry("010C", "7E8 04 41 0C 1A F8"));
        buffer.push(make_entry("010D", "7E8 03 41 0D 5C"));
        buffer.push(make_entry("0105", "7E8 03 41 05 4B"));

        assert_eq!(buffer.len(), 3);

        let last = buffer.get_last(2);
        assert_eq!(last.len(), 2);
        assert_eq!(last[0].command, "0105"); // most recent first
        assert_eq!(last[1].command, "010D");
    }

    #[test]
    fn test_monitor_ring_buffer_limits() {
        let mut buffer = SessionMonitorBuffer::new(500);
        for i in 0..600 {
            buffer.push(make_entry(&format!("CMD{}", i), "OK"));
        }
        assert_eq!(buffer.len(), 500);

        // Oldest should be CMD100 (0-99 were evicted)
        let all = buffer.get_last(500);
        assert_eq!(all.last().unwrap().command, "CMD100");
        assert_eq!(all.first().unwrap().command, "CMD599");
    }

    #[test]
    fn test_monitor_callback_fires() {
        use std::sync::{Arc, Mutex};

        let received = Arc::new(Mutex::new(Vec::new()));
        let received_clone = Arc::clone(&received);

        let mut buffer = SessionMonitorBuffer::new(500);
        buffer.set_callback(Box::new(move |entry| {
            received_clone.lock().unwrap().push(entry.command.clone());
        }));

        buffer.push(make_entry("010C", "OK"));
        buffer.push(make_entry("ATSH7E0", "OK"));

        let entries = received.lock().unwrap();
        assert_eq!(entries.len(), 2);
        assert_eq!(entries[0], "010C");
        assert_eq!(entries[1], "ATSH7E0");
    }

    #[test]
    fn test_monitor_detects_pid_mismatch() {
        use crate::subscription::SubscriptionManager;

        // Use the existing extract_pid_from_response
        let response = "7E8 03 41 04 32";
        let sent_pid = "010D";
        let response_pid = SubscriptionManager::extract_pid_from_response(response);

        assert_eq!(response_pid, Some("0104".to_string()));
        let mismatch = response_pid
            .as_ref()
            .map(|rp| rp != sent_pid)
            .unwrap_or(false);
        assert!(
            mismatch,
            "Should detect mismatch: sent 010D but response has 41 04"
        );
    }

    #[test]
    fn test_monitor_at_commands_flagged() {
        let entry = make_entry("ATSH7E0", "OK");
        assert!(entry.is_at_command);

        let entry2 = make_entry("010C", "7E8 04 41 0C 1A F8");
        assert!(!entry2.is_at_command);
    }

    #[test]
    fn test_get_history_returns_latest() {
        let mut buffer = SessionMonitorBuffer::new(500);
        for i in 0..100 {
            buffer.push(make_entry(&format!("PID{:03}", i), "OK"));
        }

        let last15 = buffer.get_last(15);
        assert_eq!(last15.len(), 15);
        assert_eq!(last15[0].command, "PID099"); // most recent
        assert_eq!(last15[14].command, "PID085"); // 15th most recent
    }

    #[test]
    fn test_clear_buffer() {
        let mut buffer = SessionMonitorBuffer::new(500);
        buffer.push(make_entry("010C", "OK"));
        buffer.push(make_entry("010D", "OK"));
        assert_eq!(buffer.len(), 2);

        buffer.clear();
        assert!(buffer.is_empty());
        assert_eq!(buffer.len(), 0);
    }

    #[test]
    fn test_entry_serializes_to_json() {
        let mut entry = make_entry("010C", "7E8 04 41 0C 1A F8");
        entry.subscription_id = Some("abc-123".to_string());
        entry.target_controller = Some("7E0".to_string());
        entry.response_pid = Some("010C".to_string());
        entry.tier = Some("Fast".to_string());

        let json = serde_json::to_string(&entry).unwrap();
        assert!(json.contains("\"command\":\"010C\""));
        assert!(json.contains("\"subscription_id\":\"abc-123\""));
        assert!(json.contains("\"tier\":\"Fast\""));
        assert!(json.contains("\"pid_mismatch\":false"));
    }
}
