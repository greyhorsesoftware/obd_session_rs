//! Session-owned passive-listen buffer (plan M7).
//!
//! ONE queue + counters shared by every transport: the session hands each
//! platform an `on_line` closure via `OBDPlatformInterface::enter_monitor_mode`,
//! and whatever the transport captures lands here. Transport-agnostic by
//! construction — `sink_read`/`sink_stats`/drop-oldest are written once.

use std::collections::VecDeque;
use std::sync::Mutex;

/// A captured frame/line while sinking. `raw` is the line as the link
/// delivered it (LH3: prompt stripped) — the host sniffer, the learn-streaming
/// histogram and the MCP `sink_read` consume it verbatim. Frames additionally
/// carry the parsed `id` / `data` and the arrival stamp `ts_us`; non-frame
/// lines (`STOPPED`, `BUFFER FULL`, the gap marker) have `id == None`.
#[derive(Clone)]
pub struct SinkFrame {
    pub t_ms: u64,
    pub raw: String,
    pub id: Option<String>,
    pub ts_us: u64,
    pub data: Vec<u8>,
}

/// Counters snapshot (`sink_stats`).
#[derive(Clone, Copy)]
pub struct SinkStats {
    pub active: bool,
    pub started_at_ms: u64,
    pub received_total: u64,
    pub read_total: u64,
    pub queued: u64,
    pub dropped: u64,
}

struct Inner {
    active: bool,
    started_at_ms: u64,
    queue: VecDeque<SinkFrame>,
    received_total: u64,
    read_total: u64,
    dropped: u64,
}

/// Bounded drop-oldest ring with counters. Thread-safe; `push` is called from
/// arbitrary transport threads, reads from the FFI/API side.
pub struct SinkBuffer {
    inner: Mutex<Inner>,
}

impl SinkBuffer {
    pub const CAPACITY: usize = 10_000;

    pub fn new() -> Self {
        SinkBuffer {
            inner: Mutex::new(Inner {
                active: false,
                started_at_ms: 0,
                queue: VecDeque::new(),
                received_total: 0,
                read_total: 0,
                dropped: 0,
            }),
        }
    }

    fn now_ms() -> u64 {
        use std::time::{SystemTime, UNIX_EPOCH};
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0)
    }

    /// Reset counters and mark active.
    pub fn start(&self) {
        let mut s = self.inner.lock().unwrap();
        *s = Inner {
            active: true,
            started_at_ms: Self::now_ms(),
            queue: VecDeque::new(),
            received_total: 0,
            read_total: 0,
            dropped: 0,
        };
    }

    /// Append a captured non-frame line. No-op when not active. Drops the
    /// oldest entry past capacity and counts the drop.
    pub fn push(&self, line: String) {
        let ts_us = Self::now_ms() * 1000;
        self.push_entry(SinkFrame {
            t_ms: ts_us / 1000,
            raw: line,
            id: None,
            ts_us,
            data: Vec::new(),
        });
    }

    /// Append a parsed frame (LH3) — stamped at the link's arrival time.
    pub fn push_frame(&self, frame: crate::link::Frame) {
        self.push_entry(SinkFrame {
            t_ms: frame.ts_us / 1000,
            raw: frame.raw,
            id: Some(frame.id),
            ts_us: frame.ts_us,
            data: frame.data,
        });
    }

    fn push_entry(&self, entry: SinkFrame) {
        let mut s = self.inner.lock().unwrap();
        if !s.active {
            return;
        }
        s.received_total += 1;
        if s.queue.len() >= Self::CAPACITY {
            s.queue.pop_front();
            s.dropped += 1;
        }
        s.queue.push_back(entry);
    }

    /// Drain up to `max` queued frames (oldest first). Allowed after `stop`
    /// so a final read can collect the tail.
    pub fn read(&self, max: usize) -> Vec<SinkFrame> {
        let mut s = self.inner.lock().unwrap();
        let n = max.min(s.queue.len());
        let out: Vec<SinkFrame> = s.queue.drain(..n).collect();
        s.read_total += out.len() as u64;
        out
    }

    pub fn stats(&self) -> SinkStats {
        let s = self.inner.lock().unwrap();
        SinkStats {
            active: s.active,
            started_at_ms: s.started_at_ms,
            received_total: s.received_total,
            read_total: s.read_total,
            queued: s.queue.len() as u64,
            dropped: s.dropped,
        }
    }

    /// Mark inactive; further pushes are ignored. Queue retained for a final read.
    pub fn stop(&self) {
        self.inner.lock().unwrap().active = false;
    }

    pub fn is_active(&self) -> bool {
        self.inner.lock().unwrap().active
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn queue_read_and_stats() {
        let buf = SinkBuffer::new();

        // Inactive: pushes are ignored.
        buf.push("ignored".to_string());
        assert_eq!(buf.read(10).len(), 0);

        buf.start();
        assert!(buf.is_active());
        for i in 0..5 {
            buf.push(format!("line {i}"));
        }
        let s = buf.stats();
        assert!(s.active);
        assert_eq!(s.received_total, 5);
        assert_eq!(s.queued, 5);
        assert_eq!(s.dropped, 0);

        let frames = buf.read(3);
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[0].raw, "line 0"); // oldest first
        let s = buf.stats();
        assert_eq!(s.read_total, 3);
        assert_eq!(s.queued, 2);

        buf.stop();
        assert!(!buf.is_active());
        // Remaining frames still drainable after stop.
        assert_eq!(buf.read(10).len(), 2);
    }

    #[test]
    fn ring_drops_oldest_past_capacity() {
        let buf = SinkBuffer::new();
        buf.start();
        for i in 0..(SinkBuffer::CAPACITY + 100) {
            buf.push(format!("f{i}"));
        }
        let s = buf.stats();
        assert_eq!(s.received_total as usize, SinkBuffer::CAPACITY + 100);
        assert_eq!(s.queued as usize, SinkBuffer::CAPACITY);
        assert_eq!(s.dropped, 100);
        // The oldest 100 were dropped; the front is now f100.
        let front = buf.read(1);
        assert_eq!(front[0].raw, "f100");
    }

    #[test]
    fn restart_resets_counters() {
        let buf = SinkBuffer::new();
        buf.start();
        buf.push("a".into());
        buf.stop();
        buf.start();
        let s = buf.stats();
        assert_eq!(s.received_total, 0);
        assert_eq!(s.queued, 0);
    }
}
