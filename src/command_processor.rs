//! Command Processor with serial execution and rate limiting
//!
//! Manages command execution with strict serial processing to ensure
//! OBD-II adapter compatibility and implements token bucket rate limiting.

#![allow(dead_code)]

use std::collections::VecDeque;
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use crate::error::SessionError;
use crate::platform::OBDPlatformInterface;
use crate::rate_limit::TokenBucket;

/// Statistics for command processing performance
#[derive(Debug, Clone)]
pub struct CommandProcessorStats {
    /// Total commands processed
    pub total_commands: u64,
    /// Commands completed successfully
    pub completed_commands: u64,
    /// Commands that failed
    pub failed_commands: u64,
    /// Start time of statistics collection
    pub start_time: Instant,
    /// Running sum of command completion times (for average calculation).
    /// Sum + count instead of a Vec — the per-command Vec grew unbounded
    /// over a long session (hours at 10–20 cmd/s = millions of entries).
    pub completion_time_total: Duration,
    /// Number of completions in `completion_time_total`.
    pub completion_time_count: u64,
    /// Current commands in progress
    pub in_progress_commands: u32,
    /// Signal (member) completions — a piped exchange counts once per member
    /// that got data, so this measures delivered signal updates where
    /// `completed_commands` measures wire exchanges (BP2).
    pub completed_signals: u64,
    /// Command-completion timestamps inside the rolling rate window (pruned
    /// on insert) — rates answer "how fast RIGHT NOW", not session-average,
    /// so idle/init time doesn't dilute them.
    pub recent_commands: VecDeque<Instant>,
    /// (timestamp, member count) per exchange inside the rolling window.
    pub recent_signals: VecDeque<(Instant, u64)>,
}

/// Rolling window for the per-second rate readouts.
pub const RATE_WINDOW: Duration = Duration::from_secs(10);

/// Prune entries older than the rolling window from a rate deque (stats
/// only). Entries are pushed in time order, so popping from the front until
/// the first in-window entry is exact.
fn prune_window<T>(deque: &mut VecDeque<T>, now: Instant, timestamp: impl Fn(&T) -> Instant) {
    if let Some(cutoff) = now.checked_sub(RATE_WINDOW) {
        while deque.front().is_some_and(|e| timestamp(e) < cutoff) {
            deque.pop_front();
        }
    }
}

impl Default for CommandProcessorStats {
    fn default() -> Self {
        Self {
            total_commands: 0,
            completed_commands: 0,
            failed_commands: 0,
            start_time: Instant::now(),
            completion_time_total: Duration::ZERO,
            completion_time_count: 0,
            in_progress_commands: 0,
            completed_signals: 0,
            recent_commands: VecDeque::new(),
            recent_signals: VecDeque::new(),
        }
    }
}

impl CommandProcessorStats {
    /// Seconds the rolling window currently covers: RATE_WINDOW once the
    /// session is older than it, the (shorter) session age before that.
    fn window_secs(&self) -> f64 {
        self.start_time
            .elapsed()
            .min(RATE_WINDOW)
            .as_secs_f64()
            .max(0.001)
    }

    /// Wire exchanges per second over the rolling window.
    pub fn pids_per_second(&self) -> f64 {
        let cutoff = Instant::now().checked_sub(RATE_WINDOW);
        let n = self
            .recent_commands
            .iter()
            .filter(|t| cutoff.map_or(true, |c| **t >= c))
            .count();
        n as f64 / self.window_secs()
    }

    /// Delivered signal updates per second over the rolling window
    /// (per-member; the honest measured counterpart to the projection's
    /// potential PIDs/sec).
    pub fn signals_per_second(&self) -> f64 {
        let cutoff = Instant::now().checked_sub(RATE_WINDOW);
        let n: u64 = self
            .recent_signals
            .iter()
            .filter(|(t, _)| cutoff.map_or(true, |c| *t >= c))
            .map(|(_, n)| *n)
            .sum();
        n as f64 / self.window_secs()
    }

    /// Calculate average completion time
    pub fn average_completion_time(&self) -> Option<Duration> {
        if self.completion_time_count == 0 {
            None
        } else {
            Some(
                self.completion_time_total / self.completion_time_count.min(u32::MAX as u64) as u32,
            )
        }
    }

    /// Reset all statistics
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// Pending command in the queue
#[derive(Debug, Clone)]
struct PendingCommand {
    /// The OBD command string
    command: String,
    /// Timeout in milliseconds
    timeout_ms: u32,
    /// Timestamp when command was queued
    queued_at: Instant,
}

/// Outstanding command with timing info
#[derive(Debug)]
struct OutstandingCommand {
    command: String,
    /// When the command was added to the queue
    queued_at: Instant,
    /// When the command was actually sent to the adapter
    start_time: Instant,
}

/// Command processor with serial execution and response synchronization
pub struct CommandProcessor {
    /// Command queue
    queue: Arc<Mutex<VecDeque<PendingCommand>>>,
    /// Rate limiter
    rate_limiter: Arc<Mutex<TokenBucket>>,
    /// Control flag for background thread
    running: Arc<Mutex<bool>>,
    /// Condition variable for queue notifications
    queue_condvar: Arc<Condvar>,
    /// Maximum queue size
    max_queue_size: usize,
    /// Platform interface
    platform: Arc<dyn OBDPlatformInterface>,
    /// Currently outstanding command (None if no command in progress)
    outstanding_command: Arc<Mutex<Option<OutstandingCommand>>>,
    /// True while the worker is between popping a command off the queue and
    /// recording it as outstanding (a window that includes the rate limiter's
    /// blocking wait). Without this, `wait_for_idle` can observe
    /// queue-empty + no-outstanding mid-dispatch and report idle while a
    /// command is still on its way out — callers then read a response capture
    /// that doesn't exist yet.
    dispatching: std::sync::atomic::AtomicBool,
    /// Number of response callbacks currently executing. The completion-driven
    /// chain queues its NEXT command near the END of the data callback, well
    /// after `mark_command_completed` clears `outstanding` — so a live chain
    /// looks momentarily idle in that gap. A `kick_pipeline_if_idle` landing
    /// there would start a SECOND chain (pipeline permanently 2 deep). Holding
    /// a guard for the callback's whole duration makes `wait_for_idle` treat
    /// "completion in progress" as busy.
    completion_guards: Arc<std::sync::atomic::AtomicUsize>,
    /// While true, the worker does NOT dispatch queued commands — the
    /// stream's monitor mode owns the transport, and a command sent during
    /// monitor loses its response to the line tap (the transport's no-prompt
    /// fallback then DROPS the connection — hardware-verified). Queued work
    /// waits; released when the stream breaks/stops.
    hold_dispatch: std::sync::atomic::AtomicBool,
    /// Bumped on every queue_command. A live chain's "token" cycles
    /// queue → dispatching → outstanding → completion guard → queue; a
    /// sequential read of those four indicators can be lapped by the token
    /// (each stage read AFTER the token left it) and misreport idle. The only
    /// way the token wraps back upstream is a queue_command, so an idle
    /// verdict is valid only if this counter didn't move during the scan.
    activity: std::sync::atomic::AtomicU64,
    /// Is a completion-driven polling chain alive? Maintained by the session's
    /// data callback (true when a completion queues a successor, false when a
    /// trigger picks nothing / errors pause everything) and reset by
    /// abort/stop. `kick_pipeline_if_idle` claims chain birth through a CAS on
    /// this flag — an explicit invariant instead of observing momentary
    /// processor idleness, which a live chain fakes between completing one
    /// command and queueing the next.
    chain_alive: std::sync::atomic::AtomicBool,
    /// Condition variable for command completion
    command_complete_condvar: Arc<Condvar>,
    /// Command processing statistics
    stats: Arc<Mutex<CommandProcessorStats>>,
}

/// RAII guard from `begin_completion` — while alive, `wait_for_idle` reports
/// the processor busy (a response callback is mid-flight and may queue the
/// chain's next command).
pub struct CompletionGuard(Arc<std::sync::atomic::AtomicUsize>);

impl Drop for CompletionGuard {
    fn drop(&mut self) {
        self.0.fetch_sub(1, std::sync::atomic::Ordering::SeqCst);
    }
}

/// Timing info returned when a command completes.
#[derive(Debug, Clone)]
pub struct CommandTiming {
    /// Time spent waiting in the queue before being sent
    pub queue_time_ms: f64,
    /// Time from send to response (adapter round-trip)
    pub response_time_ms: f64,
    /// Total time from queue entry to response
    pub total_time_ms: f64,
}

impl CommandProcessor {
    /// Create a new command processor
    pub fn new(
        platform: Arc<dyn OBDPlatformInterface>,
        commands_per_second: f64,
        max_burst_tokens: usize,
        max_queue_size: usize,
    ) -> Self {
        let rate_limiter = TokenBucket::with_rate(commands_per_second, max_burst_tokens);

        Self {
            queue: Arc::new(Mutex::new(VecDeque::new())),
            rate_limiter: Arc::new(Mutex::new(rate_limiter)),
            running: Arc::new(Mutex::new(true)),
            queue_condvar: Arc::new(Condvar::new()),
            max_queue_size,
            platform,
            outstanding_command: Arc::new(Mutex::new(None)),
            dispatching: std::sync::atomic::AtomicBool::new(false),
            hold_dispatch: std::sync::atomic::AtomicBool::new(false),
            completion_guards: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
            activity: std::sync::atomic::AtomicU64::new(0),
            chain_alive: std::sync::atomic::AtomicBool::new(false),
            command_complete_condvar: Arc::new(Condvar::new()),
            stats: Arc::new(Mutex::new(CommandProcessorStats::default())),
        }
    }

    /// Queue a command for execution
    ///
    /// Returns immediately after queuing. Command will be executed asynchronously
    /// with rate limiting and serial processing.
    pub fn queue_command(&self, command: String, timeout_ms: u32) -> Result<(), SessionError> {
        let mut queue = self.queue.lock().unwrap();

        // Check queue size limit
        if queue.len() >= self.max_queue_size {
            return Err(SessionError::InternalError(
                "Command queue full".to_string(),
            ));
        }

        let pending = PendingCommand {
            command,
            timeout_ms,
            queued_at: Instant::now(),
        };

        queue.push_back(pending);
        self.activity
            .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        self.queue_condvar.notify_one();

        Ok(())
    }

    /// Start the command processing thread
    ///
    /// This spawns a background thread that processes commands serially
    /// with rate limiting.
    pub fn start_processing_thread(
        self: Arc<Self>,
    ) -> Result<thread::JoinHandle<()>, SessionError> {
        let handle = thread::spawn(move || {
            self.processing_loop();
        });

        Ok(handle)
    }

    /// Stop the command processing thread
    pub fn stop(&self) {
        self.hold_dispatch
            .store(false, std::sync::atomic::Ordering::SeqCst);
        self.set_chain_alive(false);
        *self.running.lock().unwrap() = false;
        // Notify while HOLDING the queue mutex: the idle branch of the
        // processing loop checks `running`, then waits on queue_condvar with
        // the queue lock held. Notifying without the lock can land in the gap
        // between that check and the wait — a lost wakeup that leaves the
        // worker asleep forever and hangs join() on shutdown.
        let _queue = self.queue.lock().unwrap();
        self.queue_condvar.notify_all();
        drop(_queue);
        // Same reasoning for a worker blocked on an outstanding command.
        let _outstanding = self.outstanding_command.lock().unwrap();
        self.command_complete_condvar.notify_all();
    }

    /// Main processing loop (runs on background thread)
    fn processing_loop(&self) {
        while *self.running.lock().unwrap() {
            // Check if we can process a command (no outstanding command)
            let can_process = {
                let outstanding = self.outstanding_command.lock().unwrap();
                outstanding.is_none()
            };

            if self.hold_dispatch.load(std::sync::atomic::Ordering::SeqCst) {
                // Transport held by the stream — park until released.
                std::thread::sleep(std::time::Duration::from_millis(50));
                continue;
            }
            if can_process {
                // Try to get next command from queue. `dispatching` is raised
                // under the queue lock so no observer can see the command as
                // neither queued, dispatching, nor outstanding.
                let pending_command = {
                    let mut queue = self.queue.lock().unwrap();
                    let cmd = queue.pop_front();
                    if cmd.is_some() {
                        self.dispatching
                            .store(true, std::sync::atomic::Ordering::SeqCst);
                    }
                    cmd
                };

                if let Some(command) = pending_command {
                    self.process_command(command);
                    // Cleared only after process_command has recorded the
                    // command as outstanding (or it already completed).
                    self.dispatching
                        .store(false, std::sync::atomic::Ordering::SeqCst);
                } else {
                    // No commands in queue, wait for new commands or completion
                    let mut outstanding = self.outstanding_command.lock().unwrap();
                    while *self.running.lock().unwrap() && outstanding.is_none() {
                        // Wait for either new commands or command completion
                        let queue_guard = self.queue.lock().unwrap();
                        if queue_guard.is_empty() {
                            // Wait for queue change or completion signal
                            drop(outstanding);
                            let _queue_guard = self.queue_condvar.wait(queue_guard).unwrap();
                            outstanding = self.outstanding_command.lock().unwrap();
                        } else {
                            break; // Have commands to process
                        }
                    }
                }
            } else {
                // Have outstanding command, wait for completion
                let mut outstanding = self.outstanding_command.lock().unwrap();
                while *self.running.lock().unwrap() && outstanding.is_some() {
                    outstanding = self.command_complete_condvar.wait(outstanding).unwrap();
                }
            }
        }
    }

    /// Process a single command with rate limiting
    fn process_command(&self, pending: PendingCommand) {
        // Apply rate limiting
        let mut rate_limiter = self.rate_limiter.lock().unwrap();
        rate_limiter.consume_blocking();
        drop(rate_limiter); // Release lock

        // Record command start in stats
        let command_start = Instant::now();
        {
            let mut stats = self.stats.lock().unwrap();
            stats.total_commands += 1;
            stats.in_progress_commands += 1;
        }

        // Mark command as outstanding before sending
        {
            let mut outstanding = self.outstanding_command.lock().unwrap();
            *outstanding = Some(OutstandingCommand {
                command: pending.command.clone(),
                queued_at: pending.queued_at,
                start_time: command_start,
            });
        }

        // Log command sent
        crate::platform::logging::log_command_sent(&pending.command);

        // Send command to platform
        // NOTE: Platform send_command must return immediately
        self.platform
            .send_command(&pending.command, pending.timeout_ms);
    }

    /// Hold/release dispatch (stream monitor mode owns the transport).
    pub fn set_hold_dispatch(&self, hold: bool) {
        self.hold_dispatch
            .store(hold, std::sync::atomic::Ordering::SeqCst);
        if !hold {
            // Wake the worker in case commands queued up during the hold.
            let _q = self.queue.lock().unwrap();
            self.queue_condvar.notify_all();
        }
    }

    /// Any command queued, dispatching, or outstanding? Downstream read order
    /// (same argument as in `wait_for_idle`) so a command in transit is never
    /// missed. Used by the data callback to avoid declaring the chain dead
    /// while its next wire (or a pending one-shot) is still in flight.
    pub fn has_pending_work(&self) -> bool {
        if !self.queue.lock().unwrap().is_empty() {
            return true;
        }
        if self.dispatching.load(std::sync::atomic::Ordering::SeqCst) {
            return true;
        }
        self.outstanding_command.lock().unwrap().is_some()
    }

    /// Record whether the polling chain is alive (see `chain_alive`).
    pub fn set_chain_alive(&self, alive: bool) {
        self.chain_alive
            .store(alive, std::sync::atomic::Ordering::SeqCst);
    }

    /// Atomically claim the right to birth a new polling chain: succeeds only
    /// if no chain is alive, and marks one alive. The caller MUST then queue
    /// the chain's first command.
    pub fn try_claim_chain(&self) -> bool {
        self.chain_alive
            .compare_exchange(
                false,
                true,
                std::sync::atomic::Ordering::SeqCst,
                std::sync::atomic::Ordering::SeqCst,
            )
            .is_ok()
    }

    /// Mark the start of response-callback processing; the returned guard
    /// keeps `wait_for_idle` reporting busy until it drops (see
    /// `completion_guards`). Take it at the TOP of the data callback so the
    /// chain never looks idle between clearing `outstanding` and queueing the
    /// next command.
    pub fn begin_completion(&self) -> CompletionGuard {
        self.completion_guards
            .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        CompletionGuard(Arc::clone(&self.completion_guards))
    }

    /// Mark a command as completed (called when response is received).
    /// Returns timing info if the command matched the outstanding command.
    pub fn mark_command_completed(&self, command: &str) -> Option<CommandTiming> {
        let mut outstanding = self.outstanding_command.lock().unwrap();
        if let Some(current_command) = &*outstanding {
            if current_command.command == command {
                let response_time = current_command.start_time.elapsed();
                let queue_time = current_command.queued_at.elapsed() - response_time;
                let total_time = current_command.queued_at.elapsed();

                // Update stats
                let mut stats = self.stats.lock().unwrap();
                stats.completed_commands += 1;
                stats.in_progress_commands -= 1;
                stats.completion_time_total += response_time;
                stats.completion_time_count += 1;
                let now = Instant::now();
                stats.recent_commands.push_back(now);
                prune_window(&mut stats.recent_commands, now, |t| *t);

                *outstanding = None;
                self.command_complete_condvar.notify_all();
                // Also notify queue waiters in case there are queued commands
                self.queue_condvar.notify_one();

                return Some(CommandTiming {
                    queue_time_ms: queue_time.as_secs_f64() * 1000.0,
                    response_time_ms: response_time.as_secs_f64() * 1000.0,
                    total_time_ms: total_time.as_secs_f64() * 1000.0,
                });
            }
        }
        None
    }

    /// Get current queue size
    pub fn queue_size(&self) -> usize {
        self.queue.lock().unwrap().len()
    }

    /// Check if processor is running
    pub fn is_running(&self) -> bool {
        *self.running.lock().unwrap()
    }

    /// Get available rate limit tokens
    pub fn available_tokens(&self) -> f64 {
        self.rate_limiter.lock().unwrap().available_tokens()
    }

    /// Clear the command queue
    pub fn clear_queue(&self) {
        self.queue.lock().unwrap().clear();
    }

    /// Abandon the in-flight command (if any) and clear the queue. Used on
    /// disconnect: without this, a command outstanding at disconnect time
    /// (e.g. a subscription poll) is NEVER completed, so the processing loop
    /// blocks on it forever and the NEXT connect's init commands can never be
    /// picked up — `wait_for_idle` then times out ("Adapter initialization
    /// timed out"). Notifies waiters so both the loop and `wait_for_idle`
    /// unblock.
    pub fn abort_outstanding_and_clear(&self) {
        // A dead/dropped connection also ends any transport hold — the next
        // connect's init commands must dispatch.
        self.hold_dispatch
            .store(false, std::sync::atomic::Ordering::SeqCst);
        // An aborted in-flight command's callback never runs, so no successor
        // will ever be queued — any chain is dead. Without this reset a stale
        // `chain_alive = true` would suppress every future kick and the next
        // session's dashboard would never poll.
        self.set_chain_alive(false);
        self.queue.lock().unwrap().clear();
        let mut outstanding = self.outstanding_command.lock().unwrap();
        if outstanding.is_some() {
            *outstanding = None;
            let mut stats = self.stats.lock().unwrap();
            if stats.in_progress_commands > 0 {
                stats.in_progress_commands -= 1;
            }
        }
        self.command_complete_condvar.notify_all();
        self.queue_condvar.notify_all();
    }

    /// Update rate limiting parameters
    pub fn update_rate_limit(&self, commands_per_second: f64, max_burst_tokens: usize) {
        let mut rate_limiter = self.rate_limiter.lock().unwrap();
        *rate_limiter = TokenBucket::with_rate(commands_per_second, max_burst_tokens);
    }

    /// Get current statistics
    /// BP2: count delivered signal updates — one per member that got data,
    /// so a piped exchange adds several where completed_commands adds one.
    pub fn record_signal_completions(&self, n: u64) {
        if n > 0 {
            let mut stats = self.stats.lock().unwrap();
            stats.completed_signals += n;
            let now = Instant::now();
            stats.recent_signals.push_back((now, n));
            prune_window(&mut stats.recent_signals, now, |(t, _)| *t);
        }
    }

    pub fn get_stats(&self) -> CommandProcessorStats {
        self.stats.lock().unwrap().clone()
    }

    /// Wait until the command queue is empty and no command is outstanding.
    ///
    /// Blocks the calling thread until all queued commands have completed.
    /// Returns after timeout if commands don't complete in time.
    pub fn wait_for_idle(&self, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;

        // Wait for outstanding command to complete
        {
            let mut outstanding = self.outstanding_command.lock().unwrap();
            while outstanding.is_some() {
                let remaining = deadline.saturating_duration_since(Instant::now());
                if remaining.is_zero() {
                    return false;
                }
                let (guard, timeout_result) = self
                    .command_complete_condvar
                    .wait_timeout(outstanding, remaining)
                    .unwrap();
                outstanding = guard;
                if timeout_result.timed_out() && outstanding.is_some() {
                    return false;
                }
            }
        }

        // Check if queue is also empty. READ ORDER MATTERS: a live command
        // ("token") moves queue → dispatching → outstanding → completion
        // guard, with overlap at every hand-off (dispatching raised under the
        // queue lock; outstanding set before dispatching clears; the guard
        // taken before outstanding clears). Scanning in that same downstream
        // order can therefore never miss it — and the only way back upstream
        // is a queue_command, which the activity epoch detects.
        loop {
            let epoch = self.activity.load(std::sync::atomic::Ordering::SeqCst);
            let queue_empty = self.queue.lock().unwrap().is_empty();
            let not_dispatching = !self.dispatching.load(std::sync::atomic::Ordering::SeqCst);
            let no_outstanding = self.outstanding_command.lock().unwrap().is_none();
            let no_completions = self
                .completion_guards
                .load(std::sync::atomic::Ordering::SeqCst)
                == 0;
            let epoch_stable = self.activity.load(std::sync::atomic::Ordering::SeqCst) == epoch;

            if queue_empty && not_dispatching && no_outstanding && no_completions && epoch_stable {
                return true;
            }

            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return false;
            }

            // Wait for next completion signal
            let outstanding = self.outstanding_command.lock().unwrap();
            if outstanding.is_some() {
                let _ = self
                    .command_complete_condvar
                    .wait_timeout(outstanding, remaining)
                    .unwrap();
            } else {
                // Queue not empty but no outstanding — processing loop hasn't picked it up yet
                drop(outstanding);
                thread::sleep(Duration::from_millis(10));
            }
        }
    }

    /// Reset statistics
    pub fn reset_stats(&self) {
        let mut stats = self.stats.lock().unwrap();
        stats.reset();
    }
}

/// Command processor builder for fluent configuration
pub struct CommandProcessorBuilder {
    commands_per_second: f64,
    max_burst_tokens: usize,
    max_queue_size: usize,
}

impl CommandProcessorBuilder {
    /// Create a new builder with defaults
    pub fn new() -> Self {
        Self {
            commands_per_second: 20.0,
            max_burst_tokens: 30,
            max_queue_size: 1000,
        }
    }

    /// Set commands per second
    pub fn commands_per_second(mut self, cps: f64) -> Self {
        self.commands_per_second = cps;
        self
    }

    /// Set maximum burst tokens
    pub fn max_burst_tokens(mut self, tokens: usize) -> Self {
        self.max_burst_tokens = tokens;
        self
    }

    /// Set maximum queue size
    pub fn max_queue_size(mut self, size: usize) -> Self {
        self.max_queue_size = size;
        self
    }

    /// Build the command processor
    pub fn build(self, platform: Arc<dyn OBDPlatformInterface>) -> CommandProcessor {
        CommandProcessor::new(
            platform,
            self.commands_per_second,
            self.max_burst_tokens,
            self.max_queue_size,
        )
    }
}

impl Default for CommandProcessorBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::external_platform::create_mock_external_platform;
    use std::sync::mpsc;

    #[test]
    fn test_command_queueing() {
        let (tx, rx) = mpsc::channel();
        let (platform, _context) = create_mock_external_platform();

        // Set up data callback to receive responses
        let tx_clone = tx.clone();
        platform.set_data_callback(Some(Box::new(move |cmd, result| {
            tx_clone.send((cmd, result)).unwrap();
        })));

        let processor = Arc::new(CommandProcessor::new(
            platform.clone() as Arc<dyn OBDPlatformInterface>,
            100.0,
            10,
            100,
        ));

        // Wire up completion callback to mark commands as completed
        let processor_for_callback = Arc::clone(&processor);
        platform.set_completion_callback(Some(Box::new(move |cmd: String| {
            processor_for_callback.mark_command_completed(&cmd);
        })));

        // Queue some commands
        processor.queue_command("ATZ".to_string(), 1000).unwrap();
        processor.queue_command("ATE0".to_string(), 1000).unwrap();

        assert_eq!(processor.queue_size(), 2);

        // Start processing thread
        let handle = Arc::clone(&processor).start_processing_thread().unwrap();

        // Wait for commands to be processed
        let (cmd1, _) = rx.recv_timeout(Duration::from_secs(2)).unwrap();
        let (cmd2, _) = rx.recv_timeout(Duration::from_secs(2)).unwrap();

        assert_eq!(cmd1, "ATZ");
        assert_eq!(cmd2, "ATE0");

        // Stop processor
        processor.stop();
        handle.join().unwrap();
    }

    #[test]
    fn test_queue_size_limit() {
        let (platform, _context) = create_mock_external_platform();
        let processor = CommandProcessor::new(platform, 100.0, 10, 2);

        // Should succeed
        assert!(processor.queue_command("CMD1".to_string(), 1000).is_ok());
        assert!(processor.queue_command("CMD2".to_string(), 1000).is_ok());

        // Should fail - queue full
        assert!(processor.queue_command("CMD3".to_string(), 1000).is_err());
        assert_eq!(processor.queue_size(), 2);
    }

    #[test]
    fn test_rate_limiting() {
        let (tx, rx) = mpsc::channel();
        let (platform, _context) = create_mock_external_platform();

        // Set up data callback to receive responses
        let tx_clone = tx.clone();
        platform.set_data_callback(Some(Box::new(move |cmd, result| {
            tx_clone.send((cmd, result)).unwrap();
        })));

        // Very slow rate limiting: 0.1 commands/second (10 seconds per command)
        let processor = Arc::new(CommandProcessor::new(
            platform.clone() as Arc<dyn OBDPlatformInterface>,
            0.1,
            1,
            10,
        ));

        // Wire up completion callback to mark commands as completed
        let processor_for_callback = Arc::clone(&processor);
        platform.set_completion_callback(Some(Box::new(move |cmd: String| {
            processor_for_callback.mark_command_completed(&cmd);
        })));

        let handle = Arc::clone(&processor).start_processing_thread().unwrap();

        let start = Instant::now();

        // Queue two commands
        processor.queue_command("CMD1".to_string(), 1000).unwrap();
        processor.queue_command("CMD2".to_string(), 1000).unwrap();

        // Receive first command
        let (cmd1, _) = rx.recv_timeout(Duration::from_secs(2)).unwrap();
        assert_eq!(cmd1, "CMD1");

        // Second command should be delayed due to rate limiting
        // (Note: Mock platform has 50ms delay, so this test may be timing-sensitive)
        let (cmd2, _) = rx.recv_timeout(Duration::from_secs(15)).unwrap();
        assert_eq!(cmd2, "CMD2");

        let elapsed = start.elapsed();
        // Should take at least 10 seconds due to rate limiting
        assert!(elapsed >= Duration::from_secs(10));

        processor.stop();
        handle.join().unwrap();
    }

    #[test]
    fn test_clear_queue() {
        let (platform, _context) = create_mock_external_platform();
        let processor = CommandProcessor::new(platform, 100.0, 10, 10);

        processor.queue_command("CMD1".to_string(), 1000).unwrap();
        processor.queue_command("CMD2".to_string(), 1000).unwrap();
        assert_eq!(processor.queue_size(), 2);

        processor.clear_queue();
        assert_eq!(processor.queue_size(), 0);
    }
}
