//! Token bucket rate limiting implementation
//!
//! Provides rate limiting with burst capacity for OBD command execution.
//! Ensures commands are processed at a controlled rate while allowing
//! short bursts of activity.

use std::time::{Duration, Instant};

/// Token bucket rate limiter
///
/// Implements the token bucket algorithm for rate limiting:
/// - Fixed capacity bucket that refills at constant rate
/// - Each command consumes exactly 1 token
/// - Commands are blocked when bucket is empty
/// - Allows burst capacity for responsive behavior
#[derive(Debug)]
pub struct TokenBucket {
    /// Maximum capacity of the bucket
    capacity: f64,
    /// Rate of token refill (tokens per second)
    refill_rate: f64,
    /// Current number of tokens
    tokens: f64,
    /// Last refill timestamp
    last_refill: Instant,
}

impl TokenBucket {
    /// Create a new token bucket
    ///
    /// # Arguments
    /// * `capacity` - Maximum number of tokens the bucket can hold
    /// * `refill_rate` - Tokens added per second
    pub fn new(capacity: f64, refill_rate: f64) -> Self {
        Self {
            capacity,
            refill_rate,
            tokens: capacity, // Start full
            last_refill: Instant::now(),
        }
    }

    /// Create token bucket with commands per second
    ///
    /// # Arguments
    /// * `commands_per_second` - Target rate in commands/second
    /// * `burst_capacity` - Maximum burst capacity
    pub fn with_rate(commands_per_second: f64, burst_capacity: usize) -> Self {
        Self::new(burst_capacity as f64, commands_per_second)
    }

    /// Try to consume a single token
    ///
    /// Refills the bucket based on elapsed time, then attempts to consume 1 token.
    /// Returns true if token was consumed, false if rate limited.
    pub fn try_consume(&mut self) -> bool {
        self.refill();
        if self.tokens >= 1.0 {
            self.tokens -= 1.0;
            true
        } else {
            false
        }
    }

    /// Try to consume multiple tokens at once
    ///
    /// Useful for batch operations. Returns the number of tokens actually consumed.
    ///
    /// # Arguments
    /// * `tokens_needed` - Number of tokens to consume
    pub fn try_consume_batch(&mut self, tokens_needed: f64) -> f64 {
        self.refill();
        let available = self.tokens.min(tokens_needed);
        self.tokens -= available;
        available
    }

    /// Wait until a token is available and consume it
    ///
    /// This is a blocking operation that will sleep until a token is available.
    /// Use with caution in async contexts.
    pub fn consume_blocking(&mut self) {
        loop {
            self.refill();
            if self.tokens >= 1.0 {
                self.tokens -= 1.0;
                return;
            }

            // Calculate wait time for next token
            let wait_tokens = 1.0 - self.tokens;
            let wait_seconds = wait_tokens / self.refill_rate;
            let wait_duration = Duration::from_secs_f64(wait_seconds.max(0.001)); // Min 1ms

            std::thread::sleep(wait_duration);
        }
    }

    /// Get current token count (after refill)
    pub fn available_tokens(&mut self) -> f64 {
        self.refill();
        self.tokens
    }

    /// Check if at least one token is available
    pub fn can_consume(&mut self) -> bool {
        self.refill();
        self.tokens >= 1.0
    }

    /// Get time until next token is available
    pub fn time_until_next_token(&mut self) -> Duration {
        self.refill();
        if self.tokens >= 1.0 {
            Duration::ZERO
        } else {
            let tokens_needed = 1.0 - self.tokens;
            Duration::from_secs_f64(tokens_needed / self.refill_rate)
        }
    }

    /// Refill tokens based on elapsed time
    fn refill(&mut self) {
        let now = Instant::now();
        let elapsed = now.duration_since(self.last_refill).as_secs_f64();
        self.tokens = (self.tokens + elapsed * self.refill_rate).min(self.capacity);
        self.last_refill = now;
    }
}

/// Rate limit error types
#[derive(Debug, Clone, PartialEq)]
pub enum RateLimitError {
    /// Operation blocked due to rate limiting
    RateLimited,
    /// Rate limiter is at capacity
    AtCapacity,
}

impl std::fmt::Display for RateLimitError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RateLimitError::RateLimited => write!(f, "Rate limited"),
            RateLimitError::AtCapacity => write!(f, "At capacity"),
        }
    }
}

impl std::error::Error for RateLimitError {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn test_token_bucket_basic() {
        let mut bucket = TokenBucket::new(10.0, 2.0); // 10 capacity, 2 tokens/sec

        // Should start full
        assert!(bucket.available_tokens() >= 10.0);

        // Should be able to consume initially
        assert!(bucket.try_consume());
        assert_eq!(bucket.available_tokens().round(), 9.0);
    }

    #[test]
    fn test_token_bucket_rate_limiting() {
        let mut bucket = TokenBucket::new(1.0, 0.1); // 1 capacity, 0.1 tokens/sec

        // Consume the initial token
        assert!(bucket.try_consume());
        assert!(!bucket.can_consume());

        // Should still be rate limited after short wait
        std::thread::sleep(Duration::from_millis(100));
        assert!(!bucket.can_consume());

        // Should have token after 10 seconds
        std::thread::sleep(Duration::from_secs(10));
        assert!(bucket.can_consume());
    }

    #[test]
    fn test_token_bucket_refill() {
        let mut bucket = TokenBucket::new(10.0, 1.0); // 1 token/sec

        // Consume all tokens
        for _ in 0..10 {
            assert!(bucket.try_consume());
        }
        assert!(!bucket.can_consume());

        // Wait for refill
        std::thread::sleep(Duration::from_secs(5));
        let available = bucket.available_tokens();
        assert!(available >= 5.0 && available <= 6.0); // Allow some timing tolerance
    }

    #[test]
    fn test_batch_consume() {
        let mut bucket = TokenBucket::new(10.0, 1.0);

        // Try to consume 5 tokens
        let consumed = bucket.try_consume_batch(5.0);
        assert_eq!(consumed.round(), 5.0);
        assert_eq!(bucket.available_tokens().round(), 5.0);

        // Try to consume more than available
        let consumed = bucket.try_consume_batch(10.0);
        assert_eq!(consumed.round(), 5.0);
        assert_eq!(bucket.available_tokens().round(), 0.0);
    }
}
