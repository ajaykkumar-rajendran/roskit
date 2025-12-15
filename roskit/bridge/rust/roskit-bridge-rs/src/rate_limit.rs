//! Rate limiting module for RosKit Bridge
//!
//! Provides token bucket rate limiting for client operations.

use std::time::Instant;
use parking_lot::Mutex;

/// Token bucket rate limiter
pub struct RateLimiter {
    /// Tokens per second
    rate: f64,
    /// Maximum burst size
    burst: f64,
    /// Current token count
    tokens: Mutex<f64>,
    /// Last update time
    last_update: Mutex<Instant>,
}

impl RateLimiter {
    /// Create a new rate limiter
    ///
    /// # Arguments
    /// * `rate` - Tokens per second
    /// * `burst` - Maximum burst size (initial tokens)
    pub fn new(rate: f64, burst: f64) -> Self {
        Self {
            rate,
            burst,
            tokens: Mutex::new(burst),
            last_update: Mutex::new(Instant::now()),
        }
    }

    /// Check if a request should be allowed
    ///
    /// Returns `true` if the request is allowed (consumes one token),
    /// `false` if rate limited.
    pub fn allow(&self) -> bool {
        let now = Instant::now();
        let mut tokens = self.tokens.lock();
        let mut last_update = self.last_update.lock();

        // Add tokens based on elapsed time
        let elapsed = now.duration_since(*last_update).as_secs_f64();
        *tokens = (*tokens + elapsed * self.rate).min(self.burst);
        *last_update = now;

        if *tokens >= 1.0 {
            *tokens -= 1.0;
            true
        } else {
            false
        }
    }

    /// Check if a request should be allowed, consuming `n` tokens
    pub fn allow_n(&self, n: f64) -> bool {
        let now = Instant::now();
        let mut tokens = self.tokens.lock();
        let mut last_update = self.last_update.lock();

        // Add tokens based on elapsed time
        let elapsed = now.duration_since(*last_update).as_secs_f64();
        *tokens = (*tokens + elapsed * self.rate).min(self.burst);
        *last_update = now;

        if *tokens >= n {
            *tokens -= n;
            true
        } else {
            false
        }
    }

    /// Get current available tokens (for debugging/metrics)
    pub fn available_tokens(&self) -> f64 {
        *self.tokens.lock()
    }
}

/// Rate limit configuration
#[derive(Debug, Clone)]
pub struct RateLimitConfig {
    /// Subscribe requests per second
    pub subscribe_rate: f64,
    /// Subscribe burst size
    pub subscribe_burst: f64,
    /// Publish requests per second
    pub publish_rate: f64,
    /// Publish burst size
    pub publish_burst: f64,
    /// Service call requests per second
    pub service_rate: f64,
    /// Service call burst size
    pub service_burst: f64,
}

impl Default for RateLimitConfig {
    fn default() -> Self {
        Self {
            subscribe_rate: 10.0,
            subscribe_burst: 20.0,
            publish_rate: 100.0,
            publish_burst: 200.0,
            service_rate: 10.0,
            service_burst: 20.0,
        }
    }
}

impl RateLimitConfig {
    /// Create a disabled rate limit config (unlimited)
    pub fn disabled() -> Self {
        Self {
            subscribe_rate: f64::MAX,
            subscribe_burst: f64::MAX,
            publish_rate: f64::MAX,
            publish_burst: f64::MAX,
            service_rate: f64::MAX,
            service_burst: f64::MAX,
        }
    }

    /// Check if rate limiting is effectively disabled
    pub fn is_disabled(&self) -> bool {
        self.subscribe_rate >= f64::MAX / 2.0
    }
}

/// Per-client rate limiters
pub struct ClientRateLimiters {
    pub subscribe: RateLimiter,
    pub publish: RateLimiter,
    pub service: RateLimiter,
}

impl ClientRateLimiters {
    /// Create rate limiters from config
    pub fn new(config: &RateLimitConfig) -> Self {
        Self {
            subscribe: RateLimiter::new(config.subscribe_rate, config.subscribe_burst),
            publish: RateLimiter::new(config.publish_rate, config.publish_burst),
            service: RateLimiter::new(config.service_rate, config.service_burst),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread::sleep;
    use std::time::Duration;

    #[test]
    fn test_rate_limiter_allows_burst() {
        let limiter = RateLimiter::new(10.0, 5.0);

        // Should allow burst of 5
        for _ in 0..5 {
            assert!(limiter.allow());
        }

        // 6th request should be denied
        assert!(!limiter.allow());
    }

    #[test]
    fn test_rate_limiter_refills() {
        let limiter = RateLimiter::new(10.0, 1.0);

        // Use the initial token
        assert!(limiter.allow());
        assert!(!limiter.allow());

        // Wait for refill (100ms = 1 token at 10/s)
        sleep(Duration::from_millis(110));

        // Should have ~1 token now
        assert!(limiter.allow());
    }

    #[test]
    fn test_rate_limiter_burst_cap() {
        let limiter = RateLimiter::new(100.0, 5.0);

        // Wait long enough to accumulate many tokens
        sleep(Duration::from_millis(100));

        // Should only allow burst amount, not more
        for _ in 0..5 {
            assert!(limiter.allow());
        }
        assert!(!limiter.allow());
    }

    #[test]
    fn test_allow_n() {
        let limiter = RateLimiter::new(10.0, 10.0);

        // Consume 5 tokens
        assert!(limiter.allow_n(5.0));

        // Should have 5 left
        assert!(limiter.allow_n(5.0));

        // Should not have enough
        assert!(!limiter.allow_n(1.0));
    }

    #[test]
    fn test_config_default() {
        let config = RateLimitConfig::default();
        assert_eq!(config.subscribe_rate, 10.0);
        assert_eq!(config.publish_rate, 100.0);
        assert!(!config.is_disabled());
    }

    #[test]
    fn test_config_disabled() {
        let config = RateLimitConfig::disabled();
        assert!(config.is_disabled());
    }
}
