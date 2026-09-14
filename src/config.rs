//! Configuration structures for OBD session management

/// Configuration for the command processor
#[derive(Debug, Clone)]
pub struct CommandProcessorConfig {
    /// Commands per second (rate limit)
    pub commands_per_second: f64,
    /// Maximum burst capacity for token bucket
    pub max_burst_tokens: usize,
    /// Default timeout for commands in milliseconds
    pub command_timeout_ms: u32,
    /// Maximum number of queued commands
    pub max_queue_size: usize,
}

impl Default for CommandProcessorConfig {
    fn default() -> Self {
        Self {
            commands_per_second: 20.0, // Default 20 commands/second
            max_burst_tokens: 30,      // Allow burst up to 30 tokens
            command_timeout_ms: 5000, // 5 second timeout (needed for multi-frame responses like VIN)
            max_queue_size: 1000,     // Reasonable queue limit
        }
    }
}

/// Configuration for the OBD session manager
#[derive(Debug, Clone)]
pub struct OBDSessionConfig {
    /// Command processor configuration
    pub command_processor: CommandProcessorConfig,
    /// Background thread poll interval
    pub poll_interval_ms: u64,
    /// Maximum subscriptions allowed
    pub max_subscriptions: usize,
    /// PID cache TTL in milliseconds
    pub pid_cache_ttl_ms: u32,
    /// Base path for session logs (None = no logging)
    /// Logs will be written to: {log_base_path}/{session_name}.log
    pub log_base_path: Option<String>,
    /// Session name for log file (None = auto-generate UUID)
    pub session_name: Option<String>,
    /// Base path for PID discovery cache (None = no caching)
    /// Cache files will be written to: {cache_path}/{VIN}.cache
    pub cache_path: Option<String>,
    /// AT init commands sent after Bluetooth connect (e.g., ["ATE0", "ATH1"])
    /// Empty = skip init entirely.
    pub init_commands: Vec<String>,
    /// BP1 kill switch: probe OBDLink Batched Commands (`STBC 1`) at init and
    /// pipe due picks into one exchange when supported. The probe gates
    /// hardware anyway (clones answer `?`); this exists to force serial.
    pub enable_pipe_batching: bool,
    /// B2 kill switch: probe J1979 multi-PID (`010C0D`) after identify and
    /// chunk length-confirmed Mode 01 pids when the VEHICLE supports it.
    /// The probe gates cars anyway; this exists to force un-chunked.
    pub enable_j1979_chunking: bool,
}

impl Default for OBDSessionConfig {
    fn default() -> Self {
        Self {
            command_processor: CommandProcessorConfig::default(),
            poll_interval_ms: 100,  // Poll every 100ms
            max_subscriptions: 100, // Reasonable subscription limit
            pid_cache_ttl_ms: 5000, // 5 second cache TTL
            log_base_path: None,    // No logging by default
            session_name: None,     // Auto-generate UUID if logging enabled
            cache_path: None,       // No caching by default
            init_commands: vec![
                // Sensible defaults if caller doesn't provide
                "ATE0".to_string(),
                "ATH1".to_string(),
            ],
            enable_pipe_batching: true,
            enable_j1979_chunking: true,
        }
    }
}

/// Configuration for PID registry
#[derive(Debug, Clone)]
pub struct PIDRegistryConfig {
    /// Default cache TTL for PID responses
    pub default_cache_ttl_ms: u32,
    /// Maximum cached responses per PID
    pub max_cache_entries: usize,
}

impl Default for PIDRegistryConfig {
    fn default() -> Self {
        Self {
            default_cache_ttl_ms: 5000, // 5 seconds
            max_cache_entries: 1000,    // Reasonable cache size
        }
    }
}
