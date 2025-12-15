//! RosKit Bridge - High-performance ROS2 Web Bridge
//!
//! A Rust implementation of the RosKit bridge providing WebSocket connectivity
//! to ROS2 with optimized binary encoding for large data types.

mod auth;
mod encoders;
mod protocol;
mod rate_limit;
mod ros2;
mod server;
mod tls;

use auth::AuthConfig;
use clap::Parser;
use rate_limit::RateLimitConfig;
use server::{Server, ServerConfig};
use std::sync::Arc;
use tls::TlsConfig;
use tracing::info;
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

/// RosKit Bridge - High-performance ROS2 Web Bridge
#[derive(Parser, Debug)]
#[command(name = "roskit-bridge")]
#[command(author = "RosKit Contributors")]
#[command(version)]
#[command(about = "High-performance ROS2 web bridge with binary protocol support")]
struct Args {
    /// Host address to bind to
    #[arg(short = 'H', long, default_value = "0.0.0.0")]
    host: String,

    /// Port to listen on
    #[arg(short, long, default_value_t = 9091)]
    port: u16,

    /// ROS distribution name
    #[arg(long, default_value = "humble")]
    ros_distro: String,

    /// Enable verbose logging
    #[arg(short, long)]
    verbose: bool,

    /// Idle timeout in seconds (0 = disabled)
    #[arg(long, default_value_t = 300)]
    idle_timeout: u64,

    /// Heartbeat interval in seconds (0 = disabled)
    #[arg(long, default_value_t = 30)]
    heartbeat_interval: u64,

    /// Maximum message queue size
    #[arg(long, default_value_t = 1024)]
    max_queue_size: usize,

    /// Maximum message size in bytes (0 = unlimited)
    #[arg(long, default_value_t = 10485760)]
    max_message_size: usize,

    // TLS options
    /// Path to TLS certificate file (PEM format). Enables TLS when set.
    #[arg(long)]
    tls_cert: Option<String>,

    /// Path to TLS private key file (PEM format). Required if --tls-cert is set.
    #[arg(long)]
    tls_key: Option<String>,

    // Authentication options
    /// Enable authentication with specified token(s). Can be specified multiple times.
    #[arg(long = "auth-token")]
    auth_tokens: Vec<String>,

    /// Path to file containing authentication tokens (one per line)
    #[arg(long)]
    auth_token_file: Option<String>,

    // Rate limiting options
    /// Subscribe requests per second per client (0 = unlimited)
    #[arg(long, default_value_t = 10.0)]
    rate_limit_subscribe: f64,

    /// Publish requests per second per client (0 = unlimited)
    #[arg(long, default_value_t = 100.0)]
    rate_limit_publish: f64,

    /// Service call requests per second per client (0 = unlimited)
    #[arg(long, default_value_t = 10.0)]
    rate_limit_service: f64,

    /// Disable rate limiting entirely
    #[arg(long)]
    no_rate_limit: bool,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let args = Args::parse();

    // Initialize logging
    let filter = if args.verbose {
        "roskit_bridge=debug,tower_http=debug"
    } else {
        "roskit_bridge=info"
    };

    tracing_subscriber::registry()
        .with(tracing_subscriber::EnvFilter::try_from_default_env().unwrap_or_else(|_| filter.into()))
        .with(tracing_subscriber::fmt::layer())
        .init();

    info!(
        "Starting RosKit Bridge v{} (ROS {})",
        env!("CARGO_PKG_VERSION"),
        args.ros_distro
    );

    // Configure TLS if certificates are provided
    let tls_config = match (&args.tls_cert, &args.tls_key) {
        (Some(cert), Some(key)) => {
            info!("TLS enabled");
            Some(TlsConfig::new(cert, key))
        }
        (Some(_), None) => {
            return Err("--tls-key is required when --tls-cert is specified".into());
        }
        (None, Some(_)) => {
            return Err("--tls-cert is required when --tls-key is specified".into());
        }
        (None, None) => None,
    };

    // Configure authentication
    let mut auth_config = if !args.auth_tokens.is_empty() {
        info!("Authentication enabled with {} token(s)", args.auth_tokens.len());
        AuthConfig::with_tokens(args.auth_tokens)
    } else if let Some(ref token_file) = args.auth_token_file {
        info!("Authentication enabled with token file: {}", token_file);
        AuthConfig::with_token_file(token_file)
    } else {
        AuthConfig::default()
    };

    // Load tokens from file if configured
    if auth_config.token_file.is_some() {
        auth_config.load_tokens_from_file()?;
    }

    // Configure rate limiting
    let rate_limit_config = if args.no_rate_limit {
        info!("Rate limiting disabled");
        RateLimitConfig::disabled()
    } else {
        info!(
            "Rate limits: subscribe={}/s, publish={}/s, service={}/s",
            args.rate_limit_subscribe, args.rate_limit_publish, args.rate_limit_service
        );
        RateLimitConfig {
            subscribe_rate: args.rate_limit_subscribe,
            subscribe_burst: args.rate_limit_subscribe * 2.0,
            publish_rate: args.rate_limit_publish,
            publish_burst: args.rate_limit_publish * 2.0,
            service_rate: args.rate_limit_service,
            service_burst: args.rate_limit_service * 2.0,
        }
    };

    let config = ServerConfig {
        host: args.host,
        port: args.port,
        ros_distro: args.ros_distro,
        node_name: "roskit_bridge".to_string(),
        idle_timeout_secs: args.idle_timeout,
        heartbeat_interval_secs: args.heartbeat_interval,
        max_queue_size: args.max_queue_size,
        max_message_size: args.max_message_size,
        tls_config,
        auth_config,
        rate_limit_config,
    };

    let server = Arc::new(Server::new(config)?);

    // Handle shutdown signals
    let server_clone = Arc::clone(&server);
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.ok();
        info!("Received shutdown signal");
        server_clone.shutdown();
    });

    // Run the server
    server.run().await
}
