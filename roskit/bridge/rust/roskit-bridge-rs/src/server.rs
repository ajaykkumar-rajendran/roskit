//! WebSocket server for RosKit Bridge

use crate::auth::{extract_token_from_request, AuthConfig, AuthManager};
use crate::protocol::{
    decode_cbor, encode_cbor, ChannelInfo, Encoding, ErrorResponse, Message, MessageType,
    NodeListResponse, ServerInfo, ServiceInfo, ServiceListResponse, SubscribeRequest,
    TopicListResponse, UnsubscribeRequest,
};
use crate::rate_limit::{ClientRateLimiters, RateLimitConfig};
use crate::ros2::{Ros2Context, RosMessage};
use crate::tls::TlsConfig;
use bytes::Bytes;
use futures_util::{SinkExt, StreamExt};
use parking_lot::RwLock;
use serde::Deserialize;
use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::io::{AsyncRead, AsyncWrite};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::broadcast;
use tokio::sync::mpsc;
use tokio_rustls::TlsAcceptor;
use tokio_tungstenite::{
    accept_hdr_async,
    tungstenite::{
        handshake::server::{Callback, ErrorResponse as WsErrorResponse, Request, Response},
        http::StatusCode,
        Message as WsMessage,
    },
    WebSocketStream,
};
use tracing::{debug, error, info, warn};

/// Channel state for a subscription
#[derive(Debug)]
pub struct Channel {
    pub id: u32,
    pub topic: String,
    pub msg_type: String,
    pub encoding: Encoding,
}

/// Client connection state
pub struct ClientState {
    pub addr: SocketAddr,
    pub channels: RwLock<HashMap<u32, Channel>>,
}

/// WebSocket handshake callback for authentication
struct AuthCallback {
    auth_manager: Arc<AuthManager>,
    addr: SocketAddr,
}

impl Callback for AuthCallback {
    fn on_request(
        self,
        request: &Request,
        response: Response,
    ) -> Result<Response, WsErrorResponse> {
        // Extract token from headers and query params
        let headers: Vec<(String, String)> = request
            .headers()
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_str().unwrap_or("").to_string()))
            .collect();

        let query_params = request.uri().query();
        let token = extract_token_from_request(&headers, query_params);

        // Validate authentication
        if self.auth_manager.is_enabled() {
            match token {
                Some(ref t) if self.auth_manager.validate_token(t) => {
                    debug!("Authenticated connection from {}", self.addr);
                    Ok(response)
                }
                Some(_) => {
                    warn!("Invalid token from {}", self.addr);
                    let mut err_response = WsErrorResponse::new(None);
                    *err_response.status_mut() = StatusCode::UNAUTHORIZED;
                    Err(err_response)
                }
                None => {
                    warn!("Missing authentication token from {}", self.addr);
                    let mut err_response = WsErrorResponse::new(None);
                    *err_response.status_mut() = StatusCode::UNAUTHORIZED;
                    Err(err_response)
                }
            }
        } else {
            // Auth disabled, allow all connections
            Ok(response)
        }
    }
}

/// Server configuration
#[derive(Debug, Clone)]
pub struct ServerConfig {
    pub host: String,
    pub port: u16,
    pub ros_distro: String,
    pub node_name: String,
    /// Idle timeout in seconds (0 = disabled)
    pub idle_timeout_secs: u64,
    /// Heartbeat interval in seconds (0 = disabled)
    pub heartbeat_interval_secs: u64,
    /// Maximum message queue size for backpressure
    pub max_queue_size: usize,
    /// Maximum message size in bytes (0 = unlimited)
    pub max_message_size: usize,
    /// TLS configuration (None = plain WebSocket)
    pub tls_config: Option<TlsConfig>,
    /// Authentication configuration
    pub auth_config: AuthConfig,
    /// Rate limiting configuration
    pub rate_limit_config: RateLimitConfig,
}

impl Default for ServerConfig {
    fn default() -> Self {
        Self {
            host: "0.0.0.0".to_string(),
            port: 9091,
            ros_distro: "humble".to_string(),
            node_name: "roskit_bridge".to_string(),
            idle_timeout_secs: 300, // 5 minutes
            heartbeat_interval_secs: 30,
            max_queue_size: 1024,
            max_message_size: 10 * 1024 * 1024, // 10 MB
            tls_config: None,
            auth_config: AuthConfig::default(),
            rate_limit_config: RateLimitConfig::default(),
        }
    }
}

/// Publish request payload
#[derive(Debug, Deserialize)]
struct PublishRequest {
    topic: String,
    msg_type: String,
    data: serde_json::Value,
}

/// Service call request payload
#[derive(Debug, Deserialize)]
struct ServiceCallRequest {
    service: String,
    service_type: String,
    request: serde_json::Value,
    #[serde(default)]
    request_id: Option<String>,
    #[serde(default = "default_timeout")]
    timeout_ms: u64,
}

fn default_timeout() -> u64 {
    5000
}

/// RosKit Bridge Server
pub struct Server {
    config: ServerConfig,
    next_channel_id: AtomicU32,
    shutdown_tx: broadcast::Sender<()>,
    ros2_context: Arc<Ros2Context>,
    tls_acceptor: Option<TlsAcceptor>,
    auth_manager: Arc<AuthManager>,
}

impl Server {
    /// Create a new server with ROS2 integration
    pub fn new(config: ServerConfig) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let (shutdown_tx, _) = broadcast::channel(1);
        let ros2_context = Arc::new(Ros2Context::new(&config.node_name)?);

        // Build TLS acceptor if configured
        let tls_acceptor = if let Some(ref tls_config) = config.tls_config {
            Some(tls_config.build_acceptor()?)
        } else {
            None
        };

        // Create auth manager
        let auth_manager = Arc::new(AuthManager::new(config.auth_config.clone()));

        Ok(Self {
            config,
            next_channel_id: AtomicU32::new(1),
            shutdown_tx,
            ros2_context,
            tls_acceptor,
            auth_manager,
        })
    }

    /// Get the next channel ID
    fn next_channel_id(&self) -> u32 {
        self.next_channel_id.fetch_add(1, Ordering::SeqCst)
    }

    /// Run the server
    pub async fn run(self: Arc<Self>) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let addr = format!("{}:{}", self.config.host, self.config.port);
        let listener = TcpListener::bind(&addr).await?;

        let protocol = if self.tls_acceptor.is_some() { "wss" } else { "ws" };
        info!("RosKit Bridge listening on {}://{}", protocol, addr);

        if self.auth_manager.is_enabled() {
            info!("Authentication is enabled");
        }

        // ROS2 context now manages its own dedicated thread internally
        // No need to spawn a separate thread here

        let mut shutdown_rx = self.shutdown_tx.subscribe();

        loop {
            tokio::select! {
                result = listener.accept() => {
                    match result {
                        Ok((stream, addr)) => {
                            let server = Arc::clone(&self);
                            tokio::spawn(async move {
                                if let Err(e) = server.handle_tcp_connection(stream, addr).await {
                                    error!("Error handling connection from {}: {}", addr, e);
                                }
                            });
                        }
                        Err(e) => {
                            error!("Failed to accept connection: {}", e);
                        }
                    }
                }
                _ = shutdown_rx.recv() => {
                    info!("Server shutting down");
                    break;
                }
            }
        }

        Ok(())
    }

    /// Handle TCP connection (with optional TLS upgrade)
    async fn handle_tcp_connection(
        &self,
        stream: TcpStream,
        addr: SocketAddr,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        if let Some(ref acceptor) = self.tls_acceptor {
            // TLS connection
            match acceptor.accept(stream).await {
                Ok(tls_stream) => {
                    self.handle_websocket_connection(tls_stream, addr).await
                }
                Err(e) => {
                    warn!("TLS handshake failed from {}: {}", addr, e);
                    Err(e.into())
                }
            }
        } else {
            // Plain TCP connection
            self.handle_websocket_connection(stream, addr).await
        }
    }

    /// Handle a WebSocket connection over any async stream
    async fn handle_websocket_connection<S>(
        &self,
        stream: S,
        addr: SocketAddr,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>>
    where
        S: AsyncRead + AsyncWrite + Unpin + Send + 'static,
    {
        // Create auth callback for WebSocket handshake
        let auth_callback = AuthCallback {
            auth_manager: Arc::clone(&self.auth_manager),
            addr,
        };

        // Accept WebSocket with authentication callback
        let ws_stream = match accept_hdr_async(stream, auth_callback).await {
            Ok(stream) => stream,
            Err(e) => {
                // Check if this was an authentication error (401)
                if e.to_string().contains("401") {
                    info!("Connection from {} rejected: authentication failed", addr);
                } else {
                    warn!("WebSocket handshake failed from {}: {}", addr, e);
                }
                return Err(e.into());
            }
        };

        self.handle_ws_stream(ws_stream, addr).await
    }

    /// Handle the WebSocket stream
    async fn handle_ws_stream<S>(
        &self,
        ws_stream: WebSocketStream<S>,
        addr: SocketAddr,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>>
    where
        S: AsyncRead + AsyncWrite + Unpin + Send + 'static,
    {
        let (mut ws_sender, mut ws_receiver) = ws_stream.split();

        info!("New WebSocket connection from {}", addr);

        let client_state = Arc::new(ClientState {
            addr,
            channels: RwLock::new(HashMap::new()),
        });

        // Send server info with protocol version and capabilities
        let server_info = ServerInfo {
            name: "roskit-bridge-rs".to_string(),
            version: env!("CARGO_PKG_VERSION").to_string(),
            protocol_version: crate::protocol::PROTOCOL_VERSION,
            ros_distro: self.config.ros_distro.clone(),
            capabilities: vec![
                "subscribe".to_string(),
                "publish".to_string(),
                "topics".to_string(),
                "services".to_string(),
                "service_list".to_string(),
                "node_list".to_string(),
                "parameters".to_string(),
            ],
            encodings: vec![
                "raw".to_string(),
                "cbor".to_string(),
                "json".to_string(),
                "png".to_string(),
                "jpeg".to_string(),
                "binary".to_string(),
            ],
        };

        let payload = encode_cbor(&server_info)?;
        let msg = Message::new(MessageType::ServerInfo, 0, payload, Encoding::Cbor);
        ws_sender
            .send(WsMessage::Binary(msg.encode().to_vec()))
            .await?;

        // Backpressure channel for outbound messages
        let max_queue = self.config.max_queue_size;
        let (tx, mut rx) = mpsc::channel::<WsMessage>(max_queue);

        // Channel for ROS messages - bridge from crossbeam to tokio
        let (ros_tx, mut ros_rx) = mpsc::channel::<RosMessage>(max_queue);

        // Spawn a task to forward ROS2 messages from crossbeam channel to tokio channel
        let ros2_message_rx = self.ros2_context.message_receiver();
        let ros2_forwarder = tokio::task::spawn_blocking(move || {
            while let Ok(msg) = ros2_message_rx.recv() {
                if ros_tx.blocking_send(msg).is_err() {
                    // Channel closed, stop forwarding
                    break;
                }
            }
        });

        // Heartbeat interval setup
        let heartbeat_interval = if self.config.heartbeat_interval_secs > 0 {
            Some(Duration::from_secs(self.config.heartbeat_interval_secs))
        } else {
            None
        };

        // Idle timeout setup
        let idle_timeout = if self.config.idle_timeout_secs > 0 {
            Some(Duration::from_secs(self.config.idle_timeout_secs))
        } else {
            None
        };

        // Sender task - handles WebSocket messages, ROS messages, and heartbeats
        // Run inline instead of spawning to avoid Send requirements
        let sender_future = async move {
            let mut heartbeat_timer = heartbeat_interval.map(tokio::time::interval);
            if let Some(ref mut timer) = heartbeat_timer {
                timer.tick().await; // Skip first tick
            }

            loop {
                tokio::select! {
                    Some(msg) = rx.recv() => {
                        if let Err(e) = ws_sender.send(msg).await {
                            warn!("Failed to send message to {}: {}", addr, e);
                            break;
                        }
                    }
                    Some(ros_msg) = ros_rx.recv() => {
                        // Convert ROS message to WebSocket message
                        let ws_msg = Message::new(
                            MessageType::Message,
                            ros_msg.channel_id,
                            ros_msg.payload,
                            ros_msg.encoding,
                        );
                        if let Err(e) = ws_sender.send(WsMessage::Binary(ws_msg.encode().to_vec())).await {
                            warn!("Failed to send ROS message to {}: {}", addr, e);
                            break;
                        }
                    }
                    _ = async {
                        if let Some(ref mut timer) = heartbeat_timer {
                            timer.tick().await
                        } else {
                            std::future::pending::<()>().await;
                            unreachable!()
                        }
                    } => {
                        // Send WebSocket ping for heartbeat
                        if let Err(e) = ws_sender.send(WsMessage::Ping(vec![])).await {
                            warn!("Failed to send heartbeat ping to {}: {}", addr, e);
                            break;
                        }
                        debug!("Sent heartbeat ping to {}", addr);
                    }
                    else => break,
                }
            }
        };

        // Use tokio::spawn with the Send-compatible future
        let sender_handle = tokio::spawn(sender_future);

        // Track last activity for idle timeout
        let mut last_activity = Instant::now();
        let max_message_size = self.config.max_message_size;

        // Create per-client rate limiters
        let rate_limiters = ClientRateLimiters::new(&self.config.rate_limit_config);

        // Receiver loop with idle timeout
        loop {
            let timeout_duration = idle_timeout.unwrap_or(Duration::from_secs(3600)); // Default 1 hour if disabled

            tokio::select! {
                result = ws_receiver.next() => {
                    match result {
                        Some(Ok(WsMessage::Binary(data))) => {
                            last_activity = Instant::now();

                            // Check message size
                            if max_message_size > 0 && data.len() > max_message_size {
                                warn!(
                                    "Message from {} exceeds max size ({} > {}), rejecting",
                                    addr, data.len(), max_message_size
                                );
                                Self::send_error_to(&tx, &format!(
                                    "Message too large: {} bytes exceeds limit of {} bytes",
                                    data.len(), max_message_size
                                )).await?;
                                continue;
                            }

                            if let Err(e) = self
                                .handle_message(&data, &client_state, &tx, &rate_limiters)
                                .await
                            {
                                warn!("Error handling message from {}: {}", addr, e);
                            }
                        }
                        Some(Ok(WsMessage::Close(_))) => {
                            info!("Client {} disconnected", addr);
                            break;
                        }
                        Some(Ok(WsMessage::Ping(data))) => {
                            last_activity = Instant::now();
                            if let Err(e) = tx.send(WsMessage::Pong(data)).await {
                                warn!("Failed to send pong: {}", e);
                            }
                        }
                        Some(Ok(WsMessage::Pong(_))) => {
                            // Received pong response to our heartbeat ping
                            last_activity = Instant::now();
                            debug!("Received heartbeat pong from {}", addr);
                        }
                        Some(Ok(WsMessage::Text(_))) => {
                            // Text messages not supported, ignore
                            last_activity = Instant::now();
                            debug!("Ignoring text message from {}", addr);
                        }
                        Some(Ok(WsMessage::Frame(_))) => {
                            // Raw frames, ignore
                        }
                        Some(Err(e)) => {
                            error!("WebSocket error from {}: {}", addr, e);
                            break;
                        }
                        None => {
                            info!("WebSocket stream ended for {}", addr);
                            break;
                        }
                    }
                }
                _ = tokio::time::sleep(timeout_duration) => {
                    // Check if idle timeout exceeded
                    if idle_timeout.is_some() && last_activity.elapsed() > timeout_duration {
                        warn!(
                            "Client {} exceeded idle timeout ({:?}), disconnecting",
                            addr, timeout_duration
                        );
                        break;
                    }
                }
            }
        }

        // Cleanup: drop sender channels to stop sender task
        drop(tx);
        let _ = sender_handle.await;
        ros2_forwarder.abort();

        // Cleanup client channels - unsubscribe from ROS topics
        let channels = client_state.channels.read();
        for (id, channel) in channels.iter() {
            debug!("Cleaning up channel {} for topic {}", id, channel.topic);
            self.ros2_context.unsubscribe(*id);
        }

        info!("Connection cleanup complete for {}", addr);

        Ok(())
    }

    /// Handle a binary message from a client
    async fn handle_message(
        &self,
        data: &[u8],
        client_state: &Arc<ClientState>,
        ws_sender: &mpsc::Sender<WsMessage>,
        rate_limiters: &ClientRateLimiters,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let message = Message::parse(data)?;

        match message.header.message_type {
            MessageType::Subscribe => {
                // Check rate limit for subscribe
                if !rate_limiters.subscribe.allow() {
                    warn!("Rate limit exceeded for subscribe from {}", client_state.addr);
                    Self::send_error_to(ws_sender, "Rate limit exceeded for subscribe").await?;
                    return Ok(());
                }
                self.handle_subscribe(&message, client_state, ws_sender)
                    .await?;
            }
            MessageType::Unsubscribe => {
                self.handle_unsubscribe(&message, client_state).await?;
            }
            MessageType::TopicList => {
                self.handle_topic_list(ws_sender).await?;
            }
            MessageType::Publish => {
                // Check rate limit for publish
                if !rate_limiters.publish.allow() {
                    warn!("Rate limit exceeded for publish from {}", client_state.addr);
                    Self::send_error_to(ws_sender, "Rate limit exceeded for publish").await?;
                    return Ok(());
                }
                self.handle_publish(&message, ws_sender).await?;
            }
            MessageType::ServiceCall => {
                // Check rate limit for service calls
                if !rate_limiters.service.allow() {
                    warn!("Rate limit exceeded for service call from {}", client_state.addr);
                    Self::send_error_to(ws_sender, "Rate limit exceeded for service call").await?;
                    return Ok(());
                }
                self.handle_service_call(&message, ws_sender).await?;
            }
            MessageType::ServiceList => {
                self.handle_service_list(ws_sender).await?;
            }
            MessageType::NodeList => {
                self.handle_node_list(ws_sender).await?;
            }
            MessageType::Ping => {
                let pong = Message::new(MessageType::Pong, 0, Bytes::new(), Encoding::Raw);
                ws_sender
                    .send(WsMessage::Binary(pong.encode().to_vec()))
                    .await?;
            }
            _ => {
                warn!(
                    "Unhandled message type: {:?}",
                    message.header.message_type
                );
            }
        }

        Ok(())
    }

    /// Handle subscribe request
    async fn handle_subscribe(
        &self,
        message: &Message,
        client_state: &Arc<ClientState>,
        ws_sender: &mpsc::Sender<WsMessage>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let request: SubscribeRequest = decode_cbor(&message.payload)?;
        info!("Subscribe request for topic: {}", request.topic);

        // Generate channel ID
        let channel_id = self.next_channel_id();

        // Determine encoding based on message type
        let msg_type = request.msg_type.clone().unwrap_or_default();
        let encoding = self.get_encoding_for_type(&msg_type);

        // Create channel
        let channel = Channel {
            id: channel_id,
            topic: request.topic.clone(),
            msg_type: msg_type.clone(),
            encoding,
        };

        // Store channel
        client_state.channels.write().insert(channel_id, channel);

        // Send channel info response
        let channel_info = ChannelInfo {
            channel_id,
            topic: request.topic.clone(),
            msg_type: msg_type.clone(),
            encoding: encoding.into(),
        };

        let payload = encode_cbor(&channel_info)?;
        let response = Message::new(MessageType::ChannelInfo, channel_id, payload, Encoding::Cbor);
        ws_sender
            .send(WsMessage::Binary(response.encode().to_vec()))
            .await?;

        // Subscribe to ROS topic
        if let Err(e) = self
            .ros2_context
            .subscribe(
                channel_id,
                &request.topic,
                &msg_type,
                request.throttle_ms,
            )
            .await
        {
            error!(
                "Failed to subscribe to ROS topic {}: {}",
                request.topic, e
            );
            Self::send_error_to(ws_sender, &format!("Failed to subscribe: {}", e)).await?;
        }

        Ok(())
    }

    /// Handle unsubscribe request
    async fn handle_unsubscribe(
        &self,
        message: &Message,
        client_state: &Arc<ClientState>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let request: UnsubscribeRequest = decode_cbor(&message.payload)?;
        info!("Unsubscribe request for channel: {}", request.channel_id);

        client_state.channels.write().remove(&request.channel_id);
        self.ros2_context.unsubscribe(request.channel_id);

        Ok(())
    }

    /// Handle topic list request
    async fn handle_topic_list(
        &self,
        ws_sender: &mpsc::Sender<WsMessage>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let topics = self.ros2_context.get_topics();

        let response = TopicListResponse { topics };
        let payload = encode_cbor(&response)?;
        let msg = Message::new(
            MessageType::TopicListResponse,
            0,
            payload,
            Encoding::Cbor,
        );
        ws_sender
            .send(WsMessage::Binary(msg.encode().to_vec()))
            .await?;

        Ok(())
    }

    /// Handle service list request
    async fn handle_service_list(
        &self,
        ws_sender: &mpsc::Sender<WsMessage>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let services_raw = self.ros2_context.get_services();

        let services: Vec<ServiceInfo> = services_raw
            .into_iter()
            .flat_map(|(name, types)| {
                types.into_iter().map(move |service_type| ServiceInfo {
                    name: name.clone(),
                    service_type,
                })
            })
            .collect();

        let response = ServiceListResponse { services };
        let payload = encode_cbor(&response)?;
        let msg = Message::new(
            MessageType::ServiceListResponse,
            0,
            payload,
            Encoding::Cbor,
        );
        ws_sender
            .send(WsMessage::Binary(msg.encode().to_vec()))
            .await?;

        Ok(())
    }

    /// Handle node list request
    async fn handle_node_list(
        &self,
        ws_sender: &mpsc::Sender<WsMessage>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let nodes = self.ros2_context.get_nodes();

        let response = NodeListResponse { nodes };
        let payload = encode_cbor(&response)?;
        let msg = Message::new(
            MessageType::NodeListResponse,
            0,
            payload,
            Encoding::Cbor,
        );
        ws_sender
            .send(WsMessage::Binary(msg.encode().to_vec()))
            .await?;

        Ok(())
    }

    /// Handle publish request
    async fn handle_publish(
        &self,
        message: &Message,
        ws_sender: &mpsc::Sender<WsMessage>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let request: PublishRequest = decode_cbor(&message.payload)?;
        info!("Publish request for topic: {}", request.topic);

        match self
            .ros2_context
            .publish(&request.topic, &request.msg_type, &request.data)
            .await
        {
            Ok(()) => {
                debug!("Published message to {}", request.topic);
            }
            Err(e) => {
                error!("Failed to publish to {}: {}", request.topic, e);
                Self::send_error_to(ws_sender, &format!("Publish failed: {}", e)).await?;
            }
        }

        Ok(())
    }

    /// Handle service call request
    async fn handle_service_call(
        &self,
        message: &Message,
        ws_sender: &mpsc::Sender<WsMessage>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let request: ServiceCallRequest = decode_cbor(&message.payload)?;
        info!("Service call request for: {}", request.service);

        let timeout = Duration::from_millis(request.timeout_ms);

        match self
            .ros2_context
            .call_service(
                &request.service,
                &request.service_type,
                &request.request,
                timeout,
            )
            .await
        {
            Ok(response) => {
                let mut resp_payload = serde_json::json!({
                    "response": response,
                });
                if let Some(req_id) = &request.request_id {
                    resp_payload["request_id"] = serde_json::Value::String(req_id.clone());
                }

                let payload = encode_cbor(&resp_payload)?;
                let resp_msg =
                    Message::new(MessageType::ServiceResponse, 0, payload, Encoding::Cbor);
                ws_sender
                    .send(WsMessage::Binary(resp_msg.encode().to_vec()))
                    .await?;
            }
            Err(e) => {
                error!("Service call failed for {}: {}", request.service, e);
                let mut error_payload = serde_json::json!({
                    "error": format!("Service call failed: {}", e),
                });
                if let Some(req_id) = &request.request_id {
                    error_payload["request_id"] = serde_json::Value::String(req_id.clone());
                }

                let payload = encode_cbor(&error_payload)?;
                let resp_msg =
                    Message::new(MessageType::ServiceResponse, 0, payload, Encoding::Cbor);
                ws_sender
                    .send(WsMessage::Binary(resp_msg.encode().to_vec()))
                    .await?;
            }
        }

        Ok(())
    }

    /// Determine best encoding for a message type
    fn get_encoding_for_type(&self, msg_type: &str) -> Encoding {
        if msg_type.contains("OccupancyGrid") {
            Encoding::Png
        } else if msg_type.contains("LaserScan") {
            Encoding::Binary
        } else if msg_type.contains("Image") || msg_type.contains("CompressedImage") {
            Encoding::Jpeg
        } else {
            Encoding::Cbor
        }
    }

    /// Send an error response
    async fn send_error_to(
        ws_sender: &mpsc::Sender<WsMessage>,
        error_msg: &str,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let error = ErrorResponse {
            error: error_msg.to_string(),
        };
        let payload = encode_cbor(&error)?;
        let msg = Message::new(MessageType::Error, 0, payload, Encoding::Cbor);
        ws_sender
            .send(WsMessage::Binary(msg.encode().to_vec()))
            .await?;
        Ok(())
    }

    /// Shutdown the server
    pub fn shutdown(&self) {
        let _ = self.shutdown_tx.send(());
    }
}
