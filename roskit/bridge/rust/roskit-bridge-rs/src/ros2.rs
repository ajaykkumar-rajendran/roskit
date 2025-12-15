//! ROS2 integration using rclrs (official ROS2 Rust client)
//!
//! This module provides thread-safe subscription, publishing, and service call functionality.
//! All ROS2 operations run on a dedicated thread with message passing to the async runtime.

use bytes::Bytes;
use crossbeam_channel::{bounded, Receiver, Sender};
use serde_json::Value as JsonValue;
#[cfg(feature = "ros2")]
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;
use tracing::{debug, error, info, warn};

use crate::protocol::{Encoding, TopicInfo};

/// Message from ROS2 subscription to send to WebSocket client
#[derive(Debug, Clone)]
pub struct RosMessage {
    pub channel_id: u32,
    pub payload: Bytes,
    pub encoding: Encoding,
}

/// Command sent to the ROS2 thread
#[derive(Debug)]
enum Ros2Command {
    Subscribe {
        channel_id: u32,
        topic: String,
        msg_type: String,
        throttle_ms: Option<u32>,
        response_tx: Sender<Result<(), String>>,
    },
    Unsubscribe {
        channel_id: u32,
    },
    Publish {
        topic: String,
        msg_type: String,
        data: JsonValue,
        response_tx: Sender<Result<(), String>>,
    },
    ServiceCall {
        service: String,
        service_type: String,
        request: JsonValue,
        timeout: Duration,
        response_tx: Sender<Result<JsonValue, String>>,
    },
    GetTopics {
        response_tx: Sender<Vec<TopicInfo>>,
    },
    GetServices {
        response_tx: Sender<Vec<(String, Vec<String>)>>,
    },
    GetNodes {
        response_tx: Sender<Vec<String>>,
    },
    Shutdown,
}

/// Subscription handle stored in the ROS2 thread
#[cfg(feature = "ros2")]
struct SubscriptionHandle {
    topic: String,
    #[allow(dead_code)]
    msg_type: String,
    active: Arc<AtomicBool>,
}

/// ROS2 context that manages all ROS2 operations on a dedicated thread
pub struct Ros2Context {
    command_tx: Sender<Ros2Command>,
    message_rx: Receiver<RosMessage>,
    shutdown: Arc<AtomicBool>,
    thread_handle: Option<JoinHandle<()>>,
}

// Implement Send + Sync for Ros2Context since it only holds thread-safe types
unsafe impl Send for Ros2Context {}
unsafe impl Sync for Ros2Context {}

impl Ros2Context {
    /// Create a new ROS2 context
    pub fn new(node_name: &str) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let (command_tx, command_rx) = bounded::<Ros2Command>(1024);
        let (message_tx, message_rx) = bounded::<RosMessage>(4096);
        let shutdown = Arc::new(AtomicBool::new(false));
        let shutdown_clone = Arc::clone(&shutdown);
        let node_name = node_name.to_string();

        // Spawn dedicated ROS2 thread
        let thread_handle = thread::spawn(move || {
            if let Err(e) = run_ros2_thread(&node_name, command_rx, message_tx, shutdown_clone) {
                error!("ROS2 thread error: {}", e);
            }
        });

        Ok(Self {
            command_tx,
            message_rx,
            shutdown,
            thread_handle: Some(thread_handle),
        })
    }

    /// Get the message receiver for ROS2 messages
    pub fn message_receiver(&self) -> Receiver<RosMessage> {
        self.message_rx.clone()
    }

    /// Subscribe to a ROS2 topic
    pub async fn subscribe(
        &self,
        channel_id: u32,
        topic: &str,
        msg_type: &str,
        throttle_ms: Option<u32>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let (response_tx, response_rx) = bounded(1);

        self.command_tx
            .send(Ros2Command::Subscribe {
                channel_id,
                topic: topic.to_string(),
                msg_type: msg_type.to_string(),
                throttle_ms,
                response_tx,
            })
            .map_err(|e| format!("Failed to send subscribe command: {}", e))?;

        match response_rx.recv_timeout(Duration::from_secs(5)) {
            Ok(Ok(())) => Ok(()),
            Ok(Err(e)) => Err(e.into()),
            Err(_) => Err("Subscribe timeout".into()),
        }
    }

    /// Unsubscribe from a topic
    pub fn unsubscribe(&self, channel_id: u32) {
        let _ = self.command_tx.send(Ros2Command::Unsubscribe { channel_id });
    }

    /// Publish a message to a topic
    pub async fn publish(
        &self,
        topic: &str,
        msg_type: &str,
        data: &JsonValue,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let (response_tx, response_rx) = bounded(1);

        self.command_tx
            .send(Ros2Command::Publish {
                topic: topic.to_string(),
                msg_type: msg_type.to_string(),
                data: data.clone(),
                response_tx,
            })
            .map_err(|e| format!("Failed to send publish command: {}", e))?;

        match response_rx.recv_timeout(Duration::from_secs(5)) {
            Ok(Ok(())) => Ok(()),
            Ok(Err(e)) => Err(e.into()),
            Err(_) => Err("Publish timeout".into()),
        }
    }

    /// Call a ROS2 service
    pub async fn call_service(
        &self,
        service: &str,
        service_type: &str,
        request: &JsonValue,
        timeout: Duration,
    ) -> Result<JsonValue, Box<dyn std::error::Error + Send + Sync>> {
        let (response_tx, response_rx) = bounded(1);

        self.command_tx
            .send(Ros2Command::ServiceCall {
                service: service.to_string(),
                service_type: service_type.to_string(),
                request: request.clone(),
                timeout,
                response_tx,
            })
            .map_err(|e| format!("Failed to send service call command: {}", e))?;

        match response_rx.recv_timeout(timeout + Duration::from_secs(1)) {
            Ok(Ok(response)) => Ok(response),
            Ok(Err(e)) => Err(e.into()),
            Err(_) => Err("Service call timeout".into()),
        }
    }

    /// Get list of available topics
    pub fn get_topics(&self) -> Vec<TopicInfo> {
        let (response_tx, response_rx) = bounded(1);

        if self
            .command_tx
            .send(Ros2Command::GetTopics { response_tx })
            .is_err()
        {
            return Vec::new();
        }

        response_rx
            .recv_timeout(Duration::from_secs(2))
            .unwrap_or_default()
    }

    /// Get list of available services
    pub fn get_services(&self) -> Vec<(String, Vec<String>)> {
        let (response_tx, response_rx) = bounded(1);

        if self
            .command_tx
            .send(Ros2Command::GetServices { response_tx })
            .is_err()
        {
            return Vec::new();
        }

        response_rx
            .recv_timeout(Duration::from_secs(2))
            .unwrap_or_default()
    }

    /// Get list of available nodes
    pub fn get_nodes(&self) -> Vec<String> {
        let (response_tx, response_rx) = bounded(1);

        if self
            .command_tx
            .send(Ros2Command::GetNodes { response_tx })
            .is_err()
        {
            return Vec::new();
        }

        response_rx
            .recv_timeout(Duration::from_secs(2))
            .unwrap_or_default()
    }
}

impl Drop for Ros2Context {
    fn drop(&mut self) {
        info!("Shutting down ROS2 context");
        self.shutdown.store(true, Ordering::SeqCst);
        let _ = self.command_tx.send(Ros2Command::Shutdown);

        if let Some(handle) = self.thread_handle.take() {
            let _ = handle.join();
        }
    }
}

// ============================================================================
// ROS2 Thread Implementation (Stub - enable ros2 feature for full impl)
// ============================================================================

fn run_ros2_thread(
    node_name: &str,
    command_rx: Receiver<Ros2Command>,
    message_tx: Sender<RosMessage>,
    shutdown: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    info!("Starting ROS2 thread for node '{}'", node_name);

    #[cfg(feature = "ros2")]
    {
        run_ros2_thread_impl(node_name, command_rx, message_tx, shutdown)
    }

    #[cfg(not(feature = "ros2"))]
    {
        run_ros2_thread_stub(node_name, command_rx, message_tx, shutdown)
    }
}

/// Stub implementation when ros2 feature is disabled
#[cfg(not(feature = "ros2"))]
fn run_ros2_thread_stub(
    node_name: &str,
    command_rx: Receiver<Ros2Command>,
    _message_tx: Sender<RosMessage>,
    shutdown: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    warn!(
        "ROS2 feature not enabled. Running in stub mode. Node name: {}",
        node_name
    );
    warn!("Build with --features ros2 to enable ROS2 integration");

    // Process commands but return errors
    while !shutdown.load(Ordering::SeqCst) {
        match command_rx.recv_timeout(Duration::from_millis(100)) {
            Ok(cmd) => match cmd {
                Ros2Command::Subscribe { response_tx, topic, .. } => {
                    warn!("Stub: Cannot subscribe to {} - ROS2 not enabled", topic);
                    let _ = response_tx.send(Err("ROS2 feature not enabled. Build with --features ros2".to_string()));
                }
                Ros2Command::Publish { response_tx, topic, .. } => {
                    warn!("Stub: Cannot publish to {} - ROS2 not enabled", topic);
                    let _ = response_tx.send(Err("ROS2 feature not enabled. Build with --features ros2".to_string()));
                }
                Ros2Command::ServiceCall { response_tx, service, .. } => {
                    warn!("Stub: Cannot call service {} - ROS2 not enabled", service);
                    let _ = response_tx.send(Err("ROS2 feature not enabled. Build with --features ros2".to_string()));
                }
                Ros2Command::GetTopics { response_tx } => {
                    // Return empty list in stub mode
                    let _ = response_tx.send(Vec::new());
                }
                Ros2Command::GetServices { response_tx } => {
                    let _ = response_tx.send(Vec::new());
                }
                Ros2Command::GetNodes { response_tx } => {
                    let _ = response_tx.send(Vec::new());
                }
                Ros2Command::Unsubscribe { channel_id } => {
                    debug!("Stub: Unsubscribe channel {}", channel_id);
                }
                Ros2Command::Shutdown => {
                    info!("Stub: Received shutdown command");
                    break;
                }
            },
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                // Normal timeout, continue
            }
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                info!("Command channel disconnected");
                break;
            }
        }
    }

    info!("ROS2 stub thread shutting down");
    Ok(())
}

// ============================================================================
// Full ROS2 Implementation (when ros2 feature is enabled)
// ============================================================================

#[cfg(feature = "ros2")]
fn run_ros2_thread_impl(
    node_name: &str,
    command_rx: Receiver<Ros2Command>,
    message_tx: Sender<RosMessage>,
    shutdown: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    use rclrs::{Context, Node};

    // Initialize ROS2 context
    let context = Context::new(std::env::args())?;
    let node = Node::new(&context, node_name)?;

    let mut subscriptions: HashMap<u32, SubscriptionHandle> = HashMap::new();

    info!("ROS2 node '{}' initialized with rclrs", node_name);

    // Main loop
    while !shutdown.load(Ordering::SeqCst) {
        // Process commands with timeout
        match command_rx.recv_timeout(Duration::from_millis(10)) {
            Ok(cmd) => {
                process_ros2_command(cmd, &node, &mut subscriptions, &message_tx)?;
            }
            Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                // Normal timeout, continue spinning
            }
            Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                info!("Command channel disconnected, shutting down ROS2 thread");
                break;
            }
        }

        // Spin once to process ROS2 callbacks
        if let Err(e) = rclrs::spin_once(&node, Some(Duration::from_millis(1))) {
            debug!("Spin error (may be normal): {}", e);
        }
    }

    info!("ROS2 thread shutting down");
    Ok(())
}

#[cfg(feature = "ros2")]
fn process_ros2_command(
    cmd: Ros2Command,
    node: &rclrs::Node,
    subscriptions: &mut HashMap<u32, SubscriptionHandle>,
    message_tx: &Sender<RosMessage>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    match cmd {
        Ros2Command::Subscribe {
            channel_id,
            topic,
            msg_type,
            throttle_ms,
            response_tx,
        } => {
            info!(
                "Subscribing to {} (type: {}) for channel {}",
                topic, msg_type, channel_id
            );

            let result = create_ros2_subscription(
                node,
                channel_id,
                &topic,
                &msg_type,
                throttle_ms,
                message_tx.clone(),
            );

            match result {
                Ok(handle) => {
                    subscriptions.insert(channel_id, handle);
                    let _ = response_tx.send(Ok(()));
                }
                Err(e) => {
                    error!("Failed to subscribe: {}", e);
                    let _ = response_tx.send(Err(e.to_string()));
                }
            }
        }

        Ros2Command::Unsubscribe { channel_id } => {
            if let Some(handle) = subscriptions.remove(&channel_id) {
                handle.active.store(false, Ordering::SeqCst);
                info!(
                    "Unsubscribed from {} (channel {})",
                    handle.topic, channel_id
                );
            }
        }

        Ros2Command::Publish {
            topic,
            msg_type,
            data,
            response_tx,
        } => {
            let result = publish_ros2_message(node, &topic, &msg_type, &data);
            let _ = response_tx.send(result.map_err(|e| e.to_string()));
        }

        Ros2Command::ServiceCall {
            service,
            service_type,
            request,
            timeout,
            response_tx,
        } => {
            let result = call_ros2_service(node, &service, &service_type, &request, timeout);
            let _ = response_tx.send(result.map_err(|e| e.to_string()));
        }

        Ros2Command::GetTopics { response_tx } => {
            let topics = match node.get_topic_names_and_types() {
                Ok(t) => t
                    .into_iter()
                    .flat_map(|(name, types)| {
                        types.into_iter().map(move |msg_type| TopicInfo {
                            name: name.clone(),
                            msg_type,
                        })
                    })
                    .collect(),
                Err(e) => {
                    error!("Failed to get topics: {}", e);
                    Vec::new()
                }
            };
            let _ = response_tx.send(topics);
        }

        Ros2Command::GetServices { response_tx } => {
            let services = match node.get_service_names_and_types() {
                Ok(s) => s,
                Err(e) => {
                    error!("Failed to get services: {}", e);
                    Vec::new()
                }
            };
            let _ = response_tx.send(services);
        }

        Ros2Command::GetNodes { response_tx } => {
            let nodes = match node.get_node_names() {
                Ok(n) => n,
                Err(e) => {
                    error!("Failed to get nodes: {}", e);
                    Vec::new()
                }
            };
            let _ = response_tx.send(nodes);
        }

        Ros2Command::Shutdown => {
            info!("Received shutdown command");
        }
    }

    Ok(())
}

#[cfg(feature = "ros2")]
fn create_ros2_subscription(
    node: &rclrs::Node,
    channel_id: u32,
    topic: &str,
    msg_type: &str,
    _throttle_ms: Option<u32>,
    message_tx: Sender<RosMessage>,
) -> Result<SubscriptionHandle, Box<dyn std::error::Error + Send + Sync>> {
    use crate::protocol::encode_cbor;

    let active = Arc::new(AtomicBool::new(true));
    let active_clone = Arc::clone(&active);

    match msg_type {
        "std_msgs/msg/String" => {
            let tx = message_tx.clone();
            let _sub = node.create_subscription::<std_msgs::msg::String, _>(
                topic,
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: std_msgs::msg::String| {
                    if !active_clone.load(Ordering::SeqCst) {
                        return;
                    }
                    if let Ok(payload) = encode_cbor(&serde_json::json!({"data": msg.data}))
                    {
                        let _ = tx.send(RosMessage {
                            channel_id,
                            payload,
                            encoding: Encoding::Cbor,
                        });
                    }
                },
            )?;
        }

        "geometry_msgs/msg/Twist" => {
            let tx = message_tx.clone();
            let active_clone = Arc::clone(&active);
            let _sub = node.create_subscription::<geometry_msgs::msg::Twist, _>(
                topic,
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: geometry_msgs::msg::Twist| {
                    if !active_clone.load(Ordering::SeqCst) {
                        return;
                    }
                    let json = serde_json::json!({
                        "linear": {"x": msg.linear.x, "y": msg.linear.y, "z": msg.linear.z},
                        "angular": {"x": msg.angular.x, "y": msg.angular.y, "z": msg.angular.z}
                    });
                    if let Ok(payload) = encode_cbor(&json) {
                        let _ = tx.send(RosMessage {
                            channel_id,
                            payload,
                            encoding: Encoding::Cbor,
                        });
                    }
                },
            )?;
        }

        "nav_msgs/msg/Odometry" => {
            let tx = message_tx.clone();
            let active_clone = Arc::clone(&active);
            let _sub = node.create_subscription::<nav_msgs::msg::Odometry, _>(
                topic,
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: nav_msgs::msg::Odometry| {
                    if !active_clone.load(Ordering::SeqCst) {
                        return;
                    }
                    let json = serde_json::json!({
                        "header": {
                            "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                            "frame_id": msg.header.frame_id
                        },
                        "child_frame_id": msg.child_frame_id,
                        "pose": {
                            "pose": {
                                "position": {"x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y, "z": msg.pose.pose.position.z},
                                "orientation": {"x": msg.pose.pose.orientation.x, "y": msg.pose.pose.orientation.y, "z": msg.pose.pose.orientation.z, "w": msg.pose.pose.orientation.w}
                            }
                        },
                        "twist": {
                            "twist": {
                                "linear": {"x": msg.twist.twist.linear.x, "y": msg.twist.twist.linear.y, "z": msg.twist.twist.linear.z},
                                "angular": {"x": msg.twist.twist.angular.x, "y": msg.twist.twist.angular.y, "z": msg.twist.twist.angular.z}
                            }
                        }
                    });
                    if let Ok(payload) = encode_cbor(&json) {
                        let _ = tx.send(RosMessage {
                            channel_id,
                            payload,
                            encoding: Encoding::Cbor,
                        });
                    }
                },
            )?;
        }

        _ => {
            warn!("Unsupported message type: {}", msg_type);
            return Err(format!("Unsupported message type: {}", msg_type).into());
        }
    }

    Ok(SubscriptionHandle {
        topic: topic.to_string(),
        msg_type: msg_type.to_string(),
        active,
    })
}

#[cfg(feature = "ros2")]
fn publish_ros2_message(
    node: &rclrs::Node,
    topic: &str,
    msg_type: &str,
    data: &JsonValue,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    match msg_type {
        "std_msgs/msg/String" => {
            let publisher =
                node.create_publisher::<std_msgs::msg::String>(topic, rclrs::QOS_PROFILE_DEFAULT)?;
            let msg = std_msgs::msg::String {
                data: data
                    .get("data")
                    .and_then(|v| v.as_str())
                    .unwrap_or("")
                    .to_string(),
            };
            publisher.publish(msg)?;
        }

        "geometry_msgs/msg/Twist" => {
            let publisher =
                node.create_publisher::<geometry_msgs::msg::Twist>(topic, rclrs::QOS_PROFILE_DEFAULT)?;
            let linear = data.get("linear").unwrap_or(&JsonValue::Null);
            let angular = data.get("angular").unwrap_or(&JsonValue::Null);
            let msg = geometry_msgs::msg::Twist {
                linear: geometry_msgs::msg::Vector3 {
                    x: linear.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    y: linear.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    z: linear.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0),
                },
                angular: geometry_msgs::msg::Vector3 {
                    x: angular.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    y: angular.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    z: angular.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0),
                },
            };
            publisher.publish(msg)?;
        }

        _ => {
            return Err(format!("Unsupported message type for publish: {}", msg_type).into());
        }
    }

    Ok(())
}

#[cfg(feature = "ros2")]
fn call_ros2_service(
    _node: &rclrs::Node,
    service: &str,
    service_type: &str,
    _request: &JsonValue,
    _timeout: Duration,
) -> Result<JsonValue, Box<dyn std::error::Error + Send + Sync>> {
    // Service calls with rclrs require more setup
    // For now, return an error for unsupported services
    warn!(
        "Service call not yet implemented for {} ({})",
        service, service_type
    );
    Err(format!("Service calls not yet implemented: {}", service_type).into())
}
