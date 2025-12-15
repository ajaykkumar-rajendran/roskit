//! ROS2 integration stub
//!
//! This module provides stub implementations for ROS2 operations.
//! The WebSocket server runs but ROS2 operations return errors.
//!
//! For full ROS2 integration, use the Python bridge or integrate with ros2_rust separately.

use bytes::Bytes;
use crossbeam_channel::{bounded, Receiver, Sender};
use serde_json::Value as JsonValue;
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
        #[allow(dead_code)]
        throttle_ms: Option<u32>,
        response_tx: Sender<Result<(), String>>,
    },
    Unsubscribe {
        channel_id: u32,
    },
    Publish {
        topic: String,
        #[allow(dead_code)]
        msg_type: String,
        #[allow(dead_code)]
        data: JsonValue,
        response_tx: Sender<Result<(), String>>,
    },
    ServiceCall {
        service: String,
        #[allow(dead_code)]
        service_type: String,
        #[allow(dead_code)]
        request: JsonValue,
        #[allow(dead_code)]
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
            if let Err(e) = run_ros2_thread_stub(&node_name, command_rx, message_tx, shutdown_clone)
            {
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

/// Stub implementation - ROS2 operations return errors
fn run_ros2_thread_stub(
    node_name: &str,
    command_rx: Receiver<Ros2Command>,
    _message_tx: Sender<RosMessage>,
    shutdown: Arc<AtomicBool>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    warn!(
        "ROS2 stub mode active. Node name: '{}'. ROS2 operations will return errors.",
        node_name
    );
    warn!("For full ROS2 support, use the Python bridge (roskit_bridge)");

    // Process commands but return errors for ROS2 operations
    while !shutdown.load(Ordering::SeqCst) {
        match command_rx.recv_timeout(Duration::from_millis(100)) {
            Ok(cmd) => match cmd {
                Ros2Command::Subscribe {
                    response_tx, topic, ..
                } => {
                    warn!("Stub: Cannot subscribe to '{}' - ROS2 not available", topic);
                    let _ = response_tx.send(Err(
                        "ROS2 not available in Rust bridge. Use Python bridge for ROS2 support."
                            .to_string(),
                    ));
                }
                Ros2Command::Publish {
                    response_tx, topic, ..
                } => {
                    warn!("Stub: Cannot publish to '{}' - ROS2 not available", topic);
                    let _ = response_tx.send(Err(
                        "ROS2 not available in Rust bridge. Use Python bridge for ROS2 support."
                            .to_string(),
                    ));
                }
                Ros2Command::ServiceCall {
                    response_tx,
                    service,
                    ..
                } => {
                    warn!(
                        "Stub: Cannot call service '{}' - ROS2 not available",
                        service
                    );
                    let _ = response_tx.send(Err(
                        "ROS2 not available in Rust bridge. Use Python bridge for ROS2 support."
                            .to_string(),
                    ));
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
