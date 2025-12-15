//! ROS2 integration using r2r
//!
//! Provides subscription, publishing, and service call functionality.

use bytes::Bytes;
use parking_lot::RwLock;
use r2r::{self, QosProfile};
use serde_json::Value as JsonValue;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::mpsc;
use tracing::{debug, error, info, warn};

use crate::encoders::{encode_laser_scan, encode_message_cbor, encode_occupancy_grid, encode_image, encode_compressed_image, ImageFormat};
use crate::protocol::{encode_cbor, Encoding, Message, MessageType, TopicInfo};

/// ROS2 context wrapper
pub struct Ros2Context {
    node: Arc<RwLock<r2r::Node>>,
    subscriptions: RwLock<HashMap<u32, SubscriptionHandle>>,
    publishers: RwLock<HashMap<String, PublisherHandle>>,
}

/// Handle to a ROS2 subscription
struct SubscriptionHandle {
    topic: String,
    msg_type: String,
    _cancel_tx: mpsc::Sender<()>,
}

/// Handle to a ROS2 publisher
struct PublisherHandle {
    topic: String,
    msg_type: String,
    publisher: Arc<dyn std::any::Any + Send + Sync>,
}

/// Message from ROS2 subscription to send to WebSocket client
#[derive(Debug)]
pub struct RosMessage {
    pub channel_id: u32,
    pub payload: Bytes,
    pub encoding: Encoding,
}

impl Ros2Context {
    /// Create a new ROS2 context
    pub fn new(node_name: &str) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let ctx = r2r::Context::create()?;
        let node = r2r::Node::create(ctx, node_name, "")?;

        Ok(Self {
            node: Arc::new(RwLock::new(node)),
            subscriptions: RwLock::new(HashMap::new()),
            publishers: RwLock::new(HashMap::new()),
        })
    }

    /// Spin the ROS2 node (should be called in a background task)
    pub async fn spin(&self) {
        loop {
            {
                let mut node = self.node.write();
                // Spin once with a small timeout
                if node.spin_once(Duration::from_millis(10)).is_err() {
                    tokio::time::sleep(Duration::from_millis(10)).await;
                }
            }
            // Yield to other tasks
            tokio::task::yield_now().await;
        }
    }

    /// Get list of available topics
    pub fn get_topics(&self) -> Vec<TopicInfo> {
        let node = self.node.read();
        let topic_names_and_types = node.get_topic_names_and_types();

        match topic_names_and_types {
            Ok(topics) => topics
                .into_iter()
                .flat_map(|(name, types)| {
                    types.into_iter().map(move |msg_type| TopicInfo {
                        name: name.clone(),
                        msg_type,
                    })
                })
                .collect(),
            Err(e) => {
                error!("Failed to get topic list: {}", e);
                Vec::new()
            }
        }
    }

    /// Subscribe to a ROS2 topic
    pub async fn subscribe(
        &self,
        channel_id: u32,
        topic: &str,
        msg_type: &str,
        throttle_ms: Option<u32>,
        message_tx: mpsc::Sender<RosMessage>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        info!(
            "Subscribing to topic {} (type: {}) for channel {}",
            topic, msg_type, channel_id
        );

        let (cancel_tx, mut cancel_rx) = mpsc::channel::<()>(1);

        // Determine encoding based on message type
        let encoding = Self::get_encoding_for_type(msg_type);

        // Create subscription based on message type
        let topic_str = topic.to_string();
        let msg_type_str = msg_type.to_string();
        let throttle_duration = throttle_ms.map(|ms| Duration::from_millis(ms as u64));

        // Use dynamic subscription for generic message handling
        let node = self.node.clone();
        let message_tx_clone = message_tx.clone();

        tokio::spawn(async move {
            let result = Self::run_subscription(
                node,
                channel_id,
                &topic_str,
                &msg_type_str,
                encoding,
                throttle_duration,
                message_tx_clone,
                &mut cancel_rx,
            )
            .await;

            if let Err(e) = result {
                error!(
                    "Subscription error for topic {} (channel {}): {}",
                    topic_str, channel_id, e
                );
            }
        });

        // Store subscription handle
        let handle = SubscriptionHandle {
            topic: topic.to_string(),
            msg_type: msg_type.to_string(),
            _cancel_tx: cancel_tx,
        };

        self.subscriptions.write().insert(channel_id, handle);

        Ok(())
    }

    /// Run subscription loop for a specific topic
    async fn run_subscription(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        msg_type: &str,
        encoding: Encoding,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        // Create typed subscription based on message type
        match msg_type {
            "sensor_msgs/msg/LaserScan" => {
                Self::subscribe_laser_scan(
                    node,
                    channel_id,
                    topic,
                    encoding,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "nav_msgs/msg/OccupancyGrid" => {
                Self::subscribe_occupancy_grid(
                    node,
                    channel_id,
                    topic,
                    encoding,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "nav_msgs/msg/Odometry" => {
                Self::subscribe_odometry(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "geometry_msgs/msg/PoseStamped" => {
                Self::subscribe_pose_stamped(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "nav_msgs/msg/Path" => {
                Self::subscribe_path(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "std_msgs/msg/String" => {
                Self::subscribe_string(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "sensor_msgs/msg/Image" => {
                Self::subscribe_image(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "sensor_msgs/msg/CompressedImage" => {
                Self::subscribe_compressed_image(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "sensor_msgs/msg/Imu" => {
                Self::subscribe_imu(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "sensor_msgs/msg/JointState" => {
                Self::subscribe_joint_state(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "tf2_msgs/msg/TFMessage" => {
                Self::subscribe_tf(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "geometry_msgs/msg/Twist" => {
                Self::subscribe_twist(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "std_msgs/msg/Bool" => {
                Self::subscribe_bool(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "std_msgs/msg/Int32" => {
                Self::subscribe_int32(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "std_msgs/msg/Float32" => {
                Self::subscribe_float32(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            "std_msgs/msg/Float64" => {
                Self::subscribe_float64(
                    node,
                    channel_id,
                    topic,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
            _ => {
                // For unknown types, try generic JSON subscription
                warn!(
                    "Unknown message type {}, using generic subscription",
                    msg_type
                );
                Self::subscribe_generic(
                    node,
                    channel_id,
                    topic,
                    msg_type,
                    throttle_duration,
                    message_tx,
                    cancel_rx,
                )
                .await
            }
        }
    }

    /// Subscribe to LaserScan messages
    async fn subscribe_laser_scan(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        encoding: Encoding,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::sensor_msgs::msg::LaserScan>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    // Apply throttling
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    // Encode message
                    let payload = match encoding {
                        Encoding::Binary => {
                            let encoded = encode_laser_scan(
                                &msg.ranges,
                                &msg.intensities,
                                msg.angle_min,
                                msg.angle_max,
                                msg.angle_increment,
                                msg.time_increment,
                                msg.scan_time,
                                msg.range_min,
                                msg.range_max,
                            );
                            // Format: [metadata_length][CBOR metadata][ranges][intensities]
                            let metadata = encode_cbor(&encoded.metadata)?;
                            let mut buf = bytes::BytesMut::new();
                            buf.extend_from_slice(&(metadata.len() as u32).to_be_bytes());
                            buf.extend_from_slice(&metadata);
                            buf.extend_from_slice(&encoded.binary_data);
                            buf.freeze()
                        }
                        _ => {
                            encode_message_cbor(&msg)?
                        }
                    };

                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        debug!("Channel {} closed, stopping subscription", channel_id);
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    debug!("Subscription for channel {} cancelled", channel_id);
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to OccupancyGrid messages
    async fn subscribe_occupancy_grid(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        encoding: Encoding,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::nav_msgs::msg::OccupancyGrid>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = match encoding {
                        Encoding::Png => {
                            match encode_occupancy_grid(
                                &msg.data,
                                msg.info.width,
                                msg.info.height,
                                msg.info.resolution,
                                msg.info.origin.position.x,
                                msg.info.origin.position.y,
                            ) {
                                Ok(encoded) => {
                                    // Format: [metadata_length][CBOR metadata][PNG]
                                    let metadata = encode_cbor(&encoded.metadata)?;
                                    let mut buf = bytes::BytesMut::new();
                                    buf.extend_from_slice(&(metadata.len() as u32).to_be_bytes());
                                    buf.extend_from_slice(&metadata);
                                    buf.extend_from_slice(&encoded.png_data);
                                    buf.freeze()
                                }
                                Err(e) => {
                                    error!("Failed to encode occupancy grid: {}", e);
                                    continue;
                                }
                            }
                        }
                        _ => encode_message_cbor(&msg)?
                    };

                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to Odometry messages
    async fn subscribe_odometry(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::nav_msgs::msg::Odometry>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = encode_message_cbor(&msg)?;
                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding: Encoding::Cbor,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to PoseStamped messages
    async fn subscribe_pose_stamped(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::geometry_msgs::msg::PoseStamped>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = encode_message_cbor(&msg)?;
                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding: Encoding::Cbor,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to Path messages
    async fn subscribe_path(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::nav_msgs::msg::Path>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = encode_message_cbor(&msg)?;
                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding: Encoding::Cbor,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to String messages
    async fn subscribe_string(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::std_msgs::msg::String>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = encode_message_cbor(&msg)?;
                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding: Encoding::Cbor,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to Image messages (raw camera images)
    async fn subscribe_image(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::sensor_msgs::msg::Image>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    // Encode raw image to JPEG
                    let encoded = match encode_image(
                        &msg.data,
                        msg.width,
                        msg.height,
                        &msg.encoding,
                        msg.step,
                        &msg.header.frame_id,
                        msg.header.stamp.sec,
                        msg.header.stamp.nanosec,
                    ) {
                        Ok(enc) => enc,
                        Err(e) => {
                            error!("Failed to encode image: {}", e);
                            continue;
                        }
                    };

                    // Format: [metadata_length][CBOR metadata][JPEG data]
                    let metadata = match crate::protocol::encode_cbor(&encoded.metadata) {
                        Ok(m) => m,
                        Err(e) => {
                            error!("Failed to encode image metadata: {}", e);
                            continue;
                        }
                    };

                    let mut buf = bytes::BytesMut::new();
                    buf.extend_from_slice(&(metadata.len() as u32).to_be_bytes());
                    buf.extend_from_slice(&metadata);
                    buf.extend_from_slice(&encoded.image_data);

                    let encoding = match encoded.format {
                        ImageFormat::Jpeg => Encoding::Jpeg,
                        ImageFormat::Png => Encoding::Png,
                    };

                    let ros_msg = RosMessage {
                        channel_id,
                        payload: buf.freeze(),
                        encoding,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to CompressedImage messages (pre-compressed camera images)
    async fn subscribe_compressed_image(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::sensor_msgs::msg::CompressedImage>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    // Handle compressed image (may pass through or re-encode)
                    let encoded = match encode_compressed_image(
                        &msg.data,
                        &msg.format,
                        &msg.header.frame_id,
                        msg.header.stamp.sec,
                        msg.header.stamp.nanosec,
                    ) {
                        Ok(enc) => enc,
                        Err(e) => {
                            error!("Failed to encode compressed image: {}", e);
                            continue;
                        }
                    };

                    // Format: [metadata_length][CBOR metadata][image data]
                    let metadata = match crate::protocol::encode_cbor(&encoded.metadata) {
                        Ok(m) => m,
                        Err(e) => {
                            error!("Failed to encode compressed image metadata: {}", e);
                            continue;
                        }
                    };

                    let mut buf = bytes::BytesMut::new();
                    buf.extend_from_slice(&(metadata.len() as u32).to_be_bytes());
                    buf.extend_from_slice(&metadata);
                    buf.extend_from_slice(&encoded.image_data);

                    let encoding = match encoded.format {
                        ImageFormat::Jpeg => Encoding::Jpeg,
                        ImageFormat::Png => Encoding::Png,
                    };

                    let ros_msg = RosMessage {
                        channel_id,
                        payload: buf.freeze(),
                        encoding,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to IMU messages
    async fn subscribe_imu(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::sensor_msgs::msg::Imu>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = encode_message_cbor(&msg)?;
                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding: Encoding::Cbor,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to JointState messages
    async fn subscribe_joint_state(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::sensor_msgs::msg::JointState>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = encode_message_cbor(&msg)?;
                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding: Encoding::Cbor,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to TF messages
    async fn subscribe_tf(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::tf2_msgs::msg::TFMessage>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = encode_message_cbor(&msg)?;
                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding: Encoding::Cbor,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to Twist messages
    async fn subscribe_twist(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::geometry_msgs::msg::Twist>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = encode_message_cbor(&msg)?;
                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding: Encoding::Cbor,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to Bool messages
    async fn subscribe_bool(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::std_msgs::msg::Bool>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = encode_message_cbor(&msg)?;
                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding: Encoding::Cbor,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to Int32 messages
    async fn subscribe_int32(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::std_msgs::msg::Int32>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = encode_message_cbor(&msg)?;
                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding: Encoding::Cbor,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to Float32 messages
    async fn subscribe_float32(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::std_msgs::msg::Float32>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = encode_message_cbor(&msg)?;
                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding: Encoding::Cbor,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Subscribe to Float64 messages
    async fn subscribe_float64(
        node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        throttle_duration: Option<Duration>,
        message_tx: mpsc::Sender<RosMessage>,
        cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let sub = {
            let mut node = node.write();
            node.subscribe::<r2r::std_msgs::msg::Float64>(topic, QosProfile::default())?
        };

        let mut last_send = std::time::Instant::now();

        tokio::pin!(sub);

        loop {
            tokio::select! {
                Some(msg) = sub.next() => {
                    if let Some(throttle) = throttle_duration {
                        if last_send.elapsed() < throttle {
                            continue;
                        }
                    }
                    last_send = std::time::Instant::now();

                    let payload = encode_message_cbor(&msg)?;
                    let ros_msg = RosMessage {
                        channel_id,
                        payload,
                        encoding: Encoding::Cbor,
                    };

                    if message_tx.send(ros_msg).await.is_err() {
                        break;
                    }
                }
                _ = cancel_rx.recv() => {
                    break;
                }
            }
        }

        Ok(())
    }

    /// Generic subscription for unknown message types (attempts JSON conversion)
    async fn subscribe_generic(
        _node: Arc<RwLock<r2r::Node>>,
        channel_id: u32,
        topic: &str,
        msg_type: &str,
        _throttle_duration: Option<Duration>,
        _message_tx: mpsc::Sender<RosMessage>,
        _cancel_rx: &mut mpsc::Receiver<()>,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        // For now, log unsupported types - full dynamic introspection requires more work
        warn!(
            "Generic subscription not fully implemented for {} ({}). Channel {} will not receive messages.",
            topic, msg_type, channel_id
        );
        Ok(())
    }

    /// Unsubscribe from a topic
    pub fn unsubscribe(&self, channel_id: u32) {
        if let Some(handle) = self.subscriptions.write().remove(&channel_id) {
            info!(
                "Unsubscribed from topic {} (channel {})",
                handle.topic, channel_id
            );
            // The cancel_tx will be dropped, which signals the subscription task to stop
        }
    }

    /// Publish a message to a topic
    pub async fn publish(
        &self,
        topic: &str,
        msg_type: &str,
        data: &JsonValue,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        info!("Publishing to topic {} (type: {})", topic, msg_type);

        match msg_type {
            "std_msgs/msg/String" => {
                let msg_str = data
                    .get("data")
                    .and_then(|v| v.as_str())
                    .unwrap_or_default();

                let mut node = self.node.write();
                let publisher =
                    node.create_publisher::<r2r::std_msgs::msg::String>(topic, QosProfile::default())?;

                let msg = r2r::std_msgs::msg::String {
                    data: msg_str.to_string(),
                };

                publisher.publish(&msg)?;
            }
            "geometry_msgs/msg/Twist" => {
                let mut node = self.node.write();
                let publisher = node
                    .create_publisher::<r2r::geometry_msgs::msg::Twist>(topic, QosProfile::default())?;

                let linear = data.get("linear").unwrap_or(&JsonValue::Null);
                let angular = data.get("angular").unwrap_or(&JsonValue::Null);

                let msg = r2r::geometry_msgs::msg::Twist {
                    linear: r2r::geometry_msgs::msg::Vector3 {
                        x: linear.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0),
                        y: linear.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0),
                        z: linear.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    },
                    angular: r2r::geometry_msgs::msg::Vector3 {
                        x: angular.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0),
                        y: angular.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0),
                        z: angular.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    },
                };

                publisher.publish(&msg)?;
            }
            "geometry_msgs/msg/PoseStamped" => {
                let mut node = self.node.write();
                let publisher = node.create_publisher::<r2r::geometry_msgs::msg::PoseStamped>(
                    topic,
                    QosProfile::default(),
                )?;

                let header = data.get("header").unwrap_or(&JsonValue::Null);
                let pose = data.get("pose").unwrap_or(&JsonValue::Null);
                let position = pose.get("position").unwrap_or(&JsonValue::Null);
                let orientation = pose.get("orientation").unwrap_or(&JsonValue::Null);

                let msg = r2r::geometry_msgs::msg::PoseStamped {
                    header: r2r::std_msgs::msg::Header {
                        stamp: r2r::builtin_interfaces::msg::Time {
                            sec: header
                                .get("stamp")
                                .and_then(|s| s.get("sec"))
                                .and_then(|v| v.as_i64())
                                .unwrap_or(0) as i32,
                            nanosec: header
                                .get("stamp")
                                .and_then(|s| s.get("nanosec"))
                                .and_then(|v| v.as_u64())
                                .unwrap_or(0) as u32,
                        },
                        frame_id: header
                            .get("frame_id")
                            .and_then(|v| v.as_str())
                            .unwrap_or("")
                            .to_string(),
                    },
                    pose: r2r::geometry_msgs::msg::Pose {
                        position: r2r::geometry_msgs::msg::Point {
                            x: position.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0),
                            y: position.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0),
                            z: position.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0),
                        },
                        orientation: r2r::geometry_msgs::msg::Quaternion {
                            x: orientation.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0),
                            y: orientation.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0),
                            z: orientation.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0),
                            w: orientation.get("w").and_then(|v| v.as_f64()).unwrap_or(1.0),
                        },
                    },
                };

                publisher.publish(&msg)?;
            }
            "geometry_msgs/msg/Pose" => {
                let mut node = self.node.write();
                let publisher = node.create_publisher::<r2r::geometry_msgs::msg::Pose>(
                    topic,
                    QosProfile::default(),
                )?;

                let position = data.get("position").unwrap_or(&JsonValue::Null);
                let orientation = data.get("orientation").unwrap_or(&JsonValue::Null);

                let msg = r2r::geometry_msgs::msg::Pose {
                    position: r2r::geometry_msgs::msg::Point {
                        x: position.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0),
                        y: position.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0),
                        z: position.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    },
                    orientation: r2r::geometry_msgs::msg::Quaternion {
                        x: orientation.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0),
                        y: orientation.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0),
                        z: orientation.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0),
                        w: orientation.get("w").and_then(|v| v.as_f64()).unwrap_or(1.0),
                    },
                };

                publisher.publish(&msg)?;
            }
            "geometry_msgs/msg/Point" => {
                let mut node = self.node.write();
                let publisher = node.create_publisher::<r2r::geometry_msgs::msg::Point>(
                    topic,
                    QosProfile::default(),
                )?;

                let msg = r2r::geometry_msgs::msg::Point {
                    x: data.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    y: data.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    z: data.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0),
                };

                publisher.publish(&msg)?;
            }
            "geometry_msgs/msg/Vector3" => {
                let mut node = self.node.write();
                let publisher = node.create_publisher::<r2r::geometry_msgs::msg::Vector3>(
                    topic,
                    QosProfile::default(),
                )?;

                let msg = r2r::geometry_msgs::msg::Vector3 {
                    x: data.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    y: data.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0),
                    z: data.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0),
                };

                publisher.publish(&msg)?;
            }
            "geometry_msgs/msg/TwistStamped" => {
                let mut node = self.node.write();
                let publisher = node.create_publisher::<r2r::geometry_msgs::msg::TwistStamped>(
                    topic,
                    QosProfile::default(),
                )?;

                let header = data.get("header").unwrap_or(&JsonValue::Null);
                let twist = data.get("twist").unwrap_or(&JsonValue::Null);
                let linear = twist.get("linear").unwrap_or(&JsonValue::Null);
                let angular = twist.get("angular").unwrap_or(&JsonValue::Null);

                let msg = r2r::geometry_msgs::msg::TwistStamped {
                    header: r2r::std_msgs::msg::Header {
                        stamp: r2r::builtin_interfaces::msg::Time {
                            sec: header
                                .get("stamp")
                                .and_then(|s| s.get("sec"))
                                .and_then(|v| v.as_i64())
                                .unwrap_or(0) as i32,
                            nanosec: header
                                .get("stamp")
                                .and_then(|s| s.get("nanosec"))
                                .and_then(|v| v.as_u64())
                                .unwrap_or(0) as u32,
                        },
                        frame_id: header
                            .get("frame_id")
                            .and_then(|v| v.as_str())
                            .unwrap_or("")
                            .to_string(),
                    },
                    twist: r2r::geometry_msgs::msg::Twist {
                        linear: r2r::geometry_msgs::msg::Vector3 {
                            x: linear.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0),
                            y: linear.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0),
                            z: linear.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0),
                        },
                        angular: r2r::geometry_msgs::msg::Vector3 {
                            x: angular.get("x").and_then(|v| v.as_f64()).unwrap_or(0.0),
                            y: angular.get("y").and_then(|v| v.as_f64()).unwrap_or(0.0),
                            z: angular.get("z").and_then(|v| v.as_f64()).unwrap_or(0.0),
                        },
                    },
                };

                publisher.publish(&msg)?;
            }
            "std_msgs/msg/Bool" => {
                let mut node = self.node.write();
                let publisher = node.create_publisher::<r2r::std_msgs::msg::Bool>(
                    topic,
                    QosProfile::default(),
                )?;

                let msg = r2r::std_msgs::msg::Bool {
                    data: data.get("data").and_then(|v| v.as_bool()).unwrap_or(false),
                };

                publisher.publish(&msg)?;
            }
            "std_msgs/msg/Int32" => {
                let mut node = self.node.write();
                let publisher = node.create_publisher::<r2r::std_msgs::msg::Int32>(
                    topic,
                    QosProfile::default(),
                )?;

                let msg = r2r::std_msgs::msg::Int32 {
                    data: data.get("data").and_then(|v| v.as_i64()).unwrap_or(0) as i32,
                };

                publisher.publish(&msg)?;
            }
            "std_msgs/msg/Float32" => {
                let mut node = self.node.write();
                let publisher = node.create_publisher::<r2r::std_msgs::msg::Float32>(
                    topic,
                    QosProfile::default(),
                )?;

                let msg = r2r::std_msgs::msg::Float32 {
                    data: data.get("data").and_then(|v| v.as_f64()).unwrap_or(0.0) as f32,
                };

                publisher.publish(&msg)?;
            }
            "std_msgs/msg/Float64" => {
                let mut node = self.node.write();
                let publisher = node.create_publisher::<r2r::std_msgs::msg::Float64>(
                    topic,
                    QosProfile::default(),
                )?;

                let msg = r2r::std_msgs::msg::Float64 {
                    data: data.get("data").and_then(|v| v.as_f64()).unwrap_or(0.0),
                };

                publisher.publish(&msg)?;
            }
            "std_msgs/msg/Empty" => {
                let mut node = self.node.write();
                let publisher = node.create_publisher::<r2r::std_msgs::msg::Empty>(
                    topic,
                    QosProfile::default(),
                )?;

                let msg = r2r::std_msgs::msg::Empty {};

                publisher.publish(&msg)?;
            }
            _ => {
                return Err(format!("Unsupported message type for publish: {}", msg_type).into());
            }
        }

        Ok(())
    }

    /// Call a ROS2 service
    pub async fn call_service(
        &self,
        service_name: &str,
        service_type: &str,
        request: &JsonValue,
        timeout: Duration,
    ) -> Result<JsonValue, Box<dyn std::error::Error + Send + Sync>> {
        info!(
            "Calling service {} (type: {})",
            service_name, service_type
        );

        match service_type {
            "std_srvs/srv/Empty" => {
                let client = {
                    let mut node = self.node.write();
                    node.create_client::<r2r::std_srvs::srv::Empty::Service>(service_name)?
                };

                // Wait for service to be available
                let available = client.is_available()?;
                if !available {
                    return Err("Service not available".into());
                }

                let req = r2r::std_srvs::srv::Empty::Request {};
                let response = tokio::time::timeout(timeout, client.request(&req)?).await??;

                Ok(serde_json::json!({}))
            }
            "std_srvs/srv/SetBool" => {
                let client = {
                    let mut node = self.node.write();
                    node.create_client::<r2r::std_srvs::srv::SetBool::Service>(service_name)?
                };

                let req = r2r::std_srvs::srv::SetBool::Request {
                    data: request.get("data").and_then(|v| v.as_bool()).unwrap_or(false),
                };

                let response = tokio::time::timeout(timeout, client.request(&req)?).await??;

                Ok(serde_json::json!({
                    "success": response.success,
                    "message": response.message
                }))
            }
            "std_srvs/srv/Trigger" => {
                let client = {
                    let mut node = self.node.write();
                    node.create_client::<r2r::std_srvs::srv::Trigger::Service>(service_name)?
                };

                let req = r2r::std_srvs::srv::Trigger::Request {};
                let response = tokio::time::timeout(timeout, client.request(&req)?).await??;

                Ok(serde_json::json!({
                    "success": response.success,
                    "message": response.message
                }))
            }
            "rcl_interfaces/srv/GetParameters" => {
                let client = {
                    let mut node = self.node.write();
                    node.create_client::<r2r::rcl_interfaces::srv::GetParameters::Service>(service_name)?
                };

                let names: Vec<String> = request
                    .get("names")
                    .and_then(|v| v.as_array())
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_str().map(String::from))
                            .collect()
                    })
                    .unwrap_or_default();

                let req = r2r::rcl_interfaces::srv::GetParameters::Request { names };
                let response = tokio::time::timeout(timeout, client.request(&req)?).await??;

                // Convert parameter values to JSON
                let values: Vec<JsonValue> = response
                    .values
                    .iter()
                    .map(|v| {
                        serde_json::json!({
                            "type": v.type_,
                            "bool_value": v.bool_value,
                            "integer_value": v.integer_value,
                            "double_value": v.double_value,
                            "string_value": v.string_value,
                            "byte_array_value": v.byte_array_value,
                            "bool_array_value": v.bool_array_value,
                            "integer_array_value": v.integer_array_value,
                            "double_array_value": v.double_array_value,
                            "string_array_value": v.string_array_value
                        })
                    })
                    .collect();

                Ok(serde_json::json!({ "values": values }))
            }
            "rcl_interfaces/srv/SetParameters" => {
                let client = {
                    let mut node = self.node.write();
                    node.create_client::<r2r::rcl_interfaces::srv::SetParameters::Service>(service_name)?
                };

                let parameters: Vec<r2r::rcl_interfaces::msg::Parameter> = request
                    .get("parameters")
                    .and_then(|v| v.as_array())
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|p| {
                                let name = p.get("name")?.as_str()?.to_string();
                                let value = p.get("value")?;

                                Some(r2r::rcl_interfaces::msg::Parameter {
                                    name,
                                    value: r2r::rcl_interfaces::msg::ParameterValue {
                                        type_: value.get("type").and_then(|v| v.as_u64()).unwrap_or(0) as u8,
                                        bool_value: value.get("bool_value").and_then(|v| v.as_bool()).unwrap_or(false),
                                        integer_value: value.get("integer_value").and_then(|v| v.as_i64()).unwrap_or(0),
                                        double_value: value.get("double_value").and_then(|v| v.as_f64()).unwrap_or(0.0),
                                        string_value: value.get("string_value").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                                        byte_array_value: value.get("byte_array_value")
                                            .and_then(|v| v.as_array())
                                            .map(|arr| arr.iter().filter_map(|v| v.as_u64().map(|n| n as u8)).collect())
                                            .unwrap_or_default(),
                                        bool_array_value: value.get("bool_array_value")
                                            .and_then(|v| v.as_array())
                                            .map(|arr| arr.iter().filter_map(|v| v.as_bool()).collect())
                                            .unwrap_or_default(),
                                        integer_array_value: value.get("integer_array_value")
                                            .and_then(|v| v.as_array())
                                            .map(|arr| arr.iter().filter_map(|v| v.as_i64()).collect())
                                            .unwrap_or_default(),
                                        double_array_value: value.get("double_array_value")
                                            .and_then(|v| v.as_array())
                                            .map(|arr| arr.iter().filter_map(|v| v.as_f64()).collect())
                                            .unwrap_or_default(),
                                        string_array_value: value.get("string_array_value")
                                            .and_then(|v| v.as_array())
                                            .map(|arr| arr.iter().filter_map(|v| v.as_str().map(String::from)).collect())
                                            .unwrap_or_default(),
                                    },
                                })
                            })
                            .collect()
                    })
                    .unwrap_or_default();

                let req = r2r::rcl_interfaces::srv::SetParameters::Request { parameters };
                let response = tokio::time::timeout(timeout, client.request(&req)?).await??;

                let results: Vec<JsonValue> = response
                    .results
                    .iter()
                    .map(|r| {
                        serde_json::json!({
                            "successful": r.successful,
                            "reason": r.reason
                        })
                    })
                    .collect();

                Ok(serde_json::json!({ "results": results }))
            }
            "rcl_interfaces/srv/ListParameters" => {
                let client = {
                    let mut node = self.node.write();
                    node.create_client::<r2r::rcl_interfaces::srv::ListParameters::Service>(service_name)?
                };

                let prefixes: Vec<String> = request
                    .get("prefixes")
                    .and_then(|v| v.as_array())
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_str().map(String::from))
                            .collect()
                    })
                    .unwrap_or_default();

                let depth = request.get("depth").and_then(|v| v.as_u64()).unwrap_or(0) as u64;

                let req = r2r::rcl_interfaces::srv::ListParameters::Request { prefixes, depth };
                let response = tokio::time::timeout(timeout, client.request(&req)?).await??;

                Ok(serde_json::json!({
                    "result": {
                        "names": response.result.names,
                        "prefixes": response.result.prefixes
                    }
                }))
            }
            _ => {
                Err(format!("Unsupported service type: {}", service_type).into())
            }
        }
    }

    /// Get list of available services
    pub fn get_services(&self) -> Vec<(String, Vec<String>)> {
        let node = self.node.read();
        match node.get_service_names_and_types() {
            Ok(services) => services,
            Err(e) => {
                error!("Failed to get service list: {}", e);
                Vec::new()
            }
        }
    }

    /// Get list of available nodes
    pub fn get_nodes(&self) -> Vec<String> {
        let node = self.node.read();
        match node.get_node_names() {
            Ok(nodes) => nodes,
            Err(e) => {
                error!("Failed to get node list: {}", e);
                Vec::new()
            }
        }
    }

    /// Determine best encoding for a message type
    fn get_encoding_for_type(msg_type: &str) -> Encoding {
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
}

impl Drop for Ros2Context {
    fn drop(&mut self) {
        info!("Shutting down ROS2 context");
        // Subscriptions will be cleaned up when their handles are dropped
        self.subscriptions.write().clear();
        self.publishers.write().clear();
    }
}

use futures_util::StreamExt;
