//! RosKit Binary Protocol
//!
//! Wire format for high-performance ROS2 web communication.

use bytes::{Buf, BufMut, Bytes, BytesMut};
use serde::{Deserialize, Serialize};
use std::time::{SystemTime, UNIX_EPOCH};

/// Magic bytes identifying RosKit protocol
pub const MAGIC: [u8; 2] = [b'R', b'K'];

/// Protocol version
pub const PROTOCOL_VERSION: u8 = 1;

/// Header size in bytes
pub const HEADER_SIZE: usize = 24;

/// Message types
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MessageType {
    // Client -> Server
    Subscribe = 0x01,
    Unsubscribe = 0x02,
    Publish = 0x03,
    ServiceCall = 0x04,
    TopicList = 0x05,
    Ping = 0x06,
    ServiceList = 0x07,
    NodeList = 0x08,

    // Server -> Client
    Message = 0x10,
    ChannelInfo = 0x11,
    ServiceResponse = 0x12,
    TopicListResponse = 0x13,
    ServerInfo = 0x14,
    Pong = 0x15,
    ServiceListResponse = 0x16,
    NodeListResponse = 0x17,
    Error = 0xFF,
}

impl TryFrom<u8> for MessageType {
    type Error = ProtocolError;

    fn try_from(value: u8) -> Result<Self, ProtocolError> {
        match value {
            0x01 => Ok(MessageType::Subscribe),
            0x02 => Ok(MessageType::Unsubscribe),
            0x03 => Ok(MessageType::Publish),
            0x04 => Ok(MessageType::ServiceCall),
            0x05 => Ok(MessageType::TopicList),
            0x06 => Ok(MessageType::Ping),
            0x07 => Ok(MessageType::ServiceList),
            0x08 => Ok(MessageType::NodeList),
            0x10 => Ok(MessageType::Message),
            0x11 => Ok(MessageType::ChannelInfo),
            0x12 => Ok(MessageType::ServiceResponse),
            0x13 => Ok(MessageType::TopicListResponse),
            0x14 => Ok(MessageType::ServerInfo),
            0x15 => Ok(MessageType::Pong),
            0x16 => Ok(MessageType::ServiceListResponse),
            0x17 => Ok(MessageType::NodeListResponse),
            0xFF => Ok(MessageType::Error),
            _ => Err(ProtocolError::InvalidMessageType(value)),
        }
    }
}

/// Payload encoding types
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Encoding {
    Raw = 0x00,
    Cbor = 0x01,
    Json = 0x02,
    Png = 0x03,
    Jpeg = 0x04,
    Binary = 0x05,
}

impl TryFrom<u8> for Encoding {
    type Error = ProtocolError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(Encoding::Raw),
            0x01 => Ok(Encoding::Cbor),
            0x02 => Ok(Encoding::Json),
            0x03 => Ok(Encoding::Png),
            0x04 => Ok(Encoding::Jpeg),
            0x05 => Ok(Encoding::Binary),
            _ => Err(ProtocolError::InvalidEncoding(value)),
        }
    }
}

impl From<Encoding> for String {
    fn from(e: Encoding) -> Self {
        match e {
            Encoding::Raw => "raw".to_string(),
            Encoding::Cbor => "cbor".to_string(),
            Encoding::Json => "json".to_string(),
            Encoding::Png => "png".to_string(),
            Encoding::Jpeg => "jpeg".to_string(),
            Encoding::Binary => "binary".to_string(),
        }
    }
}

/// Message flags
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Flags {
    None = 0x00,
    Compressed = 0x01,
}

/// Protocol errors
#[derive(Debug, thiserror::Error)]
pub enum ProtocolError {
    #[error("Invalid magic bytes")]
    InvalidMagic,
    #[error("Invalid protocol version: {0}")]
    InvalidVersion(u8),
    #[error("Invalid message type: {0}")]
    InvalidMessageType(u8),
    #[error("Invalid encoding: {0}")]
    InvalidEncoding(u8),
    #[error("Message too short: expected {expected}, got {actual}")]
    MessageTooShort { expected: usize, actual: usize },
    #[error("Serialization error: {0}")]
    SerializationError(String),
}

/// Message header
#[derive(Debug, Clone)]
pub struct MessageHeader {
    pub message_type: MessageType,
    pub channel_id: u32,
    pub timestamp_ns: u64,
    pub encoding: Encoding,
    pub flags: u8,
    pub payload_length: u32,
}

impl MessageHeader {
    /// Parse header from bytes
    pub fn parse(data: &[u8]) -> Result<Self, ProtocolError> {
        if data.len() < HEADER_SIZE {
            return Err(ProtocolError::MessageTooShort {
                expected: HEADER_SIZE,
                actual: data.len(),
            });
        }

        let mut buf = &data[..];

        // Check magic
        let magic = [buf.get_u8(), buf.get_u8()];
        if magic != MAGIC {
            return Err(ProtocolError::InvalidMagic);
        }

        // Check version
        let version = buf.get_u8();
        if version != PROTOCOL_VERSION {
            return Err(ProtocolError::InvalidVersion(version));
        }

        let message_type = MessageType::try_from(buf.get_u8())?;
        let channel_id = buf.get_u32();
        let timestamp_ns = buf.get_u64();
        let encoding = Encoding::try_from(buf.get_u8())?;
        let flags = buf.get_u8();
        let _reserved = buf.get_u16(); // Skip reserved bytes
        let payload_length = buf.get_u32();

        Ok(Self {
            message_type,
            channel_id,
            timestamp_ns,
            encoding,
            flags,
            payload_length,
        })
    }

    /// Encode header to bytes
    pub fn encode(&self) -> BytesMut {
        let mut buf = BytesMut::with_capacity(HEADER_SIZE);

        buf.put_slice(&MAGIC);
        buf.put_u8(PROTOCOL_VERSION);
        buf.put_u8(self.message_type as u8);
        buf.put_u32(self.channel_id);
        buf.put_u64(self.timestamp_ns);
        buf.put_u8(self.encoding as u8);
        buf.put_u8(self.flags);
        buf.put_u16(0); // Reserved
        buf.put_u32(self.payload_length);

        buf
    }
}

/// Complete message with header and payload
#[derive(Debug)]
pub struct Message {
    pub header: MessageHeader,
    pub payload: Bytes,
}

impl Message {
    /// Parse a complete message from bytes
    pub fn parse(data: &[u8]) -> Result<Self, ProtocolError> {
        let header = MessageHeader::parse(data)?;

        let total_size = HEADER_SIZE + header.payload_length as usize;
        if data.len() < total_size {
            return Err(ProtocolError::MessageTooShort {
                expected: total_size,
                actual: data.len(),
            });
        }

        let payload = Bytes::copy_from_slice(&data[HEADER_SIZE..total_size]);

        Ok(Self { header, payload })
    }

    /// Create a new message
    pub fn new(
        message_type: MessageType,
        channel_id: u32,
        payload: Bytes,
        encoding: Encoding,
    ) -> Self {
        let timestamp_ns = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0);

        let header = MessageHeader {
            message_type,
            channel_id,
            timestamp_ns,
            encoding,
            flags: Flags::None as u8,
            payload_length: payload.len() as u32,
        };

        Self { header, payload }
    }

    /// Encode message to bytes
    pub fn encode(&self) -> Bytes {
        let mut buf = self.header.encode();
        buf.extend_from_slice(&self.payload);
        buf.freeze()
    }
}

// ===== CBOR Message Types =====

#[derive(Debug, Serialize, Deserialize)]
pub struct SubscribeRequest {
    pub topic: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub msg_type: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub throttle_ms: Option<u32>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct UnsubscribeRequest {
    pub channel_id: u32,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ChannelInfo {
    pub channel_id: u32,
    pub topic: String,
    pub msg_type: String,
    pub encoding: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ServerInfo {
    pub name: String,
    pub version: String,
    #[serde(default)]
    pub protocol_version: u8,
    pub ros_distro: String,
    pub capabilities: Vec<String>,
    #[serde(default)]
    pub encodings: Vec<String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TopicInfo {
    pub name: String,
    pub msg_type: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct TopicListResponse {
    pub topics: Vec<TopicInfo>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ServiceInfo {
    pub name: String,
    pub service_type: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ServiceListResponse {
    pub services: Vec<ServiceInfo>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct NodeListResponse {
    pub nodes: Vec<String>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ErrorResponse {
    pub error: String,
}

/// Encode a value to CBOR bytes
pub fn encode_cbor<T: Serialize>(value: &T) -> Result<Bytes, ProtocolError> {
    let mut buf = Vec::new();
    ciborium::into_writer(value, &mut buf)
        .map_err(|e| ProtocolError::SerializationError(e.to_string()))?;
    Ok(Bytes::from(buf))
}

/// Decode CBOR bytes to a value
pub fn decode_cbor<T: for<'de> Deserialize<'de>>(data: &[u8]) -> Result<T, ProtocolError> {
    ciborium::from_reader(data).map_err(|e| ProtocolError::SerializationError(e.to_string()))
}
