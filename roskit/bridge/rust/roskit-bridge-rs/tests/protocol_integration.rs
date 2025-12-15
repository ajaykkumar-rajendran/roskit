//! Protocol Integration Tests
//!
//! Tests for protocol message encoding/decoding and server behavior.
//! These tests verify the binary protocol format and server responses
//! without requiring ROS2 runtime.

use bytes::Bytes;

// Protocol constants matching src/protocol.rs
const MAGIC: [u8; 2] = [b'R', b'K'];
const PROTOCOL_VERSION: u8 = 1;
const HEADER_SIZE: usize = 24;

/// Message types
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq)]
enum MessageType {
    Subscribe = 0x01,
    Unsubscribe = 0x02,
    Publish = 0x03,
    ServiceCall = 0x04,
    TopicList = 0x05,
    Ping = 0x06,
    ServiceList = 0x07,
    NodeList = 0x08,
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

/// Encoding types
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq)]
enum Encoding {
    Raw = 0x00,
    Cbor = 0x01,
    Json = 0x02,
    Png = 0x03,
    Jpeg = 0x04,
    Binary = 0x05,
}

/// Create a protocol message
fn create_message(
    msg_type: MessageType,
    channel_id: u32,
    payload: &[u8],
    encoding: Encoding,
    timestamp_ns: u64,
    flags: u8,
) -> Vec<u8> {
    let mut buf = Vec::with_capacity(HEADER_SIZE + payload.len());

    // Magic
    buf.extend_from_slice(&MAGIC);
    // Version
    buf.push(PROTOCOL_VERSION);
    // Message type
    buf.push(msg_type as u8);
    // Channel ID (4 bytes big-endian)
    buf.extend_from_slice(&channel_id.to_be_bytes());
    // Timestamp (8 bytes big-endian)
    buf.extend_from_slice(&timestamp_ns.to_be_bytes());
    // Encoding
    buf.push(encoding as u8);
    // Flags
    buf.push(flags);
    // Reserved (2 bytes)
    buf.extend_from_slice(&[0u8; 2]);
    // Payload length (4 bytes big-endian)
    buf.extend_from_slice(&(payload.len() as u32).to_be_bytes());
    // Payload
    buf.extend_from_slice(payload);

    buf
}

/// Parse message header
fn parse_header(data: &[u8]) -> Option<(MessageType, u32, u64, Encoding, u8, u32)> {
    if data.len() < HEADER_SIZE {
        return None;
    }

    // Check magic
    if data[0] != MAGIC[0] || data[1] != MAGIC[1] {
        return None;
    }

    // Check version
    if data[2] != PROTOCOL_VERSION {
        return None;
    }

    let msg_type = data[3];
    let channel_id = u32::from_be_bytes([data[4], data[5], data[6], data[7]]);
    let timestamp_ns = u64::from_be_bytes([
        data[8], data[9], data[10], data[11],
        data[12], data[13], data[14], data[15],
    ]);
    let encoding = data[16];
    let flags = data[17];
    let payload_len = u32::from_be_bytes([data[20], data[21], data[22], data[23]]);

    Some((
        match msg_type {
            0x01 => MessageType::Subscribe,
            0x06 => MessageType::Ping,
            0x07 => MessageType::ServiceList,
            0x08 => MessageType::NodeList,
            0x10 => MessageType::Message,
            0x14 => MessageType::ServerInfo,
            0x15 => MessageType::Pong,
            0x16 => MessageType::ServiceListResponse,
            0x17 => MessageType::NodeListResponse,
            0xFF => MessageType::Error,
            _ => return None,
        },
        channel_id,
        timestamp_ns,
        match encoding {
            0x00 => Encoding::Raw,
            0x01 => Encoding::Cbor,
            0x02 => Encoding::Json,
            0x03 => Encoding::Png,
            0x04 => Encoding::Jpeg,
            0x05 => Encoding::Binary,
            _ => return None,
        },
        flags,
        payload_len,
    ))
}

#[test]
fn test_ping_message_encoding() {
    let msg = create_message(MessageType::Ping, 0, &[], Encoding::Raw, 0, 0);

    assert_eq!(msg.len(), HEADER_SIZE);

    let expected = hex::decode("524b01060000000000000000000000000000000000000000").unwrap();
    assert_eq!(msg, expected, "PING message encoding mismatch");

    let parsed = parse_header(&msg);
    assert!(parsed.is_some());
    let (msg_type, channel_id, _, encoding, flags, payload_len) = parsed.unwrap();
    assert_eq!(msg_type, MessageType::Ping);
    assert_eq!(channel_id, 0);
    assert_eq!(encoding, Encoding::Raw);
    assert_eq!(flags, 0);
    assert_eq!(payload_len, 0);
}

#[test]
fn test_node_list_message_encoding() {
    let msg = create_message(MessageType::NodeList, 0, &[], Encoding::Raw, 0, 0);

    let expected = hex::decode("524b01080000000000000000000000000000000000000000").unwrap();
    assert_eq!(msg, expected, "NODE_LIST message encoding mismatch");

    let parsed = parse_header(&msg);
    assert!(parsed.is_some());
    let (msg_type, ..) = parsed.unwrap();
    assert_eq!(msg_type, MessageType::NodeList);
}

#[test]
fn test_service_list_message_encoding() {
    let msg = create_message(MessageType::ServiceList, 0, &[], Encoding::Raw, 0, 0);

    let expected = hex::decode("524b01070000000000000000000000000000000000000000").unwrap();
    assert_eq!(msg, expected, "SERVICE_LIST message encoding mismatch");
}

#[test]
fn test_message_with_channel_id() {
    let msg = create_message(MessageType::Message, 42, &[0xf6], Encoding::Cbor, 1_000_000_000, 0);

    let parsed = parse_header(&msg);
    assert!(parsed.is_some());
    let (msg_type, channel_id, timestamp, encoding, _, payload_len) = parsed.unwrap();

    assert_eq!(msg_type, MessageType::Message);
    assert_eq!(channel_id, 42);
    assert_eq!(timestamp, 1_000_000_000);
    assert_eq!(encoding, Encoding::Cbor);
    assert_eq!(payload_len, 1);
    assert_eq!(msg[HEADER_SIZE], 0xf6); // CBOR null
}

#[test]
fn test_max_channel_id() {
    let max_channel = u32::MAX;
    let msg = create_message(MessageType::Message, max_channel, &[], Encoding::Raw, 0, 0);

    let parsed = parse_header(&msg);
    assert!(parsed.is_some());
    let (_, channel_id, ..) = parsed.unwrap();
    assert_eq!(channel_id, max_channel);
}

#[test]
fn test_invalid_magic_rejected() {
    let mut msg = create_message(MessageType::Ping, 0, &[], Encoding::Raw, 0, 0);
    msg[0] = b'R';
    msg[1] = b'W'; // Wrong magic

    let parsed = parse_header(&msg);
    assert!(parsed.is_none(), "Should reject invalid magic bytes");
}

#[test]
fn test_invalid_version_rejected() {
    let mut msg = create_message(MessageType::Ping, 0, &[], Encoding::Raw, 0, 0);
    msg[2] = 2; // Invalid version

    let parsed = parse_header(&msg);
    assert!(parsed.is_none(), "Should reject invalid protocol version");
}

#[test]
fn test_truncated_header_rejected() {
    let msg = hex::decode("524b010600000000").unwrap(); // Only 8 bytes

    let parsed = parse_header(&msg);
    assert!(parsed.is_none(), "Should reject truncated header");
}

#[test]
fn test_cbor_subscribe_request() {
    // Subscribe request with CBOR payload: {"topic": "/scan"}
    let cbor_payload = hex::decode("a165746f706963652f7363616e").unwrap();
    let msg = create_message(
        MessageType::Subscribe,
        0,
        &cbor_payload,
        Encoding::Cbor,
        0,
        0,
    );

    let parsed = parse_header(&msg);
    assert!(parsed.is_some());
    let (msg_type, _, _, encoding, _, payload_len) = parsed.unwrap();

    assert_eq!(msg_type, MessageType::Subscribe);
    assert_eq!(encoding, Encoding::Cbor);
    assert_eq!(payload_len, cbor_payload.len() as u32);

    // Verify payload
    let payload = &msg[HEADER_SIZE..];
    assert_eq!(payload, cbor_payload.as_slice());
}

#[test]
fn test_all_encoding_types() {
    let encodings = [
        (Encoding::Raw, 0x00),
        (Encoding::Cbor, 0x01),
        (Encoding::Json, 0x02),
        (Encoding::Png, 0x03),
        (Encoding::Jpeg, 0x04),
        (Encoding::Binary, 0x05),
    ];

    for (encoding, expected_byte) in encodings {
        let msg = create_message(MessageType::Message, 1, &[], encoding, 0, 0);
        assert_eq!(msg[16], expected_byte, "Encoding {:?} should be 0x{:02x}", encoding, expected_byte);
    }
}

#[test]
fn test_all_message_types() {
    let message_types = [
        (MessageType::Subscribe, 0x01),
        (MessageType::Ping, 0x06),
        (MessageType::ServiceList, 0x07),
        (MessageType::NodeList, 0x08),
        (MessageType::Message, 0x10),
        (MessageType::ServerInfo, 0x14),
        (MessageType::Pong, 0x15),
        (MessageType::ServiceListResponse, 0x16),
        (MessageType::NodeListResponse, 0x17),
        (MessageType::Error, 0xFF),
    ];

    for (msg_type, expected_byte) in message_types {
        let msg = create_message(msg_type, 0, &[], Encoding::Raw, 0, 0);
        assert_eq!(msg[3], expected_byte, "MessageType {:?} should be 0x{:02x}", msg_type, expected_byte);
    }
}
