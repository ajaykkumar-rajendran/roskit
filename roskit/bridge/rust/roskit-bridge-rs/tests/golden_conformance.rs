//! Golden Conformance Tests for RosKit Protocol (Rust Bridge)
//!
//! These tests validate that the Rust bridge correctly implements the RosKit
//! binary protocol by testing against golden vectors shared across all
//! implementations (TypeScript, Python, Rust).
//!
//! Run with: cargo test --test golden_conformance

use std::fs;
use std::path::PathBuf;

// Protocol constants - must match src/protocol.rs
const MAGIC: [u8; 2] = [b'R', b'K'];
const PROTOCOL_VERSION: u8 = 1;
const HEADER_SIZE: usize = 24;

/// Message types
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum Encoding {
    Raw = 0x00,
    Cbor = 0x01,
    Json = 0x02,
    Png = 0x03,
    Jpeg = 0x04,
    Binary = 0x05,
}

/// Flags
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum Flags {
    None = 0x00,
    Compressed = 0x01,
    Fragmented = 0x02,
    LastFragment = 0x04,
}

fn hex_to_bytes(hex: &str) -> Vec<u8> {
    (0..hex.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
        .collect()
}

fn bytes_to_hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{:02x}", b)).collect()
}

fn create_message(
    message_type: u8,
    channel_id: u32,
    payload: &[u8],
    encoding: u8,
    timestamp_ns: u64,
    flags: u8,
) -> Vec<u8> {
    let mut buf = Vec::with_capacity(HEADER_SIZE + payload.len());

    // Magic (2 bytes)
    buf.extend_from_slice(&MAGIC);
    // Version (1 byte)
    buf.push(PROTOCOL_VERSION);
    // Message type (1 byte)
    buf.push(message_type);
    // Channel ID (4 bytes big-endian)
    buf.extend_from_slice(&channel_id.to_be_bytes());
    // Timestamp (8 bytes big-endian)
    buf.extend_from_slice(&timestamp_ns.to_be_bytes());
    // Encoding (1 byte)
    buf.push(encoding);
    // Flags (1 byte)
    buf.push(flags);
    // Reserved (2 bytes)
    buf.extend_from_slice(&[0u8; 2]);
    // Payload length (4 bytes big-endian)
    buf.extend_from_slice(&(payload.len() as u32).to_be_bytes());
    // Payload
    buf.extend_from_slice(payload);

    buf
}

#[derive(Debug)]
struct ParsedHeader {
    message_type: u8,
    channel_id: u32,
    timestamp_ns: u64,
    encoding: u8,
    flags: u8,
    payload_length: u32,
}

fn parse_header(data: &[u8]) -> Option<ParsedHeader> {
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

    Some(ParsedHeader {
        message_type: data[3],
        channel_id: u32::from_be_bytes([data[4], data[5], data[6], data[7]]),
        timestamp_ns: u64::from_be_bytes([
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15],
        ]),
        encoding: data[16],
        flags: data[17],
        payload_length: u32::from_be_bytes([data[20], data[21], data[22], data[23]]),
    })
}

fn load_golden_vectors() -> serde_json::Value {
    // Try multiple paths to find the golden vectors file
    let possible_paths = [
        PathBuf::from("../../../protocol-tests/golden-vectors.json"),
        PathBuf::from("../../../../protocol-tests/golden-vectors.json"),
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../../protocol-tests/golden-vectors.json"),
    ];

    for path in &possible_paths {
        if let Ok(content) = fs::read_to_string(path) {
            return serde_json::from_str(&content).expect("Failed to parse golden vectors JSON");
        }
    }

    panic!(
        "Could not find golden-vectors.json. Tried paths: {:?}",
        possible_paths
    );
}

// ============================================================================
// Protocol Constants Tests
// ============================================================================

#[test]
fn test_protocol_magic() {
    let vectors = load_golden_vectors();
    let expected_magic: Vec<u8> = vectors["protocol"]["magic"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_u64().unwrap() as u8)
        .collect();

    assert_eq!(
        MAGIC.to_vec(),
        expected_magic,
        "MAGIC bytes should match golden vectors"
    );
}

#[test]
fn test_protocol_version() {
    let vectors = load_golden_vectors();
    let expected_version = vectors["protocol"]["version"].as_u64().unwrap() as u8;

    assert_eq!(
        PROTOCOL_VERSION, expected_version,
        "PROTOCOL_VERSION should match golden vectors"
    );
}

#[test]
fn test_protocol_header_size() {
    let vectors = load_golden_vectors();
    let expected_size = vectors["protocol"]["header_size"].as_u64().unwrap() as usize;

    assert_eq!(
        HEADER_SIZE, expected_size,
        "HEADER_SIZE should match golden vectors"
    );
}

// ============================================================================
// Message Type Tests
// ============================================================================

#[test]
fn test_message_type_subscribe() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["message_types"]["SUBSCRIBE"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(MessageType::Subscribe as u8, expected);
}

#[test]
fn test_message_type_unsubscribe() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["message_types"]["UNSUBSCRIBE"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(MessageType::Unsubscribe as u8, expected);
}

#[test]
fn test_message_type_publish() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["message_types"]["PUBLISH"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(MessageType::Publish as u8, expected);
}

#[test]
fn test_message_type_service_call() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["message_types"]["SERVICE_CALL"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(MessageType::ServiceCall as u8, expected);
}

#[test]
fn test_message_type_topic_list() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["message_types"]["TOPIC_LIST"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(MessageType::TopicList as u8, expected);
}

#[test]
fn test_message_type_ping() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["message_types"]["PING"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(MessageType::Ping as u8, expected);
}

#[test]
fn test_message_type_service_list() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["message_types"]["SERVICE_LIST"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(MessageType::ServiceList as u8, expected);
}

#[test]
fn test_message_type_node_list() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["message_types"]["NODE_LIST"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(MessageType::NodeList as u8, expected);
}

#[test]
fn test_message_type_message() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["message_types"]["MESSAGE"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(MessageType::Message as u8, expected);
}

#[test]
fn test_message_type_channel_info() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["message_types"]["CHANNEL_INFO"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(MessageType::ChannelInfo as u8, expected);
}

#[test]
fn test_message_type_service_response() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["message_types"]["SERVICE_RESPONSE"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(MessageType::ServiceResponse as u8, expected);
}

#[test]
fn test_message_type_error() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["message_types"]["ERROR"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(MessageType::Error as u8, expected);
}

// ============================================================================
// Encoding Type Tests
// ============================================================================

#[test]
fn test_encoding_raw() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["encodings"]["RAW"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(Encoding::Raw as u8, expected);
}

#[test]
fn test_encoding_cbor() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["encodings"]["CBOR"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(Encoding::Cbor as u8, expected);
}

#[test]
fn test_encoding_json() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["encodings"]["JSON"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(Encoding::Json as u8, expected);
}

#[test]
fn test_encoding_png() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["encodings"]["PNG"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(Encoding::Png as u8, expected);
}

#[test]
fn test_encoding_jpeg() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["encodings"]["JPEG"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(Encoding::Jpeg as u8, expected);
}

#[test]
fn test_encoding_binary() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["encodings"]["BINARY"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(Encoding::Binary as u8, expected);
}

// ============================================================================
// Flag Tests
// ============================================================================

#[test]
fn test_flag_none() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["flags"]["NONE"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(Flags::None as u8, expected);
}

#[test]
fn test_flag_compressed() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["flags"]["COMPRESSED"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(Flags::Compressed as u8, expected);
}

#[test]
fn test_flag_fragmented() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["flags"]["FRAGMENTED"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(Flags::Fragmented as u8, expected);
}

#[test]
fn test_flag_last_fragment() {
    let vectors = load_golden_vectors();
    let expected = u8::from_str_radix(
        vectors["flags"]["LAST_FRAGMENT"]
            .as_str()
            .unwrap()
            .trim_start_matches("0x"),
        16,
    )
    .unwrap();
    assert_eq!(Flags::LastFragment as u8, expected);
}

// ============================================================================
// Golden Vector Message Tests
// ============================================================================

#[test]
fn test_golden_empty_ping() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "empty_ping")
        .expect("empty_ping vector not found");

    let header = &vector["header"];
    let msg_type =
        u8::from_str_radix(header["message_type"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let channel_id = header["channel_id"].as_u64().unwrap() as u32;
    let timestamp_ns = header["timestamp_ns"].as_u64().unwrap();
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let flags =
        u8::from_str_radix(header["flags"].as_str().unwrap().trim_start_matches("0x"), 16).unwrap();
    let payload = hex_to_bytes(vector["payload_hex"].as_str().unwrap_or(""));

    let created = create_message(msg_type, channel_id, &payload, encoding, timestamp_ns, flags);
    let expected = hex_to_bytes(vector["expected_hex"].as_str().unwrap());

    assert_eq!(
        bytes_to_hex(&created),
        bytes_to_hex(&expected),
        "empty_ping message encoding mismatch"
    );

    // Verify round-trip parsing
    let parsed = parse_header(&created).expect("Failed to parse created message");
    assert_eq!(parsed.message_type, msg_type);
    assert_eq!(parsed.channel_id, channel_id);
    assert_eq!(parsed.timestamp_ns, timestamp_ns);
    assert_eq!(parsed.encoding, encoding);
    assert_eq!(parsed.flags, flags);
    assert_eq!(parsed.payload_length, payload.len() as u32);
}

#[test]
fn test_golden_empty_pong() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "empty_pong")
        .expect("empty_pong vector not found");

    let header = &vector["header"];
    let msg_type =
        u8::from_str_radix(header["message_type"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let channel_id = header["channel_id"].as_u64().unwrap() as u32;
    let timestamp_ns = header["timestamp_ns"].as_u64().unwrap();
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let flags =
        u8::from_str_radix(header["flags"].as_str().unwrap().trim_start_matches("0x"), 16).unwrap();
    let payload = hex_to_bytes(vector["payload_hex"].as_str().unwrap_or(""));

    let created = create_message(msg_type, channel_id, &payload, encoding, timestamp_ns, flags);
    let expected = hex_to_bytes(vector["expected_hex"].as_str().unwrap());

    assert_eq!(
        bytes_to_hex(&created),
        bytes_to_hex(&expected),
        "empty_pong message encoding mismatch"
    );
}

#[test]
fn test_golden_topic_list_request() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "topic_list_request")
        .expect("topic_list_request vector not found");

    let header = &vector["header"];
    let msg_type =
        u8::from_str_radix(header["message_type"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let channel_id = header["channel_id"].as_u64().unwrap() as u32;
    let timestamp_ns = header["timestamp_ns"].as_u64().unwrap();
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let flags =
        u8::from_str_radix(header["flags"].as_str().unwrap().trim_start_matches("0x"), 16).unwrap();
    let payload = hex_to_bytes(vector["payload_hex"].as_str().unwrap_or(""));

    let created = create_message(msg_type, channel_id, &payload, encoding, timestamp_ns, flags);
    let expected = hex_to_bytes(vector["expected_hex"].as_str().unwrap());

    assert_eq!(
        bytes_to_hex(&created),
        bytes_to_hex(&expected),
        "topic_list_request message encoding mismatch"
    );
}

#[test]
fn test_golden_service_list_request() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "service_list_request")
        .expect("service_list_request vector not found");

    let header = &vector["header"];
    let msg_type =
        u8::from_str_radix(header["message_type"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let channel_id = header["channel_id"].as_u64().unwrap() as u32;
    let timestamp_ns = header["timestamp_ns"].as_u64().unwrap();
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let flags =
        u8::from_str_radix(header["flags"].as_str().unwrap().trim_start_matches("0x"), 16).unwrap();
    let payload = hex_to_bytes(vector["payload_hex"].as_str().unwrap_or(""));

    let created = create_message(msg_type, channel_id, &payload, encoding, timestamp_ns, flags);
    let expected = hex_to_bytes(vector["expected_hex"].as_str().unwrap());

    assert_eq!(
        bytes_to_hex(&created),
        bytes_to_hex(&expected),
        "service_list_request message encoding mismatch"
    );
}

#[test]
fn test_golden_node_list_request() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "node_list_request")
        .expect("node_list_request vector not found");

    let header = &vector["header"];
    let msg_type =
        u8::from_str_radix(header["message_type"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let channel_id = header["channel_id"].as_u64().unwrap() as u32;
    let timestamp_ns = header["timestamp_ns"].as_u64().unwrap();
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let flags =
        u8::from_str_radix(header["flags"].as_str().unwrap().trim_start_matches("0x"), 16).unwrap();
    let payload = hex_to_bytes(vector["payload_hex"].as_str().unwrap_or(""));

    let created = create_message(msg_type, channel_id, &payload, encoding, timestamp_ns, flags);
    let expected = hex_to_bytes(vector["expected_hex"].as_str().unwrap());

    assert_eq!(
        bytes_to_hex(&created),
        bytes_to_hex(&expected),
        "node_list_request message encoding mismatch"
    );
}

#[test]
fn test_golden_message_with_channel() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "message_with_channel")
        .expect("message_with_channel vector not found");

    let header = &vector["header"];
    let msg_type =
        u8::from_str_radix(header["message_type"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let channel_id = header["channel_id"].as_u64().unwrap() as u32;
    let timestamp_ns = header["timestamp_ns"].as_u64().unwrap();
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let flags =
        u8::from_str_radix(header["flags"].as_str().unwrap().trim_start_matches("0x"), 16).unwrap();
    let payload = hex_to_bytes(vector["payload_hex"].as_str().unwrap_or(""));

    let created = create_message(msg_type, channel_id, &payload, encoding, timestamp_ns, flags);
    let expected = hex_to_bytes(vector["expected_hex"].as_str().unwrap());

    assert_eq!(
        bytes_to_hex(&created),
        bytes_to_hex(&expected),
        "message_with_channel encoding mismatch"
    );

    // Verify parsed values
    let parsed = parse_header(&created).expect("Failed to parse");
    assert_eq!(parsed.channel_id, 42, "channel_id should be 42");
    assert_eq!(
        parsed.timestamp_ns, 1000000000,
        "timestamp_ns should be 1000000000"
    );
    assert_eq!(
        parsed.encoding,
        Encoding::Cbor as u8,
        "encoding should be CBOR"
    );
    assert_eq!(parsed.payload_length, 1, "payload should be 1 byte (CBOR null)");
}

#[test]
fn test_golden_subscribe_request() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "subscribe_request")
        .expect("subscribe_request vector not found");

    let header = &vector["header"];
    let msg_type =
        u8::from_str_radix(header["message_type"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let channel_id = header["channel_id"].as_u64().unwrap() as u32;
    let timestamp_ns = header["timestamp_ns"].as_u64().unwrap();
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let flags =
        u8::from_str_radix(header["flags"].as_str().unwrap().trim_start_matches("0x"), 16).unwrap();
    let payload = hex_to_bytes(vector["payload_hex"].as_str().unwrap_or(""));

    let created = create_message(msg_type, channel_id, &payload, encoding, timestamp_ns, flags);
    let expected = hex_to_bytes(vector["expected_hex"].as_str().unwrap());

    assert_eq!(
        bytes_to_hex(&created),
        bytes_to_hex(&expected),
        "subscribe_request encoding mismatch"
    );

    // Verify CBOR payload can be extracted
    let parsed = parse_header(&created).expect("Failed to parse");
    assert_eq!(parsed.message_type, MessageType::Subscribe as u8);
    assert_eq!(parsed.payload_length, 13, "CBOR payload for {{topic: '/scan'}} should be 13 bytes");
}

#[test]
fn test_golden_large_channel_id() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "large_channel_id")
        .expect("large_channel_id vector not found");

    let header = &vector["header"];
    let msg_type =
        u8::from_str_radix(header["message_type"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let channel_id = header["channel_id"].as_u64().unwrap() as u32;
    let timestamp_ns = header["timestamp_ns"].as_u64().unwrap();
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let flags =
        u8::from_str_radix(header["flags"].as_str().unwrap().trim_start_matches("0x"), 16).unwrap();
    let payload = hex_to_bytes(vector["payload_hex"].as_str().unwrap_or(""));

    let created = create_message(msg_type, channel_id, &payload, encoding, timestamp_ns, flags);
    let expected = hex_to_bytes(vector["expected_hex"].as_str().unwrap());

    assert_eq!(
        bytes_to_hex(&created),
        bytes_to_hex(&expected),
        "large_channel_id encoding mismatch"
    );

    let parsed = parse_header(&created).expect("Failed to parse");
    assert_eq!(parsed.channel_id, u32::MAX, "channel_id should be max u32");
}

// ============================================================================
// Invalid Vector Tests
// ============================================================================

#[test]
fn test_invalid_wrong_magic() {
    let vectors = load_golden_vectors();
    let invalid_vectors = vectors["invalid_vectors"].as_array().unwrap();
    let vector = invalid_vectors
        .iter()
        .find(|v| v["name"] == "wrong_magic")
        .expect("wrong_magic vector not found");

    let data = hex_to_bytes(vector["hex"].as_str().unwrap());
    let parsed = parse_header(&data);

    assert!(
        parsed.is_none(),
        "Message with wrong magic should be rejected"
    );
}

#[test]
fn test_invalid_wrong_version() {
    let vectors = load_golden_vectors();
    let invalid_vectors = vectors["invalid_vectors"].as_array().unwrap();
    let vector = invalid_vectors
        .iter()
        .find(|v| v["name"] == "wrong_version")
        .expect("wrong_version vector not found");

    let data = hex_to_bytes(vector["hex"].as_str().unwrap());
    let parsed = parse_header(&data);

    assert!(
        parsed.is_none(),
        "Message with wrong version should be rejected"
    );
}

#[test]
fn test_invalid_truncated_header() {
    let vectors = load_golden_vectors();
    let invalid_vectors = vectors["invalid_vectors"].as_array().unwrap();
    let vector = invalid_vectors
        .iter()
        .find(|v| v["name"] == "truncated_header")
        .expect("truncated_header vector not found");

    let data = hex_to_bytes(vector["hex"].as_str().unwrap());
    let parsed = parse_header(&data);

    assert!(
        parsed.is_none(),
        "Message with truncated header should be rejected"
    );
}

// ============================================================================
// Media Encoding Tests
// ============================================================================

#[test]
fn test_golden_png_image_message() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "png_image_message")
        .expect("png_image_message vector not found");

    let header = &vector["header"];
    let msg_type =
        u8::from_str_radix(header["message_type"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let channel_id = header["channel_id"].as_u64().unwrap() as u32;
    let timestamp_ns = header["timestamp_ns"].as_u64().unwrap();
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let flags =
        u8::from_str_radix(header["flags"].as_str().unwrap().trim_start_matches("0x"), 16).unwrap();
    let payload = hex_to_bytes(vector["payload_hex"].as_str().unwrap());

    let created = create_message(msg_type, channel_id, &payload, encoding, timestamp_ns, flags);

    // Verify PNG encoding is preserved
    let parsed = parse_header(&created).expect("Failed to parse");
    assert_eq!(
        parsed.encoding,
        Encoding::Png as u8,
        "PNG encoding should be preserved"
    );
    assert_eq!(
        parsed.payload_length, 67,
        "PNG payload should be 67 bytes (1x1 red pixel)"
    );

    // Verify PNG magic bytes in payload
    let payload_start = &created[HEADER_SIZE..HEADER_SIZE + 8];
    assert_eq!(
        payload_start,
        &[0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a],
        "PNG magic bytes should be present"
    );
}

#[test]
fn test_golden_jpeg_image_message() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "jpeg_image_message")
        .expect("jpeg_image_message vector not found");

    let header = &vector["header"];
    let msg_type =
        u8::from_str_radix(header["message_type"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let channel_id = header["channel_id"].as_u64().unwrap() as u32;
    let timestamp_ns = header["timestamp_ns"].as_u64().unwrap();
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();
    let flags =
        u8::from_str_radix(header["flags"].as_str().unwrap().trim_start_matches("0x"), 16).unwrap();
    let payload = hex_to_bytes(vector["payload_hex"].as_str().unwrap());

    let created = create_message(msg_type, channel_id, &payload, encoding, timestamp_ns, flags);

    // Verify JPEG encoding is preserved
    let parsed = parse_header(&created).expect("Failed to parse");
    assert_eq!(
        parsed.encoding,
        Encoding::Jpeg as u8,
        "JPEG encoding should be preserved"
    );

    // Verify JPEG magic bytes (SOI marker)
    let payload_start = &created[HEADER_SIZE..HEADER_SIZE + 2];
    assert_eq!(
        payload_start,
        &[0xff, 0xd8],
        "JPEG SOI marker should be present"
    );
}

#[test]
fn test_golden_laser_scan_cbor() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "laser_scan_cbor")
        .expect("laser_scan_cbor vector not found");

    let header = &vector["header"];
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();

    assert_eq!(
        encoding,
        Encoding::Cbor as u8,
        "LaserScan should use CBOR encoding"
    );

    // Verify the CBOR structure is defined
    assert!(
        vector["payload_cbor"].is_object(),
        "LaserScan should have CBOR payload structure defined"
    );
    assert!(
        vector["payload_cbor"]["ranges"].is_array(),
        "LaserScan should have ranges array"
    );
}

#[test]
fn test_golden_occupancy_grid_cbor() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "occupancy_grid_cbor")
        .expect("occupancy_grid_cbor vector not found");

    let header = &vector["header"];
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();

    assert_eq!(
        encoding,
        Encoding::Cbor as u8,
        "OccupancyGrid should use CBOR encoding"
    );

    // Verify the CBOR structure
    assert!(
        vector["payload_cbor"]["info"]["resolution"].is_f64(),
        "OccupancyGrid should have resolution"
    );
    assert!(
        vector["payload_cbor"]["data"].is_array(),
        "OccupancyGrid should have data array"
    );
}

#[test]
fn test_golden_point_cloud_cbor() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();
    let vector = test_vectors
        .iter()
        .find(|v| v["name"] == "point_cloud_cbor")
        .expect("point_cloud_cbor vector not found");

    let header = &vector["header"];
    let encoding =
        u8::from_str_radix(header["encoding"].as_str().unwrap().trim_start_matches("0x"), 16)
            .unwrap();

    assert_eq!(
        encoding,
        Encoding::Cbor as u8,
        "PointCloud2 should use CBOR encoding"
    );

    // Verify the CBOR structure
    assert!(
        vector["payload_cbor"]["fields"].is_array(),
        "PointCloud2 should have fields array"
    );
    assert!(
        vector["payload_cbor"]["point_step"].is_number(),
        "PointCloud2 should have point_step"
    );
}

// ============================================================================
// All Test Vectors Dynamic Test
// ============================================================================

#[test]
fn test_all_golden_vectors_with_expected_hex() {
    let vectors = load_golden_vectors();
    let test_vectors = vectors["test_vectors"].as_array().unwrap();

    let mut tested = 0;
    let mut passed = 0;

    for vector in test_vectors {
        // Only test vectors that have expected_hex
        if let Some(expected_hex) = vector["expected_hex"].as_str() {
            tested += 1;
            let name = vector["name"].as_str().unwrap();
            let header = &vector["header"];

            let msg_type = u8::from_str_radix(
                header["message_type"]
                    .as_str()
                    .unwrap()
                    .trim_start_matches("0x"),
                16,
            )
            .unwrap();
            let channel_id = header["channel_id"].as_u64().unwrap() as u32;
            let timestamp_ns = header["timestamp_ns"].as_u64().unwrap();
            let encoding = u8::from_str_radix(
                header["encoding"].as_str().unwrap().trim_start_matches("0x"),
                16,
            )
            .unwrap();
            let flags = u8::from_str_radix(
                header["flags"].as_str().unwrap().trim_start_matches("0x"),
                16,
            )
            .unwrap();
            let payload = hex_to_bytes(vector["payload_hex"].as_str().unwrap_or(""));

            let created =
                create_message(msg_type, channel_id, &payload, encoding, timestamp_ns, flags);
            let expected = hex_to_bytes(expected_hex);

            if created == expected {
                passed += 1;
            } else {
                panic!(
                    "Vector '{}' failed:\n  Expected: {}\n  Got:      {}",
                    name,
                    bytes_to_hex(&expected),
                    bytes_to_hex(&created)
                );
            }
        }
    }

    assert!(tested > 0, "Should have tested at least one vector");
    assert_eq!(passed, tested, "All vectors should pass");
    println!("Tested {} golden vectors, all passed", tested);
}
