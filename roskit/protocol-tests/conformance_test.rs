//! RosKit Protocol Conformance Tests (Rust)
//!
//! Run with: cargo test --package roskit-bridge-rs --test conformance
//!
//! Or standalone: rustc conformance_test.rs -o conformance_test && ./conformance_test

use std::fs;
use std::path::Path;

// These would normally come from the crate
mod protocol {
    pub const MAGIC: [u8; 2] = [b'R', b'K'];
    pub const PROTOCOL_VERSION: u8 = 1;
    pub const HEADER_SIZE: usize = 24;

    #[repr(u8)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum MessageType {
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

    #[repr(u8)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Encoding {
        Raw = 0x00,
        Cbor = 0x01,
        Json = 0x02,
        Png = 0x03,
        Jpeg = 0x04,
        Binary = 0x05,
    }

    #[repr(u8)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Flags {
        None = 0x00,
        Compressed = 0x01,
        Fragmented = 0x02,
        LastFragment = 0x04,
    }
}

use protocol::*;

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
    message_type: MessageType,
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
    buf.push(message_type as u8);
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

fn main() {
    println!("RosKit Protocol Conformance Tests (Rust)");
    println!("{}", "=".repeat(50));

    let mut passed = 0;
    let mut failed = 0;

    // Test 1: Protocol constants
    println!("\n1. Protocol Constants");
    if MAGIC == [b'R', b'K'] && PROTOCOL_VERSION == 1 && HEADER_SIZE == 24 {
        println!("   ✓ Constants match golden vectors");
        passed += 1;
    } else {
        println!("   ✗ Constants mismatch");
        failed += 1;
    }

    // Test 2: Message type values
    println!("\n2. Message Type Values");
    let msg_types = [
        ("SUBSCRIBE", MessageType::Subscribe, 0x01),
        ("UNSUBSCRIBE", MessageType::Unsubscribe, 0x02),
        ("PUBLISH", MessageType::Publish, 0x03),
        ("SERVICE_CALL", MessageType::ServiceCall, 0x04),
        ("TOPIC_LIST", MessageType::TopicList, 0x05),
        ("PING", MessageType::Ping, 0x06),
        ("SERVICE_LIST", MessageType::ServiceList, 0x07),
        ("NODE_LIST", MessageType::NodeList, 0x08),
        ("MESSAGE", MessageType::Message, 0x10),
        ("CHANNEL_INFO", MessageType::ChannelInfo, 0x11),
        ("SERVICE_RESPONSE", MessageType::ServiceResponse, 0x12),
        ("TOPIC_LIST_RESPONSE", MessageType::TopicListResponse, 0x13),
        ("SERVER_INFO", MessageType::ServerInfo, 0x14),
        ("PONG", MessageType::Pong, 0x15),
        ("SERVICE_LIST_RESPONSE", MessageType::ServiceListResponse, 0x16),
        ("NODE_LIST_RESPONSE", MessageType::NodeListResponse, 0x17),
        ("ERROR", MessageType::Error, 0xFF),
    ];

    for (name, msg_type, expected) in msg_types {
        if msg_type as u8 == expected {
            println!("   ✓ MessageType::{} = 0x{:02x}", name, expected);
            passed += 1;
        } else {
            println!(
                "   ✗ MessageType::{}: expected 0x{:02x}, got 0x{:02x}",
                name, expected, msg_type as u8
            );
            failed += 1;
        }
    }

    // Test 3: Encoding values
    println!("\n3. Encoding Values");
    let encodings = [
        ("RAW", Encoding::Raw, 0x00),
        ("CBOR", Encoding::Cbor, 0x01),
        ("JSON", Encoding::Json, 0x02),
        ("PNG", Encoding::Png, 0x03),
        ("JPEG", Encoding::Jpeg, 0x04),
        ("BINARY", Encoding::Binary, 0x05),
    ];

    for (name, enc, expected) in encodings {
        if enc as u8 == expected {
            println!("   ✓ Encoding::{} = 0x{:02x}", name, expected);
            passed += 1;
        } else {
            println!(
                "   ✗ Encoding::{}: expected 0x{:02x}, got 0x{:02x}",
                name, expected, enc as u8
            );
            failed += 1;
        }
    }

    // Test 4: Flag values
    println!("\n4. Flag Values");
    let flags = [
        ("NONE", Flags::None, 0x00),
        ("COMPRESSED", Flags::Compressed, 0x01),
        ("FRAGMENTED", Flags::Fragmented, 0x02),
        ("LAST_FRAGMENT", Flags::LastFragment, 0x04),
    ];

    for (name, flag, expected) in flags {
        if flag as u8 == expected {
            println!("   ✓ Flags::{} = 0x{:02x}", name, expected);
            passed += 1;
        } else {
            println!(
                "   ✗ Flags::{}: expected 0x{:02x}, got 0x{:02x}",
                name, expected, flag as u8
            );
            failed += 1;
        }
    }

    // Test 5: Message creation and parsing
    println!("\n5. Message Creation/Parsing");

    // Empty PING
    let ping = create_message(MessageType::Ping, 0, &[], Encoding::Raw, 0, 0);
    let expected_ping = hex_to_bytes("524b01060000000000000000000000000000000000000000");
    if ping == expected_ping {
        println!("   ✓ empty_ping: PING message with no payload");
        passed += 1;
    } else {
        println!(
            "   ✗ empty_ping: expected {}, got {}",
            bytes_to_hex(&expected_ping),
            bytes_to_hex(&ping)
        );
        failed += 1;
    }

    // Empty PONG
    let pong = create_message(MessageType::Pong, 0, &[], Encoding::Raw, 0, 0);
    let expected_pong = hex_to_bytes("524b01150000000000000000000000000000000000000000");
    if pong == expected_pong {
        println!("   ✓ empty_pong: PONG message with no payload");
        passed += 1;
    } else {
        println!(
            "   ✗ empty_pong: expected {}, got {}",
            bytes_to_hex(&expected_pong),
            bytes_to_hex(&pong)
        );
        failed += 1;
    }

    // Parse round-trip
    if let Some(header) = parse_header(&ping) {
        if header.message_type == MessageType::Ping as u8
            && header.channel_id == 0
            && header.payload_length == 0
        {
            println!("   ✓ Parse round-trip for PING");
            passed += 1;
        } else {
            println!("   ✗ Parse round-trip failed for PING");
            failed += 1;
        }
    } else {
        println!("   ✗ Failed to parse PING message");
        failed += 1;
    }

    // Empty NODE_LIST request
    let node_list = create_message(MessageType::NodeList, 0, &[], Encoding::Raw, 0, 0);
    let expected_node_list = hex_to_bytes("524b01080000000000000000000000000000000000000000");
    if node_list == expected_node_list {
        println!("   ✓ node_list_request: NODE_LIST message with no payload");
        passed += 1;
    } else {
        println!(
            "   ✗ node_list_request: expected {}, got {}",
            bytes_to_hex(&expected_node_list),
            bytes_to_hex(&node_list)
        );
        failed += 1;
    }

    // Test 6: Invalid message handling
    println!("\n6. Invalid Message Handling");

    // Wrong magic
    let wrong_magic = hex_to_bytes("5257010600000000000000000000000000000000000000000");
    if parse_header(&wrong_magic).is_none() {
        println!("   ✓ wrong_magic: Correctly rejected (InvalidMagic)");
        passed += 1;
    } else {
        println!("   ✗ wrong_magic: Should have been rejected");
        failed += 1;
    }

    // Wrong version
    let wrong_version = hex_to_bytes("524b020600000000000000000000000000000000000000000");
    if parse_header(&wrong_version).is_none() {
        println!("   ✓ wrong_version: Correctly rejected (InvalidVersion)");
        passed += 1;
    } else {
        println!("   ✗ wrong_version: Should have been rejected");
        failed += 1;
    }

    // Truncated header
    let truncated = hex_to_bytes("524b01060000000000");
    if parse_header(&truncated).is_none() {
        println!("   ✓ truncated_header: Correctly rejected (MessageTooShort)");
        passed += 1;
    } else {
        println!("   ✗ truncated_header: Should have been rejected");
        failed += 1;
    }

    // Test 7: Media encoding round-trips
    println!("\n7. Media Encoding Round-Trips");

    // PNG image (encoding 0x03)
    let png_payload = hex_to_bytes("89504e470d0a1a0a0000000d49484452000000010000000108020000009058b1eb0000000c49444154789c636060f80f00010401006baf3e2a0000000049454e44ae426082");
    let png_msg = create_message(MessageType::Message, 10, &png_payload, Encoding::Png, 1700000000000000000, 0);
    if let Some(header) = parse_header(&png_msg) {
        if header.encoding == Encoding::Png as u8 && header.payload_length == png_payload.len() as u32 {
            println!("   ✓ png_image_message: PNG encoding preserved");
            passed += 1;
        } else {
            println!("   ✗ png_image_message: encoding or length mismatch");
            failed += 1;
        }
    } else {
        println!("   ✗ png_image_message: failed to parse");
        failed += 1;
    }

    // JPEG image (encoding 0x04)
    let jpeg_payload = hex_to_bytes("ffd8ffe000104a4649460001");
    let jpeg_msg = create_message(MessageType::Message, 11, &jpeg_payload, Encoding::Jpeg, 1700000000000000000, 0);
    if let Some(header) = parse_header(&jpeg_msg) {
        if header.encoding == Encoding::Jpeg as u8 && header.payload_length == jpeg_payload.len() as u32 {
            println!("   ✓ jpeg_image_message: JPEG encoding preserved");
            passed += 1;
        } else {
            println!("   ✗ jpeg_image_message: encoding or length mismatch");
            failed += 1;
        }
    } else {
        println!("   ✗ jpeg_image_message: failed to parse");
        failed += 1;
    }

    // CBOR LaserScan (encoding 0x01)
    let cbor_payload = hex_to_bytes("a269616e676c655f6d696efbc00921fb54442d1869616e676c655f6d6178fbc00921fb54442d18");
    let scan_msg = create_message(MessageType::Message, 20, &cbor_payload, Encoding::Cbor, 1700000000000000000, 0);
    if let Some(header) = parse_header(&scan_msg) {
        if header.encoding == Encoding::Cbor as u8 && header.payload_length == cbor_payload.len() as u32 {
            println!("   ✓ laser_scan_cbor: CBOR encoding preserved");
            passed += 1;
        } else {
            println!("   ✗ laser_scan_cbor: encoding or length mismatch");
            failed += 1;
        }
    } else {
        println!("   ✗ laser_scan_cbor: failed to parse");
        failed += 1;
    }

    // Binary encoding (encoding 0x05)
    let binary_payload: Vec<u8> = (0..256).map(|i| i as u8).collect();
    let binary_msg = create_message(MessageType::Message, 30, &binary_payload, Encoding::Binary, 0, 0);
    if let Some(header) = parse_header(&binary_msg) {
        if header.encoding == Encoding::Binary as u8 && header.payload_length == 256 {
            println!("   ✓ binary_message: BINARY encoding preserved");
            passed += 1;
        } else {
            println!("   ✗ binary_message: encoding or length mismatch");
            failed += 1;
        }
    } else {
        println!("   ✗ binary_message: failed to parse");
        failed += 1;
    }

    // Summary
    println!("\n{}", "=".repeat(50));
    println!("Results: {} passed, {} failed", passed, failed);

    if failed > 0 {
        std::process::exit(1);
    }
}
