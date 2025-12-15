//! Media Encoding Round-Trip Tests for RosKit Rust Bridge
//!
//! Tests for encoding and decoding media payloads (PNG, JPEG, CBOR)
//! including image validation and ROS2 message structure preservation.
//!
//! Run with: cargo test --test media_roundtrip

use std::io::Cursor;

// Protocol constants
const MAGIC: [u8; 2] = [b'R', b'K'];
const PROTOCOL_VERSION: u8 = 1;
const HEADER_SIZE: usize = 24;

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
enum MessageType {
    Message = 0x10,
}

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

fn create_message(
    message_type: u8,
    channel_id: u32,
    payload: &[u8],
    encoding: u8,
    timestamp_ns: u64,
    flags: u8,
) -> Vec<u8> {
    let mut buf = Vec::with_capacity(HEADER_SIZE + payload.len());

    buf.extend_from_slice(&MAGIC);
    buf.push(PROTOCOL_VERSION);
    buf.push(message_type);
    buf.extend_from_slice(&channel_id.to_be_bytes());
    buf.extend_from_slice(&timestamp_ns.to_be_bytes());
    buf.push(encoding);
    buf.push(flags);
    buf.extend_from_slice(&[0u8; 2]);
    buf.extend_from_slice(&(payload.len() as u32).to_be_bytes());
    buf.extend_from_slice(payload);

    buf
}

#[derive(Debug)]
struct ParsedHeader {
    message_type: u8,
    channel_id: u32,
    timestamp_ns: u64,
    encoding: u8,
    #[allow(dead_code)]
    flags: u8,
    payload_length: u32,
}

fn parse_message(data: &[u8]) -> Option<(ParsedHeader, &[u8])> {
    if data.len() < HEADER_SIZE {
        return None;
    }

    if data[0] != MAGIC[0] || data[1] != MAGIC[1] {
        return None;
    }

    if data[2] != PROTOCOL_VERSION {
        return None;
    }

    let payload_length = u32::from_be_bytes([data[20], data[21], data[22], data[23]]);

    if data.len() < HEADER_SIZE + payload_length as usize {
        return None;
    }

    let header = ParsedHeader {
        message_type: data[3],
        channel_id: u32::from_be_bytes([data[4], data[5], data[6], data[7]]),
        timestamp_ns: u64::from_be_bytes([
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15],
        ]),
        encoding: data[16],
        flags: data[17],
        payload_length,
    };

    let payload = &data[HEADER_SIZE..HEADER_SIZE + payload_length as usize];
    Some((header, payload))
}

// ============================================================================
// PNG Round-Trip Tests
// ============================================================================

/// Minimal valid 1x1 red PNG (67 bytes)
fn create_minimal_png() -> Vec<u8> {
    hex::decode(
        "89504e470d0a1a0a0000000d49484452000000010000000108020000009058b1eb\
         0000000c49444154789c636060f80f00010401006baf3e2a0000000049454e44ae426082",
    )
    .unwrap()
}

#[test]
fn test_png_encoding_preserved() {
    let png_data = create_minimal_png();
    let message = create_message(
        MessageType::Message as u8,
        10,
        &png_data,
        Encoding::Png as u8,
        1700000000000000000,
        0,
    );

    let (header, payload) = parse_message(&message).expect("Failed to parse");

    assert_eq!(header.encoding, Encoding::Png as u8);
    assert_eq!(header.payload_length, png_data.len() as u32);
    assert_eq!(payload, png_data.as_slice());
}

#[test]
fn test_png_magic_bytes_valid() {
    let png_data = create_minimal_png();

    // PNG magic bytes: 89 50 4e 47 0d 0a 1a 0a
    assert_eq!(
        &png_data[0..8],
        &[0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]
    );
}

#[test]
fn test_png_decode_with_image_crate() {
    let png_data = create_minimal_png();

    // Use the image crate to decode and verify
    let cursor = Cursor::new(&png_data);
    let img = image::io::Reader::new(cursor)
        .with_guessed_format()
        .expect("Failed to guess format")
        .decode()
        .expect("Failed to decode PNG");

    assert_eq!(img.width(), 1);
    assert_eq!(img.height(), 1);
}

#[test]
fn test_png_round_trip_with_metadata() {
    let png_data = create_minimal_png();

    // Simulate OccupancyGrid-style message with CBOR metadata prefix
    let metadata = ciborium::Value::Map(vec![
        (
            ciborium::Value::Text("width".into()),
            ciborium::Value::Integer(1.into()),
        ),
        (
            ciborium::Value::Text("height".into()),
            ciborium::Value::Integer(1.into()),
        ),
        (
            ciborium::Value::Text("resolution".into()),
            ciborium::Value::Float(0.05),
        ),
    ]);

    let mut metadata_bytes = Vec::new();
    ciborium::into_writer(&metadata, &mut metadata_bytes).expect("Failed to encode CBOR");

    // Format: [4-byte metadata length][CBOR metadata][PNG data]
    let mut payload = Vec::new();
    payload.extend_from_slice(&(metadata_bytes.len() as u32).to_be_bytes());
    payload.extend_from_slice(&metadata_bytes);
    payload.extend_from_slice(&png_data);

    let message = create_message(
        MessageType::Message as u8,
        21,
        &payload,
        Encoding::Png as u8,
        1700000000000000000,
        0,
    );

    let (header, parsed_payload) = parse_message(&message).expect("Failed to parse");

    assert_eq!(header.encoding, Encoding::Png as u8);

    // Extract metadata length
    let metadata_len =
        u32::from_be_bytes([parsed_payload[0], parsed_payload[1], parsed_payload[2], parsed_payload[3]]) as usize;

    // Extract and verify CBOR metadata
    let metadata_cbor = &parsed_payload[4..4 + metadata_len];
    let decoded_metadata: ciborium::Value =
        ciborium::from_reader(metadata_cbor).expect("Failed to decode CBOR");

    if let ciborium::Value::Map(map) = decoded_metadata {
        let width = map
            .iter()
            .find(|(k, _)| k == &ciborium::Value::Text("width".into()))
            .map(|(_, v)| v);
        assert!(width.is_some());
    } else {
        panic!("Expected CBOR map");
    }

    // Extract and verify PNG
    let png_bytes = &parsed_payload[4 + metadata_len..];
    assert_eq!(png_bytes, png_data.as_slice());
}

// ============================================================================
// JPEG Round-Trip Tests
// ============================================================================

/// Create minimal JPEG data (just SOI + EOI markers for testing)
fn create_minimal_jpeg_header() -> Vec<u8> {
    // Minimal JPEG: SOI marker (FFD8) + APP0 marker start
    hex::decode("ffd8ffe000104a46494600010100000100010000").unwrap()
}

#[test]
fn test_jpeg_encoding_preserved() {
    let jpeg_data = create_minimal_jpeg_header();
    let message = create_message(
        MessageType::Message as u8,
        11,
        &jpeg_data,
        Encoding::Jpeg as u8,
        1700000000000000000,
        0,
    );

    let (header, payload) = parse_message(&message).expect("Failed to parse");

    assert_eq!(header.encoding, Encoding::Jpeg as u8);
    assert_eq!(payload, jpeg_data.as_slice());
}

#[test]
fn test_jpeg_soi_marker() {
    let jpeg_data = create_minimal_jpeg_header();

    // JPEG SOI marker: FF D8
    assert_eq!(&jpeg_data[0..2], &[0xff, 0xd8]);
}

#[test]
fn test_jpeg_app0_marker() {
    let jpeg_data = create_minimal_jpeg_header();

    // APP0 marker: FF E0
    assert_eq!(&jpeg_data[2..4], &[0xff, 0xe0]);

    // JFIF identifier: "JFIF\0"
    assert_eq!(&jpeg_data[6..11], b"JFIF\0");
}

// ============================================================================
// CBOR Round-Trip Tests
// ============================================================================

#[test]
fn test_cbor_laser_scan_round_trip() {
    // Create LaserScan-like CBOR structure
    let laser_scan = ciborium::Value::Map(vec![
        (
            ciborium::Value::Text("header".into()),
            ciborium::Value::Map(vec![
                (
                    ciborium::Value::Text("stamp".into()),
                    ciborium::Value::Map(vec![
                        (
                            ciborium::Value::Text("sec".into()),
                            ciborium::Value::Integer(1700000000.into()),
                        ),
                        (
                            ciborium::Value::Text("nanosec".into()),
                            ciborium::Value::Integer(0.into()),
                        ),
                    ]),
                ),
                (
                    ciborium::Value::Text("frame_id".into()),
                    ciborium::Value::Text("laser".into()),
                ),
            ]),
        ),
        (
            ciborium::Value::Text("angle_min".into()),
            ciborium::Value::Float(-1.5707963),
        ),
        (
            ciborium::Value::Text("angle_max".into()),
            ciborium::Value::Float(1.5707963),
        ),
        (
            ciborium::Value::Text("ranges".into()),
            ciborium::Value::Array(vec![
                ciborium::Value::Float(1.0),
                ciborium::Value::Float(2.0),
                ciborium::Value::Float(3.0),
                ciborium::Value::Float(4.0),
                ciborium::Value::Float(5.0),
            ]),
        ),
    ]);

    let mut cbor_bytes = Vec::new();
    ciborium::into_writer(&laser_scan, &mut cbor_bytes).expect("Failed to encode CBOR");

    let message = create_message(
        MessageType::Message as u8,
        20,
        &cbor_bytes,
        Encoding::Cbor as u8,
        1700000000000000000,
        0,
    );

    let (header, payload) = parse_message(&message).expect("Failed to parse");

    assert_eq!(header.encoding, Encoding::Cbor as u8);
    assert_eq!(payload, cbor_bytes.as_slice());

    // Decode and verify structure
    let decoded: ciborium::Value = ciborium::from_reader(payload).expect("Failed to decode CBOR");

    if let ciborium::Value::Map(map) = decoded {
        // Verify ranges array exists
        let ranges = map
            .iter()
            .find(|(k, _)| k == &ciborium::Value::Text("ranges".into()))
            .map(|(_, v)| v);

        assert!(ranges.is_some());
        if let Some(ciborium::Value::Array(arr)) = ranges {
            assert_eq!(arr.len(), 5);
        }
    } else {
        panic!("Expected CBOR map");
    }
}

#[test]
fn test_cbor_occupancy_grid_round_trip() {
    // Create OccupancyGrid-like CBOR structure
    let grid = ciborium::Value::Map(vec![
        (
            ciborium::Value::Text("info".into()),
            ciborium::Value::Map(vec![
                (
                    ciborium::Value::Text("resolution".into()),
                    ciborium::Value::Float(0.05),
                ),
                (
                    ciborium::Value::Text("width".into()),
                    ciborium::Value::Integer(100.into()),
                ),
                (
                    ciborium::Value::Text("height".into()),
                    ciborium::Value::Integer(100.into()),
                ),
            ]),
        ),
        (
            ciborium::Value::Text("data".into()),
            ciborium::Value::Array(vec![
                ciborium::Value::Integer(0.into()),
                ciborium::Value::Integer(100.into()),
                ciborium::Value::Integer((-1i8).into()),
                ciborium::Value::Integer(50.into()),
            ]),
        ),
    ]);

    let mut cbor_bytes = Vec::new();
    ciborium::into_writer(&grid, &mut cbor_bytes).expect("Failed to encode CBOR");

    let message = create_message(
        MessageType::Message as u8,
        21,
        &cbor_bytes,
        Encoding::Cbor as u8,
        1700000000000000000,
        0,
    );

    let (header, payload) = parse_message(&message).expect("Failed to parse");

    assert_eq!(header.encoding, Encoding::Cbor as u8);

    // Decode and verify
    let decoded: ciborium::Value = ciborium::from_reader(payload).expect("Failed to decode CBOR");

    if let ciborium::Value::Map(map) = decoded {
        let info = map
            .iter()
            .find(|(k, _)| k == &ciborium::Value::Text("info".into()))
            .map(|(_, v)| v);

        assert!(info.is_some());
    }
}

#[test]
fn test_cbor_point_cloud_round_trip() {
    // Create PointCloud2-like CBOR structure
    let cloud = ciborium::Value::Map(vec![
        (
            ciborium::Value::Text("header".into()),
            ciborium::Value::Map(vec![(
                ciborium::Value::Text("frame_id".into()),
                ciborium::Value::Text("lidar".into()),
            )]),
        ),
        (
            ciborium::Value::Text("height".into()),
            ciborium::Value::Integer(1.into()),
        ),
        (
            ciborium::Value::Text("width".into()),
            ciborium::Value::Integer(100.into()),
        ),
        (
            ciborium::Value::Text("fields".into()),
            ciborium::Value::Array(vec![
                ciborium::Value::Map(vec![
                    (
                        ciborium::Value::Text("name".into()),
                        ciborium::Value::Text("x".into()),
                    ),
                    (
                        ciborium::Value::Text("offset".into()),
                        ciborium::Value::Integer(0.into()),
                    ),
                    (
                        ciborium::Value::Text("datatype".into()),
                        ciborium::Value::Integer(7.into()),
                    ), // FLOAT32
                ]),
                ciborium::Value::Map(vec![
                    (
                        ciborium::Value::Text("name".into()),
                        ciborium::Value::Text("y".into()),
                    ),
                    (
                        ciborium::Value::Text("offset".into()),
                        ciborium::Value::Integer(4.into()),
                    ),
                    (
                        ciborium::Value::Text("datatype".into()),
                        ciborium::Value::Integer(7.into()),
                    ),
                ]),
                ciborium::Value::Map(vec![
                    (
                        ciborium::Value::Text("name".into()),
                        ciborium::Value::Text("z".into()),
                    ),
                    (
                        ciborium::Value::Text("offset".into()),
                        ciborium::Value::Integer(8.into()),
                    ),
                    (
                        ciborium::Value::Text("datatype".into()),
                        ciborium::Value::Integer(7.into()),
                    ),
                ]),
            ]),
        ),
        (
            ciborium::Value::Text("point_step".into()),
            ciborium::Value::Integer(12.into()),
        ),
        (
            ciborium::Value::Text("is_dense".into()),
            ciborium::Value::Bool(true),
        ),
    ]);

    let mut cbor_bytes = Vec::new();
    ciborium::into_writer(&cloud, &mut cbor_bytes).expect("Failed to encode CBOR");

    let message = create_message(
        MessageType::Message as u8,
        22,
        &cbor_bytes,
        Encoding::Cbor as u8,
        1700000000000000000,
        0,
    );

    let (header, payload) = parse_message(&message).expect("Failed to parse");

    assert_eq!(header.encoding, Encoding::Cbor as u8);

    // Decode and verify fields
    let decoded: ciborium::Value = ciborium::from_reader(payload).expect("Failed to decode CBOR");

    if let ciborium::Value::Map(map) = decoded {
        let fields = map
            .iter()
            .find(|(k, _)| k == &ciborium::Value::Text("fields".into()))
            .map(|(_, v)| v);

        if let Some(ciborium::Value::Array(arr)) = fields {
            assert_eq!(arr.len(), 3);
        } else {
            panic!("Expected fields array");
        }
    }
}

// ============================================================================
// Binary Encoding Tests
// ============================================================================

#[test]
fn test_binary_float32_array_round_trip() {
    // Simulate binary-encoded float32 array (like LaserScan ranges)
    let floats: Vec<f32> = vec![1.0, 2.5, 3.7, 4.2, 5.9];
    let mut binary_data = Vec::new();

    for f in &floats {
        binary_data.extend_from_slice(&f.to_le_bytes());
    }

    let message = create_message(
        MessageType::Message as u8,
        30,
        &binary_data,
        Encoding::Binary as u8,
        0,
        0,
    );

    let (header, payload) = parse_message(&message).expect("Failed to parse");

    assert_eq!(header.encoding, Encoding::Binary as u8);
    assert_eq!(payload.len(), floats.len() * 4);

    // Decode floats
    let decoded_floats: Vec<f32> = payload
        .chunks_exact(4)
        .map(|chunk| f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
        .collect();

    assert_eq!(decoded_floats, floats);
}

#[test]
fn test_binary_point_cloud_xyz() {
    // Simulate XYZ point cloud (3 points, 12 bytes each)
    #[derive(Debug, PartialEq)]
    struct Point {
        x: f32,
        y: f32,
        z: f32,
    }

    let points = vec![
        Point {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        },
        Point {
            x: 4.0,
            y: 5.0,
            z: 6.0,
        },
        Point {
            x: 7.0,
            y: 8.0,
            z: 9.0,
        },
    ];

    let mut binary_data = Vec::new();
    for p in &points {
        binary_data.extend_from_slice(&p.x.to_le_bytes());
        binary_data.extend_from_slice(&p.y.to_le_bytes());
        binary_data.extend_from_slice(&p.z.to_le_bytes());
    }

    let message = create_message(
        MessageType::Message as u8,
        31,
        &binary_data,
        Encoding::Binary as u8,
        0,
        0,
    );

    let (header, payload) = parse_message(&message).expect("Failed to parse");

    assert_eq!(header.encoding, Encoding::Binary as u8);
    assert_eq!(payload.len(), 36); // 3 points * 12 bytes

    // Decode points
    let decoded_points: Vec<Point> = payload
        .chunks_exact(12)
        .map(|chunk| Point {
            x: f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]),
            y: f32::from_le_bytes([chunk[4], chunk[5], chunk[6], chunk[7]]),
            z: f32::from_le_bytes([chunk[8], chunk[9], chunk[10], chunk[11]]),
        })
        .collect();

    assert_eq!(decoded_points, points);
}

// ============================================================================
// Large Payload Tests
// ============================================================================

#[test]
fn test_large_binary_payload() {
    // Simulate large point cloud (10000 points)
    let num_points = 10000;
    let point_size = 12; // XYZ float32
    let mut binary_data = Vec::with_capacity(num_points * point_size);

    for i in 0..num_points {
        let x = i as f32;
        let y = (i * 2) as f32;
        let z = (i * 3) as f32;
        binary_data.extend_from_slice(&x.to_le_bytes());
        binary_data.extend_from_slice(&y.to_le_bytes());
        binary_data.extend_from_slice(&z.to_le_bytes());
    }

    let message = create_message(
        MessageType::Message as u8,
        40,
        &binary_data,
        Encoding::Binary as u8,
        0,
        0,
    );

    let (header, payload) = parse_message(&message).expect("Failed to parse");

    assert_eq!(header.payload_length, (num_points * point_size) as u32);
    assert_eq!(payload.len(), num_points * point_size);

    // Spot check first and last points
    let first_x = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
    assert_eq!(first_x, 0.0);

    let last_offset = (num_points - 1) * point_size;
    let last_x = f32::from_le_bytes([
        payload[last_offset],
        payload[last_offset + 1],
        payload[last_offset + 2],
        payload[last_offset + 3],
    ]);
    assert_eq!(last_x, (num_points - 1) as f32);
}

#[test]
fn test_large_cbor_payload() {
    // Create large CBOR array (1000 elements)
    let large_array: Vec<ciborium::Value> = (0..1000)
        .map(|i| ciborium::Value::Integer(i.into()))
        .collect();

    let cbor_value = ciborium::Value::Map(vec![(
        ciborium::Value::Text("data".into()),
        ciborium::Value::Array(large_array),
    )]);

    let mut cbor_bytes = Vec::new();
    ciborium::into_writer(&cbor_value, &mut cbor_bytes).expect("Failed to encode CBOR");

    let message = create_message(
        MessageType::Message as u8,
        41,
        &cbor_bytes,
        Encoding::Cbor as u8,
        0,
        0,
    );

    let (header, payload) = parse_message(&message).expect("Failed to parse");

    assert_eq!(header.encoding, Encoding::Cbor as u8);

    // Decode and verify
    let decoded: ciborium::Value = ciborium::from_reader(payload).expect("Failed to decode CBOR");

    if let ciborium::Value::Map(map) = decoded {
        let data = map
            .iter()
            .find(|(k, _)| k == &ciborium::Value::Text("data".into()))
            .map(|(_, v)| v);

        if let Some(ciborium::Value::Array(arr)) = data {
            assert_eq!(arr.len(), 1000);
        } else {
            panic!("Expected data array");
        }
    }
}

// ============================================================================
// Channel and Timestamp Preservation Tests
// ============================================================================

#[test]
fn test_channel_id_preserved_for_media() {
    let png_data = create_minimal_png();

    for channel_id in [0u32, 1, 100, 1000, u32::MAX] {
        let message = create_message(
            MessageType::Message as u8,
            channel_id,
            &png_data,
            Encoding::Png as u8,
            0,
            0,
        );

        let (header, _) = parse_message(&message).expect("Failed to parse");
        assert_eq!(header.channel_id, channel_id);
    }
}

#[test]
fn test_timestamp_preserved_for_media() {
    let png_data = create_minimal_png();

    for timestamp in [0u64, 1, 1700000000000000000, u64::MAX] {
        let message = create_message(
            MessageType::Message as u8,
            1,
            &png_data,
            Encoding::Png as u8,
            timestamp,
            0,
        );

        let (header, _) = parse_message(&message).expect("Failed to parse");
        assert_eq!(header.timestamp_ns, timestamp);
    }
}
