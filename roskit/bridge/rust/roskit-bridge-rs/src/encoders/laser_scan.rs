//! LaserScan encoder - uses binary float32 for efficient transmission

use bytes::{BufMut, Bytes, BytesMut};
use serde::Serialize;

/// Metadata for the laser scan (sent as CBOR header)
#[derive(Debug, Serialize)]
pub struct LaserScanMetadata {
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub time_increment: f32,
    pub scan_time: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub num_ranges: u32,
    pub num_intensities: u32,
}

/// Encoded laser scan with binary data
#[derive(Debug)]
pub struct EncodedLaserScan {
    pub binary_data: Bytes,
    pub metadata: LaserScanMetadata,
}

/// Encode a laser scan to binary format
///
/// Format:
/// - Header: CBOR-encoded metadata
/// - Ranges: float32 array (little-endian)
/// - Intensities: float32 array (little-endian, if present)
pub fn encode_laser_scan(
    ranges: &[f32],
    intensities: &[f32],
    angle_min: f32,
    angle_max: f32,
    angle_increment: f32,
    time_increment: f32,
    scan_time: f32,
    range_min: f32,
    range_max: f32,
) -> EncodedLaserScan {
    let num_ranges = ranges.len() as u32;
    let num_intensities = intensities.len() as u32;

    // Calculate buffer size: 4 bytes per float32
    let buffer_size = (num_ranges + num_intensities) as usize * 4;
    let mut buf = BytesMut::with_capacity(buffer_size);

    // Write ranges as little-endian float32
    for &range in ranges {
        buf.put_f32_le(range);
    }

    // Write intensities as little-endian float32
    for &intensity in intensities {
        buf.put_f32_le(intensity);
    }

    let metadata = LaserScanMetadata {
        angle_min,
        angle_max,
        angle_increment,
        time_increment,
        scan_time,
        range_min,
        range_max,
        num_ranges,
        num_intensities,
    };

    EncodedLaserScan {
        binary_data: buf.freeze(),
        metadata,
    }
}

/// Encode laser scan ranges only (no intensities)
pub fn encode_laser_scan_ranges_only(
    ranges: &[f32],
    angle_min: f32,
    angle_max: f32,
    angle_increment: f32,
    range_min: f32,
    range_max: f32,
) -> EncodedLaserScan {
    encode_laser_scan(
        ranges,
        &[],
        angle_min,
        angle_max,
        angle_increment,
        0.0,
        0.0,
        range_min,
        range_max,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_laser_scan() {
        let ranges = vec![1.0f32, 2.0, 3.0, 4.0];
        let intensities = vec![100.0f32, 200.0, 300.0, 400.0];

        let encoded = encode_laser_scan(
            &ranges,
            &intensities,
            -1.57,
            1.57,
            0.01,
            0.0,
            0.1,
            0.1,
            30.0,
        );

        assert_eq!(encoded.metadata.num_ranges, 4);
        assert_eq!(encoded.metadata.num_intensities, 4);
        // 8 floats * 4 bytes = 32 bytes
        assert_eq!(encoded.binary_data.len(), 32);
    }

    #[test]
    fn test_encode_ranges_only() {
        let ranges = vec![1.0f32, 2.0, 3.0];

        let encoded = encode_laser_scan_ranges_only(&ranges, -1.57, 1.57, 0.01, 0.1, 30.0);

        assert_eq!(encoded.metadata.num_ranges, 3);
        assert_eq!(encoded.metadata.num_intensities, 0);
        assert_eq!(encoded.binary_data.len(), 12);
    }
}
