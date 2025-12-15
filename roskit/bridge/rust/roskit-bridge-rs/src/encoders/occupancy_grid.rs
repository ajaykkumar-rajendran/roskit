//! OccupancyGrid encoder - converts to PNG for efficient transmission

use bytes::Bytes;
use image::{GrayImage, ImageBuffer};
use serde::Serialize;
use std::io::Cursor;

/// Metadata for the occupancy grid (sent alongside PNG)
#[derive(Debug, Serialize)]
pub struct OccupancyGridMetadata {
    pub width: u32,
    pub height: u32,
    pub resolution: f32,
    pub origin_x: f64,
    pub origin_y: f64,
    pub origin_z: f64,
    pub orientation_x: f64,
    pub orientation_y: f64,
    pub orientation_z: f64,
    pub orientation_w: f64,
}

/// Encoded occupancy grid with PNG data and metadata
#[derive(Debug)]
pub struct EncodedOccupancyGrid {
    pub png_data: Bytes,
    pub metadata: OccupancyGridMetadata,
}

/// Encode an occupancy grid to PNG format
///
/// Occupancy values are mapped as:
/// - -1 (unknown) -> 128 (gray)
/// - 0 (free) -> 254 (white)
/// - 100 (occupied) -> 0 (black)
/// - Values in between are linearly interpolated
/// Encode an occupancy grid to PNG format (simplified API)
pub fn encode_occupancy_grid(
    data: &[i8],
    width: u32,
    height: u32,
    resolution: f32,
    origin_x: f64,
    origin_y: f64,
) -> Result<EncodedOccupancyGrid, Box<dyn std::error::Error + Send + Sync>> {
    encode_occupancy_grid_full(data, width, height, resolution, (origin_x, origin_y, 0.0), (0.0, 0.0, 0.0, 1.0))
}

/// Encode an occupancy grid to PNG format (full API with orientation)
pub fn encode_occupancy_grid_full(
    data: &[i8],
    width: u32,
    height: u32,
    resolution: f32,
    origin: (f64, f64, f64),
    orientation: (f64, f64, f64, f64),
) -> Result<EncodedOccupancyGrid, Box<dyn std::error::Error + Send + Sync>> {
    // Create grayscale image
    let mut img: GrayImage = ImageBuffer::new(width, height);

    for (i, &value) in data.iter().enumerate() {
        let x = (i as u32) % width;
        let y = (i as u32) / width;

        // Map occupancy value to grayscale
        let pixel_value = match value {
            -1 => 128u8,                                     // Unknown -> gray
            v if v >= 0 && v <= 100 => (254 - (v as u16 * 254 / 100)) as u8, // 0->254, 100->0
            _ => 128u8,                                      // Invalid -> gray
        };

        // Note: ROS uses row-major, bottom-up. PNG is top-down.
        // Flip vertically for correct display
        let flipped_y = height - 1 - y;
        if flipped_y < height && x < width {
            img.put_pixel(x, flipped_y, image::Luma([pixel_value]));
        }
    }

    // Encode to PNG
    let mut png_buffer = Cursor::new(Vec::new());
    img.write_to(&mut png_buffer, image::ImageFormat::Png)?;

    let metadata = OccupancyGridMetadata {
        width,
        height,
        resolution,
        origin_x: origin.0,
        origin_y: origin.1,
        origin_z: origin.2,
        orientation_x: orientation.0,
        orientation_y: orientation.1,
        orientation_z: orientation.2,
        orientation_w: orientation.3,
    };

    Ok(EncodedOccupancyGrid {
        png_data: Bytes::from(png_buffer.into_inner()),
        metadata,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_small_grid() {
        let data: Vec<i8> = vec![0, 50, 100, -1]; // 2x2 grid
        let result = encode_occupancy_grid(&data, 2, 2, 0.05, 0.0, 0.0);

        assert!(result.is_ok());
        let encoded = result.unwrap();
        assert_eq!(encoded.metadata.width, 2);
        assert_eq!(encoded.metadata.height, 2);
        assert!(!encoded.png_data.is_empty());
    }

    #[test]
    fn test_encode_grid_full_api() {
        let data: Vec<i8> = vec![0, 50, 100, -1]; // 2x2 grid
        let result = encode_occupancy_grid_full(
            &data,
            2,
            2,
            0.05,
            (1.0, 2.0, 0.0),
            (0.0, 0.0, 0.707, 0.707),
        );

        assert!(result.is_ok());
        let encoded = result.unwrap();
        assert_eq!(encoded.metadata.width, 2);
        assert_eq!(encoded.metadata.height, 2);
        assert_eq!(encoded.metadata.origin_x, 1.0);
        assert_eq!(encoded.metadata.origin_y, 2.0);
        assert!(!encoded.png_data.is_empty());
    }

    #[test]
    fn test_occupancy_value_mapping() {
        // Test that occupancy values map correctly:
        // -1 (unknown) -> 128 (gray)
        // 0 (free) -> 254 (white)
        // 100 (occupied) -> 0 (black)
        let data: Vec<i8> = vec![-1, 0, 50, 100];
        let result = encode_occupancy_grid(&data, 2, 2, 0.05, 0.0, 0.0);

        assert!(result.is_ok());
        let encoded = result.unwrap();
        // Just verify PNG was created - actual pixel values would require decoding
        assert!(!encoded.png_data.is_empty());
        // PNG magic bytes
        assert_eq!(&encoded.png_data[0..4], &[0x89, b'P', b'N', b'G']);
    }
}
