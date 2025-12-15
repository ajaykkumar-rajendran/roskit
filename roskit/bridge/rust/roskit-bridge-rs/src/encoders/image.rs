//! Image encoder - handles sensor_msgs/Image and CompressedImage
//!
//! Encodes raw Image messages to JPEG for efficient transmission.
//! CompressedImage messages are passed through if already JPEG/PNG,
//! or re-encoded if in an unsupported format.

use bytes::Bytes;
use image::{ImageBuffer, Rgb, Rgba, DynamicImage};
use serde::Serialize;
use std::io::Cursor;

/// Metadata for the image (sent alongside image data)
#[derive(Debug, Serialize)]
pub struct ImageMetadata {
    pub width: u32,
    pub height: u32,
    pub encoding: String,
    pub frame_id: String,
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
}

/// Encoded image with JPEG/PNG data and metadata
#[derive(Debug)]
pub struct EncodedImage {
    pub image_data: Bytes,
    pub metadata: ImageMetadata,
    pub format: ImageFormat,
}

/// Output image format
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ImageFormat {
    Jpeg,
    Png,
}

/// JPEG quality for encoding (0-100)
const JPEG_QUALITY: u8 = 85;

/// Encode a raw ROS Image message to JPEG
///
/// Supported ROS encodings:
/// - rgb8: 3-channel RGB
/// - rgba8: 4-channel RGBA (alpha is discarded)
/// - bgr8: 3-channel BGR (converted to RGB)
/// - bgra8: 4-channel BGRA (converted to RGB)
/// - mono8: 8-bit grayscale
/// - mono16: 16-bit grayscale (converted to 8-bit)
pub fn encode_image(
    data: &[u8],
    width: u32,
    height: u32,
    encoding: &str,
    step: u32,
    frame_id: &str,
    stamp_sec: i32,
    stamp_nanosec: u32,
) -> Result<EncodedImage, Box<dyn std::error::Error + Send + Sync>> {
    let dynamic_image = match encoding {
        "rgb8" => {
            let img: ImageBuffer<Rgb<u8>, Vec<u8>> =
                ImageBuffer::from_raw(width, height, data.to_vec())
                    .ok_or("Failed to create RGB image buffer")?;
            DynamicImage::ImageRgb8(img)
        }
        "rgba8" => {
            let img: ImageBuffer<Rgba<u8>, Vec<u8>> =
                ImageBuffer::from_raw(width, height, data.to_vec())
                    .ok_or("Failed to create RGBA image buffer")?;
            DynamicImage::ImageRgba8(img)
        }
        "bgr8" => {
            // Convert BGR to RGB
            let mut rgb_data = Vec::with_capacity((width * height * 3) as usize);
            for y in 0..height {
                for x in 0..width {
                    let idx = (y * step + x * 3) as usize;
                    if idx + 2 < data.len() {
                        rgb_data.push(data[idx + 2]); // R
                        rgb_data.push(data[idx + 1]); // G
                        rgb_data.push(data[idx]);     // B
                    }
                }
            }
            let img: ImageBuffer<Rgb<u8>, Vec<u8>> =
                ImageBuffer::from_raw(width, height, rgb_data)
                    .ok_or("Failed to create RGB image from BGR")?;
            DynamicImage::ImageRgb8(img)
        }
        "bgra8" => {
            // Convert BGRA to RGB (discard alpha)
            let mut rgb_data = Vec::with_capacity((width * height * 3) as usize);
            for y in 0..height {
                for x in 0..width {
                    let idx = (y * step + x * 4) as usize;
                    if idx + 2 < data.len() {
                        rgb_data.push(data[idx + 2]); // R
                        rgb_data.push(data[idx + 1]); // G
                        rgb_data.push(data[idx]);     // B
                    }
                }
            }
            let img: ImageBuffer<Rgb<u8>, Vec<u8>> =
                ImageBuffer::from_raw(width, height, rgb_data)
                    .ok_or("Failed to create RGB image from BGRA")?;
            DynamicImage::ImageRgb8(img)
        }
        "mono8" => {
            let img: ImageBuffer<image::Luma<u8>, Vec<u8>> =
                ImageBuffer::from_raw(width, height, data.to_vec())
                    .ok_or("Failed to create grayscale image buffer")?;
            DynamicImage::ImageLuma8(img)
        }
        "mono16" | "16UC1" => {
            // Convert 16-bit to 8-bit by scaling
            let mut mono8_data = Vec::with_capacity((width * height) as usize);
            for y in 0..height {
                for x in 0..width {
                    let idx = (y * step + x * 2) as usize;
                    if idx + 1 < data.len() {
                        // Read as little-endian u16 and scale to u8
                        let value = u16::from_le_bytes([data[idx], data[idx + 1]]);
                        mono8_data.push((value >> 8) as u8);
                    }
                }
            }
            let img: ImageBuffer<image::Luma<u8>, Vec<u8>> =
                ImageBuffer::from_raw(width, height, mono8_data)
                    .ok_or("Failed to create grayscale image from mono16")?;
            DynamicImage::ImageLuma8(img)
        }
        _ => {
            return Err(format!("Unsupported image encoding: {}", encoding).into());
        }
    };

    // Encode to JPEG
    let mut jpeg_buffer = Cursor::new(Vec::new());
    dynamic_image.write_to(&mut jpeg_buffer, image::ImageFormat::Jpeg)?;

    let metadata = ImageMetadata {
        width,
        height,
        encoding: encoding.to_string(),
        frame_id: frame_id.to_string(),
        stamp_sec,
        stamp_nanosec,
    };

    Ok(EncodedImage {
        image_data: Bytes::from(jpeg_buffer.into_inner()),
        metadata,
        format: ImageFormat::Jpeg,
    })
}

/// Handle a CompressedImage message
///
/// If the format is already JPEG or PNG, pass through directly.
/// Otherwise, attempt to decode and re-encode as JPEG.
pub fn encode_compressed_image(
    data: &[u8],
    format: &str,
    frame_id: &str,
    stamp_sec: i32,
    stamp_nanosec: u32,
) -> Result<EncodedImage, Box<dyn std::error::Error + Send + Sync>> {
    // Detect format from the format string or magic bytes
    let (output_data, output_format) = if format.contains("jpeg") || format.contains("jpg") {
        // Already JPEG, pass through
        (Bytes::from(data.to_vec()), ImageFormat::Jpeg)
    } else if format.contains("png") {
        // Already PNG, pass through
        (Bytes::from(data.to_vec()), ImageFormat::Png)
    } else if is_jpeg(data) {
        // Magic bytes indicate JPEG
        (Bytes::from(data.to_vec()), ImageFormat::Jpeg)
    } else if is_png(data) {
        // Magic bytes indicate PNG
        (Bytes::from(data.to_vec()), ImageFormat::Png)
    } else {
        // Try to decode and re-encode as JPEG
        let img = image::load_from_memory(data)?;
        let mut jpeg_buffer = Cursor::new(Vec::new());
        img.write_to(&mut jpeg_buffer, image::ImageFormat::Jpeg)?;
        (Bytes::from(jpeg_buffer.into_inner()), ImageFormat::Jpeg)
    };

    // Get dimensions by decoding (only if we need them)
    let (width, height) = match image::load_from_memory(&output_data) {
        Ok(img) => (img.width(), img.height()),
        Err(_) => (0, 0), // Unknown dimensions
    };

    let metadata = ImageMetadata {
        width,
        height,
        encoding: format.to_string(),
        frame_id: frame_id.to_string(),
        stamp_sec,
        stamp_nanosec,
    };

    Ok(EncodedImage {
        image_data: output_data,
        metadata,
        format: output_format,
    })
}

/// Check if data starts with JPEG magic bytes
fn is_jpeg(data: &[u8]) -> bool {
    data.len() >= 2 && data[0] == 0xFF && data[1] == 0xD8
}

/// Check if data starts with PNG magic bytes
fn is_png(data: &[u8]) -> bool {
    data.len() >= 4 && data[0..4] == [0x89, b'P', b'N', b'G']
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_is_jpeg() {
        assert!(is_jpeg(&[0xFF, 0xD8, 0xFF, 0xE0]));
        assert!(!is_jpeg(&[0x89, b'P', b'N', b'G']));
        assert!(!is_jpeg(&[]));
    }

    #[test]
    fn test_is_png() {
        assert!(is_png(&[0x89, b'P', b'N', b'G', 0x0D, 0x0A]));
        assert!(!is_png(&[0xFF, 0xD8]));
        assert!(!is_png(&[]));
    }

    #[test]
    fn test_encode_rgb8_image() {
        // Create a simple 2x2 RGB image
        let data: Vec<u8> = vec![
            255, 0, 0,    // Red
            0, 255, 0,    // Green
            0, 0, 255,    // Blue
            255, 255, 0,  // Yellow
        ];

        let result = encode_image(
            &data, 2, 2, "rgb8", 6, "camera_frame", 1234, 567890
        );

        assert!(result.is_ok());
        let encoded = result.unwrap();
        assert_eq!(encoded.metadata.width, 2);
        assert_eq!(encoded.metadata.height, 2);
        assert_eq!(encoded.metadata.encoding, "rgb8");
        assert_eq!(encoded.format, ImageFormat::Jpeg);
        assert!(is_jpeg(&encoded.image_data));
    }

    #[test]
    fn test_encode_mono8_image() {
        // Create a simple 2x2 grayscale image
        let data: Vec<u8> = vec![0, 64, 128, 255];

        let result = encode_image(
            &data, 2, 2, "mono8", 2, "depth_frame", 1234, 567890
        );

        assert!(result.is_ok());
        let encoded = result.unwrap();
        assert_eq!(encoded.metadata.width, 2);
        assert_eq!(encoded.metadata.height, 2);
        assert_eq!(encoded.format, ImageFormat::Jpeg);
    }

    #[test]
    fn test_unsupported_encoding() {
        let data: Vec<u8> = vec![0, 0, 0, 0];
        let result = encode_image(&data, 2, 2, "bayer_rggb8", 2, "frame", 0, 0);
        assert!(result.is_err());
    }
}
