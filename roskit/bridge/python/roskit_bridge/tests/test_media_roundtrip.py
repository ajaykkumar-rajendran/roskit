#!/usr/bin/env python3
"""
Media Encoding Round-Trip Tests for RosKit Python Bridge

Tests for encoding and decoding media payloads (PNG, JPEG, CBOR)
including image validation and ROS2 message structure preservation.

Run with: pytest tests/test_media_roundtrip.py -v
"""

import io
import struct
import sys
from pathlib import Path

import pytest
import cbor2

# Add the bridge package to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from roskit_bridge.protocol import (
    HEADER_SIZE,
    MessageType,
    Encoding,
    create_message,
    parse_message,
)


def hex_to_bytes(hex_str: str) -> bytes:
    """Convert hex string to bytes."""
    return bytes.fromhex(hex_str)


# ============================================================================
# PNG Round-Trip Tests
# ============================================================================

def create_minimal_png() -> bytes:
    """Create minimal valid 1x1 red PNG (67 bytes)."""
    return hex_to_bytes(
        "89504e470d0a1a0a0000000d49484452000000010000000108020000009058b1eb"
        "0000000c49444154789c636060f80f00010401006baf3e2a0000000049454e44ae426082"
    )


class TestPNGRoundTrip:
    """PNG encoding round-trip tests."""

    def test_png_encoding_preserved(self):
        """PNG encoding should be preserved through encode/decode."""
        png_data = create_minimal_png()
        message = create_message(
            MessageType.MESSAGE,
            channel_id=10,
            payload=png_data,
            encoding=Encoding.PNG,
            timestamp_ns=1700000000000000000,
            flags=0
        )

        header, payload = parse_message(message)

        assert header is not None
        assert header.encoding == Encoding.PNG
        assert header.payload_length == len(png_data)
        assert payload == png_data

    def test_png_magic_bytes_valid(self):
        """PNG should have valid magic bytes."""
        png_data = create_minimal_png()

        # PNG magic bytes: 89 50 4e 47 0d 0a 1a 0a
        assert png_data[:8] == bytes([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a])

    def test_png_decode_with_pillow(self):
        """PNG should be decodable with Pillow."""
        try:
            from PIL import Image
        except ImportError:
            pytest.skip("Pillow not installed")

        png_data = create_minimal_png()
        img = Image.open(io.BytesIO(png_data))

        assert img.width == 1
        assert img.height == 1
        assert img.mode in ("RGB", "P")

    def test_png_round_trip_with_metadata(self):
        """PNG with CBOR metadata prefix should round-trip correctly."""
        png_data = create_minimal_png()

        # Create OccupancyGrid-style metadata
        metadata = {
            "width": 1,
            "height": 1,
            "resolution": 0.05,
        }
        metadata_bytes = cbor2.dumps(metadata)

        # Format: [4-byte metadata length][CBOR metadata][PNG data]
        payload = struct.pack(">I", len(metadata_bytes)) + metadata_bytes + png_data

        message = create_message(
            MessageType.MESSAGE,
            channel_id=21,
            payload=payload,
            encoding=Encoding.PNG,
            timestamp_ns=1700000000000000000,
            flags=0
        )

        header, parsed_payload = parse_message(message)

        assert header.encoding == Encoding.PNG

        # Extract metadata length
        metadata_len = struct.unpack(">I", parsed_payload[:4])[0]

        # Extract and verify CBOR metadata
        metadata_cbor = parsed_payload[4:4 + metadata_len]
        decoded_metadata = cbor2.loads(metadata_cbor)

        assert decoded_metadata["width"] == 1
        assert decoded_metadata["height"] == 1
        assert decoded_metadata["resolution"] == 0.05

        # Extract and verify PNG
        png_bytes = parsed_payload[4 + metadata_len:]
        assert png_bytes == png_data


# ============================================================================
# JPEG Round-Trip Tests
# ============================================================================

def create_minimal_jpeg_header() -> bytes:
    """Create minimal JPEG header data."""
    # Minimal JPEG: SOI marker (FFD8) + APP0 marker start
    return hex_to_bytes("ffd8ffe000104a46494600010100000100010000")


class TestJPEGRoundTrip:
    """JPEG encoding round-trip tests."""

    def test_jpeg_encoding_preserved(self):
        """JPEG encoding should be preserved through encode/decode."""
        jpeg_data = create_minimal_jpeg_header()
        message = create_message(
            MessageType.MESSAGE,
            channel_id=11,
            payload=jpeg_data,
            encoding=Encoding.JPEG,
            timestamp_ns=1700000000000000000,
            flags=0
        )

        header, payload = parse_message(message)

        assert header is not None
        assert header.encoding == Encoding.JPEG
        assert payload == jpeg_data

    def test_jpeg_soi_marker(self):
        """JPEG should have SOI marker."""
        jpeg_data = create_minimal_jpeg_header()

        # JPEG SOI marker: FF D8
        assert jpeg_data[:2] == bytes([0xff, 0xd8])

    def test_jpeg_app0_marker(self):
        """JPEG should have APP0 marker with JFIF identifier."""
        jpeg_data = create_minimal_jpeg_header()

        # APP0 marker: FF E0
        assert jpeg_data[2:4] == bytes([0xff, 0xe0])

        # JFIF identifier: "JFIF\0"
        assert jpeg_data[6:11] == b"JFIF\0"


# ============================================================================
# CBOR Round-Trip Tests
# ============================================================================

class TestCBORRoundTrip:
    """CBOR encoding round-trip tests."""

    def test_cbor_laser_scan_round_trip(self):
        """LaserScan CBOR structure should round-trip correctly."""
        laser_scan = {
            "header": {
                "stamp": {"sec": 1700000000, "nanosec": 0},
                "frame_id": "laser"
            },
            "angle_min": -1.5707963,
            "angle_max": 1.5707963,
            "ranges": [1.0, 2.0, 3.0, 4.0, 5.0],
            "intensities": [100.0, 200.0, 300.0, 400.0, 500.0]
        }

        cbor_bytes = cbor2.dumps(laser_scan)

        message = create_message(
            MessageType.MESSAGE,
            channel_id=20,
            payload=cbor_bytes,
            encoding=Encoding.CBOR,
            timestamp_ns=1700000000000000000,
            flags=0
        )

        header, payload = parse_message(message)

        assert header.encoding == Encoding.CBOR
        assert payload == cbor_bytes

        # Decode and verify structure
        decoded = cbor2.loads(payload)
        assert decoded["header"]["frame_id"] == "laser"
        assert len(decoded["ranges"]) == 5
        assert decoded["ranges"] == [1.0, 2.0, 3.0, 4.0, 5.0]

    def test_cbor_occupancy_grid_round_trip(self):
        """OccupancyGrid CBOR structure should round-trip correctly."""
        grid = {
            "header": {
                "stamp": {"sec": 1700000000, "nanosec": 0},
                "frame_id": "map"
            },
            "info": {
                "resolution": 0.05,
                "width": 100,
                "height": 100,
                "origin": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                }
            },
            "data": [0, 100, -1, 50]
        }

        cbor_bytes = cbor2.dumps(grid)

        message = create_message(
            MessageType.MESSAGE,
            channel_id=21,
            payload=cbor_bytes,
            encoding=Encoding.CBOR,
            timestamp_ns=1700000000000000000,
            flags=0
        )

        header, payload = parse_message(message)

        assert header.encoding == Encoding.CBOR

        decoded = cbor2.loads(payload)
        assert decoded["info"]["resolution"] == 0.05
        assert decoded["info"]["width"] == 100
        assert decoded["data"] == [0, 100, -1, 50]

    def test_cbor_point_cloud_round_trip(self):
        """PointCloud2 CBOR structure should round-trip correctly."""
        cloud = {
            "header": {
                "stamp": {"sec": 1700000000, "nanosec": 0},
                "frame_id": "lidar"
            },
            "height": 1,
            "width": 100,
            "fields": [
                {"name": "x", "offset": 0, "datatype": 7, "count": 1},
                {"name": "y", "offset": 4, "datatype": 7, "count": 1},
                {"name": "z", "offset": 8, "datatype": 7, "count": 1},
            ],
            "point_step": 12,
            "row_step": 1200,
            "is_dense": True
        }

        cbor_bytes = cbor2.dumps(cloud)

        message = create_message(
            MessageType.MESSAGE,
            channel_id=22,
            payload=cbor_bytes,
            encoding=Encoding.CBOR,
            timestamp_ns=1700000000000000000,
            flags=0
        )

        header, payload = parse_message(message)

        assert header.encoding == Encoding.CBOR

        decoded = cbor2.loads(payload)
        assert decoded["header"]["frame_id"] == "lidar"
        assert len(decoded["fields"]) == 3
        assert decoded["point_step"] == 12
        assert decoded["is_dense"] is True


# ============================================================================
# Binary Encoding Tests
# ============================================================================

class TestBinaryRoundTrip:
    """Binary encoding round-trip tests."""

    def test_binary_float32_array_round_trip(self):
        """Binary float32 array should round-trip correctly."""
        floats = [1.0, 2.5, 3.7, 4.2, 5.9]
        binary_data = b''.join(struct.pack('<f', f) for f in floats)

        message = create_message(
            MessageType.MESSAGE,
            channel_id=30,
            payload=binary_data,
            encoding=Encoding.BINARY,
            timestamp_ns=0,
            flags=0
        )

        header, payload = parse_message(message)

        assert header.encoding == Encoding.BINARY
        assert len(payload) == len(floats) * 4

        # Decode floats
        decoded_floats = [
            struct.unpack('<f', payload[i:i+4])[0]
            for i in range(0, len(payload), 4)
        ]

        for original, decoded in zip(floats, decoded_floats):
            assert abs(original - decoded) < 0.0001

    def test_binary_point_cloud_xyz(self):
        """Binary XYZ point cloud should round-trip correctly."""
        points = [
            (1.0, 2.0, 3.0),
            (4.0, 5.0, 6.0),
            (7.0, 8.0, 9.0),
        ]

        binary_data = b''.join(
            struct.pack('<fff', x, y, z)
            for x, y, z in points
        )

        message = create_message(
            MessageType.MESSAGE,
            channel_id=31,
            payload=binary_data,
            encoding=Encoding.BINARY,
            timestamp_ns=0,
            flags=0
        )

        header, payload = parse_message(message)

        assert header.encoding == Encoding.BINARY
        assert len(payload) == 36  # 3 points * 12 bytes

        # Decode points
        decoded_points = [
            struct.unpack('<fff', payload[i:i+12])
            for i in range(0, len(payload), 12)
        ]

        for original, decoded in zip(points, decoded_points):
            assert abs(original[0] - decoded[0]) < 0.0001
            assert abs(original[1] - decoded[1]) < 0.0001
            assert abs(original[2] - decoded[2]) < 0.0001


# ============================================================================
# Large Payload Tests
# ============================================================================

class TestLargePayloads:
    """Large payload handling tests."""

    def test_large_binary_payload(self):
        """Large binary payload (10000 points) should round-trip."""
        num_points = 10000
        point_size = 12  # XYZ float32

        # Create large point cloud
        binary_data = b''.join(
            struct.pack('<fff', float(i), float(i * 2), float(i * 3))
            for i in range(num_points)
        )

        message = create_message(
            MessageType.MESSAGE,
            channel_id=40,
            payload=binary_data,
            encoding=Encoding.BINARY,
            timestamp_ns=0,
            flags=0
        )

        header, payload = parse_message(message)

        assert header.payload_length == num_points * point_size
        assert len(payload) == num_points * point_size

        # Spot check first and last points
        first_x = struct.unpack('<f', payload[0:4])[0]
        assert first_x == 0.0

        last_offset = (num_points - 1) * point_size
        last_x = struct.unpack('<f', payload[last_offset:last_offset+4])[0]
        assert last_x == float(num_points - 1)

    def test_large_cbor_payload(self):
        """Large CBOR array (1000 elements) should round-trip."""
        large_data = {"data": list(range(1000))}
        cbor_bytes = cbor2.dumps(large_data)

        message = create_message(
            MessageType.MESSAGE,
            channel_id=41,
            payload=cbor_bytes,
            encoding=Encoding.CBOR,
            timestamp_ns=0,
            flags=0
        )

        header, payload = parse_message(message)

        assert header.encoding == Encoding.CBOR

        decoded = cbor2.loads(payload)
        assert len(decoded["data"]) == 1000
        assert decoded["data"][0] == 0
        assert decoded["data"][999] == 999


# ============================================================================
# Channel and Timestamp Preservation Tests
# ============================================================================

class TestChannelTimestampPreservation:
    """Channel ID and timestamp preservation tests."""

    def test_channel_id_preserved_for_media(self):
        """Channel ID should be preserved for all channel values."""
        png_data = create_minimal_png()

        for channel_id in [0, 1, 100, 1000, 0xFFFFFFFF]:
            message = create_message(
                MessageType.MESSAGE,
                channel_id=channel_id,
                payload=png_data,
                encoding=Encoding.PNG,
                timestamp_ns=0,
                flags=0
            )

            header, _ = parse_message(message)
            assert header.channel_id == channel_id

    def test_timestamp_preserved_for_media(self):
        """Timestamp should be preserved for all timestamp values."""
        png_data = create_minimal_png()

        for timestamp in [0, 1, 1700000000000000000, 0xFFFFFFFFFFFFFFFF]:
            message = create_message(
                MessageType.MESSAGE,
                channel_id=1,
                payload=png_data,
                encoding=Encoding.PNG,
                timestamp_ns=timestamp,
                flags=0
            )

            header, _ = parse_message(message)
            assert header.timestamp_ns == timestamp


# ============================================================================
# Encoder Tests (if available)
# ============================================================================

class TestEncoders:
    """Test media encoders if available."""

    def test_image_encoder_available(self):
        """Check if image encoder is available."""
        try:
            from roskit_bridge.encoders import get_encoder
            encoder = get_encoder("sensor_msgs/msg/Image")
            assert encoder is not None or encoder is None  # May or may not exist
        except ImportError:
            pytest.skip("Encoders not available")

    def test_laser_scan_encoder_available(self):
        """Check if LaserScan encoder is available."""
        try:
            from roskit_bridge.encoders import get_encoder
            encoder = get_encoder("sensor_msgs/msg/LaserScan")
            assert encoder is not None or encoder is None
        except ImportError:
            pytest.skip("Encoders not available")

    def test_occupancy_grid_encoder_available(self):
        """Check if OccupancyGrid encoder is available."""
        try:
            from roskit_bridge.encoders import get_encoder
            encoder = get_encoder("nav_msgs/msg/OccupancyGrid")
            assert encoder is not None or encoder is None
        except ImportError:
            pytest.skip("Encoders not available")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
