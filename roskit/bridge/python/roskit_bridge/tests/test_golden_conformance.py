#!/usr/bin/env python3
"""
RosKit Protocol Golden Conformance Tests (Python Bridge)

These tests validate that the Python bridge correctly implements the RosKit
binary protocol by testing against golden vectors shared across all
implementations (TypeScript, Python, Rust).

Run with: pytest tests/test_golden_conformance.py -v
Or: python -m pytest tests/test_golden_conformance.py -v
"""

import json
import sys
from pathlib import Path

import pytest

# Add the bridge package to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from roskit_bridge.protocol import (
    MAGIC,
    PROTOCOL_VERSION,
    HEADER_SIZE,
    MessageType,
    Encoding,
    Flags,
    create_message,
    parse_message,
)


def load_golden_vectors():
    """Load golden test vectors from shared JSON file."""
    possible_paths = [
        Path(__file__).parent.parent.parent.parent.parent / "protocol-tests" / "golden-vectors.json",
        Path(__file__).parent.parent.parent.parent.parent.parent / "protocol-tests" / "golden-vectors.json",
    ]

    for path in possible_paths:
        if path.exists():
            with open(path) as f:
                return json.load(f)

    pytest.skip(f"Could not find golden-vectors.json. Tried: {possible_paths}")


def hex_to_bytes(hex_str: str) -> bytes:
    """Convert hex string to bytes."""
    return bytes.fromhex(hex_str)


def bytes_to_hex(data: bytes) -> str:
    """Convert bytes to hex string."""
    return data.hex()


# ============================================================================
# Protocol Constants Tests
# ============================================================================

class TestProtocolConstants:
    """Test protocol constants match golden vectors."""

    @pytest.fixture
    def vectors(self):
        return load_golden_vectors()

    def test_magic_bytes(self, vectors):
        """MAGIC bytes should match golden vectors."""
        expected = bytes(vectors["protocol"]["magic"])
        assert MAGIC == expected, f"Expected {expected!r}, got {MAGIC!r}"

    def test_protocol_version(self, vectors):
        """PROTOCOL_VERSION should match golden vectors."""
        expected = vectors["protocol"]["version"]
        assert PROTOCOL_VERSION == expected

    def test_header_size(self, vectors):
        """HEADER_SIZE should match golden vectors."""
        expected = vectors["protocol"]["header_size"]
        assert HEADER_SIZE == expected


# ============================================================================
# Message Type Tests
# ============================================================================

class TestMessageTypes:
    """Test message type values match golden vectors."""

    @pytest.fixture
    def vectors(self):
        return load_golden_vectors()

    def test_subscribe(self, vectors):
        expected = int(vectors["message_types"]["SUBSCRIBE"], 16)
        assert MessageType.SUBSCRIBE.value == expected

    def test_unsubscribe(self, vectors):
        expected = int(vectors["message_types"]["UNSUBSCRIBE"], 16)
        assert MessageType.UNSUBSCRIBE.value == expected

    def test_publish(self, vectors):
        expected = int(vectors["message_types"]["PUBLISH"], 16)
        assert MessageType.PUBLISH.value == expected

    def test_service_call(self, vectors):
        expected = int(vectors["message_types"]["SERVICE_CALL"], 16)
        assert MessageType.SERVICE_CALL.value == expected

    def test_topic_list(self, vectors):
        expected = int(vectors["message_types"]["TOPIC_LIST"], 16)
        assert MessageType.TOPIC_LIST.value == expected

    def test_ping(self, vectors):
        expected = int(vectors["message_types"]["PING"], 16)
        assert MessageType.PING.value == expected

    def test_service_list(self, vectors):
        expected = int(vectors["message_types"]["SERVICE_LIST"], 16)
        assert MessageType.SERVICE_LIST.value == expected

    def test_node_list(self, vectors):
        expected = int(vectors["message_types"]["NODE_LIST"], 16)
        assert MessageType.NODE_LIST.value == expected

    def test_message(self, vectors):
        expected = int(vectors["message_types"]["MESSAGE"], 16)
        assert MessageType.MESSAGE.value == expected

    def test_channel_info(self, vectors):
        expected = int(vectors["message_types"]["CHANNEL_INFO"], 16)
        assert MessageType.CHANNEL_INFO.value == expected

    def test_service_response(self, vectors):
        expected = int(vectors["message_types"]["SERVICE_RESPONSE"], 16)
        assert MessageType.SERVICE_RESPONSE.value == expected

    def test_topic_list_response(self, vectors):
        expected = int(vectors["message_types"]["TOPIC_LIST_RESPONSE"], 16)
        assert MessageType.TOPIC_LIST_RESPONSE.value == expected

    def test_server_info(self, vectors):
        expected = int(vectors["message_types"]["SERVER_INFO"], 16)
        assert MessageType.SERVER_INFO.value == expected

    def test_pong(self, vectors):
        expected = int(vectors["message_types"]["PONG"], 16)
        assert MessageType.PONG.value == expected

    def test_service_list_response(self, vectors):
        expected = int(vectors["message_types"]["SERVICE_LIST_RESPONSE"], 16)
        assert MessageType.SERVICE_LIST_RESPONSE.value == expected

    def test_node_list_response(self, vectors):
        expected = int(vectors["message_types"]["NODE_LIST_RESPONSE"], 16)
        assert MessageType.NODE_LIST_RESPONSE.value == expected

    def test_error(self, vectors):
        expected = int(vectors["message_types"]["ERROR"], 16)
        assert MessageType.ERROR.value == expected


# ============================================================================
# Encoding Type Tests
# ============================================================================

class TestEncodings:
    """Test encoding values match golden vectors."""

    @pytest.fixture
    def vectors(self):
        return load_golden_vectors()

    def test_raw(self, vectors):
        expected = int(vectors["encodings"]["RAW"], 16)
        assert Encoding.RAW.value == expected

    def test_cbor(self, vectors):
        expected = int(vectors["encodings"]["CBOR"], 16)
        assert Encoding.CBOR.value == expected

    def test_json(self, vectors):
        expected = int(vectors["encodings"]["JSON"], 16)
        assert Encoding.JSON.value == expected

    def test_png(self, vectors):
        expected = int(vectors["encodings"]["PNG"], 16)
        assert Encoding.PNG.value == expected

    def test_jpeg(self, vectors):
        expected = int(vectors["encodings"]["JPEG"], 16)
        assert Encoding.JPEG.value == expected

    def test_binary(self, vectors):
        expected = int(vectors["encodings"]["BINARY"], 16)
        assert Encoding.BINARY.value == expected


# ============================================================================
# Flag Tests
# ============================================================================

class TestFlags:
    """Test flag values match golden vectors."""

    @pytest.fixture
    def vectors(self):
        return load_golden_vectors()

    def test_none(self, vectors):
        expected = int(vectors["flags"]["NONE"], 16)
        assert Flags.NONE.value == expected

    def test_compressed(self, vectors):
        expected = int(vectors["flags"]["COMPRESSED"], 16)
        assert Flags.COMPRESSED.value == expected

    def test_fragmented(self, vectors):
        expected = int(vectors["flags"]["FRAGMENTED"], 16)
        assert Flags.FRAGMENTED.value == expected

    def test_last_fragment(self, vectors):
        expected = int(vectors["flags"]["LAST_FRAGMENT"], 16)
        assert Flags.LAST_FRAGMENT.value == expected


# ============================================================================
# Golden Vector Message Tests
# ============================================================================

class TestGoldenVectors:
    """Test message encoding/decoding against golden vectors."""

    @pytest.fixture
    def vectors(self):
        return load_golden_vectors()

    def get_vector(self, vectors, name: str):
        """Get a test vector by name."""
        for v in vectors["test_vectors"]:
            if v["name"] == name:
                return v
        pytest.fail(f"Vector '{name}' not found")

    def test_empty_ping(self, vectors):
        """PING message with no payload."""
        vector = self.get_vector(vectors, "empty_ping")
        header = vector["header"]

        msg_type = MessageType(int(header["message_type"], 16))
        channel_id = header["channel_id"]
        timestamp_ns = header["timestamp_ns"]
        encoding = Encoding(int(header["encoding"], 16))
        flags = int(header["flags"], 16)
        payload = hex_to_bytes(vector.get("payload_hex", ""))

        created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)
        expected = hex_to_bytes(vector["expected_hex"])

        assert bytes_to_hex(created) == bytes_to_hex(expected), \
            f"empty_ping mismatch:\nExpected: {bytes_to_hex(expected)}\nGot: {bytes_to_hex(created)}"

        # Verify round-trip parsing
        parsed_header, parsed_payload = parse_message(created)
        assert parsed_header is not None
        assert parsed_header.message_type == msg_type
        assert parsed_header.channel_id == channel_id
        assert parsed_header.encoding == encoding
        assert parsed_payload == payload

    def test_empty_pong(self, vectors):
        """PONG message with no payload."""
        vector = self.get_vector(vectors, "empty_pong")
        header = vector["header"]

        msg_type = MessageType(int(header["message_type"], 16))
        channel_id = header["channel_id"]
        timestamp_ns = header["timestamp_ns"]
        encoding = Encoding(int(header["encoding"], 16))
        flags = int(header["flags"], 16)
        payload = hex_to_bytes(vector.get("payload_hex", ""))

        created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)
        expected = hex_to_bytes(vector["expected_hex"])

        assert created == expected

    def test_topic_list_request(self, vectors):
        """TOPIC_LIST request with no payload."""
        vector = self.get_vector(vectors, "topic_list_request")
        header = vector["header"]

        msg_type = MessageType(int(header["message_type"], 16))
        channel_id = header["channel_id"]
        timestamp_ns = header["timestamp_ns"]
        encoding = Encoding(int(header["encoding"], 16))
        flags = int(header["flags"], 16)
        payload = hex_to_bytes(vector.get("payload_hex", ""))

        created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)
        expected = hex_to_bytes(vector["expected_hex"])

        assert created == expected

    def test_service_list_request(self, vectors):
        """SERVICE_LIST request with no payload."""
        vector = self.get_vector(vectors, "service_list_request")
        header = vector["header"]

        msg_type = MessageType(int(header["message_type"], 16))
        channel_id = header["channel_id"]
        timestamp_ns = header["timestamp_ns"]
        encoding = Encoding(int(header["encoding"], 16))
        flags = int(header["flags"], 16)
        payload = hex_to_bytes(vector.get("payload_hex", ""))

        created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)
        expected = hex_to_bytes(vector["expected_hex"])

        assert created == expected

    def test_node_list_request(self, vectors):
        """NODE_LIST request with no payload."""
        vector = self.get_vector(vectors, "node_list_request")
        header = vector["header"]

        msg_type = MessageType(int(header["message_type"], 16))
        channel_id = header["channel_id"]
        timestamp_ns = header["timestamp_ns"]
        encoding = Encoding(int(header["encoding"], 16))
        flags = int(header["flags"], 16)
        payload = hex_to_bytes(vector.get("payload_hex", ""))

        created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)
        expected = hex_to_bytes(vector["expected_hex"])

        assert created == expected

    def test_message_with_channel(self, vectors):
        """MESSAGE on channel 42 with CBOR payload."""
        vector = self.get_vector(vectors, "message_with_channel")
        header = vector["header"]

        msg_type = MessageType(int(header["message_type"], 16))
        channel_id = header["channel_id"]
        timestamp_ns = header["timestamp_ns"]
        encoding = Encoding(int(header["encoding"], 16))
        flags = int(header["flags"], 16)
        payload = hex_to_bytes(vector.get("payload_hex", ""))

        created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)
        expected = hex_to_bytes(vector["expected_hex"])

        assert created == expected

        # Verify parsed values
        parsed_header, _ = parse_message(created)
        assert parsed_header.channel_id == 42
        assert parsed_header.timestamp_ns == 1000000000
        assert parsed_header.encoding == Encoding.CBOR

    def test_subscribe_request(self, vectors):
        """SUBSCRIBE request for /scan topic."""
        vector = self.get_vector(vectors, "subscribe_request")
        header = vector["header"]

        msg_type = MessageType(int(header["message_type"], 16))
        channel_id = header["channel_id"]
        timestamp_ns = header["timestamp_ns"]
        encoding = Encoding(int(header["encoding"], 16))
        flags = int(header["flags"], 16)
        payload = hex_to_bytes(vector.get("payload_hex", ""))

        created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)
        expected = hex_to_bytes(vector["expected_hex"])

        assert created == expected

    def test_large_channel_id(self, vectors):
        """MESSAGE with max channel ID (2^32-1)."""
        vector = self.get_vector(vectors, "large_channel_id")
        header = vector["header"]

        msg_type = MessageType(int(header["message_type"], 16))
        channel_id = header["channel_id"]
        timestamp_ns = header["timestamp_ns"]
        encoding = Encoding(int(header["encoding"], 16))
        flags = int(header["flags"], 16)
        payload = hex_to_bytes(vector.get("payload_hex", ""))

        created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)
        expected = hex_to_bytes(vector["expected_hex"])

        assert created == expected

        # Verify max u32 value
        parsed_header, _ = parse_message(created)
        assert parsed_header.channel_id == 0xFFFFFFFF


# ============================================================================
# Invalid Vector Tests
# ============================================================================

class TestInvalidVectors:
    """Test invalid message handling."""

    @pytest.fixture
    def vectors(self):
        return load_golden_vectors()

    def get_invalid_vector(self, vectors, name: str):
        """Get an invalid test vector by name."""
        for v in vectors["invalid_vectors"]:
            if v["name"] == name:
                return v
        pytest.fail(f"Invalid vector '{name}' not found")

    def test_wrong_magic(self, vectors):
        """Message with incorrect magic bytes should be rejected."""
        vector = self.get_invalid_vector(vectors, "wrong_magic")
        data = hex_to_bytes(vector["hex"])

        header, _ = parse_message(data)
        assert header is None, "Message with wrong magic should be rejected"

    def test_wrong_version(self, vectors):
        """Message with unsupported protocol version should be rejected."""
        vector = self.get_invalid_vector(vectors, "wrong_version")
        data = hex_to_bytes(vector["hex"])

        header, _ = parse_message(data)
        assert header is None, "Message with wrong version should be rejected"

    def test_truncated_header(self, vectors):
        """Message with incomplete header should be rejected."""
        vector = self.get_invalid_vector(vectors, "truncated_header")
        data = hex_to_bytes(vector["hex"])

        header, _ = parse_message(data)
        assert header is None, "Message with truncated header should be rejected"


# ============================================================================
# Media Encoding Tests
# ============================================================================

class TestMediaEncodings:
    """Test media encoding round-trips."""

    @pytest.fixture
    def vectors(self):
        return load_golden_vectors()

    def get_vector(self, vectors, name: str):
        """Get a test vector by name."""
        for v in vectors["test_vectors"]:
            if v["name"] == name:
                return v
        pytest.fail(f"Vector '{name}' not found")

    def test_png_image_message(self, vectors):
        """PNG image encoding should be preserved."""
        vector = self.get_vector(vectors, "png_image_message")
        header = vector["header"]

        msg_type = MessageType(int(header["message_type"], 16))
        channel_id = header["channel_id"]
        timestamp_ns = header["timestamp_ns"]
        encoding = Encoding(int(header["encoding"], 16))
        flags = int(header["flags"], 16)
        payload = hex_to_bytes(vector["payload_hex"])

        created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)

        parsed_header, parsed_payload = parse_message(created)
        assert parsed_header.encoding == Encoding.PNG
        assert parsed_header.payload_length == 67

        # Verify PNG magic bytes
        assert parsed_payload[:8] == bytes([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a])

    def test_jpeg_image_message(self, vectors):
        """JPEG image encoding should be preserved."""
        vector = self.get_vector(vectors, "jpeg_image_message")
        header = vector["header"]

        msg_type = MessageType(int(header["message_type"], 16))
        channel_id = header["channel_id"]
        timestamp_ns = header["timestamp_ns"]
        encoding = Encoding(int(header["encoding"], 16))
        flags = int(header["flags"], 16)
        payload = hex_to_bytes(vector["payload_hex"])

        created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)

        parsed_header, parsed_payload = parse_message(created)
        assert parsed_header.encoding == Encoding.JPEG

        # Verify JPEG SOI marker
        assert parsed_payload[:2] == bytes([0xff, 0xd8])

    def test_laser_scan_cbor(self, vectors):
        """LaserScan CBOR encoding should be defined."""
        vector = self.get_vector(vectors, "laser_scan_cbor")
        header = vector["header"]

        encoding = int(header["encoding"], 16)
        assert encoding == Encoding.CBOR.value

        # Verify CBOR structure is defined
        assert "payload_cbor" in vector
        assert "ranges" in vector["payload_cbor"]
        assert isinstance(vector["payload_cbor"]["ranges"], list)

    def test_occupancy_grid_cbor(self, vectors):
        """OccupancyGrid CBOR encoding should be defined."""
        vector = self.get_vector(vectors, "occupancy_grid_cbor")
        header = vector["header"]

        encoding = int(header["encoding"], 16)
        assert encoding == Encoding.CBOR.value

        # Verify CBOR structure
        assert "payload_cbor" in vector
        assert "info" in vector["payload_cbor"]
        assert "resolution" in vector["payload_cbor"]["info"]

    def test_point_cloud_cbor(self, vectors):
        """PointCloud2 CBOR encoding should be defined."""
        vector = self.get_vector(vectors, "point_cloud_cbor")
        header = vector["header"]

        encoding = int(header["encoding"], 16)
        assert encoding == Encoding.CBOR.value

        # Verify CBOR structure
        assert "payload_cbor" in vector
        assert "fields" in vector["payload_cbor"]
        assert isinstance(vector["payload_cbor"]["fields"], list)


# ============================================================================
# All Vectors Dynamic Test
# ============================================================================

class TestAllVectors:
    """Run all golden vectors dynamically."""

    @pytest.fixture
    def vectors(self):
        return load_golden_vectors()

    def test_all_vectors_with_expected_hex(self, vectors):
        """All vectors with expected_hex should encode correctly."""
        tested = 0

        for vector in vectors["test_vectors"]:
            if "expected_hex" not in vector:
                continue

            tested += 1
            name = vector["name"]
            header = vector["header"]

            msg_type = MessageType(int(header["message_type"], 16))
            channel_id = header["channel_id"]
            timestamp_ns = header["timestamp_ns"]
            encoding = Encoding(int(header["encoding"], 16))
            flags = int(header["flags"], 16)
            payload = hex_to_bytes(vector.get("payload_hex", ""))

            created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)
            expected = hex_to_bytes(vector["expected_hex"])

            assert created == expected, \
                f"Vector '{name}' failed:\nExpected: {bytes_to_hex(expected)}\nGot: {bytes_to_hex(created)}"

        assert tested > 0, "Should have tested at least one vector"
        print(f"Tested {tested} golden vectors, all passed")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
