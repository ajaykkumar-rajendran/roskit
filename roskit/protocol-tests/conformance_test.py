#!/usr/bin/env python3
"""
RosKit Protocol Conformance Tests (Python)

Run with: python3 protocol-tests/conformance_test.py
"""

import json
import sys
from pathlib import Path

# Add the bridge package to path
sys.path.insert(0, str(Path(__file__).parent.parent / "bridge" / "python" / "roskit_bridge"))

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


def hex_to_bytes(hex_str: str) -> bytes:
    """Convert hex string to bytes."""
    return bytes.fromhex(hex_str)


def bytes_to_hex(data: bytes) -> str:
    """Convert bytes to hex string."""
    return data.hex()


def load_vectors() -> dict:
    """Load golden test vectors."""
    vectors_path = Path(__file__).parent / "golden-vectors.json"
    with open(vectors_path) as f:
        return json.load(f)


def run_tests() -> tuple[int, int]:
    """Run all conformance tests. Returns (passed, failed) counts."""
    vectors = load_vectors()
    passed = 0
    failed = 0

    print("RosKit Protocol Conformance Tests (Python)")
    print("=" * 50)

    # Test 1: Protocol constants
    print("\n1. Protocol Constants")
    try:
        assert MAGIC == b'RK', f"MAGIC mismatch: expected b'RK', got {MAGIC}"
        assert PROTOCOL_VERSION == vectors["protocol"]["version"], \
            f"PROTOCOL_VERSION mismatch: expected {vectors['protocol']['version']}, got {PROTOCOL_VERSION}"
        assert HEADER_SIZE == vectors["protocol"]["header_size"], \
            f"HEADER_SIZE mismatch: expected {vectors['protocol']['header_size']}, got {HEADER_SIZE}"
        print("   ✓ Constants match golden vectors")
        passed += 1
    except AssertionError as e:
        print(f"   ✗ {e}")
        failed += 1

    # Test 2: Message type values
    print("\n2. Message Type Values")
    for name, hex_value in vectors["message_types"].items():
        expected = int(hex_value, 16)
        try:
            actual = MessageType[name].value
            assert actual == expected, f"MessageType.{name}: expected 0x{expected:02x}, got 0x{actual:02x}"
            print(f"   ✓ MessageType.{name} = 0x{expected:02x}")
            passed += 1
        except (KeyError, AssertionError) as e:
            print(f"   ✗ MessageType.{name}: {e}")
            failed += 1

    # Test 3: Encoding values
    print("\n3. Encoding Values")
    for name, hex_value in vectors["encodings"].items():
        expected = int(hex_value, 16)
        try:
            actual = Encoding[name].value
            assert actual == expected, f"Encoding.{name}: expected 0x{expected:02x}, got 0x{actual:02x}"
            print(f"   ✓ Encoding.{name} = 0x{expected:02x}")
            passed += 1
        except (KeyError, AssertionError) as e:
            print(f"   ✗ Encoding.{name}: {e}")
            failed += 1

    # Test 4: Flag values
    print("\n4. Flag Values")
    for name, hex_value in vectors["flags"].items():
        expected = int(hex_value, 16)
        try:
            actual = Flags[name].value
            assert actual == expected, f"Flags.{name}: expected 0x{expected:02x}, got 0x{actual:02x}"
            print(f"   ✓ Flags.{name} = 0x{expected:02x}")
            passed += 1
        except (KeyError, AssertionError) as e:
            print(f"   ✗ Flags.{name}: {e}")
            failed += 1

    # Test 5: Message creation and parsing
    print("\n5. Message Creation/Parsing")
    for vector in vectors["test_vectors"]:
        if "expected_hex" not in vector:
            continue

        name = vector["name"]
        msg_type = MessageType(int(vector["header"]["message_type"], 16))
        channel_id = vector["header"]["channel_id"]
        timestamp_ns = vector["header"]["timestamp_ns"]
        encoding = Encoding(int(vector["header"]["encoding"], 16))
        flags = int(vector["header"]["flags"], 16)
        payload = hex_to_bytes(vector.get("payload_hex", ""))
        expected = hex_to_bytes(vector["expected_hex"])

        try:
            # Test creation
            created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)
            assert created == expected, \
                f"Create mismatch:\n  Expected: {bytes_to_hex(expected)}\n  Got:      {bytes_to_hex(created)}"

            # Test parsing
            header, parsed_payload = parse_message(created)
            assert header is not None, "Failed to parse created message"
            assert header.message_type == msg_type, f"Parse messageType mismatch"
            assert header.channel_id == channel_id, f"Parse channelId mismatch"
            assert header.encoding == encoding, f"Parse encoding mismatch"
            assert header.payload_length == len(payload), f"Parse payloadLength mismatch"
            assert parsed_payload == payload, f"Parse payload mismatch"

            print(f"   ✓ {name}: {vector['description']}")
            passed += 1
        except AssertionError as e:
            print(f"   ✗ {name}: {e}")
            failed += 1

    # Test 6: Invalid message handling
    print("\n6. Invalid Message Handling")
    for vector in vectors["invalid_vectors"]:
        name = vector["name"]
        try:
            data = hex_to_bytes(vector["hex"])
            header, _ = parse_message(data)

            if header is None:
                print(f"   ✓ {name}: Correctly rejected ({vector['error']})")
                passed += 1
            else:
                print(f"   ✗ {name}: Should have been rejected but was parsed")
                failed += 1
        except Exception:
            # Expected to throw for invalid messages
            print(f"   ✓ {name}: Correctly threw error")
            passed += 1

    # Test 7: Media encoding round-trips
    print("\n7. Media Encoding Round-Trips")
    media_names = ["png_image_message", "jpeg_image_message", "laser_scan_cbor", "occupancy_grid_cbor", "point_cloud_cbor"]
    media_vectors = [v for v in vectors["test_vectors"] if v["name"] in media_names]

    for vector in media_vectors:
        name = vector["name"]
        msg_type = MessageType(int(vector["header"]["message_type"], 16))
        encoding = Encoding(int(vector["header"]["encoding"], 16))
        description = vector.get("payload_description", vector["description"])

        try:
            if "payload_hex" in vector:
                payload = hex_to_bytes(vector["payload_hex"])
                channel_id = vector["header"]["channel_id"]
                timestamp_ns = vector["header"]["timestamp_ns"]
                flags = int(vector["header"]["flags"], 16)

                # Create and parse round-trip
                created = create_message(msg_type, channel_id, payload, encoding, timestamp_ns, flags)
                header, parsed_payload = parse_message(created)

                assert header is not None, "Failed to parse created message"
                assert header.encoding == encoding, f"Encoding mismatch: expected {encoding}, got {header.encoding}"
                assert parsed_payload == payload, "Payload mismatch"

                print(f"   ✓ {name}: {description}")
                passed += 1
            elif "payload_cbor" in vector:
                # CBOR vectors - just verify they're defined
                print(f"   ✓ {name}: CBOR structure defined ({description})")
                passed += 1
        except AssertionError as e:
            print(f"   ✗ {name}: {e}")
            failed += 1

    return passed, failed


def main():
    passed, failed = run_tests()

    print("\n" + "=" * 50)
    print(f"Results: {passed} passed, {failed} failed")

    if failed > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
