"""RosKit binary protocol constants and utilities.

Protocol frame format (24 bytes header):
| Magic (2) | Version (1) | Type (1) | Channel (4) | Timestamp (8) | Encoding (1) | Flags (1) | Reserved (2) | Length (4) | Payload |

Magic: "RK" (0x52, 0x4B)
Version: Protocol version (currently 1)
Type: Message type
Channel: Channel ID (for subscription messages)
Timestamp: Nanoseconds since epoch
Encoding: Payload encoding format
Flags: Message flags (compression, fragmentation)
Reserved: Reserved for future use
Length: Payload length in bytes
"""

import struct
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional

# Protocol magic bytes: "RK" (RosKit) - must match TypeScript/Rust
MAGIC = b'RK'
PROTOCOL_VERSION = 1

# Header size in bytes (24 bytes to match TypeScript/Rust)
HEADER_SIZE = 24


class MessageType(IntEnum):
    """Message types in the RosKit protocol.

    These values MUST match TypeScript (protocol.ts) and Rust (protocol.rs).
    """
    # Client -> Server
    SUBSCRIBE = 0x01
    UNSUBSCRIBE = 0x02
    PUBLISH = 0x03
    SERVICE_CALL = 0x04
    TOPIC_LIST = 0x05
    PING = 0x06
    SERVICE_LIST = 0x07
    NODE_LIST = 0x08

    # Server -> Client
    MESSAGE = 0x10
    CHANNEL_INFO = 0x11
    SERVICE_RESPONSE = 0x12
    TOPIC_LIST_RESPONSE = 0x13
    SERVER_INFO = 0x14
    PONG = 0x15
    SERVICE_LIST_RESPONSE = 0x16
    NODE_LIST_RESPONSE = 0x17
    ERROR = 0xFF


class Encoding(IntEnum):
    """Payload encoding types.

    These values MUST match TypeScript (protocol.ts) and Rust (protocol.rs).
    """
    RAW = 0x00
    CBOR = 0x01
    JSON = 0x02
    PNG = 0x03
    JPEG = 0x04
    BINARY = 0x05


class Flags(IntEnum):
    """Message flags."""
    NONE = 0x00
    COMPRESSED = 0x01
    FRAGMENTED = 0x02
    LAST_FRAGMENT = 0x04


@dataclass
class MessageHeader:
    """Binary message header (24 bytes).

    Layout:
    - Offset 0-1: Magic "RK" (2 bytes)
    - Offset 2: Version (1 byte)
    - Offset 3: Message type (1 byte)
    - Offset 4-7: Channel ID (4 bytes, big-endian)
    - Offset 8-15: Timestamp (8 bytes, big-endian nanoseconds)
    - Offset 16: Encoding (1 byte)
    - Offset 17: Flags (1 byte)
    - Offset 18-19: Reserved (2 bytes)
    - Offset 20-23: Payload length (4 bytes, big-endian)
    """
    message_type: MessageType
    channel_id: int
    timestamp_ns: int
    encoding: Encoding
    flags: int
    payload_length: int

    def to_bytes(self) -> bytes:
        """Serialize header to bytes (24 bytes)."""
        return struct.pack(
            '>2sBBIQBBHI',
            MAGIC,
            PROTOCOL_VERSION,
            self.message_type,
            self.channel_id,      # 4 bytes (I)
            self.timestamp_ns,    # 8 bytes (Q)
            self.encoding,        # 1 byte (B)
            self.flags,           # 1 byte (B)
            0,                    # 2 bytes reserved (H)
            self.payload_length   # 4 bytes (I)
        )

    @classmethod
    def from_bytes(cls, data: bytes) -> Optional['MessageHeader']:
        """Deserialize header from bytes."""
        if len(data) < HEADER_SIZE:
            return None

        magic, version, msg_type, channel_id, timestamp_ns, encoding, flags, _reserved, payload_length = struct.unpack(
            '>2sBBIQBBHI', data[:HEADER_SIZE]
        )

        if magic != MAGIC:
            return None

        if version != PROTOCOL_VERSION:
            return None

        return cls(
            message_type=MessageType(msg_type),
            channel_id=channel_id,
            timestamp_ns=timestamp_ns,
            encoding=Encoding(encoding),
            flags=flags,
            payload_length=payload_length
        )


def create_message(
    message_type: MessageType,
    channel_id: int,
    payload: bytes,
    encoding: Encoding = Encoding.CBOR,
    timestamp_ns: int = 0,
    flags: int = Flags.NONE
) -> bytes:
    """Create a complete binary message with header and payload."""
    header = MessageHeader(
        message_type=message_type,
        channel_id=channel_id,
        timestamp_ns=timestamp_ns,
        encoding=encoding,
        flags=flags,
        payload_length=len(payload)
    )
    return header.to_bytes() + payload


def parse_message(data: bytes) -> tuple[Optional[MessageHeader], bytes]:
    """Parse a binary message, returning header and payload."""
    header = MessageHeader.from_bytes(data)
    if header is None:
        return None, b''

    payload = data[HEADER_SIZE:HEADER_SIZE + header.payload_length]
    return header, payload
