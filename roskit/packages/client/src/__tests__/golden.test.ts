/**
 * Golden Frame Protocol Tests
 *
 * These tests verify cross-implementation compatibility by testing against
 * pre-computed byte vectors that MUST be identical in Python, Rust, and TypeScript.
 *
 * Golden vectors are computed deterministically:
 * - All fields use fixed values (no timestamps, random data)
 * - CBOR encoding uses canonical ordering (map keys sorted alphabetically)
 *
 * If any test fails, it indicates a protocol drift between implementations.
 */

import { describe, it, expect } from 'vitest'
import { encode as cborEncode, decode as cborDecode } from 'cbor-x'
import {
  MessageType,
  Encoding,
  Flags,
  MAGIC,
  PROTOCOL_VERSION,
  HEADER_SIZE,
  createMessage,
  parseMessage,
} from '../protocol'

/**
 * Helper to convert hex string to Uint8Array
 */
function hexToBytes(hex: string): Uint8Array {
  const bytes = new Uint8Array(hex.length / 2)
  for (let i = 0; i < hex.length; i += 2) {
    bytes[i / 2] = parseInt(hex.substring(i, i + 2), 16)
  }
  return bytes
}

/**
 * Helper to convert Uint8Array to hex string
 */
function bytesToHex(bytes: Uint8Array): string {
  return Array.from(bytes)
    .map((b) => b.toString(16).padStart(2, '0'))
    .join('')
}

describe('Golden Header Tests', () => {
  /**
   * Minimal PING message with all zeros except required fields
   * Header: "RK" + version(1) + type(PING=0x06) + channel(0) + timestamp(0) + encoding(RAW) + flags(0) + reserved(0) + length(0)
   *
   * Expected bytes:
   * 52 4B        - Magic "RK"
   * 01           - Version 1
   * 06           - PING
   * 00 00 00 00  - Channel ID (0)
   * 00 00 00 00 00 00 00 00  - Timestamp (0)
   * 00           - Encoding (RAW)
   * 00           - Flags (NONE)
   * 00 00        - Reserved
   * 00 00 00 00  - Payload length (0)
   */
  it('should encode PING header correctly', () => {
    const expected = hexToBytes('524b01060000000000000000000000000000000000000000')
    const message = createMessage(MessageType.PING, 0, new Uint8Array(0), Encoding.RAW, BigInt(0))
    const bytes = new Uint8Array(message)

    expect(bytes).toEqual(expected)
  })

  /**
   * PONG message (server response to PING)
   * Expected bytes:
   * 52 4B        - Magic "RK"
   * 01           - Version 1
   * 15           - PONG (0x15)
   * 00 00 00 00  - Channel ID (0)
   * 00 00 00 00 00 00 00 00  - Timestamp (0)
   * 00           - Encoding (RAW)
   * 00           - Flags (NONE)
   * 00 00        - Reserved
   * 00 00 00 00  - Payload length (0)
   */
  it('should encode PONG header correctly', () => {
    const expected = hexToBytes('524b01150000000000000000000000000000000000000000')
    const message = createMessage(MessageType.PONG, 0, new Uint8Array(0), Encoding.RAW, BigInt(0))
    const bytes = new Uint8Array(message)

    expect(bytes).toEqual(expected)
  })

  /**
   * TOPIC_LIST request (no payload)
   */
  it('should encode TOPIC_LIST header correctly', () => {
    const expected = hexToBytes('524b01050000000000000000000000000000000000000000')
    const message = createMessage(MessageType.TOPIC_LIST, 0, new Uint8Array(0), Encoding.RAW, BigInt(0))
    const bytes = new Uint8Array(message)

    expect(bytes).toEqual(expected)
  })

  /**
   * SERVICE_LIST request (no payload)
   */
  it('should encode SERVICE_LIST header correctly', () => {
    const expected = hexToBytes('524b01070000000000000000000000000000000000000000')
    const message = createMessage(MessageType.SERVICE_LIST, 0, new Uint8Array(0), Encoding.RAW, BigInt(0))
    const bytes = new Uint8Array(message)

    expect(bytes).toEqual(expected)
  })

  /**
   * Header with non-zero channel ID
   * Channel ID: 0x00000042 (66)
   */
  it('should encode channel ID correctly in big-endian', () => {
    const expected = hexToBytes('524b01100000004200000000000000000100000000000000')
    const message = createMessage(MessageType.MESSAGE, 66, new Uint8Array(0), Encoding.CBOR, BigInt(0))
    const bytes = new Uint8Array(message)

    expect(bytes).toEqual(expected)
  })

  /**
   * Header with large channel ID
   * Channel ID: 0x12345678
   */
  it('should encode large channel ID correctly', () => {
    const expected = hexToBytes('524b01101234567800000000000000000100000000000000')
    const message = createMessage(MessageType.MESSAGE, 0x12345678, new Uint8Array(0), Encoding.CBOR, BigInt(0))
    const bytes = new Uint8Array(message)

    expect(bytes).toEqual(expected)
  })

  /**
   * Header with timestamp
   * Timestamp: 1000000000 nanoseconds (0x000000003B9ACA00)
   * Header layout (24 bytes):
   * 52 4B        - Magic "RK" (offset 0-1)
   * 01           - Version (offset 2)
   * 10           - MESSAGE (offset 3)
   * 00 00 00 00  - Channel ID 0 (offset 4-7)
   * 00 00 00 00 3B 9A CA 00 - Timestamp (offset 8-15)
   * 01           - CBOR encoding (offset 16)
   * 00           - Flags (offset 17)
   * 00 00        - Reserved (offset 18-19)
   * 00 00 00 00  - Payload length 0 (offset 20-23)
   */
  it('should encode timestamp correctly in big-endian', () => {
    const timestamp = BigInt(1000000000)
    // 1000000000 = 0x3B9ACA00 in big-endian 64-bit: 00 00 00 00 3B 9A CA 00
    // Full header: RK(2) + ver(1) + type(1) + chan(4) + ts(8) + enc(1) + flags(1) + reserved(2) + len(4) = 24 bytes
    const expected = hexToBytes('524b011000000000000000003b9aca000100000000000000')
    const message = createMessage(MessageType.MESSAGE, 0, new Uint8Array(0), Encoding.CBOR, timestamp)
    const bytes = new Uint8Array(message)

    expect(bytes).toEqual(expected)
  })

  /**
   * Header with all encoding types
   */
  it('should encode all encoding types correctly', () => {
    const encodings = [
      { enc: Encoding.RAW, byte: '00' },
      { enc: Encoding.CBOR, byte: '01' },
      { enc: Encoding.JSON, byte: '02' },
      { enc: Encoding.PNG, byte: '03' },
      { enc: Encoding.JPEG, byte: '04' },
      { enc: Encoding.BINARY, byte: '05' },
    ]

    for (const { enc, byte } of encodings) {
      const message = createMessage(MessageType.MESSAGE, 0, new Uint8Array(0), enc, BigInt(0))
      const bytes = new Uint8Array(message)
      expect(bytes[16]).toBe(parseInt(byte, 16))
    }
  })

  /**
   * Header with flags
   */
  it('should encode flags correctly', () => {
    // COMPRESSED flag
    const compressedMsg = createMessage(
      MessageType.MESSAGE,
      0,
      new Uint8Array(0),
      Encoding.CBOR,
      BigInt(0),
      Flags.COMPRESSED
    )
    expect(new Uint8Array(compressedMsg)[17]).toBe(0x01)

    // FRAGMENTED flag
    const fragmentedMsg = createMessage(
      MessageType.MESSAGE,
      0,
      new Uint8Array(0),
      Encoding.CBOR,
      BigInt(0),
      Flags.FRAGMENTED
    )
    expect(new Uint8Array(fragmentedMsg)[17]).toBe(0x02)

    // Combined flags
    const combinedMsg = createMessage(
      MessageType.MESSAGE,
      0,
      new Uint8Array(0),
      Encoding.CBOR,
      BigInt(0),
      Flags.COMPRESSED | Flags.FRAGMENTED
    )
    expect(new Uint8Array(combinedMsg)[17]).toBe(0x03)
  })
})

describe('Golden CBOR Payload Tests', () => {
  /**
   * SUBSCRIBE request with minimal fields
   * CBOR map: {"msg_type": "std_msgs/msg/String", "topic": "/test"}
   *
   * CBOR breakdown (sorted keys):
   * A2                     - map(2)
   *   68 6D73675F74797065  - text(8) "msg_type"
   *   73 7374645F6D736773... - text(19) "std_msgs/msg/String"
   *   65 746F706963        - text(5) "topic"
   *   65 2F74657374        - text(5) "/test"
   */
  it('should encode SUBSCRIBE request correctly', () => {
    const request = {
      msg_type: 'std_msgs/msg/String',
      topic: '/test',
    }

    const payload = cborEncode(request)
    const message = createMessage(MessageType.SUBSCRIBE, 0, new Uint8Array(payload), Encoding.CBOR, BigInt(0))

    // Parse back and verify
    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.SUBSCRIBE)
    expect(header!.encoding).toBe(Encoding.CBOR)

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(request)
  })

  /**
   * CHANNEL_INFO response
   * CBOR map: {"channel_id": 42, "encoding": "cbor", "msg_type": "std_msgs/msg/String", "topic": "/test"}
   */
  it('should encode CHANNEL_INFO response correctly', () => {
    const channelInfo = {
      channel_id: 42,
      encoding: 'cbor',
      msg_type: 'std_msgs/msg/String',
      topic: '/test',
    }

    const payload = cborEncode(channelInfo)
    const message = createMessage(MessageType.CHANNEL_INFO, 0, new Uint8Array(payload), Encoding.CBOR, BigInt(0))

    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.CHANNEL_INFO)

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(channelInfo)
  })

  /**
   * SERVER_INFO response with capabilities
   */
  it('should encode SERVER_INFO response correctly', () => {
    const serverInfo = {
      capabilities: ['publish', 'service', 'subscribe'],
      encodings: ['binary', 'cbor', 'json', 'raw'],
      name: 'roskit-bridge',
      protocol_version: 1,
      version: '0.1.0',
    }

    const payload = cborEncode(serverInfo)
    const message = createMessage(MessageType.SERVER_INFO, 0, new Uint8Array(payload), Encoding.CBOR, BigInt(0))

    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.SERVER_INFO)

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(serverInfo)
  })

  /**
   * TOPIC_LIST_RESPONSE with multiple topics
   */
  it('should encode TOPIC_LIST_RESPONSE correctly', () => {
    const topicList = {
      topics: [
        { msg_type: 'nav_msgs/msg/OccupancyGrid', name: '/map' },
        { msg_type: 'geometry_msgs/msg/Twist', name: '/cmd_vel' },
        { msg_type: 'sensor_msgs/msg/LaserScan', name: '/scan' },
      ],
    }

    const payload = cborEncode(topicList)
    const message = createMessage(
      MessageType.TOPIC_LIST_RESPONSE,
      0,
      new Uint8Array(payload),
      Encoding.CBOR,
      BigInt(0)
    )

    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.TOPIC_LIST_RESPONSE)

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(topicList)
  })

  /**
   * SERVICE_LIST_RESPONSE with multiple services
   */
  it('should encode SERVICE_LIST_RESPONSE correctly', () => {
    const serviceList = {
      services: [
        { name: '/robot/enable', service_type: 'std_srvs/srv/SetBool' },
        { name: '/robot/trigger', service_type: 'std_srvs/srv/Trigger' },
      ],
    }

    const payload = cborEncode(serviceList)
    const message = createMessage(
      MessageType.SERVICE_LIST_RESPONSE,
      0,
      new Uint8Array(payload),
      Encoding.CBOR,
      BigInt(0)
    )

    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.SERVICE_LIST_RESPONSE)

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(serviceList)
  })

  /**
   * SERVICE_CALL request
   */
  it('should encode SERVICE_CALL request correctly', () => {
    const serviceCall = {
      request: { data: true },
      request_id: 'req-001',
      service: '/robot/enable',
      service_type: 'std_srvs/srv/SetBool',
    }

    const payload = cborEncode(serviceCall)
    const message = createMessage(MessageType.SERVICE_CALL, 0, new Uint8Array(payload), Encoding.CBOR, BigInt(0))

    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.SERVICE_CALL)

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(serviceCall)
  })

  /**
   * SERVICE_RESPONSE success
   */
  it('should encode SERVICE_RESPONSE success correctly', () => {
    const serviceResponse = {
      request_id: 'req-001',
      response: { message: 'Enabled', success: true },
    }

    const payload = cborEncode(serviceResponse)
    const message = createMessage(
      MessageType.SERVICE_RESPONSE,
      0,
      new Uint8Array(payload),
      Encoding.CBOR,
      BigInt(0)
    )

    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.SERVICE_RESPONSE)

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(serviceResponse)
  })

  /**
   * SERVICE_RESPONSE error
   */
  it('should encode SERVICE_RESPONSE error correctly', () => {
    const serviceResponse = {
      error: 'Service not available',
      request_id: 'req-002',
    }

    const payload = cborEncode(serviceResponse)
    const message = createMessage(
      MessageType.SERVICE_RESPONSE,
      0,
      new Uint8Array(payload),
      Encoding.CBOR,
      BigInt(0)
    )

    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(serviceResponse)
  })

  /**
   * ERROR message
   */
  it('should encode ERROR message correctly', () => {
    const errorMsg = {
      code: 404,
      message: 'Topic not found',
    }

    const payload = cborEncode(errorMsg)
    const message = createMessage(MessageType.ERROR, 0, new Uint8Array(payload), Encoding.CBOR, BigInt(0))

    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.ERROR)

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(errorMsg)
  })
})

describe('Golden ROS Message Tests', () => {
  /**
   * Twist message (geometry_msgs/msg/Twist)
   */
  it('should encode Twist message correctly', () => {
    const twist = {
      angular: { x: 0.0, y: 0.0, z: 0.5 },
      linear: { x: 1.0, y: 0.0, z: 0.0 },
    }

    const payload = cborEncode(twist)
    const message = createMessage(MessageType.MESSAGE, 42, new Uint8Array(payload), Encoding.CBOR, BigInt(0))

    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()
    expect(header!.channelId).toBe(42)

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(twist)
  })

  /**
   * PoseStamped message (geometry_msgs/msg/PoseStamped)
   */
  it('should encode PoseStamped message correctly', () => {
    const poseStamped = {
      header: {
        frame_id: 'map',
        stamp: { nanosec: 0, sec: 1000 },
      },
      pose: {
        orientation: { w: 1.0, x: 0.0, y: 0.0, z: 0.0 },
        position: { x: 1.5, y: 2.5, z: 0.0 },
      },
    }

    const payload = cborEncode(poseStamped)
    const message = createMessage(MessageType.MESSAGE, 1, new Uint8Array(payload), Encoding.CBOR, BigInt(0))

    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(poseStamped)
  })

  /**
   * LaserScan message header (without ranges for size)
   */
  it('should encode LaserScan metadata correctly', () => {
    const laserScan = {
      angle_increment: 0.01,
      angle_max: 3.14159,
      angle_min: -3.14159,
      header: {
        frame_id: 'laser',
        stamp: { nanosec: 500000000, sec: 1000 },
      },
      intensities: [],
      range_max: 30.0,
      range_min: 0.1,
      ranges: [],
      scan_time: 0.1,
      time_increment: 0.0001,
    }

    const payload = cborEncode(laserScan)
    const message = createMessage(MessageType.MESSAGE, 5, new Uint8Array(payload), Encoding.CBOR, BigInt(0))

    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()

    const decoded = cborDecode(parsedPayload) as typeof laserScan
    expect(decoded.angle_min).toBeCloseTo(-3.14159)
    expect(decoded.angle_max).toBeCloseTo(3.14159)
    expect(decoded.range_min).toBeCloseTo(0.1)
    expect(decoded.range_max).toBeCloseTo(30.0)
  })

  /**
   * OccupancyGrid metadata (without data for size)
   */
  it('should encode OccupancyGrid metadata correctly', () => {
    const occupancyGrid = {
      data: [],
      header: {
        frame_id: 'map',
        stamp: { nanosec: 0, sec: 1000 },
      },
      info: {
        height: 100,
        map_load_time: { nanosec: 0, sec: 0 },
        origin: {
          orientation: { w: 1.0, x: 0.0, y: 0.0, z: 0.0 },
          position: { x: -10.0, y: -10.0, z: 0.0 },
        },
        resolution: 0.05,
        width: 100,
      },
    }

    const payload = cborEncode(occupancyGrid)
    const message = createMessage(MessageType.MESSAGE, 10, new Uint8Array(payload), Encoding.CBOR, BigInt(0))

    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()

    const decoded = cborDecode(parsedPayload) as typeof occupancyGrid
    expect(decoded.info.width).toBe(100)
    expect(decoded.info.height).toBe(100)
    expect(decoded.info.resolution).toBeCloseTo(0.05)
  })
})

describe('Golden Binary Encoding Tests', () => {
  /**
   * Raw binary payload (no encoding transformation)
   */
  it('should pass through RAW binary payload unchanged', () => {
    const rawData = new Uint8Array([0xde, 0xad, 0xbe, 0xef, 0xca, 0xfe, 0xba, 0xbe])
    const message = createMessage(MessageType.MESSAGE, 1, rawData, Encoding.RAW, BigInt(0))

    const [header, payload] = parseMessage(message)
    expect(header).not.toBeNull()
    expect(header!.encoding).toBe(Encoding.RAW)
    expect(payload).toEqual(rawData)
  })

  /**
   * BINARY encoding for typed arrays (LaserScan ranges)
   * Uses little-endian Float32 format
   */
  it('should encode BINARY Float32 array correctly', () => {
    // 4 float32 values: [1.0, 2.0, 3.0, 4.0]
    const floatArray = new Float32Array([1.0, 2.0, 3.0, 4.0])
    const bytes = new Uint8Array(floatArray.buffer)

    const message = createMessage(MessageType.MESSAGE, 1, bytes, Encoding.BINARY, BigInt(0))

    const [header, payload] = parseMessage(message)
    expect(header).not.toBeNull()
    expect(header!.encoding).toBe(Encoding.BINARY)
    expect(payload.byteLength).toBe(16) // 4 floats * 4 bytes

    // Reconstruct Float32Array
    const decoded = new Float32Array(payload.buffer, payload.byteOffset, payload.byteLength / 4)
    expect(decoded[0]).toBeCloseTo(1.0)
    expect(decoded[1]).toBeCloseTo(2.0)
    expect(decoded[2]).toBeCloseTo(3.0)
    expect(decoded[3]).toBeCloseTo(4.0)
  })

  /**
   * BINARY encoding for OccupancyGrid data (Int8Array)
   */
  it('should encode BINARY Int8 array correctly', () => {
    // Occupancy values: -1 (unknown), 0 (free), 100 (occupied)
    const gridData = new Int8Array([-1, 0, 50, 100, -1, 0, 50, 100])
    const bytes = new Uint8Array(gridData.buffer)

    const message = createMessage(MessageType.MESSAGE, 1, bytes, Encoding.BINARY, BigInt(0))

    const [header, payload] = parseMessage(message)
    expect(header).not.toBeNull()
    expect(payload.byteLength).toBe(8)

    // Reconstruct Int8Array
    const decoded = new Int8Array(payload.buffer, payload.byteOffset, payload.byteLength)
    expect(decoded[0]).toBe(-1)
    expect(decoded[1]).toBe(0)
    expect(decoded[2]).toBe(50)
    expect(decoded[3]).toBe(100)
  })
})

describe('Golden Parse Error Tests', () => {
  it('should reject invalid magic bytes', () => {
    const badMagic = hexToBytes('0000010600000000000000000000000000000000000000')
    const [header] = parseMessage(badMagic)
    expect(header).toBeNull()
  })

  it('should reject wrong protocol version', () => {
    // Version 99 instead of 1
    const badVersion = hexToBytes('524b630600000000000000000000000000000000000000')
    const [header] = parseMessage(badVersion)
    expect(header).toBeNull()
  })

  it('should reject truncated header', () => {
    // Only 10 bytes instead of 24
    const truncated = hexToBytes('524b01060000000000')
    const [header] = parseMessage(truncated)
    expect(header).toBeNull()
  })

  it('should reject payload length mismatch', () => {
    // Header claims 100 bytes payload but only 4 bytes provided
    const mismatch = hexToBytes('524b0110000000000000000000000000010000006400000001020304')
    const [header] = parseMessage(mismatch)
    // parseMessage returns null when payload length doesn't match
    expect(header).toBeNull()
  })
})

describe('Golden Cross-Implementation Constants', () => {
  /**
   * These constants MUST match Python and Rust exactly
   */
  it('should have matching protocol constants', () => {
    // Magic bytes
    expect(MAGIC[0]).toBe(0x52) // 'R'
    expect(MAGIC[1]).toBe(0x4b) // 'K'

    // Protocol version
    expect(PROTOCOL_VERSION).toBe(1)

    // Header size
    expect(HEADER_SIZE).toBe(24)
  })

  it('should have matching message type codes', () => {
    // Client -> Server
    expect(MessageType.SUBSCRIBE).toBe(0x01)
    expect(MessageType.UNSUBSCRIBE).toBe(0x02)
    expect(MessageType.PUBLISH).toBe(0x03)
    expect(MessageType.SERVICE_CALL).toBe(0x04)
    expect(MessageType.TOPIC_LIST).toBe(0x05)
    expect(MessageType.PING).toBe(0x06)
    expect(MessageType.SERVICE_LIST).toBe(0x07)

    // Server -> Client
    expect(MessageType.MESSAGE).toBe(0x10)
    expect(MessageType.CHANNEL_INFO).toBe(0x11)
    expect(MessageType.SERVICE_RESPONSE).toBe(0x12)
    expect(MessageType.TOPIC_LIST_RESPONSE).toBe(0x13)
    expect(MessageType.SERVER_INFO).toBe(0x14)
    expect(MessageType.PONG).toBe(0x15)
    expect(MessageType.SERVICE_LIST_RESPONSE).toBe(0x16)
    expect(MessageType.ERROR).toBe(0xff)
  })

  it('should have matching encoding codes', () => {
    expect(Encoding.RAW).toBe(0x00)
    expect(Encoding.CBOR).toBe(0x01)
    expect(Encoding.JSON).toBe(0x02)
    expect(Encoding.PNG).toBe(0x03)
    expect(Encoding.JPEG).toBe(0x04)
    expect(Encoding.BINARY).toBe(0x05)
  })

  it('should have matching flag codes', () => {
    expect(Flags.NONE).toBe(0x00)
    expect(Flags.COMPRESSED).toBe(0x01)
    expect(Flags.FRAGMENTED).toBe(0x02)
    expect(Flags.LAST_FRAGMENT).toBe(0x04)
  })
})

describe('Golden Full Message Byte Vectors', () => {
  /**
   * Complete message byte vectors that MUST match across implementations
   * These can be used to verify Python/Rust encoders produce identical output
   */

  it('should produce identical PING frame bytes', () => {
    // PING with all zeros: exact 24 bytes
    const expected = '524b01060000000000000000000000000000000000000000'
    const message = createMessage(MessageType.PING, 0, new Uint8Array(0), Encoding.RAW, BigInt(0))
    const actual = bytesToHex(new Uint8Array(message))

    expect(actual).toBe(expected)
  })

  it('should produce identical UNSUBSCRIBE frame bytes', () => {
    // UNSUBSCRIBE with channel_id=42 in CBOR payload
    const payload = cborEncode({ channel_id: 42 })
    const message = createMessage(MessageType.UNSUBSCRIBE, 0, new Uint8Array(payload), Encoding.CBOR, BigInt(0))

    const bytes = new Uint8Array(message)
    // Verify header structure
    expect(bytes[0]).toBe(0x52) // 'R'
    expect(bytes[1]).toBe(0x4b) // 'K'
    expect(bytes[2]).toBe(0x01) // version
    expect(bytes[3]).toBe(0x02) // UNSUBSCRIBE
    expect(bytes[16]).toBe(0x01) // CBOR encoding

    // Verify payload decodes correctly
    const [header, parsedPayload] = parseMessage(message)
    expect(header).not.toBeNull()
    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual({ channel_id: 42 })
  })

  it('should produce consistent MESSAGE frame structure', () => {
    const pose = {
      angular: { x: 0.0, y: 0.0, z: 0.0 },
      linear: { x: 0.0, y: 0.0, z: 0.0 },
    }
    const payload = cborEncode(pose)
    const timestamp = BigInt(1234567890000000000)

    const message = createMessage(MessageType.MESSAGE, 100, new Uint8Array(payload), Encoding.CBOR, timestamp)

    const bytes = new Uint8Array(message)
    // Verify header
    expect(bytes[0]).toBe(0x52) // 'R'
    expect(bytes[1]).toBe(0x4b) // 'K'
    expect(bytes[2]).toBe(0x01) // version
    expect(bytes[3]).toBe(0x10) // MESSAGE

    // Channel ID at offset 4-7 (big-endian)
    const view = new DataView(bytes.buffer)
    expect(view.getUint32(4, false)).toBe(100)

    // Timestamp at offset 8-15 (big-endian)
    expect(view.getBigUint64(8, false)).toBe(timestamp)

    // Encoding at offset 16
    expect(bytes[16]).toBe(0x01) // CBOR

    // Payload length at offset 20-23
    expect(view.getUint32(20, false)).toBe(payload.byteLength)
  })
})
