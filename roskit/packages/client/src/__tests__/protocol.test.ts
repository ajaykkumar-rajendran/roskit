/**
 * Protocol Conformance Tests
 *
 * Tests the binary protocol encoding/decoding matches between
 * TypeScript, Python, and Rust implementations.
 */

import { describe, it, expect } from 'vitest'
import { encode as cborEncode, decode as cborDecode } from 'cbor-x'
import {
  MessageType,
  Encoding,
  MAGIC,
  PROTOCOL_VERSION,
  HEADER_SIZE,
  createMessage,
  parseMessage,
} from '../protocol'

describe('Protocol Constants', () => {
  it('should have correct magic bytes', () => {
    // MAGIC is now a Uint8Array [0x52, 0x4B] = "RK"
    expect(MAGIC[0]).toBe(0x52) // 'R'
    expect(MAGIC[1]).toBe(0x4b) // 'K'
  })

  it('should have correct header size', () => {
    expect(HEADER_SIZE).toBe(24)
  })

  it('should have protocol version 1', () => {
    expect(PROTOCOL_VERSION).toBe(1)
  })
})

describe('Message Type Enum', () => {
  it('should have correct request message types', () => {
    expect(MessageType.SUBSCRIBE).toBe(0x01)
    expect(MessageType.UNSUBSCRIBE).toBe(0x02)
    expect(MessageType.PUBLISH).toBe(0x03)
    expect(MessageType.SERVICE_CALL).toBe(0x04)
    expect(MessageType.TOPIC_LIST).toBe(0x05)
    expect(MessageType.PING).toBe(0x06)
    expect(MessageType.SERVICE_LIST).toBe(0x07)
  })

  it('should have correct response message types', () => {
    expect(MessageType.MESSAGE).toBe(0x10)
    expect(MessageType.CHANNEL_INFO).toBe(0x11)
    expect(MessageType.SERVICE_RESPONSE).toBe(0x12)
    expect(MessageType.TOPIC_LIST_RESPONSE).toBe(0x13)
    expect(MessageType.SERVER_INFO).toBe(0x14)
    expect(MessageType.PONG).toBe(0x15)
    expect(MessageType.SERVICE_LIST_RESPONSE).toBe(0x16)
    expect(MessageType.ERROR).toBe(0xff)
  })
})

describe('Encoding Enum', () => {
  it('should have correct encoding types', () => {
    expect(Encoding.RAW).toBe(0x00)
    expect(Encoding.CBOR).toBe(0x01)
    expect(Encoding.JSON).toBe(0x02)
    expect(Encoding.PNG).toBe(0x03)
    expect(Encoding.JPEG).toBe(0x04)
    expect(Encoding.BINARY).toBe(0x05)
  })
})

describe('createMessage', () => {
  it('should create valid message with header', () => {
    const payload = new Uint8Array([1, 2, 3, 4])
    const message = createMessage(MessageType.MESSAGE, 42, payload, Encoding.CBOR)

    expect(message).toBeInstanceOf(ArrayBuffer)
    expect(message.byteLength).toBe(HEADER_SIZE + payload.byteLength)
  })

  it('should encode magic bytes correctly', () => {
    const message = createMessage(MessageType.PING, 0, new Uint8Array(0), Encoding.RAW)
    const bytes = new Uint8Array(message)

    // Magic bytes should be first 2 bytes "RK"
    expect(bytes[0]).toBe(0x52) // 'R'
    expect(bytes[1]).toBe(0x4b) // 'K'
  })

  it('should encode version correctly', () => {
    const message = createMessage(MessageType.PING, 0, new Uint8Array(0), Encoding.RAW)
    const bytes = new Uint8Array(message)
    expect(bytes[2]).toBe(PROTOCOL_VERSION)
  })

  it('should encode message type correctly', () => {
    const message = createMessage(MessageType.SUBSCRIBE, 0, new Uint8Array(0), Encoding.RAW)
    const bytes = new Uint8Array(message)
    expect(bytes[3]).toBe(MessageType.SUBSCRIBE)
  })

  it('should encode channel ID correctly as 32-bit', () => {
    const message = createMessage(MessageType.MESSAGE, 0x12345678, new Uint8Array(0), Encoding.RAW)
    const view = new DataView(message)
    const channelId = view.getUint32(4, false) // big-endian, 32-bit
    expect(channelId).toBe(0x12345678)
  })

  it('should encode timestamp as 64-bit', () => {
    const timestamp = BigInt('1234567890123456789')
    const message = createMessage(MessageType.MESSAGE, 0, new Uint8Array(0), Encoding.RAW, timestamp)

    // Timestamp is at offset 8, 8 bytes
    const view = new DataView(message)
    const decoded = view.getBigUint64(8, false)

    expect(decoded).toBe(timestamp)
  })

  it('should encode encoding type correctly', () => {
    const message = createMessage(MessageType.MESSAGE, 0, new Uint8Array(0), Encoding.PNG)
    const bytes = new Uint8Array(message)
    expect(bytes[16]).toBe(Encoding.PNG)
  })

  it('should encode payload length correctly', () => {
    const payload = new Uint8Array(256)
    const message = createMessage(MessageType.MESSAGE, 0, payload, Encoding.CBOR)
    const view = new DataView(message)
    const length = view.getUint32(20, false) // big-endian, at offset 20
    expect(length).toBe(256)
  })

  it('should append payload correctly', () => {
    const payload = new Uint8Array([0xde, 0xad, 0xbe, 0xef])
    const message = createMessage(MessageType.MESSAGE, 0, payload, Encoding.RAW)
    const bytes = new Uint8Array(message)

    expect(bytes[HEADER_SIZE]).toBe(0xde)
    expect(bytes[HEADER_SIZE + 1]).toBe(0xad)
    expect(bytes[HEADER_SIZE + 2]).toBe(0xbe)
    expect(bytes[HEADER_SIZE + 3]).toBe(0xef)
  })
})

describe('parseMessage', () => {
  it('should parse a valid message', () => {
    const payload = new Uint8Array([1, 2, 3, 4])
    const original = createMessage(MessageType.MESSAGE, 123, payload, Encoding.CBOR)
    const [header, parsedPayload] = parseMessage(original)

    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.MESSAGE)
    expect(header!.encoding).toBe(Encoding.CBOR)
    expect(header!.payloadLength).toBe(4)
    expect(header!.channelId).toBe(123)
    // Compare payload content
    expect(parsedPayload.length).toBe(payload.length)
    for (let i = 0; i < payload.length; i++) {
      expect(parsedPayload[i]).toBe(payload[i])
    }
  })

  it('should return null for invalid magic bytes', () => {
    const badMessage = new Uint8Array(HEADER_SIZE + 4)
    badMessage[0] = 0x00
    badMessage[1] = 0x00

    const [header] = parseMessage(badMessage)
    expect(header).toBeNull()
  })

  it('should return null for message too short', () => {
    const shortMessage = new Uint8Array(10)
    const [header] = parseMessage(shortMessage)
    expect(header).toBeNull()
  })

  it('should handle empty payload', () => {
    const message = createMessage(MessageType.PING, 0, new Uint8Array(0), Encoding.RAW)
    const [header, payload] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(payload.byteLength).toBe(0)
  })
})

describe('CBOR Encoding Round-trip', () => {
  it('should encode and decode SubscribeRequest', () => {
    const request = {
      topic: '/scan',
      msg_type: 'sensor_msgs/msg/LaserScan',
      throttle_ms: 100,
    }

    const encoded = cborEncode(request)
    const decoded = cborDecode(encoded)

    expect(decoded).toEqual(request)
  })

  it('should encode and decode TopicInfo', () => {
    const topics = {
      topics: [
        { name: '/map', types: ['nav_msgs/msg/OccupancyGrid'] },
        { name: '/scan', types: ['sensor_msgs/msg/LaserScan'] },
      ],
    }

    const encoded = cborEncode(topics)
    const decoded = cborDecode(encoded)

    expect(decoded).toEqual(topics)
  })

  it('should encode and decode ServerInfo', () => {
    const info = {
      name: 'roskit-bridge',
      version: '0.1.0',
      protocol_version: 1,
      capabilities: ['subscribe', 'publish', 'service'],
    }

    const encoded = cborEncode(info)
    const decoded = cborDecode(encoded)

    expect(decoded).toEqual(info)
  })

  it('should encode and decode ChannelInfo', () => {
    const channel = {
      channel_id: 42,
      topic: '/scan',
      msg_type: 'sensor_msgs/msg/LaserScan',
      encoding: 'binary',
    }

    const encoded = cborEncode(channel)
    const decoded = cborDecode(encoded)

    expect(decoded).toEqual(channel)
  })

  it('should handle nested message structures', () => {
    const pose = {
      header: {
        stamp: { sec: 1234567890, nanosec: 123456789 },
        frame_id: 'map',
      },
      pose: {
        position: { x: 1.5, y: 2.5, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: 0.707, w: 0.707 },
      },
    }

    const encoded = cborEncode(pose)
    const decoded = cborDecode(encoded)

    expect(decoded).toEqual(pose)
  })

  it('should handle arrays of floats efficiently', () => {
    const ranges = new Float32Array(1000).fill(5.0)
    const data = { ranges: Array.from(ranges) }

    const encoded = cborEncode(data)
    const decoded = cborDecode(encoded) as { ranges: number[] }

    expect(decoded.ranges.length).toBe(1000)
    expect(decoded.ranges[0]).toBeCloseTo(5.0)
  })
})

describe('Full Message Round-trip', () => {
  it('should create and parse SUBSCRIBE message', () => {
    const request = {
      topic: '/cmd_vel',
      msg_type: 'geometry_msgs/msg/Twist',
    }

    const payload = new Uint8Array(cborEncode(request))
    const message = createMessage(MessageType.SUBSCRIBE, 0, payload, Encoding.CBOR)
    const [header, parsedPayload] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.SUBSCRIBE)

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(request)
  })

  it('should create and parse MESSAGE response', () => {
    const pose = {
      position: { x: 1.0, y: 2.0, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
    }

    const payload = new Uint8Array(cborEncode(pose))
    const timestamp = BigInt(Date.now() * 1_000_000) // Convert ms to ns
    const message = createMessage(MessageType.MESSAGE, 42, payload, Encoding.CBOR, timestamp)
    const [header, parsedPayload] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.MESSAGE)
    expect(header!.channelId).toBe(42)

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(pose)
  })

  it('should handle large payloads', () => {
    // Simulate a large OccupancyGrid (1000x1000 = 1M cells)
    const gridSize = 100 * 100 // Use smaller size for test
    const gridData = new Int8Array(gridSize)
    for (let i = 0; i < gridSize; i++) {
      gridData[i] = Math.floor(Math.random() * 101) - 1 // -1 to 100
    }

    const payload = new Uint8Array(gridData.buffer)
    const message = createMessage(MessageType.MESSAGE, 1, payload, Encoding.RAW)
    const [header, parsedPayload] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(parsedPayload.byteLength).toBe(gridSize)
  })

  it('should create and parse SERVICE_LIST message', () => {
    const message = createMessage(MessageType.SERVICE_LIST, 0, new Uint8Array(0), Encoding.RAW)
    const [header, _payload] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.SERVICE_LIST)
  })

  it('should create and parse SERVICE_LIST_RESPONSE', () => {
    const services = {
      services: [
        { name: '/robot/enable', service_type: 'std_srvs/srv/SetBool' },
        { name: '/robot/trigger', service_type: 'std_srvs/srv/Trigger' },
      ],
    }

    const payload = new Uint8Array(cborEncode(services))
    const message = createMessage(MessageType.SERVICE_LIST_RESPONSE, 0, payload, Encoding.CBOR)
    const [header, parsedPayload] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.SERVICE_LIST_RESPONSE)

    const decoded = cborDecode(parsedPayload)
    expect(decoded).toEqual(services)
  })
})
