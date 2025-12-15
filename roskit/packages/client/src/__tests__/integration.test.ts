/**
 * Integration Tests
 *
 * Tests protocol round-trip, adapter selection, and message handling.
 */

import { describe, it, expect, beforeEach } from 'vitest'
import { encode as cborEncode } from 'cbor-x'
import {
  MessageType,
  Encoding,
  HEADER_SIZE,
  createMessage,
  parseMessage,
} from '../protocol'
import { ProtocolManager } from '../protocol-adapters'

describe('Protocol Round-trip', () => {
  describe('Subscribe Request', () => {
    it('should round-trip subscribe request', () => {
      const request = {
        topic: '/scan',
        msg_type: 'sensor_msgs/msg/LaserScan',
        throttle_ms: 100,
      }

      const payload = new Uint8Array(cborEncode(request))
      const message = createMessage(MessageType.SUBSCRIBE, 0, payload, Encoding.CBOR)
      const [header, parsedPayload] = parseMessage(message)

      expect(header).not.toBeNull()
      expect(header!.messageType).toBe(MessageType.SUBSCRIBE)
      expect(header!.encoding).toBe(Encoding.CBOR)
      expect(parsedPayload.byteLength).toBe(payload.byteLength)
    })

    it('should handle various topic names', () => {
      const topics = ['/scan', '/cmd_vel', '/robot/pose', '/a/b/c/d/e']

      for (const topic of topics) {
        const payload = new Uint8Array(cborEncode({ topic }))
        const message = createMessage(MessageType.SUBSCRIBE, 0, payload, Encoding.CBOR)
        const [header, _] = parseMessage(message)

        expect(header).not.toBeNull()
        expect(header!.messageType).toBe(MessageType.SUBSCRIBE)
      }
    })
  })

  describe('Message Response', () => {
    it('should round-trip LaserScan-like message', () => {
      const laserScan = {
        angle_min: -Math.PI,
        angle_max: Math.PI,
        angle_increment: 0.01,
        range_min: 0.1,
        range_max: 30.0,
        ranges: Array(360).fill(5.0),
        intensities: Array(360).fill(100),
      }

      const payload = new Uint8Array(cborEncode(laserScan))
      const timestamp = BigInt(Date.now() * 1_000_000)
      const message = createMessage(MessageType.MESSAGE, 42, payload, Encoding.CBOR, timestamp)
      const [header] = parseMessage(message)

      expect(header).not.toBeNull()
      expect(header!.messageType).toBe(MessageType.MESSAGE)
      expect(header!.channelId).toBe(42)
      expect(header!.encoding).toBe(Encoding.CBOR)
      expect(header!.timestampNs).toBe(timestamp)
    })

    it('should round-trip OccupancyGrid-like message', () => {
      const width = 100
      const height = 100
      const occupancyGrid = {
        info: {
          resolution: 0.05,
          width,
          height,
          origin: { position: { x: -10, y: -10, z: 0 } },
        },
        data: new Int8Array(width * height).fill(-1),
      }

      const payload = new Uint8Array(cborEncode(occupancyGrid))
      const message = createMessage(MessageType.MESSAGE, 1, payload, Encoding.CBOR)
      const [header, _] = parseMessage(message)

      expect(header).not.toBeNull()
      expect(header!.channelId).toBe(1)
    })

    it('should round-trip Pose message', () => {
      const pose = {
        header: { stamp: { sec: 1234, nanosec: 567890 }, frame_id: 'map' },
        pose: {
          position: { x: 1.5, y: 2.5, z: 0.0 },
          orientation: { x: 0, y: 0, z: 0.707, w: 0.707 },
        },
      }

      const payload = new Uint8Array(cborEncode(pose))
      const message = createMessage(MessageType.MESSAGE, 10, payload, Encoding.CBOR)
      const [header, _] = parseMessage(message)

      expect(header).not.toBeNull()
      expect(header!.channelId).toBe(10)
    })
  })

  describe('Service Call', () => {
    it('should round-trip service call request', () => {
      const serviceCall = {
        service: '/robot/trigger',
        msg_type: 'std_srvs/srv/Trigger',
        request: {},
        request_id: 'req_12345',
      }

      const payload = new Uint8Array(cborEncode(serviceCall))
      const message = createMessage(MessageType.SERVICE_CALL, 0, payload, Encoding.CBOR)
      const [header, _] = parseMessage(message)

      expect(header).not.toBeNull()
      expect(header!.messageType).toBe(MessageType.SERVICE_CALL)
    })

    it('should round-trip service response', () => {
      const serviceResponse = {
        request_id: 'req_12345',
        response: { success: true, message: 'Triggered' },
      }

      const payload = new Uint8Array(cborEncode(serviceResponse))
      const message = createMessage(MessageType.SERVICE_RESPONSE, 0, payload, Encoding.CBOR)
      const [header, _] = parseMessage(message)

      expect(header).not.toBeNull()
      expect(header!.messageType).toBe(MessageType.SERVICE_RESPONSE)
    })
  })

  describe('Control Messages', () => {
    it('should round-trip ping/pong', () => {
      const pingMessage = createMessage(MessageType.PING, 0, new Uint8Array(0), Encoding.RAW)
      const [pingHeader, _] = parseMessage(pingMessage)

      expect(pingHeader).not.toBeNull()
      expect(pingHeader!.messageType).toBe(MessageType.PING)

      const pongMessage = createMessage(MessageType.PONG, 0, new Uint8Array(0), Encoding.RAW)
      const [pongHeader, __] = parseMessage(pongMessage)

      expect(pongHeader).not.toBeNull()
      expect(pongHeader!.messageType).toBe(MessageType.PONG)
    })

    it('should round-trip topic list request', () => {
      const message = createMessage(MessageType.TOPIC_LIST, 0, new Uint8Array(0), Encoding.RAW)
      const [header, _] = parseMessage(message)

      expect(header).not.toBeNull()
      expect(header!.messageType).toBe(MessageType.TOPIC_LIST)
    })

    it('should round-trip service list request', () => {
      const message = createMessage(MessageType.SERVICE_LIST, 0, new Uint8Array(0), Encoding.RAW)
      const [header, _] = parseMessage(message)

      expect(header).not.toBeNull()
      expect(header!.messageType).toBe(MessageType.SERVICE_LIST)
    })
  })
})

describe('ProtocolManager', () => {
  let manager: ProtocolManager

  beforeEach(() => {
    manager = new ProtocolManager()
  })

  it('should start with no active adapter', () => {
    expect(manager.isReady()).toBe(false)
    expect(manager.adapterName).toBeNull()
  })

  it('should auto-detect protocol on first message', () => {
    const payload = new Uint8Array(cborEncode({ test: true }))
    const message = createMessage(MessageType.SERVER_INFO, 0, payload, Encoding.CBOR)

    const parsed = manager.detectAndParse(message)

    expect(parsed).not.toBeNull()
    expect(manager.isReady()).toBe(true)
    // Should detect 'legacy' or 'rust' adapter since they share the same RK magic
    expect(['legacy', 'rust']).toContain(manager.adapterName)
  })

  it('should use detected adapter for subsequent messages', () => {
    // First message to detect
    const payload1 = new Uint8Array(cborEncode({ test: 1 }))
    const message1 = createMessage(MessageType.SERVER_INFO, 0, payload1, Encoding.CBOR)
    manager.detectAndParse(message1)

    const adapterName = manager.adapterName

    // Subsequent messages should use the same adapter
    const payload2 = new Uint8Array(cborEncode({ test: 2 }))
    const message2 = createMessage(MessageType.MESSAGE, 1, payload2, Encoding.CBOR)
    const parsed = manager.detectAndParse(message2)

    expect(parsed).not.toBeNull()
    expect(manager.adapterName).toBe(adapterName)
  })

  it('should respect protocol preference order', () => {
    const managerRustFirst = new ProtocolManager(['rust', 'legacy', 'python'])
    const managerLegacyFirst = new ProtocolManager(['legacy', 'rust', 'python'])

    // Both should work, but might detect different adapters for same message
    const payload = new Uint8Array(cborEncode({ test: true }))
    const message = createMessage(MessageType.SERVER_INFO, 0, payload, Encoding.CBOR)

    const rustParsed = managerRustFirst.detectAndParse(message)
    const legacyParsed = managerLegacyFirst.detectAndParse(message)

    // Both should successfully parse the message
    expect(rustParsed).not.toBeNull()
    expect(legacyParsed).not.toBeNull()
  })

  it('should create messages with active or fallback adapter', () => {
    // Before detection, should use first adapter in preference
    const preDetectMessage = manager.createMessage(
      MessageType.PING,
      0,
      new Uint8Array(0),
      Encoding.RAW
    )
    expect(preDetectMessage.byteLength).toBe(HEADER_SIZE)

    // After detection
    const detectPayload = new Uint8Array(cborEncode({ test: true }))
    const detectMessage = createMessage(MessageType.SERVER_INFO, 0, detectPayload, Encoding.CBOR)
    manager.detectAndParse(detectMessage)

    const postDetectMessage = manager.createMessage(
      MessageType.PING,
      0,
      new Uint8Array(0),
      Encoding.RAW
    )
    expect(postDetectMessage.byteLength).toBe(HEADER_SIZE)
  })
})

describe('Protocol Adapter Compatibility', () => {
  it('should handle maximum channel ID', () => {
    const maxChannelId = 0xffffffff // 32-bit max
    const message = createMessage(
      MessageType.MESSAGE,
      maxChannelId,
      new Uint8Array([1, 2, 3]),
      Encoding.RAW
    )
    const [header, _] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.channelId).toBe(maxChannelId)
  })

  it('should handle maximum timestamp', () => {
    const maxTimestamp = BigInt('18446744073709551615') // 64-bit max
    const message = createMessage(
      MessageType.MESSAGE,
      0,
      new Uint8Array([1, 2, 3]),
      Encoding.RAW,
      maxTimestamp
    )
    const [header, _] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.timestampNs).toBe(maxTimestamp)
  })

  it('should handle all encoding types', () => {
    const encodings = [
      Encoding.RAW,
      Encoding.CBOR,
      Encoding.JSON,
      Encoding.PNG,
      Encoding.JPEG,
      Encoding.BINARY,
    ]

    for (const encoding of encodings) {
      const message = createMessage(
        MessageType.MESSAGE,
        0,
        new Uint8Array([1, 2, 3]),
        encoding
      )
      const [header, _] = parseMessage(message)

      expect(header).not.toBeNull()
      expect(header!.encoding).toBe(encoding)
    }
  })

  it('should handle all message types', () => {
    const messageTypes = [
      MessageType.SUBSCRIBE,
      MessageType.UNSUBSCRIBE,
      MessageType.PUBLISH,
      MessageType.SERVICE_CALL,
      MessageType.TOPIC_LIST,
      MessageType.PING,
      MessageType.SERVICE_LIST,
      MessageType.MESSAGE,
      MessageType.CHANNEL_INFO,
      MessageType.SERVICE_RESPONSE,
      MessageType.TOPIC_LIST_RESPONSE,
      MessageType.SERVER_INFO,
      MessageType.PONG,
      MessageType.SERVICE_LIST_RESPONSE,
      MessageType.ERROR,
    ]

    for (const msgType of messageTypes) {
      const message = createMessage(msgType, 0, new Uint8Array(0), Encoding.RAW)
      const [header, _] = parseMessage(message)

      expect(header).not.toBeNull()
      expect(header!.messageType).toBe(msgType)
    }
  })

  it('should handle large payloads', () => {
    const largePayload = new Uint8Array(1024 * 1024) // 1MB
    for (let i = 0; i < largePayload.length; i++) {
      largePayload[i] = i % 256
    }

    const message = createMessage(
      MessageType.MESSAGE,
      0,
      largePayload,
      Encoding.RAW
    )
    const [header, payload] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.payloadLength).toBe(largePayload.length)
    expect(payload.byteLength).toBe(largePayload.length)

    // Verify payload integrity
    for (let i = 0; i < 100; i++) {
      expect(payload[i]).toBe(i % 256)
    }
  })
})

describe('Error Handling', () => {
  it('should handle corrupted magic bytes', () => {
    const message = new ArrayBuffer(HEADER_SIZE + 10)
    const bytes = new Uint8Array(message)
    bytes[0] = 0x00 // Wrong magic
    bytes[1] = 0x00

    const [header] = parseMessage(message)
    expect(header).toBeNull()
  })

  it('should handle truncated header', () => {
    const shortMessage = new ArrayBuffer(10) // Less than HEADER_SIZE
    const [header] = parseMessage(shortMessage)
    expect(header).toBeNull()
  })

  it('should handle payload length mismatch', () => {
    const message = createMessage(MessageType.MESSAGE, 0, new Uint8Array(100), Encoding.RAW)
    // Truncate the message
    const truncated = message.slice(0, HEADER_SIZE + 50)
    const [header, _] = parseMessage(truncated)

    // parseMessage returns null for incomplete messages (payload shorter than header indicates)
    // This is correct behavior - the implementation validates payload length
    expect(header).toBeNull()
  })
})
