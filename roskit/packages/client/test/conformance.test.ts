import { describe, it, expect } from 'vitest'
import { ProtocolManager } from '../src/protocol-adapters'
import { MessageType, Encoding, createMessage, parseMessage } from '../src/protocol'
import { decodeLaserScanBinary } from '../src/decoders'
import { encode as cborEncode } from 'cbor-x'

describe('Protocol adapters', () => {
  it('detects and parses legacy frames', () => {
    const payload = new Uint8Array(cborEncode({ hello: 'world' }))
    const frame = createMessage(MessageType.TOPIC_LIST, 0, payload, Encoding.CBOR)
    const pm = new ProtocolManager(['legacy'])
    const parsed = pm.detectAndParse(frame as ArrayBuffer)
    expect(parsed?.header.messageType).toBe(MessageType.TOPIC_LIST)
  })

  it('parses rust-like frames via adapter', () => {
    // Rust adapter uses different magic/version; simulate via ProtocolManager detection
    const pm = new ProtocolManager(['rust'])
    const payload = new Uint8Array(cborEncode({ topics: [] }))
    const frame = pm.createMessage(MessageType.TOPIC_LIST_RESPONSE, 0, payload, Encoding.CBOR)
    const parsed = pm.detectAndParse(frame)
    expect(parsed?.header.messageType).toBe(MessageType.TOPIC_LIST_RESPONSE)
  })

  it('parses python-like frames via adapter', () => {
    const pm = new ProtocolManager(['python'])
    const payload = new Uint8Array(cborEncode({ topics: [] }))
    const frame = pm.createMessage(MessageType.TOPIC_LIST_RESPONSE, 0, payload, Encoding.CBOR)
    const parsed = pm.detectAndParse(frame)
    expect(parsed?.header.messageType).toBe(MessageType.TOPIC_LIST_RESPONSE)
  })

  it('parses python service list response with 24-byte header', () => {
    const pm = new ProtocolManager(['python'])
    const payload = new Uint8Array(cborEncode({ services: [{ name: '/s', types: ['std_srvs/srv/Empty'] }] }))
    const frame = pm.createMessage(MessageType.SERVICE_LIST_RESPONSE, 0, payload, Encoding.CBOR)
    const parsed = pm.detectAndParse(frame)
    expect(parsed?.header.messageType).toBe(MessageType.SERVICE_LIST_RESPONSE)
    expect(parsed?.header.payloadLength).toBe(payload.length)
  })
})

describe('Media round-trip', () => {
  it('decodes LaserScan metadata-prefixed binary', () => {
    const metadata = {
      angle_min: 0,
      angle_max: 1,
      angle_increment: 0.1,
      range_min: 0.1,
      range_max: 10,
      ranges_count: 2,
    }
    const metaBytes = new Uint8Array(cborEncode(metadata))
    const ranges = new Float32Array([1, 2])
    const payload = new Uint8Array(4 + metaBytes.byteLength + ranges.byteLength)
    const view = new DataView(payload.buffer)
    view.setUint32(0, metaBytes.byteLength, false)
    payload.set(metaBytes, 4)
    payload.set(new Uint8Array(ranges.buffer), 4 + metaBytes.byteLength)

    const decoded = decodeLaserScanBinary(payload)
    expect(decoded.ranges[0]).toBeCloseTo(1)
    expect(decoded.ranges[1]).toBeCloseTo(2)
    expect(decoded.angle_max).toBeCloseTo(1)
  })
})
