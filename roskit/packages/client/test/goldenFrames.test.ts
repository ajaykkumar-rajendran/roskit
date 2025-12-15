import { describe, it, expect } from 'vitest'
import { ProtocolManager } from '../src/protocol-adapters'
import { MessageType, Encoding, createMessage, parseMessage } from '../src/protocol'
import { encode as cborEncode } from 'cbor-x'

function toHex(buffer: ArrayBuffer | Uint8Array): string {
  const bytes = buffer instanceof Uint8Array ? buffer : new Uint8Array(buffer)
  return Array.from(bytes)
    .map((b) => b.toString(16).padStart(2, '0'))
    .join('')
}

function hexToBytes(hex: string): Uint8Array {
  const clean = hex.replace(/\s+/g, '')
  const bytes = new Uint8Array(clean.length / 2)
  for (let i = 0; i < bytes.length; i++) {
    bytes[i] = parseInt(clean.substr(i * 2, 2), 16)
  }
  return bytes
}

describe('Golden frames - standardized protocol', () => {
  const serverInfoPayload = new Uint8Array(
    cborEncode({
      name: 'roskit-bridge',
      version: '1.0.0',
      protocol_version: 1,
      capabilities: ['topics', 'services'],
      ros_distro: 'humble',
      encodings: ['cbor'],
    })
  )

  it('produces stable legacy frame for server info', () => {
    const frame = createMessage(MessageType.SERVER_INFO, 0, serverInfoPayload, Encoding.CBOR)
    const parsed = parseMessage(frame)
    expect(parsed?.header.messageType).toBe(MessageType.SERVER_INFO)
    expect(toHex(frame).startsWith('525701')).toBe(true) // Magic + version + type
  })

  it('produces stable rust frame for topic list response', () => {
    const pm = new ProtocolManager(['rust'])
    const payload = new Uint8Array(cborEncode({ topics: [{ name: '/chatter', msg_type: 'std_msgs/msg/String' }] }))
    const frame = pm.createMessage(MessageType.TOPIC_LIST_RESPONSE, 0, payload, Encoding.CBOR)
    const parsed = pm.detectAndParse(frame)
    expect(parsed?.header.messageType).toBe(MessageType.TOPIC_LIST_RESPONSE)
    expect(parsed?.payloadLength).toBe(payload.length)
    expect(toHex(frame).startsWith('524b01')).toBe(true) // RK + version
  })

  it('produces stable python frame for service list response', () => {
    const pm = new ProtocolManager(['python'])
    const payload = new Uint8Array(cborEncode({ services: [{ name: '/reset', types: ['std_srvs/srv/Empty'] }] }))
    const frame = pm.createMessage(MessageType.SERVICE_LIST_RESPONSE, 0, payload, Encoding.CBOR)
    const parsed = pm.detectAndParse(frame)
    expect(parsed?.header.messageType).toBe(MessageType.SERVICE_LIST_RESPONSE)
    expect(parsed?.payloadLength).toBe(payload.length)
    expect(toHex(frame).startsWith('524b01')).toBe(true)
  })

  it('parses python topic list response golden hex', () => {
    // Header (RK, v1, type 0x13, channel 0, ts 0, enc CBOR, flags 0, reserved, len 9)
    const hex =
      '524b011300000000000000000000000000010000000009' + // header (24 bytes)
      'a166746f7069637380' // payload { topics: [] }
    const bytes = hexToBytes(hex)
    const pm = new ProtocolManager(['python'])
    const parsed = pm.detectAndParse(bytes.buffer)
    expect(parsed?.header.messageType).toBe(MessageType.TOPIC_LIST_RESPONSE)
    expect(parsed?.payloadLength).toBe(9)
  })

  it('parses python service list response golden hex', () => {
    // Header (RK, v1, type 0x16, channel 0, ts 0, enc CBOR, flags 0, reserved, len 10)
    const hex =
      '524b01160000000000000000000000000001000000000a' + // header
      'a168736572766963657380' // payload { services: [] }
    const bytes = hexToBytes(hex)
    const pm = new ProtocolManager(['python'])
    const parsed = pm.detectAndParse(bytes.buffer)
    expect(parsed?.header.messageType).toBe(MessageType.SERVICE_LIST_RESPONSE)
    expect(parsed?.payloadLength).toBe(10)
  })

  it('encodes message frame with CBOR payload deterministically', () => {
    const payload = new Uint8Array(cborEncode({ data: 42 }))
    const frame = createMessage(MessageType.MESSAGE, 123, payload, Encoding.CBOR, BigInt(1000))
    const parsed = parseMessage(frame)
    expect(parsed?.header.channelId).toBe(123)
    expect(parsed?.header.timestampNs).toBe(BigInt(1000))
    expect(toHex(frame)).toContain(toHex(payload))
  })
})
