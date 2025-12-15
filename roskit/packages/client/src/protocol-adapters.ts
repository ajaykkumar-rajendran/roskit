import {
  MessageType,
  Encoding,
  type Message,
  parseMessage,
  createMessage as createMessageFrame,
} from './protocol'

type ProtocolName = 'legacy' | 'rust' | 'python'

export interface ProtocolAdapter {
  name: ProtocolName
  parseMessage(data: ArrayBuffer): Message | null
  createMessage(
    messageType: MessageType,
    channelId: number,
    payload: Uint8Array,
    encoding: Encoding,
    timestampNs?: bigint,
    flags?: number
  ): ArrayBuffer
}

// ----- Legacy/Standard adapter (current TypeScript/Rust protocol) -----
// This adapter now uses the standardized 24-byte header "RK" protocol.
class LegacyAdapter implements ProtocolAdapter {
  name: ProtocolName = 'legacy'

  parseMessage(data: ArrayBuffer): Message | null {
    const [header, payload] = parseMessage(data)
    if (header === null) return null
    return { header, payload }
  }

  createMessage(
    messageType: MessageType,
    channelId: number,
    payload: Uint8Array,
    encoding: Encoding,
    timestampNs: bigint = BigInt(0),
    flags: number = 0
  ): ArrayBuffer {
    return createMessageFrame(messageType, channelId, payload, encoding, timestampNs, flags)
  }
}

// ----- Rust bridge adapter -----
// Uses the same standardized protocol as LegacyAdapter (24-byte header "RK")
class RustAdapter implements ProtocolAdapter {
  name: ProtocolName = 'rust'

  parseMessage(data: ArrayBuffer): Message | null {
    const [header, payload] = parseMessage(data)
    if (header === null) return null
    return { header, payload }
  }

  createMessage(
    messageType: MessageType,
    channelId: number,
    payload: Uint8Array,
    encoding: Encoding,
    timestampNs: bigint = BigInt(0),
    flags: number = 0
  ): ArrayBuffer {
    return createMessageFrame(messageType, channelId, payload, encoding, timestampNs, flags)
  }
}

// ----- Python bridge adapter -----
// Wire format (big endian):
// | Magic (2) | Version (1) | Type (1) | Channel (4) | Timestamp (8) | Encoding (1) | Flags (1) | Reserved (2) | Length (4) |
const PY_MAGIC = new Uint8Array([0x52, 0x4b]) // "RK"
const PY_HEADER_SIZE = 24

// Python wire type mappings (different from standardized protocol)
const PY_WIRE_TO_CANONICAL: Record<number, MessageType | undefined> = {
  0x01: MessageType.SUBSCRIBE,
  0x02: MessageType.UNSUBSCRIBE,
  0x03: MessageType.MESSAGE,
  0x04: MessageType.SERVICE_CALL,
  0x05: MessageType.TOPIC_LIST,
  0x06: MessageType.PING,
  0x07: MessageType.SERVICE_LIST,
  0x08: MessageType.NODE_LIST,
  0x10: MessageType.MESSAGE,
  0x11: MessageType.CHANNEL_INFO,
  0x12: MessageType.SERVICE_RESPONSE,
  0x13: MessageType.TOPIC_LIST_RESPONSE,
  0x14: MessageType.SERVER_INFO,
  0x15: MessageType.PONG,
  0x16: MessageType.SERVICE_LIST_RESPONSE,
  0x17: MessageType.NODE_LIST_RESPONSE,
  0xff: MessageType.ERROR,
}

const PY_CANONICAL_TO_WIRE: Record<MessageType, number | undefined> = {
  [MessageType.SUBSCRIBE]: 0x01,
  [MessageType.UNSUBSCRIBE]: 0x02,
  [MessageType.PUBLISH]: 0x03,
  [MessageType.SERVICE_CALL]: 0x04,
  [MessageType.TOPIC_LIST]: 0x05,
  [MessageType.SERVICE_LIST]: 0x07,
  [MessageType.NODE_LIST]: 0x08,
  [MessageType.PING]: 0x06,
  [MessageType.MESSAGE]: 0x10,
  [MessageType.SERVICE_RESPONSE]: 0x12,
  [MessageType.CHANNEL_INFO]: 0x11,
  [MessageType.SERVER_INFO]: 0x14,
  [MessageType.TOPIC_LIST_RESPONSE]: 0x13,
  [MessageType.SERVICE_LIST_RESPONSE]: 0x16,
  [MessageType.NODE_LIST_RESPONSE]: 0x17,
  [MessageType.PONG]: 0x15,
  [MessageType.ERROR]: 0xff,
}

// Python encoding mappings
const PY_ENCODING_MAP: Record<number, Encoding | undefined> = {
  0: Encoding.RAW,
  1: Encoding.CBOR,
  2: Encoding.JSON,
  3: Encoding.PNG,
  4: Encoding.JPEG,
  5: Encoding.BINARY,
}

class PythonAdapter implements ProtocolAdapter {
  name: ProtocolName = 'python'

  parseMessage(data: ArrayBuffer): Message | null {
    if (data.byteLength < PY_HEADER_SIZE) return null
    const view = new DataView(data)
    const bytes = new Uint8Array(data)

    if (bytes[0] !== PY_MAGIC[0] || bytes[1] !== PY_MAGIC[1]) return null
    if (view.getUint8(2) !== 1) return null

    const rawType = view.getUint8(3)
    const messageType = PY_WIRE_TO_CANONICAL[rawType]
    if (messageType === undefined) return null

    const channelId = view.getUint32(4, false)
    const timestampNs = view.getBigUint64(8, false)
    const rawEncoding = view.getUint8(16)
    const encoding = PY_ENCODING_MAP[rawEncoding] ?? Encoding.RAW
    const flags = view.getUint8(17)
    const payloadLength = view.getUint32(20, false)

    if (data.byteLength < PY_HEADER_SIZE + payloadLength) return null

    const payload = new Uint8Array(data, PY_HEADER_SIZE, payloadLength)
    return {
      header: {
        messageType,
        channelId,
        timestampNs,
        encoding,
        flags,
        payloadLength,
      },
      payload,
    }
  }

  createMessage(
    messageType: MessageType,
    channelId: number,
    payload: Uint8Array,
    encoding: Encoding,
    timestampNs: bigint = BigInt(0),
    flags: number = 0
  ): ArrayBuffer {
    const wireType = PY_CANONICAL_TO_WIRE[messageType]
    if (wireType === undefined) {
      throw new Error(`Python adapter does not support message type ${messageType}`)
    }

    const buffer = new ArrayBuffer(PY_HEADER_SIZE + payload.length)
    const view = new DataView(buffer)
    const bytes = new Uint8Array(buffer)

    bytes[0] = PY_MAGIC[0]
    bytes[1] = PY_MAGIC[1]
    view.setUint8(2, 1)
    view.setUint8(3, wireType)
    view.setUint32(4, channelId, false)
    view.setBigUint64(8, timestampNs, false)
    view.setUint8(16, encoding)
    view.setUint8(17, flags)
    view.setUint16(18, 0) // reserved
    view.setUint32(20, payload.length, false)
    bytes.set(payload, PY_HEADER_SIZE)

    return buffer
  }
}

// ----- Manager -----
export class ProtocolManager {
  private adapters: ProtocolAdapter[]
  private activeAdapter: ProtocolAdapter | null = null
  private readyResolver: (() => void) | null = null
  private readyPromise: Promise<void>

  constructor(preference: ProtocolName[] = ['legacy', 'rust', 'python']) {
    const registry: Record<ProtocolName, ProtocolAdapter> = {
      legacy: new LegacyAdapter(),
      rust: new RustAdapter(),
      python: new PythonAdapter(),
    }
    // Remove duplicates while preserving order
    const seen = new Set<ProtocolName>()
    this.adapters = preference
      .filter((name) => {
        if (seen.has(name)) return false
        seen.add(name)
        return true
      })
      .map((name) => registry[name])
    // Ensure we always have all adapters available as fallback
    ;(['legacy', 'rust', 'python'] as ProtocolName[]).forEach((name) => {
      if (!seen.has(name)) {
        this.adapters.push(registry[name])
      }
    })

    this.readyPromise = new Promise((resolve) => {
      this.readyResolver = resolve
    })
  }

  get adapterName(): ProtocolName | null {
    return this.activeAdapter?.name ?? null
  }

  isReady(): boolean {
    return this.activeAdapter !== null
  }

  async waitUntilReady(): Promise<void> {
    if (this.isReady()) return
    await this.readyPromise
  }

  detectAndParse(data: ArrayBuffer): Message | null {
    if (this.activeAdapter) {
      return this.activeAdapter.parseMessage(data)
    }

    for (const adapter of this.adapters) {
      const parsed = adapter.parseMessage(data)
      if (parsed) {
        this.activeAdapter = adapter
        this.readyResolver?.()
        return parsed
      }
    }

    return null
  }

  createMessage(
    messageType: MessageType,
    channelId: number,
    payload: Uint8Array,
    encoding: Encoding,
    timestampNs?: bigint,
    flags?: number
  ): ArrayBuffer {
    const adapter = this.activeAdapter ?? this.adapters[0]
    return adapter.createMessage(messageType, channelId, payload, encoding, timestampNs, flags)
  }
}

export const DEFAULT_PROTOCOL_ORDER: ProtocolName[] = ['rust', 'python', 'legacy']
