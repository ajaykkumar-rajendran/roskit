/**
 * RosKit Binary Protocol Constants and Types
 *
 * Protocol frame format (24 bytes header):
 * | Magic (2) | Version (1) | Type (1) | Channel (4) | Timestamp (8) | Encoding (1) | Flags (1) | Reserved (2) | Length (4) | Payload |
 *
 * Magic: "RK" (0x52, 0x4B)
 * Version: Protocol version (currently 1)
 * Type: Message type
 * Channel: Channel ID (for subscription messages)
 * Timestamp: Nanoseconds since epoch
 * Encoding: Payload encoding format
 * Flags: Message flags (compression, fragmentation)
 * Reserved: Reserved for future use
 * Length: Payload length in bytes
 */

// Protocol constants
export const MAGIC = new Uint8Array([0x52, 0x4b]) // "RK"
export const PROTOCOL_VERSION = 1
export const HEADER_SIZE = 24

// Message types
export enum MessageType {
  // Client -> Server
  SUBSCRIBE = 0x01,
  UNSUBSCRIBE = 0x02,
  PUBLISH = 0x03,
  SERVICE_CALL = 0x04,
  TOPIC_LIST = 0x05,
  PING = 0x06,
  SERVICE_LIST = 0x07,
  NODE_LIST = 0x08,

  // Server -> Client
  MESSAGE = 0x10,
  CHANNEL_INFO = 0x11,
  SERVICE_RESPONSE = 0x12,
  TOPIC_LIST_RESPONSE = 0x13,
  SERVER_INFO = 0x14,
  PONG = 0x15,
  SERVICE_LIST_RESPONSE = 0x16,
  NODE_LIST_RESPONSE = 0x17,
  ERROR = 0xff,
}

// Payload encoding types
export enum Encoding {
  RAW = 0x00,
  CBOR = 0x01,
  JSON = 0x02,
  PNG = 0x03,
  JPEG = 0x04,
  BINARY = 0x05,
}

// Message flags
export enum Flags {
  NONE = 0x00,
  COMPRESSED = 0x01,
  FRAGMENTED = 0x02,
  LAST_FRAGMENT = 0x04,
}

// Parsed message header
export interface MessageHeader {
  messageType: MessageType
  channelId: number
  timestampNs: bigint
  encoding: Encoding
  flags: number
  payloadLength: number
}

// Parsed message
export interface Message {
  header: MessageHeader
  payload: Uint8Array
}

/**
 * Create a binary message frame
 *
 * Header layout (24 bytes):
 * Offset 0-1: Magic "RK"
 * Offset 2: Version
 * Offset 3: Message type
 * Offset 4-7: Channel ID (32-bit big-endian)
 * Offset 8-15: Timestamp (64-bit big-endian nanoseconds)
 * Offset 16: Encoding
 * Offset 17: Flags
 * Offset 18-19: Reserved (0)
 * Offset 20-23: Payload length (32-bit big-endian)
 */
export function createMessage(
  messageType: MessageType,
  channelId: number,
  payload: Uint8Array,
  encoding: Encoding = Encoding.CBOR,
  timestampNs: bigint = BigInt(0),
  flags: number = Flags.NONE
): ArrayBuffer {
  const buffer = new ArrayBuffer(HEADER_SIZE + payload.length)
  const view = new DataView(buffer)
  const bytes = new Uint8Array(buffer)

  // Magic bytes "RK"
  bytes[0] = MAGIC[0]
  bytes[1] = MAGIC[1]

  // Version
  view.setUint8(2, PROTOCOL_VERSION)

  // Message type
  view.setUint8(3, messageType)

  // Channel ID (32-bit big-endian)
  view.setUint32(4, channelId, false)

  // Timestamp (64-bit big-endian nanoseconds)
  view.setBigUint64(8, timestampNs, false)

  // Encoding
  view.setUint8(16, encoding)

  // Flags
  view.setUint8(17, flags)

  // Reserved (2 bytes)
  view.setUint16(18, 0, false)

  // Payload length (32-bit big-endian)
  view.setUint32(20, payload.length, false)

  // Payload
  bytes.set(payload, HEADER_SIZE)

  return buffer
}

/**
 * Parse a binary message frame
 * Returns [header, payload] tuple or null if invalid
 */
export function parseMessage(data: ArrayBuffer | Uint8Array): [MessageHeader, Uint8Array] | [null, Uint8Array] {
  const buffer = data instanceof Uint8Array ? data.buffer : data
  const dataView = data instanceof Uint8Array ? data : new Uint8Array(data)

  if (dataView.byteLength < HEADER_SIZE) {
    return [null, new Uint8Array(0)]
  }

  const view = new DataView(buffer, data instanceof Uint8Array ? data.byteOffset : 0)

  // Validate magic bytes
  if (dataView[0] !== MAGIC[0] || dataView[1] !== MAGIC[1]) {
    console.error('Invalid magic bytes')
    return [null, new Uint8Array(0)]
  }

  // Validate version
  const version = view.getUint8(2)
  if (version !== PROTOCOL_VERSION) {
    console.error(`Unsupported protocol version: ${version}`)
    return [null, new Uint8Array(0)]
  }

  const header: MessageHeader = {
    messageType: view.getUint8(3) as MessageType,
    channelId: view.getUint32(4, false), // 32-bit channel ID
    timestampNs: view.getBigUint64(8, false),
    encoding: view.getUint8(16) as Encoding,
    flags: view.getUint8(17),
    payloadLength: view.getUint32(20, false),
  }

  // Validate payload length
  if (dataView.byteLength < HEADER_SIZE + header.payloadLength) {
    console.error('Incomplete message payload')
    return [null, new Uint8Array(0)]
  }

  const payloadStart = data instanceof Uint8Array ? data.byteOffset + HEADER_SIZE : HEADER_SIZE
  const payload = new Uint8Array(buffer, payloadStart, header.payloadLength)

  return [header, payload]
}

/**
 * Parse a binary message frame (legacy interface)
 * @deprecated Use parseMessage which returns a tuple
 */
export function parseMessageLegacy(data: ArrayBuffer): Message | null {
  const [header, payload] = parseMessage(data)
  if (header === null) {
    return null
  }
  return { header, payload }
}

// Common ROS message type interfaces
export interface Header {
  stamp: { sec: number; nanosec: number }
  frame_id: string
}

export interface Point {
  x: number
  y: number
  z: number
}

export interface Quaternion {
  x: number
  y: number
  z: number
  w: number
}

export interface Pose {
  position: Point
  orientation: Quaternion
}

export interface PoseStamped {
  header: Header
  pose: Pose
}

export interface MapMetaData {
  map_load_time: { sec: number; nanosec: number }
  resolution: number
  width: number
  height: number
  origin: Pose
}

export interface OccupancyGrid {
  header: Header
  info: MapMetaData
  data: number[] | Uint8Array
}

export interface LaserScan {
  header: Header
  angle_min: number
  angle_max: number
  angle_increment: number
  time_increment: number
  scan_time: number
  range_min: number
  range_max: number
  ranges: number[] | Float32Array
  intensities: number[] | Float32Array
}

export interface Path {
  header: Header
  poses: PoseStamped[]
}

export interface Twist {
  linear: Point
  angular: Point
}

export interface TwistStamped {
  header: Header
  twist: Twist
}

// Channel information from server
export interface ChannelInfo {
  channel_id: number
  topic: string
  msg_type: string
  encoding: string
}

// Server information
export interface ServerInfo {
  name: string
  version: string
  protocol_version?: number
  ros_distro?: string
  capabilities?: string[]
  encodings?: string[]
}

// Topic information
export interface TopicInfo {
  name: string
  msg_type: string
}

// Service information
export interface ServiceInfo {
  name: string
  service_type: string
}

// Subscribe request
export interface SubscribeRequest {
  topic: string
  msg_type?: string
  throttle_ms?: number
}

// Unsubscribe request
export interface UnsubscribeRequest {
  channel_id: number
}

// Service call request
export interface ServiceCallRequest {
  service: string
  service_type: string
  request: unknown
  request_id?: string
  timeout_ms?: number
}

// Service call response
export interface ServiceCallResponse {
  response?: unknown
  error?: string
  request_id?: string
}

// Topic list response
export interface TopicListResponse {
  topics: TopicInfo[]
}

// Service list response
export interface ServiceListResponse {
  services: ServiceInfo[]
}

// Node list response
export interface NodeListResponse {
  nodes: string[]
}
