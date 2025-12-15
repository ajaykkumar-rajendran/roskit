/**
 * @roskit/client
 *
 * High-performance TypeScript client SDK for RosKit Bridge.
 */

// Main client
export { RosKitClient, RosWebClient } from './client'

// Protocol types and utilities
export {
  MessageType,
  Encoding,
  Flags,
  MAGIC,
  PROTOCOL_VERSION,
  HEADER_SIZE,
  createMessage,
  parseMessage,
  type MessageHeader,
  type Message,
  type Header,
  type Point,
  type Quaternion,
  type Pose,
  type PoseStamped,
  type MapMetaData,
  type OccupancyGrid,
  type LaserScan,
  type Path,
  type Twist,
  type TwistStamped,
  type ChannelInfo,
  type ServerInfo,
  type TopicInfo,
  type SubscribeRequest,
  type UnsubscribeRequest,
} from './protocol'

// Type exports
export type {
  ConnectionState,
  RosKitClientConfig,
  RosKitClientEvents,
  RosWebClientConfig,
  RosWebClientEvents,
  SubscriptionOptions,
  Subscription,
  MessageCallback,
  PublisherOptions,
  Publisher,
  ServiceCallOptions,
  ChannelState,
} from './types'

// Decoder utilities
export {
  decodeCbor,
  decodePng,
  decodeJpeg,
  decodeBinaryFloat32,
  decodeBinaryFloat64,
  decodeBinaryInt32,
  decodeLaserScanBinary,
  decodePayload,
  decodePayloadWithWorker,
} from './decoders'

// Web Worker utilities for offloading heavy decoding
export {
  DecodeWorkerManager,
  getDecodeWorkerManager,
  terminateDecodeWorkerManager,
} from './workers'
export type { DecodeWorkerOptions } from './workers'
