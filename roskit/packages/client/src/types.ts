/**
 * RosKit Client Types
 */

import type { ChannelInfo, ServerInfo, TopicInfo } from './protocol'

// Connection states
export type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error'

/** Retry policy configuration */
export interface RetryPolicy {
  /** Maximum number of retries (default: 3) */
  maxRetries?: number
  /** Base delay between retries in milliseconds (default: 1000) */
  baseDelayMs?: number
  /** Maximum delay between retries in milliseconds (default: 10000) */
  maxDelayMs?: number
  /** Exponential backoff multiplier (default: 2) */
  backoffMultiplier?: number
  /** Jitter factor 0-1 to randomize delays (default: 0.1) */
  jitter?: number
}

// Client configuration
export interface RosKitClientConfig {
  /** WebSocket URL (e.g., ws://localhost:9091) */
  url: string
  /** Auto-reconnect on disconnect */
  autoReconnect?: boolean
  /** Reconnect delay in milliseconds */
  reconnectDelay?: number
  /** Maximum reconnect attempts (0 = unlimited) */
  maxReconnectAttempts?: number
  /** Maximum delay between reconnect attempts (ms) */
  maxReconnectDelay?: number
  /** Connection timeout in milliseconds */
  connectionTimeout?: number
  /** Subscription handshake timeout in milliseconds */
  subscriptionTimeoutMs?: number
  /** Maximum saved subscriptions tracked for reconnect (evicts oldest) */
  maxSavedSubscriptions?: number
  /** Maximum age for saved subscriptions before eviction (ms) */
  savedSubscriptionMaxAgeMs?: number
  /** Interval for eviction sweeps (ms) */
  evictionIntervalMs?: number
  /** Maximum age for pending service calls before forced cleanup on disconnect (ms) */
  pendingServiceMaxAgeMs?: number
  /** Preferred protocol adapters order */
  protocolPreference?: Array<'rust' | 'python' | 'legacy'>
  /** Minimum required protocol version (disconnect if server doesn't support) */
  minProtocolVersion?: number
  /** Required capabilities (disconnect if server doesn't support) */
  requiredCapabilities?: string[]
  /** Enable worker-based decoding for heavy payloads (default: false) */
  useWorkerDecode?: boolean
  /** Minimum payload size in bytes to use worker decode (default: 10000) */
  workerDecodeThreshold?: number
  /** Retry policy for operations like service calls */
  retryPolicy?: RetryPolicy
}

// Event types
export interface RosKitClientEvents {
  connect: () => void
  disconnect: (reason?: string) => void
  reconnecting: (attempt: number, delay: number) => void
  error: (error: RosKitError) => void
  serverInfo: (info: ServerInfo) => void
  topicList: (topics: TopicInfo[]) => void
  serviceList: (services: { name: string; types: string[] }[]) => void
  nodeList: (nodes: string[]) => void
  channelInfo: (info: ChannelInfo) => void
  message: (channelId: number, data: unknown, timestamp: bigint) => void
}

/** Message drop policy when queue is full */
export type DropPolicy = 'newest' | 'oldest'

// Subscription options
export interface SubscriptionOptions {
  /** ROS message type (auto-detected if not provided) */
  msgType?: string
  /** Server-side throttle rate in milliseconds (requests bridge to throttle) */
  throttleMs?: number
  /** Client-side throttle (drop updates faster than this) in milliseconds */
  clientThrottleMs?: number
  /** Queue size for buffering messages per callback (default: 1, no queueing) */
  queueSize?: number
  /** Drop policy when queue is full (default: 'newest' - drop new messages) */
  dropPolicy?: DropPolicy
  /** Use web workers for decoding heavy payloads (PNG, JPEG, LaserScan) */
  useWorkerDecode?: boolean
  /** Max retries on subscription failure (default: 3) */
  maxRetries?: number
  /** Base delay for retries (ms, default: 300) */
  retryDelayMs?: number
}

// Subscription handle
export interface Subscription {
  /** Unique subscription ID */
  id: string
  /** Channel ID from server */
  channelId: number
  /** Topic name */
  topic: string
  /** Message type */
  msgType: string
  /** Unsubscribe from topic */
  unsubscribe: () => void
}

// Message callback type
export type MessageCallback<T = unknown> = (message: T, timestamp: bigint) => void
export type RosKitErrorKind = 'network' | 'decode' | 'server' | 'timeout' | 'protocol' | 'unknown'

export interface RosKitErrorContext {
  topic?: string
  channelId?: number
  service?: string
  requestId?: string
}

export class RosKitError extends Error {
  kind: RosKitErrorKind
  context?: RosKitErrorContext

  constructor(
    kind: RosKitErrorKind,
    message: string,
    options?: { cause?: unknown; context?: RosKitErrorContext }
  ) {
    super(message, options)
    this.kind = kind
    this.context = options?.context
  }

  /** Create a copy with additional context */
  withContext(ctx: RosKitErrorContext): RosKitError {
    const err = new RosKitError(this.kind, this.message, {
      cause: this.cause,
      context: { ...this.context, ...ctx },
    })
    err.stack = this.stack
    return err
  }
}

// Publisher options
export interface PublisherOptions {
  /** ROS message type */
  msgType: string
  /** Queue size */
  queueSize?: number
}

// Publisher handle
export interface Publisher<T = unknown> {
  /** Topic name */
  topic: string
  /** Message type */
  msgType: string
  /** Publish a message */
  publish: (message: T) => void
  /** Unadvertise */
  unadvertise: () => void
}

// Service call options
export interface ServiceCallOptions {
  /** Timeout in milliseconds */
  timeout?: number
}

/** Per-callback state for queue management */
export interface CallbackState {
  callback: MessageCallback
  options: SubscriptionOptions
  queue: Array<{ message: unknown; timestamp: bigint }>
  lastDeliveredTs: number
  processing: boolean
}

// Internal channel state
export interface ChannelState {
  id: number
  topic: string
  msgType: string
  encoding: string
  callbacks: Set<MessageCallback>
  callbackStates: Map<MessageCallback, CallbackState>
  lastDeliveredTs?: number
  /** Whether this channel uses worker decoding */
  useWorkerDecode: boolean
}

// Legacy type aliases for backward compatibility
export type RosWebClientConfig = RosKitClientConfig
export type RosWebClientEvents = RosKitClientEvents
