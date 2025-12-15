/**
 * RosKit Client
 *
 * High-performance WebSocket client for ROS2 communication.
 */

import { encode as cborEncode } from 'cbor-x'
import {
  MessageType,
  Encoding,
  PROTOCOL_VERSION,
  type ServerInfo,
  type TopicInfo,
  type ChannelInfo,
  type SubscribeRequest,
  type UnsubscribeRequest,
} from './protocol'
import { decodePayload, decodeCbor, decodeLaserScanBinary, decodePayloadWithWorker } from './decoders'
import {
  RosKitError,
  type RosKitClientConfig,
  type RosKitClientEvents,
  type ConnectionState,
  type SubscriptionOptions,
  type Subscription,
  type MessageCallback,
  type ChannelState,
  type CallbackState,
  type RosKitErrorKind,
  type RosKitErrorContext,
  type RetryPolicy,
} from './types'
import { DEFAULT_PROTOCOL_ORDER, ProtocolManager } from './protocol-adapters'
import { DecodeWorkerManager, getDecodeWorkerManager } from './workers'

/** Default retry policy */
const DEFAULT_RETRY_POLICY: Required<RetryPolicy> = {
  maxRetries: 3,
  baseDelayMs: 1000,
  maxDelayMs: 10000,
  backoffMultiplier: 2,
  jitter: 0.1,
}

type EventCallback<K extends keyof RosKitClientEvents> = RosKitClientEvents[K]

export class RosKitClient {
  private config: Required<RosKitClientConfig>
  private ws: WebSocket | null = null
  private state: ConnectionState = 'disconnected'
  private reconnectAttempts = 0
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null
  private protocol: ProtocolManager
  private adapterAnnounced = false

  // Server state
  private serverInfo: ServerInfo | null = null
  private topics: TopicInfo[] = []

  // Channel management
  private channels: Map<number, ChannelState> = new Map()
  private topicToChannel: Map<string, number> = new Map()
  private pendingSubscriptions: Map<
    string,
    { resolve: (sub: Subscription) => void; reject: (err: Error) => void; options: SubscriptionOptions; callback?: MessageCallback; createdAt: number }
  > = new Map()
  private nextSubscriptionId = 1
  private savedSubscriptions: Map<string, { callbacks: Set<MessageCallback>; options: SubscriptionOptions; createdAt: number; lastAccessedAt: number }> = new Map()
  private pendingServiceCalls: Map<
    string,
    { resolve: (value: unknown) => void; reject: (err: Error) => void; timeout: ReturnType<typeof setTimeout>; createdAt: number }
  > = new Map()
  private evictionTimer: ReturnType<typeof setInterval> | null = null

  // Event handlers
  private eventHandlers: Map<keyof RosKitClientEvents, Set<EventCallback<keyof RosKitClientEvents>>> = new Map()

  // Worker manager for heavy decoding
  private workerManager: DecodeWorkerManager | null = null
  private retryPolicy: Required<RetryPolicy>

  constructor(config: RosKitClientConfig) {
    this.config = {
      url: config.url,
      autoReconnect: config.autoReconnect ?? true,
      reconnectDelay: config.reconnectDelay ?? 1000,
      maxReconnectAttempts: config.maxReconnectAttempts ?? 0,
      maxReconnectDelay: config.maxReconnectDelay ?? 30000,
      connectionTimeout: config.connectionTimeout ?? 5000,
      subscriptionTimeoutMs: config.subscriptionTimeoutMs ?? 10000,
      maxSavedSubscriptions: config.maxSavedSubscriptions ?? 1000,
      savedSubscriptionMaxAgeMs: config.savedSubscriptionMaxAgeMs ?? 300000, // 5 minutes
      evictionIntervalMs: config.evictionIntervalMs ?? 60000, // 1 minute
      pendingServiceMaxAgeMs: config.pendingServiceMaxAgeMs ?? 120000, // 2 minutes
      protocolPreference: config.protocolPreference ?? DEFAULT_PROTOCOL_ORDER,
      minProtocolVersion: config.minProtocolVersion ?? 0,
      requiredCapabilities: config.requiredCapabilities ?? [],
      useWorkerDecode: config.useWorkerDecode ?? false,
      workerDecodeThreshold: config.workerDecodeThreshold ?? 10000,
      retryPolicy: config.retryPolicy ?? {},
    }
    this.protocol = new ProtocolManager(this.config.protocolPreference)
    this.retryPolicy = { ...DEFAULT_RETRY_POLICY, ...config.retryPolicy }

    // Initialize worker manager if enabled
    if (this.config.useWorkerDecode) {
      this.workerManager = getDecodeWorkerManager()
    }

    // Start eviction timer
    this.startEvictionTimer()
  }

  /**
   * Start periodic eviction of stale subscriptions
   */
  private startEvictionTimer(): void {
    if (this.evictionTimer) return
    this.evictionTimer = setInterval(() => {
      this.runEviction()
    }, this.config.evictionIntervalMs)
  }

  /**
   * Stop eviction timer
   */
  private stopEvictionTimer(): void {
    if (this.evictionTimer) {
      clearInterval(this.evictionTimer)
      this.evictionTimer = null
    }
  }

  /**
   * Run eviction of stale saved subscriptions and pending subscriptions
   */
  private runEviction(): void {
    const now = Date.now()
    const maxAge = this.config.savedSubscriptionMaxAgeMs

    // Evict stale saved subscriptions (those not accessed for maxAge)
    for (const [topic, record] of this.savedSubscriptions) {
      // Don't evict if there's an active channel for this topic
      if (this.topicToChannel.has(topic)) {
        // Update lastAccessedAt for active subscriptions
        record.lastAccessedAt = now
        continue
      }
      // Evict if too old and no active callbacks
      if (now - record.lastAccessedAt > maxAge) {
        this.savedSubscriptions.delete(topic)
      }
    }

    // Evict stale pending subscriptions (subscriptions that never completed)
    // Use subscription timeout as max age for pending subs
    const pendingMaxAge = this.config.subscriptionTimeoutMs * 2
    for (const [topic, pending] of this.pendingSubscriptions) {
      if (now - pending.createdAt > pendingMaxAge) {
        pending.reject(this.buildError('timeout', `Subscription evicted: ${topic}`, { context: { topic } }))
        this.pendingSubscriptions.delete(topic)
      }
    }

    // Evict stale pending service calls (guardrail in case timeout or disconnect doesn't trigger)
    const pendingServiceMaxAge = this.config.pendingServiceMaxAgeMs
    if (pendingServiceMaxAge > 0) {
      for (const [id, call] of Array.from(this.pendingServiceCalls.entries())) {
        if (now - call.createdAt > pendingServiceMaxAge) {
          // Clear the existing timeout to prevent double-rejection
          clearTimeout(call.timeout)
          call.reject(this.buildError('timeout', `Service call evicted after ${pendingServiceMaxAge}ms`, {
            context: { requestId: id },
          }))
          this.pendingServiceCalls.delete(id)
        }
      }
    }
  }

  // ===== Connection Management =====

  /**
   * Connect to RosKit Bridge
   */
  async connect(): Promise<void> {
    if (this.state === 'connected' || this.state === 'connecting') {
      return
    }

    this.state = 'connecting'
    // Reset protocol detection for each connect attempt
    this.protocol = new ProtocolManager(this.config.protocolPreference)
    this.adapterAnnounced = false

    return new Promise((resolve, reject) => {
      const timeoutId = setTimeout(() => {
        if (this.state === 'connecting') {
          this.ws?.close()
          reject(new Error('Connection timeout'))
        }
      }, this.config.connectionTimeout)

      try {
        this.ws = new WebSocket(this.config.url)
        this.ws.binaryType = 'arraybuffer'

        this.ws.onopen = () => {
          clearTimeout(timeoutId)
          this.state = 'connected'
          this.reconnectAttempts = 0
          this.emit('connect')
          this.protocol.waitUntilReady().then(() => {
            this.restoreSubscriptions()
          })
          resolve()
        }

        this.ws.onclose = (event) => {
          clearTimeout(timeoutId)
          const wasConnected = this.state === 'connected'
          this.state = 'disconnected'

          // Clear pending service calls on disconnect
          this.expirePendingServiceCalls(event.reason || 'Connection closed')

          // Clear channel state
          this.channels.clear()
          this.topicToChannel.clear()

          this.emit('disconnect', event.reason || 'Connection closed')

          if (wasConnected && this.config.autoReconnect) {
            this.scheduleReconnect()
          }
        }

        this.ws.onerror = (ev) => {
          clearTimeout(timeoutId)
          this.state = 'error'
          const error = this.buildError('network', 'WebSocket error', { cause: ev })
          this.emit('error', error)
          reject(error)
        }

        this.ws.onmessage = (event) => {
          this.handleMessage(event.data as ArrayBuffer)
        }
      } catch (err) {
        clearTimeout(timeoutId)
        this.state = 'error'
        reject(err)
      }
    })
  }

  /**
   * Destroy client and clear all state
   */
  destroy(): void {
    this.stopEvictionTimer()
    this.disconnect()
    this.savedSubscriptions.clear()
    this.pendingSubscriptions.clear()
    this.channels.clear()
    this.topicToChannel.clear()
    this.eventHandlers.clear()
  }

  private buildError(kind: RosKitErrorKind, message: string, options?: { cause?: unknown; context?: RosKitErrorContext }): RosKitError {
    return new RosKitError(kind, message, options)
  }

  /**
   * Disconnect from server
   */
  disconnect(): void {
    this.config.autoReconnect = false
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }
    if (this.ws) {
      this.ws.close()
      this.ws = null
    }
    this.state = 'disconnected'
    this.channels.clear()
    this.topicToChannel.clear()
    this.savedSubscriptions.clear()
    this.pendingSubscriptions.clear()

    // Expire all pending service calls
    this.expirePendingServiceCalls('Client disconnected')
  }

  /**
   * Expire all pending service calls with an error
   */
  private expirePendingServiceCalls(reason: string): void {
    for (const [requestId, entry] of this.pendingServiceCalls) {
      clearTimeout(entry.timeout)
      entry.reject(this.buildError('network', reason, { context: { requestId } }))
    }
    this.pendingServiceCalls.clear()
  }

  /**
   * Get current connection state
   */
  getState(): ConnectionState {
    return this.state
  }

  /**
   * Check if connected
   */
  isConnected(): boolean {
    return this.state === 'connected'
  }

  private scheduleReconnect(): void {
    if (this.config.maxReconnectAttempts > 0 && this.reconnectAttempts >= this.config.maxReconnectAttempts) {
      this.emit('error', this.buildError('network', `Max reconnect attempts (${this.config.maxReconnectAttempts}) reached`))
      return
    }

    this.reconnectAttempts++
    const exponent = Math.min(this.reconnectAttempts, 6)
    const baseDelay = this.config.reconnectDelay * Math.max(1, 2 ** (exponent - 1))
    const jitter = baseDelay * 0.3 * Math.random()
    const delay = Math.min(baseDelay + jitter, this.config.maxReconnectDelay)

    // Emit reconnecting event before scheduling
    this.emit('reconnecting', this.reconnectAttempts, delay)

    this.reconnectTimer = setTimeout(() => {
      this.connect().catch(() => {
        // Error handled via events
      })
    }, delay)
  }

  // ===== Message Handling =====

  private handleMessage(data: ArrayBuffer): void {
    const message = this.protocol.detectAndParse(data)
    if (!message) {
      console.warn('Unable to parse incoming frame')
      return
    }

    if (!this.adapterAnnounced && this.protocol.adapterName) {
      this.adapterAnnounced = true
      console.info(`RosKitClient detected protocol adapter: ${this.protocol.adapterName}`)
    }

    const { header, payload } = message

    switch (header.messageType) {
      case MessageType.SERVER_INFO:
        this.handleServerInfo(payload)
        break
      case MessageType.TOPIC_LIST_RESPONSE:
        this.handleTopicList(payload)
        break
      case MessageType.SERVICE_LIST_RESPONSE:
        this.handleServiceList(payload)
        break
      case MessageType.NODE_LIST_RESPONSE:
        this.handleNodeList(payload)
        break
      case MessageType.CHANNEL_INFO:
        this.handleChannelInfo(payload)
        break
      case MessageType.MESSAGE:
        this.handleDataMessage(header.channelId, payload, header.encoding, header.timestampNs)
        break
      case MessageType.SERVICE_RESPONSE:
        this.handleServiceResponse(payload)
        break
      case MessageType.PONG:
        // Pong received
        break
      case MessageType.ERROR:
        this.handleError(payload)
        break
      default:
        console.warn(`Unhandled message type: ${header.messageType}`)
    }
  }

  private handleServerInfo(payload: Uint8Array): void {
    this.serverInfo = decodeCbor<ServerInfo>(payload)

    // Validate protocol version
    const serverVersion = this.serverInfo.protocol_version ?? 0
    if (this.config.minProtocolVersion > 0 && serverVersion < this.config.minProtocolVersion) {
      const error = this.buildError(
        'protocol',
        `Server protocol version ${serverVersion} is below minimum required ${this.config.minProtocolVersion}`
      )
      this.emit('error', error)
      this.disconnect()
      return
    }
    if (serverVersion !== PROTOCOL_VERSION) {
      const error = this.buildError(
        'protocol',
        `Protocol version mismatch (server ${serverVersion}, client ${PROTOCOL_VERSION})`
      )
      this.emit('error', error)
      this.disconnect()
      return
    }

    // Validate required capabilities
    const serverCapabilities = this.serverInfo.capabilities ?? []
    const missingCapabilities = this.config.requiredCapabilities.filter(
      (cap) => !serverCapabilities.includes(cap)
    )
    if (missingCapabilities.length > 0) {
      const error = this.buildError('protocol', `Server missing required capabilities: ${missingCapabilities.join(', ')}`)
      this.emit('error', error)
      this.disconnect()
      return
    }

    this.emit('serverInfo', this.serverInfo)
  }

  private handleTopicList(payload: Uint8Array): void {
    const data = decodeCbor<{ topics?: unknown }>(payload)
    const normalizeTopic = (t: any): TopicInfo => {
      // Handle both formats: { msg_type: string } and { types: string[] }
      const msgType = t?.msg_type ?? (Array.isArray(t?.types) ? t.types[0] : '') ?? ''
      return {
        name: (t?.name as string) ?? (t?.topic as string) ?? '',
        msg_type: msgType as string,
      }
    }

    if (Array.isArray((data as any).topics)) {
      this.topics = (data as any).topics.map(normalizeTopic)
    } else {
      this.topics = []
    }

    this.emit('topicList', this.topics)
  }

  private handleServiceList(payload: Uint8Array): void {
    try {
      const data = decodeCbor<{ services?: Array<{ name: string; types?: string[] }> }>(payload)
      const services = Array.isArray(data.services)
        ? data.services.map((s) => ({
            name: (s as any).name ?? '',
            types: Array.isArray((s as any).types) ? ((s as any).types as string[]) : [],
          }))
        : []
      this.emit('serviceList', services)
    } catch (err) {
      this.emit('error', this.buildError('decode', 'Failed to decode service list', { cause: err }))
    }
  }

  private handleNodeList(payload: Uint8Array): void {
    try {
      const data = decodeCbor<{ nodes?: string[] }>(payload)
      const nodes = Array.isArray(data.nodes) ? data.nodes : []
      this.emit('nodeList', nodes)
    } catch (err) {
      this.emit('error', this.buildError('decode', 'Failed to decode node list', { cause: err }))
    }
  }

  private handleChannelInfo(payload: Uint8Array): void {
    const info = decodeCbor<ChannelInfo>(payload)

    // Check for pending subscription
    const pending = this.pendingSubscriptions.get(info.topic)
    if (pending) {
      this.pendingSubscriptions.delete(info.topic)

      // Determine if we should use worker decoding for this channel
      const useWorker = pending.options.useWorkerDecode ?? this.config.useWorkerDecode

      // Create channel state
      const channelState: ChannelState = {
        id: info.channel_id,
        topic: info.topic,
        msgType: info.msg_type,
        encoding: info.encoding,
        callbacks: new Set(),
        callbackStates: new Map(),
        useWorkerDecode: useWorker,
      }

      // Re-attach saved callbacks if we have them
      const saved = this.savedSubscriptions.get(info.topic)
      if (saved) {
        for (const cb of saved.callbacks) {
          channelState.callbacks.add(cb as MessageCallback)
          // Initialize callback state with saved options
          channelState.callbackStates.set(cb as MessageCallback, {
            callback: cb as MessageCallback,
            options: saved.options,
            queue: [],
            lastDeliveredTs: 0,
            processing: false,
          })
        }
      }

      this.channels.set(info.channel_id, channelState)
      this.topicToChannel.set(info.topic, info.channel_id)

      // Create subscription handle
      const subscriptionId = `sub_${this.nextSubscriptionId++}`
      const unsubscribe = pending?.callback
        ? () => this.unsubscribeTopic(info.topic, pending.callback)
        : () => this.unsubscribeTopic(info.topic)
      const subscription: Subscription = {
        id: subscriptionId,
        channelId: info.channel_id,
        topic: info.topic,
        msgType: info.msg_type,
        unsubscribe,
      }

      pending.resolve(subscription)
    }

    this.emit('channelInfo', info)
  }

  private async handleDataMessage(
    channelId: number,
    payload: Uint8Array,
    encoding: Encoding,
    timestamp: bigint
  ): Promise<void> {
    const channel = this.channels.get(channelId)
    if (!channel) {
      return
    }

    try {
      let decoded: unknown

      // Check if we should use worker for decoding heavy payloads
      // Only offload to worker if payload exceeds threshold (default 10KB)
      const payloadSize = payload.byteLength
      const meetsThreshold = payloadSize >= this.config.workerDecodeThreshold
      const isHeavyEncoding = encoding === Encoding.PNG || encoding === Encoding.JPEG || encoding === Encoding.BINARY
      const shouldUseWorker = channel.useWorkerDecode && this.workerManager?.isReady() && isHeavyEncoding && meetsThreshold

      if (shouldUseWorker && this.workerManager) {
        // Use worker for heavy decoding
        decoded = await decodePayloadWithWorker(payload, encoding, this.workerManager)
      } else if (encoding === Encoding.BINARY && channel.msgType.includes('LaserScan')) {
        // Special handling for known binary formats
        decoded = decodeLaserScanBinary(payload)
      } else {
        decoded = await decodePayload(payload, encoding)
      }

      // Deliver to each callback with per-callback throttling and queueing
      for (const callback of channel.callbacks) {
        const cbState = channel.callbackStates.get(callback)
        if (cbState) {
          this.deliverToCallback(channel, cbState, decoded, timestamp)
        } else {
          // Fallback for callbacks without state
          try {
            callback(decoded, timestamp)
          } catch (err) {
            console.error('Error in message callback:', err)
          }
        }
      }

      // Emit global message event
      this.emit('message', channelId, decoded, timestamp)
    } catch (err) {
      console.error('Error decoding message:', err)
      this.emit('error', this.buildError('decode', 'Failed to decode incoming message', {
        cause: err,
        context: { channelId, topic: channel.topic },
      }))
    }
  }

  /**
   * Deliver message to a callback with throttling and queue management
   */
  private deliverToCallback(
    channel: ChannelState,
    cbState: CallbackState,
    message: unknown,
    timestamp: bigint
  ): void {
    const now = Date.now()
    const throttleMs = cbState.options.clientThrottleMs ?? 0
    const queueSize = cbState.options.queueSize ?? 1
    const dropPolicy = cbState.options.dropPolicy ?? 'newest'

    // Check throttle
    if (throttleMs > 0 && now - cbState.lastDeliveredTs < throttleMs) {
      // Throttled - add to queue if queueSize > 1
      if (queueSize > 1) {
        if (cbState.queue.length >= queueSize) {
          // Queue full - apply drop policy
          if (dropPolicy === 'oldest') {
            cbState.queue.shift() // Remove oldest
            cbState.queue.push({ message, timestamp })
          }
          // 'newest' policy: drop incoming message (don't add)
        } else {
          cbState.queue.push({ message, timestamp })
        }
      }
      return
    }

    // Deliver immediately
    cbState.lastDeliveredTs = now
    try {
      cbState.callback(message, timestamp)
    } catch (err) {
      this.emit('error', this.buildError('unknown', 'Error in message callback', {
        cause: err,
        context: { topic: channel.topic, channelId: channel.id },
      }))
    }

    // Process queued messages
    this.processCallbackQueue(channel, cbState)
  }

  /**
   * Process queued messages for a callback
   */
  private processCallbackQueue(channel: ChannelState, cbState: CallbackState): void {
    if (cbState.processing || cbState.queue.length === 0) return

    const throttleMs = cbState.options.clientThrottleMs ?? 0
    if (throttleMs > 0) {
      cbState.processing = true
      setTimeout(() => {
        cbState.processing = false
        const queued = cbState.queue.shift()
        if (queued) {
          cbState.lastDeliveredTs = Date.now()
          try {
            cbState.callback(queued.message, queued.timestamp)
          } catch (err) {
            this.emit('error', this.buildError('unknown', 'Error in queued message callback', {
              cause: err,
              context: { topic: channel.topic, channelId: channel.id },
            }))
          }
          // Recursively process remaining queue
          this.processCallbackQueue(channel, cbState)
        }
      }, throttleMs)
    }
  }

  private handleError(payload: Uint8Array): void {
    const error = decodeCbor<{ error: string }>(payload)
    console.error('Server error:', error.error)
    this.emit('error', this.buildError('server', error.error, {}))
  }

  private handleServiceResponse(payload: Uint8Array): void {
    try {
      const data = decodeCbor<{ request_id?: string; response?: unknown; error?: string; service?: string }>(payload)
      const id = data.request_id
      if (!id) {
        this.emit('error', this.buildError('protocol', 'Service response missing request_id'))
        return
      }
      const entry = this.pendingServiceCalls.get(id)
      if (!entry) {
        // Unknown/expired request, drop
        return
      }
      clearTimeout(entry.timeout)
      this.pendingServiceCalls.delete(id)
      if (data.error) {
        // Enrich error with context from response
        entry.reject(this.buildError('server', data.error, {
          context: { requestId: id, service: data.service },
        }))
      } else {
        entry.resolve(data.response)
      }
    } catch (err) {
      this.emit('error', this.buildError('decode', 'Failed to decode service response', { cause: err }))
    }
  }

  // ===== Public API =====
  /**
   * Remove saved subscription state (used by hooks during cleanup)
   */
  clearSavedSubscription(topic: string, callback?: MessageCallback): void {
    const record = this.savedSubscriptions.get(topic)
    if (!record) return
    if (callback) {
      record.callbacks.delete(callback)
    } else {
      record.callbacks.clear()
    }
    if (record.callbacks.size === 0) {
      this.savedSubscriptions.delete(topic)
    }
  }

  private ensureSavedCapacity(): void {
    if (this.savedSubscriptions.size < this.config.maxSavedSubscriptions) return
    // Evict oldest entry
    const oldest = this.savedSubscriptions.keys().next()
    if (!oldest.done) {
      this.savedSubscriptions.delete(oldest.value)
    }
  }

  /**
   * Subscribe to a ROS topic
   */
  async subscribe<T = unknown>(
    topic: string,
    callback: MessageCallback<T>,
    options: SubscriptionOptions = {}
  ): Promise<Subscription> {
    if (!this.isConnected()) {
      throw new Error('Not connected')
    }

    // Track desired subscription for reconnects
    const now = Date.now()
    const existing = this.savedSubscriptions.get(topic)
    const saved = existing ?? { callbacks: new Set<MessageCallback>(), options, createdAt: now, lastAccessedAt: now }
    saved.callbacks.add(callback as MessageCallback)
    saved.options = { ...saved.options, ...options }
    saved.lastAccessedAt = now
    this.ensureSavedCapacity()
    this.savedSubscriptions.set(topic, saved)

    // Check if already subscribed
    const existingChannelId = this.topicToChannel.get(topic)
    if (existingChannelId !== undefined) {
      const channel = this.channels.get(existingChannelId)!
      channel.callbacks.add(callback as MessageCallback)

      return {
        id: `sub_${this.nextSubscriptionId++}`,
        channelId: existingChannelId,
        topic,
        msgType: channel.msgType,
        unsubscribe: () => this.unsubscribeTopic(topic, callback as MessageCallback),
      }
    }

    // Create subscription request
    const request: SubscribeRequest = {
      topic,
      msg_type: options.msgType,
      throttle_ms: options.throttleMs,
    }

    const payload = new Uint8Array(cborEncode(request))
    await this.protocol.waitUntilReady()
    const message = this.protocol.createMessage(MessageType.SUBSCRIBE, 0, payload, Encoding.CBOR)

    // Wait for channel info response
    return new Promise((resolve, reject) => {
      this.pendingSubscriptions.set(topic, { resolve, reject, options, callback: callback as MessageCallback, createdAt: Date.now() })

      this.ws!.send(message)

      // Add callback after subscription is confirmed
      const originalResolve = resolve
      this.pendingSubscriptions.get(topic)!.resolve = (sub: Subscription) => {
        const channel = this.channels.get(sub.channelId)!
        channel.callbacks.add(callback as MessageCallback)
        originalResolve(sub)
      }

      // Timeout
      setTimeout(() => {
        if (this.pendingSubscriptions.has(topic)) {
          this.pendingSubscriptions.delete(topic)
          reject(new Error(`Subscription timeout for ${topic}`))
        }
      }, this.config.subscriptionTimeoutMs)
    })
  }

  private unsubscribeTopic(topic: string, callback?: MessageCallback): void {
    const record = this.savedSubscriptions.get(topic)
    if (record) {
      if (callback) {
        record.callbacks.delete(callback)
      }
      if (record.callbacks.size === 0) {
        this.savedSubscriptions.delete(topic)
      }
    }

    const channelId = this.topicToChannel.get(topic)
    const channel = channelId !== undefined ? this.channels.get(channelId) : undefined
    if (!channelId || !channel) {
      return
    }

    if (callback) {
      channel.callbacks.delete(callback)
    }

    if (channel.callbacks.size > 0) {
      return
    }

    const request: UnsubscribeRequest = { channel_id: channelId }
    const payload = new Uint8Array(cborEncode(request))
    const message = this.protocol.createMessage(MessageType.UNSUBSCRIBE, channelId, payload, Encoding.CBOR)

    this.ws?.send(message)

    this.channels.delete(channelId)
    this.topicToChannel.delete(topic)
  }

  /**
   * Request list of available topics
   */
  async getTopics(): Promise<TopicInfo[]> {
    if (!this.isConnected()) {
      throw new Error('Not connected')
    }

    await this.protocol.waitUntilReady()
    const message = this.protocol.createMessage(MessageType.TOPIC_LIST, 0, new Uint8Array(0), Encoding.RAW)
    this.ws!.send(message)

    // Return cached topics, will be updated via event
    return new Promise((resolve) => {
      const handler = (topics: TopicInfo[]) => {
        this.off('topicList', handler)
        resolve(topics)
      }
      this.on('topicList', handler)
    })
  }

  /**
   * Get server information
   */
  getServerInfo(): ServerInfo | null {
    return this.serverInfo
  }

  /**
   * Get the protocol version supported by the server
   */
  getProtocolVersion(): number {
    return this.serverInfo?.protocol_version ?? 0
  }

  /**
   * Check if the server supports a specific capability
   */
  hasCapability(capability: string): boolean {
    return this.serverInfo?.capabilities?.includes(capability) ?? false
  }

  /**
   * Check if the server supports a specific encoding
   */
  supportsEncoding(encoding: string): boolean {
    // If encodings not provided, assume basic encodings are supported
    if (!this.serverInfo?.encodings || this.serverInfo.encodings.length === 0) {
      return ['raw', 'cbor', 'json'].includes(encoding)
    }
    return this.serverInfo.encodings.includes(encoding)
  }

  /**
   * Get all capabilities supported by the server
   */
  getCapabilities(): string[] {
    return this.serverInfo?.capabilities ?? []
  }

  /**
   * Get all encodings supported by the server
   */
  getSupportedEncodings(): string[] {
    return this.serverInfo?.encodings ?? ['raw', 'cbor', 'json']
  }

  /**
   * Request list of available services
   */
  async getServices(): Promise<{ name: string; types: string[] }[]> {
    if (!this.isConnected()) {
      throw new Error('Not connected')
    }

    await this.protocol.waitUntilReady()
    const message = this.protocol.createMessage(MessageType.SERVICE_LIST, 0, new Uint8Array(0), Encoding.RAW)
    this.ws!.send(message)

    return new Promise((resolve) => {
      const handler = (services: { name: string; types: string[] }[]) => {
        this.off('serviceList', handler)
        resolve(services)
      }
      this.on('serviceList', handler)
    })
  }

  /**
   * Request list of available nodes
   */
  async getNodes(): Promise<string[]> {
    if (!this.isConnected()) {
      throw new Error('Not connected')
    }

    await this.protocol.waitUntilReady()
    const message = this.protocol.createMessage(MessageType.NODE_LIST, 0, new Uint8Array(0), Encoding.RAW)
    this.ws!.send(message)

    return new Promise((resolve) => {
      const handler = (nodes: string[]) => {
        this.off('nodeList', handler)
        resolve(nodes)
      }
      this.on('nodeList', handler)
    })
  }

  /**
   * Re-subscribe to saved topics after a reconnect
   */
  private restoreSubscriptions(): void {
    if (!this.isConnected()) {
      return
    }

    for (const [topic, record] of this.savedSubscriptions.entries()) {
      if (record.callbacks.size === 0) {
        continue
      }

      const existingChannelId = this.topicToChannel.get(topic)
      if (existingChannelId !== undefined) {
        const channel = this.channels.get(existingChannelId)
        if (channel) {
          for (const cb of record.callbacks) {
            channel.callbacks.add(cb as MessageCallback)
          }
        }
        continue
      }

      const [primaryCallback] = Array.from(record.callbacks)
      if (!primaryCallback) continue

      this.subscribe(topic, primaryCallback as MessageCallback, record.options).catch((err) => {
        console.error(`Failed to restore subscription for ${topic}:`, err)
      })
    }
  }

  /**
   * Send ping to server
   */
  ping(): void {
    if (!this.isConnected()) {
      return
    }

    const message = this.protocol.createMessage(MessageType.PING, 0, new Uint8Array(0), Encoding.RAW)
    this.ws!.send(message)
  }

  /**
   * Publish a ROS message
   */
  publish<T extends Record<string, unknown>>(topic: string, msgType: string, data: T): void {
    if (!this.isConnected()) {
      throw this.buildError('network', 'Not connected')
    }
    const payload = new Uint8Array(cborEncode({ topic, msg_type: msgType, data }))
    const message = this.protocol.createMessage(MessageType.PUBLISH, 0, payload, Encoding.CBOR)
    this.ws!.send(message)
  }

  /**
   * Call a ROS service with correlation and optional retries
   */
  serviceCall<TRequest extends Record<string, unknown>, TResponse = unknown>(
    service: string,
    msgType: string,
    request: TRequest,
    options: { timeoutMs?: number; retries?: number } = {}
  ): Promise<TResponse> {
    const maxRetries = options.retries ?? this.retryPolicy.maxRetries
    return this.serviceCallWithRetry(service, msgType, request, {
      ...options,
      retries: maxRetries,
    }, 0)
  }

  /**
   * Internal service call with retry logic
   */
  private async serviceCallWithRetry<TRequest extends Record<string, unknown>, TResponse = unknown>(
    service: string,
    msgType: string,
    request: TRequest,
    options: { timeoutMs?: number; retries?: number },
    attempt: number
  ): Promise<TResponse> {
    if (!this.isConnected()) {
      throw this.buildError('network', 'Not connected', { context: { service } })
    }

    const requestId = `svc_${Date.now()}_${Math.random().toString(16).slice(2)}`
    const timeoutMs = options.timeoutMs ?? 10000
    const maxRetries = options.retries ?? 0
    const payload = new Uint8Array(cborEncode({ service, msg_type: msgType, request, request_id: requestId }))
    const message = this.protocol.createMessage(MessageType.SERVICE_CALL, 0, payload, Encoding.CBOR)

    try {
      return await new Promise<TResponse>((resolve, reject) => {
        const timeout = setTimeout(() => {
          this.pendingServiceCalls.delete(requestId)
          reject(this.buildError('timeout', `Service call timed out: ${service}`, {
            context: { service, requestId },
          }))
        }, timeoutMs)

        this.pendingServiceCalls.set(requestId, { resolve: resolve as (v: unknown) => void, reject, timeout, createdAt: Date.now() })
        this.ws!.send(message)
      })
    } catch (err) {
      const error = err instanceof RosKitError ? err : this.buildError('unknown', String(err), { cause: err })

      // Check if we should retry
      if (attempt < maxRetries && this.shouldRetry(error)) {
        const delay = this.calculateRetryDelay(attempt)
        await this.sleep(delay)
        return this.serviceCallWithRetry(service, msgType, request, options, attempt + 1)
      }

      throw error.withContext({ service, requestId })
    }
  }

  /**
   * Determine if an error is retryable
   */
  private shouldRetry(error: RosKitError): boolean {
    // Retry on timeout or network errors, not on server/protocol errors
    return error.kind === 'timeout' || error.kind === 'network'
  }

  /**
   * Calculate delay before next retry with exponential backoff and jitter
   */
  private calculateRetryDelay(attempt: number): number {
    const { baseDelayMs, maxDelayMs, backoffMultiplier, jitter } = this.retryPolicy
    const exponentialDelay = baseDelayMs * Math.pow(backoffMultiplier, attempt)
    const clampedDelay = Math.min(exponentialDelay, maxDelayMs)
    const jitterAmount = clampedDelay * jitter * (Math.random() * 2 - 1)
    return Math.max(0, clampedDelay + jitterAmount)
  }

  /**
   * Sleep helper
   */
  private sleep(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms))
  }

  // ===== Event Emitter =====

  /**
   * Register event handler
   */
  on<K extends keyof RosKitClientEvents>(event: K, callback: RosKitClientEvents[K]): void {
    if (!this.eventHandlers.has(event)) {
      this.eventHandlers.set(event, new Set())
    }
    this.eventHandlers.get(event)!.add(callback as EventCallback<keyof RosKitClientEvents>)
  }

  /**
   * Remove event handler
   */
  off<K extends keyof RosKitClientEvents>(event: K, callback: RosKitClientEvents[K]): void {
    this.eventHandlers.get(event)?.delete(callback as EventCallback<keyof RosKitClientEvents>)
  }

  /**
   * Emit event
   */
  private emit<K extends keyof RosKitClientEvents>(
    event: K,
    ...args: Parameters<RosKitClientEvents[K]>
  ): void {
    const handlers = this.eventHandlers.get(event)
    if (handlers) {
      for (const handler of handlers) {
        try {
          ;(handler as (...args: Parameters<RosKitClientEvents[K]>) => void)(...args)
        } catch (err) {
          console.error(`Error in event handler for ${event}:`, err)
        }
      }
    }
  }
}

// Legacy export for backward compatibility
export { RosKitClient as RosWebClient }
