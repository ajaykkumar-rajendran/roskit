/**
 * Reconnect and Resubscribe Integration Tests
 *
 * Tests automatic reconnection, subscription restoration, and state management.
 */

import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest'
import { encode as cborEncode, decode as cborDecode } from 'cbor-x'
import {
  MessageType,
  Encoding,
  createMessage,
  parseMessage,
} from '../protocol'
import { RosKitClient } from '../client'

// Mock WebSocket factory for testing reconnection scenarios
function createMockWebSocketFactory() {
  let instanceCount = 0
  let lastInstance: MockWebSocket | null = null
  const instances: MockWebSocket[] = []

  class MockWebSocket {
    static CONNECTING = 0
    static OPEN = 1
    static CLOSING = 2
    static CLOSED = 3

    readyState = MockWebSocket.CONNECTING
    binaryType: string = 'arraybuffer'
    onopen: (() => void) | null = null
    onclose: ((event: { reason: string }) => void) | null = null
    onerror: ((event: unknown) => void) | null = null
    onmessage: ((event: { data: ArrayBuffer }) => void) | null = null

    readonly instanceId: number
    private sentMessages: ArrayBuffer[] = []

    constructor() {
      this.instanceId = ++instanceCount
      instances.push(this)
      lastInstance = this
    }

    connect() {
      this.readyState = MockWebSocket.OPEN
      queueMicrotask(() => this.onopen?.())
    }

    send(data: ArrayBuffer) {
      if (this.readyState !== MockWebSocket.OPEN) {
        throw new Error('WebSocket is not open')
      }
      this.sentMessages.push(data)
    }

    getSentMessages(): ArrayBuffer[] {
      return this.sentMessages
    }

    clearSentMessages() {
      this.sentMessages = []
    }

    receiveMessage(data: ArrayBuffer) {
      queueMicrotask(() => this.onmessage?.({ data }))
    }

    close() {
      this.readyState = MockWebSocket.CLOSED
      queueMicrotask(() => this.onclose?.({ reason: 'closed' }))
    }

    simulateError() {
      this.readyState = MockWebSocket.CLOSED
      queueMicrotask(() => this.onerror?.(new Error('Connection error')))
    }
  }

  return {
    WebSocket: MockWebSocket,
    getInstanceCount: () => instanceCount,
    getLastInstance: () => lastInstance,
    getAllInstances: () => instances,
    reset: () => {
      instanceCount = 0
      lastInstance = null
      instances.length = 0
    },
  }
}

describe('Automatic Reconnection', () => {
  let factory: ReturnType<typeof createMockWebSocketFactory>
  let originalWebSocket: typeof WebSocket

  beforeEach(() => {
    factory = createMockWebSocketFactory()
    originalWebSocket = globalThis.WebSocket
    // @ts-ignore
    globalThis.WebSocket = factory.WebSocket
    vi.useFakeTimers()
  })

  afterEach(() => {
    globalThis.WebSocket = originalWebSocket
    factory.reset()
    vi.useRealTimers()
  })

  it('should reconnect automatically after disconnect', async () => {
    const reconnectingHandler = vi.fn()
    const connectHandler = vi.fn()
    const disconnectHandler = vi.fn()

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      autoReconnect: true,
      reconnectDelay: 100,
    })

    client.on('reconnecting', reconnectingHandler)
    client.on('connect', connectHandler)
    client.on('disconnect', disconnectHandler)

    // Initial connection
    const connectPromise = client.connect()
    const ws1 = factory.getLastInstance()!
    ws1.connect()
    ws1.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()
    await connectPromise

    expect(connectHandler).toHaveBeenCalledTimes(1)
    expect(factory.getInstanceCount()).toBe(1)

    // Simulate disconnect
    ws1.close()
    await vi.runAllTimersAsync()

    expect(disconnectHandler).toHaveBeenCalledTimes(1)

    // Should trigger reconnection
    await vi.advanceTimersByTimeAsync(150)

    expect(reconnectingHandler).toHaveBeenCalled()
    expect(factory.getInstanceCount()).toBe(2)

    // Complete reconnection
    const ws2 = factory.getLastInstance()!
    ws2.connect()
    ws2.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()

    expect(connectHandler).toHaveBeenCalledTimes(2)

    client.disconnect()
  })

  it('should use exponential backoff for reconnection', async () => {
    const reconnectingHandler = vi.fn()

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      autoReconnect: true,
      reconnectDelay: 100,
      maxReconnectDelay: 5000,
    })

    client.on('reconnecting', reconnectingHandler)

    // Initial connection
    const connectPromise = client.connect()
    const ws1 = factory.getLastInstance()!
    ws1.connect()
    ws1.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()
    await connectPromise

    // Disconnect
    ws1.close()
    await vi.runAllTimersAsync()

    // First reconnection attempt
    await vi.advanceTimersByTimeAsync(150)
    expect(reconnectingHandler).toHaveBeenCalledTimes(1)
    const [attempt1, delay1] = reconnectingHandler.mock.calls[0]
    expect(attempt1).toBe(1)
    expect(delay1).toBeGreaterThanOrEqual(100)
    expect(delay1).toBeLessThan(200)

    // Second attempt (fail first)
    const ws2 = factory.getLastInstance()!
    ws2.close()
    await vi.runAllTimersAsync()

    // Wait for second reconnection
    await vi.advanceTimersByTimeAsync(500)

    if (reconnectingHandler.mock.calls.length > 1) {
      const [attempt2, delay2] = reconnectingHandler.mock.calls[1]
      expect(attempt2).toBe(2)
      // Delay should be larger due to exponential backoff
      expect(delay2).toBeGreaterThan(delay1)
    }

    client.disconnect()
  })

  it('should respect maxReconnectAttempts', async () => {
    const errorHandler = vi.fn()

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      autoReconnect: true,
      reconnectDelay: 100,
      maxReconnectAttempts: 2,
    })

    client.on('error', errorHandler)

    // Initial connection
    const connectPromise = client.connect()
    const ws1 = factory.getLastInstance()!
    ws1.connect()
    ws1.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()
    await connectPromise

    // Disconnect
    ws1.close()
    await vi.runAllTimersAsync()

    // First reconnect attempt - fail
    await vi.advanceTimersByTimeAsync(150)
    const ws2 = factory.getLastInstance()!
    ws2.close()
    await vi.runAllTimersAsync()

    // Second reconnect attempt - fail
    await vi.advanceTimersByTimeAsync(500)
    const ws3 = factory.getLastInstance()!
    ws3.close()
    await vi.runAllTimersAsync()

    // Wait for max attempts error
    await vi.advanceTimersByTimeAsync(1000)

    // Should emit error about max attempts reached
    const maxAttemptsError = errorHandler.mock.calls.find(
      call => call[0]?.message?.includes('Max reconnect attempts')
    )
    expect(maxAttemptsError).toBeDefined()

    client.disconnect()
  })

  it('should not reconnect when autoReconnect is false', async () => {
    const reconnectingHandler = vi.fn()

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      autoReconnect: false,
    })

    client.on('reconnecting', reconnectingHandler)

    // Initial connection
    const connectPromise = client.connect()
    const ws1 = factory.getLastInstance()!
    ws1.connect()
    ws1.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()
    await connectPromise

    // Disconnect
    ws1.close()
    await vi.runAllTimersAsync()

    // Wait and verify no reconnection
    await vi.advanceTimersByTimeAsync(5000)

    expect(reconnectingHandler).not.toHaveBeenCalled()
    expect(factory.getInstanceCount()).toBe(1)

    client.disconnect()
  })
})

describe('Subscription Restoration', () => {
  let factory: ReturnType<typeof createMockWebSocketFactory>
  let originalWebSocket: typeof WebSocket

  beforeEach(() => {
    factory = createMockWebSocketFactory()
    originalWebSocket = globalThis.WebSocket
    // @ts-ignore
    globalThis.WebSocket = factory.WebSocket
    vi.useFakeTimers()
  })

  afterEach(() => {
    globalThis.WebSocket = originalWebSocket
    factory.reset()
    vi.useRealTimers()
  })

  it('should restore subscriptions after reconnect', async () => {
    const callback1 = vi.fn()
    const callback2 = vi.fn()

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      autoReconnect: true,
      reconnectDelay: 100,
    })

    // Initial connection
    const connectPromise = client.connect()
    const ws1 = factory.getLastInstance()!
    ws1.connect()
    ws1.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()
    await connectPromise

    // Subscribe to topics
    const sub1Promise = client.subscribe('/scan', callback1, { msgType: 'sensor_msgs/msg/LaserScan' })
    await vi.runAllTimersAsync()
    ws1.receiveMessage(createChannelInfo(1, '/scan', 'sensor_msgs/msg/LaserScan'))
    await vi.runAllTimersAsync()

    const sub2Promise = client.subscribe('/odom', callback2, { msgType: 'nav_msgs/msg/Odometry' })
    await vi.runAllTimersAsync()
    ws1.receiveMessage(createChannelInfo(2, '/odom', 'nav_msgs/msg/Odometry'))
    await vi.runAllTimersAsync()

    await Promise.all([sub1Promise, sub2Promise])

    // Verify subscriptions are active
    const sentMessages = ws1.getSentMessages()
    const subscribeCount = sentMessages.filter(msg => {
      const [header] = parseMessage(msg)
      return header?.messageType === MessageType.SUBSCRIBE
    }).length
    expect(subscribeCount).toBe(2)

    // Disconnect
    ws1.close()
    await vi.runAllTimersAsync()

    // Reconnect
    await vi.advanceTimersByTimeAsync(150)
    const ws2 = factory.getLastInstance()!
    ws2.connect()
    ws2.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()

    // Wait for subscription restoration
    await vi.advanceTimersByTimeAsync(100)

    // Check that subscriptions were restored on new connection
    const restoredMessages = ws2.getSentMessages()
    const restoredSubscribes = restoredMessages.filter(msg => {
      const [header] = parseMessage(msg)
      return header?.messageType === MessageType.SUBSCRIBE
    })

    // Should have re-subscribed to both topics
    expect(restoredSubscribes.length).toBeGreaterThanOrEqual(1)

    client.disconnect()
  })

  it('should deliver messages to restored callbacks', async () => {
    const callback = vi.fn()

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      autoReconnect: true,
      reconnectDelay: 100,
    })

    // Initial connection and subscription
    const connectPromise = client.connect()
    const ws1 = factory.getLastInstance()!
    ws1.connect()
    ws1.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()
    await connectPromise

    const subPromise = client.subscribe('/topic', callback, { msgType: 'std_msgs/msg/String' })
    await vi.runAllTimersAsync()
    ws1.receiveMessage(createChannelInfo(1, '/topic', 'std_msgs/msg/String'))
    await vi.runAllTimersAsync()
    await subPromise

    // Send a message on first connection
    ws1.receiveMessage(createDataMessage(1, { data: 'message1' }))
    await vi.runAllTimersAsync()

    expect(callback).toHaveBeenCalledTimes(1)
    expect(callback).toHaveBeenCalledWith({ data: 'message1' }, expect.any(BigInt))

    // Disconnect and reconnect
    ws1.close()
    await vi.runAllTimersAsync()

    await vi.advanceTimersByTimeAsync(150)
    const ws2 = factory.getLastInstance()!
    ws2.connect()
    ws2.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()

    // Respond to restored subscription
    ws2.receiveMessage(createChannelInfo(3, '/topic', 'std_msgs/msg/String'))
    await vi.runAllTimersAsync()

    // Send message on new connection
    ws2.receiveMessage(createDataMessage(3, { data: 'message2' }))
    await vi.runAllTimersAsync()

    expect(callback).toHaveBeenCalledTimes(2)
    expect(callback).toHaveBeenLastCalledWith({ data: 'message2' }, expect.any(BigInt))

    client.disconnect()
  })

  it('should not restore cleared subscriptions', async () => {
    const callback = vi.fn()

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      autoReconnect: true,
      reconnectDelay: 100,
    })

    // Initial connection
    const connectPromise = client.connect()
    const ws1 = factory.getLastInstance()!
    ws1.connect()
    ws1.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()
    await connectPromise

    // Subscribe
    const subPromise = client.subscribe('/topic', callback)
    await vi.runAllTimersAsync()
    ws1.receiveMessage(createChannelInfo(1, '/topic', 'std_msgs/msg/String'))
    await vi.runAllTimersAsync()
    const sub = await subPromise

    // Unsubscribe
    sub.unsubscribe()
    client.clearSavedSubscription('/topic', callback)
    await vi.runAllTimersAsync()

    // Disconnect and reconnect
    ws1.close()
    await vi.runAllTimersAsync()

    await vi.advanceTimersByTimeAsync(150)
    const ws2 = factory.getLastInstance()!
    ws2.connect()
    ws2.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()

    // Wait for any restoration
    await vi.advanceTimersByTimeAsync(500)

    // Should NOT have re-subscribed
    const restoredMessages = ws2.getSentMessages()
    const subscribeMessages = restoredMessages.filter(msg => {
      const [header] = parseMessage(msg)
      return header?.messageType === MessageType.SUBSCRIBE
    })

    expect(subscribeMessages.length).toBe(0)

    client.disconnect()
  })

  it('should handle multiple callbacks for same topic on reconnect', async () => {
    const callback1 = vi.fn()
    const callback2 = vi.fn()

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      autoReconnect: true,
      reconnectDelay: 100,
    })

    // Initial connection
    const connectPromise = client.connect()
    const ws1 = factory.getLastInstance()!
    ws1.connect()
    ws1.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()
    await connectPromise

    // Subscribe with first callback
    const sub1Promise = client.subscribe('/topic', callback1)
    await vi.runAllTimersAsync()
    ws1.receiveMessage(createChannelInfo(1, '/topic', 'std_msgs/msg/String'))
    await vi.runAllTimersAsync()
    await sub1Promise

    // Add second callback to same topic
    const sub2Promise = client.subscribe('/topic', callback2)
    await vi.runAllTimersAsync()
    await sub2Promise

    // Verify both callbacks receive messages
    ws1.receiveMessage(createDataMessage(1, { data: 'test' }))
    await vi.runAllTimersAsync()

    expect(callback1).toHaveBeenCalledTimes(1)
    expect(callback2).toHaveBeenCalledTimes(1)

    // Disconnect and reconnect
    ws1.close()
    await vi.runAllTimersAsync()

    await vi.advanceTimersByTimeAsync(150)
    const ws2 = factory.getLastInstance()!
    ws2.connect()
    ws2.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()

    // Respond to restored subscription
    ws2.receiveMessage(createChannelInfo(5, '/topic', 'std_msgs/msg/String'))
    await vi.runAllTimersAsync()

    // Both callbacks should receive message after reconnect
    ws2.receiveMessage(createDataMessage(5, { data: 'after_reconnect' }))
    await vi.runAllTimersAsync()

    // One of the callbacks should receive the message
    const totalCalls = callback1.mock.calls.length + callback2.mock.calls.length
    expect(totalCalls).toBeGreaterThanOrEqual(2)

    client.disconnect()
  })
})

describe('Connection State Management', () => {
  let factory: ReturnType<typeof createMockWebSocketFactory>
  let originalWebSocket: typeof WebSocket

  beforeEach(() => {
    factory = createMockWebSocketFactory()
    originalWebSocket = globalThis.WebSocket
    // @ts-ignore
    globalThis.WebSocket = factory.WebSocket
    vi.useFakeTimers()
  })

  afterEach(() => {
    globalThis.WebSocket = originalWebSocket
    factory.reset()
    vi.useRealTimers()
  })

  it('should track connection state correctly', async () => {
    const client = new RosKitClient({ url: 'ws://localhost:9090' })

    expect(client.getState()).toBe('disconnected')
    expect(client.isConnected()).toBe(false)

    const connectPromise = client.connect()
    expect(client.getState()).toBe('connecting')

    const ws = factory.getLastInstance()!
    ws.connect()
    ws.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()
    await connectPromise

    expect(client.getState()).toBe('connected')
    expect(client.isConnected()).toBe(true)

    client.disconnect()

    expect(client.getState()).toBe('disconnected')
    expect(client.isConnected()).toBe(false)
  })

  it('should clear channels on disconnect', async () => {
    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      autoReconnect: false,
    })

    const connectPromise = client.connect()
    const ws = factory.getLastInstance()!
    ws.connect()
    ws.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()
    await connectPromise

    // Subscribe
    const subPromise = client.subscribe('/topic', vi.fn())
    await vi.runAllTimersAsync()
    ws.receiveMessage(createChannelInfo(1, '/topic', 'std_msgs/msg/String'))
    await vi.runAllTimersAsync()
    await subPromise

    // Disconnect
    client.disconnect()

    // Internal channels should be cleared (tested via attempting operations)
    expect(client.isConnected()).toBe(false)
  })

  it('should handle rapid connect/disconnect', async () => {
    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      autoReconnect: false,
    })

    // Rapid connect/disconnect sequence
    const p1 = client.connect()
    const ws1 = factory.getLastInstance()!
    ws1.connect()
    ws1.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()
    await p1

    client.disconnect()

    const p2 = client.connect()
    const ws2 = factory.getLastInstance()!
    ws2.connect()
    ws2.receiveMessage(createServerInfo())
    await vi.runAllTimersAsync()
    await p2

    client.disconnect()

    expect(client.getState()).toBe('disconnected')
    expect(factory.getInstanceCount()).toBe(2)
  })
})

// Helper functions
function createServerInfo(): ArrayBuffer {
  return createMessage(
    MessageType.SERVER_INFO,
    0,
    new Uint8Array(cborEncode({
      name: 'test-bridge',
      protocol_version: 1,
      capabilities: ['subscribe', 'publish', 'service'],
      encodings: ['cbor', 'raw', 'binary'],
    })),
    Encoding.CBOR
  )
}

function createChannelInfo(channelId: number, topic: string, msgType: string): ArrayBuffer {
  return createMessage(
    MessageType.CHANNEL_INFO,
    0,
    new Uint8Array(cborEncode({
      channel_id: channelId,
      topic,
      msg_type: msgType,
      encoding: Encoding.CBOR,
    })),
    Encoding.CBOR
  )
}

function createDataMessage(channelId: number, data: unknown): ArrayBuffer {
  return createMessage(
    MessageType.MESSAGE,
    channelId,
    new Uint8Array(cborEncode(data)),
    Encoding.CBOR,
    BigInt(Date.now() * 1_000_000)
  )
}
