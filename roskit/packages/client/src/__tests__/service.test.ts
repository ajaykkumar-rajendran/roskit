/**
 * Service Timeout and Correlation Tests
 *
 * Tests request_id matching, timeout handling, and retry logic for service calls.
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
import { RosKitError } from '../types'

// Mock WebSocket for testing
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

  private sentMessages: ArrayBuffer[] = []

  connect() {
    this.readyState = MockWebSocket.OPEN
    setTimeout(() => this.onopen?.(), 0)
  }

  send(data: ArrayBuffer) {
    this.sentMessages.push(data)
  }

  getSentMessages(): ArrayBuffer[] {
    return this.sentMessages
  }

  getLastSentMessage(): ArrayBuffer | undefined {
    return this.sentMessages[this.sentMessages.length - 1]
  }

  clearSentMessages() {
    this.sentMessages = []
  }

  receiveMessage(data: ArrayBuffer) {
    this.onmessage?.({ data })
  }

  close() {
    this.readyState = MockWebSocket.CLOSED
    this.onclose?.({ reason: 'closed' })
  }
}

describe('Service Request ID Correlation', () => {
  let mockWs: MockWebSocket
  let originalWebSocket: typeof WebSocket

  beforeEach(() => {
    mockWs = new MockWebSocket()
    originalWebSocket = globalThis.WebSocket
    // @ts-ignore - mock WebSocket
    globalThis.WebSocket = class extends MockWebSocket {
      constructor() {
        super()
        mockWs = this as any
        setTimeout(() => {
          this.connect()
          // Send server info to complete connection
          const serverInfo = createMessage(
            MessageType.SERVER_INFO,
            0,
            new Uint8Array(cborEncode({ name: 'test', protocol_version: 1 })),
            Encoding.CBOR
          )
          this.receiveMessage(serverInfo)
        }, 10)
      }
    }
  })

  afterEach(() => {
    globalThis.WebSocket = originalWebSocket
    vi.restoreAllMocks()
  })

  it('should generate unique request IDs for each service call', async () => {
    const client = new RosKitClient({ url: 'ws://localhost:9090' })
    await client.connect()

    // Make multiple service calls without awaiting
    const call1 = client.serviceCall('/service1', 'std_srvs/srv/Trigger', {}, { timeoutMs: 100 })
    const call2 = client.serviceCall('/service2', 'std_srvs/srv/Trigger', {}, { timeoutMs: 100 })
    const call3 = client.serviceCall('/service3', 'std_srvs/srv/Trigger', {}, { timeoutMs: 100 })

    // Wait for messages to be sent
    await new Promise(r => setTimeout(r, 50))

    const messages = mockWs.getSentMessages()
    expect(messages.length).toBe(3)

    // Extract request IDs
    const requestIds = messages.map(msg => {
      const [header, payload] = parseMessage(msg)
      if (header?.messageType === MessageType.SERVICE_CALL) {
        const decoded = cborDecode(payload) as { request_id: string }
        return decoded.request_id
      }
      return null
    }).filter(Boolean)

    // All request IDs should be unique
    const uniqueIds = new Set(requestIds)
    expect(uniqueIds.size).toBe(3)

    // Clean up - let timeouts expire
    await Promise.allSettled([call1, call2, call3])
    client.disconnect()
  })

  it('should match response to correct pending request', async () => {
    const client = new RosKitClient({ url: 'ws://localhost:9090' })
    await client.connect()

    // Start two service calls
    const call1Promise = client.serviceCall('/service1', 'std_srvs/srv/Trigger', {}, { timeoutMs: 5000 })
    const call2Promise = client.serviceCall('/service2', 'std_srvs/srv/Trigger', {}, { timeoutMs: 5000 })

    await new Promise(r => setTimeout(r, 50))

    // Extract request IDs from sent messages
    const messages = mockWs.getSentMessages()
    const requestIds: string[] = []
    for (const msg of messages) {
      const [header, payload] = parseMessage(msg)
      if (header?.messageType === MessageType.SERVICE_CALL) {
        const decoded = cborDecode(payload) as { request_id: string }
        requestIds.push(decoded.request_id)
      }
    }

    expect(requestIds.length).toBe(2)

    // Respond to second call first (out of order)
    const response2 = createMessage(
      MessageType.SERVICE_RESPONSE,
      0,
      new Uint8Array(cborEncode({ request_id: requestIds[1], response: { value: 'second' } })),
      Encoding.CBOR
    )
    mockWs.receiveMessage(response2)

    // Respond to first call
    const response1 = createMessage(
      MessageType.SERVICE_RESPONSE,
      0,
      new Uint8Array(cborEncode({ request_id: requestIds[0], response: { value: 'first' } })),
      Encoding.CBOR
    )
    mockWs.receiveMessage(response1)

    const [result1, result2] = await Promise.all([call1Promise, call2Promise])

    // Verify correct correlation despite out-of-order responses
    expect(result1).toEqual({ value: 'first' })
    expect(result2).toEqual({ value: 'second' })

    client.disconnect()
  })

  it('should ignore responses with unknown request IDs', async () => {
    const client = new RosKitClient({ url: 'ws://localhost:9090' })
    const errorHandler = vi.fn()
    client.on('error', errorHandler)
    await client.connect()

    // Send response with unknown request ID
    const unknownResponse = createMessage(
      MessageType.SERVICE_RESPONSE,
      0,
      new Uint8Array(cborEncode({ request_id: 'unknown_id_12345', response: { data: 'test' } })),
      Encoding.CBOR
    )
    mockWs.receiveMessage(unknownResponse)

    await new Promise(r => setTimeout(r, 50))

    // Should not cause errors (silently ignored)
    // The client drops unknown request IDs without emitting an error
    client.disconnect()
  })

  it('should reject on service error response', async () => {
    const client = new RosKitClient({ url: 'ws://localhost:9090' })
    await client.connect()

    const callPromise = client.serviceCall('/failing_service', 'std_srvs/srv/Trigger', {}, { timeoutMs: 5000 })

    await new Promise(r => setTimeout(r, 50))

    // Extract request ID
    const messages = mockWs.getSentMessages()
    const [header, payload] = parseMessage(messages[0])
    const { request_id } = cborDecode(payload) as { request_id: string }

    // Send error response
    const errorResponse = createMessage(
      MessageType.SERVICE_RESPONSE,
      0,
      new Uint8Array(cborEncode({ request_id, error: 'Service unavailable' })),
      Encoding.CBOR
    )
    mockWs.receiveMessage(errorResponse)

    await expect(callPromise).rejects.toThrow('Service unavailable')

    client.disconnect()
  })
})

describe('Service Timeout Handling', () => {
  let originalWebSocket: typeof WebSocket

  beforeEach(() => {
    originalWebSocket = globalThis.WebSocket
    vi.useFakeTimers()
  })

  afterEach(() => {
    globalThis.WebSocket = originalWebSocket
    vi.useRealTimers()
  })

  it('should timeout service call after specified duration', async () => {
    let mockWs: MockWebSocket | null = null

    // @ts-ignore
    globalThis.WebSocket = class extends MockWebSocket {
      constructor() {
        super()
        mockWs = this as any
        queueMicrotask(() => {
          this.connect()
          const serverInfo = createMessage(
            MessageType.SERVER_INFO,
            0,
            new Uint8Array(cborEncode({ name: 'test', protocol_version: 1 })),
            Encoding.CBOR
          )
          this.receiveMessage(serverInfo)
        })
      }
    }

    const client = new RosKitClient({ url: 'ws://localhost:9090' })
    const connectPromise = client.connect()

    await vi.runAllTimersAsync()
    await connectPromise

    const callPromise = client.serviceCall('/slow_service', 'std_srvs/srv/Trigger', {}, { timeoutMs: 1000 })

    // Advance time past timeout
    await vi.advanceTimersByTimeAsync(1001)

    await expect(callPromise).rejects.toThrow(/timed out/)

    client.disconnect()
  })

  it('should use default timeout when not specified', async () => {
    let mockWs: MockWebSocket | null = null

    // @ts-ignore
    globalThis.WebSocket = class extends MockWebSocket {
      constructor() {
        super()
        mockWs = this as any
        queueMicrotask(() => {
          this.connect()
          const serverInfo = createMessage(
            MessageType.SERVER_INFO,
            0,
            new Uint8Array(cborEncode({ name: 'test', protocol_version: 1 })),
            Encoding.CBOR
          )
          this.receiveMessage(serverInfo)
        })
      }
    }

    const client = new RosKitClient({ url: 'ws://localhost:9090' })
    const connectPromise = client.connect()

    await vi.runAllTimersAsync()
    await connectPromise

    // No timeout specified - should use default (10000ms)
    const callPromise = client.serviceCall('/service', 'std_srvs/srv/Trigger', {})

    // Advance time past default timeout
    await vi.advanceTimersByTimeAsync(10001)

    await expect(callPromise).rejects.toThrow(/timed out/)

    client.disconnect()
  })

  it('should clear timeout on successful response', async () => {
    let mockWs: MockWebSocket | null = null

    // @ts-ignore
    globalThis.WebSocket = class extends MockWebSocket {
      constructor() {
        super()
        mockWs = this as any
        queueMicrotask(() => {
          this.connect()
          const serverInfo = createMessage(
            MessageType.SERVER_INFO,
            0,
            new Uint8Array(cborEncode({ name: 'test', protocol_version: 1 })),
            Encoding.CBOR
          )
          this.receiveMessage(serverInfo)
        })
      }
    }

    const client = new RosKitClient({ url: 'ws://localhost:9090' })
    const connectPromise = client.connect()

    await vi.runAllTimersAsync()
    await connectPromise

    const callPromise = client.serviceCall('/service', 'std_srvs/srv/Trigger', {}, { timeoutMs: 5000 })

    // Get request ID
    await vi.advanceTimersByTimeAsync(10)
    const messages = mockWs!.getSentMessages()
    const [, payload] = parseMessage(messages[0])
    const { request_id } = cborDecode(payload) as { request_id: string }

    // Respond before timeout
    const response = createMessage(
      MessageType.SERVICE_RESPONSE,
      0,
      new Uint8Array(cborEncode({ request_id, response: { success: true } })),
      Encoding.CBOR
    )
    mockWs!.receiveMessage(response)

    const result = await callPromise
    expect(result).toEqual({ success: true })

    // Advance time past original timeout - should not cause issues
    await vi.advanceTimersByTimeAsync(10000)

    client.disconnect()
  })

  it('should expire all pending calls on disconnect', async () => {
    let mockWs: MockWebSocket | null = null

    // @ts-ignore
    globalThis.WebSocket = class extends MockWebSocket {
      constructor() {
        super()
        mockWs = this as any
        queueMicrotask(() => {
          this.connect()
          const serverInfo = createMessage(
            MessageType.SERVER_INFO,
            0,
            new Uint8Array(cborEncode({ name: 'test', protocol_version: 1 })),
            Encoding.CBOR
          )
          this.receiveMessage(serverInfo)
        })
      }
    }

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      autoReconnect: false,
    })
    const connectPromise = client.connect()

    await vi.runAllTimersAsync()
    await connectPromise

    const call1 = client.serviceCall('/service1', 'std_srvs/srv/Trigger', {}, { timeoutMs: 60000 })
    const call2 = client.serviceCall('/service2', 'std_srvs/srv/Trigger', {}, { timeoutMs: 60000 })

    // Wait for calls to be sent
    await vi.advanceTimersByTimeAsync(10)

    // Simulate disconnect
    mockWs!.close()

    // Both calls should be rejected
    await expect(call1).rejects.toThrow()
    await expect(call2).rejects.toThrow()
  })
})

describe('Service Retry Logic', () => {
  let originalWebSocket: typeof WebSocket

  beforeEach(() => {
    originalWebSocket = globalThis.WebSocket
    vi.useFakeTimers()
  })

  afterEach(() => {
    globalThis.WebSocket = originalWebSocket
    vi.useRealTimers()
  })

  it('should retry on timeout when retries configured', async () => {
    let mockWs: MockWebSocket | null = null
    let attemptCount = 0

    // @ts-ignore
    globalThis.WebSocket = class extends MockWebSocket {
      constructor() {
        super()
        mockWs = this as any
        queueMicrotask(() => {
          this.connect()
          const serverInfo = createMessage(
            MessageType.SERVER_INFO,
            0,
            new Uint8Array(cborEncode({ name: 'test', protocol_version: 1 })),
            Encoding.CBOR
          )
          this.receiveMessage(serverInfo)
        })
      }

      send(data: ArrayBuffer) {
        super.send(data)
        attemptCount++

        // Succeed on third attempt
        if (attemptCount === 3) {
          const [, payload] = parseMessage(data)
          const { request_id } = cborDecode(payload) as { request_id: string }

          queueMicrotask(() => {
            const response = createMessage(
              MessageType.SERVICE_RESPONSE,
              0,
              new Uint8Array(cborEncode({ request_id, response: { success: true, attempt: attemptCount } })),
              Encoding.CBOR
            )
            mockWs!.receiveMessage(response)
          })
        }
      }
    }

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      retryPolicy: {
        maxRetries: 3,
        baseDelayMs: 100,
        backoffMultiplier: 2,
      },
    })
    const connectPromise = client.connect()
    await vi.runAllTimersAsync()
    await connectPromise

    const callPromise = client.serviceCall('/service', 'std_srvs/srv/Trigger', {}, {
      timeoutMs: 500,
      retries: 3,
    })

    // First attempt times out
    await vi.advanceTimersByTimeAsync(501)
    // Retry delay + second attempt times out
    await vi.advanceTimersByTimeAsync(600)
    // Retry delay + third attempt succeeds
    await vi.advanceTimersByTimeAsync(300)

    const result = await callPromise
    expect(result).toEqual({ success: true, attempt: 3 })
    expect(attemptCount).toBe(3)

    client.disconnect()
  })

  it('should not retry on server error (non-retryable)', async () => {
    let mockWs: MockWebSocket | null = null
    let attemptCount = 0

    // @ts-ignore
    globalThis.WebSocket = class extends MockWebSocket {
      constructor() {
        super()
        mockWs = this as any
        queueMicrotask(() => {
          this.connect()
          const serverInfo = createMessage(
            MessageType.SERVER_INFO,
            0,
            new Uint8Array(cborEncode({ name: 'test', protocol_version: 1 })),
            Encoding.CBOR
          )
          this.receiveMessage(serverInfo)
        })
      }

      send(data: ArrayBuffer) {
        super.send(data)
        attemptCount++

        const [header, payload] = parseMessage(data)
        if (header?.messageType === MessageType.SERVICE_CALL) {
          const { request_id } = cborDecode(payload) as { request_id: string }

          queueMicrotask(() => {
            // Return server error (not retryable)
            const response = createMessage(
              MessageType.SERVICE_RESPONSE,
              0,
              new Uint8Array(cborEncode({ request_id, error: 'Service not found' })),
              Encoding.CBOR
            )
            mockWs!.receiveMessage(response)
          })
        }
      }
    }

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      retryPolicy: { maxRetries: 3 },
    })
    const connectPromise = client.connect()
    await vi.runAllTimersAsync()
    await connectPromise

    const callPromise = client.serviceCall('/service', 'std_srvs/srv/Trigger', {}, { retries: 3 })

    await vi.advanceTimersByTimeAsync(100)

    await expect(callPromise).rejects.toThrow('Service not found')
    expect(attemptCount).toBe(1) // No retries for server errors

    client.disconnect()
  })
})

describe('Service Call Request Format', () => {
  it('should encode service call with correct fields', () => {
    const serviceCall = {
      service: '/robot/navigate',
      msg_type: 'nav2_msgs/srv/NavigateToPose',
      request: {
        pose: {
          header: { frame_id: 'map', stamp: { sec: 0, nanosec: 0 } },
          pose: {
            position: { x: 1.0, y: 2.0, z: 0.0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          },
        },
      },
      request_id: 'test_123',
    }

    const payload = new Uint8Array(cborEncode(serviceCall))
    const message = createMessage(MessageType.SERVICE_CALL, 0, payload, Encoding.CBOR)
    const [header, decoded] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.SERVICE_CALL)

    const parsed = cborDecode(decoded) as typeof serviceCall
    expect(parsed.service).toBe('/robot/navigate')
    expect(parsed.msg_type).toBe('nav2_msgs/srv/NavigateToPose')
    expect(parsed.request_id).toBe('test_123')
    expect(parsed.request.pose.pose.position.x).toBe(1.0)
  })

  it('should handle empty request body', () => {
    const serviceCall = {
      service: '/trigger',
      msg_type: 'std_srvs/srv/Trigger',
      request: {},
      request_id: 'trigger_1',
    }

    const payload = new Uint8Array(cborEncode(serviceCall))
    const message = createMessage(MessageType.SERVICE_CALL, 0, payload, Encoding.CBOR)
    const [header, decoded] = parseMessage(message)

    expect(header).not.toBeNull()

    const parsed = cborDecode(decoded) as typeof serviceCall
    expect(parsed.request).toEqual({})
  })

  it('should handle complex nested request', () => {
    const serviceCall = {
      service: '/compute_path',
      msg_type: 'nav_msgs/srv/GetPlan',
      request: {
        start: {
          header: { frame_id: 'map', stamp: { sec: 100, nanosec: 500000 } },
          pose: {
            position: { x: 0, y: 0, z: 0 },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          },
        },
        goal: {
          header: { frame_id: 'map', stamp: { sec: 100, nanosec: 500000 } },
          pose: {
            position: { x: 10, y: 10, z: 0 },
            orientation: { x: 0, y: 0, z: 0.707, w: 0.707 },
          },
        },
        tolerance: 0.5,
      },
      request_id: 'path_req_1',
    }

    const payload = new Uint8Array(cborEncode(serviceCall))
    const decoded = cborDecode(payload) as typeof serviceCall

    expect(decoded.request.start.pose.position).toEqual({ x: 0, y: 0, z: 0 })
    expect(decoded.request.goal.pose.position).toEqual({ x: 10, y: 10, z: 0 })
    expect(decoded.request.tolerance).toBe(0.5)
  })
})

describe('Service Response Format', () => {
  it('should decode successful response', () => {
    const response = {
      request_id: 'req_12345',
      response: {
        success: true,
        message: 'Operation completed',
        result: { value: 42 },
      },
    }

    const payload = new Uint8Array(cborEncode(response))
    const message = createMessage(MessageType.SERVICE_RESPONSE, 0, payload, Encoding.CBOR)
    const [header, decoded] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.messageType).toBe(MessageType.SERVICE_RESPONSE)

    const parsed = cborDecode(decoded) as typeof response
    expect(parsed.request_id).toBe('req_12345')
    expect(parsed.response.success).toBe(true)
    expect(parsed.response.result.value).toBe(42)
  })

  it('should decode error response', () => {
    const response = {
      request_id: 'req_failed',
      error: 'Service execution failed: robot not ready',
    }

    const payload = new Uint8Array(cborEncode(response))
    const message = createMessage(MessageType.SERVICE_RESPONSE, 0, payload, Encoding.CBOR)
    const [header, decoded] = parseMessage(message)

    expect(header).not.toBeNull()

    const parsed = cborDecode(decoded) as typeof response
    expect(parsed.request_id).toBe('req_failed')
    expect(parsed.error).toContain('robot not ready')
    expect(parsed).not.toHaveProperty('response')
  })

  it('should handle response with binary data', () => {
    const response = {
      request_id: 'binary_req',
      response: {
        data: new Uint8Array([1, 2, 3, 4, 5]),
        format: 'raw',
      },
    }

    const payload = new Uint8Array(cborEncode(response))
    const decoded = cborDecode(payload) as typeof response

    expect(decoded.response.data.length).toBe(5)
    expect(decoded.response.format).toBe('raw')
  })
})

describe('Service Call Age-Based Eviction', () => {
  let originalWebSocket: typeof WebSocket

  beforeEach(() => {
    originalWebSocket = globalThis.WebSocket
    vi.useFakeTimers()
  })

  afterEach(() => {
    globalThis.WebSocket = originalWebSocket
    vi.useRealTimers()
  })

  it('should evict pending service calls older than pendingServiceMaxAgeMs', async () => {
    let mockWs: MockWebSocket | null = null

    // @ts-ignore
    globalThis.WebSocket = class extends MockWebSocket {
      constructor() {
        super()
        mockWs = this as any
        queueMicrotask(() => {
          this.connect()
          const serverInfo = createMessage(
            MessageType.SERVER_INFO,
            0,
            new Uint8Array(cborEncode({ name: 'test', protocol_version: 1 })),
            Encoding.CBOR
          )
          this.receiveMessage(serverInfo)
        })
      }
    }

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      pendingServiceMaxAgeMs: 30000, // 30 seconds max age
      evictionIntervalMs: 10000, // Eviction runs every 10 seconds
    })
    const connectPromise = client.connect()

    await vi.runAllTimersAsync()
    await connectPromise

    // Make a service call with a very long timeout (60 seconds)
    // This ensures the regular timeout doesn't fire before eviction
    const callPromise = client.serviceCall('/slow_service', 'std_srvs/srv/Trigger', {}, { timeoutMs: 60000 })

    // Wait for call to be sent
    await vi.advanceTimersByTimeAsync(100)

    // Advance time past the pendingServiceMaxAgeMs but not the call timeout
    // First eviction at 10s - call is 10s old (not evicted)
    await vi.advanceTimersByTimeAsync(10000)

    // Second eviction at 20s - call is 20s old (not evicted)
    await vi.advanceTimersByTimeAsync(10000)

    // Third eviction at 30s - call is 30s old (not evicted, exactly at boundary)
    await vi.advanceTimersByTimeAsync(10000)

    // Fourth eviction at 40s - call is 40s old (>30s, should be evicted)
    await vi.advanceTimersByTimeAsync(10000)

    // The call should be rejected due to age-based eviction
    await expect(callPromise).rejects.toThrow(/evicted after 30000ms/)

    client.disconnect()
  })

  it('should not evict service calls that complete before max age', async () => {
    let mockWs: MockWebSocket | null = null

    // @ts-ignore
    globalThis.WebSocket = class extends MockWebSocket {
      constructor() {
        super()
        mockWs = this as any
        queueMicrotask(() => {
          this.connect()
          const serverInfo = createMessage(
            MessageType.SERVER_INFO,
            0,
            new Uint8Array(cborEncode({ name: 'test', protocol_version: 1 })),
            Encoding.CBOR
          )
          this.receiveMessage(serverInfo)
        })
      }
    }

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      pendingServiceMaxAgeMs: 30000,
      evictionIntervalMs: 10000,
    })
    const connectPromise = client.connect()

    await vi.runAllTimersAsync()
    await connectPromise

    const callPromise = client.serviceCall('/fast_service', 'std_srvs/srv/Trigger', {}, { timeoutMs: 60000 })

    // Wait for call to be sent
    await vi.advanceTimersByTimeAsync(100)

    // Get the request_id from the sent message
    const sentMessage = mockWs!.getLastSentMessage()!
    const [, payload] = parseMessage(sentMessage)
    const { request_id } = cborDecode(payload) as { request_id: string }

    // Respond before eviction time
    await vi.advanceTimersByTimeAsync(5000)

    const response = createMessage(
      MessageType.SERVICE_RESPONSE,
      0,
      new Uint8Array(cborEncode({ request_id, response: { success: true } })),
      Encoding.CBOR
    )
    mockWs!.receiveMessage(response)

    // The call should resolve successfully
    const result = await callPromise
    expect(result).toEqual({ success: true })

    client.disconnect()
  })

  it('should clear timeout when evicting by age', async () => {
    let mockWs: MockWebSocket | null = null

    // @ts-ignore
    globalThis.WebSocket = class extends MockWebSocket {
      constructor() {
        super()
        mockWs = this as any
        queueMicrotask(() => {
          this.connect()
          const serverInfo = createMessage(
            MessageType.SERVER_INFO,
            0,
            new Uint8Array(cborEncode({ name: 'test', protocol_version: 1 })),
            Encoding.CBOR
          )
          this.receiveMessage(serverInfo)
        })
      }
    }

    const client = new RosKitClient({
      url: 'ws://localhost:9090',
      pendingServiceMaxAgeMs: 20000, // 20 seconds
      evictionIntervalMs: 5000, // 5 second eviction interval
    })
    const connectPromise = client.connect()

    await vi.runAllTimersAsync()
    await connectPromise

    // Call with 60 second timeout
    const callPromise = client.serviceCall('/service', 'std_srvs/srv/Trigger', {}, { timeoutMs: 60000 })

    // Wait for call to be sent
    await vi.advanceTimersByTimeAsync(100)

    // Advance past max age (25s > 20s)
    await vi.advanceTimersByTimeAsync(25000)

    // Call should be rejected by eviction (not timeout)
    await expect(callPromise).rejects.toThrow(/evicted after 20000ms/)

    // Advance past the original timeout - should not cause any errors
    // because the timeout should have been cleared
    await vi.advanceTimersByTimeAsync(60000)

    client.disconnect()
  })
})
