/**
 * React Hook Tests with Mocked Client
 *
 * Tests useSubscription, useService, and context hooks with fully mocked RosKitClient.
 */

import React, { act } from 'react'
import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest'
import { renderHook, waitFor } from '@testing-library/react'
import { RosKitProvider } from '../context'
import { useSubscription } from '../hooks/useSubscription'
import { useService } from '../hooks/useService'
import { SubscriptionStatus, ServiceCallStatus, ConnectionStatus } from '../types'
import type { RosKitClient, Subscription, MessageCallback } from '@roskit/client'

// Create a mock client factory
function createMockClient(overrides: Partial<RosKitClient> = {}): RosKitClient {
  const eventHandlers = new Map<string, Set<(...args: any[]) => void>>()

  const mockClient: Partial<RosKitClient> = {
    connect: vi.fn().mockResolvedValue(undefined),
    disconnect: vi.fn(),
    isConnected: vi.fn().mockReturnValue(true),
    getState: vi.fn().mockReturnValue('connected'),
    subscribe: vi.fn(),
    getTopics: vi.fn().mockResolvedValue([]),
    getServices: vi.fn().mockResolvedValue([]),
    publish: vi.fn(),
    serviceCall: vi.fn(),
    ping: vi.fn(),
    getServerInfo: vi.fn().mockReturnValue(null),
    getProtocolVersion: vi.fn().mockReturnValue(1),
    hasCapability: vi.fn().mockReturnValue(true),
    supportsEncoding: vi.fn().mockReturnValue(true),
    getCapabilities: vi.fn().mockReturnValue([]),
    getSupportedEncodings: vi.fn().mockReturnValue(['cbor']),
    clearSavedSubscription: vi.fn(),
    destroy: vi.fn(),
    on: vi.fn((event: string, callback: (...args: any[]) => void) => {
      if (!eventHandlers.has(event)) {
        eventHandlers.set(event, new Set())
      }
      eventHandlers.get(event)!.add(callback)
    }),
    off: vi.fn((event: string, callback: (...args: any[]) => void) => {
      eventHandlers.get(event)?.delete(callback)
    }),
    // Helper to emit events in tests
    _emit: (event: string, ...args: any[]) => {
      eventHandlers.get(event)?.forEach(cb => cb(...args))
    },
    ...overrides,
  }

  return mockClient as RosKitClient
}

// Mock RosKitClient constructor
vi.mock('@roskit/client', async () => {
  const actual = await vi.importActual('@roskit/client')
  return {
    ...actual,
    RosKitClient: vi.fn(),
  }
})

describe('useSubscription', () => {
  let mockClient: RosKitClient & { _emit: (event: string, ...args: any[]) => void }
  let subscriptionCallback: MessageCallback | null = null

  beforeEach(() => {
    vi.useFakeTimers()

    mockClient = createMockClient({
      subscribe: vi.fn().mockImplementation(async (topic: string, callback: MessageCallback) => {
        subscriptionCallback = callback
        const subscription: Subscription = {
          id: 'sub_1',
          channelId: 1,
          topic,
          msgType: 'std_msgs/msg/String',
          unsubscribe: vi.fn(),
        }
        return subscription
      }),
    }) as any

    const { RosKitClient } = require('@roskit/client')
    RosKitClient.mockImplementation(() => mockClient)
  })

  afterEach(() => {
    vi.useRealTimers()
    vi.clearAllMocks()
    subscriptionCallback = null
  })

  const wrapper = ({ children }: { children: React.ReactNode }) => (
    <RosKitProvider config={{ url: 'ws://localhost:9090' }} autoConnect>
      {children}
    </RosKitProvider>
  )

  it('should start with initial state', async () => {
    // Simulate connection
    setTimeout(() => mockClient._emit('connect'), 10)

    const { result } = renderHook(() => useSubscription('/test'), { wrapper })

    expect(result.current.data).toBeNull()
    expect(result.current.isSubscribed).toBe(false)
    expect(result.current.error).toBeNull()
    expect(result.current.messageCount).toBe(0)
  })

  it('should subscribe when connected', async () => {
    const { result } = renderHook(() => useSubscription('/topic'), { wrapper })

    // Simulate connection
    await act(async () => {
      mockClient._emit('connect')
      await vi.advanceTimersByTimeAsync(100)
    })

    await waitFor(() => {
      expect(mockClient.subscribe).toHaveBeenCalledWith(
        '/topic',
        expect.any(Function),
        expect.any(Object)
      )
    })
  })

  it('should update data when message received', async () => {
    const { result } = renderHook(() => useSubscription<{ data: string }>('/topic'), { wrapper })

    await act(async () => {
      mockClient._emit('connect')
      await vi.advanceTimersByTimeAsync(100)
    })

    // Wait for subscription
    await waitFor(() => {
      expect(subscriptionCallback).not.toBeNull()
    })

    // Simulate message
    await act(async () => {
      subscriptionCallback!({ data: 'hello' }, BigInt(1000))
    })

    expect(result.current.data).toEqual({ data: 'hello' })
    expect(result.current.messageCount).toBe(1)
    expect(result.current.timestamp).toBe(BigInt(1000))
  })

  it('should call onMessage callback', async () => {
    const onMessage = vi.fn()

    const { result } = renderHook(
      () => useSubscription<{ value: number }>('/topic', { onMessage }),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
      await vi.advanceTimersByTimeAsync(100)
    })

    await waitFor(() => {
      expect(subscriptionCallback).not.toBeNull()
    })

    await act(async () => {
      subscriptionCallback!({ value: 42 }, BigInt(2000))
    })

    expect(onMessage).toHaveBeenCalledWith({ value: 42 }, BigInt(2000))
  })

  it('should not subscribe when disabled', async () => {
    const { result } = renderHook(
      () => useSubscription('/topic', { enabled: false }),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
      await vi.advanceTimersByTimeAsync(100)
    })

    expect(result.current.status).toBe(SubscriptionStatus.DISABLED)
    expect(mockClient.subscribe).not.toHaveBeenCalled()
  })

  it('should handle subscription error', async () => {
    const subscribeError = new Error('Subscription failed')
    mockClient.subscribe = vi.fn().mockRejectedValue(subscribeError)

    const { result } = renderHook(
      () => useSubscription('/topic', { retryPolicy: { maxRetries: 0 } }),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
      await vi.advanceTimersByTimeAsync(100)
    })

    await waitFor(() => {
      expect(result.current.status).toBe(SubscriptionStatus.ERROR)
      expect(result.current.error).toBe(subscribeError)
    })
  })

  it('should retry on subscription failure', async () => {
    let attemptCount = 0
    mockClient.subscribe = vi.fn().mockImplementation(async () => {
      attemptCount++
      if (attemptCount < 3) {
        throw new Error('Temporary failure')
      }
      subscriptionCallback = vi.fn()
      return {
        id: 'sub_1',
        channelId: 1,
        topic: '/topic',
        msgType: 'test',
        unsubscribe: vi.fn(),
      }
    })

    const { result } = renderHook(
      () => useSubscription('/topic', {
        retryPolicy: { maxRetries: 5, initialDelayMs: 100, backoffMultiplier: 1 }
      }),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
      await vi.advanceTimersByTimeAsync(50)
    })

    // First attempt fails
    await waitFor(() => {
      expect(result.current.retry.attempt).toBe(1)
    })

    // Advance past retry delay
    await act(async () => {
      await vi.advanceTimersByTimeAsync(150)
    })

    // Second attempt fails
    await waitFor(() => {
      expect(result.current.retry.attempt).toBe(2)
    })

    // Advance past retry delay
    await act(async () => {
      await vi.advanceTimersByTimeAsync(150)
    })

    // Third attempt succeeds
    await waitFor(() => {
      expect(result.current.status).toBe(SubscriptionStatus.SUBSCRIBED)
    })

    expect(attemptCount).toBe(3)
  })

  it('should trigger manual retry', async () => {
    let attemptCount = 0
    mockClient.subscribe = vi.fn().mockImplementation(async () => {
      attemptCount++
      if (attemptCount === 1) {
        throw new Error('First attempt fails')
      }
      subscriptionCallback = vi.fn()
      return {
        id: 'sub_1',
        channelId: 1,
        topic: '/topic',
        msgType: 'test',
        unsubscribe: vi.fn(),
      }
    })

    const { result } = renderHook(
      () => useSubscription('/topic', {
        retryPolicy: { maxRetries: 0 } // No auto-retry
      }),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
      await vi.advanceTimersByTimeAsync(100)
    })

    await waitFor(() => {
      expect(result.current.status).toBe(SubscriptionStatus.ERROR)
    })

    // Manual retry
    await act(async () => {
      result.current.retryNow()
      await vi.advanceTimersByTimeAsync(100)
    })

    await waitFor(() => {
      expect(result.current.status).toBe(SubscriptionStatus.SUBSCRIBED)
    })

    expect(attemptCount).toBe(2)
  })

  it('should unsubscribe on unmount', async () => {
    const unsubscribeFn = vi.fn()
    mockClient.subscribe = vi.fn().mockResolvedValue({
      id: 'sub_1',
      channelId: 1,
      topic: '/topic',
      msgType: 'test',
      unsubscribe: unsubscribeFn,
    })

    const { unmount } = renderHook(() => useSubscription('/topic'), { wrapper })

    await act(async () => {
      mockClient._emit('connect')
      await vi.advanceTimersByTimeAsync(100)
    })

    await waitFor(() => {
      expect(mockClient.subscribe).toHaveBeenCalled()
    })

    unmount()

    expect(unsubscribeFn).toHaveBeenCalled()
  })

  it('should wait for connection', async () => {
    mockClient.isConnected = vi.fn().mockReturnValue(false)

    const { result } = renderHook(() => useSubscription('/topic'), { wrapper })

    expect(result.current.status).toBe(SubscriptionStatus.WAITING_FOR_CONNECTION)
    expect(mockClient.subscribe).not.toHaveBeenCalled()
  })
})

describe('useService', () => {
  let mockClient: RosKitClient & { _emit: (event: string, ...args: any[]) => void }

  beforeEach(() => {
    vi.useFakeTimers()

    mockClient = createMockClient({
      serviceCall: vi.fn().mockResolvedValue({ success: true, message: 'OK' }),
    }) as any

    const { RosKitClient } = require('@roskit/client')
    RosKitClient.mockImplementation(() => mockClient)
  })

  afterEach(() => {
    vi.useRealTimers()
    vi.clearAllMocks()
  })

  const wrapper = ({ children }: { children: React.ReactNode }) => (
    <RosKitProvider config={{ url: 'ws://localhost:9090' }} autoConnect>
      {children}
    </RosKitProvider>
  )

  it('should start with idle state', async () => {
    await act(async () => {
      mockClient._emit('connect')
    })

    const { result } = renderHook(
      () => useService('/service', 'std_srvs/srv/Trigger'),
      { wrapper }
    )

    expect(result.current.status).toBe(ServiceCallStatus.IDLE)
    expect(result.current.loading).toBe(false)
    expect(result.current.data).toBeNull()
    expect(result.current.error).toBeNull()
  })

  it('should call service and return result', async () => {
    const { result } = renderHook(
      () => useService<{ success: boolean; message: string }>('/trigger', 'std_srvs/srv/Trigger'),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
    })

    let callResult: any
    await act(async () => {
      callResult = await result.current.call({})
    })

    expect(callResult).toEqual({ success: true, message: 'OK' })
    expect(result.current.status).toBe(ServiceCallStatus.SUCCESS)
    expect(result.current.data).toEqual({ success: true, message: 'OK' })
    expect(result.current.loading).toBe(false)
  })

  it('should show loading state during call', async () => {
    let resolveCall: (value: any) => void
    mockClient.serviceCall = vi.fn().mockImplementation(() => {
      return new Promise(resolve => {
        resolveCall = resolve
      })
    })

    const { result } = renderHook(
      () => useService('/slow_service', 'test/srv/Slow'),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
    })

    let callPromise: Promise<any>
    act(() => {
      callPromise = result.current.call({})
    })

    // Should be loading
    expect(result.current.status).toBe(ServiceCallStatus.CALLING)
    expect(result.current.loading).toBe(true)

    await act(async () => {
      resolveCall!({ done: true })
      await callPromise!
    })

    expect(result.current.status).toBe(ServiceCallStatus.SUCCESS)
    expect(result.current.loading).toBe(false)
  })

  it('should handle service error', async () => {
    const serviceError = new Error('Service failed')
    mockClient.serviceCall = vi.fn().mockRejectedValue(serviceError)

    const { result } = renderHook(
      () => useService('/failing', 'test/srv/Fail', { retryPolicy: { maxRetries: 0 } }),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
    })

    await act(async () => {
      try {
        await result.current.call({})
      } catch (e) {
        // Expected to throw
      }
    })

    expect(result.current.status).toBe(ServiceCallStatus.ERROR)
    expect(result.current.error).toBe(serviceError)
    expect(result.current.loading).toBe(false)
  })

  it('should retry on failure', async () => {
    let attemptCount = 0
    mockClient.serviceCall = vi.fn().mockImplementation(async () => {
      attemptCount++
      if (attemptCount < 3) {
        throw new Error('Temporary error')
      }
      return { success: true }
    })

    const { result } = renderHook(
      () => useService('/flaky', 'test/srv/Flaky', {
        retryPolicy: { maxRetries: 5, initialDelayMs: 50, backoffMultiplier: 1 }
      }),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
    })

    let callResult: any
    await act(async () => {
      const promise = result.current.call({})
      // Fast-forward through retries
      await vi.advanceTimersByTimeAsync(200)
      callResult = await promise
    })

    expect(callResult).toEqual({ success: true })
    expect(attemptCount).toBe(3)
  })

  it('should reset state', async () => {
    mockClient.serviceCall = vi.fn().mockRejectedValue(new Error('Failed'))

    const { result } = renderHook(
      () => useService('/service', 'test/srv/Test', { retryPolicy: { maxRetries: 0 } }),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
    })

    await act(async () => {
      try {
        await result.current.call({})
      } catch (e) {
        // Expected
      }
    })

    expect(result.current.status).toBe(ServiceCallStatus.ERROR)

    act(() => {
      result.current.reset()
    })

    expect(result.current.status).toBe(ServiceCallStatus.IDLE)
    expect(result.current.error).toBeNull()
    expect(result.current.data).toBeNull()
  })

  it('should pass timeout to service call', async () => {
    const { result } = renderHook(
      () => useService('/service', 'test/srv/Test', { timeoutMs: 5000 }),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
    })

    await act(async () => {
      await result.current.call({ request: 'data' })
    })

    expect(mockClient.serviceCall).toHaveBeenCalledWith(
      '/service',
      'test/srv/Test',
      { request: 'data' },
      { timeoutMs: 5000 }
    )
  })

  it('should track retry state', async () => {
    let attemptCount = 0
    mockClient.serviceCall = vi.fn().mockImplementation(async () => {
      attemptCount++
      if (attemptCount < 2) {
        throw new Error('Retry needed')
      }
      return { ok: true }
    })

    const { result } = renderHook(
      () => useService('/service', 'test/srv/Test', {
        retryPolicy: { maxRetries: 3, initialDelayMs: 100, backoffMultiplier: 1 }
      }),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
    })

    let callPromise: Promise<any>
    act(() => {
      callPromise = result.current.call({})
    })

    // Wait for first failure and retry scheduling
    await act(async () => {
      await vi.advanceTimersByTimeAsync(50)
    })

    // Should show retry state
    expect(result.current.retry.attempt).toBe(1)
    expect(result.current.retry.nextRetryAt).not.toBeNull()

    // Complete the call
    await act(async () => {
      await vi.advanceTimersByTimeAsync(150)
      await callPromise!
    })

    // Retry state should be cleared on success
    expect(result.current.retry.attempt).toBe(0)
  })
})

describe('useSubscription multiple callbacks', () => {
  let mockClient: RosKitClient & { _emit: (event: string, ...args: any[]) => void }
  let callbacks: MessageCallback[] = []

  beforeEach(() => {
    vi.useFakeTimers()
    callbacks = []

    mockClient = createMockClient({
      subscribe: vi.fn().mockImplementation(async (topic: string, callback: MessageCallback) => {
        callbacks.push(callback)
        return {
          id: `sub_${callbacks.length}`,
          channelId: 1,
          topic,
          msgType: 'test',
          unsubscribe: vi.fn(),
        }
      }),
    }) as any

    const { RosKitClient } = require('@roskit/client')
    RosKitClient.mockImplementation(() => mockClient)
  })

  afterEach(() => {
    vi.useRealTimers()
    vi.clearAllMocks()
  })

  const wrapper = ({ children }: { children: React.ReactNode }) => (
    <RosKitProvider config={{ url: 'ws://localhost:9090' }} autoConnect>
      {children}
    </RosKitProvider>
  )

  it('should support multiple subscriptions to same topic', async () => {
    const onMessage1 = vi.fn()
    const onMessage2 = vi.fn()

    const { result: result1 } = renderHook(
      () => useSubscription('/topic', { onMessage: onMessage1 }),
      { wrapper }
    )

    const { result: result2 } = renderHook(
      () => useSubscription('/topic', { onMessage: onMessage2 }),
      { wrapper }
    )

    await act(async () => {
      mockClient._emit('connect')
      await vi.advanceTimersByTimeAsync(100)
    })

    // Both should have subscribed
    expect(callbacks.length).toBe(2)

    // Deliver message to both
    await act(async () => {
      callbacks.forEach(cb => cb({ msg: 'hello' }, BigInt(1)))
    })

    expect(onMessage1).toHaveBeenCalledWith({ msg: 'hello' }, BigInt(1))
    expect(onMessage2).toHaveBeenCalledWith({ msg: 'hello' }, BigInt(1))
  })
})

describe('Connection status changes', () => {
  let mockClient: RosKitClient & { _emit: (event: string, ...args: any[]) => void }

  beforeEach(() => {
    vi.useFakeTimers()

    mockClient = createMockClient() as any

    const { RosKitClient } = require('@roskit/client')
    RosKitClient.mockImplementation(() => mockClient)
  })

  afterEach(() => {
    vi.useRealTimers()
    vi.clearAllMocks()
  })

  const wrapper = ({ children }: { children: React.ReactNode }) => (
    <RosKitProvider config={{ url: 'ws://localhost:9090' }} autoConnect>
      {children}
    </RosKitProvider>
  )

  it('should resubscribe on reconnection', async () => {
    let subscriptionCallback: MessageCallback | null = null
    mockClient.subscribe = vi.fn().mockImplementation(async (topic: string, callback: MessageCallback) => {
      subscriptionCallback = callback
      return {
        id: 'sub_1',
        channelId: 1,
        topic,
        msgType: 'test',
        unsubscribe: vi.fn(),
      }
    })

    const { result } = renderHook(
      () => useSubscription('/topic'),
      { wrapper }
    )

    // Initial connection
    await act(async () => {
      mockClient._emit('connect')
      await vi.advanceTimersByTimeAsync(100)
    })

    await waitFor(() => {
      expect(mockClient.subscribe).toHaveBeenCalledTimes(1)
    })

    // Simulate disconnect
    mockClient.isConnected = vi.fn().mockReturnValue(false)
    await act(async () => {
      mockClient._emit('disconnect')
      await vi.advanceTimersByTimeAsync(10)
    })

    expect(result.current.status).toBe(SubscriptionStatus.WAITING_FOR_CONNECTION)

    // Simulate reconnect
    mockClient.isConnected = vi.fn().mockReturnValue(true)
    await act(async () => {
      mockClient._emit('connect')
      await vi.advanceTimersByTimeAsync(100)
    })

    // Should have resubscribed
    await waitFor(() => {
      expect(mockClient.subscribe).toHaveBeenCalledTimes(2)
    })
  })
})
