/**
 * Types and Exports Tests
 *
 * Verify that all types and exports are correctly exposed from the package.
 */

import { describe, it, expect } from 'vitest'
import {
  // Status enums
  SubscriptionStatus,
  ServiceCallStatus,
  ConnectionStatus,
  // Retry utilities
  DEFAULT_SUBSCRIPTION_RETRY_POLICY,
  DEFAULT_SERVICE_RETRY_POLICY,
  calculateRetryDelay,
  shouldRetry,
} from '../types'
import type { RetryPolicy, RetryState } from '../types'

describe('SubscriptionStatus Enum', () => {
  it('should have all expected values', () => {
    expect(SubscriptionStatus.IDLE).toBe('idle')
    expect(SubscriptionStatus.SUBSCRIBING).toBe('subscribing')
    expect(SubscriptionStatus.SUBSCRIBED).toBe('subscribed')
    expect(SubscriptionStatus.ERROR).toBe('error')
    expect(SubscriptionStatus.RETRYING).toBe('retrying')
    expect(SubscriptionStatus.DISABLED).toBe('disabled')
    expect(SubscriptionStatus.WAITING_FOR_CONNECTION).toBe('waiting_for_connection')
  })

  it('should have exactly 7 status values', () => {
    const values = Object.values(SubscriptionStatus)
    expect(values).toHaveLength(7)
  })
})

describe('ServiceCallStatus Enum', () => {
  it('should have all expected values', () => {
    expect(ServiceCallStatus.IDLE).toBe('idle')
    expect(ServiceCallStatus.CALLING).toBe('calling')
    expect(ServiceCallStatus.SUCCESS).toBe('success')
    expect(ServiceCallStatus.ERROR).toBe('error')
  })

  it('should have exactly 4 status values', () => {
    const values = Object.values(ServiceCallStatus)
    expect(values).toHaveLength(4)
  })
})

describe('ConnectionStatus Enum', () => {
  it('should have all expected values', () => {
    expect(ConnectionStatus.DISCONNECTED).toBe('disconnected')
    expect(ConnectionStatus.CONNECTING).toBe('connecting')
    expect(ConnectionStatus.CONNECTED).toBe('connected')
    expect(ConnectionStatus.ERROR).toBe('error')
    expect(ConnectionStatus.RECONNECTING).toBe('reconnecting')
  })

  it('should have exactly 5 status values', () => {
    const values = Object.values(ConnectionStatus)
    expect(values).toHaveLength(5)
  })
})

describe('Default Retry Policies', () => {
  it('should have sensible defaults for subscription retry', () => {
    expect(DEFAULT_SUBSCRIPTION_RETRY_POLICY.maxRetries).toBe(3)
    expect(DEFAULT_SUBSCRIPTION_RETRY_POLICY.initialDelayMs).toBeGreaterThan(0)
    expect(DEFAULT_SUBSCRIPTION_RETRY_POLICY.maxDelayMs).toBeGreaterThan(DEFAULT_SUBSCRIPTION_RETRY_POLICY.initialDelayMs)
    expect(DEFAULT_SUBSCRIPTION_RETRY_POLICY.backoffMultiplier).toBeGreaterThan(1)
    expect(DEFAULT_SUBSCRIPTION_RETRY_POLICY.jitter).toBeGreaterThanOrEqual(0)
    expect(DEFAULT_SUBSCRIPTION_RETRY_POLICY.jitter).toBeLessThanOrEqual(1)
  })

  it('should have sensible defaults for service retry (no retries by default)', () => {
    expect(DEFAULT_SERVICE_RETRY_POLICY.maxRetries).toBe(0)
    expect(DEFAULT_SERVICE_RETRY_POLICY.initialDelayMs).toBeGreaterThan(0)
  })
})

describe('calculateRetryDelay', () => {
  const policy: Required<RetryPolicy> = {
    maxRetries: 5,
    initialDelayMs: 1000,
    maxDelayMs: 10000,
    backoffMultiplier: 2,
    jitter: 0, // No jitter for predictable tests
    retryOn: ['all'],
  }

  it('should return initial delay for first attempt', () => {
    const delay = calculateRetryDelay(0, policy)
    expect(delay).toBe(1000)
  })

  it('should apply exponential backoff', () => {
    const delay0 = calculateRetryDelay(0, policy)
    const delay1 = calculateRetryDelay(1, policy)
    const delay2 = calculateRetryDelay(2, policy)

    expect(delay0).toBe(1000)
    expect(delay1).toBe(2000)
    expect(delay2).toBe(4000)
  })

  it('should cap delay at maxDelayMs', () => {
    const delay10 = calculateRetryDelay(10, policy)
    expect(delay10).toBe(10000)
  })

  it('should apply jitter when configured', () => {
    const policyWithJitter: Required<RetryPolicy> = {
      ...policy,
      jitter: 0.5,
    }

    // Run multiple times to verify jitter is applied
    const delays = new Set<number>()
    for (let i = 0; i < 10; i++) {
      delays.add(calculateRetryDelay(0, policyWithJitter))
    }

    // With 50% jitter, we should see some variation
    // All values should be between 500 and 1500
    for (const delay of delays) {
      expect(delay).toBeGreaterThanOrEqual(500)
      expect(delay).toBeLessThanOrEqual(1500)
    }
  })
})

describe('shouldRetry', () => {
  const policy: Required<RetryPolicy> = {
    maxRetries: 3,
    initialDelayMs: 1000,
    maxDelayMs: 10000,
    backoffMultiplier: 2,
    jitter: 0,
    retryOn: ['network', 'timeout'],
  }

  it('should not retry if max retries exceeded', () => {
    const error = new Error('Network error')
    expect(shouldRetry(error, policy, 3)).toBe(false)
    expect(shouldRetry(error, policy, 10)).toBe(false)
  })

  it('should retry network errors when configured', () => {
    const error = new Error('WebSocket connection failed')
    expect(shouldRetry(error, policy, 0)).toBe(true)
  })

  it('should retry timeout errors when configured', () => {
    const error = new Error('Request timed out')
    expect(shouldRetry(error, policy, 0)).toBe(true)
  })

  it('should not retry non-matching errors', () => {
    const error = new Error('Invalid data format')
    expect(shouldRetry(error, policy, 0)).toBe(false)
  })

  it('should retry all errors when configured with "all"', () => {
    const allPolicy: Required<RetryPolicy> = {
      ...policy,
      retryOn: ['all'],
    }

    expect(shouldRetry(new Error('Any error'), allPolicy, 0)).toBe(true)
    expect(shouldRetry(new Error('Completely random'), allPolicy, 0)).toBe(true)
  })

  it('should respect RosKitError kind', () => {
    const networkError = new Error('Network error') as any
    networkError.kind = 'network'

    const serverError = new Error('Server error') as any
    serverError.kind = 'server'

    expect(shouldRetry(networkError, policy, 0)).toBe(true)
    expect(shouldRetry(serverError, policy, 0)).toBe(false)
  })
})

describe('RetryState Type', () => {
  it('should have correct shape', () => {
    const state: RetryState = {
      attempt: 0,
      nextRetryAt: null,
      retryInMs: null,
    }

    expect(state.attempt).toBe(0)
    expect(state.nextRetryAt).toBeNull()
    expect(state.retryInMs).toBeNull()
  })

  it('should allow numeric values for retry timing', () => {
    const state: RetryState = {
      attempt: 3,
      nextRetryAt: Date.now() + 5000,
      retryInMs: 5000,
    }

    expect(state.attempt).toBe(3)
    expect(state.nextRetryAt).toBeGreaterThan(0)
    expect(state.retryInMs).toBe(5000)
  })
})
