/**
 * useSubscription - Subscribe to ROS topics with React state management
 */

import { useState, useEffect, useCallback, useRef } from 'react'
import type { Subscription, SubscriptionOptions, MessageCallback } from '@roskit/client'
import { useRosKit, useRosKitClient } from '../context'
import {
  SubscriptionStatus,
  type RetryPolicy,
  type RetryState,
  DEFAULT_SUBSCRIPTION_RETRY_POLICY,
  calculateRetryDelay,
  shouldRetry,
} from '../types'

interface UseSubscriptionOptions<T> extends SubscriptionOptions {
  /** Only subscribe when enabled (default: true) */
  enabled?: boolean
  /** Callback for each message */
  onMessage?: MessageCallback<T>
  /** Retry policy configuration */
  retryPolicy?: RetryPolicy
  /** Callback when subscription status changes */
  onStatusChange?: (status: SubscriptionStatus) => void
}

interface UseSubscriptionResult<T> {
  /** Latest message data */
  data: T | null
  /** Subscription status */
  status: SubscriptionStatus
  /** Whether subscription is active (status === SUBSCRIBED) */
  isSubscribed: boolean
  /** Subscription error */
  error: Error | null
  /** Message timestamp */
  timestamp: bigint | null
  /** Message count received */
  messageCount: number
  /** Retry state information */
  retry: RetryState
  /** Manually retry subscription */
  retryNow: () => void
}

/**
 * Subscribe to a ROS topic and receive messages as React state
 *
 * @example
 * ```tsx
 * function LaserScanDisplay() {
 *   const { data, status, isSubscribed } = useSubscription<LaserScan>('/scan')
 *
 *   if (status === SubscriptionStatus.SUBSCRIBING) return <div>Subscribing...</div>
 *   if (status === SubscriptionStatus.ERROR) return <div>Subscription failed</div>
 *   if (!data) return <div>Waiting for data...</div>
 *   return <div>Received {data.ranges.length} points</div>
 * }
 * ```
 *
 * @example
 * ```tsx
 * // With retry policy
 * const { data, retry } = useSubscription<OccupancyGrid>('/map', {
 *   throttleMs: 1000,
 *   retryPolicy: { maxRetries: 5, initialDelayMs: 500 },
 *   onStatusChange: (status) => console.log('Status:', status),
 * })
 *
 * if (retry.attempt > 0) {
 *   console.log(`Retry ${retry.attempt}, next in ${retry.retryInMs}ms`)
 * }
 * ```
 */
export function useSubscription<T = unknown>(
  topic: string,
  options: UseSubscriptionOptions<T> = {}
): UseSubscriptionResult<T> {
  const {
    enabled = true,
    onMessage,
    retryPolicy: userRetryPolicy,
    onStatusChange,
    ...subscriptionOptions
  } = options

  // Merge user retry policy with defaults
  const retryPolicy: Required<RetryPolicy> = {
    ...DEFAULT_SUBSCRIPTION_RETRY_POLICY,
    ...userRetryPolicy,
  }

  const { connectionState } = useRosKit()
  const client = useRosKitClient()
  const [data, setData] = useState<T | null>(null)
  const [status, setStatus] = useState<SubscriptionStatus>(
    enabled ? SubscriptionStatus.WAITING_FOR_CONNECTION : SubscriptionStatus.DISABLED
  )
  const [error, setError] = useState<Error | null>(null)
  const [timestamp, setTimestamp] = useState<bigint | null>(null)
  const [messageCount, setMessageCount] = useState(0)
  const [retryState, setRetryState] = useState<RetryState>({
    attempt: 0,
    nextRetryAt: null,
    retryInMs: null,
  })

  const subscriptionRef = useRef<Subscription | null>(null)
  const onMessageRef = useRef(onMessage)
  const onStatusChangeRef = useRef(onStatusChange)
  const retryTimeoutRef = useRef<ReturnType<typeof setTimeout> | null>(null)
  const manualRetryRef = useRef<(() => void) | null>(null)

  // Keep callback refs up to date
  useEffect(() => {
    onMessageRef.current = onMessage
  }, [onMessage])

  useEffect(() => {
    onStatusChangeRef.current = onStatusChange
  }, [onStatusChange])

  // Update status and notify
  const updateStatus = useCallback((newStatus: SubscriptionStatus) => {
    setStatus(newStatus)
    onStatusChangeRef.current?.(newStatus)
  }, [])

  // Message handler
  const handleMessage: MessageCallback<T> = useCallback((message, ts) => {
    setData(message)
    setTimestamp(ts)
    setMessageCount((c) => c + 1)
    onMessageRef.current?.(message, ts)
  }, [])

  // Manual retry function
  const retryNow = useCallback(() => {
    manualRetryRef.current?.()
  }, [])

  // Update retry countdown
  useEffect(() => {
    if (retryState.nextRetryAt === null) {
      return
    }

    const updateCountdown = () => {
      const remaining = Math.max(0, retryState.nextRetryAt! - Date.now())
      setRetryState((prev) => ({ ...prev, retryInMs: remaining }))
    }

    updateCountdown()
    const interval = setInterval(updateCountdown, 100)
    return () => clearInterval(interval)
  }, [retryState.nextRetryAt])

  // Subscribe/unsubscribe effect
  useEffect(() => {
    // Clear any pending retry
    if (retryTimeoutRef.current) {
      clearTimeout(retryTimeoutRef.current)
      retryTimeoutRef.current = null
    }

    if (!enabled) {
      updateStatus(SubscriptionStatus.DISABLED)
      setRetryState({ attempt: 0, nextRetryAt: null, retryInMs: null })
      return
    }

    if (connectionState !== 'connected' || !client.isConnected()) {
      updateStatus(SubscriptionStatus.WAITING_FOR_CONNECTION)
      setRetryState({ attempt: 0, nextRetryAt: null, retryInMs: null })
      return
    }

    let cancelled = false
    let currentAttempt = 0

    const subscribe = async () => {
      if (cancelled) return

      try {
        updateStatus(
          currentAttempt > 0 ? SubscriptionStatus.RETRYING : SubscriptionStatus.SUBSCRIBING
        )

        const sub = await client.subscribe<T>(topic, handleMessage, subscriptionOptions)

        if (cancelled) {
          sub.unsubscribe()
          return
        }

        subscriptionRef.current = sub
        updateStatus(SubscriptionStatus.SUBSCRIBED)
        setError(null)
        setRetryState({ attempt: 0, nextRetryAt: null, retryInMs: null })
      } catch (err) {
        if (cancelled) {
          client.clearSavedSubscription(topic, handleMessage as MessageCallback)
          return
        }

        const error = err instanceof Error ? err : new Error(String(err))
        setError(error)
        client.clearSavedSubscription(topic, handleMessage as MessageCallback)

        // Check if we should retry
        if (shouldRetry(error, retryPolicy, currentAttempt)) {
          const delay = calculateRetryDelay(currentAttempt, retryPolicy)
          const nextRetryAt = Date.now() + delay
          currentAttempt++

          setRetryState({
            attempt: currentAttempt,
            nextRetryAt,
            retryInMs: delay,
          })
          updateStatus(SubscriptionStatus.RETRYING)

          retryTimeoutRef.current = setTimeout(subscribe, delay)
        } else {
          updateStatus(SubscriptionStatus.ERROR)
          setRetryState((prev) => ({ ...prev, nextRetryAt: null, retryInMs: null }))
        }
      }
    }

    // Store manual retry function
    manualRetryRef.current = () => {
      if (retryTimeoutRef.current) {
        clearTimeout(retryTimeoutRef.current)
        retryTimeoutRef.current = null
      }
      // Reset attempt count on manual retry
      currentAttempt = 0
      setRetryState({ attempt: 0, nextRetryAt: null, retryInMs: null })
      subscribe()
    }

    subscribe()

    return () => {
      cancelled = true
      manualRetryRef.current = null

      if (retryTimeoutRef.current) {
        clearTimeout(retryTimeoutRef.current)
        retryTimeoutRef.current = null
      }

      if (subscriptionRef.current) {
        subscriptionRef.current.unsubscribe()
        subscriptionRef.current = null
      } else {
        // If subscription never established, clear any saved callbacks
        client.clearSavedSubscription(topic, handleMessage as MessageCallback)
      }

      // Don't update status on cleanup - let next effect set appropriate status
    }
  }, [
    topic,
    enabled,
    client,
    connectionState,
    handleMessage,
    updateStatus,
    subscriptionOptions.msgType,
    subscriptionOptions.throttleMs,
    subscriptionOptions.clientThrottleMs,
    retryPolicy.maxRetries,
    retryPolicy.initialDelayMs,
    retryPolicy.maxDelayMs,
    retryPolicy.backoffMultiplier,
  ])

  return {
    data,
    status,
    isSubscribed: status === SubscriptionStatus.SUBSCRIBED,
    error,
    timestamp,
    messageCount,
    retry: retryState,
    retryNow,
  }
}

/**
 * Subscribe to a topic with automatic JSON parsing
 * Useful for custom message types
 */
export function useJsonSubscription<T = unknown>(
  topic: string,
  options: UseSubscriptionOptions<T> = {}
): UseSubscriptionResult<T> {
  return useSubscription<T>(topic, options)
}
