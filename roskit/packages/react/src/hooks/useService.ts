/**
 * useService - Call a ROS service with status tracking and retry support
 */

import { useCallback, useState, useRef } from 'react'
import { useRosKit } from '../context'
import {
  ServiceCallStatus,
  type RetryPolicy,
  type RetryState,
  DEFAULT_SERVICE_RETRY_POLICY,
  calculateRetryDelay,
  shouldRetry,
} from '../types'

interface UseServiceOptions {
  /** Timeout in milliseconds */
  timeoutMs?: number
  /** Retry policy configuration */
  retryPolicy?: RetryPolicy
  /** Callback when status changes */
  onStatusChange?: (status: ServiceCallStatus) => void
}

interface UseServiceResult<TRes> {
  /** Call the service with a request */
  call: <TReq extends Record<string, unknown>>(request: TReq) => Promise<TRes>
  /** Service call status */
  status: ServiceCallStatus
  /** Whether a call is in progress (status === CALLING) */
  loading: boolean
  /** Last error if any */
  error: Error | null
  /** Last successful response */
  data: TRes | null
  /** Retry state information */
  retry: RetryState
  /** Reset state to idle */
  reset: () => void
}

/**
 * useService - Call a ROS service with status tracking and optional retry
 *
 * @example
 * ```tsx
 * function TriggerButton() {
 *   const { call, status, error } = useService<TriggerResponse>(
 *     '/robot/trigger',
 *     'std_srvs/srv/Trigger'
 *   )
 *
 *   const handleClick = async () => {
 *     try {
 *       const response = await call({})
 *       console.log('Success:', response.message)
 *     } catch (err) {
 *       console.error('Failed:', err)
 *     }
 *   }
 *
 *   return (
 *     <button onClick={handleClick} disabled={status === ServiceCallStatus.CALLING}>
 *       {status === ServiceCallStatus.CALLING ? 'Calling...' : 'Trigger'}
 *     </button>
 *   )
 * }
 * ```
 *
 * @example
 * ```tsx
 * // With retry policy
 * const { call, retry } = useService<SetBoolResponse>(
 *   '/robot/enable',
 *   'std_srvs/srv/SetBool',
 *   {
 *     timeoutMs: 5000,
 *     retryPolicy: { maxRetries: 3, initialDelayMs: 500 },
 *     onStatusChange: (status) => console.log('Status:', status),
 *   }
 * )
 * ```
 */
export function useService<TRes = unknown>(
  service: string,
  msgType: string,
  options: UseServiceOptions = {}
): UseServiceResult<TRes> {
  const { timeoutMs, retryPolicy: userRetryPolicy, onStatusChange } = options

  // Merge user retry policy with defaults
  const retryPolicy: Required<RetryPolicy> = {
    ...DEFAULT_SERVICE_RETRY_POLICY,
    ...userRetryPolicy,
  }

  const { callService } = useRosKit()
  const [status, setStatus] = useState<ServiceCallStatus>(ServiceCallStatus.IDLE)
  const [error, setError] = useState<Error | null>(null)
  const [data, setData] = useState<TRes | null>(null)
  const [retryState, setRetryState] = useState<RetryState>({
    attempt: 0,
    nextRetryAt: null,
    retryInMs: null,
  })

  const onStatusChangeRef = useRef(onStatusChange)
  onStatusChangeRef.current = onStatusChange

  // Update status and notify
  const updateStatus = useCallback((newStatus: ServiceCallStatus) => {
    setStatus(newStatus)
    onStatusChangeRef.current?.(newStatus)
  }, [])

  const reset = useCallback(() => {
    updateStatus(ServiceCallStatus.IDLE)
    setError(null)
    setData(null)
    setRetryState({ attempt: 0, nextRetryAt: null, retryInMs: null })
  }, [updateStatus])

  const call = useCallback(
    async <TReq extends Record<string, unknown>>(request: TReq): Promise<TRes> => {
      updateStatus(ServiceCallStatus.CALLING)
      setError(null)
      setRetryState({ attempt: 0, nextRetryAt: null, retryInMs: null })

      let currentAttempt = 0
      let lastError: Error | null = null

      const attemptCall = async (): Promise<TRes> => {
        try {
          const res = await callService<TReq, TRes>(service, msgType, request, { timeoutMs })
          updateStatus(ServiceCallStatus.SUCCESS)
          setData(res)
          setRetryState({ attempt: 0, nextRetryAt: null, retryInMs: null })
          return res
        } catch (err) {
          lastError = err instanceof Error ? err : new Error(String(err))

          // Check if we should retry
          if (shouldRetry(lastError, retryPolicy, currentAttempt)) {
            const delay = calculateRetryDelay(currentAttempt, retryPolicy)
            currentAttempt++

            setRetryState({
              attempt: currentAttempt,
              nextRetryAt: Date.now() + delay,
              retryInMs: delay,
            })

            // Wait for delay then retry
            await new Promise((resolve) => setTimeout(resolve, delay))
            return attemptCall()
          }

          // No more retries
          updateStatus(ServiceCallStatus.ERROR)
          setError(lastError)
          setRetryState((prev) => ({ ...prev, nextRetryAt: null, retryInMs: null }))
          throw lastError
        }
      }

      return attemptCall()
    },
    [callService, msgType, timeoutMs, service, retryPolicy, updateStatus]
  )

  return {
    call,
    status,
    loading: status === ServiceCallStatus.CALLING,
    error,
    data,
    retry: retryState,
    reset,
  }
}
