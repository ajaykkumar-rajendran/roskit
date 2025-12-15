/**
 * usePath - Specialized hook for Path messages
 */

import { useMemo } from 'react'
import type { Path, PoseStamped, Point } from '@roskit/client'
import { useSubscription } from './useSubscription'
import { SubscriptionStatus, type RetryPolicy, type RetryState } from '../types'

interface UsePathOptions {
  /** Only subscribe when enabled */
  enabled?: boolean
  /** Throttle rate in ms */
  throttleMs?: number
  /** Callback for each path update */
  onPath?: (path: Path) => void
  /** Retry policy configuration */
  retryPolicy?: RetryPolicy
  /** Callback when subscription status changes */
  onStatusChange?: (status: SubscriptionStatus) => void
}

interface UsePathResult {
  /** Raw path data */
  path: Path | null
  /** Path poses */
  poses: PoseStamped[]
  /** Simple 2D points array for easy rendering */
  points: Point[]
  /** Path length in meters */
  length: number
  /** Subscription status */
  status: SubscriptionStatus
  /** Whether subscribed */
  isSubscribed: boolean
  /** Error if any */
  error: Error | null
  /** Update count */
  updateCount: number
  /** Retry state information */
  retry: RetryState
  /** Manually retry subscription */
  retryNow: () => void
}

/**
 * Subscribe to a Path topic with computed path length
 *
 * @example
 * ```tsx
 * function PathOverlay() {
 *   const { points, length, status, isSubscribed } = usePath('/global_plan')
 *
 *   if (status === SubscriptionStatus.SUBSCRIBING) return <div>Loading path...</div>
 *
 *   return (
 *     <svg>
 *       <polyline
 *         points={points.map(p => `${p.x},${p.y}`).join(' ')}
 *         stroke="green"
 *         fill="none"
 *       />
 *       <text>Path length: {length.toFixed(2)}m</text>
 *     </svg>
 *   )
 * }
 * ```
 */
export function usePath(topic: string, options: UsePathOptions = {}): UsePathResult {
  const { enabled = true, throttleMs = 200, onPath, retryPolicy, onStatusChange } = options

  const { data, status, isSubscribed, error, messageCount, retry, retryNow } =
    useSubscription<Path>(topic, {
      enabled,
      throttleMs,
      onMessage: onPath,
      retryPolicy,
      onStatusChange,
    })

  // Extract simple 2D points
  const points = useMemo(() => {
    if (!data?.poses) return []
    return data.poses.map((p) => p.pose.position)
  }, [data])

  // Compute path length
  const length = useMemo(() => {
    if (points.length < 2) return 0

    let total = 0
    for (let i = 1; i < points.length; i++) {
      const dx = points[i].x - points[i - 1].x
      const dy = points[i].y - points[i - 1].y
      const dz = points[i].z - points[i - 1].z
      total += Math.sqrt(dx * dx + dy * dy + dz * dz)
    }

    return total
  }, [points])

  return {
    path: data,
    poses: data?.poses ?? [],
    points,
    length,
    status,
    isSubscribed,
    error,
    updateCount: messageCount,
    retry,
    retryNow,
  }
}
