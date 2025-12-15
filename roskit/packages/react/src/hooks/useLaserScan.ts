/**
 * useLaserScan - Specialized hook for LaserScan messages
 */

import { useMemo } from 'react'
import type { LaserScan } from '@roskit/client'
import { useSubscription } from './useSubscription'
import { SubscriptionStatus, type RetryPolicy, type RetryState } from '../types'

interface UseLaserScanOptions {
  /** Only subscribe when enabled */
  enabled?: boolean
  /** Throttle rate in ms */
  throttleMs?: number
  /** Callback for each scan */
  onScan?: (scan: LaserScan) => void
  /** Retry policy configuration */
  retryPolicy?: RetryPolicy
  /** Callback when subscription status changes */
  onStatusChange?: (status: SubscriptionStatus) => void
}

interface UseLaserScanResult {
  /** Raw laser scan data */
  scan: LaserScan | null
  /** Subscription status */
  status: SubscriptionStatus
  /** Whether subscribed */
  isSubscribed: boolean
  /** Error if any */
  error: Error | null
  /** Cartesian points computed from scan */
  points: { x: number; y: number; intensity?: number }[]
  /** Scan count */
  scanCount: number
  /** Retry state information */
  retry: RetryState
  /** Manually retry subscription */
  retryNow: () => void
}

/**
 * Subscribe to a LaserScan topic with computed Cartesian points
 *
 * @example
 * ```tsx
 * function LidarDisplay() {
 *   const { points, status, isSubscribed } = useLaserScan('/scan')
 *
 *   if (status === SubscriptionStatus.SUBSCRIBING) return <div>Subscribing...</div>
 *
 *   return (
 *     <svg viewBox="-10 -10 20 20">
 *       {points.map((p, i) => (
 *         <circle key={i} cx={p.x} cy={p.y} r={0.05} fill="red" />
 *       ))}
 *     </svg>
 *   )
 * }
 * ```
 */
export function useLaserScan(topic: string, options: UseLaserScanOptions = {}): UseLaserScanResult {
  const { enabled = true, throttleMs = 100, onScan, retryPolicy, onStatusChange } = options

  const { data, status, isSubscribed, error, messageCount, retry, retryNow } =
    useSubscription<LaserScan>(topic, {
      enabled,
      throttleMs,
      onMessage: onScan,
      retryPolicy,
      onStatusChange,
    })

  // Compute Cartesian points from polar coordinates
  const points = useMemo(() => {
    if (!data) return []

    const result: { x: number; y: number; intensity?: number }[] = []
    const ranges = data.ranges instanceof Float32Array ? data.ranges : new Float32Array(data.ranges)
    const intensities =
      data.intensities instanceof Float32Array
        ? data.intensities
        : data.intensities?.length
          ? new Float32Array(data.intensities)
          : null

    for (let i = 0; i < ranges.length; i++) {
      const range = ranges[i]

      // Skip invalid ranges
      if (range < data.range_min || range > data.range_max || !isFinite(range)) {
        continue
      }

      const angle = data.angle_min + i * data.angle_increment
      const x = range * Math.cos(angle)
      const y = range * Math.sin(angle)

      const point: { x: number; y: number; intensity?: number } = { x, y }
      if (intensities && i < intensities.length) {
        point.intensity = intensities[i]
      }

      result.push(point)
    }

    return result
  }, [data])

  return {
    scan: data,
    status,
    isSubscribed,
    error,
    points,
    scanCount: messageCount,
    retry,
    retryNow,
  }
}
