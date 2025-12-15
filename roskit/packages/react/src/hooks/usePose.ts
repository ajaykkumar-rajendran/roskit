/**
 * usePose - Specialized hook for Pose/PoseStamped messages
 */

import { useMemo } from 'react'
import type { PoseStamped, Pose, Point, Quaternion } from '@roskit/client'
import { useSubscription } from './useSubscription'
import { SubscriptionStatus, type RetryPolicy, type RetryState } from '../types'

interface UsePoseOptions {
  /** Only subscribe when enabled */
  enabled?: boolean
  /** Throttle rate in ms */
  throttleMs?: number
  /** Callback for each pose update */
  onPose?: (pose: PoseStamped) => void
  /** Retry policy configuration */
  retryPolicy?: RetryPolicy
  /** Callback when subscription status changes */
  onStatusChange?: (status: SubscriptionStatus) => void
}

interface UsePoseResult {
  /** Raw pose stamped data */
  poseStamped: PoseStamped | null
  /** Pose data */
  pose: Pose | null
  /** Position */
  position: Point | null
  /** Orientation as quaternion */
  orientation: Quaternion | null
  /** Yaw angle in radians */
  yaw: number | null
  /** Yaw angle in degrees */
  yawDegrees: number | null
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
 * Convert quaternion to Euler yaw angle
 */
function quaternionToYaw(q: Quaternion): number {
  // yaw (z-axis rotation)
  const siny_cosp = 2 * (q.w * q.z + q.x * q.y)
  const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
  return Math.atan2(siny_cosp, cosy_cosp)
}

/**
 * Subscribe to a Pose topic with computed yaw angle
 *
 * @example
 * ```tsx
 * function RobotMarker() {
 *   const { position, yawDegrees, status, isSubscribed } = usePose('/robot_pose')
 *
 *   if (status === SubscriptionStatus.SUBSCRIBING) return <div>Subscribing...</div>
 *   if (!position) return null
 *
 *   return (
 *     <div
 *       style={{
 *         transform: `translate(${position.x}px, ${position.y}px) rotate(${yawDegrees}deg)`,
 *       }}
 *     >
 *       Robot
 *     </div>
 *   )
 * }
 * ```
 */
export function usePose(topic: string, options: UsePoseOptions = {}): UsePoseResult {
  const { enabled = true, throttleMs = 50, onPose, retryPolicy, onStatusChange } = options

  const { data, status, isSubscribed, error, messageCount, retry, retryNow } =
    useSubscription<PoseStamped>(topic, {
      enabled,
      throttleMs,
      onMessage: onPose,
      retryPolicy,
      onStatusChange,
    })

  // Compute yaw from quaternion
  const yaw = useMemo(() => {
    if (!data?.pose?.orientation) return null
    return quaternionToYaw(data.pose.orientation)
  }, [data])

  const yawDegrees = useMemo(() => {
    if (yaw === null) return null
    return (yaw * 180) / Math.PI
  }, [yaw])

  return {
    poseStamped: data,
    pose: data?.pose ?? null,
    position: data?.pose?.position ?? null,
    orientation: data?.pose?.orientation ?? null,
    yaw,
    yawDegrees,
    status,
    isSubscribed,
    error,
    updateCount: messageCount,
    retry,
    retryNow,
  }
}
