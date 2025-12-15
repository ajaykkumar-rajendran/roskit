/**
 * useOccupancyGrid - Specialized hook for OccupancyGrid (map) messages
 */

import { useMemo } from 'react'
import type { OccupancyGrid, MapMetaData } from '@roskit/client'
import { useSubscription } from './useSubscription'
import { SubscriptionStatus, type RetryPolicy, type RetryState } from '../types'

interface UseOccupancyGridOptions {
  /** Only subscribe when enabled */
  enabled?: boolean
  /** Throttle rate in ms */
  throttleMs?: number
  /** Callback for each map update */
  onMap?: (map: OccupancyGrid) => void
  /** Retry policy configuration */
  retryPolicy?: RetryPolicy
  /** Callback when subscription status changes */
  onStatusChange?: (status: SubscriptionStatus) => void
}

interface UseOccupancyGridResult {
  /** Raw occupancy grid data */
  map: OccupancyGrid | null
  /** Map metadata */
  info: MapMetaData | null
  /** Subscription status */
  status: SubscriptionStatus
  /** Whether subscribed */
  isSubscribed: boolean
  /** Error if any */
  error: Error | null
  /** Map as ImageData-compatible RGBA array (for canvas rendering) */
  imageData: Uint8ClampedArray | null
  /** Update count */
  updateCount: number
  /** Retry state information */
  retry: RetryState
  /** Manually retry subscription */
  retryNow: () => void
}

/**
 * Subscribe to an OccupancyGrid topic with pre-computed image data
 *
 * @example
 * ```tsx
 * function MapDisplay() {
 *   const { imageData, info, status, isSubscribed } = useOccupancyGrid('/map')
 *
 *   useEffect(() => {
 *     if (!imageData || !info) return
 *     const canvas = canvasRef.current
 *     const ctx = canvas.getContext('2d')
 *     const imgData = new ImageData(imageData, info.width, info.height)
 *     ctx.putImageData(imgData, 0, 0)
 *   }, [imageData, info])
 *
 *   if (status === SubscriptionStatus.SUBSCRIBING) return <div>Loading map...</div>
 *
 *   return <canvas ref={canvasRef} />
 * }
 * ```
 */
export function useOccupancyGrid(
  topic: string,
  options: UseOccupancyGridOptions = {}
): UseOccupancyGridResult {
  const { enabled = true, throttleMs = 1000, onMap, retryPolicy, onStatusChange } = options

  const { data, status, isSubscribed, error, messageCount, retry, retryNow } =
    useSubscription<OccupancyGrid>(topic, {
      enabled,
      throttleMs,
      onMessage: onMap,
      retryPolicy,
      onStatusChange,
    })

  // Convert occupancy grid to RGBA image data
  const imageData = useMemo(() => {
    if (!data) return null

    const { info } = data
    const gridData = data.data instanceof Uint8Array ? data.data : new Uint8Array(data.data)

    const rgba = new Uint8ClampedArray(info.width * info.height * 4)

    for (let i = 0; i < gridData.length; i++) {
      const value = gridData[i]
      const idx = i * 4

      if (value === 255 || value === -1) {
        // Unknown - gray
        rgba[idx] = 128
        rgba[idx + 1] = 128
        rgba[idx + 2] = 128
        rgba[idx + 3] = 255
      } else if (value === 0) {
        // Free - white/light
        rgba[idx] = 240
        rgba[idx + 1] = 240
        rgba[idx + 2] = 240
        rgba[idx + 3] = 255
      } else {
        // Occupied - map to grayscale (darker = more occupied)
        const gray = Math.floor(255 - (value / 100) * 255)
        rgba[idx] = gray
        rgba[idx + 1] = gray
        rgba[idx + 2] = gray
        rgba[idx + 3] = 255
      }
    }

    return rgba
  }, [data])

  return {
    map: data,
    info: data?.info ?? null,
    status,
    isSubscribed,
    error,
    imageData,
    updateCount: messageCount,
    retry,
    retryNow,
  }
}
