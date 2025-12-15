/**
 * useTopics - Hook for listing available ROS topics
 */

import { useState, useEffect, useCallback, useMemo } from 'react'
import type { TopicInfo } from '@roskit/client'
import { useRosKit } from '../context'

interface UseTopicsOptions {
  /** Auto-refresh interval in ms (0 to disable) */
  refreshInterval?: number
  /** Filter topics by message type */
  messageTypeFilter?: string | string[]
  /** Filter topics by name pattern */
  namePattern?: string | RegExp
}

interface UseTopicsResult {
  /** List of available topics */
  topics: TopicInfo[]
  /** Filtered topics (if filter applied) */
  filteredTopics: TopicInfo[]
  /** Loading state */
  isLoading: boolean
  /** Error if any */
  error: Error | null
  /** Manually refresh topic list */
  refresh: () => Promise<void>
}

/**
 * Get list of available ROS topics
 *
 * @example
 * ```tsx
 * function TopicList() {
 *   const { topics, isLoading, refresh } = useTopics()
 *
 *   return (
 *     <div>
 *       <button onClick={refresh}>Refresh</button>
 *       <ul>
 *         {topics.map(t => (
 *           <li key={t.name}>{t.name} - {t.msg_type}</li>
 *         ))}
 *       </ul>
 *     </div>
 *   )
 * }
 * ```
 *
 * @example
 * ```tsx
 * // Filter by message type
 * const { filteredTopics } = useTopics({
 *   messageTypeFilter: 'sensor_msgs/msg/LaserScan',
 * })
 * ```
 */
export function useTopics(options: UseTopicsOptions = {}): UseTopicsResult {
  const { refreshInterval = 0, messageTypeFilter, namePattern } = options

  const { topics: contextTopics, refreshTopics, connectionState } = useRosKit()
  const [isLoading, setIsLoading] = useState(false)
  const [error, setError] = useState<Error | null>(null)

  const refresh = useCallback(async () => {
    if (connectionState !== 'connected') {
      return
    }

    setIsLoading(true)
    setError(null)

    try {
      await refreshTopics()
    } catch (err) {
      setError(err instanceof Error ? err : new Error(String(err)))
    } finally {
      setIsLoading(false)
    }
  }, [refreshTopics, connectionState])

  // Initial load and auto-refresh
  useEffect(() => {
    if (connectionState !== 'connected') {
      return
    }

    // Initial fetch
    refresh()

    // Set up auto-refresh
    if (refreshInterval > 0) {
      const intervalId = setInterval(refresh, refreshInterval)
      return () => clearInterval(intervalId)
    }
  }, [connectionState, refreshInterval, refresh])

  const filteredTopics = useMemo(() => {
    let result = contextTopics

    if (messageTypeFilter) {
      const types = Array.isArray(messageTypeFilter) ? messageTypeFilter : [messageTypeFilter]
      result = result.filter((topic) => types.some((filter) => topic.msg_type.includes(filter)))
    }

    if (namePattern) {
      const pattern = typeof namePattern === 'string' ? new RegExp(namePattern) : namePattern
      result = result.filter((topic) => pattern.test(topic.name))
    }

    return result
  }, [contextTopics, messageTypeFilter, namePattern])

  return {
    topics: contextTopics,
    filteredTopics,
    isLoading,
    error,
    refresh,
  }
}
