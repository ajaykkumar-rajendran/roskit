/**
 * Unified ROS Hook (RosKit-only)
 *
 * Provides access to ROS functionality via @roskit/client.
 */

import { useCallback, useMemo } from 'react'
import { useConnectionStore } from '@/stores/connectionStore'
import type { RosKitClient } from '@roskit/client'

/** Subscription handle */
export interface UnifiedSubscription {
  id: string
  topic: string
  msgType: string
  unsubscribe: () => void
}

/** Service call options */
export interface ServiceCallOptions {
  timeoutMs?: number
}

/** Publish options */
export interface PublishOptions {
  queueSize?: number
}

/** Result type for the hook */
export interface UseRosResult {
  /** Whether a connection is active */
  isConnected: boolean
  /** Subscribe to a topic */
  subscribe: <T>(
    topic: string,
    msgType: string,
    callback: (msg: T, timestamp?: bigint) => void,
    options?: { throttleMs?: number }
  ) => Promise<UnifiedSubscription>
  /** Publish a message to a topic */
  publish: <T extends Record<string, unknown>>(
    topic: string,
    msgType: string,
    message: T,
    options?: PublishOptions
  ) => Promise<void>
  /** Call a ROS service */
  callService: <TReq extends Record<string, unknown>, TRes = unknown>(
    serviceName: string,
    serviceType: string,
    request: TReq,
    options?: ServiceCallOptions
  ) => Promise<TRes>
  /** Get RosKitClient instance */
  getClient: () => RosKitClient | null
}

/**
 * Hook that provides access to ROS functionality via RosKit.
 */
export function useRos(robotId: string | null): UseRosResult {
  const { robots, getClient: getStoreClient } = useConnectionStore()

  const robot = robotId ? robots[robotId] : null
  const client = robotId ? getStoreClient(robotId) : null

  const isConnected = robot?.status === 'connected'

  const getClient = useCallback((): RosKitClient | null => {
    return client
  }, [client])

  const subscribe = useCallback(async <T>(
    topic: string,
    msgType: string,
    callback: (msg: T, timestamp?: bigint) => void,
    options?: { throttleMs?: number }
  ): Promise<UnifiedSubscription> => {
    if (!isConnected || !client) {
      throw new Error('Not connected')
    }

    const sub = await client.subscribe<T>(
      topic,
      (msg, ts) => callback(msg, ts),
      { msgType, throttleMs: options?.throttleMs }
    )

    return {
      id: sub.id,
      topic: sub.topic,
      msgType: sub.msgType,
      unsubscribe: () => sub.unsubscribe(),
    }
  }, [isConnected, client])

  const publish = useCallback(async <T extends Record<string, unknown>>(
    topic: string,
    msgType: string,
    message: T,
    _options?: PublishOptions
  ): Promise<void> => {
    if (!isConnected || !client) {
      throw new Error('Not connected')
    }

    client.publish(topic, msgType, message)
  }, [isConnected, client])

  const callService = useCallback(async <TReq extends Record<string, unknown>, TRes = unknown>(
    serviceName: string,
    serviceType: string,
    request: TReq,
    options?: ServiceCallOptions
  ): Promise<TRes> => {
    if (!isConnected || !client) {
      throw new Error('Not connected')
    }

    return client.serviceCall<TReq, TRes>(serviceName, serviceType, request, {
      timeoutMs: options?.timeoutMs,
    })
  }, [isConnected, client])

  return useMemo(() => ({
    isConnected,
    subscribe,
    publish,
    callService,
    getClient,
  }), [isConnected, subscribe, publish, callService, getClient])
}

/**
 * Hook that provides access to ROS for the active robot.
 */
export function useActiveRos(): UseRosResult {
  const activeRobotId = useConnectionStore((s) => s.activeRobotId)
  return useRos(activeRobotId)
}
