/**
 * RosKit React Context
 *
 * Provides RosKitClient instance to the React component tree.
 */

import { createContext, useContext, useEffect, useState, useCallback, useMemo, type ReactNode } from 'react'
import {
  RosKitClient,
  type RosKitClientConfig,
  type ServerInfo,
  type TopicInfo,
} from '@roskit/client'
import { ConnectionStatus } from './types'

/** Reconnect info exposed to consumers */
interface ReconnectInfo {
  /** Current reconnect attempt number (1-based, 0 if not reconnecting) */
  attempt: number
  /** Delay until next attempt in ms (null if not reconnecting) */
  delayMs: number | null
  /** Time until next reconnect in ms (counts down) */
  remainingMs: number | null
}

// Context value type
interface RosKitContextValue {
  client: RosKitClient | null
  /** Connection status with reconnecting state */
  connectionStatus: ConnectionStatus
  /** Legacy connection state for backward compatibility */
  connectionState: 'disconnected' | 'connecting' | 'connected' | 'error'
  /** Reconnection information */
  reconnect: ReconnectInfo
  serverInfo: ServerInfo | null
  topics: TopicInfo[]
  error: Error | null
  connect: () => Promise<void>
  disconnect: () => void
  refreshTopics: () => Promise<void>
  publish: <T extends Record<string, unknown>>(topic: string, msgType: string, data: T) => void
  callService: <TReq extends Record<string, unknown>, TRes = unknown>(
    service: string,
    msgType: string,
    request: TReq,
    options?: { timeoutMs?: number }
  ) => Promise<TRes>
}

// Create context with null default
const RosKitContext = createContext<RosKitContextValue | null>(null)

// Provider props
interface RosKitProviderProps {
  children: ReactNode
  config: RosKitClientConfig
  autoConnect?: boolean
  /** Callback when connection status changes */
  onStatusChange?: (status: ConnectionStatus) => void
}

/**
 * RosKitProvider - Provides RosKitClient to component tree
 *
 * @example
 * ```tsx
 * <RosKitProvider config={{ url: 'ws://localhost:9091' }} autoConnect>
 *   <App />
 * </RosKitProvider>
 * ```
 *
 * @example
 * ```tsx
 * // With status change callback
 * <RosKitProvider
 *   config={{ url: 'ws://localhost:9091', autoReconnect: true }}
 *   autoConnect
 *   onStatusChange={(status) => {
 *     if (status === ConnectionStatus.RECONNECTING) {
 *       console.log('Connection lost, reconnecting...')
 *     }
 *   }}
 * >
 *   <App />
 * </RosKitProvider>
 * ```
 */
export function RosKitProvider({ children, config, autoConnect = false, onStatusChange }: RosKitProviderProps) {
  const client = useMemo(
    () => new RosKitClient(config),
    [
      config.url,
      config.autoReconnect,
      config.reconnectDelay,
      config.maxReconnectAttempts,
      config.maxReconnectDelay,
      config.connectionTimeout,
      config.subscriptionTimeoutMs,
      JSON.stringify(config.protocolPreference ?? []),
    ]
  )
  const [connectionStatus, setConnectionStatus] = useState<ConnectionStatus>(ConnectionStatus.DISCONNECTED)
  const [reconnectInfo, setReconnectInfo] = useState<ReconnectInfo>({
    attempt: 0,
    delayMs: null,
    remainingMs: null,
  })
  const [serverInfo, setServerInfo] = useState<ServerInfo | null>(null)
  const [topics, setTopics] = useState<TopicInfo[]>([])
  const [error, setError] = useState<Error | null>(null)

  // Callback ref for status change
  const onStatusChangeRef = useCallback((status: ConnectionStatus) => {
    setConnectionStatus(status)
    onStatusChange?.(status)
  }, [onStatusChange])

  // Update reconnect countdown
  useEffect(() => {
    if (reconnectInfo.delayMs === null || connectionStatus !== ConnectionStatus.RECONNECTING) {
      return
    }

    const startTime = Date.now()
    const targetTime = startTime + reconnectInfo.delayMs

    const updateCountdown = () => {
      const remaining = Math.max(0, targetTime - Date.now())
      setReconnectInfo((prev) => ({ ...prev, remainingMs: remaining }))
    }

    updateCountdown()
    const interval = setInterval(updateCountdown, 100)
    return () => clearInterval(interval)
  }, [reconnectInfo.delayMs, connectionStatus])

  // Set up event listeners
  useEffect(() => {
    const handleConnect = () => {
      onStatusChangeRef(ConnectionStatus.CONNECTED)
      setReconnectInfo({ attempt: 0, delayMs: null, remainingMs: null })
      setError(null)
    }

    const handleDisconnect = () => {
      // Only set to disconnected if not already reconnecting
      // (reconnecting event comes before disconnect)
      setConnectionStatus((prev) => {
        if (prev === ConnectionStatus.RECONNECTING) {
          return prev
        }
        onStatusChange?.(ConnectionStatus.DISCONNECTED)
        return ConnectionStatus.DISCONNECTED
      })
    }

    const handleReconnecting = (attempt: number, delay: number) => {
      onStatusChangeRef(ConnectionStatus.RECONNECTING)
      setReconnectInfo({
        attempt,
        delayMs: delay,
        remainingMs: delay,
      })
    }

    const handleError = (err: Error) => {
      onStatusChangeRef(ConnectionStatus.ERROR)
      setError(err)
    }

    const handleServerInfo = (info: ServerInfo) => {
      setServerInfo(info)
    }

    const handleTopicList = (topicList: TopicInfo[]) => {
      setTopics(topicList)
    }

    client.on('connect', handleConnect)
    client.on('disconnect', handleDisconnect)
    client.on('reconnecting', handleReconnecting)
    client.on('error', handleError)
    client.on('serverInfo', handleServerInfo)
    client.on('topicList', handleTopicList)

    return () => {
      client.off('connect', handleConnect)
      client.off('disconnect', handleDisconnect)
      client.off('reconnecting', handleReconnecting)
      client.off('error', handleError)
      client.off('serverInfo', handleServerInfo)
      client.off('topicList', handleTopicList)
    }
  }, [client, onStatusChangeRef, onStatusChange])

  // Auto-connect on mount
  useEffect(() => {
    if (autoConnect) {
      client.connect().catch(setError)
    }

    return () => {
      client.disconnect()
    }
  }, [client, autoConnect])

  const connect = useCallback(async () => {
    onStatusChangeRef(ConnectionStatus.CONNECTING)
    try {
      await client.connect()
    } catch (err) {
      setError(err instanceof Error ? err : new Error(String(err)))
      throw err
    }
  }, [client, onStatusChangeRef])

  const disconnect = useCallback(() => {
    client.disconnect()
  }, [client])

  const refreshTopics = useCallback(async () => {
    const topicList = await client.getTopics()
    setTopics(topicList)
  }, [client])

  const publish = useCallback(<T extends Record<string, unknown>>(topic: string, msgType: string, data: T) => {
    client.publish(topic, msgType, data)
  }, [client])

  const callService = useCallback(
    <TReq extends Record<string, unknown>, TRes = unknown>(
      service: string,
      msgType: string,
      request: TReq,
      options?: { timeoutMs?: number }
    ) => client.serviceCall<TReq, TRes>(service, msgType, request, options),
    [client]
  )

  // Map ConnectionStatus to legacy ConnectionState for backward compatibility
  const connectionState = useMemo(() => {
    switch (connectionStatus) {
      case ConnectionStatus.DISCONNECTED:
        return 'disconnected'
      case ConnectionStatus.CONNECTING:
        return 'connecting'
      case ConnectionStatus.CONNECTED:
        return 'connected'
      case ConnectionStatus.RECONNECTING:
        return 'connecting' // Map reconnecting to connecting for legacy
      case ConnectionStatus.ERROR:
        return 'error'
      default:
        return 'disconnected'
    }
  }, [connectionStatus])

  const value: RosKitContextValue = {
    client,
    connectionStatus,
    connectionState,
    reconnect: reconnectInfo,
    serverInfo,
    topics,
    error,
    connect,
    disconnect,
    refreshTopics,
    publish,
    callService,
  }

  return <RosKitContext.Provider value={value}>{children}</RosKitContext.Provider>
}

/**
 * useRosKit - Access RosKit context
 *
 * @example
 * ```tsx
 * function MyComponent() {
 *   const { connectionStatus, connect, reconnect } = useRosKit()
 *
 *   if (connectionStatus === ConnectionStatus.RECONNECTING) {
 *     return <div>Reconnecting... attempt {reconnect.attempt}</div>
 *   }
 *
 *   return <button onClick={connect}>Connect ({connectionStatus})</button>
 * }
 * ```
 */
export function useRosKit(): RosKitContextValue {
  const context = useContext(RosKitContext)
  if (!context) {
    throw new Error('useRosKit must be used within a RosKitProvider')
  }
  return context
}

/**
 * useRosKitClient - Access raw RosKitClient instance
 *
 * @example
 * ```tsx
 * function MyComponent() {
 *   const client = useRosKitClient()
 *   // Use client directly for advanced operations
 * }
 * ```
 */
export function useRosKitClient(): RosKitClient {
  const { client } = useRosKit()
  if (!client) {
    throw new Error('RosKitClient not available')
  }
  return client
}
