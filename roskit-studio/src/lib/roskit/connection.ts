import { RosKitClient, type RosKitClientConfig } from '@roskit/client'

export interface RoskitConnection {
  client: RosKitClient
  url: string
  cleanup: () => void
}

export interface RoskitConnectionOptions {
  /** Auto-reconnect on disconnect (default: true) */
  autoReconnect?: boolean
  /** Connection timeout in ms (default: 10000) */
  connectionTimeout?: number
  /** Enable worker-based decoding for images/LaserScan (default: true) */
  useWorkerDecode?: boolean
  /** Minimum payload size to use worker decode in bytes (default: 10000) */
  workerDecodeThreshold?: number
}

/**
 * Create a RosKit client connection with sensible defaults for Studio.
 */
export function createRoskitConnection(
  url: string,
  options: RoskitConnectionOptions = {}
): RoskitConnection {
  const config: RosKitClientConfig = {
    url,
    autoReconnect: options.autoReconnect ?? true,
    connectionTimeout: options.connectionTimeout ?? 10000,
    useWorkerDecode: options.useWorkerDecode ?? true,
    workerDecodeThreshold: options.workerDecodeThreshold ?? 10000,
    // Reconnect with exponential backoff
    reconnectDelay: 1000,
    maxReconnectDelay: 30000,
    // Require basic capabilities
    requiredCapabilities: ['subscribe', 'publish'],
  }

  const client = new RosKitClient(config)

  // Start connection
  client.connect().catch((err) => {
    console.error('[RosKit] Initial connection failed:', err)
  })

  return {
    client,
    url,
    cleanup: () => client.destroy(),
  }
}
