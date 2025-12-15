/**
 * RosKit Connection Store
 *
 * Manages connections using the high-performance RosKit protocol
 * with binary encoding support for large messages like maps and laser scans.
 */

import { create } from 'zustand'
import { RosKitClient, type ConnectionState, type TopicInfo, type ServerInfo } from '@roskit/client'

interface RosKitConnection {
  id: string
  name: string
  url: string
  client: RosKitClient
  state: ConnectionState
  serverInfo: ServerInfo | null
  topics: TopicInfo[]
  nodes: string[]
  error: string | null
}

interface RosKitStore {
  connections: Record<string, RosKitConnection>
  activeConnectionId: string | null

  // Actions
  addConnection: (url: string, name?: string) => Promise<string>
  removeConnection: (id: string) => void
  setActiveConnection: (id: string | null) => void
  refreshTopics: (id: string) => Promise<void>
  refreshNodes: (id: string) => Promise<void>
  refreshAll: (id: string) => Promise<void>
  getClient: (id: string) => RosKitClient | null
  getActiveClient: () => RosKitClient | null
}

export const useRosKitStore = create<RosKitStore>((set, get) => ({
  connections: {},
  activeConnectionId: null,

  addConnection: async (url: string, name?: string) => {
    const id = url

    if (get().connections[id]) {
      console.warn(`RosWeb connection ${id} already exists`)
      return id
    }

    const client = new RosKitClient({
      url,
      autoReconnect: true,
      reconnectDelay: 2000,
      maxReconnectAttempts: 5,
    })

    const connection: RosKitConnection = {
      id,
      name: name || url,
      url,
      client,
      state: 'disconnected',
      serverInfo: null,
      topics: [],
      nodes: [],
      error: null,
    }

    set((state) => ({
      connections: { ...state.connections, [id]: connection },
    }))

    // Set up event listeners
    client.on('connect', () => {
      set((state) => ({
        connections: {
          ...state.connections,
          [id]: { ...state.connections[id], state: 'connected', error: null },
        },
      }))
    })

    client.on('disconnect', (reason) => {
      set((state) => ({
        connections: {
          ...state.connections,
          [id]: { ...state.connections[id], state: 'disconnected', error: reason || null },
        },
      }))
    })

    client.on('error', (error) => {
      set((state) => ({
        connections: {
          ...state.connections,
          [id]: { ...state.connections[id], state: 'error', error: error.message },
        },
      }))
    })

    client.on('serverInfo', (info) => {
      set((state) => ({
        connections: {
          ...state.connections,
          [id]: { ...state.connections[id], serverInfo: info },
        },
      }))
    })

    client.on('topicList', (topics) => {
      set((state) => ({
        connections: {
          ...state.connections,
          [id]: { ...state.connections[id], topics },
        },
      }))
    })

    client.on('nodeList', (nodes) => {
      set((state) => ({
        connections: {
          ...state.connections,
          [id]: { ...state.connections[id], nodes },
        },
      }))
    })

    // Try to connect
    try {
      set((state) => ({
        connections: {
          ...state.connections,
          [id]: { ...state.connections[id], state: 'connecting' },
        },
      }))

      await client.connect()

      // Set as active if no active connection
      if (!get().activeConnectionId) {
        set({ activeConnectionId: id })
      }

      // Fetch topics and nodes
      await get().refreshAll(id)
    } catch (error) {
      const errorMsg = error instanceof Error ? error.message : 'Connection failed'
      set((state) => ({
        connections: {
          ...state.connections,
          [id]: { ...state.connections[id], state: 'error', error: errorMsg },
        },
      }))
    }

    return id
  },

  removeConnection: (id: string) => {
    const connection = get().connections[id]
    if (connection) {
      connection.client.disconnect()
    }

    set((state) => {
      const { [id]: _, ...remaining } = state.connections
      return {
        connections: remaining,
        activeConnectionId: state.activeConnectionId === id
          ? Object.keys(remaining)[0] || null
          : state.activeConnectionId,
      }
    })
  },

  setActiveConnection: (id: string | null) => {
    set({ activeConnectionId: id })
  },

  refreshTopics: async (id: string) => {
    const connection = get().connections[id]
    if (!connection || connection.state !== 'connected') return

    try {
      const topics = await connection.client.getTopics()
      set((state) => ({
        connections: {
          ...state.connections,
          [id]: { ...state.connections[id], topics },
        },
      }))
    } catch (error) {
      console.error('Failed to refresh topics:', error)
    }
  },

  refreshNodes: async (id: string) => {
    const connection = get().connections[id]
    if (!connection || connection.state !== 'connected') return

    try {
      const nodes = await connection.client.getNodes()
      set((state) => ({
        connections: {
          ...state.connections,
          [id]: { ...state.connections[id], nodes },
        },
      }))
    } catch (error) {
      console.error('Failed to refresh nodes:', error)
    }
  },

  refreshAll: async (id: string) => {
    await Promise.all([
      get().refreshTopics(id),
      get().refreshNodes(id),
    ])
  },

  getClient: (id: string) => {
    return get().connections[id]?.client || null
  },

  getActiveClient: () => {
    const { activeConnectionId, connections } = get()
    if (!activeConnectionId) return null
    return connections[activeConnectionId]?.client || null
  },
}))
