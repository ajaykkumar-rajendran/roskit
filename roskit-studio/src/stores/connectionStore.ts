import { create } from 'zustand'
import { RosKitClient } from '@roskit/client'
import type { Robot, TopicInfo, ConnectionStatus } from '@/types/ros'

const STORAGE_KEY = 'ros2-debugger-connections'

// Saved connection info (without runtime data)
interface SavedConnection {
  id: string
  name: string
  ip: string
  port: number
}

function loadSavedConnections(): SavedConnection[] {
  try {
    const stored = localStorage.getItem(STORAGE_KEY)
    if (stored) {
      return JSON.parse(stored)
    }
  } catch (e) {
    console.error('Failed to load saved connections:', e)
  }
  return []
}

function saveConnections(robots: Record<string, Robot>) {
  try {
    const connections: SavedConnection[] = Object.values(robots).map((r) => ({
      id: r.id,
      name: r.name,
      ip: r.ip,
      port: r.port,
    }))
    localStorage.setItem(STORAGE_KEY, JSON.stringify(connections))
  } catch (e) {
    console.error('Failed to save connections:', e)
  }
}

interface ConnectionStore {
  robots: Record<string, Robot>
  clients: Record<string, RosKitClient>
  activeRobotId: string | null
  isInitialized: boolean

  addRobot: (ip: string, port?: number, name?: string) => Promise<void>
  removeRobot: (id: string) => void
  setActiveRobot: (id: string | null) => void
  refreshRobotData: (id: string) => Promise<void>
  updateRobotStatus: (id: string, status: ConnectionStatus, error?: string) => void
  getClient: (id: string) => RosKitClient | null
  reconnectSavedRobots: () => Promise<void>
}

export const useConnectionStore = create<ConnectionStore>((set, get) => ({
  robots: {},
  clients: {},
  activeRobotId: null,
  isInitialized: false,

  addRobot: async (ip: string, port = 9090, name?: string) => {
    const id = `${ip}:${port}`

    if (get().robots[id]) {
      console.warn(`Robot ${id} already exists`)
      return
    }

    const robot: Robot = {
      id,
      name: name || ip,
      ip,
      port,
      status: 'connecting',
      topics: [],
      nodes: [],
      services: [],
    }

    set((state) => ({
      robots: { ...state.robots, [id]: robot },
    }))

    try {
      console.log(`[RosKit] Connecting to ws://${ip}:${port}...`)

      const client = new RosKitClient({
        url: `ws://${ip}:${port}`,
        autoReconnect: true,
        reconnectDelay: 2000,
        maxReconnectAttempts: 10,
        connectionTimeout: 10000,
        useWorkerDecode: true,
        workerDecodeThreshold: 10000,
        requiredCapabilities: ['subscribe', 'publish'],
      })

      const connectionPromise = new Promise<void>((resolve, reject) => {
        const timeout = setTimeout(() => {
          console.error(`[RosKit] Connection timeout after 10s`)
          client.destroy()
          reject(new Error('Connection timeout - server may be unreachable'))
        }, 10000)

        client.on('connect', () => {
          console.log(`[RosKit] Connected successfully to ${ip}:${port}`)
          clearTimeout(timeout)
          resolve()
        })

        client.on('error', (error) => {
          console.error(`[RosKit] Connection error:`, error)
          clearTimeout(timeout)
          reject(error)
        })

        client.on('disconnect', (reason) => {
          console.log(`[RosKit] Disconnected: ${reason}`)
          get().updateRobotStatus(id, 'disconnected', reason ?? undefined)
        })
      })

      // Start connection
      client.connect()

      await connectionPromise

      set((state) => ({
        clients: { ...state.clients, [id]: client },
        activeRobotId: state.activeRobotId || id,
      }))

      get().updateRobotStatus(id, 'connected')
      await get().refreshRobotData(id)

      // Save to localStorage after successful connection
      saveConnections(get().robots)
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : 'Connection failed'
      get().updateRobotStatus(id, 'error', errorMessage)
    }
  },

  removeRobot: (id: string) => {
    const client = get().clients[id]
    if (client) {
      client.destroy()
    }

    set((state) => {
      const { [id]: _removedRobot, ...remainingRobots } = state.robots
      const { [id]: _removedClient, ...remainingClients } = state.clients

      // Update localStorage
      saveConnections(remainingRobots)

      return {
        robots: remainingRobots,
        clients: remainingClients,
        activeRobotId: state.activeRobotId === id
          ? Object.keys(remainingRobots)[0] || null
          : state.activeRobotId,
      }
    })
  },

  setActiveRobot: (id: string | null) => {
    set({ activeRobotId: id })
  },

  refreshRobotData: async (id: string) => {
    const client = get().clients[id]
    if (!client) return

    try {
      const [topicsResult, servicesResult, nodesResult] = await Promise.all([
        client.getTopics(),
        client.getServices(),
        client.getNodes(),
      ])

      const topics: TopicInfo[] = topicsResult.map((t) => ({
        name: t.name,
        type: (t as { msg_type?: string }).msg_type || 'unknown',
        publishers: 0,
        subscribers: 0,
      }))
      const services = servicesResult.map((s) => s.name)
      const nodes = nodesResult

      set((state) => ({
        robots: {
          ...state.robots,
          [id]: {
            ...state.robots[id],
            topics,
            nodes,
            services,
          },
        },
      }))
    } catch (error) {
      console.error('Failed to refresh robot data:', error)
    }
  },

  updateRobotStatus: (id: string, status: ConnectionStatus, error?: string) => {
    set((state) => ({
      robots: {
        ...state.robots,
        [id]: {
          ...state.robots[id],
          status,
          error,
        },
      },
    }))
  },

  getClient: (id: string) => {
    return get().clients[id] ?? null
  },

  reconnectSavedRobots: async () => {
    if (get().isInitialized) return

    set({ isInitialized: true })

    const savedConnections = loadSavedConnections()
    if (savedConnections.length === 0) return

    console.log(`[RosKit] Reconnecting to ${savedConnections.length} saved robot(s)...`)

    // Reconnect to all saved robots
    for (const conn of savedConnections) {
      // Don't await - let them connect in parallel
      get().addRobot(conn.ip, conn.port, conn.name)
    }
  },
}))
