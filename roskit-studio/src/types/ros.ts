export type ConnectionStatus = 'connecting' | 'connected' | 'disconnected' | 'error'

export interface Robot {
  id: string
  name: string
  ip: string
  port: number
  status: ConnectionStatus
  topics: TopicInfo[]
  nodes: string[]
  services: string[]
  error?: string
}

export interface TopicInfo {
  name: string
  type: string
  publishers: number
  subscribers: number
}

export interface TopicMessage {
  topic: string
  message: unknown
  timestamp: number
}

export interface NodeInfo {
  name: string
  publishers: string[]
  subscribers: string[]
  services: string[]
}

export interface ServiceInfo {
  name: string
  type: string
}
