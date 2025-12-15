export interface RecordedMessage {
  timestamp: number
  topicName: string
  messageType: string
  data: unknown
}

export interface RecordedSession {
  id: string
  name: string
  robotName: string
  robotIp: string
  startTime: number
  endTime: number | null
  duration: number
  topics: string[]
  messageCount: number
  messages: RecordedMessage[]
  metadata: {
    nodes: string[]
    services: string[]
  }
}

export interface SessionSummary {
  id: string
  name: string
  robotName: string
  startTime: number
  duration: number
  messageCount: number
  topicCount: number
}
