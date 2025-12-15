export interface TopicStats {
  hz: number
  messageCount: number
  lastReceived: number | null
  avgLatency: number | null
  minLatency: number | null
  maxLatency: number | null
  jitter: number | null
  bytesPerSecond: number | null
  droppedMessages: number
}

export interface BenchmarkHistory {
  timestamp: number
  hz: number
  latency: number | null
  jitter: number | null
  bytesPerSecond: number | null
}

export interface TopicBenchmark {
  topicName: string
  stats: TopicStats
  history: BenchmarkHistory[]
  isRecording: boolean
}

export interface NodePerformance {
  nodeName: string
  cpuUsage: number | null
  memoryUsage: number | null
  callbackTime: number | null
}

export interface SystemBenchmark {
  totalTopics: number
  activeTopics: number
  totalNodes: number
  totalBandwidth: number
  uptime: number
}
