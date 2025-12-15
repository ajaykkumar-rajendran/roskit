import type { TopicStats, BenchmarkHistory } from '@/types/benchmark'

const MAX_HISTORY_SIZE = 100
const MAX_TIMESTAMPS = 200

export interface MessageTimestamp {
  receivedAt: number
  messageStamp?: number
  byteSize?: number
  sequenceId?: number
}

export function createInitialStats(): TopicStats {
  return {
    hz: 0,
    messageCount: 0,
    lastReceived: null,
    avgLatency: null,
    minLatency: null,
    maxLatency: null,
    jitter: null,
    bytesPerSecond: null,
    droppedMessages: 0,
  }
}

export function calculateHz(timestamps: number[]): number {
  if (timestamps.length < 2) return 0

  const timeSpan = (timestamps[timestamps.length - 1] - timestamps[0]) / 1000
  if (timeSpan === 0) return 0

  return Math.round(((timestamps.length - 1) / timeSpan) * 10) / 10
}

export function calculateLatency(
  receivedAt: number,
  messageStamp?: { sec: number; nanosec?: number; nsec?: number }
): number | null {
  if (!messageStamp) return null

  const nanosec = messageStamp.nanosec ?? messageStamp.nsec ?? 0

  // Validate that sec looks like a reasonable Unix timestamp (after year 2000, before year 2100)
  // Year 2000 = 946684800, Year 2100 = 4102444800
  const isValidUnixTimestamp = messageStamp.sec > 946684800 && messageStamp.sec < 4102444800

  if (!isValidUnixTimestamp) {
    // The timestamp might be simulation time, relative time, or invalid
    // Skip latency calculation for non-wall-clock timestamps
    return null
  }

  const msgTimeMs = messageStamp.sec * 1000 + nanosec / 1_000_000
  const latency = receivedAt - msgTimeMs

  // Sanity check: latency should be between -10 seconds and +60 seconds
  // Negative latency can happen due to clock drift between robot and browser
  // Very large positive latency indicates clock sync issues
  if (latency < -10000 || latency > 60000) {
    return null
  }

  return Math.round(latency * 100) / 100
}

export function calculateJitter(timestamps: number[]): number | null {
  if (timestamps.length < 3) return null

  const intervals: number[] = []
  for (let i = 1; i < timestamps.length; i++) {
    intervals.push(timestamps[i] - timestamps[i - 1])
  }

  const mean = intervals.reduce((a, b) => a + b, 0) / intervals.length
  const variance =
    intervals.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / intervals.length

  return Math.round(Math.sqrt(variance) * 10) / 10
}

export function calculateBytesPerSecond(
  byteSizes: number[],
  timestamps: number[]
): number | null {
  if (timestamps.length < 2 || byteSizes.length < 2) return null

  const timeSpanSeconds = (timestamps[timestamps.length - 1] - timestamps[0]) / 1000
  if (timeSpanSeconds === 0) return null

  const totalBytes = byteSizes.reduce((a, b) => a + b, 0)
  return Math.round(totalBytes / timeSpanSeconds)
}

export function calculateDroppedMessages(sequenceIds: number[]): number {
  if (sequenceIds.length < 2) return 0

  let dropped = 0
  for (let i = 1; i < sequenceIds.length; i++) {
    const expected = sequenceIds[i - 1] + 1
    if (sequenceIds[i] > expected) {
      dropped += sequenceIds[i] - expected
    }
  }

  return dropped
}

export function updateStats(
  currentTimestamps: MessageTimestamp[],
  newMessage: MessageTimestamp
): {
  timestamps: MessageTimestamp[]
  stats: TopicStats
} {
  const timestamps = [...currentTimestamps, newMessage]

  if (timestamps.length > MAX_TIMESTAMPS) {
    timestamps.shift()
  }

  const receivedTimestamps = timestamps.map((t) => t.receivedAt)
  const latencies = timestamps
    .map((t) => (t.messageStamp ? t.receivedAt - t.messageStamp : null))
    .filter((l): l is number => l !== null)

  const byteSizes = timestamps
    .map((t) => t.byteSize)
    .filter((b): b is number => b !== undefined)

  const sequenceIds = timestamps
    .map((t) => t.sequenceId)
    .filter((s): s is number => s !== undefined)

  const hz = calculateHz(receivedTimestamps)
  const jitter = calculateJitter(receivedTimestamps)
  const bytesPerSecond = calculateBytesPerSecond(byteSizes, receivedTimestamps)
  const droppedMessages = calculateDroppedMessages(sequenceIds)

  const stats: TopicStats = {
    hz,
    messageCount: timestamps.length,
    lastReceived: newMessage.receivedAt,
    avgLatency: latencies.length > 0
      ? Math.round((latencies.reduce((a, b) => a + b, 0) / latencies.length) * 100) / 100
      : null,
    minLatency: latencies.length > 0 ? Math.min(...latencies) : null,
    maxLatency: latencies.length > 0 ? Math.max(...latencies) : null,
    jitter,
    bytesPerSecond,
    droppedMessages,
  }

  return { timestamps, stats }
}

export function addToHistory(
  history: BenchmarkHistory[],
  stats: TopicStats
): BenchmarkHistory[] {
  const newEntry: BenchmarkHistory = {
    timestamp: Date.now(),
    hz: stats.hz,
    latency: stats.avgLatency,
    jitter: stats.jitter,
    bytesPerSecond: stats.bytesPerSecond,
  }

  const newHistory = [...history, newEntry]

  if (newHistory.length > MAX_HISTORY_SIZE) {
    newHistory.shift()
  }

  return newHistory
}

export function estimateMessageSize(message: unknown): number {
  try {
    return new Blob([JSON.stringify(message)]).size
  } catch {
    return 0
  }
}

export function extractMessageStamp(
  message: unknown
): { sec: number; nanosec?: number; nsec?: number } | undefined {
  if (typeof message !== 'object' || message === null) return undefined

  const msg = message as Record<string, unknown>

  if (msg.header && typeof msg.header === 'object') {
    const header = msg.header as Record<string, unknown>
    if (header.stamp && typeof header.stamp === 'object') {
      return header.stamp as { sec: number; nanosec?: number; nsec?: number }
    }
  }

  if (msg.stamp && typeof msg.stamp === 'object') {
    return msg.stamp as { sec: number; nanosec?: number; nsec?: number }
  }

  return undefined
}

export function extractSequenceId(message: unknown): number | undefined {
  if (typeof message !== 'object' || message === null) return undefined

  const msg = message as Record<string, unknown>

  if (msg.header && typeof msg.header === 'object') {
    const header = msg.header as Record<string, unknown>
    if (typeof header.seq === 'number') {
      return header.seq
    }
  }

  return undefined
}
