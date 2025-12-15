import type { TopicStats, BenchmarkHistory } from '@/types/benchmark'

export interface ExportData {
  exportedAt: string
  robotId: string
  robotName: string
  topics: TopicExportData[]
}

export interface TopicExportData {
  topicName: string
  messageType: string
  stats: TopicStats
  history: BenchmarkHistory[]
  messages: unknown[]
}

export function exportToJSON(data: ExportData): string {
  return JSON.stringify(data, null, 2)
}

export function exportToCSV(data: ExportData): string {
  const lines: string[] = []

  // Header
  lines.push('Topic Name,Message Type,Message Count,Hz,Avg Latency (ms),Jitter (ms),Bytes/sec')

  // Data rows
  data.topics.forEach((topic) => {
    const row = [
      `"${topic.topicName}"`,
      `"${topic.messageType}"`,
      topic.stats.messageCount,
      topic.stats.hz.toFixed(2),
      topic.stats.avgLatency !== null ? topic.stats.avgLatency.toFixed(2) : 'N/A',
      topic.stats.jitter !== null ? topic.stats.jitter.toFixed(2) : 'N/A',
      topic.stats.bytesPerSecond !== null ? topic.stats.bytesPerSecond.toFixed(2) : 'N/A',
    ]
    lines.push(row.join(','))
  })

  return lines.join('\n')
}

export function exportHistoryToCSV(data: ExportData): string {
  const lines: string[] = []

  // Header
  lines.push('Timestamp,Topic Name,Hz,Latency (ms),Jitter (ms),Bytes/sec')

  // Data rows
  data.topics.forEach((topic) => {
    topic.history.forEach((h) => {
      const row = [
        new Date(h.timestamp).toISOString(),
        `"${topic.topicName}"`,
        h.hz.toFixed(2),
        h.latency !== null ? h.latency.toFixed(2) : 'N/A',
        h.jitter !== null ? h.jitter.toFixed(2) : 'N/A',
        h.bytesPerSecond !== null ? h.bytesPerSecond.toFixed(2) : 'N/A',
      ]
      lines.push(row.join(','))
    })
  })

  return lines.join('\n')
}

export function downloadFile(content: string, filename: string, mimeType: string): void {
  const blob = new Blob([content], { type: mimeType })
  const url = URL.createObjectURL(blob)
  const a = document.createElement('a')
  a.href = url
  a.download = filename
  document.body.appendChild(a)
  a.click()
  document.body.removeChild(a)
  URL.revokeObjectURL(url)
}

export function generateFilename(robotName: string, extension: string): string {
  const timestamp = new Date().toISOString().replace(/[:.]/g, '-')
  const safeName = robotName.replace(/[^a-zA-Z0-9]/g, '_')
  return `ros2-debug-${safeName}-${timestamp}.${extension}`
}
