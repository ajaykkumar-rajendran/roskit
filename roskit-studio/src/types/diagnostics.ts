export type DiagnosticSeverity = 'info' | 'warning' | 'error'

export interface DiagnosticAlert {
  id: string
  severity: DiagnosticSeverity
  title: string
  message: string
  topicName?: string
  nodeName?: string
  timestamp: number
  dismissed: boolean
}

export interface DiagnosticRule {
  id: string
  name: string
  description: string
  check: (context: DiagnosticContext) => DiagnosticAlert | null
}

export interface DiagnosticContext {
  topics: Array<{ name: string; type: string; hz?: number; lastMessage?: number }>
  nodes: string[]
  subscriptions: Record<string, { hz: number; latency: number | null; droppedMessages: number }>
}
