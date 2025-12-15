import { create } from 'zustand'
import type { DiagnosticAlert } from '@/types/diagnostics'

interface DiagnosticStore {
  alerts: DiagnosticAlert[]
  enabledChecks: string[]

  addAlert: (alert: Omit<DiagnosticAlert, 'id' | 'timestamp' | 'dismissed'>) => void
  dismissAlert: (id: string) => void
  clearAlerts: () => void
  toggleCheck: (checkId: string) => void
}

export const useDiagnosticStore = create<DiagnosticStore>((set, get) => ({
  alerts: [],
  enabledChecks: ['low_hz', 'high_latency', 'dropped_messages', 'stale_topic', 'missing_tf'],

  addAlert: (alert) => {
    const { alerts } = get()

    // Avoid duplicate alerts
    const exists = alerts.some(
      (a) =>
        !a.dismissed &&
        a.title === alert.title &&
        a.topicName === alert.topicName &&
        Date.now() - a.timestamp < 60000 // Within 1 minute
    )

    if (exists) return

    const newAlert: DiagnosticAlert = {
      ...alert,
      id: crypto.randomUUID(),
      timestamp: Date.now(),
      dismissed: false,
    }

    set({ alerts: [newAlert, ...alerts].slice(0, 50) })
  },

  dismissAlert: (id) => {
    set((state) => ({
      alerts: state.alerts.map((a) =>
        a.id === id ? { ...a, dismissed: true } : a
      ),
    }))
  },

  clearAlerts: () => {
    set({ alerts: [] })
  },

  toggleCheck: (checkId) => {
    set((state) => ({
      enabledChecks: state.enabledChecks.includes(checkId)
        ? state.enabledChecks.filter((c) => c !== checkId)
        : [...state.enabledChecks, checkId],
    }))
  },
}))

// Diagnostic check functions
export function checkLowHz(topicName: string, hz: number): DiagnosticAlert | null {
  if (hz > 0 && hz < 1) {
    return {
      id: '',
      severity: 'warning',
      title: 'Low frequency topic',
      message: `Topic "${topicName}" is publishing at ${hz.toFixed(2)} Hz, which is unusually low.`,
      topicName,
      timestamp: 0,
      dismissed: false,
    }
  }
  return null
}

export function checkHighLatency(topicName: string, latency: number): DiagnosticAlert | null {
  if (latency > 100) {
    return {
      id: '',
      severity: latency > 500 ? 'error' : 'warning',
      title: 'High latency detected',
      message: `Topic "${topicName}" has ${latency.toFixed(0)}ms latency. This may indicate network issues or clock sync problems.`,
      topicName,
      timestamp: 0,
      dismissed: false,
    }
  }
  return null
}

export function checkDroppedMessages(topicName: string, dropped: number): DiagnosticAlert | null {
  if (dropped > 10) {
    return {
      id: '',
      severity: dropped > 100 ? 'error' : 'warning',
      title: 'Messages being dropped',
      message: `Topic "${topicName}" has dropped ${dropped} messages. This may indicate buffer overflow or network issues.`,
      topicName,
      timestamp: 0,
      dismissed: false,
    }
  }
  return null
}

export function checkStaleTopic(topicName: string, lastMessage: number): DiagnosticAlert | null {
  const staleness = Date.now() - lastMessage
  if (staleness > 10000) { // 10 seconds
    return {
      id: '',
      severity: staleness > 30000 ? 'error' : 'warning',
      title: 'Stale topic',
      message: `Topic "${topicName}" hasn't received messages for ${Math.floor(staleness / 1000)}s.`,
      topicName,
      timestamp: 0,
      dismissed: false,
    }
  }
  return null
}

export function checkMissingCriticalTopics(topics: string[]): DiagnosticAlert | null {
  const criticalTopics = ['/tf', '/tf_static', '/clock']
  const missing = criticalTopics.filter((t) => !topics.includes(t))

  if (missing.length > 0) {
    return {
      id: '',
      severity: 'info',
      title: 'Missing common topics',
      message: `These common ROS2 topics are not available: ${missing.join(', ')}`,
      timestamp: 0,
      dismissed: false,
    }
  }
  return null
}
