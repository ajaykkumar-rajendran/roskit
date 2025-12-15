import { useEffect } from 'react'
import {
  AlertTriangle,
  AlertCircle,
  Info,
  X,
  Bell,
  BellOff,
  Trash2,
} from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { ScrollArea } from '@/components/ui/scroll-area'
import {
  useDiagnosticStore,
  checkLowHz,
  checkHighLatency,
  checkDroppedMessages,
  checkStaleTopic,
} from '@/stores/diagnosticStore'
import { useTopicStore } from '@/stores/topicStore'
import { useConnectionStore } from '@/stores/connectionStore'
import { cn } from '@/lib/utils/cn'
import type { DiagnosticSeverity } from '@/types/diagnostics'

const severityConfig: Record<DiagnosticSeverity, { icon: typeof AlertCircle; color: string; bgColor: string }> = {
  info: { icon: Info, color: 'text-blue-500', bgColor: 'bg-blue-500/10' },
  warning: { icon: AlertTriangle, color: 'text-yellow-500', bgColor: 'bg-yellow-500/10' },
  error: { icon: AlertCircle, color: 'text-red-500', bgColor: 'bg-red-500/10' },
}

export function DiagnosticsPanel() {
  const { alerts, dismissAlert, clearAlerts, addAlert, enabledChecks } = useDiagnosticStore()
  const subscriptions = useTopicStore((s) => s.subscriptions)
  const { activeRobotId } = useConnectionStore()

  // Run diagnostic checks periodically
  useEffect(() => {
    if (!activeRobotId) return

    const runChecks = () => {
      Object.entries(subscriptions).forEach(([key, sub]) => {
        if (!key.startsWith(`${activeRobotId}:`)) return
        if (!sub.isSubscribed) return

        // Check low Hz
        if (enabledChecks.includes('low_hz')) {
          const alert = checkLowHz(sub.topicName, sub.stats.hz)
          if (alert) addAlert(alert)
        }

        // Check high latency
        if (enabledChecks.includes('high_latency') && sub.stats.avgLatency !== null) {
          const alert = checkHighLatency(sub.topicName, sub.stats.avgLatency)
          if (alert) addAlert(alert)
        }

        // Check dropped messages
        if (enabledChecks.includes('dropped_messages')) {
          const alert = checkDroppedMessages(sub.topicName, sub.stats.droppedMessages)
          if (alert) addAlert(alert)
        }

        // Check stale topics
        if (enabledChecks.includes('stale_topic') && sub.stats.lastReceived) {
          const alert = checkStaleTopic(sub.topicName, sub.stats.lastReceived)
          if (alert) addAlert(alert)
        }
      })
    }

    const interval = setInterval(runChecks, 5000)
    return () => clearInterval(interval)
  }, [activeRobotId, subscriptions, enabledChecks, addAlert])

  const activeAlerts = alerts.filter((a) => !a.dismissed)
  const hasAlerts = activeAlerts.length > 0

  const alertCounts = {
    error: activeAlerts.filter((a) => a.severity === 'error').length,
    warning: activeAlerts.filter((a) => a.severity === 'warning').length,
    info: activeAlerts.filter((a) => a.severity === 'info').length,
  }

  return (
    <Card>
      <CardHeader className="pb-3">
        <div className="flex items-center justify-between">
          <CardTitle className="text-base flex items-center gap-2">
            {hasAlerts ? (
              <Bell className="h-4 w-4 text-yellow-500" />
            ) : (
              <BellOff className="h-4 w-4 text-[hsl(var(--muted-foreground))]" />
            )}
            Diagnostics
            {hasAlerts && (
              <div className="flex items-center gap-1 ml-2">
                {alertCounts.error > 0 && (
                  <Badge variant="error" className="h-5 px-1.5">{alertCounts.error}</Badge>
                )}
                {alertCounts.warning > 0 && (
                  <Badge variant="warning" className="h-5 px-1.5">{alertCounts.warning}</Badge>
                )}
                {alertCounts.info > 0 && (
                  <Badge variant="secondary" className="h-5 px-1.5">{alertCounts.info}</Badge>
                )}
              </div>
            )}
          </CardTitle>
          {hasAlerts && (
            <Button variant="ghost" size="sm" onClick={clearAlerts}>
              <Trash2 className="h-4 w-4 mr-1" />
              Clear
            </Button>
          )}
        </div>
      </CardHeader>
      <CardContent>
        {!hasAlerts ? (
          <div className="text-center py-6 text-[hsl(var(--muted-foreground))]">
            <BellOff className="h-8 w-8 mx-auto opacity-50" />
            <p className="mt-2 text-sm">No active alerts</p>
            <p className="text-xs mt-1">Subscribe to topics to enable diagnostics</p>
          </div>
        ) : (
          <ScrollArea className="h-[300px]">
            <div className="space-y-2 pr-4">
              {activeAlerts.map((alert) => {
                const config = severityConfig[alert.severity]
                const Icon = config.icon

                return (
                  <div
                    key={alert.id}
                    className={cn(
                      'flex items-start gap-3 p-3 rounded-lg',
                      config.bgColor
                    )}
                  >
                    <Icon className={cn('h-5 w-5 shrink-0 mt-0.5', config.color)} />
                    <div className="flex-1 min-w-0">
                      <div className="flex items-center justify-between gap-2">
                        <p className="font-medium text-sm">{alert.title}</p>
                        <Button
                          variant="ghost"
                          size="icon"
                          className="h-6 w-6 shrink-0"
                          onClick={() => dismissAlert(alert.id)}
                        >
                          <X className="h-3.5 w-3.5" />
                        </Button>
                      </div>
                      <p className="text-sm text-[hsl(var(--muted-foreground))] mt-0.5">
                        {alert.message}
                      </p>
                      <div className="flex items-center gap-2 mt-2 text-xs text-[hsl(var(--muted-foreground))]">
                        {alert.topicName && (
                          <Badge variant="secondary" className="text-xs font-mono">
                            {alert.topicName}
                          </Badge>
                        )}
                        <span>{new Date(alert.timestamp).toLocaleTimeString()}</span>
                      </div>
                    </div>
                  </div>
                )
              })}
            </div>
          </ScrollArea>
        )}
      </CardContent>
    </Card>
  )
}
