import { useState, useEffect } from 'react'
import {
  Activity,
  Cpu,
  HardDrive,
  Wifi,
  TrendingUp,
  TrendingDown,
  Minus,
} from 'lucide-react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Progress } from '@/components/ui/progress'
import { useConnectionStore } from '@/stores/connectionStore'
import { useTopicStore } from '@/stores/topicStore'

interface PerformanceMetrics {
  messageRate: number
  messageRateTrend: 'up' | 'down' | 'stable'
  totalMessages: number
  activeTopics: number
  avgLatency: number
  avgLatencyTrend: 'up' | 'down' | 'stable'
  connectionHealth: number
  memoryUsage: number
}

export function PerformanceMonitor() {
  const [metrics, setMetrics] = useState<PerformanceMetrics>({
    messageRate: 0,
    messageRateTrend: 'stable',
    totalMessages: 0,
    activeTopics: 0,
    avgLatency: 0,
    avgLatencyTrend: 'stable',
    connectionHealth: 100,
    memoryUsage: 0,
  })

  const [history, setHistory] = useState<{ rate: number; latency: number }[]>([])

  const { robots, activeRobotId } = useConnectionStore()
  const subscriptions = useTopicStore((s) => s.subscriptions)

  const activeRobot = activeRobotId ? robots[activeRobotId] : null

  useEffect(() => {
    if (!activeRobot || activeRobot.status !== 'connected') return

    const interval = setInterval(() => {
      // Calculate metrics from subscriptions
      const activeSubscriptions = Object.entries(subscriptions).filter(
        ([key, sub]) => key.startsWith(`${activeRobotId}:`) && sub.isSubscribed
      )

      let totalRate = 0
      let totalLatency = 0
      let latencyCount = 0
      let totalMessages = 0

      activeSubscriptions.forEach(([, sub]) => {
        if (sub.stats) {
          totalRate += sub.stats.hz
          totalMessages += sub.stats.messageCount
          if (sub.stats.avgLatency !== null) {
            totalLatency += sub.stats.avgLatency
            latencyCount++
          }
        }
      })

      const avgLatency = latencyCount > 0 ? totalLatency / latencyCount : 0

      // Determine trends based on history
      const prevMetrics = history[history.length - 1]
      let rateTrend: 'up' | 'down' | 'stable' = 'stable'
      let latencyTrend: 'up' | 'down' | 'stable' = 'stable'

      if (prevMetrics) {
        if (totalRate > prevMetrics.rate * 1.1) rateTrend = 'up'
        else if (totalRate < prevMetrics.rate * 0.9) rateTrend = 'down'

        if (avgLatency > prevMetrics.latency * 1.2) latencyTrend = 'up'
        else if (avgLatency < prevMetrics.latency * 0.8) latencyTrend = 'down'
      }

      // Calculate connection health based on latency and message rate
      let health = 100
      if (avgLatency > 100) health -= 20
      if (avgLatency > 500) health -= 30
      if (totalRate === 0 && activeSubscriptions.length > 0) health -= 50

      // Estimate memory usage (simulated based on message count)
      const memUsage = Math.min(100, (totalMessages / 10000) * 100)

      setMetrics({
        messageRate: totalRate,
        messageRateTrend: rateTrend,
        totalMessages,
        activeTopics: activeSubscriptions.length,
        avgLatency,
        avgLatencyTrend: latencyTrend,
        connectionHealth: Math.max(0, health),
        memoryUsage: memUsage,
      })

      // Keep last 10 history entries
      setHistory((prev) => [...prev.slice(-9), { rate: totalRate, latency: avgLatency }])
    }, 1000)

    return () => clearInterval(interval)
  }, [activeRobotId, activeRobot?.status, subscriptions, history])

  const TrendIcon = ({ trend }: { trend: 'up' | 'down' | 'stable' }) => {
    if (trend === 'up') return <TrendingUp className="h-3 w-3 text-green-500" />
    if (trend === 'down') return <TrendingDown className="h-3 w-3 text-red-500" />
    return <Minus className="h-3 w-3 text-[hsl(var(--muted-foreground))]" />
  }

  const getHealthColor = (health: number) => {
    if (health >= 80) return 'text-green-500'
    if (health >= 50) return 'text-yellow-500'
    return 'text-red-500'
  }

  if (!activeRobot || activeRobot.status !== 'connected') {
    return (
      <Card>
        <CardHeader className="pb-3">
          <CardTitle className="text-base flex items-center gap-2">
            <Activity className="h-4 w-4" />
            Performance Monitor
          </CardTitle>
        </CardHeader>
        <CardContent>
          <p className="text-sm text-[hsl(var(--muted-foreground))] text-center py-4">
            Connect to a robot to view performance metrics
          </p>
        </CardContent>
      </Card>
    )
  }

  return (
    <Card>
      <CardHeader className="pb-3">
        <div className="flex items-center justify-between">
          <CardTitle className="text-base flex items-center gap-2">
            <Activity className="h-4 w-4" />
            Performance Monitor
          </CardTitle>
          <Badge
            variant={metrics.connectionHealth >= 80 ? 'success' : metrics.connectionHealth >= 50 ? 'warning' : 'destructive'}
          >
            {metrics.connectionHealth}% Health
          </Badge>
        </div>
      </CardHeader>
      <CardContent className="space-y-4">
        {/* Message Rate */}
        <div className="grid grid-cols-2 gap-4">
          <div className="space-y-1">
            <div className="flex items-center gap-2 text-sm text-[hsl(var(--muted-foreground))]">
              <Wifi className="h-4 w-4" />
              Message Rate
              <TrendIcon trend={metrics.messageRateTrend} />
            </div>
            <p className="text-2xl font-bold font-mono">
              {metrics.messageRate.toFixed(1)}
              <span className="text-sm font-normal text-[hsl(var(--muted-foreground))]"> Hz</span>
            </p>
          </div>

          <div className="space-y-1">
            <div className="flex items-center gap-2 text-sm text-[hsl(var(--muted-foreground))]">
              <Cpu className="h-4 w-4" />
              Avg Latency
              <TrendIcon trend={metrics.avgLatencyTrend} />
            </div>
            <p className={`text-2xl font-bold font-mono ${metrics.avgLatency > 100 ? 'text-yellow-500' : ''} ${metrics.avgLatency > 500 ? 'text-red-500' : ''}`}>
              {metrics.avgLatency.toFixed(0)}
              <span className="text-sm font-normal text-[hsl(var(--muted-foreground))]"> ms</span>
            </p>
          </div>
        </div>

        {/* Stats Row */}
        <div className="grid grid-cols-2 gap-4 py-2">
          <div className="text-center">
            <p className="text-2xl font-bold font-mono">{metrics.activeTopics}</p>
            <p className="text-xs text-[hsl(var(--muted-foreground))]">Active Topics</p>
          </div>
          <div className="text-center">
            <p className="text-2xl font-bold font-mono">
              {metrics.totalMessages > 1000
                ? `${(metrics.totalMessages / 1000).toFixed(1)}K`
                : metrics.totalMessages}
            </p>
            <p className="text-xs text-[hsl(var(--muted-foreground))]">Total Messages</p>
          </div>
        </div>

        {/* Resource Bars */}
        <div className="space-y-3">
          <div className="space-y-1">
            <div className="flex items-center justify-between text-xs">
              <span className="flex items-center gap-1 text-[hsl(var(--muted-foreground))]">
                <Wifi className={getHealthColor(metrics.connectionHealth)} />
                Connection Health
              </span>
              <span className="font-mono">{metrics.connectionHealth}%</span>
            </div>
            <Progress value={metrics.connectionHealth} className="h-2" />
          </div>

          <div className="space-y-1">
            <div className="flex items-center justify-between text-xs">
              <span className="flex items-center gap-1 text-[hsl(var(--muted-foreground))]">
                <HardDrive className="h-3 w-3" />
                Buffer Usage
              </span>
              <span className="font-mono">{metrics.memoryUsage.toFixed(0)}%</span>
            </div>
            <Progress value={metrics.memoryUsage} className="h-2" />
          </div>
        </div>
      </CardContent>
    </Card>
  )
}
