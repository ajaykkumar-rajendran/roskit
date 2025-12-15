import { useMemo } from 'react'
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  Legend,
} from 'recharts'
import { Activity, Clock, Zap, AlertTriangle, Download } from 'lucide-react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { ScrollArea } from '@/components/ui/scroll-area'
import { useTopicStore } from '@/stores/topicStore'
import { useConnectionStore } from '@/stores/connectionStore'
import {
  exportToJSON,
  exportToCSV,
  exportHistoryToCSV,
  downloadFile,
  generateFilename,
  type ExportData,
} from '@/lib/export/exportData'

export function BenchmarkPanel() {
  const { robots, activeRobotId } = useConnectionStore()
  const subscriptions = useTopicStore((s) => s.subscriptions)

  const activeRobot = activeRobotId ? robots[activeRobotId] : null

  const activeSubscriptions = useMemo(() => {
    if (!activeRobotId) return []

    return Object.entries(subscriptions)
      .filter(
        ([key, sub]) => key.startsWith(`${activeRobotId}:`) && sub.isSubscribed
      )
      .map(([key, sub]) => ({ key, ...sub }))
  }, [subscriptions, activeRobotId])

  const systemStats = useMemo(() => {
    const totalHz = activeSubscriptions.reduce((sum, sub) => sum + sub.stats.hz, 0)
    const totalMessages = activeSubscriptions.reduce(
      (sum, sub) => sum + sub.stats.messageCount,
      0
    )
    const avgLatency =
      activeSubscriptions.length > 0
        ? activeSubscriptions.reduce(
            (sum, sub) => sum + (sub.stats.avgLatency || 0),
            0
          ) / activeSubscriptions.filter((s) => s.stats.avgLatency !== null).length
        : null
    const totalBandwidth = activeSubscriptions.reduce(
      (sum, sub) => sum + (sub.stats.bytesPerSecond || 0),
      0
    )

    return { totalHz, totalMessages, avgLatency, totalBandwidth }
  }, [activeSubscriptions])

  const prepareExportData = (): ExportData => {
    return {
      exportedAt: new Date().toISOString(),
      robotId: activeRobot?.id || '',
      robotName: activeRobot?.name || '',
      topics: activeSubscriptions.map((sub) => ({
        topicName: sub.topicName,
        messageType: sub.messageType,
        stats: sub.stats,
        history: sub.history,
        messages: sub.messages,
      })),
    }
  }

  const handleExportJSON = () => {
    const data = prepareExportData()
    const content = exportToJSON(data)
    downloadFile(content, generateFilename(activeRobot?.name || 'robot', 'json'), 'application/json')
  }

  const handleExportCSV = () => {
    const data = prepareExportData()
    const content = exportToCSV(data)
    downloadFile(content, generateFilename(activeRobot?.name || 'robot', 'csv'), 'text/csv')
  }

  const handleExportHistory = () => {
    const data = prepareExportData()
    const content = exportHistoryToCSV(data)
    downloadFile(content, generateFilename(`${activeRobot?.name || 'robot'}-history`, 'csv'), 'text/csv')
  }

  if (!activeRobot || activeRobot.status !== 'connected') {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <Activity className="h-12 w-12 opacity-50" />
        <p className="mt-3">Connect to a robot to view benchmarks</p>
      </div>
    )
  }

  if (activeSubscriptions.length === 0) {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <Activity className="h-12 w-12 opacity-50" />
        <p className="mt-3">Subscribe to topics to see benchmarks</p>
        <p className="mt-1 text-sm">Go to Topics tab and click Subscribe</p>
      </div>
    )
  }

  return (
    <ScrollArea className="h-full">
      <div className="p-4 space-y-4">
        {/* Export Buttons */}
        <div className="flex items-center justify-between">
          <h2 className="text-lg font-semibold">Benchmark Dashboard</h2>
          <div className="flex items-center gap-2">
            <Button variant="outline" size="sm" onClick={handleExportJSON}>
              <Download className="h-4 w-4 mr-2" />
              JSON
            </Button>
            <Button variant="outline" size="sm" onClick={handleExportCSV}>
              <Download className="h-4 w-4 mr-2" />
              CSV
            </Button>
            <Button variant="outline" size="sm" onClick={handleExportHistory}>
              <Download className="h-4 w-4 mr-2" />
              History CSV
            </Button>
          </div>
        </div>

        {/* System Overview */}
        <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
          <Card>
            <CardContent className="p-4">
              <div className="flex items-center gap-2">
                <Activity className="h-4 w-4 text-green-400" />
                <span className="text-sm text-[hsl(var(--muted-foreground))]">
                  Total Hz
                </span>
              </div>
              <p className="mt-2 text-2xl font-bold">{systemStats.totalHz.toFixed(1)}</p>
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-4">
              <div className="flex items-center gap-2">
                <Clock className="h-4 w-4 text-blue-400" />
                <span className="text-sm text-[hsl(var(--muted-foreground))]">
                  Avg Latency
                </span>
              </div>
              <p className="mt-2 text-2xl font-bold">
                {systemStats.avgLatency !== null
                  ? `${systemStats.avgLatency.toFixed(1)}ms`
                  : 'N/A'}
              </p>
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-4">
              <div className="flex items-center gap-2">
                <Zap className="h-4 w-4 text-yellow-400" />
                <span className="text-sm text-[hsl(var(--muted-foreground))]">
                  Bandwidth
                </span>
              </div>
              <p className="mt-2 text-2xl font-bold">
                {systemStats.totalBandwidth > 1024 * 1024
                  ? `${(systemStats.totalBandwidth / (1024 * 1024)).toFixed(1)} MB/s`
                  : systemStats.totalBandwidth > 1024
                  ? `${(systemStats.totalBandwidth / 1024).toFixed(1)} KB/s`
                  : `${systemStats.totalBandwidth} B/s`}
              </p>
            </CardContent>
          </Card>

          <Card>
            <CardContent className="p-4">
              <div className="flex items-center gap-2">
                <AlertTriangle className="h-4 w-4 text-red-400" />
                <span className="text-sm text-[hsl(var(--muted-foreground))]">
                  Active Topics
                </span>
              </div>
              <p className="mt-2 text-2xl font-bold">{activeSubscriptions.length}</p>
            </CardContent>
          </Card>
        </div>

        {/* Topic Benchmarks */}
        {activeSubscriptions.map(({ key, topicName, stats, history }) => (
          <Card key={key}>
            <CardHeader className="pb-2">
              <div className="flex items-center justify-between">
                <CardTitle className="text-sm font-mono">{topicName}</CardTitle>
                <div className="flex items-center gap-2">
                  <Badge
                    variant={stats.hz > 0 ? 'success' : 'error'}
                    className="font-mono"
                  >
                    {stats.hz.toFixed(1)} Hz
                  </Badge>
                  {stats.avgLatency !== null && (
                    <Badge variant="secondary" className="font-mono">
                      {stats.avgLatency.toFixed(1)}ms
                    </Badge>
                  )}
                </div>
              </div>
            </CardHeader>
            <CardContent>
              <div className="grid grid-cols-2 md:grid-cols-5 gap-4 mb-4 text-sm">
                <div>
                  <p className="text-[hsl(var(--muted-foreground))]">Messages</p>
                  <p className="font-mono">{stats.messageCount}</p>
                </div>
                <div>
                  <p className="text-[hsl(var(--muted-foreground))]">Min Latency</p>
                  <p className="font-mono">
                    {stats.minLatency !== null ? `${stats.minLatency.toFixed(1)}ms` : 'N/A'}
                  </p>
                </div>
                <div>
                  <p className="text-[hsl(var(--muted-foreground))]">Max Latency</p>
                  <p className="font-mono">
                    {stats.maxLatency !== null ? `${stats.maxLatency.toFixed(1)}ms` : 'N/A'}
                  </p>
                </div>
                <div>
                  <p className="text-[hsl(var(--muted-foreground))]">Jitter</p>
                  <p className="font-mono">
                    {stats.jitter !== null ? `±${stats.jitter.toFixed(1)}ms` : 'N/A'}
                  </p>
                </div>
                <div>
                  <p className="text-[hsl(var(--muted-foreground))]">Dropped</p>
                  <p className="font-mono">{stats.droppedMessages}</p>
                </div>
              </div>

              {history.length > 1 && (
                <div className="h-[150px] mt-4">
                  <ResponsiveContainer width="100%" height="100%">
                    <LineChart data={history}>
                      <CartesianGrid strokeDasharray="3 3" stroke="hsl(var(--border))" />
                      <XAxis
                        dataKey="timestamp"
                        tickFormatter={(ts) =>
                          new Date(ts).toLocaleTimeString([], {
                            minute: '2-digit',
                            second: '2-digit',
                          })
                        }
                        stroke="hsl(var(--muted-foreground))"
                        fontSize={10}
                      />
                      <YAxis
                        yAxisId="hz"
                        stroke="hsl(var(--muted-foreground))"
                        fontSize={10}
                      />
                      <YAxis
                        yAxisId="latency"
                        orientation="right"
                        stroke="hsl(var(--muted-foreground))"
                        fontSize={10}
                      />
                      <Tooltip
                        contentStyle={{
                          backgroundColor: 'hsl(var(--card))',
                          borderColor: 'hsl(var(--border))',
                          borderRadius: '8px',
                        }}
                        labelFormatter={(ts) => new Date(ts).toLocaleTimeString()}
                      />
                      <Legend />
                      <Line
                        yAxisId="hz"
                        type="monotone"
                        dataKey="hz"
                        name="Hz"
                        stroke="#22c55e"
                        dot={false}
                        strokeWidth={2}
                      />
                      <Line
                        yAxisId="latency"
                        type="monotone"
                        dataKey="latency"
                        name="Latency (ms)"
                        stroke="#3b82f6"
                        dot={false}
                        strokeWidth={2}
                      />
                    </LineChart>
                  </ResponsiveContainer>
                </div>
              )}
            </CardContent>
          </Card>
        ))}
      </div>
    </ScrollArea>
  )
}
