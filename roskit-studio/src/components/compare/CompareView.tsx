import { useMemo } from 'react'
import { ArrowLeftRight, Check, X, Minus } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { ScrollArea } from '@/components/ui/scroll-area'
import { useConnectionStore } from '@/stores/connectionStore'
import { useCompareStore } from '@/stores/compareStore'
import { cn } from '@/lib/utils/cn'

type ComparisonStatus = 'match' | 'missing' | 'extra'

interface ComparisonItem {
  name: string
  type?: string
  status: Record<string, ComparisonStatus>
}

export function CompareView() {
  const robots = useConnectionStore((s) => s.robots)
  const { selectedRobots, stopCompare } = useCompareStore()

  const selectedRobotData = selectedRobots
    .map((id) => robots[id])
    .filter((r) => r && r.status === 'connected')

  const topicComparison = useMemo(() => {
    const allTopics = new Map<string, ComparisonItem>()

    selectedRobotData.forEach((robot) => {
      robot.topics.forEach((topic) => {
        if (!allTopics.has(topic.name)) {
          allTopics.set(topic.name, {
            name: topic.name,
            type: topic.type,
            status: {},
          })
        }
        const item = allTopics.get(topic.name)!
        item.status[robot.id] = 'match'
      })
    })

    // Mark missing topics for each robot
    allTopics.forEach((item) => {
      selectedRobotData.forEach((robot) => {
        if (!item.status[robot.id]) {
          item.status[robot.id] = 'missing'
        }
      })
    })

    return Array.from(allTopics.values()).sort((a, b) => {
      // Sort by number of missing robots (most differences first)
      const aMissing = Object.values(a.status).filter((s) => s === 'missing').length
      const bMissing = Object.values(b.status).filter((s) => s === 'missing').length
      if (aMissing !== bMissing) return bMissing - aMissing
      return a.name.localeCompare(b.name)
    })
  }, [selectedRobotData])

  const nodeComparison = useMemo(() => {
    const allNodes = new Map<string, ComparisonItem>()

    selectedRobotData.forEach((robot) => {
      robot.nodes.forEach((node) => {
        if (!allNodes.has(node)) {
          allNodes.set(node, {
            name: node,
            status: {},
          })
        }
        const item = allNodes.get(node)!
        item.status[robot.id] = 'match'
      })
    })

    allNodes.forEach((item) => {
      selectedRobotData.forEach((robot) => {
        if (!item.status[robot.id]) {
          item.status[robot.id] = 'missing'
        }
      })
    })

    return Array.from(allNodes.values()).sort((a, b) => {
      const aMissing = Object.values(a.status).filter((s) => s === 'missing').length
      const bMissing = Object.values(b.status).filter((s) => s === 'missing').length
      if (aMissing !== bMissing) return bMissing - aMissing
      return a.name.localeCompare(b.name)
    })
  }, [selectedRobotData])

  const serviceComparison = useMemo(() => {
    const allServices = new Map<string, ComparisonItem>()

    selectedRobotData.forEach((robot) => {
      robot.services.forEach((service) => {
        if (!allServices.has(service)) {
          allServices.set(service, {
            name: service,
            status: {},
          })
        }
        const item = allServices.get(service)!
        item.status[robot.id] = 'match'
      })
    })

    allServices.forEach((item) => {
      selectedRobotData.forEach((robot) => {
        if (!item.status[robot.id]) {
          item.status[robot.id] = 'missing'
        }
      })
    })

    return Array.from(allServices.values()).sort((a, b) => {
      const aMissing = Object.values(a.status).filter((s) => s === 'missing').length
      const bMissing = Object.values(b.status).filter((s) => s === 'missing').length
      if (aMissing !== bMissing) return bMissing - aMissing
      return a.name.localeCompare(b.name)
    })
  }, [selectedRobotData])

  // Calculate summary stats
  const topicDifferences = topicComparison.filter((t) =>
    Object.values(t.status).some((s) => s === 'missing')
  ).length

  const nodeDifferences = nodeComparison.filter((n) =>
    Object.values(n.status).some((s) => s === 'missing')
  ).length

  const serviceDifferences = serviceComparison.filter((s) =>
    Object.values(s.status).some((st) => st === 'missing')
  ).length

  const StatusIcon = ({ status }: { status: ComparisonStatus }) => {
    switch (status) {
      case 'match':
        return <Check className="h-4 w-4 text-green-500" />
      case 'missing':
        return <X className="h-4 w-4 text-red-500" />
      default:
        return <Minus className="h-4 w-4 text-gray-500" />
    }
  }

  return (
    <div className="flex flex-col h-full">
      {/* Header */}
      <div className="p-4 border-b border-[hsl(var(--border))] flex items-center justify-between">
        <div className="flex items-center gap-3">
          <ArrowLeftRight className="h-5 w-5" />
          <h2 className="font-semibold">Robot Comparison</h2>
          <Badge variant="secondary">{selectedRobotData.length} robots</Badge>
        </div>
        <Button variant="outline" size="sm" onClick={stopCompare}>
          Exit Compare
        </Button>
      </div>

      {/* Summary Cards */}
      <div className="p-4 grid grid-cols-3 gap-4">
        <Card>
          <CardContent className="pt-4">
            <div className="text-2xl font-bold">{topicComparison.length}</div>
            <p className="text-sm text-[hsl(var(--muted-foreground))]">Total Topics</p>
            {topicDifferences > 0 && (
              <Badge variant="error" className="mt-2">
                {topicDifferences} differences
              </Badge>
            )}
          </CardContent>
        </Card>
        <Card>
          <CardContent className="pt-4">
            <div className="text-2xl font-bold">{nodeComparison.length}</div>
            <p className="text-sm text-[hsl(var(--muted-foreground))]">Total Nodes</p>
            {nodeDifferences > 0 && (
              <Badge variant="error" className="mt-2">
                {nodeDifferences} differences
              </Badge>
            )}
          </CardContent>
        </Card>
        <Card>
          <CardContent className="pt-4">
            <div className="text-2xl font-bold">{serviceComparison.length}</div>
            <p className="text-sm text-[hsl(var(--muted-foreground))]">Total Services</p>
            {serviceDifferences > 0 && (
              <Badge variant="error" className="mt-2">
                {serviceDifferences} differences
              </Badge>
            )}
          </CardContent>
        </Card>
      </div>

      {/* Comparison Tables */}
      <ScrollArea className="flex-1">
        <div className="p-4 space-y-6">
          {/* Topics Comparison */}
          <Card>
            <CardHeader className="pb-3">
              <CardTitle className="text-base">Topics</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="border rounded-lg overflow-hidden">
                <table className="w-full text-sm">
                  <thead>
                    <tr className="border-b bg-[hsl(var(--muted))]">
                      <th className="text-left p-3 font-medium">Topic Name</th>
                      <th className="text-left p-3 font-medium">Type</th>
                      {selectedRobotData.map((robot) => (
                        <th key={robot.id} className="text-center p-3 font-medium min-w-[100px]">
                          {robot.name}
                        </th>
                      ))}
                    </tr>
                  </thead>
                  <tbody>
                    {topicComparison.slice(0, 50).map((item) => {
                      const hasDiff = Object.values(item.status).some((s) => s === 'missing')
                      return (
                        <tr
                          key={item.name}
                          className={cn(
                            'border-b last:border-b-0',
                            hasDiff && 'bg-red-500/5'
                          )}
                        >
                          <td className="p-3 font-mono text-xs">{item.name}</td>
                          <td className="p-3 text-xs text-[hsl(var(--muted-foreground))]">
                            {item.type}
                          </td>
                          {selectedRobotData.map((robot) => (
                            <td key={robot.id} className="p-3 text-center">
                              <StatusIcon status={item.status[robot.id]} />
                            </td>
                          ))}
                        </tr>
                      )
                    })}
                  </tbody>
                </table>
                {topicComparison.length > 50 && (
                  <div className="p-3 text-center text-sm text-[hsl(var(--muted-foreground))] border-t">
                    Showing 50 of {topicComparison.length} topics
                  </div>
                )}
              </div>
            </CardContent>
          </Card>

          {/* Nodes Comparison */}
          <Card>
            <CardHeader className="pb-3">
              <CardTitle className="text-base">Nodes</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="border rounded-lg overflow-hidden">
                <table className="w-full text-sm">
                  <thead>
                    <tr className="border-b bg-[hsl(var(--muted))]">
                      <th className="text-left p-3 font-medium">Node Name</th>
                      {selectedRobotData.map((robot) => (
                        <th key={robot.id} className="text-center p-3 font-medium min-w-[100px]">
                          {robot.name}
                        </th>
                      ))}
                    </tr>
                  </thead>
                  <tbody>
                    {nodeComparison.slice(0, 50).map((item) => {
                      const hasDiff = Object.values(item.status).some((s) => s === 'missing')
                      return (
                        <tr
                          key={item.name}
                          className={cn(
                            'border-b last:border-b-0',
                            hasDiff && 'bg-red-500/5'
                          )}
                        >
                          <td className="p-3 font-mono text-xs">{item.name}</td>
                          {selectedRobotData.map((robot) => (
                            <td key={robot.id} className="p-3 text-center">
                              <StatusIcon status={item.status[robot.id]} />
                            </td>
                          ))}
                        </tr>
                      )
                    })}
                  </tbody>
                </table>
                {nodeComparison.length > 50 && (
                  <div className="p-3 text-center text-sm text-[hsl(var(--muted-foreground))] border-t">
                    Showing 50 of {nodeComparison.length} nodes
                  </div>
                )}
              </div>
            </CardContent>
          </Card>

          {/* Services Comparison */}
          <Card>
            <CardHeader className="pb-3">
              <CardTitle className="text-base">Services</CardTitle>
            </CardHeader>
            <CardContent>
              <div className="border rounded-lg overflow-hidden">
                <table className="w-full text-sm">
                  <thead>
                    <tr className="border-b bg-[hsl(var(--muted))]">
                      <th className="text-left p-3 font-medium">Service Name</th>
                      {selectedRobotData.map((robot) => (
                        <th key={robot.id} className="text-center p-3 font-medium min-w-[100px]">
                          {robot.name}
                        </th>
                      ))}
                    </tr>
                  </thead>
                  <tbody>
                    {serviceComparison.slice(0, 50).map((item) => {
                      const hasDiff = Object.values(item.status).some((s) => s === 'missing')
                      return (
                        <tr
                          key={item.name}
                          className={cn(
                            'border-b last:border-b-0',
                            hasDiff && 'bg-red-500/5'
                          )}
                        >
                          <td className="p-3 font-mono text-xs">{item.name}</td>
                          {selectedRobotData.map((robot) => (
                            <td key={robot.id} className="p-3 text-center">
                              <StatusIcon status={item.status[robot.id]} />
                            </td>
                          ))}
                        </tr>
                      )
                    })}
                  </tbody>
                </table>
                {serviceComparison.length > 50 && (
                  <div className="p-3 text-center text-sm text-[hsl(var(--muted-foreground))] border-t">
                    Showing 50 of {serviceComparison.length} services
                  </div>
                )}
              </div>
            </CardContent>
          </Card>
        </div>
      </ScrollArea>
    </div>
  )
}
