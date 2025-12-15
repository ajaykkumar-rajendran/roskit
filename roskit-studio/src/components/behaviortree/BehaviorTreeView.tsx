import {
  TreeDeciduous,
  Upload,
  Play,
  Pause,
  RotateCcw,
} from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { ScrollArea } from '@/components/ui/scroll-area'
import { BTNodeComponent } from './BTNodeComponent'
import { useBehaviorTreeStore } from '@/stores/behaviorTreeStore'
import { useConnectionStore } from '@/stores/connectionStore'
import type { BTNode } from '@/types/behaviortree'

export function BehaviorTreeView() {
  const { robots, activeRobotId, getClient } = useConnectionStore()
  const {
    trees,
    activeTreeId,
    isSubscribed,
    subscribeToStatus,
    unsubscribeFromStatus,
    loadDemoTree,
    loadTreeFromXML,
  } = useBehaviorTreeStore()

  const activeRobot = activeRobotId ? robots[activeRobotId] : null
  const activeTree = activeTreeId ? trees[activeTreeId] : null

  const handleSubscribe = async () => {
    if (!activeRobotId) return

    const client = getClient(activeRobotId)
    if (client) {
      if (isSubscribed) {
        unsubscribeFromStatus()
      } else {
        await subscribeToStatus(client, activeRobotId)
      }
    }
  }

  const handleLoadDemo = () => {
    if (activeRobotId) {
      loadDemoTree(activeRobotId)
    }
  }

  const handleFileUpload = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0]
    if (!file || !activeRobotId) return

    const reader = new FileReader()
    reader.onload = (e) => {
      const xml = e.target?.result as string
      loadTreeFromXML(activeRobotId, xml, file.name.replace('.xml', ''))
    }
    reader.readAsText(file)
  }

  const countNodes = (node: BTNode): { total: number; byStatus: Record<string, number> } => {
    const byStatus: Record<string, number> = { idle: 0, running: 0, success: 0, failure: 0 }
    let total = 1
    byStatus[node.status]++

    if (node.children) {
      node.children.forEach((child) => {
        const childCount = countNodes(child)
        total += childCount.total
        Object.keys(byStatus).forEach((k) => {
          byStatus[k] += childCount.byStatus[k]
        })
      })
    }

    return { total, byStatus }
  }

  if (!activeRobot) {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <TreeDeciduous className="h-12 w-12 opacity-50" />
        <p className="mt-3">Select a robot to view behavior tree</p>
      </div>
    )
  }

  if (activeRobot.status !== 'connected') {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <TreeDeciduous className="h-12 w-12 opacity-50" />
        <p className="mt-3">Robot is not connected</p>
      </div>
    )
  }

  if (!activeTree) {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <TreeDeciduous className="h-12 w-12 opacity-50" />
        <p className="mt-3">No behavior tree loaded</p>
        <p className="mt-1 text-sm">Load a tree from XML or use the demo</p>
        <div className="flex items-center gap-3 mt-4">
          <Button variant="outline" onClick={handleLoadDemo}>
            <Play className="h-4 w-4 mr-2" />
            Load Demo Tree
          </Button>
          <label className="cursor-pointer">
            <Button variant="outline" asChild>
              <span>
                <Upload className="h-4 w-4 mr-2" />
                Upload XML
              </span>
            </Button>
            <input
              type="file"
              accept=".xml"
              className="hidden"
              onChange={handleFileUpload}
            />
          </label>
        </div>
      </div>
    )
  }

  const nodeStats = countNodes(activeTree.rootNode)

  return (
    <div className="flex flex-col h-full">
      {/* Header */}
      <div className="p-4 border-b border-[hsl(var(--border))]">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <TreeDeciduous className="h-5 w-5" />
            <div>
              <h2 className="font-semibold">{activeTree.name}</h2>
              <p className="text-xs text-[hsl(var(--muted-foreground))]">
                {nodeStats.total} nodes • Last update:{' '}
                {new Date(activeTree.lastUpdate).toLocaleTimeString()}
              </p>
            </div>
          </div>
          <div className="flex items-center gap-2">
            <Button
              variant={isSubscribed ? 'default' : 'outline'}
              size="sm"
              onClick={handleSubscribe}
            >
              {isSubscribed ? (
                <>
                  <Pause className="h-4 w-4 mr-2" />
                  Unsubscribe
                </>
              ) : (
                <>
                  <Play className="h-4 w-4 mr-2" />
                  Subscribe
                </>
              )}
            </Button>
            <Button variant="outline" size="sm" onClick={handleLoadDemo}>
              <RotateCcw className="h-4 w-4 mr-2" />
              Reset
            </Button>
            <label className="cursor-pointer">
              <Button variant="outline" size="sm" asChild>
                <span>
                  <Upload className="h-4 w-4 mr-2" />
                  Load XML
                </span>
              </Button>
              <input
                type="file"
                accept=".xml"
                className="hidden"
                onChange={handleFileUpload}
              />
            </label>
          </div>
        </div>

        {/* Status summary */}
        <div className="flex items-center gap-2 mt-3">
          <Badge variant="secondary" className="gap-1">
            <span className="w-2 h-2 rounded-full bg-gray-400" />
            Idle: {nodeStats.byStatus.idle}
          </Badge>
          <Badge variant="secondary" className="gap-1">
            <span className="w-2 h-2 rounded-full bg-blue-400 animate-pulse" />
            Running: {nodeStats.byStatus.running}
          </Badge>
          <Badge variant="secondary" className="gap-1">
            <span className="w-2 h-2 rounded-full bg-green-400" />
            Success: {nodeStats.byStatus.success}
          </Badge>
          <Badge variant="secondary" className="gap-1">
            <span className="w-2 h-2 rounded-full bg-red-400" />
            Failure: {nodeStats.byStatus.failure}
          </Badge>
        </div>
      </div>

      {/* Legend */}
      <div className="px-4 py-2 border-b border-[hsl(var(--border))] flex flex-wrap gap-3 text-xs">
        <div className="flex items-center gap-1">
          <div className="w-4 h-4 rounded bg-purple-500" />
          <span>Root</span>
        </div>
        <div className="flex items-center gap-1">
          <div className="w-4 h-4 rounded bg-blue-500" />
          <span>Sequence</span>
        </div>
        <div className="flex items-center gap-1">
          <div className="w-4 h-4 rounded bg-orange-500" />
          <span>Selector</span>
        </div>
        <div className="flex items-center gap-1">
          <div className="w-4 h-4 rounded bg-cyan-500" />
          <span>Parallel</span>
        </div>
        <div className="flex items-center gap-1">
          <div className="w-4 h-4 rounded bg-green-500" />
          <span>Action</span>
        </div>
        <div className="flex items-center gap-1">
          <div className="w-4 h-4 rounded bg-yellow-500" />
          <span>Condition</span>
        </div>
        <div className="flex items-center gap-1">
          <div className="w-4 h-4 rounded bg-pink-500" />
          <span>Decorator</span>
        </div>
      </div>

      {/* Tree View */}
      <ScrollArea className="flex-1">
        <div className="p-4">
          <BTNodeComponent node={activeTree.rootNode} />
        </div>
      </ScrollArea>
    </div>
  )
}
