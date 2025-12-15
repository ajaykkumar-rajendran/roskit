import { useCallback, useEffect, useState } from 'react'
import {
  ReactFlow,
  Controls,
  Background,
  useNodesState,
  useEdgesState,
  addEdge,
  type Connection,
  type Node,
  type Edge,
  BackgroundVariant,
} from '@xyflow/react'
import '@xyflow/react/dist/style.css'
import { GitBranch, RefreshCw, Loader2 } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { useConnectionStore } from '@/stores/connectionStore'
import { RosNode } from './RosNode'
import { RosTopic } from './RosTopic'

const nodeTypes = {
  rosNode: RosNode,
  rosTopic: RosTopic,
}

type GraphNode = Node<{ label: string }>
type GraphEdge = Edge

export function GraphView() {
  const [nodes, setNodes, onNodesChange] = useNodesState<GraphNode>([])
  const [edges, setEdges, onEdgesChange] = useEdgesState<GraphEdge>([])
  const [isLoading, setIsLoading] = useState(false)

  const { robots, activeRobotId } = useConnectionStore()
  const activeRobot = activeRobotId ? robots[activeRobotId] : null

  const onConnect = useCallback(
    (params: Connection) => setEdges((eds) => addEdge(params, eds)),
    [setEdges]
  )

  const buildGraph = useCallback(() => {
    if (!activeRobot || activeRobot.status !== 'connected') return

    setIsLoading(true)

    try {
      const graphNodes: GraphNode[] = []
      const graphEdges: GraphEdge[] = []

      // Create node elements
      const nodeCount = activeRobot.nodes.length
      const topicCount = activeRobot.topics.length

      activeRobot.nodes.forEach((nodeName, index) => {
        graphNodes.push({
          id: `node-${nodeName}`,
          type: 'rosNode',
          position: {
            x: 50,
            y: (index * 600) / Math.max(nodeCount, 1),
          },
          data: { label: nodeName },
        })
      })

      // Create topic elements from robot state
      activeRobot.topics.forEach((topic, index) => {
        graphNodes.push({
          id: `topic-${topic.name}`,
          type: 'rosTopic',
          position: {
            x: 400,
            y: (index * 600) / Math.max(topicCount, 1),
          },
          data: { label: topic.name },
        })
      })

      // Note: Node-topic connections require NODE_INFO protocol support
      // which is not yet implemented. For now, just show nodes and topics.

      setNodes(graphNodes)
      setEdges(graphEdges)
    } catch (error) {
      console.error('Failed to build graph:', error)
    } finally {
      setIsLoading(false)
    }
  }, [activeRobot, setNodes, setEdges])

  useEffect(() => {
    if (activeRobot?.status === 'connected') {
      buildGraph()
    } else {
      setNodes([])
      setEdges([])
    }
  }, [activeRobot?.id, activeRobot?.status, buildGraph, setNodes, setEdges])

  if (!activeRobot) {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <GitBranch className="h-12 w-12 opacity-50" />
        <p className="mt-3">Select a robot to view graph</p>
      </div>
    )
  }

  if (activeRobot.status !== 'connected') {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <GitBranch className="h-12 w-12 opacity-50" />
        <p className="mt-3">Robot is not connected</p>
      </div>
    )
  }

  return (
    <div className="flex flex-col h-full">
      <div className="p-4 border-b border-[hsl(var(--border))] flex items-center justify-between">
        <div className="flex items-center gap-2">
          <h2 className="font-semibold">Graph View</h2>
          {isLoading && <Loader2 className="h-4 w-4 animate-spin" />}
        </div>
        <Button variant="outline" size="sm" onClick={buildGraph} disabled={isLoading}>
          <RefreshCw className={`h-4 w-4 mr-2 ${isLoading ? 'animate-spin' : ''}`} />
          Refresh
        </Button>
      </div>

      <div className="flex-1">
        <ReactFlow
          nodes={nodes}
          edges={edges}
          onNodesChange={onNodesChange}
          onEdgesChange={onEdgesChange}
          onConnect={onConnect}
          nodeTypes={nodeTypes}
          fitView
          className="bg-[hsl(var(--background))]"
        >
          <Controls className="bg-[hsl(var(--card))] border-[hsl(var(--border))]" />
          <Background
            variant={BackgroundVariant.Dots}
            gap={20}
            size={1}
            color="hsl(var(--border))"
          />
        </ReactFlow>
      </div>

      <div className="p-2 border-t border-[hsl(var(--border))] flex items-center gap-4 text-xs text-[hsl(var(--muted-foreground))]">
        <div className="flex items-center gap-1.5">
          <div className="w-3 h-3 rounded bg-blue-500" />
          <span>Node</span>
        </div>
        <div className="flex items-center gap-1.5">
          <div className="w-3 h-3 rounded bg-green-500" />
          <span>Topic</span>
        </div>
        <div className="flex items-center gap-1.5">
          <div className="w-8 h-0.5 bg-green-500" />
          <span>Publishes</span>
        </div>
        <div className="flex items-center gap-1.5">
          <div className="w-8 h-0.5 bg-blue-500" />
          <span>Subscribes</span>
        </div>
      </div>
    </div>
  )
}
