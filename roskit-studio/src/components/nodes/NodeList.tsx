import { useState, useMemo } from 'react'
import { Search, Box } from 'lucide-react'
import { Input } from '@/components/ui/input'
import { NodeRow } from './NodeRow'
import { useConnectionStore } from '@/stores/connectionStore'

export function NodeList() {
  const [search, setSearch] = useState('')
  const { robots, activeRobotId } = useConnectionStore()

  const activeRobot = activeRobotId ? robots[activeRobotId] : null

  const filteredNodes = useMemo(() => {
    if (!activeRobot) return []

    const searchLower = search.toLowerCase()
    return activeRobot.nodes.filter((node) =>
      node.toLowerCase().includes(searchLower)
    )
  }, [activeRobot, search])

  if (!activeRobot) {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <Box className="h-12 w-12 opacity-50" />
        <p className="mt-3">Select a robot to view nodes</p>
      </div>
    )
  }

  if (activeRobot.status !== 'connected') {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <Box className="h-12 w-12 opacity-50" />
        <p className="mt-3">Robot is not connected</p>
      </div>
    )
  }

  return (
    <div className="flex flex-col h-full overflow-hidden">
      <div className="p-4 border-b border-[hsl(var(--border))] shrink-0">
        <div className="flex items-center gap-2">
          <h2 className="font-semibold">Nodes</h2>
          <span className="text-sm text-[hsl(var(--muted-foreground))]">
            ({filteredNodes.length})
          </span>
        </div>
        <div className="relative mt-3">
          <Search className="absolute left-3 top-1/2 -translate-y-1/2 h-4 w-4 text-[hsl(var(--muted-foreground))]" />
          <Input
            placeholder="Search nodes..."
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            className="pl-9"
          />
        </div>
      </div>

      <div className="flex-1 min-h-0 overflow-y-auto">
        <div className="divide-y divide-[hsl(var(--border))]">
          {filteredNodes.length === 0 ? (
            <div className="p-4 text-center text-[hsl(var(--muted-foreground))]">
              {search ? 'No nodes match your search' : 'No nodes available'}
            </div>
          ) : (
            filteredNodes.map((node) => (
              <NodeRow key={node} nodeName={node} robotId={activeRobot.id} />
            ))
          )}
        </div>
      </div>
    </div>
  )
}
