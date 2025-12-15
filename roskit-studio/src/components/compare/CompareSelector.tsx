import { ArrowLeftRight } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { useCompareStore } from '@/stores/compareStore'
import { useConnectionStore } from '@/stores/connectionStore'

export function CompareSelector() {
  const { selectedRobots, startCompare, clearSelection, isComparing } = useCompareStore()
  const robots = useConnectionStore((s) => s.robots)

  const connectedCount = Object.values(robots).filter(
    (r) => r.status === 'connected'
  ).length

  if (connectedCount < 2 || isComparing) {
    return null
  }

  return (
    <div className="p-3 border-t border-[hsl(var(--border))] bg-[hsl(var(--muted))]">
      <div className="flex items-center justify-between mb-2">
        <div className="flex items-center gap-2">
          <ArrowLeftRight className="h-4 w-4" />
          <span className="text-sm font-medium">Compare Mode</span>
        </div>
        {selectedRobots.length > 0 && (
          <Badge variant="secondary">{selectedRobots.length} selected</Badge>
        )}
      </div>
      <p className="text-xs text-[hsl(var(--muted-foreground))] mb-3">
        Select 2-4 robots to compare their topics, nodes, and services
      </p>
      <div className="flex gap-2">
        <Button
          variant="default"
          size="sm"
          className="flex-1"
          disabled={selectedRobots.length < 2}
          onClick={startCompare}
        >
          Compare ({selectedRobots.length})
        </Button>
        {selectedRobots.length > 0 && (
          <Button variant="outline" size="sm" onClick={clearSelection}>
            Clear
          </Button>
        )}
      </div>
    </div>
  )
}
