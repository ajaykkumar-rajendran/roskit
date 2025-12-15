import { ScrollArea } from '@/components/ui/scroll-area'
import { ConnectionDialog } from './ConnectionDialog'
import { RobotCard } from './RobotCard'
import { CompareSelector } from '@/components/compare/CompareSelector'
import { useConnectionStore } from '@/stores/connectionStore'
import { Bot } from 'lucide-react'

export function ConnectionSidebar() {
  const robots = useConnectionStore((s) => s.robots)
  const robotList = Object.values(robots)

  return (
    <div className="flex flex-col h-full border-r border-[hsl(var(--border))] bg-[hsl(var(--card))]">
      <div className="p-4 border-b border-[hsl(var(--border))]">
        <div className="flex items-center justify-between mb-4">
          <h2 className="font-semibold">Robots</h2>
          <ConnectionDialog />
        </div>
      </div>

      <ScrollArea className="flex-1">
        <div className="p-4 space-y-3">
          {robotList.length === 0 ? (
            <div className="text-center py-8">
              <Bot className="h-12 w-12 mx-auto text-[hsl(var(--muted-foreground))] opacity-50" />
              <p className="mt-3 text-sm text-[hsl(var(--muted-foreground))]">
                No robots connected
              </p>
              <p className="mt-1 text-xs text-[hsl(var(--muted-foreground))]">
                Click "Add Robot" to connect
              </p>
            </div>
          ) : (
            robotList.map((robot) => <RobotCard key={robot.id} robot={robot} />)
          )}
        </div>
      </ScrollArea>

      <CompareSelector />
    </div>
  )
}
