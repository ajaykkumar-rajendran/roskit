import { LayoutDashboard } from 'lucide-react'
import { SessionRecorder } from '@/components/session/SessionRecorder'
import { DiagnosticsPanel } from '@/components/diagnostics/DiagnosticsPanel'
import { PerformanceMonitor } from '@/components/performance/PerformanceMonitor'
import { useConnectionStore } from '@/stores/connectionStore'
import { ScrollArea } from '@/components/ui/scroll-area'

export function DashboardView() {
  const { robots, activeRobotId } = useConnectionStore()
  const activeRobot = activeRobotId ? robots[activeRobotId] : null

  if (!activeRobot) {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <LayoutDashboard className="h-12 w-12 opacity-50" />
        <p className="mt-3">Select a robot to view dashboard</p>
      </div>
    )
  }

  return (
    <ScrollArea className="h-full">
      <div className="p-4 space-y-4">
        {/* Top Row - Performance and Diagnostics */}
        <div className="grid gap-4 md:grid-cols-2">
          <PerformanceMonitor />
          <DiagnosticsPanel />
        </div>

        {/* Bottom Row - Session Recording */}
        <SessionRecorder />
      </div>
    </ScrollArea>
  )
}
