import { Bot, Wifi, WifiOff, RefreshCw, X, AlertCircle, Check } from 'lucide-react'
import { Card } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { cn } from '@/lib/utils/cn'
import { useConnectionStore } from '@/stores/connectionStore'
import { useTopicStore } from '@/stores/topicStore'
import { useCompareStore } from '@/stores/compareStore'
import type { Robot } from '@/types/ros'

interface RobotCardProps {
  robot: Robot
}

export function RobotCard({ robot }: RobotCardProps) {
  const { activeRobotId, setActiveRobot, removeRobot, refreshRobotData } =
    useConnectionStore()
  const { unsubscribeAll } = useTopicStore()
  const { selectedRobots, toggleRobotSelection, isComparing } = useCompareStore()

  const isSelectedForCompare = selectedRobots.includes(robot.id)
  const connectedRobotCount = Object.values(useConnectionStore.getState().robots).filter(
    (r) => r.status === 'connected'
  ).length
  const showCompareCheckbox = connectedRobotCount >= 2 && robot.status === 'connected' && !isComparing

  const isActive = activeRobotId === robot.id

  const handleRemove = () => {
    unsubscribeAll(robot.id)
    removeRobot(robot.id)
  }

  const handleRefresh = () => {
    refreshRobotData(robot.id)
  }

  const statusConfig = {
    connected: {
      icon: Wifi,
      color: 'text-green-400',
      badge: 'success' as const,
      label: 'Connected',
    },
    connecting: {
      icon: Wifi,
      color: 'text-yellow-400 animate-pulse',
      badge: 'warning' as const,
      label: 'Connecting',
    },
    disconnected: {
      icon: WifiOff,
      color: 'text-gray-400',
      badge: 'secondary' as const,
      label: 'Disconnected',
    },
    error: {
      icon: AlertCircle,
      color: 'text-red-400',
      badge: 'error' as const,
      label: 'Error',
    },
  }

  const status = statusConfig[robot.status]
  const StatusIcon = status.icon

  const handleCompareToggle = (e: React.MouseEvent) => {
    e.stopPropagation()
    toggleRobotSelection(robot.id)
  }

  return (
    <Card
      className={cn(
        'p-3 cursor-pointer transition-all hover:border-[hsl(var(--primary))]/50',
        isActive && 'border-[hsl(var(--primary))] bg-[hsl(var(--primary))]/5',
        isSelectedForCompare && 'ring-2 ring-blue-500/50'
      )}
      onClick={() => setActiveRobot(robot.id)}
    >
      <div className="flex items-start justify-between gap-2">
        <div className="flex items-center gap-2 min-w-0">
          {showCompareCheckbox ? (
            <button
              onClick={handleCompareToggle}
              className={cn(
                'h-5 w-5 shrink-0 rounded border flex items-center justify-center transition-colors',
                isSelectedForCompare
                  ? 'bg-blue-500 border-blue-500 text-white'
                  : 'border-[hsl(var(--border))] hover:border-blue-500'
              )}
            >
              {isSelectedForCompare && <Check className="h-3 w-3" />}
            </button>
          ) : (
            <Bot className="h-5 w-5 shrink-0 text-[hsl(var(--muted-foreground))]" />
          )}
          <div className="min-w-0">
            <p className="font-medium truncate">{robot.name}</p>
            <p className="text-xs text-[hsl(var(--muted-foreground))] truncate">
              {robot.ip}:{robot.port}
            </p>
          </div>
        </div>
        <StatusIcon className={cn('h-4 w-4 shrink-0', status.color)} />
      </div>

      <div className="mt-3 flex items-center justify-between">
        <Badge variant={status.badge} className="text-xs">
          {status.label}
        </Badge>
        <div className="flex items-center gap-1">
          {robot.status === 'connected' && (
            <Button
              variant="ghost"
              size="icon"
              className="h-7 w-7"
              onClick={(e) => {
                e.stopPropagation()
                handleRefresh()
              }}
            >
              <RefreshCw className="h-3.5 w-3.5" />
            </Button>
          )}
          <Button
            variant="ghost"
            size="icon"
            className="h-7 w-7 text-[hsl(var(--muted-foreground))] hover:text-[hsl(var(--destructive))]"
            onClick={(e) => {
              e.stopPropagation()
              handleRemove()
            }}
          >
            <X className="h-3.5 w-3.5" />
          </Button>
        </div>
      </div>

      {robot.status === 'connected' && (
        <div className="mt-2 flex gap-3 text-xs text-[hsl(var(--muted-foreground))]">
          <span>{robot.topics.length} topics</span>
          <span>{robot.nodes.length} nodes</span>
        </div>
      )}

      {robot.error && (
        <p className="mt-2 text-xs text-red-400 truncate">{robot.error}</p>
      )}
    </Card>
  )
}
