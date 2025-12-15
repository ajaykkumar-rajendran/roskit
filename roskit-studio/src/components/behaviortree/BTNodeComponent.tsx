import { memo } from 'react'
import {
  ArrowRight,
  GitBranch,
  Layers,
  Play,
  HelpCircle,
  Repeat,
  CheckCircle2,
  XCircle,
  Loader2,
  Circle,
} from 'lucide-react'
import { cn } from '@/lib/utils/cn'
import type { BTNode, BTNodeStatus, BTNodeType } from '@/types/behaviortree'

interface BTNodeComponentProps {
  node: BTNode
  depth?: number
  isLast?: boolean
}

const nodeTypeConfig: Record<BTNodeType, { icon: typeof ArrowRight; color: string; label: string }> = {
  root: { icon: GitBranch, color: 'bg-purple-500', label: 'Root' },
  sequence: { icon: ArrowRight, color: 'bg-blue-500', label: 'Sequence' },
  selector: { icon: HelpCircle, color: 'bg-orange-500', label: 'Selector' },
  parallel: { icon: Layers, color: 'bg-cyan-500', label: 'Parallel' },
  action: { icon: Play, color: 'bg-green-500', label: 'Action' },
  condition: { icon: HelpCircle, color: 'bg-yellow-500', label: 'Condition' },
  decorator: { icon: Repeat, color: 'bg-pink-500', label: 'Decorator' },
  subtree: { icon: GitBranch, color: 'bg-indigo-500', label: 'SubTree' },
}

const statusConfig: Record<BTNodeStatus, { icon: typeof Circle; color: string; bgColor: string }> = {
  idle: { icon: Circle, color: 'text-gray-400', bgColor: 'bg-gray-400/20' },
  running: { icon: Loader2, color: 'text-blue-400', bgColor: 'bg-blue-400/20' },
  success: { icon: CheckCircle2, color: 'text-green-400', bgColor: 'bg-green-400/20' },
  failure: { icon: XCircle, color: 'text-red-400', bgColor: 'bg-red-400/20' },
}

export const BTNodeComponent = memo(function BTNodeComponent({
  node,
  depth = 0,
  isLast = true,
}: BTNodeComponentProps) {
  const typeConfig = nodeTypeConfig[node.type]
  const status = statusConfig[node.status]
  const TypeIcon = typeConfig.icon
  const StatusIcon = status.icon

  const hasChildren = node.children && node.children.length > 0

  return (
    <div className="relative">
      {/* Connection line from parent */}
      {depth > 0 && (
        <div
          className="absolute left-0 top-0 w-6 border-l-2 border-b-2 border-[hsl(var(--border))] rounded-bl-lg"
          style={{
            height: '20px',
            marginLeft: `${(depth - 1) * 24 + 12}px`,
          }}
        />
      )}

      {/* Vertical line to siblings */}
      {depth > 0 && !isLast && (
        <div
          className="absolute left-0 border-l-2 border-[hsl(var(--border))]"
          style={{
            top: '20px',
            bottom: 0,
            marginLeft: `${(depth - 1) * 24 + 12}px`,
          }}
        />
      )}

      {/* Node */}
      <div
        className="flex items-center gap-2 py-1.5"
        style={{ paddingLeft: `${depth * 24}px` }}
      >
        <div
          className={cn(
            'flex items-center gap-2 px-3 py-1.5 rounded-lg border transition-all',
            status.bgColor,
            node.status === 'running' && 'ring-2 ring-blue-400/50'
          )}
        >
          {/* Type indicator */}
          <div className={cn('w-6 h-6 rounded flex items-center justify-center', typeConfig.color)}>
            <TypeIcon className="h-3.5 w-3.5 text-white" />
          </div>

          {/* Node name */}
          <div className="min-w-0">
            <p className="text-sm font-medium truncate max-w-[180px]">{node.name}</p>
            {node.nodeClass && (
              <p className="text-xs text-[hsl(var(--muted-foreground))] truncate max-w-[180px]">
                {node.nodeClass}
              </p>
            )}
          </div>

          {/* Status indicator */}
          <StatusIcon
            className={cn(
              'h-4 w-4 shrink-0',
              status.color,
              node.status === 'running' && 'animate-spin'
            )}
          />

          {/* Execution time for running nodes */}
          {node.status === 'running' && node.executionTime && (
            <span className="text-xs text-[hsl(var(--muted-foreground))]">
              {node.executionTime}ms
            </span>
          )}
        </div>
      </div>

      {/* Children */}
      {hasChildren && (
        <div className="relative">
          {node.children!.map((child, index) => (
            <BTNodeComponent
              key={child.id}
              node={child}
              depth={depth + 1}
              isLast={index === node.children!.length - 1}
            />
          ))}
        </div>
      )}
    </div>
  )
})
