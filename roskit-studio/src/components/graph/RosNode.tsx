import { memo } from 'react'
import { Handle, Position, type NodeProps, type Node } from '@xyflow/react'
import { Box } from 'lucide-react'

type RosNodeType = Node<{ label: string }, 'rosNode'>

export const RosNode = memo(({ data }: NodeProps<RosNodeType>) => {
  return (
    <div className="px-3 py-2 rounded-lg border border-blue-500/50 bg-blue-500/10 min-w-[120px]">
      <Handle
        type="target"
        position={Position.Left}
        className="bg-blue-500! w-2! h-2!"
      />
      <div className="flex items-center gap-2">
        <Box className="h-4 w-4 text-blue-400 shrink-0" />
        <span className="text-xs font-mono text-blue-100 truncate max-w-[150px]">
          {data.label}
        </span>
      </div>
      <Handle
        type="source"
        position={Position.Right}
        className="bg-blue-500! w-2! h-2!"
      />
    </div>
  )
})

RosNode.displayName = 'RosNode'
