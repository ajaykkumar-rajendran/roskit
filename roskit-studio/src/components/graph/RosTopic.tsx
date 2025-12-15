import { memo } from 'react'
import { Handle, Position, type NodeProps, type Node } from '@xyflow/react'
import { Radio } from 'lucide-react'

type RosTopicType = Node<{ label: string }, 'rosTopic'>

export const RosTopic = memo(({ data }: NodeProps<RosTopicType>) => {
  return (
    <div className="px-3 py-2 rounded-lg border border-green-500/50 bg-green-500/10 min-w-[120px]">
      <Handle
        type="target"
        position={Position.Left}
        className="bg-green-500! w-2! h-2!"
      />
      <div className="flex items-center gap-2">
        <Radio className="h-4 w-4 text-green-400 shrink-0" />
        <span className="text-xs font-mono text-green-100 truncate max-w-[150px]">
          {data.label}
        </span>
      </div>
      <Handle
        type="source"
        position={Position.Right}
        className="bg-green-500! w-2! h-2!"
      />
    </div>
  )
})

RosTopic.displayName = 'RosTopic'
