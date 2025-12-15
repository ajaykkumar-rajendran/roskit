import { Box } from 'lucide-react'

interface NodeRowProps {
  nodeName: string
  robotId: string
}

export function NodeRow({ nodeName }: NodeRowProps) {
  return (
    <div className="hover:bg-[hsl(var(--accent))]/50 transition-colors">
      <div className="p-3 flex items-center gap-2">
        <Box className="h-4 w-4 shrink-0 text-blue-400" />
        <span className="font-mono text-sm truncate">{nodeName}</span>
      </div>
    </div>
  )
}
