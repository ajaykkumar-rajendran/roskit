import { useState, useEffect } from 'react'
import {
  Settings2,
  Search,
  RefreshCw,
  Save,
  ChevronRight,
  ChevronDown,
} from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { Badge } from '@/components/ui/badge'
import { ScrollArea } from '@/components/ui/scroll-area'
import { useConnectionStore } from '@/stores/connectionStore'
import { useActiveRos } from '@/hooks/useRos'
import { cn } from '@/lib/utils/cn'

interface Parameter {
  name: string
  value: unknown
  type: string
}

interface NodeParameters {
  nodeName: string
  parameters: Parameter[]
  isExpanded: boolean
}

export function ParameterEditor() {
  const [search, setSearch] = useState('')
  const [nodeParams, setNodeParams] = useState<NodeParameters[]>([])
  const [loading, setLoading] = useState(false)
  const [editingParam, setEditingParam] = useState<string | null>(null)
  const [editValue, setEditValue] = useState('')

  const { robots, activeRobotId } = useConnectionStore()
  const { isConnected, callService } = useActiveRos()
  const activeRobot = activeRobotId ? robots[activeRobotId] : null

  const fetchParameters = async () => {
    if (!activeRobot || !isConnected) return

    setLoading(true)

    try {
      // Get parameters for each node
      const results: NodeParameters[] = []

      for (const nodeName of activeRobot.nodes.slice(0, 10)) { // Limit to 10 nodes
        try {
          const params = await getNodeParameters(nodeName)
          if (params.length > 0) {
            results.push({
              nodeName,
              parameters: params,
              isExpanded: false,
            })
          }
        } catch {
          // Node might not support parameters
        }
      }

      setNodeParams(results)
    } finally {
      setLoading(false)
    }
  }

  const getNodeParameters = async (nodeName: string): Promise<Parameter[]> => {
    try {
      const result = await callService<{ prefixes: string[]; depth: number }, { result?: { names?: string[] } }>(
        `${nodeName}/list_parameters`,
        'rcl_interfaces/srv/ListParameters',
        { prefixes: [], depth: 0 },
        { timeoutMs: 2000 }
      )

      const names = result?.result?.names || []
      return names.map((name) => ({
        name,
        value: '...',
        type: 'unknown',
      }))
    } catch {
      return []
    }
  }

  const toggleNode = (nodeName: string) => {
    setNodeParams((prev) =>
      prev.map((np) =>
        np.nodeName === nodeName ? { ...np, isExpanded: !np.isExpanded } : np
      )
    )
  }

  const handleEdit = (nodeName: string, paramName: string, currentValue: unknown) => {
    setEditingParam(`${nodeName}:${paramName}`)
    setEditValue(String(currentValue))
  }

  const handleSave = async (nodeName: string, paramName: string) => {
    if (!isConnected) return

    // Try to parse the value
    let parsedValue: unknown = editValue
    try {
      parsedValue = JSON.parse(editValue)
    } catch {
      // Keep as string
    }

    try {
      await callService(
        `${nodeName}/set_parameters`,
        'rcl_interfaces/srv/SetParameters',
        {
          parameters: [{
            name: paramName,
            value: { type: 0, bool_value: false, integer_value: 0, double_value: 0, string_value: String(parsedValue) },
          }],
        },
        { timeoutMs: 5000 }
      )
      setEditingParam(null)
      fetchParameters() // Refresh
    } catch (error) {
      console.error('Failed to set parameter:', error)
    }
  }

  const filteredNodes = nodeParams.filter((np) => {
    if (!search) return true
    const searchLower = search.toLowerCase()
    return (
      np.nodeName.toLowerCase().includes(searchLower) ||
      np.parameters.some((p) => p.name.toLowerCase().includes(searchLower))
    )
  })

  useEffect(() => {
    if (isConnected) {
      fetchParameters()
    }
  }, [activeRobotId, isConnected])

  if (!activeRobot || !isConnected) {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <Settings2 className="h-12 w-12 opacity-50" />
        <p className="mt-3">Connect to a robot to view parameters</p>
      </div>
    )
  }

  return (
    <div className="flex flex-col h-full">
      <div className="p-4 border-b border-[hsl(var(--border))] space-y-3">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-2">
            <Settings2 className="h-5 w-5" />
            <h2 className="font-semibold">Parameters</h2>
            <Badge variant="secondary">{nodeParams.length} nodes</Badge>
          </div>
          <Button
            variant="outline"
            size="sm"
            onClick={fetchParameters}
            disabled={loading}
          >
            <RefreshCw className={cn('h-4 w-4 mr-2', loading && 'animate-spin')} />
            Refresh
          </Button>
        </div>
        <div className="relative">
          <Search className="absolute left-3 top-1/2 -translate-y-1/2 h-4 w-4 text-[hsl(var(--muted-foreground))]" />
          <Input
            placeholder="Search parameters..."
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            className="pl-9"
          />
        </div>
      </div>

      <ScrollArea className="flex-1">
        <div className="p-4 space-y-2">
          {loading ? (
            <div className="text-center py-8 text-[hsl(var(--muted-foreground))]">
              <RefreshCw className="h-8 w-8 mx-auto animate-spin opacity-50" />
              <p className="mt-2">Loading parameters...</p>
            </div>
          ) : filteredNodes.length === 0 ? (
            <div className="text-center py-8 text-[hsl(var(--muted-foreground))]">
              <Settings2 className="h-8 w-8 mx-auto opacity-50" />
              <p className="mt-2">No parameters found</p>
            </div>
          ) : (
            filteredNodes.map((np) => (
              <div
                key={np.nodeName}
                className="border border-[hsl(var(--border))] rounded-lg overflow-hidden"
              >
                <button
                  className="w-full flex items-center gap-2 p-3 hover:bg-[hsl(var(--accent))] transition-colors"
                  onClick={() => toggleNode(np.nodeName)}
                >
                  {np.isExpanded ? (
                    <ChevronDown className="h-4 w-4" />
                  ) : (
                    <ChevronRight className="h-4 w-4" />
                  )}
                  <span className="font-mono text-sm">{np.nodeName}</span>
                  <Badge variant="secondary" className="ml-auto">
                    {np.parameters.length}
                  </Badge>
                </button>

                {np.isExpanded && (
                  <div className="border-t border-[hsl(var(--border))] divide-y divide-[hsl(var(--border))]">
                    {np.parameters.map((param) => {
                      const editKey = `${np.nodeName}:${param.name}`
                      const isEditing = editingParam === editKey

                      return (
                        <div
                          key={param.name}
                          className="flex items-center gap-3 px-3 py-2 pl-9"
                        >
                          <span className="font-mono text-xs flex-1 truncate">
                            {param.name}
                          </span>
                          {isEditing ? (
                            <div className="flex items-center gap-2">
                              <Input
                                value={editValue}
                                onChange={(e) => setEditValue(e.target.value)}
                                className="h-7 w-32 text-xs"
                              />
                              <Button
                                variant="ghost"
                                size="icon"
                                className="h-7 w-7"
                                onClick={() => handleSave(np.nodeName, param.name)}
                              >
                                <Save className="h-3.5 w-3.5" />
                              </Button>
                            </div>
                          ) : (
                            <Badge
                              variant="secondary"
                              className="font-mono text-xs cursor-pointer hover:bg-[hsl(var(--primary))]/20"
                              onClick={() => handleEdit(np.nodeName, param.name, param.value)}
                            >
                              {String(param.value)}
                            </Badge>
                          )}
                        </div>
                      )
                    })}
                  </div>
                )}
              </div>
            ))
          )}
        </div>
      </ScrollArea>
    </div>
  )
}
