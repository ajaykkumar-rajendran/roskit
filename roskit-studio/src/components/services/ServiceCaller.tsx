import { useState } from 'react'
import { Play, X, Loader2 } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { useRos } from '@/hooks/useRos'

interface ServiceCallerProps {
  serviceName: string
  serviceType: string
  robotId: string
  onClose: () => void
}

export function ServiceCaller({ serviceName, serviceType, robotId, onClose }: ServiceCallerProps) {
  const [request, setRequest] = useState('{}')
  const [response, setResponse] = useState<string | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [isLoading, setIsLoading] = useState(false)

  const { isConnected, callService } = useRos(robotId)

  const handleCall = async () => {
    setError(null)
    setResponse(null)
    setIsLoading(true)

    if (!isConnected) {
      setError('Robot not connected')
      setIsLoading(false)
      return
    }

    try {
      const parsedRequest = JSON.parse(request)
      const result = await callService(serviceName, serviceType, parsedRequest, {
        timeoutMs: 30000,
      })
      setResponse(JSON.stringify(result, null, 2))
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Service call failed')
    } finally {
      setIsLoading(false)
    }
  }

  return (
    <Card className="w-full max-w-lg">
      <CardHeader className="pb-3">
        <div className="flex items-start justify-between">
          <div>
            <CardTitle className="text-base font-mono">{serviceName}</CardTitle>
            <div className="flex items-center gap-2 mt-1">
              <Badge variant="secondary" className="font-mono text-xs">
                {serviceType}
              </Badge>
            </div>
          </div>
          <Button variant="ghost" size="icon" onClick={onClose}>
            <X className="h-4 w-4" />
          </Button>
        </div>
      </CardHeader>
      <CardContent>
        <div className="space-y-4">
          <div>
            <label className="text-sm font-medium mb-2 block">Request (JSON)</label>
            <textarea
              value={request}
              onChange={(e) => setRequest(e.target.value)}
              className="w-full h-32 p-3 font-mono text-sm bg-[hsl(var(--background))] border border-[hsl(var(--border))] rounded-lg resize-none focus:outline-none focus:ring-2 focus:ring-[hsl(var(--primary))]"
              placeholder='{}'
            />
          </div>

          <Button onClick={handleCall} disabled={isLoading || !isConnected} className="w-full gap-2">
            {isLoading ? (
              <>
                <Loader2 className="h-4 w-4 animate-spin" />
                Calling...
              </>
            ) : (
              <>
                <Play className="h-4 w-4" />
                Call Service
              </>
            )}
          </Button>

          {error && (
            <div className="p-3 bg-red-500/10 border border-red-500/50 rounded-lg">
              <p className="text-sm text-red-500">{error}</p>
            </div>
          )}

          {response && (
            <div>
              <label className="text-sm font-medium mb-2 block">Response</label>
              <pre className="p-3 bg-[hsl(var(--background))] border border-[hsl(var(--border))] rounded-lg font-mono text-xs overflow-auto max-h-40">
                {response}
              </pre>
            </div>
          )}
        </div>
      </CardContent>
    </Card>
  )
}
