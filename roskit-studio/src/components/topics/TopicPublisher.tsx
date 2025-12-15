import { useState } from 'react'
import { Send, X } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { useRos } from '@/hooks/useRos'

interface TopicPublisherProps {
  topicName: string
  messageType: string
  robotId: string
  onClose: () => void
}

export function TopicPublisher({ topicName, messageType, robotId, onClose }: TopicPublisherProps) {
  const [message, setMessage] = useState('{\n  \n}')
  const [error, setError] = useState<string | null>(null)
  const [success, setSuccess] = useState(false)

  const { isConnected, publish } = useRos(robotId)

  const handlePublish = async () => {
    setError(null)
    setSuccess(false)

    if (!isConnected) {
      setError('Robot not connected')
      return
    }

    try {
      const parsedMessage = JSON.parse(message)
      await publish(topicName, messageType, parsedMessage)
      setSuccess(true)
      setTimeout(() => setSuccess(false), 2000)
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Invalid JSON')
    }
  }

  return (
    <Card className="w-full max-w-lg">
      <CardHeader className="pb-3">
        <div className="flex items-start justify-between">
          <div>
            <CardTitle className="text-base font-mono">{topicName}</CardTitle>
            <div className="flex items-center gap-2 mt-1">
              <Badge variant="secondary" className="font-mono text-xs">
                {messageType}
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
            <label className="text-sm font-medium mb-2 block">Message (JSON)</label>
            <textarea
              value={message}
              onChange={(e) => setMessage(e.target.value)}
              className="w-full h-40 p-3 font-mono text-sm bg-[hsl(var(--background))] border border-[hsl(var(--border))] rounded-lg resize-none focus:outline-none focus:ring-2 focus:ring-[hsl(var(--primary))]"
              placeholder='{"key": "value"}'
            />
          </div>

          {error && (
            <p className="text-sm text-red-500">{error}</p>
          )}

          {success && (
            <p className="text-sm text-green-500">Message published successfully!</p>
          )}

          <Button onClick={handlePublish} disabled={!isConnected} className="w-full gap-2">
            <Send className="h-4 w-4" />
            Publish Message
          </Button>
        </div>
      </CardContent>
    </Card>
  )
}
