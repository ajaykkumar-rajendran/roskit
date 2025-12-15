import { useState, useRef, useEffect } from 'react'
import { X, Trash2, Copy, Check, Pause, Play } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { ScrollArea } from '@/components/ui/scroll-area'
import { useTopicStore } from '@/stores/topicStore'
import { cn } from '@/lib/utils/cn'

interface TopicSubscriberProps {
  subscriptionKey: string
  onClose: () => void
}

export function TopicSubscriber({ subscriptionKey, onClose }: TopicSubscriberProps) {
  const [isPaused, setIsPaused] = useState(false)
  const [copied, setCopied] = useState(false)
  const scrollRef = useRef<HTMLDivElement>(null)
  const [autoScroll] = useState(true)

  const subscription = useTopicStore((s) => s.subscriptions[subscriptionKey])
  const clearMessages = useTopicStore((s) => s.clearMessages)
  const unsubscribe = useTopicStore((s) => s.unsubscribe)

  const messages = subscription?.messages ?? []
  const stats = subscription?.stats
  const latestMessage = messages[messages.length - 1]

  useEffect(() => {
    if (autoScroll && scrollRef.current && !isPaused) {
      scrollRef.current.scrollTop = scrollRef.current.scrollHeight
    }
  }, [messages.length, autoScroll, isPaused])

  const handleCopy = () => {
    if (latestMessage) {
      navigator.clipboard.writeText(JSON.stringify(latestMessage, null, 2))
      setCopied(true)
      setTimeout(() => setCopied(false), 2000)
    }
  }

  const handleClose = () => {
    unsubscribe(subscriptionKey)
    onClose()
  }

  if (!subscription) {
    return null
  }

  const topicName = subscription.topicName

  return (
    <Card className="flex flex-col h-full">
      <CardHeader className="pb-3 border-b border-[hsl(var(--border))]">
        <div className="flex items-start justify-between gap-2">
          <div className="min-w-0">
            <CardTitle className="text-base font-mono truncate">{topicName}</CardTitle>
            <p className="text-xs text-[hsl(var(--muted-foreground))] truncate mt-1">
              {subscription.messageType}
            </p>
          </div>
          <Button variant="ghost" size="icon" className="h-8 w-8 shrink-0" onClick={handleClose}>
            <X className="h-4 w-4" />
          </Button>
        </div>

        {stats && (
          <div className="flex flex-wrap gap-2 mt-3">
            <Badge
              variant={stats.hz > 0 ? 'success' : 'error'}
              className="font-mono"
            >
              {stats.hz.toFixed(1)} Hz
            </Badge>
            {stats.avgLatency !== null && (
              <Badge variant="secondary" className="font-mono">
                {stats.avgLatency.toFixed(1)}ms latency
              </Badge>
            )}
            {stats.jitter !== null && (
              <Badge variant="secondary" className="font-mono">
                ±{stats.jitter.toFixed(1)}ms jitter
              </Badge>
            )}
            <Badge variant="outline" className="font-mono">
              {stats.messageCount} msgs
            </Badge>
          </div>
        )}

        <div className="flex items-center gap-2 mt-3">
          <Button
            variant="outline"
            size="sm"
            onClick={() => setIsPaused(!isPaused)}
            className="gap-1.5"
          >
            {isPaused ? (
              <>
                <Play className="h-3.5 w-3.5" />
                Resume
              </>
            ) : (
              <>
                <Pause className="h-3.5 w-3.5" />
                Pause
              </>
            )}
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={() => clearMessages(subscriptionKey)}
            className="gap-1.5"
          >
            <Trash2 className="h-3.5 w-3.5" />
            Clear
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={handleCopy}
            className="gap-1.5"
            disabled={!latestMessage}
          >
            {copied ? (
              <>
                <Check className="h-3.5 w-3.5" />
                Copied
              </>
            ) : (
              <>
                <Copy className="h-3.5 w-3.5" />
                Copy
              </>
            )}
          </Button>
        </div>
      </CardHeader>

      <CardContent className="flex-1 p-0 overflow-hidden">
        <ScrollArea className="h-full" ref={scrollRef}>
          <div className="p-4">
            {messages.length === 0 ? (
              <p className="text-sm text-[hsl(var(--muted-foreground))] text-center py-8">
                Waiting for messages...
              </p>
            ) : (
              <pre
                className={cn(
                  'text-xs font-mono whitespace-pre-wrap break-all',
                  'bg-[hsl(var(--background))] p-3 rounded-lg'
                )}
              >
                {JSON.stringify(
                  isPaused ? messages[messages.length - 1] : latestMessage,
                  null,
                  2
                )}
              </pre>
            )}
          </div>
        </ScrollArea>
      </CardContent>
    </Card>
  )
}
