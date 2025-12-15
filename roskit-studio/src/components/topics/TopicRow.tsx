import { useState } from 'react'
import { Radio, Eye, EyeOff, Info, Send } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import {
  Tooltip,
  TooltipContent,
  TooltipTrigger,
  TooltipProvider,
} from '@/components/ui/tooltip'
import {
  Dialog,
  DialogContent,
  DialogTrigger,
} from '@/components/ui/dialog'
import { cn } from '@/lib/utils/cn'
import { useConnectionStore } from '@/stores/connectionStore'
import { useTopicStore } from '@/stores/topicStore'
import { TopicPublisher } from './TopicPublisher'
import { TopicRecorder } from './TopicRecorder'
import type { TopicInfo } from '@/types/ros'

interface TopicRowProps {
  topic: TopicInfo
  robotId: string
}

export function TopicRow({ topic, robotId }: TopicRowProps) {
  const [showPublisher, setShowPublisher] = useState(false)
  const getClient = useConnectionStore((s) => s.getClient)
  const { subscriptions, subscribeRosKit, unsubscribe } = useTopicStore()

  const client = getClient(robotId)

  const subscriptionKey = `${robotId}:${topic.name}`
  const subscription = subscriptions[subscriptionKey]
  const isSubscribed = subscription?.isSubscribed ?? false
  const stats = subscription?.stats

  const handleToggleSubscribe = async () => {
    if (isSubscribed) {
      unsubscribe(subscriptionKey)
    } else if (client) {
      await subscribeRosKit(client, robotId, topic.name, topic.type)
    }
  }

  const getHzBadgeVariant = (hz: number) => {
    if (hz === 0) return 'error'
    if (hz < 1) return 'warning'
    return 'success'
  }

  return (
    <div
      className={cn(
        'p-3 hover:bg-[hsl(var(--accent))]/50 transition-colors',
        isSubscribed && 'bg-[hsl(var(--primary))]/5'
      )}
    >
      <div className="flex items-start justify-between gap-2">
        <div className="min-w-0 flex-1">
          <div className="flex items-center gap-2">
            <Radio
              className={cn(
                'h-4 w-4 shrink-0',
                isSubscribed ? 'text-green-400' : 'text-[hsl(var(--muted-foreground))]'
              )}
            />
            <span className="font-mono text-sm truncate">{topic.name}</span>
          </div>
          <p className="mt-1 text-xs text-[hsl(var(--muted-foreground))] truncate pl-6">
            {topic.type}
          </p>
        </div>

        <div className="flex items-center gap-2 shrink-0">
          {isSubscribed && (
            <TopicRecorder
              subscriptionKey={subscriptionKey}
              topicName={topic.name}
              messageType={topic.type}
            />
          )}

          {isSubscribed && stats && (
            <Badge
              variant={getHzBadgeVariant(stats.hz)}
              className="font-mono text-xs"
            >
              {stats.hz.toFixed(1)} Hz
            </Badge>
          )}

          <TooltipProvider>
            <Tooltip>
              <TooltipTrigger asChild>
                <Button
                  variant={isSubscribed ? 'default' : 'outline'}
                  size="sm"
                  onClick={handleToggleSubscribe}
                  className="gap-1.5"
                >
                  {isSubscribed ? (
                    <>
                      <EyeOff className="h-3.5 w-3.5" />
                      <span className="hidden sm:inline">Unsubscribe</span>
                    </>
                  ) : (
                    <>
                      <Eye className="h-3.5 w-3.5" />
                      <span className="hidden sm:inline">Subscribe</span>
                    </>
                  )}
                </Button>
              </TooltipTrigger>
              <TooltipContent>
                {isSubscribed ? 'Stop receiving messages' : 'Start receiving messages'}
              </TooltipContent>
            </Tooltip>
          </TooltipProvider>

          <Dialog open={showPublisher} onOpenChange={setShowPublisher}>
            <TooltipProvider>
              <Tooltip>
                <TooltipTrigger asChild>
                  <DialogTrigger asChild>
                    <Button variant="ghost" size="icon" className="h-8 w-8">
                      <Send className="h-4 w-4" />
                    </Button>
                  </DialogTrigger>
                </TooltipTrigger>
                <TooltipContent>Publish message</TooltipContent>
              </Tooltip>
            </TooltipProvider>
            <DialogContent className="p-0 border-0 bg-transparent shadow-none">
              <TopicPublisher
                topicName={topic.name}
                messageType={topic.type}
                robotId={robotId}
                onClose={() => setShowPublisher(false)}
              />
            </DialogContent>
          </Dialog>

          <TooltipProvider>
            <Tooltip>
              <TooltipTrigger asChild>
                <Button variant="ghost" size="icon" className="h-8 w-8">
                  <Info className="h-4 w-4" />
                </Button>
              </TooltipTrigger>
              <TooltipContent side="left" className="max-w-xs">
                <div className="space-y-1">
                  <p className="font-semibold">{topic.name}</p>
                  <p className="text-xs opacity-80">Type: {topic.type}</p>
                  <p className="text-xs opacity-80">Protocol: RosKit</p>
                  {stats && (
                    <>
                      <p className="text-xs opacity-80">
                        Messages: {stats.messageCount}
                      </p>
                      {stats.avgLatency !== null && (
                        <p className="text-xs opacity-80">
                          Latency: {stats.avgLatency.toFixed(1)}ms
                        </p>
                      )}
                    </>
                  )}
                </div>
              </TooltipContent>
            </Tooltip>
          </TooltipProvider>
        </div>
      </div>

      {isSubscribed && stats && (
        <div className="mt-2 pl-6 flex flex-wrap gap-x-4 gap-y-1 text-xs text-[hsl(var(--muted-foreground))]">
          <span>Messages: {stats.messageCount}</span>
          {stats.avgLatency !== null && (
            <span>Latency: {stats.avgLatency.toFixed(1)}ms</span>
          )}
          {stats.jitter !== null && <span>Jitter: {stats.jitter.toFixed(1)}ms</span>}
          {stats.bytesPerSecond !== null && (
            <span>
              {stats.bytesPerSecond > 1024 * 1024
                ? `${(stats.bytesPerSecond / (1024 * 1024)).toFixed(1)} MB/s`
                : stats.bytesPerSecond > 1024
                ? `${(stats.bytesPerSecond / 1024).toFixed(1)} KB/s`
                : `${stats.bytesPerSecond} B/s`}
            </span>
          )}
        </div>
      )}
    </div>
  )
}
