import { useState, useEffect, useRef } from 'react'
import {
  Circle,
  Square,
  Download,
  Trash2,
  Clock,
  HardDrive,
} from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogFooter,
} from '@/components/ui/dialog'
import { useTopicStore } from '@/stores/topicStore'

interface RecordedMessage {
  timestamp: number
  data: unknown
}

interface TopicRecording {
  topicName: string
  messageType: string
  startTime: number
  messages: RecordedMessage[]
}

interface TopicRecorderProps {
  subscriptionKey: string
  topicName: string
  messageType: string
}

export function TopicRecorder({ subscriptionKey, topicName, messageType }: TopicRecorderProps) {
  const [isRecording, setIsRecording] = useState(false)
  const [recording, setRecording] = useState<TopicRecording | null>(null)
  const [showExportDialog, setShowExportDialog] = useState(false)
  const [recordingTime, setRecordingTime] = useState(0)

  const subscriptions = useTopicStore((s) => s.subscriptions)
  const subscription = subscriptions[subscriptionKey]
  const lastMessageRef = useRef<unknown>(null)
  const startTimeRef = useRef<number>(0)

  // Track recording time
  useEffect(() => {
    if (!isRecording) {
      setRecordingTime(0)
      return
    }

    const interval = setInterval(() => {
      setRecordingTime(Date.now() - startTimeRef.current)
    }, 100)

    return () => clearInterval(interval)
  }, [isRecording])

  // Record messages when subscribed and recording
  useEffect(() => {
    if (!isRecording || !subscription?.messages?.length) return

    const currentMessages = subscription.messages
    const latestMessage = currentMessages[currentMessages.length - 1]

    if (latestMessage && latestMessage !== lastMessageRef.current) {
      lastMessageRef.current = latestMessage

      setRecording((prev) => {
        if (!prev) return prev
        return {
          ...prev,
          messages: [...prev.messages, {
            timestamp: Date.now(),
            data: latestMessage,
          }],
        }
      })
    }
  }, [isRecording, subscription?.messages])

  const handleStartRecording = () => {
    startTimeRef.current = Date.now()
    setRecording({
      topicName,
      messageType,
      startTime: Date.now(),
      messages: [],
    })
    setIsRecording(true)
  }

  const handleStopRecording = () => {
    setIsRecording(false)
    if (recording && recording.messages.length > 0) {
      setShowExportDialog(true)
    }
  }

  const handleExport = (format: 'json' | 'csv') => {
    if (!recording) return

    let content: string
    let filename: string

    if (format === 'json') {
      content = JSON.stringify({
        topicName: recording.topicName,
        messageType: recording.messageType,
        startTime: recording.startTime,
        duration: recording.messages.length > 0
          ? recording.messages[recording.messages.length - 1].timestamp - recording.startTime
          : 0,
        messageCount: recording.messages.length,
        messages: recording.messages.map((m) => ({
          timestamp: m.timestamp,
          relativeTime: m.timestamp - recording.startTime,
          data: m.data,
        })),
      }, null, 2)
      filename = `${topicName.replace(/\//g, '_')}_${Date.now()}.json`
    } else {
      // CSV format
      const headers = ['timestamp', 'relative_time_ms', 'data']
      const rows = recording.messages.map((m) => [
        m.timestamp,
        m.timestamp - recording.startTime,
        JSON.stringify(m.data),
      ])
      content = [headers.join(','), ...rows.map((r) => r.join(','))].join('\n')
      filename = `${topicName.replace(/\//g, '_')}_${Date.now()}.csv`
    }

    const blob = new Blob([content], { type: format === 'json' ? 'application/json' : 'text/csv' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = filename
    a.click()
    URL.revokeObjectURL(url)

    setShowExportDialog(false)
  }

  const handleDiscard = () => {
    setRecording(null)
    setShowExportDialog(false)
  }

  const formatDuration = (ms: number) => {
    const seconds = Math.floor(ms / 1000)
    const minutes = Math.floor(seconds / 60)
    return `${minutes}:${(seconds % 60).toString().padStart(2, '0')}`
  }

  const estimateSize = () => {
    if (!recording) return '0 B'
    const bytes = JSON.stringify(recording.messages).length
    if (bytes > 1024 * 1024) return `${(bytes / (1024 * 1024)).toFixed(1)} MB`
    if (bytes > 1024) return `${(bytes / 1024).toFixed(1)} KB`
    return `${bytes} B`
  }

  if (!subscription?.isSubscribed) {
    return null
  }

  return (
    <>
      <div className="flex items-center gap-2">
        {!isRecording ? (
          <Button
            variant="ghost"
            size="icon"
            className="h-7 w-7 text-red-500 hover:text-red-600 hover:bg-red-500/10"
            onClick={handleStartRecording}
            title="Start recording"
          >
            <Circle className="h-3.5 w-3.5 fill-current" />
          </Button>
        ) : (
          <Button
            variant="ghost"
            size="icon"
            className="h-7 w-7"
            onClick={handleStopRecording}
            title="Stop recording"
          >
            <Square className="h-3.5 w-3.5 fill-current" />
          </Button>
        )}

        {isRecording && recording && (
          <div className="flex items-center gap-2 text-xs">
            <Badge variant="destructive" className="gap-1 animate-pulse">
              <Circle className="h-2 w-2 fill-current" />
              REC
            </Badge>
            <span className="font-mono text-[hsl(var(--muted-foreground))]">
              {formatDuration(recordingTime)}
            </span>
            <span className="text-[hsl(var(--muted-foreground))]">
              {recording.messages.length} msgs
            </span>
          </div>
        )}
      </div>

      <Dialog open={showExportDialog} onOpenChange={setShowExportDialog}>
        <DialogContent>
          <DialogHeader>
            <DialogTitle>Export Recording</DialogTitle>
          </DialogHeader>
          <div className="py-4 space-y-4">
            <div className="space-y-2">
              <p className="text-sm font-medium">{topicName}</p>
              <div className="flex items-center gap-4 text-sm text-[hsl(var(--muted-foreground))]">
                <span className="flex items-center gap-1">
                  <Clock className="h-4 w-4" />
                  {recording && formatDuration(
                    recording.messages.length > 0
                      ? recording.messages[recording.messages.length - 1].timestamp - recording.startTime
                      : 0
                  )}
                </span>
                <span>{recording?.messages.length || 0} messages</span>
                <span className="flex items-center gap-1">
                  <HardDrive className="h-4 w-4" />
                  {estimateSize()}
                </span>
              </div>
            </div>

            <div className="flex gap-2">
              <Button
                variant="outline"
                className="flex-1"
                onClick={() => handleExport('json')}
              >
                <Download className="h-4 w-4 mr-2" />
                Export JSON
              </Button>
              <Button
                variant="outline"
                className="flex-1"
                onClick={() => handleExport('csv')}
              >
                <Download className="h-4 w-4 mr-2" />
                Export CSV
              </Button>
            </div>
          </div>
          <DialogFooter>
            <Button variant="ghost" onClick={handleDiscard}>
              <Trash2 className="h-4 w-4 mr-2" />
              Discard
            </Button>
          </DialogFooter>
        </DialogContent>
      </Dialog>
    </>
  )
}
