import { useState, useEffect } from 'react'
import {
  Circle,
  Square,
  Save,
  Trash2,
  Download,
  Upload,
  Play,
  Clock,
  Database,
} from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { ScrollArea } from '@/components/ui/scroll-area'
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogFooter,
} from '@/components/ui/dialog'
import { useSessionStore } from '@/stores/sessionStore'
import { useConnectionStore } from '@/stores/connectionStore'
import { useTopicStore } from '@/stores/topicStore'

export function SessionRecorder() {
  const [saveDialogOpen, setSaveDialogOpen] = useState(false)
  const [sessionName, setSessionName] = useState('')
  const [recordingTime, setRecordingTime] = useState(0)

  const { robots, activeRobotId } = useConnectionStore()
  const subscriptions = useTopicStore((s) => s.subscriptions)
  const {
    isRecording,
    currentSession,
    savedSessions,
    startRecording,
    stopRecording,
    saveSession,
    deleteSession,
    exportSession,
    loadSession,
    startPlayback,
  } = useSessionStore()

  const activeRobot = activeRobotId ? robots[activeRobotId] : null

  // Update recording time
  useEffect(() => {
    if (!isRecording || !currentSession) {
      setRecordingTime(0)
      return
    }

    const interval = setInterval(() => {
      setRecordingTime(Date.now() - currentSession.startTime)
    }, 100)

    return () => clearInterval(interval)
  }, [isRecording, currentSession])

  const handleStartRecording = () => {
    if (!activeRobot) return

    const activeTopics = Object.entries(subscriptions)
      .filter(([key, sub]) => key.startsWith(`${activeRobotId}:`) && sub.isSubscribed)
      .map(([, sub]) => sub.topicName)

    startRecording(
      activeRobot.name,
      activeRobot.ip,
      activeTopics,
      activeRobot.nodes,
      activeRobot.services
    )
  }

  const handleStopRecording = () => {
    const session = stopRecording()
    if (session && session.messageCount > 0) {
      setSaveDialogOpen(true)
      setSessionName(`Session ${new Date().toLocaleDateString()} ${new Date().toLocaleTimeString()}`)
    }
  }

  const handleSave = () => {
    saveSession(sessionName)
    setSaveDialogOpen(false)
    setSessionName('')
  }

  const handleExport = (id: string) => {
    const json = exportSession(id)
    if (!json) return

    const blob = new Blob([json], { type: 'application/json' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `ros2-session-${id}.json`
    a.click()
    URL.revokeObjectURL(url)
  }

  const handleImport = (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0]
    if (!file) return

    const reader = new FileReader()
    reader.onload = (e) => {
      const json = e.target?.result as string
      useSessionStore.getState().importSession(json)
    }
    reader.readAsText(file)
  }

  const handlePlaySession = (id: string) => {
    const session = loadSession(id)
    if (session) {
      startPlayback(session)
    }
  }

  const formatDuration = (ms: number) => {
    const seconds = Math.floor(ms / 1000)
    const minutes = Math.floor(seconds / 60)
    const hours = Math.floor(minutes / 60)

    if (hours > 0) {
      return `${hours}:${(minutes % 60).toString().padStart(2, '0')}:${(seconds % 60).toString().padStart(2, '0')}`
    }
    return `${minutes}:${(seconds % 60).toString().padStart(2, '0')}`
  }

  return (
    <Card>
      <CardHeader className="pb-3">
        <div className="flex items-center justify-between">
          <CardTitle className="text-base flex items-center gap-2">
            <Database className="h-4 w-4" />
            Session Recording
          </CardTitle>
          <label className="cursor-pointer">
            <Button variant="ghost" size="sm" asChild>
              <span>
                <Upload className="h-4 w-4 mr-1" />
                Import
              </span>
            </Button>
            <input
              type="file"
              accept=".json"
              className="hidden"
              onChange={handleImport}
            />
          </label>
        </div>
      </CardHeader>
      <CardContent className="space-y-4">
        {/* Recording Controls */}
        <div className="flex items-center gap-3">
          {!isRecording ? (
            <Button
              onClick={handleStartRecording}
              disabled={!activeRobot || activeRobot.status !== 'connected'}
              className="gap-2"
              variant="destructive"
            >
              <Circle className="h-4 w-4 fill-current" />
              Record
            </Button>
          ) : (
            <Button onClick={handleStopRecording} variant="secondary" className="gap-2">
              <Square className="h-4 w-4 fill-current" />
              Stop
            </Button>
          )}

          {isRecording && currentSession && (
            <div className="flex items-center gap-3 text-sm">
              <Badge variant="destructive" className="gap-1 animate-pulse">
                <Circle className="h-2 w-2 fill-current" />
                Recording
              </Badge>
              <span className="font-mono">{formatDuration(recordingTime)}</span>
              <span className="text-[hsl(var(--muted-foreground))]">
                {currentSession.messageCount} messages
              </span>
            </div>
          )}
        </div>

        {/* Saved Sessions */}
        {savedSessions.length > 0 && (
          <div className="space-y-2">
            <h4 className="text-sm font-medium">Saved Sessions</h4>
            <ScrollArea className="h-[200px]">
              <div className="space-y-2 pr-4">
                {savedSessions.map((session) => (
                  <div
                    key={session.id}
                    className="flex items-center justify-between p-2 rounded-lg bg-[hsl(var(--muted))] hover:bg-[hsl(var(--accent))]"
                  >
                    <div className="min-w-0 flex-1">
                      <p className="text-sm font-medium truncate">{session.name}</p>
                      <div className="flex items-center gap-2 text-xs text-[hsl(var(--muted-foreground))]">
                        <span>{session.robotName}</span>
                        <span>•</span>
                        <span className="flex items-center gap-1">
                          <Clock className="h-3 w-3" />
                          {formatDuration(session.duration)}
                        </span>
                        <span>•</span>
                        <span>{session.messageCount} msgs</span>
                      </div>
                    </div>
                    <div className="flex items-center gap-1">
                      <Button
                        variant="ghost"
                        size="icon"
                        className="h-7 w-7"
                        onClick={() => handlePlaySession(session.id)}
                        title="Replay session"
                      >
                        <Play className="h-3.5 w-3.5" />
                      </Button>
                      <Button
                        variant="ghost"
                        size="icon"
                        className="h-7 w-7"
                        onClick={() => handleExport(session.id)}
                        title="Export session"
                      >
                        <Download className="h-3.5 w-3.5" />
                      </Button>
                      <Button
                        variant="ghost"
                        size="icon"
                        className="h-7 w-7 text-red-500 hover:text-red-600"
                        onClick={() => deleteSession(session.id)}
                        title="Delete session"
                      >
                        <Trash2 className="h-3.5 w-3.5" />
                      </Button>
                    </div>
                  </div>
                ))}
              </div>
            </ScrollArea>
          </div>
        )}

        {savedSessions.length === 0 && !isRecording && (
          <p className="text-sm text-[hsl(var(--muted-foreground))] text-center py-4">
            No saved sessions. Start recording to capture topic messages.
          </p>
        )}

        {/* Save Dialog */}
        <Dialog open={saveDialogOpen} onOpenChange={setSaveDialogOpen}>
          <DialogContent>
            <DialogHeader>
              <DialogTitle>Save Recording</DialogTitle>
            </DialogHeader>
            <div className="py-4">
              <label className="text-sm font-medium">Session Name</label>
              <Input
                value={sessionName}
                onChange={(e) => setSessionName(e.target.value)}
                placeholder="Enter session name..."
                className="mt-2"
              />
            </div>
            <DialogFooter>
              <Button variant="outline" onClick={() => setSaveDialogOpen(false)}>
                Discard
              </Button>
              <Button onClick={handleSave}>
                <Save className="h-4 w-4 mr-2" />
                Save Session
              </Button>
            </DialogFooter>
          </DialogContent>
        </Dialog>
      </CardContent>
    </Card>
  )
}
