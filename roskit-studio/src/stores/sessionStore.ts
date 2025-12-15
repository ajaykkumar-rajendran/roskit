import { create } from 'zustand'
import { persist } from 'zustand/middleware'
import type { RecordedSession, RecordedMessage, SessionSummary } from '@/types/session'

interface SessionStore {
  // Recording state
  isRecording: boolean
  currentSession: RecordedSession | null

  // Saved sessions (summaries for list, full data loaded on demand)
  savedSessions: SessionSummary[]

  // Playback state
  isPlaying: boolean
  playbackSession: RecordedSession | null
  playbackIndex: number
  playbackSpeed: number

  // Recording actions
  startRecording: (robotName: string, robotIp: string, topics: string[], nodes: string[], services: string[]) => void
  stopRecording: () => RecordedSession | null
  addMessage: (topicName: string, messageType: string, data: unknown) => void

  // Session management
  saveSession: (name: string) => void
  loadSession: (id: string) => RecordedSession | null
  deleteSession: (id: string) => void
  exportSession: (id: string) => string | null
  importSession: (json: string) => boolean

  // Playback actions
  startPlayback: (session: RecordedSession) => void
  stopPlayback: () => void
  setPlaybackSpeed: (speed: number) => void
  seekTo: (index: number) => void
  nextMessage: () => RecordedMessage | null
}

const MAX_MESSAGES_PER_SESSION = 10000
const MAX_SAVED_SESSIONS = 20

export const useSessionStore = create<SessionStore>()(
  persist(
    (set, get) => ({
      isRecording: false,
      currentSession: null,
      savedSessions: [],
      isPlaying: false,
      playbackSession: null,
      playbackIndex: 0,
      playbackSpeed: 1,

      startRecording: (robotName, robotIp, topics, nodes, services) => {
        const session: RecordedSession = {
          id: crypto.randomUUID(),
          name: `Session ${new Date().toLocaleString()}`,
          robotName,
          robotIp,
          startTime: Date.now(),
          endTime: null,
          duration: 0,
          topics,
          messageCount: 0,
          messages: [],
          metadata: { nodes, services },
        }
        set({ isRecording: true, currentSession: session })
      },

      stopRecording: () => {
        const { currentSession } = get()
        if (!currentSession) return null

        const endTime = Date.now()
        const completedSession: RecordedSession = {
          ...currentSession,
          endTime,
          duration: endTime - currentSession.startTime,
        }

        set({ isRecording: false, currentSession: null })
        return completedSession
      },

      addMessage: (topicName, messageType, data) => {
        const { currentSession, isRecording } = get()
        if (!isRecording || !currentSession) return
        if (currentSession.messages.length >= MAX_MESSAGES_PER_SESSION) return

        const message: RecordedMessage = {
          timestamp: Date.now(),
          topicName,
          messageType,
          data,
        }

        set({
          currentSession: {
            ...currentSession,
            messages: [...currentSession.messages, message],
            messageCount: currentSession.messageCount + 1,
          },
        })
      },

      saveSession: (name) => {
        const { currentSession, savedSessions } = get()
        const sessionToSave = currentSession || get().playbackSession
        if (!sessionToSave) return

        const namedSession = { ...sessionToSave, name }

        // Store full session in localStorage separately
        localStorage.setItem(`ros2-session-${namedSession.id}`, JSON.stringify(namedSession))

        // Add summary to list
        const summary: SessionSummary = {
          id: namedSession.id,
          name: namedSession.name,
          robotName: namedSession.robotName,
          startTime: namedSession.startTime,
          duration: namedSession.duration,
          messageCount: namedSession.messageCount,
          topicCount: namedSession.topics.length,
        }

        let updatedSessions = [summary, ...savedSessions]
        if (updatedSessions.length > MAX_SAVED_SESSIONS) {
          const removed = updatedSessions.pop()
          if (removed) {
            localStorage.removeItem(`ros2-session-${removed.id}`)
          }
        }

        set({ savedSessions: updatedSessions })
      },

      loadSession: (id) => {
        const stored = localStorage.getItem(`ros2-session-${id}`)
        if (!stored) return null
        try {
          return JSON.parse(stored) as RecordedSession
        } catch {
          return null
        }
      },

      deleteSession: (id) => {
        localStorage.removeItem(`ros2-session-${id}`)
        set((state) => ({
          savedSessions: state.savedSessions.filter((s) => s.id !== id),
        }))
      },

      exportSession: (id) => {
        const session = get().loadSession(id)
        if (!session) return null
        return JSON.stringify(session, null, 2)
      },

      importSession: (json) => {
        try {
          const session = JSON.parse(json) as RecordedSession
          if (!session.id || !session.messages) return false

          // Generate new ID to avoid conflicts
          session.id = crypto.randomUUID()

          localStorage.setItem(`ros2-session-${session.id}`, JSON.stringify(session))

          const summary: SessionSummary = {
            id: session.id,
            name: session.name,
            robotName: session.robotName,
            startTime: session.startTime,
            duration: session.duration,
            messageCount: session.messageCount,
            topicCount: session.topics.length,
          }

          set((state) => ({
            savedSessions: [summary, ...state.savedSessions].slice(0, MAX_SAVED_SESSIONS),
          }))

          return true
        } catch {
          return false
        }
      },

      startPlayback: (session) => {
        set({
          isPlaying: true,
          playbackSession: session,
          playbackIndex: 0,
        })
      },

      stopPlayback: () => {
        set({
          isPlaying: false,
          playbackSession: null,
          playbackIndex: 0,
        })
      },

      setPlaybackSpeed: (speed) => {
        set({ playbackSpeed: speed })
      },

      seekTo: (index) => {
        const { playbackSession } = get()
        if (!playbackSession) return
        const maxIndex = playbackSession.messages.length - 1
        set({ playbackIndex: Math.max(0, Math.min(index, maxIndex)) })
      },

      nextMessage: () => {
        const { playbackSession, playbackIndex } = get()
        if (!playbackSession) return null
        if (playbackIndex >= playbackSession.messages.length) {
          set({ isPlaying: false })
          return null
        }
        const message = playbackSession.messages[playbackIndex]
        set({ playbackIndex: playbackIndex + 1 })
        return message
      },
    }),
    {
      name: 'ros2-debugger-sessions',
      partialize: (state) => ({
        savedSessions: state.savedSessions,
      }),
    }
  )
)
