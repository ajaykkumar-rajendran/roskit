import { create } from 'zustand'
import type { RosKitClient, Subscription as RosKitSubscription } from '@roskit/client'
import type { TopicStats, BenchmarkHistory } from '@/types/benchmark'
import {
  createInitialStats,
  updateStats,
  addToHistory,
  estimateMessageSize,
  extractMessageStamp,
  extractSequenceId,
  calculateLatency,
  type MessageTimestamp,
} from '@/lib/benchmark/calculator'

interface TopicSubscription {
  topicName: string
  messageType: string
  robotId: string
  roskitSub: RosKitSubscription | null
  messages: unknown[]
  stats: TopicStats
  history: BenchmarkHistory[]
  timestamps: MessageTimestamp[]
  isSubscribed: boolean
}

interface TopicStore {
  subscriptions: Record<string, TopicSubscription>
  subscribeRosKit: (
    client: RosKitClient,
    robotId: string,
    topicName: string,
    messageType: string
  ) => Promise<void>
  unsubscribe: (key: string) => void
  unsubscribeAll: (robotId: string) => void
  getSubscription: (key: string) => TopicSubscription | null
  clearMessages: (key: string) => void
}

const MAX_MESSAGES = 50
const HISTORY_UPDATE_INTERVAL = 1000

export const useTopicStore = create<TopicStore>((set, get) => ({
  subscriptions: {},

  subscribeRosKit: async (client, robotId, topicName, messageType) => {
    const key = `${robotId}:${topicName}`

    if (get().subscriptions[key]?.isSubscribed) {
      return
    }

    const subscription: TopicSubscription = {
      topicName,
      messageType,
      robotId,
      roskitSub: null,
      messages: [],
      stats: createInitialStats(),
      history: [],
      timestamps: [],
      isSubscribed: true,
    }

    set((state) => ({
      subscriptions: { ...state.subscriptions, [key]: subscription },
    }))

    let lastHistoryUpdate = Date.now()

    try {
      const roskitSub = await client.subscribe(
        topicName,
        (message: unknown, _timestamp: bigint) => {
          const now = Date.now()
          const messageStamp = extractMessageStamp(message)
          const byteSize = estimateMessageSize(message)
          const sequenceId = extractSequenceId(message)

          // Use RosKit timestamp if available, otherwise fall back to message stamp
          const latency = messageStamp
            ? calculateLatency(now, messageStamp as { sec: number; nanosec?: number })
            : null

          const newTimestamp: MessageTimestamp = {
            receivedAt: now,
            messageStamp: latency !== null ? now - latency : undefined,
            byteSize,
            sequenceId,
          }

          set((state) => {
            const sub = state.subscriptions[key]
            if (!sub) return state

            const { timestamps, stats } = updateStats(sub.timestamps, newTimestamp)

            const messages = [...sub.messages, message]
            if (messages.length > MAX_MESSAGES) {
              messages.shift()
            }

            let history = sub.history
            if (now - lastHistoryUpdate > HISTORY_UPDATE_INTERVAL) {
              history = addToHistory(history, stats)
              lastHistoryUpdate = now
            }

            return {
              subscriptions: {
                ...state.subscriptions,
                [key]: {
                  ...sub,
                  messages,
                  stats,
                  history,
                  timestamps,
                },
              },
            }
          })
        },
        { msgType: messageType }
      )

      // Store the subscription handle
      set((state) => ({
        subscriptions: {
          ...state.subscriptions,
          [key]: {
            ...state.subscriptions[key],
            roskitSub,
          },
        },
      }))
    } catch (error) {
      console.error(`Failed to subscribe to ${topicName}:`, error)
      // Mark as not subscribed on error
      set((state) => ({
        subscriptions: {
          ...state.subscriptions,
          [key]: {
            ...state.subscriptions[key],
            isSubscribed: false,
          },
        },
      }))
    }
  },

  unsubscribe: (key: string) => {
    const subscription = get().subscriptions[key]

    if (subscription?.roskitSub) {
      subscription.roskitSub.unsubscribe()
    }

    set((state) => ({
      subscriptions: {
        ...state.subscriptions,
        [key]: {
          ...state.subscriptions[key],
          isSubscribed: false,
          roskitSub: null,
        },
      },
    }))
  },

  unsubscribeAll: (robotId: string) => {
    const subscriptions = get().subscriptions
    const keysToUnsubscribe = Object.keys(subscriptions).filter((key) =>
      key.startsWith(`${robotId}:`)
    )

    keysToUnsubscribe.forEach((key) => {
      get().unsubscribe(key)
    })
  },

  getSubscription: (key: string) => {
    return get().subscriptions[key] || null
  },

  clearMessages: (key: string) => {
    set((state) => ({
      subscriptions: {
        ...state.subscriptions,
        [key]: {
          ...state.subscriptions[key],
          messages: [],
          timestamps: [],
          stats: createInitialStats(),
          history: [],
        },
      },
    }))
  },
}))
