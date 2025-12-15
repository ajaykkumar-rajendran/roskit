import { create } from 'zustand'
import { persist } from 'zustand/middleware'

export type Theme = 'light' | 'dark' | 'system'

export interface ConnectionPreset {
  id: string
  name: string
  ip: string
  port: number
  createdAt: number
}

interface SettingsStore {
  // Theme
  theme: Theme
  setTheme: (theme: Theme) => void

  // Default port for RosKit connections
  defaultPort: number
  setDefaultPort: (port: number) => void

  // Connection Presets
  presets: ConnectionPreset[]
  addPreset: (preset: Omit<ConnectionPreset, 'id' | 'createdAt'>) => void
  removePreset: (id: string) => void
  updatePreset: (id: string, updates: Partial<ConnectionPreset>) => void

  // UI Preferences
  sidebarCollapsed: boolean
  setSidebarCollapsed: (collapsed: boolean) => void
  showLiveDataPanel: boolean
  setShowLiveDataPanel: (show: boolean) => void
  defaultTab: string
  setDefaultTab: (tab: string) => void

  // Message Display
  maxMessagesPerTopic: number
  setMaxMessagesPerTopic: (max: number) => void
  jsonExpandDepth: number
  setJsonExpandDepth: (depth: number) => void
}

export const useSettingsStore = create<SettingsStore>()(
  persist(
    (set) => ({
      // Theme
      theme: 'dark',
      setTheme: (theme) => {
        set({ theme })
        applyTheme(theme)
      },

      // Default port for RosKit connections
      defaultPort: 9090,
      setDefaultPort: (port) => set({ defaultPort: port }),

      // Connection Presets
      presets: [],
      addPreset: (preset) =>
        set((state) => ({
          presets: [
            ...state.presets,
            {
              ...preset,
              id: crypto.randomUUID(),
              createdAt: Date.now(),
            },
          ],
        })),
      removePreset: (id) =>
        set((state) => ({
          presets: state.presets.filter((p) => p.id !== id),
        })),
      updatePreset: (id, updates) =>
        set((state) => ({
          presets: state.presets.map((p) =>
            p.id === id ? { ...p, ...updates } : p
          ),
        })),

      // UI Preferences
      sidebarCollapsed: false,
      setSidebarCollapsed: (collapsed) => set({ sidebarCollapsed: collapsed }),
      showLiveDataPanel: true,
      setShowLiveDataPanel: (show) => set({ showLiveDataPanel: show }),
      defaultTab: 'topics',
      setDefaultTab: (tab) => set({ defaultTab: tab }),

      // Message Display
      maxMessagesPerTopic: 100,
      setMaxMessagesPerTopic: (max) => set({ maxMessagesPerTopic: max }),
      jsonExpandDepth: 2,
      setJsonExpandDepth: (depth) => set({ jsonExpandDepth: depth }),
    }),
    {
      name: 'ros2-debugger-settings',
      onRehydrateStorage: () => (state) => {
        if (state) {
          applyTheme(state.theme)
        }
      },
    }
  )
)

function applyTheme(theme: Theme) {
  const root = document.documentElement
  const systemDark = window.matchMedia('(prefers-color-scheme: dark)').matches

  if (theme === 'dark' || (theme === 'system' && systemDark)) {
    root.classList.add('dark')
  } else {
    root.classList.remove('dark')
  }
}

// Initialize theme on load
if (typeof window !== 'undefined') {
  const stored = localStorage.getItem('ros2-debugger-settings')
  if (stored) {
    try {
      const { state } = JSON.parse(stored)
      applyTheme(state?.theme || 'dark')
    } catch {
      applyTheme('dark')
    }
  }
}
