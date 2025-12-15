import { useEffect } from 'react'
import { Header } from '@/components/layout/Header'
import { ConnectionSidebar } from '@/components/connection/ConnectionSidebar'
import { MainContent } from '@/components/layout/MainContent'
import { TooltipProvider } from '@/components/ui/tooltip'
import { useKeyboardShortcuts } from '@/hooks/useKeyboardShortcuts'
import { useSettingsStore } from '@/stores/settingsStore'
import { useConnectionStore } from '@/stores/connectionStore'

function App() {
  const theme = useSettingsStore((s) => s.theme)
  const reconnectSavedRobots = useConnectionStore((s) => s.reconnectSavedRobots)

  // Apply theme on mount and changes
  useEffect(() => {
    const root = document.documentElement
    const systemDark = window.matchMedia('(prefers-color-scheme: dark)').matches

    if (theme === 'dark' || (theme === 'system' && systemDark)) {
      root.classList.add('dark')
    } else {
      root.classList.remove('dark')
    }
  }, [theme])

  // Reconnect to saved robots on mount
  useEffect(() => {
    reconnectSavedRobots()
  }, [reconnectSavedRobots])

  // Enable keyboard shortcuts
  useKeyboardShortcuts()

  return (
    <TooltipProvider>
      <div className="h-screen flex flex-col bg-[hsl(var(--background))]">
        <Header />
        <div className="flex-1 flex overflow-hidden min-h-0">
          <div className="w-[280px] shrink-0 h-full">
            <ConnectionSidebar />
          </div>
          <MainContent />
        </div>
      </div>
    </TooltipProvider>
  )
}

export default App
