import { useEffect } from 'react'
import { useSettingsStore } from '@/stores/settingsStore'

interface ShortcutHandler {
  key: string
  ctrl?: boolean
  shift?: boolean
  alt?: boolean
  handler: () => void
  description: string
}

export function useKeyboardShortcuts(customShortcuts?: ShortcutHandler[]) {
  const { theme, setTheme } = useSettingsStore()

  useEffect(() => {
    const allShortcuts: ShortcutHandler[] = [
      // Theme toggle
      {
        key: 'd',
        ctrl: true,
        shift: true,
        handler: () => {
          setTheme(theme === 'dark' ? 'light' : 'dark')
        },
        description: 'Toggle dark/light theme',
      },
      ...(customShortcuts || []),
    ]

    const handleKeyDown = (e: KeyboardEvent) => {
      // Don't trigger shortcuts when typing in input fields
      if (
        e.target instanceof HTMLInputElement ||
        e.target instanceof HTMLTextAreaElement
      ) {
        return
      }

      for (const shortcut of allShortcuts) {
        const ctrlMatch = shortcut.ctrl ? e.ctrlKey || e.metaKey : !e.ctrlKey && !e.metaKey
        const shiftMatch = shortcut.shift ? e.shiftKey : !e.shiftKey
        const altMatch = shortcut.alt ? e.altKey : !e.altKey
        const keyMatch = e.key.toLowerCase() === shortcut.key.toLowerCase()

        if (ctrlMatch && shiftMatch && altMatch && keyMatch) {
          e.preventDefault()
          shortcut.handler()
          return
        }
      }
    }

    window.addEventListener('keydown', handleKeyDown)
    return () => window.removeEventListener('keydown', handleKeyDown)
  }, [theme, setTheme, customShortcuts])
}

export function getShortcutsList(): { key: string; description: string }[] {
  return [
    { key: 'Ctrl+Shift+D', description: 'Toggle dark/light theme' },
    { key: '1-6', description: 'Switch tabs (Topics, Nodes, Graph, etc.)' },
    { key: 'Ctrl+K', description: 'Focus search' },
    { key: 'Escape', description: 'Close dialogs' },
  ]
}
