import { useState, useEffect } from 'react'
import { Keyboard } from 'lucide-react'
import { Button } from '@/components/ui/button'
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog'
import { Badge } from '@/components/ui/badge'

interface Shortcut {
  keys: string[]
  description: string
  category: string
}

const shortcuts: Shortcut[] = [
  // Navigation
  { keys: ['1'], description: 'Go to Topics tab', category: 'Navigation' },
  { keys: ['2'], description: 'Go to Nodes tab', category: 'Navigation' },
  { keys: ['3'], description: 'Go to Graph tab', category: 'Navigation' },
  { keys: ['4'], description: 'Go to Benchmark tab', category: 'Navigation' },
  { keys: ['5'], description: 'Go to Services tab', category: 'Navigation' },
  { keys: ['6'], description: 'Go to Behavior Tree tab', category: 'Navigation' },

  // Actions
  { keys: ['Ctrl', 'K'], description: 'Focus search input', category: 'Actions' },
  { keys: ['Ctrl', 'Shift', 'D'], description: 'Toggle dark/light mode', category: 'Actions' },
  { keys: ['Ctrl', 'Shift', 'C'], description: 'Open connection dialog', category: 'Actions' },
  { keys: ['?'], description: 'Show keyboard shortcuts', category: 'Actions' },

  // General
  { keys: ['Escape'], description: 'Close dialog/panel', category: 'General' },
]

export function KeyboardShortcutsDialog() {
  const [open, setOpen] = useState(false)

  // Open with ? key
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (
        e.key === '?' &&
        !(e.target instanceof HTMLInputElement) &&
        !(e.target instanceof HTMLTextAreaElement)
      ) {
        e.preventDefault()
        setOpen(true)
      }
    }

    window.addEventListener('keydown', handleKeyDown)
    return () => window.removeEventListener('keydown', handleKeyDown)
  }, [])

  const categories = [...new Set(shortcuts.map((s) => s.category))]

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        <Button variant="ghost" size="icon" title="Keyboard shortcuts (?)">
          <Keyboard className="h-5 w-5" />
        </Button>
      </DialogTrigger>
      <DialogContent className="max-w-lg">
        <DialogHeader>
          <DialogTitle className="flex items-center gap-2">
            <Keyboard className="h-5 w-5" />
            Keyboard Shortcuts
          </DialogTitle>
        </DialogHeader>

        <div className="mt-4 space-y-6">
          {categories.map((category) => (
            <div key={category}>
              <h3 className="text-sm font-semibold text-[hsl(var(--muted-foreground))] mb-3">
                {category}
              </h3>
              <div className="space-y-2">
                {shortcuts
                  .filter((s) => s.category === category)
                  .map((shortcut, index) => (
                    <div
                      key={index}
                      className="flex items-center justify-between py-1.5"
                    >
                      <span className="text-sm">{shortcut.description}</span>
                      <div className="flex items-center gap-1">
                        {shortcut.keys.map((key, keyIndex) => (
                          <span key={keyIndex} className="flex items-center gap-1">
                            <Badge
                              variant="secondary"
                              className="font-mono text-xs px-2 py-0.5"
                            >
                              {key}
                            </Badge>
                            {keyIndex < shortcut.keys.length - 1 && (
                              <span className="text-[hsl(var(--muted-foreground))]">+</span>
                            )}
                          </span>
                        ))}
                      </div>
                    </div>
                  ))}
              </div>
            </div>
          ))}
        </div>

        <p className="text-xs text-[hsl(var(--muted-foreground))] mt-4">
          Press <Badge variant="secondary" className="text-xs mx-1">?</Badge> anytime to show this dialog
        </p>
      </DialogContent>
    </Dialog>
  )
}
