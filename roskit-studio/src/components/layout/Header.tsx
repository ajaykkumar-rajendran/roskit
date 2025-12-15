import { Bot, Github } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { ThemeToggle } from '@/components/settings/ThemeToggle'
import { SettingsDialog } from '@/components/settings/SettingsDialog'
import { KeyboardShortcutsDialog } from '@/components/settings/KeyboardShortcutsDialog'

export function Header() {
  return (
    <header className="h-14 border-b border-[hsl(var(--border))] bg-[hsl(var(--card))] flex items-center justify-between px-4">
      <div className="flex items-center gap-3">
        <div className="flex items-center gap-2">
          <Bot className="h-6 w-6 text-[hsl(var(--primary))]" />
          <h1 className="font-semibold text-lg">ROS2 Debugger</h1>
        </div>
        <span className="text-xs text-[hsl(var(--muted-foreground))] bg-[hsl(var(--muted))] px-2 py-0.5 rounded">
          v0.1.0
        </span>
      </div>

      <div className="flex items-center gap-1">
        <KeyboardShortcutsDialog />
        <ThemeToggle />
        <SettingsDialog />
        <Button variant="ghost" size="icon" asChild>
          <a
            href="https://github.com"
            target="_blank"
            rel="noopener noreferrer"
          >
            <Github className="h-5 w-5" />
          </a>
        </Button>
      </div>
    </header>
  )
}
