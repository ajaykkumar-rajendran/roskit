import { useState } from 'react'
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog'
import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { Badge } from '@/components/ui/badge'
import { Plus, Star, Zap, Wifi } from 'lucide-react'
import { useConnectionStore } from '@/stores/connectionStore'
import { useSettingsStore } from '@/stores/settingsStore'

// Parse a URL or IP string and extract components
function parseConnectionInput(input: string): { ip: string; port?: string } {
  const trimmed = input.trim()

  // Check if it's a full URL (ws://, wss://)
  const urlMatch = trimmed.match(/^wss?:\/\/([^:/]+)(?::(\d+))?/)
  if (urlMatch) {
    const [, host, port] = urlMatch
    return { ip: host, port: port || undefined }
  }

  // Check if it's IP:port format
  const ipPortMatch = trimmed.match(/^([^:]+):(\d+)$/)
  if (ipPortMatch) {
    return { ip: ipPortMatch[1], port: ipPortMatch[2] }
  }

  // Just return as-is (plain IP or hostname)
  return { ip: trimmed }
}

export function ConnectionDialog() {
  const [open, setOpen] = useState(false)
  const [ip, setIp] = useState('')
  const [port, setPort] = useState('9090')
  const [name, setName] = useState('')
  const [isConnecting, setIsConnecting] = useState(false)

  const addRobot = useConnectionStore((s) => s.addRobot)
  const { presets, addPreset, defaultPort } = useSettingsStore()

  // Handle IP input changes - auto-parse URLs
  const handleIpChange = (value: string) => {
    const parsed = parseConnectionInput(value)
    setIp(parsed.ip)
    if (parsed.port) {
      setPort(parsed.port)
    }
  }

  const handleConnect = async (saveAsPreset = false) => {
    if (!ip.trim()) return

    setIsConnecting(true)
    try {
      const portNum = parseInt(port) || defaultPort
      await addRobot(ip.trim(), portNum, name.trim() || undefined)

      if (saveAsPreset && name.trim()) {
        const exists = presets.some(
          (p) => p.ip === ip.trim() && p.port === portNum
        )
        if (!exists) {
          addPreset({
            name: name.trim(),
            ip: ip.trim(),
            port: portNum,
          })
        }
      }

      setOpen(false)
      setIp('')
      setPort('9090')
      setName('')
    } finally {
      setIsConnecting(false)
    }
  }

  const handlePresetClick = (preset: typeof presets[0]) => {
    setName(preset.name)
    setIp(preset.ip)
    setPort(preset.port.toString())
  }

  const handleQuickConnect = async (preset: typeof presets[0]) => {
    setIsConnecting(true)
    try {
      await addRobot(preset.ip, preset.port, preset.name)
      setOpen(false)
    } finally {
      setIsConnecting(false)
    }
  }

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        <Button size="sm" className="gap-2">
          <Plus className="h-4 w-4" />
          Add Robot
        </Button>
      </DialogTrigger>
      <DialogContent className="max-w-md">
        <DialogHeader>
          <DialogTitle>Connect to Robot</DialogTitle>
          <DialogDescription>
            Enter the robot's IP address and RosKit bridge port to connect.
          </DialogDescription>
        </DialogHeader>

        {/* Presets */}
        {presets.length > 0 && (
          <div className="border-b border-[hsl(var(--border))] pb-4">
            <div className="flex items-center gap-2 mb-3">
              <Star className="h-4 w-4 text-yellow-500" />
              <span className="text-sm font-medium">Saved Connections</span>
            </div>
            <div className="flex flex-wrap gap-2">
              {presets.map((preset) => (
                <Badge
                  key={preset.id}
                  variant="secondary"
                  className="cursor-pointer hover:bg-[hsl(var(--primary))]/20 py-1.5 px-3 gap-2"
                  onClick={() => handlePresetClick(preset)}
                >
                  <Wifi className="h-3 w-3" />
                  <span>{preset.name}</span>
                  <button
                    className="hover:text-[hsl(var(--primary))]"
                    onClick={(e) => {
                      e.stopPropagation()
                      handleQuickConnect(preset)
                    }}
                    title="Quick connect"
                  >
                    <Zap className="h-3 w-3" />
                  </button>
                </Badge>
              ))}
            </div>
          </div>
        )}

        <div className="grid gap-4 py-4">
          <div className="grid gap-2">
            <label htmlFor="name" className="text-sm font-medium">
              Name (optional)
            </label>
            <Input
              id="name"
              placeholder="My Robot"
              value={name}
              onChange={(e) => setName(e.target.value)}
            />
          </div>
          <div className="grid gap-2">
            <label htmlFor="ip" className="text-sm font-medium">
              IP Address or URL
            </label>
            <Input
              id="ip"
              placeholder="192.168.1.100 or ws://host:port"
              value={ip}
              onChange={(e) => handleIpChange(e.target.value)}
              onKeyDown={(e) => e.key === 'Enter' && handleConnect()}
            />
            <p className="text-xs text-[hsl(var(--muted-foreground))]">
              Paste a full URL (ws://...) to auto-fill port
            </p>
          </div>
          <div className="grid gap-2">
            <label htmlFor="port" className="text-sm font-medium">
              Port
            </label>
            <Input
              id="port"
              placeholder="9090"
              value={port}
              onChange={(e) => setPort(e.target.value)}
              onKeyDown={(e) => e.key === 'Enter' && handleConnect()}
            />
          </div>
        </div>
        <DialogFooter className="flex-col sm:flex-row gap-2">
          <Button variant="outline" onClick={() => setOpen(false)}>
            Cancel
          </Button>
          {name.trim() && !presets.some((p) => p.ip === ip && p.port === parseInt(port)) && (
            <Button
              variant="secondary"
              onClick={() => handleConnect(true)}
              disabled={!ip.trim() || isConnecting}
            >
              <Star className="h-4 w-4 mr-2" />
              Connect & Save
            </Button>
          )}
          <Button onClick={() => handleConnect(false)} disabled={!ip.trim() || isConnecting}>
            {isConnecting ? 'Connecting...' : 'Connect'}
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  )
}
