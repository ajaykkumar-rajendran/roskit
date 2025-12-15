import { useState } from 'react'
import { Settings, Moon, Sun, Monitor, Trash2, Plus, Wifi } from 'lucide-react'
import { Button } from '@/components/ui/button'
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Label } from '@/components/ui/label'
import { Input } from '@/components/ui/input'
import { Switch } from '@/components/ui/switch'
import { Card } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { useSettingsStore, type Theme } from '@/stores/settingsStore'
import { cn } from '@/lib/utils/cn'

export function SettingsDialog() {
  const [open, setOpen] = useState(false)
  const {
    theme,
    setTheme,
    presets,
    addPreset,
    removePreset,
    showLiveDataPanel,
    setShowLiveDataPanel,
    maxMessagesPerTopic,
    setMaxMessagesPerTopic,
  } = useSettingsStore()

  const [newPreset, setNewPreset] = useState<{ name: string; ip: string; port: string }>({
    name: '',
    ip: '',
    port: '9090',
  })

  const handleAddPreset = () => {
    if (newPreset.name && newPreset.ip && newPreset.port) {
      addPreset({
        name: newPreset.name,
        ip: newPreset.ip,
        port: parseInt(newPreset.port, 10),
      })
      setNewPreset({ name: '', ip: '', port: '9090' })
    }
  }

  const themeOptions: { value: Theme; label: string; icon: typeof Sun }[] = [
    { value: 'light', label: 'Light', icon: Sun },
    { value: 'dark', label: 'Dark', icon: Moon },
    { value: 'system', label: 'System', icon: Monitor },
  ]

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        <Button variant="ghost" size="icon">
          <Settings className="h-5 w-5" />
        </Button>
      </DialogTrigger>
      <DialogContent className="max-w-2xl">
        <DialogHeader>
          <DialogTitle>Settings</DialogTitle>
        </DialogHeader>

        <Tabs defaultValue="appearance" className="mt-4">
          <TabsList className="grid w-full grid-cols-3">
            <TabsTrigger value="appearance">Appearance</TabsTrigger>
            <TabsTrigger value="presets">Connection Presets</TabsTrigger>
            <TabsTrigger value="display">Display</TabsTrigger>
          </TabsList>

          <TabsContent value="appearance" className="space-y-6 mt-4">
            <div className="space-y-2">
              <Label>Theme</Label>
              <div className="flex gap-2">
                {themeOptions.map((option) => {
                  const Icon = option.icon
                  return (
                    <button
                      key={option.value}
                      onClick={() => setTheme(option.value)}
                      className={cn(
                        'flex-1 flex items-center justify-center gap-2 p-3 rounded-lg border transition-all',
                        theme === option.value
                          ? 'border-[hsl(var(--primary))] bg-[hsl(var(--primary))]/10'
                          : 'border-[hsl(var(--border))] hover:border-[hsl(var(--primary))]/50'
                      )}
                    >
                      <Icon className="h-4 w-4" />
                      <span className="text-sm">{option.label}</span>
                    </button>
                  )
                })}
              </div>
            </div>
          </TabsContent>

          <TabsContent value="presets" className="space-y-6 mt-4">
            <div className="space-y-4">
              <Label>Saved Connections</Label>
              {presets.length === 0 ? (
                <p className="text-sm text-[hsl(var(--muted-foreground))]">
                  No saved connections. Add a preset below.
                </p>
              ) : (
                <div className="space-y-2">
                  {presets.map((preset) => (
                    <Card key={preset.id} className="p-3 flex items-center justify-between">
                      <div className="flex items-center gap-3">
                        <Wifi className="h-4 w-4 text-[hsl(var(--muted-foreground))]" />
                        <div>
                          <p className="font-medium">{preset.name}</p>
                          <p className="text-sm text-[hsl(var(--muted-foreground))]">
                            {preset.ip}:{preset.port}
                          </p>
                        </div>
                      </div>
                      <Button
                        variant="ghost"
                        size="icon"
                        onClick={() => removePreset(preset.id)}
                      >
                        <Trash2 className="h-4 w-4 text-red-500" />
                      </Button>
                    </Card>
                  ))}
                </div>
              )}

              <div className="border-t border-[hsl(var(--border))] pt-4">
                <Label className="mb-3 block">Add New Preset</Label>
                <div className="grid grid-cols-3 gap-3">
                  <Input
                    placeholder="Name"
                    value={newPreset.name}
                    onChange={(e) => setNewPreset({ ...newPreset, name: e.target.value })}
                  />
                  <Input
                    placeholder="IP Address"
                    value={newPreset.ip}
                    onChange={(e) => setNewPreset({ ...newPreset, ip: e.target.value })}
                  />
                  <Input
                    placeholder="Port"
                    type="number"
                    value={newPreset.port}
                    onChange={(e) => setNewPreset({ ...newPreset, port: e.target.value })}
                  />
                </div>
                <Button onClick={handleAddPreset} className="mt-3 gap-2">
                  <Plus className="h-4 w-4" />
                  Add Preset
                </Button>
              </div>
            </div>
          </TabsContent>

          <TabsContent value="display" className="space-y-6 mt-4">
            <div className="flex items-center justify-between">
              <div>
                <Label>Show Live Data Panel</Label>
                <p className="text-sm text-[hsl(var(--muted-foreground))]">
                  Display subscribed topics in side panel
                </p>
              </div>
              <Switch
                checked={showLiveDataPanel}
                onCheckedChange={setShowLiveDataPanel}
              />
            </div>

            <div className="space-y-2">
              <Label>Max Messages Per Topic</Label>
              <p className="text-sm text-[hsl(var(--muted-foreground))]">
                Number of messages to keep in history
              </p>
              <div className="flex items-center gap-4">
                <Input
                  type="number"
                  value={maxMessagesPerTopic}
                  onChange={(e) => setMaxMessagesPerTopic(parseInt(e.target.value, 10))}
                  className="w-32"
                  min={10}
                  max={1000}
                />
                <Badge variant="secondary">{maxMessagesPerTopic} messages</Badge>
              </div>
            </div>
          </TabsContent>
        </Tabs>
      </DialogContent>
    </Dialog>
  )
}
