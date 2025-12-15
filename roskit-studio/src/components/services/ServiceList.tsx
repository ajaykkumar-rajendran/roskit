import { useState, useMemo } from 'react'
import { Search, Wrench, Play } from 'lucide-react'
import { Input } from '@/components/ui/input'
import { Button } from '@/components/ui/button'
import { ScrollArea } from '@/components/ui/scroll-area'
import {
  Dialog,
  DialogContent,
  DialogTrigger,
} from '@/components/ui/dialog'
import { ServiceCaller } from './ServiceCaller'
import { useConnectionStore } from '@/stores/connectionStore'

export function ServiceList() {
  const [search, setSearch] = useState('')
  const [selectedService, setSelectedService] = useState<string | null>(null)
  const { robots, activeRobotId } = useConnectionStore()

  const activeRobot = activeRobotId ? robots[activeRobotId] : null

  const filteredServices = useMemo(() => {
    if (!activeRobot) return []

    const searchLower = search.toLowerCase()
    return activeRobot.services.filter((service) =>
      service.toLowerCase().includes(searchLower)
    )
  }, [activeRobot, search])

  if (!activeRobot) {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <Wrench className="h-12 w-12 opacity-50" />
        <p className="mt-3">Select a robot to view services</p>
      </div>
    )
  }

  if (activeRobot.status !== 'connected') {
    return (
      <div className="flex flex-col items-center justify-center h-full text-[hsl(var(--muted-foreground))]">
        <Wrench className="h-12 w-12 opacity-50" />
        <p className="mt-3">Robot is not connected</p>
      </div>
    )
  }

  return (
    <div className="flex flex-col h-full">
      <div className="p-4 border-b border-[hsl(var(--border))]">
        <div className="flex items-center gap-2">
          <h2 className="font-semibold">Services</h2>
          <span className="text-sm text-[hsl(var(--muted-foreground))]">
            ({filteredServices.length})
          </span>
        </div>
        <div className="relative mt-3">
          <Search className="absolute left-3 top-1/2 -translate-y-1/2 h-4 w-4 text-[hsl(var(--muted-foreground))]" />
          <Input
            placeholder="Search services..."
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            className="pl-9"
          />
        </div>
      </div>

      <ScrollArea className="flex-1">
        <div className="divide-y divide-[hsl(var(--border))]">
          {filteredServices.length === 0 ? (
            <div className="p-4 text-center text-[hsl(var(--muted-foreground))]">
              {search ? 'No services match your search' : 'No services available'}
            </div>
          ) : (
            filteredServices.map((service) => (
              <div
                key={service}
                className="p-3 hover:bg-[hsl(var(--accent))]/50 transition-colors flex items-center justify-between"
              >
                <div className="flex items-center gap-2 min-w-0">
                  <Wrench className="h-4 w-4 text-[hsl(var(--muted-foreground))] shrink-0" />
                  <span className="font-mono text-sm truncate">{service}</span>
                </div>
                <Dialog
                  open={selectedService === service}
                  onOpenChange={(open) => setSelectedService(open ? service : null)}
                >
                  <DialogTrigger asChild>
                    <Button variant="outline" size="sm" className="gap-1.5 shrink-0">
                      <Play className="h-3.5 w-3.5" />
                      Call
                    </Button>
                  </DialogTrigger>
                  <DialogContent className="p-0 border-0 bg-transparent shadow-none">
                    <ServiceCaller
                      serviceName={service}
                      serviceType=""
                      robotId={activeRobot.id}
                      onClose={() => setSelectedService(null)}
                    />
                  </DialogContent>
                </Dialog>
              </div>
            ))
          )}
        </div>
      </ScrollArea>
    </div>
  )
}
