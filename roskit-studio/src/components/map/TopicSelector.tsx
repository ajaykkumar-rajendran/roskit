import { useState, useMemo } from 'react'
import { Settings2 } from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { Label } from '@/components/ui/label'
import {
  Dialog,
  DialogContent,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog'
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select'
import { useMapStore, type LayerSettings } from '@/stores/mapStore'
import { useConnectionStore } from '@/stores/connectionStore'
import { DEFAULT_TOPICS } from '@/types/map'

interface TopicInfo {
  name: string
  type: string
}

export function TopicSelector() {
  const [open, setOpen] = useState(false)
  const { layers, setLayerTopic } = useMapStore()
  const { activeRobotId, robots } = useConnectionStore()

  const activeRobot = activeRobotId ? robots[activeRobotId] : null
  const availableTopics: TopicInfo[] = activeRobot?.topics || []

  // Filter topics by message type
  const getTopicsForType = (messageTypes: string[]): TopicInfo[] => {
    return availableTopics.filter((t) =>
      messageTypes.some((mt) => t.type.includes(mt))
    )
  }

  const mapTopics = useMemo(
    () => getTopicsForType(['OccupancyGrid']),
    [availableTopics]
  )
  const poseTopics = useMemo(
    () => getTopicsForType(['Pose', 'Odometry']),
    [availableTopics]
  )
  const scanTopics = useMemo(
    () => getTopicsForType(['LaserScan']),
    [availableTopics]
  )
  const pathTopics = useMemo(
    () => getTopicsForType(['Path']),
    [availableTopics]
  )

  const renderTopicSelect = (
    label: string,
    layerKey: keyof LayerSettings,
    currentTopic: string,
    suggestedTopics: readonly string[],
    filteredTopics: TopicInfo[]
  ) => {
    const allOptions = [
      ...new Set([
        currentTopic,
        ...suggestedTopics,
        ...filteredTopics.map((t) => t.name),
      ]),
    ].filter(Boolean)

    return (
      <div className="space-y-2">
        <Label>{label}</Label>
        <div className="flex gap-2">
          <Select
            value={currentTopic}
            onValueChange={(value: string) => setLayerTopic(layerKey, value)}
          >
            <SelectTrigger className="flex-1">
              <SelectValue placeholder="Select topic" />
            </SelectTrigger>
            <SelectContent>
              {allOptions.length > 0 ? (
                allOptions.map((topic) => (
                  <SelectItem key={topic} value={topic}>
                    <span className="font-mono text-xs">{topic}</span>
                  </SelectItem>
                ))
              ) : (
                <SelectItem value="_none" disabled>
                  No topics available
                </SelectItem>
              )}
            </SelectContent>
          </Select>
          <Input
            value={currentTopic}
            onChange={(e) => setLayerTopic(layerKey, e.target.value)}
            placeholder="Custom topic"
            className="flex-1 font-mono text-xs"
          />
        </div>
      </div>
    )
  }

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger asChild>
        <Button variant="ghost" size="sm" className="gap-2">
          <Settings2 className="h-4 w-4" />
          Topics
        </Button>
      </DialogTrigger>
      <DialogContent className="max-w-lg">
        <DialogHeader>
          <DialogTitle>Configure Topic Subscriptions</DialogTitle>
        </DialogHeader>

        <div className="space-y-4 max-h-[60vh] overflow-y-auto pr-2">
          {renderTopicSelect(
            'Map Topic',
            'map',
            layers.map.topic,
            DEFAULT_TOPICS.map,
            mapTopics
          )}

          {renderTopicSelect(
            'Global Costmap Topic',
            'globalCostmap',
            layers.globalCostmap.topic,
            DEFAULT_TOPICS.costmap,
            mapTopics
          )}

          {renderTopicSelect(
            'Local Costmap Topic',
            'localCostmap',
            layers.localCostmap.topic,
            DEFAULT_TOPICS.costmap.filter((t) => t.includes('local')),
            mapTopics
          )}

          {renderTopicSelect(
            'Robot Pose Topic',
            'robot',
            layers.robot.topic,
            DEFAULT_TOPICS.robot,
            poseTopics
          )}

          {renderTopicSelect(
            'Laser Scan Topic',
            'laserScan',
            layers.laserScan.topic,
            DEFAULT_TOPICS.laserScan,
            scanTopics
          )}

          {renderTopicSelect(
            'Global Path Topic',
            'globalPath',
            layers.globalPath.topic,
            DEFAULT_TOPICS.globalPath,
            pathTopics
          )}

          {renderTopicSelect(
            'Local Path Topic',
            'localPath',
            layers.localPath.topic,
            DEFAULT_TOPICS.localPath,
            pathTopics
          )}
        </div>

        {!activeRobot && (
          <p className="text-sm text-[hsl(var(--muted-foreground))] mt-2">
            Connect to a robot to see available topics
          </p>
        )}
      </DialogContent>
    </Dialog>
  )
}
