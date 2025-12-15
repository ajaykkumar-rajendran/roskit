import {
  Eye,
  EyeOff,
  Crosshair,
  Layers,
  RotateCcw,
  ZoomIn,
  ZoomOut,
} from 'lucide-react'
import { Button } from '@/components/ui/button'
import { Switch } from '@/components/ui/switch'
import { Slider } from '@/components/ui/slider'
import { Label } from '@/components/ui/label'
import {
  Popover,
  PopoverContent,
  PopoverTrigger,
} from '@/components/ui/popover'
import { useMapStore } from '@/stores/mapStore'

interface MapControlsProps {
  onZoomIn: () => void
  onZoomOut: () => void
  onResetView: () => void
}

export function MapControls({ onZoomIn, onZoomOut, onResetView }: MapControlsProps) {
  const {
    layers,
    viewState,
    setLayerEnabled,
    setLayerOpacity,
    setFollowRobot,
    setRobotTrailEnabled,
    setLaserScanPointSize,
  } = useMapStore()

  return (
    <div className="flex items-center gap-2 p-2 border-b border-[hsl(var(--border))]">
      {/* Zoom Controls */}
      <div className="flex items-center gap-1">
        <Button variant="ghost" size="icon" onClick={onZoomIn} title="Zoom In">
          <ZoomIn className="h-4 w-4" />
        </Button>
        <Button variant="ghost" size="icon" onClick={onZoomOut} title="Zoom Out">
          <ZoomOut className="h-4 w-4" />
        </Button>
        <Button variant="ghost" size="icon" onClick={onResetView} title="Reset View">
          <RotateCcw className="h-4 w-4" />
        </Button>
      </div>

      <div className="w-px h-6 bg-[hsl(var(--border))]" />

      {/* Follow Robot Toggle */}
      <Button
        variant={viewState.followRobot ? 'default' : 'ghost'}
        size="sm"
        onClick={() => setFollowRobot(!viewState.followRobot)}
        className="gap-2"
      >
        <Crosshair className="h-4 w-4" />
        Follow
      </Button>

      <div className="w-px h-6 bg-[hsl(var(--border))]" />

      {/* Layers Panel */}
      <Popover>
        <PopoverTrigger asChild>
          <Button variant="ghost" size="sm" className="gap-2">
            <Layers className="h-4 w-4" />
            Layers
          </Button>
        </PopoverTrigger>
        <PopoverContent className="w-80" align="start">
          <div className="space-y-4">
            <h4 className="font-medium text-sm">Layer Settings</h4>

            {/* Map Layer */}
            <div className="space-y-2">
              <div className="flex items-center justify-between">
                <Label className="flex items-center gap-2">
                  {layers.map.enabled ? (
                    <Eye className="h-4 w-4" />
                  ) : (
                    <EyeOff className="h-4 w-4 opacity-50" />
                  )}
                  Map
                </Label>
                <Switch
                  checked={layers.map.enabled}
                  onCheckedChange={(checked) => setLayerEnabled('map', checked)}
                />
              </div>
              {layers.map.enabled && (
                <div className="pl-6">
                  <Label className="text-xs text-[hsl(var(--muted-foreground))]">
                    Opacity: {Math.round(layers.map.opacity * 100)}%
                  </Label>
                  <Slider
                    value={[layers.map.opacity]}
                    onValueChange={(values: number[]) => setLayerOpacity('map', values[0])}
                    min={0}
                    max={1}
                    step={0.1}
                    className="mt-1"
                  />
                </div>
              )}
            </div>

            {/* Global Costmap Layer */}
            <div className="space-y-2">
              <div className="flex items-center justify-between">
                <Label className="flex items-center gap-2">
                  {layers.globalCostmap.enabled ? (
                    <Eye className="h-4 w-4" />
                  ) : (
                    <EyeOff className="h-4 w-4 opacity-50" />
                  )}
                  Global Costmap
                </Label>
                <Switch
                  checked={layers.globalCostmap.enabled}
                  onCheckedChange={(checked) => setLayerEnabled('globalCostmap', checked)}
                />
              </div>
              {layers.globalCostmap.enabled && (
                <div className="pl-6">
                  <Label className="text-xs text-[hsl(var(--muted-foreground))]">
                    Opacity: {Math.round(layers.globalCostmap.opacity * 100)}%
                  </Label>
                  <Slider
                    value={[layers.globalCostmap.opacity]}
                    onValueChange={(values: number[]) => setLayerOpacity('globalCostmap', values[0])}
                    min={0}
                    max={1}
                    step={0.1}
                    className="mt-1"
                  />
                </div>
              )}
            </div>

            {/* Local Costmap Layer */}
            <div className="space-y-2">
              <div className="flex items-center justify-between">
                <Label className="flex items-center gap-2">
                  {layers.localCostmap.enabled ? (
                    <Eye className="h-4 w-4" />
                  ) : (
                    <EyeOff className="h-4 w-4 opacity-50" />
                  )}
                  Local Costmap
                </Label>
                <Switch
                  checked={layers.localCostmap.enabled}
                  onCheckedChange={(checked) => setLayerEnabled('localCostmap', checked)}
                />
              </div>
              {layers.localCostmap.enabled && (
                <div className="pl-6">
                  <Label className="text-xs text-[hsl(var(--muted-foreground))]">
                    Opacity: {Math.round(layers.localCostmap.opacity * 100)}%
                  </Label>
                  <Slider
                    value={[layers.localCostmap.opacity]}
                    onValueChange={(values: number[]) => setLayerOpacity('localCostmap', values[0])}
                    min={0}
                    max={1}
                    step={0.1}
                    className="mt-1"
                  />
                </div>
              )}
            </div>

            {/* Robot Layer */}
            <div className="space-y-2">
              <div className="flex items-center justify-between">
                <Label className="flex items-center gap-2">
                  {layers.robot.enabled ? (
                    <Eye className="h-4 w-4" />
                  ) : (
                    <EyeOff className="h-4 w-4 opacity-50" />
                  )}
                  Robot
                </Label>
                <Switch
                  checked={layers.robot.enabled}
                  onCheckedChange={(checked) => setLayerEnabled('robot', checked)}
                />
              </div>
              {layers.robot.enabled && (
                <div className="pl-6 flex items-center gap-2">
                  <Label className="text-xs">Show Trail</Label>
                  <Switch
                    checked={layers.robot.showTrail}
                    onCheckedChange={setRobotTrailEnabled}
                  />
                </div>
              )}
            </div>

            {/* Laser Scan Layer */}
            <div className="space-y-2">
              <div className="flex items-center justify-between">
                <Label className="flex items-center gap-2">
                  {layers.laserScan.enabled ? (
                    <Eye className="h-4 w-4" />
                  ) : (
                    <EyeOff className="h-4 w-4 opacity-50" />
                  )}
                  Laser Scan
                </Label>
                <Switch
                  checked={layers.laserScan.enabled}
                  onCheckedChange={(checked) => setLayerEnabled('laserScan', checked)}
                />
              </div>
              {layers.laserScan.enabled && (
                <div className="pl-6">
                  <Label className="text-xs text-[hsl(var(--muted-foreground))]">
                    Point Size: {layers.laserScan.pointSize}
                  </Label>
                  <Slider
                    value={[layers.laserScan.pointSize]}
                    onValueChange={(values: number[]) => setLaserScanPointSize(values[0])}
                    min={1}
                    max={10}
                    step={1}
                    className="mt-1"
                  />
                </div>
              )}
            </div>

            {/* Global Path Layer */}
            <div className="flex items-center justify-between">
              <Label className="flex items-center gap-2">
                {layers.globalPath.enabled ? (
                  <Eye className="h-4 w-4" />
                ) : (
                  <EyeOff className="h-4 w-4 opacity-50" />
                )}
                <span className="flex items-center gap-2">
                  Global Path
                  <span
                    className="w-3 h-3 rounded-full"
                    style={{ backgroundColor: layers.globalPath.color }}
                  />
                </span>
              </Label>
              <Switch
                checked={layers.globalPath.enabled}
                onCheckedChange={(checked) => setLayerEnabled('globalPath', checked)}
              />
            </div>

            {/* Local Path Layer */}
            <div className="flex items-center justify-between">
              <Label className="flex items-center gap-2">
                {layers.localPath.enabled ? (
                  <Eye className="h-4 w-4" />
                ) : (
                  <EyeOff className="h-4 w-4 opacity-50" />
                )}
                <span className="flex items-center gap-2">
                  Local Path
                  <span
                    className="w-3 h-3 rounded-full"
                    style={{ backgroundColor: layers.localPath.color }}
                  />
                </span>
              </Label>
              <Switch
                checked={layers.localPath.enabled}
                onCheckedChange={(checked) => setLayerEnabled('localPath', checked)}
              />
            </div>
          </div>
        </PopoverContent>
      </Popover>
    </div>
  )
}
