import { useMapStore } from '@/stores/mapStore'

export function MapLegend() {
  const { layers, data } = useMapStore()

  const hasData =
    data.occupancyGrid ||
    data.globalCostmap ||
    data.localCostmap ||
    data.robotPose ||
    data.laserScan ||
    data.globalPath ||
    data.localPath

  if (!hasData) return null

  return (
    <div className="absolute top-4 right-4 bg-[hsl(var(--card))]/90 backdrop-blur-sm rounded-lg border border-[hsl(var(--border))] p-3 text-xs space-y-2">
      <h4 className="font-medium text-sm mb-2">Legend</h4>

      {/* Occupancy Grid Legend */}
      {layers.map.enabled && data.occupancyGrid && (
        <div className="space-y-1">
          <p className="text-[hsl(var(--muted-foreground))]">Occupancy Grid</p>
          <div className="flex items-center gap-2">
            <div className="flex items-center gap-1">
              <div className="w-3 h-3 rounded-sm bg-white border border-gray-300" />
              <span>Free</span>
            </div>
            <div className="flex items-center gap-1">
              <div className="w-3 h-3 rounded-sm bg-gray-400" />
              <span>Unknown</span>
            </div>
            <div className="flex items-center gap-1">
              <div className="w-3 h-3 rounded-sm bg-black" />
              <span>Occupied</span>
            </div>
          </div>
        </div>
      )}

      {/* Costmap Legend */}
      {(layers.globalCostmap.enabled || layers.localCostmap.enabled) &&
        (data.globalCostmap || data.localCostmap) && (
          <div className="space-y-1">
            <p className="text-[hsl(var(--muted-foreground))]">Costmap</p>
            <div className="flex items-center gap-1">
              <div className="w-16 h-3 rounded-sm bg-gradient-to-r from-blue-900 via-yellow-500 to-red-500" />
              <span>Low → High Cost</span>
            </div>
          </div>
        )}

      {/* Active Layers */}
      <div className="pt-2 border-t border-[hsl(var(--border))]">
        <p className="text-[hsl(var(--muted-foreground))] mb-1">Active Layers</p>
        <div className="flex flex-wrap gap-1">
          {layers.map.enabled && data.occupancyGrid && (
            <span className="px-1.5 py-0.5 bg-[hsl(var(--muted))] rounded">
              Map
            </span>
          )}
          {layers.globalCostmap.enabled && data.globalCostmap && (
            <span className="px-1.5 py-0.5 bg-[hsl(var(--muted))] rounded">
              Global Costmap
            </span>
          )}
          {layers.localCostmap.enabled && data.localCostmap && (
            <span className="px-1.5 py-0.5 bg-[hsl(var(--muted))] rounded">
              Local Costmap
            </span>
          )}
          {layers.robot.enabled && data.robotPose && (
            <span className="px-1.5 py-0.5 bg-blue-500/20 text-blue-400 rounded">
              Robot
            </span>
          )}
          {layers.laserScan.enabled && data.laserScan && (
            <span
              className="px-1.5 py-0.5 rounded"
              style={{
                backgroundColor: `${layers.laserScan.color}33`,
                color: layers.laserScan.color,
              }}
            >
              LiDAR
            </span>
          )}
          {layers.globalPath.enabled && data.globalPath && (
            <span
              className="px-1.5 py-0.5 rounded"
              style={{
                backgroundColor: `${layers.globalPath.color}33`,
                color: layers.globalPath.color,
              }}
            >
              Global Path
            </span>
          )}
          {layers.localPath.enabled && data.localPath && (
            <span
              className="px-1.5 py-0.5 rounded"
              style={{
                backgroundColor: `${layers.localPath.color}33`,
                color: layers.localPath.color,
              }}
            >
              Local Path
            </span>
          )}
        </div>
      </div>

      {/* Robot Position Info */}
      {data.robotPose && (
        <div className="pt-2 border-t border-[hsl(var(--border))] font-mono">
          <p className="text-[hsl(var(--muted-foreground))]">Robot Position</p>
          <p>
            X: {data.robotPose.position.x.toFixed(2)}m, Y:{' '}
            {data.robotPose.position.y.toFixed(2)}m
          </p>
        </div>
      )}
    </div>
  )
}
