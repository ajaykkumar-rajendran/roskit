import { useEffect } from 'react'
import { useThreeScene } from './hooks/useThreeScene'
import { OccupancyGridLayer } from './layers/OccupancyGridLayer'
import { RobotLayer } from './layers/RobotLayer'
import { LaserScanLayer } from './layers/LaserScanLayer'
import { PathLayer } from './layers/PathLayer'
import { CostmapLayer } from './layers/CostmapLayer'
import { useMapStore } from '@/stores/mapStore'

interface MapCanvasProps {
  onSceneReady?: () => void
}

export function MapCanvas({ onSceneReady }: MapCanvasProps) {
  const { containerRef, sceneRef, zoomIn, zoomOut, resetView } = useThreeScene()
  const { layers, data, viewState, setCenter } = useMapStore()

  // Notify parent when scene is ready
  useEffect(() => {
    if (sceneRef.current && onSceneReady) {
      onSceneReady()
    }
  }, [sceneRef.current, onSceneReady])

  // Follow robot if enabled
  useEffect(() => {
    if (viewState.followRobot && data.robotPose) {
      setCenter(data.robotPose.position.x, data.robotPose.position.y)
    }
  }, [viewState.followRobot, data.robotPose, setCenter])

  return (
    <div className="relative w-full h-full">
      <div ref={containerRef} className="w-full h-full" />

      {/* Render layers */}
      {sceneRef.current && (
        <>
          {/* Base map layer */}
          {layers.map.enabled && data.occupancyGrid && (
            <OccupancyGridLayer
              scene={sceneRef.current}
              occupancyGrid={data.occupancyGrid}
              opacity={layers.map.opacity}
              zIndex={0}
            />
          )}

          {/* Global costmap layer */}
          {layers.globalCostmap.enabled && data.globalCostmap && (
            <CostmapLayer
              scene={sceneRef.current}
              costmap={data.globalCostmap}
              opacity={layers.globalCostmap.opacity}
              zIndex={1}
            />
          )}

          {/* Local costmap layer */}
          {layers.localCostmap.enabled && data.localCostmap && (
            <CostmapLayer
              scene={sceneRef.current}
              costmap={data.localCostmap}
              opacity={layers.localCostmap.opacity}
              zIndex={2}
            />
          )}

          {/* Global path layer */}
          {layers.globalPath.enabled && data.globalPath && (
            <PathLayer
              scene={sceneRef.current}
              path={data.globalPath}
              color={layers.globalPath.color}
              lineWidth={layers.globalPath.lineWidth}
              zIndex={3}
            />
          )}

          {/* Local path layer */}
          {layers.localPath.enabled && data.localPath && (
            <PathLayer
              scene={sceneRef.current}
              path={data.localPath}
              color={layers.localPath.color}
              lineWidth={layers.localPath.lineWidth}
              zIndex={4}
            />
          )}

          {/* Laser scan layer */}
          {layers.laserScan.enabled && data.laserScan && data.robotPose && (
            <LaserScanLayer
              scene={sceneRef.current}
              laserScan={data.laserScan}
              robotPose={data.robotPose}
              color={layers.laserScan.color}
              pointSize={layers.laserScan.pointSize}
              colorByIntensity={layers.laserScan.colorByIntensity}
              zIndex={5}
            />
          )}

          {/* Robot layer */}
          {layers.robot.enabled && data.robotPose && (
            <RobotLayer
              scene={sceneRef.current}
              pose={data.robotPose}
              trail={layers.robot.showTrail ? data.robotTrail : []}
              zIndex={6}
            />
          )}
        </>
      )}

      {/* Zoom controls overlay */}
      <div className="absolute bottom-4 right-4 flex flex-col gap-2">
        <button
          onClick={zoomIn}
          className="w-8 h-8 bg-[hsl(var(--card))] border border-[hsl(var(--border))] rounded flex items-center justify-center hover:bg-[hsl(var(--accent))] transition-colors"
          title="Zoom In"
        >
          +
        </button>
        <button
          onClick={zoomOut}
          className="w-8 h-8 bg-[hsl(var(--card))] border border-[hsl(var(--border))] rounded flex items-center justify-center hover:bg-[hsl(var(--accent))] transition-colors"
          title="Zoom Out"
        >
          −
        </button>
        <button
          onClick={resetView}
          className="w-8 h-8 bg-[hsl(var(--card))] border border-[hsl(var(--border))] rounded flex items-center justify-center hover:bg-[hsl(var(--accent))] transition-colors text-xs"
          title="Reset View"
        >
          ⟲
        </button>
      </div>

      {/* Coordinates display */}
      <div className="absolute bottom-4 left-4 bg-[hsl(var(--card))]/80 px-2 py-1 rounded text-xs font-mono">
        <span>Zoom: {viewState.zoom.toFixed(2)}x</span>
        <span className="ml-3">
          Center: ({viewState.centerX.toFixed(2)}, {viewState.centerY.toFixed(2)})
        </span>
      </div>
    </div>
  )
}
