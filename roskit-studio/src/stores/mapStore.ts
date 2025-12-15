import { create } from 'zustand'
import type {
  OccupancyGrid,
  LaserScan,
  Path,
  Pose,
  TrailPoint,
  MapViewState,
} from '@/types/map'

export interface LayerSettings {
  map: {
    enabled: boolean
    opacity: number
    topic: string
  }
  globalCostmap: {
    enabled: boolean
    opacity: number
    topic: string
  }
  localCostmap: {
    enabled: boolean
    opacity: number
    topic: string
  }
  robot: {
    enabled: boolean
    topic: string
    showTrail: boolean
    trailLength: number
  }
  laserScan: {
    enabled: boolean
    topic: string
    pointSize: number
    color: string
    colorByIntensity: boolean
  }
  globalPath: {
    enabled: boolean
    topic: string
    color: string
    lineWidth: number
  }
  localPath: {
    enabled: boolean
    topic: string
    color: string
    lineWidth: number
  }
}

interface MapData {
  occupancyGrid: OccupancyGrid | null
  globalCostmap: OccupancyGrid | null
  localCostmap: OccupancyGrid | null
  robotPose: Pose | null
  robotTrail: TrailPoint[]
  laserScan: LaserScan | null
  globalPath: Path | null
  localPath: Path | null
}

interface MapStore {
  layers: LayerSettings
  viewState: MapViewState
  data: MapData
  isInitialized: boolean

  // Layer actions
  setLayerEnabled: (layerKey: keyof LayerSettings, enabled: boolean) => void
  setLayerTopic: (layerKey: keyof LayerSettings, topic: string) => void
  setLayerOpacity: (layerKey: 'map' | 'globalCostmap' | 'localCostmap', opacity: number) => void
  setRobotTrailEnabled: (enabled: boolean) => void
  setLaserScanPointSize: (size: number) => void
  setLaserScanColor: (color: string) => void
  setPathColor: (pathType: 'globalPath' | 'localPath', color: string) => void

  // View actions
  setZoom: (zoom: number) => void
  setCenter: (x: number, y: number) => void
  setFollowRobot: (follow: boolean) => void
  resetView: () => void

  // Data actions
  setOccupancyGrid: (grid: OccupancyGrid) => void
  setGlobalCostmap: (costmap: OccupancyGrid) => void
  setLocalCostmap: (costmap: OccupancyGrid) => void
  setRobotPose: (pose: Pose) => void
  setLaserScan: (scan: LaserScan) => void
  setGlobalPath: (path: Path) => void
  setLocalPath: (path: Path) => void
  clearAllData: () => void
  setInitialized: (initialized: boolean) => void
}

const MAX_TRAIL_POINTS = 500

const initialLayers: LayerSettings = {
  map: {
    enabled: true,
    opacity: 1,
    topic: '/map',
  },
  globalCostmap: {
    enabled: false,
    opacity: 0.5,
    topic: '/global_costmap/costmap',
  },
  localCostmap: {
    enabled: false,
    opacity: 0.5,
    topic: '/local_costmap/costmap',
  },
  robot: {
    enabled: true,
    topic: '/odom',
    showTrail: true,
    trailLength: 200,
  },
  laserScan: {
    enabled: true,
    topic: '/scan',
    pointSize: 2,
    color: '#ff0000',
    colorByIntensity: false,
  },
  globalPath: {
    enabled: true,
    topic: '/plan',
    color: '#00ff00',
    lineWidth: 2,
  },
  localPath: {
    enabled: true,
    topic: '/local_plan',
    color: '#ffff00',
    lineWidth: 2,
  },
}

const initialViewState: MapViewState = {
  zoom: 1,
  centerX: 0,
  centerY: 0,
  followRobot: true,
}

const initialData: MapData = {
  occupancyGrid: null,
  globalCostmap: null,
  localCostmap: null,
  robotPose: null,
  robotTrail: [],
  laserScan: null,
  globalPath: null,
  localPath: null,
}

export const useMapStore = create<MapStore>((set, get) => ({
  layers: initialLayers,
  viewState: initialViewState,
  data: initialData,
  isInitialized: false,

  // Layer actions
  setLayerEnabled: (layerKey, enabled) => {
    set((state) => ({
      layers: {
        ...state.layers,
        [layerKey]: {
          ...state.layers[layerKey],
          enabled,
        },
      },
    }))
  },

  setLayerTopic: (layerKey, topic) => {
    set((state) => ({
      layers: {
        ...state.layers,
        [layerKey]: {
          ...state.layers[layerKey],
          topic,
        },
      },
    }))
  },

  setLayerOpacity: (layerKey, opacity) => {
    set((state) => ({
      layers: {
        ...state.layers,
        [layerKey]: {
          ...state.layers[layerKey],
          opacity,
        },
      },
    }))
  },

  setRobotTrailEnabled: (enabled) => {
    set((state) => ({
      layers: {
        ...state.layers,
        robot: {
          ...state.layers.robot,
          showTrail: enabled,
        },
      },
    }))
  },

  setLaserScanPointSize: (size) => {
    set((state) => ({
      layers: {
        ...state.layers,
        laserScan: {
          ...state.layers.laserScan,
          pointSize: size,
        },
      },
    }))
  },

  setLaserScanColor: (color) => {
    set((state) => ({
      layers: {
        ...state.layers,
        laserScan: {
          ...state.layers.laserScan,
          color,
        },
      },
    }))
  },

  setPathColor: (pathType, color) => {
    set((state) => ({
      layers: {
        ...state.layers,
        [pathType]: {
          ...state.layers[pathType],
          color,
        },
      },
    }))
  },

  // View actions
  setZoom: (zoom) => {
    set((state) => ({
      viewState: { ...state.viewState, zoom },
    }))
  },

  setCenter: (x, y) => {
    set((state) => ({
      viewState: { ...state.viewState, centerX: x, centerY: y },
    }))
  },

  setFollowRobot: (follow) => {
    set((state) => ({
      viewState: { ...state.viewState, followRobot: follow },
    }))
  },

  resetView: () => {
    set({ viewState: initialViewState })
  },

  // Data actions
  setOccupancyGrid: (grid) => {
    set((state) => ({
      data: { ...state.data, occupancyGrid: grid },
    }))
  },

  setGlobalCostmap: (costmap) => {
    set((state) => ({
      data: { ...state.data, globalCostmap: costmap },
    }))
  },

  setLocalCostmap: (costmap) => {
    set((state) => ({
      data: { ...state.data, localCostmap: costmap },
    }))
  },

  setRobotPose: (pose) => {
    const { layers, data } = get()
    const now = Date.now()

    let newTrail = data.robotTrail
    if (layers.robot.showTrail) {
      newTrail = [
        ...data.robotTrail,
        { x: pose.position.x, y: pose.position.y, timestamp: now },
      ]
      // Limit trail length
      const maxLength = Math.min(layers.robot.trailLength, MAX_TRAIL_POINTS)
      if (newTrail.length > maxLength) {
        newTrail = newTrail.slice(-maxLength)
      }
    }

    set((state) => ({
      data: {
        ...state.data,
        robotPose: pose,
        robotTrail: newTrail,
      },
    }))
  },

  setLaserScan: (scan) => {
    set((state) => ({
      data: { ...state.data, laserScan: scan },
    }))
  },

  setGlobalPath: (path) => {
    set((state) => ({
      data: { ...state.data, globalPath: path },
    }))
  },

  setLocalPath: (path) => {
    set((state) => ({
      data: { ...state.data, localPath: path },
    }))
  },

  clearAllData: () => {
    set({ data: initialData })
  },

  setInitialized: (initialized) => {
    set({ isInitialized: initialized })
  },
}))
