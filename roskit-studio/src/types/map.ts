// ROS2 Standard Message Types for Map Visualization

export interface Header {
  stamp: {
    sec: number
    nanosec: number
  }
  frame_id: string
}

export interface Point {
  x: number
  y: number
  z: number
}

export interface Quaternion {
  x: number
  y: number
  z: number
  w: number
}

export interface Pose {
  position: Point
  orientation: Quaternion
}

export interface PoseStamped {
  header: Header
  pose: Pose
}

export interface PoseWithCovariance {
  pose: Pose
  covariance: number[]
}

export interface PoseWithCovarianceStamped {
  header: Header
  pose: PoseWithCovariance
}

export interface Twist {
  linear: Point
  angular: Point
}

export interface TwistWithCovariance {
  twist: Twist
  covariance: number[]
}

export interface Odometry {
  header: Header
  child_frame_id: string
  pose: PoseWithCovariance
  twist: TwistWithCovariance
}

export interface MapMetaData {
  map_load_time: {
    sec: number
    nanosec: number
  }
  resolution: number
  width: number
  height: number
  origin: Pose
}

export interface OccupancyGrid {
  header: Header
  info: MapMetaData
  data: number[]
}

export interface LaserScan {
  header: Header
  angle_min: number
  angle_max: number
  angle_increment: number
  time_increment: number
  scan_time: number
  range_min: number
  range_max: number
  ranges: number[]
  intensities: number[]
}

export interface Path {
  header: Header
  poses: PoseStamped[]
}

export interface TransformStamped {
  header: Header
  child_frame_id: string
  transform: {
    translation: Point
    rotation: Quaternion
  }
}

export interface TFMessage {
  transforms: TransformStamped[]
}

// Layer Configuration Types

export type LayerType = 'map' | 'costmap' | 'robot' | 'laserScan' | 'globalPath' | 'localPath'

export interface LayerConfig {
  enabled: boolean
  opacity: number
  topic: string
  color?: string
}

export interface MapLayerConfig extends LayerConfig {
  type: 'map'
}

export interface CostmapLayerConfig extends LayerConfig {
  type: 'costmap'
}

export interface RobotLayerConfig extends LayerConfig {
  type: 'robot'
  showTrail: boolean
  trailLength: number
}

export interface LaserScanLayerConfig extends LayerConfig {
  type: 'laserScan'
  pointSize: number
  colorByIntensity: boolean
}

export interface PathLayerConfig extends LayerConfig {
  type: 'path'
  lineWidth: number
}

export type AnyLayerConfig =
  | MapLayerConfig
  | CostmapLayerConfig
  | RobotLayerConfig
  | LaserScanLayerConfig
  | PathLayerConfig

// View State

export interface MapViewState {
  zoom: number
  centerX: number
  centerY: number
  followRobot: boolean
}

// Robot Trail Point

export interface TrailPoint {
  x: number
  y: number
  timestamp: number
}

// Default Topic Suggestions

export const DEFAULT_TOPICS = {
  map: ['/map'],
  costmap: ['/global_costmap/costmap', '/local_costmap/costmap'],
  robot: ['/robot_pose', '/amcl_pose', '/odom'],
  laserScan: ['/scan', '/lidar/scan', '/laser/scan'],
  globalPath: ['/plan', '/global_plan', '/planned_path'],
  localPath: ['/local_plan', '/controller_server/local_plan'],
} as const

// Helper to convert quaternion to yaw angle
export function quaternionToYaw(q: Quaternion): number {
  const siny_cosp = 2 * (q.w * q.z + q.x * q.y)
  const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
  return Math.atan2(siny_cosp, cosy_cosp)
}

// Helper to extract pose from various message types
export function extractPose(message: unknown): Pose | null {
  if (!message || typeof message !== 'object') return null

  const msg = message as Record<string, unknown>

  // PoseStamped
  if (msg.pose && typeof msg.pose === 'object') {
    const pose = msg.pose as Record<string, unknown>
    if (pose.position && pose.orientation) {
      return pose as unknown as Pose
    }
    // PoseWithCovariance (from Odometry)
    if (pose.pose && typeof pose.pose === 'object') {
      return pose.pose as unknown as Pose
    }
  }

  // Direct pose
  if (msg.position && msg.orientation) {
    return msg as unknown as Pose
  }

  return null
}
