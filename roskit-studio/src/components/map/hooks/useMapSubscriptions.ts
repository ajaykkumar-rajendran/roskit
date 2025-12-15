import { useEffect, useRef } from 'react'
import type { Subscription as RosKitSubscription } from '@roskit/client'
import { useMapStore } from '@/stores/mapStore'
import { useConnectionStore } from '@/stores/connectionStore'
import type { OccupancyGrid, LaserScan, Path } from '@/types/map'
import { extractPose } from '@/types/map'

export function useMapSubscriptions() {
  const { activeRobotId, getClient } = useConnectionStore()
  const robots = useConnectionStore((s) => s.robots)
  const {
    layers,
    setOccupancyGrid,
    setGlobalCostmap,
    setLocalCostmap,
    setRobotPose,
    setLaserScan,
    setGlobalPath,
    setLocalPath,
    clearAllData,
    setInitialized,
  } = useMapStore()

  const subscriptionsRef = useRef<Map<string, RosKitSubscription>>(new Map())

  // Get active robot's ROS connection
  const activeRobot = activeRobotId ? robots[activeRobotId] : null
  const client = activeRobotId ? getClient(activeRobotId) : null

  // Cleanup function
  const cleanupSubscriptions = () => {
    subscriptionsRef.current.forEach((sub) => {
      sub.unsubscribe()
    })
    subscriptionsRef.current.clear()
  }

  // Subscribe to a topic
  const subscribeTopic = async <T>(
    topicName: string,
    messageType: string,
    callback: (message: T) => void,
    throttleMs?: number
  ) => {
    if (!client || !topicName) return

    // Unsubscribe if already subscribed
    const existing = subscriptionsRef.current.get(topicName)
    if (existing) {
      existing.unsubscribe()
    }

    try {
      const sub = await client.subscribe<T>(
        topicName,
        callback,
        { msgType: messageType, throttleMs }
      )
      subscriptionsRef.current.set(topicName, sub)
    } catch (error) {
      console.error(`Failed to subscribe to ${topicName}:`, error)
    }
  }

  // Subscribe to all configured topics
  useEffect(() => {
    if (!client || activeRobot?.status !== 'connected') {
      cleanupSubscriptions()
      clearAllData()
      setInitialized(false)
      return
    }

    // Subscribe to map topic
    if (layers.map.enabled && layers.map.topic) {
      subscribeTopic<OccupancyGrid>(
        layers.map.topic,
        'nav_msgs/msg/OccupancyGrid',
        (msg) => {
          setOccupancyGrid(msg)
          setInitialized(true)
        }
      )
    }

    // Subscribe to global costmap
    if (layers.globalCostmap.enabled && layers.globalCostmap.topic) {
      subscribeTopic<OccupancyGrid>(
        layers.globalCostmap.topic,
        'nav_msgs/msg/OccupancyGrid',
        setGlobalCostmap,
        100 // Throttle to 10Hz
      )
    }

    // Subscribe to local costmap
    if (layers.localCostmap.enabled && layers.localCostmap.topic) {
      subscribeTopic<OccupancyGrid>(
        layers.localCostmap.topic,
        'nav_msgs/msg/OccupancyGrid',
        setLocalCostmap,
        100 // Throttle to 10Hz
      )
    }

    // Subscribe to robot pose (try multiple message types)
    if (layers.robot.enabled && layers.robot.topic) {
      const robotTopic = layers.robot.topic

      // Determine message type based on topic name
      let messageType = 'geometry_msgs/msg/PoseStamped'
      if (robotTopic.includes('odom')) {
        messageType = 'nav_msgs/msg/Odometry'
      } else if (robotTopic.includes('amcl')) {
        messageType = 'geometry_msgs/msg/PoseWithCovarianceStamped'
      }

      subscribeTopic<unknown>(
        robotTopic,
        messageType,
        (msg) => {
          const pose = extractPose(msg)
          if (pose) {
            setRobotPose(pose)
          }
        },
        33 // Throttle to ~30Hz
      )
    }

    // Subscribe to laser scan
    if (layers.laserScan.enabled && layers.laserScan.topic) {
      subscribeTopic<LaserScan>(
        layers.laserScan.topic,
        'sensor_msgs/msg/LaserScan',
        setLaserScan,
        33 // Throttle to ~30Hz
      )
    }

    // Subscribe to global path
    if (layers.globalPath.enabled && layers.globalPath.topic) {
      subscribeTopic<Path>(
        layers.globalPath.topic,
        'nav_msgs/msg/Path',
        setGlobalPath
      )
    }

    // Subscribe to local path
    if (layers.localPath.enabled && layers.localPath.topic) {
      subscribeTopic<Path>(
        layers.localPath.topic,
        'nav_msgs/msg/Path',
        setLocalPath,
        100 // Throttle to 10Hz
      )
    }

    return () => {
      cleanupSubscriptions()
    }
  }, [
    client,
    activeRobot?.status,
    layers.map.enabled,
    layers.map.topic,
    layers.globalCostmap.enabled,
    layers.globalCostmap.topic,
    layers.localCostmap.enabled,
    layers.localCostmap.topic,
    layers.robot.enabled,
    layers.robot.topic,
    layers.laserScan.enabled,
    layers.laserScan.topic,
    layers.globalPath.enabled,
    layers.globalPath.topic,
    layers.localPath.enabled,
    layers.localPath.topic,
  ])

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      cleanupSubscriptions()
    }
  }, [])
}
