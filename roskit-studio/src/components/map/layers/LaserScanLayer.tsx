import { useEffect, useRef } from 'react'
import * as THREE from 'three'
import type { LaserScan, Pose } from '@/types/map'
import { quaternionToYaw } from '@/types/map'
import { getIntensityColor } from '@/lib/map/colorMaps'

interface LaserScanLayerProps {
  scene: THREE.Scene
  laserScan: LaserScan
  robotPose: Pose
  color: string
  pointSize: number
  colorByIntensity: boolean
  zIndex: number
}

export function LaserScanLayer({
  scene,
  laserScan,
  robotPose,
  color,
  pointSize,
  colorByIntensity,
  zIndex,
}: LaserScanLayerProps) {
  const pointsRef = useRef<THREE.Points | null>(null)

  useEffect(() => {
    if (!laserScan || !robotPose) return

    const { ranges, intensities, angle_min, angle_increment, range_min, range_max } = laserScan
    const { position, orientation } = robotPose
    const robotYaw = quaternionToYaw(orientation)

    // Calculate valid points
    const validPoints: THREE.Vector3[] = []
    const colors: THREE.Color[] = []

    // Find intensity range for coloring
    let minIntensity = Infinity
    let maxIntensity = -Infinity
    if (colorByIntensity && intensities && intensities.length > 0) {
      for (const intensity of intensities) {
        if (intensity > 0) {
          minIntensity = Math.min(minIntensity, intensity)
          maxIntensity = Math.max(maxIntensity, intensity)
        }
      }
    }

    const baseColor = new THREE.Color(color)

    for (let i = 0; i < ranges.length; i++) {
      const range = ranges[i]

      // Filter invalid ranges
      if (range < range_min || range > range_max || !isFinite(range)) {
        continue
      }

      // Calculate point in robot frame
      const angle = angle_min + i * angle_increment
      const localX = range * Math.cos(angle)
      const localY = range * Math.sin(angle)

      // Transform to world frame
      const cosYaw = Math.cos(robotYaw)
      const sinYaw = Math.sin(robotYaw)
      const worldX = position.x + localX * cosYaw - localY * sinYaw
      const worldY = position.y + localX * sinYaw + localY * cosYaw

      validPoints.push(new THREE.Vector3(worldX, worldY, zIndex * 0.01 + 0.05))

      // Determine color
      if (colorByIntensity && intensities && intensities[i] !== undefined) {
        colors.push(getIntensityColor(intensities[i], minIntensity, maxIntensity))
      } else {
        colors.push(baseColor.clone())
      }
    }

    if (validPoints.length === 0) return

    // Create geometry
    const geometry = new THREE.BufferGeometry()
    const positions = new Float32Array(validPoints.length * 3)
    const colorArray = new Float32Array(validPoints.length * 3)

    for (let i = 0; i < validPoints.length; i++) {
      positions[i * 3] = validPoints[i].x
      positions[i * 3 + 1] = validPoints[i].y
      positions[i * 3 + 2] = validPoints[i].z

      colorArray[i * 3] = colors[i].r
      colorArray[i * 3 + 1] = colors[i].g
      colorArray[i * 3 + 2] = colors[i].b
    }

    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
    geometry.setAttribute('color', new THREE.BufferAttribute(colorArray, 3))

    // Create or update points
    if (!pointsRef.current) {
      const material = new THREE.PointsMaterial({
        size: pointSize * 0.02,
        vertexColors: true,
        sizeAttenuation: false,
      })
      pointsRef.current = new THREE.Points(geometry, material)
      pointsRef.current.name = 'laserScan'
      scene.add(pointsRef.current)
    } else {
      pointsRef.current.geometry.dispose()
      pointsRef.current.geometry = geometry
      const material = pointsRef.current.material as THREE.PointsMaterial
      material.size = pointSize * 0.02
    }

    return () => {
      if (pointsRef.current) {
        scene.remove(pointsRef.current)
        pointsRef.current.geometry.dispose()
        ;(pointsRef.current.material as THREE.Material).dispose()
        pointsRef.current = null
      }
    }
  }, [scene, laserScan, robotPose, color, pointSize, colorByIntensity, zIndex])

  return null
}
