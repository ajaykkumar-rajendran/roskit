import { useEffect, useRef } from 'react'
import * as THREE from 'three'
import type { Path } from '@/types/map'

interface PathLayerProps {
  scene: THREE.Scene
  path: Path
  color: string
  lineWidth: number
  zIndex: number
}

export function PathLayer({ scene, path, color, lineWidth, zIndex }: PathLayerProps) {
  const lineRef = useRef<THREE.Line | null>(null)
  const pointsRef = useRef<THREE.Points | null>(null)

  useEffect(() => {
    if (!path || !path.poses || path.poses.length < 2) {
      // Clean up if path is empty
      if (lineRef.current) {
        scene.remove(lineRef.current)
        lineRef.current.geometry.dispose()
        ;(lineRef.current.material as THREE.Material).dispose()
        lineRef.current = null
      }
      if (pointsRef.current) {
        scene.remove(pointsRef.current)
        pointsRef.current.geometry.dispose()
        ;(pointsRef.current.material as THREE.Material).dispose()
        pointsRef.current = null
      }
      return
    }

    const threeColor = new THREE.Color(color)
    const points: THREE.Vector3[] = []

    for (const poseStamped of path.poses) {
      const { x, y } = poseStamped.pose.position
      points.push(new THREE.Vector3(x, y, zIndex * 0.01 + 0.02))
    }

    // Create line geometry
    const lineGeometry = new THREE.BufferGeometry().setFromPoints(points)

    // Create or update line
    if (!lineRef.current) {
      const lineMaterial = new THREE.LineBasicMaterial({
        color: threeColor,
        linewidth: lineWidth, // Note: linewidth > 1 only works in some browsers
      })
      lineRef.current = new THREE.Line(lineGeometry, lineMaterial)
      lineRef.current.name = 'path'
      scene.add(lineRef.current)
    } else {
      lineRef.current.geometry.dispose()
      lineRef.current.geometry = lineGeometry
      const material = lineRef.current.material as THREE.LineBasicMaterial
      material.color = threeColor
    }

    // Create waypoint markers for better visibility
    const pointsGeometry = new THREE.BufferGeometry().setFromPoints(points)

    if (!pointsRef.current) {
      const pointsMaterial = new THREE.PointsMaterial({
        color: threeColor,
        size: 0.05,
        sizeAttenuation: true,
      })
      pointsRef.current = new THREE.Points(pointsGeometry, pointsMaterial)
      pointsRef.current.name = 'pathPoints'
      scene.add(pointsRef.current)
    } else {
      pointsRef.current.geometry.dispose()
      pointsRef.current.geometry = pointsGeometry
      const material = pointsRef.current.material as THREE.PointsMaterial
      material.color = threeColor
    }

    return () => {
      if (lineRef.current) {
        scene.remove(lineRef.current)
        lineRef.current.geometry.dispose()
        ;(lineRef.current.material as THREE.Material).dispose()
        lineRef.current = null
      }
      if (pointsRef.current) {
        scene.remove(pointsRef.current)
        pointsRef.current.geometry.dispose()
        ;(pointsRef.current.material as THREE.Material).dispose()
        pointsRef.current = null
      }
    }
  }, [scene, path, color, lineWidth, zIndex])

  return null
}
