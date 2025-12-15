import { useEffect, useRef } from 'react'
import * as THREE from 'three'
import type { Pose, TrailPoint } from '@/types/map'
import { quaternionToYaw } from '@/types/map'
import { ROBOT_COLORS } from '@/lib/map/colorMaps'

interface RobotLayerProps {
  scene: THREE.Scene
  pose: Pose
  trail: TrailPoint[]
  zIndex: number
}

export function RobotLayer({ scene, pose, trail, zIndex }: RobotLayerProps) {
  const robotGroupRef = useRef<THREE.Group | null>(null)
  const trailLineRef = useRef<THREE.Line | null>(null)

  // Create robot marker
  useEffect(() => {
    // Create a group for the robot
    const group = new THREE.Group()
    group.name = 'robot'

    // Robot body (circle)
    const bodyGeometry = new THREE.CircleGeometry(0.3, 32)
    const bodyMaterial = new THREE.MeshBasicMaterial({
      color: ROBOT_COLORS.body,
      side: THREE.DoubleSide,
    })
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial)
    body.position.z = 0.01
    group.add(body)

    // Direction arrow
    const arrowShape = new THREE.Shape()
    arrowShape.moveTo(0, 0.4)
    arrowShape.lineTo(-0.15, 0.1)
    arrowShape.lineTo(0.15, 0.1)
    arrowShape.closePath()

    const arrowGeometry = new THREE.ShapeGeometry(arrowShape)
    const arrowMaterial = new THREE.MeshBasicMaterial({
      color: ROBOT_COLORS.arrow,
      side: THREE.DoubleSide,
    })
    const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial)
    arrow.position.z = 0.02
    group.add(arrow)

    // Outer ring
    const ringGeometry = new THREE.RingGeometry(0.28, 0.32, 32)
    const ringMaterial = new THREE.MeshBasicMaterial({
      color: ROBOT_COLORS.arrow,
      side: THREE.DoubleSide,
    })
    const ring = new THREE.Mesh(ringGeometry, ringMaterial)
    ring.position.z = 0.015
    group.add(ring)

    scene.add(group)
    robotGroupRef.current = group

    return () => {
      if (robotGroupRef.current) {
        robotGroupRef.current.traverse((child) => {
          if (child instanceof THREE.Mesh) {
            child.geometry.dispose()
            ;(child.material as THREE.Material).dispose()
          }
        })
        scene.remove(robotGroupRef.current)
        robotGroupRef.current = null
      }
    }
  }, [scene])

  // Update robot position and orientation
  useEffect(() => {
    if (!robotGroupRef.current || !pose) return

    const { position, orientation } = pose
    const yaw = quaternionToYaw(orientation)

    robotGroupRef.current.position.set(position.x, position.y, zIndex * 0.01 + 0.1)
    robotGroupRef.current.rotation.z = yaw
  }, [pose, zIndex])

  // Create/update trail
  useEffect(() => {
    if (trail.length < 2) {
      if (trailLineRef.current) {
        scene.remove(trailLineRef.current)
        trailLineRef.current.geometry.dispose()
        ;(trailLineRef.current.material as THREE.Material).dispose()
        trailLineRef.current = null
      }
      return
    }

    const points = trail.map((p) => new THREE.Vector3(p.x, p.y, zIndex * 0.01))
    const geometry = new THREE.BufferGeometry().setFromPoints(points)

    if (!trailLineRef.current) {
      const material = new THREE.LineBasicMaterial({
        color: ROBOT_COLORS.trail,
        linewidth: 2,
        transparent: true,
        opacity: 0.6,
      })
      trailLineRef.current = new THREE.Line(geometry, material)
      trailLineRef.current.name = 'robotTrail'
      scene.add(trailLineRef.current)
    } else {
      trailLineRef.current.geometry.dispose()
      trailLineRef.current.geometry = geometry
    }

    return () => {
      if (trailLineRef.current) {
        scene.remove(trailLineRef.current)
        trailLineRef.current.geometry.dispose()
        ;(trailLineRef.current.material as THREE.Material).dispose()
        trailLineRef.current = null
      }
    }
  }, [scene, trail, zIndex])

  return null
}
