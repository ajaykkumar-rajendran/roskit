import { useEffect, useRef } from 'react'
import * as THREE from 'three'
import type { OccupancyGrid } from '@/types/map'
import { getOccupancyColor } from '@/lib/map/colorMaps'

interface OccupancyGridLayerProps {
  scene: THREE.Scene
  occupancyGrid: OccupancyGrid
  opacity: number
  zIndex: number
}

export function OccupancyGridLayer({
  scene,
  occupancyGrid,
  opacity,
  zIndex,
}: OccupancyGridLayerProps) {
  const meshRef = useRef<THREE.Mesh | null>(null)
  const textureRef = useRef<THREE.DataTexture | null>(null)

  useEffect(() => {
    if (!occupancyGrid || !occupancyGrid.info) return

    const { width, height, resolution, origin } = occupancyGrid.info
    const { data } = occupancyGrid

    // Create texture from occupancy data
    const textureData = new Uint8Array(width * height * 4)

    for (let i = 0; i < data.length; i++) {
      const value = data[i]
      const color = getOccupancyColor(value)

      // Flip Y coordinate for proper orientation
      const x = i % width
      const y = Math.floor(i / width)
      const flippedIndex = (height - 1 - y) * width + x

      textureData[flippedIndex * 4 + 0] = color.r
      textureData[flippedIndex * 4 + 1] = color.g
      textureData[flippedIndex * 4 + 2] = color.b
      textureData[flippedIndex * 4 + 3] = color.a
    }

    // Create or update texture
    if (!textureRef.current) {
      textureRef.current = new THREE.DataTexture(
        textureData,
        width,
        height,
        THREE.RGBAFormat
      )
      textureRef.current.minFilter = THREE.NearestFilter
      textureRef.current.magFilter = THREE.NearestFilter
    } else {
      textureRef.current.image.data = textureData
      textureRef.current.image.width = width
      textureRef.current.image.height = height
    }
    textureRef.current.needsUpdate = true

    // Calculate world dimensions
    const worldWidth = width * resolution
    const worldHeight = height * resolution

    // Create or update mesh
    if (!meshRef.current) {
      const geometry = new THREE.PlaneGeometry(worldWidth, worldHeight)
      const material = new THREE.MeshBasicMaterial({
        map: textureRef.current,
        transparent: true,
        opacity,
        side: THREE.DoubleSide,
      })

      meshRef.current = new THREE.Mesh(geometry, material)
      meshRef.current.name = 'occupancyGrid'
      scene.add(meshRef.current)
    } else {
      // Update geometry if size changed
      const mesh = meshRef.current
      mesh.geometry.dispose()
      mesh.geometry = new THREE.PlaneGeometry(worldWidth, worldHeight)

      // Update material
      const material = mesh.material as THREE.MeshBasicMaterial
      material.map = textureRef.current
      material.opacity = opacity
      material.needsUpdate = true
    }

    // Position the mesh based on origin
    // The origin is at the bottom-left corner of the map
    meshRef.current.position.set(
      origin.position.x + worldWidth / 2,
      origin.position.y + worldHeight / 2,
      zIndex * 0.01 // Small z offset for layering
    )

    return () => {
      if (meshRef.current) {
        scene.remove(meshRef.current)
        meshRef.current.geometry.dispose()
        ;(meshRef.current.material as THREE.Material).dispose()
        meshRef.current = null
      }
      if (textureRef.current) {
        textureRef.current.dispose()
        textureRef.current = null
      }
    }
  }, [scene, occupancyGrid, opacity, zIndex])

  // Update opacity
  useEffect(() => {
    if (meshRef.current) {
      const material = meshRef.current.material as THREE.MeshBasicMaterial
      material.opacity = opacity
    }
  }, [opacity])

  return null // This component only manages Three.js objects
}
