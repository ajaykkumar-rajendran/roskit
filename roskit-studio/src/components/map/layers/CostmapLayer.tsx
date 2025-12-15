import { useEffect, useRef } from 'react'
import * as THREE from 'three'
import type { OccupancyGrid } from '@/types/map'
import { getCostmapColor } from '@/lib/map/colorMaps'

interface CostmapLayerProps {
  scene: THREE.Scene
  costmap: OccupancyGrid
  opacity: number
  zIndex: number
}

export function CostmapLayer({
  scene,
  costmap,
  opacity,
  zIndex,
}: CostmapLayerProps) {
  const meshRef = useRef<THREE.Mesh | null>(null)
  const textureRef = useRef<THREE.DataTexture | null>(null)

  useEffect(() => {
    if (!costmap || !costmap.info) return

    const { width, height, resolution, origin } = costmap.info
    const { data } = costmap

    // Create texture from costmap data
    const textureData = new Uint8Array(width * height * 4)

    for (let i = 0; i < data.length; i++) {
      const value = data[i]
      const color = getCostmapColor(value)

      // Flip Y coordinate for proper orientation
      const x = i % width
      const y = Math.floor(i / width)
      const flippedIndex = (height - 1 - y) * width + x

      textureData[flippedIndex * 4 + 0] = Math.round(color.r * 255)
      textureData[flippedIndex * 4 + 1] = Math.round(color.g * 255)
      textureData[flippedIndex * 4 + 2] = Math.round(color.b * 255)
      // Make free space (0) more transparent
      textureData[flippedIndex * 4 + 3] = value === 0 ? 0 : Math.round(opacity * 255)
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
        opacity: 1, // Opacity is baked into texture
        side: THREE.DoubleSide,
        depthWrite: false, // Prevent z-fighting with base map
      })

      meshRef.current = new THREE.Mesh(geometry, material)
      meshRef.current.name = 'costmap'
      scene.add(meshRef.current)
    } else {
      // Update geometry if size changed
      const mesh = meshRef.current
      mesh.geometry.dispose()
      mesh.geometry = new THREE.PlaneGeometry(worldWidth, worldHeight)

      // Update material
      const material = mesh.material as THREE.MeshBasicMaterial
      material.map = textureRef.current
      material.needsUpdate = true
    }

    // Position the mesh based on origin
    meshRef.current.position.set(
      origin.position.x + worldWidth / 2,
      origin.position.y + worldHeight / 2,
      zIndex * 0.01
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
  }, [scene, costmap, opacity, zIndex])

  return null
}
