import * as THREE from 'three'

// Costmap color gradient (similar to RViz costmap visualization)
// 0 = free (transparent/light blue), 100 = lethal (purple), 254 = inscribed, 255 = unknown
export function getCostmapColor(cost: number): THREE.Color {
  if (cost === -1 || cost === 255) {
    // Unknown
    return new THREE.Color(0.5, 0.5, 0.5)
  }

  if (cost === 0) {
    // Free
    return new THREE.Color(0.0, 0.0, 0.3) // Dark blue
  }

  if (cost === 254) {
    // Inscribed (lethal within inscribed radius)
    return new THREE.Color(0.0, 1.0, 1.0) // Cyan
  }

  if (cost >= 253) {
    // Lethal
    return new THREE.Color(1.0, 0.0, 1.0) // Magenta
  }

  // Gradient from blue (low cost) to red (high cost)
  const normalized = cost / 252

  if (normalized < 0.5) {
    // Blue to yellow
    const t = normalized * 2
    return new THREE.Color(t, t, 1 - t)
  } else {
    // Yellow to red
    const t = (normalized - 0.5) * 2
    return new THREE.Color(1, 1 - t, 0)
  }
}

// Occupancy grid colors
export function getOccupancyColor(value: number): { r: number; g: number; b: number; a: number } {
  if (value === -1) {
    // Unknown
    return { r: 128, g: 128, b: 128, a: 255 }
  }

  if (value === 0) {
    // Free
    return { r: 240, g: 240, b: 240, a: 255 }
  }

  if (value >= 100) {
    // Occupied
    return { r: 0, g: 0, b: 0, a: 255 }
  }

  // Partially occupied (gradient)
  const occupancy = value / 100
  const gray = Math.round(240 - occupancy * 240)
  return { r: gray, g: gray, b: gray, a: 255 }
}

// LaserScan intensity to color
export function getIntensityColor(intensity: number, minIntensity: number, maxIntensity: number): THREE.Color {
  const range = maxIntensity - minIntensity
  if (range === 0) {
    return new THREE.Color(1, 0, 0) // Default red
  }

  const normalized = (intensity - minIntensity) / range

  // Hot colormap (black -> red -> yellow -> white)
  if (normalized < 0.33) {
    const t = normalized / 0.33
    return new THREE.Color(t, 0, 0)
  } else if (normalized < 0.66) {
    const t = (normalized - 0.33) / 0.33
    return new THREE.Color(1, t, 0)
  } else {
    const t = (normalized - 0.66) / 0.34
    return new THREE.Color(1, 1, t)
  }
}

// Path colors presets
export const PATH_COLORS = {
  globalPlan: '#00ff00', // Green
  localPlan: '#ffff00', // Yellow
  selectedPath: '#00ffff', // Cyan
  executedPath: '#888888', // Gray
}

// Robot color
export const ROBOT_COLORS = {
  body: '#2196f3', // Blue
  arrow: '#ffffff', // White
  trail: '#4caf50', // Green
}

// Create a gradient texture for costmap
export function createCostmapGradientTexture(): THREE.DataTexture {
  const width = 256
  const data = new Uint8Array(width * 4)

  for (let i = 0; i < width; i++) {
    const color = getCostmapColor(i)
    const alpha = i === 0 ? 0 : 200 // Free space is more transparent

    data[i * 4 + 0] = Math.round(color.r * 255)
    data[i * 4 + 1] = Math.round(color.g * 255)
    data[i * 4 + 2] = Math.round(color.b * 255)
    data[i * 4 + 3] = alpha
  }

  const texture = new THREE.DataTexture(data, width, 1, THREE.RGBAFormat)
  texture.needsUpdate = true
  return texture
}
