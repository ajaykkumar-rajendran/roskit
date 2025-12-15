/**
 * Decoder Tests
 *
 * Tests the decoding of various message formats including
 * LaserScan binary, CBOR, and metadata-prefixed images.
 */

import { describe, it, expect } from 'vitest'
import { encode as cborEncode } from 'cbor-x'
import {
  decodeCbor,
  decodeBinaryFloat32,
  decodeBinaryFloat64,
  decodeBinaryInt32,
  decodeLaserScanBinary,
} from '../decoders'

describe('CBOR Decoder', () => {
  it('should decode simple object', () => {
    const original = { foo: 'bar', num: 42 }
    const encoded = new Uint8Array(cborEncode(original))
    const decoded = decodeCbor<typeof original>(encoded)

    expect(decoded).toEqual(original)
  })

  it('should decode nested objects', () => {
    const original = {
      header: {
        stamp: { sec: 123, nanosec: 456 },
        frame_id: 'base_link',
      },
    }
    const encoded = new Uint8Array(cborEncode(original))
    const decoded = decodeCbor<typeof original>(encoded)

    expect(decoded).toEqual(original)
  })

  it('should decode arrays', () => {
    const original = { values: [1, 2, 3, 4, 5] }
    const encoded = new Uint8Array(cborEncode(original))
    const decoded = decodeCbor<typeof original>(encoded)

    expect(decoded).toEqual(original)
  })

  it('should handle empty objects', () => {
    const original = {}
    const encoded = new Uint8Array(cborEncode(original))
    const decoded = decodeCbor(encoded)

    expect(decoded).toEqual(original)
  })
})

describe('Binary Float32 Decoder', () => {
  it('should decode float32 array', () => {
    const original = new Float32Array([1.0, 2.5, 3.14159, -4.0])
    const payload = new Uint8Array(original.buffer)
    const decoded = decodeBinaryFloat32(payload)

    expect(decoded.length).toBe(4)
    expect(decoded[0]).toBeCloseTo(1.0)
    expect(decoded[1]).toBeCloseTo(2.5)
    expect(decoded[2]).toBeCloseTo(3.14159, 4)
    expect(decoded[3]).toBeCloseTo(-4.0)
  })

  it('should handle empty array', () => {
    const payload = new Uint8Array(0)
    const decoded = decodeBinaryFloat32(payload)

    expect(decoded.length).toBe(0)
  })

  it('should handle large arrays', () => {
    const original = new Float32Array(10000)
    for (let i = 0; i < 10000; i++) {
      original[i] = Math.random() * 100
    }
    const payload = new Uint8Array(original.buffer)
    const decoded = decodeBinaryFloat32(payload)

    expect(decoded.length).toBe(10000)
    for (let i = 0; i < 10000; i++) {
      expect(decoded[i]).toBeCloseTo(original[i], 5)
    }
  })

  it('should handle typed array views with offset', () => {
    // Create a larger buffer with offset
    const buffer = new ArrayBuffer(100)
    const view = new Uint8Array(buffer, 10, 16) // 4 floats starting at offset 10
    const floats = new Float32Array(4)
    floats.set([1.0, 2.0, 3.0, 4.0])
    view.set(new Uint8Array(floats.buffer))

    const decoded = decodeBinaryFloat32(view)
    expect(decoded.length).toBe(4)
  })
})

describe('Binary Float64 Decoder', () => {
  it('should decode float64 array', () => {
    const original = new Float64Array([1.0, 2.5, Math.PI, -4.0])
    const payload = new Uint8Array(original.buffer)
    const decoded = decodeBinaryFloat64(payload)

    expect(decoded.length).toBe(4)
    expect(decoded[0]).toBe(1.0)
    expect(decoded[1]).toBe(2.5)
    expect(decoded[2]).toBe(Math.PI)
    expect(decoded[3]).toBe(-4.0)
  })
})

describe('Binary Int32 Decoder', () => {
  it('should decode int32 array', () => {
    const original = new Int32Array([1, -2, 100, -1000000])
    const payload = new Uint8Array(original.buffer)
    const decoded = decodeBinaryInt32(payload)

    expect(decoded.length).toBe(4)
    expect(decoded[0]).toBe(1)
    expect(decoded[1]).toBe(-2)
    expect(decoded[2]).toBe(100)
    expect(decoded[3]).toBe(-1000000)
  })
})

describe('LaserScan Binary Decoder', () => {
  /**
   * Create a mock LaserScan payload with metadata prefix
   * Format: [metadata_length (4 bytes)][CBOR metadata][ranges][intensities]
   */
  function createLaserScanPayload(
    ranges: number[],
    intensities: number[],
    metadata: {
      angle_min: number
      angle_max: number
      angle_increment: number
      range_min: number
      range_max: number
    }
  ): Uint8Array {
    const fullMetadata = {
      ...metadata,
      ranges_count: ranges.length,
      intensities_count: intensities.length,
    }

    const metadataCbor = new Uint8Array(cborEncode(fullMetadata))

    // Create ranges and intensities as Float32Array
    const rangesFloat = new Float32Array(ranges)
    const intensitiesFloat = new Float32Array(intensities)

    // Calculate total size
    const totalSize =
      4 + // metadata length
      metadataCbor.byteLength +
      rangesFloat.byteLength +
      intensitiesFloat.byteLength

    const payload = new Uint8Array(totalSize)
    const view = new DataView(payload.buffer)

    // Write metadata length (big-endian)
    view.setUint32(0, metadataCbor.byteLength, false)

    // Write metadata
    payload.set(metadataCbor, 4)

    // Write ranges
    const rangesOffset = 4 + metadataCbor.byteLength
    payload.set(new Uint8Array(rangesFloat.buffer), rangesOffset)

    // Write intensities
    const intensitiesOffset = rangesOffset + rangesFloat.byteLength
    payload.set(new Uint8Array(intensitiesFloat.buffer), intensitiesOffset)

    return payload
  }

  it('should decode LaserScan with metadata prefix', () => {
    const ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
    const intensities = [100.0, 200.0, 300.0, 400.0, 500.0]
    const metadata = {
      angle_min: -1.57,
      angle_max: 1.57,
      angle_increment: 0.01,
      range_min: 0.1,
      range_max: 30.0,
    }

    const payload = createLaserScanPayload(ranges, intensities, metadata)
    const decoded = decodeLaserScanBinary(payload)

    expect(decoded.angle_min).toBeCloseTo(-1.57, 2)
    expect(decoded.angle_max).toBeCloseTo(1.57, 2)
    expect(decoded.angle_increment).toBeCloseTo(0.01, 2)
    expect(decoded.range_min).toBeCloseTo(0.1, 2)
    expect(decoded.range_max).toBeCloseTo(30.0, 2)

    expect(decoded.ranges.length).toBe(5)
    expect(decoded.intensities.length).toBe(5)

    for (let i = 0; i < 5; i++) {
      expect(decoded.ranges[i]).toBeCloseTo(ranges[i], 4)
      expect(decoded.intensities[i]).toBeCloseTo(intensities[i], 4)
    }
  })

  it('should decode LaserScan with empty intensities', () => {
    const ranges = [1.0, 2.0, 3.0]
    const intensities: number[] = []
    const metadata = {
      angle_min: -0.5,
      angle_max: 0.5,
      angle_increment: 0.1,
      range_min: 0.0,
      range_max: 10.0,
    }

    const payload = createLaserScanPayload(ranges, intensities, metadata)
    const decoded = decodeLaserScanBinary(payload)

    expect(decoded.ranges.length).toBe(3)
    expect(decoded.intensities.length).toBe(0)
  })

  it('should handle large LaserScan data', () => {
    // Simulate a typical LiDAR with 1080 points
    const numPoints = 1080
    const ranges = Array.from({ length: numPoints }, () => Math.random() * 30)
    const intensities = Array.from({ length: numPoints }, () => Math.random() * 1000)
    const metadata = {
      angle_min: -Math.PI,
      angle_max: Math.PI,
      angle_increment: (2 * Math.PI) / numPoints,
      range_min: 0.1,
      range_max: 30.0,
    }

    const payload = createLaserScanPayload(ranges, intensities, metadata)
    const decoded = decodeLaserScanBinary(payload)

    expect(decoded.ranges.length).toBe(numPoints)
    expect(decoded.intensities.length).toBe(numPoints)
  })

  it('should fallback to legacy format when no metadata prefix', () => {
    // Legacy format: [angle_min, angle_max, angle_increment, range_min, range_max, ...ranges]
    const header = new Float32Array([
      -1.57, // angle_min
      1.57, // angle_max
      0.01, // angle_increment
      0.1, // range_min
      30.0, // range_max
    ])
    const ranges = new Float32Array([1.0, 2.0, 3.0, 4.0])
    const intensities = new Float32Array([100.0, 200.0, 300.0, 400.0])

    // Combine into single buffer
    const totalFloats = header.length + ranges.length + intensities.length
    const combined = new Float32Array(totalFloats)
    combined.set(header, 0)
    combined.set(ranges, header.length)
    combined.set(intensities, header.length + ranges.length)

    const payload = new Uint8Array(combined.buffer)
    const decoded = decodeLaserScanBinary(payload)

    // Note: Legacy format doesn't distinguish ranges from intensities cleanly
    // The decoder splits remaining floats 50/50
    expect(decoded.angle_min).toBeCloseTo(-1.57, 2)
    expect(decoded.angle_max).toBeCloseTo(1.57, 2)
  })
})

describe('Metadata Prefix Detection', () => {
  it('should detect valid metadata prefix', () => {
    const metadata = { test: 'value' }
    const metadataCbor = new Uint8Array(cborEncode(metadata))
    const imageData = new Uint8Array([0x89, 0x50, 0x4e, 0x47]) // PNG magic

    const payload = new Uint8Array(4 + metadataCbor.byteLength + imageData.byteLength)
    const view = new DataView(payload.buffer)
    view.setUint32(0, metadataCbor.byteLength, false)
    payload.set(metadataCbor, 4)
    payload.set(imageData, 4 + metadataCbor.byteLength)

    // Check that metadata length is detected correctly
    const detectedLength = view.getUint32(0, false)
    expect(detectedLength).toBe(metadataCbor.byteLength)
  })

  it('should handle payloads without metadata prefix', () => {
    // Raw PNG data starts with magic bytes that won't be valid CBOR length
    const pngMagic = new Uint8Array([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a])

    const view = new DataView(pngMagic.buffer)
    const possibleLength = view.getUint32(0, false)

    // PNG magic interpreted as length would be very large
    expect(possibleLength).toBeGreaterThan(pngMagic.byteLength)
  })
})
