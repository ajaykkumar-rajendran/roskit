/**
 * Media Round-Trip Tests
 *
 * Tests encoding/decoding of image (PNG, JPEG) and binary (LaserScan) payloads.
 */

import { describe, it, expect } from 'vitest'
import { encode as cborEncode } from 'cbor-x'
import {
  MessageType,
  Encoding,
  createMessage,
  parseMessage,
} from '../protocol'
import {
  decodeCbor,
  decodeBinaryFloat32,
  decodeBinaryFloat64,
  decodeBinaryInt32,
  decodeLaserScanBinary,
} from '../decoders'

describe('Binary Float32 Round-Trip', () => {
  it('should encode and decode aligned float32 array', () => {
    const original = new Float32Array([1.5, 2.5, 3.5, -4.5, 0, 100.123])
    const payload = new Uint8Array(original.buffer)

    const message = createMessage(MessageType.MESSAGE, 1, payload, Encoding.BINARY)
    const [header, decoded] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.encoding).toBe(Encoding.BINARY)

    const result = decodeBinaryFloat32(decoded)
    expect(result.length).toBe(original.length)
    for (let i = 0; i < original.length; i++) {
      expect(result[i]).toBeCloseTo(original[i], 5)
    }
  })

  it('should handle unaligned float32 array', () => {
    const original = new Float32Array([1.0, 2.0, 3.0, 4.0])
    // Create unaligned buffer by adding 1 byte prefix
    const unaligned = new Uint8Array(1 + original.byteLength)
    unaligned.set(new Uint8Array(original.buffer), 1)

    const slice = unaligned.slice(1)
    const result = decodeBinaryFloat32(slice)

    expect(result.length).toBe(original.length)
    for (let i = 0; i < original.length; i++) {
      expect(result[i]).toBeCloseTo(original[i], 5)
    }
  })

  it('should handle empty float32 array', () => {
    const empty = new Uint8Array(0)
    const result = decodeBinaryFloat32(empty)
    expect(result.length).toBe(0)
  })

  it('should handle partial float (truncated bytes)', () => {
    // 5 bytes = 1 float + 1 extra byte (ignored)
    const data = new Uint8Array(5)
    const view = new DataView(data.buffer)
    view.setFloat32(0, 42.5, true)

    const result = decodeBinaryFloat32(data)
    expect(result.length).toBe(1)
    expect(result[0]).toBeCloseTo(42.5, 5)
  })
})

describe('Binary Float64 Round-Trip', () => {
  it('should encode and decode float64 array', () => {
    const original = new Float64Array([Math.PI, Math.E, -1.23456789012345])
    const payload = new Uint8Array(original.buffer)

    const result = decodeBinaryFloat64(payload)
    expect(result.length).toBe(original.length)
    for (let i = 0; i < original.length; i++) {
      expect(result[i]).toBeCloseTo(original[i], 10)
    }
  })

  it('should handle unaligned float64 array', () => {
    const original = new Float64Array([1.0, 2.0])
    const unaligned = new Uint8Array(3 + original.byteLength)
    unaligned.set(new Uint8Array(original.buffer), 3)

    const slice = unaligned.slice(3)
    const result = decodeBinaryFloat64(slice)

    expect(result.length).toBe(original.length)
    for (let i = 0; i < original.length; i++) {
      expect(result[i]).toBeCloseTo(original[i], 10)
    }
  })
})

describe('Binary Int32 Round-Trip', () => {
  it('should encode and decode int32 array', () => {
    const original = new Int32Array([0, 1, -1, 2147483647, -2147483648])
    const payload = new Uint8Array(original.buffer)

    const result = decodeBinaryInt32(payload)
    expect(result.length).toBe(original.length)
    for (let i = 0; i < original.length; i++) {
      expect(result[i]).toBe(original[i])
    }
  })

  it('should handle unaligned int32 array', () => {
    const original = new Int32Array([100, 200, 300])
    const unaligned = new Uint8Array(2 + original.byteLength)
    unaligned.set(new Uint8Array(original.buffer), 2)

    const slice = unaligned.slice(2)
    const result = decodeBinaryInt32(slice)

    expect(result.length).toBe(original.length)
    for (let i = 0; i < original.length; i++) {
      expect(result[i]).toBe(original[i])
    }
  })
})

describe('LaserScan Binary Round-Trip', () => {
  it('should encode and decode LaserScan with metadata format', () => {
    const metadata = {
      angle_min: -Math.PI,
      angle_max: Math.PI,
      angle_increment: 0.01,
      range_min: 0.1,
      range_max: 30.0,
      ranges_count: 4,
    }
    const ranges = new Float32Array([1.0, 2.0, 3.0, 4.0])
    const intensities = new Float32Array([100, 200, 300, 400])

    // Encode metadata
    const metaCbor = new Uint8Array(cborEncode(metadata))
    const metaLengthBytes = new Uint8Array(4)
    new DataView(metaLengthBytes.buffer).setUint32(0, metaCbor.byteLength, false)

    // Combine: [meta_length][meta_cbor][ranges][intensities]
    const payload = new Uint8Array(
      4 + metaCbor.byteLength + ranges.byteLength + intensities.byteLength
    )
    payload.set(metaLengthBytes, 0)
    payload.set(metaCbor, 4)
    payload.set(new Uint8Array(ranges.buffer), 4 + metaCbor.byteLength)
    payload.set(new Uint8Array(intensities.buffer), 4 + metaCbor.byteLength + ranges.byteLength)

    const message = createMessage(MessageType.MESSAGE, 10, payload, Encoding.BINARY)
    const [header, decoded] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.channelId).toBe(10)

    const result = decodeLaserScanBinary(decoded)
    expect(result.angle_min).toBeCloseTo(-Math.PI, 5)
    expect(result.angle_max).toBeCloseTo(Math.PI, 5)
    expect(result.angle_increment).toBeCloseTo(0.01, 5)
    expect(result.range_min).toBeCloseTo(0.1, 5)
    expect(result.range_max).toBeCloseTo(30.0, 5)
    expect(result.ranges.length).toBe(4)
    expect(result.intensities.length).toBe(4)

    for (let i = 0; i < 4; i++) {
      expect(result.ranges[i]).toBeCloseTo(ranges[i], 5)
      expect(result.intensities[i]).toBeCloseTo(intensities[i], 5)
    }
  })

  it('should handle legacy LaserScan format (header floats)', () => {
    // Legacy format: [angle_min, angle_max, angle_increment, range_min, range_max, ...ranges, ...intensities]
    const headerFloats = new Float32Array([
      -1.57, // angle_min
      1.57,  // angle_max
      0.1,   // angle_increment
      0.05,  // range_min
      10.0,  // range_max
    ])
    const ranges = new Float32Array([1.0, 2.0, 3.0])
    const intensities = new Float32Array([50, 60, 70])

    const allFloats = new Float32Array(headerFloats.length + ranges.length + intensities.length)
    allFloats.set(headerFloats, 0)
    allFloats.set(ranges, headerFloats.length)
    allFloats.set(intensities, headerFloats.length + ranges.length)

    const payload = new Uint8Array(allFloats.buffer)
    const result = decodeLaserScanBinary(payload)

    expect(result.angle_min).toBeCloseTo(-1.57, 5)
    expect(result.angle_max).toBeCloseTo(1.57, 5)
    expect(result.angle_increment).toBeCloseTo(0.1, 5)
    expect(result.range_min).toBeCloseTo(0.05, 5)
    expect(result.range_max).toBeCloseTo(10.0, 5)
  })

  it('should handle LaserScan with no intensities', () => {
    const metadata = {
      angle_min: 0,
      angle_max: 3.14,
      angle_increment: 0.1,
      range_min: 0.0,
      range_max: 50.0,
      ranges_count: 3,
    }
    const ranges = new Float32Array([5.0, 10.0, 15.0])

    const metaCbor = new Uint8Array(cborEncode(metadata))
    const metaLengthBytes = new Uint8Array(4)
    new DataView(metaLengthBytes.buffer).setUint32(0, metaCbor.byteLength, false)

    const payload = new Uint8Array(4 + metaCbor.byteLength + ranges.byteLength)
    payload.set(metaLengthBytes, 0)
    payload.set(metaCbor, 4)
    payload.set(new Uint8Array(ranges.buffer), 4 + metaCbor.byteLength)

    const result = decodeLaserScanBinary(payload)
    expect(result.ranges.length).toBe(3)
    expect(result.intensities.length).toBe(0)
  })

  it('should handle large LaserScan (360 degree scan)', () => {
    const numPoints = 360
    const metadata = {
      angle_min: 0,
      angle_max: 2 * Math.PI,
      angle_increment: (2 * Math.PI) / numPoints,
      range_min: 0.1,
      range_max: 30.0,
      ranges_count: numPoints,
    }

    const ranges = new Float32Array(numPoints)
    const intensities = new Float32Array(numPoints)
    for (let i = 0; i < numPoints; i++) {
      ranges[i] = 5.0 + Math.sin(i * 0.1)
      intensities[i] = 100 + i
    }

    const metaCbor = new Uint8Array(cborEncode(metadata))
    const metaLengthBytes = new Uint8Array(4)
    new DataView(metaLengthBytes.buffer).setUint32(0, metaCbor.byteLength, false)

    const payload = new Uint8Array(
      4 + metaCbor.byteLength + ranges.byteLength + intensities.byteLength
    )
    payload.set(metaLengthBytes, 0)
    payload.set(metaCbor, 4)
    payload.set(new Uint8Array(ranges.buffer), 4 + metaCbor.byteLength)
    payload.set(new Uint8Array(intensities.buffer), 4 + metaCbor.byteLength + ranges.byteLength)

    const result = decodeLaserScanBinary(payload)
    expect(result.ranges.length).toBe(numPoints)
    expect(result.intensities.length).toBe(numPoints)
  })
})

describe('CBOR Binary Data Round-Trip', () => {
  it('should handle nested binary arrays in CBOR', () => {
    const pointCloud = {
      header: {
        stamp: { sec: 1234567890, nanosec: 123456789 },
        frame_id: 'base_link',
      },
      points: Array.from({ length: 100 }, (_, i) => ({
        x: i * 0.1,
        y: i * 0.2,
        z: i * 0.3,
      })),
      is_dense: true,
    }

    const encoded = new Uint8Array(cborEncode(pointCloud))
    const message = createMessage(MessageType.MESSAGE, 5, encoded, Encoding.CBOR)
    const [header, payload] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.encoding).toBe(Encoding.CBOR)

    const decoded = decodeCbor<typeof pointCloud>(payload)
    expect(decoded.header.frame_id).toBe('base_link')
    expect(decoded.points.length).toBe(100)
    expect(decoded.points[50].x).toBeCloseTo(5.0, 5)
    expect(decoded.is_dense).toBe(true)
  })

  it('should handle Uint8Array fields in CBOR (image data)', () => {
    const imageMsg = {
      header: {
        stamp: { sec: 100, nanosec: 0 },
        frame_id: 'camera',
      },
      height: 4,
      width: 4,
      encoding: 'rgb8',
      step: 12,
      data: new Uint8Array(48).fill(128), // 4x4 RGB image
    }

    const encoded = new Uint8Array(cborEncode(imageMsg))
    const message = createMessage(MessageType.MESSAGE, 3, encoded, Encoding.CBOR)
    const [header, payload] = parseMessage(message)

    expect(header).not.toBeNull()

    const decoded = decodeCbor<typeof imageMsg>(payload)
    expect(decoded.width).toBe(4)
    expect(decoded.height).toBe(4)
    expect(decoded.encoding).toBe('rgb8')
    // CBOR may decode Uint8Array as regular array or Uint8Array
    expect(decoded.data.length).toBe(48)
  })

  it('should handle typed arrays in CBOR (OccupancyGrid)', () => {
    const width = 50
    const height = 50
    const occupancyGrid = {
      info: {
        resolution: 0.05,
        width,
        height,
        origin: {
          position: { x: -5, y: -5, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      },
      data: new Int8Array(width * height).fill(-1), // Unknown cells
    }

    // Mark some cells as occupied/free
    occupancyGrid.data[0] = 0    // Free
    occupancyGrid.data[1] = 100  // Occupied
    occupancyGrid.data[2] = 50   // Partial

    const encoded = new Uint8Array(cborEncode(occupancyGrid))
    const decoded = decodeCbor<typeof occupancyGrid>(encoded)

    expect(decoded.info.resolution).toBe(0.05)
    expect(decoded.info.width).toBe(50)
    expect(decoded.data.length).toBe(2500)
    expect(decoded.data[0]).toBe(0)
    expect(decoded.data[1]).toBe(100)
    expect(decoded.data[2]).toBe(50)
  })
})

describe('Protocol Message with Media Encoding', () => {
  it('should preserve PNG encoding flag through round-trip', () => {
    // Simulate PNG payload (just bytes, not actual PNG)
    const fakePng = new Uint8Array([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a])

    const message = createMessage(MessageType.MESSAGE, 1, fakePng, Encoding.PNG)
    const [header, payload] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.encoding).toBe(Encoding.PNG)
    expect(payload.length).toBe(fakePng.length)
    expect(payload[0]).toBe(0x89)
    expect(payload[1]).toBe(0x50)
  })

  it('should preserve JPEG encoding flag through round-trip', () => {
    // Simulate JPEG payload (just bytes, not actual JPEG)
    const fakeJpeg = new Uint8Array([0xff, 0xd8, 0xff, 0xe0])

    const message = createMessage(MessageType.MESSAGE, 2, fakeJpeg, Encoding.JPEG)
    const [header, payload] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.encoding).toBe(Encoding.JPEG)
    expect(payload.length).toBe(fakeJpeg.length)
    expect(payload[0]).toBe(0xff)
    expect(payload[1]).toBe(0xd8)
  })

  it('should handle PNG with CBOR metadata prefix', () => {
    const metadata = { width: 640, height: 480, format: 'rgb8' }
    const metaCbor = new Uint8Array(cborEncode(metadata))
    const fakePngData = new Uint8Array([0x89, 0x50, 0x4e, 0x47])

    // Format: [4-byte meta length BE][cbor metadata][png data]
    const payload = new Uint8Array(4 + metaCbor.byteLength + fakePngData.byteLength)
    const view = new DataView(payload.buffer)
    view.setUint32(0, metaCbor.byteLength, false)
    payload.set(metaCbor, 4)
    payload.set(fakePngData, 4 + metaCbor.byteLength)

    const message = createMessage(MessageType.MESSAGE, 1, payload, Encoding.PNG)
    const [header, decoded] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(header!.encoding).toBe(Encoding.PNG)
    expect(decoded.byteLength).toBe(payload.byteLength)

    // Verify metadata can be extracted
    const metaLength = new DataView(decoded.buffer, decoded.byteOffset).getUint32(0, false)
    expect(metaLength).toBe(metaCbor.byteLength)

    const extractedMeta = decodeCbor<typeof metadata>(
      decoded.slice(4, 4 + metaLength)
    )
    expect(extractedMeta.width).toBe(640)
    expect(extractedMeta.height).toBe(480)
  })
})

describe('Large Payload Handling', () => {
  it('should handle 1MB binary payload', () => {
    const size = 1024 * 1024
    const largeData = new Uint8Array(size)
    for (let i = 0; i < size; i++) {
      largeData[i] = i % 256
    }

    const message = createMessage(MessageType.MESSAGE, 1, largeData, Encoding.BINARY)
    const [header, payload] = parseMessage(message)

    expect(header).not.toBeNull()
    expect(payload.byteLength).toBe(size)

    // Verify data integrity at key points
    expect(payload[0]).toBe(0)
    expect(payload[255]).toBe(255)
    expect(payload[256]).toBe(0)
    expect(payload[size - 1]).toBe((size - 1) % 256)
  })

  it('should handle high-resolution LaserScan (1440 points)', () => {
    const numPoints = 1440 // 0.25 degree resolution
    const metadata = {
      angle_min: 0,
      angle_max: 2 * Math.PI,
      angle_increment: (2 * Math.PI) / numPoints,
      range_min: 0.01,
      range_max: 100.0,
      ranges_count: numPoints,
    }

    const ranges = new Float32Array(numPoints)
    const intensities = new Float32Array(numPoints)
    for (let i = 0; i < numPoints; i++) {
      ranges[i] = 10.0 + 5.0 * Math.sin(i * 0.01)
      intensities[i] = 1000 + Math.random() * 3000
    }

    const metaCbor = new Uint8Array(cborEncode(metadata))
    const metaLengthBytes = new Uint8Array(4)
    new DataView(metaLengthBytes.buffer).setUint32(0, metaCbor.byteLength, false)

    const payload = new Uint8Array(
      4 + metaCbor.byteLength + ranges.byteLength + intensities.byteLength
    )
    payload.set(metaLengthBytes, 0)
    payload.set(metaCbor, 4)
    payload.set(new Uint8Array(ranges.buffer), 4 + metaCbor.byteLength)
    payload.set(new Uint8Array(intensities.buffer), 4 + metaCbor.byteLength + ranges.byteLength)

    const message = createMessage(MessageType.MESSAGE, 1, payload, Encoding.BINARY)
    const [header, decoded] = parseMessage(message)

    expect(header).not.toBeNull()

    const result = decodeLaserScanBinary(decoded)
    expect(result.ranges.length).toBe(numPoints)
    expect(result.intensities.length).toBe(numPoints)
  })
})
