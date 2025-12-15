/**
 * Payload Decoders
 *
 * Decode various encoding formats from the RosWeb protocol.
 */

import { decode as cborDecode } from 'cbor-x'
import { Encoding } from '../protocol'

/**
 * Decode CBOR payload to JavaScript object
 */
export function decodeCbor<T = unknown>(payload: Uint8Array): T {
  return cborDecode(payload) as T
}

/**
 * Decode PNG image to ImageData
 */
export async function decodePng(
  payload: Uint8Array
): Promise<{
  width: number
  height: number
  data: Uint8Array
  metadata?: unknown
}> {
  // Detect optional [metadata_length][CBOR metadata][png]
  let pngStart = 0
  let metadata: unknown
  if (payload.byteLength > 8) {
    const metaLength = new DataView(payload.buffer, payload.byteOffset, payload.byteLength).getUint32(0, false)
    if (metaLength > 0 && metaLength + 4 < payload.byteLength) {
      const metaBytes = payload.slice(4, 4 + metaLength)
      metadata = decodeCbor(metaBytes)
      pngStart = 4 + metaLength
    }
  }

  // Create blob from payload slice - copy to new ArrayBuffer for BlobPart compatibility
  const pngBytes = payload.slice(pngStart)
  const arrayBuffer = new ArrayBuffer(pngBytes.byteLength)
  new Uint8Array(arrayBuffer).set(pngBytes)
  const blob = new Blob([arrayBuffer], { type: 'image/png' })
  const imageBitmap = await createImageBitmap(blob)

  // Draw to canvas to extract pixel data
  const canvas = new OffscreenCanvas(imageBitmap.width, imageBitmap.height)
  const ctx = canvas.getContext('2d')!
  ctx.drawImage(imageBitmap, 0, 0)

  const imageData = ctx.getImageData(0, 0, imageBitmap.width, imageBitmap.height)

  return {
    width: imageBitmap.width,
    height: imageBitmap.height,
    data: new Uint8Array(imageData.data.buffer),
    metadata,
  }
}

/**
 * Decode JPEG image to ImageData
 */
export async function decodeJpeg(
  payload: Uint8Array
): Promise<{
  width: number
  height: number
  data: Uint8Array
  metadata?: unknown
}> {
  let jpegStart = 0
  let metadata: unknown
  if (payload.byteLength > 8) {
    const metaLength = new DataView(payload.buffer, payload.byteOffset, payload.byteLength).getUint32(0, false)
    if (metaLength > 0 && metaLength + 4 < payload.byteLength) {
      const metaBytes = payload.slice(4, 4 + metaLength)
      metadata = decodeCbor(metaBytes)
      jpegStart = 4 + metaLength
    }
  }

  const jpegBytes = payload.slice(jpegStart)
  const arrayBuffer = new ArrayBuffer(jpegBytes.byteLength)
  new Uint8Array(arrayBuffer).set(jpegBytes)
  const blob = new Blob([arrayBuffer], { type: 'image/jpeg' })
  const imageBitmap = await createImageBitmap(blob)

  const canvas = new OffscreenCanvas(imageBitmap.width, imageBitmap.height)
  const ctx = canvas.getContext('2d')!
  ctx.drawImage(imageBitmap, 0, 0)

  const imageData = ctx.getImageData(0, 0, imageBitmap.width, imageBitmap.height)

  return {
    width: imageBitmap.width,
    height: imageBitmap.height,
    data: new Uint8Array(imageData.data.buffer),
    metadata,
  }
}

/**
 * Decode binary float32 array
 * Handles unaligned data by copying when necessary
 */
export function decodeBinaryFloat32(payload: Uint8Array): Float32Array {
  const count = Math.floor(payload.byteLength / 4)
  // Check if the offset is aligned to 4 bytes
  if (payload.byteOffset % 4 === 0) {
    return new Float32Array(payload.buffer, payload.byteOffset, count)
  }
  // Unaligned - need to copy the data
  const aligned = new Float32Array(count)
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength)
  for (let i = 0; i < count; i++) {
    aligned[i] = view.getFloat32(i * 4, true) // little-endian
  }
  return aligned
}

/**
 * Decode binary float64 array
 * Handles unaligned data by copying when necessary
 */
export function decodeBinaryFloat64(payload: Uint8Array): Float64Array {
  const count = Math.floor(payload.byteLength / 8)
  // Check if the offset is aligned to 8 bytes
  if (payload.byteOffset % 8 === 0) {
    return new Float64Array(payload.buffer, payload.byteOffset, count)
  }
  // Unaligned - need to copy the data
  const aligned = new Float64Array(count)
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength)
  for (let i = 0; i < count; i++) {
    aligned[i] = view.getFloat64(i * 8, true) // little-endian
  }
  return aligned
}

/**
 * Decode binary int32 array
 * Handles unaligned data by copying when necessary
 */
export function decodeBinaryInt32(payload: Uint8Array): Int32Array {
  const count = Math.floor(payload.byteLength / 4)
  // Check if the offset is aligned to 4 bytes
  if (payload.byteOffset % 4 === 0) {
    return new Int32Array(payload.buffer, payload.byteOffset, count)
  }
  // Unaligned - need to copy the data
  const aligned = new Int32Array(count)
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength)
  for (let i = 0; i < count; i++) {
    aligned[i] = view.getInt32(i * 4, true) // little-endian
  }
  return aligned
}

/**
 * Decode LaserScan binary format
 * Format: [angle_min, angle_max, angle_increment, range_min, range_max, ...ranges, ...intensities]
 */
export function decodeLaserScanBinary(payload: Uint8Array): {
  angle_min: number
  angle_max: number
  angle_increment: number
  range_min: number
  range_max: number
  ranges: Float32Array
  intensities: Float32Array
} {
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength)
  const metaLength = payload.byteLength >= 4 ? view.getUint32(0, false) : 0

  // If the leading u32 is a sensible metadata length, parse metadata-aware format.
  const hasMetadata = metaLength > 0 && metaLength < payload.byteLength - 4

  if (hasMetadata) {
    const metaStart = 4
    const metaEnd = metaStart + metaLength
    const metadata = decodeCbor<any>(payload.slice(metaStart, metaEnd))

    const dataStart = metaEnd
    const floatCount = Math.floor((payload.byteLength - dataStart) / 4)
    const floats = decodeBinaryFloat32(payload.slice(dataStart, dataStart + floatCount * 4))

    const rangesCount = metadata?.ranges_count ?? Math.floor(floatCount / 2)
    const ranges = floats.slice(0, rangesCount)
    const intensities = floats.slice(rangesCount)

    return {
      angle_min: metadata?.angle_min ?? 0,
      angle_max: metadata?.angle_max ?? 0,
      angle_increment: metadata?.angle_increment ?? 0,
      range_min: metadata?.range_min ?? 0,
      range_max: metadata?.range_max ?? 0,
      ranges: new Float32Array(ranges),
      intensities: new Float32Array(intensities),
    }
  }

  // Fallback to legacy format: header floats followed by ranges/intensities interleaved.
  const floats = decodeBinaryFloat32(payload)

  const angle_min = floats[0]
  const angle_max = floats[1]
  const angle_increment = floats[2]
  const range_min = floats[3]
  const range_max = floats[4]

  // Remaining floats are ranges, then intensities (if present)
  const dataLength = floats.length - 5
  const rangeCount = Math.floor(dataLength / 2)

  const ranges = floats.slice(5, 5 + rangeCount)
  const intensities = floats.slice(5 + rangeCount)

  return {
    angle_min,
    angle_max,
    angle_increment,
    range_min,
    range_max,
    ranges: new Float32Array(ranges),
    intensities: new Float32Array(intensities),
  }
}

/**
 * Auto-decode payload based on encoding type
 */
export async function decodePayload<T = unknown>(
  payload: Uint8Array,
  encoding: Encoding
): Promise<T> {
  switch (encoding) {
    case Encoding.CBOR:
      return decodeCbor<T>(payload)
    case Encoding.PNG:
      return (await decodePng(payload)) as T
    case Encoding.JPEG:
      return (await decodeJpeg(payload)) as T
    case Encoding.BINARY:
      return decodeBinaryFloat32(payload) as T
    case Encoding.RAW:
      return payload as T
    default:
      console.warn(`Unknown encoding: ${encoding}, returning raw payload`)
      return payload as T
  }
}

/**
 * Auto-decode payload using Web Workers when available
 *
 * This is the recommended method for decoding high-frequency streams
 * like camera images and laser scans. It offloads heavy decoding
 * operations to a worker pool, preventing UI jank.
 *
 * Falls back to main thread decoding if workers are unavailable.
 */
export async function decodePayloadWithWorker<T = unknown>(
  payload: Uint8Array,
  encoding: Encoding,
  workerManager?: import('../workers').DecodeWorkerManager
): Promise<T> {
  // Lazy import to avoid loading worker code when not needed
  const { getDecodeWorkerManager } = await import('../workers')
  const manager = workerManager ?? getDecodeWorkerManager()

  // For encodings that benefit from worker offloading
  if (manager.isReady()) {
    try {
      switch (encoding) {
        case Encoding.PNG:
          return (await manager.decodePng(payload)) as T
        case Encoding.JPEG:
          return (await manager.decodeJpeg(payload)) as T
        case Encoding.BINARY:
          // LaserScan binary format
          return (await manager.decodeLaserScan(payload)) as T
        default:
          // Fall through to main thread for other encodings
          break
      }
    } catch (error) {
      // Fall back to main thread on worker error
      console.warn('Worker decode failed, falling back to main thread:', error)
    }
  }

  // Fallback to main thread decoding
  return decodePayload<T>(payload, encoding)
}
