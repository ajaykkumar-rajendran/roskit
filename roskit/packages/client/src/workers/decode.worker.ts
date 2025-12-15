/**
 * Decode Worker - Offload heavy image/scan decoding to a Web Worker
 *
 * This worker handles CPU-intensive decoding operations off the main thread
 * to prevent UI jank during high-frequency data streams.
 */

import { decode as cborDecode } from 'cbor-x'

// Message types for communication with main thread
export interface DecodeRequest {
  id: number
  type: 'png' | 'jpeg' | 'laser_scan' | 'cbor'
  payload: ArrayBuffer
}

export interface DecodeResponse {
  id: number
  success: boolean
  result?: unknown
  error?: string
  transferables?: ArrayBuffer[]
}

/**
 * Decode CBOR payload
 */
function decodeCbor<T = unknown>(payload: Uint8Array): T {
  return cborDecode(payload) as T
}

/**
 * Decode PNG image to pixel data
 */
async function decodePng(payload: Uint8Array): Promise<{
  width: number
  height: number
  data: ArrayBuffer
  metadata?: unknown
}> {
  let pngStart = 0
  let metadata: unknown

  if (payload.byteLength > 8) {
    const metaLength = new DataView(payload.buffer, payload.byteOffset, payload.byteLength).getUint32(
      0,
      false
    )
    if (metaLength > 0 && metaLength + 4 < payload.byteLength) {
      const metaBytes = payload.slice(4, 4 + metaLength)
      metadata = decodeCbor(metaBytes)
      pngStart = 4 + metaLength
    }
  }

  const pngBytes = payload.slice(pngStart)
  const blob = new Blob([pngBytes], { type: 'image/png' })
  const imageBitmap = await createImageBitmap(blob)

  const canvas = new OffscreenCanvas(imageBitmap.width, imageBitmap.height)
  const ctx = canvas.getContext('2d')!
  ctx.drawImage(imageBitmap, 0, 0)

  const imageData = ctx.getImageData(0, 0, imageBitmap.width, imageBitmap.height)

  return {
    width: imageBitmap.width,
    height: imageBitmap.height,
    data: imageData.data.buffer,
    metadata,
  }
}

/**
 * Decode JPEG image to pixel data
 */
async function decodeJpeg(payload: Uint8Array): Promise<{
  width: number
  height: number
  data: ArrayBuffer
  metadata?: unknown
}> {
  let jpegStart = 0
  let metadata: unknown

  if (payload.byteLength > 8) {
    const metaLength = new DataView(payload.buffer, payload.byteOffset, payload.byteLength).getUint32(
      0,
      false
    )
    if (metaLength > 0 && metaLength + 4 < payload.byteLength) {
      const metaBytes = payload.slice(4, 4 + metaLength)
      metadata = decodeCbor(metaBytes)
      jpegStart = 4 + metaLength
    }
  }

  const jpegBytes = payload.slice(jpegStart)
  const blob = new Blob([jpegBytes], { type: 'image/jpeg' })
  const imageBitmap = await createImageBitmap(blob)

  const canvas = new OffscreenCanvas(imageBitmap.width, imageBitmap.height)
  const ctx = canvas.getContext('2d')!
  ctx.drawImage(imageBitmap, 0, 0)

  const imageData = ctx.getImageData(0, 0, imageBitmap.width, imageBitmap.height)

  return {
    width: imageBitmap.width,
    height: imageBitmap.height,
    data: imageData.data.buffer,
    metadata,
  }
}

/**
 * Decode LaserScan binary format
 */
function decodeLaserScan(payload: Uint8Array): {
  angle_min: number
  angle_max: number
  angle_increment: number
  range_min: number
  range_max: number
  ranges: ArrayBuffer
  intensities: ArrayBuffer
} {
  const view = new DataView(payload.buffer, payload.byteOffset, payload.byteLength)
  const metaLength = payload.byteLength >= 4 ? view.getUint32(0, false) : 0
  const hasMetadata = metaLength > 0 && metaLength < payload.byteLength - 4

  if (hasMetadata) {
    const metaStart = 4
    const metaEnd = metaStart + metaLength
    const metadata = decodeCbor<{
      angle_min?: number
      angle_max?: number
      angle_increment?: number
      range_min?: number
      range_max?: number
      ranges_count?: number
    }>(payload.slice(metaStart, metaEnd))

    const dataStart = metaEnd
    const floatCount = Math.floor((payload.byteLength - dataStart) / 4)
    const floats = new Float32Array(payload.buffer, payload.byteOffset + dataStart, floatCount)

    const rangesCount = metadata?.ranges_count ?? Math.floor(floatCount / 2)
    const ranges = floats.slice(0, rangesCount)
    const intensities = floats.slice(rangesCount)

    return {
      angle_min: metadata?.angle_min ?? 0,
      angle_max: metadata?.angle_max ?? 0,
      angle_increment: metadata?.angle_increment ?? 0,
      range_min: metadata?.range_min ?? 0,
      range_max: metadata?.range_max ?? 0,
      ranges: ranges.buffer.slice(ranges.byteOffset, ranges.byteOffset + ranges.byteLength),
      intensities: intensities.buffer.slice(
        intensities.byteOffset,
        intensities.byteOffset + intensities.byteLength
      ),
    }
  }

  // Legacy format fallback
  const floats = new Float32Array(payload.buffer, payload.byteOffset, Math.floor(payload.byteLength / 4))

  const dataLength = floats.length - 5
  const rangeCount = Math.floor(dataLength / 2)

  const ranges = floats.slice(5, 5 + rangeCount)
  const intensities = floats.slice(5 + rangeCount)

  return {
    angle_min: floats[0],
    angle_max: floats[1],
    angle_increment: floats[2],
    range_min: floats[3],
    range_max: floats[4],
    ranges: ranges.buffer.slice(ranges.byteOffset, ranges.byteOffset + ranges.byteLength),
    intensities: intensities.buffer.slice(
      intensities.byteOffset,
      intensities.byteOffset + intensities.byteLength
    ),
  }
}

/**
 * Handle incoming decode requests
 */
self.onmessage = async (event: MessageEvent<DecodeRequest>) => {
  const { id, type, payload } = event.data
  const uint8Payload = new Uint8Array(payload)

  try {
    let result: unknown
    let transferables: ArrayBuffer[] = []

    switch (type) {
      case 'png': {
        const decoded = await decodePng(uint8Payload)
        result = {
          width: decoded.width,
          height: decoded.height,
          data: decoded.data,
          metadata: decoded.metadata,
        }
        transferables = [decoded.data]
        break
      }
      case 'jpeg': {
        const decoded = await decodeJpeg(uint8Payload)
        result = {
          width: decoded.width,
          height: decoded.height,
          data: decoded.data,
          metadata: decoded.metadata,
        }
        transferables = [decoded.data]
        break
      }
      case 'laser_scan': {
        const decoded = decodeLaserScan(uint8Payload)
        result = {
          angle_min: decoded.angle_min,
          angle_max: decoded.angle_max,
          angle_increment: decoded.angle_increment,
          range_min: decoded.range_min,
          range_max: decoded.range_max,
          ranges: decoded.ranges,
          intensities: decoded.intensities,
        }
        transferables = [decoded.ranges, decoded.intensities]
        break
      }
      case 'cbor': {
        result = decodeCbor(uint8Payload)
        break
      }
      default:
        throw new Error(`Unknown decode type: ${type}`)
    }

    const response: DecodeResponse = {
      id,
      success: true,
      result,
      transferables,
    }

    self.postMessage(response, { transfer: transferables })
  } catch (error) {
    const response: DecodeResponse = {
      id,
      success: false,
      error: error instanceof Error ? error.message : String(error),
    }

    self.postMessage(response)
  }
}

// Signal that worker is ready
self.postMessage({ type: 'ready' })
