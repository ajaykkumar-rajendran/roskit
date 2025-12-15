import { describe, it, expect } from 'vitest'
import { decodePayloadWithWorker } from '../src/decoders'
import { Encoding } from '../src/protocol'
import { encode as cborEncode } from 'cbor-x'

async function createPng(width: number, height: number, color: [number, number, number, number]) {
  const canvas = new OffscreenCanvas(width, height)
  const ctx = canvas.getContext('2d')
  if (!ctx) throw new Error('No 2d context')
  const imageData = ctx.createImageData(width, height)
  for (let i = 0; i < width * height; i++) {
    const idx = i * 4
    imageData.data[idx] = color[0]
    imageData.data[idx + 1] = color[1]
    imageData.data[idx + 2] = color[2]
    imageData.data[idx + 3] = color[3]
  }
  ctx.putImageData(imageData, 0, 0)
  const blob = await canvas.convertToBlob({ type: 'image/png' })
  const buffer = new Uint8Array(await blob.arrayBuffer())
  return buffer
}

async function createJpeg(width: number, height: number, color: [number, number, number, number]) {
  const canvas = new OffscreenCanvas(width, height)
  const ctx = canvas.getContext('2d')
  if (!ctx) throw new Error('No 2d context')
  ctx.fillStyle = `rgba(${color[0]}, ${color[1]}, ${color[2]}, ${color[3] / 255})`
  ctx.fillRect(0, 0, width, height)
  const blob = await canvas.convertToBlob({ type: 'image/jpeg', quality: 0.8 })
  const buffer = new Uint8Array(await blob.arrayBuffer())
  return buffer
}

describe('Media round-trip (happy-dom)', () => {
  it('decodes PNG via worker path', async () => {
    const png = await createPng(4, 4, [255, 0, 0, 255])
    const decoded = await decodePayloadWithWorker<{ width: number; height: number; data: Uint8Array }>(
      png,
      Encoding.PNG
    )
    expect(decoded.width).toBe(4)
    expect(decoded.height).toBe(4)
    // first pixel should be red
    expect(decoded.data[0]).toBe(255)
    expect(decoded.data[1]).toBe(0)
    expect(decoded.data[2]).toBe(0)
    expect(decoded.data[3]).toBe(255)
  })

  it('decodes JPEG via worker path', async () => {
    const jpg = await createJpeg(4, 4, [0, 255, 0, 255])
    const decoded = await decodePayloadWithWorker<{ width: number; height: number; data: Uint8Array }>(
      jpg,
      Encoding.JPEG
    )
    expect(decoded.width).toBe(4)
    expect(decoded.height).toBe(4)
  })

  it('decodes OccupancyGrid-style PNG with metadata prefix', async () => {
    const png = await createPng(2, 2, [128, 128, 128, 255])
    const metadata = {
      width: 2,
      height: 2,
      resolution: 0.05,
      origin: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
    }
    const metaBytes = new Uint8Array(cborEncode(metadata))
    const payload = new Uint8Array(4 + metaBytes.byteLength + png.byteLength)
    const view = new DataView(payload.buffer)
    view.setUint32(0, metaBytes.byteLength, false)
    payload.set(metaBytes, 4)
    payload.set(png, 4 + metaBytes.byteLength)

    const decoded = await decodePayloadWithWorker<{
      width: number
      height: number
      data: Uint8Array
      metadata?: unknown
    }>(payload, Encoding.PNG)

    expect(decoded.width).toBe(2)
    expect(decoded.height).toBe(2)
    expect(decoded.metadata && (decoded.metadata as any).width).toBe(2)
    expect(decoded.data[3]).toBe(255)
  })
})
