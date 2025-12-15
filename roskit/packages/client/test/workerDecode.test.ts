import { describe, it, expect, vi } from 'vitest'
import { decodePayloadWithWorker } from '../src/decoders'
import { Encoding } from '../src/protocol'

describe('decodePayloadWithWorker', () => {
  it('uses worker manager when available', async () => {
    const payload = new Uint8Array([1, 2, 3])
    const manager = {
      isReady: () => true,
      decodePng: vi.fn().mockResolvedValue({ width: 1, height: 1, data: new Uint8Array([0, 0, 0, 0]) }),
      decodeJpeg: vi.fn(),
      decodeLaserScan: vi.fn(),
    }

    const result = await decodePayloadWithWorker(payload, Encoding.PNG, manager as any)
    expect(manager.decodePng).toHaveBeenCalledTimes(1)
    expect((result as any).width).toBe(1)
  })

  it('falls back to main thread when worker not ready', async () => {
    const payload = new Uint8Array([0x9f, 0xf4]) // invalid CBOR, but will be treated as raw
    const manager = {
      isReady: () => false,
    }
    const result = await decodePayloadWithWorker(payload, Encoding.RAW, manager as any)
    expect(result).toBe(payload)
  })
})
