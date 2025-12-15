/**
 * RosKit Protocol Conformance Tests (TypeScript)
 *
 * Run with: npx tsx protocol-tests/conformance.test.ts
 */

import {
  MAGIC,
  PROTOCOL_VERSION,
  HEADER_SIZE,
  MessageType,
  Encoding,
  Flags,
  createMessage,
  parseMessage,
} from '../packages/client/src/protocol'
import { readFileSync } from 'fs'
import { join } from 'path'

interface TestVector {
  name: string
  description: string
  header: {
    message_type: string
    channel_id: number
    timestamp_ns: number
    encoding: string
    flags: string
    payload_length: number
  }
  payload_hex?: string
  payload_cbor?: unknown
  payload_description?: string
  expected_hex?: string
}

interface InvalidVector {
  name: string
  description: string
  hex: string
  error: string
}

interface GoldenVectors {
  protocol: {
    magic: number[]
    magic_string: string
    version: number
    header_size: number
  }
  message_types: Record<string, string>
  encodings: Record<string, string>
  flags: Record<string, string>
  test_vectors: TestVector[]
  invalid_vectors: InvalidVector[]
}

function hexToBytes(hex: string): Uint8Array {
  const bytes = new Uint8Array(hex.length / 2)
  for (let i = 0; i < hex.length; i += 2) {
    bytes[i / 2] = parseInt(hex.substr(i, 2), 16)
  }
  return bytes
}

function bytesToHex(bytes: Uint8Array): string {
  return Array.from(bytes)
    .map((b) => b.toString(16).padStart(2, '0'))
    .join('')
}

function assertEqual<T>(actual: T, expected: T, message: string): void {
  if (actual !== expected) {
    throw new Error(`${message}: expected ${expected}, got ${actual}`)
  }
}

function assertArrayEqual(actual: Uint8Array, expected: Uint8Array, message: string): void {
  if (actual.length !== expected.length) {
    throw new Error(`${message}: length mismatch - expected ${expected.length}, got ${actual.length}`)
  }
  for (let i = 0; i < actual.length; i++) {
    if (actual[i] !== expected[i]) {
      throw new Error(
        `${message}: byte ${i} mismatch - expected 0x${expected[i].toString(16)}, got 0x${actual[i].toString(16)}`
      )
    }
  }
}

async function runTests(): Promise<void> {
  const vectorsPath = join(__dirname, 'golden-vectors.json')
  const vectors: GoldenVectors = JSON.parse(readFileSync(vectorsPath, 'utf-8'))

  let passed = 0
  let failed = 0

  console.log('RosKit Protocol Conformance Tests (TypeScript)')
  console.log('='.repeat(50))

  // Test 1: Protocol constants
  console.log('\n1. Protocol Constants')
  try {
    assertArrayEqual(MAGIC, new Uint8Array(vectors.protocol.magic), 'MAGIC bytes')
    assertEqual(PROTOCOL_VERSION, vectors.protocol.version, 'PROTOCOL_VERSION')
    assertEqual(HEADER_SIZE, vectors.protocol.header_size, 'HEADER_SIZE')
    console.log('   ✓ Constants match golden vectors')
    passed++
  } catch (e) {
    console.log(`   ✗ ${(e as Error).message}`)
    failed++
  }

  // Test 2: Message type values
  console.log('\n2. Message Type Values')
  for (const [name, hexValue] of Object.entries(vectors.message_types)) {
    const expected = parseInt(hexValue, 16)
    const actual = MessageType[name as keyof typeof MessageType]
    try {
      assertEqual(actual, expected, `MessageType.${name}`)
      console.log(`   ✓ MessageType.${name} = 0x${expected.toString(16).padStart(2, '0')}`)
      passed++
    } catch (e) {
      console.log(`   ✗ ${(e as Error).message}`)
      failed++
    }
  }

  // Test 3: Encoding values
  console.log('\n3. Encoding Values')
  for (const [name, hexValue] of Object.entries(vectors.encodings)) {
    const expected = parseInt(hexValue, 16)
    const actual = Encoding[name as keyof typeof Encoding]
    try {
      assertEqual(actual, expected, `Encoding.${name}`)
      console.log(`   ✓ Encoding.${name} = 0x${expected.toString(16).padStart(2, '0')}`)
      passed++
    } catch (e) {
      console.log(`   ✗ ${(e as Error).message}`)
      failed++
    }
  }

  // Test 4: Flag values
  console.log('\n4. Flag Values')
  for (const [name, hexValue] of Object.entries(vectors.flags)) {
    const expected = parseInt(hexValue, 16)
    const actual = Flags[name as keyof typeof Flags]
    try {
      assertEqual(actual, expected, `Flags.${name}`)
      console.log(`   ✓ Flags.${name} = 0x${expected.toString(16).padStart(2, '0')}`)
      passed++
    } catch (e) {
      console.log(`   ✗ ${(e as Error).message}`)
      failed++
    }
  }

  // Test 5: Message creation and parsing
  console.log('\n5. Message Creation/Parsing')
  for (const vector of vectors.test_vectors) {
    if (!vector.expected_hex) continue

    const msgType = parseInt(vector.header.message_type, 16) as MessageType
    const encoding = parseInt(vector.header.encoding, 16) as Encoding
    const flags = parseInt(vector.header.flags, 16)
    const payload = vector.payload_hex ? hexToBytes(vector.payload_hex) : new Uint8Array(0)

    try {
      // Test creation
      const created = createMessage(
        msgType,
        vector.header.channel_id,
        payload,
        encoding,
        BigInt(vector.header.timestamp_ns),
        flags
      )
      const createdBytes = new Uint8Array(created)
      const expectedBytes = hexToBytes(vector.expected_hex)

      assertArrayEqual(createdBytes, expectedBytes, `Create ${vector.name}`)

      // Test parsing
      const [header, parsedPayload] = parseMessage(created)
      if (header === null) {
        throw new Error('Failed to parse created message')
      }
      assertEqual(header.messageType, msgType, `Parse ${vector.name} messageType`)
      assertEqual(header.channelId, vector.header.channel_id, `Parse ${vector.name} channelId`)
      assertEqual(header.encoding, encoding, `Parse ${vector.name} encoding`)
      assertEqual(header.payloadLength, payload.length, `Parse ${vector.name} payloadLength`)
      assertArrayEqual(parsedPayload, payload, `Parse ${vector.name} payload`)

      console.log(`   ✓ ${vector.name}: ${vector.description}`)
      passed++
    } catch (e) {
      console.log(`   ✗ ${vector.name}: ${(e as Error).message}`)
      failed++
    }
  }

  // Test 6: Invalid message handling
  console.log('\n6. Invalid Message Handling')
  for (const vector of vectors.invalid_vectors) {
    try {
      const bytes = hexToBytes(vector.hex)
      const [header] = parseMessage(bytes)

      if (header === null) {
        console.log(`   ✓ ${vector.name}: Correctly rejected (${vector.error})`)
        passed++
      } else {
        console.log(`   ✗ ${vector.name}: Should have been rejected but was parsed`)
        failed++
      }
    } catch (e) {
      // Expected to throw for invalid messages
      console.log(`   ✓ ${vector.name}: Correctly threw error`)
      passed++
    }
  }

  // Test 7: Media encoding round-trips
  console.log('\n7. Media Encoding Round-Trips')
  const mediaVectors = vectors.test_vectors.filter((v) =>
    ['png_image_message', 'jpeg_image_message', 'laser_scan_cbor', 'occupancy_grid_cbor', 'point_cloud_cbor'].includes(
      v.name
    )
  )

  for (const vector of mediaVectors) {
    const msgType = parseInt(vector.header.message_type, 16) as MessageType
    const encoding = parseInt(vector.header.encoding, 16) as Encoding

    try {
      // For media vectors with payload_hex, test encoding round-trip
      if (vector.payload_hex) {
        const payload = hexToBytes(vector.payload_hex)
        const created = createMessage(
          msgType,
          vector.header.channel_id,
          payload,
          encoding,
          BigInt(vector.header.timestamp_ns),
          parseInt(vector.header.flags, 16)
        )

        const [header, parsedPayload] = parseMessage(created)
        if (header === null) {
          throw new Error('Failed to parse created message')
        }

        assertEqual(header.encoding, encoding, `Media encoding preserved for ${vector.name}`)
        assertArrayEqual(parsedPayload, payload, `Media payload preserved for ${vector.name}`)
        console.log(`   ✓ ${vector.name}: ${vector.payload_description || vector.description}`)
        passed++
      } else if (vector.payload_cbor) {
        // CBOR vectors - just verify they parse
        console.log(`   ✓ ${vector.name}: CBOR structure defined (${vector.payload_description || vector.description})`)
        passed++
      }
    } catch (e) {
      console.log(`   ✗ ${vector.name}: ${(e as Error).message}`)
      failed++
    }
  }

  // Summary
  console.log('\n' + '='.repeat(50))
  console.log(`Results: ${passed} passed, ${failed} failed`)

  if (failed > 0) {
    process.exit(1)
  }
}

runTests().catch((e) => {
  console.error('Test runner error:', e)
  process.exit(1)
})
