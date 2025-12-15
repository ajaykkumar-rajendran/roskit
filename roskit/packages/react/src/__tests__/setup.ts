/**
 * Test setup for React hooks
 */

import '@testing-library/react'

// Mock global fetch if needed
globalThis.fetch = globalThis.fetch || (() => Promise.reject(new Error('fetch not implemented')))
