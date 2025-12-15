import { defineConfig } from 'vitest/config'

export default defineConfig({
  test: {
    globals: true,
    // Use happy-dom to support canvas/ImageBitmap for media round-trips
    environment: 'happy-dom',
    include: ['src/**/*.test.ts', 'test/**/*.test.ts'],
    coverage: {
      provider: 'v8',
      reporter: ['text', 'json', 'html'],
      include: ['src/**/*.ts'],
      exclude: ['src/**/*.test.ts', 'src/__tests__/**'],
    },
  },
})
