import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    include: ['test/**/*.test.ts', 'test/**/*.test.mjs'],
    testTimeout: 180_000,
    hookTimeout: 180_000,
    // The CDK synths are CPU-bound; one file at a time keeps the run predictable on small CI runners.
    fileParallelism: false,
  },
});
