import { defineConfig } from 'vitest/config';
import { viteSingleFile } from 'vite-plugin-singlefile';
import type { Plugin } from 'vite';

/**
 * manifold-3d verweist selbst auf seine WASM-Datei (new URL(…, import.meta.url)). PartJoin übergibt
 * die Datei selbst (src/io/union.ts); ohne diesen Schritt landete sie doppelt im Build.
 */
const manifoldWasmOnce: Plugin = {
  name: 'manifold-wasm-once',
  enforce: 'pre',
  transform(code, id) {
    if (!/manifold-3d[\\/]manifold\.js$/.test(id)) return null;
    return code.replace('new URL("manifold.wasm",import.meta.url)', 'new URL("manifold.wasm",location.href)');
  },
};

// `--mode single` erzeugt eine einzelne, eigenständige HTML-Datei (dist-single/index.html),
// die man ohne Server einfach im Browser öffnen kann.
export default defineConfig(({ mode }) => ({
  base: './',
  plugins: mode === 'single' ? [manifoldWasmOnce, viteSingleFile()] : [manifoldWasmOnce],
  build: {
    outDir: mode === 'single' ? 'dist-single' : 'dist',
    chunkSizeWarningLimit: 2000,
  },
  test: {
    include: ['tests/**/*.test.ts'],
  },
}));
