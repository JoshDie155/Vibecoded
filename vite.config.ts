import { readFileSync } from 'node:fs';
import { gzipSync } from 'node:zlib';
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

/**
 * OpenCascade (occt-import-js) für den CAD-Import: das 7,6 MB große WebAssembly gzip-gepackt als
 * Base64-Text (rund 4 MB). So steckt es auch in der eigenständigen HTML-Datei und funktioniert ohne
 * Internet; entpackt wird erst beim ersten STEP-Import (src/io/step.ts).
 */
const occtWasm: Plugin = {
  name: 'occt-wasm',
  resolveId: (id) => (id === 'virtual:occt-wasm' ? '\0virtual:occt-wasm' : null),
  load(id) {
    if (id !== '\0virtual:occt-wasm') return null;
    const gz = gzipSync(readFileSync('node_modules/occt-import-js/dist/occt-import-js.wasm'), { level: 9 });
    return `export default ${JSON.stringify(gz.toString('base64'))};`;
  },
};

// `--mode single` erzeugt eine einzelne, eigenständige HTML-Datei (dist-single/index.html),
// die man ohne Server einfach im Browser öffnen kann.
export default defineConfig(({ mode }) => ({
  base: './',
  plugins: mode === 'single' ? [manifoldWasmOnce, occtWasm, viteSingleFile()] : [manifoldWasmOnce, occtWasm],
  worker: { plugins: () => [occtWasm] },
  build: {
    outDir: mode === 'single' ? 'dist-single' : 'dist',
    chunkSizeWarningLimit: 2000,
  },
  test: {
    include: ['tests/**/*.test.ts'],
  },
}));
