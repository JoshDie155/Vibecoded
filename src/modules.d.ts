/** OpenCascade-WebAssembly, gzip-gepackt und Base64-kodiert (vite.config.ts: occtWasm). */
declare module 'virtual:occt-wasm' {
  const base64: string;
  export default base64;
}

/** occt-import-js (Emscripten-Modul, ohne eigene Typen). */
declare module 'occt-import-js' {
  const init: (overrides?: { wasmBinary?: Uint8Array; locateFile?: (path: string) => string }) => Promise<{
    ReadFile(format: string, content: Uint8Array, params: Record<string, unknown> | null): unknown;
  }>;
  export default init;
}
