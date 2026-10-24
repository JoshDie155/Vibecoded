/**
 * OpenCascade als WebAssembly für den CAD-Import – gzip-gepackt im Programm (vite.config.ts: occtWasm).
 * Eigenes Modul, damit es nur im Hauptprogramm steckt und nicht zusätzlich im Hintergrund-Thread; der
 * bekommt die gepackten Bytes beim ersten Aufruf und entpackt sie selbst.
 */
import b64 from 'virtual:occt-wasm';

let gz: Uint8Array | null = null;

/** Gepacktes WebAssembly (gzip). */
export function occtWasmGz(): Uint8Array {
  if (!gz) {
    const bin = atob(b64);
    gz = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) gz[i] = bin.charCodeAt(i);
  }
  return gz;
}
