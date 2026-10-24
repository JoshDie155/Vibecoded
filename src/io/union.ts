/**
 * Echte Vereinigung mehrerer Teile zu einem geschlossenen Körper (boolesche Vereinigung mit
 * manifold-3d). Überlappungen und innenliegende Flächen verschwinden, der Slicer bekommt ein
 * einziges wasserdichtes Netz. Teile, die selbst nicht geschlossen sind, lassen sich nicht vereinigen:
 * Sie werden unverändert angehängt und gemeldet.
 *
 * Die Bibliothek (WebAssembly, ~0,5 MB) wird erst beim ersten Gebrauch geladen.
 */
import type { Manifold, ManifoldToplevel } from 'manifold-3d';

export interface UnionInput {
  name: string;
  /** Nicht indizierte Dreiecke in Weltkoordinaten. */
  positions: Float32Array;
}

export interface UnionResult {
  /** Nicht indizierte Dreiecke des Ergebnisses. */
  positions: Float32Array;
  /** Namen der Teile, die nicht geschlossen sind und deshalb nur angehängt wurden. */
  notClosed: string[];
}

let modulePromise: Promise<ManifoldToplevel> | null = null;

/** Lädt manifold-3d; die WASM-Datei wird ohne Netzwerkzugriff übergeben, wenn sie eingebettet ist. */
export function loadManifold(wasmUrl?: string): Promise<ManifoldToplevel> {
  modulePromise ??= (async () => {
    const { default: Module } = await import('manifold-3d');
    const config: Record<string, unknown> = {};
    if (wasmUrl?.startsWith('data:')) {
      const b64 = wasmUrl.slice(wasmUrl.indexOf(',') + 1);
      const bin = atob(b64);
      const bytes = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
      config.wasmBinary = bytes;
      // Verhindert, dass das Modul selbst nach der Datei sucht
      config.locateFile = () => 'manifold.wasm';
    } else if (wasmUrl) {
      config.locateFile = () => wasmUrl;
    }
    const wasm = await (Module as unknown as (c: Record<string, unknown>) => Promise<ManifoldToplevel>)(config);
    wasm.setup();
    return wasm;
  })();
  modulePromise.catch(() => (modulePromise = null));
  return modulePromise;
}

/** Gleiche Eckpunkte (bitgenau) zusammenfassen – manifold braucht ein indiziertes Netz. */
export function indexed(p: Float32Array): { verts: Float32Array; tris: Uint32Array } {
  const map = new Map<string, number>();
  const verts: number[] = [];
  const tris = new Uint32Array(p.length / 3);
  for (let i = 0; i < p.length; i += 3) {
    const key = `${p[i]},${p[i + 1]},${p[i + 2]}`;
    let v = map.get(key);
    if (v === undefined) {
      v = verts.length / 3;
      map.set(key, v);
      verts.push(p[i], p[i + 1], p[i + 2]);
    }
    tris[i / 3] = v;
  }
  return { verts: Float32Array.from(verts), tris };
}

/**
 * Geschlossener Körper aus nicht indizierten Dreiecken – oder null, wenn das Netz nicht wasserdicht ist.
 * Jedes Dreieck trägt seine Nummer (faceID): So lässt sich nach Schnitten und Bohrungen nachsehen, aus
 * welchem Dreieck ein Ergebnisdreieck stammt (siehe trackedPositions).
 */
const originals = new WeakMap<Manifold, number>();
/** Kennung des Ausgangskörpers für trackedPositions (−1, wenn nicht mit solidOf gebaut). */
export const originalIdOf = (m: Manifold) => originals.get(m) ?? -1;

export function solidOf(wasm: ManifoldToplevel, positions: Float32Array): Manifold | null {
  const { verts, tris } = indexed(positions);
  const faceID = new Uint32Array(tris.length / 3).map((_, i) => i);
  const id = wasm.Manifold.reserveIDs(1);
  const mesh = new wasm.Mesh({
    numProp: 3, vertProperties: verts, triVerts: tris, faceID,
    runIndex: new Uint32Array([0, tris.length]), runOriginalID: new Uint32Array([id]),
  });
  // Fast deckungsgleiche Ränder (Nähte) schließen
  mesh.merge();
  try {
    const m = new wasm.Manifold(mesh);
    if (!m.isEmpty()) {
      originals.set(m, id);
      return m;
    }
    m.delete();
  } catch { /* nicht geschlossen */ }
  return null;
}

/**
 * Nicht indizierte Dreiecke eines Ergebnisses und je Dreieck die Nummer des Dreiecks im Ausgangskörper
 * (originalID), aus dem es stammt – −1 für neue Flächen (Bohrungen, Schnittflächen, Zapfen).
 */
export function trackedPositions(m: Manifold, originalID: number): { positions: Float32Array; source: Int32Array } {
  const res = m.getMesh();
  const np = res.numProp;
  const n = res.triVerts.length / 3;
  const positions = new Float32Array(n * 9);
  for (let i = 0; i < res.triVerts.length; i++) {
    const v = res.triVerts[i] * np;
    positions[i * 3] = res.vertProperties[v];
    positions[i * 3 + 1] = res.vertProperties[v + 1];
    positions[i * 3 + 2] = res.vertProperties[v + 2];
  }
  const source = new Int32Array(n).fill(-1);
  const runs = res.runOriginalID ?? new Uint32Array(0);
  for (let r = 0; r < runs.length; r++) {
    if (runs[r] !== originalID) continue;
    for (let t = res.runIndex[r] / 3; t < res.runIndex[r + 1] / 3; t++) source[t] = res.faceID[t];
  }
  return { positions, source };
}

/** Nicht indizierte Dreiecke eines Körpers. */
export function positionsOfSolid(m: Manifold): Float32Array {
  const res = m.getMesh();
  const np = res.numProp;
  const out = new Float32Array(res.triVerts.length * 3);
  for (let i = 0; i < res.triVerts.length; i++) {
    const v = res.triVerts[i] * np;
    out[i * 3] = res.vertProperties[v];
    out[i * 3 + 1] = res.vertProperties[v + 1];
    out[i * 3 + 2] = res.vertProperties[v + 2];
  }
  return out;
}

/**
 * Vereinigt alle Teile zu einem Körper.
 * @param wasmUrl Adresse der manifold.wasm (im Browser aus dem Build), in Node nicht nötig
 */
export async function unionParts(parts: UnionInput[], wasmUrl?: string): Promise<UnionResult> {
  const wasm = await loadManifold(wasmUrl);
  const { Manifold } = wasm;
  const solids: Manifold[] = [];
  const loose: Float32Array[] = [];
  const notClosed: string[] = [];
  for (const part of parts) {
    const m = solidOf(wasm, part.positions);
    if (m) solids.push(m);
    else {
      notClosed.push(part.name);
      loose.push(part.positions);
    }
  }
  let out: Float32Array = new Float32Array(0);
  if (solids.length) {
    const u = solids.length === 1 ? solids[0] : Manifold.union(solids);
    out = positionsOfSolid(u);
    if (u !== solids[0]) u.delete();
    for (const m of solids) m.delete();
  }
  const total = out.length + loose.reduce((s, p) => s + p.length, 0);
  const positions = new Float32Array(total);
  positions.set(out, 0);
  let o = out.length;
  for (const p of loose) {
    positions.set(p, o);
    o += p.length;
  }
  return { positions, notClosed };
}
