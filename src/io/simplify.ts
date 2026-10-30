/**
 * Netz vereinfachen (manifold-3d): weniger Dreiecke, die Oberfläche weicht dabei höchstens um die
 * Toleranz ab – etwa für Scans oder fein modellierte Figuren, die D3DForge und den Slicer bremsen.
 *
 * Gruppen (je Dreieck eine Nummer) halten Grenzen fest: Kanten zwischen verschiedenen Gruppen bleiben, wo
 * sie sind, ein Dreieck mit eigener Gruppe bleibt unverändert. So kommen Bemalung und gewählte Flächen
 * genau mit.
 */
import type { Manifold, ManifoldToplevel } from 'manifold-3d';
import { indexed, loadManifold } from './union';

export interface SimplifyResult {
  /** Nicht indizierte Dreiecke des vereinfachten Netzes. */
  positions: Float32Array;
  /** Je Dreieck ein Ausgangsdreieck derselben Gruppe (−1 ohne Gruppen). */
  source: Int32Array;
  before: number;
  after: number;
}

export class SimplifyError extends Error {
  name = 'SimplifyError';
}

function solid(wasm: ManifoldToplevel, positions: Float32Array, groups?: Uint32Array): Manifold {
  const { verts, tris } = indexed(positions);
  const mesh = new wasm.Mesh({ numProp: 3, vertProperties: verts, triVerts: tris, ...(groups ? { faceID: groups } : {}) });
  mesh.merge();
  let m: Manifold | null = null;
  try {
    m = new wasm.Manifold(mesh);
  } catch { /* nicht geschlossen */ }
  if (!m || m.isEmpty()) {
    m?.delete();
    throw new SimplifyError('Das Teil ist kein geschlossener Körper (nicht wasserdicht) – erst „Netz reparieren“.');
  }
  return m;
}

export async function simplifyMesh(positions: Float32Array, tolerance: number, groups?: Uint32Array, wasmUrl?: string): Promise<SimplifyResult> {
  const wasm = await loadManifold(wasmUrl);
  const m = solid(wasm, positions, groups);
  const s = m.simplify(tolerance);
  try {
    const res = s.getMesh();
    const np = res.numProp;
    const n = res.triVerts.length / 3;
    const out = new Float32Array(n * 9);
    for (let i = 0; i < res.triVerts.length; i++) {
      const v = res.triVerts[i] * np;
      out[i * 3] = res.vertProperties[v];
      out[i * 3 + 1] = res.vertProperties[v + 1];
      out[i * 3 + 2] = res.vertProperties[v + 2];
    }
    const source = new Int32Array(n).fill(-1);
    if (groups) {
      // Erstes Dreieck je Gruppe als Stellvertreter
      const first = new Map<number, number>();
      groups.forEach((g, t) => { if (!first.has(g)) first.set(g, t); });
      for (let t = 0; t < n; t++) source[t] = first.get(res.faceID[t]) ?? -1;
    }
    return { positions: out, source, before: positions.length / 9, after: n };
  } finally {
    s.delete();
    m.delete();
  }
}

/** Dreieckszahl nach dem Vereinfachen je Toleranz (für die Auswahl). */
export async function simplifyCounts(positions: Float32Array, tolerances: number[], groups?: Uint32Array, wasmUrl?: string): Promise<number[]> {
  const wasm = await loadManifold(wasmUrl);
  const m = solid(wasm, positions, groups);
  try {
    return tolerances.map((t) => {
      const s = m.simplify(t);
      const n = s.numTri();
      s.delete();
      return n;
    });
  } finally {
    m.delete();
  }
}
