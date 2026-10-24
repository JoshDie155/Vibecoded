/**
 * Aussparungen an einer angeklickten Stelle (boolesche Differenz mit manifold-3d): Tasche für einen
 * Magneten, Loch für einen Gewindeeinsatz zum Einschmelzen, Schraubenloch mit Kopfsenkung (Zylinder- oder
 * Senkkopf), Sechskant-Tasche für eine Mutter oder eine freie Bohrung. Die Aussparung zeigt senkrecht in
 * die Fläche hinein.
 */
import type { Manifold, ManifoldToplevel, Mat4 } from 'manifold-3d';
import { type Vec3, cross, normalize } from '../math/align';
import { rayDistance } from './cut';
import { loadManifold, originalIdOf, positionsOfSolid, solidOf, trackedPositions } from './union';

export type PocketKind = 'magnet' | 'insert' | 'screw' | 'nut' | 'hole';

export interface PocketSpec {
  kind: PocketKind;
  /** Magnet, Einsatz, Bohrung: ⌀ · Schraube: Durchgangsloch · Mutter: Schlüsselweite (mm). */
  diameter: number;
  /** Magnet, Einsatz, Bohrung: Tiefe · Schraube: Tiefe der Kopfsenkung · Mutter: Höhe (mm). */
  depth: number;
  /** Spiel je Seite (mm). */
  clearance?: number;
  /** Schraube: Kopf-⌀. */
  head?: number;
  /** Schraube mit Senkkopf (90°). */
  countersunk?: boolean;
  /** Bohrung ganz durch, bei Schraube und Mutter: Schraubenloch durch das Teil. */
  through?: boolean;
  /** Mutter: Schraubenloch-⌀. */
  bolt?: number;
}

/** Ort: Punkt auf der Oberfläche und Außennormale dort (Koordinaten des Netzes). */
export interface PocketPlace {
  point: Vec3;
  normal: Vec3;
}

export interface Preset {
  label: string;
  spec: Omit<PocketSpec, 'kind'>;
}

/**
 * Gängige Maße. Einsätze: typische Werte für Gewindeeinsätze zum Einschmelzen (z. B. Ruthex, Standard-
 * länge) – im Zweifel mit dem Datenblatt abgleichen. Schrauben: Durchgangsloch mittel (ISO 273), Köpfe
 * nach ISO 4762 / ISO 10642 mit Zugabe. Muttern: ISO 4032.
 */
export const PRESETS: Record<PocketKind, Preset[]> = {
  magnet: [[5, 2], [6, 2], [6, 3], [8, 2], [8, 3], [10, 2], [10, 3], [12, 3], [15, 3], [20, 3]]
    .map(([d, h]) => ({ label: `${d} × ${h} mm`, spec: { diameter: d, depth: h + 0.2, clearance: 0.1 } })),
  insert: [['M2', 3.2, 4], ['M2.5', 3.6, 4.5], ['M3', 4, 5.7], ['M4', 5.6, 8.1], ['M5', 6.4, 9.5]]
    .map(([m, d, l]) => ({ label: `${m} (⌀ ${String(d).replace('.', ',')})`, spec: { diameter: d as number, depth: (l as number) + 1, clearance: 0 } })),
  screw: [['M2', 2.4, 3.8, 2], ['M2.5', 2.9, 4.5, 2.5], ['M3', 3.4, 5.5, 3], ['M4', 4.5, 7, 4], ['M5', 5.5, 8.5, 5], ['M6', 6.6, 10, 6]]
    .map(([m, d, dk, k]) => ({ label: m as string, spec: { diameter: d as number, head: (dk as number) + 0.5, depth: (k as number) + 0.3, through: true, clearance: 0 } })),
  nut: [['M2', 4, 1.6, 2.4], ['M2.5', 5, 2, 2.9], ['M3', 5.5, 2.4, 3.4], ['M4', 7, 3.2, 4.5], ['M5', 8, 4.7, 5.5], ['M6', 10, 5.2, 6.6]]
    .map(([m, s, h, b]) => ({ label: m as string, spec: { diameter: s as number, depth: (h as number) + 0.2, bolt: b as number, through: true, clearance: 0.15 } })),
  hole: [3, 4, 5, 6, 8, 10].map((d) => ({ label: `⌀ ${d}`, spec: { diameter: d, depth: 10, clearance: 0 } })),
};

/** Senkkopf-⌀ (ISO 10642, gerundet) zum Durchgangsloch der Schraube. */
export const COUNTERSUNK_HEAD: Record<string, number> = { M2: 4.4, 'M2.5': 5.5, M3: 6.5, M4: 8.6, M5: 10.4, M6: 12.4 };

const SEGMENTS = 48;

/** Werkzeug-Koordinaten: z zeigt von der Oberfläche ins Teil, Ursprung am Punkt. */
function frame(place: PocketPlace): Mat4 {
  const z = normalize(place.normal).map((v) => -v) as Vec3;
  const u = normalize(cross(z, Math.abs(z[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0]));
  const v = cross(z, u);
  const p = place.point;
  return [u[0], u[1], u[2], 0, v[0], v[1], v[2], 0, z[0], z[1], z[2], 0, p[0], p[1], p[2], 1];
}

/**
 * Körper, der abgezogen wird. thickness: Materialstärke hinter dem Punkt (für „durch“).
 * Oben ragt er 0,5 mm aus der Fläche, damit sie sauber aufgeht.
 */
export function pocketTool(wasm: ManifoldToplevel, place: PocketPlace, spec: PocketSpec, thickness: number, keep: <T extends { delete(): void }>(x: T) => T): Manifold {
  const { Manifold } = wasm;
  const c = Math.max(0, spec.clearance ?? 0);
  const out = 0.5;
  const through = (Number.isFinite(thickness) ? thickness : 1000) + 1;
  const cyl = (r: number, z0: number, z1: number, segs = SEGMENTS) => keep(keep(Manifold.cylinder(z1 - z0, r, r, segs, false)).translate([0, 0, z0]));
  const parts: Manifold[] = [];
  switch (spec.kind) {
    case 'magnet':
      parts.push(cyl(spec.diameter / 2 + c, -out, spec.depth));
      break;
    case 'insert':
      parts.push(cyl(spec.diameter / 2 + c, -out, spec.depth));
      break;
    case 'hole':
      parts.push(cyl(spec.diameter / 2 + c, -out, spec.through ? through : spec.depth));
      break;
    case 'screw': {
      const head = (spec.head ?? spec.diameter * 1.8) / 2 + c;
      const r = spec.diameter / 2 + c;
      parts.push(cyl(r, -out, spec.through === false ? spec.depth + spec.diameter * 3 : through));
      if (spec.countersunk) {
        // 90°-Kegel: von Kopf-⌀ an der Oberfläche auf das Loch
        const h = head - r;
        parts.push(cyl(head, -out, 0), keep(keep(Manifold.cylinder(h, head, r, SEGMENTS, false)).translate([0, 0, 0])));
      } else parts.push(cyl(head, -out, spec.depth));
      break;
    }
    case 'nut': {
      // Sechskant: Umkreis = Schlüsselweite / √3
      const R = (spec.diameter + 2 * c) / Math.sqrt(3);
      parts.push(cyl(R, -out, spec.depth, 6));
      if (spec.through && spec.bolt) parts.push(cyl(spec.bolt / 2, -out, through));
      break;
    }
  }
  return keep(keep(Manifold.union(parts)).transform(frame(place)));
}

/** Materialstärke hinter dem Punkt (entlang der Innenrichtung). */
export const thicknessAt = (positions: Float32Array, place: PocketPlace) =>
  rayDistance(positions, place.point.map((v, k) => v - normalize(place.normal)[k] * 1e-4) as Vec3, normalize(place.normal).map((v) => -v) as Vec3);

export interface PocketResult {
  positions: Float32Array;
  /** Je Dreieck: Herkunft im alten Netz (−1 = Wand der Aussparung). */
  source: Int32Array;
  notes: string[];
}

export class PocketError extends Error {}

export async function addPocket(positions: Float32Array, place: PocketPlace, spec: PocketSpec, wasmUrl?: string): Promise<PocketResult> {
  const wasm = await loadManifold(wasmUrl);
  const solid = solidOf(wasm, positions);
  if (!solid) throw new PocketError('Das Teil ist kein geschlossener Körper (nicht wasserdicht) – erst „Netz reparieren“.');
  const live: { delete(): void }[] = [solid];
  const keep = <T extends { delete(): void }>(x: T): T => {
    live.push(x);
    return x;
  };
  try {
    const notes: string[] = [];
    const thick = thicknessAt(positions, place);
    const blind = spec.kind === 'magnet' || spec.kind === 'insert' || (spec.kind === 'hole' && !spec.through);
    if (blind && Number.isFinite(thick) && spec.depth > thick - 0.6) {
      notes.push(spec.depth >= thick
        ? `Achtung: Das Teil ist hier nur ${thick.toFixed(1).replace('.', ',')} mm dick – die Aussparung geht durch.`
        : `Hinten bleiben nur ${(thick - spec.depth).toFixed(1).replace('.', ',')} mm Material.`);
    }
    const tool = pocketTool(wasm, place, spec, thick, keep);
    const res = keep(solid.subtract(tool));
    if (Math.abs(res.volume() - solid.volume()) < 1e-6) throw new PocketError('Die Aussparung trifft das Teil nicht.');
    const t = trackedPositions(res, originalIdOf(solid));
    return { positions: t.positions, source: t.source, notes };
  } finally {
    for (const x of live) x.delete();
  }
}

/** Form der Aussparung für die Vorschau (Koordinaten des Netzes). */
export async function pocketPreview(positions: Float32Array, place: PocketPlace, spec: PocketSpec, wasmUrl?: string): Promise<{ tool: Float32Array; thickness: number }> {
  const wasm = await loadManifold(wasmUrl);
  const live: { delete(): void }[] = [];
  const keep = <T extends { delete(): void }>(x: T): T => {
    live.push(x);
    return x;
  };
  try {
    const thickness = thicknessAt(positions, place);
    return { tool: positionsOfSolid(pocketTool(wasm, place, spec, thickness, keep)), thickness };
  } finally {
    for (const x of live) x.delete();
  }
}
