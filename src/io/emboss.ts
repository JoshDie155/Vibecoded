/**
 * Beschriftung einprägen (manifold-3d): Umrisse – Text oder Logo, in mm – an einer Stelle der Oberfläche
 * erhaben aufsetzen oder vertieft einlassen. Die Form folgt der Oberfläche: Strahlen senkrecht zur
 * Fläche tasten ein Höhenfeld unter der Form ab, die Form wird fein unterteilt und jeder Punkt auf die
 * Oberfläche darunter gelegt – so liegt Schrift auch auf Rundungen satt auf. Auf ebenen Flächen bleibt
 * sie unverändert eben (keine unnötig feinen Dreiecke).
 */
import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import type { Manifold, ManifoldToplevel, Mat4, Vec2 } from 'manifold-3d';
import { type Vec3, cross, normalize } from '../math/align';
import { thicknessAt } from './pocket';
import { loadManifold, originalIdOf, positionsOfSolid, solidOf, trackedPositions } from './union';

export type EmbossMode = 'raised' | 'engraved';

/** Ort: Punkt auf der Oberfläche, Außennormale dort und die Richtung „oben“ für die Schrift (Netz-Koordinaten). */
export interface EmbossPlace {
  point: Vec3;
  normal: Vec3;
  up: Vec3;
}

export interface EmbossSpec {
  /** Umrisse in mm (x nach rechts, y nach oben, Mitte bei 0, 0). Außen gegen, Löcher mit dem Uhrzeigersinn. */
  polys: Vec2[][];
  mode: EmbossMode;
  /** Höhe (erhaben) bzw. Tiefe (vertieft) in mm. */
  depth: number;
  /** Drehung in der Fläche (Grad, gegen den Uhrzeigersinn von außen gesehen). */
  rotate?: number;
}

/** Abstände zu allen nach außen zeigenden Flächen entlang des Strahls (aufsteigend). */
export type SurfaceHit = (origin: Vec3, dir: Vec3, far: number) => number[];

export interface EmbossResult {
  positions: Float32Array;
  /** Je Dreieck: Herkunft im alten Netz (−1 = Fläche der Beschriftung). */
  source: Int32Array;
  notes: string[];
}

export class EmbossError extends Error {
  name = 'EmbossError';
}

/** So weit steckt erhabene Schrift im Teil, so weit ragt das Werkzeug für vertiefte heraus (mm). */
const ROOT = 0.3;
const OUT = 0.5;

/** Achsen der Fläche: u nach rechts, v nach oben, n nach außen (u × v = n). */
export function embossFrame(place: EmbossPlace, rotate = 0): { u: Vec3; v: Vec3; n: Vec3 } {
  const n = normalize(place.normal);
  const along = (a: Vec3) => {
    const d = a[0] * n[0] + a[1] * n[1] + a[2] * n[2];
    return [a[0] - d * n[0], a[1] - d * n[1], a[2] - d * n[2]] as Vec3;
  };
  let v = along(place.up);
  if (Math.hypot(...v) < 1e-6) v = along(Math.abs(n[2]) < 0.9 ? [0, 0, 1] : [0, 1, 0]);
  v = normalize(v);
  const u = cross(v, n);
  const a = (rotate * Math.PI) / 180;
  const c = Math.cos(a);
  const s = Math.sin(a);
  return {
    u: [u[0] * c + v[0] * s, u[1] * c + v[1] * s, u[2] * c + v[2] * s],
    v: [v[0] * c - u[0] * s, v[1] * c - u[1] * s, v[2] * c - u[2] * s],
    n,
  };
}

/** Strahltest über eine BVH (im Hintergrund-Thread wird sie aus dem Netz gebaut, siehe surfaceOf). */
export function bvhSurface(bvh: MeshBVH): SurfaceHit {
  const ray = new THREE.Ray();
  return (o, d, far) => {
    ray.origin.set(o[0], o[1], o[2]);
    ray.direction.set(d[0], d[1], d[2]);
    return bvh.raycast(ray, THREE.FrontSide, 0, far).map((h) => h.distance).sort((a, b) => a - b);
  };
}

export function surfaceOf(positions: Float32Array): SurfaceHit {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  return bvhSurface(new MeshBVH(g));
}

function bounds(polys: Vec2[][]) {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const poly of polys) for (const [x, y] of poly) {
    x0 = Math.min(x0, x); y0 = Math.min(y0, y); x1 = Math.max(x1, x); y1 = Math.max(y1, y);
  }
  return { x0, y0, x1, y1 };
}

/** Höhenfeld der Oberfläche über der Form (Abstand entlang n zur Ebene durch den Punkt). */
export interface HeightField {
  sample(x: number, y: number): number;
  /** Höhenunterschied im Feld. */
  relief: number;
  /** Anteil der Stellen ohne passende Oberfläche darunter (über eine Kante oder Stufe hinaus). */
  missing: number;
  step: number;
}

/** Steiler als das folgt die Form der Oberfläche nicht (Stufe, Wand dahinter): dort läuft sie eben weiter. */
const MAX_SLOPE = Math.tan((60 * Math.PI) / 180);

/**
 * Höhenfeld: Vom angeklickten Punkt aus wird das Raster Nachbar für Nachbar abgetastet; je Stelle zählt
 * die Oberfläche, die zur Höhe des Nachbarn passt. So klettert die Schrift nicht an einer Wand hoch oder
 * fällt eine Stufe hinab, sondern bleibt auf der zusammenhängenden Fläche.
 */
export function heightField(hit: SurfaceHit, place: EmbossPlace, frame: { u: Vec3; v: Vec3; n: Vec3 }, box: { x0: number; y0: number; x1: number; y1: number }): HeightField {
  const { u, v, n } = frame;
  const diag = Math.hypot(box.x1 - box.x0, box.y1 - box.y0);
  const step = Math.min(1, Math.max(0.15, diag / 120));
  const range = Math.max(2, 0.6 * diag);
  const gx0 = Math.min(0, box.x0) - step;
  const gy0 = Math.min(0, box.y0) - step;
  const nx = Math.ceil((Math.max(0, box.x1) - gx0) / step) + 2;
  const ny = Math.ceil((Math.max(0, box.y1) - gy0) / step) + 2;
  const p = place.point;
  const down: Vec3 = [-n[0], -n[1], -n[2]];
  const heights = (k: number) => {
    const x = gx0 + (k % nx) * step;
    const y = gy0 + Math.floor(k / nx) * step;
    const o: Vec3 = [0, 1, 2].map((c) => p[c] + x * u[c] + y * v[c] + range * n[c]) as Vec3;
    return hit(o, down, 2 * range).map((d) => range - d);
  };
  const h = new Float64Array(nx * ny).fill(NaN);
  // Start am angeklickten Punkt (Höhe 0)
  const start = Math.round(-gy0 / step) * nx + Math.round(-gx0 / step);
  const near = (list: number[], ref: number) => list.reduce((b, x) => (Number.isNaN(b) || Math.abs(x - ref) < Math.abs(b - ref) ? x : b), NaN);
  const s0 = near(heights(start), 0);
  h[start] = Number.isFinite(s0) && Math.abs(s0) < step * MAX_SLOPE ? s0 : 0;
  let miss = 0;
  let lo = h[start], hi = h[start];
  const queue = [start];
  for (let q = 0; q < queue.length; q++) {
    const k = queue[q];
    const i = k % nx;
    const j = (k - i) / nx;
    for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      const a = i + di;
      const b = j + dj;
      if (a < 0 || b < 0 || a >= nx || b >= ny) continue;
      const m = b * nx + a;
      if (!Number.isNaN(h[m])) continue;
      const s = near(heights(m), h[k]);
      if (Number.isFinite(s) && Math.abs(s - h[k]) <= step * MAX_SLOPE) {
        h[m] = s;
        lo = Math.min(lo, s);
        hi = Math.max(hi, s);
      } else {
        // Keine passende Oberfläche: eben weiter
        h[m] = h[k];
        miss++;
      }
      queue.push(m);
    }
  }
  return {
    sample(x, y) {
      const fx = Math.min(nx - 1.000001, Math.max(0, (x - gx0) / step));
      const fy = Math.min(ny - 1.000001, Math.max(0, (y - gy0) / step));
      const i = Math.floor(fx), j = Math.floor(fy);
      const a = fx - i, b = fy - j;
      const k = j * nx + i;
      return (h[k] * (1 - a) + h[k + 1] * a) * (1 - b) + (h[k + nx] * (1 - a) + h[k + nx + 1] * a) * b;
    },
    relief: hi - lo,
    missing: miss / (nx * ny),
    step,
  };
}

/**
 * Körper der Beschriftung in Netz-Koordinaten: erhaben von knapp unter der Oberfläche bis zur Höhe
 * darüber, vertieft von der Tiefe unter der Oberfläche bis etwas darüber hinaus.
 */
export function embossTool(
  wasm: ManifoldToplevel, hit: SurfaceHit, place: EmbossPlace, spec: EmbossSpec, keep: <T extends { delete(): void }>(x: T) => T,
): { tool: Manifold; field: HeightField } {
  if (!spec.polys.length) throw new EmbossError('Kein Text bzw. keine Form zum Einprägen.');
  const frame = embossFrame(place, spec.rotate ?? 0);
  const box = bounds(spec.polys);
  const field = heightField(hit, place, frame, box);
  const cs = keep(new wasm.CrossSection(spec.polys, 'Positive'));
  if (cs.isEmpty()) throw new EmbossError('Die Form hat keine Fläche.');
  const depth = Math.max(0.05, spec.depth);
  const z0 = spec.mode === 'raised' ? -ROOT : -depth;
  const z1 = spec.mode === 'raised' ? depth : OUT;
  const { u, v, n } = frame;
  const p = place.point;
  let tool: Manifold;
  if (field.relief < 1e-3) {
    // Eben: einfach hochziehen und an die Stelle legen
    const s = field.sample(0, 0);
    tool = keep(keep(cs.extrude(z1 - z0)).translate([0, 0, z0 + s]));
  } else {
    tool = keep(keep(keep(cs.extrude(1)).refineToLength(field.step)).warp((q) => {
      const s = field.sample(q[0], q[1]);
      q[2] = s + z0 + q[2] * (z1 - z0);
    }));
  }
  const m: Mat4 = [u[0], u[1], u[2], 0, v[0], v[1], v[2], 0, n[0], n[1], n[2], 0, p[0], p[1], p[2], 1];
  return { tool: keep(tool.transform(m)), field };
}

/** Beschriftung ins Teil rechnen. hit: Strahltest auf dem Teil (ohne: BVH wird hier gebaut). */
export async function embossMesh(positions: Float32Array, place: EmbossPlace, spec: EmbossSpec, wasmUrl?: string, hit?: SurfaceHit): Promise<EmbossResult> {
  const wasm = await loadManifold(wasmUrl);
  const solid = solidOf(wasm, positions);
  if (!solid) throw new EmbossError('Das Teil ist kein geschlossener Körper (nicht wasserdicht) – erst „Netz reparieren“.');
  const live: { delete(): void }[] = [solid];
  const keep = <T extends { delete(): void }>(x: T): T => {
    live.push(x);
    return x;
  };
  try {
    const notes: string[] = [];
    const { tool, field } = embossTool(wasm, hit ?? surfaceOf(positions), place, spec, keep);
    if (field.missing > 0.02) notes.push('Die Beschriftung ragt über eine Kante oder Stufe hinaus – dort läuft sie eben weiter.');
    const n = normalize(place.normal);
    if (spec.mode === 'raised' && n[2] < -0.5) notes.push('Erhabene Schrift an einer Unterseite braucht Stützen – vertieft geht ohne.');
    if (spec.mode === 'engraved') {
      const t = thicknessAt(positions, { point: place.point, normal: place.normal });
      if (Number.isFinite(t) && spec.depth > t - 0.6) {
        notes.push(spec.depth >= t ? 'Achtung: Die Schrift geht hier durch das Teil.' : `Hinter der Schrift bleiben nur ${(t - spec.depth).toFixed(1).replace('.', ',')} mm.`);
      }
    }
    const res = keep(spec.mode === 'raised' ? solid.add(tool) : solid.subtract(tool));
    if (Math.abs(res.volume() - solid.volume()) < 1e-6) throw new EmbossError('Die Beschriftung trifft das Teil nicht.');
    const t = trackedPositions(res, originalIdOf(solid));
    return { positions: t.positions, source: t.source, notes };
  } finally {
    for (const x of live) x.delete();
  }
}

/** Form der Beschriftung für die Vorschau (Netz-Koordinaten). */
export async function embossPreview(hit: SurfaceHit, place: EmbossPlace, spec: EmbossSpec, wasmUrl?: string): Promise<{ tool: Float32Array; relief: number; missing: number }> {
  const wasm = await loadManifold(wasmUrl);
  const live: { delete(): void }[] = [];
  const keep = <T extends { delete(): void }>(x: T): T => {
    live.push(x);
    return x;
  };
  try {
    const { tool, field } = embossTool(wasm, hit, place, spec, keep);
    return { tool: positionsOfSolid(tool), relief: field.relief, missing: field.missing };
  } finally {
    for (const x of live) x.delete();
  }
}
