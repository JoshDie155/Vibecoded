/**
 * Teile zerschneiden (boolesche Operationen mit manifold-3d): Ein geschlossener Körper wird an einer
 * Ebene in zwei Hälften geteilt – auf Wunsch mit Steckverbindern: ein Zapfen an der einen Hälfte und die
 * passende Bohrung in der anderen, oder Bohrungen in beiden für getrennt gedruckte Dübel.
 *
 * Die Verbinder sitzen dort, wo der Querschnitt genug Material hat (Wandstärke um die Bohrung), und
 * reichen höchstens so tief, dass sie auf der anderen Seite nicht durchstoßen.
 */
import type { CrossSection, Manifold, ManifoldToplevel, Mat4, SimplePolygon } from 'manifold-3d';
import { type Vec3, cross, len, normalize } from '../math/align';
import { loadManifold, originalIdOf, positionsOfSolid, solidOf, trackedPositions } from './union';

export type Connector = 'none' | 'pin' | 'dowel';

export interface CutOptions {
  /** Schnittebene in den Koordinaten des Netzes: normal·x = offset. Hälfte B liegt in Richtung normal. */
  normal: Vec3;
  offset: number;
  connector: Connector;
  /** Anzahl der Verbinder, 0 = automatisch (je Querschnittsfläche einer, bei langen zwei). */
  count?: number;
  /** Durchmesser in mm, 0 = automatisch aus dem Querschnitt. */
  diameter?: number;
  /** Spiel (radial) zwischen Zapfen bzw. Dübel und Bohrung in mm. */
  clearance?: number;
}

export interface Placed {
  /** Mitte in der Schnittebene (Koordinaten des Netzes). */
  center: Vec3;
  diameter: number;
  /** Einstecktiefe in Hälfte A bzw. B (mm). */
  depthA: number;
  depthB: number;
}

export interface CutResult {
  /** Hälfte gegen die Normale (A) und in Richtung der Normale (B), nicht indizierte Dreiecke. */
  a: Float32Array;
  b: Float32Array;
  /** Je Dreieck der Hälften: Dreieck des Ausgangsnetzes, aus dem es stammt (−1 = Schnittfläche, Verbinder). */
  aSource: Int32Array;
  bSource: Int32Array;
  /** Getrennt gedruckte Dübel, an ihrer Stelle im Schnitt. */
  dowels: Float32Array[];
  connectors: Placed[];
  /** Hinweise, z. B. zu dünne Stellen ohne Verbinder. */
  notes: string[];
}

export class CutError extends Error {}

/** Material, das um eine Bohrung mindestens stehen bleibt (mm). */
const WALL = 1.2;
/** Zapfen und Dübel sind so viel kürzer als die Bohrung, damit sie nicht aufsitzen (mm). */
const SHORTER = 0.3;
const SEGMENTS = 40;

/** Ebene als z' = 0: Grundvektoren u, v in der Ebene (Rechtssystem mit n) und die Matrizen dazu. */
function planeFrame(n: Vec3, d: number) {
  const u = normalize(cross(n, Math.abs(n[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0]));
  const v = cross(n, u);
  const toPlane: Mat4 = [u[0], v[0], n[0], 0, u[1], v[1], n[1], 0, u[2], v[2], n[2], 0, 0, 0, -d, 1];
  const fromPlane: Mat4 = [u[0], u[1], u[2], 0, v[0], v[1], v[2], 0, n[0], n[1], n[2], 0, n[0] * d, n[1] * d, n[2] * d, 1];
  const point = (x: number, y: number, z: number): Vec3 => [
    u[0] * x + v[0] * y + n[0] * (z + d), u[1] * x + v[1] * y + n[1] * (z + d), u[2] * x + v[2] * y + n[2] * (z + d),
  ];
  return { toPlane, fromPlane, point };
}

/** Punkt in Polygonen (gerade/ungerade – Löcher zählen mit). */
function inside(polys: SimplePolygon[], x: number, y: number): boolean {
  let odd = false;
  for (const poly of polys) {
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const [xi, yi] = poly[i];
      const [xj, yj] = poly[j];
      if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) odd = !odd;
    }
  }
  return odd;
}

/** Erster Schnittpunkt eines Strahls mit dem Netz (Abstand) – für die Wandstärke hinter einem Verbinder. */
export function rayDistance(positions: Float32Array, o: Vec3, dir: Vec3): number {
  let best = Infinity;
  for (let t = 0; t < positions.length; t += 9) {
    const ax = positions[t], ay = positions[t + 1], az = positions[t + 2];
    const e1x = positions[t + 3] - ax, e1y = positions[t + 4] - ay, e1z = positions[t + 5] - az;
    const e2x = positions[t + 6] - ax, e2y = positions[t + 7] - ay, e2z = positions[t + 8] - az;
    const px = dir[1] * e2z - dir[2] * e2y, py = dir[2] * e2x - dir[0] * e2z, pz = dir[0] * e2y - dir[1] * e2x;
    const det = e1x * px + e1y * py + e1z * pz;
    if (Math.abs(det) < 1e-12) continue;
    const inv = 1 / det;
    const sx = o[0] - ax, sy = o[1] - ay, sz = o[2] - az;
    const bu = (sx * px + sy * py + sz * pz) * inv;
    if (bu < 0 || bu > 1) continue;
    const qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
    const bv = (dir[0] * qx + dir[1] * qy + dir[2] * qz) * inv;
    if (bv < 0 || bu + bv > 1) continue;
    const dist = (e2x * qx + e2y * qy + e2z * qz) * inv;
    if (dist > 1e-6 && dist < best) best = dist;
  }
  return best;
}

/**
 * Stellen für k Verbinder in einer (schon um Radius und Wand verkleinerten) Querschnittsfläche: einer in
 * der Mitte, mehrere gleichmäßig entlang der Hauptrichtung. Je Stelle mehrere Versuche, die schrittweise
 * zur Mitte rücken – falls dort zu wenig Tiefe ist.
 */
function spots(inset: CrossSection, k: number): [number, number][][] {
  const polys = inset.toPolygons();
  const { min, max } = inset.bounds();
  const step = Math.max(0.2, Math.max(max[0] - min[0], max[1] - min[1]) / 40);
  const cand: [number, number][] = [];
  for (let x = min[0] + step / 2; x < max[0]; x += step) {
    for (let y = min[1] + step / 2; y < max[1]; y += step) if (inside(polys, x, y)) cand.push([x, y]);
  }
  // Sehr schmal: die Eckpunkte des Umrisses selbst
  if (!cand.length) for (const p of polys) for (const q of p) cand.push([q[0], q[1]]);
  if (!cand.length) return [];
  const mx = cand.reduce((s, c) => s + c[0], 0) / cand.length;
  const my = cand.reduce((s, c) => s + c[1], 0) / cand.length;
  // Hauptrichtung (größter Eigenvektor der Streuung)
  let sxx = 0, sxy = 0, syy = 0;
  for (const [x, y] of cand) {
    sxx += (x - mx) ** 2;
    sxy += (x - mx) * (y - my);
    syy += (y - my) ** 2;
  }
  const ang = 0.5 * Math.atan2(2 * sxy, sxx - syy);
  const ax = Math.cos(ang), ay = Math.sin(ang);
  const ts = cand.map(([x, y]) => (x - mx) * ax + (y - my) * ay);
  const lo = Math.min(...ts), hi = Math.max(...ts);
  const nearest = (px: number, py: number) => cand.reduce((a, c) => ((c[0] - px) ** 2 + (c[1] - py) ** 2 < (a[0] - px) ** 2 + (a[1] - py) ** 2 ? c : a));
  const out: [number, number][][] = [];
  for (let i = 0; i < k; i++) {
    const t = k === 1 ? 0 : lo + ((i + 0.5) / k) * (hi - lo);
    const tries: [number, number][] = [];
    for (let j = 0; j <= 4; j++) {
      const c = nearest(mx + ax * t * (1 - j / 4), my + ay * t * (1 - j / 4));
      if (!tries.some((q) => q[0] === c[0] && q[1] === c[1])) tries.push(c);
    }
    out.push(tries);
  }
  return out;
}

/** Körper des zuletzt geschnittenen bzw. angesehenen Netzes – die Vorschau fragt bei jedem Schieben. */
let cached: { positions: Float32Array; solid: Manifold | null } | null = null;
function solidFor(wasm: ManifoldToplevel, positions: Float32Array): Manifold | null {
  if (cached?.positions !== positions) {
    cached?.solid?.delete();
    cached = { positions, solid: solidOf(wasm, positions) };
  }
  return cached.solid;
}

/** Zwischengespeicherten Körper freigeben (Zerschneiden beendet). */
export function releaseCut() {
  cached?.solid?.delete();
  cached = null;
}

interface Plan {
  frame: ReturnType<typeof planeFrame>;
  n: Vec3;
  dia: number;
  r: number;
  c: number;
  spots: { x: number; y: number; depthA: number; depthB: number }[];
  notes: string[];
}

/** Wo die Verbinder sitzen (in Ebenenkoordinaten) und wie tief sie reichen. */
function plan(solid: Manifold, positions: Float32Array, n: Vec3, d: number, opts: CutOptions, keep: <T extends { delete(): void }>(x: T) => T): Plan {
  const frame = planeFrame(n, d);
  const notes: string[] = [];
  const c = Math.max(0, opts.clearance ?? 0.2);
  const section = keep(keep(solid.transform(frame.toPlane)).slice(0));
  const comps = section.decompose().map(keep).filter((x) => x.area() > 1e-6).sort((a, b) => b.area() - a.area());
  // Durchmesser: vorgegeben, sonst ein Drittel der schmalen Seite der größten Fläche (3–10 mm)
  let dia = opts.diameter && opts.diameter > 0 ? opts.diameter : 0;
  if (!dia && comps.length) {
    const { min, max } = comps[0].bounds();
    dia = Math.min(10, Math.max(3, Math.round((Math.min(max[0] - min[0], max[1] - min[1]) / 3) * 2) / 2));
    // Muss mit Wand hineinpassen – sonst kleiner, bis 2 mm
    while (dia > 2 && keep(comps[0].offset(-(dia / 2 + c + WALL), 'Round')).isEmpty()) dia -= 0.5;
  }
  const r = dia / 2;
  const insets = comps.map((comp) => keep(comp.offset(-(r + c + WALL), 'Round')));
  // Wie viele Verbinder in welche Fläche
  let per = insets.map((inset) => {
    if (inset.isEmpty()) return 0;
    const { min, max } = inset.bounds();
    return Math.max(max[0] - min[0], max[1] - min[1]) > 6 * dia ? 2 : 1;
  });
  if (opts.count && opts.count > 0) {
    per = insets.map(() => 0);
    const usable = insets.map((x, i) => (x.isEmpty() ? -1 : i)).filter((i) => i >= 0);
    for (let k = 0; k < opts.count && usable.length; k++) per[usable[k % usable.length]]++;
  }
  if (!comps.length) notes.push('Die Schnittebene geht nicht durch das Teil.');
  else if (insets.some((x) => x.isEmpty())) {
    notes.push(insets.every((x) => x.isEmpty())
      ? `Der Querschnitt ist zu schmal für Verbinder mit ⌀ ${String(dia).replace('.', ',')} mm.`
      : 'Schmale Stellen des Querschnitts bekommen keinen Verbinder.');
  }
  const deep = Math.min(12, Math.max(3, 1.5 * dia));
  // Tiefe je Seite: höchstens so tief, dass dahinter noch Wand bleibt – auch am Rand der Bohrung,
  // wo eine schräge Außenfläche näher sein kann als in der Mitte
  const depthAt = (x: number, y: number) => {
    const rim = [frame.point(x, y, 0), ...Array.from({ length: 8 }, (_, k) => frame.point(x + (r + c) * Math.cos((k * Math.PI) / 4), y + (r + c) * Math.sin((k * Math.PI) / 4), 0))];
    const thick = (dir: Vec3) => Math.min(...rim.map((q) => rayDistance(positions, q, dir)));
    return [Math.min(deep, thick([-n[0], -n[1], -n[2]]) - WALL), Math.min(deep, thick(n) - WALL)];
  };
  const out: Plan['spots'] = [];
  let shallow = 0;
  insets.forEach((inset, i) => {
    if (!per[i]) return;
    for (const tries of spots(inset, per[i])) {
      const hit = tries.map(([x, y]) => ({ x, y, d: depthAt(x, y) })).find((t) => t.d[0] >= 2 && t.d[1] >= 2);
      if (hit) out.push({ x: hit.x, y: hit.y, depthA: hit.d[0], depthB: hit.d[1] });
      else shallow++;
    }
  });
  if (shallow) notes.push(`${shallow} Verbinder ${shallow > 1 ? 'hätten' : 'hätte'} nicht genug Tiefe und ${shallow > 1 ? 'entfallen' : 'entfällt'}.`);
  return { frame, n, dia, r, c, spots: out, notes };
}

/** Nur die Lage der Verbinder (Vorschau), ohne zu schneiden. */
export async function planConnectors(positions: Float32Array, opts: CutOptions, wasmUrl?: string): Promise<{ connectors: Placed[]; notes: string[]; closed: boolean }> {
  const wasm = await loadManifold(wasmUrl);
  const solid = solidFor(wasm, positions);
  if (!solid) return { connectors: [], notes: [], closed: false };
  const live: { delete(): void }[] = [];
  const keep = <T extends { delete(): void }>(x: T): T => {
    live.push(x);
    return x;
  };
  try {
    const n = normalize(opts.normal);
    const pl = plan(solid, positions, n, opts.offset / len(opts.normal), opts, keep);
    return { connectors: pl.spots.map((s) => ({ center: pl.frame.point(s.x, s.y, 0), diameter: pl.dia, depthA: s.depthA, depthB: s.depthB })), notes: pl.notes, closed: true };
  } finally {
    for (const x of live) x.delete();
  }
}

export async function cutMesh(positions: Float32Array, opts: CutOptions, wasmUrl?: string): Promise<CutResult> {
  const wasm = await loadManifold(wasmUrl);
  return cutWith(wasm, positions, opts);
}

export function cutWith(wasm: ManifoldToplevel, positions: Float32Array, opts: CutOptions): CutResult {
  const solid = solidFor(wasm, positions);
  if (!solid) throw new CutError('Das Teil ist kein geschlossener Körper (nicht wasserdicht) – so lässt es sich nicht zerschneiden.');
  const live: { delete(): void }[] = [];
  const keep = <T extends { delete(): void }>(x: T): T => {
    live.push(x);
    return x;
  };
  try {
    const n = normalize(opts.normal);
    const d = opts.offset / len(opts.normal);
    const [bSide, aSide] = solid.splitByPlane(n, d);
    keep(bSide);
    keep(aSide);
    if (aSide.isEmpty() || bSide.isEmpty()) throw new CutError('Die Schnittebene geht nicht durch das Teil.');
    const notes: string[] = [];
    const connectors: Placed[] = [];
    const dowels: Float32Array[] = [];
    let A: Manifold = aSide;
    let B: Manifold = bSide;
    if (opts.connector !== 'none') {
      const pl = plan(solid, positions, n, d, opts, keep);
      notes.push(...pl.notes);
      const { r, c } = pl;
      const cyl = (radius: number, z0: number, z1: number, x: number, y: number) =>
        keep(keep(wasm.Manifold.cylinder(z1 - z0, radius, radius, SEGMENTS, false)).translate([x, y, z0]));
      const pinsA: Manifold[] = [];
      const toolsA: Manifold[] = [];
      const toolsB: Manifold[] = [];
      const dowelTools: Manifold[] = [];
      for (const { x, y, depthA, depthB } of pl.spots) {
        connectors.push({ center: pl.frame.point(x, y, 0), diameter: pl.dia, depthA, depthB });
        if (opts.connector === 'pin') {
          pinsA.push(cyl(r, -depthA, depthB - SHORTER, x, y));
          toolsB.push(cyl(r + c, -0.5, depthB + c, x, y));
        } else {
          toolsA.push(cyl(r + c, -(depthA + c), 0.5, x, y));
          toolsB.push(cyl(r + c, -0.5, depthB + c, x, y));
          dowelTools.push(cyl(r, -(depthA - SHORTER), depthB - SHORTER, x, y));
        }
      }
      const place = (list: Manifold[]) => keep(keep(wasm.Manifold.union(list)).transform(pl.frame.fromPlane));
      if (pinsA.length) A = keep(A.add(place(pinsA)));
      if (toolsA.length) A = keep(A.subtract(place(toolsA)));
      if (toolsB.length) B = keep(B.subtract(place(toolsB)));
      for (const t of dowelTools) dowels.push(positionsOfSolid(keep(t.transform(pl.frame.fromPlane))));
    }
    const id = originalIdOf(solid);
    const ta = trackedPositions(A, id);
    const tb = trackedPositions(B, id);
    return { a: ta.positions, b: tb.positions, aSource: ta.source, bSource: tb.source, dowels, connectors, notes };
  } finally {
    for (const x of live) x.delete();
  }
}
