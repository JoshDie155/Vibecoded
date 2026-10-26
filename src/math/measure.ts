/**
 * Messen wie in Bambu Studio: Merkmale unter dem Mauszeiger und die Maße zwischen zweien.
 *
 * Ebene Flächen sind aneinanderhängende Dreiecke gleicher Ausrichtung. Ihr Rand zerfällt in Kreise
 * (Bohrungen, runde Außenkanten – auch Bögen ab etwa 80°), Vielecke (5–8 gleiche Seiten wie ein Sechskant:
 * Kanten mit gemeinsamem Mittelpunkt) und gerade Kanten; deren Enden sind Ecken. Auf gekrümmten Flächen
 * gibt es statt der Dreieckskanten des Netzes Zylinder (Bohrung, Zapfen) – oder einfach den Punkt.
 *
 * Alles in den Koordinaten des Netzes; toWorld bringt ein Merkmal in die Lage des Teils.
 */
import { type Vec3, add, cross, dist, dot, len, normalize, scale, sub } from './align';
import { type Topology, detectFeature } from './features';

export type MFeature =
  | { kind: 'point'; p: Vec3; on: 'vertex' | 'edge' | 'circle' | 'plane' | 'surface' | 'center' }
  /** Gerade Kante; center/sides: Seite eines regelmäßigen Vielecks mit diesem Mittelpunkt. */
  | { kind: 'edge'; a: Vec3; b: Vec3; center?: Vec3; sides?: number }
  /** Kreis oder Bogen (arc < 2π) um normal; start: Richtung vom Mittelpunkt zum Anfang des Bogens. */
  | { kind: 'circle'; center: Vec3; normal: Vec3; radius: number; arc: number; start: Vec3 }
  | { kind: 'plane'; normal: Vec3; origin: Vec3; region: number; area: number }
  | { kind: 'cylinder'; center: Vec3; axis: Vec3; radius: number; length: number; hole: boolean; tris: number[] };

interface Region {
  tris: number[];
  normal: Vec3;
  origin: Vec3;
  area: number;
  /** false: sieht nach einer Facette einer gekrümmten Fläche aus (wird bei Bedarf geprüft). */
  planar: boolean;
  checked: boolean;
  features?: MFeature[];
}

export interface MeasureModel {
  topo: Topology;
  /** Fläche je Dreieck (−1: noch nicht bestimmt). */
  triRegion: Int32Array;
  regions: Region[];
  /** Gekrümmte Flächen: Zylinder bzw. null (Freiform) je Dreieck. */
  curved: Map<number, Extract<MFeature, { kind: 'cylinder' }> | null>;
}

const DEG = Math.PI / 180;
/** Dreiecke einer ebenen Fläche: Normale höchstens so weit von der des ersten. */
const FLAT_COS = Math.cos(0.25 * DEG);
/** Knick unter diesem Winkel zählt als glatter Übergang (Facetten einer Rundung). */
const SMOOTH_COS = Math.cos(20 * DEG);

const v3 = (a: ArrayLike<number>, i: number): Vec3 => [a[3 * i], a[3 * i + 1], a[3 * i + 2]];

export function measureModel(topo: Topology): MeasureModel {
  return { topo, triRegion: new Int32Array(topo.triCount).fill(-1), regions: [], curved: new Map() };
}

/** Ebene Fläche um ein Dreieck (wird beim ersten Mal gesucht). */
function regionOf(m: MeasureModel, t: number): { id: number; r: Region } {
  const known = m.triRegion[t];
  if (known >= 0) return { id: known, r: m.regions[known] };
  const { normals, neighbors, areas, centroids, positions } = m.topo;
  const id = m.regions.length;
  const n0 = v3(normals, t);
  const zero = (n: Vec3) => n[0] === 0 && n[1] === 0 && n[2] === 0;
  const tris = [t];
  m.triRegion[t] = id;
  if (!zero(n0)) {
    for (let i = 0; i < tris.length; i++) {
      const u = tris[i];
      for (let k = 0; k < 3; k++) {
        const v = neighbors[3 * u + k];
        if (v < 0 || m.triRegion[v] >= 0) continue;
        const nv = v3(normals, v);
        // Entartete Dreiecke (ohne Normale) gehören zur Fläche, in der sie liegen
        if (!zero(nv) && dot(nv, n0) < FLAT_COS) continue;
        m.triRegion[v] = id;
        tris.push(v);
      }
    }
  }
  let area = 0;
  let origin: Vec3 = [0, 0, 0];
  for (const u of tris) {
    area += areas[u];
    origin = add(origin, scale(v3(centroids, u), areas[u]));
  }
  origin = area > 0 ? scale(origin, 1 / area) : v3(centroids, t);
  // Wie viel des Randes geht glatt in die Nachbarflächen über? Viel: Facette einer Rundung
  let smooth = 0;
  let total = 0;
  for (const u of tris) {
    for (let k = 0; k < 3; k++) {
      const v = neighbors[3 * u + k];
      if (v >= 0 && m.triRegion[v] === id) continue;
      const l = dist(v3(positions, 3 * u + k), v3(positions, 3 * u + ((k + 1) % 3)));
      total += l;
      if (v >= 0 && !zero(n0) && dot(v3(normals, v), n0) > SMOOTH_COS) smooth += l;
    }
  }
  const r: Region = { tris, normal: n0, origin, area, planar: !(total > 0 && smooth > 0.6 * total), checked: false };
  m.regions.push(r);
  return { id, r };
}

/** Gekrümmte Fläche: Zylinder (Bohrung/Zapfen), sonst null. Nebenbei: doch eine ebene Fläche? */
function curvedAt(m: MeasureModel, t: number, r: Region): Extract<MFeature, { kind: 'cylinder' }> | null | 'plane' {
  if (m.curved.has(t)) return m.curved.get(t)!;
  const f = detectFeature(m.topo, t, { maxTris: 60000 });
  if (f.kind === 'plane') {
    // Ebene, nur von Rundungen eingefasst (z. B. Deckfläche mit Verrundung)
    r.planar = true;
    return 'plane';
  }
  const out = f.kind === 'cylinder'
    ? { kind: 'cylinder' as const, center: f.center, axis: f.axis, radius: f.radius, length: f.length, hole: f.hole, tris: f.tris }
    : null;
  for (const u of f.tris) m.curved.set(u, out);
  m.curved.set(t, out);
  return out;
}

// ---------------------------------------------------------------- Rand einer Fläche zerlegen

/** Zwei Richtungen in der Ebene zur Normale n. */
function planeBasis(n: Vec3): [Vec3, Vec3] {
  const helper: Vec3 = Math.abs(n[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const u = normalize(cross(n, helper));
  return [u, cross(n, u)];
}

/** Ausgleichskreis (kleinste Quadrate) durch Punkte in der Ebene; maxDev: größte Abweichung vom Radius. */
function fitCircle(pts: Vec3[], n: Vec3): { center: Vec3; radius: number; maxDev: number } | null {
  if (pts.length < 3) return null;
  const [u, v] = planeBasis(n);
  let mx = 0;
  let my = 0;
  const xy = pts.map((p) => [dot(p, u), dot(p, v)] as const);
  for (const [x, y] of xy) { mx += x; my += y; }
  mx /= xy.length;
  my /= xy.length;
  // Kreisgleichung x² + y² + D x + E y + F = 0 (zentriert für die Genauigkeit)
  let sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sxz = 0, syz = 0, sz = 0;
  for (const [x0, y0] of xy) {
    const x = x0 - mx;
    const y = y0 - my;
    const z = x * x + y * y;
    sxx += x * x; sxy += x * y; syy += y * y; sx += x; sy += y; sxz += x * z; syz += y * z; sz += z;
  }
  const N = xy.length;
  // Normalgleichungen für [D, E, F]
  const A = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, N]];
  const b = [-sxz, -syz, -sz];
  const det = (M: number[][]) => M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
  const d = det(A);
  if (!(Math.abs(d) > 1e-18)) return null;
  const col = (k: number) => A.map((row, i) => row.map((x, j) => (j === k ? b[i] : x)));
  const D = det(col(0)) / d;
  const E = det(col(1)) / d;
  const F = det(col(2)) / d;
  const cx = -D / 2;
  const cy = -E / 2;
  const r2 = cx * cx + cy * cy - F;
  if (!(r2 > 0)) return null;
  const radius = Math.sqrt(r2);
  let maxDev = 0;
  for (const [x0, y0] of xy) maxDev = Math.max(maxDev, Math.abs(Math.hypot(x0 - mx - cx, y0 - my - cy) - radius));
  // Mittelpunkt in 3D: auf der Ebene der Punkte
  const h = pts.reduce((s, p) => s + dot(p, n), 0) / pts.length;
  const center = add(add(scale(u, cx + mx), scale(v, cy + my)), scale(n, h));
  return { center, radius, maxDev };
}

/** Vorzeichenbehafteter Winkel von a nach b um n. */
const angleAround = (a: Vec3, b: Vec3, n: Vec3) => Math.atan2(dot(cross(a, b), n), dot(a, b));

/** Kreis/Bogen aus Randpunkten in Umlaufrichtung (Normale so gedreht, dass der Bogen positiv läuft). */
function arcFeature(pts: Vec3[], n: Vec3, center: Vec3, radius: number): Extract<MFeature, { kind: 'circle' }> {
  let sweep = 0;
  for (let i = 1; i < pts.length; i++) sweep += angleAround(sub(pts[i - 1], center), sub(pts[i], center), n);
  const normal = sweep >= 0 ? n : scale(n, -1);
  const arc = Math.min(2 * Math.PI, Math.abs(sweep));
  return { kind: 'circle', center, normal, radius, arc: arc > 2 * Math.PI - 0.05 ? 2 * Math.PI : arc, start: normalize(sub(pts[0], center)) };
}

/** Eine Randschleife (geschlossen, ohne doppelten Endpunkt) in Kreise, Bögen und Kanten zerlegen. */
export function splitLoop(raw: Vec3[], n: Vec3): MFeature[] {
  // Doppelte Punkte weg
  const pts: Vec3[] = [];
  for (const p of raw) if (!pts.length || dist(p, pts[pts.length - 1]) > 1e-9) pts.push(p);
  while (pts.length > 2 && dist(pts[0], pts[pts.length - 1]) <= 1e-9) pts.pop();
  const N = pts.length;
  if (N < 3) return [];
  const at = (i: number) => pts[((i % N) + N) % N];
  const segLen = (i: number) => dist(at(i), at(i + 1));
  const lengths = pts.map((_, i) => segLen(i));
  const sameLengths = (tol: number) => {
    const mean = lengths.reduce((s, l) => s + l, 0) / N;
    return lengths.every((l) => Math.abs(l - mean) <= tol * mean);
  };

  // Ganze Schleife ein Kreis (mehr als 8 Punkte) oder ein regelmäßiges Vieleck (5–8)
  if (N >= 5) {
    const fit = fitCircle(pts, n);
    if (fit && fit.maxDev <= Math.max(0.01 * fit.radius, 1e-4)) {
      // Punkte laufen gleichmäßig um den Mittelpunkt (keine Rückwärtsschritte, keine großen Sprünge)
      const steps = pts.map((_, i) => angleAround(sub(at(i), fit.center), sub(at(i + 1), fit.center), n));
      const sign = Math.sign(steps[0]);
      const even = steps.every((s) => Math.sign(s) === sign && Math.abs(s) < 75 * DEG);
      const total = Math.abs(steps.reduce((s, x) => s + x, 0));
      if (even && Math.abs(total - 2 * Math.PI) < 0.05) {
        if (N > 8 && sameLengths(0.25)) return [{ ...arcFeature([...pts, pts[0]], n, fit.center, fit.radius), arc: 2 * Math.PI }];
        if (N <= 8 && sameLengths(0.02)) {
          return pts.map((p, i) => ({ kind: 'edge' as const, a: p, b: at(i + 1), center: fit.center, sides: N }));
        }
      }
    }
  }

  // Sonst: Bögen suchen, der Rest sind Kanten
  const dirs = pts.map((_, i) => normalize(sub(at(i + 1), at(i))));
  // Knick an Punkt i (zwischen Segment i−1 und i)
  const turn = pts.map((_, i) => angleAround(dirs[(i - 1 + N) % N], dirs[i], n));
  const smooth = (i: number) => Math.abs(turn[(i + N) % N]) > 0.3 * DEG && Math.abs(turn[(i + N) % N]) < 40 * DEG;
  const similar = (i: number, j: number) => {
    const a = turn[(i + N) % N];
    const b = turn[(j + N) % N];
    return Math.sign(a) === Math.sign(b) && Math.abs(a - b) <= Math.max(0.5 * DEG, 0.2 * Math.abs(a))
      && Math.abs(lengths[(i + N) % N] - lengths[(j + N) % N]) <= 0.25 * Math.max(lengths[(i + N) % N], lengths[(j + N) % N]);
  };
  // Beginnen an einem Punkt, der sicher nicht mitten in einem Bogen liegt: Ecke oder Übergang
  let s0 = turn.findIndex((_, i) => !smooth(i) || !similar(i - 1, i));
  if (s0 < 0) s0 = 0;
  /** Segment i (von Punkt i nach i+1) gehört zu einem Bogen. */
  const inArc = new Array<boolean>(N).fill(false);
  const out: MFeature[] = [];
  for (let k = 0; k < N;) {
    const i = s0 + k;
    if (!smooth(i)) { k++; continue; }
    // Lauf gleichartiger Knicke ab Punkt i
    let j = i;
    while (j - s0 + 1 < N && smooth(j + 1) && similar(j, j + 1)) j++;
    const count = j - i + 1;
    if (count >= 3) {
      // Bogen von Punkt i−1 bis j+1 – nur, wenn er mindestens etwa 80° überspannt
      const arcPts = Array.from({ length: count + 2 }, (_, q) => at(i - 1 + q));
      const fit = fitCircle(arcPts, n);
      if (fit && fit.maxDev <= Math.max(0.02 * fit.radius, 1e-4)) {
        const arc = arcFeature(arcPts, n, fit.center, fit.radius);
        if (arc.arc >= 80 * DEG) {
          out.push(arc);
          for (let q = i - 1; q <= j; q++) inArc[((q % N) + N) % N] = true;
        }
      }
    }
    k = j - s0 + 1;
  }
  // Kanten: Segmente außerhalb der Bögen; gerade aufeinanderfolgende zusammenfassen
  const edges: { a: Vec3; b: Vec3; dir: Vec3 }[] = [];
  for (let k = 0; k < N; k++) {
    const i = (s0 + k) % N;
    if (inArc[i] || lengths[i] <= 1e-9) continue;
    const last = edges[edges.length - 1];
    if (last && !inArc[(i - 1 + N) % N] && dist(last.b, at(i)) <= 1e-9 && dot(last.dir, dirs[i]) > Math.cos(0.5 * DEG)) {
      last.b = at(i + 1);
      continue;
    }
    edges.push({ a: at(i), b: at(i + 1), dir: dirs[i] });
  }
  // Über den Anfang hinweg zusammenfassen
  if (edges.length > 1) {
    const first = edges[0];
    const last = edges[edges.length - 1];
    if (dist(last.b, first.a) <= 1e-9 && dot(last.dir, first.dir) > Math.cos(0.5 * DEG)) {
      first.a = last.a;
      edges.pop();
    }
  }
  for (const e of edges) out.push({ kind: 'edge', a: e.a, b: e.b });
  return out;
}

/** Merkmale einer ebenen Fläche: Randkreise, -bögen, -kanten und zuletzt die Fläche selbst. */
export function planeFeatures(m: MeasureModel, id: number): MFeature[] {
  const r = m.regions[id];
  if (r.features) return r.features;
  const { neighbors, vertIds, vertPos } = m.topo;
  // Randkanten in Umlaufrichtung der Dreiecke: Nachbar gehört nicht zur Fläche
  const next = new Map<number, number[]>();
  for (const u of r.tris) {
    for (let k = 0; k < 3; k++) {
      const v = neighbors[3 * u + k];
      if (v >= 0 && m.triRegion[v] === id) continue;
      const a = vertIds[3 * u + k];
      const b = vertIds[3 * u + ((k + 1) % 3)];
      if (a === b) continue;
      const list = next.get(a);
      if (list) list.push(b);
      else next.set(a, [b]);
    }
  }
  const features: MFeature[] = [];
  for (const startVert of [...next.keys()]) {
    while (next.get(startVert)?.length) {
      const loop = [startVert];
      let cur = startVert;
      for (let guard = 0; guard < 1_000_000; guard++) {
        const outs = next.get(cur);
        if (!outs?.length) break;
        cur = outs.pop()!;
        if (cur === startVert) break;
        loop.push(cur);
      }
      if (loop.length >= 3) features.push(...splitLoop(loop.map((i) => v3(vertPos, i)), r.normal));
    }
  }
  features.push({ kind: 'plane', normal: r.normal, origin: r.origin, region: id, area: r.area });
  r.features = features;
  return features;
}

// ---------------------------------------------------------------- Merkmal unter dem Zeiger

/** Kürzester Abstand von p zur Strecke a–b und der Punkt darauf. */
export function closestOnSegment(p: Vec3, a: Vec3, b: Vec3): { d: number; q: Vec3; t: number } {
  const ab = sub(b, a);
  const l2 = dot(ab, ab);
  const t = l2 > 0 ? Math.max(0, Math.min(1, dot(sub(p, a), ab) / l2)) : 0;
  const q = add(a, scale(ab, t));
  return { d: dist(p, q), q, t };
}

/** Nächster Punkt auf dem Kreis (bzw. Bogen) und sein Abstand. */
export function closestOnCircle(p: Vec3, f: Extract<MFeature, { kind: 'circle' }>): { d: number; q: Vec3 } {
  const rel = sub(p, f.center);
  const h = dot(rel, f.normal);
  let inPlane = sub(rel, scale(f.normal, h));
  if (len(inPlane) < 1e-12) inPlane = f.start;
  let dir = normalize(inPlane);
  if (f.arc < 2 * Math.PI) {
    // Auf den Bogen begrenzen
    let ang = angleAround(f.start, dir, f.normal);
    if (ang < 0) ang += 2 * Math.PI;
    if (ang > f.arc) {
      const end = rotateAbout(f.start, f.normal, f.arc);
      dir = ang - f.arc < 2 * Math.PI - ang ? end : f.start;
    }
  }
  const q = add(f.center, scale(dir, f.radius));
  return { d: dist(p, q), q };
}

function rotateAbout(v: Vec3, axis: Vec3, ang: number): Vec3 {
  const c = Math.cos(ang);
  const s = Math.sin(ang);
  return add(add(scale(v, c), scale(cross(axis, v), s)), scale(axis, dot(axis, v) * (1 - c)));
}

/**
 * Merkmal am Punkt p (Koordinaten des Netzes) auf Dreieck tri: Kante, Ecke oder Kreis, wenn p höchstens
 * tol davon entfernt ist, sonst die Fläche; auf gekrümmten Flächen ein Zylinder oder der Punkt.
 */
export function featureAt(m: MeasureModel, tri: number, p: Vec3, tol: number): MFeature {
  const { id, r } = regionOf(m, tri);
  if (!r.planar) {
    const c = curvedAt(m, tri, r);
    if (c !== 'plane') {
      // Am Rand einer Rundung: lieber der Kreis bzw. die Kante der angrenzenden ebenen Fläche
      return borderNear(m, tri, p, tol, (rid) => rid !== id && m.regions[rid].planar) ?? c ?? { kind: 'point', p, on: 'surface' };
    }
  }
  return borderNear(m, tri, p, tol, (rid) => rid === id) ?? planeFeatures(m, id).at(-1)!;
}

/** Nächster Kreis, Kante oder Ecke am Rand der ebenen Flächen (Auswahl über ok) nahe p. */
function borderNear(m: MeasureModel, tri: number, p: Vec3, tol: number, ok: (region: number) => boolean): MFeature | null {
  const { neighbors, positions } = m.topo;
  // Dreiecke in Reichweite des Zeigers
  const seen = new Set([tri]);
  const queue = [tri];
  const regions = new Set<number>();
  for (let i = 0; i < queue.length && i < 400; i++) {
    const u = queue[i];
    const rid = regionOf(m, u).id;
    if (ok(rid)) regions.add(rid);
    for (let k = 0; k < 3; k++) {
      const v = neighbors[3 * u + k];
      if (v < 0 || seen.has(v)) continue;
      seen.add(v);
      const a = v3(positions, 3 * v);
      const b = v3(positions, 3 * v + 1);
      const c = v3(positions, 3 * v + 2);
      if (Math.min(closestOnSegment(p, a, b).d, closestOnSegment(p, b, c).d, closestOnSegment(p, c, a).d) <= tol) queue.push(v);
    }
  }
  let best: MFeature | null = null;
  let bestD = tol;
  for (const rid of regions) {
    for (const f of planeFeatures(m, rid)) {
      let d = Infinity;
      if (f.kind === 'edge') d = closestOnSegment(p, f.a, f.b).d;
      else if (f.kind === 'circle') d = closestOnCircle(p, f).d;
      if (d <= bestD) { bestD = d; best = f; }
    }
  }
  if (best?.kind === 'edge') {
    // Nahe am Ende: die Ecke
    const snap = Math.min(tol * 1.5, 0.25 * dist(best.a, best.b));
    if (dist(p, best.a) <= snap) return { kind: 'point', p: best.a, on: 'vertex' };
    if (dist(p, best.b) <= snap) return { kind: 'point', p: best.b, on: 'vertex' };
  }
  return best;
}

/** Punkt auf einem Merkmal, möglichst nahe an p (Umschalt + Klick: beliebiger Punkt). */
export function pointOn(f: MFeature, p: Vec3): Extract<MFeature, { kind: 'point' }> {
  if (f.kind === 'point') return f;
  if (f.kind === 'edge') return { kind: 'point', p: closestOnSegment(p, f.a, f.b).q, on: 'edge' };
  if (f.kind === 'circle') return { kind: 'point', p: closestOnCircle(p, f).q, on: 'circle' };
  return { kind: 'point', p, on: f.kind === 'plane' ? 'plane' : 'surface' };
}

/** Mittelpunkt eines Kreises, Vielecks oder Zylinders (sonst nichts). */
export function centerOf(f: MFeature): Extract<MFeature, { kind: 'point' }> | null {
  if (f.kind === 'circle' || f.kind === 'cylinder') return { kind: 'point', p: f.center, on: 'center' };
  if (f.kind === 'edge' && f.center) return { kind: 'point', p: f.center, on: 'center' };
  return null;
}

/** Merkmal in die Lage des Teils bringen (Matrix spaltenweise wie bei three.js; gleichmäßige Skalierung). */
export function toWorld(f: MFeature, e: ArrayLike<number>): MFeature {
  const P = (p: Vec3): Vec3 => [
    e[0] * p[0] + e[4] * p[1] + e[8] * p[2] + e[12],
    e[1] * p[0] + e[5] * p[1] + e[9] * p[2] + e[13],
    e[2] * p[0] + e[6] * p[1] + e[10] * p[2] + e[14],
  ];
  const D = (d: Vec3): Vec3 => normalize([e[0] * d[0] + e[4] * d[1] + e[8] * d[2], e[1] * d[0] + e[5] * d[1] + e[9] * d[2], e[2] * d[0] + e[6] * d[1] + e[10] * d[2]]);
  const s = Math.cbrt(Math.abs(e[0] * (e[5] * e[10] - e[9] * e[6]) - e[4] * (e[1] * e[10] - e[9] * e[2]) + e[8] * (e[1] * e[6] - e[5] * e[2]))) || 1;
  switch (f.kind) {
    case 'point': return { ...f, p: P(f.p) };
    case 'edge': return { ...f, a: P(f.a), b: P(f.b), ...(f.center ? { center: P(f.center) } : {}) };
    case 'circle': return { ...f, center: P(f.center), normal: D(f.normal), radius: f.radius * s, start: D(f.start) };
    case 'plane': return { ...f, normal: D(f.normal), origin: P(f.origin), area: f.area * s * s };
    case 'cylinder': return { ...f, center: P(f.center), axis: D(f.axis), radius: f.radius * s, length: f.length * s };
  }
}

// ---------------------------------------------------------------- Maße zwischen zwei Merkmalen

export interface Span {
  d: number;
  from: Vec3;
  to: Vec3;
}

export interface MeasureResult {
  /** Winkel zwischen den Merkmalen (Grad); inner: bei zwei Flächen der Winkel dazwischen. */
  angle?: { deg: number; inner?: number };
  /** Senkrechter Abstand (parallele Kanten/Flächen, Punkt zur Ebene/Geraden, Achsabstand). */
  perpendicular?: Span & { axis?: boolean };
  /** Kürzester Abstand der Merkmale selbst (bei Kreisen und Zylindern: zum Mittelpunkt). */
  direct?: Span;
}

const PARALLEL_SIN = Math.sin(0.05 * DEG);
const parallel = (a: Vec3, b: Vec3) => len(cross(a, b)) < PARALLEL_SIN;
const perpendicularTo = (a: Vec3, b: Vec3) => Math.abs(dot(a, b)) < PARALLEL_SIN;
const deg = (rad: number) => rad / DEG;
/** Winkel zwischen Richtungen ohne Vorzeichen (0–90°). */
const lineAngle = (a: Vec3, b: Vec3) => deg(Math.acos(Math.min(1, Math.abs(dot(normalize(a), normalize(b))))));

/** Lot von p auf die Gerade durch o mit Richtung d. */
function toLine(p: Vec3, o: Vec3, d: Vec3): Span {
  const q = add(o, scale(d, dot(sub(p, o), d)));
  return { d: dist(p, q), from: p, to: q };
}
function toPlane(p: Vec3, o: Vec3, n: Vec3): Span {
  const h = dot(sub(p, o), n);
  return { d: Math.abs(h), from: p, to: sub(p, scale(n, h)) };
}
/** Kürzeste Verbindung zweier Strecken. */
function segmentSegment(a0: Vec3, a1: Vec3, b0: Vec3, b1: Vec3): Span {
  const d1 = sub(a1, a0);
  const d2 = sub(b1, b0);
  const r = sub(a0, b0);
  const A = dot(d1, d1);
  const E = dot(d2, d2);
  const F = dot(d2, r);
  let s = 0;
  let t = 0;
  if (A <= 1e-18 && E <= 1e-18) return { d: dist(a0, b0), from: a0, to: b0 };
  if (A <= 1e-18) t = Math.max(0, Math.min(1, F / E));
  else {
    const C = dot(d1, r);
    if (E <= 1e-18) s = Math.max(0, Math.min(1, -C / A));
    else {
      const B = dot(d1, d2);
      const den = A * E - B * B;
      s = den > 1e-18 ? Math.max(0, Math.min(1, (B * F - C * E) / den)) : 0;
      t = (B * s + F) / E;
      if (t < 0) { t = 0; s = Math.max(0, Math.min(1, -C / A)); }
      else if (t > 1) { t = 1; s = Math.max(0, Math.min(1, (B - C) / A)); }
    }
  }
  const p = add(a0, scale(d1, s));
  const q = add(b0, scale(d2, t));
  return { d: dist(p, q), from: p, to: q };
}
/** Abstand zweier paralleler Geraden (Lot von o1 auf die zweite). */
const lineLine = (o1: Vec3, o2: Vec3, d: Vec3) => toLine(o1, o2, d);
/** Lot zwischen parallelen Strecken – mitten im gemeinsamen Stück, sonst am nächsten Ende. */
function parallelSegments(a0: Vec3, a1: Vec3, b0: Vec3, b1: Vec3): Span {
  const d = sub(a1, a0);
  const len = Math.hypot(...d);
  if (len < 1e-12) return lineLine(a0, b0, normalize(sub(b1, b0)));
  const u = scale(d, 1 / len);
  const t0 = dot(sub(b0, a0), u);
  const t1 = dot(sub(b1, a0), u);
  const lo = Math.max(0, Math.min(t0, t1));
  const hi = Math.min(len, Math.max(t0, t1));
  const t = lo <= hi ? (lo + hi) / 2 : Math.min(t0, t1) > len ? len : 0;
  return toLine(add(a0, scale(u, t)), b0, u);
}

/** Einfache Gestalt eines Merkmals für die Maße. */
type Prim =
  | { t: 'P'; p: Vec3 }
  | { t: 'L'; a: Vec3; b: Vec3; d: Vec3 }
  /** Achse mit Mittelpunkt: Kreis (Normale) oder Zylinder */
  | { t: 'A'; c: Vec3; d: Vec3; cyl: boolean }
  | { t: 'S'; o: Vec3; n: Vec3 };

function prim(f: MFeature): Prim {
  switch (f.kind) {
    case 'point': return { t: 'P', p: f.p };
    case 'edge': return { t: 'L', a: f.a, b: f.b, d: normalize(sub(f.b, f.a)) };
    case 'circle': return { t: 'A', c: f.center, d: f.normal, cyl: false };
    case 'cylinder': return { t: 'A', c: f.center, d: f.axis, cyl: true };
    case 'plane': return { t: 'S', o: f.origin, n: f.normal };
  }
}

const ORDER = { P: 0, L: 1, A: 2, S: 3 } as const;
const flip = (s: Span | undefined): Span | undefined => s && { d: s.d, from: s.to, to: s.from };

/** Maße zwischen zwei Merkmalen (Weltkoordinaten). */
export function measureBetween(fa: MFeature, fb: MFeature): MeasureResult {
  let a = prim(fa);
  let b = prim(fb);
  const swap = ORDER[a.t] > ORDER[b.t];
  if (swap) [a, b] = [b, a];
  const res: MeasureResult = {};
  if (a.t === 'P') {
    if (b.t === 'P') res.direct = { d: dist(a.p, b.p), from: a.p, to: b.p };
    else if (b.t === 'L') {
      res.perpendicular = toLine(a.p, b.a, b.d);
      const s = closestOnSegment(a.p, b.a, b.b);
      res.direct = { d: s.d, from: a.p, to: s.q };
    } else if (b.t === 'A') {
      res.direct = { d: dist(a.p, b.c), from: a.p, to: b.c };
      // Abstand zur Achse (z. B. Wand zur Bohrungsmitte)
      if (b.cyl) res.perpendicular = { ...toLine(a.p, b.c, b.d), axis: true };
    } else res.perpendicular = toPlane(a.p, b.o, b.n);
  } else if (a.t === 'L') {
    if (b.t === 'L') {
      if (parallel(a.d, b.d)) res.perpendicular = parallelSegments(a.a, a.b, b.a, b.b);
      else res.angle = { deg: lineAngle(a.d, b.d) };
      res.direct = segmentSegment(a.a, a.b, b.a, b.b);
    } else if (b.t === 'A') {
      const s = closestOnSegment(b.c, a.a, a.b);
      res.direct = { d: s.d, from: s.q, to: b.c };
      // Von der Mitte senkrecht auf die Kante
      if (parallel(a.d, b.d)) res.perpendicular = { ...flip(toLine(b.c, a.a, a.d))!, axis: true };
      else if (b.cyl) res.angle = { deg: lineAngle(a.d, b.d) };
    } else if (b.t === 'S') {
      res.angle = { deg: 90 - lineAngle(a.d, b.n) };
      if (perpendicularTo(a.d, b.n)) res.perpendicular = toPlane(a.a, b.o, b.n);
    }
  } else if (a.t === 'A' && b.t !== 'S') {
    if (b.t === 'A') {
      res.direct = { d: dist(a.c, b.c), from: a.c, to: b.c };
      if (parallel(a.d, b.d)) res.perpendicular = { ...lineLine(a.c, b.c, b.d), axis: true };
      else res.angle = { deg: lineAngle(a.d, b.d) };
    }
  } else if (a.t === 'A' && b.t === 'S') {
    res.angle = { deg: 90 - lineAngle(a.d, b.n) };
    // Kreis in einer parallelen Ebene, Zylinder parallel zur Fläche: Abstand der Mitte zur Fläche
    if (parallel(a.d, b.n) || perpendicularTo(a.d, b.n)) res.perpendicular = toPlane(a.c, b.o, b.n);
  } else if (a.t === 'S' && b.t === 'S') {
    if (parallel(a.n, b.n)) res.perpendicular = toPlane(a.o, b.o, b.n);
    else {
      const between = deg(Math.acos(Math.max(-1, Math.min(1, dot(a.n, b.n)))));
      res.angle = { deg: between, inner: 180 - between };
    }
  }
  // Senkrechter Abstand gleich dem direkten: nur einmal zeigen (als Abstand)
  if (res.perpendicular && res.direct && Math.abs(res.perpendicular.d - res.direct.d) < 1e-6 && !res.perpendicular.axis) delete res.direct;
  if (res.angle && res.angle.deg < 0.05 && res.angle.inner === undefined) delete res.angle;
  if (swap) {
    if (res.direct) res.direct = flip(res.direct);
    if (res.perpendicular) res.perpendicular = { ...flip(res.perpendicular)!, axis: res.perpendicular.axis };
  }
  return res;
}
