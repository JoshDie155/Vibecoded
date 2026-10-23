/**
 * Gerade Kanten erkennen: Kanten zwischen zwei Dreiecken, die stark abknicken (oder offene Ränder),
 * und von der angeklickten Stelle aus entlang der Geraden verlängert – so wird aus vielen kleinen
 * Kantenstücken eines fein unterteilten Netzes die ganze Kante eines Quaders.
 */
import { type Vec3, add, dot, len, normalize, scale, sub } from './align';
import { type EdgeFeature, type Topology } from './features';

export interface EdgeOptions {
  /** Ab diesem Knick (Grad) zwischen zwei Dreiecken gilt die gemeinsame Kante als scharf. */
  sharpDeg?: number;
  /** Höchstens so weit (Grad) dürfen Kantenstücke einer Geraden voneinander abweichen. */
  straightDeg?: number;
}

interface SharpEdges {
  /** Eckpunkte (verschweißt) und ein angrenzendes Dreieck je scharfer Kante. */
  va: Int32Array;
  vb: Int32Array;
  tri: Int32Array;
  /** Normalen-Summe der angrenzenden Dreiecke je Kante. */
  normal: Float32Array;
  /** Kanten je Eckpunkt (kompakt: start[v] … start[v + 1]). */
  start: Int32Array;
  list: Int32Array;
}

const cache = new WeakMap<Topology, Map<number, SharpEdges>>();

/** Alle scharfen Kanten eines Netzes (einmal je Netz und Schwelle berechnet). */
function sharpEdges(topo: Topology, sharpDeg: number): SharpEdges {
  let byDeg = cache.get(topo);
  if (!byDeg) cache.set(topo, (byDeg = new Map()));
  const hit = byDeg.get(sharpDeg);
  if (hit) return hit;
  const cosMax = Math.cos((sharpDeg * Math.PI) / 180);
  const { triCount, neighbors, normals, vertIds } = topo;
  const va: number[] = [];
  const vb: number[] = [];
  const tri: number[] = [];
  const nrm: number[] = [];
  for (let t = 0; t < triCount; t++) {
    const nx = normals[3 * t], ny = normals[3 * t + 1], nz = normals[3 * t + 2];
    if (nx === 0 && ny === 0 && nz === 0) continue;
    for (let k = 0; k < 3; k++) {
      const n = neighbors[3 * t + k];
      if (n >= 0 && n < t) continue; // Kante schon vom Nachbarn aus gezählt
      const mx = n >= 0 ? normals[3 * n] : 0, my = n >= 0 ? normals[3 * n + 1] : 0, mz = n >= 0 ? normals[3 * n + 2] : 0;
      if (n >= 0 && nx * mx + ny * my + nz * mz > cosMax) continue;
      va.push(vertIds[3 * t + k]);
      vb.push(vertIds[3 * t + ((k + 1) % 3)]);
      tri.push(t);
      nrm.push(nx + mx, ny + my, nz + mz);
    }
  }
  const start = new Int32Array(topo.vertCount + 1);
  for (let e = 0; e < va.length; e++) { start[va[e] + 1]++; start[vb[e] + 1]++; }
  for (let v = 0; v < topo.vertCount; v++) start[v + 1] += start[v];
  const fill = start.slice(0, topo.vertCount);
  const list = new Int32Array(va.length * 2);
  for (let e = 0; e < va.length; e++) { list[fill[va[e]]++] = e; list[fill[vb[e]]++] = e; }
  const res = { va: Int32Array.from(va), vb: Int32Array.from(vb), tri: Int32Array.from(tri), normal: Float32Array.from(nrm), start, list };
  byDeg.set(sharpDeg, res);
  return res;
}

const vert = (topo: Topology, v: number): Vec3 => [topo.vertPos[3 * v], topo.vertPos[3 * v + 1], topo.vertPos[3 * v + 2]];

/** Abstand eines Punktes von der Strecke a–b. */
function segmentDistance(p: Vec3, a: Vec3, b: Vec3): number {
  const ab = sub(b, a);
  const l2 = dot(ab, ab);
  const t = l2 > 0 ? Math.min(1, Math.max(0, dot(sub(p, a), ab) / l2)) : 0;
  return len(sub(p, add(a, scale(ab, t))));
}

/**
 * Gerade Kante nahe der angeklickten Stelle.
 * @param tri angeklicktes Dreieck
 * @param point Trefferpunkt (lokal)
 * @param radius Suchradius um den Trefferpunkt (lokal, z. B. ein paar Bildschirmpixel)
 * @returns die Kante oder null, wenn in der Nähe keine scharfe Kante liegt
 */
export function detectEdge(topo: Topology, tri: number, point: Vec3, radius: number, opts: EdgeOptions = {}): EdgeFeature | null {
  const se = sharpEdges(topo, opts.sharpDeg ?? 25);
  // Dreiecke in der Nähe (Flutfüllung ab dem Treffer, begrenzt durch den Suchradius)
  const seen = new Set<number>([tri]);
  const stack = [tri];
  const verts = new Set<number>();
  while (stack.length && seen.size < 5000) {
    const t = stack.pop()!;
    for (let k = 0; k < 3; k++) verts.add(topo.vertIds[3 * t + k]);
    for (let k = 0; k < 3; k++) {
      const n = topo.neighbors[3 * t + k];
      if (n < 0 || seen.has(n)) continue;
      const c: Vec3 = [topo.centroids[3 * n], topo.centroids[3 * n + 1], topo.centroids[3 * n + 2]];
      // Großzügig: auch große Dreiecke, deren Mitte weiter weg liegt, können eine nahe Kante haben
      const a = vert(topo, topo.vertIds[3 * n]);
      const reach = Math.max(len(sub(a, c)), len(sub(vert(topo, topo.vertIds[3 * n + 1]), c)), len(sub(vert(topo, topo.vertIds[3 * n + 2]), c)));
      if (len(sub(c, point)) > radius + reach) continue;
      seen.add(n);
      stack.push(n);
    }
  }
  // Nächstgelegene scharfe Kante
  let best = -1;
  let bestD = radius;
  for (const v of verts) {
    for (let i = se.start[v]; i < se.start[v + 1]; i++) {
      const e = se.list[i];
      const d = segmentDistance(point, vert(topo, se.va[e]), vert(topo, se.vb[e]));
      if (d < bestD) { bestD = d; best = e; }
    }
  }
  if (best < 0) return null;

  // Entlang der Geraden in beide Richtungen verlängern
  const p0 = vert(topo, se.va[best]);
  const dir = normalize(sub(vert(topo, se.vb[best]), p0));
  const cosStraight = Math.cos(((opts.straightDeg ?? 3) * Math.PI) / 180);
  const tol = Math.max(topo.diag * 1e-5, 1e-6);
  const used = new Set<number>([best]);
  const ends: [number, number] = [se.va[best], se.vb[best]];
  for (const [end, sign] of [[1, 1], [0, -1]] as const) {
    let v = ends[end];
    for (let guard = 0; guard < 100000; guard++) {
      let next = -1;
      let nextV = -1;
      let bestCos = cosStraight;
      for (let i = se.start[v]; i < se.start[v + 1]; i++) {
        const e = se.list[i];
        if (used.has(e)) continue;
        const w = se.va[e] === v ? se.vb[e] : se.va[e];
        const step = sub(vert(topo, w), vert(topo, v));
        const l = len(step);
        if (l === 0) continue;
        const c = (dot(step, dir) * sign) / l;
        // Auf der Geraden bleiben: Abstand des neuen Punktes von der Ausgangsgeraden
        const off = sub(vert(topo, w), p0);
        const dist = len(sub(off, scale(dir, dot(off, dir))));
        if (c > bestCos && dist < tol + l * 0.05) { bestCos = c; next = e; nextV = w; }
      }
      if (next < 0) break;
      used.add(next);
      v = nextV;
    }
    ends[end] = v;
  }
  const a = vert(topo, ends[0]);
  const b = vert(topo, ends[1]);
  const length = len(sub(b, a));
  if (length === 0) return null;
  let n: Vec3 = [0, 0, 0];
  for (const e of used) n = add(n, [se.normal[3 * e], se.normal[3 * e + 1], se.normal[3 * e + 2]]);
  return {
    kind: 'edge',
    tris: [...used].map((e) => se.tri[e]).sort((x, y) => x - y),
    a, b,
    center: scale(add(a, b), 0.5),
    dir: normalize(sub(b, a)),
    length,
    normal: len(n) > 0 ? normalize(n) : [0, 0, 0],
  };
}
