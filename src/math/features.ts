/**
 * Formelemente ("Features") an einer angeklickten Stelle erkennen, ähnlich wie in einem
 * CAD-Programm: ebene Fläche, Zylinderfläche (Zapfen oder Bohrung, auch als Teilbogen wie die
 * Rundung eines D-Profils) oder Freiformfläche. Bei ebenen Flächen wird zusätzlich geprüft, ob
 * der äußere Umriss ein Kreis ist (Scheibe, Stirnfläche eines Zapfens), damit Kreise Mitte auf
 * Mitte verbunden werden können.
 *
 * Grundlage ist die Nachbarschaft der Dreiecke: Eckpunkte werden über ihre Position verschweißt,
 * danach ist für jede Kante bekannt, welches Dreieck auf der anderen Seite liegt. Von der
 * angeklickten Stelle aus wachsen Regionen (Flutfüllung) mit unterschiedlichen Kriterien.
 * Jede Flutfüllung ist auf `maxTris` Dreiecke begrenzt, damit ein Klick auf ein riesiges Netz
 * nie hängt.
 */
import { type Vec3, add, dot, len, normalize, scale, sub } from './align';

export interface Topology {
  triCount: number;
  /** Eingabe, nicht indiziert (9 Werte pro Dreieck). */
  positions: Float32Array;
  /** Flächennormale je Dreieck (Einheitsvektor, 3 pro Dreieck); [0, 0, 0] bei entarteten Dreiecken. */
  normals: Float32Array;
  /** Flächeninhalt je Dreieck. */
  areas: Float32Array;
  /** Schwerpunkt je Dreieck (3 pro Dreieck). */
  centroids: Float32Array;
  /** Nachbar über die Kanten (v0v1, v1v2, v2v0); -1 bei offenem Rand oder mehrdeutiger Kante. */
  neighbors: Int32Array;
  /** Nummer des verschweißten Eckpunkts je Dreiecksecke (3 pro Dreieck). */
  vertIds: Int32Array;
  /** Position je verschweißtem Eckpunkt (3 pro Punkt). */
  vertPos: Float32Array;
  vertCount: number;
  /** Diagonale des umschließenden Quaders (Bezugsgröße für Toleranzen). */
  diag: number;
}

export interface PlaneFeature {
  kind: 'plane';
  tris: number[];
  /** Flächengewichteter Schwerpunkt. */
  centroid: Vec3;
  /** Nach außen zeigende Normale (flächengewichtetes Mittel). */
  normal: Vec3;
  area: number;
  /** Äußere Randschleife der Fläche, geordnet (bei Flächen mit Löchern die äußerste). */
  outline: Vec3[];
  /**
   * Nur gesetzt, wenn der äußere Umriss ein Kreis ist: mindestens 8 Punkte, Abweichung unter 1 %
   * des Radius und keine Lücke zwischen benachbarten Umrisspunkten von `smoothAngleDeg` oder mehr.
   */
  circle?: { center: Vec3; radius: number; rms: number };
  /** Hauptrichtungen in der Ebene (u, v, Normale bilden ein Rechtssystem) und Ausdehnung entlang u und v. */
  axes: { u: Vec3; v: Vec3; size: [number, number] };
}

export interface CylinderFeature {
  kind: 'cylinder';
  tris: number[];
  /** Achsrichtung (Einheitsvektor, betragsgrößte Komponente positiv). */
  axis: Vec3;
  /** Punkt auf der Achse in der Mitte der Mantelfläche. */
  center: Vec3;
  radius: number;
  /** Ausdehnung der Mantelfläche entlang der Achse. */
  length: number;
  /** true: Normalen zeigen zur Achse (Bohrung), false: nach außen (Zapfen, Welle). */
  hole: boolean;
  /** Abgedeckter Winkel um die Achse in Radiant (2π = voller Zylinder). */
  arc: number;
  /** Restabweichung der Eckpunkte vom Zylinder. */
  rms: number;
  area: number;
}

export interface SurfaceFeature {
  kind: 'surface';
  tris: number[];
  centroid: Vec3;
  normal: Vec3;
  area: number;
}

export type Feature = PlaneFeature | CylinderFeature | SurfaceFeature;

export interface FeatureOptions {
  /** Höchstens so weit (Grad) darf eine Dreiecksnormale von der angeklickten Ebene abweichen. */
  planeAngleDeg?: number;
  /**
   * Größter Knickwinkel (Grad) zwischen benachbarten Dreiecken einer glatten Fläche. 35° erkennt
   * auch grob facettierte Zylinder (ab 12 Facetten = 30°), Kanten von 45° und mehr trennen immer.
   */
  smoothAngleDeg?: number;
  /** Obergrenze für die Größe jeder Region (Dreiecke). */
  maxTris?: number;
}

export const FEATURE_DEFAULTS = { planeAngleDeg: 1.5, smoothAngleDeg: 35, maxTris: 200000 } as const;

const DEG = Math.PI / 180;
const TWO_PI = 2 * Math.PI;
/** Zylinder: Normalen weichen höchstens um diesen Winkel von der Senkrechten zur Achse ab. */
const CYL_AXIS_SIN = Math.sin(4 * DEG);
/**
 * Zylinder: mittlere Neigung der Normalen gegen die Senkrechte zur Achse (flächengewichtet).
 * Facetten echter Zylinder stehen exakt parallel zur Achse; ein Band einer Kugel, das die Toleranz
 * oben ausschöpft, kommt dagegen auf mindestens etwa 1,3°.
 */
const CYL_MAX_TILT_SIN = Math.sin(1 * DEG);
/** Zylinder: Normale zeigt ungefähr radial (nach außen bzw. bei Bohrungen nach innen). */
const CYL_RADIAL_COS = Math.cos(30 * DEG);
/** Startregion der Zylinderschätzung: Normalen innerhalb dieses Winkels zur Klick-Normale. */
const CYL_LOCAL_COS = Math.cos(35 * DEG);
/** Zylinder: Toleranz beim Wachsen (relativ zum Radius), damit auch leicht verrauschte Netze zusammenhängen. */
const CYL_GROW_TOL = 0.02;
/**
 * Zylinder: erlaubte Restabweichung der fertigen Region (relativ zum Radius). Eckpunkte echter
 * Zylinder liegen exakt auf dem Mantel; Streifen von Kegeln oder Kugeln, die nur bis an die
 * Wachstumstoleranz passen, liegen deutlich darüber (typisch 0,3–0,7 %).
 */
const CYL_MAX_RMS = 0.002;
const CYL_MIN_ARC = 45 * DEG;
/** Achsvorschläge aus Kreuzprodukten: so viele nahe Dreiecke ansehen, höchstens so viele Vorschläge. */
const CAND_NEAR = 48;
const CAND_MAX = 8;
const CAND_SAME_COS = Math.cos(3 * DEG);
const CIRCLE_MAX_RMS = 0.01;
const CIRCLE_MIN_POINTS = 8;
const SURFACE_COS = Math.cos(15 * DEG);
const ARC_BINS = 720;

const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

/** Vorzeichen so wählen, dass die betragsgrößte Komponente positiv ist (deterministisch). */
function canonical(v: Vec3): Vec3 {
  let i = 0;
  if (Math.abs(v[1]) > Math.abs(v[i])) i = 1;
  if (Math.abs(v[2]) > Math.abs(v[i])) i = 2;
  return v[i] < 0 ? scale(v, -1) : v;
}

/** Zwei Einheitsvektoren e1, e2 senkrecht zu n mit e1 × e2 = n. */
function basis(n: Vec3): [Vec3, Vec3] {
  const h: Vec3 = Math.abs(n[0]) < 0.6 ? [1, 0, 0] : Math.abs(n[1]) < 0.6 ? [0, 1, 0] : [0, 0, 1];
  const e1 = normalize(cross(h, n));
  return [e1, cross(n, e1)];
}

// ---------------------------------------------------------------------------------------------
// Topologie
// ---------------------------------------------------------------------------------------------

/** Mischt Zellkoordinaten zu einem 32-Bit-Hash. */
function cellHash(ix: number, iy: number, iz: number): number {
  let h = Math.imul(ix | 0, 0x8da6b343) ^ Math.imul(iy | 0, 0xd8163841) ^ Math.imul(iz | 0, 0xcb1ab31f);
  h ^= h >>> 16;
  h = Math.imul(h, 0x85ebca6b);
  h ^= h >>> 13;
  h = Math.imul(h, 0xc2b2ae35);
  h ^= h >>> 16;
  return h >>> 0;
}

/**
 * Verschweißt Ecken, die höchstens `tol` voneinander entfernt sind (Raster mit Zellgröße 2·tol in
 * einer offenen Hashtabelle). Bitgleiche Punkte werden mit einem Zugriff gefunden; nur für neue
 * Punkte werden auch die Nachbarzellen geprüft, in die ein naher Punkt fallen könnte.
 */
function weld(positions: Float32Array, corners: number, tol: number) {
  const vertIds = new Int32Array(corners);
  const firstCorner = new Int32Array(corners);
  let size = 16;
  while (size < corners * 1.5) size *= 2;
  const mask = size - 1;
  const table = new Int32Array(size).fill(-1);
  const inv = 1 / (2 * tol);
  const tol2 = tol * tol;
  let count = 0;

  const find = (h: number, x: number, y: number, z: number): number => {
    for (let s = h; ; s = (s + 1) & mask) {
      const v = table[s];
      if (v < 0) return -1;
      const o = 3 * firstCorner[v];
      const dx = positions[o] - x;
      const dy = positions[o + 1] - y;
      const dz = positions[o + 2] - z;
      if (dx * dx + dy * dy + dz * dz <= tol2) return v;
    }
  };

  for (let c = 0; c < corners; c++) {
    const x = positions[3 * c];
    const y = positions[3 * c + 1];
    const z = positions[3 * c + 2];
    if (!Number.isFinite(x + y + z)) {
      // Ungültige Koordinaten nie verschweißen (und nicht in die Tabelle, sonst lange Ketten)
      firstCorner[count] = c;
      vertIds[c] = count++;
      continue;
    }
    const fx = x * inv;
    const fy = y * inv;
    const fz = z * inv;
    const ix = Math.floor(fx);
    const iy = Math.floor(fy);
    const iz = Math.floor(fz);
    const h = cellHash(ix, iy, iz) & mask;
    let v = find(h, x, y, z);
    if (v < 0) {
      // Ein Punkt innerhalb der Toleranz liegt höchstens eine halbe Zelle entfernt
      const sx = fx - ix < 0.5 ? -1 : 1;
      const sy = fy - iy < 0.5 ? -1 : 1;
      const sz = fz - iz < 0.5 ? -1 : 1;
      for (let m = 1; m < 8 && v < 0; m++) {
        const hm = cellHash(ix + (m & 1 ? sx : 0), iy + (m & 2 ? sy : 0), iz + (m & 4 ? sz : 0)) & mask;
        v = find(hm, x, y, z);
      }
    }
    if (v < 0) {
      v = count++;
      firstCorner[v] = c;
      let s = h;
      while (table[s] >= 0) s = (s + 1) & mask;
      table[s] = v;
    }
    vertIds[c] = v;
  }
  return { vertIds, vertCount: count, firstCorner };
}

/**
 * Baut die Nachbarschaft der Dreiecke auf (Laufzeit linear in der Dreieckszahl).
 * @param weldTol Eckpunkte mit kleinerem Abstand gelten als gleich (Standard: 1e-5 · Diagonale)
 */
export function buildTopology(positions: Float32Array, weldTol?: number): Topology {
  const triCount = Math.floor(positions.length / 9);
  const corners = triCount * 3;

  let minX = Infinity;
  let minY = Infinity;
  let minZ = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  let maxZ = -Infinity;
  for (let i = 0; i < corners * 3; i += 3) {
    const x = positions[i];
    const y = positions[i + 1];
    const z = positions[i + 2];
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
  }
  const d = Math.hypot(maxX - minX, maxY - minY, maxZ - minZ);
  const diag = triCount > 0 && Number.isFinite(d) ? d : 0;
  const tol = weldTol !== undefined && weldTol > 0 ? weldTol : Math.max(diag * 1e-5, 1e-9);

  const { vertIds, vertCount, firstCorner } = weld(positions, corners, tol);
  const vertPos = new Float32Array(vertCount * 3);
  for (let v = 0; v < vertCount; v++) {
    const o = 3 * firstCorner[v];
    vertPos[3 * v] = positions[o];
    vertPos[3 * v + 1] = positions[o + 1];
    vertPos[3 * v + 2] = positions[o + 2];
  }

  // Flächennormalen, Flächeninhalte und Schwerpunkte
  const normals = new Float32Array(corners);
  const areas = new Float32Array(triCount);
  const centroids = new Float32Array(corners);
  // Dreiecke, die flacher als diese Höhe sind, haben keine verlässliche Normale
  const minHeight = Math.max(diag * 1e-7, 1e-12);
  const collapsed = (t: number) =>
    vertIds[3 * t] === vertIds[3 * t + 1] || vertIds[3 * t + 1] === vertIds[3 * t + 2] || vertIds[3 * t] === vertIds[3 * t + 2];
  for (let t = 0; t < triCount; t++) {
    const o = 9 * t;
    const ax = positions[o], ay = positions[o + 1], az = positions[o + 2];
    const bx = positions[o + 3], by = positions[o + 4], bz = positions[o + 5];
    const cx = positions[o + 6], cy = positions[o + 7], cz = positions[o + 8];
    // n = (c - b) × (a - b)
    const ux = cx - bx, uy = cy - by, uz = cz - bz;
    const vx = ax - bx, vy = ay - by, vz = az - bz;
    const nx = uy * vz - uz * vy;
    const ny = uz * vx - ux * vz;
    const nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz);
    areas[t] = Number.isFinite(l) ? l / 2 : 0;
    centroids[3 * t] = (ax + bx + cx) / 3;
    centroids[3 * t + 1] = (ay + by + cy) / 3;
    centroids[3 * t + 2] = (az + bz + cz) / 3;
    const wx = ax - cx, wy = ay - cy, wz = az - cz;
    const maxEdge = Math.sqrt(Math.max(ux * ux + uy * uy + uz * uz, vx * vx + vy * vy + vz * vz, wx * wx + wy * wy + wz * wz));
    if (!collapsed(t) && l > minHeight * maxEdge && Number.isFinite(l)) {
      normals[3 * t] = nx / l;
      normals[3 * t + 1] = ny / l;
      normals[3 * t + 2] = nz / l;
    }
  }

  // Dreiecke je Eckpunkt (kompakte Listen); zusammengefallene Dreiecke bleiben außen vor
  const start = new Int32Array(vertCount + 1);
  for (let t = 0; t < triCount; t++) {
    if (collapsed(t)) continue;
    for (let k = 0; k < 3; k++) start[vertIds[3 * t + k] + 1]++;
  }
  for (let v = 0; v < vertCount; v++) start[v + 1] += start[v];
  const fill = start.slice(0, vertCount);
  const inc = new Int32Array(start[vertCount]);
  for (let t = 0; t < triCount; t++) {
    if (collapsed(t)) continue;
    for (let k = 0; k < 3; k++) inc[fill[vertIds[3 * t + k]]++] = t;
  }

  // Nachbar über jede Kante: das einzige andere Dreieck, das beide Endpunkte enthält
  const neighbors = new Int32Array(corners).fill(-1);
  for (let t = 0; t < triCount; t++) {
    if (collapsed(t)) continue;
    for (let k = 0; k < 3; k++) {
      const u = vertIds[3 * t + k];
      const w = vertIds[3 * t + (k === 2 ? 0 : k + 1)];
      // Liste des Eckpunkts mit weniger Dreiecken durchsuchen (Fächer mit vielen Dreiecken)
      const uSmaller = start[u + 1] - start[u] <= start[w + 1] - start[w];
      const x = uSmaller ? u : w;
      const y = uSmaller ? w : u;
      let found = -1;
      let count = 0;
      for (let i = start[x]; i < start[x + 1]; i++) {
        const s = inc[i];
        if (s === t) continue;
        if (vertIds[3 * s] === y || vertIds[3 * s + 1] === y || vertIds[3 * s + 2] === y) {
          count++;
          found = s;
        }
      }
      if (count === 1) neighbors[3 * t + k] = found;
    }
  }

  return { triCount, positions, normals, areas, centroids, neighbors, vertIds, vertPos, vertCount, diag };
}

// ---------------------------------------------------------------------------------------------
// Hilfsfunktionen für Regionen
// ---------------------------------------------------------------------------------------------

/** Markierungsfelder für Flutfüllungen, je Topologie einmal angelegt und wiederverwendet. */
interface Scratch {
  visit: Int32Array;
  ref: Int32Array;
  member: Int32Array;
  vmark: Int32Array;
  /** Fortlaufender Zähler: jede Markierung bekommt einen neuen Wert, alte werden so ungültig. */
  stamp: number;
}

interface Ctx {
  topo: Topology;
  limit: number;
  s: Scratch;
}

const scratchCache = new WeakMap<Topology, Scratch>();

function makeCtx(topo: Topology, limit: number): Ctx {
  let s = scratchCache.get(topo);
  if (!s || s.stamp > 2e9) {
    s = {
      visit: new Int32Array(topo.triCount),
      ref: new Int32Array(topo.triCount),
      member: new Int32Array(topo.triCount),
      vmark: new Int32Array(topo.vertCount),
      stamp: 0,
    };
    scratchCache.set(topo, s);
  }
  return { topo, limit, s };
}

const isDegenerate = (topo: Topology, t: number) =>
  topo.normals[3 * t] === 0 && topo.normals[3 * t + 1] === 0 && topo.normals[3 * t + 2] === 0;
const normalOf = (topo: Topology, t: number): Vec3 => [topo.normals[3 * t], topo.normals[3 * t + 1], topo.normals[3 * t + 2]];
const centroidOf = (topo: Topology, t: number): Vec3 => [topo.centroids[3 * t], topo.centroids[3 * t + 1], topo.centroids[3 * t + 2]];
const vertexOf = (topo: Topology, v: number): Vec3 => [topo.vertPos[3 * v], topo.vertPos[3 * v + 1], topo.vertPos[3 * v + 2]];
const ndot = (topo: Topology, s: number, t: number) => {
  const n = topo.normals;
  return n[3 * s] * n[3 * t] + n[3 * s + 1] * n[3 * t + 1] + n[3 * s + 2] * n[3 * t + 2];
};
const ndotv = (topo: Topology, t: number, v: Vec3) =>
  topo.normals[3 * t] * v[0] + topo.normals[3 * t + 1] * v[1] + topo.normals[3 * t + 2] * v[2];

interface Region {
  /** Dreiecke in Reihenfolge der Breitensuche (nahe am Startdreieck zuerst). */
  tris: number[];
  /** Markierung in `visit`, gültig bis zur nächsten Flutfüllung. */
  stamp: number;
}

/**
 * Breitensuche über Kantennachbarn. `accept(ref, to)` entscheidet, ob `to` dazugehört; `ref` ist
 * das letzte nicht entartete Dreieck auf dem Weg (entartete Dreiecke haben keine Normale und
 * werden nur durchquert).
 */
function flood(ctx: Ctx, seed: number, limit: number, accept: (ref: number, to: number) => boolean): Region {
  const { neighbors } = ctx.topo;
  const { visit, ref } = ctx.s;
  const stamp = ++ctx.s.stamp;
  const tris = [seed];
  visit[seed] = stamp;
  ref[seed] = seed;
  for (let i = 0; i < tris.length && tris.length < limit; i++) {
    const t = tris[i];
    const r = ref[t];
    for (let k = 0; k < 3; k++) {
      const nb = neighbors[3 * t + k];
      if (nb < 0 || visit[nb] === stamp || !accept(r, nb)) continue;
      visit[nb] = stamp;
      ref[nb] = isDegenerate(ctx.topo, nb) ? r : nb;
      tris.push(nb);
      if (tris.length >= limit) break;
    }
  }
  return { tris, stamp };
}

/** Markiert eine Dreiecksmenge in `member` und gibt die Markierung zurück. */
function markMembers(ctx: Ctx, tris: number[]): number {
  const stamp = ++ctx.s.stamp;
  for (const t of tris) ctx.s.member[t] = stamp;
  return stamp;
}

/** Verschweißte Eckpunkte einer Region, jeder nur einmal. */
function regionVertices(ctx: Ctx, tris: number[]): number[] {
  const { vertIds } = ctx.topo;
  const { vmark } = ctx.s;
  const stamp = ++ctx.s.stamp;
  const out: number[] = [];
  for (const t of tris) {
    for (let k = 0; k < 3; k++) {
      const v = vertIds[3 * t + k];
      if (vmark[v] !== stamp) {
        vmark[v] = stamp;
        out.push(v);
      }
    }
  }
  return out;
}

function areaOf(topo: Topology, tris: number[]): number {
  let a = 0;
  for (const t of tris) a += topo.areas[t];
  return a;
}

/** Flächeninhalt, flächengewichteter Schwerpunkt und mittlere Normale einer Region. */
function regionStats(topo: Topology, tris: number[]): { area: number; centroid: Vec3; normal: Vec3 } {
  const { areas, centroids, normals } = topo;
  let A = 0;
  let cx = 0, cy = 0, cz = 0;
  let nx = 0, ny = 0, nz = 0;
  for (const t of tris) {
    const a = areas[t];
    A += a;
    cx += a * centroids[3 * t];
    cy += a * centroids[3 * t + 1];
    cz += a * centroids[3 * t + 2];
    nx += a * normals[3 * t];
    ny += a * normals[3 * t + 1];
    nz += a * normals[3 * t + 2];
  }
  const centroid: Vec3 = A > 0 ? [cx / A, cy / A, cz / A] : centroidOf(topo, tris[0]);
  return { area: A, centroid, normal: normalize([nx, ny, nz]) };
}

// ---------------------------------------------------------------------------------------------
// Lineare Algebra und Kreisanpassung
// ---------------------------------------------------------------------------------------------

/** Eigenwerte (aufsteigend) und Eigenvektoren einer symmetrischen 3x3-Matrix (zyklisches Jacobi-Verfahren). */
function eigenSym3(m: number[]): { values: Vec3; vectors: [Vec3, Vec3, Vec3] } {
  const A = [
    [m[0], m[1], m[2]],
    [m[3], m[4], m[5]],
    [m[6], m[7], m[8]],
  ];
  const V = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
  let norm = 0;
  for (const x of m) norm += x * x;
  for (let sweep = 0; sweep < 50; sweep++) {
    const off = A[0][1] * A[0][1] + A[0][2] * A[0][2] + A[1][2] * A[1][2];
    if (!(off > 1e-30 * norm)) break;
    for (let p = 0; p < 3; p++) {
      for (let q = p + 1; q < 3; q++) {
        if (Math.abs(A[p][q]) < 1e-300) continue;
        const theta = (A[q][q] - A[p][p]) / (2 * A[p][q]);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < 3; k++) {
          const akp = A[k][p];
          const akq = A[k][q];
          A[k][p] = c * akp - s * akq;
          A[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < 3; k++) {
          const apk = A[p][k];
          const aqk = A[q][k];
          A[p][k] = c * apk - s * aqk;
          A[q][k] = s * apk + c * aqk;
        }
        for (let k = 0; k < 3; k++) {
          const vkp = V[k][p];
          const vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq;
          V[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  const order = [0, 1, 2].sort((i, j) => A[i][i] - A[j][j]);
  const vec = (j: number): Vec3 => [V[0][j], V[1][j], V[2][j]];
  return {
    values: [A[order[0]][order[0]], A[order[1]][order[1]], A[order[2]][order[2]]],
    vectors: [vec(order[0]), vec(order[1]), vec(order[2])],
  };
}

/** Löst ein 3x3-Gleichungssystem (Cramersche Regel), null wenn (fast) singulär. */
function solve3(m: number[], b: Vec3): Vec3 | null {
  const det =
    m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
  const scaleRef = Math.abs(m[0] * m[4] * m[8]) + 1e-300;
  if (!(Math.abs(det) > 1e-14 * scaleRef)) return null;
  const x =
    (b[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (b[1] * m[8] - m[5] * b[2]) + m[2] * (b[1] * m[7] - m[4] * b[2])) / det;
  const y =
    (m[0] * (b[1] * m[8] - m[5] * b[2]) - b[0] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * b[2] - b[1] * m[6])) / det;
  const z =
    (m[0] * (m[4] * b[2] - b[1] * m[7]) - m[1] * (m[3] * b[2] - b[1] * m[6]) + b[0] * (m[3] * m[7] - m[4] * m[6])) / det;
  return [x, y, z];
}

interface Circle2 {
  cx: number;
  cy: number;
  r: number;
  rms: number;
}

function circleRms(xs: ArrayLike<number>, ys: ArrayLike<number>, cx: number, cy: number, r: number): number {
  let s = 0;
  for (let i = 0; i < xs.length; i++) {
    const e = Math.hypot(xs[i] - cx, ys[i] - cy) - r;
    s += e * e;
  }
  return Math.sqrt(s / xs.length);
}

/**
 * Kreis durch Punkte in der Ebene: algebraische Lösung (Kåsa, um den Schwerpunkt zentriert),
 * danach geometrisch verfeinert (Gauß-Newton auf die Abstände zum Kreis).
 */
function fitCircle2D(xs: ArrayLike<number>, ys: ArrayLike<number>): Circle2 | null {
  const n = xs.length;
  if (n < 3) return null;
  let mx = 0;
  let my = 0;
  for (let i = 0; i < n; i++) {
    mx += xs[i];
    my += ys[i];
  }
  mx /= n;
  my /= n;
  let suu = 0, suv = 0, svv = 0, suuu = 0, svvv = 0, suvv = 0, svuu = 0;
  for (let i = 0; i < n; i++) {
    const u = xs[i] - mx;
    const v = ys[i] - my;
    const uu = u * u;
    const vv = v * v;
    suu += uu;
    suv += u * v;
    svv += vv;
    suuu += uu * u;
    svvv += vv * v;
    suvv += u * vv;
    svuu += v * uu;
  }
  const det = suu * svv - suv * suv;
  // Punkte (fast) auf einer Geraden: kein Kreis bestimmbar
  if (!(det > 1e-12 * (suu + svv) * (suu + svv))) return null;
  const b1 = 0.5 * (suuu + suvv);
  const b2 = 0.5 * (svvv + svuu);
  const uc = (b1 * svv - b2 * suv) / det;
  const vc = (b2 * suu - b1 * suv) / det;
  let cx = uc + mx;
  let cy = vc + my;
  let r = Math.sqrt(uc * uc + vc * vc + (suu + svv) / n);
  let rms = circleRms(xs, ys, cx, cy, r);

  for (let it = 0; it < 10 && rms > 1e-12 * r; it++) {
    const J = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    const g: Vec3 = [0, 0, 0];
    for (let i = 0; i < n; i++) {
      const dx = xs[i] - cx;
      const dy = ys[i] - cy;
      const d = Math.hypot(dx, dy);
      if (d < 1e-12) continue;
      const j0 = -dx / d;
      const j1 = -dy / d;
      const e = d - r;
      J[0] += j0 * j0; J[1] += j0 * j1; J[2] -= j0;
      J[4] += j1 * j1; J[5] -= j1;
      J[8] += 1;
      g[0] -= j0 * e; g[1] -= j1 * e; g[2] += e;
    }
    J[3] = J[1]; J[6] = J[2]; J[7] = J[5];
    const step = solve3(J, g);
    if (!step) break;
    const ncx = cx + step[0];
    const ncy = cy + step[1];
    const nr = r + step[2];
    if (!(nr > 0)) break;
    const nrms = circleRms(xs, ys, ncx, ncy, nr);
    if (!(nrms < rms)) break;
    const gain = rms - nrms;
    cx = ncx;
    cy = ncy;
    r = nr;
    rms = nrms;
    // Kaum noch Verbesserung: fertig
    if (gain < 1e-3 * rms || Math.hypot(step[0], step[1], step[2]) < 1e-12 * r) break;
  }
  return { cx, cy, r, rms };
}

/** Median (bei vielen Werten aus einer gleichmäßigen Stichprobe, das genügt als Schwelle). */
function median(values: Float64Array): number {
  const step = Math.max(1, Math.floor(values.length / 2048));
  const s = (step > 1 ? values.filter((_, i) => i % step === 0) : values.slice()).sort();
  return s.length ? s[s.length >> 1] : 0;
}

/** Kreisanpassung mit Ausreißerbehandlung: weit abliegende Punkte werden schrittweise verworfen. */
function robustCircle(xs: Float64Array, ys: Float64Array): Circle2 | null {
  let c = fitCircle2D(xs, ys);
  let px = xs;
  let py = ys;
  for (let it = 0; it < 3; it++) {
    if (!c || c.rms <= 1e-3 * c.r) break;
    const res = new Float64Array(px.length);
    for (let i = 0; i < px.length; i++) res[i] = Math.abs(Math.hypot(px[i] - c.cx, py[i] - c.cy) - c.r);
    const thr = Math.max(3 * median(res), 1e-3 * c.r);
    let count = 0;
    for (let i = 0; i < res.length; i++) if (res[i] <= thr) count++;
    if (count < 3 || count === px.length) break;
    const qx = new Float64Array(count);
    const qy = new Float64Array(count);
    for (let i = 0, j = 0; i < res.length; i++) {
      if (res[i] > thr) continue;
      qx[j] = px[i];
      qy[j++] = py[i];
    }
    const c2 = fitCircle2D(qx, qy);
    if (!c2) break;
    c = c2;
    px = qx;
    py = qy;
  }
  return c;
}

// ---------------------------------------------------------------------------------------------
// Zylinder
// ---------------------------------------------------------------------------------------------

interface CylModel {
  axis: Vec3;
  e1: Vec3;
  e2: Vec3;
  /** Punkt auf der Achse. */
  c0: Vec3;
  r: number;
  rms: number;
  hole: boolean;
  /** cos der größten erlaubten Winkelbreite eines Dreiecks um die Achse. */
  spanCos: number;
  /** Mittlerer Eigenwert der Normalen-Streumatrix relativ zur Spur (0 = alle Normalen gleich). */
  spread: number;
  /** Mittleres Quadrat von n·Achse (flächengewichtet), 0 bei einem exakten Zylinder. */
  tilt2: number;
}

/** Flächengewichtete Streumatrix Σ A·n·nᵀ der Normalen (zeilenweise 3x3). */
function normalScatter(topo: Topology, tris: number[]): number[] {
  const { normals, areas } = topo;
  let xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
  for (const t of tris) {
    const a = areas[t];
    const x = normals[3 * t];
    const y = normals[3 * t + 1];
    const z = normals[3 * t + 2];
    xx += a * x * x; xy += a * x * y; xz += a * x * z;
    yy += a * y * y; yz += a * y * z; zz += a * z * z;
  }
  return [xx, xy, xz, xy, yy, yz, xz, yz, zz];
}

/** Zwischenspeicher: Lage eines Punkts im Zylindersystem (x, y senkrecht zur Achse, h entlang der Achse). */
const LOC = new Float64Array(3);

/** Rechnet den Punkt arr[o..o+2] ins Zylindersystem um (Ergebnis in LOC, ohne Speicheranforderung). */
function toCyl(m: CylModel, arr: ArrayLike<number>, o: number): Float64Array {
  const dx = arr[o] - m.c0[0];
  const dy = arr[o + 1] - m.c0[1];
  const dz = arr[o + 2] - m.c0[2];
  LOC[0] = dx * m.e1[0] + dy * m.e1[1] + dz * m.e1[2];
  LOC[1] = dx * m.e2[0] + dy * m.e2[1] + dz * m.e2[2];
  LOC[2] = dx * m.axis[0] + dy * m.axis[1] + dz * m.axis[2];
  return LOC;
}

/** Radiale Komponente der Normale von t am Schwerpunkt (positiv = von der Achse weg); NaN auf der Achse. */
function radialNormal(topo: Topology, t: number, m: CylModel): number {
  const { normals } = topo;
  const p = toCyl(m, topo.centroids, 3 * t);
  const rho = Math.hypot(p[0], p[1]);
  if (rho < 1e-12) return NaN;
  const nx = normals[3 * t], ny = normals[3 * t + 1], nz = normals[3 * t + 2];
  const n1 = nx * m.e1[0] + ny * m.e1[1] + nz * m.e1[2];
  const n2 = nx * m.e2[0] + ny * m.e2[1] + nz * m.e2[2];
  return (n1 * p[0] + n2 * p[1]) / rho;
}

/** Zwischenspeicher für radiale Einheitsvektoren der drei Ecken eines Dreiecks. */
const RAD = new Float64Array(6);

/**
 * Prüft die drei Ecken gegen den Zylinder (Abstand zur Achse ≈ Radius).
 * Gibt -2 zurück, wenn eine Ecke nicht auf dem Mantel liegt, sonst cos der Winkelbreite des Dreiecks.
 */
function cornersOnCylinder(topo: Topology, t: number, m: CylModel, tolR: number): number {
  const { vertIds, vertPos } = topo;
  for (let k = 0; k < 3; k++) {
    const p = toCyl(m, vertPos, 3 * vertIds[3 * t + k]);
    const rho = Math.hypot(p[0], p[1]);
    if (!(Math.abs(rho - m.r) <= tolR) || rho < 1e-12) return -2;
    RAD[2 * k] = p[0] / rho;
    RAD[2 * k + 1] = p[1] / rho;
  }
  return Math.min(
    RAD[0] * RAD[2] + RAD[1] * RAD[3],
    RAD[2] * RAD[4] + RAD[3] * RAD[5],
    RAD[0] * RAD[4] + RAD[1] * RAD[5],
  );
}

/** Liegt das Dreieck auf dem Zylinder (Ecken auf dem Mantel, Normale radial, schmal genug)? */
function onCylinder(topo: Topology, t: number, m: CylModel, tolR: number): boolean {
  const span = cornersOnCylinder(topo, t, m, tolR);
  if (span < -1) return false;
  if (isDegenerate(topo, t)) return true;
  if (Math.abs(ndotv(topo, t, m.axis)) > CYL_AXIS_SIN) return false;
  const nr = radialNormal(topo, t, m);
  if (!((m.hole ? -nr : nr) >= CYL_RADIAL_COS)) return false;
  // Begrenzte Winkelbreite schließt z. B. die Abflachung eines D-Profils aus, deren Ecken auch auf dem Kreis liegen
  return span >= m.spanCos;
}

/**
 * Passt einen Zylinder an eine Dreiecksmenge an: Achse aus der Normalenstreuung (die Normalen
 * eines Zylinders stehen senkrecht zur Achse, also Richtung mit der kleinsten Streuung), danach
 * Kreis durch die in die Ebene senkrecht zur Achse projizierten Eckpunkte.
 */
function fitCylinderModel(ctx: Ctx, tris: number[], axisGuess: Vec3, maxSpan: number): CylModel | null {
  const topo = ctx.topo;
  const M = normalScatter(topo, tris);
  const { values, vectors } = eigenSym3(M);
  const trace = values[0] + values[1] + values[2];
  if (!(trace > 0)) return null;
  const spread = values[1] / trace;
  // Bei zu kleiner Streuung (sehr schmaler Streifen) ist die Achse unbestimmt: Vorschlag behalten
  let axis = axisGuess;
  if (spread > 2e-3) axis = dot(vectors[0], axisGuess) < 0 ? scale(vectors[0], -1) : vectors[0];
  const Ma: Vec3 = [
    M[0] * axis[0] + M[1] * axis[1] + M[2] * axis[2],
    M[3] * axis[0] + M[4] * axis[1] + M[5] * axis[2],
    M[6] * axis[0] + M[7] * axis[1] + M[8] * axis[2],
  ];
  const [e1, e2] = basis(axis);

  const verts = regionVertices(ctx, tris);
  if (verts.length < 3) return null;
  // Erst im Hilfssystem mit Ursprung im ersten Eckpunkt rechnen, danach den Kreismittelpunkt übernehmen
  const model: CylModel = {
    axis, e1, e2, c0: vertexOf(topo, verts[0]), r: 0, rms: 0, hole: false, spanCos: -1, spread, tilt2: dot(axis, Ma) / trace,
  };
  const xs = new Float64Array(verts.length);
  const ys = new Float64Array(verts.length);
  for (let i = 0; i < verts.length; i++) {
    const p = toCyl(model, topo.vertPos, 3 * verts[i]);
    xs[i] = p[0];
    ys[i] = p[1];
  }
  const c = robustCircle(xs, ys);
  if (!c || !(c.r > 0)) return null;
  model.c0 = add(model.c0, add(scale(e1, c.cx), scale(e2, c.cy)));
  model.r = c.r;
  model.rms = c.rms;

  // Zapfen (Normalen nach außen) oder Bohrung (zur Achse hin); typische Winkelbreite der Dreiecke
  let s = 0;
  const spans: number[] = [];
  const step = Math.max(1, Math.floor(tris.length / 1000));
  for (let i = 0; i < tris.length; i++) {
    const t = tris[i];
    if (isDegenerate(topo, t)) continue;
    const nr = radialNormal(topo, t, model);
    if (Number.isNaN(nr)) continue;
    s += topo.areas[t] * nr;
    if (i % step === 0) {
      const sc = cornersOnCylinder(topo, t, model, Infinity);
      if (sc >= -1) spans.push(Math.acos(Math.min(1, sc)));
    }
  }
  model.hole = s < 0;
  const typical = spans.length ? median(Float64Array.from(spans)) : maxSpan;
  model.spanCos = Math.cos(Math.min(maxSpan, Math.max(4 * typical, 2 * DEG)));
  return model;
}

/**
 * Vorschläge für die Zylinderachse: aus der Normalenstreuung der ganzen glatten Region (sauberer
 * Zylinder) und aus Kreuzprodukten der Klick-Normale mit Normalen naher Dreiecke (lokale
 * Krümmungsrichtung, hilft wenn die glatte Region über Verrundungen in andere Flächen übergeht).
 */
function axisCandidates(topo: Topology, seed: number, smooth: number[]): Vec3[] {
  const out: Vec3[] = [];
  const push = (v: Vec3) => {
    const l = len(v);
    if (!(l > 1e-9)) return;
    const a = scale(v, 1 / l);
    if (out.some((b) => Math.abs(dot(a, b)) > CAND_SAME_COS)) return;
    out.push(a);
  };
  push(eigenSym3(normalScatter(topo, smooth)).vectors[0]);
  const n0 = normalOf(topo, seed);
  const cosMin = Math.cos(DEG);
  for (let i = 1; i < smooth.length && i < CAND_NEAR && out.length < CAND_MAX; i++) {
    const t = smooth[i];
    if (isDegenerate(topo, t)) continue;
    const c = ndot(topo, seed, t);
    if (c > cosMin || c < 0.5) continue;
    push(cross(n0, normalOf(topo, t)));
  }
  return out;
}

const wrapAngle = (a: number) => a - TWO_PI * Math.round(a / TWO_PI);
const mod = (i: number, n: number) => ((i % n) + n) % n;

/** Kennzahlen der fertigen Zylinderregion; null, wenn sie kein überzeugender Zylinder ist. */
function finishCylinder(ctx: Ctx, region: Region, m: CylModel, inSmooth: number, planeArea: number): CylinderFeature | null {
  const topo = ctx.topo;
  const { vertIds, vertPos, neighbors, areas } = topo;
  const { tris, stamp } = region;

  // Restabweichung und Ausdehnung entlang der Achse
  const verts = regionVertices(ctx, tris);
  let s2 = 0;
  let hMin = Infinity;
  let hMax = -Infinity;
  for (const v of verts) {
    const p = toCyl(m, vertPos, 3 * v);
    const e = Math.hypot(p[0], p[1]) - m.r;
    s2 += e * e;
    if (p[2] < hMin) hMin = p[2];
    if (p[2] > hMax) hMax = p[2];
  }
  const rms = Math.sqrt(s2 / verts.length);

  // Abgedeckter Winkel: feine Abschnitte, deren Mitte im Winkelintervall eines Dreiecks liegt
  const arcW = TWO_PI / ARC_BINS;
  const arcBins = new Uint8Array(ARC_BINS);
  const th = [0, 0, 0];
  let area = 0;
  for (const t of tris) {
    area += areas[t];
    for (let k = 0; k < 3; k++) {
      const p = toCyl(m, vertPos, 3 * vertIds[3 * t + k]);
      th[k] = Math.atan2(p[1], p[0]);
    }
    const d1 = wrapAngle(th[1] - th[0]);
    const d2 = wrapAngle(th[2] - th[0]);
    const a0 = th[0] + Math.min(0, d1, d2);
    const a1 = th[0] + Math.max(0, d1, d2);
    let b0 = Math.ceil(a0 / arcW - 0.5);
    let b1 = Math.floor(a1 / arcW - 0.5);
    if (b1 < b0) b0 = b1 = Math.floor((a0 + a1) / 2 / arcW);
    for (let b = b0; b <= b1; b++) arcBins[mod(b, ARC_BINS)] = 1;
  }
  let covered = 0;
  for (let b = 0; b < ARC_BINS; b++) covered += arcBins[b];
  const arc = covered * arcW;

  // Rand der Region: Anteil, an dem die glatte Fläche weitergeht ("weich"). Für weiche Randkanten
  // quer zur Achse außerdem: Abstand von den Enden des Zylinders, an welchem Ende sie liegen und
  // wie hoch (entlang der Achse) die angrenzenden Dreiecke sind.
  let soft = 0;
  let edge = 0;
  let devSum = 0;
  let devLen = 0;
  let softLo = 0;
  let softHi = 0;
  let nbHeight = 0;
  const axial = (v: number) => toCyl(m, vertPos, 3 * v)[2];
  for (const t of tris) {
    for (let k = 0; k < 3; k++) {
      const nb = neighbors[3 * t + k];
      if (nb >= 0 && ctx.s.visit[nb] === stamp) continue;
      const u = vertIds[3 * t + k];
      const w = vertIds[3 * t + (k === 2 ? 0 : k + 1)];
      const ex = vertPos[3 * w] - vertPos[3 * u];
      const ey = vertPos[3 * w + 1] - vertPos[3 * u + 1];
      const ez = vertPos[3 * w + 2] - vertPos[3 * u + 2];
      const l = Math.hypot(ex, ey, ez);
      edge += l;
      if (nb < 0 || ctx.s.member[nb] !== inSmooth) continue;
      soft += l;
      // Kanten entlang der Achse (z. B. tangential anschließende Flanke eines Langlochs) sind erlaubt
      if (l > 0 && Math.abs(ex * m.axis[0] + ey * m.axis[1] + ez * m.axis[2]) > 0.5 * l) continue;
      const hu = axial(u);
      const hw = axial(w);
      const du = Math.min(hu - hMin, hMax - hu);
      const dw = Math.min(hw - hMin, hMax - hw);
      devSum += (l * (du * du + dw * dw)) / 2;
      devLen += l;
      if (hu + hw < hMin + hMax) softLo += l;
      else softHi += l;
      let nMin = Infinity;
      let nMax = -Infinity;
      for (let j = 0; j < 3; j++) {
        const h = axial(vertIds[3 * nb + j]);
        nMin = Math.min(nMin, h);
        nMax = Math.max(nMax, h);
      }
      nbHeight += l * (nMax - nMin);
    }
  }
  const softFrac = edge > 0 ? soft / edge : 0;
  const softDev = devLen > 0 ? Math.sqrt(devSum / devLen) : 0;
  const length = hMax - hMin;

  if (!(m.r > 0) || !(rms <= CYL_MAX_RMS * m.r)) return null;
  if (arc < CYL_MIN_ARC || !(area > 1.5 * planeArea) || m.spread < 0.01) return null;
  if (!(m.tilt2 <= CYL_MAX_TILT_SIN * CYL_MAX_TILT_SIN)) return null;
  // Geht die glatte Fläche ringsum weiter, muss sie das an den Enden tun (Verrundung, Kapsel).
  // Weiche Seitenränder in wechselnder Höhe zeigen einen Streifen einer doppelt gekrümmten
  // Fläche (z. B. Kugel), dessen Facetten nur zufällig zylinderförmig aussehen.
  if (softFrac > 0.5 && softDev > 0.05 * length + topo.diag * 1e-6) return null;
  // Geht sie an beiden Enden weiter, muss das Band deutlich höher sein als die angrenzenden
  // Dreiecke; sonst ist es nur eine Facettenreihe einer runden Fläche (Äquator einer Kugel).
  if (softLo > 0 && softHi > 0 && length < (2 * nbHeight) / devLen) return null;
  return {
    kind: 'cylinder',
    tris,
    axis: canonical(m.axis),
    center: add(m.c0, scale(m.axis, (hMin + hMax) / 2)),
    radius: m.r,
    length,
    hole: m.hole,
    arc,
    rms,
    area,
  };
}

/** Zwei Modelle beschreiben (bis auf Rundungsfehler) denselben Zylinder. */
function sameCylinder(a: CylModel, b: CylModel): boolean {
  const tol = 1e-6 * a.r;
  const d = sub(b.c0, a.c0);
  const off = len(sub(d, scale(a.axis, dot(d, a.axis))));
  return Math.abs(dot(a.axis, b.axis)) > 1 - 1e-10 && Math.abs(a.r - b.r) < tol && off < tol &&
    a.hole === b.hole && Math.abs(a.spanCos - b.spanCos) < 1e-6;
}

/**
 * Zylinder zu einem Achsvorschlag: Startschätzung aus einer kleinen Region um den Klick, danach
 * abwechselnd wachsen (nur Dreiecke, die auf dem Zylinder liegen) und neu anpassen.
 */
function growCylinder(
  ctx: Ctx,
  seed: number,
  axis0: Vec3,
  inSmooth: number,
  planeArea: number,
  smoothAngle: number,
): CylinderFeature | null {
  const topo = ctx.topo;
  const { member } = ctx.s;
  if (Math.abs(ndotv(topo, seed, axis0)) > CYL_AXIS_SIN) return null;
  const maxSpan = 2 * smoothAngle;
  const local = flood(ctx, seed, ctx.limit, (_r, to) =>
    member[to] === inSmooth &&
    (isDegenerate(topo, to) || (Math.abs(ndotv(topo, to, axis0)) <= CYL_AXIS_SIN && ndot(topo, to, seed) >= CYL_LOCAL_COS)));
  // Normalen, die nicht gemeinsam senkrecht zu einer Achse stehen (z. B. Kugel), früh verwerfen
  const tooTilted = (m: CylModel | null) => !m || !(m.tilt2 <= 4 * CYL_MAX_TILT_SIN * CYL_MAX_TILT_SIN);
  let model = fitCylinderModel(ctx, local.tris, axis0, maxSpan);
  if (!model || tooTilted(model)) return null;
  let region = local;
  for (let it = 0; it < 8; it++) {
    const m = model;
    const tolR = Math.min(0.05 * m.r, Math.max(CYL_GROW_TOL * m.r, 3 * m.rms)) + topo.diag * 1e-6;
    if (!onCylinder(topo, seed, m, tolR)) return null;
    const next = flood(ctx, seed, ctx.limit, (_r, to) => member[to] === inSmooth && onCylinder(topo, to, m, tolR));
    const refit = fitCylinderModel(ctx, next.tris, m.axis, maxSpan);
    const unchanged = next.tris.length === region.tris.length;
    region = next;
    if (!refit || tooTilted(refit)) return null;
    model = refit;
    // Region unverändert oder Modell praktisch gleich geblieben: weiteres Wachsen ändert nichts mehr
    if (unchanged || sameCylinder(m, refit)) break;
  }
  return finishCylinder(ctx, region, model, inSmooth, planeArea);
}

function detectCylinder(ctx: Ctx, seed: number, smooth: number[], planeArea: number, smoothAngle: number): CylinderFeature | null {
  const inSmooth = markMembers(ctx, smooth);
  let best: CylinderFeature | null = null;
  for (const axis of axisCandidates(ctx.topo, seed, smooth)) {
    const f = growCylinder(ctx, seed, axis, inSmooth, planeArea, smoothAngle);
    if (f && (!best || f.area > best.area)) best = f;
  }
  return best;
}

// ---------------------------------------------------------------------------------------------
// Ebene
// ---------------------------------------------------------------------------------------------

/** Randschleifen einer Region als Folgen verschweißter Eckpunkte. */
function boundaryLoops(ctx: Ctx, tris: number[], stamp: number): number[][] {
  const { neighbors, vertIds } = ctx.topo;
  const eu: number[] = [];
  const ev: number[] = [];
  for (const t of tris) {
    for (let k = 0; k < 3; k++) {
      const nb = neighbors[3 * t + k];
      if (nb >= 0 && ctx.s.member[nb] === stamp) continue;
      const u = vertIds[3 * t + k];
      const w = vertIds[3 * t + (k === 2 ? 0 : k + 1)];
      if (u === w) continue;
      eu.push(u);
      ev.push(w);
    }
  }
  // Randkanten je Eckpunkt; ohne Richtung verkettet, damit auch uneinheitlicher Umlaufsinn klappt
  const at = new Map<number, number[]>();
  const link = (v: number, e: number) => {
    const l = at.get(v);
    if (l) l.push(e);
    else at.set(v, [e]);
  };
  for (let e = 0; e < eu.length; e++) {
    link(eu[e], e);
    link(ev[e], e);
  }
  const used = new Uint8Array(eu.length);
  const loops: number[][] = [];
  for (let e0 = 0; e0 < eu.length; e0++) {
    if (used[e0]) continue;
    used[e0] = 1;
    const start = eu[e0];
    const loop = [start];
    let cur = ev[e0];
    while (cur !== start) {
      loop.push(cur);
      const next = at.get(cur)?.find((e) => !used[e]);
      if (next === undefined) break;
      used[next] = 1;
      cur = eu[next] === cur ? ev[next] : eu[next];
    }
    loops.push(loop);
  }
  return loops;
}

/** Von einer geschlossenen Punktfolge umschlossene Fläche, projiziert auf die Ebene mit Normale n. */
function enclosedArea(pts: Vec3[], n: Vec3): number {
  let s = 0;
  for (let i = 1; i + 1 < pts.length; i++) s += dot(cross(sub(pts[i], pts[0]), sub(pts[i + 1], pts[0])), n);
  return Math.abs(s) / 2;
}

/** Kreis durch den Umriss, falls er einer ist (geringe Abweichung, Punkte rundherum verteilt). */
function outlineCircle(outline: Vec3[], n: Vec3, maxGap: number): PlaneFeature['circle'] {
  if (outline.length < CIRCLE_MIN_POINTS) return undefined;
  const [e1, e2] = basis(n);
  const o = outline[0];
  const xs = new Float64Array(outline.length);
  const ys = new Float64Array(outline.length);
  outline.forEach((p, i) => {
    const d = sub(p, o);
    xs[i] = dot(d, e1);
    ys[i] = dot(d, e2);
  });
  const c = fitCircle2D(xs, ys);
  if (!c || !(c.rms <= CIRCLE_MAX_RMS * c.r)) return undefined;
  // Größte Lücke zwischen benachbarten Punkten (schließt z. B. die gerade Kante eines D-Profils aus)
  const ang = Array.from(xs, (x, i) => Math.atan2(ys[i] - c.cy, x - c.cx)).sort((a, b) => a - b);
  let gap = ang[0] + TWO_PI - ang[ang.length - 1];
  for (let i = 1; i < ang.length; i++) gap = Math.max(gap, ang[i] - ang[i - 1]);
  if (gap >= maxGap) return undefined;
  return { center: add(o, add(scale(e1, c.cx), scale(e2, c.cy))), radius: c.r, rms: c.rms };
}

/** Hauptrichtungen der Fläche (flächengewichtete Hauptkomponenten) und Ausdehnung entlang dieser. */
function planeAxes(ctx: Ctx, tris: number[], n: Vec3, centroid: Vec3): PlaneFeature['axes'] {
  const { positions, areas } = ctx.topo;
  const C = [0, 0, 0, 0, 0, 0, 0, 0, 0];
  for (const t of tris) {
    const a = areas[t];
    if (!(a > 0)) continue;
    // Zweites Flächenmoment eines Dreiecks (Ecken relativ zum Schwerpunkt der Fläche):
    // A/12 · (Σ vᵢ·vᵢᵀ + s·sᵀ) mit s = Summe der drei Ecken
    const o = 9 * t;
    const vs: Vec3[] = [0, 3, 6].map((k): Vec3 => [
      positions[o + k] - centroid[0], positions[o + k + 1] - centroid[1], positions[o + k + 2] - centroid[2],
    ]);
    vs.push(add(add(vs[0], vs[1]), vs[2]));
    for (const v of vs) for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[3 * i + j] += (a / 12) * v[i] * v[j];
  }
  let u = normalize(eigenSym3(C).vectors[2]);
  u = normalize(sub(u, scale(n, dot(u, n))));
  if (len(u) < 0.5) u = basis(n)[0];
  u = canonical(u);
  const v = cross(n, u);
  let uMin = Infinity, uMax = -Infinity, vMin = Infinity, vMax = -Infinity;
  for (const id of regionVertices(ctx, tris)) {
    const d = sub(vertexOf(ctx.topo, id), centroid);
    const x = dot(d, u);
    const y = dot(d, v);
    uMin = Math.min(uMin, x);
    uMax = Math.max(uMax, x);
    vMin = Math.min(vMin, y);
    vMax = Math.max(vMax, y);
  }
  return { u, v, size: [uMax - uMin, vMax - vMin] };
}

function planeFeature(ctx: Ctx, tris: number[], smoothAngle: number): PlaneFeature {
  const { area, centroid, normal } = regionStats(ctx.topo, tris);
  const stamp = markMembers(ctx, tris);
  // Äußerer Umriss: die Schleife mit der größten umschlossenen Fläche
  let outline: Vec3[] = [];
  let best = -1;
  for (const loop of boundaryLoops(ctx, tris, stamp)) {
    const pts = loop.map((v) => vertexOf(ctx.topo, v));
    const a = enclosedArea(pts, normal);
    if (a > best) {
      best = a;
      outline = pts;
    }
  }
  const f: PlaneFeature = { kind: 'plane', tris, centroid, normal, area, outline, axes: planeAxes(ctx, tris, normal, centroid) };
  const circle = outlineCircle(outline, normal, smoothAngle);
  if (circle) f.circle = circle;
  return f;
}

// ---------------------------------------------------------------------------------------------
// Erkennung
// ---------------------------------------------------------------------------------------------

/** Entartetes Dreieck angeklickt: nächstgelegenes brauchbares Nachbardreieck verwenden. */
function usableSeed(ctx: Ctx, seedTri: number): number {
  if (!isDegenerate(ctx.topo, seedTri)) return seedTri;
  const near = flood(ctx, seedTri, 64, () => true).tris;
  return near.find((t) => !isDegenerate(ctx.topo, t)) ?? -1;
}

/**
 * Bestimmt das Formelement, zu dem das angeklickte Dreieck gehört:
 * 1. Ebene: Dreiecke mit (fast) gleicher Normale in derselben Ebene wie das Startdreieck.
 * 2. Glatte Fläche: Ausbreitung über Kanten mit kleinem Knickwinkel.
 * 3. Ist die glatte Fläche deutlich größer als die Ebene, wird ein Zylinder angepasst.
 * 4. Sonst Ebene (mit Umriss, ggf. Kreis), oder bei doppelt gekrümmten Flächen eine Freiformfläche.
 */
export function detectFeature(topo: Topology, seedTri: number, opts: FeatureOptions = {}): Feature {
  if (!Number.isInteger(seedTri) || seedTri < 0 || seedTri >= topo.triCount)
    throw new RangeError(`Dreieck ${seedTri} gibt es nicht.`);
  const planeAngle = (opts.planeAngleDeg ?? FEATURE_DEFAULTS.planeAngleDeg) * DEG;
  const smoothAngle = (opts.smoothAngleDeg ?? FEATURE_DEFAULTS.smoothAngleDeg) * DEG;
  const limit = Math.max(1, Math.floor(opts.maxTris ?? FEATURE_DEFAULTS.maxTris));
  const ctx = makeCtx(topo, limit);
  const seed = usableSeed(ctx, seedTri);
  if (seed < 0)
    return { kind: 'surface', tris: [seedTri], centroid: centroidOf(topo, seedTri), normal: [0, 0, 0], area: topo.areas[seedTri] };

  // Ebene durch das Startdreieck
  const n0 = normalOf(topo, seed);
  const d0 = dot(n0, centroidOf(topo, seed));
  const cosPlane = Math.cos(planeAngle);
  const distTol = topo.diag * 2e-4 + 1e-9;
  const { centroids } = topo;
  const plane = flood(ctx, seed, limit, (_r, to) => {
    const off = n0[0] * centroids[3 * to] + n0[1] * centroids[3 * to + 1] + n0[2] * centroids[3 * to + 2] - d0;
    return Math.abs(off) <= distTol && (isDegenerate(topo, to) || ndotv(topo, to, n0) >= cosPlane);
  }).tris;

  // Glatte Fläche: Knickwinkel zwischen benachbarten Dreiecken klein
  const cosSmooth = Math.cos(smoothAngle);
  const smooth = flood(ctx, seed, limit, (r, to) => isDegenerate(topo, to) || ndot(topo, r, to) >= cosSmooth).tris;

  const aPlane = areaOf(topo, plane);
  const aSmooth = areaOf(topo, smooth);
  if (aSmooth > 1.5 * aPlane) {
    const cyl = detectCylinder(ctx, seed, smooth, aPlane, smoothAngle);
    if (cyl) return cyl;
  }

  // Ebene, wenn sie von scharfen Kanten begrenzt ist oder einen nennenswerten Teil der glatten Fläche ausmacht
  // (einzelne Facetten einer gekrümmten Fläche sind keine Ebene)
  let planeTris = 0;
  for (const t of plane) if (!isDegenerate(topo, t)) planeTris++;
  if (aSmooth <= 1.5 * aPlane || aPlane >= 0.05 * aSmooth || (planeTris >= 3 && aPlane >= 0.01 * aSmooth))
    return planeFeature(ctx, plane, smoothAngle);

  // Freiformfläche: Normalen nahe der Klick-Normale, ohne über Kanten zu springen
  const tris = flood(ctx, seed, limit, (r, to) =>
    isDegenerate(topo, to) || (ndotv(topo, to, n0) >= SURFACE_COS && ndot(topo, r, to) >= cosSmooth)).tris;
  const { area, centroid, normal } = regionStats(topo, tris);
  return { kind: 'surface', tris, centroid, normal, area };
}
