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
import { type Rigid, type Vec3, add, apply, dot, len, normalize, rotate, scale, sub } from './align';

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
  /**
   * Nachbar über die Kanten (v0v1, v1v2, v2v0); -1 bei offenem Rand oder mehrdeutiger Kante.
   * Ein doppelt gespeichertes Dreieck zählt dabei nicht mit, es übernimmt die Nachbarn des Originals.
   */
  neighbors: Int32Array;
  /** Nummer des verschweißten Eckpunkts je Dreiecksecke (3 pro Dreieck). */
  vertIds: Int32Array;
  /** Position je verschweißtem Eckpunkt (3 pro Punkt). */
  vertPos: Float32Array;
  vertCount: number;
  /** Diagonale des umschließenden Quaders (Bezugsgröße für Toleranzen, nur endliche Koordinaten). */
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
  /** Restabweichung der Eckpunkte vom Zylinder (flächengewichtet). */
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

/** Gerade, scharfe Kante (oder offener Rand) – Kante auf Kante setzt Teile wie an einem Lineal an. */
export interface EdgeFeature {
  kind: 'edge';
  /** Dreiecke an der Kante (je Kantenstück eines). */
  tris: number[];
  /** Endpunkte, Mitte und Richtung (Einheitsvektor von a nach b). */
  a: Vec3;
  b: Vec3;
  center: Vec3;
  dir: Vec3;
  length: number;
  /** Mittlere Normale der angrenzenden Flächen (zeigt vom Teil weg). */
  normal: Vec3;
}

export type Feature = PlaneFeature | CylinderFeature | SurfaceFeature | EdgeFeature;

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
/** Rundungsfehler von Float32-Koordinaten (großzügig), bezogen auf den größten Koordinatenbetrag. */
const FLOAT_EPS = 4e-7;

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
/** Startregion: höchstens so viele Dreiecke (nächstgelegene zuerst). */
const LOCAL_MAX = 20000;
/**
 * Wachstumstoleranz = Vielfaches der Restabweichung der letzten Anpassung. Bei sauber facettierten
 * Netzen ist diese praktisch null, dann gilt nur die Rechengenauigkeit: tangential anschließende
 * Verrundungen, Kugelkappen usw. liegen schon in der ersten Facettenreihe deutlich daneben und
 * bleiben draußen. Bei verrauschten Netzen wächst die Toleranz mit dem Rauschen.
 */
const CYL_GROW_K = 4;
/** Obergrenze der Wachstumstoleranz (relativ zum Radius). */
const CYL_GROW_MAX = 0.05;
/** Untergrenze der Wachstumstoleranz (relativ zum Radius), zusätzlich zum Float32-Rundungsfehler. */
const CYL_EPS_REL = 2e-5;
const GROW_ITER = 8;
/** Restabweichung, bis zu der ein Zylinder ohne weitere Prüfung als exakt gilt (relativ zum Radius). */
const CYL_EXACT_RMS = 1e-4;
/** Größte erlaubte Restabweichung verrauschter Zylinder (relativ zum Radius). */
const CYL_MAX_RMS = 0.02;
/**
 * Verrauschte Zylinder: höchstens dieser Anteil der Abweichung darf sich durch einen über die
 * Höhe veränderlichen Radius erklären lassen (Kegel, Kugel, Tonne). Zufälliges Rauschen erklärt
 * sich so nur zu einem kleinen Teil.
 */
const STRUCT_MAX = 0.5;
const STRUCT_MIN_POINTS = 24;
const CYL_MIN_ARC = 45 * DEG;
/** Achsschätzung: Mindestschwelle für die Neigung einer Normalen gegen die Achse. */
const AXIS_CUT_MIN = Math.sin(0.25 * DEG);
/** Kanten zwischen Dreiecken mit fast gleicher Normale sagen nichts über die Achse aus. */
const CREASE_COS = Math.cos(0.02 * DEG);
const CAND_SAME_COS = Math.cos(3 * DEG);

/** Ebene: Abstand der Schwerpunkte von der Ebene (relativ zur Diagonale des Netzes). */
const PLANE_DIST_REL = 2e-4;
/** Ebene: Abweichung der Eckpunkte von der Ebene, bezogen auf die kleinere Ausdehnung der Fläche. */
const FLAT_REL = 2e-4;
/** Übergänge in gekrümmte Nachbarflächen: so viele Randkanten werden untersucht. */
const JUNCTION_SAMPLES = 12;
const JUNCTION_MAX = 20000;
/**
 * Knick zwischen einer ebenen Region und ihrer Nachbarfacette, gemessen an dem Knick, den eine
 * Facette derselben Krümmung hätte: Facetten liegen bei etwa 1, ebene Flächen, an die eine Rundung
 * tangential anschließt, bei höchstens 0,5.
 */
const FACET_RATIO = 0.7;
const PARALLEL_COS = Math.cos(0.05 * DEG);
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
 * Zusammengefallene Dreiecke, Dreiecke mit ungültigen Koordinaten und doppelt gespeicherte
 * Dreiecke (gleiche Ecken im gleichen Umlaufsinn) zählen bei der Suche nach Kantennachbarn nicht mit.
 * @param weldTol Eckpunkte mit kleinerem Abstand gelten als gleich (Standard: 1e-5 · Diagonale)
 */
export function buildTopology(positions: Float32Array, weldTol?: number): Topology {
  const triCount = Math.floor(positions.length / 9);
  const corners = triCount * 3;

  // Umschließender Quader nur aus endlichen Koordinaten (ein einziger Punkt im Unendlichen
  // würde sonst alle relativen Toleranzen verderben)
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
    if (!Number.isFinite(x + y + z)) continue;
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
  }
  const d = Math.hypot(maxX - minX, maxY - minY, maxZ - minZ);
  const diag = Number.isFinite(d) ? d : 0;
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
  // Dreiecke ohne Nachbarschaft: zusammengefallen oder mit ungültigen Koordinaten
  const skip = new Uint8Array(triCount);
  // Dreiecke, die flacher als diese Höhe sind, haben keine verlässliche Normale
  const minHeight = Math.max(diag * 1e-7, 1e-12);
  for (let t = 0; t < triCount; t++) {
    const o = 9 * t;
    const ax = positions[o], ay = positions[o + 1], az = positions[o + 2];
    const bx = positions[o + 3], by = positions[o + 4], bz = positions[o + 5];
    const cx = positions[o + 6], cy = positions[o + 7], cz = positions[o + 8];
    const i0 = vertIds[3 * t], i1 = vertIds[3 * t + 1], i2 = vertIds[3 * t + 2];
    const finite = Number.isFinite(ax + ay + az + bx + by + bz + cx + cy + cz);
    if (!finite || i0 === i1 || i1 === i2 || i0 === i2) skip[t] = 1;
    // n = (c - b) × (a - b)
    const ux = cx - bx, uy = cy - by, uz = cz - bz;
    const vx = ax - bx, vy = ay - by, vz = az - bz;
    const nx = uy * vz - uz * vy;
    const ny = uz * vx - ux * vz;
    const nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz);
    areas[t] = finite ? l / 2 : 0;
    centroids[3 * t] = (ax + bx + cx) / 3;
    centroids[3 * t + 1] = (ay + by + cy) / 3;
    centroids[3 * t + 2] = (az + bz + cz) / 3;
    const wx = ax - cx, wy = ay - cy, wz = az - cz;
    const maxEdge = Math.sqrt(Math.max(ux * ux + uy * uy + uz * uz, vx * vx + vy * vy + vz * vz, wx * wx + wy * wy + wz * wz));
    if (!skip[t] && l > minHeight * maxEdge) {
      normals[3 * t] = nx / l;
      normals[3 * t + 1] = ny / l;
      normals[3 * t + 2] = nz / l;
    }
  }

  // Dreiecke je Eckpunkt (kompakte Listen, aufsteigend sortiert)
  const start = new Int32Array(vertCount + 1);
  for (let t = 0; t < triCount; t++) {
    if (skip[t]) continue;
    for (let k = 0; k < 3; k++) start[vertIds[3 * t + k] + 1]++;
  }
  for (let v = 0; v < vertCount; v++) start[v + 1] += start[v];
  const fill = start.slice(0, vertCount);
  const inc = new Int32Array(start[vertCount]);
  for (let t = 0; t < triCount; t++) {
    if (skip[t]) continue;
    for (let k = 0; k < 3; k++) inc[fill[vertIds[3 * t + k]]++] = t;
  }
  const degree = (v: number) => start[v + 1] - start[v];

  // Doppelt gespeicherte Dreiecke: gleiche Ecken in gleicher Reihenfolge wie ein früheres Dreieck.
  // Sie würden sonst alle drei Kanten mehrdeutig machen und die Fläche zerteilen.
  const dupOf = new Int32Array(triCount).fill(-1);
  for (let t = 0; t < triCount; t++) {
    if (skip[t]) continue;
    const a = vertIds[3 * t], b = vertIds[3 * t + 1], c = vertIds[3 * t + 2];
    const x = degree(a) <= degree(b) ? (degree(a) <= degree(c) ? a : c) : degree(b) <= degree(c) ? b : c;
    for (let i = start[x]; i < start[x + 1]; i++) {
      const s = inc[i];
      if (s >= t) break;
      let j = 0;
      while (j < 3 && vertIds[3 * s + j] !== a) j++;
      if (j < 3 && vertIds[3 * s + ((j + 1) % 3)] === b && vertIds[3 * s + ((j + 2) % 3)] === c) {
        dupOf[t] = dupOf[s] >= 0 ? dupOf[s] : s;
        break;
      }
    }
  }

  // Nachbar über jede Kante: das einzige andere Dreieck, das beide Endpunkte enthält
  const neighbors = new Int32Array(corners).fill(-1);
  for (let t = 0; t < triCount; t++) {
    if (skip[t] || dupOf[t] >= 0) continue;
    for (let k = 0; k < 3; k++) {
      const u = vertIds[3 * t + k];
      const w = vertIds[3 * t + (k === 2 ? 0 : k + 1)];
      // Liste des Eckpunkts mit weniger Dreiecken durchsuchen (Fächer mit vielen Dreiecken)
      const uSmaller = degree(u) <= degree(w);
      const x = uSmaller ? u : w;
      const y = uSmaller ? w : u;
      let found = -1;
      let count = 0;
      for (let i = start[x]; i < start[x + 1]; i++) {
        const s = inc[i];
        if (s === t || dupOf[s] >= 0) continue;
        if (vertIds[3 * s] === y || vertIds[3 * s + 1] === y || vertIds[3 * s + 2] === y) {
          count++;
          found = s;
        }
      }
      if (count === 1) neighbors[3 * t + k] = found;
    }
  }
  // Duplikate übernehmen die Nachbarn ihres Originals (Kante für Kante)
  for (let t = 0; t < triCount; t++) {
    const o = dupOf[t];
    if (o < 0) continue;
    let j = 0;
    while (vertIds[3 * o + j] !== vertIds[3 * t]) j++;
    for (let k = 0; k < 3; k++) neighbors[3 * t + k] = neighbors[3 * o + ((j + k) % 3)];
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
  /** Zugehörigkeit zur glatten Region (Zylindersuche) bzw. zur ebenen Region. */
  member: Int32Array;
  /** Zweite Zugehörigkeit (fertige Zylinderregion, Nachbarfacetten einer Ebene). */
  group: Int32Array;
  vmark: Int32Array;
  /** Gewichte je Eckpunkt (Flächenanteil), nur für die Eckpunkte der aktuellen Region gültig. */
  vw: Float64Array;
  /** Fortlaufender Zähler: jede Markierung bekommt einen neuen Wert, alte werden so ungültig. */
  stamp: number;
  /** Größter Betrag einer Koordinate (Bezugsgröße für Float32-Rundungsfehler). */
  coordMag: number;
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
    let coordMag = 0;
    for (let i = 0; i < topo.vertPos.length; i++) {
      const a = Math.abs(topo.vertPos[i]);
      if (a > coordMag && a < Infinity) coordMag = a;
    }
    s = {
      visit: new Int32Array(topo.triCount),
      ref: new Int32Array(topo.triCount),
      member: new Int32Array(topo.triCount),
      group: new Int32Array(topo.triCount),
      vmark: new Int32Array(topo.vertCount),
      vw: new Float64Array(topo.vertCount),
      stamp: 0,
      coordMag,
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
/** Kante k des Dreiecks t als Paar verschweißter Eckpunkte. */
const edgeVerts = (topo: Topology, t: number, k: number): [number, number] =>
  [topo.vertIds[3 * t + k], topo.vertIds[3 * t + (k === 2 ? 0 : k + 1)]];

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

/** Markiert eine Dreiecksmenge im Feld `arr` (member oder group) und gibt die Markierung zurück. */
function mark(ctx: Ctx, arr: Int32Array, tris: number[]): number {
  const stamp = ++ctx.s.stamp;
  for (const t of tris) arr[t] = stamp;
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

/** Eckpunkte einer Region mit Gewicht (ein Drittel der Fläche jedes angrenzenden Dreiecks der Region). */
function weightedVertices(ctx: Ctx, tris: number[]): { verts: number[]; weights: Float64Array } {
  const { vertIds, areas } = ctx.topo;
  const { vw } = ctx.s;
  const verts = regionVertices(ctx, tris);
  for (const v of verts) vw[v] = 0;
  for (const t of tris) {
    const a = areas[t] / 3;
    vw[vertIds[3 * t]] += a;
    vw[vertIds[3 * t + 1]] += a;
    vw[vertIds[3 * t + 2]] += a;
  }
  return { verts, weights: Float64Array.from(verts, (v) => vw[v]) };
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
    if (!(a > 0)) continue;
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

/** Ausdehnung einer Punktmenge entlang einer Richtung. */
function extentAlong(topo: Topology, verts: number[], d: Vec3): number {
  let lo = Infinity;
  let hi = -Infinity;
  for (const v of verts) {
    const x = topo.vertPos[3 * v] * d[0] + topo.vertPos[3 * v + 1] * d[1] + topo.vertPos[3 * v + 2] * d[2];
    if (x < lo) lo = x;
    if (x > hi) hi = x;
  }
  return hi > lo ? hi - lo : 0;
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

/** Löst A·x = b für eine n×n-Matrix (zeilenweise, wird überschrieben); null wenn singulär. */
function solveLinear(A: Float64Array, b: Float64Array, n: number): Float64Array | null {
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(A[r * n + c]) > Math.abs(A[p * n + c])) p = r;
    if (!(Math.abs(A[p * n + c]) > 1e-300)) return null;
    if (p !== c) {
      for (let k = 0; k < n; k++) [A[c * n + k], A[p * n + k]] = [A[p * n + k], A[c * n + k]];
      [b[c], b[p]] = [b[p], b[c]];
    }
    for (let r = c + 1; r < n; r++) {
      const f = A[r * n + c] / A[c * n + c];
      if (f === 0) continue;
      for (let k = c; k < n; k++) A[r * n + k] -= f * A[c * n + k];
      b[r] -= f * b[c];
    }
  }
  const x = new Float64Array(n);
  for (let r = n - 1; r >= 0; r--) {
    let s = b[r];
    for (let k = r + 1; k < n; k++) s -= A[r * n + k] * x[k];
    x[r] = s / A[r * n + r];
  }
  return x;
}

interface Circle2 {
  cx: number;
  cy: number;
  r: number;
  rms: number;
}

function circleRms(xs: ArrayLike<number>, ys: ArrayLike<number>, ws: ArrayLike<number> | null, cx: number, cy: number, r: number): number {
  let s = 0;
  let W = 0;
  for (let i = 0; i < xs.length; i++) {
    const w = ws ? ws[i] : 1;
    const e = Math.hypot(xs[i] - cx, ys[i] - cy) - r;
    s += w * e * e;
    W += w;
  }
  return W > 0 ? Math.sqrt(s / W) : 0;
}

/**
 * Kreis durch Punkte in der Ebene, optional gewichtet: algebraische Lösung (Kåsa, um den
 * Schwerpunkt zentriert), danach geometrisch verfeinert (Gauß-Newton auf die Abstände zum Kreis).
 */
function fitCircle2D(xs: ArrayLike<number>, ys: ArrayLike<number>, ws: ArrayLike<number> | null = null): Circle2 | null {
  const n = xs.length;
  if (n < 3) return null;
  let W = 0;
  let mx = 0;
  let my = 0;
  for (let i = 0; i < n; i++) {
    const w = ws ? ws[i] : 1;
    W += w;
    mx += w * xs[i];
    my += w * ys[i];
  }
  if (!(W > 0)) return null;
  mx /= W;
  my /= W;
  let suu = 0, suv = 0, svv = 0, suuu = 0, svvv = 0, suvv = 0, svuu = 0;
  for (let i = 0; i < n; i++) {
    const w = ws ? ws[i] : 1;
    const u = xs[i] - mx;
    const v = ys[i] - my;
    const uu = u * u;
    const vv = v * v;
    suu += w * uu;
    suv += w * u * v;
    svv += w * vv;
    suuu += w * uu * u;
    svvv += w * vv * v;
    suvv += w * u * vv;
    svuu += w * v * uu;
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
  let r = Math.sqrt(uc * uc + vc * vc + (suu + svv) / W);
  let rms = circleRms(xs, ys, ws, cx, cy, r);

  for (let it = 0; it < 10 && rms > 1e-12 * r; it++) {
    const J = new Float64Array(9);
    const g = new Float64Array(3);
    for (let i = 0; i < n; i++) {
      const w = ws ? ws[i] : 1;
      const dx = xs[i] - cx;
      const dy = ys[i] - cy;
      const d = Math.hypot(dx, dy);
      if (d < 1e-12 || !(w > 0)) continue;
      const j0 = -dx / d;
      const j1 = -dy / d;
      const e = d - r;
      J[0] += w * j0 * j0; J[1] += w * j0 * j1; J[2] -= w * j0;
      J[4] += w * j1 * j1; J[5] -= w * j1;
      J[8] += w;
      g[0] -= w * j0 * e; g[1] -= w * j1 * e; g[2] += w * e;
    }
    J[3] = J[1]; J[6] = J[2]; J[7] = J[5];
    const step = solveLinear(J, g, 3);
    if (!step) break;
    const ncx = cx + step[0];
    const ncy = cy + step[1];
    const nr = r + step[2];
    if (!(nr > 0)) break;
    const nrms = circleRms(xs, ys, ws, ncx, ncy, nr);
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

/** Gewichteter Median (bei vielen Werten aus einer gleichmäßigen Stichprobe). */
function weightedMedian(values: Float64Array, ws: Float64Array): number {
  const step = Math.max(1, Math.floor(values.length / 4096));
  const idx: number[] = [];
  let total = 0;
  for (let i = 0; i < values.length; i += step) {
    if (!(ws[i] > 0)) continue;
    idx.push(i);
    total += ws[i];
  }
  idx.sort((a, b) => values[a] - values[b]);
  let acc = 0;
  for (const i of idx) {
    acc += ws[i];
    if (acc >= total / 2) return values[i];
  }
  return 0;
}

/**
 * Gewichtete Kreisanpassung mit Ausreißerbehandlung: Punkte, die deutlich weiter vom Kreis
 * entfernt sind als die typische (gewichtete) Abweichung, werden schrittweise verworfen.
 * `floor` ist die kleinste Abweichung, die überhaupt als Ausreißer gilt (Rechengenauigkeit).
 */
function robustCircle(xs: Float64Array, ys: Float64Array, ws: Float64Array, floor: number): Circle2 | null {
  let c = fitCircle2D(xs, ys, ws);
  let px = xs;
  let py = ys;
  let pw = ws;
  let total = 0;
  for (let i = 0; i < ws.length; i++) total += ws[i];
  for (let it = 0; it < 4 && c; it++) {
    const res = new Float64Array(px.length);
    for (let i = 0; i < px.length; i++) res[i] = Math.abs(Math.hypot(px[i] - c.cx, py[i] - c.cy) - c.r);
    const thr = Math.max(3 * 1.4826 * weightedMedian(res, pw), floor + CYL_EPS_REL * c.r);
    let count = 0;
    let kept = 0;
    for (let i = 0; i < res.length; i++) {
      if (res[i] > thr) continue;
      count++;
      kept += pw[i];
    }
    if (count === px.length || count < 3 || kept < 0.3 * total) break;
    const qx = new Float64Array(count);
    const qy = new Float64Array(count);
    const qw = new Float64Array(count);
    for (let i = 0, j = 0; i < res.length; i++) {
      if (res[i] > thr) continue;
      qx[j] = px[i];
      qy[j] = py[i];
      qw[j++] = pw[i];
    }
    const c2 = fitCircle2D(qx, qy, qw);
    if (!c2) break;
    c = c2;
    px = qx;
    py = qy;
    pw = qw;
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
  /** Gewichtete Restabweichung der bei der Anpassung verwendeten Eckpunkte (Maß für das Rauschen). */
  rms: number;
  hole: boolean;
  /** cos der größten erlaubten Winkelbreite eines Dreiecks um die Achse. */
  spanCos: number;
  /** Mittlerer Eigenwert der Normalen-Streumatrix relativ zur Spur (0 = alle Normalen gleich). */
  spread: number;
  /** Mittleres Quadrat von n·Achse (flächengewichtet), 0 bei einem exakten Zylinder. */
  tilt2: number;
}

/** Kleinste Abweichung, die bei diesem Zylinder über der Rechengenauigkeit liegt. */
const cylEps = (ctx: Ctx, m: CylModel) => CYL_EPS_REL * m.r + FLOAT_EPS * ctx.s.coordMag;

/** Toleranz für Eckpunkte beim Wachsen: aus dem Rauschen der letzten Anpassung, mindestens die Rechengenauigkeit. */
const growTol = (ctx: Ctx, m: CylModel) => Math.min(CYL_GROW_MAX * m.r, Math.max(cylEps(ctx, m), CYL_GROW_K * m.rms));

/** Flächengewichtete Streumatrix Σ A·n·nᵀ der Normalen (zeilenweise 3x3), optional nur für ausgewählte Dreiecke. */
function normalScatter(topo: Topology, tris: number[], use?: (t: number) => boolean): number[] {
  const { normals, areas } = topo;
  let xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
  for (const t of tris) {
    if (use && !use(t)) continue;
    const a = areas[t];
    const x = normals[3 * t];
    const y = normals[3 * t + 1];
    const z = normals[3 * t + 2];
    xx += a * x * x; xy += a * x * y; xz += a * x * z;
    yy += a * y * y; yz += a * y * z; zz += a * z * z;
  }
  return [xx, xy, xz, xy, yy, yz, xz, yz, zz];
}

/**
 * Achse aus der Normalenstreuung (die Normalen eines Zylinders stehen senkrecht zur Achse, also
 * Richtung mit der kleinsten Streuung). Dreiecke, deren Normale deutlich stärker gegen die Achse
 * geneigt ist als üblich (erste Reihe einer tangential anschließenden Verrundung), werden
 * schrittweise weggelassen. Bei zu kleiner Streuung (schmaler Streifen) bleibt der Vorschlag.
 */
function robustAxis(topo: Topology, tris: number[], guess: Vec3): { axis: Vec3; spread: number; tilt2: number } | null {
  let axis = guess;
  let spread = 0;
  let cut = Infinity;
  for (let it = 0; it < 4; it++) {
    const a0 = axis;
    const c0 = cut;
    const M = normalScatter(topo, tris, (t) => Math.abs(ndotv(topo, t, a0)) <= c0);
    const { values, vectors } = eigenSym3(M);
    const trace = values[0] + values[1] + values[2];
    if (!(trace > 0)) {
      if (it === 0) return null;
      break;
    }
    spread = values[1] / trace;
    if (spread > 2e-3) axis = dot(vectors[0], axis) < 0 ? scale(vectors[0], -1) : vectors[0];
    // Neue Schwelle aus der mittleren Neigung der bisher verwendeten Dreiecke
    let s2 = 0;
    let sw = 0;
    for (const t of tris) {
      const a = topo.areas[t];
      const nd = ndotv(topo, t, a0);
      if (!(a > 0) || Math.abs(nd) > c0) continue;
      const na = ndotv(topo, t, axis);
      s2 += a * na * na;
      sw += a;
    }
    const next = Math.max(AXIS_CUT_MIN, 2.5 * Math.sqrt(s2 / sw));
    if (next >= cut && dot(axis, a0) > 1 - 1e-14) break;
    cut = next;
  }
  let s2 = 0;
  let sw = 0;
  for (const t of tris) {
    const a = topo.areas[t];
    if (!(a > 0)) continue;
    const na = ndotv(topo, t, axis);
    s2 += a * na * na;
    sw += a;
  }
  return { axis, spread, tilt2: sw > 0 ? s2 / sw : 0 };
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

/** Bohrung oder Zapfen und typische Winkelbreite der Dreiecke (für die Breitenbegrenzung). */
function orientModel(topo: Topology, tris: number[], m: CylModel, maxSpan: number): void {
  let s = 0;
  const spans: number[] = [];
  const step = Math.max(1, Math.floor(tris.length / 1000));
  for (let i = 0; i < tris.length; i++) {
    const t = tris[i];
    if (isDegenerate(topo, t)) continue;
    const nr = radialNormal(topo, t, m);
    if (Number.isNaN(nr)) continue;
    s += topo.areas[t] * nr;
    if (i % step === 0) {
      const sc = cornersOnCylinder(topo, t, m, Infinity);
      if (sc >= -1) spans.push(Math.acos(Math.min(1, sc)));
    }
  }
  m.hole = s < 0;
  const typical = spans.length ? median(Float64Array.from(spans)) : maxSpan;
  m.spanCos = Math.cos(Math.min(maxSpan, Math.max(4 * typical, 2 * DEG)));
}

/**
 * Passt einen Zylinder an eine Dreiecksmenge an: Achse aus der Normalenstreuung, danach Kreis durch
 * die in die Ebene senkrecht zur Achse projizierten Eckpunkte (flächengewichtet, mit Ausreißerbehandlung).
 */
function fitCylinderModel(ctx: Ctx, tris: number[], axisGuess: Vec3, maxSpan: number): CylModel | null {
  const topo = ctx.topo;
  const ax = robustAxis(topo, tris, axisGuess);
  if (!ax) return null;
  const { axis, spread, tilt2 } = ax;
  const [e1, e2] = basis(axis);
  const { verts, weights } = weightedVertices(ctx, tris);
  if (verts.length < 3) return null;
  // Erst im Hilfssystem mit Ursprung im ersten Eckpunkt rechnen, danach den Kreismittelpunkt übernehmen
  const model: CylModel = { axis, e1, e2, c0: vertexOf(topo, verts[0]), r: 0, rms: 0, hole: false, spanCos: -1, spread, tilt2 };
  const xs = new Float64Array(verts.length);
  const ys = new Float64Array(verts.length);
  for (let i = 0; i < verts.length; i++) {
    const p = toCyl(model, topo.vertPos, 3 * verts[i]);
    xs[i] = p[0];
    ys[i] = p[1];
  }
  const c = robustCircle(xs, ys, weights, FLOAT_EPS * ctx.s.coordMag);
  if (!c || !(c.r > 0)) return null;
  model.c0 = add(model.c0, add(scale(e1, c.cx), scale(e2, c.cy)));
  model.r = c.r;
  model.rms = c.rms;
  orientModel(topo, tris, model, maxSpan);
  return model;
}

interface AxisCandidate {
  axis: Vec3;
  /** Richtung einer Kante zwischen zwei Facetten: bei sauber facettierten Zylindern exakt die Achse. */
  exact: boolean;
}

/**
 * Vorschläge für die Zylinderachse: Kanten des Startdreiecks zu nicht koplanaren Nachbarn (bei
 * facettierten Zylindern laufen diese Kanten exakt parallel zur Achse), Normalenstreuung der
 * nahen Umgebung und Normalenstreuung der ganzen glatten Region.
 */
function axisCandidates(ctx: Ctx, seed: number, smooth: number[], inSmooth: number): AxisCandidate[] {
  const topo = ctx.topo;
  const out: AxisCandidate[] = [];
  const push = (v: Vec3, exact: boolean) => {
    const l = len(v);
    if (!(l > 1e-9)) return;
    const a = scale(v, 1 / l);
    if (out.some((b) => Math.abs(dot(a, b.axis)) > CAND_SAME_COS)) return;
    out.push({ axis: a, exact });
  };
  for (let k = 0; k < 3; k++) {
    const nb = topo.neighbors[3 * seed + k];
    if (nb < 0 || ctx.s.member[nb] !== inSmooth || isDegenerate(topo, nb) || ndot(topo, seed, nb) > CREASE_COS) continue;
    const [u, w] = edgeVerts(topo, seed, k);
    push(sub(vertexOf(topo, w), vertexOf(topo, u)), true);
  }
  const cosLocal = CYL_LOCAL_COS;
  const near = flood(ctx, seed, Math.min(ctx.limit, LOCAL_MAX), (_r, to) =>
    ctx.s.member[to] === inSmooth && (isDegenerate(topo, to) || ndot(topo, to, seed) >= cosLocal)).tris;
  push(eigenSym3(normalScatter(topo, near)).vectors[0], false);
  push(eigenSym3(normalScatter(topo, smooth)).vectors[0], false);
  return out;
}

/** Startmodell aus der Umgebung des Klicks: Dreiecke mit Normale senkrecht zur Achse und nahe der Klick-Normale. */
function localModel(ctx: Ctx, seed: number, axis: Vec3, inSmooth: number, maxSpan: number): CylModel | null {
  const topo = ctx.topo;
  const local = flood(ctx, seed, Math.min(ctx.limit, LOCAL_MAX), (_r, to) =>
    ctx.s.member[to] === inSmooth &&
    (isDegenerate(topo, to) || (Math.abs(ndotv(topo, to, axis)) <= CYL_AXIS_SIN && ndot(topo, to, seed) >= CYL_LOCAL_COS)));
  return fitCylinderModel(ctx, local.tris, axis, maxSpan);
}

/**
 * Startmodelle aus möglichst wenigen Punkten: Kreis durch die (in die Ebene senkrecht zur Achse
 * projizierten) Ecken des Startdreiecks und je einen weiteren Eckpunkt naher Dreiecke. Grenzt der
 * Klick an eine tangential anschließende Fläche, ist so wenigstens eines der Modelle exakt.
 */
function minimalModels(ctx: Ctx, seed: number, axis: Vec3, inSmooth: number, maxSpan: number): CylModel[] {
  const topo = ctx.topo;
  const [e1, e2] = basis(axis);
  const o = vertexOf(topo, topo.vertIds[3 * seed]);
  const proj = (v: number): [number, number] => {
    const d = sub(vertexOf(topo, v), o);
    return [dot(d, e1), dot(d, e2)];
  };
  const own = [0, 1, 2].map((k) => proj(topo.vertIds[3 * seed + k]));
  let ext = 0;
  for (let i = 0; i < 3; i++) ext = Math.max(ext, Math.hypot(own[i][0] - own[(i + 1) % 3][0], own[i][1] - own[(i + 1) % 3][1]));
  if (!(ext > 0)) return [];
  const mergeTol = 1e-3 * ext;
  const isNew = (p: [number, number], list: [number, number][]) => list.every((q) => Math.hypot(p[0] - q[0], p[1] - q[1]) > mergeTol);
  const distinct: [number, number][] = [];
  for (const p of own) if (isNew(p, distinct)) distinct.push(p);

  const sets: [number, number][][] = [];
  if (distinct.length === 3) sets.push(distinct);
  else if (distinct.length === 2) {
    const near = flood(ctx, seed, 16, (_r, to) =>
      ctx.s.member[to] === inSmooth && !isDegenerate(topo, to) && Math.abs(ndotv(topo, to, axis)) <= CYL_AXIS_SIN).tris;
    const thirds: [number, number][] = [];
    for (let i = 1; i < near.length && thirds.length < 2; i++) {
      for (let k = 0; k < 3 && thirds.length < 2; k++) {
        const p = proj(topo.vertIds[3 * near[i] + k]);
        if (isNew(p, distinct) && isNew(p, thirds)) thirds.push(p);
      }
    }
    for (const p of thirds) sets.push([...distinct, p]);
  }

  const out: CylModel[] = [];
  for (const pts of sets) {
    const c = fitCircle2D(pts.map((p) => p[0]), pts.map((p) => p[1]));
    if (!c || !(c.r > 0)) continue;
    const na = ndotv(topo, seed, axis);
    const m: CylModel = {
      axis, e1, e2, c0: add(o, add(scale(e1, c.cx), scale(e2, c.cy))), r: c.r, rms: 0, hole: false, spanCos: -1, spread: 0, tilt2: na * na,
    };
    orientModel(topo, [seed], m, maxSpan);
    out.push(m);
  }
  return out;
}

/** Zwei Modelle beschreiben (bis auf `rel` · Radius) denselben Zylinder. */
function sameCylinder(a: CylModel, b: CylModel, rel = 1e-6): boolean {
  const tol = rel * a.r;
  const d = sub(b.c0, a.c0);
  const off = len(sub(d, scale(a.axis, dot(d, a.axis))));
  return Math.abs(dot(a.axis, b.axis)) > 1 - Math.max(1e-10, (rel * rel) / 2) && Math.abs(a.r - b.r) < tol && off < tol &&
    a.hole === b.hole && Math.abs(a.spanCos - b.spanCos) < 1e-6;
}

interface Grown {
  region: Region;
  model: CylModel;
}

/**
 * Lässt den Zylinder vom Startmodell aus wachsen: abwechselnd alle Dreiecke sammeln, die auf dem
 * Zylinder liegen (Toleranz aus dem Rauschen der letzten Anpassung), und neu anpassen.
 */
function growCylinder(ctx: Ctx, seed: number, m0: CylModel, inSmooth: number, maxSpan: number): Grown | null {
  const topo = ctx.topo;
  const { member } = ctx.s;
  // Normalen, die nicht gemeinsam senkrecht zu einer Achse stehen (z. B. Kugel), früh verwerfen
  const tooTilted = (m: CylModel) => !(m.tilt2 <= 4 * CYL_MAX_TILT_SIN * CYL_MAX_TILT_SIN);
  let model = m0;
  let region: Region | null = null;
  for (let it = 0; it < GROW_ITER; it++) {
    const m = model;
    const tol = growTol(ctx, m);
    if (!onCylinder(topo, seed, m, tol)) return null;
    const next = flood(ctx, seed, ctx.limit, (_r, to) => member[to] === inSmooth && onCylinder(topo, to, m, tol));
    const refit = fitCylinderModel(ctx, next.tris, m.axis, maxSpan);
    if (!refit || tooTilted(refit) || !(refit.rms <= CYL_MAX_RMS * refit.r)) return null;
    const unchanged = region !== null && next.tris.length === region.tris.length;
    region = next;
    model = refit;
    // Region unverändert oder Modell praktisch gleich geblieben: weiteres Wachsen ändert nichts mehr
    if (unchanged || sameCylinder(m, refit)) break;
  }
  return region ? { region, model } : null;
}

/**
 * Anteil der Abweichungen, der sich durch einen über die Höhe veränderlichen Radius erklären lässt
 * (0 = gar nicht, 1 = vollständig). Verglichen werden zwei gewichtete Ausgleichsrechnungen der
 * radialen Abweichung e(θ, s) (s = Höhe, auf −1…1 normiert): mit den Termen, die nur eine kleine
 * Korrektur desselben Zylinders beschreiben (Radius, Lage und Neigung der Achse), und zusätzlich
 * mit s und s² (Kegel, Kugel, Tonne).
 */
function radiusTrend(es: Float64Array, ths: Float64Array, ss: Float64Array, ws: Float64Array): number {
  const P = 7;
  const G = new Float64Array(P * P);
  const b = new Float64Array(P);
  const phi = new Float64Array(P);
  let ee = 0;
  for (let i = 0; i < es.length; i++) {
    const w = ws[i];
    if (!(w > 0)) continue;
    const c = Math.cos(ths[i]);
    const s = Math.sin(ths[i]);
    const h = ss[i];
    phi[0] = 1; phi[1] = c; phi[2] = s; phi[3] = h * c; phi[4] = h * s; phi[5] = h; phi[6] = h * h;
    for (let j = 0; j < P; j++) {
      b[j] += w * phi[j] * es[i];
      for (let k = 0; k < P; k++) G[j * P + k] += w * phi[j] * phi[k];
    }
    ee += w * es[i] * es[i];
  }
  if (!(ee > 0)) return 0;
  const rss = (n: number) => {
    const A = new Float64Array(n * n);
    const y = new Float64Array(n);
    let tr = 0;
    for (let j = 0; j < n; j++) tr += G[j * P + j];
    for (let j = 0; j < n; j++) {
      y[j] = b[j];
      for (let k = 0; k < n; k++) A[j * n + k] = G[j * P + k];
      // Kleine Regularisierung: abhängige Terme (z. B. s² ≡ 1 bei nur zwei Höhen) stören nicht
      A[j * n + j] += 1e-9 * tr;
    }
    const x = solveLinear(A, y, n);
    if (!x) return ee;
    let fit = 0;
    for (let j = 0; j < n; j++) fit += x[j] * b[j];
    return Math.max(0, ee - fit);
  };
  const r0 = rss(5);
  const r1 = rss(7);
  return r0 > 0 ? Math.min(1, Math.max(0, (r0 - r1) / r0)) : 0;
}

/** Kennzahlen der fertigen Zylinderregion; null, wenn sie kein überzeugender Zylinder ist. */
function finishCylinder(ctx: Ctx, g: Grown, cosSmooth: number, planeArea: number): CylinderFeature | null {
  const topo = ctx.topo;
  const { vertIds, vertPos, neighbors, areas } = topo;
  const { region, model: m } = g;
  const { tris } = region;
  const tol = growTol(ctx, m);
  const eps = cylEps(ctx, m);
  if (!(m.r > 0)) return null;

  // Restabweichung (flächengewichtet) und Ausdehnung entlang der Achse
  const { verts, weights } = weightedVertices(ctx, tris);
  const es = new Float64Array(verts.length);
  const ths = new Float64Array(verts.length);
  const hs = new Float64Array(verts.length);
  let s2 = 0;
  let sw = 0;
  let hMin = Infinity;
  let hMax = -Infinity;
  for (let i = 0; i < verts.length; i++) {
    const p = toCyl(m, vertPos, 3 * verts[i]);
    es[i] = Math.hypot(p[0], p[1]) - m.r;
    ths[i] = Math.atan2(p[1], p[0]);
    hs[i] = p[2];
    s2 += weights[i] * es[i] * es[i];
    sw += weights[i];
    if (p[2] < hMin) hMin = p[2];
    if (p[2] > hMax) hMax = p[2];
  }
  const rms = sw > 0 ? Math.sqrt(s2 / sw) : 0;
  const length = hMax - hMin;
  if (!(rms <= CYL_EXACT_RMS * m.r + 2 * eps)) {
    // Nicht exakt: nur als verrauschter Zylinder zulassen, wenn die Abweichung zufällig aussieht
    if (!(rms <= CYL_MAX_RMS * m.r) || verts.length < STRUCT_MIN_POINTS) return null;
    const half = length / 2;
    const ss = Float64Array.from(hs, (h) => (half > 0 ? (h - hMin - half) / half : 0));
    if (radiusTrend(es, ths, ss, weights) > STRUCT_MAX) return null;
  }

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

  if (arc < CYL_MIN_ARC || !(area > 1.5 * planeArea) || m.spread < 0.01) return null;
  if (!(m.tilt2 <= CYL_MAX_TILT_SIN * CYL_MAX_TILT_SIN)) return null;

  // Rand der Region: Anteil, an dem die Fläche glatt weitergeht ("weich"). Für weiche Randkanten
  // quer zur Achse außerdem: Abstand von den Enden des Zylinders, an welchem Ende sie liegen und
  // wie hoch (entlang der Achse) die angrenzenden Dreiecke sind.
  const inRegion = mark(ctx, ctx.s.group, tris);
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
      if (nb >= 0 && ctx.s.group[nb] === inRegion) continue;
      // Nachbar liegt selbst auf dem Zylinder: die Region wurde nur durch maxTris abgeschnitten
      if (nb >= 0 && onCylinder(topo, nb, m, tol)) continue;
      const [u, w] = edgeVerts(topo, t, k);
      const ex = vertPos[3 * w] - vertPos[3 * u];
      const ey = vertPos[3 * w + 1] - vertPos[3 * u + 1];
      const ez = vertPos[3 * w + 2] - vertPos[3 * u + 2];
      const l = Math.hypot(ex, ey, ez);
      edge += l;
      if (nb < 0 || !(isDegenerate(topo, nb) || isDegenerate(topo, t) || ndot(topo, t, nb) >= cosSmooth)) continue;
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

const wrapAngle = (a: number) => a - TWO_PI * Math.round(a / TWO_PI);
const mod = (i: number, n: number) => ((i % n) + n) % n;

function detectCylinder(ctx: Ctx, seed: number, smooth: number[], planeArea: number, smoothAngle: number): CylinderFeature | null {
  const topo = ctx.topo;
  const inSmooth = mark(ctx, ctx.s.member, smooth);
  const maxSpan = 2 * smoothAngle;
  const cosSmooth = Math.cos(smoothAngle);
  const grown: CylModel[] = [];
  let best: CylinderFeature | null = null;
  for (const cand of axisCandidates(ctx, seed, smooth, inSmooth)) {
    if (Math.abs(ndotv(topo, seed, cand.axis)) > CYL_AXIS_SIN) continue;
    const local = localModel(ctx, seed, cand.axis, inSmooth, maxSpan);
    const starts = local ? [local] : [];
    // Startmodelle aus wenigen Punkten erst bei Bedarf (nur bei exakter Achsrichtung sinnvoll)
    if (cand.exact) starts.push(...minimalModels(ctx, seed, cand.axis, inSmooth, maxSpan));
    for (const m0 of starts) {
      if (grown.some((m) => sameCylinder(m, m0, 1e-4))) continue;
      const g = growCylinder(ctx, seed, m0, inSmooth, maxSpan);
      if (!g || grown.some((m) => sameCylinder(m, g.model, 1e-4))) continue;
      grown.push(g.model);
      const f = finishCylinder(ctx, g, cosSmooth, planeArea);
      if (!f) continue;
      if (!best || f.area > best.area) best = f;
      break;
    }
  }
  return best;
}

// ---------------------------------------------------------------------------------------------
// Ebene
// ---------------------------------------------------------------------------------------------

interface PlaneRegion {
  tris: number[];
  normal: Vec3;
  centroid: Vec3;
  area: number;
}

/**
 * Ebene Region um ein Dreieck: Dreiecke mit (fast) gleicher Normale, deren Schwerpunkt in der Ebene
 * liegt. Die Ebene wird danach aus der ganzen Region neu bestimmt und die Suche wiederholt; so
 * spielt es keine Rolle, ob das Startdreieck ein schmaler Splitter mit ungenauer Normale ist.
 */
function growPlane(
  ctx: Ctx, seed: number, cosPlane: number, distTol: number, limit: number, skip?: (t: number) => boolean, rounds = 4,
): PlaneRegion {
  const topo = ctx.topo;
  const { centroids } = topo;
  let n = normalOf(topo, seed);
  let d = dot(n, centroidOf(topo, seed));
  let res: PlaneRegion = { tris: [seed], normal: n, centroid: centroidOf(topo, seed), area: topo.areas[seed] };
  for (let it = 0; it < rounds; it++) {
    const pn = n;
    const pd = d;
    const tris = flood(ctx, seed, limit, (_r, to) => {
      if (skip && skip(to)) return false;
      const off = pn[0] * centroids[3 * to] + pn[1] * centroids[3 * to + 1] + pn[2] * centroids[3 * to + 2] - pd;
      return Math.abs(off) <= distTol && (isDegenerate(topo, to) || ndotv(topo, to, pn) >= cosPlane);
    }).tris;
    const st = regionStats(topo, tris);
    const same = it > 0 && tris.length === res.tris.length;
    res = { tris, normal: st.area > 0 ? st.normal : pn, centroid: st.centroid, area: st.area };
    if (same) break;
    n = res.normal;
    d = dot(n, res.centroid);
  }
  return res;
}

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
function planeAxes(ctx: Ctx, tris: number[], verts: number[], n: Vec3, centroid: Vec3): PlaneFeature['axes'] {
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
  return { u, v, size: [extentAlong(ctx.topo, verts, u), extentAlong(ctx.topo, verts, v)] };
}

function planeFeature(ctx: Ctx, tris: number[], verts: number[], smoothAngle: number): PlaneFeature {
  const { area, centroid, normal } = regionStats(ctx.topo, tris);
  const stamp = mark(ctx, ctx.s.member, tris);
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
  const f: PlaneFeature = { kind: 'plane', tris, centroid, normal, area, outline, axes: planeAxes(ctx, tris, verts, normal, centroid) };
  const circle = outlineCircle(outline, normal, smoothAngle);
  if (circle) f.circle = circle;
  return f;
}

interface PlaneTest {
  cosPlane: number;
  cosSmooth: number;
  distTol: number;
}

/**
 * Übergang von der ebenen Region P über die Kante (u, w) in die Nachbarfacette Q (und weiter in die
 * Facette R dahinter). Gehört P als Facette zu einer gekrümmten Fläche, knickt es zu Q ungefähr so
 * stark ab, wie es Krümmung (gemessen zwischen Q und R) und Breite von P und Q ergeben. Eine ebene
 * Fläche, an die eine Rundung tangential anschließt, knickt dagegen höchstens halb so stark ab
 * (die erste Facette der Rundung steht nur um ihren halben Facettenwinkel schräg).
 * Ergebnis: true, wenn der Übergang zu einer eigenen ebenen Fläche passt.
 */
function junctionIsFlat(
  ctx: Ctx, P: PlaneFeature, pVerts: number[], pStamp: number, q: number, u: number, w: number, pt: PlaneTest, qStamp: number,
): boolean {
  const topo = ctx.topo;
  const { member, group } = ctx.s;
  const limit = Math.min(ctx.limit, JUNCTION_MAX);
  const Q = growPlane(ctx, q, pt.cosPlane, pt.distTol, limit, (t) => member[t] === pStamp, 2);
  for (const t of Q.tris) group[t] = qStamp;
  const cPQ = dot(P.normal, Q.normal);
  // Kein Knick (nur versetzt oder durch maxTris abgeschnitten): P ist eine eigene Fläche
  if (cPQ > PARALLEL_COS) return true;
  const e = normalize(sub(vertexOf(topo, w), vertexOf(topo, u)));
  const qVerts = regionVertices(ctx, Q.tris);
  const wP = extentAlong(topo, pVerts, normalize(cross(P.normal, e)));
  const wQ = extentAlong(topo, qVerts, normalize(cross(Q.normal, e)));

  // Gegenüberliegende Kante von Q: weicher Übergang, möglichst parallel zur Kante P–Q
  let bestPar = 0.5;
  let r = -1;
  let rEdge: Vec3 = e;
  for (const t of Q.tris) {
    for (let k = 0; k < 3; k++) {
      const nb = topo.neighbors[3 * t + k];
      if (nb < 0 || member[nb] === pStamp || group[nb] === qStamp || isDegenerate(topo, nb)) continue;
      if (!(ndotv(topo, nb, Q.normal) >= pt.cosSmooth)) continue;
      const [a, b] = edgeVerts(topo, t, k);
      const d = sub(vertexOf(topo, b), vertexOf(topo, a));
      const l = len(d);
      if (!(l > 0)) continue;
      const par = Math.abs(dot(d, e)) / l;
      if (par > bestPar) {
        bestPar = par;
        r = nb;
        rEdge = scale(d, 1 / l);
      }
    }
  }
  // Q endet an harten Kanten: Knick zwischen zwei Flächen, P ist eine eigene Fläche
  if (r < 0) return true;
  const R = growPlane(ctx, r, pt.cosPlane, pt.distTol, limit, (t) => member[t] === pStamp || group[t] === qStamp, 2);
  const aQR = Math.acos(Math.min(1, dot(Q.normal, R.normal)));
  const wQ2 = extentAlong(topo, qVerts, normalize(cross(Q.normal, rEdge)));
  if (!(aQR > 1e-3) || !(wQ2 > 0)) return true;
  // Knick je Länge zwischen Q und R (Breite von R wie die von Q angenommen)
  const curvature = aQR / wQ2;
  const facetAngle = (curvature * (wP + wQ)) / 2;
  return Math.acos(Math.min(1, cPQ)) < FACET_RATIO * facetAngle;
}

/**
 * Ist die ebene Region eine eigene Fläche (true) oder nur eine Facette einer gekrümmten Fläche?
 * Ohne weiche Übergänge in Nachbardreiecke ist sie immer eine Fläche; sonst muss sie exakt eben sein
 * (fein facettierte Rundungen fasst die Winkeltoleranz sonst zu schmalen Streifen zusammen), und die
 * Übergänge an ihrem Rand müssen zu einer ebenen Fläche passen (siehe junctionIsFlat).
 */
function isFace(ctx: Ctx, P: PlaneFeature, pVerts: number[], pt: PlaneTest): boolean {
  const topo = ctx.topo;
  const pStamp = mark(ctx, ctx.s.member, P.tris);
  const edges: number[] = [];
  for (const t of P.tris) {
    for (let k = 0; k < 3; k++) {
      const nb = topo.neighbors[3 * t + k];
      if (nb < 0 || ctx.s.member[nb] === pStamp || isDegenerate(topo, nb)) continue;
      if (ndotv(topo, nb, P.normal) >= pt.cosSmooth) edges.push(3 * t + k);
    }
  }
  if (!edges.length) return true;

  // Eben? Größte Abweichung der Eckpunkte von der Ebene, bezogen auf die schmalere Seite
  const d0 = dot(P.normal, P.centroid);
  let dev = 0;
  for (const v of pVerts) dev = Math.max(dev, Math.abs(dot(P.normal, vertexOf(topo, v)) - d0));
  const minSize = Math.min(P.axes.size[0], P.axes.size[1]);
  if (!(dev <= FLAT_REL * minSize + FLOAT_EPS * ctx.s.coordMag + 1e-12)) return false;

  // Gleichmäßig verteilte Stichprobe der weichen Randkanten; je Nachbarfacette nur einmal rechnen
  const verdicts = new Map<number, boolean>();
  const step = Math.max(1, Math.ceil(edges.length / JUNCTION_SAMPLES));
  let flat = 0;
  let facet = 0;
  for (let i = 0; i < edges.length; i += step) {
    const t = Math.floor(edges[i] / 3);
    const k = edges[i] % 3;
    const nb = topo.neighbors[edges[i]];
    const [u, w] = edgeVerts(topo, t, k);
    const l = len(sub(vertexOf(topo, w), vertexOf(topo, u)));
    let v = verdicts.get(ctx.s.group[nb]);
    if (v === undefined) {
      const qStamp = ++ctx.s.stamp;
      v = junctionIsFlat(ctx, P, pVerts, pStamp, nb, u, w, pt, qStamp);
      verdicts.set(qStamp, v);
    }
    if (v) flat += l;
    else facet += l;
  }
  return flat >= facet;
}

// ---------------------------------------------------------------------------------------------
// Erkennung
// ---------------------------------------------------------------------------------------------

/** Doppelt gespeichertes Dreieck (seine Nachbarn kennen es nicht): das Original suchen. */
function originalOf(topo: Topology, t: number): number {
  const { neighbors, vertIds } = topo;
  for (let k = 0; k < 3; k++) {
    const nb = neighbors[3 * t + k];
    if (nb < 0) continue;
    if (neighbors[3 * nb] === t || neighbors[3 * nb + 1] === t || neighbors[3 * nb + 2] === t) return t;
    for (let j = 0; j < 3; j++) {
      const o = neighbors[3 * nb + j];
      if (o < 0) continue;
      const same = [0, 1, 2].every((i) => vertIds[3 * o] === vertIds[3 * t + i] || vertIds[3 * o + 1] === vertIds[3 * t + i] ||
        vertIds[3 * o + 2] === vertIds[3 * t + i]);
      if (same) return o;
    }
  }
  return t;
}

/** Entartetes Dreieck angeklickt: nächstgelegenes brauchbares Nachbardreieck verwenden. */
function usableSeed(ctx: Ctx, seedTri: number): number {
  const seed = originalOf(ctx.topo, seedTri);
  if (!isDegenerate(ctx.topo, seed)) return seed;
  const near = flood(ctx, seed, 64, () => true).tris;
  return near.find((t) => !isDegenerate(ctx.topo, t)) ?? -1;
}

/**
 * Bestimmt das Formelement, zu dem das angeklickte Dreieck gehört:
 * 1. Ebene: Dreiecke mit (fast) gleicher Normale in derselben Ebene wie das Startdreieck.
 * 2. Glatte Fläche: Ausbreitung über Kanten mit kleinem Knickwinkel.
 * 3. Ist die glatte Fläche deutlich größer als die Ebene, wird ein Zylinder angepasst.
 * 4. Sonst Ebene (mit Umriss, ggf. Kreis), wenn sie eine eigene Fläche ist und nicht nur eine
 *    Facette einer gekrümmten Fläche; andernfalls Freiformfläche.
 */
/**
 * Liegt die angeklickte ebene Region selbst auf dem gefundenen Zylinder? Bei einer Facette eines
 * facettierten Zylinders liegen ihre Eckpunkte genau auf dem Mantel. Ein echtes ebenes Stück zwischen
 * tangential anschließenden Rundungen (z. B. die kurze Seite eines stark abgerundeten Quadrats) weicht
 * dagegen deutlich ab – dann ist die angeklickte Fläche eine Ebene, kein Zylinder.
 */
function planeOnCylinder(ctx: Ctx, tris: number[], cyl: CylinderFeature): boolean {
  if (tris.length < 2) return true;
  const { vertPos, normals, neighbors } = ctx.topo;
  const verts = regionVertices(ctx, tris);
  const tol = Math.max(3 * cyl.rms, 2e-3 * cyl.radius);
  for (const v of verts) {
    const d: Vec3 = [vertPos[3 * v] - cyl.center[0], vertPos[3 * v + 1] - cyl.center[1], vertPos[3 * v + 2] - cyl.center[2]];
    const along = dot(d, cyl.axis);
    const radial = len([d[0] - along * cyl.axis[0], d[1] - along * cyl.axis[1], d[2] - along * cyl.axis[2]]);
    if (Math.abs(radial - cyl.radius) > tol) return false;
  }
  // Breite der Region quer zur Achse gegen den Knick zu den seitlichen Nachbarn: Eine Facette eines
  // facettierten Zylinders ist etwa r · Knickwinkel breit, ein ebenes Stück zwischen Rundungen viel breiter
  const n: Vec3 = [normals[3 * tris[0]], normals[3 * tris[0] + 1], normals[3 * tris[0] + 2]];
  const u = normalize(cross(cyl.axis, n));
  let lo = Infinity;
  let hi = -Infinity;
  for (const v of verts) {
    const x = vertPos[3 * v] * u[0] + vertPos[3 * v + 1] * u[1] + vertPos[3 * v + 2] * u[2];
    lo = Math.min(lo, x);
    hi = Math.max(hi, x);
  }
  const inRegion = new Set(tris);
  let sum = 0;
  let count = 0;
  for (const t of tris) {
    for (let k = 0; k < 3; k++) {
      const nb = neighbors[3 * t + k];
      if (nb < 0 || inRegion.has(nb)) continue;
      const m: Vec3 = [normals[3 * nb], normals[3 * nb + 1], normals[3 * nb + 2]];
      // Nur seitliche Nachbarn (Mantel), nicht die Stirnflächen
      if (Math.abs(dot(m, cyl.axis)) > 0.5) continue;
      sum += Math.acos(Math.min(1, Math.max(-1, dot(m, n))));
      count++;
    }
  }
  if (!count || sum <= 0) return true;
  return (hi - lo) / cyl.radius <= 2.2 * (sum / count);
}

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
  const pt: PlaneTest = {
    cosPlane: Math.cos(planeAngle),
    cosSmooth: Math.cos(smoothAngle),
    distTol: topo.diag * PLANE_DIST_REL + FLOAT_EPS * ctx.s.coordMag + 1e-9,
  };
  const plane = growPlane(ctx, seed, pt.cosPlane, pt.distTol, limit);

  // Glatte Fläche: Knickwinkel zwischen benachbarten Dreiecken klein
  const smooth = flood(ctx, seed, limit, (r, to) => isDegenerate(topo, to) || ndot(topo, r, to) >= pt.cosSmooth).tris;
  if (areaOf(topo, smooth) > 1.5 * plane.area) {
    const cyl = detectCylinder(ctx, seed, smooth, plane.area, smoothAngle);
    if (cyl && planeOnCylinder(ctx, plane.tris, cyl)) return cyl;
  }

  const pVerts = regionVertices(ctx, plane.tris);
  const pf = planeFeature(ctx, plane.tris, pVerts, smoothAngle);
  if (isFace(ctx, pf, pVerts, pt)) return pf;

  // Freiformfläche: Normalen nahe der Klick-Normale, ohne über Kanten zu springen
  const n0 = normalOf(topo, seed);
  const tris = flood(ctx, seed, limit, (r, to) =>
    isDegenerate(topo, to) || (ndotv(topo, to, n0) >= SURFACE_COS && ndot(topo, r, to) >= pt.cosSmooth)).tris;
  const { area, centroid, normal } = regionStats(topo, tris);
  return { kind: 'surface', tris, centroid, normal, area };
}

/**
 * Fläche in ein anderes Koordinatensystem übertragen (starre Bewegung T). Die Dreiecksliste bleibt
 * die des ursprünglichen Teils.
 */
export function transformFeature<F extends Feature>(f: F, T: Rigid): F {
  const p = (v: Vec3) => apply(T, v);
  const d = (v: Vec3) => rotate(T.R, v);
  if (f.kind === 'cylinder') return { ...f, center: p(f.center), axis: d(f.axis) };
  if (f.kind === 'edge') return { ...f, a: p(f.a), b: p(f.b), center: p(f.center), dir: d(f.dir), normal: d(f.normal) };
  if (f.kind === 'surface') return { ...f, centroid: p(f.centroid), normal: d(f.normal) };
  return {
    ...f,
    centroid: p(f.centroid),
    normal: d(f.normal),
    outline: f.outline.map(p),
    circle: f.circle ? { ...f.circle, center: p(f.circle.center) } : undefined,
    axes: { ...f.axes, u: d(f.axes.u), v: d(f.axes.v) },
  };
}

/**
 * Fläche aus frei gewählten Dreiecken (z. B. mit dem Pinsel bemalt): eben, wenn alle Dreiecke in einer
 * Ebene liegen (dann mit Umriss und Hauptachsen wie eine erkannte Fläche), sonst Freiformfläche mit
 * mittlerer Normale.
 */
export function featureFromTris(topo: Topology, tris: number[]): Feature {
  const list = [...new Set(tris)].filter((t) => t >= 0 && t < topo.triCount && !isDegenerate(topo, t));
  if (!list.length) throw new RangeError('Keine gültigen Dreiecke ausgewählt.');
  const { area, centroid, normal } = regionStats(topo, list);
  const ctx = makeCtx(topo, list.length + 1);
  const verts = regionVertices(ctx, list);
  // Eben: Normalen höchstens um den Ebenen-Winkel geneigt und Eckpunkte nahe der Ebene
  const cos = Math.cos(FEATURE_DEFAULTS.planeAngleDeg * 2 * DEG);
  const tol = topo.diag * PLANE_DIST_REL * 4 + FLOAT_EPS * ctx.s.coordMag + 1e-9;
  const flat = list.every((t) => ndotv(topo, t, normal) >= cos)
    && verts.every((v) => Math.abs(dot(sub(vertexOf(topo, v), centroid), normal)) <= tol);
  if (flat) return planeFeature(ctx, list, verts, FEATURE_DEFAULTS.smoothAngleDeg * DEG);
  return { kind: 'surface', tris: list, centroid, normal, area };
}
