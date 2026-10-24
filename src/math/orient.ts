/**
 * Druckausrichtung: Eine Baugruppe so drehen, dass eine Fläche unten auf der Druckplatte liegt –
 * von Hand gewählt oder automatisch die mit der größten Auflage.
 *
 * Auflage für eine Richtung n (nach unten zeigend): alle Dreiecke, deren Normale n ist und die in
 * der Stützebene liegen (am weitesten in Richtung n). Kandidaten für n sind die Normalenrichtungen
 * mit der größten Gesamtfläche.
 */
import { type Rigid, type Vec3, IDENTITY, cross, dot, len, normalize, rotationAbout } from './align';

const DOWN: Vec3 = [0, 0, -1];

/** Drehung um pivot, die die Richtung n nach unten (−z) dreht – auf kürzestem Weg. */
export function layFlat(n: Vec3, pivot: Vec3): Rigid {
  const u = normalize(n);
  const c = dot(u, DOWN);
  if (c > 1 - 1e-9) return IDENTITY;
  // Zeigt die Fläche nach oben: um eine waagerechte Achse wenden
  const axis = c < -1 + 1e-9 ? ([1, 0, 0] as Vec3) : normalize(cross(u, DOWN));
  return rotationAbout(pivot, axis, Math.acos(Math.max(-1, Math.min(1, c))));
}

export interface BaseChoice {
  /** Richtung (Weltkoordinaten), die nach unten soll. */
  normal: Vec3;
  /** Auflagefläche in dieser Lage (mm²). */
  area: number;
  /** Auflagefläche in der jetzigen Lage (nach unten, −z). */
  current: number;
}

/**
 * Beste Auflage eines Netzes (nicht indizierte Dreiecke in Weltkoordinaten): größte Auflagefläche;
 * bei fast gleicher Fläche die Lage mit dem tieferen Schwerpunkt (steht stabiler).
 */
export function bestBase(positions: Float32Array, opts: { candidates?: number } = {}): BaseChoice {
  const n = positions.length / 9;
  const normals = new Float32Array(n * 3);
  const areas = new Float32Array(n);
  const cents = new Float32Array(n * 3);
  const lo = [Infinity, Infinity, Infinity];
  const hi = [-Infinity, -Infinity, -Infinity];
  // Schwerpunkt der Oberfläche (für die Standfestigkeit)
  const g: Vec3 = [0, 0, 0];
  let total = 0;
  for (let t = 0; t < n; t++) {
    const o = t * 9;
    const ax = positions[o + 3] - positions[o], ay = positions[o + 4] - positions[o + 1], az = positions[o + 5] - positions[o + 2];
    const bx = positions[o + 6] - positions[o], by = positions[o + 7] - positions[o + 1], bz = positions[o + 8] - positions[o + 2];
    const cx = ay * bz - az * by, cy = az * bx - ax * bz, cz = ax * by - ay * bx;
    const l = Math.hypot(cx, cy, cz);
    areas[t] = l / 2;
    if (l > 0) {
      normals[t * 3] = cx / l;
      normals[t * 3 + 1] = cy / l;
      normals[t * 3 + 2] = cz / l;
    }
    for (let k = 0; k < 3; k++) {
      const c = (positions[o + k] + positions[o + 3 + k] + positions[o + 6 + k]) / 3;
      cents[t * 3 + k] = c;
      g[k] += c * areas[t];
      for (let v = 0; v < 3; v++) {
        lo[k] = Math.min(lo[k], positions[o + v * 3 + k]);
        hi[k] = Math.max(hi[k], positions[o + v * 3 + k]);
      }
    }
    total += areas[t];
  }
  if (total > 0) for (let k = 0; k < 3; k++) g[k] /= total;
  const diag = Math.hypot(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]) || 1;
  const tol = 0.02 + diag * 1e-4;
  const cosTol = Math.cos((2 * Math.PI) / 180);

  // Normalenrichtungen bündeln (Raster von etwa 1°), Fläche und mittlere Richtung je Bündel
  const K = 60;
  const bins = new Map<string, { area: number; sum: Vec3 }>();
  for (let t = 0; t < n; t++) {
    if (!(areas[t] > 0)) continue;
    const nx = normals[t * 3], ny = normals[t * 3 + 1], nz = normals[t * 3 + 2];
    const key = `${Math.round(nx * K)},${Math.round(ny * K)},${Math.round(nz * K)}`;
    const b = bins.get(key);
    const a = areas[t];
    if (b) {
      b.area += a;
      b.sum[0] += nx * a;
      b.sum[1] += ny * a;
      b.sum[2] += nz * a;
    } else bins.set(key, { area: a, sum: [nx * a, ny * a, nz * a] });
  }
  const dirs = [...bins.values()]
    .sort((a, b) => b.area - a.area)
    .slice(0, opts.candidates ?? 40)
    .map((b) => normalize(b.sum))
    .filter((d) => len(d) > 0.5);

  /** Auflagefläche und Schwerpunkthöhe, wenn d nach unten zeigt. */
  const contact = (d: Vec3) => {
    let support = -Infinity;
    for (let i = 0; i < positions.length; i += 3) support = Math.max(support, positions[i] * d[0] + positions[i + 1] * d[1] + positions[i + 2] * d[2]);
    let area = 0;
    for (let t = 0; t < n; t++) {
      if (normals[t * 3] * d[0] + normals[t * 3 + 1] * d[1] + normals[t * 3 + 2] * d[2] < cosTol) continue;
      if (cents[t * 3] * d[0] + cents[t * 3 + 1] * d[1] + cents[t * 3 + 2] * d[2] < support - tol) continue;
      area += areas[t];
    }
    return { area, height: support - dot(g, d) };
  };

  const current = contact(DOWN);
  let best = { normal: DOWN, ...current };
  for (const d of dirs) {
    const c = contact(d);
    const better = c.area > best.area * 1.02 || (c.area > best.area * 0.98 && c.height < best.height - tol);
    if (better) best = { normal: d, ...c };
  }
  return { normal: best.normal, area: best.area, current: current.area };
}
