/**
 * Platz für eine Beschriftung (z. B. die Nummer eines Stücks): die größte ebene Fläche unten (liegt auf
 * der Platte) oder oben, darin das größte Rechteck in der Größe der Schrift – entlang der Längsrichtung
 * der Fläche, mit etwas Rand und ohne Löcher (Bohrungen der Verbinder) zu treffen.
 */
import type { Vec3 } from './align';

export interface LabelSpot {
  /** Mitte der Schrift auf der Fläche (Weltkoordinaten). */
  point: Vec3;
  normal: Vec3;
  /** Richtung „oben“ der Schrift. */
  up: Vec3;
  /** Größte Schrifthöhe, die passt (höchstens die verlangte). */
  height: number;
  /** Fläche der gewählten Ebene (mm²). */
  area: number;
}

/**
 * positions: Dreiecke in Weltkoordinaten. size: Ausdehnung der Schrift je mm Schrifthöhe (Breite, Höhe).
 * Ohne passende Fläche (oder kleiner als minHeight): null.
 */
export function labelSpot(positions: Float32Array, side: 'bottom' | 'top', size: { w: number; h: number }, maxHeight: number, minHeight = 2): LabelSpot | null {
  const dir = side === 'bottom' ? -1 : 1;
  // Nach außen zeigende waagerechte Dreiecke, nach Höhe sortiert
  const levels = new Map<number, { area: number; tris: number[] }>();
  for (let t = 0; t < positions.length / 9; t++) {
    const o = 9 * t;
    const ux = positions[o + 3] - positions[o], uy = positions[o + 4] - positions[o + 1], uz = positions[o + 5] - positions[o + 2];
    const vx = positions[o + 6] - positions[o], vy = positions[o + 7] - positions[o + 1], vz = positions[o + 8] - positions[o + 2];
    const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const len = Math.hypot(nx, ny, nz);
    if (len < 1e-12 || (dir * nz) / len < 0.995) continue;
    const key = Math.round((positions[o + 2] + positions[o + 5] + positions[o + 8]) / 3 / 0.05);
    const l = levels.get(key) ?? { area: 0, tris: [] };
    l.area += len / 2;
    l.tris.push(t);
    levels.set(key, l);
  }
  let best: { key: number; area: number; tris: number[] } | null = null;
  for (const [key, l] of levels) if (!best || l.area > best.area) best = { key, ...l };
  if (!best) return null;
  const tris = best.tris;
  let z = 0;
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const t of tris) for (let k = 0; k < 3; k++) {
    const x = positions[9 * t + 3 * k], y = positions[9 * t + 3 * k + 1];
    x0 = Math.min(x0, x); y0 = Math.min(y0, y); x1 = Math.max(x1, x); y1 = Math.max(y1, y);
    z += positions[9 * t + 3 * k + 2];
  }
  z /= 3 * tris.length;
  // Raster der Fläche
  const cell = Math.max(0.2, Math.max(x1 - x0, y1 - y0) / 250);
  const gx = x0 - cell, gy = y0 - cell;
  const nx = Math.ceil((x1 - x0) / cell) + 3, ny = Math.ceil((y1 - y0) / cell) + 3;
  const inside = new Uint8Array(nx * ny);
  for (const t of tris) {
    const ax = positions[9 * t], ay = positions[9 * t + 1];
    const bx = positions[9 * t + 3], by = positions[9 * t + 4];
    const cx = positions[9 * t + 6], cy = positions[9 * t + 7];
    const det = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
    if (Math.abs(det) < 1e-12) continue;
    const i0 = Math.max(0, Math.floor((Math.min(ax, bx, cx) - gx) / cell)), i1 = Math.min(nx - 1, Math.ceil((Math.max(ax, bx, cx) - gx) / cell));
    const j0 = Math.max(0, Math.floor((Math.min(ay, by, cy) - gy) / cell)), j1 = Math.min(ny - 1, Math.ceil((Math.max(ay, by, cy) - gy) / cell));
    for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) {
      const px = gx + (i + 0.5) * cell, py = gy + (j + 0.5) * cell;
      const l1 = ((bx - px) * (cy - py) - (cx - px) * (by - py)) / det;
      const l2 = ((cx - px) * (ay - py) - (ax - px) * (cy - py)) / det;
      if (l1 >= -1e-9 && l2 >= -1e-9 && 1 - l1 - l2 >= -1e-9) inside[j * nx + i] = 1;
    }
  }
  // Abstand zum Rand (Fasenmaß 1 / √2, in Zellen)
  const dist = new Float32Array(nx * ny);
  for (let k = 0; k < dist.length; k++) dist[k] = inside[k] ? 1e9 : 0;
  const D = Math.SQRT2;
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
    const k = j * nx + i;
    if (!dist[k]) continue;
    let d = dist[k];
    if (i > 0) d = Math.min(d, dist[k - 1] + 1);
    if (j > 0) {
      d = Math.min(d, dist[k - nx] + 1);
      if (i > 0) d = Math.min(d, dist[k - nx - 1] + D);
      if (i < nx - 1) d = Math.min(d, dist[k - nx + 1] + D);
    }
    dist[k] = d;
  }
  for (let j = ny - 1; j >= 0; j--) for (let i = nx - 1; i >= 0; i--) {
    const k = j * nx + i;
    if (!dist[k]) continue;
    let d = dist[k];
    if (i < nx - 1) d = Math.min(d, dist[k + 1] + 1);
    if (j < ny - 1) {
      d = Math.min(d, dist[k + nx] + 1);
      if (i < nx - 1) d = Math.min(d, dist[k + nx + 1] + D);
      if (i > 0) d = Math.min(d, dist[k + nx - 1] + D);
    }
    dist[k] = d;
  }
  // Längsrichtung der Fläche (Hauptachse)
  let n = 0, mx = 0, my = 0;
  for (let k = 0; k < inside.length; k++) if (inside[k]) { n++; mx += k % nx; my += Math.floor(k / nx); }
  if (!n) return null;
  mx /= n; my /= n;
  let sxx = 0, syy = 0, sxy = 0;
  for (let k = 0; k < inside.length; k++) if (inside[k]) {
    const dx = (k % nx) - mx, dy = Math.floor(k / nx) - my;
    sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
  }
  const theta = 0.5 * Math.atan2(2 * sxy, sxx - syy);
  // Kandidaten: örtliche Maxima des Abstands, die weitesten zuerst (bei Gleichstand nahe der Mitte)
  const cand: number[] = [];
  for (let j = 1; j < ny - 1; j++) for (let i = 1; i < nx - 1; i++) {
    const k = j * nx + i;
    const d = dist[k];
    if (!d) continue;
    let top = true;
    for (let dj = -1; dj <= 1 && top; dj++) for (let di = -1; di <= 1; di++) if (dist[k + dj * nx + di] > d) { top = false; break; }
    if (top) cand.push(k);
  }
  const off = (k: number) => Math.hypot((k % nx) - mx, Math.floor(k / nx) - my);
  cand.sort((a, b) => dist[b] - dist[a] || off(a) - off(b));
  const isIn = (x: number, y: number) => {
    const i = Math.floor((x - gx) / cell), j = Math.floor((y - gy) / cell);
    // Eine Zelle Abstand zum Rand: das Raster soll die Fläche nie größer machen, als sie ist
    return i >= 0 && j >= 0 && i < nx && j < ny && dist[j * nx + i] >= 1.5;
  };
  const margin = (h: number) => 0.12 * h + 0.6;
  const fits = (cx: number, cy: number, ang: number, h: number) => {
    const hw = (size.w * h) / 2 + margin(h);
    const hh = (size.h * h) / 2 + margin(h);
    const c = Math.cos(ang), s = Math.sin(ang);
    const step = cell * 0.9;
    for (let a = -hw; a <= hw + 1e-9; a += Math.min(step, hw)) {
      for (let b = -hh; b <= hh + 1e-9; b += Math.min(step, hh)) {
        if (!isIn(cx + a * c - b * s, cy + a * s + b * c)) return false;
      }
    }
    return true;
  };
  let spot: { x: number; y: number; ang: number; h: number } | null = null;
  for (const k of cand.slice(0, 30)) {
    const cx = gx + ((k % nx) + 0.5) * cell;
    const cy = gy + (Math.floor(k / nx) + 0.5) * cell;
    const r = dist[k] * cell;
    for (const ang of [theta, theta + Math.PI / 2]) {
      // Das Rechteck enthält einen Kreis mit seiner halben kürzeren Seite: größer als der Abstand zum Rand passt nie
      let hi = Math.min(maxHeight, (2 * r) / Math.min(size.w, size.h));
      let lo = 0;
      if (spot && hi <= spot.h) continue;
      if (fits(cx, cy, ang, hi)) lo = hi;
      else for (let it = 0; it < 12; it++) {
        const mid = (lo + hi) / 2;
        if (fits(cx, cy, ang, mid)) lo = mid;
        else hi = mid;
      }
      if (lo > 0 && (!spot || lo > spot.h + 1e-6)) spot = { x: cx, y: cy, ang, h: lo };
    }
    if (spot && spot.h >= maxHeight - 1e-6) break;
  }
  if (!spot || spot.h < minHeight) return null;
  // Leserichtung möglichst nach +x (bzw. +y), „oben“ = n × Leserichtung
  let ux = Math.cos(spot.ang), uy = Math.sin(spot.ang);
  if (ux < -1e-6 || (Math.abs(ux) <= 1e-6 && uy < 0)) { ux = -ux; uy = -uy; }
  const normal: Vec3 = [0, 0, dir];
  const up: Vec3 = [-dir * uy, dir * ux, 0];
  return { point: [spot.x, spot.y, z], normal, up, height: spot.h, area: best.area };
}
