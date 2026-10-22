/**
 * Automatische Vorschläge: Welche Flächen zweier Teile gehören zusammen? Schnittflächen eines
 * zerlegten Modells sind (fast) deckungsgleich – gleiche Fläche, gleicher Umfang, gleiche
 * Abmessungen. Zapfen und Bohrung haben fast denselben Durchmesser. Gute Kandidaten werden
 * anschließend wirklich aufeinandergelegt (Umriss-Abgleich), das Ergebnis bestimmt die Bewertung.
 */
import { type Mat3, type Rigid, type Vec3, apply, sub } from './align';
import type { Feature, PlaneFeature } from './features';
import { alignOutlines } from './outline';

export interface FacePair {
  fixed: Feature;
  moving: Feature;
  /** Ähnlichkeit der Form (0…1). */
  shape: number;
}

const ratio = (a: number, b: number) => (a > 0 && b > 0 ? Math.min(a, b) / Math.max(a, b) : 0);

function perimeter(loop: Vec3[]): number {
  let s = 0;
  for (let i = 0; i < loop.length; i++) {
    const a = loop[i];
    const b = loop[(i + 1) % loop.length];
    s += Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
  }
  return s;
}

/**
 * Trägheitsradien des Umrisses (Wurzeln der Eigenwerte der Flächenträgheit, größerer zuerst) –
 * anders als die Ausdehnung entlang der Hauptachsen unabhängig davon, wie die Achsen bei fast
 * quadratischen oder runden Flächen zufällig liegen.
 */
export function gyrationRadii(f: PlaneFeature): [number, number] {
  const { u, v } = f.axes;
  const pts = f.outline.map((p) => [p[0] * u[0] + p[1] * u[1] + p[2] * u[2], p[0] * v[0] + p[1] * v[1] + p[2] * v[2]]);
  // Flächenmomente eines Polygons (Gaußsche Trapezformel)
  let A = 0, cx = 0, cy = 0, ixx = 0, iyy = 0, ixy = 0;
  for (let i = 0; i < pts.length; i++) {
    const [x0, y0] = pts[i];
    const [x1, y1] = pts[(i + 1) % pts.length];
    const c = x0 * y1 - x1 * y0;
    A += c;
    cx += (x0 + x1) * c;
    cy += (y0 + y1) * c;
    ixx += (y0 * y0 + y0 * y1 + y1 * y1) * c;
    iyy += (x0 * x0 + x0 * x1 + x1 * x1) * c;
    ixy += (x0 * y1 + 2 * x0 * y0 + 2 * x1 * y1 + x1 * y0) * c;
  }
  A /= 2;
  if (Math.abs(A) < 1e-12) return [0, 0];
  cx /= 6 * A;
  cy /= 6 * A;
  // Auf den Schwerpunkt bezogen (Steiner)
  const sxx = iyy / 12 / A - cx * cx; // ∫x² / A
  const syy = ixx / 12 / A - cy * cy; // ∫y² / A
  const sxy = ixy / 24 / A - cx * cy;
  const m = (sxx + syy) / 2;
  const d = Math.sqrt(((sxx - syy) / 2) ** 2 + sxy * sxy);
  return [Math.sqrt(Math.max(0, m + d)), Math.sqrt(Math.max(0, m - d))];
}

/** Wie gut passen zwei Flächen der Form nach zusammen (0 = gar nicht, 1 = identisch)? */
export function shapeSimilarity(a: Feature, b: Feature): number {
  if (a.kind === 'plane' && b.kind === 'plane') {
    if (!!a.circle !== !!b.circle) return 0;
    const [a1, a2] = gyrationRadii(a);
    const [b1, b2] = gyrationRadii(b);
    let s = ratio(a.area, b.area) * ratio(perimeter(a.outline), perimeter(b.outline)) * ratio(a1, b1) * ratio(a2, b2);
    if (a.circle && b.circle) s *= ratio(a.circle.radius, b.circle.radius) ** 2;
    return s;
  }
  if (a.kind === 'cylinder' && b.kind === 'cylinder') {
    const length = ratio(a.length, b.length) ** 0.3;
    if (a.hole !== b.hole) {
      const hole = a.hole ? a : b;
      const peg = a.hole ? b : a;
      const gap = hole.radius - peg.radius;
      // Übliches Spiel für Steckverbindungen: 0 … 0,3 mm; dicker als die Bohrung geht nicht
      if (gap < -0.05 - 0.01 * hole.radius) return 0;
      return Math.exp(-Math.max(0, gap - 0.3) / 0.3) * length;
    }
    // Zwei Zapfen (z. B. geteilte Säule) oder zwei Bohrungen: gleicher Durchmesser
    return 0.6 * ratio(a.radius, b.radius) ** 8 * length;
  }
  return 0;
}

/** Die vielversprechendsten Flächenpaare (fest ↔ beweglich), beste zuerst. */
export function candidatePairs(fixed: Feature[], moving: Feature[], opts: { minShape?: number; max?: number } = {}): FacePair[] {
  const out: FacePair[] = [];
  for (const f of fixed) for (const m of moving) {
    const shape = shapeSimilarity(f, m);
    if (shape >= (opts.minShape ?? 0.85)) out.push({ fixed: f, moving: m, shape });
  }
  out.sort((a, b) => b.shape - a.shape);
  return out.slice(0, opts.max ?? 12);
}

/** Drehung, die die Spalten von `from` auf die Spalten von `to` abbildet (beides Orthonormalbasen). */
function basisRotation(to: [Vec3, Vec3, Vec3], from: [Vec3, Vec3, Vec3]): Mat3 {
  const R = [0, 0, 0, 0, 0, 0, 0, 0, 0] as Mat3;
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
    let s = 0;
    for (let k = 0; k < 3; k++) s += to[k][i] * from[k][j];
    R[i * 3 + j] = s;
  }
  return R;
}

/**
 * Legt zwei ebene Flächen Fläche auf Fläche (Normalen entgegengesetzt) und richtet die Umrisse aus.
 * Probiert die vier Zuordnungen der Hauptachsen und behält die mit der kleinsten Restabweichung.
 * @returns Lage des beweglichen Teils im Koordinatensystem des festen Teils und Umriss-Abweichung (mm)
 */
export function placePlanePair(fixed: PlaneFeature, moving: PlaneFeature, fast = false): { T: Rigid; rms: number } | null {
  const size = Math.max(...fixed.axes.size, ...moving.axes.size, 1e-6);
  const from: [Vec3, Vec3, Vec3] = [moving.axes.u, moving.axes.v, moving.normal];
  const nf = fixed.normal;
  const neg = (v: Vec3): Vec3 => [-v[0], -v[1], -v[2]];
  const targets: [Vec3, Vec3, Vec3][] = [];
  // Sind die Hauptachsen nicht eindeutig (fast quadratisch, rund, regelmäßiges Vieleck), zusätzlich in 45°-Schritten
  const [g1, g2] = gyrationRadii(fixed);
  const steps = g2 > 0.9 * g1 ? 8 : 2;
  for (let k = 0; k < steps; k++) {
    const a = (k * 2 * Math.PI) / steps;
    const c = Math.cos(a);
    const sn = Math.sin(a);
    // u, v der festen Fläche um die Normale gedreht
    const su: Vec3 = [fixed.axes.u[0] * c + fixed.axes.v[0] * sn, fixed.axes.u[1] * c + fixed.axes.v[1] * sn, fixed.axes.u[2] * c + fixed.axes.v[2] * sn];
    const sv: Vec3 = [fixed.axes.v[0] * c - fixed.axes.u[0] * sn, fixed.axes.v[1] * c - fixed.axes.u[1] * sn, fixed.axes.v[2] * c - fixed.axes.u[2] * sn];
    targets.push([su, neg(sv), neg(nf)]);
    // Bei fast gleich langen Achsen auch vertauscht
    targets.push([sv, su, neg(nf)]);
  }
  let best: { T: Rigid; rms: number } | null = null;
  for (const to of targets) {
    const R = basisRotation(to, from);
    const c = apply({ R, t: [0, 0, 0] }, moving.centroid);
    const T0: Rigid = { R, t: sub(fixed.centroid, c) };
    const res = alignOutlines(moving.outline, fixed.outline, T0, nf, 0.25 * size, fast ? { samples: 120, maxDst: 600, iterations: 20 } : {});
    const cand = res ?? null;
    if (cand && (!best || cand.rms < best.rms)) best = cand;
  }
  return best;
}
