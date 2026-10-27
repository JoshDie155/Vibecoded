/**
 * Standfestigkeit: Steht ein Modell sicher, wenn es so wie jetzt auf dem Tisch steht? Der Schwerpunkt muss
 * über der Standfläche liegen – der konvexen Hülle aller Punkte, die den Boden berühren. Wie weit er vom
 * Rand entfernt ist, im Verhältnis zu seiner Höhe, ergibt den Winkel, ab dem das Modell kippt.
 */
import type { Vec3 } from './align';

export type P2 = [number, number];

/** Konvexe Hülle (gegen den Uhrzeigersinn, ohne doppelte Punkte), monotone Kette nach Andrew. */
export function convexHull(points: P2[]): P2[] {
  const pts = [...points].sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const uniq: P2[] = [];
  for (const p of pts) if (!uniq.length || p[0] !== uniq[uniq.length - 1][0] || p[1] !== uniq[uniq.length - 1][1]) uniq.push(p);
  if (uniq.length < 3) return uniq;
  const cross = (o: P2, a: P2, b: P2) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower: P2[] = [];
  for (const p of uniq) {
    while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop();
    lower.push(p);
  }
  const upper: P2[] = [];
  for (let i = uniq.length - 1; i >= 0; i--) {
    const p = uniq[i];
    while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop();
    upper.push(p);
  }
  return lower.slice(0, -1).concat(upper.slice(0, -1));
}

function toSegment(p: P2, a: P2, b: P2): { d: number; q: P2 } {
  const dx = b[0] - a[0];
  const dy = b[1] - a[1];
  const l2 = dx * dx + dy * dy;
  const t = l2 > 0 ? Math.max(0, Math.min(1, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2)) : 0;
  const q: P2 = [a[0] + t * dx, a[1] + t * dy];
  return { d: Math.hypot(p[0] - q[0], p[1] - q[1]), q };
}

export interface Stability {
  /** Schwerpunkt über der Standfläche? */
  inside: boolean;
  /** Abstand des Schwerpunkts (von oben gesehen) zum Rand der Standfläche – negativ: außerhalb (mm). */
  margin: number;
  /** Höhe des Schwerpunkts über dem Boden (mm). */
  height: number;
  /** Neigung, ab der das Modell kippt (Grad); negativ: kippt schon so. */
  tipDeg: number;
  /** Nächster Punkt am Rand der Standfläche (dorthin kippt es am ehesten). */
  edge: P2;
}

/**
 * @param com Schwerpunkt (Welt)
 * @param hull Standfläche (konvexe Hülle, gegen den Uhrzeigersinn)
 * @param ground Höhe des Bodens
 */
export function stabilityOf(com: Vec3, hull: P2[], ground: number): Stability {
  const p: P2 = [com[0], com[1]];
  const height = Math.max(0, com[2] - ground);
  let best = { d: Infinity, q: p };
  if (hull.length === 1) best = { d: Math.hypot(p[0] - hull[0][0], p[1] - hull[0][1]), q: hull[0] };
  for (let i = 0; i < hull.length && hull.length > 1; i++) {
    const s = toSegment(p, hull[i], hull[(i + 1) % hull.length]);
    if (s.d < best.d) best = s;
  }
  // Innen: links von allen Kanten (gegen den Uhrzeigersinn)
  let inside = hull.length >= 3;
  for (let i = 0; i < hull.length && inside; i++) {
    const a = hull[i];
    const b = hull[(i + 1) % hull.length];
    if ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < 0) inside = false;
  }
  const margin = hull.length ? (inside ? best.d : -best.d) : -Infinity;
  const tipDeg = (Math.atan2(margin, Math.max(height, 1e-9)) * 180) / Math.PI;
  return { inside, margin, height, tipDeg, edge: best.q };
}
