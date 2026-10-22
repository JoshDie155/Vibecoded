/**
 * Umriss-Abgleich für zwei aufeinanderliegende ebene Flächen: Bei Fläche auf Fläche sind Drehung und
 * Verschiebung in der Ebene frei, der Flächenkontakt verrät darüber nichts. Haben beide Flächen
 * denselben Umriss (typisch bei zerlegten Modellen: Schnittfläche auf Schnittfläche), werden die
 * Umrisse in der Ebene exakt übereinandergelegt (2D-ICP).
 */
import { type Rigid, type Vec3, apply, compose, cross, dot, normalize, rotationAbout, sub } from './align';

/** Gleichmäßig verteilte Punkte entlang eines geschlossenen Umrisses. */
export function resampleLoop(loop: Vec3[], count: number): Vec3[] {
  const n = loop.length;
  if (n < 2) return loop.slice();
  const seg: number[] = [];
  let total = 0;
  for (let i = 0; i < n; i++) {
    const a = loop[i];
    const b = loop[(i + 1) % n];
    const l = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    seg.push(l);
    total += l;
  }
  if (total <= 0) return loop.slice();
  const out: Vec3[] = [];
  let i = 0;
  let acc = 0;
  for (let k = 0; k < count; k++) {
    const d = (k / count) * total;
    while (acc + seg[i] < d && i < n - 1) acc += seg[i++];
    const t = seg[i] > 0 ? (d - acc) / seg[i] : 0;
    const a = loop[i];
    const b = loop[(i + 1) % n];
    out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]);
  }
  return out;
}

/** Nächster Punkt auf einem geschlossenen Polygonzug. */
function closestOnLoop(p: Vec3, loop: Vec3[]): Vec3 {
  let best: Vec3 = loop[0];
  let bd = Infinity;
  for (let i = 0; i < loop.length; i++) {
    const a = loop[i];
    const b = loop[(i + 1) % loop.length];
    const ab = sub(b, a);
    const l2 = dot(ab, ab);
    const t = l2 > 0 ? Math.min(1, Math.max(0, dot(sub(p, a), ab) / l2)) : 0;
    const c: Vec3 = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
    const d = (c[0] - p[0]) ** 2 + (c[1] - p[1]) ** 2 + (c[2] - p[2]) ** 2;
    if (d < bd) { bd = d; best = c; }
  }
  return best;
}

/**
 * @param srcLoop Umriss am beweglichen Teil (lokal)
 * @param dstLoop Umriss am festen Teil (im Lösungs-Koordinatensystem)
 * @param T       aktuelle Lage des beweglichen Teils (Flächen liegen bereits aufeinander)
 * @param normal  Normale der festen Fläche
 * @param tol     größte zulässige Restabweichung; passen die Umrisse schlechter, wird nichts geändert
 */
export function alignOutlines(
  srcLoop: Vec3[],
  dstLoop: Vec3[],
  T: Rigid,
  normal: Vec3,
  tol: number,
  /** Für schnelle Bewertungen: weniger Punkte und Iterationen. */
  opts: { samples?: number; maxDst?: number; iterations?: number } = {},
): { T: Rigid; rms: number } | null {
  if (srcLoop.length < 3 || dstLoop.length < 3) return null;
  const n = normalize(normal);
  const u = normalize(Math.abs(n[0]) < 0.9 ? cross(n, [1, 0, 0]) : cross(n, [0, 1, 0]));
  const v = cross(n, u);
  const src = resampleLoop(srcLoop, opts.samples ?? 240);
  const maxDst = opts.maxDst ?? 4000;
  const dst = dstLoop.length > maxDst ? resampleLoop(dstLoop, maxDst) : dstLoop;
  let cur = T;
  let rms = Infinity;
  for (let it = 0; it < (opts.iterations ?? 40); it++) {
    const P = src.map((p) => apply(cur, p));
    const Q = P.map((p) => closestOnLoop(p, dst));
    // 2D-Procrustes in der Ebene (u, v): Drehung um die Normale und Verschiebung
    let pu = 0, pv = 0, qu = 0, qv = 0;
    for (let i = 0; i < P.length; i++) {
      pu += dot(P[i], u); pv += dot(P[i], v); qu += dot(Q[i], u); qv += dot(Q[i], v);
    }
    const m = P.length;
    pu /= m; pv /= m; qu /= m; qv /= m;
    let sxx = 0, sxy = 0, err = 0;
    for (let i = 0; i < m; i++) {
      const a = [dot(P[i], u) - pu, dot(P[i], v) - pv];
      const b = [dot(Q[i], u) - qu, dot(Q[i], v) - qv];
      sxx += a[0] * b[0] + a[1] * b[1];
      sxy += a[0] * b[1] - a[1] * b[0];
      err += (P[i][0] - Q[i][0]) ** 2 + (P[i][1] - Q[i][1]) ** 2 + (P[i][2] - Q[i][2]) ** 2;
    }
    rms = Math.sqrt(err / m);
    const angle = Math.atan2(sxy, sxx);
    // Drehen um den Schwerpunkt der Punkte (Achse parallel zur Normalen), dann den Schwerpunkt verschieben
    const pivot: Vec3 = [pu * u[0] + pv * v[0], pu * u[1] + pv * v[1], pu * u[2] + pv * v[2]];
    const du = qu - pu;
    const dv = qv - pv;
    const shift: Rigid = { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: [du * u[0] + dv * v[0], du * u[1] + dv * v[1], du * u[2] + dv * v[2]] };
    cur = compose(shift, compose(rotationAbout(pivot, n, angle), cur));
    if (Math.abs(angle) < 1e-9 && Math.hypot(du, dv) < 1e-9) break;
  }
  // Restabweichung nach dem letzten Schritt
  const P = src.map((p) => apply(cur, p));
  let err = 0;
  for (const p of P) {
    const q = closestOnLoop(p, dst);
    err += (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2;
  }
  rms = Math.sqrt(err / P.length);
  return rms <= tol ? { T: cur, rms } : null;
}
