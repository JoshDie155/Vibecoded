/**
 * Löst eine "Verbindung": Markierungen auf einem beweglichen Teil (src) sollen auf die
 * Markierungen eines festen Teils (dst) fallen.
 *
 * - Zuordnung automatisch: Welche Markierung zu welcher gehört, wird über alle möglichen
 *   Zuordnungen bestimmt. Abstände zwischen Punkten bleiben bei starren Bewegungen erhalten,
 *   deshalb passt nur die richtige Zuordnung wirklich gut (kleinste Restabweichung).
 * - Flächennormalen: An einer Kontaktstelle zeigen die Oberflächen der beiden Teile genau
 *   gegeneinander. Das wird über Hilfspunkte (Punkt + Normale) mit ausgerichtet. Damit reichen
 *   schon 1 oder 2 Markierungen, und spiegelsymmetrische Fälle werden eindeutig.
 */
import {
  type Rigid, type Vec3, add, apply, compose, dist, normalize, rotationAbout, scale, solveRigid, sub,
} from './align';

export interface Marker {
  p: Vec3;
  /** Nach außen zeigende Flächennormale an der Markierung. */
  n: Vec3;
}

export type NormalMode = 'auto' | 'on' | 'off';

export interface JointInput {
  src: Marker[];
  dst: Marker[];
  /** true: Markierung i gehört zu Markierung i. false: Zuordnung automatisch finden. */
  ordered: boolean;
  normals: NormalMode;
}

export interface JointSolution {
  T: Rigid;
  /** perm[i] = Index der dst-Markierung, die zu src[i] gehört. */
  perm: number[];
  /** Mittlere Abweichung der Punkte nach der Ausrichtung (gleiche Einheit wie die Modelle). */
  rmsd: number;
  /** Interne Bewertung (Punkte + Normalen). */
  cost: number;
  /** Verbleibender Freiheitsgrad: Drehachse durch `point`, falls die Lage nicht eindeutig ist. */
  freeAxis?: { point: Vec3; axis: Vec3 };
  /** Passt fast genauso gut wie die beste Lösung (mögliche Verwechslung). */
  nearTie?: boolean;
}

export interface JointResult {
  solutions: JointSolution[];
  /** true, wenn eine andere Zuordnung fast genauso gut passt. */
  ambiguous: boolean;
  error?: string;
}

const MAX_FULL_ENUM = 7;
const MAX_SOLUTIONS = 6;

function normalWeight(mode: NormalMode, n: number): number {
  if (mode === 'off') return 0;
  if (mode === 'on') return 1;
  return n < 3 ? 1 : 0.2;
}

/** Charakteristische Größe der Markierungswolke (für Normalen-Hilfspunkte und Toleranzen). */
function spread(pts: Vec3[]): number {
  if (pts.length < 2) return 1;
  let m = 0;
  for (let i = 0; i < pts.length; i++) for (let j = i + 1; j < pts.length; j++) m = Math.max(m, dist(pts[i], pts[j]));
  return m || 1;
}

function isCollinear(pts: Vec3[], tol: number): boolean {
  if (pts.length < 3) return true;
  // Suche den Punkt mit größtem Abstand zur Linie durch die beiden entferntesten Punkte
  let a = 0;
  let b = 1;
  let best = -1;
  for (let i = 0; i < pts.length; i++)
    for (let j = i + 1; j < pts.length; j++) {
      const d = dist(pts[i], pts[j]);
      if (d > best) { best = d; a = i; b = j; }
    }
  const dir = normalize(sub(pts[b], pts[a]));
  for (const p of pts) {
    const v = sub(p, pts[a]);
    const along = v[0] * dir[0] + v[1] * dir[1] + v[2] * dir[2];
    if (dist(v, scale(dir, along)) > tol) return false;
  }
  return true;
}

function evaluate(src: Marker[], dst: Marker[], perm: number[], wN: number, s: number): JointSolution {
  const P: Vec3[] = [];
  const Q: Vec3[] = [];
  const W: number[] = [];
  for (let i = 0; i < src.length; i++) {
    const d = dst[perm[i]];
    P.push(src[i].p);
    Q.push(d.p);
    W.push(1);
    if (wN > 0) {
      // Normalen sollen entgegengesetzt sein: src-Normale → -dst-Normale
      P.push(add(src[i].p, scale(src[i].n, s)));
      Q.push(sub(d.p, scale(d.n, s)));
      W.push(wN);
    }
  }
  const T = solveRigid(P, Q, W);
  let sp = 0;
  let sAll = 0;
  let wAll = 0;
  for (let k = 0; k < P.length; k++) {
    const e = dist(apply(T, P[k]), Q[k]);
    sAll += W[k] * e * e;
    wAll += W[k];
  }
  for (let i = 0; i < src.length; i++) {
    const e = dist(apply(T, src[i].p), dst[perm[i]].p);
    sp += e * e;
  }
  return { T, perm, rmsd: Math.sqrt(sp / src.length), cost: Math.sqrt(sAll / wAll) };
}

function* permutations(n: number): Generator<number[]> {
  const a = [...Array(n).keys()];
  const c = new Array(n).fill(0);
  yield a.slice();
  let i = 0;
  while (i < n) {
    if (c[i] < i) {
      const k = i % 2 === 0 ? 0 : c[i];
      [a[k], a[i]] = [a[i], a[k]];
      yield a.slice();
      c[i]++;
      i = 0;
    } else {
      c[i] = 0;
      i++;
    }
  }
}

/**
 * Für viele Markierungen: Tiefensuche, die Zuordnungen verwirft, sobald paarweise Abstände
 * nicht mehr zusammenpassen (Abstände sind bei starren Bewegungen invariant).
 */
function* consistentPermutations(src: Vec3[], dst: Vec3[], tol: number): Generator<number[]> {
  const n = src.length;
  const perm: number[] = [];
  const used = new Array(n).fill(false);
  function* rec(i: number): Generator<number[]> {
    if (i === n) { yield perm.slice(); return; }
    for (let j = 0; j < n; j++) {
      if (used[j]) continue;
      let ok = true;
      for (let k = 0; k < i; k++) {
        if (Math.abs(dist(src[i], src[k]) - dist(dst[j], dst[perm[k]])) > tol) { ok = false; break; }
      }
      if (!ok) continue;
      used[j] = true;
      perm.push(j);
      yield* rec(i + 1);
      perm.pop();
      used[j] = false;
    }
  }
  yield* rec(0);
}

export function solveJoint(input: JointInput): JointResult {
  const { src, dst } = input;
  const n = src.length;
  if (n === 0 || dst.length === 0) return { solutions: [], ambiguous: false, error: 'Noch keine Markierungen.' };
  if (n !== dst.length)
    return {
      solutions: [],
      ambiguous: false,
      error: `Ungleiche Anzahl Markierungen (${dst.length} ↔ ${n}).`,
    };

  const wN = normalWeight(input.normals, n);
  const s = Math.max(spread(dst.map((m) => m.p)), spread(src.map((m) => m.p))) * 0.25 || 1;
  const sizeRef = spread(dst.map((m) => m.p));

  let candidates: Iterable<number[]>;
  if (input.ordered) candidates = [[...Array(n).keys()]];
  else if (n <= MAX_FULL_ENUM) candidates = permutations(n);
  else candidates = consistentPermutations(src.map((m) => m.p), dst.map((m) => m.p), sizeRef * 0.08);

  const solutions: JointSolution[] = [];
  for (const perm of candidates) {
    solutions.push(evaluate(src, dst, perm, wN, s));
    if (solutions.length > 50000) break;
  }
  if (solutions.length === 0)
    return { solutions: [], ambiguous: false, error: 'Keine passende Zuordnung gefunden – Markierungen prüfen.' };
  solutions.sort((a, b) => a.cost - b.cost);
  const top = solutions.slice(0, MAX_SOLUTIONS);

  for (const sol of top) {
    const freeAxis = findFreeAxis(src, dst, sol, wN);
    if (freeAxis) sol.freeAxis = freeAxis;
  }

  if (!input.ordered)
    for (const sol of top.slice(1)) sol.nearTie = sol.cost - top[0].cost < 0.03 * (sizeRef || 1);
  const ambiguous = top.some((s) => s.nearTie);
  return { solutions: top, ambiguous };
}

/** Bestimmt, ob nach der Ausrichtung noch eine Drehung offen ist (z. B. bei nur einem Punkt). */
function findFreeAxis(src: Marker[], dst: Marker[], sol: JointSolution, wN: number) {
  const n = src.length;
  const pts = sol.perm.map((j) => dst[j].p);
  const tol = spread(pts) * 1e-3 + 1e-9;
  if (wN === 0) {
    if (n === 2 || (n > 2 && isCollinear(pts, tol))) return { point: pts[0], axis: normalize(sub(pts[1], pts[0])) };
    return undefined;
  }
  if (n === 1) return { point: dst[sol.perm[0]].p, axis: normalize(dst[sol.perm[0]].n) };
  if (isCollinear(pts, tol)) {
    // Kollineare Punkte mit Normalen: Achse ist festgelegt, wenn die Normalen nicht parallel zur Linie sind.
    const axis = normalize(sub(pts[pts.length - 1], pts[0]));
    const nAll = dst.map((m) => Math.abs(m.n[0] * axis[0] + m.n[1] * axis[1] + m.n[2] * axis[2]));
    if (nAll.every((c) => c > 0.999)) return { point: pts[0], axis };
  }
  return undefined;
}

/**
 * Wendet Feineinstellungen an: Drehung um die freie Achse und Einsinken (Überlappung),
 * damit der Slicer die Teile zu einem Körper verschmilzt.
 */
export function applyAdjustments(
  sol: JointSolution,
  dst: Marker[],
  spinDeg: number,
  overlap: number,
): Rigid {
  let T = sol.T;
  if (sol.freeAxis && spinDeg) T = compose(rotationAbout(sol.freeAxis.point, sol.freeAxis.axis, (spinDeg * Math.PI) / 180), T);
  if (overlap) {
    // Mittlere Normale des festen Teils an den Kontaktstellen zeigt zum beweglichen Teil hin.
    let nAvg: Vec3 = [0, 0, 0];
    for (const m of dst) nAvg = add(nAvg, m.n);
    nAvg = normalize(nAvg);
    T = { R: T.R, t: sub(T.t, scale(nAvg, overlap)) };
  }
  return T;
}

