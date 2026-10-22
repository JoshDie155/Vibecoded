/**
 * Verbindung über Bedingungen ("Mates") wie in CAD-Programmen: Punkte, ebene Flächen und Achsen
 * (Zapfen, Bohrungen, runde Teile) des beweglichen Teils (lokale Koordinaten) werden auf die
 * passenden Merkmale des festen Teils (Weltkoordinaten) gelegt.
 *
 * - Gelöst wird eine nichtlineare Ausgleichsrechnung (Levenberg–Marquardt) über die sechs
 *   Freiheitsgrade der starren Bewegung. Mehrere Startwerte (geschlossene Lösung, gedrehte
 *   Varianten, beide Achsrichtungen) vermeiden lokale Minima und liefern echte Alternativen.
 * - Harte Bedingungen (Flächen liegen aufeinander, Achsen fluchten) haben Vorrang. Weiche Terme mit
 *   kleinem Gewicht zentrieren die Teile nur dort, wo sonst nichts die Lage festlegt.
 * - Danach wird bestimmt, welche Bewegungen noch frei sind (Bewegungen, die alle Merkmale des festen
 *   Teils in sich überführen), damit die Oberfläche nur dafür einen Gizmo anbietet.
 */
import {
  type Mat3, type Rigid, type Vec3, add, apply, axisAngle, compose, dist, dot, len, normalize, quatToMat3, rotate,
  rotationAbout, scale, solveRigid, sub,
} from './align';
import type { FreeMotion } from './motion';

export type Constraint =
  /** Punkt auf Punkt: T(src) = dst. */
  | { kind: 'point'; src: Vec3; dst: Vec3; weight?: number }
  /**
   * Ebene Flächen liegen aufeinander: Normalen (nach außen) entgegengesetzt, T(src.p) auf der
   * dst-Ebene. Weich: T(src.p) möglichst nahe an dst.p (zentriert die Flächen).
   */
  | { kind: 'plane'; src: { p: Vec3; n: Vec3 }; dst: { p: Vec3; n: Vec3 }; weight?: number }
  /**
   * Achsen fluchten (koaxial): R·src.d = +dst.d, bzw. −dst.d bei flip. Ohne flip werden beide
   * Richtungen als getrennte Lösungen geliefert. Weich: T(src.p) entlang der Achse nahe an dst.p.
   */
  | { kind: 'axis'; src: { p: Vec3; d: Vec3 }; dst: { p: Vec3; d: Vec3 }; weight?: number; flip?: boolean };

export interface ConstraintSolution {
  /** Transformation lokal → Welt des beweglichen Teils. */
  T: Rigid;
  /** Mittlere Abweichung (RMS) der harten Abstandsbedingungen in mm. */
  distError: number;
  /** Mittlere Abweichung (RMS) der harten Winkelbedingungen in Grad. */
  angleError: number;
  /** Noch freie Bewegungen in Weltkoordinaten (leer, wenn die Lage eindeutig ist). */
  free: FreeMotion[];
  /** Je Bedingung (Reihenfolge wie in der Eingabe): Achsrichtung umgedreht? Sonst false. */
  flips: boolean[];
  /** Gesamtkosten (harte + weiche Terme, gewichtete Fehlerquadrate), danach wird sortiert. */
  cost: number;
}

export interface ConstraintOptions {
  /** Startlage, z. B. die aktuelle Lage des Teils. Freie Drehungen bleiben davon erhalten. */
  init?: Rigid;
  /** Gewicht der weichen Zentrierterme relativ zu den harten Bedingungen (Standard 1e-3). */
  softWeight?: number;
  /** Charakteristische Länge in mm, macht Winkel und Abstände vergleichbar (Standard: Ausdehnung der Merkmale). */
  scale?: number;
}

const MAX_STARTS = 32;
const MAX_SOLUTIONS = 8;
/** Gruppen von Achsen ohne vorgegebene Richtung, deren beide Richtungen durchprobiert werden (2^n). */
const MAX_FLIP_GROUPS = 4;
/** Bis zu so vielen offenen Achsen werden auch die in sich widersprüchlichen Kombinationen versucht. */
const MAX_FLIP_AXES = 4;
/** Zwei Achsen, die in beiden Teilen um weniger als ~72° von der Senkrechten abweichen, sind gekoppelt. */
const LINK_COS = 0.3;
/**
 * Richtungen, deren Eigenwert unter diesem Anteil des größten liegt, gelten als frei (Eigenwerte in
 * mm² je mm Bewegung, Drehungen am Hebel der Merkmalsausdehnung): z. B. bleibt die Drehung eines
 * Zapfens frei, wenn Schulterfläche und Achse bis zu ~0,2° gegeneinander verkippt erkannt wurden
 * (Messrauschen aus tessellierten oder unsauberen Netzen); ab ~0,5° ist sie gesperrt.
 */
const FREE_TOL = 3e-6;
/** Anteil des Drehteils, ab dem ein Vektor des Nullraums als Drehung (statt Verschiebung) gilt. */
const ROT_TOL = 1e-4;
const SAME_ANGLE = (0.5 * Math.PI) / 180;
const SAME_DIST = 1e-3;
const AXES: Vec3[] = [
  [1, 0, 0],
  [0, 1, 0],
  [0, 0, 1],
];

/** Aufbereitete Bedingung (normierte Richtungen, Gewicht als Faktor auf die Residuen). */
interface Item {
  kind: 'point' | 'plane' | 'axis';
  /** Position in der Eingabeliste. */
  index: number;
  /** Wurzel des Gewichts. */
  w: number;
  ps: Vec3;
  pd: Vec3;
  /** Normale bzw. Achsrichtung (Einheitsvektor, [0, 0, 0] wenn entartet). */
  ds: Vec3;
  dd: Vec3;
  /** Orthonormalbasis senkrecht zu dd. */
  t1: Vec3;
  t2: Vec3;
  /** Beide Richtungen gültig. */
  ok: boolean;
  flip?: boolean;
}

interface Evaluation {
  /** Summe der gewichteten Fehlerquadrate. */
  cost: number;
  /** JᵀJ (6x6 zeilenweise) und Jᵀr, Parameter (P·ω, v) mit Drehung um c. */
  A: number[];
  g: number[];
  c: Vec3;
}

const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const neg = (a: Vec3): Vec3 => [-a[0], -a[1], -a[2]];
const num = (x: unknown) => (typeof x === 'number' && Number.isFinite(x) ? x : 0);
const vec = (v: unknown): Vec3 => (Array.isArray(v) ? [num(v[0]), num(v[1]), num(v[2])] : [0, 0, 0]);
/** Winkel zwischen zwei Richtungen (Radiant), auch für kleine Winkel genau. */
const angleBetween = (a: Vec3, b: Vec3) => Math.atan2(len(cross(a, b)), dot(a, b));

function invert(T: Rigid): Rigid {
  const R = T.R;
  const Rt: Mat3 = [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
  return { R: Rt, t: neg(rotate(Rt, T.t)) };
}

/** Quaternion (w ≥ 0) einer Rotationsmatrix nach Shepperd. */
function matToQuat(R: Mat3): number[] {
  const [m00, m01, m02, m10, m11, m12, m20, m21, m22] = R;
  const tr = m00 + m11 + m22;
  let q: number[];
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    q = [s / 4, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s];
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
    q = [(m21 - m12) / s, s / 4, (m01 + m10) / s, (m02 + m20) / s];
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
    q = [(m02 - m20) / s, (m01 + m10) / s, s / 4, (m12 + m21) / s];
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
    q = [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, s / 4];
  }
  const n = Math.hypot(q[0], q[1], q[2], q[3]) * (q[0] < 0 ? -1 : 1);
  return n ? q.map((v) => v / n) : [1, 0, 0, 0];
}

/** Macht eine (durch Rundung leicht verzerrte) Rotationsmatrix wieder orthonormal. */
function orthonormalize(R: Mat3): Mat3 {
  return quatToMat3(matToQuat(R));
}

/** Rotationsvektor (Achse · Winkel, Winkel in [0, π]). */
function rotationVector(R: Mat3): Vec3 {
  const q = matToQuat(R);
  const s = Math.hypot(q[1], q[2], q[3]);
  if (s < 1e-300) return [0, 0, 0];
  const th = 2 * Math.atan2(s, q[0]);
  return [(q[1] / s) * th, (q[2] / s) * th, (q[3] / s) * th];
}

/** Eigenwerte/-vektoren einer kleinen symmetrischen Matrix (zyklisches Jacobi-Verfahren). */
function symEigen(Ain: number[][]): { values: number[]; vectors: number[][] } {
  const n = Ain.length;
  const A = Ain.map((r) => r.slice());
  const V: number[][] = A.map((_, i) => A.map((__, j) => (i === j ? 1 : 0)));
  let norm = 0;
  for (const r of A) for (const v of r) norm += v * v;
  for (let sweep = 0; sweep < 60; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += A[p][q] * A[p][q];
    if (off <= 1e-30 * norm || off < 1e-300) break;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        if (Math.abs(A[p][q]) < 1e-300) continue;
        const theta = (A[q][q] - A[p][p]) / (2 * A[p][q]);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < n; k++) {
          const akp = A[k][p];
          const akq = A[k][q];
          A[k][p] = c * akp - s * akq;
          A[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < n; k++) {
          const apk = A[p][k];
          const aqk = A[q][k];
          A[p][k] = c * apk - s * aqk;
          A[q][k] = s * apk + c * aqk;
        }
        for (let k = 0; k < n; k++) {
          const vkp = V[k][p];
          const vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq;
          V[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  return {
    values: A.map((r, i) => r[i]),
    // Spalten von V sind die Eigenvektoren
    vectors: A.map((_, j) => V.map((r) => r[j])),
  };
}

/** Löst M·x = b (n×n zeilenweise) mit Spaltenpivotsuche; null bei singulärer Matrix. */
function solveLinear(M: number[], b: number[], n: number): number[] | null {
  const a = M.slice();
  const x = b.slice();
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(a[r * n + col]) > Math.abs(a[piv * n + col])) piv = r;
    const pv = a[piv * n + col];
    if (!(Math.abs(pv) > 1e-300)) return null;
    if (piv !== col) {
      for (let k = 0; k < n; k++) [a[piv * n + k], a[col * n + k]] = [a[col * n + k], a[piv * n + k]];
      [x[piv], x[col]] = [x[col], x[piv]];
    }
    for (let r = col + 1; r < n; r++) {
      const f = a[r * n + col] / pv;
      if (f === 0) continue;
      for (let k = col; k < n; k++) a[r * n + k] -= f * a[col * n + k];
      x[r] -= f * x[col];
    }
  }
  for (let r = n - 1; r >= 0; r--) {
    let s = x[r];
    for (let k = r + 1; k < n; k++) s -= a[r * n + k] * x[k];
    x[r] = s / a[r * n + r];
  }
  return x;
}

/** Orthonormalisiert eine Liste von Richtungen und verwirft (nahezu) abhängige. */
function gramSchmidt(vs: Vec3[]): Vec3[] {
  const out: Vec3[] = [];
  for (const v of vs) {
    let r = v;
    for (const o of out) r = sub(r, scale(o, dot(r, o)));
    if (len(r) > 1e-6) out.push(normalize(r));
  }
  return out;
}

/**
 * Anschauliche Basis eines Unterraums: zuerst Richtungen der Bedingungen (Normalen, Achsen), die im
 * Unterraum liegen, dann möglichst achsparallele Richtungen.
 */
function niceBasis(span: Vec3[], prefs: Vec3[]): Vec3[] {
  const S = gramSchmidt(span);
  const out: Vec3[] = [];
  const residual = (v: Vec3) => {
    let p: Vec3 = [0, 0, 0];
    for (const s of S) p = add(p, scale(s, dot(v, s)));
    for (const o of out) p = sub(p, scale(o, dot(p, o)));
    return p;
  };
  for (const p of prefs) {
    if (out.length >= S.length) break;
    const r = residual(p);
    if (len(r) > 0.9) out.push(normalize(r));
  }
  while (out.length < S.length) {
    let best: Vec3 | null = null;
    let bestLen = 1e-6;
    for (const e of AXES) {
      const r = residual(e);
      if (len(r) > bestLen) {
        best = r;
        bestLen = len(r);
      }
    }
    if (!best) break;
    out.push(normalize(best));
  }
  return out;
}

/** Zwei Einheitsvektoren senkrecht zu n und zueinander. */
function tangents(n: Vec3): [Vec3, Vec3] {
  const t1 = normalize(cross(n, Math.abs(n[0]) < 0.6 ? [1, 0, 0] : [0, 1, 0]));
  return [t1, cross(n, t1)];
}

function prepare(cons: Constraint[]): Item[] {
  const items: Item[] = [];
  cons.forEach((c, index) => {
    if (!c || typeof c !== 'object') return;
    const weight = typeof c.weight === 'number' && Number.isFinite(c.weight) ? Math.max(0, c.weight) : 1;
    const w = Math.sqrt(weight);
    if (c.kind === 'point') {
      const [t1, t2] = tangents([0, 0, 1]);
      items.push({ kind: 'point', index, w, ps: vec(c.src), pd: vec(c.dst), ds: [0, 0, 0], dd: [0, 0, 0], t1, t2, ok: true });
    } else if (c.kind === 'plane' || c.kind === 'axis') {
      const ds = normalize(vec(c.kind === 'plane' ? c.src?.n : c.src?.d));
      const dd = normalize(vec(c.kind === 'plane' ? c.dst?.n : c.dst?.d));
      const [t1, t2] = tangents(dd);
      items.push({
        kind: c.kind,
        index,
        w,
        ps: vec(c.src?.p),
        pd: vec(c.dst?.p),
        ds,
        dd,
        t1,
        t2,
        ok: len(ds) > 0.5 && len(dd) > 0.5,
        flip: c.kind === 'axis' && typeof c.flip === 'boolean' ? c.flip : undefined,
      });
    }
  });
  return items;
}

/** Charakteristische Länge: größte Ausdehnung der Merkmalspunkte, mindestens 1 mm. */
function charLength(items: Item[]): number {
  const extent = (pts: Vec3[]) => {
    const lo: Vec3 = [Infinity, Infinity, Infinity];
    const hi: Vec3 = [-Infinity, -Infinity, -Infinity];
    for (const p of pts)
      for (let k = 0; k < 3; k++) {
        lo[k] = Math.min(lo[k], p[k]);
        hi[k] = Math.max(hi[k], p[k]);
      }
    return pts.length ? dist(lo, hi) : 0;
  };
  return Math.max(1, extent(items.map((i) => i.ps)), extent(items.map((i) => i.pd)));
}

/** Zielrichtung der gedrehten src-Richtung: Ebenen gegeneinander, Achsen gleich oder umgedreht. */
const targetDir = (it: Item, flip: boolean) => (it.kind === 'plane' || flip ? neg(it.dd) : it.dd);

/**
 * Residuen und Normalgleichungen. Winkelfehler zählen mit dem Hebel L. Parameter x = (P·ω, v):
 * kleine Drehung ω um c, danach Verschiebung v. Ein Weltpunkt y bewegt sich dabei um
 * ω × (y − c) + v, eine Richtung u um ω × u. sw = Faktor der weichen Terme (0: nur harte Bedingungen).
 */
function evaluate(items: Item[], fl: boolean[], T: Rigid, L: number, sw: number, P = L): Evaluation {
  const A = new Array(36).fill(0);
  const g = new Array(6).fill(0);
  let cost = 0;
  const addRow = (j: number[], r: number) => {
    for (let a = 0; a < 6; a++) {
      g[a] += j[a] * r;
      for (let b = 0; b < 6; b++) A[a * 6 + b] += j[a] * j[b];
    }
    cost += r * r;
  };
  let c: Vec3 = [0, 0, 0];
  const ys = items.map((it) => apply(T, it.ps));
  for (const y of ys) c = add(c, y);
  c = scale(c, 1 / Math.max(1, ys.length));

  // Komponente m·(y − Ziel) eines Weltpunkts (Faktor f)
  const posRow = (yc: Vec3, m: Vec3, r: number, f: number) => {
    const k = cross(yc, m);
    addRow([(f * k[0]) / P, (f * k[1]) / P, (f * k[2]) / P, f * m[0], f * m[1], f * m[2]], f * r);
  };
  // Komponente L·m·(u − Ziel) einer gedrehten Richtung u
  const dirRow = (u: Vec3, m: Vec3, r: number, f: number) => {
    const k = scale(cross(u, m), (f * L) / P);
    addRow([k[0], k[1], k[2], 0, 0, 0], f * r);
  };

  items.forEach((it, i) => {
    if (it.w === 0) return;
    const e = sub(ys[i], it.pd);
    const yc = sub(ys[i], c);
    if (it.kind === 'point' || !it.ok) {
      // Entartete Achse wirkt wie ein Punkt, entartete Ebene zieht nur weich zum Punkt
      const f = it.kind === 'plane' ? it.w * sw : it.w;
      if (f > 0) for (const m of AXES) posRow(yc, m, dot(m, e), f);
      return;
    }
    const u = rotate(T.R, it.ds);
    const a = sub(u, targetDir(it, fl[i]));
    for (const m of AXES) dirRow(u, m, L * dot(m, a), it.w);
    if (it.kind === 'plane') {
      posRow(yc, it.dd, dot(it.dd, e), it.w);
      if (sw > 0) {
        posRow(yc, it.t1, dot(it.t1, e), it.w * sw);
        posRow(yc, it.t2, dot(it.t2, e), it.w * sw);
      }
    } else {
      posRow(yc, it.t1, dot(it.t1, e), it.w);
      posRow(yc, it.t2, dot(it.t2, e), it.w);
      if (sw > 0) posRow(yc, it.dd, dot(it.dd, e), it.w * sw);
    }
  });
  return { cost, A, g, c };
}

/** Wendet den Parameterschritt x = (P·ω, v) mit Drehzentrum c auf T an. */
function step(T: Rigid, x: number[], c: Vec3, P: number): Rigid {
  const w: Vec3 = [x[0] / P, x[1] / P, x[2] / P];
  const th = len(w);
  const R: Mat3 = th > 1e-14 ? axisAngle(w, th) : [1, 0, 0, 0, 1, 0, 0, 0, 1];
  const Tn = compose({ R, t: add(sub(c, rotate(R, c)), [x[3], x[4], x[5]]) }, T);
  return { R: orthonormalize(Tn.R), t: Tn.t };
}

/** Levenberg–Marquardt ab T0. */
function refine(items: Item[], fl: boolean[], T0: Rigid, L: number, sw: number, maxIter: number, lambda0: number): Rigid {
  let T = T0;
  let ev = evaluate(items, fl, T, L, sw);
  let lambda = lambda0;
  for (let it = 0; it < maxIter; it++) {
    if (!(ev.cost > 1e-30 * L * L)) break;
    let maxDiag = 0;
    for (let i = 0; i < 6; i++) maxDiag = Math.max(maxDiag, ev.A[i * 7]);
    if (!(maxDiag > 0)) break;
    const M = ev.A.slice();
    for (let i = 0; i < 6; i++) M[i * 7] += lambda * maxDiag;
    const x = solveLinear(M, ev.g.map((v) => -v), 6);
    if (!x || !(Math.hypot(...x) >= 1e-12 * L)) break;
    const Tn = step(T, x, ev.c, L);
    const en = evaluate(items, fl, Tn, L, sw);
    if (en.cost < ev.cost) {
      T = Tn;
      ev = en;
      // Untergrenze: Rundungsrauschen im Nullraum soll das Teil nicht entlang freier Bewegungen verschieben
      lambda = Math.max(lambda * 0.1, 1e-10);
    } else {
      lambda *= 10;
      if (lambda > 1e10) break;
    }
  }
  return T;
}

/**
 * Nachpolieren nur mit den harten Bedingungen (Gauß–Newton): entfernt die kleine Verfälschung durch
 * die weichen Terme. Geschritten wird nur in steifen Richtungen; (fast) freie Richtungen bleiben, wo
 * die weichen Terme sie hingelegt haben. Sonst schöbe Messrauschen das Teil dorthin, wo leicht schiefe
 * Achsen zufällig am besten passen – bei fast parallelen Achsen u. U. Meter weit weg.
 * Steif heißt wie in analyse(): Winkel zählen dafür mit dem Hebel P (Merkmalsausdehnung) statt L,
 * sonst erschienen bei großem L neben den Winkeln selbst die Abstände als „weich".
 */
function polish(items: Item[], fl: boolean[], T0: Rigid, L: number, P: number, maxIter: number): Rigid {
  let T = T0;
  let ev = evaluate(items, fl, T, L, 0, P);
  for (let it = 0; it < maxIter && ev.cost > 1e-30 * L * L; it++) {
    const stiff = evaluate(items, fl, T, P, 0, P).A;
    const { values, vectors } = symEigen([0, 1, 2, 3, 4, 5].map((i) => stiff.slice(i * 6, i * 6 + 6)));
    const maxEig = Math.max(0, ...values);
    if (!(maxEig > 0)) break;
    // Gauß–Newton-Schritt im Unterraum der steifen Eigenrichtungen S: (SᵀAS)·y = −Sᵀg, x = S·y
    const S = vectors.filter((_, k) => values[k] > FREE_TOL * maxEig);
    const k = S.length;
    const M = new Array(k * k).fill(0);
    const b = S.map((s) => -s.reduce((acc, v, r) => acc + v * ev.g[r], 0));
    for (let a = 0; a < k; a++)
      for (let c = 0; c < k; c++)
        for (let r = 0; r < 6; r++) for (let q = 0; q < 6; q++) M[a * k + c] += S[a][r] * ev.A[r * 6 + q] * S[c][q];
    const y = k ? solveLinear(M, b, k) : null;
    if (!y) break;
    const x = [0, 0, 0, 0, 0, 0];
    S.forEach((s, a) => {
      for (let r = 0; r < 6; r++) x[r] += y[a] * s[r];
    });
    if (!(Math.hypot(...x) >= 1e-12 * P)) break;
    // Schrittweite halbieren, bis die Kosten sinken
    let next: { T: Rigid; ev: Evaluation } | null = null;
    for (let h = 0, f = 1; h < 8 && !next; h++, f /= 2) {
      const Tn = step(T, x.map((v) => v * f), ev.c, P);
      const en = evaluate(items, fl, Tn, L, 0, P);
      if (en.cost < ev.cost) next = { T: Tn, ev: en };
    }
    if (!next) break;
    const gain = ev.cost - next.ev.cost;
    T = next.T;
    ev = next.ev;
    if (gain <= 1e-14 * ev.cost) break;
  }
  return T;
}

/**
 * Geschlossene Näherung nach Horn über Hilfspunkte: Punkte direkt, Ebenen über Mittelpunkt und
 * Mittelpunkt + L·n (gegen −n), Achsen über Achspunkt und Achspunkt + L·d (Vorzeichen nach flip).
 * Achsen mit skip[i] tragen nur ihren Punkt bei (Richtung noch unbekannt).
 */
function closedForm(items: Item[], fl: boolean[], L: number, skip?: boolean[]): Rigid {
  const P: Vec3[] = [];
  const Q: Vec3[] = [];
  let W: number[] = [];
  items.forEach((it, i) => {
    const w2 = it.w * it.w;
    P.push(it.ps);
    Q.push(it.pd);
    W.push(it.kind === 'plane' && !it.ok ? w2 * 1e-3 : w2);
    if (it.kind !== 'point' && it.ok && !skip?.[i]) {
      P.push(add(it.ps, scale(it.ds, L)));
      Q.push(add(it.pd, scale(targetDir(it, fl[i]), L)));
      W.push(w2);
    }
  });
  if (!(W.reduce((s, v) => s + v, 0) > 1e-300)) W = W.map(() => 1);
  return solveRigid(P, Q, W);
}

/** Achse einer Richtungsgruppe; rel = Richtung umgekehrt zur ersten Achse der Gruppe. */
type GroupMember = { i: number; rel: boolean };

interface FlipPlan {
  /** Richtungskombinationen; die ersten `consistent` sind in sich stimmig. */
  combos: boolean[][];
  consistent: number;
  /** Gruppen jenseits der Obergrenze: Richtung wird je Start aus der Näherungslage bestimmt. */
  late: GroupMember[][];
}

/**
 * Richtungskombinationen der Achsen ohne Vorgabe. Exakt gilt R·d_s,i = s_i·d_d,i; da Drehungen Winkel
 * erhalten, folgt s_i·s_j = sign((d_s,i·d_s,j)·(d_d,i·d_d,j)), sobald zwei Achsen nicht (fast)
 * senkrecht stehen. Solche Achsen bilden eine Gruppe mit nur einer freien Richtung (z. B. alle Stifte
 * eines Flanschs, auch wenn die Flächenerkennung ihre Richtungen beliebig gewählt hat). Zuerst kommen
 * alle Kombinationen der Gruppen, bei wenigen Achsen danach die übrigen (in sich widersprüchlichen)
 * als schlechtere Alternativen.
 */
function flipPlan(items: Item[]): FlipPlan {
  const base = items.map((it) => it.kind === 'axis' && it.flip === true);
  const open = items
    .map((it, i) => (it.kind === 'axis' && it.ok && it.w > 0 && it.flip === undefined ? i : -1))
    .filter((i) => i >= 0);
  const seen = new Set<number>();
  const groups: GroupMember[][] = [];
  for (const i0 of open) {
    if (seen.has(i0)) continue;
    seen.add(i0);
    const g: GroupMember[] = [{ i: i0, rel: false }];
    for (let k = 0; k < g.length; k++) {
      const a = items[g[k].i];
      for (const j of open) {
        if (seen.has(j)) continue;
        const cs = dot(a.ds, items[j].ds);
        const cd = dot(a.dd, items[j].dd);
        if (Math.min(Math.abs(cs), Math.abs(cd)) < LINK_COS) continue;
        seen.add(j);
        g.push({ i: j, rel: g[k].rel !== (cs * cd < 0) });
      }
    }
    groups.push(g);
  }
  const enumerated = groups.slice(0, MAX_FLIP_GROUPS);
  const combos: boolean[][] = [];
  for (let mask = 0; mask < 1 << enumerated.length; mask++) {
    const fl = base.slice();
    enumerated.forEach((g, k) => {
      for (const m of g) fl[m.i] = ((mask >> k) & 1) === 1 ? !m.rel : m.rel;
    });
    combos.push(fl);
  }
  const consistent = combos.length;
  if (open.length <= MAX_FLIP_AXES) {
    const known = new Set(combos.map((fl) => fl.map(Number).join('')));
    for (let mask = 0; mask < 1 << open.length; mask++) {
      const fl = base.slice();
      open.forEach((i, k) => {
        fl[i] = ((mask >> k) & 1) === 1;
      });
      if (!known.has(fl.map(Number).join(''))) combos.push(fl);
    }
  }
  return { combos, consistent, late: groups.slice(MAX_FLIP_GROUPS) };
}

/**
 * Geschlossene Lösung einer Kombination. Die Richtung jeder Gruppe jenseits der Obergrenze folgt aus
 * einer Näherung ohne deren Richtungen (Mehrheit der Achsen der Gruppe).
 */
function baseStart(items: Item[], fl0: boolean[], late: GroupMember[][], L: number): { T: Rigid; fl: boolean[] } {
  if (late.length === 0) return { T: closedForm(items, fl0, L), fl: fl0 };
  const skip = items.map(() => false);
  for (const g of late) for (const m of g) skip[m.i] = true;
  const R = closedForm(items, fl0, L, skip).R;
  const fl = fl0.slice();
  for (const g of late) {
    let vote = 0;
    for (const m of g) vote += (m.rel ? -1 : 1) * dot(rotate(R, items[m.i].ds), items[m.i].dd);
    for (const m of g) fl[m.i] = vote < 0 ? !m.rel : m.rel;
  }
  return { T: closedForm(items, fl, L), fl };
}

/**
 * Startwerte: Startlage und geschlossene Lösung je Richtungskombination (stimmige zuerst), danach um
 * Normalen und Achsen gedrehte Varianten (falls die Drehung darum unbestimmt oder die Näherung schlecht ist).
 */
function buildStarts(items: Item[], plan: FlipPlan, L: number, init?: Rigid): { T: Rigid; fl: boolean[] }[] {
  const starts: { T: Rigid; fl: boolean[] }[] = [];
  const push = (s: { T: Rigid; fl: boolean[] }) => {
    if (starts.length < MAX_STARTS) starts.push(s);
  };
  const bases = plan.combos.map((fl) => baseStart(items, fl, plan.late, L));
  const primary = bases.slice(0, plan.consistent);
  const secondary = bases.slice(plan.consistent);
  for (const b of primary) {
    if (init) push({ T: init, fl: b.fl });
    push(b);
  }
  for (const b of secondary) push(b);
  const dirs = items.filter((it) => it.kind !== 'point' && it.ok && it.w > 0);
  for (const group of [primary, secondary])
    for (const deg of [180, 90, 270])
      for (const it of dirs)
        for (const b of group) push({ T: compose(rotationAbout(it.pd, it.dd, (deg * Math.PI) / 180), b.T), fl: b.fl });
  return starts;
}

/** Restfehler der harten Bedingungen (RMS, mm bzw. Grad). */
function errors(items: Item[], fl: boolean[], T: Rigid): { distError: number; angleError: number } {
  let sd = 0;
  let nd = 0;
  let sa = 0;
  let na = 0;
  items.forEach((it, i) => {
    if (it.w === 0) return;
    const e = sub(apply(T, it.ps), it.pd);
    if (it.kind === 'point' || !it.ok) {
      if (it.kind === 'plane') return;
      sd += dot(e, e);
      nd++;
      return;
    }
    sa += angleBetween(rotate(T.R, it.ds), targetDir(it, fl[i])) ** 2;
    na++;
    const d2 = it.kind === 'plane' ? dot(it.dd, e) ** 2 : dot(it.t1, e) ** 2 + dot(it.t2, e) ** 2;
    sd += d2;
    nd++;
  });
  return {
    distError: nd ? Math.sqrt(sd / nd) : 0,
    angleError: na ? (Math.sqrt(sa / na) * 180) / Math.PI : 0,
  };
}

const preferredDirs = (items: Item[]) => items.filter((it) => it.kind !== 'point' && it.ok && it.w > 0).map((it) => it.dd);

/**
 * Zerlegt den Nullraum in anschauliche Bewegungen: reine Verschiebungen und Drehungen um Achsen
 * durch passende Punkte (der Punkt der Achse, der dem Schwerpunkt c der Merkmale am nächsten liegt).
 * Punkt-, Ebenen- und Achsbedingungen lassen nie eine echte Schraubung frei; ein Schubanteil entlang
 * einer Drehachse, der keine eigene freie Verschiebung ist, ist Rauschen und wird verworfen.
 */
function niceMotions(basis: number[][], c: Vec3, P: number, prefs: Vec3[]): FreeMotion[] {
  const k = basis.length;
  if (k === 0) return [];
  // Drehanteile trennen: Singulärwertzerlegung des Drehteils der Basis über die Gram-Matrix
  const G = basis.map((a) => basis.map((b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2]));
  const { values, vectors } = symEigen(G);
  const trans: Vec3[] = [];
  const rots: { a: Vec3; v: Vec3 }[] = [];
  vectors.forEach((z, j) => {
    const x = [0, 0, 0, 0, 0, 0];
    for (let i = 0; i < k; i++) for (let r = 0; r < 6; r++) x[r] += z[i] * basis[i][r];
    const v: Vec3 = [x[3], x[4], x[5]];
    if (values[j] < ROT_TOL) trans.push(v);
    else rots.push({ a: [x[0] / P, x[1] / P, x[2] / P], v });
  });
  const tr = gramSchmidt(trans);

  // Drehungen je Einheitswinkel, ohne Anteile der freien Verschiebungen
  const rotAxes: Vec3[] = [];
  const rotV: Vec3[] = [];
  for (const r of rots) {
    const wl = len(r.a);
    let v = scale(r.v, 1 / wl);
    for (const t of tr) v = sub(v, scale(t, dot(v, t)));
    rotAxes.push(scale(r.a, 1 / wl));
    rotV.push(v);
  }
  const free: FreeMotion[] = [];
  for (const n of niceBasis(rotAxes, prefs)) {
    let v: Vec3 = [0, 0, 0];
    rotAxes.forEach((a, j) => {
      v = add(v, scale(rotV[j], dot(n, a)));
    });
    // Geschwindigkeit in c ist v = −n × (q − c)  ⇒  q = c + n × v (der Anteil von v entlang n fällt weg)
    let q = add(c, cross(n, v));
    // Freie Verschiebungen quer zur Achse erlauben, die Achse durch den Punkt nahe c zu legen
    for (const t of gramSchmidt(tr.map((t) => sub(t, scale(n, dot(t, n)))))) q = add(q, scale(t, dot(sub(c, q), t)));
    free.push({ kind: 'rotate', point: q, axis: n });
  }
  for (const t of niceBasis(tr, prefs)) free.push({ kind: 'translate', axis: t });
  return free;
}

/**
 * Freie Bewegungen an der Lage T. Jede Bedingung lässt genau die Bewegungen zu, die ihr Merkmal des
 * festen Teils in sich überführen: Drehungen um den Zielpunkt; Verschieben in der Ebene und Drehen um
 * ihre Normale; Drehen um die Achslinie und Verschieben entlang. Frei ist der Durchschnitt dieser
 * Räume (bis auf FREE_TOL, damit Messrauschen nichts sperrt); die Kosten ändern sich entlang dieser
 * Bewegungen nicht, auch wenn die Bedingungen nicht ganz erfüllt sind. (Der Nullraum der
 * Gauß–Newton-Matrix an der Lösung taugt dafür nicht: bei leicht widersprüchlichen Merkmalen dreht
 * sich der Restfehler mit, eine freie Drehung erschiene gesperrt.) Gewichte spielen keine Rolle, jede
 * Bedingung mit Gewicht > 0 zählt. P = Merkmalsausdehnung.
 */
function analyse(items: Item[], T: Rigid, P: number): FreeMotion[] {
  const A = [0, 1, 2, 3, 4, 5].map(() => [0, 0, 0, 0, 0, 0]);
  const addRow = (j: number[]) => {
    for (let a = 0; a < 6; a++) for (let b = 0; b < 6; b++) A[a][b] += j[a] * j[b];
  };
  const ys = items.map((it) => apply(T, it.ps));
  let c: Vec3 = [0, 0, 0];
  for (const y of ys) c = add(c, y);
  c = scale(c, 1 / Math.max(1, ys.length));
  // Punkt q (Welt) bewegt sich in Richtung m nicht: m·(v + ω × (q − c)) = 0
  const posRow = (q: Vec3, m: Vec3) => {
    const k = scale(cross(sub(q, c), m), 1 / P);
    addRow([k[0], k[1], k[2], m[0], m[1], m[2]]);
  };
  items.forEach((it, i) => {
    if (it.w === 0) return;
    if (it.kind === 'point' || (it.kind === 'axis' && !it.ok)) {
      for (const m of AXES) posRow(it.pd, m);
      return;
    }
    // Ebene ohne Normale zieht nur weich
    if (!it.ok) return;
    // Richtung bleibt erhalten: ω × dd = 0
    for (const m of AXES) addRow([...cross(it.dd, m), 0, 0, 0]);
    // Fußpunkt der Fläche bzw. Achse beim beweglichen Teil
    const h = dot(it.dd, sub(ys[i], it.pd));
    if (it.kind === 'plane') posRow(sub(ys[i], scale(it.dd, h)), it.dd);
    else {
      const q = add(it.pd, scale(it.dd, h));
      posRow(q, it.t1);
      posRow(q, it.t2);
    }
  });
  const { values, vectors } = symEigen(A);
  const maxEig = Math.max(0, ...values);
  const basis = vectors.filter((_, i) => values[i] <= FREE_TOL * maxEig);
  return niceMotions(basis, c, P, preferredDirs(items));
}

/** Gleiche Lage innerhalb der Toleranz (Drehung < 0,5°, Merkmalsschwerpunkt < 1e-3·L). */
function sameTransform(A: Rigid, B: Rigid, cs: Vec3, L: number): boolean {
  const D = compose(B, invert(A));
  return len(rotationVector(D.R)) < SAME_ANGLE && dist(apply(A, cs), apply(B, cs)) < SAME_DIST * L;
}

function validRigid(T: Rigid | undefined): T is Rigid {
  return !!T && Array.isArray(T.R) && T.R.length === 9 && Array.isArray(T.t) && T.t.length === 3 &&
    [...T.R, ...T.t].every((v) => Number.isFinite(v));
}

const finiteRigid = (T: Rigid) => [...T.R, ...T.t].every((v) => Number.isFinite(v));

function scaleOf(items: Item[], s: number | undefined): number {
  return typeof s === 'number' && Number.isFinite(s) && s > 0 ? s : charLength(items);
}

/**
 * Sucht die starre Transformation T (lokal → Welt des beweglichen Teils), die alle Bedingungen
 * erfüllt. Liefert verschiedene Lösungen, beste zuerst (z. B. beide Richtungen einer Achse), höchstens
 * eine je Richtungskombination: bei festen Richtungen bilden die Lagen, die alle Bedingungen erfüllen,
 * genau eine Bahn der freien Bewegungen. Weitere Minima derselben Kombination sind schlechtere lokale
 * Minima, Kopien, die nur um eine freie Achse gedreht sind, oder Scheinlösungen weit weg (fast
 * parallele Achsen).
 * Wirft nie: bei widersprüchlichen oder entarteten Eingaben gibt es die bestmögliche Näherung.
 */
export function solveConstraints(cons: Constraint[], opts: ConstraintOptions = {}): ConstraintSolution[] {
  const items = prepare(Array.isArray(cons) ? cons : []);
  if (items.length === 0) return [];
  const L = scaleOf(items, opts.scale);
  const Lf = charLength(items);
  const softWeight = typeof opts.softWeight === 'number' && Number.isFinite(opts.softWeight)
    ? Math.max(0, opts.softWeight)
    : 1e-3;
  const sw = Math.sqrt(softWeight);
  const init = validRigid(opts.init) ? { R: orthonormalize(opts.init.R), t: [...opts.init.t] as Vec3 } : undefined;

  const starts = buildStarts(items, flipPlan(items), L, init);
  const cands: { T: Rigid; fl: boolean[]; cost: number; order: number }[] = [];
  starts.forEach((s, order) => {
    let T = refine(items, s.fl, s.T, L, sw, 100, 1e-3);
    // Verfälschung durch die weichen Terme in den steifen Richtungen entfernen
    if (sw > 0) T = polish(items, s.fl, T, L, Lf, 40);
    if (!finiteRigid(T)) return;
    const cost = evaluate(items, s.fl, T, L, sw).cost;
    if (Number.isFinite(cost)) cands.push({ T, fl: s.fl, cost, order });
  });

  // Sortieren; nahezu gleiche Kosten nach Startreihenfolge (Startlage und stimmige Richtungen zuerst)
  const floor = 1e-12 * L * L;
  cands.sort((a, b) => a.cost - b.cost);
  const ranked: typeof cands = [];
  for (let i = 0; i < cands.length;) {
    const lim = cands[i].cost * (1 + 1e-6) + floor;
    let j = i + 1;
    while (j < cands.length && cands[j].cost <= lim) j++;
    ranked.push(...cands.slice(i, j).sort((a, b) => a.order - b.order));
    i = j;
  }

  let cs: Vec3 = [0, 0, 0];
  for (const it of items) cs = add(cs, it.ps);
  cs = scale(cs, 1 / items.length);
  const accepted: typeof cands = [];
  for (const c of ranked) {
    if (accepted.length >= MAX_SOLUTIONS) break;
    const dup = accepted.some((a) => a.fl.every((f, i) => f === c.fl[i]) || sameTransform(a.T, c.T, cs, L));
    if (!dup) accepted.push(c);
  }

  return accepted.map((a) => {
    const flips = (Array.isArray(cons) ? cons : []).map(() => false);
    items.forEach((it, i) => {
      flips[it.index] = a.fl[i];
    });
    return { T: a.T, ...errors(items, a.fl, a.T), free: analyse(items, a.T, Lf), flips, cost: a.cost };
  });
}

/**
 * Bewertet eine gegebene Lage (z. B. nachdem das Teil mit dem Gizmo entlang einer freien Bewegung
 * verschoben wurde): Restfehler und an dieser Lage freie Bewegungen. Achsen ohne Angabe in flips
 * bzw. ohne flip in der Bedingung nehmen die Richtung, in die sie gerade zeigen.
 */
export function evaluateConstraints(
  cons: Constraint[],
  T: Rigid,
  opts: { flips?: boolean[] } = {},
): { distError: number; angleError: number; free: FreeMotion[] } {
  const items = prepare(Array.isArray(cons) ? cons : []);
  if (items.length === 0 || !validRigid(T)) return { distError: 0, angleError: 0, free: [] };
  const Tn = { R: orthonormalize(T.R), t: [...T.t] as Vec3 };
  const fl = items.map((it) => {
    if (it.kind !== 'axis') return false;
    const f = opts.flips?.[it.index] ?? it.flip;
    return typeof f === 'boolean' ? f : dot(rotate(Tn.R, it.ds), it.dd) < 0;
  });
  return { ...errors(items, fl, Tn), free: analyse(items, Tn, charLength(items)) };
}
