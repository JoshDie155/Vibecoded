/**
 * Ganze Wand einer Bohrung oder eines Zapfens finden. Die Erkennung (detectFeature) liefert nur die
 * zusammenhängende Fläche ums angeklickte Dreieck – bei zerschnittenen Modellen endet sie aber, wo der
 * Schnitt durch die Bohrung geht (der Rest liegt im anderen Teil), und Splitter oder unsaubere Netze
 * zerteilen sie auch innerhalb eines Teils (dann mit leicht falschem Durchmesser).
 *
 * Hier zählt die Geometrie: alle Dreiecke aller Netze, die auf demselben Zylinder liegen (Abstand zur
 * Achse, Normale quer zur Achse, richtige Seite) und entlang der Achse an die angeklickte Wand anschließen.
 * Mittelpunkt und Radius werden aus allen Stücken neu bestimmt.
 */
import type { Vec3 } from './align';
import type { Topology } from './features';

export interface WallSource {
  topo: Topology;
  /** Lage des Netzes (4×4, spaltenweise wie three.js; starr). */
  matrix: ArrayLike<number>;
}

export interface WallSeed {
  /** Netz, auf dem angeklickt wurde. */
  source: number;
  /** Erkannte Dreiecke (die angeklickte Wand). */
  tris: number[];
  /** Zylinder in Koordinaten dieses Netzes. */
  center: Vec3;
  axis: Vec3;
  radius: number;
  hole: boolean;
}

export interface Wall {
  /** Zylinder in Weltkoordinaten (neu bestimmt). */
  center: Vec3;
  axis: Vec3;
  radius: number;
  /** Dreiecke je Netz (leer, wenn es nichts beiträgt). */
  pieces: number[][];
  /** Ausdehnung entlang der Achse, bezogen auf center. */
  span: [number, number];
}

const tx = (m: ArrayLike<number>, p: ArrayLike<number>, o = 0): Vec3 => [
  m[0] * p[o] + m[4] * p[o + 1] + m[8] * p[o + 2] + m[12],
  m[1] * p[o] + m[5] * p[o + 1] + m[9] * p[o + 2] + m[13],
  m[2] * p[o] + m[6] * p[o + 1] + m[10] * p[o + 2] + m[14],
];
const td = (m: ArrayLike<number>, d: Vec3): Vec3 => {
  const v: Vec3 = [m[0] * d[0] + m[4] * d[1] + m[8] * d[2], m[1] * d[0] + m[5] * d[1] + m[9] * d[2], m[2] * d[0] + m[6] * d[1] + m[10] * d[2]];
  const l = Math.hypot(...v) || 1;
  return [v[0] / l, v[1] / l, v[2] / l];
};

/** Kreis durch Punkte in der Ebene (Kasa): Mittelpunkt und Radius, oder null bei zu wenig Streuung. */
function fitCircle2(us: number[], vs: number[]): { cu: number; cv: number; r: number } | null {
  const n = us.length;
  if (n < 3) return null;
  let mu = 0, mv = 0;
  for (let i = 0; i < n; i++) { mu += us[i]; mv += vs[i]; }
  mu /= n; mv /= n;
  let suu = 0, svv = 0, suv = 0, suuu = 0, svvv = 0, suvv = 0, svuu = 0;
  for (let i = 0; i < n; i++) {
    const u = us[i] - mu, v = vs[i] - mv;
    suu += u * u; svv += v * v; suv += u * v;
    suuu += u * u * u; svvv += v * v * v; suvv += u * v * v; svuu += v * u * u;
  }
  const det = suu * svv - suv * suv;
  if (!(Math.abs(det) > 1e-12 * (suu + svv) ** 2)) return null;
  const bu = (suuu + suvv) / 2, bv = (svvv + svuu) / 2;
  const cu = (bu * svv - bv * suv) / det, cv = (bv * suu - bu * suv) / det;
  return { cu: cu + mu, cv: cv + mv, r: Math.sqrt(cu * cu + cv * cv + (suu + svv) / n) };
}

export function cylinderWall(sources: WallSource[], seed: WallSeed): Wall {
  const sm = sources[seed.source].matrix;
  let center = tx(sm, seed.center);
  const axis = td(sm, seed.axis);
  let radius = seed.radius;
  // Basis quer zur Achse
  const ref: Vec3 = Math.abs(axis[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const U = normalize3([axis[1] * ref[2] - axis[2] * ref[1], axis[2] * ref[0] - axis[0] * ref[2], axis[0] * ref[1] - axis[1] * ref[0]]);
  const V: Vec3 = [axis[1] * U[2] - axis[2] * U[1], axis[2] * U[0] - axis[0] * U[2], axis[0] * U[1] - axis[1] * U[0]];

  /** Liegt das Dreieck auf dem Zylinder (Welt), mit Toleranz für den Abstand zur Achse? */
  const onWall = (src: WallSource, t: number, tol: number): boolean => {
    const { topo, matrix } = src;
    const pos = topo.positions;
    {
      if (!(topo.areas[t] > 0)) return false;
      let ok = true;
      let rx = 0, ry = 0, rz = 0;
      const ang: number[] = [];
      for (let k = 0; k < 3 && ok; k++) {
        const p = tx(matrix, pos, 9 * t + 3 * k);
        const qx = p[0] - center[0], qy = p[1] - center[1], qz = p[2] - center[2];
        const h = qx * axis[0] + qy * axis[1] + qz * axis[2];
        const ex = qx - h * axis[0], ey = qy - h * axis[1], ez = qz - h * axis[2];
        if (Math.abs(Math.hypot(ex, ey, ez) - radius) > tol) ok = false;
        rx += ex; ry += ey; rz += ez;
        ang.push(Math.atan2(ex * V[0] + ey * V[1] + ez * V[2], ex * U[0] + ey * U[1] + ez * U[2]));
      }
      if (!ok) return false;
      // Schmale Facette der Wand, kein Dreieck quer durch die Bohrung (Ecken weit auseinander auf dem Kreis)
      let spread = 0;
      for (let i = 0; i < 3; i++) for (let j = i + 1; j < 3; j++) {
        const d = Math.abs(ang[i] - ang[j]);
        spread = Math.max(spread, Math.min(d, 2 * Math.PI - d));
      }
      if (spread > Math.PI / 4) return false;
      const n = td(matrix, [topo.normals[3 * t], topo.normals[3 * t + 1], topo.normals[3 * t + 2]]);
      // Quer zur Achse und auf der richtigen Seite (Bohrung: Normale zur Achse hin)
      if (Math.abs(n[0] * axis[0] + n[1] * axis[1] + n[2] * axis[2]) > 0.3) return false;
      const side = n[0] * rx + n[1] * ry + n[2] * rz;
      return seed.hole ? side < 0 : side > 0;
    }
  };
  /** Alle Dreiecke aller Netze auf dem Zylinder. */
  const collect = (tol: number) => sources.map((src) => {
    const out: number[] = [];
    for (let t = 0; t < src.topo.triCount; t++) if (onWall(src, t, tol)) out.push(t);
    return out;
  });
  /** Vom angeklickten Stück aus über Nachbarn weiter, solange es auf dem Zylinder bleibt. */
  const grow = (tol: number): number[] => {
    const src = sources[seed.source];
    const set = new Set(seed.tris);
    const stack = [...seed.tris];
    while (stack.length && set.size < 200000) {
      const t = stack.pop()!;
      for (let k = 0; k < 3; k++) {
        const nb = src.topo.neighbors[3 * t + k];
        if (nb < 0 || set.has(nb) || !onWall(src, nb, tol)) continue;
        set.add(nb);
        stack.push(nb);
      }
    }
    return [...set];
  };

  /**
   * Mittelpunkt und Radius aus den Eckpunkten der angeklickten Wand neu – mit Ausreißern weg (Splitter
   * neben der Wand): erst alle, dann die besser passende Hälfte, dann nur die fast genau auf dem Kreis.
   * Nur nötig, wenn die Erkennung nicht genau passt; nur bei plausiblem Ergebnis übernommen.
   */
  const refit = (tris: number[]): boolean => {
    const seen = new Set<string>();
    let us: number[] = [];
    let vs: number[] = [];
    {
      const { topo, matrix } = sources[seed.source];
      for (const t of tris) for (let k = 0; k < 3; k++) {
        const p = tx(matrix, topo.positions, 9 * t + 3 * k);
        const q: Vec3 = [p[0] - center[0], p[1] - center[1], p[2] - center[2]];
        const u = q[0] * U[0] + q[1] * U[1] + q[2] * U[2];
        const v = q[0] * V[0] + q[1] * V[1] + q[2] * V[2];
        // Eckpunkte übereinander (lange Dreiecke einer Wand) nur einmal
        const key = `${u.toFixed(4)},${v.toFixed(4)}`;
        if (seen.has(key)) continue;
        seen.add(key);
        us.push(u);
        vs.push(v);
      }
    }
    // Passt der Kreis schon über einen großen Bogen genau, bleibt er (die Erkennung ist dann exakt)
    const off = us.map((u, i) => Math.abs(Math.hypot(u, vs[i]) - radius)).sort((a, b) => a - b);
    const angles = us.map((u, i) => Math.atan2(vs[i], u)).sort((a, b) => a - b);
    let gapMax = angles.length ? 2 * Math.PI - (angles[angles.length - 1] - angles[0]) : 2 * Math.PI;
    for (let i = 1; i < angles.length; i++) gapMax = Math.max(gapMax, angles[i] - angles[i - 1]);
    const arc = 2 * Math.PI - gapMax;
    if (off.length && arc >= Math.PI / 2 && off[Math.floor(off.length * 0.9)] <= Math.max(0.002, 0.0005 * radius)) return true;
    let c = fitCircle2(us, vs);
    for (const keepShare of [0.5, 0]) {
      if (!c) return false;
      const cc = c;
      const res = us.map((u, i) => Math.abs(Math.hypot(u - cc.cu, vs[i] - cc.cv) - cc.r));
      const limit = keepShare > 0 ? [...res].sort((a, b) => a - b)[Math.floor(res.length * keepShare)] : Math.max(0.003, 0.002 * cc.r);
      const idx = res.map((r, i) => (r <= limit ? i : -1)).filter((i) => i >= 0);
      if (idx.length < 6) break;
      us = idx.map((i) => us[i]);
      vs = idx.map((i) => vs[i]);
      c = fitCircle2(us, vs);
    }
    if (!c || Math.abs(c.r - radius) > 0.1 * radius || Math.hypot(c.cu, c.cv) > 0.1 * radius) return false;
    center = [center[0] + c.cu * U[0] + c.cv * V[0], center[1] + c.cu * U[1] + c.cv * V[1], center[2] + c.cu * U[2] + c.cv * V[2]];
    radius = c.r;
    return false;
  };

  // Ein schmaler Bogen passt auch zu einem etwas falschen Kreis: Wand wachsen lassen und neu bestimmen,
  // bis sie nicht mehr größer wird – je größer der Bogen, desto eindeutiger der Kreis
  let grown = seed.tris.length;
  // Passt die Erkennung über einen großen Bogen schon genau, gilt sie
  const exact = refit(seed.tris);
  for (let i = 0; i < 6 && !exact; i++) {
    const tris = grow(Math.min(0.15, Math.max(0.03, 0.01 * radius)));
    refit(tris);
    if (tris.length <= grown && i > 0) break;
    grown = tris.length;
  }
  const pieces = collect(Math.max(0.02, 0.01 * radius));

  // Entlang der Achse: nur, was an die angeklickte Wand anschließt (keine zweite Bohrung auf derselben Achse)
  const spans: { s: number; t: number; lo: number; hi: number }[] = [];
  pieces.forEach((tris, s) => {
    const { topo, matrix } = sources[s];
    for (const t of tris) {
      let lo = Infinity, hi = -Infinity;
      for (let k = 0; k < 3; k++) {
        const p = tx(matrix, topo.positions, 9 * t + 3 * k);
        const h = (p[0] - center[0]) * axis[0] + (p[1] - center[1]) * axis[1] + (p[2] - center[2]) * axis[2];
        lo = Math.min(lo, h);
        hi = Math.max(hi, h);
      }
      spans.push({ s, t, lo, hi });
    }
  });
  const seedSet = new Set(seed.tris);
  const seeded = spans.filter((x) => x.s === seed.source && seedSet.has(x.t));
  const gap = Math.max(0.5, 0.1 * radius);
  let lo = seeded.length ? Math.min(...seeded.map((x) => x.lo)) : Infinity;
  let hi = seeded.length ? Math.max(...seeded.map((x) => x.hi)) : -Infinity;
  const keep: number[][] = sources.map(() => []);
  if (seeded.length) {
    // Anschließende Stücke aufnehmen, bis nichts mehr dazukommt
    const sorted = spans.slice().sort((a, b) => a.lo - b.lo);
    let grown = true;
    const taken = new Set<(typeof spans)[number]>();
    while (grown) {
      grown = false;
      for (const x of sorted) {
        if (taken.has(x) || x.lo > hi + gap || x.hi < lo - gap) continue;
        taken.add(x);
        if (x.lo < lo || x.hi > hi) grown = true;
        lo = Math.min(lo, x.lo);
        hi = Math.max(hi, x.hi);
      }
    }
    for (const x of taken) keep[x.s].push(x.t);
  }
  // Die angeklickten Dreiecke gehören immer dazu (auch Splitter, die knapp außerhalb liegen)
  const own = new Set(keep[seed.source]);
  for (const t of seed.tris) if (!own.has(t)) keep[seed.source].push(t);
  for (const k of keep) k.sort((a, b) => a - b);
  return { center, axis, radius, pieces: keep, span: [lo, hi] };
}

function normalize3(v: Vec3): Vec3 {
  const l = Math.hypot(...v) || 1;
  return [v[0] / l, v[1] / l, v[2] / l];
}
