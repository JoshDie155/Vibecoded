/**
 * Randfälle und praxisnahe Szenarien für den Bedingungs-Löser (Gegenprüfung): Größenordnungen,
 * große Drehungen, fast entartete und leicht widersprüchliche Eingaben (Messrauschen), viele
 * Achsen ohne Richtungsangabe sowie Bedingungen, die aus (fein tessellierten) Netzen bestimmt werden.
 */
import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import {
  type Rigid, type Vec3, add, apply, axisAngle, compose, dist, dot, jacobiEigen4, len, normalize, rotate,
  rotationAbout, scale, sub,
} from '../src/math/align';
import {
  type Constraint, type ConstraintOptions, type ConstraintSolution, evaluateConstraints, solveConstraints,
} from '../src/math/constraints';
import { type FreeMotion, applyMotion } from '../src/math/motion';
import { box, concat, cylinder, dPeg, dRing, transformPositions } from './helpers/meshes';

const deg = (d: number) => (d * Math.PI) / 180;
const neg = (a: Vec3): Vec3 => [-a[0], -a[1], -a[2]];
const cross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

/** Reproduzierbarer Zufall (mulberry32). */
function rng(seed: number) {
  let s = seed >>> 0;
  return () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Zufällige Lage; Drehwinkel bis maxAngle (Standard: bis 180°). */
function randomRigid(r: () => number, tScale = 200, maxAngle = Math.PI): Rigid {
  const ax: Vec3 = [r() - 0.5, r() - 0.5, r() - 0.5];
  return {
    R: axisAngle(len(ax) > 1e-3 ? ax : [0, 0, 1], maxAngle * r()),
    t: [(r() - 0.5) * tScale, (r() - 0.5) * tScale, (r() - 0.5) * tScale],
  };
}

const T0: Rigid = { R: axisAngle([0.3, -1, 0.5], 2.0), t: [50, 20, -10] };

/** Punkt auf Punkt. */
const point = (T: Rigid, p: Vec3): Constraint => ({ kind: 'point', src: p, dst: apply(T, p) });
/** Fläche des beweglichen Teils (Normale n) auf Gegenfläche; dst.p um `off` in der Ebene versetzt. */
const plane = (T: Rigid, p: Vec3, n: Vec3, off: Vec3 = [0, 0, 0]): Constraint => ({
  kind: 'plane',
  src: { p, n },
  dst: { p: apply(T, add(p, off)), n: neg(rotate(T.R, n)) },
});
/** Achse; dst.p um `along` entlang der Achse versetzt, dst.d bei reverse umgekehrt. */
const axis = (T: Rigid, p: Vec3, d: Vec3, along = 0, reverse = false): Constraint => ({
  kind: 'axis',
  src: { p, d },
  dst: { p: apply(T, add(p, scale(normalize(d), along))), d: scale(rotate(T.R, d), reverse ? -1 : 1) },
});

/** Abstand eines Punkts von der Geraden durch p mit Richtung d. */
function lineDist(q: Vec3, p: Vec3, d: Vec3): number {
  const n = normalize(d);
  const v = sub(q, p);
  return len(sub(v, scale(n, dot(v, n))));
}

/** Größte Verletzung der harten Bedingungen (mm bzw. Längeneinheiten). */
function violation(cons: Constraint[], T: Rigid, flips: boolean[]): number {
  let m = 0;
  cons.forEach((c, i) => {
    if (c.kind === 'point') m = Math.max(m, dist(apply(T, c.src), c.dst));
    else if (c.kind === 'plane') {
      m = Math.max(m, len(add(rotate(T.R, normalize(c.src.n)), normalize(c.dst.n))));
      m = Math.max(m, Math.abs(dot(sub(apply(T, c.src.p), c.dst.p), normalize(c.dst.n))));
    } else {
      m = Math.max(m, len(sub(rotate(T.R, normalize(c.src.d)), scale(normalize(c.dst.d), flips[i] ? -1 : 1))));
      m = Math.max(m, lineDist(apply(T, c.src.p), c.dst.p, c.dst.d));
    }
  });
  return m;
}

/** Größter Abstand zweier Lagen an Prüfpunkten. */
const probeErr = (A: Rigid, B: Rigid, probes: Vec3[]) => Math.max(...probes.map((p) => dist(apply(A, p), apply(B, p))));

/** Echte Drehung: orthonormal und Determinante +1. */
function expectRotation(T: Rigid) {
  const R = T.R;
  const c0: Vec3 = [R[0], R[3], R[6]];
  const c1: Vec3 = [R[1], R[4], R[7]];
  const c2: Vec3 = [R[2], R[5], R[8]];
  for (const [a, b, v] of [[c0, c0, 1], [c1, c1, 1], [c2, c2, 1], [c0, c1, 0], [c0, c2, 0], [c1, c2, 0]] as [Vec3, Vec3, number][])
    expect(Math.abs(dot(a, b) - v)).toBeLessThan(1e-9);
  expect(dot(cross(c0, c1), c2)).toBeCloseTo(1, 9);
}

/** Jede freie Bewegung muss die Bedingungen erfüllt lassen. */
function expectMotionsKeep(cons: Constraint[], sol: ConstraintSolution, tol = 1e-6) {
  for (const m of sol.free)
    for (const amount of [0.7, -2.3, 12]) {
      const T = applyMotion(sol.T, m, m.kind === 'rotate' ? amount / 4 : amount);
      expect(violation(cons, T, sol.flips)).toBeLessThan(tol);
    }
}

type Rot = Extract<FreeMotion, { kind: 'rotate' }>;
const rotations = (s: ConstraintSolution) => s.free.filter((m): m is Rot => m.kind === 'rotate');
const translations = (s: ConstraintSolution) => s.free.filter((m) => m.kind === 'translate');
/** Lösungen, die die Oberfläche als passend anbietet (gleiche Schwelle wie in main.ts). */
const fitting = (r: ConstraintSolution[], tol = 0.1) => r.filter((s) => s.angleError < 1 && s.distError < tol);

/** Rigid → THREE.Matrix4 (für transformPositions). */
function toMatrix4(T: Rigid): THREE.Matrix4 {
  const R = T.R;
  return new THREE.Matrix4().set(R[0], R[1], R[2], T.t[0], R[3], R[4], R[5], T.t[1], R[6], R[7], R[8], T.t[2], 0, 0, 0, 1);
}

// ---------------------------------------------------------------------------------------------
// Merkmale direkt aus Dreiecksnetzen (nicht indiziert, 9 Werte je Dreieck)

const triVerts = (pos: Float32Array, i: number): [Vec3, Vec3, Vec3] => [
  [pos[9 * i], pos[9 * i + 1], pos[9 * i + 2]],
  [pos[9 * i + 3], pos[9 * i + 4], pos[9 * i + 5]],
  [pos[9 * i + 6], pos[9 * i + 7], pos[9 * i + 8]],
];

/** Indizes der Dreiecke, deren Normale und Schwerpunkt (im Ausgangsnetz) die Bedingung erfüllen. */
function selectTris(pos: Float32Array, pred: (n: Vec3, c: Vec3) => boolean): number[] {
  const out: number[] = [];
  for (let i = 0; i < pos.length / 9; i++) {
    const [a, b, c] = triVerts(pos, i);
    const cr = cross(sub(b, a), sub(c, a));
    if (len(cr) < 1e-14) continue;
    if (pred(normalize(cr), scale(add(add(a, b), c), 1 / 3))) out.push(i);
  }
  return out;
}

/** Ebene: flächengewichteter Schwerpunkt und Normale der gewählten Dreiecke. */
function planeOf(pos: Float32Array, tris: number[]): { p: Vec3; n: Vec3 } {
  let A: Vec3 = [0, 0, 0];
  let C: Vec3 = [0, 0, 0];
  let W = 0;
  for (const i of tris) {
    const [a, b, c] = triVerts(pos, i);
    const cr = cross(sub(b, a), sub(c, a));
    const w = len(cr);
    A = add(A, cr);
    C = add(C, scale(add(add(a, b), c), w / 3));
    W += w;
  }
  return { p: scale(C, 1 / W), n: normalize(A) };
}

/** Zylinderachse: Richtung senkrecht zu allen Mantelnormalen (kleinster Eigenvektor), Punkt = Schwerpunkt. */
function axisOf(pos: Float32Array, tris: number[]): { p: Vec3; d: Vec3 } {
  const M = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 1e9],
  ];
  let C: Vec3 = [0, 0, 0];
  let W = 0;
  for (const i of tris) {
    const [a, b, c] = triVerts(pos, i);
    const cr = cross(sub(b, a), sub(c, a));
    const w = len(cr);
    const n = scale(cr, 1 / w);
    for (let r = 0; r < 3; r++) for (let k = 0; k < 3; k++) M[r][k] += w * n[r] * n[k];
    C = add(C, scale(add(add(a, b), c), w / 3));
    W += w;
  }
  const { values, vectors } = jacobiEigen4(M);
  let best = 0;
  for (let k = 1; k < 4; k++) if (values[k] < values[best]) best = k;
  const d = normalize([vectors[best][0], vectors[best][1], vectors[best][2]]);
  // Vorzeichen wie in der Flächenerkennung: größte Komponente positiv (also beliebig zur Gegenseite)
  let m = 0;
  if (Math.abs(d[1]) > Math.abs(d[m])) m = 1;
  if (Math.abs(d[2]) > Math.abs(d[m])) m = 2;
  return { p: scale(C, 1 / W), d: d[m] < 0 ? neg(d) : d };
}

/** Wackelt jede Ecke einzeln (nicht verschweißte Nähte, ungenauer Export). */
function jitter(pos: Float32Array, amount: number, r: () => number): Float32Array {
  const out = new Float32Array(pos.length);
  for (let i = 0; i < pos.length; i++) out[i] = pos[i] + (r() * 2 - 1) * amount;
  return out;
}

/**
 * Platte mit zentrischer Bohrung um die Z-Achse, Unterseite bei z = 0, als einfaches Streifennetz
 * (8 Dreiecke je Segment; schnell auch für sehr feine Teilungen). Normalen nach außen.
 */
function fineRing(rOut: number, rIn: number, h: number, seg: number): Float32Array {
  const out = new Float32Array(seg * 8 * 9);
  let o = 0;
  const tri = (a: Vec3, b: Vec3, c: Vec3) => {
    out.set(a, o);
    out.set(b, o + 3);
    out.set(c, o + 6);
    o += 9;
  };
  for (let i = 0; i < seg; i++) {
    const a0 = (2 * Math.PI * i) / seg;
    const a1 = (2 * Math.PI * (i + 1)) / seg;
    const P = (r: number, a: number, z: number): Vec3 => [r * Math.cos(a), r * Math.sin(a), z];
    // Außenmantel (Normale radial nach außen)
    tri(P(rOut, a0, 0), P(rOut, a1, 0), P(rOut, a1, h));
    tri(P(rOut, a0, 0), P(rOut, a1, h), P(rOut, a0, h));
    // Bohrungswand (Normale zur Achse hin)
    tri(P(rIn, a0, 0), P(rIn, a1, h), P(rIn, a1, 0));
    tri(P(rIn, a0, 0), P(rIn, a0, h), P(rIn, a1, h));
    // Oberseite (+Z) und Unterseite (−Z)
    tri(P(rIn, a0, h), P(rOut, a0, h), P(rOut, a1, h));
    tri(P(rIn, a0, h), P(rOut, a1, h), P(rIn, a1, h));
    tri(P(rIn, a0, 0), P(rOut, a1, 0), P(rOut, a0, 0));
    tri(P(rIn, a0, 0), P(rIn, a1, 0), P(rOut, a1, 0));
  }
  return out;
}

/**
 * Zapfen mit Schulter in Platte mit Bohrung, Bedingungen aus den Netzen bestimmt:
 * bewegliches Teil = Quader 40×30×5 mit Zapfen ⌀ 8 bei (8, 5) auf der Oberseite (Schulter z = 5);
 * festes Teil = Platte mit Bohrung ⌀ 8,2, mit Tring in die Welt gelegt (Float32), optional verrauscht.
 */
function pegIntoPlate(seg: number, noise: number, seed: number) {
  const r = rng(seed);
  const peg = transformPositions(cylinder(4, 10, seg), new THREE.Matrix4().makeTranslation(8, 5, 5));
  const moving = jitter(concat(box(40, 30, 5), peg), noise, r);
  const Tring = randomRigid(r, 300);
  const plateLocal = fineRing(20, 4.1, 6, seg);
  const fixed = jitter(transformPositions(plateLocal, toMatrix4(Tring)), noise, r);

  const movingRef = concat(box(40, 30, 5), peg);
  const shoulder = selectTris(movingRef, (n, c) => n[2] > 0.99 && Math.abs(c[2] - 5) < 1e-3 && Math.hypot(c[0] - 8, c[1] - 5) > 4.2);
  const mantle = selectTris(movingRef, (n, c) => Math.abs(n[2]) < 0.1 && Math.hypot(c[0] - 8, c[1] - 5) < 4.5);
  const bottom = selectTris(plateLocal, (n) => n[2] < -0.99);
  const bore = selectTris(plateLocal, (n, c) => Math.abs(n[2]) < 0.1 && Math.hypot(c[0], c[1]) < 4.5);

  const cons: Constraint[] = [
    { kind: 'plane', src: planeOf(moving, shoulder), dst: planeOf(fixed, bottom) },
    { kind: 'axis', src: axisOf(moving, mantle), dst: axisOf(fixed, bore) },
  ];
  // Wahre Lage (bis auf die Drehung um die Achse): Zapfenachse auf Bohrungsachse, Schulter auf Unterseite
  const Ttrue = compose(Tring, { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: [-8, -5, -5] });
  return { cons, Tring, Ttrue, triangles: (moving.length + fixed.length) / 9 };
}

/** Prüft den Sitz des Zapfens in der Bohrung (Platte im Weltsystem über Tring). */
function expectPegSeated(s: ConstraintSolution, Tring: Rigid, tol: number) {
  const holeAxis = rotate(Tring.R, [0, 0, 1]);
  const holeP = apply(Tring, [0, 0, 0]);
  // Zapfenachse fällt mit der Bohrungsachse zusammen
  for (const q of [[8, 5, 5], [8, 5, 15]] as Vec3[]) expect(lineDist(apply(s.T, q), holeP, holeAxis)).toBeLessThan(tol);
  // Schulter liegt auf der Plattenunterseite, der Zapfen ragt IN die Platte (nicht von ihr weg)
  expect(Math.abs(dot(sub(apply(s.T, [0, 0, 5]), holeP), holeAxis))).toBeLessThan(tol);
  expect(dot(sub(apply(s.T, [8, 5, 15]), holeP), holeAxis)).toBeCloseTo(10, 1);
  expect(dot(sub(apply(s.T, [0, 0, 0]), holeP), holeAxis)).toBeCloseTo(-5, 1);
}

// ---------------------------------------------------------------------------------------------

describe('solveConstraints – Größenordnungen und Lage im Raum', () => {
  it('winzige (0,15 mm) und große (1 m) Teile: exakte Lage und richtige freie Bewegungen', () => {
    const r = rng(1);
    for (const k of [0.005, 0.05, 1, 30]) {
      const T = randomRigid(r, 200 * k);
      const P = (v: Vec3): Vec3 => scale(v, k);
      const probes = [P([0, 0, 0]), P([30, 30, 30])];

      const three = solveConstraints([point(T, P([0, 0, 0])), point(T, P([30, 0, 0])), point(T, P([5, 20, 8]))]);
      expect(three.length).toBe(1);
      expect(probeErr(three[0].T, T, probes)).toBeLessThan(1e-7 * Math.max(1, k * 30));
      expect(three[0].free).toEqual([]);

      const p2 = [plane(T, P([0, 0, 0]), [0, 0, -1], P([15, -4, 0])), point(T, P([10, 4, 0])), point(T, P([-6, 12, 0]))];
      const rp2 = solveConstraints(p2);
      expect(probeErr(rp2[0].T, T, probes)).toBeLessThan(1e-7 * Math.max(1, k * 30));
      expect(rp2[0].free).toEqual([]);

      const peg = [plane(T, P([0, 0, 0]), [0, 0, -1], P([12, -7, 0])), axis(T, P([0, 0, -4]), [0, 0, 1], -6 * k)];
      const rpeg = solveConstraints(peg);
      expect(rpeg[0].flips).toEqual([false, false]);
      expect(violation(peg, rpeg[0].T, rpeg[0].flips)).toBeLessThan(1e-8 * Math.max(1, k * 30));
      expect(rpeg[0].free.length).toBe(1);
      expect(rotations(rpeg[0]).length).toBe(1);
      expectMotionsKeep(peg, rpeg[0], 1e-6 * Math.max(1, k * 30));

      const one = solveConstraints([plane(T, P([4, -2, 0]), [0, 0, -1], P([7, 3, 0]))]);
      expect(one.length).toBe(1);
      expect(translations(one[0]).length).toBe(2);
      expect(rotations(one[0]).length).toBe(1);

      const ax = solveConstraints([axis(T, P([3, 1, -2]), [0, 0, 1], 9 * k)]);
      expect(ax.length).toBe(2);
      for (const s of ax) {
        expect(translations(s).length).toBe(1);
        expect(rotations(s).length).toBe(1);
      }
    }
  });

  it('kleine Merkmale in großer Szene (scale = 500 wie in der Oberfläche): gleiche Ergebnisse', () => {
    // Zapfen ⌀ 3 mm, Schulter 2 mm daneben: L ist viel größer als die Merkmale
    const peg = [plane(T0, [0, 0, 0], [0, 0, -1], [2, -1, 0]), axis(T0, [0, 0, -1], [0, 0, 1], -1)];
    const rpeg = solveConstraints(peg, { scale: 500 });
    expect(violation(peg, rpeg[0].T, rpeg[0].flips)).toBeLessThan(1e-8);
    expect(rpeg[0].free.length).toBe(1);
    expect(rotations(rpeg[0]).length).toBe(1);
    const rot = rotations(rpeg[0])[0];
    expect(lineDist(rot.point, apply(T0, [0, 0, 0]), rotate(T0.R, [0, 0, 1]))).toBeLessThan(1e-6);

    const three = [point(T0, [0, 0, 0]), point(T0, [2, 0, 0]), point(T0, [0.5, 1.5, 0.3])];
    const r3 = solveConstraints(three, { scale: 500 });
    expect(r3.length).toBe(1);
    expect(probeErr(r3[0].T, T0, [[0, 0, 0], [10, 10, 10]])).toBeLessThan(1e-6);
    expect(r3[0].free).toEqual([]);

    const pp = [plane(T0, [0, 0, 0], [0, 0, -1], [1, 1, 0]), point(T0, [1.5, 0.5, 0])];
    const rpp = solveConstraints(pp, { scale: 500 });
    expect(rpp.length).toBe(1);
    expect(rpp[0].free.length).toBe(1);
    expect(rotations(rpp[0]).length).toBe(1);
  });

  it('Zapfen ⌀ 1 mm in sehr großer Szene (scale bis 30 000) mit starker Zentrierung: trotzdem exakt', () => {
    const peg = [plane(T0, [0, 0, 0], [0, 0, -1], [0.8, 0.3, 0]), axis(T0, [0, 0, -0.5], [0, 0, 1], -0.3)];
    for (const s of [300, 3000, 30000]) {
      const r = solveConstraints(peg, { scale: s, softWeight: 0.1 });
      expect(r[0].flips).toEqual([false, false]);
      expect(violation(peg, r[0].T, r[0].flips)).toBeLessThan(1e-9);
      expect(rotations(r[0]).length).toBe(1);
      expect(r[0].free.length).toBe(1);
    }
  });

  it('Teile 10 m vom Ursprung entfernt: exakt und mit freien Bewegungen', () => {
    const Tfar: Rigid = { R: axisAngle([1, 2, 3], 2.5), t: [20000, -15000, 8000] };
    const o: Vec3 = [5000, 5000, 5000];
    const three = solveConstraints([point(Tfar, o), point(Tfar, add(o, [30, 0, 0])), point(Tfar, add(o, [5, 20, 8]))]);
    expect(probeErr(three[0].T, Tfar, [o, add(o, [30, 30, 30])])).toBeLessThan(1e-6);
    const peg = [plane(Tfar, o, [0, 0, -1], [12, -7, 0]), axis(Tfar, add(o, [0, 0, -4]), [0, 0, 1], -6)];
    const rpeg = solveConstraints(peg);
    expect(violation(peg, rpeg[0].T, rpeg[0].flips)).toBeLessThan(1e-6);
    expect(rotations(rpeg[0]).length).toBe(1);
    expect(rpeg[0].free.length).toBe(1);
    expectMotionsKeep(peg, rpeg[0], 1e-5);
  });

  it('zufällige Lagen bis 180°: Ecke aus drei Ebenen, Zapfen mit Punkt, Ebene mit zwei Punkten', () => {
    const r = rng(7);
    for (let k = 0; k < 60; k++) {
      const T = randomRigid(r, 600);
      const probes: Vec3[] = [[0, 0, 0], [10, 10, 10], [-20, 5, 3]];
      // Würfelecke in eine Raumecke, Flächenmitten weit auseinander
      const corner = [
        plane(T, [5, 5, 0], [0, 0, -1], [80 * (r() - 0.5), 80 * (r() - 0.5), 0]),
        plane(T, [0, 5, 5], [-1, 0, 0], [0, 80 * (r() - 0.5), 80 * (r() - 0.5)]),
        plane(T, [5, 0, 5], [0, -1, 0], [80 * (r() - 0.5), 0, 80 * (r() - 0.5)]),
      ];
      const rc = solveConstraints(corner);
      expect(probeErr(rc[0].T, T, probes)).toBeLessThan(1e-6);
      expect(rc[0].free).toEqual([]);
      expectRotation(rc[0].T);

      const pegPt = [plane(T, [0, 0, 0], [0, 0, -1], [30 * r(), 30 * r(), 0]), axis(T, [0, 0, -4], [0, 0, 1], -6 * r()), point(T, [8, 3, 0])];
      const rp = solveConstraints(pegPt);
      expect(probeErr(rp[0].T, T, probes)).toBeLessThan(1e-6);
      expect(rp[0].free).toEqual([]);

      const p2 = [plane(T, [0, 0, 0], [0, 0, -1], [15, -4, 0]), point(T, [10, 4, 0]), point(T, [-6, 12, 0])];
      expect(probeErr(solveConstraints(p2)[0].T, T, probes)).toBeLessThan(1e-6);
    }
  });

  it('genau 180° um verschiedene Achsen wird gefunden', () => {
    for (const ax of [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 1, 1], [0, 0, -1]] as Vec3[]) {
      const T: Rigid = { R: axisAngle(ax, Math.PI), t: [3, -4, 5] };
      const a = [plane(T, [0, 0, 0], [0, 0, -1], [15, -4, 0]), point(T, [10, 4, 0]), point(T, [-6, 12, 0])];
      expect(probeErr(solveConstraints(a)[0].T, T, [[0, 0, 0], [10, 10, 10]])).toBeLessThan(1e-6);
      const b = [plane(T, [0, 0, 0], [0, 0, -1], [-20, 8, 0]), axis(T, [0, 0, -4], [0, 0, 1], -6), point(T, [8, 3, 0])];
      expect(probeErr(solveConstraints(b)[0].T, T, [[0, 0, 0], [10, 10, 10]])).toBeLessThan(1e-6);
    }
  });

  it('Startlage genau falsch herum (Fläche zeigt in die Gegenrichtung): trotzdem exakt', () => {
    const peg = [plane(T0, [0, 0, 0], [0, 0, -1], [12, -7, 0]), axis(T0, [0, 0, -4], [0, 0, 1], -6)];
    const upsideDown = compose(rotationAbout(apply(T0, [0, 0, 0]), rotate(T0.R, [1, 0, 0]), Math.PI), T0);
    const r = solveConstraints(peg, { init: upsideDown });
    expect(r[0].flips).toEqual([false, false]);
    expect(violation(peg, r[0].T, r[0].flips)).toBeLessThan(1e-8);
    const one = [plane(T0, [0, 0, 0], [0, 0, -1], [12, -7, 0])];
    const r1 = solveConstraints(one, { init: upsideDown });
    expect(violation(one, r1[0].T, r1[0].flips)).toBeLessThan(1e-8);
  });
});

describe('solveConstraints – Bedeutung der Bedingungen und freie Bewegungen', () => {
  it('Stift liegt flach in einer Nut (Achse parallel zur Fläche): beide Richtungen exakt, nur Verschieben frei', () => {
    const cons = [plane(T0, [0, 0, 0], [0, 0, -1], [3, 2, 0]), axis(T0, [0, 0, 5], [1, 0, 0], 4)];
    const r = solveConstraints(cons);
    const exact = r.filter((s) => s.distError < 1e-8 && s.angleError < 1e-6);
    expect(exact.length).toBe(2);
    expect(exact.map((s) => s.flips[1]).sort()).toEqual([false, true]);
    for (const s of exact) {
      expect(violation(cons, s.T, s.flips)).toBeLessThan(1e-8);
      expect(s.free.length).toBe(1);
      expect(s.free[0].kind).toBe('translate');
      if (cons[1].kind !== 'axis') throw new Error();
      expect(Math.abs(dot(s.free[0].axis, normalize(cons[1].dst.d)))).toBeCloseTo(1, 9);
      expectMotionsKeep(cons, s);
    }
  });

  it('Punkt auf der Achse: beide Richtungen exakt, Drehung um die Achse durch den Punkt frei', () => {
    const cons = [axis(T0, [0, 0, 0], [0, 0, 1], 5), point(T0, [0, 0, 7])];
    const r = solveConstraints(cons);
    expect(r.length).toBe(2);
    expect(r.map((s) => s.flips[0]).sort()).toEqual([false, true]);
    if (cons[0].kind !== 'axis' || cons[1].kind !== 'point') throw new Error();
    for (const s of r) {
      expect(violation(cons, s.T, s.flips)).toBeLessThan(1e-8);
      expect(s.flips[1]).toBe(false);
      expect(s.free.length).toBe(1);
      const rot = rotations(s)[0];
      expect(Math.abs(dot(rot.axis, cons[0].dst.d))).toBeCloseTo(1, 9);
      expect(lineDist(cons[1].dst, rot.point, rot.axis)).toBeLessThan(1e-6);
      expectMotionsKeep(cons, s);
    }
  });

  it('Welle in zwei Lagern (zwei Achsen auf einer Linie): Drehen und Schieben entlang der Achse', () => {
    const cons = [axis(T0, [0, 0, 0], [0, 0, 1], 2), axis(T0, [0, 0, 40], [0, 0, 1], -3)];
    const r = solveConstraints(cons);
    const exact = r.filter((s) => s.distError < 1e-8 && s.angleError < 1e-6);
    expect(exact.map((s) => s.flips.join())).toEqual(['false,false', 'true,true']);
    for (const s of exact) {
      expect(rotations(s).length).toBe(1);
      expect(translations(s).length).toBe(1);
      expectMotionsKeep(cons, s);
    }
  });

  it('Nut aus zwei gegenüberliegenden Ebenen: zwei Verschiebungen quer zur Normalen, Drehung um die Normale', () => {
    const cons = [plane(T0, [0, 0, 0], [-1, 0, 0], [0, 3, 2]), plane(T0, [10, 0, 0], [1, 0, 0], [0, -4, 1])];
    const r = solveConstraints(cons);
    expect(r.length).toBe(1);
    const s = r[0];
    const nx = rotate(T0.R, [1, 0, 0]);
    expect(violation(cons, s.T, s.flips)).toBeLessThan(1e-8);
    expect(translations(s).length).toBe(2);
    for (const t of translations(s)) expect(Math.abs(dot(t.axis, nx))).toBeLessThan(1e-9);
    expect(rotations(s).length).toBe(1);
    expect(Math.abs(dot(rotations(s)[0].axis, nx))).toBeCloseTo(1, 9);
    expectMotionsKeep(cons, s);
  });

  it('Ebene und zwei Punkte entlang der Normalen: Drehung um diese Linie bleibt frei', () => {
    const cons = [plane(T0, [0, 0, 0], [0, 0, -1], [5, 5, 0]), point(T0, [3, 3, 2]), point(T0, [3, 3, 10])];
    const r = solveConstraints(cons);
    expect(r.length).toBe(1);
    const s = r[0];
    expect(s.free.length).toBe(1);
    const rot = rotations(s)[0];
    expect(Math.abs(dot(rot.axis, rotate(T0.R, [0, 0, 1])))).toBeCloseTo(1, 9);
    expect(lineDist(apply(T0, [3, 3, 0]), rot.point, rot.axis)).toBeLessThan(1e-6);
    expectMotionsKeep(cons, s);
  });

  it('Deckel auf Dose (Kreisfläche + Mittelpunkt + Achse, stimmig überbestimmt): nur Drehung um die Achse', () => {
    const cons = [
      plane(T0, [0, 0, 0], [0, 0, -1], [0, 0, 0]),
      point(T0, [0, 0, 0]),
      axis(T0, [0, 0, -6], [0, 0, 1], 2),
    ];
    const r = solveConstraints(cons);
    const s = r[0];
    expect(s.flips[2]).toBe(false);
    expect(violation(cons, s.T, s.flips)).toBeLessThan(1e-8);
    expect(s.free.length).toBe(1);
    const rot = rotations(s)[0];
    expect(Math.abs(dot(rot.axis, rotate(T0.R, [0, 0, 1])))).toBeCloseTo(1, 9);
    expect(lineDist(apply(T0, [0, 0, 0]), rot.point, rot.axis)).toBeLessThan(1e-6);
    expectMotionsKeep(cons, s);
  });

  it('Drehpunkte und Richtungen der freien Bewegungen stimmen bei zufälligen Lagen', () => {
    const r = rng(3);
    for (let k = 0; k < 30; k++) {
      const T = randomRigid(r, 400);
      const nW = rotate(T.R, [0, 0, 1]);

      // Eine Ebene: Drehung um die Normale durch die Mitte der Gegenfläche
      const c1 = plane(T, [4, -2, 0], [0, 0, -1], [20 * r(), 20 * r(), 0]);
      if (c1.kind !== 'plane') throw new Error();
      const s1 = solveConstraints([c1])[0];
      const r1 = rotations(s1);
      expect(r1.length).toBe(1);
      expect(dot(r1[0].axis, c1.dst.n)).toBeCloseTo(1, 9);
      expect(lineDist(c1.dst.p, r1[0].point, r1[0].axis)).toBeLessThan(1e-6);
      for (const t of translations(s1)) expect(Math.abs(dot(t.axis, nW))).toBeLessThan(1e-9);
      expectMotionsKeep([c1], s1);

      // Eine Achse: Drehpunkt auf der Achse, Verschieben entlang der Achse
      const c2 = axis(T, [3, 1, -2], [0, 0, 1], 9 * r());
      if (c2.kind !== 'axis') throw new Error();
      for (const s of solveConstraints([c2])) {
        expect(lineDist(rotations(s)[0].point, c2.dst.p, c2.dst.d)).toBeLessThan(1e-6);
        expect(Math.abs(dot(translations(s)[0].axis, nW))).toBeCloseTo(1, 9);
        expectMotionsKeep([c2], s);
      }

      // Zwei Punkte: Drehung um die Verbindungslinie
      const c3 = [point(T, [0, 0, 0]), point(T, [20, 5, 0])];
      const s3 = solveConstraints(c3)[0];
      expect(s3.free.length).toBe(1);
      const r3 = rotations(s3)[0];
      expect(Math.abs(dot(r3.axis, normalize(rotate(T.R, [20, 5, 0]))))).toBeCloseTo(1, 9);
      expect(lineDist(apply(T, [0, 0, 0]), r3.point, r3.axis)).toBeLessThan(1e-6);
      expectMotionsKeep(c3, s3);

      // Ein Punkt: drei Drehungen um diesen Punkt
      const c4 = point(T, [3, 4, 5]);
      if (c4.kind !== 'point') throw new Error();
      const s4 = solveConstraints([c4])[0];
      expect(rotations(s4).length).toBe(3);
      for (const m of rotations(s4)) expect(dist(m.point, c4.dst)).toBeLessThan(1e-6);
    }
  });

  it('nicht normierte Normalen und Richtungen wirken wie normierte', () => {
    const unit = [plane(T0, [0, 0, 0], [0, 0, -1], [12, -7, 0]), axis(T0, [0, 0, -4], [0, 0, 1], -6), point(T0, [8, 3, 0])];
    const scaled: Constraint[] = unit.map((c) => {
      if (c.kind === 'plane') return { ...c, src: { p: c.src.p, n: scale(c.src.n, 7) }, dst: { p: c.dst.p, n: scale(c.dst.n, 0.01) } };
      if (c.kind === 'axis') return { ...c, src: { p: c.src.p, d: scale(c.src.d, 0.02) }, dst: { p: c.dst.p, d: scale(c.dst.d, 50) } };
      return c;
    });
    const s = solveConstraints(scaled)[0];
    expect(probeErr(s.T, T0, [[0, 0, 0], [10, 10, 10]])).toBeLessThan(1e-6);
    expect(s.distError).toBeLessThan(1e-8);
    expect(s.angleError).toBeLessThan(1e-6);
  });

  it('Gewichte: stark gewichtete Bedingungen setzen sich gegen eine falsche durch', () => {
    const cons: Constraint[] = [
      { ...point(T0, [0, 0, 0]), weight: 1000 },
      { ...point(T0, [30, 0, 0]), weight: 1000 },
      { ...point(T0, [5, 20, 8]), weight: 1000 },
      { kind: 'point', src: [10, 10, 10], dst: add(apply(T0, [10, 10, 10]), [5, 0, 0]), weight: 1e-3 },
    ];
    const s = solveConstraints(cons)[0];
    expect(probeErr(s.T, T0, [[0, 0, 0], [30, 0, 0], [5, 20, 8]])).toBeLessThan(1e-3);
    // Gemeldet wird die ungewichtete Abweichung: der falsche Punkt liegt ~5 mm daneben
    expect(s.distError).toBeGreaterThan(2);
  });

  it('Eingaben werden nicht verändert, Lösungen sind echte Drehungen und paarweise verschieden', () => {
    const cons = [plane(T0, [0, 0, 0], [0, 0, -2], [12, -7, 0]), axis(T0, [0, 0, -4], [0, 0, 3], -6), point(T0, [8, 3, 0])];
    const init: Rigid = { R: axisAngle([0, 1, 0], 1), t: [1, 2, 3] };
    const before = JSON.stringify({ cons, init });
    const r = solveConstraints(cons, { init });
    expect(JSON.stringify({ cons, init })).toBe(before);
    for (const s of r) expectRotation(s.T);
    for (let i = 0; i < r.length; i++)
      for (let j = i + 1; j < r.length; j++) expect(probeErr(r[i].T, r[j].T, [[0, 0, 0], [20, 0, 0], [0, 20, 0], [0, 0, 20]])).toBeGreaterThan(1e-3);
  });
});

describe('solveConstraints – viele Achsen ohne Richtungsangabe', () => {
  it('fünf parallele Stifte, deren Achsen im festen Teil umgekehrt erkannt wurden: exakte Lösung zuerst', () => {
    const pins: Constraint[] = [];
    for (let i = 0; i < 5; i++) pins.push(axis(T0, [i * 10, (i * i) % 7, 0], [0, 0, 1], 0, true));
    const r = solveConstraints(pins);
    expect(r[0].distError).toBeLessThan(1e-6);
    expect(r[0].angleError).toBeLessThan(1e-6);
    expect(r[0].flips).toEqual([true, true, true, true, true]);
  });

  it('Flansch: Schulterfläche und sechs Schraubenlöcher, Achsen im festen Teil umgekehrt erkannt', () => {
    // Parallele Bohrungen bekommen in der Flächenerkennung dasselbe Vorzeichen: alle umgekehrt
    const flange: Constraint[] = [plane(T0, [0, 0, 0], [0, 0, -1], [3, 4, 0])];
    for (let i = 0; i < 6; i++) {
      const a = (i * Math.PI) / 3 + 0.2;
      flange.push(axis(T0, [Math.cos(a) * 40, Math.sin(a) * 40, 0], [0, 0, 1], 0, true));
    }
    const s = solveConstraints(flange)[0];
    expect(s.flips).toEqual([false, true, true, true, true, true, true]);
    expect(s.distError).toBeLessThan(1e-6);
    expect(s.angleError).toBeLessThan(1e-6);
    expect(probeErr(s.T, T0, [[0, 0, 0], [40, 0, 0], [0, 40, 0]])).toBeLessThan(1e-5);
    expect(s.free).toEqual([]);
  });

  it('acht Stifte im Kreis, Achsrichtungen abwechselnd erkannt: exakte Lösung wird gefunden', () => {
    const pins: Constraint[] = [];
    for (let i = 0; i < 8; i++) pins.push(axis(T0, [Math.cos(i) * 30, Math.sin(i) * 30, 0], [0, 0, 1], 0, i % 2 === 0));
    const s = solveConstraints(pins)[0];
    expect(s.distError).toBeLessThan(1e-6);
    expect(s.angleError).toBeLessThan(1e-6);
    expect(s.flips).toEqual([true, false, true, false, true, false, true, false]);
  });

  it('zwei Stiftgruppen (senkrecht zueinander) mit je drei Stiften, Richtungen beliebig erkannt: exakt', () => {
    const pins: Constraint[] = [];
    for (let i = 0; i < 3; i++) pins.push(axis(T0, [i * 12, 0, 0], [0, 0, 1], 1, i === 1));
    for (let i = 0; i < 3; i++) pins.push(axis(T0, [0, 5 + i * 9, 20], [1, 0, 0], -2, i !== 0));
    const r = solveConstraints(pins);
    expect(r[0].distError).toBeLessThan(1e-6);
    expect(r[0].angleError).toBeLessThan(1e-6);
    expect(r[0].flips).toEqual([false, true, false, false, true, true]);
    expect(probeErr(r[0].T, T0, [[0, 0, 0], [30, 30, 30]])).toBeLessThan(1e-5);
    expect(r[0].free).toEqual([]);
  });

  it('sechs Achsen, die sich paarweise widersprechen (mehr Gruppen als durchprobiert werden): kein Absturz', () => {
    const S: Vec3[] = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 0, 0], [0, 1, 0], [0, 0, 1]];
    const D: Vec3[] = [[1, 0, 0], [1, 0, 0], [1, 0, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0]];
    const cons: Constraint[] = S.map((d, i) => ({ kind: 'axis', src: { p: [i * 5, 0, 0], d }, dst: { p: [0, i * 5, 0], d: D[i] } }));
    const r = solveConstraints(cons);
    expect(r.length).toBeGreaterThan(0);
    for (const s of r) {
      expectRotation(s.T);
      expect(s.flips.length).toBe(6);
      expect(Number.isFinite(s.distError) && Number.isFinite(s.angleError) && Number.isFinite(s.cost)).toBe(true);
    }
  });
});

describe('solveConstraints – leichtes Messrauschen (Merkmale nie ganz exakt)', () => {
  /** Zapfen mit Schulter, Schulternormale des beweglichen Teils um e gegen die Zapfenachse verkippt. */
  const tiltedShoulder = (e: number): Constraint[] => [
    { kind: 'plane', src: { p: [0, 0, 0], n: [Math.sin(e), 0, -Math.cos(e)] }, dst: { p: apply(T0, [12, -7, 0]), n: rotate(T0.R, [0, 0, 1]) } },
    axis(T0, [0, 0, -4], [0, 0, 1], -6),
  ];

  it('Schulter 0,01° schief zur Zapfenachse (Messrauschen): Drehung um die Achse bleibt frei', () => {
    // Abweichung wie sie beim Einpassen von Ebene und Zylinder in ein tesselliertes Netz entsteht.
    // Die Kosten ändern sich beim Drehen um die Bohrungsachse überhaupt nicht.
    const s = solveConstraints(tiltedShoulder(deg(0.01)))[0];
    expect(s.flips).toEqual([false, false]);
    expect(s.angleError).toBeLessThan(0.01);
    expect(rotations(s).length).toBe(1);
    expect(s.free.length).toBe(1);
    expect(Math.abs(dot(rotations(s)[0].axis, rotate(T0.R, [0, 0, 1])))).toBeGreaterThan(Math.cos(deg(0.1)));
    expect(lineDist(rotations(s)[0].point, apply(T0, [0, 0, 0]), rotate(T0.R, [0, 0, 1]))).toBeLessThan(0.01);
  });

  it('Schulter 0,01° schief zur Zapfenachse: nur eine passende Lösung (keine nur verdrehten Kopien)', () => {
    for (const scaleOpt of [undefined, 300]) {
      const ok = fitting(solveConstraints(tiltedShoulder(deg(0.01)), { scale: scaleOpt }));
      expect(ok.length).toBe(1);
    }
  });

  it('Gegenfläche 0,05° verkippt, Flächenmitte neben der Achse: keine freie Verschiebung entlang der Achse', () => {
    // Die Schulter legt die Höhe fest; eine gemeldete Verschiebung hebt sie von der Platte ab
    const e = deg(0.05);
    const cons: Constraint[] = [
      { kind: 'plane', src: { p: [-8, -5, 0], n: [0, 0, -1] }, dst: { p: apply(T0, [-8, -5, 0]), n: rotate(T0.R, [Math.sin(e), 0, Math.cos(e)]) } },
      axis(T0, [0, 0, -4], [0, 0, 1], -6),
    ];
    const s = solveConstraints(cons)[0];
    expect(s.distError).toBeLessThan(1e-6);
    expect(translations(s)).toEqual([]);
    expect(rotations(s).length).toBe(1);
    // Jede gemeldete Bewegung muss den Sitz (bis auf das Rauschen) erhalten
    expectMotionsKeep(cons, s, 0.05);
  });

  it('Schulterfläche mit Mitte neben der Achse, Gegenfläche 0,005° verkippt: Drehung bleibt frei', () => {
    // Quader mit seitlich versetztem Zapfen: Flächenmitte 9 mm neben der Achse
    const e = deg(0.005);
    const cons: Constraint[] = [
      { kind: 'plane', src: { p: [-8, -5, 0], n: [0, 0, -1] }, dst: { p: apply(T0, [-8, -5, 0]), n: rotate(T0.R, [Math.sin(e), 0, Math.cos(e)]) } },
      axis(T0, [0, 0, -4], [0, 0, 1], -6),
    ];
    const ok = fitting(solveConstraints(cons));
    expect(ok.length).toBe(1);
    expect(rotations(ok[0]).length).toBe(1);
    expect(ok[0].free.length).toBe(1);
  });

  it('zwei fast parallele Stifte (0,005°): keine Lösungen, die das Teil kilometerweit verschieben', () => {
    const e = deg(0.005);
    const cons: Constraint[] = [
      axis(T0, [0, 0, 0], [0, 0, 1], 3),
      { kind: 'axis', src: { p: [25, 0, 0], d: [0, 0, 1] }, dst: { p: apply(T0, [25, 0, 0]), d: rotate(T0.R, [Math.sin(e), 0, Math.cos(e)]) } },
    ];
    const r = solveConstraints(cons);
    // Die Lage entlang der Stifte legt nur das Rauschen von 0,005° fest: sie gilt als frei und wird
    // weich zentriert (statt dorthin zu rutschen, wo die schiefen Achsen zufällig exakt passen)
    expect(r[0].distError).toBeLessThan(1e-3);
    expect(translations(r[0]).length).toBe(1);
    expect(rotations(r[0])).toEqual([]);
    // Jede angebotene Lösung muss in der Nähe der Gegenstücke bleiben (die Stifte sind 25 mm auseinander)
    for (const s of fitting(r)) {
      for (const c of cons) if (c.kind === 'axis') expect(dist(apply(s.T, c.src.p), c.dst.p)).toBeLessThan(100);
    }
    // Als passend dürfen höchstens die beiden Steckrichtungen angeboten werden
    expect(fitting(r).length).toBeLessThanOrEqual(2);
  });

  it('Welle in zwei Lagern, Achsen 0,005° windschief erkannt: bleibt mittig, Drehen und Schieben frei', () => {
    // Die Stelle, an der die beiden fast gleichen Achslinien am besten passen, liegt zufällig weit weg
    const e = deg(0.005);
    const cons: Constraint[] = [
      axis(T0, [0, 0, 0], [0, 0, 1], 2),
      {
        kind: 'axis',
        src: { p: [0, 0, 40], d: [0, 0, 1] },
        dst: { p: add(apply(T0, [0, 0, 37]), rotate(T0.R, [0, 0.001, 0])), d: rotate(T0.R, [Math.sin(e), 0, Math.cos(e)]) },
      },
    ];
    const r = solveConstraints(cons);
    for (const s of fitting(r)) {
      // Weich zentriert: Wellenmitte 0,5 mm neben der Mitte zwischen den Lagern (Versatz +2 und −3 mm)
      expect(dist(apply(s.T, [0, 0, 20]), apply(T0, [0, 0, 20]))).toBeLessThan(1);
      expect(s.distError).toBeLessThan(0.01);
      expect(rotations(s).length).toBe(1);
      expect(translations(s).length).toBe(1);
    }
    expect(fitting(r).map((s) => s.flips.join())).toEqual(['false,false', 'true,true']);
  });

  it('Gegenfläche 0,5° verkippt (kein Rauschen mehr): Drehung um die Achse ist gesperrt', () => {
    // Keine starre Drehung oder Verschiebung erhält hier den (Kompromiss-)Sitz: Drehen um die Achse
    // verkippt die Schulter gegen die Fläche, Drehen um die Winkelhalbierende hebt sie an
    const e = deg(0.5);
    const cons: Constraint[] = [
      { kind: 'plane', src: { p: [-8, -5, 0], n: [0, 0, -1] }, dst: { p: apply(T0, [-8, -5, 0]), n: rotate(T0.R, [Math.sin(e), 0, Math.cos(e)]) } },
      axis(T0, [0, 0, -4], [0, 0, 1], -6),
    ];
    const s = solveConstraints(cons)[0];
    expect(s.flips).toEqual([false, false]);
    expect(s.distError).toBeLessThan(1e-6);
    expect(s.angleError).toBeCloseTo(0.25, 2);
    expect(s.free).toEqual([]);
  });

  it('Ebene und Punkt 0,1 mm neben der Ebene: keine zweite Lösung, die nur um die freie Achse gedreht ist', () => {
    const cons = [plane(T0, [0, 0, 0], [0, 0, -1], [5, 5, 0]), point(T0, [10, 4, 0])];
    if (cons[1].kind !== 'point') throw new Error();
    cons[1] = { ...cons[1], dst: add(cons[1].dst, scale(rotate(T0.R, [0, 0, 1]), 0.1)) };
    const r = solveConstraints(cons);
    expect(r[0].distError).toBeLessThan(0.1);
    expect(rotations(r[0]).length).toBe(1);
    expect(r.length).toBe(1);
  });
});

describe('solveConstraints – Bedingungen aus Dreiecksnetzen', () => {
  it('Zapfen mit Schulter in Platte (48 Segmente, Float32): sitzt, Drehung um die Bohrungsachse frei', () => {
    const { cons, Tring } = pegIntoPlate(48, 0, 5);
    const ok = fitting(solveConstraints(cons, { scale: 300 }));
    expect(ok.length).toBe(1);
    const s = ok[0];
    expectPegSeated(s, Tring, 1e-3);
    expect(s.free.length).toBe(1);
    const rot = rotations(s)[0];
    const holeAxis = rotate(Tring.R, [0, 0, 1]);
    expect(Math.abs(dot(rot.axis, holeAxis))).toBeGreaterThan(Math.cos(deg(0.01)));
    expect(lineDist(rot.point, apply(Tring, [0, 0, 0]), holeAxis)).toBeLessThan(1e-3);
  });

  it('fein tesselliert (über 200 000 Dreiecke): Lösen bleibt schnell und genau', () => {
    const { cons, Tring, triangles } = pegIntoPlate(25000, 0, 9);
    expect(triangles).toBeGreaterThan(200000);
    const t0 = performance.now();
    const r = solveConstraints(cons, { scale: 300 });
    const ms = performance.now() - t0;
    expect(ms).toBeLessThan(100);
    const ok = fitting(r);
    expect(ok.length).toBe(1);
    expectPegSeated(ok[0], Tring, 1e-3);
    expect(rotations(ok[0]).length).toBe(1);
  });

  it('nicht verschweißte Nähte (Ecken um ±0,002 mm verrauscht): sitzt, Drehung um die Achse bleibt frei', () => {
    for (const seed of [21, 22, 23, 24]) {
      const { cons, Tring } = pegIntoPlate(48, 0.002, seed);
      const ok = fitting(solveConstraints(cons, { scale: 300 }));
      expect(ok.length).toBeGreaterThan(0);
      expectPegSeated(ok[0], Tring, 0.01);
      expect(rotations(ok[0]).length).toBe(1);
      expect(ok[0].free.length).toBe(1);
      // Kopien, die nur um die Achse gedreht sind, sind keine echten Alternativen
      expect(ok.length).toBe(1);
    }
  });
});

describe('solveConstraints – Laufzeit', () => {
  it('viele Bedingungen und alle Startwerte: unter 150 ms (gemessen etwa 20 ms)', () => {
    const worst = [
      axis(T0, [0, 0, 0], [0, 0, 1], 1, true), axis(T0, [20, 0, 0], [0, 0, 1], 2),
      axis(T0, [0, 20, 0], [0, 0, 1], -1, true), axis(T0, [20, 20, 3], [0, 0, 1], 0),
      plane(T0, [10, 10, 0], [0, 0, -1], [5, 5, 0]), plane(T0, [-5, 10, 5], [-1, 0, 0], [0, 3, 3]),
    ];
    solveConstraints(worst, { init: T0 });
    let t = performance.now();
    const r = solveConstraints(worst, { init: T0 });
    expect(performance.now() - t).toBeLessThan(150);
    expect(r[0].distError).toBeLessThan(1e-6);

    const pts: Constraint[] = [];
    for (let i = 0; i < 2000; i++) pts.push(point(T0, [Math.sin(i) * 50, Math.cos(i * 1.3) * 40, Math.sin(i * 0.7) * 30]));
    t = performance.now();
    const rp = solveConstraints(pts);
    expect(performance.now() - t).toBeLessThan(150);
    expect(probeErr(rp[0].T, T0, [[0, 0, 0], [50, 50, 50]])).toBeLessThan(1e-6);
  });
});

describe('evaluateConstraints – Randfälle', () => {
  it('nach Drehen um die freie Achse bleiben Fehler und freie Bewegungen gleich (auch bei großen Winkeln)', () => {
    const cons = [plane(T0, [0, 0, 0], [0, 0, -1], [12, -7, 0]), axis(T0, [0, 0, -4], [0, 0, 1])];
    const s = solveConstraints(cons)[0];
    for (const a of [deg(90), deg(179), deg(-135)]) {
      const T = applyMotion(s.T, s.free[0], a);
      const e = evaluateConstraints(cons, T, { flips: s.flips });
      expect(e.distError).toBeLessThan(1e-8);
      expect(e.angleError).toBeLessThan(1e-6);
      expect(e.free.length).toBe(1);
      expect(e.free[0].kind).toBe('rotate');
    }
  });
});

describe('solveConstraints – Richtungen, Umkehrungen und echte Mehrdeutigkeiten', () => {
  it('flip ausdrücklich angegeben: genau eine Lösung mit der verlangten Achsrichtung', () => {
    const base = axis(T0, [3, 1, -2], [0, 0, 1], 4);
    if (base.kind !== 'axis') throw new Error();
    for (const flip of [false, true]) {
      const r = solveConstraints([{ ...base, flip }]);
      expect(r.length).toBe(1);
      expect(r[0].flips).toEqual([flip]);
      expect(dot(rotate(r[0].T.R, base.src.d), normalize(base.dst.d))).toBeCloseTo(flip ? -1 : 1, 9);
      expect(lineDist(apply(r[0].T, base.src.p), base.dst.p, base.dst.d)).toBeLessThan(1e-9);
      expect(rotations(r[0]).length).toBe(1);
      expect(translations(r[0]).length).toBe(1);
    }
  });

  it('Achse und Punkt neben der Achse: beide Steckrichtungen exakt, jeweils eindeutig', () => {
    const cons = [axis(T0, [0, 0, 0], [0, 0, 1], 5), point(T0, [6, 2, 3])];
    const r = solveConstraints(cons);
    const exact = r.filter((s) => s.distError < 1e-8 && s.angleError < 1e-6);
    expect(exact.map((s) => s.flips[0]).sort()).toEqual([false, true]);
    for (const s of exact) {
      expect(violation(cons, s.T, s.flips)).toBeLessThan(1e-8);
      expect(s.free).toEqual([]);
    }
    expect(probeErr(exact.find((s) => !s.flips[0])!.T, T0, [[0, 0, 0], [10, 10, 10]])).toBeLessThan(1e-6);
  });

  it('Kreuzgelenk (zwei sich schneidende, senkrechte Achsen): alle vier Richtungskombinationen exakt', () => {
    const cons = [axis(T0, [0, 0, 0], [0, 0, 1], 2), axis(T0, [0, 0, 10], [1, 0, 0], -3)];
    const exact = solveConstraints(cons).filter((s) => s.distError < 1e-8 && s.angleError < 1e-6);
    expect(exact.map((s) => s.flips.map(Number).join('')).sort()).toEqual(['00', '01', '10', '11']);
    for (const s of exact) {
      expect(violation(cons, s.T, s.flips)).toBeLessThan(1e-8);
      expect(s.free).toEqual([]);
    }
  });

  it('windschiefe, senkrechte Achsen: nur beide oder keine umgekehrt sind exakt', () => {
    const cons = [axis(T0, [0, 0, 0], [0, 0, 1], 2), axis(T0, [0, 5, 10], [1, 0, 0], -3)];
    const r = solveConstraints(cons);
    const exact = r.filter((s) => s.distError < 1e-8 && s.angleError < 1e-6);
    expect(exact.map((s) => s.flips.map(Number).join('')).sort()).toEqual(['00', '11']);
    for (const s of exact) expect(s.free).toEqual([]);
    expect(probeErr(r[0].T, T0, [[0, 0, 0], [10, 10, 10]])).toBeLessThan(1e-6);
  });

  it('Schulter mit zwei symmetrischen Stiften: jeder Stift bleibt in seiner Bohrung, genau eine exakte Lösung', () => {
    // Die um 180° gedrehte Lage vertauscht die Stifte und erfüllt die gegebenen Paarungen daher nicht
    const cons = [
      plane(T0, [12.5, 0, 0], [0, 0, -1], [3, 4, 0]),
      axis(T0, [0, 0, 0], [0, 0, 1], -3),
      axis(T0, [25, 0, 0], [0, 0, 1], -3),
    ];
    const r = solveConstraints(cons);
    const exact = r.filter((s) => s.distError < 1e-8 && s.angleError < 1e-6);
    expect(exact.length).toBe(1);
    expect(r[0].flips).toEqual([false, false, false]);
    expect(r[0].free).toEqual([]);
    expect(probeErr(r[0].T, T0, [[0, 0, 0], [25, 0, 0], [0, 20, 0]])).toBeLessThan(1e-6);
  });

  it('Winkel an Wand (zwei senkrechte Ebenen): nur Verschieben entlang der Kante', () => {
    const r = rng(11);
    for (let k = 0; k < 20; k++) {
      const T = randomRigid(r, 300);
      const cons = [plane(T, [5, 5, 0], [0, 0, -1], [3, 2, 0]), plane(T, [0, 5, 5], [-1, 0, 0], [0, -4, 1])];
      const s = solveConstraints(cons)[0];
      expect(violation(cons, s.T, s.flips)).toBeLessThan(1e-8);
      expect(rotations(s)).toEqual([]);
      expect(translations(s).length).toBe(1);
      expect(Math.abs(dot(translations(s)[0].axis, rotate(T.R, [0, 1, 0])))).toBeCloseTo(1, 9);
      expectMotionsKeep(cons, s);
    }
  });

  it('Quader in Nut mit 0,2 mm Spiel: mittig, Restfehler 0,1 mm, Bewegungen in der Nut bleiben frei', () => {
    const nx = rotate(T0.R, [1, 0, 0]);
    const cons: Constraint[] = [
      { kind: 'plane', src: { p: [0, 0, 0], n: [-1, 0, 0] }, dst: { p: add(apply(T0, [0, 3, 2]), scale(nx, -0.1)), n: nx } },
      { kind: 'plane', src: { p: [10, 0, 0], n: [1, 0, 0] }, dst: { p: add(apply(T0, [10, -4, 1]), scale(nx, 0.1)), n: neg(nx) } },
    ];
    const r = solveConstraints(cons);
    expect(r.length).toBe(1);
    const s = r[0];
    expect(s.distError).toBeCloseTo(0.1, 6);
    expect(s.angleError).toBeLessThan(1e-6);
    // Mittig in der Nut
    expect(Math.abs(dot(sub(apply(s.T, [5, 0, 0]), apply(T0, [5, 0, 0])), nx))).toBeLessThan(1e-6);
    expect(translations(s).length).toBe(2);
    expect(rotations(s).length).toBe(1);
    expect(Math.abs(dot(rotations(s)[0].axis, nx))).toBeCloseTo(1, 9);
    for (const m of s.free) {
      const e = evaluateConstraints(cons, applyMotion(s.T, m, m.kind === 'rotate' ? 1 : 7));
      expect(e.distError).toBeCloseTo(0.1, 6);
      expect(e.angleError).toBeLessThan(1e-6);
    }
  });

  it('drei Punkte auf einer Linie: Drehung um diese Linie bleibt frei', () => {
    const cons = [point(T0, [0, 0, 0]), point(T0, [10, 5, 0]), point(T0, [30, 15, 0])];
    const s = solveConstraints(cons)[0];
    expect(s.free.length).toBe(1);
    const rot = rotations(s)[0];
    expect(Math.abs(dot(rot.axis, normalize(rotate(T0.R, [2, 1, 0]))))).toBeCloseTo(1, 9);
    expect(lineDist(apply(T0, [0, 0, 0]), rot.point, rot.axis)).toBeLessThan(1e-6);
    expectMotionsKeep(cons, s);
  });

  it('zwei gleiche Punkte wirken wie einer: drei orthonormale Drehachsen durch den Punkt', () => {
    const c = point(T0, [3, 4, 5]);
    if (c.kind !== 'point') throw new Error();
    const r = solveConstraints([c, c]);
    expect(r.length).toBe(1);
    const rot = rotations(r[0]);
    expect(rot.length).toBe(3);
    expect(translations(r[0])).toEqual([]);
    for (let i = 0; i < 3; i++) {
      expect(len(rot[i].axis)).toBeCloseTo(1, 9);
      expect(dist(rot[i].point, c.dst)).toBeLessThan(1e-6);
      for (let j = i + 1; j < 3; j++) expect(Math.abs(dot(rot[i].axis, rot[j].axis))).toBeLessThan(1e-9);
    }
  });

  it('ungültige Optionen werden ignoriert (scale, softWeight, init)', () => {
    const cons = [plane(T0, [0, 0, 0], [0, 0, -1], [15, -4, 0]), point(T0, [10, 4, 0]), point(T0, [-6, 12, 0])];
    const bad: ConstraintOptions[] = [
      { scale: NaN }, { scale: -5 }, { scale: 0 }, { scale: Infinity }, { softWeight: NaN }, { softWeight: -1 },
      { init: { R: [NaN, 0, 0, 0, 1, 0, 0, 0, 1], t: [0, 0, 0] } },
      { init: { R: [2, 0, 0, 0, 2, 0, 0, 0, 2], t: [1, 2, 3] } },
      // Spiegelung statt Drehung
      { init: { R: [1, 0, 0, 0, 1, 0, 0, 0, -1], t: [0, 0, 0] } },
    ];
    for (const opts of bad) {
      const r = solveConstraints(cons, opts);
      expect(r.length).toBeGreaterThan(0);
      expectRotation(r[0].T);
      expect(probeErr(r[0].T, T0, [[0, 0, 0], [10, 10, 10]])).toBeLessThan(1e-6);
      expect(r[0].free).toEqual([]);
    }
  });
});

describe('solveConstraints – D-Zapfen aus Dreiecksnetzen', () => {
  it('D-Zapfen in D-Bohrung (Schulter, Achse, Abflachung): eindeutige, exakte Lage', () => {
    const r = rng(31);
    for (let k = 0; k < 5; k++) {
      // 12 Segmente: die D-Bohrung des Hilfsnetzes wird immer so fein geteilt, Zapfen und Bohrung passen dann genau
      const pegLocal = transformPositions(dPeg(4, 2.5, 10, 12), new THREE.Matrix4().makeTranslation(8, 5, 5));
      const moving = concat(box(40, 30, 5), pegLocal);
      const plateLocal = dRing(20, 4, 2.5, 6, 12);
      const Tring = randomRigid(r, 300);
      const nearPeg = (c: Vec3) => Math.hypot(c[0] - 8, c[1] - 5) < 4.5;
      const nearHole = (c: Vec3) => Math.hypot(c[0], c[1]) < 4.5;
      const shoulder = selectTris(moving, (n, c) => n[2] > 0.99 && Math.abs(c[2] - 5) < 1e-3);
      const mantle = selectTris(moving, (n, c) => Math.abs(n[2]) < 0.1 && n[0] < 0.9 && nearPeg(c));
      const pegFlat = selectTris(moving, (n, c) => n[0] > 0.99 && nearPeg(c));
      const bottom = selectTris(plateLocal, (n) => n[2] < -0.99);
      const bore = selectTris(plateLocal, (n, c) => Math.abs(n[2]) < 0.1 && n[0] > -0.9 && nearHole(c));
      const holeFlat = selectTris(plateLocal, (n, c) => n[0] < -0.99 && nearHole(c));
      expect(pegFlat.length).toBeGreaterThan(0);
      expect(holeFlat.length).toBeGreaterThan(0);
      const toWorld = (pl: { p: Vec3; n: Vec3 }) => ({ p: apply(Tring, pl.p), n: rotate(Tring.R, pl.n) });
      const toWorldAxis = (a: { p: Vec3; d: Vec3 }) => ({ p: apply(Tring, a.p), d: rotate(Tring.R, a.d) });
      const cons: Constraint[] = [
        { kind: 'plane', src: planeOf(moving, shoulder), dst: toWorld(planeOf(plateLocal, bottom)) },
        { kind: 'axis', src: axisOf(moving, mantle), dst: toWorldAxis(axisOf(plateLocal, bore)) },
        { kind: 'plane', src: planeOf(moving, pegFlat), dst: toWorld(planeOf(plateLocal, holeFlat)) },
      ];
      const ok = fitting(solveConstraints(cons, { scale: 300 }));
      expect(ok.length).toBe(1);
      const Ttrue = compose(Tring, { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: [-8, -5, -5] });
      expect(probeErr(ok[0].T, Ttrue, [[8, 5, 5], [8, 5, 15], [20, 15, 0], [-20, -15, 0]])).toBeLessThan(1e-5);
      expect(ok[0].free).toEqual([]);
    }
  });
});

describe('evaluateConstraints – freie Verschiebungen', () => {
  it('nach Verschieben in der Ebene: weiterhin zwei Verschiebungen und eine Drehung, kein Fehler', () => {
    const c = plane(T0, [4, -2, 0], [0, 0, -1], [7, 3, 0]);
    const s = solveConstraints([c])[0];
    let T = s.T;
    for (const m of translations(s)) T = applyMotion(T, m, 25);
    T = applyMotion(T, rotations(s)[0], deg(130));
    const e = evaluateConstraints([c], T);
    expect(e.distError).toBeLessThan(1e-9);
    expect(e.angleError).toBeLessThan(1e-6);
    expect(e.free.filter((m) => m.kind === 'translate').length).toBe(2);
    expect(e.free.filter((m) => m.kind === 'rotate').length).toBe(1);
  });
});
