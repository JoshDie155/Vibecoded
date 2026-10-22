import { describe, expect, it } from 'vitest';
import {
  type Rigid, type Vec3, add, apply, axisAngle, compose, dist, dot, len, rotate, rotationAbout, scale, sub,
} from '../src/math/align';
import {
  type Constraint, type ConstraintSolution, evaluateConstraints, solveConstraints,
} from '../src/math/constraints';
import { type FreeMotion, applyMotion } from '../src/math/motion';

const deg = (d: number) => (d * Math.PI) / 180;
const neg = (a: Vec3): Vec3 => [-a[0], -a[1], -a[2]];

/** Wahre Lage des beweglichen Teils (lokal → Welt). */
const T0: Rigid = { R: axisAngle([0.3, -1, 0.5], 2.0), t: [50, 20, -10] };
const T170: Rigid = { R: axisAngle([1, 1, 0.2], deg(170)), t: [-30, 80, 15] };

/** Punkt auf Punkt. */
const point = (T: Rigid, p: Vec3): Constraint => ({ kind: 'point', src: p, dst: apply(T, p) });
/** Fläche des beweglichen Teils (Normale n) auf Gegenfläche; dst.p um `off` in der Ebene versetzt. */
const plane = (T: Rigid, p: Vec3, n: Vec3, off: Vec3 = [0, 0, 0]): Constraint => ({
  kind: 'plane',
  src: { p, n },
  dst: { p: apply(T, add(p, off)), n: neg(rotate(T.R, n)) },
});
/** Achse; dst.p um `along` entlang der Achse versetzt, dst.d bei reverse umgekehrt. */
const axis = (T: Rigid, p: Vec3, d: Vec3, along = 0, reverse = false, flip?: boolean): Constraint => ({
  kind: 'axis',
  src: { p, d },
  dst: { p: apply(T, add(p, scale(d, along))), d: scale(rotate(T.R, d), reverse ? -1 : 1) },
  ...(flip === undefined ? {} : { flip }),
});

/** Abstand eines Punkts von der Geraden durch p mit Richtung d. */
function lineDist(q: Vec3, p: Vec3, d: Vec3): number {
  const n = scale(d, 1 / len(d));
  const v = sub(q, p);
  return len(sub(v, scale(n, dot(v, n))));
}

/** Größte Verletzung der harten Bedingungen (mm bzw. Längeneinheiten). */
function violation(cons: Constraint[], T: Rigid, flips: boolean[]): number {
  let m = 0;
  cons.forEach((c, i) => {
    if (c.kind === 'point') m = Math.max(m, dist(apply(T, c.src), c.dst));
    else if (c.kind === 'plane') {
      m = Math.max(m, len(add(rotate(T.R, c.src.n), c.dst.n)));
      m = Math.max(m, Math.abs(dot(sub(apply(T, c.src.p), c.dst.p), c.dst.n)));
    } else {
      m = Math.max(m, len(sub(rotate(T.R, c.src.d), scale(c.dst.d, flips[i] ? -1 : 1))));
      m = Math.max(m, lineDist(apply(T, c.src.p), c.dst.p, c.dst.d));
    }
  });
  return m;
}

function expectRigid(T: Rigid, ref: Rigid, tol = 1e-6) {
  for (let i = 0; i < 9; i++) expect(Math.abs(T.R[i] - ref.R[i])).toBeLessThan(tol);
  for (let i = 0; i < 3; i++) expect(Math.abs(T.t[i] - ref.t[i])).toBeLessThan(tol * 100);
}

/** Jede freie Bewegung muss die Bedingungen erfüllt lassen. */
function expectMotionsKeep(cons: Constraint[], sol: ConstraintSolution) {
  for (const m of sol.free)
    for (const amount of [0.7, -2.3, 12]) {
      const T = applyMotion(sol.T, m, m.kind === 'rotate' ? amount / 4 : amount);
      expect(violation(cons, T, sol.flips)).toBeLessThan(1e-6);
    }
}

const rotations = (s: ConstraintSolution) => s.free.filter((m): m is Extract<FreeMotion, { kind: 'rotate' }> => m.kind === 'rotate');
const translations = (s: ConstraintSolution) => s.free.filter((m) => m.kind === 'translate');

describe('solveConstraints', () => {
  it('drei Punkte legen die Lage eindeutig fest', () => {
    const cons = [point(T0, [0, 0, 0]), point(T0, [30, 0, 0]), point(T0, [5, 20, 8])];
    const r = solveConstraints(cons);
    expect(r.length).toBe(1);
    expectRigid(r[0].T, T0);
    expect(r[0].distError).toBeLessThan(1e-9);
    expect(r[0].angleError).toBe(0);
    expect(r[0].free).toEqual([]);
    expect(r[0].flips).toEqual([false, false, false]);
  });

  it('eine Ebene: Flächen liegen zentriert aufeinander, zwei Verschiebungen und eine Drehung frei', () => {
    const c = plane(T0, [4, -2, 0], [0, 0, -1], [7, 3, 0]);
    const r = solveConstraints([c]);
    expect(r.length).toBe(1);
    const s = r[0];
    if (c.kind !== 'plane') throw new Error();
    const n = rotate(s.T.R, c.src.n);
    expect(len(add(n, c.dst.n))).toBeLessThan(1e-9);
    const p = apply(s.T, c.src.p);
    expect(Math.abs(dot(sub(p, c.dst.p), c.dst.n))).toBeLessThan(1e-9);
    // Weiche Zentrierung: Flächenmitte auf die Mitte der Gegenfläche
    expect(dist(p, c.dst.p)).toBeLessThan(1e-6);

    const tr = translations(s);
    const rot = rotations(s);
    expect(tr.length).toBe(2);
    expect(rot.length).toBe(1);
    for (const t of tr) expect(Math.abs(dot(t.axis, c.dst.n))).toBeLessThan(1e-9);
    expect(Math.abs(dot(tr[0].axis, tr[1].axis))).toBeLessThan(1e-9);
    // Drehung um die Normale der Gegenfläche durch deren Mitte
    expect(dot(rot[0].axis, c.dst.n)).toBeCloseTo(1, 9);
    expect(lineDist(c.dst.p, rot[0].point, rot[0].axis)).toBeLessThan(1e-6);
    expectMotionsKeep([c], s);
  });

  it('eine Achse: Achslinien fallen zusammen, beide Richtungen als getrennte Lösungen', () => {
    const c = axis(T0, [3, 1, -2], [0, 0, 1], 9);
    if (c.kind !== 'axis') throw new Error();
    const r = solveConstraints([c]);
    expect(r.length).toBe(2);
    expect(r.map((s) => s.flips[0]).sort()).toEqual([false, true]);
    expect(r[0].flips).toEqual([false]);
    for (const s of r) {
      expect(violation([c], s.T, s.flips)).toBeLessThan(1e-9);
      expect(s.distError).toBeLessThan(1e-9);
      expect(s.angleError).toBeLessThan(1e-6);
      // Weiche Zentrierung entlang der Achse
      expect(dist(apply(s.T, c.src.p), c.dst.p)).toBeLessThan(1e-6);
      const tr = translations(s);
      const rot = rotations(s);
      expect(tr.length).toBe(1);
      expect(rot.length).toBe(1);
      expect(dot(tr[0].axis, c.dst.d)).toBeCloseTo(1, 9);
      expect(dot(rot[0].axis, c.dst.d)).toBeCloseTo(1, 9);
      expect(lineDist(rot[0].point, c.dst.p, c.dst.d)).toBeLessThan(1e-6);
      expectMotionsKeep([c], s);
    }
  });

  it('Zapfen mit Schulter in Bohrung: sitzt exakt, nur die Drehung um die Achse bleibt frei', () => {
    // Bewegliches Teil: Schulterfläche bei z = 0 (Normale −Z), Zapfen nach unten entlang Z.
    // Festes Teil: Plattenmitte liegt neben der Bohrung, Achspunkt tiefer in der Bohrung.
    const cons = [plane(T0, [0, 0, 0], [0, 0, -1], [12, -7, 0]), axis(T0, [0, 0, -4], [0, 0, 1], -6)];
    const r = solveConstraints(cons);
    const s = r[0];
    expect(s.flips).toEqual([false, false]);
    expect(s.distError).toBeLessThan(1e-7);
    expect(s.angleError).toBeLessThan(1e-6);
    for (const p of [[0, 0, 0], [0, 0, -10]] as Vec3[]) expect(dist(apply(s.T, p), apply(T0, p))).toBeLessThan(1e-6);
    expect(s.free.length).toBe(1);
    const rot = rotations(s)[0];
    expect(Math.abs(dot(rot.axis, rotate(T0.R, [0, 0, 1])))).toBeCloseTo(1, 9);
    expect(lineDist(apply(T0, [0, 0, 0]), rot.point, rot.axis)).toBeLessThan(1e-6);
    expectMotionsKeep(cons, s);
    // Die umgekehrte Achsrichtung passt nicht zur Schulter: nur als eine schlechtere Alternative
    expect(r.length).toBe(2);
    expect(r[1].flips).toEqual([false, true]);
    expect(r[1].angleError).toBeGreaterThan(45);
  });

  it('Ebene und ein Punkt: Drehung um die Normale durch den Punkt bleibt frei', () => {
    const cons = [plane(T0, [0, 0, 0], [0, 0, -1], [5, 5, 0]), point(T0, [10, 4, 0])];
    const r = solveConstraints(cons);
    expect(r.length).toBe(1);
    const s = r[0];
    expect(violation(cons, s.T, s.flips)).toBeLessThan(1e-9);
    expect(s.free.length).toBe(1);
    const rot = rotations(s)[0];
    if (cons[0].kind !== 'plane' || cons[1].kind !== 'point') throw new Error();
    expect(dot(rot.axis, cons[0].dst.n)).toBeCloseTo(1, 9);
    expect(lineDist(cons[1].dst, rot.point, rot.axis)).toBeLessThan(1e-6);
    expectMotionsKeep(cons, s);
  });

  it('Ebene und zwei Punkte: eindeutig und exakt', () => {
    const cons = [plane(T0, [0, 0, 0], [0, 0, -1], [15, -4, 0]), point(T0, [10, 4, 0]), point(T0, [-6, 12, 0])];
    const r = solveConstraints(cons);
    expect(r.length).toBe(1);
    expectRigid(r[0].T, T0);
    expect(r[0].free).toEqual([]);
    expect(r[0].distError).toBeLessThan(1e-8);
  });

  it('einzelner Punkt: drei Drehungen um den Punkt frei', () => {
    const c = point(T0, [3, 4, 5]);
    const r = solveConstraints([c]);
    expect(r.length).toBe(1);
    if (c.kind !== 'point') throw new Error();
    const rot = rotations(r[0]);
    expect(rot.length).toBe(3);
    expect(translations(r[0]).length).toBe(0);
    for (const m of rot) expect(dist(m.point, c.dst)).toBeLessThan(1e-6);
    expectMotionsKeep([c], r[0]);
  });

  it('zwei parallele Achsen (zwei Stifte): nur die Verschiebung entlang der Achsen ist frei', () => {
    const cons = [axis(T0, [0, 0, 0], [0, 0, 1], 3), axis(T0, [25, 0, 0], [0, 0, 1], -2)];
    const r = solveConstraints(cons);
    const s = r[0];
    expect(s.distError).toBeLessThan(1e-8);
    expect(s.flips).toEqual([false, false]);
    expect(s.free.length).toBe(1);
    expect(s.free[0].kind).toBe('translate');
    expectMotionsKeep(cons, s);
    // Um 180° gedreht eingesteckt ist ebenfalls exakt
    expect(r[1].flips).toEqual([true, true]);
    expect(r[1].distError).toBeLessThan(1e-8);
    // Nur eine Achse umgedreht passt nicht: je Kombination genau eine schlechtere Alternative
    expect(r.length).toBe(4);
    expect(r.slice(2).map((o) => o.flips.join()).sort()).toEqual(['false,true', 'true,false']);
    for (const o of r.slice(2)) expect(o.angleError).toBeGreaterThan(45);
  });

  it('D-Zapfen (Schulter, Achse, Abflachung) liegt eindeutig fest', () => {
    const cons = [
      plane(T0, [0, 0, 0], [0, 0, -1], [10, 10, 0]),
      axis(T0, [0, 0, -5], [0, 0, 1], 0, false, false),
      plane(T0, [2.5, 0, -5], [1, 0, 0], [0, 1, -1]),
    ];
    const r = solveConstraints(cons);
    expect(r.length).toBe(1);
    expectRigid(r[0].T, T0);
    expect(r[0].free).toEqual([]);
  });

  it('widersprüchliche Bedingungen: kein Absturz, Restfehler wird gemeldet', () => {
    const cons = [point(T0, [0, 0, 0]), point(T0, [30, 0, 0]), point(T0, [5, 20, 8])];
    if (cons[1].kind !== 'point') throw new Error();
    cons[1] = { ...cons[1], dst: add(cons[1].dst, [2, -1, 1]) };
    const r = solveConstraints(cons);
    expect(r.length).toBeGreaterThan(0);
    expect(r[0].distError).toBeGreaterThan(0.1);
    expect(r[0].distError).toBeLessThan(3);

    // Achse parallel zur Fläche, im festen Teil aber um 30° gegen die Fläche geneigt
    const bad: Constraint[] = [
      plane(T0, [0, 0, 0], [0, 0, -1]),
      { kind: 'axis', src: { p: [0, 0, -4], d: [1, 0, 0] }, dst: { p: apply(T0, [0, 0, -4]), d: rotate(T0.R, [Math.cos(deg(30)), 0, Math.sin(deg(30))]) }, flip: false },
    ];
    const r2 = solveConstraints(bad);
    expect(r2.length).toBeGreaterThan(0);
    expect(r2[0].angleError).toBeGreaterThan(5);
    expect(r2[0].angleError).toBeLessThan(30);
  });

  it('große Drehungen (170°) werden über mehrere Startwerte exakt gefunden', () => {
    const a = [plane(T170, [0, 0, 0], [0, 0, -1], [15, -4, 0]), point(T170, [10, 4, 0]), point(T170, [-6, 12, 0])];
    expectRigid(solveConstraints(a)[0].T, T170);

    const b = [
      plane(T170, [0, 0, 0], [0, 0, -1], [-20, 8, 0]),
      axis(T170, [0, 0, -4], [0, 0, 1], -6),
      point(T170, [8, 3, 0]),
    ];
    const rb = solveConstraints(b);
    expectRigid(rb[0].T, T170);
    expect(rb[0].free).toEqual([]);

    const c = [point(T170, [0, 0, 0]), point(T170, [30, 0, 0]), point(T170, [5, 20, 8])];
    expectRigid(solveConstraints(c)[0].T, T170);

    // Gleiche Bedingungen mit einer weit entfernten Startlage
    const rc = solveConstraints(b, { init: { R: axisAngle([0, 0, 1], deg(-120)), t: [500, 0, 0] } });
    expectRigid(rc[0].T, T170);
  });

  it('Startlage: freie Drehung um die Achse bleibt erhalten', () => {
    const cons = [plane(T0, [0, 0, 0], [0, 0, -1], [12, -7, 0]), axis(T0, [0, 0, -4], [0, 0, 1], 0)];
    // Startlage verschoben und um eine Querachse verkippt, um die Zapfenachse aber genau wie T0
    const tilt = rotationAbout(apply(T0, [0, 0, 0]), rotate(T0.R, [1, 0, 0]), deg(4));
    const init = compose({ R: tilt.R, t: add(tilt.t, [1, -2, 0.5]) }, T0);
    const r = solveConstraints(cons, { init });
    expect(r[0].distError).toBeLessThan(1e-7);
    for (const probe of [[20, 0, 0], [0, 20, 0]] as Vec3[]) expect(dist(apply(r[0].T, probe), apply(T0, probe))).toBeLessThan(0.05);
  });

  it('weiche Zentrierung verfälscht harte Bedingungen nicht', () => {
    // Plattenmitte weit neben der Bohrung: Zentrierung zieht seitlich, darf aber nichts verschieben
    const cons = [plane(T0, [0, 0, 0], [0, 0, -1], [80, 60, 0]), axis(T0, [0, 0, -4], [0, 0, 1], -30)];
    const s = solveConstraints(cons, { softWeight: 0.1 })[0];
    expect(violation(cons, s.T, s.flips)).toBeLessThan(1e-9);
  });

  it('entartete Eingaben werfen nicht', () => {
    expect(solveConstraints([])).toEqual([]);
    const zeroN: Constraint = { kind: 'plane', src: { p: [0, 0, 0], n: [0, 0, 0] }, dst: { p: [1, 2, 3], n: [0, 0, 1] } };
    const zeroD: Constraint = { kind: 'axis', src: { p: [0, 0, 0], d: [0, 0, 0] }, dst: { p: [1, 2, 3], d: [0, 0, 0] } };
    const nan: Constraint = { kind: 'point', src: [NaN, 0, 0], dst: [0, Infinity, 0] };
    const dupPlane = plane(T0, [0, 0, 0], [0, 0, -1]);
    const cases: Constraint[][] = [
      [zeroN],
      [zeroD],
      [nan],
      [dupPlane, dupPlane, dupPlane],
      [zeroN, zeroD, nan],
      [{ ...point(T0, [1, 1, 1]), weight: 0 }],
      [{ ...point(T0, [1, 1, 1]), weight: -3 }, { ...point(T0, [9, 1, 1]), weight: NaN }],
      [point(T0, [0, 0, 0]), point(T0, [0, 0, 0])],
    ];
    for (const cons of cases) {
      const r = solveConstraints(cons);
      expect(r.length).toBeGreaterThan(0);
      for (const s of r) {
        expect([...s.T.R, ...s.T.t].every(Number.isFinite)).toBe(true);
        expect(Number.isFinite(s.distError) && Number.isFinite(s.angleError) && Number.isFinite(s.cost)).toBe(true);
        expect(s.flips.length).toBe(cons.length);
      }
    }
    // Doppelte parallele Ebenen verhalten sich wie eine
    const r = solveConstraints([dupPlane, dupPlane]);
    expect(r.length).toBe(1);
    expect(translations(r[0]).length).toBe(2);
    expect(rotations(r[0]).length).toBe(1);
  });
});

describe('evaluateConstraints', () => {
  it('meldet nach einer Gizmo-Bewegung weiterhin dieselben freien Bewegungen', () => {
    const cons = [plane(T0, [0, 0, 0], [0, 0, -1], [12, -7, 0]), axis(T0, [0, 0, -4], [0, 0, 1])];
    const s = solveConstraints(cons)[0];
    const T = applyMotion(s.T, s.free[0], 1.3);
    const e = evaluateConstraints(cons, T);
    expect(e.distError).toBeLessThan(1e-7);
    expect(e.free.length).toBe(1);
    expect(e.free[0].kind).toBe('rotate');
    const moved = evaluateConstraints(cons, { R: T.R, t: add(T.t, [0, 0, 1]) });
    expect(moved.distError).toBeGreaterThan(0.1);
  });
});

