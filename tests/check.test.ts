import { describe, expect, it } from 'vitest';
import { MeshBVH } from 'three-mesh-bvh';
import { type Rigid, IDENTITY, type Vec3, axisAngle } from '../src/math/align';
import { type CheckTarget, checkPair, typicalGap } from '../src/math/check';
import { box, cylinder, geometryFrom, ring } from './helpers/meshes';

const at = (z: number, R = IDENTITY.R): Rigid => ({ R, t: [0, 0, z] });
function target(positions: Float32Array, matrix: Rigid = IDENTITY): CheckTarget {
  const g = geometryFrom(positions);
  const bb = g.boundingBox!;
  const min: Vec3 = [bb.min.x + matrix.t[0], bb.min.y + matrix.t[1], bb.min.z + matrix.t[2]];
  const max: Vec3 = [bb.max.x + matrix.t[0], bb.max.y + matrix.t[1], bb.max.z + matrix.t[2]];
  return { bvh: new MeshBVH(g), matrix, box: { min, max } };
}
const opts = { contactTol: 0.05, gapTol: 0.6 };
/** Anzahl eingefärbter Dreiecke */
const n = (a: Float32Array) => a.length / 9;

describe('checkPair', () => {
  const base = box(20, 20, 10);
  const lid = box(20, 20, 5);
  const fixed = target(base);

  it('aufliegender Deckel: Berührung, keine Durchdringung', () => {
    const r = checkPair(lid, at(10), fixed, opts);
    expect(r.penetrating).toHaveLength(0);
    expect(n(r.touching)).toBeGreaterThanOrEqual(2);
    expect(r.gap).toHaveLength(0);
  });

  it('eingesunkener Deckel: Durchdringung mit richtiger Tiefe', () => {
    const r = checkPair(lid, at(9.7), fixed, opts);
    expect(n(r.penetrating)).toBeGreaterThanOrEqual(2);
    expect(r.maxPen).toBeCloseTo(0.3, 2);
  });

  it('schwebender Deckel: Spalt mit richtigem Maß', () => {
    const r = checkPair(lid, at(10.2), fixed, opts);
    expect(r.penetrating).toHaveLength(0);
    expect(n(r.gap)).toBeGreaterThanOrEqual(2);
    expect(r.minGap).toBeCloseTo(0.2, 3);
    expect(r.maxGap).toBeCloseTo(0.2, 3);
  });

  it('unterteilt große Dreiecke: eingefärbt wird nur die echte Kontaktfläche', () => {
    // Kleiner Klotz 4 × 4 auf großer Platte 40 × 40 (aus nur 2 Dreiecken je Seite)
    const area = (a: Float32Array) => {
      let s = 0;
      for (let i = 0; i < a.length; i += 9) {
        const u = [a[i + 3] - a[i], a[i + 4] - a[i + 1], a[i + 5] - a[i + 2]];
        const v = [a[i + 6] - a[i], a[i + 7] - a[i + 1], a[i + 8] - a[i + 2]];
        s += Math.hypot(u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]) / 2;
      }
      return s;
    };
    const r = checkPair(box(40, 40, 2), at(-2), target(box(4, 4, 5), at(0)), { ...opts, maxEdge: 1 });
    // Oberseite der Platte: berührt nur unter dem Klotz (16 mm²), nicht die ganze Platte (1600 mm²)
    expect(area(r.touching)).toBeGreaterThan(8);
    expect(area(r.touching)).toBeLessThan(40);
  });

  it('weit entfernte Teile werden übersprungen', () => {
    const r = checkPair(lid, { R: IDENTITY.R, t: [100, 0, 0] }, fixed, opts);
    expect(n(r.penetrating) + n(r.touching) + n(r.gap)).toBe(0);
  });

  it('Zapfen mit Spiel in der Bohrung: Spalt = Spiel, keine Durchdringung', () => {
    const hole = target(ring(12, 5.15, 8, 96));
    const r = checkPair(cylinder(5, 8, 96), at(0), hole, opts);
    expect(r.penetrating).toHaveLength(0);
    expect(n(r.gap)).toBeGreaterThan(50);
    expect(r.maxGap).toBeLessThan(0.16);
    expect(r.minGap).toBeGreaterThan(0.14);
    expect(typicalGap([r])).toBeCloseTo(0.15, 2);
  });

  it('zu dicker Zapfen: Durchdringung = Übermaß', () => {
    const hole = target(ring(12, 4.8, 8, 96));
    const r = checkPair(cylinder(5, 8, 96), at(0, axisAngle([0, 0, 1], 0.3)), hole, opts);
    expect(n(r.penetrating)).toBeGreaterThan(50);
    expect(r.maxPen).toBeGreaterThan(0.15);
    expect(r.maxPen).toBeLessThan(0.21);
  });
});
