import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { buildTopology } from '../src/math/features';
import { type MFeature, centerOf, featureAt, measureBetween, measureModel, planeFeatures, pointOn, splitLoop, toWorld } from '../src/math/measure';
import type { Vec3 } from '../src/math/align';
import { box, concat, cylinder, dPeg, ring, toPositions, transformPositions } from './helpers/meshes';

/** Dreieck, das den Punkt p (auf der Oberfläche) enthält. */
function triAt(positions: Float32Array, p: Vec3): number {
  let best = -1;
  let bestD = Infinity;
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3(), q = new THREE.Vector3();
  const tri = new THREE.Triangle();
  const P = new THREE.Vector3(...p);
  for (let t = 0; t < positions.length / 9; t++) {
    a.fromArray(positions, 9 * t); b.fromArray(positions, 9 * t + 3); c.fromArray(positions, 9 * t + 6);
    tri.set(a, b, c);
    if (tri.getArea() < 1e-12) continue;
    tri.closestPointToPoint(P, q);
    const d = q.distanceTo(P);
    if (d < bestD) { bestD = d; best = t; }
  }
  return best;
}

function setup(positions: Float32Array) {
  const m = measureModel(buildTopology(positions));
  const at = (p: Vec3, tol = 0.3): MFeature => featureAt(m, triAt(positions, p), p, tol);
  return { m, at };
}

const close = (a: Vec3, b: Vec3, eps = 1e-3) => a.every((x, i) => Math.abs(x - b[i]) < eps);
const edgeLen = (f: MFeature) => (f.kind === 'edge' ? Math.hypot(f.b[0] - f.a[0], f.b[1] - f.a[1], f.b[2] - f.a[2]) : NaN);

describe('Messen: Merkmale wie in Bambu Studio', () => {
  it('Quader: Fläche, Kante (Länge), Ecke', () => {
    const { at } = setup(box(20, 10, 5));
    const top = at([2, 1, 5]);
    expect(top.kind).toBe('plane');
    expect(top.kind === 'plane' && top.area).toBeCloseTo(200, 6);
    // Nahe der langen Kante vorne oben (y = −5): die ganze Kante, nicht nur ein Dreieck
    const e = at([3, -4.85, 5]);
    expect(e.kind).toBe('edge');
    expect(edgeLen(e)).toBeCloseTo(20, 6);
    expect(edgeLen(at([9.85, 1, 5]))).toBeCloseTo(10, 6);
    // In der Ecke: Eckpunkt
    const v = at([9.85, -4.85, 5]);
    expect(v).toEqual({ kind: 'point', p: [10, -5, 5], on: 'vertex' });
    // Seitenfläche: senkrechte Kante 5 mm
    expect(edgeLen(at([9.85, -5, 2.5]))).toBeCloseTo(5, 6);
  });

  it('Zylinder: Rand des Deckels ist ein Kreis (Durchmesser), der Mantel ein Zylinder', () => {
    const { at } = setup(cylinder(5, 8, 48));
    const c = at([4.9, 0, 8]);
    expect(c.kind).toBe('circle');
    if (c.kind !== 'circle') return;
    expect(c.radius).toBeCloseTo(5, 3);
    expect(c.arc).toBeCloseTo(2 * Math.PI, 6);
    expect(close(c.center, [0, 0, 8])).toBe(true);
    expect(at([0, 0, 8]).kind).toBe('plane');
    const side = at([5 * Math.cos(0.07), 5 * Math.sin(0.07), 4]);
    expect(side.kind).toBe('cylinder');
    expect(side.kind === 'cylinder' && side.radius).toBeCloseTo(5, 2);
    expect(side.kind === 'cylinder' && side.hole).toBe(false);
    // Vom Mantel aus knapp unter dem Rand: der Kreis des Deckels
    const rim = at([5 * Math.cos(0.07), 5 * Math.sin(0.07), 7.8]);
    expect(rim.kind === 'circle' && rim.radius).toBeCloseTo(5, 3);
    expect(rim.kind === 'circle' && rim.center[2]).toBeCloseTo(8, 6);
    expect(at([5 * Math.cos(0.07), 5 * Math.sin(0.07), 0.15]).kind).toBe('circle');
    expect(centerOf(c)).toEqual({ kind: 'point', p: c.center, on: 'center' });
  });

  it('Ring: Bohrung und Außenrand der Deckfläche sind zwei Kreise', () => {
    const { at } = setup(ring(10, 3, 4, 64));
    const inner = at([3.1, 0, 4]);
    const outer = at([9.9, 0, 4]);
    expect(inner.kind === 'circle' && inner.radius).toBeCloseTo(3, 3);
    expect(outer.kind === 'circle' && outer.radius).toBeCloseTo(10, 3);
    // Wand der Bohrung: Zylinder als Bohrung
    const wall = at([3 * Math.cos(0.05), 3 * Math.sin(0.05), 2]);
    expect(wall.kind === 'cylinder' && wall.hole).toBe(true);
  });

  it('Sechskant: Kanten mit gemeinsamem Mittelpunkt', () => {
    // Sechseck als Linienzug (ein Kreis mit 6 Segmenten bekäme hier 12 Punkte)
    const hex = new THREE.Shape();
    for (let k = 0; k < 6; k++) hex[k ? 'lineTo' : 'moveTo'](6 * Math.cos((k * Math.PI) / 3), 6 * Math.sin((k * Math.PI) / 3));
    const { at } = setup(toPositions(new THREE.ExtrudeGeometry(hex, { depth: 3, bevelEnabled: false })));
    // Mitte der Seite von (6, 0) nach (3, 5,2), etwas nach innen
    const e = at([4.5 - 0.0866, 2.598 - 0.05, 3]);
    expect(e.kind).toBe('edge');
    if (e.kind !== 'edge') return;
    expect(e.sides).toBe(6);
    expect(edgeLen(e)).toBeCloseTo(6, 3);
    expect(close(e.center!, [0, 0, 3])).toBe(true);
  });

  it('D-Profil: Bogen (Durchmesser) und gerade Kante getrennt', () => {
    const { m, at } = setup(dPeg(5, 3, 4, 48));
    const arc = at([-4.95, 0, 4]);
    expect(arc.kind).toBe('circle');
    if (arc.kind !== 'circle') return;
    expect(arc.radius).toBeCloseTo(5, 2);
    // Bogen über 360° − 2·acos(0,6) ≈ 254°
    expect(arc.arc / Math.PI * 180).toBeGreaterThan(240);
    expect(arc.arc / Math.PI * 180).toBeLessThan(260);
    const flat = at([2.95, 0, 4]);
    expect(flat.kind).toBe('edge');
    expect(edgeLen(flat)).toBeCloseTo(8, 1);
    const top = at([0, 0, 4]);
    expect(top.kind).toBe('plane');
    const feats = planeFeatures(m, top.kind === 'plane' ? top.region : -1);
    expect(feats.filter((f) => f.kind === 'circle')).toHaveLength(1);
    expect(feats.filter((f) => f.kind === 'edge')).toHaveLength(1);
  });

  it('gerundetes Rechteck: vier Bögen, vier gerade Kanten', () => {
    const s = new THREE.Shape();
    const w = 30, h = 16, r = 4;
    s.moveTo(-w / 2 + r, -h / 2);
    s.lineTo(w / 2 - r, -h / 2);
    s.absarc(w / 2 - r, -h / 2 + r, r, -Math.PI / 2, 0, false);
    s.lineTo(w / 2, h / 2 - r);
    s.absarc(w / 2 - r, h / 2 - r, r, 0, Math.PI / 2, false);
    s.lineTo(-w / 2 + r, h / 2);
    s.absarc(-w / 2 + r, h / 2 - r, r, Math.PI / 2, Math.PI, false);
    s.lineTo(-w / 2, -h / 2 + r);
    s.absarc(-w / 2 + r, -h / 2 + r, r, Math.PI, 1.5 * Math.PI, false);
    const pos = toPositions(new THREE.ExtrudeGeometry(s, { depth: 3, bevelEnabled: false, curveSegments: 12 }));
    const { m, at } = setup(pos);
    const top = at([0, 0, 3]);
    const feats = planeFeatures(m, top.kind === 'plane' ? top.region : -1);
    expect(feats.filter((f) => f.kind === 'circle')).toHaveLength(4);
    expect(feats.filter((f) => f.kind === 'edge')).toHaveLength(4);
    const corner = at([w / 2 - r + r * Math.cos(0.8) - 0.05, h / 2 - r + r * Math.sin(0.8) - 0.05, 3]);
    expect(corner.kind === 'circle' && corner.radius).toBeCloseTo(4, 2);
    expect(corner.kind === 'circle' && corner.arc / Math.PI * 180).toBeCloseTo(90, 0);
    expect(edgeLen(at([0, -h / 2 + 0.05, 3]))).toBeCloseTo(w - 2 * r, 3);
    expect(edgeLen(at([w / 2 - 0.05, 0, 3]))).toBeCloseTo(h - 2 * r, 3);
  });

  it('splitLoop: Quadrat bleibt vier Kanten, zusätzliche Punkte auf einer Seite werden zusammengefasst', () => {
    const sq: Vec3[] = [[0, 0, 0], [5, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0]];
    const f = splitLoop(sq, [0, 0, 1]);
    expect(f.map((x) => x.kind)).toEqual(['edge', 'edge', 'edge', 'edge']);
    expect(f.map(edgeLen).sort()).toEqual([10, 10, 10, 10]);
  });

  it('Punkt auf Merkmal und Lage im Teil', () => {
    const { at } = setup(box(20, 10, 5));
    const e = at([3, -4.85, 5]);
    expect(pointOn(e, [3, -4.85, 5])).toEqual({ kind: 'point', p: [3, -5, 5], on: 'edge' });
    const M = new THREE.Matrix4().makeRotationZ(Math.PI / 2).setPosition(100, 0, 0);
    const w = toWorld(e, M.elements);
    expect(w.kind === 'edge' && close(w.a, [105, -10, 5], 1e-6) && close(w.b, [105, 10, 5], 1e-6)).toBe(true);
  });
});

describe('Messen: Maße zwischen zwei Merkmalen', () => {
  const plane = (o: Vec3, n: Vec3): MFeature => ({ kind: 'plane', origin: o, normal: n, region: 0, area: 1 });
  const edge = (a: Vec3, b: Vec3): MFeature => ({ kind: 'edge', a, b });
  const pt = (p: Vec3): MFeature => ({ kind: 'point', p, on: 'vertex' });
  const circle = (c: Vec3, n: Vec3, r: number): MFeature => ({ kind: 'circle', center: c, normal: n, radius: r, arc: 2 * Math.PI, start: [1, 0, 0] });

  it('zwei parallele Flächen: Abstand; schräge: Winkel und Innenwinkel', () => {
    expect(measureBetween(plane([0, 0, 5], [0, 0, 1]), plane([3, 2, 0], [0, 0, -1])).perpendicular?.d).toBeCloseTo(5, 9);
    const r = measureBetween(plane([0, 0, 5], [0, 0, 1]), plane([0, 0, 0], [1, 0, 0]));
    expect(r.angle?.deg).toBeCloseTo(90, 9);
    const c = measureBetween(plane([0, 0, 0], [0, 0, 1]), plane([0, 0, 0], [Math.SQRT1_2, 0, Math.SQRT1_2]));
    expect(c.angle).toEqual({ deg: expect.closeTo(45, 9), inner: expect.closeTo(135, 9) });
  });

  it('Punkt–Punkt, Punkt–Kante, Punkt–Fläche', () => {
    expect(measureBetween(pt([0, 0, 0]), pt([3, 4, 0])).direct?.d).toBe(5);
    const pe = measureBetween(pt([5, 3, 0]), edge([0, 0, 0], [2, 0, 0]));
    expect(pe.perpendicular?.d).toBeCloseTo(3, 9);
    expect(pe.direct?.d).toBeCloseTo(Math.hypot(3, 3), 9);
    expect(pe.direct?.from).toEqual([5, 3, 0]);
    expect(measureBetween(pt([1, 1, 7]), plane([0, 0, 2], [0, 0, 1])).perpendicular?.d).toBeCloseTo(5, 9);
  });

  it('Kanten: parallel → Abstand, sonst Winkel; kürzeste Verbindung', () => {
    const p = measureBetween(edge([0, 0, 0], [10, 0, 0]), edge([0, 4, 3], [10, 4, 3]));
    expect(p.perpendicular?.d).toBeCloseTo(5, 9);
    expect(p.angle).toBeUndefined();
    // Maßlinie mitten im gemeinsamen Stück
    const o = measureBetween(edge([10, 0, 0], [0, 0, 0]), edge([4, 2, 0], [20, 2, 0]));
    expect(o.perpendicular?.from[0]).toBeCloseTo(7, 9);
    expect(o.perpendicular?.to).toEqual([expect.closeTo(7, 9), expect.closeTo(2, 9), expect.closeTo(0, 9)]);
    const s = measureBetween(edge([0, 0, 0], [10, 0, 0]), edge([12, 0, 0], [12, 5, 0]));
    expect(s.angle?.deg).toBeCloseTo(90, 9);
    expect(s.direct?.d).toBeCloseTo(2, 9);
  });

  it('Kreise: Mittelpunktabstand, Achsabstand, Mitte zur Fläche', () => {
    const r = measureBetween(circle([0, 0, 0], [0, 0, 1], 2), circle([30, 40, 0], [0, 0, 1], 3));
    expect(r.direct?.d).toBeCloseTo(50, 9);
    const h = measureBetween(circle([0, 0, 10], [0, 0, 1], 2), circle([3, 4, 0], [0, 0, -1], 3));
    expect(h.perpendicular).toMatchObject({ d: expect.closeTo(5, 9), axis: true });
    expect(h.direct?.d).toBeCloseTo(Math.hypot(5, 10), 9);
    expect(measureBetween(circle([0, 0, 10], [0, 0, 1], 2), plane([0, 0, 0], [0, 0, -1])).perpendicular?.d).toBeCloseTo(10, 9);
    // Kante parallel zur Bohrungsachse: Abstand der Bohrungsmitte von der Kante
    expect(measureBetween(edge([7, 0, 0], [7, 0, 10]), circle([2, 0, 10], [0, 0, 1], 1)).perpendicular?.d).toBeCloseTo(5, 9);
  });

  it('am echten Netz: Bohrungsabstand zweier Ringe nebeneinander', () => {
    const pos = concat(ring(10, 3, 4, 64), transformPositions(ring(10, 3, 4, 64), new THREE.Matrix4().makeTranslation(25, 0, 0)));
    const { at } = setup(pos);
    const a = at([3.1, 0, 4]);
    const b = at([28.1, 0, 4]);
    expect(measureBetween(a, b).direct?.d).toBeCloseTo(25, 3);
  });
});
