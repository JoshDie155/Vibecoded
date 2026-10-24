import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import { type Vec3, apply, rotate } from '../src/math/align';
import { bestBase, layFlat } from '../src/math/orient';
import { box, concat, cylinder, transformPositions } from './helpers/meshes';

describe('layFlat', () => {
  it('dreht die Richtung nach unten, auch aus der Gegenrichtung', () => {
    for (const n of [[1, 0, 0], [0.3, -0.5, 0.8], [0, 0, 1], [0, 0, -1]] as Vec3[]) {
      const T = layFlat(n, [5, 5, 5]);
      const d = rotate(T.R, n);
      const l = Math.hypot(...n);
      expect(d[2] / l).toBeCloseTo(-1, 9);
      // Drehpunkt bleibt liegen
      expect(apply(T, [5, 5, 5]).map((x) => +x.toFixed(9))).toEqual([5, 5, 5]);
    }
  });
});

describe('bestBase', () => {
  it('legt einen hochkant stehenden Quader auf seine größte Seite', () => {
    // 40 × 20 × 4, auf der schmalen 20 × 4-Seite stehend
    const p = transformPositions(box(40, 20, 4), new THREE.Matrix4().makeRotationY(Math.PI / 2));
    const c = bestBase(p);
    expect(c.area).toBeCloseTo(800, 0);
    expect(c.current).toBeCloseTo(80, 0);
    expect(Math.abs(c.normal[0])).toBeCloseTo(1, 6);
  });

  it('bleibt bei einer Lage, die schon die größte Auflage hat', () => {
    const c = bestBase(box(40, 20, 4));
    expect(c.normal).toEqual([0, 0, -1]);
    expect(c.area).toBeCloseTo(c.current, 6);
  });

  it('Auflage zählt nur, was unten in einer Ebene liegt (Zylinder auf Sockel)', () => {
    // Sockel 30 × 30 × 5 mit Zylinder obendrauf: unten liegt der Sockel, oben die Zylinder-Stirnfläche
    const p = concat(box(30, 30, 5), transformPositions(cylinder(5, 20, 32), new THREE.Matrix4().makeTranslation(0, 0, 5)));
    const turned = transformPositions(p, new THREE.Matrix4().makeRotationX(2.2));
    const c = bestBase(turned);
    expect(c.area).toBeCloseTo(900, 0);
    // Danach liegt der Sockel unten
    const T = layFlat(c.normal, [0, 0, 0]);
    const m = new THREE.Matrix4().set(T.R[0], T.R[1], T.R[2], T.t[0], T.R[3], T.R[4], T.R[5], T.t[1], T.R[6], T.R[7], T.R[8], T.t[2], 0, 0, 0, 1);
    const flat = transformPositions(turned, m);
    expect(bestBase(flat).current).toBeCloseTo(900, 0);
  });
});
