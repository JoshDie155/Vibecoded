import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import { type Vec3, dist, dot } from '../src/math/align';
import {
  type CylinderFeature, type Feature, type PlaneFeature, buildTopology, detectFeature, similarity, similarityFeature, transformMesh,
} from '../src/math/features';
import { box, concat, cylinder, transformPositions } from './helpers/meshes';

const triWhere = (pos: Float32Array, pred: (c: number[]) => boolean) => {
  for (let t = 0; t < pos.length / 9; t++) {
    const c = [0, 1, 2].map((k) => (pos[t * 9 + k] + pos[t * 9 + 3 + k] + pos[t * 9 + 6 + k]) / 3);
    if (pred(c)) return t;
  }
  throw new Error('kein Dreieck');
};
const near = (a: Vec3, b: Vec3, tol = 1e-3) => expect(dist(a, b)).toBeLessThan(tol);
const cross = (a: Vec3, b: Vec3): Vec3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

/** Platte 30 × 20 × 4 mit Zapfen r 5, 10 hoch, verschoben (nicht im Ursprung). */
const model = () => transformPositions(
  concat(box(30, 20, 4), transformPositions(cylinder(5, 10, 48), new THREE.Matrix4().makeTranslation(6, 2, 4))),
  new THREE.Matrix4().makeTranslation(3, -2, 1),
);
const top = (pos: Float32Array) => triWhere(pos, (c) => c[2] > 4.9 && c[2] < 5.1 && Math.hypot(c[0] - 9, c[1]) > 6);
const mantle = (pos: Float32Array) => triWhere(pos, (c) => Math.hypot(c[0] - 9, c[1]) > 4.9 && c[2] > 7 && c[2] < 13);

/** Umgerechnete Fläche = auf dem umgerechneten Netz neu erkannte Fläche. */
function sameAsDetected(m: THREE.Matrix4) {
  const pos = model();
  const S = similarity(m.elements);
  const moved = transformMesh(pos, m.elements);
  for (const tri of [top(pos), mantle(pos)]) {
    const f = detectFeature(buildTopology(pos), tri);
    const want = detectFeature(buildTopology(moved), tri) as Feature;
    const got = similarityFeature(f, S);
    expect(got.kind).toBe(want.kind);
    expect([...got.tris].sort((a, b) => a - b)).toEqual([...want.tris].sort((a, b) => a - b));
    if (got.kind === 'plane') {
      const w = want as PlaneFeature;
      near(got.centroid, w.centroid);
      near(got.normal, w.normal, 1e-6);
      expect(got.area).toBeCloseTo(w.area, 3);
      // Hauptachsen bleiben ein Rechtssystem
      near(cross(got.axes.u, got.axes.v), got.normal, 1e-6);
      expect(Math.max(...got.axes.size)).toBeCloseTo(Math.max(...w.axes.size), 3);
    } else {
      const c = got as CylinderFeature;
      const w = want as CylinderFeature;
      near(c.center, w.center);
      expect(Math.abs(dot(c.axis, w.axis))).toBeGreaterThan(1 - 1e-9);
      near(c.axis, w.axis, 1e-6);
      expect(c.radius).toBeCloseTo(w.radius, 4);
      expect(c.length).toBeCloseTo(w.length, 4);
      expect(c.hole).toBe(w.hole);
    }
  }
}

describe('Skalieren und Spiegeln', () => {
  it('Skalieren um einen Punkt: Flächen wie neu erkannt', () => {
    const c = new THREE.Vector3(5, 1, 3);
    sameAsDetected(new THREE.Matrix4().makeTranslation(c.x, c.y, c.z).multiply(new THREE.Matrix4().makeScale(2.5, 2.5, 2.5)).multiply(new THREE.Matrix4().makeTranslation(-c.x, -c.y, -c.z)));
  });

  it('Spiegeln: Flächen wie neu erkannt, Normalen zeigen nach außen', () => {
    const m = new THREE.Matrix4().makeTranslation(4, 0, 0).multiply(new THREE.Matrix4().makeScale(-1, 1, 1));
    sameAsDetected(m);
    sameAsDetected(new THREE.Matrix4().makeScale(1, 1, -1));
    // Umlaufsinn: Oberseite der Platte zeigt nach dem Spiegeln an x weiterhin nach oben
    const pos = model();
    const moved = transformMesh(pos, m.elements);
    const f = detectFeature(buildTopology(moved), top(pos)) as PlaneFeature;
    near(f.normal, [0, 0, 1], 1e-6);
    // Volumen bleibt positiv (geschlossenes Netz mit Außennormalen)
    const vol = (p: Float32Array) => {
      let v = 0;
      for (let t = 0; t < p.length; t += 9) {
        const a: Vec3 = [p[t], p[t + 1], p[t + 2]], b: Vec3 = [p[t + 3], p[t + 4], p[t + 5]], cc: Vec3 = [p[t + 6], p[t + 7], p[t + 8]];
        v += dot(a, cross(b, cc)) / 6;
      }
      return v;
    };
    expect(vol(moved)).toBeCloseTo(vol(pos), 3);
    expect(vol(pos)).toBeGreaterThan(0);
  });

  it('Faktor und Spiegelung werden erkannt', () => {
    const S = similarity(new THREE.Matrix4().makeRotationZ(0.7).multiply(new THREE.Matrix4().makeScale(-3, 3, 3)).elements);
    expect(S.s).toBeCloseTo(3, 9);
    expect(S.mirror).toBe(true);
    expect(similarity(new THREE.Matrix4().makeScale(0.5, 0.5, 0.5).elements).mirror).toBe(false);
  });
});
