import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { type Rigid, IDENTITY, apply, axisAngle, compose, rotationAbout } from '../src/math/align';
import { sampleSurface } from '../src/math/icp';
import type { FreeMotion } from '../src/math/motion';
import { settleContact } from '../src/math/settle';
import { alignOutlines } from '../src/math/outline';
import { box, cylinder, dPeg, dRing, geometryFrom, ring } from './helpers/meshes';

const shift = (z: number): Rigid => ({ R: IDENTITY.R, t: [0, 0, z] });

describe('settleContact', () => {
  it('zieht einen schwebenden Ring genau auf die Auflage (Verschieben entlang der Achse)', () => {
    const base = box(40, 40, 6);
    const fixed = geometryFrom(base);
    const bvh = new MeshBVH(fixed);
    const cap = geometryFrom(ring(10, 5, 8));
    const samples = sampleSurface(cap, 3000);
    const free: FreeMotion[] = [
      { kind: 'translate', axis: [0, 0, 1] },
      { kind: 'rotate', point: [0, 0, 0], axis: [0, 0, 1] },
    ];
    for (const z0 of [6.12, 5.9, 6.03]) {
      const res = settleContact(samples, shift(z0), { bvh }, free, { maxPairDist: 0.3 });
      expect(res).not.toBeNull();
      expect(res!.T.t[2]).toBeCloseTo(6, 5);
      // Drehung um die Achse hat keinen Einfluss und bleibt unverändert
      expect(res!.T.R[0]).toBeCloseTo(1, 9);
    }
  });

  it('richtet eine leicht verkippte Auflage nur entlang freier Bewegungen aus', () => {
    const base = box(40, 40, 6);
    const bvh = new MeshBVH(geometryFrom(base));
    const samples = sampleSurface(geometryFrom(box(20, 20, 4)), 3000);
    // Kippen um die x-Achse ist frei, Verschieben entlang z auch
    const tilt = compose(shift(6.05), { R: axisAngle([1, 0, 0], 0.004), t: [0, 0, 0] });
    const free: FreeMotion[] = [
      { kind: 'translate', axis: [0, 0, 1] },
      { kind: 'rotate', point: [0, 0, 6], axis: [1, 0, 0] },
    ];
    const res = settleContact(samples, tilt, { bvh }, free, { maxPairDist: 0.3, maxRotation: 0.01 });
    expect(res).not.toBeNull();
    for (const p of [[10, 10, 0], [-10, -10, 0], [10, -10, 0]] as [number, number, number][]) expect(apply(res!.T, p)[2]).toBeCloseTo(6, 4);
    // Ohne erlaubte Drehung bleibt die Neigung, nur die Höhe wird angepasst
    const flat = settleContact(samples, tilt, { bvh }, free, { maxPairDist: 0.3 });
    expect(flat!.T.R).toEqual(tilt.R);
  });

  it('dreht einen D-Zapfen mit Spiel nicht einseitig in Kontakt', () => {
    const hole = dRing(12, 5.15, 3.15, 8);
    const bvh = new MeshBVH(geometryFrom(hole));
    const peg = sampleSurface(geometryFrom(dPeg(5, 3, 8)), 3000);
    const free: FreeMotion[] = [{ kind: 'rotate', point: [0, 0, 0], axis: [0, 0, 1] }];
    const T0 = shift(0);
    const res = settleContact(peg, T0, { bvh }, free, { maxPairDist: 0.3 });
    expect(res!.T.R).toEqual(T0.R);
  });

  it('meldet null ohne Kontakt', () => {
    const base = box(40, 40, 6);
    const bvh = new MeshBVH(geometryFrom(base));
    const samples = sampleSurface(geometryFrom(cylinder(5, 5)), 500);
    expect(settleContact(samples, shift(20), { bvh }, [{ kind: 'translate', axis: [0, 0, 1] }], { maxPairDist: 0.3 })).toBeNull();
  });
});

describe('alignOutlines', () => {
  const rect = (w: number, d: number, z: number): [number, number, number][] => [
    [-w / 2, -d / 2, z], [w / 2, -d / 2, z], [w / 2, d / 2, z], [-w / 2, d / 2, z],
  ];

  it('legt gleiche Rechtecke exakt übereinander', () => {
    const dst = rect(40, 28, 24);
    const src = rect(40, 28, 0);
    // fast richtig: 0,4° verdreht und 0,3 mm verschoben
    const T = compose({ R: IDENTITY.R, t: [0.3, -0.2, 24] }, rotationAbout([0, 0, 0], [0, 0, 1], 0.007));
    const res = alignOutlines(src, dst, T, [0, 0, 1], 0.5);
    expect(res).not.toBeNull();
    expect(res!.rms).toBeLessThan(1e-4);
    const c = apply(res!.T, [20, 14, 0]);
    expect(c[0]).toBeCloseTo(20, 4);
    expect(c[1]).toBeCloseTo(14, 4);
    expect(c[2]).toBeCloseTo(24, 9);
  });

  it('ändert nichts, wenn die Umrisse nicht zusammenpassen', () => {
    const res = alignOutlines(rect(20, 10, 0), rect(40, 28, 24), shift(24), [0, 0, 1], 0.5);
    expect(res).toBeNull();
  });

  it('funktioniert auch bei schräg im Raum liegenden Flächen', () => {
    const R = axisAngle([1, 2, 3], 1.2);
    const M = new THREE.Matrix4().set(R[0], R[1], R[2], 5, R[3], R[4], R[5], -3, R[6], R[7], R[8], 2, 0, 0, 0, 1);
    const dst = rect(30, 12, 0).map((p) => new THREE.Vector3(...p).applyMatrix4(M).toArray() as [number, number, number]);
    const n = new THREE.Vector3(0, 0, 1).transformDirection(M).toArray() as [number, number, number];
    const exact: Rigid = { R, t: [5, -3, 2] };
    const T = compose(rotationAbout(apply(exact, [0, 0, 0]), n, 0.01), exact);
    const res = alignOutlines(rect(30, 12, 0), dst, T, n, 0.5);
    expect(res).not.toBeNull();
    const c = apply(res!.T, [15, 6, 0]);
    const e = apply(exact, [15, 6, 0]);
    for (let k = 0; k < 3; k++) expect(c[k]).toBeCloseTo(e[k], 4);
  });
});
