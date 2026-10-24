import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { describe, expect, it } from 'vitest';
import { IDENTITY, type Vec3, apply } from '../src/math/align';
import { detectEdge } from '../src/math/edges';
import { type EdgeFeature, buildTopology, detectFeature, featureFromTris, mergeFeatures } from '../src/math/features';
import { fitFeatures } from '../src/math/fit';
import { sampleSurface } from '../src/math/icp';
import { describeFeature, featureTag } from '../src/math/mate';
import { box, cylinder, geometryFrom, toPositions, transformPositions } from './helpers/meshes';

/** Dreieck, das den Punkt enthält (Mitte am nächsten). */
function triNear(pos: Float32Array, p: Vec3): number {
  let best = -1;
  let bd = Infinity;
  for (let t = 0; t < pos.length / 9; t++) {
    const c = [0, 1, 2].map((k) => (pos[t * 9 + k] + pos[t * 9 + 3 + k] + pos[t * 9 + 6 + k]) / 3);
    const d = Math.hypot(c[0] - p[0], c[1] - p[1], c[2] - p[2]);
    if (d < bd) { bd = d; best = t; }
  }
  return best;
}

describe('detectEdge', () => {
  it('findet die gerade Kante neben dem Klick, über die ganze Länge', () => {
    const pos = box(20, 10, 6);
    const topo = buildTopology(pos);
    // Klick auf die Oberseite, knapp neben der Kante x = 10
    const p: Vec3 = [9.6, 2, 6];
    const e = detectEdge(topo, triNear(pos, p), p, 1)!;
    expect(e).not.toBeNull();
    expect(e.length).toBeCloseTo(10, 5);
    expect(Math.abs(e.dir[1])).toBeCloseTo(1, 6);
    expect(e.center[0]).toBeCloseTo(10, 5);
    expect(e.center[2]).toBeCloseTo(6, 5);
    // Normale zeigt nach außen-oben (zwischen +x und +z)
    expect(e.normal[0]).toBeGreaterThan(0.5);
    expect(e.normal[2]).toBeGreaterThan(0.5);
  });

  it('setzt fein unterteilte Kantenstücke zu einer Kante zusammen', () => {
    const g = new THREE.BoxGeometry(30, 12, 8, 15, 6, 4);
    g.translate(0, 0, 4);
    const pos = toPositions(g);
    const topo = buildTopology(pos);
    const p: Vec3 = [3.3, 5.8, 8];
    const e = detectEdge(topo, triNear(pos, p), p, 0.5)!;
    expect(e.length).toBeCloseTo(30, 4);
    expect(Math.abs(e.dir[0])).toBeCloseTo(1, 6);
    expect(e.tris.length).toBeGreaterThanOrEqual(15);
  });

  it('meldet nichts, wenn keine Kante in der Nähe ist', () => {
    const pos = box(20, 10, 6);
    const topo = buildTopology(pos);
    const p: Vec3 = [0, 0, 6];
    expect(detectEdge(topo, triNear(pos, p), p, 1)).toBeNull();
  });

  it('weiche Facetten eines Zylinders sind keine Kanten', () => {
    const pos = cylinder(5, 10, 48);
    const topo = buildTopology(pos);
    const p: Vec3 = [5, 0, 5];
    expect(detectEdge(topo, triNear(pos, p), p, 0.3)).toBeNull();
  });

  it('wird verständlich beschrieben', () => {
    const pos = box(20, 10, 6);
    const p: Vec3 = [9.8, 0, 6];
    const e = detectEdge(buildTopology(pos), triNear(pos, p), p, 1)!;
    expect(describeFeature(e)).toBe('Kante 10,0 mm');
    expect(featureTag(e)).toBe('L');
  });
});

describe('Kante auf Kante', () => {
  it('legt einen Klotz bündig an die Kante eines Sockels (Fläche + Kante)', () => {
    const fixed = box(10, 10, 10);
    const moving = transformPositions(box(10, 10, 4), new THREE.Matrix4().makeRotationZ(1.1).setPosition(40, 25, 3));
    const fTopo = buildTopology(fixed);
    const mTopo = buildTopology(moving);
    const top = detectFeature(fTopo, triNear(fixed, [0, 0, 10]));
    const fEdge = detectEdge(fTopo, triNear(fixed, [4.8, 0, 10]), [4.8, 0, 10], 0.5)!;
    // Unterseite und eine untere Kante des verdrehten Klotzes
    const M = new THREE.Matrix4().makeRotationZ(1.1).setPosition(40, 25, 3);
    const w = (p: Vec3) => new THREE.Vector3(...p).applyMatrix4(M).toArray() as Vec3;
    const bottom = detectFeature(mTopo, triNear(moving, w([0, 0, 0])));
    const mEdge = detectEdge(mTopo, triNear(moving, w([4.8, 0, 0])), w([4.8, 0, 0]), 0.5) as EdgeFeature;
    expect(bottom.kind).toBe('plane');
    expect(mEdge.length).toBeCloseTo(10, 4);
    const geom = geometryFrom(fixed);
    const fit = fitFeatures({
      srcF: [bottom, mEdge], dstF: [top, fEdge], srcM: [], dstM: [], names: { src: 'B', dst: 'A' },
      alt: 0, seat: true, ordered: false, normals: 'auto', overlap: 0, scale: 20,
      samples: sampleSurface(geometryFrom(moving), 2000), target: { bvh: new MeshBVH(geom), positions: fixed, matrix: IDENTITY },
    });
    expect(fit.error).toBeUndefined();
    // Die Ecke (5, 5, 0) des Klotzes landet auf (5, 5, 10) oder (5, -5, 10) – Kante auf Kante, Fläche auf Fläche
    const corner = apply(fit.rel, w([5, 5, 0]));
    expect(corner[0]).toBeCloseTo(5, 3);
    expect(corner[2]).toBeCloseTo(10, 3);
    expect(Math.abs(corner[1])).toBeCloseTo(5, 2);
  });
});

describe('featureFromTris (Pinsel)', () => {
  it('bemalte Dreiecke einer Ebene ergeben eine ebene Fläche mit Umriss', () => {
    const g = new THREE.BoxGeometry(20, 10, 6, 10, 5, 3);
    g.translate(0, 0, 3);
    const pos = toPositions(g);
    const topo = buildTopology(pos);
    // Ein Teil der Oberseite: Dreiecke mit Mitte bei x < 0
    const tris: number[] = [];
    for (let t = 0; t < pos.length / 9; t++) {
      const z = [2, 5, 8].map((k) => pos[t * 9 + k]);
      const x = (pos[t * 9] + pos[t * 9 + 3] + pos[t * 9 + 6]) / 3;
      if (z.every((v) => Math.abs(v - 6) < 1e-6) && x < 0) tris.push(t);
    }
    const f = featureFromTris(topo, tris);
    expect(f.kind).toBe('plane');
    if (f.kind !== 'plane') return;
    expect(f.normal[2]).toBeCloseTo(1, 6);
    expect(f.area).toBeCloseTo(100, 4);
    expect(f.centroid[0]).toBeCloseTo(-5, 4);
    expect(Math.max(...f.axes.size)).toBeCloseTo(10, 3);
  });

  it('über eine Kante hinweg bemalt: Freiformfläche mit mittlerer Normale', () => {
    const pos = box(10, 10, 10);
    const topo = buildTopology(pos);
    const top = triNear(pos, [0, 0, 10]);
    const side = triNear(pos, [5, 0, 5]);
    const f = featureFromTris(topo, [top, side]);
    expect(f.kind).toBe('surface');
    if (f.kind !== 'surface') return;
    expect(f.normal[0]).toBeGreaterThan(0.3);
    expect(f.normal[2]).toBeGreaterThan(0.3);
  });
});

describe('mergeFeatures (Flächen zusammenfassen)', () => {
  const half = (pos: Float32Array, pred: (x: number) => boolean) => {
    const tris: number[] = [];
    for (let t = 0; t < pos.length / 9; t++) {
      const z = [2, 5, 8].map((k) => pos[t * 9 + k]);
      const x = (pos[t * 9] + pos[t * 9 + 3] + pos[t * 9 + 6]) / 3;
      if (z.every((v) => Math.abs(v - 6) < 1e-6) && pred(x)) tris.push(t);
    }
    return tris;
  };
  const g = new THREE.BoxGeometry(20, 10, 6, 10, 5, 3);
  g.translate(0, 0, 3);
  const pos = toPositions(g);

  it('zwei Hälften einer Ebene ergeben wieder die ganze ebene Fläche', () => {
    const f = mergeFeatures([
      { positions: pos, tris: half(pos, (x) => x < 0), T: IDENTITY },
      { positions: pos, tris: half(pos, (x) => x > 0), T: IDENTITY },
    ]);
    expect(f.kind).toBe('plane');
    if (f.kind !== 'plane') return;
    expect(f.area).toBeCloseTo(200, 3);
    expect(f.centroid[0]).toBeCloseTo(0, 4);
    expect(Math.max(...f.axes.size)).toBeCloseTo(20, 2);
  });

  it('fasst Flächen zweier Teile zusammen (zweites Teil an seiner Lage)', () => {
    // Zweites Teil: dieselbe Box, um 20 mm verschoben – lokal liegt seine Hälfte bei x < 0
    const T = { R: IDENTITY.R, t: [20, 0, 0] as Vec3 };
    const f = mergeFeatures([
      { positions: pos, tris: half(pos, (x) => x > 0), T: IDENTITY },
      { positions: pos, tris: half(pos, (x) => x < 0), T },
    ]);
    expect(f.kind).toBe('plane');
    if (f.kind !== 'plane') return;
    expect(f.area).toBeCloseTo(200, 3);
    expect(f.centroid[0]).toBeCloseTo(10, 4);
  });

  it('nicht in einer Ebene: Freiformfläche', () => {
    const b = box(10, 10, 10);
    const f = mergeFeatures([
      { positions: b, tris: [triNear(b, [0, 0, 10])], T: IDENTITY },
      { positions: b, tris: [triNear(b, [5, 0, 5])], T: IDENTITY },
    ]);
    expect(f.kind).toBe('surface');
  });

  // Mantel eines Zapfens (r 7,5, 48 Facetten), aufgeteilt nach Winkel um die Achse
  const pin = cylinder(7.5, 4, 48);
  const side = (pred: (angle: number) => boolean) => {
    const tris: number[] = [];
    for (let t = 0; t < pin.length / 9; t++) {
      const z = [2, 5, 8].map((k) => pin[t * 9 + k]);
      if (z.every((v) => Math.abs(v - z[0]) < 1e-6)) continue; // Deckel
      const x = (pin[t * 9] + pin[t * 9 + 3] + pin[t * 9 + 6]) / 3;
      const y = (pin[t * 9 + 1] + pin[t * 9 + 4] + pin[t * 9 + 7]) / 3;
      if (pred(Math.atan2(y, x))) tris.push(t);
    }
    return tris;
  };

  it('zwei Stücke einer runden Fläche bleiben rund (Achse der runden Fläche als Startwert)', () => {
    const f = mergeFeatures([
      { positions: pin, tris: side((a) => a >= 0 && a < 1), T: IDENTITY },
      { positions: pin, tris: side((a) => a >= 1 && a < 2.2), T: IDENTITY },
    ], [0, 0, 1]);
    expect(f.kind).toBe('cylinder');
    if (f.kind !== 'cylinder') return;
    expect(f.radius).toBeCloseTo(7.5, 3);
    expect(f.hole).toBe(false);
    expect(Math.abs(f.axis[2])).toBeCloseTo(1, 6);
    expect(f.arc).toBeGreaterThan(1.9);
  });

  it('runde Fläche mit einer ebenen zusammen: Freiformfläche', () => {
    const b = transformPositions(box(10, 10, 4), new THREE.Matrix4().makeTranslation(12, 0, 0));
    const both = new Float32Array(pin.length + b.length);
    both.set(pin);
    both.set(b, pin.length);
    const top = triNear(b, [12, 0, 4]) + pin.length / 9;
    const f = mergeFeatures([
      { positions: both, tris: side((a) => a >= 0 && a < 1), T: IDENTITY },
      { positions: both, tris: [top], T: IDENTITY },
    ], [0, 0, 1]);
    expect(f.kind).toBe('surface');
  });
});
