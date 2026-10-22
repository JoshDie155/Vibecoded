import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { apply, axisAngle } from '../src/math/align';
import { type Feature, type PlaneFeature, buildTopology } from '../src/math/features';
import { findCandidateFaces } from '../src/math/segment';
import { candidatePairs, placePlanePair, shapeSimilarity } from '../src/math/suggest';
import { box, cylinder, ring, toPositions, transformPositions } from './helpers/meshes';

/** L-förmiges Prisma (unsymmetrisch), Unterseite bei z = 0. */
function lPrism(h: number): Float32Array {
  const s = new THREE.Shape();
  s.moveTo(0, 0); s.lineTo(30, 0); s.lineTo(30, 8); s.lineTo(10, 8); s.lineTo(10, 20); s.lineTo(0, 20); s.lineTo(0, 0);
  return toPositions(new THREE.ExtrudeGeometry(s, { depth: h, bevelEnabled: false }));
}
const planeAt = (fs: Feature[], nz: number, z: number) =>
  fs.find((f): f is PlaneFeature => f.kind === 'plane' && Math.abs(f.normal[2] - nz) < 1e-6 && Math.abs(f.centroid[2] - z) < 1e-4)!;

describe('findCandidateFaces', () => {
  it('Quader: sechs ebene Flächen', () => {
    const fs = findCandidateFaces(buildTopology(box(20, 10, 5)));
    expect(fs.filter((f) => f.kind === 'plane')).toHaveLength(6);
  });

  it('Zylinder: Mantel und zwei Kreisflächen; Ring: Bohrung', () => {
    const c = findCandidateFaces(buildTopology(cylinder(5, 10, 48)));
    expect(c.filter((f) => f.kind === 'cylinder')).toHaveLength(1);
    expect(c.filter((f) => f.kind === 'plane' && f.circle)).toHaveLength(2);
    const r = findCandidateFaces(buildTopology(ring(12, 5.1, 8, 48)));
    expect(r.some((f) => f.kind === 'cylinder' && f.hole && Math.abs(f.radius - 5.1) < 0.05)).toBe(true);
  });
});

describe('Vorschläge', () => {
  it('erkennt deckungsgleiche Schnittflächen und legt sie exakt aufeinander', () => {
    const fixedFaces = findCandidateFaces(buildTopology(lPrism(10)));
    // Bewegliches Teil: anderes Stück desselben L-Profils, beliebig gedreht und verschoben
    const M = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(1, 2, 3).normalize(), 2.1).setPosition(50, -20, 7);
    const moving = transformPositions(lPrism(6), M);
    const movingFaces = findCandidateFaces(buildTopology(moving));
    const pairs = candidatePairs(fixedFaces, movingFaces);
    expect(pairs.length).toBeGreaterThan(0);
    // Die L-Flächen passen am besten (Ober-/Unterseiten beider Teile)
    expect(pairs[0].shape).toBeGreaterThan(0.99);
    const top = planeAt(fixedFaces, 1, 10);
    const bottom = movingFaces.find((f): f is PlaneFeature => f.kind === 'plane' && f.area > 300 && pairs.some((p) => p.moving === f && p.fixed === top))!;
    const placed = placePlanePair(top, bottom);
    expect(placed).not.toBeNull();
    expect(placed!.rms).toBeLessThan(1e-3);
    // Eine Ecke des beweglichen L landet genau auf der passenden Ecke des festen
    const corner = new THREE.Vector3(10, 20, 0).applyMatrix4(M);
    const w = apply(placed!.T, [corner.x, corner.y, corner.z]);
    const hits = [[10, 20, 10], [10, 20, 16]].some((c) => Math.hypot(w[0] - c[0], w[1] - c[1], w[2] - c[2]) < 1e-3);
    // Die untere Fläche des beweglichen Teils liegt auf z = 10 (die obere wäre gespiegelt nicht deckungsgleich)
    expect(hits || Math.abs(w[2] - 10) < 1e-3).toBe(true);
  });

  it('L-Fläche passt nicht auf die gespiegelte L-Fläche desselben Teils (Rest groß)', () => {
    const faces = findCandidateFaces(buildTopology(lPrism(10)));
    const top = planeAt(faces, 1, 10);
    // Oberseite auf Oberseite: Normalen gegeneinander erzwingt eine Spiegelung des L → passt nicht
    const placed = placePlanePair(top, top);
    expect(placed === null || placed.rms > 0.5).toBe(true);
  });

  it('Zapfen passt zu Bohrung mit kleinem Spiel, nicht zu einer viel größeren', () => {
    const peg = findCandidateFaces(buildTopology(cylinder(5, 10, 48))).find((f) => f.kind === 'cylinder')!;
    const hole = findCandidateFaces(buildTopology(ring(12, 5.15, 8, 48))).find((f) => f.kind === 'cylinder' && f.hole)!;
    const big = findCandidateFaces(buildTopology(ring(14, 7, 8, 48))).find((f) => f.kind === 'cylinder' && f.hole)!;
    expect(shapeSimilarity(hole, peg)).toBeGreaterThan(0.8);
    expect(shapeSimilarity(big, peg)).toBeLessThan(0.01);
  });

  it('dreht nach Hauptachsen richtig, auch bei großer Drehung um die Normale', () => {
    const fixedFaces = findCandidateFaces(buildTopology(lPrism(10)));
    const top = planeAt(fixedFaces, 1, 10);
    for (const a of [0.3, 1.7, 3.0, 4.4]) {
      const R = axisAngle([0, 0, 1], a);
      const M = new THREE.Matrix4().set(R[0], R[1], R[2], 5, R[3], R[4], R[5], 9, R[6], R[7], R[8], -3, 0, 0, 0, 1);
      const mf = findCandidateFaces(buildTopology(transformPositions(lPrism(6), M)));
      const bottom = mf.find((f): f is PlaneFeature => f.kind === 'plane' && f.normal[2] < -0.99)!;
      const placed = placePlanePair(top, bottom);
      expect(placed!.rms).toBeLessThan(1e-3);
    }
  });
});

describe('Quadrate und Kreise (Hauptachsen nicht eindeutig)', () => {
  it('Trägheitsradien hängen nicht von der Lage der Hauptachsen ab', async () => {
    const { gyrationRadii } = await import('../src/math/suggest');
    const a = findCandidateFaces(buildTopology(box(12, 12, 8))).find((f): f is PlaneFeature => f.kind === 'plane' && f.normal[2] > 0.99)!;
    const R = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(1, 2, 0.3).normalize(), 0.77);
    const b = findCandidateFaces(buildTopology(transformPositions(box(12, 12, 8), R))).find((f): f is PlaneFeature => f.kind === 'plane' && Math.abs(f.area - 144) < 1e-3)!;
    const [a1, a2] = gyrationRadii(a);
    const [b1, b2] = gyrationRadii(b);
    expect(a1).toBeCloseTo(12 / Math.sqrt(12), 3);
    expect(a2).toBeCloseTo(12 / Math.sqrt(12), 3);
    expect(b1).toBeCloseTo(a1, 3);
    expect(b2).toBeCloseTo(a2, 3);
    expect(shapeSimilarity(a, b)).toBeGreaterThan(0.99);
  });

  it('legt verdrehte Quadrate exakt aufeinander', () => {
    const fixed = findCandidateFaces(buildTopology(box(12, 12, 8))).find((f): f is PlaneFeature => f.kind === 'plane' && f.normal[2] > 0.99)!;
    for (const angle of [0.2, 0.785, 1.3]) {
      const R = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(0.3, 1, 0.5).normalize(), 1.9).multiply(new THREE.Matrix4().makeRotationZ(angle));
      const m = findCandidateFaces(buildTopology(transformPositions(box(12, 12, 8), R)));
      const bottom = m.find((f): f is PlaneFeature => f.kind === 'plane' && Math.abs(f.area - 144) < 1e-3)!;
      expect(placePlanePair(fixed, bottom)!.rms).toBeLessThan(1e-3);
    }
  });
});
