import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import { type CylinderFeature, type PlaneFeature, buildTopology, detectFeature } from '../src/math/features';
import { offsetCylinder, offsetPlane } from '../src/math/offset';
import { box, concat, cylinder, ring, transformPositions } from './helpers/meshes';

const triWhere = (pos: Float32Array, pred: (c: number[]) => boolean) => {
  for (let t = 0; t < pos.length / 9; t++) {
    const c = [0, 1, 2].map((k) => (pos[t * 9 + k] + pos[t * 9 + 3 + k] + pos[t * 9 + 6 + k]) / 3);
    if (pred(c)) return t;
  }
  throw new Error('kein Dreieck');
};

describe('Passungshilfe', () => {
  it('macht einen Zapfen auf einer Platte dünner – Deckel und Platte bleiben dicht', () => {
    // Platte 30 × 30 × 4 mit Zapfen r 5, 10 hoch obendrauf (Zapfen als eigenes, aufgesetztes Netz)
    const pos = concat(box(30, 30, 4), transformPositions(cylinder(5, 10, 48), new THREE.Matrix4().makeTranslation(0, 0, 4)));
    const topo = buildTopology(pos);
    // Mantel (nicht der Deckel bei z = 14)
    const f = detectFeature(topo, triWhere(pos, (c) => Math.hypot(c[0], c[1]) > 4.9 && c[2] > 6 && c[2] < 12)) as CylinderFeature;
    expect(f.kind).toBe('cylinder');
    const r = offsetCylinder(topo, f, f.radius - 0.1);
    expect(r.flippedArea).toBeLessThan(0.1);
    const back = detectFeature(buildTopology(r.positions), f.tris[0]) as CylinderFeature;
    expect(back.kind).toBe('cylinder');
    expect(back.radius).toBeCloseTo(f.radius - 0.1, 4);
    // Höhe unverändert, die Platte ebenso (ihre Ecken gehören nicht zum Zapfen)
    expect(back.length).toBeCloseTo(f.length, 6);
    expect(Array.from(r.positions.subarray(0, box(30, 30, 4).length))).toEqual(Array.from(pos.subarray(0, box(30, 30, 4).length)));
  });

  it('macht eine Bohrung weiter', () => {
    const pos = ring(10, 4, 6, 48);
    const topo = buildTopology(pos);
    const f = detectFeature(topo, triWhere(pos, (c) => Math.hypot(c[0], c[1]) < 4.1 && c[2] > 1 && c[2] < 5)) as CylinderFeature;
    expect(f.kind).toBe('cylinder');
    expect(f.hole).toBe(true);
    const r = offsetCylinder(topo, f, f.radius + 0.15);
    // Nur Splitter der Deckel-Triangulierung kippen
    expect(r.flippedArea).toBeLessThan(0.2);
    const back = detectFeature(buildTopology(r.positions), f.tris[0]) as CylinderFeature;
    expect(back.radius).toBeCloseTo(f.radius + 0.15, 4);
    expect(back.hole).toBe(true);
  });

  it('setzt eine ebene Fläche zurück; zu viel wird gemeldet', () => {
    const pos = box(20, 10, 6);
    const topo = buildTopology(pos);
    const f = detectFeature(topo, triWhere(pos, (c) => Math.abs(c[2] - 6) < 1e-6)) as PlaneFeature;
    expect(f.kind).toBe('plane');
    const r = offsetPlane(topo, f, -0.2);
    expect(r.flippedArea).toBe(0);
    let top = -Infinity;
    for (let i = 2; i < r.positions.length; i += 3) top = Math.max(top, r.positions[i]);
    expect(top).toBeCloseTo(5.8, 5);
    // 7 mm zurück bei 6 mm Höhe: die Seitenwände kippen um
    expect(offsetPlane(topo, f, -7).flippedArea).toBeGreaterThan(50);
  });
});
