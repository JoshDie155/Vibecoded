import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { meshHealth, repairMesh } from '../src/math/repair';
import { cutMesh } from '../src/io/cut';
import { box, concat, cylinder, transformPositions } from './helpers/meshes';

function volume(p: Float32Array): number {
  let v = 0;
  for (let i = 0; i < p.length; i += 9) {
    const [ax, ay, az, bx, by, bz, cx, cy, cz] = p.subarray(i, i + 9);
    v += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6;
  }
  return v;
}
/** Dreieck t umdrehen (Ecken 2 und 3 tauschen). */
function flipTri(p: Float32Array, t: number): Float32Array {
  const q = p.slice();
  q.set(p.subarray(9 * t + 6, 9 * t + 9), 9 * t + 3);
  q.set(p.subarray(9 * t + 3, 9 * t + 6), 9 * t + 6);
  return q;
}
/** Dreiecke entfernen, deren Schwerpunkt pred erfüllt. */
function without(p: Float32Array, pred: (c: number[]) => boolean): Float32Array {
  const keep: number[] = [];
  for (let t = 0; t < p.length / 9; t++) {
    const c = [0, 1, 2].map((k) => (p[9 * t + k] + p[9 * t + 3 + k] + p[9 * t + 6 + k]) / 3);
    if (!pred(c)) keep.push(...p.subarray(9 * t, 9 * t + 9));
  }
  return Float32Array.from(keep);
}

describe('Netz prüfen', () => {
  it('geschlossener Quader ist in Ordnung, Reparatur ändert nichts', () => {
    const p = box(10, 20, 30);
    const h = meshHealth(p);
    expect(h).toMatchObject({ openEdges: 0, nonManifoldEdges: 0, flippedEdges: 0, degenerate: 0, duplicates: 0, shells: 1, closed: true });
    expect(h.volume).toBeCloseTo(6000, 3);
    const r = repairMesh(p);
    expect(Array.from(r.positions)).toEqual(Array.from(p));
    expect(Array.from(r.map)).toEqual([...Array(12).keys()]);
    expect(r.report).toMatchObject({ welded: 0, removed: 0, flipped: 0, holes: 0 });
  });

  it('erkennt Löcher, verdrehte Flächen und Duplikate', () => {
    const open = without(box(10, 10, 10), (c) => c[2] > 9.9);
    expect(meshHealth(open)).toMatchObject({ openEdges: 4, closed: false });
    expect(meshHealth(flipTri(box(10, 10, 10), 3)).flippedEdges).toBe(3);
    const dup = concat(box(10, 10, 10), box(10, 10, 10).subarray(0, 9));
    expect(meshHealth(dup)).toMatchObject({ duplicates: 1, degenerate: 0, closed: false });
    // Gleiche Ecken, gegensinnig: Innenwand – die Kanten zählen doppelt
    const wall = concat(box(10, 10, 10), flipTri(box(10, 10, 10).subarray(0, 9), 0));
    expect(meshHealth(wall)).toMatchObject({ duplicates: 2, closed: false });
    expect(meshHealth(wall).nonManifoldEdges).toBe(0);
  });
});

describe('Netz reparieren', () => {
  it('schließt ein Loch (fehlender Deckel)', () => {
    const open = without(box(10, 10, 10), (c) => c[2] > 9.9);
    const r = repairMesh(open);
    expect(r.report.holes).toBe(1);
    expect(r.report.holeTriangles).toBe(2);
    expect(r.report.after.closed).toBe(true);
    expect(volume(r.positions)).toBeCloseTo(1000, 3);
    // Die übrigen Dreiecke behalten ihre Reihenfolge
    expect(Array.from(r.map)).toEqual([...Array(10).keys()]);
  });

  it('schließt ein verwinkeltes, ebenes Loch richtig (L-Form, Ohren abschneiden)', () => {
    const shape = new THREE.Shape([[0, 0], [20, 0], [20, 8], [8, 8], [8, 20], [0, 20]].map(([x, y]) => new THREE.Vector2(x, y)));
    const g = new THREE.ExtrudeGeometry(shape, { depth: 5, bevelEnabled: false });
    const full = new Float32Array(g.toNonIndexed().getAttribute('position').array);
    const vol = (20 * 8 + 8 * 12) * 5;
    expect(volume(full)).toBeCloseTo(vol, 3);
    const open = without(full, (c) => c[2] > 4.99);
    const r = repairMesh(open);
    expect(r.report.after.closed).toBe(true);
    expect(volume(r.positions)).toBeCloseTo(vol, 3);
  });

  it('schließt einen offenen Rohrdeckel (rund, viele Kanten)', () => {
    const open = without(cylinder(6, 10, 48), (c) => c[2] > 9.99);
    const r = repairMesh(open);
    expect(r.report.after.closed).toBe(true);
    expect(volume(r.positions)).toBeCloseTo(volume(cylinder(6, 10, 48)), 2);
  });

  it('dreht verdrehte und nach innen zeigende Flächen um', () => {
    let p = box(10, 10, 10);
    for (const t of [1, 4, 7]) p = flipTri(p, t);
    const r = repairMesh(p);
    expect(r.report.flipped).toBe(3);
    expect(r.report.after.closed).toBe(true);
    expect(volume(r.positions)).toBeCloseTo(1000, 3);
    expect(Array.from(r.flippedTris).filter(Boolean)).toHaveLength(3);
    // Ganz nach innen gedreht (gespiegelt eingelesen)
    let inv = box(10, 10, 10);
    for (let t = 0; t < 12; t++) inv = flipTri(inv, t);
    expect(volume(inv)).toBeCloseTo(-1000, 3);
    const r2 = repairMesh(inv);
    expect(volume(r2.positions)).toBeCloseTo(1000, 3);
    expect(r2.report.flipped).toBe(12);
  });

  it('verschweißt feine Risse und entfernt Duplikate und Innenwände', () => {
    // Ecken um 1e-5 mm verrutscht
    const p = box(10, 10, 10);
    const cracked = p.slice();
    for (let i = 0; i < cracked.length; i += 9) cracked[i] += 1e-5;
    expect(meshHealth(cracked).closed).toBe(true); // innerhalb der Toleranz schon gleich
    const r = repairMesh(cracked);
    expect(r.report.welded).toBeGreaterThan(0);
    expect(r.report.after.closed).toBe(true);
    // Zwei Würfel, die sich eine Fläche teilen (gleiche Dreiecke, gegensinnig) → ein Quader
    const a = box(10, 10, 10);
    const top = without(a, (c) => c[2] < 9.9);
    const b = concat(without(transformPositions(box(10, 10, 10), new THREE.Matrix4().makeTranslation(0, 0, 10)), (c) => c[2] < 10.1), flipTri(flipTri(top, 0), 1));
    expect(meshHealth(b).closed).toBe(true);
    const r2 = repairMesh(concat(a, b));
    expect(r2.report.removed).toBe(4);
    expect(r2.report.after.closed).toBe(true);
    expect(r2.report.after.shells).toBe(1);
    expect(volume(r2.positions)).toBeCloseTo(2000, 3);
  });

  it('danach lässt sich das Teil zerschneiden', async () => {
    const open = without(box(30, 30, 30), (c) => c[2] > 29.9);
    await expect(cutMesh(open, { normal: [0, 0, 1], offset: 15, connector: 'none' })).rejects.toThrow('nicht wasserdicht');
    const r = repairMesh(open);
    const cut = await cutMesh(r.positions, { normal: [0, 0, 1], offset: 15, connector: 'pin' });
    expect(volume(cut.a) + volume(cut.b)).toBeGreaterThan(26000);
  });
});

describe('Netz prüfen: Ausrichtung', () => {
  it('ganz nach innen gedreht gilt nicht als geschlossen', () => {
    let inv = box(10, 10, 10);
    for (let t = 0; t < 12; t++) inv = flipTri(inv, t);
    expect(meshHealth(inv)).toMatchObject({ openEdges: 0, flippedEdges: 0, closed: false });
    expect(meshHealth(inv).volume).toBeLessThan(0);
  });
});
