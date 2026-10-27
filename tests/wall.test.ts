import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { buildTopology, detectFeature } from '../src/math/features';
import { cylinderWall } from '../src/math/wall';
import { ring, transformPositions } from './helpers/meshes';

const I = new THREE.Matrix4().elements;

/** Dreiecke eines Netzes nach Winkel ihrer Mitte (um z) auf zwei Netze verteilen – wie ein Schnitt durch die Bohrung. */
function splitByAngle(p: Float32Array, from: number, to: number): [Float32Array, Float32Array] {
  const a: number[] = [];
  const b: number[] = [];
  for (let t = 0; t < p.length / 9; t++) {
    const cx = (p[9 * t] + p[9 * t + 3] + p[9 * t + 6]) / 3;
    const cy = (p[9 * t + 1] + p[9 * t + 4] + p[9 * t + 7]) / 3;
    const ang = ((Math.atan2(cy, cx) * 180) / Math.PI + 360) % 360;
    (ang >= from && ang < to ? a : b).push(...p.subarray(9 * t, 9 * t + 9));
  }
  return [new Float32Array(a), new Float32Array(b)];
}

/** Zahl der Dreiecke auf der Zylinderwand mit Radius r (Mitte nahe r, Normale quer zur z-Achse). */
function wallCount(p: Float32Array, r: number): number {
  const topo = buildTopology(p);
  let n = 0;
  for (let t = 0; t < topo.triCount; t++) {
    const c = [topo.centroids[3 * t], topo.centroids[3 * t + 1]];
    if (Math.abs(Math.hypot(c[0], c[1]) - r) < 0.3 && Math.abs(topo.normals[3 * t + 2]) < 0.1) n++;
  }
  return n;
}

/** Ein Dreieck der Bohrungswand (Normale zur Achse hin) bei etwa diesem Winkel. */
function wallTri(p: Float32Array, rHole: number, deg: number): number {
  const topo = buildTopology(p);
  let best = -1;
  let bestD = Infinity;
  for (let t = 0; t < topo.triCount; t++) {
    const c = [topo.centroids[3 * t], topo.centroids[3 * t + 1]];
    if (Math.abs(Math.hypot(c[0], c[1]) - rHole) > 0.3 || Math.abs(topo.normals[3 * t + 2]) > 0.1) continue;
    const d = Math.abs((((Math.atan2(c[1], c[0]) * 180) / Math.PI - deg) % 360 + 540) % 360 - 180);
    if (d < bestD) { bestD = d; best = t; }
  }
  return best;
}

describe('Ganze Wand einer Bohrung', () => {
  it('über zwei Teile verteilt (Schnitt durch die Bohrung): beide Stücke, Durchmesser genau', () => {
    const [pa, pb] = splitByAngle(ring(15, 3, 8, 48), 0, 200);
    const sources = [pa, pb].map((p) => ({ topo: buildTopology(p), matrix: I }));
    const seed = wallTri(pa, 3, 100);
    const f = detectFeature(sources[0].topo, seed);
    expect(f.kind).toBe('cylinder');
    if (f.kind !== 'cylinder') return;
    const w = cylinderWall(sources, { source: 0, tris: f.tris, center: f.center, axis: f.axis, radius: f.radius, hole: f.hole });
    expect(w.radius).toBeCloseTo(3, 4);
    // Die ganze Wand, auf beide Teile verteilt
    expect(w.pieces[0].length + w.pieces[1].length).toBe(wallCount(ring(15, 3, 8, 48), 3));
    expect(w.pieces[1].length).toBeGreaterThan(30);
    expect(w.span[1] - w.span[0]).toBeCloseTo(8, 4);
  });

  it('Teile in verschiedener Lage (Matrix): Wand trotzdem gefunden', () => {
    const [pa, pb] = splitByAngle(ring(15, 3, 8, 48), 0, 200);
    // Teil B liegt in seinem Netz woanders, die Lage bringt es zurück
    const move = new THREE.Matrix4().makeRotationZ(0.7).setPosition(40, -10, 5);
    const pbLocal = transformPositions(pb, move.clone().invert());
    const sources = [{ topo: buildTopology(pa), matrix: I }, { topo: buildTopology(pbLocal), matrix: move.elements }];
    const f = detectFeature(sources[0].topo, wallTri(pa, 3, 100));
    if (f.kind !== 'cylinder') throw new Error(f.kind);
    const w = cylinderWall(sources, { source: 0, tris: f.tris, center: f.center, axis: f.axis, radius: f.radius, hole: true });
    expect(w.pieces[0].length + w.pieces[1].length).toBe(wallCount(ring(15, 3, 8, 48), 3));
  });

  it('schmaler Bogen mit ungenauem Kreis: wächst auf die ganze Wand, Durchmesser stimmt', () => {
    const p = ring(15, 3, 8, 64);
    const topo = buildTopology(p);
    const seed = wallTri(p, 3, 45);
    // Nur drei Facetten, Kreis absichtlich falsch (kleiner, versetzt)
    const tris: number[] = [];
    for (let t = 0; t < topo.triCount; t++) {
      const c = [topo.centroids[3 * t], topo.centroids[3 * t + 1]];
      const ang = (Math.atan2(c[1], c[0]) * 180) / Math.PI;
      if (Math.abs(Math.hypot(c[0], c[1]) - 3) < 0.3 && Math.abs(topo.normals[3 * t + 2]) < 0.1 && ang > 35 && ang < 55) tris.push(t);
    }
    expect(tris).toContain(seed);
    const w = cylinderWall([{ topo, matrix: I }], { source: 0, tris, center: [0.08, 0.06, 4], axis: [0, 0, 1], radius: 2.93, hole: true });
    expect(w.radius).toBeCloseTo(3, 2);
    expect(Math.hypot(w.center[0], w.center[1])).toBeLessThan(0.01);
    expect(w.pieces[0].length).toBe(wallCount(p, 3));
  });

  it('zweite Bohrung auf derselben Achse, aber getrennt: bleibt draußen', () => {
    const a = ring(15, 3, 8, 48);
    const b = transformPositions(ring(15, 3, 8, 48), new THREE.Matrix4().makeTranslation(0, 0, 20));
    const sources = [a, b].map((p) => ({ topo: buildTopology(p), matrix: I }));
    const f = detectFeature(sources[0].topo, wallTri(a, 3, 10));
    if (f.kind !== 'cylinder') throw new Error(f.kind);
    const w = cylinderWall(sources, { source: 0, tris: f.tris, center: f.center, axis: f.axis, radius: f.radius, hole: true });
    expect(w.pieces[0].length).toBe(wallCount(a, 3));
    expect(w.pieces[1].length).toBe(0);
    // Liegt das zweite Teil direkt an (bei z = 8), gehört seine Bohrung dazu
    const c = transformPositions(ring(15, 3, 8, 48), new THREE.Matrix4().makeTranslation(0, 0, 8));
    const w2 = cylinderWall([sources[0], { topo: buildTopology(c), matrix: I }], { source: 0, tris: f.tris, center: f.center, axis: f.axis, radius: f.radius, hole: true });
    expect(w2.pieces[1].length).toBe(wallCount(a, 3));
  });

  it('Zapfen statt Bohrung: Außenwand des Rings, nicht die Bohrung', () => {
    const p = ring(15, 3, 8, 48);
    const topo = buildTopology(p);
    const f = detectFeature(topo, wallTri(p, 15, 30));
    if (f.kind !== 'cylinder') throw new Error(f.kind);
    expect(f.hole).toBe(false);
    const w = cylinderWall([{ topo, matrix: I }], { source: 0, tris: f.tris, center: f.center, axis: f.axis, radius: f.radius, hole: false });
    expect(w.radius).toBeCloseTo(15, 4);
    expect(w.pieces[0].length).toBe(wallCount(p, 15));
  });
});
