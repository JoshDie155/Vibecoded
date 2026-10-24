import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { SimplifyError, simplifyCounts, simplifyMesh } from '../src/io/simplify';
import { meshHealth } from '../src/math/repair';
import { box, toPositions } from './helpers/meshes';

/** Fein unterteilte Kugel (geschlossen, verschweißt). */
function sphere(r: number, detail: number): Float32Array {
  const g = new THREE.IcosahedronGeometry(r, detail);
  return toPositions(g);
}
const volume = (p: Float32Array) => {
  let v = 0;
  for (let i = 0; i < p.length; i += 9) {
    const [ax, ay, az, bx, by, bz, cx, cy, cz] = p.subarray(i, i + 9);
    v += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6;
  }
  return v;
};

describe('Netz vereinfachen', () => {
  const ball = sphere(20, 40);

  it('weniger Dreiecke, geschlossen, Oberfläche höchstens um die Toleranz verschoben', async () => {
    const r = await simplifyMesh(ball, 0.05);
    expect(r.before).toBe(ball.length / 9);
    expect(r.after).toBe(r.positions.length / 9);
    expect(r.after).toBeLessThan(r.before * 0.6);
    expect(meshHealth(r.positions).closed).toBe(true);
    // Ecken bleiben auf der Kugelfläche (Teilmenge der alten Punkte), Dreiecksmitten höchstens 0,05 mm darunter
    let worst = 0;
    for (let i = 0; i < r.positions.length; i += 9) {
      const c = new THREE.Vector3();
      for (let k = 0; k < 3; k++) c.add(new THREE.Vector3(r.positions[i + 3 * k], r.positions[i + 3 * k + 1], r.positions[i + 3 * k + 2]));
      worst = Math.max(worst, 20 - c.divideScalar(3).length());
    }
    const facet = (() => {
      // Abstand der Mitte eines Ausgangsdreiecks zur Kugel: so weit lag das Netz schon vorher innen
      let m = 0;
      for (let i = 0; i < ball.length; i += 9) {
        const c = new THREE.Vector3(ball[i] + ball[i + 3] + ball[i + 6], ball[i + 1] + ball[i + 4] + ball[i + 7], ball[i + 2] + ball[i + 5] + ball[i + 8]).divideScalar(3);
        m = Math.max(m, 20 - c.length());
      }
      return m;
    })();
    expect(worst).toBeLessThan(facet + 0.05 + 1e-3);
    expect(volume(r.positions) / volume(ball)).toBeGreaterThan(0.995);
    expect(Array.from(r.source).every((s) => s === -1)).toBe(true);
  });

  it('Gruppen: Grenzen bleiben, Dreiecke mit eigener Gruppe unverändert, Herkunft je Gruppe', async () => {
    const n = ball.length / 9;
    const z = (t: number) => (ball[t * 9 + 2] + ball[t * 9 + 5] + ball[t * 9 + 8]) / 3;
    const x = (t: number) => (ball[t * 9] + ball[t * 9 + 3] + ball[t * 9 + 6]) / 3;
    // Kappe oben als Gruppe 1, einzelne Dreiecke am Äquator mit eigener Gruppe
    const single = new Set<number>();
    const groups = new Uint32Array(n).map((_, t) => {
      if (Math.abs(z(t)) < 0.4 && x(t) > 19) { single.add(t); return 100 + t; }
      return z(t) > 10 ? 1 : 0;
    });
    expect(single.size).toBeGreaterThan(3);
    const r = await simplifyMesh(ball, 0.1, groups);
    expect(r.after).toBeLessThan(r.before * 0.6);
    expect(meshHealth(r.positions).closed).toBe(true);
    const kept = new Map<number, number>();
    r.source.forEach((s, i) => { if (single.has(s)) kept.set(s, i); });
    expect(kept.size).toBe(single.size);
    const key = (p: Float32Array, t: number) => [0, 1, 2].map((k) => Array.from(p.subarray(t * 9 + 3 * k, t * 9 + 3 * k + 3)).map((v) => v.toFixed(4)).join(',')).sort().join(' ');
    for (const [s, i] of kept) expect(key(r.positions, i)).toBe(key(ball, s));
    // Kappe: alle Dreiecke stammen aus Gruppe 1 und liegen oberhalb der alten Grenze
    const capRep = groups.indexOf(1);
    for (let i = 0; i < r.after; i++) {
      const src = r.source[i];
      expect(src).toBeGreaterThanOrEqual(0);
      if (src !== capRep) continue;
      for (let k = 0; k < 3; k++) expect(r.positions[i * 9 + 3 * k + 2]).toBeGreaterThan(9);
    }
  });

  it('Anzahl je Toleranz: gröber = weniger Dreiecke; ein Würfel bleibt ein Würfel', async () => {
    const counts = await simplifyCounts(ball, [0.01, 0.05, 0.2]);
    expect(counts[0]).toBeGreaterThan(counts[1]);
    expect(counts[1]).toBeGreaterThan(counts[2]);
    const cube = await simplifyMesh(box(10, 10, 10), 0.5);
    expect(cube.after).toBe(12);
    expect(volume(cube.positions)).toBeCloseTo(1000, 3);
  });

  it('offene Netze: verständlicher Fehler', async () => {
    await expect(simplifyMesh(box(10, 10, 10).slice(0, -9), 0.05)).rejects.toThrow(SimplifyError);
    await expect(simplifyMesh(box(10, 10, 10).slice(0, -9), 0.05)).rejects.toThrow('Netz reparieren');
  });
});
