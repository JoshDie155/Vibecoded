import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { unionParts } from '../src/io/union';
import { box, cylinder, transformPositions } from './helpers/meshes';

/** Volumen eines geschlossenen Netzes (Divergenzsatz). */
function volume(p: Float32Array): number {
  let v = 0;
  for (let i = 0; i < p.length; i += 9) {
    const [ax, ay, az, bx, by, bz, cx, cy, cz] = p.subarray(i, i + 9);
    v += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6;
  }
  return v;
}
const move = (p: Float32Array, x: number, y: number, z: number) => transformPositions(p, new THREE.Matrix4().makeTranslation(x, y, z));

describe('unionParts', () => {
  it('vereinigt überlappende Teile: Überlappung zählt nur einmal', async () => {
    // Zwei Quader 10 × 10 × 10, um 5 mm versetzt: 1000 + 1000 − 500
    const r = await unionParts([
      { name: 'A', positions: box(10, 10, 10) },
      { name: 'B', positions: move(box(10, 10, 10), 5, 0, 0) },
    ]);
    expect(r.notClosed).toEqual([]);
    expect(volume(r.positions)).toBeCloseTo(1500, 3);
  });

  it('eingesunkener Deckel und Zapfen werden ein Körper', async () => {
    const r = await unionParts([
      { name: 'Haus', positions: box(40, 28, 24) },
      { name: 'Deckel', positions: move(box(40, 28, 5), 0, 0, 23.8) },
      { name: 'Zapfen', positions: move(cylinder(3, 10, 32), 0, 0, 25) },
    ]);
    expect(r.notClosed).toEqual([]);
    const v = 40 * 28 * 28.8 + Math.PI * 9 * (35 - 28.8) * 0.997; // Zylinder ist 32-eckig
    expect(volume(r.positions)).toBeGreaterThan(v * 0.995);
    expect(volume(r.positions)).toBeLessThan(40 * 28 * 28.8 + Math.PI * 9 * 6.2);
  });

  it('nicht geschlossene Teile werden angehängt und gemeldet', async () => {
    const open = box(10, 10, 10).slice(0, 9 * 10); // zwei Dreiecke fehlen
    const r = await unionParts([
      { name: 'Ganz', positions: move(box(10, 10, 10), 20, 0, 0) },
      { name: 'Offen', positions: open },
    ]);
    expect(r.notClosed).toEqual(['Offen']);
    expect(r.positions.length).toBeGreaterThanOrEqual(open.length + 12 * 9);
  });
});
