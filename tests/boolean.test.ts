import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { BoolError, booleanParts } from '../src/io/boolean';
import { box, cylinder, transformPositions } from './helpers/meshes';

function volume(p: Float32Array): number {
  let v = 0;
  for (let i = 0; i < p.length; i += 9) {
    const [ax, ay, az, bx, by, bz, cx, cy, cz] = p.subarray(i, i + 9);
    v += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6;
  }
  return v;
}
const move = (p: Float32Array, x: number, y: number, z: number) => transformPositions(p, new THREE.Matrix4().makeTranslation(x, y, z));
/** Querschnitt eines Testzylinders (r, 32 Segmente) – aus seinem Volumen. */
const ngon = (r: number, n: number) => volume(cylinder(r, 10, n)) / 10;

describe('Teile verrechnen', () => {
  const block = { name: 'Block', positions: box(40, 40, 20) };
  const peg = { name: 'Zapfen', positions: move(cylinder(5, 30, 32), 0, 0, -5) };
  it('abziehen: Loch in Form des anderen Teils', async () => {
    const r = await booleanParts(block, [peg], 'subtract');
    expect(volume(r.positions)).toBeCloseTo(40 * 40 * 20 - ngon(5, 32) * 20, 1);
    expect([...r.source].filter((s) => s >= 0).length).toBeGreaterThan(4);
  });
  it('vereinigen: ein Körper, Überlappung zählt einmal', async () => {
    const r = await booleanParts(block, [peg], 'union');
    expect(volume(r.positions)).toBeCloseTo(40 * 40 * 20 + ngon(5, 32) * 10, 1);
  });
  it('Schnittmenge: nur das Gemeinsame', async () => {
    const r = await booleanParts(block, [peg], 'intersect');
    expect(volume(r.positions)).toBeCloseTo(ngon(5, 32) * 20, 1);
  });
  it('mehrere Werkzeuge auf einmal', async () => {
    const r = await booleanParts(block, [peg, { name: 'Z2', positions: move(cylinder(3, 30, 32), 12, 12, -5) }], 'subtract');
    expect(volume(r.positions)).toBeCloseTo(40 * 40 * 20 - (ngon(5, 32) + ngon(3, 32)) * 20, 1);
  });
  it('verständliche Fehler', async () => {
    await expect(booleanParts(block, [{ name: 'weit weg', positions: move(box(5, 5, 5), 100, 0, 0) }], 'subtract')).rejects.toThrow('berühren sich nicht');
    await expect(booleanParts(block, [{ name: 'weit weg', positions: move(box(5, 5, 5), 100, 0, 0) }], 'intersect')).rejects.toThrow('überschneiden sich nicht');
    await expect(booleanParts(block, [{ name: 'offen', positions: box(5, 5, 5).slice(0, -9) }], 'union')).rejects.toThrow(BoolError);
  });
});
