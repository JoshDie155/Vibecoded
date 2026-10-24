import { describe, expect, it } from 'vitest';
import { addPocket, PRESETS } from '../src/io/pocket';
import { meshHealth } from '../src/math/repair';
import { box } from './helpers/meshes';

function volume(p: Float32Array): number {
  let v = 0;
  for (let i = 0; i < p.length; i += 9) {
    const [ax, ay, az, bx, by, bz, cx, cy, cz] = p.subarray(i, i + 9);
    v += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6;
  }
  return v;
}
const ngon = (r: number, n = 48) => 0.5 * n * r * r * Math.sin((2 * Math.PI) / n);
const top = { point: [0, 0, 20] as [number, number, number], normal: [0, 0, 1] as [number, number, number] };

describe('Aussparungen', () => {
  it('Magnet 6 × 3: runde Tasche mit Spiel von oben', async () => {
    const spec = { kind: 'magnet' as const, ...PRESETS.magnet[2].spec };
    const r = await addPocket(box(30, 30, 20), top, spec);
    expect(volume(r.positions)).toBeCloseTo(30 * 30 * 20 - ngon(3.1) * 3.2, 1);
    expect(meshHealth(r.positions).closed).toBe(true);
    expect(r.notes).toEqual([]);
    // Unberührte Dreiecke behalten ihre Herkunft, die Taschenwände sind neu
    expect([...r.source].filter((s) => s >= 0).length).toBeGreaterThan(8);
    expect([...r.source].some((s) => s < 0)).toBe(true);
  });

  it('Gewindeeinsatz M3 und seitliche Fläche', async () => {
    const spec = { kind: 'insert' as const, ...PRESETS.insert[2].spec };
    const r = await addPocket(box(30, 30, 20), { point: [15, 0, 10], normal: [1, 0, 0] }, spec);
    expect(volume(r.positions)).toBeCloseTo(30 * 30 * 20 - ngon(2) * 6.7, 1);
  });

  it('Schraube M3 mit Zylinderkopf: Loch ganz durch, Kopf versenkt', async () => {
    const spec = { kind: 'screw' as const, ...PRESETS.screw[2].spec };
    const r = await addPocket(box(30, 30, 20), top, spec);
    const want = 30 * 30 * 20 - ngon(1.7) * (20 - 3.3) - ngon(3) * 3.3;
    expect(volume(r.positions)).toBeCloseTo(want, 0);
    expect(meshHealth(r.positions).shells).toBe(1);
  });

  it('Senkkopf: Kegel statt Zylinder', async () => {
    const spec = { kind: 'screw' as const, ...PRESETS.screw[2].spec, head: 6.5, countersunk: true };
    const r = await addPocket(box(30, 30, 20), top, spec);
    // Kegelstumpf von r 3,25 auf 1,7 über 1,55 mm
    const h = 3.25 - 1.7;
    const cone = (Math.PI * h * (3.25 ** 2 + 3.25 * 1.7 + 1.7 ** 2)) / 3;
    const v = 30 * 30 * 20 - volume(r.positions);
    expect(v).toBeGreaterThan(cone * 0.97 + Math.PI * 1.7 ** 2 * (20 - h) * 0.97);
    expect(v).toBeLessThan(cone * 1.01 + Math.PI * 1.7 ** 2 * (20 - h) * 1.01);
  });

  it('Mutter M3: Sechskant mit Schraubenloch', async () => {
    const spec = { kind: 'nut' as const, ...PRESETS.nut[2].spec };
    const r = await addPocket(box(30, 30, 20), top, spec);
    const s = 5.5 + 0.3;
    const hex = (Math.sqrt(3) / 2) * s * s * 2.6;
    const want = 30 * 30 * 20 - hex - ngon(1.7) * (20 - 2.6);
    expect(volume(r.positions)).toBeCloseTo(want, 0);
  });

  it('warnt, wenn das Teil zu dünn ist', async () => {
    const spec = { kind: 'magnet' as const, ...PRESETS.magnet[2].spec };
    const r = await addPocket(box(30, 30, 3), { point: [0, 0, 3], normal: [0, 0, 1] }, spec);
    expect(r.notes.join(' ')).toMatch(/geht durch/);
  });

  it('daneben: klare Meldung', async () => {
    const spec = { kind: 'hole' as const, ...PRESETS.hole[2].spec };
    await expect(addPocket(box(10, 10, 10), { point: [100, 100, 10], normal: [0, 0, 1] }, spec)).rejects.toThrow('trifft das Teil nicht');
  });
});
