import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { CutError, cutMesh } from '../src/io/cut';
import { box, transformPositions } from './helpers/meshes';

/** Volumen eines geschlossenen Netzes (Divergenzsatz). */
function volume(p: Float32Array): number {
  let v = 0;
  for (let i = 0; i < p.length; i += 9) {
    const [ax, ay, az, bx, by, bz, cx, cy, cz] = p.subarray(i, i + 9);
    v += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6;
  }
  return v;
}
const bounds = (p: Float32Array) => {
  const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < p.length; i += 3) for (let k = 0; k < 3; k++) { mn[k] = Math.min(mn[k], p[i + k]); mx[k] = Math.max(mx[k], p[i + k]); }
  return { mn, mx };
};
/** Fläche eines regelmäßigen 40-Ecks mit Umkreisradius r (so baut manifold die Zylinder). */
const ngon = (r: number) => 0.5 * 40 * r * r * Math.sin((2 * Math.PI) / 40);

describe('Zerschneiden', () => {
  it('teilt einen Würfel an einer Ebene in zwei geschlossene Hälften', async () => {
    const r = await cutMesh(box(20, 20, 20), { normal: [0, 0, 1], offset: 8, connector: 'none' });
    expect(volume(r.a)).toBeCloseTo(20 * 20 * 8, 3);
    expect(volume(r.b)).toBeCloseTo(20 * 20 * 12, 3);
    expect(bounds(r.a).mx[2]).toBeCloseTo(8, 5);
    expect(bounds(r.b).mn[2]).toBeCloseTo(8, 5);
    expect(r.connectors).toEqual([]);
  });

  it('Zapfen an der einen, Bohrung mit Spiel an der anderen Hälfte', async () => {
    const r = await cutMesh(box(30, 30, 30), { normal: [0, 0, 1], offset: 15, connector: 'pin', diameter: 6, clearance: 0.2 });
    expect(r.connectors).toHaveLength(1);
    const [p] = r.connectors;
    expect(Math.hypot(p.center[0], p.center[1])).toBeLessThan(1);
    expect(p.center[2]).toBeCloseTo(15, 6);
    expect(p.depthA).toBeCloseTo(9, 6); // 1,5 × ⌀, genug Material dahinter
    // A: Würfelhälfte plus herausstehender Zapfen (⌀ 6, 9 − 0,3 hoch)
    expect(volume(r.a)).toBeCloseTo(30 * 30 * 15 + ngon(3) * (9 - 0.3), 1);
    expect(bounds(r.a).mx[2]).toBeCloseTo(15 + 9 - 0.3, 4);
    // B: Würfelhälfte minus Bohrung (⌀ 6,4, 9,2 tief)
    expect(volume(r.b)).toBeCloseTo(30 * 30 * 15 - ngon(3.2) * 9.2, 1);
    expect(r.dowels).toEqual([]);
  });

  it('Dübel: Bohrungen in beiden Hälften und getrennte Dübel', async () => {
    const r = await cutMesh(box(30, 30, 30), { normal: [0, 0, 1], offset: 15, connector: 'dowel', diameter: 5, clearance: 0.15 });
    expect(r.connectors).toHaveLength(1);
    expect(r.dowels).toHaveLength(1);
    const depth = 7.5;
    expect(volume(r.a)).toBeCloseTo(30 * 30 * 15 - ngon(2.65) * (depth + 0.15), 1);
    expect(volume(r.b)).toBeCloseTo(30 * 30 * 15 - ngon(2.65) * (depth + 0.15), 1);
    expect(volume(r.dowels[0])).toBeCloseTo(ngon(2.5) * (2 * depth - 0.6), 1);
  });

  it('lange Querschnitte bekommen zwei Verbinder, weit auseinander', async () => {
    const r = await cutMesh(box(90, 14, 30), { normal: [0, 0, 1], offset: 15, connector: 'pin', clearance: 0.2 });
    expect(r.connectors).toHaveLength(2);
    const [a, b] = r.connectors;
    // Bei einem Viertel und drei Vierteln der Länge
    expect(Math.abs(a.center[0] - b.center[0])).toBeGreaterThan(35);
    // Mit Wand zum Rand: ⌀ automatisch (ein Drittel von 14 → 4,5 mm)
    expect(a.diameter).toBe(4.5);
    for (const c of r.connectors) expect(Math.abs(c.center[1])).toBeLessThan(14 / 2 - c.diameter / 2 - 1.2 + 1e-6);
  });

  it('feste Anzahl und schräge Ebene', async () => {
    const tilt = new THREE.Vector3(1, 0, 1).normalize();
    const r = await cutMesh(box(40, 40, 40), { normal: [tilt.x, tilt.y, tilt.z], offset: tilt.dot(new THREE.Vector3(0, 0, 20)), connector: 'pin', count: 3, diameter: 4 });
    expect(r.connectors).toHaveLength(3);
    for (const c of r.connectors) expect(tilt.dot(new THREE.Vector3(...c.center)) - tilt.dot(new THREE.Vector3(0, 0, 20))).toBeCloseTo(0, 6);
    // Würfel + herausstehende Zapfen − Bohrungen (⌀ 4,4, 0,2 tiefer)
    const want = 40 ** 3 + r.connectors.reduce((s, c) => s + ngon(2) * (c.depthB - 0.3) - ngon(2.2) * (c.depthB + 0.2), 0);
    expect(volume(r.a) + volume(r.b)).toBeCloseTo(want, 0);
  });

  it('zu dünn für Verbinder: Schnitt ohne, mit Hinweis', async () => {
    const r = await cutMesh(box(60, 10, 3), { normal: [1, 0, 0], offset: 0, connector: 'pin' });
    expect(r.connectors).toEqual([]);
    expect(r.notes.join(' ')).toMatch(/zu schmal/);
    expect(volume(r.a)).toBeCloseTo(30 * 10 * 3, 3);
  });

  it('flache Teile: Verbinder nur so tief, dass hinten Wand bleibt', async () => {
    // Platte 40 × 40 × 6, Schnitt quer (x = 0): dahinter je 20 mm – Tiefe 1,5 × ⌀ = 6
    const r = await cutMesh(transformPositions(box(40, 40, 30), new THREE.Matrix4()), { normal: [0, 0, 1], offset: 4, connector: 'dowel', diameter: 6 });
    expect(r.connectors).toHaveLength(1);
    expect(r.connectors[0].depthA).toBeCloseTo(4 - 1.2, 6);
  });

  it('meldet offene Netze und Ebenen neben dem Teil', async () => {
    const open = box(10, 10, 10).slice(0, -18); // eine Seite fehlt zur Hälfte
    await expect(cutMesh(open, { normal: [0, 0, 1], offset: 5, connector: 'none' })).rejects.toThrow(CutError);
    await expect(cutMesh(box(10, 10, 10), { normal: [0, 0, 1], offset: 50, connector: 'none' })).rejects.toThrow('geht nicht durch');
  });
});

describe('Zerschneiden: Herkunft der Dreiecke', () => {
  it('jedes Ergebnisdreieck kennt sein Ausgangsdreieck (gleiche Ebene), Schnittflächen −1', async () => {
    const src = box(20, 20, 20);
    const r = await cutMesh(src, { normal: [0, 0, 1], offset: 8, connector: 'pin', diameter: 4 });
    const normal = (p: Float32Array, t: number) => {
      const a = new THREE.Vector3().fromArray(p, 9 * t), b = new THREE.Vector3().fromArray(p, 9 * t + 3), c = new THREE.Vector3().fromArray(p, 9 * t + 6);
      return b.sub(a).cross(c.sub(a)).normalize();
    };
    for (const [pos, source] of [[r.a, r.aSource], [r.b, r.bSource]] as const) {
      expect(source.length).toBe(pos.length / 9);
      let tracked = 0;
      for (let t = 0; t < source.length; t++) {
        if (source[t] < 0) continue;
        tracked++;
        expect(normal(pos, t).dot(normal(src, source[t]))).toBeGreaterThan(0.999);
      }
      expect(tracked).toBeGreaterThan(5);
      expect([...source].some((s) => s < 0)).toBe(true);
    }
  });
});

describe('Zerschneiden: Schwalbenschwanz', () => {
  it('Feder an A, Nut mit Spiel in B – zusammengesetzt ohne Überschneidung', async () => {
    const { booleanParts } = await import('../src/io/boolean');
    const r = await cutMesh(box(60, 40, 30), { normal: [0, 0, 1], offset: 15, connector: 'dovetail', clearance: 0.2 });
    expect(r.connectors).toHaveLength(1);
    const [dt] = r.connectors;
    expect(dt.slide).toBeDefined();
    // Feder entlang der kurzen Seite (Y), Breite ein Viertel der langen (60 → 15 mm)
    expect(Math.abs(dt.slide![1])).toBeCloseTo(1, 6);
    expect(dt.diameter).toBe(15);
    expect(volume(r.a)).toBeGreaterThan(60 * 40 * 15 + 100);
    expect(volume(r.b)).toBeLessThan(60 * 40 * 15 - 100);
    // Nut etwas größer als die Feder (Spiel)
    expect(60 * 40 * 15 - volume(r.b)).toBeGreaterThan(volume(r.a) - 60 * 40 * 15);
    // Zusammengesteckt überschneiden sich die Hälften nicht (höchstens Berührung)
    const overlap = await booleanParts({ name: 'A', positions: r.a }, [{ name: 'B', positions: r.b }], 'intersect').then((x) => volume(x.positions), () => 0);
    expect(overlap).toBeLessThan(0.5);
    // Die Feder steht nirgends über das Teil hinaus
    const bb = (p: Float32Array) => { let mn = Infinity, mx = -Infinity; for (let i = 1; i < p.length; i += 3) { mn = Math.min(mn, p[i]); mx = Math.max(mx, p[i]); } return [mn, mx]; };
    expect(bb(r.a)[0]).toBeGreaterThanOrEqual(-20 - 1e-6);
    expect(bb(r.a)[1]).toBeLessThanOrEqual(20 + 1e-6);
  });

  it('flache Teile: Feder niedriger, notfalls keine', async () => {
    const r = await cutMesh(box(60, 40, 6), { normal: [0, 0, 1], offset: 3, connector: 'dovetail' });
    expect(r.connectors).toEqual([]);
    expect(r.notes.join(' ')).toMatch(/zu schmal oder zu flach/);
  });
});
