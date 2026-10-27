import { describe, expect, it } from 'vitest';
import { loadManifold } from '../src/io/union';
import { EmbossError, embossFrame, embossMesh } from '../src/io/emboss';
import { printable, textOutline } from '../src/text';
import * as THREE from 'three';
import { box, cylinder, toPositions } from './helpers/meshes';

function volume(p: Float32Array): number {
  let v = 0;
  for (let i = 0; i < p.length; i += 9) {
    const [ax, ay, az, bx, by, bz, cx, cy, cz] = p.subarray(i, i + 9);
    v += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6;
  }
  return v;
}

/** Fläche der Umrisse (positiv gefüllt) laut manifold. */
async function areaOf(polys: [number, number][][]): Promise<number> {
  const wasm = await loadManifold();
  const cs = new wasm.CrossSection(polys, 'Positive');
  const a = cs.area();
  cs.delete();
  return a;
}

describe('Text als Umrisse', () => {
  it('Großbuchstaben so hoch wie verlangt, Löcher in A und B, mittig', () => {
    const o = textOutline('AB', 10);
    expect(o.height).toBeCloseTo(10, 1);
    expect(o.width).toBeGreaterThan(12);
    // A: außen + 1 Loch, B: außen + 2 Löcher – Löcher laufen andersherum
    const signed = o.polys.map((p) => p.reduce((s, [x, y], i) => { const [x1, y1] = p[(i + 1) % p.length]; return s + x * y1 - x1 * y; }, 0));
    expect(signed.filter((a) => a > 0)).toHaveLength(2);
    expect(signed.filter((a) => a < 0)).toHaveLength(3);
    const xs = o.polys.flat().map(([x]) => x);
    expect(Math.min(...xs) + Math.max(...xs)).toBeCloseTo(0, 6);
  });

  it('Umlaute und ß sind in der Schrift; ⌀ wird ø, Unbekanntes fällt weg', () => {
    expect(printable('Größe ÄÖÜ')).toBe('Größe ÄÖÜ');
    expect(printable('⌀ 5 ☃')).toBe('ø 5 ');
    expect(textOutline('Ä', 8).height).toBeGreaterThan(8.5);
  });

  it('zwei Zeilen untereinander', () => {
    const one = textOutline('12', 6);
    const two = textOutline('12\n34', 6);
    expect(two.height).toBeGreaterThan(one.height + 6);
  });
});

describe('Beschriftung einprägen', () => {
  const top = { point: [0, 0, 10] as [number, number, number], normal: [0, 0, 1] as [number, number, number], up: [0, 1, 0] as [number, number, number] };

  it('erhaben auf der Oberseite: Volumen wächst um Fläche × Höhe, Schrift steht oben heraus', async () => {
    const o = textOutline('PJ', 8);
    const area = await areaOf(o.polys);
    const r = await embossMesh(box(40, 30, 10), top, { polys: o.polys, mode: 'raised', depth: 0.8 }, undefined);
    expect(volume(r.positions)).toBeCloseTo(40 * 30 * 10 + area * 0.8, 1);
    let zmax = -Infinity;
    for (let i = 2; i < r.positions.length; i += 3) zmax = Math.max(zmax, r.positions[i]);
    expect(zmax).toBeCloseTo(10.8, 5);
    // Neue Flächen ohne Herkunft, die Oberseite außerhalb der Schrift bleibt ein altes Dreieck (geteilt)
    expect([...r.source].some((s) => s < 0)).toBe(true);
    expect([...r.source].some((s) => s >= 0)).toBe(true);
    expect(r.notes).toEqual([]);
  });

  it('vertieft: Volumen sinkt um Fläche × Tiefe; zu tief in dünner Wand gibt einen Hinweis', async () => {
    const o = textOutline('8', 6);
    const area = await areaOf(o.polys);
    const r = await embossMesh(box(30, 30, 10), top, { polys: o.polys, mode: 'engraved', depth: 0.6 }, undefined);
    expect(volume(r.positions)).toBeCloseTo(30 * 30 * 10 - area * 0.6, 1);
    const thin = await embossMesh(box(30, 30, 1), { ...top, point: [0, 0, 1] }, { polys: o.polys, mode: 'engraved', depth: 0.6 }, undefined);
    expect(thin.notes.join(' ')).toMatch(/nur 0,4 mm/);
  });

  it('auf einem Zylinder folgt die Schrift der Rundung', async () => {
    const o = textOutline('ABC', 8);
    expect(o.width).toBeGreaterThan(18);
    const place = { point: [15, 0, 20] as [number, number, number], normal: [1, 0, 0] as [number, number, number], up: [0, 0, 1] as [number, number, number] };
    const base = cylinder(15, 40, 96);
    const r = await embossMesh(base, place, { polys: o.polys, mode: 'raised', depth: 1 }, undefined);
    // Alle Punkte der Beschriftung liegen in der Schale zwischen Zylinder und Zylinder + 1 mm
    let worst = 0;
    for (let t = 0; t < r.source.length; t++) {
      if (r.source[t] >= 0) continue;
      for (let k = 0; k < 3; k++) {
        const x = r.positions[9 * t + 3 * k], y = r.positions[9 * t + 3 * k + 1];
        const rad = Math.hypot(x, y);
        worst = Math.max(worst, rad - 16, 14.6 - rad);
      }
    }
    expect(worst).toBeLessThan(0.05);
    // Ungefähr Fläche × Höhe dazu (Rundung macht die äußere Fläche etwas größer)
    const area = await areaOf(o.polys);
    const added = volume(r.positions) - volume(base);
    expect(added).toBeGreaterThan(area * 0.9);
    expect(added).toBeLessThan(area * 1.15);
  });

  it('an einer Stufe klettert die Schrift nicht die Wand hoch, sondern läuft eben weiter', async () => {
    // L-Profil: unten 40 breit bis z = 5, links ein Block bis z = 15 (Wand bei x = −10), 20 tief
    const shape = new THREE.Shape([[-20, 0], [20, 0], [20, 5], [-10, 5], [-10, 15], [-20, 15]].map(([x, y]) => new THREE.Vector2(x, y)));
    const g = new THREE.ExtrudeGeometry(shape, { depth: 20, bevelEnabled: false });
    g.rotateX(Math.PI / 2).translate(0, 10, 0);
    const p = toPositions(g);
    const o = textOutline('STUFE', 8);
    expect(o.width).toBeGreaterThan(24);
    const place = { point: [2, 0, 5] as [number, number, number], normal: [0, 0, 1] as [number, number, number], up: [0, 1, 0] as [number, number, number] };
    const r = await embossMesh(p, place, { polys: o.polys, mode: 'raised', depth: 1 }, undefined);
    let top = -Infinity;
    for (let t = 0; t < r.source.length; t++) {
      if (r.source[t] >= 0) continue;
      for (let k = 0; k < 3; k++) {
        const x = r.positions[9 * t + 3 * k], z = r.positions[9 * t + 3 * k + 2];
        if (x > -10) top = Math.max(top, z);
      }
    }
    expect(top).toBeCloseTo(6, 5);
    expect(r.notes.join(' ')).toMatch(/Kante oder Stufe/);
  });

  it('Drehung: 90° stellt die Schrift hochkant', () => {
    const f = embossFrame(top, 90);
    expect(f.u[1]).toBeCloseTo(1, 9);
    expect(f.v[0]).toBeCloseTo(-1, 9);
    // u × v = n (von außen lesbar)
    const n = [f.u[1] * f.v[2] - f.u[2] * f.v[1], f.u[2] * f.v[0] - f.u[0] * f.v[2], f.u[0] * f.v[1] - f.u[1] * f.v[0]];
    expect(n[2]).toBeCloseTo(1, 9);
  });

  it('über den Rand hinaus: Hinweis; daneben: Fehler', async () => {
    const o = textOutline('LANG', 8);
    const r = await embossMesh(box(20, 20, 10), top, { polys: o.polys, mode: 'engraved', depth: 0.5 }, undefined);
    expect(r.notes.join(' ')).toMatch(/über eine Kante/);
    await expect(embossMesh(box(20, 20, 10), { ...top, point: [0, 0, 30] }, { polys: o.polys, mode: 'engraved', depth: 0.5 }, undefined)).rejects.toThrow(EmbossError);
  });
});
