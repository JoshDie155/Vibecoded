import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { labelSpot } from '../src/math/labelspot';
import { box, concat, ring, transformPositions } from './helpers/meshes';

/** Rechteck der Schrift (wie labelSpot es prüft) als vier Ecken. */
function corners(s: NonNullable<ReturnType<typeof labelSpot>>, size: { w: number; h: number }): [number, number][] {
  const m = 0.12 * s.height + 0.6;
  const hw = (size.w * s.height) / 2 + m, hh = (size.h * s.height) / 2 + m;
  // Leserichtung u = v × n
  const [vx, vy] = s.up;
  const n = s.normal[2];
  const ux = vy * n, uy = -vx * n;
  return [[-1, -1], [1, -1], [1, 1], [-1, 1]].map(([a, b]) => [s.point[0] + a * hw * ux + b * hh * vx, s.point[1] + a * hw * uy + b * hh * vy]);
}

describe('Platz für Nummern', () => {
  const size = { w: 2, h: 1.2 };

  it('Unterseite eines Quaders: mittig, entlang der langen Seite, so groß wie die Breite erlaubt', () => {
    const s = labelSpot(box(60, 20, 10), 'bottom', size, 30)!;
    expect(s).not.toBeNull();
    expect(s.point[2]).toBeCloseTo(0, 6);
    expect(s.normal).toEqual([0, 0, -1]);
    expect(Math.abs(s.point[0])).toBeLessThan(15);
    // Schrift läuft entlang x: „oben“ zeigt quer (y)
    expect(Math.abs(s.up[1])).toBeCloseTo(1, 6);
    // 20 mm breit: Höhe · 1,2 + 2 · Rand ≤ 20 → knapp 13 mm
    expect(s.height).toBeGreaterThan(12);
    expect(s.height).toBeLessThan(13.1);
    for (const [x, y] of corners(s, size)) {
      expect(Math.abs(x)).toBeLessThanOrEqual(30 + 1e-6);
      expect(Math.abs(y)).toBeLessThanOrEqual(10 + 1e-6);
    }
    // Von unten gelesen: Leserichtung × oben = Normale (nach unten)
    const u = [s.up[1] * s.normal[2], -s.up[0] * s.normal[2], 0];
    expect(u[0] * s.up[1] - u[1] * s.up[0]).toBeCloseTo(-1, 6);
  });

  it('höchstens so groß wie verlangt; zu kleine Fläche: kein Platz', () => {
    expect(labelSpot(box(60, 20, 10), 'top', size, 6)!.height).toBeCloseTo(6, 6);
    expect(labelSpot(box(4, 4, 4), 'top', size, 10)).toBeNull();
  });

  it('Ring: die Nummer meidet die Bohrung', () => {
    const s = labelSpot(ring(20, 6, 8, 64), 'bottom', size, 30)!;
    expect(s).not.toBeNull();
    // Kein Eckpunkt des Rechtecks in der Bohrung oder außerhalb des Rings, auch keine Kante quer durch die Bohrung
    const pts = corners(s, size);
    for (let i = 0; i < 4; i++) {
      const [a, b] = [pts[i], pts[(i + 1) % 4]];
      for (let t = 0; t <= 1; t += 0.05) {
        const r = Math.hypot(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]));
        expect(r).toBeLessThan(20 + 1e-6);
      }
    }
    const cx = s.point[0], cy = s.point[1];
    expect(Math.hypot(cx, cy)).toBeGreaterThan(6 + (size.h * s.height) / 2);
    expect(s.height).toBeGreaterThan(3);
  });

  it('oben: die größere von zwei Ebenen (Stufe)', () => {
    // Stufe: unten 40 × 20 bis z = 5, darauf links 10 × 20 bis z = 15
    const p = concat(box(40, 20, 5), transformPositions(box(10, 20, 10), new THREE.Matrix4().makeTranslation(-15, 0, 5)));
    const s = labelSpot(p, 'top', size, 30)!;
    expect(s.point[2]).toBeCloseTo(5, 6);
    expect(s.point[0]).toBeGreaterThan(-10);
  });
});
