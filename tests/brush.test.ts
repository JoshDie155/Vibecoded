import { describe, expect, it } from 'vitest';
import { brushPaint, decodePaint, paintLeaves, solidPaint, triangleDistance } from '../src/io/paint';

type V3 = [number, number, number];
const A: V3 = [0, 0, 0];
const B: V3 = [100, 0, 0];
const C: V3 = [0, 100, 0];

/** Fläche je Zustand eines bemalten Dreiecks. */
function areas(code: string, a: V3 = A, b: V3 = B, c: V3 = C): Map<number, number> {
  const out = new Map<number, number>();
  paintLeaves(decodePaint(code)!, a, b, c, (s, x, y, z) => {
    const u = [y[0] - x[0], y[1] - x[1], y[2] - x[2]];
    const v = [z[0] - x[0], z[1] - x[1], z[2] - x[2]];
    const ar = Math.hypot(u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]) / 2;
    out.set(s, (out.get(s) ?? 0) + ar);
  });
  return out;
}

describe('Malen mit dem Pinsel', () => {
  it('Abstand Punkt–Dreieck: innen, an der Kante, an der Ecke', () => {
    expect(triangleDistance([10, 10, 5], A, B, C)).toBeCloseTo(5, 9);
    expect(triangleDistance([50, -3, 0], A, B, C)).toBeCloseTo(3, 9);
    expect(triangleDistance([-3, -4, 0], A, B, C)).toBeCloseTo(5, 9);
    expect(triangleDistance([60, 60, 0], A, B, C)).toBeCloseTo(Math.SQRT2 * 10, 9);
  });

  it('kleiner Pinsel mitten auf einem großen Dreieck: Kreis fein unterteilt, Rest unbemalt', () => {
    const code = brushPaint('', A, B, C, { center: [30, 30, 0], radius: 5, state: 2, minEdge: 0.5 });
    const a = areas(code);
    // Kreisfläche π·25 ≈ 78,5 (am Rand auf ±halbe Teildreiecke genau)
    expect(a.get(2)!).toBeGreaterThan(70);
    expect(a.get(2)!).toBeLessThan(87);
    expect(a.get(0)! + a.get(2)!).toBeCloseTo(5000, 6);
  });

  it('Kugel umschließt das Dreieck: ganz bemalt; weit weg: unverändert', () => {
    expect(brushPaint('', A, B, C, { center: [30, 30, 0], radius: 200, state: 3, minEdge: 1 })).toBe(solidPaint(3));
    const once = brushPaint('', A, B, C, { center: [30, 30, 0], radius: 5, state: 1, minEdge: 1 });
    expect(brushPaint(once, A, B, C, { center: [300, 300, 0], radius: 5, state: 2, minEdge: 1 })).toBe(once);
  });

  it('Radieren (Zustand 0) über dieselbe Stelle: wieder unbemalt, Teildreiecke zusammengefasst', () => {
    const p = brushPaint('', A, B, C, { center: [30, 30, 0], radius: 5, state: 1, minEdge: 0.5 });
    expect(brushPaint(p, A, B, C, { center: [30, 30, 0], radius: 7, state: 0, minEdge: 0.5 })).toBe('');
  });

  it('übermalt fein bemalte Bambu-Dreiecke, ohne den Rest zu verändern', () => {
    // Bambu: eine Seite geteilt (zwei Kinder); Kind 1 (die Hälfte bei C) Filament 1, Kind 0 (bei B) unbemalt
    const bambu = '041';
    const before = areas(bambu);
    expect(before.get(1)).toBeGreaterThan(0);
    // Kleiner Strich mit Filament 3 in der unbemalten Hälfte: Filament 1 bleibt gleich groß
    const code = brushPaint(bambu, A, B, C, { center: [70, 10, 0], radius: 4, state: 3, minEdge: 0.5 });
    const after = areas(code);
    expect(after.get(1)).toBeCloseTo(before.get(1)!, 6);
    expect(after.get(3)!).toBeGreaterThan(40);
  });

  it('dieselbe Farbe nochmal: nichts ändert sich', () => {
    const p = brushPaint('', A, B, C, { center: [30, 30, 0], radius: 5, state: 1, minEdge: 0.5 });
    expect(brushPaint(p, A, B, C, { center: [30, 30, 0], radius: 5, state: 1, minEdge: 0.5 })).toBe(p);
  });
});
