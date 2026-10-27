import { describe, expect, it } from 'vitest';
import { convexHull, stabilityOf, type P2 } from '../src/math/stability';

describe('Standfestigkeit', () => {
  it('konvexe Hülle: Innenpunkte und doppelte Punkte fallen weg, gegen den Uhrzeigersinn', () => {
    const h = convexHull([[0, 0], [10, 0], [10, 10], [0, 10], [5, 5], [2, 3], [10, 0], [5, 0]]);
    expect(h).toEqual([[0, 0], [10, 0], [10, 10], [0, 10]]);
  });

  it('Würfel: Schwerpunkt mittig, kippt ab 45°', () => {
    const square: P2[] = [[-10, -10], [10, -10], [10, 10], [-10, 10]];
    const s = stabilityOf([0, 0, 10], square, 0);
    expect(s.inside).toBe(true);
    expect(s.margin).toBeCloseTo(10, 9);
    expect(s.tipDeg).toBeCloseTo(45, 9);
  });

  it('hoher Turm auf kleiner Fläche kippt früh; Schwerpunkt neben der Fläche: kippt schon', () => {
    const small: P2[] = [[-2, -2], [2, -2], [2, 2], [-2, 2]];
    const tower = stabilityOf([0, 0, 40], small, 0);
    expect(tower.tipDeg).toBeCloseTo((Math.atan(2 / 40) * 180) / Math.PI, 9);
    const off = stabilityOf([5, 0, 10], small, 0);
    expect(off.inside).toBe(false);
    expect(off.margin).toBeCloseTo(-3, 9);
    expect(off.tipDeg).toBeLessThan(0);
    expect(off.edge).toEqual([2, 0]);
  });

  it('Boden höher als null; nur ein Berührpunkt (Kugel): balanciert höchstens', () => {
    const s = stabilityOf([0, 0, 15], [[-5, -5], [5, -5], [5, 5], [-5, 5]], 5);
    expect(s.height).toBeCloseTo(10, 9);
    expect(s.tipDeg).toBeCloseTo((Math.atan(5 / 10) * 180) / Math.PI, 9);
    const ball = stabilityOf([0, 0, 10], [[0, 0]], 0);
    expect(ball.inside).toBe(false);
    expect(ball.margin).toBeCloseTo(0, 9);
  });
});
