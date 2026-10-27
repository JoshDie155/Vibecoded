import { describe, expect, it } from 'vitest';
import { groupIdentical, layoutBalloons } from '../src/math/sheet';

describe('Montageanleitung', () => {
  it('gleiche Teile zusammen – auch anders gedreht; ähnliche, aber andere getrennt', () => {
    const dowel = { tris: 160, volume: 392.7, dims: [5, 5, 20] as [number, number, number] };
    const lying = { tris: 160, volume: 392.7, dims: [20, 5, 5] as [number, number, number] };
    const longer = { tris: 160, volume: 412.3, dims: [5, 5, 21] as [number, number, number] };
    const half = { tris: 1200, volume: 18000, dims: [40, 30, 15] as [number, number, number] };
    const otherHalf = { tris: 1200, volume: 18000.01, dims: [40, 30, 15] as [number, number, number] };
    expect(groupIdentical([half, dowel, otherHalf, lying, longer])).toEqual([[0, 2], [1, 3], [4]]);
  });

  it('Ballons überdecken sich nicht, bleiben im Bild und liegen nahe an ihrem Anker', () => {
    const anchors = [{ x: 400, y: 300 }, { x: 405, y: 302 }, { x: 410, y: 298 }, { x: 100, y: 100 }, { x: 790, y: 590 }];
    const r = 16;
    const pts = layoutBalloons(anchors, { x: 400, y: 300 }, 800, 600, r);
    for (let i = 0; i < pts.length; i++) {
      expect(pts[i].x).toBeGreaterThanOrEqual(r);
      expect(pts[i].x).toBeLessThanOrEqual(800 - r);
      expect(pts[i].y).toBeGreaterThanOrEqual(r);
      expect(pts[i].y).toBeLessThanOrEqual(600 - r);
      for (let j = i + 1; j < pts.length; j++) expect(Math.hypot(pts[i].x - pts[j].x, pts[i].y - pts[j].y)).toBeGreaterThan(2 * r);
      expect(Math.hypot(pts[i].x - anchors[i].x, pts[i].y - anchors[i].y)).toBeLessThan(160);
    }
    // Außen liegende Anker: der Ballon sitzt weiter außen
    expect(pts[3].x).toBeLessThan(100);
    expect(pts[3].y).toBeLessThan(100);
  });
});
