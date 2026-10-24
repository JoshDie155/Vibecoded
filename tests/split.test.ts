import { describe, expect, it } from 'vitest';
import { planSplit, splitPlanes } from '../src/math/split';

describe('Auf Druckplattengröße zerteilen', () => {
  it('passt es schon, bleibt es ein Stück', () => {
    expect(planSplit([200, 100, 50], [256, 256], 256, 10)).toMatchObject({ counts: [1, 1, 1] });
  });
  it('zu lang: zwei Stücke entlang X, mit Rand', () => {
    const p = planSplit([400, 100, 50], [256, 256], 256, 10);
    expect(p.counts).toEqual([2, 1, 1]);
    expect(p.piece[0]).toBe(200);
    // 500 mm bei 246 mm nutzbar → drei Stücke
    expect(planSplit([500, 100, 50], [256, 256], 256, 10).counts).toEqual([3, 1, 1]);
  });
  it('zu hoch und zu breit', () => {
    expect(planSplit([300, 300, 300], [256, 256], 256, 5).counts).toEqual([2, 2, 2]);
    // Ohne bekannte Höhe nur in der Fläche
    expect(planSplit([300, 100, 900], [256, 256], undefined, 5).counts).toEqual([2, 1, 1]);
  });
  it('nutzt die längere Plattenseite, wenn das Stücke spart', () => {
    // Platte 320 breit, 350 tief: 330 × 600 mm – gedreht passt die Breite (330 ≤ 340), 2 statt 4 Stücke
    const p = planSplit([330, 600, 50], [320, 350], 325, 10);
    expect(p.counts).toEqual([1, 2, 1]);
    expect(p.turned).toBe(true);
    expect(planSplit([600, 330, 50], [320, 350], 325, 10)).toMatchObject({ counts: [2, 1, 1], turned: false });
  });
  it('gleich große Stücke', () => {
    expect(splitPlanes(0, 300, 3)).toEqual([100, 200]);
    expect(splitPlanes(-50, 50, 1)).toEqual([]);
  });
});
