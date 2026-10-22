import { describe, expect, it } from 'vitest';
import type { Vec3 } from '../src/math/align';
import type { CylinderFeature, Feature, PlaneFeature } from '../src/math/features';
import { describeFeature, planMate } from '../src/math/mate';

function plane(centroid: Vec3, normal: Vec3, size: [number, number] = [40, 28], circle?: PlaneFeature['circle']): PlaneFeature {
  return {
    kind: 'plane', tris: [0], centroid, normal, area: size[0] * size[1], outline: [],
    axes: { u: [1, 0, 0], v: [0, 1, 0], size }, ...(circle ? { circle } : {}),
  };
}

function cyl(center: Vec3, axis: Vec3, radius: number, hole: boolean): CylinderFeature {
  return { kind: 'cylinder', tris: [0], axis, center, radius, length: 10, hole, arc: Math.PI * 2, rms: 0, area: 1 };
}

describe('planMate', () => {
  it('macht aus zwei ebenen Flächen eine Ebenen-Bedingung', () => {
    const plan = planMate([plane([0, 0, 0], [0, 0, -1])], [plane([0, 0, 24], [0, 0, 1])]);
    expect(plan.error).toBeUndefined();
    expect(plan.constraints).toHaveLength(1);
    expect(plan.constraints[0].kind).toBe('plane');
    expect(plan.contactNormal).toEqual([0, 0, 1]);
  });

  it('setzt Kreisflächen zusätzlich mittig aufeinander', () => {
    const c = { center: [0, 0, 0] as Vec3, radius: 10, rms: 0 };
    const plan = planMate([plane([0, 0, 0], [0, 0, -1], [20, 20], c)], [plane([5, 5, 10], [0, 0, 1], [20, 20], { ...c, center: [5, 5, 10] })]);
    expect(plan.constraints.map((k) => k.kind)).toEqual(['plane', 'point']);
    expect(plan.pairs[0].kind).toBe('circle');
  });

  it('paart Zapfen und Bohrung koaxial und meldet das Spiel', () => {
    const plan = planMate([cyl([0, 0, 0], [0, 0, 1], 6, false)], [cyl([1, 2, 3], [1, 0, 0], 6.15, true)]);
    expect(plan.constraints[0].kind).toBe('axis');
    expect(plan.notes.join(' ')).toContain('Spiel 0,15 mm');
  });

  it('warnt, wenn der Zapfen dicker als die Bohrung ist', () => {
    const plan = planMate([cyl([0, 0, 0], [0, 0, 1], 6.2, false)], [cyl([0, 0, 0], [0, 0, 1], 6, true)]);
    expect(plan.notes.join(' ')).toContain('dicker');
  });

  it('ordnet flache und runde Flächen getrennt zu, unabhängig von der Reihenfolge', () => {
    const src: Feature[] = [cyl([0, 0, 0], [0, 0, 1], 5, false), plane([0, 0, 0], [0, 0, -1])];
    const dst: Feature[] = [plane([0, 0, 6], [0, 0, 1]), cyl([0, 0, 6], [0, 0, 1], 5.1, true)];
    const plan = planMate(src, dst);
    expect(plan.error).toBeUndefined();
    expect(plan.pairs).toEqual([
      { kind: 'plane', src: 1, dst: 0 },
      { kind: 'axis', src: 0, dst: 1 },
    ]);
  });

  it('meldet einen Fehler bei ungleichartigen Flächen', () => {
    const plan = planMate([cyl([0, 0, 0], [0, 0, 1], 5, false)], [plane([0, 0, 0], [0, 0, 1])], { src: 'Kappe', dst: 'Sockel' });
    expect(plan.error).toContain('Sockel');
    expect(plan.constraints).toHaveLength(0);
  });
});

describe('describeFeature', () => {
  it('beschreibt Flächen verständlich', () => {
    expect(describeFeature(cyl([0, 0, 0], [0, 0, 1], 6, false))).toBe('Zapfen ⌀ 12,0 mm');
    expect(describeFeature(cyl([0, 0, 0], [0, 0, 1], 3.075, true))).toBe('Bohrung ⌀ 6,15 mm');
    expect(describeFeature(plane([0, 0, 0], [0, 0, 1], [40, 28]))).toBe('Ebene Fläche 40,0 × 28,0 mm');
  });
});
