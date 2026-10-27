import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { MATERIALS, massProps, printEstimate, printedCentroid } from '../src/math/mass';
import { box, concat, cylinder, ring, transformPositions } from './helpers/meshes';

const PLA = { density: 1.24, infill: 0.15, walls: 2, price: 20 };

describe('Masse und Schwerpunkt', () => {
  it('Quader: Volumen, Oberfläche, Schwerpunkt – auch weit vom Ursprung', () => {
    const far = transformPositions(box(20, 10, 5), new THREE.Matrix4().makeTranslation(1000, -2000, 3));
    const m = massProps(far);
    expect(m.volume).toBeCloseTo(1000, 6);
    expect(m.area).toBeCloseTo(2 * (200 + 100 + 50), 6);
    expect(m.centroid[0]).toBeCloseTo(1000, 6);
    expect(m.centroid[1]).toBeCloseTo(-2000, 6);
    expect(m.centroid[2]).toBeCloseTo(5.5, 6);
  });

  it('Ring: Bohrung zählt nicht mit; nach innen zeigende Normalen ändern nichts', () => {
    const m = massProps(ring(10, 3, 4, 256));
    expect(m.volume).toBeCloseTo(Math.PI * (100 - 9) * 4, -1);
    const flipped = transformPositions(ring(10, 3, 4, 256), new THREE.Matrix4().makeScale(1, 1, -1));
    expect(massProps(flipped).volume).toBeCloseTo(m.volume, 6);
  });

  it('Schwerpunkt eines L aus zwei Quadern', () => {
    const L = concat(box(10, 10, 10), transformPositions(box(10, 10, 10), new THREE.Matrix4().makeTranslation(10, 0, 0)), transformPositions(box(10, 10, 10), new THREE.Matrix4().makeTranslation(0, 0, 10)));
    const m = massProps(L);
    expect(m.volume).toBeCloseTo(3000, 6);
    expect(m.centroid[0]).toBeCloseTo(10 / 3, 6);
    expect(m.centroid[2]).toBeCloseTo(5 + 10 / 3, 6);
  });
});

describe('Druckschätzung', () => {
  it('20-mm-Würfel in PLA: Hülle wie ein um 0,84 mm versetzter Würfel, ~3,4 g', () => {
    const m = massProps(box(20, 20, 20));
    const e = printEstimate(m.volume, m.area, PLA, [20, 20, 20]);
    const inner = (20 - 2 * 0.84) ** 3;
    expect(e.shell).toBeCloseTo(8000 - inner, 6);
    expect(e.plastic).toBeCloseTo(8000 - inner + 0.15 * inner, 6);
    expect(e.grams).toBeGreaterThan(3.2);
    expect(e.grams).toBeLessThan(3.7);
    // 1,75 mm: 2,405 mm² Querschnitt
    expect(e.meters).toBeCloseTo(e.plastic / (Math.PI * 0.875 ** 2) / 1000, 9);
    expect(e.cost).toBeCloseTo((e.grams / 1000) * 20, 9);
  });

  it('100 % Füllung: das ganze Volumen; dünne Teile sind ganz Hülle', () => {
    const m = massProps(box(20, 20, 20));
    expect(printEstimate(m.volume, m.area, { ...PLA, infill: 1 }).plastic).toBeCloseTo(8000, 6);
    const thin = massProps(box(40, 40, 1));
    const e = printEstimate(thin.volume, thin.area, PLA, [40, 40, 1]);
    expect(e.plastic).toBeCloseTo(1600, 6);
    expect(e.shell).toBeCloseTo(1600, 6);
  });

  it('Material ändert nur das Gewicht, nicht das Volumen', () => {
    const m = massProps(cylinder(10, 20, 96));
    const petg = MATERIALS.find((x) => x.id === 'petg')!;
    const a = printEstimate(m.volume, m.area, PLA);
    const b = printEstimate(m.volume, m.area, { ...PLA, density: petg.density });
    expect(b.plastic).toBeCloseTo(a.plastic, 9);
    expect(b.grams / a.grams).toBeCloseTo(petg.density / 1.24, 9);
  });

  it('Schwerpunkt des Drucks: bei wenig Füllung näher an der Oberfläche', () => {
    // Zylinder mit dicker Scheibe unten: Oberfläche und Volumen haben verschiedene Schwerpunkte
    const shape = concat(cylinder(20, 2, 96), cylinder(3, 40, 48));
    const m = massProps(shape);
    const hollow = printedCentroid(m, printEstimate(m.volume, m.area, { ...PLA, infill: 0 }));
    const full = printedCentroid(m, printEstimate(m.volume, m.area, { ...PLA, infill: 1 }));
    expect(full[2]).toBeCloseTo(m.centroid[2], 9);
    expect(Math.abs(hollow[2] - m.surfaceCentroid[2])).toBeLessThan(Math.abs(full[2] - m.surfaceCentroid[2]));
  });
});
