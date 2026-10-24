import { describe, expect, it } from 'vitest';
import { clearanceLabel, makeToleranceTest } from '../src/io/testpiece';
import { meshHealth } from '../src/math/repair';

const bounds = (p: Float32Array) => {
  const mn = [Infinity, Infinity, Infinity], mx = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < p.length; i += 3) for (let k = 0; k < 3; k++) { mn[k] = Math.min(mn[k], p[i + k]); mx[k] = Math.max(mx[k], p[i + k]); }
  return { mn, mx };
};

describe('Toleranz-Testkörper', () => {
  it('Platte mit sechs Löchern und erhabener Schrift, Zapfen mit Griff – beide geschlossen', async () => {
    const r = await makeToleranceTest({ diameter: 10, clearances: [0.1, 0.15, 0.2, 0.25, 0.3, 0.4] });
    const hp = meshHealth(r.plate);
    expect(hp.closed).toBe(true);
    expect(meshHealth(r.pin).closed).toBe(true);
    const b = bounds(r.plate);
    // 3 × 2 Löcher, Schrift 0,6 mm über der Platte
    expect(b.mx[0] - b.mn[0]).toBeCloseTo(3 * (10 + 0.8 + 8) + 4, 3);
    expect(b.mx[2]).toBeCloseTo(6 + 0.6, 4);
    const pin = bounds(r.pin);
    expect(pin.mx[0] - pin.mn[0]).toBeCloseTo(16, 2);
    // Griff 2 mm, Zapfen 10 mm (länger als die Platte dick ist), Fase 0,6 mm
    expect(pin.mx[2]).toBeCloseTo(12.6, 3);
  });

  it('Beschriftung mit Komma, ohne überflüssige Nullen', () => {
    expect(clearanceLabel(0.15)).toBe('0,15');
    expect(clearanceLabel(0.2)).toBe('0,2');
  });
});
