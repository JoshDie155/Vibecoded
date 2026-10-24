import { describe, expect, it } from 'vitest';
import { decodePaint, encodePaint, paintLeaves, paintStates, remapPaint } from '../src/io/paint';

// Zeichenketten aus einem echten Bambu-Studio-Projekt (Farbbemalung und Fuzzy-Skin-Bemalung)
const REAL = ['4', '8', '0C', '1C', '400A', '400200A', '40446400A200A', '000400344000344043303', '00004A2', '44440AA', '40446405A', '0400606'];

describe('Bemalung (TriangleSelector-Kodierung)', () => {
  it('liest Blätter: Zustand 0–2 direkt, ab 3 mit zusätzlichem Halbbyte', () => {
    expect(decodePaint('4')).toEqual({ state: 1 });
    expect(decodePaint('8')).toEqual({ state: 2 });
    expect(decodePaint('0C')).toEqual({ state: 3 });
    expect(decodePaint('1C')).toEqual({ state: 4 });
    // 15 + 15 + 2 + 3 = 35: zwei volle Halbbytes vor dem Rest
    expect(decodePaint('2FFC')).toEqual({ state: 35 });
    expect(encodePaint({ state: 35 })).toBe('2FFC');
    expect(encodePaint({ state: 0 })).toBe('');
  });

  it('echte Zeichenketten bleiben beim Hin- und Rückweg unverändert', () => {
    for (const code of REAL) expect(encodePaint(decodePaint(code)!)).toBe(code);
  });

  it('zerlegte Dreiecke: Kinder in umgekehrter Reihenfolge gespeichert', () => {
    // '400A': Wurzel 0xA = zwei geteilte Seiten, besondere Seite 2; Kinder (gelesen: 0, 0, 4) → [1, 0, 0]
    expect(decodePaint('400A')).toEqual({ split: 2, special: 2, children: [{ state: 1 }, { state: 0 }, { state: 0 }] });
  });

  it('ungültiger Inhalt ergibt null statt eines Fehlers', () => {
    expect(decodePaint('A')).toBeNull(); // zerlegt, aber keine Kinder
    expect(decodePaint('XYZ')).toBeNull();
  });

  it('Zustände sammeln und umnummerieren', () => {
    expect([...paintStates('40446400A200A')].sort()).toEqual([0, 1]);
    // 1 → 5 (ab 3 mit zusätzlichem Halbbyte), Aufteilung bleibt
    const moved = remapPaint('40446400A200A', (s) => (s ? 5 : 0));
    expect([...paintStates(moved)].sort()).toEqual([0, 5]);
    expect(remapPaint(moved, (s) => (s ? 1 : 0))).toBe('40446400A200A');
    expect(remapPaint('1C', () => 0)).toBe('');
  });

  it('Teildreiecke decken das Dreieck lückenlos ab, Ecken wie im Slicer', () => {
    const area = (a: number[], b: number[], c: number[]) => Math.abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) / 2;
    for (const code of REAL) {
      let sum = 0;
      paintLeaves(decodePaint(code)!, [0, 0, 0], [4, 0, 0], [0, 4, 0], (_, a, b, c) => { sum += area(a, b, c); });
      expect(sum).toBeCloseTo(8, 9);
    }
    // Eine geteilte Seite, besondere Seite 0: Mitte der Seite B–C, Kinder (A, B, M) und (M, C, A)
    const leaves: number[][][] = [];
    paintLeaves({ split: 1, special: 0, children: [{ state: 1 }, { state: 2 }] }, [0, 0, 0], [4, 0, 0], [0, 4, 0], (_, a, b, c) => leaves.push([a, b, c]));
    expect(leaves).toEqual([[[0, 0, 0], [4, 0, 0], [2, 2, 0]], [[2, 2, 0], [0, 4, 0], [0, 0, 0]]]);
  });
});
