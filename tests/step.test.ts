import { describe, expect, it } from 'vitest';
import { type OcctResult, occtToParts } from '../src/io/step';

// Ergebnis von occt-import-js im Kleinen: Baugruppe mit Platte und Winkel aus zwei Körpern
const quad = { attributes: { position: { array: [0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0] } }, index: { array: [0, 1, 2, 0, 2, 3] } };
const result: OcctResult = {
  success: true,
  root: {
    name: '', meshes: [], children: [{
      name: 'ASM', meshes: [], children: [
        { name: 'PLATE', meshes: [0], children: [] },
        { name: 'BRACKET', meshes: [1, 2], children: [] },
      ],
    }],
  },
  meshes: [
    { name: 'SOLID', color: [0, 1, 0], brep_faces: [], ...quad },
    { name: 'Winkel', brep_faces: [{ first: 0, last: 1, color: [1, 0, 0] }], ...quad },
    { name: 'SOLID', brep_faces: [], ...quad },
  ],
};

describe('STEP-Import (occt-import-js)', () => {
  it('Körper werden Teile: Name aus dem Körper oder der Baugruppe, Farbe, Gruppe je Baugruppe', () => {
    const parts = occtToParts(result);
    expect(parts.map((p) => [p.name, p.color])).toEqual([['PLATE', '#00ff00'], ['Winkel', '#ff0000'], ['BRACKET', undefined]]);
    expect(parts[1].group).toBe(parts[2].group);
    expect(parts[0].group).not.toBe(parts[1].group);
    // Indiziertes Netz → einzelne Dreiecke
    expect(Array.from(parts[0].positions)).toEqual([0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0]);
  });

  it('meldet unlesbare Dateien', () => {
    expect(() => occtToParts({ ...result, success: false })).toThrow('ließ sich nicht lesen');
  });
});
