import { describe, expect, it } from 'vitest';
import { type Rigid, apply, axisAngle, dist, rotate } from '../src/math/align';
import { type Marker, applyAdjustments, solveJoint } from '../src/math/match';

const T0: Rigid = { R: axisAngle([0.3, -1, 0.5], 2.0), t: [50, 20, -10] };
const move = (m: Marker): Marker => ({ p: apply(T0, m.p), n: rotate(T0.R, m.n) });
/** Gegenstück auf dem festen Teil: gleicher Ort, Normale entgegengesetzt. */
const counterpart = (m: Marker): Marker => ({ p: m.p, n: [-m.n[0], -m.n[1], -m.n[2]] });

// Kontaktfläche: Rechteck mit unregelmäßigen Ecken (Normale +Z am beweglichen Teil)
const contact: Marker[] = [
  { p: [0, 0, 0], n: [0, 0, 1] },
  { p: [30, 0, 0], n: [0, 0, 1] },
  { p: [30, 12, 0], n: [0, 0, 1] },
  { p: [0, 20, 0], n: [0, 0, 1] },
  { p: [14, 5, 0], n: [0, 0, 1] },
];

function setup(count: number, shuffle: number[]) {
  // src = Markierungen am beweglichen Teil (verschoben/gedreht), dst = am festen Teil
  const dstAll = contact.slice(0, count).map(counterpart);
  const srcAll = contact.slice(0, count).map((m) => move(m));
  const src = shuffle.map((i) => srcAll[i]);
  return { src, dst: dstAll };
}

describe('solveJoint', () => {
  it('findet die richtige Zuordnung bei vertauschter Reihenfolge', () => {
    const shuffle = [3, 0, 4, 2, 1];
    const { src, dst } = setup(5, shuffle);
    const r = solveJoint({ src, dst, ordered: false, normals: 'auto' });
    expect(r.error).toBeUndefined();
    expect(r.solutions[0].perm).toEqual(shuffle);
    expect(r.solutions[0].rmsd).toBeLessThan(1e-6);
    expect(r.ambiguous).toBe(false);
  });

  it('richtet mit drei Punkten exakt aus', () => {
    const { src, dst } = setup(3, [0, 1, 2]);
    const r = solveJoint({ src, dst, ordered: true, normals: 'off' });
    const T = r.solutions[0].T;
    for (const m of src) {
      const back = apply(T, m.p);
      expect(dst.some((d) => dist(d.p, back) < 1e-6)).toBe(true);
    }
    expect(r.solutions[0].freeAxis).toBeUndefined();
  });

  it('nutzt Normalen bei zwei Punkten und meldet keine freie Achse', () => {
    const { src, dst } = setup(2, [0, 1]);
    const r = solveJoint({ src, dst, ordered: true, normals: 'auto' });
    const T = r.solutions[0].T;
    expect(r.solutions[0].rmsd).toBeLessThan(1e-6);
    // Normale des beweglichen Teils muss danach der festen Normale entgegengesetzt sein
    const n = rotate(T.R, src[0].n);
    expect(n[2]).toBeCloseTo(-dst[0].n[2], 6);
    expect(r.solutions[0].freeAxis).toBeUndefined();
  });

  it('meldet bei einem Punkt die Normale als freie Drehachse', () => {
    const { src, dst } = setup(1, [0]);
    const r = solveJoint({ src, dst, ordered: false, normals: 'auto' });
    expect(r.solutions[0].freeAxis).toBeDefined();
    const T = applyAdjustments(r.solutions[0], dst, 90, 0);
    expect(dist(apply(T, src[0].p), dst[0].p)).toBeLessThan(1e-9);
  });

  it('erkennt ungleiche Anzahl', () => {
    const { src, dst } = setup(3, [0, 1, 2]);
    expect(solveJoint({ src: src.slice(0, 2), dst, ordered: false, normals: 'auto' }).error).toBeTruthy();
  });

  it('markiert symmetrische Anordnungen ohne Normalen als mehrdeutig', () => {
    const sq: Marker[] = [
      { p: [0, 0, 0], n: [0, 0, 1] },
      { p: [10, 0, 0], n: [0, 0, 1] },
      { p: [10, 10, 0], n: [0, 0, 1] },
      { p: [0, 10, 0], n: [0, 0, 1] },
    ];
    const r = solveJoint({ src: sq.map(move), dst: sq.map(counterpart), ordered: false, normals: 'off' });
    expect(r.ambiguous).toBe(true);
  });

  it('Einsinken verschiebt entlang der Kontaktnormale in das feste Teil', () => {
    const { src, dst } = setup(3, [0, 1, 2]);
    const r = solveJoint({ src, dst, ordered: true, normals: 'auto' });
    const T = applyAdjustments(r.solutions[0], dst, 0, 0.3);
    const p = apply(T, src[0].p);
    // feste Normale zeigt nach -Z (zum beweglichen Teil hin), Einsinken also nach +Z
    expect(p[2] - dst[0].p[2]).toBeCloseTo(0.3, 9);
  });
});
