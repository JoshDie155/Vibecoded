import { describe, expect, it } from 'vitest';
import { type Rigid, type Vec3, apply, axisAngle, dist, rmsd, solveRigid } from '../src/math/align';

const T0: Rigid = { R: axisAngle([1, 2, 3], 1.1), t: [10, -4, 7] };
const pts: Vec3[] = [
  [0, 0, 0],
  [20, 0, 0],
  [0, 15, 0],
  [3, 4, 12],
];

describe('solveRigid', () => {
  it('stellt eine bekannte Transformation exakt wieder her', () => {
    const dst = pts.map((p) => apply(T0, p));
    const T = solveRigid(pts, dst);
    expect(rmsd(T, pts, dst)).toBeLessThan(1e-9);
    for (let i = 0; i < 9; i++) expect(T.R[i]).toBeCloseTo(T0.R[i], 9);
  });

  it('liefert bei einem Punkt eine reine Verschiebung', () => {
    const T = solveRigid([[1, 2, 3]], [[4, 6, 8]]);
    expect(T.R).toEqual([1, 0, 0, 0, 1, 0, 0, 0, 1]);
    expect(T.t).toEqual([3, 4, 5]);
  });

  it('ist robust gegen kleines Rauschen', () => {
    const dst = pts.map((p, i) => {
      const q = apply(T0, p);
      return [q[0] + 0.01 * (i % 2 ? 1 : -1), q[1], q[2]] as Vec3;
    });
    const T = solveRigid(pts, dst);
    expect(rmsd(T, pts, dst)).toBeLessThan(0.02);
    expect(dist(apply(T, pts[3]), apply(T0, pts[3]))).toBeLessThan(0.05);
  });
});
