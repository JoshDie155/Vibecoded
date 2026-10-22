/**
 * Genaues Anlegen nach dem automatischen Einpassen: Das Abtasten findet die richtige Stellung, ist aber
 * nur so fein wie seine Schrittweite. Hier werden die Kontaktflächen exakt aufeinandergezogen –
 * Punkt-zu-Ebene-ICP, das sich aber nur entlang der noch freien Bewegungen bewegen darf. So bleiben
 * die Bedingungen der Verbindung (z. B. koaxial) erhalten, und ein Zapfen sitzt genau auf Anschlag.
 */
import { Vector3 } from 'three';
import type { MeshBVH } from 'three-mesh-bvh';
import { type Rigid, type Vec3, apply, compose, cross, dot, normalize, rotate, sub } from './align';
import type { Sample } from './icp';
import { type FreeMotion, motionTransform } from './motion';

export interface SettleTarget {
  bvh: MeshBVH;
  /** Nicht-indizierte Positionen des festen Teils (lokal), für die Flächennormalen. */
  positions: Float32Array;
}

export interface SettleOptions {
  /** Nur Punktpaare näher als dieser Abstand zählen als Kontakt. */
  maxPairDist: number;
  iterations?: number;
}

/**
 * @param samples Oberflächenpunkte des beweglichen Teils (lokal)
 * @param T       Lage des beweglichen Teils im Koordinatensystem des festen Teils
 * @param free    freie Bewegungen (im selben Koordinatensystem)
 * @returns verbesserte Lage und mittlerer Restabstand der Kontaktpaare, oder null ohne Kontakt
 */
export function settleContact(
  samples: Sample[],
  T: Rigid,
  target: SettleTarget,
  free: FreeMotion[],
  opts: SettleOptions,
): { T: Rigid; rms: number; pairs: number } | null {
  const k = free.length;
  if (!k || !samples.length) return null;
  const pos = target.positions;
  const hit = { point: new Vector3(), distance: 0, faceIndex: 0 };
  const q = new Vector3();
  let cur = T;
  let rms = 0;
  let pairs = 0;
  for (let it = 0; it < (opts.iterations ?? 12); it++) {
    // Normalgleichungen JᵀJ δ = −Jᵀr mit r = n · (w − c) je Kontaktpaar
    const A = Array.from({ length: k }, () => new Array(k).fill(0));
    const b = new Array(k).fill(0);
    let sum = 0;
    pairs = 0;
    for (const s of samples) {
      const w = apply(cur, s.p);
      q.set(w[0], w[1], w[2]);
      if (!target.bvh.closestPointToPoint(q, hit, 0, opts.maxPairDist) || hit.distance > opts.maxPairDist) continue;
      const n = faceNormal(pos, hit.faceIndex);
      // Nur Flächen, die gegeneinander zeigen
      if (dot(rotate(cur.R, s.n), n) > -0.9) continue;
      const r = dot(n, sub(w, [hit.point.x, hit.point.y, hit.point.z]));
      const J = free.map((m) => (m.kind === 'translate' ? dot(n, normalize(m.axis)) : dot(n, cross(normalize(m.axis), sub(w, m.point)))));
      for (let a = 0; a < k; a++) {
        b[a] -= J[a] * r;
        for (let c = 0; c < k; c++) A[a][c] += J[a] * J[c];
      }
      sum += r * r;
      pairs++;
    }
    if (pairs < 3) return it === 0 ? null : { T: cur, rms, pairs };
    rms = Math.sqrt(sum / pairs);
    // Leichte Dämpfung: Richtungen ohne Information (z. B. Verschieben in der Kontaktebene) bleiben unverändert
    const trace = A.reduce((s, row, i) => s + row[i], 0) || 1;
    for (let a = 0; a < k; a++) A[a][a] += 1e-6 * trace;
    const delta = solve(A, b);
    if (!delta) break;
    for (let a = 0; a < k; a++) if (delta[a]) cur = compose(motionTransform(free[a], delta[a]), cur);
    if (Math.max(...delta.map(Math.abs)) < 1e-7) break;
  }
  return { T: cur, rms, pairs };
}

function faceNormal(p: Float32Array, f: number): Vec3 {
  const i = f * 9;
  const u: Vec3 = [p[i + 3] - p[i], p[i + 4] - p[i + 1], p[i + 5] - p[i + 2]];
  const v: Vec3 = [p[i + 6] - p[i], p[i + 7] - p[i + 1], p[i + 8] - p[i + 2]];
  return normalize(cross(u, v));
}

/** Kleines lineares Gleichungssystem (Gauß mit Pivotsuche). */
function solve(A: number[][], b: number[]): number[] | null {
  const n = b.length;
  const M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-300) return null;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let x = c; x <= n; x++) M[r][x] -= f * M[c][x];
    }
  }
  return M.map((row, i) => row[n] / M[i][i]);
}
