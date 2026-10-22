/**
 * Genaues Anlegen nach dem automatischen Einpassen: Das Abtasten findet die richtige Stellung, ist aber
 * nur so fein wie seine Schrittweite. Hier werden die Kontaktflächen exakt aufeinandergezogen –
 * Punkt-zu-Ebene-ICP, das sich aber nur entlang der noch freien Bewegungen bewegen darf. So bleiben
 * die Bedingungen der Verbindung (z. B. koaxial) erhalten, und ein Zapfen sitzt genau auf Anschlag.
 */
import { Vector3 } from 'three';
import type { MeshBVH } from 'three-mesh-bvh';
import { type Rigid, apply, compose, cross, dot, normalize, rotate, sub } from './align';
import type { Sample } from './icp';
import { type FreeMotion, motionTransform } from './motion';
import { bvhTriangleNormal } from './bvhtri';

export interface SettleTarget {
  /** BVH des festen Teils; Lage des beweglichen Teils wird im Koordinatensystem des festen Teils angegeben. */
  bvh: MeshBVH;
}

export interface SettleOptions {
  /** Nur Punktpaare näher als dieser Abstand zählen als Kontakt. */
  maxPairDist: number;
  iterations?: number;
  /**
   * Größte Gesamtdrehung (Radiant, Standard 0: Drehungen bleiben, wie sie sind). Die Drehung hat das
   * Abtasten schon bestimmt; mit Spiel (z. B. D-Zapfen in D-Bohrung) würde Punkt-zu-Ebene sonst einseitig
   * „in Kontakt drehen".
   */
  maxRotation?: number;
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
  const hit = { point: new Vector3(), distance: 0, faceIndex: 0 };
  const q = new Vector3();
  let cur = T;
  let rms = 0;
  let pairs = 0;
  const maxRot = opts.maxRotation ?? 0;
  /** Bisher entlang jeder Bewegung zurückgelegt. */
  const moved = new Array(k).fill(0);
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
      const n = bvhTriangleNormal(target.bvh, hit.faceIndex);
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
    for (let a = 0; a < k; a++) {
      // Drehungen nur im kleinen Rahmen nachführen, Verschiebungen je Schritt höchstens um maxPairDist
      const lim = free[a].kind === 'rotate' ? maxRot : Infinity;
      const step = free[a].kind === 'rotate'
        ? Math.max(-lim - moved[a], Math.min(lim - moved[a], delta[a]))
        : Math.max(-opts.maxPairDist, Math.min(opts.maxPairDist, delta[a]));
      delta[a] = step;
      moved[a] += step;
      if (step) cur = compose(motionTransform(free[a], step), cur);
    }
    if (Math.max(...delta.map(Math.abs)) < 1e-7) break;
  }
  return { T: cur, rms, pairs };
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
