/**
 * Freiheitsgrade, die nach dem Verbinden noch offen sind (z. B. Drehung um eine Zylinderachse
 * oder Verschieben in einer Ebene). Alle Angaben in Weltkoordinaten.
 */
import { type Rigid, type Vec3, compose, normalize, rotationAbout, scale } from './align';

export type FreeMotion =
  | { kind: 'rotate'; point: Vec3; axis: Vec3 }
  | { kind: 'translate'; axis: Vec3 };

/** Bewegung um `amount` (Radiant bei Drehung, Längeneinheit bei Verschiebung). */
export function motionTransform(m: FreeMotion, amount: number): Rigid {
  if (m.kind === 'rotate') return rotationAbout(m.point, m.axis, amount);
  return { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: scale(normalize(m.axis), amount) };
}

/** Wendet die Bewegung nach T an (in Weltkoordinaten). */
export function applyMotion(T: Rigid, m: FreeMotion, amount: number): Rigid {
  return compose(motionTransform(m, amount), T);
}
