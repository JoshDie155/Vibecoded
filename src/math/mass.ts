/**
 * Masse eines gedruckten Teils schätzen: Volumen und Oberfläche des Netzes, daraus Wände (Hülle) und
 * Füllung wie im Slicer – Gramm, Meter Filament und Kosten. Dazu der Schwerpunkt (für die Standfestigkeit).
 *
 * Die Hülle ist so dick wie die Wände (Anzahl × Linienbreite); das Innere ist der um diese Dicke nach innen
 * versetzte Körper. Dessen Volumen wird zweimal geschätzt – über Volumen und Oberfläche (genau für Würfel,
 * Kugel, Zylinder) und über die Abmessungen (genau für flache Platten) – und das kleinere genommen.
 * Ohne Stützen, Brim und Spülturm – genau rechnet der Slicer.
 */
import type { Vec3 } from './align';

export interface MassProps {
  /** Volumen (mm³, Betrag – auch bei nach innen zeigenden Normalen). */
  volume: number;
  /** Oberfläche (mm²). */
  area: number;
  /** Schwerpunkt des vollen Körpers. */
  centroid: Vec3;
  /** Schwerpunkt der Oberfläche (dorthin wandert er bei wenig Füllung: die Wände tragen das Gewicht). */
  surfaceCentroid: Vec3;
}

export function massProps(positions: ArrayLike<number>): MassProps {
  const n = Math.floor(positions.length / 9);
  // Bezugspunkt in der Mitte: weniger Rundungsfehler bei weit vom Ursprung liegenden Netzen
  let ox = 0, oy = 0, oz = 0;
  for (let t = 0; t < n; t++) { ox += positions[9 * t]; oy += positions[9 * t + 1]; oz += positions[9 * t + 2]; }
  if (n) { ox /= n; oy /= n; oz /= n; }
  let vol = 0, area = 0;
  let cx = 0, cy = 0, cz = 0;
  let sx = 0, sy = 0, sz = 0;
  for (let t = 0; t < n; t++) {
    const o = 9 * t;
    const ax = positions[o] - ox, ay = positions[o + 1] - oy, az = positions[o + 2] - oz;
    const bx = positions[o + 3] - ox, by = positions[o + 4] - oy, bz = positions[o + 5] - oz;
    const qx = positions[o + 6] - ox, qy = positions[o + 7] - oy, qz = positions[o + 8] - oz;
    // Tetraeder (Bezugspunkt, a, b, c): vorzeichenbehaftetes Volumen
    const v = (ax * (by * qz - bz * qy) - ay * (bx * qz - bz * qx) + az * (bx * qy - by * qx)) / 6;
    vol += v;
    cx += v * (ax + bx + qx) / 4;
    cy += v * (ay + by + qy) / 4;
    cz += v * (az + bz + qz) / 4;
    const ux = bx - ax, uy = by - ay, uz = bz - az;
    const wx = qx - ax, wy = qy - ay, wz = qz - az;
    const a = Math.hypot(uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx) / 2;
    area += a;
    sx += a * (ax + bx + qx) / 3;
    sy += a * (ay + by + qy) / 3;
    sz += a * (az + bz + qz) / 3;
  }
  const surface: Vec3 = area > 0 ? [ox + sx / area, oy + sy / area, oz + sz / area] : [ox, oy, oz];
  const centroid: Vec3 = Math.abs(vol) > 1e-12 ? [ox + cx / vol, oy + cy / vol, oz + cz / vol] : surface;
  return { volume: Math.abs(vol), area, centroid, surfaceCentroid: surface };
}

export interface Material {
  id: string;
  label: string;
  /** g/cm³ */
  density: number;
  /** Üblicher Preis in €/kg (Vorgabe, änderbar). */
  price: number;
}

/** Dichten nach Datenblättern gängiger Filamente; Preise grob (Bambu-Shop, Rollen à 1 kg). */
export const MATERIALS: Material[] = [
  { id: 'pla', label: 'PLA', density: 1.24, price: 20 },
  { id: 'petg', label: 'PETG', density: 1.27, price: 20 },
  { id: 'abs', label: 'ABS', density: 1.04, price: 20 },
  { id: 'asa', label: 'ASA', density: 1.07, price: 25 },
  { id: 'tpu', label: 'TPU', density: 1.21, price: 35 },
  { id: 'pc', label: 'PC', density: 1.2, price: 35 },
  { id: 'pa', label: 'PA (Nylon)', density: 1.14, price: 45 },
  { id: 'placf', label: 'PLA-CF', density: 1.22, price: 35 },
  { id: 'petgcf', label: 'PETG-CF', density: 1.29, price: 35 },
];

export interface PrintSettings {
  /** g/cm³ */
  density: number;
  /** Füllung 0…1 (Bambu Studio: 15 %). */
  infill: number;
  /** Anzahl der Wände (Bambu Studio: 2). */
  walls: number;
  /** €/kg */
  price: number;
  /** Linienbreite (mm), Standard 0,42 bei 0,4-mm-Düse. */
  lineWidth?: number;
  /** Filamentdurchmesser (mm). */
  diameter?: number;
}

export interface PrintEstimate {
  /** Gedrucktes Material (mm³). */
  plastic: number;
  /** Davon in Wänden, Boden und Deckel (mm³). */
  shell: number;
  /** Innenraum, der mit der Füllung gedruckt wird (mm³). */
  inner: number;
  grams: number;
  /** Meter Filament */
  meters: number;
  cost: number;
}

/**
 * Hülle und Füllung eines Körpers mit diesem Volumen und dieser Oberfläche.
 * @param dims Abmessungen des Teils (umschließender Quader), damit dünne Platten ganz massiv werden
 */
export function printEstimate(volume: number, area: number, s: PrintSettings, dims?: Vec3): PrintEstimate {
  const t = Math.max(0, s.walls) * (s.lineWidth ?? 0.42);
  let inner = 0;
  if (volume > 0 && area > 0) {
    // Innenkörper: um t nach innen versetzt (r = 3V/A ist der „Radius“ des Körpers)
    const k = 1 - (area * t) / (3 * volume);
    inner = k > 0 ? volume * k ** 3 : 0;
    // Über die Abmessungen: jede Richtung schrumpft um 2t
    if (dims) inner = Math.min(inner, volume * dims.reduce((f, d) => f * Math.max(0, 1 - (2 * t) / Math.max(d, 1e-9)), 1));
  }
  const shell = volume - inner;
  const plastic = shell + Math.min(1, Math.max(0, s.infill)) * inner;
  const grams = (plastic / 1000) * s.density;
  const d = s.diameter ?? 1.75;
  const meters = plastic / (Math.PI * (d / 2) ** 2) / 1000;
  return { plastic, shell, inner, grams, meters, cost: (grams / 1000) * s.price };
}

/**
 * Schwerpunkt des gedruckten Teils: Die Hülle sitzt an der Oberfläche (ihr Schwerpunkt ≈ der der
 * Oberfläche), der Innenraum ist der volle Körper ohne Hülle – davon wird nur der Füllanteil gedruckt.
 * Volle Füllung ergibt genau den Schwerpunkt des Körpers, keine Füllung den der Hülle.
 */
export function printedCentroid(m: MassProps, e: PrintEstimate): Vec3 {
  if (e.plastic <= 0 || m.volume <= 0) return m.centroid;
  const f = e.inner > 0 ? (e.plastic - e.shell) / e.inner : 0;
  const S = e.shell;
  return [0, 1, 2].map((i) => (S * (1 - f) * m.surfaceCentroid[i] + f * m.volume * m.centroid[i]) / e.plastic) as Vec3;
}
