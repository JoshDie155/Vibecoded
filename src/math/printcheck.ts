/**
 * Druckprüfung: Überhänge (Flächen, die zu flach nach unten zeigen und Stützen brauchen) und dünne Wände.
 *
 * Überhang ab Winkel θ (von der Senkrechten gemessen, wie in den Slicern): Eine Fläche hängt über, wenn
 * ihre Normale weniger als 90° − θ von „senkrecht nach unten“ abweicht. Senkrechte Wände sind also nie
 * Überhang, eine waagerechte Unterseite immer – außer sie liegt auf der Druckplatte.
 */
export interface Overhangs {
  /** Nummern der überhängenden Dreiecke. */
  tris: number[];
  /** Ihre Fläche (mm²). */
  area: number;
}

/**
 * @param m Lage des Netzes (4×4 spaltenweise, Weltkoordinaten)
 * @param floorZ Höhe der Druckplatte für dieses Teil (tiefster Punkt seiner Baugruppe)
 */
export function overhangs(positions: Float32Array, m: ArrayLike<number>, floorZ: number, angleDeg: number): Overhangs {
  const limit = Math.cos(((90 - angleDeg) * Math.PI) / 180);
  const tris: number[] = [];
  let area = 0;
  const wz = (i: number) => m[2] * positions[i] + m[6] * positions[i + 1] + m[10] * positions[i + 2] + m[14];
  for (let t = 0; t < positions.length / 9; t++) {
    const o = 9 * t;
    const ax = positions[o + 3] - positions[o], ay = positions[o + 4] - positions[o + 1], az = positions[o + 5] - positions[o + 2];
    const bx = positions[o + 6] - positions[o], by = positions[o + 7] - positions[o + 1], bz = positions[o + 8] - positions[o + 2];
    const nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
    const l = Math.hypot(nx, ny, nz);
    if (!(l > 0)) continue;
    // z-Anteil der Normale in Weltlage
    const z = (m[2] * nx + m[6] * ny + m[10] * nz) / l;
    if (-z < limit) continue;
    // Auf der Platte liegende Unterseiten sind gestützt
    if (Math.max(wz(o), wz(o + 3), wz(o + 6)) < floorZ + 0.2) continue;
    tris.push(t);
    area += l / 2;
  }
  return { tris, area };
}

/** Dreiecke mit gemessener Wandstärke unter min (thickness je Dreieck, NaN/∞ = nicht gemessen). */
export function thinWalls(positions: Float32Array, thickness: Float32Array, min: number): Overhangs {
  const tris: number[] = [];
  let area = 0;
  for (let t = 0; t < thickness.length; t++) {
    if (!(thickness[t] < min)) continue;
    const o = 9 * t;
    const ax = positions[o + 3] - positions[o], ay = positions[o + 4] - positions[o + 1], az = positions[o + 5] - positions[o + 2];
    const bx = positions[o + 6] - positions[o], by = positions[o + 7] - positions[o + 1], bz = positions[o + 8] - positions[o + 2];
    tris.push(t);
    area += Math.hypot(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx) / 2;
  }
  return { tris, area };
}

/** Dreiecke tris aus positions herauskopieren (für die Einfärbung). */
export function pickTriangles(positions: Float32Array, tris: number[]): Float32Array {
  const out = new Float32Array(tris.length * 9);
  tris.forEach((t, i) => out.set(positions.subarray(9 * t, 9 * t + 9), 9 * i));
  return out;
}
