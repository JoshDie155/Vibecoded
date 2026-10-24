/**
 * Passungshilfe: eine Fläche eines Teils versetzen – Zapfen dünner, Bohrung weiter, ebene Fläche
 * zurücksetzen oder vorziehen. Alle Dreiecksecken an den Eckpunkten der Fläche wandern mit, auch die
 * der Nachbarflächen (z. B. der Deckel eines Zapfens), damit das Netz geschlossen bleibt. Reihenfolge
 * und Anzahl der Dreiecke bleiben gleich – Bemalung und gewählte Flächen gelten weiter.
 */
import type { Vec3 } from './align';
import type { CylinderFeature, PlaneFeature, Topology } from './features';

export interface OffsetResult {
  positions: Float32Array;
  /**
   * Fläche (mm², vorher) der Dreiecke, deren Ausrichtung sich umkehrt: bei zu großem Versatz kippen
   * Nachbarflächen um. Winzige Splitter (z. B. aus der Triangulierung eines Deckels) sind harmlos.
   */
  flippedArea: number;
  /** Verschobene Eckpunkte. */
  moved: number;
}

/** Verschiebt alle Ecken an den Eckpunkten der Dreiecke tris mit move (Position → neue Position). */
function moveVertices(topo: Topology, tris: number[], move: (p: Vec3) => Vec3): OffsetResult {
  const src = topo.positions;
  const out = src.slice();
  const ids = new Set<number>();
  for (const t of tris) for (let k = 0; k < 3; k++) ids.add(topo.vertIds[3 * t + k]);
  const touched = new Set<number>();
  for (let c = 0; c < topo.triCount * 3; c++) {
    if (!ids.has(topo.vertIds[c])) continue;
    const q = move([src[3 * c], src[3 * c + 1], src[3 * c + 2]]);
    out[3 * c] = q[0];
    out[3 * c + 1] = q[1];
    out[3 * c + 2] = q[2];
    touched.add(Math.floor(c / 3));
  }
  // Umgekippte Dreiecke
  let flippedArea = 0;
  for (const t of touched) {
    const n0 = topo.normals.subarray(3 * t, 3 * t + 3);
    if (n0[0] === 0 && n0[1] === 0 && n0[2] === 0) continue;
    const o = 9 * t;
    const ax = out[o + 3] - out[o], ay = out[o + 4] - out[o + 1], az = out[o + 5] - out[o + 2];
    const bx = out[o + 6] - out[o], by = out[o + 7] - out[o + 1], bz = out[o + 8] - out[o + 2];
    const nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
    if (nx * n0[0] + ny * n0[1] + nz * n0[2] <= 0) flippedArea += topo.areas[t];
  }
  return { positions: out, flippedArea, moved: ids.size };
}

/** Zylinder (Zapfen oder Bohrung) auf einen neuen Radius bringen: Eckpunkte radial versetzt. */
export function offsetCylinder(topo: Topology, f: CylinderFeature, radius: number): OffsetResult {
  const d = radius - f.radius;
  const [cx, cy, cz] = f.center;
  const [ax, ay, az] = f.axis;
  return moveVertices(topo, f.tris, (p) => {
    const qx = p[0] - cx, qy = p[1] - cy, qz = p[2] - cz;
    const h = qx * ax + qy * ay + qz * az;
    const rx = qx - h * ax, ry = qy - h * ay, rz = qz - h * az;
    const l = Math.hypot(rx, ry, rz);
    if (l < 1e-9) return p;
    return [p[0] + (rx / l) * d, p[1] + (ry / l) * d, p[2] + (rz / l) * d];
  });
}

/** Ebene Fläche entlang ihrer Außennormale versetzen (negativ = ins Teil hinein, zurücksetzen). */
export function offsetPlane(topo: Topology, f: PlaneFeature, distance: number): OffsetResult {
  const [nx, ny, nz] = f.normal;
  return moveVertices(topo, f.tris, (p) => [p[0] + nx * distance, p[1] + ny * distance, p[2] + nz * distance]);
}
