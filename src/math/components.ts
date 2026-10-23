/**
 * Lose Stücke eines Netzes trennen: Viele Downloads stecken alle Teile einer Druckplatte in ein
 * einziges Netz (eine STL, ein 3MF-Objekt). Zusammenhängende Dreiecke bilden ein Stück; Stücke,
 * die sich berühren oder ineinander stecken (z. B. Augen in einem Kopf), bleiben ein Teil. Stücke,
 * die nur nebeneinander liegen – auch dicht an dicht –, werden getrennt.
 */
import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { buildTopology } from './features';

export interface SplitOptions {
  /** Abstand, bis zu dem sich zwei Stücke berühren (Standard: 1e-4 der Diagonale, mindestens 0,01 mm). */
  touchTol?: number;
  /** Bei mehr losen Stücken wird nicht getrennt (stark zerstückelte Netze). */
  maxPieces?: number;
}

interface Piece {
  tris: number[];
  min: THREE.Vector3;
  max: THREE.Vector3;
  geom?: THREE.BufferGeometry;
  bvh?: MeshBVH;
}

function find(parent: Int32Array, x: number): number {
  while (parent[x] !== x) {
    parent[x] = parent[parent[x]];
    x = parent[x];
  }
  return x;
}

function geometryOf(positions: Float32Array, tris: number[]): THREE.BufferGeometry {
  const out = new Float32Array(tris.length * 9);
  tris.forEach((t, k) => out.set(positions.subarray(t * 9, t * 9 + 9), k * 9));
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(out, 3));
  return g;
}

/** Liegt der Punkt im Inneren des geschlossenen Netzes (Strahl in mehrere Richtungen, ungerade Trefferzahl)? */
function inside(bvh: MeshBVH, p: THREE.Vector3): boolean {
  let votes = 0;
  for (const d of [new THREE.Vector3(1, 0.13, 0.07), new THREE.Vector3(-0.11, 1, 0.05), new THREE.Vector3(0.09, -0.06, 1)]) {
    const ray = new THREE.Ray(p, d.normalize());
    const hits = bvh.raycast(ray, THREE.DoubleSide);
    if (hits.length % 2 === 1) votes++;
  }
  return votes >= 2;
}

/**
 * Trennt die losen Stücke eines Netzes (nicht indizierte Dreiecke).
 * @returns ein Positions-Array je Teil; genau eines, wenn nichts zu trennen ist
 */
export function splitLoosePieces(positions: Float32Array, opts: SplitOptions = {}): Float32Array[] {
  const triCount = positions.length / 9;
  if (triCount < 2) return [positions];
  const topo = buildTopology(positions);
  // Zusammenhang über gemeinsame (verschweißte) Eckpunkte – robuster als über Kanten
  const parent = new Int32Array(topo.vertCount);
  for (let i = 0; i < parent.length; i++) parent[i] = i;
  const union = (a: number, b: number) => {
    const ra = find(parent, a);
    const rb = find(parent, b);
    if (ra !== rb) parent[ra] = rb;
  };
  for (let t = 0; t < triCount; t++) {
    union(topo.vertIds[3 * t], topo.vertIds[3 * t + 1]);
    union(topo.vertIds[3 * t + 1], topo.vertIds[3 * t + 2]);
  }
  const byRoot = new Map<number, Piece>();
  for (let t = 0; t < triCount; t++) {
    const r = find(parent, topo.vertIds[3 * t]);
    let piece = byRoot.get(r);
    if (!piece) {
      piece = { tris: [], min: new THREE.Vector3(Infinity, Infinity, Infinity), max: new THREE.Vector3(-Infinity, -Infinity, -Infinity) };
      byRoot.set(r, piece);
    }
    piece.tris.push(t);
    for (let k = 0; k < 3; k++) {
      const o = t * 9 + k * 3;
      piece.min.min(new THREE.Vector3(positions[o], positions[o + 1], positions[o + 2]));
      piece.max.max(new THREE.Vector3(positions[o], positions[o + 1], positions[o + 2]));
    }
  }
  const pieces = [...byRoot.values()];
  if (pieces.length < 2 || pieces.length > (opts.maxPieces ?? 2000)) return [positions];

  const tol = opts.touchTol ?? Math.max(0.01, topo.diag * 1e-4);
  const group = new Int32Array(pieces.length);
  for (let i = 0; i < group.length; i++) group[i] = i;
  const bvhOf = (p: Piece) => {
    p.geom ??= geometryOf(positions, p.tris);
    if (!p.bvh) {
      p.bvh = new MeshBVH(p.geom);
      // Für Abstandsabfragen zwischen zwei Stücken muss die Gegenseite ihre BVH mitbringen
      p.geom.boundsTree = p.bvh;
    }
    return p.bvh;
  };
  const size = (p: Piece) => p.max.clone().sub(p.min).lengthSq();
  const t1 = { point: new THREE.Vector3(), distance: 0, faceIndex: 0 };
  const t2 = { point: new THREE.Vector3(), distance: 0, faceIndex: 0 };
  const ident = new THREE.Matrix4();
  for (let i = 0; i < pieces.length; i++) {
    for (let k = i + 1; k < pieces.length; k++) {
      const a = pieces[i];
      const b = pieces[k];
      // Umschließende Quader müssen sich (fast) überschneiden
      if (a.min.x > b.max.x + tol || b.min.x > a.max.x + tol || a.min.y > b.max.y + tol || b.min.y > a.max.y + tol ||
        a.min.z > b.max.z + tol || b.min.z > a.max.z + tol) continue;
      if (find(group, i) === find(group, k)) continue;
      // Berühren sich die Oberflächen?
      let together = !!bvhOf(a).closestPointToGeometry(bvhOf(b).geometry, ident, t1, t2, 0, tol) && t1.distance <= tol;
      // Steckt das kleinere Stück im größeren (z. B. ein Auge im Kopf)?
      if (!together) {
        const [small, big] = size(a) < size(b) ? [a, b] : [b, a];
        const o = small.tris[0] * 9;
        together = inside(bvhOf(big), new THREE.Vector3(positions[o], positions[o + 1], positions[o + 2]));
      }
      if (together) group[find(group, i)] = find(group, k);
    }
  }
  const clusters = new Map<number, number[]>();
  pieces.forEach((p, i) => {
    const r = find(group, i);
    const list = clusters.get(r);
    if (list) list.push(...p.tris);
    else clusters.set(r, [...p.tris]);
  });
  for (const p of pieces) p.geom?.dispose();
  if (clusters.size < 2) return [positions];
  return [...clusters.values()]
    .map((tris) => tris.sort((x, y) => x - y))
    .sort((x, y) => x[0] - y[0])
    .map((tris) => geometryOf(positions, tris).getAttribute('position').array as Float32Array);
}
