/**
 * Dreiecke so lesen, wie die BVH sie meldet: three-mesh-bvh sortiert beim Aufbau den Index der
 * Geometrie um (bei nicht indizierten Netzen legt sie einen an). `faceIndex` aus Strahl- und
 * Abstandsabfragen bezieht sich auf diesen umsortierten Index, nicht auf die Reihenfolge der
 * Dreiecke im Positions-Array.
 */
import type { MeshBVH } from 'three-mesh-bvh';
import type { Vec3 } from './align';

/** Nummern der drei Eckpunkte von Dreieck `face` (wie von der BVH gemeldet). */
export function bvhTriangleVertices(bvh: MeshBVH, face: number): [number, number, number] {
  const index = bvh.geometry.index;
  const f = face * 3;
  return index ? [index.getX(f), index.getX(f + 1), index.getX(f + 2)] : [f, f + 1, f + 2];
}

/** Flächennormale (Einheitsvektor, lokal) von Dreieck `face` laut Umlaufsinn. */
export function bvhTriangleNormal(bvh: MeshBVH, face: number): Vec3 {
  const pos = bvh.geometry.getAttribute('position');
  const [a, b, c] = bvhTriangleVertices(bvh, face);
  const ax = pos.getX(a), ay = pos.getY(a), az = pos.getZ(a);
  const ux = pos.getX(b) - ax, uy = pos.getY(b) - ay, uz = pos.getZ(b) - az;
  const vx = pos.getX(c) - ax, vy = pos.getY(c) - ay, vz = pos.getZ(c) - az;
  const nx = uy * vz - uz * vy;
  const ny = uz * vx - ux * vz;
  const nz = ux * vy - uy * vx;
  const l = Math.hypot(nx, ny, nz);
  return l > 0 ? [nx / l, ny / l, nz / l] : [0, 0, 0];
}
