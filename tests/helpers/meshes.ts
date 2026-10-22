/**
 * Testnetze als nicht-indizierte Positionslisten (9 Werte pro Dreieck), Z zeigt nach oben,
 * Normalen nach außen (Umlaufsinn gegen den Uhrzeigersinn von außen gesehen).
 */
import * as THREE from 'three';

export function toPositions(g: THREE.BufferGeometry): Float32Array {
  const ni = g.index ? g.toNonIndexed() : g;
  return new Float32Array(ni.getAttribute('position').array as ArrayLike<number>);
}

export function geometryFrom(positions: Float32Array): THREE.BufferGeometry {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  g.computeVertexNormals();
  g.computeBoundingBox();
  g.computeBoundingSphere();
  return g;
}

/** Quader, Unterseite bei z = 0, in XY zentriert. */
export function box(w: number, d: number, h: number): Float32Array {
  const g = new THREE.BoxGeometry(w, d, h);
  g.translate(0, 0, h / 2);
  return toPositions(g);
}

function extrude(shape: THREE.Shape, h: number, segments: number): Float32Array {
  const g = new THREE.ExtrudeGeometry(shape, { depth: h, bevelEnabled: false, curveSegments: segments });
  return toPositions(g);
}

/** Massiver Zylinder (Zapfen) um die Z-Achse, Unterseite bei z = 0. */
export function cylinder(r: number, h: number, segments = 48): Float32Array {
  const s = new THREE.Shape();
  s.absarc(0, 0, r, 0, Math.PI * 2, false);
  return extrude(s, h, segments);
}

/** Ring (Scheibe mit zentrischer Bohrung) um die Z-Achse, Unterseite bei z = 0. */
export function ring(rOuter: number, rInner: number, h: number, segments = 48): Float32Array {
  const s = new THREE.Shape();
  s.absarc(0, 0, rOuter, 0, Math.PI * 2, false);
  const hole = new THREE.Path();
  hole.absarc(0, 0, rInner, 0, Math.PI * 2, true);
  s.holes.push(hole);
  return extrude(s, h, segments);
}

/** D-Profil: Kreis mit Radius r, bei x = flat abgeflacht (flat < r). */
function dShape(r: number, flat: number, path: THREE.Path, clockwise: boolean) {
  const a = Math.acos(flat / r);
  if (!clockwise) {
    path.absarc(0, 0, r, a, Math.PI * 2 - a, false);
    path.lineTo(flat, r * Math.sin(a));
  } else {
    path.moveTo(flat, r * Math.sin(a));
    path.absarc(0, 0, r, a, Math.PI * 2 - a, false);
    path.lineTo(flat, r * Math.sin(a));
    // Umlaufsinn umdrehen
    const pts = path.getPoints();
    path.curves = [];
    pts.reverse();
    path.setFromPoints(pts);
  }
}

/** Zapfen mit D-Profil (verdrehsicher), Unterseite bei z = 0. */
export function dPeg(r: number, flat: number, h: number, segments = 48): Float32Array {
  const s = new THREE.Shape();
  dShape(r, flat, s, false);
  return extrude(s, h, segments);
}

/** Scheibe mit D-förmiger Bohrung, Unterseite bei z = 0. */
export function dRing(rOuter: number, r: number, flat: number, h: number, segments = 48): Float32Array {
  const s = new THREE.Shape();
  s.absarc(0, 0, rOuter, 0, Math.PI * 2, false);
  const hole = new THREE.Path();
  dShape(r, flat, hole, true);
  s.holes.push(hole);
  return extrude(s, h, segments);
}

/** Wendet eine Matrix auf eine Positionsliste an (Kopie). */
export function transformPositions(p: Float32Array, m: THREE.Matrix4): Float32Array {
  const out = new Float32Array(p.length);
  const v = new THREE.Vector3();
  for (let i = 0; i < p.length; i += 3) {
    v.set(p[i], p[i + 1], p[i + 2]).applyMatrix4(m);
    out[i] = v.x;
    out[i + 1] = v.y;
    out[i + 2] = v.z;
  }
  return out;
}

/** Fügt mehrere Positionslisten zusammen. */
export function concat(...parts: Float32Array[]): Float32Array {
  const out = new Float32Array(parts.reduce((s, p) => s + p.length, 0));
  let o = 0;
  for (const p of parts) {
    out.set(p, o);
    o += p.length;
  }
  return out;
}
