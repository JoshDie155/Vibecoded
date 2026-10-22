/** Kleines Beispiel: Haus und Dach als getrennte Teile, das Dach liegt verdreht daneben. */
import * as THREE from 'three';

type P = [number, number, number];

/** Baut eine Geometrie aus Polygonen (Eckpunkte gegen den Uhrzeigersinn, von außen gesehen). */
function fromPolygons(polys: P[][]): THREE.BufferGeometry {
  const out: number[] = [];
  for (const poly of polys) for (let i = 1; i < poly.length - 1; i++) out.push(...poly[0], ...poly[i], ...poly[i + 1]);
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(out, 3));
  return g;
}

function finish(g: THREE.BufferGeometry) {
  g.computeVertexNormals();
  g.computeBoundingBox();
  g.computeBoundingSphere();
  return g;
}

export function makeDemo(): { name: string; geom: THREE.BufferGeometry }[] {
  const W = 40;
  const D = 28;
  const H = 24;
  const x0 = -W / 2, x1 = W / 2, y0 = -D / 2, y1 = D / 2;

  const house = fromPolygons([
    [[x0, y0, 0], [x0, y1, 0], [x1, y1, 0], [x1, y0, 0]], // unten
    [[x0, y0, H], [x1, y0, H], [x1, y1, H], [x0, y1, H]], // oben
    [[x0, y0, 0], [x1, y0, 0], [x1, y0, H], [x0, y0, H]],
    [[x1, y0, 0], [x1, y1, 0], [x1, y1, H], [x1, y0, H]],
    [[x1, y1, 0], [x0, y1, 0], [x0, y1, H], [x1, y1, H]],
    [[x0, y1, 0], [x0, y0, 0], [x0, y0, H], [x0, y1, H]],
  ]);

  // Satteldach, First entlang X, mit kleinem Zapfen zum Ausrichten in der Mitte
  const R = 15;
  const roof = fromPolygons([
    [[x0, y0, 0], [x0, y1, 0], [x1, y1, 0], [x1, y0, 0]], // Unterseite (liegt auf dem Haus)
    [[x0, y0, 0], [x1, y0, 0], [x1, 0, R], [x0, 0, R]], // Dachfläche vorne
    [[x1, y1, 0], [x0, y1, 0], [x0, 0, R], [x1, 0, R]], // Dachfläche hinten
    [[x0, y1, 0], [x0, y0, 0], [x0, 0, R]], // Giebel links
    [[x1, y0, 0], [x1, y1, 0], [x1, 0, R]], // Giebel rechts
  ]);
  // Dach verdreht/auf den Kopf gestellt ablegen, wie es oft aus dem Slicer-Layout kommt
  roof.rotateX(Math.PI);
  roof.rotateZ(0.7);

  return [
    { name: 'Haus', geom: finish(house) },
    { name: 'Dach', geom: finish(roof) },
  ];
}

function extrude(shape: THREE.Shape, depth: number, segments = 64): THREE.BufferGeometry {
  const g = new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false, curveSegments: segments });
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', (g.index ? g.toNonIndexed() : g).getAttribute('position').clone());
  return out;
}

/** D-Profil: Kreis mit Radius r, bei x = flat abgeflacht. */
function dPath(path: THREE.Path, r: number, flat: number) {
  const a = Math.acos(flat / r);
  path.absarc(0, 0, r, a, Math.PI * 2 - a, false);
  path.lineTo(flat, r * Math.sin(a));
}

function merge(...geoms: THREE.BufferGeometry[]): THREE.BufferGeometry {
  const arrays = geoms.map((g) => g.getAttribute('position').array as Float32Array);
  const out = new Float32Array(arrays.reduce((s, a) => s + a.length, 0));
  let o = 0;
  for (const a of arrays) { out.set(a, o); o += a.length; }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(out, 3));
  return g;
}

/**
 * Rundes, verdrehsicheres Beispiel: Sockel mit D-förmigem Zapfen und eine Kappe mit passender
 * D-Bohrung (0,15 mm Spiel). Die Kappe liegt verdreht und gekippt daneben.
 */
export function makeRoundDemo(): { name: string; geom: THREE.BufferGeometry }[] {
  const disc = new THREE.Shape();
  disc.absarc(0, 0, 22, 0, Math.PI * 2, false);
  const base = extrude(disc, 6);
  const pegShape = new THREE.Shape();
  dPath(pegShape, 6, 3.5);
  const peg = extrude(pegShape, 10);
  peg.translate(0, 0, 6);

  const cap = new THREE.Shape();
  cap.absarc(0, 0, 13, 0, Math.PI * 2, false);
  const hole = new THREE.Path();
  dPath(hole, 6.15, 3.65);
  cap.holes.push(hole);
  const capGeom = extrude(cap, 14);
  capGeom.rotateX(Math.PI / 2);
  capGeom.rotateZ(0.9);

  return [
    { name: 'Sockel', geom: finish(merge(base, peg)) },
    { name: 'Kappe', geom: finish(capGeom) },
  ];
}
