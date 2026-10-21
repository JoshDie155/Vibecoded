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
