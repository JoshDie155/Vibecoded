import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import { type Vec3, dist, dot, len } from '../src/math/align';
import {
  type CylinderFeature, type Feature, type PlaneFeature, type SurfaceFeature, type Topology, buildTopology, detectFeature,
} from '../src/math/features';
import { box, concat, cylinder, dPeg, dRing, ring, toPositions, transformPositions } from './helpers/meshes';

const toV = (v: THREE.Vector3): Vec3 => [v.x, v.y, v.z];

/** Erstes Dreieck, dessen Normale und Schwerpunkt die Bedingung erfüllen. */
function findTri(topo: Topology, pred: (n: Vec3, c: Vec3) => boolean): number {
  for (let t = 0; t < topo.triCount; t++) {
    const n: Vec3 = [topo.normals[3 * t], topo.normals[3 * t + 1], topo.normals[3 * t + 2]];
    const c: Vec3 = [topo.centroids[3 * t], topo.centroids[3 * t + 1], topo.centroids[3 * t + 2]];
    if (len(n) > 0.5 && pred(n, c)) return t;
  }
  throw new Error('Kein passendes Dreieck gefunden');
}

function asPlane(f: Feature): PlaneFeature {
  expect(f.kind).toBe('plane');
  return f as PlaneFeature;
}

function asCylinder(f: Feature): CylinderFeature {
  expect(f.kind).toBe('cylinder');
  return f as CylinderFeature;
}

function asSurface(f: Feature): SurfaceFeature {
  expect(f.kind).toBe('surface');
  return f as SurfaceFeature;
}

const expectNear = (a: Vec3, b: Vec3, tol = 1e-3) => expect(dist(a, b)).toBeLessThan(tol);
const expectParallel = (a: Vec3, b: Vec3, tol = 1e-5) => expect(Math.abs(dot(a, b))).toBeGreaterThan(1 - tol);

/** Extrudiert eine Kontur entlang +Z (wie die Hilfsnetze). */
function extrude(shape: THREE.Shape, depth: number, segments = 24): Float32Array {
  return toPositions(new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false, curveSegments: segments }));
}

/** Platte 40 × 20 × 5 mit Langloch (Halbkreise r = 3 um x = ±6, gerade Flanken bei y = ±3). */
function slotPlate(): Float32Array {
  const s = new THREE.Shape();
  s.moveTo(-20, -10);
  s.lineTo(20, -10);
  s.lineTo(20, 10);
  s.lineTo(-20, 10);
  const h = new THREE.Path();
  h.moveTo(-6, -3);
  h.lineTo(6, -3);
  h.absarc(6, 0, 3, -Math.PI / 2, Math.PI / 2, false);
  h.lineTo(-6, 3);
  h.absarc(-6, 0, 3, Math.PI / 2, (3 * Math.PI) / 2, false);
  s.holes.push(h);
  return extrude(s, 5);
}

/** Dreht den Umlaufsinn aller Dreiecke um, falls das Netz nach innen orientiert ist. */
function outward(p: Float32Array): Float32Array {
  let vol = 0;
  for (let i = 0; i < p.length; i += 9) {
    const a = new THREE.Vector3(p[i], p[i + 1], p[i + 2]);
    const b = new THREE.Vector3(p[i + 3], p[i + 4], p[i + 5]);
    const c = new THREE.Vector3(p[i + 6], p[i + 7], p[i + 8]);
    vol += a.dot(b.cross(c));
  }
  if (vol >= 0) return p;
  const out = p.slice();
  for (let i = 0; i < p.length; i += 9) for (let k = 0; k < 3; k++) [out[i + 3 + k], out[i + 6 + k]] = [p[i + 6 + k], p[i + 3 + k]];
  return out;
}

/**
 * Drehteil um die Y-Achse: Platte (r = 20, 3 hoch), darauf ein Zapfen r = 5 bis y = 15,
 * am Fuß mit r = 2 verrundet (die Mantelfläche beginnt bei y = 5).
 */
function filletPeg(): Float32Array {
  const pts = [new THREE.Vector2(0, 0), new THREE.Vector2(20, 0), new THREE.Vector2(20, 3)];
  for (let i = 0; i <= 8; i++) {
    const a = -Math.PI / 2 - (i / 8) * (Math.PI / 2);
    pts.push(new THREE.Vector2(7 + 2 * Math.cos(a), 5 + 2 * Math.sin(a)));
  }
  pts.push(new THREE.Vector2(5, 15), new THREE.Vector2(0, 15));
  return outward(toPositions(new THREE.LatheGeometry(pts, 48)));
}

const POSES: [string, THREE.Matrix4][] = [
  ['Ausgangslage', new THREE.Matrix4()],
  [
    'gedreht und verschoben',
    new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(0.3, -1, 0.5).normalize(), 2.0).setPosition(40, -25, 12),
  ],
];

describe.each(POSES)('detectFeature (%s)', (_pose, m) => {
  const P = (v: Vec3): Vec3 => toV(new THREE.Vector3(...v).applyMatrix4(m));
  const D = (v: Vec3): Vec3 => toV(new THREE.Vector3(...v).transformDirection(m));
  /** Erkennt das Feature am Dreieck, das in der Ausgangslage `pred` erfüllt, im bewegten Netz. */
  const detectAt = (pos: Float32Array, pred: (n: Vec3, c: Vec3) => boolean) =>
    detectFeature(buildTopology(transformPositions(pos, m)), findTri(buildTopology(pos), pred));

  it('Quaderfläche → Ebene mit Normale, Fläche, Schwerpunkt und vier Ecken', () => {
    const f = asPlane(detectAt(box(30, 20, 10), (n) => n[2] > 0.99));
    expectNear(f.normal, D([0, 0, 1]), 1e-5);
    expect(f.area).toBeCloseTo(600, 2);
    expectNear(f.centroid, P([0, 0, 10]));
    expect(f.tris).toHaveLength(2);
    expect(f.outline).toHaveLength(4);
    for (const c of [[15, 10, 10], [-15, 10, 10], [-15, -10, 10], [15, -10, 10]] as Vec3[])
      expect(f.outline.some((p) => dist(p, P(c)) < 1e-3)).toBe(true);
    // Geordnet: aufeinanderfolgende Ecken sind durch eine Rechteckseite verbunden
    f.outline.forEach((p, i) => {
      const l = dist(p, f.outline[(i + 1) % f.outline.length]);
      expect(Math.min(Math.abs(l - 30), Math.abs(l - 20))).toBeLessThan(1e-3);
    });
    expect(f.circle).toBeUndefined();
    expectParallel(f.axes.u, D([1, 0, 0]));
    expectParallel(f.axes.v, D([0, 1, 0]));
    expect(f.axes.size[0]).toBeCloseTo(30, 3);
    expect(f.axes.size[1]).toBeCloseTo(20, 3);
    // u, v und Normale bilden ein Rechtssystem
    const n = new THREE.Vector3(...f.axes.u).cross(new THREE.Vector3(...f.axes.v));
    expectNear(toV(n), f.normal, 1e-5);
  });

  it('Seitenfläche des Quaders → eigene Ebene', () => {
    const f = asPlane(detectAt(box(30, 20, 10), (n) => n[0] > 0.99));
    expectNear(f.normal, D([1, 0, 0]), 1e-5);
    expect(f.area).toBeCloseTo(200, 2);
    expectNear(f.centroid, P([15, 0, 5]));
  });

  it.each([
    [48, 96],
    [16, 32],
    [8, 16],
    [6, 12],
  ])('Zylindermantel mit %i Segmenten (%i Facetten) → Zylinder', (segments, facets) => {
    const f = asCylinder(detectAt(cylinder(5, 10, segments), (n) => Math.abs(n[2]) < 1e-3));
    expect(f.radius).toBeCloseTo(5, 3);
    expectParallel(f.axis, D([0, 0, 1]));
    expect(f.length).toBeCloseTo(10, 3);
    expectNear(f.center, P([0, 0, 5]));
    expect(f.hole).toBe(false);
    expect(f.arc).toBeCloseTo(2 * Math.PI, 2);
    expect(f.rms).toBeLessThan(1e-3);
    expect(f.tris).toHaveLength(2 * facets);
    expect(f.area).toBeGreaterThan(0.95 * 2 * Math.PI * 5 * 10);
  });

  it('Stirnfläche eines Zylinders → Ebene mit Kreis', () => {
    const top = asPlane(detectAt(cylinder(5, 10), (n) => n[2] > 0.99));
    expectNear(top.normal, D([0, 0, 1]), 1e-5);
    expect(top.circle).toBeDefined();
    expectNear(top.circle!.center, P([0, 0, 10]));
    expect(top.circle!.radius).toBeCloseTo(5, 3);
    expect(top.circle!.rms).toBeLessThan(1e-3);
    expectNear(top.centroid, P([0, 0, 10]));
    const bottom = asPlane(detectAt(cylinder(5, 10, 6), (n) => n[2] < -0.99));
    expectNear(bottom.normal, D([0, 0, -1]), 1e-5);
    expectNear(bottom.circle!.center, P([0, 0, 0]));
    expect(bottom.circle!.radius).toBeCloseTo(5, 3);
  });

  it('Ring: Innenwand → Bohrung, Außenwand → Zapfen, Oberseite → Kreis mit Außenradius', () => {
    const pos = ring(15, 5, 8);
    const inner = asCylinder(detectAt(pos, (n, c) => Math.abs(n[2]) < 1e-3 && Math.hypot(c[0], c[1]) < 10));
    expect(inner.hole).toBe(true);
    expect(inner.radius).toBeCloseTo(5, 3);
    expect(inner.length).toBeCloseTo(8, 3);
    expectNear(inner.center, P([0, 0, 4]));
    expectParallel(inner.axis, D([0, 0, 1]));
    expect(inner.arc).toBeCloseTo(2 * Math.PI, 2);

    const outer = asCylinder(detectAt(pos, (n, c) => Math.abs(n[2]) < 1e-3 && Math.hypot(c[0], c[1]) > 10));
    expect(outer.hole).toBe(false);
    expect(outer.radius).toBeCloseTo(15, 3);

    const top = asPlane(detectAt(pos, (n) => n[2] > 0.99));
    expectNear(top.normal, D([0, 0, 1]), 1e-5);
    expect(top.circle).toBeDefined();
    expect(top.circle!.radius).toBeCloseTo(15, 3);
    expectNear(top.circle!.center, P([0, 0, 8]));
    expect(top.area).toBeCloseTo(Math.PI * (15 * 15 - 5 * 5), -1);
    // Umriss ist der äußere Rand (nicht der Rand der Bohrung), der Reihe nach rundherum
    expect(top.outline).toHaveLength(96);
    top.outline.forEach((p, i) => {
      expect(dist(p, P([0, 0, 8]))).toBeCloseTo(15, 3);
      expect(dist(p, top.outline[(i + 1) % 96])).toBeCloseTo(2 * 15 * Math.sin(Math.PI / 96), 3);
    });
  });

  it('D-Profil: Rundung → Teilzylinder, Abflachung → Ebene, Stirnfläche ohne Kreis', () => {
    const pos = dPeg(5, 3, 10);
    const a = Math.acos(3 / 5);
    const round = asCylinder(detectAt(pos, (n) => Math.abs(n[2]) < 1e-3 && n[0] < -0.5));
    expect(round.radius).toBeCloseTo(5, 3);
    expect(round.hole).toBe(false);
    expect(Math.abs(round.arc - (2 * Math.PI - 2 * a))).toBeLessThan(0.02);
    expectNear(round.center, P([0, 0, 5]));
    expectParallel(round.axis, D([0, 0, 1]));
    expect(round.length).toBeCloseTo(10, 3);

    const flat = asPlane(detectAt(pos, (n) => n[0] > 0.999));
    expectNear(flat.normal, D([1, 0, 0]), 1e-5);
    expect(flat.area).toBeCloseTo(80, 2);
    expect(flat.outline).toHaveLength(4);
    expect(flat.circle).toBeUndefined();

    const top = asPlane(detectAt(pos, (n) => n[2] > 0.99));
    expect(top.circle).toBeUndefined();
  });

  it('Scheibe mit D-Bohrung: Rundung der Bohrung → Teilzylinder als Bohrung', () => {
    const a = Math.acos(3 / 5);
    const f = asCylinder(detectAt(dRing(15, 5, 3, 8), (n, c) => Math.abs(n[2]) < 1e-3 && Math.hypot(c[0], c[1]) < 10 && c[0] < 0));
    expect(f.hole).toBe(true);
    expect(f.radius).toBeCloseTo(5, 3);
    expect(Math.abs(f.arc - (2 * Math.PI - 2 * a))).toBeLessThan(0.02);
    expectNear(f.center, P([0, 0, 4]));
  });

  it('Langloch: Rundung → halbe Bohrung, tangential anschließende Flanke → Ebene', () => {
    const pos = slotPlate();
    const round = asCylinder(detectAt(pos, (n, c) => Math.abs(n[2]) < 1e-3 && c[0] > 7 && c[0] < 10 && Math.abs(c[1]) < 2));
    expect(round.hole).toBe(true);
    expect(round.radius).toBeCloseTo(3, 3);
    expectNear(round.center, P([6, 0, 2.5]));
    expect(Math.abs(round.arc - Math.PI)).toBeLessThan(0.02);

    const flank = asPlane(detectAt(pos, (n, c) => n[1] > 0.999 && Math.abs(c[1] + 3) < 1e-3));
    expectNear(flank.normal, D([0, 1, 0]), 1e-5);
    expect(flank.area).toBeCloseTo(60, 2);
  });

  it('Zapfen mit Verrundung am Fuß → Zylinder ohne die Verrundung', () => {
    const pos = filletPeg();
    const peg = asCylinder(detectAt(pos, (n, c) => Math.abs(n[1]) < 1e-3 && Math.hypot(c[0], c[2]) < 6 && c[1] > 7));
    expect(peg.radius).toBeCloseTo(5, 3);
    expectParallel(peg.axis, D([0, 1, 0]));
    expect(peg.length).toBeCloseTo(10, 3);
    expectNear(peg.center, P([0, 10, 0]));
    expect(peg.hole).toBe(false);
    expect(peg.arc).toBeCloseTo(2 * Math.PI, 2);

    const plate = asPlane(detectAt(pos, (n, c) => n[1] > 0.999 && c[1] > 2.9 && Math.hypot(c[0], c[2]) > 10));
    expect(plate.circle!.radius).toBeCloseTo(20, 3);
    expectNear(plate.circle!.center, P([0, 3, 0]));
  });

  it('Kapsel: Mantel zwischen den Halbkugeln → Zylinder, Halbkugel → Freiformfläche', () => {
    const pos = toPositions(new THREE.CapsuleGeometry(4, 12, 4, 24));
    const side = asCylinder(detectAt(pos, (n) => Math.abs(n[1]) < 1e-3));
    expect(side.radius).toBeCloseTo(4, 3);
    expect(side.length).toBeCloseTo(12, 3);
    expectParallel(side.axis, D([0, 1, 0]));
    expectNear(side.center, P([0, 0, 0]));
    expect(side.arc).toBeCloseTo(2 * Math.PI, 2);
    expect(detectAt(pos, (n) => n[1] > 0.9).kind).toBe('surface');
  });

  // 15 Reihen: die Facettenreihe am Äquator ist exakt zylindrisch, gehört aber zur Kugel
  it.each([
    [16, 12],
    [32, 16],
    [32, 15],
    [64, 32],
  ])('Kugel mit %i × %i Segmenten → Freiformfläche', (w, h) => {
    const pos = toPositions(new THREE.SphereGeometry(10, w, h));
    const preds = [(n: Vec3) => Math.abs(n[1]) < 0.15, (n: Vec3) => n[1] > 0.9, (n: Vec3) => n[0] < -0.5 && n[1] < -0.3];
    for (const pred of preds) {
      const f = asSurface(detectAt(pos, pred));
      expect(f.tris.length).toBeGreaterThan(1);
      expect(len(f.normal)).toBeCloseTo(1, 6);
      // Mittlere Normale zeigt vom Kugelmittelpunkt zum Schwerpunkt der Region
      const radial = toV(new THREE.Vector3(...f.centroid).sub(new THREE.Vector3(...P([0, 0, 0]))).normalize());
      expect(dot(f.normal, radial)).toBeGreaterThan(0.95);
    }
  });

  it('Torus, Kegel und Kegelstumpf ergeben keine Zylinder', () => {
    const cases = [
      new THREE.TorusGeometry(10, 3, 16, 48),
      new THREE.ConeGeometry(5, 10, 32),
      new THREE.CylinderGeometry(3, 5, 10, 48),
      // nur 1° Neigung: über die Länge ändert sich der Radius um 3,5 %
      new THREE.CylinderGeometry(5 - 10 * Math.tan(Math.PI / 180), 5, 10, 48),
    ];
    for (const g of cases) {
      const pos = toPositions(g);
      const local = buildTopology(pos);
      const moved = buildTopology(transformPositions(pos, m));
      for (let t = 0; t < local.triCount; t += 9) {
        // Grund- und Deckflächen (Normale entlang Y) auslassen, die sind eben
        if (Math.abs(local.normals[3 * t + 1]) > 0.99) continue;
        expect(detectFeature(moved, t).kind).not.toBe('cylinder');
      }
    }
  });
});

describe('detectFeature – Grenzen', () => {
  it('begrenzt jede Region auf maxTris', () => {
    const sphere = buildTopology(toPositions(new THREE.SphereGeometry(10, 64, 32)));
    expect(detectFeature(sphere, 500, { maxTris: 50 }).tris.length).toBeLessThanOrEqual(50);
    const plane = buildTopology(toPositions(new THREE.PlaneGeometry(100, 100, 100, 100)));
    const f = detectFeature(plane, 0, { maxTris: 1000 });
    expect(f.kind).toBe('plane');
    expect(f.tris.length).toBeLessThanOrEqual(1000);
  });

  it('kleinerer Glättungswinkel trennt grobe Facetten', () => {
    const topo = buildTopology(cylinder(5, 10, 6));
    const seed = findTri(topo, (n) => Math.abs(n[2]) < 1e-3);
    expect(detectFeature(topo, seed).kind).toBe('cylinder');
    expect(detectFeature(topo, seed, { smoothAngleDeg: 25 }).kind).toBe('plane');
  });

  it('meldet ungültige Dreiecksnummern', () => {
    const topo = buildTopology(box(1, 1, 1));
    expect(() => detectFeature(topo, 12)).toThrow(RangeError);
    expect(() => detectFeature(topo, -1)).toThrow(RangeError);
  });

  it('läuft auch auf großen Netzen schnell (keine quadratische Laufzeit)', () => {
    const pos = toPositions(new THREE.SphereGeometry(10, 512, 256));
    const t0 = performance.now();
    const topo = buildTopology(pos);
    const f = detectFeature(topo, 1000);
    const ms = performance.now() - t0;
    expect(topo.triCount).toBeGreaterThan(250000);
    expect(f.kind).toBe('surface');
    expect(ms).toBeLessThan(5000);
  });
});

describe('buildTopology', () => {
  /** Prüft, dass Nachbarschaften gegenseitig sind. */
  function expectSymmetric(topo: Topology) {
    for (let t = 0; t < topo.triCount; t++)
      for (let k = 0; k < 3; k++) {
        const nb = topo.neighbors[3 * t + k];
        if (nb < 0) continue;
        const back = [0, 1, 2].map((j) => topo.neighbors[3 * nb + j]);
        expect(back).toContain(t);
      }
  }

  it('geschlossener Quader: jedes Dreieck hat drei Nachbarn', () => {
    const topo = buildTopology(box(30, 20, 10));
    expect(topo.triCount).toBe(12);
    expect(topo.vertCount).toBe(8);
    expect(Array.from(topo.neighbors).every((n) => n >= 0)).toBe(true);
    expectSymmetric(topo);
    for (let t = 0; t < topo.triCount; t++) {
      expect(len([topo.normals[3 * t], topo.normals[3 * t + 1], topo.normals[3 * t + 2]])).toBeCloseTo(1, 6);
    }
    expect(Array.from(topo.areas).reduce((s, a) => s + a, 0)).toBeCloseTo(2 * (600 + 300 + 200), 3);
  });

  it('Zylinder, Ring und Kugel sind nach dem Verschweißen geschlossen', () => {
    for (const pos of [cylinder(5, 10), ring(15, 5, 8), dRing(15, 5, 3, 8), toPositions(new THREE.SphereGeometry(10, 16, 12))]) {
      const topo = buildTopology(pos);
      expect(Array.from(topo.neighbors).every((n) => n >= 0)).toBe(true);
      expectSymmetric(topo);
    }
  });

  it('verschweißt leicht verrauschte Eckpunkte innerhalb der Toleranz', () => {
    const pos = box(30, 20, 10);
    let seed = 1;
    const noisy = pos.map((x) => {
      seed = (seed * 16807) % 2147483647;
      return x + (seed / 2147483647 - 0.5) * 1e-5;
    });
    const topo = buildTopology(noisy);
    expect(topo.vertCount).toBe(8);
    expect(Array.from(topo.neighbors).every((n) => n >= 0)).toBe(true);
    // Mit sehr kleiner Toleranz bleiben die Punkte getrennt
    expect(buildTopology(noisy, 1e-9).vertCount).toBeGreaterThan(8);
  });

  it('offene und mehrdeutige Kanten ergeben -1', () => {
    // Quader ohne die beiden Dreiecke der Oberseite: deren Nachbarn haben eine offene Kante
    const pos = box(30, 20, 10);
    const topo0 = buildTopology(pos);
    const keep: number[] = [];
    for (let t = 0; t < topo0.triCount; t++) if (topo0.normals[3 * t + 2] < 0.99) keep.push(t);
    const open = new Float32Array(keep.flatMap((t) => Array.from(pos.subarray(9 * t, 9 * t + 9))));
    expect(Array.from(buildTopology(open).neighbors).filter((n) => n < 0)).toHaveLength(4);

    // Drei Dreiecke an einer Kante
    const fan = new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
    const topo = buildTopology(fan);
    for (let t = 0; t < 3; t++) expect(topo.neighbors[3 * t]).toBe(-1);
  });

  it('entartete Dreiecke stören weder Topologie noch Erkennung', () => {
    const extra = new Float32Array([
      // zusammengefallene Ecke
      15, 10, 10, 15, 10, 10, -15, 10, 10,
      // drei Punkte auf einer Linie
      -15, -10, 10, 0, -10, 10, 15, -10, 10,
      // ganz ohne Ausdehnung
      0, 0, 0, 0, 0, 0, 0, 0, 0,
    ]);
    const pos = concat(box(30, 20, 10), extra);
    const topo = buildTopology(pos);
    expect(Array.from(topo.normals.subarray(36)).every((x) => x === 0)).toBe(true);
    expect(Array.from(topo.areas.subarray(12)).every((a) => a < 1e-9)).toBe(true);
    for (let t = 12; t < 15; t++) expect(() => detectFeature(topo, t)).not.toThrow();
    const f = asPlane(detectFeature(topo, findTri(topo, (n) => n[2] > 0.99)));
    expect(f.area).toBeCloseTo(600, 2);
    const side = asPlane(detectFeature(topo, findTri(topo, (n) => n[1] < -0.99)));
    expect(side.area).toBeCloseTo(300, 2);
  });

  it('leere Eingabe', () => {
    const topo = buildTopology(new Float32Array(0));
    expect(topo.triCount).toBe(0);
    expect(topo.vertCount).toBe(0);
  });
});
