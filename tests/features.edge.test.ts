/**
 * Randfälle und realistische Netze für die Flächenerkennung (Review): feine und grobe Facetten,
 * tangential anschließende Verrundungen, Lage weit weg vom Ursprung, sehr kleine und große Teile,
 * verrauschte oder fehlerhafte Eingaben und große Netze.
 */
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { describe, expect, it } from 'vitest';
import { type Vec3, dist, dot, len } from '../src/math/align';
import {
  type CylinderFeature, type Feature, type PlaneFeature, type Topology, buildTopology, detectFeature,
} from '../src/math/features';
import { box, concat, cylinder, dPeg, ring, toPositions, transformPositions } from './helpers/meshes';

const toV = (v: THREE.Vector3): Vec3 => [v.x, v.y, v.z];
const DEG = Math.PI / 180;

/** Erstes Dreieck, dessen Normale und Schwerpunkt die Bedingung erfüllen. */
function findTri(topo: Topology, pred: (n: Vec3, c: Vec3) => boolean): number {
  for (let t = 0; t < topo.triCount; t++) {
    const n: Vec3 = [topo.normals[3 * t], topo.normals[3 * t + 1], topo.normals[3 * t + 2]];
    const c: Vec3 = [topo.centroids[3 * t], topo.centroids[3 * t + 1], topo.centroids[3 * t + 2]];
    if (len(n) > 0.5 && pred(n, c)) return t;
  }
  throw new Error('Kein passendes Dreieck gefunden');
}

/** Alle Dreiecke, die die Bedingung erfüllen. */
function allTris(topo: Topology, pred: (n: Vec3, c: Vec3) => boolean): number[] {
  const out: number[] = [];
  for (let t = 0; t < topo.triCount; t++) {
    const n: Vec3 = [topo.normals[3 * t], topo.normals[3 * t + 1], topo.normals[3 * t + 2]];
    const c: Vec3 = [topo.centroids[3 * t], topo.centroids[3 * t + 1], topo.centroids[3 * t + 2]];
    if (len(n) > 0.5 && pred(n, c)) out.push(t);
  }
  return out;
}

function asPlane(f: Feature): PlaneFeature {
  expect(f.kind).toBe('plane');
  return f as PlaneFeature;
}

function asCylinder(f: Feature): CylinderFeature {
  expect(f.kind).toBe('cylinder');
  return f as CylinderFeature;
}

const expectNear = (a: Vec3, b: Vec3, tol = 1e-3) => expect(dist(a, b)).toBeLessThan(tol);
const expectParallel = (a: Vec3, b: Vec3, tol = 1e-5) => expect(Math.abs(dot(a, b))).toBeGreaterThan(1 - tol);

/** Dreht den Umlaufsinn aller Dreiecke um (Normalen zeigen danach in die Gegenrichtung). */
function flipWinding(p: Float32Array): Float32Array {
  const out = p.slice();
  for (let i = 0; i < p.length; i += 9) for (let k = 0; k < 3; k++) [out[i + 3 + k], out[i + 6 + k]] = [p[i + 6 + k], p[i + 3 + k]];
  return out;
}

/** Richtet ein Netz nach außen aus (LatheGeometry erzeugt je nach Profil innen liegende Normalen). */
function outward(p: Float32Array): Float32Array {
  let vol = 0;
  for (let i = 0; i < p.length; i += 9) {
    const a = new THREE.Vector3(p[i], p[i + 1], p[i + 2]);
    const b = new THREE.Vector3(p[i + 3], p[i + 4], p[i + 5]);
    const c = new THREE.Vector3(p[i + 6], p[i + 7], p[i + 8]);
    vol += a.dot(b.cross(c));
  }
  return vol >= 0 ? p : flipWinding(p);
}

function extrude(shape: THREE.Shape, depth: number, segments = 12): Float32Array {
  return toPositions(new THREE.ExtrudeGeometry(shape, { depth, bevelEnabled: false, curveSegments: segments }));
}

/** Rechteck w × h mit Eckenradius r (in XY zentriert). */
function roundedRect(w: number, h: number, r: number): THREE.Shape {
  const s = new THREE.Shape();
  const x = w / 2 - r;
  const y = h / 2 - r;
  s.moveTo(-x, -h / 2);
  s.lineTo(x, -h / 2);
  s.absarc(x, -y, r, -Math.PI / 2, 0, false);
  s.lineTo(w / 2, y);
  s.absarc(x, y, r, 0, Math.PI / 2, false);
  s.lineTo(-x, h / 2);
  s.absarc(-x, y, r, Math.PI / 2, Math.PI, false);
  s.lineTo(-w / 2, -y);
  s.absarc(-x, -y, r, Math.PI, 1.5 * Math.PI, false);
  return s;
}

/** Platte 40 × 20 × 5 mit Langloch (Halbkreise r = 3 um x = ±half, gerade Flanken bei y = ±3). */
function slotPlate(half: number): Float32Array {
  const s = new THREE.Shape();
  s.moveTo(-20, -10);
  s.lineTo(20, -10);
  s.lineTo(20, 10);
  s.lineTo(-20, 10);
  const h = new THREE.Path();
  h.moveTo(-half, -3);
  h.lineTo(half, -3);
  h.absarc(half, 0, 3, -Math.PI / 2, Math.PI / 2, false);
  h.lineTo(-half, 3);
  h.absarc(-half, 0, 3, Math.PI / 2, (3 * Math.PI) / 2, false);
  s.holes.push(h);
  return extrude(s, 5, 24);
}

/**
 * Drehteil um Y: runde Platte (r = R, 5 hoch), darauf ein Zapfen r = 3 bis y = 15, Fuß mit r = 1
 * und Kopfkante mit r = 0,5 verrundet. Die ebene Stirnfläche hat damit den Radius 2,5.
 */
function pegOnPlate(R = 50, fs = 4, seg = 48): Float32Array {
  const pts = [new THREE.Vector2(0, 0), new THREE.Vector2(R, 0), new THREE.Vector2(R, 5)];
  for (let i = 0; i <= fs; i++) {
    const a = -Math.PI / 2 - (i / fs) * (Math.PI / 2);
    pts.push(new THREE.Vector2(4 + Math.cos(a), 6 + Math.sin(a)));
  }
  for (let i = 0; i <= fs; i++) {
    const a = (i / fs) * (Math.PI / 2);
    pts.push(new THREE.Vector2(2.5 + 0.5 * Math.cos(a), 14.5 + 0.5 * Math.sin(a)));
  }
  pts.push(new THREE.Vector2(0, 15));
  return outward(toPositions(new THREE.LatheGeometry(pts, seg)));
}

/**
 * Drehteil um Y: Platte (r = R, 3 hoch), Zapfen r = 5 bis y = 15, Fuß mit r = 2 in `fs` Segmenten
 * verrundet (Mantel von y = 5 bis 15, in `rows` Reihen).
 */
function filletPeg(fs: number, R = 20, rows = 1, seg = 48): Float32Array {
  const pts = [new THREE.Vector2(0, 0), new THREE.Vector2(R, 0), new THREE.Vector2(R, 3)];
  for (let i = 0; i <= fs; i++) {
    const a = -Math.PI / 2 - (i / fs) * (Math.PI / 2);
    pts.push(new THREE.Vector2(7 + 2 * Math.cos(a), 5 + 2 * Math.sin(a)));
  }
  for (let i = 1; i <= rows; i++) pts.push(new THREE.Vector2(5, 5 + (10 * i) / rows));
  pts.push(new THREE.Vector2(0, 15));
  return outward(toPositions(new THREE.LatheGeometry(pts, seg)));
}

/** Pseudozufall in [-1, 1] aus einer Position (gleiche Position → gleicher Wert, Verschweißen bleibt möglich). */
function hashNoise(x: number, y: number, z: number): number {
  const s = Math.sin(Math.round(x * 1e4) * 12.9898 + Math.round(y * 1e4) * 78.233 + Math.round(z * 1e4) * 37.719) * 43758.5453;
  return (s - Math.floor(s)) * 2 - 1;
}

/** Verschiebt alle Punkte radial (um die Z-Achse) um bis zu ±amp · Abstand. */
function radialJitter(p: Float32Array, amp: number): Float32Array {
  const out = p.slice();
  for (let i = 0; i < p.length; i += 3) {
    const k = 1 + amp * hashNoise(p[i], p[i + 1], p[i + 2]);
    out[i] = p[i] * k;
    out[i + 1] = p[i + 1] * k;
  }
  return out;
}

/** Kegelstumpf um Y (Radius unten 5, oben um `deg` Grad verjüngt), offen. */
function taper(deg: number, radial: number, rows: number): Float32Array {
  return toPositions(new THREE.CylinderGeometry(5 - 20 * Math.tan(deg * DEG), 5, 20, radial, rows, true));
}

// ---------------------------------------------------------------------------------------------

describe('Ebenen, die tangential in gekrümmte Flächen übergehen', () => {
  it('kleine Stirnfläche eines verrundeten Zapfens auf großer Platte → Ebene mit Kreis', () => {
    // Fuß und Kopfkante verrundet: die glatte Fläche reicht über die ganze Platte (π·50² ≈ 7850 mm²),
    // die Stirnfläche selbst ist eine exakt ebene Kreisfläche r = 2,5
    const topo = buildTopology(pegOnPlate());
    const cap = asPlane(detectFeature(topo, findTri(topo, (n, c) => n[1] > 0.9999 && c[1] > 14.9)));
    expectNear(cap.normal, [0, 1, 0], 1e-5);
    expect(cap.area).toBeCloseTo(Math.PI * 2.5 * 2.5, 0);
    expect(cap.circle).toBeDefined();
    expect(cap.circle!.radius).toBeCloseTo(2.5, 3);
    expectNear(cap.circle!.center, [0, 15, 0]);
  });

  it('kurze ebene Seite eines stark abgerundeten Quadrats → Ebene', () => {
    // 22 × 22 mit Eckenradius 10: jede Seite hat ein 2 mm breites, exakt ebenes Stück
    const topo = buildTopology(extrude(roundedRect(22, 22, 10), 10));
    const f = asPlane(detectFeature(topo, findTri(topo, (n) => n[0] > 0.9999)));
    expectNear(f.normal, [1, 0, 0], 1e-5);
    expect(f.area).toBeCloseTo(20, 2);
  });

  it('kurze gerade Flanke eines Langlochs → Ebene', () => {
    // Langloch mit nur 1 mm Geradenstück zwischen den Halbkreisen
    const topo = buildTopology(slotPlate(0.5));
    const f = asPlane(detectFeature(topo, findTri(topo, (n, c) => n[1] > 0.9999 && Math.abs(c[1] + 3) < 1e-3)));
    expectNear(f.normal, [0, 1, 0], 1e-5);
    expect(f.area).toBeCloseTo(5, 2);
  });

  it('große Seiten eines abgerundeten Rechtecks bleiben Ebenen (Regression)', () => {
    const topo = buildTopology(extrude(roundedRect(40, 20, 5), 10));
    expect(asPlane(detectFeature(topo, findTri(topo, (n) => n[1] > 0.9999))).area).toBeCloseTo(300, 2);
    expect(asPlane(detectFeature(topo, findTri(topo, (n) => n[0] > 0.9999))).area).toBeCloseTo(100, 2);
    const top = asPlane(detectFeature(topo, findTri(topo, (n) => n[2] > 0.9999)));
    expect(top.circle).toBeUndefined();
  });
});

describe('Stirnflächen mit Splitterdreiecken (Float32-Rundung)', () => {
  // Die Deckel der extrudierten Zylinder bestehen zum Teil aus sehr schmalen Dreiecken; deren
  // Normale weicht nach Drehung und Rundung auf Float32 um Bruchteile eines Grads ab. Egal welches
  // Deckeldreieck angeklickt wird, das Ergebnis muss die ganze Kreisfläche sein.
  it.each([
    ['fein facettiert, gedreht, am Ursprung', 5, 256, [0, 0, 0]],
    ['48 Segmente, gedreht, an Druckbett-Koordinaten', 2.5, 48, [150, 120, 20]],
  ] as [string, number, number, Vec3][])('%s: jedes Deckeldreieck liefert die ganze Kreisfläche', (_name, r, seg, off) => {
    const pos = cylinder(r, 4, seg);
    const local = buildTopology(pos);
    const m = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(1, 2, 3).normalize(), 0.7).setPosition(...off);
    const topo = buildTopology(transformPositions(pos, m));
    const capTris = allTris(local, (n) => n[2] > 0.99);
    const center = toV(new THREE.Vector3(0, 0, 4).applyMatrix4(m));
    const bad: number[] = [];
    for (const t of capTris) {
      const f = detectFeature(topo, t);
      const ok = f.kind === 'plane' && f.tris.length === capTris.length && !!f.circle &&
        Math.abs(f.circle.radius - r) < 1e-3 * r && dist(f.circle.center, center) < 1e-3;
      if (!ok) bad.push(t);
    }
    expect(bad).toEqual([]);
  });
});

describe('Zylinder neben fein unterteilten tangentialen Flächen', () => {
  it.each([
    [4, 24],
    [8, 48],
    [12, 48],
    [16, 48],
    [32, 64],
  ])('Kapsel mit %i Segmenten je Halbkugel (%i rundherum) → Zylinder r = 4, Länge 12', (capSeg, radial) => {
    const topo = buildTopology(toPositions(new THREE.CapsuleGeometry(4, 12, capSeg, radial)));
    const f = asCylinder(detectFeature(topo, findTri(topo, (n) => Math.abs(n[1]) < 1e-3)));
    expect(f.radius).toBeCloseTo(4, 3);
    expect(Math.abs(f.length - 12)).toBeLessThan(0.02);
    expectNear(f.center, [0, 0, 0], 0.01);
    expectParallel(f.axis, [0, 1, 0]);
    expect(f.arc).toBeCloseTo(2 * Math.PI, 2);
  });

  it.each([4, 8, 16, 32])('Fußverrundung mit %i Segmenten gehört nicht zum Zylinder', (fs) => {
    const topo = buildTopology(filletPeg(fs));
    const f = asCylinder(detectFeature(topo, findTri(topo, (n, c) => Math.abs(n[1]) < 1e-3 && Math.hypot(c[0], c[2]) < 6 && c[1] > 7)));
    expect(f.radius).toBeCloseTo(5, 3);
    // Die Mantelfläche reicht exakt von y = 5 bis 15
    expect(Math.abs(f.length - 10)).toBeLessThan(0.02);
    expectNear(f.center, [0, 10, 0], 0.01);
    expect(f.rms).toBeLessThan(1e-3);
  });

  it.each([2, 4, 6, 8, 12])('abgerundeter Quader (%i Segmente): Kantenverrundung → Viertelzylinder', (seg) => {
    // 40 × 30 × 20, Radius 3: die Kante entlang Y bei x = 17, z = 7 ist ein exakter Viertelzylinder der Länge 24
    const topo = buildTopology(toPositions(new RoundedBoxGeometry(40, 30, 20, seg, 3)));
    const seed = findTri(topo, (n, c) => Math.abs(n[1]) < 1e-4 && n[0] > 0.5 && n[2] > 0.5 && Math.abs(c[1]) < 5);
    const f = asCylinder(detectFeature(topo, seed));
    expect(f.radius).toBeCloseTo(3, 3);
    expect(f.hole).toBe(false);
    expectParallel(f.axis, [0, 1, 0]);
    expect(Math.abs(f.arc - Math.PI / 2)).toBeLessThan(0.02);
    expect(Math.abs(f.length - 24)).toBeLessThan(0.02);
    expectNear(f.center, [17, 0, 7], 0.01);
    // Die ebenen Seiten bleiben eigene Ebenen
    expect(asPlane(detectFeature(topo, findTri(topo, (n) => n[2] > 0.99999))).area).toBeCloseTo(34 * 24, 1);
  });

  it.each([
    [40, 20, 5],
    [22, 22, 10],
  ])('abgerundetes Rechteck %i × %i, Eckenradius %i: Ecke → Viertelzylinder', (w, h, r) => {
    const topo = buildTopology(extrude(roundedRect(w, h, r), 10));
    const f = asCylinder(detectFeature(topo, findTri(topo, (n) => n[0] > 0.6 && n[1] > 0.6)));
    expect(f.radius).toBeCloseTo(r, 3);
    expect(Math.abs(f.arc - Math.PI / 2)).toBeLessThan(0.02);
    expect(f.length).toBeCloseTo(10, 3);
    expectNear(f.center, [w / 2 - r, h / 2 - r, 5]);
  });

  it('sehr fein facettierter verrundeter Zapfen auf großer Platte → Zylinder', () => {
    // 1400 Segmente rundherum (0,26° je Facette), Mantel in 20 Reihen, Platte r = 40
    const topo = buildTopology(filletPeg(8, 40, 20, 1400));
    const seed = findTri(topo, (n, c) => Math.abs(n[1]) < 1e-3 && Math.hypot(c[0], c[2]) < 6 && c[1] > 9 && c[1] < 12);
    const f = asCylinder(detectFeature(topo, seed));
    expect(f.radius).toBeCloseTo(5, 3);
    expect(f.length).toBeCloseTo(10, 2);
    expectParallel(f.axis, [0, 1, 0]);
  });
});

describe('Kegel und Kegelstümpfe', () => {
  it('Kegelstumpf mit 1° Neigung ist auch bei begrenzter Region kein Zylinder', () => {
    const topo = buildTopology(taper(1, 128, 10));
    for (const s of [0.13, 0.5, 0.77]) {
      const f = detectFeature(topo, Math.floor(s * topo.triCount), { maxTris: 500 });
      expect(f.tris.length).toBeLessThanOrEqual(500);
      expect(f.kind).not.toBe('cylinder');
    }
  });

  it('Kegelstumpf mit 1° Neigung und 1 Mio. Dreiecken ist mit Standardeinstellungen kein Zylinder', () => {
    const topo = buildTopology(taper(1, 2048, 250));
    expect(topo.triCount).toBeGreaterThan(1e6);
    const f = detectFeature(topo, Math.floor(0.5 * topo.triCount));
    expect(f.kind).not.toBe('cylinder');
  }, 60000);

  it('fein facettierter Kegelstumpf → Freiformfläche (kein schmaler Ebenenstreifen)', () => {
    // 256 Facetten rundherum: benachbarte Facetten liegen nur 1,4° auseinander
    const topo = buildTopology(taper(1, 256, 50));
    for (const s of [0.13, 0.5, 0.77]) expect(detectFeature(topo, Math.floor(s * topo.triCount)).kind).toBe('surface');
  });
});

describe('Verrauschte und fehlerhafte Eingaben', () => {
  it('Zylinder mit ±1 % radialem Rauschen wird noch als Zylinder erkannt', () => {
    // RMS des Rauschens ≈ 0,6 % des Radius, also deutlich unter den in der Vorgabe genannten 2 %
    const topo = buildTopology(radialJitter(cylinder(5, 10, 48), 0.01));
    expect(topo.vertCount).toBe(192);
    const f = asCylinder(detectFeature(topo, findTri(topo, (n) => Math.abs(n[2]) < 0.05)));
    expect(Math.abs(f.radius - 5)).toBeLessThan(0.05);
    expectParallel(f.axis, [0, 0, 1], 1e-3);
    expect(f.hole).toBe(false);
  });

  it('Zylinder mit ±0,2 % radialem Rauschen (Regression)', () => {
    const topo = buildTopology(radialJitter(cylinder(5, 10, 48), 0.002));
    const f = asCylinder(detectFeature(topo, findTri(topo, (n) => Math.abs(n[2]) < 0.05)));
    expect(Math.abs(f.radius - 5)).toBeLessThan(0.01);
  });

  it('ein Dreieck mit unendlicher Koordinate verdirbt die Erkennung des übrigen Netzes nicht', () => {
    const m = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(0.3, -1, 0.5).normalize(), 2.0).setPosition(40, -25, 12);
    const local = buildTopology(box(30, 20, 10));
    const seed = findTri(local, (n) => n[2] > 0.99);
    for (const bad of [Infinity, -Infinity, NaN]) {
      const topo = buildTopology(concat(transformPositions(box(30, 20, 10), m), new Float32Array([bad, 0, 0, 1, 0, 0, 0, 1, 0])));
      const f = asPlane(detectFeature(topo, seed));
      expect(f.area).toBeCloseTo(600, 1);
      expect(f.outline).toHaveLength(4);
    }
  });

  it('ein doppelt gespeichertes Dreieck trennt die Fläche nicht', () => {
    const b = box(30, 20, 10);
    const local = buildTopology(b);
    const [t0, t1] = allTris(local, (n) => n[2] > 0.99);
    const topo = buildTopology(concat(b, b.slice(9 * t0, 9 * t0 + 9)));
    const f = asPlane(detectFeature(topo, t1));
    expect(f.area).toBeCloseTo(600, 2);
    expect(f.outline).toHaveLength(4);
  });

  it('nicht verschweißte Naht wird mit passender Toleranz geschlossen (Regression)', () => {
    // Ein Viertel des Mantels liegt 1 µm daneben: mit Standardtoleranz offen, mit weldTol = 0,005 geschlossen
    const pos = cylinder(5, 10, 48);
    const local = buildTopology(pos);
    const shifted = pos.slice();
    for (const t of allTris(local, (n, c) => Math.abs(n[2]) < 0.5 && c[0] > 0 && c[1] > 0))
      for (let i = 9 * t; i < 9 * t + 9; i += 3) {
        shifted[i] += 1e-3;
        shifted[i + 1] += 1e-3;
      }
    const seed = findTri(local, (n, c) => Math.abs(n[2]) < 1e-3 && c[0] < 0);
    const f = asCylinder(detectFeature(buildTopology(shifted, 0.005), seed));
    expect(f.tris).toHaveLength(192);
    expect(f.arc).toBeCloseTo(2 * Math.PI, 2);
    expect(f.radius).toBeCloseTo(5, 2);
  });
});

describe('Maßstab, Lage und Orientierung (Regression)', () => {
  it.each([0.001, 0.01, 0.1, 10, 100])('Zylinder im Maßstab %f', (s) => {
    const m = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(-0.4, 0.2, 1).normalize(), 1.1).scale(new THREE.Vector3(s, s, s));
    m.setPosition(3 * s, -2 * s, s);
    const local = buildTopology(cylinder(5, 10, 48));
    const topo = buildTopology(transformPositions(cylinder(5, 10, 48), m));
    const f = asCylinder(detectFeature(topo, findTri(local, (n) => Math.abs(n[2]) < 1e-3)));
    expect(f.radius / s).toBeCloseTo(5, 3);
    expect(f.length / s).toBeCloseTo(10, 3);
    expectNear(f.center, toV(new THREE.Vector3(0, 0, 5).applyMatrix4(m)), 1e-4 * s);
    const cap = asPlane(detectFeature(topo, findTri(local, (n) => n[2] > 0.99)));
    expect(cap.circle!.radius / s).toBeCloseTo(5, 3);
  });

  it('dünne Ränder und sehr dünne Stifte', () => {
    const rimTopo = buildTopology(cylinder(20, 0.2, 360));
    const rim = asCylinder(detectFeature(rimTopo, findTri(rimTopo, (n) => Math.abs(n[2]) < 1e-3)));
    expect(rim.radius).toBeCloseTo(20, 3);
    expect(rim.length).toBeCloseTo(0.2, 4);
    const pinTopo = buildTopology(cylinder(0.3, 30, 24));
    const pin = asCylinder(detectFeature(pinTopo, findTri(pinTopo, (n) => Math.abs(n[2]) < 1e-3)));
    expect(pin.radius).toBeCloseTo(0.3, 4);
    expect(pin.length).toBeCloseTo(30, 3);
  });

  it('Achse (1, −1, 1): alle Mantel-Dreiecke liefern dieselbe Achse und Mitte', () => {
    const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(1, -1, 1).normalize());
    const m = new THREE.Matrix4().makeRotationFromQuaternion(q).setPosition(3, 4, 5);
    const local = buildTopology(cylinder(5, 10, 48));
    const topo = buildTopology(transformPositions(cylinder(5, 10, 48), m));
    const seeds = allTris(local, (n) => Math.abs(n[2]) < 1e-3);
    const first = asCylinder(detectFeature(topo, seeds[0]));
    expectNear(first.center, toV(new THREE.Vector3(0, 0, 5).applyMatrix4(m)));
    for (const t of seeds) {
      const f = asCylinder(detectFeature(topo, t));
      expectNear(f.axis, first.axis, 1e-6);
      expectNear(f.center, first.center, 1e-4);
    }
  });

  it('umgedrehter Umlaufsinn: Zapfen wird Bohrung und Normalen kehren sich um', () => {
    const topo = buildTopology(flipWinding(cylinder(5, 10, 48)));
    expect(asCylinder(detectFeature(topo, findTri(topo, (n) => Math.abs(n[2]) < 1e-3))).hole).toBe(true);
    const cap = asPlane(detectFeature(topo, findTri(topo, (n) => n[2] < -0.99)));
    expectNear(cap.normal, [0, 0, -1], 1e-5);
    expectNear(cap.circle!.center, [0, 0, 10]);
    const r = buildTopology(flipWinding(ring(15, 5, 8)));
    expect(asCylinder(detectFeature(r, findTri(r, (n, c) => Math.abs(n[2]) < 1e-3 && Math.hypot(c[0], c[1]) < 10))).hole).toBe(false);
  });

  it('gespiegeltes D-Profil (mit korrigiertem Umlaufsinn) wird gleich erkannt', () => {
    const pos = flipWinding(transformPositions(dPeg(5, 3, 10), new THREE.Matrix4().makeScale(-1, 1, 1)));
    const topo = buildTopology(pos);
    const round = asCylinder(detectFeature(topo, findTri(topo, (n) => Math.abs(n[2]) < 1e-3 && n[0] > 0.5)));
    expect(round.hole).toBe(false);
    expect(Math.abs(round.arc - (2 * Math.PI - 2 * Math.acos(3 / 5)))).toBeLessThan(0.02);
    const flat = asPlane(detectFeature(topo, findTri(topo, (n) => n[0] < -0.999)));
    expectNear(flat.normal, [-1, 0, 0], 1e-5);
    expect(flat.area).toBeCloseTo(80, 2);
  });

  it('Platte mit Bohrung: Wand → Bohrung, Oberseite → rechteckiger Umriss ohne Kreis', () => {
    const s = new THREE.Shape();
    s.moveTo(-30, -30);
    s.lineTo(30, -30);
    s.lineTo(30, 30);
    s.lineTo(-30, 30);
    const h = new THREE.Path();
    h.absarc(5, -3, 4, 0, Math.PI * 2, true);
    s.holes.push(h);
    const topo = buildTopology(extrude(s, 6, 24));
    const wall = asCylinder(detectFeature(topo, findTri(topo, (n) => Math.abs(n[2]) < 1e-3 && Math.abs(n[0]) < 0.99 && Math.abs(n[1]) < 0.99)));
    expect(wall.hole).toBe(true);
    expect(wall.radius).toBeCloseTo(4, 3);
    expectNear(wall.center, [5, -3, 3]);
    const top = asPlane(detectFeature(topo, findTri(topo, (n) => n[2] > 0.99)));
    expect(top.outline).toHaveLength(4);
    expect(top.circle).toBeUndefined();
    expect(top.area).toBeCloseTo(3600 - Math.PI * 16, 0);
  });
});

describe('buildTopology – Randfälle (Regression)', () => {
  it('Nachbarn liegen an der dokumentierten Kante (v0v1, v1v2, v2v0)', () => {
    // Viereck A B C D aus (A, B, C) und (A, C, D): gemeinsame Kante ist C-A bzw. A-C
    const quad = new Float32Array([0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0]);
    const topo = buildTopology(quad);
    expect(Array.from(topo.neighbors)).toEqual([-1, -1, 1, 0, -1, -1]);
  });

  it('verschweißt Punkte auch über Rasterzellgrenzen hinweg', () => {
    const tol = 1e-3;
    // Jede Koordinate höchstens 0,25 · tol verschoben: zwei Kopien liegen damit höchstens 0,87 · tol auseinander
    const pos = box(2, 2, 2).map((x, i) => x + (i % 2 ? 0.25 : -0.25) * tol * (((i * 7919) % 13) / 13));
    const topo = buildTopology(pos, tol);
    expect(topo.vertCount).toBe(8);
    expect(Array.from(topo.neighbors).every((n) => n >= 0)).toBe(true);
    // Kopien knapp links und rechts von 0 (verschiedene Rasterzellen)
    const near0 = new Float32Array([-1e-4, 0, 0, 1, 0, 0, 0, 1, 0, 1e-4, 0, 0, 0, -1, 0, 1, 0, 0]);
    const t2 = buildTopology(near0, tol);
    expect(t2.vertCount).toBe(4);
    expect(t2.neighbors[0]).toBe(1);
  });
});

describe('Laufzeit auf großen Netzen', () => {
  it('Zylinder mit 200 000 Dreiecken: Erkennung schnell und korrekt', () => {
    const pos = toPositions(new THREE.CylinderGeometry(5, 5, 20, 1024, 100));
    let t0 = performance.now();
    const topo = buildTopology(pos);
    const tBuild = performance.now() - t0;
    expect(topo.triCount).toBeGreaterThan(200000);
    const seeds = [0.13, 0.31, 0.5, 0.77].map((s) => Math.floor(s * 1024 * 100 * 2));
    t0 = performance.now();
    for (const s of seeds) {
      const f = asCylinder(detectFeature(topo, s));
      expect(f.radius).toBeCloseTo(5, 3);
      expectParallel(f.axis, [0, 1, 0]);
      expect(f.tris.length).toBeLessThanOrEqual(200000);
    }
    const tDetect = (performance.now() - t0) / seeds.length;
    expect(tBuild).toBeLessThan(3000);
    expect(tDetect).toBeLessThan(1500);
    // maxTris gilt auch für Zylinder
    expect(detectFeature(topo, seeds[0], { maxTris: 5000 }).tris.length).toBeLessThanOrEqual(5000);
  }, 30000);

  it('Torus und Kegelstumpf mit 200 000 Dreiecken bleiben schnell', () => {
    for (const g of [new THREE.TorusGeometry(20, 5, 150, 700), new THREE.CylinderGeometry(5 - 20 * Math.tan(3 * DEG), 5, 20, 1024, 100)]) {
      const topo = buildTopology(toPositions(g));
      const t0 = performance.now();
      for (const s of [0.13, 0.5, 0.77]) expect(detectFeature(topo, Math.floor(s * topo.triCount)).kind).not.toBe('cylinder');
      expect((performance.now() - t0) / 3).toBeLessThan(2000);
    }
  }, 30000);
});
