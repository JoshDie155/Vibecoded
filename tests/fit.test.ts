import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { describe, expect, it } from 'vitest';
import { IDENTITY, type Vec3, apply, compose, rotationAbout } from '../src/math/align';
import { type Feature, type PlaneFeature, buildTopology, detectFeature } from '../src/math/features';
import { bestPairing, fitFeatures, similarPlanes } from '../src/math/fit';
import { sampleSurface } from '../src/math/icp';
import { box, concat, geometryFrom, toPositions, transformPositions } from './helpers/meshes';

/** Prisma aus einem Umriss in der XY-Ebene, Unterseite bei z = 0. */
function prism(points: [number, number][], h: number): Float32Array {
  const s = new THREE.Shape(points.map(([x, y]) => new THREE.Vector2(x, y)));
  return toPositions(new THREE.ExtrudeGeometry(s, { depth: h, bevelEnabled: false }));
}

/** Quadrat mit abgeschrägten Ecken (Kantenlänge a, Fase c), um den Ursprung. */
function chamfered(a: number, c: number): [number, number][] {
  const h = a / 2;
  return [[-h + c, -h], [h - c, -h], [h, -h + c], [h, h - c], [h - c, h], [-h + c, h], [-h, h - c], [-h, -h + c]];
}

/** Erstes Dreieck, dessen Mitte die Bedingung erfüllt. */
function triWhere(pos: Float32Array, pred: (c: Vec3) => boolean): number {
  for (let t = 0; t < pos.length / 9; t++) {
    const c: Vec3 = [0, 1, 2].map((k) => (pos[t * 9 + k] + pos[t * 9 + 3 + k] + pos[t * 9 + 6 + k]) / 3) as Vec3;
    if (pred(c)) return t;
  }
  throw new Error('kein Dreieck gefunden');
}

function setup(fixedPos: Float32Array, fixedSeed: (c: Vec3) => boolean, movingLocal: Float32Array, movingSeed: (c: Vec3) => boolean, scramble: THREE.Matrix4) {
  const seedM = triWhere(movingLocal, movingSeed);
  const movingPos = transformPositions(movingLocal, scramble);
  const fixedGeom = geometryFrom(fixedPos);
  const bvh = new MeshBVH(fixedGeom);
  const dF = detectFeature(buildTopology(fixedPos), triWhere(fixedPos, fixedSeed));
  const sF = detectFeature(buildTopology(movingPos), seedM);
  const samples = sampleSurface(geometryFrom(movingPos), 3000);
  return { dF, sF, samples, target: { bvh, positions: fixedPos, matrix: IDENTITY } };
}

const fitOf = (s: ReturnType<typeof setup>, scale: number) =>
  fitFeatures({
    srcF: [s.sF], dstF: [s.dF], srcM: [], dstM: [], names: { src: 'B', dst: 'A' },
    alt: 0, seat: true, ordered: false, normals: 'auto', overlap: 0, scale, samples: s.samples, target: s.target,
  });

const centerOf = (f: Feature): Vec3 => (f.kind === 'cylinder' || f.kind === 'edge' ? f.center : f.centroid);

describe('similarPlanes', () => {
  const plane = (area: number, size: number) => ({ kind: 'plane', area, axes: { size: [size, size * 0.8] } }) as unknown as PlaneFeature;
  it('etwa gleich große Flächen gelten als Paar zum Mittig-Aufsetzen', () => {
    expect(similarPlanes(plane(14.8, 3.9), plane(16, 5))).toBe(true);
    expect(similarPlanes(plane(36, 6), plane(1600, 40))).toBe(false);
    expect(similarPlanes(plane(10, 5), plane(30, 5))).toBe(false);
  });
});

describe('fitFeatures', () => {
  // Festes Teil: Sockel 10 × 10 mit einer Wand daneben, an der ein Teil „bis zum Anschlag" rutschen könnte
  const fixed = concat(box(10, 10, 5), transformPositions(box(2, 14, 15), new THREE.Matrix4().makeTranslation(8, 0, 0)));
  const top = (c: Vec3) => Math.abs(c[2] - 5) < 1e-6 && Math.abs(c[0]) < 4.9;
  const scramble = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(0.3, -0.5, 0.8).normalize(), 2.1).setPosition(30, -12, 7);

  it('setzt etwa gleich große ebene Flächen mittig aufeinander, statt bis zur Wand zu schieben', () => {
    // Bewegliches Teil: Klotz mit gefastem Grundriss (nicht deckungsgleich mit dem Sockel)
    const moving = prism(chamfered(9.6, 1.5), 6);
    const s = setup(fixed, top, moving, (c) => Math.abs(c[2]) < 1e-6, scramble);
    expect(s.dF.kind).toBe('plane');
    expect(s.sF.kind).toBe('plane');
    const fit = fitOf(s, 20);
    expect(fit.error).toBeUndefined();
    expect(fit.info).toContain('mittig');
    // Mitte der Unterseite liegt auf der Mitte der Sockel-Oberseite
    const c = apply(fit.rel, centerOf(s.sF));
    expect(Math.hypot(c[0], c[1], c[2] - 5)).toBeLessThan(0.05);
    // Von Hand frei bleiben Drehen um die Normale und Verschieben in der Ebene
    expect(fit.free.map((m) => m.kind).sort()).toEqual(['rotate', 'translate', 'translate']);
  });

  it('kleine Teile auf großen Flächen bleiben in der Ebene verschiebbar', () => {
    const plate = concat(box(40, 40, 2), transformPositions(box(2, 40, 12), new THREE.Matrix4().makeTranslation(6, 0, 0)));
    const moving = box(6, 6, 6);
    const s = setup(plate, (c) => Math.abs(c[2] - 2) < 1e-6 && c[0] < 4, moving, (c) => Math.abs(c[2]) < 1e-6, scramble);
    const fit = fitOf(s, 25);
    expect(fit.info).not.toContain('mittig');
    expect(fit.free.filter((m) => m.kind === 'translate')).toHaveLength(2);
    // Aufgesetzt, die Lage in der Ebene bestimmt das Einpassen
    expect(apply(fit.rel, centerOf(s.sF))[2]).toBeCloseTo(2, 3);
  });

  it('meldet eine gleich gut passende 180°-Drehung als Wechselmöglichkeit', () => {
    // Rechteckiger Zapfen (nicht deckungsgleicher Umriss) auf rechteckiger Aufnahme: 0° und 180° passen gleich
    const base = concat(box(12, 6, 4), transformPositions(box(12, 1, 3), new THREE.Matrix4().makeTranslation(0, 3.5, 2.5)),
      transformPositions(box(12, 1, 3), new THREE.Matrix4().makeTranslation(0, -3.5, 2.5)));
    const moving = prism(chamfered(5.8, 0.8).map(([x, y]) => [x * 2, y] as [number, number]), 5);
    const s = setup(base, (c) => Math.abs(c[2] - 4) < 1e-6 && Math.abs(c[1]) < 2.9, moving, (c) => Math.abs(c[2]) < 1e-6, scramble);
    const fit = fitOf(s, 20);
    expect(fit.error).toBeUndefined();
    expect(fit.turn).toBeDefined();
    expect(fit.turn!.angles.some((a) => Math.abs(a - Math.PI) < 0.1)).toBe(true);
    // Die Wechsel-Drehung bleibt auf der Aufnahme: Mitte unverändert
    const turned = compose(rotationAbout(fit.turn!.point, fit.turn!.axis, fit.turn!.angles[0]), fit.rel);
    const a = apply(fit.rel, centerOf(s.sF));
    const b = apply(turned, centerOf(s.sF));
    expect(Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2])).toBeLessThan(1e-6);
  });

  it('ordnet Flächen richtig zu, auch wenn sie auf beiden Teilen in anderer Reihenfolge gewählt wurden', () => {
    // Stufe: Sockel 20 × 10 × 10 mit Wand rechts; der Klotz sitzt auf dem Sockel und liegt an der Wand an
    const step = concat(box(20, 10, 10), transformPositions(box(4, 10, 20), new THREE.Matrix4().makeTranslation(12, 0, 0)));
    const moving = box(20, 10, 6);
    const M = new THREE.Matrix4().makeRotationAxis(new THREE.Vector3(0.2, 0.9, -0.3).normalize(), 1.7).setPosition(-40, 30, 12);
    const movingPos = transformPositions(moving, M);
    const w = (p: Vec3) => new THREE.Vector3(...p).applyMatrix4(M).toArray() as Vec3;
    const fTopo = buildTopology(step);
    const mTopo = buildTopology(movingPos);
    const top = detectFeature(fTopo, triWhere(step, (c) => Math.abs(c[2] - 10) < 1e-6 && c[0] < 9.9));
    const wall = detectFeature(fTopo, triWhere(step, (c) => Math.abs(c[0] - 10) < 1e-6 && c[2] > 10));
    const bottom = detectFeature(mTopo, triWhere(moving, (c) => Math.abs(c[2]) < 1e-6));
    const right = detectFeature(mTopo, triWhere(moving, (c) => Math.abs(c[0] - 10) < 1e-6));
    // Klickreihenfolge vertauscht: bewegliches Teil zuerst die rechte Seite, festes Teil zuerst die Oberseite
    const paired = bestPairing([right, bottom], [top, wall], 30);
    expect(paired.changed).toBe(true);
    expect(paired.srcF[0]).toBe(bottom);
    const fit = fitFeatures({
      srcF: [right, bottom], dstF: [top, wall], srcM: [], dstM: [], names: { src: 'B', dst: 'A' },
      alt: 0, seat: true, ordered: false, normals: 'auto', overlap: 0, scale: 30,
      samples: sampleSurface(geometryFrom(movingPos), 2000), target: { bvh: new MeshBVH(geometryFrom(step)), positions: step, matrix: IDENTITY },
    });
    expect(fit.error).toBeUndefined();
    expect(fit.info).toContain('zugeordnet');
    // Ecke (10, ·, 0) des Klotzes: an der Wand (x = 10) auf dem Sockel (z = 10)
    const c = apply(fit.rel, w([10, 5, 0]));
    expect(c[0]).toBeCloseTo(10, 3);
    expect(c[2]).toBeCloseTo(10, 3);
    const top2 = apply(fit.rel, w([0, 0, 6]));
    expect(top2[2]).toBeCloseTo(16, 3);
  });

  it('richtige Reihenfolge bleibt unverändert', () => {
    const step = concat(box(20, 10, 10), transformPositions(box(4, 10, 20), new THREE.Matrix4().makeTranslation(12, 0, 0)));
    const moving = box(20, 10, 6);
    const fTopo = buildTopology(step);
    const mTopo = buildTopology(moving);
    const top = detectFeature(fTopo, triWhere(step, (c) => Math.abs(c[2] - 10) < 1e-6 && c[0] < 9.9));
    const wall = detectFeature(fTopo, triWhere(step, (c) => Math.abs(c[0] - 10) < 1e-6 && c[2] > 10));
    const bottom = detectFeature(mTopo, triWhere(moving, (c) => Math.abs(c[2]) < 1e-6));
    const right = detectFeature(mTopo, triWhere(moving, (c) => Math.abs(c[0] - 10) < 1e-6));
    expect(bestPairing([bottom, right], [top, wall], 30).changed).toBe(false);
  });
});
