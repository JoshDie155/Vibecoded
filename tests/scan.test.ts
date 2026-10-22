import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { describe, expect, it } from 'vitest';
import { IDENTITY, type Rigid, type Vec3, apply, axisAngle, compose, rotate } from '../src/math/align';
import type { Sample } from '../src/math/icp';
import type { FreeMotion } from '../src/math/motion';
import { type ScanTarget, autoSeat, contactScore, scanMotion } from '../src/math/scan';
import { box, concat, cylinder, dPeg, dRing, geometryFrom, ring, transformPositions } from './helpers/meshes';

const DEG = Math.PI / 180;
const opts = { contactDist: 0.3, maxDepth: 2 };
const Z: Vec3 = [0, 0, 1];

/** Reproduzierbare Zufallszahlen (mulberry32). */
function rng(seed: number) {
  return () => {
    seed = (seed + 0x6d2b79f5) | 0;
    let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Flächengewichtete Oberflächenpunkte mit Flächennormalen, reproduzierbar. */
function samplesOf(pos: Float32Array, count: number, seed = 1): Sample[] {
  const tris = pos.length / 9;
  const cum = new Float64Array(tris);
  const normals: Vec3[] = [];
  let total = 0;
  for (let i = 0; i < tris; i++) {
    const o = i * 9;
    const u = [pos[o + 3] - pos[o], pos[o + 4] - pos[o + 1], pos[o + 5] - pos[o + 2]];
    const v = [pos[o + 6] - pos[o], pos[o + 7] - pos[o + 1], pos[o + 8] - pos[o + 2]];
    const n: Vec3 = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    const l = Math.hypot(...n);
    total += l / 2;
    cum[i] = total;
    normals.push(l > 0 ? [n[0] / l, n[1] / l, n[2] / l] : [0, 0, 1]);
  }
  const rand = rng(seed);
  const out: Sample[] = [];
  for (let k = 0; k < count; k++) {
    const r = rand() * total;
    let lo = 0;
    let hi = tris - 1;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (cum[mid] < r) lo = mid + 1;
      else hi = mid;
    }
    let a = rand();
    let b = rand();
    if (a + b > 1) {
      a = 1 - a;
      b = 1 - b;
    }
    const o = lo * 9;
    const p: Vec3 = [0, 1, 2].map((j) => pos[o + j] + a * (pos[o + 3 + j] - pos[o + j]) + b * (pos[o + 6 + j] - pos[o + j])) as Vec3;
    out.push({ p, n: normals[lo] });
  }
  return out;
}

function targetOf(positions: Float32Array, matrix: Rigid = IDENTITY): ScanTarget {
  return { bvh: new MeshBVH(geometryFrom(positions)), positions, matrix };
}

const shift = (t: Vec3): Rigid => ({ R: [...IDENTITY.R], t });
const turn = (angle: number, axis: Vec3 = Z): Rigid => ({ R: axisAngle(axis, angle), t: [0, 0, 0] });
/** Winkelabstand modulo 2π. */
const angDiff = (a: number, b: number) => Math.abs(Math.atan2(Math.sin(a - b), Math.cos(a - b)));
const minZ = (s: Sample[], T: Rigid) => Math.min(...s.map((x) => apply(T, x.p)[2]));

describe('contactScore', () => {
  const fixed = targetOf(box(30, 20, 10));
  const moving = samplesOf(box(30, 20, 6), 1500);

  it('zählt Kontakt bei genau aufliegender Fläche', () => {
    const s = contactScore(moving, shift([0, 0, 10]), fixed, opts);
    expect(s.contact).toBeGreaterThan(0.2);
    expect(s.penetration).toBe(0);
  });

  it('meldet Durchdringung, wenn das Teil hineingedrückt wird', () => {
    const touching = contactScore(moving, shift([0, 0, 10]), fixed, opts);
    const pushed = contactScore(moving, shift([0, 0, 9]), fixed, opts);
    expect(pushed.penetration).toBeGreaterThan(0.1);
    expect(pushed.score).toBeLessThan(touching.score);
  });

  it('hat keinen Kontakt mit Abstand', () => {
    const s = contactScore(moving, shift([0, 0, 13]), fixed, opts);
    expect(s.contact).toBe(0);
    expect(s.penetration).toBe(0);
  });

  it('liefert mit indirektem BVH (ohne Index) dasselbe Ergebnis', () => {
    const positions = box(30, 20, 10);
    const indirect: ScanTarget = { bvh: new MeshBVH(geometryFrom(positions), { indirect: true }), positions, matrix: IDENTITY };
    for (const z of [10, 9.5, 10.2]) {
      expect(contactScore(moving, shift([3, 1, z]), indirect, opts)).toEqual(contactScore(moving, shift([3, 1, z]), fixed, opts));
    }
  });

  it('berücksichtigt die Lage des festen Teils', () => {
    const F: Rigid = { R: axisAngle([1, 2, -0.5], 1.1), t: [40, -12, 7] };
    const s = contactScore(moving, compose(F, shift([0, 0, 10])), targetOf(box(30, 20, 10), F), opts);
    expect(s.contact).toBeGreaterThan(0.2);
    expect(s.penetration).toBe(0);
  });
});

describe('scanMotion: Drehung', () => {
  // D-Zapfen (r 5, Fläche bei x = 3) in D-Bohrung mit 0,1 mm Spiel, steht oben und unten über
  const hole = dRing(12, 5.1, 3.1, 8);
  const peg = dPeg(5, 3, 14);
  const pegSamples = samplesOf(peg, 1200, 7);
  const rot: FreeMotion = { kind: 'rotate', point: [0, 0, 0], axis: Z };

  it('findet die Drehung eines D-Zapfens in der D-Bohrung', () => {
    const T = compose(turn(47 * DEG), shift([0, 0, -3]));
    const r = scanMotion(pegSamples, T, targetOf(hole), rot, [-Math.PI, Math.PI], opts);
    expect(r.flat).toBe(false);
    expect(angDiff(r.best, -47 * DEG)).toBeLessThan(2 * DEG);
    expect(r.ties).toHaveLength(0);
    expect(r.profile).toHaveLength(72);
    // Richtig gedreht: kein Eindringen, Kontakt an den Seitenflächen
    const s = contactScore(pegSamples, r.T, targetOf(hole), opts);
    expect(s.penetration).toBe(0);
    expect(s.contact).toBeGreaterThan(0.2);
  });

  it('findet die Drehung auch bei gedrehtem und verschobenem festen Teil', () => {
    const F: Rigid = { R: axisAngle([0.4, -1, 0.3], 0.8), t: [20, 5, -30] };
    const T = compose(F, compose(turn(-112 * DEG), shift([0, 0, -3])));
    const m: FreeMotion = { kind: 'rotate', point: apply(F, [0, 0, 0]), axis: rotate(F.R, Z) };
    const r = scanMotion(pegSamples, T, targetOf(hole, F), m, [0, 2 * Math.PI], opts);
    expect(r.flat).toBe(false);
    expect(angDiff(r.best, 112 * DEG)).toBeLessThan(2 * DEG);
  });

  it('arbeitet auch in einem Teilbereich und ohne Verfeinerung', () => {
    const T = compose(turn(47 * DEG), shift([0, 0, -3]));
    const part = scanMotion(pegSamples, T, targetOf(hole), rot, [-90 * DEG, 0], opts);
    expect(part.profile).toHaveLength(19);
    expect(angDiff(part.best, -47 * DEG)).toBeLessThan(2 * DEG);
    const coarse = scanMotion(pegSamples, T, targetOf(hole), rot, [-Math.PI, Math.PI], { ...opts, refine: false });
    expect(angDiff(coarse.best, -47 * DEG)).toBeLessThan(5 * DEG);
  });

  it('meldet bei rundem Zapfen in runder Bohrung ein flaches Profil', () => {
    const samples = samplesOf(cylinder(5, 14), 1000, 3);
    const r = scanMotion(samples, compose(turn(30 * DEG), shift([0, 0, -3])), targetOf(ring(12, 5.1, 8)), rot, [-Math.PI, Math.PI], opts);
    expect(r.flat).toBe(true);
    expect(r.best).toBe(0);
  });

  it('Rechteck auf Rechteck: 0° ist am besten, 180° passt genauso', () => {
    const samples = samplesOf(box(30, 20, 6), 1500, 5);
    const m: FreeMotion = { kind: 'rotate', point: [0, 0, 10], axis: Z };
    const r = scanMotion(samples, shift([0, 0, 10]), targetOf(box(30, 20, 10)), m, [-Math.PI, Math.PI], opts);
    expect(r.flat).toBe(false);
    expect(angDiff(r.best, 0)).toBeLessThan(1 * DEG);
    expect(r.ties).toHaveLength(1);
    expect(angDiff(r.ties[0], Math.PI)).toBeLessThan(1.5 * DEG);
  });

  it('Rechteck verdreht aufgesetzt: Drehung wird korrigiert', () => {
    const samples = samplesOf(box(30, 20, 6), 1500, 5);
    const m: FreeMotion = { kind: 'rotate', point: [0, 0, 10], axis: Z };
    const r = scanMotion(samples, compose(shift([0, 0, 10]), turn(23 * DEG)), targetOf(box(30, 20, 10)), m, [-Math.PI, Math.PI], opts);
    expect(angDiff(r.best, -23 * DEG)).toBeLessThan(1.5 * DEG);
    expect(r.ties).toHaveLength(1);
    expect(angDiff(r.ties[0], r.best + Math.PI)).toBeLessThan(1.5 * DEG);
  });

  it('Quadrat auf Quadrat: gleich gute Drehungen bei 90°, 180° und 270°', () => {
    const samples = samplesOf(box(20, 20, 6), 1500, 9);
    const m: FreeMotion = { kind: 'rotate', point: [0, 0, 10], axis: Z };
    const r = scanMotion(samples, compose(shift([0, 0, 10]), turn(10 * DEG)), targetOf(box(20, 20, 10)), m, [-Math.PI, Math.PI], opts);
    expect(Math.min(...[0, 1, 2, 3].map((k) => angDiff(r.best, (k * 90 - 10) * DEG)))).toBeLessThan(1.5 * DEG);
    expect(r.ties).toHaveLength(3);
    const rel = r.ties.map((x) => ((x - r.best + 4 * Math.PI) % (2 * Math.PI)) / DEG).sort((a, b) => a - b);
    [90, 180, 270].forEach((d, i) => expect(Math.abs(rel[i] - d)).toBeLessThan(1.5));
  });
});

describe('scanMotion: Verschiebung', () => {
  const plate = targetOf(box(40, 40, 10));
  const pegSamples = samplesOf(cylinder(5, 10), 1000, 11);
  const slide: FreeMotion = { kind: 'translate', axis: Z };

  it('schiebt einen Zapfen von oben bis zur Berührung auf die Platte', () => {
    const r = scanMotion(pegSamples, shift([0, 0, 13]), plate, slide, [-10, 10], opts);
    expect(r.flat).toBe(false);
    expect(r.best).toBeCloseTo(-3, 1);
    const z = minZ(pegSamples, r.T);
    expect(z).toBeGreaterThan(10 - 0.05);
    expect(z).toBeLessThan(10 + opts.contactDist);
    const s = contactScore(pegSamples, r.T, plate, opts);
    expect(s.penetration).toBe(0);
    expect(s.contact).toBeGreaterThan(0.1);
  });

  it('zieht einen zu tief steckenden Zapfen zurück', () => {
    const r = scanMotion(pegSamples, shift([0, 0, 8.5]), plate, slide, [-10, 10], opts);
    const z = minZ(pegSamples, r.T);
    expect(Math.abs(z - 10)).toBeLessThan(0.05);
    expect(contactScore(pegSamples, r.T, plate, opts).penetration).toBe(0);
  });

  it('behält die Lage, wenn das Profil flach ist', () => {
    // Weit weg vom festen Teil: nirgends Kontakt
    const r = scanMotion(pegSamples, shift([100, 0, 30]), plate, slide, [-5, 5], opts);
    expect(r.flat).toBe(true);
    expect(r.best).toBe(0);
  });

  it('verschiebt in der Ebene zur größten Auflagefläche', () => {
    const samples = samplesOf(box(30, 20, 6), 1500, 5);
    const m: FreeMotion = { kind: 'translate', axis: [1, 0, 0] };
    const r = scanMotion(samples, shift([6, 0, 10]), targetOf(box(30, 20, 10)), m, [-15, 15], opts);
    expect(r.flat).toBe(false);
    expect(Math.abs(r.best + 6)).toBeLessThan(0.5);
  });

  it('lässt einen Zapfen in einer durchgehenden Bohrung, wo er ist', () => {
    // Zapfen ragt oben und unten 3 mm heraus: Kontakt bleibt beim Schieben im Rahmen des Rauschens gleich
    const samples = samplesOf(cylinder(5, 14), 1000, 5);
    const r = scanMotion(samples, shift([0, 0, -3]), targetOf(ring(12, 5.1, 8)), slide, [-14, 14], opts);
    expect(r.flat).toBe(false);
    expect(r.best).toBe(0);
  });
});

describe('autoSeat', () => {
  // D-Zapfen mit Bund oben: Schaft z 0…12, Bund (r 8) z 12…14
  const peg = concat(dPeg(5, 3, 12), transformPositions(cylinder(8, 2), new THREE.Matrix4().makeTranslation(0, 0, 12)));
  const samples = samplesOf(peg, 1000, 13);
  const hole = targetOf(dRing(12, 5.1, 3.1, 8));
  const free: FreeMotion[] = [
    { kind: 'translate', axis: Z },
    { kind: 'rotate', point: [0, 0, 0], axis: Z },
  ];

  it('dreht einen verdrehsicheren Zapfen richtig und schiebt ihn bis zum Bund ein', () => {
    // Bund 3 mm über der Oberseite der Scheibe, Zapfen um 47° verdreht
    const T = compose(turn(47 * DEG), shift([0, 0, -1]));
    const t0 = performance.now();
    const r = autoSeat(samples, T, hole, free, opts);
    const ms = performance.now() - t0;
    expect(ms).toBeLessThan(2000);
    expect(r.flatRotation).toBe(false);
    expect(r.ties).toHaveLength(0);
    // Ergebnis: ungedreht, Bund liegt auf z = 8 auf
    const ex = rotate(r.T.R, [1, 0, 0]);
    expect(Math.abs(Math.atan2(ex[1], ex[0]))).toBeLessThan(2 * DEG);
    expect(r.T.t[2]).toBeGreaterThan(-4 - 0.05);
    expect(r.T.t[2]).toBeLessThan(-4 + opts.contactDist);
    expect(contactScore(samples, r.T, hole, opts).penetration).toBe(0);
    expect(r.info).toMatch(/^Drehung automatisch bestimmt \(-4[678]°\)/);
    expect(r.info).toContain('Anschlag');
  });

  it('meldet bei rundem Zapfen eine frei wählbare Drehung', () => {
    const round = samplesOf(concat(cylinder(5, 12), transformPositions(cylinder(8, 2), new THREE.Matrix4().makeTranslation(0, 0, 12))), 1000, 17);
    const T = compose(turn(30 * DEG), shift([0, 0, -1]));
    const r = autoSeat(round, T, targetOf(ring(12, 5.1, 8)), free, opts);
    expect(r.flatRotation).toBe(true);
    expect(r.info).toContain('Teil ist rund – Drehung frei wählbar');
    // Drehung bleibt, wie sie war; eingeschoben wird trotzdem
    expect(rotate(r.T.R, [1, 0, 0])[1]).toBeCloseTo(Math.sin(30 * DEG), 6);
    expect(Math.abs(r.T.t[2] + 4)).toBeLessThan(opts.contactDist);
  });

  it('liefert bei rechteckiger Auflage die 180°-Alternative', () => {
    // Wie in der Anwendung: nur Punkte in der Nähe der Kontaktfläche
    const samplesBox = samplesOf(box(30, 20, 6), 5000, 5).filter((s) => s.p[2] < 1);
    const plane: FreeMotion[] = [
      { kind: 'rotate', point: [0, 0, 10], axis: Z },
      { kind: 'translate', axis: [1, 0, 0] },
      { kind: 'translate', axis: [0, 1, 0] },
    ];
    const r = autoSeat(samplesBox, compose(shift([5, -3, 10]), turn(15 * DEG)), targetOf(box(30, 20, 10)), plane, opts);
    expect(r.flatRotation).toBe(false);
    expect(r.ties).toHaveLength(1);
    expect(angDiff(r.ties[0], Math.PI)).toBeLessThan(2 * DEG);
    const c = apply(r.T, [0, 0, 0]);
    expect(Math.hypot(c[0], c[1])).toBeLessThan(1);
    const ex = rotate(r.T.R, [1, 0, 0]);
    expect(Math.abs(Math.sin(Math.atan2(ex[1], ex[0])))).toBeLessThan(Math.sin(2 * DEG));
  });

  it('ohne freie Bewegungen bleibt alles, wie es ist', () => {
    const T = shift([1, 2, 3]);
    const r = autoSeat(samples, T, hole, [], opts);
    expect(r.T).toEqual(T);
    expect(r.info).toBe('');
  });
});
