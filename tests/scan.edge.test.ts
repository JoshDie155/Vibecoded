/**
 * Randfälle und realistische Szenarien für das automatische Einpassen (src/math/scan.ts):
 * andere Facettierungen, schräge Lagen, Vorzeichen von Achsen und Normalen, vertauschte Rollen
 * (Bohrung bewegt), winzige und riesige Teile, Sacklöcher, Passfedern und große Netze.
 */
import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { mergeVertices } from 'three/addons/utils/BufferGeometryUtils.js';
import { describe, expect, it } from 'vitest';
import { IDENTITY, type Mat3, type Rigid, type Vec3, apply, axisAngle, compose, rotate } from '../src/math/align';
import type { Sample } from '../src/math/icp';
import { type FreeMotion, applyMotion } from '../src/math/motion';
import { type ScanTarget, autoSeat, contactScore, scanMotion } from '../src/math/scan';
import { box, concat, cylinder, dPeg, dRing, geometryFrom, ring, toPositions, transformPositions } from './helpers/meshes';

const DEG = Math.PI / 180;
const TWO_PI = 2 * Math.PI;
const opts = { contactDist: 0.3, maxDepth: 2 };
const Z: Vec3 = [0, 0, 1];
const FULL: [number, number] = [-Math.PI, Math.PI];
/** Zeitbudget für autoSeat auf großen Netzen: Vorgabe „deutlich unter ~1 s“, mit Reserve für ausgelastete Testrechner. */
const BUDGET_MS = 1500;

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
    const p = [0, 1, 2].map((j) => pos[o + j] + a * (pos[o + 3 + j] - pos[o + j]) + b * (pos[o + 6 + j] - pos[o + j])) as Vec3;
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
/** Winkelabstand modulo einer Symmetrie (z. B. 60° beim Sechskant). */
const angDiffMod = (a: number, b: number, period: number) => {
  const d = (((a - b) % period) + period) % period;
  return Math.min(d, period - d);
};
/** Drehung um Z (Richtung der lokalen X-Achse in der XY-Ebene). */
const yaw = (T: Rigid) => {
  const e = rotate(T.R, [1, 0, 0]);
  return Math.atan2(e[1], e[0]);
};
const minZ = (s: Sample[], T: Rigid) => Math.min(...s.map((x) => apply(T, x.p)[2]));
const maxX = (s: Sample[], T: Rigid) => Math.max(...s.map((x) => apply(T, x.p)[0]));
const scaleM = (s: number) => new THREE.Matrix4().makeScale(s, s, s);
const moveM = (x: number, y: number, z: number) => new THREE.Matrix4().makeTranslation(x, y, z);

/** Starre Transformation umkehren. */
function inv(T: Rigid): Rigid {
  const R = T.R;
  const Rt: Mat3 = [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
  const t = rotate(Rt, T.t);
  return { R: Rt, t: [-t[0], -t[1], -t[2]] };
}

/** Echte Drehung (orthonormal, det = 1) mit endlichen Werten? */
function isRigid(T: Rigid, tol = 1e-9): boolean {
  const R = T.R;
  if (![...R, ...T.t].every(Number.isFinite)) return false;
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      const v = R[3 * i] * R[3 * j] + R[3 * i + 1] * R[3 * j + 1] + R[3 * i + 2] * R[3 * j + 2];
      if (Math.abs(v - (i === j ? 1 : 0)) > tol) return false;
    }
  }
  const det = R[0] * (R[4] * R[8] - R[5] * R[7]) - R[1] * (R[3] * R[8] - R[5] * R[6]) + R[2] * (R[3] * R[7] - R[4] * R[6]);
  return Math.abs(det - 1) < tol;
}

/** Vorzeichenbehaftetes Volumen (positiv bei Normalen nach außen). */
function signedVolume(p: Float32Array): number {
  let v = 0;
  for (let o = 0; o < p.length; o += 9) {
    const [ax, ay, az, bx, by, bz, cx, cy, cz] = [p[o], p[o + 1], p[o + 2], p[o + 3], p[o + 4], p[o + 5], p[o + 6], p[o + 7], p[o + 8]];
    v += ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx);
  }
  return v / 6;
}

/** Profilkanten in Stücke von höchstens maxLen teilen (für dichte Netze). */
function subdivide(profile: [number, number][], maxLen: number): [number, number][] {
  const out: [number, number][] = [];
  for (let k = 0; k < profile.length; k++) {
    const [r0, z0] = profile[k];
    const [r1, z1] = profile[(k + 1) % profile.length];
    const m = Math.max(1, Math.ceil(Math.hypot(r1 - r0, z1 - z0) / maxLen));
    for (let i = 0; i < m; i++) out.push([r0 + ((r1 - r0) * i) / m, z0 + ((z1 - z0) * i) / m]);
  }
  return out;
}

/**
 * Rotationskörper um Z aus einem geschlossenen Profil [r, z] (gegen den Uhrzeigersinn umlaufen,
 * Material links), Normalen nach außen. Nicht indiziert, 9 Werte je Dreieck.
 */
function lathe(profile: [number, number][], segs: number): Float32Array {
  const out: number[] = [];
  const P = (r: number, z: number, a: number) => [r * Math.cos(a), r * Math.sin(a), z];
  for (let k = 0; k < profile.length; k++) {
    const [r0, z0] = profile[k];
    const [r1, z1] = profile[(k + 1) % profile.length];
    for (let s = 0; s < segs; s++) {
      const a0 = (s / segs) * TWO_PI;
      const a1 = ((s + 1) / segs) * TWO_PI;
      const A = P(r0, z0, a0);
      const B = P(r1, z1, a0);
      const C = P(r1, z1, a1);
      const D = P(r0, z0, a1);
      // (D − A) × (B − A) zeigt bei diesem Umlaufsinn nach außen; auf der Achse entartete Dreiecke weglassen
      if (r0 > 0) out.push(...A, ...D, ...B);
      if (r1 > 0) out.push(...D, ...C, ...B);
    }
  }
  return new Float32Array(out);
}

type P2 = [number, number];
const circlePts = (r: number, n: number): P2[] => Array.from({ length: n }, (_, i) => [r * Math.cos((TWO_PI * i) / n), r * Math.sin((TWO_PI * i) / n)]);
/** D-Profil: Kreis r, bei x = flat abgeflacht (Bogen mit n Teilstücken). */
function dPts(r: number, flat: number, n: number): P2[] {
  const a = Math.acos(flat / r);
  return Array.from({ length: n + 1 }, (_, i) => {
    const t = a + ((TWO_PI - 2 * a) * i) / n;
    return [r * Math.cos(t), r * Math.sin(t)] as P2;
  });
}
/** Kreis r mit rechteckiger Nase (Passfeder) bzw. Nut in +x: halbe Breite w, reicht bis x = reach. */
function keyedPts(r: number, w: number, reach: number, n: number): P2[] {
  const a = Math.asin(w / r);
  const pts = Array.from({ length: n + 1 }, (_, i) => {
    const t = a + ((TWO_PI - 2 * a) * i) / n;
    return [r * Math.cos(t), r * Math.sin(t)] as P2;
  });
  pts.push([reach, -w], [reach, w]);
  return pts;
}
/** Prisma aus Umriss und Löchern, Unterseite bei z = 0 (Umlaufsinn ordnet three.js selbst). */
function prism(outer: P2[], holes: P2[][], h: number, steps = 1): Float32Array {
  const s = new THREE.Shape(outer.map(([x, y]) => new THREE.Vector2(x, y)));
  for (const hp of holes) s.holes.push(new THREE.Path(hp.map(([x, y]) => new THREE.Vector2(x, y))));
  return toPositions(new THREE.ExtrudeGeometry(s, { depth: h, steps, bevelEnabled: false }));
}

describe('Testnetze (Voraussetzung für die übrigen Tests)', () => {
  it('alle verwendeten Netze sind geschlossen und nach außen orientiert', () => {
    const cases: [Float32Array, number][] = [
      [lathe([[0, 0], [12, 0], [12, 13], [5.1, 13], [5.1, 5], [0, 5]], 64), Math.PI * (144 * 13 - 5.1 * 5.1 * 8)],
      [lathe(subdivide([[5.1, 0], [12, 0], [12, 8], [5.1, 8]], 1), 64), Math.PI * (144 - 5.1 * 5.1) * 8],
      [prism(keyedPts(5, 1, 6.2, 96), [], 14), 0],
      [prism(circlePts(12, 96), [keyedPts(5.1, 1.1, 6.3, 96)], 8), 0],
      [cylinder(5, 14, 3), 0],
      [ring(12, 5.1, 8, 3), 0],
      [cylinder(5, 14, 2), 0],
    ];
    for (const [p, expected] of cases) {
      const v = signedVolume(p);
      expect(v).toBeGreaterThan(0);
      if (expected) expect(Math.abs(v - expected) / expected).toBeLessThan(0.01);
    }
  });
});

describe('contactScore: Randfälle', () => {
  const movingBox = samplesOf(box(30, 20, 6), 1500, 21);
  const fixedBox = targetOf(box(30, 20, 10));

  it('fein unterteilte feste Fläche liefert denselben Kontakt wie eine grobe', () => {
    const g = new THREE.BoxGeometry(30, 20, 10, 40, 30, 12);
    g.translate(0, 0, 5);
    const fine = targetOf(toPositions(g));
    for (const z of [10, 10.1, 9.6]) {
      const a = contactScore(movingBox, shift([0, 0, z]), fixedBox, opts);
      const b = contactScore(movingBox, shift([0, 0, z]), fine, opts);
      expect(Math.abs(a.contact - b.contact)).toBeLessThan(0.02);
      expect(Math.abs(a.penetration - b.penetration)).toBeLessThan(0.02);
    }
  });

  it('Kontakt nur innerhalb von contactDist, Durchdringung wächst mit der Tiefe', () => {
    const at = (z: number) => contactScore(movingBox, shift([0, 0, z]), fixedBox, opts);
    expect(at(10 + 0.5 * opts.contactDist).contact).toBeGreaterThan(0.2);
    expect(at(10 + 1.5 * opts.contactDist).contact).toBe(0);
    const p1 = at(9.9).penetration;
    const p2 = at(9.6).penetration;
    const p3 = at(8.5).penetration;
    expect(p1).toBeGreaterThan(0);
    expect(p2).toBeGreaterThan(p1);
    expect(p3).toBeGreaterThanOrEqual(p2);
  });

  it('auch von unten: Normalen nach unten werden richtig behandelt', () => {
    // Oberseite des beweglichen Quaders an die Unterseite des festen
    const touching = contactScore(movingBox, shift([0, 0, -6]), fixedBox, opts);
    expect(touching.contact).toBeGreaterThan(0.2);
    expect(touching.penetration).toBe(0);
    const pushed = contactScore(movingBox, shift([0, 0, -5.2]), fixedBox, opts);
    expect(pushed.penetration).toBeGreaterThan(0.1);
  });

  it('nach innen zeigende Normalen der Punkte zählen nicht als Kontakt', () => {
    // Kleinere Grundfläche, damit keine Seitenflächen bündig mit denen des festen Teils liegen
    const small = samplesOf(box(20, 10, 6), 1500, 24);
    expect(contactScore(small, shift([0, 0, 10]), fixedBox, opts).contact).toBeGreaterThan(0.2);
    const flipped = small.map((s) => ({ p: s.p, n: [-s.n[0], -s.n[1], -s.n[2]] as Vec3 }));
    expect(contactScore(flipped, shift([0, 0, 10]), fixedBox, opts).contact).toBe(0);
  });

  it('bewegte Bohrung um festen Zapfen: Kontakt mit Spiel, Durchdringung bei seitlichem Versatz', () => {
    const ringSamples = samplesOf(ring(12, 5.1, 8), 2000, 22);
    const peg = targetOf(cylinder(5, 20));
    const centered = contactScore(ringSamples, shift([0, 0, 5]), peg, opts);
    expect(centered.contact).toBeGreaterThan(0.1);
    expect(centered.penetration).toBe(0);
    const off = contactScore(ringSamples, shift([0.6, 0, 5]), peg, opts);
    expect(off.penetration).toBeGreaterThan(0.01);
  });

  it('Zapfen seitlich in der Bohrung verschoben dringt in die Wand ein', () => {
    const pegSamples = samplesOf(cylinder(5, 14), 1500, 23);
    const hole = targetOf(ring(12, 5.1, 8));
    expect(contactScore(pegSamples, shift([0, 0, -3]), hole, opts).penetration).toBe(0);
    expect(contactScore(pegSamples, shift([0.5, 0, -3]), hole, opts).penetration).toBeGreaterThan(0.01);
  });

  it('doppelte Dreiecke und nicht verschweißte Nähte ändern nichts', () => {
    const p = box(30, 20, 10);
    const doubled = targetOf(concat(p, p));
    // Jede Ecke um höchstens 1 µm verrückt: Nähte nicht mehr exakt geschlossen
    const jitter = p.slice();
    const rand = rng(5);
    for (let i = 0; i < jitter.length; i++) jitter[i] += (rand() - 0.5) * 2e-3;
    const seams = targetOf(jitter);
    for (const z of [10, 9.5]) {
      const ref = contactScore(movingBox, shift([1, 2, z]), fixedBox, opts);
      for (const t of [doubled, seams]) {
        const s = contactScore(movingBox, shift([1, 2, z]), t, opts);
        expect(Math.abs(s.contact - ref.contact)).toBeLessThan(0.02);
        expect(Math.abs(s.penetration - ref.penetration)).toBeLessThan(0.02);
      }
    }
  });

  it('BVH über verschweißter (indizierter) Geometrie liefert dieselben Normalen', () => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(box(30, 20, 10), 3));
    const welded = mergeVertices(g);
    expect(welded.index).not.toBeNull();
    // positions: nicht indizierte Positionen desselben Netzes, wie in ScanTarget beschrieben
    const target: ScanTarget = { bvh: new MeshBVH(welded), positions: toPositions(welded), matrix: IDENTITY };
    for (const z of [10, 9.5]) {
      const ref = contactScore(movingBox, shift([1, 2, z]), fixedBox, opts);
      const s = contactScore(movingBox, shift([1, 2, z]), target, opts);
      expect(Math.abs(s.contact - ref.contact)).toBeLessThan(0.02);
      expect(Math.abs(s.penetration - ref.penetration)).toBeLessThan(0.02);
    }
  });

  it('leere Punktliste: alles 0', () => {
    expect(contactScore([], IDENTITY, fixedBox, opts)).toEqual({ contact: 0, penetration: 0, score: 0 });
  });
});

describe('scanMotion: Drehung, Randfälle', () => {
  const hole = targetOf(dRing(12, 5.1, 3.1, 8));
  const peg = dPeg(5, 3, 14);
  const pegSamples = samplesOf(peg, 1200, 31);
  const rot: FreeMotion = { kind: 'rotate', point: [0, 0, 0], axis: Z };
  const start = (a: number) => compose(turn(a), shift([0, 0, -3]));

  it('Sechskant in Sechskantbohrung: fünf gleich gute Alternativen im 60°-Raster', () => {
    const hex = samplesOf(cylinder(5, 14, 3), 1200, 32);
    const r = scanMotion(hex, start(20 * DEG), targetOf(ring(12, 5.1, 8, 3)), rot, FULL, opts);
    expect(r.flat).toBe(false);
    expect(angDiffMod(r.best, -20 * DEG, 60 * DEG)).toBeLessThan(1.5 * DEG);
    expect(r.ties).toHaveLength(5);
    const rel = r.ties.map((x) => (((x - r.best) % TWO_PI) + TWO_PI) % TWO_PI / DEG).sort((a, b) => a - b);
    [60, 120, 180, 240, 300].forEach((d, i) => expect(Math.abs(rel[i] - d)).toBeLessThan(2));
  });

  it('Vierkant in Vierkantbohrung: Alternativen bei 90°, 180° und 270°', () => {
    const sq = samplesOf(cylinder(5, 14, 2), 1200, 33);
    const r = scanMotion(sq, start(-33 * DEG), targetOf(ring(12, 5.1, 8, 2)), rot, FULL, opts);
    expect(r.flat).toBe(false);
    expect(angDiffMod(r.best, 33 * DEG, 90 * DEG)).toBeLessThan(1.5 * DEG);
    expect(r.ties).toHaveLength(3);
  });

  it('Korrektur nahe ±180° über die Periodengrenze hinweg', () => {
    for (const a of [178, 180, -179, 183]) {
      const r = scanMotion(pegSamples, start(a * DEG), hole, rot, FULL, opts);
      expect(r.flat).toBe(false);
      expect(r.ties).toHaveLength(0);
      expect(angDiff(r.best, -a * DEG)).toBeLessThan(2 * DEG);
      expect(r.best).toBeGreaterThanOrEqual(-Math.PI);
      expect(r.best).toBeLessThan(Math.PI);
      expect(angDiff(yaw(r.T), 0)).toBeLessThan(2 * DEG);
    }
  });

  it('Bereich [0, 2π]: kleine negative Korrektur kommt als Winkel knapp unter 2π', () => {
    const r = scanMotion(pegSamples, start(5 * DEG), hole, rot, [0, TWO_PI], opts);
    expect(r.best).toBeGreaterThanOrEqual(0);
    expect(r.best).toBeLessThan(TWO_PI);
    expect(angDiff(r.best, -5 * DEG)).toBeLessThan(1.5 * DEG);
    expect(r.best).toBeGreaterThan(Math.PI);
  });

  it('umgekehrte Drehachse: Betrag mit umgekehrtem Vorzeichen, gleiche Endlage', () => {
    const neg: FreeMotion = { kind: 'rotate', point: [0, 0, 0], axis: [0, 0, -1] };
    const r = scanMotion(pegSamples, start(47 * DEG), hole, neg, FULL, opts);
    expect(angDiff(r.best, 47 * DEG)).toBeLessThan(2 * DEG);
    expect(angDiff(yaw(r.T), 0)).toBeLessThan(2 * DEG);
  });

  it('Drehpunkt an anderer Stelle der Achse und nicht normierte Achse', () => {
    const m: FreeMotion = { kind: 'rotate', point: [0, 0, 40], axis: [0, 0, 7] };
    const r = scanMotion(pegSamples, start(-63 * DEG), hole, m, FULL, opts);
    expect(angDiff(r.best, 63 * DEG)).toBeLessThan(2 * DEG);
    expect(angDiff(yaw(r.T), 0)).toBeLessThan(2 * DEG);
    // Drehung um die Achse verschiebt nichts
    expect(Math.hypot(r.T.t[0], r.T.t[1])).toBeLessThan(1e-9);
  });

  it('vertauschte Rollen: D-Bohrung bewegt, D-Zapfen fest', () => {
    const ringSamples = samplesOf(dRing(12, 5.1, 3.1, 8), 2000, 34);
    const fixedPeg = targetOf(transformPositions(peg, moveM(0, 0, -3)));
    const r = scanMotion(ringSamples, turn(-60 * DEG), fixedPeg, rot, FULL, opts);
    expect(r.flat).toBe(false);
    expect(angDiff(r.best, 60 * DEG)).toBeLessThan(2 * DEG);
  });

  it('grob facettierter Zapfen in fein facettierter Bohrung', () => {
    const coarse = samplesOf(dPeg(5, 3, 14, 6), 1200, 35);
    const fineHole = targetOf(dRing(12, 5.1, 3.1, 8, 128));
    const r = scanMotion(coarse, start(-81 * DEG), fineHole, rot, FULL, opts);
    expect(r.flat).toBe(false);
    expect(angDiff(r.best, 81 * DEG)).toBeLessThan(2 * DEG);
  });

  it('großes Spiel (0,5 mm, mehr als contactDist): Mitte des Spiels', () => {
    const loose = targetOf(dRing(12, 5.5, 3.5, 8));
    const r = scanMotion(pegSamples, start(38 * DEG), loose, rot, FULL, opts);
    expect(r.flat).toBe(false);
    expect(angDiff(r.best, -38 * DEG)).toBeLessThan(2 * DEG);
    expect(contactScore(pegSamples, r.T, loose, opts).penetration).toBe(0);
  });

  it('Passfeder: runder Zapfen mit Nase in Bohrung mit Nut', () => {
    const keyPeg = samplesOf(prism(keyedPts(5, 1, 6.2, 96), [], 14), 1500, 36);
    const keyHole = targetOf(prism(circlePts(12, 96), [keyedPts(5.1, 1.1, 6.3, 96)], 8));
    for (const a of [-140, 25, 100]) {
      const r = scanMotion(keyPeg, start(a * DEG), keyHole, rot, FULL, opts);
      expect(r.flat).toBe(false);
      expect(r.ties).toHaveLength(0);
      expect(angDiff(r.best, -a * DEG)).toBeLessThan(2 * DEG);
    }
  });

  it('D-Zapfen in runder Bohrung: Drehung egal, Profil flach', () => {
    const r = scanMotion(pegSamples, start(40 * DEG), targetOf(ring(12, 5.1, 8)), rot, FULL, opts);
    expect(r.flat).toBe(true);
    expect(r.best).toBe(0);
  });

  it('runder Zapfen in grob facettierter runder Bohrung bleibt flach', () => {
    const round = samplesOf(cylinder(5, 14), 1200, 37);
    const r = scanMotion(round, start(10 * DEG), targetOf(ring(12, 5.1, 8, 13)), rot, FULL, opts);
    expect(r.flat).toBe(true);
  });

  it('winzige Teile (Ø 0,1 mm): gleiche Lösung bei mitskalierten Toleranzen', () => {
    const s = 0.01;
    const tiny = samplesOf(transformPositions(peg, scaleM(s)), 1200, 31);
    const tinyHole = targetOf(transformPositions(dRing(12, 5.1, 3.1, 8), scaleM(s)));
    const T = compose(turn(47 * DEG), shift([0, 0, -3 * s]));
    const r = scanMotion(tiny, T, tinyHole, rot, FULL, { contactDist: opts.contactDist * s, maxDepth: opts.maxDepth * s });
    expect(r.flat).toBe(false);
    expect(angDiff(r.best, -47 * DEG)).toBeLessThan(2 * DEG);
  });

  it('riesige Teile (Ø 1000 mm): gleiche Lösung bei mitskalierten Toleranzen', () => {
    const s = 100;
    const big = samplesOf(transformPositions(peg, scaleM(s)), 1200, 31);
    const bigHole = targetOf(transformPositions(dRing(12, 5.1, 3.1, 8), scaleM(s)));
    const T = compose(turn(-128 * DEG), shift([0, 0, -3 * s]));
    const m: FreeMotion = { kind: 'rotate', point: [0, 0, 0], axis: Z };
    const r = scanMotion(big, T, bigHole, m, FULL, { contactDist: opts.contactDist * s, maxDepth: opts.maxDepth * s });
    expect(r.flat).toBe(false);
    expect(angDiff(r.best, 128 * DEG)).toBeLessThan(2 * DEG);
  });

  it('schräg im Raum liegende Teile, großer Drehfehler', () => {
    const F: Rigid = { R: axisAngle([-0.7, 0.2, 1], 2.4), t: [-150, 80, 33] };
    const T = compose(F, start(170 * DEG));
    const m: FreeMotion = { kind: 'rotate', point: apply(F, [0, 0, 5]), axis: rotate(F.R, [0, 0, -1]) };
    const r = scanMotion(pegSamples, T, targetOf(dRing(12, 5.1, 3.1, 8), F), m, FULL, opts);
    expect(r.flat).toBe(false);
    expect(angDiff(yaw(compose(inv(F), r.T)), 0)).toBeLessThan(2 * DEG);
  });

  it('weniger Schritte: Profil mit steps Einträgen, Winkel trotzdem gefunden', () => {
    const r = scanMotion(pegSamples, start(-12 * DEG), hole, rot, FULL, { ...opts, steps: 36 });
    expect(r.profile).toHaveLength(36);
    expect(r.profile.every((p) => Number.isFinite(p.score) && p.amount >= -Math.PI && p.amount < Math.PI)).toBe(true);
    expect(angDiff(r.best, 12 * DEG)).toBeLessThan(2 * DEG);
  });

  it('Bereich über mehr als eine Umdrehung: keine Alternative, die dieselbe Lage ist', () => {
    const r = scanMotion(pegSamples, start(47 * DEG), hole, rot, [-TWO_PI, TWO_PI], opts);
    expect(angDiff(r.best, -47 * DEG)).toBeLessThan(2 * DEG);
    // Der D-Zapfen passt nur in einer Lage: Alternativen dürfen nicht nur 360° weiter dieselbe sein
    expect(r.ties.filter((x) => angDiff(x, r.best) < 5 * DEG)).toHaveLength(0);
  });

  it('Deckel am Scharnier: dreht bis zur Auflage statt in die Luft', () => {
    // Deckel 30 × 20 × 2 auf einem Quader 30 × 20 × 10, Scharnier entlang Y an der linken Oberkante
    const fixed = targetOf(box(30, 20, 10));
    const lid = samplesOf(box(30, 20, 2), 1500, 61);
    const hinge: FreeMotion = { kind: 'rotate', point: [-15, 0, 10], axis: [0, 1, 0] };
    const closed = contactScore(lid, shift([0, 0, 10]), fixed, opts);
    expect(closed.contact).toBeGreaterThan(0.3);
    // Auch ein fast geschlossener Deckel (3°) muss zu, nicht in die Mitte des freien Schwenkbereichs
    for (const open of [3, 60, 120]) {
      const T0 = applyMotion(shift([0, 0, 10]), hinge, -open * DEG);
      const r = scanMotion(lid, T0, fixed, hinge, FULL, opts);
      expect(r.flat).toBe(false);
      expect(angDiff(r.best, open * DEG)).toBeLessThan(1 * DEG);
      const s = contactScore(lid, r.T, fixed, opts);
      // Am Anschlag sind einzelne Punkte knapp (Hundertstel mm) im Inneren zulässig: das Abtasten toleriert
      // 0,2 % Ausreißer, damit facettierte Kontaktflächen nicht zu früh stoppen. Genau anlegen übernimmt settle.ts.
      expect(s.penetration).toBeLessThan(1e-3);
      expect(s.contact).toBeGreaterThan(0.5 * closed.contact);
    }
  });

  it('Motorwelle mit flacher D-Abflachung (0,5 mm) findet ihre Drehung', () => {
    const shaft = samplesOf(dPeg(2.5, 2.0, 12), 1200, 62);
    const shaftHole = targetOf(dRing(8, 2.6, 2.1, 6));
    for (const a of [30, 180, -100]) {
      const r = scanMotion(shaft, start(a * DEG), shaftHole, rot, FULL, opts);
      expect(r.flat).toBe(false);
      expect(r.ties).toHaveLength(0);
      expect(angDiff(r.best, -a * DEG)).toBeLessThan(2 * DEG);
    }
  });

  it('Rechteck 30 × 26 (nicht quadratisch): nur die 180°-Alternative', () => {
    const s = samplesOf(box(30, 26, 6), 1500, 63);
    const m: FreeMotion = { kind: 'rotate', point: [0, 0, 10], axis: Z };
    const r = scanMotion(s, compose(shift([0, 0, 10]), turn(12 * DEG)), targetOf(box(30, 26, 10)), m, FULL, opts);
    expect(angDiff(r.best, -12 * DEG)).toBeLessThan(1.5 * DEG);
    expect(r.ties).toHaveLength(1);
    expect(angDiff(r.ties[0], r.best + Math.PI)).toBeLessThan(1.5 * DEG);
  });

  it('T ist genau T0 um best bewegt', () => {
    const T0 = start(47 * DEG);
    const r = scanMotion(pegSamples, T0, hole, rot, FULL, opts);
    const expected = applyMotion(T0, rot, r.best);
    r.T.R.forEach((v, i) => expect(v).toBeCloseTo(expected.R[i], 12));
    r.T.t.forEach((v, i) => expect(v).toBeCloseTo(expected.t[i], 12));
  });
});

describe('scanMotion: Verschiebung, Randfälle', () => {
  const plate = targetOf(box(40, 40, 10));
  const pegSamples = samplesOf(cylinder(5, 10), 1000, 41);

  it('Sackloch: Zapfen wird bis zum Grund eingeschoben', () => {
    // Block r 12, Höhe 13, Bohrung r 5,1 von z = 5 bis 13
    const block = targetOf(lathe([[0, 0], [12, 0], [12, 13], [5.1, 13], [5.1, 5], [0, 5]], 200));
    const samples = samplesOf(cylinder(5, 14), 1500, 42);
    const r = scanMotion(samples, shift([0, 0, 8]), block, { kind: 'translate', axis: Z }, [-10, 10], opts);
    expect(r.flat).toBe(false);
    const z = minZ(samples, r.T);
    expect(z).toBeGreaterThan(5 - 0.05);
    expect(z).toBeLessThan(5 + opts.contactDist);
    expect(contactScore(samples, r.T, block, opts).penetration).toBe(0);
  });

  it('umgekehrte Schiebeachse: Betrag mit umgekehrtem Vorzeichen, gleiche Endlage', () => {
    const r = scanMotion(pegSamples, shift([0, 0, 13]), plate, { kind: 'translate', axis: [0, 0, -1] }, [-10, 10], opts);
    expect(r.best).toBeCloseTo(3, 1);
    expect(minZ(pegSamples, r.T)).toBeGreaterThan(10 - 0.05);
    expect(minZ(pegSamples, r.T)).toBeLessThan(10 + opts.contactDist);
  });

  it('nicht normierte Schiebeachse: Betrag trotzdem in mm', () => {
    const r = scanMotion(pegSamples, shift([0, 0, 13]), plate, { kind: 'translate', axis: [0, 0, 5] }, [-10, 10], opts);
    expect(r.best).toBeCloseTo(-3, 1);
    expect(r.T.t[2]).toBeCloseTo(10, 1);
  });

  it('schräge Schiebeachse im Raum (fester Teil gedreht und verschoben)', () => {
    const F: Rigid = { R: axisAngle([0.5, 1, -0.2], 1.3), t: [12, -40, 25] };
    const m: FreeMotion = { kind: 'translate', axis: rotate(F.R, Z) };
    const r = scanMotion(pegSamples, compose(F, shift([0, 0, 14])), targetOf(box(40, 40, 10), F), m, [-10, 10], opts);
    expect(r.best).toBeCloseTo(-4, 1);
    const rel = compose(inv(F), r.T);
    expect(minZ(pegSamples, rel)).toBeGreaterThan(10 - 0.05);
    expect(minZ(pegSamples, rel)).toBeLessThan(10 + opts.contactDist);
  });

  it('schiebt einen Quader auf der Platte bis an eine Wand', () => {
    // Platte x −20…20, z 0…10; Wand x 20…25, z 0…20
    const wall = targetOf(concat(box(40, 40, 10), transformPositions(box(5, 40, 20), moveM(22.5, 0, 0))));
    const samples = samplesOf(box(10, 10, 6), 1500, 43);
    const r = scanMotion(samples, shift([11, 0, 10]), wall, { kind: 'translate', axis: [1, 0, 0] }, [-10, 10], opts);
    expect(r.flat).toBe(false);
    const x = maxX(samples, r.T);
    expect(x).toBeGreaterThan(20 - opts.contactDist);
    expect(x).toBeLessThan(20 + 0.05);
    expect(contactScore(samples, r.T, wall, opts).penetration).toBe(0);
  });

  it('verschiebt in der Ebene auch bei kleinem Versatz (3 mm) zur größten Auflagefläche', () => {
    // 30 × 20 auf 30 × 20, um 3 mm (10 %) versetzt: mittig gibt es deutlich mehr Kontakt, das Profil ist nicht flach
    const samples = samplesOf(box(30, 20, 6), 1500, 5);
    const fixed = targetOf(box(30, 20, 10));
    const T0 = shift([3, 0, 10]);
    expect(contactScore(samples, shift([0, 0, 10]), fixed, opts).contact).toBeGreaterThan(1.08 * contactScore(samples, T0, fixed, opts).contact);
    const r = scanMotion(samples, T0, fixed, { kind: 'translate', axis: [1, 0, 0] }, [-15, 15], opts);
    expect(r.flat).toBe(false);
    expect(Math.abs(r.best + 3)).toBeLessThan(0.5);
  });

  it('winzige Teile: Zapfen setzt genau auf', () => {
    const s = 0.01;
    const tiny = samplesOf(transformPositions(cylinder(5, 10), scaleM(s)), 1000, 41);
    const tinyPlate = targetOf(transformPositions(box(40, 40, 10), scaleM(s)));
    const o = { contactDist: opts.contactDist * s, maxDepth: opts.maxDepth * s };
    const r = scanMotion(tiny, shift([0, 0, 13 * s]), tinyPlate, { kind: 'translate', axis: Z }, [-10 * s, 10 * s], o);
    expect(r.flat).toBe(false);
    const z = minZ(tiny, r.T);
    expect(z).toBeGreaterThan(10 * s - 0.05 * s);
    expect(z).toBeLessThan(10 * s + o.contactDist);
  });

  it('riesige Teile: Zapfen setzt genau auf', () => {
    const s = 100;
    const big = samplesOf(transformPositions(cylinder(5, 10), scaleM(s)), 1000, 41);
    const bigPlate = targetOf(transformPositions(box(40, 40, 10), scaleM(s)));
    const o = { contactDist: opts.contactDist * s, maxDepth: opts.maxDepth * s };
    const r = scanMotion(big, shift([0, 0, 13 * s]), bigPlate, { kind: 'translate', axis: Z }, [-10 * s, 10 * s], o);
    const z = minZ(big, r.T);
    expect(z).toBeGreaterThan(10 * s - 0.05 * s);
    expect(z).toBeLessThan(10 * s + o.contactDist);
  });

  it('entartete Achse (Länge 0) und leere Punktliste: nichts bewegt, keine NaN', () => {
    const T = shift([0, 0, 13]);
    const r = scanMotion(pegSamples, T, plate, { kind: 'translate', axis: [0, 0, 0] }, [-10, 10], opts);
    expect(isRigid(r.T)).toBe(true);
    expect(r.T.t).toEqual([0, 0, 13]);
    const e = scanMotion([], T, plate, { kind: 'translate', axis: Z }, [-10, 10], opts);
    expect(e.best).toBe(0);
    expect(e.flat).toBe(true);
    expect(e.T.t).toEqual([0, 0, 13]);
  });
});

describe('autoSeat: Randfälle', () => {
  const collar = transformPositions(cylinder(8, 2), moveM(0, 0, 12));
  const dCollarPeg = concat(dPeg(5, 3, 12), collar);
  const samples = samplesOf(dCollarPeg, 1000, 51);
  const hole = dRing(12, 5.1, 3.1, 8);
  const holeT = targetOf(hole);
  const free: FreeMotion[] = [
    { kind: 'translate', axis: Z },
    { kind: 'rotate', point: [0, 0, 0], axis: Z },
  ];

  it('D-Zapfen mit Bund, der noch ganz über der Bohrung steht, wird gedreht und eingeschoben', () => {
    // Spitze 1 mm über bzw. bündig mit der Oberseite der Scheibe, um 47° verdreht
    for (const z of [9, 8]) {
      const T = compose(turn(47 * DEG), shift([0, 0, z]));
      const r = autoSeat(samples, T, holeT, free, opts);
      expect(angDiff(yaw(r.T), 0)).toBeLessThan(2 * DEG);
      expect(r.T.t[2]).toBeGreaterThan(-4 - 0.05);
      expect(r.T.t[2]).toBeLessThan(-4 + opts.contactDist);
      expect(contactScore(samples, r.T, holeT, opts).penetration).toBe(0);
    }
  });

  it('runder Zapfen mit Bund 4 mm über der Bohrung wird bis zum Bund eingeschoben', () => {
    const round = samplesOf(concat(cylinder(5, 12), collar), 1000, 64);
    for (const z of [9, 12]) {
      const r = autoSeat(round, compose(turn(30 * DEG), shift([0, 0, z])), targetOf(ring(12, 5.1, 8)), free, opts);
      expect(r.flatRotation).toBe(true);
      expect(Math.abs(r.T.t[2] + 4)).toBeLessThan(opts.contactDist);
    }
  });

  it('D-Zapfen mit Bund in beliebiger Weltlage, Achsen mit umgekehrtem Vorzeichen', () => {
    const F: Rigid = { R: axisAngle([0.3, -1, 0.6], 2.1), t: [55, -7, 120] };
    const axis = rotate(F.R, [0, 0, -1]);
    const worldFree: FreeMotion[] = [
      { kind: 'translate', axis },
      { kind: 'rotate', point: apply(F, [0, 0, 3]), axis },
    ];
    const T = compose(F, compose(turn(-120 * DEG), shift([0, 0, -1])));
    const target = targetOf(hole, F);
    const r = autoSeat(samples, T, target, worldFree, opts);
    expect(r.flatRotation).toBe(false);
    const rel = compose(inv(F), r.T);
    expect(angDiff(yaw(rel), 0)).toBeLessThan(2 * DEG);
    expect(rel.t[2]).toBeGreaterThan(-4 - 0.05);
    expect(rel.t[2]).toBeLessThan(-4 + opts.contactDist);
    expect(contactScore(samples, r.T, target, opts).penetration).toBe(0);
  });

  it('Sechskant mit Bund: Alternativen im 60°-Raster relativ zur neuen Lage', () => {
    const hexPeg = samplesOf(concat(cylinder(5, 12, 3), collar), 1000, 52);
    const r = autoSeat(hexPeg, compose(turn(20 * DEG), shift([0, 0, -1])), targetOf(ring(12, 5.1, 8, 3)), free, opts);
    expect(r.flatRotation).toBe(false);
    expect(angDiffMod(yaw(r.T), 0, 60 * DEG)).toBeLessThan(2 * DEG);
    expect(r.ties).toHaveLength(5);
    const rel = [...r.ties].sort((a, b) => a - b).map((x) => x / DEG);
    [60, 120, 180, 240, 300].forEach((d, i) => expect(Math.abs(rel[i] - d)).toBeLessThan(2));
    expect(Math.abs(r.T.t[2] + 4)).toBeLessThan(opts.contactDist);
    expect(r.info).toContain('passt auch um');
  });

  it('Passfeder mit Bund: richtige Drehung und bis zum Bund eingeschoben', () => {
    const keyPeg = samplesOf(concat(prism(keyedPts(5, 1, 6.2, 96), [], 12), collar), 1000, 53);
    const keyHole = targetOf(prism(circlePts(12, 96), [keyedPts(5.1, 1.1, 6.3, 96)], 8));
    const r = autoSeat(keyPeg, compose(turn(-95 * DEG), shift([0, 0, -1])), keyHole, free, opts);
    expect(r.flatRotation).toBe(false);
    expect(r.ties).toHaveLength(0);
    expect(angDiff(yaw(r.T), 0)).toBeLessThan(2 * DEG);
    expect(Math.abs(r.T.t[2] + 4)).toBeLessThan(opts.contactDist);
  });

  it('kleines Teil auf großer Platte: Drehung frei, Lage bleibt', () => {
    const small = samplesOf(box(10, 10, 6), 4000, 54).filter((s) => s.p[2] < 1);
    const plane: FreeMotion[] = [
      { kind: 'rotate', point: [0, 0, 10], axis: Z },
      { kind: 'translate', axis: [1, 0, 0] },
      { kind: 'translate', axis: [0, 1, 0] },
    ];
    const T = compose(shift([5, 3, 10]), turn(20 * DEG));
    const r = autoSeat(small, T, targetOf(box(40, 40, 10)), plane, opts);
    expect(r.flatRotation).toBe(true);
    const c = apply(r.T, [0, 0, 0]);
    expect(Math.hypot(c[0] - 5, c[1] - 3)).toBeLessThan(0.5);
    expect(angDiff(yaw(r.T), 20 * DEG)).toBeLessThan(1 * DEG);
  });

  it('Rechteck auf schräg liegendem Rechteck, um 80° verdreht aufgesetzt', () => {
    const F: Rigid = { R: axisAngle([0.3, 1, 0.2], 2.0), t: [10, -20, 5] };
    const flatSamples = samplesOf(box(30, 20, 6), 5000, 55).filter((s) => s.p[2] < 1);
    const plane: FreeMotion[] = [
      { kind: 'rotate', point: apply(F, [0, 0, 10]), axis: rotate(F.R, Z) },
      { kind: 'translate', axis: rotate(F.R, [1, 0, 0]) },
      { kind: 'translate', axis: rotate(F.R, [0, 1, 0]) },
    ];
    const T = compose(F, compose(shift([0, 0, 10]), turn(80 * DEG)));
    const r = autoSeat(flatSamples, T, targetOf(box(30, 20, 10), F), plane, opts);
    const rel = compose(inv(F), r.T);
    expect(r.flatRotation).toBe(false);
    expect(angDiffMod(yaw(rel), 0, Math.PI)).toBeLessThan(2 * DEG);
    expect(Math.hypot(rel.t[0], rel.t[1])).toBeLessThan(1);
    expect(rel.t[2]).toBeCloseTo(10, 6);
    expect(r.ties).toHaveLength(1);
    expect(angDiff(r.ties[0], Math.PI)).toBeLessThan(2 * DEG);
  });

  it('Rechteck leicht versetzt (2 mm / 1 mm) aufgesetzt: wird mittig und gerade ausgerichtet', () => {
    const flatSamples = samplesOf(box(30, 20, 6), 5000, 55).filter((s) => s.p[2] < 1);
    const fixed = targetOf(box(30, 20, 10));
    const plane: FreeMotion[] = [
      { kind: 'rotate', point: [0, 0, 10], axis: Z },
      { kind: 'translate', axis: [1, 0, 0] },
      { kind: 'translate', axis: [0, 1, 0] },
    ];
    const r = autoSeat(flatSamples, compose(shift([2, -1, 10]), turn(15 * DEG)), fixed, plane, opts);
    expect(angDiffMod(yaw(r.T), 0, Math.PI)).toBeLessThan(2 * DEG);
    expect(Math.hypot(r.T.t[0], r.T.t[1])).toBeLessThan(1);
    // Die Endlage hat (fast) den Kontakt der mittigen, geraden Lage
    const best = contactScore(flatSamples, shift([0, 0, 10]), fixed, opts).contact;
    expect(contactScore(flatSamples, r.T, fixed, opts).contact).toBeGreaterThan(0.97 * best);
  });

  it('leere Punktliste: Lage bleibt, keine NaN', () => {
    const T = compose(turn(10 * DEG), shift([1, 2, 3]));
    const r = autoSeat([], T, holeT, free, opts);
    expect(isRigid(r.T)).toBe(true);
    r.T.t.forEach((v, i) => expect(v).toBeCloseTo(T.t[i], 12));
    expect(r.ties).toEqual([]);
  });

  it('großes Netz (≈ 200 000 Dreiecke): autoSeat mit 1000 Punkten bleibt schnell', () => {
    // Runde Bohrung als Rotationskörper, Innenwand 2000 × 50 Vierecke
    const dense = lathe(subdivide([[5.1, 0], [12, 0], [12, 8], [5.1, 8]], 0.16), 2000);
    expect(dense.length / 9).toBeGreaterThan(200000);
    const target = targetOf(dense);
    const round = samplesOf(concat(cylinder(5, 12), collar), 1000, 56);
    const t0 = performance.now();
    const r = autoSeat(round, compose(turn(30 * DEG), shift([0, 0, -1])), target, free, opts);
    const ms = performance.now() - t0;
    expect(r.flatRotation).toBe(true);
    expect(Math.abs(r.T.t[2] + 4)).toBeLessThan(opts.contactDist);
    expect(ms).toBeLessThan(BUDGET_MS);
  });

  it('großes Netz mit D-Bohrung (≈ 200 000 Dreiecke): Drehung und Anschlag, schnell', () => {
    const dense = prism(circlePts(12, 1000), [dPts(5.1, 3.1, 700)], 8, 60);
    expect(dense.length / 9).toBeGreaterThan(200000);
    const target = targetOf(dense);
    const t0 = performance.now();
    const r = autoSeat(samples, compose(turn(47 * DEG), shift([0, 0, -1])), target, free, opts);
    const ms = performance.now() - t0;
    expect(angDiff(yaw(r.T), 0)).toBeLessThan(2 * DEG);
    expect(Math.abs(r.T.t[2] + 4)).toBeLessThan(opts.contactDist);
    expect(ms).toBeLessThan(BUDGET_MS);
  });
});
