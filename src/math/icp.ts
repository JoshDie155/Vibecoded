/**
 * Feinausrichtung (ICP, "Iterative Closest Point") nahe der Kontaktstelle:
 * Eckpunkte des beweglichen Teils in der Nähe der Markierungen werden wiederholt auf die
 * nächstgelegenen Punkte der Oberfläche des festen Teils gezogen.
 */
import * as THREE from 'three';
import { MeshBVH } from 'three-mesh-bvh';
import { MeshSurfaceSampler } from 'three/addons/math/MeshSurfaceSampler.js';
import { bvhTriangleNormal } from './bvhtri';
import { type Rigid, type Vec3, apply, compose, dist, dot, rotate, solveRigid } from './align';

export interface Sample {
  p: Vec3;
  n: Vec3;
}

export interface IcpOptions {
  /** Nur Eckpunkte innerhalb dieses Radius um die Kontaktmarkierungen werden verwendet. */
  regionRadius: number;
  /** Paare mit größerem Abstand werden ignoriert (keine echte Kontaktfläche). */
  maxPairDist: number;
  iterations?: number;
  maxSamples?: number;
}

const toV = (v: THREE.Vector3): Vec3 => [v.x, v.y, v.z];

/**
 * Gleichmäßig über die Oberfläche verteilte Punkte (flächengewichtet) – funktioniert auch bei
 * groben CAD-Netzen, die nur aus wenigen großen Dreiecken bestehen.
 */
/** Zufallszahlen mit festem Startwert (mulberry32). */
function seeded(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export function sampleSurface(geom: THREE.BufferGeometry, count: number): Sample[] {
  // Flächennormalen statt geglätteter Normalen, damit Kanten sauber getrennt bleiben
  const flat = new THREE.BufferGeometry();
  flat.setAttribute('position', geom.getAttribute('position'));
  flat.computeVertexNormals();
  const sampler = new MeshSurfaceSampler(new THREE.Mesh(flat));
  // Fester Startwert: dasselbe Teil ergibt immer dieselben Punkte – und damit dasselbe Ergebnis
  // (setRandomGenerator gibt es seit three r148, die Typen kennen es noch nicht)
  (sampler as unknown as { setRandomGenerator(f: () => number): void }).setRandomGenerator(seeded(0x9e3779b9 ^ count ^ geom.getAttribute('position').count));
  sampler.build();
  const v = new THREE.Vector3();
  const n = new THREE.Vector3();
  const out: Sample[] = [];
  for (let i = 0; i < count; i++) {
    sampler.sample(v, n);
    out.push({ p: toV(v), n: toV(n.normalize()) });
  }
  return out;
}


/**
 * Prüft, ob zwei Lagen desselben Teils geometrisch gleich aussehen (symmetrisches Teil):
 * D = A⁻¹·B bildet das Teil dann auf sich selbst ab.
 */
export function sameShape(bvh: MeshBVH, samples: Sample[], A: THREE.Matrix4, B: THREE.Matrix4, tol: number): boolean {
  const D = A.clone().invert().multiply(B);
  const p = new THREE.Vector3();
  const target = { point: new THREE.Vector3(), distance: 0, faceIndex: 0 };
  const step = Math.max(1, Math.floor(samples.length / 400));
  for (let i = 0; i < samples.length; i += step) {
    p.set(...samples[i].p).applyMatrix4(D);
    if (!bvh.closestPointToPoint(p, target, 0, tol)) return false;
  }
  return true;
}

/**
 * @param movingSamples Oberflächenpunkte des beweglichen Teils (lokale Koordinaten)
 * @param T           aktuelle Transformation lokal → Welt des beweglichen Teils
 * @param fixedBVH    BVH des festen Teils (lokale Koordinaten)
 * @param fixedMatrix Transformation lokal → Welt des festen Teils
 * @param contacts    Kontaktpunkte in Weltkoordinaten
 * @returns verbesserte Transformation, oder null wenn nicht genug Kontakt gefunden wurde
 */
export function refineICP(
  movingSamples: Sample[],
  T: Rigid,
  fixedBVH: MeshBVH,
  fixedMatrix: THREE.Matrix4,
  contacts: Vec3[],
  opts: IcpOptions,
): { T: Rigid; rmsd: number; pairs: number } | null {
  const iterations = opts.iterations ?? 25;
  const maxSamples = opts.maxSamples ?? 3000;

  // Nur Oberflächenpunkte in der Kontaktregion verwenden
  const local = movingSamples.filter((s) => {
    const w = apply(T, s.p);
    return contacts.some((c) => dist(c, w) < opts.regionRadius);
  });
  if (local.length < 6) return null;
  const step = Math.max(1, Math.floor(local.length / maxSamples));
  const samples = local.filter((_, i) => i % step === 0);

  const inv = fixedMatrix.clone().invert();
  const fixedRot = new THREE.Matrix3().setFromMatrix4(fixedMatrix);
  const fn = new THREE.Vector3();
  const target = { point: new THREE.Vector3(), distance: 0, faceIndex: 0 };
  const q = new THREE.Vector3();
  let cur = T;
  let lastRmsd = Infinity;
  let pairs = 0;

  for (let it = 0; it < iterations; it++) {
    const P: Vec3[] = [];
    const Q: Vec3[] = [];
    for (const s of samples) {
      const w = apply(cur, s.p);
      q.set(w[0], w[1], w[2]).applyMatrix4(inv);
      if (!fixedBVH.closestPointToPoint(q, target, 0, opts.maxPairDist)) continue;
      // Nur echte Kontaktflächen: Oberflächen müssen gegeneinander zeigen
      fn.set(...bvhTriangleNormal(fixedBVH, target.faceIndex)).applyMatrix3(fixedRot).normalize();
      if (dot(rotate(cur.R, s.n), toV(fn)) > -0.9) continue;
      const c = target.point.clone().applyMatrix4(fixedMatrix);
      P.push(w);
      Q.push(toV(c));
    }
    pairs = P.length;
    if (pairs < 6) return null;
    const dT = solveRigid(P, Q);
    cur = compose(dT, cur);
    let s = 0;
    for (let i = 0; i < P.length; i++) s += dist(apply(dT, P[i]), Q[i]) ** 2;
    const r = Math.sqrt(s / P.length);
    if (Math.abs(lastRmsd - r) < 1e-6) {
      lastRmsd = r;
      break;
    }
    lastRmsd = r;
  }
  return { T: cur, rmsd: lastRmsd, pairs };
}
