/**
 * Prüfansicht: Wo berühren sich zwei Teile, wo bleibt ein Spalt, wo dringen sie ineinander ein?
 * Je Dreieck des einen Teils werden Schwerpunkt und (leicht nach innen gezogene) Ecken gegen die
 * Oberfläche des anderen Teils geprüft – über die BVH des anderen Teils, also O(n log m).
 */
import { Vector3 } from 'three';
import type { MeshBVH } from 'three-mesh-bvh';
import { type Rigid, type Vec3, apply, compose, cross, dot, len, normalize, rotate } from './align';
import { bvhTriangleNormal } from './bvhtri';

export interface CheckTarget {
  bvh: MeshBVH;
  /** Lage des anderen Teils (lokal → Welt). */
  matrix: Rigid;
  /** Umschließender Quader des anderen Teils in Weltkoordinaten. */
  box: { min: Vec3; max: Vec3 };
}

export interface CheckOptions {
  /** Abstand, bis zu dem Flächen als berührend gelten (und Eindringen als unbedeutend). */
  contactTol: number;
  /** Größter Spalt, der noch angezeigt wird. */
  gapTol: number;
  /**
   * Größere Dreiecke werden für die Prüfung unterteilt, damit die Einfärbung der echten Kontaktfläche
   * folgt und nicht ganzen (bei CAD-Exporten oft riesigen) Dreiecken.
   */
  maxEdge?: number;
}

export interface PairCheck {
  /** Eingefärbte Stücke des geprüften Teils (nicht indizierte Dreiecke, lokale Koordinaten). */
  penetrating: Float32Array;
  touching: Float32Array;
  gap: Float32Array;
  /** Spaltmaß und Fläche je Stück in `gap` (für den vorherrschenden Spalt). */
  gapValues: Float32Array;
  gapAreas: Float32Array;
  /** Größte gefundene Eindringtiefe bzw. größter Spalt an Gegenflächen (mm). */
  maxPen: number;
  maxGap: number;
  /** Kleinster Spalt an Gegenflächen (mm), Infinity wenn keiner. */
  minGap: number;
}

function invert(T: Rigid): Rigid {
  const R = T.R;
  const Rt: Rigid['R'] = [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
  const t = rotate(Rt, T.t);
  return { R: Rt, t: [-t[0], -t[1], -t[2]] };
}

/** Anteil, um den die Prüfpunkte an den Ecken zur Mitte gezogen werden (an Kanten ist das Vorzeichen sonst unsicher). */
const PULL = 0.15;
/** Höchstens so oft wird halbiert (an der längsten Kante); begrenzt die Stücke je Dreieck auf 2^14. */
const MAX_DEPTH = 14;

const mid = (a: Vec3, b: Vec3): Vec3 => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
const edge = (a: Vec3, b: Vec3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

/** Zwischenstand einer Prüfung (wird in Abschnitten gefüllt, damit die Oberfläche nicht einfriert). */
export interface CheckAccumulator {
  pen: number[];
  touch: number[];
  gap: number[];
  gapV: number[];
  gapA: number[];
  maxPen: number;
  maxGap: number;
  minGap: number;
}

export function newAccumulator(): CheckAccumulator {
  return { pen: [], touch: [], gap: [], gapV: [], gapA: [], maxPen: 0, maxGap: 0, minGap: Infinity };
}

export function finishCheck(acc: CheckAccumulator): PairCheck {
  return {
    penetrating: Float32Array.from(acc.pen),
    touching: Float32Array.from(acc.touch),
    gap: Float32Array.from(acc.gap),
    gapValues: Float32Array.from(acc.gapV),
    gapAreas: Float32Array.from(acc.gapA),
    maxPen: acc.maxPen,
    maxGap: acc.maxGap,
    minGap: acc.minGap,
  };
}

/**
 * Prüft die Dreiecke [from, to) eines Teils (positions lokal, matrix lokal → Welt) gegen ein anderes
 * Teil, jedes `stride`-te (Stichprobe bei sehr feinen Netzen).
 */
export function checkRange(
  positions: Float32Array, matrix: Rigid, other: CheckTarget, opts: CheckOptions,
  acc: CheckAccumulator, from: number, to: number, stride = 1,
): void {
  const toOther = compose(invert(other.matrix), matrix);
  const R = toOther.R;
  const reach = opts.gapTol;
  const maxEdge = opts.maxEdge ?? Infinity;
  const { min, max } = other.box;
  const q = new Vector3();
  const hit = { point: new Vector3(), distance: 0, faceIndex: 0 };

  /** Einstufung eines Stücks: kleine Stücke über den Schwerpunkt, große zusätzlich über die Ecken. */
  const classify = (a: Vec3, b: Vec3, c: Vec3, n: Vec3, small: boolean) => {
    const g: Vec3 = [(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3];
    let pen = 0;
    let touch = false;
    let gap = Infinity;
    for (const v of small ? [g] : [g, a, b, c]) {
      const p: Vec3 = v === g ? g : [v[0] + (g[0] - v[0]) * PULL, v[1] + (g[1] - v[1]) * PULL, v[2] + (g[2] - v[2]) * PULL];
      const w = apply(toOther, p);
      q.set(w[0], w[1], w[2]);
      if (!other.bvh.closestPointToPoint(q, hit, 0, reach) || hit.distance > reach) continue;
      const nf = bvhTriangleNormal(other.bvh, hit.faceIndex);
      const d = hit.distance;
      const sd = (w[0] - hit.point.x) * nf[0] + (w[1] - hit.point.y) * nf[1] + (w[2] - hit.point.z) * nf[2];
      if (sd < -opts.contactTol && -sd >= 0.9 * d) pen = Math.max(pen, -sd);
      else if (d <= opts.contactTol) touch = true;
      // Spalt nur, wenn die Gegenfläche direkt gegenüberliegt (nicht schräg an einer Kante vorbei)
      else if (sd >= 0.9 * d && dot(n, nf) < -0.5) gap = Math.min(gap, d);
    }
    const list = pen > 0 ? acc.pen : touch ? acc.touch : gap < Infinity ? acc.gap : null;
    if (!list) return;
    list.push(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]);
    if (pen > 0) acc.maxPen = Math.max(acc.maxPen, pen);
    else if (!touch) {
      acc.maxGap = Math.max(acc.maxGap, gap);
      acc.minGap = Math.min(acc.minGap, gap);
      acc.gapV.push(gap);
      acc.gapA.push(len(cross([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]])) / 2);
    }
  };

  /**
   * Große Dreiecke an der längsten Kante halbieren, bis alle Kanten kurz genug sind – so entstehen auch
   * bei langen, dünnen Dreiecken (typisch für Deckflächen aus CAD-Exporten) nur so viele Stücke wie nötig.
   */
  const visit = (a: Vec3, b: Vec3, c: Vec3, n: Vec3, depth: number) => {
    // Schneller Ausschluss: Stück weit außerhalb des anderen Teils
    const g: Vec3 = [(a[0] + b[0] + c[0]) / 3, (a[1] + b[1] + c[1]) / 3, (a[2] + b[2] + c[2]) / 3];
    const gw = apply(matrix, g);
    const ext = Math.max(edge(a, g), edge(b, g), edge(c, g)) + reach;
    if (gw[0] < min[0] - ext || gw[1] < min[1] - ext || gw[2] < min[2] - ext || gw[0] > max[0] + ext || gw[1] > max[1] + ext || gw[2] > max[2] + ext) return;
    const eab = edge(a, b);
    const ebc = edge(b, c);
    const eca = edge(c, a);
    const longest = Math.max(eab, ebc, eca);
    if (depth < MAX_DEPTH && longest > maxEdge) {
      // Umlaufsinn der Hälften bleibt erhalten
      if (longest === eab) {
        const m = mid(a, b);
        visit(a, m, c, n, depth + 1);
        visit(m, b, c, n, depth + 1);
      } else if (longest === ebc) {
        const m = mid(b, c);
        visit(a, b, m, n, depth + 1);
        visit(a, m, c, n, depth + 1);
      } else {
        const m = mid(c, a);
        visit(a, b, m, n, depth + 1);
        visit(m, b, c, n, depth + 1);
      }
      return;
    }
    classify(a, b, c, n, longest <= maxEdge);
  };

  for (let t = from; t < to; t += stride) {
    const o = t * 9;
    const a: Vec3 = [positions[o], positions[o + 1], positions[o + 2]];
    const b: Vec3 = [positions[o + 3], positions[o + 4], positions[o + 5]];
    const c: Vec3 = [positions[o + 6], positions[o + 7], positions[o + 8]];
    const nl = normalize(cross([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]));
    if (!nl[0] && !nl[1] && !nl[2]) continue;
    visit(a, b, c, rotate(R, nl), 0);
  }
}

/** Prüft alle Dreiecke eines Teils gegen ein anderes Teil (auf einmal). */
export function checkPair(positions: Float32Array, matrix: Rigid, other: CheckTarget, opts: CheckOptions): PairCheck {
  const acc = newAccumulator();
  checkRange(positions, matrix, other, opts, acc, 0, positions.length / 9);
  return finishCheck(acc);
}

/** Vorherrschender Spalt: flächengewichteter Median über alle Spalt-Stücke. */
export function typicalGap(results: PairCheck[]): number {
  const items: [number, number][] = [];
  for (const r of results) for (let i = 0; i < r.gapValues.length; i++) items.push([r.gapValues[i], r.gapAreas[i]]);
  if (!items.length) return NaN;
  items.sort((x, y) => x[0] - y[0]);
  const half = items.reduce((s, x) => s + x[1], 0) / 2;
  let acc = 0;
  for (const [v, a] of items) {
    acc += a;
    if (acc >= half) return v;
  }
  return items[items.length - 1][0];
}
