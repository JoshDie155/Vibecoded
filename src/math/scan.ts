/**
 * Automatisches Einpassen nach dem Verbinden: Nach einer Paarung (z. B. Zapfen koaxial in einer
 * Bohrung, Fläche auf Fläche) bleiben oft Bewegungen frei – Drehen um die Achse, Schieben entlang
 * der Achse oder in der Ebene. Hier wird bewertet, wie gut das bewegliche Teil am festen anliegt
 * (Kontaktfläche ohne Durchdringung), und die freien Bewegungen werden danach abgetastet:
 *
 * - Verdrehsichere Teile (D-Profil, Nut, eckige Zapfen) finden so ihre richtige Drehung.
 * - Ein Zapfen wird bis zum Anschlag eingeschoben (größter Kontakt, ohne einzudringen).
 * - Ist das Profil flach (runde Teile), bleibt die Lage, wie sie ist, und die Drehung wird als
 *   frei gemeldet. Gleich gute Drehungen (Rechteck: 180°, Quadrat: 90°) werden mitgeliefert.
 *
 * Kontakt und Durchdringung werden über den nächsten Punkt der Oberfläche des festen Teils
 * bestimmt (BVH, O(log n) je Punkt) – eine Bewertung kostet O(Punkte · log Dreiecke).
 */
import { Vector3 } from 'three';
import type { MeshBVH } from 'three-mesh-bvh';
import { type Mat3, type Rigid, type Vec3, apply, compose, dot, len, normalize, rotate, sub } from './align';
import type { Sample } from './icp';
import { type FreeMotion, applyMotion } from './motion';

export interface ScanTarget {
  /** BVH des festen Teils (lokale Koordinaten des festen Teils). */
  bvh: MeshBVH;
  /** Nicht-indizierte Positionen des festen Teils (dieselben wie in der Geometrie des BVH). */
  positions: Float32Array;
  /** Transformation lokal → Welt des festen Teils. */
  matrix: Rigid;
}

export interface ContactOptions {
  /** Punkte innerhalb dieses Abstands mit entgegengesetzten Normalen gelten als Kontakt (etwa 1–2 % der Teilgröße). */
  contactDist: number;
  /** Suchradius für die Prüfung auf Durchdringung. */
  maxDepth: number;
}

export interface ContactScore {
  /** Anteil der Punkte (0…1) mit Kontakt. */
  contact: number;
  /** Anteil der Punkte im Inneren des festen Teils, nach Tiefe gewichtet (je Punkt höchstens 1). */
  penetration: number;
  /** contact − 3 · penetration */
  score: number;
}

export interface ScanResult {
  /** Beste Verschiebung (Radiant bei Drehung, mm bei Verschiebung). */
  best: number;
  /** T um `best` bewegt. */
  T: Rigid;
  /** Grobes Abtastprofil. */
  profile: { amount: number; score: number }[];
  /** Kein nennenswerter Unterschied im Profil (rundes Teil, nichts zu entscheiden). */
  flat: boolean;
  /** Andere Beträge, die (fast) genauso gut passen, z. B. 180° beim Rechteck. */
  ties: number[];
}

/** Gewicht der Durchdringung gegenüber dem Kontakt. */
const LAMBDA = 3;
/** Normalen gelten als entgegengesetzt unterhalb dieses Kosinus (≈ 135°). */
const OPPOSE = -0.7;
/** Durchdringung zählt erst ab diesem Anteil von contactDist (Facettierung, Rundungsfehler). */
const PEN_EPS = 0.1;
/** Strengere Schwelle für den Anschlag beim Verschieben (Anteil von contactDist). */
const STOP_EPS = 0.02;
/** Höchstzahl der Punkte beim groben bzw. feinen Abtasten (Anschlag und Endlage prüfen alle). */
const COARSE_SAMPLES = 300;
const FINE_SAMPLES = 800;
const TWO_PI = Math.PI * 2;
const DEG = Math.PI / 180;

interface Eval extends ContactScore {
  /** Anzahl der Punkte im Inneren (ohne Gewichtung). */
  penCount: number;
  /** Größte Eindringtiefe, auch unterhalb der Toleranz. */
  maxPen: number;
}

const _q = new Vector3();
const _hit = { point: new Vector3(), distance: 0, faceIndex: 0 };

function invert(T: Rigid): Rigid {
  const R = T.R;
  const Rt: Mat3 = [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
  const t = rotate(Rt, T.t);
  return { R: Rt, t: [-t[0], -t[1], -t[2]] };
}

/**
 * Bewertet jeden `stride`-ten Punkt. M bildet lokale Koordinaten des beweglichen Teils direkt auf
 * lokale Koordinaten des festen Teils ab, so müssen die Flächennormalen nicht transformiert werden.
 */
function evaluate(samples: Sample[], M: Rigid, target: ScanTarget, cd: number, maxDepth: number, stride = 1): Eval {
  const R = M.R;
  const t = M.t;
  const pos = target.positions;
  // Der BVH sortiert die Dreiecke über einen Index um: faceIndex bezieht sich auf diesen Index
  const index = target.bvh.geometry.index?.array ?? null;
  const eps = PEN_EPS * cd;
  let n = 0;
  let contact = 0;
  let pen = 0;
  let penCount = 0;
  let maxPen = 0;
  for (let i = 0; i < samples.length; i += stride) {
    n++;
    const p = samples[i].p;
    const qx = R[0] * p[0] + R[1] * p[1] + R[2] * p[2] + t[0];
    const qy = R[3] * p[0] + R[4] * p[1] + R[5] * p[2] + t[1];
    const qz = R[6] * p[0] + R[7] * p[1] + R[8] * p[2] + t[2];
    _q.set(qx, qy, qz);
    const hit = target.bvh.closestPointToPoint(_q, _hit, 0, maxDepth);
    // Der BVH kann auch Treffer jenseits von maxDepth liefern, die dann nicht der nächste Punkt sein müssen
    if (!hit || hit.distance > maxDepth) continue;
    const f3 = hit.faceIndex * 3;
    const ia = (index ? index[f3] : f3) * 3;
    const ib = (index ? index[f3 + 1] : f3 + 1) * 3;
    const ic = (index ? index[f3 + 2] : f3 + 2) * 3;
    const ux = pos[ib] - pos[ia], uy = pos[ib + 1] - pos[ia + 1], uz = pos[ib + 2] - pos[ia + 2];
    const vx = pos[ic] - pos[ia], vy = pos[ic + 1] - pos[ia + 1], vz = pos[ic + 2] - pos[ia + 2];
    let nx = uy * vz - uz * vy;
    let ny = uz * vx - ux * vz;
    let nz = ux * vy - uy * vx;
    const nl = Math.hypot(nx, ny, nz);
    if (nl < 1e-20) continue;
    nx /= nl;
    ny /= nl;
    nz /= nl;
    const d = hit.distance;
    // Vorzeichenbehafteter Abstand: negativ = im Inneren des festen Teils
    const sd = (qx - hit.point.x) * nx + (qy - hit.point.y) * ny + (qz - hit.point.z) * nz;
    // Nur wenn der Versatz überwiegend entlang der Normalen liegt (an spitzen Kanten ist das Vorzeichen sonst unsicher)
    const along = Math.abs(sd) >= 0.5 * d;
    if (sd < 0 && along) {
      if (-sd > maxPen) maxPen = -sd;
      if (-sd > eps) {
        penCount++;
        pen += Math.min(1, -sd / cd);
        continue;
      }
    }
    if (d > cd) continue;
    // Kontakt nur bei gegeneinander zeigenden Flächen, die sich auch gegenüberliegen
    const m = samples[i].n;
    const mx = R[0] * m[0] + R[1] * m[1] + R[2] * m[2];
    const my = R[3] * m[0] + R[4] * m[1] + R[5] * m[2];
    const mz = R[6] * m[0] + R[7] * m[1] + R[8] * m[2];
    if (mx * nx + my * ny + mz * nz > OPPOSE) continue;
    if (d > 0.25 * cd && Math.abs(sd) < 0.7 * d) continue;
    contact++;
  }
  if (n === 0) return { contact: 0, penetration: 0, score: 0, penCount: 0, maxPen: 0 };
  return { contact: contact / n, penetration: pen / n, score: (contact - LAMBDA * pen) / n, penCount, maxPen };
}

/**
 * Wie gut liegt das bewegliche Teil (Punkte `samples` in lokalen Koordinaten, Lage T lokal → Welt)
 * am festen Teil an? Kontakt: Punkt näher als contactDist an der Oberfläche, Normalen entgegengesetzt.
 * Durchdringung: Punkt liegt (bis zur Tiefe maxDepth) im Inneren des festen Teils.
 */
export function contactScore(samples: Sample[], T: Rigid, target: ScanTarget, opts: ContactOptions): ContactScore {
  const e = evaluate(samples, compose(invert(target.matrix), T), target, opts.contactDist, opts.maxDepth);
  return { contact: e.contact, penetration: e.penetration, score: e.score };
}

/** Zusammenhängende Bereiche [Anfang, Länge] mit mask = true, bei periodic auch über das Ende hinweg. */
function runs(mask: boolean[], periodic: boolean): [number, number][] {
  const n = mask.length;
  if (n === 0) return [];
  if (mask.every(Boolean)) return [[0, n]];
  // Periodisch an einer Lücke beginnen, damit kein Bereich zerschnitten wird
  const start = periodic ? mask.indexOf(false) : 0;
  const out: [number, number][] = [];
  let i = 0;
  while (i < n) {
    if (!mask[(start + i) % n]) {
      i++;
      continue;
    }
    let l = 0;
    while (i + l < n && mask[(start + i + l) % n]) l++;
    out.push([(start + i) % n, l]);
    i += l;
  }
  return out;
}

const clamp = (x: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, x));
const maxOf = (a: number[]) => a.reduce((m, v) => (v > m ? v : m), -Infinity);
const minOf = (a: number[]) => a.reduce((m, v) => (v < m ? v : m), Infinity);
/** Unterschiede unterhalb dieser Schwelle gelten als Rauschen. */
const flatTol = (n: number, cMax: number) => Math.max(2.5 / n, 0.05 * cMax);
/** Winkel auf [lo, lo + 2π). */
const wrapFrom = (a: number, lo: number) => lo + (((a - lo) % TWO_PI) + TWO_PI) % TWO_PI;

interface Ctx {
  /** Punktzahl gesamt, grob und fein, mit den zugehörigen Schrittweiten durch die Punktliste. */
  n: number;
  nc: number;
  stride: number;
  nf: number;
  fstride: number;
  cd: number;
  at: (x: number, stride: number, cd?: number) => Eval;
}

interface Internal {
  result: ScanResult;
  /** Verschiebung endet an einem Anschlag (dahinter würde das Teil eindringen). */
  stop: boolean;
  /** Größter Kontaktanteil im groben Profil. */
  cMax: number;
}

/** Ergebnis der Auswahl innerhalb eines Profils. */
type Picked = Omit<Internal, 'result'> & Pick<ScanResult, 'best' | 'profile' | 'flat' | 'ties'>;

/**
 * Feines Abtasten in [lo, hi] mit Schrittweite h/3. Liegt das Maximum ohne Durchdringung zwischen
 * Lagen mit Durchdringung (Spiel), ist das Ergebnis die Mitte dieses Bereichs (Ränder per Bisektion).
 * Sonst der Schwerpunkt der Bewertung nahe am Maximum – mittelt das Rauschen flacher Spitzen und
 * Plateaus. Als Bewertung zählt das Maximum.
 */
function fineMax(ctx: Ctx, lo: number, hi: number, h: number): { x: number; score: number } {
  const m = Math.min(36, Math.max(6, Math.ceil((hi - lo) / (h / 3))));
  const xs = Array.from({ length: m + 1 }, (_, i) => lo + ((hi - lo) * i) / m);
  const ev = xs.map((x) => ctx.at(x, ctx.fstride));
  const s = ev.map((e) => e.score);
  const fMax = maxOf(s);
  const b = s.indexOf(fMax);
  const noise = Math.floor(0.002 * ctx.nf);
  const noPen = (e: Eval) => e.penCount <= noise;
  let l = b;
  let r = b;
  if (noPen(ev[b])) {
    while (l > 0 && noPen(ev[l - 1])) l--;
    while (r < m && noPen(ev[r + 1])) r++;
  }
  if (noPen(ev[b]) && l > 0 && r < m) {
    const edge = (a: number, o: number) => {
      for (let k = 0; k < 5; k++) {
        const mid = (a + o) / 2;
        if (noPen(ctx.at(mid, ctx.fstride))) a = mid;
        else o = mid;
      }
      return a;
    };
    return { x: (edge(xs[l], xs[l - 1]) + edge(xs[r], xs[r + 1])) / 2, score: fMax };
  }
  // Zusammenhängender Bereich über fMax − tol, gewichtet mit der Überhöhung
  const floor = fMax - Math.max(1.5 / ctx.nf, 0.1 * (fMax - minOf(s)));
  l = b;
  r = b;
  while (l > 0 && s[l - 1] >= floor) l--;
  while (r < m && s[r + 1] >= floor) r++;
  let sw = 0;
  let sx = 0;
  for (let i = l; i <= r; i++) {
    sw += s[i] - floor;
    sx += (s[i] - floor) * xs[i];
  }
  return { x: sw > 0 ? sx / sw : xs[b], score: fMax };
}

function scanRotate(ctx: Ctx, range: [number, number], steps: number, refine: boolean): Picked {
  const span = range[1] - range[0];
  const periodic = span >= TWO_PI - 1e-9;
  const h = span / steps;
  const xs = Array.from({ length: periodic ? steps : steps + 1 }, (_, i) => range[0] + i * h);
  const coarse = xs.map((x) => ctx.at(x, ctx.stride));
  const sc = coarse.map((e) => e.score);
  const profile = xs.map((amount, i) => ({ amount, score: sc[i] }));
  const sMax = maxOf(sc);
  const R = sMax - minOf(sc);
  const cMax = maxOf(coarse.map((e) => e.contact));
  if (R <= flatTol(ctx.nc, cMax)) return { best: clamp(0, range[0], range[1]), profile, flat: true, ties: [], stop: false, cMax };

  // Bereiche nahe am Maximum: der beste ist das Ergebnis, die übrigen sind gleich gute Alternativen.
  // Großzügig, weil schon ein kleiner Versatz zum Drehpunkt gleichwertige Lagen etwas schlechter aussehen lässt.
  // Hysterese: getrennte Spitzen erst bei deutlichem Einbruch dazwischen (sonst zerfällt eine Spitze im Rauschen)
  const tieTol = Math.max(1.5 / ctx.nc, 0.25 * R);
  const nx = sc.length;
  const peaks: { x: number; score: number }[] = [];
  for (const [s0, l] of runs(sc.map((s) => s >= sMax - 2 * tieTol), periodic)) {
    const idx = Array.from({ length: l }, (_, k) => s0 + k).filter((k) => sc[k % nx] >= sMax - tieTol);
    if (!idx.length) continue;
    const i0 = idx[0];
    const i1 = idx[idx.length - 1];
    let lo = range[0] + (i0 - 1) * h;
    let hi = range[0] + (i1 + 1) * h;
    if (!periodic) {
      lo = Math.max(range[0], lo);
      hi = Math.min(range[1], hi);
    }
    if (refine) peaks.push(fineMax(ctx, lo, hi, h));
    else peaks.push({ x: range[0] + ((i0 + i1) / 2) * h, score: maxOf(idx.map((k) => sc[k % nx])) });
  }
  peaks.sort((a, b) => b.score - a.score);
  // Unter gleich guten Lagen die nächste zur aktuellen nehmen, damit nichts unnötig umspringt
  const tieTolF = Math.max(1.5 / ctx.nf, 0.25 * R);
  const good = peaks.filter((p) => p.score >= peaks[0].score - tieTolF);
  const away = (x: number) => Math.abs(periodic ? wrapFrom(x, -Math.PI) : x);
  const chosen = good.reduce((a, b) => (away(b.x) < away(a.x) ? b : a));
  const wrap = (x: number) => (periodic ? wrapFrom(x, range[0]) : x);
  const best = wrap(chosen.x);
  const ties = good.filter((p) => p !== chosen).map((p) => wrap(p.x)).sort((a, b) => a - b);
  return { best, profile, flat: false, ties, stop: false, cMax };
}

function scanTranslate(ctx: Ctx, range: [number, number], steps: number, refine: boolean): Picked {
  const cd = ctx.cd;
  const span = range[1] - range[0];
  const h = span / steps;
  const xs = Array.from({ length: steps + 1 }, (_, i) => range[0] + i * h);
  // Grob mit großzügigerem Kontaktabstand, damit das schmale Kontaktfenster nicht übersprungen wird
  const coarse = xs.map((x) => ctx.at(x, ctx.stride, Math.max(cd, 1.2 * h)));
  const sc = coarse.map((e) => e.score);
  const profile = xs.map((amount, i) => ({ amount, score: sc[i] }));
  const cMax = maxOf(coarse.map((e) => e.contact));
  const keep = clamp(0, range[0], range[1]);
  const base = { profile, ties: [] as number[], cMax };
  if (maxOf(sc) - minOf(sc) <= flatTol(ctx.nc, cMax)) return { ...base, best: keep, flat: true, stop: false };

  const pick = (x: number[], ev: Eval[], n: number, rel: number) => {
    // Größter Kontakt ohne Durchdringung; bei mehreren gleich guten Lagen die nächste zur aktuellen
    const noise = Math.floor(0.002 * n);
    const ok = ev.map((e) => e.penCount <= noise);
    if (!ok.some(Boolean)) return null;
    const cBest = maxOf(ev.filter((_, i) => ok[i]).map((e) => e.contact));
    const tol = Math.max(1.5 / n, rel * cBest);
    let c = -1;
    ev.forEach((e, i) => {
      if (ok[i] && e.contact >= cBest - tol && (c < 0 || Math.abs(x[i] - keep) < Math.abs(x[c] - keep))) c = i;
    });
    // Bereich mit Hysterese: zusammenhängend ab cBest − 3·tol, Ränder dort, wo cBest − tol erreicht wird
    let a = c;
    let b = c;
    while (a > 0 && ok[a - 1] && ev[a - 1].contact >= cBest - 3 * tol) a--;
    while (b < ev.length - 1 && ok[b + 1] && ev[b + 1].contact >= cBest - 3 * tol) b++;
    // Schwerpunkt der Überhöhung über cBest − 3·tol: mittelt das Rauschen, bei symmetrischem Profil die Mitte
    let sw = 0;
    let sx = 0;
    for (let i = a; i <= b; i++) {
      const w = ev[i].contact - (cBest - 3 * tol);
      sw += w;
      sx += w * x[i];
    }
    while (ev[a].contact < cBest - tol) a++;
    while (ev[b].contact < cBest - tol) b--;
    return { a, b, ok, noise, cBest, tol, center: sw > 0 ? sx / sw : (x[a] + x[b]) / 2 };
  };
  /** Erster Punkt mit Durchdringung höchstens `reach` Schritte hinter dem Rand `from` (sonst −1). */
  const stopIndex = (ok: boolean[], from: number, dir: number, reach: number) => {
    for (let k = 1; k <= reach; k++) {
      const i = from + dir * k;
      if (i < 0 || i >= ok.length) return -1;
      if (!ok[i]) return i;
    }
    return -1;
  };

  const g = pick(xs, coarse, ctx.nc, 0.05);
  if (!g) {
    // Überall Durchdringung: wenigstens die am wenigsten schlechte Lage
    const b = sc.indexOf(maxOf(sc));
    const best = refine ? fineMax(ctx, Math.max(range[0], xs[b] - h), Math.min(range[1], xs[b] + h), h).x : xs[b];
    return { ...base, best, flat: false, stop: false };
  }
  if (!refine) {
    const lp = stopIndex(g.ok, g.a, -1, 1) >= 0;
    const rp = stopIndex(g.ok, g.b, 1, 1) >= 0;
    const best = lp && rp ? (xs[g.a] + xs[g.b]) / 2 : lp ? xs[g.a] : rp ? xs[g.b] : clamp(keep, xs[g.a], xs[g.b]);
    return { ...base, best, flat: false, stop: lp || rp };
  }

  // Fein mit mehr Punkten und dem echten Kontaktabstand
  // Zwei grobe Schritte Rand: der großzügigere grobe Kontaktabstand kann den Bereich verschieben
  const lo = Math.max(range[0], xs[g.a] - 2 * h);
  const hi = Math.min(range[1], xs[g.b] + 2 * h);
  const m = Math.min(80, Math.max(8, Math.ceil((hi - lo) / (0.5 * cd))));
  const hf = (hi - lo) / m;
  const xf = Array.from({ length: m + 1 }, (_, i) => lo + hf * i);
  const ef = xf.map((x) => ctx.at(x, ctx.fstride));
  const f = pick(xf, ef, ctx.nf, 0.02);
  if (!f) {
    const b = ef.reduce((bi, e, i) => (e.score > ef[bi].score ? i : bi), 0);
    return { ...base, best: xf[b], flat: false, stop: false };
  }
  // Anschlag: Durchdringung kurz hinter dem besten Bereich (innerhalb von contactDist)
  const reach = Math.max(1, Math.ceil(cd / hf));
  const il = stopIndex(f.ok, f.a, -1, reach);
  const ir = stopIndex(f.ok, f.b, 1, reach);
  if (il >= 0 || ir >= 0) {
    // Grenze zur Durchdringung durch Bisektion, möglichst genau bis zur Berührung
    const stopAt = (inside: number, outside: number) => {
      const strict = ctx.at(inside, 1).maxPen <= STOP_EPS * cd;
      const noise = Math.floor(0.002 * ctx.n);
      const good = (x: number) => {
        const e = ctx.at(x, 1);
        return strict ? e.maxPen <= STOP_EPS * cd : e.penCount <= noise;
      };
      for (let k = 0; k < 9; k++) {
        const mid = (inside + outside) / 2;
        if (good(mid)) inside = mid;
        else outside = mid;
      }
      return inside;
    };
    const xl = il >= 0 ? stopAt(xf[il + 1], xf[il]) : undefined;
    const xr = ir >= 0 ? stopAt(xf[ir - 1], xf[ir]) : undefined;
    const best = xl !== undefined && xr !== undefined ? (xl + xr) / 2 : (xl ?? xr)!;
    return { ...base, best, flat: false, stop: true };
  }
  // Kein Anschlag: aktuelle Lage behalten, wenn der Gewinn im Rauschen der Punktzahl untergeht (Plateau,
  // z. B. Zapfen durch eine Bohrung), sonst zur Mitte des besten Bereichs (Teil mittig auf der Auflage)
  const e0 = ctx.at(keep, ctx.fstride);
  const keepTol = Math.max(f.tol, 1.5 * Math.sqrt(f.cBest / ctx.nf));
  if (e0.penCount <= f.noise && e0.contact >= f.cBest - keepTol) return { ...base, best: keep, flat: false, stop: false };
  return { ...base, best: f.center, flat: false, stop: false };
}

function scanInternal(
  samples: Sample[],
  T: Rigid,
  target: ScanTarget,
  motion: FreeMotion,
  range: [number, number],
  opts: ContactOptions & { steps?: number; refine?: boolean },
): Internal {
  const inv = invert(target.matrix);
  const n = samples.length;
  const stride = Math.max(1, Math.ceil(n / COARSE_SAMPLES));
  const fstride = Math.max(1, Math.ceil(n / FINE_SAMPLES));
  const cd = opts.contactDist;
  const ctx: Ctx = {
    n: Math.max(1, n),
    nc: Math.max(1, Math.ceil(n / stride)),
    stride,
    nf: Math.max(1, Math.ceil(n / fstride)),
    fstride,
    cd,
    at: (x, s, c = cd) => evaluate(samples, compose(inv, applyMotion(T, motion, x)), target, c, opts.maxDepth, s),
  };
  const refine = opts.refine ?? true;
  const span = Math.max(0, range[1] - range[0]);
  if (n === 0 || span === 0) {
    const best = clamp(0, range[0], range[1]);
    return { result: { best, T: applyMotion(T, motion, best), profile: [], flat: true, ties: [] }, stop: false, cMax: 0 };
  }
  let r: Picked;
  if (motion.kind === 'rotate') {
    const steps = Math.max(4, Math.round(opts.steps ?? Math.max(8, Math.ceil((72 * span) / TWO_PI))));
    r = scanRotate(ctx, range, steps, refine);
  } else {
    const steps = Math.max(4, Math.round(opts.steps ?? Math.min(48, Math.max(24, Math.ceil(span / cd)))));
    r = scanTranslate(ctx, range, steps, refine);
  }
  const { best, profile, flat, ties, stop, cMax } = r;
  return { result: { best, T: applyMotion(T, motion, best), profile, flat, ties }, stop, cMax };
}

/**
 * Tastet eine freie Bewegung im Bereich `range` ab (grob in `steps` Schritten, Standard 72 für eine
 * volle Umdrehung, danach fein um das Beste). Eine volle Umdrehung wird periodisch behandelt.
 * Drehungen: höchste Bewertung; bei Spiel die Mitte des gleich guten Bereichs.
 * Verschiebungen: größter Kontakt ohne Durchdringung, am Anschlag genau bis zur Berührung;
 * ohne Unterschied bleibt die aktuelle Lage (best = 0).
 */
export function scanMotion(
  samples: Sample[],
  T: Rigid,
  target: ScanTarget,
  motion: FreeMotion,
  range: [number, number],
  opts: ContactOptions & { steps?: number; refine?: boolean },
): ScanResult {
  return scanInternal(samples, T, target, motion, range, opts).result;
}

/** Ausdehnung der Punkte (in Weltkoordinaten) entlang einer Richtung. */
function extentAlong(samples: Sample[], T: Rigid, axis: Vec3): number {
  const u = normalize(axis);
  let lo = Infinity;
  let hi = -Infinity;
  for (const s of samples) {
    const d = dot(apply(T, s.p), u);
    lo = Math.min(lo, d);
    hi = Math.max(hi, d);
  }
  return hi > lo ? hi - lo : 0;
}

/** Schwerpunkt der Punkte in Weltkoordinaten. */
function centroid(samples: Sample[], T: Rigid): Vec3 {
  const c: Vec3 = [0, 0, 0];
  for (const s of samples) {
    const w = apply(T, s.p);
    c[0] += w[0];
    c[1] += w[1];
    c[2] += w[2];
  }
  const n = Math.max(1, samples.length);
  return [c[0] / n, c[1] / n, c[2] / n];
}

/** Lassen die Verschiebungen die ganze Ebene senkrecht zur Achse frei (z. B. Fläche auf Fläche)? */
function spansPlane(axis: Vec3, free: FreeMotion[]): boolean {
  const a = normalize(axis);
  // Verschieberichtungen in die Ebene projiziert; Richtungen entlang der Achse fallen weg
  const dirs = free
    .filter((m) => m.kind === 'translate')
    .map((m) => {
      const t = normalize(m.axis);
      const k = dot(t, a);
      return [t[0] - k * a[0], t[1] - k * a[1], t[2] - k * a[2]] as Vec3;
    })
    .filter((d) => dot(d, d) > 0.25)
    .map(normalize);
  return dirs.some((d, i) => dirs.some((e, j) => j > i && Math.abs(dot(d, e)) < 0.9));
}

/** Winkel auf (−π, π]. */
const wrapAngle = (a: number) => {
  const w = wrapFrom(a, -Math.PI);
  return w === -Math.PI ? Math.PI : w;
};
const deg = (a: number) => Math.round(a / DEG);
const mm = (x: number) => x.toFixed(x < 10 ? 2 : 1).replace('.', ',');

/**
 * Setzt das bewegliche Teil selbsttätig ein: Koordinatenweise Suche über die freien Bewegungen
 * (Drehungen über eine volle Umdrehung, Verschiebungen über ±translateRange, Standard: Ausdehnung
 * der Punkte entlang der Richtung), wiederholt in bis zu `rounds` Runden (Standard 2), solange sich
 * noch etwas ändert.
 * @returns neue Lage, ob eine Drehung frei geblieben ist (rundes Teil), gleich gute Drehungen
 *          relativ zur neuen Lage (Radiant in (0, 2π)) und eine kurze Beschreibung
 */
export function autoSeat(
  samples: Sample[],
  T: Rigid,
  target: ScanTarget,
  free: FreeMotion[],
  opts: ContactOptions & { rotateRange?: [number, number]; translateRange?: number; rounds?: number },
): { T: Rigid; flatRotation: boolean; ties: number[]; info: string } {
  const rounds = opts.rounds ?? 2;
  const rotRange = opts.rotateRange ?? [-Math.PI, Math.PI];
  const scanOpts = { contactDist: opts.contactDist, maxDepth: opts.maxDepth };
  let cur = T;
  const total = free.map(() => 0);
  const stopped = free.map(() => false);
  const last: (Internal | undefined)[] = free.map(() => undefined);
  /** Beim letzten Abtasten tatsächlich übernommener Betrag (für die relativen Alternativen). */
  const applied = free.map(() => 0);
  const dirty = free.map(() => true);
  // Ist auch das Verschieben in der Ebene frei, darf die Drehachse durch die Mitte des Teils gehen:
  // dann ändert die Drehung die Lage nicht nebenbei, und die Suche läuft nicht gegeneinander
  const planar = free.map((m) => m.kind === 'rotate' && spansPlane(m.axis, free));
  // Erst drehen, dann schieben: ein falsch gedrehter Zapfen ließe sich nicht einschieben. Bei Fläche auf
  // Fläche umgekehrt: erst mittig schieben, dann drehen (bei versetzter Auflage ist die Drehung unscharf)
  const rotFirst = !planar.some(Boolean);
  const order = free.map((_, i) => i).sort((a, b) => Number((free[a].kind === 'rotate') !== rotFirst) - Number((free[b].kind === 'rotate') !== rotFirst));

  for (let r = 0; r < rounds && dirty.some(Boolean); r++) {
    for (const i of order) {
      if (!dirty[i]) continue;
      let m = free[i];
      if (m.kind === 'rotate' && planar[i]) {
        // Liegt die Mitte nahe am vorgegebenen Drehpunkt (meist die Mitte der Fläche), bleibt dieser:
        // er ist genauer als der Schwerpunkt zufälliger Punkte (wichtig für gleich gute Drehungen)
        const c = centroid(samples, cur);
        const a = normalize(m.axis);
        const d = sub(c, m.point);
        const k = dot(d, a);
        if (len([d[0] - k * a[0], d[1] - k * a[1], d[2] - k * a[2]]) > 4 * opts.contactDist) m = { kind: 'rotate', point: c, axis: m.axis };
      }
      let range: [number, number];
      if (m.kind === 'rotate') range = rotRange;
      else {
        const w = opts.translateRange ?? Math.max(extentAlong(samples, cur, m.axis), 4 * opts.contactDist);
        range = [-w, w];
      }
      const res = scanInternal(samples, cur, target, m, range, scanOpts);
      // Beim erneuten Abtasten nur deutliche Änderungen übernehmen (sonst springt die Lage im Rauschen hin und her)
      const again = last[i] !== undefined;
      last[i] = res;
      dirty[i] = false;
      stopped[i] = res.stop;
      const b = m.kind === 'rotate' ? wrapAngle(res.result.best) : res.result.best;
      const minMove = m.kind === 'rotate' ? (again ? 0.5 * DEG : 1e-4) : (again ? 0.05 : 1e-3) * opts.contactDist;
      applied[i] = 0;
      if (Math.abs(b) <= minMove) continue;
      cur = res.result.T;
      total[i] += b;
      applied[i] = res.result.best;
      // Nach einer Änderung müssen die anderen Bewegungen erneut geprüft werden
      for (let j = 0; j < free.length; j++) if (j !== i) dirty[j] = true;
    }
  }

  let flatRotation = false;
  let ties: number[] = [];
  const info: string[] = [];
  order.forEach((i) => {
    const m = free[i];
    const res = last[i];
    if (!res) return;
    if (m.kind === 'rotate') {
      if (res.result.flat) {
        flatRotation = true;
        info.push(res.cMax > 0 ? 'Teil ist rund – Drehung frei wählbar' : 'Kein Kontakt gefunden – Drehung frei wählbar');
        return;
      }
      const rel = res.result.ties.map((x) => wrapFrom(x - applied[i], 0)).filter((x) => x > 1e-3 && x < TWO_PI - 1e-3);
      if (!ties.length) ties = rel;
      const also = rel.length ? ` – passt auch um ${rel.map(deg).join('/')}° gedreht` : '';
      info.push(`Drehung automatisch bestimmt (${deg(wrapAngle(total[i]))}°)${also}`);
    } else if (Math.abs(total[i]) > 1e-3 * opts.contactDist) {
      info.push(stopped[i] ? `bis zum Anschlag verschoben (${mm(Math.abs(total[i]))} mm)` : `Lage angepasst (${mm(Math.abs(total[i]))} mm)`);
    } else if (stopped[i]) {
      info.push('liegt am Anschlag an');
    }
  });
  return { T: cur, flatRotation, ties, info: [...new Set(info)].join(' · ') };
}
