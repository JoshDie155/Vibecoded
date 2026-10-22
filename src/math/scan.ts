/**
 * Automatisches Einpassen nach dem Verbinden: Nach einer Paarung (z. B. Zapfen koaxial in einer
 * Bohrung, Fläche auf Fläche) bleiben oft Bewegungen frei – Drehen um die Achse, Schieben entlang
 * der Achse oder in der Ebene. Hier wird bewertet, wie gut das bewegliche Teil am festen anliegt
 * (Kontaktfläche ohne Durchdringung), und die freien Bewegungen werden danach abgetastet:
 *
 * - Verdrehsichere Teile (D-Profil, Nut, eckige Zapfen) finden so ihre richtige Drehung.
 * - Ein Zapfen wird bis zum Anschlag eingeschoben (größter Kontakt, ohne einzudringen).
 * - Drehungen mit einseitigem Anschlag (Deckel am Scharnier) drehen bis zur Auflage.
 * - Ist das Profil flach (runde Teile), bleibt die Lage, wie sie ist, und die Drehung wird als
 *   frei gemeldet. Gleich gute Drehungen (Rechteck: 180°, Quadrat: 90°) werden mitgeliefert.
 *
 * Kontakt und Durchdringung werden über den nächsten Punkt der Oberfläche des festen Teils
 * bestimmt (BVH, O(log n) je Punkt) – eine Bewertung kostet O(Punkte · log Dreiecke).
 */
import { type BufferAttribute, type BufferGeometry, type InterleavedBufferAttribute, Vector3 } from 'three';
import type { MeshBVH } from 'three-mesh-bvh';
import { type Mat3, type Rigid, type Vec3, apply, compose, dot, len, normalize, rotate, sub } from './align';
import type { Sample } from './icp';
import { type FreeMotion, applyMotion } from './motion';

export interface ScanTarget {
  /** BVH des festen Teils (lokale Koordinaten des festen Teils). */
  bvh: MeshBVH;
  /**
   * Nicht-indizierte Positionen des festen Teils. Die Flächennormalen werden aus der Geometrie des
   * BVH gelesen, weil sich dessen faceIndex auf deren (umsortierten) Index bezieht – so stimmen sie
   * auch bei verschweißter (indizierter) Geometrie.
   */
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
  /** Grobes Abtastprofil (Bewertung wie contactScore). */
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
/** Strengere Schwelle für den Anschlag (Anteil von contactDist). */
const STOP_EPS = 0.02;
/** Höchstzahl der Punkte beim groben bzw. feinen Abtasten (Anschlag und Endlage prüfen alle). */
const COARSE_SAMPLES = 300;
const FINE_SAMPLES = 800;
/** Punkte und Schritte für die Vorprüfung, ob ein Zapfen gedreht tiefer hineinginge. */
const PROBE_SAMPLES = 150;
const PROBE_STEPS = 36;
/** Höchstzahl grob nachgemessener Anschläge je Drehung (viele Zähne, viel Spiel). */
const MAX_STOPS = 16;
/** Höchstzahl der Punkte im feinen Raster einer Verschiebung (den Anschlag misst die Bisektion). */
const FINE_SLIDE_STEPS = 24;
const TWO_PI = Math.PI * 2;
const DEG = Math.PI / 180;

interface Eval extends ContactScore {
  /** Anzahl der Punkte im Inneren (ohne Gewichtung). */
  penCount: number;
  /** Größte Eindringtiefe, auch unterhalb der Toleranz. */
  maxPen: number;
  /** Kontaktanteil mit dem großzügigeren Abstand `wide` (grobes Abtasten von Verschiebungen). */
  contactWide: number;
  /** Je bewertetem Punkt 1 bei Kontakt, sonst 0 (nur auf Anfrage). */
  flags?: Uint8Array;
}

const _q = new Vector3();
const _hit = { point: new Vector3(), distance: 0, faceIndex: 0 };
const _tri = new Float64Array(9);

function invert(T: Rigid): Rigid {
  const R = T.R;
  const Rt: Mat3 = [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
  const t = rotate(Rt, T.t);
  return { R: Rt, t: [-t[0], -t[1], -t[2]] };
}

/** Ecken von Dreieck `face` nach _tri, so wie der BVH es sieht (faceIndex bezieht sich auf dessen Index). */
function readTriangle(
  attr: BufferAttribute | InterleavedBufferAttribute,
  flat: ArrayLike<number> | null,
  index: ArrayLike<number> | null,
  face: number,
) {
  for (let k = 0; k < 3; k++) {
    const v = index ? index[3 * face + k] : 3 * face + k;
    if (flat) {
      _tri[3 * k] = flat[3 * v];
      _tri[3 * k + 1] = flat[3 * v + 1];
      _tri[3 * k + 2] = flat[3 * v + 2];
    } else {
      _tri[3 * k] = attr.getX(v);
      _tri[3 * k + 1] = attr.getY(v);
      _tri[3 * k + 2] = attr.getZ(v);
    }
  }
}

/**
 * Bewertet jeden `stride`-ten Punkt. M bildet lokale Koordinaten des beweglichen Teils direkt auf
 * lokale Koordinaten des festen Teils ab, so müssen die Flächennormalen nicht transformiert werden.
 * `wide` (≥ cd) zählt zusätzlich Kontakt mit größerem Abstand; die Durchdringung misst immer mit cd.
 */
function evaluate(
  samples: Sample[],
  M: Rigid,
  target: ScanTarget,
  cd: number,
  maxDepth: number,
  stride = 1,
  wide = cd,
  withFlags = false,
): Eval {
  const R = M.R;
  const t = M.t;
  const geom: BufferGeometry = target.bvh.geometry;
  const attr = geom.getAttribute('position');
  // Schneller Weg über das Array, sonst (verschachtelt, normiert) über die Zugriffsfunktionen
  const flat = (attr as BufferAttribute).isBufferAttribute && attr.itemSize === 3 && !attr.normalized ? attr.array : null;
  const index = geom.index?.array ?? null;
  const eps = PEN_EPS * cd;
  const cw = Math.max(cd, wide);
  const reach = Math.max(maxDepth, cw);
  const flags = withFlags ? new Uint8Array(Math.ceil(samples.length / stride)) : undefined;
  let n = 0;
  let contact = 0;
  let contactWide = 0;
  let pen = 0;
  let penCount = 0;
  let maxPen = 0;
  for (let i = 0; i < samples.length; i += stride, n++) {
    const p = samples[i].p;
    const qx = R[0] * p[0] + R[1] * p[1] + R[2] * p[2] + t[0];
    const qy = R[3] * p[0] + R[4] * p[1] + R[5] * p[2] + t[1];
    const qz = R[6] * p[0] + R[7] * p[1] + R[8] * p[2] + t[2];
    _q.set(qx, qy, qz);
    const hit = target.bvh.closestPointToPoint(_q, _hit, 0, reach);
    // Der BVH kann auch Treffer jenseits des Suchradius liefern, die dann nicht der nächste Punkt sein müssen
    if (!hit || hit.distance > reach) continue;
    readTriangle(attr, flat, index, hit.faceIndex);
    const ux = _tri[3] - _tri[0], uy = _tri[4] - _tri[1], uz = _tri[5] - _tri[2];
    const vx = _tri[6] - _tri[0], vy = _tri[7] - _tri[1], vz = _tri[8] - _tri[2];
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
    if (sd < 0 && along && d <= maxDepth) {
      if (-sd > maxPen) maxPen = -sd;
      if (-sd > eps) {
        penCount++;
        pen += Math.min(1, -sd / cd);
        continue;
      }
    }
    if (d > cw) continue;
    // Kontakt nur bei gegeneinander zeigenden Flächen, die sich auch gegenüberliegen
    const m = samples[i].n;
    const mx = R[0] * m[0] + R[1] * m[1] + R[2] * m[2];
    const my = R[3] * m[0] + R[4] * m[1] + R[5] * m[2];
    const mz = R[6] * m[0] + R[7] * m[1] + R[8] * m[2];
    if (mx * nx + my * ny + mz * nz > OPPOSE) continue;
    const facing = Math.abs(sd) >= 0.7 * d;
    if (d > 0.25 * cw && !facing) continue;
    contactWide++;
    if (d > cd || (d > 0.25 * cd && !facing)) continue;
    contact++;
    if (flags) flags[n] = 1;
  }
  if (n === 0) return { contact: 0, contactWide: 0, penetration: 0, score: 0, penCount: 0, maxPen: 0, flags };
  return {
    contact: contact / n,
    contactWide: contactWide / n,
    penetration: pen / n,
    score: (contact - LAMBDA * pen) / n,
    penCount,
    maxPen,
    flags,
  };
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

/**
 * Hat Lage b deutlich mehr Kontakt als Lage a? Verglichen werden dieselben Punkte (Vorzeichentest
 * nach McNemar): Punkte mit Kontakt nur in a gegen Punkte mit Kontakt nur in b. Gleich viel Kontakt
 * an anderer Stelle (Plateau, symmetrisches Spiel) ist kein Unterschied, ein einseitiger Gewinn schon
 * bei wenigen Punkten – anders als beim Vergleich der Anteile, die beide im Rauschen schwanken.
 * @returns 1 wenn b mehr Kontakt hat, −1 wenn a, sonst 0
 */
function moreContact(a: Uint8Array, b: Uint8Array): number {
  let onlyA = 0;
  let onlyB = 0;
  for (let i = 0; i < a.length; i++) {
    if (a[i] > b[i]) onlyA++;
    else if (b[i] > a[i]) onlyB++;
  }
  const lim = 2 * Math.sqrt(onlyA + onlyB) + 1;
  return onlyB - onlyA > lim ? 1 : onlyA - onlyB > lim ? -1 : 0;
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
/**
 * Toleranz für „fast so gut wie das Beste“: ein Viertel des positiven Teils des Profils. Durchdringung
 * (stark negativ) zählt nicht mit, sonst sähen Lagen ganz ohne Kontakt fast so gut aus wie die beste.
 */
const nearTol = (n: number, top: number, low: number) => Math.max(1.5 / n, 0.25 * (top - (top > 0 ? Math.max(low, 0) : low)));
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
  /** Bewertung nach Bewegung um x mit jedem `stride`-ten Punkt. */
  at: (x: number, stride: number, wide?: number, flags?: boolean) => Eval;
}

interface Internal {
  result: ScanResult;
  /** Verschiebung endet an einem Anschlag (dahinter würde das Teil eindringen). */
  stop: boolean;
  /** Seite des Anschlags (−1 / 1), 0 ohne oder bei Anschlägen auf beiden Seiten. */
  stopDir: number;
  /** Der beste Bereich reicht bis an dieses Ende des Abtastbereichs (−1 / 1): dahinter kann es weitergehen. */
  open: number;
  /** Größter Kontaktanteil im groben Profil. */
  cMax: number;
}

/** Ergebnis der Auswahl innerhalb eines Profils. */
type Picked = Omit<Internal, 'result'> & Pick<ScanResult, 'best' | 'profile' | 'flat' | 'ties'>;

/**
 * Grenze zur Durchdringung zwischen `inside` (frei) und `outside` (dringt ein), mit allen Punkten:
 * möglichst genau bis zur Berührung, bei unsauberer Berührung (Facetten, Kanten) mit kleiner Toleranz.
 */
function touch(ctx: Ctx, inside: number, outside: number, iters: number): number {
  const lim = STOP_EPS * ctx.cd;
  const noise = Math.floor(0.002 * ctx.n);
  // Mit weniger Punkten frei, mit allen nicht: so weit zurück, bis es frei ist
  let e = ctx.at(inside, 1);
  for (let k = 0, step = inside - outside; k < 6 && e.penCount > noise; k++, step *= 2) {
    outside = inside;
    inside += step;
    e = ctx.at(inside, 1);
  }
  const strict = e.maxPen <= lim;
  for (let k = 0; k < iters; k++) {
    const mid = (inside + outside) / 2;
    const em = ctx.at(mid, 1);
    if (strict ? em.maxPen <= lim : em.penCount <= noise) inside = mid;
    else outside = mid;
  }
  return inside;
}

/** Freie Lage am Rand eines Bereichs ohne Durchdringung (Anschlag). */
interface Stop {
  /** Letzte freie und erste eindringende Lage. */
  x: number;
  out: number;
  score: number;
  /** Kontakt je Punkt (feine Punkte), für den Vergleich zweier Anschläge. */
  flags?: Uint8Array;
}

/** Mögliche Endlage einer Drehung nach dem Feinabtasten (am Anschlag mit dessen Daten). */
interface Seat {
  x: number;
  score: number;
  stop?: Stop;
}

function scanRotate(ctx: Ctx, range: [number, number], steps: number, refine: boolean): Picked {
  const span = range[1] - range[0];
  const periodic = span >= TWO_PI - 1e-9;
  // Mehr als eine Umdrehung bringt nichts Neues (und jede Lage doppelt): genau eine abtasten
  const h = (periodic ? TWO_PI : span) / steps;
  const n = periodic ? steps : steps + 1;
  const xs = Array.from({ length: n }, (_, i) => range[0] + i * h);
  const coarse = xs.map((x) => ctx.at(x, ctx.stride));
  const sc = coarse.map((e) => e.score);
  const profile = xs.map((amount, i) => ({ amount, score: sc[i] }));
  const sMin = minOf(sc);
  const cMax = maxOf(coarse.map((e) => e.contact));
  const base = { profile, stop: false, stopDir: 0, open: 0, cMax };
  if (maxOf(sc) - sMin <= flatTol(ctx.nc, cMax)) return { ...base, best: clamp(0, range[0], range[1]), flat: true, ties: [] };

  const noiseC = Math.floor(0.002 * ctx.nc);
  const noiseF = Math.floor(0.002 * ctx.nf);
  const ok = coarse.map((e) => e.penCount <= noiseC);
  /** Nachbar im Raster (periodisch ringsum), −1 jenseits des Bereichs. */
  const nb = (i: number, d: number) => (periodic ? (i + d + n) % n : i + d >= 0 && i + d < n ? i + d : -1);

  // Anschläge: Übergänge von frei zu eindringend grob per Bisektion nachmessen. Die beste Lage liegt oft
  // genau dort (Deckel am Scharnier) und fällt zwischen zwei Rasterpunkte, die dann beide schlecht aussehen
  const stops: (Stop | undefined)[][] = xs.map(() => [undefined, undefined]);
  const edges: [number, number][] = [];
  ok.forEach((o, i) => {
    if (!o) return;
    for (const d of [-1, 1]) {
      const j = nb(i, d);
      if (j >= 0 && !ok[j]) edges.push([i, d]);
    }
  });
  edges.sort((a, b) => sc[b[0]] - sc[a[0]]);
  for (const [i, d] of edges.slice(0, MAX_STOPS)) {
    let inside = xs[i];
    let out = xs[i] + d * h;
    let score = sc[i];
    for (let k = 0; k < 4; k++) {
      const mid = (inside + out) / 2;
      const e = ctx.at(mid, ctx.stride);
      if (e.penCount <= noiseC) {
        inside = mid;
        score = e.score;
      } else out = mid;
    }
    stops[i][d < 0 ? 0 : 1] = { x: inside, out, score };
  }
  const eff = sc.map((s, i) => Math.max(s, stops[i][0]?.score ?? -Infinity, stops[i][1]?.score ?? -Infinity));
  const sBest = maxOf(eff);
  const tieTol = nearTol(ctx.nc, sBest, sMin);

  /** Anschlag fein nachmessen (Bisektion mit den feinen Punkten), mit Kontakt je Punkt. */
  const fineStop = (inside: number, out: number, e = ctx.at(inside, ctx.fstride, ctx.cd, true)): Stop => {
    for (let k = 0; k < 5; k++) {
      const mid = (inside + out) / 2;
      const em = ctx.at(mid, ctx.fstride, ctx.cd, true);
      if (em.penCount <= noiseF) {
        inside = mid;
        e = em;
      } else out = mid;
    }
    return { x: inside, out, score: e.score, flags: e.flags };
  };

  /**
   * Anderes Ende des freien Bereichs, der am Rand eines Fensters (Rasterpunkt k bei x) weitergeht, in
   * Richtung d: der Anschlag dort und die beste grobe Bewertung unterwegs. undefined, wenn es keinen gibt
   * (Ende des Bereichs, ringsum frei) oder k grob nicht frei ist.
   */
  const farStop = (k: number, d: number, x: number): { stop: Stop; inner: number } | undefined => {
    if (!ok[k]) return undefined;
    let inner = -Infinity;
    for (let c = 0; c < n; c++) {
      inner = Math.max(inner, sc[k]);
      const j = nb(k, d);
      if (j < 0) return undefined;
      if (!ok[j]) {
        const s = stops[k][d < 0 ? 0 : 1];
        if (!s) return undefined;
        // In der fortlaufenden Zählung des Fensters (über ±π hinaus)
        const shift = x + c * d * h - xs[k];
        return { stop: fineStop(s.x + shift, s.out + shift), inner };
      }
      k = j;
    }
    return undefined;
  };

  /**
   * Feines Abtasten in [lo, hi] (Ränder sind die Rasterpunkte kl und kr) mit Schrittweite h/3.
   * Liegt die beste freie Lage an einem Anschlag, zählt der Anschlag (bis zur Berührung drehen).
   * Sind beide Enden des freien Bereichs Anschläge mit gleich viel Kontakt (Spiel), ist es die Mitte.
   * Sonst der Schwerpunkt der Bewertung nahe am Maximum – mittelt das Rauschen flacher Spitzen und Plateaus.
   */
  const refineWindow = (lo: number, hi: number, kl: number, kr: number): Seat => {
    const m = Math.min(36, Math.max(6, Math.ceil((hi - lo) / (h / 3))));
    const fx = Array.from({ length: m + 1 }, (_, i) => lo + ((hi - lo) * i) / m);
    const ev = fx.map((x) => ctx.at(x, ctx.fstride, ctx.cd, true));
    const s = ev.map((e) => e.score);
    const fMax = maxOf(s);
    const b = s.indexOf(fMax);
    const free = (e: Eval) => e.penCount <= noiseF;
    if (free(ev[b])) {
      let l = b;
      let r = b;
      while (l > 0 && free(ev[l - 1])) l--;
      while (r < m && free(ev[r + 1])) r++;
      // Anschläge: im Fenster fein, sonst am anderen Ende des freien Bereichs (grobes Raster)
      const left = l > 0 ? { stop: fineStop(fx[l], fx[l - 1], ev[l]), inner: -Infinity } : farStop(kl, -1, lo);
      const right = r < m ? { stop: fineStop(fx[r], fx[r + 1], ev[r]), inner: -Infinity } : farStop(kr, 1, hi);
      const tol = nearTol(ctx.nf, fMax, minOf(s));
      let seat = left?.stop ?? right?.stop;
      if (left && right) {
        const c = moreContact(left.stop.flags!, right.stop.flags!);
        const top = Math.max(left.stop.score, right.stop.score);
        // Spiel: beide Anschläge gleich gut und dazwischen nichts Besseres
        if (c === 0 && top >= fMax - tol && Math.max(left.inner, right.inner) <= top + tieTol) {
          return { x: (left.stop.x + right.stop.x) / 2, score: Math.max(top, fMax) };
        }
        seat = c > 0 ? right.stop : c < 0 ? left.stop : left.stop.score >= right.stop.score ? left.stop : right.stop;
      }
      if (seat && seat.score >= fMax - tol) return { x: seat.x, score: Math.max(seat.score, fMax), stop: seat };
    }
    // Zusammenhängender Bereich über fMax − tol, gewichtet mit der Überhöhung
    const floor = fMax - Math.max(1.5 / ctx.nf, 0.1 * (fMax - minOf(s)));
    let l = b;
    let r = b;
    while (l > 0 && s[l - 1] >= floor) l--;
    while (r < m && s[r + 1] >= floor) r++;
    let sw = 0;
    let sx = 0;
    for (let i = l; i <= r; i++) {
      sw += s[i] - floor;
      sx += (s[i] - floor) * fx[i];
    }
    return { x: sw > 0 ? sx / sw : fx[b], score: fMax };
  };

  // Bereiche nahe am Maximum: der beste ist das Ergebnis, die übrigen sind gleich gute Alternativen.
  // Hysterese: getrennte Spitzen erst bei deutlichem Einbruch dazwischen (sonst zerfällt eine Spitze im Rauschen)
  const seats: Seat[] = [];
  for (const [s0, l] of runs(eff.map((s) => s >= sBest - 2 * tieTol), periodic)) {
    const idx = Array.from({ length: l }, (_, k) => s0 + k).filter((k) => eff[k % n] >= sBest - tieTol);
    if (!idx.length) continue;
    const i0 = idx[0];
    const i1 = idx[idx.length - 1];
    if (!refine) {
      // Mitte der besten Rasterpunkte; ein besserer Anschlag daneben geht vor
      const seat: Seat = { x: range[0] + ((i0 + i1) / 2) * h, score: maxOf(idx.map((k) => sc[k % n])) };
      for (const k of idx) {
        for (const st of stops[k % n]) {
          if (st && st.score > seat.score) {
            seat.x = st.x + (k - (k % n)) * h;
            seat.score = st.score;
          }
        }
      }
      seats.push(seat);
      continue;
    }
    let lo = range[0] + (i0 - 1) * h;
    let hi = range[0] + (i1 + 1) * h;
    if (!periodic) {
      lo = Math.max(range[0], lo);
      hi = Math.min(range[1], hi);
    }
    const kl = periodic ? (i0 - 1 + n) % n : Math.max(0, i0 - 1);
    const kr = periodic ? (i1 + 1) % n : Math.min(n - 1, i1 + 1);
    seats.push(refineWindow(lo, hi, kl, kr));
  }
  if (!seats.length) return { ...base, best: clamp(0, range[0], range[1]), flat: false, ties: [] };

  // Dieselbe Lage aus benachbarten Fenstern (z. B. beide Ränder eines Spiels) nur einmal
  const gap = (a: number, b: number) => (periodic ? Math.abs(wrapFrom(a - b, -Math.PI)) : Math.abs(a - b));
  seats.sort((a, b) => b.score - a.score);
  const uniq = seats.filter((s, i) => seats.findIndex((o) => gap(o.x, s.x) < h / 2) === i);
  const top = uniq[0].score;
  const good = uniq.filter((p) => p.score >= top - nearTol(ctx.nf, top, sMin));
  // Unter gleich guten Lagen die nächste zur aktuellen nehmen, damit nichts unnötig umspringt
  const away = (x: number) => Math.abs(periodic ? wrapFrom(x, -Math.PI) : x);
  const chosen = good.reduce((a, b) => (away(b.x) < away(a.x) ? b : a));
  // Am Anschlag mit allen Punkten genau bis zur Berührung
  const x = chosen.stop ? touch(ctx, chosen.stop.x, chosen.stop.out, 6) : chosen.x;
  const wrap = (v: number) => (periodic ? wrapFrom(v, range[0]) : clamp(v, range[0], range[1]));
  const ties = good.filter((p) => p !== chosen).map((p) => wrap(p.x)).sort((a, b) => a - b);
  return { ...base, best: wrap(x), flat: false, ties };
}

function scanTranslate(ctx: Ctx, range: [number, number], steps: number, refine: boolean): Picked {
  const cd = ctx.cd;
  const span = range[1] - range[0];
  const h = span / steps;
  const xs = Array.from({ length: steps + 1 }, (_, i) => range[0] + i * h);
  // Grob mit großzügigerem Kontaktabstand, damit das schmale Kontaktfenster nicht übersprungen wird
  // (das Profil zeigt trotzdem die echte Bewertung)
  const coarse = xs.map((x) => ctx.at(x, ctx.stride, Math.max(cd, 1.2 * h)));
  const profile = xs.map((amount, i) => ({ amount, score: coarse[i].score }));
  const wide = coarse.map((e) => e.contactWide);
  const sw = coarse.map((e) => e.contactWide - LAMBDA * e.penetration);
  const cMax = maxOf(wide);
  const keep = clamp(0, range[0], range[1]);
  const base = { profile, ties: [] as number[], cMax, stop: false, stopDir: 0, open: 0 };
  if (maxOf(sw) - minOf(sw) <= flatTol(ctx.nc, cMax)) return { ...base, best: keep, flat: true };

  const pick = (x: number[], ev: Eval[], c: number[], n: number, rel: number) => {
    // Größter Kontakt ohne Durchdringung; bei mehreren gleich guten Lagen die nächste zur aktuellen
    const noise = Math.floor(0.002 * n);
    const ok = ev.map((e) => e.penCount <= noise);
    if (!ok.some(Boolean)) return null;
    const cBest = maxOf(c.filter((_, i) => ok[i]));
    const tol = Math.max(1.5 / n, rel * cBest);
    let k = -1;
    c.forEach((v, i) => {
      if (ok[i] && v >= cBest - tol && (k < 0 || Math.abs(x[i] - keep) < Math.abs(x[k] - keep))) k = i;
    });
    // Bereich mit Hysterese: zusammenhängend ab cBest − 3·tol, Ränder dort, wo cBest − tol erreicht wird
    let a = k;
    let b = k;
    while (a > 0 && ok[a - 1] && c[a - 1] >= cBest - 3 * tol) a--;
    while (b < c.length - 1 && ok[b + 1] && c[b + 1] >= cBest - 3 * tol) b++;
    // Schwerpunkt der Überhöhung über cBest − 3·tol: mittelt das Rauschen, bei symmetrischem Profil die Mitte
    let sumW = 0;
    let sumX = 0;
    for (let i = a; i <= b; i++) {
      const w = c[i] - (cBest - 3 * tol);
      sumW += w;
      sumX += w * x[i];
    }
    while (c[a] < cBest - tol) a++;
    while (c[b] < cBest - tol) b--;
    return { a, b, ok, noise, center: sumW > 0 ? sumX / sumW : (x[a] + x[b]) / 2 };
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

  const g = pick(xs, coarse, wide, ctx.nc, 0.05);
  if (!g) {
    // Überall Durchdringung: wenigstens die am wenigsten schlechte Lage
    const b = sw.indexOf(maxOf(sw));
    if (!refine) return { ...base, best: xs[b], flat: false };
    const lo = Math.max(range[0], xs[b] - h);
    const hi = Math.min(range[1], xs[b] + h);
    const xf = Array.from({ length: 9 }, (_, i) => lo + ((hi - lo) * i) / 8);
    const sf = xf.map((x) => ctx.at(x, ctx.fstride).score);
    return { ...base, best: xf[sf.indexOf(maxOf(sf))], flat: false };
  }
  // Reicht der beste Bereich bis an ein Ende, kann es dahinter weitergehen (zu kurzer Abtastbereich)
  const open = g.a === 0 && g.b < steps ? -1 : g.b === steps && g.a > 0 ? 1 : 0;
  if (!refine) {
    const lp = stopIndex(g.ok, g.a, -1, 1) >= 0;
    const rp = stopIndex(g.ok, g.b, 1, 1) >= 0;
    const best = lp && rp ? (xs[g.a] + xs[g.b]) / 2 : lp ? xs[g.a] : rp ? xs[g.b] : clamp(keep, xs[g.a], xs[g.b]);
    return { ...base, best, flat: false, stop: lp || rp, stopDir: lp === rp ? 0 : lp ? -1 : 1, open: lp || rp ? 0 : open };
  }

  // Fein mit mehr Punkten und dem echten Kontaktabstand
  // Zwei grobe Schritte Rand: der großzügigere grobe Kontaktabstand kann den Bereich verschieben
  const lo = Math.max(range[0], xs[g.a] - 2 * h);
  const hi = Math.min(range[1], xs[g.b] + 2 * h);
  const m = Math.min(FINE_SLIDE_STEPS, Math.max(8, Math.ceil((hi - lo) / (0.5 * cd))));
  const hf = (hi - lo) / m;
  const xf = Array.from({ length: m + 1 }, (_, i) => lo + hf * i);
  const ef = xf.map((x) => ctx.at(x, ctx.fstride));
  const f = pick(xf, ef, ef.map((e) => e.contact), ctx.nf, 0.02);
  if (!f) {
    const b = ef.reduce((bi, e, i) => (e.score > ef[bi].score ? i : bi), 0);
    return { ...base, best: xf[b], flat: false };
  }
  // Anschlag: Durchdringung kurz hinter dem besten Bereich (innerhalb von contactDist)
  const reach = Math.max(1, Math.ceil(cd / hf));
  const il = stopIndex(f.ok, f.a, -1, reach);
  const ir = stopIndex(f.ok, f.b, 1, reach);
  if (il >= 0 || ir >= 0) {
    // Grenze zur Durchdringung durch Bisektion, möglichst genau bis zur Berührung
    const xl = il >= 0 ? touch(ctx, xf[il + 1], xf[il], 8) : undefined;
    const xr = ir >= 0 ? touch(ctx, xf[ir - 1], xf[ir], 8) : undefined;
    const best = xl !== undefined && xr !== undefined ? (xl + xr) / 2 : (xl ?? xr)!;
    return { ...base, best, flat: false, stop: true, stopDir: xl !== undefined && xr !== undefined ? 0 : xl !== undefined ? -1 : 1 };
  }
  // Kein Anschlag: aktuelle Lage behalten, wenn sie nicht deutlich weniger Kontakt hat (Plateau, z. B.
  // Zapfen durch eine Bohrung), sonst zur Mitte des besten Bereichs (Teil mittig auf der Auflage)
  const e0 = ctx.at(keep, ctx.fstride, cd, true);
  const ec = ctx.at(f.center, ctx.fstride, cd, true);
  if (e0.penCount <= f.noise && moreContact(e0.flags!, ec.flags!) <= 0) return { ...base, best: keep, flat: false, open };
  return { ...base, best: f.center, flat: false, open };
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
    at: (x, s, w = cd, flags = false) =>
      evaluate(samples, compose(inv, applyMotion(T, motion, x)), target, cd, opts.maxDepth, s, w, flags),
  };
  const refine = opts.refine ?? true;
  const span = Math.max(0, range[1] - range[0]);
  if (n === 0 || span === 0) {
    const best = clamp(0, range[0], range[1]);
    return { result: { best, T: applyMotion(T, motion, best), profile: [], flat: true, ties: [] }, stop: false, stopDir: 0, open: 0, cMax: 0 };
  }
  let r: Picked;
  if (motion.kind === 'rotate') {
    const turn = Math.min(span, TWO_PI);
    const steps = Math.max(4, Math.round(opts.steps ?? Math.max(8, Math.ceil((72 * turn) / TWO_PI))));
    r = scanRotate(ctx, range, steps, refine);
  } else {
    const steps = Math.max(4, Math.round(opts.steps ?? Math.min(48, Math.max(24, Math.ceil(span / cd)))));
    r = scanTranslate(ctx, range, steps, refine);
  }
  const { best, profile, flat, ties, ...rest } = r;
  return { result: { best, T: applyMotion(T, motion, best), profile, flat, ties }, ...rest };
}

/**
 * Tastet eine freie Bewegung im Bereich `range` ab (grob in `steps` Schritten, Standard 72 für eine
 * volle Umdrehung, danach fein um das Beste). Ab einer vollen Umdrehung wird periodisch abgetastet
 * (genau eine Umdrehung, best in [range[0], range[0] + 2π)).
 * Drehungen: höchste Bewertung; am Anschlag bis zur Berührung, bei Spiel die Mitte zwischen den Anschlägen.
 * Verschiebungen: größter Kontakt ohne Durchdringung, am Anschlag genau bis zur Berührung;
 * ohne deutlichen Gewinn bleibt die aktuelle Lage (best = 0).
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
 * noch etwas ändert. Eine Verschiebung, deren bester Bereich am Ende des Abtastbereichs liegt, wird
 * von dort aus fortgesetzt. Setzt ein Zapfen beim Einschieben nur auf (verdreht über der Bohrung),
 * wird die Drehung ein Stück weiter innen bestimmt und erneut eingeschoben.
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
  const cd = opts.contactDist;
  // Ist auch das Verschieben in der Ebene frei, darf die Drehachse durch die Mitte des Teils gehen:
  // dann ändert die Drehung die Lage nicht nebenbei, und die Suche läuft nicht gegeneinander
  const planar = free.map((m) => m.kind === 'rotate' && spansPlane(m.axis, free));
  // Erst drehen, dann schieben: ein falsch gedrehter Zapfen ließe sich nicht einschieben. Bei Fläche auf
  // Fläche umgekehrt: erst mittig schieben, dann drehen (bei versetzter Auflage ist die Drehung unscharf)
  const rotFirst = !planar.some(Boolean);
  const order = free.map((_, i) => i).sort((a, b) => Number((free[a].kind === 'rotate') !== rotFirst) - Number((free[b].kind === 'rotate') !== rotFirst));
  const slideRange = (m: FreeMotion, Tc: Rigid): [number, number] => {
    const w = opts.translateRange ?? Math.max(extentAlong(samples, Tc, m.axis), 4 * cd);
    return [-w, w];
  };

  /** Verschieben; reicht der beste Bereich ans Ende des Abtastbereichs, von dort aus weiter (höchstens 3-mal). */
  const slide = (T0: Rigid, m: FreeMotion, minMove: number) => {
    let Tc = T0;
    let amount = 0;
    let res = scanInternal(samples, Tc, target, m, slideRange(m, Tc), scanOpts);
    for (let ext = 0; Math.abs(res.result.best) > minMove; ext++) {
      Tc = res.result.T;
      amount += res.result.best;
      if (ext >= 3 || !res.open || Math.sign(res.result.best) !== res.open) break;
      res = scanInternal(samples, Tc, target, m, slideRange(m, Tc), scanOpts);
    }
    return { T: Tc, amount, res };
  };

  /**
   * Setzt das Teil beim Schieben entlang einer Drehachse auf (z. B. verdrehter D-Zapfen auf der
   * Stirnseite der Bohrung), zeigt erst die Lage ein Stück weiter innen, wie es gedreht sein muss:
   * dort dringt es falsch gedreht ein, richtig gedreht nicht. Drehung dort bestimmen und erneut
   * einschieben; übernommen wird das nur bei deutlich mehr Kontakt ohne Durchdringung.
   */
  const deeper = (cur: Rigid, rot: FreeMotion, m: FreeMotion, dir: number) => {
    const inv = invert(target.matrix);
    const n = samples.length;
    const Tp = applyMotion(cur, m, dir * 0.5 * opts.maxDepth);
    // Vorprüfung grob mit wenigen Punkten: gibt es dort überhaupt eine deutlich bessere Drehung?
    const ps = Math.max(1, Math.ceil(n / PROBE_SAMPLES));
    const probe = (a: number) => evaluate(samples, compose(inv, applyMotion(Tp, rot, a)), target, cd, opts.maxDepth, ps).score;
    const s0 = probe(0);
    let sBest = -Infinity;
    for (let k = 1; k < PROBE_STEPS; k++) sBest = Math.max(sBest, probe((k * TWO_PI) / PROBE_STEPS));
    if (sBest <= s0 + Math.max(3 / Math.ceil(n / ps), 0.1 * Math.abs(s0))) return undefined;
    const rr = scanInternal(samples, Tp, target, rot, rotRange, scanOpts);
    if (rr.result.flat) return undefined;
    const s = slide(applyMotion(cur, rot, rr.result.best), m, 1e-3 * cd);
    const fs = Math.max(1, Math.ceil(n / FINE_SAMPLES));
    const before = evaluate(samples, compose(inv, cur), target, cd, opts.maxDepth, fs, cd, true);
    const after = evaluate(samples, compose(inv, s.T), target, cd, opts.maxDepth, fs, cd, true);
    if (after.penCount > Math.floor(0.002 * Math.ceil(n / fs)) || moreContact(before.flags!, after.flags!) <= 0) return undefined;
    return { rot: rr, slide: s };
  };

  /** Koordinatenweise Suche ab der Lage `start`. */
  const run = (start: Rigid) => {
    let cur = start;
    const total = free.map(() => 0);
    const last: (Internal | undefined)[] = free.map(() => undefined);
    /** Beim letzten Abtasten tatsächlich übernommener Betrag (für die relativen Alternativen). */
    const applied = free.map(() => 0);
    const dirty = free.map(() => true);
    /** Nach einer Änderung müssen die anderen Bewegungen erneut geprüft werden. */
    const touched = (i: number) => dirty.forEach((_, j) => j !== i && (dirty[j] = true));
    for (let r = 0; r < rounds && dirty.some(Boolean); r++) {
      for (const i of order) {
        if (!dirty[i]) continue;
        dirty[i] = false;
        // Beim erneuten Abtasten nur deutliche Änderungen übernehmen (sonst springt die Lage im Rauschen hin und her)
        const again = last[i] !== undefined;
        let m = free[i];
        if (m.kind === 'translate') {
          const s = slide(cur, m, (again ? 0.05 : 1e-3) * cd);
          last[i] = s.res;
          if (s.amount !== 0) {
            cur = s.T;
            total[i] += s.amount;
            touched(i);
          }
          // Anschlag auf einer Seite: ginge es mit einer anderen Drehung um dieselbe Achse weiter hinein?
          if (!s.res.stop || !s.res.stopDir) continue;
          for (const j of order) {
            const rm = free[j];
            if (rm.kind !== 'rotate' || Math.abs(dot(normalize(rm.axis), normalize(m.axis))) < 0.999) continue;
            const d = deeper(cur, rm, m, s.res.stopDir);
            if (!d) continue;
            cur = d.slide.T;
            total[j] += wrapAngle(d.rot.result.best);
            applied[j] = d.rot.result.best;
            last[j] = d.rot;
            total[i] += d.slide.amount;
            last[i] = d.slide.res;
            touched(i);
            break;
          }
          continue;
        }
        if (planar[i]) {
          // Liegt die Mitte nahe am vorgegebenen Drehpunkt (meist die Mitte der Fläche), bleibt dieser:
          // er ist genauer als der Schwerpunkt zufälliger Punkte (wichtig für gleich gute Drehungen)
          const c = centroid(samples, cur);
          const a = normalize(m.axis);
          const d = sub(c, m.point);
          const k = dot(d, a);
          if (len([d[0] - k * a[0], d[1] - k * a[1], d[2] - k * a[2]]) > 4 * cd) m = { kind: 'rotate', point: c, axis: m.axis };
        }
        const res = scanInternal(samples, cur, target, m, rotRange, scanOpts);
        last[i] = res;
        applied[i] = 0;
        const b = wrapAngle(res.result.best);
        if (Math.abs(b) <= (again ? 0.5 * DEG : 1e-4)) continue;
        cur = res.result.T;
        total[i] += b;
        applied[i] = res.result.best;
        touched(i);
      }
    }
    return { cur, total, last, applied };
  };

  const fs = Math.max(1, Math.ceil(samples.length / FINE_SAMPLES));
  const inv = invert(target.matrix);
  const finalScore = (Tc: Rigid) => evaluate(samples, compose(inv, Tc), target, cd, opts.maxDepth, fs).score;
  let best = run(T);
  // Zapfen, die noch ganz vor der Bohrung stehen, geben beim Drehen kein Signal: zusätzlich zuerst so weit
  // einführen, dass das Teil möglichst nah an den Wänden liegt, und das bessere Ergebnis nehmen
  const pre = preInsert(samples, T, target, free, opts, slideRange);
  if (pre) {
    const other = run(pre.T);
    if (finalScore(other.cur) > finalScore(best.cur) + 1e-9) {
      other.total[pre.index] += pre.amount;
      best = other;
    }
  }
  const { cur, total, last, applied } = best;

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
        info.push(freeRotationInfo(samples, cur, target, m.axis, opts));
        return;
      }
      const rel = res.result.ties.map((x) => wrapFrom(x - applied[i], 0)).filter((x) => x > 1e-3 && x < TWO_PI - 1e-3);
      if (!ties.length) ties = rel;
      const also = rel.length ? ` – passt auch um ${rel.map(deg).join('/')}° gedreht` : '';
      info.push(`Drehung automatisch bestimmt (${deg(wrapAngle(total[i]))}°)${also}`);
    } else if (Math.abs(total[i]) > 1e-3 * cd) {
      info.push(res.stop ? `bis zum Anschlag verschoben (${mm(Math.abs(total[i]))} mm)` : `Lage angepasst (${mm(Math.abs(total[i]))} mm)`);
    } else if (res.stop) {
      info.push('liegt am Anschlag an');
    }
  });
  return { T: cur, flatRotation, ties, info: [...new Set(info)].join(' · ') };
}

/**
 * Lage entlang einer Achse, in der möglichst viele Punkte nahe an der Oberfläche des festen Teils liegen
 * (Durchdringung egal) – bei einem Zapfen vor der Bohrung: eingeführt. Nur wenn Drehen und Schieben um bzw.
 * entlang derselben Achse frei sind und sich dabei deutlich mehr Nähe ergibt.
 */
function preInsert(
  samples: Sample[],
  T: Rigid,
  target: ScanTarget,
  free: FreeMotion[],
  opts: ContactOptions,
  slideRange: (m: FreeMotion, T: Rigid) => [number, number],
): { T: Rigid; index: number; amount: number } | undefined {
  const index = free.findIndex(
    (m) => m.kind === 'translate' && free.some((r) => r.kind === 'rotate' && Math.abs(dot(normalize(r.axis), normalize(m.axis))) > 0.999),
  );
  if (index < 0 || !samples.length) return undefined;
  const m = free[index];
  const inv = invert(target.matrix);
  const stride = Math.max(1, Math.ceil(samples.length / COARSE_SAMPLES));
  const near = (x: number) => {
    const M = compose(inv, applyMotion(T, m, x));
    let k = 0;
    let n = 0;
    for (let i = 0; i < samples.length; i += stride, n++) {
      const p = apply(M, samples[i].p);
      _q.set(p[0], p[1], p[2]);
      const hit = target.bvh.closestPointToPoint(_q, _hit, 0, opts.maxDepth);
      if (hit && hit.distance <= opts.maxDepth) k++;
    }
    return n ? k / n : 0;
  };
  const [lo, hi] = slideRange(m, T);
  // Weit genug, dass ein Zapfen auch aus einer Teillänge vor der Bohrung ganz eingeführt werden kann
  const w = 2 * Math.max(Math.abs(lo), Math.abs(hi));
  const steps = 48;
  const here = near(0);
  let bx = 0;
  let bv = here;
  for (let k = 0; k <= steps; k++) {
    const x = -w + (2 * w * k) / steps;
    const v = near(x);
    if (v > bv + 1e-9) { bv = v; bx = x; }
  }
  if (bv < here + Math.max(0.1, 0.2 * here) || Math.abs(bx) < opts.contactDist) return undefined;
  return { T: applyMotion(T, m, bx), index, amount: bx };
}

/**
 * Hinweis zu einer Drehung ohne Einfluss: Liegt ein nennenswerter Teil des Kontakts an Flächen quer
 * zur Achse (Zapfen in Bohrung), ist das Teil rund; sonst liegt es nur eben auf (die Auflage ist
 * größer als das Teil, z. B. ein Quadrat auf einer großen Platte).
 */
function freeRotationInfo(samples: Sample[], T: Rigid, target: ScanTarget, axis: Vec3, opts: ContactOptions): string {
  const e = evaluate(samples, compose(invert(target.matrix), T), target, opts.contactDist, opts.maxDepth, 1, opts.contactDist, true);
  const a = normalize(axis);
  let contact = 0;
  let radial = 0;
  e.flags!.forEach((f, k) => {
    if (!f) return;
    contact++;
    if (Math.abs(dot(rotate(T.R, samples[k].n), a)) < 0.5) radial++;
  });
  if (!contact) return 'Kein Kontakt gefunden – Drehung frei wählbar';
  return radial >= 0.2 * contact ? 'Teil ist rund – Drehung frei wählbar' : 'Drehung frei wählbar – die Auflage bleibt dabei gleich';
}
