/**
 * Zwei Teile über ausgewählte Flächen (und optional Punkte) zusammensetzen: Bedingungen aufstellen
 * und lösen, dann das, was die Auswahl offen lässt, über Flächenkontakt bestimmen (Einpassen),
 * Kontaktflächen anlegen und gleiche Umrisse genau übereinanderlegen.
 *
 * Alles im Koordinatensystem des festen Teils; das Ergebnis ist die Lage des beweglichen Teils darin.
 */
import { type Rigid, type Vec3, IDENTITY, apply, compose } from './align';
import { type EdgeFeature, type Feature, type PlaneFeature } from './features';
import { type Sample } from './icp';
import { type Marker, type NormalMode, solveJoint } from './match';
import { type MatePlan, describeFeature, planMate } from './mate';
import { type FreeMotion } from './motion';
import { solveConstraints } from './constraints';
import { type ScanTarget, autoSeat } from './scan';
import { settleContact } from './settle';
import { alignOutlines } from './outline';
import { placePlanePair, shapeSimilarity } from './suggest';

export interface FitInput {
  /** Flächen und Punkte des beweglichen (src) und des festen Teils (dst), lokal. */
  srcF: Feature[];
  dstF: Feature[];
  srcM: Marker[];
  dstM: Marker[];
  names: { src: string; dst: string };
  /** Gewählte Alternative (0 = beste). */
  alt: number;
  /** Freie Drehung/Verschiebung über Flächenkontakt bestimmen. */
  seat: boolean;
  ordered: boolean;
  normals: NormalMode;
  /** Teile so weit (mm) ineinanderschieben. */
  overlap: number;
  /** Typische Modellgröße (mm). */
  scale: number;
  /** Oberflächenpunkte des beweglichen Teils in der Nähe der Auswahl (lokal). */
  samples: Sample[];
  /** Das feste Teil (matrix = Identität: gerechnet wird in seinem Koordinatensystem). */
  target: ScanTarget;
}

export interface FitResult {
  error?: string;
  rel: Rigid;
  /** Freie Bewegungen im Koordinatensystem des festen Teils. */
  free: FreeMotion[];
  /** Anzahl Alternativen. */
  count: number;
  distError: number;
  angleError: number;
  info: string;
  flatRotation: boolean;
  notes: string[];
  summary: string;
  perm?: number[];
  /**
   * Weitere Drehungen um die freie Achse, die genauso gut passen (z. B. 180° bei einem rechteckigen
   * Zapfen): Achse durch point, Winkel relativ zur gewählten Lage (Bogenmaß), alles im festen Teil.
   */
  turn?: { point: Vec3; axis: Vec3; angles: number[] };
}

/**
 * Sind die beiden ebenen Flächen etwa gleich groß (typisch: Schnittflächen, Zapfen-Stirn auf
 * Aussparungsgrund)? Dann gehören sie mittig aufeinander – ein Verschieben „bis zum Anschlag"
 * wäre hier falsch, gerade bei Modellen, deren Teile sich absichtlich etwas überlappen.
 */
export function similarPlanes(a: PlaneFeature, b: PlaneFeature): boolean {
  const ratio = (x: number, y: number) => Math.min(x, y) / Math.max(x, y);
  return ratio(a.area, b.area) >= 0.6 && ratio(Math.max(...a.axes.size), Math.max(...b.axes.size)) >= 0.6;
}

/** Bezugspunkt einer Fläche (Mitte), lokal. */
function centerOf(f: Feature): Vec3 {
  if (f.kind === 'cylinder' || f.kind === 'edge') return f.center;
  if (f.kind === 'plane' && f.circle) return f.circle.center;
  return f.centroid;
}

const fmt = (x: number) => (x < 0.001 ? '< 0.001' : x.toFixed(x < 1 ? 3 : 2));

type Group = 'flat' | 'round' | 'edge';
const groupOf = (f: Feature): Group => (f.kind === 'cylinder' ? 'round' : f.kind === 'edge' ? 'edge' : 'flat');

function permutations(n: number): number[][] {
  if (n <= 1) return [Array.from({ length: n }, (_, i) => i)];
  const out: number[][] = [];
  for (const rest of permutations(n - 1))
    for (let k = 0; k <= rest.length; k++) out.push([...rest.slice(0, k), n - 1, ...rest.slice(k)]);
  return out;
}

/**
 * Welche Fläche gehört zu welcher? Gepaart wird je Art (eben, rund, Kante) in Auswahlreihenfolge –
 * wurden die Flächen auf beiden Teilen in unterschiedlicher Reihenfolge angeklickt, passt das nicht.
 * Deshalb werden alle Zuordnungen durchprobiert: Bei der richtigen liegen die Flächenmitten nach dem
 * Lösen aufeinander und die Winkel zwischen den Flächen stimmen überein.
 * @returns die bewegliche Auswahl in der besten Reihenfolge und ob sie von der Auswahlreihenfolge abweicht
 */
export function bestPairing(srcF: Feature[], dstF: Feature[], scale: number): { srcF: Feature[]; changed: boolean } {
  const slots: Record<Group, number[]> = { flat: [], round: [], edge: [] };
  srcF.forEach((f, i) => slots[groupOf(f)].push(i));
  const groups = (Object.keys(slots) as Group[]).filter((g) => slots[g].length > 1);
  if (!groups.length || groups.some((g) => slots[g].length > 5)) return { srcF, changed: false };
  // Alle Kombinationen der Reihenfolgen je Gruppe (höchstens 5! = 120 je Gruppe)
  let combos: Feature[][] = [srcF];
  for (const g of groups) {
    const next: Feature[][] = [];
    for (const base of combos)
      for (const perm of permutations(slots[g].length)) {
        const arr = base.slice();
        slots[g].forEach((slot, k) => { arr[slot] = base[slots[g][perm[k]]]; });
        next.push(arr);
      }
    combos = next;
    if (combos.length > 720) return { srcF, changed: false };
  }
  const score = (src: Feature[]) => {
    const plan = planMate(src, dstF);
    if (plan.error) return Infinity;
    const sol = solveConstraints(plan.constraints, { scale })[0];
    if (!sol) return Infinity;
    let sq = 0;
    for (const p of plan.pairs) {
      const a = apply(sol.T, centerOf(src[p.src]));
      const b = centerOf(dstF[p.dst]);
      sq += (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2;
    }
    return Math.sqrt(sq / Math.max(1, plan.pairs.length)) / scale + sol.angleError / 90 + sol.distError / scale;
  };
  const asSelected = score(srcF);
  let best = srcF;
  let bestScore = asSelected;
  for (const c of combos) {
    if (c.every((f, i) => f === srcF[i])) continue;
    const v = score(c);
    if (v < bestScore) { best = c; bestScore = v; }
  }
  // Nur abweichen, wenn die andere Zuordnung deutlich besser passt
  if (best !== srcF && bestScore < asSelected * 0.7 - 0.002) return { srcF: best, changed: true };
  return { srcF, changed: false };
}

/** Etwa gleich große Flächen (eben oder Freiform) – gehören mittig aufeinander. */
function similarFlat(a: Feature, b: Feature): boolean {
  if (a.kind === 'plane' && b.kind === 'plane') return similarPlanes(a, b);
  if ((a.kind === 'plane' || a.kind === 'surface') && (b.kind === 'plane' || b.kind === 'surface'))
    return Math.min(a.area, b.area) / Math.max(a.area, b.area) >= 0.5;
  return false;
}

export function fitFeatures(input: FitInput): FitResult {
  const { dstF, srcM, dstM, scale } = input;
  const fail = (error: string): FitResult => ({
    error, rel: IDENTITY, free: [], count: 0, distError: 0, angleError: 0, info: '', flatRotation: false, notes: [], summary: '',
  });
  // Zuordnung der Flächen unabhängig von der Klickreihenfolge
  const pairing = bestPairing(input.srcF, dstF, scale);
  const srcF = pairing.srcF;
  const plan: MatePlan = planMate(srcF, dstF, input.names);
  if (plan.error) return fail(plan.error);
  let perm: number[] | undefined;
  if (srcM.length || dstM.length) {
    if (srcM.length !== dstM.length) return fail(`Ungleiche Anzahl Punkte (${dstM.length} ↔ ${srcM.length}).`);
    // Welche Punkte zusammengehören, bestimmt wie bisher die automatische Zuordnung
    const pr = solveJoint({ src: srcM, dst: dstM, ordered: input.ordered, normals: input.normals });
    perm = pr.solutions[0]?.perm;
    if (perm) srcM.forEach((m, i) => plan.constraints.push({ kind: 'point', src: m.p, dst: dstM[perm![i]].p }));
  }
  // Genau ein Paar ebener Flächen
  const only = plan.constraints.length === 1 ? plan.pairs[0] : undefined;
  const sP = only?.kind === 'plane' && srcF[only.src].kind === 'plane' ? (srcF[only.src] as PlaneFeature) : undefined;
  const dP = only?.kind === 'plane' && dstF[only.dst].kind === 'plane' ? (dstF[only.dst] as PlaneFeature) : undefined;
  // Etwa gleich große Flächen mittig aufeinander: offen bleibt nur die Drehung um die Normale
  const centered = !!sP && !!dP && similarPlanes(sP, dP);
  if (centered) plan.constraints.push({ kind: 'point', src: sP!.centroid, dst: dP!.centroid });
  // Genau ein Kantenpaar etwa gleicher Länge: Mitte auf Mitte, offen bleibt die Drehung um die Kante
  const sE = only?.kind === 'edge' && srcF[only.src].kind === 'edge' ? (srcF[only.src] as EdgeFeature) : undefined;
  const dE = only?.kind === 'edge' && dstF[only.dst].kind === 'edge' ? (dstF[only.dst] as EdgeFeature) : undefined;
  const edgeCentered = !!sE && !!dE && Math.min(sE.length, dE.length) / Math.max(sE.length, dE.length) >= 0.8;
  if (edgeCentered) plan.constraints.push({ kind: 'point', src: sE!.center, dst: dE!.center });

  const all = solveConstraints(plan.constraints, { scale });
  if (!all.length) return fail('Die Flächen lassen sich nicht zusammensetzen.');
  // Lösungen, die die Bedingungen grob verletzen (z. B. falsch herum gesteckter Zapfen), nicht als Alternative anbieten
  const tol = Math.max(0.1, scale * 0.01);
  const fitting = all.filter((x) => x.angleError < 1 && x.distError < tol);
  const sols = fitting.length ? fitting : all.slice(0, 1);
  const sol = sols[Math.min(input.alt, sols.length - 1)];
  let rel = sol.T;
  const info: string[] = [];
  let flatRotation = false;
  let ties: number[] = [];
  if (pairing.changed) info.push('Flächen automatisch einander zugeordnet');
  // Mehrere Flächenpaare, alle etwa gleich groß (z. B. Zapfen in Aussparungen): mittig lassen und nur
  // noch drehen, nicht „bis zum Anschlag" schieben
  const flatPairs = plan.pairs.filter((p) => p.kind === 'plane' || p.kind === 'surface');
  const allSimilar = !centered && flatPairs.length > 1 && flatPairs.length === plan.pairs.length && !srcM.length
    && flatPairs.every((p) => similarFlat(srcF[p.src], dstF[p.dst]));
  const seatFree = allSimilar ? sol.free.filter((m) => m.kind === 'rotate') : sol.free;
  // Zwei deckungsgleiche ebene Flächen (typisch: Schnittflächen) direkt über Hauptachsen und Umriss
  // aufeinanderlegen – exakt und unabhängig davon, wie verdreht das Teil anfangs liegt
  const congruent = sP && dP && input.alt === 0 && shapeSimilarity(dP, sP) >= 0.95 ? placePlanePair(dP, sP) : null;
  if (congruent && congruent.rms <= Math.max(0.05, Math.max(...dP!.axes.size) * 0.01)) {
    rel = congruent.T;
    info.push(`Umrisse deckungsgleich (Rest ${fmt(congruent.rms)} mm)`);
  } else if (input.seat && seatFree.length) {
    if (centered || allSimilar) info.push('Flächen mittig aufeinander');
    if (edgeCentered) info.push('Kanten mittig aufeinander');
    const seat = autoSeat(input.samples, rel, input.target, seatFree, { contactDist: scale * 0.012, maxDepth: scale * 0.06 });
    rel = seat.T;
    if (seat.info) info.push(seat.info);
    flatRotation = seat.flatRotation;
    ties = seat.ties;
    // Das Abtasten ist nur so fein wie seine Schrittweite – Kontaktflächen danach genau anlegen
    const settled = settleContact(input.samples, rel, input.target, seatFree, { maxPairDist: scale * 0.012 });
    if (settled) rel = settled.T;
    // Fläche auf Fläche mit gleichem Umriss (z. B. Schnittflächen): Umrisse exakt übereinanderlegen
    if (sP && dP) {
      const size = Math.max(...dP.axes.size);
      const aligned = alignOutlines(sP.outline, dP.outline, rel, dP.normal, Math.max(0.05, size * 0.02));
      if (aligned) {
        rel = aligned.T;
        info.push(`Umrisse deckungsgleich (Rest ${fmt(aligned.rms)} mm)`);
      }
    }
  } else if (centered || allSimilar) info.push('Flächen mittig aufeinander');
  else if (edgeCentered) info.push('Kanten mittig aufeinander');
  if (input.overlap && plan.contactNormal) {
    const n = plan.contactNormal;
    rel = { R: rel.R, t: [rel.t[0] - n[0] * input.overlap, rel.t[1] - n[1] * input.overlap, rel.t[2] - n[2] * input.overlap] };
  }
  const rot = sol.free.find((m) => m.kind === 'rotate');
  const first = plan.pairs[0] ? srcF[plan.pairs[0].src] : undefined;
  const turn = ties.length && rot && first ? { point: compose(rel, { R: IDENTITY.R, t: centerOf(first) }).t, axis: rot.axis, angles: ties } : undefined;
  const summary = plan.pairs.map((p) => `${describeFeature(dstF[p.dst])} ↔ ${describeFeature(srcF[p.src])}`).join(' · ');
  // Von Hand bleibt bei mittig aufgesetzten Flächen auch das Verschieben in der Ebene möglich
  const free: FreeMotion[] = centered
    ? [{ kind: 'rotate', point: dP!.centroid, axis: dP!.normal }, { kind: 'translate', axis: dP!.axes.u }, { kind: 'translate', axis: dP!.axes.v }]
    : edgeCentered
      ? [{ kind: 'rotate', point: dE!.center, axis: dE!.dir }, { kind: 'translate', axis: dE!.dir }]
      : sol.free;
  return {
    rel, free, count: sols.length, distError: sol.distError, angleError: sol.angleError,
    info: info.join(' · '), flatRotation, notes: plan.notes, summary, perm, turn,
  };
}
