/**
 * Macht aus ausgewählten Flächen beider Teile Paarungen ("Mates") und daraus Bedingungen
 * für den Löser: ebene Fläche auf ebene Fläche, Kreisfläche mittig auf Kreisfläche,
 * Zylinder koaxial in Zylinder (Zapfen in Bohrung), Kante auf Kante.
 */
import type { Vec3 } from './align';
import type { Constraint } from './constraints';
import type { Feature } from './features';

export type MateKind = 'plane' | 'circle' | 'axis' | 'surface' | 'edge';

export interface MatePair {
  kind: MateKind;
  /** Index in der Auswahl des beweglichen bzw. festen Teils. */
  src: number;
  dst: number;
}

export interface MatePlan {
  constraints: Constraint[];
  pairs: MatePair[];
  /** Hinweise für die Anzeige (Spiel, unterschiedliche Durchmesser …). */
  notes: string[];
  /** Normale der ersten ebenen Kontaktfläche am festen Teil (für „Einsinken"). */
  contactNormal?: Vec3;
  error?: string;
}

const isRound = (f: Feature) => f.kind === 'cylinder';
const isEdge = (f: Feature) => f.kind === 'edge';
const mm = (x: number) => x.toFixed(x < 10 ? 2 : 1).replace('.', ',');

/** Kurze, verständliche Beschreibung einer Fläche für die Seitenleiste. */
export function describeFeature(f: Feature): string {
  if (f.kind === 'cylinder') {
    const d = `⌀ ${mm(f.radius * 2)} mm`;
    const part = f.arc < Math.PI * 1.9 ? ', Teilzylinder' : '';
    return f.hole ? `Bohrung ${d}${part}` : `Zapfen ${d}${part}`;
  }
  if (f.kind === 'plane') {
    if (f.circle) return `Kreisfläche ⌀ ${mm(f.circle.radius * 2)} mm`;
    const [a, b] = f.axes.size;
    return `Ebene Fläche ${mm(Math.max(a, b))} × ${mm(Math.min(a, b))} mm`;
  }
  if (f.kind === 'edge') return `Kante ${mm(f.length)} mm`;
  return 'Freiformfläche';
}

/** Kürzel für die Beschriftung im 3D-Fenster: E = Ebene, K = Kreisfläche, Z = Zylinder, F = Freiform, L = Kante (Linie). */
export function featureTag(f: Feature): string {
  return f.kind === 'cylinder' ? 'Z' : f.kind === 'edge' ? 'L' : f.kind === 'plane' ? (f.circle ? 'K' : 'E') : 'F';
}

/**
 * @param src Flächen des beweglichen Teils (lokale Koordinaten)
 * @param dst Flächen des festen Teils (in dem Koordinatensystem, in dem gelöst wird)
 */
export function planMate(src: Feature[], dst: Feature[], names = { src: 'Teil 2', dst: 'Teil 1' }): MatePlan {
  const plan: MatePlan = { constraints: [], pairs: [], notes: [] };
  // Ebene und Freiform-Flächen bilden eine Gruppe, runde Flächen die andere – jeweils in Auswahlreihenfolge
  const flatS = src.map((f, i) => ({ f, i })).filter((x) => !isRound(x.f) && !isEdge(x.f));
  const flatD = dst.map((f, i) => ({ f, i })).filter((x) => !isRound(x.f) && !isEdge(x.f));
  const roundS = src.map((f, i) => ({ f, i })).filter((x) => isRound(x.f));
  const roundD = dst.map((f, i) => ({ f, i })).filter((x) => isRound(x.f));
  const edgeS = src.map((f, i) => ({ f, i })).filter((x) => isEdge(x.f));
  const edgeD = dst.map((f, i) => ({ f, i })).filter((x) => isEdge(x.f));

  if (flatS.length !== flatD.length || roundS.length !== roundD.length || edgeS.length !== edgeD.length) {
    const count = (n: number, r: number, e: number) =>
      [n ? `${n} flache` : '', r ? `${r} runde` : '', e ? `${e} Kante(n)` : ''].filter(Boolean).join(' und ') || 'keine';
    plan.error =
      `Die Auswahl passt nicht zusammen: „${names.dst}" hat ${count(flatD.length, roundD.length, edgeD.length)}, ` +
      `„${names.src}" ${count(flatS.length, roundS.length, edgeS.length)} Fläche(n). Wähle auf beiden Teilen Gleichartiges.`;
    return plan;
  }

  for (let k = 0; k < flatS.length; k++) {
    const s = flatS[k];
    const d = flatD[k];
    const sp = s.f.kind === 'plane' ? s.f : null;
    const dp = d.f.kind === 'plane' ? d.f : null;
    const sc = s.f as Extract<Feature, { kind: 'plane' | 'surface' }>;
    const dc = d.f as Extract<Feature, { kind: 'plane' | 'surface' }>;
    plan.constraints.push({ kind: 'plane', src: { p: sc.centroid, n: sc.normal }, dst: { p: dc.centroid, n: dc.normal } });
    plan.contactNormal ??= dc.normal;
    if (sp?.circle && dp?.circle) {
      // Kreisflächen: Mittelpunkte genau aufeinander, nur die Drehung bleibt offen
      plan.constraints.push({ kind: 'point', src: sp.circle.center, dst: dp.circle.center });
      plan.pairs.push({ kind: 'circle', src: s.i, dst: d.i });
      const diff = Math.abs(sp.circle.radius - dp.circle.radius) * 2;
      if (diff > Math.max(0.05, 0.02 * dp.circle.radius))
        plan.notes.push(`Kreisflächen unterschiedlich groß (⌀ ${mm(dp.circle.radius * 2)} ↔ ${mm(sp.circle.radius * 2)} mm)`);
    } else {
      plan.pairs.push({ kind: s.f.kind === 'surface' || d.f.kind === 'surface' ? 'surface' : 'plane', src: s.i, dst: d.i });
      if (s.f.kind === 'surface' || d.f.kind === 'surface') plan.notes.push('Freiformfläche – „Feinausrichtung" empfohlen');
    }
  }

  for (let k = 0; k < roundS.length; k++) {
    const s = roundS[k].f as Extract<Feature, { kind: 'cylinder' }>;
    const d = roundD[k].f as Extract<Feature, { kind: 'cylinder' }>;
    plan.constraints.push({ kind: 'axis', src: { p: s.center, d: s.axis }, dst: { p: d.center, d: d.axis } });
    plan.pairs.push({ kind: 'axis', src: roundS[k].i, dst: roundD[k].i });
    if (s.hole !== d.hole) {
      const hole = s.hole ? s : d;
      const peg = s.hole ? d : s;
      const gap = hole.radius - peg.radius;
      if (gap < -0.005) plan.notes.push(`Achtung: Zapfen ist ${mm(-gap * 2)} mm dicker als die Bohrung`);
      else plan.notes.push(`Spiel ${mm(Math.max(0, gap))} mm pro Seite`);
    } else if (Math.abs(s.radius - d.radius) > Math.max(0.05, 0.02 * d.radius)) {
      plan.notes.push(`Durchmesser unterschiedlich (⌀ ${mm(d.radius * 2)} ↔ ${mm(s.radius * 2)} mm)`);
    }
  }

  // Kante auf Kante: die Geraden fluchten (beide Richtungen sind mögliche Lösungen)
  for (let k = 0; k < edgeS.length; k++) {
    const s = edgeS[k].f as Extract<Feature, { kind: 'edge' }>;
    const d = edgeD[k].f as Extract<Feature, { kind: 'edge' }>;
    plan.constraints.push({ kind: 'axis', src: { p: s.center, d: s.dir }, dst: { p: d.center, d: d.dir } });
    plan.pairs.push({ kind: 'edge', src: edgeS[k].i, dst: edgeD[k].i });
    if (Math.abs(s.length - d.length) > Math.max(0.05, 0.02 * d.length))
      plan.notes.push(`Kanten unterschiedlich lang (${mm(d.length)} ↔ ${mm(s.length)} mm)`);
  }
  return plan;
}
