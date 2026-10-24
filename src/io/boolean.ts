/**
 * Teile miteinander verrechnen (boolesche Operationen mit manifold-3d): vereinigen, die übrigen aus
 * einem Teil ausschneiden oder nur die Überschneidung behalten. Alle Netze kommen in denselben
 * Koordinaten (die des Zielteils) herein; das Ergebnis behält, aus welchem Dreieck des Zielteils jedes
 * Dreieck stammt – für Bemalung und gewählte Flächen.
 */
import type { Manifold } from 'manifold-3d';
import { loadManifold, originalIdOf, solidOf, trackedPositions } from './union';

export type BoolOp = 'union' | 'subtract' | 'intersect';

export interface BoolInput {
  name: string;
  /** Nicht indizierte Dreiecke in den Koordinaten des Zielteils. */
  positions: Float32Array;
}

export interface BoolResult {
  positions: Float32Array;
  /** Je Dreieck: Dreieck des Zielteils, aus dem es stammt (−1 = von den anderen Teilen oder neu). */
  source: Int32Array;
}

export class BoolError extends Error {
  name = 'BoolError';
}

/** target ⊕ others: Vereinigung, Differenz (target − others) oder Schnittmenge. */
export async function booleanParts(target: BoolInput, others: BoolInput[], op: BoolOp, wasmUrl?: string): Promise<BoolResult> {
  const wasm = await loadManifold(wasmUrl);
  const live: { delete(): void }[] = [];
  const keep = <T extends { delete(): void }>(x: T): T => {
    live.push(x);
    return x;
  };
  try {
    const open: string[] = [];
    const solid = (p: BoolInput) => {
      const m = solidOf(wasm, p.positions);
      if (!m) open.push(p.name);
      else keep(m);
      return m;
    };
    const t = solid(target);
    const rest = others.map(solid);
    if (!t || rest.some((m) => !m)) {
      throw new BoolError(`Nicht geschlossen (nicht wasserdicht): ${open.map((n) => `„${n}“`).join(', ')} – erst „Netz reparieren“.`);
    }
    const tools = rest as Manifold[];
    const tool = tools.length === 1 ? tools[0] : keep(wasm.Manifold.union(tools));
    const res = keep(op === 'union' ? t.add(tool) : op === 'subtract' ? t.subtract(tool) : t.intersect(tool));
    if (res.isEmpty()) throw new BoolError(op === 'intersect' ? 'Die Teile überschneiden sich nicht – es bliebe nichts übrig.' : 'Es bliebe nichts übrig.');
    if (op !== 'union' && Math.abs(res.volume() - t.volume()) < 1e-6) throw new BoolError('Die Teile berühren sich nicht – nichts zu ändern.');
    return trackedPositions(res, originalIdOf(t));
  } finally {
    for (const x of live) x.delete();
  }
}
