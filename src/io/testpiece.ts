/**
 * Toleranz-Testkörper: eine Platte mit Löchern in steigendem Spiel (erhaben beschriftet) und ein
 * einzelner Zapfen mit Griff. Beides drucken, den Zapfen der Reihe nach in die Löcher stecken – das
 * erste Loch, in das er ohne Gewalt passt, nennt das Spiel für Steckverbindungen auf diesem Drucker.
 *
 * Löcher und Zapfen stehen beim Druck senkrecht – wie die Zapfen beim waagerechten Zerschneiden.
 */
import type { Manifold } from 'manifold-3d';
import { loadManifold, positionsOfSolid } from './union';

export interface ToleranceOptions {
  /** Zapfen-⌀ (mm). */
  diameter: number;
  /** Spiel je Seite (mm), je Loch eines. */
  clearances: number[];
}

/** Siebensegment-Ziffern in einem 3 × 5-Feld: Segmente als Rechtecke [x0, y0, x1, y1]. */
function glyph(ch: string, w: number, h: number, s: number): [number, number, number, number][] {
  const seg: Record<string, [number, number, number, number]> = {
    a: [0, h - s, w, h], b: [w - s, h / 2, w, h], c: [w - s, 0, w, h / 2], d: [0, 0, w, s],
    e: [0, 0, s, h / 2], f: [0, h / 2, s, h], g: [0, h / 2 - s / 2, w, h / 2 + s / 2],
  };
  const map: Record<string, string> = {
    0: 'abcdef', 1: 'bc', 2: 'abged', 3: 'abgcd', 4: 'fgbc', 5: 'afgcd', 6: 'afgedc', 7: 'abc', 8: 'abcdefg', 9: 'abfgcd',
  };
  if (ch === '.' || ch === ',') return [[0, 0, s, s]];
  return [...(map[ch] ?? '')].map((k) => seg[k]);
}

/** Breite eines Zeichens. */
const advance = (ch: string, w: number, s: number) => (ch === '.' || ch === ',' ? s : w);

/** „0,15“ ohne überflüssige Nullen, mit Komma. */
export const clearanceLabel = (c: number) => String(+c.toFixed(3)).replace('.', ',');

export async function makeToleranceTest(opts: ToleranceOptions, wasmUrl?: string): Promise<{ plate: Float32Array; pin: Float32Array }> {
  const wasm = await loadManifold(wasmUrl);
  const { Manifold } = wasm;
  const live: { delete(): void }[] = [];
  const keep = <T extends { delete(): void }>(x: T): T => {
    live.push(x);
    return x;
  };
  try {
    const d = opts.diameter;
    const list = opts.clearances.length ? opts.clearances : [0.2];
    const cols = Math.min(list.length, 3);
    const rows = Math.ceil(list.length / cols);
    const pitch = d + 2 * Math.max(...list) + 8;
    const labelH = 5;
    const cell = { w: pitch, h: pitch + labelH + 2 };
    const thick = Math.max(4, Math.min(8, d * 0.6));
    const W = cols * cell.w + 4;
    const D = rows * cell.h + 4;
    let plate: Manifold = keep(Manifold.cube([W, D, thick], false));
    const holes: Manifold[] = [];
    const marks: Manifold[] = [];
    const s = 0.9;
    list.forEach((c, i) => {
      const col = i % cols;
      const row = Math.floor(i / cols);
      const cx = 2 + col * cell.w + cell.w / 2;
      const cy = D - 2 - row * cell.h - (pitch / 2);
      holes.push(keep(keep(Manifold.cylinder(thick + 2, d / 2 + c, d / 2 + c, 64, false)).translate([cx, cy, -1])));
      // Beschriftung darunter, erhaben 0,6 mm
      const text = clearanceLabel(c);
      const gw = 3, gap = 0.9;
      const width = [...text].reduce((n, ch) => n + advance(ch, gw, s) + gap, -gap);
      let x = cx - width / 2;
      const y = cy - pitch / 2 - labelH + 1;
      for (const ch of text) {
        for (const [x0, y0, x1, y1] of glyph(ch, gw, labelH, s)) {
          marks.push(keep(keep(Manifold.cube([x1 - x0, y1 - y0, 0.6], false)).translate([x + x0, y + y0, thick])));
        }
        x += advance(ch, gw, s) + gap;
      }
    });
    plate = keep(plate.subtract(keep(Manifold.union(holes))));
    if (marks.length) plate = keep(plate.add(keep(Manifold.union(marks))));
    // Zapfen: Griffscheibe unten, Zapfen nach oben (steht so auf der Platte)
    const grip = keep(Manifold.cylinder(2, d / 2 + 3, d / 2 + 3, 64, false));
    const pin = keep(keep(Manifold.cylinder(thick + 4, d / 2, d / 2, 64, false)).translate([0, 0, 2]));
    // Kleine Fase oben, damit der Zapfen leicht einfädelt
    const tip = keep(keep(Manifold.cylinder(0.6, d / 2, d / 2 - 0.6, 64, false)).translate([0, 0, thick + 6]));
    const pinBody = keep(Manifold.union([grip, pin, tip]));
    return { plate: positionsOfSolid(plate), pin: positionsOfSolid(pinBody) };
  } finally {
    for (const x of live) x.delete();
  }
}
