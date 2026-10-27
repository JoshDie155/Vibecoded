/**
 * Umrisse für Beschriftungen: Text in einer eingebauten Schrift (Droid Sans fett oder normal, Apache-2.0)
 * oder ein Logo aus einer SVG-Datei – als Polygone in mm, Mitte bei (0, 0), y nach oben. Außenränder
 * laufen gegen den Uhrzeigersinn, Löcher mit ihm (Füllregel „positiv“ in manifold).
 */
import * as THREE from 'three';
import { Font } from 'three/examples/jsm/loaders/FontLoader.js';
import type { Vec2 } from 'manifold-3d';
import boldData from './fonts/sans-bold.json';
import regularData from './fonts/sans-regular.json';

export type FontName = 'bold' | 'regular';

export interface Outline {
  polys: Vec2[][];
  width: number;
  height: number;
}

const fonts: Partial<Record<FontName, Font>> = {};
const fontOf = (name: FontName) => (fonts[name] ??= new Font((name === 'bold' ? boldData : regularData) as never));

/** Zeichen, die die Schrift nicht hat, durch ähnliche ersetzen (⌀ → ø), sonst weglassen. */
const SUBSTITUTE: Record<string, string> = { '⌀': 'ø', '∅': 'ø', '−': '-', '‐': '-', '\t': ' ' };

export function printable(text: string, name: FontName = 'bold'): string {
  const glyphs = (fontOf(name).data as { glyphs: Record<string, unknown> }).glyphs;
  return [...text].map((c) => SUBSTITUTE[c] ?? c).filter((c) => c === '\n' || c in glyphs).join('');
}

function polygonsOf(shapes: THREE.Shape[], divisions: number, map: (p: THREE.Vector2) => Vec2): Vec2[][] {
  const out: Vec2[][] = [];
  const ring = (pts: THREE.Vector2[], outer: boolean) => {
    const list = pts.map(map);
    if (list.length > 1) {
      const [a, b] = [list[0], list[list.length - 1]];
      if (Math.abs(a[0] - b[0]) < 1e-9 && Math.abs(a[1] - b[1]) < 1e-9) list.pop();
    }
    if (list.length < 3) return;
    let area = 0;
    for (let i = 0; i < list.length; i++) {
      const [x0, y0] = list[i];
      const [x1, y1] = list[(i + 1) % list.length];
      area += x0 * y1 - x1 * y0;
    }
    if (Math.abs(area) < 1e-10) return;
    if ((area > 0) !== outer) list.reverse();
    out.push(list);
  };
  for (const s of shapes) {
    const { shape, holes } = s.extractPoints(divisions);
    ring(shape, true);
    for (const h of holes) ring(h, false);
  }
  return out;
}

function centered(polys: Vec2[][]): Outline {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const p of polys) for (const [x, y] of p) {
    x0 = Math.min(x0, x); y0 = Math.min(y0, y); x1 = Math.max(x1, x); y1 = Math.max(y1, y);
  }
  if (!polys.length) return { polys, width: 0, height: 0 };
  const cx = (x0 + x1) / 2;
  const cy = (y0 + y1) / 2;
  return { polys: polys.map((p) => p.map(([x, y]) => [x - cx, y - cy] as Vec2)), width: x1 - x0, height: y1 - y0 };
}

/** Verhältnis Großbuchstabenhöhe / Schriftgröße (em). */
const capRatio: Partial<Record<FontName, number>> = {};
function capOf(name: FontName): number {
  if (capRatio[name] === undefined) {
    let top = 0;
    for (const s of fontOf(name).generateShapes('H', 1)) for (const p of s.getPoints()) top = Math.max(top, p.y);
    capRatio[name] = top || 0.7;
  }
  return capRatio[name]!;
}

/**
 * Text als Umrisse. height: Höhe der Großbuchstaben in mm. Mehrere Zeilen (\n) werden mittig gesetzt.
 */
export function textOutline(text: string, height: number, name: FontName = 'bold'): Outline {
  const font = fontOf(name);
  const em = height / capOf(name);
  const lines = printable(text, name).split('\n');
  const lead = 1.3 * height;
  const divisions = Math.max(3, Math.min(12, Math.round(height)));
  const polys: Vec2[][] = [];
  lines.forEach((line, i) => {
    if (!line.trim()) return;
    const shapes = font.generateShapes(line, em);
    let x0 = Infinity, x1 = -Infinity;
    for (const s of shapes) for (const p of s.getPoints()) { x0 = Math.min(x0, p.x); x1 = Math.max(x1, p.x); }
    const dx = -(x0 + x1) / 2;
    const dy = -i * lead;
    polys.push(...polygonsOf(shapes, divisions, (p) => [p.x + dx, p.y + dy]));
  });
  return centered(polys);
}

/**
 * Logo aus einer SVG-Datei: alle gefüllten Flächen, auf die Höhe in mm gebracht. Nur Linien ohne Füllung
 * bleiben weg (strokes).
 */
export async function svgOutline(svg: string, height: number): Promise<Outline & { skipped: number }> {
  const { SVGLoader } = await import('three/examples/jsm/loaders/SVGLoader.js');
  const data = new SVGLoader().parse(svg);
  const shapes: THREE.Shape[] = [];
  let skipped = 0;
  for (const path of data.paths) {
    const style = (path.userData?.style ?? {}) as { fill?: string; fillOpacity?: number; visibility?: string };
    if (style.fill === 'none' || style.fill === 'transparent' || style.fillOpacity === 0 || style.visibility === 'hidden') {
      skipped++;
      continue;
    }
    shapes.push(...SVGLoader.createShapes(path));
  }
  // Größe bestimmen, dann auf die gewünschte Höhe skalieren (SVG: y nach unten)
  let y0 = Infinity, y1 = -Infinity;
  for (const s of shapes) for (const p of s.getPoints()) { y0 = Math.min(y0, p.y); y1 = Math.max(y1, p.y); }
  const k = y1 > y0 ? height / (y1 - y0) : 1;
  const out = centered(polygonsOf(shapes, 8, (p) => [p.x * k, -p.y * k]));
  return { ...out, skipped };
}
