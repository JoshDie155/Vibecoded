/**
 * Bemalung aus Bambu Studio, OrcaSlicer und PrusaSlicer (TriangleSelector): Jedes Dreieck des
 * Netzes kann rekursiv in Teildreiecke zerlegt sein, jedes Blatt trägt einen Zustand – bei der
 * Farbbemalung (paint_color) die Filamentnummer, 0 = Filament des Teils; bei Stützen 1 = erzwingen,
 * 2 = verhindern.
 *
 * Gespeichert wird je Dreieck eine Hex-Zeichenkette, von hinten gelesen, je Zeichen 4 Bit:
 *   Blatt:   xx00 mit xx = Zustand (0–2); xx = 11 heißt: Zustand − 3 folgt in weiteren 4 Bit
 *            (je 1111 davor zählt 15 dazu)
 *   Zerlegt: ssnn mit nn = Zahl der geteilten Seiten (1–3), ss = besondere Seite; danach folgen
 *            die nn + 1 Kinder in umgekehrter Reihenfolge.
 */

export type PaintNode =
  | { state: number }
  | { split: 1 | 2 | 3; special: number; children: PaintNode[] };

type V3 = [number, number, number];

const HEX = '0123456789ABCDEF';

/** Zerlegt eine Zeichenkette; null bei ungültigem Inhalt. */
export function decodePaint(code: string): PaintNode | null {
  let i = code.length - 1;
  const next = () => {
    if (i < 0) throw new Error('zu kurz');
    const v = parseInt(code[i--], 16);
    if (Number.isNaN(v)) throw new Error('kein Hex-Zeichen');
    return v;
  };
  const node = (depth: number): PaintNode => {
    if (depth > 40) throw new Error('zu tief');
    const c = next();
    const split = c & 3;
    if (split === 0) {
      if ((c & 0b1100) !== 0b1100) return { state: c >> 2 };
      let n = next();
      let extra = 0;
      while (n === 15) { extra++; n = next(); }
      return { state: n + 15 * extra + 3 };
    }
    const children = new Array<PaintNode>(split + 1);
    for (let k = split; k >= 0; k--) children[k] = node(depth + 1);
    return { split: split as 1 | 2 | 3, special: c >> 2, children };
  };
  if (!code) return { state: 0 };
  try {
    return node(0);
  } catch {
    return null;
  }
}

/** Gegenstück zu decodePaint; unbemalte Dreiecke ergeben ''. */
export function encodePaint(root: PaintNode): string {
  if ('state' in root && root.state === 0) return '';
  const nib: number[] = [];
  const put = (n: PaintNode) => {
    if ('state' in n) {
      if (n.state < 3) { nib.push(n.state << 2); return; }
      nib.push(0b1100);
      let r = n.state - 3;
      while (r >= 15) { nib.push(15); r -= 15; }
      nib.push(r);
      return;
    }
    nib.push((n.special << 2) | n.split);
    for (let k = n.split; k >= 0; k--) put(n.children[k]);
  };
  put(root);
  let out = '';
  for (let k = nib.length - 1; k >= 0; k--) out += HEX[nib[k]];
  return out;
}

/** Alle Zustände, die in der Zeichenkette vorkommen. */
export function paintStates(code: string, into = new Set<number>()): Set<number> {
  const visit = (n: PaintNode) => {
    if ('state' in n) into.add(n.state);
    else n.children.forEach(visit);
  };
  const root = decodePaint(code);
  if (root) visit(root);
  return into;
}

/** Zustände umnummerieren (z. B. Filamente nach dem Zusammenführen mehrerer Projekte). */
export function remapPaint(code: string, map: (state: number) => number): string {
  const root = decodePaint(code);
  if (!root) return '';
  const visit = (n: PaintNode): PaintNode =>
    'state' in n ? { state: map(n.state) } : { ...n, children: n.children.map(visit) };
  return encodePaint(visit(root));
}

const mid = (a: V3, b: V3): V3 => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];

/**
 * Teildreiecke mit ihrem Zustand, Eckpunkte in derselben Reihenfolge wie im Slicer
 * (TriangleSelector::perform_split): Ecken ab der besonderen Seite, neue Punkte auf den Kantenmitten.
 */
export function paintLeaves(root: PaintNode, a: V3, b: V3, c: V3, out: (state: number, a: V3, b: V3, c: V3) => void) {
  const visit = (n: PaintNode, v: V3[]) => {
    if ('state' in n) { out(n.state, v[0], v[1], v[2]); return; }
    const s = n.special % 3;
    const [A, B, C] = [v[s], v[(s + 1) % 3], v[(s + 2) % 3]];
    const ch = n.children;
    if (n.split === 1) {
      const m = mid(B, C);
      visit(ch[0], [A, B, m]);
      visit(ch[1], [m, C, A]);
    } else if (n.split === 2) {
      const m1 = mid(B, A);
      const m2 = mid(A, C);
      visit(ch[0], [A, m1, m2]);
      visit(ch[1], [m1, B, m2]);
      visit(ch[2], [B, C, m2]);
    } else {
      const m01 = mid(A, B);
      const m12 = mid(B, C);
      const m20 = mid(C, A);
      visit(ch[0], [A, m01, m20]);
      visit(ch[1], [m01, B, m12]);
      visit(ch[2], [m12, C, m20]);
      visit(ch[3], [m01, m12, m20]);
    }
  };
  visit(root, [a, b, c]);
}

/**
 * Bemalung auf ein neues Netz übertragen, dessen Dreiecke aus dem alten stammen (source[i] = Dreieck im
 * alten Netz oder −1 für neue Flächen, z. B. nach einer Bohrung oder einem Schnitt). Einfarbige Dreiecke
 * behalten ihre Farbe auch zerteilt; fein bemalte nur, wenn sie unverändert blieben – deren Ecken kommen
 * dafür in die alte Reihenfolge (die Kodierung bezieht sich auf sie), positions wird entsprechend
 * umgeschrieben. lost zählt Dreiecke, deren feine Bemalung nicht übertragbar war.
 */
export function carryPaint(
  oldPositions: Float32Array, paint: Record<string, string[]>, positions: Float32Array, source: Int32Array,
): { paint: Record<string, string[]>; positions: Float32Array; lost: number } {
  const n = source.length;
  const out = positions.slice();
  const layers = Object.entries(paint);
  const result: Record<string, string[]> = Object.fromEntries(layers.map(([k]) => [k, new Array<string>(n).fill('')]));
  const same = (i: number, a: number, j: number, b: number) => Math.abs(positions[9 * i + 3 * a] - oldPositions[9 * j + 3 * b]) < 1e-4
    && Math.abs(positions[9 * i + 3 * a + 1] - oldPositions[9 * j + 3 * b + 1]) < 1e-4
    && Math.abs(positions[9 * i + 3 * a + 2] - oldPositions[9 * j + 3 * b + 2]) < 1e-4;
  let lost = 0;
  for (let i = 0; i < n; i++) {
    const j = source[i];
    if (j < 0) continue;
    const fine = layers.some(([, codes]) => codes[j] && !('state' in (decodePaint(codes[j]) ?? { state: 0 })));
    let rot = -1;
    if (fine) {
      // Unverändert? Dann Ecke k des neuen = Ecke (k + r) des alten
      for (let r = 0; r < 3 && rot < 0; r++) if (same(i, 0, j, r) && same(i, 1, j, (1 + r) % 3) && same(i, 2, j, (2 + r) % 3)) rot = r;
      if (rot > 0) for (let k = 0; k < 3; k++) for (let c = 0; c < 3; c++) out[9 * i + 3 * ((k + rot) % 3) + c] = positions[9 * i + 3 * k + c];
      if (rot < 0) lost++;
    }
    for (const [k, codes] of layers) {
      const code = codes[j];
      if (!code) continue;
      const node = decodePaint(code);
      if (node && 'state' in node) result[k][i] = code;
      else if (rot >= 0) result[k][i] = code;
    }
  }
  return { paint: result, positions: out, lost };
}
