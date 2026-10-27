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

/** Kleinster Schwerpunktanteil von p im Dreieck x, y, z (in dessen Ebene; ≥ 0 heißt: liegt darin). */
function insideness(p: V3, x: V3, y: V3, z: V3): number {
  const v0 = [y[0] - x[0], y[1] - x[1], y[2] - x[2]];
  const v1 = [z[0] - x[0], z[1] - x[1], z[2] - x[2]];
  const v2 = [p[0] - x[0], p[1] - x[1], p[2] - x[2]];
  const dot = (a: number[], b: number[]) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const d00 = dot(v0, v0), d01 = dot(v0, v1), d11 = dot(v1, v1), d20 = dot(v2, v0), d21 = dot(v2, v1);
  const den = d00 * d11 - d01 * d01;
  if (!(Math.abs(den) > 1e-30)) return -Infinity;
  const v = (d11 * d20 - d01 * d21) / den;
  const w = (d00 * d21 - d01 * d20) / den;
  return Math.min(1 - v - w, v, w);
}

/**
 * Häufigster Zustand einer fein bemalten Fläche (Dreieck a, b, c) auf dem Stück p0, p1, p2, das in ihr
 * liegt – abgelesen an einigen Punkten des Stücks.
 */
export function paintStateOn(root: PaintNode, a: V3, b: V3, c: V3, p0: V3, p1: V3, p2: V3): number {
  const leaves: { state: number; v: [V3, V3, V3] }[] = [];
  paintLeaves(root, a, b, c, (state, x, y, z) => leaves.push({ state, v: [x, y, z] }));
  const at = (u: number, v: number, w: number): V3 => [0, 1, 2].map((k) => u * p0[k] + v * p1[k] + w * p2[k]) as V3;
  const samples = [at(1 / 3, 1 / 3, 1 / 3), at(2 / 3, 1 / 6, 1 / 6), at(1 / 6, 2 / 3, 1 / 6), at(1 / 6, 1 / 6, 2 / 3), at(0.5, 0.5, 0), at(0, 0.5, 0.5), at(0.5, 0, 0.5)];
  const votes = new Map<number, number>();
  samples.forEach((q, i) => {
    let best = -Infinity;
    let state = 0;
    for (const l of leaves) {
      const w = insideness(q, l.v[0], l.v[1], l.v[2]);
      if (w > best) { best = w; state = l.state; }
    }
    // Mitte zählt doppelt (entscheidet bei Gleichstand)
    votes.set(state, (votes.get(state) ?? 0) + (i === 0 ? 2 : 1));
  });
  return [...votes].sort((x, y) => y[1] - x[1])[0][0];
}

/**
 * Bemalung auf ein neues Netz übertragen, dessen Dreiecke aus dem alten stammen (source[i] = Dreieck im
 * alten Netz oder −1 für neue Flächen, z. B. nach einer Bohrung oder einem Schnitt). Einfarbige Dreiecke
 * behalten ihre Farbe auch zerteilt; fein bemalte bleiben fein, wenn sie unverändert blieben – deren Ecken
 * kommen dafür in die alte Reihenfolge (die Kodierung bezieht sich auf sie), positions wird entsprechend
 * umgeschrieben. Geteilte oder veränderte fein bemalte Dreiecke bekommen die Farbe, die an ihrer Stelle lag.
 * lost zählt Dreiecke, deren feine Bemalung so vereinfacht wurde.
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
      if (!node) continue;
      if ('state' in node || rot >= 0) result[k][i] = code;
      else {
        const v = (arr: Float32Array, t: number, c: number): V3 => [arr[9 * t + 3 * c], arr[9 * t + 3 * c + 1], arr[9 * t + 3 * c + 2]];
        result[k][i] = encodePaint({ state: paintStateOn(node, v(oldPositions, j, 0), v(oldPositions, j, 1), v(oldPositions, j, 2), v(positions, i, 0), v(positions, i, 1), v(positions, i, 2)) });
      }
    }
  }
  return { paint: result, positions: out, lost };
}

// ---------------------------------------------------------------- Malen (Pinsel)

const sub3 = (a: V3, b: V3): V3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot3 = (a: V3, b: V3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const dist3 = (a: V3, b: V3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

/** Abstand eines Punkts zum Dreieck (nächster Punkt nach Ericson, „Real-Time Collision Detection“). */
export function triangleDistance(p: V3, a: V3, b: V3, c: V3): number {
  const ab = sub3(b, a), ac = sub3(c, a), ap = sub3(p, a);
  const d1 = dot3(ab, ap), d2 = dot3(ac, ap);
  if (d1 <= 0 && d2 <= 0) return dist3(p, a);
  const bp = sub3(p, b);
  const d3 = dot3(ab, bp), d4 = dot3(ac, bp);
  if (d3 >= 0 && d4 <= d3) return dist3(p, b);
  const vc = d1 * d4 - d3 * d2;
  if (vc <= 0 && d1 >= 0 && d3 <= 0) {
    const v = d1 / (d1 - d3);
    return dist3(p, [a[0] + v * ab[0], a[1] + v * ab[1], a[2] + v * ab[2]]);
  }
  const cp = sub3(p, c);
  const d5 = dot3(ab, cp), d6 = dot3(ac, cp);
  if (d6 >= 0 && d5 <= d6) return dist3(p, c);
  const vb = d5 * d2 - d1 * d6;
  if (vb <= 0 && d2 >= 0 && d6 <= 0) {
    const w = d2 / (d2 - d6);
    return dist3(p, [a[0] + w * ac[0], a[1] + w * ac[1], a[2] + w * ac[2]]);
  }
  const va = d3 * d6 - d5 * d4;
  if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) {
    const w = (d4 - d3) / (d4 - d3 + (d5 - d6));
    return dist3(p, [b[0] + w * (c[0] - b[0]), b[1] + w * (c[1] - b[1]), b[2] + w * (c[2] - b[2])]);
  }
  const den = 1 / (va + vb + vc);
  const v = vb * den, w = vc * den;
  return dist3(p, [a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w]);
}

/** Ecken der Kinder eines zerlegten Knotens – wie in paintLeaves (TriangleSelector::perform_split). */
function childVerts(n: Extract<PaintNode, { split: number }>, v: V3[]): V3[][] {
  const s = n.special % 3;
  const [A, B, C] = [v[s], v[(s + 1) % 3], v[(s + 2) % 3]];
  if (n.split === 1) {
    const m = mid(B, C);
    return [[A, B, m], [m, C, A]];
  }
  if (n.split === 2) {
    const m1 = mid(B, A);
    const m2 = mid(A, C);
    return [[A, m1, m2], [m1, B, m2], [B, C, m2]];
  }
  const m01 = mid(A, B);
  const m12 = mid(B, C);
  const m20 = mid(C, A);
  return [[A, m01, m20], [m01, B, m12], [m12, C, m20], [m01, m12, m20]];
}

export interface BrushOptions {
  /** Mitte und Radius der Pinselkugel (Koordinaten des Netzes). */
  center: V3;
  radius: number;
  /** Neuer Zustand (Filament; 0 = Filament des Teils, also radieren). */
  state: number;
  /** Teildreiecke am Rand höchstens so lang (Kante) – feiner wird nicht zerlegt. */
  minEdge: number;
  /** Höchste Zerlegungstiefe. */
  maxDepth?: number;
}

/**
 * Pinselstrich auf einem Dreieck: Was in der Kugel liegt, bekommt den Zustand. Liegt das Dreieck nur
 * teilweise darin, wird es (wie im Slicer) in vier Teildreiecke zerlegt, bis die Stücke klein genug sind;
 * gleich bemalte Geschwister werden danach wieder zusammengefasst. Liefert die neue Kodierung.
 */
export function brushPaint(code: string, a: V3, b: V3, c: V3, o: BrushOptions): string {
  const root = decodePaint(code) ?? { state: 0 };
  const maxDepth = o.maxDepth ?? 9;
  const r2 = o.radius * o.radius;
  const inside = (p: V3) => {
    const d = sub3(p, o.center);
    return dot3(d, d) <= r2;
  };
  const visit = (n: PaintNode, v: V3[], depth: number): PaintNode => {
    if (inside(v[0]) && inside(v[1]) && inside(v[2])) return { state: o.state };
    if (triangleDistance(o.center, v[0], v[1], v[2]) > o.radius) return n;
    const edge = Math.max(dist3(v[0], v[1]), dist3(v[1], v[2]), dist3(v[2], v[0]));
    if ('state' in n) {
      if (n.state === o.state) return n;
      // Klein genug: nach der Mitte entscheiden
      if (edge <= o.minEdge || depth >= maxDepth) {
        const m: V3 = [(v[0][0] + v[1][0] + v[2][0]) / 3, (v[0][1] + v[1][1] + v[2][1]) / 3, (v[0][2] + v[1][2] + v[2][2]) / 3];
        return inside(m) ? { state: o.state } : n;
      }
      n = { split: 3, special: 0, children: [{ state: n.state }, { state: n.state }, { state: n.state }, { state: n.state }] };
    }
    const sn = n as Extract<PaintNode, { split: number }>;
    const verts = childVerts(sn, v);
    const children = sn.children.map((ch, k) => visit(ch, verts[k], depth + 1));
    // Alle Kinder gleich: wieder ein Blatt
    const first = children[0];
    if ('state' in first && children.every((ch) => 'state' in ch && ch.state === first.state)) return { state: first.state };
    return { ...sn, children };
  };
  return encodePaint(visit(root, [a, b, c], 0));
}

/** Kodierung für ein ganz in einem Zustand bemaltes Dreieck. */
export const solidPaint = (state: number) => encodePaint({ state });
