/**
 * Netze prüfen und reparieren – damit Zerschneiden, Vereinigen und die Slicer ein geschlossenes Netz
 * bekommen (viele heruntergeladene STL-Dateien sind es nicht):
 *
 * - Eckpunkte verschweißen, die fast aufeinanderliegen (schließt feine Risse),
 * - zusammengefallene und doppelt gespeicherte Dreiecke entfernen; liegen zwei Dreiecke gegensinnig
 *   aufeinander (Innenwand zwischen zwei Körpern), fallen beide weg,
 * - den Umlaufsinn benachbarter Dreiecke vereinheitlichen und jedes Stück nach außen drehen,
 * - Löcher schließen: den Rand als Vieleck abtragen (Ohren abschneiden), bei verwinkelten Rändern ein
 *   Fächer um die Mitte.
 *
 * Die Reihenfolge der übrigen Dreiecke bleibt, neue kommen ans Ende – so lässt sich Bemalung übertragen.
 */
import { weld } from './features';

export interface MeshHealth {
  triangles: number;
  /** Kanten mit nur einem Dreieck (Ränder von Löchern). */
  openEdges: number;
  /** Kanten mit mehr als zwei Dreiecken. */
  nonManifoldEdges: number;
  /** Kanten, an denen benachbarte Dreiecke gleichsinnig umlaufen (verdrehte Flächen). */
  flippedEdges: number;
  /** Zusammengefallene Dreiecke (ohne Fläche) – schaden nicht. */
  degenerate: number;
  /** Doppelt gespeicherte Dreiecke (auch gegensinnige Paare als Innenwand). */
  duplicates: number;
  /** Zusammenhängende Stücke. */
  shells: number;
  /** Vorzeichenbehaftetes Volumen (negativ: Normalen zeigen nach innen). */
  volume: number;
  /** Geschlossen, einheitlich und nach außen ausgerichtet – bereit zum Zerschneiden. */
  closed: boolean;
}

export interface RepairReport {
  welded: number;
  removed: number;
  flipped: number;
  holes: number;
  holeTriangles: number;
  /** Danach noch übrig (nicht behebbar, z. B. Kanten mit drei Dreiecken). */
  after: MeshHealth;
}

export interface RepairResult {
  positions: Float32Array;
  /** Altes Dreieck → neues (−1 = entfernt). */
  map: Int32Array;
  /** Je neuem Dreieck: Umlaufsinn umgedreht (1) – dessen Bemalung gilt so nicht mehr. */
  flippedTris: Uint8Array;
  report: RepairReport;
}

function diagonal(p: Float32Array): number {
  const mn = [Infinity, Infinity, Infinity];
  const mx = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < p.length; i += 3) {
    for (let k = 0; k < 3; k++) {
      const v = p[i + k];
      if (!Number.isFinite(v)) continue;
      if (v < mn[k]) mn[k] = v;
      if (v > mx[k]) mx[k] = v;
    }
  }
  const d = Math.hypot(mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2]);
  return Number.isFinite(d) ? d : 0;
}

/** Verschweißtes, indiziertes Netz: Ecken je Dreieck und eine Position je Punkt. */
function indexedMesh(positions: Float32Array) {
  const triCount = Math.floor(positions.length / 9);
  const tol = Math.max(diagonal(positions) * 1e-5, 1e-9);
  const { vertIds, vertCount, firstCorner } = weld(positions, triCount * 3, tol);
  const verts: Float64Array<ArrayBuffer> = new Float64Array(vertCount * 3);
  for (let v = 0; v < vertCount; v++) for (let k = 0; k < 3; k++) verts[3 * v + k] = positions[3 * firstCorner[v] + k];
  return { triCount, tris: vertIds, vertCount, verts };
}

/** Kanten → Dreiecke (mit Richtung): Schlüssel aus beiden Punktnummern. */
function edgeUses(tris: Int32Array, alive: (t: number) => boolean, triCount: number, vertCount: number) {
  const uses = new Map<number, number[]>();
  for (let t = 0; t < triCount; t++) {
    if (!alive(t)) continue;
    for (let k = 0; k < 3; k++) {
      const a = tris[3 * t + k];
      const b = tris[3 * t + ((k + 1) % 3)];
      const key = a < b ? a * vertCount + b : b * vertCount + a;
      // Vorzeichen: +(t+1) wenn a→b aufsteigend läuft, sonst −(t+1)
      const entry = a < b ? t + 1 : -(t + 1);
      const list = uses.get(key);
      if (list) list.push(entry);
      else uses.set(key, [entry]);
    }
  }
  return uses;
}

function signedVolume(tris: ArrayLike<number>, verts: Float64Array, list: Iterable<number>, flip?: Uint8Array): number {
  let v = 0;
  for (const t of list) {
    const a = tris[3 * t], b0 = tris[3 * t + 1], c0 = tris[3 * t + 2];
    const [b, c] = flip?.[t] ? [c0, b0] : [b0, c0];
    const ax = verts[3 * a], ay = verts[3 * a + 1], az = verts[3 * a + 2];
    const bx = verts[3 * b], by = verts[3 * b + 1], bz = verts[3 * b + 2];
    const cx = verts[3 * c], cy = verts[3 * c + 1], cz = verts[3 * c + 2];
    v += (ax * (by * cz - bz * cy) - ay * (bx * cz - bz * cx) + az * (bx * cy - by * cx)) / 6;
  }
  return v;
}

/**
 * Zusammengefallene und doppelte Dreiecke. Je Satz gleicher Ecken zählt der Umlaufsinn: gleichsinnige
 * Kopien sind Duplikate, gegensinnige heben sich auf (Innenwand) – es bleibt höchstens eins.
 */
function deadTriangles(tris: Int32Array, triCount: number): Uint8Array {
  const dead = new Uint8Array(triCount);
  const groups = new Map<string, number[]>();
  for (let t = 0; t < triCount; t++) {
    const a = tris[3 * t], b = tris[3 * t + 1], c = tris[3 * t + 2];
    if (a === b || b === c || a === c) { dead[t] = 1; continue; }
    const lo = Math.min(a, b, c), hi = Math.max(a, b, c);
    const key = `${lo},${a + b + c - lo - hi},${hi}`;
    const g = groups.get(key);
    if (g) g.push(t);
    else groups.set(key, [t]);
  }
  /** Umlaufsinn als Vorzeichen der Permutation zur sortierten Reihenfolge. */
  const sense = (t: number) => {
    const a = tris[3 * t], b = tris[3 * t + 1], c = tris[3 * t + 2];
    return (a < b ? 1 : 0) + (b < c ? 1 : 0) + (c < a ? 1 : 0) === 2 ? 1 : -1;
  };
  for (const g of groups.values()) {
    if (g.length < 2) continue;
    const plus = g.filter((t) => sense(t) > 0);
    const minus = g.filter((t) => sense(t) < 0);
    const keep = plus.length > minus.length ? plus[0] : minus.length > plus.length ? minus[0] : -1;
    // 2 = doppelt (1 = zusammengefallen)
    for (const t of g) if (t !== keep) dead[t] = 2;
  }
  return dead;
}

export function meshHealth(positions: Float32Array): MeshHealth {
  const { triCount, tris, vertCount, verts } = indexedMesh(positions);
  const dead = deadTriangles(tris, triCount);
  const uses = edgeUses(tris, (t) => !dead[t], triCount, vertCount);
  let openEdges = 0, nonManifoldEdges = 0, flippedEdges = 0;
  // Stücke über Kanten zusammenfassen
  const root = new Int32Array(triCount).map((_, i) => i);
  const find = (x: number): number => {
    while (root[x] !== x) { root[x] = root[root[x]]; x = root[x]; }
    return x;
  };
  for (const list of uses.values()) {
    if (list.length === 1) openEdges++;
    else if (list.length > 2) nonManifoldEdges++;
    else if (Math.sign(list[0]) === Math.sign(list[1])) flippedEdges++;
    for (let i = 1; i < list.length; i++) {
      const x = find(Math.abs(list[0]) - 1), y = find(Math.abs(list[i]) - 1);
      if (x !== y) root[x] = y;
    }
  }
  const alive = [...Array(triCount).keys()].filter((t) => !dead[t]);
  const shells = new Set(alive.map(find)).size;
  const duplicates = dead.reduce((n, d) => n + (d === 2 ? 1 : 0), 0);
  const degenerate = triCount - alive.length - duplicates;
  const volume = signedVolume(tris, verts, alive);
  return {
    triangles: triCount, openEdges, nonManifoldEdges, flippedEdges, degenerate, duplicates, shells, volume,
    // Zusammengefallene Dreiecke (ohne Fläche) schaden nicht, doppelte schon (manifold lehnt sie ab).
    // Nach innen gedreht (negatives Volumen) zählt nicht als geschlossen.
    closed: openEdges === 0 && nonManifoldEdges === 0 && flippedEdges === 0 && duplicates === 0 && alive.length > 0 && volume > 0,
  };
}

// ---------------------------------------------------------------- Löcher schließen

type V3 = [number, number, number];

/**
 * Vieleck (Punktnummern, Umlaufsinn = gewünschte Ausrichtung der neuen Dreiecke) in Dreiecke zerlegen:
 * Ohren abschneiden in der Ebene der Newell-Normale; klappt das nicht, Fächer um den Schwerpunkt.
 */
function fillPolygon(loop: number[], verts: Float64Array, addVertex: (p: V3) => number): number[][] {
  const k = loop.length;
  if (k === 3) return [[loop[0], loop[1], loop[2]]];
  const P = loop.map((v) => [verts[3 * v], verts[3 * v + 1], verts[3 * v + 2]] as V3);
  // Newell-Normale: zeigt so, dass das Vieleck um sie gegen den Uhrzeigersinn läuft
  const n: V3 = [0, 0, 0];
  for (let i = 0; i < k; i++) {
    const a = P[i], b = P[(i + 1) % k];
    n[0] += (a[1] - b[1]) * (a[2] + b[2]);
    n[1] += (a[2] - b[2]) * (a[0] + b[0]);
    n[2] += (a[0] - b[0]) * (a[1] + b[1]);
  }
  const nl = Math.hypot(...n);
  const fan = () => {
    const c: V3 = [0, 0, 0];
    for (const p of P) for (let j = 0; j < 3; j++) c[j] += p[j] / k;
    const ci = addVertex(c);
    return loop.map((v, i) => [ci, v, loop[(i + 1) % k]]);
  };
  if (nl < 1e-12 || k > 3000) return fan();
  for (let j = 0; j < 3; j++) n[j] /= nl;
  // Ebene: u, v mit u × v = n
  const u: V3 = Math.abs(n[0]) < 0.9 ? [0, n[2], -n[1]] : [-n[2], 0, n[0]];
  const ul = Math.hypot(...u);
  for (let j = 0; j < 3; j++) u[j] /= ul;
  const w: V3 = [n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2], n[0] * u[1] - n[1] * u[0]];
  const q = P.map((p) => [p[0] * u[0] + p[1] * u[1] + p[2] * u[2], p[0] * w[0] + p[1] * w[1] + p[2] * w[2]]);
  // Nicht eben: Fächer um die Mitte statt flacher Zerlegung
  const off = P.map((p) => p[0] * n[0] + p[1] * n[1] + p[2] * n[2]);
  const spread = Math.max(...off) - Math.min(...off);
  let size = 0;
  for (const a of q) for (const b of q) size = Math.max(size, Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]));
  if (spread > 0.25 * size) return fan();
  const cross2 = (a: number[], b: number[], c: number[]) => (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
  const idx = [...Array(k).keys()];
  const out: number[][] = [];
  let guard = 0;
  while (idx.length > 3 && guard++ < k * k) {
    let cut = false;
    for (let i = 0; i < idx.length; i++) {
      const ia = idx[(i + idx.length - 1) % idx.length], ib = idx[i], ic = idx[(i + 1) % idx.length];
      const a = q[ia], b = q[ib], c = q[ic];
      if (cross2(a, b, c) <= 1e-12) continue; // spitze Ecke nach innen
      let empty = true;
      for (const j of idx) {
        if (j === ia || j === ib || j === ic) continue;
        const p = q[j];
        if (cross2(a, b, p) >= 0 && cross2(b, c, p) >= 0 && cross2(c, a, p) >= 0) { empty = false; break; }
      }
      if (!empty) continue;
      out.push([loop[ia], loop[ib], loop[ic]]);
      idx.splice(i, 1);
      cut = true;
      break;
    }
    if (!cut) return fan();
  }
  out.push([loop[idx[0]], loop[idx[1]], loop[idx[2]]]);
  return out;
}

export function repairMesh(positions: Float32Array): RepairResult {
  const { triCount, tris, vertCount, verts: v0 } = indexedMesh(positions);
  const extra: number[] = [];
  const addVertex = (p: V3) => {
    extra.push(...p);
    return vertCount + extra.length / 3 - 1;
  };
  // Wie viele Ecken tatsächlich bewegt wurden
  let welded = 0;
  for (let c = 0; c < triCount * 3; c++) {
    const v = tris[c];
    if (positions[3 * c] !== v0[3 * v] || positions[3 * c + 1] !== v0[3 * v + 1] || positions[3 * c + 2] !== v0[3 * v + 2]) welded++;
  }
  const dead = deadTriangles(tris, triCount);
  const alive = (t: number) => !dead[t];
  // Umlaufsinn angleichen: Breitensuche über Kanten mit genau zwei Dreiecken
  const flip = new Uint8Array(triCount);
  const comp = new Int32Array(triCount).fill(-1);
  const uses = edgeUses(tris, alive, triCount, vertCount);
  const nb: number[][] = Array.from({ length: triCount }, () => []);
  for (const list of uses.values()) {
    if (list.length !== 2) continue;
    const [x, y] = list;
    const tx = Math.abs(x) - 1, ty = Math.abs(y) - 1;
    // Gleiches Vorzeichen = gleiche Richtung = einer von beiden ist verdreht
    const same = Math.sign(x) === Math.sign(y) ? 1 : 0;
    nb[tx].push(ty, same);
    nb[ty].push(tx, same);
  }
  let comps = 0;
  for (let s = 0; s < triCount; s++) {
    if (dead[s] || comp[s] >= 0) continue;
    const queue = [s];
    comp[s] = comps;
    for (let qi = 0; qi < queue.length; qi++) {
      const t = queue[qi];
      const list = nb[t];
      for (let i = 0; i < list.length; i += 2) {
        const o = list[i];
        if (comp[o] >= 0) continue;
        comp[o] = comps;
        flip[o] = flip[t] ^ list[i + 1];
        queue.push(o);
      }
    }
    comps++;
  }
  // Löcher: Randkanten in der (angeglichenen) Richtung ihres Dreiecks zu Schleifen verbinden
  const out = new Map<number, { to: number; tri: number }[]>();
  for (const [key, list] of uses) {
    if (list.length !== 1) continue;
    const t = Math.abs(list[0]) - 1;
    const lo = Math.floor(key / vertCount), hi = key % vertCount;
    // Vorzeichen: Richtung im ursprünglichen Dreieck (aufsteigend?), umgedrehte Dreiecke andersherum
    const asc = (list[0] > 0) !== !!flip[t];
    const from = asc ? lo : hi;
    const l = out.get(from);
    const e = { to: asc ? hi : lo, tri: t };
    if (l) l.push(e);
    else out.set(from, [e]);
  }
  const fills: { tri: number[]; comp: number }[] = [];
  let holes = 0;
  const fillLoop = (loop: number[], c: number) => {
    if (loop.length < 3) return;
    // Die neuen Dreiecke laufen gegen den Rand: Schleife umkehren
    holes++;
    for (const tri of fillPolygon([...loop].reverse(), v0, addVertex)) fills.push({ tri, comp: c });
  };
  for (const start of [...out.keys()]) {
    while (out.get(start)?.length) {
      const first = out.get(start)!.pop()!;
      const c = comp[first.tri];
      const loop = [start];
      const at = new Map([[start, 0]]);
      let cur = first.to;
      let ok = true;
      while (cur !== start) {
        // Berühren sich zwei Ränder in einem Punkt: die geschlossene Teilschleife einzeln füllen
        const i = at.get(cur);
        if (i !== undefined) {
          const sub = loop.splice(i);
          for (const v of sub) at.delete(v);
          fillLoop(sub, c);
        }
        at.set(cur, loop.length);
        loop.push(cur);
        const next = out.get(cur);
        if (!next?.length) { ok = false; break; }
        cur = next.pop()!.to;
      }
      if (ok) fillLoop(loop, c);
    }
  }
  const verts = extra.length ? mergeVerts(v0, extra) : v0;
  // Jedes Stück nach außen drehen (Volumen positiv) – mit seinen neuen Deckeln
  const byComp: number[][] = Array.from({ length: comps }, () => []);
  for (let t = 0; t < triCount; t++) if (!dead[t]) byComp[comp[t]].push(t);
  const fillTris = new Int32Array(fills.length * 3);
  fills.forEach((f, i) => fillTris.set(f.tri, 3 * i));
  const compFlip = new Uint8Array(comps);
  for (let c = 0; c < comps; c++) {
    const own = signedVolume(tris, verts, byComp[c], flip);
    const caps = signedVolume(fillTris, verts, fills.map((f, i) => (f.comp === c ? i : -1)).filter((i) => i >= 0));
    if (own + caps < 0) compFlip[c] = 1;
  }
  // Ausgabe: übrige Dreiecke in alter Reihenfolge, dann die Deckel
  const kept = [...Array(triCount).keys()].filter((t) => !dead[t]);
  const total = kept.length + fills.length;
  const res = new Float32Array(total * 9);
  const map = new Int32Array(triCount).fill(-1);
  const flippedTris = new Uint8Array(total);
  const put = (i: number, a: number, b: number, c: number) => {
    for (const [k, v] of [a, b, c].entries()) for (let j = 0; j < 3; j++) res[9 * i + 3 * k + j] = verts[3 * v + j];
  };
  let flipped = 0;
  kept.forEach((t, i) => {
    map[t] = i;
    const f = flip[t] ^ compFlip[comp[t]];
    const [a, b, c] = [tris[3 * t], tris[3 * t + 1], tris[3 * t + 2]];
    if (f) { put(i, a, c, b); flippedTris[i] = 1; flipped++; } else put(i, a, b, c);
  });
  fills.forEach((f, i) => {
    const [a, b, c] = f.tri;
    if (compFlip[f.comp]) put(kept.length + i, a, c, b);
    else put(kept.length + i, a, b, c);
  });
  return {
    positions: res,
    map,
    flippedTris,
    report: { welded, removed: triCount - kept.length, flipped, holes, holeTriangles: fills.length, after: meshHealth(res) },
  };
}

function mergeVerts(base: Float64Array, extra: number[]): Float64Array<ArrayBuffer> {
  const out = new Float64Array(base.length + extra.length);
  out.set(base);
  out.set(extra, base.length);
  return out;
}
