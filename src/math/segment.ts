/**
 * Kandidaten für automatische Vorschläge: die markanten Flächen eines Teils – ebene Flächen
 * (z. B. Schnittflächen) und Zylinder (Zapfen, Bohrungen). Das Netz wird in einem Durchgang in
 * ebene bzw. glatte Bereiche zerlegt (jedes Dreieck wird einmal besucht); nur für die größten
 * Bereiche wird die genaue Flächenerkennung aufgerufen.
 */
import { type Feature, type Topology, detectFeature } from './features';

export interface SegmentOptions {
  /** Kleinste Fläche (mm²) eines Kandidaten; Standard: 0,2 % der Oberfläche, mindestens 1 mm². */
  minArea?: number;
  /** Höchstens so viele ebene Flächen bzw. Zylinder (die größten). */
  maxPlanes?: number;
  maxCylinders?: number;
}

const COS_PLANE = Math.cos((1.5 * Math.PI) / 180);
const COS_SMOOTH = Math.cos((35 * Math.PI) / 180);

/** Zerlegt in zusammenhängende Bereiche; `accept(von, nach)` entscheidet über jeden Schritt. */
function regions(topo: Topology, accept: (seed: number, from: number, to: number) => boolean): { seed: number; tris: number[]; area: number }[] {
  const n = topo.triCount;
  const seen = new Uint8Array(n);
  const out: { seed: number; tris: number[]; area: number }[] = [];
  const stack: number[] = [];
  for (let s = 0; s < n; s++) {
    if (seen[s] || topo.areas[s] <= 0) continue;
    seen[s] = 1;
    stack.push(s);
    const tris: number[] = [];
    let area = 0;
    while (stack.length) {
      const t = stack.pop()!;
      tris.push(t);
      area += topo.areas[t];
      for (let k = 0; k < 3; k++) {
        const nb = topo.neighbors[3 * t + k];
        if (nb < 0 || seen[nb] || topo.areas[nb] <= 0 || !accept(s, t, nb)) continue;
        seen[nb] = 1;
        stack.push(nb);
      }
    }
    out.push({ seed: s, tris, area });
  }
  return out;
}

const ndot = (topo: Topology, a: number, b: number) =>
  topo.normals[3 * a] * topo.normals[3 * b] + topo.normals[3 * a + 1] * topo.normals[3 * b + 1] + topo.normals[3 * a + 2] * topo.normals[3 * b + 2];

/** Markante ebene Flächen und Zylinder eines Teils (lokale Koordinaten), größte zuerst. */
export function findCandidateFaces(topo: Topology, opts: SegmentOptions = {}): Feature[] {
  let total = 0;
  for (let t = 0; t < topo.triCount; t++) total += topo.areas[t];
  const minArea = opts.minArea ?? Math.max(1, total * 0.002);
  const detectOpts = { maxTris: 150_000 };
  const taken = new Set<number>();
  const out: Feature[] = [];

  // Ebene Bereiche: Normale bleibt nahe der des Startdreiecks
  const planes = regions(topo, (seed, _from, to) => ndot(topo, seed, to) >= COS_PLANE)
    .filter((r) => r.area >= minArea)
    .sort((a, b) => b.area - a.area)
    .slice(0, opts.maxPlanes ?? 40);
  for (const r of planes) {
    if (taken.has(r.seed)) continue;
    const f = detectFeature(topo, r.seed, detectOpts);
    if (f.kind !== 'plane' || f.area < minArea) continue;
    f.tris.forEach((t) => taken.add(t));
    out.push(f);
  }

  // Glatte, gekrümmte Bereiche: Knick zum Nachbarn klein – darin werden Zylinder gesucht
  const smooth = regions(topo, (_seed, from, to) => ndot(topo, from, to) >= COS_SMOOTH)
    .filter((r) => r.area >= minArea && r.tris.length >= 4)
    .sort((a, b) => b.area - a.area)
    .slice(0, (opts.maxCylinders ?? 30) * 2);
  let cylinders = 0;
  for (const r of smooth) {
    if (cylinders >= (opts.maxCylinders ?? 30)) break;
    // Startdreieck in der Mitte des Bereichs (weg von Übergängen)
    const seed = r.tris[Math.floor(r.tris.length / 2)];
    if (taken.has(seed)) continue;
    const f = detectFeature(topo, seed, detectOpts);
    if (f.kind !== 'cylinder' || f.area < minArea) continue;
    f.tris.forEach((t) => taken.add(t));
    out.push(f);
    cylinders++;
  }
  return out;
}
