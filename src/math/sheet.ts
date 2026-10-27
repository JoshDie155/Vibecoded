/**
 * Bausteine der Montageanleitung: gleiche Teile zusammenfassen (Stückliste) und Nummern-Ballons um das Bild
 * verteilen, ohne dass sie sich überdecken.
 */

export interface PartShape {
  /** Dreiecke, Volumen (mm³) und Kantenlängen des Umrisses in beliebiger Reihenfolge (mm). */
  tris: number;
  volume: number;
  dims: [number, number, number];
}

/**
 * Gleiche Teile (z. B. vier Dübel) gehören zusammen: gleiche Zahl an Dreiecken, gleiches Volumen und gleiche
 * Ausdehnung (unabhängig davon, wie sie liegen). Ergebnis: Gruppen von Indizes in der Reihenfolge des ersten
 * Auftretens.
 */
export function groupIdentical(list: PartShape[]): number[][] {
  const groups: number[][] = [];
  const same = (a: PartShape, b: PartShape) => {
    if (a.tris !== b.tris) return false;
    if (Math.abs(a.volume - b.volume) > 1e-3 * Math.max(1, Math.abs(a.volume))) return false;
    const da = [...a.dims].sort((x, y) => x - y);
    const db = [...b.dims].sort((x, y) => x - y);
    return da.every((x, k) => Math.abs(x - db[k]) <= 0.02 + 1e-3 * x);
  };
  list.forEach((s, i) => {
    const g = groups.find((gr) => same(list[gr[0]], s));
    if (g) g.push(i);
    else groups.push([i]);
  });
  return groups;
}

export interface P { x: number; y: number }

/**
 * Ballons (Radius r) für Ankerpunkte im Bild (Breite w, Höhe h): jeder zunächst vom Bildmittelpunkt aus nach
 * außen versetzt, dann auseinandergeschoben, bis sie sich nicht mehr überdecken und im Bild bleiben. Die
 * Hinweislinie geht vom Anker zum Ballon.
 */
export function layoutBalloons(anchors: P[], center: P, w: number, h: number, r: number, push = 3.2 * r): P[] {
  const pts = anchors.map((a, i) => {
    let dx = a.x - center.x, dy = a.y - center.y;
    let len = Math.hypot(dx, dy);
    if (len < 1e-6) {
      // Mitte: reihum in verschiedene Richtungen
      const t = (i * 2.399) % (2 * Math.PI);
      dx = Math.cos(t); dy = Math.sin(t); len = 1;
    }
    return { x: a.x + (dx / len) * push, y: a.y + (dy / len) * push };
  });
  const clamp = (p: P) => {
    p.x = Math.min(w - r - 2, Math.max(r + 2, p.x));
    p.y = Math.min(h - r - 2, Math.max(r + 2, p.y));
  };
  pts.forEach(clamp);
  const gap = 2 * r + 6;
  for (let it = 0; it < 200; it++) {
    let moved = false;
    for (let i = 0; i < pts.length; i++) {
      for (let j = i + 1; j < pts.length; j++) {
        let dx = pts[j].x - pts[i].x, dy = pts[j].y - pts[i].y;
        let d = Math.hypot(dx, dy);
        if (d >= gap) continue;
        if (d < 1e-6) { dx = 1; dy = 0.3 * (j % 3 - 1); d = Math.hypot(dx, dy); }
        const k = (gap - d) / 2 / d;
        pts[i].x -= dx * k; pts[i].y -= dy * k;
        pts[j].x += dx * k; pts[j].y += dy * k;
        moved = true;
      }
      // Nicht direkt auf einem fremden Anker liegen
      for (let a = 0; a < anchors.length; a++) {
        if (a === i) continue;
        const dx = pts[i].x - anchors[a].x, dy = pts[i].y - anchors[a].y;
        const d = Math.hypot(dx, dy);
        if (d < r + 4 && d > 1e-6) {
          pts[i].x += (dx / d) * (r + 4 - d);
          pts[i].y += (dy / d) * (r + 4 - d);
          moved = true;
        }
      }
    }
    pts.forEach(clamp);
    if (!moved) break;
  }
  return pts;
}
