import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { overhangs, thinWalls } from '../src/math/printcheck';
import { box, concat, transformPositions } from './helpers/meshes';

const I = new THREE.Matrix4().elements;

describe('Druckprüfung', () => {
  it('Quader auf der Platte: kein Überhang', () => {
    expect(overhangs(box(10, 10, 10), I, 0, 45).tris).toEqual([]);
  });
  it('T-Form: die Unterseite der Querplatte hängt über', () => {
    // Stiel 10 × 10 × 20, oben Platte 40 × 10 × 5
    const t = concat(box(10, 10, 20), transformPositions(box(40, 10, 5), new THREE.Matrix4().makeTranslation(0, 0, 20)));
    const r = overhangs(t, I, 0, 45);
    // Unterseite der Platte 40 × 10 (die innere Fläche über dem Stiel zählt hier mit, das Netz ist nicht vereinigt)
    expect(r.area).toBeCloseTo(400, 3);
  });
  it('schräge Flächen: Grenze beim eingestellten Winkel', () => {
    // Fläche, die 30° von der Senkrechten nach unten geneigt ist (Normale 60° von unten)
    const tri = (deg: number) => {
      const a = (deg * Math.PI) / 180;
      // Normale (0, −sin, −cos)… als Dreieck in einer Ebene mit dieser Normale, hoch über der Platte
      const n = new THREE.Vector3(0, Math.sin(a), -Math.cos(a));
      const u = new THREE.Vector3(1, 0, 0);
      const v = new THREE.Vector3().crossVectors(n, u);
      const p = new THREE.Vector3(0, 0, 50);
      const pts = [p, p.clone().add(u), p.clone().add(v)];
      // Umlaufsinn passend zur Normale
      const geo = new Float32Array(pts.flatMap((q) => q.toArray()));
      const e1 = pts[1].clone().sub(pts[0]), e2 = pts[2].clone().sub(pts[0]);
      if (e1.cross(e2).dot(n) < 0) geo.set([...pts[0].toArray(), ...pts[2].toArray(), ...pts[1].toArray()]);
      return geo;
    };
    // Normale 40° von unten = Fläche 50° von der Senkrechten → Überhang bei 45°
    expect(overhangs(tri(40), I, 0, 45).tris).toHaveLength(1);
    // Normale 50° von unten = 40° von der Senkrechten → geht noch
    expect(overhangs(tri(50), I, 0, 45).tris).toHaveLength(0);
    // Großzügiger (60°): auch 40° von unten noch druckbar
    expect(overhangs(tri(40), I, 0, 60).tris).toHaveLength(0);
  });
  it('gedreht: was jetzt nach unten zeigt, zählt', () => {
    const flip = new THREE.Matrix4().makeRotationX(Math.PI).premultiply(new THREE.Matrix4().makeTranslation(0, 0, 30)).elements;
    // Umgedrehter Quader in der Luft: die alte Oberseite zeigt nach unten
    expect(overhangs(box(10, 10, 10), flip, 0, 45).area).toBeCloseTo(100, 3);
  });
  it('dünne Wände nach Messwerten', () => {
    const th = new Float32Array([5, 0.5, Infinity, NaN, 0.79]);
    const pos = new Float32Array(5 * 9).map((_, i) => (i % 9 === 3 || i % 9 === 7 ? 1 : 0));
    expect(thinWalls(pos, th, 0.8).tris).toEqual([1, 4]);
  });
});
