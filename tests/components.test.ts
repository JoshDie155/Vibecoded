import { describe, expect, it } from 'vitest';
import * as THREE from 'three';
import { splitLoosePieces } from '../src/math/components';
import { buildTopology } from '../src/math/features';
import { box, concat, cylinder, ring, transformPositions } from './helpers/meshes';

const at = (p: Float32Array, x: number, y = 0, z = 0) => transformPositions(p, new THREE.Matrix4().makeTranslation(x, y, z));
const tris = (list: Float32Array[]) => list.map((p) => p.length / 9);

describe('splitLoosePieces', () => {
  it('ein geschlossenes Teil bleibt eins', () => {
    const p = box(10, 10, 10);
    const r = splitLoosePieces(p);
    expect(r).toHaveLength(1);
    expect(r[0]).toBe(p);
  });

  it('gibt die Nachbarschaften des Netzes zum Weiterverwenden heraus', () => {
    const p = ring(10, 3, 4, 32);
    const info: { topo?: ReturnType<typeof buildTopology> } = {};
    expect(splitLoosePieces(p, {}, info)).toHaveLength(1);
    const fresh = buildTopology(p);
    expect(info.topo?.triCount).toBe(fresh.triCount);
    expect(Array.from(info.topo!.neighbors)).toEqual(Array.from(fresh.neighbors));
  });

  it('trennt Teile, die nebeneinander auf der Platte liegen – auch dicht an dicht', () => {
    const r = splitLoosePieces(concat(box(10, 10, 10), at(box(10, 10, 10), 10.5), at(cylinder(3, 8, 24), 30)));
    expect(r).toHaveLength(3);
    expect(tris(r)).toEqual([12, 12, cylinder(3, 8, 24).length / 9]);
  });

  it('aufeinanderliegende und sich durchdringende Teile bleiben zusammen', () => {
    // Deckel liegt auf, Zapfen steckt halb im Quader
    const r = splitLoosePieces(concat(box(10, 10, 10), at(box(10, 10, 2), 0, 0, 10), at(cylinder(2, 6, 24), 0, 0, 8)));
    expect(r).toHaveLength(1);
  });

  it('ein ganz innen steckendes Teil (Auge im Kopf) bleibt beim Kopf', () => {
    const r = splitLoosePieces(concat(box(20, 20, 20), at(box(2, 2, 2), 3, 3, 8), at(box(5, 5, 5), 40)));
    expect(r).toHaveLength(2);
    expect(tris(r)).toEqual([24, 12]);
  });

  it('ein Teil, das nur im Loch eines Rings liegt, ist ein eigenes Teil', () => {
    // Umschließende Quader überschneiden sich, die Teile berühren sich aber nicht
    const r = splitLoosePieces(concat(ring(20, 10, 3, 48), at(cylinder(4, 3, 24), 0, 0, 0)));
    expect(r).toHaveLength(2);
  });
});
