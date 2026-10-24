import { describe, expect, it } from 'vitest';
import { strToU8, zipSync } from 'fflate';
import { type Blobs, newProject, packProject, referencedKeys, unpackProject } from '../src/io/project';

const project = () => {
  const blobs: Blobs = new Map([
    ['a', new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0])],
    ['a-m0', new Float32Array([0, 0, 0, 2, 0, 0, 0, 2, 0.5])],
  ]);
  const data = newProject({
    parts: [{
      id: 3, name: 'Schild', color: '#1f79e5', layout: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 10, 0, 0, 1], visible: true, geometry: 'a',
      paint: { paint_color: ['8'] }, palette: ['#1f79e5', '#ffffff'], settings: { wall_loops: '4' }, objectSettings: { layer_height: '0.28' },
      modifiers: [{ name: 'text_shape', subtype: 'modifier_part', color: '#ffffff', settings: {}, geometry: 'a-m0' }],
    }],
    joints: [{ id: 1, a: 3, b: null, fa: [{ kind: 'plane', tris: [0], on: 3 }] }],
    baseId: 3,
    activeJointId: 1,
    settings: { onFloor: true, printer: 'P2S' },
    camera: { position: [1, 2, 3], target: [0, 0, 0] },
  });
  return { data, blobs };
};

describe('PartJoin-Projektdatei', () => {
  it('Hin- und Rückweg: Beschreibung und Netze bleiben gleich', () => {
    const { data, blobs } = project();
    const back = unpackProject(packProject(data, blobs));
    expect(back.data).toEqual(data);
    expect([...back.blobs.keys()].sort()).toEqual(['a', 'a-m0']);
    expect(Array.from(back.blobs.get('a-m0')!)).toEqual(Array.from(blobs.get('a-m0')!));
    expect(referencedKeys(back.data)).toEqual(new Set(['a', 'a-m0']));
  });

  it('meldet fremde, unvollständige und zu neue Dateien verständlich', () => {
    expect(() => unpackProject(strToU8('kein zip'))).toThrow('kein PartJoin-Projekt');
    expect(() => unpackProject(zipSync({ 'x.txt': strToU8('hallo') }))).toThrow('kein PartJoin-Projekt');
    const { data, blobs } = project();
    blobs.delete('a-m0');
    expect(() => unpackProject(packProject(data, blobs))).toThrow('fehlt ein Netz');
    const future = { ...data, version: 99 };
    expect(() => unpackProject(packProject(future, project().blobs))).toThrow('neueren PartJoin-Version');
  });
});
