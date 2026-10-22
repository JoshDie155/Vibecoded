import { describe, expect, it } from 'vitest';
import { strFromU8, strToU8, unzipSync, zipSync } from 'fflate';
import { colorSlots, read3MF, write3MF } from '../src/io/threemf';
import { box, cylinder, transformPositions } from './helpers/meshes';
import * as THREE from 'three';

const bbox = (p: Float32Array) => {
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < p.length; i += 3)
    for (let k = 0; k < 3; k++) { min[k] = Math.min(min[k], p[i + k]); max[k] = Math.max(max[k], p[i + k]); }
  return { min, max };
};

const RELS = `<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>`;

/** Netz als 3MF-XML (vertices/triangles), Positionen nicht indiziert. */
function meshXml(p: Float32Array, triAttr = ''): string {
  let v = '';
  let t = '';
  for (let i = 0; i < p.length / 3; i++) v += `<vertex x="${p[i * 3]}" y="${p[i * 3 + 1]}" z="${p[i * 3 + 2]}"/>`;
  for (let i = 0; i < p.length / 9; i++) t += `<triangle v1="${i * 3}" v2="${i * 3 + 1}" v3="${i * 3 + 2}"${triAttr}/>`;
  return `<mesh><vertices>${v}</vertices><triangles>${t}</triangles></mesh>`;
}

describe('write3MF / read3MF', () => {
  it('mehrfarbig: Teile, Namen und Farben bleiben beim Hin- und Rückweg erhalten', () => {
    const parts = [
      { name: 'Körper', positions: box(20, 20, 10), color: '#FF0000' },
      { name: 'Kopf', positions: transformPositions(cylinder(5, 6, 24), new THREE.Matrix4().makeTranslation(0, 0, 10)), color: '#00aa00' },
      { name: 'Hut', positions: transformPositions(box(6, 6, 2), new THREE.Matrix4().makeTranslation(0, 0, 16)), color: '#ff0000' },
    ];
    const back = read3MF(write3MF(parts, 'multicolor'));
    expect(back.map((p) => p.name)).toEqual(['Körper', 'Kopf', 'Hut']);
    expect(back.map((p) => p.color)).toEqual(['#ff0000', '#00aa00', '#ff0000']);
    expect(new Set(back.map((p) => p.group)).size).toBe(1);
    back.forEach((p, i) => {
      expect(p.positions.length).toBe(parts[i].positions.length);
      const a = bbox(p.positions);
      const b = bbox(parts[i].positions);
      for (let k = 0; k < 3; k++) {
        expect(a.min[k]).toBeCloseTo(b.min[k], 4);
        expect(a.max[k]).toBeCloseTo(b.max[k], 4);
      }
    });
  });

  it('schreibt Extruder je Teil und die Farbliste für den Slicer', () => {
    const parts = [
      { name: 'A', positions: box(10, 10, 10), color: '#123456' },
      { name: 'B', positions: box(5, 5, 5), color: '#abcdef' },
      { name: 'C', positions: box(2, 2, 2), color: '#123456' },
    ];
    const files = unzipSync(write3MF(parts, 'multicolor'));
    const cfg = strFromU8(files['Metadata/Slic3r_PE_model.config']);
    const extruders = [...cfg.matchAll(/key="extruder" value="(\d+)"/g)].map((m) => Number(m[1]));
    expect(extruders).toEqual([1, 2, 1]);
    // Farben stehen standardkonform im Modell (keine Slicer-Einstellungsdatei, die als Projekt gelten könnte)
    expect(strFromU8(files['3D/3dmodel.model'])).toContain('<base name="Farbe 1" displaycolor="#123456FF"/><base name="Farbe 2" displaycolor="#ABCDEFFF"/>');
    expect(files['Metadata/Slic3r_PE.config']).toBeUndefined();
    expect(colorSlots(parts)).toEqual(['#123456', '#abcdef']);
    // Dreiecksbereiche schließen lückenlos aneinander an
    const ranges = [...cfg.matchAll(/firstid="(\d+)" lastid="(\d+)"/g)].map((m) => [Number(m[1]), Number(m[2])]);
    expect(ranges[0][0]).toBe(0);
    for (let i = 1; i < ranges.length; i++) expect(ranges[i][0]).toBe(ranges[i - 1][1] + 1);
  });

  it('ein Körper: alle Dreiecke in einem Objekt ohne Farbangaben', () => {
    const files = unzipSync(write3MF([{ name: 'A', positions: box(10, 10, 10), color: '#ff0000' }, { name: 'B', positions: box(2, 2, 2), color: '#00ff00' }], 'single'));
    const model = strFromU8(files['3D/3dmodel.model']);
    expect(model.match(/<object /g)).toHaveLength(1);
    expect(model).not.toContain('basematerials');
    expect(files['Metadata/Slic3r_PE_model.config']).toBeUndefined();
  });
});

describe('read3MF: Dateien aus Slicern', () => {
  it('Bambu Studio / OrcaSlicer: Teile in Unterdateien, Farben aus der Filament-Zuordnung', () => {
    const sub = `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06">
<resources>
<object id="1" p:UUID="a" type="model">${meshXml(box(10, 10, 10))}</object>
<object id="2" p:UUID="b" type="model">${meshXml(cylinder(2, 4, 12))}</object>
</resources><build/></model>`;
    const main = `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06">
<metadata name="Application">BambuStudio-01.09.00.70</metadata>
<resources>
<object id="3" p:UUID="c" type="model"><components>
<component p:path="/3D/Objects/object_1.model" objectid="1" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>
<component p:path="/3D/Objects/object_1.model" objectid="2" transform="1 0 0 0 1 0 0 0 1 0 0 10"/>
</components></object>
</resources>
<build><item objectid="3" transform="1 0 0 0 1 0 0 0 1 100 50 0" printable="1"/></build></model>`;
    const settings = `<?xml version="1.0" encoding="UTF-8"?>
<config>
  <object id="3">
    <metadata key="name" value="Figur"/>
    <metadata key="extruder" value="1"/>
    <part id="1" subtype="normal_part"><metadata key="name" value="Sockel"/><metadata key="extruder" value="2"/></part>
    <part id="2" subtype="normal_part"><metadata key="name" value="Knopf"/></part>
  </object>
</config>`;
    const project = JSON.stringify({ filament_colour: ['#FFFFFF', '#FF8000', '#0000FF'] });
    const data = zipSync({
      '_rels/.rels': strToU8(RELS),
      '3D/3dmodel.model': strToU8(main),
      '3D/Objects/object_1.model': strToU8(sub),
      'Metadata/model_settings.config': strToU8(settings),
      'Metadata/project_settings.config': strToU8(project),
    });
    const parts = read3MF(data);
    expect(parts.map((p) => p.name)).toEqual(['Sockel', 'Knopf']);
    // Sockel hat Extruder 2, der Knopf erbt Extruder 1 vom Objekt
    expect(parts.map((p) => p.color)).toEqual(['#ff8000', '#ffffff']);
    expect(parts[0].group).toBe(parts[1].group);
    const b = bbox(parts[1].positions);
    expect(b.min[0]).toBeCloseTo(98, 4); // Lage aus Komponente + Druckbett-Position
    expect(b.min[2]).toBeCloseTo(10, 4);
  });

  it('PrusaSlicer: Teile als Dreiecksbereiche eines Netzes mit Extruderfarben', () => {
    const a = box(10, 10, 10);
    const b = transformPositions(box(4, 4, 4), new THREE.Matrix4().makeTranslation(0, 0, 12));
    const all = new Float32Array(a.length + b.length);
    all.set(a);
    all.set(b, a.length);
    const model = `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
<resources><object id="1" type="model">${meshXml(all)}</object></resources>
<build><item objectid="1"/></build></model>`;
    const cfg = `<?xml version="1.0" encoding="UTF-8"?>
<config><object id="1" instances_count="1"><metadata type="object" key="name" value="Turm"/>
<volume firstid="0" lastid="${a.length / 9 - 1}"><metadata type="volume" key="name" value="Unten"/><metadata type="volume" key="extruder" value="1"/></volume>
<volume firstid="${a.length / 9}" lastid="${all.length / 9 - 1}"><metadata type="volume" key="name" value="Oben"/><metadata type="volume" key="extruder" value="2"/></volume>
</object></config>`;
    const ini = '; extruder_colour = "#FF0000;#00FF00"\n';
    const parts = read3MF(zipSync({
      '_rels/.rels': strToU8(RELS),
      '3D/3dmodel.model': strToU8(model),
      'Metadata/Slic3r_PE_model.config': strToU8(cfg),
      'Metadata/Slic3r_PE.config': strToU8(ini),
    }));
    expect(parts.map((p) => [p.name, p.color, p.positions.length])).toEqual([
      ['Unten', '#ff0000', a.length],
      ['Oben', '#00ff00', b.length],
    ]);
  });

  it('Standard-3MF: Farben je Dreieck trennen ein Objekt in farbige Teile; Einheiten werden umgerechnet', () => {
    const a = box(1, 1, 1);
    const model = `<?xml version="1.0" encoding="UTF-8"?>
<model unit="centimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:m="http://schemas.microsoft.com/3dmanufacturing/material/2015/02">
<resources>
<m:colorgroup id="5"><m:color color="#FF0000FF"/><m:color color="#0000FFFF"/></m:colorgroup>
<object id="1" name="Würfel &amp; Co" type="model" pid="5" pindex="0">${meshXml(a).replace(/<triangle v1="(\d+)" v2="(\d+)" v3="(\d+)"\/>/g, (m, x, y, z) => (Number(x) < 18 ? m : `<triangle v1="${x}" v2="${y}" v3="${z}" pid="5" p1="1"/>`))}</object>
</resources>
<build><item objectid="1"/></build></model>`;
    const parts = read3MF(zipSync({ '_rels/.rels': strToU8(RELS), '3D/3dmodel.model': strToU8(model) }));
    expect(parts).toHaveLength(2);
    expect(parts.map((p) => p.color)).toEqual(['#ff0000', '#0000ff']);
    expect(parts[0].name).toBe('Würfel & Co – Farbe 1');
    expect(parts[0].positions.length + parts[1].positions.length).toBe(a.length);
    const bb = bbox(new Float32Array([...parts[0].positions, ...parts[1].positions]));
    expect(bb.max[2] - bb.min[2]).toBeCloseTo(10, 4); // 1 cm = 10 mm
  });

  it('wirft eine verständliche Meldung bei Dateien ohne Modell', () => {
    expect(() => read3MF(zipSync({ 'foo.txt': strToU8('x') }))).toThrow('kein 3D-Modell');
  });
});
