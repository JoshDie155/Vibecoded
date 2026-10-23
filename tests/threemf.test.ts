import { describe, expect, it } from 'vitest';
import { strFromU8, strToU8, unzipSync, zipSync } from 'fflate';
import { colorSlots, read3MF, write3MF, write3MFBambu } from '../src/io/threemf';
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

describe('read3MF: Bambu-Projekt mit mehreren Druckplatten', () => {
  /** Unterdatei wie von Bambu Studio: ein Objekt je Datei, bemalte Dreiecke (paint_color). */
  const subModel = (id: number, p: Float32Array) => `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p">
 <metadata name="BambuStudio:3mfVersion">1</metadata>
 <resources>
  <object id="${id}" p:UUID="0000000${id}-81cb-4c03-9d28-80fed5dfa1dc" type="model">
   ${meshXml(p, ' paint_color="4"')}
  </object>
 </resources>
 <build/>
</model>`;

  function project(collidingIds: boolean) {
    // Vier Objekte auf zwei Druckplatten (Platte 2 liegt in Bambu Studio rechts daneben)
    const shapes = [box(10, 10, 10), cylinder(3, 20, 24), box(30, 5, 2), cylinder(8, 4, 32)];
    const names = ['Kopf.stl', 'Arm links.stl', 'Flügel.stl', 'Sockel.stl'];
    const pos = [[128, 128, 5], [150, 100, 0], [435.6, 128, 1], [460, 160, 2]];
    const files: Record<string, Uint8Array> = { '_rels/.rels': strToU8(RELS) };
    let objects = '';
    let items = '';
    let settings = '';
    let rels = '';
    shapes.forEach((p, i) => {
      const sub = collidingIds ? 1 : i * 2 + 1;
      const outer = i * 2 + 2;
      files[`3D/Objects/object_${i + 1}.model`] = strToU8(subModel(sub, p));
      rels += `<Relationship Target="/3D/Objects/object_${i + 1}.model" Id="rel-${i + 1}" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>`;
      objects += `<object id="${outer}" p:UUID="0000000${i}-61cb-4c03-9d28-80fed5dfa1dc" type="model"><components><component p:path="/3D/Objects/object_${i + 1}.model" objectid="${sub}" p:UUID="0001000${i}-b206-40ff-9872-83e8017abed1" transform="1 0 0 0 1 0 0 0 1 0 0 0"/></components></object>`;
      items += `<item objectid="${outer}" p:UUID="0000000${i}-b1ec-4553-aec9-835e5b724bb4" transform="1 0 0 0 1 0 0 0 1 ${pos[i].join(' ')}" printable="1"/>`;
      settings += `<object id="${outer}"><metadata key="name" value="${names[i]}"/><metadata key="extruder" value="${(i % 3) + 1}"/>
<part id="${sub}" subtype="normal_part"><metadata key="name" value="${names[i]}"/><metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/><mesh_stat edges_fixed="0" degenerate_facets="0" facets_reversed="0" backwards_edges="0"/></part></object>`;
    });
    const plates = `<plate><metadata key="plater_id" value="1"/><model_instance><metadata key="object_id" value="2"/><metadata key="instance_id" value="0"/></model_instance><model_instance><metadata key="object_id" value="4"/><metadata key="instance_id" value="0"/></model_instance></plate>
<plate><metadata key="plater_id" value="2"/><model_instance><metadata key="object_id" value="6"/><metadata key="instance_id" value="0"/></model_instance><model_instance><metadata key="object_id" value="8"/><metadata key="instance_id" value="0"/></model_instance></plate>`;
    files['3D/3dmodel.model'] = strToU8(`<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p">
 <metadata name="Application">BambuStudio-02.02.00.85</metadata>
 <metadata name="BambuStudio:3mfVersion">1</metadata>
 <resources>${objects}</resources>
 <build p:UUID="2c7c17d8-22b5-4d84-8835-1976022ea369">${items}</build>
</model>`);
    files['3D/_rels/3dmodel.model.rels'] = strToU8(`<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">${rels}</Relationships>`);
    files['Metadata/model_settings.config'] = strToU8(`<?xml version="1.0" encoding="UTF-8"?><config>${settings}${plates}</config>`);
    files['Metadata/project_settings.config'] = strToU8(JSON.stringify({ filament_colour: ['#F2F2F2', '#1F6FEB', '#E5484D'] }));
    files['Metadata/plate_1.json'] = strToU8('{}');
    return { data: zipSync(files), shapes };
  }

  for (const colliding of [false, true]) {
    it(`erkennt jedes Objekt auf jeder Platte als eigenes Teil${colliding ? ' (gleiche Objekt-IDs in den Unterdateien)' : ''}`, () => {
      const { data, shapes } = project(colliding);
      const parts = read3MF(data);
      expect(parts.map((p) => p.name)).toEqual(['Kopf', 'Arm links', 'Flügel', 'Sockel']);
      expect(parts.map((p) => p.color)).toEqual(['#f2f2f2', '#1f6feb', '#e5484d', '#f2f2f2']);
      expect(new Set(parts.map((p) => p.group)).size).toBe(4);
      parts.forEach((p, i) => expect(p.positions.length).toBe(shapes[i].length));
      // Lage auf der zweiten Platte bleibt erhalten
      expect(bbox(parts[2].positions).min[0]).toBeCloseTo(420.6, 3);
    });
  }
});

describe('write3MFBambu', () => {
  const parts = [
    { name: 'Körper', positions: box(20, 20, 10), color: '#ff0000' },
    { name: 'Kopf & Hut', positions: transformPositions(cylinder(5, 6, 24), new THREE.Matrix4().makeTranslation(0, 0, 10)), color: '#00aa00' },
    { name: 'Fuß', positions: transformPositions(box(6, 6, 2), new THREE.Matrix4().makeTranslation(0, 0, -2)), color: '#FF0000' },
  ];

  it('schreibt das Projektformat von Bambu Studio: Kennung, Unterdatei, Filament je Teil', () => {
    const files = unzipSync(write3MFBambu(parts));
    const main = strFromU8(files['3D/3dmodel.model']);
    expect(main).toMatch(/<metadata name="Application">BambuStudio-\d{2}\.\d{2}\.\d{2}\.\d{2}<\/metadata>/);
    expect(main).toContain('<metadata name="BambuStudio:3mfVersion">1</metadata>');
    expect(main.match(/<component p:path="\/3D\/Objects\/object_1.model" objectid="\d"/g)).toHaveLength(3);
    expect(files['3D/_rels/3dmodel.model.rels']).toBeDefined();
    const cfg = strFromU8(files['Metadata/model_settings.config']);
    expect([...cfg.matchAll(/key="extruder" value="(\d+)"/g)].map((m) => Number(m[1]))).toEqual([1, 1, 2, 1]);
    expect(cfg).toContain('value="Kopf &amp; Hut"');
    expect(cfg).toContain('<plate>');
  });

  it('wird vom eigenen Leser als drei Teile mit Namen, Lage zueinander und Filament-Gruppen gelesen', () => {
    const back = read3MF(write3MFBambu(parts));
    expect(back.map((p) => p.name)).toEqual(['Körper', 'Kopf & Hut', 'Fuß']);
    expect(new Set(back.map((p) => p.group)).size).toBe(1);
    // Mittig auf der Platte, Unterseite auf z = 0, Abstände zueinander unverändert
    const all = new Float32Array([...back[0].positions, ...back[1].positions, ...back[2].positions]);
    const bb = bbox(all);
    expect((bb.min[0] + bb.max[0]) / 2).toBeCloseTo(128, 4);
    expect((bb.min[1] + bb.max[1]) / 2).toBeCloseTo(128, 4);
    expect(bb.min[2]).toBeCloseTo(0, 4);
    expect(bbox(back[1].positions).min[2] - bbox(back[0].positions).min[2]).toBeCloseTo(10, 4);
  });
});
