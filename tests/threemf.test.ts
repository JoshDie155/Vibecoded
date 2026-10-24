import { describe, expect, it } from 'vitest';
import { strFromU8, strToU8, unzipSync, zipSync } from 'fflate';
import { colorSlots, read3MF, write3MF, write3MFBambu } from '../src/io/threemf';
import { paintStates as paintStatesOf } from '../src/io/paint';
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
    // Farben kommen über die Filamente der Projekteinstellungen zurück
    expect(back.map((p) => p.color)).toEqual(['#ff0000', '#00aa00', '#ff0000']);
  });

  it('legt gültige Projekteinstellungen bei: System-Profile, Filamentfarben, gleich lange Filament-Listen', () => {
    const files = unzipSync(write3MFBambu(parts));
    const json = JSON.parse(strFromU8(files['Metadata/project_settings.config']));
    // Bambu Studio 2.8.2 verwirft Projekte ohne Bambu-Drucker oder ohne nozzle_diameter
    expect(json.printer_model).toBe('Bambu Lab X1 Carbon');
    expect(json.nozzle_diameter).toEqual(['0.4']);
    expect(json.printer_settings_id).toBe('Bambu Lab X1 Carbon 0.4 nozzle');
    expect(json.print_settings_id).toBe('0.20mm Standard @BBL X1C');
    expect(json.filament_colour).toEqual(['#FF0000', '#00AA00']);
    for (const key of ['filament_settings_id', 'filament_multi_colour', 'filament_colour_type', 'filament_type', 'filament_ids',
      'filament_diameter', 'filament_extruder_variant', 'filament_self_index', 'filament_map'])
      expect(json[key], key).toHaveLength(2);
    expect(json.filament_self_index).toEqual(['1', '2']);
    expect(json.inherits_group).toHaveLength(4);
    expect(json.different_settings_to_system).toHaveLength(4);
    // Alle Werte als Zeichenketten bzw. Listen von Zeichenketten (Format von Bambu Studio)
    for (const [k, v] of Object.entries(json)) {
      if (Array.isArray(v)) v.forEach((x) => expect(typeof x, k).toBe('string'));
      else expect(typeof v, k).toBe('string');
    }
  });

  it('nimmt die Profile und die Plattenmitte des gewählten Druckers', () => {
    const data = write3MFBambu(parts, { printer: 'A1M' });
    const json = JSON.parse(strFromU8(unzipSync(data)['Metadata/project_settings.config']));
    expect(json.printer_settings_id).toBe('Bambu Lab A1 mini 0.4 nozzle');
    expect(json.filament_settings_id).toEqual(['Bambu PLA Basic @BBL A1M', 'Bambu PLA Basic @BBL A1M']);
    const back = read3MF(data);
    const bb = bbox(new Float32Array([...back[0].positions, ...back[1].positions, ...back[2].positions]));
    expect((bb.min[0] + bb.max[0]) / 2).toBeCloseTo(90, 4);
    expect((bb.min[1] + bb.max[1]) / 2).toBeCloseTo(90, 4);
  });

  it('Drucker mit zwei Düsen: Düsen-Angaben je Extruder, Filamente verteilt Bambu Studio automatisch', () => {
    const json = JSON.parse(strFromU8(unzipSync(write3MFBambu(parts, { printer: 'H2C' }))['Metadata/project_settings.config']));
    expect(json.printer_model).toBe('Bambu Lab H2C');
    expect(json.printer_settings_id).toBe('Bambu Lab H2C 0.4 nozzle');
    expect(json.nozzle_diameter).toEqual(['0.4', '0.4']);
    expect(json.extruder_type).toEqual(['Direct Drive', 'Direct Drive']);
    expect(json.nozzle_volume_type).toEqual(['Standard', 'Standard']);
    expect(json.filament_map_mode).toBe('Auto For Flush');
    expect(json.filament_settings_id).toEqual(['Bambu PLA Basic @BBL H2C', 'Bambu PLA Basic @BBL H2C']);
    // Einzeldüse: keine Extruder-Angaben
    const single = JSON.parse(strFromU8(unzipSync(write3MFBambu(parts, { printer: 'P2S' }))['Metadata/project_settings.config']));
    expect(single.nozzle_diameter).toEqual(['0.4']);
    expect(single.extruder_type).toBeUndefined();
  });

  it('erkennt beim Lesen den Drucker eines Bambu-Projekts', () => {
    const info: { printer?: string } = {};
    read3MF(write3MFBambu(parts, { printer: 'P1S' }), info);
    expect(info.printer).toBe('P1S');
    const none: { printer?: string } = {};
    read3MF(write3MF(parts, 'multicolor'), none);
    expect(none.printer).toBeUndefined();
  });
});

describe('Slicer-Zusätze: Bemalung, Modifier, Einstellungen', () => {
  // Bambu-Projekt: Schild mit eingelassener Aufschrift (Modifier, Filament 2) und bemalten Dreiecken
  const plate = box(40, 10, 2);
  const text = transformPositions(box(20, 4, 0.5), new THREE.Matrix4().makeTranslation(0, 0, -0.9));
  const paintCount = plate.length / 9;
  // Dreieck 0 in Filament 3 ('0C'), Dreieck 1 zur Hälfte in Filament 2 ('801': eine Seite geteilt, Kinder rückwärts 0 und 8)
  const paintAttr = (t: number) => (t === 0 ? ' paint_color="0C"' : t === 1 ? ' paint_color="801"' : t === 2 ? ' paint_fuzzy_skin="4"' : '');
  const meshWithPaint = (p: Float32Array) => {
    let v = '';
    let t = '';
    for (let i = 0; i < p.length / 3; i++) v += `<vertex x="${p[i * 3]}" y="${p[i * 3 + 1]}" z="${p[i * 3 + 2]}"/>`;
    for (let i = 0; i < p.length / 9; i++) t += `<triangle v1="${i * 3}" v2="${i * 3 + 1}" v3="${i * 3 + 2}"${paintAttr(i)}/>`;
    return `<mesh><vertices>${v}</vertices><triangles>${t}</triangles></mesh>`;
  };
  const project = () => zipSync({
    '_rels/.rels': strToU8(RELS),
    '3D/3dmodel.model': strToU8(`<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06">
<resources><object id="3" type="model"><components>
<component p:path="/3D/Objects/object_1.model" objectid="1" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>
<component p:path="/3D/Objects/object_1.model" objectid="2" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>
</components></object></resources>
<build><item objectid="3" transform="1 0 0 0 1 0 0 0 1 100 50 1" printable="1"/></build></model>`),
    '3D/Objects/object_1.model': strToU8(`<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
<resources><object id="1" type="model">${meshWithPaint(plate)}</object><object id="2" type="model">${meshXml(text)}</object></resources><build/></model>`),
    'Metadata/model_settings.config': strToU8(`<?xml version="1.0" encoding="UTF-8"?>
<config>
  <object id="3">
    <metadata key="name" value="Schild"/>
    <metadata key="extruder" value="1"/>
    <metadata key="layer_height" value="0.28"/>
    <metadata key="sparse_infill_density" value="20%"/>
    <part id="1" subtype="normal_part"><metadata key="name" value="Schild.stl"/><metadata key="wall_loops" value="4"/></part>
    <part id="2" subtype="modifier_part"><metadata key="name" value="text_shape"/><metadata key="extruder" value="2"/><metadata key="bottom_surface_pattern" value="concentric"/></part>
  </object>
</config>`),
    'Metadata/project_settings.config': strToU8(JSON.stringify({ filament_colour: ['#0000FF', '#FFFFFF', '#FF0000'] })),
  });

  it('liest Modifier als Zusatz des Teils statt als eigenes Teil, mit Filamentfarbe und Einstellungen', () => {
    const parts = read3MF(project());
    expect(parts.map((p) => p.name)).toEqual(['Schild']);
    const [p] = parts;
    expect(p.color).toBe('#0000ff');
    expect(p.modifiers).toHaveLength(1);
    const m = p.modifiers![0];
    expect([m.name, m.subtype, m.color]).toEqual(['text_shape', 'modifier_part', '#ffffff']);
    expect(m.settings).toEqual({ bottom_surface_pattern: 'concentric' });
    // Lage wie das Teil (Druckbett-Position eingerechnet)
    expect(bbox(m.positions).min[0]).toBeCloseTo(90, 4);
    expect(bbox(m.positions).min[2]).toBeCloseTo(1 - 0.9, 4);
    // Einstellungen: Objekt-Einstellungen, die je Teil gelten können, plus die des Teils – die Schichthöhe
    // gilt nur je Objekt und steht extra
    expect(p.settings).toEqual({ sparse_infill_density: '20%', wall_loops: '4' });
    expect(p.objectSettings).toEqual({ layer_height: '0.28' });
  });

  it('liest die Bemalung je Dreieck samt Filamentfarben', () => {
    const [p] = read3MF(project());
    expect(p.paint!.paint_color.slice(0, 3)).toEqual(['0C', '801', '']);
    expect(p.paint!.paint_color).toHaveLength(paintCount);
    expect(p.paint!.paint_fuzzy_skin[2]).toBe('4');
    expect(p.palette).toEqual(['#0000ff', '#ffffff', '#ff0000']);
  });

  it('Bambu-Export schreibt Modifier, Bemalung und Einstellungen mit den neuen Filamentnummern', () => {
    const [p] = read3MF(project());
    // Ein anderes Teil zuerst: Rot wird Filament 1, Blau 2, Weiß 3
    const other = { name: 'Rot', positions: transformPositions(box(5, 5, 5), new THREE.Matrix4().makeTranslation(0, 30, 0)), color: '#ff0000' };
    const files = unzipSync(write3MFBambu([other, { ...p, color: p.color! }], { printer: 'P2S' }));
    const cfg = strFromU8(files['Metadata/model_settings.config']);
    expect(cfg).toMatch(/<part id="3" subtype="modifier_part">\s*<metadata key="name" value="text_shape"\/>\s*<metadata key="matrix"[^>]*\/>\s*<metadata key="extruder" value="3"\/>\s*<metadata key="bottom_surface_pattern" value="concentric"\/>/);
    expect(cfg).toContain('<metadata key="wall_loops" value="4"/>');
    expect(cfg).not.toContain('layer_height');
    const objects = strFromU8(files['3D/Objects/object_1.model']);
    // Filament 3 (Rot) heißt jetzt Filament 1: Zustand 1 = '4'
    expect(objects).toContain('paint_color="4"');
    expect(objects).toContain('paint_fuzzy_skin="4"');
    const colors = JSON.parse(strFromU8(files['Metadata/project_settings.config'])).filament_colour;
    expect(colors).toEqual(['#FF0000', '#0000FF', '#FFFFFF']);
    // Und zurück: gleiche Farben an denselben Dreiecken
    const back = read3MF(write3MFBambu([other, { ...p, color: p.color! }], { printer: 'P2S' }));
    const plateBack = back.find((q) => q.name === 'Schild')!;
    const colorAt = (q: typeof p, t: number) => {
      const code = q.paint!.paint_color[t];
      return code ? [...paintStatesOf(code)].map((s) => (s ? q.palette![s - 1] : q.color)) : [];
    };
    for (const t of [0, 1, 2]) expect(colorAt(plateBack, t)).toEqual(colorAt(p, t));
    expect(back.flatMap((q) => q.modifiers ?? []).map((m) => m.color)).toEqual(['#ffffff']);
  });

  it('entartete Dreiecke fallen beim Export weg, ohne die Bemalung zu verschieben', () => {
    const tri = (x: number) => [x, 0, 0, x + 1, 0, 0, x, 1, 0];
    const positions = new Float32Array([...tri(0), 5, 5, 5, 5, 5, 5, 6, 6, 6, ...tri(10)]);
    const part = { name: 'A', positions, color: '#00ff00', paint: { paint_color: ['', '8', '4'] }, palette: ['#ff0000', '#0000ff'] };
    const objects = strFromU8(unzipSync(write3MFBambu([part]))['3D/Objects/object_1.model']);
    const tris = [...objects.matchAll(/<triangle [^>]*\/>/g)].map((m) => m[0]);
    expect(tris).toHaveLength(2);
    expect(tris[0]).not.toContain('paint_color');
    // Filament 1 (Rot) → Filament 2 des Exports: Zustand 2 = '8'
    expect(tris[1]).toContain('paint_color="8"');
  });

  it('PrusaSlicer: Modifier-Bereiche werden Zusatz statt Teil, Bemalung als mmu_segmentation', () => {
    const a = box(10, 10, 10);
    const b = transformPositions(box(4, 4, 4), new THREE.Matrix4().makeTranslation(0, 0, 3));
    const all = new Float32Array(a.length + b.length);
    all.set(a);
    all.set(b, a.length);
    let v = '';
    let t = '';
    for (let i = 0; i < all.length / 3; i++) v += `<vertex x="${all[i * 3]}" y="${all[i * 3 + 1]}" z="${all[i * 3 + 2]}"/>`;
    for (let i = 0; i < all.length / 9; i++) t += `<triangle v1="${i * 3}" v2="${i * 3 + 1}" v3="${i * 3 + 2}"${i === 0 ? ' slic3rpe:mmu_segmentation="8"' : ''}/>`;
    const model = `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:slic3rpe="http://schemas.slic3r.org/3mf/2017/06">
<resources><object id="1" type="model"><mesh><vertices>${v}</vertices><triangles>${t}</triangles></mesh></object></resources>
<build><item objectid="1"/></build></model>`;
    const cfg = `<?xml version="1.0" encoding="UTF-8"?>
<config><object id="1" instances_count="1"><metadata type="object" key="name" value="Würfel"/>
<volume firstid="0" lastid="${a.length / 9 - 1}"><metadata type="volume" key="name" value="Würfel"/><metadata type="volume" key="extruder" value="1"/></volume>
<volume firstid="${a.length / 9}" lastid="${all.length / 9 - 1}"><metadata type="volume" key="name" value="Kern"/><metadata type="volume" key="volume_type" value="ParameterModifier"/><metadata type="volume" key="extruder" value="2"/></volume>
</object></config>`;
    const parts = read3MF(zipSync({
      '_rels/.rels': strToU8(RELS),
      '3D/3dmodel.model': strToU8(model),
      'Metadata/Slic3r_PE_model.config': strToU8(cfg),
      'Metadata/Slic3r_PE.config': strToU8('; extruder_colour = "#FF0000;#00FF00"\n'),
    }));
    expect(parts.map((p) => [p.name, p.positions.length])).toEqual([['Würfel', a.length]]);
    expect(parts[0].modifiers!.map((m) => [m.name, m.subtype, m.color, m.positions.length])).toEqual([['Kern', 'modifier_part', '#00ff00', b.length]]);
    expect(parts[0].paint!.paint_color[0]).toBe('8');
    // Farbiger 3MF-Export: Bemalung im PrusaSlicer-Format, Filament 2 (Grün) ist dort Filament 2
    const out = strFromU8(unzipSync(write3MF([{ ...parts[0], color: parts[0].color! }], 'multicolor'))['3D/3dmodel.model']);
    expect(out).toContain('slic3rpe:mmu_segmentation="8"');
  });
});

describe('write3MFBambu: ein Objekt je Baugruppe', () => {
  const at = (x: number, z = 0) => new THREE.Matrix4().makeTranslation(x, 0, z);
  const parts = [
    { name: 'Sitz', positions: transformPositions(box(20, 20, 30), at(0)), color: '#1f79e5', object: 'a', objectName: 'Sitz' },
    { name: 'Lehne', positions: transformPositions(box(20, 4, 20), at(0, 30)), color: '#1f79e5', object: 'a', objectName: 'Sitz' },
    {
      name: 'Schild', positions: transformPositions(box(30, 10, 2), at(60)), color: '#ff0000', object: 'b', objectName: 'Schild',
      objectSettings: { layer_height: '0.28', brim_type: 'no_brim' },
    },
  ];

  it('schreibt jede Baugruppe als eigenes Objekt mit eigener Unterdatei, Lage und Platteneintrag', () => {
    const files = unzipSync(write3MFBambu(parts, { printer: 'P2S' }));
    const main = strFromU8(files['3D/3dmodel.model']);
    expect(main.match(/<item /g)).toHaveLength(2);
    expect(files['3D/Objects/object_1.model']).toBeDefined();
    expect(files['3D/Objects/object_2.model']).toBeDefined();
    expect(strFromU8(files['3D/_rels/3dmodel.model.rels'])).toContain('/3D/Objects/object_2.model');
    const cfg = strFromU8(files['Metadata/model_settings.config']);
    expect([...cfg.matchAll(/<object id="(\d+)">\s*<metadata key="name" value="([^"]+)"/g)].map((m) => m[2])).toEqual(['Sitz', 'Schild']);
    expect(cfg.match(/<model_instance>/g)).toHaveLength(2);
    expect(cfg.match(/<assemble_item /g)).toHaveLength(2);
    // Eindeutige Ids über alle Objekte
    const ids = [...cfg.matchAll(/<(?:object|part) id="(\d+)"/g)].map((m) => m[1]);
    expect(new Set(ids).size).toBe(ids.length);
    // Objekt-Einstellungen gehören zum Schild-Objekt
    expect(cfg).toMatch(/value="Schild"\/>\s*<metadata key="extruder" value="2"\/>\s*<metadata key="layer_height" value="0.28"\/>\s*<metadata key="brim_type" value="no_brim"\/>/);
  });

  it('wird als zwei Objekte zurückgelesen, alles an seiner Stelle und auf der Platte', () => {
    const back = read3MF(write3MFBambu(parts, { printer: 'P2S' }));
    expect(back.map((p) => p.name)).toEqual(['Sitz', 'Lehne', 'Schild']);
    expect(back[0].group).toBe(back[1].group);
    expect(back[2].group).not.toBe(back[0].group);
    expect(back[2].objectSettings).toEqual({ layer_height: '0.28', brim_type: 'no_brim' });
    // Abstände wie vorher: Schild 60 mm neben dem Sitz, Lehne 30 mm über dem Boden
    const b = back.map((p) => bbox(p.positions));
    expect((b[2].min[0] + b[2].max[0]) / 2 - (b[0].min[0] + b[0].max[0]) / 2).toBeCloseTo(60, 4);
    expect(b[1].min[2] - b[0].min[2]).toBeCloseTo(30, 4);
    expect(Math.min(...b.map((x) => x.min[2]))).toBeCloseTo(0, 4);
  });

  it('Objekt-Einstellungen nur, wenn alle Teile des Objekts sie gleich mitbringen', () => {
    const mixed = [
      { ...parts[0], objectSettings: { layer_height: '0.2', brim_type: 'no_brim' } },
      { ...parts[1], objectSettings: { layer_height: '0.28', brim_type: 'no_brim' } },
    ];
    const cfg = strFromU8(unzipSync(write3MFBambu(mixed))['Metadata/model_settings.config']);
    expect(cfg).toContain('<metadata key="brim_type" value="no_brim"/>');
    expect(cfg).not.toContain('layer_height');
  });
});
