/** Export als STL (binär), OBJ und 3MF – wahlweise als ein Objekt oder positionierte Einzelteile. */
import * as THREE from 'three';
import { zipSync, strToU8 } from 'fflate';
import { mergeGeometries, mergeVertices } from 'three/addons/utils/BufferGeometryUtils.js';

export interface ExportPart {
  name: string;
  /** Geometrie bereits in Weltkoordinaten. */
  geometry: THREE.BufferGeometry;
}

function positionsOnly(g: THREE.BufferGeometry): THREE.BufferGeometry {
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', (g.index ? g.toNonIndexed() : g).getAttribute('position').clone());
  return out;
}

function merged(parts: ExportPart[]): THREE.BufferGeometry {
  return mergeGeometries(parts.map((p) => positionsOnly(p.geometry)))!;
}

export function toSTL(parts: ExportPart[]): Blob {
  const pos = merged(parts).getAttribute('position');
  const tris = pos.count / 3;
  const buf = new ArrayBuffer(84 + tris * 50);
  const dv = new DataView(buf);
  const header = 'PartJoin – zusammengefuegtes Modell';
  for (let i = 0; i < Math.min(80, header.length); i++) dv.setUint8(i, header.charCodeAt(i) & 0x7f);
  dv.setUint32(80, tris, true);
  const a = new THREE.Vector3();
  const b = new THREE.Vector3();
  const c = new THREE.Vector3();
  const n = new THREE.Vector3();
  let o = 84;
  for (let t = 0; t < tris; t++) {
    a.fromBufferAttribute(pos, t * 3);
    b.fromBufferAttribute(pos, t * 3 + 1);
    c.fromBufferAttribute(pos, t * 3 + 2);
    n.subVectors(c, b).cross(a.clone().sub(b)).normalize();
    for (const v of [n, a, b, c]) {
      dv.setFloat32(o, v.x, true);
      dv.setFloat32(o + 4, v.y, true);
      dv.setFloat32(o + 8, v.z, true);
      o += 12;
    }
    dv.setUint16(o, 0, true);
    o += 2;
  }
  return new Blob([buf], { type: 'model/stl' });
}

function indexed(g: THREE.BufferGeometry) {
  const m = mergeVertices(positionsOnly(g), 1e-5);
  return { pos: m.getAttribute('position'), idx: m.index! };
}

const f = (x: number) => +x.toFixed(5);

export function toOBJ(parts: ExportPart[]): Blob {
  const lines = ['# PartJoin – zusammengefuegtes Modell'];
  let offset = 1;
  for (const p of parts) {
    const { pos, idx } = indexed(p.geometry);
    lines.push(`o ${p.name.replace(/\s+/g, '_')}`);
    for (let i = 0; i < pos.count; i++) lines.push(`v ${f(pos.getX(i))} ${f(pos.getY(i))} ${f(pos.getZ(i))}`);
    for (let i = 0; i < idx.count; i += 3)
      lines.push(`f ${idx.getX(i) + offset} ${idx.getX(i + 1) + offset} ${idx.getX(i + 2) + offset}`);
    offset += pos.count;
  }
  return new Blob([lines.join('\n')], { type: 'text/plain' });
}

function meshXML(g: THREE.BufferGeometry): string {
  const { pos, idx } = indexed(g);
  const v: string[] = [];
  for (let i = 0; i < pos.count; i++) v.push(`<vertex x="${f(pos.getX(i))}" y="${f(pos.getY(i))}" z="${f(pos.getZ(i))}"/>`);
  const t: string[] = [];
  for (let i = 0; i < idx.count; i += 3) {
    const a = idx.getX(i);
    const b = idx.getX(i + 1);
    const c = idx.getX(i + 2);
    if (a === b || b === c || a === c) continue; // degenerierte Dreiecke weglassen
    t.push(`<triangle v1="${a}" v2="${b}" v3="${c}"/>`);
  }
  return `<mesh><vertices>${v.join('')}</vertices><triangles>${t.join('')}</triangles></mesh>`;
}

const esc = (s: string) => s.replace(/[<>&"]/g, (c) => ({ '<': '&lt;', '>': '&gt;', '&': '&amp;', '"': '&quot;' })[c]!);

/**
 * @param separate  true: jedes Teil als eigenes Objekt (positioniert, z. B. für Mehrfarbdruck),
 *                  false: alles als ein einziges Objekt.
 */
export function to3MF(parts: ExportPart[], separate: boolean): Blob {
  const objects = separate
    ? parts.map((p, i) => ({ id: i + 1, name: p.name, xml: meshXML(p.geometry) }))
    : [{ id: 1, name: 'PartJoin', xml: meshXML(merged(parts)) }];
  let resources = objects.map((o) => `<object id="${o.id}" name="${esc(o.name)}" type="model">${o.xml}</object>`).join('');
  let build = objects.map((o) => `<item objectid="${o.id}"/>`).join('');
  if (separate && objects.length > 1) {
    // Ein Sammelobjekt mit Komponenten → Slicer behandeln es als ein Objekt aus mehreren Teilen
    const id = objects.length + 1;
    resources += `<object id="${id}" name="PartJoin" type="model"><components>${objects
      .map((o) => `<component objectid="${o.id}"/>`)
      .join('')}</components></object>`;
    build = `<item objectid="${id}"/>`;
  }
  const model = `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="de-DE" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
<resources>${resources}</resources>
<build>${build}</build>
</model>`;
  const contentTypes = `<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>`;
  const rels = `<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>`;
  const zip = zipSync({
    '[Content_Types].xml': strToU8(contentTypes),
    '_rels/.rels': strToU8(rels),
    '3D/3dmodel.model': strToU8(model),
  });
  return new Blob([zip], { type: 'model/3mf' });
}

export function download(blob: Blob, name: string) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
}
