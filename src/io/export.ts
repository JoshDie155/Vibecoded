/** Export als STL (binär) und OBJ sowie Speichern im Browser. 3MF: siehe threemf.ts. */
import * as THREE from 'three';
import { zipSync } from 'fflate';
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
  const header = 'D3DForge – zusammengefuegtes Modell';
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
  const lines = ['# D3DForge – zusammengefuegtes Modell'];
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

interface ViewerDownloads {
  save(req: { filename: string; data: Uint8Array }): Promise<unknown>;
}

/**
 * Als veröffentlichtes claude.ai-Artefakt sind direkte Downloads gesperrt; dort bietet der Viewer
 * eine Speichern-Funktion an, die aber nur bestimmte Endungen erlaubt – daher als ZIP.
 */
const viewerDownloads: Promise<ViewerDownloads | null> = (async () => {
  const claude = (window as unknown as { claude?: { use(name: string): Promise<unknown> } }).claude;
  if (!claude?.use) return null;
  try {
    return (await claude.use('downloads')) as ViewerDownloads | null;
  } catch {
    return null;
  }
})();

/** Speichert die Datei. Wirft, wenn das Speichern fehlschlägt (außer der Nutzer bricht ab). */
export async function download(blob: Blob, name: string) {
  const viewer = await viewerDownloads;
  if (viewer) {
    const zip = zipSync({ [name]: new Uint8Array(await blob.arrayBuffer()) });
    try {
      await viewer.save({ filename: `${name}.zip`, data: zip });
    } catch (e) {
      if ((e as { code?: string }).code !== 'declined') throw e;
    }
    return;
  }
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  // Hinter einem offenen Dialog ist die Seite „inert“ – dort würde der Dateiname nicht übernommen
  (document.querySelector('dialog[open]') ?? document.body).appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
}
