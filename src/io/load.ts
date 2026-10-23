/**
 * Lädt beliebige gängige 3D-Formate. Mehrteilige Dateien (3MF mit mehreren Objekten, GLB/FBX/OBJ mit
 * mehreren Netzen) werden auf Wunsch in einzelne Teile mit ihrer Farbe zerlegt; die Lage der Teile
 * zueinander bleibt dabei erhalten. Übernommen werden nur Positionen – Hierarchien und
 * Transformationen der Datei werden eingerechnet.
 */
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader } from 'three/addons/loaders/MTLLoader.js';
import { PLYLoader } from 'three/addons/loaders/PLYLoader.js';
import { AMFLoader } from 'three/addons/loaders/AMFLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { FBXLoader } from 'three/addons/loaders/FBXLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { VRMLLoader } from 'three/addons/loaders/VRMLLoader.js';
import { TDSLoader } from 'three/addons/loaders/TDSLoader.js';
import { VTKLoader } from 'three/addons/loaders/VTKLoader.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { read3MF } from './threemf';

export const SUPPORTED = ['stl', 'obj', 'mtl', 'ply', '3mf', 'amf', 'glb', 'gltf', 'fbx', 'dae', 'wrl', '3ds', 'vtk'] as const;

export interface LoadedPart {
  name: string;
  geom: THREE.BufferGeometry;
  /** '#rrggbb', falls die Datei eine Farbe angibt. */
  color?: string;
  /** Teile desselben Objekts (gleiche Gruppe) bewegen sich gemeinsam. */
  group?: string;
}

export interface LoadOptions {
  /** Mehrteilige Dateien in einzelne Teile zerlegen. */
  split: boolean;
  /** Inhalt einer passenden .mtl-Datei (Farben für OBJ). */
  mtl?: string;
}

/** Mehr Netze als hier werden nicht mehr einzeln geladen (z. B. ein GLB aus tausenden Einzelteilen). */
const MAX_SPLIT = 1000;

const text = (buf: ArrayBuffer) => new TextDecoder().decode(buf);

async function parse(ext: string, buf: ArrayBuffer, mtl?: string): Promise<THREE.Object3D | THREE.BufferGeometry> {
  switch (ext) {
    case 'stl':
      return new STLLoader().parse(buf);
    case 'ply':
      return new PLYLoader().parse(buf);
    case 'obj': {
      const loader = new OBJLoader();
      if (mtl) {
        const materials = new MTLLoader().parse(mtl, '');
        materials.preload();
        loader.setMaterials(materials);
      }
      return loader.parse(text(buf));
    }
    case 'amf':
      return new AMFLoader().parse(buf);
    case 'fbx':
      return new FBXLoader().parse(buf, '');
    case 'dae':
      return new ColladaLoader().parse(text(buf), '').scene;
    case 'wrl':
      return new VRMLLoader().parse(text(buf), '');
    case '3ds':
      return new TDSLoader().parse(buf, '');
    case 'vtk':
      return new VTKLoader().parse(buf, '');
    case 'glb':
    case 'gltf':
      return new Promise((resolve, reject) =>
        new GLTFLoader().parse(buf, '', (g) => resolve(g.scene), (e) => reject(e)),
      );
    default:
      throw new Error(`Dateiformat .${ext} wird (noch) nicht unterstützt.`);
  }
}

function toPositionOnly(g: THREE.BufferGeometry): THREE.BufferGeometry {
  const src = g.index ? g.toNonIndexed() : g;
  const out = new THREE.BufferGeometry();
  out.setAttribute('position', src.getAttribute('position').clone());
  return out;
}

function finish(geom: THREE.BufferGeometry): THREE.BufferGeometry {
  geom.computeVertexNormals();
  geom.computeBoundingBox();
  geom.computeBoundingSphere();
  return geom;
}

/** Mittlere Farbe von Eckpunktfarben (z. B. PLY oder eingefärbte GLB-Netze). */
function averageVertexColor(g: THREE.BufferGeometry): string | undefined {
  const c = g.getAttribute('color');
  if (!c || !c.count) return undefined;
  let r = 0, gr = 0, b = 0;
  for (let i = 0; i < c.count; i++) { r += c.getX(i); gr += c.getY(i); b += c.getZ(i); }
  return `#${new THREE.Color(r / c.count, gr / c.count, b / c.count).getHexString()}`;
}

/** Farbe eines Netzes aus seinem Material; reines Weiß gilt als „keine Angabe" (Vorgabe vieler Loader). */
function meshColor(mesh: THREE.Mesh): string | undefined {
  const vc = averageVertexColor(mesh.geometry);
  if (vc) return vc;
  const mat = (Array.isArray(mesh.material) ? mesh.material[0] : mesh.material) as THREE.MeshStandardMaterial | undefined;
  const hex = mat?.color?.getHexString();
  return hex && hex !== 'ffffff' ? `#${hex}` : undefined;
}

export async function loadModel(file: File, opts: LoadOptions = { split: true }): Promise<LoadedPart[]> {
  const ext = file.name.split('.').pop()?.toLowerCase() ?? '';
  const base = file.name.replace(/\.[^.]+$/, '');
  const buf = await file.arrayBuffer();

  if (ext === '3mf') {
    const parts = read3MF(new Uint8Array(buf));
    const list = opts.split && parts.length <= MAX_SPLIT ? parts : [{ ...parts[0], positions: concat(parts.map((p) => p.positions)) }];
    return list.map((p) => {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(p.positions, 3));
      return { name: list.length > 1 ? p.name : base, geom: finish(g), color: p.color, group: list.length > 1 ? `${file.name}:${p.group}` : undefined };
    });
  }

  const res = await parse(ext, buf, opts.mtl);
  if (res instanceof THREE.BufferGeometry) {
    return [{ name: base, geom: finish(toPositionOnly(res)), color: averageVertexColor(res) }];
  }
  res.updateMatrixWorld(true);
  const meshes: { name: string; geom: THREE.BufferGeometry; color?: string; top: THREE.Object3D }[] = [];
  res.traverse((o) => {
    const mesh = o as THREE.Mesh;
    if (!mesh.isMesh || !mesh.geometry?.getAttribute('position')) return;
    const g = toPositionOnly(mesh.geometry);
    g.applyMatrix4(mesh.matrixWorld);
    // Oberstes Objekt unterhalb der Wurzel: Netze darunter gehören zu einem Objekt und bleiben beisammen
    let top: THREE.Object3D = mesh;
    while (top.parent && top.parent !== res) top = top.parent;
    meshes.push({ name: mesh.name || mesh.parent?.name || base, geom: g, color: meshColor(mesh), top });
  });
  if (meshes.length === 0) throw new Error('Die Datei enthält keine Dreiecksnetze.');
  if (meshes.length === 1 || !opts.split || meshes.length > MAX_SPLIT) {
    const geom = meshes.length === 1 ? meshes[0].geom : mergeGeometries(meshes.map((m) => m.geom))!;
    return [{ name: base, geom: finish(geom), color: meshes[0].color }];
  }
  const seen = new Map<string, number>();
  const tops = [...new Set(meshes.map((m) => m.top))];
  const perTop = new Map(tops.map((t) => [t, meshes.filter((m) => m.top === t).length]));
  return meshes.map((m) => {
    const k = (seen.get(m.name) ?? 0) + 1;
    seen.set(m.name, k);
    const group = perTop.get(m.top)! > 1 ? `${file.name}:${tops.indexOf(m.top)}` : undefined;
    return { name: k > 1 ? `${m.name} (${k})` : m.name, geom: finish(m.geom), color: m.color, group };
  });
}

function concat(list: Float32Array[]): Float32Array {
  const out = new Float32Array(list.reduce((s, p) => s + p.length, 0));
  let o = 0;
  for (const p of list) { out.set(p, o); o += p.length; }
  return out;
}
