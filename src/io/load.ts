/**
 * Lädt beliebige gängige 3D-Formate und macht daraus eine einzige Dreiecks-Geometrie
 * (nur Positionen, Hierarchien und Transformationen der Datei werden eingerechnet).
 */
import * as THREE from 'three';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { PLYLoader } from 'three/addons/loaders/PLYLoader.js';
import { ThreeMFLoader } from 'three/addons/loaders/3MFLoader.js';
import { AMFLoader } from 'three/addons/loaders/AMFLoader.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { FBXLoader } from 'three/addons/loaders/FBXLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { VRMLLoader } from 'three/addons/loaders/VRMLLoader.js';
import { TDSLoader } from 'three/addons/loaders/TDSLoader.js';
import { VTKLoader } from 'three/addons/loaders/VTKLoader.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

export const SUPPORTED = ['stl', 'obj', 'ply', '3mf', 'amf', 'glb', 'gltf', 'fbx', 'dae', 'wrl', '3ds', 'vtk'] as const;

const text = (buf: ArrayBuffer) => new TextDecoder().decode(buf);

async function parse(ext: string, buf: ArrayBuffer): Promise<THREE.Object3D | THREE.BufferGeometry> {
  switch (ext) {
    case 'stl':
      return new STLLoader().parse(buf);
    case 'ply':
      return new PLYLoader().parse(buf);
    case 'obj':
      return new OBJLoader().parse(text(buf));
    case '3mf':
      return new ThreeMFLoader().parse(buf);
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

export async function loadModel(file: File): Promise<THREE.BufferGeometry> {
  const ext = file.name.split('.').pop()?.toLowerCase() ?? '';
  const buf = await file.arrayBuffer();
  const res = await parse(ext, buf);

  let geom: THREE.BufferGeometry;
  if (res instanceof THREE.BufferGeometry) {
    geom = toPositionOnly(res);
  } else {
    res.updateMatrixWorld(true);
    const parts: THREE.BufferGeometry[] = [];
    res.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (!mesh.isMesh || !mesh.geometry?.getAttribute('position')) return;
      const g = toPositionOnly(mesh.geometry);
      g.applyMatrix4(mesh.matrixWorld);
      parts.push(g);
    });
    if (parts.length === 0) throw new Error('Die Datei enthält keine Dreiecksnetze.');
    geom = parts.length === 1 ? parts[0] : mergeGeometries(parts)!;
  }
  geom.computeVertexNormals();
  geom.computeBoundingBox();
  geom.computeBoundingSphere();
  return geom;
}
