/**
 * Darstellung der Slicer-Zusätze eines Teils (Bambu Studio / OrcaSlicer / PrusaSlicer):
 *
 * - Farbbemalung: Teildreiecke in der Filamentfarbe, knapp vor der Oberfläche.
 * - Modifier mit eigenem Filament (z. B. eingelassene Aufschriften): Die Oberfläche des Teils wird
 *   dort in der Filamentfarbe gezeigt, wo sie im Modifier liegt – so wie sie gedruckt aussieht. Dazu
 *   zählt der Stencil-Puffer, wie viele Modifier-Flächen vor der sichtbaren Oberfläche liegen
 *   (ungerade = innen); danach wird das Teil an diesen Stellen in der Filamentfarbe neu gezeichnet.
 * - Übrige Modifier, negative Volumen, Stützen-Volumen: durchscheinend wie im Slicer.
 *
 * Alles hängt als Kind am Netz des Teils, bewegt sich also mit und wird beim Picken übergangen.
 */
import * as THREE from 'three';
import { decodePaint, paintLeaves } from './io/paint';
import type { Modifier, SlicerExtras } from './io/threemf';

const SURFACE = { roughness: 0.65, metalness: 0.05 };
const GHOST: Record<string, number> = { modifier_part: 0xb5d334, negative_part: 0x9e9e9e, support_enforcer: 0x37b24d, support_blocker: 0xe03131 };

/** Jeder farbige Modifier braucht drei Zeichenschritte direkt hintereinander. */
let nextOrder = 10;

const noPick = (o: THREE.Object3D) => {
  o.raycast = () => {};
  o.userData.decor = true;
  return o;
};

function geometryOf(positions: Float32Array): THREE.BufferGeometry {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  g.computeVertexNormals();
  g.computeBoundingSphere();
  return g;
}

/** Teildreiecke der Farbbemalung (Zustand 0 = Farbe des Teils, wird nicht eigens gezeichnet). */
export function paintOverlay(positions: Float32Array, codes: string[], palette: string[]): THREE.Mesh | null {
  const pos: number[] = [];
  const col: number[] = [];
  const c = new THREE.Color();
  const v = (t: number, k: number): [number, number, number] => [positions[t * 9 + k * 3], positions[t * 9 + k * 3 + 1], positions[t * 9 + k * 3 + 2]];
  for (let t = 0; t < codes.length; t++) {
    if (!codes[t]) continue;
    const root = decodePaint(codes[t]);
    if (!root) continue;
    paintLeaves(root, v(t, 0), v(t, 1), v(t, 2), (state, a, b, d) => {
      if (!state) return;
      c.set(palette[state - 1] ?? '#ff00ff');
      pos.push(...a, ...b, ...d);
      for (let k = 0; k < 3; k++) col.push(c.r, c.g, c.b);
    });
  }
  if (!pos.length) return null;
  const g = geometryOf(new Float32Array(pos));
  g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  const mesh = new THREE.Mesh(g, new THREE.MeshStandardMaterial({
    ...SURFACE, vertexColors: true, side: THREE.DoubleSide, polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -2,
  }));
  return noPick(mesh) as THREE.Mesh;
}

/** Modifier mit eigenem Filament: Oberfläche des Teils innerhalb des Modifiers einfärben. */
function colorModifier(partGeom: THREE.BufferGeometry, m: Modifier): THREE.Object3D[] {
  const g = geometryOf(m.positions);
  const order = nextOrder;
  nextOrder += 3;
  // 1. Modifier-Flächen vor der Oberfläche zählen (Bit 1 umschalten). Um einen festen Betrag nach vorn
  //    versetzt, damit bündig aufliegende Aufschriften als „innen" gelten – nicht abhängig von der
  //    Neigung, sonst kämen schräg gesehene Seitenwände durch die Oberfläche.
  const count = new THREE.Mesh(g, new THREE.MeshBasicMaterial({
    colorWrite: false, depthWrite: false, depthFunc: THREE.LessDepth, side: THREE.DoubleSide,
    polygonOffset: true, polygonOffsetFactor: 0, polygonOffsetUnits: -8,
    stencilWrite: true, stencilFunc: THREE.AlwaysStencilFunc, stencilRef: 1, stencilWriteMask: 1, stencilZPass: THREE.InvertStencilOp,
  }));
  // 2. Das Teil selbst in der Filamentfarbe, nur wo das Bit gesetzt ist (und es dort zurücksetzen)
  const tint = new THREE.Mesh(partGeom, new THREE.MeshStandardMaterial({
    ...SURFACE, color: m.color, side: THREE.DoubleSide, depthFunc: THREE.LessEqualDepth,
    polygonOffset: true, polygonOffsetFactor: 0, polygonOffsetUnits: -1,
    stencilWrite: true, stencilFunc: THREE.EqualStencilFunc, stencilRef: 1, stencilFuncMask: 1, stencilWriteMask: 1, stencilZPass: THREE.ZeroStencilOp,
  }));
  // 3. Übrige gesetzte Bits löschen (etwa wo ein anderes Teil davor liegt)
  const clear = new THREE.Mesh(g, new THREE.MeshBasicMaterial({
    colorWrite: false, depthWrite: false, depthTest: false, side: THREE.DoubleSide,
    stencilWrite: true, stencilFunc: THREE.AlwaysStencilFunc, stencilWriteMask: 1, stencilZPass: THREE.ZeroStencilOp,
  }));
  count.renderOrder = order;
  tint.renderOrder = order + 1;
  clear.renderOrder = order + 2;
  return [count, tint, clear].map(noPick);
}

function ghostModifier(m: Modifier): THREE.Object3D {
  return noPick(new THREE.Mesh(geometryOf(m.positions), new THREE.MeshStandardMaterial({
    color: GHOST[m.subtype] ?? GHOST.modifier_part, transparent: true, opacity: 0.22, depthWrite: false, side: THREE.DoubleSide,
  })));
}

/** Farbbemalung als Kind des Netzes (markiert, damit sie sich einzeln erneuern lässt). */
function addPaintOverlay(mesh: THREE.Mesh, extras: SlicerExtras) {
  const codes = extras.paint?.paint_color;
  if (!codes) return;
  const overlay = paintOverlay(mesh.geometry.getAttribute('position').array as Float32Array, codes, extras.palette ?? []);
  if (!overlay) return;
  overlay.userData.paintOverlay = true;
  mesh.add(overlay);
}

/** Farbbemalung neu zeichnen (nach dem Malen oder Rückgängig). */
export function refreshPaint(mesh: THREE.Mesh, extras: SlicerExtras) {
  for (const o of [...mesh.children]) {
    if (!o.userData.paintOverlay) continue;
    mesh.remove(o);
    (o as THREE.Mesh).geometry.dispose();
    ((o as THREE.Mesh).material as THREE.Material).dispose();
  }
  addPaintOverlay(mesh, extras);
}

/** Hängt die Darstellung der Zusätze an das Netz des Teils. */
export function decorate(mesh: THREE.Mesh, extras: SlicerExtras) {
  addPaintOverlay(mesh, extras);
  for (const m of extras.modifiers ?? []) {
    if (m.subtype === 'modifier_part' && m.color) mesh.add(...colorModifier(mesh.geometry, m));
    else mesh.add(ghostModifier(m));
  }
}

/** Gibt die Darstellung wieder frei (die Geometrie des Teils selbst bleibt). */
export function disposeDecor(mesh: THREE.Mesh) {
  for (const o of mesh.children) {
    if (!o.userData.decor) continue;
    const m = o as THREE.Mesh;
    if (m.geometry !== mesh.geometry) m.geometry.dispose();
    (m.material as THREE.Material).dispose();
  }
}
