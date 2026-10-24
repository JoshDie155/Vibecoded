import './style.css';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { acceleratedRaycast, computeBoundsTree, disposeBoundsTree, MeshBVH } from 'three-mesh-bvh';
import { type Rigid, type Vec3, IDENTITY, compose, rotate, rotationAbout } from './math/align';
import { type JointResult, type Marker, type NormalMode, applyAdjustments, solveJoint } from './math/match';
import { type Sample, refineICP, sameShape, sampleSurface } from './math/icp';
import { type EdgeFeature, type Feature, type Topology, buildTopology, detectFeature, featureFromTris, mergeFeatures, transformFeature } from './math/features';
import { detectEdge } from './math/edges';
import { checkRange, finishCheck, newAccumulator, typicalGap } from './math/check';
import { findCandidateFaces } from './math/segment';
import { splitLoosePieces } from './math/components';
import { candidatePairs, placePlanePair } from './math/suggest';
import { contactScore } from './math/scan';
import type { FreeMotion } from './math/motion';
import { describeFeature, featureTag } from './math/mate';
import { fitFeatures } from './math/fit';
import { type LoadedPart, loadModel, SUPPORTED } from './io/load';
import { type ExportPart, download, toOBJ, toSTL } from './io/export';
import { BAMBU_PRINTERS, colorSlots, write3MF, write3MFBambu } from './io/threemf';
import { unionParts } from './io/union';
import manifoldWasm from 'manifold-3d/manifold.wasm?url';
import { makeDemo, makeRoundDemo } from './demo';

THREE.BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
THREE.BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;
THREE.Mesh.prototype.raycast = acceleratedRaycast;
// 3D-Druck-Konvention: Z zeigt nach oben
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

// ---------------------------------------------------------------- Zustand

interface Part {
  id: number;
  name: string;
  geom: THREE.BufferGeometry;
  mesh: THREE.Mesh;
  color: THREE.Color;
  /** Ausgangslage nach dem Laden (nebeneinander aufgereiht oder Lage aus der Datei). */
  layout: THREE.Matrix4;
  visible: boolean;
  /** Oberflächenpunkte (lokal) für Feinausrichtung, Symmetrieprüfung und automatisches Einpassen. */
  samples: Sample[];
  /** Nachbarschaften der Dreiecke für die Flächenerkennung (wird erst bei Bedarf berechnet). */
  topo?: Topology;
  /** Teile derselben Gruppe (z. B. ein Objekt aus einer 3MF-Datei) bewegen sich gemeinsam. */
  group?: string;
  /** Markante Flächen für automatische Vorschläge (wird erst bei Bedarf berechnet). */
  candidates?: Feature[];
}

/** Vorschlag für eine Verbindung: diese Fläche am einen Teil passt zu jener am anderen. */
interface Proposal {
  a: Part;
  b: Part;
  fa: Feature;
  fb: Feature;
  score: number;
}

type Side = 'a' | 'b';

interface FeatureCache {
  key: string;
  /** Lage des beweglichen Teils im Koordinatensystem des festen Teils. */
  rel: Rigid;
  /** Freie Bewegungen im Koordinatensystem des festen Teils. */
  free: FreeMotion[];
  count: number;
  distError: number;
  angleError: number;
  info: string;
  flatRotation: boolean;
  notes: string[];
  summary: string;
  perm?: number[];
  turn?: { point: Vec3; axis: Vec3; angles: number[] };
}

/**
 * Fläche einer Verbindung; `on`: Teil, auf dem sie liegt, falls nicht das Teil der Seite selbst.
 * `pieces`: zusammengefasste Fläche aus diesen Einzelflächen (Geometrie dann im Teil der Seite).
 */
type PickedFeature = Feature & { on?: number; pieces?: PickedFeature[] };

interface Joint {
  id: number;
  color: THREE.Color;
  a: number | null;
  b: number | null;
  ma: Marker[];
  mb: Marker[];
  /**
   * Ausgewählte Flächen (lokale Koordinaten des Teils, auf dem sie liegen). Eine Fläche darf auch auf
   * einem Nachbarteil liegen, das schon fest mit dem Teil dieser Seite verbunden ist (`on`).
   */
  fa: PickedFeature[];
  fb: PickedFeature[];
  ordered: boolean;
  normals: NormalMode;
  overlap: number;
  spin: number;
  /** Freie Drehung/Verschiebung automatisch über Flächenkontakt bestimmen. */
  seat: boolean;
  applied: boolean;
  alt: number;
  /**
   * Korrektur von Hand (Gizmo) oder aus der Feinausrichtung, im Koordinatensystem des festen Teils –
   * so bleibt sie richtig, auch wenn das feste Teil selbst noch bewegt wird.
   */
  adjust?: Rigid;
  adjustInfo?: string;
  // Ergebnis der letzten Berechnung
  result?: JointResult;
  cache?: FeatureCache;
  /** Zuordnung der Punkte: perm[i] = Index der Markierung am festen Teil für Markierung i am beweglichen. */
  perm?: number[];
  /** Freie Bewegungen in Weltkoordinaten (für das Gizmo). */
  free?: FreeMotion[];
  fixedSide?: Side;
  status?: { kind: 'ok' | 'warn' | 'err' | 'info'; text: string };
  /** Automatische Vorschläge für diese Verbindung und der gerade gewählte. */
  proposals?: Proposal[];
  proposalIdx?: number;
  /**
   * Zuletzt berechnete Lage (bewegliches Teil im Koordinatensystem des festen Teils). Solange eine
   * weitere Fläche/Markierung noch ihr Gegenstück sucht, bleibt das Teil dort, statt zurückzuspringen.
   */
  last?: { side: Side; rel: Rigid };
  /** Gewählte der gleich gut passenden Drehungen (0 = automatisch bestimmte). */
  turnIdx?: number;
  /** Teile nur vorläufig automatisch eingetragen: der erste Klick bestimmt Teil 1, der zweite Teil 2. */
  auto?: boolean;
}

const PART_COLORS = ['#8fa7c9', '#d9a066', '#88b98a', '#c58fb8', '#d7c46a', '#7fbfbf', '#c98f8f', '#a3a3d6'];
const JOINT_COLORS = ['#e8423f', '#2f6fde', '#16a34a', '#d97706', '#9333ea', '#0891b2', '#db2777'];

const parts: Part[] = [];
const joints: Joint[] = [];
let baseId: number | null = null;
let activeJointId: number | null = null;
let nextPartId = 1;
let nextJointId = 1;
/** Abschnitt mit den ausgeblendeten Teilen eingeklappt. */
let hiddenFolded = false;
/** Teileliste eingeklappt (bleibt im Browser gespeichert). */
let partsFolded = false;
try { partsFolded = localStorage.getItem('partjoin.partsFolded') === '1'; } catch { /* ohne Speicher */ }
const SNAP_PX = 14;
/** Auch sehr fein aufgelöste Flächen (Millionen Dreiecke) vollständig erfassen. */
const FEATURE_OPTS = { maxTris: 4_000_000 };
/** Auswahlmodus im 3D-Fenster: Flächen (Standard), Kanten, Punkte oder Pinsel (Fläche selbst bemalen). */
type PickMode = 'face' | 'edge' | 'point' | 'brush';
/** Pinselgröße in Bildschirmpixeln (Radius). */
const BRUSH_PX = 16;
let pickMode: PickMode = 'face';
/** Fangradius für Kanten in Bildschirmpixeln. */
const EDGE_PX = 10;
/** Welches Teil wird von welcher Verbindung bewegt (nach dem letzten solveAll). */
const movedBy = new Map<number, number>();
/** Von welchem Ausgangsteil aus wurde jedes Teil platziert (Teile mit gleichem Wert hängen zusammen). */
const seedOf = new Map<number, number>();

// ---------------------------------------------------------------- Szene

const view = document.getElementById('view')!;
const partsEl = document.getElementById('parts')!;
const jointsEl = document.getElementById('joints')!;
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
view.prepend(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 10000);
camera.position.set(120, -160, 120);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
// Mittlere Maustaste verschiebt die Ansicht (wie in den meisten Slicern), gezoomt wird mit dem Mausrad
controls.mouseButtons = { LEFT: THREE.MOUSE.ROTATE, MIDDLE: THREE.MOUSE.PAN, RIGHT: THREE.MOUSE.PAN };
// Mausrad zoomt dorthin, wo der Mauszeiger steht
controls.zoomToCursor = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x445066, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(0.4, -0.6, 1);
camera.add(sun);
scene.add(camera);

const grid = new THREE.GridHelper(400, 40, 0x8890a0, 0xb8bec9);
grid.rotation.x = Math.PI / 2;
(grid.material as THREE.Material).transparent = true;
(grid.material as THREE.Material).opacity = 0.35;
scene.add(grid);

const markerGroup = new THREE.Group();
scene.add(markerGroup);
const faceGroup = new THREE.Group();
scene.add(faceGroup);
/** Einfärbung der Prüfansicht (Berührung, Spalt, Durchdringung). */
const checkGroup = new THREE.Group();
scene.add(checkGroup);
let checkOn = false;

const cursor = new THREE.Mesh(
  new THREE.SphereGeometry(1, 16, 12),
  new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.85, depthTest: false }),
);
cursor.renderOrder = 10;
cursor.visible = false;
scene.add(cursor);

/** Vorschau der Fläche unter dem Mauszeiger im Flächenmodus. */
const hoverMesh = new THREE.Mesh(
  new THREE.BufferGeometry(),
  new THREE.MeshBasicMaterial({
    color: 0xffffff, transparent: true, opacity: 0.35, side: THREE.DoubleSide, depthWrite: false,
    polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2,
  }),
);
hoverMesh.matrixAutoUpdate = false;
hoverMesh.visible = false;
scene.add(hoverMesh);
let hoverKey = '';
let hoverTris: Set<number> | null = null;
/** Vorschau der Kante unter dem Mauszeiger im Kantenmodus. */
const hoverLine = new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false, transparent: true, opacity: 0.9 }));
hoverLine.matrixAutoUpdate = false;
hoverLine.visible = false;
hoverLine.renderOrder = 15;
scene.add(hoverLine);

function applyTheme() {
  const dark = matchMedia('(prefers-color-scheme: dark)').matches &&
    document.documentElement.dataset.theme !== 'light' || document.documentElement.dataset.theme === 'dark';
  const bg = getComputedStyle(document.documentElement).getPropertyValue('--view').trim();
  renderer.setClearColor(new THREE.Color(bg || (dark ? '#161a21' : '#e9ecf1')));
}
applyTheme();
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applyTheme);
new MutationObserver(applyTheme).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });

function resize() {
  const w = view.clientWidth;
  const h = view.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);
resize();

/** Markierungen behalten auf dem Bildschirm eine feste Größe (in Pixeln). */
function scaleMarkers() {
  const h = renderer.domElement.clientHeight || 1;
  const k = (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2))) / h;
  for (const o of markerGroup.children) {
    const u = o.userData as { px: number; anchor: THREE.Vector3; normal: THREE.Vector3; lift: number };
    const wpp = k * camera.position.distanceTo(u.anchor);
    o.scale.setScalar(u.px * wpp);
    o.position.copy(u.anchor).addScaledVector(u.normal, u.lift * wpp);
  }
  const wpp = k * camera.position.distanceTo(cursor.position);
  // Im Pinselmodus zeigt der Cursor die Pinselgröße
  cursor.scale.setScalar((pickMode === 'brush' ? BRUSH_PX : 4) * wpp);
  (cursor.material as THREE.MeshBasicMaterial).opacity = pickMode === 'brush' ? 0.3 : 0.85;
}

renderer.setAnimationLoop(() => {
  controls.update();
  // Clipping-Ebenen an den Abstand anpassen – beim Heranzoomen an den Mauszeiger wird sonst abgeschnitten
  const dist = camera.position.distanceTo(controls.target);
  const near = dist / 100;
  if (Math.abs(camera.near - near) > near * 0.2) {
    camera.near = near;
    camera.far = Math.max(dist * 100, sceneScale() * 40);
    camera.updateProjectionMatrix();
  }
  scaleMarkers();
  renderer.render(scene, camera);
});

/** Vier Pfeile: Teil frei bewegen */
const MOVE_ICON = '<svg viewBox="0 0 24 24" width="18" height="18" aria-hidden="true"><path fill="currentColor" d="M12 1.5 15.5 5H13v6h6V8.5l3.5 3.5-3.5 3.5V13h-6v6h2.5L12 22.5 8.5 19H11v-6H5v2.5L1.5 12 5 8.5V11h6V5H8.5z"/></svg>';

// ---------------------------------------------------------------- Rückgängig / Wiederholen

/**
 * Zustand zum Zurückholen: Teile (Lage, Sichtbarkeit, Farbe, Name, Gruppe) und Verbindungen samt
 * Auswahl. Geometrie und Zwischenergebnisse werden geteilt, nicht kopiert – sie werden nie verändert,
 * nur ersetzt.
 */
interface Snapshot {
  parts: { part: Part; layout: THREE.Matrix4; visible: boolean; color: string; name: string; group?: string }[];
  joints: Joint[];
  baseId: number | null;
  activeJointId: number | null;
  sig: string;
}
const UNDO_LIMIT = 60;
const undoStack: Snapshot[] = [];
const redoStack: Snapshot[] = [];
/** Zustand vor der laufenden Aktion; wird erst bei der nächsten Aktion (oder beim Rückgängig) übernommen, wenn sich etwas geändert hat. */
let pendingUndo: Snapshot | null = null;
/** Entfernte Teile, die die Historie noch zurückholen kann. */
const removedParts = new Set<Part>();

const jointCopy = (j: Joint): Joint => ({ ...j, ma: [...j.ma], mb: [...j.mb], fa: [...j.fa], fb: [...j.fb] });

/** Kurzer Fingerabdruck des Zustands: hat eine Aktion wirklich etwas geändert? */
function stateSig(): string {
  const r = (x: number) => Math.round(x * 1e4);
  const fs = (f: PickedFeature) => [f.kind, f.on ?? 0, f.tris[0], f.tris.length, f.pieces?.length ?? 0];
  return JSON.stringify([
    baseId,
    parts.map((p) => [p.id, p.name, p.visible, p.color.getHexString(), p.group ?? '', p.layout.elements.map(r)]),
    joints.map((j) => [
      j.id, j.a, j.b, j.applied, j.alt, j.seat, j.ordered, j.normals, j.overlap, j.spin, j.turnIdx ?? 0, j.proposalIdx ?? 0,
      j.adjust ? [...j.adjust.R, ...j.adjust.t].map(r) : 0,
      j.ma.map((m) => m.p.map(r)), j.mb.map((m) => m.p.map(r)), j.fa.map(fs), j.fb.map(fs),
    ]),
  ]);
}

function snapshot(): Snapshot {
  return {
    parts: parts.map((p) => ({ part: p, layout: p.layout.clone(), visible: p.visible, color: `#${p.color.getHexString()}`, name: p.name, group: p.group })),
    joints: joints.map(jointCopy),
    baseId,
    activeJointId,
    sig: stateSig(),
  };
}

function restoreSnapshot(snap: Snapshot) {
  closeGizmo();
  const keep = new Set(snap.parts.map((e) => e.part));
  for (const p of parts) if (!keep.has(p)) { scene.remove(p.mesh); removedParts.add(p); }
  parts.length = 0;
  for (const e of snap.parts) {
    const p = e.part;
    if (!p.mesh.parent) scene.add(p.mesh);
    removedParts.delete(p);
    p.layout.copy(e.layout);
    p.visible = e.visible;
    p.mesh.visible = e.visible;
    p.color.set(e.color);
    (p.mesh.material as THREE.MeshStandardMaterial).color.copy(p.color);
    p.name = e.name;
    p.group = e.group;
    parts.push(p);
  }
  // Verbindungsobjekte möglichst weiterverwenden (an ihnen hängen zwischengespeicherte Ergebnisse)
  const next = snap.joints.map((sj) => {
    const copy = jointCopy(sj);
    const existing = joints.find((x) => x.id === sj.id);
    if (!existing) return copy;
    for (const k of Object.keys(existing) as (keyof Joint)[]) if (!(k in copy)) delete existing[k];
    return Object.assign(existing, copy);
  });
  joints.length = 0;
  joints.push(...next);
  baseId = snap.baseId;
  activeJointId = snap.activeJointId;
  mergePick.clear();
  solveAll();
  renderAll();
}

/** Teile, die weder angezeigt noch von der Historie gebraucht werden, endgültig freigeben. */
function releaseUnused() {
  const used = new Set<Part>(parts);
  for (const s of [...undoStack, ...redoStack, ...(pendingUndo ? [pendingUndo] : [])]) for (const e of s.parts) used.add(e.part);
  for (const p of removedParts) {
    if (used.has(p)) continue;
    p.geom.disposeBoundsTree();
    p.geom.dispose();
    (p.mesh.material as THREE.Material).dispose();
    removedParts.delete(p);
  }
}

/** Vor jeder Bedienung aufrufen: merkt sich den Zustand, damit sie sich zurücknehmen lässt. */
function beginAction() {
  commitAction();
  pendingUndo = snapshot();
}

function commitAction() {
  if (pendingUndo && pendingUndo.sig !== stateSig()) {
    undoStack.push(pendingUndo);
    if (undoStack.length > UNDO_LIMIT) undoStack.shift();
    redoStack.length = 0;
    pendingUndo = null;
    releaseUnused();
  }
  pendingUndo = null;
}

function undo() {
  commitAction();
  const s = undoStack.pop();
  if (!s) { toast('Nichts mehr rückgängig zu machen.', 1800); return; }
  redoStack.push(snapshot());
  restoreSnapshot(s);
  toast('Rückgängig gemacht · Strg/⌘+Umschalt+Z stellt es wieder her', 2200);
}

function redo() {
  commitAction();
  const s = redoStack.pop();
  if (!s) { toast('Nichts zu wiederholen.', 1800); return; }
  undoStack.push(snapshot());
  restoreSnapshot(s);
  toast('Wiederholt', 1500);
}

// Jede Bedienung beginnt mit Maus oder Tastatur im Fenster, in der Seitenleiste oder mit dem Ablegen von Dateien.
// Die Erfassung läuft vor allen anderen Handlern (auch vor dem Malen und dem Strg+Ziehen).
const isUndoKey = (e: KeyboardEvent) => (e.ctrlKey || e.metaKey) && ['z', 'y'].includes(e.key.toLowerCase());
for (const id of ['view', 'panel']) {
  const el = document.getElementById(id)!;
  el.addEventListener('pointerdown', () => beginAction(), { capture: true });
  el.addEventListener('keydown', (e) => { if (!isUndoKey(e)) beginAction(); }, { capture: true });
}
window.addEventListener('drop', () => beginAction(), { capture: true });

// ---------------------------------------------------------------- Hilfsfunktionen

const v3 = (v: THREE.Vector3): Vec3 => [v.x, v.y, v.z];
const getPart = (id: number | null) => parts.find((p) => p.id === id);
const activeJoint = () => joints.find((j) => j.id === activeJointId);
const esc = (s: string) => s.replace(/[<>&"]/g, (c) => ({ '<': '&lt;', '>': '&gt;', '&': '&amp;', '"': '&quot;' })[c]!);
const hasFeatures = (j: Joint) => j.fa.length > 0 || j.fb.length > 0;
const positionsOf = (p: Part) => p.geom.getAttribute('position').array as Float32Array;

function rigidToMatrix(T: Rigid): THREE.Matrix4 {
  const R = T.R;
  return new THREE.Matrix4().set(R[0], R[1], R[2], T.t[0], R[3], R[4], R[5], T.t[1], R[6], R[7], R[8], T.t[2], 0, 0, 0, 1);
}

function matrixToRigid(m: THREE.Matrix4): Rigid {
  const e = m.elements; // spaltenweise
  return { R: [e[0], e[4], e[8], e[1], e[5], e[9], e[2], e[6], e[10]], t: [e[12], e[13], e[14]] };
}

function invertRigid(T: Rigid): Rigid {
  const R = T.R;
  const Rt: Rigid['R'] = [R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]];
  const t = rotate(Rt, T.t);
  return { R: Rt, t: [-t[0], -t[1], -t[2]] };
}

/** Bewegung D im Koordinatensystem M als Weltbewegung: M · D · M⁻¹. */
const conjugate = (M: Rigid, D: Rigid) => compose(compose(M, D), invertRigid(M));

function markerToWorld(m: Marker, M: THREE.Matrix4): Marker {
  const p = new THREE.Vector3(...m.p).applyMatrix4(M);
  const n = new THREE.Vector3(...m.n).transformDirection(M);
  return { p: v3(p), n: v3(n) };
}

function motionToWorld(m: FreeMotion, M: Rigid): FreeMotion {
  return m.kind === 'rotate'
    ? { kind: 'rotate', point: compose(M, { R: IDENTITY.R, t: m.point }).t, axis: rotate(M.R, m.axis) }
    : { kind: 'translate', axis: rotate(M.R, m.axis) };
}

/** Bezugspunkt einer Fläche (Mitte), lokal. */
function featureCenter(f: Feature): Vec3 {
  if (f.kind === 'cylinder' || f.kind === 'edge') return f.center;
  if (f.kind === 'plane' && f.circle) return f.circle.center;
  return f.centroid;
}

/** Stelle auf der Oberfläche für die Beschriftung (lokal): bei Zylindern das angeklickte Dreieck. */
function featureAnchor(part: Part, f: Feature): [Vec3, Vec3] {
  if (f.kind !== 'cylinder') return [featureCenter(f), f.normal];
  const p = positionsOf(part);
  const i = f.tris[0] * 9;
  const c: Vec3 = [(p[i] + p[i + 3] + p[i + 6]) / 3, (p[i + 1] + p[i + 4] + p[i + 7]) / 3, (p[i + 2] + p[i + 5] + p[i + 8]) / 3];
  const t = part.topo?.normals;
  const n: Vec3 = t ? [t[f.tris[0] * 3], t[f.tris[0] * 3 + 1], t[f.tris[0] * 3 + 2]] : [0, 0, 0];
  return [c, n];
}

/** Ungefähre Größe einer Fläche (Radius des umschließenden Bereichs). */
function featureRadius(f: Feature): number {
  if (f.kind === 'cylinder') return Math.hypot(f.radius, f.length / 2);
  if (f.kind === 'plane') return Math.hypot(f.axes.size[0], f.axes.size[1]) / 2;
  if (f.kind === 'edge') return f.length / 2;
  return Math.sqrt(f.area / Math.PI);
}

/** Typische Modellgröße – bestimmt Markergröße, Fangradius usw. */
function sceneScale(): number {
  let s = 0;
  for (const p of parts) s = Math.max(s, p.geom.boundingSphere!.radius);
  return s || 50;
}

let toastTimer = 0;
function toast(msg: string, ms = 3200) {
  const el = document.getElementById('toast')!;
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = window.setTimeout(() => el.classList.remove('show'), ms);
}

function fitView() {
  const box = new THREE.Box3();
  for (const p of parts) if (p.visible) box.expandByObject(p.mesh);
  if (box.isEmpty()) return;
  const c = box.getCenter(new THREE.Vector3());
  const r = box.getSize(new THREE.Vector3()).length() / 2;
  const dir = camera.position.clone().sub(controls.target).normalize();
  const d = r / Math.sin(THREE.MathUtils.degToRad(camera.fov / 2)) * 1.05;
  controls.target.copy(c);
  camera.position.copy(c).addScaledVector(dir, d);
  camera.near = d / 100;
  camera.far = d * 100;
  camera.updateProjectionMatrix();
}

// ---------------------------------------------------------------- Teile

/** Platz rechts neben den bereits geladenen Teilen für einen Bereich mit diesem Umriss. */
function nextLayout(bb: THREE.Box3): THREE.Matrix4 {
  let x = 0;
  for (const p of parts) x = Math.max(x, new THREE.Box3().setFromObject(p.mesh).max.x);
  const gap = parts.length ? sceneScale() * 0.3 : 0;
  return new THREE.Matrix4().makeTranslation(
    parts.length ? x + gap - bb.min.x : -(bb.min.x + bb.max.x) / 2,
    -(bb.min.y + bb.max.y) / 2,
    -bb.min.z,
  );
}

function addPart(name: string, geom: THREE.BufferGeometry, opts: { color?: string; group?: string; layout?: THREE.Matrix4 } = {}) {
  geom.computeBoundsTree();
  const color = new THREE.Color(opts.color ?? PART_COLORS[(nextPartId - 1) % PART_COLORS.length]);
  const mesh = new THREE.Mesh(
    geom,
    new THREE.MeshStandardMaterial({ color, roughness: 0.65, metalness: 0.05, side: THREE.DoubleSide, flatShading: false }),
  );
  mesh.matrixAutoUpdate = false;
  // Lage aus der Datei, mehrteilige Dateien gemeinsam verschoben – sonst neben die anderen Teile
  const layout = opts.layout?.clone()
    ?? ((document.getElementById('keepPos') as HTMLInputElement).checked ? new THREE.Matrix4() : nextLayout(geom.boundingBox!));
  mesh.matrix.copy(layout);
  mesh.matrixWorldNeedsUpdate = true;

  const part: Part = {
    id: nextPartId++, name, geom, mesh, color, layout, visible: true, samples: sampleSurface(geom, 6000), group: opts.group,
  };
  mesh.userData.partId = part.id;
  parts.push(part);
  scene.add(mesh);
  if (baseId === null) baseId = part.id;
  return part;
}

/** Alle Teile, die sich mit diesem gemeinsam bewegen (inklusive des Teils selbst). */
function groupOf(p: Part): Part[] {
  return p.group ? parts.filter((q) => q.group === p.group) : [p];
}

/**
 * Sorgt dafür, dass es eine Verbindung zum Loslegen gibt: zwischen zwei Teilen, die noch nicht
 * zusammenhängen (nicht derselben Gruppe angehören und nicht schon verbunden sind).
 */
function ensureJoint() {
  if (parts.length < 2 || joints.some((j) => !j.applied && j.ma.length + j.mb.length + j.fa.length + j.fb.length === 0)) return;
  const unit = (p: Part) => p.group ?? `#${p.id}`;
  const connected = new Set(joints.flatMap((j) => [j.a, j.b]).filter((x): x is number => x !== null).map((id) => unit(getPart(id)!)));
  const a = joints.length ? parts.find((p) => connected.has(unit(p))) : getPart(baseId);
  const b = parts.find((p) => a && unit(p) !== unit(a) && !connected.has(unit(p)));
  if (!a || !b) return;
  const j = addJoint();
  j.a = a.id;
  j.b = b.id;
  j.auto = true;
}

async function loadFiles(files: FileList | File[]) {
  const list = Array.from(files);
  const split = (document.getElementById('splitParts') as HTMLInputElement).checked;
  const keep = (document.getElementById('keepPos') as HTMLInputElement).checked;
  // Farben für OBJ: passende .mtl-Datei (gleicher Name oder die einzige mitgegebene)
  const mtls = new Map<string, string>();
  for (const f of list) if (f.name.toLowerCase().endsWith('.mtl')) mtls.set(f.name.replace(/\.[^.]+$/, '').toLowerCase(), await f.text());
  for (const file of list) {
    if (file.name.toLowerCase().endsWith('.mtl')) continue;
    try {
      const stem = file.name.replace(/\.[^.]+$/, '').toLowerCase();
      const mtl = mtls.get(stem) ?? (mtls.size === 1 ? [...mtls.values()][0] : undefined);
      const info: { printer?: string } = {};
      let loaded: LoadedPart[] = await loadModel(file, { split, mtl, info });
      // Drucker eines Bambu-Projekts vorauswählen (die eigene Wahl bleibt gespeichert)
      if (info.printer) bambuPrinter.value = info.printer;
      // Stecken mehrere lose Teile in einem Netz (z. B. eine ganze Druckplatte), diese trennen
      let separated = false;
      if (split) {
        const members = new Map<string, number>();
        for (const l of loaded) if (l.group) members.set(l.group, (members.get(l.group) ?? 0) + 1);
        loaded = loaded.flatMap((l) => {
          const pieces = splitLoosePieces(l.geom.getAttribute('position').array as Float32Array);
          if (pieces.length < 2) return [l];
          separated = true;
          l.geom.dispose();
          // Gehörte das Netz zu einem Objekt aus mehreren Teilen, bleiben die Stücke in dessen Gruppe
          const group = l.group && members.get(l.group)! > 1 ? l.group : undefined;
          return pieces.map((p, k) => {
            const g = new THREE.BufferGeometry();
            g.setAttribute('position', new THREE.BufferAttribute(p, 3));
            g.computeVertexNormals();
            g.computeBoundingBox();
            g.computeBoundingSphere();
            return { name: `${l.name} ${k + 1}`, geom: g, color: l.color, group };
          });
        });
      }
      // Teile einer Datei gemeinsam verschieben, damit ihre Lage zueinander erhalten bleibt
      const bb = new THREE.Box3();
      for (const l of loaded) bb.union(l.geom.boundingBox!);
      const layout = keep ? new THREE.Matrix4() : nextLayout(bb);
      for (const l of loaded) addPart(l.name, l.geom, { color: l.color, group: l.group, layout });
      if (loaded.length > 1) toast(`${file.name}: ${loaded.length} Teile geladen${separated ? ' (lose Teile getrennt)' : ''}.`, 5000);
    } catch (e) {
      toast(`${file.name}: ${(e as Error).message}`);
    }
  }
  ensureJoint();
  solveAll();
  fitView();
  renderAll();
}

function removePart(id: number) {
  const p = getPart(id);
  if (!p) return;
  if (gizmoTarget && (gizmoTarget.kind === 'part' ? gizmoTarget.id === id : true)) closeGizmo();
  scene.remove(p.mesh);
  // Nicht freigeben: „Rückgängig" kann das Teil zurückholen (freigegeben wird beim Aufräumen der Historie)
  removedParts.add(p);
  parts.splice(parts.indexOf(p), 1);
  for (const j of joints) {
    if (j.a === id) { j.a = null; j.ma = []; j.fa = []; j.applied = false; markersChanged(j); }
    if (j.b === id) { j.b = null; j.mb = []; j.fb = []; j.applied = false; markersChanged(j); }
    if (j.fa.some((f) => f.on === id) || j.fb.some((f) => f.on === id)) {
      j.fa = j.fa.filter((f) => f.on !== id);
      j.fb = j.fb.filter((f) => f.on !== id);
      markersChanged(j);
    }
  }
  if (baseId === id) baseId = parts[0]?.id ?? null;
  solveAll();
  renderAll();
}

function topologyOf(p: Part): Topology {
  p.topo ??= buildTopology(positionsOf(p));
  return p.topo;
}

// ---------------------------------------------------------------- Verbindungen

function addJoint() {
  // Teil 1: ein bereits verbundenes Teil (oder die Basis), Teil 2: das erste noch freie Teil
  const used = new Set(joints.flatMap((j) => [j.a, j.b]).filter((x): x is number => x !== null));
  const a = used.size ? ([...used].find((x) => x === baseId) ?? [...used][0]) : baseId;
  const b = parts.find((p) => p.id !== a && !used.has(p.id))?.id ?? null;
  const id = nextJointId++;
  const j: Joint = {
    id,
    color: new THREE.Color(JOINT_COLORS[(id - 1) % JOINT_COLORS.length]),
    a,
    b,
    ma: [],
    mb: [],
    fa: [],
    fb: [],
    ordered: false,
    normals: 'auto',
    overlap: 0,
    spin: 0,
    seat: true,
    applied: false,
    alt: 0,
  };
  joints.push(j);
  activeJointId = j.id;
  return j;
}

function removeJoint(id: number) {
  const i = joints.findIndex((j) => j.id === id);
  if (i < 0) return;
  if (gizmoTarget?.kind === 'joint' && gizmoTarget.id === id) closeGizmo();
  joints.splice(i, 1);
  if (activeJointId === id) activeJointId = joints[joints.length - 1]?.id ?? null;
  solveAll();
  renderAll();
}

/** Auswahl hat sich geändert → alte Alternativen, Korrekturen und Zwischenergebnisse verwerfen. */
function markersChanged(j: Joint) {
  mergePick.delete(j.id);
  j.alt = 0;
  j.turnIdx = 0;
  j.adjust = undefined;
  j.adjustInfo = undefined;
  j.cache = undefined;
}

/**
 * Setzt alle Teile zusammen: Ausgehend vom Basisteil werden die Verbindungen der Reihe nach
 * gelöst (Breitensuche), jedes neu erreichte Teil wird an ein bereits platziertes angefügt.
 */
function solveAll() {
  const placed = new Map<number, THREE.Matrix4>();
  movedBy.clear();
  seedOf.clear();
  let currentSeed = -1;
  for (const j of joints) {
    j.result = undefined;
    j.fixedSide = undefined;
    j.status = undefined;
    j.perm = undefined;
    j.free = undefined;
    if (!j.applied) j.last = undefined;
  }
  const seeds = [baseId, ...joints.map((j) => j.a), ...parts.map((p) => p.id)].filter(
    (x): x is number => x !== null && !!getPart(x),
  );

  /** Teil platzieren – die übrigen Teile seiner Gruppe mit derselben Bewegung. */
  const place = (id: number, M: THREE.Matrix4, queue: number[], jointId?: number) => {
    const p = getPart(id)!;
    const delta = M.clone().multiply(p.layout.clone().invert());
    for (const q of groupOf(p)) {
      if (placed.has(q.id)) continue;
      placed.set(q.id, q === p ? M.clone() : delta.clone().multiply(q.layout));
      seedOf.set(q.id, currentSeed);
      if (jointId !== undefined) movedBy.set(q.id, jointId);
      queue.push(q.id);
    }
  };

  /** Verbindungen, deren Flächen auf Nachbarteilen liegen, die noch nicht platziert sind. */
  let deferred: { j: Joint; cur: number }[] = [];
  /** Verbindung j vom bereits platzierten Teil cur aus lösen und das andere Teil platzieren. */
  const handle = (j: Joint, cur: number, queue: number[]) => {
        if (!j.applied || j.a === null || j.b === null) return;
        if (j.a !== cur && j.b !== cur) return;
        const fixedSide: Side = j.a === cur ? 'a' : 'b';
        const other = fixedSide === 'a' ? j.b : j.a;
        if (placed.has(other)) {
          const same = getPart(cur)!.group && getPart(cur)!.group === getPart(other)!.group;
          if (!j.fixedSide)
            j.status = {
              kind: 'warn',
              text: same
                ? 'Beide Teile gehören zur selben Gruppe und liegen schon richtig zueinander. Zum Verbinden ein Teil aus der Gruppe lösen (⛓ in der Teileliste).'
                : 'Beide Teile sind schon über andere Verbindungen festgelegt – diese Verbindung wird ignoriert.',
            };
          return;
        }
        // Flächen auf Nachbarteilen: nur auf der festen Seite, und diese Teile müssen schon liegen
        const movingSide: Side = fixedSide === 'a' ? 'b' : 'a';
        if (helperParts(j, movingSide).length) {
          j.status = {
            kind: 'err',
            text: `Flächen auf weiteren Teilen gehen nur auf der Seite, die schon festliegt. Hier liegt „${nameOf(cur)}" zuerst fest – wähle die Flächen auf dessen Seite oder ein anderes Basisteil.`,
          };
          return;
        }
        if (helperParts(j, fixedSide).some((id) => !placed.has(id))) {
          if (!deferred.some((d) => d.j === j)) deferred.push({ j, cur });
          return;
        }
        j.fixedSide = fixedSide;
        const fixedM = placed.get(cur)!;
        const Mf = matrixToRigid(fixedM);
        const waiting = pendingSide(j);
        if (waiting && j.last) {
          // Auswahl unvollständig: bisherige Lage behalten
          const rel = j.last.side === fixedSide ? j.last.rel : invertRigid(j.last.rel);
          j.status = { kind: 'info', text: `Wähle noch das Gegenstück auf „${nameOf(waiting === 'a' ? j.a : j.b)}" – bis dahin bleibt die bisherige Lage.` };
          place(other, rigidToMatrix(compose(Mf, rel)), queue, j.id);
          return;
        }
        const coarse = hasFeatures(j)
          ? solveFeatureJoint(j, getPart(cur)!, getPart(other)!, Mf, placed)
          : solvePointJoint(j, getPart(other)!, fixedM);
        if (!coarse) return;
        const T = j.adjust ? compose(conjugate(Mf, j.adjust), coarse) : coarse;
        j.last = { side: fixedSide, rel: compose(invertRigid(Mf), T) };
        place(other, rigidToMatrix(T), queue, j.id);
  };

  for (const seed of seeds) {
    if (placed.has(seed)) continue;
    currentSeed = seed;
    const queue: number[] = [];
    place(seed, getPart(seed)!.layout.clone(), queue);
    const run = () => {
      while (queue.length) {
        const cur = queue.shift()!;
        for (const j of joints) handle(j, cur, queue);
      }
    };
    run();
    // Zurückgestellte Verbindungen erneut versuchen, solange dadurch weitere Teile platziert werden
    for (let again = true; again; ) {
      for (const d of deferred.splice(0)) handle(d.j, d.cur, queue);
      again = queue.length > 0;
      run();
    }
  }
  for (const { j } of deferred) {
    const missing = helperParts(j, j.a !== null && placed.has(j.a) ? 'a' : 'b').filter((id) => !placed.has(id));
    j.status = {
      kind: 'err',
      text: `${missing.map((id) => `„${nameOf(id)}"`).join(', ')} liegt noch nicht fest – erst damit verbinden, dann lassen sich seine Flächen hier nutzen.`,
    };
  }
  deferred = [];

  for (const p of parts) {
    p.mesh.matrix.copy(placed.get(p.id) ?? p.layout);
    p.mesh.matrixWorldNeedsUpdate = true;
    p.mesh.updateMatrixWorld(true);
  }
  for (const j of joints) {
    if (j.applied || j.status) continue;
    const na = j.ma.length;
    const nb = j.mb.length;
    if (j.a === null || j.b === null) j.status = { kind: 'info', text: 'Wähle zwei Teile aus (oder klicke sie im Modell an).' };
    else if (hasFeatures(j)) {
      if (j.fa.length === 0 || j.fb.length === 0)
        j.status = { kind: 'info', text: `Wähle noch die passende Fläche auf „${nameOf(j.fa.length ? j.b : j.a)}".` };
      else j.status = { kind: 'info', text: 'Bereit zum Verbinden.' };
    } else if (na === 0 && nb === 0) j.status = { kind: 'info', text: 'Klicke auf passende Punkte oder Flächen beider Teile.' };
    else if (na !== nb) j.status = { kind: 'info', text: `Noch ${Math.abs(na - nb)} Markierung(en) auf ${na < nb ? nameOf(j.a) : nameOf(j.b)} setzen.` };
    else j.status = { kind: 'info', text: na < 3 ? 'Bereit. (Mit 3+ Punkten wird es am genauesten.)' : 'Bereit zum Verbinden.' };
  }
}

/** Nachbarteile, auf denen Flächen dieser Seite liegen (auch Teilflächen zusammengefasster Flächen). */
function helperParts(j: Joint, side: Side): number[] {
  const own = side === 'a' ? j.a : j.b;
  return [...new Set((side === 'a' ? j.fa : j.fb).flatMap((f) => f.pieces ?? [f]).map((f) => f.on).filter((id): id is number => id !== undefined && id !== own))];
}

/** Zum Zusammenfassen angehakte Flächen je Verbindung (eine Seite). */
const mergePick = new Map<number, { side: Side; idx: Set<number> }>();
const mergeable = (f: Feature) => f.kind === 'plane' || f.kind === 'surface';

/**
 * Fasst Einzelflächen einer Seite zu einer Fläche zusammen – im Koordinatensystem des Teils dieser Seite,
 * Teilflächen auf Nachbarteilen über deren Lage `worldOf`.
 */
function mergeOf(j: Joint, side: Side, pieces: PickedFeature[], worldOf: (id: number) => THREE.Matrix4): PickedFeature {
  const own = getPart(side === 'a' ? j.a : j.b)!;
  const inv = worldOf(own.id).clone().invert();
  const merged = mergeFeatures(pieces.map((f) => {
    const part = featurePart(j, side, f)!;
    return { positions: positionsOf(part), tris: f.tris, T: matrixToRigid(inv.clone().multiply(worldOf(part.id))) };
  }));
  return { ...merged, tris: pieces[0].tris, pieces };
}

/** Zusammengefasste Flächen auf Nachbarteilen mit deren aktueller Lage neu berechnen (zwischengespeichert). */
const mergedCache = new WeakMap<PickedFeature, { key: string; f: PickedFeature }>();
function currentMerge(j: Joint, side: Side, f: PickedFeature, worldOf: (id: number) => THREE.Matrix4): PickedFeature {
  const own = side === 'a' ? j.a : j.b;
  if (!f.pieces?.some((p) => p.on !== undefined && p.on !== own)) return f;
  const inv = worldOf(own!).clone().invert();
  const key = JSON.stringify(f.pieces.map((p) => [p.on ?? 0, inv.clone().multiply(worldOf(p.on ?? own!)).elements.map((x) => Math.round(x * 1e4))]));
  const hit = mergedCache.get(f);
  if (hit?.key === key) return hit.f;
  const g = mergeOf(j, side, f.pieces, worldOf);
  mergedCache.set(f, { key, f: g });
  return g;
}

/** Teil, auf dem eine Fläche liegt. */
function featurePart(j: Joint, side: Side, f: PickedFeature): Part | undefined {
  return getPart(f.on ?? (side === 'a' ? j.a : j.b));
}

/** Alle Teile, die (ohne die Verbindung skip) fest mit dem Teil id zusammenhängen – über Verbindungen und Gruppen. */
function rigidWith(id: number, skip: Joint): Set<number> {
  const seen = new Set<number>([id]);
  const stack = [id];
  while (stack.length) {
    const cur = stack.pop()!;
    const p = getPart(cur);
    const next = [
      ...(p ? groupOf(p).map((q) => q.id) : []),
      ...joints
        .filter((k) => k !== skip && k.applied && k.fixedSide && k.a !== null && k.b !== null && (k.a === cur || k.b === cur))
        .map((k) => (k.a === cur ? k.b! : k.a!)),
    ];
    for (const n of next) if (!seen.has(n)) { seen.add(n); stack.push(n); }
  }
  return seen;
}

/**
 * Darf auf diesem Teil eine Fläche für die Verbindung gewählt werden, obwohl es keins ihrer beiden
 * Teile ist? Ja, wenn es fest mit genau einer Seite verbunden ist (z. B. die Haare am Kopf, wenn
 * eine Maske auf Kopf und Haare zugleich passt).
 */
function helperSide(j: Joint | undefined, part: Part): Side | null {
  if (!j || j.a === null || j.b === null || part.id === j.a || part.id === j.b) return null;
  const inA = rigidWith(j.a, j).has(part.id);
  const inB = rigidWith(j.b, j).has(part.id);
  return inA && !inB ? 'a' : inB && !inA ? 'b' : null;
}

/** Seite, auf der zu einer Fläche oder Markierung noch das Gegenstück fehlt (sonst null). */
function pendingSide(j: Joint): Side | null {
  const count = (l: Feature[], kind: 'cylinder' | 'edge') => l.filter((f) => f.kind === kind).length;
  const flat = (l: Feature[]) => l.length - count(l, 'cylinder') - count(l, 'edge');
  const diff = [
    j.ma.length - j.mb.length, flat(j.fa) - flat(j.fb),
    count(j.fa, 'cylinder') - count(j.fb, 'cylinder'), count(j.fa, 'edge') - count(j.fb, 'edge'),
  ];
  const d = diff.find((x) => x !== 0);
  return d === undefined ? null : d < 0 ? 'a' : 'b';
}

/** Verbindung nur über Punkte (bisheriges Verfahren). Liefert die Lage des beweglichen Teils in Weltkoordinaten. */
function solvePointJoint(j: Joint, moving: Part, fixedM: THREE.Matrix4): Rigid | null {
  const fixedSide = j.fixedSide!;
  const dst = (fixedSide === 'a' ? j.ma : j.mb).map((m) => markerToWorld(m, fixedM));
  const src = fixedSide === 'a' ? j.mb : j.ma;
  const res = solveJoint({ src, dst, ordered: j.ordered, normals: j.normals });
  j.result = res;
  if (res.error) {
    j.status = { kind: 'err', text: res.error };
    return null;
  }
  dropEquivalent(res, moving);
  const sol = res.solutions[Math.min(j.alt, res.solutions.length - 1)];
  j.perm = sol.perm;
  if (sol.freeAxis) j.free = [{ kind: 'rotate', point: sol.freeAxis.point, axis: sol.freeAxis.axis }];
  const T = applyAdjustments(sol, dst, j.spin, j.overlap);

  const scaleRef = sceneScale();
  // Handgesetzte Punkte (ohne Einrasten) sind nie ganz exakt – grobe Fehler deuten auf falsche Punkte hin
  const good = !!j.adjust || sol.rmsd < Math.max(0.1, scaleRef * 0.015);
  let text = `Abweichung ${fmt(sol.rmsd)} mm`;
  if (j.adjustInfo) text += ` · ${j.adjustInfo}`;
  if (res.ambiguous && j.alt === 0) {
    j.status = { kind: 'warn', text: `${text} – Zuordnung nicht eindeutig. Prüfe das Ergebnis, nutze „Alternative" oder setze einen weiteren, unsymmetrischen Punkt.` };
  } else if (!good) {
    j.status = { kind: 'warn', text: `${text} – die Punkte passen nicht genau zusammen. Markierungen prüfen.` };
  } else {
    j.status = { kind: 'ok', text: `✓ Verbunden · ${text}` };
  }
  return T;
}

/**
 * Verbindung über Flächen (und optional zusätzliche Punkte). Gelöst wird im Koordinatensystem des
 * festen Teils, so kann das Ergebnis zwischengespeichert werden – das automatische Einpassen ist teuer.
 */
function solveFeatureJoint(j: Joint, fixed: Part, moving: Part, Mf: Rigid, placed: Map<number, THREE.Matrix4>): Rigid | null {
  const fixedSide = j.fixedSide!;
  const srcF = fixedSide === 'a' ? j.fb : j.fa;
  const srcM = fixedSide === 'a' ? j.mb : j.ma;
  const dstM = fixedSide === 'a' ? j.ma : j.mb;
  // Flächen auf Nachbarteilen in das Koordinatensystem des festen Teils übertragen
  const helpers = helperRelations(j, fixedSide, fixed, (id) => placed.get(id)!);
  const dstF = (fixedSide === 'a' ? j.fa : j.fb).map((f) => {
    if (f.pieces) return currentMerge(j, fixedSide, f, (id) => placed.get(id)!);
    const rel = f.on !== undefined ? helpers.find((h) => h.id === f.on)?.rel : undefined;
    return rel ? transformFeature(f, matrixToRigid(rel)) : f;
  });
  const sig = (f: PickedFeature) => [f.on ?? 0, f.tris[0], f.tris.length, (f.pieces ?? []).map((p) => [p.on ?? 0, p.tris[0], p.tris.length])];
  const key = JSON.stringify([
    fixed.id, moving.id, srcF.map(sig), dstF.map(sig),
    helpers.map((h) => [h.id, h.rel.elements.map((x) => Math.round(x * 1e4))]),
    srcM, dstM, j.alt, j.seat, j.ordered, j.normals, j.overlap,
  ]);

  if (j.cache?.key !== key) {
    j.cache = undefined;
    const fit = fitFeatures({
      srcF, dstF, srcM, dstM, names: { src: moving.name, dst: fixed.name },
      alt: j.alt, seat: j.seat, ordered: j.ordered, normals: j.normals, overlap: j.overlap, scale: sceneScale(),
      samples: regionSamples(moving, srcF, srcM),
      target: { ...fixedTarget(j, fixed, helpers), matrix: IDENTITY },
    });
    if (fit.error) {
      j.status = { kind: 'err', text: fit.error };
      return null;
    }
    j.cache = { key, ...fit };
  }

  const c = j.cache;
  j.perm = c.perm;
  j.free = c.free.map((m) => motionToWorld(m, Mf));
  const bits = [c.summary, ...c.notes];
  if (c.info) bits.push(c.info);
  if (j.adjustInfo) bits.push(j.adjustInfo);
  const bad = c.distError > Math.max(0.1, sceneScale() * 0.01) || c.angleError > 1;
  if (bad && !j.adjust) {
    j.status = { kind: 'warn', text: `Flächen passen nicht genau zusammen (Abweichung ${fmt(c.distError)} mm / ${c.angleError.toFixed(1)}°). ${bits.join(' · ')}` };
  } else if (c.flatRotation && !j.adjust) {
    j.status = { kind: 'warn', text: `✓ Verbunden · ${bits.join(' · ')}. Zum Drehen: „Von Hand anpassen".` };
  } else {
    j.status = { kind: 'ok', text: `✓ Verbunden · ${bits.join(' · ')}` };
  }
  return compose(Mf, c.rel);
}

/** Lage der Nachbarteile (mit Flächen dieser Seite) im Koordinatensystem des festen Teils. */
function helperRelations(j: Joint, side: Side, fixed: Part, worldOf: (id: number) => THREE.Matrix4) {
  const inv = worldOf(fixed.id).clone().invert();
  return helperParts(j, side).map((id) => ({ id, rel: inv.clone().multiply(worldOf(id)) }));
}

const mergedTargets = new WeakMap<Joint, { key: string; bvh: MeshBVH; positions: Float32Array }>();

/**
 * Das feste Teil für die Kontaktsuche – liegen Flächen auch auf Nachbarteilen, gehören diese mit
 * dazu (zusammengeführt im Koordinatensystem des festen Teils, zwischengespeichert).
 */
function fixedTarget(j: Joint, fixed: Part, helpers: { id: number; rel: THREE.Matrix4 }[]): { bvh: MeshBVH; positions: Float32Array } {
  if (!helpers.length) return { bvh: fixed.geom.boundsTree as MeshBVH, positions: positionsOf(fixed) };
  const key = JSON.stringify([fixed.id, helpers.map((h) => [h.id, h.rel.elements.map((x) => Math.round(x * 1e4))])]);
  const hit = mergedTargets.get(j);
  if (hit?.key === key) return hit;
  const lists = [positionsOf(fixed), ...helpers.map((h) => {
    const src = positionsOf(getPart(h.id)!);
    const out = new Float32Array(src.length);
    const v = new THREE.Vector3();
    for (let i = 0; i < src.length; i += 3) {
      v.set(src[i], src[i + 1], src[i + 2]).applyMatrix4(h.rel);
      out[i] = v.x; out[i + 1] = v.y; out[i + 2] = v.z;
    }
    return out;
  })];
  const positions = new Float32Array(lists.reduce((n, l) => n + l.length, 0));
  let o = 0;
  for (const l of lists) { positions.set(l, o); o += l.length; }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  const entry = { key, bvh: new MeshBVH(g), positions };
  mergedTargets.set(j, entry);
  return entry;
}

/** Oberflächenpunkte des beweglichen Teils in der Nähe der gewählten Flächen und Punkte (lokal). */
function regionSamples(moving: Part, feats: Feature[], markers: Marker[]): Sample[] {
  const regions: { c: Vec3; r: number }[] = [
    ...feats.map((f) => ({ c: featureCenter(f), r: featureRadius(f) * 1.6 + sceneScale() * 0.05 })),
    ...markers.map((m) => ({ c: m.p, r: sceneScale() * 0.25 })),
  ];
  const near = moving.samples.filter((s) =>
    regions.some((g) => Math.hypot(s.p[0] - g.c[0], s.p[1] - g.c[1], s.p[2] - g.c[2]) < g.r),
  );
  const list = near.length >= 50 ? near : moving.samples;
  const step = Math.max(1, Math.floor(list.length / 1500));
  return list.filter((_, i) => i % step === 0);
}

/**
 * Entfernt Alternativen, die bei symmetrischen Teilen geometrisch zum selben Ergebnis führen
 * (z. B. ein rechteckiges Dach, das um 180° gedreht genauso passt) – das ist keine echte Mehrdeutigkeit.
 */
function dropEquivalent(res: JointResult, moving: Part) {
  const bvh = moving.geom.boundsTree as MeshBVH;
  const tol = Math.max(sceneScale() * 0.01, res.solutions[0].rmsd * 3);
  const kept: typeof res.solutions = [];
  for (const sol of res.solutions) {
    const M = rigidToMatrix(sol.T);
    if (kept.some((k) => sameShape(bvh, moving.samples, rigidToMatrix(k.T), M, tol))) continue;
    kept.push(sol);
  }
  res.solutions = kept;
  res.ambiguous = kept.some((k) => k.nearTie);
}

const fmt = (x: number) => (x < 0.001 ? '< 0.001' : x.toFixed(x < 1 ? 3 : 2));
const nameOf = (id: number | null) => getPart(id)?.name ?? '–';

function alternativesOf(j: Joint): number {
  return hasFeatures(j) ? (j.cache?.count ?? 0) : (j.result?.solutions.length ?? 0);
}

function connect(j: Joint) {
  j.applied = true;
  solveAll();
  if (j.status?.kind === 'err') {
    j.applied = false;
    toast(j.status.text, 5000);
  }
  // Ansicht bewusst nicht verändern: man arbeitet meist weiter an derselben Stelle
  renderAll();
}

function refine(j: Joint) {
  if (!j.applied || !j.fixedSide) return;
  const fixed = getPart(j.fixedSide === 'a' ? j.a : j.b)!;
  const moving = getPart(j.fixedSide === 'a' ? j.b : j.a)!;
  const Mf = matrixToRigid(fixed.mesh.matrixWorld);
  const side = j.fixedSide;
  const contacts = [
    ...(side === 'a' ? j.ma : j.mb).map((m) => markerToWorld(m, fixed.mesh.matrixWorld).p),
    ...(side === 'a' ? j.fa : j.fb).map((f) => {
      const M = matrixToRigid((featurePart(j, side, f) ?? fixed).mesh.matrixWorld);
      return compose(M, { R: IDENTITY.R, t: featureCenter(f) }).t;
    }),
  ];
  const target = fixedTarget(j, fixed, helperRelations(j, side, fixed, (id) => getPart(id)!.mesh.matrixWorld));
  if (!contacts.length) return;
  const s = sceneScale();
  const current = matrixToRigid(moving.mesh.matrixWorld);
  const rmsd = j.result?.solutions[0]?.rmsd ?? 0;
  const res = refineICP(moving.samples, current, target.bvh, fixed.mesh.matrixWorld, contacts, {
    regionRadius: Math.max(s * 0.35, ...(j.fixedSide === 'a' ? j.fa : j.fb).map((f) => featureRadius(f) * 1.3)),
    maxPairDist: Math.max(rmsd * 4, s * 0.02),
  });
  if (!res) {
    toast('Feinausrichtung: zu wenig Kontaktfläche in der Nähe der Auswahl gefunden.');
    return;
  }
  // Korrektur als Bewegung im Koordinatensystem des festen Teils speichern
  const Dw = compose(res.T, invertRigid(current));
  j.adjust = compose(conjugate(invertRigid(Mf), Dw), j.adjust ?? IDENTITY);
  j.adjustInfo = `fein ausgerichtet (${res.pairs} Flächenpunkte, Rest ${fmt(res.rmsd)} mm)`;
  solveAll();
  renderAll();
}

// ---------------------------------------------------------------- Automatische Vorschläge

function candidatesOf(p: Part): Feature[] {
  p.candidates ??= findCandidateFaces(topologyOf(p));
  return p.candidates;
}

/**
 * Bewertet, welche Flächen von a und b zusammenpassen. Ebene Flächenpaare werden wirklich
 * aufeinandergelegt (Umriss-Abgleich) und auf Durchdringung geprüft.
 */
function proposePair(a: Part, b: Part, max = 6): Proposal[] {
  const s = sceneScale();
  const out: Proposal[] = [];
  const upA = new THREE.Vector3();
  for (const pr of candidatePairs(candidatesOf(a), candidatesOf(b), { max })) {
    let score = pr.shape;
    if (pr.fixed.kind === 'plane' && pr.moving.kind === 'plane') {
      const placed = placePlanePair(pr.fixed, pr.moving, true);
      if (!placed) continue;
      const size = Math.max(...pr.fixed.axes.size);
      score *= Math.exp(-placed.rms / (0.01 * size + 0.02));
      // Durchdringt das Teil in dieser Lage das andere, passt es so nicht
      const step = Math.max(1, Math.floor(b.samples.length / 400));
      const pen = contactScore(b.samples.filter((_, i) => i % step === 0), placed.T,
        { bvh: a.geom.boundsTree as MeshBVH, positions: positionsOf(a), matrix: IDENTITY },
        { contactDist: s * 0.01, maxDepth: s * 0.05 }).penetration;
      score *= Math.exp(-30 * pen);
      // Bei gleich guten Lagen bevorzugen: aufeinandersetzen (feste Fläche zeigt nach oben)
      upA.set(...pr.fixed.normal).transformDirection(a.mesh.matrixWorld);
      score *= 1 + 0.02 * upA.z;
    }
    if (score >= 0.5) out.push({ a, b, fa: pr.fixed, fb: pr.moving, score });
  }
  return out.sort((x, y) => y.score - x.score);
}

function applySuggestion(j: Joint, sg: Proposal) {
  j.a = sg.a.id;
  j.b = sg.b.id;
  j.fa = [sg.fa];
  j.fb = [sg.fb];
  j.ma = [];
  j.mb = [];
  markersChanged(j);
  j.applied = true;
}

function proposeJoint(j: Joint) {
  const a = getPart(j.a);
  const b = getPart(j.b);
  if (!a || !b) { toast('Wähle zuerst zwei Teile aus.'); return; }
  toast('Suche passende Flächen …', 20000);
  setTimeout(() => {
    const list = proposePair(a, b);
    if (!list.length) {
      toast('Keine passenden Flächen gefunden – bitte von Hand auswählen.', 5000);
      return;
    }
    j.proposals = list;
    j.proposalIdx = 0;
    applySuggestion(j, list[0]);
    solveAll();
    renderAll();
    toast(list.length > 1 ? `${list.length} Vorschläge gefunden – der beste ist angewendet.` : 'Passende Flächen gefunden und verbunden.', 4000);
  }, 30);
}

/**
 * Verbindet alle Teile automatisch: Für jedes Paar von Einheiten (Gruppen bzw. einzelne Teile) wird
 * der beste Vorschlag gesucht, dann ab dem Basisteil ein Baum aus den besten Verbindungen gebildet.
 */
async function connectAll() {
  const unitOf = new Map<number, number>();
  // Einheiten: Gruppen und bereits über Verbindungen zusammengesetzte Teile (Union-Find)
  const parent = new Map(parts.map((p) => [p.id, p.id]));
  const find = (x: number): number => (parent.get(x) === x ? x : (parent.set(x, find(parent.get(x)!)), parent.get(x)!));
  const join = (x: number, y: number) => parent.set(find(x), find(y));
  for (const p of parts) for (const q of groupOf(p)) join(p.id, q.id);
  for (const j of joints) if (j.applied && j.a !== null && j.b !== null && j.status?.kind !== 'err') join(j.a, j.b);
  for (const p of parts) unitOf.set(p.id, find(p.id));
  const units = [...new Set(unitOf.values())];
  if (units.length < 2) { toast('Alle Teile hängen schon zusammen.'); return; }
  const members = (u: number) => parts.filter((p) => unitOf.get(p.id) === u && p.visible);

  const btn = document.getElementById('connectAll') as HTMLButtonElement;
  btn.disabled = true;
  const edges: { u: number; v: number; sg: Proposal }[] = [];
  // Je Paar von Einheiten mehrere Kandidaten, falls die beste Fläche schon anderweitig vergeben wird
  const pairCount = (units.length * (units.length - 1)) / 2;
  let done = 0;
  try {
    for (let i = 0; i < units.length; i++) {
      for (let k = i + 1; k < units.length; k++) {
        for (const pa of members(units[i])) for (const pb of members(units[k]))
          for (const sg of proposePair(pa, pb, 4).slice(0, 3)) if (sg.score >= 0.6) edges.push({ u: units[i], v: units[k], sg });
        done++;
        toast(`Suche passende Flächen … ${Math.round((100 * done) / pairCount)} %`, 20000);
        await new Promise((r) => setTimeout(r, 0));
      }
    }
  } finally {
    btn.disabled = false;
  }
  // Baum der besten Verbindungen ab der Einheit des Basisteils (Prim)
  const start = unitOf.get(baseId ?? parts[0].id)!;
  const inTree = new Set([start]);
  const chosen: { sg: Proposal; fixedIsA: boolean }[] = [];
  // Jede Fläche nur einmal verwenden: zwei Teile auf derselben Schnittfläche wären übereinander
  const used = new Set<Feature>();
  for (;;) {
    let pick: (typeof edges)[number] | undefined;
    for (const e of edges)
      if (inTree.has(e.u) !== inTree.has(e.v) && !used.has(e.sg.fa) && !used.has(e.sg.fb) && (!pick || e.sg.score > pick.sg.score)) pick = e;
    if (!pick) break;
    used.add(pick.sg.fa);
    used.add(pick.sg.fb);
    chosen.push({ sg: pick.sg, fixedIsA: inTree.has(unitOf.get(pick.sg.a.id)!) });
    inTree.add(pick.u);
    inTree.add(pick.v);
  }
  // Leere, noch nicht verbundene Verbindungen ersetzen
  for (const j of [...joints]) if (!j.applied && j.ma.length + j.mb.length + j.fa.length + j.fb.length === 0) joints.splice(joints.indexOf(j), 1);
  for (const { sg, fixedIsA } of chosen) {
    const j = addJoint();
    // Teil 1 ist das Teil, das schon (über die Basis) festliegt
    applySuggestion(j, fixedIsA ? sg : { a: sg.b, b: sg.a, fa: sg.fb, fb: sg.fa, score: sg.score });
    j.proposals = [sg];
    j.proposalIdx = 0;
  }
  solveAll();
  fitView();
  renderAll();
  const missing = units.filter((u) => !inTree.has(u)).flatMap((u) => members(u).map((p) => p.name));
  toast(
    `${chosen.length} Verbindung(en) angelegt.` + (missing.length ? ` Ohne passenden Partner: ${missing.join(', ')}.` : ''),
    8000,
  );
}

// ---------------------------------------------------------------- Gizmo (von Hand bewegen/drehen)

const tc = new TransformControls(camera, renderer.domElement);
tc.setSpace('local');
tc.setSize(1.25);
tc.setColors(0xff4d5e, 0x3ddc84, 0x4d8dff, 0xffd23f);
scene.add(tc.getHelper());
/** Durch die Verbindung gesperrte Gizmo-Achsen (lokal X, Y, Z): grau und nicht greifbar. */
let gizmoLocked: [boolean, boolean, boolean] = [false, false, false];

/**
 * Gizmo gut sichtbar und immer vollständig: volle, kräftige Drehringe statt Halbkreisen, kräftigere
 * Verschiebeachsen. Gesperrte Richtungen bleiben grau sichtbar, statt zu verschwinden.
 */
function setupGizmoLook() {
  type Handle = THREE.Mesh<THREE.BufferGeometry, THREE.MeshBasicMaterial>;
  const g = (tc as unknown as {
    _gizmo: THREE.Object3D & { gizmo: Record<string, THREE.Object3D>; picker: Record<string, THREE.Object3D> };
  })._gizmo;
  const ring = (axis: string, tube: number) => {
    const geo = new THREE.TorusGeometry(0.5, tube, 8, 128); // Ring um Z
    if (axis === 'X') geo.rotateY(Math.PI / 2);
    if (axis === 'Y') geo.rotateX(Math.PI / 2);
    return geo;
  };
  const shaft = (axis: string) => {
    const geo = new THREE.CylinderGeometry(0.014, 0.014, 1, 8); // entlang Y, von -0,5 bis 0,5
    if (axis === 'X') geo.rotateZ(-Math.PI / 2);
    if (axis === 'Z') geo.rotateX(Math.PI / 2);
    return geo;
  };
  const isAxis = (n: string) => n === 'X' || n === 'Y' || n === 'Z';
  for (const h of g.gizmo.rotate.children as Handle[]) if (isAxis(h.name)) h.geometry = ring(h.name, 0.016);
  for (const h of g.picker.rotate.children as Handle[]) if (isAxis(h.name)) h.geometry = ring(h.name, 0.09);
  for (const h of g.gizmo.translate.children as Handle[]) {
    const p = (h.geometry as THREE.CylinderGeometry).parameters;
    if (isAxis(h.name) && h.geometry.type === 'CylinderGeometry' && p?.radiusTop === p?.radiusBottom) h.geometry = shaft(h.name);
    else if (isAxis(h.name) && h.geometry.type === 'CylinderGeometry') {
      // Pfeilspitzen an Ort und Stelle vergrößern
      h.geometry.computeBoundingBox();
      const c = h.geometry.boundingBox!.getCenter(new THREE.Vector3());
      h.geometry.translate(-c.x, -c.y, -c.z).scale(1.5, 1.5, 1.5).translate(c.x, c.y, c.z);
    }
  }
  // Gesperrte Richtungen: dünne gestrichelte Führungslinien statt der Griffe – sichtbar, aber erkennbar nicht greifbar
  const lockMat = new THREE.MeshBasicMaterial({ color: 0x9aa0aa, transparent: true, opacity: 0.8, depthTest: false, depthWrite: false, toneMapped: false });
  const orient = (geo: THREE.BufferGeometry, axis: string, from: 'y' | 'z') => {
    if (from === 'y') { if (axis === 'X') geo.rotateZ(-Math.PI / 2); if (axis === 'Z') geo.rotateX(Math.PI / 2); }
    else { if (axis === 'X') geo.rotateY(Math.PI / 2); if (axis === 'Y') geo.rotateX(Math.PI / 2); }
    return geo;
  };
  const dashedLine = (axis: string) => {
    const dashes: THREE.BufferGeometry[] = [];
    for (let y = -0.6; y < 0.6; y += 0.1) dashes.push(new THREE.CylinderGeometry(0.007, 0.007, 0.055, 6).translate(0, y + 0.0275, 0));
    return orient(mergeGeometries(dashes)!, axis, 'y');
  };
  const dashedRing = (axis: string) => {
    const n = 32;
    const dashes: THREE.BufferGeometry[] = [];
    for (let k = 0; k < n; k++) dashes.push(new THREE.TorusGeometry(0.5, 0.007, 4, 4, (Math.PI * 2) / n * 0.55).rotateZ((k * Math.PI * 2) / n));
    return orient(mergeGeometries(dashes)!, axis, 'z');
  };
  for (const axis of ['X', 'Y', 'Z']) {
    const line = new THREE.Mesh(dashedLine(axis), lockMat.clone());
    line.name = `lock${axis}`;
    line.renderOrder = Infinity;
    g.gizmo.translate.add(line);
    const ringGuide = new THREE.Mesh(dashedRing(axis), lockMat.clone());
    ringGuide.name = `lock${axis}`;
    ringGuide.renderOrder = Infinity;
    g.gizmo.rotate.add(ringGuide);
  }
  const lockedName = (name: string) => ['X', 'Y', 'Z'].some((a, i) => gizmoLocked[i] && name.includes(a));
  const anyLocked = () => gizmoLocked.some(Boolean);
  const free = (name: string) => name === 'E' || name === 'XYZE' || name === 'XYZ';
  const base = g.updateMatrixWorld.bind(g);
  g.updateMatrixWorld = (force?: boolean) => {
    base(force);
    const mode = tc.getMode();
    if (mode !== 'translate' && mode !== 'rotate') return;
    for (const h of g.gizmo[mode].children) {
      if (h.name.startsWith('lock')) h.visible = gizmoLocked['XYZ'.indexOf(h.name[4])];
      // Freies Drehen/Verschieben in Blickrichtung nur, wenn nichts gesperrt ist
      else if ((free(h.name) && anyLocked()) || lockedName(h.name)) h.visible = false;
    }
    for (const h of g.picker[mode].children) if ((free(h.name) && anyLocked()) || lockedName(h.name)) h.visible = false;
  };
}
setupGizmoLook();
const gizmoAnchor = new THREE.Object3D();
gizmoAnchor.matrixAutoUpdate = true;
scene.add(gizmoAnchor);
let gizmoTarget: { kind: 'joint' | 'part'; id: number; assembly?: boolean } | null = null;
let gizmoMode: 'translate' | 'rotate' = 'rotate';
let gizmoFreeOnly = true;
let gizmoSnapDeg = 0;
let dragStart: { anchor: THREE.Matrix4; adjust: Rigid; layouts: Map<number, THREE.Matrix4> } | null = null;
/** Klicks, die auf dem Gizmo landen, dürfen keine Markierungen setzen. */
let suppressClick = false;

tc.addEventListener('dragging-changed', (e) => {
  controls.enabled = !e.value;
  if (e.value) {
    suppressClick = true;
    const j = gizmoTarget?.kind === 'joint' ? joints.find((x) => x.id === gizmoTarget!.id) : undefined;
    const p = gizmoTarget?.kind === 'part' ? getPart(gizmoTarget.id) : undefined;
    gizmoAnchor.updateMatrixWorld(true);
    dragStart = {
      anchor: gizmoAnchor.matrixWorld.clone(),
      adjust: j?.adjust ?? IDENTITY,
      layouts: new Map((p ? groupOf(p) : []).map((q) => [q.id, q.layout.clone()])),
    };
  } else {
    dragStart = null;
    renderAll();
  }
});

tc.addEventListener('objectChange', () => {
  if (!dragStart || !gizmoTarget) return;
  gizmoAnchor.updateMatrixWorld(true);
  const D = gizmoAnchor.matrixWorld.clone().multiply(dragStart.anchor.clone().invert());
  if (gizmoTarget.kind === 'part') {
    for (const [id, L] of dragStart.layouts) getPart(id)?.layout.copy(D.clone().multiply(L));
  } else {
    const j = joints.find((x) => x.id === gizmoTarget!.id);
    const fixed = j && getPart(j.fixedSide === 'a' ? j.a : j.b);
    if (!j || !fixed) return;
    const Mf = matrixToRigid(fixed.mesh.matrixWorld);
    j.adjust = compose(conjugate(invertRigid(Mf), matrixToRigid(D)), dragStart.adjust);
    j.adjustInfo = 'von Hand angepasst';
  }
  solveAll();
  renderLight();
});

/** Freie Bewegungen, die das Gizmo zeigen soll (Weltkoordinaten). Leer = alles frei. */
function gizmoMotions(): FreeMotion[] | null {
  if (!gizmoTarget || gizmoTarget.kind === 'part' || !gizmoFreeOnly) return null;
  return joints.find((x) => x.id === gizmoTarget!.id)?.free ?? [];
}

function placeGizmo() {
  if (!gizmoTarget) return;
  let moving: Part | undefined;
  let pivot: THREE.Vector3 | null = null;
  let zAxis: THREE.Vector3 | null = null;
  let xHint: THREE.Vector3 | null = null;
  if (gizmoTarget.kind === 'part') {
    moving = getPart(gizmoTarget.id);
  } else {
    const j = joints.find((x) => x.id === gizmoTarget!.id);
    if (!j?.applied || !j.fixedSide) { closeGizmo(); return; }
    moving = getPart(j.fixedSide === 'a' ? j.b : j.a);
    const free = j.free ?? [];
    const rot = free.find((m) => m.kind === 'rotate');
    const trans = free.filter((m) => m.kind === 'translate');
    if (rot && rot.kind === 'rotate') {
      pivot = new THREE.Vector3(...rot.point);
      zAxis = new THREE.Vector3(...rot.axis);
    } else if (trans.length === 1) zAxis = new THREE.Vector3(...trans[0].axis);
    else if (trans.length === 2) zAxis = new THREE.Vector3(...trans[0].axis).cross(new THREE.Vector3(...trans[1].axis));
    const t0 = trans[0];
    if (t0 && zAxis && Math.abs(new THREE.Vector3(...t0.axis).normalize().dot(zAxis.clone().normalize())) < 0.99)
      xHint = new THREE.Vector3(...t0.axis);
    if (!pivot && moving) {
      // Mitte der gewählten Flächen/Punkte des beweglichen Teils
      const side = j.fixedSide === 'a' ? 'b' : 'a';
      const pts = [...(side === 'a' ? j.fa : j.fb).map(featureCenter), ...(side === 'a' ? j.ma : j.mb).map((m) => m.p)];
      if (pts.length) {
        const c = pts.reduce((a, p) => [a[0] + p[0] / pts.length, a[1] + p[1] / pts.length, a[2] + p[2] / pts.length], [0, 0, 0]);
        pivot = new THREE.Vector3(...c).applyMatrix4(moving.mesh.matrixWorld);
      }
    }
  }
  if (!moving) { closeGizmo(); return; }
  if (!pivot && gizmoTarget.assembly) {
    // Drehpunkt in der Mitte des ganzen zusammengesetzten Modells
    const box = new THREE.Box3();
    for (const p of assemblyOf(moving)) box.expandByObject(p.mesh);
    pivot = box.getCenter(new THREE.Vector3());
  }
  if (!pivot) pivot = new THREE.Box3().setFromObject(moving.mesh).getCenter(new THREE.Vector3());
  const z = (zAxis ?? new THREE.Vector3(0, 0, 1)).normalize();
  let x = xHint ? xHint.clone().addScaledVector(z, -xHint.dot(z)) : new THREE.Vector3(1, 0, 0).addScaledVector(z, -z.x);
  if (x.lengthSq() < 1e-6) x = new THREE.Vector3(0, 1, 0).addScaledVector(z, -z.y);
  x.normalize();
  const y = z.clone().cross(x);
  gizmoAnchor.matrix.makeBasis(x, y, z).setPosition(pivot);
  gizmoAnchor.matrix.decompose(gizmoAnchor.position, gizmoAnchor.quaternion, gizmoAnchor.scale);
  gizmoAnchor.updateMatrixWorld(true);
  tc.attach(gizmoAnchor);
  updateGizmoAxes();
}

/** Zeigt nur die Achsen, entlang/um die sich das Teil laut Verbindung noch bewegen darf. */
function updateGizmoAxes() {
  tc.setMode(gizmoMode);
  tc.setRotationSnap(gizmoSnapDeg ? THREE.MathUtils.degToRad(gizmoSnapDeg) : null);
  const motions = gizmoMotions();
  const hint = document.getElementById('gizmoHint')!;
  tc.showX = tc.showY = tc.showZ = true;
  if (!motions) {
    gizmoLocked = [false, false, false];
    hint.textContent = '';
    return;
  }
  const axes = [new THREE.Vector3(), new THREE.Vector3(), new THREE.Vector3()];
  gizmoAnchor.matrixWorld.extractBasis(axes[0], axes[1], axes[2]);
  const wanted = motions.filter((m) => (gizmoMode === 'rotate' ? m.kind === 'rotate' : m.kind === 'translate'));
  // Orthonormalbasis der erlaubten Richtungen, dann prüfen, welche Gizmo-Achsen darin liegen
  const basis: THREE.Vector3[] = [];
  for (const m of wanted) {
    const v = new THREE.Vector3(...m.axis).normalize();
    for (const b of basis) v.addScaledVector(b, -v.dot(b));
    if (v.length() > 1e-3) basis.push(v.normalize());
  }
  const inSpan = (a: THREE.Vector3) => Math.hypot(...basis.map((b) => a.dot(b))) > 0.98;
  gizmoLocked = axes.map((a) => !inSpan(a)) as [boolean, boolean, boolean];
  const what = gizmoMode === 'rotate' ? 'Drehen' : 'Verschieben';
  hint.textContent = gizmoLocked.every(Boolean)
    ? `${what} ist durch die Verbindung festgelegt (gestrichelt). Für freies ${what} den Haken bei „Nur freie Richtungen" entfernen.`
    : gizmoLocked.some(Boolean)
      ? 'Farbig: frei beweglich · grau gestrichelt: durch die Verbindung gesperrt.'
      : '';
}

/** Alle Teile, die (über Verbindungen und Gruppen) zusammen mit diesem platziert werden. */
function assemblyOf(p: Part): Part[] {
  const seed = seedOf.get(p.id);
  return parts.filter((q) => seedOf.get(q.id) === seed);
}

function openGizmo(target: { kind: 'joint' | 'part'; id: number; assembly?: boolean }, mode?: 'translate' | 'rotate') {
  gizmoTarget = target;
  if (target.kind === 'joint') {
    const j = joints.find((x) => x.id === target.id);
    const free = j?.free ?? [];
    gizmoMode = mode ?? (free.some((m) => m.kind === 'rotate') ? 'rotate' : 'translate');
  } else gizmoMode = mode ?? 'translate';
  document.getElementById('gizmoBar')!.hidden = false;
  placeGizmo();
  renderGizmoBar();
}

function closeGizmo() {
  gizmoTarget = null;
  tc.detach();
  document.getElementById('gizmoBar')!.hidden = true;
}

function renderGizmoBar() {
  const bar = document.getElementById('gizmoBar')!;
  if (!gizmoTarget) return;
  const label =
    gizmoTarget.assembly
      ? 'Gesamtes Modell bewegen'
      : gizmoTarget.kind === 'part'
      ? `„${nameOf(gizmoTarget.id)}" bewegen`
      : `Verbindung ${joints.findIndex((x) => x.id === gizmoTarget!.id) + 1} anpassen`;
  bar.querySelector('[data-g="title"]')!.textContent = label;
  bar.querySelectorAll<HTMLButtonElement>('[data-g-mode]').forEach((b) => b.classList.toggle('on', b.dataset.gMode === gizmoMode));
  (bar.querySelector('#gizmoFree') as HTMLInputElement).checked = gizmoFreeOnly;
  (bar.querySelector('[data-g="free"]') as HTMLElement).hidden = gizmoTarget.kind === 'part';
  (bar.querySelector('#gizmoSnap') as HTMLSelectElement).value = String(gizmoSnapDeg);
  (bar.querySelector('[data-g="reset"]') as HTMLElement).hidden = gizmoTarget.kind === 'part';
}

// ---------------------------------------------------------------- Markierungen und Flächen (3D)

function labelSprite(text: string, color: THREE.Color): THREE.Sprite {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d')!;
  g.fillStyle = `#${color.getHexString()}`;
  g.beginPath();
  g.arc(32, 32, 28, 0, Math.PI * 2);
  g.fill();
  g.lineWidth = 4;
  g.strokeStyle = '#ffffff';
  g.stroke();
  g.fillStyle = '#ffffff';
  g.font = `bold ${text.length > 1 ? 24 : 30}px system-ui, sans-serif`;
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(text, 32, 34);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  s.renderOrder = 20;
  return s;
}

/** Teilgeometrie aus ausgewählten Dreiecken (lokale Koordinaten). */
function subGeometry(part: Part, tris: Iterable<number>): THREE.BufferGeometry {
  const src = positionsOf(part);
  const list = [...tris];
  const out = new Float32Array(list.length * 9);
  list.forEach((t, k) => out.set(src.subarray(t * 9, t * 9 + 9), k * 9));
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(out, 3));
  return g;
}

function disposeGroup(group: THREE.Group) {
  for (const o of [...group.children]) {
    group.remove(o);
    o.traverse((x) => {
      const m = x as THREE.Mesh;
      m.geometry?.dispose();
      const mat = m.material as THREE.Material & { map?: THREE.Texture };
      mat?.map?.dispose();
      mat?.dispose();
    });
  }
}

function rebuildMarkers() {
  disposeGroup(markerGroup);
  disposeGroup(faceGroup);
  const sphere = new THREE.SphereGeometry(1, 16, 12);
  for (const j of joints) {
    const active = j.id === activeJointId;
    const movingSide = j.fixedSide ? (j.fixedSide === 'a' ? 'b' : 'a') : null;
    for (const side of ['a', 'b'] as Side[]) {
      const own = getPart(side === 'a' ? j.a : j.b);
      const list = own?.visible ? (side === 'a' ? j.ma : j.mb) : [];
      const part = own!;
      // Auf der bewegten Seite nach dem Verbinden die erkannte Zuordnung anzeigen
      list.forEach((m, i) => {
        const w = markerToWorld(m, part.mesh.matrixWorld);
        const anchor = new THREE.Vector3(...w.p);
        const normal = new THREE.Vector3(...w.n);
        const dot = new THREE.Mesh(
          sphere,
          new THREE.MeshBasicMaterial({ color: j.color, transparent: !active, opacity: active ? 1 : 0.45 }),
        );
        dot.userData = { joint: j.id, side, index: i, kind: 'point', px: 5, anchor, normal, lift: 0 };
        markerGroup.add(dot);

        const num = side === movingSide && j.perm ? j.perm[i] + 1 : i + 1;
        const label = labelSprite(String(num), j.color);
        label.material.opacity = active ? 1 : 0.5;
        // Beschriftung etwas von der Oberfläche abheben
        label.userData = { ...dot.userData, px: 22, lift: side === 'a' ? 16 : 26 };
        markerGroup.add(label);
      });

      // Ausgewählte Flächen farbig hervorheben und beschriften
      const feats = side === 'a' ? j.fa : j.fb;
      const counters: Record<string, number> = {};
      feats.forEach((f, i) => {
        const group = f.kind === 'cylinder' ? 'round' : f.kind === 'edge' ? 'edge' : 'flat';
        counters[group] = (counters[group] ?? 0) + 1;
        const part = featurePart(j, side, f);
        if (!part?.visible) return;
        for (const piece of f.pieces ?? [f]) {
          const on = featurePart(j, side, piece);
          if (!on?.visible) continue;
          const overlay = piece.kind === 'edge'
            ? new THREE.Mesh(edgeTube(piece), new THREE.MeshBasicMaterial({ color: j.color, transparent: true, opacity: active ? 1 : 0.5, depthTest: false }))
            : new THREE.Mesh(
              subGeometry(on, piece.tris),
              new THREE.MeshBasicMaterial({
                color: j.color, transparent: true, opacity: active ? 0.55 : 0.25, side: THREE.DoubleSide, depthWrite: false,
                polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1,
              }),
            );
          if (piece.kind === 'edge') overlay.renderOrder = 12;
          overlay.matrixAutoUpdate = false;
          overlay.matrix.copy(on.mesh.matrixWorld);
          overlay.userData = { joint: j.id, side, index: i, kind: 'face' };
          faceGroup.add(overlay);
        }

        const label = labelSprite(`${featureTag(f)}${counters[group]}`, j.color);
        label.material.opacity = active ? 1 : 0.5;
        const [ca, na] = featureAnchor(part, f);
        const c = new THREE.Vector3(...ca).applyMatrix4(part.mesh.matrixWorld);
        const n = new THREE.Vector3(...na).transformDirection(part.mesh.matrixWorld);
        label.userData = { joint: j.id, side, index: i, kind: 'face', px: 24, anchor: c, normal: n, lift: side === 'a' ? 14 : 34 };
        markerGroup.add(label);
      });
    }
  }
  scaleMarkers();
}

// ---------------------------------------------------------------- Prüfansicht

const CHECK_COLORS = { pen: 0xe5484d, touch: 0x30a46c, gap: 0xf5a524 };

/** Laufende Prüfung; eine neue Prüfung (nach einer Änderung) bricht die alte ab. */
let checkGen = 0;
/** Höchstens so viele Dreiecke je Teil werden geprüft (darüber eine gleichmäßige Stichprobe). */
const CHECK_BUDGET = 300_000;

/**
 * Prüft alle sichtbaren Teile paarweise und färbt Berührung, Spalt und Durchdringung ein. Läuft in
 * Abschnitten, damit die Oberfläche bei großen Netzen bedienbar bleibt.
 */
async function runCheck() {
  const gen = ++checkGen;
  disposeGroup(checkGroup);
  const el = document.getElementById('checkResult')!;
  if (!checkOn) {
    el.innerHTML = '';
    return;
  }
  // Färbung etwa im Raster von 1–2 % der Teilgröße
  const opts = { contactTol: 0.03, gapTol: 0.6, maxEdge: Math.min(3, Math.max(0.4, sceneScale() * 0.015)) };
  const visible = parts.filter((p) => p.visible);
  const boxes = new Map(visible.map((p) => [p.id, new THREE.Box3().setFromObject(p.mesh)]));
  const target = (p: Part) => {
    const b = boxes.get(p.id)!;
    return { bvh: p.geom.boundsTree as MeshBVH, matrix: matrixToRigid(p.mesh.matrixWorld), box: { min: v3(b.min), max: v3(b.max) } };
  };
  const show = (p: Part, kind: keyof typeof CHECK_COLORS, tris: Float32Array) => {
    if (!tris.length) return;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(tris, 3));
    const overlay = new THREE.Mesh(g, new THREE.MeshBasicMaterial({
      color: CHECK_COLORS[kind], transparent: true, opacity: 0.85, side: THREE.DoubleSide, depthWrite: false,
      polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1,
    }));
    // Durchdringung zuletzt zeichnen, damit sie sichtbar bleibt
    overlay.renderOrder = kind === 'pen' ? 3 : kind === 'touch' ? 2 : 1;
    overlay.matrixAutoUpdate = false;
    overlay.matrix.copy(p.mesh.matrixWorld);
    checkGroup.add(overlay);
  };
  const pairs: [Part, Part][] = [];
  for (let i = 0; i < visible.length; i++)
    for (let k = i + 1; k < visible.length; k++)
      if (boxes.get(visible[i].id)!.clone().expandByScalar(opts.gapTol).intersectsBox(boxes.get(visible[k].id)!)) pairs.push([visible[i], visible[k]]);
  const total = pairs.reduce((s, [A, B]) => s + Math.min(CHECK_BUDGET, positionsOf(A).length / 9) + Math.min(CHECK_BUDGET, positionsOf(B).length / 9), 0) || 1;
  let done = 0;
  let sampled = false;
  let last = performance.now();
  const rows: { text: string; bad: boolean }[] = [];

  /** Ein Teil gegen ein anderes, in Abschnitten. Liefert null, wenn die Prüfung abgebrochen wurde. */
  const directed = async (P: Part, Q: Part) => {
    const pos = positionsOf(P);
    const n = pos.length / 9;
    const stride = Math.max(1, Math.ceil(n / CHECK_BUDGET));
    if (stride > 1) sampled = true;
    const acc = newAccumulator();
    const M = matrixToRigid(P.mesh.matrixWorld);
    const T = target(Q);
    const chunk = 20_000 * stride;
    for (let from = 0; from < n; from += chunk) {
      checkRange(pos, M, T, opts, acc, from, Math.min(n, from + chunk), stride);
      done += Math.ceil((Math.min(n, from + chunk) - from) / stride);
      if (performance.now() - last > 120) {
        el.innerHTML = `<div class="help" style="margin:0">Prüfe Passung … ${Math.round((100 * done) / total)} %</div>`;
        await new Promise((r) => setTimeout(r, 0));
        last = performance.now();
        if (gen !== checkGen) return null;
      }
    }
    return finishCheck(acc);
  };

  for (const [A, B] of pairs) {
    const ra = await directed(A, B);
    const rb = ra && (await directed(B, A));
    if (!ra || !rb) return;
    for (const [p, r] of [[A, ra], [B, rb]] as const) {
      show(p, 'gap', r.gap);
      show(p, 'touch', r.touching);
      show(p, 'pen', r.penetrating);
    }
    const touch = ra.touching.length + rb.touching.length > 0;
    const pen = Math.max(ra.maxPen, rb.maxPen);
    const gap = ra.gap.length + rb.gap.length > 0;
    if (!touch && !pen && !gap) continue;
    const bits: string[] = [];
    if (touch) bits.push('liegen an');
    if (gap) {
      const lo = Math.min(ra.minGap, rb.minGap);
      const hi = Math.max(ra.maxGap, rb.maxGap);
      const typ = typicalGap([ra, rb]);
      bits.push(hi - lo < 0.01 ? `Spalt ${mmText(typ)} mm` : `Spalt meist ${mmText(typ)} mm (${mmText(lo)}–${mmText(hi)})`);
    }
    if (pen) bits.push(`dringen bis ${mmText(pen)} mm ineinander ein`);
    rows.push({ text: `${A.name} ↔ ${B.name}: ${bits.join(' · ')}`, bad: pen > 0 });
  }
  if (gen !== checkGen) return;
  const legend = `<div class="legend"><span style="--c:#30a46c">liegt an</span><span style="--c:#f5a524">Spalt bis 0,6 mm</span><span style="--c:#e5484d">dringt ein</span></div>`;
  const note = sampled ? '<div class="help" style="margin:0">Sehr feines Netz: geprüft wurde eine gleichmäßige Stichprobe der Dreiecke.</div>' : '';
  el.innerHTML = legend + (rows.length
    ? `<ul>${rows.map((r) => `<li class="${r.bad ? 'bad' : ''}">${esc(r.text)}</li>`).join('')}</ul>`
    : '<div class="help" style="margin:0">Keine Teile berühren sich.</div>') + note;
}

const mmText = (x: number) => x.toFixed(x < 0.1 ? 3 : 2).replace('.', ',');

// ---------------------------------------------------------------- Auswahl per Maus

const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();
let downPos: { x: number; y: number } | null = null;

function setNdc(e: PointerEvent | MouseEvent) {
  const rect = renderer.domElement.getBoundingClientRect();
  ndc.set(((e.clientX - rect.left) / rect.width) * 2 - 1, -((e.clientY - rect.top) / rect.height) * 2 + 1);
  raycaster.setFromCamera(ndc, camera);
}

interface Hit {
  part: Part;
  local: Marker;
  world: THREE.Vector3;
  tri: number;
  /** Modellgröße eines Bildschirmpixels am Trefferpunkt. */
  worldPerPx: number;
  /** Ungerasteter Trefferpunkt (lokal). */
  raw: Vec3;
}

/** Trifft die Oberfläche eines Teils; rastet im Punktmodus optional an der nächsten Ecke ein. */
function pick(e: PointerEvent | MouseEvent): Hit | null {
  setNdc(e);
  const meshes = parts.filter((p) => p.visible).map((p) => p.mesh);
  const hit = raycaster.intersectObjects(meshes, false)[0];
  if (!hit || !hit.face || hit.faceIndex == null) return null;
  const part = getPart(hit.object.userData.partId)!;
  const inv = part.mesh.matrixWorld.clone().invert();
  const local = hit.point.clone().applyMatrix4(inv);
  // Außennormale laut Dreiecks-Umlaufsinn (nicht zur Kamera gedreht)
  const n = hit.face.normal.clone();
  const raw = v3(local);
  const snap = (document.getElementById('snap') as HTMLInputElement).checked;
  // Fangradius in Bildschirmpixeln, umgerechnet in Modellgröße am Trefferpunkt
  const rect = renderer.domElement.getBoundingClientRect();
  const worldPerPx = (2 * hit.distance * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2))) / rect.height;
  if (snap && pickMode === 'point') {
    const sphere = new THREE.Sphere(local.clone(), SNAP_PX * worldPerPx);
    let best: THREE.Vector3 | null = null;
    let bd = sphere.radius;
    (part.geom.boundsTree as MeshBVH).shapecast({
      intersectsBounds: (box) => box.intersectsSphere(sphere),
      intersectsTriangle: (tri) => {
        for (const v of [tri.a, tri.b, tri.c]) {
          const d = v.distanceTo(local);
          if (d < bd) { bd = d; best = v.clone(); }
        }
      },
    });
    if (best) local.copy(best);
  }
  const world = local.clone().applyMatrix4(part.mesh.matrixWorld);
  // Die BVH meldet faceIndex in ihrer eigenen (umsortierten) Reihenfolge – das Dreieck im Netz ergibt
  // sich aus den Eckpunkten, die bei nicht indizierten Netzen zu je dritt zu einem Dreieck gehören
  return { part, local: { p: v3(local), n: v3(n.normalize()) }, world, tri: Math.floor(hit.face.a / 3), worldPerPx, raw };
}

/** Darf ein Klick auf dieses Teil in der aktiven Verbindung landen? */
function usableFor(j: Joint | undefined, part: Part) {
  return !!j && (j.a === part.id || j.b === part.id || j.a === null || j.b === null);
}

function updateHover(h: Hit | null) {
  hoverLine.visible = false;
  if (pickMode === 'brush') return;
  if (pickMode === 'point' || !h || !(usableFor(activeJoint(), h.part) || helperSide(activeJoint(), h.part))) {
    hoverMesh.visible = false;
    hoverKey = '';
    hoverTris = null;
    return;
  }
  if (pickMode === 'edge') {
    hoverMesh.visible = false;
    hoverKey = '';
    hoverTris = null;
    const edge = edgeAt(h);
    if (!edge) return;
    hoverLine.geometry.dispose();
    hoverLine.geometry = edgeTube(edge);
    hoverLine.matrix.copy(h.part.mesh.matrixWorld);
    (hoverLine.material as THREE.MeshBasicMaterial).color.copy(activeJoint()!.color).lerp(new THREE.Color(1, 1, 1), 0.3);
    hoverLine.visible = true;
    return;
  }
  const key = `${h.part.id}`;
  if (key === hoverKey && hoverTris?.has(h.tri)) return;
  const f = detectFeature(topologyOf(h.part), h.tri, FEATURE_OPTS);
  hoverKey = key;
  hoverTris = new Set(f.tris);
  hoverMesh.geometry.dispose();
  hoverMesh.geometry = subGeometry(h.part, f.tris);
  hoverMesh.matrix.copy(h.part.mesh.matrixWorld);
  hoverMesh.visible = true;
  const j = activeJoint()!;
  (hoverMesh.material as THREE.MeshBasicMaterial).color.copy(j.color).lerp(new THREE.Color(1, 1, 1), 0.4);
}

// Strg/⌘ + Ziehen: Teil (mit allem, was daran hängt) frei über die Bauplatte schieben
let platformDrag: { id: number; plane: THREE.Plane; start: THREE.Vector3; layouts: Map<number, THREE.Matrix4> } | null = null;
renderer.domElement.addEventListener('pointerdown', (e) => {
  if (e.button !== 0 || !(e.ctrlKey || e.metaKey)) return;
  const h = pick(e);
  if (!h) return;
  // Vor OrbitControls und Gizmo abfangen: die Ansicht soll sich dabei nicht drehen
  e.stopImmediatePropagation();
  e.preventDefault();
  const seed = seedOf.get(h.part.id);
  const members = parts.filter((q) => seedOf.get(q.id) === seed);
  platformDrag = {
    id: e.pointerId,
    plane: new THREE.Plane(new THREE.Vector3(0, 0, 1), -h.world.z),
    start: h.world.clone(),
    layouts: new Map(members.map((q) => [q.id, q.layout.clone()])),
  };
  renderer.domElement.setPointerCapture(e.pointerId);
  renderer.domElement.style.cursor = 'grabbing';
  hideTip();
}, { capture: true });
renderer.domElement.addEventListener('pointermove', (e) => {
  if (!platformDrag || e.pointerId !== platformDrag.id) return;
  e.stopImmediatePropagation();
  setNdc(e);
  const at = raycaster.ray.intersectPlane(platformDrag.plane, new THREE.Vector3());
  if (!at) return;
  const d = at.sub(platformDrag.start);
  const T = new THREE.Matrix4().makeTranslation(d.x, d.y, 0);
  for (const [id, L] of platformDrag.layouts) getPart(id)?.layout.copy(T.clone().multiply(L));
  solveAll();
  renderLight();
}, { capture: true });
const endPlatformDrag = (e: PointerEvent) => {
  if (!platformDrag || e.pointerId !== platformDrag.id) return;
  e.stopImmediatePropagation();
  platformDrag = null;
  renderer.domElement.style.cursor = 'default';
  renderAll();
};
renderer.domElement.addEventListener('pointerup', endPlatformDrag, { capture: true });
renderer.domElement.addEventListener('pointercancel', endPlatformDrag, { capture: true });

// Pinsel: mit gedrückter linker Maustaste Dreiecke bemalen, beim Loslassen wird daraus eine Fläche
let painting: { id: number; part: Part; side: Side; on?: number; tris: Set<number> } | null = null;
/** Dreiecke im Pinselradius um den Treffer, zusammenhängend mit dem getroffenen Dreieck. */
function paintAt(h: Hit) {
  if (!painting || h.part !== painting.part) return;
  const topo = topologyOf(h.part);
  const r = BRUSH_PX * h.worldPerPx;
  const p = h.raw;
  const stack = [h.tri];
  const seen = new Set<number>(stack);
  while (stack.length && seen.size < 20000) {
    const t = stack.pop()!;
    const c = [topo.centroids[3 * t], topo.centroids[3 * t + 1], topo.centroids[3 * t + 2]];
    if (Math.hypot(c[0] - p[0], c[1] - p[1], c[2] - p[2]) > r && t !== h.tri) continue;
    painting.tris.add(t);
    for (let k = 0; k < 3; k++) {
      const n = topo.neighbors[3 * t + k];
      if (n >= 0 && !seen.has(n)) { seen.add(n); stack.push(n); }
    }
  }
  hoverMesh.geometry.dispose();
  hoverMesh.geometry = subGeometry(h.part, [...painting.tris]);
  hoverMesh.matrix.copy(h.part.mesh.matrixWorld);
  (hoverMesh.material as THREE.MeshBasicMaterial).color.copy(activeJoint()?.color ?? new THREE.Color(1, 1, 1)).lerp(new THREE.Color(1, 1, 1), 0.2);
  hoverMesh.visible = true;
}
renderer.domElement.addEventListener('pointerdown', (e) => {
  if (pickMode !== 'brush' || e.button !== 0 || e.ctrlKey || e.metaKey || e.shiftKey || e.altKey) return;
  const h = pick(e);
  const j = activeJoint();
  if (!h || !j || (tc.object && tc.axis)) return;
  let side: Side | null = j.a === h.part.id ? 'a' : j.b === h.part.id ? 'b' : null;
  const helper = side ? null : helperSide(j, h.part);
  side ??= helper;
  if (!side) return;
  // Beginnt der Strich auf einem Teil der Verbindung, wird gemalt statt die Ansicht gedreht
  e.stopImmediatePropagation();
  painting = { id: e.pointerId, part: h.part, side, on: helper ? h.part.id : undefined, tris: new Set() };
  renderer.domElement.setPointerCapture(e.pointerId);
  paintAt(h);
}, { capture: true });
renderer.domElement.addEventListener('pointermove', (e) => {
  if (!painting || e.pointerId !== painting.id) return;
  e.stopImmediatePropagation();
  const h = pick(e);
  if (h) paintAt(h);
}, { capture: true });
const endPainting = (e: PointerEvent) => {
  if (!painting || e.pointerId !== painting.id) return;
  e.stopImmediatePropagation();
  const { part, side, on, tris } = painting;
  painting = null;
  hoverMesh.visible = false;
  const j = activeJoint();
  if (!j || !tris.size) return;
  const f: PickedFeature = featureFromTris(topologyOf(part), [...tris]);
  if (on !== undefined) f.on = on;
  (side === 'a' ? j.fa : j.fb).push(f);
  markersChanged(j);
  solveAll();
  renderAll();
};
renderer.domElement.addEventListener('pointerup', endPainting, { capture: true });
renderer.domElement.addEventListener('pointercancel', endPainting, { capture: true });

renderer.domElement.addEventListener('pointerdown', (e) => {
  downPos = { x: e.clientX, y: e.clientY };
  // Klick auf einen Gizmo-Griff
  suppressClick = !!(tc.object && tc.axis);
});

// ---------------------------------------------------------------- Namen zeigen (Überfahren, Shift+Klick, Liste)

const tip = document.getElementById('tip')!;
let tipPart: number | null = null;
let tipTimer = 0;

function hideTip() {
  clearTimeout(tipTimer);
  tip.hidden = true;
  tipPart = null;
  highlightRow(null);
}

/** Zeigt nach kurzem Verweilen den Namen des Teils unter dem Mauszeiger. */
function hoverName(e: PointerEvent, part: Part | null) {
  if (!part) { hideTip(); return; }
  const rect = view.getBoundingClientRect();
  const place = () => {
    const x = Math.min(e.clientX - rect.left + 16, rect.width - tip.offsetWidth - 8);
    const y = Math.min(e.clientY - rect.top + 18, rect.height - tip.offsetHeight - 8);
    tip.style.left = `${Math.max(8, x)}px`;
    tip.style.top = `${Math.max(8, y)}px`;
  };
  if (tipPart === part.id) {
    if (!tip.hidden) place();
    return;
  }
  hideTip();
  tipPart = part.id;
  tipTimer = window.setTimeout(() => {
    const size = groupOf(part).length;
    tip.innerHTML = `<span class="dot" style="background:#${part.color.getHexString()}"></span><span>${esc(part.name)}${size > 1 ? ` <small>· Gruppe aus ${size} Teilen</small>` : ''}</span>`;
    tip.hidden = false;
    place();
    highlightRow(part.id);
  }, 350);
}

function highlightRow(id: number | null) {
  partsEl.querySelectorAll('li.hl').forEach((li) => li.classList.remove('hl'));
  if (id !== null) partsEl.querySelector(`li[data-part="${id}"]`)?.classList.add('hl');
}

/** Teil kurz im 3D-Fenster aufleuchten lassen. */
function flashPart(p: Part) {
  const mat = p.mesh.material as THREE.MeshStandardMaterial;
  mat.emissive.copy(p.color).multiplyScalar(0.6);
  setTimeout(() => mat.emissive.setRGB(0, 0, 0), 900);
}

/** Shift+Klick im 3D-Fenster: zugehörige Zeile in der Teileliste zeigen. */
function revealPart(p: Part) {
  if (partsFolded) setPartsFolded(false);
  const li = partsEl.querySelector<HTMLElement>(`li[data-part="${p.id}"]`);
  if (!li) return;
  li.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
  li.classList.remove('flash');
  void li.offsetWidth;
  li.classList.add('flash');
  flashPart(p);
}

// Zeile in der Teileliste überfahren: Teil im 3D-Fenster hervorheben
partsEl.addEventListener('pointerover', (e) => {
  const li = (e.target as HTMLElement).closest<HTMLElement>('li[data-part]');
  const p = li && getPart(Number(li.dataset.part));
  for (const q of parts) (q.mesh.material as THREE.MeshStandardMaterial).emissive.setRGB(0, 0, 0);
  if (p) (p.mesh.material as THREE.MeshStandardMaterial).emissive.copy(p.color).multiplyScalar(0.35);
});
partsEl.addEventListener('pointerleave', () => {
  for (const q of parts) (q.mesh.material as THREE.MeshStandardMaterial).emissive.setRGB(0, 0, 0);
});

/** Doppelklick auf einen Namen: direkt in der Liste umbenennen. */
partsEl.addEventListener('dblclick', (e) => {
  const span = (e.target as HTMLElement).closest<HTMLElement>('[data-act="name"]');
  const p = span && getPart(Number(span.dataset.id));
  if (!span || !p) return;
  const input = document.createElement('input');
  input.className = 'rename';
  input.value = p.name;
  span.replaceWith(input);
  input.focus();
  input.select();
  let done = false;
  const finish = (keep: boolean) => {
    if (done) return;
    done = true;
    const name = input.value.trim();
    if (keep && name) p.name = name;
    renderAll();
  };
  input.addEventListener('keydown', (k) => {
    if (k.key === 'Enter') finish(true);
    else if (k.key === 'Escape') finish(false);
  });
  input.addEventListener('blur', () => finish(true));
});

renderer.domElement.addEventListener('pointermove', (e) => {
  if (e.buttons || tc.dragging) {
    cursor.visible = false;
    hideTip();
    return;
  }
  const h = pick(e);
  hoverName(e, h?.part ?? null);
  const j = activeJoint();
  const usable = !!h && (usableFor(j, h.part) || (pickMode !== 'point' && !!helperSide(j, h.part))) && !(tc.object && tc.axis);
  cursor.visible = usable && (pickMode === 'point' || pickMode === 'brush');
  if (h && usable) {
    cursor.position.copy(h.world);
    (cursor.material as THREE.MeshBasicMaterial).color.copy(j!.color).lerp(new THREE.Color(1, 1, 1), 0.35);
  }
  updateHover(usable ? h : null);
  renderer.domElement.style.cursor = usable ? (pickMode === 'point' ? 'crosshair' : 'pointer') : 'default';
});

renderer.domElement.addEventListener('pointerleave', () => {
  cursor.visible = false;
  updateHover(null);
  hideTip();
});

renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downPos || e.button !== 0) return;
  const moved = Math.hypot(e.clientX - downPos.x, e.clientY - downPos.y);
  downPos = null;
  if (suppressClick) { suppressClick = false; return; }
  if (moved > 5) return; // war eine Kamerabewegung

  const h = pick(e);
  if (!h) return;
  if (e.shiftKey) {
    revealPart(h.part);
    return;
  }
  let j = activeJoint();
  if (!j) {
    if (parts.length < 2) { toast('Lade mindestens zwei Teile.'); return; }
    j = addJoint();
  }
  // Automatisch angelegte, noch leere Verbindung: der erste Klick bestimmt Teil 1, der nächste Teil 2
  if (j.auto && !j.applied && j.ma.length + j.mb.length + j.fa.length + j.fb.length === 0) {
    j.a = h.part.id;
    j.b = null;
  }
  j.auto = false;
  // Teile der Verbindung per Klick zuweisen, wenn noch nicht gewählt
  if (j.a === null && j.b !== h.part.id) j.a = h.part.id;
  else if (j.b === null && j.a !== h.part.id) j.b = h.part.id;

  let side: Side | null = j.a === h.part.id ? 'a' : j.b === h.part.id ? 'b' : null;
  // Fläche auf einem Nachbarteil, das schon fest mit einem der beiden Teile verbunden ist
  const helper = !side ? helperSide(j, h.part) : null;
  if (helper && pickMode !== 'point') side = helper;
  if (!side) {
    toast(helper
      ? `Punkte gehen nur auf „${nameOf(j.a)}" und „${nameOf(j.b)}" – Flächen dürfen auch auf fest verbundenen Nachbarteilen wie „${h.part.name}" liegen.`
      : `„${h.part.name}" gehört nicht zur aktiven Verbindung. Wähle sie links aus oder lege eine neue an.`, 5000);
    return;
  }
  if (pickMode === 'brush') return; // Pinsel arbeitet beim Ziehen
  if (pickMode === 'face') toggleFace(j, side, h.part, h.tri, helper ? h.part.id : undefined, e.altKey);
  else if (pickMode === 'edge') {
    const edge = edgeAt(h);
    if (!edge) { toast('Hier ist keine gerade Kante – näher an eine Kante klicken.'); return; }
    toggleEdge(j, side, h.part, edge, helper ? h.part.id : undefined);
  }
  else {
    (side === 'a' ? j.ma : j.mb).push(h.local);
    markersChanged(j);
    solveAll();
    renderAll();
  }
});

/** Wählt die Fläche unter einem Dreieck aus – oder hebt die Auswahl auf, wenn sie schon gewählt ist. */
function toggleFace(j: Joint, side: Side, part: Part, tri: number, on?: number, add = false) {
  const list = side === 'a' ? j.fa : j.fb;
  const has = (f: PickedFeature) => (f.pieces ?? [f]).some((p) => featurePart(j, side, p) === part && p.tris.includes(tri));
  const existing = list.findIndex((f) => f.kind !== 'edge' && has(f));
  if (existing >= 0) list.splice(existing, 1);
  else {
    const f: PickedFeature = detectFeature(topologyOf(part), tri, FEATURE_OPTS);
    if (on !== undefined) f.on = on;
    // Alt/⌥ + Klick: zur zuletzt gewählten Fläche dieser Seite hinzufügen
    const last = add ? list[list.length - 1] : undefined;
    if (last && mergeable(last) && mergeable(f)) {
      list[list.length - 1] = mergeOf(j, side, [...(last.pieces ?? [last]), f], (id) => getPart(id)!.mesh.matrixWorld);
    } else {
      if (add) toast('Hinzufügen geht nur zu einer ebenen oder Freiformfläche – die Fläche wurde einzeln gewählt.');
      list.push(f);
    }
  }
  markersChanged(j);
  if (j.ma.length + j.mb.length + j.fa.length + j.fb.length === 0) j.applied = false;
  solveAll();
  renderAll();
}

/** Gerade Kante in der Nähe des Treffers (lokale Koordinaten des Teils). */
function edgeAt(h: Hit): EdgeFeature | null {
  return detectEdge(topologyOf(h.part), h.tri, h.raw, EDGE_PX * h.worldPerPx);
}

/** Kante als dünne Röhre (lokale Koordinaten). */
function edgeTube(f: EdgeFeature, thick = 1): THREE.BufferGeometry {
  const r = Math.max(f.length * 0.012, sceneScale() * 0.0025) * thick;
  const g = new THREE.CylinderGeometry(r, r, f.length, 10);
  g.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(...f.dir)));
  g.translate(...f.center);
  return g;
}

/** Wählt eine Kante aus – oder hebt die Auswahl auf, wenn sie schon gewählt ist. */
function toggleEdge(j: Joint, side: Side, part: Part, edge: EdgeFeature, on?: number) {
  const list = side === 'a' ? j.fa : j.fb;
  const same = (f: PickedFeature) => f.kind === 'edge' && featurePart(j, side, f) === part && f.tris.some((t) => edge.tris.includes(t));
  const existing = list.findIndex(same);
  if (existing >= 0) list.splice(existing, 1);
  else {
    const f: PickedFeature = edge;
    if (on !== undefined) f.on = on;
    list.push(f);
  }
  markersChanged(j);
  if (j.ma.length + j.mb.length + j.fa.length + j.fb.length === 0) j.applied = false;
  solveAll();
  renderAll();
}

renderer.domElement.addEventListener('contextmenu', (e) => {
  e.preventDefault();
  // Strg+Klick ist auf dem Mac ein Rechtsklick – dort beginnt das Verschieben, nichts löschen
  if (e.ctrlKey && e.button === 0) return;
  setNdc(e);
  const targets = [...markerGroup.children.filter((o) => (o as THREE.Mesh).isMesh), ...faceGroup.children];
  const hit = raycaster.intersectObjects(targets, false)[0];
  if (!hit) return;
  const { joint, side, index, kind } = hit.object.userData as { joint: number; side: Side; index: number; kind: 'point' | 'face' };
  const j = joints.find((x) => x.id === joint);
  if (!j) return;
  if (kind === 'face') (side === 'a' ? j.fa : j.fb).splice(index, 1);
  else (side === 'a' ? j.ma : j.mb).splice(index, 1);
  markersChanged(j);
  if (j.ma.length + j.mb.length + j.fa.length + j.fb.length === 0) j.applied = false;
  solveAll();
  renderAll();
});

function setPickMode(mode: PickMode) {
  pickMode = mode;
  document.querySelectorAll<HTMLButtonElement>('[data-pick]').forEach((b) => b.classList.toggle('on', b.dataset.pick === mode));
  // Einrasten an Ecken gibt es nur für Punkte
  document.getElementById('snap')!.closest('label')!.hidden = mode !== 'point';
  updateHover(null);
  cursor.visible = false;
  renderHint();
}

/** Textfeld mit eigener Rückgängig-Funktion des Browsers? (Kästchen, Regler und Listen gehören nicht dazu.) */
function editsText(el: EventTarget | null): boolean {
  if (!(el instanceof HTMLElement)) return false;
  if (el.isContentEditable || el instanceof HTMLTextAreaElement) return true;
  return el instanceof HTMLInputElement && ['text', 'number', 'search', 'email', 'url', 'password', 'tel'].includes(el.type);
}

window.addEventListener('keydown', (e) => {
  // In Textfeldern gelten die Tasten dem Feld (auch dessen eigenes Rückgängig)
  if (editsText(e.target)) return;
  // Kästchen, Listen usw.: nur Strg+Z/Y übernehmen, Buchstaben-Kürzel nicht
  if ((e.target as HTMLElement).closest('input, select, textarea') && !isUndoKey(e)) return;
  // Strg/⌘+Z: letzte Aktion zurücknehmen, Strg/⌘+Umschalt+Z oder Strg+Y: wiederholen
  if ((e.ctrlKey || e.metaKey) && (e.key.toLowerCase() === 'z' || e.key.toLowerCase() === 'y')) {
    e.preventDefault();
    if (e.key.toLowerCase() === 'y' || e.shiftKey) redo();
    else undo();
    return;
  }
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  const k = e.key.toLowerCase();
  if (k === 'p') setPickMode('point');
  else if (k === 'f') setPickMode('face');
  else if (k === 'k') setPickMode('edge');
  else if (k === 'b') setPickMode('brush');
  else if (gizmoTarget && (k === 'g' || k === 'r')) {
    gizmoMode = k === 'g' ? 'translate' : 'rotate';
    updateGizmoAxes();
    renderGizmoBar();
  } else if (gizmoTarget && k === 'escape') closeGizmo();
});

// ---------------------------------------------------------------- Seitenleiste


function renderParts() {
  const row = (p: Part) => `<li data-part="${p.id}"${p.visible ? '' : ' class="off"'}>
        <input type="color" class="swatch" data-act="color" data-id="${p.id}" value="#${p.color.getHexString()}" title="Farbe (wird beim mehrfarbigen 3MF-Export übernommen)" />
        <input type="checkbox" data-act="vis" data-id="${p.id}" ${p.visible ? 'checked' : ''} title="Sichtbar" />
        <span class="name" data-act="name" data-id="${p.id}" title="${esc(p.name)} – Klick: im Modell zeigen, Doppelklick: umbenennen">${esc(p.name)}</span>
        <label class="base" title="Das Basisteil bleibt liegen, alle anderen werden daran angefügt.">
          <input type="radio" name="base" data-act="base" data-id="${p.id}" ${p.id === baseId ? 'checked' : ''}/> Basis
        </label>
        ${p.group && groupOf(p).length > 1 ? `<button class="icon" data-act="ungroup" data-id="${p.id}" title="Gehört zu einer Gruppe (${groupOf(p).length} Teile, bewegen sich gemeinsam). Klicken, um dieses Teil aus der Gruppe zu lösen.">⛓</button>` : ''}
        <button class="icon move${gizmoPart()?.id === p.id && !gizmoTarget?.assembly ? ' on' : ''}" data-act="move-part" data-id="${p.id}" title="Teil von Hand bewegen/drehen (nochmal klicken: fertig)" aria-label="Teil von Hand bewegen/drehen" aria-pressed="${gizmoPart()?.id === p.id}">${MOVE_ICON}</button>
        <button class="icon rm" data-act="rm-part" data-id="${p.id}" title="Entfernen" aria-label="Entfernen">✕</button>
      </li>`;
  // Sichtbare Teile oben, ausgeblendete in einem eigenen, einklappbaren Abschnitt
  const hidden = parts.filter((p) => !p.visible);
  partsEl.innerHTML = parts.filter((p) => p.visible).map(row).join('') + (hidden.length
    ? `<li class="sep"><button class="icon fold" data-act="fold-hidden" aria-expanded="${!hiddenFolded}" title="Ausgeblendete Teile ein- oder ausklappen">${hiddenFolded ? '▸' : '▾'}</button>
        <span>Ausgeblendet (${hidden.length})</span></li>${hiddenFolded ? '' : hidden.map(row).join('')}`
    : '');
  // Alle ein-/ausblenden
  const shown = parts.filter((p) => p.visible).length;
  const bar = document.getElementById('partsBar')!;
  bar.hidden = parts.length < 2;
  // Eingeklappt: nur die Kopfzeile mit Zähler
  const folded = partsFolded && parts.length >= 2;
  partsEl.hidden = folded;
  const toggle = document.getElementById('partsToggle')!;
  toggle.textContent = folded ? '▸' : '▾';
  toggle.setAttribute('aria-expanded', String(!folded));
  const all = document.getElementById('allVis') as HTMLInputElement;
  all.checked = shown === parts.length;
  all.indeterminate = shown > 0 && shown < parts.length;
  document.getElementById('partsCount')!.textContent = `${shown} von ${parts.length} sichtbar`;
}

function setPartsFolded(on: boolean) {
  partsFolded = on;
  try { localStorage.setItem('partjoin.partsFolded', on ? '1' : '0'); } catch { /* ohne Speicher */ }
  renderParts();
}
document.getElementById('partsToggle')!.addEventListener('click', () => setPartsFolded(!partsFolded));

document.getElementById('allVis')!.addEventListener('change', (e) => {
  const on = (e.target as HTMLInputElement).checked;
  for (const p of parts) { p.visible = on; p.mesh.visible = on; }
  renderAll();
});

function partOptions(sel: number | null, exclude: number | null) {
  return (
    `<option value="">– wählen –</option>` +
    parts
      .filter((p) => p.id !== exclude && (p.visible || p.id === sel))
      .map((p) => `<option value="${p.id}" ${p.id === sel ? 'selected' : ''}>${esc(p.name)}</option>`)
      .join('')
  );
}

function jointStatusHTML(j: Joint) {
  return j.status ? `<div class="status ${j.status.kind}" data-status="${j.id}">${esc(j.status.text)}</div>` : `<div class="status" data-status="${j.id}"></div>`;
}

function featureChips(j: Joint, side: Side) {
  const list = side === 'a' ? j.fa : j.fb;
  if (!list.length) return '';
  const counters: Record<string, number> = {};
  return `<ul class="chips">${list
    .map((f, i) => {
      const group = f.kind === 'cylinder' ? 'round' : f.kind === 'edge' ? 'edge' : 'flat';
      counters[group] = (counters[group] ?? 0) + 1;
      const own = side === 'a' ? j.a : j.b;
      // Auf welchen Teilen die Fläche liegt – nur nennen, wenn nicht ausschließlich auf dem Teil der Seite
      const where = [...new Set((f.pieces ?? [f]).map((p) => p.on ?? own))];
      // Lange Namen lieber nach einem Unterstrich umbrechen als mitten im Wort
      const nice = (id: number | null) => esc(nameOf(id)).replace(/_/g, '_<wbr>');
      const on = where.some((x) => x !== own) ? `<small>auf „${where.map(nice).join('" und „')}"</small>` : '';
      const merged = f.pieces ? `<small>aus ${f.pieces.length} Flächen zusammengefasst</small>` : '';
      const pick = mergePick.get(j.id);
      const checked = pick?.side === side && pick.idx.has(i);
      return `<li>${mergeable(f) && list.filter(mergeable).length > 1
        ? `<input type="checkbox" data-act="merge-pick" data-side="${side}" data-idx="${i}" data-id="${j.id}" ${checked ? 'checked' : ''} title="Zum Zusammenfassen auswählen" />`
        : ''}<b>${featureTag(f)}${counters[group]}</b> <span class="desc">${esc(describeFeature(f))}${merged}${on}</span>
        ${f.pieces ? `<button class="icon" data-act="split-face" data-side="${side}" data-idx="${i}" data-id="${j.id}" title="Wieder in die Einzelflächen trennen">⇹</button>` : ''}
        <button class="icon" data-act="rm-face" data-side="${side}" data-idx="${i}" data-id="${j.id}" title="Fläche abwählen">✕</button></li>`;
    })
    .join('')}</ul>${mergePick.get(j.id)?.side === side && mergePick.get(j.id)!.idx.size > 1
    ? `<button class="ghost small merge" data-act="merge-faces" data-side="${side}" data-id="${j.id}" title="Die angehakten Flächen zählen danach als eine Fläche – z. B. ein Aussparungsgrund aus zwei Teilen.">${mergePick.get(j.id)!.idx.size} Flächen zu einer zusammenfassen</button>`
    : ''}`;
}

/** Knopf für weitere, gleich gut passende Drehungen (z. B. rechteckiger Zapfen um 180° gedreht). */
function turnButton(j: Joint): string {
  const t = j.cache?.turn;
  if (!t) return '';
  const n = t.angles.length + 1;
  const i = (j.turnIdx ?? 0) % n;
  const deg = (a: number) => Math.round(THREE.MathUtils.radToDeg(a));
  const label = n === 2 ? (i ? 'Zurückdrehen' : `Um ${deg(t.angles[0])}° drehen`) : `Andere Drehung ${i + 1}/${n}`;
  return `<button class="ghost" data-act="turn" data-id="${j.id}" title="Passt genauso gut: ${t.angles.map((a) => `${deg(a)}°`).join(', ')} gedreht. Klicken, um die Lage zu wechseln.">${label}</button>`;
}

function renderJoints() {
  jointsEl.innerHTML = joints
    .map((j, idx) => {
      const active = j.id === activeJointId;
      const feat = hasFeatures(j);
      const sol = !feat ? j.result?.solutions[Math.min(j.alt, (j.result?.solutions.length ?? 1) - 1)] : undefined;
      const nAlt = alternativesOf(j);
      const ready =
        j.a !== null && j.b !== null &&
        (feat ? j.fa.length > 0 && j.fb.length > 0 && j.ma.length === j.mb.length : j.ma.length > 0 && j.ma.length === j.mb.length);
      const head = `<div class="head">
          <span class="dot" style="background:#${j.color.getHexString()}"></span>
          <span class="grow">Verbindung ${idx + 1}</span>
          <button class="icon" data-act="rm-joint" data-id="${j.id}" title="Verbindung löschen">✕</button>
        </div>`;
      const sel = [
        j.ma.length + j.mb.length ? `${j.ma.length}/${j.mb.length} Punkte` : '',
        j.fa.length + j.fb.length ? `${j.fa.length}/${j.fb.length} Flächen` : '',
      ].filter(Boolean).join(' · ') || 'noch nichts gewählt';
      if (!active) {
        return `<div class="joint" data-act="activate" data-id="${j.id}">${head}
          <div class="help" style="margin:0">${esc(nameOf(j.a))} ↔ ${esc(nameOf(j.b))} · ${sel}</div>
          ${jointStatusHTML(j)}</div>`;
      }
      const isGizmo = gizmoTarget?.kind === 'joint' && gizmoTarget.id === j.id;
      return `<div class="joint active">${head}
        <label class="field">Teil 1<select data-act="sel-a" data-id="${j.id}">${partOptions(j.a, j.b)}</select></label>
        <label class="field">Teil 2<select data-act="sel-b" data-id="${j.id}">${partOptions(j.b, j.a)}</select></label>
        <div class="counts">
          <div class="c"><span>Teil 1: <b>${j.ma.length}</b> P · <b>${j.fa.length}</b> F</span><button class="icon" data-act="clr-a" data-id="${j.id}" title="Auswahl auf Teil 1 löschen">⌫</button></div>
          <div class="c"><span>Teil 2: <b>${j.mb.length}</b> P · <b>${j.fb.length}</b> F</span><button class="icon" data-act="clr-b" data-id="${j.id}" title="Auswahl auf Teil 2 löschen">⌫</button></div>
        </div>
        ${featureChips(j, 'a')}${featureChips(j, 'b')}
        ${feat
          ? `<label class="check small" title="Sucht die Drehung um eine freie Achse und schiebt das Teil bis zum Anschlag – über Flächenkontakt ohne Durchdringung."><input type="checkbox" data-act="seat" data-id="${j.id}" ${j.seat ? 'checked' : ''}/>
              Automatisch einpassen (Drehung &amp; Anschlag suchen)</label>`
          : `<label class="check small"><input type="checkbox" data-act="ordered" data-id="${j.id}" ${j.ordered ? 'checked' : ''}/>
              Reihenfolge vorgeben (sonst automatische Zuordnung)</label>
            <label class="field">Flächen­normalen
              <span class="seg">${(['auto', 'on', 'off'] as NormalMode[])
                .map((m) => `<button data-act="normals" data-val="${m}" data-id="${j.id}" class="${j.normals === m ? 'on' : ''}">${{ auto: 'Auto', on: 'An', off: 'Aus' }[m]}</button>`)
                .join('')}</span></label>`}
        <label class="field" title="Schiebt das Teil etwas ineinander, damit der Slicer beide Teile sicher zu einem Körper verschmilzt.">Einsinken (mm)
          <input type="number" step="0.05" min="-5" max="5" value="${j.overlap}" data-act="overlap" data-id="${j.id}"/></label>
        ${sol?.freeAxis ? `<label class="field" title="Bei wenigen Punkten ist die Drehung um die Kontaktachse nicht festgelegt.">Drehung <span class="row"><input type="range" min="-180" max="180" step="1" value="${j.spin}" data-act="spin" data-id="${j.id}"/><span data-spinval="${j.id}" style="width:38px;text-align:right">${j.spin}°</span></span></label>` : ''}
        <div class="row wrap">
          ${j.a !== null && j.b !== null
            ? j.proposals && j.proposals.length > 1
              ? `<button class="ghost" data-act="next-proposal" data-id="${j.id}" title="Nächsten automatischen Vorschlag ausprobieren">Nächster Vorschlag ${(j.proposalIdx ?? 0) + 1}/${j.proposals.length}</button>`
              : `<button class="ghost" data-act="propose" data-id="${j.id}" title="Sucht passende Flächen auf beiden Teilen (deckungsgleiche Schnittflächen, Zapfen und Bohrungen) und verbindet sie.">Vorschlagen</button>`
            : ''}
          ${j.applied
            ? `<button class="ghost" data-act="disconnect" data-id="${j.id}">Lösen</button>
               <button class="${isGizmo ? '' : 'ghost'}" data-act="gizmo" data-id="${j.id}" title="Mit Pfeilen und Drehring nachjustieren – standardmäßig nur in den Richtungen, die die Verbindung offen lässt.">${isGizmo ? 'Anpassen beenden' : 'Von Hand anpassen'}</button>
               <button class="ghost" data-act="refine" data-id="${j.id}" title="Zieht die Kontaktflächen nahe der Auswahl genau aufeinander (ICP).">Feinausrichtung</button>
               ${turnButton(j)}
               ${nAlt > 1 && !j.ordered ? `<button class="ghost" data-act="alt" data-id="${j.id}" title="Nächste mögliche Lösung ausprobieren">Alternative ${Math.min(j.alt, nAlt - 1) + 1}/${nAlt}</button>` : ''}`
            : `<button data-act="connect" data-id="${j.id}" ${ready ? '' : 'disabled'}>Verbinden</button>`}
        </div>
        ${jointStatusHTML(j)}
      </div>`;
    })
    .join('');
}

function renderHint() {
  const el = document.getElementById('hint')!;
  const j = activeJoint();
  const what = { face: 'Flächen', edge: 'Kanten', point: 'Punkte', brush: 'Flächen' }[pickMode];
  if (parts.length === 0) el.textContent = 'Lade zwei oder mehr Modelle – oder probiere ein Beispiel.';
  else if (parts.length === 1) el.textContent = 'Lade noch ein weiteres Teil, das angefügt werden soll.';
  else if (!j) el.textContent = 'Lege links eine Verbindung an.';
  else if (j.auto && !j.ma.length && !j.mb.length && !j.fa.length && !j.fb.length) {
    el.textContent = {
      face: 'Klicke die Kontaktfläche auf einem Teil – danach die passende Fläche auf dem Teil, das daran soll. Oder links „Alles automatisch verbinden".',
      edge: 'Klicke eine gerade Kante auf einem Teil – danach die passende Kante auf dem Teil, das daran soll.',
      point: 'Klicke markante Punkte auf einem Teil – danach dieselben Stellen auf dem Teil, das daran soll.',
      brush: 'Male die Kontaktstelle auf einem Teil an – danach die passende Stelle auf dem Teil, das daran soll.',
    }[pickMode];
  } else if (j.a !== null && j.b === null) {
    const what = { face: 'Fläche', edge: 'Kante', point: 'Stelle', brush: 'Stelle' }[pickMode];
    el.textContent = `Jetzt die passende ${what} auf dem Teil anklicken, das an „${nameOf(j.a)}" soll.`;
  } else if (j.a === null || j.b === null) el.textContent = 'Klicke die beiden Teile an, die verbunden werden sollen.';
  else if (j.applied) el.textContent = `Verbunden. Weitere ${what} verfeinern das Ergebnis live · Rechtsklick entfernt eine Auswahl.`;
  else if (pickMode === 'brush') {
    el.textContent = `Pinsel: mit gedrückter Maustaste über die Kontaktstelle auf „${nameOf(j.fa.length <= j.fb.length ? j.a : j.b)}" streichen – beim Loslassen wird daraus eine Fläche. Daneben ziehen dreht die Ansicht.`;
  } else if (pickMode === 'edge') {
    const na = j.fa.filter((f) => f.kind === 'edge').length;
    const nb = j.fb.filter((f) => f.kind === 'edge').length;
    if (!na && !nb) el.textContent = `Klicke nahe an eine gerade Kante von „${nameOf(j.a)}" und an die passende Kante von „${nameOf(j.b)}". Kanten lassen sich mit Flächen kombinieren.`;
    else if (na !== nb) el.textContent = `Jetzt die passende Kante auf „${nameOf(na < nb ? j.a : j.b)}" anklicken.`;
    else el.textContent = 'Kanten gewählt. Mit einer Fläche dazu wird die Lage eindeutig – oder „Verbinden" klicken.';
  } else if (pickMode === 'face') {
    if (!j.fa.length && !j.fb.length) el.textContent = `Klicke die Kontaktfläche auf „${nameOf(j.a)}" und die passende auf „${nameOf(j.b)}" (z. B. Zapfen und Bohrung).`;
    else if (!j.fa.length || !j.fb.length) el.textContent = `Jetzt die passende Fläche auf „${nameOf(j.fa.length ? j.b : j.a)}" anklicken.`;
    else el.textContent = 'Flächen gewählt. Weitere Flächenpaare (z. B. Auflage + Zapfen) machen es eindeutiger – oder „Verbinden" klicken.';
  } else {
    const na = j.ma.length;
    const nb = j.mb.length;
    if (na === 0 && nb === 0) el.textContent = `Klicke markante Punkte auf „${nameOf(j.a)}" – danach dieselben Stellen auf „${nameOf(j.b)}". Tipp: Mit „Flächen" (Taste F) geht es meist schneller.`;
    else if (na !== nb) el.textContent = `Noch ${Math.abs(na - nb)} Punkt(e) auf „${na < nb ? nameOf(j.a) : nameOf(j.b)}" – Reihenfolge egal.`;
    else el.textContent = `${na} Punktpaar(e) gesetzt. Weitere Punkte setzen oder links auf „Verbinden" klicken.`;
  }
}

/** Läuft gerade ein Neuaufbau? Beim Ersetzen der Seitenleiste kann ein fokussiertes Feld sein „change"-Ereignis auslösen. */
let rendering = false;

function renderAll() {
  if (rendering) return;
  // Gizmo schließen, wenn sein Teil ausgeblendet oder entfernt wurde
  if (gizmoTarget && !gizmoPart()?.visible) closeGizmo();
  rendering = true;
  try {
    renderParts();
    renderJoints();
  } finally {
    rendering = false;
  }
  renderHint();
  rebuildMarkers();
  void runCheck();
  if (gizmoTarget && !tc.dragging) {
    placeGizmo();
    renderGizmoBar();
  }
}

/** Das Teil, das das Gizmo gerade bewegt. */
function gizmoPart(): Part | undefined {
  if (!gizmoTarget) return undefined;
  if (gizmoTarget.kind === 'part') return getPart(gizmoTarget.id);
  const j = joints.find((x) => x.id === gizmoTarget!.id);
  return j?.fixedSide ? getPart(j.fixedSide === 'a' ? j.b : j.a) : undefined;
}

/** Nur Statuszeilen und 3D-Anzeige aktualisieren (z. B. während ein Regler oder das Gizmo gezogen wird). */
function renderLight() {
  for (const j of joints) {
    const el = jointsEl.querySelector(`[data-status="${j.id}"]`);
    if (el) {
      el.className = `status ${j.status?.kind ?? ''}`;
      el.textContent = j.status?.text ?? '';
    }
  }
  rebuildMarkers();
}

document.getElementById('panel')!.addEventListener('click', (e) => {
  const t = (e.target as HTMLElement).closest<HTMLElement>('[data-act]');
  if (!t) return;
  const id = Number(t.dataset.id);
  const j = joints.find((x) => x.id === id);
  switch (t.dataset.act) {
    case 'activate':
      activeJointId = id;
      renderAll();
      break;
    case 'rm-joint':
      e.stopPropagation();
      removeJoint(id);
      break;
    case 'rm-part':
      removePart(id);
      break;
    case 'name': {
      const p = getPart(id);
      if (p) flashPart(p);
      break;
    }
    case 'ungroup': {
      const p = getPart(id);
      if (!p) break;
      // Lage beibehalten: aktuelle Position wird die neue Ausgangslage
      p.layout.copy(p.mesh.matrixWorld);
      p.group = undefined;
      solveAll();
      renderAll();
      toast(`„${p.name}" bewegt sich jetzt unabhängig von seiner Gruppe.`);
      break;
    }
    case 'fold-hidden':
      hiddenFolded = !hiddenFolded;
      renderParts();
      break;
    case 'move-part': {
      // Nochmal klicken schließt das Gizmo wieder
      if (gizmoPart()?.id === id && !gizmoTarget?.assembly) {
        closeGizmo();
        renderAll();
        break;
      }
      // Wird das Teil von einer Verbindung bewegt, wird diese angepasst
      const jid = movedBy.get(id);
      if (jid !== undefined) {
        activeJointId = jid;
        openGizmo({ kind: 'joint', id: jid });
      } else openGizmo({ kind: 'part', id });
      renderAll();
      break;
    }
    case 'clr-a':
    case 'clr-b':
      if (!j) break;
      if (t.dataset.act === 'clr-a') { j.ma = []; j.fa = []; } else { j.mb = []; j.fb = []; }
      j.applied = false;
      markersChanged(j);
      solveAll();
      renderAll();
      break;
    case 'merge-faces': {
      const pick = j && mergePick.get(j.id);
      if (!j || !pick) break;
      const side = pick.side;
      const list = side === 'a' ? j.fa : j.fb;
      const idx = [...pick.idx].sort((x, y) => x - y).filter((i) => list[i] && mergeable(list[i]));
      if (idx.length < 2) break;
      try {
        const merged = mergeOf(j, side, idx.flatMap((i) => list[i].pieces ?? [list[i]]), (id) => getPart(id)!.mesh.matrixWorld);
        for (const i of [...idx].reverse()) list.splice(i, 1);
        list.splice(idx[0], 0, merged);
      } catch (err) {
        toast(`Zusammenfassen nicht möglich: ${(err as Error).message}`);
      }
      markersChanged(j);
      solveAll();
      renderAll();
      break;
    }
    case 'split-face': {
      if (!j) break;
      const list = t.dataset.side === 'a' ? j.fa : j.fb;
      const i = Number(t.dataset.idx);
      const pieces = list[i]?.pieces;
      if (!pieces) break;
      list.splice(i, 1, ...pieces);
      markersChanged(j);
      solveAll();
      renderAll();
      break;
    }
    case 'rm-face':
      if (!j) break;
      (t.dataset.side === 'a' ? j.fa : j.fb).splice(Number(t.dataset.idx), 1);
      markersChanged(j);
      if (j.ma.length + j.mb.length + j.fa.length + j.fb.length === 0) j.applied = false;
      solveAll();
      renderAll();
      break;
    case 'normals':
      if (!j) break;
      j.normals = t.dataset.val as NormalMode;
      markersChanged(j);
      solveAll();
      renderAll();
      break;
    case 'connect':
      if (j) connect(j);
      break;
    case 'disconnect':
      if (!j) break;
      if (gizmoTarget?.kind === 'joint' && gizmoTarget.id === j.id) closeGizmo();
      j.applied = false;
      j.adjust = undefined;
      j.adjustInfo = undefined;
      solveAll();
      renderAll();
      break;
    case 'gizmo':
      if (!j) break;
      if (gizmoTarget?.kind === 'joint' && gizmoTarget.id === j.id) closeGizmo();
      else openGizmo({ kind: 'joint', id: j.id });
      renderAll();
      break;
    case 'refine':
      if (j) refine(j);
      break;
    case 'propose':
      if (j) proposeJoint(j);
      break;
    case 'next-proposal': {
      if (!j?.proposals?.length) break;
      j.proposalIdx = ((j.proposalIdx ?? 0) + 1) % j.proposals.length;
      applySuggestion(j, j.proposals[j.proposalIdx]);
      solveAll();
      renderAll();
      break;
    }
    case 'turn': {
      const t = j?.cache?.turn;
      if (!j || !t) break;
      j.turnIdx = ((j.turnIdx ?? 0) + 1) % (t.angles.length + 1);
      const a = j.turnIdx ? t.angles[j.turnIdx - 1] : 0;
      j.adjust = a ? rotationAbout(t.point, t.axis, a) : undefined;
      j.adjustInfo = a ? `um ${Math.round(THREE.MathUtils.radToDeg(a))}° gedreht (passt ebenso)` : undefined;
      solveAll();
      renderAll();
      break;
    }
    case 'alt': {
      if (!j) break;
      const n = alternativesOf(j);
      if (n < 2) break;
      j.alt = (Math.min(j.alt, n - 1) + 1) % n;
      j.turnIdx = 0;
      j.adjust = undefined;
      j.adjustInfo = undefined;
      solveAll();
      renderAll();
      break;
    }
  }
});

document.getElementById('panel')!.addEventListener('input', (e) => {
  const t = e.target as HTMLInputElement;
  const j = joints.find((x) => x.id === Number(t.dataset.id));
  if (t.dataset.act === 'spin' && j) {
    j.spin = Number(t.value);
    const lbl = jointsEl.querySelector(`[data-spinval="${j.id}"]`);
    if (lbl) lbl.textContent = `${j.spin}°`;
    solveAll();
    renderLight();
  } else if (t.dataset.act === 'overlap' && j) {
    j.overlap = Number(t.value) || 0;
    solveAll();
    renderLight();
  }
});

document.getElementById('panel')!.addEventListener('change', (e) => {
  const t = e.target as HTMLInputElement;
  const id = Number(t.dataset.id);
  const j = joints.find((x) => x.id === id);
  switch (t.dataset.act) {
    case 'merge-pick': {
      if (!j) break;
      const side = t.dataset.side as Side;
      const prev = mergePick.get(j.id);
      const pick = prev?.side === side ? prev : { side, idx: new Set<number>() };
      if (t.checked) pick.idx.add(Number(t.dataset.idx));
      else pick.idx.delete(Number(t.dataset.idx));
      mergePick.set(j.id, pick);
      renderJoints();
      break;
    }
    case 'vis': {
      const p = getPart(id);
      if (p) { p.visible = t.checked; p.mesh.visible = t.checked; }
      renderAll();
      break;
    }
    case 'color': {
      const p = getPart(id);
      if (p) {
        p.color.set(t.value);
        (p.mesh.material as THREE.MeshStandardMaterial).color.copy(p.color);
      }
      break;
    }
    case 'base':
      baseId = id;
      solveAll();
      renderAll();
      fitView();
      break;
    case 'sel-a':
    case 'sel-b': {
      if (!j) break;
      const v = t.value ? Number(t.value) : null;
      j.auto = false;
      if (t.dataset.act === 'sel-a') { j.a = v; j.ma = []; j.fa = []; } else { j.b = v; j.mb = []; j.fb = []; }
      j.applied = false;
      j.proposals = undefined;
      markersChanged(j);
      solveAll();
      renderAll();
      break;
    }
    case 'ordered':
      if (!j) break;
      j.ordered = t.checked;
      markersChanged(j);
      solveAll();
      renderAll();
      break;
    case 'seat':
      if (!j) break;
      j.seat = t.checked;
      j.cache = undefined;
      solveAll();
      renderAll();
      break;
    case 'spin':
    case 'overlap':
      renderAll();
      break;
  }
});

document.getElementById('addJoint')!.addEventListener('click', () => {
  if (parts.length < 2) { toast('Lade zuerst mindestens zwei Teile.'); return; }
  addJoint();
  solveAll();
  renderAll();
});

document.getElementById('fit')!.addEventListener('click', fitView);
document.getElementById('moveAll')!.addEventListener('click', () => {
  const base = getPart(baseId);
  if (!base) { toast('Lade zuerst ein Modell.'); return; }
  openGizmo({ kind: 'part', id: base.id, assembly: true }, 'translate');
  renderAll();
});

// Seitenleiste in der Breite verstellen; die Breite bleibt im Browser gespeichert
{
  const panel = document.getElementById('panel')!;
  const resizer = document.getElementById('resizer')!;
  const KEY = 'partjoin.panelWidth';
  const apply = (w: number) => document.documentElement.style.setProperty('--panel-w', `${Math.round(w)}px`);
  try {
    const saved = Number(localStorage.getItem(KEY));
    if (saved >= 240) apply(Math.min(saved, window.innerWidth * 0.7));
  } catch {
    // ohne gespeicherte Breite
  }
  resizer.addEventListener('pointerdown', (e) => {
    resizer.setPointerCapture(e.pointerId);
    resizer.classList.add('drag');
    const left = panel.getBoundingClientRect().left;
    const move = (m: PointerEvent) => apply(Math.max(240, Math.min(window.innerWidth * 0.7, m.clientX - left)));
    const up = () => {
      resizer.classList.remove('drag');
      resizer.removeEventListener('pointermove', move);
      resizer.removeEventListener('pointerup', up);
      try { localStorage.setItem(KEY, String(panel.getBoundingClientRect().width)); } catch { /* nicht speicherbar */ }
    };
    resizer.addEventListener('pointermove', move);
    resizer.addEventListener('pointerup', up);
  });
  resizer.addEventListener('dblclick', () => {
    document.documentElement.style.removeProperty('--panel-w');
    try { localStorage.removeItem(KEY); } catch { /* nicht speicherbar */ }
  });
}
document.getElementById('connectAll')!.addEventListener('click', () => {
  if (parts.length < 2) { toast('Lade zuerst mindestens zwei Teile.'); return; }
  void connectAll();
});
document.getElementById('checkBtn')!.addEventListener('click', () => {
  checkOn = !checkOn;
  document.getElementById('checkBtn')!.classList.toggle('ghost', !checkOn);
  document.getElementById('checkBtn')!.textContent = checkOn ? 'Prüfung ausblenden' : 'Passung prüfen';
  void runCheck();
});
document.querySelectorAll<HTMLButtonElement>('[data-pick]').forEach((b) =>
  b.addEventListener('click', () => setPickMode(b.dataset.pick as PickMode)),
);

// Gizmo-Leiste
const gizmoBar = document.getElementById('gizmoBar')!;
gizmoBar.addEventListener('click', (e) => {
  const t = (e.target as HTMLElement).closest<HTMLElement>('[data-g-mode], [data-g]');
  if (!t) return;
  if (t.dataset.gMode) {
    gizmoMode = t.dataset.gMode as 'translate' | 'rotate';
    updateGizmoAxes();
    renderGizmoBar();
  } else if (t.dataset.g === 'done') {
    closeGizmo();
    renderAll();
  } else if (t.dataset.g === 'reset' && gizmoTarget?.kind === 'joint') {
    const j = joints.find((x) => x.id === gizmoTarget!.id);
    if (j) {
      j.adjust = undefined;
      j.adjustInfo = undefined;
      solveAll();
      renderAll();
    }
  }
});
gizmoBar.addEventListener('change', (e) => {
  const t = e.target as HTMLInputElement;
  if (t.id === 'gizmoFree') gizmoFreeOnly = t.checked;
  else if (t.id === 'gizmoSnap') gizmoSnapDeg = Number(t.value);
  placeGizmo();
});

const fileInput = document.getElementById('file') as HTMLInputElement;
fileInput.accept = SUPPORTED.map((s) => `.${s}`).join(',');
fileInput.addEventListener('change', () => {
  if (fileInput.files?.length) loadFiles(fileInput.files);
  fileInput.value = '';
});

const drop = document.getElementById('drop')!;
for (const target of [drop, view]) {
  target.addEventListener('dragover', (e) => { e.preventDefault(); drop.classList.add('over'); });
  target.addEventListener('dragleave', () => drop.classList.remove('over'));
  target.addEventListener('drop', (e) => {
    e.preventDefault();
    drop.classList.remove('over');
    const files = (e as DragEvent).dataTransfer?.files;
    if (files?.length) loadFiles(files);
  });
}

function loadDemo(items: { name: string; geom: THREE.BufferGeometry }[], hint: string, mode: PickMode) {
  closeGizmo();
  for (const p of [...parts]) removePart(p.id);
  joints.length = 0;
  activeJointId = null;
  const keep = document.getElementById('keepPos') as HTMLInputElement;
  const wasChecked = keep.checked;
  keep.checked = false;
  for (const { name, geom } of items) addPart(name, geom);
  keep.checked = wasChecked;
  ensureJoint();
  setPickMode(mode);
  solveAll();
  fitView();
  renderAll();
  toast(hint, 7000);
}

document.getElementById('demo')!.addEventListener('click', () =>
  loadDemo(makeDemo(), 'Beispiel „Haus": Klicke die Oberseite des Hauses und die Unterseite des Dachs an, dann „Verbinden" – oder setze unter „Punkte" die vier Ecken.', 'face'),
);
document.getElementById('demoRound')!.addEventListener('click', () =>
  loadDemo(makeRoundDemo(), 'Beispiel „Zapfen": Klicke den runden Teil des Zapfens und die runde Bohrungswand der Kappe an, dann „Verbinden". Die Drehung findet PartJoin über die Abflachung selbst.', 'face'),
);

// Drucker für das Bambu-Projekt: zuletzt gewählter oder der eines geladenen Bambu-Projekts
const bambuPrinter = document.getElementById('bambuPrinter') as HTMLSelectElement;
bambuPrinter.innerHTML = BAMBU_PRINTERS.map((p) => `<option value="${p.id}">${p.label}</option>`).join('');
try { bambuPrinter.value = localStorage.getItem('partjoin.bambuPrinter') ?? 'X1C'; } catch { /* ohne Speicher */ }
if (!bambuPrinter.value) bambuPrinter.value = 'X1C';
function setBambuPrinter(id: string) {
  bambuPrinter.value = id;
  try { localStorage.setItem('partjoin.bambuPrinter', id); } catch { /* ohne Speicher */ }
}
bambuPrinter.addEventListener('change', () => setBambuPrinter(bambuPrinter.value));

document.querySelectorAll<HTMLButtonElement>('[data-export]').forEach((b) =>
  b.addEventListener('click', () => void exportAs(b.dataset.export ?? 'stl')),
);

/** Exportiert alle sichtbaren Teile in ihrer zusammengesetzten Lage. */
async function exportAs(kind: string) {
  const visible = parts.filter((p) => p.visible);
  if (!visible.length) { toast('Keine sichtbaren Teile zum Exportieren.'); return; }
  const world = visible.map((p) => ({
    name: p.name,
    color: `#${p.color.getHexString()}`,
    positions: p.geom.clone().applyMatrix4(p.mesh.matrixWorld).getAttribute('position').array as Float32Array,
  }));
  try {
    if (kind === '3mf-color' || kind === '3mf-bambu') {
      const data = kind === '3mf-bambu' ? write3MFBambu(world, { printer: bambuPrinter.value }) : write3MF(world, 'multicolor');
      await download(new Blob([data as Uint8Array<ArrayBuffer>], { type: 'model/3mf' }), 'zusammengefuegt.3mf');
      const slots = colorSlots(world);
      toast(kind === '3mf-bambu'
        ? `Gespeichert – ${slots.length} Filament${slots.length > 1 ? 'e' : ''} in den Teilefarben, Drucker ${bambuPrinter.selectedOptions[0]?.text ?? ''}.`
        : `Gespeichert. Im Slicer zuordnen: ${slots.map((c, i) => `Extruder ${i + 1} = ${c.toUpperCase()}`).join(' · ')}`, 9000);
      return;
    }
    let body: { name: string; positions: Float32Array }[] = world;
    if ((document.getElementById('unionExport') as HTMLInputElement).checked && world.length > 1) {
      toast('Vereinige die Teile zu einem Körper …', 20000);
      await new Promise((r) => setTimeout(r, 30));
      try {
        const res = await unionParts(world, manifoldWasm);
        body = [{ name: 'PartJoin', positions: res.positions }];
        toast(res.notClosed.length
          ? `Vereinigt. Nicht geschlossen und deshalb nur angehängt: ${res.notClosed.join(', ')}`
          : 'Zu einem Körper vereinigt.', 6000);
      } catch (e) {
        toast(`Vereinigen nicht möglich (${(e as Error).message ?? e}) – Export ohne Vereinigung.`, 6000);
      }
    }
    const geomOf = (p: { positions: Float32Array }) => {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(p.positions, 3));
      return g;
    };
    const list: ExportPart[] = body.map((p) => ({ name: p.name, geometry: geomOf(p) }));
    if (kind === 'stl') await download(toSTL(list), 'zusammengefuegt.stl');
    else if (kind === 'obj') await download(toOBJ(list), 'zusammengefuegt.obj');
    else await download(new Blob([write3MF(body.map((p) => ({ ...p, color: '#cccccc' })), 'single') as Uint8Array<ArrayBuffer>], { type: 'model/3mf' }), 'zusammengefuegt.3mf');
  } catch (e) {
    toast(`Speichern nicht möglich: ${(e as Error).message ?? e}`);
  }
}

// Für automatisierte Tests / Debugging
Object.assign(window as unknown as Record<string, unknown>, {
  partjoin: {
    parts, joints, solveAll, renderAll, camera, controls, setPickMode, openGizmo, connect, tc, gizmoAnchor,
    checkCount: () => checkGroup.children.length,
    gizmoLocked: () => [...gizmoLocked],
    hoverLineVisible: () => hoverLine.visible,
    undoInfo: () => ({ undo: undoStack.map((u) => u.parts.map((e) => e.visible)), redo: redoStack.length, pending: pendingUndo?.parts.map((e) => e.visible) ?? null }),
    proposePair: (a: number, b: number) => proposePair(getPart(a)!, getPart(b)!).map((s) => ({ score: s.score, fa: s.fa.kind + ':' + Math.round((s.fa as { area?: number }).area ?? 0), fb: s.fb.kind + ':' + Math.round((s.fb as { area?: number }).area ?? 0) })),
    /** Fläche wie per Klick auswählen (für automatisierte Tests). */
    selectFace: (partId: number, tri: number) => {
      const j = activeJoint();
      const part = getPart(partId);
      const side: Side | null = !j || !part ? null : j.a === partId ? 'a' : j.b === partId ? 'b' : null;
      const helper = j && part && !side ? helperSide(j, part) : null;
      if (j && part && (side || helper)) toggleFace(j, (side ?? helper)!, part, tri, helper ? partId : undefined);
    },
  },
});

solveAll();
renderAll();
