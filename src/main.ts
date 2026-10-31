import './style.css';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { acceleratedRaycast, computeBoundsTree, disposeBoundsTree, MeshBVH } from 'three-mesh-bvh';
import { type Rigid, type Vec3, IDENTITY, compose, cross, rotate, rotationAbout, sub } from './math/align';
import { type JointResult, type Marker, type NormalMode, applyAdjustments, solveJoint } from './math/match';
import { type Sample, refineICP, sameShape, sampleSurface } from './math/icp';
import { type CylinderFeature, type EdgeFeature, type Feature, type PlaneFeature, type Topology, buildTopology, detectFeature, featureFromTris, mergeFeatures, similarity, similarityFeature, transformFeature, transformMesh } from './math/features';
import { detectEdge } from './math/edges';
import { type MFeature, type MeasureModel, centerOf, featureAt, measureBetween, measureModel, pointOn, toWorld } from './math/measure';
import { checkRange, finishCheck, newAccumulator, typicalGap } from './math/check';
import { findCandidateFaces } from './math/segment';
import { splitLoosePieces } from './math/components';
import { type MassProps, type Material, type PrintEstimate, type PrintSettings, MATERIALS, massProps, printEstimate, printedCentroid } from './math/mass';
import { type P2, convexHull, stabilityOf } from './math/stability';
import { candidatePairs, placePlanePair } from './math/suggest';
import { contactScore } from './math/scan';
import type { FreeMotion } from './math/motion';
import { describeFeature, featureTag } from './math/mate';
import { fitFeatures } from './math/fit';
import { layFlat } from './math/orient';
import { offsetCylinder, offsetPlane } from './math/offset';
import { cylinderWall } from './math/wall';
import { type LoadedPart, loadModel, SUPPORTED } from './io/load';
import { type ExportPart, download, toOBJ, toSTL } from './io/export';
import { PRINTERS, VENDORS, printerById, printerName } from './printers';
import { type ColoredPart, type ReadInfo, type SlicerExtras, bambuPlateOrigin, colorSlots } from './io/threemf';
import { compute, hasWorker } from './compute/client';
import { decorate, disposeDecor, refreshPaint } from './decor';
import {
  type Blobs, type ProjectData, PROJECT_EXT, PROJECT_EXTS, clearSession, loadSession, newProject, saveSession, sessionInfo, unpackProject,
} from './io/project';
import type { Connector } from './io/cut';
import type { MeshHealth } from './math/repair';
import { planSplit, splitPlanes } from './math/split';
import { overhangs, pickTriangles, thinWalls } from './math/printcheck';
import { COUNTERSUNK_HEAD, PRESETS, type PocketKind, type PocketSpec, pocketPreview } from './io/pocket';
import { type EmbossMode, type EmbossPlace, type EmbossResult, type EmbossSpec, bvhSurface, embossPreview } from './io/emboss';
import { type FontName, type Outline, svgOutline, textOutline } from './text';
import { labelSpot } from './math/labelspot';
import { groupIdentical, layoutBalloons } from './math/sheet';
import { brushPaint, carryPaint, decodePaint, paintStates, solidPaint, triangleDistance } from './io/paint';
import { clearanceLabel } from './io/testpiece';
import type { BoolOp } from './io/boolean';
import manifoldWasm from 'manifold-3d/manifold.wasm?url';
import { makeDemo, makeRoundDemo } from './demo';

THREE.BufferGeometry.prototype.computeBoundsTree = computeBoundsTree;
THREE.BufferGeometry.prototype.disposeBoundsTree = disposeBoundsTree;
THREE.Mesh.prototype.raycast = acceleratedRaycast;
// 3D-Druck-Konvention: Z zeigt nach oben
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

// ---------------------------------------------------------------- Zustand

/** paint, palette, modifiers, settings: Slicer-Zusätze aus 3MF-Projekten, Lage wie geom. */
interface Part extends SlicerExtras {
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
  /** Mit „Lage aus der Datei übernehmen" geladen: steht mit allen so geladenen Teilen gemeinsam auf der Platte. */
  fromFile?: boolean;
  /**
   * Teile mit gleichem Wert liegen laut Datei richtig zueinander (z. B. eine CAD-Baugruppe oder die Netze
   * einer OBJ-Datei): Sie stehen gemeinsam auf der Platte und werden zusammen ein Objekt im Slicer.
   */
  together?: string;
  /** Ergebnis der Netzprüfung (läuft nach dem Laden im Hintergrund). */
  health?: MeshHealth;
  /** Zählt Änderungen der Bemalung (für Rückgängig und die Sitzungssicherung). */
  paintRev?: number;
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
  /** Nach Skalieren/Spiegeln aller beteiligten Teile: Ergebnis gilt weiter, nur der Schlüssel ist neu. */
  rekey?: boolean;
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
/** Werkzeug im 3D-Fenster (siehe setTool) und die im Passungs-Werkzeug gewählte Fläche. */
let tool: 'lay' | 'fit' | 'measure' | 'pocket' | 'paint' | 'text' | null = null;
/**
 * Gewählte Fläche der Passung. wall: ganze Wand einer Bohrung bzw. eines Zapfens (Weltkoordinaten) – auch
 * Stücke in angrenzenden Teilen, z. B. wenn ein Schnitt durch die Bohrung geht.
 */
let fitTarget: {
  part: Part;
  f: CylinderFeature | PlaneFeature;
  wall?: { center: Vec3; axis: Vec3; radius: number; span: [number, number]; pieces: { part: Part; tris: number[] }[] };
} | null = null;
/** Ort der Aussparung (Werkzeug „Aussparung“, Koordinaten des Teils). */
let pocketPlace: { partId: number; point: Vec3; normal: Vec3 } | null = null;
/** Beschriften: Text, Logo oder Nummern – und die angeklickte Stelle (Koordinaten des Teils). */
let textSrc: 'text' | 'logo' | 'number' = 'text';
let textPlace: { partId: number; place: EmbossPlace } | null = null;
/** Abschnitt mit den ausgeblendeten Teilen eingeklappt. */
let hiddenFolded = false;
/** Ausgewählte Teile (Mehrfachauswahl) und die zuletzt angeklickte Zeile für Umschalt+Klick. */
const selected = new Set<number>();
let selAnchor: number | null = null;
/** Teileliste eingeklappt (bleibt im Browser gespeichert). */
let partsFolded = false;
// Gespeicherte Einstellungen heißen noch „partjoin.…“ (früherer Name) – so bleiben sie erhalten
try { partsFolded = localStorage.getItem('partjoin.partsFolded') === '1'; } catch { /* ohne Speicher */ }
const SNAP_PX = 14;
/** Auch sehr fein aufgelöste Flächen (Millionen Dreiecke) vollständig erfassen. */
const FEATURE_OPTS = { maxTris: 4_000_000 };
/** Auswahlmodus im 3D-Fenster: Flächen (Standard), Kanten, Punkte oder Polygone (Fläche selbst bemalen). */
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
/** Um so viel hat der feste Boden jedes Teil angehoben (z, nach dem letzten solveAll). */
const floorShift = new Map<number, number>();
/**
 * Baugruppe je Teil (nach dem letzten solveAll): Teile, die über Verbindungen oder Gruppen zusammenhängen,
 * dazu alle mit der Lage aus der Datei geladenen. Jede steht für sich auf der Platte und wird im
 * Bambu-Export ein eigenes Objekt. Wert: Id eines ihrer Teile.
 */
const unitOf = new Map<number, number>();

// ---------------------------------------------------------------- Szene

const view = document.getElementById('view')!;
const onFloor = document.getElementById('onFloor') as HTMLInputElement;
const partsEl = document.getElementById('parts')!;
const jointsEl = document.getElementById('joints')!;
// Stencil-Puffer für die Darstellung farbiger Modifier (decor.ts)
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true, stencil: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
// Schnittansicht: Schnittebene je Material
renderer.localClippingEnabled = true;
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
/** Außenmaße im Bild (Taste D) */
let dimsOn = false;

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
/** Rahmen um ausgewählte Teile. */
const selGroup = new THREE.Group();
scene.add(selGroup);
const selBar = document.getElementById('selBar')!;

/** Akzentfarbe des Designs – zwischengespeichert: getComputedStyle nach Änderungen an der Seite erzwingt ein neues Layout. */
let accentColor = '#2f6fde';
function applyTheme() {
  const dark = matchMedia('(prefers-color-scheme: dark)').matches &&
    document.documentElement.dataset.theme !== 'light' || document.documentElement.dataset.theme === 'dark';
  const css = getComputedStyle(document.documentElement);
  const bg = css.getPropertyValue('--view').trim();
  accentColor = css.getPropertyValue('--accent').trim() || '#2f6fde';
  renderer.setClearColor(new THREE.Color(bg || (dark ? '#161a21' : '#e9ecf1')));
}
applyTheme();
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applyTheme);
new MutationObserver(applyTheme).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });

/** Höhe der Zeichenfläche (aus resize – clientHeight zu lesen erzwänge nach Änderungen an der Seite ein Layout). */
let viewHeight = 1;
function resize() {
  const w = view.clientWidth;
  const h = view.clientHeight;
  viewHeight = Math.max(1, h);
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);
resize();

/**
 * Drehen und Verschieben der Ansicht ohne Bildschirmrand: Stößt der Mauszeiger beim Ziehen an den Rand des
 * Fensters, wird er festgehalten (Pointer Lock) – die Bewegung geht unbegrenzt weiter, bis die Taste losgelassen
 * wird. Lässt der Browser das nicht zu (z. B. in einer eingebetteten Vorschau), dreht die Ansicht weiter,
 * solange der Zeiger am Rand bleibt. Gerechnet wird wie in OrbitControls selbst.
 */
{
  const orbit = controls as unknown as { state: number; _rotateLeft?(a: number): void; _rotateUp?(a: number): void; _pan?(x: number, y: number): void };
  const ROTATE = 0;
  const PAN = 2;
  const canvas = renderer.domElement;
  const locked = () => document.pointerLockElement === canvas;
  const usable = () => !!orbit._rotateLeft && !!orbit._rotateUp && !!orbit._pan;
  /**
   * Laufendes Ziehen der Ansicht – selbst gemerkt, denn manche Browser beenden das Ziehen von OrbitControls,
   * sobald der Zeiger festgehalten wird. asked: in diesem Zug schon angefragt (nur einmal – eine zweite Anfrage,
   * solange die erste läuft, lehnt der Browser ab); edge: in diesem Zug dreht stattdessen der Rand weiter.
   */
  let drag: { mode: number; x: number; y: number; asked: boolean; edge: boolean; failed?: boolean } | null = null;
  /** Festhalten geht hier nicht (zweimal hintereinander abgelehnt oder nicht vorhanden): dann dreht der Rand weiter. */
  let noLock = typeof canvas.requestPointerLock !== 'function';
  let failures = 0;
  const lockFailed = () => {
    // Je Zug einmal zählen (Fehler-Ereignis und abgelehntes Promise kommen beide)
    if (!drag || drag.failed) return;
    drag.failed = true;
    drag.edge = true;
    if (++failures < 2 || noLock) return;
    noLock = true;
    toast('Der Browser lässt den Mauszeiger hier nicht festhalten – halte ihn beim Drehen am Rand, dann dreht die Ansicht weiter.', 7000);
  };
  const move = (dx: number, dy: number) => {
    if (!drag || !usable()) return;
    if (drag.mode === ROTATE) {
      orbit._rotateLeft!((2 * Math.PI * dx * controls.rotateSpeed) / viewHeight);
      orbit._rotateUp!((2 * Math.PI * dy * controls.rotateSpeed) / viewHeight);
    } else orbit._pan!(dx * controls.panSpeed, dy * controls.panSpeed);
    controls.update();
  };
  /** Richtung, in der der Zeiger am Fensterrand steht (0, wenn nicht). */
  const edgeOf = (x: number, y: number) => {
    const m = 3;
    return { x: x <= m ? -1 : x >= innerWidth - 1 - m ? 1 : 0, y: y <= m ? -1 : y >= innerHeight - 1 - m ? 1 : 0 };
  };
  controls.addEventListener('start', () => {
    drag = orbit.state === ROTATE || orbit.state === PAN ? { mode: orbit.state, x: -1, y: -1, asked: false, edge: noLock } : null;
  });
  document.addEventListener('pointerlockerror', () => { if (drag) lockFailed(); });
  document.addEventListener('pointerlockchange', () => {
    if (locked()) failures = 0;
    // Mit Esc freigegeben, Taste noch unten: in diesem Zug am Rand weiterdrehen
    else if (drag) drag.edge = true;
  });
  // Erst hier, dann OrbitControls: Während der Zeiger festgehalten ist, sieht es keine Bewegung (manche Browser
  // melden dann eine andere Position, das gäbe einen Sprung)
  canvas.addEventListener('pointermove', (e) => {
    if (locked()) { e.stopImmediatePropagation(); return; }
    if (!drag || e.pointerType !== 'mouse' || !usable()) return;
    drag.x = e.clientX;
    drag.y = e.clientY;
    const edge = edgeOf(e.clientX, e.clientY);
    if (!(edge.x || edge.y) || drag.edge || drag.asked) return;
    drag.asked = true;
    try {
      // Neuere Browser geben ein Promise zurück, das abgelehnt wird, wenn es nicht erlaubt ist
      (canvas.requestPointerLock() as unknown as Promise<void> | undefined)?.catch?.(() => { if (drag) lockFailed(); });
    } catch {
      lockFailed();
    }
  }, { capture: true });
  document.addEventListener('mousemove', (e) => {
    if (!locked() || !drag) return;
    // Manche Browser melden direkt nach dem Festhalten einen Sprung – den nicht mitnehmen
    if (Math.abs(e.movementX) > 300 || Math.abs(e.movementY) > 300) return;
    move(e.movementX, e.movementY);
  });
  // Ohne Festhalten: Zeiger am Rand dreht weiter (wie 250 Pixel Ziehen je Sekunde – eine Umdrehung in gut 3 s)
  let last = performance.now();
  const edgeStep = (t: number) => {
    requestAnimationFrame(edgeStep);
    const dt = Math.min(0.05, Math.max(0, (t - last) / 1000));
    last = t;
    if (!drag || locked() || !drag.edge || drag.x < 0) return;
    const edge = edgeOf(drag.x, drag.y);
    if (edge.x || edge.y) move(edge.x * 250 * dt, edge.y * 250 * dt);
  };
  requestAnimationFrame(edgeStep);
  const end = () => {
    drag = null;
    if (locked()) document.exitPointerLock();
  };
  document.addEventListener('pointerup', end);
  document.addEventListener('mouseup', end);
  window.addEventListener('blur', end);
}

/** Markierungen behalten auf dem Bildschirm eine feste Größe (in Pixeln). */
function scaleMarkers() {
  const k = (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2))) / viewHeight;
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
  stepViewAnim();
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
  applySectionClip();
  const undo = explodeForRender();
  renderer.render(scene, camera);
  undo();
  updateCube();
  placeMeasureLabels();
  placeLabelList(standLabelAt);
  placeLabelList(dimLabelAt, true);
  renderActive();
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
  parts: { part: Part; layout: THREE.Matrix4; visible: boolean; color: string; name: string; group?: string; paint?: Record<string, string[]>; palette?: string[]; paintRev: number }[];
  joints: Joint[];
  baseId: number | null;
  activeJointId: number | null;
  plates: number;
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
    plateTotal,
    parts.map((p) => [p.id, p.geom.uuid, p.name, p.visible, p.color.getHexString(), p.group ?? '', p.layout.elements.map(r), p.paintRev ?? 0]),
    joints.map((j) => [
      j.id, j.a, j.b, j.applied, j.alt, j.seat, j.ordered, j.normals, j.overlap, j.spin, j.turnIdx ?? 0, j.proposalIdx ?? 0,
      j.adjust ? [...j.adjust.R, ...j.adjust.t].map(r) : 0,
      j.ma.map((m) => m.p.map(r)), j.mb.map((m) => m.p.map(r)), j.fa.map(fs), j.fb.map(fs),
    ]),
  ]);
}

function snapshot(): Snapshot {
  return {
    parts: parts.map((p) => ({
      part: p, layout: p.layout.clone(), visible: p.visible, color: `#${p.color.getHexString()}`, name: p.name, group: p.group,
      paint: p.paint, palette: p.palette, paintRev: p.paintRev ?? 0,
    })),
    joints: joints.map(jointCopy),
    baseId,
    activeJointId,
    plates: plateTotal,
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
    // Bemalung (jeder Strich ersetzt die Liste, die alte bleibt hier erhalten)
    if ((p.paintRev ?? 0) !== e.paintRev) {
      p.paint = e.paint;
      p.palette = e.palette;
      p.paintRev = e.paintRev;
      refreshPaint(p.mesh, p);
    }
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
  plateTotal = snap.plates;
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
    disposeDecor(p.mesh);
    p.geom.disposeBoundsTree();
    p.geom.dispose();
    (p.mesh.material as THREE.Material).dispose();
    removedParts.delete(p);
  }
}

/**
 * Speicher für alte Netze in der Historie begrenzen: Bei großen Netzen (Scans mit Millionen Dreiecken)
 * behält jede Reparatur, jedes Vereinfachen oder Schneiden das vorige Netz – ohne Grenze läuft der Tab voll.
 */
let undoBytes = 800e6;
let undoTrimmed = false;
function trimHistory() {
  const retained = () => {
    const current = new Set(parts);
    const held = new Set<Part>();
    for (const s of [...undoStack, ...redoStack]) for (const e of s.parts) if (!current.has(e.part)) held.add(e.part);
    // Positionen, Normalen und Suchbaum (grob)
    let bytes = 0;
    for (const p of held) bytes += 3 * positionsOf(p).byteLength;
    return bytes;
  };
  let dropped = false;
  while (undoStack.length > 1 && retained() > undoBytes) {
    undoStack.shift();
    dropped = true;
  }
  if (!dropped) return;
  releaseUnused();
  undoTrimmed = true;
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
    trimHistory();
  }
  pendingUndo = null;
}

function undo() {
  commitAction();
  const s = undoStack.pop();
  if (!s) {
    toast(undoTrimmed ? 'Nichts mehr rückgängig zu machen – ältere Schritte wurden wegen der großen Netze nicht behalten.' : 'Nichts mehr rückgängig zu machen.', undoTrimmed ? 4000 : 1800);
    return;
  }
  redoStack.push(snapshot());
  restoreSnapshot(s);
  toast('Rückgängig gemacht · ↷ oder Strg/⌘+Umschalt+Z stellt es wieder her', 2200);
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
document.getElementById('undoBtn')!.addEventListener('click', () => undo());
document.getElementById('redoBtn')!.addEventListener('click', () => redo());
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

/** WASM-Datei von manifold-3d als absolute Adresse (oder data:-URL) – der Hintergrund-Thread kennt keine relativen Pfade. */
const manifoldUrl = new URL(manifoldWasm, location.href).href;
/** Kennung je Netz: Der Hintergrund-Thread hält den Körper fürs Zerschneiden, solange sich das Netz nicht ändert. */
const meshKeys = new WeakMap<Float32Array, string>();
let meshCount = 0;
function meshKey(positions: Float32Array): string {
  let k = meshKeys.get(positions);
  if (!k) meshKeys.set(positions, (k = `netz-${++meshCount}`));
  return k;
}
/** Fehler mit einer Meldung für den Nutzer (aus io/cut, io/pocket, io/boolean – auch aus dem Hintergrund-Thread). */
const userError = (err: unknown, ...names: string[]) => names.includes((err as Error)?.name);

/** Tiefster Punkt eines Teils in der Lage M (Welt-z); je Drehung zwischengespeichert, Verschieben kostet nichts. */
const lowCache = new WeakMap<Part, { key: string; z: number }>();
function lowestZ(p: Part, M: THREE.Matrix4): number {
  const e = M.elements;
  const key = `${e[2]},${e[6]},${e[10]}`;
  let c = lowCache.get(p);
  if (c?.key !== key) {
    const pos = positionsOf(p);
    let z = Infinity;
    for (let i = 0; i < pos.length; i += 3) z = Math.min(z, e[2] * pos[i] + e[6] * pos[i + 1] + e[10] * pos[i + 2]);
    c = { key, z };
    lowCache.set(p, c);
  }
  return c.z + e[14];
}

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
  placeToast(el);
  clearTimeout(toastTimer);
  toastTimer = window.setTimeout(() => el.classList.remove('show'), ms);
}

/** Meldung über Hinweis und Leisten unten rechts halten, statt sie zu überdecken. */
function placeToast(el: HTMLElement) {
  el.style.bottom = '';
  const v = view.getBoundingClientRect();
  const t = el.getBoundingClientRect();
  let bottom = v.bottom - t.bottom;
  const others = ['hint', 'explodeBar', 'sectionBar'].map((id) => document.getElementById(id)!).filter((o) => !o.hidden && o.textContent?.trim());
  // Zweimal: Hinweis und Leisten können übereinander liegen
  for (let k = 0; k < 2; k++) {
    for (const o of others) {
      const r = o.getBoundingClientRect();
      if (!r.width || r.left >= t.right || r.right <= t.left) continue;
      const top = v.bottom - bottom - t.height;
      if (r.top < v.bottom - bottom + 8 && r.bottom > top - 8) bottom = v.bottom - r.top + 8;
    }
  }
  el.style.bottom = `${bottom}px`;
}

/**
 * Umriss eines Teils in Weltkoordinaten (ohne angehängte Darstellung wie Modifier) – aus den Eckpunkten,
 * nicht aus dem gedrehten Umrissquader, der bei schräg liegenden Teilen zu groß wäre. Je Drehung
 * zwischengespeichert, Verschieben kostet nichts.
 */
const boxCache = new WeakMap<Part, { key: string; min: THREE.Vector3; max: THREE.Vector3 }>();
function partBox(p: Part): THREE.Box3 {
  p.mesh.updateWorldMatrix(false, false);
  const e = p.mesh.matrixWorld.elements;
  const key = `${e[0]},${e[1]},${e[2]},${e[4]},${e[5]},${e[6]},${e[8]},${e[9]},${e[10]}`;
  let c = boxCache.get(p);
  if (c?.key !== key) {
    const pos = positionsOf(p);
    const mn = [Infinity, Infinity, Infinity];
    const mx = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < pos.length; i += 3) {
      const x = pos[i], y = pos[i + 1], z = pos[i + 2];
      for (let k = 0; k < 3; k++) {
        const w = e[k] * x + e[4 + k] * y + e[8 + k] * z;
        if (w < mn[k]) mn[k] = w;
        if (w > mx[k]) mx[k] = w;
      }
    }
    c = { key, min: new THREE.Vector3(...mn), max: new THREE.Vector3(...mx) };
    boxCache.set(p, c);
  }
  const t = new THREE.Vector3(e[12], e[13], e[14]);
  return new THREE.Box3(c.min.clone().add(t), c.max.clone().add(t));
}

function fitView() {
  const box = new THREE.Box3();
  for (const p of parts) if (p.visible) box.union(partBox(p));
  if (box.isEmpty()) return;
  fitBox(box);
}

/** Kamera so, dass der Umriss ganz zu sehen ist (Blickrichtung bleibt). */
function fitBox(box: THREE.Box3) {
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

// ---------------------------------------------------------------- Ansichtswürfel

/**
 * Würfel oben rechts, der sich mit der Ansicht dreht (CSS 3D). Fläche, Kante oder Ecke anklicken
 * schwenkt die Kamera dorthin – um den Drehpunkt, mit gleichem Abstand. Vorne ist −Y, oben +Z.
 */
const CUBE_FACES: { name: string; n: THREE.Vector3Tuple; r: THREE.Vector3Tuple; u: THREE.Vector3Tuple }[] = [
  // r × u = n: von außen gesehen rechts und oben
  { name: 'Vorne', n: [0, -1, 0], r: [1, 0, 0], u: [0, 0, 1] },
  { name: 'Hinten', n: [0, 1, 0], r: [-1, 0, 0], u: [0, 0, 1] },
  { name: 'Rechts', n: [1, 0, 0], r: [0, 1, 0], u: [0, 0, 1] },
  { name: 'Links', n: [-1, 0, 0], r: [0, -1, 0], u: [0, 0, 1] },
  { name: 'Oben', n: [0, 0, 1], r: [1, 0, 0], u: [0, 1, 0] },
  { name: 'Unten', n: [0, 0, -1], r: [1, 0, 0], u: [0, -1, 0] },
];
/** Tasten wie in Bambu Studio: 0 Schrägansicht, 1 oben, 2 unten, 3 vorne, 4 hinten, 5 links, 6 rechts. */
const VIEW_KEYS: Record<string, THREE.Vector3Tuple> = {
  0: [0.6, -0.8, 0.6], 1: [0, 0, 1], 2: [0, 0, -1], 3: [0, -1, 0], 4: [0, 1, 0], 5: [-1, 0, 0], 6: [1, 0, 0],
};
const cubeEl = document.getElementById('viewCube')!;
const cubeInner = cubeEl.querySelector<HTMLElement>('.cube')!;
const CUBE_HALF = 30;
cubeInner.innerHTML = CUBE_FACES.map(({ name, n, r, u }) => {
  const m = [r[0], r[1], r[2], 0, -u[0], -u[1], -u[2], 0, n[0], n[1], n[2], 0, n[0] * CUBE_HALF, n[1] * CUBE_HALF, n[2] * CUBE_HALF, 1];
  const cells: string[] = [];
  for (const j of [1, 0, -1]) {
    for (const i of [-1, 0, 1]) {
      const d = [0, 1, 2].map((k) => n[k] + i * r[k] + j * u[k]);
      cells.push(`<span data-dir="${d.join(',')}">${i === 0 && j === 0 ? name : ''}</span>`);
    }
  }
  return `<div class="face" style="transform: matrix3d(${m.join(',')})">${cells.join('')}</div>`;
}).join('');

let cubeTransform = '';
const cubeMatrix = new THREE.Matrix4();
/** Würfel nach der Kamera drehen: Welt → Ansicht, dann y nach unten wie in CSS. */
function updateCube() {
  cubeMatrix.extractRotation(camera.matrixWorldInverse).premultiply(new THREE.Matrix4().makeScale(1, -1, 1));
  const t = `matrix3d(${Array.from(cubeMatrix.elements, (v) => v.toFixed(4)).join(',')})`;
  if (t !== cubeTransform) cubeInner.style.transform = cubeTransform = t;
}

let viewAnim: { from: THREE.Vector3; q: THREE.Quaternion; t0: number; ms: number } | null = null;
/** Kamera auf die Richtung dir (vom Drehpunkt aus) schwenken. */
function viewFrom(dir: THREE.Vector3Tuple) {
  const to = new THREE.Vector3(...dir).normalize();
  // Genau von oben oder unten: vorne soll unten (bzw. oben) im Bild liegen
  if (Math.abs(to.z) > 0.9999) to.set(0, -1e-3, Math.sign(to.z)).normalize();
  const from = camera.position.clone().sub(controls.target).normalize();
  const angle = from.angleTo(to);
  if (angle < 1e-4) return;
  // Schwung der letzten Mausbewegung verwerfen, sonst dreht die Ansicht danach weiter
  const c = controls as unknown as { _sphericalDelta?: THREE.Spherical; _panOffset?: THREE.Vector3 };
  c._sphericalDelta?.set(0, 0, 0);
  c._panOffset?.set(0, 0, 0);
  viewAnim = { from, q: new THREE.Quaternion().setFromUnitVectors(from, to), t0: performance.now(), ms: 180 + 220 * (angle / Math.PI) };
}
/** Ein Schritt der Schwenkbewegung (in der Zeichenschleife). */
function stepViewAnim() {
  if (!viewAnim) return;
  const t = Math.min(1, (performance.now() - viewAnim.t0) / viewAnim.ms);
  const e = t < 0.5 ? 2 * t * t : 1 - (-2 * t + 2) ** 2 / 2;
  const dir = viewAnim.from.clone().applyQuaternion(new THREE.Quaternion().slerp(viewAnim.q, e));
  const dist = camera.position.distanceTo(controls.target);
  camera.position.copy(controls.target).addScaledVector(dir, dist);
  if (t >= 1) viewAnim = null;
}
// Selbst drehen oder schieben beendet das Schwenken
controls.addEventListener('start', () => { viewAnim = null; });

/** Ansicht um den Drehpunkt drehen (Ziehen am Würfel): wie Ziehen in der Ansicht. */
function orbitBy(dTheta: number, dPhi: number) {
  const o = camera.position.clone().sub(controls.target);
  const r = o.length();
  const theta = Math.atan2(o.y, o.x) - dTheta;
  const phi = THREE.MathUtils.clamp(Math.acos(THREE.MathUtils.clamp(o.z / r, -1, 1)) - dPhi, 1e-3, Math.PI - 1e-3);
  o.set(r * Math.sin(phi) * Math.cos(theta), r * Math.sin(phi) * Math.sin(theta), r * Math.cos(phi));
  camera.position.copy(controls.target).add(o);
}

let cubeDrag: { x: number; y: number; moved: boolean; cell: HTMLElement | null } | null = null;
const cubeCell = (t: EventTarget | null) => (t as HTMLElement | null)?.closest<HTMLElement>('[data-dir]') ?? null;
function highlightCell(cell: HTMLElement | null) {
  for (const el of cubeInner.querySelectorAll('.hl')) el.classList.remove('hl');
  // Kanten und Ecken gehören zu mehreren Flächen: alle Teile hervorheben
  if (cell) for (const el of cubeInner.querySelectorAll(`[data-dir="${cell.dataset.dir}"]`)) el.classList.add('hl');
}
cubeEl.addEventListener('pointerdown', (e) => {
  if (e.button !== 0 || (e.target as HTMLElement).closest('button')) return;
  cubeDrag = { x: e.clientX, y: e.clientY, moved: false, cell: cubeCell(e.target) };
  cubeEl.setPointerCapture(e.pointerId);
});
cubeEl.addEventListener('pointermove', (e) => {
  if (!cubeDrag) {
    highlightCell(cubeCell(e.target));
    return;
  }
  const dx = e.clientX - cubeDrag.x, dy = e.clientY - cubeDrag.y;
  if (!cubeDrag.moved && Math.hypot(dx, dy) < 4) return;
  if (!cubeDrag.moved) {
    viewAnim = null;
    highlightCell(null);
  }
  cubeDrag.moved = true;
  cubeDrag.x = e.clientX;
  cubeDrag.y = e.clientY;
  orbitBy(dx * Math.PI / 110, dy * Math.PI / 110);
});
const endCubeDrag = (e: PointerEvent) => {
  if (!cubeDrag) return;
  const { moved, cell } = cubeDrag;
  cubeDrag = null;
  if (cubeEl.hasPointerCapture(e.pointerId)) cubeEl.releasePointerCapture(e.pointerId);
  if (!moved && cell && e.type === 'pointerup') viewFrom(cell.dataset.dir!.split(',').map(Number) as THREE.Vector3Tuple);
};
cubeEl.addEventListener('pointerup', endCubeDrag);
cubeEl.addEventListener('pointercancel', endCubeDrag);
cubeEl.addEventListener('pointerleave', () => { if (!cubeDrag) highlightCell(null); });
document.getElementById('viewHome')!.addEventListener('click', () => viewFrom(VIEW_KEYS[0]));

// ---------------------------------------------------------------- Teile

/** Platz rechts neben den bereits geladenen Teilen für einen Bereich mit diesem Umriss. */
function nextLayout(bb: THREE.Box3): THREE.Matrix4 {
  let x = 0;
  for (const p of parts) x = Math.max(x, partBox(p).max.x);
  const gap = parts.length ? sceneScale() * 0.3 : 0;
  return new THREE.Matrix4().makeTranslation(
    parts.length ? x + gap - bb.min.x : -(bb.min.x + bb.max.x) / 2,
    -(bb.min.y + bb.max.y) / 2,
    -bb.min.z,
  );
}

function addPart(
  name: string, geom: THREE.BufferGeometry,
  opts: { color?: string; group?: string; layout?: THREE.Matrix4; extras?: SlicerExtras; samples?: Sample[]; id?: number; topo?: Topology } = {},
) {
  if (!geom.boundsTree) geom.computeBoundsTree();
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
    id: opts.id ?? nextPartId++, name, geom, mesh, color, layout, visible: true, samples: opts.samples ?? sampleSurface(geom, 6000), group: opts.group, ...opts.extras,
  };
  if (opts.topo) part.topo = opts.topo;
  decorate(mesh, part);
  mesh.userData.partId = part.id;
  parts.push(part);
  scene.add(mesh);
  if (baseId === null) baseId = part.id;
  checkHealth(part);
  return part;
}

/** Netz im Hintergrund prüfen; offene Netze bekommen ein Warnzeichen in der Teileliste. */
function checkHealth(p: Part) {
  // Erst nach dem Laden der übrigen Dateien, damit das Laden nicht wartet
  setTimeout(() => {
    if (p.health || !parts.includes(p)) return;
    compute('meshHealth', { positions: positionsOf(p) }).then((h) => {
      p.health = h;
      if (parts.includes(p) && !h.closed) { renderParts(); renderStatus(); renderMaterial(); }
    }).catch(() => { /* ohne Prüfung */ });
  }, 50);
}

/** Kurzbeschreibung der Netzfehler. */
function healthText(h: MeshHealth): string {
  const bits = [
    h.openEdges ? `${h.openEdges} offene Kante${h.openEdges > 1 ? 'n' : ''}` : '',
    h.flippedEdges ? `${h.flippedEdges} verdrehte Kante${h.flippedEdges > 1 ? 'n' : ''}` : '',
    h.nonManifoldEdges ? `${h.nonManifoldEdges} mehrdeutige Kante${h.nonManifoldEdges > 1 ? 'n' : ''}` : '',
    h.duplicates ? `${h.duplicates} doppelte Dreieck${h.duplicates > 1 ? 'e' : ''}` : '',
    h.volume < 0 && !h.openEdges ? 'Normalen zeigen nach innen' : '',
  ].filter(Boolean);
  return bits.join(', ') || 'in Ordnung';
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

/** Laden nacheinander: Dateien, die während des Ladens dazukommen, warten (sonst lägen sie übereinander). */
let loadQueue: Promise<void> = Promise.resolve();
function loadFiles(files: FileList | File[]): Promise<void> {
  const list = Array.from(files);
  loadQueue = loadQueue.then(() => loadList(list)).catch((e) => toast(`Laden fehlgeschlagen: ${(e as Error).message}`));
  return loadQueue;
}

async function loadList(list: File[]) {
  const before = parts.length;
  const split = (document.getElementById('splitParts') as HTMLInputElement).checked;
  const keep = (document.getElementById('keepPos') as HTMLInputElement).checked;
  // Farben für OBJ: passende .mtl-Datei (gleicher Name oder die einzige mitgegebene)
  const mtls = new Map<string, string>();
  for (const f of list) if (f.name.toLowerCase().endsWith('.mtl')) mtls.set(f.name.replace(/\.[^.]+$/, '').toLowerCase(), await f.text());
  // Projekt (auch alte .partjoin): ersetzt die aktuelle Arbeit, weitere Dateien kommen dazu
  const project = list.find((f) => PROJECT_EXTS.some((x) => f.name.toLowerCase().endsWith(`.${x}`)));
  if (project) {
    try {
      const { data, blobs } = unpackProject(new Uint8Array(await project.arrayBuffer()));
      openProject(data, blobs);
      toast(`Projekt „${project.name}" geöffnet${undoStack.length || pendingUndo ? ' – Strg+Z holt die vorige Arbeit zurück' : ''}.`, 5000);
    } catch (e) {
      toast(`${project.name}: ${(e as Error).message}`);
    }
    if (list.length === 1) return;
  }
  for (const file of list) {
    if (file === project || file.name.toLowerCase().endsWith('.mtl')) continue;
    try {
      const stem = file.name.replace(/\.[^.]+$/, '').toLowerCase();
      const mtl = mtls.get(stem) ?? (mtls.size === 1 ? [...mtls.values()][0] : undefined);
      const info: ReadInfo = {};
      // Große Dateien brauchen einen Moment: sagen, dass etwas passiert
      if (file.size > 4e6 || /\.(step|stp|iges|igs|brep|brp)$/i.test(file.name)) {
        toast(`Lade „${file.name}“${file.size > 4e6 ? ` (${(file.size / 1e6).toFixed(0)} MB)` : ''} …`, 120000);
        await new Promise((r) => setTimeout(r, 30));
      }
      let loaded: LoadedPart[] = await loadModel(file, { split, mtl, info });
      // Drucker eines Bambu-Projekts vorauswählen (Teile, die schon da sind, bleiben auf ihrer Platte)
      if (info.printer && info.printer !== bambuPrinter.value) changePrinter(info.printer);
      // Stecken mehrere lose Teile in einem Netz (z. B. eine ganze Druckplatte), diese trennen
      let separated = false;
      // Nachbarschaften aus der Prüfung weiterverwenden (sonst beim ersten Zeigen auf das Teil nochmal)
      const topos = new Map<LoadedPart, Topology>();
      if (split) {
        const members = new Map<string, number>();
        for (const l of loaded) if (l.group) members.set(l.group, (members.get(l.group) ?? 0) + 1);
        loaded = loaded.flatMap((l) => {
          // Bemalung und Modifier gehören zu genau diesem Netz – solche Teile bleiben ganz
          if (l.paint || l.modifiers) return [l];
          const found: { topo?: Topology } = {};
          const pieces = splitLoosePieces(l.geom.getAttribute('position').array as Float32Array, {}, found);
          if (pieces.length < 2) {
            if (found.topo) topos.set(l, found.topo);
            return [l];
          }
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
            return { name: `${l.name} ${k + 1}`, geom: g, color: l.color, group, together: l.together, plate: l.plate };
          });
        });
      }
      // Teile einer Datei gemeinsam verschieben, damit ihre Lage zueinander erhalten bleibt
      const bb = new THREE.Box3();
      for (const l of loaded) bb.union(l.geom.boundingBox!);
      const layout = keep ? new THREE.Matrix4() : nextLayout(bb);
      // Bambu-Projekt mit Platten: jedes Objekt auf seine Platte, an dieselbe Stelle wie in Bambu Studio;
      // die Platten kommen hinter die schon belegten
      let plateLayout: ((l: LoadedPart) => THREE.Matrix4) | null = null;
      if (info.plates && loaded.some((l) => l.plate !== undefined)) {
        const n = info.plates;
        const fileBed = info.bed ?? currentPrinter().bed;
        const occupied = [...occupiedPlates()];
        const offset = parts.length && occupied.length ? Math.max(...occupied) + 1 : 0;
        // Leere Arbeitsfläche: genau die Platten des Projekts
        relayoutPlates(parts.length ? Math.max(plateTotal, offset + n) : n);
        plateLayout = (l) => {
          const i = Math.min(n - 1, l.plate ?? 0);
          const [ox, oy] = bambuPlateOrigin(i, n, fileBed);
          const c = plateCenter(offset + i);
          return new THREE.Matrix4().makeTranslation(c.x - ox - fileBed[0] / 2, c.y - oy - fileBed[1] / 2, 0);
        };
      }
      // Suchbaum und Oberflächenpunkte vorab, mit kurzen Pausen: große Dateien blockieren die Oberfläche nicht
      const samples: Sample[][] = [];
      let lastYield = performance.now();
      for (const l of loaded) {
        l.geom.computeBoundsTree();
        samples.push(sampleSurface(l.geom, 6000));
        if (performance.now() - lastYield > 60) {
          await new Promise((r) => setTimeout(r, 0));
          lastYield = performance.now();
        }
      }
      for (const [i, l] of loaded.entries()) {
        const { paint, palette, modifiers, settings, objectSettings } = l;
        const part = addPart(l.name, l.geom, {
          color: l.color, group: l.group, layout: plateLayout ? plateLayout(l) : layout, samples: samples[i], topo: topos.get(l), extras: { paint, palette, modifiers, settings, objectSettings },
        });
        // Auf Platten verteilt: jedes Objekt bleibt für sich (nicht alle Teile der Datei gemeinsam)
        if (keep && !plateLayout) part.fromFile = true;
        if (l.together) part.together = l.together;
      }
      if (loaded.length > 1) toast(`${file.name}: ${loaded.length} Teile geladen${plateLayout && info.plates! > 1 ? ` – auf ${info.plates} Platten wie im Slicer` : ''}${separated ? ' (lose Teile getrennt)' : ''}.`, 5000);
    } catch (e) {
      toast(`${file.name}: ${(e as Error).message}`);
    }
  }
  ensureJoint();
  solveAll();
  fitView();
  renderAll();
  // Ladehinweis nicht stehen lassen
  const note = document.getElementById('toast')!;
  if (note.classList.contains('show') && note.textContent?.startsWith('Lade „')) {
    const n = Math.max(0, parts.length - before);
    toast(n ? `Geladen: ${n} Teil${n === 1 ? '' : 'e'}.` : 'Geladen.', 2000);
  }
  // Markante Flächen schon jetzt im Hintergrund suchen: „Vorschlagen" ist dann sofort da
  if (hasWorker()) void ensureCandidates(parts.filter((p) => p.visible)).catch(() => {});
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
/** Zählt jede Neuberechnung der Lage (für Zwischenspeicher, die von ihr abhängen). */
let solveCount = 0;
function solveAll() {
  solveCount++;
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

  // Baugruppen: über Verbindungen und Gruppen platzierte Teile – dazu bleiben Teile, die laut Datei
  // zusammengehören, untereinander, wie sie geladen wurden (z. B. Kopf über Körper, eine CAD-Baugruppe):
  // Baugruppen, in denen solche Teile stecken, zählen als eine.
  unitOf.clear();
  const root = new Map<number, number>();
  const find = (x: number): number => {
    const r = root.get(x) ?? x;
    if (r === x) return x;
    const top = find(r);
    root.set(x, top);
    return top;
  };
  const first = new Map<string, number>();
  for (const p of parts) {
    const label = togetherOf(p);
    if (!label) continue;
    const seed = seedOf.get(p.id) ?? p.id;
    const other = first.get(label);
    if (other === undefined) first.set(label, seed);
    else if (find(seed) !== find(other)) root.set(find(seed), find(other));
  }
  for (const p of parts) unitOf.set(p.id, find(seedOf.get(p.id) ?? p.id));
  const key = (p: Part) => unitOf.get(p.id)!;
  // Fester Boden: jede Baugruppe steht mit ihrem tiefsten Punkt auf der Druckplatte (z = 0)
  floorShift.clear();
  if (onFloor.checked) {
    const low = new Map<number, number>();
    for (const p of parts) low.set(key(p), Math.min(low.get(key(p)) ?? Infinity, lowestZ(p, placed.get(p.id) ?? p.layout)));
    for (const p of parts) {
      const z = low.get(key(p))!;
      if (Number.isFinite(z) && z !== 0) floorShift.set(p.id, -z);
    }
    // Freie Bewegungen der Verbindungen liegen in Weltkoordinaten: mit anheben
    for (const j of joints) {
      const dz = j.a !== null ? floorShift.get(j.a) : undefined;
      if (dz && j.free) j.free = j.free.map((m) => (m.kind === 'rotate' ? { ...m, point: [m.point[0], m.point[1], m.point[2] + dz] as Vec3 } : m));
    }
  }
  for (const p of parts) {
    p.mesh.matrix.copy(placed.get(p.id) ?? p.layout);
    const dz = floorShift.get(p.id);
    if (dz) p.mesh.matrix.premultiply(new THREE.Matrix4().makeTranslation(0, 0, dz));
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
/** Zusammenfassen lassen sich alle Flächen, nur Kanten nicht. */
const mergeable = (f: Feature) => f.kind !== 'edge';

/**
 * Fasst Einzelflächen einer Seite zu einer Fläche zusammen – im Koordinatensystem des Teils dieser Seite,
 * Teilflächen auf Nachbarteilen über deren Lage `worldOf`.
 */
function mergeOf(j: Joint, side: Side, pieces: PickedFeature[], worldOf: (id: number) => THREE.Matrix4): PickedFeature {
  const own = getPart(side === 'a' ? j.a : j.b)!;
  const inv = worldOf(own.id).clone().invert();
  const list = pieces.map((f) => {
    const part = featurePart(j, side, f)!;
    return { positions: positionsOf(part), tris: f.tris, T: matrixToRigid(inv.clone().multiply(worldOf(part.id))) };
  });
  // Runde Fläche dabei: Achse als Startwert, damit z. B. zwei Stücke einer Bohrung rund bleiben
  const k = pieces.findIndex((f) => f.kind === 'cylinder');
  const round = k >= 0 ? rotate(list[k].T.R, (pieces[k] as CylinderFeature).axis) : undefined;
  const merged = mergeFeatures(list, round);
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

  if (j.cache?.key !== key && j.cache?.rekey) j.cache = { ...j.cache, key, rekey: false };
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

/** Wer laut Datei zusammengehört (siehe Part.together). */
const togetherOf = (p: Part) => (p.fromFile ? 'file' : p.together);

/** Laufende Flächensuche je Teil (im Hintergrund-Thread). */
const candidateJobs = new WeakMap<Part, Promise<Feature[]>>();
function candidatesAsync(p: Part): Promise<Feature[]> {
  if (p.candidates) return Promise.resolve(p.candidates);
  let job = candidateJobs.get(p);
  if (!job) {
    job = compute('candidates', { positions: positionsOf(p) }).then((c) => (p.candidates ??= c));
    candidateJobs.set(p, job);
  }
  return job;
}

/** Markante Flächen der Teile im Hintergrund suchen (je Teil nur einmal); progress meldet den Fortschritt. */
async function ensureCandidates(list: Part[], progress?: (done: number, total: number) => void) {
  const todo = list.filter((p) => !p.candidates);
  let n = 0;
  await Promise.all(todo.map((p) => candidatesAsync(p).then(() => progress?.(++n, todo.length))));
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
  void ensureCandidates([a, b]).then(() => {
    // Inzwischen gelöscht oder umgestellt: nichts mehr tun
    if (!joints.includes(j) || getPart(j.a) !== a || getPart(j.b) !== b) return;
    beginAction();
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
  }, (e) => toast(`Suche nicht möglich: ${(e as Error).message ?? e}`));
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
  try {
    await ensureCandidates(units.flatMap(members), (d, n) => toast(`Untersuche die Teile … ${d} von ${n}`, 20000));
  } catch (e) {
    btn.disabled = false;
    toast(`Suche nicht möglich: ${(e as Error).message ?? e}`);
    return;
  }
  const edges: { u: number; v: number; sg: Proposal }[] = [];
  let lastYield = performance.now();
  // Je Paar von Einheiten mehrere Kandidaten, falls die beste Fläche schon anderweitig vergeben wird
  const pairCount = (units.length * (units.length - 1)) / 2;
  let done = 0;
  try {
    for (let i = 0; i < units.length; i++) {
      for (let k = i + 1; k < units.length; k++) {
        for (const pa of members(units[i])) for (const pb of members(units[k]))
          for (const sg of proposePair(pa, pb, 4).slice(0, 3)) if (sg.score >= 0.6) edges.push({ u: units[i], v: units[k], sg });
        done++;
        // Nur ab und zu kurz abgeben: jede Pause kostet ein Bild
        if (performance.now() - lastYield > 80) {
          toast(`Suche passende Flächen … ${Math.round((100 * done) / pairCount)} %`, 20000);
          await new Promise((r) => setTimeout(r, 0));
          lastYield = performance.now();
        }
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
      // Anhebung durch den Boden übernehmen, sonst verschöbe ein Drehen das Teil seitlich
      layouts: new Map((p ? groupOf(p) : []).map((q) => [q.id, new THREE.Matrix4().makeTranslation(0, 0, floorShift.get(q.id) ?? 0).multiply(q.layout)])),
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
  updateGizmoNumbers();
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
    for (const p of assemblyOf(moving)) box.union(partBox(p));
    pivot = box.getCenter(new THREE.Vector3());
  }
  if (!pivot) pivot = partBox(moving).getCenter(new THREE.Vector3());
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
    // Fester Boden: die Höhe eines Teils (einer Baugruppe) gibt die Druckplatte vor
    const floorLock = onFloor.checked && gizmoTarget?.kind === 'part' && gizmoMode === 'translate';
    gizmoLocked = [false, false, floorLock];
    hint.textContent = floorLock ? 'Die Höhe gibt die Druckplatte vor (grau gestrichelt). Zum freien Anheben „Auf die Druckplatte stellen" ausschalten.' : '';
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
      ? assemblyOf(getPart(gizmoTarget.id)!).some((q) => q.id === baseId) ? 'Gesamtes Modell bewegen' : `Baugruppe „${nameOf(gizmoTarget.id)}" bewegen`
      : gizmoTarget.kind === 'part'
      ? `„${nameOf(gizmoTarget.id)}" bewegen`
      : `Verbindung ${joints.findIndex((x) => x.id === gizmoTarget!.id) + 1} anpassen`;
  bar.querySelector('[data-g="title"]')!.textContent = label;
  bar.querySelectorAll<HTMLButtonElement>('[data-g-mode]').forEach((b) => b.classList.toggle('on', b.dataset.gMode === gizmoMode));
  (bar.querySelector('#gizmoFree') as HTMLInputElement).checked = gizmoFreeOnly;
  (bar.querySelector('[data-g="free"]') as HTMLElement).hidden = gizmoTarget.kind === 'part';
  (bar.querySelector('#gizmoSnap') as HTMLSelectElement).value = String(gizmoSnapDeg);
  (bar.querySelector('[data-g="reset"]') as HTMLElement).hidden = gizmoTarget.kind === 'part';
  updateGizmoNumbers();
}

// ---------------------------------------------------------------- Genau platzieren

/** Frei liegendes Teil des Gizmos und alles, was mit ihm platziert wird. */
function gizmoUnit(): { seed: Part; members: Part[] } | null {
  if (!gizmoTarget || gizmoTarget.kind !== 'part') return null;
  const seed = getPart(gizmoTarget.id);
  return seed ? { seed, members: assemblyOf(seed) } : null;
}

/** Ausgangslage einer frei liegenden Gruppe ändern – alles, was daran hängt, folgt. */
function moveSeedBy(seed: Part, T: THREE.Matrix4) {
  for (const q of groupOf(seed)) q.layout.premultiply(T);
}

const gposInputs = [...document.querySelectorAll<HTMLInputElement>('[data-gpos]')];
/** Lage im Gizmo: Mitte X/Y und Unterkante Z der Baugruppe. */
function updateGizmoNumbers() {
  const row = document.querySelector<HTMLElement>('#gizmoBar [data-g="num"]')!;
  const u = gizmoUnit();
  row.hidden = !u;
  if (!u) return;
  const b = unitBox(u.members);
  const vals = [(b.min.x + b.max.x) / 2, (b.min.y + b.max.y) / 2, b.min.z];
  gposInputs.forEach((el, i) => { if (document.activeElement !== el) el.value = String(+vals[i].toFixed(2)); });
  gposInputs[2].disabled = onFloor.checked;
  gposInputs[2].title = onFloor.checked ? 'Die Druckplatte gibt die Höhe vor („Auf die Druckplatte stellen“ ausschalten zum Anheben)' : 'Unterkante (mm)';
}

for (const [i, el] of gposInputs.entries()) {
  el.addEventListener('change', () => {
    const u = gizmoUnit();
    const want = Number(el.value);
    if (!u || !Number.isFinite(want)) return;
    const b = unitBox(u.members);
    const cur = [(b.min.x + b.max.x) / 2, (b.min.y + b.max.y) / 2, b.min.z][i];
    const d = new THREE.Vector3().setComponent(i, want - cur);
    moveSeedBy(u.seed, new THREE.Matrix4().makeTranslation(d.x, d.y, d.z));
    solveAll();
    renderAll();
  });
  el.addEventListener('keydown', (e) => { if (e.key === 'Enter') el.dispatchEvent(new Event('change')); });
}
document.querySelectorAll<HTMLButtonElement>('[data-grot]').forEach((b) => b.addEventListener('click', () => {
  const u = gizmoUnit();
  const deg = Number((document.getElementById('gRotDeg') as HTMLInputElement).value);
  if (!u || !Number.isFinite(deg) || !deg) return;
  const c = unitBox(u.members).getCenter(new THREE.Vector3());
  const axis = new THREE.Vector3().setComponent(Number(b.dataset.grot), 1);
  turnUnit(groupOf(u.seed), rotationAbout([c.x, c.y, c.z], [axis.x, axis.y, axis.z], THREE.MathUtils.degToRad(deg)));
  renderAll();
}));

/** Pfeiltasten: was bewegt wird – die Auswahl (je Baugruppe einmal) oder das Teil im Gizmo. */
function nudgeTargets(): Part[] {
  const list = selected.size ? selectedParts() : gizmoUnit() ? [gizmoUnit()!.seed] : [];
  const seeds = new Map<number, Part>();
  for (const q of list) {
    const s = getPart(seedOf.get(q.id) ?? q.id) ?? q;
    seeds.set(s.id, s);
  }
  return [...seeds.values()];
}

/** Richtung auf der Platte zur Bildschirmrichtung (an die nächste Achse gerastet). */
function screenAxis(dx: number, dy: number): THREE.Vector3 {
  const right = new THREE.Vector3(1, 0, 0).applyQuaternion(camera.quaternion).setZ(0);
  const fwd = controls.target.clone().sub(camera.position).setZ(0);
  const v = right.normalize().multiplyScalar(dx).add(fwd.normalize().multiplyScalar(dy));
  return Math.abs(v.x) >= Math.abs(v.y) ? new THREE.Vector3(Math.sign(v.x), 0, 0) : new THREE.Vector3(0, Math.sign(v.y), 0);
}

let nudging = 0;
function nudge(e: KeyboardEvent): boolean {
  const dirs: Record<string, [number, number] | 'up' | 'down'> = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, 1], ArrowDown: [0, -1], PageUp: 'up', PageDown: 'down' };
  const d = dirs[e.key];
  if (!d || e.ctrlKey || e.metaKey) return false;
  const seeds = nudgeTargets();
  if (!seeds.length) return false;
  e.preventDefault();
  const step = e.shiftKey ? 10 : e.altKey ? 0.1 : 1;
  const v = d === 'up' ? new THREE.Vector3(0, 0, 1) : d === 'down' ? new THREE.Vector3(0, 0, -1) : screenAxis(d[0], d[1]);
  v.multiplyScalar(step);
  // Schnelle Folge von Tastendrücken = ein Schritt zum Rückgängigmachen
  if (!nudging) beginAction();
  clearTimeout(nudging);
  nudging = window.setTimeout(() => { nudging = 0; }, 900);
  for (const sd of seeds) moveSeedBy(sd, new THREE.Matrix4().makeTranslation(v.x, v.y, v.z));
  solveAll();
  renderAll();
  return true;
}

/** Baugruppen der Auswahl ausrichten (min/mid/max) oder gleichmäßig verteilen. */
function alignSelection(axis: number, how: 'min' | 'mid' | 'max' | 'dist') {
  const seeds = nudgeTargets();
  if (seeds.length < 2) { toast('Zum Ausrichten mindestens zwei Baugruppen auswählen.'); return; }
  const items = seeds.map((sd) => ({ seed: sd, box: unitBox(assemblyOf(sd)) }));
  const lo = (b: THREE.Box3) => b.min.getComponent(axis);
  const hi = (b: THREE.Box3) => b.max.getComponent(axis);
  const shift = (sd: Part, by: number) => moveSeedBy(sd, new THREE.Matrix4().makeTranslation(...(new THREE.Vector3().setComponent(axis, by).toArray() as [number, number, number])));
  if (how === 'dist') {
    items.sort((a, b) => lo(a.box) + hi(a.box) - lo(b.box) - hi(b.box));
    const span = hi(items[items.length - 1].box) - lo(items[0].box);
    const sum = items.reduce((n, it) => n + hi(it.box) - lo(it.box), 0);
    let gap = (span - sum) / (items.length - 1);
    if (gap < 2) gap = 5;
    let at = lo(items[0].box);
    for (const it of items) {
      shift(it.seed, at - lo(it.box));
      at += hi(it.box) - lo(it.box) + gap;
    }
  } else {
    const target = how === 'min' ? Math.min(...items.map((it) => lo(it.box)))
      : how === 'max' ? Math.max(...items.map((it) => hi(it.box)))
      : items.reduce((n, it) => n + (lo(it.box) + hi(it.box)) / 2, 0) / items.length;
    for (const it of items) {
      const cur = how === 'min' ? lo(it.box) : how === 'max' ? hi(it.box) : (lo(it.box) + hi(it.box)) / 2;
      shift(it.seed, target - cur);
    }
  }
  solveAll();
  renderAll();
}
const selAlign = document.getElementById('selAlign') as HTMLSelectElement;
selAlign.addEventListener('change', () => {
  const [axis, how] = selAlign.value.split('-');
  selAlign.value = '';
  if (how) alignSelection(Number(axis), how as 'min' | 'mid' | 'max' | 'dist');
});

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
  const boxes = new Map(visible.map((p) => [p.id, partBox(p)]));
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
/**
 * Finger auf dem Bild (Touch, Stift): mit zwei Fingern wird gezoomt oder geschoben – das ist kein Antippen.
 * Gezählt wird vor allen anderen Handlern.
 */
const touches = new Set<number>();
let multiTouch = false;
renderer.domElement.addEventListener('pointerdown', (e) => {
  if (e.pointerType === 'mouse') return;
  if (!touches.size) multiTouch = false;
  touches.add(e.pointerId);
  if (touches.size > 1) multiTouch = true;
}, { capture: true });
for (const type of ['pointerup', 'pointercancel'] as const) window.addEventListener(type, (e) => { touches.delete(e.pointerId); }, { capture: true });

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
  // In der Explosionsansicht liegen die Teile nur scheinbar woanders – nichts auswählen
  if (explode > 0) return null;
  setNdc(e);
  const meshes = parts.filter((p) => p.visible).map((p) => p.mesh);
  // Im Schnitt nur, was zu sehen ist
  const hit = raycaster.intersectObjects(meshes, false).find((h) => sectionKeeps(h.point, getPart(h.object.userData.partId)));
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
  if (snap && (pickMode === 'point' || tool === 'measure')) {
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

function updateHover(h: Hit | null, anyFace = false) {
  hoverLine.visible = false;
  // Während das Passungs-Feld offen ist, bleibt die gewählte Fläche markiert
  if (fitTarget) return;
  if (pickMode === 'brush' && !anyFace) return;
  if (!h || (!anyFace && (pickMode === 'point' || !(usableFor(activeJoint(), h.part) || helperSide(activeJoint(), h.part))))) {
    hoverMesh.visible = false;
    hoverKey = '';
    hoverTris = null;
    return;
  }
  if (pickMode === 'edge' && !anyFace) {
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
  const j = activeJoint();
  (hoverMesh.material as THREE.MeshBasicMaterial).color.set(anyFace || !j ? 0x2f6fde : j.color).lerp(new THREE.Color(1, 1, 1), 0.4);
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

// Polygone: mit gedrückter linker Maustaste Dreiecke bemalen, beim Loslassen wird daraus eine Fläche
/** Laufender Strich im Modus „Polygone“; first: Dreieck, auf dem er begann. */
let painting: { id: number; part: Part; side: Side; on?: number; tris: Set<number>; first: number } | null = null;
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
  if (pickMode !== 'brush' || tool || e.button !== 0 || e.ctrlKey || e.metaKey || e.shiftKey || e.altKey) return;
  const h = pick(e);
  const j = activeJoint();
  if (!h || !j || (tc.object && tc.axis)) return;
  let side: Side | null = j.a === h.part.id ? 'a' : j.b === h.part.id ? 'b' : null;
  const helper = side ? null : helperSide(j, h.part);
  side ??= helper;
  if (!side) return;
  // Beginnt der Strich auf einem Teil der Verbindung, wird gemalt statt die Ansicht gedreht
  e.stopImmediatePropagation();
  painting = { id: e.pointerId, part: h.part, side, on: helper ? h.part.id : undefined, tris: new Set(), first: h.tri };
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
  const { part, side, on, tris, first } = painting;
  painting = null;
  hoverMesh.visible = false;
  const j = activeJoint();
  if (!j || !tris.size) return;
  const list = side === 'a' ? j.fa : j.fb;
  // Beginnt der Strich auf einer schon gewählten Fläche dieses Teils (oder liegt er überwiegend darauf):
  // diese abwählen – wie ein zweiter Klick im Modus „Flächen“
  const hit = list.findIndex((g) => g.kind !== 'edge' && (() => {
    const own = new Set((g.pieces ?? [g]).filter((q) => featurePart(j, side, q) === part).flatMap((q) => q.tris));
    if (own.has(first)) return true;
    let n = 0;
    for (const t of tris) if (own.has(t)) n++;
    return n >= 0.6 * tris.size;
  })());
  if (hit >= 0) list.splice(hit, 1);
  else {
    const f: PickedFeature = featureFromTris(topologyOf(part), [...tris]);
    if (on !== undefined) f.on = on;
    list.push(f);
  }
  if (j.ma.length + j.mb.length + j.fa.length + j.fb.length === 0) j.applied = false;
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
  if ((part?.id ?? null) !== statusHover) {
    statusHover = part?.id ?? null;
    renderStatus();
  }
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
  if (span && p) renameInList(p);
});

function renameInList(p: Part) {
  // Zeile sichtbar machen (eingeklappte Liste, eingeklappte ausgeblendete Teile)
  if (partsFolded) setPartsFolded(false);
  if (!p.visible && hiddenFolded) {
    hiddenFolded = false;
    renderParts();
  }
  const span = partsEl.querySelector<HTMLElement>(`[data-act="name"][data-id="${p.id}"]`);
  if (!span) return;
  span.scrollIntoView({ block: 'nearest' });
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
}

renderer.domElement.addEventListener('pointermove', (e) => {
  if (e.buttons || tc.dragging) {
    cursor.visible = false;
    hideTip();
    return;
  }
  const h = pick(e);
  // Beim Malen stört der Name am Zeiger
  hoverName(e, tool === 'paint' ? null : h?.part ?? null);
  if (tool === 'paint') {
    cursor.visible = false;
    updateHover(null);
    updatePaintCursor(h, e.shiftKey || paintErase);
    renderer.domElement.style.cursor = h ? (paintMode === 'fill' ? 'cell' : 'none') : 'default';
    return;
  }
  if (tool === 'measure') {
    cursor.visible = false;
    updateHover(null);
    updateMeasureHover(h, e.shiftKey);
    renderer.domElement.style.cursor = h || measureCenterAt(e) ? 'pointer' : 'default';
    return;
  }
  if (tool) {
    // Werkzeug (Auf Fläche legen, Passung): jede Fläche jedes Teils
    cursor.visible = false;
    updateHover(h, true);
    renderer.domElement.style.cursor = h ? 'pointer' : 'default';
    return;
  }
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
  if (tool === 'measure') updateMeasureHover(null, false);
  if (!paintStroke) paintCursor.visible = false;
  hideTip();
});

renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downPos || e.button !== 0) return;
  const moved = Math.hypot(e.clientX - downPos.x, e.clientY - downPos.y);
  downPos = null;
  if (suppressClick) { suppressClick = false; return; }
  if (moved > (e.pointerType === 'mouse' ? 5 : 12)) return; // war eine Kamerabewegung
  if (e.pointerType !== 'mouse' && multiTouch) return; // zwei Finger: Zoomen/Schieben

  const h = pick(e);
  if (tool === 'measure') {
    clickMeasure(e, h);
    return;
  }
  // Malen arbeitet beim Drücken und Ziehen
  if (tool === 'paint') return;
  if (!h) return;
  if (tool === 'lay') {
    layOnFace(h);
    return;
  }
  if (tool === 'fit') {
    pickFit(h);
    return;
  }
  if (tool === 'pocket') {
    pickPocket(h);
    return;
  }
  if (tool === 'text') {
    pickText(h);
    return;
  }
  // Umschalt+Klick: Teil zur Auswahl hinzufügen (oder herausnehmen) und in der Liste zeigen
  if (e.shiftKey) {
    toggleSelected(h.part.id);
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
  // Beim Verbinden die Karte der Verbindung zeigen
  if (tabNow === 'parts') showTab('connect');
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
    // Alt/⌥ + Klick: zur zuletzt gewählten Fläche dieser Seite hinzufügen (Kanten zählen nicht)
    let lastIdx = add ? list.length - 1 : -1;
    while (lastIdx >= 0 && !mergeable(list[lastIdx])) lastIdx--;
    const last = list[lastIdx];
    if (last) {
      list[lastIdx] = mergeOf(j, side, [...(last.pieces ?? [last]), f], (id) => getPart(id)!.mesh.matrixWorld);
    } else {
      if (add) toast('Auf dieser Seite ist noch keine Fläche gewählt – die Fläche wurde einzeln gewählt. Alt/⌥ + Klick hängt die nächste daran.');
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

// Kein Browsermenü im 3D-Fenster – ein Rechtsklick ohne Ziehen öffnet das eigene (siehe Kontextmenü)
renderer.domElement.addEventListener('contextmenu', (e) => e.preventDefault());

/** Markierung (Punkt oder gewählte Fläche) unter dem Mauszeiger. */
function markerAt(e: PointerEvent | MouseEvent): { joint: Joint; side: Side; index: number; kind: 'point' | 'face' } | null {
  setNdc(e);
  const targets = [...markerGroup.children.filter((o) => (o as THREE.Mesh).isMesh), ...faceGroup.children];
  const hit = raycaster.intersectObjects(targets, false)[0];
  if (!hit) return null;
  const { joint, side, index, kind } = hit.object.userData as { joint: number; side: Side; index: number; kind: 'point' | 'face' };
  const j = joints.find((x) => x.id === joint);
  return j ? { joint: j, side, index, kind } : null;
}

function removeMarker(m: { joint: Joint; side: Side; index: number; kind: 'point' | 'face' }) {
  const j = m.joint;
  if (m.kind === 'face') (m.side === 'a' ? j.fa : j.fb).splice(m.index, 1);
  else (m.side === 'a' ? j.ma : j.mb).splice(m.index, 1);
  markersChanged(j);
  if (j.ma.length + j.mb.length + j.fa.length + j.fb.length === 0) j.applied = false;
  solveAll();
  renderAll();
}

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
  // Hilfe offen: nur sie bedienen (Esc schließt sie selbst)
  if (helpDialog.open) return;
  if (e.key === '?' || e.key === 'F1') {
    e.preventDefault();
    openHelp();
    return;
  }
  // Kästchen, Listen usw.: nur Strg+Z/Y übernehmen, Buchstaben-Kürzel nicht
  if ((e.target as HTMLElement).closest('input, select, textarea') && !isUndoKey(e)) return;
  // Strg/⌘+Z: letzte Aktion zurücknehmen, Strg/⌘+Umschalt+Z oder Strg+Y: wiederholen
  if ((e.ctrlKey || e.metaKey) && (e.key.toLowerCase() === 'z' || e.key.toLowerCase() === 'y')) {
    e.preventDefault();
    if (e.key.toLowerCase() === 'y' || e.shiftKey) redo();
    else undo();
    return;
  }
  // Pfeiltasten: Auswahl verschieben
  if (nudge(e)) return;
  // Strg/⌘+A: alle sichtbaren Teile auswählen
  if ((e.ctrlKey || e.metaKey) && !e.altKey && e.key.toLowerCase() === 'a') {
    if (!parts.length) return;
    e.preventDefault();
    for (const p of parts) if (p.visible) selected.add(p.id);
    renderParts();
    return;
  }
  if (e.ctrlKey || e.metaKey || e.altKey) return;
  const k = e.key.toLowerCase();
  if (k === 'p') setPickMode('point');
  else if (k === 'f') setPickMode('face');
  else if (k === 'k') setPickMode('edge');
  else if (k === 'b') setPickMode('brush');
  else if (k === 'l') setTool(tool === 'lay' ? null : 'lay');
  else if (k === 't') setTool(tool === 'fit' ? null : 'fit');
  else if (k === 'm') setTool(tool === 'measure' ? null : 'measure');
  else if (k === 'e') setTool(tool === 'pocket' ? null : 'pocket');
  else if (k === 'n') setTool(tool === 'paint' ? null : 'paint');
  else if (k === 'w') setTool(tool === 'text' ? null : 'text');
  else if (k === 'a') void arrangeOnBed();
  else if (/^[0-6]$/.test(k)) viewFrom(VIEW_KEYS[k]);
  else if (k === 'c') setSection(!section);
  else if (k === 'd') setDims(!dimsOn);
  else if (k === 'u') setAnalysis(!anaOn);
  else if (k === 'x') setExplode(explode > 0 || explodeAnim?.to ? 0 : 0.8);
  else if (k === 'escape' && (explode > 0 || explodeAnim?.to)) setExplode(0);
  else if (k === 's') {
    // Auswahl oder das Teil unter dem Mauszeiger (das „s“ soll nicht im Eingabefeld landen)
    e.preventDefault();
    const hovered = getPart(statusHover);
    if (selected.size > 1) openScale({ sel: [...selected] });
    else if (hovered || selected.size === 1) {
      const p = hovered ?? getPart([...selected][0])!;
      openScale({ partId: p.id, which: unitMembers(p).length > 1 ? 'unit' : 'part' });
    } else toast('Zum Skalieren mit der Maus auf ein Teil zeigen (oder Teile auswählen) und S drücken.');
  } else if (scaleTarget && k === 'escape') closeScale();
  else if (cutTarget && k === 'escape') setSection(false);
  else if (splitTarget && k === 'escape') closeSplit();
  else if (simpTarget && k === 'escape') closeSimplify();
  else if (!tolBar.hidden && k === 'escape') tolBar.hidden = true;
  else if (tool === 'measure' && k === 'escape' && measureSel.length) { measureSel.pop(); renderMeasure(); }
  else if (tool === 'measure' && (k === 'delete' || k === 'backspace')) { measureSel = []; renderMeasure(); }
  else if (tool && k === 'escape') setTool(null);
  else if (gizmoTarget && (k === 'g' || k === 'r')) {
    gizmoMode = k === 'g' ? 'translate' : 'rotate';
    updateGizmoAxes();
    renderGizmoBar();
  } else if (gizmoTarget && k === 'escape') closeGizmo();
  else if (k === 'escape') clearSelection();
});

// ---------------------------------------------------------------- Seitenleiste


function renderParts() {
  // Nummerieren: die Auswahl bestimmt, welche Teile eine Nummer bekommen
  if (tool === 'text' && textSrc === 'number') scheduleTextPreview();
  const row = (p: Part) => `<li data-part="${p.id}"${p.visible && !selected.has(p.id) ? '' : ` class="${[p.visible ? '' : 'off', selected.has(p.id) ? 'sel' : ''].join(' ').trim()}"`}>
        <input type="color" class="swatch" data-act="color" data-id="${p.id}" value="#${p.color.getHexString()}" title="Farbe (wird beim mehrfarbigen 3MF-Export übernommen)" />
        <input type="checkbox" data-act="vis" data-id="${p.id}" ${p.visible ? 'checked' : ''} title="Sichtbar" />
        <span class="name" data-act="name" data-id="${p.id}" title="${esc(p.name)} – Klick: im Modell zeigen, Doppelklick: umbenennen">${esc(p.name)}</span>
        <label class="base" title="Das Basisteil bleibt liegen, alle anderen werden daran angefügt.">
          <input type="radio" name="base" data-act="base" data-id="${p.id}" ${p.id === baseId ? 'checked' : ''}/> Basis
        </label>
        ${p.health && !p.health.closed ? `<button class="icon warn" data-act="repair" data-id="${p.id}" title="Netz nicht geschlossen: ${esc(healthText(p.health))}. Zerschneiden und Vereinigen brauchen ein geschlossenes Netz – klicken zum Reparieren." aria-label="Netz reparieren">⚠</button>` : ''}
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
  renderSelection();
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
  if (explodeAnim?.to || explode > 0) el.textContent = 'Explosionsansicht – nur zum Ansehen. Regler unten rechts, X oder ✕ schließt sie.';
  else if (tool === 'lay') el.textContent = 'Klicke die Fläche, die unten auf der Druckplatte liegen soll – die ganze Baugruppe dreht sich mit. Esc bricht ab.';
  else if (tool === 'pocket') el.textContent = pocketPlace
    ? 'Art und Größe oben wählen, dann „Einsetzen“ – oder eine andere Stelle anklicken. Esc beendet.'
    : 'Klicke die Stelle an, an der die Aussparung sitzen soll (auf runden Flächen rastet sie in der Mitte ein). Esc beendet.';
  else if (tool === 'text') el.textContent = textSrc === 'number'
    ? `Nummern für ${textTargets().length} Teil${textTargets().length === 1 ? '' : 'e'} – Teile anklicken nimmt sie dazu oder heraus. „Einprägen“ schreibt sie, Esc beendet.`
    : textPlace
      ? 'Text, Größe und Relief oben einstellen, dann „Einprägen“ – oder eine andere Stelle anklicken. Esc beendet.'
      : 'Klicke die Stelle an, an der die Beschriftung sitzen soll – sie folgt der Oberfläche. Esc beendet.';
  else if (tool === 'measure') el.textContent = measureSel.length === 1
    ? 'Zweites Merkmal anklicken für Abstand und Winkel – erneut anklicken wählt ab. Kreise: die Mitte ist anklickbar. Entf: neu, Esc: zurück.'
    : measureSel.length === 2
      ? 'Ein weiterer Klick ersetzt das zweite Merkmal, ein Klick auf ein gewähltes wählt es ab. Entf: neu, Esc: zurück.'
      : 'Zeige auf eine Ecke, Kante, einen Kreis oder eine Fläche und klicke (Umschalt: beliebiger Punkt). Esc beendet.';
  else if (tool === 'paint') el.textContent = paintMode === 'fill'
    ? 'Klicke eine Fläche an – sie wird bis zu Knicken über dem Winkel gefüllt. Umschalt + Klick: Filament des Teils. Esc beendet.'
    : 'Über das Teil streichen malt, Umschalt radiert. Alt + Mausrad: Pinselgröße. Ziehen neben dem Teil dreht die Ansicht. Esc beendet.';
  else if (tool === 'fit') el.textContent = fitTarget ? 'Wert oben eingeben und „Übernehmen" – oder eine andere Fläche anklicken. Esc bricht ab.' : 'Klicke einen Zapfen, eine Bohrung oder eine ebene Fläche an, deren Passung du ändern willst. Esc bricht ab.';
  else if (parts.length === 0) el.textContent = 'Lade zwei oder mehr Modelle – oder probiere ein Beispiel. Taste ? zeigt Anleitung und Tastenkürzel.';
  else if (parts.length === 1) el.textContent = 'Lade noch ein weiteres Teil, das angefügt werden soll.';
  else if (!j) el.textContent = 'Lege im Reiter „Verbinden“ eine Verbindung an.';
  else if (j.auto && !j.ma.length && !j.mb.length && !j.fa.length && !j.fb.length) {
    el.textContent = {
      face: 'Klicke die Kontaktfläche auf einem Teil – danach die passende Fläche auf dem Teil, das daran soll. Oder im Reiter „Verbinden" „Alles automatisch verbinden".',
      edge: 'Klicke eine gerade Kante auf einem Teil – danach die passende Kante auf dem Teil, das daran soll.',
      point: 'Klicke markante Punkte auf einem Teil – danach dieselben Stellen auf dem Teil, das daran soll.',
      brush: 'Streiche mit gedrückter Maustaste über die Polygone der Kontaktstelle auf einem Teil – danach über die passende Stelle auf dem anderen.',
    }[pickMode];
  } else if (j.a !== null && j.b === null) {
    const what = { face: 'Fläche', edge: 'Kante', point: 'Stelle', brush: 'Stelle' }[pickMode];
    el.textContent = `Jetzt die passende ${what} auf dem Teil anklicken, das an „${nameOf(j.a)}" soll.`;
  } else if (j.a === null || j.b === null) el.textContent = 'Klicke die beiden Teile an, die verbunden werden sollen.';
  else if (j.applied) el.textContent = `Verbunden. Weitere ${what} verfeinern das Ergebnis live · Rechtsklick entfernt eine Auswahl.`;
  else if (pickMode === 'brush') {
    el.textContent = `Polygone: mit gedrückter Maustaste über die Kontaktstelle auf „${nameOf(j.fa.length <= j.fb.length ? j.a : j.b)}" streichen – beim Loslassen wird daraus eine Fläche; über eine Auswahl streichen hebt sie auf. Daneben ziehen dreht die Ansicht.`;
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
  // Ausgewählte Teile, die es nicht mehr gibt, vergessen
  for (const id of selected) if (!getPart(id)) selected.delete(id);
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
  drawBed();
  renderStatus();
  renderMaterial();
  if (standOn) renderStand();
  if (dimsOn) renderDims();
  // Maße folgen bewegten oder geänderten Teilen
  if (tool === 'measure') renderMeasure();
  if (scaleTarget) renderScaleBar(true);
  if (section) { sectionRange(); updateSection(); }
  scheduleAnalysis();
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
  drawSelection();
}

function setBase(id: number) {
  baseId = id;
  solveAll();
  renderAll();
  fitView();
}

/** Teil aus seiner Gruppe lösen; es bleibt, wo es ist. */
function ungroupPart(p: Part) {
  // Lage beibehalten: aktuelle Position wird die neue Ausgangslage
  p.layout.copy(p.mesh.matrixWorld);
  p.group = undefined;
  solveAll();
  renderAll();
  toast(`„${p.name}" bewegt sich jetzt unabhängig von seiner Gruppe.`);
}

/** Gizmo für ein Teil öffnen – oder schließen, wenn es schon offen ist. */
function movePartByHand(id: number) {
  // Nochmal klicken schließt das Gizmo wieder
  if (gizmoPart()?.id === id && !gizmoTarget?.assembly) {
    closeGizmo();
    renderAll();
    return;
  }
  // Wird das Teil von einer Verbindung bewegt, wird diese angepasst
  const jid = movedBy.get(id);
  if (jid !== undefined) {
    activeJointId = jid;
    openGizmo({ kind: 'joint', id: jid });
  } else openGizmo({ kind: 'part', id });
  renderAll();
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
      if (p) ungroupPart(p);
      break;
    }
    case 'fold-hidden':
      hiddenFolded = !hiddenFolded;
      renderParts();
      break;
    case 'move-part':
      movePartByHand(id);
      break;
    case 'repair': {
      const p = getPart(id);
      if (p) void repairParts([p]);
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
      setBase(id);
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

// ---------------------------------------------------------------- Kopfleiste

/**
 * Passt die Werkzeugleiste nicht in die Breite, in Stufen enger werden: erst nur Symbole (die Bezeichnung
 * steht im Tooltip), dann auch ohne Schriftzug und Export-Text, zuletzt schmalere Knöpfe ohne Hilfe-Knopf
 * (Hilfe bleibt im Menü und auf „?“). Reicht auch das nicht, lässt sich die Leiste wischen – ein
 * ausgeblendeter Rand zeigt, dass dort noch Werkzeuge liegen.
 */
{
  const topbar = document.getElementById('topbar')!;
  const bar = document.getElementById('toolbar')!;
  const LEVELS = ['compact', 'tight', 'narrow'];
  const edges = () => {
    const more = bar.scrollWidth - bar.clientWidth > 1;
    bar.classList.toggle('more-l', more && bar.scrollLeft > 1);
    bar.classList.toggle('more-r', more && bar.scrollLeft < bar.scrollWidth - bar.clientWidth - 1);
  };
  bar.addEventListener('scroll', edges, { passive: true });
  /** Breite der Kopfleiste, ab der die jeweilige Stufe wieder entfallen kann. */
  const needs: number[] = [];
  const fit = () => {
    for (let i = LEVELS.length - 1; i >= 0; i--) {
      if (topbar.classList.contains(LEVELS[i]) && topbar.clientWidth >= needs[i]) topbar.classList.remove(LEVELS[i]);
    }
    for (let i = 0; i < LEVELS.length; i++) {
      if (topbar.classList.contains(LEVELS[i])) continue;
      if (bar.scrollWidth <= bar.clientWidth + 1) break;
      needs[i] = topbar.clientWidth + (bar.scrollWidth - bar.clientWidth) + 8;
      topbar.classList.add(LEVELS[i]);
    }
    edges();
  };
  const ro = new ResizeObserver(fit);
  ro.observe(topbar);
  // Auch wenn rechts etwas breiter oder schmaler wird (anderer Drucker gewählt)
  ro.observe(topbar.querySelector('.tright')!);
}

// ---------------------------------------------------------------- Seitenleiste: Reiter

/**
 * Reiter „Teile“, „Verbinden“, „Drucken“: Die drei Seiten liegen nebeneinander und gleiten beim Umschalten
 * (am Tablet auch per Wischen). Was auf einer anderen Seite in den Blick gerückt wird (Teil in der Liste
 * zeigen, Tastatur-Fokus), wechselt die Seite mit.
 */
const TABS = ['parts', 'connect', 'print'] as const;
type Tab = (typeof TABS)[number];
const pagesEl = document.getElementById('pages')!;
let tabNow: Tab = 'parts';
const calmMotion = matchMedia('(prefers-reduced-motion: reduce)');

function markTab(t: Tab) {
  tabNow = t;
  for (const b of document.querySelectorAll<HTMLElement>('.ptabs [data-tab]')) {
    const on = b.dataset.tab === t;
    b.classList.toggle('on', on);
    b.setAttribute('aria-selected', String(on));
  }
}

function showTab(t: Tab, smooth = true) {
  markTab(t);
  pagesEl.scrollTo({ left: TABS.indexOf(t) * pagesEl.clientWidth, behavior: smooth && !calmMotion.matches ? 'smooth' : 'auto' });
}

// Erst wenn die Seite eingerastet ist, den Reiter umstellen (nicht beim Durchgleiten)
pagesEl.addEventListener('scroll', () => {
  const w = Math.max(1, pagesEl.clientWidth);
  const i = Math.round(pagesEl.scrollLeft / w);
  if (Math.abs(pagesEl.scrollLeft - i * w) < 2 && TABS[i] && TABS[i] !== tabNow) markTab(TABS[i]);
}, { passive: true });
document.querySelector('.ptabs')!.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('[data-tab]');
  if (b) showTab(b.dataset.tab as Tab);
});
// Breite der Seitenleiste geändert: auf derselben Seite bleiben
new ResizeObserver(() => { pagesEl.scrollLeft = TABS.indexOf(tabNow) * pagesEl.clientWidth; }).observe(pagesEl);
document.getElementById('exportBtn')!.addEventListener('click', () => {
  showTab('print', false);
  document.getElementById('exportHead')!.scrollIntoView({ block: 'start', inline: 'nearest', behavior: calmMotion.matches ? 'auto' : 'smooth' });
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
fileInput.accept = [...SUPPORTED, ...PROJECT_EXTS].map((s) => `.${s}`).join(',');
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
  loadDemo(makeRoundDemo(), 'Beispiel „Zapfen": Klicke den runden Teil des Zapfens und die runde Bohrungswand der Kappe an, dann „Verbinden". Die Drehung findet D3DForge über die Abflachung selbst.', 'face'),
);

// Drucker (nach Hersteller gruppiert): zuletzt gewählter oder der eines geladenen Projekts
const bambuPrinter = document.getElementById('bambuPrinter') as HTMLSelectElement;
bambuPrinter.innerHTML = VENDORS.map((v) => `<optgroup label="${esc(v)}">${PRINTERS.filter((p) => p.vendor === v)
  .map((p) => `<option value="${p.id}" title="${esc(`${p.vendor} ${p.label}: ${p.bed[0]} × ${p.bed[1]}${p.height ? ` × ${p.height}` : ''} mm${p.tools ? ` · ${p.tools} Düsen/Werkzeuge` : ''}`)}">${esc(p.label)}</option>`).join('')}</optgroup>`).join('');
try { bambuPrinter.value = localStorage.getItem('partjoin.bambuPrinter') ?? 'X1C'; } catch { /* ohne Speicher */ }
if (!bambuPrinter.value) bambuPrinter.value = 'X1C';
/** Drucker, für den die Platten gerade liegen. */
let shownPrinter = bambuPrinter.value;
/** Hersteller über dem Modellnamen (die Liste zeigt ihn als Gruppe, die geschlossene Auswahl nicht). */
const printerVendor = document.getElementById('printerVendor')!;
const renderPrinterVendor = () => { printerVendor.textContent = (printerById(bambuPrinter.value) ?? PRINTERS[0]).vendor; };
renderPrinterVendor();
function setBambuPrinter(id: string) {
  bambuPrinter.value = id;
  shownPrinter = id;
  renderPrinterVendor();
  try { localStorage.setItem('partjoin.bambuPrinter', id); } catch { /* ohne Speicher */ }
}
/** Anderer Drucker: Platten bekommen seine Größe, die Teile bleiben auf ihrer Platte an derselben Stelle. */
function changePrinter(id: string) {
  const from = (printerById(shownPrinter) ?? PRINTERS[0]).bed;
  setBambuPrinter(id);
  if (parts.length) relayoutPlates(plateTotal, (i) => i, from);
  renderAll();
  if (splitTarget) renderSplit();
  applyMyClearance();
  renderExportTarget();
}
bambuPrinter.addEventListener('change', () => changePrinter(bambuPrinter.value));

onFloor.addEventListener('change', () => {
  // Ausschalten: alles bleibt, wo es gerade steht (die Anhebung wird Teil der Ausgangslage)
  if (!onFloor.checked) for (const p of parts) p.layout.premultiply(new THREE.Matrix4().makeTranslation(0, 0, floorShift.get(p.id) ?? 0));
  solveAll();
  if (gizmoTarget) placeGizmo();
  renderAll();
  toast(onFloor.checked ? 'Jede Baugruppe steht jetzt auf der Druckplatte.' : 'Teile bleiben frei im Raum, wo sie gerade sind.');
});

/** Speichern und Exportieren nacheinander: ein Doppelklick erzeugt keine zweite Datei. */
let saving = false;
async function exclusive(run: () => Promise<void>) {
  if (saving) { toast('Einen Moment – die vorige Datei wird noch gespeichert.', 2500); return; }
  saving = true;
  try {
    await run();
  } finally {
    saving = false;
  }
}

/**
 * Export: Ziel (Slicer bzw. Format) aus der Liste, dann „Exportieren“. Ohne eigene Wahl richtet sich das Ziel
 * nach dem Slicer des gewählten Druckers.
 */
const exportTarget = document.getElementById('exportTarget') as HTMLSelectElement;
const EXPORT_HELP: Record<string, string> = {
  bambu: 'Bambu-Studio-Projekt: jede Baugruppe ein Objekt aus ihren Teilen, die Teilefarben als Filamente, Bemalung, Aufschriften und Platten wie hier. Bei Bambu-Lab-Druckern samt Drucker-, Prozess- und Filamentprofil.',
  orca: 'Projekt im Format von Bambu Studio, das OrcaSlicer und seine Ableger (Creality Print, Anycubic Slicer Next, ElegooSlicer, QIDI Studio, Snapmaker Orca, Orca-Flashforge) lesen: Objekte, Filamente in den Teilefarben, Bemalung und Platten. Bei Druckern anderer Hersteller ohne Profile – Drucker und Prozess wählst du nach dem Öffnen im Slicer.',
  prusa: 'PrusaSlicer-Format (auch SuperSlicer): ein Objekt aus Teilen mit eigenem Extruder je Farbe und Bemalung. Im Slicer jedem Extruder das passende Filament zuordnen.',
  cura: 'Für UltiMaker Cura: jedes Teil ein eigenes Objekt mit Namen und Extruder je Farbe (1. Farbe = Extruder 1 …), so platziert wie hier. Bemalung kennt Cura nicht.',
  '3mf': 'Alle Teile als ein Körper ohne Farben – für jeden Slicer.',
  stl: 'STL für jeden Slicer – ohne Farben.',
  obj: 'OBJ für jeden Slicer – ohne Farben.',
};
let exportChosen = false;
try { exportChosen = !!localStorage.getItem('partjoin.exportTarget'); if (exportChosen) exportTarget.value = localStorage.getItem('partjoin.exportTarget')!; } catch { /* ohne Speicher */ }
function renderExportTarget() {
  if (!exportChosen) exportTarget.value = currentPrinter().slicer;
  if (!exportTarget.value) exportTarget.value = 'orca';
  document.getElementById('exportHelp')!.textContent = EXPORT_HELP[exportTarget.value] ?? '';
}
exportTarget.addEventListener('change', () => {
  exportChosen = true;
  try { localStorage.setItem('partjoin.exportTarget', exportTarget.value); } catch { /* ohne Speicher */ }
  renderExportTarget();
});
document.getElementById('exportGo')!.addEventListener('click', () => void exclusive(() => exportAs(exportTarget.value)));

/**
 * Vorschaubild (PNG) der sichtbaren Teile – schräg von vorne, durchsichtiger Hintergrund, nur die Teile.
 * within: nur, was in diesem Umriss liegt (eine Platte). Doppelt groß gezeichnet und verkleinert, damit
 * die Kanten weich werden.
 */
async function thumbnail(within?: THREE.Box3, size = 512): Promise<Uint8Array | undefined> {
  const shown = parts.filter((p) => p.visible && (!within || within.containsPoint(partBox(p).getCenter(new THREE.Vector3()))));
  if (!shown.length) return undefined;
  const box = unitBox(shown);
  const c = box.getCenter(new THREE.Vector3());
  const r = Math.max(1, box.getSize(new THREE.Vector3()).length() / 2);
  const cam = new THREE.PerspectiveCamera(30, 1, r / 50, r * 50);
  cam.up.set(0, 0, 1);
  cam.position.copy(c).add(new THREE.Vector3(0.75, -1.15, 0.85).normalize().multiplyScalar(r / Math.sin(THREE.MathUtils.degToRad(15)) * 1.02));
  cam.lookAt(c);
  cam.updateMatrixWorld();
  // Ausschnitt eng um die Punkte der Teile legen, mittig und quadratisch
  const S = size * 2;
  const lo = new THREE.Vector2(Infinity, Infinity), hi = new THREE.Vector2(-Infinity, -Infinity);
  const v = new THREE.Vector3();
  for (const p of shown) {
    p.mesh.updateMatrixWorld();
    const pos = p.mesh.geometry.getAttribute('position');
    const step = Math.max(1, Math.floor(pos.count / 20000));
    for (let i = 0; i < pos.count; i += step) {
      v.fromBufferAttribute(pos, i).applyMatrix4(p.mesh.matrixWorld).project(cam);
      lo.set(Math.min(lo.x, v.x), Math.min(lo.y, -v.y));
      hi.set(Math.max(hi.x, v.x), Math.max(hi.y, -v.y));
    }
  }
  const side = Math.max(hi.x - lo.x, hi.y - lo.y) * 1.08 * S / 2;
  cam.setViewOffset(S, S, ((lo.x + hi.x) / 2 + 1) * S / 2 - side / 2, ((lo.y + hi.y) / 2 + 1) * S / 2 - side / 2, side, side);
  // Licht wie in der Ansicht: an der Kamera befestigt
  cam.add(sun.clone());
  scene.add(cam);
  // Nur die Teile zeigen (samt ihrer Darstellung), alles andere kurz ausblenden
  const hidden: THREE.Object3D[] = [];
  for (const o of scene.children) {
    if (o === cam || (o as THREE.Light).isLight || parts.some((p) => p.mesh === o)) continue;
    if (o.visible) { o.visible = false; hidden.push(o); }
  }
  for (const p of parts) if (p.visible && !shown.includes(p)) { p.mesh.visible = false; hidden.push(p.mesh); }
  const clip = renderer.localClippingEnabled;
  renderer.localClippingEnabled = false;
  const rt = new THREE.WebGLRenderTarget(S, S, { colorSpace: THREE.SRGBColorSpace });
  const clear = renderer.getClearColor(new THREE.Color());
  const alpha = renderer.getClearAlpha();
  const pixels = new Uint8Array(S * S * 4);
  try {
    renderer.setRenderTarget(rt);
    renderer.setClearColor(0x000000, 0);
    renderer.clear();
    renderer.render(scene, cam);
    renderer.readRenderTargetPixels(rt, 0, 0, S, S, pixels);
  } finally {
    renderer.setRenderTarget(null);
    renderer.setClearColor(clear, alpha);
    renderer.localClippingEnabled = clip;
    for (const o of hidden) o.visible = true;
    scene.remove(cam);
    rt.dispose();
  }
  // Zeilen umdrehen (WebGL zählt von unten) und verkleinern
  const big = document.createElement('canvas');
  big.width = big.height = S;
  const img = new ImageData(S, S);
  for (let y = 0; y < S; y++) img.data.set(pixels.subarray((S - 1 - y) * S * 4, (S - y) * S * 4), y * S * 4);
  big.getContext('2d')!.putImageData(img, 0, 0);
  const small = document.createElement('canvas');
  small.width = small.height = size;
  const g = small.getContext('2d')!;
  g.imageSmoothingQuality = 'high';
  g.drawImage(big, 0, 0, size, size);
  const blob = await new Promise<Blob | null>((res) => small.toBlob(res, 'image/png'));
  return blob ? new Uint8Array(await blob.arrayBuffer()) : undefined;
}

/** Vorschaubilder je Platte (Bambu) bzw. eines für alles. */
async function thumbnails(plates: number): Promise<Uint8Array[]> {
  try {
    if (plates <= 1) {
      const t = await thumbnail();
      return t ? [t] : [];
    }
    const [w, d] = currentPrinter().bed;
    const out: Uint8Array[] = [];
    for (let i = 0; i < plates; i++) {
      const c = plateCenter(i);
      const b = new THREE.Box3(new THREE.Vector3(c.x - w / 2, c.y - d / 2, -1e6), new THREE.Vector3(c.x + w / 2, c.y + d / 2, 1e6));
      out.push((await thumbnail(b)) ?? new Uint8Array(0));
    }
    return out;
  } catch {
    // Ohne Vorschaubild geht der Export trotzdem
    return [];
  }
}

function transformed(positions: Float32Array, m: THREE.Matrix4): Float32Array {
  const out = new Float32Array(positions.length);
  const v = new THREE.Vector3();
  for (let i = 0; i < positions.length; i += 3) v.fromArray(positions, i).applyMatrix4(m).toArray(out, i);
  return out;
}

/** Exportiert alle sichtbaren Teile in ihrer zusammengesetzten Lage. */
async function exportAs(kind: string) {
  const visible = parts.filter((p) => p.visible);
  if (!visible.length) { toast('Keine sichtbaren Teile zum Exportieren.'); return; }
  const world: ColoredPart[] = visible.map((p) => ({
    name: p.name,
    color: `#${p.color.getHexString()}`,
    positions: p.geom.clone().applyMatrix4(p.mesh.matrixWorld).getAttribute('position').array as Float32Array,
    // Bemalung je Dreieck bleibt gültig (gleiche Dreiecke, gleiche Reihenfolge), Modifier ziehen mit um
    paint: p.paint,
    palette: p.palette,
    settings: p.settings,
    objectSettings: p.objectSettings,
    // Jede Baugruppe ein eigenes Objekt im Slicer, benannt nach ihrem ersten Teil
    object: String(unitOf.get(p.id) ?? p.id),
    objectName: getPart(unitOf.get(p.id) ?? p.id)?.name,
    modifiers: p.modifiers?.map((m) => ({
      ...m,
      positions: transformed(m.positions, p.mesh.matrixWorld),
      // Text: Lage mitziehen, das eigene Netz bleibt
      transform: m.transform && new THREE.Matrix4().fromArray(m.transform).premultiply(p.mesh.matrixWorld).toArray(),
    })),
  }));
  try {
    const fit = bedFit();
    const onBed = !fit.outside.length && !fit.tooBig.length;
    const plates = onBed ? plateTotal : 1;
    // Platte je Teil: die seiner Baugruppe
    if (onBed && plates > 1) {
      const unitPlate = new Map<number, number>();
      for (const u of visibleUnits()) for (const q of u.members) unitPlate.set(q.id, plateOfBox(u.box).i);
      visible.forEach((p, i) => { world[i].plate = unitPlate.get(p.id) ?? 0; });
    }
    // Frühere Bezeichnungen der Export-Knöpfe
    if (kind === '3mf-bambu') kind = 'bambu';
    if (kind === '3mf-color') kind = 'prusa';
    const printer = currentPrinter();
    if (kind === 'bambu' || kind === 'orca' || kind === 'prusa' || kind === 'cura') {
      toast('Speichere …', 20000);
      const project = kind === 'bambu' || kind === 'orca';
      const data = project
        ? await compute('write3mfBambu', {
          parts: world,
          // Liegt alles auf der Platte, kommt es im Slicer genau so an
          opts: {
            printer: printer.bambu ? printer.id : undefined,
            bed: printer.bambu ? undefined : printer.bed,
            height: printer.height,
            plateCenter: onBed ? [0, 0] : undefined,
            plateCenters: onBed && plates > 1 ? Array.from({ length: plates }, (_, i) => plateCenter(i).toArray() as [number, number]) : undefined,
            thumbnails: await thumbnails(plates),
          },
        })
        : await compute('write3mf', { parts: world, mode: kind === 'cura' ? 'cura' : 'multicolor', thumbnail: (await thumbnails(1))[0], bed: printer.bed });
      await download(new Blob([data as Uint8Array<ArrayBuffer>], { type: 'model/3mf' }), 'zusammengefuegt.3mf');
      const slots = colorSlots(world);
      const objects = new Set(world.map((w) => w.object)).size;
      const where = onBed ? (plates > 1 ? ` ${plates} Platten, alles liegt wie hier.` : ' Alles liegt wie hier auf der Platte.') : ' Nicht alles lag auf der Platte – im Slicer „Anordnen“.';
      toast(project
        ? `Gespeichert – ${objects} Objekt${objects > 1 ? 'e (je Baugruppe eins)' : ''}, ${slots.length} Filament${slots.length > 1 ? 'e' : ''} in den Teilefarben${printer.bambu ? `, Drucker ${printerName(printer)}` : ` – Drucker- und Prozessprofil für den ${printerName(printer)} im Slicer wählen`}.${where}`
        : `Gespeichert. Im Slicer zuordnen: ${slots.map((c, i) => `Extruder ${i + 1} = ${c.toUpperCase()}`).join(' · ')}`, 9000);
      return;
    }
    let body: { name: string; positions: Float32Array }[] = world;
    if ((document.getElementById('unionExport') as HTMLInputElement).checked && world.length > 1) {
      toast('Vereinige die Teile zu einem Körper …', 20000);
      await new Promise((r) => setTimeout(r, 30));
      try {
        const res = await compute('unionParts', { parts: world.map((p) => ({ name: p.name, positions: p.positions })), wasm: manifoldUrl });
        body = [{ name: 'D3DForge', positions: res.positions }];
        toast(res.notClosed.length
          ? `Vereinigt. Nicht geschlossen und deshalb nur angehängt: ${res.notClosed.join(', ')} – vorher Rechtsklick › „Netz reparieren“.`
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
    else {
      const data = await compute('write3mf', { parts: body.map((p) => ({ name: p.name, positions: p.positions, color: '#cccccc' })), mode: 'single', thumbnail: (await thumbnails(1))[0] });
      await download(new Blob([data as Uint8Array<ArrayBuffer>], { type: 'model/3mf' }), 'zusammengefuegt.3mf');
    }
  } catch (e) {
    toast(`Speichern nicht möglich: ${(e as Error).message ?? e}`);
  }
}

// ---------------------------------------------------------------- Druckausrichtung

/**
 * Werkzeuge im 3D-Fenster: „Auf Fläche legen" (der nächste Klick wählt die Fläche, die unten liegen soll)
 * und „Passung" (Klick auf Zapfen, Bohrung oder ebene Fläche, dann Maß eingeben).
 */
const layBtn = document.getElementById('layFlat')!;
const fitBtn = document.getElementById('fitTool')!;
const measureBtn = document.getElementById('measureTool')!;
const paintBtn = document.getElementById('paintTool')!;
const textBtn = document.getElementById('textTool')!;
const paintBar = document.getElementById('paintBar')!;
function setTool(t: typeof tool) {
  tool = t && parts.length > 0 ? t : null;
  layBtn.classList.toggle('on', tool === 'lay');
  fitBtn.classList.toggle('on', tool === 'fit');
  measureBtn.classList.toggle('on', tool === 'measure');
  paintBtn.classList.toggle('on', tool === 'paint');
  if (tool !== 'paint') closePaint();
  else openPaint();
  if (tool !== 'fit') closeFit();
  if (tool !== 'measure') clearMeasure();
  else renderMeasure();
  if (tool !== 'pocket') closePocket();
  textBtn.classList.toggle('on', tool === 'text');
  if (tool !== 'text') closeText();
  else openText();
  if (!tool) updateHover(null);
  renderHint();
}
layBtn.addEventListener('click', () => setTool(tool === 'lay' ? null : 'lay'));
fitBtn.addEventListener('click', () => setTool(tool === 'fit' ? null : 'fit'));
measureBtn.addEventListener('click', () => setTool(tool === 'measure' ? null : 'measure'));

/** Teile der Baugruppe (siehe unitOf), zu der p gehört. */
function unitMembers(p: Part): Part[] {
  const key = unitOf.get(p.id) ?? p.id;
  return parts.filter((q) => (unitOf.get(q.id) ?? q.id) === key);
}

function unitBox(members: Part[]): THREE.Box3 {
  const box = new THREE.Box3();
  for (const q of members) box.union(partBox(q));
  return box;
}

/** Dreht eine Baugruppe (starre Bewegung in Weltkoordinaten) und stellt sie auf die Druckplatte. */
function turnUnit(members: Part[], T: Rigid) {
  const M = rigidToMatrix(T);
  for (const q of members) q.layout.premultiply(M);
  solveAll();
  // Ohne festen Boden trotzdem auf die Platte setzen
  if (!onFloor.checked) {
    const z = Math.min(...members.map((q) => lowestZ(q, q.mesh.matrixWorld)));
    if (Number.isFinite(z) && Math.abs(z) > 1e-9) {
      const D = new THREE.Matrix4().makeTranslation(0, 0, -z);
      for (const q of members) q.layout.premultiply(D);
      solveAll();
    }
  }
  if (gizmoTarget) placeGizmo();
}

/** Name einer Baugruppe für Meldungen. */
const unitName = (members: Part[]) => (members.length > 1 ? `Baugruppe „${getPart(unitOf.get(members[0].id) ?? members[0].id)?.name ?? members[0].name}"` : `„${members[0].name}"`);

function layOnFace(h: Hit) {
  const f = detectFeature(topologyOf(h.part), h.tri, FEATURE_OPTS);
  if (f.kind === 'cylinder' || f.kind === 'edge') {
    toast('Das ist eine runde Fläche – zum Hinlegen eine ebene Fläche anklicken.');
    return;
  }
  const n = new THREE.Vector3(...f.normal).transformDirection(h.part.mesh.matrixWorld);
  if (n.lengthSq() < 0.5) { toast('Diese Stelle hat keine eindeutige Richtung – eine größere ebene Fläche wählen.'); return; }
  const members = unitMembers(h.part);
  const c = unitBox(members).getCenter(new THREE.Vector3());
  turnUnit(members, layFlat([n.x, n.y, n.z], [c.x, c.y, c.z]));
  setTool(null);
  renderAll();
  toast(`${unitName(members)} liegt jetzt mit dieser Fläche auf der Druckplatte.${f.kind === 'surface' ? ' (gekrümmte Fläche: ausgerichtet nach ihrer mittleren Richtung)' : ''}`, 5000);
}

/** Jede sichtbare Baugruppe mit ihrer größten ebenen Auflage auf die Platte legen. */
async function autoOrient() {
  const seen = new Set<number>();
  const units: Part[][] = [];
  for (const p of parts) {
    if (!p.visible) continue;
    const key = unitOf.get(p.id) ?? p.id;
    if (seen.has(key)) continue;
    seen.add(key);
    units.push(unitMembers(p));
  }
  if (!units.length) { toast('Keine sichtbaren Teile.'); return; }
  const btn = document.getElementById('autoOrient') as HTMLButtonElement;
  btn.disabled = true;
  toast('Suche die beste Auflage …', 20000);
  let turned = 0;
  try {
    // Im Hintergrund rechnen, die Teile liegen dabei unverändert
    const choices = await Promise.all(units.map((members) =>
      compute('bestBase', { positions: worldPositions(members) })));
    beginAction();
    units.forEach((members, i) => {
      const c = choices[i];
      // Nur drehen, wenn es spürbar besser aufliegt als jetzt
      if (!(c.area > c.current * 1.05 + 1)) return;
      const box = unitBox(members).getCenter(new THREE.Vector3());
      turnUnit(members, layFlat(c.normal, [box.x, box.y, box.z]));
      turned++;
    });
  } catch (e) {
    toast(`Ausrichten nicht möglich: ${(e as Error).message ?? e}`);
    return;
  } finally {
    btn.disabled = false;
  }
  renderAll();
  toast(turned
    ? `${turned} ${turned === 1 ? 'Baugruppe liegt' : 'Baugruppen liegen'} jetzt mit der größten ebenen Auflage auf der Platte${units.length > turned ? ', die übrigen lagen schon gut' : ''}. Strg+Z macht es rückgängig.`
    : 'Alles liegt schon mit der größten ebenen Auflage auf der Platte.', 6000);
}
document.getElementById('autoOrient')!.addEventListener('click', () => void autoOrient());

/** Dreiecke mehrerer Teile in Weltkoordinaten (ein Feld). */
function worldPositions(members: Part[]): Float32Array {
  const total = members.reduce((n, q) => n + positionsOf(q).length, 0);
  const out = new Float32Array(total);
  const v = new THREE.Vector3();
  let o = 0;
  for (const q of members) {
    const pos = positionsOf(q);
    for (let i = 0; i < pos.length; i += 3, o += 3) v.fromArray(pos, i).applyMatrix4(q.mesh.matrixWorld).toArray(out, o);
  }
  return out;
}

// ---------------------------------------------------------------- Druckprüfung: Überhänge und Wände

/**
 * Überhänge (rot: braucht Stützen) und dünne Wände (lila) einfärben – für die Lage, wie sie gerade ist.
 * Die Wandstärke misst je Dreieck ein Strahl entgegen der Flächennormale bis zur Gegenseite; bei sehr
 * feinen Netzen nur eine Stichprobe. Sie hängt nicht von der Lage ab und wird je Netz einmal berechnet.
 */
let anaOn = false;
let anaGen = 0;
let anaTimer = 0;
const anaGroup = new THREE.Group();
scene.add(anaGroup);
const anaBar = document.getElementById('anaBar')!;
const anaOver = document.getElementById('anaOver') as HTMLInputElement;
const anaAngle = document.getElementById('anaAngle') as HTMLInputElement;
const anaThin = document.getElementById('anaThin') as HTMLInputElement;
const anaMin = document.getElementById('anaMin') as HTMLInputElement;
const anaInfo = document.getElementById('anaInfo')!;
const anaBtn = document.getElementById('anaBtn')!;
const thickCache = new WeakMap<Part, { th: Float32Array; sampled: boolean }>();

async function thicknessOf(p: Part, gen: number): Promise<{ th: Float32Array; sampled: boolean } | null> {
  const hit = thickCache.get(p);
  if (hit) return hit;
  const pos = positionsOf(p);
  const n = pos.length / 9;
  const stride = Math.max(1, Math.ceil(n / 250_000));
  const th = new Float32Array(n).fill(NaN);
  const bvh = p.geom.boundsTree as MeshBVH;
  const ray = new THREE.Ray();
  const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3(), e = new THREE.Vector3(), nrm = new THREE.Vector3();
  let k = 0;
  for (let t = 0; t < n; t += stride) {
    a.fromArray(pos, 9 * t);
    b.fromArray(pos, 9 * t + 3);
    c.fromArray(pos, 9 * t + 6);
    nrm.subVectors(b, a).cross(e.subVectors(c, a));
    if (nrm.lengthSq() > 0) {
      nrm.normalize();
      const o = a.add(b).add(c).multiplyScalar(1 / 3).addScaledVector(nrm, -1e-3);
      ray.set(o, nrm.negate());
      const h = bvh.raycastFirst(ray, THREE.DoubleSide);
      th[t] = h ? h.distance + 1e-3 : Infinity;
    }
    if (++k % 20000 === 0) {
      await new Promise((r) => setTimeout(r, 0));
      if (gen !== anaGen || getPart(p.id) !== p) return null;
    }
  }
  const res = { th, sampled: stride > 1 };
  thickCache.set(p, res);
  return res;
}

/**
 * Liegt direkt unter dem Dreieck Material desselben Teils? Dann ist es eine innere Fläche (z. B. wo zwei
 * nicht vereinigte Körper ineinanderstecken) und braucht keine Stütze.
 */
function restsOnSelf(p: Part, t: number): boolean {
  const pos = positionsOf(p);
  const inv = p.mesh.matrixWorld.clone().invert();
  const down = new THREE.Vector3(0, 0, -1).transformDirection(inv);
  const c = new THREE.Vector3(
    (pos[9 * t] + pos[9 * t + 3] + pos[9 * t + 6]) / 3,
    (pos[9 * t + 1] + pos[9 * t + 4] + pos[9 * t + 7]) / 3,
    (pos[9 * t + 2] + pos[9 * t + 5] + pos[9 * t + 8]) / 3,
  ).addScaledVector(down, 1e-3);
  const h = (p.geom.boundsTree as MeshBVH).raycastFirst(new THREE.Ray(c, down), THREE.DoubleSide);
  if (!h) return false;
  // Trifft der Strahl zuerst eine nach unten zeigende Fläche, beginnt er im Material: innere Fläche.
  // Sonst liegt die Fläche höchstens dicht über Material desselben Teils auf.
  return (h.face?.normal.dot(down) ?? 0) > 0 || h.distance < 0.3;
}

function areaOf(pos: Float32Array, tris: number[]): number {
  let a = 0;
  for (const t of tris) {
    const o = 9 * t;
    const ax = pos[o + 3] - pos[o], ay = pos[o + 4] - pos[o + 1], az = pos[o + 5] - pos[o + 2];
    const bx = pos[o + 6] - pos[o], by = pos[o + 7] - pos[o + 1], bz = pos[o + 8] - pos[o + 2];
    a += Math.hypot(ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx) / 2;
  }
  return a;
}

function scheduleAnalysis() {
  if (!anaOn) return;
  clearTimeout(anaTimer);
  anaTimer = window.setTimeout(() => void runAnalysis(), 120);
}

const cm2 = (mm2: number) => (mm2 >= 100 ? `${(mm2 / 100).toFixed(1).replace('.', ',')} cm²` : `${Math.round(mm2)} mm²`);

async function runAnalysis() {
  const gen = ++anaGen;
  if (!anaOn) { disposeGroup(anaGroup); return; }
  const angle = Math.min(89, Math.max(1, Number(anaAngle.value) || 45));
  const min = Math.max(0.05, Number(String(anaMin.value).replace(',', '.')) || 0.8);
  const visible = parts.filter((p) => p.visible);
  // Plattenhöhe je Baugruppe: ihr tiefster Punkt
  const low = new Map<number, number>();
  for (const p of visible) {
    const key = unitOf.get(p.id) ?? p.id;
    low.set(key, Math.min(low.get(key) ?? Infinity, partBox(p).min.z));
  }
  const layers: { p: Part; tris: Float32Array; color: number; order: number }[] = [];
  let over = 0, thin = 0, sampled = false;
  for (const p of visible) {
    p.mesh.updateMatrixWorld();
    const pos = positionsOf(p);
    if (anaOver.checked) {
      const r = overhangs(pos, p.mesh.matrixWorld.elements, low.get(unitOf.get(p.id) ?? p.id) ?? 0, angle);
      const tris = r.tris.filter((t) => !restsOnSelf(p, t));
      over += r.tris.length === tris.length ? r.area : areaOf(pos, tris);
      if (tris.length) layers.push({ p, tris: pickTriangles(pos, tris), color: 0xe5484d, order: 4 });
    }
    if (anaThin.checked) {
      if (!thickCache.has(p)) anaInfo.textContent = 'Messe Wandstärken …';
      const th = await thicknessOf(p, gen);
      if (!th || gen !== anaGen) return;
      sampled ||= th.sampled;
      const r = thinWalls(pos, th.th, min);
      thin += r.area;
      if (r.tris.length) layers.push({ p, tris: pickTriangles(pos, r.tris), color: 0x9b5de5, order: 5 });
    }
  }
  disposeGroup(anaGroup);
  for (const l of layers) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(l.tris, 3));
    const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({
      color: l.color, transparent: true, opacity: 0.8, side: THREE.DoubleSide, depthWrite: false,
      polygonOffset: true, polygonOffsetFactor: -1, polygonOffsetUnits: -1,
    }));
    m.renderOrder = l.order;
    m.matrixAutoUpdate = false;
    m.matrix.copy(l.p.mesh.matrixWorld);
    m.raycast = () => {};
    anaGroup.add(m);
  }
  const mm = String(min).replace('.', ',');
  anaInfo.innerHTML = [
    anaOver.checked ? (over > 0.5 ? `<span class="chip over"></span>Überhang ${cm2(over)} – braucht Stützen` : 'keine Überhänge') : '',
    anaThin.checked ? (thin > 0.5 ? `<span class="chip thin"></span>dünner als ${mm} mm: ${cm2(thin)}` : `keine Wände unter ${mm} mm`) : '',
    sampled ? '(Wandstärke als Stichprobe)' : '',
  ].filter(Boolean).join(' · ');
}

function setAnalysis(on: boolean) {
  if (on && !parts.length) { toast('Lade zuerst ein Modell.'); return; }
  anaOn = on;
  anaBar.hidden = !on;
  anaBtn.classList.toggle('on', on);
  if (on) void runAnalysis();
  else { anaGen++; disposeGroup(anaGroup); }
}
anaBtn.addEventListener('click', () => setAnalysis(!anaOn));
document.getElementById('anaClose')!.addEventListener('click', () => setAnalysis(false));
for (const el of [anaOver, anaThin]) el.addEventListener('change', scheduleAnalysis);
for (const el of [anaAngle, anaMin]) el.addEventListener('input', scheduleAnalysis);

// ---------------------------------------------------------------- Explosionsansicht

/**
 * Baugruppen auseinandergezogen zeigen – nur fürs Auge: Die Teile rücken beim Zeichnen von der Mitte
 * ihrer Baugruppe weg und gleich danach zurück. Lage, Export und Prüfungen bleiben unberührt. Solange
 * die Ansicht offen ist, sind Markierungen, Auswahlrahmen und das Anklicken aus.
 */
let explode = 0;
let explodeAnim: { from: number; to: number; t0: number } | null = null;
const explodeBar = document.getElementById('explodeBar')!;
const explodeRange = document.getElementById('explodeRange') as HTMLInputElement;
const explodeBtn = document.getElementById('explodeTool')!;

/** Versatz je Teil (Welt) für den Faktor k. */
function explodeOffsets(k: number): Map<Part, THREE.Vector3> {
  const out = new Map<Part, THREE.Vector3>();
  const seen = new Set<number>();
  for (const p of parts) {
    if (!p.visible) continue;
    const key = unitOf.get(p.id) ?? p.id;
    if (seen.has(key)) continue;
    seen.add(key);
    const members = unitMembers(p).filter((q) => q.visible);
    if (members.length < 2) continue;
    const box = unitBox(members);
    const c = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3()).length();
    members.forEach((q, i) => {
      const d = partBox(q).getCenter(new THREE.Vector3()).sub(c);
      // Teile genau in der Mitte (z. B. ineinander gesteckt) nach oben schieben, der Reihe nach
      if (d.length() < size * 0.05) d.set(0, 0, (i + 1) * size * 0.15);
      // Weg von der Mitte: so weit wie das Teil von ihr entfernt ist, plus ein Viertel der Baugruppe
      out.set(q, d.clone().normalize().multiplyScalar(k * (d.length() + size * 0.25)));
    });
  }
  return out;
}

let explodeCache: { key: string; offsets: Map<Part, THREE.Vector3> } | null = null;
/** Vor dem Zeichnen auseinanderziehen; gibt zurück, wie es rückgängig geht. */
function explodeForRender(): () => void {
  if (explodeAnim) {
    const t = Math.min(1, (performance.now() - explodeAnim.t0) / 260);
    explode = explodeAnim.from + (explodeAnim.to - explodeAnim.from) * (1 - (1 - t) ** 3);
    explodeRange.value = String(Math.round(explode * 100));
    if (t >= 1) {
      explode = explodeAnim.to;
      explodeAnim = null;
      if (!explode) {
        setExplodeVisuals(false);
        renderHint();
      }
    }
  }
  if (explode <= 0) return () => {};
  const key = `${explode}|${solveCount}|${parts.length}|${parts.filter((p) => p.visible).length}`;
  if (explodeCache?.key !== key) explodeCache = { key, offsets: explodeOffsets(explode) };
  const moved: THREE.Mesh[] = [];
  for (const [p, d] of explodeCache.offsets) {
    p.mesh.matrix.premultiply(new THREE.Matrix4().makeTranslation(d.x, d.y, d.z));
    p.mesh.matrixWorldNeedsUpdate = true;
    moved.push(p.mesh);
  }
  return () => {
    for (const [p, d] of explodeCache!.offsets) {
      p.mesh.matrix.premultiply(new THREE.Matrix4().makeTranslation(-d.x, -d.y, -d.z));
      p.mesh.matrixWorldNeedsUpdate = true;
      p.mesh.updateMatrixWorld(true);
    }
  };
}

/** Was in der Explosionsansicht nicht passt, ausblenden. */
function setExplodeVisuals(on: boolean) {
  for (const g of [markerGroup, faceGroup, selGroup, checkGroup, anaGroup]) g.visible = !on;
  view.classList.toggle('exploded', on);
}

function setExplode(target: number) {
  if (target > 0 && !parts.length) return;
  if (target > 0) {
    setTool(null);
    if (gizmoTarget) closeGizmo();
    updateHover(null);
    hideTip();
    setExplodeVisuals(true);
  }
  explodeAnim = { from: explode, to: target, t0: performance.now() };
  explodeBar.hidden = target <= 0;
  explodeBtn.classList.toggle('on', target > 0);
  renderHint();
}

explodeBtn.addEventListener('click', () => setExplode(explode > 0 || explodeAnim?.to ? 0 : 0.8));
explodeRange.addEventListener('input', () => {
  explodeAnim = null;
  explode = Number(explodeRange.value) / 100;
  if (explode <= 0) {
    explode = 0.001;
  }
});
document.getElementById('explodeClose')!.addEventListener('click', () => setExplode(0));

// ---------------------------------------------------------------- Schnittansicht

/**
 * Schnitt entlang X, Y oder Z: Alles auf einer Seite der Ebene wird ausgeblendet (Teile, gewählte Flächen,
 * Prüfansicht). Wo die Ebene durch Material geht, liegt eine Schnittfläche in einem dunkleren Ton des
 * Teils: Je Teil zählt der Stencil-Puffer, wie oft der Sehstrahl hinter der Ebene Rück- und Vorderseiten
 * kreuzt (≠ 0 = innen), dann wird dort eine Fläche in der Schnittebene gezeichnet und der Zähler
 * zurückgesetzt. Picken und Messen treffen nur, was sichtbar ist.
 */
const sectionPlane = new THREE.Plane();
const sectionPlanes = [sectionPlane];
/** Achse, Lage (mm) und welche Seite bleibt. */
let section: { axis: 0 | 1 | 2; value: number; keepHigh: boolean } | null = null;
let clipApplied = false;
const sectionBar = document.getElementById('sectionBar')!;
const sectionPos = document.getElementById('sectionPos') as HTMLInputElement;
const sectionBtn = document.getElementById('sectionTool')!;
const sectionOutline = new THREE.LineLoop(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0xe5484d, transparent: true, opacity: 0.8, depthTest: false }));
sectionOutline.renderOrder = 7;
sectionOutline.visible = false;
sectionOutline.raycast = () => {};
scene.add(sectionOutline);

const capGroup = new THREE.Group();
scene.add(capGroup);
const capQuad = new THREE.PlaneGeometry(1, 1);
/** Je Teil: Zählschritte (hängen am Teil) und die Schnittfläche (in der Ebene). */
const caps = new Map<Part, { back: THREE.Mesh; front: THREE.Mesh; quad: THREE.Mesh<THREE.PlaneGeometry, THREE.MeshStandardMaterial> }>();
/** Nach allem anderen zeichnen, je Teil drei Schritte direkt hintereinander. */
let capOrder = 100_000;

function makeCaps(p: Part) {
  const pass = (side: THREE.Side, op: THREE.StencilOp) => {
    const m = new THREE.Mesh(p.geom, new THREE.MeshBasicMaterial({
      side, depthWrite: false, depthTest: false, colorWrite: false, clippingPlanes: sectionPlanes,
      stencilWrite: true, stencilFunc: THREE.AlwaysStencilFunc, stencilFail: op, stencilZFail: op, stencilZPass: op,
    }));
    m.raycast = () => {};
    // Wird mit der Darstellung des Teils freigegeben (das Netz gehört dem Teil)
    m.userData.decor = true;
    return m;
  };
  const back = pass(THREE.BackSide, THREE.IncrementWrapStencilOp);
  const front = pass(THREE.FrontSide, THREE.DecrementWrapStencilOp);
  const quad = new THREE.Mesh(capQuad, new THREE.MeshStandardMaterial({
    roughness: 0.9, metalness: 0, side: THREE.DoubleSide,
    stencilWrite: true, stencilRef: 0, stencilFunc: THREE.NotEqualStencilFunc,
    stencilFail: THREE.ReplaceStencilOp, stencilZFail: THREE.ReplaceStencilOp, stencilZPass: THREE.ReplaceStencilOp,
  }));
  quad.raycast = () => {};
  back.renderOrder = capOrder;
  front.renderOrder = capOrder + 1;
  quad.renderOrder = capOrder + 2;
  capOrder += 3;
  p.mesh.add(back, front);
  capGroup.add(quad);
  caps.set(p, { back, front, quad });
}

function dropCaps(p: Part) {
  const c = caps.get(p);
  if (!c) return;
  for (const m of [c.back, c.front]) {
    p.mesh.remove(m);
    (m.material as THREE.Material).dispose();
  }
  capGroup.remove(c.quad);
  c.quad.material.dispose();
  caps.delete(p);
}

/** Schnittflächen in die Ebene legen (groß genug für den ganzen Aufbau). */
function placeCaps() {
  if (!section) return;
  const box = sceneBox();
  const size = box.isEmpty() ? 1000 : box.getSize(new THREE.Vector3()).length() * 4 + 10;
  const c = box.isEmpty() ? new THREE.Vector3() : box.getCenter(new THREE.Vector3());
  sectionPlane.projectPoint(c, c);
  const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, 1), sectionPlane.normal);
  for (const { quad } of caps.values()) {
    quad.position.copy(c);
    quad.quaternion.copy(q);
    quad.scale.set(size, size, 1);
  }
}

/** Liegt ein Punkt (Welt) im sichtbaren Teil? */
const sectionKeeps = (p: THREE.Vector3, part?: Part) => !section || (!!cutTarget && part?.id !== cutTarget.partId) || sectionPlane.distanceToPoint(p) >= -1e-6;

/** Umriss aller sichtbaren Teile – beim Zerschneiden nur des Teils, das zerschnitten wird. */
function sceneBox(): THREE.Box3 {
  const box = new THREE.Box3();
  for (const p of parts) if (p.visible && (!cutTarget || p.id === cutTarget.partId)) box.union(partBox(p));
  return box;
}

function updateSection() {
  if (!section) return;
  const { axis, value, keepHigh } = section;
  const n = new THREE.Vector3().setComponent(axis, keepHigh ? 1 : -1);
  // Behalten wird, wo n·x + d ≥ 0
  sectionPlane.set(n, keepHigh ? -value : value);
  document.getElementById('sectionVal')!.textContent = `${'XYZ'[axis]} = ${mmText1(value)} mm`;
  // Umriss der Schnittebene über den ganzen Aufbau
  const box = sceneBox();
  if (box.isEmpty()) { sectionOutline.visible = false; return; }
  box.expandByScalar(2);
  const [a, b] = [0, 1, 2].filter((k) => k !== axis);
  const corner = (u: number, v: number) => {
    const c = new THREE.Vector3().setComponent(axis, value);
    c.setComponent(a, u ? box.max.getComponent(a) : box.min.getComponent(a));
    c.setComponent(b, v ? box.max.getComponent(b) : box.min.getComponent(b));
    return c;
  };
  sectionOutline.geometry.setFromPoints([corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)]);
  sectionOutline.visible = true;
  placeCaps();
  if (cutTarget) scheduleCutPreview();
}

/** Schieberegler über die Ausdehnung der Teile entlang der Achse. */
function sectionRange() {
  if (!section) return;
  const box = sceneBox();
  const lo = box.isEmpty() ? -50 : box.min.getComponent(section.axis);
  const hi = box.isEmpty() ? 50 : box.max.getComponent(section.axis);
  sectionPos.min = String(+(lo - 0.5).toFixed(2));
  sectionPos.max = String(+(hi + 0.5).toFixed(2));
  sectionPos.step = String(Math.max(0.05, +((hi - lo) / 400).toFixed(2)));
  section.value = THREE.MathUtils.clamp(section.value, lo - 0.5, hi + 0.5);
  sectionPos.value = String(section.value);
}

function setSection(on: boolean, axis?: 0 | 1 | 2) {
  if (on && !parts.length) { toast('Lade zuerst ein Modell.'); return; }
  if (!on) {
    section = null;
    sectionBar.hidden = true;
    sectionOutline.visible = false;
    if (cutTarget) closeCut();
  } else {
    const ax = axis ?? section?.axis ?? 1;
    const box = sceneBox();
    // Zur Kamera hin aufschneiden: vorne (−Y), rechts (+X) bzw. oben (+Z) fällt weg
    const keepHigh = ax === 1;
    const mid = box.isEmpty() ? 0 : (box.min.getComponent(ax) + box.max.getComponent(ax)) / 2;
    section = { axis: ax, value: section && section.axis === ax ? section.value : mid, keepHigh: section && section.axis === ax ? section.keepHigh : keepHigh };
    sectionBar.hidden = false;
    sectionRange();
    sectionBar.querySelectorAll<HTMLElement>('[data-axis]').forEach((b) => b.classList.toggle('on', Number(b.dataset.axis) === ax));
  }
  sectionBtn.classList.toggle('on', !!section);
  view.classList.toggle('cut', !!section);
  updateSection();
  updateHover(null);
}

/** Je Bild: Schnittebene an allen betroffenen Materialien (auch an neu angelegten). */
function applySectionClip() {
  if (!section && !clipApplied) return;
  const set = (o: THREE.Object3D, on: boolean) => o.traverse((c) => {
    const m = (c as THREE.Mesh).material;
    if (!m) return;
    const planes = on ? sectionPlanes : null;
    for (const mat of Array.isArray(m) ? m : [m]) if (mat.clippingPlanes !== planes) mat.clippingPlanes = planes;
  });
  // Schnittflächen für alle sichtbaren Teile (beim Zerschneiden nur für dieses), entfernte Teile aufräumen
  const cutsPart = (p: Part) => !!section && (!cutTarget || p.id === cutTarget.partId);
  for (const p of [...caps.keys()]) if (!cutsPart(p) || !parts.includes(p)) dropCaps(p);
  if (section) {
    let added = false;
    for (const p of parts) {
      if (!cutsPart(p)) continue;
      if (!caps.has(p)) { makeCaps(p); added = true; }
      const c = caps.get(p)!;
      c.quad.visible = p.visible;
      const hsl = { h: 0, s: 0, l: 0 };
      p.color.getHSL(hsl);
      c.quad.material.color.setHSL(hsl.h, Math.min(1, hsl.s * 1.1), hsl.l * 0.5);
    }
    if (added) placeCaps();
  }
  for (const p of parts) set(p.mesh, cutsPart(p));
  for (const o of [faceGroup, checkGroup, anaGroup, hoverMesh, hoverLine]) set(o, !!section && !cutTarget);
  clipApplied = !!section;
}

sectionBtn.addEventListener('click', () => setSection(!section));
sectionBar.querySelectorAll<HTMLElement>('[data-axis]').forEach((b) => b.addEventListener('click', () => setSection(true, Number(b.dataset.axis) as 0 | 1 | 2)));
sectionPos.addEventListener('input', () => {
  if (!section) return;
  section.value = Number(sectionPos.value);
  updateSection();
});
document.getElementById('sectionFlip')!.addEventListener('click', () => {
  if (!section) return;
  section.keepHigh = !section.keepHigh;
  updateSection();
});
document.getElementById('sectionClose')!.addEventListener('click', () => setSection(false));

// ---------------------------------------------------------------- Neue Netze, alte Auswahl

/**
 * Gewählte Flächen eines Teils auf ein neues Netz übertragen: next(t) = die Dreiecke im neuen Netz, die
 * aus Dreieck t hervorgegangen sind. Flächen ohne Nachfolger fallen weg.
 */
function carryFeatures(partId: number, next: (t: number) => number[]) {
  for (const j of joints) {
    const owner = (f: PickedFeature, side: Side) => f.on ?? (side === 'a' ? j.a : j.b);
    const conv = (f: PickedFeature, side: Side): PickedFeature | null => {
      const pieces = f.pieces?.map((q) => conv(q, side)).filter((q): q is PickedFeature => !!q);
      if (owner(f, side) !== partId) return pieces ? { ...f, pieces } : f;
      const tris = [...new Set(f.tris.flatMap(next))];
      return tris.length ? { ...f, tris, ...(pieces ? { pieces } : {}) } : null;
    };
    const fa = j.fa.map((f) => conv(f, 'a')).filter((f): f is PickedFeature => !!f);
    const fb = j.fb.map((f) => conv(f, 'b')).filter((f): f is PickedFeature => !!f);
    if (fa.some((f, i) => f !== j.fa[i]) || fb.some((f, i) => f !== j.fb[i]) || fa.length !== j.fa.length || fb.length !== j.fb.length) {
      if (fa.length !== j.fa.length || fb.length !== j.fb.length) markersChanged(j);
      j.fa = fa;
      j.fb = fb;
      j.proposals = undefined;
      mergePick.delete(j.id);
    }
  }
}

/** Herkunft (neu → alt) umkehren: je altem Dreieck die neuen. */
function successors(source: Int32Array): (t: number) => number[] {
  const m = new Map<number, number[]>();
  source.forEach((s, i) => {
    if (s < 0) return;
    const l = m.get(s);
    if (l) l.push(i);
    else m.set(s, [i]);
  });
  return (t) => m.get(t) ?? [];
}

// ---------------------------------------------------------------- Netz reparieren

/**
 * Netze reparieren (im Hintergrund): verschweißen, Duplikate weg, Umlaufsinn angleichen, Löcher zu.
 * Die übrigen Dreiecke behalten ihre Reihenfolge – Bemalung und gewählte Flächen werden übertragen
 * (bei umgedrehten Dreiecken entfällt deren Bemalung).
 */
async function repairParts(list: Part[]) {
  if (!list.length) return;
  toast(list.length > 1 ? `Repariere ${list.length} Netze …` : `Repariere „${list[0].name}“ …`, 20000);
  const notes: string[] = [];
  let changed = 0;
  for (const p of list) {
    let r;
    try {
      r = await compute('repairMesh', { positions: positionsOf(p) });
    } catch (err) {
      notes.push(`„${p.name}“: ${(err as Error).message}`);
      continue;
    }
    if (getPart(p.id) !== p) continue;
    const rep = r.report;
    const did = rep.welded + rep.removed + rep.flipped + rep.holes > 0;
    if (!did) {
      p.health = rep.after;
      notes.push(`„${p.name}“ war schon in Ordnung.`);
      continue;
    }
    // Bemalung übertragen
    let lostPaint = false;
    let paint = p.paint;
    if (paint) {
      const n = r.positions.length / 9;
      paint = Object.fromEntries(Object.entries(paint).map(([k, codes]) => {
        const out = new Array<string>(n).fill('');
        codes.forEach((c, t) => {
          const to = r.map[t];
          if (to < 0 || !c) return;
          if (r.flippedTris[to]) lostPaint = true;
          else out[to] = c;
        });
        return [k, out];
      }));
    }
    // Gewählte Flächen auf diesem Teil: neue Dreiecksnummern
    carryFeatures(p.id, (t) => (r.map[t] >= 0 ? [r.map[t]] : []));
    const fresh = replaceGeometry(p, r.positions, { paint, modifiers: p.modifiers });
    fresh.health = rep.after;
    changed++;
    const bits = [
      rep.holes ? `${rep.holes} Loch${rep.holes > 1 ? 'er' : ''} geschlossen` : '',
      rep.flipped ? `${rep.flipped} Dreieck${rep.flipped > 1 ? 'e' : ''} umgedreht` : '',
      rep.removed ? `${rep.removed} doppelte/leere entfernt` : '',
      rep.welded && !rep.holes && !rep.flipped && !rep.removed ? 'Risse geschlossen' : '',
    ].filter(Boolean);
    notes.push(`„${p.name}“: ${bits.join(', ') || 'Punkte verschweißt'}${rep.after.closed ? ' – jetzt geschlossen.' : ` – noch offen (${healthText(rep.after)}).`}${lostPaint ? ' Bemalung umgedrehter Dreiecke entfernt.' : ''}`);
  }
  if (changed) {
    solveAll();
    renderAll();
  } else renderParts();
  toast(notes.join(' '), 8000);
  if (cutTarget && list.some((q) => q.id === cutTarget!.partId)) scheduleCutPreview();
}

/** Millimeter mit Komma, höchstens zwei Nachkommastellen. */
const mmc = (x: number) => String(+x.toFixed(2)).replace('.', ',');

// ---------------------------------------------------------------- Netz vereinfachen

/**
 * Netz vereinfachen (im Hintergrund): weniger Dreiecke – etwa für Scans oder fein modellierte Figuren,
 * die D3DForge und den Slicer bremsen. Die Form weicht höchstens um die gewählte Abweichung ab. Bemalung
 * und gewählte Flächen bleiben genau erhalten: ihre Ränder bleiben stehen, fein bemalte Dreiecke unverändert.
 */
const SIMPLIFY_TOLS = [0.01, 0.02, 0.05, 0.1, 0.2, 0.5];
let simpTarget: { ids: number[]; counts?: number[]; open: string[] } | null = null;
let simpBusy = false;
const simpBar = document.getElementById('simpBar')!;
const simpTol = document.getElementById('simpTol') as HTMLSelectElement;
const simpHelp = document.getElementById('simpHelp')!;
const triCount = (p: Part) => positionsOf(p).length / 9;
const countText = (n: number) => n.toLocaleString('de-DE');
simpTol.innerHTML = SIMPLIFY_TOLS.map((t) => `<option value="${t}">${mmc(t)} mm</option>`).join('');
simpTol.value = '0.05';

/** Dreiecke der gewählten Flächen, die auf diesem Teil liegen (je Fläche eine Liste). */
function featureTrisOf(partId: number): number[][] {
  const out: number[][] = [];
  const visit = (f: PickedFeature, owner: number | null) => {
    if ((f.on ?? owner) === partId && f.tris.length) out.push(f.tris);
    for (const q of f.pieces ?? []) visit(q, owner);
  };
  for (const j of joints) {
    for (const f of j.fa) visit(f, j.a);
    for (const f of j.fb) visit(f, j.b);
  }
  return out;
}

/** Gruppe je Dreieck: gleiche Bemalung und dieselben gewählten Flächen – fein bemalte Dreiecke einzeln. */
function simplifyGroups(p: Part): Uint32Array | undefined {
  const feats = featureTrisOf(p.id);
  const layers = p.paint ? Object.values(p.paint) : [];
  if (!feats.length && !layers.some((codes) => codes.some(Boolean))) return undefined;
  const n = triCount(p);
  const member: string[] = new Array(n).fill('');
  feats.forEach((tris, k) => { for (const t of tris) member[t] += `${k},`; });
  const uniform = new Map<string, boolean>();
  const isUniform = (c: string) => {
    let u = uniform.get(c);
    if (u === undefined) {
      const node = decodePaint(c);
      uniform.set(c, (u = !!node && 'state' in node));
    }
    return u;
  };
  const ids = new Map<string, number>();
  const out = new Uint32Array(n);
  for (let t = 0; t < n; t++) {
    let key = member[t];
    for (const codes of layers) {
      const c = codes[t];
      key += !c ? '|' : isUniform(c) ? `|${c}` : `|#${t}`;
    }
    let id = ids.get(key);
    if (id === undefined) ids.set(key, (id = ids.size));
    out[t] = id;
  }
  return out;
}

function openSimplify(list: Part[]) {
  if (!list.length) return;
  setTool(null);
  closeScale();
  closeSplit();
  if (section) setSection(false);
  if (gizmoTarget) closeGizmo();
  simpTarget = { ids: list.map((q) => q.id), open: [] };
  simpBar.hidden = false;
  renderSimplify();
  void countSimplify();
}

function closeSimplify() {
  simpTarget = null;
  simpBar.hidden = true;
}

const simpParts = () => (simpTarget?.ids ?? []).map(getPart).filter((q): q is Part => !!q);

function renderSimplify() {
  const list = simpParts();
  if (!simpTarget || !list.length) { closeSimplify(); return; }
  const total = list.reduce((s, q) => s + triCount(q), 0);
  document.getElementById('simpTitle')!.textContent = list.length > 1 ? `${list.length} Netze vereinfachen` : `„${list[0].name}“ vereinfachen`;
  const c = simpTarget.counts;
  simpTol.querySelectorAll('option').forEach((o, i) => {
    o.textContent = `${mmc(SIMPLIFY_TOLS[i])} mm${c ? ` – ${countText(c[i])} Dreiecke (−${Math.max(0, Math.round((1 - c[i] / total) * 100))} %)` : ''}`;
  });
  const keep = list.some((q) => simplifyGroups(q));
  simpHelp.textContent = [
    `Jetzt ${countText(total)} Dreiecke.`,
    c ? 'Die Form weicht höchstens um die gewählte Abweichung ab – 0,05 mm sieht man im Druck nicht; für enge Passungen (Bohrungen, Zapfen) lieber 0,01–0,02 mm.' : 'Berechne, wie viele übrig bleiben …',
    keep ? 'Bemalung und gewählte Flächen bleiben erhalten.' : '',
    simpTarget.open.length ? `Nicht geschlossen und deshalb ausgelassen: ${simpTarget.open.join(', ')} – erst „Netz reparieren“.` : '',
  ].filter(Boolean).join(' ');
  (document.getElementById('simpApply') as HTMLButtonElement).disabled = simpBusy || simpTarget.open.length === list.length;
}

/** Dreieckszahl je Abweichung (für die Auswahl), alle Teile zusammen. */
async function countSimplify() {
  const t = simpTarget;
  if (!t) return;
  const sums = SIMPLIFY_TOLS.map(() => 0);
  const open: string[] = [];
  for (const p of simpParts()) {
    try {
      const c = await compute('simplifyCounts', { positions: positionsOf(p), tolerances: SIMPLIFY_TOLS, groups: simplifyGroups(p), wasm: manifoldUrl });
      c.forEach((n, i) => (sums[i] += n));
    } catch (err) {
      if (!userError(err, 'SimplifyError')) { if (simpTarget === t) simpHelp.textContent = `Vereinfachen nicht möglich: ${(err as Error).message}`; return; }
      open.push(`„${p.name}“`);
      // Bleibt, wie es ist
      SIMPLIFY_TOLS.forEach((_, i) => (sums[i] += triCount(p)));
    }
  }
  if (simpTarget !== t) return;
  t.counts = sums;
  t.open = open;
  renderSimplify();
}

async function applySimplify() {
  const t = simpTarget;
  if (!t || simpBusy) return;
  const tol = Number(simpTol.value);
  const list = simpParts();
  simpBusy = true;
  renderSimplify();
  toast(list.length > 1 ? `Vereinfache ${list.length} Netze …` : `Vereinfache „${list[0].name}“ …`, 30000);
  const notes: string[] = [];
  let before = 0;
  let after = 0;
  let changed = 0;
  let lost = 0;
  try {
    for (const p of list) {
      let r;
      try {
        r = await compute('simplifyMesh', { positions: positionsOf(p), tolerance: tol, groups: simplifyGroups(p), wasm: manifoldUrl });
      } catch (err) {
        notes.push(userError(err, 'SimplifyError') ? `„${p.name}“ ist nicht geschlossen – erst „Netz reparieren“.` : `„${p.name}“: ${(err as Error).message}`);
        continue;
      }
      if (getPart(p.id) !== p) continue;
      if (r.after >= r.before) { notes.push(`„${p.name}“ lässt sich so nicht weiter vereinfachen.`); continue; }
      const c = p.paint ? carryPaint(positionsOf(p), p.paint, r.positions, r.source) : { positions: r.positions, paint: undefined, lost: 0 };
      lost += c.lost;
      carryFeatures(p.id, successors(r.source));
      const fresh = replaceGeometry(p, c.positions, { paint: c.paint, modifiers: p.modifiers });
      checkHealth(fresh);
      before += r.before;
      after += r.after;
      changed++;
    }
  } finally {
    simpBusy = false;
  }
  closeSimplify();
  if (changed) {
    solveAll();
    renderAll();
  }
  toast([
    changed ? `${changed > 1 ? `${changed} Netze` : 'Netz'} vereinfacht: ${countText(before)} → ${countText(after)} Dreiecke (−${Math.round((1 - after / before) * 100)} %), Abweichung höchstens ${mmc(tol)} mm.` : '',
    lost ? `Feine Bemalung an ${lost} Dreieck${lost > 1 ? 'en' : ''} zu je einer Farbe vereinfacht.` : '',
    ...notes,
  ].filter(Boolean).join(' '), 7000);
}

document.getElementById('simpApply')!.addEventListener('click', () => void applySimplify());
document.getElementById('simpCancel')!.addEventListener('click', closeSimplify);

// ---------------------------------------------------------------- Zerschneiden

/**
 * Ein Teil an der Schnittebene in zwei Teile zerschneiden – etwa weil es nicht auf die Platte passt oder
 * ohne Stützen druckbar werden soll. Die Schnittansicht zeigt dabei nur dieses Teil aufgeschnitten; die
 * sichtbare Hälfte bekommt die Zapfen, die andere die Bohrungen (oder beide Bohrungen für Dübel).
 * Danach liegen beide Hälften mit der Schnittfläche nach unten (Zapfen nach oben) nebeneinander.
 */
let cutTarget: { partId: number } | null = null;
let cutConnector: Connector = 'pin';
const cutBar = document.getElementById('cutBar')!;
const cutCount = document.getElementById('cutCount') as HTMLSelectElement;
const cutDia = document.getElementById('cutDia') as HTMLInputElement;
const cutGap = document.getElementById('cutGap') as HTMLInputElement;
const cutFlat = document.getElementById('cutFlat') as HTMLInputElement;
const cutHelp = document.getElementById('cutHelp')!;
const cutPreview = new THREE.Group();
scene.add(cutPreview);
let cutTimer = 0;
let cutSeq = 0;
let cutBusy = false;

function openCut(p: Part) {
  setTool(null);
  closeScale();
  closeSimplify();
  if (gizmoTarget) closeGizmo();
  cutTarget = { partId: p.id };
  document.getElementById('cutTitle')!.textContent = `„${p.name}“ zerschneiden`;
  cutBar.hidden = false;
  renderCutBar();
  // Waagerecht durch die Mitte, die untere Hälfte bleibt sichtbar (Zapfen zeigen nach oben)
  if (!section) {
    setSection(true, 2);
    if (section) {
      const box = partBox(p);
      const sec = section as { axis: 0 | 1 | 2; value: number; keepHigh: boolean };
      sec.value = (box.min.z + box.max.z) / 2;
      sec.keepHigh = false;
      sectionRange();
      updateSection();
    }
  } else {
    sectionRange();
    updateSection();
  }
}

function closeCut() {
  cutTarget = null;
  cutBar.hidden = true;
  clearCutPreview();
  void compute('releaseCut', undefined);
  if (section) { sectionRange(); updateSection(); }
}

function renderCutBar() {
  cutBar.querySelectorAll<HTMLElement>('[data-conn]').forEach((b) => b.classList.toggle('on', b.dataset.conn === cutConnector));
  document.getElementById('cutDiaLabel')!.textContent = cutConnector === 'dovetail' ? 'Breite' : '⌀';
  for (const el of cutBar.querySelectorAll<HTMLElement>('.conn-opt')) el.hidden = cutConnector === 'none';
}

function clearCutPreview() {
  for (const o of [...cutPreview.children]) {
    cutPreview.remove(o);
    (o as THREE.Mesh).geometry.dispose();
    ((o as THREE.Mesh).material as THREE.Material).dispose();
  }
}

/** Schnittebene und Einstellungen in den Koordinaten des Teils (Hälfte A = die sichtbare). */
function cutOptions(p: Part) {
  p.mesh.updateMatrixWorld();
  const local = sectionPlane.clone().applyMatrix4(p.mesh.matrixWorld.clone().invert());
  // Behalten wird k·x + c ≥ 0; die Normale fürs Schneiden zeigt zur weggeschnittenen Seite
  return {
    normal: [-local.normal.x, -local.normal.y, -local.normal.z] as Vec3,
    offset: local.constant,
    connector: cutConnector,
    count: Number(cutCount.value) || 0,
    diameter: Number(cutDia.value) || 0,
    clearance: Math.max(0, Number(cutGap.value) || 0),
  };
}

function scheduleCutPreview() {
  clearTimeout(cutTimer);
  cutTimer = window.setTimeout(() => void updateCutPreview(), 180);
}


async function updateCutPreview() {
  const p = cutTarget && getPart(cutTarget.partId);
  if (!p || !section) return;
  const seq = ++cutSeq;
  const opts = cutOptions(p);
  if (opts.connector === 'none') {
    clearCutPreview();
    cutHelp.textContent = 'Schnitt ohne Verbinder – die Hälften lassen sich z. B. kleben.';
    return;
  }
  let res;
  try {
    res = await compute('planConnectors', { positions: positionsOf(p), opts, key: meshKey(positionsOf(p)), wasm: manifoldUrl });
  } catch {
    return;
  }
  if (seq !== cutSeq || !cutTarget) return;
  clearCutPreview();
  if (!res.closed) {
    cutHelp.innerHTML = 'Das Teil ist nicht geschlossen (nicht wasserdicht) – so lässt es sich nicht zerschneiden. <button class="ghost small" data-cut-repair>Netz reparieren</button>';
    return;
  }
  const W = p.mesh.matrixWorld;
  const n = new THREE.Vector3(...opts.normal).transformDirection(W);
  const color = new THREE.Color(accentColor);
  if (res.shape) {
    // Schwalbenschwanz: die Feder selbst zeigen
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(res.shape, 3));
    g.computeVertexNormals();
    const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.75, depthTest: false }));
    m.renderOrder = 8;
    m.raycast = () => {};
    m.matrixAutoUpdate = false;
    m.matrix.copy(W);
    cutPreview.add(m);
  }
  for (const c of res.shape ? [] : res.connectors) {
    const h = c.depthA + c.depthB;
    const m = new THREE.Mesh(new THREE.CylinderGeometry(c.diameter / 2, c.diameter / 2, h, 32), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.75, depthTest: false }));
    m.renderOrder = 8;
    m.raycast = () => {};
    // Zylinder entlang y → entlang der Schnittnormale, von −TiefeA bis +TiefeB
    m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), n);
    m.position.copy(new THREE.Vector3(...c.center).applyMatrix4(W)).addScaledVector(n, (c.depthB - c.depthA) / 2);
    cutPreview.add(m);
  }
  const kind = cutConnector === 'pin' ? 'Zapfen' : cutConnector === 'snap' ? 'Schnappstift' + (res.connectors.length > 1 ? 'e' : '') : 'Dübel';
  const first = res.connectors[0];
  cutHelp.textContent = cutConnector === 'dovetail'
    ? [
      res.connectors.length
        ? `${res.connectors.length} Schwalbenschwanz${res.connectors.length > 1 ? 'e' : ''}, ${mmc(first.diameter)} mm breit, ${mmc(first.depthB)} mm hoch, Spiel ${mmc(opts.clearance)} mm. Die sichtbare Hälfte bekommt die Feder, die andere wird seitlich aufgeschoben.`
        : 'Kein Platz für einen Schwalbenschwanz.',
      ...res.notes,
    ].filter(Boolean).join(' ')
    : [
      res.connectors.length
        ? `${res.connectors.length} ${kind} ⌀ ${mmc(first.diameter)} mm, Spiel ${mmc(opts.clearance)} mm, ${mmc(Math.min(...res.connectors.map((c) => Math.min(c.depthA, c.depthB))))}–${mmc(Math.max(...res.connectors.map((c) => Math.max(c.depthA, c.depthB))))} mm tief.`
        : 'Kein Platz für Verbinder.',
      cutConnector === 'pin' ? 'Die sichtbare Hälfte bekommt die Zapfen.' : '',
      cutConnector === 'snap' ? 'Beide Hälften bekommen Bohrungen mit Hinterschnitt, die Stifte werden eigene Teile (flach liegend drucken) und rasten beim Zusammenstecken ein.' : '',
      ...res.notes,
    ].filter(Boolean).join(' ');
}

/** Name der Hälfte nach ihrer Lage zur Ebene (Richtung d zeigt in die Hälfte). */
function halfName(d: THREE.Vector3): string {
  const a = [Math.abs(d.x), Math.abs(d.y), Math.abs(d.z)];
  const k = a.indexOf(Math.max(...a));
  return [['links', 'rechts'], ['vorne', 'hinten'], ['unten', 'oben']][k][d.getComponent(k) > 0 ? 1 : 0];
}

async function applyCut() {
  const p = cutTarget && getPart(cutTarget.partId);
  if (!p || !section || cutBusy) return;
  const opts = cutOptions(p);
  cutBusy = true;
  toast('Schneide …', 20000);
  let res;
  try {
    res = await compute('cutMesh', { positions: positionsOf(p), opts, key: meshKey(positionsOf(p)), wasm: manifoldUrl });
  } catch (err) {
    toast(userError(err, 'CutError') ? `${(err as Error).message}${/wasserdicht/.test((err as Error).message) ? ' Erst „Netz reparieren“.' : ''}` : `Zerschneiden fehlgeschlagen: ${(err as Error).message}`, 6000);
    return;
  } finally {
    cutBusy = false;
  }
  if (getPart(p.id) !== p) return;
  const W = p.mesh.matrixWorld.clone();
  // Richtung in die sichtbare Hälfte (Welt)
  const k = sectionPlane.normal.clone();
  // Alles, was am Teil hängt, bleibt, wo es ist – die Verbindungen zum Teil selbst entfallen
  for (const q of unitMembers(p)) q.layout.copy(q.mesh.matrixWorld);
  const wasBase = baseId === p.id;
  // Bemalung: die Hälften stammen Dreieck für Dreieck aus dem Teil
  const carry = (pos: Float32Array, source: Int32Array) => (p.paint ? carryPaint(positionsOf(p), p.paint, pos, source) : { positions: pos, paint: undefined, lost: 0 });
  const ca = carry(res.a, res.aSource);
  const cb = carry(res.b, res.bSource);
  const lost = [p.modifiers?.length ? 'Modifier/Text entfernt' : '', ca.lost + cb.lost ? 'Feine Bemalung an der Schnittkante zu je einer Farbe vereinfacht' : ''].filter(Boolean);
  const color = `#${p.color.getHexString()}`;
  const extras = { settings: p.settings, objectSettings: p.objectSettings, palette: p.palette };
  const geo = (pos: Float32Array) => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.computeVertexNormals();
    g.computeBoundingBox();
    g.computeBoundingSphere();
    return g;
  };
  removePart(p.id);
  const kept = addPart(`${p.name} (${halfName(k)})`, geo(ca.positions), { color, layout: W, extras: { ...extras, paint: ca.paint } });
  const other = addPart(`${p.name} (${halfName(k.clone().negate())})`, geo(cb.positions), { color, layout: W, extras: { ...extras, paint: cb.paint } });
  const dowels = res.dowels.map((d, i) => addPart(`${p.name} ${cutConnector === 'snap' ? 'Schnappstift' : 'Dübel'} ${i + 1}`, geo(d), { color, layout: W }));
  // Schnappstifte liegen auf ihrer Abflachung (Richtung in Weltkoordinaten)
  const flat = res.dowelFlat && new THREE.Vector3(...res.dowelFlat).transformDirection(W);
  if (wasBase) baseId = kept.id;
  solveAll();
  // Zapfen und Feder stehen heraus: diese Hälfte mit der Schnittfläche nach oben
  const pins = (cutConnector === 'pin' || cutConnector === 'dovetail') && res.connectors.length > 0;
  if (cutFlat.checked) {
    // Schnittflächen nach unten; Zapfen zeigen nach oben; Dübel stehen
    const turn = (q: Part, down: THREE.Vector3) => {
      const c = partBox(q).getCenter(new THREE.Vector3());
      turnUnit([q], layFlat([down.x, down.y, down.z], [c.x, c.y, c.z]));
    };
    turn(kept, pins ? k : k.clone().negate());
    turn(other, k);
    for (const d of dowels) turn(d, flat ?? k);
  }
  // Nebeneinander: die andere Hälfte rechts der sichtbaren, die Dübel dahinter
  solveAll();
  const gap = Math.max(5, sceneScale() * 0.05);
  const ref = partBox(kept);
  let x = ref.max.x + gap;
  const row = cutFlat.checked ? [other, ...dowels] : dowels;
  for (const q of row) {
    const b = partBox(q);
    q.layout.premultiply(new THREE.Matrix4().makeTranslation(x - b.min.x, (ref.min.y + ref.max.y) / 2 - (b.min.y + b.max.y) / 2, 0));
    solveAll();
    x = partBox(q).max.x + gap;
  }
  selected.clear();
  for (const q of [kept, other, ...dowels]) selected.add(q.id);
  setSection(false);
  solveAll();
  renderAll();
  // Die Teile liegen jetzt woanders: alles ins Bild
  fitView();
  const nc = res.connectors.length;
  const what = !nc ? '' : cutConnector === 'dovetail'
    ? ` mit ${nc > 1 ? `${nc} Schwalbenschwänzen` : 'Schwalbenschwanz'} (${mmc(res.connectors[0].diameter)} mm breit) – die obere Hälfte seitlich aufschieben`
    : ` mit ${nc} ${cutConnector === 'pin' ? 'Zapfen' : cutConnector === 'snap' ? (nc > 1 ? 'Schnappstiften' : 'Schnappstift') : nc > 1 ? 'Dübeln' : 'Dübel'} ⌀ ${mmc(res.connectors[0].diameter)} mm${cutConnector === 'snap' ? ' – zum Zusammenstecken einrasten lassen' : ''}`;
  toast(`„${p.name}“ zerschnitten${what}.${lost.length ? ` ${lost.join(', ')}.` : ''}${res.notes.length ? ` ${res.notes.join(' ')}` : ''}`, 6000);
}

cutBar.querySelectorAll<HTMLElement>('[data-conn]').forEach((b) => b.addEventListener('click', () => {
  cutConnector = b.dataset.conn as Connector;
  renderCutBar();
  scheduleCutPreview();
}));
for (const el of [cutCount, cutDia, cutGap]) el.addEventListener('input', scheduleCutPreview);
document.getElementById('cutApply')!.addEventListener('click', () => void applyCut());
cutHelp.addEventListener('click', (e) => {
  const p = cutTarget && getPart(cutTarget.partId);
  if (p && (e.target as HTMLElement).closest('[data-cut-repair]')) void repairParts([p]);
});
document.getElementById('cutCancel')!.addEventListener('click', () => setSection(false));

// ---------------------------------------------------------------- Aussparungen

/**
 * Aussparung an der angeklickten Stelle: Magnet, Gewindeeinsatz, Schraube (Zylinder-/Senkkopf), Mutter
 * oder Bohrung – senkrecht in die Fläche. Gewählte Flächen und Bemalung bleiben, wo die Aussparung sie
 * nicht wegnimmt.
 */
let pocketKind: PocketKind = 'magnet';
let pocketBusy = false;
const pocketBar = document.getElementById('pocketBar')!;
const pocketPreset = document.getElementById('pocketPreset') as HTMLSelectElement;
const pocketDia = document.getElementById('pocketDia') as HTMLInputElement;
const pocketDepth = document.getElementById('pocketDepth') as HTMLInputElement;
const pocketHead = document.getElementById('pocketHead') as HTMLInputElement;
const pocketSunk = document.getElementById('pocketSunk') as HTMLInputElement;
const pocketThrough = document.getElementById('pocketThrough') as HTMLInputElement;
const pocketGap = document.getElementById('pocketGap') as HTMLInputElement;
const pocketHelp = document.getElementById('pocketHelp')!;
const pocketGroup = new THREE.Group();
pocketGroup.matrixAutoUpdate = false;
scene.add(pocketGroup);
let pocketTimer = 0;
let pocketSeq = 0;

const DIA_LABEL: Record<PocketKind, string> = { magnet: '⌀', insert: 'Loch ⌀', screw: 'Loch ⌀', nut: 'Schlüsselweite', hole: '⌀' };
const DEPTH_LABEL: Record<PocketKind, string> = { magnet: 'Tiefe', insert: 'Tiefe', screw: 'Kopftiefe', nut: 'Höhe', hole: 'Tiefe' };

function pickPocket(h: Hit) {
  // Runde ebene Fläche (Zapfen, Dom): in die Mitte
  const f = detectFeature(topologyOf(h.part), h.tri, FEATURE_OPTS);
  let point: Vec3 = h.raw;
  let normal: Vec3 = [...topologyOf(h.part).normals.subarray(3 * h.tri, 3 * h.tri + 3)] as Vec3;
  if (f.kind === 'plane') {
    normal = f.normal;
    if (f.circle) point = f.circle.center;
  }
  pocketPlace = { partId: h.part.id, point, normal };
  pocketBar.hidden = false;
  document.getElementById('pocketTitle')!.textContent = `Aussparung in „${h.part.name}“`;
  renderPocket();
  renderHint();
}

function closePocket() {
  if (!pocketPlace && document.getElementById('pocketBar')!.hidden) return;
  pocketPlace = null;
  pocketBar.hidden = true;
  clearPocketPreview();
}

function clearPocketPreview() {
  for (const o of [...pocketGroup.children]) {
    pocketGroup.remove(o);
    (o as THREE.Mesh).geometry.dispose();
    ((o as THREE.Mesh).material as THREE.Material).dispose();
  }
}

/** Voreinstellungen der gewählten Art in die Felder. */
function applyPreset() {
  const pre = PRESETS[pocketKind][Number(pocketPreset.value)];
  if (!pre) return;
  const sp = pre.spec;
  pocketDia.value = String(sp.diameter);
  pocketDepth.value = String(+sp.depth.toFixed(2));
  pocketGap.value = String(sp.clearance ?? 0);
  if (pocketKind === 'screw') pocketHead.value = String(pocketSunk.checked ? COUNTERSUNK_HEAD[pre.label] ?? sp.head : sp.head);
  pocketThrough.checked = pocketKind === 'hole' ? false : !!sp.through;
}

function renderPocket() {
  pocketBar.querySelectorAll<HTMLElement>('[data-pkind]').forEach((b) => b.classList.toggle('on', b.dataset.pkind === pocketKind));
  document.getElementById('pocketDiaLabel')!.textContent = DIA_LABEL[pocketKind];
  document.getElementById('pocketDepthLabel')!.textContent = DEPTH_LABEL[pocketKind];
  for (const el of pocketBar.querySelectorAll<HTMLElement>('[data-for]')) el.hidden = !el.dataset.for!.split(' ').includes(pocketKind);
  document.getElementById('pocketThroughLabel')!.textContent = pocketKind === 'hole' ? 'ganz durch' : 'Schraubenloch durch';
  schedulePocketPreview();
}

function pocketSpec(): PocketSpec {
  const pre = PRESETS[pocketKind][Number(pocketPreset.value)];
  return {
    kind: pocketKind,
    diameter: Math.max(0.5, Number(pocketDia.value) || 1),
    depth: Math.max(0.2, Number(pocketDepth.value) || 1),
    clearance: Math.max(0, Number(pocketGap.value) || 0),
    head: Number(pocketHead.value) || undefined,
    countersunk: pocketKind === 'screw' && pocketSunk.checked,
    through: pocketThrough.checked,
    bolt: pocketKind === 'nut' ? pre?.spec.bolt ?? Number(pocketDia.value) * 0.6 : undefined,
  };
}

function schedulePocketPreview() {
  clearTimeout(pocketTimer);
  pocketTimer = window.setTimeout(() => void updatePocketPreview(), 90);
}

async function updatePocketPreview() {
  const p = pocketPlace && getPart(pocketPlace.partId);
  if (!p || !pocketPlace) { clearPocketPreview(); return; }
  const seq = ++pocketSeq;
  const place = { point: pocketPlace.point, normal: pocketPlace.normal };
  const spec = pocketSpec();
  let res;
  try {
    res = await pocketPreview(positionsOf(p), place, spec, manifoldWasm);
  } catch {
    return;
  }
  if (seq !== pocketSeq || !pocketPlace) return;
  clearPocketPreview();
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(res.tool, 3));
  g.computeVertexNormals();
  const accent = accentColor;
  const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color: accent, transparent: true, opacity: 0.55, depthTest: false }));
  m.renderOrder = 8;
  m.raycast = () => {};
  pocketGroup.add(m);
  p.mesh.updateMatrixWorld();
  pocketGroup.matrix.copy(p.mesh.matrixWorld);
  pocketGroup.matrixWorldNeedsUpdate = true;
  const t = res.thickness;
  const blind = pocketKind === 'magnet' || pocketKind === 'insert' || (pocketKind === 'hole' && !spec.through);
  const mm = (x: number) => x.toFixed(1).replace('.', ',');
  pocketHelp.innerHTML = [
    Number.isFinite(t) ? `Material hier: ${mm(t)} mm.` : '',
    blind && Number.isFinite(t) && spec.depth > t - 0.6
      ? `<span class="warn">${spec.depth >= t ? 'Die Aussparung geht durch.' : `Hinten bleiben nur ${mm(t - spec.depth)} mm.`}</span>` : '',
    pocketKind === 'insert' ? 'Werte für gängige Einsätze – mit dem Datenblatt abgleichen.' : '',
    pocketKind === 'magnet' ? 'Tiefe mit 0,2 mm Zugabe; Spiel je Seite.' : '',
  ].filter(Boolean).join(' ');
}

async function applyPocket() {
  const p = pocketPlace && getPart(pocketPlace.partId);
  if (!p || !pocketPlace || pocketBusy) return;
  pocketBusy = true;
  let r;
  try {
    r = await compute('addPocket', { positions: positionsOf(p), place: { point: pocketPlace.point, normal: pocketPlace.normal }, spec: pocketSpec(), wasm: manifoldUrl });
  } catch (err) {
    toast(userError(err, 'PocketError') ? (err as Error).message : `Aussparung nicht möglich: ${(err as Error).message}`, 6000);
    return;
  } finally {
    pocketBusy = false;
  }
  if (getPart(p.id) !== p) return;
  let positions = r.positions;
  let paint = p.paint;
  let lost = 0;
  if (paint) {
    const c = carryPaint(positionsOf(p), paint, positions, r.source);
    ({ paint, positions } = c);
    lost = c.lost;
  }
  carryFeatures(p.id, successors(r.source));
  const fresh = replaceGeometry(p, positions, { paint, modifiers: p.modifiers });
  pocketPlace.partId = fresh.id;
  solveAll();
  renderAll();
  clearPocketPreview();
  const what = { magnet: 'Magnettasche', insert: 'Loch für den Gewindeeinsatz', screw: 'Schraubenloch', nut: 'Muttertasche', hole: 'Bohrung' }[pocketKind];
  toast(`${what} eingesetzt.${r.notes.length ? ` ${r.notes.join(' ')}` : ''}${lost ? ` Feine Bemalung an ${lost} Dreiecken zu je einer Farbe vereinfacht.` : ''} Nächste Stelle anklicken oder Esc.`, 6000);
}

pocketBar.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('[data-pkind]');
  if (!b) return;
  pocketKind = b.dataset.pkind as PocketKind;
  pocketPreset.innerHTML = PRESETS[pocketKind].map((pr, i) => `<option value="${i}">${pr.label}</option>`).join('');
  pocketPreset.value = String(pocketKind === 'magnet' ? 2 : pocketKind === 'hole' ? 2 : Math.min(2, PRESETS[pocketKind].length - 1));
  applyPreset();
  renderPocket();
});
pocketPreset.addEventListener('change', () => { applyPreset(); renderPocket(); });
pocketSunk.addEventListener('change', () => { applyPreset(); renderPocket(); });
for (const el of [pocketDia, pocketDepth, pocketHead, pocketGap]) el.addEventListener('input', schedulePocketPreview);
pocketThrough.addEventListener('change', schedulePocketPreview);
document.getElementById('pocketApply')!.addEventListener('click', () => void applyPocket());
document.getElementById('pocketClose')!.addEventListener('click', () => setTool(null));
pocketPreset.innerHTML = PRESETS.magnet.map((pr, i) => `<option value="${i}">${pr.label}</option>`).join('');
pocketPreset.value = '2';
applyPreset();

// ---------------------------------------------------------------- Beschriften

/**
 * Beschriftung (Taste W): Text oder SVG-Logo an der angeklickten Stelle erhaben aufsetzen oder vertieft
 * einlassen – die Form folgt der Oberfläche. „Nummern“ prägt 1/7, 2/7 … in die ausgewählten Teile, je auf
 * die größte Fläche unten (liegt auf der Platte) oder oben. Auf Wunsch in eigenem Filament (als Bemalung).
 */
let textMode: EmbossMode = 'raised';
let textSide: 'bottom' | 'top' = 'bottom';
let textLogo: { name: string; outline: Outline; skipped: number } | null = null;
let textBusy = false;
const textBar = document.getElementById('textBar')!;
const textValue = document.getElementById('textValue') as HTMLTextAreaElement;
const textFont = document.getElementById('textFont') as HTMLSelectElement;
const textSize = document.getElementById('textSize') as HTMLInputElement;
const textDepth = document.getElementById('textDepth') as HTMLInputElement;
const textRotate = document.getElementById('textRotate') as HTMLInputElement;
const textColor = document.getElementById('textColor') as HTMLSelectElement;
const textPattern = document.getElementById('textPattern') as HTMLInputElement;
const textStart = document.getElementById('textStart') as HTMLInputElement;
const textLogoFile = document.getElementById('textLogoFile') as HTMLInputElement;
const textApplyBtn = document.getElementById('textApply') as HTMLButtonElement;
const textHelp = document.getElementById('textHelp')!;
const textGroup = new THREE.Group();
scene.add(textGroup);
let textTimer = 0;
let textSeq = 0;

/** Eine Beschriftung, bereit zum Einprägen (Ort und Form in Koordinaten des Teils). */
interface TextJob { part: Part; place: EmbossPlace; spec: EmbossSpec; label: string; outline: Outline }

/**
 * Teile für die Nummern: die ausgewählten, sonst alle sichtbaren – in der Reihenfolge der Liste. Dübel und
 * Schnappstifte sind untereinander gleich und bleiben ohne Nummer (außer es sind nur solche gewählt).
 */
function textTargets(): Part[] {
  const sel = parts.filter((p) => p.visible && selected.has(p.id));
  const list = sel.length ? sel : parts.filter((p) => p.visible);
  const pieces = list.filter((p) => !/ (Dübel|Schnappstift) \d+$/.test(p.name));
  return pieces.length ? pieces : list;
}

const numIn = (el: HTMLInputElement, lo: number, hi: number, dflt: number) => {
  const v = Number(el.value);
  return Number.isFinite(v) && el.value.trim() !== '' ? Math.min(hi, Math.max(lo, v)) : dflt;
};
/** Unten nur vertieft – erhaben läge das Teil nicht mehr flach auf der Platte. */
const textModeNow = (): EmbossMode => (textSrc === 'number' && textSide === 'bottom' ? 'engraved' : textMode);
const scaledOutline = (o: Outline, k: number): Outline => ({ polys: o.polys.map((p) => p.map(([x, y]) => [x * k, y * k] as [number, number])), width: o.width * k, height: o.height * k });

/** „Oben“ für die Schrift: an senkrechten Flächen die Welt-Höhe, an waagerechten die Bildschirm-Höhe (lesbar aus der Ansicht). */
function textUpFor(p: Part, n: Vec3): Vec3 {
  p.mesh.updateMatrixWorld();
  const M = p.mesh.matrixWorld;
  const nW = new THREE.Vector3(...n).transformDirection(M);
  const upW = Math.abs(nW.z) > 0.7 ? new THREE.Vector3(0, 1, 0).applyQuaternion(camera.quaternion) : new THREE.Vector3(0, 0, 1);
  const u = upW.transformDirection(M.clone().invert());
  return [u.x, u.y, u.z];
}

function pickText(h: Hit) {
  if (textSrc === 'number') {
    toggleSelected(h.part.id);
    scheduleTextPreview();
    renderHint();
    return;
  }
  const topo = topologyOf(h.part);
  const f = detectFeature(topo, h.tri, FEATURE_OPTS);
  const normal: Vec3 = f.kind === 'plane' ? f.normal : [...topo.normals.subarray(3 * h.tri, 3 * h.tri + 3)] as Vec3;
  textPlace = { partId: h.part.id, place: { point: h.raw, normal, up: textUpFor(h.part, normal) } };
  renderText();
  renderHint();
}

function openText() {
  textBar.hidden = false;
  renderText();
}

function closeText() {
  if (!textPlace && document.getElementById('textBar')!.hidden) return;
  textPlace = null;
  textBar.hidden = true;
  clearTextPreview();
}

function clearTextPreview() {
  for (const g of [...textGroup.children]) {
    textGroup.remove(g);
    for (const o of g.children) {
      (o as THREE.Mesh).geometry.dispose();
      ((o as THREE.Mesh).material as THREE.Material).dispose();
    }
  }
}

function setTextSource(s: typeof textSrc) {
  textSrc = s;
  if (s !== 'number') clearTextPreview();
  renderText();
  renderHint();
}

function renderTextColors() {
  const list = filamentList();
  const cur = textColor.value;
  textColor.innerHTML = '<option value="">wie das Teil</option>'
    + list.map((c, i) => `<option value="${c}">Filament ${i + 1} (${c})</option>`).join('')
    + '<option value="+">Neue Farbe …</option>';
  textColor.value = list.includes(cur) ? cur : '';
}

function renderText() {
  textBar.querySelectorAll<HTMLElement>('[data-tsrc]').forEach((b) => b.classList.toggle('on', b.dataset.tsrc === textSrc));
  const mode = textModeNow();
  textBar.querySelectorAll<HTMLButtonElement>('[data-tmode]').forEach((b) => {
    b.classList.toggle('on', b.dataset.tmode === mode);
    b.disabled = textSrc === 'number' && textSide === 'bottom' && b.dataset.tmode === 'raised';
  });
  textBar.querySelectorAll<HTMLElement>('[data-tside]').forEach((b) => b.classList.toggle('on', b.dataset.tside === textSide));
  for (const el of textBar.querySelectorAll<HTMLElement>('[data-tfor]')) el.hidden = !el.dataset.tfor!.split(' ').includes(textSrc);
  document.getElementById('textDepthLabel')!.textContent = mode === 'raised' ? 'Höhe' : 'Tiefe';
  document.getElementById('textSizeLabel')!.textContent = textSrc === 'number' ? 'bis' : 'Größe';
  const picker = document.getElementById('textLogoPick')!;
  picker.textContent = textLogo ? `${textLogo.name} ✓` : 'SVG laden …';
  const p = textPlace && getPart(textPlace.partId);
  document.getElementById('textTitle')!.textContent = textSrc === 'number' ? 'Nummerieren' : p ? `Beschriften: „${p.name}“` : 'Beschriften';
  textApplyBtn.textContent = textSrc === 'number' ? 'Nummern einprägen' : 'Einprägen';
  renderTextColors();
  scheduleTextPreview();
}

/** Was „Einprägen“ jetzt täte – und welche Teile (beim Nummerieren) keinen Platz haben. */
function textJobs(): { jobs: TextJob[]; skipped: Part[] } {
  const font = textFont.value as FontName;
  const size = numIn(textSize, 1, 300, 8);
  const depth = numIn(textDepth, 0.1, 20, 0.6);
  const mode = textModeNow();
  if (textSrc !== 'number') {
    const p = textPlace && getPart(textPlace.partId);
    if (!p || !textPlace) return { jobs: [], skipped: [] };
    const outline = textSrc === 'logo'
      ? textLogo && scaledOutline(textLogo.outline, size)
      : textOutline(textValue.value, size, font);
    if (!outline || !outline.polys.length) return { jobs: [], skipped: [] };
    const label = textSrc === 'logo' ? textLogo!.name : textValue.value.trim();
    return { jobs: [{ part: p, place: textPlace.place, spec: { polys: outline.polys, mode, depth, rotate: Number(textRotate.value) || 0 }, label, outline }], skipped: [] };
  }
  const targets = textTargets();
  const start = Math.round(numIn(textStart, 0, 1e6, 1));
  const pattern = textPattern.value.trim() || '{n}';
  const fmt = (n: number, N: number) => pattern.replaceAll('{n}', String(n)).replaceAll('{N}', String(N));
  // Platz für die breiteste Nummer suchen; Teile ohne Platz zählen nicht mit
  const last = start + targets.length - 1;
  const unit = textOutline(fmt(last, last), 1, font);
  if (!unit.polys.length) return { jobs: [], skipped: [] };
  const found = targets.map((p) => ({ p, spot: labelSpot(worldPositions([p]), textSide, { w: unit.width, h: unit.height }, size) }));
  const ok = found.filter((x) => x.spot);
  const N = start + ok.length - 1;
  const jobs = ok.map(({ p, spot }, i): TextJob => {
    const label = fmt(start + i, N);
    const outline = textOutline(label, spot!.height, font);
    const inv = p.mesh.matrixWorld.clone().invert();
    const pt = new THREE.Vector3(...spot!.point).applyMatrix4(inv);
    const n = new THREE.Vector3(...spot!.normal).transformDirection(inv);
    const up = new THREE.Vector3(...spot!.up).transformDirection(inv);
    return { part: p, label, outline, place: { point: [pt.x, pt.y, pt.z], normal: [n.x, n.y, n.z], up: [up.x, up.y, up.z] }, spec: { polys: outline.polys, mode, depth } };
  });
  return { jobs, skipped: found.filter((x) => !x.spot).map((x) => x.p) };
}

function scheduleTextPreview() {
  clearTimeout(textTimer);
  textTimer = window.setTimeout(() => void updateTextPreview(), 120);
}

async function updateTextPreview() {
  const seq = ++textSeq;
  if (tool !== 'text') return;
  const { jobs, skipped } = textJobs();
  const shapes: { part: Part; tool: Float32Array }[] = [];
  let info: { relief: number; missing: number } | null = null;
  let error = '';
  for (const j of jobs) {
    try {
      const r = await embossPreview(bvhSurface(j.part.geom.boundsTree as MeshBVH), j.place, j.spec, manifoldWasm);
      shapes.push({ part: j.part, tool: r.tool });
      info = r;
    } catch (err) {
      error = (err as Error).message;
    }
    if (seq !== textSeq) return;
  }
  clearTextPreview();
  for (const s of shapes) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(s.tool, 3));
    g.computeVertexNormals();
    const m = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color: textColor.value && textColor.value !== '+' ? textColor.value : accentColor, transparent: true, opacity: 0.6, depthTest: false }));
    m.renderOrder = 8;
    m.raycast = () => {};
    const holder = new THREE.Group();
    holder.matrixAutoUpdate = false;
    s.part.mesh.updateMatrixWorld();
    holder.matrix.copy(s.part.mesh.matrixWorld);
    holder.add(m);
    textGroup.add(holder);
  }
  const mm = mmText1;
  const mode = textModeNow();
  const out: string[] = [];
  if (textSrc === 'number') {
    const n = jobs.length + skipped.length;
    out.push(n ? `${jobs.length} von ${n} Teil${n === 1 ? '' : 'en'} bekommen eine Nummer (${jobs.map((j) => esc(j.label)).slice(0, 8).join(', ')}${jobs.length > 8 ? ' …' : ''}), ${textSide === 'bottom' ? 'unten vertieft' : `oben ${mode === 'raised' ? 'erhaben' : 'vertieft'}`}, so groß wie die Fläche erlaubt.` : 'Keine sichtbaren Teile.');
    if (skipped.length) out.push(`<span class="warn">Kein Platz auf: ${skipped.slice(0, 6).map((p) => `„${esc(p.name)}“`).join(', ')}${skipped.length > 6 ? ' …' : ''}</span> (zu kleine Fläche – andere Seite oder kleineres Muster probieren).`);
  } else if (!textPlace) {
    out.push(textSrc === 'logo' && !textLogo ? 'Erst „SVG laden …“ – gefüllte Flächen werden zur Form –, dann die Stelle anklicken.' : 'Die Stelle auf dem Teil anklicken.');
  } else if (jobs.length) {
    const o = jobs[0].outline;
    out.push(`${mm(o.width)} × ${mm(o.height)} mm, ${mode === 'raised' ? `${mm(numIn(textDepth, 0.1, 20, 0.6))} mm erhaben` : `${mm(numIn(textDepth, 0.1, 20, 0.6))} mm tief`}.`);
    if (info && info.relief > 0.05) out.push(`Folgt der Wölbung (${mm(info.relief)} mm Höhenunterschied).`);
    if (info && info.missing > 0.02) out.push('<span class="warn">Ragt über eine Kante oder Stufe hinaus – dort läuft sie eben weiter.</span>');
    if (textSrc === 'text' && numIn(textSize, 1, 300, 8) < 4) out.push('Unter 4 mm Schrifthöhe werden Buchstaben mit 0,4-mm-Düse unscharf – fett hilft.');
    if (textSrc === 'logo' && textLogo?.skipped) out.push(`${textLogo.skipped} Linie${textLogo.skipped === 1 ? '' : 'n'} ohne Füllung übersprungen.`);
  } else if (textSrc === 'text') out.push('Text eingeben.');
  if (error) out.push(`<span class="warn">${esc(error)}</span>`);
  if (textColor.value) out.push('Die Farbe kommt als Bemalung ins Slicer-Projekt.');
  textHelp.innerHTML = out.join(' ');
}

/** Ergebnis ins Teil übernehmen: Bemalung mitnehmen, auf Wunsch die neuen Flächen einfärben. */
function commitEmboss(p: Part, r: EmbossResult): Part {
  let positions = r.positions;
  let paint = p.paint;
  let palette = p.palette;
  if (paint) {
    const c = carryPaint(positionsOf(p), paint, positions, r.source);
    ({ paint, positions } = c);
    if (c.lost) r.notes.push(`Feine Bemalung an ${c.lost} Dreiecken zu je einer Farbe vereinfacht.`);
  }
  const color = textColor.value;
  if (color && color !== '+' && color !== hexOf(p)) {
    const pal = [...(palette ?? [])];
    const code = solidPaint(paintStateFor(p, pal, color));
    const codes = paint?.paint_color ? [...paint.paint_color] : new Array<string>(r.source.length).fill('');
    for (let t = 0; t < r.source.length; t++) if (r.source[t] < 0) codes[t] = code;
    paint = { ...paint, paint_color: codes };
    palette = pal;
  }
  carryFeatures(p.id, successors(r.source));
  return replaceGeometry(p, positions, { paint, palette, modifiers: p.modifiers });
}

async function applyText() {
  if (textBusy) return;
  const { jobs } = textJobs();
  if (!jobs.length) {
    toast(textSrc === 'number' ? 'Auf keinem Teil ist Platz für die Nummer.' : textSrc === 'logo' && !textLogo ? 'Erst ein SVG-Logo laden.' : !textPlace ? 'Klicke zuerst die Stelle an.' : 'Kein Text zum Einprägen.');
    return;
  }
  textBusy = true;
  textApplyBtn.disabled = true;
  const notes: string[] = [];
  let done = 0;
  try {
    for (const j of jobs) {
      const who = jobs.length > 1 ? `„${j.part.name}“: ` : '';
      let r: EmbossResult;
      try {
        r = await compute('emboss', { positions: positionsOf(j.part), place: j.place, spec: j.spec, wasm: manifoldUrl });
      } catch (err) {
        notes.push(who + (userError(err, 'EmbossError') ? (err as Error).message : `Beschriften nicht möglich: ${(err as Error).message}`));
        continue;
      }
      if (getPart(j.part.id) !== j.part) continue;
      commitEmboss(j.part, r);
      notes.push(...r.notes.map((n) => who + n));
      done++;
    }
  } finally {
    textBusy = false;
    textApplyBtn.disabled = false;
  }
  if (done) {
    solveAll();
    renderAll();
  }
  clearTextPreview();
  if (textSrc !== 'number' && done) textPlace = null;
  // Nummern sind fertig – sonst zeigte die Vorschau gleich die nächsten auf denselben Teilen
  if (textSrc === 'number' && done) setTool(null);
  const what = textSrc === 'number' ? `${done} Teil${done === 1 ? '' : 'e'} nummeriert.` : textSrc === 'logo' ? 'Logo eingeprägt.' : 'Beschriftung eingeprägt.';
  toast(done ? `${what}${notes.length ? ` ${notes.join(' ')}` : ''}${textSrc === 'number' ? '' : ' Nächste Stelle anklicken oder Esc.'}` : notes.join(' ') || 'Nichts eingeprägt.', 7000);
  if (tool === 'text') renderText();
  renderHint();
}

textBar.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLButtonElement>('[data-tsrc], [data-tmode], [data-tside]');
  if (!b || b.disabled) return;
  if (b.dataset.tsrc) {
    setTextSource(b.dataset.tsrc as typeof textSrc);
    if (textSrc === 'logo' && !textLogo) textLogoFile.click();
    return;
  }
  if (b.dataset.tmode) textMode = b.dataset.tmode as EmbossMode;
  if (b.dataset.tside) textSide = b.dataset.tside as typeof textSide;
  renderText();
});
for (const el of [textValue, textSize, textDepth, textRotate, textPattern, textStart]) el.addEventListener('input', scheduleTextPreview);
textFont.addEventListener('change', scheduleTextPreview);
textColor.addEventListener('change', () => {
  if (textColor.value !== '+') { scheduleTextPreview(); return; }
  // Neue Farbe: über den Farbwähler, danach steht sie auch beim Malen bereit
  const pick = document.createElement('input');
  pick.type = 'color';
  pick.value = '#c12e1f';
  pick.addEventListener('change', () => {
    const c = pick.value.toLowerCase();
    if (!paintExtra.includes(c)) paintExtra.push(c);
    renderTextColors();
    textColor.value = c;
    scheduleTextPreview();
  });
  textColor.value = '';
  pick.click();
});
textValue.addEventListener('keydown', (e) => {
  if (e.key === 'Enter' && !e.shiftKey) {
    e.preventDefault();
    void applyText();
  }
});
document.getElementById('textLogoPick')!.addEventListener('click', () => textLogoFile.click());
textLogoFile.addEventListener('change', async () => {
  const f = textLogoFile.files?.[0];
  textLogoFile.value = '';
  if (!f) return;
  try {
    const o = await svgOutline(await f.text(), 1);
    if (!o.polys.length) { toast('In der SVG-Datei sind keine gefüllten Flächen.', 5000); return; }
    textLogo = { name: f.name, outline: o, skipped: o.skipped };
    setTextSource('logo');
  } catch (err) {
    toast(`SVG nicht lesbar: ${(err as Error).message}`, 5000);
  }
});
textApplyBtn.addEventListener('click', () => void applyText());
document.getElementById('textClose')!.addEventListener('click', () => setTool(null));
textBtn.addEventListener('click', () => setTool(tool === 'text' ? null : 'text'));

// ---------------------------------------------------------------- Auf Druckplattengröße zerteilen

/**
 * Ein Teil, das nicht auf die Platte passt, in gleich große Stücke zerteilen – so viele wie nötig, damit
 * jedes Stück (mit Rand für die Verbinder) auf die Platte des gewählten Druckers passt. Die Stücke
 * bekommen Dübel (oder Zapfen) und liegen danach auseinandergerückt wie im Ganzen.
 */
let splitTarget: { partId: number; counts: [number, number, number] } | null = null;
let splitConnector: Connector = 'dowel';
let splitBusy = false;
const splitBar = document.getElementById('splitBar')!;
const splitGap = document.getElementById('splitGap') as HTMLInputElement;
const splitPreview = new THREE.Group();
scene.add(splitPreview);

/** Rand je Stück: Zapfen stehen bis zu 12 mm heraus. */
const splitMargin = () => (splitConnector === 'pin' ? 15 : splitConnector === 'dovetail' ? 10 : splitConnector === 'snap' ? 4 : 3);

/** Passt das Teil (so, wie es liegt) auf die Platte – auch um 90° gedreht? */
function fitsBed(size: THREE.Vector3, margin = 0): boolean {
  const printer = currentPrinter();
  const [w, d] = printer.bed.map((x) => x - margin);
  const flat = (size.x <= w + 0.01 && size.y <= d + 0.01) || (size.x <= d + 0.01 && size.y <= w + 0.01);
  return flat && (!printer.height || size.z <= printer.height - margin + 0.01);
}

function openSplit(p: Part) {
  setTool(null);
  closeScale();
  closeSimplify();
  if (section) setSection(false);
  if (gizmoTarget) closeGizmo();
  const printer = currentPrinter();
  const s = partBox(p).getSize(new THREE.Vector3());
  const plan = planSplit([s.x, s.y, s.z], printer.bed, printer.height, splitMargin());
  splitTarget = { partId: p.id, counts: plan.counts };
  splitBar.hidden = false;
  renderSplit();
}

function closeSplit() {
  splitTarget = null;
  splitBar.hidden = true;
  clearSplitPreview();
}

function clearSplitPreview() {
  for (const o of [...splitPreview.children]) {
    splitPreview.remove(o);
    (o as THREE.Mesh).geometry.dispose();
    ((o as THREE.Mesh).material as THREE.Material).dispose();
  }
}

function renderSplit() {
  const p = splitTarget && getPart(splitTarget.partId);
  if (!p || !splitTarget) { closeSplit(); return; }
  const printer = currentPrinter();
  const box = partBox(p);
  const s = box.getSize(new THREE.Vector3());
  const c = splitTarget.counts;
  document.getElementById('splitTitle')!.textContent = `„${p.name}“ (${mmText1(s.x)} × ${mmText1(s.y)} × ${mmText1(s.z)} mm) für den ${printerName(printer)} zerteilen`;
  for (let a = 0; a < 3; a++) {
    splitBar.querySelector(`[data-split-count="${a}"]`)!.textContent = String(c[a]);
    (splitBar.querySelector(`[data-split-dec="${a}"]`) as HTMLButtonElement).disabled = c[a] <= 1;
    (splitBar.querySelector(`[data-split-inc="${a}"]`) as HTMLButtonElement).disabled = c[a] >= 12;
  }
  splitBar.querySelectorAll<HTMLElement>('[data-sconn]').forEach((b) => b.classList.toggle('on', b.dataset.sconn === splitConnector));
  const piece = new THREE.Vector3(s.x / c[0], s.y / c[1], s.z / c[2]);
  const total = c[0] * c[1] * c[2];
  const fits = fitsBed(piece, splitMargin());
  (document.getElementById('splitApply') as HTMLButtonElement).disabled = total < 2;
  document.getElementById('splitHelp')!.innerHTML = `${total} Stück${total > 1 ? 'e' : ''}, je ${mmText1(piece.x)} × ${mmText1(piece.y)} × ${mmText1(piece.z)} mm`
    + (fits ? ' – jedes passt auf die Platte.' : ' – <span class="warn">so passen die Stücke noch nicht auf die Platte.</span>')
    + (splitConnector === 'none' ? ' Ohne Verbinder (zum Kleben).' : splitConnector === 'pin' ? ' Zapfen zeigen nach +X/+Y/oben.' : splitConnector === 'dovetail' ? ' Schwalbenschwänze: die Stücke werden seitlich aufgeschoben.' : splitConnector === 'snap' ? ' Schnappstifte werden eigene Teile (flach liegend) und rasten ein.' : ' Dübel werden eigene Teile.');
  // Schnittebenen als rote Rechtecke
  clearSplitPreview();
  const fill = new THREE.Color(0xe5484d);
  for (let a = 0; a < 3; a++) {
    for (const v of splitPlanes(box.min.getComponent(a), box.max.getComponent(a), c[a])) {
      const [u, w] = [0, 1, 2].filter((k) => k !== a);
      const corner = (i: number, j: number) => new THREE.Vector3().setComponent(a, v)
        .setComponent(u, i ? box.max.getComponent(u) : box.min.getComponent(u))
        .setComponent(w, j ? box.max.getComponent(w) : box.min.getComponent(w));
      const pts = [corner(0, 0), corner(1, 0), corner(1, 1), corner(0, 1)];
      const line = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(pts), new THREE.LineBasicMaterial({ color: fill, depthTest: false, transparent: true, opacity: 0.9 }));
      const g = new THREE.BufferGeometry().setFromPoints([pts[0], pts[1], pts[2], pts[0], pts[2], pts[3]]);
      const quad = new THREE.Mesh(g, new THREE.MeshBasicMaterial({ color: fill, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false }));
      for (const o of [line, quad]) {
        o.raycast = () => {};
        o.renderOrder = 7;
        splitPreview.add(o);
      }
    }
  }
}

/** Ausdehnung eines Netzes (eigene Koordinaten) entlang einer Weltachse. */
function extentAlong(positions: Float32Array, W: THREE.Matrix4, axis: number): [number, number] {
  const e = W.elements;
  let lo = Infinity, hi = -Infinity;
  for (let i = 0; i < positions.length; i += 3) {
    const v = e[axis] * positions[i] + e[4 + axis] * positions[i + 1] + e[8 + axis] * positions[i + 2] + e[12 + axis];
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  return [lo, hi];
}

async function applySplit() {
  const p = splitTarget && getPart(splitTarget.partId);
  if (!p || !splitTarget || splitBusy) return;
  const counts = splitTarget.counts;
  const W = p.mesh.matrixWorld.clone();
  const inv = W.clone().invert();
  const box = partBox(p);
  const size = box.getSize(new THREE.Vector3());
  const planes: { axis: number; value: number }[] = [];
  for (let a = 0; a < 3; a++) for (const v of splitPlanes(box.min.getComponent(a), box.max.getComponent(a), counts[a])) planes.push({ axis: a, value: v });
  if (!planes.length) return;
  const clearance = Math.max(0, Number(splitGap.value) || 0);
  const orig = positionsOf(p);
  let pieces: { positions: Float32Array; cell: number[]; source: Int32Array }[] = [{ positions: orig, cell: [0, 0, 0], source: new Int32Array(orig.length / 9).map((_, i) => i) }];
  const dowels: Float32Array[] = [];
  /** Schnappstifte: Richtung zur Abflachung je Stift (Koordinaten des Teils). */
  const flats: (Vec3 | undefined)[] = [];
  const notes = new Set<string>();
  splitBusy = true;
  let done = 0;
  const cuts = counts[0] * counts[1] * counts[2] - 1;
  try {
    for (const pl of planes) {
      const next: typeof pieces = [];
      for (const piece of pieces) {
        const [lo, hi] = extentAlong(piece.positions, W, pl.axis);
        if (!(lo < pl.value - 0.01 && hi > pl.value + 0.01)) { next.push(piece); continue; }
        toast(`Zerteile „${p.name}“ … ${done + 1} von ${cuts}`, 30000);
        await new Promise((r) => setTimeout(r, 20));
        // Ebene in Teilkoordinaten; Hälfte B liegt in Richtung der Achse
        const local = new THREE.Plane(new THREE.Vector3().setComponent(pl.axis, 1), -pl.value).applyMatrix4(inv);
        const r = await compute('cutMesh', { positions: piece.positions, opts: { normal: [local.normal.x, local.normal.y, local.normal.z], offset: -local.constant, connector: splitConnector, clearance }, wasm: manifoldUrl });
        for (const n of r.notes) notes.add(n);
        const up = [...piece.cell];
        up[pl.axis]++;
        // Herkunft bis zum Ausgangsteil verketten (für die Bemalung)
        const chain = (src: Int32Array) => src.map((t) => (t >= 0 ? piece.source[t] : -1));
        next.push({ positions: r.a, cell: piece.cell, source: chain(r.aSource) }, { positions: r.b, cell: up, source: chain(r.bSource) });
        dowels.push(...r.dowels);
        flats.push(...r.dowels.map(() => r.dowelFlat));
        done++;
      }
      pieces = next;
    }
  } catch (err) {
    toast(userError(err, 'CutError') ? `${(err as Error).message}${/wasserdicht/.test((err as Error).message) ? ' Erst „Netz reparieren“.' : ''}` : `Zerteilen fehlgeschlagen: ${(err as Error).message}`, 7000);
    return;
  } finally {
    splitBusy = false;
    void compute('releaseCut', undefined);
  }
  if (getPart(p.id) !== p) return;
  // Was am Teil hängt, bleibt, wo es ist; die Verbindungen zum Teil selbst entfallen
  for (const q of unitMembers(p)) q.layout.copy(q.mesh.matrixWorld);
  const wasBase = baseId === p.id;
  const color = `#${p.color.getHexString()}`;
  const extras = { settings: p.settings, objectSettings: p.objectSettings };
  const geo = (pos: Float32Array) => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.computeVertexNormals();
    g.computeBoundingBox();
    g.computeBoundingSphere();
    return g;
  };
  removePart(p.id);
  // Reihenfolge: von unten nach oben, vorne nach hinten, links nach rechts
  pieces.sort((a, b) => a.cell[2] - b.cell[2] || a.cell[1] - b.cell[1] || a.cell[0] - b.cell[0]);
  const made = pieces.map((pc, i) => {
    const c = p.paint ? carryPaint(orig, p.paint, pc.positions, pc.source) : { positions: pc.positions, paint: undefined };
    return { part: addPart(`${p.name} ${i + 1}/${pieces.length}`, geo(c.positions), { color, layout: W, extras: { ...extras, paint: c.paint, palette: p.palette } }), cell: pc.cell };
  });
  if (wasBase) baseId = made[0].part.id;
  solveAll();
  // Auseinanderrücken; höhere Lagen kommen hinter die unteren auf die Platte
  const gap = Math.max(8, Math.min(size.x, size.y) * 0.04);
  for (const { part, cell } of made) {
    const b = partBox(part);
    part.layout.premultiply(new THREE.Matrix4().makeTranslation(
      cell[0] * gap,
      cell[1] * gap + cell[2] * (size.y + counts[1] * gap + gap),
      box.min.z - b.min.z,
    ));
  }
  solveAll();
  // Dübel stehend in einer Reihe davor, Schnappstifte flach auf ihrer Abflachung
  let x = box.min.x;
  for (const [i, d] of dowels.entries()) {
    const q = addPart(`${p.name} ${flats[i] ? 'Schnappstift' : 'Dübel'} ${i + 1}`, geo(d), { color, layout: W });
    const s = partBox(q).getSize(new THREE.Vector3());
    const f = flats[i];
    const axis = f ? new THREE.Vector3(...f).transformDirection(W)
      : s.x >= s.y && s.x >= s.z ? new THREE.Vector3(1, 0, 0) : s.y >= s.z ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(0, 0, 1);
    const c = partBox(q).getCenter(new THREE.Vector3());
    turnUnit([q], layFlat([axis.x, axis.y, axis.z], [c.x, c.y, c.z]));
    const b = partBox(q);
    q.layout.premultiply(new THREE.Matrix4().makeTranslation(x - b.min.x, box.min.y - gap * 2 - b.max.y, 0));
    solveAll();
    x = partBox(q).max.x + 4;
    made.push({ part: q, cell: [0, 0, 0] });
  }
  selected.clear();
  for (const { part } of made) selected.add(part.id);
  closeSplit();
  solveAll();
  renderAll();
  fitView();
  toast(`„${p.name}“ in ${pieces.length} Stücke zerteilt${dowels.length ? ` + ${dowels.length} ${flats[0] ? 'Schnappstift' + (dowels.length > 1 ? 'e' : '') : 'Dübel'}` : ''}. „Auf der Platte anordnen“ (A) legt sie auf die Platte.${notes.size ? ` ${[...notes].join(' ')}` : ''}`, 8000);
}

splitBar.addEventListener('click', (e) => {
  const t = e.target as HTMLElement;
  if (!splitTarget) return;
  const dec = t.closest<HTMLElement>('[data-split-dec]');
  const inc = t.closest<HTMLElement>('[data-split-inc]');
  const conn = t.closest<HTMLElement>('[data-sconn]');
  if (dec) splitTarget.counts[Number(dec.dataset.splitDec)] = Math.max(1, splitTarget.counts[Number(dec.dataset.splitDec)] - 1);
  else if (inc) splitTarget.counts[Number(inc.dataset.splitInc)] = Math.min(12, splitTarget.counts[Number(inc.dataset.splitInc)] + 1);
  else if (conn) splitConnector = conn.dataset.sconn as Connector;
  else return;
  renderSplit();
});
document.getElementById('splitApply')!.addEventListener('click', () => void applySplit());
document.getElementById('splitCancel')!.addEventListener('click', closeSplit);

// ---------------------------------------------------------------- Teile verrechnen

/**
 * Teile miteinander verrechnen: vereinigen (die anderen gehen im Ziel auf), die anderen aus dem Ziel
 * ausschneiden oder nur die Überschneidung behalten (die anderen werden dabei ausgeblendet, nicht
 * gelöscht). Das Ziel behält Lage, Verbindungen, Bemalung und Modifier.
 */
let boolBusy = false;
async function combineParts(target: Part, others: Part[], op: BoolOp) {
  if (boolBusy || !others.length) return;
  boolBusy = true;
  target.mesh.updateMatrixWorld();
  const inv = target.mesh.matrixWorld.clone().invert();
  const into = (q: Part) => {
    q.mesh.updateMatrixWorld();
    return inv.clone().multiply(q.mesh.matrixWorld);
  };
  toast(op === 'union' ? 'Vereinige …' : op === 'subtract' ? 'Schneide aus …' : 'Berechne die Überschneidung …', 20000);
  let r;
  try {
    r = await compute('booleanParts', {
      target: { name: target.name, positions: positionsOf(target) },
      others: others.map((q) => ({ name: q.name, positions: transformMesh(positionsOf(q), into(q).elements) })),
      op, wasm: manifoldUrl,
    });
  } catch (err) {
    toast(userError(err, 'BoolError') ? (err as Error).message : `Nicht möglich: ${(err as Error).message}`, 7000);
    return;
  } finally {
    boolBusy = false;
  }
  if (getPart(target.id) !== target) return;
  // Was an den anderen hängt, bleibt, wo es ist
  for (const q of others) for (const m of unitMembers(q)) m.layout.copy(m.mesh.matrixWorld);
  let positions = r.positions;
  let paint = target.paint;
  let lost = 0;
  if (paint) {
    const c = carryPaint(positionsOf(target), paint, positions, r.source);
    ({ paint, positions } = c);
    lost = c.lost;
  }
  carryFeatures(target.id, successors(r.source));
  // Beim Vereinigen kommen die Modifier der anderen mit (in die Koordinaten des Ziels)
  let modifiers = target.modifiers;
  if (op === 'union') {
    const extra = others.flatMap((q) => (q.modifiers ?? []).map((m) => {
      const L = into(q);
      return { ...m, positions: transformMesh(m.positions, L.elements), transform: m.transform && L.clone().multiply(new THREE.Matrix4().fromArray(m.transform)).toArray() };
    }));
    if (extra.length) modifiers = [...(modifiers ?? []), ...extra];
  }
  const fresh = replaceGeometry(target, positions, { paint, modifiers });
  const otherPaint = op === 'union' && others.some((q) => q.paint);
  if (op === 'union') for (const q of others) removePart(q.id);
  else for (const q of others) { q.visible = false; q.mesh.visible = false; }
  selected.clear();
  selected.add(fresh.id);
  solveAll();
  renderAll();
  const names = others.map((q) => `„${q.name}“`).join(', ');
  toast((op === 'union' ? `${names} mit „${target.name}“ vereinigt.`
    : op === 'subtract' ? `${names} aus „${target.name}“ ausgeschnitten – ${others.length > 1 ? 'die Werkzeuge sind' : 'das Werkzeug ist'} ausgeblendet.`
    : `Nur die Überschneidung von „${target.name}“ mit ${names} bleibt – die anderen sind ausgeblendet.`)
    + (lost ? ` Feine Bemalung an ${lost} Dreiecken zu je einer Farbe vereinfacht.` : '') + (otherPaint ? ' Die Bemalung der anderen Teile entfällt.' : ''), 7000);
}

/** Einträge fürs Verrechnen der Auswahl (Ziel: zuletzt angeklicktes Teil, sonst das erste). */
function boolEntries(list: Part[]): { label: string; run: () => void; title?: string }[] {
  if (list.length < 2) return [];
  const anchor = list.find((q) => q.id === selAnchor) ?? list[0];
  const rest = (t: Part) => list.filter((q) => q !== t);
  return [
    { label: `Vereinigen (zu „${anchor.name}“)`, run: () => void combineParts(anchor, rest(anchor), 'union'), title: 'Ein Körper ohne Überlappungen und Innenwände' },
    ...list.slice(0, 4).map((t) => ({ label: `Die anderen aus „${t.name}“ ausschneiden`, run: () => void combineParts(t, rest(t), 'subtract'), title: 'Die anderen Teile werden zu Aussparungen und danach ausgeblendet' })),
    { label: `Nur die Überschneidung behalten (in „${anchor.name}“)`, run: () => void combineParts(anchor, rest(anchor), 'intersect') },
  ];
}

const selBool = document.getElementById('selBool') as HTMLSelectElement;
let boolChoices: { label: string; run: () => void }[] = [];
function renderBoolSelect(list: Part[]) {
  boolChoices = boolEntries(list);
  selBool.innerHTML = `<option value="">Verrechnen …</option>${boolChoices.map((c, i) => `<option value="${i}">${esc(c.label)}</option>`).join('')}`;
}
selBool.addEventListener('change', () => {
  const c = boolChoices[Number(selBool.value)];
  selBool.value = '';
  if (c) {
    beginAction();
    c.run();
  }
});

// ---------------------------------------------------------------- Toleranz-Test

/**
 * Testkörper fürs eigene Spiel: Platte mit Löchern in steigendem Spiel und ein Zapfen. Das ermittelte
 * Spiel lässt sich als Standard speichern – Zerschneiden und Zerteilen nehmen es dann vor.
 */
const tolBar = document.getElementById('tolBar')!;
const tolDia = document.getElementById('tolDia') as HTMLInputElement;
const tolList = document.getElementById('tolList') as HTMLInputElement;
const tolMine = document.getElementById('tolMine') as HTMLInputElement;

const tolFor = document.getElementById('tolFor')!;
/** Eigenes Spiel je Drucker und Material (z. B. „P2S|petg“), mit dem Testkörper ermittelt. */
const CLEAR_KEY = 'partjoin.clearances';
const clearKey = () => `${bambuPrinter.value}|${matPrefs.kind}`;
function savedClearances(): Record<string, number> {
  try {
    const v = JSON.parse(localStorage.getItem(CLEAR_KEY) ?? '{}');
    return v && typeof v === 'object' ? v : {};
  } catch {
    return {};
  }
}

/** Spiel für Drucker und Material; sonst das allgemein gespeicherte (ältere Fassung), sonst 0,2 mm. */
function myClearance(): number {
  const own = savedClearances()[clearKey()];
  if (typeof own === 'number' && own >= 0 && own < 2) return own;
  try {
    const v = Number(localStorage.getItem('partjoin.clearance'));
    return v > 0 && v < 2 ? v : 0.2;
  } catch {
    return 0.2;
  }
}
const clearanceFor = () => `${printerName(currentPrinter())} · ${material().label}`;
function applyMyClearance() {
  const c = String(myClearance());
  cutGap.value = c;
  splitGap.value = c;
  tolMine.value = clearanceLabel(myClearance());
  tolFor.textContent = `für ${clearanceFor()}${savedClearances()[clearKey()] === undefined ? ' (noch nicht gemessen)' : ''}`;
}

const parseList = (s: string) => s.split(/[\s;]+/).map((x) => Number(x.replace(',', '.'))).filter((x) => x >= 0 && x < 2);

async function generateTolerance() {
  const d = Math.max(2, Math.min(30, Number(tolDia.value.replace(',', '.')) || 10));
  const list = parseList(tolList.value).slice(0, 12);
  if (!list.length) { toast('Bitte mindestens einen Spielwert eingeben, z. B. „0,1 0,2 0,3“.'); return; }
  toast('Erzeuge den Testkörper …', 20000);
  let r;
  try {
    r = await compute('toleranceTest', { opts: { diameter: d, clearances: list }, wasm: manifoldUrl });
  } catch (err) {
    toast(`Testkörper nicht möglich: ${(err as Error).message}`);
    return;
  }
  const geo = (pos: Float32Array) => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.computeVertexNormals();
    g.computeBoundingBox();
    g.computeBoundingSphere();
    return g;
  };
  const gp = geo(r.plate);
  const plate = addPart(`Toleranztest Platte (${list.map(clearanceLabel).join(' / ')} mm)`, gp, { color: '#8fa7c9', layout: nextLayout(gp.boundingBox!) });
  const gz = geo(r.pin);
  const pin = addPart(`Toleranztest Zapfen ⌀ ${clearanceLabel(d)}`, gz, { color: '#d9a066', layout: nextLayout(gz.boundingBox!) });
  selected.clear();
  selected.add(plate.id);
  selected.add(pin.id);
  solveAll();
  renderAll();
  fitView();
  toast('Testkörper erzeugt. Drucken, dann den Zapfen der Reihe nach in die Löcher stecken: Das erste, in das er ohne Gewalt passt, ist dein Spiel – oben bei „Mein Spiel“ speichern.', 10000);
}

document.getElementById('tolTest')!.addEventListener('click', () => {
  setTool(null);
  tolBar.hidden = !tolBar.hidden;
});
document.getElementById('tolMake')!.addEventListener('click', () => void generateTolerance());
document.getElementById('tolClose')!.addEventListener('click', () => { tolBar.hidden = true; });
document.getElementById('tolSave')!.addEventListener('click', () => {
  const v = Number(tolMine.value.replace(',', '.'));
  if (!(v >= 0 && v < 2)) { toast('Bitte ein Spiel zwischen 0 und 2 mm eingeben.'); return; }
  try { localStorage.setItem(CLEAR_KEY, JSON.stringify({ ...savedClearances(), [clearKey()]: v })); } catch { /* ohne Speicher */ }
  applyMyClearance();
  toast(`Spiel ${clearanceLabel(v)} mm für ${clearanceFor()} gespeichert – Zerschneiden, Zerteilen und die Passung nehmen es jetzt vor.`, 6000);
});

// ---------------------------------------------------------------- Skalieren und Spiegeln

/**
 * Teile mit einer Ähnlichkeitsabbildung A (Weltkoordinaten) umformen: skalieren oder spiegeln. Die Lage
 * der Teile bleibt, ihre Netze werden in eigenen Koordinaten mit L = W⁻¹·A·W umgerechnet, ebenso die
 * Flächen und Punkte der Verbindungen und die Modifier. Gehören alle Teile einer Verbindung dazu, bleibt
 * ihr Ergebnis gültig (beide Seiten gleich umgeformt); sonst wird sie neu berechnet.
 * Beim Spiegeln gehen Bemalung und bearbeitbarer Text verloren (der Text bleibt als Form erhalten).
 */
function transformParts(list: Part[], A: THREE.Matrix4): { paint: number; text: number } {
  const maps = new Map<number, ReturnType<typeof similarity> & { m: THREE.Matrix4 }>();
  for (const p of list) {
    p.mesh.updateMatrixWorld();
    const m = p.mesh.matrixWorld.clone().invert().multiply(A).multiply(p.mesh.matrixWorld);
    maps.set(p.id, { ...similarity(m.elements), m });
  }
  const S = similarity(A.elements);
  for (const j of joints) {
    const sidePart = (side: Side) => (side === 'a' ? j.a : j.b);
    const involved = new Set([j.a, j.b].filter((x): x is number => x !== null));
    const conv = (f: PickedFeature, side: Side): PickedFeature => {
      if (f.on !== undefined) involved.add(f.on);
      const owner = f.on ?? sidePart(side);
      const T = owner !== null ? maps.get(owner) : undefined;
      const g = T ? similarityFeature(f, T) : f;
      return f.pieces ? { ...g, pieces: f.pieces.map((q) => conv(q, side)) } : g;
    };
    const fa = j.fa.map((f) => conv(f, 'a'));
    const fb = j.fb.map((f) => conv(f, 'b'));
    if (![...involved].some((id) => maps.has(id))) continue;
    const convM = (list: Marker[], side: Side) => {
      const T = maps.get(sidePart(side) ?? -1);
      return T ? list.map((m) => ({ p: T.point(m.p), n: T.dir(m.n) })) : list;
    };
    j.fa = fa;
    j.fb = fb;
    j.ma = convM(j.ma, 'a');
    j.mb = convM(j.mb, 'b');
    j.proposals = undefined;
    const Tf = j.fixedSide ? maps.get(sidePart(j.fixedSide) ?? -1) : undefined;
    if ([...involved].every((id) => maps.has(id)) && j.cache && Tf) {
      // Ergebnis im Koordinatensystem des festen Teils: Lage bleibt, Bewegungsachsen und Fehler mitrechnen
      const c = j.cache;
      j.cache = {
        ...c,
        rekey: true,
        distError: c.distError * S.s,
        free: c.free.map((m) => (m.kind === 'rotate' ? { kind: 'rotate', point: Tf.point(m.point), axis: Tf.dir(m.axis) } : { kind: 'translate', axis: Tf.dir(m.axis) })),
        turn: c.turn && { point: Tf.point(c.turn.point), axis: Tf.dir(c.turn.axis), angles: Tf.mirror ? c.turn.angles.map((a) => -a) : c.turn.angles },
      };
    } else if (![...involved].every((id) => maps.has(id))) markersChanged(j);
  }
  const lost = { paint: 0, text: 0 };
  for (const p of list) {
    const T = maps.get(p.id)!;
    const e = T.m.elements;
    const modifiers = p.modifiers?.map((m) => {
      const positions = transformMesh(m.positions, e);
      if (!m.textInfo || !m.transform) return { ...m, positions };
      if (T.mirror) {
        lost.text++;
        const { textInfo: _t, local: _l, transform: _m, ...rest } = m;
        return { ...rest, positions };
      }
      // Text: das eigene Netz bleibt, seine Lage wird mit umgerechnet (Bambu Studio skaliert ihn so)
      return { ...m, positions, transform: T.m.clone().multiply(new THREE.Matrix4().fromArray(m.transform)).toArray() };
    });
    if (T.mirror && p.paint) lost.paint++;
    replaceGeometry(p, transformMesh(positionsOf(p), e), { paint: T.mirror ? undefined : p.paint, modifiers });
  }
  solveAll();
  renderAll();
  return lost;
}

const scaleBar = document.getElementById('scaleBar')!;
const scalePct = document.getElementById('scalePct') as HTMLInputElement;
const scaleDims = ['scaleX', 'scaleY', 'scaleZ'].map((id) => document.getElementById(id) as HTMLInputElement);
/** Was skaliert wird: ein Teil (allein oder mit seiner Baugruppe) oder die Auswahl. */
let scaleTarget: { partId: number; which: 'unit' | 'part' } | { sel: number[] } | null = null;

function scaleMembers(): Part[] {
  if (!scaleTarget) return [];
  if ('sel' in scaleTarget) return scaleTarget.sel.map((id) => getPart(id)).filter((p): p is Part => !!p);
  const p = getPart(scaleTarget.partId);
  if (!p) return [];
  return scaleTarget.which === 'unit' ? unitMembers(p) : [p];
}

function openScale(target: NonNullable<typeof scaleTarget>) {
  if (!('sel' in target) && !getPart(target.partId)) return;
  setTool(null);
  closeSimplify();
  scaleTarget = target;
  scaleBar.hidden = false;
  renderScaleBar();
  scalePct.focus();
  scalePct.select();
}

function closeScale() {
  scaleTarget = null;
  scaleBar.hidden = true;
}

const fmtNum = (x: number) => String(+x.toFixed(2));

function renderScaleBar(keepPct = false) {
  const list = scaleMembers();
  if (!scaleTarget || !list.length) { closeScale(); return; }
  const first = list[0];
  const unit = 'sel' in scaleTarget ? [] : unitMembers(first);
  document.getElementById('scaleTitle')!.textContent = 'sel' in scaleTarget
    ? `${list.length} Teile skalieren / spiegeln`
    : list.length > 1 ? `${unitName(list)} skalieren / spiegeln` : `„${first.name}“ skalieren / spiegeln`;
  const seg = document.getElementById('scaleWhich')!;
  seg.hidden = 'sel' in scaleTarget || unit.length < 2;
  seg.querySelectorAll<HTMLButtonElement>('[data-which]').forEach((b) => b.classList.toggle('on', !('sel' in scaleTarget!) && b.dataset.which === scaleTarget!.which));
  if (!keepPct) scalePct.value = '100';
  updateScaleDims();
  const paint = list.some((p) => p.paint);
  const text = list.some((p) => p.modifiers?.some((m) => m.textInfo));
  document.getElementById('scaleHelp')!.textContent = [
    'Skaliert gleichmäßig um die Mitte; Verbindungen und Bemalung bleiben.',
    paint || text ? `Spiegeln entfernt ${[paint ? 'die Bemalung' : '', text ? 'die Bearbeitbarkeit des Textes (die Form bleibt)' : ''].filter(Boolean).join(' und ')}.` : '',
  ].filter(Boolean).join(' ');
}

/** Maße nach dem eingegebenen Faktor. */
function updateScaleDims(except?: HTMLInputElement) {
  const k = (Number(scalePct.value) || 100) / 100;
  const size = sizeOf(scaleMembers());
  [size.x, size.y, size.z].forEach((v, i) => { if (scaleDims[i] !== except) scaleDims[i].value = fmtNum(v * k); });
}

scalePct.addEventListener('input', () => updateScaleDims());
scaleDims.forEach((input, i) => input.addEventListener('input', () => {
  const size = sizeOf(scaleMembers());
  const cur = [size.x, size.y, size.z][i];
  const want = Number(input.value);
  if (!(want > 0) || !(cur > 0)) return;
  scalePct.value = fmtNum((want / cur) * 100);
  updateScaleDims(input);
}));
for (const el of [scalePct, ...scaleDims]) {
  el.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') { e.preventDefault(); applyScale(); }
    else if (e.key === 'Escape') { e.preventDefault(); closeScale(); }
  });
}
document.getElementById('scaleQuick')!.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('[data-k]');
  if (!b) return;
  scalePct.value = fmtNum(Number(b.dataset.k) * 100);
  updateScaleDims();
});
document.getElementById('scaleWhich')!.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('[data-which]');
  if (!b || !scaleTarget || 'sel' in scaleTarget) return;
  scaleTarget.which = b.dataset.which as 'unit' | 'part';
  renderScaleBar(true);
});

/** Abbildung um die Mitte der Teile: gleichmäßig skalieren (k) bzw. an einer Achse spiegeln. */
function aroundCenter(list: Part[], S: THREE.Matrix4): THREE.Matrix4 {
  const c = unitBox(list).getCenter(new THREE.Vector3());
  return new THREE.Matrix4().makeTranslation(c.x, c.y, c.z).multiply(S).multiply(new THREE.Matrix4().makeTranslation(-c.x, -c.y, -c.z));
}

function applyScale() {
  const list = scaleMembers();
  const k = Number(scalePct.value) / 100;
  if (!list.length) return;
  if (!(k > 0) || !Number.isFinite(k)) { toast('Bitte eine Größe über 0 % eingeben.'); return; }
  if (Math.abs(k - 1) < 1e-9) return;
  beginAction();
  transformParts(list, aroundCenter(list, new THREE.Matrix4().makeScale(k, k, k)));
  const s = sizeOf(scaleMembers());
  toast(`Auf ${fmtNum(k * 100).replace('.', ',')} % skaliert: ${mmText1(s.x)} × ${mmText1(s.y)} × ${mmText1(s.z)} mm.`);
  renderScaleBar();
}

function mirrorAxis(axis: number) {
  const list = scaleMembers();
  if (!list.length) return;
  beginAction();
  const d = [1, 1, 1];
  d[axis] = -1;
  const lost = transformParts(list, aroundCenter(list, new THREE.Matrix4().makeScale(d[0], d[1], d[2])));
  const notes = [lost.paint ? 'Bemalung entfernt' : '', lost.text ? 'Text ist jetzt eine feste Form' : ''].filter(Boolean);
  toast(`An ${'XYZ'[axis]} gespiegelt${notes.length ? ` – ${notes.join(', ')}` : ''}.`);
  renderScaleBar(true);
}

document.getElementById('scaleApply')!.addEventListener('click', applyScale);
document.getElementById('scaleClose')!.addEventListener('click', closeScale);
scaleBar.querySelectorAll<HTMLButtonElement>('[data-mirror]').forEach((b) => b.addEventListener('click', () => mirrorAxis(Number(b.dataset.mirror))));

// ---------------------------------------------------------------- Mehrfachauswahl und Gruppen

/**
 * Mehrere Teile auswählen – in der Teileliste mit Strg/⌘+Klick (einzeln) und Umschalt+Klick (Bereich),
 * im 3D-Fenster mit Umschalt+Klick, Strg/⌘+A wählt alle sichtbaren. Für die Auswahl: ausblenden,
 * Farbe, gruppieren (bewegen sich danach gemeinsam), Gruppe auflösen, entfernen.
 */
const selectedParts = () => parts.filter((p) => selected.has(p.id));

function toggleSelected(id: number) {
  if (selected.has(id)) selected.delete(id);
  else selected.add(id);
  selAnchor = id;
  renderParts();
}

/** Umschalt+Klick in der Liste: alle Zeilen zwischen der zuletzt angeklickten und dieser. */
function selectRange(id: number) {
  const order = [...partsEl.querySelectorAll<HTMLElement>('li[data-part]')].map((li) => Number(li.dataset.part));
  const a = order.indexOf(selAnchor ?? id);
  const b = order.indexOf(id);
  if (a < 0 || b < 0) { toggleSelected(id); return; }
  for (const x of order.slice(Math.min(a, b), Math.max(a, b) + 1)) selected.add(x);
  renderParts();
}

function clearSelection() {
  if (!selected.size) return;
  selected.clear();
  renderParts();
}

/** Leiste über der Liste und Rahmen im 3D-Fenster. */
function renderSelection() {
  const list = selectedParts();
  selBar.hidden = list.length < 2;
  if (list.length >= 2) {
    document.getElementById('selCount')!.textContent = `${list.length} ausgewählt`;
    const grouped = list.some((p) => p.group && groupOf(p).length > 1);
    (selBar.querySelector('[data-sel="ungroup"]') as HTMLElement).hidden = !grouped;
    (selBar.querySelector('[data-sel="group"]') as HTMLButtonElement).disabled = sameGroup(list);
    renderBoolSelect(list);
  }
  drawSelection();
  // Maße der Auswahl (sonst aller Baugruppen)
  if (dimsOn) renderDims();
}

/** Bilden die Teile schon genau eine Gruppe? */
const sameGroup = (list: Part[]) => !!list[0]?.group && list.every((p) => p.group === list[0].group) && groupOf(list[0]).length === list.length;

function drawSelection() {
  for (const o of [...selGroup.children]) {
    selGroup.remove(o);
    (o as THREE.LineSegments).geometry.dispose();
    ((o as THREE.LineSegments).material as THREE.Material).dispose();
  }
  const accent = accentColor;
  for (const p of selectedParts()) {
    if (!p.visible) continue;
    const helper = new THREE.Box3Helper(partBox(p).expandByScalar(0.3), new THREE.Color(accent));
    const m = helper.material as THREE.LineBasicMaterial;
    m.depthTest = false;
    m.transparent = true;
    m.opacity = 0.85;
    helper.renderOrder = 6;
    helper.raycast = () => {};
    selGroup.add(helper);
  }
}

/** Ausgewählte Teile zu einer Gruppe verbinden: Sie bleiben, wo sie sind, und bewegen sich künftig gemeinsam. */
function groupParts(list: Part[]) {
  if (list.length < 2) return;
  const taken = new Set(parts.map((p) => p.group));
  let n = 1;
  while (taken.has(`Gruppe ${n}`)) n++;
  const name = `Gruppe ${n}`;
  for (const p of list) {
    p.layout.copy(p.mesh.matrixWorld);
    p.group = name;
  }
  solveAll();
  renderAll();
  toast(`${list.length} Teile bilden jetzt „${name}" und bewegen sich gemeinsam. Lösen: Rechtsklick › „Gruppe auflösen".`);
}

/** Gruppen der Teile auflösen (alle Mitglieder). */
function ungroupAll(list: Part[]) {
  const members = new Set(list.flatMap((p) => (p.group ? groupOf(p) : [])));
  for (const p of members) {
    p.layout.copy(p.mesh.matrixWorld);
    p.group = undefined;
  }
  solveAll();
  renderAll();
  if (members.size) toast(`${members.size} Teile bewegen sich jetzt einzeln.`);
}

function removeParts(list: Part[]) {
  for (const p of list) removePart(p.id);
  selected.clear();
  renderAll();
}

// Strg/⌘+Klick und Umschalt+Klick in der Teileliste (nicht auf Knöpfen und Feldern)
partsEl.addEventListener('click', (e) => {
  const li = (e.target as HTMLElement).closest<HTMLElement>('li[data-part]');
  if (!li || (e.target as HTMLElement).closest('input, button, label')) return;
  const id = Number(li.dataset.part);
  if (e.ctrlKey || e.metaKey) toggleSelected(id);
  else if (e.shiftKey) selectRange(id);
  else {
    // Einfacher Klick: Auswahl aufheben, der Name lässt das Teil wie bisher aufleuchten
    selAnchor = id;
    clearSelection();
    return;
  }
  e.stopPropagation();
});
// Umschalt+Klick soll keinen Text markieren
partsEl.addEventListener('mousedown', (e) => { if (e.shiftKey) e.preventDefault(); });

selBar.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('[data-sel]');
  const list = selectedParts();
  if (!b || !list.length) return;
  switch (b.dataset.sel) {
    case 'group': groupParts(list); break;
    case 'ungroup': ungroupAll(list); break;
    case 'hide': setVisible(list, false); break;
    case 'color': {
      const r = b.getBoundingClientRect();
      pickColor(list, r.left, r.bottom);
      break;
    }
    case 'remove': removeParts(list); break;
    case 'clear': clearSelection(); break;
  }
});

// ---------------------------------------------------------------- Kontextmenü

/**
 * Rechtsklick auf ein Teil (im 3D-Fenster oder in der Teileliste): die Befehle für dieses Teil an einer
 * Stelle. Im 3D-Fenster öffnet nur ein Rechtsklick ohne Ziehen das Menü – rechts ziehen verschiebt
 * weiterhin die Ansicht (auf dem Mac käme das Browser-Ereignis schon beim Drücken).
 */
type MenuEntry = { label: string; run: () => void; disabled?: boolean; danger?: boolean; title?: string } | 'sep';
const ctxMenu = document.getElementById('ctxMenu')!;
const ctxColor = document.getElementById('ctxColor') as HTMLInputElement;
let ctxEntries: MenuEntry[] = [];

function openMenu(x: number, y: number, head: string, entries: MenuEntry[]) {
  hideTip();
  // Trennlinien nur zwischen Einträgen
  const list = entries.filter((e, i) => e !== 'sep' || (i > 0 && entries[i - 1] !== 'sep' && i < entries.length - 1));
  ctxEntries = list;
  ctxMenu.innerHTML = head + list.map((e, i) => e === 'sep'
    ? '<hr />'
    : `<button role="menuitem" data-i="${i}"${e.disabled ? ' disabled' : ''}${e.danger ? ' class="danger"' : ''}${e.title ? ` title="${esc(e.title)}"` : ''}>${esc(e.label)}</button>`).join('');
  ctxMenu.hidden = false;
  const r = ctxMenu.getBoundingClientRect();
  ctxMenu.style.left = `${Math.max(4, Math.min(x, innerWidth - r.width - 4))}px`;
  ctxMenu.style.top = `${Math.max(4, Math.min(y, innerHeight - r.height - 4))}px`;
  ctxMenu.querySelector<HTMLButtonElement>('button:not(:disabled)')?.focus({ preventScroll: true });
}

function closeMenu() {
  if (ctxMenu.hidden) return;
  ctxMenu.hidden = true;
  ctxEntries = [];
}

ctxMenu.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLButtonElement>('button[data-i]');
  const entry = b && ctxEntries[Number(b.dataset.i)];
  if (!entry || entry === 'sep' || entry.disabled) return;
  closeMenu();
  beginAction();
  entry.run();
});
ctxMenu.addEventListener('contextmenu', (e) => e.preventDefault());
// Klick daneben, Scrollen, Fenstergröße: Menü schließen
window.addEventListener('pointerdown', (e) => { if (!ctxMenu.contains(e.target as Node)) closeMenu(); }, { capture: true });
window.addEventListener('wheel', closeMenu, { capture: true, passive: true });
window.addEventListener('resize', closeMenu);
window.addEventListener('blur', closeMenu);
// Tasten: Esc schließt, Pfeile wählen, Enter/Leertaste lösen aus – alle anderen Kürzel ruhen
window.addEventListener('keydown', (e) => {
  if (ctxMenu.hidden) return;
  if (e.key === 'Enter' || e.key === ' ') return;
  e.preventDefault();
  e.stopImmediatePropagation();
  if (e.key === 'Escape' || e.key === 'Tab') closeMenu();
  else if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
    const items = [...ctxMenu.querySelectorAll<HTMLButtonElement>('button:not(:disabled)')];
    const i = items.indexOf(document.activeElement as HTMLButtonElement);
    items[(i + (e.key === 'ArrowDown' ? 1 : items.length - 1)) % items.length]?.focus();
  }
}, { capture: true });

/** Farbe von Teilen über die Farbauswahl des Systems ändern (live). */
function pickColor(list: Part[], x: number, y: number) {
  ctxColor.value = `#${list[0].color.getHexString()}`;
  ctxColor.style.left = `${x}px`;
  ctxColor.style.top = `${y}px`;
  ctxColor.oninput = () => {
    for (const p of list) {
      p.color.set(ctxColor.value);
      (p.mesh.material as THREE.MeshStandardMaterial).color.copy(p.color);
    }
  };
  ctxColor.onchange = () => renderAll();
  try {
    ctxColor.showPicker();
  } catch {
    ctxColor.click();
  }
}

function setVisible(list: Part[], on: boolean) {
  for (const q of list) { q.visible = on; q.mesh.visible = on; }
  renderAll();
}

function partMenu(p: Part, h: Hit | null, x: number, y: number, marker?: ReturnType<typeof markerAt>) {
  // Rechtsklick auf ein ausgewähltes Teil: Befehle für die ganze Auswahl
  if (selected.has(p.id) && selected.size > 1 && !marker) { selectionMenu(x, y); return; }
  const members = unitMembers(p);
  const s = sizeOf([p]);
  const others = parts.filter((q) => q !== p && q.visible);
  const outsideUnit = parts.filter((q) => q.visible && !members.includes(q));
  const assembly = assemblyOf(p);
  const seed = getPart(seedOf.get(p.id) ?? p.id) ?? p;
  const head = `<div class="head"><b>${esc(p.name)}</b><small>${mmText1(s.x)} × ${mmText1(s.y)} × ${mmText1(s.z)} mm${members.length > 1 ? ` · Baugruppe mit ${members.length} Teilen` : ''}</small></div>`;
  const entries: MenuEntry[] = [];
  if (marker) {
    entries.push({ label: marker.kind === 'face' ? 'Diese Fläche abwählen' : 'Diesen Punkt entfernen', run: () => removeMarker(marker) }, 'sep');
  }
  entries.push(
    { label: 'Ausblenden', run: () => setVisible([p], false), disabled: !p.visible },
    others.length
      ? { label: 'Nur dieses Teil zeigen', run: () => setVisible(others, false) }
      : { label: 'Alle Teile zeigen', run: () => setVisible(parts, true), disabled: parts.every((q) => q.visible) },
  );
  if (members.length > 1 && outsideUnit.length) entries.push({ label: 'Nur diese Baugruppe zeigen', run: () => setVisible(outsideUnit, false) });
  entries.push(
    'sep',
    { label: 'Farbe ändern …', run: () => pickColor([p], x, y) },
    { label: 'Umbenennen …', run: () => renameInList(p) },
    { label: p.id === baseId ? 'Ist das Basisteil' : 'Als Basisteil festlegen', run: () => setBase(p.id), disabled: p.id === baseId, title: 'Das Basisteil bleibt liegen, alle anderen werden daran angefügt.' },
    { label: 'In der Teileliste zeigen', run: () => revealPart(p) },
    'sep',
  );
  if (h) {
    entries.push(
      { label: 'Mit dieser Fläche auf die Platte legen', run: () => layOnFace(h), title: 'Dreht die Baugruppe so, dass die angeklickte ebene Fläche unten liegt.' },
      { label: 'Passung an dieser Stelle …', run: () => { setTool('fit'); pickFit(h); }, title: 'Zapfen dünner, Bohrung weiter oder Fläche versetzen.' },
      { label: 'Aussparung hier …', run: () => { setTool('pocket'); pickPocket(h); }, title: 'Magnet, Gewindeeinsatz, Schraube, Mutter oder Bohrung an dieser Stelle (Taste E)' },
      { label: 'Hier beschriften …', run: () => { setTextSource(textSrc === 'number' ? 'text' : textSrc); setTool('text'); pickText(h); }, title: 'Text oder Logo erhaben oder vertieft an dieser Stelle (Taste W)' },
    );
  }
  entries.push(
    { label: 'Skalieren / Spiegeln …', run: () => openScale({ partId: p.id, which: members.length > 1 ? 'unit' : 'part' }), title: 'Größe in % oder mm, Zoll → mm, an X/Y/Z spiegeln (Taste S)' },
    { label: section && !cutTarget ? 'Hier zerschneiden …' : 'Zerschneiden …', run: () => openCut(p), title: 'In zwei Teile schneiden, auf Wunsch mit Zapfen oder Dübeln zum Zusammenstecken' },
    ...(fitsBed(partBox(p).getSize(new THREE.Vector3())) ? [] : [{ label: 'Auf Druckplattengröße zerteilen …', run: () => openSplit(p), title: `Zu groß für den ${printerName(currentPrinter())}: in Stücke teilen, die auf die Platte passen` }]),
    { label: p.health && !p.health.closed ? 'Netz reparieren ⚠' : 'Netz prüfen und reparieren', run: () => void repairParts([p]), title: p.health ? `Netz: ${healthText(p.health)}` : 'Löcher schließen, verdrehte Flächen umdrehen, Duplikate entfernen' },
    { label: 'Netz vereinfachen …', run: () => openSimplify([p]), title: `${countText(triCount(p))} Dreiecke – weniger machen D3DForge und den Slicer schneller; die Form weicht höchstens um die gewählte Abweichung ab` },
    { label: gizmoPart()?.id === p.id && !gizmoTarget?.assembly ? 'Bewegen beenden' : 'Von Hand bewegen/drehen', run: () => movePartByHand(p.id) },
  );
  if (assembly.length > 1) entries.push({ label: assembly.some((q) => q.id === baseId) ? 'Gesamtes Modell bewegen' : 'Baugruppe bewegen', run: () => { openGizmo({ kind: 'part', id: seed.id, assembly: true }, 'translate'); renderAll(); } });
  if (p.group && groupOf(p).length > 1) {
    entries.push(
      { label: 'Aus der Gruppe lösen', run: () => ungroupPart(p), title: `Gehört zu „${p.group}" (${groupOf(p).length} Teile, bewegen sich gemeinsam).` },
      { label: 'Gruppe auflösen', run: () => ungroupAll([p]), title: 'Alle Teile der Gruppe bewegen sich danach einzeln.' },
    );
  }
  entries.push(
    'sep',
    { label: selected.has(p.id) ? 'Aus der Auswahl nehmen' : 'Zur Auswahl hinzufügen', run: () => toggleSelected(p.id), title: 'Mehrere Teile auswählen: Umschalt+Klick im 3D-Fenster, Strg/⌘+Klick in der Liste.' },
    { label: 'Entfernen', run: () => removePart(p.id), danger: true },
  );
  openMenu(x, y, head, entries);
}

function selectionMenu(x: number, y: number) {
  const list = selectedParts();
  const others = parts.filter((q) => q.visible && !selected.has(q.id));
  const grouped = list.some((q) => q.group && groupOf(q).length > 1);
  const s = sizeOf(list);
  openMenu(x, y, `<div class="head"><b>${list.length} Teile ausgewählt</b><small>zusammen ${mmText1(s.x)} × ${mmText1(s.y)} × ${mmText1(s.z)} mm</small></div>`, [
    { label: 'Ausblenden', run: () => setVisible(list, false) },
    { label: 'Nur diese zeigen', run: () => setVisible(others, false), disabled: !others.length },
    { label: 'Farbe für alle …', run: () => pickColor(list, x, y) },
    { label: 'Skalieren / Spiegeln …', run: () => openScale({ sel: list.map((q) => q.id) }) },
    { label: 'Netze reparieren', run: () => void repairParts(list) },
    { label: 'Netze vereinfachen …', run: () => openSimplify(list) },
    { label: 'Nummerieren …', run: () => { setTextSource('number'); setTool('text'); }, title: '1/7, 2/7 … auf die ausgewählten Teile prägen – je auf die größte Fläche unten oder oben (Taste W)' },
    'sep',
    ...boolEntries(list),
    'sep',
    { label: 'Gruppieren', run: () => groupParts(list), disabled: sameGroup(list), title: 'Die Teile bleiben, wo sie sind, und bewegen sich künftig gemeinsam.' },
    ...(grouped ? [{ label: 'Gruppe auflösen', run: () => ungroupAll(list) }] : []),
    'sep',
    { label: 'Auswahl aufheben', run: clearSelection },
    { label: `${list.length} Teile entfernen`, run: () => removeParts(list), danger: true },
  ]);
}

/** Rechtsklick ins Leere. */
function viewMenu(x: number, y: number) {
  const hidden = parts.filter((q) => !q.visible);
  openMenu(x, y, '', [
    { label: 'Ansicht anpassen', run: fitView, disabled: !parts.length },
    { label: 'Schrägansicht', run: () => viewFrom(VIEW_KEYS[0]) },
    { label: `Alle Teile zeigen${hidden.length ? ` (${hidden.length} ausgeblendet)` : ''}`, run: () => setVisible(parts, true), disabled: !hidden.length },
    'sep',
    { label: 'Auf der Platte anordnen', run: () => void arrangeOnBed(), disabled: !parts.length },
    { label: platesShown ? 'Druckplatten ausblenden' : 'Druckplatten einblenden', run: () => setPlatesShown(!platesShown), title: 'Umriss, Raster und Beschriftung der Platten (Knopf „Platten“ unter dem Würfel)' },
    { label: 'Platte hinzufügen', run: addPlate, title: 'Weitere Druckplatte wie im Slicer – das Raster ordnet sich neu, die Teile wandern mit ihrer Platte' },
    { label: 'Leere Platten entfernen', run: removeEmptyPlates, disabled: plateTotal === 1 || occupiedPlates().size >= plateTotal },
    ...(plateTotal > 1 ? [{ label: 'Alle Platten ansehen', run: () => viewPlate(-1) }] : []),
    'sep',
    { label: 'Hilfe und Tastenkürzel', run: openHelp },
  ]);
}

let rightDown: { x: number; y: number } | null = null;
renderer.domElement.addEventListener('pointerdown', (e) => {
  rightDown = e.button === 2 ? { x: e.clientX, y: e.clientY } : null;
}, { capture: true });
renderer.domElement.addEventListener('pointerup', (e) => {
  if (e.button !== 2 || !rightDown) return;
  const moved = Math.hypot(e.clientX - rightDown.x, e.clientY - rightDown.y);
  rightDown = null;
  if (moved > 5) return; // war ein Verschieben der Ansicht
  contextAt(e);
});

/** Was ein Rechtsklick an dieser Stelle öffnet: Menü des Teils oder der Ansicht (Punkte werden entfernt). */
function contextAt(e: PointerEvent) {
  const marker = markerAt(e);
  // Punkte sind klein: direkt entfernen (wie bisher); gewählte Flächen bedecken oft das ganze Teil – Menü
  if (marker?.kind === 'point') {
    beginAction();
    removeMarker(marker);
    return;
  }
  const h = pick(e);
  if (h) partMenu(h.part, h, e.clientX, e.clientY, marker);
  else viewMenu(e.clientX, e.clientY);
}

/**
 * Touch: lange drücken (ohne zu ziehen) wirkt wie ein Rechtsklick – Tablets haben keinen. Werkzeuge, die den
 * Finger selbst nehmen (Malen, Pinsel), lassen es nicht bis hierher kommen.
 */
const LONG_PRESS_MS = 550;
function onLongPress(el: HTMLElement, start: (e: PointerEvent) => boolean, fire: (e: PointerEvent) => void) {
  let press: { id: number; x: number; y: number; timer: number } | null = null;
  const cancel = () => {
    if (press) clearTimeout(press.timer);
    press = null;
  };
  el.addEventListener('pointerdown', (e) => {
    cancel();
    if (e.pointerType === 'mouse' || e.button !== 0 || touches.size > 1 || !start(e)) return;
    press = { id: e.pointerId, x: e.clientX, y: e.clientY, timer: window.setTimeout(() => { press = null; fire(e); }, LONG_PRESS_MS) };
  });
  el.addEventListener('pointermove', (e) => {
    if (press && e.pointerId === press.id && Math.hypot(e.clientX - press.x, e.clientY - press.y) > 10) cancel();
  }, { capture: true });
  for (const type of ['pointerup', 'pointercancel', 'pointerleave'] as const) el.addEventListener(type, (e) => { if (press && e.pointerId === press.id) cancel(); }, { capture: true });
  el.addEventListener('pointerdown', (e) => { if (press && e.pointerId !== press.id) cancel(); }, { capture: true });
}
// Nicht, wenn der Finger einen Griff des Gizmos hält
onLongPress(renderer.domElement, () => !multiTouch && !(tc.object && tc.axis), (e) => {
  if (touches.size > 1) return;
  suppressClick = true;
  contextAt(e);
});
let listLongPressed = false;
onLongPress(partsEl, (e) => !!(e.target as HTMLElement).closest('li[data-part]') && !(e.target as HTMLElement).closest('input, button'), (e) => {
  const li = (e.target as HTMLElement).closest<HTMLElement>('li[data-part]');
  const p = li && getPart(Number(li.dataset.part));
  if (!p) return;
  listLongPressed = true;
  partMenu(p, null, e.clientX, e.clientY);
});
// Der Klick nach dem langen Drücken wählt die Zeile nicht aus
partsEl.addEventListener('click', (e) => {
  if (!listLongPressed) return;
  listLongPressed = false;
  e.preventDefault();
  e.stopImmediatePropagation();
}, { capture: true });
partsEl.addEventListener('contextmenu', (e) => {
  const li = (e.target as HTMLElement).closest<HTMLElement>('li[data-part]');
  const p = li && getPart(Number(li.dataset.part));
  if (!p || (e.target as HTMLElement).closest('input.rename')) return;
  e.preventDefault();
  partMenu(p, null, e.clientX, e.clientY);
});

// ---------------------------------------------------------------- Statuszeile

/** Teil unter dem Mauszeiger (für die Abmessungen in der Statuszeile). */
let statusHover: number | null = null;
const statusEl = document.getElementById('status')!;
const mmText1 = (x: number) => x.toFixed(1).replace('.', ',');

/** Abmessungen (Breite × Tiefe × Höhe) mehrerer Teile in Weltkoordinaten. */
function sizeOf(members: Part[]): THREE.Vector3 {
  return unitBox(members).getSize(new THREE.Vector3());
}

function renderStatus() {
  if (!parts.length) { statusEl.innerHTML = ''; return; }
  const shown = parts.filter((p) => p.visible);
  const tris = shown.reduce((n, p) => n + positionsOf(p).length / 9, 0);
  const connected = joints.filter((j) => j.applied && j.status?.kind !== 'err').length;
  const bits = [
    `<b>${shown.length}</b>${shown.length === parts.length ? '' : ` von ${parts.length}`} Teil${parts.length === 1 ? '' : 'e'}`,
    `${tris.toLocaleString('de-DE')} Dreiecke`,
    `${connected} Verbindung${connected === 1 ? '' : 'en'}`,
  ];
  const p = getPart(statusHover);
  if (p) {
    const members = unitMembers(p);
    const s = sizeOf(members);
    bits.push(`${members.length > 1 ? `Baugruppe „${esc(p.name)}“ (${members.length} Teile)` : `„${esc(p.name)}“`}: <b>${mmText1(s.x)} × ${mmText1(s.y)} × ${mmText1(s.z)} mm</b>, ≈ ${gramText(estimateOf(members).grams)} g ${esc(material().label)}`);
  }
  const open = parts.filter((q) => q.health && !q.health.closed);
  if (open.length) bits.push(`<span class="warn">⚠ ${open.length > 1 ? `${open.length} Netze sind` : `„${esc(open[0].name)}“ ist`} nicht geschlossen – Rechtsklick › „Netz reparieren“</span>`);
  statusEl.innerHTML = bits.join(' · ') + bedWarning() + nozzleNote();
}

// ---------------------------------------------------------------- Filament und Kosten

const matKind = document.getElementById('matKind') as HTMLSelectElement;
const matInfill = document.getElementById('matInfill') as HTMLInputElement;
const matWalls = document.getElementById('matWalls') as HTMLInputElement;
const matPrice = document.getElementById('matPrice') as HTMLInputElement;
const matResult = document.getElementById('matResult')!;
const MAT_KEY = 'partjoin.material';
/** Vorgaben für die Schätzung (Bambu Studio: 2 Wände, 15 % Füllung); Preise je Material getrennt. */
const matPrefs: { kind: string; infill: number; walls: number; prices: Record<string, number> } = { kind: 'pla', infill: 15, walls: 2, prices: {} };
try {
  Object.assign(matPrefs, JSON.parse(localStorage.getItem(MAT_KEY) ?? '{}'));
} catch {
  // ohne gespeicherte Vorgaben
}
if (!MATERIALS.some((m) => m.id === matPrefs.kind)) matPrefs.kind = 'pla';
matKind.innerHTML = MATERIALS.map((m) => `<option value="${m.id}">${esc(m.label)}</option>`).join('');
const material = (): Material => MATERIALS.find((m) => m.id === matPrefs.kind) ?? MATERIALS[0];
const priceOf = (m: Material) => matPrefs.prices[m.id] ?? m.price;

function showMatPrefs() {
  matKind.value = matPrefs.kind;
  matInfill.value = String(matPrefs.infill);
  matWalls.value = String(matPrefs.walls);
  matPrice.value = String(priceOf(material()));
}
showMatPrefs();

function matChanged() {
  const num = (el: HTMLInputElement, min: number, max: number, fallback: number) => {
    const v = Number(el.value);
    return el.value.trim() !== '' && Number.isFinite(v) ? Math.min(max, Math.max(min, v)) : fallback;
  };
  matPrefs.infill = num(matInfill, 0, 100, matPrefs.infill);
  matPrefs.walls = Math.round(num(matWalls, 1, 10, matPrefs.walls));
  matPrefs.prices[matPrefs.kind] = num(matPrice, 0, 10000, priceOf(material()));
  try { localStorage.setItem(MAT_KEY, JSON.stringify(matPrefs)); } catch { /* nicht speicherbar */ }
  renderMaterial();
  renderStatus();
}
matKind.addEventListener('change', () => {
  matPrefs.kind = matKind.value;
  showMatPrefs();
  matChanged();
  // Spiel des neuen Materials
  applyMyClearance();
});
for (const el of [matInfill, matWalls, matPrice]) {
  el.addEventListener('input', matChanged);
  // Ungültiges beim Verlassen durch den gültigen Wert ersetzen
  el.addEventListener('change', showMatPrefs);
}

/** Volumen, Oberfläche und Schwerpunkt je Netz (einmal gerechnet). */
const massCache = new WeakMap<THREE.BufferGeometry, { m: MassProps; dims: Vec3 }>();
function massOf(p: Part): { m: MassProps; dims: Vec3 } {
  let c = massCache.get(p.geom);
  if (!c) {
    if (!p.geom.boundingBox) p.geom.computeBoundingBox();
    const size = p.geom.boundingBox!.getSize(new THREE.Vector3());
    c = { m: massProps(positionsOf(p)), dims: [size.x, size.y, size.z] };
    massCache.set(p.geom, c);
  }
  return c;
}

const printSettings = (): PrintSettings => ({ density: material().density, infill: matPrefs.infill / 100, walls: matPrefs.walls, price: priceOf(material()) });

/** Wände und Füllung eines Teils: eigene Einstellungen aus dem Bambu-Projekt, sonst die Vorgaben. */
function settingsFor(p: Part, s: PrintSettings): PrintSettings {
  const own = { ...p.objectSettings, ...p.settings };
  const infill = parseFloat(own.sparse_infill_density ?? '');
  const walls = parseInt(own.wall_loops ?? '', 10);
  return { ...s, infill: Number.isFinite(infill) ? infill / 100 : s.infill, walls: walls > 0 ? walls : s.walls };
}

/** Summe über Teile (jedes Teil mit eigenen Wänden, wie im Slicer). */
function estimateOf(list: Part[]): PrintEstimate & { open: boolean } {
  const s = printSettings();
  const sum = { plastic: 0, shell: 0, inner: 0, grams: 0, meters: 0, cost: 0, open: false };
  for (const p of list) {
    const { m, dims } = massOf(p);
    const e = printEstimate(m.volume, m.area, settingsFor(p, s), dims);
    sum.plastic += e.plastic;
    sum.shell += e.shell;
    sum.inner += e.inner;
    sum.grams += e.grams;
    sum.meters += e.meters;
    sum.cost += e.cost;
    if (p.health && !p.health.closed) sum.open = true;
  }
  return sum;
}

const gramText = (g: number) => (g < 10 ? g.toFixed(1).replace('.', ',') : Math.round(g).toLocaleString('de-DE'));
const estimateText = (e: PrintEstimate) =>
  `≈ <b>${gramText(e.grams)} g</b> · ${e.meters.toFixed(1).replace('.', ',')} m · ${e.cost.toFixed(2).replace('.', ',')} €`;

function renderMaterial() {
  const units = visibleUnits();
  if (!units.length) { matResult.innerHTML = ''; return; }
  const all = estimateOf(units.flatMap((u) => u.members));
  const rows = [`<div>Sichtbare Teile: ${estimateText(all)}</div>`];
  // Je Platte, wenn mehrere belegt sind
  const byPlate = new Map<number, Part[]>();
  for (const u of units) {
    const i = plateOfBox(u.box).i;
    byPlate.set(i, [...(byPlate.get(i) ?? []), ...u.members]);
  }
  if (byPlate.size > 1) {
    for (const [i, list] of [...byPlate].sort((a, b) => a[0] - b[0])) rows.push(`<div class="plate">Platte ${i + 1}: ${estimateText(estimateOf(list))}</div>`);
  }
  const own = units.some((u) => u.members.some((p) => p.settings?.wall_loops || p.settings?.sparse_infill_density || p.objectSettings?.wall_loops || p.objectSettings?.sparse_infill_density));
  rows.push(`<div class="note">${all.open ? '⚠ Ein Netz ist nicht geschlossen – dort ungenau. ' : ''}${own ? 'Teile mit eigenen Wänden oder eigener Füllung aus dem Slicer-Projekt rechnen damit. ' : ''}Schätzung ohne Stützen, Brim und Spülturm – genau rechnet der Slicer.</div>`);
  matResult.innerHTML = rows.join('');
}

// ---------------------------------------------------------------- Standfestigkeit

let standOn = false;
const standGroup = new THREE.Group();
scene.add(standGroup);
const standLabels = document.createElement('div');
standLabels.id = 'standLabels';
view.appendChild(standLabels);
let standLabelAt: { el: HTMLElement; at: THREE.Vector3 }[] = [];
const standResult = document.getElementById('standResult')!;
const standBtn = document.getElementById('standBtn')!;
const STAND_COLORS = { ok: '#30a46c', tight: '#f5a524', tips: '#e5484d' };

/** Tiefste Punkte eines Teils (Welt): alles bis 2 mm über seinem tiefsten Punkt, je Lage einmal gesucht. */
const lowPoints = new WeakMap<THREE.BufferGeometry, { key: string; min: number; pts: Float32Array }>();
function lowPointsOf(p: Part): { min: number; pts: Float32Array } {
  const e = p.mesh.matrixWorld.elements;
  const key = Array.from(e, (x) => x.toFixed(5)).join(',');
  const known = lowPoints.get(p.geom);
  if (known?.key === key) return known;
  const pos = positionsOf(p);
  let min = Infinity;
  for (let i = 0; i < pos.length; i += 3) {
    const z = e[2] * pos[i] + e[6] * pos[i + 1] + e[10] * pos[i + 2] + e[14];
    if (z < min) min = z;
  }
  const out: number[] = [];
  for (let i = 0; i < pos.length; i += 3) {
    const z = e[2] * pos[i] + e[6] * pos[i + 1] + e[10] * pos[i + 2] + e[14];
    if (z > min + 2) continue;
    out.push(e[0] * pos[i] + e[4] * pos[i + 1] + e[8] * pos[i + 2] + e[12], e[1] * pos[i] + e[5] * pos[i + 1] + e[9] * pos[i + 2] + e[13], z);
  }
  const r = { key, min, pts: new Float32Array(out) };
  lowPoints.set(p.geom, r);
  return r;
}

/** Schwerpunkt (mit Wänden und Füllung wie bei „Filament & Kosten“), Standfläche und Kippwinkel je Baugruppe. */
function standOf(members: Part[]) {
  const s = printSettings();
  let mass = 0;
  const com = new THREE.Vector3();
  for (const p of members) {
    const { m, dims } = massOf(p);
    const e = printEstimate(m.volume, m.area, settingsFor(p, s), dims);
    com.addScaledVector(new THREE.Vector3(...printedCentroid(m, e)).applyMatrix4(p.mesh.matrixWorld), e.grams);
    mass += e.grams;
  }
  if (mass > 0) com.multiplyScalar(1 / mass);
  const lows = members.map(lowPointsOf);
  const ground = Math.min(...lows.map((l) => l.min));
  const box = unitBox(members);
  // Was den Boden berührt: bis 0,1 mm darüber (bei großen Teilen etwas mehr)
  const tol = 0.1 + 0.002 * (box.max.z - box.min.z);
  const touch: P2[] = [];
  for (const l of lows) for (let i = 0; i < l.pts.length; i += 3) if (l.pts[i + 2] <= ground + tol) touch.push([l.pts[i], l.pts[i + 1]]);
  const hull = convexHull(touch);
  return { com, ground, hull, mass, ...stabilityOf([com.x, com.y, com.z], hull, ground) };
}

function renderStand() {
  disposeGroup(standGroup);
  standLabels.replaceChildren();
  standLabelAt = [];
  if (!standOn) { standResult.innerHTML = ''; return; }
  const rows: string[] = [];
  for (const u of visibleUnits()) {
    const st = standOf(u.members);
    const deg = Math.round(st.tipDeg);
    const kind = !st.inside || st.tipDeg < 5 ? 'tips' : st.tipDeg < 15 ? 'tight' : 'ok';
    const color = STAND_COLORS[kind];
    const size = Math.max(...u.box.getSize(new THREE.Vector3()).toArray(), 1);
    const z = st.ground + 0.05;
    // Standfläche auf dem Boden, Schwerpunkt mit Lot
    if (st.hull.length >= 3) {
      const shape = new THREE.Shape(st.hull.map(([x, y]) => new THREE.Vector2(x, y)));
      // Durch das Teil hindurch sichtbar: sonst verdeckt es seine eigene Standfläche
      const fill = new THREE.Mesh(new THREE.ShapeGeometry(shape), new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.3, depthTest: false, depthWrite: false, side: THREE.DoubleSide }));
      fill.position.z = z;
      fill.renderOrder = 11;
      standGroup.add(fill);
    }
    if (st.hull.length >= 2) {
      const outline = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(st.hull.map(([x, y]) => new THREE.Vector3(x, y, z))), new THREE.LineBasicMaterial({ color, depthTest: false }));
      outline.renderOrder = 11;
      standGroup.add(outline);
    }
    const dot = new THREE.Mesh(new THREE.SphereGeometry(0.018 * size, 16, 12), new THREE.MeshBasicMaterial({ color, depthTest: false }));
    dot.position.copy(st.com);
    const foot = new THREE.Vector3(st.com.x, st.com.y, z);
    const drop = new THREE.Line(new THREE.BufferGeometry().setFromPoints([st.com, foot]), new THREE.LineDashedMaterial({ color, dashSize: 0.03 * size, gapSize: 0.02 * size, depthTest: false }));
    drop.computeLineDistances();
    const footDot = new THREE.Mesh(new THREE.CircleGeometry(0.014 * size, 20), new THREE.MeshBasicMaterial({ color, depthTest: false, side: THREE.DoubleSide }));
    footDot.position.copy(foot);
    for (const o of [dot, drop, footDot]) o.renderOrder = 12;
    standGroup.add(dot, drop, footDot);
    const label = document.createElement('span');
    label.className = `mlabel stand ${kind}`;
    label.textContent = !st.inside ? 'kippt um' : `kippt ab ${deg}°`;
    standLabels.appendChild(label);
    standLabelAt.push({ el: label, at: st.com.clone() });
    const name = `„${esc(u.name)}“`;
    if (!st.inside) {
      rows.push(`<div class="tips">✕ ${name} kippt um – der Schwerpunkt liegt ${mmText1(Math.max(0, -st.margin))} mm neben der Standfläche.</div>`);
    } else if (kind === 'tips') {
      rows.push(`<div class="tips">⚠ ${name} steht wackelig – kippt schon ab ${deg}° Neigung.</div>`);
    } else if (kind === 'tight') {
      rows.push(`<div class="tight">⚠ ${name} steht knapp – kippt ab ${deg}° Neigung.</div>`);
    } else rows.push(`<div class="ok">✓ ${name} steht sicher – kippt erst ab ${deg}° Neigung.</div>`);
  }
  rows.push('<div class="note">So, wie die Baugruppen gerade stehen. Schwerpunkt mit Wänden und Füllung wie bei „Filament &amp; Kosten“; Punkt und Lot zeigen ihn, die Fläche am Boden ist die Standfläche.</div>');
  standResult.innerHTML = rows.join('');
}

standBtn.addEventListener('click', () => {
  standOn = !standOn && parts.length > 0;
  if (!parts.length) toast('Lade zuerst ein Modell.');
  standBtn.classList.toggle('ghost', !standOn);
  standBtn.title = standOn ? 'Standfestigkeit ausblenden' : 'Steht das Modell sicher? Zeigt Schwerpunkt und Standfläche jeder Baugruppe – und ab welcher Neigung sie kippt (so, wie sie gerade steht).';
  renderStand();
});

/** Hinweis, wenn Baugruppen nicht auf die Druckplatte passen. */
function bedWarning(): string {
  const { outside, tooBig } = bedFit();
  const printer = currentPrinter();
  if (tooBig.length) {
    const n = tooBig[0].name;
    const single = tooBig.find((u) => (u as { members?: Part[] }).members?.length === 1) as { members: Part[] } | undefined;
    return ` · <span class="warn">⚠ ${tooBig.length > 1 ? `${tooBig.length} Baugruppen sind` : `„${esc(n)}“ ist`} zu groß für den ${esc(printerName(printer))} (${printer.bed[0]} × ${printer.bed[1]}${printer.height ? ` × ${printer.height}` : ''} mm)</span>`
      + (single ? ` <button class="linkish" data-split-part="${single.members[0].id}">Zerteilen …</button>` : '');
  }
  if (outside.length) return ` · <span class="warn">⚠ ${outside.length > 1 ? `${outside.length} Baugruppen liegen` : `„${esc(outside[0].name)}“ liegt`} nicht ganz auf einer Platte – „Auf der Platte anordnen“ (A)</span>`;
  return '';
}

statusEl.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('[data-split-part]');
  const p = b && getPart(Number(b.dataset.splitPart));
  if (p) openSplit(p);
});

// ---------------------------------------------------------------- Druckplatte

const currentPrinter = () => printerById(bambuPrinter.value) ?? PRINTERS[0];

/** Umriss der Druckplatte des gewählten Druckers, Mitte im Ursprung (dorthin legt der Export die Platte). */
const bedGroup = new THREE.Group();
scene.add(bedGroup);
let bedDrawn = '';

/**
 * Druckplatten wie in Bambu Studio: plateTotal Platten im Raster – etwa √n Spalten, Abstand ein Fünftel
 * der Platte, weitere Reihen nach vorne (siehe bambuPlateOrigin). Platte 1 liegt mit ihrer Mitte im
 * Ursprung. Die Zahl steht fest wie in Bambu Studio; ändert sie sich (Anordnen, Platte hinzufügen,
 * Bambu-Projekt laden) oder der Drucker, wandern die Teile mit ihrer Platte.
 */
let plateTotal = 1;
let platesShown = true;
try { platesShown = localStorage.getItem('partjoin.platesShown') !== '0'; } catch { /* ohne Speicher */ }

/** Mitte von Platte i (bei count Platten dieser Größe). */
function plateCenter(i: number, count = plateTotal, bed = currentPrinter().bed): THREE.Vector2 {
  const [x, y] = bambuPlateOrigin(i, count, bed);
  return new THREE.Vector2(x, y);
}

/** Nächste Platte zu einem Umriss – und liegt er ganz darauf? */
function plateOfBox(box: THREE.Box3, count = plateTotal, bed = currentPrinter().bed): { i: number; inside: boolean } {
  const [w, d] = bed;
  const cx = (box.min.x + box.max.x) / 2;
  const cy = (box.min.y + box.max.y) / 2;
  let i = 0;
  let best = Infinity;
  for (let k = 0; k < count; k++) {
    const c = plateCenter(k, count, bed);
    const dist = ((cx - c.x) / w) ** 2 + ((cy - c.y) / d) ** 2;
    if (dist < best) { best = dist; i = k; }
  }
  const c = plateCenter(i, count, bed);
  const eps = 0.01;
  const inside = box.min.x >= c.x - w / 2 - eps && box.max.x <= c.x + w / 2 + eps && box.min.y >= c.y - d / 2 - eps && box.max.y <= c.y + d / 2 + eps;
  return { i, inside };
}

/** Alle Baugruppen, auch ausgeblendete Teile. */
function allUnits(): Part[][] {
  const seen = new Set<number>();
  const out: Part[][] = [];
  for (const p of parts) {
    const key = unitOf.get(p.id) ?? p.id;
    if (seen.has(key)) continue;
    seen.add(key);
    out.push(unitMembers(p));
  }
  return out;
}

/** Platten, auf oder neben denen etwas liegt (auch Ausgeblendetes) – die übrigen sind leer. */
function occupiedPlates(): Set<number> {
  const used = new Set<number>();
  for (const members of allUnits()) {
    const box = unitBox(members);
    if (!box.isEmpty()) used.add(plateOfBox(box).i);
  }
  return used;
}

/**
 * Platten neu legen: total Platten; was auf Platte i liegt, kommt auf Platte to(i) – an dieselbe Stelle
 * darauf. from: Plattengröße vorher (anderer Drucker).
 */
function relayoutPlates(total: number, to: (i: number) => number = (i) => i, from: [number, number] = currentPrinter().bed) {
  const before = plateTotal;
  total = Math.max(1, total);
  const moves = allUnits().map((members) => {
    const box = unitBox(members);
    if (box.isEmpty()) return null;
    const i = plateOfBox(box, before, from).i;
    const a = plateCenter(i, before, from);
    const b = plateCenter(Math.max(0, Math.min(total - 1, to(i))), total);
    return { members, dx: b.x - a.x, dy: b.y - a.y };
  });
  plateTotal = total;
  for (const m of moves) {
    if (!m || (Math.abs(m.dx) < 1e-9 && Math.abs(m.dy) < 1e-9)) continue;
    const T = new THREE.Matrix4().makeTranslation(m.dx, m.dy, 0);
    for (const q of m.members) q.layout.premultiply(T);
  }
  solveAll();
  if (gizmoTarget) placeGizmo();
}

function addPlate() {
  beginAction();
  relayoutPlates(plateTotal + 1);
  renderAll();
  viewPlate(plateTotal - 1);
  toast(`Platte ${plateTotal} hinzugefügt – Teile mit Strg/⌘ + Ziehen daraufschieben oder „Anordnen“ (A).`, 5000);
}

function removeEmptyPlates() {
  const used = occupiedPlates();
  const keep = Array.from({ length: plateTotal }, (_, i) => i).filter((i) => used.has(i));
  if (keep.length === plateTotal || plateTotal === 1) return;
  const removed = plateTotal - Math.max(1, keep.length);
  beginAction();
  const map = new Map(keep.map((i, k) => [i, k]));
  relayoutPlates(keep.length, (i) => map.get(i) ?? 0);
  renderAll();
  toast(`${removed} leere Platte${removed > 1 ? 'n' : ''} entfernt.`);
}

/** Ältere Projekte ohne Plattenzahl: so viele Platten, dass die meisten Baugruppen ganz auf einer liegen. */
function guessPlates(): number {
  const units = visibleUnits();
  let best = 1;
  let most = -1;
  for (let n = 1; n <= 36; n++) {
    const k = units.filter((u) => plateOfBox(u.box, n).inside).length;
    if (k > most) { most = k; best = n; }
  }
  return best;
}

/** Ansicht auf eine Platte (−1: alle Platten). */
function viewPlate(i: number) {
  const [w, d] = currentPrinter().bed;
  const box = new THREE.Box3();
  for (let k = 0; k < plateTotal; k++) {
    if (i >= 0 && k !== i) continue;
    const c = plateCenter(k);
    box.expandByPoint(new THREE.Vector3(c.x - w / 2, c.y - d / 2, 0)).expandByPoint(new THREE.Vector3(c.x + w / 2, c.y + d / 2, 0));
  }
  for (const p of parts) if (p.visible && (i < 0 || plateOfBox(partBox(p)).i === i)) box.max.z = Math.max(box.max.z, partBox(p).max.z);
  // Etwas Rand: die Beschriftung vor und hinter der Platte soll mit ins Bild
  fitBox(box.expandByVector(new THREE.Vector3(w * 0.05, d * 0.1, 0)));
}

function setPlatesShown(on: boolean) {
  platesShown = on;
  try { localStorage.setItem('partjoin.platesShown', on ? '1' : '0'); } catch { /* ohne Speicher */ }
  drawBed();
}

/** Zwei Düsen: Streifen der Platte (X ab dem linken Rand), die nur eine der Düsen erreicht. */
function nozzleZones(printer = currentPrinter()): { x0: number; x1: number; side: 'links' | 'rechts' }[] {
  if (!printer.reach) return [];
  const [l, r] = printer.reach;
  const out: { x0: number; x1: number; side: 'links' | 'rechts' }[] = [];
  if (r[0] > l[0]) out.push({ x0: l[0], x1: Math.min(r[0], l[1]), side: 'links' });
  if (r[1] > l[1]) out.push({ x0: Math.max(l[1], r[0]), x1: r[1], side: 'rechts' });
  return out;
}
const ZONE_COLORS = { links: '#e39b2d', rechts: '#9a6cf0' };

/** Beschriftung flach auf der Platte (Breite in mm; anchor: wo der Text am Punkt steht). */
function flatLabel(text: string, color: string, width: number, x: number, y: number, anchor: 'left' | 'right' = 'left'): THREE.Mesh {
  const c = document.createElement('canvas');
  c.width = 512;
  c.height = 64;
  const g = c.getContext('2d')!;
  g.font = '600 34px system-ui, sans-serif';
  g.fillStyle = color;
  g.textAlign = anchor;
  g.fillText(text, anchor === 'left' ? 4 : 508, 44);
  const label = new THREE.Mesh(new THREE.PlaneGeometry(width, width / 8), new THREE.MeshBasicMaterial({ map: new THREE.CanvasTexture(c), transparent: true, depthWrite: false }));
  label.position.set(anchor === 'left' ? x + width / 2 : x - width / 2, y, 0.03);
  label.raycast = () => {};
  return label;
}

function drawBed() {
  const printer = currentPrinter();
  const { outside, tooBig } = bedFit();
  const bad = outside.length + tooBig.length > 0;
  bedGroup.visible = platesShown;
  // Ohne Platten: ein schlichtes Raster als Boden
  grid.visible = !platesShown;
  renderPlateBar();
  const key = `${printer.id}|${bad}|${plateTotal}`;
  if (key === bedDrawn) return;
  bedDrawn = key;
  disposeGroup(bedGroup);
  const [w, d] = printer.bed;
  const color = bad ? 0xe5484d : 0x2f6fde;
  const zones = nozzleZones(printer);
  const lw = Math.min(w * 0.6, 160);
  for (let i = 0; i < plateTotal; i++) {
    const { x, y } = plateCenter(i);
    const plate = new THREE.Mesh(
      new THREE.PlaneGeometry(w, d),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.06, depthWrite: false, side: THREE.DoubleSide }),
    );
    plate.position.set(x, y, -0.02);
    plate.raycast = () => {};
    const outline = new THREE.LineLoop(
      new THREE.BufferGeometry().setFromPoints([[-w / 2, -d / 2], [w / 2, -d / 2], [w / 2, d / 2], [-w / 2, d / 2]].map(([px, py]) => new THREE.Vector3(x + px, y + py, 0.02))),
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.8 }),
    );
    outline.raycast = () => {};
    // Raster alle 10 mm
    const lines: number[] = [];
    for (let gx = 10; gx < w - 1e-6; gx += 10) lines.push(x - w / 2 + gx, y - d / 2, 0.01, x - w / 2 + gx, y + d / 2, 0.01);
    for (let gy = 10; gy < d - 1e-6; gy += 10) lines.push(x - w / 2, y - d / 2 + gy, 0.01, x + w / 2, y - d / 2 + gy, 0.01);
    const gg = new THREE.BufferGeometry();
    gg.setAttribute('position', new THREE.Float32BufferAttribute(lines, 3));
    const gridLines = new THREE.LineSegments(gg, new THREE.LineBasicMaterial({ color: 0x8890a0, transparent: true, opacity: 0.22, depthWrite: false }));
    gridLines.raycast = () => {};
    // Beschriftung vorne links
    const text = i === 0 ? `${printerName(printer)} · ${w} × ${d} mm${plateTotal > 1 ? ' · Platte 1' : ''}` : `Platte ${i + 1}`;
    bedGroup.add(plate, outline, gridLines, flatLabel(text, bad ? '#e5484d' : '#2f6fde', lw, x - w / 2 + 2, y - d / 2 - lw / 16 - 2));
    // Bereiche, die nur eine Düse erreicht: getönt, mit Grenzlinie und Beschriftung hinter der Platte
    for (const z of zones) {
      const zc = new THREE.Color(ZONE_COLORS[z.side]);
      const zw = z.x1 - z.x0;
      const strip = new THREE.Mesh(new THREE.PlaneGeometry(zw, d), new THREE.MeshBasicMaterial({ color: zc, transparent: true, opacity: 0.18, depthWrite: false, side: THREE.DoubleSide }));
      strip.position.set(x - w / 2 + z.x0 + zw / 2, y, -0.01);
      strip.raycast = () => {};
      const edge = x - w / 2 + (z.side === 'links' ? z.x1 : z.x0);
      const border = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(edge, y - d / 2, 0.02), new THREE.Vector3(edge, y + d / 2, 0.02)]),
        new THREE.LineDashedMaterial({ color: zc, dashSize: 4, gapSize: 3, transparent: true, opacity: 0.9 }),
      );
      border.computeLineDistances();
      border.raycast = () => {};
      const zl = Math.min(w * 0.45, 110);
      const label = z.side === 'links'
        ? flatLabel('◧ nur linke Düse', ZONE_COLORS.links, zl, x - w / 2, y + d / 2 + zl / 16 + 1, 'left')
        : flatLabel('nur rechte Düse ◨', ZONE_COLORS.rechts, zl, x + w / 2, y + d / 2 + zl / 16 + 1, 'right');
      bedGroup.add(strip, border, label);
    }
  }
}

/** Liegt eine Baugruppe in einem Streifen, den nur eine Düse erreicht? Hinweis für die Statuszeile. */
function nozzleNote(): string {
  const zones = nozzleZones();
  if (!zones.length) return '';
  const [w] = currentPrinter().bed;
  const hits: { name: string; sides: string[] }[] = [];
  for (const u of visibleUnits()) {
    const pl = plateOfBox(u.box);
    if (!pl.inside) continue;
    const x0 = plateCenter(pl.i).x - w / 2;
    const sides = zones.filter((z) => u.box.min.x < x0 + z.x1 - 0.01 && u.box.max.x > x0 + z.x0 + 0.01).map((z) => z.side);
    if (sides.length) hits.push({ name: u.name, sides });
  }
  if (!hits.length) return '';
  const one = hits[0];
  const text = hits.length > 1
    ? `${hits.length} Baugruppen liegen in Bereichen, die nur eine Düse erreicht`
    : one.sides.length > 1
      ? `„${esc(one.name)}“ reicht in beide Randstreifen, die jeweils nur eine Düse erreicht`
      : `„${esc(one.name)}“ liegt im Bereich, den nur die ${one.sides[0] === 'links' ? 'linke' : 'rechte'} Düse erreicht`;
  return ` · <span class="note" title="Der Slicer druckt dort nur mit den Filamenten dieser Düse – mehrfarbige Teile besser weiter zur Mitte schieben.">◧ ${text}</span>`;
}

const plateBar = document.getElementById('plateBar')!;
let plateBarHtml = '';
function renderPlateBar() {
  const used = occupiedPlates();
  const list = document.getElementById('plateList')!;
  const html = Array.from({ length: plateTotal }, (_, i) => `<button class="ghost small${used.has(i) ? '' : ' empty'}" data-plate="${i}" title="Platte ${i + 1} ansehen${used.has(i) ? '' : ' (leer)'}">${i + 1}</button>`).join('')
    + (plateTotal > 1 && used.size < plateTotal ? '<button class="ghost small" data-plate-trim title="Leere Platten entfernen">−</button>' : '');
  // Nur bei Änderung neu aufbauen (ein Klick darauf soll nicht ins Leere gehen)
  if (html !== plateBarHtml) list.innerHTML = plateBarHtml = html;
  document.getElementById('platesShow')!.classList.toggle('on', platesShown);
  plateBar.classList.toggle('off', !platesShown);
}
plateBar.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('button');
  if (!b) return;
  if (b.id === 'platesShow') setPlatesShown(!platesShown);
  else if (b.id === 'plateAdd') addPlate();
  else if (b.dataset.plateTrim !== undefined) removeEmptyPlates();
  else if (b.dataset.plate !== undefined) viewPlate(Number(b.dataset.plate));
});

/** Sichtbare Baugruppen mit ihrem Umriss (Weltkoordinaten). */
function visibleUnits(): { members: Part[]; box: THREE.Box3; name: string }[] {
  const seen = new Set<number>();
  const out: { members: Part[]; box: THREE.Box3; name: string }[] = [];
  for (const p of parts) {
    if (!p.visible) continue;
    const key = unitOf.get(p.id) ?? p.id;
    if (seen.has(key)) continue;
    seen.add(key);
    const members = unitMembers(p).filter((q) => q.visible);
    out.push({ members, box: unitBox(members), name: getPart(key)?.name ?? p.name });
  }
  return out;
}

/** Welche Baugruppen liegen neben der Platte, welche sind größer als sie (auch gedreht)? */
function bedFit() {
  const printer = currentPrinter();
  const [w, d] = printer.bed;
  const eps = 0.01;
  const outside: { name: string }[] = [];
  const tooBig: { name: string }[] = [];
  for (const u of visibleUnits()) {
    const s = u.box.getSize(new THREE.Vector3());
    const fits = ((s.x <= w + eps && s.y <= d + eps) || (s.x <= d + eps && s.y <= w + eps)) && (!printer.height || s.z <= printer.height + eps);
    if (!fits) tooBig.push(u);
    else if (!plateOfBox(u.box).inside) outside.push(u);
  }
  return { outside, tooBig };
}

/**
 * Alle sichtbaren Baugruppen auf der Platte anordnen: Reihen von vorn nach hinten, größte zuerst, mit
 * Abstand. Was nicht mehr passt, kommt rechts neben die Platte.
 */
async function arrangeOnBed() {
  const units = visibleUnits();
  if (!units.length) { toast('Keine sichtbaren Teile.'); return; }
  beginAction();
  const [w, d] = currentPrinter().bed;
  const gap = 6;
  const items = units
    .map((u) => ({ ...u, size: u.box.getSize(new THREE.Vector3()) }))
    .sort((a, b) => b.size.y - a.size.y || b.size.x - a.size.x);
  // Platte für Platte füllen: Reihen von vorn nach hinten; was auf keine Platte passt, kommt dahinter
  let plate = 0;
  let x = -w / 2;
  let y = -d / 2;
  let rowDepth = 0;
  const big = items.filter((it) => it.size.x > w || it.size.y > d);
  // Erst nur Platte und Lage darauf (Mitte der Platte = 0) – die Lage der Platten hängt von ihrer Zahl ab
  const local = new Map<(typeof items)[number], [number, number, number]>();
  for (const it of items) {
    if (big.includes(it)) continue;
    if (x + it.size.x > w / 2 && x > -w / 2) {
      // neue Reihe
      x = -w / 2;
      y += rowDepth + gap;
      rowDepth = 0;
    }
    if (y + it.size.y > d / 2) {
      // neue Platte
      plate++;
      x = -w / 2;
      y = -d / 2;
      rowDepth = 0;
    }
    local.set(it, [plate, x + it.size.x / 2, y + it.size.y / 2]);
    x += it.size.x + gap;
    rowDepth = Math.max(rowDepth, it.size.y);
  }
  const plates = plate + 1;
  // So viele Platten wie nötig; Ausgeblendetes wandert mit seiner Platte
  relayoutPlates(plates);
  const placed = new Map<(typeof items)[number], [number, number]>();
  for (const [it, [i, lx, ly]] of local) {
    const c = plateCenter(i);
    placed.set(it, [c.x + lx, c.y + ly]);
  }
  // Zu groß für jede Platte: rechts neben das Raster
  let ox = Math.max(...Array.from({ length: plates }, (_, i) => plateCenter(i).x)) + w / 2 + 20;
  for (const it of big) {
    placed.set(it, [ox + it.size.x / 2, -d / 2 + it.size.y / 2]);
    ox += it.size.x + gap;
  }
  const overflow = big.length;
  for (const it of items) {
    const [cx, cy] = placed.get(it)!;
    const c = unitBox(it.members).getCenter(new THREE.Vector3());
    const T = new THREE.Matrix4().makeTranslation(cx - c.x, cy - c.y, 0);
    // Ganze Baugruppe verschieben (auch ausgeblendete Teile)
    for (const q of unitMembers(it.members[0])) q.layout.premultiply(T);
  }
  solveAll();
  if (gizmoTarget) placeGizmo();
  renderAll();
  fitView();
  const where = plates > 1 ? `auf ${plates} Platten` : 'auf der Platte';
  toast(overflow
    ? `${items.length - overflow} von ${items.length} Baugruppen ${where} angeordnet, ${overflow} ${overflow > 1 ? 'sind' : 'ist'} für jede Platte zu groß und ${overflow > 1 ? 'liegen' : 'liegt'} daneben – „Auf Druckplattengröße zerteilen“.`
    : `${items.length} Baugruppe${items.length === 1 ? '' : 'n'} ${where} angeordnet.${plates > 1 ? ' Der Export ins Slicer-Projekt übernimmt die Platten.' : ''}`, 6000);
}
document.getElementById('arrange')!.addEventListener('click', () => void arrangeOnBed());

// ---------------------------------------------------------------- Messen

/**
 * Messen wie in Bambu Studio: Unter dem Zeiger leuchtet das Merkmal auf – Ecke, Kante, Kreis oder Bogen,
 * ebene Fläche, auf Rundungen der Zylinder. Ein Klick wählt es (Kante: Länge, Kreis: Durchmesser), ein
 * zweites zeigt Winkel und Abstände. Nochmal klicken wählt ab, ein drittes ersetzt das zweite. Bei Kreisen
 * und Vielecken lässt sich der Mittelpunkt anklicken, mit Umschalt ein beliebiger Punkt. Entf beginnt neu.
 * Gespeichert in Koordinaten des Teils – bewegt es sich, wandert die Messung mit.
 */
interface MPick {
  part: Part;
  f: MFeature;
  /** Mitte bzw. Punkt auf diesem Merkmal */
  src?: MFeature;
  /** Netz, auf dem das Merkmal gefunden wurde (ändert es sich, etwa durch Rückgängig, fällt es weg) */
  geom?: THREE.BufferGeometry;
  /** f liegt schon in Weltkoordinaten (Maßlinien) */
  world?: boolean;
}
let measureSel: MPick[] = [];
let measureHover: MPick | null = null;
/** Letzter Treffer unter dem Zeiger (für Umschalt ohne Mausbewegung). */
let measureLast: { h: Hit | null; shift: boolean } = { h: null, shift: false };
const measureGroup = new THREE.Group();
scene.add(measureGroup);
const measureHoverGroup = new THREE.Group();
scene.add(measureHoverGroup);
const measureBar = document.getElementById('measureBar')!;
const measureText = document.getElementById('measureText')!;
const measureLabels = document.createElement('div');
measureLabels.id = 'measureLabels';
view.appendChild(measureLabels);
let measureLabelAt: { el: HTMLElement; at: THREE.Vector3 }[] = [];
const measureModels = new WeakMap<THREE.BufferGeometry, MeasureModel>();
const MEASURE_COLORS = { hover: '#ffc266', first: '#ff8a00', second: '#14b8a6', line: '#ffb000', perp: '#b48cff' };

function measureModelOf(p: Part): MeasureModel {
  let m = measureModels.get(p.geom);
  if (!m) measureModels.set(p.geom, (m = measureModel(topologyOf(p))));
  return m;
}

function clearMeasure() {
  measureSel = [];
  measureHover = null;
  disposeGroup(measureGroup);
  disposeGroup(measureHoverGroup);
  measureLabels.replaceChildren();
  measureLabelAt = [];
  measureBar.hidden = true;
}

const mmText2 = (x: number) => x.toFixed(2).replace('.', ',');

const degText = (x: number) => `${x.toFixed(1).replace('.', ',')}°`;

const worldOf = (s: MPick) => (s.world ? s.f : toWorld(s.f, s.part.mesh.matrixWorld.elements));

/** Beschreibung eines gewählten Merkmals, z. B. „Kante · Länge 25,00 mm“. */
function measureName(s: MPick): string {
  const f = worldOf(s);
  const src = s.src && toWorld(s.src, s.part.mesh.matrixWorld.elements);
  switch (f.kind) {
    case 'point':
      if (f.on === 'center') return src?.kind === 'cylinder' ? 'Mitte der Achse' : src?.kind === 'edge' ? `Mitte des ${src.sides ?? ''}-Ecks` : 'Mitte des Kreises';
      return { vertex: 'Ecke', edge: 'Punkt auf Kante', circle: 'Punkt auf Kreis', plane: 'Punkt auf Fläche', surface: 'Punkt auf Fläche', center: 'Mitte' }[f.on];
    case 'edge': return `Kante · Länge <b>${mmText2(Math.hypot(...sub(f.b, f.a)))} mm</b>${f.sides ? ` (${f.sides}-Eck)` : ''}`;
    case 'circle': return f.arc >= 2 * Math.PI - 1e-6
      ? `Kreis · <b>⌀ ${mmText2(2 * f.radius)} mm</b>`
      : `Bogen · <b>⌀ ${mmText2(2 * f.radius)} mm</b> (r ${mmText2(f.radius)}, ${degText(f.arc / Math.PI * 180)})`;
    case 'plane': return `Fläche · ${mmText2(f.area)} mm²`;
    case 'cylinder': return `${f.hole ? 'Bohrung' : 'Zapfen'} · <b>⌀ ${mmText2(2 * f.radius)} mm</b>, ${mmText2(f.length)} mm lang`;
  }
}

/** Sind zwei Auswahlen dasselbe Merkmal? */
function sameMeasure(a: MPick, b: MPick): boolean {
  if (a.part !== b.part || a.f.kind !== b.f.kind) return false;
  const near = (x: Vec3, y: Vec3) => Math.hypot(...sub(x, y)) < 1e-6;
  const f = a.f;
  const g = b.f as typeof f;
  switch (f.kind) {
    case 'point': return near(f.p, (g as typeof f).p);
    case 'edge': return (near(f.a, (g as typeof f).a) && near(f.b, (g as typeof f).b)) || (near(f.a, (g as typeof f).b) && near(f.b, (g as typeof f).a));
    case 'circle': return near(f.center, (g as typeof f).center) && Math.abs(f.radius - (g as typeof f).radius) < 1e-6;
    case 'plane': return f.region === (g as typeof f).region;
    case 'cylinder': return f.tris[0] === (g as typeof f).tris[0];
  }
}

/** Modellgröße eines Bildschirmpixels an einem Punkt. */
const pxAt = (p: Vec3) => (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) * camera.position.distanceTo(new THREE.Vector3(...p))) / viewHeight;

// ---------------------------------------------------------------- Maße im Bild

const dimGroup = new THREE.Group();
scene.add(dimGroup);
const dimLabels = document.createElement('div');
dimLabels.id = 'dimLabels';
view.appendChild(dimLabels);
let dimLabelAt: { el: HTMLElement; at: THREE.Vector3 }[] = [];
const dimsBtn = document.getElementById('dimsTool')!;
const DIM_COLORS = { x: '#e5484d', y: '#30a46c', z: '#3b82f6' };

/** Bildschirmabstand zweier Weltpunkte (Pixel). */
function screenDist(a: THREE.Vector3, b: THREE.Vector3): number {
  const pa = a.clone().project(camera);
  const pb = b.clone().project(camera);
  return Math.hypot((pa.x - pb.x) * renderer.domElement.clientWidth, (pa.y - pb.y) * viewHeight) / 2;
}

/** Maßlinie a–b mit Hilfslinien von den Kanten des Teils (ea, eb) und Pfeilen an den Enden. */
function dimLine(a: THREE.Vector3, b: THREE.Vector3, ea: THREE.Vector3, eb: THREE.Vector3, axis: 'x' | 'y' | 'z', value: number) {
  // Fast in Blickrichtung: nicht lesbar, also weglassen (z. B. die Höhe in der Draufsicht)
  const dir = b.clone().sub(a).normalize();
  const view = a.clone().add(b).multiplyScalar(0.5).sub(camera.position).normalize();
  if (screenDist(a, b) < 24 || Math.abs(dir.dot(view)) > 0.94) return;
  const color = DIM_COLORS[axis];
  // Hilfslinien etwas über die Maßlinie hinaus
  const over = (e: THREE.Vector3, to: THREE.Vector3) => to.clone().add(to.clone().sub(e).normalize().multiplyScalar(4 * pxAt(v3(to))));
  const lines = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints([a, b, ea, over(ea, a), eb, over(eb, b)]), new THREE.LineBasicMaterial({ color, depthTest: false }));
  dimGroup.add(lines);
  for (const [p, d] of [[a, dir.clone().negate()], [b, dir]] as const) {
    const len = 9 * pxAt(v3(p));
    const cone = new THREE.Mesh(new THREE.ConeGeometry(len * 0.32, len, 10), new THREE.MeshBasicMaterial({ color, depthTest: false }));
    cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), d);
    // Spitze auf dem Endpunkt
    cone.position.copy(p).addScaledVector(d, -len / 2);
    dimGroup.add(cone);
  }
  const el = document.createElement('span');
  el.className = `mlabel d${axis}`;
  el.textContent = `${axis.toUpperCase()} ${mmText1(value)} mm`;
  dimLabels.appendChild(el);
  dimLabelAt.push({ el, at: a.clone().add(b).multiplyScalar(0.5) });
}

/** Breite, Tiefe und Höhe eines Umrisses – an den Kanten, die zur Kamera zeigen. */
function drawDims(b: THREE.Box3) {
  const size = b.getSize(new THREE.Vector3());
  const c = b.getCenter(new THREE.Vector3());
  const cam = camera.position;
  const gap = Math.max(1, 0.08 * Math.max(size.x, size.y, size.z));
  const y = cam.y < c.y ? b.min.y : b.max.y;
  const sy = y === b.min.y ? -1 : 1;
  const x = cam.x > c.x ? b.max.x : b.min.x;
  const sx = x === b.max.x ? 1 : -1;
  const z = b.min.z;
  const V = (px: number, py: number, pz: number) => new THREE.Vector3(px, py, pz);
  dimLine(V(b.min.x, y + sy * gap, z), V(b.max.x, y + sy * gap, z), V(b.min.x, y, z), V(b.max.x, y, z), 'x', size.x);
  dimLine(V(x + sx * gap, b.min.y, z), V(x + sx * gap, b.max.y, z), V(x, b.min.y, z), V(x, b.max.y, z), 'y', size.y);
  const g = 0.7 * gap;
  dimLine(V(x + sx * g, y + sy * g, b.min.z), V(x + sx * g, y + sy * g, b.max.z), V(x, y, b.min.z), V(x, y, b.max.z), 'z', size.z);
}

function renderDims() {
  disposeGroup(dimGroup);
  dimLabels.replaceChildren();
  dimLabelAt = [];
  if (!dimsOn) return;
  const sel = selectedParts().filter((p) => p.visible);
  const boxes = sel.length ? [unitBox(sel)] : visibleUnits().map((u) => u.box);
  for (const b of boxes) if (!b.isEmpty()) drawDims(b);
  dimGroup.traverse((o) => { o.renderOrder = 12; o.raycast = () => {}; });
  placeLabelList(dimLabelAt, true);
}

function setDims(on: boolean) {
  dimsOn = on && parts.length > 0;
  dimsBtn.classList.toggle('on', dimsOn);
  renderDims();
}
dimsBtn.addEventListener('click', () => setDims(!dimsOn));
// Kamera bewegt: Maßlinien an die Seiten zur Kamera, Pfeile in Bildschirmgröße
let dimsRedraw = 0;
controls.addEventListener('change', () => {
  if (!dimsOn || dimsRedraw) return;
  dimsRedraw = requestAnimationFrame(() => {
    dimsRedraw = 0;
    renderDims();
  });
});

/** Darstellung eines Merkmals (Welt): Punkt als Kugel, Kante als Stab, Kreis als Ring, Fläche getönt. */
function measureObject(s: MPick, color: string, emphasis = 1): THREE.Object3D {
  const f = worldOf(s);
  const mat = (opacity = 0.95) => new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthTest: false });
  const group = new THREE.Group();
  const tube = (a: Vec3, b: Vec3, r: number) => {
    const A = new THREE.Vector3(...a);
    const B = new THREE.Vector3(...b);
    const g = new THREE.CylinderGeometry(r, r, A.distanceTo(B), 8);
    g.applyQuaternion(new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), B.clone().sub(A).normalize()));
    g.translate(...A.add(B).multiplyScalar(0.5).toArray());
    return new THREE.Mesh(g, mat());
  };
  const dot = (p: Vec3, px: number) => {
    const m = new THREE.Mesh(new THREE.SphereGeometry(px * pxAt(p), 14, 10), mat());
    m.position.set(...p);
    return m;
  };
  if (f.kind === 'point') group.add(dot(f.p, 5 * emphasis));
  else if (f.kind === 'edge') group.add(tube(f.a, f.b, 1.6 * emphasis * pxAt(f.a.map((x, i) => (x + f.b[i]) / 2) as Vec3)));
  else if (f.kind === 'circle') {
    const steps = Math.max(12, Math.ceil((f.arc / (2 * Math.PI)) * 96));
    const v = cross(f.normal, f.start);
    const pts = Array.from({ length: steps + 1 }, (_, i) => {
      const t = (f.arc * i) / steps;
      return new THREE.Vector3(...f.center).addScaledVector(new THREE.Vector3(...f.start), f.radius * Math.cos(t)).addScaledVector(new THREE.Vector3(...v), f.radius * Math.sin(t));
    });
    const curve = new THREE.CatmullRomCurve3(pts, f.arc >= 2 * Math.PI - 1e-6);
    group.add(new THREE.Mesh(new THREE.TubeGeometry(curve, steps, 1.6 * emphasis * pxAt(f.center), 6, f.arc >= 2 * Math.PI - 1e-6), mat()));
  } else {
    // Fläche bzw. Zylinder: Dreiecke getönt
    const tris = f.kind === 'cylinder' ? f.tris : measureModelOf(s.part).regions[f.region].tris;
    // Mit Tiefenprüfung, damit die Tönung nicht durch andere Teile scheint
    const tint = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.35, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2 });
    const m = new THREE.Mesh(subGeometry(s.part, tris), tint);
    m.matrixAutoUpdate = false;
    m.matrix.copy(s.part.mesh.matrixWorld);
    group.add(m);
  }
  // Mittelpunkt zum Anklicken (Kreis, Vieleck, Zylinder)
  const c = centerOf(f);
  if (c && emphasis <= 1 && measureSel.includes(s)) group.add(dot(c.p, 3.5));
  group.traverse((o) => { o.renderOrder = 12; o.raycast = () => {}; });
  return group;
}

function measureLabel(text: string, at: Vec3, kind = '') {
  const el = document.createElement('span');
  el.className = `mlabel ${kind}`;
  el.innerHTML = text;
  measureLabels.appendChild(el);
  measureLabelAt.push({ el, at: new THREE.Vector3(...at) });
}

/** Beschriftungen an ihre Stelle im Bild (jedes Bild neu). */
function placeMeasureLabels() {
  placeLabelList(measureLabelAt);
}

/** center: mitten auf den Punkt (Maßzahl auf der Maßlinie), sonst darüber. */
function placeLabelList(list: { el: HTMLElement; at: THREE.Vector3 }[], center = false) {
  if (!list.length) return;
  const w = renderer.domElement.clientWidth;
  for (const { el, at } of list) {
    const p = at.clone().project(camera);
    const hidden = p.z > 1 || p.z < -1;
    el.style.display = hidden ? 'none' : '';
    el.style.transform = `translate(${((p.x + 1) / 2) * w}px, ${((1 - p.y) / 2) * viewHeight}px) translate(-50%, ${center ? '-50%' : '-130%'})`;
  }
}

/** Gewählte Merkmale, Maßlinien und Werte zeichnen. */
function renderMeasure() {
  disposeGroup(measureGroup);
  measureLabels.replaceChildren();
  measureLabelAt = [];
  if (tool !== 'measure') return;
  measureSel = measureSel.filter((s) => getPart(s.part.id) === s.part && s.part.visible && s.geom === s.part.geom);
  const colors = [MEASURE_COLORS.first, MEASURE_COLORS.second];
  measureSel.forEach((s, i) => measureGroup.add(measureObject(s, colors[i])));
  const rows: string[] = [];
  const copy = (v: string) => `<button class="icon mcopy" data-copy="${v}" title="Wert kopieren" aria-label="Wert kopieren">⧉</button>`;
  measureSel.forEach((s, i) => rows.push(`<div class="msel"><i style="background:${colors[i]}"></i><span>${measureName(s)}</span><button class="icon" data-mdrop="${i}" title="Abwählen" aria-label="Abwählen">✕</button></div>`));
  const line = (from: Vec3, to: Vec3, color: string) => {
    if (Math.hypot(...sub(to, from)) < 1e-9) return;
    measureGroup.add(measureObject({ part: measureSel[0].part, f: { kind: 'edge', a: from, b: to }, world: true }, color, 0.6));
  };
  if (measureSel.length === 1) {
    // Ein Merkmal: sein Maß direkt am Teil
    const f = worldOf(measureSel[0]);
    if (f.kind === 'edge') measureLabel(`${mmText2(Math.hypot(...sub(f.b, f.a)))} mm`, f.a.map((x, i) => (x + f.b[i]) / 2) as Vec3);
    else if (f.kind === 'circle' || f.kind === 'cylinder') measureLabel(`⌀ ${mmText2(2 * f.radius)} mm`, f.center);
  } else if (measureSel.length === 2) {
    const [a, b] = measureSel.map(worldOf);
    const r = measureBetween(a, b);
    if (r.angle) {
      rows.push(`<div class="mres"><span>Winkel <b>${degText(r.angle.deg)}</b>${r.angle.inner !== undefined && degText(r.angle.inner) !== degText(r.angle.deg) ? ` (dazwischen ${degText(r.angle.inner)})` : ''}</span>${copy(r.angle.deg.toFixed(2))}</div>`);
    }
    if (r.perpendicular) {
      const p = r.perpendicular;
      const name = p.axis ? 'Achsabstand' : r.direct ? 'Senkrechter Abstand' : 'Abstand';
      rows.push(`<div class="mres"><span>${name} <b>${mmText2(p.d)} mm</b></span>${copy(p.d.toFixed(3))}</div>`);
      line(p.from, p.to, MEASURE_COLORS.perp);
      measureLabel(`${mmText2(p.d)} mm`, p.from.map((x, i) => (x + p.to[i]) / 2) as Vec3, 'perp');
    }
    if (r.direct) {
      const d = r.direct;
      rows.push(`<div class="mres"><span>${r.perpendicular ? 'Direkter Abstand' : 'Abstand'} <b>${mmText2(d.d)} mm</b></span>${copy(d.d.toFixed(3))}</div>`);
      const delta = sub(d.to, d.from);
      if (d.d > 1e-6) rows.push(`<div class="mres xyz">ΔX ${mmText2(Math.abs(delta[0]))} · ΔY ${mmText2(Math.abs(delta[1]))} · ΔZ ${mmText2(Math.abs(delta[2]))} mm</div>`);
      line(d.from, d.to, MEASURE_COLORS.line);
      measureLabel(`${mmText2(d.d)} mm`, d.from.map((x, i) => (x + d.to[i]) / 2) as Vec3);
    }
    if (!r.angle && !r.perpendicular && !r.direct) rows.push('<div class="mres">Kein gemeinsames Maß – z. B. zwei Flächen, Kanten oder Punkte wählen.</div>');
  }
  if (!measureSel.length) rows.push('<div class="mres help">Zeige auf eine Kante, einen Kreis, eine Ecke oder Fläche und klicke. Umschalt: beliebiger Punkt.</div>');
  measureText.innerHTML = rows.join('');
  measureBar.hidden = false;
  placeMeasureLabels();
  renderHint();
}

/** Hervorhebung unter dem Zeiger; Umschalt: der Punkt auf dem Merkmal. */
function updateMeasureHover(h: Hit | null, shift: boolean) {
  measureLast = { h, shift };
  disposeGroup(measureHoverGroup);
  measureHover = null;
  if (!h) return;
  const m = measureModelOf(h.part);
  // Fangbereich: 8 Pixel, in Koordinaten des Teils
  const s = h.part.mesh.matrixWorld.getMaxScaleOnAxis() || 1;
  let f = featureAt(m, h.tri, h.raw, (8 * h.worldPerPx) / s);
  let src: MFeature | undefined;
  if (shift) {
    src = f;
    f = pointOn(f, h.raw);
  }
  measureHover = { part: h.part, f, src, geom: h.part.geom };
  const known = measureSel.find((x) => sameMeasure(x, measureHover!));
  if (!known) measureHoverGroup.add(measureObject(measureHover, MEASURE_COLORS.hover));
}

/** Mittelpunkt eines gewählten Merkmals unter dem Zeiger (höchstens 10 Pixel entfernt)? */
function measureCenterAt(e: MouseEvent): { i: number; pick: MPick } | null {
  const rect = renderer.domElement.getBoundingClientRect();
  for (let i = measureSel.length - 1; i >= 0; i--) {
    const s = measureSel[i];
    const base = s.src && s.f.kind === 'point' && s.f.on === 'center' ? s.src : s.f;
    const c = centerOf(base);
    if (!c) continue;
    const p = new THREE.Vector3(...c.p).applyMatrix4(s.part.mesh.matrixWorld).project(camera);
    const x = rect.left + ((p.x + 1) / 2) * rect.width;
    const y = rect.top + ((1 - p.y) / 2) * rect.height;
    if (Math.hypot(e.clientX - x, e.clientY - y) <= 10) {
      // Schon die Mitte gewählt: zurück zum Merkmal, sonst zur Mitte
      return { i, pick: base === s.f ? { part: s.part, f: c, src: base, geom: s.geom } : { part: s.part, f: base, geom: s.geom } };
    }
  }
  return null;
}

/** Klick im Messen: wählen, abwählen, ersetzen (wie in Bambu Studio). */
function clickMeasure(e: MouseEvent, h: Hit | null) {
  const center = measureCenterAt(e);
  if (center) {
    measureSel[center.i] = center.pick;
  } else {
    if (!h) return;
    if (!measureHover || measureHover.part !== h.part) updateMeasureHover(h, e.shiftKey);
    const item = measureHover!;
    const i = measureSel.findIndex((s) => sameMeasure(s, item));
    if (i >= 0) measureSel.splice(i, 1);
    else if (measureSel.length < 2) measureSel.push(item);
    else measureSel[1] = item;
  }
  disposeGroup(measureHoverGroup);
  renderMeasure();
}

measureText.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('button');
  if (!b) return;
  if (b.dataset.copy) {
    void navigator.clipboard?.writeText(b.dataset.copy.replace('.', ',')).then(() => toast(`${b.dataset.copy!.replace('.', ',')} kopiert.`), () => toast('Kopieren nicht möglich.'));
  } else if (b.dataset.mdrop) {
    measureSel.splice(Number(b.dataset.mdrop), 1);
    renderMeasure();
  }
});
document.getElementById('measureReset')!.addEventListener('click', () => { measureSel = []; renderMeasure(); });
// Umschalt drücken/loslassen ohne Mausbewegung: Hervorhebung wechselt zum Punkt bzw. Merkmal
for (const t of ['keydown', 'keyup'] as const) {
  window.addEventListener(t, (e) => {
    if (tool !== 'measure' || e.key !== 'Shift' || e.shiftKey === measureLast.shift) return;
    updateMeasureHover(measureLast.h, e.shiftKey);
  });
}
// Dicke der Linien und Kugeln in Bildschirmpixeln: nach Kamerabewegung neu zeichnen
let measureRedraw = 0;
controls.addEventListener('change', () => {
  if (tool !== 'measure' || measureRedraw) return;
  measureRedraw = requestAnimationFrame(() => {
    measureRedraw = 0;
    renderMeasure();
    if (measureLast.h) updateMeasureHover(measureLast.h, measureLast.shift);
  });
});

// ---------------------------------------------------------------- Passung

const fitBar = document.getElementById('fitBar')!;
const fitValue = document.getElementById('fitValue') as HTMLInputElement;

const fitGroup = new THREE.Group();
scene.add(fitGroup);
const fitAlso = document.getElementById('fitAlso') as HTMLInputElement;

function closeFit() {
  fitTarget = null;
  fitBar.hidden = true;
  disposeGroup(fitGroup);
  updateHover(null);
}

/** Ganze Wand der Bohrung bzw. des Zapfens: alle sichtbaren Teile, deren Umriss die Achse erreicht. */
function fitWallOf(part: Part, f: CylinderFeature) {
  const W = part.mesh.matrixWorld;
  const c = new THREE.Vector3(...f.center).applyMatrix4(W);
  const a = new THREE.Vector3(...f.axis).transformDirection(W);
  const near = parts.filter((p) => {
    if (!p.visible) return false;
    if (p === part) return true;
    // Abstand des Umrisses zur Achse (grob: Mitte minus halbe Diagonale)
    const b = partBox(p);
    const m = b.getCenter(new THREE.Vector3()).sub(c);
    const off = m.clone().sub(a.clone().multiplyScalar(m.dot(a))).length();
    return off - b.getSize(new THREE.Vector3()).length() / 2 <= f.radius + 1;
  });
  const sources = near.map((p) => ({ topo: topologyOf(p), matrix: p.mesh.matrixWorld.elements }));
  const w = cylinderWall(sources, { source: near.indexOf(part), tris: f.tris, center: f.center, axis: f.axis, radius: f.radius, hole: f.hole });
  return { center: w.center, axis: w.axis, radius: w.radius, span: w.span, pieces: near.map((p, i) => ({ part: p, tris: w.pieces[i] })).filter((x) => x.tris.length) };
}

/** Gewählte Wand hervorheben (in allen Teilen, die dazugehören). */
function showFitWall() {
  disposeGroup(fitGroup);
  const wall = fitTarget?.wall;
  if (!wall) return;
  for (const { part, tris } of wall.pieces) {
    if (part !== fitTarget!.part && !fitAlso.checked) continue;
    const m = new THREE.Mesh(subGeometry(part, tris), new THREE.MeshBasicMaterial({
      color: new THREE.Color(0x2f6fde).lerp(new THREE.Color(1, 1, 1), 0.4), transparent: true, opacity: 0.85,
      polygonOffset: true, polygonOffsetFactor: -2, polygonOffsetUnits: -2,
    }));
    m.matrixAutoUpdate = false;
    m.matrix.copy(part.mesh.matrixWorld);
    m.renderOrder = 2;
    fitGroup.add(m);
  }
  hoverMesh.visible = false;
}
fitAlso.addEventListener('change', showFitWall);

const mm2 = (x: number) => x.toFixed(2).replace('.', ',');

function pickFit(h: Hit) {
  const f = detectFeature(topologyOf(h.part), h.tri, FEATURE_OPTS);
  if (f.kind !== 'cylinder' && f.kind !== 'plane') {
    toast('Passung geht für Zapfen, Bohrungen und ebene Flächen – hier ist eine Freiformfläche.');
    return;
  }
  fitTarget = null;
  updateHover(h, true);
  fitTarget = { part: h.part, f };
  const quick = document.getElementById('fitQuick')!;
  const help = document.getElementById('fitHelp')!;
  const alsoWrap = document.getElementById('fitAlsoWrap')!;
  alsoWrap.hidden = true;
  if (f.kind === 'cylinder') {
    // Ganze Wand, auch über Schnitte hinweg in angrenzende Teile; der Durchmesser aus allen Stücken
    const wall = fitWallOf(h.part, f);
    fitTarget.wall = wall;
    const others = wall.pieces.filter((x) => x.part !== h.part).map((x) => x.part);
    if (others.length) {
      alsoWrap.hidden = false;
      fitAlso.checked = true;
      document.getElementById('fitAlsoText')!.textContent = `auch in ${others.slice(0, 2).map((p) => `„${p.name}“`).join(', ')}${others.length > 2 ? ` und ${others.length - 2} weiteren` : ''} (dieselbe ${f.hole ? 'Bohrung' : 'Rundung'})`;
    }
    showFitWall();
    const d = 2 * wall.radius;
    document.getElementById('fitTitle')!.textContent = `${f.hole ? 'Bohrung' : 'Zapfen'} ⌀ ${mm2(d)} mm · ${h.part.name}`;
    document.getElementById('fitLabel')!.textContent = 'Neuer Durchmesser';
    fitValue.value = d.toFixed(2);
    // Zapfen dünner, Bohrung weiter
    const sign = f.hole ? 1 : -1;
    // Mein Spiel gilt je Seite: der Durchmesser ändert sich um das Doppelte
    const mine = 2 * myClearance();
    quick.innerHTML = `<button class="ghost small" data-fit="${(d + sign * mine).toFixed(2)}" title="Spiel für ${esc(clearanceFor())} (Toleranz-Test): ${mm2(myClearance())} mm je Seite">Mein Spiel ${sign > 0 ? '+' : '−'}${mm2(mine).replace(/0$/, '')}</button>`
      + [0.1, 0.2, 0.3].map((x) => `<button class="ghost small" data-fit="${(d + sign * x).toFixed(2)}">${sign > 0 ? '+' : '−'}${mm2(x).replace(/0$/, '')}</button>`).join('');
    help.textContent = f.hole
      ? 'Die Bohrung wird weiter – für ein Gegenstück, das sich später hineinstecken lässt.'
      : 'Der Zapfen wird dünner – üblich sind 0,1–0,3 mm, damit getrennt gedruckte Teile ineinanderpassen.';
  } else {
    document.getElementById('fitTitle')!.textContent = `Ebene Fläche · ${h.part.name}`;
    document.getElementById('fitLabel')!.textContent = 'Zurücksetzen um';
    fitValue.value = myClearance().toFixed(2);
    quick.innerHTML = `<button class="ghost small" data-fit="${myClearance().toFixed(2)}" title="Spiel für ${esc(clearanceFor())} (Toleranz-Test)">Mein Spiel ${mm2(myClearance()).replace(/0$/, '')}</button>`
      + [0.1, 0.2, 0.3].map((x) => `<button class="ghost small" data-fit="${x.toFixed(2)}">${mm2(x).replace(/0$/, '')}</button>`).join('');
    help.textContent = 'Die Fläche wandert ins Teil hinein und lässt so viel Spalt zum Gegenstück (negativ: nach außen).';
  }
  fitBar.hidden = false;
  fitValue.focus();
  fitValue.select();
  renderHint();
}

document.getElementById('fitQuick')!.addEventListener('click', (e) => {
  const v = (e.target as HTMLElement).closest<HTMLElement>('[data-fit]')?.dataset.fit;
  if (v) fitValue.value = v;
});
document.getElementById('fitCancel')!.addEventListener('click', () => setTool(null));
document.getElementById('fitApply')!.addEventListener('click', () => applyFit());
fitValue.addEventListener('keydown', (e) => {
  if (e.key === 'Enter') applyFit();
  if (e.key === 'Escape') setTool(null);
});

function applyFit() {
  if (!fitTarget) return;
  const { part, f } = fitTarget;
  const v = Number(fitValue.value.replace(',', '.'));
  if (!Number.isFinite(v)) { toast('Bitte eine Zahl eingeben.'); return; }
  const topo = topologyOf(part);
  let res;
  let text: string;
  if (f.kind === 'cylinder') {
    const wall = fitTarget.wall ?? { center: f.center, axis: f.axis, radius: f.radius, span: undefined, pieces: [{ part, tris: f.tris }] };
    const d = 2 * wall.radius;
    if (!(v > 0) || Math.abs(v - d) > Math.max(2, d * 0.3)) { toast(`Der neue Durchmesser sollte nahe bei ${mm2(d)} mm liegen.`); return; }
    if (Math.abs(v - d) < 0.005) { setTool(null); return; }
    // Jedes beteiligte Teil in seinen eigenen Koordinaten
    const changes: { p: Part; res: ReturnType<typeof offsetCylinder>; area: number }[] = [];
    for (const { part: p, tris } of wall.pieces) {
      if (p !== part && !fitAlso.checked) continue;
      const inv = p.mesh.matrixWorld.clone().invert();
      const center = v3(new THREE.Vector3(...wall.center).applyMatrix4(inv));
      const axis = v3(new THREE.Vector3(...wall.axis).transformDirection(inv));
      const t = topologyOf(p);
      // Alle Eckpunkte auf der Wand wandern mit (auch Ränder, die nur zu Nachbardreiecken gehören)
      const onWall = wall.span && Number.isFinite(wall.span[0]) ? { tol: Math.max(0.01, 0.003 * wall.radius), span: wall.span } : undefined;
      changes.push({ p, res: offsetCylinder(t, { ...f, center, axis, radius: wall.radius, tris }, v / 2, onWall), area: tris.reduce((s, x) => s + t.areas[x], 0) });
    }
    if (changes.some((c) => c.res.flippedArea > Math.max(0.5, c.area * 0.02))) {
      toast('So viel geht nicht: angrenzende Flächen würden umklappen. Bitte einen kleineren Wert.', 6000);
      return;
    }
    beginAction();
    for (const c of changes) replaceGeometry(c.p, c.res.positions);
    setTool(null);
    solveAll();
    renderAll();
    if (checkOn) void runCheck();
    const more = changes.filter((c) => c.p !== part).map((c) => `„${c.p.name}“`);
    toast(`${f.hole ? 'Bohrung' : 'Zapfen'} jetzt ⌀ ${mm2(v)} mm (vorher ${mm2(d)} mm)${more.length ? ` – auch in ${more.join(', ')}` : ''}. Strg+Z macht es rückgängig.`, 6000);
    return;
  } else {
    if (Math.abs(v) > 3) { toast('Mehr als 3 mm ist keine Passung mehr – bitte einen kleineren Wert.'); return; }
    if (Math.abs(v) < 0.005) { setTool(null); return; }
    res = offsetPlane(topo, f, -v);
    text = v > 0 ? `Fläche um ${mm2(v)} mm zurückgesetzt.` : `Fläche um ${mm2(-v)} mm vorgezogen.`;
  }
  // Kippen Nachbarflächen um, wäre das Teil kaputt – winzige Splitter einer Deckel-Triangulierung sind harmlos
  const area = f.area;
  if (res.flippedArea > Math.max(0.5, area * 0.02)) {
    toast('So viel geht nicht: angrenzende Flächen würden umklappen. Bitte einen kleineren Wert.', 6000);
    return;
  }
  beginAction();
  replaceGeometry(part, res.positions);
  setTool(null);
  solveAll();
  renderAll();
  if (checkOn) void runCheck();
  toast(`${text} Strg+Z macht es rückgängig.`, 6000);
}

/**
 * Teil mit neuer Form (gleiche Dreiecke, andere Eckpunkte): ein neues Teil-Objekt mit derselben Id an
 * seiner Stelle. Das alte bleibt für „Rückgängig" erhalten; Verbindungen bleiben, wo sie sind.
 */
function replaceGeometry(old: Part, positions: Float32Array, extras?: Partial<Pick<Part, 'paint' | 'palette' | 'modifiers'>>): Part {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  g.computeVertexNormals();
  g.computeBoundingBox();
  g.computeBoundingSphere();
  const fresh = addPart(old.name, g, {
    id: old.id, color: `#${old.color.getHexString()}`, group: old.group, layout: old.layout,
    extras: { paint: old.paint, palette: old.palette, modifiers: old.modifiers, settings: old.settings, objectSettings: old.objectSettings, ...extras },
  });
  fresh.visible = old.visible;
  fresh.mesh.visible = old.visible;
  if (old.fromFile) fresh.fromFile = true;
  fresh.together = old.together;
  parts.splice(parts.indexOf(fresh), 1);
  parts.splice(parts.indexOf(old), 1, fresh);
  scene.remove(old.mesh);
  removedParts.add(old);
  if (gizmoTarget) closeGizmo();
  // Zwischengespeicherte Netze der Gegenstücke neu aufbauen
  for (const j of joints) mergedTargets.delete(j);
  return fresh;
}

// ---------------------------------------------------------------- Montageanleitung

/**
 * Montageanleitung zum Ausdrucken (oder als PDF): ein Bild der Teile – auf Wunsch auseinandergezogen wie in
 * der Explosionsansicht, aus der Blickrichtung des 3D-Fensters – mit nummerierten Ballons, dazu die
 * Stückliste (gleiche Teile zusammengefasst) und die Schritte aus den Verbindungen.
 */
const sheetDialog = document.getElementById('sheet') as HTMLDialogElement;
const sheetExplode = document.getElementById('sheetExplode') as HTMLInputElement;
const sheetTitle = document.getElementById('sheetTitle') as HTMLInputElement;
const sheetImg = document.getElementById('sheetImg') as HTMLImageElement;
let sheetCanvas: HTMLCanvasElement | null = null;

/** Teilearten: gleiche Teile bekommen dieselbe Nummer. */
function sheetTypes(shown: Part[]): Part[][] {
  return groupIdentical(shown.map((p) => {
    const { m, dims } = massOf(p);
    return { tris: positionsOf(p).length / 9, volume: m.volume, dims };
  })).map((g) => g.map((k) => shown[k]));
}

/**
 * Bild der Teile (weißer Grund) mit Ballons. Die Teile werden nur für dieses Bild auseinandergezogen und
 * gleich wieder zurückgesetzt; der Ankerpunkt jedes Ballons liegt auf einer sichtbaren Stelle des Teils.
 */
function sheetImage(shown: Part[], types: Part[][], exploded: boolean, W = 1400, H = 880): HTMLCanvasElement {
  const noOf = new Map<Part, number>();
  types.forEach((t, i) => t.forEach((p) => noOf.set(p, i + 1)));
  // Weiter auseinander als in der Ansicht: Zapfen und Bohrungen sollen frei liegen
  const offsets = exploded ? explodeOffsets(1.3) : new Map<Part, THREE.Vector3>();
  const move = (sign: number) => {
    for (const [p, d] of offsets) {
      p.mesh.matrix.premultiply(new THREE.Matrix4().makeTranslation(sign * d.x, sign * d.y, sign * d.z));
      p.mesh.matrixWorldNeedsUpdate = true;
      p.mesh.updateMatrixWorld(true);
    }
  };
  const S = 2;
  const pixels = new Uint8Array(W * S * H * S * 4);
  const anchors: { part: Part; x: number; y: number }[] = [];
  const hidden: THREE.Object3D[] = [];
  const clip = renderer.localClippingEnabled;
  const clear = renderer.getClearColor(new THREE.Color());
  const alpha = renderer.getClearAlpha();
  const rt = new THREE.WebGLRenderTarget(W * S, H * S, { colorSpace: THREE.SRGBColorSpace, stencilBuffer: true });
  const cam = new THREE.PerspectiveCamera(30, W / H, 0.1, 10);
  move(1);
  try {
    const box = new THREE.Box3();
    for (const p of shown) box.union(new THREE.Box3().setFromObject(p.mesh));
    const c = box.getCenter(new THREE.Vector3());
    const r = Math.max(1, box.getSize(new THREE.Vector3()).length() / 2);
    cam.near = r / 50;
    cam.far = r * 50;
    // Blickrichtung und „oben“ wie im 3D-Fenster
    const dir = camera.position.clone().sub(controls.target).normalize();
    cam.up.copy(new THREE.Vector3(0, 1, 0).applyQuaternion(camera.quaternion));
    cam.position.copy(c).addScaledVector(dir, (r / Math.sin(THREE.MathUtils.degToRad(15))) * 1.02);
    cam.lookAt(c);
    cam.updateProjectionMatrix();
    cam.updateMatrixWorld();
    // Ausschnitt eng um die Teile, mit Rand für die Ballons
    const lo = new THREE.Vector2(Infinity, Infinity), hi = new THREE.Vector2(-Infinity, -Infinity);
    const v = new THREE.Vector3();
    for (const p of shown) {
      const pos = p.mesh.geometry.getAttribute('position');
      const step = Math.max(1, Math.floor(pos.count / 20000));
      for (let i = 0; i < pos.count; i += step) {
        v.fromBufferAttribute(pos, i).applyMatrix4(p.mesh.matrixWorld).project(cam);
        lo.set(Math.min(lo.x, v.x), Math.min(lo.y, -v.y));
        hi.set(Math.max(hi.x, v.x), Math.max(hi.y, -v.y));
      }
    }
    const cw = ((hi.x - lo.x) / 2) * W, ch = ((hi.y - lo.y) / 2) * H;
    const sw = Math.max(cw, (ch * W) / H, 1) / 0.74, sh = (sw * H) / W;
    const cx = (((lo.x + hi.x) / 2 + 1) / 2) * W, cy = (((lo.y + hi.y) / 2 + 1) / 2) * H;
    cam.setViewOffset(W, H, cx - sw / 2, cy - sh / 2, sw, sh);
    cam.add(sun.clone());
    scene.add(cam);
    for (const o of scene.children) {
      if (o === cam || (o as THREE.Light).isLight || parts.some((p) => p.mesh === o)) continue;
      if (o.visible) { o.visible = false; hidden.push(o); }
    }
    for (const p of parts) if (p.mesh.visible && !shown.includes(p)) { p.mesh.visible = false; hidden.push(p.mesh); }
    renderer.localClippingEnabled = false;
    renderer.setRenderTarget(rt);
    renderer.setClearColor(0xffffff, 1);
    renderer.clear();
    renderer.render(scene, cam);
    renderer.readRenderTargetPixels(rt, 0, 0, W * S, H * S, pixels);
    // Anker: sichtbare Stelle möglichst nahe der Mitte des Teils im Bild
    const ray = new THREE.Raycaster() as THREE.Raycaster & { firstHitOnly?: boolean };
    ray.firstHitOnly = true;
    const meshes = shown.map((p) => p.mesh);
    for (const p of shown) {
      const mid = partBox(p).getCenter(new THREE.Vector3()).project(cam);
      const pos = p.mesh.geometry.getAttribute('position');
      const step = Math.max(1, Math.floor(pos.count / 400));
      const cands: THREE.Vector3[] = [];
      for (let i = 0; i < pos.count; i += step) cands.push(new THREE.Vector3().fromBufferAttribute(pos, i).applyMatrix4(p.mesh.matrixWorld).project(cam));
      cands.sort((a, b) => Math.hypot(a.x - mid.x, a.y - mid.y) - Math.hypot(b.x - mid.x, b.y - mid.y));
      let at = mid;
      for (const q of cands.slice(0, 60)) {
        // Etwas zur Mitte hin: nicht genau auf der Kante
        const t = new THREE.Vector2(q.x + (mid.x - q.x) * 0.15, q.y + (mid.y - q.y) * 0.15);
        ray.setFromCamera(t, cam);
        const hit = ray.intersectObjects(meshes, false)[0];
        if (hit?.object === p.mesh) { at = hit.point.clone().project(cam); break; }
      }
      anchors.push({ part: p, x: ((at.x + 1) / 2) * W, y: ((1 - at.y) / 2) * H });
    }
  } finally {
    renderer.setRenderTarget(null);
    renderer.setClearColor(clear, alpha);
    renderer.localClippingEnabled = clip;
    for (const o of hidden) o.visible = true;
    scene.remove(cam);
    rt.dispose();
    move(-1);
  }
  // Zeilen umdrehen, verkleinern
  const big = document.createElement('canvas');
  big.width = W * S;
  big.height = H * S;
  const img = new ImageData(W * S, H * S);
  const row = W * S * 4;
  for (let y = 0; y < H * S; y++) img.data.set(pixels.subarray((H * S - 1 - y) * row, (H * S - y) * row), y * row);
  big.getContext('2d')!.putImageData(img, 0, 0);
  const out = document.createElement('canvas');
  out.width = W;
  out.height = H;
  const g = out.getContext('2d')!;
  g.imageSmoothingQuality = 'high';
  g.drawImage(big, 0, 0, W, H);
  // Ballons: je Teil einer – bei mehr als drei gleichen nur einer mit „× Anzahl“
  const R = 17;
  const list = anchors.filter((a) => {
    const t = types[noOf.get(a.part)! - 1];
    return t.length <= 3 || t[0] === a.part;
  });
  const pts = layoutBalloons(list, { x: W / 2, y: H / 2 }, W, H, R);
  g.lineWidth = 1.6;
  g.strokeStyle = '#333';
  g.fillStyle = '#333';
  list.forEach((a, i) => {
    g.beginPath();
    g.moveTo(a.x, a.y);
    g.lineTo(pts[i].x, pts[i].y);
    g.stroke();
    g.beginPath();
    g.arc(a.x, a.y, 3, 0, 2 * Math.PI);
    g.fill();
  });
  list.forEach((a, i) => {
    const no = noOf.get(a.part)!;
    const n = types[no - 1].length;
    g.beginPath();
    g.arc(pts[i].x, pts[i].y, R, 0, 2 * Math.PI);
    g.fillStyle = '#fff';
    g.fill();
    g.lineWidth = 2.2;
    g.strokeStyle = '#222';
    g.stroke();
    g.fillStyle = '#111';
    g.font = `700 ${no > 99 ? 13 : 16}px system-ui, sans-serif`;
    g.textAlign = 'center';
    g.textBaseline = 'middle';
    g.fillText(String(no), pts[i].x, pts[i].y + 1);
    if (n > 3) {
      g.font = '600 13px system-ui, sans-serif';
      g.textAlign = 'left';
      const label = `× ${n}`;
      const tw = g.measureText(label).width;
      g.fillStyle = '#fff';
      g.fillRect(pts[i].x + R + 2, pts[i].y - 9, tw + 6, 18);
      g.fillStyle = '#111';
      g.fillText(label, pts[i].x + R + 5, pts[i].y + 1);
    }
  });
  return out;
}

/** Schritte aus den Verbindungen: vom Basisteil aus in der Reihenfolge, in der die Teile angesetzt werden. */
function sheetSteps(shown: Part[], noOf: Map<Part, number>): string[] {
  const tag = (p: Part) => `<span class="no">${noOf.get(p) ?? '?'}</span> ${esc(p.name)}`;
  const steps: string[] = [];
  const done = new Set<number>();
  const used = joints.filter((j) => j.applied && j.fixedSide && j.a !== null && j.b !== null && j.status?.kind !== 'err');
  for (const seed of [baseId, ...parts.map((p) => p.id)]) {
    if (seed === null || done.has(seed)) continue;
    done.add(seed);
    const queue = [seed];
    for (let q = 0; q < queue.length; q++) {
      const cur = queue[q];
      for (const j of used) {
        const fixed = j.fixedSide === 'a' ? j.a : j.b;
        const moving = (j.fixedSide === 'a' ? j.b : j.a)!;
        if (fixed !== cur || done.has(moving)) continue;
        done.add(moving);
        queue.push(moving);
        const pm = getPart(moving), pf = getPart(cur);
        if (!pm || !pf || !shown.includes(pm) || !shown.includes(pf)) continue;
        const fm = (j.fixedSide === 'a' ? j.fb : j.fa)[0];
        const ff = (j.fixedSide === 'a' ? j.fa : j.fb)[0];
        // Zapfen in eine Bohrung, Bohrung auf einen Zapfen, sonst Fläche an Fläche
        const pin = (f: Feature | undefined, hole: boolean) => f?.kind === 'cylinder' && f.hole === hole;
        const word = pin(fm, false) && pin(ff, true) ? 'in' : pin(fm, true) && pin(ff, false) ? 'auf' : 'an';
        const how = fm && ff ? ` – ${describeFeature(fm)} ${word} ${describeFeature(ff)}` : ' – an den markierten Stellen';
        steps.push(`${tag(pm)} ${word} ${tag(pf)} setzen${how}.`);
      }
    }
  }
  // Zerschnittene / zerteilte Teile: Dübel bzw. Schnappstifte und die Stücke
  const kinds = [['Dübel', 'Dübel in die Bohrungen stecken (bei Bedarf mit etwas Leim)'], ['Schnappstift', 'Schnappstifte in die Bohrungen drücken, bis sie einrasten']] as const;
  for (const [word, text] of kinds) {
    const list = shown.filter((p) => new RegExp(` ${word} \\d+$`).test(p.name));
    if (!list.length) continue;
    const nos = [...new Set(list.map((p) => noOf.get(p)!))];
    steps.push(`${text}: ${nos.map((n) => `<span class="no">${n}</span>`).join(' ')} (${list.length}×).`);
  }
  const pieces = shown.filter((p) => /\s\d+\/\d+$/.test(p.name) || /\((oben|unten)\)$/.test(p.name));
  if (pieces.length > 1) steps.push(`Die Stücke ${pieces.map((p) => tag(p)).join(', ')} zusammenstecken – so, wie sie im Ganzen lagen.`);
  return steps;
}

function renderSheet() {
  const shown = parts.filter((p) => p.visible);
  if (!shown.length) return;
  const types = sheetTypes(shown);
  const noOf = new Map<Part, number>();
  types.forEach((t, i) => t.forEach((p) => noOf.set(p, i + 1)));
  sheetCanvas = sheetImage(shown, types, sheetExplode.checked);
  sheetImg.src = sheetCanvas.toDataURL('image/png');
  const fil = filamentList();
  const rows = types.map((t, i) => {
    const p = t[0];
    const s = partBox(p).getSize(new THREE.Vector3());
    const e = estimateOf([p]);
    const color = hexOf(p);
    const slot = fil.indexOf(color) + 1;
    const names = [...new Set(t.map((q) => q.name))];
    return `<tr><td><span class="no">${i + 1}</span></td><td class="num">${t.length}×</td>`
      + `<td>${esc(names[0])}${names.length > 1 ? ` <small>u. a. ${esc(names.slice(1, 3).join(', '))}${names.length > 3 ? ' …' : ''}</small>` : ''}</td>`
      + `<td class="num">${mmText1(s.x)} × ${mmText1(s.y)} × ${mmText1(s.z)}</td>`
      + `<td><span class="dot" style="--c:${color}"></span> ${slot ? `Filament ${slot}` : ''}${p.paint?.paint_color?.some(Boolean) ? ' + Bemalung' : ''}</td>`
      + `<td class="num">${gramText(e.grams * t.length)} g</td></tr>`;
  });
  const all = estimateOf(shown);
  document.getElementById('sheetParts')!.innerHTML = `<thead><tr><th>Nr.</th><th class="num">Anzahl</th><th>Teil</th><th class="num">Maße (mm, wie gelegt)</th><th>Filament</th><th class="num">≈ Gewicht</th></tr></thead><tbody>${rows.join('')}</tbody>`;
  document.getElementById('sheetMeta')!.textContent = [
    new Date().toLocaleDateString('de-DE'),
    `${shown.length} Teil${shown.length === 1 ? '' : 'e'} (${types.length} verschiedene)`,
    `≈ ${gramText(all.grams)} g ${material().label}`,
    printerName(currentPrinter()),
  ].join(' · ');
  const steps = sheetSteps(shown, noOf);
  document.getElementById('sheetSteps')!.innerHTML = steps.length
    ? steps.map((s) => `<li>${s}</li>`).join('')
    : '<li>Keine Verbindungen angelegt – die Lage der Teile zeigt das Bild.</li>';
}

function openSheet() {
  if (!parts.some((p) => p.visible)) { toast('Keine sichtbaren Teile.'); return; }
  if (!sheetTitle.value.trim()) sheetTitle.value = projectName();
  sheetExplode.disabled = !explodeOffsets(0.8).size;
  if (sheetExplode.disabled) sheetExplode.checked = false;
  if (!sheetDialog.open) sheetDialog.showModal();
  renderSheet();
}

/** Drucken: nur die Seite (der Titel als Text), danach wieder der Dialog. */
function printSheet() {
  const area = document.getElementById('printArea')!;
  const page = document.getElementById('sheetPage')!.cloneNode(true) as HTMLElement;
  page.removeAttribute('id');
  const h = document.createElement('h1');
  h.className = 'sheet-title';
  h.textContent = sheetTitle.value;
  page.querySelector('.sheet-title')!.replaceWith(h);
  area.replaceChildren(page);
  document.body.classList.add('print-sheet');
  sheetDialog.close();
  const done = () => {
    window.removeEventListener('afterprint', done);
    document.body.classList.remove('print-sheet');
    area.replaceChildren();
    if (!sheetDialog.open) sheetDialog.showModal();
  };
  window.addEventListener('afterprint', done);
  const title = document.title;
  document.title = sheetTitle.value || title;
  window.print();
  document.title = title;
}

document.getElementById('sheetBtn')!.addEventListener('click', openSheet);
sheetExplode.addEventListener('change', renderSheet);
document.getElementById('sheetPrint')!.addEventListener('click', printSheet);
document.getElementById('sheetPng')!.addEventListener('click', () => {
  const name = sheetTitle.value.replace(/[\\/:*?"<>|–—]+/g, ' ').replace(/\s+/g, ' ').trim() || projectName();
  sheetCanvas?.toBlob((b) => { if (b) void download(b, `${name} Montage.png`); }, 'image/png');
});
document.getElementById('sheetClose')!.addEventListener('click', () => sheetDialog.close());
sheetDialog.addEventListener('click', (e) => {
  if (e.target === sheetDialog) sheetDialog.close();
});

// ---------------------------------------------------------------- Hilfe

const helpDialog = document.getElementById('help') as HTMLDialogElement;
function openHelp() {
  if (!helpDialog.open) helpDialog.showModal();
}
document.getElementById('helpBtn')!.addEventListener('click', openHelp);
document.getElementById('helpClose')!.addEventListener('click', () => helpDialog.close());
// Klick neben das Fenster schließt es
helpDialog.addEventListener('click', (e) => {
  if (e.target === helpDialog) helpDialog.close();
});

// ---------------------------------------------------------------- Projekt speichern / öffnen

/** Schlüssel der Netze eines Teils in Projektdatei und Sitzungssicherung (bleibt für das Teil gleich). */
const geomKeys = new WeakMap<Part, string>();
let geomCounter = 0;
function geomKey(p: Part): string {
  let k = geomKeys.get(p);
  if (!k) {
    k = `${Date.now().toString(36)}-${(geomCounter++).toString(36)}-${Math.random().toString(36).slice(2, 8)}`;
    geomKeys.set(p, k);
  }
  return k;
}

/** Der ganze Arbeitsstand als Projekt; die Netze getrennt (sie ändern sich nie). */
function projectState(): { data: ProjectData; blobs: Blobs } {
  const blobs: Blobs = new Map();
  const list = parts.map((p) => {
    const key = geomKey(p);
    blobs.set(key, positionsOf(p));
    return {
      id: p.id, name: p.name, color: `#${p.color.getHexString()}`, group: p.group, layout: p.layout.toArray(),
      visible: p.visible, fromFile: p.fromFile, together: p.together, geometry: key,
      paint: p.paint, palette: p.palette, settings: p.settings, objectSettings: p.objectSettings,
      modifiers: p.modifiers?.map(({ positions, local, ...m }, i) => {
        blobs.set(`${key}-m${i}`, positions);
        if (local) blobs.set(`${key}-l${i}`, local);
        return { ...m, geometry: `${key}-m${i}`, localGeometry: local ? `${key}-l${i}` : undefined };
      }),
    };
  });
  // Nur, was der Benutzer festgelegt hat – Ergebnisse rechnet solveAll beim Öffnen neu
  const saved = joints.map((j) => ({
    id: j.id, color: `#${j.color.getHexString()}`, a: j.a, b: j.b, ma: j.ma, mb: j.mb, fa: j.fa, fb: j.fb,
    ordered: j.ordered, normals: j.normals, overlap: j.overlap, spin: j.spin, seat: j.seat, applied: j.applied, alt: j.alt,
    adjust: j.adjust, adjustInfo: j.adjustInfo, last: j.last, turnIdx: j.turnIdx, auto: j.auto,
  }));
  return {
    data: newProject({
      parts: list, joints: saved, baseId, activeJointId,
      settings: { onFloor: onFloor.checked, printer: bambuPrinter.value, plates: plateTotal },
      camera: { position: camera.position.toArray(), target: controls.target.toArray() },
    }),
    blobs,
  };
}

/** Ersetzt die aktuelle Arbeit durch ein Projekt (die vorige bleibt in der Historie: Strg+Z). */
function openProject(data: ProjectData, blobs: Blobs) {
  closeGizmo();
  for (const p of parts) {
    scene.remove(p.mesh);
    removedParts.add(p);
  }
  parts.length = 0;
  joints.length = 0;
  mergePick.clear();
  baseId = null;
  activeJointId = null;
  // Ids werden neu vergeben, Verweise in den Verbindungen entsprechend umgestellt
  const ids = new Map<number, number>();
  for (const sp of data.parts) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(blobs.get(sp.geometry)!, 3));
    g.computeVertexNormals();
    g.computeBoundingBox();
    g.computeBoundingSphere();
    const modifiers = sp.modifiers?.map(({ geometry, localGeometry, ...m }) => ({
      ...m, positions: blobs.get(geometry)!, ...(localGeometry ? { local: blobs.get(localGeometry) } : {}),
    }));
    const p = addPart(sp.name, g, {
      color: sp.color, group: sp.group, layout: new THREE.Matrix4().fromArray(sp.layout),
      extras: { paint: sp.paint, palette: sp.palette, modifiers, settings: sp.settings, objectSettings: sp.objectSettings },
    });
    p.visible = sp.visible !== false;
    p.mesh.visible = p.visible;
    if (sp.fromFile) p.fromFile = true;
    if (sp.together) p.together = sp.together;
    geomKeys.set(p, sp.geometry);
    ids.set(sp.id, p.id);
  }
  const part = (id: unknown) => (typeof id === 'number' ? ids.get(id) ?? null : null);
  const feature = (f: PickedFeature): PickedFeature => {
    const out = { ...f } as PickedFeature;
    if (f.on !== undefined) out.on = part(f.on) ?? undefined;
    if (f.pieces) out.pieces = f.pieces.map(feature);
    return out;
  };
  const jointIds = new Map<number, number>();
  for (const raw of data.joints) {
    const sj = raw as unknown as Joint & { color: string };
    const j: Joint = {
      ...sj, id: nextJointId++, color: new THREE.Color(sj.color), a: part(sj.a), b: part(sj.b),
      ma: sj.ma ?? [], mb: sj.mb ?? [], fa: (sj.fa ?? []).map(feature), fb: (sj.fb ?? []).map(feature),
    };
    jointIds.set(sj.id, j.id);
    joints.push(j);
  }
  baseId = part(data.baseId) ?? parts[0]?.id ?? null;
  activeJointId = (data.activeJointId !== null ? jointIds.get(data.activeJointId) : undefined) ?? joints[0]?.id ?? null;
  onFloor.checked = data.settings?.onFloor ?? true;
  const printer = data.settings?.printer;
  if (printer && printerById(printer)) setBambuPrinter(printer);
  ensureJoint();
  solveAll();
  plateTotal = data.settings?.plates ?? guessPlates();
  renderAll();
  if (data.camera) {
    camera.position.fromArray(data.camera.position);
    controls.target.fromArray(data.camera.target);
    controls.update();
  } else fitView();
}

/** Dateiname aus dem Basisteil, ohne Zeichen, die Dateisysteme nicht mögen. */
const projectName = () => (getPart(baseId)?.name ?? parts[0]?.name ?? 'D3DForge').replace(/[\\/:*?"<>|]+/g, '_').trim() || 'D3DForge';

async function saveProject() {
  if (!parts.length) {
    toast('Noch keine Teile geladen – es gibt nichts zu speichern.');
    return;
  }
  const { data, blobs } = projectState();
  toast('Speichere Projekt …', 20000);
  try {
    const bytes = await compute('packProject', { data, blobs });
    await download(new Blob([bytes as Uint8Array<ArrayBuffer>], { type: 'application/zip' }), `${projectName()}.${PROJECT_EXT}`);
  } catch (e) {
    toast(`Speichern nicht möglich: ${(e as Error).message ?? e}`, 6000);
    return;
  }
  toast('Projekt gespeichert. Zum Weiterarbeiten die Datei hier hineinziehen oder „Projekt öffnen" wählen.', 6000);
}

const projFile = document.getElementById('projFile') as HTMLInputElement;
document.getElementById('projOpen')!.addEventListener('click', () => projFile.click());
projFile.addEventListener('change', () => {
  if (projFile.files?.length) void loadFiles(projFile.files);
  projFile.value = '';
});
document.getElementById('projSave')!.addEventListener('click', () => void exclusive(saveProject));
// Strg/⌘+S speichert das Projekt statt der Webseite
window.addEventListener('keydown', (e) => {
  if (!(e.ctrlKey || e.metaKey) || e.shiftKey || e.altKey || e.key.toLowerCase() !== 's') return;
  e.preventDefault();
  void exclusive(saveProject);
});

// ---------------------------------------------------------------- Sitzung im Browser sichern

/** Erst sichern, wenn über eine vorhandene Sitzung entschieden ist – sonst überschriebe die leere Seite sie. */
let sessionArmed = false;
let sessionSig = '';
let sessionBusy = false;
/** Laufende Sicherung – wer währenddessen sichern will, wartet darauf und sichert danach den neuen Stand. */
let sessionDone: Promise<void> = Promise.resolve();
const restoreBar = document.getElementById('restoreBar')!;

/** Maustaste gedrückt (Ziehen, Drehen der Ansicht): Sicherung wartet, damit nichts ruckelt. */
let pointerHeld = false;
window.addEventListener('pointerdown', () => { pointerHeld = true; }, true);
for (const t of ['pointerup', 'pointercancel', 'blur'] as const) window.addEventListener(t, () => { pointerHeld = false; }, true);

async function autosave(force = false) {
  // Neue Arbeit begonnen, ohne über die alte Sitzung zu entscheiden: die neue gilt
  if (!sessionArmed && !restoreBar.hidden && parts.length) {
    restoreBar.hidden = true;
    sessionArmed = true;
  }
  if (!sessionArmed || (pointerHeld && !force)) return;
  if (sessionBusy) {
    // Nicht überspringen: Der Stand kann sich seit Beginn der laufenden Sicherung geändert haben
    await sessionDone;
    return autosave(force);
  }
  const sig = `${stateSig()}|${onFloor.checked}|${bambuPrinter.value}`;
  if (sig === sessionSig) return;
  sessionBusy = true;
  sessionDone = (async () => {
    try {
      if (!parts.length) await clearSession();
      else {
        const { data, blobs } = projectState();
        await saveSession(data, blobs);
      }
      sessionSig = sig;
    } catch {
      // Kein Speicher im Browser (z. B. privates Fenster oder voll): dann eben ohne Sicherung
      sessionArmed = false;
    } finally {
      sessionBusy = false;
    }
  })();
  await sessionDone;
}
// Regelmäßig – läuft noch eine Sicherung, kommt die nächste Runde dran (nur ausdrückliche Aufrufe warten)
setInterval(() => { if (!sessionBusy) void autosave(); }, 2500);
document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'hidden') void autosave(true);
});

document.getElementById('restoreYes')!.addEventListener('click', async () => {
  restoreBar.hidden = true;
  try {
    const s = await loadSession();
    if (s) {
      openProject(s.data, s.blobs);
      sessionSig = `${stateSig()}|${onFloor.checked}`;
      toast('Letzte Sitzung wiederhergestellt.');
    } else toast('Die gesicherte Sitzung ist unvollständig und lässt sich nicht wiederherstellen.');
  } catch (e) {
    toast(`Wiederherstellen nicht möglich: ${(e as Error).message ?? e}`);
  }
  sessionArmed = true;
});
document.getElementById('restoreNo')!.addEventListener('click', async () => {
  restoreBar.hidden = true;
  try { await clearSession(); } catch { /* ohne Speicher */ }
  sessionArmed = true;
});

void (async () => {
  let info: ProjectData | null = null;
  try {
    info = await sessionInfo();
  } catch {
    return; // kein IndexedDB: keine Sicherung
  }
  if (!info || parts.length) {
    sessionArmed = true;
    return;
  }
  const n = info.parts.length;
  const k = info.joints.filter((j) => j.applied).length;
  const when = new Date(info.savedAt).toLocaleString('de-DE', { dateStyle: 'short', timeStyle: 'short' });
  document.getElementById('restoreText')!.textContent =
    `Letzte Sitzung vom ${when}: ${n} Teil${n === 1 ? '' : 'e'}, ${k} Verbindung${k === 1 ? '' : 'en'}.`;
  restoreBar.hidden = false;
})();

// Für automatisierte Tests / Debugging
// ---------------------------------------------------------------- Farben malen

/**
 * Farbbemalung wie in Bambu Studio (Taste N): Filament wählen und mit dem Pinsel über das Teil streichen
 * oder eine Fläche füllen; Umschalt radiert (Filament des Teils). Gespeichert wird wie in Bambu Studio je
 * Dreieck (am Pinselrand fein unterteilt), der Export ins Bambu-Projekt übernimmt es als Bemalung.
 */
const paintSwatches = document.getElementById('paintSwatches')!;
const paintSize = document.getElementById('paintSize') as HTMLInputElement;
const paintAngle = document.getElementById('paintAngle') as HTMLInputElement;
/** Übliche Bambu-Filamentfarben zum schnellen Hinzufügen. */
const PAINT_PRESETS = ['#ffffff', '#000000', '#c12e1f', '#0a2989', '#f4ee2a', '#00ae42', '#ff6a13', '#8e9089'];
/** Vom Benutzer hinzugefügte Farben, die noch nirgends verwendet sind. */
const paintExtra: string[] = [];
let paintColor = '';
let paintMode: 'brush' | 'fill' = 'brush';
/** Radieren ohne Umschalttaste (Touch). */
let paintErase = false;
const paintCursor = new THREE.Mesh(new THREE.RingGeometry(0.9, 1, 48), new THREE.MeshBasicMaterial({ color: 0xffffff, depthTest: false, transparent: true, opacity: 0.9, side: THREE.DoubleSide }));
paintCursor.renderOrder = 13;
paintCursor.visible = false;
paintCursor.raycast = () => {};
scene.add(paintCursor);

const hexOf = (p: Part) => `#${p.color.getHexString()}`;

/** Filamente in der Reihenfolge des Exports (Teilefarben, bemalte Farben), dazu neu hinzugefügte. */
function filamentList(): string[] {
  const list = colorSlots(parts.filter((p) => p.visible).map((p) => ({ ...p, color: hexOf(p), positions: new Float32Array(0) } as unknown as ColoredPart)));
  for (const c of paintExtra) if (!list.includes(c)) list.push(c);
  return list;
}

function renderPaintBar() {
  const list = filamentList();
  if (!list.includes(paintColor)) paintColor = list[1] ?? list[0] ?? '';
  paintSwatches.innerHTML = list.map((c, i) => `<button class="swatch${c === paintColor ? ' on' : ''}" data-color="${c}" style="--c:${c}" title="Filament ${i + 1} (${c}) – damit malen">${i + 1}</button>`).join('')
    + `<span class="presets" title="Farbe hinzufügen">+ ${PAINT_PRESETS.filter((c) => !list.includes(c)).map((c) => `<button class="preset" data-add="${c}" style="--c:${c}" title="${c} hinzufügen"></button>`).join('')}<input type="color" id="paintPick" value="#3366ff" title="Eigene Farbe hinzufügen" /></span>`;
  paintBar.querySelectorAll<HTMLElement>('[data-pmode]').forEach((b) => b.classList.toggle('on', b.dataset.pmode === paintMode));
  document.getElementById('paintSizeWrap')!.hidden = paintMode !== 'brush';
  document.getElementById('paintAngleWrap')!.hidden = paintMode !== 'fill';
  document.getElementById('paintSizeVal')!.textContent = `${mmText1(Number(paintSize.value))} mm`;
  document.getElementById('paintInfo')!.textContent = list.length > 4 ? `${list.length} Filamente – mehr als viele Mehrfarbsysteme fassen (meist 4 Plätze)` : `${list.length} Filament${list.length === 1 ? '' : 'e'}`;
}

function openPaint() {
  paintBar.hidden = false;
  renderPaintBar();
}
function closePaint() {
  paintBar.hidden = true;
  paintCursor.visible = false;
}

paintSwatches.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('[data-color], [data-add]');
  if (!b) return;
  const c = (b.dataset.color ?? b.dataset.add)!.toLowerCase();
  if (b.dataset.add && !paintExtra.includes(c)) paintExtra.push(c);
  paintColor = c;
  renderPaintBar();
});
paintSwatches.addEventListener('change', (e) => {
  const input = e.target as HTMLInputElement;
  if (input.id !== 'paintPick') return;
  const c = input.value.toLowerCase();
  if (!paintExtra.includes(c)) paintExtra.push(c);
  paintColor = c;
  renderPaintBar();
});
paintBar.querySelectorAll<HTMLElement>('[data-pmode]').forEach((b) => b.addEventListener('click', () => {
  paintMode = b.dataset.pmode as typeof paintMode;
  renderPaintBar();
  renderHint();
}));
paintSize.addEventListener('input', () => renderPaintBar());
document.getElementById('paintErase')!.addEventListener('click', (e) => {
  paintErase = !paintErase;
  (e.currentTarget as HTMLElement).classList.toggle('on', paintErase);
  (e.currentTarget as HTMLElement).setAttribute('aria-pressed', String(paintErase));
});
document.getElementById('paintClose')!.addEventListener('click', () => setTool(null));
paintBtn.addEventListener('click', () => setTool(tool === 'paint' ? null : 'paint'));

/** Zustand (Filamentnummer im Teil) für eine Farbe; die Farbe des Teils ist 0. Neue Farben kommen in die Palette. */
function paintStateFor(p: Part, palette: string[], color: string): number {
  if (!color || color === hexOf(p)) return 0;
  const i = palette.findIndex((c) => c.toLowerCase() === color);
  if (i >= 0) return i + 1;
  palette.push(color);
  return palette.length;
}

/** Laufender Strich: Bemalung und Palette werden erst am Ende ins Teil geschrieben (eine Rückgängig-Stufe). */
let paintStroke: { id: number; part: Part; codes: string[]; palette: string[]; state: number; last: Vec3 | null; changed: boolean } | null = null;
const paintDirty = new Set<Part>();
let paintFrame = 0;
/** Bemalung im Bild erneuern – höchstens einmal je Bild. */
function schedulePaint(p: Part) {
  paintDirty.add(p);
  if (paintFrame) return;
  paintFrame = requestAnimationFrame(() => {
    paintFrame = 0;
    for (const q of paintDirty) {
      const extras = paintStroke?.part === q ? { paint: { ...q.paint, paint_color: paintStroke.codes }, palette: paintStroke.palette } : q;
      refreshPaint(q.mesh, extras);
    }
    paintDirty.clear();
  });
}

/** Ein Tupfer des Pinsels (Kugel um p, Koordinaten des Teils) – nur auf Dreiecken, die zur Kamera zeigen. */
function paintDab(p: Vec3, tri: number) {
  const s = paintStroke!;
  const topo = topologyOf(s.part);
  const R = Math.max(0.05, Number(paintSize.value) / 2);
  const cam = camera.position.clone().applyMatrix4(s.part.mesh.matrixWorld.clone().invert());
  const pos = topo.positions;
  const V = (t: number, k: number): Vec3 => [pos[9 * t + 3 * k], pos[9 * t + 3 * k + 1], pos[9 * t + 3 * k + 2]];
  const stack = [tri];
  const seen = new Set(stack);
  // Rand fein genug, dass er rund wirkt (wie in Bambu Studio), aber nicht feiner als nötig
  const opts = { center: p, radius: R, state: s.state, minEdge: Math.max(R / 10, 0.03) };
  while (stack.length && seen.size < 200000) {
    const t = stack.pop()!;
    if (triangleDistance(p, V(t, 0), V(t, 1), V(t, 2)) > R) continue;
    const n = [topo.normals[3 * t], topo.normals[3 * t + 1], topo.normals[3 * t + 2]];
    const c = [topo.centroids[3 * t], topo.centroids[3 * t + 1], topo.centroids[3 * t + 2]];
    const facing = n[0] * (cam.x - c[0]) + n[1] * (cam.y - c[1]) + n[2] * (cam.z - c[2]) > 0;
    if (facing || t === tri) {
      const code = brushPaint(s.codes[t] ?? '', V(t, 0), V(t, 1), V(t, 2), opts);
      if (code !== (s.codes[t] ?? '')) { s.codes[t] = code; s.changed = true; }
    }
    for (let k = 0; k < 3; k++) {
      const nb = topo.neighbors[3 * t + k];
      if (nb >= 0 && !seen.has(nb)) { seen.add(nb); stack.push(nb); }
    }
  }
  schedulePaint(s.part);
}

/** Pinsel von der letzten Stelle bis hierher: Tupfer im Abstand eines halben Radius (keine Lücken bei schnellen Strichen). */
function paintTo(h: Hit) {
  const s = paintStroke!;
  const R = Math.max(0.05, Number(paintSize.value) / 2);
  const from = s.last;
  s.last = h.raw;
  if (!from) { paintDab(h.raw, h.tri); return; }
  const d = Math.hypot(h.raw[0] - from[0], h.raw[1] - from[1], h.raw[2] - from[2]);
  const steps = Math.min(40, Math.ceil(d / (R / 2)));
  for (let i = 1; i <= steps; i++) {
    const f = i / steps;
    // Zwischenpunkte gehören meist zu Nachbardreiecken; die Suche beginnt am getroffenen Dreieck
    paintDab([from[0] + f * (h.raw[0] - from[0]), from[1] + f * (h.raw[1] - from[1]), from[2] + f * (h.raw[2] - from[2])], h.tri);
  }
}

/** Füllen: zusammenhängende Dreiecke, solange der Knick zum Nachbarn kleiner als der Winkel ist. */
function paintFill(h: Hit) {
  const s = paintStroke!;
  const topo = topologyOf(s.part);
  const cos = Math.cos((Math.max(0, Math.min(90, Number(paintAngle.value) || 30)) * Math.PI) / 180);
  const code = solidPaint(s.state);
  const stack = [h.tri];
  const seen = new Set(stack);
  while (stack.length) {
    const t = stack.pop()!;
    if ((s.codes[t] ?? '') !== code) { s.codes[t] = code; s.changed = true; }
    for (let k = 0; k < 3; k++) {
      const nb = topo.neighbors[3 * t + k];
      if (nb < 0 || seen.has(nb)) continue;
      const d = topo.normals[3 * t] * topo.normals[3 * nb] + topo.normals[3 * t + 1] * topo.normals[3 * nb + 1] + topo.normals[3 * t + 2] * topo.normals[3 * nb + 2];
      if (d < cos) continue;
      seen.add(nb);
      stack.push(nb);
    }
  }
  schedulePaint(s.part);
}

renderer.domElement.addEventListener('pointerdown', (e) => {
  if (tool !== 'paint' || e.button !== 0 || e.ctrlKey || e.metaKey || e.altKey) return;
  const h = pick(e);
  // Neben dem Teil: Ansicht drehen wie sonst
  if (!h) return;
  const erase = e.shiftKey || paintErase;
  if (!paintColor && !erase) { toast('Wähle oben ein Filament – oder füge mit „+“ eine Farbe hinzu.'); return; }
  e.stopImmediatePropagation();
  const p = h.part;
  const palette = [...(p.palette ?? [])];
  const state = erase ? 0 : paintStateFor(p, palette, paintColor);
  const n = positionsOf(p).length / 9;
  const codes = p.paint?.paint_color ? [...p.paint.paint_color] : new Array<string>(n).fill('');
  paintStroke = { id: e.pointerId, part: p, codes, palette, state, last: null, changed: false };
  if (paintMode === 'fill') {
    paintFill(h);
    endPaintStroke();
    return;
  }
  renderer.domElement.setPointerCapture(e.pointerId);
  paintTo(h);
}, { capture: true });
renderer.domElement.addEventListener('pointermove', (e) => {
  if (!paintStroke || e.pointerId !== paintStroke.id) return;
  e.stopImmediatePropagation();
  const h = pick(e);
  updatePaintCursor(h, e.shiftKey || paintErase);
  if (h && h.part === paintStroke.part) paintTo(h);
  else paintStroke.last = null;
}, { capture: true });

/** Strich fertig: Bemalung und Palette ins Teil (Rückgängig und Sitzung sehen es). */
function endPaintStroke() {
  const s = paintStroke;
  paintStroke = null;
  if (!s || !s.changed) return;
  const p = s.part;
  // Palette ohne unbenutzte Farben am Ende
  const used = new Set<number>();
  for (const c of s.codes) if (c) paintStates(c, used);
  const palette = s.palette.slice(0, Math.max(0, ...used));
  const painted = s.codes.some((c) => c);
  p.paint = { ...p.paint, paint_color: s.codes };
  if (!painted) delete p.paint.paint_color;
  p.palette = palette.length ? palette : undefined;
  p.paintRev = (p.paintRev ?? 0) + 1;
  refreshPaint(p.mesh, p);
  renderAll();
  if (tool === 'paint') renderPaintBar();
}
const endPaint = (e: PointerEvent) => {
  if (!paintStroke || e.pointerId !== paintStroke.id) return;
  e.stopImmediatePropagation();
  endPaintStroke();
};
renderer.domElement.addEventListener('pointerup', endPaint, { capture: true });
renderer.domElement.addEventListener('pointercancel', endPaint, { capture: true });

/** Pinselkreis auf der Oberfläche unter dem Zeiger (Füllen: kein Kreis). */
function updatePaintCursor(h: Hit | null, erase: boolean) {
  paintCursor.visible = !!h && tool === 'paint' && paintMode === 'brush';
  if (!h || !paintCursor.visible) return;
  const R = Math.max(0.05, Number(paintSize.value) / 2);
  const n = new THREE.Vector3(...h.local.n).transformDirection(h.part.mesh.matrixWorld);
  paintCursor.position.copy(h.world).addScaledVector(n, 0.02);
  paintCursor.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), n);
  paintCursor.scale.setScalar(R);
  (paintCursor.material as THREE.MeshBasicMaterial).color.set(erase ? '#ffffff' : paintColor || '#ffffff');
}

// Alt + Mausrad: Pinselgröße (wie in Bambu Studio)
renderer.domElement.addEventListener('wheel', (e) => {
  if (tool !== 'paint' || !e.altKey) return;
  e.preventDefault();
  e.stopImmediatePropagation();
  const v = Number(paintSize.value) * (e.deltaY > 0 ? 1 / 1.15 : 1.15);
  paintSize.value = String(Math.round(Math.min(Number(paintSize.max), Math.max(Number(paintSize.min), v)) * 2) / 2);
  renderPaintBar();
}, { capture: true, passive: false });

// ---------------------------------------------------------------- Aktive Anzeigen (rechts)

/**
 * Übersicht am rechten Rand: was gerade eingeschaltet ist (Werkzeug, Maße, Überhänge, Standfestigkeit,
 * Passungsprüfung, Schnitt, Explosion) – mit Farblegende und ✕ zum Ausschalten.
 */
const activeBar = document.getElementById('activeBar')!;
const TOOL_LABEL: Record<'lay' | 'fit' | 'measure' | 'pocket' | 'paint' | 'text', string> = { lay: 'Auf Fläche legen', fit: 'Passung', measure: 'Messen', pocket: 'Aussparung', paint: 'Malen', text: 'Beschriften' };
const hex = (c: number) => `#${c.toString(16).padStart(6, '0')}`;
interface ActiveItem { id: string; label: string; title: string; dots?: { color: string; label: string }[]; off: () => void }
let activeList: ActiveItem[] = [];
let activeSig = '';

function activeItems(): ActiveItem[] {
  const out: ActiveItem[] = [];
  if (tool) out.push({ id: 'tool', label: TOOL_LABEL[tool], title: 'Werkzeug beenden (Esc)', off: () => setTool(null) });
  if (dimsOn) {
    out.push({
      id: 'dims', label: 'Maße', title: 'Maße ausblenden (D)', off: () => setDims(false),
      dots: [{ color: DIM_COLORS.x, label: 'X' }, { color: DIM_COLORS.y, label: 'Y' }, { color: DIM_COLORS.z, label: 'Z' }],
    });
  }
  if (anaOn) {
    out.push({
      id: 'ana', label: 'Überhänge & Wände', title: 'Druckprüfung ausblenden (U)', off: () => setAnalysis(false),
      dots: [{ color: '#e5484d', label: 'hängt über' }, { color: '#9b5de5', label: 'zu dünn' }],
    });
  }
  if (standOn) {
    out.push({
      id: 'stand', label: 'Standfestigkeit', title: 'Standfestigkeit ausblenden', off: () => standBtn.click(),
      dots: [{ color: STAND_COLORS.ok, label: 'sicher' }, { color: STAND_COLORS.tight, label: 'knapp' }, { color: STAND_COLORS.tips, label: 'kippt' }],
    });
  }
  if (checkOn) {
    out.push({
      id: 'check', label: 'Passung prüfen', title: 'Prüfung ausblenden', off: () => document.getElementById('checkBtn')!.click(),
      dots: [{ color: hex(CHECK_COLORS.touch), label: 'liegt an' }, { color: hex(CHECK_COLORS.gap), label: 'Spalt' }, { color: hex(CHECK_COLORS.pen), label: 'dringt ein' }],
    });
  }
  if (section) out.push({ id: 'section', label: `Schnitt ${'XYZ'[section.axis]}`, title: 'Schnittansicht beenden (C)', off: () => setSection(false) });
  if (explode > 0 || explodeAnim?.to) out.push({ id: 'explode', label: 'Explosion', title: 'Explosionsansicht beenden (X)', off: () => setExplode(0) });
  return out;
}

/** Jedes Bild: neu aufbauen, wenn sich etwas geändert hat; immer unter die Platten-Leiste. */
function renderActive() {
  const items = activeItems();
  const sig = items.map((i) => `${i.id}:${i.label}`).join('|');
  if (sig !== activeSig) {
    activeSig = sig;
    activeList = items;
    activeBar.hidden = !items.length;
    activeBar.innerHTML = items.map((i, k) => `<div class="achip">${i.dots ? `<span class="dots">${i.dots.map((d) => `<i style="background:${d.color}" title="${esc(d.label)}"></i>`).join('')}</span>` : ''}<span>${esc(i.label)}</span><button class="icon" data-active="${k}" title="${esc(i.title)}" aria-label="${esc(i.title)}">✕</button></div>`).join('');
  }
  if (!items.length) return;
  const v = view.getBoundingClientRect();
  const pb = plateBar.getBoundingClientRect();
  const top = (pb.height ? pb.bottom : v.top + 122) - v.top + 8;
  if (activeBar.style.top !== `${top}px`) activeBar.style.top = `${top}px`;
}
activeBar.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest<HTMLElement>('[data-active]');
  if (b) activeList[Number(b.dataset.active)]?.off();
});

// Spiel für Drucker und Material in die Felder (erst hier: braucht Drucker- und Materialauswahl)
applyMyClearance();
// Exportziel passend zum Drucker (erst hier: braucht die Druckerliste)
renderExportTarget();

Object.assign(window as unknown as Record<string, unknown>, {
  partjoin: {
    parts, joints, solveAll, renderAll, camera, controls, setPickMode, openGizmo, connect, tc, gizmoAnchor, autosave, hasWorker,
    checkCount: () => checkGroup.children.length,
    /** Belegte Netze und Texturen auf der Grafikkarte (für Tests auf Speicherlecks). */
    gpu: () => ({ ...renderer.info.memory }),
    /** Speichergrenze der Historie (für Tests kleiner). */
    setUndoBytes: (n: number) => { undoBytes = n; },
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
// Beim Start: die ganze Platte im Bild
if (!parts.length) viewPlate(0);
