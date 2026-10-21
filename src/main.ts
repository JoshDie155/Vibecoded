import './style.css';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { acceleratedRaycast, computeBoundsTree, disposeBoundsTree, MeshBVH } from 'three-mesh-bvh';
import { type Rigid, type Vec3, compose } from './math/align';
import { type JointResult, type Marker, type NormalMode, applyAdjustments, solveJoint } from './math/match';
import { type Sample, refineICP, sameShape, sampleSurface } from './math/icp';
import { loadModel, SUPPORTED } from './io/load';
import { type ExportPart, download, to3MF, toOBJ, toSTL } from './io/export';
import { makeDemo } from './demo';

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
  /** Ausgangslage nach dem Laden (nebeneinander aufgereiht). */
  layout: THREE.Matrix4;
  visible: boolean;
  /** Oberflächenpunkte (lokal) für Feinausrichtung und Symmetrieprüfung. */
  samples: Sample[];
}

type Side = 'a' | 'b';

interface Joint {
  id: number;
  color: THREE.Color;
  a: number | null;
  b: number | null;
  ma: Marker[];
  mb: Marker[];
  ordered: boolean;
  normals: NormalMode;
  overlap: number;
  spin: number;
  applied: boolean;
  alt: number;
  /** Korrektur aus der Feinausrichtung (Weltkoordinaten), wird nach der Grobausrichtung angewendet. */
  icp?: Rigid;
  icpInfo?: string;
  // Ergebnis der letzten Berechnung
  result?: JointResult;
  fixedSide?: Side;
  status?: { kind: 'ok' | 'warn' | 'err' | 'info'; text: string };
}

const PART_COLORS = ['#8fa7c9', '#d9a066', '#88b98a', '#c58fb8', '#d7c46a', '#7fbfbf', '#c98f8f', '#a3a3d6'];
const JOINT_COLORS = ['#e8423f', '#2f6fde', '#16a34a', '#d97706', '#9333ea', '#0891b2', '#db2777'];

const parts: Part[] = [];
const joints: Joint[] = [];
let baseId: number | null = null;
let activeJointId: number | null = null;
let nextPartId = 1;
let nextJointId = 1;
const history: { joint: number; side: Side }[] = [];
const SNAP_PX = 14;

// ---------------------------------------------------------------- Szene

const view = document.getElementById('view')!;
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
view.prepend(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 10000);
camera.position.set(120, -160, 120);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

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

const cursor = new THREE.Mesh(
  new THREE.SphereGeometry(1, 16, 12),
  new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.85, depthTest: false }),
);
cursor.renderOrder = 10;
cursor.visible = false;
scene.add(cursor);

function applyTheme() {
  const dark = matchMedia('(prefers-color-scheme: dark)').matches &&
    document.documentElement.dataset.theme !== 'light' || document.documentElement.dataset.theme === 'dark';
  const bg = getComputedStyle(document.documentElement).getPropertyValue('--view').trim();
  renderer.setClearColor(new THREE.Color(bg || (dark ? '#161a21' : '#e9ecf1')));
}
applyTheme();
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applyTheme);

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
  cursor.scale.setScalar(4 * wpp);
}

renderer.setAnimationLoop(() => {
  controls.update();
  scaleMarkers();
  renderer.render(scene, camera);
});

// ---------------------------------------------------------------- Hilfsfunktionen

const v3 = (v: THREE.Vector3): Vec3 => [v.x, v.y, v.z];
const getPart = (id: number | null) => parts.find((p) => p.id === id);
const activeJoint = () => joints.find((j) => j.id === activeJointId);
const esc = (s: string) => s.replace(/[<>&"]/g, (c) => ({ '<': '&lt;', '>': '&gt;', '&': '&amp;', '"': '&quot;' })[c]!);

function rigidToMatrix(T: Rigid): THREE.Matrix4 {
  const R = T.R;
  return new THREE.Matrix4().set(R[0], R[1], R[2], T.t[0], R[3], R[4], R[5], T.t[1], R[6], R[7], R[8], T.t[2], 0, 0, 0, 1);
}

function matrixToRigid(m: THREE.Matrix4): Rigid {
  const e = m.elements; // spaltenweise
  return { R: [e[0], e[4], e[8], e[1], e[5], e[9], e[2], e[6], e[10]], t: [e[12], e[13], e[14]] };
}

function markerToWorld(m: Marker, M: THREE.Matrix4): Marker {
  const p = new THREE.Vector3(...m.p).applyMatrix4(M);
  const n = new THREE.Vector3(...m.n).transformDirection(M);
  return { p: v3(p), n: v3(n) };
}

/** Typische Modellgröße – bestimmt Markergröße, Fangradius usw. */
function sceneScale(): number {
  let s = 0;
  for (const p of parts) s = Math.max(s, p.geom.boundingSphere!.radius);
  return s || 50;
}

let toastTimer = 0;
function toast(msg: string) {
  const el = document.getElementById('toast')!;
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = window.setTimeout(() => el.classList.remove('show'), 3200);
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

function addPart(name: string, geom: THREE.BufferGeometry) {
  geom.computeBoundsTree();
  const color = new THREE.Color(PART_COLORS[(nextPartId - 1) % PART_COLORS.length]);
  const mesh = new THREE.Mesh(
    geom,
    new THREE.MeshStandardMaterial({ color, roughness: 0.65, metalness: 0.05, side: THREE.DoubleSide, flatShading: false }),
  );
  mesh.matrixAutoUpdate = false;

  // Neben die bereits geladenen Teile legen, damit man bequem markieren kann
  const bb = geom.boundingBox!;
  let x = 0;
  for (const p of parts) {
    const b = new THREE.Box3().setFromObject(p.mesh);
    x = Math.max(x, b.max.x);
  }
  const gap = parts.length ? sceneScale() * 0.3 : 0;
  const layout = new THREE.Matrix4().makeTranslation(
    parts.length ? x + gap - bb.min.x : -(bb.min.x + bb.max.x) / 2,
    -(bb.min.y + bb.max.y) / 2,
    -bb.min.z,
  );
  mesh.matrix.copy(layout);
  mesh.matrixWorldNeedsUpdate = true;

  const part: Part = { id: nextPartId++, name, geom, mesh, color, layout, visible: true, samples: sampleSurface(geom, 4000) };
  mesh.userData.partId = part.id;
  parts.push(part);
  scene.add(mesh);
  if (baseId === null) baseId = part.id;

  // Ab zwei Teilen automatisch eine Verbindung anlegen, damit man direkt loslegen kann
  const unconnected = parts.filter((p) => !joints.some((j) => j.a === p.id || j.b === p.id));
  if (parts.length >= 2 && (joints.length === 0 || unconnected.includes(part))) {
    const open = joints.find((j) => j.b === null);
    if (open) open.b = part.id;
    else addJoint();
  }
}

async function loadFiles(files: FileList | File[]) {
  for (const file of Array.from(files)) {
    try {
      const geom = await loadModel(file);
      addPart(file.name.replace(/\.[^.]+$/, ''), geom);
    } catch (e) {
      toast(`${file.name}: ${(e as Error).message}`);
    }
  }
  solveAll();
  fitView();
  renderAll();
}

function removePart(id: number) {
  const p = getPart(id);
  if (!p) return;
  scene.remove(p.mesh);
  p.geom.disposeBoundsTree();
  p.geom.dispose();
  parts.splice(parts.indexOf(p), 1);
  for (const j of joints) {
    if (j.a === id) { j.a = null; j.ma = []; j.applied = false; }
    if (j.b === id) { j.b = null; j.mb = []; j.applied = false; }
  }
  if (baseId === id) baseId = parts[0]?.id ?? null;
  solveAll();
  renderAll();
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
    ordered: false,
    normals: 'auto',
    overlap: 0,
    spin: 0,
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
  joints.splice(i, 1);
  if (activeJointId === id) activeJointId = joints[joints.length - 1]?.id ?? null;
  solveAll();
  renderAll();
}

/** Markierungen haben sich geändert → alte Alternativen/Feinausrichtung verwerfen. */
function markersChanged(j: Joint) {
  j.alt = 0;
  j.icp = undefined;
  j.icpInfo = undefined;
}

/**
 * Setzt alle Teile zusammen: Ausgehend vom Basisteil werden die Verbindungen der Reihe nach
 * gelöst (Breitensuche), jedes neu erreichte Teil wird an ein bereits platziertes angefügt.
 */
function solveAll() {
  const placed = new Map<number, THREE.Matrix4>();
  for (const j of joints) {
    j.result = undefined;
    j.fixedSide = undefined;
    j.status = undefined;
  }
  const seeds = [baseId, ...joints.map((j) => j.a), ...parts.map((p) => p.id)].filter(
    (x): x is number => x !== null && !!getPart(x),
  );

  for (const seed of seeds) {
    if (placed.has(seed)) continue;
    placed.set(seed, getPart(seed)!.layout.clone());
    const queue = [seed];
    while (queue.length) {
      const cur = queue.shift()!;
      for (const j of joints) {
        if (!j.applied || j.a === null || j.b === null) continue;
        if (j.a !== cur && j.b !== cur) continue;
        const fixedSide: Side = j.a === cur ? 'a' : 'b';
        const other = fixedSide === 'a' ? j.b : j.a;
        if (placed.has(other)) {
          if (!j.fixedSide) j.status = { kind: 'warn', text: 'Beide Teile sind schon über andere Verbindungen festgelegt – diese Verbindung wird ignoriert.' };
          continue;
        }
        const fixedM = placed.get(cur)!;
        const dst = (fixedSide === 'a' ? j.ma : j.mb).map((m) => markerToWorld(m, fixedM));
        const src = fixedSide === 'a' ? j.mb : j.ma;
        const res = solveJoint({ src, dst, ordered: j.ordered, normals: j.normals });
        j.result = res;
        j.fixedSide = fixedSide;
        if (res.error) {
          j.status = { kind: 'err', text: res.error };
          continue;
        }
        dropEquivalent(res, getPart(other)!);
        const sol = res.solutions[Math.min(j.alt, res.solutions.length - 1)];
        let T = applyAdjustments(sol, dst, j.spin, j.overlap);
        if (j.icp) T = compose(j.icp, T);
        placed.set(other, rigidToMatrix(T));
        queue.push(other);

        const scaleRef = sceneScale();
        // Handgesetzte Punkte (ohne Einrasten) sind nie ganz exakt – grobe Fehler deuten auf falsche Punkte hin
        const good = !!j.icp || sol.rmsd < Math.max(0.1, scaleRef * 0.015);
        let text = `Abweichung ${fmt(sol.rmsd)} mm`;
        if (j.icpInfo) text += ` · ${j.icpInfo}`;
        if (res.ambiguous && j.alt === 0) {
          j.status = { kind: 'warn', text: `${text} – Zuordnung nicht eindeutig. Prüfe das Ergebnis, nutze „Alternative" oder setze einen weiteren, unsymmetrischen Punkt.` };
        } else if (!good) {
          j.status = { kind: 'warn', text: `${text} – die Punkte passen nicht genau zusammen. Markierungen prüfen.` };
        } else {
          j.status = { kind: 'ok', text: `✓ Verbunden · ${text}` };
        }
      }
    }
  }

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
    else if (na === 0 && nb === 0) j.status = { kind: 'info', text: 'Klicke auf passende Punkte beider Teile.' };
    else if (na !== nb) j.status = { kind: 'info', text: `Noch ${Math.abs(na - nb)} Markierung(en) auf ${na < nb ? nameOf(j.a) : nameOf(j.b)} setzen.` };
    else j.status = { kind: 'info', text: na < 3 ? 'Bereit. (Mit 3+ Punkten wird es am genauesten.)' : 'Bereit zum Verbinden.' };
  }
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

function connect(j: Joint) {
  j.applied = true;
  solveAll();
  if (j.status?.kind === 'err') {
    j.applied = false;
    toast(j.status.text);
  } else fitView();
  renderAll();
}

function refine(j: Joint) {
  if (!j.applied || !j.fixedSide || !j.result?.solutions.length) return;
  const fixed = getPart(j.fixedSide === 'a' ? j.a : j.b)!;
  const moving = getPart(j.fixedSide === 'a' ? j.b : j.a)!;
  const contacts = (j.fixedSide === 'a' ? j.ma : j.mb).map((m) => markerToWorld(m, fixed.mesh.matrixWorld).p);
  const s = sceneScale();
  const sol = j.result.solutions[Math.min(j.alt, j.result.solutions.length - 1)];
  const dst = (j.fixedSide === 'a' ? j.ma : j.mb).map((m) => markerToWorld(m, fixed.mesh.matrixWorld));
  const coarse = applyAdjustments(sol, dst, j.spin, j.overlap);
  const res = refineICP(moving.samples, coarse, fixed.geom, fixed.geom.boundsTree as MeshBVH, fixed.mesh.matrixWorld, contacts, {
    regionRadius: s * 0.35,
    maxPairDist: Math.max(sol.rmsd * 4, s * 0.02),
  });
  if (!res) {
    toast('Feinausrichtung: zu wenig Kontaktfläche in der Nähe der Markierungen gefunden.');
    return;
  }
  // Korrektur relativ zur Grobausrichtung speichern
  const inv = rigidToMatrix(coarse).invert();
  j.icp = matrixToRigid(rigidToMatrix(res.T).multiply(inv));
  j.icpInfo = `fein ausgerichtet (${res.pairs} Flächenpunkte, Rest ${fmt(res.rmsd)} mm)`;
  solveAll();
  renderAll();
}

// ---------------------------------------------------------------- Markierungen (3D)

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
  g.font = 'bold 30px system-ui, sans-serif';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(text, 32, 34);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  s.renderOrder = 20;
  return s;
}

function rebuildMarkers() {
  for (const o of [...markerGroup.children]) {
    markerGroup.remove(o);
    o.traverse((x) => {
      const m = x as THREE.Mesh;
      m.geometry?.dispose();
      const mat = m.material as THREE.Material & { map?: THREE.Texture };
      mat?.map?.dispose();
      mat?.dispose();
    });
  }
  const sphere = new THREE.SphereGeometry(1, 16, 12);
  for (const j of joints) {
    const active = j.id === activeJointId;
    for (const side of ['a', 'b'] as Side[]) {
      const part = getPart(side === 'a' ? j.a : j.b);
      if (!part || !part.visible) continue;
      const list = side === 'a' ? j.ma : j.mb;
      // Auf der bewegten Seite nach dem Verbinden die erkannte Zuordnung anzeigen
      const sol = j.result?.solutions[Math.min(j.alt, (j.result?.solutions.length ?? 1) - 1)];
      const movingSide = j.fixedSide ? (j.fixedSide === 'a' ? 'b' : 'a') : null;
      list.forEach((m, i) => {
        const w = markerToWorld(m, part.mesh.matrixWorld);
        const anchor = new THREE.Vector3(...w.p);
        const normal = new THREE.Vector3(...w.n);
        const dot = new THREE.Mesh(
          sphere,
          new THREE.MeshBasicMaterial({ color: j.color, transparent: !active, opacity: active ? 1 : 0.45 }),
        );
        dot.userData = { joint: j.id, side, index: i, px: 5, anchor, normal, lift: 0 };
        markerGroup.add(dot);

        const num = side === movingSide && sol ? sol.perm[i] + 1 : i + 1;
        const label = labelSprite(String(num), j.color);
        label.material.opacity = active ? 1 : 0.5;
        // Beschriftung etwas von der Oberfläche abheben
        label.userData = { ...dot.userData, px: 22, lift: side === 'a' ? 16 : 26 };
        markerGroup.add(label);
      });
    }
  }
  scaleMarkers();
}

// ---------------------------------------------------------------- Auswahl per Maus

const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();
let downPos: { x: number; y: number } | null = null;

function setNdc(e: PointerEvent | MouseEvent) {
  const rect = renderer.domElement.getBoundingClientRect();
  ndc.set(((e.clientX - rect.left) / rect.width) * 2 - 1, -((e.clientY - rect.top) / rect.height) * 2 + 1);
  raycaster.setFromCamera(ndc, camera);
}

/** Trifft die Oberfläche eines Teils; rastet optional an der nächsten Ecke ein. */
function pick(e: PointerEvent | MouseEvent): { part: Part; local: Marker; world: THREE.Vector3 } | null {
  setNdc(e);
  const meshes = parts.filter((p) => p.visible).map((p) => p.mesh);
  const hit = raycaster.intersectObjects(meshes, false)[0];
  if (!hit || !hit.face) return null;
  const part = getPart(hit.object.userData.partId)!;
  const inv = part.mesh.matrixWorld.clone().invert();
  const local = hit.point.clone().applyMatrix4(inv);
  // Außennormale laut Dreiecks-Umlaufsinn (nicht zur Kamera gedreht)
  const n = hit.face.normal.clone();
  const snap = (document.getElementById('snap') as HTMLInputElement).checked;
  if (snap) {
    // Fangradius in Bildschirmpixeln, umgerechnet in Modellgröße am Trefferpunkt
    const rect = renderer.domElement.getBoundingClientRect();
    const worldPerPx = (2 * hit.distance * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2))) / rect.height;
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
  return { part, local: { p: v3(local), n: v3(n.normalize()) }, world };
}

renderer.domElement.addEventListener('pointerdown', (e) => {
  downPos = { x: e.clientX, y: e.clientY };
});

renderer.domElement.addEventListener('pointermove', (e) => {
  if (e.buttons) { cursor.visible = false; return; }
  const h = pick(e);
  const j = activeJoint();
  const usable = h && j && (j.a === h.part.id || j.b === h.part.id || j.a === null || j.b === null);
  cursor.visible = !!usable;
  if (h && usable) {
    cursor.position.copy(h.world);
    (cursor.material as THREE.MeshBasicMaterial).color.copy(j!.color).lerp(new THREE.Color(1, 1, 1), 0.35);
  }
  renderer.domElement.style.cursor = usable ? 'crosshair' : 'default';
});

renderer.domElement.addEventListener('pointerleave', () => (cursor.visible = false));

renderer.domElement.addEventListener('pointerup', (e) => {
  if (!downPos || e.button !== 0) return;
  const moved = Math.hypot(e.clientX - downPos.x, e.clientY - downPos.y);
  downPos = null;
  if (moved > 5) return; // war eine Kamerabewegung

  const h = pick(e);
  if (!h) return;
  let j = activeJoint();
  if (!j) {
    if (parts.length < 2) { toast('Lade mindestens zwei Teile.'); return; }
    j = addJoint();
  }
  // Teile der Verbindung per Klick zuweisen, wenn noch nicht gewählt
  if (j.a === null && j.b !== h.part.id) j.a = h.part.id;
  else if (j.b === null && j.a !== h.part.id) j.b = h.part.id;

  const side: Side | null = j.a === h.part.id ? 'a' : j.b === h.part.id ? 'b' : null;
  if (!side) {
    toast(`„${h.part.name}" gehört nicht zur aktiven Verbindung. Wähle sie links aus oder lege eine neue an.`);
    return;
  }
  (side === 'a' ? j.ma : j.mb).push(h.local);
  history.push({ joint: j.id, side });
  markersChanged(j);
  solveAll();
  renderAll();
});

renderer.domElement.addEventListener('contextmenu', (e) => {
  e.preventDefault();
  setNdc(e);
  const hit = raycaster.intersectObjects(markerGroup.children.filter((o) => (o as THREE.Mesh).isMesh), false)[0];
  if (!hit) return;
  const { joint, side, index } = hit.object.userData as { joint: number; side: Side; index: number };
  const j = joints.find((x) => x.id === joint);
  if (!j) return;
  (side === 'a' ? j.ma : j.mb).splice(index, 1);
  markersChanged(j);
  if (j.ma.length === 0 && j.mb.length === 0) j.applied = false;
  solveAll();
  renderAll();
});

window.addEventListener('keydown', (e) => {
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') {
    const last = history.pop();
    const j = last && joints.find((x) => x.id === last.joint);
    if (!j) return;
    e.preventDefault();
    (last!.side === 'a' ? j.ma : j.mb).pop();
    markersChanged(j);
    solveAll();
    renderAll();
  }
});

// ---------------------------------------------------------------- Seitenleiste

const partsEl = document.getElementById('parts')!;
const jointsEl = document.getElementById('joints')!;

function renderParts() {
  partsEl.innerHTML = parts
    .map(
      (p) => `<li>
        <span class="dot" style="background:#${p.color.getHexString()}"></span>
        <input type="checkbox" data-act="vis" data-id="${p.id}" ${p.visible ? 'checked' : ''} title="Sichtbar" />
        <span class="name" title="${esc(p.name)}">${esc(p.name)}</span>
        <label class="base" title="Das Basisteil bleibt liegen, alle anderen werden daran angefügt.">
          <input type="radio" name="base" data-act="base" data-id="${p.id}" ${p.id === baseId ? 'checked' : ''}/> Basis
        </label>
        <button class="icon" data-act="rm-part" data-id="${p.id}" title="Entfernen">✕</button>
      </li>`,
    )
    .join('');
}

function partOptions(sel: number | null, exclude: number | null) {
  return (
    `<option value="">– wählen –</option>` +
    parts
      .filter((p) => p.id !== exclude)
      .map((p) => `<option value="${p.id}" ${p.id === sel ? 'selected' : ''}>${esc(p.name)}</option>`)
      .join('')
  );
}

function jointStatusHTML(j: Joint) {
  return j.status ? `<div class="status ${j.status.kind}" data-status="${j.id}">${esc(j.status.text)}</div>` : `<div class="status" data-status="${j.id}"></div>`;
}

function renderJoints() {
  jointsEl.innerHTML = joints
    .map((j, idx) => {
      const active = j.id === activeJointId;
      const sol = j.result?.solutions[Math.min(j.alt, (j.result?.solutions.length ?? 1) - 1)];
      const nAlt = j.result?.solutions.length ?? 0;
      const ready = j.a !== null && j.b !== null && j.ma.length > 0 && j.ma.length === j.mb.length;
      const head = `<div class="head">
          <span class="dot" style="background:#${j.color.getHexString()}"></span>
          <span class="grow">Verbindung ${idx + 1}</span>
          <button class="icon" data-act="rm-joint" data-id="${j.id}" title="Verbindung löschen">✕</button>
        </div>`;
      if (!active) {
        return `<div class="joint" data-act="activate" data-id="${j.id}">${head}
          <div class="help" style="margin:0">${esc(nameOf(j.a))} ↔ ${esc(nameOf(j.b))} · ${j.ma.length}/${j.mb.length} Punkte</div>
          ${jointStatusHTML(j)}</div>`;
      }
      return `<div class="joint active">${head}
        <label class="field">Teil 1<select data-act="sel-a" data-id="${j.id}">${partOptions(j.a, j.b)}</select></label>
        <label class="field">Teil 2<select data-act="sel-b" data-id="${j.id}">${partOptions(j.b, j.a)}</select></label>
        <div class="counts">
          <div class="c"><span>Teil 1: <b>${j.ma.length}</b></span><button class="icon" data-act="clr-a" data-id="${j.id}" title="Markierungen löschen">⌫</button></div>
          <div class="c"><span>Teil 2: <b>${j.mb.length}</b></span><button class="icon" data-act="clr-b" data-id="${j.id}" title="Markierungen löschen">⌫</button></div>
        </div>
        <label class="check small"><input type="checkbox" data-act="ordered" data-id="${j.id}" ${j.ordered ? 'checked' : ''}/>
          Reihenfolge vorgeben (sonst automatische Zuordnung)</label>
        <label class="field">Flächen­normalen
          <span class="seg">${(['auto', 'on', 'off'] as NormalMode[])
            .map((m) => `<button data-act="normals" data-val="${m}" data-id="${j.id}" class="${j.normals === m ? 'on' : ''}">${{ auto: 'Auto', on: 'An', off: 'Aus' }[m]}</button>`)
            .join('')}</span></label>
        <label class="field" title="Schiebt das Teil etwas ineinander, damit der Slicer beide Teile sicher zu einem Körper verschmilzt.">Einsinken (mm)
          <input type="number" step="0.05" min="-5" max="5" value="${j.overlap}" data-act="overlap" data-id="${j.id}"/></label>
        ${sol?.freeAxis ? `<label class="field" title="Bei wenigen Punkten ist die Drehung um die Kontaktachse nicht festgelegt.">Drehung <span class="row"><input type="range" min="-180" max="180" step="1" value="${j.spin}" data-act="spin" data-id="${j.id}"/><span data-spinval="${j.id}" style="width:38px;text-align:right">${j.spin}°</span></span></label>` : ''}
        <div class="row wrap">
          ${j.applied
            ? `<button class="ghost" data-act="disconnect" data-id="${j.id}">Lösen</button>
               <button class="ghost" data-act="refine" data-id="${j.id}" title="Zieht die Kontaktflächen nahe der Markierungen genau aufeinander (ICP).">Feinausrichtung</button>
               ${nAlt > 1 && !j.ordered ? `<button class="ghost" data-act="alt" data-id="${j.id}" title="Nächstbeste Zuordnung der Markierungen ausprobieren">Alternative ${Math.min(j.alt, nAlt - 1) + 1}/${nAlt}</button>` : ''}`
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
  if (parts.length === 0) el.textContent = 'Lade zwei oder mehr Modelle – oder probiere das Beispiel.';
  else if (parts.length === 1) el.textContent = 'Lade noch ein weiteres Teil, das angefügt werden soll.';
  else if (!j) el.textContent = 'Lege links eine Verbindung an.';
  else if (j.a === null || j.b === null) el.textContent = 'Klicke die beiden Teile an, die verbunden werden sollen.';
  else if (j.applied) el.textContent = 'Verbunden. Weitere Punkte verfeinern das Ergebnis live · Rechtsklick entfernt eine Markierung.';
  else {
    const na = j.ma.length;
    const nb = j.mb.length;
    if (na === 0 && nb === 0) el.textContent = `Klicke markante Punkte auf „${nameOf(j.a)}" – danach dieselben Stellen auf „${nameOf(j.b)}".`;
    else if (na !== nb) el.textContent = `Noch ${Math.abs(na - nb)} Punkt(e) auf „${na < nb ? nameOf(j.a) : nameOf(j.b)}" – Reihenfolge egal.`;
    else el.textContent = `${na} Punktpaar(e) gesetzt. Weitere Punkte setzen oder links auf „Verbinden" klicken.`;
  }
}

function renderAll() {
  renderParts();
  renderJoints();
  renderHint();
  rebuildMarkers();
}

/** Nur Statuszeilen aktualisieren (z. B. während ein Regler gezogen wird). */
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
    case 'clr-a':
    case 'clr-b':
      if (!j) break;
      if (t.dataset.act === 'clr-a') j.ma = [];
      else j.mb = [];
      j.applied = false;
      markersChanged(j);
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
      j.applied = false;
      j.icp = undefined;
      j.icpInfo = undefined;
      solveAll();
      renderAll();
      break;
    case 'refine':
      if (j) refine(j);
      break;
    case 'alt':
      if (!j?.result) break;
      j.alt = (Math.min(j.alt, j.result.solutions.length - 1) + 1) % j.result.solutions.length;
      j.icp = undefined;
      j.icpInfo = undefined;
      solveAll();
      renderAll();
      break;
  }
});

document.getElementById('panel')!.addEventListener('input', (e) => {
  const t = e.target as HTMLInputElement;
  const j = joints.find((x) => x.id === Number(t.dataset.id));
  if (t.dataset.act === 'spin' && j) {
    j.spin = Number(t.value);
    j.icp = undefined;
    j.icpInfo = undefined;
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
    case 'vis': {
      const p = getPart(id);
      if (p) { p.visible = t.checked; p.mesh.visible = t.checked; }
      renderAll();
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
      if (t.dataset.act === 'sel-a') { j.a = v; j.ma = []; } else { j.b = v; j.mb = []; }
      j.applied = false;
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

document.getElementById('demo')!.addEventListener('click', () => {
  for (const p of [...parts]) removePart(p.id);
  joints.length = 0;
  activeJointId = null;
  for (const { name, geom } of makeDemo()) addPart(name, geom);
  solveAll();
  fitView();
  renderAll();
  toast('Beispiel geladen: Setze z. B. die vier Ecken der Unterseite des Dachs und die vier oberen Ecken des Hauses.');
});

document.querySelectorAll<HTMLButtonElement>('[data-export]').forEach((b) =>
  b.addEventListener('click', () => {
    const list: ExportPart[] = parts
      .filter((p) => p.visible)
      .map((p) => ({ name: p.name, geometry: p.geom.clone().applyMatrix4(p.mesh.matrixWorld) }));
    if (!list.length) { toast('Keine sichtbaren Teile zum Exportieren.'); return; }
    const kind = b.dataset.export;
    if (kind === 'stl') download(toSTL(list), 'zusammengefuegt.stl');
    else if (kind === 'obj') download(toOBJ(list), 'zusammengefuegt.obj');
    else download(to3MF(list, kind === '3mf-parts'), 'zusammengefuegt.3mf');
  }),
);

// Für automatisierte Tests / Debugging
Object.assign(window as unknown as Record<string, unknown>, {
  partjoin: { parts, joints, solveAll, renderAll, camera, controls },
});

solveAll();
renderAll();
