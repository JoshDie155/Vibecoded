/**
 * PartJoin-Projekte: alle Teile mit Lage, Farbe, Gruppe und Slicer-Zusätzen sowie alle Verbindungen in
 * einer Datei (.partjoin – ein ZIP-Archiv aus project.json und den Netzen als Float32-Binärdaten).
 *
 * Dazu die automatische Sicherung der Sitzung im Browser (IndexedDB): Nach einem Neuladen der Seite
 * lässt sich der letzte Stand zurückholen. Die Netze werden dort einzeln abgelegt und nur geschrieben,
 * wenn sie noch fehlen; bei jeder Änderung wird nur die (kleine) Beschreibung erneuert.
 */
import { strFromU8, strToU8, unzipSync, zipSync } from 'fflate';
import type { Modifier } from './threemf';

export const PROJECT_EXT = 'partjoin';
const VERSION = 1;

export interface ProjectPart {
  /** Id beim Speichern – Verbindungen verweisen darauf, beim Öffnen werden sie neu vergeben. */
  id: number;
  name: string;
  color: string;
  group?: string;
  /** Ausgangslage (4×4, spaltenweise wie THREE.Matrix4). */
  layout: number[];
  visible: boolean;
  fromFile?: boolean;
  /** Schlüssel des Netzes (nicht indizierte Dreiecke, mm). */
  geometry: string;
  paint?: Record<string, string[]>;
  palette?: string[];
  settings?: Record<string, string>;
  objectSettings?: Record<string, string>;
  /** Modifier: Netz (geometry) und bei Text sein Netz in eigenen Koordinaten (localGeometry). */
  modifiers?: (Omit<Modifier, 'positions' | 'local'> & { geometry: string; localGeometry?: string })[];
}

export interface ProjectData {
  app: 'PartJoin';
  version: number;
  savedAt: string;
  parts: ProjectPart[];
  /** Verbindungen mit Teilen (Ids wie in parts), Markierungen, Flächen und Einstellungen. */
  joints: Record<string, unknown>[];
  baseId: number | null;
  activeJointId: number | null;
  settings: { onFloor: boolean; printer?: string };
  camera?: { position: number[]; target: number[] };
}

/** Netze je Schlüssel. */
export type Blobs = Map<string, Float32Array>;

export function packProject(data: ProjectData, blobs: Blobs): Uint8Array {
  const files: Record<string, Uint8Array | [Uint8Array, { level: 0 | 1 | 6 }]> = { 'project.json': strToU8(JSON.stringify(data)) };
  // Netze lassen sich kaum packen – schnell statt klein
  for (const [k, v] of blobs) files[`geometry/${k}.bin`] = [new Uint8Array(v.buffer, v.byteOffset, v.byteLength), { level: 1 }];
  return zipSync(files, { level: 6 });
}

export function unpackProject(bytes: Uint8Array): { data: ProjectData; blobs: Blobs } {
  let files: Record<string, Uint8Array>;
  try {
    files = unzipSync(bytes);
  } catch {
    throw new Error('Die Datei ist kein PartJoin-Projekt.');
  }
  const json = files['project.json'];
  if (!json) throw new Error('Die Datei ist kein PartJoin-Projekt.');
  const data = JSON.parse(strFromU8(json)) as ProjectData;
  if (data.app !== 'PartJoin' || !Array.isArray(data.parts)) throw new Error('Die Datei ist kein PartJoin-Projekt.');
  if (data.version > VERSION) throw new Error('Das Projekt stammt aus einer neueren PartJoin-Version.');
  const blobs: Blobs = new Map();
  for (const [name, u8] of Object.entries(files)) {
    const m = /^geometry\/(.+)\.bin$/.exec(name);
    // Kopie: Float32Array braucht einen an 4 Byte ausgerichteten Puffer
    if (m) blobs.set(m[1], new Float32Array(u8.slice().buffer));
  }
  for (const key of referencedKeys(data)) if (!blobs.has(key)) throw new Error('Im Projekt fehlt ein Netz – die Datei ist unvollständig.');
  return { data, blobs };
}

export function newProject(fields: Omit<ProjectData, 'app' | 'version' | 'savedAt'>): ProjectData {
  return { app: 'PartJoin', version: VERSION, savedAt: new Date().toISOString(), ...fields };
}

/** Alle Netz-Schlüssel, auf die das Projekt verweist. */
export function referencedKeys(data: ProjectData): Set<string> {
  const keys = new Set<string>();
  for (const p of data.parts) {
    keys.add(p.geometry);
    for (const m of p.modifiers ?? []) {
      keys.add(m.geometry);
      if (m.localGeometry) keys.add(m.localGeometry);
    }
  }
  return keys;
}

// ---------------------------------------------------------------- Sitzung im Browser

const DB = 'partjoin';
const SESSION = 'session';
const GEOMETRY = 'geometry';

function openDb(): Promise<IDBDatabase> {
  return new Promise((resolve, reject) => {
    const req = indexedDB.open(DB, 1);
    req.onupgradeneeded = () => {
      req.result.createObjectStore(SESSION);
      req.result.createObjectStore(GEOMETRY);
    };
    req.onsuccess = () => resolve(req.result);
    req.onerror = () => reject(req.error);
  });
}

const done = (tx: IDBTransaction) => new Promise<void>((resolve, reject) => {
  tx.oncomplete = () => resolve();
  tx.onerror = () => reject(tx.error);
  tx.onabort = () => reject(tx.error);
});

const result = <T>(req: IDBRequest<T>) => new Promise<T>((resolve, reject) => {
  req.onsuccess = () => resolve(req.result);
  req.onerror = () => reject(req.error);
});

/**
 * Speichert die Beschreibung; von den Netzen nur die, die noch nicht abgelegt sind (Netze ändern sich nie).
 * Nicht mehr gebrauchte Netze werden gelöscht.
 */
export async function saveSession(data: ProjectData, blobs: Blobs): Promise<void> {
  const db = await openDb();
  try {
    const tx = db.transaction([SESSION, GEOMETRY], 'readwrite');
    const geo = tx.objectStore(GEOMETRY);
    const stored = new Set((await result(geo.getAllKeys())).map(String));
    const used = referencedKeys(data);
    for (const [k, v] of blobs) if (!stored.has(k)) geo.put(v, k);
    for (const k of stored) if (!used.has(k)) geo.delete(k);
    tx.objectStore(SESSION).put(data, 'current');
    await done(tx);
  } finally {
    db.close();
  }
}

/** Beschreibung der gesicherten Sitzung (ohne Netze) – oder null. */
export async function sessionInfo(): Promise<ProjectData | null> {
  const db = await openDb();
  try {
    const data = await result(db.transaction(SESSION).objectStore(SESSION).get('current'));
    return (data as ProjectData | undefined)?.parts?.length ? (data as ProjectData) : null;
  } finally {
    db.close();
  }
}

/** Gesicherte Sitzung samt Netzen – oder null, wenn keine (vollständige) vorhanden ist. */
export async function loadSession(): Promise<{ data: ProjectData; blobs: Blobs } | null> {
  const db = await openDb();
  try {
    const tx = db.transaction([SESSION, GEOMETRY]);
    const data = (await result(tx.objectStore(SESSION).get('current'))) as ProjectData | undefined;
    if (!data?.parts?.length) return null;
    const blobs: Blobs = new Map();
    const geo = tx.objectStore(GEOMETRY);
    for (const key of referencedKeys(data)) {
      const v = (await result(geo.get(key))) as Float32Array | undefined;
      if (!v) return null;
      blobs.set(key, v);
    }
    return { data, blobs };
  } finally {
    db.close();
  }
}

export async function clearSession(): Promise<void> {
  const db = await openDb();
  try {
    const tx = db.transaction([SESSION, GEOMETRY], 'readwrite');
    tx.objectStore(SESSION).clear();
    tx.objectStore(GEOMETRY).clear();
    await done(tx);
  } finally {
    db.close();
  }
}
