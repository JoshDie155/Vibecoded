/**
 * 3MF lesen und schreiben – mit Teilen und Farben.
 *
 * Lesen: Jedes Teil (Netz) wird ein eigenes Teil, Teile eines Objekts bilden eine Gruppe und
 * behalten ihre Lage zueinander. Farben kommen aus dem 3MF-Standard (basematerials, colorgroup)
 * oder aus den Filament-Zuordnungen von Bambu Studio / OrcaSlicer (Metadata/model_settings.config,
 * project_settings.config) bzw. PrusaSlicer (Metadata/Slic3r_PE_model.config, Slic3r_PE.config).
 * Mehrere Modelldateien im Archiv (3D/Objects/*.model, Production-Erweiterung) werden aufgelöst.
 *
 * Schreiben: entweder ein einziger Körper oder ein Objekt aus mehreren Teilen mit je eigener Farbe
 * und eigenem Extruder – so ordnet der Slicer jedem Teil direkt ein Filament zu.
 *
 * Bewusst ohne DOM: Die XML-Dateien sind maschinell erzeugt und regelmäßig aufgebaut, ein schneller
 * Durchlauf mit regulären Ausdrücken genügt und funktioniert auch in Tests ohne Browser.
 */
import { strFromU8, strToU8, unzipSync, zipSync } from 'fflate';

export interface ImportedPart {
  name: string;
  /** Nicht indizierte Dreiecke (9 Werte pro Dreieck) in Millimetern, Lage aus der Datei. */
  positions: Float32Array;
  /** Farbe als '#rrggbb', falls die Datei eine angibt. */
  color?: string;
  /** Teile desselben Objekts haben dieselbe Gruppe. */
  group: string;
}

export interface ColoredPart {
  name: string;
  /** Nicht indizierte Dreiecke in Weltkoordinaten (mm). */
  positions: Float32Array;
  /** '#rrggbb' */
  color: string;
}

// ---------------------------------------------------------------- Lesen

type M12 = number[]; // 3MF-Matrix: m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32 (Zeilenvektoren)
const IDENT: M12 = [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0];

interface RawMesh {
  verts: Float64Array;
  tris: Uint32Array;
  /** Eigenschaft je Dreieck (pid, p1), -1 = Vorgabe des Objekts. */
  triPid: Int32Array;
  triP1: Int32Array;
}

interface RawObject {
  id: string;
  name?: string;
  pid?: string;
  pindex?: number;
  mesh?: RawMesh;
  components?: { objectid: string; path?: string; transform: M12 }[];
}

interface RawModel {
  scale: number;
  objects: Map<string, RawObject>;
  /** id → Farben der Einträge */
  colors: Map<string, string[]>;
  build: { objectid: string; path?: string; transform: M12 }[];
}

const UNITS: Record<string, number> = { micron: 0.001, millimeter: 1, centimeter: 10, inch: 25.4, foot: 304.8, meter: 1000 };

function attrs(s: string): Record<string, string> {
  const out: Record<string, string> = {};
  const re = /([\w:.-]+)\s*=\s*(?:"([^"]*)"|'([^']*)')/g;
  let m: RegExpExecArray | null;
  while ((m = re.exec(s))) out[m[1]] = m[2] ?? m[3];
  return out;
}

/** Attribut ohne Rücksicht auf den Namensraum-Präfix (p:path, path …). */
function attr(a: Record<string, string>, name: string): string | undefined {
  if (a[name] !== undefined) return a[name];
  for (const k in a) if (k.endsWith(`:${name}`)) return a[k];
  return undefined;
}

function xmlUnescape(s: string): string {
  return s.replace(/&(lt|gt|amp|quot|apos|#\d+|#x[0-9a-fA-F]+);/g, (_, e: string) => {
    if (e === 'lt') return '<';
    if (e === 'gt') return '>';
    if (e === 'amp') return '&';
    if (e === 'quot') return '"';
    if (e === 'apos') return "'";
    return String.fromCodePoint(e[1] === 'x' ? parseInt(e.slice(2), 16) : parseInt(e.slice(1), 10));
  });
}

function parseTransform(s: string | undefined): M12 {
  if (!s) return IDENT;
  const v = s.trim().split(/\s+/).map(Number);
  return v.length === 12 && v.every(Number.isFinite) ? v : IDENT;
}

/** a, dann b (Punkt · a · b). */
function mul(a: M12, b: M12): M12 {
  const r = (i: number, j: number) => a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
  return [
    r(0, 0), r(0, 1), r(0, 2),
    r(1, 0), r(1, 1), r(1, 2),
    r(2, 0), r(2, 1), r(2, 2),
    a[9] * b[0] + a[10] * b[3] + a[11] * b[6] + b[9],
    a[9] * b[1] + a[10] * b[4] + a[11] * b[7] + b[10],
    a[9] * b[2] + a[10] * b[5] + a[11] * b[8] + b[11],
  ];
}

function normColor(c: string | undefined): string | undefined {
  if (!c) return undefined;
  const m = /^#?([0-9a-fA-F]{6})([0-9a-fA-F]{2})?$/.exec(c.trim());
  return m ? `#${m[1].toLowerCase()}` : undefined;
}

function parseMesh(body: string): RawMesh {
  const vs: number[] = [];
  const vre = /<(?:\w+:)?vertex\b([^>]*?)\/?>/g;
  let m: RegExpExecArray | null;
  while ((m = vre.exec(body))) {
    const a = attrs(m[1]);
    vs.push(Number(a.x), Number(a.y), Number(a.z));
  }
  const ts: number[] = [];
  const pids: number[] = [];
  const p1s: number[] = [];
  const tre = /<(?:\w+:)?triangle\b([^>]*?)\/?>/g;
  while ((m = tre.exec(body))) {
    const a = attrs(m[1]);
    ts.push(Number(a.v1), Number(a.v2), Number(a.v3));
    pids.push(a.pid !== undefined ? Number(a.pid) : -1);
    p1s.push(a.p1 !== undefined ? Number(a.p1) : -1);
  }
  return { verts: Float64Array.from(vs), tris: Uint32Array.from(ts), triPid: Int32Array.from(pids), triP1: Int32Array.from(p1s) };
}

function parseModel(xml: string): RawModel {
  const head = /<(?:\w+:)?model\b([^>]*)>/.exec(xml);
  const unit = head ? attrs(head[1]).unit : undefined;
  const model: RawModel = { scale: UNITS[unit ?? 'millimeter'] ?? 1, objects: new Map(), colors: new Map(), build: [] };

  const bre = /<(?:\w+:)?basematerials\b([^>]*)>([\s\S]*?)<\/(?:\w+:)?basematerials>/g;
  let m: RegExpExecArray | null;
  while ((m = bre.exec(xml))) {
    const id = attrs(m[1]).id;
    const list = [...m[2].matchAll(/<(?:\w+:)?base\b([^>]*?)\/?>/g)].map((b) => normColor(attrs(b[1]).displaycolor) ?? '#cccccc');
    if (id) model.colors.set(id, list);
  }
  const cre = /<(?:\w+:)?colorgroup\b([^>]*)>([\s\S]*?)<\/(?:\w+:)?colorgroup>/g;
  while ((m = cre.exec(xml))) {
    const id = attrs(m[1]).id;
    const list = [...m[2].matchAll(/<(?:\w+:)?color\b([^>]*?)\/?>/g)].map((b) => normColor(attrs(b[1]).color) ?? '#cccccc');
    if (id) model.colors.set(id, list);
  }

  const ore = /<(?:\w+:)?object\b([^>]*)>([\s\S]*?)<\/(?:\w+:)?object>/g;
  while ((m = ore.exec(xml))) {
    const a = attrs(m[1]);
    if (!a.id) continue;
    const obj: RawObject = { id: a.id, name: a.name ? xmlUnescape(a.name) : undefined, pid: a.pid, pindex: a.pindex !== undefined ? Number(a.pindex) : undefined };
    const meshBody = /<(?:\w+:)?mesh\b[^>]*>([\s\S]*?)<\/(?:\w+:)?mesh>/.exec(m[2]);
    if (meshBody) obj.mesh = parseMesh(meshBody[1]);
    const comps = /<(?:\w+:)?components\b[^>]*>([\s\S]*?)<\/(?:\w+:)?components>/.exec(m[2]);
    if (comps) {
      obj.components = [...comps[1].matchAll(/<(?:\w+:)?component\b([^>]*?)\/?>/g)].map((c) => {
        const ca = attrs(c[1]);
        return { objectid: ca.objectid, path: attr(ca, 'path'), transform: parseTransform(ca.transform) };
      });
    }
    model.objects.set(obj.id, obj);
  }

  const build = /<(?:\w+:)?build\b[^>]*>([\s\S]*?)<\/(?:\w+:)?build>/.exec(xml);
  if (build) {
    for (const it of build[1].matchAll(/<(?:\w+:)?item\b([^>]*?)\/?>/g)) {
      const a = attrs(it[1]);
      if (a.objectid) model.build.push({ objectid: a.objectid, path: attr(a, 'path'), transform: parseTransform(a.transform) });
    }
  }
  return model;
}

const normPath = (p: string) => p.replace(/^\//, '');

/** Hauptmodell laut _rels/.rels, sonst die übliche Stelle. */
function rootModelPath(files: Record<string, Uint8Array>): string {
  const rels = files['_rels/.rels'];
  if (rels) {
    const m = /Target\s*=\s*"([^"]+\.model)"/i.exec(strFromU8(rels));
    if (m && files[normPath(m[1])]) return normPath(m[1]);
  }
  if (files['3D/3dmodel.model']) return '3D/3dmodel.model';
  const any = Object.keys(files).find((k) => k.toLowerCase().endsWith('.model'));
  if (!any) throw new Error('Die 3MF-Datei enthält kein 3D-Modell.');
  return any;
}

/** Filament-Zuordnung aus den Slicer-Metadaten: Objekt-ID → { name, extruder, Teile }. */
interface SlicerObject {
  name?: string;
  extruder?: number;
  /** Bambu/Orca: Teil-ID (= objectid der Komponente) → Name/Extruder */
  parts: Map<string, { name?: string; extruder?: number }>;
  /** PrusaSlicer: Dreiecksbereiche im Netz des Objekts */
  volumes: { first: number; last: number; name?: string; extruder?: number }[];
}

function metadata(body: string): Record<string, string> {
  const out: Record<string, string> = {};
  for (const m of body.matchAll(/<metadata\b([^>]*?)\/?>/g)) {
    const a = attrs(m[1]);
    if (a.key) out[a.key] = xmlUnescape(a.value ?? '');
  }
  return out;
}

function readSlicerConfig(files: Record<string, Uint8Array>): { objects: Map<string, SlicerObject>; colors: string[] } {
  const objects = new Map<string, SlicerObject>();
  let colors: string[] = [];

  // Bambu Studio / OrcaSlicer
  const bambu = files['Metadata/model_settings.config'];
  if (bambu) {
    const xml = strFromU8(bambu);
    for (const o of xml.matchAll(/<object\b([^>]*)>([\s\S]*?)<\/object>/g)) {
      const id = attrs(o[1]).id;
      if (!id) continue;
      const own = metadata(o[2].replace(/<part\b[\s\S]*?<\/part>/g, ''));
      const entry: SlicerObject = { name: own.name, extruder: own.extruder ? Number(own.extruder) : undefined, parts: new Map(), volumes: [] };
      for (const p of o[2].matchAll(/<part\b([^>]*)>([\s\S]*?)<\/part>/g)) {
        const pa = attrs(p[1]);
        if (pa.subtype && pa.subtype !== 'normal_part') continue;
        const md = metadata(p[2]);
        if (pa.id) entry.parts.set(pa.id, { name: md.name, extruder: md.extruder ? Number(md.extruder) : undefined });
      }
      objects.set(id, entry);
    }
  }
  const project = files['Metadata/project_settings.config'];
  if (project) {
    try {
      const json = JSON.parse(strFromU8(project));
      const list = json.filament_colour ?? json.extruder_colour;
      if (Array.isArray(list)) colors = list.map((c: string) => normColor(c) ?? '#cccccc');
    } catch {
      // keine gültigen Projekteinstellungen – Farben bleiben offen
    }
  }

  // PrusaSlicer
  const prusa = files['Metadata/Slic3r_PE_model.config'];
  if (prusa) {
    const xml = strFromU8(prusa);
    for (const o of xml.matchAll(/<object\b([^>]*)>([\s\S]*?)<\/object>/g)) {
      const id = attrs(o[1]).id;
      if (!id) continue;
      const own = metadata(o[2].replace(/<volume\b[\s\S]*?<\/volume>/g, ''));
      const entry: SlicerObject = objects.get(id) ?? { parts: new Map(), volumes: [] };
      entry.name ??= own.name;
      if (own.extruder) entry.extruder ??= Number(own.extruder);
      for (const v of o[2].matchAll(/<volume\b([^>]*)>([\s\S]*?)<\/volume>/g)) {
        const va = attrs(v[1]);
        const md = metadata(v[2]);
        if (md.modifier === '1' || (md.volume_type && md.volume_type !== 'ModelPart')) continue;
        entry.volumes.push({ first: Number(va.firstid), last: Number(va.lastid), name: md.name, extruder: md.extruder ? Number(md.extruder) : undefined });
      }
      objects.set(id, entry);
    }
  }
  const prusaIni = files['Metadata/Slic3r_PE.config'];
  if (prusaIni && !colors.length) {
    const ini = strFromU8(prusaIni);
    const m = /^;\s*(?:extruder_colour|filament_colour)\s*=\s*(.+)$/m.exec(ini);
    if (m) colors = m[1].split(';').map((c) => normColor(c.replace(/"/g, '')) ?? '#cccccc');
  }
  return { objects, colors };
}

/** Liest alle Teile einer 3MF-Datei. */
export function read3MF(data: Uint8Array): ImportedPart[] {
  const files = unzipSync(data);
  const root = rootModelPath(files);
  const models = new Map<string, RawModel>();
  const modelAt = (path: string | undefined): RawModel | undefined => {
    const p = path ? normPath(path) : root;
    if (!models.has(p)) {
      const f = files[p];
      if (!f) return undefined;
      models.set(p, parseModel(strFromU8(f)));
    }
    return models.get(p);
  };
  const main = modelAt(root)!;
  const slicer = readSlicerConfig(files);
  const extruderColor = (e: number | undefined) => (e && slicer.colors[e - 1]) || undefined;

  const parts: ImportedPart[] = [];
  const names = new Map<string, number>();
  const uniqueName = (raw: string) => {
    // Slicer übernehmen oft den Dateinamen samt Endung („Kopf.stl")
    const n = raw.replace(/\.(stl|obj|3mf|amf|ply|step|stp|glb|gltf|fbx)$/i, '') || raw;
    const k = names.get(n) ?? 0;
    names.set(n, k + 1);
    return k ? `${n} (${k + 1})` : n;
  };

  /** Teil(e) aus einem Netz: nach Dreiecksfarbe bzw. PrusaSlicer-Volumen getrennt. */
  const emit = (
    model: RawModel, obj: RawObject, tf: M12, group: string, baseName: string,
    fallback: string | undefined, volumes: SlicerObject['volumes'],
  ) => {
    const mesh = obj.mesh!;
    const s = model.scale;
    const triCount = mesh.tris.length / 3;
    const objColor = obj.pid !== undefined ? model.colors.get(obj.pid)?.[obj.pindex ?? 0] : undefined;
    // Bereiche: PrusaSlicer-Volumen oder sonst Dreiecke gleicher Farbe
    const keyOf = new Array<string>(triCount);
    const info = new Map<string, { name: string; color?: string }>();
    /** Farbe eines Dreiecks laut 3MF (pid/p1, sonst Vorgabe des Objekts). */
    const triColor = (t: number) => {
      const pid = mesh.triPid[t] >= 0 ? String(mesh.triPid[t]) : obj.pid;
      const idx = mesh.triP1[t] >= 0 ? mesh.triP1[t] : obj.pindex ?? 0;
      return pid !== undefined ? model.colors.get(pid)?.[idx] : undefined;
    };
    if (volumes.length) {
      volumes.forEach((v, k) => {
        const first = Math.max(0, v.first);
        for (let t = first; t <= Math.min(triCount - 1, v.last); t++) keyOf[t] = `v${k}`;
        const own = first < triCount ? triColor(first) : undefined;
        info.set(`v${k}`, { name: v.name ?? `${baseName} ${k + 1}`, color: extruderColor(v.extruder) ?? own ?? fallback ?? objColor });
      });
    }
    for (let t = 0; t < triCount; t++) {
      if (keyOf[t]) continue;
      const c = triColor(t) ?? fallback ?? objColor;
      const key = `c${c ?? ''}`;
      keyOf[t] = key;
      if (!info.has(key)) info.set(key, { name: baseName, color: c });
    }
    const groups = new Map<string, number[]>();
    for (let t = 0; t < triCount; t++) {
      const list = groups.get(keyOf[t]);
      if (list) list.push(t);
      else groups.set(keyOf[t], [t]);
    }
    const several = groups.size > 1;
    let k = 0;
    for (const [key, tris] of groups) {
      k++;
      const out = new Float32Array(tris.length * 9);
      let o = 0;
      for (const t of tris) {
        for (let c = 0; c < 3; c++) {
          const v = mesh.tris[t * 3 + c] * 3;
          const x = mesh.verts[v] * s;
          const y = mesh.verts[v + 1] * s;
          const z = mesh.verts[v + 2] * s;
          out[o++] = x * tf[0] + y * tf[3] + z * tf[6] + tf[9] * s;
          out[o++] = x * tf[1] + y * tf[4] + z * tf[7] + tf[10] * s;
          out[o++] = x * tf[2] + y * tf[5] + z * tf[8] + tf[11] * s;
        }
      }
      const i = info.get(key)!;
      const name = several && !key.startsWith('v') ? `${i.name} – Farbe ${k}` : i.name;
      parts.push({ name: uniqueName(name), positions: out, color: i.color, group });
    }
  };

  const walk = (
    model: RawModel, id: string, tf: M12, group: string, depth: number,
    meta: { name?: string; color?: string; slicer?: SlicerObject },
  ) => {
    const obj = model.objects.get(id);
    if (!obj || depth > 16) return;
    if (obj.mesh && obj.mesh.tris.length) {
      const part = meta.slicer?.parts.get(id);
      const name = part?.name ?? obj.name ?? meta.name ?? 'Teil';
      const color = extruderColor(part?.extruder) ?? meta.color;
      emit(model, obj, tf, group, name, color, depth === 0 ? meta.slicer?.volumes ?? [] : []);
    }
    for (const c of obj.components ?? []) {
      const sub = c.path ? modelAt(c.path) : model;
      if (!sub) continue;
      // Übersetzung der Komponente liegt in den Einheiten des aufrufenden Modells
      walk(sub, c.objectid, mul(c.transform, tf), group, depth + 1, { ...meta, name: obj.name ?? meta.name });
    }
  };

  const items = main.build.length ? main.build : [...main.objects.keys()].map((objectid) => ({ objectid, path: undefined, transform: IDENT }));
  items.forEach((it, n) => {
    const model = it.path ? modelAt(it.path) : main;
    if (!model) return;
    const so = slicer.objects.get(it.objectid);
    const obj = model.objects.get(it.objectid);
    walk(model, it.objectid, it.transform, `3mf-${n}`, 0, {
      name: so?.name ?? obj?.name,
      color: extruderColor(so?.extruder),
      slicer: so,
    });
  });
  if (!parts.length) throw new Error('Die 3MF-Datei enthält keine Dreiecksnetze.');
  return parts;
}

// ---------------------------------------------------------------- Schreiben

const f5 = (x: number) => String(+x.toFixed(5));
const esc = (s: string) => s.replace(/[<>&"]/g, (c) => ({ '<': '&lt;', '>': '&gt;', '&': '&amp;', '"': '&quot;' })[c]!);

/** Gleiche Eckpunkte zusammenfassen (3MF verlangt ein indiziertes Netz). */
function weld(p: Float32Array): { verts: number[]; tris: number[] } {
  const map = new Map<string, number>();
  const verts: number[] = [];
  const tris: number[] = [];
  const idx: number[] = [0, 0, 0];
  for (let t = 0; t < p.length; t += 9) {
    for (let c = 0; c < 3; c++) {
      const x = p[t + c * 3];
      const y = p[t + c * 3 + 1];
      const z = p[t + c * 3 + 2];
      const key = `${f5(x)},${f5(y)},${f5(z)}`;
      let i = map.get(key);
      if (i === undefined) {
        i = verts.length / 3;
        map.set(key, i);
        verts.push(x, y, z);
      }
      idx[c] = i;
    }
    // Entartete Dreiecke (nach dem Zusammenfassen) weglassen
    if (idx[0] === idx[1] || idx[1] === idx[2] || idx[0] === idx[2]) continue;
    tris.push(idx[0], idx[1], idx[2]);
  }
  return { verts, tris };
}

/** Farben → Extrudernummern (1, 2, …) in der Reihenfolge des ersten Auftretens. */
export function colorSlots(parts: ColoredPart[]): string[] {
  const list: string[] = [];
  for (const p of parts) if (!list.includes(p.color.toLowerCase())) list.push(p.color.toLowerCase());
  return list;
}

/**
 * @param mode 'single': ein Körper aus allen Teilen (ohne Farben),
 *             'multicolor': ein Objekt aus mehreren Teilen mit je eigener Farbe/eigenem Extruder.
 */
export function write3MF(parts: ColoredPart[], mode: 'single' | 'multicolor'): Uint8Array {
  const slots = colorSlots(parts);
  const vChunks: string[] = [];
  const tChunks: string[] = [];
  const volumes: { first: number; last: number; name: string; extruder: number }[] = [];
  let vOffset = 0;
  let tCount = 0;
  const list = mode === 'single'
    ? [{ name: 'PartJoin', color: slots[0] ?? '#cccccc', positions: concat(parts.map((p) => p.positions)) }]
    : parts;
  for (const part of list) {
    const { verts, tris } = weld(part.positions);
    for (let i = 0; i < verts.length; i += 3) vChunks.push(`<vertex x="${f5(verts[i])}" y="${f5(verts[i + 1])}" z="${f5(verts[i + 2])}"/>`);
    const slot = Math.max(0, slots.indexOf(part.color.toLowerCase()));
    const colorAttr = mode === 'multicolor' ? ` pid="1" p1="${slot}"` : '';
    for (let i = 0; i < tris.length; i += 3)
      tChunks.push(`<triangle v1="${tris[i] + vOffset}" v2="${tris[i + 1] + vOffset}" v3="${tris[i + 2] + vOffset}"${colorAttr}/>`);
    const n = tris.length / 3;
    if (n) volumes.push({ first: tCount, last: tCount + n - 1, name: part.name, extruder: slot + 1 });
    vOffset += verts.length / 3;
    tCount += n;
  }
  const materials = mode === 'multicolor'
    ? `<basematerials id="1">${slots.map((c, i) => `<base name="Farbe ${i + 1}" displaycolor="${c.toUpperCase()}FF"/>`).join('')}</basematerials>`
    : '';
  const objAttr = mode === 'multicolor' ? ' pid="1" pindex="0"' : '';
  const model = `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="de-DE" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:slic3rpe="http://schemas.slic3r.org/3mf/2017/06">
<metadata name="Application">PartJoin</metadata>
<resources>${materials}<object id="2" name="PartJoin" type="model"${objAttr}><mesh><vertices>${vChunks.join('')}</vertices><triangles>${tChunks.join('')}</triangles></mesh></object></resources>
<build><item objectid="2"/></build>
</model>`;
  const contentTypes = `<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
<Default Extension="config" ContentType="text/xml"/>
</Types>`;
  const rels = `<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>`;
  const files: Record<string, Uint8Array> = {
    '[Content_Types].xml': strToU8(contentTypes),
    '_rels/.rels': strToU8(rels),
    '3D/3dmodel.model': strToU8(model),
  };
  if (mode === 'multicolor') {
    // PrusaSlicer-Format (von Bambu Studio und OrcaSlicer ebenfalls gelesen): ein Objekt, Teile als
    // Dreiecksbereiche mit eigenem Namen und Extruder
    const vol = volumes
      .map((v) => `  <volume firstid="${v.first}" lastid="${v.last}">
   <metadata type="volume" key="name" value="${esc(v.name)}"/>
   <metadata type="volume" key="volume_type" value="ModelPart"/>
   <metadata type="volume" key="extruder" value="${v.extruder}"/>
  </volume>`)
      .join('\n');
    files['Metadata/Slic3r_PE_model.config'] = strToU8(`<?xml version="1.0" encoding="UTF-8"?>
<config>
 <object id="2" instances_count="1">
  <metadata type="object" key="name" value="PartJoin"/>
${vol}
 </object>
</config>
`);
  }
  return zipSync(files);
}

/** Gleichbleibende Pseudo-UUID (Bambu Studio erwartet p:UUID-Attribute, Inhalt ist beliebig). */
function uuid(kind: number, n: number): string {
  const h = (x: number, len: number) => x.toString(16).padStart(len, '0').slice(-len);
  return `${h(n, 8)}-${h(kind, 4)}-4000-8000-${h(n * 7919 + kind, 12)}`;
}

export interface BambuOptions {
  /** Mitte der Druckplatte (mm), Standard 128/128 (X1, P1, A1). */
  bedCenter?: [number, number];
  /** Versionsangabe für Bambu Studio (Format „BambuStudio-xx.xx.xx.xx"). */
  version?: string;
}

/**
 * Projektformat von Bambu Studio / OrcaSlicer: ein Objekt aus mehreren Teilen (je Teil ein Netz in
 * 3D/Objects/object_1.model), Filament je Teil in Metadata/model_settings.config. Ohne diese
 * Kennzeichnung laden beide Slicer nur die Geometrie. Die Farben selbst übernimmt der Slicer aus den
 * eingestellten Filamenten – Teile gleicher Farbe bekommen dieselbe Filament-Nummer.
 */
export function write3MFBambu(parts: ColoredPart[], opts: BambuOptions = {}): Uint8Array {
  const slots = colorSlots(parts);
  // Baugruppe mittig auf die Platte, Unterseite auf z = 0
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (const p of parts)
    for (let i = 0; i < p.positions.length; i += 3)
      for (let k = 0; k < 3; k++) {
        min[k] = Math.min(min[k], p.positions[i + k]);
        max[k] = Math.max(max[k], p.positions[i + k]);
      }
  const c = [(min[0] + max[0]) / 2, (min[1] + max[1]) / 2, min[2]];
  const [bx, by] = opts.bedCenter ?? [128, 128];
  const objId = parts.length + 1;

  const sub: string[] = [];
  const components: string[] = [];
  const settings: string[] = [];
  parts.forEach((part, i) => {
    const id = i + 1;
    const local = new Float32Array(part.positions.length);
    for (let k = 0; k < local.length; k++) local[k] = part.positions[k] - c[k % 3];
    const { verts, tris } = weld(local);
    const v: string[] = [];
    for (let k = 0; k < verts.length; k += 3) v.push(`<vertex x="${f5(verts[k])}" y="${f5(verts[k + 1])}" z="${f5(verts[k + 2])}"/>`);
    const t: string[] = [];
    for (let k = 0; k < tris.length; k += 3) t.push(`<triangle v1="${tris[k]}" v2="${tris[k + 1]}" v3="${tris[k + 2]}"/>`);
    sub.push(`  <object id="${id}" p:UUID="${uuid(1, id)}" type="model">
   <mesh><vertices>${v.join('')}</vertices><triangles>${t.join('')}</triangles></mesh>
  </object>`);
    components.push(`    <component p:path="/3D/Objects/object_1.model" objectid="${id}" p:UUID="${uuid(2, id)}" transform="1 0 0 0 1 0 0 0 1 0 0 0"/>`);
    const extruder = Math.max(0, slots.indexOf(part.color.toLowerCase())) + 1;
    settings.push(`    <part id="${id}" subtype="normal_part">
      <metadata key="name" value="${esc(part.name)}"/>
      <metadata key="matrix" value="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"/>
      <metadata key="extruder" value="${extruder}"/>
    </part>`);
  });
  const ns = 'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p"';
  const version = opts.version ?? 'BambuStudio-01.09.00.70';
  const main = `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" ${ns}>
 <metadata name="Application">${version}</metadata>
 <metadata name="BambuStudio:3mfVersion">1</metadata>
 <metadata name="Title">PartJoin</metadata>
 <metadata name="Designer">PartJoin</metadata>
 <resources>
  <object id="${objId}" p:UUID="${uuid(3, objId)}" type="model">
   <components>
${components.join('\n')}
   </components>
  </object>
 </resources>
 <build p:UUID="${uuid(4, 1)}">
  <item objectid="${objId}" p:UUID="${uuid(5, objId)}" transform="1 0 0 0 1 0 0 0 1 ${f5(bx)} ${f5(by)} 0" printable="1"/>
 </build>
</model>`;
  const objects = `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" ${ns}>
 <metadata name="BambuStudio:3mfVersion">1</metadata>
 <resources>
${sub.join('\n')}
 </resources>
 <build/>
</model>`;
  const config = `<?xml version="1.0" encoding="UTF-8"?>
<config>
  <object id="${objId}">
    <metadata key="name" value="PartJoin"/>
    <metadata key="extruder" value="1"/>
${settings.join('\n')}
  </object>
  <plate>
    <metadata key="plater_id" value="1"/>
    <metadata key="plater_name" value=""/>
    <metadata key="locked" value="false"/>
    <model_instance>
      <metadata key="object_id" value="${objId}"/>
      <metadata key="instance_id" value="0"/>
      <metadata key="identify_id" value="${100 + objId}"/>
    </model_instance>
  </plate>
  <assemble>
   <assemble_item object_id="${objId}" instance_id="0" transform="1 0 0 0 1 0 0 0 1 ${f5(bx)} ${f5(by)} 0" offset="0 0 0" />
  </assemble>
</config>
`;
  const contentTypes = `<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
 <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
 <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
 <Default Extension="png" ContentType="image/png"/>
 <Default Extension="gcode" ContentType="text/x.gcode"/>
</Types>`;
  const rels = `<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/3D/3dmodel.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>`;
  const modelRels = `<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
 <Relationship Target="/3D/Objects/object_1.model" Id="rel-1" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>`;
  return zipSync({
    '[Content_Types].xml': strToU8(contentTypes),
    '_rels/.rels': strToU8(rels),
    '3D/3dmodel.model': strToU8(main),
    '3D/_rels/3dmodel.model.rels': strToU8(modelRels),
    '3D/Objects/object_1.model': strToU8(objects),
    'Metadata/model_settings.config': strToU8(config),
  });
}

function concat(list: Float32Array[]): Float32Array {
  const out = new Float32Array(list.reduce((s, p) => s + p.length, 0));
  let o = 0;
  for (const p of list) { out.set(p, o); o += p.length; }
  return out;
}
