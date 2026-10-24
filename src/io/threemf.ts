/**
 * 3MF lesen und schreiben – mit Teilen und Farben.
 *
 * Lesen: Jedes Teil (Netz) wird ein eigenes Teil, Teile eines Objekts bilden eine Gruppe und
 * behalten ihre Lage zueinander. Farben kommen aus dem 3MF-Standard (basematerials, colorgroup)
 * oder aus den Filament-Zuordnungen von Bambu Studio / OrcaSlicer (Metadata/model_settings.config,
 * project_settings.config) bzw. PrusaSlicer (Metadata/Slic3r_PE_model.config, Slic3r_PE.config).
 * Mehrere Modelldateien im Archiv (3D/Objects/*.model, Production-Erweiterung) werden aufgelöst.
 * Modifier (z. B. Aufschriften), Bemalung und Druckeinstellungen je Teil kommen als SlicerExtras mit.
 *
 * Schreiben: entweder ein einziger Körper oder ein Objekt aus mehreren Teilen mit je eigener Farbe
 * und eigenem Extruder – so ordnet der Slicer jedem Teil direkt ein Filament zu.
 *
 * Bewusst ohne DOM: Die XML-Dateien sind maschinell erzeugt und regelmäßig aufgebaut, ein schneller
 * Durchlauf mit regulären Ausdrücken genügt und funktioniert auch in Tests ohne Browser.
 */
import { strFromU8, strToU8, unzipSync, zipSync } from 'fflate';
import { paintStates, remapPaint } from './paint';

/**
 * Slicer-Zusätze eines Teils (Bambu Studio / OrcaSlicer, teils PrusaSlicer). Sie ändern den Druck,
 * nicht die Form, und werden beim Export nach Bambu Studio wieder mitgeschrieben.
 */
export interface SlicerExtras {
  /**
   * Bemalung je Dreieck (gleiche Reihenfolge wie positions, '' = unbemalt): paint_color (Farben),
   * paint_supports, paint_seam, paint_fuzzy_skin. Kodierung siehe paint.ts.
   */
  paint?: Record<string, string[]>;
  /** Filamentfarben zur Farbbemalung: Zustand k = Filament k = palette[k - 1]. */
  palette?: string[];
  /** Modifier (z. B. Aufschriften in anderer Farbe), negative Volumen, Stützen-Volumen. */
  modifiers?: Modifier[];
  /** Eigene Druckeinstellungen des Teils (Schlüssel wie in Bambu Studio, z. B. wall_loops). */
  settings?: Record<string, string>;
  /** Einstellungen seines ursprünglichen Objekts, die nur je Objekt gelten (z. B. layer_height, brim_type). */
  objectSettings?: Record<string, string>;
}

export interface Modifier {
  name: string;
  /** Art wie in Bambu Studio: modifier_part, negative_part, support_enforcer, support_blocker */
  subtype: string;
  /** Nicht indizierte Dreiecke im selben Koordinatensystem wie das Teil. */
  positions: Float32Array;
  /** Filamentfarbe, falls der Modifier ein eigenes Filament vorgibt. */
  color?: string;
  /** Übrige Einstellungen des Modifiers. */
  settings: Record<string, string>;
  /**
   * Nur bei Text (Bambu Studio): Textangaben (das text_info-Element), das Netz in eigenen Koordinaten
   * und seine Lage (4×4 spaltenweise, eigene Koordinaten → Koordinaten von positions). Damit bleibt der
   * Text nach dem Export in Bambu Studio bearbeitbar – es erzeugt ihn in diesen Koordinaten neu.
   */
  textInfo?: string;
  local?: Float32Array;
  transform?: number[];
}

export interface ImportedPart extends SlicerExtras {
  name: string;
  /** Nicht indizierte Dreiecke (9 Werte pro Dreieck) in Millimetern, Lage aus der Datei. */
  positions: Float32Array;
  /** Farbe als '#rrggbb', falls die Datei eine angibt. */
  color?: string;
  /** Teile desselben Objekts haben dieselbe Gruppe. */
  group: string;
}

export interface ColoredPart extends SlicerExtras {
  name: string;
  /** Teile mit gleichem Wert werden ein Objekt im Slicer (Bambu-Export); ohne Angabe ein gemeinsames. */
  object?: string;
  /** Name dieses Objekts im Slicer. */
  objectName?: string;
  /** Nicht indizierte Dreiecke in Weltkoordinaten (mm); Modifier ebenso. */
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
  /** Bemalung je Dreieck, nur vorhandene Arten */
  paint: Record<string, string[]>;
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

/** Farben für Filamente, zu denen die Datei keine Farbe nennt. */
const FALLBACK_COLORS = ['#ffffff', '#1f1f1f', '#e53935', '#1e88e5', '#43a047', '#fdd835', '#fb8c00', '#8e24aa'];

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

/** 3MF-Matrix (Zeilenvektoren, Verschiebung in Modelleinheiten) → 4×4 spaltenweise in mm. */
function m12ToMatrix(tf: M12, scale: number): number[] {
  return [tf[0], tf[1], tf[2], 0, tf[3], tf[4], tf[5], 0, tf[6], tf[7], tf[8], 0, tf[9] * scale, tf[10] * scale, tf[11] * scale, 1];
}

/** 4×4 spaltenweise → 3MF-Matrix (transform-Attribut). */
const matrixToM12 = (e: number[]) => [e[0], e[1], e[2], e[4], e[5], e[6], e[8], e[9], e[10], e[12], e[13], e[14]];

function normColor(c: string | undefined): string | undefined {
  if (!c) return undefined;
  const m = /^#?([0-9a-fA-F]{6})([0-9a-fA-F]{2})?$/.exec(c.trim());
  return m ? `#${m[1].toLowerCase()}` : undefined;
}

/** Bemalungs-Attribute der Dreiecke (Bambu Studio / OrcaSlicer bzw. PrusaSlicer) → Name in Bambu Studio. */
const PAINT_ATTRS: Record<string, string> = {
  paint_color: 'paint_color',
  paint_supports: 'paint_supports',
  paint_seam: 'paint_seam',
  paint_fuzzy_skin: 'paint_fuzzy_skin',
  'slic3rpe:mmu_segmentation': 'paint_color',
  'slic3rpe:custom_supports': 'paint_supports',
  'slic3rpe:custom_seam': 'paint_seam',
};

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
  const paint: Record<string, string[]> = {};
  const tre = /<(?:\w+:)?triangle\b([^>]*?)\/?>/g;
  while ((m = tre.exec(body))) {
    const a = attrs(m[1]);
    const t = pids.length;
    ts.push(Number(a.v1), Number(a.v2), Number(a.v3));
    pids.push(a.pid !== undefined ? Number(a.pid) : -1);
    p1s.push(a.p1 !== undefined ? Number(a.p1) : -1);
    for (const k in a) {
      const kind = PAINT_ATTRS[k];
      if (kind && a[k]) (paint[kind] ??= [])[t] = a[k].toUpperCase();
    }
  }
  for (const kind in paint) {
    const list = paint[kind];
    for (let t = 0; t < pids.length; t++) list[t] ??= '';
  }
  return { verts: Float64Array.from(vs), tris: Uint32Array.from(ts), triPid: Int32Array.from(pids), triP1: Int32Array.from(p1s), paint };
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
  /** Druckeinstellungen des Objekts, die auch je Teil gelten können (REGION_KEYS) */
  settings: Record<string, string>;
  /** Einstellungen, die nur je Objekt gelten (OBJECT_KEYS) */
  objectSettings: Record<string, string>;
  /** Bambu/Orca: Teil-ID (= objectid der Komponente) → Name/Extruder/Art/Einstellungen */
  parts: Map<string, { name?: string; extruder?: number; subtype?: string; settings: Record<string, string>; textInfo?: string }>;
  /** PrusaSlicer: Dreiecksbereiche im Netz des Objekts */
  volumes: { first: number; last: number; name?: string; extruder?: number; subtype?: string }[];
}

/**
 * Einstellungen, die Bambu Studio je Teil oder Modifier erlaubt (PrintRegionConfig, Bambu Studio
 * 2.8). Objekt-Einstellungen wie die Schichthöhe gelten für ein ganzes Objekt und lassen sich nach
 * dem Zusammenfügen zu einem Objekt nicht mehr je Teil festlegen. Ohne die Filament-Nummern
 * (wall_filament …), die sich beim Export neu ordnen würden.
 */
const REGION_KEYS = new Set((
  'bottom_shell_layers bottom_shell_thickness bridge_angle bridge_flow counterbore_hole_bridging overhang_totally_speed '
  + 'bridge_speed ensure_vertical_shell_thickness top_surface_pattern bottom_surface_pattern monotonic_travel_into_wall '
  + 'top_surface_density bottom_surface_density internal_solid_infill_pattern outer_wall_line_width outer_wall_speed '
  + 'infill_direction symmetric_infill_y_axis infill_shift_step sparse_infill_lattice_angle_1 sparse_infill_lattice_angle_2 '
  + 'infill_rotate_step skeleton_infill_density skin_infill_density sparse_infill_density fill_multiline infill_lock_depth '
  + 'skin_infill_depth sparse_infill_pattern locked_skin_infill_pattern locked_skeleton_infill_pattern fuzzy_skin '
  + 'fuzzy_skin_thickness fuzzy_skin_point_distance fuzzy_skin_first_layer fuzzy_skin_noise_type fuzzy_skin_scale '
  + 'fuzzy_skin_octaves fuzzy_skin_persistence fuzzy_skin_mode gap_infill_speed sparse_infill_line_width '
  + 'skin_infill_line_width skeleton_infill_line_width infill_wall_overlap sparse_infill_speed infill_combination '
  + 'ironing_type ironing_pattern ironing_flow ironing_spacing ironing_inset ironing_direction ironing_speed '
  + 'detect_overhang_wall smooth_speed_discontinuity_area smooth_coefficient inner_wall_line_width inner_wall_speed '
  + 'wall_loops minimum_sparse_infill_area internal_solid_infill_line_width internal_solid_infill_speed detect_thin_wall '
  + 'top_surface_line_width top_shell_layers top_shell_thickness top_surface_speed small_perimeter_speed '
  + 'small_perimeter_threshold vertical_shell_speed top_color_penetration_layers bottom_color_penetration_layers '
  + 'infill_instead_top_bottom_surfaces wall_sequence enable_overhang_speed overhang_1_4_speed overhang_2_4_speed '
  + 'overhang_3_4_speed overhang_4_4_speed enable_height_slowdown slowdown_start_height slowdown_start_speed '
  + 'slowdown_start_acc slowdown_end_height slowdown_end_speed slowdown_end_acc sparse_infill_anchor '
  + 'sparse_infill_anchor_max top_solid_infill_flow_ratio initial_layer_flow_ratio filter_out_gap_fill precise_outer_wall '
  + 'print_flow_ratio override_filament_scarf_seam_setting seam_slope_type seam_slope_conditional seam_slope_start_height '
  + 'seam_slope_gap seam_slope_entire_loop seam_slope_min_length seam_slope_steps seam_slope_inner_walls '
  + 'embedding_wall_into_infill alternate_extra_wall'
).split(' '));

const regionSettings = (md: Record<string, string>) =>
  Object.fromEntries(Object.entries(md).filter(([k]) => REGION_KEYS.has(k)));

/**
 * Einstellungen, die in Bambu Studio nur für ein ganzes Objekt gelten (PrintObjectConfig, Bambu Studio
 * 2.8), z. B. Schichthöhe, Brim, Stützen. Sie bleiben erhalten, solange ein exportiertes Objekt nur
 * Teile mit denselben Werten enthält. Ohne die Filament-Nummern (support_filament …).
 */
const OBJECT_KEYS = new Set((
  'brim_object_gap brim_type brim_width bridge_no_support elefant_foot_compensation max_bridge_length line_width '
  + 'interface_shells layer_height mmu_segmented_region_max_width mmu_segmented_region_interlocking_depth '
  + 'raft_contact_distance raft_expansion raft_first_layer_density raft_first_layer_expansion raft_layers '
  + 'seam_position seam_placement_away_from_overhangs slice_closing_radius slicing_mode enable_support support_type '
  + 'support_angle support_on_build_plate_only support_critical_regions_only support_remove_small_overhang '
  + 'support_top_z_distance support_bottom_z_distance enforce_support_layers support_line_width '
  + 'support_interface_not_for_body support_interface_loop_pattern '
  + 'support_interface_top_layers support_interface_bottom_layers support_interface_spacing enable_support_ironing '
  + 'support_ironing_pattern support_ironing_flow support_ironing_spacing support_ironing_inset '
  + 'support_ironing_direction support_ironing_speed support_interface_speed support_base_pattern '
  + 'support_interface_pattern support_base_pattern_spacing support_expansion support_speed support_style '
  + 'thick_bridges support_threshold_angle support_object_xy_distance support_object_first_layer_gap '
  + 'xy_hole_compensation xy_contour_compensation enable_circle_compensation circle_compensation_manual_offset '
  + 'apply_scarf_seam_on_circles flush_into_objects flush_into_infill flush_into_support '
  + 'tree_support_branch_distance tree_support_branch_diameter tree_support_branch_angle '
  + 'tree_support_branch_diameter_angle tree_support_wall_count detect_narrow_internal_solid_infill '
  + 'detect_floating_vertical_shell support_bottom_interface_spacing internal_bridge_support_thickness '
  + 'wall_generator wall_transition_length wall_transition_filter_deviation wall_transition_angle '
  + 'wall_distribution_count min_feature_size min_bead_width top_one_wall_type top_area_threshold '
  + 'only_one_wall_first_layer seam_gap wipe_speed role_base_wipe_speed precise_z_height interlocking_beam '
  + 'interlocking_beam_width interlocking_orientation interlocking_beam_layer_count interlocking_depth '
  + 'interlocking_boundary_avoidance scarf_angle_threshold'
).split(' '));

const objectSettingsOf = (md: Record<string, string>) =>
  Object.fromEntries(Object.entries(md).filter(([k]) => OBJECT_KEYS.has(k)));

/** Teilarten von Bambu Studio, die kein druckbarer Körper sind. */
const MODIFIER_SUBTYPES = new Set(['modifier_part', 'negative_part', 'support_enforcer', 'support_blocker']);

/** Art eines PrusaSlicer-Volumens → Bambu-Bezeichnung. */
const PRUSA_SUBTYPES: Record<string, string> = {
  ModelPart: 'normal_part', ParameterModifier: 'modifier_part', NegativeVolume: 'negative_part',
  SupportEnforcer: 'support_enforcer', SupportBlocker: 'support_blocker',
};

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
      const entry: SlicerObject = {
        name: own.name, extruder: own.extruder ? Number(own.extruder) : undefined, settings: regionSettings(own),
        objectSettings: objectSettingsOf(own), parts: new Map(), volumes: [],
      };
      for (const p of o[2].matchAll(/<part\b([^>]*)>([\s\S]*?)<\/part>/g)) {
        const pa = attrs(p[1]);
        const md = metadata(p[2]);
        if (pa.id) {
          entry.parts.set(pa.id, {
            name: md.name, extruder: md.extruder ? Number(md.extruder) : undefined, subtype: pa.subtype, settings: regionSettings(md),
            textInfo: /<text_info\b[^>]*?\/>/.exec(p[2])?.[0],
          });
        }
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
      const entry: SlicerObject = objects.get(id) ?? { settings: {}, objectSettings: {}, parts: new Map(), volumes: [] };
      entry.name ??= own.name;
      if (own.extruder) entry.extruder ??= Number(own.extruder);
      for (const v of o[2].matchAll(/<volume\b([^>]*)>([\s\S]*?)<\/volume>/g)) {
        const va = attrs(v[1]);
        const md = metadata(v[2]);
        const subtype = md.volume_type ? PRUSA_SUBTYPES[md.volume_type] ?? 'modifier_part' : md.modifier === '1' ? 'modifier_part' : 'normal_part';
        entry.volumes.push({ first: Number(va.firstid), last: Number(va.lastid), name: md.name, extruder: md.extruder ? Number(md.extruder) : undefined, subtype });
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

/**
 * Liest alle Teile einer 3MF-Datei.
 * @param info bekommt den Bambu-Drucker des Projekts (Kennung aus BAMBU_PRINTERS), falls erkennbar
 */
export function read3MF(data: Uint8Array, info?: { printer?: string }): ImportedPart[] {
  const files = unzipSync(data);
  if (info) info.printer = bambuPrinterOf(files);
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
  /** Modifier des gerade gelesenen Objekts; sie hängen am ersten Teil des Objekts. */
  let modifiers: Modifier[] = [];
  const names = new Map<string, number>();
  // Slicer übernehmen oft den Dateinamen samt Endung („Kopf.stl")
  const stripExt = (raw: string) => raw.replace(/\.(stl|obj|3mf|amf|ply|step|stp|glb|gltf|fbx)$/i, '') || raw;
  const uniqueName = (raw: string) => {
    const n = stripExt(raw);
    const k = names.get(n) ?? 0;
    names.set(n, k + 1);
    return k ? `${n} (${k + 1})` : n;
  };

  /** Dreiecke eines Netzes in Millimetern und in der Lage tf (Eckpunkte in der Reihenfolge der Datei). */
  const place = (model: RawModel, mesh: RawMesh, tris: number[], tf: M12) => {
    const s = model.scale;
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
    return out;
  };
  const allTris = (mesh: RawMesh) => Array.from({ length: mesh.tris.length / 3 }, (_, t) => t);

  /** Bemalung der gewählten Dreiecke (nur Arten, die dort vorkommen). */
  const paintOf = (mesh: RawMesh, tris: number[]) => {
    let out: Record<string, string[]> | undefined;
    for (const kind in mesh.paint) {
      const list = tris.map((t) => mesh.paint[kind][t]);
      if (list.some(Boolean)) (out ??= {})[kind] = list;
    }
    return out;
  };
  /** Filamentfarben zur Farbbemalung, ergänzt um Vorgaben für Filamente ohne Farbangabe. */
  const paletteFor = (paint: Record<string, string[]>) => {
    const states = new Set<number>();
    for (const c of paint.paint_color ?? []) if (c) paintStates(c, states);
    const palette = [...slicer.colors];
    for (const k of states) for (let i = palette.length; i < k; i++) palette.push(FALLBACK_COLORS[i % FALLBACK_COLORS.length]);
    return palette;
  };

  /** Teil(e) aus einem Netz: nach Dreiecksfarbe bzw. PrusaSlicer-Volumen getrennt. */
  const emit = (
    model: RawModel, obj: RawObject, tf: M12, group: string, baseName: string,
    fallback: string | undefined, volumes: SlicerObject['volumes'], settings: Record<string, string>,
    objectSettings: Record<string, string>,
  ) => {
    const mesh = obj.mesh!;
    const triCount = mesh.tris.length / 3;
    const objColor = obj.pid !== undefined ? model.colors.get(obj.pid)?.[obj.pindex ?? 0] : undefined;
    // Bereiche: PrusaSlicer-Volumen oder sonst Dreiecke gleicher Farbe
    const keyOf = new Array<string>(triCount);
    const info = new Map<string, { name: string; color?: string; subtype?: string }>();
    /** Farbe eines Dreiecks laut 3MF (pid/p1, sonst Vorgabe des Objekts). */
    const triColor = (t: number) => {
      const pid = mesh.triPid[t] >= 0 ? String(mesh.triPid[t]) : obj.pid;
      const idx = mesh.triP1[t] >= 0 ? mesh.triP1[t] : obj.pindex ?? 0;
      return pid !== undefined ? model.colors.get(pid)?.[idx] : undefined;
    };
    if (volumes.length) {
      volumes.forEach((v, k) => {
        const first = Math.max(0, v.first);
        // Modifier & Co. (PrusaSlicer): eigene Dreiecksbereiche im selben Netz
        const modifier = v.subtype !== undefined && v.subtype !== 'normal_part';
        const key = `${modifier ? 'm' : 'v'}${k}`;
        for (let t = first; t <= Math.min(triCount - 1, v.last); t++) keyOf[t] = key;
        const own = first < triCount ? triColor(first) : undefined;
        info.set(key, modifier
          ? { name: v.name ?? `Modifier ${k + 1}`, color: extruderColor(v.extruder), subtype: v.subtype }
          : { name: v.name ?? `${baseName} ${k + 1}`, color: extruderColor(v.extruder) ?? own ?? fallback ?? objColor });
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
    const several = [...groups.keys()].filter((key) => !key.startsWith('m')).length > 1;
    let k = 0;
    for (const [key, tris] of groups) {
      const i = info.get(key)!;
      const positions = place(model, mesh, tris, tf);
      if (i.subtype) {
        modifiers.push({ name: stripExt(i.name), subtype: i.subtype, positions, color: i.color, settings: {} });
        continue;
      }
      k++;
      const name = several && !key.startsWith('v') ? `${i.name} – Farbe ${k}` : i.name;
      const part: ImportedPart = { name: uniqueName(name), positions, color: i.color, group };
      const paint = paintOf(mesh, tris);
      if (paint) {
        part.paint = paint;
        if (paint.paint_color) part.palette = paletteFor(paint);
      }
      if (Object.keys(settings).length) part.settings = settings;
      if (Object.keys(objectSettings).length) part.objectSettings = objectSettings;
      parts.push(part);
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
      if (part?.subtype && MODIFIER_SUBTYPES.has(part.subtype)) {
        // Modifier, negative Volumen, Stützen-Volumen: kein eigenes Teil, gehört zum Objekt
        const m: Modifier = {
          name: stripExt(part.name ?? obj.name ?? 'Modifier'), subtype: part.subtype,
          positions: place(model, obj.mesh, allTris(obj.mesh), tf), color: extruderColor(part.extruder), settings: part.settings,
        };
        if (part.textInfo) {
          // Text: Netz in eigenen Koordinaten und seine Lage merken (siehe Modifier.textInfo)
          m.textInfo = part.textInfo;
          m.local = place(model, obj.mesh, allTris(obj.mesh), IDENT);
          m.transform = m12ToMatrix(tf, model.scale);
        }
        modifiers.push(m);
      } else {
        const name = part?.name ?? obj.name ?? meta.name ?? 'Teil';
        const color = extruderColor(part?.extruder) ?? meta.color;
        // Einstellungen des Objekts gelten für jedes seiner Teile, eigene des Teils haben Vorrang
        const settings = { ...meta.slicer?.settings, ...part?.settings };
        emit(model, obj, tf, group, name, color, depth === 0 ? meta.slicer?.volumes ?? [] : [], settings, meta.slicer?.objectSettings ?? {});
      }
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
    const first = parts.length;
    modifiers = [];
    walk(model, it.objectid, it.transform, `3mf-${n}`, 0, {
      name: so?.name ?? obj?.name,
      color: extruderColor(so?.extruder),
      slicer: so,
    });
    // Modifier wirken im Slicer auf das ganze Objekt; sie hängen am ersten Teil und bewegen sich mit dessen Gruppe
    if (modifiers.length && parts.length > first) parts[first].modifiers = modifiers;
  });
  if (!parts.length) throw new Error('Die 3MF-Datei enthält keine Dreiecksnetze.');
  return parts;
}

// ---------------------------------------------------------------- Schreiben

const f5 = (x: number) => String(+x.toFixed(5));
const esc = (s: string) => s.replace(/[<>&"]/g, (c) => ({ '<': '&lt;', '>': '&gt;', '&': '&amp;', '"': '&quot;' })[c]!);

/**
 * Gleiche Eckpunkte zusammenfassen (3MF verlangt ein indiziertes Netz).
 * kept: Nummer des Ausgangsdreiecks je Dreieck des Ergebnisses (entartete fallen weg).
 */
function weld(p: Float32Array): { verts: number[]; tris: number[]; kept: number[] } {
  const map = new Map<string, number>();
  const verts: number[] = [];
  const tris: number[] = [];
  const kept: number[] = [];
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
    kept.push(t / 9);
  }
  return { verts, tris, kept };
}

/** Filamente, mit denen ein Teil bemalt ist (ohne 0 = Filament des Teils). */
function paintedStates(p: SlicerExtras): number[] {
  const states = new Set<number>();
  for (const c of p.paint?.paint_color ?? []) if (c) paintStates(c, states);
  states.delete(0);
  return [...states].sort((a, b) => a - b);
}

/**
 * Farben → Extrudernummern (1, 2, …) in der Reihenfolge des ersten Auftretens: erst die Farben der
 * Teile, dann die zusätzlichen aus Bemalung und Modifiern.
 */
export function colorSlots(parts: ColoredPart[]): string[] {
  const list: string[] = [];
  const add = (c: string | undefined) => {
    if (c && !list.includes(c.toLowerCase())) list.push(c.toLowerCase());
  };
  for (const p of parts) add(p.color);
  for (const p of parts) {
    for (const k of paintedStates(p)) add(p.palette?.[k - 1]);
    for (const m of p.modifiers ?? []) add(m.color);
  }
  return list;
}

/** Namen der Bemalungs-Attribute je Slicer-Format. */
const BAMBU_PAINT: Record<string, string> = {
  paint_color: 'paint_color', paint_supports: 'paint_supports', paint_seam: 'paint_seam', paint_fuzzy_skin: 'paint_fuzzy_skin',
};
const PRUSA_PAINT: Record<string, string> = {
  paint_color: 'slic3rpe:mmu_segmentation', paint_supports: 'slic3rpe:custom_supports', paint_seam: 'slic3rpe:custom_seam',
};

/**
 * Bemalungs-Attribute eines Dreiecks für den Export (src = Dreieck im Teil). Die Farbbemalung wird auf
 * die Filamentnummern des Exports umgestellt, die übrige bleibt, wie sie ist.
 */
function paintWriter(part: ColoredPart, slots: string[], names: Record<string, string>): ((src: number) => string) | undefined {
  const paint = part.paint;
  if (!paint) return undefined;
  const slotOf = (k: number) => {
    const c = part.palette?.[k - 1];
    const i = c ? slots.indexOf(c.toLowerCase()) : -1;
    return i >= 0 ? i + 1 : k;
  };
  const kinds = Object.keys(paint).filter((k) => names[k]);
  return (src) => {
    let out = '';
    for (const kind of kinds) {
      const code = paint[kind][src];
      const v = code && (kind === 'paint_color' ? remapPaint(code, (k) => (k ? slotOf(k) : 0)) : code);
      if (v) out += ` ${names[kind]}="${v}"`;
    }
    return out;
  };
}

const settingLines = (settings: Record<string, string> | undefined, indent: string) =>
  Object.entries(settings ?? {}).map(([k, v]) => `\n${indent}<metadata key="${esc(k)}" value="${esc(v)}"/>`).join('');

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
    const { verts, tris, kept } = weld(part.positions);
    for (let i = 0; i < verts.length; i += 3) vChunks.push(`<vertex x="${f5(verts[i])}" y="${f5(verts[i + 1])}" z="${f5(verts[i + 2])}"/>`);
    const slot = Math.max(0, slots.indexOf(part.color.toLowerCase()));
    const colorAttr = mode === 'multicolor' ? ` pid="1" p1="${slot}"` : '';
    // Bemalung im PrusaSlicer-Format (andere Programme übergehen die Attribute). Modifier fehlen hier
    // bewusst: Programme ohne Slicer-Metadaten würden sie als Teil des Körpers drucken.
    const paint = mode === 'multicolor' ? paintWriter(part, slots, PRUSA_PAINT) : undefined;
    for (let i = 0; i < tris.length; i += 3)
      tChunks.push(`<triangle v1="${tris[i] + vOffset}" v2="${tris[i + 1] + vOffset}" v3="${tris[i + 2] + vOffset}"${colorAttr}${paint?.(kept[i / 3]) ?? ''}/>`);
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

/** Bambu-Drucker mit genau einer Düse: Namen der System-Profile (Bambu Studio ab 2.0). */
export interface BambuPrinter {
  id: string;
  label: string;
  /** printer_model in Bambu Studio */
  model: string;
  printer: string;
  process: string;
  filament: string;
  /** Druckbereich in mm */
  bed: [number, number];
  /** Drucker mit zwei Düsen: Art jedes Extruders (wie im Druckerprofil von Bambu Studio). */
  extruders?: string[];
}

export const BAMBU_PRINTERS: BambuPrinter[] = [
  { id: 'X1C', label: 'X1 Carbon', model: 'Bambu Lab X1 Carbon', printer: 'Bambu Lab X1 Carbon 0.4 nozzle', process: '0.20mm Standard @BBL X1C', filament: 'Bambu PLA Basic @BBL X1C', bed: [256, 256] },
  { id: 'X1', label: 'X1', model: 'Bambu Lab X1', printer: 'Bambu Lab X1 0.4 nozzle', process: '0.20mm Standard @BBL X1C', filament: 'Bambu PLA Basic @BBL X1', bed: [256, 256] },
  { id: 'X1E', label: 'X1E', model: 'Bambu Lab X1E', printer: 'Bambu Lab X1E 0.4 nozzle', process: '0.20mm Standard @BBL X1C', filament: 'Bambu PLA Basic @BBL X1C', bed: [256, 256] },
  { id: 'P1S', label: 'P1S', model: 'Bambu Lab P1S', printer: 'Bambu Lab P1S 0.4 nozzle', process: '0.20mm Standard @BBL X1C', filament: 'Bambu PLA Basic @BBL P1S 0.4 nozzle', bed: [256, 256] },
  { id: 'P1P', label: 'P1P', model: 'Bambu Lab P1P', printer: 'Bambu Lab P1P 0.4 nozzle', process: '0.20mm Standard @BBL P1P', filament: 'Bambu PLA Basic @BBL P1P', bed: [256, 256] },
  { id: 'P2S', label: 'P2S', model: 'Bambu Lab P2S', printer: 'Bambu Lab P2S 0.4 nozzle', process: '0.20mm Standard @BBL P2S', filament: 'Bambu PLA Basic @BBL P2S', bed: [256, 256] },
  { id: 'A1', label: 'A1', model: 'Bambu Lab A1', printer: 'Bambu Lab A1 0.4 nozzle', process: '0.20mm Standard @BBL A1', filament: 'Bambu PLA Basic @BBL A1', bed: [256, 256] },
  { id: 'A1M', label: 'A1 mini', model: 'Bambu Lab A1 mini', printer: 'Bambu Lab A1 mini 0.4 nozzle', process: '0.20mm Standard @BBL A1M', filament: 'Bambu PLA Basic @BBL A1M', bed: [180, 180] },
  { id: 'A2L', label: 'A2L', model: 'Bambu Lab A2L', printer: 'Bambu Lab A2L 0.4 nozzle', process: '0.20mm Standard @BBL A2L', filament: 'Bambu PLA Basic @BBL A2L 0.4 nozzle', bed: [330, 320] },
  { id: 'H2S', label: 'H2S', model: 'Bambu Lab H2S', printer: 'Bambu Lab H2S 0.4 nozzle', process: '0.20mm Standard @BBL H2S', filament: 'Bambu PLA Basic @BBL H2S', bed: [340, 320] },
  { id: 'H2C', label: 'H2C', model: 'Bambu Lab H2C', printer: 'Bambu Lab H2C 0.4 nozzle', process: '0.20mm Standard @BBL H2C', filament: 'Bambu PLA Basic @BBL H2C', bed: [330, 320], extruders: ['Direct Drive', 'Direct Drive'] },
  { id: 'H2D', label: 'H2D', model: 'Bambu Lab H2D', printer: 'Bambu Lab H2D 0.4 nozzle', process: '0.20mm Standard @BBL H2D', filament: 'Bambu PLA Basic @BBL H2D', bed: [350, 320], extruders: ['Direct Drive', 'Direct Drive'] },
  { id: 'H2DP', label: 'H2D Pro', model: 'Bambu Lab H2D Pro', printer: 'Bambu Lab H2D Pro 0.4 nozzle', process: '0.20mm Standard @BBL H2DP', filament: 'Bambu PLA Basic @BBL H2DP', bed: [350, 320], extruders: ['Direct Drive', 'Direct Drive'] },
  { id: 'X2D', label: 'X2D', model: 'Bambu Lab X2D', printer: 'Bambu Lab X2D 0.4 nozzle', process: '0.20mm Standard @BBL X2D', filament: 'Bambu PLA Basic @BBL X2D 0.4 nozzle', bed: [256, 256], extruders: ['Direct Drive', 'Bowden'] },
];

export interface BambuOptions {
  /** Kennung aus BAMBU_PRINTERS, Standard X1C. */
  printer?: string;
  /** Versionsangabe für Bambu Studio (Format „BambuStudio-xx.xx.xx.xx"). */
  version?: string;
}

/**
 * Projekteinstellungen (Metadata/project_settings.config). Ohne sie lädt Bambu Studio ab 2.8.2 nur
 * die Geometrie („ungültige Konfiguration"). Es genügen die Namen der System-Profile – Bambu Studio
 * übernimmt alle übrigen Werte aus diesen Profilen – plus die Filamentfarben, die Projektsache sind.
 * Die Listen je Filament müssen gleich lang sein, sonst kürzt Bambu Studio die Filamente.
 */
function bambuProjectSettings(printer: BambuPrinter, colors: string[], version: string): string {
  const n = Math.max(1, colors.length);
  const list = colors.length ? colors.map((c) => c.toUpperCase()) : ['#FFFFFF'];
  const each = (v: string) => Array.from({ length: n }, () => v);
  return JSON.stringify({
    name: 'project_settings',
    from: 'project',
    version,
    printer_technology: 'FFF',
    printer_model: printer.model,
    printer_settings_id: printer.printer,
    print_settings_id: printer.process,
    nozzle_diameter: (printer.extruders ?? ['']).map(() => '0.4'),
    // Zwei Düsen: Art und Volumen je Düse wie im Druckerprofil, Filamente verteilt Bambu Studio selbst
    ...(printer.extruders
      ? { extruder_type: printer.extruders, nozzle_volume_type: printer.extruders.map(() => 'Standard'), filament_map_mode: 'Auto For Flush' }
      : {}),
    filament_settings_id: each(printer.filament),
    filament_colour: list,
    filament_multi_colour: list,
    filament_colour_type: each('1'),
    filament_type: each('PLA'),
    filament_ids: each('GFA00'),
    filament_diameter: each('1.75'),
    filament_extruder_variant: each('Direct Drive Standard'),
    filament_self_index: list.map((_, i) => String(i + 1)),
    filament_map: each('1'),
    inherits_group: Array.from({ length: n + 2 }, () => ''),
    different_settings_to_system: Array.from({ length: n + 2 }, () => ''),
  }, null, 4);
}

/** Bambu-Drucker eines Projekts (printer_model aus den Projekteinstellungen), falls bekannt. */
export function bambuPrinterOf(files: Record<string, Uint8Array>): string | undefined {
  const project = files['Metadata/project_settings.config'];
  if (!project) return undefined;
  try {
    const model = JSON.parse(strFromU8(project)).printer_model;
    return BAMBU_PRINTERS.find((p) => p.model === model)?.id;
  } catch {
    return undefined;
  }
}

/**
 * Projektformat von Bambu Studio / OrcaSlicer: ein Objekt aus mehreren Teilen (je Teil ein Netz in
 * 3D/Objects/object_1.model), Filament je Teil in Metadata/model_settings.config, Drucker,
 * Profile und Filamentfarben in Metadata/project_settings.config. Teile gleicher Farbe bekommen
 * dieselbe Filament-Nummer. Modifier werden weitere Teile des Objekts (subtype), Bemalung steht an
 * den Dreiecken, Einstellungen je Teil in model_settings.config.
 */
export function write3MFBambu(parts: ColoredPart[], opts: BambuOptions = {}): Uint8Array {
  const slots = colorSlots(parts);
  const box = (list: ColoredPart[]) => {
    const min = [Infinity, Infinity, Infinity];
    const max = [-Infinity, -Infinity, -Infinity];
    for (const p of list)
      for (let i = 0; i < p.positions.length; i += 3)
        for (let k = 0; k < 3; k++) {
          min[k] = Math.min(min[k], p.positions[i + k]);
          max[k] = Math.max(max[k], p.positions[i + k]);
        }
    return { min, max };
  };
  // Alles zusammen mittig auf die Platte, Unterseite auf z = 0; die Objekte liegen zueinander wie in PartJoin
  const all = box(parts);
  const c = [(all.min[0] + all.max[0]) / 2, (all.min[1] + all.max[1]) / 2, all.min[2]];
  const printer = BAMBU_PRINTERS.find((p) => p.id === opts.printer) ?? BAMBU_PRINTERS[0];
  const [bx, by] = [printer.bed[0] / 2, printer.bed[1] / 2];
  const groups = new Map<string, ColoredPart[]>();
  for (const p of parts) {
    const k = p.object ?? '';
    const list = groups.get(k);
    if (list) list.push(p);
    else groups.set(k, [p]);
  }

  const identity = '1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1';
  const files: Record<string, Uint8Array> = {};
  const mainObjects: string[] = [];
  const items: string[] = [];
  const configObjects: string[] = [];
  const instances: string[] = [];
  const assemble: string[] = [];
  const subRels: string[] = [];
  const ns = 'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02" xmlns:BambuStudio="http://schemas.bambulab.com/package/2021" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06" requiredextensions="p"';
  let nextId = 1;
  let fileNo = 0;
  for (const list of groups.values()) {
    // Ursprung des Objekts: Mitte seiner Grundfläche, Unterseite
    const b = box(list);
    const o = [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, b.min[2]];
    const path = `3D/Objects/object_${++fileNo}.model`;
    const sub: string[] = [];
    const components: string[] = [];
    const settings: string[] = [];
    /**
     * Netz als eigenes Objekt in der Unterdatei plus Komponente im Objekt. Mit place (4×4, spaltenweise,
     * Weltkoordinaten) bleibt das Netz in eigenen Koordinaten und die Komponente bekommt diese Lage.
     */
    const meshObject = (positions: Float32Array, triAttrs?: (src: number) => string, place?: number[]) => {
      const id = nextId++;
      const local = new Float32Array(positions.length);
      if (place) local.set(positions);
      else for (let k = 0; k < local.length; k++) local[k] = positions[k] - o[k % 3];
      const transform = place
        ? matrixToM12([...place.slice(0, 12), place[12] - o[0], place[13] - o[1], place[14] - o[2], 1]).map(f5).join(' ')
        : '1 0 0 0 1 0 0 0 1 0 0 0';
      const { verts, tris, kept } = weld(local);
      const v: string[] = [];
      for (let k = 0; k < verts.length; k += 3) v.push(`<vertex x="${f5(verts[k])}" y="${f5(verts[k + 1])}" z="${f5(verts[k + 2])}"/>`);
      const t: string[] = [];
      for (let k = 0; k < tris.length; k += 3) t.push(`<triangle v1="${tris[k]}" v2="${tris[k + 1]}" v3="${tris[k + 2]}"${triAttrs?.(kept[k / 3]) ?? ''}/>`);
      sub.push(`  <object id="${id}" p:UUID="${uuid(1, id)}" type="model">
   <mesh><vertices>${v.join('')}</vertices><triangles>${t.join('')}</triangles></mesh>
  </object>`);
      components.push(`    <component p:path="/${path}" objectid="${id}" p:UUID="${uuid(2, id)}" transform="${transform}"/>`);
      return id;
    };
    for (const part of list) {
      const id = meshObject(part.positions, paintWriter(part, slots, BAMBU_PAINT));
      const extruder = Math.max(0, slots.indexOf(part.color.toLowerCase())) + 1;
      settings.push(`    <part id="${id}" subtype="normal_part">
      <metadata key="name" value="${esc(part.name)}"/>
      <metadata key="matrix" value="${identity}"/>
      <metadata key="extruder" value="${extruder}"/>${settingLines(part.settings, '      ')}
    </part>`);
    }
    // Modifier nach den Teilen; im Slicer wirken sie auf alle Teile des Objekts, die sie berühren
    for (const part of list) {
      for (const m of part.modifiers ?? []) {
        // Text: in eigenen Koordinaten mit Lage, damit Bambu Studio ihn bearbeiten kann
        const text = m.textInfo && m.local && m.transform ? m : undefined;
        const id = text ? meshObject(text.local!, undefined, text.transform) : meshObject(m.positions);
        const slot = m.color ? slots.indexOf(m.color.toLowerCase()) + 1 : 0;
        const matrix = text ? rowMajor(text.transform!, o) : identity;
        settings.push(`    <part id="${id}" subtype="${esc(m.subtype)}">
      <metadata key="name" value="${esc(m.name)}"/>
      <metadata key="matrix" value="${matrix}"/>${slot ? `\n      <metadata key="extruder" value="${slot}"/>` : ''}${settingLines(m.settings, '      ')}${text ? `\n      ${text.textInfo}` : ''}
    </part>`);
      }
    }
    files[path] = strToU8(`<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" ${ns}>
 <metadata name="BambuStudio:3mfVersion">1</metadata>
 <resources>
${sub.join('\n')}
 </resources>
 <build/>
</model>`);
    subRels.push(` <Relationship Target="/${path}" Id="rel-${fileNo}" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>`);

    const objId = nextId++;
    const place = `1 0 0 0 1 0 0 0 1 ${f5(o[0] - c[0] + bx)} ${f5(o[1] - c[1] + by)} ${f5(o[2] - c[2])}`;
    mainObjects.push(`  <object id="${objId}" p:UUID="${uuid(3, objId)}" type="model">
   <components>
${components.join('\n')}
   </components>
  </object>`);
    items.push(`  <item objectid="${objId}" p:UUID="${uuid(5, objId)}" transform="${place}" printable="1"/>`);
    const name = list[0].objectName ?? (groups.size === 1 ? 'PartJoin' : list[0].name);
    const firstSlot = Math.max(0, slots.indexOf(list[0].color.toLowerCase())) + 1;
    configObjects.push(`  <object id="${objId}">
    <metadata key="name" value="${esc(name)}"/>
    <metadata key="extruder" value="${firstSlot}"/>${settingLines(commonSettings(list), '    ')}
${settings.join('\n')}
  </object>`);
    instances.push(`    <model_instance>
      <metadata key="object_id" value="${objId}"/>
      <metadata key="instance_id" value="0"/>
      <metadata key="identify_id" value="${100 + objId}"/>
    </model_instance>`);
    assemble.push(`   <assemble_item object_id="${objId}" instance_id="0" transform="${place}" offset="0 0 0" />`);
  }

  // Ab 2.0, sonst passt Bambu Studio die Einstellungen alter Projekte an (Prime Tower)
  const version = opts.version ?? 'BambuStudio-02.00.00.00';
  const main = `<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xml:lang="en-US" ${ns}>
 <metadata name="Application">${version}</metadata>
 <metadata name="BambuStudio:3mfVersion">1</metadata>
 <metadata name="Title">PartJoin</metadata>
 <metadata name="Designer">PartJoin</metadata>
 <resources>
${mainObjects.join('\n')}
 </resources>
 <build p:UUID="${uuid(4, 1)}">
${items.join('\n')}
 </build>
</model>`;
  const config = `<?xml version="1.0" encoding="UTF-8"?>
<config>
${configObjects.join('\n')}
  <plate>
    <metadata key="plater_id" value="1"/>
    <metadata key="plater_name" value=""/>
    <metadata key="locked" value="false"/>
${instances.join('\n')}
  </plate>
  <assemble>
${assemble.join('\n')}
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
${subRels.join('\n')}
</Relationships>`;
  return zipSync({
    '[Content_Types].xml': strToU8(contentTypes),
    '_rels/.rels': strToU8(rels),
    '3D/3dmodel.model': strToU8(main),
    '3D/_rels/3dmodel.model.rels': strToU8(modelRels),
    ...files,
    'Metadata/model_settings.config': strToU8(config),
    'Metadata/project_settings.config': strToU8(bambuProjectSettings(printer, slots, version.replace(/^BambuStudio-/, ''))),
  });
}

/** Lage eines Teils im Objekt (Ursprung o) als Text: 4×4 zeilenweise, wie Bambu Studio „matrix" schreibt. */
function rowMajor(e: number[], o: number[]): string {
  const t = [e[12] - o[0], e[13] - o[1], e[14] - o[2]];
  return [e[0], e[4], e[8], t[0], e[1], e[5], e[9], t[1], e[2], e[6], e[10], t[2], 0, 0, 0, 1].map(f5).join(' ');
}

/**
 * Objekt-Einstellungen eines exportierten Objekts: nur, was alle seine Teile gleich mitbringen. Stammen
 * die Teile aus Objekten mit verschiedener Schichthöhe (oder hat eins keine eigene), gilt das Profil.
 */
function commonSettings(list: ColoredPart[]): Record<string, string> {
  const first = list[0].objectSettings ?? {};
  return Object.fromEntries(Object.entries(first).filter(([k, v]) => list.every((p) => p.objectSettings?.[k] === v)));
}

function concat(list: Float32Array[]): Float32Array {
  const out = new Float32Array(list.reduce((s, p) => s + p.length, 0));
  let o = 0;
  for (const p of list) { out.set(p, o); o += p.length; }
  return out;
}
