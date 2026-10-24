/**
 * CAD-Dateien (STEP, IGES, BREP) über OpenCascade (occt-import-js, WebAssembly): Die exakte Geometrie
 * wird für den Druck fein trianguliert (Sehnenfehler 0,02 mm, höchstens 11° je Facette). Jeder Körper
 * wird ein Teil mit Namen und Farbe aus der Datei; Körper derselben Baugruppe bilden eine Gruppe und
 * behalten ihre Lage zueinander.
 */
export const CAD_FORMATS: Record<string, 'step' | 'iges' | 'brep'> = { stp: 'step', step: 'step', igs: 'iges', iges: 'iges', brp: 'brep', brep: 'brep' };

/** Triangulierung für den Druck (Längen in mm). */
export const CAD_MESHING = { linearUnit: 'millimeter', linearDeflectionType: 'absolute_value', linearDeflection: 0.02, angularDeflection: 0.2 };

export interface OcctNode {
  name: string;
  meshes: number[];
  children: OcctNode[];
}

export interface OcctMesh {
  name: string;
  color?: [number, number, number];
  brep_faces: { first: number; last: number; color: [number, number, number] | null }[];
  attributes: { position: { array: ArrayLike<number> } };
  index: { array: ArrayLike<number> };
}

export interface OcctResult {
  success: boolean;
  root: OcctNode;
  meshes: OcctMesh[];
}

export interface CadPart {
  name: string;
  /** Nicht indizierte Dreiecke in mm. */
  positions: Float32Array;
  color?: string;
  /** Körper derselben Baugruppe (Knoten der Datei) haben dieselbe Gruppe. */
  group: string;
}

const hex = (c: ArrayLike<number>) => `#${[0, 1, 2].map((k) => Math.round(Math.max(0, Math.min(1, c[k])) * 255).toString(16).padStart(2, '0')).join('')}`;

/** Allgemeine Körpernamen aus CAD-Programmen – dann lieber den Namen der Baugruppe nehmen. */
const GENERIC = /^(solid|body|körper|shape|part|open\s*cascade.*|)\s*\d*$/i;

/** Ergebnis von occt-import-js → Teile. */
export function occtToParts(r: OcctResult): CadPart[] {
  if (!r.success) throw new Error('Die CAD-Datei ließ sich nicht lesen.');
  const out: CadPart[] = [];
  let groupNo = 0;
  const visit = (node: OcctNode, inherited: string) => {
    const nodeName = node.name || inherited;
    const group = `cad-${groupNo++}`;
    for (const i of node.meshes) {
      const m = r.meshes[i];
      const pos = m.attributes.position.array;
      const idx = m.index.array;
      const positions = new Float32Array(idx.length * 3);
      for (let k = 0; k < idx.length; k++) {
        const v = idx[k] * 3;
        positions[k * 3] = pos[v];
        positions[k * 3 + 1] = pos[v + 1];
        positions[k * 3 + 2] = pos[v + 2];
      }
      if (!positions.length) continue;
      // Farbe des Körpers, sonst die häufigste Flächenfarbe
      const faceColor = m.brep_faces.find((f) => f.color)?.color;
      const color = m.color ?? faceColor ?? undefined;
      out.push({ name: (m.name && !GENERIC.test(m.name.trim()) ? m.name : nodeName) || 'Körper', positions, color: color ? hex(color) : undefined, group });
    }
    for (const c of node.children) visit(c, nodeName);
  };
  visit(r.root, '');
  return out;
}
