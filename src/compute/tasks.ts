/**
 * Rechenaufgaben, die im Hintergrund laufen (compute/worker.ts) – oder, falls der Browser keinen
 * Worker erlaubt, genauso im Hauptthread (compute/client.ts). Ein- und Ausgaben sind reine Daten.
 */
import { type ColoredPart, type ImportedPart, type BambuOptions, type ReadInfo, read3MF, write3MF, write3MFBambu } from '../io/threemf';
import { type Blobs, type ProjectData, packProject } from '../io/project';
import { type Feature, buildTopology } from '../math/features';
import { findCandidateFaces } from '../math/segment';
import { type BaseChoice, bestBase } from '../math/orient';
import { CAD_MESHING, type CadPart, type OcctResult, occtToParts } from '../io/step';
import occtImport from 'occt-import-js';
import { gunzipSync } from 'fflate';
import { type MeshHealth, type RepairResult, meshHealth, repairMesh } from '../math/repair';
import { type CutOptions, type CutResult, cutMesh, planConnectors, releaseCut } from '../io/cut';
import { type PocketPlace, type PocketResult, type PocketSpec, addPocket } from '../io/pocket';
import { type BoolInput, type BoolOp, type BoolResult, booleanParts } from '../io/boolean';
import { type UnionInput, type UnionResult, unionParts } from '../io/union';
import { type ToleranceOptions, makeToleranceTest } from '../io/testpiece';
import { type SimplifyResult, simplifyCounts, simplifyMesh } from '../io/simplify';

/** OpenCascade einmal je Thread starten. */
let occt: ReturnType<typeof occtImport> | null = null;

export interface TaskResult<R> {
  result: R;
  /** Puffer, die der Worker abgibt statt sie zu kopieren. */
  transfer?: Transferable[];
}

const buffers = (arrays: (ArrayBufferView | undefined)[]) => [...new Set(arrays.filter((a): a is ArrayBufferView => !!a).map((a) => a.buffer as ArrayBuffer))];

export const TASKS = {
  /** 3MF lesen. */
  read3mf(a: { data: Uint8Array }): TaskResult<{ parts: ImportedPart[] } & ReadInfo> {
    const info: ReadInfo = {};
    const parts = read3MF(a.data, info);
    return { result: { parts, ...info }, transfer: buffers(parts.flatMap((p) => [p.positions, ...(p.modifiers ?? []).map((m) => m.positions)])) };
  },
  /** Markante Flächen eines Teils für automatische Vorschläge. */
  candidates(a: { positions: Float32Array }): TaskResult<Feature[]> {
    return { result: findCandidateFaces(buildTopology(a.positions)) };
  },
  write3mfBambu(a: { parts: ColoredPart[]; opts: BambuOptions }): TaskResult<Uint8Array> {
    const r = write3MFBambu(a.parts, a.opts);
    return { result: r, transfer: buffers([r]) };
  },
  write3mf(a: { parts: ColoredPart[]; mode: 'single' | 'multicolor'; thumbnail?: Uint8Array }): TaskResult<Uint8Array> {
    const r = write3MF(a.parts, a.mode, a.thumbnail);
    return { result: r, transfer: buffers([r]) };
  },
  /** Größte ebene Auflage (Dreiecke in Weltkoordinaten). */
  bestBase(a: { positions: Float32Array }): TaskResult<BaseChoice> {
    return { result: bestBase(a.positions) };
  },
  /** CAD-Datei (STEP, IGES, BREP) lesen und triangulieren; wasmGz: OpenCascade gzip-gepackt (nur beim ersten Mal gebraucht). */
  async readCad(a: { data: Uint8Array; format: 'step' | 'iges' | 'brep'; wasmGz: Uint8Array }): Promise<TaskResult<CadPart[]>> {
    occt ??= occtImport({ wasmBinary: gunzipSync(a.wasmGz) });
    const r = (await occt).ReadFile(a.format, a.data, CAD_MESHING) as OcctResult;
    const parts = occtToParts(r);
    return { result: parts, transfer: buffers(parts.map((p) => p.positions)) };
  },
  /** Ist das Netz geschlossen (für Zerschneiden, Vereinigen, Slicer)? */
  meshHealth(a: { positions: Float32Array }): TaskResult<MeshHealth> {
    return { result: meshHealth(a.positions) };
  },
  repairMesh(a: { positions: Float32Array }): TaskResult<RepairResult> {
    const r = repairMesh(a.positions);
    return { result: r, transfer: buffers([r.positions, r.map, r.flippedTris]) };
  },
  // Boolesche Operationen (manifold-3d); wasm: Adresse der WASM-Datei (absolut oder data:-URL)
  /** Lage der Verbinder für die Vorschau; key: Kennung des Netzes, damit der Körper zwischengespeichert bleibt. */
  async planConnectors(a: { positions: Float32Array; opts: CutOptions; key?: string; wasm: string }): Promise<TaskResult<Awaited<ReturnType<typeof planConnectors>>>> {
    const r = await planConnectors(a.positions, a.opts, a.wasm, a.key);
    return { result: r, transfer: buffers([r.shape]) };
  },
  async cutMesh(a: { positions: Float32Array; opts: CutOptions; key?: string; wasm: string }): Promise<TaskResult<CutResult>> {
    const r = await cutMesh(a.positions, a.opts, a.wasm, a.key);
    return { result: r, transfer: buffers([r.a, r.b, r.aSource, r.bSource, ...r.dowels]) };
  },
  /** Zwischengespeicherten Körper freigeben. */
  releaseCut(): TaskResult<null> {
    releaseCut();
    return { result: null };
  },
  async addPocket(a: { positions: Float32Array; place: PocketPlace; spec: PocketSpec; wasm: string }): Promise<TaskResult<PocketResult>> {
    const r = await addPocket(a.positions, a.place, a.spec, a.wasm);
    return { result: r, transfer: buffers([r.positions, r.source]) };
  },
  async booleanParts(a: { target: BoolInput; others: BoolInput[]; op: BoolOp; wasm: string }): Promise<TaskResult<BoolResult>> {
    const r = await booleanParts(a.target, a.others, a.op, a.wasm);
    return { result: r, transfer: buffers([r.positions, r.source]) };
  },
  async unionParts(a: { parts: UnionInput[]; wasm: string }): Promise<TaskResult<UnionResult>> {
    const r = await unionParts(a.parts, a.wasm);
    return { result: r, transfer: buffers([r.positions]) };
  },
  async toleranceTest(a: { opts: ToleranceOptions; wasm: string }): Promise<TaskResult<{ plate: Float32Array; pin: Float32Array }>> {
    const r = await makeToleranceTest(a.opts, a.wasm);
    return { result: r, transfer: buffers([r.plate, r.pin]) };
  },
  /** Netz vereinfachen; groups: Grenzen, die bleiben sollen (Bemalung, gewählte Flächen). */
  async simplifyMesh(a: { positions: Float32Array; tolerance: number; groups?: Uint32Array; wasm: string }): Promise<TaskResult<SimplifyResult>> {
    const r = await simplifyMesh(a.positions, a.tolerance, a.groups, a.wasm);
    return { result: r, transfer: buffers([r.positions, r.source]) };
  },
  async simplifyCounts(a: { positions: Float32Array; tolerances: number[]; groups?: Uint32Array; wasm: string }): Promise<TaskResult<number[]>> {
    return { result: await simplifyCounts(a.positions, a.tolerances, a.groups, a.wasm) };
  },
  packProject(a: { data: ProjectData; blobs: Blobs }): TaskResult<Uint8Array> {
    const r = packProject(a.data, a.blobs);
    return { result: r, transfer: buffers([r]) };
  },
};

export type Tasks = typeof TASKS;
export type TaskName = keyof Tasks;
export type TaskArgs<K extends TaskName> = Parameters<Tasks[K]>[0];
export type TaskOutput<K extends TaskName> = Awaited<ReturnType<Tasks[K]>>['result'];
