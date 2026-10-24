/**
 * Rechenaufgaben, die im Hintergrund laufen (compute/worker.ts) – oder, falls der Browser keinen
 * Worker erlaubt, genauso im Hauptthread (compute/client.ts). Ein- und Ausgaben sind reine Daten.
 */
import { type ColoredPart, type ImportedPart, type BambuOptions, read3MF, write3MF, write3MFBambu } from '../io/threemf';
import { type Blobs, type ProjectData, packProject } from '../io/project';
import { type Feature, buildTopology } from '../math/features';
import { findCandidateFaces } from '../math/segment';
import { type BaseChoice, bestBase } from '../math/orient';

export interface TaskResult<R> {
  result: R;
  /** Puffer, die der Worker abgibt statt sie zu kopieren. */
  transfer?: Transferable[];
}

const buffers = (arrays: (ArrayBufferView | undefined)[]) => [...new Set(arrays.filter((a): a is ArrayBufferView => !!a).map((a) => a.buffer as ArrayBuffer))];

export const TASKS = {
  /** 3MF lesen. */
  read3mf(a: { data: Uint8Array }): TaskResult<{ parts: ImportedPart[]; printer?: string }> {
    const info: { printer?: string } = {};
    const parts = read3MF(a.data, info);
    return { result: { parts, printer: info.printer }, transfer: buffers(parts.flatMap((p) => [p.positions, ...(p.modifiers ?? []).map((m) => m.positions)])) };
  },
  /** Markante Flächen eines Teils für automatische Vorschläge. */
  candidates(a: { positions: Float32Array }): TaskResult<Feature[]> {
    return { result: findCandidateFaces(buildTopology(a.positions)) };
  },
  write3mfBambu(a: { parts: ColoredPart[]; opts: BambuOptions }): TaskResult<Uint8Array> {
    const r = write3MFBambu(a.parts, a.opts);
    return { result: r, transfer: buffers([r]) };
  },
  write3mf(a: { parts: ColoredPart[]; mode: 'single' | 'multicolor' }): TaskResult<Uint8Array> {
    const r = write3MF(a.parts, a.mode);
    return { result: r, transfer: buffers([r]) };
  },
  /** Größte ebene Auflage (Dreiecke in Weltkoordinaten). */
  bestBase(a: { positions: Float32Array }): TaskResult<BaseChoice> {
    return { result: bestBase(a.positions) };
  },
  packProject(a: { data: ProjectData; blobs: Blobs }): TaskResult<Uint8Array> {
    const r = packProject(a.data, a.blobs);
    return { result: r, transfer: buffers([r]) };
  },
};

export type Tasks = typeof TASKS;
export type TaskName = keyof Tasks;
export type TaskArgs<K extends TaskName> = Parameters<Tasks[K]>[0];
export type TaskOutput<K extends TaskName> = ReturnType<Tasks[K]>['result'];
