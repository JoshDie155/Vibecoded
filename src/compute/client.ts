/**
 * Rechnet im Hintergrund-Thread (Web Worker), damit die Oberfläche bedienbar bleibt. Lässt der
 * Browser keinen Worker zu (z. B. Sicherheitseinstellungen bei lokal geöffneten Dateien), läuft
 * dieselbe Aufgabe im Hauptthread – langsamer zu bedienen, aber mit gleichem Ergebnis.
 */
import ComputeWorker from './worker?worker&inline';
import { TASKS, type TaskArgs, type TaskName, type TaskOutput } from './tasks';

interface Pending {
  task: TaskName;
  args: unknown;
  resolve: (v: unknown) => void;
  reject: (e: Error) => void;
}

let worker: Worker | null | undefined;
let nextId = 1;
const pending = new Map<number, Pending>();

/** Im Hauptthread rechnen – einen Moment später, damit Meldungen vorher noch erscheinen. */
function local<K extends TaskName>(task: K, args: TaskArgs<K>): Promise<TaskOutput<K>> {
  return new Promise((resolve, reject) => setTimeout(() => {
    try {
      resolve((TASKS[task] as (a: TaskArgs<K>) => { result: TaskOutput<K> })(args).result);
    } catch (e) {
      reject(e as Error);
    }
  }, 30));
}

/** Worker unbrauchbar: offene Aufgaben im Hauptthread nachholen. */
function giveUp() {
  worker?.terminate();
  worker = null;
  const open = [...pending.values()];
  pending.clear();
  for (const p of open) local(p.task, p.args as never).then(p.resolve, p.reject);
}

function getWorker(): Worker | null {
  if (worker !== undefined) return worker;
  try {
    worker = new ComputeWorker();
    worker.onmessage = (e: MessageEvent<{ id: number; result?: unknown; error?: string }>) => {
      const p = pending.get(e.data.id);
      if (!p) return;
      pending.delete(e.data.id);
      if (e.data.error !== undefined) p.reject(new Error(e.data.error));
      else p.resolve(e.data.result);
    };
    worker.onerror = (e) => {
      e.preventDefault();
      giveUp();
    };
  } catch {
    worker = null;
  }
  return worker;
}

/** Aufgabe im Hintergrund rechnen. Eingaben werden kopiert (der Aufrufer behält seine Daten). */
export function compute<K extends TaskName>(task: K, args: TaskArgs<K>): Promise<TaskOutput<K>> {
  const w = getWorker();
  if (!w) return local(task, args);
  return new Promise((resolve, reject) => {
    const id = nextId++;
    pending.set(id, { task, args, resolve: resolve as (v: unknown) => void, reject });
    try {
      w.postMessage({ id, task, args });
    } catch {
      // Nicht übertragbare Daten o. Ä.: im Hauptthread rechnen
      pending.delete(id);
      local(task, args).then(resolve, reject);
    }
  });
}

/** Gibt es einen Hintergrund-Thread (wird bei Bedarf gestartet)? Sonst liefe alles im Hauptthread. */
export const hasWorker = () => !!getWorker();
