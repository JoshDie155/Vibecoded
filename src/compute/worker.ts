/** Hintergrund-Thread für rechenintensive Aufgaben (siehe tasks.ts). */
import { TASKS, type TaskName } from './tasks';

self.onmessage = async (e: MessageEvent<{ id: number; task: TaskName; args: never }>) => {
  const { id, task, args } = e.data;
  try {
    const { result, transfer } = await (TASKS[task] as (a: never) => { result: unknown; transfer?: Transferable[] })(args);
    (self as unknown as Worker).postMessage({ id, result }, transfer ?? []);
  } catch (err) {
    // Art des Fehlers mitgeben (z. B. CutError: Meldung für den Nutzer statt technischer Fehler)
    (self as unknown as Worker).postMessage({ id, error: (err as Error).message ?? String(err), name: (err as Error).name });
  }
};
