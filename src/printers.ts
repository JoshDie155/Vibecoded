/**
 * Drucker verschiedener Hersteller: Bauraum (mm), bei Druckern mit eingeschränkter Reichweite einzelner
 * Düsen deren Bereiche, und für Bambu Lab die Namen der System-Profile (für das Projekt im Export).
 *
 * Maße nach Herstellerangaben; wo Quellen sich widersprechen, gilt der kleinere Wert (lieber zu vorsichtig
 * als ein Teil, das dann doch nicht passt).
 */
import { BAMBU_PRINTERS, type BambuPrinter } from './io/threemf';

/** Slicer bzw. Slicer-Familie mit eigenem Projektformat. */
export type SlicerId = 'bambu' | 'orca' | 'prusa' | 'cura';

export interface Printer {
  /** Feste Kennung (bleibt gespeichert); Bambu-Drucker behalten ihre bisherigen Kennungen. */
  id: string;
  vendor: string;
  /** Modellname ohne Hersteller. */
  label: string;
  /** Druckbereich in mm. */
  bed: [number, number];
  /** Größte Druckhöhe in mm, falls bekannt. */
  height?: number;
  /** Zwei Düsen mit eingeschränkter Reichweite: Bereich in X (mm ab links), den die linke bzw. rechte erreicht. */
  reach?: [[number, number], [number, number]];
  /** Mehrere Düsen bzw. Werkzeuge (nur zur Anzeige). */
  tools?: number;
  /** Bambu Lab: Namen der System-Profile für das Projekt. */
  bambu?: BambuPrinter;
  /** Slicer des Herstellers – Vorauswahl beim Export. */
  slicer: SlicerId;
}

const bambu: Printer[] = BAMBU_PRINTERS.map((b) => ({
  id: b.id, vendor: 'Bambu Lab', label: b.label, bed: b.bed, height: b.height, reach: b.reach,
  tools: b.extruders?.length, bambu: b, slicer: 'bambu',
}));

type Row = [id: string, label: string, w: number, d: number, h: number | undefined, extra?: Partial<Printer>];
const vendor = (name: string, slicer: SlicerId, rows: Row[]): Printer[] =>
  rows.map(([id, label, w, d, h, extra]) => ({ id, vendor: name, label, bed: [w, d], height: h, slicer, ...extra }));

export const PRINTERS: Printer[] = [
  ...bambu,
  ...vendor('Prusa', 'prusa', [
    ['PRUSA-CORE1', 'CORE One', 250, 220, 270],
    ['PRUSA-CORE1L', 'CORE One L', 300, 300, 330],
    ['PRUSA-MK4S', 'MK4S', 250, 210, 220],
    ['PRUSA-MK4', 'MK4', 250, 210, 220],
    ['PRUSA-MK3S', 'MK3S+', 250, 210, 210],
    ['PRUSA-MINI', 'MINI+', 180, 180, 180],
    ['PRUSA-XL', 'XL', 360, 360, 360, { tools: 5 }],
  ]),
  ...vendor('Creality', 'orca', [
    ['CR-K2PLUS', 'K2 Plus', 350, 350, 350],
    ['CR-K2PRO', 'K2 Pro', 300, 300, 300],
    ['CR-K2', 'K2', 260, 260, 260],
    ['CR-K1MAX', 'K1 Max', 300, 300, 300],
    ['CR-K1C', 'K1C', 220, 220, 250],
    ['CR-K1', 'K1', 220, 220, 250],
    ['CR-E3V3', 'Ender-3 V3', 220, 220, 250],
    ['CR-E3V3KE', 'Ender-3 V3 KE', 220, 220, 240],
    ['CR-E3V3SE', 'Ender-3 V3 SE', 220, 220, 250],
    ['CR-E3V3PLUS', 'Ender-3 V3 Plus', 300, 300, 330],
  ]),
  ...vendor('Anycubic', 'orca', [
    ['AC-KS1', 'Kobra S1', 250, 250, 250],
    ['AC-K3', 'Kobra 3', 250, 250, 260],
    ['AC-K3MAX', 'Kobra 3 Max', 420, 420, 500],
    ['AC-K2NEO', 'Kobra 2 Neo', 220, 220, 250],
  ]),
  ...vendor('Elegoo', 'orca', [
    ['EL-CC', 'Centauri Carbon', 256, 256, 256],
    ['EL-N4', 'Neptune 4', 225, 225, 265],
    ['EL-N4PRO', 'Neptune 4 Pro', 225, 225, 265],
    ['EL-N4PLUS', 'Neptune 4 Plus', 320, 320, 385],
    ['EL-N4MAX', 'Neptune 4 Max', 420, 420, 480],
  ]),
  ...vendor('QIDI', 'orca', [
    ['QD-Q2', 'Q2', 270, 270, 256],
    ['QD-Q1PRO', 'Q1 Pro', 245, 245, 240],
    ['QD-PLUS5', 'Plus5', 320, 320, 300],
    ['QD-PLUS4', 'Plus4', 305, 305, 280],
    ['QD-XMAX3', 'X-Max 3', 325, 325, 315],
  ]),
  ...vendor('Sovol', 'orca', [
    ['SV-SV08', 'SV08', 350, 350, 345],
    ['SV-SV06ACE', 'SV06 ACE', 220, 220, 250],
  ]),
  ...vendor('Voron', 'orca', [
    ['VO-24-250', '2.4 (250)', 250, 250, 220],
    ['VO-24-300', '2.4 (300)', 300, 300, 280],
    ['VO-24-350', '2.4 (350)', 350, 350, 330],
    ['VO-TRI-250', 'Trident (250)', 250, 250, 250],
    ['VO-TRI-300', 'Trident (300)', 300, 300, 250],
    ['VO-TRI-350', 'Trident (350)', 350, 350, 250],
  ]),
  ...vendor('Snapmaker', 'orca', [
    ['SM-U1', 'U1', 270, 270, 270, { tools: 4 }],
    ['SM-J1', 'J1 / J1s', 300, 200, 200, { tools: 2 }],
  ]),
  ...vendor('Flashforge', 'orca', [
    ['FF-AD5M', 'Adventurer 5M / 5M Pro', 220, 220, 220],
    ['FF-AD5X', 'AD5X', 220, 220, 220],
  ]),
  ...vendor('Artillery', 'orca', [
    ['AR-X4PLUS', 'Sidewinder X4 Plus', 300, 300, 400],
    ['AR-X4PRO', 'Sidewinder X4 Pro', 240, 240, 260],
  ]),
  ...vendor('UltiMaker', 'cura', [
    ['UM-S7', 'S7', 330, 240, 300, { tools: 2 }],
    ['UM-S5', 'S5', 330, 240, 300, { tools: 2 }],
    ['UM-S3', 'S3', 230, 190, 200, { tools: 2 }],
  ]),
];

/** Hersteller in der Reihenfolge der Liste. */
export const VENDORS = [...new Set(PRINTERS.map((p) => p.vendor))];

export const printerById = (id: string): Printer | undefined => PRINTERS.find((p) => p.id === id);

/** Voller Name, z. B. „Prusa CORE One“. */
export const printerName = (p: Printer) => `${p.vendor} ${p.label}`;
