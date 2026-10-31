import { describe, expect, it } from 'vitest';
import { PRINTERS, VENDORS, printerById, printerName } from '../src/printers';
import { BAMBU_PRINTERS } from '../src/io/threemf';

describe('Druckerliste', () => {
  it('eindeutige Kennungen, sinnvolle Maße, Bambu-Kennungen unverändert', () => {
    expect(new Set(PRINTERS.map((p) => p.id)).size).toBe(PRINTERS.length);
    for (const p of PRINTERS) {
      expect(p.bed[0]).toBeGreaterThanOrEqual(150);
      expect(p.bed[1]).toBeGreaterThanOrEqual(150);
      if (p.height) expect(p.height).toBeGreaterThanOrEqual(150);
    }
    for (const b of BAMBU_PRINTERS) expect(printerById(b.id)?.bambu).toBe(b);
  });

  it('mehrere Hersteller, jeder mit Slicer-Vorauswahl; nur Bambu-Drucker tragen Bambu-Profile', () => {
    expect(VENDORS.length).toBeGreaterThanOrEqual(10);
    expect(VENDORS).toContain('Prusa');
    expect(VENDORS).toContain('Creality');
    expect(PRINTERS.filter((p) => p.bambu).every((p) => p.vendor === 'Bambu Lab')).toBe(true);
    expect(printerById('PRUSA-MK4S')?.slicer).toBe('prusa');
    expect(printerById('UM-S5')?.slicer).toBe('cura');
    expect(printerName(printerById('CR-K1MAX')!)).toBe('Creality K1 Max');
  });
});
