/**
 * Zu große Teile auf Druckplattengröße zerteilen: Wie viele Stücke je Achse, damit jedes Stück (mit Rand
 * für Verbinder) auf die Platte passt – auch um 90° gedreht? Gewählt wird die kleinste Stückzahl.
 */
export type Vec3 = [number, number, number];

export interface SplitPlan {
  /** Stücke entlang X, Y, Z. */
  counts: Vec3;
  /** Größe eines Stücks (mm). */
  piece: Vec3;
  /** Stücke müssen auf der Platte um 90° gedreht liegen (X entlang der Plattentiefe). */
  turned: boolean;
}

/**
 * @param size Ausdehnung des Teils (mm)
 * @param bed Breite und Tiefe der Platte (mm)
 * @param height Bauhöhe (mm), unbekannt = beliebig hoch
 * @param margin Rand je Stück, z. B. für herausstehende Zapfen (mm)
 */
export function planSplit(size: Vec3, bed: [number, number], height: number | undefined, margin: number): SplitPlan {
  const need = (len: number, room: number) => Math.max(1, Math.ceil(len / Math.max(1, room - margin) - 1e-9));
  const nz = height ? need(size[2], height) : 1;
  const plans = [false, true].map((turned) => {
    const [w, d] = turned ? [bed[1], bed[0]] : bed;
    const counts: Vec3 = [need(size[0], w), need(size[1], d), nz];
    return { counts, turned, piece: size.map((s, k) => s / counts[k]) as Vec3 };
  });
  const total = (p: SplitPlan) => p.counts[0] * p.counts[1] * p.counts[2];
  return total(plans[1]) < total(plans[0]) ? plans[1] : plans[0];
}

/** Lage der Schnittebenen entlang einer Achse: gleich große Stücke zwischen min und max. */
export function splitPlanes(min: number, max: number, count: number): number[] {
  return Array.from({ length: Math.max(0, count - 1) }, (_, k) => min + ((k + 1) * (max - min)) / count);
}
