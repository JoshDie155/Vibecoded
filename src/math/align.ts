/**
 * Starre Ausrichtung (Rotation + Translation) zweier Punktmengen nach Horn (1987),
 * "Closed-form solution of absolute orientation using unit quaternions".
 *
 * Gesucht sind R und t, sodass  R * src[i] + t ≈ dst[i]  (gewichtete kleinste Quadrate).
 * Spiegelungen sind ausgeschlossen, das Ergebnis ist immer eine echte Rotation.
 */

export type Vec3 = [number, number, number];

/** 3x3-Matrix zeilenweise. */
export type Mat3 = [number, number, number, number, number, number, number, number, number];

export interface Rigid {
  R: Mat3;
  t: Vec3;
}

export const IDENTITY: Rigid = { R: [1, 0, 0, 0, 1, 0, 0, 0, 1], t: [0, 0, 0] };

export function apply(T: Rigid, p: Vec3): Vec3 {
  const R = T.R;
  return [
    R[0] * p[0] + R[1] * p[1] + R[2] * p[2] + T.t[0],
    R[3] * p[0] + R[4] * p[1] + R[5] * p[2] + T.t[1],
    R[6] * p[0] + R[7] * p[1] + R[8] * p[2] + T.t[2],
  ];
}

export function rotate(R: Mat3, v: Vec3): Vec3 {
  return [
    R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
    R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
    R[6] * v[0] + R[7] * v[1] + R[8] * v[2],
  ];
}

export const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const scale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s];
export const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a: Vec3, b: Vec3): Vec3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const len = (a: Vec3) => Math.sqrt(dot(a, a));
export const dist = (a: Vec3, b: Vec3) => len(sub(a, b));
export function normalize(a: Vec3): Vec3 {
  const l = len(a);
  return l > 1e-12 ? scale(a, 1 / l) : [0, 0, 0];
}

/** Eigenwerte/-vektoren einer symmetrischen 4x4-Matrix (zyklisches Jacobi-Verfahren). */
export function jacobiEigen4(Ain: number[][]): { values: number[]; vectors: number[][] } {
  const A = Ain.map((r) => r.slice());
  const V = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
  for (let sweep = 0; sweep < 50; sweep++) {
    let off = 0;
    for (let p = 0; p < 4; p++) for (let q = p + 1; q < 4; q++) off += A[p][q] * A[p][q];
    if (off < 1e-22) break;
    for (let p = 0; p < 4; p++) {
      for (let q = p + 1; q < 4; q++) {
        if (Math.abs(A[p][q]) < 1e-300) continue;
        const theta = (A[q][q] - A[p][p]) / (2 * A[p][q]);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < 4; k++) {
          const akp = A[k][p];
          const akq = A[k][q];
          A[k][p] = c * akp - s * akq;
          A[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < 4; k++) {
          const apk = A[p][k];
          const aqk = A[q][k];
          A[p][k] = c * apk - s * aqk;
          A[q][k] = s * apk + c * aqk;
        }
        for (let k = 0; k < 4; k++) {
          const vkp = V[k][p];
          const vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq;
          V[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }
  return {
    values: [A[0][0], A[1][1], A[2][2], A[3][3]],
    // Spalten von V sind die Eigenvektoren
    vectors: [0, 1, 2, 3].map((j) => [V[0][j], V[1][j], V[2][j], V[3][j]]),
  };
}

export function quatToMat3(q: number[]): Mat3 {
  const [w, x, y, z] = q;
  return [
    w * w + x * x - y * y - z * z, 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), w * w - x * x + y * y - z * z, 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z,
  ];
}

/** Rotation um eine Achse (normiert) um einen Winkel in Radiant. */
export function axisAngle(axis: Vec3, angle: number): Mat3 {
  const [x, y, z] = normalize(axis);
  const h = angle / 2;
  const s = Math.sin(h);
  return quatToMat3([Math.cos(h), x * s, y * s, z * s]);
}

export function compose(A: Rigid, B: Rigid): Rigid {
  // (A ∘ B)(p) = A(B(p))
  const a = A.R;
  const b = B.R;
  const R: Mat3 = [
    a[0] * b[0] + a[1] * b[3] + a[2] * b[6], a[0] * b[1] + a[1] * b[4] + a[2] * b[7], a[0] * b[2] + a[1] * b[5] + a[2] * b[8],
    a[3] * b[0] + a[4] * b[3] + a[5] * b[6], a[3] * b[1] + a[4] * b[4] + a[5] * b[7], a[3] * b[2] + a[4] * b[5] + a[5] * b[8],
    a[6] * b[0] + a[7] * b[3] + a[8] * b[6], a[6] * b[1] + a[7] * b[4] + a[8] * b[7], a[6] * b[2] + a[7] * b[5] + a[8] * b[8],
  ];
  return { R, t: apply(A, B.t) };
}

/** Rotation um eine Achse durch einen Punkt als starre Transformation. */
export function rotationAbout(point: Vec3, axis: Vec3, angle: number): Rigid {
  const R = axisAngle(axis, angle);
  return { R, t: sub(point, rotate(R, point)) };
}

/**
 * Gewichtete starre Ausrichtung src → dst.
 * Bei nur einem Punkt ist das Ergebnis eine reine Verschiebung,
 * bei kollinearen Punkten ist die Drehung um die Linie unbestimmt (es wird die kleinste gewählt).
 */
export function solveRigid(src: Vec3[], dst: Vec3[], weights?: number[]): Rigid {
  const n = src.length;
  if (n === 0 || n !== dst.length) return { R: [...IDENTITY.R], t: [0, 0, 0] };
  const w = weights ?? new Array(n).fill(1);
  let W = 0;
  let pc: Vec3 = [0, 0, 0];
  let qc: Vec3 = [0, 0, 0];
  for (let i = 0; i < n; i++) {
    W += w[i];
    pc = add(pc, scale(src[i], w[i]));
    qc = add(qc, scale(dst[i], w[i]));
  }
  pc = scale(pc, 1 / W);
  qc = scale(qc, 1 / W);

  // Kreuzkovarianz S[a][b] = Σ w (p_a)(q_b)
  const S = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ];
  for (let i = 0; i < n; i++) {
    const p = sub(src[i], pc);
    const q = sub(dst[i], qc);
    for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) S[a][b] += w[i] * p[a] * q[b];
  }
  const [[Sxx, Sxy, Sxz], [Syx, Syy, Syz], [Szx, Szy, Szz]] = S;
  const N = [
    [Sxx + Syy + Szz, Syz - Szy, Szx - Sxz, Sxy - Syx],
    [Syz - Szy, Sxx - Syy - Szz, Sxy + Syx, Szx + Sxz],
    [Szx - Sxz, Sxy + Syx, -Sxx + Syy - Szz, Syz + Szy],
    [Sxy - Syx, Szx + Sxz, Syz + Szy, -Sxx - Syy + Szz],
  ];
  const { values, vectors } = jacobiEigen4(N);
  let best = 0;
  for (let i = 1; i < 4; i++) {
    // Bei Gleichstand (entartete Fälle) die Lösung mit der kleinsten Drehung bevorzugen (|w| groß)
    if (values[i] > values[best] + 1e-12 * (Math.abs(values[best]) + 1)) best = i;
    else if (Math.abs(values[i] - values[best]) <= 1e-12 * (Math.abs(values[best]) + 1) &&
      Math.abs(vectors[i][0]) > Math.abs(vectors[best][0])) best = i;
  }
  const q = vectors[best];
  const qn = Math.hypot(q[0], q[1], q[2], q[3]);
  const R = quatToMat3(q.map((v) => v / qn));
  const t = sub(qc, rotate(R, pc));
  return { R, t };
}

export function rmsd(T: Rigid, src: Vec3[], dst: Vec3[]): number {
  if (src.length === 0) return 0;
  let s = 0;
  for (let i = 0; i < src.length; i++) {
    const d = sub(apply(T, src[i]), dst[i]);
    s += dot(d, d);
  }
  return Math.sqrt(s / src.length);
}
