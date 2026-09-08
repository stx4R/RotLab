/**
 * src/verify.js — 직접 구현 vs Three.js 내장 대조
 *
 * 이 파일이 프로젝트에서 THREE 를 import 하는 유일한 곳이다.
 * 씬·렌더러·메시는 만들지 않는다. Matrix4 / Euler / Quaternion / Vector3 를
 * 순수 수학 객체로만 쓴다.
 */

import * as THREE from 'three';
import * as euler from './euler.js';
import * as quat from './quat.js';

/* ────────────────────────── 난수 (재현 가능) ────────────────────────── */

/** mulberry32 — 씨앗을 고정해 실패를 재현할 수 있게 한다 */
function makeRng(seed) {
  let a = seed >>> 0;
  return function rng() {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const uniform = (rng, lo, hi) => lo + (hi - lo) * rng();
const randAngle = (rng) => uniform(rng, -Math.PI, Math.PI);
const randVec3 = (rng) => [uniform(rng, -2, 2), uniform(rng, -2, 2), uniform(rng, -2, 2)];

/** S³ 위 균등 단위 쿼터니언 (4차원 공 안에서 기각 표집 후 정규화) */
function randUnitQuat(rng) {
  for (;;) {
    const w = uniform(rng, -1, 1);
    const x = uniform(rng, -1, 1);
    const y = uniform(rng, -1, 1);
    const z = uniform(rng, -1, 1);
    const n2 = w * w + x * x + y * y + z * z;
    if (n2 > 1e-6 && n2 <= 1) {
      const n = Math.sqrt(n2);
      return { w: w / n, x: x / n, y: y / n, z: z / n };
    }
  }
}

/* ──────────────────────────── 비교 도우미 ──────────────────────────── */

const maxAbs = (arr) => arr.reduce((m, v) => Math.max(m, Math.abs(v)), 0);

/** 두 3×3 행렬의 최대 절대 성분 오차 */
function matError(A, B) {
  let e = 0;
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) e = Math.max(e, Math.abs(A[i][j] - B[i][j]));
  }
  return e;
}

/** 우리 쿼터니언 vs THREE.Quaternion 의 최대 절대 성분 오차 (부호 그대로) */
function quatError(q, t) {
  return maxAbs([q.w - t.w, q.x - t.x, q.y - t.y, q.z - t.z]);
}

/** 부호를 무시한 쿼터니언 오차 — q 와 −q 는 같은 회전이다 (이중 덮개) */
function quatErrorUpToSign(a, b) {
  const plus = maxAbs([a.w - b.w, a.x - b.x, a.y - b.y, a.z - b.z]);
  const minus = maxAbs([a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z]);
  return Math.min(plus, minus);
}

const vecError = (a, b) => maxAbs([a[0] - b[0], a[1] - b[1], a[2] - b[2]]);

/** THREE 변환 도우미 (THREE.Quaternion 의 인자 순서는 x, y, z, w) */
const toThreeQuat = (q) => new THREE.Quaternion(q.x, q.y, q.z, q.w);
const fromThreeQuat = (t) => ({ w: t.w, x: t.x, y: t.y, z: t.z });

/** Matrix4 의 좌상단 3×3 을 행 우선으로 뽑는다 (elements 는 열 우선) */
function mat3FromMatrix4(m) {
  const e = m.elements;
  return [
    [e[0], e[4], e[8]],
    [e[1], e[5], e[9]],
    [e[2], e[6], e[10]],
  ];
}

/** 결과 레코드 */
function record(id, name, reference, tolerance, samples, maxError, extra = {}) {
  return {
    id,
    name,
    reference,
    tolerance,
    samples,
    maxError,
    passed: Number.isFinite(maxError) && maxError <= tolerance,
    ...extra,
  };
}

/* ───────────────────────────── 검사 1~6 ───────────────────────────── */

/** 1. eulerToMatrix vs THREE.Matrix4().makeRotationFromEuler(Euler(φ,θ,ψ,'ZYX')) */
export function checkEulerToMatrix(rng, { samples = 200, tolerance = 1e-12 } = {}) {
  let maxError = 0;
  let worst = null;
  for (let i = 0; i < samples; i++) {
    const phi = randAngle(rng);
    const theta = randAngle(rng);
    const psi = randAngle(rng);

    const mine = euler.eulerToMatrix(phi, theta, psi);
    const ref = mat3FromMatrix4(
      new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(phi, theta, psi, 'ZYX'))
    );

    const e = matError(mine, ref);
    if (e > maxError) {
      maxError = e;
      worst = { phi, theta, psi };
    }
  }
  return record(1, 'eulerToMatrix (ZYX)', 'Matrix4.makeRotationFromEuler', tolerance, samples, maxError, { worst });
}

/** 2. quat.multiply vs THREE.Quaternion.multiply */
export function checkQuatMultiply(rng, { samples = 200, tolerance = 1e-12 } = {}) {
  let maxError = 0;
  let worst = null;
  for (let i = 0; i < samples; i++) {
    const a = randUnitQuat(rng);
    const b = randUnitQuat(rng);

    const mine = quat.multiply(a, b);
    const ref = toThreeQuat(a).multiply(toThreeQuat(b));

    const e = quatError(mine, ref);
    if (e > maxError) {
      maxError = e;
      worst = { a, b };
    }
  }
  return record(2, 'quat.multiply', 'Quaternion.multiply', tolerance, samples, maxError, { worst });
}

/**
 * 3. toMatrix / fromMatrix 왕복.
 *
 * 두 방향을 모두 본다.
 *   q → R → q' : 회전행렬은 쿼터니언을 부호까지 결정하지 못하므로(이중 덮개)
 *                ± 를 무시하고 비교한다.
 *   R → q → R' : 부호 모호성이 없는 방향. 성분을 그대로 비교한다.
 */
export function checkMatrixRoundTrip(rng, { samples = 200, tolerance = 1e-12 } = {}) {
  let maxError = 0;
  let worst = null;
  for (let i = 0; i < samples; i++) {
    const q = randUnitQuat(rng);

    const R = quat.toMatrix(q);
    const back = quat.fromMatrix(R);
    const eQ = quatErrorUpToSign(q, back);

    const R2 = quat.toMatrix(back);
    const eR = matError(R, R2);

    const e = Math.max(eQ, eR);
    if (e > maxError) {
      maxError = e;
      worst = { q, eQ, eR };
    }
  }
  return record(3, 'toMatrix / fromMatrix 왕복', '자기 무모순 (± 부호 무시)', tolerance, samples, maxError, { worst });
}

/** 4. quat.rotateVector vs THREE Vector3.applyQuaternion */
export function checkRotateVector(rng, { samples = 200, tolerance = 1e-12 } = {}) {
  let maxError = 0;
  let worst = null;
  for (let i = 0; i < samples; i++) {
    const q = randUnitQuat(rng);
    const v = randVec3(rng);

    const mine = quat.rotateVector(q, v);
    const r = new THREE.Vector3(v[0], v[1], v[2]).applyQuaternion(toThreeQuat(q));

    const e = vecError(mine, [r.x, r.y, r.z]);
    if (e > maxError) {
      maxError = e;
      worst = { q, v };
    }
  }
  return record(4, 'quat.rotateVector (q v q*)', 'Vector3.applyQuaternion', tolerance, samples, maxError, { worst });
}

/** 5. slerp vs THREE.Quaternion.slerp — t = 0, 0.1, …, 1.0 */
export function checkSlerp(rng, { pairs = 100, tolerance = 1e-9 } = {}) {
  const ts = [];
  for (let k = 0; k <= 10; k++) ts.push(k / 10);

  let maxError = 0;
  let worst = null;
  let negativeDotPairs = 0;
  let maxErrorNegDot = 0;
  let maxErrorPosDot = 0;

  for (let i = 0; i < pairs; i++) {
    const q0 = randUnitQuat(rng);
    const q1 = randUnitQuat(rng);
    const d = quat.dot(q0, q1);
    if (d < 0) negativeDotPairs++;

    for (const t of ts) {
      const mine = quat.slerp(q0, q1, t);
      const ref = toThreeQuat(q0).slerp(toThreeQuat(q1), t);

      const e = quatError(mine, ref);
      if (d < 0) maxErrorNegDot = Math.max(maxErrorNegDot, e);
      else maxErrorPosDot = Math.max(maxErrorPosDot, e);

      if (e > maxError) {
        maxError = e;
        worst = { q0, q1, t, dot: d, mine, ref: fromThreeQuat(ref) };
      }
    }
  }

  const rec = record(5, 'quat.slerp', 'Quaternion.slerp', tolerance, pairs * ts.length, maxError, {
    worst,
    pairs,
    negativeDotPairs,
    maxErrorNegDot,
    maxErrorPosDot,
  });

  // dot<0 표본이 하나도 없으면 대척점 경우를 검사하지 못한 것이므로 실패로 본다.
  if (negativeDotPairs === 0) {
    rec.passed = false;
    rec.note = 'dot(q0,q1) < 0 인 표본이 없어 대척점 경우를 검사하지 못했다';
  }
  return rec;
}

/** 6. gimbalMeasure vs cos θ, 그리고 θ = ±90° 특이점 */
export function checkGimbalMeasure(
  rng,
  { samples = 200, tolerance = 1e-14, singularTolerance = 1e-15 } = {}
) {
  let maxError = 0;
  let worst = null;
  for (let i = 0; i < samples; i++) {
    const phi = randAngle(rng);
    const theta = randAngle(rng);
    const psi = randAngle(rng);

    const e = Math.abs(euler.gimbalMeasure(phi, theta, psi) - Math.cos(theta));
    if (e > maxError) {
      maxError = e;
      worst = { phi, theta, psi };
    }
  }

  // θ = ±90° 에서 E 는 특이행렬이어야 한다.
  let maxSingularDet = 0;
  let singularSamples = 0;
  for (const theta of [Math.PI / 2, -Math.PI / 2]) {
    for (let k = 0; k < 32; k++) {
      const phi = (2 * Math.PI * k) / 32 - Math.PI;
      const psi = (2 * Math.PI * ((k * 7) % 32)) / 32 - Math.PI;
      maxSingularDet = Math.max(maxSingularDet, Math.abs(euler.gimbalMeasure(phi, theta, psi)));
      singularSamples++;
    }
  }

  const rec = record(6, 'gimbalMeasure = det E', 'cos θ (해석해)', tolerance, samples, maxError, {
    worst,
    maxSingularDet,
    singularSamples,
    singularTolerance,
  });
  rec.passed = rec.passed && maxSingularDet < singularTolerance;
  return rec;
}

/* ─────────────────── 추가 점검 (6개 판정에는 포함되지 않음) ─────────────────── */

/** matrixToEuler 왕복 — 정칙 구간과 짐벌 락 구간을 함께 본다 */
export function checkMatrixToEuler(rng, { samples = 200, tolerance = 1e-12 } = {}) {
  let maxError = 0;
  for (let i = 0; i < samples; i++) {
    // |θ| ≤ 80° 로 제한해 정칙 구간에서만 각 자체를 비교한다
    const phi = randAngle(rng);
    const theta = uniform(rng, -1.396, 1.396);
    const psi = randAngle(rng);

    const R = euler.eulerToMatrix(phi, theta, psi);
    const back = euler.matrixToEuler(R);
    maxError = Math.max(
      maxError,
      Math.abs(back.phi - phi),
      Math.abs(back.theta - theta),
      Math.abs(back.psi - psi)
    );
    if (back.degenerate) maxError = Infinity;
  }

  // 짐벌 락: 각 자체는 복원되지 않아도 행렬은 복원되어야 하고 degenerate 로 표시돼야 한다.
  let lockMatrixError = 0;
  let lockFlagged = 0;
  const lockSamples = 32;
  for (let k = 0; k < lockSamples; k++) {
    const phi = randAngle(rng);
    const psi = randAngle(rng);
    const theta = (k % 2 === 0 ? 1 : -1) * (Math.PI / 2);
    const R = euler.eulerToMatrix(phi, theta, psi);
    const back = euler.matrixToEuler(R);
    if (back.degenerate) lockFlagged++;
    lockMatrixError = Math.max(
      lockMatrixError,
      matError(R, euler.eulerToMatrix(back.phi, back.theta, back.psi))
    );
  }

  const rec = record(0, 'matrixToEuler 왕복', '자기 무모순', tolerance, samples, maxError, {
    lockMatrixError,
    lockFlagged,
    lockSamples,
  });
  rec.passed = rec.passed && lockFlagged === lockSamples && lockMatrixError <= 1e-12;
  return rec;
}

/* ─────────────────────────── 실행 & 보고 ─────────────────────────── */

export const DEFAULT_SEED = 0x5eed1234;

/**
 * 6개 항목을 모두 돌린다.
 * @param {{seed?:number, extras?:boolean}} options
 */
export function runAll({ seed = DEFAULT_SEED, extras = true } = {}) {
  const rng = makeRng(seed);
  const results = [
    checkEulerToMatrix(rng),
    checkQuatMultiply(rng),
    checkMatrixRoundTrip(rng),
    checkRotateVector(rng),
    checkSlerp(rng),
    checkGimbalMeasure(rng),
  ];
  const extraResults = extras ? [checkMatrixToEuler(rng)] : [];
  return {
    seed,
    threeVersion: THREE.REVISION,
    results,
    extraResults,
    allPassed: results.every((r) => r.passed),
  };
}

const fmt = (v) => (Number.isFinite(v) ? v.toExponential(3) : String(v));
const pad = (s, n) => String(s) + ' '.repeat(Math.max(0, n - String(s).length));

/** 콘솔에 보고서를 찍는다. 6개 항목 전부 통과하면 true. */
export function printReport(report = runAll()) {
  const { seed, threeVersion, results, extraResults, allPassed } = report;

  const bar = '='.repeat(78);
  console.log(bar);
  console.log(' RotLab 수학 코어 검증   three.js r' + threeVersion + '   seed 0x' + seed.toString(16));
  console.log(bar);
  console.log(
    ' ' + pad('#', 4) + pad('항목', 30) + pad('표본', 8) +
    pad('max |오차|', 12) + pad('허용치', 12) + '결과'
  );
  console.log('-'.repeat(78));

  const line = (r) =>
    ' ' + pad(r.id || '·', 4) + pad(r.name, 30) + pad(r.samples, 8) +
    pad(fmt(r.maxError), 12) + pad(fmt(r.tolerance), 12) +
    (r.passed ? 'PASS' : 'FAIL');

  for (const r of results) console.log(line(r));
  if (extraResults && extraResults.length) {
    console.log('-'.repeat(78) + '\n 추가 점검 (6개 판정에는 포함되지 않음)');
    for (const r of extraResults) console.log(line(r));
  }
  console.log(bar);

  const slerp = results.find((r) => r.id === 5);
  if (slerp) {
    console.log(
      ' 5) dot<0 쌍 ' + slerp.negativeDotPairs + '/' + slerp.pairs +
      '   max|오차| dot<0: ' + fmt(slerp.maxErrorNegDot) +
      '   dot>=0: ' + fmt(slerp.maxErrorPosDot)
    );
  }
  const gimbal = results.find((r) => r.id === 6);
  if (gimbal) {
    console.log(
      ' 6) theta=+-90 에서 max|det E| = ' + fmt(gimbal.maxSingularDet) +
      '  (< ' + fmt(gimbal.singularTolerance) + ' 필요, ' + gimbal.singularSamples + '표본)  ' +
      (gimbal.maxSingularDet < gimbal.singularTolerance ? 'OK' : 'NG')
    );
  }

  for (const r of results) {
    if (!r.passed) {
      console.warn('\n[FAIL] ' + r.id + '. ' + r.name);
      if (r.note) console.warn('  ' + r.note);
      console.warn('  최악 표본:', r.worst);
    }
  }

  console.log(
    '\n' +
    (allPassed
      ? '[OK] 6개 항목 전부 통과'
      : '[NG] 실패: ' + results.filter((r) => !r.passed).map((r) => '#' + r.id).join(', '))
  );
  return allPassed;
}
