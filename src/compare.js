/**
 * src/compare.js — 오일러 선형보간 vs slerp 비교
 *
 * 회전 계산은 전부 1단계 모듈이 한다. THREE 는 좌우 두 오브젝트와
 * 궤적 선을 그리는 데만 쓴다.
 *
 * 측정에 관한 세 가지 결정:
 *
 * 1) 프레임 간 회전각은 쿼터니언으로 잰다.
 *      q_rel = q_t ⊗ conj(q_{t−1}),  α = 2·atan2(‖(x,y,z)‖, |w|)
 *    트레이스 공식 α = arccos((tr−1)/2) 는 α→0 에서 조건수가 무너진다.
 *    d/dx arccos(x) = −1/√(1−x²) 라 x→1 에서 발산하고, tr−1)/2 는
 *    1 − α²/4 이라 α 가 작을수록 정확히 그 구역에 들어간다.
 *    프레임 간 회전각은 늘 1° 미만이므로 트레이스 공식으로는 잴 수 없다.
 *    (1단계에서 matrixToEuler 를 asin 대신 atan2 로 간 것과 같은 이유다.)
 *    |w| 를 쓰므로 q 와 −q 의 이중 덮개도 자동으로 처리된다.
 *
 * 2) 오일러 선형보간에는 각도 래핑을 건다. 성분 차이를 [−π, π] 로 감싼 뒤
 *    최단 방향으로 간다. 래핑을 안 하면 170°→−170° 에서 340° 를 돌아가는데
 *    그건 오일러 보간의 성질이 아니라 구현 실수라 비교가 조작이 된다.
 *
 * 3) 통계는 균일 t 격자에서 낸 값을 1차 지표로 삼는다. 렌더 프레임 간격은
 *    브라우저 스케줄러 때문에 흔들리고, 그 지터가 두 방식 모두의 표준편차에
 *    섞여 들어가 정작 재려는 차이를 가린다. 실제 렌더 프레임 통계도 함께
 *    내되 지터가 섞였다는 사실을 명시한다.
 */

import * as THREE from 'three';
import { multiply, transpose } from './mat3.js';
import { eulerToMatrix, matrixToEuler } from './euler.js';
import * as quat from './quat.js';
import { makeTripod, setMatrix } from './scene.js';

/* ───────────────────────────── 상수 ───────────────────────────── */

/**
 * 두 방식의 식별색. TDS 토큰이며 패널의 범례 스와치와 같은 색이다.
 *   euler → red-500   oklch(0.628 0.218 22)
 *   slerp → white
 * 축 색(blue/green/orange-500) 과 겹치지 않는다.
 */
const METHOD_COLOR = {
  euler: 0xf03848, // red-500 — 오일러 선형보간 (왼쪽)
  slerp: 0xffffff, // white   — slerp (오른쪽)
};

/** 좌우 배치 간격 (월드 X). 카메라 방위각을 −90° 로 두면 화면 좌우와 맞는다. */
const OFFSET = 2.45;

/** 궤적을 남길 점 — 삼각대의 앞쪽(+X) 축 끝 */
const TRAIL_TIP = 1.45;

/** 균일 t 격자 스텝 수. 3초 × 60fps 와 눈금을 맞췄다. */
export const GRID_STEPS = 180;

const MAX_TRAIL_POINTS = 4096;
const RAD2DEG = 180 / Math.PI;

/* ─────────────────────────── 보간 수학 ─────────────────────────── */

/** 각도를 [−π, π] 로 감싼다 */
export const wrapPi = (a) => a - 2 * Math.PI * Math.round(a / (2 * Math.PI));

/**
 * 프레임 간 회전각 (쿼터니언 방식).  작은 각도에서도 조건수가 유지된다.
 * @returns {number} rad, [0, π]
 */
export function frameAngle(qPrev, qCur) {
  const rel = quat.multiply(qCur, quat.conjugate(qPrev));
  return 2 * Math.atan2(Math.hypot(rel.x, rel.y, rel.z), Math.abs(rel.w));
}

/** 대조용 — 원래 지시였던 트레이스 공식. 검증에만 쓴다. */
export function frameAngleTrace(Rprev, Rcur) {
  const rel = multiply(Rcur, transpose(Rprev));
  const tr = rel[0][0] + rel[1][1] + rel[2][2];
  const c = (tr - 1) / 2;
  return Math.acos(c < -1 ? -1 : c > 1 ? 1 : c);
}

/**
 * 두 공식이 큰 각도에서 일치하는지, 작은 각도에서 어디서 갈라지는지 확인한다.
 * 축-각으로 정확한 α 를 만들어 참값을 알고 있는 상태에서 잰다.
 * @returns {Array<{alphaDeg:number, quatErr:number, traceErr:number}>}
 */
export function angleFormulaCheck(
  degrees = [179, 120, 90, 45, 10, 1, 0.1, 0.01, 1e-3, 1e-4, 1e-5, 1e-6]
) {
  const axis = [0.3, -0.7, 0.5];
  const q0 = quat.normalize({ w: 0.31, x: -0.52, y: 0.44, z: 0.66 });
  return degrees.map((deg) => {
    const alpha = deg / RAD2DEG;
    const q1 = quat.multiply(quat.fromAxisAngle(axis, alpha), q0);
    const qa = frameAngle(q0, q1);
    const ta = frameAngleTrace(quat.toMatrix(q0), quat.toMatrix(q1));
    return {
      alphaDeg: deg,
      quat: qa * RAD2DEG,
      trace: ta * RAD2DEG,
      quatErr: Math.abs(qa - alpha) * RAD2DEG,
      traceErr: Math.abs(ta - alpha) * RAD2DEG,
      quatRelErr: Math.abs(qa - alpha) / alpha,
      traceRelErr: Math.abs(ta - alpha) / alpha,
    };
  });
}

/**
 * 오일러 선형보간 준비. 자세는 쿼터니언으로 들어오고, 여기서만 분해한다.
 * 분해가 유일하지 않으면(짐벌락) degenerate 로 알린다 — 값을 억지로 믿지 말라는 뜻.
 */
export function eulerPlan(qA, qB) {
  const a = matrixToEuler(quat.toMatrix(qA));
  const b = matrixToEuler(quat.toMatrix(qB));
  return {
    a,
    b,
    degenerateA: a.degenerate,
    degenerateB: b.degenerate,
    // 성분별 최단 방향 차이
    d: {
      phi: wrapPi(b.phi - a.phi),
      theta: wrapPi(b.theta - a.theta),
      psi: wrapPi(b.psi - a.psi),
    },
  };
}

/** 오일러 선형보간의 t 시점 회전행렬 */
export function eulerAt(plan, t) {
  return eulerToMatrix(
    plan.a.phi + t * plan.d.phi,
    plan.a.theta + t * plan.d.theta,
    plan.a.psi + t * plan.d.psi
  );
}

/** 회전각 수열(rad)의 통계. 값은 도 단위로 낸다. */
export function stats(anglesRad) {
  const n = anglesRad.length;
  if (n === 0) return { n: 0, mean: 0, std: 0, max: 0, total: 0 };
  let sum = 0;
  let max = 0;
  for (const a of anglesRad) {
    const d = a * RAD2DEG;
    sum += d;
    if (d > max) max = d;
  }
  const mean = sum / n;
  let v = 0;
  for (const a of anglesRad) v += (a * RAD2DEG - mean) ** 2;
  return { n, mean, std: Math.sqrt(v / n), max, total: sum };
}

/** A 에서 B 까지의 측지선 회전각 (최단 경로 길이) */
export function geodesicAngle(qA, qB) {
  return frameAngle(qA, qB);
}

/**
 * 균일 t 격자에서 두 방식을 잰다. 프레임 지터가 섞이지 않는 1차 지표.
 */
export function measureUniform(qA, qB, steps = GRID_STEPS) {
  const plan = eulerPlan(qA, qB);
  const qe = [];
  const qs = [];
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    qe.push(quat.fromMatrix(eulerAt(plan, t)));
    qs.push(quat.slerp(qA, qB, t));
  }
  const ea = [];
  const sa = [];
  for (let i = 1; i <= steps; i++) {
    ea.push(frameAngle(qe[i - 1], qe[i]));
    sa.push(frameAngle(qs[i - 1], qs[i]));
  }
  return {
    steps,
    plan,
    euler: stats(ea),
    slerp: stats(sa),
    geodesic: geodesicAngle(qA, qB) * RAD2DEG,
  };
}

/* ─────────────────────────── 화면 부품 ─────────────────────────── */

function makeTrail(color) {
  const geometry = new THREE.BufferGeometry();
  const positions = new Float32Array(MAX_TRAIL_POINTS * 3);
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setDrawRange(0, 0);
  const line = new THREE.Line(
    geometry,
    new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.95 })
  );
  line.frustumCulled = false;
  line.matrixAutoUpdate = false;
  return { line, positions, count: 0 };
}

function trailPush(trail, offset, R) {
  if (trail.count >= MAX_TRAIL_POINTS) return;
  const i = trail.count * 3;
  // 삼각대 앞쪽(+X) 축 끝점의 월드 위치
  trail.positions[i] = offset[0] + R[0][0] * TRAIL_TIP;
  trail.positions[i + 1] = offset[1] + R[1][0] * TRAIL_TIP;
  trail.positions[i + 2] = offset[2] + R[2][0] * TRAIL_TIP;
  trail.count++;
  trail.line.geometry.setDrawRange(0, trail.count);
  trail.line.geometry.attributes.position.needsUpdate = true;
  trail.line.geometry.computeBoundingSphere();
}

function trailClear(trail) {
  trail.count = 0;
  trail.line.geometry.setDrawRange(0, 0);
}

/* ─────────────────────────── 프레임 예약 ─────────────────────────── */

/**
 * 다음 프레임을 예약하고 취소 함수를 돌려준다.
 *
 * 기본은 requestAnimationFrame 이다. 다만 창이 합성되지 않는 환경
 * (백그라운드 탭, 일부 임베디드 웹뷰, 헤드리스 렌더러) 에서는 rAF 가
 * 아예 호출되지 않아 재생이 통째로 멈춘다. 100ms 안에 rAF 가 오지 않으면
 * 타이머로 대신 진행시킨다. 통계는 실제 Δt 를 그대로 기록하므로 이
 * 대체 경로로 돌아도 표에 찍히는 숫자는 정직하다.
 */
function scheduleFrame(cb) {
  let fired = false;
  let rafId = 0;
  let timerId = 0;
  const run = (ts) => {
    if (fired) return;
    fired = true;
    cancelAnimationFrame(rafId);
    clearTimeout(timerId);
    cb(ts === undefined ? performance.now() : ts);
  };
  rafId = requestAnimationFrame(run);
  timerId = setTimeout(run, 100);
  return () => {
    fired = true;
    cancelAnimationFrame(rafId);
    clearTimeout(timerId);
  };
}

/* ─────────────────────────── 비교 뷰 ─────────────────────────── */

/**
 * @param {ReturnType<import('./scene.js').createScene>} sceneApi
 */
export function createCompare(sceneApi, makeObject = makeTripod) {
  let factory = makeObject;
  let objEuler = factory(METHOD_COLOR.euler);
  let objSlerp = factory(METHOD_COLOR.slerp);
  let lastRe = null;
  let lastRs = null;
  const trailEuler = makeTrail(METHOD_COLOR.euler);
  const trailSlerp = makeTrail(METHOD_COLOR.slerp);

  objEuler.visible = objSlerp.visible = false;
  trailEuler.line.visible = trailSlerp.line.visible = false;

  sceneApi.root.add(objEuler, objSlerp, trailEuler.line, trailSlerp.line);

  const offEuler = [-OFFSET, 0, 0];
  const offSlerp = [OFFSET, 0, 0];

  let cancelFrame = null;

  function place(Re, Rs) {
    setMatrix(objEuler, Re, offEuler);
    setMatrix(objSlerp, Rs, offSlerp);
    lastRe = Re;
    lastRs = Rs;
  }

  /**
   * 좌우 오브젝트를 갈아 끼운다 (삼각대 ↔ GLB 기체).
   * 자세·표시 여부·궤적은 그대로 둔다. 보간 수학에는 손대지 않는다.
   */
  function setObjectFactory(fn) {
    const wasVisible = objEuler.visible;
    sceneApi.root.remove(objEuler, objSlerp);
    factory = fn;
    objEuler = factory(METHOD_COLOR.euler);
    objSlerp = factory(METHOD_COLOR.slerp);
    objEuler.visible = objSlerp.visible = wasVisible;
    sceneApi.root.add(objEuler, objSlerp);
    if (lastRe && lastRs) place(lastRe, lastRs);
    sceneApi.render();
  }

  function setVisible(v) {
    objEuler.visible = objSlerp.visible = v;
    trailEuler.line.visible = trailSlerp.line.visible = v;
    sceneApi.setRingsVisible(!v);
    sceneApi.setMainVisible(!v);
    if (v) {
      // 월드 +X 가 화면 오른쪽이 되는 방위각. 좌우 배치가 화면 좌우와 맞는다.
      sceneApi.setOrbit({ az: -Math.PI / 2, el: 0.30, dist: 11.5 });
    }
    sceneApi.render();
  }

  function clearTrails() {
    trailClear(trailEuler);
    trailClear(trailSlerp);
    sceneApi.render();
  }

  function stop() {
    if (cancelFrame) cancelFrame();
    cancelFrame = null;
  }

  /**
   * t 를 0→1 로 duration 동안 진행시키며 두 오브젝트를 그린다.
   * 프레임별 회전각을 기록하고, 끝나면 균일 격자 통계와 함께 넘긴다.
   */
  function play(qA, qB, { duration = 3000, onProgress, onDone, schedule = scheduleFrame } = {}) {
    stop();
    clearTrails();
    setVisible(true);

    const plan = eulerPlan(qA, qB);
    const framesEuler = [];
    const framesSlerp = [];
    const dts = [];
    const start = performance.now();

    // t=0 을 먼저 그려 둔다. 이렇게 해야 첫 구간이 잘리지 않은 온전한 프레임이 된다.
    const R0e = eulerAt(plan, 0);
    const q0s = quat.slerp(qA, qB, 0);
    place(R0e, quat.toMatrix(q0s));
    trailPush(trailEuler, offEuler, R0e);
    trailPush(trailSlerp, offSlerp, quat.toMatrix(q0s));
    sceneApi.render();

    let prevQe = quat.fromMatrix(R0e);
    let prevQs = q0s;
    let prevTime = start;
    // 첫 구간도 부분 프레임이다 — play() 를 부른 순간은 프레임 경계가 아니다.
    // 마지막과 같은 이유로 통계에서 뺀다.
    let firstFrame = true;

    function step(now) {
      const elapsed = now - start;
      // rAF 가 넘겨주는 타임스탬프는 그 프레임의 시작 시각이라 play() 를 부른
      // 순간보다 이를 수 있다. 아래를 막지 않으면 t 가 음수가 되어 보간이
      // A 바깥으로 외삽된다.
      const t = Math.min(1, Math.max(0, elapsed / duration));
      // 마지막 프레임은 t 를 1 로 자르므로 구간이 짧다. 그대로 통계에 넣으면
      // 값 하나가 이상치가 되어 표준편차를 통째로 오염시킨다 — 그리기는 하되 통계에서 뺀다.
      const truncated = elapsed > duration;

      const Re = eulerAt(plan, t);
      const qs = quat.slerp(qA, qB, t);
      const Rs = quat.toMatrix(qs);
      const qe = quat.fromMatrix(Re);

      if (!truncated && !firstFrame) {
        framesEuler.push(frameAngle(prevQe, qe));
        framesSlerp.push(frameAngle(prevQs, qs));
        dts.push(now - prevTime);
      }
      firstFrame = false;
      prevQe = qe;
      prevQs = qs;
      prevTime = now;

      place(Re, Rs);
      trailPush(trailEuler, offEuler, Re);
      trailPush(trailSlerp, offSlerp, Rs);
      sceneApi.render();

      if (onProgress) onProgress(t);

      if (t < 1) {
        cancelFrame = schedule(step);
      } else {
        cancelFrame = null;
        if (onDone) {
          onDone({
            uniform: measureUniform(qA, qB),
            frame: {
              euler: stats(framesEuler),
              slerp: stats(framesSlerp),
              meanDt: dts.length ? dts.reduce((s, v) => s + v, 0) / dts.length : 0,
              dtStd: (() => {
                if (!dts.length) return 0;
                const m = dts.reduce((s, v) => s + v, 0) / dts.length;
                return Math.sqrt(dts.reduce((s, v) => s + (v - m) ** 2, 0) / dts.length);
              })(),
            },
            plan,
            durationMs: performance.now() - start,
          });
        }
      }
    }

    cancelFrame = schedule(step);
  }

  return { setVisible, setObjectFactory, play, stop };
}
