/**
 * src/main.js — UI 와 씬/비교를 잇는다.
 *
 * 회전에 관한 숫자는 전부 1단계 모듈(euler.js / quat.js)이 낸다.
 * 화면에 찍히는 행렬과 씬에 적용되는 행렬은 같은 함수 호출 결과다.
 */

import { eulerToMatrix, matrixToEuler, gimbalMeasure } from './euler.js';
import * as quat from './quat.js';
import { createScene } from './scene.js';
import { createCompare, angleFormulaCheck, measureUniform } from './compare.js';
import { loadAircraft, MODEL_URL } from './model.js';

const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;

/** 짐벌락 근접 경고 문턱 — |det E| = |cos θ| 가 이 값 미만이면 경고 */
const NEAR_LOCK = 0.05;

/** 짐벌락 실증 버튼이 쓰는 기본 변위 */
const PROOF_DELTA = 30;

/** 재생 길이 */
const PLAY_MS = 3000;

/** 완료 검증 3 번용 — A 와 B 를 이만큼만 떨어뜨린다 */
const NEAR_IDENTICAL_DEG = 0.005;

const $ = (id) => document.getElementById(id);

const canvas = $('view');
const scene = createScene(canvas);
const compare = createCompare(scene);

/* ───────────────────────── 숫자 포맷 ───────────────────────── */

/** −0.0000 이 뜨지 않게 아주 작은 값은 0 으로 접는다 */
const fixed4 = (v) => (Math.abs(v) < 5e-5 ? '0.0000' : v.toFixed(4));
const expo = (v) => (Math.abs(v) < 1e-300 ? '0.00e+0' : v.toExponential(2));
const sgn = (n) => (n < 0 ? '−' : '+') + Math.abs(n) + '°';
const pad = (s, n) => String(s) + ' '.repeat(Math.max(0, n - String(s).length));
const padL = (s, n) => ' '.repeat(Math.max(0, n - String(s).length)) + String(s);

/** 두 3×3 행렬의 최대 절대 성분 차이 */
function maxDiff(A, B) {
  let m = 0;
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) m = Math.max(m, Math.abs(A[i][j] - B[i][j]));
  }
  return m;
}

const qStr = (q) =>
  '(' + [q.w, q.x, q.y, q.z].map((v) => (v < 0 ? '' : ' ') + v.toFixed(4)).join(', ') + ')';

/* ───────────────────────── 상태 ───────────────────────── */

const state = {
  mode: 'euler', // 'euler' | 'quat'
  view: 'edit',  // 'edit' | 'compare'

  // 오일러 모드 (도)
  yaw: 0,
  pitch: 0,
  roll: 0,

  // 쿼터니언 모드
  axis: [0, 0, 1],
  angle: 0, // 도

  // 자세 A / B — 항상 쿼터니언으로 저장한다.
  // 오일러각으로 저장하면 짐벌락을 가로지르는 구간에서 분해가 유일하지 않아
  // A, B 자체가 망가진다.
  poseA: null,
  poseB: null,
  aUser: false,
  bUser: false,
};

/** 짐벌락을 가로지르는 기본 A/B */
const PRESET_CROSS = {
  // B 의 pitch 가 95° — θ=90° 극을 실제로 가로지른다.
  // 분해하면 B 는 ψ−95 θ85 φ−170 이라 ψ 와 φ 가 크게 튄다.
  // Δψ 175° · Δφ −175° — ±180° 를 피해 잡았다. 정확히 ±180° 면 래핑 방향이
  // 원리적으로 미정이라 부동소수점 부호로 갈리고 수치가 재현되지 않는다.
  // 오일러 보간은 그 먼 길을 돌고(측지선의 2.16배), slerp 는 최단호로 간다.
  a: { yaw: 90, pitch: -45, roll: 5 },
  b: { yaw: 85, pitch: 95, roll: 10 },
};

const poseFromEuler = (yaw, pitch, roll) =>
  quat.normalize(quat.fromMatrix(eulerToMatrix(roll * DEG, pitch * DEG, yaw * DEG)));

/* ───────────────────────── 현재 자세 ───────────────────────── */

/** 모드에 상관없이 현재 자세를 한 벌로 낸다 */
function currentPose() {
  if (state.mode === 'quat') {
    const q = quat.fromAxisAngle(state.axis, state.angle * DEG);
    const R = quat.toMatrix(q);
    const decomp = matrixToEuler(R);
    // 쿼터니언 모드에서는 링이 "이 자세의 ZYX 분해" 를 보여준다.
    return { q, R, decomp, ringAngles: decomp };
  }
  const phi = state.roll * DEG;
  const theta = state.pitch * DEG;
  const psi = state.yaw * DEG;
  const R = eulerToMatrix(phi, theta, psi);
  return {
    q: quat.normalize(quat.fromMatrix(R)),
    R,
    decomp: matrixToEuler(R),
    // 오일러 모드에서는 슬라이더 값이 곧 링 각도다 (분해값이 아니라).
    ringAngles: { phi, theta, psi },
  };
}

/* ───────────────────────── 슬라이더 ───────────────────────── */

const sliders = {
  yaw: $('yaw'), pitch: $('pitch'), roll: $('roll'),
  ax: $('ax'), ay: $('ay'), az: $('az'), ang: $('ang'),
};

function readSliders() {
  state.yaw = Number(sliders.yaw.value);
  state.pitch = Number(sliders.pitch.value);
  state.roll = Number(sliders.roll.value);
  state.axis = [Number(sliders.ax.value), Number(sliders.ay.value), Number(sliders.az.value)];
  state.angle = Number(sliders.ang.value);
}

function writeSliders() {
  sliders.yaw.value = String(state.yaw);
  sliders.pitch.value = String(state.pitch);
  sliders.roll.value = String(state.roll);
  sliders.ax.value = String(state.axis[0]);
  sliders.ay.value = String(state.axis[1]);
  sliders.az.value = String(state.axis[2]);
  sliders.ang.value = String(state.angle);
}

/* ───────────────────────── 갱신 ───────────────────────── */

function update() {
  const pose = currentPose();

  scene.setAngles(pose.ringAngles.phi, pose.ringAngles.theta, pose.ringAngles.psi);

  $('yaw-out').textContent = state.yaw + '°';
  $('pitch-out').textContent = state.pitch + '°';
  $('roll-out').textContent = state.roll + '°';
  $('ax-out').textContent = state.axis[0].toFixed(3);
  $('ay-out').textContent = state.axis[1].toFixed(3);
  $('az-out').textContent = state.axis[2].toFixed(3);
  $('ang-out').textContent = state.angle + '°';

  if (state.mode === 'quat') {
    $('qdisp').textContent = qStr(pose.q);
    const aa = quat.toAxisAngle(pose.q);
    $('ndisp').textContent =
      '(' + aa.axis.map((v) => (v < 0 ? '' : ' ') + v.toFixed(4)).join(', ') + ')  α ' +
      (aa.angle * RAD).toFixed(2) + '°';
    const qe = $('qeuler');
    if (pose.decomp.degenerate) {
      qe.textContent = '분해 불가 (짐벌락 — φ 와 ψ 가 따로 정해지지 않는다)';
      qe.classList.add('undecidable');
    } else {
      qe.classList.remove('undecidable');
      qe.textContent =
        'φ ' + (pose.decomp.phi * RAD).toFixed(2) + '°  θ ' +
        (pose.decomp.theta * RAD).toFixed(2) + '°  ψ ' +
        (pose.decomp.psi * RAD).toFixed(2) + '°';
    }
  }

  // 계기판 — det E. 오일러 모드는 슬라이더 θ, 쿼터니언 모드는 분해된 θ 를 쓴다.
  const det = gimbalMeasure(pose.ringAngles.phi, pose.ringAngles.theta, pose.ringAngles.psi);
  $('det').textContent = fixed4(det);

  const gauge = document.querySelector('.gauge');
  const dof = $('dof');
  gauge.classList.remove('warn', 'locked');
  dof.classList.remove('warn', 'locked');

  if (pose.decomp.degenerate) {
    gauge.classList.add('locked');
    dof.classList.add('locked');
    dof.textContent = '짐벌락 — 자유도 2 (E 가 특이행렬)';
  } else if (Math.abs(det) < NEAR_LOCK) {
    gauge.classList.add('warn');
    dof.classList.add('warn');
    dof.textContent = '짐벌락 근접 — 자유도 3 → 2';
  } else {
    dof.textContent = '자유도 3 — 정상';
  }

  $('mat').innerHTML = pose.R.flat()
    .map((v) => {
      const cls = Math.abs(v) < 5e-5 ? 'zero' : v < 0 ? 'neg' : '';
      return '<span class="' + cls + '">' + fixed4(v) + '</span>';
    })
    .join('');

  const degen = $('degen');
  degen.classList.toggle('on', pose.decomp.degenerate);
  degen.textContent = pose.decomp.degenerate
    ? 'matrixToEuler → degenerate: true  (φ 와 ψ 가 따로 정해지지 않는다)'
    : 'matrixToEuler → degenerate: false  ·  φ ' +
      (pose.decomp.phi * RAD).toFixed(1) + '°  θ ' +
      (pose.decomp.theta * RAD).toFixed(1) + '°  ψ ' +
      (pose.decomp.psi * RAD).toFixed(1) + '°';

  renderAB();
}

/* ───────────────────────── 모드 전환 ───────────────────────── */

function setMode(mode) {
  if (state.mode === mode) return;
  const pose = currentPose(); // 전환 전 자세를 붙든다

  state.mode = mode;
  $('mode-euler').classList.toggle('on', mode === 'euler');
  $('mode-quat').classList.toggle('on', mode === 'quat');
  $('euler-controls').hidden = mode !== 'euler';
  $('quat-controls').hidden = mode !== 'quat';

  if (mode === 'quat') {
    // 축-각으로 옮긴다. 슬라이더가 α ∈ [−180,180] 이라 α > 180° 는 축을 뒤집어 접는다.
    let axis = quat.toAxisAngle(pose.q).axis;
    let angle = quat.toAxisAngle(pose.q).angle;
    if (angle > Math.PI) {
      angle = 2 * Math.PI - angle;
      axis = axis.map((v) => -v);
    }
    state.axis = axis.map((v) => Math.round(v * 1000) / 1000);
    state.angle = Math.round(angle * RAD);
  } else {
    // 오일러로 옮긴다. 짐벌락이면 분해가 유일하지 않아 한 가지 표현만 잡힌다.
    state.roll = Math.round(pose.decomp.phi * RAD);
    state.pitch = Math.round(pose.decomp.theta * RAD);
    state.yaw = Math.round(pose.decomp.psi * RAD);
  }

  writeSliders();
  exitCompare();
  update();
}

/* ───────────────────────── 짐벌락 실증 (2단계) ───────────────────────── */

function proveGimbalLock() {
  if (state.mode !== 'euler') setMode('euler');
  if (Math.abs(Math.abs(state.pitch) - 90) > 1e-9) {
    state.pitch = 90;
    writeSliders();
    update();
  }

  const th = state.pitch * DEG;
  const phi = state.roll * DEG;
  const psi = state.yaw * DEG;
  const d = PROOF_DELTA * DEG;

  // θ>0 이면 R ~ (φ−ψ) → roll 은 반대 부호, θ<0 이면 R ~ (φ+ψ) → 같은 부호
  const k = state.pitch > 0 ? -1 : 1;
  const combo = state.pitch > 0 ? '(φ − ψ)' : '(φ + ψ)';

  const Ryaw = eulerToMatrix(phi, th, psi + d);
  const R0 = eulerToMatrix(phi, th, psi);
  const eMatch = maxDiff(Ryaw, eulerToMatrix(phi + k * d, th, psi));
  const eOther = maxDiff(Ryaw, eulerToMatrix(phi - k * d, th, psi));
  const eNull = maxDiff(R0, eulerToMatrix(phi + d, th, psi - k * d));

  let sweepMatch = 0;
  let sweepNull = 0;
  for (let deg = 5; deg <= 180; deg += 5) {
    const s = deg * DEG;
    sweepMatch = Math.max(sweepMatch,
      maxDiff(eulerToMatrix(phi, th, psi + s), eulerToMatrix(phi + k * s, th, psi)));
    sweepNull = Math.max(sweepNull,
      maxDiff(R0, eulerToMatrix(phi + s, th, psi - k * s)));
  }

  const cls = (e) => (e < 1e-12 ? 'hi' : 'lo');
  const verdict = (e) => (e < 1e-12 ? '같은 회전' : '다른 회전');

  $('proof').innerHTML =
    '<span class="em">θ ' + sgn(state.pitch) + '   φ ' + sgn(state.roll) + '   ψ ' + sgn(state.yaw) + '</span>\n' +
    'θ = ±90° 에서 R 은 ' + combo + ' 에만 의존한다.\n\n' +
    '① yaw +30°  vs  roll ' + sgn(k * PROOF_DELTA) + '\n' +
    '     max|ΔR| = <span class="' + cls(eMatch) + '">' + expo(eMatch) + '</span>   ' + verdict(eMatch) + '\n' +
    '② yaw +30°  vs  roll ' + sgn(-k * PROOF_DELTA) + '   (대조군)\n' +
    '     max|ΔR| = <span class="' + cls(eOther) + '">' + expo(eOther) + '</span>   ' + verdict(eOther) + '\n' +
    '③ φ +30° 와 ψ ' + sgn(-k * PROOF_DELTA) + ' 를 동시에\n' +
    '     max|ΔR| = <span class="' + cls(eNull) + '">' + expo(eNull) + '</span>   R 이 아예 안 변한다\n\n' +
    'δ 를 5°…180° 로 훑어도\n' +
    '  ① max ' + expo(sweepMatch) + '    ③ max ' + expo(sweepNull) + '\n\n' +
    '<span class="em">자유도 3 → 2.</span> ③ 방향으로는 아무리 움직여도\n' +
    '자세가 변하지 않는다 — 축 하나가 사라졌다.';
}

/* ───────────────────────── 보간 비교 ───────────────────────── */

function renderAB() {
  for (const [key, slot] of [['A', 'a'], ['B', 'b']]) {
    const q = key === 'A' ? state.poseA : state.poseB;
    const user = key === 'A' ? state.aUser : state.bUser;
    const flag = $(slot + '-flag');
    const box = $(slot + '-q');
    if (!q) {
      flag.className = 'ab-flag';
      flag.textContent = '없음';
      box.textContent = '—';
      continue;
    }
    const d = matrixToEuler(quat.toMatrix(q));
    flag.className = 'ab-flag ' + (d.degenerate ? 'degen' : user ? 'set' : '');
    flag.textContent = d.degenerate ? '분해 불가' : user ? '지정됨' : '기본값';
    box.textContent =
      'q ' + qStr(q) + '\n' +
      (d.degenerate
        ? '오일러 분해 불가'
        : 'φ' + (d.phi * RAD).toFixed(1) + ' θ' + (d.theta * RAD).toFixed(1) +
          ' ψ' + (d.psi * RAD).toFixed(1));
  }
}

function enterCompare() {
  state.view = 'compare';
  compare.setVisible(true);
  $('hint').textContent = '비교 뷰 — 왼쪽 오일러 lerp · 오른쪽 slerp';
}

function exitCompare() {
  if (state.view !== 'compare') return;
  compare.stop();
  compare.setVisible(false);
  state.view = 'edit';
  $('playhead').hidden = true;
  $('play').disabled = false;
  $('hint').textContent = '드래그 — 궤도 · 휠 — 거리';
}

function statLine(label, s, cls) {
  return '<span class="' + cls + '">' + pad(label, 12) + '</span>' +
    padL(s.mean.toFixed(4) + '°', 11) +
    padL(s.std.toFixed(4) + '°', 11) +
    padL(s.max.toFixed(4) + '°', 11);
}

function renderStats(result) {
  const u = result.uniform;
  const f = result.frame;

  const ratio =
    u.slerp.std < 1e-9
      ? 'slerp ≈ 0 (기계 정밀도 ' + expo(u.slerp.std) + '°)'
      : '오일러 / slerp = ' + (u.euler.std / u.slerp.std).toFixed(1) + ' 배';

  const excess = (u.euler.total / u.slerp.total - 1) * 100;

  const degenNote =
    result.plan.degenerateA || result.plan.degenerateB
      ? '\n<span class="lo">주의 — ' +
        (result.plan.degenerateA ? 'A' : '') +
        (result.plan.degenerateA && result.plan.degenerateB ? '·' : '') +
        (result.plan.degenerateB ? 'B' : '') +
        ' 의 오일러 분해가 유일하지 않다 (degenerate).\n' +
        '오일러 보간의 출발/도착 각 자체가 임의로 골라진 값이다.</span>\n'
      : '';

  $('stats').innerHTML =
    '<span class="em">균일 t 격자 · ' + u.steps + ' 스텝 · 프레임 지터 없음</span>\n' +
    pad('', 12) + padL('평균', 11) + padL('표준편차', 10) + padL('최대', 11) + '\n' +
    statLine('오일러 lerp', u.euler, 'me') + '\n' +
    statLine('slerp', u.slerp, 'ms') + '\n' +
    '표준편차  ' + ratio + '\n\n' +
    '<span class="em">경로 길이 (총 회전각)</span>\n' +
    '  오일러 ' + u.euler.total.toFixed(3) + '°   slerp ' + u.slerp.total.toFixed(3) + '°\n' +
    '  측지선 ' + u.geodesic.toFixed(3) + '°   오일러 초과 ' +
    (excess >= 0 ? '+' : '') + excess.toFixed(2) + '%\n' +
    degenNote +
    '\n<span class="em">실제 렌더 프레임 · 온전한 N=' + f.euler.n + ' · Δt ' +
    f.meanDt.toFixed(1) + '±' + f.dtStd.toFixed(1) + 'ms</span>\n' +
    statLine('오일러 lerp', f.euler, 'me') + '\n' +
    statLine('slerp', f.slerp, 'ms') + '\n' +
    '<span class="dim">처음·마지막 부분 프레임은 뺐다. Δt 지터는 두 방식 모두에 섞여 있다.\n' +
    '순수한 비교는 위쪽 균일 격자 값이다.</span>';
}

function play() {
  if (!state.poseA || !state.poseB) return;
  enterCompare();
  $('playhead').hidden = false;
  $('playbar').style.width = '0%';
  $('play').disabled = true;
  $('stats').textContent = '재생 중…';

  compare.play(state.poseA, state.poseB, {
    duration: PLAY_MS,
    onProgress: (t) => {
      $('playbar').style.width = (t * 100).toFixed(1) + '%';
    },
    onDone: (result) => {
      $('play').disabled = false;
      renderStats(result);
    },
  });
}

/* ───────────────────────── 배선 ───────────────────────── */

for (const el of Object.values(sliders)) {
  el.addEventListener('input', () => {
    exitCompare();
    readSliders();
    update();
  });
}

$('mode-euler').addEventListener('click', () => setMode('euler'));
$('mode-quat').addEventListener('click', () => setMode('quat'));

$('to90').addEventListener('click', () => {
  exitCompare();
  if (state.mode !== 'euler') setMode('euler');
  state.pitch = 90;
  writeSliders();
  update();
});

$('reset').addEventListener('click', () => {
  exitCompare();
  state.yaw = state.pitch = state.roll = 0;
  state.axis = [0, 0, 1];
  state.angle = 0;
  writeSliders();
  update();
  scene.resetCamera();
  $('proof').textContent = 'θ = 90° 로 보낸 뒤 눌러라.';
});

$('prove').addEventListener('click', () => {
  exitCompare();
  proveGimbalLock();
});

$('setA').addEventListener('click', () => {
  state.poseA = currentPose().q;
  state.aUser = true;
  exitCompare();
  renderAB();
});

$('setB').addEventListener('click', () => {
  state.poseB = currentPose().q;
  state.bUser = true;
  exitCompare();
  renderAB();
});

$('play').addEventListener('click', play);
$('toEdit').addEventListener('click', exitCompare);

// A 와 B 를 거의 같은 자세로 놓아도 slerp 가 터지지 않는지 보이는 버튼.
// Ω→0 이라 sin Ω 로 나누는 경로가 무너지는 구간이다.
$('nearAB').addEventListener('click', () => {
  const q = currentPose().q;
  state.poseA = q;
  state.poseB = quat.normalize(
    quat.multiply(quat.fromAxisAngle([0.3, -0.7, 0.5], NEAR_IDENTICAL_DEG * DEG), q)
  );
  state.aUser = state.bUser = true;
  renderAB();
  play();
});

$('crossAB').addEventListener('click', () => {
  state.poseA = poseFromEuler(PRESET_CROSS.a.yaw, PRESET_CROSS.a.pitch, PRESET_CROSS.a.roll);
  state.poseB = poseFromEuler(PRESET_CROSS.b.yaw, PRESET_CROSS.b.pitch, PRESET_CROSS.b.roll);
  state.aUser = state.bUser = false;
  renderAB();
  play();
});

new ResizeObserver(() => scene.resize()).observe(canvas);

/* ───────────────────────── 시작 ───────────────────────── */

state.poseA = poseFromEuler(PRESET_CROSS.a.yaw, PRESET_CROSS.a.pitch, PRESET_CROSS.a.roll);
state.poseB = poseFromEuler(PRESET_CROSS.b.yaw, PRESET_CROSS.b.pitch, PRESET_CROSS.b.roll);

readSliders();
update();

/* ─────────────────────── GLB 기체 로딩 ─────────────────────── */

// 앱은 삼각대로 먼저 뜬다. GLB 가 도착하면 갈아 끼우고, 실패하면 삼각대로 남는다.
// 에셋 로딩이 앱을 멈추게 하지 않는다.
let assetState = { ok: false, pending: true };

function showAssetNote(text, kind) {
  const el = document.getElementById('asset-note');
  el.hidden = false;
  el.className = kind;
  el.textContent = text;
}

loadAircraft().then((res) => {
  assetState = { ...res, pending: false };
  if (res.ok) {
    scene.setPayload(res.create(null));
    compare.setObjectFactory(res.create);
    update();
  } else {
    showAssetNote(
      'GLB 로드 실패 (' + MODEL_URL + ') — 삼각대로 폴백. ' + (res.error || ''),
      'fallback'
    );
  }
});

// README 의 수치를 콘솔에서 다시 재 볼 수 있게 열어 둔다.
window.rotlab = {
  state, update, scene, compare, debug: scene.debug,
  eulerToMatrix, matrixToEuler, gimbalMeasure, quat, maxDiff,
  measureUniform, angleFormulaCheck, poseFromEuler,
  get asset() { return assetState; },
  setAngles(yaw, pitch, roll) {
    if (state.mode !== 'euler') setMode('euler');
    exitCompare();
    state.yaw = yaw; state.pitch = pitch; state.roll = roll;
    writeSliders(); update();
  },
  setPoses(qA, qB) {
    state.poseA = qA; state.poseB = qB;
    state.aUser = state.bUser = true;
    renderAB();
  },
  play,
};
