/**
 * src/main.js — UI 와 씬/비교를 잇는다.
 *
 * 회전에 관한 숫자는 전부 1단계 모듈(euler.js / quat.js)이 낸다.
 * 화면에 찍히는 행렬과 씬에 적용되는 행렬은 같은 함수 호출 결과다.
 *
 * 화면은 Toss Design System 기반이다 (style.css 의 토큰).
 * 이 파일은 그 화면의 상태 기계일 뿐, 수치는 여전히 수학 모듈에서만 온다.
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

/** 토스트가 떠 있는 시간 */
const TOAST_MS = 2400;

/** 검증 리포트의 씨앗 전수 검사 범위 — 0 … SWEEP_SEEDS−1 */
const SWEEP_SEEDS = 400;

const $ = (id) => document.getElementById(id);

const canvas = $('view');
const scene = createScene(canvas);
const compare = createCompare(scene);

const PROOF_IDLE = 'θ를 90°로 보낸 뒤 눌러주세요.';

/* ───────────────────────── 숫자 포맷 ───────────────────────── */

/** −0.0000 이 뜨지 않게 아주 작은 값은 0 으로 접는다 */
const fixed4 = (v) => (Math.abs(v) < 5e-5 ? '0.0000' : v.toFixed(4));
const expo = (v) => (Math.abs(v) < 1e-300 ? '0.00e+0' : v.toExponential(2));
const expo3 = (v) => (Number.isFinite(v) ? v.toExponential(3) : String(v));
/** 허용치처럼 딱 떨어지는 값은 1e-12 로 짧게 (1.000e-12 은 표에서 시끄럽다) */
const tolStr = (v) => (Number.isFinite(v) ? v.toExponential().replace('e+', 'e') : String(v));
const sgn = (n) => (n < 0 ? '−' : '+') + Math.abs(n) + '°';

const esc = (s) =>
  String(s).replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));

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
  screen: 'lab', // 'lab' | 'verify'
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

  preset: 'c',
  playing: false,
  result: null,
};

/**
 * 프리셋 — README 의 촬영/재현용 표와 같은 값이다.
 * angles 프리셋은 자세 하나, ab 프리셋은 A→B 한 쌍을 잡고 바로 재생한다.
 */
const PRESETS = [
  {
    key: 'a',
    label: '정상',
    note: '자유도 3, det E = 1.0000. Z링만 자전하고 안쪽 링은 실려 돌아요.',
    angles: [45, 0, 0],
  },
  {
    key: 'b',
    label: '짐벌락',
    note: 'det E = 0.0000, degenerate: true. yaw+30과 roll−30이 같은 회전이에요.',
    angles: [90, 90, 5],
  },
  {
    key: 'c',
    label: '극 통과 보간',
    // B 의 pitch 가 95° — θ=90° 극을 실제로 가로지른다.
    // Δψ 175° · Δφ −175° — ±180° 를 피해 잡았다. 정확히 ±180° 면 래핑 방향이
    // 원리적으로 미정이라 부동소수점 부호로 갈리고 수치가 재현되지 않는다.
    note: 'θ=90° 극을 실제로 가로질러요. 오일러 std 0.3186° 대 slerp 8.37e-15°, 경로 초과 +115.9%.',
    ab: [[90, -45, 5], [85, 95, 10]],
  },
  {
    key: 'd',
    label: 'θ̇·ψ̇·φ̇ = 0',
    note: '표준편차는 둘 다 1e-14인데 경로만 +31.0% 길어요. 요동만 보면 놓치는 실패예요.',
    ab: [[-70, 80, 0], [100, -80, 0]],
  },
  {
    key: 'e',
    label: '경로 폭주',
    note: '요동 없이 경로만 15.19배. slerp가 22.4° 도는 동안 오일러는 339.7° 휘돌아요.',
    ab: [[135, 85, 15], [145, 95, 5]],
  },
];

const presetByKey = (key) => PRESETS.find((p) => p.key === key);

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

/* ───────────────────────── 프리셋 칩 ───────────────────────── */

function renderChips() {
  $('chips').innerHTML = PRESETS.map(
    (p) =>
      '<button type="button" class="chip' + (state.preset === p.key ? ' on' : '') +
      '" data-preset="' + p.key + '">' + esc(p.label) + '</button>'
  ).join('');
}

function setPresetNote(text) {
  $('preset-note').textContent = text;
}

function pickPreset(p) {
  if (p.angles) {
    exitCompare();
    if (state.mode !== 'euler') setMode('euler');
    state.preset = p.key;
    [state.yaw, state.pitch, state.roll] = p.angles;
    writeSliders();
    renderChips();
    setPresetNote(p.note);
    update();
    // (b) 짐벌락 프리셋은 실증까지 한 번에 보여준다.
    if (p.key === 'b') proveGimbalLock();
    return;
  }
  state.preset = p.key;
  state.poseA = poseFromEuler(...p.ab[0]);
  state.poseB = poseFromEuler(...p.ab[1]);
  state.aUser = state.bUser = false;
  renderChips();
  setPresetNote(p.note);
  renderAB();
  play();
}

/* ───────────────────────── 토스트 ───────────────────────── */

let toastTimer = 0;

function showToast(text) {
  clearTimeout(toastTimer);
  $('toast-body').textContent = text;
  $('toast').hidden = false;
  toastTimer = setTimeout(() => { $('toast').hidden = true; }, TOAST_MS);
}

/* ───────────────────────── 스테퍼 ───────────────────────── */

function renderStepper() {
  const done = (label) => ({ cls: 'done', mark: '✓', label });
  const steps = [
    done('A·B 지정'),
    state.playing
      ? { cls: 'now', mark: '2', label: '재생' }
      : state.result ? done('재생') : { cls: '', mark: '2', label: '재생' },
    state.result ? done('결과') : { cls: '', mark: '3', label: '결과' },
  ];
  $('stepper').innerHTML = steps
    .map(
      (s) =>
        '<div class="step ' + s.cls + '">' +
        '<div class="mark">' + s.mark + '</div>' +
        '<div class="label">' + esc(s.label) + '</div>' +
        '<div class="line"></div>' +
        '</div>'
    )
    .join('');
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
      qe.textContent = '분해 불가 — 짐벌락이라 φ와 ψ가 따로 정해지지 않아요';
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

  const gauge = $('gauge');
  const dof = $('dof');
  gauge.classList.remove('warn', 'locked');
  dof.classList.remove('warn', 'locked');

  if (pose.decomp.degenerate) {
    gauge.classList.add('locked');
    dof.classList.add('locked');
    dof.textContent = '자유도 2';
    $('dof-note').textContent = 'E가 특이행렬이에요';
  } else if (Math.abs(det) < NEAR_LOCK) {
    gauge.classList.add('warn');
    dof.classList.add('warn');
    dof.textContent = '자유도 3 → 2';
    $('dof-note').textContent = '짐벌락에 가까워요';
  } else {
    dof.textContent = '자유도 3';
    $('dof-note').textContent = '정상이에요';
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
    ? 'matrixToEuler → degenerate: true — φ와 ψ가 따로 정해지지 않아요'
    : 'matrixToEuler → degenerate: false · φ ' +
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
    let { axis, angle } = quat.toAxisAngle(pose.q);
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
    'θ = ±90°에서 R은 ' + combo + ' 에만 의존해요.\n\n' +
    '① yaw +30°  vs  roll ' + sgn(k * PROOF_DELTA) + '\n' +
    '     max|ΔR| = <span class="' + cls(eMatch) + '">' + expo(eMatch) + '</span>   ' + verdict(eMatch) + '\n' +
    '② yaw +30°  vs  roll ' + sgn(-k * PROOF_DELTA) + '   (대조군)\n' +
    '     max|ΔR| = <span class="' + cls(eOther) + '">' + expo(eOther) + '</span>   ' + verdict(eOther) + '\n' +
    '③ φ +30°와 ψ ' + sgn(-k * PROOF_DELTA) + ' 를 동시에\n' +
    '     max|ΔR| = <span class="' + cls(eNull) + '">' + expo(eNull) + '</span>   R이 아예 안 변해요\n\n' +
    'δ를 5°…180°로 훑어도\n' +
    '  ① max ' + expo(sweepMatch) + '    ③ max ' + expo(sweepNull) + '\n\n' +
    '<span class="em">자유도 3 → 2.</span> ③ 방향으로는 아무리 움직여도\n' +
    '자세가 변하지 않아요 — 축 하나가 사라졌어요.';
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
  renderStepper();
}

function enterCompare() {
  state.view = 'compare';
  compare.setVisible(true);
  $('legend').hidden = false;
  $('hint').textContent = '비교 뷰 — 왼쪽 오일러 lerp · 오른쪽 slerp';
}

function exitCompare() {
  if (state.view !== 'compare') return;
  compare.stop();
  compare.setVisible(false);
  state.view = 'edit';
  state.playing = false;
  $('legend').hidden = true;
  $('playhead').hidden = true;
  setCta(false);
  $('hint').textContent = '드래그로 궤도, 휠로 거리를 바꿔요';
  renderStepper();
}

function setCta(playing) {
  const btn = $('play');
  btn.disabled = playing;
  btn.textContent = playing ? '재생 중…' : '3초 동안 재생하기';
}

/* ── 시트 ── */

/** 통계 표 12칸 — 머리행 + 오일러 + slerp */
function statTable(a, b) {
  const head = (t) => '<div class="head">' + t + '</div>';
  const cell = (t) => '<div>' + t + '</div>';
  const name = (t, cls) => '<div class="name ' + cls + '">' + t + '</div>';
  const row = (label, cls, s) =>
    name(label, cls) + cell(s.mean.toFixed(4) + '°') + cell(s.std.toFixed(4) + '°') +
    cell(s.max.toFixed(4) + '°');
  return (
    '<div class="head name"></div>' + head('평균') + head('표준편차') + head('최대') +
    row('오일러 lerp', 'euler', a) +
    row('slerp', 'slerp', b)
  );
}

function openSheet() {
  $('sheet-scrim').hidden = false;
  $('sheet').hidden = false;
}

function closeSheet() {
  $('sheet-scrim').hidden = true;
  $('sheet').hidden = true;
}

function renderSheet() {
  const r = state.result;
  if (!r) {
    $('sheet-title').textContent = '아직 재생 결과가 없어요';
    $('sheet-sub').textContent = '아래 재생 버튼을 누르면 두 방식의 통계가 여기 쌓여요.';
    $('sheet-result').hidden = true;
    return;
  }

  const u = r.uniform;
  const f = r.frame;
  const excess = (u.euler.total / u.slerp.total - 1) * 100;
  const sign = excess >= 0 ? '+' : '';

  $('sheet-title').textContent = '오일러 초과 ' + sign + excess.toFixed(2) + '%';
  $('sheet-sub').textContent =
    '같은 A에서 같은 B로 갔는데, 지나간 길이 달라요. 오일러 ' +
    u.euler.total.toFixed(1) + '° 대 slerp ' + u.slerp.total.toFixed(1) + '°.';
  $('sheet-result').hidden = false;

  $('grid-steps').textContent = '균일 t 격자 · ' + u.steps + ' 스텝';
  $('stat-table').innerHTML = statTable(u.euler, u.slerp);
  $('ratio-note').textContent =
    u.slerp.std < 1e-9
      ? 'slerp는 기계 정밀도 안에서 등속이에요 (' + expo(u.slerp.std) + '°)'
      : '오일러가 slerp의 ' + (u.euler.std / u.slerp.std).toFixed(1) + '배로 요동해요';

  $('excess').textContent = '오일러 초과 ' + sign + excess.toFixed(2) + '%';

  const maxTotal = Math.max(u.euler.total, u.slerp.total, u.geodesic);
  const barH = (v) => Math.max(4, (v / maxTotal) * 104).toFixed(1) + 'px';
  $('euler-total').textContent = u.euler.total.toFixed(1) + '°';
  $('slerp-total').textContent = u.slerp.total.toFixed(1) + '°';
  $('geo-total').textContent = u.geodesic.toFixed(1) + '°';
  $('euler-bar').style.height = barH(u.euler.total);
  $('slerp-bar').style.height = barH(u.slerp.total);
  $('geo-bar').style.height = barH(u.geodesic);

  const degenerate = r.plan.degenerateA || r.plan.degenerateB;
  $('degen-warn').hidden = !degenerate;
  if (degenerate) {
    $('degen-warn-body').textContent =
      (r.plan.degenerateA ? 'A' : '') +
      (r.plan.degenerateA && r.plan.degenerateB ? '·' : '') +
      (r.plan.degenerateB ? 'B' : '') +
      '의 오일러 분해가 유일하지 않아요 (degenerate). 오일러 보간의 출발·도착 각 자체가 ' +
      '임의로 골라진 값이라, 아래 수치는 그 선택에 딸려 있어요.';
  }

  $('frame-note').textContent =
    '온전한 N=' + f.euler.n + ' · Δt ' + f.meanDt.toFixed(1) + '±' + f.dtStd.toFixed(1) + 'ms';
  $('frame-table').innerHTML = statTable(f.euler, f.slerp);
}

function play() {
  if (!state.poseA || !state.poseB || state.playing) return;
  closeSheet();
  enterCompare();
  state.playing = true;
  $('playhead').hidden = false;
  $('playbar').style.width = '0%';
  setCta(true);
  renderStepper();

  compare.play(state.poseA, state.poseB, {
    duration: PLAY_MS,
    onProgress: (t) => {
      $('playbar').style.width = (t * 100).toFixed(1) + '%';
    },
    onDone: (result) => {
      state.playing = false;
      state.result = result;
      setCta(false);
      renderStepper();
      renderSheet();
      openSheet();
    },
  });
}

/* ───────────────────────── 검증 리포트 ───────────────────────── */

let verifyRan = false;

function showScreen(screen) {
  state.screen = screen;
  $('lab').hidden = screen !== 'lab';
  $('verify').hidden = screen !== 'verify';
  document.body.style.overflow = screen === 'verify' ? '' : 'hidden';
  if (screen === 'verify' && !verifyRan) {
    verifyRan = true;
    // 화면이 먼저 뜨고 나서 돌린다. 씨앗 400개 전수까지 1초 안쪽이다.
    // rAF 가 아니라 setTimeout 인 이유 — 배경 탭에서는 rAF 가 멈춰서 리포트가 영영 안 뜬다.
    setTimeout(runVerify, 0);
  }
  if (screen === 'lab') scene.resize();
}

async function runVerify() {
  let m;
  try {
    m = await import('./verify.js');
  } catch (err) {
    $('verify-title').textContent = '검증을 돌리지 못했어요';
    $('verify-sub').textContent = String(err && err.message ? err.message : err);
    return;
  }

  const report = m.runAll();

  // 씨앗 전수 검사 — 실패 재현성과 최악 오차를 이 자리에서 직접 잰다.
  let sweepFails = 0;
  let sweepWorst = 0;
  for (let seed = 0; seed < SWEEP_SEEDS; seed++) {
    const r = m.runAll({ seed, extras: false });
    if (!r.allPassed) sweepFails++;
    for (const x of r.results) if (x.maxError > sweepWorst) sweepWorst = x.maxError;
  }

  const failed = report.results.filter((r) => !r.passed);
  const summary = document.querySelector('.card.summary');
  summary.classList.toggle('ng', !report.allPassed);
  $('verify-mark').textContent = report.allPassed ? '✓' : '!';
  $('verify-title').textContent = report.allPassed
    ? report.results.length + '개 항목 모두 통과했어요'
    : failed.length + '개 항목이 실패했어요';
  $('verify-sub').textContent =
    '씨앗 ' + SWEEP_SEEDS + '개 전수로 실패 ' + sweepFails + '건';
  $('verify-lead').innerHTML =
    '직접 구현한 회전 수학을 three.js r' + esc(report.threeVersion) +
    ' 내장 구현과 성분 단위로 대조했어요. 전 항목 최악 오차는 ' +
    '<span class="num">' + expo3(sweepWorst) + '</span> 예요.';

  $('verify-rows').innerHTML = report.results
    .map(
      (r) =>
        '<div class="vrow' + (r.passed ? '' : ' ng') + '">' +
        '<div class="n">' + r.id + '</div>' +
        '<div class="main">' +
        '<div class="item">' + esc(r.name) + '</div>' +
        '<div class="ref">' + esc(r.reference) + ' · 표본 ' + r.samples + '</div>' +
        '</div>' +
        '<div class="right">' +
        '<div class="err">' + expo3(r.maxError) + '</div>' +
        '<div class="tol">기준 ' + tolStr(r.tolerance) + '</div>' +
        '</div>' +
        '</div>'
    )
    .join('');

  const slerp = report.results.find((r) => r.id === 5);
  const gimbal = report.results.find((r) => r.id === 6);

  $('note-slerp').innerHTML =
    '5번 표본에는 <code>dot(q₀,q₁) &lt; 0</code> 쌍이 ' +
    slerp.negativeDotPairs + '/' + slerp.pairs +
    ' 들어 있어요. 대척점 경우를 검사하지 못하면 검사 자체가 무의미하니, ' +
    '그런 쌍이 하나도 없으면 실패로 처리해요.';

  $('note-gimbal').innerHTML =
    '6번은 θ=±90°에서 <code>|det E| = ' + expo3(gimbal.maxSingularDet) + '</code> (&lt; ' +
    tolStr(gimbal.singularTolerance) + ' 요구)도 함께 확인해요.';

  // 콘솔이 여전히 본체다.
  m.printReport(report);
}

/* ───────────────────────── 배선 ───────────────────────── */

for (const el of Object.values(sliders)) {
  el.addEventListener('input', () => {
    exitCompare();
    readSliders();
    update();
  });
}

$('chips').addEventListener('click', (ev) => {
  const btn = ev.target.closest('.chip');
  if (!btn) return;
  const p = presetByKey(btn.dataset.preset);
  if (p) pickPreset(p);
});

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
  $('proof').textContent = PROOF_IDLE;
});

$('tipToggle').addEventListener('click', () => {
  $('tip').hidden = !$('tip').hidden;
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
  showToast('A 자세로 지정했어요');
});

$('setB').addEventListener('click', () => {
  state.poseB = currentPose().q;
  state.bUser = true;
  exitCompare();
  renderAB();
  showToast('B 자세로 지정했어요');
});

$('play').addEventListener('click', play);

$('openSheet').addEventListener('click', () => { renderSheet(); openSheet(); });
$('closeSheet').addEventListener('click', closeSheet);
$('sheet-scrim').addEventListener('click', closeSheet);

$('openVerify').addEventListener('click', () => showScreen('verify'));
$('closeVerify').addEventListener('click', () => showScreen('lab'));

// A 와 B 를 거의 같은 자세로 놓아도 slerp 가 터지지 않는지 보이는 버튼.
// Ω→0 이라 sin Ω 로 나누는 경로가 무너지는 구간이다.
$('nearAB').addEventListener('click', () => {
  const q = currentPose().q;
  state.poseA = q;
  state.poseB = quat.normalize(
    quat.multiply(quat.fromAxisAngle([0.3, -0.7, 0.5], NEAR_IDENTICAL_DEG * DEG), q)
  );
  state.aUser = state.bUser = true;
  state.preset = '';
  renderChips();
  setPresetNote(
    'A와 B를 0.005°만 떨어뜨렸어요. Ω→0이라 sin Ω로 나누는 경로가 무너지는 구간인데, ' +
    'nlerp로 넘어가서 터지지 않아요.'
  );
  renderAB();
  play();
});

new ResizeObserver(() => scene.resize()).observe(canvas);

/* ───────────────────────── 시작 ───────────────────────── */

{
  const p = presetByKey(state.preset);
  state.poseA = poseFromEuler(...p.ab[0]);
  state.poseB = poseFromEuler(...p.ab[1]);
  renderChips();
  setPresetNote(p.note);
}

readSliders();
update();
renderSheet();

/* ─────────────────────── GLB 기체 로딩 ─────────────────────── */

// 앱은 삼각대로 먼저 뜬다. GLB 가 도착하면 갈아 끼우고, 실패하면 삼각대로 남는다.
// 에셋 로딩이 앱을 멈추게 하지 않는다.
let assetState = { ok: false, pending: true };

function showAssetNote(html) {
  const el = $('asset-note');
  el.hidden = false;
  el.querySelector('.body').innerHTML = html;
}

loadAircraft().then((res) => {
  assetState = { ...res, pending: false };
  if (res.ok) {
    scene.setPayload(res.create(null));
    compare.setObjectFactory(res.create);
    update();
  } else {
    showAssetNote(
      '<b>기체 모델을 불러오지 못했어요</b><br>대신 삼각대로 보여드릴게요. ' +
      '회전 수치는 그대로예요 — <code>' + esc(MODEL_URL) + '</code>' +
      (res.error ? '<br>' + esc(res.error) : '')
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
    state.preset = '';
    renderChips();
    renderAB();
  },
  play,
};
