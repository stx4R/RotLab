/**
 * src/scene.js — 중첩 짐벌 링 + 방향이 보이는 삼각대
 *
 * 규칙: 회전은 전부 1단계 모듈(mat3 / euler)이 계산한다.
 * THREE 쪽은 "이미 만들어진 행렬을 Matrix4 에 넣어 적용" 만 한다.
 * THREE.Euler / Quaternion / Matrix4 의 회전 생성 메서드는 쓰지 않는다.
 * (Matrix4.set 은 성분을 그대로 써 넣는 것이라 회전 API 가 아니다.)
 *
 * 월드는 Z-up 이다. yaw 축(Z)이 화면에서 수직이라야 직관과 맞는다.
 */

import * as THREE from 'three';
import { identity, multiply, fromColumns } from './mat3.js';
import { rotX, rotY, rotZ, eulerToMatrix } from './euler.js';

/* ───────────────────────────── 상수 ───────────────────────────── */

const AXIS_COLOR = {
  z: 0x3a7ae8, // yaw   파랑
  y: 0x2f9e6f, // pitch 초록
  x: 0xe8853a, // roll  주황
};

const BG = 0x14161a;

const RING = {
  z: { radius: 2.70, tube: 0.050 },
  y: { radius: 2.25, tube: 0.050 },
  x: { radius: 1.80, tube: 0.050 },
};

/**
 * 지오메트리 기본 축을 링/막대의 축으로 돌려놓는 상수 행렬.
 * 런타임 회전이 아니라 모델링 단계의 고정 정렬이다. 그래도 1단계 모듈로 만든다.
 *
 *   TorusGeometry     기본 축 = +Z
 *   Cylinder / Cone   기본 축 = +Y
 */
const ALIGN = {
  torusZ: identity(),
  torusY: rotX(Math.PI / 2),   // +Z → ∓Y (축선은 Y)
  torusX: rotY(Math.PI / 2),   // +Z → +X
  barX: rotZ(-Math.PI / 2),    // +Y → +X
  barY: identity(),
  barZ: rotX(Math.PI / 2),     // +Y → +Z
};

const CAMERA_UP = [0, 0, 1];
const EL_LIMIT = (85 * Math.PI) / 180;
const DIST_RANGE = [4.0, 30.0];

/* ─────────────────────── 작은 벡터 도우미 ─────────────────────── */

const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

function unit(v) {
  const n = Math.hypot(v[0], v[1], v[2]);
  return [v[0] / n, v[1] / n, v[2] / n];
}

const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

/**
 * 내 3×3 회전행렬 R 과 평행이동 t 를 Object3D 의 로컬 행렬에 직접 써 넣는다.
 * Matrix4.set 은 행 우선 인자를 받는다.
 */
export function setMatrix(obj, R, t = [0, 0, 0]) {
  obj.matrixAutoUpdate = false;
  obj.matrix.set(
    R[0][0], R[0][1], R[0][2], t[0],
    R[1][0], R[1][1], R[1][2], t[1],
    R[2][0], R[2][1], R[2][2], t[2],
    0, 0, 0, 1
  );
  obj.matrixWorldNeedsUpdate = true;
}

/* ───────────────────────────── 부품 ───────────────────────────── */

/**
 * 링 하나.
 *
 *   group        ← 이 링까지의 누적 회전행렬이 들어간다
 *     aligned    ← 토러스 축을 링 축에 맞추는 고정 행렬
 *       torus
 *       marker   ← 인덱스 마커. 토러스는 자기 축 둘레로 회전 대칭이라
 *                  이 구슬이 없으면 링 자신의 스핀이 눈에 안 보인다.
 */
function makeRing(spec, alignMatrix, color) {
  const group = new THREE.Group();
  const aligned = new THREE.Group();
  setMatrix(aligned, alignMatrix);
  group.add(aligned);

  const torus = new THREE.Mesh(
    new THREE.TorusGeometry(spec.radius, spec.tube, 16, 160),
    new THREE.MeshStandardMaterial({
      color,
      roughness: 0.42,
      metalness: 0.15,
      emissive: color,
      emissiveIntensity: 0.12,
    })
  );
  torus.matrixAutoUpdate = false;
  aligned.add(torus);

  const marker = new THREE.Mesh(
    new THREE.SphereGeometry(spec.tube * 2.6, 20, 14),
    new THREE.MeshStandardMaterial({
      color,
      roughness: 0.3,
      metalness: 0.1,
      emissive: color,
      emissiveIntensity: 0.85,
    })
  );
  setMatrix(marker, identity(), [spec.radius, 0, 0]);
  aligned.add(marker);

  return group;
}

/**
 * 삼각대 — +X, +Y, +Z 세 방향으로만 팔이 나가고 끝에 화살촉이 달린다.
 * 세 방향이 서로 다르고 반대쪽이 비어 있어서 어느 자세인지 한눈에 읽힌다.
 * (구는 쓰지 않는다. 회전이 전혀 안 보인다.)
 */
export function makeTripod(hubColor = 0x9aa4b2) {
  const group = new THREE.Group();

  const ARM = 1.15;
  const ARM_R = 0.038;
  const TIP_H = 0.30;
  const TIP_R = 0.095;

  const arms = [
    { align: ALIGN.barX, color: AXIS_COLOR.x },
    { align: ALIGN.barY, color: AXIS_COLOR.y },
    { align: ALIGN.barZ, color: AXIS_COLOR.z },
  ];

  for (const { align, color } of arms) {
    const material = new THREE.MeshStandardMaterial({
      color,
      roughness: 0.35,
      metalness: 0.2,
      emissive: color,
      emissiveIntensity: 0.18,
    });

    // 막대: 기본 축 +Y, 원점 중심 → 로컬 +Y 로 절반 밀어 0..ARM 을 채운다
    const bar = new THREE.Mesh(new THREE.CylinderGeometry(ARM_R, ARM_R, ARM, 18), material);
    setMatrix(bar, align, applyAlign(align, [0, ARM / 2, 0]));
    group.add(bar);

    const tip = new THREE.Mesh(new THREE.ConeGeometry(TIP_R, TIP_H, 22), material);
    setMatrix(tip, align, applyAlign(align, [0, ARM + TIP_H / 2, 0]));
    group.add(tip);
  }

  // 허브 — 팔 세 개가 만나는 곳. 작은 정육면체라 방향을 흐리지 않는다.
  const hub = new THREE.Mesh(
    new THREE.BoxGeometry(0.17, 0.17, 0.17),
    new THREE.MeshStandardMaterial({ color: hubColor, roughness: 0.5, metalness: 0.3 })
  );
  hub.matrixAutoUpdate = false;
  group.add(hub);

  return group;
}

/** 정렬 행렬을 로컬 오프셋에 먹여서 월드 오프셋을 만든다 */
function applyAlign(A, v) {
  return [
    A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
    A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
    A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
  ];
}

/* ───────────────────────────── 씬 ───────────────────────────── */

/**
 * @param {HTMLCanvasElement} canvas
 * @returns {{setAngles:Function, resize:Function, resetCamera:Function, dispose:Function}}
 */
export function createScene(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(BG);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 200);

  const key = new THREE.DirectionalLight(0xffffff, 2.6);
  key.position.set(6, -7, 8);
  scene.add(key);
  scene.add(new THREE.AmbientLight(0x6f7a8a, 1.15));

  // 링 세 개. 바깥부터 Z → Y → X.
  const ringZ = makeRing(RING.z, ALIGN.torusZ, AXIS_COLOR.z);
  const ringY = makeRing(RING.y, ALIGN.torusY, AXIS_COLOR.y);
  const ringX = makeRing(RING.x, ALIGN.torusX, AXIS_COLOR.x);
  // 중앙 오브젝트. 기본은 삼각대이고, GLB 가 로드되면 setPayload 로 갈아 끼운다.
  let mainObject = makeTripod();
  scene.add(ringZ, ringY, ringX, mainObject);

  // 오브젝트를 갈아 끼울 때 자세를 이어받으려고 마지막 회전을 들고 있는다.
  let lastR = identity();

  /* ── 카메라 궤도 (직접 구현. OrbitControls 안 쓴다) ── */

  const orbit = { az: -1.05, el: 0.42, dist: 9.2 };

  function updateCamera() {
    const ce = Math.cos(orbit.el);
    const p = [
      orbit.dist * ce * Math.cos(orbit.az),
      orbit.dist * ce * Math.sin(orbit.az),
      orbit.dist * Math.sin(orbit.el),
    ];
    // THREE 카메라는 로컬 −Z 를 본다. 원점을 보게 하려면 로컬 +Z 가 카메라 쪽이다.
    const zc = unit(p);
    const xc = unit(cross(CAMERA_UP, zc));
    const yc = cross(zc, xc);
    setMatrix(camera, fromColumns(xc, yc, zc), p);
  }

  let dragging = false;
  let lastX = 0;
  let lastY = 0;

  const onPointerDown = (e) => {
    dragging = true;
    lastX = e.clientX;
    lastY = e.clientY;
    canvas.setPointerCapture(e.pointerId);
  };

  const onPointerMove = (e) => {
    if (!dragging) return;
    const dx = e.clientX - lastX;
    const dy = e.clientY - lastY;
    lastX = e.clientX;
    lastY = e.clientY;
    orbit.az -= dx * 0.005;
    orbit.el = clamp(orbit.el + dy * 0.005, -EL_LIMIT, EL_LIMIT);
    updateCamera();
    render();
  };

  const onPointerUp = (e) => {
    dragging = false;
    if (canvas.hasPointerCapture(e.pointerId)) canvas.releasePointerCapture(e.pointerId);
  };

  const onWheel = (e) => {
    e.preventDefault();
    orbit.dist = clamp(orbit.dist * Math.exp(e.deltaY * 0.0012), DIST_RANGE[0], DIST_RANGE[1]);
    updateCamera();
    render();
  };

  canvas.addEventListener('pointerdown', onPointerDown);
  canvas.addEventListener('pointermove', onPointerMove);
  canvas.addEventListener('pointerup', onPointerUp);
  canvas.addEventListener('pointercancel', onPointerUp);
  canvas.addEventListener('wheel', onWheel, { passive: false });

  /* ── 회전 적용 ── */

  /**
   * 링 k 의 월드 회전 = (자기보다 바깥 링들의 회전) · (자기 스핀)
   *
   *   마운트            자기 스핀   →  월드 회전
   *   Z링   I           Rz(ψ)         Rz
   *   Y링   Rz          Ry(θ)         Rz·Ry
   *   X링   Rz·Ry       Rx(φ)         Rz·Ry·Rx = R
   *   삼각대 Rz·Ry·Rx   —             R
   *
   * 마운트가 링의 축 방향을 정하고, 자기 스핀은 축 방향을 바꾸지 않는다
   * (토러스가 자기 축 둘레로 대칭이라 마커로만 보인다).
   * 그래서 바깥 링은 안쪽 각도에 전혀 영향받지 않는다 — 이것이 중첩 짐벌이다.
   *
   * @param {number} phi   roll  (rad)
   * @param {number} theta pitch (rad)
   * @param {number} psi   yaw   (rad)
   */
  function setAngles(phi, theta, psi) {
    const Rz = rotZ(psi);
    const Rzy = multiply(Rz, rotY(theta));
    const R = eulerToMatrix(phi, theta, psi); // = Rz·Ry·Rx

    setMatrix(ringZ, Rz);
    setMatrix(ringY, Rzy);
    setMatrix(ringX, R);
    setMatrix(mainObject, R);
    lastR = R;

    render();
  }

  /* ── 렌더 / 리사이즈 ── */

  function render() {
    renderer.render(scene, camera);
  }

  function resize() {
    const w = canvas.clientWidth || 1;
    const h = canvas.clientHeight || 1;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    render();
  }

  function resetCamera() {
    orbit.az = -1.05;
    orbit.el = 0.42;
    orbit.dist = 9.2;
    updateCamera();
    render();
  }

  updateCamera();
  setAngles(0, 0, 0);
  resize();

  /** 보간 비교 뷰용 — 짐벌 링은 오일러각 조작 장치라 비교 재생 중에는 감춘다 */
  function setRingsVisible(v) {
    ringZ.visible = ringY.visible = ringX.visible = v;
    render();
  }

  /** 편집용 삼각대 표시 여부. 비교 재생 중에는 좌우 두 개로 대체된다. */
  function setMainVisible(v) {
    mainObject.visible = v;
    render();
  }

  /**
   * 중앙 오브젝트를 갈아 끼운다 (삼각대 ↔ GLB 기체).
   * 현재 자세와 표시 여부를 그대로 이어받는다. 회전 계산에는 손대지 않는다.
   */
  function setPayload(next) {
    const wasVisible = mainObject.visible;
    scene.remove(mainObject);
    mainObject = next;
    mainObject.visible = wasVisible;
    setMatrix(mainObject, lastR);
    scene.add(mainObject);
    render();
  }

  /** 카메라 궤도를 코드에서 옮긴다 (비교 뷰 진입 시 좌우 배치를 맞추려고) */
  function setOrbit(next) {
    if (next.az !== undefined) orbit.az = next.az;
    if (next.el !== undefined) orbit.el = clamp(next.el, -EL_LIMIT, EL_LIMIT);
    if (next.dist !== undefined) orbit.dist = clamp(next.dist, DIST_RANGE[0], DIST_RANGE[1]);
    updateCamera();
    render();
  }

  // 검증/디버그용 노출. 렌더링 경로는 이걸 쓰지 않는다.
  const debug = {
    scene, camera,
    rings: { z: ringZ, y: ringY, x: ringX },
    get tripod() { return mainObject; },
  };

  return { setAngles, resize, resetCamera, debug,
           root: scene, render, setRingsVisible, setMainVisible, setOrbit, setPayload };
}
