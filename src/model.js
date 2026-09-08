/**
 * src/model.js — GLB 기체 로딩과 축 보정
 *
 * 이 파일은 "모델을 앱 규약에 맞춰 씬에 붙이는" 일만 한다.
 * 회전 수학(mat3 / euler / quat)에는 아무것도 끼워 넣지 않는다.
 * 보정 행렬은 모델 아래 고정 자식 노드에 딱 한 번 적용되고,
 * 짐벌 합성과 보간에 쓰이는 회전행렬은 이 파일을 전혀 거치지 않는다.
 *
 * 로더는 2단계에서 고정해 둔 vendor/GLTFLoader.js 를 쓴다.
 * importmap 이 'three/addons/loaders/GLTFLoader.js' 를 거기로 잇고,
 * 'three' 는 external 로 남아 있어 앱과 같은 three 인스턴스를 공유한다.
 * CDN 에서 새로 받으면 버전이 갈려 조용히 깨진다.
 */

import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { setMatrix } from './scene.js';

/* ───────────────────────────── 상수 ───────────────────────────── */

export const MODEL_URL = 'assets/plane.glb';

/**
 * 모델 로컬 축 → 앱 규약 보정 행렬.
 *
 * 앱 규약 (월드가 Z-up 이므로):  +x 기수,  +z 위,  ±y 날개
 * plane.glb 로컬 축 (측정 완료): −X 기수,  +Y 위,  ±Z 날개
 *
 *   FIX·(−1, 0, 0) = ( 1, 0, 0)   기수 → +x
 *   FIX·( 0, 1, 0) = ( 0, 0, 1)   위   → +z
 *   FIX·( 0, 0, 1) = ( 0, 1, 0)   날개 → ±y
 *
 * 축 (0, 1, 1)/√2 둘레 180° 회전이고 det = +1 인 정상 회전이다.
 * 값을 다시 추정하지 마라.
 */
export const MODEL_AXIS_FIX = [
  [-1, 0, 0],
  [0, 0, 1],
  [0, 1, 0],
];

/** 문서화된 모델 범위. 런타임 측정값과 어긋나면 콘솔에 경고를 낸다. */
const MODEL_EXTENT = [1.735, 0.658, 1.9];

/**
 * 정규화 목표 — 최대 범위(날개폭)를 이 크기로 맞춘다.
 * 가장 안쪽 링(X 링)이 반지름 1.8, 튜브 0.05 라 안쪽 여유 반경이 약 1.75 다.
 * 날개폭 2.6 이면 날개 끝이 중심에서 1.3 쯤이라 어떤 자세에서도 링 안에 있다.
 */
const TARGET_SPAN = 2.6;

/** 링 안쪽 여유 반경 — 자체 점검용 */
const INNER_CLEARANCE = 1.75;

/* ───────────────────────────── 도우미 ───────────────────────────── */

/** 회전행렬에 균일 배율을 먹인다. Matrix4 상단 3×3 에 그대로 들어간다. */
const scaleMatrix = (R, s) => R.map((row) => row.map((v) => v * s));

/** (s·R)·v */
function applyScaled(R, s, v) {
  return [
    s * (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2]),
    s * (R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2]),
    s * (R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]),
  ];
}

/** 비교 뷰에서 두 기체를 구별하는 작은 색 구슬 (기체 아래) */
function accentBead(color) {
  const bead = new THREE.Mesh(
    new THREE.SphereGeometry(0.13, 20, 14),
    new THREE.MeshStandardMaterial({
      color,
      roughness: 0.3,
      metalness: 0.1,
      emissive: color,
      emissiveIntensity: 0.85,
    })
  );
  setMatrix(bead, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, -0.62]);
  return bead;
}

/* ───────────────────────────── 로딩 ───────────────────────────── */

/**
 * GLB 를 읽어 "인스턴스 공장" 을 만든다.
 *
 * 절대 reject 하지 않는다 — 실패해도 { ok: false } 로 돌려주고 호출부가
 * 삼각대 폴백으로 넘어가게 한다. 앱이 에셋 로딩에 묶여 멈추면 안 된다.
 *
 * @param {string} url
 * @returns {Promise<{ok:boolean, create?:Function, info?:object, error?:string}>}
 */
export function loadAircraft(url = MODEL_URL) {
  return new Promise((resolve) => {
    const loader = new GLTFLoader();
    const t0 = performance.now();

    loader.load(
      url,
      (gltf) => {
        try {
          const src = gltf.scene;
          src.updateMatrixWorld(true);

          // 실제 범위를 재서 정규화한다 (문서값은 대조만 한다)
          const box = new THREE.Box3().setFromObject(src);
          const size = new THREE.Vector3();
          const center = new THREE.Vector3();
          box.getSize(size);
          box.getCenter(center);

          const extent = [size.x, size.y, size.z];
          const maxExtent = Math.max(...extent);
          const scale = TARGET_SPAN / maxExtent;

          const drift = extent.map((v, i) => Math.abs(v - MODEL_EXTENT[i]));
          if (Math.max(...drift) > 0.01) {
            console.warn(
              '[RotLab] 모델 범위가 문서값과 다르다 — 측정',
              extent.map((v) => +v.toFixed(3)),
              '문서',
              MODEL_EXTENT
            );
          }

          // 보정 + 정규화 + 중심 맞춤. 전부 상수이고 한 번만 곱한다.
          const fixMatrix = scaleMatrix(MODEL_AXIS_FIX, scale);
          const offset = applyScaled(MODEL_AXIS_FIX, -scale, [center.x, center.y, center.z]);

          // 보정 후 실제 바운딩 구가 가장 안쪽 링에 들어가는지 자체 점검.
          // AABB 대각선으로 재면 비행기처럼 속이 빈 형상은 크게 과대평가된다
          // (이 모델은 대각선 1.82 vs 실제 1.38). 지오메트리 바운딩 구로 잰다.
          const _v = new THREE.Vector3();
          const _c = new THREE.Vector3();
          let localRadius = 0;
          let polys = 0;
          let meshCount = 0;
          let missingNormals = 0;

          src.traverse((o) => {
            if (!o.isMesh || !o.geometry) return;
            meshCount++;
            const idx = o.geometry.getIndex();
            polys += idx ? idx.count / 3 : o.geometry.attributes.position.count / 3;
            if (!o.geometry.attributes.normal) missingNormals++;

            o.geometry.computeBoundingSphere();
            const bs = o.geometry.boundingSphere;
            if (!bs) return;
            const e = o.matrixWorld.elements;
            const meshScale = Math.max(
              _v.set(e[0], e[1], e[2]).length(),
              _v.set(e[4], e[5], e[6]).length(),
              _v.set(e[8], e[9], e[10]).length()
            );
            _c.copy(bs.center).applyMatrix4(o.matrixWorld);
            localRadius = Math.max(localRadius, _c.distanceTo(center) + bs.radius * meshScale);
          });

          const radius = localRadius * scale;

          const info = {
            url,
            ms: Math.round(performance.now() - t0),
            extent: extent.map((v) => +v.toFixed(3)),
            maxExtent: +maxExtent.toFixed(3),
            scale: +scale.toFixed(4),
            spanAfter: +(maxExtent * scale).toFixed(3),
            boundRadius: +radius.toFixed(3),
            fitsInnerRing: radius <= INNER_CLEARANCE,
            polys,
            meshes: meshCount,
            missingNormals,
          };

          /**
           * 인스턴스 하나를 만든다.
           *
           *   group   ← 회전 수학이 쓰는 노드. 여기 R 이 그대로 들어간다.
           *     fixed ← MODEL_AXIS_FIX · scale · (중심 이동). 상수, 절대 안 변한다.
           *       gltf 사본
           *     bead  ← (비교 뷰에서만) 방식 식별색
           */
          const create = (accentColor = null) => {
            const group = new THREE.Group();
            const fixed = new THREE.Group();
            setMatrix(fixed, fixMatrix, offset);
            fixed.add(src.clone(true));
            group.add(fixed);
            if (accentColor !== null) group.add(accentBead(accentColor));
            return group;
          };

          console.log(
            '[RotLab] GLB 로드 성공 — ' + url + '  ' + info.ms + 'ms  ' +
            polys.toLocaleString() + ' 폴리곤  범위 ' + JSON.stringify(info.extent) +
            '  배율 ' + info.scale + ' → 날개폭 ' + info.spanAfter
          );
          if (!info.fitsInnerRing) {
            console.warn(
              '[RotLab] 보정 후 바운딩 구 반경 ' + info.boundRadius +
              ' 가 안쪽 링 여유 ' + INNER_CLEARANCE + ' 를 넘는다 — TARGET_SPAN 을 줄여라'
            );
          }
          if (missingNormals) {
            console.log(
              '[RotLab] 이 GLB 는 NORMAL 속성이 없다 (' + missingNormals + '개 메시). ' +
              'GLTFLoader 가 flatShading 으로 처리한다.'
            );
          }

          resolve({ ok: true, create, info });
        } catch (err) {
          console.error('[RotLab] GLB 해석 실패 — ' + url, err);
          resolve({ ok: false, error: String(err && err.message ? err.message : err) });
        }
      },
      undefined,
      (err) => {
        const msg = err && err.message ? err.message : '읽을 수 없음';
        console.error('[RotLab] GLB 로드 실패 — ' + url + ' : ' + msg, err);
        resolve({ ok: false, error: msg });
      }
    );
  });
}
