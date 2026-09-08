/**
 * src/quat.js — 쿼터니언
 *
 * 표현: { w, x, y, z }.  q = w + xi + yj + zk,  i² = j² = k² = ijk = −1.
 * 모든 함수는 순수 함수다.
 *
 * 회전을 뜻하는 함수(toMatrix, rotateVector, toAxisAngle, slerp)는 단위
 * 쿼터니언을 전제로 한다. 확신이 없으면 normalize() 를 먼저 통과시켜라.
 *
 * Three.js는 이 파일에 등장하지 않는다.
 */

const EPS = 1e-12;

function clamp(v, lo, hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

/** 해밀턴 곱  ab.  (회전 합성은 "b를 적용한 뒤 a" 순서다) */
export function multiply(a, b) {
  return {
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
  };
}

/** 켤레 q* = w − xi − yj − zk */
export function conjugate(q) {
  return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
}

/** 내적 (4차원 벡터로 본 점곱) */
export function dot(a, b) {
  return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

/** 노름 |q| = √(w²+x²+y²+z²) */
export function norm(q) {
  return Math.hypot(q.w, q.x, q.y, q.z);
}

/** 단위화 */
export function normalize(q) {
  const n = norm(q);
  if (n < EPS) throw new Error('normalize: 영 쿼터니언은 단위화할 수 없다');
  return { w: q.w / n, x: q.x / n, y: q.y / n, z: q.z / n };
}

/**
 * 축-각 → 쿼터니언.  q = cos(α/2) + sin(α/2)·n̂
 * @param {number[]} axis 회전축 (단위벡터가 아니어도 됨 — 내부에서 정규화)
 * @param {number} rad 회전각 α
 */
export function fromAxisAngle(axis, rad) {
  const len = Math.hypot(axis[0], axis[1], axis[2]);
  if (len < EPS) return { w: 1, x: 0, y: 0, z: 0 }; // 축이 없으면 항등
  const half = rad / 2;
  const s = Math.sin(half) / len;
  return { w: Math.cos(half), x: axis[0] * s, y: axis[1] * s, z: axis[2] * s };
}

/**
 * 쿼터니언 → 축-각.
 * α = 2·atan2(|u|, w) 로 얻는다 — acos(w) 보다 α≈0, α≈2π 양쪽에서 안정적이다.
 * @returns {{axis:number[], angle:number}} angle ∈ [0, 2π), 항등이면 축은 [1,0,0]
 */
export function toAxisAngle(q) {
  const u = Math.hypot(q.x, q.y, q.z);
  if (u < EPS) return { axis: [1, 0, 0], angle: 0 };
  const angle = 2 * Math.atan2(u, q.w);
  return { axis: [q.x / u, q.y / u, q.z / u], angle };
}

/**
 * 쿼터니언 → 회전행렬 (단위 쿼터니언 전제)
 *
 *   R = [ 1−2(y²+z²)   2(xy−wz)    2(xz+wy)  ]
 *       [ 2(xy+wz)     1−2(x²+z²)  2(yz−wx)  ]
 *       [ 2(xz−wy)     2(yz+wx)    1−2(x²+y²)]
 */
export function toMatrix(q) {
  const { w, x, y, z } = q;
  const xx = x * x, yy = y * y, zz = z * z;
  const xy = x * y, xz = x * z, yz = y * z;
  const wx = w * x, wy = w * y, wz = w * z;
  return [
    [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
    [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
    [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
  ];
}

/**
 * 회전행렬 → 쿼터니언 (Shepperd 방법).
 *
 * 네 성분 중 절대값이 가장 큰 것을 먼저 √ 로 뽑고 나머지를 나눠 구한다.
 * 그냥 w 부터 뽑으면 180° 근처 회전에서 √(1+tr) → 0 이 되어 정밀도가 무너진다.
 *
 * q 와 −q 는 같은 회전이므로(이중 덮개) 이 함수는 부호를 하나 골라 돌려준다.
 */
export function fromMatrix(R) {
  const m00 = R[0][0], m01 = R[0][1], m02 = R[0][2];
  const m10 = R[1][0], m11 = R[1][1], m12 = R[1][2];
  const m20 = R[2][0], m21 = R[2][1], m22 = R[2][2];
  const trace = m00 + m11 + m22;

  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2; // s = 4w
    return { w: 0.25 * s, x: (m21 - m12) / s, y: (m02 - m20) / s, z: (m10 - m01) / s };
  }
  if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2; // s = 4x
    return { w: (m21 - m12) / s, x: 0.25 * s, y: (m01 + m10) / s, z: (m02 + m20) / s };
  }
  if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2; // s = 4y
    return { w: (m02 - m20) / s, x: (m01 + m10) / s, y: 0.25 * s, z: (m12 + m21) / s };
  }
  const s = Math.sqrt(1 + m22 - m00 - m11) * 2; // s = 4z
  return { w: (m10 - m01) / s, x: (m02 + m20) / s, y: (m12 + m21) / s, z: 0.25 * s };
}

/**
 * 벡터 회전 v' = q v q*  (v 는 순허수 쿼터니언 (0, v) 로 본다).
 *
 * 곱을 두 번 하지 않고 전개한 항등식을 쓴다:
 *   q v q* = (w² − u·u) v + 2(u·v) u + 2w (u × v),   u = (x, y, z)
 * 단위 쿼터니언이면 w² − u·u = 1 − 2|u|² 이고 로드리게스 공식과 일치한다.
 */
export function rotateVector(q, v) {
  const { w, x, y, z } = q;
  const uu = x * x + y * y + z * z;
  const uv = x * v[0] + y * v[1] + z * v[2];
  const a = w * w - uu;   // (w² − u·u)
  const b = 2 * uv;       // 2(u·v)
  const c = 2 * w;        // 2w
  // u × v
  const cx = y * v[2] - z * v[1];
  const cy = z * v[0] - x * v[2];
  const cz = x * v[1] - y * v[0];
  return [
    a * v[0] + b * x + c * cx,
    a * v[1] + b * y + c * cy,
    a * v[2] + b * z + c * cz,
  ];
}

/**
 * slerp 가 참 구면보간 대신 선형보간+정규화로 넘어가는 문턱.
 *
 * dot 이 이 값을 넘으면 두 자세가 거의 같아서 sin Ω 로 나누는 게 불안정해진다.
 * 0.9995 (Ω ≈ 1.81°) 는 Shoemake 이래의 관례값이고 three.js·GLM·Unity 가
 * 모두 같은 값을 쓴다. 이 구간에서 nlerp 와 참 slerp 의 각도 차이는
 * 최대 6e-5° 라 시각적으로도 수치적으로도 무해하다.
 */
export const SLERP_LINEAR_CUTOFF = 0.9995;

/**
 * 구면 선형 보간.
 *
 *   Ω = arccos(q0·q1)
 *   slerp = ( sin((1−t)Ω)·q0 + sin(tΩ)·q1 ) / sin Ω
 *
 * 두 가지를 더 처리한다.
 *
 * 1) 이중 덮개(최단호).  q 와 −q 는 같은 회전이지만 S³ 위에서는 정반대 점이다.
 *    dot < 0 이면 q1 이 q0 의 반대쪽 반구에 있다는 뜻이고, 그대로 위 식을
 *    적용하면 Ω > 90° 가 되어 회전각 180° 를 넘는 "먼 쪽 호" 를 돈다.
 *    q1 의 부호를 뒤집어 짧은 쪽 호를 고른다. 끝점은 같은 회전이므로
 *    보간 결과의 의미는 바뀌지 않고, 지나가는 경로만 짧은 쪽이 된다.
 *
 * 2) Ω → 0.  sin Ω 로 나누는 게 불안정해지므로 선형 보간 후 정규화로
 *    대체한다 (SLERP_LINEAR_CUTOFF 참고).
 */
export function slerp(q0, q1, t) {
  let d = dot(q0, q1);
  let w1 = q1.w, x1 = q1.x, y1 = q1.y, z1 = q1.z;

  if (d < 0) {
    // 최단호를 타도록 q1 → −q1
    d = -d;
    w1 = -w1; x1 = -x1; y1 = -y1; z1 = -z1;
  }

  const s = 1 - t;

  if (d > SLERP_LINEAR_CUTOFF) {
    // 두 자세가 거의 같다 → 선형 보간 후 정규화
    return normalize({
      w: s * q0.w + t * w1,
      x: s * q0.x + t * x1,
      y: s * q0.y + t * y1,
      z: s * q0.z + t * z1,
    });
  }

  const omega = Math.acos(clamp(d, -1, 1));
  const sinOmega = Math.sin(omega);
  const a = Math.sin(s * omega) / sinOmega;
  const b = Math.sin(t * omega) / sinOmega;

  return {
    w: a * q0.w + b * w1,
    x: a * q0.x + b * x1,
    y: a * q0.y + b * y1,
    z: a * q0.z + b * z1,
  };
}
