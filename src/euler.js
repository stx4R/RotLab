/**
 * src/euler.js — ZYX 오일러 각 (고정 순서)
 *
 *   φ (phi)   : x축 회전 (roll)
 *   θ (theta) : y축 회전 (pitch)
 *   ψ (psi)   : z축 회전 (yaw)
 *
 *   R = Rz(ψ) · Ry(θ) · Rx(φ)
 *
 * 즉 물체 좌표계의 벡터에 Rx 를 먼저, Rz 를 마지막에 적용한다.
 *
 * Three.js는 이 파일에 등장하지 않는다.
 */

import { multiply, fromColumns, applyToVec, determinant } from './mat3.js';

/**
 * 짐벌 특이점 판정 문턱값.
 * matrixToEuler 는 |cos θ| 가 이 값보다 작으면 분해가 유일하지 않다고 본다.
 * (cos θ ≈ 1e-6 이면 φ, ψ 각각의 정밀도는 이미 ~1e-10 rad 수준으로 무너진다.)
 */
export const GIMBAL_EPS = 1e-6;

/** x축 둘레 φ 회전 */
export function rotX(rad) {
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  return [
    [1, 0, 0],
    [0, c, -s],
    [0, s, c],
  ];
}

/** y축 둘레 θ 회전 */
export function rotY(rad) {
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  return [
    [c, 0, s],
    [0, 1, 0],
    [-s, 0, c],
  ];
}

/** z축 둘레 ψ 회전 */
export function rotZ(rad) {
  const c = Math.cos(rad);
  const s = Math.sin(rad);
  return [
    [c, -s, 0],
    [s, c, 0],
    [0, 0, 1],
  ];
}

/**
 * R = Rz(ψ) · Ry(θ) · Rx(φ)
 * @returns {number[][]} 3×3 회전행렬
 */
export function eulerToMatrix(phi, theta, psi) {
  return multiply(multiply(rotZ(psi), rotY(theta)), rotX(phi));
}

/**
 * eulerToMatrix 의 역변환.
 *
 * 전개하면
 *   R = [ cθcψ   sφsθcψ − cφsψ   cφsθcψ + sφsψ ]
 *       [ cθsψ   sφsθsψ + cφcψ   cφsθsψ − sφcψ ]
 *       [ −sθ    sφcθ            cφcθ          ]
 * 이므로 R[2][0] = −sin θ, R[2][1] = sinφ cosθ, R[2][2] = cosφ cosθ,
 *        R[1][0] = cosθ sinψ, R[0][0] = cosθ cosψ.
 *
 * cos θ 는 asin 대신 hypot(R[2][1], R[2][2]) 로 얻는다 —
 * θ 가 ±90° 근처일 때 asin 은 조건수가 나빠지지만 atan2 는 그렇지 않다.
 *
 * cos θ ≈ 0 (짐벌 락) 이면 φ 와 ψ 는 각각 결정되지 않고 φ∓ψ 조합만 남는다.
 * 이때는 φ = 0 으로 고정해 자유도를 ψ 로 몰고 degenerate: true 를 붙여 반환한다.
 *
 * @returns {{phi:number, theta:number, psi:number, degenerate:boolean}}
 *          θ ∈ [−π/2, π/2], φ·ψ ∈ (−π, π]
 */
export function matrixToEuler(R) {
  const cosTheta = Math.hypot(R[2][1], R[2][2]); // = |cos θ| ≥ 0

  if (cosTheta < GIMBAL_EPS) {
    // 짐벌 락. R[2][0] = −sin θ 의 부호로 θ = ±90° 를 고른다.
    const theta = R[2][0] <= 0 ? Math.PI / 2 : -Math.PI / 2;
    // θ = +90° :  R[0][1] = sin(φ−ψ), R[1][1] = cos(φ−ψ)
    // θ = −90° :  R[0][1] = −sin(φ+ψ), R[1][1] = cos(φ+ψ)
    // 어느 쪽이든 φ = 0 으로 두면 ψ = atan2(−R[0][1], R[1][1]).
    return {
      phi: 0,
      theta,
      psi: Math.atan2(-R[0][1], R[1][1]),
      degenerate: true,
    };
  }

  return {
    phi: Math.atan2(R[2][1], R[2][2]),
    theta: Math.atan2(-R[2][0], cosTheta),
    psi: Math.atan2(R[1][0], R[0][0]),
    degenerate: false,
  };
}

const E1 = [1, 0, 0];
const E2 = [0, 1, 0];
const E3 = [0, 0, 1];

/**
 * 각속도 사상 행렬 E.
 *
 *   ω = E · [φ̇, θ̇, ψ̇]ᵀ
 *
 * 각 오일러 각속도가 "그 각이 적용되는 시점의 좌표계" 축을 타고 들어오므로
 * 열은 각각 (Rz·Ry)e₁, (Rz)e₂, e₃ 가 된다.
 *
 *   E = [ cθcψ  −sψ  0 ]
 *       [ cθsψ   cψ  0 ]
 *       [ −sθ    0   1 ]
 */
export function angularVelocityMatrix(phi, theta, psi) {
  const Rz = rotZ(psi);
  const RzRy = multiply(Rz, rotY(theta));
  return fromColumns(applyToVec(RzRy, E1), applyToVec(Rz, E2), E3);
}

/**
 * 짐벌 척도 = det(E).
 *
 * 위 E 를 전개하면 det E = cos θ · (cos²ψ + sin²ψ) = cos θ.
 * θ → ±90° 에서 0 이 되고, 그 지점에서 E 가 특이행렬이 된다
 * — 즉 어떤 ω 는 어떤 (φ̇, θ̇, ψ̇) 로도 만들 수 없다. 이것이 짐벌 락이다.
 *
 * phi 는 결과에 영향을 주지 않지만(E 가 φ 에 의존하지 않는다) 시그니처는
 * 다른 함수들과 맞춰 둔다.
 */
export function gimbalMeasure(phi, theta, psi) {
  return determinant(angularVelocityMatrix(phi, theta, psi));
}
