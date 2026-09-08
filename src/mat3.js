/**
 * src/mat3.js — 3×3 행렬 유틸리티
 *
 * 표현: number[3][3], 행 우선(row-major). A[i][j] = i행 j열.
 * 모든 함수는 순수 함수다. 인자를 변형하지 않고 항상 새 배열을 만든다.
 *
 * Three.js는 이 파일에 등장하지 않는다.
 */

/** 3×3 단위행렬 */
export function identity() {
  return [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
}

/**
 * 열벡터 3개로 행렬을 만든다.
 * 결과 M 의 j번째 열이 c_{j+1} 이 된다.  M[i][j] = c_{j+1}[i]
 * @param {number[]} c1 첫째 열 (길이 3)
 * @param {number[]} c2 둘째 열
 * @param {number[]} c3 셋째 열
 */
export function fromColumns(c1, c2, c3) {
  return [
    [c1[0], c2[0], c3[0]],
    [c1[1], c2[1], c3[1]],
    [c1[2], c2[2], c3[2]],
  ];
}

/** 행렬 곱 (AB)[i][j] = Σ_k A[i][k] B[k][j] */
export function multiply(A, B) {
  const C = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
  ];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      let s = 0;
      for (let k = 0; k < 3; k++) s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  }
  return C;
}

/** 전치 Aᵀ[i][j] = A[j][i] */
export function transpose(A) {
  return [
    [A[0][0], A[1][0], A[2][0]],
    [A[0][1], A[1][1], A[2][1]],
    [A[0][2], A[1][2], A[2][2]],
  ];
}

/** 행렬식 (첫 행에 대한 여인수 전개) */
export function determinant(A) {
  return (
    A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
    A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
    A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])
  );
}

/** 열벡터에 적용: (Av)[i] = Σ_j A[i][j] v[j] */
export function applyToVec(A, v) {
  return [
    A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
    A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
    A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
  ];
}
