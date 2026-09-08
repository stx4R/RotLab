/**
 * tools/vendor.mjs — node_modules/three → vendor/ 단일 파일 번들
 *
 *   npm install && npm run vendor
 *
 * 왜 번들인가:
 *   three r150 이후 build/three.module.js 는 자기 완결적이지 않고
 *   ./three.core.js 를 재수출하는 껍데기다. GLTFLoader 도 마찬가지로
 *   ../utils/BufferGeometryUtils.js 와 ../utils/SkeletonUtils.js 를 끌어온다.
 *   그래서 "파일 하나 복사" 로는 둘 다 깨진다. 의존을 인라인해서 진짜
 *   단일 파일로 만든다.
 *
 * 왜 GLTFLoader 에서 three 는 external 인가:
 *   번들 안에 three 를 복제하면 앱이 쓰는 three 와 로더가 쓰는 three 가
 *   서로 다른 모듈 인스턴스가 된다. instanceof 와 상수 비교가 조용히
 *   깨지므로 'three' 는 import 로 남기고 importmap 이 잇게 한다.
 *
 * vendor/ 산출물은 레포에 커밋한다. node_modules/ 는 커밋하지 않는다.
 */

import { build } from 'esbuild';
import { readFileSync } from 'node:fs';

const pkg = JSON.parse(readFileSync('node_modules/three/package.json', 'utf8'));
const VERSION = pkg.version;                       // 예: 0.185.1
const REVISION = VERSION.split('.')[1];            // 예: 185

const header = (title, lines) =>
  [
    '/**',
    ' * ' + title,
    ' *',
    ...lines.map((l) => ' * ' + l),
    ' *',
    ' * 손으로 고치지 마라. 버전을 올리려면 package.json 의 three 를 바꾸고',
    ' * npm install && npm run vendor 를 다시 돌려라.',
    ' *',
    ' * 라이선스: MIT — Copyright (c) 2010-2025 three.js authors',
    ' */',
  ].join('\n');

const jobs = [
  {
    entryPoints: ['node_modules/three/build/three.module.js'],
    outfile: 'vendor/three.module.js',
    external: [],
    banner: header('three.js r' + REVISION + '  (npm three@' + VERSION + ')  — ESM 단일 파일 번들', [
      '원본: node_modules/three/build/three.module.js',
      '      + ./three.core.js (인라인)',
      '생성: npm run vendor',
    ]),
  },
  {
    entryPoints: ['node_modules/three/examples/jsm/loaders/GLTFLoader.js'],
    outfile: 'vendor/GLTFLoader.js',
    external: ['three'],
    banner: header('GLTFLoader — three.js r' + REVISION + '  (npm three@' + VERSION + ') 동봉본', [
      '원본: node_modules/three/examples/jsm/loaders/GLTFLoader.js',
      '      + ../utils/BufferGeometryUtils.js (인라인)',
      '      + ../utils/SkeletonUtils.js (인라인)',
      "'three' 는 external 로 남겼다 — importmap 이 vendor/three.module.js 로 잇는다.",
      'vendor/three.module.js 와 버전이 반드시 같아야 한다.',
      '생성: npm run vendor',
    ]),
  },
];

for (const job of jobs) {
  await build({
    entryPoints: job.entryPoints,
    outfile: job.outfile,
    bundle: true,
    format: 'esm',
    target: 'es2020',
    external: job.external,
    banner: { js: job.banner },
    legalComments: 'none',
    logLevel: 'warning',
  });
  console.log('  ->', job.outfile);
}

console.log('vendor/ 갱신 완료 — three@' + VERSION + ' (r' + REVISION + ')');
