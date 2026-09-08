// Node 러너:  node tests/run.node.mjs [seed]
// 브라우저용은 tests/run.html
import { runAll, printReport } from '../src/verify.js';

const seedArg = process.argv[2];
const report = runAll(seedArg ? { seed: Number(seedArg) >>> 0 } : {});
process.exit(printReport(report) ? 0 : 1);
