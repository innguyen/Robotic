// Run from repository root: node experiments/webots_s2/evidence/run-05-final/verify-evidence.js
// Independent post-run checks; does not participate in policy execution.
const fs = require('fs'), path = require('path'), crypto = require('crypto'), cp = require('child_process');
const digest = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const base = process.argv[2] || 'experiments/webots_s2/evidence/run-07-final';
let errors = [], rawEvidenceWhitespaceLines = 0, controls = 0, links = 0, maxApplied = [0, 0, 0];
const summary = JSON.parse(fs.readFileSync(base + '/summary.json', 'utf8'));
for (const [name, pass] of Object.entries(summary)) {
  if (!pass) errors.push('Final scenario failed: ' + name);
  const dir = base + '/' + name, inv = JSON.parse(fs.readFileSync(dir + '/invocation.json', 'utf8'));
  if (inv.returncode !== 0 || !inv.passed) errors.push('Bad invocation ' + name);
  if (digest(dir + '/fixture.wbt') !== inv.fixture_sha256) errors.push('Fixture hash mismatch ' + name);
  for (const [file, hash] of Object.entries(inv.identity)) {
    if (digest(file) !== hash || inv.identity_after[file] !== hash) errors.push('Source binding changed: ' + file);
  }
  const runtime = path.dirname(path.dirname(inv.command.at(-1)));
  const copies = ['adapter.py', 'runtime.py', 'geometry.py',
    ...fs.readdirSync('experiments/webots_s2/controllers').map(n => 'controllers/' + n + '/' + n + '.py')];
  for (const f of copies) {
    if (digest(path.join(runtime, f)) !== digest(path.join('experiments/webots_s2', f))) errors.push('Runtime copy drift: ' + f);
  }
  const log = dir + '/control.jsonl';
  if (!fs.existsSync(log)) continue;
  const epochTimes = {};
  for (const line of fs.readFileSync(log, 'utf8').trim().split('\n')) {
    const r = JSON.parse(line);
    if (r.event !== 'CONTROL') continue;
    controls++;
    const bounds = [r.applied[0] >= 0 ? .18 : .06, .12, Math.PI / 6];
    r.applied.forEach((v, i) => {
      maxApplied[i] = Math.max(maxApplied[i], Math.abs(v));
      if (!Number.isFinite(v) || Math.abs(v) > bounds[i] + 1e-10) errors.push('Limit violation ' + name);
    });
    if (r.monitor_steps < 1) errors.push('Unmonitored command ' + name);
    if (JSON.stringify(r.checked) !== JSON.stringify(r.applied)) errors.push('Applied command differs from checked ' + name);
    if ((r.fault || r.done) && r.applied.some(v => v !== 0)) errors.push('Fault/terminal motion ' + name);
    if (r.target && !r.recognition_visible) errors.push('Target without recognition ' + name);
    const prev = epochTimes[r.epoch];
    if (prev !== undefined && r.sim_s <= prev) errors.push('Clock not monotonic within epoch ' + name);
    epochTimes[r.epoch] = r.sim_s;
  }
}
const candidates = [...cp.execFileSync('git', ['ls-files', '--others', '--exclude-standard'], {encoding: 'utf8'}).trim().split('\n'),
  ...cp.execFileSync('git', ['diff', '--name-only'], {encoding: 'utf8'}).trim().split('\n')].filter(Boolean);
let checkedTextFiles = 0;
for (const f of new Set(candidates)) {
  if (!/\.(py|js|md|json|jsonl|wbt|txt)$/.test(f)) continue;
  const content = fs.readFileSync(f, 'utf8');
  checkedTextFiles++;
  content.split(/\r?\n/).forEach((line, i) => {
    if (/[\t ]+$/.test(line)) {
      // Preserve raw historic fixtures byte-for-byte; their hashes are evidence.
      if (f.includes('/evidence/')) rawEvidenceWhitespaceLines++;
      else errors.push(f + ':' + (i + 1) + ' trailing whitespace');
    }
  });
  if (!f.endsWith('.md')) continue;
  for (const m of content.matchAll(/\]\(([^)]+)\)/g)) {
    if (/^(https?:|#)/.test(m[1])) continue;
    const target = decodeURIComponent(m[1].split('#')[0]);
    links++;
    if (!fs.existsSync(path.resolve(path.dirname(f), target))) errors.push('Missing link ' + f + ' -> ' + target);
  }
}
const report = {finalScenarios: summary, controlRecordsChecked: controls, maxAbsApplied: maxApplied,
  sourceAndRuntimeHashesMatch: !errors.some(e => /hash|binding|copy/.test(e)), checkedTextFiles,
  localLinksChecked: links, rawEvidenceWhitespaceLinesPreserved: rawEvidenceWhitespaceLines, errors};
fs.writeFileSync(base + '/verification.json', JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify(report, null, 2));
if (errors.length) process.exitCode = 1;
