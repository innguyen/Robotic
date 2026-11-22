// Run from repository root after producing the P1b.1 audit artifacts.
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const assert = require('node:assert/strict');
const root = 'docs/p1b1-audit-logs';
const read = p => JSON.parse(fs.readFileSync(p, 'utf8').replace(/^\uFEFF/, ''));
const sha = bytes => crypto.createHash('sha256').update(bytes).digest('hex');
const bundle = root + '/verified-bundle';
const manifest = read(bundle + '/manifest.json');
for (const [name, hash] of Object.entries(manifest.verifier)) {
  assert.equal(sha(fs.readFileSync('test version/benchmark/' + name)), hash, 'stale source ' + name);
}
for (const [name, hash] of Object.entries(manifest.files)) {
  assert.equal(sha(fs.readFileSync(bundle + '/' + name)), hash, 'artifact mismatch ' + name);
}
const verifierHash = sha(Buffer.from(JSON.stringify(manifest.verifier)));
function auditRun(base) {
  const result = read(base + '/result.json'), metadata = read(base + '/run_manifest.json');
  const events = fs.readFileSync(base + '/events.jsonl', 'utf8').trim().split('\n').map(JSON.parse);
  events.forEach((e, i) => {
    assert.equal(e.event_seq, i + 1);
    assert.equal(e.episode_id, result.episode_id);
    if (i) assert(e.sim_time_s >= events[i - 1].sim_time_s, 'time regression');
  });
  assert.equal(events.at(-1).payload.end_reason, result.end_reason);
  assert.equal(result.workers_reaped, true);
  assert.equal(result.mission_success, result.end_reason === 'SUCCESS_REPORT');
  assert.equal(metadata.verifier_hash, verifierHash);
  assert(metadata.dependencies.numpy && metadata.dependencies.PyYAML);
  return { result, metadata, events };
}
const cases = fs.readdirSync(root + '/acceptance-current').sort().map(name => {
  const { result: r, metadata: m, events } = auditRun(root + '/acceptance-current/' + name);
  return { run: name, episode_id: r.episode_id, policy_id: m.policy_id, test_fault: m.test_fault,
    end_reason: r.end_reason, report_correct: r.report_correct, mission_success: r.mission_success,
    workers_reaped: r.workers_reaped, events: events.length };
});
const demo = root + '/verified-episode';
const { result, events } = auditRun(demo);
assert.equal(result.end_reason, 'SUCCESS_REPORT');
assert(events.some(e => e.event_type === 'PLANNER_BARRIER'));
const resolved = read(bundle + '/' + result.episode_id + '/resolved_episode.json');
let pose = ['x_m', 'y_m', 'yaw_rad'].map(k => resolved.episode.start_pose[k]);
const dt = resolved.shared.dt_s;
const lines = fs.readFileSync(demo + '/trajectory.csv', 'utf8').trim().split(/\r?\n/);
const keys = lines.shift().split(',');
let moving = 0, length = 0;
for (const line of lines) {
  const row = Object.fromEntries(line.split(',').map((v, i) => [keys[i], v]));
  const vx = +row.vx_executed, vy = +row.vy_executed, w = +row.wz_executed;
  const next = [pose[0] + (Math.cos(pose[2]) * vx - Math.sin(pose[2]) * vy) * dt,
    pose[1] + (Math.sin(pose[2]) * vx + Math.cos(pose[2]) * vy) * dt,
    ((pose[2] + w * dt + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI];
  const actual = [+row.x_m, +row.y_m, +row.yaw_rad];
  next.forEach((v, i) => assert(Math.abs(v - actual[i]) < 1e-10, 'Euler mismatch'));
  length += Math.hypot(actual[0] - pose[0], actual[1] - pose[1]);
  pose = actual;
  if (Math.hypot(vx, vy) > 0) moving++;
}
assert(Math.abs(length - result.path_length_m) < 1e-10);
assert.equal(lines.length * dt, result.elapsed_sim_s);
const summary = { status: 'PASS', acceptance_runs: cases.length,
  real_episode: { end_reason: result.end_reason, intervals: lines.length, moving_intervals: moving,
    planner_barriers: events.filter(e => e.event_type === 'PLANNER_BARRIER').length }, cases };
fs.writeFileSync(root + '/independent-check.json', JSON.stringify(summary, null, 2) + '\n');
console.log(JSON.stringify({ status: summary.status, acceptance_runs: cases.length, real_episode: summary.real_episode }));
