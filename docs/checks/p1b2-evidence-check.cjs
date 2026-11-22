// Run at repository root after the final P1b.2 batch, replay and resume probes.
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const assert = require('node:assert/strict');
const base = 'docs/p1b2-audit-logs';
const read = p => JSON.parse(fs.readFileSync(p, 'utf8').replace(/^\uFEFF/, ''));
const sha = b => crypto.createHash('sha256').update(b).digest('hex');
const sorted = v => Array.isArray(v) ? v.map(sorted) : v && typeof v === 'object'
  ? Object.fromEntries(Object.keys(v).sort().map(k => [k, sorted(v[k])])) : v;
const checksum = v => sha(Buffer.from(JSON.stringify(sorted(v))));
const certificate = read(base + '/verified-certificates/manifest.json');
for (const [name, hash] of Object.entries(certificate.verifier))
  assert.equal(sha(fs.readFileSync('test version/benchmark/' + name)), hash, 'stale source ' + name);
for (const [name, hash] of Object.entries(certificate.files))
  assert.equal(sha(fs.readFileSync(base + '/verified-certificates/' + name)), hash, name);
function auditBatch(name) {
  const directory = base + '/' + name;
  const manifest = read(directory + '/run_manifest.json');
  const records = fs.readFileSync(directory + '/journal.jsonl', 'utf8').trim().split('\n').map(JSON.parse);
  let head = '0'.repeat(64), rawEvents = 0;
  const attempts = new Map();
  records.forEach((record, i) => {
    const { checksum: actual, ...body } = record;
    assert.equal(record.seq, i + 1); assert.equal(record.previous, head);
    assert.equal(checksum(body), actual); head = actual;
    const key = record.episode_id + '/' + record.attempt_id;
    if (record.kind === 'START') { assert(!attempts.has(key)); attempts.set(key, record); }
    if (record.kind === 'FINISH') {
      const start = attempts.get(key); assert(start);
      for (const [file, hash] of Object.entries(record.hashes))
        assert.equal(sha(fs.readFileSync(path.join(directory, start.path, file))), hash);
      const events = fs.readFileSync(path.join(directory, start.path, 'events.jsonl'), 'utf8').trim().split('\n').map(JSON.parse);
      events.forEach((e, j) => { assert.equal(e.run_id, manifest.run_id); assert.equal(e.event_seq, j + 1); assert.equal(e.attempt_id, record.attempt_id); });
      assert(events.some(e => e.event_type === 'PLANNER_BARRIER'));
      rawEvents += events.length;
    }
  });
  assert.equal(attempts.size, 3); assert.equal(records.length, 6);
  const summary = read(directory + '/summary.json');
  assert.equal(summary.journal_head, head); assert.equal(summary.overall.completed, 3);
  const events = fs.readFileSync(directory + '/events.jsonl', 'utf8').trim().split('\n').map(JSON.parse);
  assert.equal(events.length, rawEvents);
  const keys = events.map(e => [e.episode_id, e.attempt_id, e.event_seq].join('/'));
  assert.equal(new Set(keys).size, keys.length);
  for (const file of ['attempts.csv', 'episodes.csv'])
    assert.equal(fs.readFileSync(directory + '/' + file, 'utf8').trim().split(/\r?\n/).length, 4);
  return { attempts: attempts.size, events: rawEvents, journal_head: head };
}
const batches = { original: auditBatch('verified-batch'), repeated: auditBatch('verified-repeat') };
const replay = read(base + '/replay.json');
for (const eid of ['sample-s2', 'sample-s3', 'sample-s4']) {
  assert.equal(replay[eid].status, 'PASS'); assert.equal(replay[eid].left_hash, replay[eid].right_hash);
  const cleanResult = r => Object.fromEntries(Object.entries(r).filter(([k]) => !['run_id', 'attempt_id', 'elapsed_wall_s', 'workers_reaped'].includes(k)));
  const cleanEvents = text => text.trim().split('\n').map(JSON.parse).map(e => Object.fromEntries(Object.entries(e).filter(([k]) => !['run_id', 'attempt_id'].includes(k))));
  const a = base + '/verified-batch/attempts/' + eid + '/attempt-0001/';
  const b = base + '/verified-repeat/attempts/' + eid + '/attempt-0001/';
  assert.deepEqual(cleanResult(read(a + 'result.json')), cleanResult(read(b + 'result.json')));
  assert.deepEqual(cleanEvents(fs.readFileSync(a + 'events.jsonl', 'utf8')), cleanEvents(fs.readFileSync(b + 'events.jsonl', 'utf8')));
  assert(fs.readFileSync(a + 'trajectory.csv').equals(fs.readFileSync(b + 'trajectory.csv')));
}
const kill = read(base + '/acceptance-verified/kill-evidence.json');
assert(kill.some(k => k.checkpoint === 'running'));
kill.forEach(k => { assert.notEqual(k.supervisor_exitcode, 0); assert(Object.values(k.workers_alive_after_kill).every(v => v === false)); });
const resumed = read(base + '/resume-check.json'); assert.equal(resumed.status, 'PASS'); assert.equal(resumed.journal_unchanged, true);
const result = { status: 'PASS', batches, replay_episodes: Object.keys(replay), kill_checkpoints: kill.length, resume: resumed };
fs.writeFileSync(base + '/independent-check.json', JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify(result));
