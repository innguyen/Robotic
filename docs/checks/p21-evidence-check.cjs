// Independent byte/hash/journal checks for the P2.1 dev control archive.
// Run from the repository root after p21-dev-controls.py has finished.
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const assert = require('node:assert/strict');
const base = process.argv[2] || 'docs/p21-audit-logs/dev-final';
const read = p => JSON.parse(fs.readFileSync(p, 'utf8').replace(/^\uFEFF/, ''));
const sha = b => crypto.createHash('sha256').update(b).digest('hex');
const sorted = v => Array.isArray(v) ? v.map(sorted) : v && typeof v === 'object'
  ? Object.fromEntries(Object.keys(v).sort().map(k => [k, sorted(v[k])])) : v;
const hashObject = v => sha(Buffer.from(JSON.stringify(sorted(v))));
const report = read(path.join(base, 'controls.json'));
assert.equal(report.protocol, 't1-0.2');
assert.deepEqual(report.production_diff, []);
assert(report.certificates.every(c => c.status === 'PASS'));
const cert = read(path.join(base, 'certificates/manifest.json'));
for (const [name, hash] of Object.entries(cert.verifier))
  assert.equal(sha(fs.readFileSync('test version/benchmark/' + name)), hash, name);
for (const [name, hash] of Object.entries(cert.files))
  assert.equal(sha(fs.readFileSync(path.join(base, 'certificates', name))), hash, name);
const policies = {};
for (const [id, policy] of Object.entries(report.policies)) {
  assert.equal(policy.controls.length, 5);
  assert.equal(new Set(policy.controls.map(c => c.scenario)).size, 5);
  assert.equal(policy.resume_files_unchanged, true);
  const repetitions = [];
  assert.equal(policy.runs.length, 2);
  for (const run of policy.runs) {
    const directory = path.join(base, id, run);
    const manifest = read(path.join(directory, 'run_manifest.json'));
    const records = fs.readFileSync(path.join(directory, 'journal.jsonl'), 'utf8').trim().split('\n').map(JSON.parse);
    let head = '0'.repeat(64);
    const starts = new Map(), completed = new Set();
    records.forEach((record, i) => {
      const { checksum, ...body } = record;
      assert.equal(record.seq, i + 1); assert.equal(record.previous, head);
      assert.equal(hashObject(body), checksum); head = checksum;
      assert.equal(record.attempt_id, 1);
      const key = record.episode_id;
      if (record.kind === 'START') { assert(!starts.has(key)); starts.set(key, record); }
      if (record.kind === 'FINISH') {
        assert(!completed.has(key)); completed.add(key);
        const start = starts.get(key); assert(start);
        const folder = path.join(directory, start.path);
        for (const [name, hash] of Object.entries(record.hashes))
          assert.equal(sha(fs.readFileSync(path.join(folder, name))), hash, name);
        const attempt = read(path.join(folder, 'run_manifest.json'));
        assert.equal(attempt.split, 'dev'); assert.equal(attempt.policy_id, id);
        assert.equal(attempt.policy_identity.version, '2');
        const fixture = read(path.join(base, 'certificates', key, 'resolved_episode.json')).episode;
        const certificate = read(path.join(base, 'certificates', key, 'fixture_validation.json'));
        assert.equal(attempt.fixture_hash, certificate.fixture_hash);
        assert.deepEqual(attempt.policy_identity, manifest.identity.policy_identities[key]);
        for (const [name, hash] of Object.entries(attempt.policy_identity.sources))
          assert.equal(sha(fs.readFileSync(path.join('test version', name))), hash, name);
        const result = read(path.join(folder, 'result.json'));
        assert.equal(result.workers_reaped, true);
        const events = fs.readFileSync(path.join(folder, 'events.jsonl'), 'utf8').trim().split('\n').map(JSON.parse);
        assert.equal(events.at(-1).event_type, 'EPISODE_END');
        assert.equal(new Set(events.map(e => e.event_seq)).size, events.length);
        const selected = events.filter(e => e.event_type === 'SELECTOR_DECISION' && e.payload.selection !== null).map(e => e.payload.selection);
        assert.equal(result.goals_selected, selected.length);
        assert.equal(result.fallback_goals, selected.filter(s => s.kind === 'fallback').length);
        const fixed = selected.filter(s => s.kind === 'fixed').map(s => s.waypoint_index);
        assert.equal(new Set(fixed).size, fixed.length);
        assert(events.filter(e => ['SELECTOR_DECISION','SELECTOR_STATUS','WAYPOINT_OUTCOME'].includes(e.event_type)).every(e => e.source === 'policy'));
        const csv = fs.readFileSync(path.join(folder, 'trajectory.csv'), 'utf8').trim().split(/\r?\n/).map(line => line.split(','));
        const columns = csv.shift();
        const points = [[fixture.start_pose.x_m, fixture.start_pose.y_m], ...csv.map(row => [Number(row[columns.indexOf('x_m')]), Number(row[columns.indexOf('y_m')])])];
        if (['S2','S3','S4'].includes(fixture.scenario_id)) {
          assert(result.goals_selected > 0);
          assert(points.some(p => Math.hypot(p[0]-points[0][0], p[1]-points[0][1]) > .1));
        }
        if (fixture.scenario_id === 'S3') {
          const crosses = ([[ax,ay],[bx,by]], [[cx,cy],[dx,dy]]) => {
            const cross = (ux,uy,vx,vy) => ux*vy-uy*vx;
            const den = cross(bx-ax,by-ay,dx-cx,dy-cy);
            if (den === 0) return false;
            const t = cross(cx-ax,cy-ay,dx-cx,dy-cy)/den;
            const u = cross(cx-ax,cy-ay,bx-ax,by-ay)/den;
            return t >= 0 && t <= 1 && u >= 0 && u <= 1;
          };
          assert(fixture.scenario_evidence.gates.some(g => points.slice(1).some((p, i) => crosses([points[i],p],g.segment_m))));
        }
        if (fixture.scenario_id === 'S5') {
          assert.equal(result.end_reason, 'TIMEOUT'); assert.equal(result.report, null);
          assert.equal(result.elapsed_sim_s, fixture.time_limit_s);
          assert.equal(events.filter(e => e.event_type === 'SENSE').length, Math.round(fixture.time_limit_s/.1));
          assert(!events.some(e => e.payload.end_reason === 'SEARCH_EXHAUSTED'));
        }
      }
    });
    assert.equal(starts.size, 5); assert.equal(completed.size, 5); assert.equal(records.length, 10);
    assert.equal(read(path.join(directory, 'summary.json')).journal_head, head);
    repetitions.push({ run, attempts: starts.size, journal_head: head });
  }
  for (const c of policy.controls) {
    assert.equal(c.classification, 'control_pass', c.episode_id);
    const replay = policy.replay[c.episode_id];
    assert.equal(replay.status, 'PASS'); assert.equal(replay.left_hash, replay.right_hash);
    const folder = repeat => path.join(base, id, policy.runs[repeat-1], 'attempts', c.episode_id, 'attempt-0001');
    const cleanResult = r => Object.fromEntries(Object.entries(r).filter(([k]) => !['run_id','attempt_id','elapsed_wall_s','workers_reaped'].includes(k)));
    const cleanEvents = p => fs.readFileSync(path.join(p, 'events.jsonl'), 'utf8').trim().split('\n').map(JSON.parse)
      .map(e => Object.fromEntries(Object.entries(e).filter(([k]) => !['run_id','attempt_id'].includes(k))));
    assert.deepEqual(cleanResult(read(path.join(folder(1), 'result.json'))), cleanResult(read(path.join(folder(2), 'result.json'))));
    assert.deepEqual(cleanEvents(folder(1)), cleanEvents(folder(2)));
    assert(fs.readFileSync(path.join(folder(1), 'trajectory.csv')).equals(fs.readFileSync(path.join(folder(2), 'trajectory.csv'))));
  }
  policies[id] = { repetitions, replay_episodes: Object.keys(policy.replay), protected_resume_files: policy.protected_files };
}
const result = { status: 'PASS', purpose: 'policy_dev_control_not_research', policies };
fs.writeFileSync(path.join(base, '../independent-check.json'), JSON.stringify(result, null, 2) + '\n');
console.log(JSON.stringify(result));
