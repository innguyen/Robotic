// Independent archive/CSV/path/monitor/primary checks. Run from repository root.
const fs = require('node:fs');
const path = require('node:path');
const crypto = require('node:crypto');
const assert = require('node:assert/strict');
const base = process.argv[2] || 'docs/p22-audit-logs/pilot-serial';
const root = base + '/paired';
const read = p => JSON.parse(fs.readFileSync(p, 'utf8').replace(/^\uFEFF/, ''));
const sha = b => crypto.createHash('sha256').update(b).digest('hex');
const sorted = v => Array.isArray(v) ? v.map(sorted) : v && typeof v === 'object'
  ? Object.fromEntries(Object.keys(v).sort().map(k => [k, sorted(v[k])])) : v;
const checksum = v => sha(Buffer.from(JSON.stringify(sorted(v))));
const approx = (a,b) => assert(Math.abs(a-b) < 1e-8, `${a} != ${b}`); // log consistency, not grading
function csv(file) {
  const text = fs.readFileSync(file, 'utf8'), rows = [];
  let row = [], field = '', quoted = false;
  for (let i = 0; i < text.length; i++) {
    const c = text[i];
    if (c === '"') { if (quoted && text[i+1] === '"') { field += '"'; i++; } else quoted = !quoted; }
    else if (!quoted && (c === ',' || c === '\n')) {
      row.push(field); field = ''; if (c === '\n') { rows.push(row); row = []; }
    } else if (c !== '\r' || quoted) field += c;
  }
  assert(!quoted); assert.equal(field, ''); assert.equal(row.length, 0);
  const header = rows.shift();
  return rows.map(values => { assert.equal(values.length, header.length); return Object.fromEntries(header.map((h,i) => [h, values[i]])); });
}
const pairRaw = fs.readFileSync(root + '/paired_manifest.json');
assert.equal(sha(pairRaw), fs.readFileSync(root + '/paired_manifest.sha256', 'utf8').trim());
const spec = JSON.parse(pairRaw), report = read(root + '/analysis/audit.json');
assert(['PASS','FAIL'].includes(report.status));
assert.equal(report.pair_manifest_hash, sha(pairRaw));
assert.equal(spec.protocol, 't1-0.2'); assert.equal(spec.purpose, 'dev_pilot_not_research');
assert.equal(spec.schema_version,'t1-paired-2');
assert.equal(spec.execution_profile.version,'single-job-v1');assert.equal(spec.execution_profile.max_active_jobs,1);
assert.equal(spec.metrics.version, 't1-metrics-1'); assert.equal(spec.metrics.monitor_version, 'monitor-episodes-1');
for (const [name, hash] of Object.entries(spec.metrics.sources)) assert.equal(sha(fs.readFileSync('test version/benchmark/' + name)), hash);
const bundle = spec.bundle, certificates = read(bundle + '/manifest.json');
for (const [name, hash] of Object.entries(certificates.verifier)) assert.equal(sha(fs.readFileSync('test version/benchmark/' + name)), hash);
for (const [name, hash] of Object.entries(certificates.files)) assert.equal(sha(fs.readFileSync(bundle + '/' + name)), hash);
const byKey = new Map(), seen = new Set(), diagnostics = [], strictReplays = [];
let rawAttempts = 0, continuousTotal = 0, legacyTotal = 0, stalledEpisodes = 0;
const ordinary = new Set(['PASS','IDLE','LIMIT']);
for (const reduced of report.reductions) {
  assert.equal(reduced.status, 'PASS'); assert.deepEqual(reduced.issues, []);
  const m = reduced.metrics, folder = reduced.attempt_path, result = read(folder + '/result.json');
  const manifest = read(folder + '/run_manifest.json'), batch = read(reduced.batch + '/run_manifest.json');
  const resolved = read(bundle + '/' + m.episode_id + '/resolved_episode.json');
  assert.equal(resolved.episode.split, 'dev');
  assert.equal(m.fixture_hash, sha(fs.readFileSync(bundle + '/' + m.episode_id + '/resolved_episode.json')));
  assert.deepEqual(batch.identity.fixtures, spec.corpus_bindings); assert.deepEqual(batch.identity.runtime, spec.runtime);
  assert.deepEqual(manifest.policy_identity, spec.policies[m.policy_id][m.episode_id]);
  assert.deepEqual(manifest.config, resolved.config);
  for (const [key,value] of Object.entries(spec.sensing)) assert.deepEqual(manifest[key], value);
  const key = [reduced.replica,m.policy_id,m.episode_id,m.attempt_id].join('/'); assert(!seen.has(key)); seen.add(key); byKey.set(key,reduced);
  const events = fs.readFileSync(folder + '/events.jsonl', 'utf8').trim().split('\n').filter(Boolean).map(JSON.parse);
  const trajectory = csv(folder + '/trajectory.csv'); let x = resolved.episode.start_pose.x_m, y = resolved.episode.start_pose.y_m, length = 0;
  let prev = null, continuous = 0, activeSteps = 0, activeChanges = 0, legacy = 0, previousEventState = null;
  const durations = {};
  trajectory.forEach((r,i) => {
    assert.equal(Number(r.step_index), i); approx(Number(r.sim_time_s), (i+1)*.1);
    length += Math.hypot(Number(r.x_m)-x, Number(r.y_m)-y); x = Number(r.x_m); y = Number(r.y_m);
    const state = r.monitor_state; durations[state] = (durations[state] || 0) + 1;
    if (!ordinary.has(state)) {
      activeSteps++; if (prev === null || ordinary.has(prev)) continuous++;
      else if (state !== prev) activeChanges++;
    }
    prev = state;
  });
  events.forEach((e,i) => {
    assert.equal(e.event_seq, i+1); assert.equal(e.episode_id, m.episode_id); assert.equal(e.attempt_id, m.attempt_id);
    if (e.event_type === 'MONITOR_STATE') {
      const state = e.payload.state; if (state !== previousEventState && !ordinary.has(state)) legacy++; previousEventState = state;
    }
  });
  assert.equal(events.filter(e => e.event_type === 'MONITOR_INTERVENTION').length, legacy);
  assert.equal(reduced.monitor_observed.monitor_interventions, legacy);
  assert.equal(reduced.monitor_observed.monitor_continuous_episodes, continuous);
  assert.equal(reduced.monitor_observed.monitor_intervention_state_changes, activeChanges);
  approx(reduced.monitor_observed.monitor_intervention_s, activeSteps*.1);
  if (m.trajectory_complete) {
    approx(m.path_length_m, length); assert.equal(m.monitor_interventions, legacy); assert.equal(m.monitor_continuous_episodes, continuous);
    approx(m.monitor_intervention_s, activeSteps*.1);
    for (const [state,steps] of Object.entries(durations)) approx(m.monitor_state_seconds[state], steps*.1);
  } else { assert.equal(m.path_length_m, null); assert.equal(m.monitor_continuous_episodes, null); }
  const selected = events.filter(e => e.event_type === 'SELECTOR_DECISION' && e.payload.selection !== null).map(e => e.payload.selection);
  if (m.trajectory_complete) { assert.equal(m.goals_selected, selected.length); assert.equal(m.fallback_goals, selected.filter(s => s.kind === 'fallback').length); }
  let start = null, first = null, stalled = 0;
  for (const e of events.filter(e => e.event_type === 'SELECTOR_STATUS')) {
    if (e.payload.stalled && start === null) { start = e.sim_time_s; if (first === null) first = start; }
    else if (!e.payload.stalled && start !== null) { stalled += e.sim_time_s-start; start = null; }
  }
  if (start !== null) stalled += m.elapsed_sim_s-start;
  if (m.trajectory_complete) { assert.equal(m.first_stalled_s, first); approx(m.stalled_sim_s, stalled); }
  if (first !== null) stalledEpisodes++;
  const observations = new Map(); let firstReport = null;
  for (const e of events) {
    if (e.event_type === 'CONFIRMED_DETECTION') observations.set(e.payload.observation_id,e.payload);
    if (e.event_type === 'TARGET_REPORT' && firstReport === null) {
      const r = e.payload.report, o = r && observations.get(r.observation_id), target = resolved.episode.targets[0];
      const required = ['report_id','label','x_m','y_m','frame_id','observation_id'].sort();
      const valid = r && Object.keys(r).sort().join() === required.join() && typeof r.report_id === 'string' && r.report_id.length > 0
        && Number.isFinite(r.x_m) && Number.isFinite(r.y_m) && r.frame_id === 'benchmark_map' && o && o.confirmed
        && r.label === o.label && e.sim_time_s >= o.last_confirmed_seen_s && e.sim_time_s-o.last_confirmed_seen_s <= resolved.shared.camera.memory_s;
      firstReport = Boolean(valid && target && r.label === target.label && e.sim_time_s < resolved.episode.time_limit_s
        && Math.hypot(r.x_m-target.x_m,r.y_m-target.y_m) <= resolved.shared.report_tolerance_m);
    }
  }
  assert.equal(m.report_correct,firstReport === true);
  assert.equal(m.n_false_reports,firstReport === null || firstReport === true ? 0 : 1);
  assert.equal(m.report_correct, result.report_correct); assert.equal(m.mission_success, result.mission_success);
  assert.equal(m.end_reason, result.end_reason); if (m.execution_error) assert.equal(m.mission_success, false);
  if (reduced.replica === 'primary' && m.attempt_id === 1) {
    const confirmation = events.find(e => e.event_type === 'CONFIRMED_DETECTION');
    const rawSteps = new Set(events.filter(e => e.event_type === 'RAW_DETECTION').map(e => e.step_index));
    diagnostics.push({episode_id:m.episode_id,policy_id:m.policy_id,end_reason:m.end_reason,
      classification:m.execution_error ? 'execution_error' : m.is_positive
        ? m.mission_success ? 'mission_completed' : 'positive_failure_requires_trace_review'
        : m.end_reason === 'TIMEOUT' && m.n_false_reports === 0 ? 'negative_control_completed' : 'negative_failure_requires_trace_review',
      goals_selected:m.goals_selected,fallback_goals:m.fallback_goals,first_stalled_s:m.first_stalled_s,
      stalled_sim_s:m.stalled_sim_s,unresolved_waypoints:m.unresolved_waypoints,
      failed_goal_outcomes:events.filter(e => e.event_type === 'WAYPOINT_OUTCOME' && e.payload.outcome === 'failed').length,
      raw_gap_steps_after_confirmation:confirmation ? events.filter(e => e.event_type === 'SENSE'
        && e.step_index > confirmation.step_index && !rawSteps.has(e.step_index)).length : 0,
      report_correct:m.report_correct,mission_success:m.mission_success,attempt_path:folder});
  }
  legacyTotal += legacy; continuousTotal += continuous; rawAttempts++;
}
for (const replica of ['primary','replay']) for (const policy of Object.keys(spec.policies)) for (const eid of spec.selection) {
  const slot = spec.slots[replica][policy][eid], records = fs.readFileSync(slot + '/journal.jsonl', 'utf8').trim().split('\n').map(JSON.parse);
  let head = '0'.repeat(64); const starts = new Map(), finish = new Set();
  records.forEach((record,i) => {
    const { checksum: actual, ...body } = record;
    assert.equal(record.seq,i+1); assert.equal(record.previous,head); assert.equal(checksum(body),actual); head=actual;
    if (record.kind === 'START') { assert(!starts.has(record.attempt_id)); starts.set(record.attempt_id,record); }
    if (record.kind === 'FINISH') {
      assert(!finish.has(record.attempt_id)); finish.add(record.attempt_id);
      for (const [file,hash] of Object.entries(record.hashes)) assert.equal(sha(fs.readFileSync(path.join(slot,starts.get(record.attempt_id).path,file))),hash);
    }
  });
  assert(starts.has(1)); assert.equal(starts.size,finish.size);
}
const paired = csv(root + '/analysis/paired-results.csv'); assert.equal(paired.length,spec.selection.length);
assert.deepEqual([...new Set(paired.map(r => r.episode_id))].sort(),[...spec.selection].sort());
const policies = Object.keys(spec.policies); assert.equal(policies.length,2);
for (const row of paired) {
  assert(spec.selection.includes(row.episode_id));
  for (const [prefix,policy] of [['b0','b0-fixed-order-v2'],['b1','b1-gain-distance-focus-v1']]) {
    const reduced = byKey.get(['primary',policy,row.episode_id,1].join('/')); assert(reduced);
    assert.equal(row.fixture_hash,reduced.metrics.fixture_hash);
    for (const [column,value] of Object.entries(row)) if (column.startsWith(prefix+'_')) {
      const expected = reduced.metrics[column.slice(prefix.length+1)];
      if (typeof expected === 'number') { assert.notEqual(value,''); approx(Number(value),expected); }
      else assert.equal(value, expected === null ? '' : typeof expected === 'boolean' ? expected ? 'True' : 'False' : String(expected),column);
    }
    const replay = report.replay[policy][row.episode_id];
    const a = byKey.get(['primary',policy,row.episode_id,1].join('/')).attempt_path;
    const b = byKey.get(['replay',policy,row.episode_id,1].join('/')).attempt_path;
    const trajectoryEqual = fs.readFileSync(a + '/trajectory.csv').equals(fs.readFileSync(b + '/trajectory.csv'));
    const clean = file => fs.readFileSync(file,'utf8').trim().split('\n').map(JSON.parse).map(({run_id,attempt_id,...e}) => e);
    const eventsEqual = JSON.stringify(sorted(clean(a + '/events.jsonl'))) === JSON.stringify(sorted(clean(b + '/events.jsonl')));
    const cleanResult = folder => { const {run_id,attempt_id,elapsed_wall_s,workers_reaped,...r} = read(folder + '/result.json'); return r; };
    const resultEqual = JSON.stringify(sorted(cleanResult(a))) === JSON.stringify(sorted(cleanResult(b)));
    const status = trajectoryEqual && eventsEqual && resultEqual ? 'PASS' : 'FAIL';
    assert.equal(replay.status,status);
    assert.equal(replay.left_hash === replay.right_hash,status === 'PASS');
    strictReplays.push({policy_id:policy,episode_id:row.episode_id,status,trajectory_equal:trajectoryEqual,
      semantic_events_equal:eventsEqual,result_equal:resultEqual});
  }
}
const attempts = csv(root + '/analysis/attempts.csv'), episodes = csv(root + '/analysis/episodes.csv');
assert.equal(episodes.length,spec.selection.length*2); assert(episodes.every(r => r.attempt_id === '1'));
assert.equal(attempts.length,report.reductions.filter(r => r.replica === 'primary').length);
const allAttempts = csv(root + '/analysis/all-attempts.csv');assert.equal(allAttempts.length,report.reductions.length);
assert.equal(new Set(allAttempts.map(r => [r.replica,r.policy_id,r.episode_id,r.attempt_id].join('/'))).size,allAttempts.length);
for (const r of allAttempts) assert(byKey.has([r.replica,r.policy_id,r.episode_id,r.attempt_id].join('/')));
for (const table of [attempts,episodes]) {
  const keys = new Set();
  for (const row of table) {
    const key = ['primary',row.policy_id,row.episode_id,row.attempt_id].join('/'); assert(!keys.has(key)); keys.add(key);
    const expected = byKey.get(key); assert(expected);
    for (const [name,value] of Object.entries(row)) {
      const actual = expected.metrics[name];
      if (actual === null) assert.equal(value,'');
      else if (typeof actual === 'number') { assert.notEqual(value,''); approx(Number(value),actual); }
      else if (typeof actual === 'boolean') assert.equal(value,actual ? 'True':'False');
      else if (typeof actual === 'object') assert.deepEqual(JSON.parse(value),actual);
      else assert.equal(value,actual);
    }
  }
}
for (const policy of policies) {
  const primary = report.reductions.filter(r => r.replica === 'primary' && r.metrics.policy_id === policy && r.metrics.attempt_id === 1).map(r => r.metrics);
  const positive = primary.filter(r => r.is_positive), negative = primary.filter(r => !r.is_positive), summary = report.summaries[policy];
  assert.equal(summary.started,primary.length); assert.equal(summary.execution_errors,primary.filter(r => r.execution_error).length);
  assert.equal(summary.ReportSuccess,positive.length ? positive.filter(r => r.report_correct).length/positive.length : null);
  assert.equal(summary.SafeMissionSuccess,positive.length ? positive.filter(r => r.mission_success).length/positive.length : null);
  assert.equal(summary.negative_false_alarm_rate,negative.length ? negative.filter(r => r.n_false_reports > 0).length/negative.length : null);
}
assert(stalledEpisodes > 0); assert.equal(spec.fixtures['pilot-offset-s1'].environment.split,'dev');
const offset = read(bundle + '/pilot-offset-s1/resolved_episode.json').shared.camera;
assert(offset.x_m !== 0 && offset.y_m !== 0 && offset.yaw_rad !== 0);
const replayIssues = strictReplays.filter(r => r.status !== 'PASS').map(r => 'replay mismatch: '+r.policy_id+'/'+r.episode_id);
assert.deepEqual([...report.issues].sort(),replayIssues.sort());
assert.equal(report.status,replayIssues.length ? 'FAIL' : 'PASS');
const result = { status:report.status,raw_identity_csv_monitor_checks:'PASS',purpose:'pipeline_verification_not_research',locked_pairs:paired.length,raw_attempts:rawAttempts,
  primary_attempts:episodes.length,source_files:Object.keys(certificates.verifier).length,monitor_counts_independently_checked:true,
  paired_csv_checked:true,strict_replay_checked:true,strict_replays:strictReplays,stalled_attempts_observed:stalledEpisodes,
  monitor_state_entries_observed:legacyTotal,monitor_continuous_episodes_observed:continuousTotal };
fs.writeFileSync(base + '/trace-review.json',JSON.stringify({purpose:'trace_diagnostics_not_policy_ranking',rows:diagnostics},null,2)+'\n');
fs.writeFileSync(base + '/independent-check.json',JSON.stringify(result,null,2)+'\n');console.log(JSON.stringify(result));
if (result.status !== 'PASS') process.exitCode = 1;
