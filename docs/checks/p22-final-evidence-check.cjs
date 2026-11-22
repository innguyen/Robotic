// Adds host continuity and frozen comparability to the existing independent checker.
const fs=require('node:fs'),crypto=require('node:crypto'),assert=require('node:assert/strict'),cp=require('node:child_process');
const base=process.argv[2]||'docs/p22-final-audit-logs/pilot';
const check=cp.spawnSync(process.execPath,['docs/checks/p22-evidence-check.cjs',base],{encoding:'utf8'});
if(check.status!==0){process.stderr.write(check.stderr||check.stdout);process.exit(check.status||1);}
const sha=b=>crypto.createHash('sha256').update(b).digest('hex');
const read=p=>JSON.parse(fs.readFileSync(p,'utf8').replace(/^\uFEFF/,''));
const root=base+'/paired',spec=read(root+'/paired_manifest.json'),rawIdentity=fs.readFileSync(root+'/stabilization_manifest.json');
const identity=JSON.parse(rawIdentity),audit=read(root+'/analysis/stabilization-audit.json');
assert.equal(identity.version,'stabilized-paired-1');assert.equal(identity.host_version,'host-liveness-1');
assert.equal(identity.period_s,.5);assert.equal(identity.suspend_gap_s,2);assert.equal(identity.observer_gap_s,5);
assert.equal(identity.concurrency,1);assert.equal(identity.power_settings_changed,false);assert.equal(identity.watchdog_changed,false);assert.equal(identity.scoring_changed,false);
assert.equal(identity.paired_manifest_hash,sha(fs.readFileSync(root+'/paired_manifest.json')));
for(const [name,hash] of Object.entries(identity.sources))assert.equal(sha(fs.readFileSync('test version/benchmark/'+name)),hash);
assert.equal(audit.status,'PASS');assert.deepEqual(audit.issues,[]);assert.equal(audit.paired_status,'PASS');assert.deepEqual(audit.identity,identity);
const keys=new Set();let samplesCount=0,cpuSamples=0,maxGap=0,maxSuspend=0,minUtc=Infinity,maxUtc=-Infinity;
for(const replica of ['primary','replay'])for(const policy of Object.keys(spec.policies))for(const eid of spec.selection){
  const folder=root+'/host/'+replica+'/'+policy+'/'+eid,host=read(folder+'/result.json');
  const bytes=fs.readFileSync(folder+'/samples.jsonl');assert(bytes[bytes.length-1]===10);
  const rows=bytes.toString('utf8').trim().split('\n').map(JSON.parse);assert(rows.length>=2);
  assert.equal(host.samples_hash,sha(bytes));assert.equal(host.identity_hash,sha(rawIdentity));assert.equal(host.pair_hash,identity.paired_manifest_hash);
  assert.equal(host.policy,policy);assert.equal(host.replica,replica);assert.equal(host.episode_id,eid);assert.equal(host.slot,spec.slots[replica][policy][eid]);
  const journal=fs.readFileSync(host.slot+'/journal.jsonl','utf8').trim().split('\n').map(JSON.parse);
  assert.equal(host.journal_head,journal.at(-1).checksum);assert.equal(journal.filter(r=>r.kind==='START').length,1);
  assert.equal(host.diagnostics.status,'PASS');assert.deepEqual(host.diagnostics.issues,[]);
  const match=audit.host_jobs.filter(r=>r.policy===policy&&r.replica===replica&&r.episode_id===eid);assert.equal(match.length,1);assert.deepEqual(match[0],host);
  const result=read(host.slot+'/attempts/'+eid+'/attempt-0001/result.json');assert.equal(result.trajectory_complete,true);
  assert(!['CRASH','WALL_TIMEOUT','PLANNER_TIMEOUT','INTERRUPTED','INVALID_STATE'].includes(result.end_reason));
  let previous=null,step=-1;
  rows.forEach((r,i)=>{
    assert.equal(r.seq,i+1);assert(!r.error);assert(Number.isFinite(r.clock.active_s));assert(Number.isFinite(r.clock.boot_s));
    minUtc=Math.min(minUtc,r.clock.utc_s);maxUtc=Math.max(maxUtc,r.clock.utc_s);
    if(previous){const d=r.clock.monotonic_s-previous.clock.monotonic_s;assert(d>=0&&d<=5);maxGap=Math.max(maxGap,d);
      const gap=(r.clock.boot_s-previous.clock.boot_s)-(r.clock.active_s-previous.clock.active_s);assert(gap<=2);maxSuspend=Math.max(maxSuspend,gap);}
    if(r.progress.step_index!==null){assert(r.progress.step_index>=step);step=r.progress.step_index;assert(Math.abs(r.progress.sim_time_s-(step+1)*.1)<1e-8);}
    if(Object.values(r.worker_cpu_s).some(v=>v!==null&&v>0))cpuSamples++;previous=r;
  });
  const last=rows.at(-1).progress;assert.equal(last.sim_time_s,result.elapsed_sim_s);
  assert.deepEqual(last.workers,read(host.slot+'/attempts/'+eid+'/attempt-0001/workers.json'));
  keys.add(replica+'/'+policy+'/'+eid);samplesCount+=rows.length;
}
assert.equal(keys.size,32);assert.equal(audit.host_jobs.length,32);assert(cpuSamples>0);
const power=read(base+'/host-power-events.json');assert.equal(power.query_status,'PASS');assert.deepEqual(power.events,[]);
assert.equal(power.sample_files,32);assert(power.window_start_utc_s<=minUtc&&power.window_end_utc_s>=maxUtc);
const old=read('docs/p22-audit-logs/pilot-serial/paired/paired_manifest.json');assert.deepEqual(spec.policies,old.policies);assert.deepEqual(spec.metrics,old.metrics);
for(const eid of spec.selection)for(const key of ['environment','resolved_bytes_hash','shared_hash','config_hash','budget_s','dt_s'])assert.deepEqual(spec.fixtures[eid][key],old.fixtures[eid][key]);
const output={status:'PASS',checker_version:'p22-final-evidence-1',checker_sha256:sha(fs.readFileSync(__filename)),host_jobs:keys.size,host_samples:samplesCount,cpu_observation_samples:cpuSamples,max_observer_gap_s:maxGap,max_biased_minus_active_gap_s:maxSuspend,
  independent_paired_csv_monitor_replay:'PASS',strict_replay_pairs:16,policy_and_metric_identities_unchanged:true,inputs_and_budgets_unchanged:true,kernel_power_events_in_run:0,
  limitation:'No independent planner phase heartbeat or scheduler trace; CPU-quiet alone cannot diagnose starvation/deadlock.'};
fs.writeFileSync(base+'/final-independent-check.json',JSON.stringify(output,null,2)+'\n');console.log(JSON.stringify(output));
