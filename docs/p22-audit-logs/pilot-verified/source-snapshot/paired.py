"""Locked dev paired runs. Each bounded job is one existing isolated batch.

No policy or simulator receives this manifest, certificates or reduced metrics.
Readers verify journals without using Journal(), which can repair a torn tail.
"""
import argparse
import csv
import io
import json
import math
import platform
import sys
from pathlib import Path
from importlib.metadata import version

from .artifacts import output_path,encode,digest
from .schema import ROOT,load_json
from .runner import verified_corpus
from .batch import run_batch,committed_hashes,attempts
from .policy import policy_identity
from .selectors import B0,B1
from .metrics import identity as metric_identity,reduce_attempt,summarize
from .replay import compare
from .journal import atomic

SCHEMA = 't1-paired-1'
POLICIES = (B0,B1)
REPLICAS = ('primary','replay')


def runtime():
    return dict(python=sys.version,platform=platform.platform(),dependencies={n:version(n) for n in ('numpy','PyYAML')})


def contract(catalog,bundle,selection,output):
    prepared = verified_corpus(catalog,bundle,require_all=True)  # Before filtering.
    if not selection or len(set(selection))!=len(selection) or set(selection)-set(prepared):raise ValueError('invalid paired selection')
    if any(prepared[eid][0]['split']!='dev' for eid in selection):raise ValueError('paired dev pilot cannot select test')
    bm = load_json(output_path(bundle)/'manifest.json')
    fixtures = {}
    for eid in selection:
        e,s,cfg,binding = prepared[eid]
        fixtures[eid] = dict(binding,environment={k:e[k] for k in ('environment_id','layout_hash','layout_family_id','split','seed','scenario_id')},
            resolved_bytes_hash=digest((output_path(bundle)/eid/'resolved_episode.json').read_bytes()),
            shared_hash=digest(encode(s)),config_hash=digest(encode(cfg)),budget_s=e['time_limit_s'],dt_s=s['dt_s'])
    return dict(schema_version=SCHEMA,protocol='t1-0.2',purpose='dev_pilot_not_research',output=output,catalog=catalog,bundle=bundle,
        selection=selection,fixtures=fixtures,corpus_bindings={eid:value[3] for eid,value in prepared.items()},
        registry={eid:{k:value[0][k] for k in ('environment_id','layout_hash','layout_family_id','split')} for eid,value in prepared.items()},
        config_hashes={eid:value[2]['hashes'] for eid,value in prepared.items()},
        policies={p:{eid:policy_identity(p,prepared[eid][1],prepared[eid][2]) for eid in selection} for p in POLICIES},
        evaluator_sources=bm['verifier'],metrics=metric_identity(),runtime=runtime(),
        sensing=dict(perception_mode='oracle',lidar_model=dict(beams=720,range_min_m=.05,range_max_m=6.,
            angle_min_rad=-math.pi,angle_increment_rad=2*math.pi/720)),
        dynamics='holonomic-euler-dt-0.1',planner='async_next_tick_barrier',primary_attempt=1,
        slots={rep:{p:{eid:output+'/runs/'+rep+'/'+p+'/'+eid for eid in selection} for p in POLICIES} for rep in REPLICAS})


def initialize(catalog,bundle,output,episode_ids):
    directory = output_path(output)
    if directory.exists():raise ValueError('paired output already exists')
    body = contract(catalog,bundle,list(episode_ids),output)
    directory.mkdir(parents=True)
    raw = encode(body)
    atomic(directory/'paired_manifest.json',raw)
    atomic(directory/'paired_manifest.sha256',(digest(raw)+'\n').encode())
    return body


def load_pair(output):
    directory = output_path(output); raw = (directory/'paired_manifest.json').read_bytes()
    if digest(raw)!=(directory/'paired_manifest.sha256').read_text().strip():raise ValueError('paired manifest hash mismatch')
    spec = load_json(directory/'paired_manifest.json')
    if spec.get('schema_version')!=SCHEMA or spec.get('output')!=output:raise ValueError('paired location/schema mismatch')
    expected = contract(spec['catalog'],spec['bundle'],spec['selection'],output)
    if spec!=expected:raise ValueError('paired contract/source/config/runtime identity mismatch')
    return spec


def job(output,policy,replica,episode_id,resume=False,retry_errors=False):
    spec = load_pair(output)
    if policy not in POLICIES or replica not in REPLICAS or episode_id not in spec['selection']:raise ValueError('unknown paired job')
    slot = spec['slots'][replica][policy][episode_id]
    binding = encode(dict(pair_hash=digest(encode(spec)),policy=policy,replica=replica,episode_id=episode_id,slot=slot))
    marker = output_path(output)/'bindings'/replica/policy/(episode_id+'.json')
    if marker.exists():
        if marker.read_bytes()!=binding:raise ValueError('job/pair binding changed')
    else:atomic(marker,binding)
    return run_batch(spec['catalog'],spec['bundle'],slot,policy,episode_ids=[episode_id],resume=resume,retry_errors=retry_errors)


def journal_records(path):
    raw = path.read_bytes()
    if raw and not raw.endswith(b'\n'):raise ValueError('incomplete journal; use batch resume, not read-time repair')
    records = []; head = '0'*64
    for line in raw.splitlines():
        record = json.loads(line); body = {k:v for k,v in record.items() if k!='checksum'}
        if record['seq']!=len(records)+1 or record['previous']!=head or digest(encode(body))!=record['checksum']:
            raise ValueError('journal integrity failure')
        records.append(record); head = record['checksum']
    return records,head


def read_job(spec,policy,replica,eid):
    slot = spec['slots'][replica][policy][eid]; directory = output_path(slot)
    marker = output_path(spec['output'])/'bindings'/replica/policy/(eid+'.json')
    if load_json(marker)!=dict(pair_hash=digest(encode(spec)),policy=policy,replica=replica,episode_id=eid,slot=slot):raise ValueError('job binding mismatch')
    manifest = load_json(directory/'run_manifest.json')
    expected = dict(protocol='t1-0.2',selection=[eid],policies={eid:policy},policy_identities={eid:spec['policies'][policy][eid]},
                    fixtures=spec['corpus_bindings'],runtime=spec['runtime'])
    if (manifest['identity']!=expected or manifest['registry']!=spec['registry'] or manifest['config_hashes']!=spec['config_hashes']
            or manifest['protocol']!='t1-0.2'):raise ValueError('batch/paired identity mismatch')
    records,head = journal_records(directory/'journal.jsonl')
    holder = type('JournalView',(),{'records':records})()
    state = attempts(holder)
    if not state or any(key[0]!=eid or item['finished'] is None for key,item in state.items()):raise ValueError('incomplete/foreign paired attempt')
    if sorted(key[1] for key in state)!=list(range(1,len(state)+1)):raise ValueError('attempt gap/missing primary')
    reductions = []
    for (episode_id,number),item in sorted(state.items()):
        folder = directory/item['path']
        if any(p.is_symlink() for p in (folder,*folder.parents) if p.is_relative_to(directory)):raise ValueError('attempt symlink forbidden')
        if committed_hashes(folder)!=item['finished']['hashes']:raise ValueError('completed paired attempt modified')
        attempt_manifest = load_json(folder/'run_manifest.json')
        binding = spec['corpus_bindings'][eid]
        if (attempt_manifest['policy_identity']!=spec['policies'][policy][eid] or attempt_manifest['policy_id']!=policy
                or attempt_manifest['config']['hashes']!=spec['config_hashes'][eid] or attempt_manifest['test_fault'] is not None
                or any(attempt_manifest[k]!=binding[k] for k in binding) or attempt_manifest['run_id']!=manifest['run_id']
                or any(attempt_manifest[k]!=v for k,v in spec['sensing'].items()) or attempt_manifest['planner_mode']!=spec['planner']):
            raise ValueError('attempt/paired identity mismatch')
        reduced = reduce_attempt(folder,spec['bundle'])
        if reduced['metrics']['attempt_id']!=number:raise ValueError('attempt number mismatch')
        reduced.update(batch=slot,attempt_path=folder.relative_to(ROOT).as_posix(),replica=replica)
        reductions.append(reduced)
    # Compare existing evaluator projection, without adopting it as metric input.
    with (directory/'episodes.csv').open(encoding='utf8',newline='') as stream:projection = list(csv.DictReader(stream))
    if len(projection)!=1 or projection[0]['episode_id']!=eid or projection[0]['attempt_id']!='1':raise ValueError('primary projection mismatch')
    primary = reductions[0]['metrics']
    for key in ('end_reason','report_correct','mission_success'):
        if projection[0][key]!=str(primary[key]):reductions[0]['issues'].append('episodes.csv mismatch: '+key);reductions[0]['status']='FAIL'
    return reductions


def paired_rows(spec,primary):
    """One row per locked fixture; no dropping failures or choosing a retry."""
    result = []
    fields = ('policy_id','policy_version','attempt_id','end_reason','report_correct','mission_success','execution_error',
        'elapsed_sim_s','path_length_m','goals_selected','fallback_goals','first_stalled_s','stalled_sim_s','unresolved_waypoints',
        'monitor_interventions','monitor_continuous_episodes','monitor_intervention_s')
    for eid in spec['selection']:
        environment = spec['fixtures'][eid]['environment']
        row = dict(episode_id=eid,fixture_hash=spec['fixtures'][eid]['resolved_bytes_hash'],**environment)
        for prefix,policy in (('b0',B0),('b1',B1)):
            candidates = [r['metrics'] for r in primary if r['metrics']['policy_id']==policy and r['metrics']['episode_id']==eid and r['metrics']['attempt_id']==1]
            if len(candidates)!=1:raise ValueError('missing/duplicate primary pair')
            item = candidates[0]
            if item['fixture_hash']!=row['fixture_hash'] or any(item[k]!=v for k,v in environment.items()):raise ValueError('pair fixture identity mismatch')
            row.update({prefix+'_'+key:item[key] for key in fields})
        result.append(row)
    return result


def audit(output):
    spec = load_pair(output); reductions = []; replay = {}; issues = []
    expected_slots = {output_path(slot) for rep in spec['slots'].values() for policy in rep.values() for slot in policy.values()}
    if any(p.parent not in expected_slots for p in (output_path(output)/'runs').rglob('run_manifest.json') if '/attempts/' not in p.as_posix()):
        raise ValueError('foreign/omitted batch in paired archive')
    for policy in POLICIES:
        replay[policy] = {}
        for eid in spec['selection']:
            replicas = {rep:read_job(spec,policy,rep,eid) for rep in REPLICAS}
            for values in replicas.values():reductions.extend(values)
            check = compare(ROOT/replicas['primary'][0]['attempt_path'],ROOT/replicas['replay'][0]['attempt_path'])
            replay[policy][eid] = check
            if check['status']!='PASS':issues.append('replay mismatch: '+policy+'/'+eid)
    primary = [r for r in reductions if r['replica']=='primary']
    rows = paired_rows(spec,primary)
    for reduced in reductions:
        if reduced['status']!='PASS':issues.extend([reduced['attempt_path']+': '+v for v in reduced['issues']])
    summaries = {p:summarize([r['metrics'] for r in primary if r['metrics']['policy_id']==p]) for p in POLICIES}
    directory = output_path(output)/'analysis'; directory.mkdir(exist_ok=True)
    def csv_file(name,values):
        stream = io.StringIO(newline=''); writer = csv.DictWriter(stream,fieldnames=list(values[0]));writer.writeheader()
        writer.writerows({k:encode(v).decode('utf8') if isinstance(v,(dict,list)) else v for k,v in row.items()} for row in values)
        atomic(directory/name,stream.getvalue().encode())
    csv_file('paired-results.csv',rows)
    csv_file('attempts.csv',[r['metrics'] for r in primary])
    csv_file('episodes.csv',[r['metrics'] for r in primary if r['metrics']['attempt_id']==1])
    report = dict(schema_version=SCHEMA,purpose='dev_pilot_not_research',status='FAIL' if issues else 'PASS',issues=issues,
        pair_manifest_hash=digest(encode(spec)),metrics=metric_identity(),summaries=summaries,replay=replay,reductions=reductions,
        research_conclusion=None,reason='hand-authored dev pilot; no held-out or superiority inference')
    atomic(directory/'audit.json',encode(report))
    text = '# Paired dev pilot audit\n\nPurpose: pipeline verification, not research ranking.\n\n'
    text += f"Status: {report['status']}; locked episodes: {len(rows)}; primary attempts retained: {sum(s['started'] for s in summaries.values())}.\n"
    text += '\nEvery retry is in attempts.csv; episodes.csv and paired-results.csv use attempt 1. See audit.json for exact checks and unavailable metrics.\n'
    atomic(directory/'summary.md',text.encode())
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__); commands = parser.add_subparsers(dest='action',required=True)
    init = commands.add_parser('init')
    for key in ('catalog','bundle','output'):init.add_argument('--'+key,required=True)
    init.add_argument('--episode-id',action='append',required=True)
    run = commands.add_parser('run');run.add_argument('--output',required=True);run.add_argument('--policy',choices=POLICIES,required=True)
    run.add_argument('--replica',choices=REPLICAS,required=True);run.add_argument('--episode-id',required=True)
    run.add_argument('--resume',action='store_true');run.add_argument('--retry-errors',action='store_true')
    check = commands.add_parser('audit');check.add_argument('--output',required=True)
    args = parser.parse_args()
    try:
        if args.action=='init':initialize(args.catalog,args.bundle,args.output,args.episode_id);result={'status':'INITIALIZED'}
        elif args.action=='run':
            summary = job(args.output,args.policy,args.replica,args.episode_id,args.resume,args.retry_errors)
            result = dict(status='EXECUTED',execution_errors=summary['overall']['execution_errors'])
        else:
            report = audit(args.output);result = dict(status=report['status'],issues=report['issues'])
        print(json.dumps(result));return 1 if result.get('status')=='FAIL' or result.get('execution_errors',0) else 0
    except (ValueError,OSError,KeyError,TypeError) as error:print(json.dumps({'status':'BLOCKED','error':str(error)}));return 2


if __name__=='__main__':raise SystemExit(main())
