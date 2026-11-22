"""Bounded paired dev pilot commands; each invocation runs at most two jobs.

prepare -> run (repeat until complete) -> audit. Input fixtures are immutable once
prepared. No outcome-dependent tuning, hidden test rollouts, or robot connections.
"""
import argparse
import copy
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'test version'))
from benchmark.artifacts import encode,load_catalog,build_bundle,persist,output_path
from benchmark.schema import load_json
from benchmark.layout import layout_hash
from benchmark.paired import initialize,load_pair,job,audit,journal_records,POLICIES,REPLICAS
from benchmark.batch import attempts,committed_hashes
from benchmark.journal import run_lock

FIXTURES='test version/benchmark/fixtures/'


def write_input(name,value):
    path=ROOT/name;raw=(json.dumps(value,indent=2,ensure_ascii=False)+'\n').encode('utf8')
    if path.exists():
        if path.read_bytes()!=raw:raise ValueError('pilot input differs; do not overwrite '+name)
    else:path.write_bytes(raw)


def prepare(output):
    original=load_json(ROOT/(FIXTURES+'samples.json'));advanced=load_json(ROOT/(FIXTURES+'advanced.json'))
    base=load_json(ROOT/(FIXTURES+'catalog.json'));witnesses=load_json(ROOT/(FIXTURES+'corpus-witnesses.json'))
    shared=copy.deepcopy(original['shared'])
    compact=copy.deepcopy(original['episodes'][2]);compact.update(episode_id='pilot-compact-s5',environment_id='pilot-compact-region',
        bounds_m=[-.35,-.35,.35,.35],time_limit_s=60.)
    compact['layout_hash']=layout_hash(compact)
    offset=copy.deepcopy(original['episodes'][0]);offset.update(episode_id='pilot-offset-s1',environment_id='pilot-offset-layout',
        layout_family_id='pilot-offset-family',bounds_m=[-1.2,-2.2,4.2,2.2],
        obstacles=[dict(id='rear-block',bounds_m=[-.8,1.1,-.5,1.4]),dict(id='far-block',bounds_m=[2.8,1.3,3.2,1.5])])
    offset['targets'][0].update(x_m=1.8,y_m=-.1);offset['layout_hash']=layout_hash(offset)
    offset_shared=copy.deepcopy(shared);offset_shared['camera'].update(x_m=.12,y_m=.08,yaw_rad=.08)
    lower=copy.deepcopy(advanced['episodes'][0]);lower.update(episode_id='pilot-lower-s3');lower['start_pose']['x_m']=.1
    lower['targets'][0]['y_m']=-2.5
    suites=[('p22-compact.json',dict(schema_version='t1-0.2',suite_id='p22-compact-dev',shared=shared,episodes=[compact])),
            ('p22-offset.json',dict(schema_version='t1-0.2',suite_id='p22-offset-dev',shared=offset_shared,episodes=[offset])),
            ('p22-lower.json',dict(schema_version='t1-0.2',suite_id='p22-lower-dev',shared=copy.deepcopy(advanced['shared']),episodes=[lower]))]
    for name,suite in suites:write_input(FIXTURES+name,suite)
    witnesses['pilot-offset-s1']=[[0.,0.,0.],[1.1,-.1,0.]]
    witnesses['pilot-lower-s3']=dict(poses=[[.1,0.,0.],[.1,-2.5,0.],[3.3,-2.5,0.]],
        branches={'upper':[[.1,0.,0.],[.1,2.5,0.],[3.,2.5,0.]]})
    write_input(FIXTURES+'p22-witnesses.json',witnesses)
    catalog=copy.deepcopy(base);catalog['suites'] += [FIXTURES+name for name,_ in suites];catalog['witnesses']=FIXTURES+'p22-witnesses.json'
    catalog['families']['sample-family-5']['notes'] += ' P2.2 compact bounds are an explicit same-family scale variant, not an independent family.'
    catalog['families']['sample-family-3']['notes'] += ' P2.2 lower-target episode reuses exactly this layout; start/target variant is not a new environment.'
    catalog['families']['pilot-offset-family']=dict(source='hand-authored/p22-offset-two-block-layout-v1',reviewed=True,
        notes='Reviewed construction: two disjoint axis-aligned blocks, legal start, visible S1 target, swept approach witness. Camera offset is declared equally for both policies.')
    catalog_name=FIXTURES+'p22-catalog.json';write_input(catalog_name,catalog)
    data,ss,cfgs,ww=load_catalog(catalog_name);files,checks=build_bundle(data,ss,cfgs,ww)
    if any(c['status']!='PASS' for c in checks):raise ValueError(checks)
    destination=output_path(output);persist(files,destination/'certificates')
    selection=[e['episode_id'] for s in ss for e in s['episodes'] if e['split']=='dev']
    pair=output+'/paired'
    if not output_path(pair).exists():initialize(catalog_name,output+'/certificates',pair,selection)
    else:load_pair(pair)
    provenance=dict(purpose='dev_pilot_not_research',episodes=selection,certificates=checks,
        dev_layouts=len({e['environment_id'] for s in ss for e in s['episodes'] if e['split']=='dev'}),
        dev_families=len({e['layout_family_id'] for s in ss for e in s['episodes'] if e['split']=='dev'}),
        design=['P2.1 S1-S5 retained as regression anchors, never evidence of superiority',
                'offset S1: new two-block hand-authored layout and nonzero camera x/y/yaw',
                'compact S5: same-family bounds variant; no goal can meet common minimum distance, so sensing/stalled continues to deadline',
                'lower S3: same environment/family, target in the other certified branch; unchanged common 600-second budget'])
    (destination/'provenance.json').write_bytes(encode(provenance))
    print(json.dumps(dict(status='PREPARED',episodes=len(selection),certificates=checks)),flush=True)


def finished(slot):
    folder=output_path(slot)
    if not folder.exists():return False
    records,_=journal_records(folder/'journal.jsonl')
    state=attempts(type('JournalView',(),{'records':records})())
    if not state or any(item['finished'] is None for item in state.values()):return False
    for item in state.values():
        if committed_hashes(folder/item['path'])!=item['finished']['hashes']:raise ValueError('completed job changed')
    return True


def run(output,limit):
    if not 1<=limit<=2:raise ValueError('bounded driver permits one or two jobs')
    pair=output+'/paired'
    with run_lock(output_path(pair)):spec=load_pair(pair)
    done=0
    for replica in REPLICAS:
        for eid in spec['selection']:
            for policy in POLICIES:
                slot=spec['slots'][replica][policy][eid]
                if finished(slot):continue
                summary=job(pair,policy,replica,eid,resume=output_path(slot).exists())
                print(json.dumps(dict(replica=replica,episode=eid,policy=policy,end_reasons=summary['overall']['end_reasons'],purpose='dev_pilot_not_research')),flush=True)
                done+=1
                if done>=limit:return
    print(json.dumps(dict(status='ROLLOUTS_COMPLETE',jobs_this_invocation=done)),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=('prepare','run','audit'))
    parser.add_argument('--output',required=True);parser.add_argument('--limit',type=int,default=2);args=parser.parse_args()
    if args.action=='prepare':prepare(args.output)
    elif args.action=='run':run(args.output,args.limit)
    else:
        result=audit(args.output+'/paired');print(json.dumps(dict(status=result['status'],issues=result['issues'])))
        return 0 if result['status']=='PASS' else 1
    return 0


if __name__=='__main__':raise SystemExit(main())
