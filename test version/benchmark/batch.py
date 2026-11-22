"""P1b.2 sequential batch orchestration; cooperative policies, no B0/B1 study."""
import argparse
import csv
import io
import json
import platform
import sys
import uuid
import time
import tempfile
import os
import subprocess
import yaml
from collections import Counter
from importlib.metadata import version
from pathlib import Path

from .artifacts import output_path,encode,digest
from .schema import ROOT,load_json
from .runner import verified_corpus,_execute_single
from .policy import POLICIES
from .oracle import Observation
from .evaluator import evaluate_report
from .journal import Journal,atomic,run_lock
from .lifecycle import process_alive

ERRORS={'CRASH','PLANNER_TIMEOUT','WALL_TIMEOUT','INTERRUPTED','INVALID_STATE'}


def event_records(path):
    if not path.exists():return []
    records=[]
    for line in path.read_bytes().splitlines(keepends=True):
        if not line.endswith(b'\n'):break
        record=json.loads(line)
        if record['event_seq']!=len(records)+1:raise ValueError('attempt event sequence invalid')
        records.append(record)
    return records


def recover_result(directory,prepared,run_id,attempt_id,end_reason,workers_reaped=None):
    e,s,cfg,binding=prepared
    events=event_records(directory/'events.jsonl')
    observations={};report=None;evaluation=None;received=None;first_raw=None;first_confirmed=None;last_time=0.
    for record in events:
        last_time=max(last_time,record['sim_time_s']);kind=record['event_type'];data=record['payload']
        if kind=='RAW_DETECTION' and first_raw is None:first_raw=record['sim_time_s']
        if kind=='CONFIRMED_DETECTION':
            if first_confirmed is None:first_confirmed=record['sim_time_s']
            observation=Observation(**data);observations[observation.observation_id]=observation
        if kind=='TARGET_REPORT' and received is None:
            report=data['report'];received=record['sim_time_s']
            evaluation=evaluate_report(report,observations,received,e,s)
        if kind=='REPORT_EVALUATED' and evaluation is not None:evaluation=data
    correct=bool(evaluation and evaluation['report_correct'])
    partial_length=None
    if (directory/'trajectory.csv').exists():
        previous=tuple(e['start_pose'][k] for k in ('x_m','y_m'))
        with (directory/'trajectory.csv').open(encoding='utf8',newline='') as stream:
            for row in csv.DictReader(stream):
                try:
                    pose=(float(row['x_m']),float(row['y_m']));when=float(row['sim_time_s'])
                    import math
                    if not all(math.isfinite(v) for v in (*pose,when)):break
                except (TypeError,ValueError,KeyError):break
                last_time=max(last_time,when)
                partial_length=(partial_length or 0.)+math.dist(previous,pose);previous=pose
    # A missing atomic result is an execution error, even if the worker emitted
    # a tentative terminal event. Preserve raw logs and make recovery explicit.
    result=dict(stage='P1b.2',run_id=run_id,episode_id=e['episode_id'],attempt_id=attempt_id,
                end_reason=end_reason,report=report,report_correct=correct,mission_success=False,
                search_found=first_confirmed is not None,first_raw_s=first_raw,first_confirmed_s=first_confirmed,
                report_submitted_s=received,stop_verified=False,stop_verified_s=None,
                position_error_m=evaluation.get('position_error_m') if evaluation else None,
                elapsed_sim_s=last_time,path_length_m=partial_length,elapsed_wall_s=None,trajectory_complete=False,
                workers_reaped=workers_reaped,perception_mode='oracle',n_false_reports=int(received is not None and not correct),
                recovery='supervisor lost before atomic result')
    directory.mkdir(parents=True,exist_ok=True)
    atomic(directory/'recovery.json',encode(dict(reason=end_reason,received_events=len(events),raw_logs_preserved=True)))
    atomic(directory/'result.json',encode(result))
    return result


def committed_hashes(directory):
    return {name:digest((directory/name).read_bytes()) for name in ('result.json','run_manifest.json','events.jsonl','trajectory.csv','workers.json','recovery.json') if (directory/name).is_file()}


def attempts(journal):
    state={}
    for record in journal.records:
        if record['kind']=='START':
            key=record['episode_id'],record['attempt_id']
            if type(key[1]) is not int or key[1]<1 or record['path']!=f'attempts/{key[0]}/attempt-{key[1]:04d}':raise ValueError('invalid attempt reservation path')
            if key in state:raise ValueError('duplicate START reservation')
            state[key]=dict(record,finished=None)
        elif record['kind']=='FINISH':
            key=record['episode_id'],record['attempt_id']
            if key not in state or state[key]['finished'] is not None:raise ValueError('duplicate/unreserved FINISH')
            state[key]['finished']=record
    return state


def csv_bytes(rows,columns):
    stream=io.StringIO(newline='');writer=csv.DictWriter(stream,fieldnames=columns,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    return stream.getvalue().encode('utf8')


def project(directory,journal,prepared,selected):
    rows=[];global_events=[];trajectories=[]
    for (eid,number),item in attempts(journal).items():
        folder=directory/item['path'];result=load_json(folder/'result.json') if item['finished'] else {}
        e,s,cfg,binding=prepared[eid]
        row=dict(episode_id=eid,attempt_id=number,scenario_id=e['scenario_id'],environment_id=e['environment_id'],
                 layout_hash=e['layout_hash'],layout_family_id=e['layout_family_id'],split=e['split'],seed=e['seed'],
                 fixture_hash=binding['fixture_hash'],end_reason='RUNNING',report_correct=None,mission_success=None)
        row.update(result)
        records=event_records(folder/'events.jsonl')
        if result.get('recovery'):
            records=[dict(r,event_type='UNCOMMITTED_EPISODE_END') if r['event_type']=='EPISODE_END' else r for r in records]
            for kind in ('ERROR','EPISODE_END'):
                records.append(dict(schema_version='t1-0.2',run_id=item['run_id'],episode_id=eid,attempt_id=number,
                    event_seq=len(records)+1,step_index=records[-1]['step_index'] if records else 0,
                    sim_time_s=result['elapsed_sim_s'],event_type=kind,source='recovery',payload={'end_reason':result['end_reason']}))
        global_events.extend(records)
        row.update(goals_selected=sum(r['event_type']=='GOAL_SELECTED' for r in records),fallback_goals=None,
                   first_stalled_s=next((r['sim_time_s'] for r in records if r['event_type']=='POLICY_STALLED'),None),
                   unresolved_waypoints=None)
        stall_start=None;stalled=0.
        for record in records:
            if record['event_type']=='MISSION_STATE':
                if record['payload']['state']=='POLICY_STALLED' and stall_start is None:stall_start=record['sim_time_s']
                elif record['payload']['state']!='POLICY_STALLED' and stall_start is not None:
                    stalled+=record['sim_time_s']-stall_start;stall_start=None
        if stall_start is not None and result.get('elapsed_sim_s') is not None:stalled+=max(0.,result['elapsed_sim_s']-stall_start)
        row.update(stalled_sim_s=stalled if result else None,monitor_interventions=sum(r['event_type']=='MONITOR_INTERVENTION' for r in records))
        rows.append(row)
        if (folder/'trajectory.csv').exists():
            with (folder/'trajectory.csv').open(encoding='utf8',newline='') as stream:
                trajectories.extend(dict(run_id=item['run_id'],episode_id=eid,attempt_id=number,**r) for r in csv.DictReader(stream))
    primary=[r for r in rows if r['attempt_id']==1]
    columns=['episode_id','attempt_id','scenario_id','environment_id','layout_hash','layout_family_id','split','seed','fixture_hash',
             'end_reason','report_correct','mission_success','search_found','elapsed_sim_s','elapsed_wall_s','path_length_m','trajectory_complete',
             'n_false_reports','goals_selected','fallback_goals','first_stalled_s','stalled_sim_s','unresolved_waypoints','monitor_interventions']
    atomic(directory/'attempts.csv',csv_bytes(rows,columns));atomic(directory/'episodes.csv',csv_bytes(primary,columns))
    atomic(directory/'events.jsonl',b''.join(encode(r)+b'\n' for r in global_events))
    trajectory_columns=['run_id','episode_id','attempt_id','step_index','sim_time_s','x_m','y_m','yaw_rad','vx_requested','vy_requested','wz_requested',
                        'vx_monitor','vy_monitor','wz_monitor','vx_executed','vy_executed','wz_executed','monitor_state','runner_state']
    atomic(directory/'trajectory.csv',csv_bytes(trajectories,trajectory_columns))
    def group(records):
        positive=[r for r in records if prepared[r['episode_id']][0]['targets']]
        negative=[r for r in records if not prepared[r['episode_id']][0]['targets']]
        complete=[r for r in records if r['end_reason']!='RUNNING']
        return dict(started=len(records),completed=len(complete),positive=len(positive),negative=len(negative),
                    report_correct=sum(r.get('report_correct') is True for r in positive),
                    mission_success=sum(r.get('mission_success') is True for r in positive),
                    ReportSuccess=sum(r.get('report_correct') is True for r in positive)/len(positive) if positive and all(r['end_reason']!='RUNNING' for r in positive) else None,
                    SafeMissionSuccess=sum(r.get('mission_success') is True for r in positive)/len(positive) if positive and all(r['end_reason']!='RUNNING' for r in positive) else None,
                    negative_false_alarm_rate=sum(bool(r.get('n_false_reports')) for r in negative)/len(negative) if negative and all(r['end_reason']!='RUNNING' for r in negative) else None,
                    execution_errors=sum(r['end_reason'] in ERRORS for r in records),end_reasons=dict(Counter(r['end_reason'] for r in records)))
    summary=dict(stage='P1b.2',purpose='runner_acceptance_not_research',primary_attempt=1,planned=len(selected),interrupted=False,
                 attempts=len(rows),overall=group(primary),scenarios={sid:group([r for r in primary if r['scenario_id']==sid]) for sid in ('S1','S2','S3','S4','S5')},
                 unsupported={'SPL':None,'oracle_coverage':None,'reason':'not implemented'},journal_head=journal.head)
    atomic(directory/'summary.json',encode(summary))
    text='# Runner acceptance summary (not research results)\n\nPrimary attempts only; retries never replace primary failures.\n\n'
    text+='| Scenario | Started | Completed | Runtime errors | End reasons |\n| --- | ---: | ---: | ---: | --- |\n'
    for sid,g in summary['scenarios'].items():text+=f"| {sid} | {g['started']} | {g['completed']} | {g['execution_errors']} | {g['end_reasons']} |\n"
    atomic(directory/'summary.md',text.encode('utf8'))
    return summary


def run_batch(catalog,bundle,output,policy='search-mission-smoke',episode_ids=None,seed=None,max_episodes=None,
              resume=False,retry_errors=False,policy_overrides=None,_hook=None):
    if retry_errors and not resume:raise ValueError('retry-errors requires resume')
    prepared=verified_corpus(catalog,bundle,require_all=True)  # Before selection, including resume.
    selected=list(prepared)
    if episode_ids is not None:
        if len(set(episode_ids))!=len(episode_ids) or set(episode_ids)-set(selected):raise ValueError('unknown/duplicate episode filter')
        selected=[eid for eid in selected if eid in episode_ids]
    if seed is not None:selected=[eid for eid in selected if prepared[eid][0]['seed']==seed]
    if max_episodes is not None:
        if type(max_episodes) is not int or max_episodes<=0:raise ValueError('invalid max-episodes')
        selected=selected[:max_episodes]
    if not selected:raise ValueError('empty selection')
    overrides=policy_overrides or {}
    if set(overrides)-set(selected) or any(p not in POLICIES for p in [policy,*overrides.values()]):raise ValueError('invalid policy')
    identity=dict(protocol='t1-0.2',selection=selected,policies={eid:overrides.get(eid,policy) for eid in selected},
                  fixtures={eid:prepared[eid][3] for eid in prepared},runtime=dict(python=sys.version,platform=platform.platform(),
                  dependencies={n:version(n) for n in ('numpy','PyYAML')}))
    destination=output_path(output)
    if not resume:
        if destination.exists():raise ValueError('output already exists')
        destination.parent.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='.batch-init-',dir=destination.parent) as staging:
            if not Path(staging).resolve().is_relative_to(ROOT.resolve()):raise ValueError('staging outside repository')
            sha=subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,capture_output=True,text=True).stdout.strip()
            dirty=bool(subprocess.run(['git','status','--porcelain'],cwd=ROOT,capture_output=True,text=True).stdout)
            registry={eid:{k:prepared[eid][0][k] for k in ('environment_id','layout_hash','layout_family_id','split')} for eid in prepared}
            atomic(Path(staging)/'run_manifest.json',encode(dict(stage='P1b.2',protocol='t1-0.2',run_id='batch-'+uuid.uuid4().hex[:12],
                git_sha=sha,git_dirty=dirty,registry=registry,config_hashes={eid:prepared[eid][2]['hashes'] for eid in prepared},identity=identity)))
            os.rename(staging,destination)
    elif not destination.is_dir():raise ValueError('resume output missing')
    with run_lock(destination):
        manifest=load_json(destination/'run_manifest.json')
        if manifest['identity']!=identity:raise ValueError('resume fingerprints/selection/runtime/policy mismatch')
        journal=Journal(destination/'journal.jsonl')
        interrupted={(r['episode_id'],r['attempt_id']) for r in journal.records if r['kind']=='INTERRUPT'}
        for key,item in attempts(journal).items():
            folder=destination/item['path']
            if any(p.is_symlink() for p in (folder,*folder.parents) if p.is_relative_to(destination)):raise ValueError('attempt symlink forbidden')
            if item['finished']:
                if committed_hashes(folder)!=item['finished']['hashes']:raise ValueError('completed attempt modified')
                continue
            if (folder/'workers.json').exists():
                pids=load_json(folder/'workers.json');deadline=time.monotonic()+3
                while any(process_alive(pid) for pid in pids.values()) and time.monotonic()<deadline:time.sleep(.05)
                if any(process_alive(pid) for pid in pids.values()):raise ValueError('previous workers still alive; cannot safely resume')
            if (folder/'result.json').is_file():
                result=load_json(folder/'result.json')
                if result['episode_id']!=key[0] or result['attempt_id']!=key[1] or result['run_id']!=manifest['run_id']:raise ValueError('orphan result identity mismatch')
            else:result=recover_result(folder,prepared[key[0]],manifest['run_id'],key[1],'INTERRUPTED' if key in interrupted else 'CRASH',True if (folder/'workers.json').exists() else None)
            journal.append('FINISH',episode_id=key[0],attempt_id=key[1],hashes=committed_hashes(folder),recovered=True)
        summary=project(destination,journal,prepared,selected)
        for eid in selected:
            existing=[(key,item) for key,item in attempts(journal).items() if key[0]==eid]
            if existing:
                last_key,last=max(existing,key=lambda pair:pair[0][1])
                previous=load_json(destination/last['path']/'result.json')
                if not retry_errors or previous['end_reason'] not in ERRORS:continue
                number=last_key[1]+1
            else:number=1
            relative=f'attempts/{eid}/attempt-{number:04d}'
            item=journal.append('START',episode_id=eid,attempt_id=number,path=relative,run_id=manifest['run_id'])
            try:
                if _hook:_hook('reserved',item)
                def started(pids):
                    atomic(destination/relative/'workers.json',encode(pids))
                    if _hook:_hook('workers',item)
                result=_execute_single(prepared[eid],(destination/relative).relative_to(ROOT).as_posix(),
                    identity['policies'][eid],run_id=manifest['run_id'],attempt_id=number,started=started,
                    checkpoint=(lambda:_hook('running',item)) if _hook else None)
                if _hook:_hook('result',item)
                journal.append('FINISH',episode_id=eid,attempt_id=number,hashes=committed_hashes(destination/relative),recovered=False)
                summary=project(destination,journal,prepared,selected)
                if _hook:_hook('committed',item)
                if result['end_reason']=='INTERRUPTED':
                    summary['interrupted']=True;break
            except KeyboardInterrupt:
                journal.append('INTERRUPT',episode_id=eid,attempt_id=number)
                summary=project(destination,journal,prepared,selected);summary['interrupted']=True;break
            except Exception:
                # IO/journal failure is a batch failure, not an algorithm outcome.
                # Reservation remains durable for the next resume.
                project(destination,journal,prepared,selected)
                raise
        atomic(destination/'summary.json',encode(summary))
        return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('catalog','bundle','output'):parser.add_argument('--'+name,required=True)
    parser.add_argument('--policy',choices=POLICIES,default='search-mission-smoke')
    parser.add_argument('--episode-id',action='append');parser.add_argument('--seed',type=int)
    parser.add_argument('--max-episodes',type=int);parser.add_argument('--resume',action='store_true');parser.add_argument('--retry-errors',action='store_true')
    a=parser.parse_args()
    try:
        summary=run_batch(a.catalog,a.bundle,a.output,a.policy,a.episode_id,a.seed,a.max_episodes,a.resume,a.retry_errors)
        print(json.dumps(dict(stage='P1b.2',started=summary['overall']['started'],attempts=summary['attempts'],end_reasons=summary['overall']['end_reasons'])))
        return 130 if summary['interrupted'] else (1 if summary['overall']['execution_errors'] else 0)
    except (ValueError,OSError,KeyError,TypeError,OverflowError,yaml.YAMLError) as error:print(json.dumps({'error':str(error)}));return 2


if __name__=='__main__':raise SystemExit(main())
