"""P1b.1 single-episode supervisor and evaluator worker; no batch/resume."""
import argparse
import copy
import csv
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import uuid
from dataclasses import asdict
from importlib.metadata import version
import yaml

from .artifacts import load_catalog,build_bundle,output_path,encode,digest
from .schema import ROOT
from .policy import SensorFrame,policy_worker,POLICIES,policy_identity
from .selectors import OFFICIAL_POLICIES,selection_diagnostics
from .oracle import Oracle
from .simulator import lidar,world_boxes
from .evaluator import evaluate_report,finite,safety,completed_report_reason
from .witness import euler


def verified_corpus(catalog_path,bundle_path,require_all=False):
    """Full corpus first; do not trust a stored PASS or a self-consistent manifest."""
    catalog,suites,cfgs,witnesses=load_catalog(catalog_path)
    expected,_=build_bundle(catalog,suites,cfgs,witnesses)
    directory=output_path(bundle_path)
    if not directory.is_dir():raise ValueError('certificate bundle missing')
    if any(p.is_symlink() for p in directory.rglob('*')):raise ValueError('bundle symlink forbidden')
    actual={p.relative_to(directory).as_posix():p.read_bytes() for p in directory.rglob('*') if p.is_file()}
    if actual!=expected:raise ValueError('stale/tampered/incomplete certificate bundle; regenerate validation first')
    found={}
    for suite,cfg in zip(suites,cfgs):
        for e in suite['episodes']:
            cert=json.loads(expected[e['episode_id']+'/fixture_validation.json'])
            if cert['status']!='PASS':
                if require_all:raise ValueError('full corpus is not PASS: '+e['episode_id'])
                continue
            found[e['episode_id']]=(*copy.deepcopy((e,suite['shared'],cfg)),
                dict(fixture_hash=cert['fixture_hash'],evidence_hash=cert['evidence_hash'],verifier_hash=cert['verifier_hash'],bundle_hash=digest(expected['manifest.json'])))
    return found


def verified_input(catalog_path,bundle_path,episode_id):
    found=verified_corpus(catalog_path,bundle_path)
    if episode_id not in found:raise ValueError('unknown episode or not certified PASS')
    return found[episode_id]


def _clean(value):
    if type(value) is float and not math.isfinite(value):return None
    if type(value) in (str,int,float,bool) or value is None:return value
    if isinstance(value,(list,tuple)):return [_clean(v) for v in value]
    if isinstance(value,dict):return {str(k):_clean(v) for k,v in value.items()}
    return str(type(value).__name__)


def simulate(connection,policy,e,s,cfg,fault,lifeline=None):
    from .lifecycle import watch_parent
    watch_parent(lifeline)
    from collision_monitor_ros1 import Config,Monitor,scan_points
    cc=Config(**{k:cfg['monitor'][k] for k in Config.DEFAULTS})
    monitor=Monitor(cc);boxes=world_boxes(e);oracle=Oracle(s['camera'],s['dt_s'])
    pose=tuple(e['start_pose'][k] for k in ('x_m','y_m','yaw_rad'))
    state='SEARCHING';report=None;evaluation=dict(valid=False,matched=False,report_correct=False,position_error_m=None)
    observations={};submitted=None;hold=0;monitor_state='IDLE';reason=None;elapsed=0.;k=0;path_length=0.;seen=False
    last_mission=None;last_goal=None;last_monitor=None;first_raw=None;first_confirmed=None;last_selector=None
    def event(kind,payload=None,when=None):
        source=('oracle' if kind in ('RAW_DETECTION','CONFIRMED_DETECTION') else
                'evaluator' if kind in ('REPORT_EVALUATED','COLLISION','OUT_OF_BOUNDS') else
                'monitor' if kind in ('MONITOR_STATE','MONITOR_INTERVENTION') else
                'policy' if kind in ('MISSION_STATE','GOAL_SELECTED','POLICY_STALLED','SELECTOR_DECISION','SELECTOR_STATUS','WAYPOINT_OUTCOME') else 'runner')
        connection.send(('event',dict(event_type=kind,step_index=k,sim_time_s=k*s['dt_s'] if when is None else when,source=source,payload=_clean(payload or {}))))
    def transition(next_state,when=None):
        nonlocal state
        state=next_state;event('RUNNER_STATE',{'state':state},when)
    try:
        event('EPISODE_START',{'state':state})
        ready=policy.recv()
        if not ready.get('ready'):raise RuntimeError('policy startup failed')
        for k in range(round(e['time_limit_s']/s['dt_s'])):
            now=k*s['dt_s'];event('SENSE',{'state':state})
            ranges=lidar(pose,boxes,cc)
            monitor.set_scan(scan_points(ranges,-math.pi,2*math.pi/720,.05,6.,cc),now)
            raw,observation=oracle.step(k,pose,e['targets'][0] if e['targets'] else None,e['obstacles'])
            if raw:
                if first_raw is None:first_raw=now
                event('RAW_DETECTION')
            if observation is not None:
                observations[observation.observation_id]=observation
                if observation.last_confirmed_seen_s==now:
                    if first_confirmed is None:first_confirmed=now
                    seen=True;event('CONFIRMED_DETECTION',asdict(observation))
            command=(0.,0.,0.)
            if state=='SEARCHING':
                policy.send(SensorFrame(k,tuple(pose),ranges,observation,monitor_state))
                response=policy.recv()  # Supervisor's wall watchdog bounds all policy work.
                if 'error' in response:
                    reason=response['error'];event('ERROR',response);break
                command=response['command']
                for kind,payload in response.get('selector_events',()):event(kind,payload)
                if response.get('selector_stats') is not None and response['selector_stats']!=last_selector:
                    last_selector=response['selector_stats'];event('SELECTOR_STATUS',last_selector)
                if response['planner_barrier']:event('PLANNER_BARRIER',{'application':'next policy tick'})
                if response['mission']!=last_mission:
                    last_mission=response['mission'];event('MISSION_STATE',{'state':last_mission,'covered_cells':response['covered_cells']})
                    if last_mission=='POLICY_STALLED':event('POLICY_STALLED')
                if response['goal']!=last_goal:
                    last_goal=response['goal']
                    if last_goal is not None:event('GOAL_SELECTED',{'goal':last_goal})
                if response['report'] is not None:
                    report=copy.deepcopy(response['report']);submitted=k
                    transition('REPORT_SUBMITTED');event('TARGET_REPORT',{'report':report,'received_sim_s':now})
                    evaluation=evaluate_report(report,observations,now,e,s)
                    event('REPORT_EVALUATED',evaluation)
                    transition('VERIFY_STOP');command=(0.,0.,0.)
                    if fault=='crash-after-report':raise RuntimeError('injected crash after frozen report')
            if not finite(command) or len(command)!=3:
                reason='INVALID_STATE';event('ERROR',{'reason':'nonfinite/invalid requested command'});break
            monitor.set_command(*command,now=now)
            checked,monitor_state=monitor.evaluate(now)
            if monitor_state!=last_monitor:
                event('MONITOR_STATE',{'state':monitor_state})
                if monitor_state not in ('PASS','IDLE','LIMIT'):event('MONITOR_INTERVENTION',{'state':monitor_state})
                last_monitor=monitor_state
            executed=checked
            if fault=='actuator-collision' and state=='VERIFY_STOP':executed=(20.,0.,0.)
            if fault=='actuator-bounds':executed=(0.,100.,0.)
            if fault=='actuator-drift' and state=='VERIFY_STOP':executed=(.02,0.,0.)
            if fault=='actuator-pulse' and state=='VERIFY_STOP' and k-submitted==1:executed=(.02,0.,0.)
            if not finite(executed):reason='INVALID_STATE';break
            end,unwrapped=euler(pose,executed,s['dt_s'])
            if not finite(end):reason='INVALID_STATE';break
            unsafe,checks=safety(pose,unwrapped,e,s,cfg,boxes)
            elapsed=(k+1)*s['dt_s'];path_length+=math.dist(pose[:2],end[:2])
            connection.send(('trajectory',dict(step_index=k,sim_time_s=elapsed,x_m=end[0],y_m=end[1],yaw_rad=end[2],
                vx_requested=command[0],vy_requested=command[1],wz_requested=command[2],
                vx_monitor=checked[0],vy_monitor=checked[1],wz_monitor=checked[2],
                vx_executed=executed[0],vy_executed=executed[1],wz_executed=executed[2],monitor_state=monitor_state,runner_state=state)))
            pose=end
            if unsafe:
                reason=unsafe;event('COLLISION' if unsafe=='COLLISION' else 'OUT_OF_BOUNDS',checks,elapsed);break
            if state=='VERIFY_STOP':
                stop=s['stop_check']
                slow=math.hypot(executed[0],executed[1])<=stop['linear_speed_m_s'] and abs(executed[2])<=stop['angular_speed_rad_s']
                hold=hold+1 if slow else 0
                if hold>=stop['hold_steps']:
                    event('STOP_VERIFIED',{'hold_steps':hold},elapsed)
                    reason=completed_report_reason(evaluation,pose,e,s,cfg);break
            if k+1>=round(e['time_limit_s']/s['dt_s']):reason='TIMEOUT';break
            if state=='VERIFY_STOP' and k+1-submitted>=round(s['stop_check']['timeout_s']/s['dt_s']):
                reason='STOP_NOT_VERIFIED';break
        reason=reason or 'TIMEOUT'
    except (EOFError,BrokenPipeError) as error:
        reason='CRASH';event('ERROR',{'type':type(error).__name__})
    except Exception as error:
        reason='CRASH';event('ERROR',{'type':type(error).__name__,'detail':str(error)})
    finally:
        transition('FINISHED',elapsed);event('EPISODE_END',{'end_reason':reason,'state':state},elapsed)
        connection.send(('result',dict(end_reason=reason,report_correct=evaluation['report_correct'],mission_success=reason=='SUCCESS_REPORT',
             report=_clean(report),position_error_m=evaluation['position_error_m'],elapsed_sim_s=elapsed,path_length_m=path_length,
             search_found=seen,stop_verified=hold>=s['stop_check']['hold_steps'],
             first_raw_s=first_raw,first_confirmed_s=first_confirmed,report_submitted_s=None if submitted is None else submitted*s['dt_s'],
             stop_verified_s=elapsed if hold>=s['stop_check']['hold_steps'] else None,
             trajectory_complete=reason not in ('CRASH','PLANNER_TIMEOUT','INVALID_STATE'))))
        connection.close();policy.close()


def run_single(catalog,bundle,episode_id,output,policy_id='search-mission-smoke',fault=None):
    prepared=verified_input(catalog,bundle,episode_id)
    return _execute_single(prepared,output,policy_id,fault)


def _execute_single(prepared,output,policy_id='search-mission-smoke',fault=None,run_id=None,attempt_id=1,started=None,checkpoint=None):
    """Internal evaluator-side API; only a freshly verified corpus may prepare it."""
    if policy_id not in POLICIES:raise ValueError('unsupported policy ID')
    if policy_id=='test-crash-after-report' and fault is None:fault='crash-after-report'
    if fault not in (None,'actuator-collision','actuator-bounds','actuator-drift','actuator-pulse','crash-after-report'):raise ValueError('unknown test-only fault')
    e,s,cfg,binding=copy.deepcopy(prepared);episode_id=e['episode_id']
    destination=output_path(output)
    if destination.exists():raise ValueError('output already exists; P1b.1 has no resume')
    destination.mkdir(parents=True,exist_ok=False)
    public={k:copy.deepcopy(cfg[k]) for k in ('navigation','monitor','search')}
    public.update(bounds=tuple(e['bounds_m']),camera=copy.deepcopy(s['camera']),dt_s=s['dt_s'],standoff=s['person_standoff_m'],planner_wall_timeout_s=s['planner_wall_timeout_s'],b0_spacing_m=s['b0_spacing_m'])
    run_id=run_id or 'single-'+uuid.uuid4().hex[:12]
    git_sha=subprocess.run(['git','rev-parse','HEAD'],cwd=ROOT,text=True,capture_output=True).stdout.strip()
    dirty=bool(subprocess.run(['git','status','--porcelain'],cwd=ROOT,text=True,capture_output=True).stdout)
    manifest=dict(stage='P1b.1',protocol='t1-0.2',run_id=run_id,episode_id=episode_id,policy_id=policy_id,policy_identity=policy_identity(policy_id,s,cfg),
                  git_sha=git_sha,git_dirty=dirty,python=sys.version,dependencies={name:version(name) for name in ('numpy','PyYAML')},dt_s=s['dt_s'],seed=e['seed'],config=cfg,
                  environment_id=e['environment_id'],layout_hash=e['layout_hash'],layout_family_id=e['layout_family_id'],split=e['split'],
                  perception_mode='oracle',planner_mode='async_next_tick_barrier',test_fault=fault,
                  lidar_model=dict(beams=720,range_min_m=.05,range_max_m=6.,angle_min_rad=-math.pi,angle_increment_rad=2*math.pi/720),**binding)
    (destination/'run_manifest.json').write_bytes(encode(manifest))
    ctx=mp.get_context('spawn');p_end,s_end=ctx.Pipe();rx,tx=ctx.Pipe(duplex=False)
    policy_life,policy_owner=ctx.Pipe(duplex=False);sim_life,sim_owner=ctx.Pipe(duplex=False)
    policy=ctx.Process(target=policy_worker,args=(p_end,public,policy_id,policy_life),name='t1-policy')
    worker=ctx.Process(target=simulate,args=(tx,s_end,e,s,cfg,fault,sim_life),name='t1-simulator')
    result=None;sequence=0;last_time=0.;last_step=0;report_correct=False;last_eval={};last_report=None;progress={};selector_records=[]
    events=(destination/'events.jsonl').open('w',encoding='utf8')
    trajectory=(destination/'trajectory.csv').open('w',encoding='utf8',newline='');writer=None
    def emit(event):
        nonlocal sequence,last_time,last_step,report_correct,last_eval,last_report
        sequence+=1;last_time=max(last_time,event['sim_time_s']);last_step=event['step_index']
        if event['event_type']=='REPORT_EVALUATED':last_eval=event['payload'];report_correct=last_eval['report_correct']
        if event['event_type']=='TARGET_REPORT':last_report=event['payload']['report']
        if event['event_type']=='SELECTOR_STATUS':selector_records.append(event)
        timestamps={'RAW_DETECTION':'first_raw_s','CONFIRMED_DETECTION':'first_confirmed_s','TARGET_REPORT':'report_submitted_s','STOP_VERIFIED':'stop_verified_s'}
        if event['event_type'] in timestamps:progress.setdefault(timestamps[event['event_type']],event['sim_time_s'])
        record=dict(schema_version='t1-0.2',run_id=run_id,episode_id=episode_id,attempt_id=attempt_id,event_seq=sequence,**event)
        events.write(json.dumps(_clean(record),allow_nan=False)+'\n');events.flush()
        if event['event_type'] in ('TARGET_REPORT','REPORT_EVALUATED','EPISODE_END'):os.fsync(events.fileno())
        if checkpoint is not None and event['event_type']=='SENSE':checkpoint()
    start=time.monotonic()
    try:
        policy.start();worker.start();p_end.close();s_end.close();tx.close();policy_life.close();sim_life.close()
        if started is not None:started(dict(policy_pid=policy.pid,simulator_pid=worker.pid))
        while result is None:
            if time.monotonic()-start>s['episode_wall_timeout_s']:
                result=dict(end_reason='WALL_TIMEOUT');break
            if rx.poll(.05):
                try:kind,payload=rx.recv()
                except EOFError:result=dict(end_reason='CRASH');break
                if kind=='event':emit(payload)
                elif kind=='trajectory':
                    if writer is None:writer=csv.DictWriter(trajectory,fieldnames=list(payload));writer.writeheader()
                    writer.writerow(payload);trajectory.flush();last_time=payload['sim_time_s']
                elif kind=='result':result=payload
            elif not worker.is_alive():
                result=dict(end_reason='CRASH')
    except KeyboardInterrupt:result=dict(end_reason='INTERRUPTED')
    finally:
        for process in (worker,policy):
            if process.pid is not None:
                process.join(timeout=.2)
                if process.is_alive():process.terminate();process.join(timeout=2.)
                if process.is_alive():process.kill();process.join(timeout=2.)
        rx.close()
        policy_owner.close();sim_owner.close()
        if result is None:result=dict(end_reason='CRASH')
        if 'trajectory_complete' not in result:
            result.update(report_correct=report_correct,mission_success=False,report=last_report,
                          position_error_m=last_eval.get('position_error_m'),elapsed_sim_s=last_time,trajectory_complete=False)
            emit(dict(event_type='ERROR',step_index=last_step,sim_time_s=last_time,source='supervisor',payload={'end_reason':result['end_reason']}))
            emit(dict(event_type='RUNNER_STATE',step_index=last_step,sim_time_s=last_time,source='supervisor',payload={'state':'FINISHED'}))
            emit(dict(event_type='EPISODE_END',step_index=last_step,sim_time_s=last_time,source='supervisor',payload={'end_reason':result['end_reason']}))
        result.update(stage='P1b.1',episode_id=episode_id,run_id=run_id,attempt_id=attempt_id,elapsed_wall_s=time.monotonic()-start,workers_reaped=not worker.is_alive() and not policy.is_alive())
        result.update(perception_mode='oracle',n_false_reports=int(result.get('report') is not None and not result.get('report_correct',False)))
        for name in ('first_raw_s','first_confirmed_s','report_submitted_s','stop_verified_s'):result.setdefault(name,progress.get(name))
        result.setdefault('search_found',result['first_confirmed_s'] is not None)
        result.setdefault('path_length_m',None)
        result.update(selection_diagnostics(selector_records,result.get('elapsed_sim_s',last_time),unknown=policy_id in OFFICIAL_POLICIES))
        events.close();trajectory.close()
        from .journal import atomic
        atomic(destination/'result.json',encode(result))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--catalog',required=True);parser.add_argument('--bundle',required=True)
    parser.add_argument('--episode-id',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--policy',choices=POLICIES,default='search-mission-smoke')
    args=parser.parse_args()
    try:
        result=run_single(args.catalog,args.bundle,args.episode_id,args.output,args.policy)
        print(json.dumps(result,allow_nan=False,indent=2))
        if result['end_reason']=='INTERRUPTED':return 130
        return 1 if result['end_reason'] in ('CRASH','PLANNER_TIMEOUT','WALL_TIMEOUT','INVALID_STATE') else 0
    except (ValueError,OSError,KeyError,TypeError,OverflowError,yaml.YAMLError) as error:
        print(json.dumps({'error':str(error)}));return 2


if __name__=='__main__':raise SystemExit(main())
