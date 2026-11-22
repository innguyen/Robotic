"""Evaluator-side, versioned reduction of immutable raw attempts. No policy imports.

Protocol monitor_interventions retains per-state entries. Continuous episodes are
an explicitly named extension, not a reinterpretation of the protocol field.
"""
import csv
import io
import json
import math
from pathlib import Path
from collections import Counter

from .artifacts import digest, encode, output_path
from .schema import load_json,pairs
from .oracle import Observation
from .evaluator import evaluate_report, completed_report_reason, safety
from .simulator import world_boxes
from .witness import euler

VERSION = 't1-metrics-1'
MONITOR_VERSION = 'monitor-episodes-1'
ERRORS = frozenset(('CRASH','PLANNER_TIMEOUT','WALL_TIMEOUT','INTERRUPTED','INVALID_STATE'))
NORMAL = frozenset(('PASS','IDLE','LIMIT'))
LIMITED = frozenset(('SLOWDOWN','APPROACH'))
STOPPED = frozenset(('WAIT_SCAN','WAIT_COMMAND','INVALID_POINTS','INVALID_COMMAND','STALE_SCAN',
    'STALE_COMMAND','UNOBSERVED_DIRECTION','STOP_ZONE','PREDICTED_COLLISION',
    'SCAN_FRAME_MISMATCH','STALE_SCAN_STAMP'))
END_REASONS = ERRORS | frozenset(('SUCCESS_REPORT','FALSE_REPORT','INVALID_REPORT','APPROACH_NOT_SATISFIED',
    'STOP_NOT_VERIFIED','TIMEOUT','SEARCH_EXHAUSTED','COLLISION','OUT_OF_BOUNDS'))


def identity():
    root = Path(__file__).parent
    return dict(version=VERSION, monitor_version=MONITOR_VERSION,
        sources={name:digest((root/name).read_bytes()) for name in
            ('metrics.py','evaluator.py','geometry.py','scenarios.py','oracle.py','witness.py','simulator.py')})


def close(a, b):
    """Serialization/consistency tolerance ONLY; never a grading tolerance."""
    return type(a) in (int,float) and type(b) in (int,float) and math.isclose(a,b,rel_tol=1.e-10,abs_tol=1.e-10)


def monitor_metrics(events, trajectory, dt, complete):
    entries = []; previous = None
    for event in events:
        if event['event_type'] != 'MONITOR_STATE':continue
        state = event['payload']['state']
        if state not in NORMAL | LIMITED | STOPPED:raise ValueError('unknown monitor state: '+str(state))
        if state != previous:
            if state not in NORMAL:entries.append((event['step_index'],state))
            previous = state
    markers = [(e['step_index'],e['payload']['state']) for e in events if e['event_type']=='MONITOR_INTERVENTION']
    issues = [] if entries == markers else ['monitor intervention markers disagree with state entries']
    states_at = {e['step_index']:e['payload']['state'] for e in events if e['event_type']=='MONITOR_STATE'}
    if complete and any(index >= len(trajectory) for index in states_at):issues.append('unexecuted monitor state in complete attempt')
    state = None; previous = None; continuous = 0; changes = 0; transitions = 0
    durations = Counter(); state_episodes = Counter()
    for i, row in enumerate(trajectory):
        state = states_at.get(i,state)
        actual = row['monitor_state']
        if actual not in NORMAL | LIMITED | STOPPED:raise ValueError('unknown trajectory monitor state')
        if state != actual:issues.append('trajectory/MONITOR_STATE disagreement at '+str(i))
        durations[actual] += 1
        if previous is not None and previous != actual:transitions += 1
        if actual not in NORMAL:
            if previous is None or previous in NORMAL:continuous += 1
            elif previous != actual:changes += 1
            if previous != actual:state_episodes[actual] += 1
        previous = actual
    observed = dict(monitor_interventions=len(entries), monitor_continuous_episodes=continuous,
        monitor_intervention_state_changes=changes, monitor_state_transitions=transitions,
        monitor_intervention_s=sum(v for k,v in durations.items() if k not in NORMAL)*dt,
        monitor_state_seconds={k:v*dt for k,v in sorted(durations.items())},
        monitor_state_episodes=dict(sorted(state_episodes.items())),
        monitor_severity='stopped' if any(k in STOPPED for k in durations) else
                         'limited' if any(k in LIMITED for k in durations) else 'none')
    return dict(version=MONITOR_VERSION, complete=complete,
                **{k:v if complete else None for k,v in observed.items()}, observed=observed), issues


def fixed_grid_size(episode, spacing):
    """Independent cardinality, not the selector's assigned/status counter."""
    start = episode['start_pose']; yaw = start['yaw_rad']; c,s = math.cos(yaw),math.sin(yaw)
    a,b,aa,bb = episode['bounds_m']; x,y = start['x_m'],start['y_m']
    radius = math.ceil(max(math.hypot(px-x,py-y) for px in (a,aa) for py in (b,bb))/spacing)+1
    if (2*radius+1)**2 > 1000000:raise ValueError('UNVERIFIED: fixed-grid cardinality work limit')
    return sum((i != 0 or j != 0) and a <= x+spacing*(c*i-s*j) <= aa and b <= y+spacing*(s*i+c*j) <= bb
               for i in range(-radius,radius+1) for j in range(-radius,radius+1))


def selector_metrics(events, episode, shared, policy, elapsed, complete):
    initialized = False; initial_time = None; first = None; stalled_since = None; stalled = 0.
    goals = 0; fallback = 0; assigned = set(); reached = set(); issues = []
    total = fixed_grid_size(episode,shared['b0_spacing_m']) if policy == 'b0-fixed-order-v2' else None
    for event in events:
        kind, data, now = event['event_type'], event['payload'], event['sim_time_s']
        if kind == 'SELECTOR_DECISION' and data['selection'] is not None:
            selected = data['selection']; goals += 1; fallback += selected['kind']=='fallback'
            index = selected['waypoint_index']
            if index is not None:
                if type(index) is not int or total is None or not 0 <= index < total or index in assigned:
                    issues.append('duplicate/invalid fixed waypoint index')
                assigned.add(index)
        if kind == 'WAYPOINT_OUTCOME' and data['waypoint_index'] is not None:
            index = data['waypoint_index']
            if index not in assigned:issues.append('outcome without fixed selection')
            if data['outcome']=='reached':reached.add(index)
        if kind != 'SELECTOR_STATUS':continue
        if not initialized:initial_time = now
        initialized = True
        expected = dict(goals_selected=goals,fallback_goals=fallback,
                        unresolved_waypoints=None if total is None else total-len(reached))
        for name, value in expected.items():
            if data.get(name) != value:issues.append('selector status counter mismatch: '+name)
        if type(data.get('stalled')) is not bool:raise ValueError('invalid stalled status')
        if data['stalled'] and stalled_since is None:
            stalled_since = now
            if first is None:first = now
        elif not data['stalled'] and stalled_since is not None:
            stalled += now-stalled_since; stalled_since = None
    if stalled_since is not None:stalled += max(0.,elapsed-stalled_since)
    observed = dict(goals_selected=goals,fallback_goals=fallback,first_stalled_s=first,
                    stalled_sim_s=stalled,unresolved_waypoints=None if total is None else total-len(reached))
    verified = initialized and initial_time == 0. and complete
    return dict(complete=verified, **{k:v if verified else None for k,v in observed.items()},
                observed=observed if initialized else dict.fromkeys(observed)), issues


def reduce_records(resolved, manifest, events, trajectory, result):
    """Pure reducer; supplied raw rows must be the committed prefix of one attempt."""
    e,s,cfg = resolved['episode'],resolved['shared'],resolved['config']; dt = s['dt_s']
    for key in ('trajectory_complete','report_correct','mission_success','search_found'):
        if key in result and type(result[key]) is not bool:raise ValueError('result bool required: '+key)
    if type(result['attempt_id']) is not int or result['attempt_id']<1:raise ValueError('invalid result attempt')
    issues = []; previous_time = 0.
    for i, event in enumerate(events):
        if (type(event['event_seq']) is not int or type(event['attempt_id']) is not int or event['event_seq'] != i+1 or event['run_id'] != manifest['run_id'] or event['episode_id'] != e['episode_id']
                or event['attempt_id'] != result['attempt_id'] or event['schema_version'] != 't1-0.2'):
            raise ValueError('event identity/sequence mismatch')
        now = event['sim_time_s']
        if type(now) not in (int,float) or not math.isfinite(now) or now < previous_time:raise ValueError('invalid event time')
        if type(event['step_index']) is not int or event['step_index'] < 0:raise ValueError('invalid event step')
        if not (close(now,event['step_index']*dt) or close(now,(event['step_index']+1)*dt)):raise ValueError('event off simulated clock')
        previous_time = now
    if result['episode_id'] != e['episode_id'] or result['run_id'] != manifest['run_id']:raise ValueError('result identity mismatch')
    reason = result['end_reason']; recovered = bool(result.get('recovery'))
    if reason not in END_REASONS:raise ValueError('unknown end reason')
    if reason=='SEARCH_EXHAUSTED' and manifest['policy_id'] in ('b0-fixed-order-v2','b1-gain-distance-focus-v1'):
        issues.append('official T1 policy cannot declare SEARCH_EXHAUSTED')
    terminal = [r for r in events if r['event_type']=='EPISODE_END']
    if not recovered and (not terminal or terminal[-1]['payload']['end_reason'] != reason):issues.append('terminal/result disagreement')
    complete = bool(result.get('trajectory_complete')) and reason not in ERRORS and not recovered
    observations = {}; report = None; report_time = None; report_step = None; reported = False; evaluation = None
    first_raw = None; first_confirmed = None; stop_event_time = None
    for record in events:
        kind,data,now = record['event_type'],record['payload'],record['sim_time_s']
        if kind=='RAW_DETECTION' and first_raw is None:first_raw = now
        if kind=='CONFIRMED_DETECTION':
            observation = Observation(**data); observations[observation.observation_id] = observation
            if first_confirmed is None:first_confirmed = now
        if kind=='TARGET_REPORT' and not reported:
            reported = True; report = data['report']; report_time = now; report_step = record['step_index']
            evaluation = evaluate_report(report,observations,now,e,s)
            if now >= e['time_limit_s']:issues.append('report at/after deadline')
        if kind=='REPORT_EVALUATED' and evaluation is not None and data != evaluation:issues.append('logged report evaluation mismatch')
        if kind=='STOP_VERIFIED' and stop_event_time is None:stop_event_time = now
    evaluation = evaluation or dict(valid=False,matched=False,report_correct=False,position_error_m=None)
    pose = tuple(e['start_pose'][k] for k in ('x_m','y_m','yaw_rad'))
    length = 0.; hold = 0; derived_stop = None; unsafe = None; boxes = world_boxes(e)
    for i,row in enumerate(trajectory):
        if int(row['step_index']) != i or not close(float(row['sim_time_s']),(i+1)*dt):raise ValueError('trajectory must be consecutive from zero')
        end = tuple(float(row[k]) for k in ('x_m','y_m','yaw_rad'))
        command = tuple(float(row[k]) for k in ('vx_executed','vy_executed','wz_executed'))
        if not all(math.isfinite(v) for v in (*end,*command)):raise ValueError('nonfinite trajectory')
        expected,unwrapped = euler(pose,command,dt)
        if not all(close(a,b) for a,b in zip(expected,end)):issues.append('Euler/trajectory disagreement at '+str(i))
        current_unsafe,_ = safety(pose,unwrapped,e,s,cfg,boxes)
        if current_unsafe and unsafe is None:unsafe = current_unsafe
        length += math.dist(pose[:2],end[:2]); pose = end
        if report_step is not None and i >= report_step and unsafe is None:
            check = s['stop_check']
            slow = math.hypot(command[0],command[1]) <= check['linear_speed_m_s'] and abs(command[2]) <= check['angular_speed_rad_s']
            hold = hold+1 if slow else 0
            if hold >= check['hold_steps'] and derived_stop is None:derived_stop = (i+1)*dt
    elapsed = max([len(trajectory)*dt]+[r['sim_time_s'] for r in events])
    if elapsed > e['time_limit_s']+1.e-10:issues.append('log beyond budget')
    if unsafe is not None and reason != unsafe:issues.append('geometry priority/result disagreement: '+unsafe)
    if stop_event_time is not None and (derived_stop is None or not close(stop_event_time,derived_stop)):issues.append('stop event/velocity disagreement')
    if complete:
        if derived_stop is not None and stop_event_time is None:issues.append('missing STOP_VERIFIED event')
        if unsafe is None and derived_stop is not None:
            expected_reason = completed_report_reason(evaluation,pose,e,s,cfg)
            if reason != expected_reason:issues.append('independent completion/result disagreement')
        elif reason == 'SUCCESS_REPORT':issues.append('success without verified stop')
        if reason == 'TIMEOUT' and not close(elapsed,e['time_limit_s']):issues.append('premature TIMEOUT')
        if reason == 'STOP_NOT_VERIFIED' and (report_time is None or elapsed-report_time+1.e-10 < s['stop_check']['timeout_s']):issues.append('premature stop timeout')
    report_correct = bool(evaluation['report_correct'] and report_time is not None and report_time < e['time_limit_s'])
    mission_success = bool(reason=='SUCCESS_REPORT' and complete and derived_stop is not None and unsafe is None
        and report_correct and completed_report_reason(evaluation,pose,e,s,cfg)=='SUCCESS_REPORT')
    metrics = dict(episode_id=e['episode_id'],attempt_id=result['attempt_id'],policy_id=manifest['policy_id'],
        policy_version=manifest.get('policy_identity',{}).get('version'),scenario_id=e['scenario_id'],
        environment_id=e['environment_id'],layout_hash=e['layout_hash'],layout_family_id=e['layout_family_id'],
        split=e['split'],seed=e['seed'],fixture_hash=manifest['fixture_hash'],is_positive=bool(e['targets']),end_reason=reason,
        report_correct=report_correct,mission_success=mission_success,search_found=first_confirmed is not None,
        n_false_reports=int(reported and not report_correct),first_raw_s=first_raw,first_confirmed_s=first_confirmed,
        report_submitted_s=report_time,stop_verified_s=stop_event_time,position_error_m=evaluation['position_error_m'],
        elapsed_sim_s=elapsed,path_length_m=length if complete else None,path_length_observed_m=length if trajectory else None,
        trajectory_complete=complete,execution_error=reason in ERRORS,perception_mode='oracle',
        SPL=None,oracle_coverage=None,min_clearance_m=None)
    monitor, monitor_issues = monitor_metrics(events,trajectory,dt,complete)
    selector, selector_issues = selector_metrics(events,e,s,manifest['policy_id'],elapsed,complete)
    issues.extend(monitor_issues+selector_issues)
    metrics.update({k:v for k,v in selector.items() if k not in ('complete','observed')})
    metrics.update({k:v for k,v in monitor.items() if k not in ('complete','observed','version')})
    expected_result = {k:metrics[k] for k in ('report_correct','mission_success','search_found','n_false_reports','first_raw_s',
        'first_confirmed_s','report_submitted_s','position_error_m','elapsed_sim_s')}
    if not recovered:expected_result['stop_verified_s'] = stop_event_time
    for name,value in expected_result.items():
        if name not in result:continue
        actual = result[name]
        if actual != value and not close(actual,value):issues.append('result mismatch: '+name)
    actual_length = result.get('path_length_m')
    if actual_length is not None and not close(actual_length,length):issues.append('result mismatch: path_length_m')
    for name,value in selector['observed'].items():
        if result.get(name) is not None and result[name] != value and not close(result[name],value):issues.append('result mismatch: '+name)
    return dict(version=VERSION,status='FAIL' if issues else 'PASS',measurement_status='VERIFIED' if complete else 'UNVERIFIED',
        issues=issues,metrics=metrics,monitor_observed=monitor['observed'],selector_observed=selector['observed'],
        unavailable_reasons=dict(path_length_m=None if complete else 'incomplete trajectory; observed prefix kept separately',
            selector=None if selector['complete'] else 'missing/incomplete selector telemetry',
            unresolved_waypoints='not applicable: no B0 fixed grid' if manifest['policy_id']!='b0-fixed-order-v2' else
                None if selector['complete'] else 'incomplete selector telemetry',
            monitor=None if complete else 'executed prefix only',SPL='no certified shortest reference path',
            oracle_coverage='not implemented',min_clearance_m='not measured with certified precision'))


def reduce_attempt(directory, bundle):
    directory = output_path(directory) if isinstance(directory,str) else Path(directory)
    bundle = output_path(bundle) if isinstance(bundle,str) else Path(bundle)
    manifest = load_json(directory/'run_manifest.json'); eid = manifest['episode_id']
    from .schema import identifier
    identifier(eid)
    bm = load_json(bundle/'manifest.json')
    for suffix in ('resolved_episode.json','fixture_validation.json'):
        name = eid+'/'+suffix
        if digest((bundle/name).read_bytes()) != bm['files'][name]:raise ValueError('bundle artifact hash mismatch')
    resolved_bytes = (bundle/eid/'resolved_episode.json').read_bytes()
    certificate = load_json(bundle/eid/'fixture_validation.json')
    if (certificate['status']!='PASS' or digest(resolved_bytes)!=manifest['fixture_hash'] or
            manifest['fixture_hash']!=certificate['fixture_hash'] or digest((bundle/'manifest.json').read_bytes())!=manifest['bundle_hash']):
        raise ValueError('attempt/certificate binding mismatch')
    resolved = load_json(bundle/eid/'resolved_episode.json')
    if (manifest.get('protocol')!='t1-0.2' or manifest['config']!=resolved['config'] or manifest['dt_s']!=resolved['shared']['dt_s']
            or any(manifest[k]!=resolved['episode'][k] for k in ('seed','environment_id','layout_hash','layout_family_id','split'))):
        raise ValueError('attempt resolved config/environment mismatch')
    result = load_json(directory/'result.json'); complete = result.get('trajectory_complete') is True
    events = [json.loads(line,object_pairs_hook=pairs,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite event')))
        for line in received_prefix((directory/'events.jsonl').read_bytes(),complete).splitlines()]
    raw_trajectory = received_prefix((directory/'trajectory.csv').read_bytes(),complete).decode('utf8')
    trajectory = list(csv.DictReader(io.StringIO(raw_trajectory,newline='')))
    return reduce_records(resolved,manifest,events,trajectory,result)


def received_prefix(raw,complete):
    """Never repair raw files. A partial last line is not a received record."""
    if not raw or raw.endswith(b'\n'):return raw
    if complete:raise ValueError('complete attempt has a torn log tail')
    return raw[:raw.rfind(b'\n')+1]


def summarize(rows):
    """Only primary attempts enter denominators. Missing/duplicate primary is invalid."""
    seen = set(); primary = []; ids = set()
    for row in rows:
        key = row['policy_id'],row['episode_id'],row['attempt_id']
        if key in seen or type(row['attempt_id']) is not int or row['attempt_id'] < 1:raise ValueError('duplicate/invalid attempt')
        seen.add(key); ids.add(key[:2])
        if row['attempt_id']==1:primary.append(row)
    if { (r['policy_id'],r['episode_id']) for r in primary } != ids:raise ValueError('missing primary attempt')
    for policy,eid in ids:
        numbers=sorted(k[2] for k in seen if k[:2]==(policy,eid))
        if numbers!=list(range(1,len(numbers)+1)):raise ValueError('attempt gap')
    positive = [r for r in primary if r['is_positive']]; negative = [r for r in primary if not r['is_positive']]
    return dict(version=VERSION,purpose='dev_pilot_not_research',primary_attempt=1,started=len(primary),attempts=len(rows),
        positive=len(positive),negative=len(negative),execution_errors=sum(r['execution_error'] for r in primary),
        ReportSuccess=sum(r['report_correct'] for r in positive)/len(positive) if positive else None,
        SafeMissionSuccess=sum(r['mission_success'] for r in positive)/len(positive) if positive else None,
        negative_false_alarm_rate=sum(r['n_false_reports']>0 for r in negative)/len(negative) if negative else None,
        end_reasons=dict(Counter(r['end_reason'] for r in primary)))


def main():
    import argparse
    from .journal import atomic
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('attempt','bundle','output'):parser.add_argument('--'+key,required=True)
    args=parser.parse_args()
    try:
        destination=output_path(args.output)
        if destination.exists():raise ValueError('metric output already exists')
        reduced=reduce_attempt(args.attempt,args.bundle);reduced['reducer_identity']=identity()
        atomic(destination,encode(reduced));print(json.dumps(dict(status=reduced['status'],issues=reduced['issues'])))
        return 0 if reduced['status']=='PASS' else 1
    except (ValueError,OSError,KeyError,TypeError) as error:print(json.dumps({'status':'BLOCKED','error':str(error)}));return 2


if __name__=='__main__':raise SystemExit(main())
