"""Evaluator-only certificates; no search policy or task performance measurements."""
import hashlib
import json
import math
from .geometry import footprint, box, clearance, bounds_clearance, target_box, swept, ray_hit
from .oracle import visible


def resolved_episode_bytes(e, s, cfg):
    """Persist these exact bytes as resolved_episode.json when adding run artifacts."""
    return json.dumps({'episode':e,'shared':s,'config':cfg},sort_keys=True,
                      separators=(',',':'),allow_nan=False).encode('utf8')


def approach(pose, e, s, cfg):
    t = e['targets'][0]
    cc, nav = cfg['monitor'], cfg['navigation']
    p = footprint(pose,cc['body_length'],cc['body_width'])
    dx,dy = t['x_m']-pose[0],t['y_m']-pose[1]
    angle = (math.atan2(dy,dx)-pose[2]+math.pi)%(2*math.pi)-math.pi
    return (bounds_clearance(p,e['bounds_m']) >= 0 and
            math.hypot(dx,dy) <= cc['body_length']/2+s['person_standoff_m']+nav['goal_tolerance'] and
            abs(angle) <= math.radians(nav['align_tolerance_deg']) and
            clearance(p,box(target_box(t))) >= cc['stop_margin'] and
            clearance(p,box(target_box(t))) > s['geometry_epsilon_m'] and
            all(clearance(p,box(o['bounds_m'])) > s['geometry_epsilon_m'] for o in e['obstacles']) and
            visible(pose,t,e['obstacles'],s['camera']))


def certify(e, s, cfg, witness=None):
    """Evaluator-only certificate. Input must first pass validate_corpus."""
    from .witness import check_path
    from .proofs import s2_occlusion, s4_barrier, branch_partition
    from .geometry import pose_free
    from .oracle import Oracle
    binding = resolved_episode_bytes(e,s,cfg)
    evidence = json.dumps(witness,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
    result = dict(verifier_version='p1a2-1', fixture_hash=hashlib.sha256(binding).hexdigest(),
                  input_hash=hashlib.sha256(binding).hexdigest(), evidence_hash=hashlib.sha256(evidence).hexdigest(),
                  layout_hash=e['layout_hash'],config_hashes=cfg['hashes'],traits=[],gate_components=None,
                  status='UNVERIFIED',reason='',witness_path=None,witness_time_s=None,
                  method='Euler tick sweep + exact convex-shadow/rectilinear topology proofs')
    def done(status,reason):
        result.update(status=status,reason=reason,traits=[e['scenario_id']] if status=='PASS' else [])
        return result
    cc,camera = cfg['monitor'],s['camera']
    start=tuple(e['start_pose'][k] for k in ('x_m','y_m','yaw_rad'))
    scenario=e['scenario_id']
    if scenario not in ('S1','S2','S3','S4','S5'):return done('FAIL','unknown scenario')
    if scenario=='S5':
        p=footprint(start,cc['body_length'],cc['body_width'])
        margin=min([bounds_clearance(p,e['bounds_m'])]+[clearance(p,box(o['bounds_m']))-s['geometry_epsilon_m'] for o in e['obstacles']])
        return done('PASS' if not e['targets'] and margin>0 else 'UNVERIFIED','empty target set/open-neighbourhood check')
    t=e['targets'][0]
    initial=visible(start,t,e['obstacles'],camera)
    if scenario=='S1' and not initial:return done('FAIL','S1 not visible at initial yaw')
    if scenario=='S2':
        if initial:return done('FAIL','S2 initially visible')
        if not s2_occlusion(e,s):
            if not camera['x_m'] and not camera['y_m']:
                return done('FAIL','S2 is FOV-only, not occluded')
            return done('UNVERIFIED','all-yaw offset-camera occlusion not proved')
        boxes=[o['bounds_m'] for o in e['obstacles']]+[target_box(t)]
        # A sampled yaw is used only as a POSITIVE existence witness, never as
        # a proof that no orientation can see. Occlusion above covers all yaw.
        yaw_candidates=[start[2],math.atan2(t['y_m']-start[1],t['x_m']-start[0])-camera['yaw_rad']]
        yaw_candidates += [2*math.pi*k/72 for k in range(72)]
        if not any(pose_free((start[0],start[1],yaw),cc['body_length'],cc['body_width'],e['bounds_m'],boxes,s['geometry_epsilon_m']) and
                   visible((start[0],start[1],yaw),t,[],camera) for yaw in yaw_candidates):
            return done('UNVERIFIED','no legal range/FOV yaw witness')
    partition=None
    if scenario=='S3':
        partition=branch_partition(e,s)
        if partition is None:return done('UNVERIFIED','branch topology/base occlusion not proved')
        result['gate_components']=partition['summary']
    if scenario=='S4':
        if approach(start,e,s,cfg):return done('FAIL','S4 start already in A')
        if not s4_barrier(e,s,cfg):return done('UNVERIFIED','not all straight routes to A proved blocked')
    branches={}
    if type(witness) is dict:
        if set(witness)-{'poses','branches'}:return done('FAIL','unknown witness fields')
        branches=witness.get('branches',{})
        poses=witness.get('poses')
    else:poses=witness
    path=check_path(poses,e,s,cfg,initial_hold=camera['confirm_frames'] if scenario=='S1' else 0)
    if path['status']!='PASS':return done(path['status'],path['reason'])
    if not approach(path['end_pose'],e,s,cfg):return done('FAIL','witness endpoint outside A')
    if scenario=='S3':
        main_component=partition['locate'](path['end_pose'])
        if main_component not in partition['branches'].values():return done('UNVERIFIED','main witness not in a certified branch')
        visited={main_component}
        if type(branches) is not dict:return done('FAIL','branches must be gate -> path')
        for gate,branch_poses in branches.items():
            if gate not in partition['branches']:return done('FAIL','unknown branch gate')
            branch=check_path(branch_poses,e,s,cfg)
            if branch['status']!='PASS':return done(branch['status'],'branch '+branch['reason'])
            end=branch['end_pose']
            group=partition['locate'](end)
            p=footprint(end,cc['body_length'],cc['body_width'])
            if group!=partition['branches'][gate] or bounds_clearance(p,e['bounds_m'])<=0 or any(clearance(p,box(o['bounds_m']))<=s['geometry_epsilon_m'] for o in e['obstacles']):
                return done('UNVERIFIED','no open, standable alternate branch witness')
            visited.add(group)
        if not set(partition['branches'].values())<=visited:return done('UNVERIFIED','missing branch witness')
    # Recheck sensing and confirmation on the actual integrated poses. This is
    # witness validation, not execution of a search policy.
    oracle=Oracle(camera,s['dt_s'])
    first_view=0. if initial else None
    states=[(start,0.)]+[(row['end_pose'],row['path_length_m']) for row in path['trace']]
    for k,(pose,length) in enumerate(states):
        raw,observation=oracle.step(k,pose,t,e['obstacles'])
        if scenario=='S1' and k==camera['confirm_frames'] and observation is None:
            return done('UNVERIFIED','initial stationary confirmation not obtained')
        if raw and first_view is None:first_view=length
    if scenario=='S2' and (first_view is None or first_view>s['scenario_checks']['s2_max_view_path_m']):
        return done('UNVERIFIED','no nearby visibility witness')
    # Standing at the certified visible endpoint supplies fresh confirmation.
    hold=camera['confirm_frames']+s['stop_check']['hold_steps']+1
    for k in range(len(states),len(states)+camera['confirm_frames']):
        _,obs=oracle.step(k,path['end_pose'],t,e['obstacles'])
    if obs is None:return done('UNVERIFIED','confirmation not obtained')
    ticks=path['ticks']+hold
    if ticks>round(e['time_limit_s']/s['dt_s']):return done('UNVERIFIED','witness hold exceeds budget')
    def stationary(p,length):return dict(pose=list(p),command=[0.,0.,0.],end_pose=list(p),path_length_m=length)
    trace=path['trace']+[stationary(path['end_pose'],path['length_m']) for _ in range(hold)]
    for k,row in enumerate(trace):row.update(step_index=k,sim_time_s=k*s['dt_s'])
    result.update(witness_path=poses,witness_time_s=ticks*s['dt_s'],witness_ticks=ticks,
                  witness_trace=trace)
    return done('PASS','scenario and timed witness verified')
