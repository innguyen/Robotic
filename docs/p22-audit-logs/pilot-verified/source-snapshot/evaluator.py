"""Independent task/geometry evaluator. Never supplied to a policy."""
import math
from .geometry import swept
from .scenarios import approach


def finite(values):
    return all(type(v) in (int,float) and math.isfinite(v) for v in values)


def evaluate_report(report,observations,now,e,s):
    fields={'report_id','label','x_m','y_m','frame_id','observation_id'}
    valid=(type(report) is dict and set(report)==fields and
           all(type(report[k]) is str and report[k] for k in ('report_id','label','frame_id','observation_id')) and
           finite([report['x_m'],report['y_m']]) and report['frame_id']=='benchmark_map')
    observation=observations.get(report.get('observation_id')) if valid else None
    valid=bool(valid and observation is not None and observation.confirmed and
               report['label']==observation.label and 0<=now-observation.last_confirmed_seen_s<=s['camera']['memory_s'])
    error=None;matched=False
    numeric_report=(type(report) is dict and report.get('frame_id')=='benchmark_map' and
                    'x_m' in report and 'y_m' in report and finite([report['x_m'],report['y_m']]))
    if numeric_report and e['targets']:
        t=e['targets'][0];error=math.hypot(report['x_m']-t['x_m'],report['y_m']-t['y_m'])
        matched=valid and report['label']==t['label'] and error<=s['report_tolerance_m']
    return dict(valid=valid,matched=matched,report_correct=bool(valid and matched),position_error_m=error)


def safety(a,b,e,s,cfg,boxes):
    cc=cfg['monitor'];r=math.hypot(cc['body_length'],cc['body_width'])+1
    generous=[min(a[0],b[0])-r,min(a[1],b[1])-r,max(a[0],b[0])+r,max(a[1],b[1])+r]
    collision=swept(a,b,cc['body_length'],cc['body_width'],generous,boxes,s['geometry_epsilon_m'])
    # Static boundary contact is allowed: no subdivision uncertainty for no motion.
    if a==b:
        from .geometry import bounds_clearance,footprint
        bounds='PASS' if bounds_clearance(footprint(a,cc['body_length'],cc['body_width']),e['bounds_m'])>=0 else 'FAIL'
    else:bounds=swept(a,b,cc['body_length'],cc['body_width'],e['bounds_m'],[],s['geometry_epsilon_m'])
    reason='COLLISION' if collision!='PASS' else ('OUT_OF_BOUNDS' if bounds!='PASS' else None)
    return reason,dict(collision_check=collision,bounds_check=bounds)


def completed_report_reason(evaluation,pose,e,s,cfg):
    if not evaluation['valid']:return 'INVALID_REPORT'
    if not evaluation['matched']:return 'FALSE_REPORT'
    return 'SUCCESS_REPORT' if approach(pose,e,s,cfg) else 'APPROACH_NOT_SATISFIED'
