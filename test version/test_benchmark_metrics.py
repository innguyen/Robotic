"""Hand-computed P2.2 metric vectors; no robot, ROS, or policy performance claims."""
import copy
import math
import unittest

from benchmark.artifacts import digest,encode
from benchmark.schema import ROOT,load_json,configs
from benchmark.metrics import reduce_records,summarize,received_prefix
from benchmark.layout import layout_hash

ABSENT = object()


def vector(*,positive=False,states=('IDLE','IDLE','IDLE'),commands=None,report=ABSENT,report_step=0,
           stop_time=None,reason='TIMEOUT',complete=True,policy='test-idle',extra=(),target_x=.7,observed=None):
    suite = load_json(ROOT/'test version/benchmark/fixtures/samples.json')
    s = copy.deepcopy(suite['shared']);s['camera']['confirm_frames']=1
    cfg = configs(s,ROOT)
    e = copy.deepcopy(suite['episodes'][0]);e.update(episode_id='golden',scenario_id='S1' if positive else 'S5',
        bounds_m=[-1.,-1.,1.,1.],obstacles=[],targets=[] if not positive else e['targets'],time_limit_s=max(.1,len(states)*.1))
    if positive:e['targets'][0]['x_m']=target_x
    e['layout_hash']=layout_hash(e)
    manifest = dict(run_id='golden-run',policy_id=policy,fixture_hash='golden-fixture',policy_identity={'version':'2'})
    result = dict(run_id='golden-run',episode_id='golden',attempt_id=1,end_reason=reason,trajectory_complete=complete)
    events = []; rows = []
    def add(kind,t,payload=None,step=None):
        events.append(dict(schema_version='t1-0.2',run_id='golden-run',episode_id='golden',attempt_id=1,event_seq=0,
            step_index=max(0,round(t/.1)) if step is None else step,sim_time_s=t,event_type=kind,payload=payload or {},source='golden'))
    add('EPISODE_START',0.)
    if positive or observed:
        add('RAW_DETECTION',0.)
        add('CONFIRMED_DETECTION',0.,dict(observation_id='obs-1',label='person',x_m=target_x,y_m=0.,
            frame_id='benchmark_map',last_confirmed_seen_s=0.,visible=True,confirmed=True))
    if report is not ABSENT:
        add('TARGET_REPORT',report_step*.1,dict(report=report))
    x=y=yaw=0.;previous=None
    commands = commands if commands is not None else [(0.,0.,0.)]*len(states)
    for i,(state,command) in enumerate(zip(states,commands)):
        add('SENSE',i*.1)
        if state!=previous:
            add('MONITOR_STATE',i*.1,dict(state=state))
            if state not in ('PASS','IDLE','LIMIT'):add('MONITOR_INTERVENTION',i*.1,dict(state=state))
        previous=state
        vx,vy,wz=command
        x += (math.cos(yaw)*vx-math.sin(yaw)*vy)*.1
        y += (math.sin(yaw)*vx+math.cos(yaw)*vy)*.1
        yaw = (yaw+wz*.1+math.pi)%(2*math.pi)-math.pi
        rows.append(dict(step_index=i,sim_time_s=(i+1)*.1,x_m=x,y_m=y,yaw_rad=yaw,
                         vx_executed=vx,vy_executed=vy,wz_executed=wz,monitor_state=state))
    for kind,t,payload in extra:add(kind,t,payload)
    if stop_time is not None:add('STOP_VERIFIED',stop_time,dict(hold_steps=3),len(states)-1)
    elapsed=len(states)*.1
    add('EPISODE_END',elapsed,dict(end_reason=reason),max(0,len(states)-1))
    events.sort(key=lambda event:event['sim_time_s'])
    for i,event in enumerate(events):event['event_seq']=i+1
    return dict(episode=e,shared=s,config=cfg),manifest,events,rows,result


def report(x=.7):
    return dict(report_id='r1',label='person',x_m=x,y_m=0.,frame_id='benchmark_map',observation_id='obs-1')


class MetricGoldenTests(unittest.TestCase):
    def reduce(self,**kwargs):
        result=reduce_records(*vector(**kwargs))
        self.assertEqual(result['status'],'PASS',result['issues'])
        return result

    def test_report_and_safe_success_time_zero_and_stop_point_three(self):
        r=self.reduce(positive=True,report=report(),stop_time=.3,reason='SUCCESS_REPORT')['metrics']
        self.assertTrue(r['report_correct']);self.assertTrue(r['mission_success'])
        self.assertEqual(r['first_raw_s'],0.);self.assertEqual(r['first_confirmed_s'],0.);self.assertEqual(r['report_submitted_s'],0.)
        self.assertEqual(r['stop_verified_s'],.3);self.assertAlmostEqual(r['elapsed_sim_s'],.3)
        self.assertEqual(r['path_length_m'],0.);self.assertEqual(r['n_false_reports'],0)

    def test_correct_report_outside_approach_is_not_mission_success(self):
        r=self.reduce(positive=True,target_x=1.,report=report(1.),stop_time=.3,reason='APPROACH_NOT_SATISFIED')['metrics']
        self.assertTrue(r['report_correct']);self.assertFalse(r['mission_success'])

    def test_report_boundary_no_geometry_epsilon_added(self):
        r=self.reduce(positive=True,target_x=1.,report=report(1.5),stop_time=.3,reason='APPROACH_NOT_SATISFIED')['metrics']
        self.assertTrue(r['report_correct']);self.assertEqual(r['position_error_m'],.5)
        r=self.reduce(positive=True,target_x=1.,report=report(1.5000001),stop_time=.3,reason='FALSE_REPORT')['metrics']
        self.assertFalse(r['report_correct']);self.assertEqual(r['n_false_reports'],1)

    def test_nulls_differ_from_zero_in_empty_attempt(self):
        r=self.reduce()['metrics']
        for key in ('first_raw_s','first_confirmed_s','report_submitted_s','stop_verified_s','position_error_m','SPL','oracle_coverage','min_clearance_m'):
            self.assertIsNone(r[key],key)
        self.assertEqual(r['path_length_m'],0.);self.assertEqual(r['n_false_reports'],0)

    def test_false_alarm_includes_invalid_negative_report(self):
        r=self.reduce(report={'report_id':'invalid'},stop_time=.3,reason='INVALID_REPORT')['metrics']
        self.assertFalse(r['report_correct']);self.assertEqual(r['n_false_reports'],1)
        r=self.reduce(report=report(),observed=True,stop_time=.3,reason='FALSE_REPORT')['metrics']
        self.assertEqual(r['n_false_reports'],1)

    def test_path_forward_lateral_and_pure_rotation_hand_sum(self):
        r=self.reduce(states=('PASS',)*4,commands=[(.1,0.,0.),(0.,.1,0.),(0.,0.,.2),(.1,0.,0.)])['metrics']
        self.assertAlmostEqual(r['path_length_m'],.03,places=14);self.assertAlmostEqual(r['elapsed_sim_s'],.4)

    def test_crash_retains_received_correct_report_but_not_full_path_or_counters(self):
        r=self.reduce(positive=True,report=report(),states=('PASS',),commands=[(.1,0.,0.)],reason='CRASH',complete=False)
        m=r['metrics'];self.assertTrue(m['report_correct']);self.assertFalse(m['mission_success'])
        self.assertEqual(r['measurement_status'],'UNVERIFIED');self.assertIsNone(m['path_length_m'])
        self.assertAlmostEqual(m['path_length_observed_m'],.01);self.assertIsNone(m['goals_selected'])
        self.assertIsNone(m['monitor_continuous_episodes']);self.assertEqual(r['monitor_observed']['monitor_continuous_episodes'],0)

    def test_bootstrap_crash_missing_trajectory_is_not_zero_distance_evidence(self):
        r=self.reduce(states=(),reason='CRASH',complete=False)['metrics']
        self.assertIsNone(r['path_length_m']);self.assertIsNone(r['path_length_observed_m'])
        self.assertFalse(r['report_correct']);self.assertEqual(r['elapsed_sim_s'],0.)

    def test_monitor_same_state_entries_and_continuous_episodes(self):
        r=self.reduce(states=('PASS','STOP_ZONE','STOP_ZONE','PASS','STOP_ZONE'))['metrics']
        self.assertEqual(r['monitor_interventions'],2);self.assertEqual(r['monitor_continuous_episodes'],2)
        self.assertAlmostEqual(r['monitor_intervention_s'],.3);self.assertEqual(r['monitor_state_episodes'],{'STOP_ZONE':2})
        self.assertEqual(r['monitor_state_transitions'],3)

    def test_monitor_active_transitions_are_one_continuous_episode(self):
        r=self.reduce(states=('SLOWDOWN','STOP_ZONE','SLOWDOWN'))['metrics']
        self.assertEqual(r['monitor_interventions'],3);self.assertEqual(r['monitor_continuous_episodes'],1)
        self.assertEqual(r['monitor_intervention_state_changes'],2);self.assertEqual(r['monitor_severity'],'stopped')

    def test_monitor_idle_limit_stale_and_approach_and_deadline(self):
        r=self.reduce(states=('LIMIT','IDLE','APPROACH','STALE_SCAN','PASS'))['metrics']
        self.assertEqual(r['monitor_interventions'],2);self.assertEqual(r['monitor_continuous_episodes'],1)
        self.assertAlmostEqual(r['monitor_intervention_s'],.2)
        r=self.reduce(states=('PASS','PASS','SLOWDOWN'))['metrics']
        self.assertEqual(r['monitor_continuous_episodes'],1);self.assertAlmostEqual(r['monitor_intervention_s'],.1)
        self.assertEqual(self.reduce()['metrics']['monitor_continuous_episodes'],0)

    def test_crash_mid_monitor_episode_is_observed_prefix_only(self):
        r=self.reduce(states=('STOP_ZONE','STOP_ZONE'),reason='CRASH',complete=False)
        self.assertIsNone(r['metrics']['monitor_intervention_s']);self.assertAlmostEqual(r['monitor_observed']['monitor_intervention_s'],.2)

    def test_goal_counts_stalls_and_unresolved_active_goal(self):
        status=lambda goals,fallback,unresolved,stalled:dict(goals_selected=goals,fallback_goals=fallback,unresolved_waypoints=unresolved,stalled=stalled)
        extra=[('SELECTOR_STATUS',0.,status(0,0,8,False)),
            ('SELECTOR_DECISION',.1,dict(selection=dict(point=[.75,0.],kind='fixed',waypoint_index=0))),
            ('SELECTOR_STATUS',.1,status(1,0,8,False)),
            ('WAYPOINT_OUTCOME',.2,dict(waypoint_index=0,outcome='reached')),
            ('SELECTOR_DECISION',.2,dict(selection=dict(point=[.4,.4],kind='fallback',waypoint_index=None))),
            ('SELECTOR_STATUS',.2,status(2,1,7,True)),('SELECTOR_STATUS',.4,status(2,1,7,False)),
            ('SELECTOR_STATUS',.5,status(2,1,7,True))]
        r=self.reduce(states=('IDLE',)*7,policy='b0-fixed-order-v2',extra=extra)['metrics']
        self.assertEqual(r['goals_selected'],2);self.assertEqual(r['fallback_goals'],1);self.assertEqual(r['unresolved_waypoints'],7)
        self.assertEqual(r['first_stalled_s'],.2);self.assertAlmostEqual(r['stalled_sim_s'],.4)

    def test_result_and_euler_disagreement_are_failures_not_regraded(self):
        args=vector();args[4]['mission_success']=True
        self.assertIn('result mismatch: mission_success',reduce_records(*args)['issues'])
        args=vector();args[3][0]['x_m']=.02
        self.assertEqual(reduce_records(*args)['status'],'FAIL')

    def test_stop_event_without_slow_velocity_is_fail(self):
        args=vector(positive=True,report=report(),stop_time=.3,reason='SUCCESS_REPORT',commands=[(.02,0.,0.)]*3)
        r=reduce_records(*args);self.assertEqual(r['status'],'FAIL');self.assertFalse(r['metrics']['mission_success'])

    def test_geometry_collision_independent_of_monitor_state(self):
        args=vector(states=('PASS',),commands=[(8.,0.,0.)],reason='COLLISION')
        args[0]['episode']['obstacles']=[dict(id='thin',bounds_m=[.4,-.3,.42,.3])]
        r=reduce_records(*args);self.assertEqual(r['status'],'PASS');self.assertFalse(r['metrics']['mission_success'])
        self.assertEqual(r['metrics']['monitor_continuous_episodes'],0)

    def test_invalid_sequence_and_monitor_are_rejected(self):
        args=vector();args[2][1]['event_seq']=999
        with self.assertRaises(ValueError):reduce_records(*args)
        with self.assertRaises(ValueError):self.reduce(states=('UNRECOGNIZED',))

    def test_stale_and_duplicate_reports_do_not_replace_first_report(self):
        args=vector(positive=True,states=('IDLE',)*4,report=report(),report_step=1,stop_time=.4,reason='INVALID_REPORT')
        args[0]['shared']['camera']['memory_s']=.05
        r=reduce_records(*args);self.assertEqual(r['status'],'PASS');self.assertEqual(r['metrics']['n_false_reports'],1)
        r=self.reduce(positive=True,report={'report_id':'invalid'},stop_time=.3,reason='INVALID_REPORT',
            extra=[('DUPLICATE_REPORT',.1,dict(report=report()))])
        self.assertFalse(r['metrics']['report_correct']);self.assertEqual(r['metrics']['n_false_reports'],1)

    def test_torn_prefix_is_read_only_and_complete_tail_is_rejected(self):
        raw=b'header\nrow\ntorn'
        self.assertEqual(received_prefix(raw,False),b'header\nrow\n');self.assertEqual(raw,b'header\nrow\ntorn')
        self.assertEqual(received_prefix(b'torn',False),b'')
        with self.assertRaises(ValueError):received_prefix(raw,True)

    def test_wrong_bool_and_off_clock_are_rejected(self):
        args=vector();args[4]['trajectory_complete']='false'
        with self.assertRaises(ValueError):reduce_records(*args)
        args=vector();args[2][1]['sim_time_s']=.037
        with self.assertRaises(ValueError):reduce_records(*args)

    def test_orphan_monitor_at_deadline_and_missing_initial_selector_are_not_verified(self):
        args=vector(extra=[('MONITOR_STATE',.3,dict(state='STOP_ZONE')),('MONITOR_INTERVENTION',.3,dict(state='STOP_ZONE'))])
        self.assertEqual(reduce_records(*args)['status'],'FAIL')
        r=self.reduce(policy='b0-fixed-order-v2',extra=[('SELECTOR_STATUS',.1,
            dict(goals_selected=0,fallback_goals=0,unresolved_waypoints=8,stalled=True))])
        self.assertIsNone(r['metrics']['stalled_sim_s']);self.assertAlmostEqual(r['selector_observed']['stalled_sim_s'],.2)


class AggregateGoldenTests(unittest.TestCase):
    def test_manual_denominators_errors_and_successful_retry(self):
        def row(eid,positive,report_correct,success,error=False,attempt=1,false=0):
            return dict(policy_id='p',episode_id=eid,attempt_id=attempt,is_positive=positive,report_correct=report_correct,
                mission_success=success,execution_error=error,n_false_reports=false,end_reason='CRASH' if error else 'TIMEOUT')
        rows=[row('a',True,False,False,True),row('a',True,True,True,attempt=2),row('b',True,True,False),row('c',True,True,True),
              row('d',False,False,False,false=1),row('e',False,False,False)]
        result=summarize(rows)
        self.assertEqual(result['started'],5);self.assertEqual(result['attempts'],6);self.assertEqual(result['execution_errors'],1)
        self.assertAlmostEqual(result['ReportSuccess'],2/3);self.assertAlmostEqual(result['SafeMissionSuccess'],1/3)
        self.assertEqual(result['negative_false_alarm_rate'],.5)

    def test_zero_denominators_and_zero_success(self):
        result=summarize([])
        for key in ('ReportSuccess','SafeMissionSuccess','negative_false_alarm_rate'):self.assertIsNone(result[key])
        row=dict(policy_id='p',episode_id='x',attempt_id=1,is_positive=True,report_correct=False,mission_success=False,
                 execution_error=True,n_false_reports=0,end_reason='CRASH')
        self.assertEqual(summarize([row])['ReportSuccess'],0.)
        with self.assertRaises(ValueError):summarize([row,row])
        with self.assertRaises(ValueError):summarize([{**row,'attempt_id':2}])


if __name__=='__main__':unittest.main()
