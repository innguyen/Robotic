"""P1b.1 acceptance tests execute spawned simulator + policy + evaluator + logs."""
import copy
import csv
from dataclasses import fields
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from navigation_core_ros1 import Grid
from benchmark.schema import ROOT,load_json
from benchmark.layout import layout_hash
from benchmark.artifacts import load_catalog,build_bundle,persist,output_path
from benchmark.runner import run_single,verified_input
from benchmark.policy import SensorFrame
from benchmark.oracle import Observation
from benchmark.coverage import coverage_cells


class RunnerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        parent=ROOT/'benchmark-results';parent.mkdir(exist_ok=True)
        cls.directory=Path(tempfile.mkdtemp(prefix='p1b1-test-',dir=parent))
        cls.relative=cls.directory.relative_to(ROOT).as_posix()
        cls.export=output_path(os.environ['T1_ACCEPTANCE_EXPORT']) if os.environ.get('T1_ACCEPTANCE_EXPORT') else None
        if cls.export is not None:cls.export.mkdir(parents=True,exist_ok=False)
        original=load_json(ROOT/'test version/benchmark/fixtures/samples.json')
        shared=copy.deepcopy(original['shared']);shared.update(planner_wall_timeout_s=.2,episode_wall_timeout_s=20.)
        template=copy.deepcopy(original['episodes'][0]);template.update(environment_id='acceptance-layout',layout_family_id='acceptance-family',obstacles=[],time_limit_s=5.)
        template['targets'][0]['x_m']=.7;template['layout_hash']=layout_hash(template)
        episodes=[]
        for name,x,limit in [('positive',0.,5.),('deadline',.01,1.),('late',.02,.9)]:
            e=copy.deepcopy(template);e.update(episode_id=name,time_limit_s=limit);e['start_pose']['x_m']=x;episodes.append(e)
        empty=copy.deepcopy(template);empty.update(episode_id='empty',scenario_id='S5',targets=[],time_limit_s=.5);episodes.append(empty)
        distant=copy.deepcopy(template);distant.update(episode_id='distant',time_limit_s=120.);distant['targets'][0]['x_m']=2.;episodes.append(distant)
        base=dict(schema_version='t1-0.2',suite_id='acceptance-base',shared=shared,episodes=episodes)
        offset=copy.deepcopy(base);offset['suite_id']='acceptance-offset';offset['shared']['camera'].update(x_m=.2,y_m=.1)
        off=copy.deepcopy(template);off['episode_id']='offset';off['start_pose']['y_m']=.05;offset['episodes']=[off]
        hang=copy.deepcopy(base);hang['suite_id']='acceptance-hang';hang['shared']['episode_wall_timeout_s']=3.
        he=copy.deepcopy(empty);he['episode_id']='hang';he['start_pose']['y_m']=.1;hang['episodes']=[he]
        for name,data in [('base.json',base),('offset.json',offset),('hang.json',hang)]:
            (cls.directory/name).write_text(json.dumps(data),encoding='utf8')
        witnesses={e['episode_id']:[[e['start_pose'][k] for k in ('x_m','y_m','yaw_rad')]] for s in (base,offset) for e in s['episodes'] if e['targets']}
        witnesses['distant']=[[0,0,0],[1.3,0,0]]
        (cls.directory/'witnesses.json').write_text(json.dumps(witnesses),encoding='utf8')
        catalog=dict(schema_version='t1-catalog-1',suites=[cls.relative+'/'+n for n in ('base.json','offset.json','hang.json')],
                     witnesses=cls.relative+'/witnesses.json',families={'acceptance-family':dict(source='test-authored',reviewed=True,notes='Same empty layout, one split, unique physical starts/targets.')})
        (cls.directory/'catalog.json').write_text(json.dumps(catalog),encoding='utf8')
        cls.catalog=cls.relative+'/catalog.json';cls.bundle=cls.relative+'/bundle'
        cls.catalog_data,ss,cc,ww=load_catalog(cls.catalog);files,results=build_bundle(cls.catalog_data,ss,cc,ww)
        if not all(r['status']=='PASS' for r in results):raise AssertionError(results)
        persist(files,cls.directory/'bundle')
        cls.counter=0

    @classmethod
    def tearDownClass(cls):
        if cls.directory.resolve().is_relative_to((ROOT/'benchmark-results').resolve()):shutil.rmtree(cls.directory)

    def run_case(self,episode='positive',policy='search-mission-smoke',fault=None):
        type(self).counter+=1;output=self.relative+'/run-'+str(type(self).counter)
        result=run_single(self.catalog,self.bundle,episode,output,policy,fault)
        directory=ROOT/output
        self.assertTrue(result['workers_reaped'])
        self.assertEqual(load_json(directory/'result.json'),result)
        records=[json.loads(line) for line in (directory/'events.jsonl').read_text().splitlines()]
        self.assertEqual([r['event_seq'] for r in records],list(range(1,len(records)+1)))
        self.assertEqual(records[-1]['event_type'],'EPISODE_END')
        self.assertEqual([r['sim_time_s'] for r in records],sorted(r['sim_time_s'] for r in records))
        self.assertTrue(all(r['schema_version']=='t1-0.2' for r in records))
        self.assertNotIn('NaN',(directory/'events.jsonl').read_text())
        if self.export is not None:shutil.copytree(directory,self.export/directory.name)
        return result,records,directory

    def test_real_search_mission_success_and_logging(self):
        r,events,d=self.run_case()
        self.assertEqual(r['end_reason'],'SUCCESS_REPORT');self.assertTrue(r['report_correct']);self.assertTrue(r['mission_success'])
        states=[x['payload']['state'] for x in events if x['event_type']=='RUNNER_STATE']
        self.assertEqual(states,['REPORT_SUBMITTED','VERIFY_STOP','FINISHED'])
        self.assertTrue(any(x['event_type']=='MISSION_STATE' for x in events))
        with (d/'trajectory.csv').open(newline='') as handle:rows=list(csv.DictReader(handle))
        self.assertEqual(r['elapsed_sim_s'],len(rows)*.1)
        self.assertTrue(r['stop_verified'])
        self.assertEqual(r['first_raw_s'],0.);self.assertEqual(r['first_confirmed_s'],.1)

    def test_empty_episode_timeout_not_success(self):
        r,_,_=self.run_case('empty')
        self.assertEqual(r['end_reason'],'TIMEOUT');self.assertFalse(r['report_correct']);self.assertFalse(r['mission_success'])
        self.assertEqual(r['elapsed_sim_s'],.5)

    def test_positive_timeout_without_report(self):
        r,_,_=self.run_case('positive','test-idle')
        self.assertEqual(r['end_reason'],'TIMEOUT');self.assertTrue(r['search_found'])

    def test_invalid_and_false_reports(self):
        for policy,reason in [('test-invalid-report','INVALID_REPORT'),('test-false-report','FALSE_REPORT')]:
            with self.subTest(policy=policy):
                r,_,_=self.run_case(policy=policy)
                self.assertEqual(r['end_reason'],reason);self.assertFalse(r['mission_success'])

    def test_negative_false_alarm(self):
        r,_,_=self.run_case('empty','test-invalid-report')
        self.assertEqual(r['end_reason'],'INVALID_REPORT');self.assertFalse(r['report_correct'])

    def test_stop_exact_deadline_and_timeout_with_correct_report(self):
        r,_,_=self.run_case('deadline','test-delayed-report')
        self.assertEqual(r['end_reason'],'SUCCESS_REPORT');self.assertEqual(r['elapsed_sim_s'],1.)
        r,_,_=self.run_case('late','test-delayed-report')
        self.assertEqual(r['end_reason'],'TIMEOUT');self.assertTrue(r['report_correct']);self.assertFalse(r['mission_success'])

    def test_collision_wins_over_correct_report_and_stop(self):
        r,events,_=self.run_case(policy='test-report',fault='actuator-collision')
        self.assertEqual(r['end_reason'],'COLLISION');self.assertTrue(r['report_correct']);self.assertFalse(r['mission_success'])
        self.assertTrue(any(x['event_type']=='COLLISION' for x in events))

    def test_bounds_and_invalid_state(self):
        r,_,_=self.run_case(policy='test-idle',fault='actuator-bounds')
        self.assertEqual(r['end_reason'],'OUT_OF_BOUNDS')
        r,_,_=self.run_case(policy='test-nan');self.assertEqual(r['end_reason'],'INVALID_STATE')

    def test_stop_not_verified_and_report_retained_on_crash(self):
        r,_,_=self.run_case(policy='test-report',fault='actuator-drift')
        self.assertEqual(r['end_reason'],'STOP_NOT_VERIFIED');self.assertTrue(r['report_correct'])
        r,_,_=self.run_case(policy='test-report',fault='crash-after-report')
        self.assertEqual(r['end_reason'],'CRASH');self.assertTrue(r['report_correct']);self.assertFalse(r['mission_success'])
        r,_,_=self.run_case(policy='test-crash-after-report')
        self.assertEqual(r['end_reason'],'CRASH');self.assertTrue(r['report_correct'])

    def test_camera_offset_real_mission(self):
        r,events,_=self.run_case('offset')
        self.assertEqual(r['end_reason'],'SUCCESS_REPORT')
        self.assertTrue(any(x['event_type']=='CONFIRMED_DETECTION' for x in events))

    def test_report_without_approach_and_stop_counter_reset(self):
        r,_,_=self.run_case('distant','test-report')
        self.assertEqual(r['end_reason'],'APPROACH_NOT_SATISFIED');self.assertTrue(r['report_correct'])
        r,_,_=self.run_case(policy='test-report',fault='actuator-pulse')
        self.assertEqual(r['end_reason'],'SUCCESS_REPORT');self.assertAlmostEqual(r['elapsed_sim_s'],.6)

    def test_matching_threshold_and_invalid_observation_error(self):
        from benchmark.evaluator import evaluate_report
        e={'targets':[{'x_m':1.,'y_m':0.,'label':'person'}]}
        s={'camera':{'memory_s':2.5},'report_tolerance_m':.5}
        obs=Observation('obs-1','person',1.,0.,'benchmark_map',0.,True,True)
        report=dict(report_id='report-1',label='person',x_m=1.5,y_m=0.,frame_id='benchmark_map',observation_id='obs-1')
        self.assertTrue(evaluate_report(report,{'obs-1':obs},0.,e,s)['report_correct'])
        report['x_m']=1.500001
        self.assertFalse(evaluate_report(report,{'obs-1':obs},0.,e,s)['report_correct'])
        report['observation_id']='missing'
        self.assertIsNotNone(evaluate_report(report,{},0.,e,s)['position_error_m'])

    def test_cli_rejects_malformed_yaml_as_input_error(self):
        from benchmark.runner import main
        import yaml
        arguments=['benchmark_run.py','--catalog','bad','--bundle','bad','--episode-id','positive','--output','bad']
        with patch('sys.argv',arguments),patch('benchmark.runner.run_single',side_effect=yaml.YAMLError('malformed test YAML')):
            self.assertEqual(main(),2)

    def test_planner_exception_timeout_and_policy_hang_reaped(self):
        for policy,episode,reason in [('test-planner-error','positive','CRASH'),('test-planner-hang','positive','PLANNER_TIMEOUT'),('test-policy-hang','hang','WALL_TIMEOUT')]:
            with self.subTest(policy=policy):
                r,_,_=self.run_case(episode,policy)
                self.assertEqual(r['end_reason'],reason);self.assertFalse(r['trajectory_complete'])

    def test_gate_rejects_tampering_and_nonpass(self):
        certificate=self.directory/'bundle/positive/fixture_validation.json';original=certificate.read_bytes()
        try:
            certificate.write_text('{"status":"PASS"}')
            with self.assertRaises(ValueError):verified_input(self.catalog,self.bundle,'positive')
        finally:certificate.write_bytes(original)
        witness=self.directory/'witnesses.json';before=witness.read_bytes()
        try:
            ww=json.loads(before);ww['positive']=None;witness.write_text(json.dumps(ww))
            with self.assertRaises(ValueError):verified_input(self.catalog,self.bundle,'positive')
        finally:witness.write_bytes(before)

    def test_coverage_offset_and_unknown_occlusion_no_gt(self):
        grid=Grid(np.zeros((100,100),dtype=int),np.ones((100,100),dtype=bool),[-50,-50],.05)
        camera=dict(x_m=0.,y_m=0.,yaw_rad=0.,fov_deg=20.,range_m=.5)
        centered=coverage_cells(grid,(0,0,0),camera)
        offset=coverage_cells(grid,(0,0,0),{**camera,'x_m':1.,'yaw_rad':math.pi})
        self.assertNotEqual(centered,offset);self.assertTrue(all(x>=10 for x,y in offset))
        grid.raw[:]=-1
        self.assertEqual(coverage_cells(grid,(0,0,0),camera),frozenset())
        grid.raw[:]=0;grid.raw[:,55]=1
        self.assertTrue(all(x<5 for x,y in coverage_cells(grid,(0,0,0),camera)))
        self.assertEqual({f.name for f in fields(SensorFrame)},{'step_index','pose','ranges','observation','monitor_state'})
        self.assertNotIn('targets',Observation.__slots__)


if __name__=='__main__':unittest.main()
