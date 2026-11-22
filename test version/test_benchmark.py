"""P1a.1 offline tests. No policy rollout or benchmark performance metrics."""
import copy
import builtins
from dataclasses import FrozenInstanceError
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

from benchmark.layout import canonical_layout, layout_hash, registry
from benchmark.schema import ROOT, load_json, safe_path, validate_suite, validate_corpus
from benchmark.geometry import footprint, box, clearance, bounds_clearance, ray_hit, swept
from benchmark.oracle import Oracle, visible
from benchmark.scenarios import certify, approach

FIX = ROOT/'test version/benchmark/fixtures'


class SchemaTests(unittest.TestCase):
    def setUp(self):
        self.s = load_json(FIX/'samples.json')

    def test_samples(self):
        cfg = validate_suite(self.s)
        self.assertEqual(cfg['monitor']['stop_margin'],.03)
        self.assertEqual(cfg['search']['spin_dwell_s'],1.2)
        self.assertEqual(len(cfg['hashes']),5)

    def test_bad_shared_values(self):
        cases = [('dt_s',True),('dt_s',.2),('report_tolerance_m',float('nan')),('person_standoff_m',0),
                 ('navigation_yaml','../README.md'),('monitor_yaml','C:/bad'),('extra',1)]
        for key,value in cases:
            with self.subTest(key=key,value=value):
                s=copy.deepcopy(self.s); s['shared'][key]=value
                with self.assertRaises((ValueError,OSError)): validate_suite(s)

    def test_nested_types_and_limits(self):
        for section,key,value in [('camera','confirm_frames',True),('camera','confirm_frames',2.),
                                  ('camera','fov_deg',361),('stop_check','hold_steps',0),
                                  ('stop_check','timeout_s',.1),('camera','memory_s',-1)]:
            with self.subTest(section=section,key=key,value=value):
                s=copy.deepcopy(self.s); s['shared'][section][key]=value
                with self.assertRaises(ValueError): validate_suite(s)

    def test_episode_errors(self):
        cases = [('seed',True),('seed',-1),('episode_id','../bad'),('frame_id','odom'),
                 ('time_limit_s',.15),('bounds_m',[0,0,0,1]),('split','train'),('extra',1),
                 ('targets',[]),('layout_hash','0'*64),('obstacles',{})]
        for key,value in cases:
            with self.subTest(key=key):
                s=copy.deepcopy(self.s); s['episodes'][0][key]=value
                with self.assertRaises((ValueError,TypeError)): validate_suite(s)

    def test_geometry_validation(self):
        for mutation in ('start','target','size','obstacle','target_count','duplicate'):
            with self.subTest(mutation=mutation):
                s=copy.deepcopy(self.s); e=s['episodes'][0]
                if mutation=='start': e['start_pose']['x_m']=-1
                if mutation=='target': e['targets'][0]['x_m']=4
                if mutation=='size': e['targets'][0]['size_m']=[0,.3]
                if mutation=='obstacle': e['obstacles'][0]['bounds_m']=[.8,.8,.7,1.2]
                if mutation=='target_count': e['targets']*=2
                if mutation=='duplicate': e['obstacles'].append(copy.deepcopy(e['obstacles'][0]))
                with self.assertRaises(ValueError): validate_suite(s)

    def test_path_cwd_and_symlink(self):
        self.assertTrue(safe_path('test version/navigation_ros1.yaml').is_file())
        for value in ('/etc/passwd','C:/data','a\\b','//host/file','test version/../README.md'):
            with self.assertRaises((ValueError,OSError)): safe_path(value)
        with tempfile.TemporaryDirectory() as d, tempfile.TemporaryDirectory() as outside:
            p=Path(outside)/'file'; p.write_text('x')
            try: (Path(d)/'link').symlink_to(p)
            except OSError as error: self.skipTest('symlink privilege unavailable: '+str(error))
            with self.assertRaises(ValueError): safe_path('link',Path(d))

    def test_duplicate_json_and_yaml_keys(self):
        from benchmark.schema import UniqueLoader
        import yaml
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'bad.json'; p.write_text('{"x":1,"x":2}')
            with self.assertRaises(ValueError): load_json(p)
            p.write_text('{"x":NaN}')
            with self.assertRaises(ValueError): load_json(p)
        with self.assertRaises(ValueError): yaml.load('a: 1\na: 2',Loader=UniqueLoader)

    def test_config_unknown_and_bool_numeric(self):
        from benchmark import schema
        original=schema.yaml.load
        for text in ('unknown_key: 1','resolution: true','grid_size: 12'):
            with self.subTest(text=text), patch.object(schema.yaml,'load',return_value=original(text,Loader=schema.UniqueLoader)):
                with self.assertRaises(ValueError): validate_suite(self.s)

    def test_cross_suite_registry(self):
        a=copy.deepcopy(self.s); b=copy.deepcopy(self.s)
        b['suite_id']='other'; b['episodes']=b['episodes'][:1]; e=b['episodes'][0]
        e['episode_id']='test-variant'; e['start_pose']['y_m']=.1; e['split']='test'
        with self.assertRaises(ValueError): validate_corpus([a,b])
        e['environment_id']='alias'
        with self.assertRaises(ValueError): registry([a,b])
        e['bounds_m'][2]+=1; e['layout_hash']=layout_hash(e)
        with self.assertRaises(ValueError): registry([a,b])
        e['layout_family_id']='different-family'
        registry([a,b])

    def test_physical_duplicate_ignores_seed_and_id(self):
        e=copy.deepcopy(self.s['episodes'][0]); e['episode_id']='renamed'; e['seed']=999
        self.s['episodes'].append(e)
        with self.assertRaisesRegex(ValueError,'physical'): registry([self.s])

    def test_schema_missing_types_and_s3_interface(self):
        for key,value in [('schema_version','t1-0.1'),('suite_id',True),('episodes',[]),('shared',[])]:
            s=copy.deepcopy(self.s); s[key]=value
            with self.subTest(key=key), self.assertRaises(ValueError): validate_suite(s)
        s=copy.deepcopy(self.s); e=s['episodes'][0]; e['scenario_id']='S3'
        with self.assertRaises(ValueError): validate_suite(s)
        e['scenario_evidence']={'gates':[{'id':'a','segment_m':[[0,1],[1,1]]},{'id':'b','segment_m':[[0,-1],[1,-1]]}]}
        cfg=validate_suite(s)
        self.assertEqual(certify(e,s['shared'],cfg)['status'],'UNVERIFIED')

    def test_missing_yaml_dependency_or_file_never_falls_back(self):
        s=copy.deepcopy(self.s); s['shared']['navigation_yaml']='missing.yaml'
        with self.assertRaises(OSError): validate_suite(s)

    def test_validation_never_imports_hardware_modules(self):
        original = builtins.__import__
        def guarded(name,*args,**kwargs):
            if name.split('.')[0] in ('rospy','cv2','paramiko','navigation_ros1','simulate_navigation'):
                raise AssertionError('hardware/runtime module imported: '+name)
            return original(name,*args,**kwargs)
        with patch('builtins.__import__',side_effect=guarded):
            validate_suite(self.s)


class LayoutTests(unittest.TestCase):
    def test_golden_vectors_and_javascript(self):
        vectors=load_json(FIX/'layout-golden.json')
        for v in vectors:
            with self.subTest(name=v['name']):
                self.assertEqual(canonical_layout(v['episode']),v['canonical'])
                self.assertEqual(layout_hash(v['episode']),v['hash'])
        script="const r=require('./docs/checks/layout-contract.cjs'); let s='';process.stdin.on('data',x=>s+=x);process.stdin.on('end',()=>console.log(JSON.stringify(JSON.parse(s).map(v=>[r.canonicalLayout(v.episode),r.layoutHash(v.episode)]))));"
        result=subprocess.run(['node','-e',script],input=json.dumps(vectors),text=True,capture_output=True,cwd=ROOT,check=True)
        self.assertEqual(json.loads(result.stdout),[[v['canonical'],v['hash']] for v in vectors])

    def test_invalid_precision_and_span(self):
        for value in (True,math.inf,math.nan,.0000001,1e10):
            e={'bounds_m':[value,0,1,1],'obstacles':[]}
            with self.assertRaises(ValueError): layout_hash(e)


class GeometryTests(unittest.TestCase):
    def test_rotation_and_clearance(self):
        p=footprint((0,0,math.pi/2),2,1)
        self.assertAlmostEqual(max(x for x,y in p),.5)
        self.assertAlmostEqual(clearance(box([0,0,1,1]),box([2,2,3,3])),math.sqrt(2))
        self.assertEqual(clearance(box([0,0,1,1]),box([1,0,2,1])),0)

    def test_bounds_closed_and_no_epsilon_escape(self):
        self.assertEqual(bounds_clearance(box([0,0,1,1]),[0,0,1,1]),0)
        self.assertLess(bounds_clearance(box([-.0001,0,1,1]),[0,0,1,1]),0)

    def test_ray_tangent_and_parallel(self):
        self.assertTrue(ray_hit((0,0),(2,0),[1,0,1.1,1]))
        self.assertFalse(ray_hit((0,-.1),(2,-.1),[1,0,1.1,1]))

    def test_sweep_thin_obstacle_and_rotation(self):
        self.assertEqual(swept((0,0,0),(2,0,0),.2,.2,[-3,-3,3,3],[[.512,-1,.513,1]],.001),'FAIL')
        self.assertEqual(swept((0,0,0),(0,0,math.pi/2),2,.2,[-3,-3,3,3],[[.65,.65,.75,.75]],.001),'FAIL')
        self.assertEqual(swept((0,0,0),(1,0,0),.2,.2,[-3,-3,3,3],[],.001),'PASS')

    def test_unresolved_sweep_is_not_pass(self):
        self.assertEqual(swept((0,0,0),(2,0,0),.2,.2,[-3,-3,3,3],[[.512,-1,.513,1]],.001,max_depth=0),'UNVERIFIED')


class OracleScenarioTests(unittest.TestCase):
    def setUp(self):
        self.s=load_json(FIX/'samples.json'); self.cfg=validate_suite(self.s)
        self.camera=self.s['shared']['camera']; self.t=self.s['episodes'][0]['targets'][0]

    def test_visibility_extrinsics_fov_range_occlusion(self):
        self.assertTrue(visible((0,0,0),self.t,[],self.camera))
        self.assertFalse(visible((0,0,math.pi),self.t,[],self.camera))
        self.assertFalse(visible((-2,0,0),self.t,[],self.camera))
        self.assertFalse(visible((0,0,0),self.t,[{'bounds_m':[1,-1,1.1,1]}],self.camera))
        c={**self.camera,'x_m':1,'y_m':1,'yaw_rad':-math.pi/4}
        self.assertTrue(visible((0,0,0),self.t,[],c))

    def test_exact_range_fov_and_memory_boundaries(self):
        c={**self.camera,'range_m':2,'fov_deg':180}
        self.assertTrue(visible((0,0,0),self.t,[],c))
        self.assertFalse(visible((-.00001,0,0),self.t,[],c))
        t={**self.t,'x_m':0,'y_m':2}
        self.assertTrue(visible((0,0,0),t,[],c))
        self.assertFalse(visible((0,0,-.0001),t,[],c))

    def test_confirmation_memory_and_immutable(self):
        o=Oracle(self.camera,.1)
        self.assertIsNone(o.step(0,(0,0,0),self.t,[])[1])
        obs=o.step(1,(0,0,0),self.t,[])[1]
        self.assertTrue(obs.confirmed); self.assertNotIn(self.t['id'],obs.observation_id)
        self.assertFalse(hasattr(obs,'__dict__'))
        with self.assertRaises(FrozenInstanceError): obs.x_m=12
        for i in range(2,27):
            self.assertIsNotNone(o.step(i,(0,0,math.pi),self.t,[])[1])
        self.assertIsNone(o.step(27,(0,0,math.pi),self.t,[])[1])
        with self.assertRaises(ValueError): o.step(29,(0,0,0),self.t,[])

    def test_confirmation_reset_empty_and_instance_isolation(self):
        o=Oracle(self.camera,.1)
        for i,p in enumerate(((0,0,0),(0,0,math.pi),(0,0,0))):
            self.assertIsNone(o.step(i,p,self.t,[])[1])
        self.assertIsNotNone(o.step(3,(0,0,0),self.t,[])[1])
        self.assertEqual(Oracle(self.camera,.1).step(0,(0,0,0),None,[]),(False,None))

    def test_positive_s1_s2_s5(self):
        witnesses=load_json(FIX/'witnesses.json')
        for e in self.s['episodes']:
            with self.subTest(scenario=e['scenario_id']):
                self.assertEqual(certify(e,self.s['shared'],self.cfg,witnesses.get(e['episode_id']))['status'],'PASS')

    def test_negative_s1_s2_s5(self):
        e=copy.deepcopy(self.s['episodes'][0]); e['start_pose']['yaw_rad']=math.pi
        self.assertEqual(certify(e,self.s['shared'],self.cfg)['status'],'FAIL')
        e['scenario_id']='S2'
        self.assertEqual(certify(e,self.s['shared'],self.cfg)['status'],'FAIL')
        e=copy.deepcopy(self.s['episodes'][2]); e['targets']=[self.t]
        with self.assertRaises(ValueError): validate_suite({**self.s,'episodes':[e]})

    def test_no_witness_rotating_path_short_budget_and_collision(self):
        e=self.s['episodes'][0]; s=self.s['shared']
        self.assertEqual(certify(e,s,self.cfg)['status'],'UNVERIFIED')
        self.assertEqual(certify(e,s,self.cfg,[[0,0,0],[1.3,0,.1]])['status'],'PASS')
        e=copy.deepcopy(e); e['time_limit_s']=.1
        self.assertEqual(certify(e,s,self.cfg,[[0,0,0],[1.3,0,0]])['status'],'UNVERIFIED')
        e['time_limit_s']=180.
        self.assertEqual(certify(e,s,self.cfg,[[0,0,0],[2,0,0]])['status'],'FAIL')

    def test_s3_s4_and_offset_s2_never_silently_pass(self):
        e=copy.deepcopy(self.s['episodes'][0])
        for scenario in ('S3','S4'):
            e['scenario_id']=scenario
            self.assertEqual(certify(e,self.s['shared'],self.cfg)['status'],'UNVERIFIED')
        e=copy.deepcopy(self.s['episodes'][1]); s=copy.deepcopy(self.s['shared']); s['camera']['x_m']=.01
        self.assertEqual(certify(e,s,self.cfg)['status'],'UNVERIFIED')

    def test_certificate_binding_changes_and_unverified_has_no_traits(self):
        e=copy.deepcopy(self.s['episodes'][0]); s=self.s['shared']
        first=certify(e,s,self.cfg)
        self.assertEqual(first['traits'],[])
        e['time_limit_s']+=1
        self.assertNotEqual(first['fixture_hash'],certify(e,s,self.cfg)['fixture_hash'])

    def test_approach_visibility_bounds_and_heading(self):
        e=self.s['episodes'][0]; s=self.s['shared']
        self.assertTrue(approach((1.3,0,0),e,s,self.cfg))
        self.assertFalse(approach((1.3,0,math.pi),e,s,self.cfg))
        self.assertFalse(approach((2,0,0),e,s,self.cfg))


if __name__ == '__main__':
    unittest.main()
