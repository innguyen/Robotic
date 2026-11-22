"""P1a.2 validation tests; no search policy or performance evaluation."""
import copy
import json
import math
from pathlib import Path
import random
import tempfile
import unittest
from unittest.mock import patch

from benchmark.schema import ROOT,load_json,validate_suite,validate_corpus
from benchmark.scenarios import certify
from benchmark.artifacts import load_catalog,build_bundle,persist,output_path
from benchmark.geometry import pose_free,swept,target_box
from benchmark.proofs import exact_ray_hit,shadow_covers,s2_occlusion,s4_barrier,branch_partition
from benchmark.witness import euler,check_path

FIX=ROOT/'test version/benchmark/fixtures'


class P1a2Tests(unittest.TestCase):
    def setUp(self):
        self.s=load_json(FIX/'samples.json');self.c=validate_suite(self.s)
        self.a=load_json(FIX/'advanced.json');self.ac=validate_suite(self.a)
        self.w=load_json(FIX/'corpus-witnesses.json')

    def test_euler_hand_calculated_and_wrap(self):
        end,raw=euler((1,2,math.pi/2),(.2,.1,0),.1)
        self.assertAlmostEqual(end[0],.99);self.assertAlmostEqual(end[1],2.02)
        end,raw=euler((0,0,math.pi-.01),(0,0,.2),.1)
        self.assertAlmostEqual(end[2],-math.pi+.01)
        self.assertAlmostEqual(raw[2],math.pi+.01)

    def test_initial_hold_wraps_noncanonical_start_yaw(self):
        e=copy.deepcopy(self.s['episodes'][0]); e['start_pose']['yaw_rad']=2*math.pi
        result=certify(e,self.s['shared'],self.c,[[0,0,2*math.pi],[1.3,0,2*math.pi]])
        self.assertEqual(result['status'],'PASS')
        previous=None
        for row in result['witness_trace']:
            if previous is not None:self.assertEqual(row['pose'],previous)
            end,_=euler(row['pose'],row['command'],.1)
            self.assertEqual(row['end_pose'],list(end));previous=row['end_pose']

    def test_rotating_witness_has_actual_tick_schedule(self):
        e=self.s['episodes'][0]; s=self.s['shared']
        result=certify(e,s,self.c,[[0,0,0],[1.3,0,.1]])
        self.assertEqual(result['status'],'PASS')
        self.assertEqual(len(result['witness_trace']),result['witness_ticks'])
        for row in result['witness_trace']:
            end,_=euler(row['pose'],row['command'],s['dt_s'])
            self.assertEqual(list(end),row['end_pose'])
            self.assertAlmostEqual(row['sim_time_s'],row['step_index']*.1)
            vx,vy,wz=row['command'];cc=self.c['monitor']
            self.assertTrue(-cc['max_backward']<=vx<=cc['max_forward'])
            self.assertLessEqual(abs(vy),cc['max_lateral']);self.assertLessEqual(abs(wz),cc['max_angular'])
        self.assertTrue(any(abs(r['command'][2])>0 for r in result['witness_trace']))

    def test_exact_deadline_hold_and_budget_overrun(self):
        e=copy.deepcopy(self.s['episodes'][0]); s=self.s['shared']
        witness=self.w[e['episode_id']]; result=certify(e,s,self.c,witness)
        e['time_limit_s']=result['witness_ticks']*.1
        self.assertEqual(certify(e,s,self.c,witness)['status'],'PASS')
        e['time_limit_s']-=.1
        self.assertEqual(certify(e,s,self.c,witness)['status'],'UNVERIFIED')

    def test_rotating_corner_collision_and_multiple_turns(self):
        e=copy.deepcopy(self.s['episodes'][0]);e['obstacles']=[{'id':'corner','bounds_m':[.15,.14,.155,.145]}]
        self.assertNotEqual(check_path([[0,0,0],[0,0,2*math.pi]],e,self.s['shared'],self.c)['status'],'PASS')

    def test_unwrapped_multiple_turns_not_shortcut(self):
        e=self.s['episodes'][0]
        path=check_path([[0,0,0],[0,0,4*math.pi]],e,self.s['shared'],self.c)
        self.assertEqual(path['status'],'PASS')
        self.assertAlmostEqual(sum(r['command'][2]*.1 for r in path['trace']),4*math.pi)
        self.assertAlmostEqual(path['end_pose'][2],0,places=10)

    def test_shared_camera_coverage_snapshot_only(self):
        s=copy.deepcopy(self.s);s['shared']['camera']['fov_deg']=70;s['shared']['camera']['range_m']=4
        c=validate_suite(s)
        self.assertEqual(c['search']['camera_fov_deg'],70)
        self.assertEqual(c['search']['camera_range'],4)
        self.assertEqual(self.c['search']['camera_fov_deg'],58)

    def test_exact_shadow_predicate_and_interior_property(self):
        obstacle=[1,-1,1.1,1];region=[2,-.5,3,.5]
        self.assertTrue(shadow_covers((0,0),region,[obstacle]))
        rng=random.Random(42)
        for _ in range(200):
            p=(rng.uniform(2,3),rng.uniform(-.5,.5))
            self.assertTrue(exact_ray_hit((0,0),p,obstacle))
        self.assertFalse(shadow_covers((0,0),[2,-3,3,3],[obstacle]))
        self.assertTrue(exact_ray_hit((0,0),(2,2),obstacle))
        self.assertFalse(shadow_covers((0,0),[2,0,3,2],[obstacle]))  # tangent-only corner

    def test_offset_s2_all_yaw_proof_and_positive(self):
        e=self.s['episodes'][1];s=copy.deepcopy(self.s['shared']);s['camera']['x_m']=.02
        self.assertTrue(s2_occlusion(e,s))
        self.assertEqual(certify(e,s,self.c,self.w[e['episode_id']])['status'],'PASS')
        s['camera']['y_m']=1
        self.assertFalse(s2_occlusion(e,s))
        self.assertNotEqual(certify(e,s,self.c,self.w[e['episode_id']])['status'],'PASS')

    def test_s3_continuous_partition_and_both_positive_branches(self):
        e=self.a['episodes'][0];s=self.a['shared']; partition=branch_partition(e,s)
        self.assertEqual(len(set(partition['branches'].values())),2)
        result=certify(e,s,self.ac,self.w[e['episode_id']])
        self.assertEqual(result['status'],'PASS');self.assertIsNotNone(result['gate_components'])
        bad=copy.deepcopy(self.w[e['episode_id']]);bad['branches']['lower']=[[0,0,0],[3,-2.5,0]]
        self.assertEqual(certify(e,s,self.ac,bad)['status'],'FAIL')

    def test_s3_fake_gate_and_missing_distractor(self):
        e=copy.deepcopy(self.a['episodes'][0]);s=self.a['shared'];w=self.w[e['episode_id']]
        self.assertEqual(certify(e,s,self.ac,{'poses':w['poses']})['status'],'UNVERIFIED')
        e['scenario_evidence']['gates'][0]['segment_m']=[[-.1,1],[.1,1]]
        self.assertIsNone(branch_partition(e,s))
        self.assertNotEqual(certify(e,s,self.ac,w)['status'],'PASS')

    def test_s3_gate_diagonal_or_visible_base_never_pass(self):
        e=copy.deepcopy(self.a['episodes'][0]);s=self.a['shared']
        e['scenario_evidence']['gates'][0]['segment_m']=[[-1,1],[1,1.1]]
        self.assertIsNone(branch_partition(e,s))
        e=copy.deepcopy(self.a['episodes'][0]);e['targets'][0].update(x_m=0,y_m=2.5)
        self.assertIsNone(branch_partition(e,s))

    def test_s4_universal_approach_barrier_and_detour(self):
        e=self.a['episodes'][1];s=self.a['shared']
        self.assertTrue(s4_barrier(e,s,self.ac))
        self.assertEqual(certify(e,s,self.ac,self.w[e['episode_id']])['status'],'PASS')
        self.assertEqual(certify(e,s,self.ac,[[0,0,0],[1.3,0,0]])['status'],'FAIL')

    def test_s4_only_target_centre_blocked_is_insufficient(self):
        e=copy.deepcopy(self.a['episodes'][1]);s=self.a['shared']
        e['obstacles'][0]['bounds_m']=[.8,-.01,.9,.01]
        self.assertTrue(exact_ray_hit((0,0),(2,0),e['obstacles'][0]['bounds_m']))
        self.assertFalse(s4_barrier(e,s,self.ac))
        self.assertEqual(certify(e,s,self.ac,self.w[e['episode_id']])['status'],'UNVERIFIED')

    def test_sweep_pass_implies_dense_footprint_clearance(self):
        rng=random.Random(17); cc=self.c['monitor'];bounds=[-3,-3,3,3];obs=[[1,-.3,1.1,.3]]
        accepted=0
        for _ in range(30):
            a=(rng.uniform(-1,.5),rng.uniform(-1,1),rng.uniform(-2,2))
            b=(rng.uniform(-1,.5),rng.uniform(-1,1),rng.uniform(-2,2))
            if swept(a,b,cc['body_length'],cc['body_width'],bounds,obs,.001)=='PASS':
                accepted+=1
                for k in range(51):
                    p=tuple(x+(y-x)*k/50 for x,y in zip(a,b))
                    self.assertTrue(pose_free(p,cc['body_length'],cc['body_width'],bounds,obs,.001))
        self.assertGreater(accepted,0)

    def test_catalog_independent_dev_test_and_leakage(self):
        catalog,suites,cfgs,witnesses=load_catalog('test version/benchmark/fixtures/catalog.json')
        self.assertIn('test',{e['split'] for s in suites for e in s['episodes']})
        _,results=build_bundle(catalog,suites,cfgs,witnesses)
        self.assertTrue(all(r['status']=='PASS' for r in results))
        bad=copy.deepcopy(suites);bad[-1]['episodes'][0]['layout_family_id']='sample-family-1'
        with self.assertRaises(ValueError):validate_corpus(bad)

    def test_catalog_rejects_missing_review_and_duplicate_suite_path(self):
        original=load_json(FIX/'catalog.json')
        for kind in ('unreviewed','missing-family','duplicate-suite'):
            bad=copy.deepcopy(original)
            if kind=='unreviewed':bad['families']['sample-family-1']['reviewed']=False
            if kind=='missing-family':del bad['families']['sample-family-1']
            if kind=='duplicate-suite':bad['suites'].append(bad['suites'][0])
            def loader(p):return bad if Path(p).name=='catalog.json' else load_json(p)
            with self.subTest(kind=kind),patch('benchmark.artifacts.load_json',side_effect=loader),self.assertRaises(ValueError):
                load_catalog('test version/benchmark/fixtures/catalog.json')

    def test_atomic_bundle_round_trip_and_tampering(self):
        files={'manifest.json':b'{}','example/resolved_episode.json':b'{}','example/fixture_validation.json':b'{"status":"PASS"}'}
        with tempfile.TemporaryDirectory() as d:
            dest=Path(d)/'bundle'
            self.assertEqual(persist(files,dest),'PUBLISHED')
            self.assertEqual(persist(files,dest),'VERIFIED_EXISTING')
            (dest/'example/fixture_validation.json').write_bytes(b'{"status":"FAIL"}')
            with self.assertRaises(ValueError):persist(files,dest)
            self.assertEqual((dest/'example/fixture_validation.json').read_bytes(),b'{"status":"FAIL"}')

    def test_atomic_interruption_never_publishes_partial_bundle(self):
        with tempfile.TemporaryDirectory() as d:
            dest=Path(d)/'bundle'
            with patch('benchmark.artifacts.os.rename',side_effect=OSError('injected failure')):
                with self.assertRaises(OSError):persist({'x.json':b'{}'},dest)
            self.assertFalse(dest.exists());self.assertEqual(list(Path(d).iterdir()),[])

    def test_stale_input_witness_and_verifier_rejected(self):
        catalog,suites,cfgs,witnesses=load_catalog('test version/benchmark/fixtures/catalog.json')
        # A small S5-only bundle exercises the identical content-addressing path.
        suites=copy.deepcopy(suites[-1:]);cfgs=cfgs[-1:]
        files,_=build_bundle(catalog,suites,cfgs,witnesses)
        with tempfile.TemporaryDirectory() as d:
            dest=Path(d)/'bundle';persist(files,dest)
            for kind in ('input','evidence','verifier'):
                ss=copy.deepcopy(suites);ww=copy.deepcopy(witnesses)
                if kind=='input':ss[0]['episodes'][0]['time_limit_s']+=1
                if kind=='evidence':ww['test-empty-01']=[[0,0,0]]
                changed,_=build_bundle(catalog,ss,cfgs,ww)
                if kind=='verifier':changed['manifest.json']+=b' '
                with self.subTest(kind=kind),self.assertRaises(ValueError):persist(changed,dest)

    def test_output_paths_no_traversal_or_symlink(self):
        for p in ('../out','C:/out','/tmp/out','a\\b','a/./b'):
            with self.assertRaises(ValueError):output_path(p)
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'real').mkdir()
            try:(root/'link').symlink_to(root/'real',target_is_directory=True)
            except OSError as error:self.skipTest(str(error))
            with self.assertRaises(ValueError):output_path('link/out',root)


if __name__=='__main__':unittest.main()
