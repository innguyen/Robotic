import copy
import math
import unittest
import zlib

from sar_benchmark.contracts import ContractError, public_copy
from sar_benchmark.embodiment import MecanumGeometry
from sar_benchmark.evaluator import evaluate
from sar_benchmark.reference import reference_score
from sar_benchmark.scenarios import sar_scenario
from sar_benchmark.scenario_contract import validate_scenario
from sar_benchmark.sensor_response import depth_response
from sar_benchmark.tracking import ReportTracker
from sar_benchmark.policies import detections
from sar_benchmark.admissibility import reachable_grid
from sar_benchmark.safety import SafetyGate
from sar_benchmark.transport import MAX_BYTES, decode_payload
from test_evaluator import ledger,report,add
from test_pipeline import observation


class HumanTests(unittest.TestCase):
    def test_human_gate_allows_safe_retreat_and_blocks_forward_and_rotation(self):
        obs = observation();obs["profile"]="ue5_sar_human_rgbd_scan_v1";obs["action_dt_ns"]=500_000_000
        obs["scan"]["ranges_m"]=[None]*360;obs["scan"]["status"]=["no_return"]*360
        obs["scan"]["angle_min"]=-math.pi;obs["scan"]["angle_increment"]=math.tau/360
        obs["scan"]["ranges_m"][180]=.35;obs["scan"]["status"][180]="hit"
        self.assertEqual(SafetyGate().check([.2,0,0],obs)[0],[0,0,0])
        retreat,reason = SafetyGate().check([-.1,0,0],obs)
        self.assertLess(retreat[0],0);self.assertEqual(reason,"OK")
        self.assertEqual(SafetyGate().check([0,0,.5],obs)[0],[0,0,0])
        obs["scan"]["status"][0]="invalid"
        self.assertEqual(SafetyGate().check([-.1,0,0],obs)[0],[0,0,0])

    def test_human_gate_checks_curved_command_over_full_interval(self):
        obs = observation();obs["profile"]="ue5_sar_human_rgbd_scan_v1";obs["action_dt_ns"]=500_000_000
        obs["scan"]["ranges_m"]=[None]*360;obs["scan"]["status"]=["no_return"]*360
        obs["scan"]["angle_min"]=-math.pi;obs["scan"]["angle_increment"]=math.tau/360
        obs["scan"]["ranges_m"][193]=math.hypot(.45,.1);obs["scan"]["status"][193]="hit"
        self.assertEqual(SafetyGate().check([.2,.1,.5],obs)[0],[0,0,0])
        obs["scan"]["ranges_m"][193]=None;obs["scan"]["status"][193]="no_return"
        self.assertNotEqual(SafetyGate().check([.2,.1,.5],obs)[0],[0,0,0])

    def test_authoring_grid_does_not_cross_blocked_partition(self):
        clean = reachable_grid(sar_scenario("clean"))
        blocked = reachable_grid(sar_scenario("blocked"))
        detour = reachable_grid(sar_scenario("obstacles"))
        self.assertTrue(any(x > 4 for x,y in clean))
        self.assertTrue(blocked)
        self.assertTrue(all(x < 2.1 for x,y in blocked))
        self.assertTrue(any(x > 4 for x,y in detour))
        self.assertNotEqual(sar_scenario("occluded")["boxes"],sar_scenario("obstacles")["boxes"])

    def test_localizer_rejects_invalid_and_stale_estimated_pose(self):
        class MustNotRun:
            def detect(self,*args): raise AssertionError("invalid pose reached inference")
        obs = observation();obs["odom"]["valid"] = False
        self.assertEqual(detections(obs,MustNotRun()),[])
        obs = observation(tick=5);obs["odom"]["capture_ns"] = 0
        self.assertEqual(detections(obs,MustNotRun()),[])

    def test_public_snapshot_has_no_mutable_aliases(self):
        source = observation()
        result = public_copy(source)
        result["camera"]["T_base_optical"][0][0] = 88
        result["rgb"]["data"][0] = 88
        result["depth_m"]["data"][0] = 88
        result["scan"]["ranges_m"][0] = 88
        result["scan"]["status"][0] = "invalid"
        result["odom"]["pose"][0] = 88
        result["limits"]["vx"][0] = 88
        self.assertEqual(source,observation())

    def test_pose_scenarios_and_no_private_metadata_in_observation(self):
        for environment in ("indoor","outdoor"):
            for posture in ("standing","seated","prone"):
                for kind in ("clean","occluded","absent","multi","obstacles","blocked"):
                    config = sar_scenario(kind,environment=environment,posture=posture)
                    validate_scenario(config)
                    self.assertEqual(config["profile"],"sar_human_v1")
                    obs = observation();obs["profile"]="ue5_sar_human_rgbd_scan_v1"
                    public_copy(obs)
                    obs["camera"]["victims"] = config["victims"]
                    with self.assertRaises(ContractError): public_copy(obs)

    def test_mecanum_inverse_forward_signs_and_saturation(self):
        model=MecanumGeometry()
        for body in ([.2,0,0],[0,.2,0],[0,0,.5],[.1,-.15,.3]):
            actual=model.body(model.wheels(body))
            for x,y in zip(body,actual):self.assertAlmostEqual(x,y)
        self.assertEqual([math.copysign(1,w) for w in model.wheels([0,0,1])],[-1,1,-1,1])
        self.assertLessEqual(max(abs(w) for w in model.wheels(model.limit([.3,.2,.6]))),12.000001)

    def test_synthetic_depth_repeat_holes_and_domain_separation(self):
        values=[None,1.,1.,3.,3.,3.,3.,3.]*20
        for mode in ("ideal","gaussian","structured"):
            result=depth_response(values,8,mode,7,5)
            self.assertEqual(result,depth_response(values,8,mode,7,5))
            self.assertIsNone(result[0])
        self.assertNotEqual(depth_response(values,8,"gaussian",7,5),depth_response(values,8,"gaussian",7,6))
        self.assertEqual(depth_response(values,8,"ideal",7,5),values)

    def test_verification_requires_measured_translation_and_multi_target_dedup(self):
        tracker=ReportTracker(True)
        found=[{"position":[2,0,1.2],"confidence":.8,"box":[1,1,5,7]}]
        for tick in range(4):
            obs=observation(tick=tick)
            self.assertEqual(tracker.update(obs,found),[])
        obs=observation(tick=4);obs["odom"]["pose"]=[.25,0,0]
        result=tracker.update(obs,found)
        self.assertEqual(len(result),1)
        self.assertIn("position_covariance_m2",result[0])
        obs=observation(tick=5)
        self.assertEqual(tracker.update(obs,found),[])

    def test_two_hz_clock_credit_is_not_claimed_ten_hz(self):
        obs=observation(tick=1);obs["action_dt_ns"]=500_000_000;obs["sim_time_ns"]=500_000_000
        public_copy(obs)
        data=ledger();data["action_dt_ns"]=500_000_000
        data["budget_ns"]=5_000_000_000;data["terminal_ns"]=2_500_000_000
        for frame in data["frames"].values():
            frame["capture_ns"]*=5;frame["available_ns"]*=5
        add(data,report());data["reports"][0]["accepted_ns"]=1_000_000_000
        actual,reference=evaluate(data),reference_score(data)
        self.assertEqual(actual["tp"],1)
        self.assertAlmostEqual(actual["time_integrated_recall"],.8)
        self.assertEqual(actual["time_integrated_recall"],reference["time_integrated_recall"])

    def test_subthreshold_track_does_not_suppress_later_stronger_report(self):
        tracker = ReportTracker()
        found = [{"position":[2,0,1.2],"confidence":.42,"box":[1,1,5,7]}]
        for tick in range(3):
            self.assertEqual(tracker.update(observation(tick=tick),found),[])
        self.assertEqual(tracker.reported,[])
        found[0]["confidence"] = .9
        result = tracker.update(observation(tick=3),found)
        self.assertEqual(len(result),1)
        self.assertGreaterEqual(result[0]["confidence"],.5)
        self.assertEqual(tracker.update(observation(tick=4),found),[])

    def test_compressed_transport_rejects_truncation_and_extra_stream(self):
        source=b'{"result":42}'
        self.assertEqual(decode_payload(b"Z"+zlib.compress(source)),source)
        for payload in (b"Z"+zlib.compress(source)[:-1],b"Z"+zlib.compress(source)+b"junk"):
            with self.assertRaises(ValueError):decode_payload(payload)
        with self.assertRaises(ValueError):
            decode_payload(b"Z"+zlib.compress(b"0"*(MAX_BYTES+1)))
