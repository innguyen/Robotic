import copy
import gzip
import json
import math
from pathlib import Path
import tempfile
import unittest

from sar_benchmark import PROTOCOL
from sar_benchmark.artifacts import RunStore, verify_inventory
from sar_benchmark.contracts import ContractError, DT_NS, covariance_valid, public_copy, strict_loads, validate_action
from sar_benchmark.episode import Episode, Phase
from sar_benchmark.evaluator import evaluate
from sar_benchmark.geometry import canonical_to_ue, localize, ue_rotation_to_canonical, ue_to_canonical
from sar_benchmark.policies import SearchPolicy
from sar_benchmark.safety import SafetyGate
from sar_benchmark.statistics import rq1


def observation(epoch=1, tick=0):
    def packet(frame, **kwargs):
        return {"epoch": epoch, "sequence": tick, "capture_ns": tick * DT_NS,
                "available_ns": tick * DT_NS, "valid": True, "frame_id": frame, **kwargs}
    return {"protocol": PROTOCOL, "episode_id": "opaque", "epoch": epoch, "tick": tick,
            "sequence": tick, "sim_time_ns": tick * DT_NS, "budget_ns": 3 * DT_NS,
            "profile": "ue5_static_proxy_rgbd_scan_v1",
            "camera": {"width": 8, "height": 8, "fx": 4., "fy": 4., "cx": 3.5, "cy": 3.5,
                       "T_base_optical": [[0, 0, 1, .25], [-1, 0, 0, 0], [0, -1, 0, .65], [0, 0, 0, 1]]},
            "rgb": packet("rgb_optical", data=[0] * 192),
            "depth_m": packet("rgb_optical", data=[None] * 64),
            "scan": packet("base_flu", ranges_m=[None] * 8, status=["no_return"] * 8,
                           angle_min=-math.pi, angle_increment=math.tau / 8, range_max_m=8.),
            "odom": packet("episode", pose=[0., 0., 0.]),
            "limits": {"vx": [-.1, .3], "vy": [-.2, .2], "wz": [-.6, .6]}}


class FixtureBackend:
    """Contract-only fixture. Never produces or claims UE simulation results."""
    def __init__(self):
        self.epoch, self.tick, self.stops = 0, 0, 0
        self.bad_tick = False

    def stop(self):
        self.stops += 1

    def close(self):
        pass

    def packet(self, applied=None):
        obs = observation(self.epoch, self.tick)
        return obs, {"victims": [], "safety_events": [], "path_m": 0., "applied": applied or [0.] * 3,
                     "frame": {"epoch": self.epoch, "capture_ns": self.tick * DT_NS,
                               "available_ns": self.tick * DT_NS, "valid": True,
                               "width": 8, "height": 8, "visible": {}}}

    def reset(self, config, ticks):
        self.epoch += 1
        self.tick = 0
        return self.packet()

    def advance(self, action):
        self.tick += 2 if self.bad_tick else 1
        return self.packet(action)


class PipelineTests(unittest.TestCase):
    def test_canary_fields_fail_closed_everywhere(self):
        for path in ((), ("camera",), ("rgb",), ("depth_m",), ("scan",), ("odom",), ("limits",)):
            obs = observation()
            target = obs
            for field in path:
                target = target[field]
            target["target_pose"] = [4242, 4242, 4242]
            with self.assertRaises(ContractError):
                public_copy(obs)

    def test_public_copy_is_deep_and_strict(self):
        obs = observation()
        copied = public_copy(obs)
        obs["odom"]["pose"][0] = 5
        self.assertEqual(copied["odom"]["pose"][0], 0)
        copied["depth_m"]["data"][0] = math.inf
        with self.assertRaises(ContractError):
            public_copy(copied)

    def test_strict_json_duplicate_nonfinite(self):
        for value in ('{"x":1,"x":2}', '{"x":NaN}', '{"x":Infinity}', '{bad}'):
            with self.assertRaises(ContractError):
                strict_loads(value)

    def test_timestamps_and_frames(self):
        for packet, field, value in (("rgb", "epoch", 2), ("scan", "capture_ns", 1),
                                     ("odom", "frame_id", "world_gt"), ("depth_m", "available_ns", -1)):
            obs = observation()
            obs[packet][field] = value
            with self.assertRaises(ContractError):
                public_copy(obs)

    def test_rigid_transforms_and_units(self):
        p = [1.2, -.8, .65]
        self.assertEqual(ue_to_canonical(canonical_to_ue(p)), p)
        camera = observation()["camera"]
        point = localize(3.5, 3.5, 2, camera, [1, 2, math.pi / 2])
        for actual, expected in zip(point, [1., 4.25, .65]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(ue_rotation_to_canonical([[0, -1, 0], [1, 0, 0], [0, 0, 1]]), [[0, 1, 0], [-1, 0, 0], [0, 0, 1]])

    def test_left_handed_extrinsic_rejected(self):
        obs = observation()
        obs["camera"]["T_base_optical"][0][2] = -1
        with self.assertRaises(ContractError):
            public_copy(obs)

    def test_covariance_all_principal_minors(self):
        self.assertTrue(covariance_valid([[1, .2, 0], [.2, 1, 0], [0, 0, 2]]))
        for matrix in ([[1, 2, 0], [2, 1, 0], [0, 0, 1]], [[1, 0, 0], [0, 0, 0], [0, 0, 1]],
                       [[1, .1, 0], [.2, 1, 0], [0, 0, 1]]):
            self.assertFalse(covariance_valid(matrix))

    def test_common_safety_vector_acceleration(self):
        gate = SafetyGate()
        action, reason = gate.check([.3, .2, .6], observation())
        self.assertAlmostEqual(math.hypot(*action[:2]), .05)
        self.assertAlmostEqual(action[2], .1)
        self.assertEqual(reason, "OK")

    def test_safety_stale_invalid_and_close(self):
        for mutation, expected in ((lambda o: o["scan"].update(valid=False), "STALE"),
                                    (lambda o: o["scan"]["status"].__setitem__(0, "invalid"), "INVALID_RAYS"),
                                    (lambda o: (o["scan"]["ranges_m"].__setitem__(0, .3), o["scan"]["status"].__setitem__(0, "hit")), "CLEARANCE")):
            obs = observation()
            mutation(obs)
            command, reason = SafetyGate().check([.2, 0, 0], obs)
            self.assertEqual(command, [0, 0, 0])
            self.assertTrue(reason.startswith(expected))

    def test_lifecycle_timeout_and_zero(self):
        backend = FixtureBackend()
        episode = Episode(backend, {}, 3)
        obs = episode.reset()
        policy = SearchPolicy("sweep")
        while obs:
            action = policy.act(obs)
            action["finish"] = False
            obs, _ = episode.submit(action)
        self.assertEqual(episode.phase, Phase.TERMINAL)
        self.assertEqual(episode.ledger["terminal_reason"], "TIMEOUT")
        self.assertGreaterEqual(backend.stops, 2)
        with self.assertRaises(ContractError):
            episode.submit(action)

    def test_finish_reset_epoch_and_blind_ack(self):
        backend = FixtureBackend()
        episode = Episode(backend, {}, 3)
        obs = episode.reset()
        action = SearchPolicy().act(obs)
        action["reports"] = [{"target_pose": [4242, 4242, 4242]}]
        action["finish"] = True
        _, ack = episode.submit(action)
        action["reports"][0].clear()
        self.assertEqual(ack, [{"received": True, "slot": 1}])
        self.assertIn("target_pose", episode.ledger["reports"][0]["payload"])
        self.assertEqual(evaluate(episode.ledger)["fp"], 1)
        self.assertEqual(episode.reset()["epoch"], 2)
        self.assertEqual(episode.ledger["reports"], [])

    def test_report_cap_and_malformed_slots(self):
        episode = Episode(FixtureBackend(), {}, 3)
        obs = episode.reset()
        action = SearchPolicy().act(obs)
        action["reports"] = [{}] * 5
        obs, _ = episode.submit(action)
        action = SearchPolicy().act(obs)
        action["reports"] = [{}]
        action["finish"] = True
        _, ack = episode.submit(action)
        self.assertEqual(ack, [{"received": False, "reason": "report_limit"}])
        self.assertEqual(evaluate(episode.ledger)["fp"], 5)

    def test_bad_action_stops_without_advancing(self):
        for key, value in (("epoch", 5), ("sequence", 1), ("twist", [math.nan, 0, 0]), ("teleport", [1, 1, 1])):
            backend = FixtureBackend()
            episode = Episode(backend, {}, 3)
            action = SearchPolicy().act(episode.reset())
            action[key] = value
            with self.assertRaises(ContractError):
                episode.submit(action)
            self.assertEqual(backend.tick, 0)
            self.assertEqual(episode.ledger["terminal_reason"], "AGENT_PROTOCOL_ERROR")

    def test_backend_discontinuity_is_infrastructure_error(self):
        backend = FixtureBackend()
        episode = Episode(backend, {}, 3)
        action = SearchPolicy().act(episode.reset())
        backend.bad_tick = True
        with self.assertRaises(ContractError):
            episode.submit(action)
        self.assertEqual(episode.ledger["terminal_reason"], "INFRA_ERROR")

    def test_black_sensor_control(self):
        for method in ("random", "sweep"):
            policy = SearchPolicy(method)
            self.assertEqual(policy.act(observation())["reports"], [])

    def test_artifact_exclusivity_corruption_and_inventory(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "run"
            store = RunStore(path)
            store.event({"hello": 1})
            store.close()
            self.assertTrue(verify_inventory(path))
            with self.assertRaises(FileExistsError):
                RunStore(path)
            (path / "journal.jsonl").write_text("corrupted")
            self.assertFalse(verify_inventory(path))

    def test_rq1_known_negative_interaction_and_missing_cells(self):
        rows = [{"method": "m", "family": "one", "pair_id": "p", "perception": p, "odometry": n,
                 "metrics": {"safe_localization_recall": 0 if p and n else 1}}
                for p, n in ((0, 0), (1, 0), (0, 1), (1, 1))]
        result = rq1(rows)["m"]
        self.assertEqual(result["interaction"], -1)
        self.assertIsNone(result["ci95"])
        with self.assertRaises(ValueError):
            rq1(rows[:3])
        with self.assertRaises(ValueError):
            rq1(rows + rows[:1])


if __name__ == "__main__":
    unittest.main()
