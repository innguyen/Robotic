"""Unit/core integration tests. Synthetic observations here are NOT Webots evidence."""
from dataclasses import replace
import math
from pathlib import Path
import unittest

import numpy as np

from adapter import (ANGLE_INCREMENT, ANGLE_MIN, RAYS, CoreAdapter, Observation, ZERO, actuator_limit,
                     camera_target, normalize_scan, odom_pose, rotate, world_velocity)
from fixtures import ROOT, world
from geometry import footprint_separation


def observation(now=.096, epoch=0, **changes):
    value = Observation(epoch, now, (0., 0., 0.), now, (math.inf,)*RAYS, now, True, now)
    return replace(value, **changes)


def box_scan(pose, boxes):
    angles = math.pi-(np.arange(RAYS)+.5)*ANGLE_INCREMENT + pose[2]
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    sx, sy = rotate(.12, 0, pose[2])
    origin = np.array([pose[0]+sx, pose[1]+sy])
    ranges = np.full(RAYS, math.inf)
    for lo, hi in boxes:
        with np.errstate(divide="ignore", invalid="ignore"):
            a, b = (np.array(lo)-origin)/directions, (np.array(hi)-origin)/directions
        entry, leave = np.minimum(a, b).max(axis=1), np.maximum(a, b).min(axis=1)
        hit = (leave >= np.maximum(entry, 0)) & (entry >= .05)
        ranges = np.minimum(ranges, np.where(hit, entry, math.inf))
    return tuple(np.where(ranges <= 6., ranges, math.inf))


class FramesTests(unittest.TestCase):
    def test_camera_and_odom_multiple_rotations(self):
        for yaw in (0, math.pi/2, -math.pi/2, math.pi):
            origin = (-2., -1., -.7)
            pose = (.4, -.3, yaw)
            dx, dy = rotate(1.62, .2, yaw)
            target_world = (pose[0]+dx, pose[1]+dy, 0.)
            actual = camera_target((1.5, .2, .25), odom_pose(pose, origin))
            expected = odom_pose(target_world, origin)[:2]
            self.assertLess(math.dist(actual, expected), 1e-10)

    def test_scan_pixel_centers_no_duplicate_endpoints(self):
        angles = ANGLE_MIN + np.arange(RAYS)*ANGLE_INCREMENT
        self.assertAlmostEqual(angles[0], -math.pi+ANGLE_INCREMENT/2)
        self.assertAlmostEqual(angles[-1], math.pi-ANGLE_INCREMENT/2)
        self.assertAlmostEqual(angles[RAYS//2-1], -ANGLE_INCREMENT/2)
        self.assertEqual(normalize_scan((1.,)*(RAYS-1)+(2.,))[0], 2.)
        self.assertEqual(len(set(np.round(angles, 10))), RAYS)

    def test_sat_avoids_false_rotated_corner_collision(self):
        box = (-.1, 0, .5, .7, .4)
        self.assertGreater(footprint_separation((.34071, -.59520), .29822, box), 0)
        self.assertLess(footprint_separation((-.1, 0), .29822, box), 0)

    def test_actuator_preserves_lateral_and_curvature(self):
        limited = actuator_limit((.36, .24, math.pi/3))
        np.testing.assert_allclose(limited, (.18, .12, math.pi/6))
        np.testing.assert_allclose(world_velocity((0., .1, .2), math.pi/2), (-.1, 0, 0, 0, 0, .2), atol=1e-12)
        with self.assertRaises(ValueError):
            actuator_limit((math.nan, 0, 0))

    def test_fixture_is_canonical_and_no_policy_scene_truth(self):
        self.assertEqual((ROOT / "worlds/s2_search.wbt").read_text(encoding="utf-8"), world("found"))
        for relative in ("adapter.py", "runtime.py", "controllers/s2_core/s2_core.py"):
            source = (ROOT / relative).read_text(encoding="utf-8")
            self.assertNotIn("getFromDef", source)
            self.assertNotIn("s0_smoke", source)


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.adapter = CoreAdapter((0., 0., 0.))

    def tearDown(self):
        self.adapter.close()

    def test_invalid_and_stale_fail_closed_before_policy(self):
        mutations = [dict(ranges=(math.nan,)*RAYS), dict(ranges=(0.,)*RAYS), dict(ranges=(-math.inf,)*RAYS),
                     dict(ranges=(7.,)*RAYS), dict(ranges=(1.,)*90), dict(camera_valid=False),
                     dict(world_pose=(math.nan, 0, 0)), dict(scan_stamp=-1.), dict(camera_stamp=-1.),
                     dict(pose_stamp=-1.), dict(scan_stamp=1.), dict(epoch=9), dict(scan_stamp=.05)]
        for values in mutations:
            with self.subTest(values=values):
                self.adapter.new_episode((0., 0., 0.), 0)
                result = self.adapter.step(observation(**values))
                self.assertEqual(tuple(result["applied"]), ZERO)
                self.assertEqual(self.adapter.policy_steps, 0)
                self.assertEqual(result["monitor_steps"], 1)

    def test_monitor_stops_obstacle_and_limited_commands_are_rechecked(self):
        rays = [math.inf]*RAYS
        rays[RAYS//2-1] = rays[RAYS//2] = .13  # sensor x=.12 -> hit at base x=.25
        result = self.adapter.step(observation(ranges=tuple(rays)), test_command=(2., 1., .5))
        self.assertEqual(result["monitor"], "STOP_ZONE")
        self.assertEqual(tuple(result["applied"]), ZERO)
        self.assertLessEqual(abs(result["limited"][1]), .12)

    def test_oracle_confirmation_needs_two_new_frames_same_id(self):
        first = self.adapter.step(observation(recognition=((1, (2., 0., .25)),)))
        self.assertIsNone(first["target"])
        second = self.adapter.step(observation(.192, recognition=((1, (2., 0., .25)),)))
        self.assertIsNotNone(second["target"])
        lost = self.adapter.step(observation(.288))
        self.assertIsNone(lost["target"])
        changed = self.adapter.step(observation(.384, recognition=((2, (2., 0., .25)),)))
        self.assertIsNone(changed["target"])

    def test_repeated_clock_or_buffer_does_not_confirm(self):
        self.adapter.step(observation(recognition=((1, (2., 0., .25)),)))
        result = self.adapter.step(observation(recognition=((1, (2., 0., .25)),)))
        self.assertEqual(tuple(result["applied"]), ZERO)
        self.assertIn("CLOCK", result["reason"])

    def test_held_buffer_is_not_retimestamped_or_used_by_policy(self):
        obs = observation(recognition=((1, (2., 0., .25)),))
        self.adapter.step(obs)
        count = self.adapter.policy_steps
        result = self.adapter.step(replace(obs, now=.192))
        self.assertEqual(result["reason"], "NO_NEW_SENSOR_SAMPLE")
        self.assertEqual(self.adapter.policy_steps, count)
        self.assertEqual(tuple(result["applied"]), ZERO)

    def test_partially_visible_oracle_center_outside_fov_is_accepted(self):
        self.adapter.step(observation(recognition=((1, (1., .6, .25)),)))
        result = self.adapter.step(observation(.192, recognition=((1, (1., .6, .25)),)))
        self.assertIsNotNone(result["target"])

    def test_explicit_episode_reset_clears_all_core_state(self):
        self.adapter.step(observation())
        old = self.adapter.nav
        self.adapter.mission.found = [2., 0.]
        self.adapter.new_episode((1., -1., .3), 1)
        self.assertIsNot(self.adapter.nav, old)
        self.assertFalse(self.adapter.nav.costmap.cells)
        self.assertFalse(self.adapter.mission.covered)
        self.assertIsNone(self.adapter.mission.found)
        self.assertIsNone(self.adapter.monitor.scan_time)
        self.assertIsNone(self.adapter.confirmed_id)
        result = self.adapter.step(observation(epoch=0))
        self.assertEqual(result["reason"], "EPOCH_MISMATCH")

    def test_reset_joins_actual_async_planner_worker(self):
        self.adapter.close()
        self.adapter = CoreAdapter((0., 0., 0.), async_planning=True)
        self.adapter.step(observation(.096, recognition=((1, (2., 0., .25)),)))
        self.adapter.step(observation(.192, recognition=((1, (2., 0., .25)),)))
        old = self.adapter.nav
        self.assertIsNotNone(old.plan_future)
        self.adapter.new_episode((0., 0., 0.), 1)
        self.assertTrue(old.executor._shutdown)
        self.assertTrue(all(not worker.is_alive() for worker in old.executor._threads))
        self.assertEqual(self.adapter.closed_workers, 1)


class ActualCoreIntegrationTests(unittest.TestCase):
    def test_search_navigator_monitor_complete_with_synthetic_sensor_loop(self):
        adapter = CoreAdapter((-1.8, -.8, -math.pi/2))
        pose = [-1.8, -.8, -math.pi/2]
        target = (1., -.8)
        boxes = [((-3., -3.), (-2.95, 3.)), ((2.95, -3.), (3., 3.)),
                 ((-3., -3.), (3., -2.95)), ((-3., 2.95), (3., 3.)),
                 ((.85, -1.04), (1.15, -.56))]
        reasons = set()
        try:
            for index in range(700):
                now = (index+1)*.096
                dx, dy = rotate(target[0]-pose[0], target[1]-pose[1], -pose[2])
                cx, cy = dx-.12, dy
                recognition = ((1, (cx, cy, .25)),) if cx > 0 and abs(math.atan2(cy, cx)) < .506 and math.hypot(cx, cy) < 3.5 else ()
                obs = observation(now, world_pose=tuple(pose), ranges=box_scan(pose, boxes), recognition=recognition)
                result = adapter.step(obs)
                reasons.add(result["reason"])
                vx, vy, wz = result["applied"]
                wx, wy = rotate(vx, vy, pose[2])
                pose[0] += wx*.096
                pose[1] += wy*.096
                pose[2] += wz*.096
                if result["mission"]["state"] == "FOUND":
                    break
            self.assertEqual(result["mission"]["state"], "FOUND", str(result)+str(reasons))
            self.assertIn("SEARCH_SPIN", reasons)
            self.assertGreater(adapter.navigation_steps, 0)
            self.assertEqual(tuple(result["applied"]), ZERO)
            self.assertLess(math.dist(pose[:2], target), 1.05)
        finally:
            adapter.close()


if __name__ == "__main__":
    unittest.main()
