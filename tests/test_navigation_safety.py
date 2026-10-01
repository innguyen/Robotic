import ast
import copy
import math
from pathlib import Path
import unittest

import numpy as np

from navigation_safety import (load_config, PoseHistory, scan_geometry,
                               ScanRecovery, FootprintSafety, DepthClearance, discover_footprint, same_planar_frame)


ROOT = Path(__file__).resolve().parents[1]


def config():
    cfg = load_config(str(ROOT / 'navigation_config.json'))
    cfg.update(body_length_m=0.50, body_width_m=0.36,
               lidar_extrinsics_calibrated=True)
    return cfg


class PoseTests(unittest.TestCase):
    def test_measured_circle_rotation_clearance(self):
        cfg = load_config(str(ROOT / 'navigation_config.json'))
        def unexpected_lookup(*args):
            self.fail('Measured radius must take precedence over auto discovery')
        cfg, source = discover_footprint(cfg, unexpected_lookup)
        guard = FootprintSafety(cfg)
        self.assertTrue(guard.calibrated)
        self.assertFalse(guard.rectangular)
        self.assertAlmostEqual(guard.radius + guard.margin, .25)
        for distance, allowed in ((.462, True), (.27, True), (.19, False)):
            ranges = np.full(720, 3.)
            ranges[540] = distance
            guard.update(ranges, -math.pi, math.pi / 360, .12, 8, 100)
            self.assertEqual(guard.check(0, 0, 18, 100)[0], allowed)
        ranges = np.full(720, 3.)
        ranges[:20] = np.nan
        guard.update(ranges, -math.pi, math.pi / 360, .12, 8, 100)
        self.assertTrue(guard.check(0, 0, 18, 100)[0])

    def test_ros_body_geometry_replaces_unknown_circle(self):
        cfg = load_config(str(ROOT / 'navigation_config.json'))
        cfg['body_radius_m'] = None
        params = {'/move_base/local_costmap/footprint': '[[-0.2,-0.15],[0.2,-0.15],[0.2,0.15],[-0.2,0.15]]',
                  '/move_base/local_costmap/robot_base_frame': 'base_footprint'}
        result, source = discover_footprint(cfg, params.get)
        self.assertAlmostEqual(result['body_length_m'], .4)
        self.assertAlmostEqual(result['body_width_m'], .3)
        self.assertIn('local_costmap', source)
        guard = FootprintSafety(result)
        ranges = np.full(720, 3.)
        ranges[540] = .462
        guard.update(ranges, -math.pi, math.pi / 360, .12, 8, 100)
        self.assertTrue(guard.check(0, 0, 18, 100)[0])
        ranges[:10] = np.nan
        ranges[-10:] = np.nan
        guard.update(ranges, -math.pi, math.pi / 360, .12, 8, 100)
        self.assertTrue(guard.check(0, 0, 18, 100)[0])

    def test_frame_equivalence_requires_urdf_evidence(self):
        desc = '<robot><joint type="fixed"><parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 0.1"/></joint></robot>'
        self.assertTrue(same_planar_frame('base_link', 'base_footprint', desc))
        self.assertFalse(same_planar_frame('base_link', 'base_footprint', ''))
        self.assertFalse(same_planar_frame('base_link', 'base_footprint', desc.replace('0 0 0.1', '0.1 0 0.1')))

    def test_yaw_wrap_and_no_extrapolation(self):
        history = PoseHistory()
        history.add(1, 0, 0, math.radians(179))
        history.add(1.1, .01, 0, math.radians(-179))
        pose = history.interpolate([1.05])[0]
        self.assertAlmostEqual(pose[0], .005)
        self.assertAlmostEqual(abs(pose[2]), math.pi)
        self.assertIsNone(history.interpolate([1.2]))

    def test_gap_and_reset(self):
        h = PoseHistory()
        h.add(1, 0, 0, 0)
        h.add(1.5, 0, 0, 0)
        self.assertIsNone(h.interpolate([1.2]))
        h.add(1.6, 2, 0, 0)
        self.assertEqual(h.generation, 1)
        self.assertIsNone(h.interpolate([1.55]))

    def test_deskew_and_lidar_offset(self):
        cfg = config()
        cfg['lidar_x_m'] = .10
        poses = np.array([[0, 0, 0], [1, 0, math.pi / 2]])
        origins, ends, hits = scan_geometry([1, 1], 0, 0, poses, cfg, .12, 8)
        np.testing.assert_allclose(origins, [[.1, 0], [1, .1]], atol=1e-6)
        np.testing.assert_allclose(ends, [[1.1, 0], [1, 1.1]], atol=1e-6)
        self.assertTrue(hits.all())

    def test_invalid_ranges_not_mapped_as_free(self):
        values = [float('nan'), 0, 12, float('inf'), 2]
        _, _, hits = scan_geometry(values, 0, .1, np.zeros((5, 3)), config(), .12, 8)
        self.assertEqual(hits.tolist(), [False, True])


class RecoveryTests(unittest.TestCase):
    def test_stuck_eventually_plans_without_permanent_block(self):
        recovery = ScanRecovery()
        phases = [recovery.step(0, float(t))[0] for t in range(30)]
        self.assertIn('RECOVER', phases)
        self.assertIn('PLAN_PARTIAL', phases)
        self.assertNotIn('BLOCKED', phases)

    def test_oscillation_does_not_count_as_full_turn(self):
        recovery = ScanRecovery()
        for i in range(100):
            phase, _ = recovery.step(.08 if i % 2 else 0, i * .1)
            self.assertNotEqual(phase, 'PLAN')

    def test_real_turn_and_reverse_turn(self):
        for direction in (1, -1):
            recovery = ScanRecovery()
            phase = None
            for i in range(201):
                yaw = (direction * i * math.pi / 100 + math.pi) % (2 * math.pi) - math.pi
                phase, _ = recovery.step(yaw, i * .1)
                if phase == 'PLAN':
                    break
            self.assertEqual(phase, 'PLAN')

    def test_safe_probe_can_start_after_quarter_turn(self):
        recovery = ScanRecovery()
        recovery.step(0.0, 0.0)
        recovery.step(math.radians(80), 1.0)
        self.assertFalse(recovery.partial_probe_due())
        recovery.step(math.radians(92), 1.2)
        self.assertTrue(recovery.partial_probe_due())
        self.assertFalse(recovery.partial_probe_due(math.radians(100)))


class FootprintTests(unittest.TestCase):
    def guard(self, ranges=None, cfg=None, amin=-math.pi, inc=None):
        guard = FootprintSafety(cfg or config())
        ranges = np.full(720, 3.0) if ranges is None else ranges
        guard.update(ranges, amin, inc or (2 * math.pi / len(ranges)), .12, 8, 100)
        return guard

    def test_clear_room_and_stale(self):
        g = self.guard()
        self.assertTrue(g.check(.15, 0, 18, 100.1)[0])
        self.assertFalse(g.check(.15, 0, 0, 101)[0])

    def test_unknown_sector_allows_only_in_place_rotation(self):
        g = self.guard(np.full(300, 3.), amin=-1.3, inc=2.6 / 299)
        for turn in (-18, 18):
            self.assertTrue(g.check(0, 0, turn, 100)[0])
            self.assertFalse(g.check(.1, 0, turn, 100)[0])
            self.assertFalse(g.check(0, .1, turn, 100)[0])
        self.assertFalse(g.check(0, 0, 18, 101)[0])

    def test_invalid_rays_do_not_block_turn_but_measured_collision_does(self):
        ranges = np.full(720, np.nan)
        self.assertTrue(self.guard(ranges).check(0, 0, 18, 100)[0])
        ranges[360] = .20
        self.assertFalse(self.guard(ranges).check(0, 0, 18, 100)[0])

    def test_clockwise_scan_coverage(self):
        g = self.guard(np.full(720, 3.), amin=math.pi, inc=-2 * math.pi / 720)
        self.assertTrue(g.check(.1, 0, -18, 100)[0])

    def test_corner_sweep_blocks(self):
        cfg = config()
        cfg['body_length_m'] = .8
        cfg['body_width_m'] = .3
        ranges = np.full(720, 3.)
        # Point just outside current upper-right corner but inside rotated body.
        x, y = .36, .27
        idx = round((math.atan2(y, x) + math.pi) / (2 * math.pi / 720))
        ranges[idx] = math.hypot(x, y)
        g = self.guard(ranges, cfg)
        self.assertFalse(g.check(0, 0, 25, 100)[0])

    def test_braking_distance_scales_with_speed(self):
        ranges = np.full(720, 3.)
        ranges[360] = .49
        g = self.guard(ranges)
        self.assertTrue(g.check(.05, 0, 0, 100)[0])
        self.assertFalse(g.check(.30, 0, 0, 100)[0])

    def test_invalid_close_ray_not_clear(self):
        ranges = np.full(720, 3.)
        ranges[350:370] = np.nan
        self.assertFalse(self.guard(ranges).check(.1, 0, 0, 100)[0])

    def test_isolated_dropout_does_not_close_clear_corridor(self):
        ranges = np.full(720, 3.)
        ranges[360] = np.nan
        self.assertTrue(self.guard(ranges).check(.1, 0, 0, 100)[0])
        ranges[360] = 0.
        self.assertTrue(self.guard(ranges).check(.1, 0, 0, 100)[0])

    def test_close_positive_return_is_never_filled_as_dropout(self):
        ranges = np.full(720, 3.)
        ranges[360] = .05
        self.assertFalse(self.guard(ranges).check(.1, 0, 0, 100)[0])

    def test_real_obstacle_survives_neighbor_dropout(self):
        ranges = np.full(720, 3.)
        ranges[359] = np.nan
        ranges[360] = .35
        self.assertFalse(self.guard(ranges).check(.1, 0, 0, 100)[0])

    def test_missing_rear_does_not_block_forward(self):
        ranges = np.full(720, 3.)
        ranges[:55] = np.nan
        ranges[-55:] = np.nan
        g = self.guard(ranges, cfg=load_config(str(ROOT / 'navigation_config.json')))
        self.assertTrue(g.check(.08, 0, 0, 100)[0])
        self.assertFalse(g.check(-.08, 0, 0, 100)[0])

    def test_side_obstacle_can_be_escaped_but_not_approached(self):
        cfg = load_config(str(ROOT / 'navigation_config.json'))
        ranges = np.full(720, 3.)
        x, y = .13, .245
        idx = round((math.atan2(y, x) + math.pi) / (math.pi / 360))
        ranges[idx] = math.hypot(x, y)
        g = self.guard(ranges, cfg=cfg)
        self.assertFalse(g.check(.15, 0, 0, 100)[0])
        self.assertTrue(g.check(0, -.08, 0, 100)[0])
        self.assertFalse(g.check(0, .08, 0, 100)[0])
        self.assertTrue(g.check(0, 0, 18, 100)[0])


class DepthTests(unittest.TestCase):
    def guard(self):
        cfg = config()
        cfg.update(depth_enabled=True, depth_extrinsics_calibrated=True,
                   camera_x_m=-.4, camera_y_m=0., camera_height_m=.3,
                   camera_pitch_down_deg=0., camera_yaw_deg=0.)
        return DepthClearance(cfg, FootprintSafety(cfg))

    def image(self, ceiling=None):
        # Synthetic rectified image: flat floor, optional ceiling, back wall 3m away.
        u, v = np.meshgrid(np.arange(160), np.arange(120))
        vertical = (60 - v) / 80.
        depth = np.full((120, 160), 3.)
        floor = vertical < 0
        depth[floor] = np.minimum(depth[floor], -.3 / vertical[floor])
        if ceiling is not None:
            up = vertical > 0
            depth[up] = np.minimum(depth[up], (ceiling - .3) / vertical[up])
        return depth.astype(np.float32) * 1000

    def test_overhead_clear_above_margin(self):
        g = self.guard()
        g.update(self.image(.8), (80, 80, 80, 60), 100)
        self.assertTrue(g.check(.10, 0, 0, 100)[0], g.status)

    def test_low_ceiling_blocks(self):
        g = self.guard()
        g.update(self.image(.50), (80, 80, 80, 60), 100)
        self.assertFalse(g.check(.10, 0, 0, 100)[0])

    def test_table_leg_blocks(self):
        g = self.guard()
        image = self.image(.8)
        image[45:95, 75:85] = 600
        g.update(image, (80, 80, 80, 60), 100)
        self.assertFalse(g.check(.1, 0, 0, 100)[0])

    def test_missing_depth_and_stale_block(self):
        g = self.guard()
        g.update(np.zeros((120, 160)), (80, 80, 80, 60), 100)
        self.assertFalse(g.check(.1, 0, 0, 100)[0])
        g.update(self.image(), (80, 80, 80, 60), 100)
        self.assertFalse(g.check(.1, 0, 0, 101)[0])

    def test_no_calibration_does_not_authorize(self):
        cfg = config()
        cfg['depth_enabled'] = True
        g = DepthClearance(cfg, FootprintSafety(cfg))
        self.assertFalse(g.check(.1, 0, 0, 100)[0])


class IntegrationTests(unittest.TestCase):
    def test_recent_free_cells_expire_and_new_hit_revokes_free(self):
        import cv2, threading, time
        from unittest.mock import patch
        tree = ast.parse((ROOT / 'jetauto_smart_search.py').read_text(encoding='utf-8'))
        node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'OccupancyGridMapper')
        env = dict(np=np, cv2=cv2, threading=threading, time=time, math=math)
        exec(compile(ast.Module(body=[node], type_ignores=[]), '<mapper>', 'exec'), env)
        m = env['OccupancyGridMapper']()
        origin = np.array([[0., 0.]])
        for stamp in (100., 100.2, 100.4, 100.6, 100.8, 101.):
            with patch.object(time, 'monotonic', return_value=stamp):
                m.update_world_scan(origin, np.array([[1., 0.]]), np.array([True]))
        points = np.array([[.5, 0.], [.5, .5]])
        self.assertEqual(m.recent_free(points, 101.1).tolist(), [True, False])
        self.assertFalse(m.recent_free(points, 102.).any())
        with patch.object(time, 'monotonic', return_value=101.3):
            m.update_world_scan(origin, np.array([[.5, 0.]]), np.array([True]))
        self.assertFalse(m.recent_free(points, 101.3).any())

    def test_no_translation_does_not_cycle_frontier_turns(self):
        import time
        from unittest.mock import Mock
        tree = ast.parse((ROOT / 'jetauto_smart_search.py').read_text(encoding='utf-8'))
        node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'FrontierExplorer')
        guard = Mock()
        guard.check.return_value = (False, 'SWEPT AREA UNKNOWN')
        env = dict(time=time, math=math, FOOTPRINT=guard, ScanRecovery=ScanRecovery)
        exec(compile(ast.Module(body=[node], type_ignores=[]), '<explorer>', 'exec'), env)
        mapper = Mock()
        explorer = env['FrontierExplorer'](mapper)
        explorer.phase = 'PLAN'
        for stamp in (100., 106., 112.):
            self.assertEqual(explorer.step(0, 0, 0, stamp)[:3], (0., 0., 0.))
        self.assertEqual(explorer.phase, 'WAIT')
        mapper.plan_frontier.assert_not_called()

    def test_yolo_keeps_target_under_different_class_overlap(self):
        import cv2
        from types import SimpleNamespace
        from unittest.mock import Mock
        tree = ast.parse((ROOT / 'jetauto_smart_search.py').read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SmartObjectSeeker')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'run_yolo')
        env = dict(np=np, cv2=cv2)
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<yolo>', 'exec'), env)
        output = np.zeros((1, 84, 100), dtype=np.float32)
        output[0, :4, :3] = np.array([[320], [320], [160], [240]])
        output[0, 4, 0] = .9
        output[0, 4 + 56, 1] = .8
        output[0, 4 + 56, 2] = .6
        net = Mock()
        net.forward.return_value = output
        seeker = SimpleNamespace(use_ort=False, net=net, classes=list(range(80)))
        result = env['run_yolo'](seeker, np.zeros((640, 640, 3), dtype=np.uint8))
        self.assertEqual(sorted(d['class_id'] for d in result), [0, 56])

    def test_explorer_tries_diagonal_and_revisits_failed_goal(self):
        import time
        from unittest.mock import Mock
        tree = ast.parse((ROOT / 'jetauto_smart_search.py').read_text(encoding='utf-8'))
        node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'FrontierExplorer')
        guard = Mock()
        guard.check.side_effect = lambda vx, vy, wz, now: (vx > 0 and vy > 0, 'blocked')
        env = dict(time=time, math=math, FOOTPRINT=guard, ScanRecovery=ScanRecovery)
        exec(compile(ast.Module(body=[node], type_ignores=[]), '<explorer>', 'exec'), env)
        mapper = Mock()
        mapper.plan_frontier.return_value = None
        explorer = env['FrontierExplorer'](mapper)
        explorer.phase = 'PLAN'
        result = explorer.step(0, 0, 0, 100)
        self.assertEqual(explorer.phase, 'PROBE')
        self.assertGreater(result[0], 0)
        self.assertGreater(result[1], 0)
        explorer.phase = 'MOVE'
        explorer.goal = (2., 0.)
        explorer.best_remaining = 2.
        explorer.last_progress = 90.
        explorer.step(0, 0, 0, 100)
        self.assertEqual(explorer.visited, [])
        self.assertEqual(explorer.blocked_goals, [((2., 0.), 100)])
        explorer.step(0, 0, 0, 116)
        self.assertEqual(explorer.blocked_goals, [])

    def test_final_heartbeat_depth_veto_stops_every_axis(self):
        import threading
        import time
        from types import SimpleNamespace
        from unittest.mock import Mock, patch
        tree = ast.parse((ROOT / 'jetauto_smart_search.py').read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'SmartObjectSeeker')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'motion_heartbeat_loop')
        publisher = Mock()
        seeker = SimpleNamespace(motion_lock=threading.Lock(), desired_linear_x=.1,
                                 footprint_frame_valid=True,
                                 desired_linear_y=0., desired_angular_z=0., last_motion_update=100.,
                                 manual_override_until=0., last_ai_update=100., search_mode='TRACKING',
                                 depth_guard=Mock(), cmd_publishers=[publisher])
        seeker.depth_guard.check.return_value = (False, 'DEPTH BODY VOLUME BLOCKED')
        ros = Mock()
        ros.is_shutdown.side_effect = [False, True]
        guard = Mock()
        guard.check.return_value = (True, '')
        env = dict(time=time, math=math, rospy=ros, IS_PAUSED=False, AI_COMMAND_TIMEOUT_S=2.,
                   astra_frame_time=100., robot_pose={'last_update':100.},
                   lidar_info={'last_update':100.}, FOOTPRINT=guard,
                   Twist=lambda: SimpleNamespace(linear=SimpleNamespace(), angular=SimpleNamespace()))
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<heartbeat>', 'exec'), env)
        with patch.object(time, 'time', return_value=100.), patch.object(time, 'monotonic', return_value=100.):
            env['motion_heartbeat_loop'](seeker)
        sent = publisher.publish.call_args[0][0]
        self.assertEqual((sent.linear.x, sent.linear.y, sent.angular.z), (0., 0., 0.))
        self.assertEqual(seeker.motion_block_reason, 'DEPTH BODY VOLUME BLOCKED')

    def test_mapper_accumulation_and_expiry(self):
        # Extract the actual mapper without ROS imports or starting robot threads.
        import cv2
        import threading
        from unittest.mock import Mock, patch
        import time
        tree = ast.parse((ROOT / 'jetauto_smart_search.py').read_text(encoding='utf-8'))
        node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'OccupancyGridMapper')
        env = dict(np=np, cv2=cv2, threading=threading, time=time, math=math)
        exec(compile(ast.Module(body=[node], type_ignores=[]), '<mapper>', 'exec'), env)
        m = env['OccupancyGridMapper']()
        with patch.object(time, 'monotonic', side_effect=[100., 131.]):
            m.update_world_scan(np.array([[0., 0.]]), np.array([[1., 0.]]), np.array([True]))
            gx, gy = m.world_to_grid(1, 0)
            self.assertEqual(m.grid[gy, gx], 255)
            m.update_world_scan(np.array([[0., 0.]]), np.array([[0., 1.]]), np.array([True]))
            self.assertEqual(m.grid[gy, gx], 127)
        m.clear()
        self.assertTrue(np.all(m.grid == 127))


if __name__ == '__main__':
    unittest.main()
