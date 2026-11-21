import importlib.util
import json
import math
import sys
import types
import unittest
import threading
from pathlib import Path
from unittest.mock import patch

import numpy as np

from collision_monitor_ros1 import Config, Monitor, translation_sweep_clear
from navigation_core_ros1 import NavConfig, Costmap, Grid, Navigator, astar, nearest_pose, plan_approach, footprint_mask
from navigation_ros1 import parse_target
import navigation_ros1 as adapter


def known_map(navigator, now=10.):
    half = navigator.cfg.grid_size // 2
    navigator.costmap.cells = {(i, j): (False, now)
                              for i in range(-half, half+1) for j in range(-half, half+1)}


class PlanningTests(unittest.TestCase):
    def test_batched_footprint_matches_pointwise_geometry(self):
        rng = np.random.RandomState(170)
        points = rng.uniform(-1., 1., (90, 2))
        for yaw in (0., .4, 1.57, -2.3):
            actual = footprint_mask(points, (-9, -9), (19, 19), yaw, .222, .19, .06)
            expected = np.zeros((19, 19), dtype=bool)
            for row in range(19):
                for col in range(19):
                    delta = points-((col-9+.5)*.06, (row-9+.5)*.06)
                    c, s = math.cos(yaw), math.sin(yaw)
                    expected[row, col] = np.any((np.abs(c*delta[:, 0]+s*delta[:, 1]) <= .222) &
                                               (np.abs(-s*delta[:, 0]+c*delta[:, 1]) <= .19))
            np.testing.assert_array_equal(actual, expected)

    def test_spatial_lookup_matches_full_cloud_sweep(self):
        from collision_monitor_ros1 import translation_sweep_clear
        rng = np.random.RandomState(63)
        points = rng.uniform(-2., 2., (1800, 2))
        for yaw in (0., .4, -1.3):
            grid = Grid(np.zeros((81, 81), dtype=np.int8), np.ones((81, 81), dtype=bool),
                        (-40, -40), .06, obstacles=points, footprint=(.222, .19, yaw))
            for _ in range(100):
                a = rng.uniform(-2.5, 2.5, 2)
                b = a+rng.uniform(-.5, .5, 2)
                self.assertEqual(translation_sweep_clear(points, a, b, yaw, .222, .19),
                    translation_sweep_clear(grid.nearby(a, b), a, b, yaw, .222, .19))

    def test_async_planner_does_not_block_controller_and_discards_reset_result(self):
        nav = Navigator(async_planning=True)
        entered, release = threading.Event(), threading.Event()
        original = plan_approach
        def slow_plan(*args, **kwargs):
            entered.set()
            self.assertTrue(release.wait(2.))
            return original(*args, **kwargs)
        known_map(nav)
        try:
            with patch('navigation_core_ros1.plan_approach', slow_plan):
                velocity, state = nav.step((0., 0., 0.), (2., 0.), .35, 10., .1)
                self.assertEqual(state, 'PLANNING')
                self.assertEqual(velocity, (0., 0., 0.))
                self.assertTrue(entered.wait(1.))
                # Several fresh control/map cycles continue while planning waits.
                for index in range(1, 6):
                    self.assertEqual(nav.step((0., 0., 0.), (2., 0.), .35, 10.+index*.1, .1)[1], 'PLANNING')
                old = nav.plan_future
                nav.reset()
                release.set()
                old.result(timeout=2.)
                nav.step((0., 0., 0.), (1., .8), .35, 10.6, .1)
                self.assertEqual(nav.path, [], 'Old goal result survived a reset')
                nav.plan_future.result(timeout=2.)
                velocity, state = nav.step((0., 0., 0.), (1., .8), .35, 10.7, .1)
                self.assertEqual(state, 'FOLLOW')
                self.assertGreater(np.linalg.norm(velocity), 0.)
        finally:
            release.set()
            nav.close()

    def test_map_point_ends_at_clicked_coordinate_not_object_standoff(self):
        nav = Navigator()
        known_map(nav)
        goal = (1.2, .18)
        nav.step((0., 0., 0.), goal, .55, 10., .1, exact_goal=True)
        np.testing.assert_allclose(nav.path[-1], goal)
        self.assertEqual(nav.route_strategy, 'MAP_POINT')
        self.assertNotEqual(nav.step((.65, .18, 0.), goal, .55, 10.1, .1, exact_goal=True)[1], 'ARRIVED')
        self.assertEqual(nav.step((1.15, .18, 0.), goal, .55, 10.2, .1, exact_goal=True)[1], 'ARRIVED')

    def test_map_point_in_unknown_space_is_explored_but_observed_obstacle_rejected(self):
        nav = Navigator()
        known_map(nav)
        grid = nav.costmap.snapshot((0., 0., 0.), 10.)
        goal = (1.2, .18)
        grid.raw[grid.cell(goal)] = -1
        self.assertFalse(grid.known_blocked(goal))
        path, strategy, _ = plan_approach(grid, np.zeros(2), goal, 0., 0., nav.cc, exact_goal=True)
        self.assertTrue(path)
        self.assertNotEqual(strategy, 'GOAL_BLOCKED_OR_UNKNOWN')
        grid.raw[grid.cell(goal)] = 100
        self.assertTrue(grid.known_blocked(goal))
        path, strategy, _ = plan_approach(grid, np.zeros(2), goal, 0., 0., nav.cc, exact_goal=True)
        self.assertEqual((path, strategy), ([], 'GOAL_BLOCKED_OR_UNKNOWN'))

    def test_diagonal_clear_sweep_can_pass_between_blocked_side_nodes(self):
        points = np.array([[.302, -.15], [-.182, .27]])
        raw = np.zeros((12, 12), dtype=np.int8)
        blocked = footprint_mask(points, (-5, -5), raw.shape, 0., .222, .19, .06)
        grid = Grid(raw, ~blocked, (-5, -5), .06,
                    obstacles=points, footprint=(.222, .19, 0.))
        start, goal = (.03, .03), (.09, .09)
        self.assertFalse(grid.free(grid.cell((.09, .03))))
        self.assertFalse(grid.free(grid.cell((.03, .09))))
        self.assertTrue(grid.segment_free(start, goal))
        self.assertEqual(len(astar(grid, start, goal)), 2)

    def test_batched_edge_sweep_matches_exact_per_edge_sweep(self):
        rng = np.random.RandomState(3)
        for yaw in (0., .4, -1.1):
            points = rng.uniform(-.6, .6, (40, 2))
            raw = np.zeros((25, 25), dtype=np.int8)
            grid = Grid(raw, np.ones(raw.shape, dtype=bool), (-12, -12), .06,
                        obstacles=points, footprint=(.222, .19, yaw))
            for row in range(2, 23, 3):
                for col in range(2, 23, 3):
                    for di, dj in ((0, 1), (1, 0), (1, 1), (1, -1), (0, -1), (-1, 0), (-1, -1), (-1, 1)):
                        a, b = grid.point((row, col)), grid.point((row+di, col+dj))
                        expected = translation_sweep_clear(points, a, b, yaw, .222, .19)
                        self.assertEqual(grid.edge_free((row, col), (row+di, col+dj)), expected)

    def test_diagonal_sweep_rejects_collision_between_free_endpoints(self):
        points = np.array([[.271, -.129]])
        raw = np.zeros((12, 12), dtype=np.int8)
        blocked = footprint_mask(points, (-5, -5), raw.shape, 0., .222, .19, .06)
        grid = Grid(raw, ~blocked, (-5, -5), .06,
                    obstacles=points, footprint=(.222, .19, 0.))
        start, goal = grid.cell((.03, .03)), grid.cell((.09, .09))
        self.assertTrue(grid.free(start))
        self.assertTrue(grid.free(goal))
        self.assertFalse(grid.edge_free(start, goal))

    def test_fifty_cm_corridor_is_not_closed_by_extra_grid_padding(self):
        nav = Navigator()
        known_map(nav)
        # Preserve measured wall points at +/-25 cm, rather than cell centers.
        for x in np.arange(-.4, 2.1, .03):
            for y in (-.25, .25):
                key = tuple(np.floor(np.array([x, y])/nav.cfg.resolution).astype(int))
                nav.costmap.cells[key] = (True, 10.)
                nav.costmap.obstacle_points.setdefault(key, []).append((float(x), y))
        grid = nav.costmap.snapshot((0., 0., 0.), 10.)
        self.assertTrue(grid.point_free((0., 0.)))
        self.assertTrue(grid.segment_free((0., 0.), (1.5, 0.)))
        path = astar(grid, (0., 0.), (1.5, 0.), 0., nav.cc)
        self.assertTrue(path)

    def test_corridor_too_narrow_for_body_and_six_cm_margin_stays_blocked(self):
        nav = Navigator()
        known_map(nav)
        for x in np.arange(-.4, 2.1, .03):
            for y in (-.16, .16):
                key = tuple(np.floor(np.array([x, y])/nav.cfg.resolution).astype(int))
                nav.costmap.cells[key] = (True, 10.)
                nav.costmap.obstacle_points.setdefault(key, []).append((float(x), y))
        self.assertFalse(nav.costmap.snapshot((0., 0., 0.), 10.).point_free((0., 0.)))

    def test_costmap_preserves_measured_hit_and_clears_it_when_observed_free(self):
        costmap = Costmap(NavConfig(), Config(blind_sectors_deg=[]))
        costmap.update([float('inf'), .9731, float('inf')], -.01, .01, .1, 6., (0., 0., 0.), 10.)
        point = (.096+.9731, 0.)
        grid = costmap.snapshot((0., 0., 0.), 10.)
        np.testing.assert_allclose(grid.obstacles, [point], atol=1e-10)
        costmap.update([float('inf')]*3, -.01, .01, .1, 6., (0., 0., 0.), 10.1)
        self.assertEqual(len(costmap.snapshot((0., 0., 0.), 10.1).obstacles), 0)

    def test_approach_within_current_cell_keeps_actual_final_waypoint(self):
        nav = Navigator()
        known_map(nav)
        position = (1.452099, .273877, 0.)
        velocity, state = nav.step(position, (2., 0.), .35, 10., .1)
        self.assertEqual(state, 'FOLLOW')
        self.assertGreater(velocity[0], 0.)
        self.assertGreater(np.linalg.norm(nav.path[-1]-position[:2]), 0.)

    def test_ray_traced_detour_with_blind_sectors_reaches_target(self):
        nav, position = Navigator(), np.zeros(2)
        monitor = Monitor(nav.cc)
        angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
        directions = np.column_stack((np.cos(angles), np.sin(angles)))
        for step in range(230):
            now = 10.+step*.1
            delta = position+(.096, 0)-np.array([.75, 0])
            b = directions.dot(delta)
            disc = b*b-np.dot(delta, delta)+.08**2
            ranges = np.full(360, float('inf'))
            hit = (disc >= 0) & (-b > 0)
            ranges[hit] = -b[hit]-np.sqrt(disc[hit])
            nav.costmap.update(ranges, -math.pi, math.pi/180, .05, 6., (*position, 0.), now)
            command, state = nav.step((*position, 0.), (2., 0.), .35, now, .1)
            from collision_monitor_ros1 import scan_points
            monitor.set_scan(scan_points(ranges, -math.pi, math.pi/180, .05, 6., nav.cc), now)
            monitor.set_command(*command, now=now)
            checked, _ = monitor.evaluate(now)
            position += np.array(checked[:2])*.1
            if state in ('ARRIVED', 'ALIGN'):
                break
        # Yaw is not integrated here, so a final ALIGN (turn toward target) also counts.
        self.assertIn(state, ('ARRIVED', 'ALIGN'), (state, position))
        if state == 'ALIGN':
            self.assertEqual(command[:2], (0., 0.))
            self.assertGreater(command[2]*(0.-position[1]), 0.)

    def test_escape_backs_away_from_obstacle_inside_buffer(self):
        nav = Navigator(NavConfig(memory_s=100.))
        known_map(nav)
        # 3 cm beside the right body edge: inside the 6 cm buffer, outside the chassis.
        # (Escaping away from a left obstacle would point into the rear-right blind sector.)
        nav.costmap.cells[(0, -3)] = (True, 10.)
        nav.costmap.obstacle_points[(0, -3)] = [(.0, -.16)]
        velocity, state = nav.step((0, 0, 0), (2, 0), .35, 10., .1)
        self.assertEqual(state, 'ESCAPE')
        self.assertGreater(velocity[1], 0.)
        self.assertAlmostEqual(velocity[0], 0.)

    def test_escape_avoids_blind_sector_by_turning_the_escape_direction(self):
        nav = Navigator(NavConfig(memory_s=100.))
        known_map(nav)
        # Obstacle left: straight away (-90 deg) is in the rear-right blind sector.
        nav.costmap.cells[(0, 2)] = (True, 10.)
        nav.costmap.obstacle_points[(0, 2)] = [(.0, .16)]
        velocity, state = nav.step((0, 0, 0), (2, 0), .35, 10., .1)
        self.assertEqual(state, 'ESCAPE')
        self.assertLess(velocity[1], 0.)      # still moving away from the obstacle
        self.assertGreater(velocity[0], 0.)   # tilted forward, out of the blind sector

    def test_rotates_in_place_toward_target_behind(self):
        nav = Navigator(NavConfig(memory_s=100.))
        known_map(nav)
        velocity, state = nav.step((0, 0, 0), (-1.5, .2), .35, 10., .1)
        self.assertEqual(state, 'ROTATE')
        self.assertEqual(velocity[:2], (0., 0.))
        self.assertAlmostEqual(velocity[2], nav.cc.max_angular)

    def test_approach_searches_far_side_even_if_direct_goal_is_disconnected(self):
        cc = Config(blind_sectors_deg=[])
        for direct_free in (False, True):
            with self.subTest(direct_free=direct_free):
                raw = np.full((61, 61), -1, dtype=np.int8)
                grid = Grid(raw, raw == 0, (-10, -20), .05)
                for i in range(61):
                    for j in range(61):
                        x, y = grid.point((i, j))
                        corridor = ((abs(x) < .1 and 0 <= y <= 1.3)
                                    or (abs(y-1.2) < .1 and 0 <= x <= 2.1)
                                    or (abs(x-2.) < .1 and 0 <= y <= 1.3))
                        if corridor:
                            raw[i, j] = 0
                if direct_free:
                    raw[grid.cell((1.025, .025))] = 0
                grid.allowed = raw == 0
                path, strategy, _ = plan_approach(grid, np.array([.025, .025]), (1.525, .025), .5, 0., cc)
                self.assertEqual(strategy, 'APPROACH_RING')
                self.assertGreater(path[-1][0], 1.525)
                self.assertTrue(all(grid.segment_free(a, b) for a, b in zip(path, path[1:])))

    def test_unseen_approach_uses_observed_frontier_without_crossing_unknown(self):
        raw = np.full((21, 41), -1, dtype=np.int8)
        raw[5:16, 5:25] = 0
        grid = Grid(raw, raw == 0, (-5, -10), .05)
        start, target = np.array([.025, .025]), np.array([2., .025])
        path, strategy, _ = plan_approach(grid, start, target, .5, 0., Config())
        self.assertEqual(strategy, 'OBSERVED_SUBGOAL')
        self.assertGreater(path[-1][0], .5)
        self.assertTrue(all(grid.free(grid.cell(p)) for p in path))
        self.assertGreaterEqual(np.linalg.norm(path[-1]-target), .5)

    def test_no_reachable_approach_does_not_route_through_obstacles(self):
        raw = np.full((7, 7), 100, dtype=np.int8)
        raw[1, 1] = 0
        grid = Grid(raw, raw == 0, (0, 0), .1)
        path, strategy, reached = plan_approach(grid, np.array([.15, .15]), (.55, .55), .1, 0., Config(blind_sectors_deg=[]))
        self.assertEqual(path, [])
        self.assertEqual(strategy, 'NO_REACHABLE_APPROACH')
        self.assertEqual(reached, 1)

    def test_astar_detours_around_wall(self):
        raw = np.zeros((15, 15), dtype=np.int8)
        raw[3:12, 7] = 100
        grid = Grid(raw, raw == 0, (0, 0), .1)
        path = astar(grid, (.25, .75), (1.25, .75))
        self.assertTrue(path)
        self.assertTrue(any(p[1] < .3 or p[1] > 1.2 for p in path))
        self.assertTrue(all(grid.segment_free(a, b) for a, b in zip(path, path[1:])))

    def test_astar_no_corner_cutting(self):
        allowed = np.array([[True, False], [False, True]])
        self.assertEqual(astar(Grid(np.zeros((2, 2)), allowed, (0, 0), 1), (.5, .5), (1.5, 1.5)), [])

    def test_astar_unknown_not_traversed(self):
        allowed = np.ones((5, 5), dtype=bool)
        allowed[:, 2] = False
        self.assertEqual(astar(Grid(np.zeros((5, 5)), allowed, (0, 0), 1), (.5, .5), (4.5, .5)), [])

    def test_astar_respects_blind_direction(self):
        grid = Grid(np.zeros((8, 8)), np.ones((8, 8), dtype=bool), (0, 0), 1.)
        self.assertEqual(astar(grid, (5.5, 4.5), (1.5, 4.5), 0., Config()), [])

    def test_scan_ray_and_hit_transformed_to_odom(self):
        costmap = Costmap(NavConfig(), Config(blind_sectors_deg=[]))
        costmap.update([float('inf'), 1., float('inf')], -.01, .01, .1, 6., (1., 2., 0.), 10.)
        key = tuple(np.floor(np.array([2.096, 2.])/.06).astype(int))
        self.assertTrue(costmap.cells[key][0])
        self.assertTrue(any(not occupied for occupied, _ in costmap.cells.values()))

    def test_expired_map_returns_unknown(self):
        costmap = Costmap(NavConfig(), Config())
        costmap.cells[(0, 0)] = (False, 1.)
        self.assertEqual(costmap.snapshot((0, 0, 0), 10.).raw.max(), -1)

    def test_inflation_blocks_near_obstacle(self):
        nav = Navigator()
        known_map(nav)
        nav.costmap.cells[(10, 0)] = (True, 10.)
        grid = nav.costmap.snapshot((0, 0, 0), 10.)
        self.assertFalse(grid.free(grid.cell((.45, .02))))
        self.assertTrue(grid.free(grid.cell((.39, .02))))
        self.assertTrue(grid.free(grid.cell((.02, .02))))

    def test_visible_buffer_is_six_cm_and_chassis_is_checked_separately(self):
        nav = Navigator()
        known_map(nav)
        nav.costmap.cells[(10, 0)] = (True, 10.)
        grid = nav.costmap.snapshot((0, 0, 0), 10.)
        row, col = grid.cell((.63, .03))
        self.assertTrue(grid.inflated[row, col+1])
        self.assertFalse(grid.inflated[row, col+2])
        self.assertFalse(grid.allowed[row, col+2])  # body still cannot overlap obstacle

    def test_nearest_pose_requires_time_match(self):
        history = [(1., (0, 0, 0)), (1.2, (.02, 0, 0))]
        self.assertEqual(nearest_pose(history, 1.18, .05), (.02, 0, 0))
        self.assertIsNone(nearest_pose(history, 2., .05))

    def test_invalid_scan_rejected(self):
        with self.assertRaises(ValueError):
            Costmap(NavConfig(), Config()).update([0.]*360, -math.pi, math.pi/180, .1, 6., (0, 0, 0), 10.)

    def test_arrival_stops_immediately(self):
        nav = Navigator()
        nav.previous_velocity[:] = .1
        self.assertEqual(nav.step((0, 0, 0), (.4, 0), .35, 10., .1), ((0, 0, 0), 'ARRIVED'))

    def test_no_map_no_movement(self):
        self.assertEqual(Navigator().step((0, 0, 0), (2, 0), .35, 10., .1)[1], 'START_BLOCKED')

    def test_follow_and_acceleration_limit(self):
        nav = Navigator()
        known_map(nav)
        velocity, state = nav.step((0, 0, 0), (2, 0), .35, 10., .1)
        self.assertEqual(state, 'FOLLOW')
        self.assertGreater(velocity[0], 0)
        self.assertLessEqual(math.hypot(*velocity[:2]), .02500001)
        self.assertEqual(velocity[2], 0)

    def test_replan_when_new_obstacle_blocks_path(self):
        nav = Navigator(collision_cfg=Config(blind_sectors_deg=[]))
        known_map(nav)
        nav.step((0, 0, 0), (2, 0), .35, 10., .1)
        nav.costmap.cells[(12, 0)] = (True, 10.1)
        velocity, state = nav.step((0, 0, 0), (2, 0), .35, 10.1, .1)
        self.assertEqual(state, 'FOLLOW')
        self.assertTrue(any(abs(p[1]) > nav.cc.body_width/2+nav.cfg.inflation_radius for p in nav.path))
        self.assertTrue(all(nav.grid.segment_free(a, b) for a, b in zip(nav.path, nav.path[1:])))

    def test_stuck_checker(self):
        nav = Navigator(NavConfig(memory_s=100.))
        known_map(nav)
        nav.step((0, 0, 0), (2, 0), .35, 10., .1)
        self.assertEqual(nav.step((0, 0, 0), (2, 0), .35, 19., .1)[1], 'STUCK')

    def test_goal_change_releases_stuck(self):
        nav = Navigator(NavConfig(memory_s=100.))
        known_map(nav)
        nav.step((0, 0, 0), (2, 0), .35, 10., .1)
        nav.step((0, 0, 0), (2, 0), .35, 19., .1)
        self.assertNotEqual(nav.step((0, 0, 0), (2, .5), .35, 19.1, .1)[1], 'STUCK')

    def test_partial_goal_inside_local_window(self):
        nav = Navigator()
        known_map(nav)
        velocity, state = nav.step((0, 0, 0), (8, 0), .35, 10., .1)
        self.assertEqual(state, 'FOLLOW_PARTIAL')
        self.assertGreater(velocity[0], 0)

    def test_simulated_path_and_monitor_reach_target(self):
        cfg, cc = NavConfig(memory_s=100.), Config(blind_sectors_deg=[])
        nav, monitor = Navigator(cfg, cc), Monitor(cc)
        known_map(nav)
        nav.costmap.cells[(12, 0)] = (True, 10.)
        obstacle = np.array([[.75, .03]])
        position = np.array([0., 0.])
        yaw = 0.
        greatest_sideways = 0.
        for step in range(350):
            now = 10.+step*.1
            command, state = nav.step((position[0], position[1], yaw), (2., 0.), .35, now, .1)
            c, s = math.cos(yaw), math.sin(yaw)
            delta = obstacle-position
            monitor.set_scan(np.column_stack((c*delta[:, 0]+s*delta[:, 1], -s*delta[:, 0]+c*delta[:, 1])), now)
            monitor.set_command(*command, now=now)
            checked, _ = monitor.evaluate(now)
            position += np.array([c*checked[0]-s*checked[1], s*checked[0]+c*checked[1]])*.1
            yaw += checked[2]*.1
            greatest_sideways = max(greatest_sideways, abs(position[1]))
            if state == 'ARRIVED':
                break
        self.assertEqual(state, 'ARRIVED', (state, position))
        self.assertGreater(greatest_sideways, cc.body_width/2+cfg.inflation_radius)
        self.assertGreaterEqual(np.linalg.norm(position-[2., 0.]), .35)

    def test_realistic_scan_shadow_chooses_reachable_approach_side(self):
        nav = Navigator()
        angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
        directions = np.column_stack((np.cos(angles), np.sin(angles)))
        delta = np.array([.096, 0.]) - [.75, 0.]
        b = directions.dot(delta)
        disc = b*b-np.dot(delta, delta)+.08**2
        ranges = np.full(360, float('inf'))
        hit = (disc >= 0) & (-b > 0)
        ranges[hit] = -b[hit]-np.sqrt(disc[hit])
        nav.costmap.update(ranges, -math.pi, math.pi/180, .05, 6., (0, 0, 0), 10.)
        velocity, state = nav.step((0, 0, 0), (2., 0), .35, 10., .1)
        self.assertEqual(state, 'FOLLOW')
        self.assertTrue(nav.path)
        self.assertGreater(math.hypot(*velocity[:2]), 0)


class TargetTests(unittest.TestCase):
    def payload(self, **kwargs):
        result = dict(valid=True, frame_id='odom', stamp=100., x=1., y=2., label='bottle')
        result.update(kwargs)
        return result

    def test_valid(self):
        self.assertEqual(parse_target(self.payload(), 100.2, 'odom', 1.2), ((1., 2.), 'bottle', .20000000000000284))

    def test_invalid_stale_wrong_frame(self):
        for value in (self.payload(valid=False), self.payload(frame_id='map'),
                      self.payload(stamp=98.), self.payload(x=float('nan')),
                      self.payload(stamp=101.), self.payload(label='')):
            with self.assertRaises(ValueError):
                parse_target(value, 100., 'odom', 1.2)

    def test_vision_exports_confirmed_anchor_without_motion(self):
        ros = types.ModuleType('rospy')
        modules = {'rospy': ros}
        for package, names in [('nav_msgs', ['Odometry', 'OccupancyGrid', 'Path']), ('sensor_msgs', ['Image', 'CameraInfo']), ('std_msgs', ['String'])]:
            pkg, msg = types.ModuleType(package), types.ModuleType(package+'.msg')
            for name in names:
                setattr(msg, name, lambda **kwargs: types.SimpleNamespace(**kwargs))
            pkg.msg = msg
            modules[package], modules[package+'.msg'] = pkg, msg
        with patch.dict(sys.modules, modules):
            path = Path(__file__).with_name('jetauto_find_object.py')
            spec = importlib.util.spec_from_file_location('vision_test_export', path)
            vision = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(vision)
            obj = vision.ObjectSeeker.__new__(vision.ObjectSeeker)
            obj.odom_frame, obj.last_observation_stamp = 'odom', 100.
            obj.last_world_valid = True
            obj.tracker = types.SimpleNamespace(anchor=(1., 2.))
            output = []
            obj.target_publisher = types.SimpleNamespace(publish=lambda msg: output.append(json.loads(msg.data)))
            obj._publish_target(types.SimpleNamespace(valid=True, source='DO SAU', visible=True, age=0.), 'bottle')
            self.assertEqual(parse_target(output[-1], 100.1, 'odom', 1.2)[:2], ((1., 2.), 'bottle'))
            obj.last_world_valid = False
            obj._publish_target(types.SimpleNamespace(valid=True, source='DO SAU', visible=True, age=0.), 'bottle')
            self.assertFalse(output[-1]['valid'])


class AdapterTests(unittest.TestCase):
    def test_node_pauses_gates_inputs_follows_and_stops_on_odom_reset(self):
        ns = types.SimpleNamespace
        callbacks, services, output, statuses = {}, {}, [], []
        reports = []
        navigator_seen, held_paths = [], []
        loop_outputs = {}
        clock, cycle = [100.], [0]
        ros = types.ModuleType('rospy')
        params = {'/jetauto_collision_monitor/scan_frame': 'laser', '~control_hz': 1000.}
        ros.init_node = lambda *a, **k: None
        ros.get_param = lambda name, default: params.get(name, default)
        ros.resolve_name = lambda name: name
        ros.loginfo = lambda *a: None
        ros.on_shutdown = lambda callback: None
        ros.Time = ns(now=lambda: ns(to_sec=lambda: clock[0]))

        def twist():
            return ns(linear=ns(x=0., y=0., z=0.), angular=ns(x=0., y=0., z=0.))

        def publisher(topic, *a, **k):
            def publish(message):
                if topic == '/collision_monitor/cmd_vel_raw':
                    output.append((message.linear.x, message.linear.y, message.angular.z))
                    if cycle[0] <= 11:
                        loop_outputs[cycle[0]-1] = output[-1]
                elif topic == '~state':
                    report = json.loads(message.data)
                    reports.append(report)
                    statuses.append(report['state'])
                    if report['state'] == 'MONITOR_STOP_ZONE':
                        held_paths.append(list(navigator_seen[0].path))
            return ns(publish=publish)

        def subscribe(topic, cls, callback, **kwargs):
            callbacks[topic] = callback
            return ns(unregister=lambda: None)

        def service(topic, cls, callback):
            services[topic] = callback
            return ns(shutdown=lambda: None)

        ros.Publisher, ros.Subscriber, ros.Service = publisher, subscribe, service

        def odom(stamp, x=0.):
            return ns(header=ns(frame_id='odom', stamp=ns(to_sec=lambda: stamp)),
                      child_frame_id='base_footprint', pose=ns(pose=ns(
                          position=ns(x=x, y=0.), orientation=ns(x=0., y=0., z=0., w=1.))))

        def scan(stamp):
            return ns(header=ns(frame_id='laser', stamp=ns(to_sec=lambda: stamp)),
                      ranges=[float('inf')]*360, angle_min=-math.pi,
                      angle_increment=math.pi/180, range_min=.1, range_max=6.)

        def running():
            index = cycle[0]
            cycle[0] += 1
            if index >= 16:
                return True
            clock[0] = stamp = 100.+index*.01
            if index == 0:
                return False
            if index in (1, 10):
                services['~enable'](ns(data=True))
            if index == 1:
                return False
            callbacks['/odom'](odom(stamp-1. if index == 5 else stamp, 5. if index >= 9 else 0.))
            callbacks['/scan'](scan(stamp-1. if index == 6 else stamp))
            if index != 10:
                target = dict(valid=index != 3, frame_id='map' if index == 4 else 'odom',
                              stamp=stamp, x=2., y=0., label='bottle')
                callbacks['/vision/target'](ns(data=json.dumps(target)))
            callbacks['/jetauto_collision_monitor/state'](ns(data=json.dumps(
                {'state': 'STOP_ZONE' if index == 7 else 'IDLE'})))
            if index == 11:
                callbacks['~map_goal'](ns(data=json.dumps(dict(mode='point', frame_id='odom', stamp=stamp, x=6., y=0.))))
            if index == 13:
                services['~enable'](ns(data=False))
            if index == 14:
                callbacks['~map_goal'](ns(data=json.dumps(dict(mode='vision'))))
            if index == 15:
                callbacks['~map_goal'](ns(data=json.dumps(dict(mode='point', frame_id='odom', stamp=stamp, x=500., y=0.))))
            return False

        ros.is_shutdown = running
        modules = {'rospy': ros}
        definitions = {
            'geometry_msgs.msg': {'Twist': twist, 'PoseStamped': lambda: ns(header=ns(), pose=ns(position=ns(), orientation=ns()))},
            'nav_msgs.msg': {'Odometry': object, 'Path': lambda: ns(header=ns(), poses=[]),
                             'OccupancyGrid': lambda: ns(header=ns(), info=ns(origin=ns(position=ns(), orientation=ns())))},
            'sensor_msgs.msg': {'LaserScan': object},
            'std_msgs.msg': {'String': lambda **kw: ns(**kw)},
            'std_srvs.srv': {'SetBool': object, 'SetBoolResponse': lambda **kw: ns(**kw)},
        }
        for name, definitions_for_module in definitions.items():
            parent, child = name.split('.')
            modules[parent] = types.ModuleType(parent)
            mod = types.ModuleType(name)
            for key, value in definitions_for_module.items():
                setattr(mod, key, value)
            modules[name] = mod
            setattr(modules[parent], child, mod)
        original_update, original_step = Costmap.update, Navigator.step
        callback_checks = []

        def update_while_receiving_odom(costmap, *args, **kwargs):
            if cycle[0] == 8:
                # A monitor hold must not destroy an already-computed route.
                navigator_seen[0].path = [np.array([0., 0.]), np.array([1., 0.])]
            if cycle[0] == 6:  # preserve the explicit stale-odom test case
                return original_update(costmap, *args, **kwargs)
            completed = __import__('threading').Event()
            def receive():
                callbacks['/odom'](odom(clock[0]+.001, 5. if cycle[0] >= 10 else 0.))
                if cycle[0] == 3:
                    # A new valid scan arriving during map processing must not
                    # starve the completed (still-fresh) scan's acceptance.
                    callbacks['/scan'](scan(clock[0]+.002))
                completed.set()
            worker = __import__('threading').Thread(target=receive, daemon=True)
            worker.start()
            self.assertTrue(completed.wait(.5), 'Costmap processing blocked the odometry callback')
            callback_checks.append(True)
            return original_update(costmap, *args, **kwargs)

        def step_while_pausing(navigator, *args, **kwargs):
            if not navigator_seen:
                navigator_seen.append(navigator)
            result = original_step(navigator, *args, **kwargs)
            if cycle[0] == 9:
                completed = __import__('threading').Event()
                def pause():
                    services['~enable'](ns(data=False))
                    completed.set()
                worker = __import__('threading').Thread(target=pause, daemon=True)
                worker.start()
                self.assertTrue(completed.wait(.5), 'Planning blocked the pause service')
            return result

        with patch.dict(sys.modules, modules), patch.object(Costmap, 'update', update_while_receiving_odom), patch.object(Navigator, 'step', step_while_pausing):
            adapter.main()
        self.assertEqual(statuses[:2], ['PAUSED', 'WAIT_ODOMETRY'])
        self.assertEqual(statuses[2], 'PLANNING')
        self.assertEqual(statuses[3:8], ['WAIT_TARGET', 'WAIT_TARGET', 'WAIT_ODOMETRY', 'WAIT_SCAN', 'MONITOR_STOP_ZONE'])
        self.assertEqual(statuses[8:10], ['PAUSED', 'PAUSED'])
        for index in (0, 1, 3, 4, 5, 6, 7, 8, 9):
            self.assertEqual(loop_outputs[index], (0., 0., 0.))
        self.assertEqual(loop_outputs[2], (0., 0., 0.))
        self.assertTrue(callback_checks)
        self.assertEqual(len(held_paths), 1)
        np.testing.assert_allclose(held_paths[0], [[0., 0.], [1., 0.]])
        self.assertEqual(reports[9]['map_memory']['reset_count'], 1)
        self.assertEqual(reports[9]['map_memory']['last_reset_reason'], 'ODOM_RESET')
        self.assertEqual(output[-3:], [(0., 0., 0.)]*3)
        self.assertEqual(reports[11]['goal_mode'], 'point')
        self.assertEqual(reports[12]['target'], [6., 0.], 'Camera target overrode clicked goal')
        self.assertNotEqual(reports[12]['state'], 'WAIT_TARGET')
        self.assertFalse(reports[13]['enabled'])
        self.assertEqual(reports[14]['goal_mode'], 'vision')
        self.assertEqual(reports[15]['goal_error'], 'GOAL_BLOCKED_OR_UNKNOWN')
        self.assertFalse(reports[15]['enabled'])

    def test_node_search_mode_spins_without_target_then_approaches(self):
        ns = types.SimpleNamespace
        callbacks, services, outputs, reports = {}, {}, [], []
        clock, cycle = [100.], [0]
        ros = types.ModuleType('rospy')
        params = {'/jetauto_collision_monitor/scan_frame': 'laser', '~control_hz': 1000.}
        ros.init_node = ros.loginfo = lambda *a, **k: None
        ros.get_param = lambda name, default: params.get(name, default)
        ros.resolve_name = lambda name: name
        ros.on_shutdown = lambda callback: None
        ros.Time = ns(now=lambda: ns(to_sec=lambda: clock[0]))
        twist = lambda: ns(linear=ns(x=0., y=0., z=0.), angular=ns(x=0., y=0., z=0.))

        def publisher(topic, *a, **k):
            def publish(message):
                if topic == '/collision_monitor/cmd_vel_raw':
                    outputs.append((cycle[0], message.angular.z))
                elif topic == '~state':
                    reports.append(json.loads(message.data))
            return ns(publish=publish)
        ros.Publisher = publisher
        ros.Subscriber = lambda topic, cls, callback, **k: callbacks.__setitem__(topic, callback) or ns(unregister=lambda: None)
        ros.Service = lambda topic, cls, callback: services.__setitem__(topic, callback) or ns(shutdown=lambda: None)

        def feed(stamp):
            callbacks['/odom'](ns(header=ns(frame_id='odom', stamp=ns(to_sec=lambda: stamp)),
                                  child_frame_id='base_footprint', pose=ns(pose=ns(
                                      position=ns(x=0., y=0.), orientation=ns(x=0., y=0., z=0., w=1.)))))
            callbacks['/scan'](ns(header=ns(frame_id='laser', stamp=ns(to_sec=lambda: stamp)),
                                  ranges=[float('inf')]*360, angle_min=-math.pi,
                                  angle_increment=math.pi/180, range_min=.1, range_max=6.))
            callbacks['/jetauto_collision_monitor/state'](ns(data=json.dumps({'state': 'IDLE'})))

        def running():
            index = cycle[0]
            cycle[0] += 1
            if index >= 14:
                return True
            clock[0] = stamp = 100.+index*.01
            if index == 0:
                return False
            feed(stamp)
            if index == 2:
                callbacks['~map_goal'](ns(data=json.dumps(dict(mode='search'))))
            if index >= 9:
                callbacks['/vision/target'](ns(data=json.dumps(dict(
                    valid=True, frame_id='odom', stamp=stamp, x=2., y=0., label='person'))))
            if index == 13:
                callbacks['~map_goal'](ns(data=json.dumps(dict(mode='vision'))))
            return False
        ros.is_shutdown = running
        modules = {'rospy': ros}
        for name, items in {
                'geometry_msgs.msg': {'Twist': twist, 'PoseStamped': lambda: ns(header=ns(), pose=ns(position=ns(), orientation=ns()))},
                'nav_msgs.msg': {'Odometry': object, 'Path': lambda: ns(header=ns(), poses=[]),
                                 'OccupancyGrid': lambda: ns(header=ns(), info=ns(origin=ns(position=ns(), orientation=ns())))},
                'sensor_msgs.msg': {'LaserScan': object},
                'std_msgs.msg': {'String': lambda **kw: ns(**kw)},
                'std_srvs.srv': {'SetBool': object, 'SetBoolResponse': lambda **kw: ns(**kw)}}.items():
            parent, child = name.split('.')
            modules.setdefault(parent, types.ModuleType(parent))
            modules[name] = types.ModuleType(name)
            for key, value in items.items():
                setattr(modules[name], key, value)
            setattr(modules[parent], child, modules[name])
        with patch.dict(sys.modules, modules):
            adapter.main()
        states = [report['state'] for report in reports]
        modes = [report['goal_mode'] for report in reports]
        self.assertIn('search', modes)
        # No target, no enable button: the search itself enables motion and starts its
        # spin scan (first dwelling 1.2 s wall time, so no turn within this fast test;
        # turning is covered by test_search_ros1 in the simulation).
        self.assertIn('SEARCH_SPIN', states[:9])
        self.assertTrue(all(r['enabled'] for r in reports if r['goal_mode'] == 'search'))
        searching = [r for r in reports if r['goal_mode'] == 'search']
        self.assertTrue(all(r['map_memory']['search_mode'] for r in searching))
        self.assertTrue(all(r['map_memory']['memory_s'] == 120. for r in searching))
        self.assertIsNotNone(searching[-1]['search'])
        self.assertEqual(searching[-1]['search']['state'], 'APPROACH')
        self.assertFalse(reports[-1]['map_memory']['search_mode'])
        self.assertEqual(reports[-1]['map_memory']['memory_s'], NavConfig.DEFAULTS['memory_s'])


if __name__ == '__main__':
    unittest.main()
