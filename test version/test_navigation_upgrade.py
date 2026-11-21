"""Regression cases for real-log failures and persistent-map navigation (offline)."""
import math
import unittest
from concurrent.futures import Future
from unittest.mock import patch

import numpy as np

from collision_monitor_ros1 import Config
from navigation_core_ros1 import Navigator, NavConfig, Costmap, plan_approach
from map_core_ros1 import MappedNavigator, OccupancyMap
from search_core_ros1 import SearchMission
from test_navigation_ros1 import known_map


class StabilityTests(unittest.TestCase):
    def test_stuck_remains_stopped_when_heading_changes(self):
        nav = Navigator(NavConfig(memory_s=100.))
        known_map(nav)
        nav.step((0., 0., 0.), (2., 0.), .35, 10., .1)
        self.assertEqual(nav.step((0., 0., 0.), (2., 0.), .35, 19., .1)[1], 'STUCK')
        for yaw in (1.6, -2., math.pi):
            self.assertEqual(nav.step((0., 0., yaw), (2., 0.), .35, 19.1, .1),
                             ((0., 0., 0.), 'STUCK'))

    def test_rotation_without_actual_yaw_progress_times_out(self):
        nav = Navigator(NavConfig(memory_s=100.))
        known_map(nav)
        for step in range(40):
            command, reason = nav.step((0., 0., 0.), (-2., 0.), .35, 10.+step*.1, .1)
        self.assertEqual((command, reason), ((0., 0., 0.), 'STUCK'))

    def test_rotation_hysteresis_keeps_turning_below_entry_angle(self):
        nav = Navigator()
        known_map(nav)
        goal = (2., 0.)
        self.assertEqual(nav.step((0., 0., -math.radians(46)), goal, .35, 10., .1)[1], 'ROTATE')
        self.assertEqual(nav.step((0., 0., -math.radians(40)), goal, .35, 10.1, .1)[1], 'ROTATE')
        self.assertNotEqual(nav.step((0., 0., -math.radians(29)), goal, .35, 10.2, .1)[1], 'ROTATE')

    def test_obstacle_behind_does_not_discard_remaining_route(self):
        nav = Navigator()
        known_map(nav)
        nav.step((0., 0., 0.), (2., 0.), .35, 10., .1)
        before = nav.plan_count
        nav.costmap.cells[(0, 0)] = (True, 10.1)
        command, reason = nav.step((.8, 0., 0.), (2., 0.), .35, 10.1, .1)
        self.assertEqual(reason, 'FOLLOW')
        self.assertEqual(nav.plan_count, before)
        self.assertGreater(command[0], 0.)

    def test_failed_background_replan_keeps_valid_route(self):
        nav = Navigator(async_planning=True)
        known_map(nav)
        try:
            nav.step((0., 0., 0.), (2., 0.), .35, 10., .1)
            nav.plan_future.result(timeout=5)
            nav.step((0., 0., 0.), (2., 0.), .35, 10.1, .1)
            original = [p.copy() for p in nav.path]
            failed = Future()
            failed.set_result((([], 'PLAN_BUDGET_EXCEEDED', 10), 1.))
            nav.plan_future, nav.future_generation = failed, nav.generation
            self.assertEqual(nav.step((0., 0., 0.), (2., 0.), .35, 10.2, .1)[1], 'FOLLOW')
            np.testing.assert_allclose(nav.path, original)
        finally:
            nav.close()

    def test_noisy_free_ray_does_not_erase_obstacle(self):
        costmap = Costmap(NavConfig(clear_observations=3), Config(blind_sectors_deg=[]))
        def scan(ranges, now):
            costmap.update(ranges, -.01, .01, .1, 6., (0., 0., 0.), now)
        scan([np.inf, 1., np.inf], 10.)
        hit = next(iter(costmap.obstacle_points))
        scan([np.inf]*3, 10.1)
        self.assertTrue(costmap.cells[hit][0])
        scan([np.inf, 1., np.inf], 10.2)  # a hit resets clearing evidence
        for now in (10.3, 10.4):
            scan([np.inf]*3, now)
            self.assertTrue(costmap.cells[hit][0])
        scan([np.inf]*3, 10.5)
        self.assertFalse(costmap.cells[hit][0])

    def test_planning_work_has_an_expansion_limit(self):
        nav = Navigator()
        known_map(nav)
        result = plan_approach(nav.costmap.snapshot((0., 0., 0.), 10.),
                               (0., 0.), (2., 0.), .5, 0., nav.cc, max_expansions=3)
        self.assertEqual(result[1], 'PLAN_BUDGET_EXCEEDED')


class PersistentMapTests(unittest.TestCase):
    def test_invalid_or_missing_goal_stops_instead_of_crashing(self):
        nav = MappedNavigator()
        nav.set_global_map(OccupancyMap(np.zeros((40, 40)), .1, (-2., -2., 0.)))
        for pose, target in (((0., 0., 0.), None), (None, (1., 0.)),
                             ((0., 0., 0.), ('invalid', 0.)),
                             ((0., 0., 0.), (1.,)), ((0., 0., 0.), (np.nan, 0.))):
            with self.subTest(pose=pose, target=target):
                self.assertEqual(nav.step(pose, target, .55, 10., .1),
                                 ((0., 0., 0.), 'INVALID_GOAL'))

    def test_rotated_map_origin_samples_correct_cells(self):
        occupancy = OccupancyMap([[0, 100], [-1, 0]], 1., (4., 5., math.pi/2))
        np.testing.assert_equal(occupancy.sample([[3.5, 5.5], [3.5, 6.5], [2.5, 5.5], [9, 9]]),
                                [0, 100, -1, -1])

    def test_static_map_survives_ttl_rays_and_local_reset(self):
        nav = MappedNavigator(NavConfig(clear_observations=3), Config(blind_sectors_deg=[]))
        raw = np.zeros((80, 80), dtype=np.int8)
        raw[40, 50] = 100
        occupancy = OccupancyMap(raw, .06, (-2.4, -2.4, 0.))
        nav.set_global_map(occupancy)
        nav.costmap.update([np.inf]*360, -math.pi, math.pi/180, .1, 6., (0., 0., 0.), 10.)
        nav.costmap.clear('LOCALIZATION_CORRECTION')
        grid = nav.costmap.snapshot((0., 0., 0.), 10000.)
        self.assertEqual(grid.raw[grid.cell((.63, .03))], 100)
        self.assertEqual(grid.raw[grid.cell((-.9, 0.))], 0)
        self.assertTrue(nav.costmap.info()['persistent'])

    def test_global_route_detours_around_wall_beyond_local_window(self):
        nav = MappedNavigator(NavConfig(max_plan_expansions=40000))
        raw = np.zeros((80, 210), dtype=np.int8)
        raw[0:48, 100] = 100
        occupancy = OccupancyMap(raw, .1, (-1., -3., 0.))
        nav.set_global_map(occupancy)
        path, reason, visited = occupancy.plan((0., 0., 0.), (17., 0.), 0., nav.cfg, nav.cc, True)
        self.assertEqual(reason, 'MAP_POINT')
        self.assertTrue(path)
        self.assertGreater(max(p[1] for p in path), 1.8)
        self.assertGreater(visited, 100)

    def test_unknown_map_is_never_traversed(self):
        nav = MappedNavigator()
        raw = np.zeros((40, 80), dtype=np.int8)
        raw[:, 38:42] = -1
        occupancy = OccupancyMap(raw, .1, (-1., -2., 0.))
        path, reason, _ = occupancy.plan((0., 0., 0.), (5., 0.), 0., nav.cfg, nav.cc, True)
        self.assertEqual(path, [])

    def test_exact_local_geometry_can_leave_a_circle_blocked_start(self):
        nav = MappedNavigator()
        raw = np.zeros((100, 200), dtype=np.int8)
        raw[12, :] = 100
        nav.set_global_map(OccupancyMap(raw, .02, (-.5, -.5, 0.)))
        command, reason = nav.step((0., 0., 0.), (2., 0.), 0., 10., .1, True)
        self.assertEqual(nav.global_reason, 'START_BLOCKED')
        self.assertEqual(reason, 'FOLLOW')
        self.assertGreater(command[0], 0.)

    def test_new_map_discards_old_async_global_result(self):
        nav = MappedNavigator(async_planning=True)
        try:
            nav.set_global_map(OccupancyMap(np.zeros((80, 160)), .1, (-2., -4., 0.)))
            nav.step((0., 0., 0.), (8., 0.), 0., 10., .1, True)
            old = nav.global_future
            old.result(timeout=5.)
            raw = np.zeros((80, 160)); raw[:, 55] = 100
            nav.set_global_map(OccupancyMap(raw, .1, (-2., -4., 0.)))
            command, reason = nav.step((0., 0., 0.), (8., 0.), 0., 10.1, .1, True)
            self.assertEqual(command, (0., 0., 0.))
            self.assertFalse(nav.global_path)
        finally:
            nav.close()

    def test_new_map_keeps_valid_route_but_invalidates_a_blocked_one(self):
        nav = MappedNavigator()
        raw = np.zeros((80, 160), dtype=np.int8)
        nav.set_global_map(OccupancyMap(raw, .1, (-2., -4., 0.)))
        nav.step((0., 0., 0.), (8., 0.), 0., 10., .1, True)
        original = np.asarray(nav.global_path).copy()
        generation = nav.global_generation
        raw[4, 4] = 100  # unrelated new wall observation
        nav.set_global_map(OccupancyMap(raw, .1, (-2., -4., 0.)))
        np.testing.assert_allclose(nav.global_path, original)
        self.assertEqual(nav.global_generation, generation)
        raw[:, 55] = 100  # route is now blocked
        nav.set_global_map(OccupancyMap(raw, .1, (-2., -4., 0.)))
        self.assertEqual(nav.global_path, [])
        self.assertGreater(nav.global_generation, generation)


class FoundTests(unittest.TestCase):
    def test_memory_only_target_never_reports_found(self):
        mission = SearchMission(Navigator())
        for i in range(20):
            command, reason = mission.step((0., 0., 0.), (.7, 0.), .55, 10.+i*.1, .1,
                target_visible=False, target_age=1., measured_velocity=(0., 0., 0.))
            self.assertEqual(command, (0., 0., 0.))
            self.assertEqual(reason, 'VERIFY_TARGET')
        self.assertIsNone(mission.found)

    def test_found_requires_fresh_view_and_measured_stop_for_dwell(self):
        mission = SearchMission(Navigator())
        for i in range(10):
            self.assertEqual(mission.step((0., 0., 0.), (.7, 0.), .55, 10.+i*.1, .1,
                measured_velocity=(.1, 0., 0.))[1], 'VERIFY_TARGET')
        for i in range(9):
            command, reason = mission.step((0., 0., 0.), (.7, 0.), .55, 11.+i*.1, .1,
                                           measured_velocity=(0., 0., 0.))
        self.assertEqual(reason, 'FOUND')


if __name__ == '__main__':
    unittest.main()
