"""Regression checks for a search scan outlasting the ordinary map memory."""
import math
import unittest

import numpy as np

from navigation_core_ros1 import Navigator
from search_core_ros1 import SearchMission
from simulate_navigation import robot_config


class SearchMapMemoryTests(unittest.TestCase):
    def navigator(self, search):
        cfg, cc = robot_config()
        nav = Navigator(cfg, cc)
        if search:
            SearchMission(nav)
        return nav

    def observe(self, nav, now, yaw=0., ranges=None, x=0.):
        nav.costmap.update(np.full(61, 1.5) if ranges is None else ranges,
                           -.3, .01, .05, 6., (x, 0., yaw), now)

    def test_search_retains_rear_free_space_and_hits_after_slow_spin(self):
        for search in (False, True):
            nav = self.navigator(search)
            self.observe(nav, 10., math.pi)
            hit = next(iter(nav.costmap.obstacle_points))
            point = (np.asarray(hit)+.5)*nav.cfg.resolution
            # Fresh front scans do not refresh rear observation timestamps.
            self.observe(nav, 35.)
            grid = nav.costmap.snapshot((0., 0., 0.), 35.)
            self.assertEqual(grid.raw[grid.cell((-.9, 0.))], 0 if search else -1)
            self.assertEqual(grid.raw[grid.cell(point)], 100 if search else -1)
            if search:
                self.assertEqual(nav.costmap.cells[hit][1], 10.)
                self.assertGreater(len(grid.obstacles), 0)

    def test_search_memory_is_bounded_and_reports_expiry(self):
        nav = self.navigator(True)
        self.observe(nav, 10., math.pi)
        self.observe(nav, 10.+nav.costmap.memory_s+1.)
        grid = nav.costmap.snapshot((0., 0., 0.), 10.+nav.costmap.memory_s+1.)
        self.assertEqual(grid.raw[grid.cell((-.9, 0.))], -1)
        self.assertGreater(nav.costmap.info()['expired_cells'], 0)

    def test_new_observation_clears_old_obstacle_before_memory_expires(self):
        nav = self.navigator(True)
        self.observe(nav, 10., math.pi)
        hit = next(iter(nav.costmap.obstacle_points))
        for step in range(int(nav.cfg.clear_observations)):
            self.observe(nav, 35.+step*.1, math.pi, np.full(61, np.inf))
        self.assertFalse(nav.costmap.cells[hit][0])
        self.assertNotIn(hit, nav.costmap.obstacle_points)

    def test_leaving_search_restores_normal_expiry(self):
        nav = self.navigator(True)
        self.observe(nav, 10., math.pi)
        nav.costmap.search_mode = False
        grid = nav.costmap.snapshot((0., 0., 0.), 35.)
        self.assertEqual(nav.costmap.memory_s, nav.cfg.memory_s)
        self.assertEqual(grid.raw.max(), -1)

    def test_odom_reset_clears_measured_hits_and_records_reason(self):
        nav = self.navigator(True)
        self.observe(nav, 10.)
        nav.costmap.clear('ODOM_RESET')
        self.assertEqual(nav.costmap.cells, {})
        self.assertEqual(nav.costmap.obstacle_points, {})
        self.assertEqual(nav.costmap.info()['reset_count'], 1)
        self.assertEqual(nav.costmap.info()['last_reset_reason'], 'ODOM_RESET')
        self.assertEqual(nav.costmap.snapshot((0., 0., 0.), 11.).raw.max(), -1)

    def test_search_still_evicts_cells_outside_local_window(self):
        nav = self.navigator(True)
        self.observe(nav, 10.)
        self.observe(nav, 11., x=10.)
        self.assertGreater(nav.costmap.info()['window_evicted_cells'], 0)
        self.assertEqual(nav.costmap.info()['expired_cells'], 0)


if __name__ == '__main__':
    unittest.main()
