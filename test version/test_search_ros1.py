"""Offline tests for the search mission (search_core_ros1); no ROS or robot required."""
import math
import unittest

import numpy as np

from navigation_core_ros1 import Navigator
from search_core_ros1 import SearchConfig, SearchMission
from simulate_navigation import Simulation, robot_config


def run(sim, limit=4000):
    states = []
    for _ in range(limit):
        sim.step()
        states.append(sim.state)
        if sim.state in ('FOUND', 'SEARCH_COMPLETE'):
            break
    return states


class CoverageTests(unittest.TestCase):
    def test_camera_coverage_is_a_forward_wedge_cut_at_the_first_hit(self):
        mission = SearchMission(Navigator())
        ranges = np.full(360, np.inf)
        angles = np.radians(np.arange(-180, 180))
        ranges[np.abs(angles) < np.radians(3)] = 1.0            # wall 1 m straight ahead
        mission.update_coverage(ranges, -math.pi, math.radians(1), .05, (0., 0., 0.))
        res = mission.nav.cfg.resolution
        cell = lambda x, y: (int(math.floor(x/res)), int(math.floor(y/res)))
        self.assertIn(cell(.6, 0.), mission.covered)
        self.assertNotIn(cell(1.6, 0.), mission.covered)        # behind the wall
        self.assertIn(cell(2., 1.), mission.covered)            # 26.6 deg: inside the 29 deg half view
        self.assertNotIn(cell(0., 1.), mission.covered)         # 90 deg: outside the camera view
        self.assertNotIn(cell(-1., 0.), mission.covered)        # behind the robot

    def test_config_rejects_unknown_or_invalid(self):
        with self.assertRaises(ValueError):
            SearchConfig(nope=1.)
        with self.assertRaises(ValueError):
            SearchConfig(spin_step_deg=0.)


class MissionTests(unittest.TestCase):
    def test_finds_person_hidden_behind_furniture(self):
        sim = Simulation(scenario='search', config=robot_config())
        states = run(sim)
        self.assertEqual(sim.state, 'FOUND')
        self.assertIn('SEARCH_SPIN', states)
        self.assertIn('SEARCH_EXPLORE', states)
        np.testing.assert_allclose(sim.search.found, sim.person)
        # Stopped at the person standoff, never inside the person's body.
        self.assertGreater(np.linalg.norm(sim.pose[:2]-sim.person), .4)

    def test_empty_room_search_terminates(self):
        sim = Simulation(scenario='search', config=robot_config())
        sim.start_search(None)
        run(sim, limit=6000)
        self.assertEqual(sim.state, 'SEARCH_COMPLETE')
        self.assertGreater(sim.search.info()['covered_m2'], 6.)

    def test_lost_target_revisits_last_sighting(self):
        sim = Simulation(scenario='empty', config=robot_config())
        sim.start_search((1.5, 0.))
        mission = sim.search
        # Seen once, then gone (e.g. walked away): the next goal is just before that spot.
        command, state = mission.step(sim.pose, (1.5, 0.), .55, sim.time, .1)
        self.assertEqual(mission.state, 'APPROACH')
        command, state = mission.step(sim.pose, None, .55, sim.time+.1, .1)
        self.assertEqual(mission.goal_kind, 'LAST_SEEN')
        np.testing.assert_allclose(mission.goal, (1.5-mission.cfg.lost_standoff, 0.), atol=1e-9)

    def test_dithering_goal_is_dropped_when_route_stops_shrinking(self):
        mission = SearchMission(Navigator(), SearchConfig(stall_s=5.))
        mission._goto((2., 0.), 'EXPLORE', 0.)
        pose = (0., 0., 0.)
        # Turning back and forth: never STUCK or a failure, but the route stays 2 m long.
        mission.nav.step = lambda *args, **kwargs: ((0., 0., .3), 'ROTATE')
        mission.nav.path = [np.array([1., 0.]), np.array([2., 0.])]
        now = 0.
        while now < 5.:
            now += .1
            mission.step(pose, None, .55, now, .1)
            self.assertEqual(len(mission.blacklist), 0)
        mission.step(pose, None, .55, now+.2, .1)
        self.assertEqual(len(mission.blacklist), 1)
        np.testing.assert_allclose(mission.blacklist[0], (2., 0.))

    def test_explore_goal_counts_as_reached_nearby(self):
        mission = SearchMission(Navigator())
        mission._goto((.25, 0.), 'EXPLORE', 0.)
        command, state = mission.step((0., 0., 0.), None, .55, .1, .1)
        self.assertEqual((command, state, mission.state), ((0., 0., 0.), 'SEARCH_SPIN', 'SPIN'))


if __name__ == '__main__':
    unittest.main()
