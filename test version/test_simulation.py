import unittest
import numpy as np
from simulate_navigation import Simulation, robot_config


class SimulationTests(unittest.TestCase):
    def test_scan_returns_front_face_and_occludes_back_face(self):
        sim = Simulation(scenario='empty')
        sim.boxes = [[1., -.2, 1.2, .2], [2., -.2, 2.2, .2]]
        self.assertAlmostEqual(sim.scan()[360], 1.-sim.nav.cc.scan_x)

    def test_default_gap_reaches_standoff(self):
        sim = Simulation()
        for _ in range(300):
            sim.step()
            if sim.state == 'ARRIVED':
                break
        self.assertEqual(sim.state, 'ARRIVED')
        self.assertGreater(np.linalg.norm(sim.pose[:2]), 1.)

    def test_room_with_robot_yaml_reaches_target_behind_box(self):
        # Route below the box is initially in the rear-right blind sector: needs spin recovery.
        try:
            config = robot_config()
        except ImportError:
            self.skipTest('PyYAML not installed')
        sim = Simulation(scenario='room', config=config)
        states = set()
        for _ in range(500):
            sim.step()
            states.add(sim.state)
            if sim.state in ('ARRIVED', 'STUCK'):
                break
        self.assertEqual(sim.state, 'ARRIVED')
        self.assertIn('RECOVERY_ROTATE', states)

    def test_robot_like_detour_with_async_planner_does_not_crawl_to_stuck(self):
        # Regression: near obstacles the start cell centre is blocked; invalid_path used to
        # discard every async path, alternating FOLLOW/PLANNING until STUCK.
        sim = Simulation(scenario='detour', config=robot_config())
        for _ in range(400):
            sim.step()
            if sim.state in ('ARRIVED', 'STUCK'):
                break
        self.assertEqual(sim.state, 'ARRIVED')

    def test_map_point_rejected_only_when_observed_blocked(self):
        sim = Simulation(scenario='detour')
        self.assertFalse(sim.set_goal((.81, 0.), exact=True))   # on the observed box face
        self.assertEqual(sim.goal_error, 'GOAL_BLOCKED_OR_UNKNOWN')
        self.assertTrue(sim.set_goal((1.5, 0.), exact=True))    # shadow behind the box: explored
        self.assertTrue(sim.set_goal((.5, .5), exact=True))

    def test_map_point_in_unobserved_shadow_is_reached(self):
        sim = Simulation(scenario='detour', config=robot_config())
        self.assertTrue(sim.set_goal((1.5, 0.), exact=True))
        for _ in range(500):
            sim.step()
            if sim.state in ('ARRIVED', 'STUCK'):
                break
        self.assertEqual(sim.state, 'ARRIVED')
        self.assertLess(np.linalg.norm(sim.pose[:2]-(1.5, 0.)), .15)

    def test_map_point_behind_robot_turns_and_is_reached(self):
        # Behind the robot is the LiDAR's blind rear sector: unobserved at the start.
        sim = Simulation(scenario='empty', config=robot_config())
        self.assertTrue(sim.set_goal((-1.2, -.3), exact=True))
        states = set()
        for _ in range(600):
            sim.step()
            states.add(sim.state)
            if sim.state in ('ARRIVED', 'STUCK'):
                break
        self.assertIn('ROTATE', states)
        self.assertEqual(sim.state, 'ARRIVED')
        self.assertLess(np.linalg.norm(sim.pose[:2]-(-1.2, -.3)), .15)

    def test_monitor_stops_when_obstacle_in_stop_zone(self):
        sim = Simulation(scenario='empty')
        sim.boxes = [[.20, -.08, .30, .08]]
        sim.observe()
        sim.monitor.set_command(.1, 0., 0., sim.time)
        command, state = sim.monitor.evaluate(sim.time)
        self.assertEqual(state, 'STOP_ZONE')
        self.assertEqual(command, (0., 0., 0.))


if __name__ == '__main__':
    unittest.main()
