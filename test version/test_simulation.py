import unittest
import numpy as np
from simulate_navigation import Simulation


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
