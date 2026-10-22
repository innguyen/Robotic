"""Offline tests; no ROS installation or robot required."""
import math
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np

from collision_monitor_ros1 import Config, Monitor, scan_points, translation_sweep_clear
import collision_monitor_ros1 as module


class MonitorTests(unittest.TestCase):
    def test_exact_translation_passes_close_parallel_obstacle_without_padding(self):
        self.assertTrue(translation_sweep_clear([[.1, .1901]], (0., 0.), (.2, 0.), 0., .222, .19))

    def test_exact_translation_rejects_obstacle_between_endpoints(self):
        self.assertFalse(translation_sweep_clear([[.5, 0.]], (0., 0.), (1., 0.), 0., .222, .19))

    def test_exact_translation_allows_moving_away_from_outside_buffer(self):
        self.assertTrue(translation_sweep_clear([[.223, 0.]], (0., 0.), (-.1, 0.), 0., .222, .19))

    def monitor(self, points=(), command=(0.18, 0.0, 0.0), **kwargs):
        monitor = Monitor(Config(**kwargs))
        monitor.set_scan(points, 10.0)
        monitor.set_command(*command, now=10.0)
        return monitor

    def test_clear(self):
        self.assertEqual(self.monitor().evaluate(10.1), ((0.18, 0, 0), 'PASS'))

    def test_stop_at_front(self):
        self.assertEqual(self.monitor([(0.20, 0)]).evaluate(10.1)[1], 'STOP_ZONE')

    def test_stop_at_side(self):
        self.assertEqual(self.monitor([(0, 0.18)], (0, .12, 0)).evaluate(10.1)[1], 'STOP_ZONE')

    def test_slowdown(self):
        velocity, state = self.monitor([(0.35, 0)]).evaluate(10.1)
        self.assertEqual(state, 'SLOWDOWN')
        self.assertAlmostEqual(velocity[0], .18 * .35)

    def test_clear_speed_limits_preserve_diagonal(self):
        velocity, state = self.monitor(command=(.3, .3, 0)).evaluate(10.1)
        self.assertEqual(state, 'LIMIT')
        self.assertAlmostEqual(velocity[0], .12)
        self.assertAlmostEqual(velocity[1], .12)

    def test_reverse_blind(self):
        self.assertEqual(self.monitor(command=(-.05, 0, 0)).evaluate(10.1)[1], 'UNOBSERVED_DIRECTION')

    def test_rotation_blind(self):
        self.assertEqual(self.monitor(command=(0, 0, .3)).evaluate(10.1)[1], 'UNOBSERVED_DIRECTION')

    def test_swept_rotation_hits_corner(self):
        # Outside initial rectangle, inside rectangle after rotating ~45 degrees.
        monitor = self.monitor([(.01, .24)], (0, 0, .5),
                               blind_sectors_deg=[], slow_margin=.061,
                               reaction_time=1.5)
        velocity, state = monitor.evaluate(10.1)
        self.assertIn(state, ('APPROACH', 'PREDICTED_COLLISION'))
        self.assertLess(abs(velocity[2]), .5)

    def test_approach_without_fixed_slow_zone(self):
        monitor = self.monitor([(.30, 0)], slow_margin=.061, reaction_time=1.0)
        velocity, state = monitor.evaluate(10.1)
        self.assertIn(state, ('APPROACH', 'PREDICTED_COLLISION'))
        self.assertLess(velocity[0], .18)

    def test_missing_scan(self):
        self.assertEqual(Monitor(Config()).evaluate(10)[1], 'WAIT_SCAN')

    def test_stale_scan(self):
        m = self.monitor()
        m.set_command(.1, 0, 0, 10.6)
        self.assertEqual(m.evaluate(10.6)[1], 'STALE_SCAN')

    def test_stale_command(self):
        m = self.monitor()
        m.set_scan([], 10.5)
        self.assertEqual(m.evaluate(10.5), ((0, 0, 0), 'STALE_COMMAND'))

    def test_invalid_command_does_not_reuse_previous(self):
        m = self.monitor()
        m.set_command(float('nan'), 0, 0, 10.1)
        self.assertEqual(m.evaluate(10.1), ((0, 0, 0), 'INVALID_COMMAND'))

    def test_invalid_scan_does_not_reuse_previous(self):
        m = self.monitor()
        m.invalidate_scan('BAD_SCAN')
        self.assertEqual(m.evaluate(10.1), ((0, 0, 0), 'BAD_SCAN'))

    def test_future_receipt_rejected(self):
        self.assertEqual(self.monitor().evaluate(9.0)[1], 'STALE_SCAN')

    def test_idle(self):
        self.assertEqual(self.monitor(command=(0, 0, 0)).evaluate(10.1)[1], 'IDLE')

    def test_infinite_no_return_is_not_obstacle(self):
        cfg = Config(blind_sectors_deg=[])
        pts = scan_points([float('inf')] * 360, -math.pi, math.pi/180, .1, 6, cfg)
        self.assertEqual(pts.shape, (0, 2))

    def test_invalid_scan_rejected(self):
        for values in ([0] * 360, [float('nan')] * 360, [float('-inf')] * 360):
            with self.assertRaises(ValueError):
                scan_points(values, -math.pi, math.pi/180, .1, 6, Config())

    def test_extrinsic_transform(self):
        pts = scan_points([float('inf'), 1., float('inf')], -.1, .1, .1, 6,
                          Config(blind_sectors_deg=[], scan_yaw=math.pi/2, scan_x=.096))
        np.testing.assert_allclose(pts, [[.096, 1.]], atol=1e-8)

    def test_blind_sector_points_excluded(self):
        pts = scan_points([1.] * 360, -math.pi, math.pi/180, .1, 6, Config())
        self.assertTrue(0 < len(pts) < 360)

    def test_invalid_configuration(self):
        for config in ({'rate': 0}, {'brake_decel': 0}, {'stop_margin': .3},
                       {'max_angular': float('nan')}, {'block_blind_motion': 'false'}):
            with self.assertRaises(ValueError):
                Config(**config)


class NodeTests(unittest.TestCase):
    def test_ros_adapter_rejects_frame_stale_repeated_scan_and_invalid_commands(self):
        def twist():
            return types.SimpleNamespace(linear=types.SimpleNamespace(x=0., y=0., z=0.),
                                         angular=types.SimpleNamespace(x=0., y=0., z=0.))

        ros = types.ModuleType('rospy')
        callbacks, outputs, states = {}, [], []
        clock, cycle = [100.], [0]
        params = {'~scan_frame': 'laser', '~rate': 1000.}
        ros.init_node = lambda *args, **kwargs: None
        ros.get_param = lambda key, default: params.get(key, default)
        ros.resolve_name = lambda name: name
        ros.loginfo = ros.logwarn = lambda *args: None
        ros.on_shutdown = lambda cb: None
        ros.Time = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: clock[0]))

        def publisher(topic, *args, **kwargs):
            def publish(msg):
                if topic.endswith('checked'):
                    outputs.append((msg.linear.x, msg.linear.y, msg.angular.z))
                else:
                    states.append(msg.data)
            return types.SimpleNamespace(publish=publish)

        def subscriber(topic, cls, callback, **kwargs):
            callbacks[topic] = callback
            return types.SimpleNamespace(unregister=lambda: None)

        ros.Publisher, ros.Subscriber = publisher, subscriber

        def running():
            index = cycle[0]
            cycle[0] += 1
            if index >= 6:
                return True
            clock[0] = 100. + index * .01
            stamp = clock[0]
            frame = 'wrong' if index == 1 else 'laser'
            if index == 2:
                stamp -= 1.
            if index == 4:
                stamp = 100.03  # repeat previous successful scan
            scan = types.SimpleNamespace(header=types.SimpleNamespace(
                frame_id=frame, stamp=types.SimpleNamespace(to_sec=lambda: stamp)),
                ranges=[float('inf')] * 360, angle_min=-math.pi,
                angle_increment=math.pi/180, range_min=.1, range_max=6.)
            callbacks['/scan'](scan)
            command = twist()
            command.linear.x = .1
            if index == 5:
                command.angular.x = float('nan')
            callbacks['/collision_monitor/cmd_vel_raw'](command)
            return False

        ros.is_shutdown = running
        modules = {'rospy': ros}
        for package, name, cls in [('geometry_msgs', 'Twist', twist),
                                   ('sensor_msgs', 'LaserScan', object),
                                   ('std_msgs', 'String', lambda **kwargs: types.SimpleNamespace(**kwargs))]:
            pkg, msg = types.ModuleType(package), types.ModuleType(package + '.msg')
            setattr(msg, name, cls)
            pkg.msg = msg
            modules[package], modules[package + '.msg'] = pkg, msg
        with patch.dict(sys.modules, modules):
            module.main()
        self.assertEqual([x[0] for x in outputs[:6]], [.1, 0, 0, .1, 0, 0])
        self.assertEqual(outputs[-3:], [(0, 0, 0)] * 3)
        self.assertTrue(any('SCAN_FRAME_MISMATCH' in s for s in states))
        self.assertTrue(any('STALE_SCAN_STAMP' in s for s in states))
        self.assertTrue(any('NONINCREASING_SCAN_STAMP' in s for s in states))
        self.assertTrue(any('INVALID_COMMAND' in s for s in states))


if __name__ == '__main__':
    unittest.main()
