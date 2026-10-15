import ast
import io
import math
import os
import struct
import unittest

from motion_safety_supervisor import (SafetySupervisorCore, depth_valid_fraction,
                                      lidar_valid_fraction)


ROOT = os.path.dirname(os.path.abspath(__file__))


def assigned_literal(filename, variable):
    with io.open(os.path.join(ROOT, filename), 'r', encoding='utf-8') as source:
        tree = ast.parse(source.read(), filename=filename)
    for node in tree.body:
        if isinstance(node, ast.Assign):
            if any(isinstance(target, ast.Name) and target.id == variable
                   for target in node.targets):
                return ast.literal_eval(node.value)
    raise AssertionError('{} is not assigned in {}'.format(variable, filename))


class SensorQualityTest(unittest.TestCase):
    def test_all_zero_lidar_is_invalid(self):
        self.assertEqual(lidar_valid_fraction([0.0] * 100, 0.1, 16.0), 0.0)
        self.assertGreater(lidar_valid_fraction([1.0] * 20 + [0.0] * 80, 0.1, 16.0), 0.1)

    def test_depth_zero_and_valid_frames(self):
        zero = b'\0\0' * 1000
        valid = struct.pack('<H', 1200) * 1000
        self.assertEqual(depth_valid_fraction(zero, '16UC1', False, 100, 10), 0.0)
        self.assertGreater(depth_valid_fraction(valid, '16UC1', False, 100, 10), 0.9)


class RuntimeContractTest(unittest.TestCase):
    def test_ai_apps_only_publish_desired_velocity(self):
        expected = ['/jetauto_ai/cmd_vel_desired']
        self.assertEqual(assigned_literal('jetauto_find_object.py', 'CMD_VEL_TOPICS'), expected)
        self.assertEqual(assigned_literal('jetauto_smart_search.py', 'CMD_VEL_TOPICS'), expected)

    def test_systemd_owns_safety_supervisor_in_production(self):
        with io.open(os.path.join(ROOT, 'jetauto-ai.service'), encoding='utf-8') as source:
            ai_service = source.read()
        with io.open(os.path.join(ROOT, 'jetauto-safety.service'), encoding='utf-8') as source:
            safety_service = source.read()
        self.assertIn('Environment=JETAUTO_MANAGE_SAFETY=0', ai_service)
        self.assertIn('Type=notify', safety_service)
        self.assertIn('WatchdogSec=', safety_service)
        self.assertIn('Restart=always', safety_service)


class SafetySupervisorTest(unittest.TestCase):
    def healthy(self, core, now):
        core.update_command(0.12, -0.03, math.radians(10), now)
        core.update_lidar(0.35, now)
        core.update_depth(0.60, now)
        core.update_odom(now)

    def arm(self, core, now=10.0):
        self.healthy(core, now)
        core.set_enabled(False, now)
        core.set_enabled(True, now)
        status = core.evaluate(now + 0.01)
        self.assertTrue(status['armed'])
        return status

    def test_starts_paused_and_outputs_zero(self):
        core = SafetySupervisorCore()
        self.healthy(core, 10.0)
        status = core.evaluate(10.1)
        self.assertFalse(status['armed'])
        self.assertEqual(status['output'], [0.0, 0.0, 0.0])

    def test_forwards_only_when_explicitly_armed(self):
        status = self.arm(SafetySupervisorCore())
        self.assertEqual(status['output'][:2], [0.12, -0.03])

    def test_ai_timeout_latches_until_pause_resume(self):
        core = SafetySupervisorCore(command_timeout=0.4)
        self.arm(core)
        status = core.evaluate(10.6)
        self.assertTrue(status['fault_latched'])
        self.assertEqual(status['output'], [0.0, 0.0, 0.0])
        self.healthy(core, 10.7)
        self.assertFalse(core.evaluate(10.71)['armed'])
        core.set_enabled(False, 10.8)
        core.set_enabled(True, 10.9)
        self.healthy(core, 10.9)
        self.assertTrue(core.evaluate(10.91)['armed'])

    def test_invalid_lidar_latches_stop(self):
        core = SafetySupervisorCore(min_lidar_valid=0.1)
        self.arm(core)
        core.update_lidar(0.0, 10.2)
        status = core.evaluate(10.21)
        self.assertFalse(status['armed'])
        self.assertIn('LIDAR DATA INVALID', status['reason'])

    def test_invalid_or_overspeed_command_latches_stop(self):
        core = SafetySupervisorCore()
        self.arm(core)
        core.update_command(float('nan'), 0.0, 0.0, 10.2)
        status = core.evaluate(10.21)
        self.assertFalse(status['armed'])
        self.assertIn('AI COMMAND INVALID', status['reason'])

        core = SafetySupervisorCore()
        self.arm(core)
        core.update_command(0.36, 0.0, 0.0, 10.2)
        self.assertIn('AI COMMAND INVALID', core.evaluate(10.21)['reason'])

    def test_invalid_or_stale_depth_latches_stop(self):
        core = SafetySupervisorCore(depth_timeout=0.5, min_depth_valid=0.01)
        self.arm(core)
        core.update_depth(0.0, 10.2)
        self.assertIn('DEPTH DATA INVALID', core.evaluate(10.21)['reason'])

        core = SafetySupervisorCore(depth_timeout=0.5)
        self.arm(core)
        core.update_command(0.12, 0.0, 0.0, 10.6)
        core.update_lidar(0.35, 10.6)
        core.update_odom(10.6)
        self.assertIn('DEPTH STALE', core.evaluate(10.6)['reason'])

    def test_stale_odometry_latches_stop(self):
        core = SafetySupervisorCore(odom_timeout=0.5)
        self.arm(core)
        core.update_command(0.12, 0.0, 0.0, 10.6)
        core.update_lidar(0.35, 10.6)
        core.update_depth(0.60, 10.6)
        self.assertIn('ODOMETRY STALE', core.evaluate(10.6)['reason'])

    def test_arm_grace_does_not_forward_unproven_command(self):
        core = SafetySupervisorCore(arm_grace=1.0)
        core.set_enabled(False, 9.9)
        core.set_enabled(True, 10.0)
        status = core.evaluate(10.2)
        self.assertFalse(status['armed'])
        self.assertFalse(status['fault_latched'])
        self.assertTrue(status['reason'].startswith('WAITING:'))

    def test_restart_while_enable_is_high_requires_pause_resume(self):
        core = SafetySupervisorCore()
        self.healthy(core, 10.0)
        core.set_enabled(True, 10.0)
        status = core.evaluate(10.1)
        self.assertFalse(status['armed'])
        self.assertIn('WAITING FOR PAUSE', status['reason'])
        core.set_enabled(False, 10.2)
        core.set_enabled(True, 10.3)
        self.healthy(core, 10.3)
        self.assertTrue(core.evaluate(10.31)['armed'])


if __name__ == '__main__':
    unittest.main()
