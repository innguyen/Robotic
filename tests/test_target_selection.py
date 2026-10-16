import unittest

from target_selection import (choose_target, TargetAvoidance, TargetConfirmation,
                              normalize_target,
                              approach_command, estimate_target_distance,
                              target_standoff_cm, tracking_command)


class TargetSelectionTests(unittest.TestCase):
    def test_single_detection_does_not_interrupt_scan(self):
        confirm = TargetConfirmation()
        bottle = {'box': (100, 200, 50, 100)}
        self.assertFalse(confirm.update(bottle, (640, 480), 1.0))
        self.assertFalse(confirm.update(None, (640, 480), 2.0))
        self.assertFalse(confirm.update(bottle, (640, 480), 2.2))

    def test_two_consistent_detections_confirm_target(self):
        confirm = TargetConfirmation()
        first = {'box': (100, 200, 50, 100)}
        nearby = {'box': (110, 205, 50, 100)}
        elsewhere = {'box': (400, 200, 50, 100)}
        self.assertFalse(confirm.update(first, (640, 480), 1.0))
        self.assertFalse(confirm.update(elsewhere, (640, 480), 1.2))
        self.assertTrue(confirm.update(elsewhere, (640, 480), 1.4))
        confirm.reset()
        self.assertFalse(confirm.update(first, (640, 480), 2.0))
        self.assertTrue(confirm.update(nearby, (640, 480), 2.2))

    def test_phone_aliases_match_coco_class(self):
        for value in ('phone', 'MOBILEPHONE', ' mobile phone ', 'cell_phone', 'điện thoại'):
            self.assertEqual(normalize_target(value), 'cell phone')
        self.assertEqual(normalize_target('chair'), 'chair')

    def test_standoff_and_range_use_object_class_width(self):
        chair = estimate_target_distance(100, 640, 'chair')
        phone = estimate_target_distance(100, 640, 'MOBILEPHONE')
        self.assertGreater(chair, phone * 5)
        self.assertEqual(target_standoff_cm('chair'), 75.)
        self.assertEqual(target_standoff_cm('phone'), 45.)

    def test_diagonal_approach_when_straight_is_blocked(self):
        safe = lambda vx, vy, wz: vx > 0 and vy > 0
        command = approach_command(.15, 9, 1, safe)
        self.assertGreater(command[0], 0)
        self.assertGreater(command[1], 0)
        self.assertTrue(safe(*command))
        self.assertIsNone(approach_command(.15, 9, 1, lambda *args: False))

    def test_slower_forward_before_sideways(self):
        command = approach_command(.15, 0, -1, lambda vx, vy, wz: vx <= .04)
        self.assertEqual(command, (.04, 0., 0.))

    def test_ground_bottle_wins_against_upper_bottle(self):
        high = {'class_name': 'bottle', 'conf': .74, 'box': (350, 100, 120, 210)}
        low = {'class_name': 'bottle', 'conf': .56, 'box': (500, 780, 60, 160)}
        self.assertIs(choose_target([high, low], (800, 1000), None, 'bottle'), low)
        self.assertIs(choose_target([high, low], (800, 1000), (530, 860), 'bottle'), low)

    def test_locked_target_does_not_jump_to_another_bottle(self):
        high = {'class_name': 'bottle', 'conf': .9, 'box': (350, 100, 120, 210)}
        self.assertIsNone(choose_target([high], (800, 1000), (530, 860), 'bottle'))

    def test_person_selection_keeps_confident_detection(self):
        first = {'class_name': 'person', 'conf': .85, 'box': (100, 100, 100, 400)}
        second = {'class_name': 'person', 'conf': .4, 'box': (400, 300, 100, 400)}
        self.assertIs(choose_target([first, second], (800, 1000), None, 'person'), first)


class TargetAvoidanceTests(unittest.TestCase):
    def test_centered_target_is_not_rotated_out_of_view(self):
        avoid = TargetAvoidance()
        rotation_only = lambda vx, vy, wz: vx == 0 and vy == 0
        command = avoid.command(False, (0, 0, 0), 1, rotation_only, 10, align_turn=0)
        self.assertEqual(command[:3], (0., 0., 0.))
        command = avoid.command(False, (0, 0, 0), 1, rotation_only, 12, align_turn=-5)
        self.assertEqual(command[2], -5)

    def test_escape_direction_stays_fixed_during_detection_jitter(self):
        avoid = TargetAvoidance()
        safe = lambda vx, vy, wz: True
        first = avoid.command(False, (0, 0, 0), -1, safe, 10)
        self.assertEqual(first[1], -.08)
        second = avoid.command(True, (0, -.02, 0), 1, safe, 10.2)
        third = avoid.command(False, (0, -.04, 0), 1, safe, 10.4)
        self.assertEqual(second[:3], (0., 0., 0.))
        self.assertEqual(third[1], -.04)
        self.assertEqual(avoid.direction, -1)

    def test_stops_sideways_then_confirms_clear_path_before_forward(self):
        avoid = TargetAvoidance()
        safe = lambda vx, vy, wz: True
        avoid.command(False, (0, 0, 0), -1, safe, 10)
        first_clear = avoid.command(True, (0, -.02, 0), 1, safe, 10.2)
        self.assertEqual(first_clear[:3], (0., 0., 0.))
        self.assertIsNone(avoid.command(True, (0, -.02, 0), 1, safe, 10.4))

    def test_escape_stops_after_bounded_turn(self):
        avoid = TargetAvoidance()
        safe = lambda vx, vy, wz: True
        avoid.command(False, (0, 0, 0), -1, safe, 10)
        avoid.command(False, (0, -.31, 0), 1, safe, 12)
        turn = avoid.command(False, (0, -.31, 0), 1, safe, 16)
        self.assertEqual(turn[2], -18)
        stopped = avoid.command(False, (0, -.31, -1.0), 1, safe, 17)
        self.assertEqual(stopped[:3], (0.0, 0.0, 0.0))
        self.assertEqual(avoid.phase, 'BLOCKED')
        self.assertEqual(avoid.command(True, (0, -.31, -1.0), 1, safe, 17.2)[:3],
                         (0.0, 0.0, 0.0))

    def test_checks_target_after_five_cm_instead_of_ten_or_fifteen(self):
        avoid = TargetAvoidance()
        safe = lambda vx, vy, wz: vy > 0
        first = tracking_command(avoid, .15, 0, (0, 0, 0), 1, safe, 10)
        self.assertEqual(first[:3], (0., .08, 0.))
        at_five_cm = tracking_command(avoid, .15, 0, (0, .051, 0), 1, safe, 10.7)
        self.assertEqual(at_five_cm[:3], (0., 0., 0.))
        safe_forward = lambda vx, vy, wz: vy >= 0
        confirming = tracking_command(avoid, .15, 0, (0, .051, 0), 1, safe_forward, 10.9)
        self.assertEqual(confirming[:3], (0., 0., 0.))
        rejoined = tracking_command(avoid, .15, 0, (0, .051, 0), 1, safe_forward, 11.1)
        self.assertEqual(rejoined[:3], (.15, 0., 0.))
        self.assertEqual(avoid.phase, 'IDLE')

    def test_diagonal_candidate_does_not_erase_active_side_step(self):
        avoid = TargetAvoidance()
        safe = lambda vx, vy, wz: vy > 0
        avoid.command(False, (0, 0, 0), 1, safe, 10, align_turn=0)
        next_command = tracking_command(avoid, .15, 0, (0, .02, 0), -1, safe, 10.2)
        self.assertEqual(next_command[:3], (0., .08, 0.))
        self.assertEqual(avoid.phase, 'STRAFE')
        self.assertEqual(avoid.direction, 1)

    def test_new_side_evidence_unblocks_centered_target_without_wait(self):
        avoid = TargetAvoidance()
        unknown = lambda vx, vy, wz: vx == 0 and vy == 0
        avoid.command(False, (0, 0, 0), 1, unknown, 10, align_turn=0)
        self.assertEqual(avoid.phase, 'BLOCKED')
        side_clear = lambda vx, vy, wz: vy > 0
        result = avoid.command(False, (0, 0, 0), 1, side_clear, 10.1, align_turn=0)
        self.assertEqual(result[:3], (0., .08, 0.))

    def test_unknown_sides_never_authorize_a_side_step(self):
        avoid = TargetAvoidance()
        unknown = lambda vx, vy, wz: vx == 0 and vy == 0
        result = tracking_command(avoid, .15, 0, (0, 0, 0), 1, unknown, 10)
        self.assertEqual(result[:3], (0., 0., 0.))

    def test_real_footprint_rejoins_after_five_cm_clears_obstacle_edge(self):
        import math
        from pathlib import Path
        import numpy as np
        from navigation_safety import FootprintSafety, load_config
        cfg = load_config(str(Path(__file__).resolve().parents[1] / 'navigation_config.json'))
        guard = FootprintSafety(cfg)
        avoid = TargetAvoidance()
        obstacle = (.22, -.175)

        def command(y, now):
            rel = (obstacle[0], obstacle[1] - y)
            ranges = np.full(720, 3.)
            index = round((math.atan2(rel[1], rel[0]) + math.pi) / (math.pi / 360))
            ranges[index] = math.hypot(*rel)
            guard.update(ranges, -math.pi, math.pi / 360, .12, 8., now)
            check = lambda vx, vy, wz: guard.check(vx, vy, wz, now)[0]
            return tracking_command(avoid, .15, 0., (0., y, 0.), 1, check, now)

        self.assertEqual(command(0., 100.)[:3], (0., .08, 0.))
        self.assertEqual(command(.051, 100.7)[:3], (0., 0., 0.))
        self.assertEqual(command(.051, 100.9)[:3], (.15, 0., 0.))

    def test_selects_one_alternate_side_if_preferred_is_blocked(self):
        avoid = TargetAvoidance()
        safe = lambda vx, vy, wz: vy >= 0
        first = avoid.command(False, (0, 0, 0), -1, safe, 10)
        second = avoid.command(False, (0, .01, 0), -1, safe, 10.2)
        self.assertEqual(first[1], .08)
        self.assertEqual(second[1], .08)


if __name__ == '__main__':
    unittest.main()
