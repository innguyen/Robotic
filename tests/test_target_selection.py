import unittest

from target_selection import (choose_target, TargetAvoidance, normalize_target,
                              approach_command, estimate_target_distance,
                              target_standoff_cm)


class TargetSelectionTests(unittest.TestCase):
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
        self.assertEqual(second[1], -.08)
        self.assertEqual(third[1], -.08)
        self.assertEqual(avoid.direction, -1)

    def test_requires_three_clear_frames_before_forward(self):
        avoid = TargetAvoidance()
        safe = lambda vx, vy, wz: True
        avoid.command(False, (0, 0, 0), -1, safe, 10)
        self.assertIsNotNone(avoid.command(True, (0, -.02, 0), 1, safe, 10.2))
        self.assertIsNotNone(avoid.command(True, (0, -.04, 0), 1, safe, 10.4))
        self.assertIsNone(avoid.command(True, (0, -.06, 0), 1, safe, 10.6))

    def test_escape_stops_after_bounded_turn(self):
        avoid = TargetAvoidance()
        safe = lambda vx, vy, wz: True
        avoid.command(False, (0, 0, 0), -1, safe, 10)
        turn = avoid.command(False, (0, -.31, 0), 1, safe, 12)
        self.assertEqual(turn[2], -18)
        stopped = avoid.command(False, (0, -.31, -1.0), 1, safe, 13)
        self.assertEqual(stopped[:3], (0.0, 0.0, 0.0))
        self.assertEqual(avoid.phase, 'BLOCKED')
        self.assertEqual(avoid.command(True, (0, -.31, -1.0), 1, safe, 13.2)[:3],
                         (0.0, 0.0, 0.0))
        self.assertEqual(avoid.command(False, (0, -.31, -1.0), 1, safe, 13.4)[:3],
                         (0.0, 0.0, 0.0))

    def test_selects_one_alternate_side_if_preferred_is_blocked(self):
        avoid = TargetAvoidance()
        safe = lambda vx, vy, wz: vy >= 0
        first = avoid.command(False, (0, 0, 0), -1, safe, 10)
        second = avoid.command(False, (0, .01, 0), -1, safe, 10.2)
        self.assertEqual(first[1], .08)
        self.assertEqual(second[1], .08)


if __name__ == '__main__':
    unittest.main()
