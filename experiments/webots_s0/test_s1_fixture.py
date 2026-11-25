"""Run: python -m unittest discover -s experiments/webots_s0 -p test_s1_fixture.py -v"""
import unittest

from s1_fixture import BASE, make_occlusion, validate, validate_files


class FixtureTests(unittest.TestCase):
    def test_committed_fixtures_and_sensor_contract(self):
        validate_files()

    def test_saved_end_pose_is_rejected(self):
        world = BASE.read_text(encoding="utf-8")
        for old, new in (
            ("translation -2 -1 0.1", "translation -0.088722 -1.008309 0.099405"),
            ("rotation 0 0 1 -1.5707963267948966", "rotation 0 0 1 0.007632"),
            ("axis 0 1 0", "position 15.97\n        axis 0 1 0"),
            ('controller "s0_smoke"', 'controller "s0_smoke"\n  linearVelocity 0 0 0'),
        ):
            with self.subTest(mutation=new), self.assertRaises(ValueError):
                validate(world.replace(old, new, 1))

    def test_fixture_and_sensor_drift_is_rejected(self):
        world = BASE.read_text(encoding="utf-8")
        for old, new in (("size 8 6 0.1", "size 9 6 0.1"),
                         ("width 160", "width 320"),
                         ("horizontalResolution 180", "horizontalResolution 90"),
                         ("translation 0.12 0 0.15", "translation 0.096 0 0.15"),
                         ("occlusion 1", "occlusion 0")):
            with self.subTest(mutation=new), self.assertRaises(ValueError):
                validate(world.replace(old, new, 1))

    def test_occluder_cannot_silently_lose_collision_geometry(self):
        scenario = make_occlusion(BASE.read_text(encoding="utf-8"))
        with self.assertRaises(ValueError):
            validate(scenario.replace("boundingObject USE OCCLUDER_BOX", "boundingObject NULL"), True)


if __name__ == "__main__":
    unittest.main()
