"""Offline tests for the vision tracker and web stream; no ROS or camera required."""
import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


def load_vision():
    modules = {'rospy': types.ModuleType('rospy')}
    for package, names in [('nav_msgs', ['Odometry', 'OccupancyGrid', 'Path']),
                           ('sensor_msgs', ['Image', 'CameraInfo']), ('std_msgs', ['String'])]:
        pkg, msg = types.ModuleType(package), types.ModuleType(package+'.msg')
        for name in names:
            setattr(msg, name, lambda **kwargs: types.SimpleNamespace(**kwargs))
        pkg.msg = msg
        modules[package], modules[package+'.msg'] = pkg, msg
    with patch.dict(sys.modules, modules):
        path = Path(__file__).with_name('jetauto_find_object.py')
        spec = importlib.util.spec_from_file_location('vision_tracking_test', path)
        vision = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(vision)
    return vision


vision = load_vision()


class TrackerTests(unittest.TestCase):
    def confirmed_bottle(self, pose=(0., 0., 0.)):
        tracker = vision.TargetTracker(vision.TrackingConfig())
        for i in range(3):
            self.assertTrue(tracker.observe('bottle', 0., 1.4, 10.+.2*i, pose))
        self.assertTrue(tracker.confirmed)
        return tracker

    def test_single_outlier_does_not_drop_confirmed_target(self):
        tracker = self.confirmed_bottle()
        # Motion blur / a second bottle: 40 degrees off the anchored position.
        self.assertFalse(tracker.observe('bottle', math.radians(40), 1.4, 10.8, (0., 0., 0.)))
        self.assertTrue(tracker.confirmed)
        self.assertAlmostEqual(tracker.anchor[0], 1.4)
        self.assertTrue(tracker.estimate(10.9, (0., 0., 0.)).valid)

    def test_persistent_outliers_switch_to_new_object(self):
        tracker = self.confirmed_bottle()
        accepted = [tracker.observe('bottle', math.radians(40), 1.4, 10.8+.2*i, (0., 0., 0.)) for i in range(3)]
        self.assertEqual(accepted, [False, False, True])
        self.assertFalse(tracker.confirmed)
        self.assertEqual(tracker.hits, 1)

    def test_rotation_is_compensated_by_odometry(self):
        tracker = self.confirmed_bottle()
        # Robot turned +30 deg in place: the bottle now appears at -30 deg.
        yaw = math.radians(30)
        self.assertTrue(tracker.observe('bottle', -yaw, 1.4, 11.0, (0., 0., yaw)))
        self.assertTrue(tracker.confirmed)

    def test_short_loss_keeps_confirmation_when_anchored(self):
        tracker = self.confirmed_bottle()
        self.assertTrue(tracker.estimate(12.0, (0., 0., 0.)).valid)   # 1.6 s in memory
        self.assertTrue(tracker.observe('bottle', 0., 1.4, 12.0, (0., 0., 0.)))
        self.assertTrue(tracker.confirmed)

    def test_small_object_memory_expires(self):
        tracker = self.confirmed_bottle()
        self.assertFalse(tracker.estimate(10.4+vision.TrackingConfig.target_memory_small_s+.1, (0., 0., 0.)).valid)

    def test_navigation_timeout_covers_vision_memory(self):
        import yaml
        timeout = yaml.safe_load(Path(__file__).with_name('navigation_ros1.yaml').read_text())['target_timeout']
        self.assertGreaterEqual(timeout, vision.TrackingConfig.target_memory_small_s)


class DetectorTests(unittest.TestCase):
    def test_letterbox_keeps_aspect_and_maps_boxes_back(self):
        seen = {}
        def run(_, feeds):
            blob = feeds['images']
            seen['blob'] = blob
            out = np.zeros((1, 200, 85), dtype=np.float32)   # real model: 25200 rows
            # Letterbox coords: 640x480 frame sits at rows 80..560; box centre (320, 400).
            out[0, 0, :6] = (320, 400, 64, 128, .9, .9)
            out[0, 1, :6] = (100, 300, 40, 40, .5, .5)   # 0.25 combined: below the floor
            return [out]
        seeker = vision.ObjectSeeker.__new__(vision.ObjectSeeker)
        seeker.use_ort, seeker.input_name, seeker.classes = True, 'images', ['person']
        seeker.session = types.SimpleNamespace(run=run)
        frame = np.full((480, 640, 3), 200, dtype=np.uint8)
        results = seeker.run_yolo(frame)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]['box'], [288, 256, 64, 128])   # centre (320, 320) in the frame
        blob = seen['blob'][0]
        self.assertAlmostEqual(float(blob[0, 40, 320]), 114/255., places=4)   # grey pad, not stretched image
        self.assertAlmostEqual(float(blob[0, 320, 320]), 200/255., places=4)


class OverlayTests(unittest.TestCase):
    def test_draw_overlay_on_newer_frame(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        overlay = dict(detections=[{'box': (10, 20, 50, 80), 'conf': .6, 'class_name': 'bottle'}],
                       label='bottle', text='CONFIRMED', ai_fps=4.8, hits=3, required_hits=3,
                       min_confidence=.45, target_info={'distance': 1.4, 'source': 'DO SAU'}, stream_fps=12.)
        vision.ObjectSeeker.draw_overlay(frame, overlay)
        self.assertGreater(int(frame.sum()), 0)


if __name__ == '__main__':
    unittest.main()
