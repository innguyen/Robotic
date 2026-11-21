"""Offline TF freshness, cropped LiDAR and map-frame adapter integration."""
import json
import math
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

import numpy as np

import navigation_ros1 as adapter
from map_frames_ros1 import MapFrame
from mapping_scan_ros1 import crop_scan
from navigation_core_ros1 import Costmap


class FrameTests(unittest.TestCase):
    def frame(self):
        frame = MapFrame.__new__(MapFrame)
        frame.frame, frame.odom_frame, frame.timeout = 'map', 'odom', .8
        frame.transform = frame.previous = frame.received = None
        frame.error, frame.age = 'WAIT_LOCALIZATION', None
        frame.lock, frame.sample = threading.Lock(), None
        now = 100.
        frame.ros = types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: now)))
        class Time:
            def __init__(self, value=0):
                self.value = value
            def to_sec(self):
                return self.value
            @staticmethod
            def now():
                return Time(now)
        frame.ros.Time = Time
        self.feed(frame)
        return frame

    def feed(self, frame, stamp=99.9, parent='map', child='odom', w=math.sqrt(.5)):
        ns = types.SimpleNamespace
        frame.callback(ns(transforms=[ns(header=ns(frame_id=parent, stamp=frame.ros.Time(stamp)),
            child_frame_id=child, transform=ns(translation=ns(x=10., y=20., z=0.),
                rotation=ns(x=0., y=0., z=math.sqrt(.5), w=w)))]))

    def test_rotates_odom_pose_and_target_into_map(self):
        frame = self.frame()
        self.assertEqual(frame.refresh(), '')
        np.testing.assert_allclose(frame.pose((1., 0., .2)), (10., 21., math.pi/2+.2))
        np.testing.assert_allclose(frame.point((2., 0.)), (10., 22.))
        self.assertAlmostEqual(frame.age, .1)

    def test_stale_or_missing_transform_never_becomes_identity(self):
        frame = self.frame()
        frame.sample = (98., (10., 20., 0.), time.monotonic(), '')
        self.assertEqual(frame.refresh(), 'STALE_LOCALIZATION')
        self.assertIsNone(frame.transform)
        frame.sample = None
        self.assertEqual(frame.refresh(), 'WAIT_LOCALIZATION')

    def test_wall_watchdog_stops_when_ros_clock_is_frozen(self):
        frame = self.frame()
        frame.sample = (99.9, (10., 20., 0.), time.monotonic()-1., '')
        self.assertEqual(frame.refresh(), 'STALE_LOCALIZATION')

    def test_unrelated_or_repeated_tf_does_not_refresh_watchdog(self):
        frame = self.frame()
        previous = frame.sample
        self.feed(frame, parent='odom', child='base_footprint', stamp=100.)
        self.assertEqual(frame.sample, previous)
        self.feed(frame)
        self.assertEqual(frame.sample, previous)
        self.feed(frame, stamp=100., w=0.)
        self.assertEqual(frame.refresh(), 'INVALID_LOCALIZATION')

    def test_crop_excludes_blind_sectors_and_preserves_time(self):
        ns = types.SimpleNamespace
        msg = ns(header=ns(stamp=100.), angle_min=-math.pi, angle_increment=math.pi/180,
                 time_increment=.001, ranges=[1.]*360, intensities=list(range(360)), range_min=.1, range_max=6.)
        msg.ranges[100] = 0.
        with patch.dict(sys.modules, {'rospy': ns(Duration=lambda x: x)}):
            cropped = crop_scan(msg)
        self.assertGreater(math.degrees(cropped.angle_min), -90.)
        self.assertLess(math.degrees(cropped.angle_max), 120.)
        self.assertEqual(len(cropped.ranges), 209)
        self.assertAlmostEqual(cropped.header.stamp, 100.091)
        self.assertTrue(math.isnan(cropped.ranges[9]))
        self.assertEqual(msg.header.stamp, 100.)


class AdapterMapTests(unittest.TestCase):
    def run_adapter(self, require_amcl=False, map_goal=None, late_search=False):
        ns = types.SimpleNamespace
        callbacks, services, reports, grids = {}, {}, [], []
        clock, cycle = [100.], [-1]
        class Stamp:
            def __init__(self, value=0): self.value = value
            def to_sec(self): return self.value
            @staticmethod
            def now(): return Stamp(clock[0])
        def twist(): return ns(linear=ns(x=0., y=0., z=0.), angular=ns(x=0., y=0., z=0.))
        class Publisher:
            def __init__(self, topic, *a, **kw): self.topic = topic
            def publish(self, message):
                if self.topic == '~state': reports.append(json.loads(message.data))
                if self.topic == '~costmap': grids.append(message.header.frame_id)
        class FakeFrame:
            def __init__(self, *args):
                self.error, self.age, self.received, self.timeout = '', 0., time.monotonic(), .8
            def refresh(self):
                self.received = time.monotonic()
                self.error = 'WAIT_LOCALIZATION' if 7 <= cycle[0] <= 9 else ''
                return self.error
            def correction(self): return (0., 0.)
            def pose(self, pose): return (pose[0]+10., pose[1]+20., pose[2])
            def point(self, point): return (point[0]+10., point[1]+20.)
            def close(self): pass
        ros = types.ModuleType('rospy')
        params = {'~global_frame': 'map', '/jetauto_collision_monitor/scan_frame': 'laser',
                  '~require_amcl': require_amcl}
        ros.init_node = lambda *a, **kw: None
        ros.get_param = lambda key, default=None: params.get(key, default)
        ros.resolve_name = lambda name: name
        ros.Publisher = Publisher
        def subscribe(topic, kind, callback, **kw):
            callbacks[topic] = callback
            return ns(unregister=lambda: None)
        ros.Subscriber = subscribe
        def service(topic, kind, callback):
            services[topic] = callback
            return ns(shutdown=lambda: None)
        ros.Service, ros.Time = service, Stamp
        ros.on_shutdown = lambda *a: None
        ros.loginfo = lambda *a: None
        def running():
            cycle[0] += 1
            index = cycle[0]
            if index >= 14: return True
            clock[0] = 100.+index*.1
            stamp = Stamp(clock[0])
            callbacks['/odom'](ns(header=ns(frame_id='odom', stamp=stamp), child_frame_id='base_footprint',
                pose=ns(pose=ns(position=ns(x=0., y=0.), orientation=ns(x=0., y=0., z=0., w=1.))),
                twist=ns(twist=twist())))
            callbacks['/scan'](ns(header=ns(frame_id='laser', stamp=stamp), ranges=[float('inf')]*360,
                angle_min=-math.pi, angle_increment=math.pi/180, range_min=.1, range_max=6.))
            callbacks['/jetauto_collision_monitor/state'](ns(data=json.dumps(dict(state='PASS', velocity=[0., 0., 0.]))))
            if require_amcl:
                covariance = np.eye(6)*(.9 if 5 <= index <= 6 else .01)
                callbacks['/amcl_pose'](ns(header=ns(frame_id='map'), pose=ns(covariance=covariance.ravel().tolist())))
            if index == 0:
                callbacks['/map'](ns(header=ns(frame_id='map'), data=[0]*10000,
                    info=ns(height=100, width=100, resolution=.1, origin=ns(position=ns(x=5., y=15.),
                        orientation=ns(x=0., y=0., z=0., w=1.)))))
            callbacks['/vision/target'](ns(data=json.dumps(dict(valid=True, label='person', frame_id='odom',
                stamp=clock[0], x=2., y=0., visible=True))))
            if index == 2: services['~enable'](ns(data=True))
            if index == 11 and map_goal is not None:
                callbacks['~map_goal'](ns(data=json.dumps(dict(mode='point', frame_id='map',
                    stamp=clock[0], x=map_goal[0], y=map_goal[1]))))
            return False
        ros.is_shutdown = running
        modules = {'rospy': ros}
        messages = {'geometry_msgs': {'Twist': twist, 'PoseWithCovarianceStamped': object,
                    'PoseStamped': lambda: ns(header=ns(), pose=ns(position=ns(), orientation=ns()))},
                    'nav_msgs': {'Odometry': object, 'Path': lambda: ns(header=ns(), poses=[]),
                    'OccupancyGrid': lambda: ns(header=ns(), info=ns(origin=ns(position=ns(), orientation=ns())))},
                    'sensor_msgs': {'LaserScan': object}, 'std_msgs': {'String': lambda **kw: ns(**kw)}}
        for package, items in messages.items():
            parent, msg = types.ModuleType(package), types.ModuleType(package+'.msg')
            for key, value in items.items(): setattr(msg, key, value)
            parent.msg = msg
            modules[package], modules[package+'.msg'] = parent, msg
        parent, srv = types.ModuleType('std_srvs'), types.ModuleType('std_srvs.srv')
        srv.SetBool, srv.SetBoolResponse = object, lambda **kw: ns(**kw)
        parent.srv = srv
        modules['std_srvs'], modules['std_srvs.srv'] = parent, srv
        original_update = Costmap.update
        def update(costmap, *args, **kwargs):
            result = original_update(costmap, *args, **kwargs)
            if late_search and cycle[0] == 3:
                callbacks['~map_goal'](ns(data=json.dumps(dict(mode='search', frame_id='map',
                    stamp=clock[0]))))
            return result
        with patch.dict(sys.modules, modules), patch('map_frames_ros1.MapFrame', FakeFrame), \
                patch.object(Costmap, 'update', update):
            adapter.main()
        return reports, grids

    def test_adapter_uses_map_coordinates_and_stops_on_tf_loss(self):
        reports, grids = self.run_adapter()
        fresh = [r for r in reports if r['robot_pose'] == [10., 20., 0.]]
        self.assertTrue(fresh)
        self.assertEqual(fresh[-1]['target'], [12., 20.])
        self.assertTrue(all(r['frame_id'] == 'map' for r in reports))
        lost = [r for r in reports if r['state'] == 'WAIT_LOCALIZATION']
        self.assertTrue(lost)
        self.assertTrue(all(r['velocity'] == [0., 0., 0.] for r in lost))
        self.assertTrue(all(frame == 'map' for frame in grids))
        self.assertTrue(reports[-1]['map_memory']['persistent'])

    def test_search_requested_mid_cycle_waits_for_mission_without_crashing(self):
        reports, _ = self.run_adapter(late_search=True)
        waiting = [r for r in reports if r['state'] == 'WAIT_SEARCH']
        self.assertTrue(waiting)
        self.assertTrue(all(r['velocity'] == [0., 0., 0.] for r in waiting))
        self.assertEqual(reports[-1]['goal_mode'], 'search')
        self.assertIsNotNone(reports[-1]['search'])

    def test_uncertain_amcl_blocks_motion_until_covariance_recovers(self):
        reports, grids = self.run_adapter(require_amcl=True)
        uncertain = [r for r in reports if r['state'] == 'LOCALIZATION_UNCERTAIN']
        self.assertTrue(uncertain)
        self.assertTrue(all(r['velocity'] == [0., 0., 0.] for r in uncertain))
        self.assertNotEqual(reports[-1]['state'], 'LOCALIZATION_UNCERTAIN')

    def test_map_goal_beyond_rolling_window_keeps_map_coordinates(self):
        reports, _ = self.run_adapter(map_goal=(14.5, 20.))
        self.assertEqual(reports[-1]['goal_mode'], 'point')
        self.assertEqual(reports[-1]['target'], [14.5, 20.])
        self.assertEqual(reports[-1]['goal_error'], '')

    def test_goal_outside_global_map_is_rejected(self):
        reports, _ = self.run_adapter(map_goal=(40., 20.))
        self.assertEqual(reports[-1]['goal_mode'], 'vision')
        self.assertEqual(reports[-1]['goal_error'], 'GOAL_BLOCKED_OR_UNKNOWN')


if __name__ == '__main__':
    unittest.main()
