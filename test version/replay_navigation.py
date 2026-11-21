#!/usr/bin/env python3
"""Evaluate the current controller against recorded poses/scans without ROS publishers.

Requires rosbag's Python module (run on a ROS computer). This is an open-loop
comparison on the recorded trajectory, not a replacement for closed-loop testing.
"""
import argparse
import json
import math
from collections import Counter, deque
from pathlib import Path

import numpy as np

from collision_monitor_ros1 import Monitor, scan_points
from map_core_ros1 import MappedNavigator, OccupancyMap
from navigation_core_ros1 import nearest_pose, to_world
from navigation_ros1 import STANDOFF
from simulate_navigation import robot_config


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def read_map(msg):
    info, q = msg.info, msg.info.origin.orientation
    return OccupancyMap(np.asarray(msg.data).reshape(info.height, info.width),
        info.resolution, (info.origin.position.x, info.origin.position.y, yaw_of(q)))


def replay(filename, bootstrap_map=None):
    import rosbag
    cfg, cc = robot_config()
    nav, monitor = MappedNavigator(cfg, cc), Monitor(cc)
    bootstrap = None
    if bootstrap_map is not None:
        with rosbag.Bag(str(bootstrap_map), 'r') as bag:
            for _, msg, _ in bag.read_messages(topics=['/map']):
                bootstrap = read_map(msg)
        if bootstrap is None:
            raise ValueError('Bootstrap bag contains no /map')
    history, scans = deque(maxlen=200), deque(maxlen=20)
    counts, monitor_counts = Counter(), Counter()
    task, transform, skipped, commands, last_mode = {}, None, 0, 0, None
    previous = None
    with rosbag.Bag(str(filename), 'r') as bag:
        for topic, msg, stamp in bag.read_messages():
            now = stamp.to_sec()
            if topic == '/jetauto_navigation/state':
                task = json.loads(msg.data)
            elif topic == '/tf':
                for tf in msg.transforms:
                    if tf.header.frame_id.lstrip('/') == 'map' and tf.child_frame_id.lstrip('/') == 'odom':
                        p, q = tf.transform.translation, tf.transform.rotation
                        transform = (p.x, p.y, yaw_of(q))
            elif topic == '/map':
                nav.set_global_map(read_map(msg))
            elif topic == '/odom':
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                history.append((msg.header.stamp.to_sec(), (p.x, p.y, yaw_of(q))))
            elif topic == '/scan':
                scans.append((msg, now))
            if not scans or not history:
                continue
            scan, received = scans[0]
            pose = nearest_pose(history, scan.header.stamp.to_sec(), cfg.pose_sync_tolerance)
            if pose is None:
                if now-received > cc.source_timeout:
                    scans.popleft()
                    skipped += 1
                continue
            scans.popleft()
            mapped = task.get('frame_id', 'odom') == 'map'
            if mapped:
                if nav.costmap.global_map is None and bootstrap is not None:
                    nav.set_global_map(bootstrap)
                if transform is None or nav.costmap.global_map is None:
                    skipped += 1
                    continue
                xy = to_world([pose[:2]], transform)[0]
                pose = (*xy, pose[2]+transform[2])
            elif nav.costmap.global_map is not None:
                nav.costmap.global_map = None  # local-mode trials remain in odom
                nav.reset()
            mode = task.get('goal_mode', 'vision')
            if mode != last_mode:
                nav.reset()
                last_mode = mode
            nav.costmap.search_mode = mode == 'search'
            try:
                nav.costmap.update(scan.ranges, scan.angle_min, scan.angle_increment,
                    scan.range_min, scan.range_max, pose, received)
                monitor.set_scan(scan_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                             scan.range_min, scan.range_max, cc), received)
            except ValueError:
                skipped += 1
                continue
            target = task.get('target')
            # Recorded exploration goals are replayed; this evaluates the controller
            # on an identical mission, rather than changing where the bag was recorded.
            exact = mode == 'point'
            if mode == 'search' and (task.get('search') or {}).get('state') != 'APPROACH':
                target = (task.get('search') or {}).get('goal')
                exact = True
            if target is None or not task.get('enabled'):
                continue
            velocity, reason = nav.step(pose, target, STANDOFF.get(task.get('label'), .35),
                received, .1 if previous is None else max(.001, received-previous), exact)
            previous = received
            monitor.set_command(*velocity, now=received)
            _, monitor_reason = monitor.evaluate(received)
            counts[reason] += 1
            monitor_counts[monitor_reason] += 1
            commands += 1
    nav.close()
    return dict(file=str(filename), compared_commands=commands, skipped_scans=skipped,
                bootstrap_map=None if bootstrap_map is None else str(bootstrap_map),
                states=dict(counts), monitor_states=dict(monitor_counts),
                plan_count=nav.plan_count, path_switches=nav.path_switches,
                limitation='Open-loop recorded poses, not a simulated new trajectory; no motor output.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, default=Path('replay_report.json'))
    parser.add_argument('--bootstrap-map', type=Path, help='First bag of the same session with the latched /map')
    args = parser.parse_args()
    result = replay(args.bag, args.bootstrap_map)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
