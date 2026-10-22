#!/usr/bin/env python3
"""ROS 1 velocity filter inspired by Nav2 Collision Monitor, not a Nav2 port.

Python 3.6+, NumPy, rospy and standard ROS 1 messages. Core has no ROS dependency.
Velocity units: linear m/s, angular rad/s. Scan extrinsics are measured, fixed.
"""
import json
import math
import threading
import time

import numpy as np


class Config(object):
    DEFAULTS = {
        'body_length': 0.324, 'body_width': 0.260,
        'stop_margin': 0.06, 'slow_margin': 0.25, 'slowdown_ratio': 0.35,
        'reaction_time': 0.25, 'brake_decel': 0.35,
        'max_forward': 0.18, 'max_backward': 0.06,
        'max_lateral': 0.12, 'max_angular': math.radians(30.0),
        'command_timeout': 0.4, 'source_timeout': 0.5, 'rate': 20.0,
        'scan_x': 0.096, 'scan_y': 0.0, 'scan_yaw': 0.0,
        'min_valid_fraction': 0.05,
        # Known obscured sectors, expressed around the scan sensor in degrees.
        'blind_sectors_deg': [[120.0, 180.0], [-180.0, -90.0]],
        'block_blind_motion': True,
    }

    def __init__(self, **kwargs):
        unknown = set(kwargs) - set(self.DEFAULTS)
        if unknown:
            raise ValueError('Unknown parameters: ' + str(sorted(unknown)))
        for key, default in self.DEFAULTS.items():
            value = kwargs.get(key, default)
            setattr(self, key, value)
        for key in self.DEFAULTS:
            if key not in ('blind_sectors_deg', 'block_blind_motion'):
                value = float(getattr(self, key))
                if not math.isfinite(value):
                    raise ValueError(key + ' must be finite')
                setattr(self, key, value)
        for key in ('body_length', 'body_width', 'brake_decel', 'rate',
                    'command_timeout', 'source_timeout', 'max_forward',
                    'max_backward', 'max_lateral', 'max_angular'):
            if getattr(self, key) <= 0:
                raise ValueError(key + ' must be positive')
        if not 0 <= self.stop_margin < self.slow_margin:
            raise ValueError('Require 0 <= stop_margin < slow_margin')
        if self.reaction_time < 0 or not 0 < self.slowdown_ratio <= 1:
            raise ValueError('Invalid reaction_time or slowdown_ratio')
        if not 0 < self.min_valid_fraction <= 1:
            raise ValueError('Invalid min_valid_fraction')
        if not isinstance(self.block_blind_motion, bool):
            raise ValueError('block_blind_motion must be a boolean')
        for sector in self.blind_sectors_deg:
            if len(sector) != 2 or not all(math.isfinite(float(x)) for x in sector):
                raise ValueError('Invalid blind sector')


def in_sector(angle_deg, lo, hi):
    angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
    return lo <= angle_deg <= hi if lo <= hi else angle_deg >= lo or angle_deg <= hi


def translation_sweep_clear(points, a, b, yaw, hx, hy):
    """Exact fixed-heading rectangle sweep against points (no sampling padding)."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    c, s = math.cos(yaw), math.sin(yaw)
    # Most stored map points are far from this short edge; reject them before
    # computing slab intervals during grid search.
    extent_x, extent_y = abs(c)*hx+abs(s)*hy, abs(s)*hx+abs(c)*hy
    lower = np.minimum(a, b)-(extent_x, extent_y)
    upper = np.maximum(a, b)+(extent_x, extent_y)
    points = points[np.all((points >= lower) & (points <= upper), axis=1)]
    if not len(points):
        return True
    delta = points-np.asarray(a)
    start = np.column_stack((c*delta[:, 0]+s*delta[:, 1], -s*delta[:, 0]+c*delta[:, 1]))
    move = np.asarray(b)-a
    direction = (c*move[0]+s*move[1], -s*move[0]+c*move[1])
    enter, leave = np.zeros(len(start)), np.ones(len(start))
    for axis, half in enumerate((hx, hy)):
        if abs(direction[axis]) < 1e-12:
            enter[np.abs(start[:, axis]) > half] = float('inf')
        else:
            t1 = (start[:, axis]-half)/direction[axis]
            t2 = (start[:, axis]+half)/direction[axis]
            enter = np.maximum(enter, np.minimum(t1, t2))
            leave = np.minimum(leave, np.maximum(t1, t2))
    return not np.any(enter <= leave)


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, cfg):
    """Convert a fixed planar scan to base-frame points; reject unusable scans.

    +Inf denotes a valid no-return ray. NaN/zero/negative/-Inf are unknown.
    Configured occluded sectors contribute neither obstacle points nor coverage.
    """
    values = np.asarray(ranges, dtype=float)
    metadata = (angle_min, angle_increment, range_min, range_max)
    if (values.ndim != 1 or len(values) < 3
            or not all(math.isfinite(float(v)) for v in metadata)
            or angle_increment == 0 or range_min < 0 or range_max <= range_min):
        raise ValueError('Invalid scan metadata')
    angles = angle_min + np.arange(len(values)) * angle_increment
    blind = np.zeros(len(values), dtype=bool)
    deg = (np.degrees(angles) + 180.0) % 360.0 - 180.0
    for lo, hi in cfg.blind_sectors_deg:
        blind |= ((deg >= lo) & (deg <= hi)) if lo <= hi else ((deg >= lo) | (deg <= hi))
    usable = ~blind
    finite = np.isfinite(values) & (values >= range_min) & (values <= range_max) & (values > 0)
    observed = finite | np.isposinf(values)
    if not usable.any() or float((observed & usable).sum()) / usable.sum() < cfg.min_valid_fraction:
        raise ValueError('Insufficient usable laser rays')
    hit = finite & usable
    bearing = angles[hit] + cfg.scan_yaw
    return np.column_stack((cfg.scan_x + values[hit] * np.cos(bearing),
                            cfg.scan_y + values[hit] * np.sin(bearing)))


class Monitor(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.points = None
        self.scan_time = self.command_time = None
        self.command = (0.0, 0.0, 0.0)
        self.scan_error = 'WAIT_SCAN'
        self.command_error = 'WAIT_COMMAND'

    def invalidate_scan(self, reason):
        self.points = None
        self.scan_error = reason

    def set_scan(self, points, now):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if not np.isfinite(points).all():
            self.invalidate_scan('INVALID_POINTS')
            return
        self.points, self.scan_time = points, now
        self.scan_error = ''

    def set_command(self, vx, vy, wz, now):
        command = (float(vx), float(vy), float(wz))
        self.command_time = now
        if not all(math.isfinite(v) for v in command):
            self.command = (0.0, 0.0, 0.0)
            self.command_error = 'INVALID_COMMAND'
        else:
            self.command = command
            self.command_error = ''

    def _inside(self, points, margin):
        return ((np.abs(points[:, 0]) <= self.cfg.body_length / 2 + margin)
                & (np.abs(points[:, 1]) <= self.cfg.body_width / 2 + margin))

    def _sweep_safe(self, command):
        """Sample constant body-frame Twist over reaction + conservative braking time.

        Continuing at full speed for the entire braking time overestimates travel.
        Inflate each sample by half a sampling interval of maximum edge travel.
        """
        cfg = self.cfg
        vx, vy, wz = command
        edge_speed = math.hypot(vx, vy) + abs(wz) * math.hypot(cfg.body_length, cfg.body_width) / 2
        if edge_speed < 1e-9:
            return True
        horizon = cfg.reaction_time + 1.0 / cfg.rate + edge_speed / cfg.brake_decel
        if abs(wz) < 1e-9:
            return translation_sweep_clear(self.points, (0., 0.), (vx*horizon, vy*horizon), 0.,
                cfg.body_length/2+cfg.stop_margin, cfg.body_width/2+cfg.stop_margin)
        steps = max(2, int(math.ceil(edge_speed * horizon / 0.01)))
        padding = edge_speed * horizon / steps / 2
        for t in np.linspace(0, horizon, steps + 1):
            theta = wz * t
            if abs(wz) < 1e-9:
                x, y = vx * t, vy * t
            else:
                x = (vx * math.sin(theta) + vy * (math.cos(theta) - 1)) / wz
                y = (vx * (1 - math.cos(theta)) + vy * math.sin(theta)) / wz
            delta = self.points - (x, y)
            c, s = math.cos(theta), math.sin(theta)
            local = np.column_stack((c * delta[:, 0] + s * delta[:, 1],
                                     -s * delta[:, 0] + c * delta[:, 1]))
            if self._inside(local, cfg.stop_margin + padding).any():
                return False
        return True

    def evaluate(self, now):
        cfg = self.cfg
        zero = (0.0, 0.0, 0.0)
        if self.points is None:
            return zero, self.scan_error
        if self.scan_time is None or not 0 <= now - self.scan_time <= cfg.source_timeout:
            return zero, 'STALE_SCAN'
        if self.command_error:
            return zero, self.command_error
        if self.command_time is None or not 0 <= now - self.command_time <= cfg.command_timeout:
            return zero, 'STALE_COMMAND'
        raw = self.command
        if max(abs(v) for v in raw) < 1e-9:
            return zero, 'IDLE'
        vx, vy, wz = raw
        # Preserve the requested path by scaling all components together.
        scale = min(1.0, (cfg.max_forward if vx >= 0 else cfg.max_backward) / max(abs(vx), 1e-12),
                    cfg.max_lateral / max(abs(vy), 1e-12), cfg.max_angular / max(abs(wz), 1e-12))
        state = 'LIMIT' if scale < 1 else 'PASS'
        if cfg.block_blind_motion and cfg.blind_sectors_deg:
            angle = math.degrees(math.atan2(vy, vx) - cfg.scan_yaw)
            if abs(wz) > 1e-6 or (math.hypot(vx, vy) > 1e-6 and any(
                    in_sector(angle, lo, hi) for lo, hi in cfg.blind_sectors_deg)):
                return zero, 'UNOBSERVED_DIRECTION'
        if self._inside(self.points, cfg.stop_margin).any():
            return zero, 'STOP_ZONE'
        if self._inside(self.points, cfg.slow_margin).any():
            scale = min(scale, cfg.slowdown_ratio)
            state = 'SLOWDOWN'
        candidate = tuple(v * scale for v in raw)
        if not self._sweep_safe(candidate):
            # Test discrete lower speeds without assuming monotonic collision geometry.
            for fraction in (0.75, 0.5, 0.25):
                reduced = tuple(v * fraction for v in candidate)
                if self._sweep_safe(reduced):
                    return reduced, 'APPROACH'
            return zero, 'PREDICTED_COLLISION'
        return candidate, state


def main():
    import rospy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    rospy.init_node('jetauto_collision_monitor')
    cfg = Config(**{key: rospy.get_param('~' + key, value)
                    for key, value in Config.DEFAULTS.items()})
    monitor = Monitor(cfg)
    lock = threading.RLock()
    # Explicitly configure the sensor frame; never silently assume a TF transform.
    scan_frame = rospy.get_param('~scan_frame', '').lstrip('/')
    input_topic = rospy.resolve_name(rospy.get_param('~input_topic', '/collision_monitor/cmd_vel_raw'))
    output_topic = rospy.resolve_name(rospy.get_param('~output_topic', '/collision_monitor/cmd_vel_checked'))
    scan_topic = rospy.get_param('~scan_topic', '/scan')
    if input_topic == output_topic:
        raise ValueError('Input and output topics must differ')
    if not scan_frame:
        rospy.logwarn('Set ~scan_frame to the measured LiDAR frame. Output stays zero until configured.')
    publisher = rospy.Publisher(output_topic, Twist, queue_size=1)
    status_pub = rospy.Publisher('~state', String, queue_size=1, latch=True)
    stopping = threading.Event()

    def command_callback(msg):
        with lock:
            if any(abs(v) > 1e-9 or not math.isfinite(v)
                   for v in (msg.linear.z, msg.angular.x, msg.angular.y)):
                monitor.set_command(float('nan'), 0, 0, time.monotonic())
            else:
                monitor.set_command(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())

    last_stamp = [None]

    def scan_callback(msg):
        with lock:
            try:
                if not scan_frame or msg.header.frame_id.lstrip('/') != scan_frame:
                    raise ValueError('SCAN_FRAME_MISMATCH')
                stamp = msg.header.stamp.to_sec()
                age = rospy.Time.now().to_sec() - stamp
                if stamp <= 0 or age < -0.05 or age > cfg.source_timeout:
                    raise ValueError('STALE_SCAN_STAMP')
                if last_stamp[0] is not None and stamp <= last_stamp[0]:
                    raise ValueError('NONINCREASING_SCAN_STAMP')
                points = scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                     msg.range_min, msg.range_max, cfg)
                # Receipt time alone must not give delayed data a new full lifetime.
                monitor.set_scan(points, time.monotonic() - max(0.0, age))
                last_stamp[0] = stamp
            except (ValueError, TypeError, OverflowError) as exc:
                monitor.invalidate_scan(str(exc))

    def publish_zero():
        stopping.set()
        with lock:
            for _ in range(3):
                publisher.publish(Twist())
                time.sleep(0.02)

    subscribers = [
        rospy.Subscriber(input_topic, Twist, command_callback, queue_size=1),
        rospy.Subscriber(scan_topic, LaserScan, scan_callback, queue_size=1),
    ]
    rospy.on_shutdown(publish_zero)
    rospy.loginfo('Collision filter: %s -> %s (rad/s angular units)', input_topic, output_topic)
    previous_state = None
    try:
        # Wall clock watchdog continues even if ROS /clock stalls.
        while not rospy.is_shutdown() and not stopping.is_set():
            start = time.monotonic()
            with lock:
                velocity, state = monitor.evaluate(start)
                msg = Twist()
                msg.linear.x, msg.linear.y, msg.angular.z = velocity
                publisher.publish(msg)
            status_pub.publish(String(data=json.dumps({'state': state, 'velocity': velocity})))
            if state != previous_state:
                rospy.loginfo('Collision filter state: %s', state)
                previous_state = state
            stopping.wait(max(0, 1.0 / cfg.rate - (time.monotonic() - start)))
    finally:
        publish_zero()
        for subscriber in subscribers:
            subscriber.unregister()


if __name__ == '__main__':
    main()
