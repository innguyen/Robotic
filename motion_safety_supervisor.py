#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Independent fail-safe gate between AI velocity requests and the JetAuto chassis.

The AI applications publish to ``/jetauto_ai/cmd_vel_desired``.  This process is
the only application publisher allowed to forward commands to
``/jetauto_controller/cmd_vel``.  A stale AI heartbeat, stale/invalid LiDAR,
stale/invalid Astra depth, or stale odometry latches a stop.  Recovery never
re-arms automatically: the operator must pause and explicitly resume again.

The core is ROS-independent so failure behaviour can be unit-tested on a PC.
"""
from __future__ import division, print_function

import json
import math
import os
import socket
import struct
import threading
import time


DESIRED_TOPIC = '/jetauto_ai/cmd_vel_desired'
ENABLE_TOPIC = '/jetauto_ai/safety_enable'
STATUS_TOPIC = '/jetauto_safety/status'
OUTPUT_TOPIC = '/jetauto_controller/cmd_vel'


def lidar_valid_fraction(ranges, range_min, range_max):
    """Finite returns only; zero/NaN/Inf remain unknown rather than free space."""
    if not ranges:
        return 0.0
    low = max(0.10, float(range_min))
    high = float(range_max)
    good = sum(1 for value in ranges
               if math.isfinite(value) and low <= float(value) <= high)
    return good / float(len(ranges))


def depth_valid_fraction(data, encoding, is_bigendian, width, height, samples=384):
    """Sample a ROS depth image without NumPy; all-zero/corrupt frames score zero."""
    pixels = int(width) * int(height)
    if not data or pixels <= 0:
        return 0.0
    encoding = (encoding or '').lower()
    if encoding in ('16uc1', 'mono16'):
        size, fmt = 2, ('>' if is_bigendian else '<') + 'H'
        valid_value = lambda value: 200 <= value <= 10000
    elif encoding == '32fc1':
        size, fmt = 4, ('>' if is_bigendian else '<') + 'f'
        valid_value = lambda value: math.isfinite(value) and 0.20 <= value <= 10.0
    else:
        return 0.0
    available = min(pixels, len(data) // size)
    if available <= 0:
        return 0.0
    count = min(int(samples), available)
    step = max(1, available // count)
    checked = good = 0
    for index in range(step // 2, available, step):
        if checked >= count:
            break
        try:
            value = struct.unpack_from(fmt, data, index * size)[0]
        except (struct.error, TypeError):
            break
        checked += 1
        if valid_value(value):
            good += 1
    return good / float(checked) if checked else 0.0


class SafetySupervisorCore(object):
    """State machine which only forwards a command while every proof is fresh."""

    def __init__(self, command_timeout=0.45, lidar_timeout=0.55,
                 depth_timeout=0.55, odom_timeout=0.55,
                 min_lidar_valid=0.10, min_depth_valid=0.01,
                 arm_grace=1.0):
        self.command_timeout = float(command_timeout)
        self.lidar_timeout = float(lidar_timeout)
        self.depth_timeout = float(depth_timeout)
        self.odom_timeout = float(odom_timeout)
        self.min_lidar_valid = float(min_lidar_valid)
        self.min_depth_valid = float(min_depth_valid)
        self.arm_grace = float(arm_grace)
        self.lock = threading.Lock()
        self.command = (0.0, 0.0, 0.0)
        self.command_time = None
        self.command_valid = True
        self.lidar_time = None
        self.lidar_quality = 0.0
        self.depth_time = None
        self.depth_quality = 0.0
        self.odom_time = None
        self.requested = False
        self.seen_disabled = False
        self.armed = False
        self.fault_latched = False
        self.enable_time = None
        self.last_fault = ''
        self.output = (0.0, 0.0, 0.0)

    def update_command(self, vx, vy, wz, now):
        values = (float(vx), float(vy), float(wz))
        valid = (all(math.isfinite(value) for value in values)
                 and abs(values[0]) <= 0.35 and abs(values[1]) <= 0.30
                 and abs(values[2]) <= math.radians(90.0))
        with self.lock:
            self.command = values if valid else (0.0, 0.0, 0.0)
            self.command_time = float(now)
            self.command_valid = valid

    def update_lidar(self, quality, now):
        with self.lock:
            self.lidar_quality = max(0.0, min(1.0, float(quality)))
            self.lidar_time = float(now)

    def update_depth(self, quality, now):
        with self.lock:
            self.depth_quality = max(0.0, min(1.0, float(quality)))
            self.depth_time = float(now)

    def update_odom(self, now):
        with self.lock:
            self.odom_time = float(now)

    def set_enabled(self, enabled, now):
        enabled = bool(enabled)
        with self.lock:
            if not enabled:
                self.seen_disabled = True
                self.requested = False
                self.armed = False
                self.fault_latched = False
                self.enable_time = None
                self.output = (0.0, 0.0, 0.0)
            elif self.seen_disabled and not self.requested:
                # Only a false -> true operator transition may clear a fault latch.
                self.requested = True
                self.armed = False
                self.fault_latched = False
                self.enable_time = float(now)

    @staticmethod
    def _age(now, stamp):
        return None if stamp is None else max(0.0, float(now) - stamp)

    def _health_fault(self, now):
        if not self.command_valid:
            return 'AI COMMAND INVALID'
        if self.command_time is None or now - self.command_time > self.command_timeout:
            return 'AI HEARTBEAT LOST'
        if self.lidar_time is None or now - self.lidar_time > self.lidar_timeout:
            return 'LIDAR STALE'
        if self.lidar_quality < self.min_lidar_valid:
            return 'LIDAR DATA INVALID'
        if self.depth_time is None or now - self.depth_time > self.depth_timeout:
            return 'DEPTH STALE'
        if self.depth_quality < self.min_depth_valid:
            return 'DEPTH DATA INVALID'
        if self.odom_time is None or now - self.odom_time > self.odom_timeout:
            return 'ODOMETRY STALE'
        return ''

    def evaluate(self, now):
        now = float(now)
        with self.lock:
            fault = self._health_fault(now)
            if not self.requested:
                self.armed = False
                reason = ('PAUSED - OPERATOR RESUME REQUIRED' if self.seen_disabled
                          else 'WAITING FOR PAUSE BEFORE ARM')
            elif self.fault_latched:
                self.armed = False
                reason = 'LATCHED: ' + self.last_fault
            elif fault:
                was_armed = self.armed
                self.armed = False
                in_grace = (not was_armed and self.enable_time is not None
                            and now - self.enable_time <= self.arm_grace)
                if in_grace:
                    reason = 'WAITING: ' + fault
                else:
                    self.fault_latched = True
                    self.last_fault = fault
                    reason = 'LATCHED: ' + fault
            else:
                self.armed = True
                reason = 'ARMED'
            self.output = self.command if self.armed else (0.0, 0.0, 0.0)
            return {
                'armed': self.armed,
                'requested': self.requested,
                'seen_disabled': self.seen_disabled,
                'fault_latched': self.fault_latched,
                'reason': reason,
                'last_fault': self.last_fault,
                'output': [round(value, 4) for value in self.output],
                'command_age': self._age(now, self.command_time),
                'lidar_age': self._age(now, self.lidar_time),
                'depth_age': self._age(now, self.depth_time),
                'odom_age': self._age(now, self.odom_time),
                'lidar_quality': round(self.lidar_quality, 4),
                'depth_quality': round(self.depth_quality, 4),
            }


def notify_systemd(message):
    """Minimal sd_notify implementation; harmless outside a systemd service."""
    address = os.environ.get('NOTIFY_SOCKET')
    if not address:
        return False
    if address.startswith('@'):
        address = '\0' + address[1:]
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        sock.connect(address)
        sock.sendall(message.encode('utf-8'))
        return True
    except Exception:
        return False
    finally:
        sock.close()


def run_ros_node():
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, LaserScan
    from std_msgs.msg import Bool, String

    def env_float(name, default):
        try:
            return float(os.environ.get(name, default))
        except (TypeError, ValueError):
            return float(default)

    rospy.init_node('jetauto_motion_safety', anonymous=False)
    core = SafetySupervisorCore(
        command_timeout=env_float('JETAUTO_SAFETY_COMMAND_TIMEOUT', 0.45),
        lidar_timeout=env_float('JETAUTO_SAFETY_LIDAR_TIMEOUT', 0.55),
        depth_timeout=env_float('JETAUTO_SAFETY_DEPTH_TIMEOUT', 0.55),
        odom_timeout=env_float('JETAUTO_SAFETY_ODOM_TIMEOUT', 0.55),
        min_lidar_valid=env_float('JETAUTO_SAFETY_LIDAR_VALID', 0.10),
        min_depth_valid=env_float('JETAUTO_SAFETY_DEPTH_VALID', 0.01))
    output_pub = rospy.Publisher(OUTPUT_TOPIC, Twist, queue_size=1)
    status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=True)

    rospy.Subscriber(DESIRED_TOPIC, Twist,
                     lambda msg: core.update_command(msg.linear.x, msg.linear.y,
                                                     msg.angular.z, time.monotonic()),
                     queue_size=1)
    rospy.Subscriber(ENABLE_TOPIC, Bool,
                     lambda msg: core.set_enabled(msg.data, time.monotonic()), queue_size=1)

    def on_scan(msg):
        core.update_lidar(lidar_valid_fraction(msg.ranges, msg.range_min, msg.range_max),
                          time.monotonic())

    def on_depth(msg):
        quality = depth_valid_fraction(msg.data, msg.encoding, msg.is_bigendian,
                                       msg.width, msg.height)
        core.update_depth(quality, time.monotonic())

    rospy.Subscriber('/scan', LaserScan, on_scan, queue_size=1)
    rospy.Subscriber('/depth_cam/depth/image_raw', Image, on_depth,
                     queue_size=1, buff_size=2 ** 22)
    rospy.Subscriber('/odom', Odometry,
                     lambda _msg: core.update_odom(time.monotonic()), queue_size=1)

    zero = Twist()

    def stop_chassis():
        for _attempt in range(5):
            try:
                output_pub.publish(zero)
                time.sleep(0.03)
            except Exception:
                break

    rospy.on_shutdown(stop_chassis)
    notify_systemd('READY=1\nSTATUS=Waiting for explicit Pause then Resume')
    rate = rospy.Rate(20)
    previous_reason = None
    while not rospy.is_shutdown():
        status = core.evaluate(time.monotonic())
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = status['output']
        output_pub.publish(command)
        status_pub.publish(String(data=json.dumps(status, sort_keys=True)))
        notify_systemd('WATCHDOG=1\nSTATUS=' + status['reason'])
        if status['reason'] != previous_reason:
            log = rospy.loginfo if status['armed'] else rospy.logwarn
            log('>> [SAFETY] %s', status['reason'])
            previous_reason = status['reason']
        rate.sleep()


if __name__ == '__main__':
    run_ros_node()
