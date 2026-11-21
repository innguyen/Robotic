"""Fresh direct map->odom TF without a Python-version-specific tf C extension.

AMCL and GMapping broadcast this direct edge. Other sensor transforms stay
owned by the robot drivers. Missing data never falls back to an identity pose.
"""
import math
import threading
import time

from navigation_core_ros1 import to_world


class MapFrame(object):
    def __init__(self, frame, odom_frame, timeout=0.8):
        import rospy
        from tf2_msgs.msg import TFMessage
        self.ros = rospy
        self.frame, self.odom_frame, self.timeout = frame, odom_frame, timeout
        self.transform = self.previous = None
        self.error, self.age = 'WAIT_LOCALIZATION', None
        self.received = None
        self.lock, self.sample = threading.Lock(), None
        # Melodic often ships tf's binary Python extension for Python 2. This
        # Python 3 app needs only the generated message on the direct map edge.
        self.subscriber = rospy.Subscriber('/tf', TFMessage, self.callback, queue_size=10)

    def callback(self, message):
        for msg in message.transforms:
            if (msg.header.frame_id.lstrip('/') != self.frame or
                    msg.child_frame_id.lstrip('/') != self.odom_frame):
                continue
            p, q = msg.transform.translation, msg.transform.rotation
            stamp = msg.header.stamp.to_sec()
            error = ''
            if (not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w, stamp)) or stamp <= 0 or
                    abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .1):
                error = 'INVALID_LOCALIZATION'
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)) if not error else 0.
            with self.lock:
                if not error and self.sample is not None and stamp <= self.sample[0]:
                    continue  # Repeated stamps never refresh the wall watchdog.
                self.sample = (stamp, (p.x, p.y, yaw), time.monotonic(), error)

    def refresh(self):
        with self.lock:
            sample = self.sample
        self.age = None
        if sample is None:
            self.error = 'WAIT_LOCALIZATION'
            return self.error
        stamp, transform, received, error = sample
        now = time.monotonic()
        age = self.ros.Time.now().to_sec()-stamp
        if error:
            self.error = error
        elif not -.5 <= age <= self.timeout or not 0 <= now-received <= self.timeout:
            self.error = 'STALE_LOCALIZATION'
        else:
            self.previous, self.transform = self.transform, transform
            self.error, self.age = '', max(0., age, now-received)
            self.received = now
        return self.error

    def pose(self, pose):
        xy = to_world([pose[:2]], self.transform)[0]
        return (float(xy[0]), float(xy[1]), pose[2]+self.transform[2])

    def point(self, point):
        return tuple(to_world([point], self.transform)[0])

    def correction(self):
        if self.previous is None or self.transform is None:
            return 0., 0.
        a, b = self.previous, self.transform
        return math.hypot(a[0]-b[0], a[1]-b[1]), abs((a[2]-b[2]+math.pi) % (2*math.pi)-math.pi)

    def close(self):
        self.subscriber.unregister()
