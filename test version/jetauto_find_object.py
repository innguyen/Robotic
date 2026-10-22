#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Astra YOLO recognition test. No motion commands or obstacle avoidance.
Run: python3 jetauto_find_object.py bottle --port 5010
Models: ../models by default, or --models-dir /home/jetauto/models.
Existing sensor drivers must already be running. Odometry is read-only.
"""
import argparse
import json
import math
import threading
import time
from collections import deque
from pathlib import Path
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from urllib.parse import urlparse
import cv2
import numpy as np
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, Path as RosPath
from sensor_msgs.msg import Image as RosImage, CameraInfo
from std_msgs.msg import String

TWO_PI = 2.0 * math.pi
DETECTION_CONFIDENCE = 0.35
TARGET_CONFIDENCE_DEFAULT = 0.45
TARGET_CONFIDENCE = {'person': 0.40, 'chair': 0.40, 'bottle': 0.45, 'cup': 0.45,
                     'cell phone': 0.50, 'remote': 0.50}
DEPTH_SYNC_TOLERANCE_S = 0.08
SENSOR_TIMEOUT_S = 0.5
TARGET_CLASS = 'person'
raw_frame_astra = None
cam_source_astra = 'Waiting for Astra'
frame_lock = threading.Lock()

class TrackingConfig:
    astra_x, astra_y, astra_height = -0.131, 0.02, 0.462
    astra_pitch_deg, astra_roll_deg = 7.9, 0.0
    target_memory_s, target_memory_small_s = 5.0, 1.2
    target_confirm_hits, target_confirm_hits_small = 2, 3
    target_max_bearing_jump_deg = 18.0
    target_max_distance_jump, target_max_distance_ratio = 0.60, 0.45
    target_width = {'person': 0.42, 'chair': 0.48, 'couch': 1.5, 'dining table': 0.9,
                    'suitcase': 0.45, 'cell phone': 0.075, 'bottle': 0.07, 'cup': 0.09,
                    'remote': 0.05, 'laptop': 0.33, 'tv': 0.75, 'book': 0.16,
                    'backpack': 0.30, 'sports ball': 0.22, 'teddy bear': 0.25}
    def strict_target_tracking_for(self, label):
        return label in ('bottle', 'cup', 'cell phone', 'remote', 'book', 'chair')
    def target_memory_for(self, label):
        return self.target_memory_small_s if label in ('bottle', 'cup', 'cell phone', 'remote', 'book') else self.target_memory_s
    def target_confirm_hits_for(self, label):
        return self.target_confirm_hits_small if self.strict_target_tracking_for(label) else self.target_confirm_hits
    def standoff_for(self, label):
        # Compatibility field in TargetEstimate; this app never plans movement.
        return 0.0

class CameraGeometry:
    """Fixed measured camera mounting, used only to locate targets."""
    def __init__(self, cfg):
        self.cfg = cfg
        pitch, roll = math.radians(cfg.astra_pitch_deg), math.radians(cfg.astra_roll_deg)
        self.up = np.array([math.cos(pitch)*math.sin(roll), -math.cos(pitch)*math.cos(roll), -math.sin(pitch)])
        self.height = cfg.astra_height
        self.forward = np.array([0., 0., 1.]) - self.up[2]*self.up
        self.forward /= np.linalg.norm(self.forward)
        self.left = np.cross(self.up, self.forward)
    def to_base(self, pts_cam):
        p = np.asarray(pts_cam, dtype=float).reshape(-1, 3)
        return np.column_stack((self.cfg.astra_x + p.dot(self.forward),
                                self.cfg.astra_y + p.dot(self.left),
                                p.dot(self.up) + self.height))


def wrap_scalar(angle):
    return (angle + math.pi) % TWO_PI - math.pi

class TargetEstimate(object):
    __slots__ = ('valid', 'visible', 'bearing', 'distance', 'age', 'standoff', 'radius', 'label', 'source')

    def __init__(self):
        self.valid = False
        self.visible = False
        self.bearing = 0.0
        self.distance = None
        self.age = float('inf')
        self.standoff = 0.35
        self.radius = 0.25
        self.label = ''
        self.source = ''

class TargetTracker(object):
    """Chỉ khóa các quan sát liên tiếp, nhất quán; mất dấu thì nhớ vị trí ngắn hạn trong odom."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset(None)

    def reset(self, label=None):
        self.label = label
        self.hits = 0
        self.confirmed = False
        self.last_time = None
        self.bearing = None
        self.distance = None
        self.distance_time = None
        self.source = ''
        self.anchor = None
        self.anchor_heading = None

    def observe(self, label, bearing, distance, now, pose=None, source=''):
        if label != self.label:
            self.reset(label)
        recent = self.last_time is not None and now - self.last_time <= 0.8
        strict = self.cfg.strict_target_tracking_for(label)
        expected_bearing, expected_distance = self.bearing, self.distance
        if strict and recent and pose is not None and self.anchor is not None:
            x, y, yaw = pose
            dx, dy = self.anchor[0] - x, self.anchor[1] - y
            expected_bearing = wrap_scalar(math.atan2(dy, dx) - yaw)
            expected_distance = math.hypot(dx, dy)
        consistent = recent and expected_bearing is not None
        if strict and consistent:
            jump = abs(wrap_scalar(bearing - expected_bearing))
            consistent = jump <= math.radians(self.cfg.target_max_bearing_jump_deg)
        if strict and consistent and distance is not None and expected_distance is not None:
            limit = max(self.cfg.target_max_distance_jump,
                        self.cfg.target_max_distance_ratio * max(expected_distance, 0.1))
            consistent = abs(distance - expected_distance) <= limit
        if not consistent:
            self.hits = 1
            if strict:
                self.confirmed = False
            self.bearing = bearing
            self.distance = distance
        else:
            self.hits += 1
            self.bearing = wrap_scalar(expected_bearing + 0.6 * wrap_scalar(bearing - expected_bearing))
            if distance is None:
                self.distance = expected_distance
        if self.hits >= self.cfg.target_confirm_hits_for(label):
            self.confirmed = True
        if distance is not None:
            if not consistent or self.distance is None or self.distance_time is None or now - self.distance_time > 1.0:
                self.distance = distance
            else:
                self.distance = expected_distance + 0.5 * (distance - expected_distance)
            self.distance_time = now
        elif self.distance_time is not None and now - self.distance_time > 1.0:
            self.distance = None
        self.last_time = now
        self.source = source
        if pose is not None:
            x, y, yaw = pose
            self.anchor_heading = yaw + self.bearing
            self.anchor = None
            if self.distance is not None:
                self.anchor = (x + self.distance * math.cos(self.anchor_heading),
                               y + self.distance * math.sin(self.anchor_heading))

    def missed(self, now):
        if self.last_time is None or now - self.last_time > 0.8:
            self.hits = 0

    def estimate(self, now, pose=None):
        est = TargetEstimate()
        est.label = self.label or ''
        if not self.confirmed or self.last_time is None:
            return est
        age = now - self.last_time
        if age > self.cfg.target_memory_for(self.label):
            self.confirmed = False
            return est
        bearing, distance = self.bearing, self.distance
        if pose is not None and age > 0.05:
            x, y, yaw = pose
            if self.anchor is not None:
                dx, dy = self.anchor[0] - x, self.anchor[1] - y
                distance = math.hypot(dx, dy)
                bearing = wrap_scalar(math.atan2(dy, dx) - yaw)
            elif self.anchor_heading is not None:
                bearing = wrap_scalar(self.anchor_heading - yaw)
        est.valid = True
        est.visible = age <= 0.6
        est.bearing = float(bearing)
        est.distance = distance
        est.age = age
        est.standoff = self.cfg.standoff_for(self.label)
        est.radius = 0.35 if self.label == 'person' else 0.25
        est.source = self.source
        return est

def locate_target(box, rgb_intrinsics, astra, depth_mm, depth_intrinsics, cfg, label, rgb_distortion=None):
    """Từ khung YOLO trên ảnh RGB Astra -> (góc lệch, khoảng cách tới tâm xe, nguồn).

    Ưu tiên đo độ sâu thật quanh tâm khung; không có thì ước lượng theo bề rộng vật (bảng target_width)."""
    fx, fy, cx, cy = rgb_intrinsics
    bx, by, bw, bh = [float(v) for v in box]
    u, v = bx + bw * 0.5, by + bh * 0.5
    xn, yn = (u - cx) / fx, (v - cy) / fy
    if cv2 is not None and rgb_distortion is not None and any(abs(k) > 1e-9 for k in rgb_distortion):
        try:
            und = cv2.undistortPoints(np.array([[[u, v]]], dtype=np.float32),
                                      np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64),
                                      np.asarray(rgb_distortion, dtype=np.float64))
            xn, yn = float(und[0, 0, 0]), float(und[0, 0, 1])
        except Exception:
            pass
    width = cfg.target_width.get(label)
    size_distance = None
    if width is not None and bw >= 2:
        size_distance = max(0.3, min(6.0, fx * width / bw))
    if depth_mm is not None and depth_intrinsics is not None:
        dfx, dfy, dcx, dcy = depth_intrinsics
        h, w = depth_mm.shape[:2]
        ud, vd = dcx + dfx * xn, dcy + dfy * yn
        rx = max(3, int(0.15 * bw * dfx / fx))
        ry = max(3, int(0.15 * bh * dfy / fy))
        u0, u1 = int(max(0, ud - rx)), int(min(w, ud + rx + 1))
        v0, v1 = int(max(0, vd - ry)), int(min(h, vd + ry + 1))
        if u1 > u0 and v1 > v0:
            roi = np.asarray(depth_mm[v0:v1, u0:u1], dtype=float).ravel()
            roi = roi[(roi >= 300) & (roi <= 6000)]
            if roi.size >= 8:
                z = float(np.percentile(roi, 25)) / 1000.0
                # Vật nhỏ/trong suốt thường không có depth; ROI khi đó trả về tường phía sau.
                # Chỉ tin depth nếu nó còn tương thích với khoảng cách ước lượng từ bề rộng box.
                rigid_small = label in ('bottle', 'cup', 'cell phone', 'remote', 'book')
                depth_matches_box = (not rigid_small or size_distance is None
                                     or 0.5 * size_distance <= z <= 1.8 * size_distance)
                if depth_matches_box:
                    p = astra.to_base([[xn * z, yn * z, z]])[0]
                    return math.atan2(p[1], p[0]), math.hypot(p[0], p[1]), 'DO SAU'
    # Không có độ sâu: hướng theo tia camera, khoảng cách theo bề rộng vật
    ray = np.array([xn, yn, 1.0])
    fwd, lft = float(ray.dot(astra.forward)), float(ray.dot(astra.left))
    cam_bearing = math.atan2(lft, fwd)
    if size_distance is None:
        return cam_bearing, None, 'HUONG'
    dist_cam = size_distance
    px = cfg.astra_x + dist_cam * math.cos(cam_bearing)
    py = cfg.astra_y + dist_cam * math.sin(cam_bearing)
    return math.atan2(py, px), math.hypot(px, py), 'KICH THUOC'

class ObjectSeeker:
    def __init__(self, models_dir):
        rospy.init_node('jetauto_vision_test', anonymous=True)
        self.models_dir = Path(models_dir)
        self.tracking_config = TrackingConfig()
        self.tracker = TargetTracker(self.tracking_config)
        self.astra = CameraGeometry(self.tracking_config)
        self.sensor_lock, self.target_lock = threading.Lock(), threading.Lock()
        self.astra_lock, self.state_lock = threading.Lock(), threading.Lock()
        self.depth_buffer = deque(maxlen=12)
        self.odom_history = deque(maxlen=200)
        self.odom_frame = rospy.get_param('~odom_frame', 'odom').lstrip('/')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint').lstrip('/')
        self.last_observation_stamp = 0.0
        self.last_world_valid = False
        self.target_publisher = rospy.Publisher(
            rospy.get_param('~target_topic', '/vision/target'), String, queue_size=1)
        self.map_goal_publisher = rospy.Publisher('/jetauto_navigation/map_goal', String, queue_size=1)
        self.depth_intrinsics = self.rgb_intrinsics = self.rgb_distortion = None
        self.rgb_stamp = None
        self.odom_pose = None
        self.odom_time = self.camera_time = self.depth_time = 0.0
        self.frame_seq = 0
        self.ai_fps = 0.0
        self.last_depth_sync_ms = self.last_target_confidence = None
        self.last_detections = []
        self.navigation_state = {'state': 'WAIT_NAVIGATION', 'enabled': False}
        self.navigation_received = 0.0
        self.local_map = None
        self.map_received = 0.0
        self.local_path = []
        self.jpeg = None
        self.classes = self.load_classes()
        model_path = str(self.models_dir / 'yolov5n.onnx')
        if not Path(model_path).is_file():
            raise RuntimeError('Missing model: ' + model_path)
        try:
            import onnxruntime as ort
            options = ort.SessionOptions()
            options.intra_op_num_threads = 4
            available = ort.get_available_providers()
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if 'CUDAExecutionProvider' in available else ['CPUExecutionProvider']
            self.session = ort.InferenceSession(model_path, sess_options=options, providers=providers)
            self.input_name = self.session.get_inputs()[0].name
            self.use_ort = True
        except Exception as exc:
            rospy.logwarn('Using OpenCV DNN: %s', exc)
            self.net = cv2.dnn.readNetFromONNX(model_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.use_ort = False
        self.ros_subscribers = [
            rospy.Subscriber('/depth_cam/rgb/image_raw', RosImage, self._callback_ros_astra, queue_size=1, buff_size=2**24),
            rospy.Subscriber('/depth_cam/depth/image_raw', RosImage, self._callback_depth, queue_size=1, buff_size=2**24),
            rospy.Subscriber('/depth_cam/rgb/camera_info', CameraInfo, self._callback_rgb_info, queue_size=1),
            rospy.Subscriber('/depth_cam/depth/camera_info', CameraInfo, self._callback_depth_info, queue_size=1),
            rospy.Subscriber('/odom', Odometry, self._callback_odom, queue_size=1),
            rospy.Subscriber('/jetauto_navigation/state', String, self._callback_navigation_state, queue_size=1),
            rospy.Subscriber('/jetauto_navigation/costmap', OccupancyGrid, self._callback_costmap, queue_size=1),
            rospy.Subscriber('/jetauto_navigation/path', RosPath, self._callback_path, queue_size=1),
        ]
        self.ai_thread = threading.Thread(target=self.ai_worker_loop, daemon=True)
        self.ai_thread.start()

    def load_classes(self):
        return [s.strip() for s in (self.models_dir / 'coco.names').read_text().splitlines() if s.strip()]

    def _callback_navigation_state(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                return
            with self.state_lock:
                self.navigation_state = data
                self.navigation_received = time.monotonic()
        except (ValueError, TypeError):
            pass

    def set_navigation_enabled(self, enabled):
        from std_srvs.srv import SetBool
        if type(enabled) is not bool:
            raise ValueError('enabled must be a boolean')
        rospy.wait_for_service('/jetauto_navigation/enable', timeout=2.0)
        response = rospy.ServiceProxy('/jetauto_navigation/enable', SetBool)(enabled)
        if not response.success:
            raise RuntimeError(response.message)
        return response.message

    def set_map_goal(self, data):
        mode = data.get('mode')
        if mode not in ('point', 'vision'):
            raise ValueError('Chế độ mục tiêu không hợp lệ.')
        goal = dict(mode=mode, frame_id=self.odom_frame, stamp=rospy.Time.now().to_sec())
        if mode == 'point':
            x, y = float(data['x']), float(data['y'])
            if not all(math.isfinite(v) for v in (x, y)):
                raise ValueError('Tọa độ không hợp lệ.')
            with self.state_lock:
                grid = self.local_map
                if grid is None or time.monotonic()-self.map_received > 1.:
                    raise ValueError('Bản đồ đã cũ; chờ LiDAR và odometry cập nhật.')
                col = int(math.floor((x-grid['origin'][0])/grid['resolution']))
                row = int(math.floor((y-grid['origin'][1])/grid['resolution']))
                if not (0 <= col < grid['width'] and 0 <= row < grid['height']) or grid['data'][row*grid['width']+col] != 0:
                    raise ValueError('Chọn điểm trong vùng trống đã quan sát, ngoài vùng đỏ/vàng/vật cản.')
            goal.update(x=x, y=y)
        if self.map_goal_publisher.get_num_connections() == 0:
            raise RuntimeError('Chưa kết nối bộ điều hướng; chưa gửi điểm đến.')
        self.map_goal_publisher.publish(String(data=json.dumps(goal, allow_nan=False)))

    def _callback_costmap(self, msg):
        width, height = msg.info.width, msg.info.height
        origin = msg.info.origin
        values = (msg.info.resolution, origin.position.x, origin.position.y)
        if (msg.header.frame_id.lstrip('/') != self.odom_frame
                or not 0 < width <= 201 or not 0 < height <= 201
                or len(msg.data) != width*height or not all(math.isfinite(v) for v in values)
                or msg.info.resolution <= 0):
            return
        data = dict(width=width, height=height, resolution=msg.info.resolution,
                    origin=[origin.position.x, origin.position.y], data=list(msg.data))
        with self.state_lock:
            self.local_map = data
            self.map_received = time.monotonic()

    def _callback_path(self, msg):
        if msg.header.frame_id.lstrip('/') != self.odom_frame:
            return
        points = [[p.pose.position.x, p.pose.position.y] for p in msg.poses[:2000]]
        if not all(math.isfinite(v) for point in points for v in point):
            return
        with self.state_lock:
            self.local_path = points

    def map_status(self):
        now = time.monotonic()
        with self.state_lock:
            result = dict(map=self.local_map, path=self.local_path,
                          age_s=round(now-self.map_received, 2) if self.map_received else None,
                          target=self.navigation_state.get('target'),
                          approach=self.navigation_state.get('planning', {}).get('approach_point'))
        with self.sensor_lock:
            result['robot'] = self.odom_pose if now-self.odom_time <= SENSOR_TIMEOUT_S else None
        return result

    def _callback_depth_info(self, msg):
        self.depth_intrinsics = (msg.K[0], msg.K[4], msg.K[2], msg.K[5])

    def _callback_depth(self, msg):
        depth = self._decode_depth(msg)
        if depth is not None:
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
            with self.sensor_lock:
                self.depth_buffer.append((stamp, depth))
                self.depth_time = time.monotonic()

    def reset_target(self, label):
        global TARGET_CLASS
        if label not in self.classes:
            raise ValueError('Unknown COCO class')
        with self.target_lock:
            TARGET_CLASS = label
            self.tracker.reset(label)
            self.last_world_valid = False
            self.target_publisher.publish(String(data=json.dumps({'valid': False, 'label': label})))

    def _publish_target(self, estimate, label):
        """Publish perception only; no velocity command is produced by this file."""
        valid = bool(estimate.valid and self.last_world_valid and self.tracker.anchor is not None)
        data = dict(valid=valid, label=label, frame_id=self.odom_frame,
                    stamp=self.last_observation_stamp, source=estimate.source)
        if valid:
            data.update(x=self.tracker.anchor[0], y=self.tracker.anchor[1])
        else:
            data['reason'] = 'NO_CONFIRMED_TARGET' if not estimate.valid else 'TARGET_NEEDS_MATCHED_ODOMETRY_OR_RANGE'
        self.target_publisher.publish(String(data=json.dumps(data, allow_nan=False)))


    def _callback_ros_astra(self, msg):
        global raw_frame_astra, cam_source_astra
        frame = self._decode_ros_image(msg)
        if frame is not None:
            cam_source_astra = "Astra Pro (ROS)"
            stamp = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
            with frame_lock:
                raw_frame_astra = frame
                self.rgb_stamp = stamp
                self.frame_seq += 1
            self.camera_time = time.monotonic()

    def _callback_rgb_info(self, msg):
        self.rgb_intrinsics = (msg.K[0], msg.K[4], msg.K[2], msg.K[5])
        self.rgb_distortion = list(msg.D)[:5]

    @staticmethod
    def _decode_depth(msg):
        """Anh do sau ROS -> mang uint16 (mm), khong copy du lieu."""
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
                row = msg.step // 2 if msg.step else msg.width
                return np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row)[:, :msg.width]
            if msg.encoding == '32FC1':
                dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
                row = msg.step // 4 if msg.step else msg.width
                depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row)[:, :msg.width]
                return np.clip(np.nan_to_num(depth) * 1000.0, 0, 65535).astype(np.uint16)
        except (ValueError, TypeError):
            pass
        return None

    def _callback_odom(self, msg):
        q = msg.pose.pose.orientation
        stamp = msg.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        values = (q.x, q.y, q.z, q.w, msg.pose.pose.position.x, msg.pose.pose.position.y)
        if (msg.header.frame_id.lstrip('/') != self.odom_frame
                or msg.child_frame_id.lstrip('/') != self.base_frame
                or not all(math.isfinite(v) for v in values)
                or abs(sum(v*v for v in values[:4])-1.0) > 0.1
                or stamp <= 0 or not -0.05 <= age <= SENSOR_TIMEOUT_S):
            return
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.sensor_lock:
            if self.odom_history and stamp <= self.odom_history[-1][0]:
                return
            self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
            self.odom_time = time.monotonic() - max(0.0, age)
            self.odom_history.append((stamp, self.odom_pose))

    def _decode_ros_image(self, msg):
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgra8' or frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            elif frame.ndim == 2 or frame.shape[2] == 1:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            return frame
        except Exception:
            return None

    def run_yolo(self, frame):
        INPUT_SIZE = 640
        h, w = frame.shape[:2]
        x_scale = w / float(INPUT_SIZE)
        y_scale = h / float(INPUT_SIZE)

        blob = cv2.dnn.blobFromImage(frame, 1.0 / 255.0, (INPUT_SIZE, INPUT_SIZE), swapRB=True, crop=False)

        if self.use_ort:
            outputs = self.session.run(None, {self.input_name: blob})
            arr = outputs[0]
        else:
            self.net.setInput(blob)
            arr = self.net.forward()

        arr = np.asarray(arr)
        if arr.ndim == 3:
            arr = np.squeeze(arr, axis=0)
        if arr.shape[0] < arr.shape[1]:
            arr = arr.T

        if len(arr) == 0:
            return []

        if arr.shape[1] == 85:
            obj_mask = arr[:, 4] > DETECTION_CONFIDENCE
            arr = arr[obj_mask]
            if len(arr) == 0:
                return []
            scores = arr[:, 5:]
            class_ids = np.argmax(scores, axis=1)
            confidences = np.max(scores, axis=1) * arr[:, 4]
        else:
            scores = arr[:, 4:]
            class_ids = np.argmax(scores, axis=1)
            confidences = np.max(scores, axis=1)

        conf_mask = confidences > DETECTION_CONFIDENCE
        if not np.any(conf_mask):
            return []

        arr = arr[conf_mask]
        confidences = confidences[conf_mask]
        class_ids = class_ids[conf_mask]

        cx = arr[:, 0] * x_scale
        cy = arr[:, 1] * y_scale
        bw = arr[:, 2] * x_scale
        bh = arr[:, 3] * y_scale
        x = (cx - bw / 2.0).astype(int)
        y = (cy - bh / 2.0).astype(int)
        bw = bw.astype(int)
        bh = bh.astype(int)

        boxes = np.column_stack((x, y, bw, bh)).tolist()
        conf_list = confidences.tolist()
        cid_list = class_ids.tolist()

        try:
            # NMS theo từng lớp: người đứng sát ghế không bị khung ghế "nuốt" mất
            indices = []
            for class_id in sorted(set(cid_list)):
                group = [i for i, cid in enumerate(cid_list) if cid == class_id]
                kept = cv2.dnn.NMSBoxes([boxes[i] for i in group], [conf_list[i] for i in group],
                                        DETECTION_CONFIDENCE, 0.45)
                indices.extend(group[int(k)] for k in np.array(kept).flatten())
        except Exception:
            indices = list(range(len(boxes)))

        results = []
        if len(indices) > 0:
            for i in np.array(indices).flatten():
                results.append({
                    'class_id': cid_list[i],
                    'class_name': self.classes[cid_list[i]] if cid_list[i] < len(self.classes) else 'unknown',
                    'conf': conf_list[i],
                    'box': boxes[i]
                })
        return results

    def _rgb_intrinsics(self, shape):
        h, w = shape[:2]
        if self.rgb_intrinsics is None:
            f = w / (2.0 * math.tan(math.radians(30.5)))   # Astra RGB nhìn ngang ~61 độ
            return (f, f, w / 2.0, h / 2.0)
        fx, fy, cx, cy = self.rgb_intrinsics
        scale = w / (2.0 * cx) if cx > 0 and abs(w - 2.0 * cx) > 40 else 1.0   # ảnh khác cỡ camera_info
        return (fx * scale, fy * scale, cx * scale, cy * scale)

    def _locate_target(self, box, shape, with_depth=True, depth_frame=None):
        depth = depth_frame if with_depth else None
        with self.astra_lock:
            return locate_target(box, self._rgb_intrinsics(shape), self.astra, depth, self.depth_intrinsics,
                                    self.tracking_config, TARGET_CLASS, self.rgb_distortion)

    def _pick_target(self, targets, shape):
        """Nhiều vật cùng lớp: ưu tiên vật gần hướng đang bám (không nhảy qua lại), rồi tới độ tin cậy."""
        with self.target_lock:
            recent = self.tracker.last_time is not None and time.monotonic() - self.tracker.last_time < 1.5
            last = self.tracker.bearing if recent else None
        if last is None or len(targets) == 1:
            return max(targets, key=lambda d: d['conf'])
        return max(targets, key=lambda d: d['conf'] - 0.8 * abs(wrap_scalar(
            self._locate_target(d['box'], shape, with_depth=False)[0] - last)))

    def ai_worker_loop(self):
        frame_counter = 0
        last_fps_time = time.time()
        current_ai_fps = 0.0
        last_seq = -1

        while not rospy.is_shutdown():
            raw_frame = None
            rgb_stamp = None
            with frame_lock:
                if raw_frame_astra is not None and self.frame_seq != last_seq:
                    raw_frame = raw_frame_astra.copy()
                    rgb_stamp = self.rgb_stamp
                    last_seq = self.frame_seq

            if raw_frame is None:
                time.sleep(0.01)
                continue

            frame_counter += 1
            now_wall = time.time()
            if now_wall - last_fps_time >= 1.0:
                current_ai_fps = frame_counter / (now_wall - last_fps_time)
                frame_counter = 0
                last_fps_time = now_wall

            label = TARGET_CLASS
            detections = self.run_yolo(raw_frame)
            now = time.monotonic()
            raw_targets = [d for d in detections if d['class_name'] == label]
            min_confidence = TARGET_CONFIDENCE.get(label, TARGET_CONFIDENCE_DEFAULT)
            targets = [d for d in raw_targets if d['conf'] >= min_confidence]
            best = self._pick_target(targets, raw_frame.shape) if targets else None

            # Ghép depth gần nhất với đúng frame RGB. YOLO trên Jetson mất khoảng 250 ms,
            # nên lấy "depth mới nhất" sau inference sẽ đo một thời điểm khác khi xe đang chạy.
            matched_depth = None
            depth_sync_ms = None
            if rgb_stamp:
                with self.sensor_lock:
                    if self.depth_buffer:
                        depth_stamp, candidate = min(self.depth_buffer,
                                                     key=lambda item: abs(item[0] - rgb_stamp))
                        delta = abs(depth_stamp - rgb_stamp)
                        depth_sync_ms = 1000.0 * delta
                        if depth_stamp and delta <= DEPTH_SYNC_TOLERANCE_S:
                            matched_depth = candidate

            with self.sensor_lock:
                pose = self.odom_pose if (self.odom_pose is not None and now - self.odom_time < SENSOR_TIMEOUT_S) else None
            target_center = None
            target_info = None
            if best is not None:
                # Hướng + khoảng cách tới tâm xe (đo bằng độ sâu Astra, không có thì ước lượng theo kích thước)
                bearing, distance, source = self._locate_target(best['box'], raw_frame.shape,
                                                                 depth_frame=matched_depth)
                with self.sensor_lock:
                    # Anchor the detection using odometry near RGB acquisition,
                    # not the latest pose after inference has finished.
                    sample = min(self.odom_history, key=lambda item: abs(item[0]-rgb_stamp)) if self.odom_history and rgb_stamp else None
                    pose = sample[1] if sample is not None and abs(sample[0]-rgb_stamp) <= 0.15 else None
                with self.target_lock:
                    if TARGET_CLASS != label:
                        continue
                    if self.tracker.label != label:
                        self.tracker.reset(label)
                    self.tracker.observe(label, bearing, distance, now, pose, source)
                    self.last_observation_stamp = rgb_stamp or 0.0
                    self.last_world_valid = pose is not None and distance is not None
                bx, by, bw, bh = best['box']
                target_center = (bx + bw // 2, by + bh // 2)
                target_info = {'distance': distance, 'source': source}
            else:
                with self.target_lock:
                    self.tracker.missed(now)

            with self.target_lock:
                if TARGET_CLASS != label:
                    continue
                estimate = self.tracker.estimate(now, pose)
                self._publish_target(estimate, label)
                tracker_hits = self.tracker.hits
                required_hits = self.tracking_config.target_confirm_hits_for(label)
            text = 'CONFIRMED' if estimate.valid else 'VERIFYING'
            if best is None:
                text = 'MEMORY' if estimate.valid else 'NO TARGET'
            for det in detections:
                x, y, w, h = det['box']
                color = ((0, 255, 0) if det['conf'] >= min_confidence else (0, 180, 255)) if det['class_name'] == label else (140, 140, 140)
                cv2.rectangle(raw_frame, (x, y), (x+w, y+h), color, 2)
                cv2.putText(raw_frame, '{} {:.0f}%'.format(det['class_name'], 100*det['conf']),
                            (x, max(18, y-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.putText(raw_frame, '{} | {} | {:.1f} FPS'.format(label, text, current_ai_fps),
                        (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 230, 255), 2)
            cv2.putText(raw_frame, 'HITS {}/{} | MIN CONF {:.0f}%'.format(tracker_hits, required_hits, min_confidence*100),
                        (15, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 230, 255), 1)
            if target_info and target_info['distance'] is not None:
                cv2.putText(raw_frame, '{:.2f} m | {}'.format(target_info['distance'], target_info['source']),
                            (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 230, 255), 2)
            ok, encoded = cv2.imencode('.jpg', raw_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with self.state_lock:
                if ok:
                    self.jpeg = encoded.tobytes()
                self.last_detections = detections
                self.best_target_center = target_center
                candidate = max(raw_targets, key=lambda d: d['conf']) if raw_targets else None
                self.last_target_confidence = candidate['conf'] if candidate is not None else None
                self.last_target_box = candidate['box'] if candidate is not None else None
                self.last_depth_sync_ms = depth_sync_ms
                self.ai_fps = current_ai_fps


    def status(self):
        now = time.monotonic()
        with self.sensor_lock:
            pose = self.odom_pose if now-self.odom_time < SENSOR_TIMEOUT_S else None
        with self.target_lock:
            estimate = self.tracker.estimate(now, pose)
            result = {'target': TARGET_CLASS, 'confirmed': estimate.valid,
                      'visible': estimate.visible, 'hits': self.tracker.hits,
                      'required_hits': self.tracking_config.target_confirm_hits_for(TARGET_CLASS),
                      'confidence_threshold': TARGET_CONFIDENCE.get(TARGET_CLASS, TARGET_CONFIDENCE_DEFAULT),
                      'bearing_deg': math.degrees(estimate.bearing) if estimate.valid else None,
                      'distance_m': estimate.distance if estimate.valid else None,
                      'source': estimate.source, 'classes': self.classes}
        with self.state_lock:
            result.update(ai_fps=self.ai_fps, confidence=self.last_target_confidence,
                          depth_sync_ms=self.last_depth_sync_ms)
            result['navigation'] = (dict(self.navigation_state) if now-self.navigation_received <= 1.0
                                    else {'state': 'WAIT_NAVIGATION', 'enabled': False})
        output = rospy.get_param('/jetauto_collision_monitor/output_topic', '/collision_monitor/cmd_vel_checked')
        result['mode'] = 'live' if output == '/jetauto_controller/cmd_vel' else 'preview'
        result['camera_age_s'] = round(now-self.camera_time, 2) if self.camera_time else None
        return result

PAGE = """<!doctype html><html lang="vi"><meta charset="utf-8">
<title>JetAuto Vision Test</title><style>
body{background:#101827;color:#eee;font:16px system-ui;max-width:1000px;margin:30px auto;padding:16px}
img{width:100%;border-radius:12px}select,button{font:inherit;padding:8px}pre{white-space:pre-wrap}
.views{display:grid;grid-template-columns:1fr 1fr;gap:16px}canvas{width:100%;background:#0b1220;border-radius:12px}#reason{white-space:pre-line;line-height:1.65;padding:12px;background:#1e293b;border-radius:8px}@media(max-width:800px){.views{grid-template-columns:1fr}}
</style><h1>JetAuto — Nhận diện và điều hướng</h1>
<p id="mode">Đang kết nối…</p>
<select id="target"></select> <button id="apply">Chọn mục tiêu</button>
<button id="startNav">Bắt đầu</button> <button id="pauseNav">Tạm dừng</button>
<p><button id="pickPoint">Chọn điểm trên bản đồ</button> <button id="goPoint" disabled>Đi tới điểm đã chọn</button> <button id="visionMode">Theo mục tiêu camera</button></p>
<p id="pointInfo">Chọn điểm rồi bấm Đi tới điểm đã chọn để bắt đầu. Tạm dừng dùng chung cho cả hai chế độ.</p>
<p id="error"></p><p id="reason">Đang chờ trạng thái điều hướng…</p>
<div class="views"><div><h3>Camera Astra</h3><img src="/stream" alt="Camera Astra"></div>
<div><h3>Bản đồ vật cản cục bộ</h3><canvas id="map" width="480" height="480"></canvas>
<p id="mapInfo">Đang chờ bản đồ…</p><small>Xanh: vật cản · Vàng: đệm 6 cm · Đỏ sẫm: không đủ chỗ cho thân xe · Tím: đường đi · Cam: robot · Trắng: điểm tiếp cận. Đây là bản đồ cục bộ theo odometry, chưa phải SLAM.</small></div></div><pre id="status"></pre>
<script src="/navigation_status.js"></script><script>
let initialized=false, pickingPoint=false, selectedPoint=null, latestMap=null, mapFetchedAt=0;
async function refresh(){try{const r=await fetch('/status');const d=await r.json();
document.getElementById('mode').textContent=(d.mode==='live'?'CHẠY THẬT — có điều khiển động cơ':'CHẠY THỬ — chỉ xem lệnh, không gửi tới động cơ')+' | '+d.navigation.state;
document.getElementById('startNav').textContent=d.mode==='live'?'Bắt đầu di chuyển':'Bắt đầu thử lệnh';
document.getElementById('reason').textContent=explainNavigation(d);
if(!initialized){for(const c of d.classes){const o=document.createElement('option');o.value=c;o.textContent=c;target.appendChild(o)}target.value=d.target;initialized=true}
delete d.classes;document.getElementById('status').textContent=JSON.stringify(d,null,2);
}catch(e){document.getElementById('reason').textContent='MẤT CẬP NHẬT TRẠNG THÁI: '+e.message+' (không thể xác nhận trạng thái robot hiện tại)';document.getElementById('error').textContent=e.message}}
const target=document.getElementById('target');
document.getElementById('apply').onclick=async()=>{try{const r=await fetch('/target',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({label:target.value})});if(!r.ok)throw Error(await r.text());await refresh()}catch(e){document.getElementById('error').textContent=e.message}};
async function navigation(enabled){try{const r=await fetch('/navigation',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({enabled})});if(!r.ok)throw Error(await r.text());document.getElementById('error').textContent='';await refresh()}catch(e){document.getElementById('error').textContent=e.message}}
document.getElementById('startNav').onclick=()=>navigation(true);
document.getElementById('pauseNav').onclick=()=>navigation(false);
document.getElementById('pickPoint').onclick=()=>{pickingPoint=true;document.getElementById('map').style.cursor='crosshair';document.getElementById('pointInfo').textContent='Nhấp vào vùng trống trên bản đồ. Robot chỉ chạy khi bấm Đi tới điểm đã chọn.'};
document.getElementById('map').onclick=e=>{
if(!pickingPoint)return;
try{
if(!latestMap||latestMap.age_s+(performance.now()-mapFetchedAt)/1000>1)throw Error('Bản đồ cũ; chờ bản đồ mới trước khi chọn điểm.');
const m=latestMap.map,r=e.currentTarget.getBoundingClientRect();
const col=Math.floor((e.clientX-r.left)/r.width*m.width),row=m.height-1-Math.floor((e.clientY-r.top)/r.height*m.height);
if(col<0||row<0||col>=m.width||row>=m.height||m.data[row*m.width+col]!==0)throw Error('Điểm phải nằm trong vùng trống đã quan sát, ngoài vùng đỏ/vàng.');
selectedPoint=[m.origin[0]+(col+.5)*m.resolution,m.origin[1]+(row+.5)*m.resolution];
pickingPoint=false;e.currentTarget.style.cursor='default';document.getElementById('goPoint').disabled=false;
document.getElementById('pointInfo').textContent='Điểm đã chọn: x='+selectedPoint[0].toFixed(2)+' m, y='+selectedPoint[1].toFixed(2)+' m. Bấm Đi tới điểm đã chọn.';
}catch(e){document.getElementById('pointInfo').textContent=e.message}
};
async function sendGoal(mode){try{
const body=mode==='point'?{mode,x:selectedPoint[0],y:selectedPoint[1]}:{mode};
const r=await fetch('/map_goal',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
if(!r.ok)throw Error(await r.text());
document.getElementById('pointInfo').textContent=mode==='point'?'Đã gửi điểm đến. Xem trạng thái chấp nhận/tìm đường phía dưới.':'Đã yêu cầu theo camera; bấm Bắt đầu khi mục tiêu được xác nhận.';
selectedPoint=null;document.getElementById('goPoint').disabled=true;await refresh();
}catch(e){document.getElementById('pointInfo').textContent=e.message}}
document.getElementById('goPoint').onclick=()=>sendGoal('point');
document.getElementById('visionMode').onclick=()=>sendGoal('vision');
refresh();setInterval(refresh,500);
async function refreshMap(){try{const r=await fetch('/map');const d=await r.json();const canvas=document.getElementById('map');const ctx=canvas.getContext('2d');ctx.clearRect(0,0,480,480);if(!d.map){document.getElementById('mapInfo').textContent='Chưa có bản đồ: cần odometry và LiDAR hợp lệ.';return}
const m=d.map,sx=480/m.width,sy=480/m.height;
latestMap=d;mapFetchedAt=performance.now();
for(let row=0;row<m.height;row++)for(let col=0;col<m.width;col++){const v=m.data[row*m.width+col];ctx.fillStyle=v<0?'#101827':v===100?'#22d3ee':v===99?'#a87826':v===98?'#693a4c':'#334155';ctx.fillRect(col*sx,(m.height-1-row)*sy,sx+.5,sy+.5)}
const point=p=>[(p[0]-m.origin[0])/m.resolution*sx,480-(p[1]-m.origin[1])/m.resolution*sy];
if(selectedPoint){const p=point(selectedPoint);ctx.strokeStyle='#fff';ctx.lineWidth=3;ctx.beginPath();ctx.arc(...p,9,0,Math.PI*2);ctx.stroke();}
if(d.path.length){ctx.beginPath();ctx.strokeStyle='#c084fc';ctx.lineWidth=3;d.path.forEach((p,i)=>{const xy=point(p);if(i)ctx.lineTo(...xy);else ctx.moveTo(...xy)});ctx.stroke()}
if(d.target){const p=point(d.target);ctx.beginPath();ctx.fillStyle='#4ade80';ctx.arc(...p,6,0,Math.PI*2);ctx.fill()}
if(d.approach){const p=point(d.approach);ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.strokeRect(p[0]-5,p[1]-5,10,10)}
if(d.robot){const p=point(d.robot);ctx.save();ctx.translate(...p);ctx.rotate(-d.robot[2]);ctx.fillStyle='#fb923c';const w=.324/m.resolution*sx,h=.260/m.resolution*sy;ctx.fillRect(-w/2,-h/2,w,h);ctx.strokeStyle='#fff';ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(w/2+8,0);ctx.stroke();ctx.restore()}
document.getElementById('mapInfo').textContent='Ô '+Math.round(m.resolution*100)+' cm · '+(m.width*m.resolution).toFixed(2)+' × '+(m.height*m.resolution).toFixed(2)+' m · cập nhật '+d.age_s+' giây trước'+(d.age_s>2?' — dữ liệu cũ':'');
}catch(e){document.getElementById('mapInfo').textContent=e.message}}
refreshMap();setInterval(refreshMap,500);
</script></html>"""

class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass
    def send_data(self, code, content_type, data):
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)
    def do_GET(self):
        path = urlparse(self.path).path
        if path == '/':
            self.send_data(200, 'text/html; charset=utf-8', PAGE.encode('utf-8'))
        elif path == '/status':
            self.send_data(200, 'application/json', json.dumps(self.server.seeker.status(), allow_nan=False).encode())
        elif path == '/navigation_status.js':
            self.send_data(200, 'application/javascript; charset=utf-8',
                           Path(__file__).with_name('navigation_status.js').read_bytes())
        elif path == '/map':
            self.send_data(200, 'application/json', json.dumps(self.server.seeker.map_status(), allow_nan=False).encode())
        elif path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while not rospy.is_shutdown():
                    with self.server.seeker.state_lock:
                        jpeg = self.server.seeker.jpeg
                    if jpeg:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n')
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404)
    def do_POST(self):
        if self.path not in ('/target', '/navigation', '/map_goal'):
            self.send_error(404)
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4096:
                raise ValueError('Invalid request size')
            data = json.loads(self.rfile.read(length).decode('utf-8'))
            if not isinstance(data, dict):
                raise ValueError('Expected JSON object')
            if self.path == '/map_goal':
                self.server.seeker.set_map_goal(data)
            elif self.path == '/navigation':
                if type(data.get('enabled')) is not bool:
                    raise ValueError('enabled must be a boolean')
                self.server.seeker.set_navigation_enabled(data['enabled'])
            else:
                self.server.seeker.reset_target(data['label'])
        except (ValueError, KeyError, TypeError) as exc:
            self.send_data(400, 'text/plain; charset=utf-8', str(exc).encode())
            return
        except Exception as exc:
            self.send_data(503, 'text/plain; charset=utf-8', str(exc).encode())
            return
        self.send_data(200, 'application/json', b'{"ok":true}')

def main():
    global TARGET_CLASS
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target', nargs='?', default='person')
    parser.add_argument('--models-dir', default=str(Path(__file__).resolve().parent.parent / 'models'))
    parser.add_argument('--port', type=int, default=5010)
    args = parser.parse_args(rospy.myargv()[1:])
    TARGET_CLASS = args.target.strip().lower()
    classes = (Path(args.models_dir) / 'coco.names').read_text().splitlines()
    if TARGET_CLASS not in classes:
        parser.error('Target must be a class in coco.names')
    seeker = ObjectSeeker(args.models_dir)
    server = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    server.seeker = seeker
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    rospy.loginfo('Vision test: http://<robot-ip>:%s ? no motion output', args.port)
    try:
        rospy.spin()
    finally:
        server.shutdown()
        server.server_close()

if __name__ == '__main__':
    main()

