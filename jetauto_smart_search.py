#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
JetAuto Pro - Smart Autonomous Search & Real-time 2D Grid Mapping (LiDAR + Dual Vision)
- Tệp mới độc lập: Tích hợp đầy đủ tính năng của jetauto_find_object.py cộng thêm:
  1. Vẽ bản đồ lưới 2D thời gian thực (Real-time Occupancy Grid Map) từ LiDAR G4 (/scan) + Odometry (/odom).
  2. Bộ nhớ không gian vị trí mục tiêu (Object Permanence): Ghi nhớ tọa độ thực tế (X, Y) khi mục tiêu bị khuất.
  3. Logic Tìm Kiếm Đón Đầu (Intercept Search): Tự động lái xe vòng qua vật cản tới vị trí thấy lần cuối thay vì xoay tại chỗ.
  4. Khám phá mở rộng (Frontier Exploration): Tự động tuần tra các vùng tối chưa quét trên bản đồ khi mất dấu.
  5. Web Dashboard: Xem song song 2 Camera và Bản đồ 2D trực quan.
"""

from __future__ import print_function
import sys
import os
import time
import math
import signal
import threading
from robot_voice import RobotVoice, TargetAnnouncement
from collections import deque
from navigation_safety import (load_config, PoseHistory, scan_geometry,
                               ScanRecovery, FootprintSafety, DepthClearance, discover_footprint, same_planar_frame)
import numpy as np
import cv2

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CameraInfo
from std_msgs.msg import UInt16

try:
    from sensor_msgs.msg import Image as RosImage
except ImportError:
    RosImage = None

try:
    import depthai as dai
except ImportError:
    dai = None

if sys.version_info[0] == 2:
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
    from SocketServer import ThreadingMixIn
else:
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn

# ==========================================================
# CẤU HÌNH HỆ THỐNG
# ==========================================================
TARGET_CLASS = 'person'
if len(sys.argv) > 1:
    TARGET_CLASS = sys.argv[1].strip().lower()

# Never move autonomously after startup.  The operator must explicitly resume
# AI from the dashboard after checking the surroundings.
IS_PAUSED = True
CMD_VEL_TOPICS = [
    '/jetauto_controller/cmd_vel',
    '/cmd_vel',
    '/controller/cmd_vel',
    '/hiwonder_controller/cmd_vel'
]
NAV_CONFIG = load_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'navigation_config.json'))
FOOTPRINT = FootprintSafety(NAV_CONFIG)
ROBOT_CLEARANCE_M = FOOTPRINT.radius + FOOTPRINT.margin
AI_COMMAND_TIMEOUT_S = 2.0  # CPU inference can take about one second per frame.
PERSON_STOP_CM = 80.0
TARGET_REACQUIRE_S = 1.5
LIDAR_YAW_OFFSET_RAD = math.radians(NAV_CONFIG['lidar_yaw_deg'])
# The low-mounted OAK-D sees the floor as a nearby obstacle on this robot.
# Keep its RGB feed, but use LiDAR alone for autonomous collision decisions.
USE_OAKD_FOR_AVOIDANCE = False

# Khung hình toàn cục
raw_frame_astra = None
raw_depth_astra = None  # Astra depth image, millimetres (16UC1/mono16)
raw_frame_oakd = None
depth_frame_oakd = None
display_frame_astra = None
display_frame_oakd = None
display_frame_map = None  # Khung hình bản đồ 2D thời gian thực
frame_lock = threading.Lock()
astra_frame_time = 0.0
astra_frame_seq = 0
astra_depth_time = 0.0
astra_depth_registered = False

# Bộ đệm JPEG cache trên RAM (chống nghẽn socket)
cached_jpeg_astra = None
cached_jpeg_oakd = None
cached_jpeg_combined = None
cached_jpeg_map = None
jpeg_lock = threading.Lock()
jpeg_versions = {'astra': 0, 'oakd': 0, 'map': 0}

# Thông số vật cản OAK-D (cm)
obstacle_info = {
    'dist_l': 999.0,
    'dist_c': 999.0,
    'dist_r': 999.0,
    'status': 'CHI QUAN SAT' if not USE_OAKD_FOR_AVOIDANCE else 'AN TOAN',
    'strafe_dir': 'NONE',
    'last_update': 0.0
}
obstacle_lock = threading.Lock()

# LiDAR is the primary 360-degree collision sensor.  OAK-D remains a
# close-range, forward-facing fallback only.
lidar_info = {
    'front': 999.0, 'left': 999.0, 'right': 999.0, 'rear': 999.0,
    'turn_clearance': 0.0, 'scan_span_deg': 0.0,
    'last_update': 0.0,
}
lidar_lock = threading.Lock()

# Tọa độ Robot từ Odometry (x, y mét, yaw rad)
robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'last_update': 0.0}
pose_lock = threading.Lock()
battery_state = {'voltage': None, 'percent': None, 'last_update': 0.0}
battery_lock = threading.Lock()

# Bộ nhớ vị trí mục tiêu (Object Permanence)
target_memory = {
    'seen': False,
    'x': None,
    'y': None,
    'last_seen_time': 0.0,
    'distance': 0.0
}
target_lock = threading.Lock()

seeker_instance = None
cam_source_astra = "Khoi tao..."
cam_source_oakd = "Khoi tao..."


# ==========================================================
# 1. BỘ VẼ BẢN ĐỒ LƯỚI 2D THỜI GIAN THỰC (OCCUPANCY GRID MAPPER)
# ==========================================================
class OccupancyGridMapper(object):
    """
    Tự động xây dựng bản đồ lưới 2D từ LiDAR G4 (/scan) và Odometry (/odom).
    Kích thước: 240x240 ô, độ phân giải 5cm/ô -> Phạm vi 12m x 12m.
    """
    def __init__(self, size=240, resolution=0.05):
        self.size = size
        self.res = resolution
        self.center = size // 2
        # Ma trận bản đồ: 127 = Chưa biết (Xám), 0 = Thoáng (Đen/Xanh sẫm), 255 = Vật cản (Trắng)
        self.grid = np.full((size, size), 127, dtype=np.uint8)
        self.map_lock = threading.Lock()
        self.last_update = 0.0
        self.evidence = np.zeros((size, size), dtype=np.float32)
        self.observed_at = np.zeros((size, size), dtype=np.float64)

    def world_to_grid(self, wx, wy):
        gx = int(self.center + wx / self.res)
        gy = int(self.center - wy / self.res)  # Trục Y đảo ngược cho ảnh
        return gx, gy

    def clear(self):
        with self.map_lock:
            self.grid.fill(127)
            self.evidence.fill(0)
            self.observed_at.fill(0)
            self.last_update = 0.0

    def update_world_scan(self, origins, ends, hits):
        now = time.monotonic()
        if now - self.last_update < 0.18:
            return
        with self.map_lock:
            self.last_update = now
            free_mask = np.zeros_like(self.grid)
            hit_mask = np.zeros_like(self.grid)
            for origin, end, hit in zip(origins[::2], ends[::2], hits[::2]):
                start_cell = self.world_to_grid(*origin)
                end_cell = self.world_to_grid(*end)
                cv2.line(free_mask, start_cell, end_cell, 1, 1)
                if hit and 0 <= end_cell[0] < self.size and 0 <= end_cell[1] < self.size:
                    hit_mask[end_cell[1], end_cell[0]] = 1
            free = (free_mask > 0) & (hit_mask == 0)
            occupied = hit_mask > 0
            self.evidence[free] -= 0.7
            self.evidence[occupied] += 1.5
            np.clip(self.evidence, -4.0, 4.0, out=self.evidence)
            self.observed_at[free | occupied] = now
            stale = now - self.observed_at > 30.0
            self.evidence[stale] = 0.0
            self.grid.fill(127)
            self.grid[self.evidence <= -0.6] = 20
            self.grid[self.evidence >= 1.0] = 255

    def plan_frontier(self, rx, ry, visited, preferred=None):
        """Find a reachable free cell near unknown space and a path to it."""
        with self.map_lock:
            grid = self.grid.copy()
        radius = max(1, int(math.ceil(ROBOT_CLEARANCE_M / self.res)))
        footprint = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                              (radius * 2 + 1, radius * 2 + 1))
        free = cv2.morphologyEx((grid < 60).astype(np.uint8), cv2.MORPH_CLOSE,
                                np.ones((3, 3), dtype=np.uint8))
        safe = cv2.erode(free, footprint) != 0
        occupied = cv2.dilate((grid > 200).astype(np.uint8), footprint) != 0
        safe &= ~occupied
        unknown = ((grid >= 60) & (grid <= 200)).astype(np.uint8)
        near_unknown = cv2.dilate(unknown, np.ones((17, 17), dtype=np.uint8)) != 0
        frontier = safe & near_unknown
        sx, sy = self.world_to_grid(rx, ry)
        if not (0 <= sx < self.size and 0 <= sy < self.size):
            return None
        if not safe[sy, sx]:
            # Allow a small odometry/grid rounding error, never a distant jump.
            options = [(x, y) for y in range(max(0, sy - 3), min(self.size, sy + 4))
                       for x in range(max(0, sx - 3), min(self.size, sx + 4))
                       if safe[y, x]]
            if not options:
                return None
            sx, sy = min(options, key=lambda p: (p[0] - sx) ** 2 + (p[1] - sy) ** 2)

        parents = np.full(self.size * self.size, -1, dtype=np.int32)
        start = sy * self.size + sx
        parents[start] = start
        queue = deque([start])
        preferred_cell = self.world_to_grid(*preferred) if preferred else None
        first_frontier = None
        chosen = None
        while queue:
            current = queue.popleft()
            cy, cx = divmod(current, self.size)
            if preferred_cell == (cx, cy):
                chosen = current
                break
            if first_frontier is None and frontier[cy, cx]:
                wx = (cx - self.center) * self.res
                wy = (self.center - cy) * self.res
                if (math.hypot(wx - rx, wy - ry) >= 0.75 and
                        all(math.hypot(wx - vx, wy - vy) >= 0.8 for vx, vy in visited)):
                    first_frontier = current
                    if preferred is None:
                        chosen = current
                        break
            for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                if (0 <= nx < self.size and 0 <= ny < self.size and safe[ny, nx]):
                    neighbor = ny * self.size + nx
                    if parents[neighbor] == -1:
                        parents[neighbor] = current
                        queue.append(neighbor)
        if chosen is None:
            chosen = first_frontier
        if chosen is None:
            return None
        path = []
        current = chosen
        while current != start:
            cy, cx = divmod(current, self.size)
            path.append(((cx - self.center) * self.res,
                         (self.center - cy) * self.res))
            current = int(parents[current])
        path.reverse()
        return path[-1], path

    def render_map_image(self, rx, ry, ryaw, target_info=None, search_state="",
                         search_goal=None):
        """Tạo ảnh đồ họa màu trực quan của Bản đồ 2D để gửi lên Web."""
        with self.map_lock:
            # Chuyển ma trận xám sang ảnh màu 3 kênh BGR
            color_map = np.zeros((self.size, self.size, 3), dtype=np.uint8)
            # Vùng chưa biết: Màu nền tối #0f172a
            color_map[:] = (24, 15, 11)
            # Vùng đã quét thoáng: Màu xám xanh #1e293b
            free_mask = (self.grid < 60)
            color_map[free_mask] = (59, 41, 30)
            # Vùng vật cản / Tường: Màu xanh cyan neon sáng #00e5ff
            obs_mask = (self.grid > 200)
            color_map[obs_mask] = (255, 229, 0)

        # 1. Phóng to ảnh bản đồ lên 480x480 để hiển thị sắc nét trên Web
        disp_map = cv2.resize(color_map, (480, 480), interpolation=cv2.INTER_NEAREST)
        scale = 480.0 / float(self.size)

        # 2. Vẽ vị trí Robot
        rgx, rgy = self.world_to_grid(rx, ry)
        disp_rx = int(rgx * scale)
        disp_ry = int(rgy * scale)

        if 0 <= disp_rx < 480 and 0 <= disp_ry < 480:
            # Vòng tròn thân xe
            cv2.circle(disp_map, (disp_rx, disp_ry), 8, (0, 255, 0), -1)
            cv2.circle(disp_map, (disp_rx, disp_ry), 10, (255, 255, 255), 1)
            # Mũi tên chỉ hướng đầu xe
            arrow_len = 18
            head_x = int(disp_rx + arrow_len * math.cos(ryaw))
            head_y = int(disp_ry - arrow_len * math.sin(ryaw))
            cv2.arrowedLine(disp_map, (disp_rx, disp_ry), (head_x, head_y), (0, 255, 255), 2, tipLength=0.35)

        # 3. Vẽ vị trí mục tiêu (nếu đã từng nhìn thấy)
        if target_info and target_info.get('x') is not None:
            tx, ty = target_info['x'], target_info['y']
            tgx, tgy = self.world_to_grid(tx, ty)
            disp_tx = int(tgx * scale)
            disp_ty = int(tgy * scale)

            if 0 <= disp_tx < 480 and 0 <= disp_ty < 480:
                is_live = target_info.get('seen', False)
                t_color = (0, 0, 255) if is_live else (0, 165, 255)
                # Điểm mục tiêu nhấp nháy / beacon
                cv2.circle(disp_map, (disp_tx, disp_ty), 9, t_color, -1)
                cv2.circle(disp_map, (disp_tx, disp_ty), 14, t_color, 2)
                # Nối đường ngắm từ xe đến mục tiêu
                cv2.line(disp_map, (disp_rx, disp_ry), (disp_tx, disp_ty), (100, 100, 255), 1, cv2.LINE_AA)
                label = "TARGET (LIVE)" if is_live else "LAST SEEN"
                cv2.putText(disp_map, label, (disp_tx + 12, disp_ty + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, t_color, 1)

        if search_goal is not None:
            ggx, ggy = self.world_to_grid(*search_goal)
            if 0 <= ggx < self.size and 0 <= ggy < self.size:
                point = (int(ggx * scale), int(ggy * scale))
                cv2.circle(disp_map, point, 11, (255, 100, 255), 2)
                cv2.putText(disp_map, "SEARCH", (point[0] + 12, point[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 100, 255), 1)

        # 4. Vẽ bảng thông tin HUD trên bản đồ
        cv2.rectangle(disp_map, (0, 0), (480, 42), (15, 23, 42), -1)
        cv2.putText(disp_map, "2D LIDAR MAP | 12m x 12m", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 229, 255), 2)
        if search_state:
            cv2.putText(disp_map, search_state, (12, 465), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return disp_map


grid_mapper = OccupancyGridMapper()


class FrontierExplorer(object):
    """Scan for the object, then visit reachable boundaries of known space."""
    def __init__(self, mapper):
        self.mapper = mapper
        self.visited = []
        self.reset_scan()

    def reset_scan(self, clear_visited=False):
        if clear_visited:
            self.visited = []
        self.scan_recovery = ScanRecovery()
        self.phase = 'SCAN'
        self.goal = None
        self.path = []
        self.scan_started = 0.0
        self.previous_yaw = None
        self.swept_angle = 0.0
        self.last_plan = 0.0
        self.last_progress = 0.0
        self.best_remaining = float('inf')

    def step(self, rx, ry, yaw, now):
        if self.phase in ('SCAN', 'RECOVER'):
            phase, turn = self.scan_recovery.step(yaw, now)
            self.swept_angle = abs(self.scan_recovery.net_angle)
            self.scan_started = self.scan_recovery.started
            if phase in ('PLAN', 'PLAN_PARTIAL'):
                self.phase = 'PLAN'
            else:
                self.phase = phase
                return 0.0, turn, ('CHO THU HUONG QUET KHAC' if phase == 'RECOVER'
                                   else 'QUET TIM MUC TIEU')

        if self.phase == 'PLAN' or (self.phase == 'WAIT' and now - self.last_plan >= 5.0):
            self.last_plan = now
            result = self.mapper.plan_frontier(rx, ry, self.visited)
            if result is None:
                self.phase = 'WAIT'
                self.goal = None
                return 0.0, 0.0, 'KHONG CO VUNG CHUA QUET CO DUONG AN TOAN'
            self.goal, self.path = result
            self.phase = 'MOVE'
            self.last_progress = now
            self.best_remaining = math.hypot(self.goal[0] - rx, self.goal[1] - ry)

        if self.phase == 'WAIT':
            return 0.0, 0.0, 'DA QUET HET VUNG CO THE TIEP CAN'

        remaining = math.hypot(self.goal[0] - rx, self.goal[1] - ry)
        if remaining < 0.35:
            self.visited.append(self.goal)
            self.reset_scan()
            return 0.0, 0.0, 'TOI DIEM TIM KIEM, QUET LAI'
        if remaining < self.best_remaining - 0.08:
            self.best_remaining = remaining
            self.last_progress = now
        elif now - self.last_progress > 8.0:
            self.visited.append(self.goal)
            self.phase = 'PLAN'
            self.goal = None
            return 0.0, 0.0, 'DUONG BI CHAN, CHON HUONG KHAC'

        if now - self.last_plan > 2.0:
            self.last_plan = now
            result = self.mapper.plan_frontier(rx, ry, self.visited, self.goal)
            if result is None or result[0] != self.goal:
                self.visited.append(self.goal)
                self.phase = 'PLAN'
                self.goal = None
                return 0.0, 0.0, 'DUONG THAY DOI, LAP KE HOACH LAI'
            self.path = result[1]

        if not self.path:
            self.phase = 'PLAN'
            return 0.0, 0.0, 'DANG LAP KE HOACH'
        nearest = min(range(len(self.path)),
                      key=lambda i: math.hypot(self.path[i][0] - rx, self.path[i][1] - ry))
        waypoint = self.path[-1]
        for point in self.path[nearest:]:
            if math.hypot(point[0] - rx, point[1] - ry) >= 0.18:
                waypoint = point
                break
        heading = math.atan2(waypoint[1] - ry, waypoint[0] - rx)
        heading_error = (heading - yaw + math.pi) % (2 * math.pi) - math.pi
        turn = max(-22.0, min(22.0, math.degrees(heading_error) * 0.8))
        speed = 0.10 if abs(heading_error) < math.radians(25) else 0.0
        return speed, turn, 'TIM VUNG CHUA QUET ({:.1f}m)'.format(remaining)


# ==========================================================
# 2. XỬ LÝ KHUNG HÌNH & BỘ ĐỆM JPEG CACHE
# ==========================================================
def get_frame_for_cam(cam_name='astra'):
    """Lấy khung hình cho từng camera cụ thể hoặc bản đồ 2D."""
    if cam_name == 'combined':
        return get_combined_frame()
    if cam_name == 'map':
        with frame_lock:
            if display_frame_map is not None:
                return display_frame_map.copy()
        f = np.zeros((480, 480, 3), dtype=np.uint8)
        cv2.putText(f, "DANG KHOI TAO BAN DO...", (40, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 229, 255), 2)
        return f

    with frame_lock:
        if cam_name == 'astra':
            if display_frame_astra is not None:
                return display_frame_astra.copy()
            if raw_frame_astra is not None:
                return raw_frame_astra.copy()
            title = "ASTRA PRO PLUS (AI TRACKER)"
            status = cam_source_astra
        else:
            if display_frame_oakd is not None:
                return display_frame_oakd.copy()
            if raw_frame_oakd is not None:
                return raw_frame_oakd.copy()
            title = "LUXONIS OAK-D (RADAR NE VAT CAN)"
            status = cam_source_oakd

    f = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(f, (0, 0), (640, 480), (15, 23, 42), -1)
    cv2.putText(f, title, (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 229, 255), 2)
    cv2.putText(f, "DANG CHO TIN HIEU CAMERA...", (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
    cv2.putText(f, status, (30, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (148, 163, 184), 1)
    return f


def get_combined_frame():
    """Ghép 2 camera thành 1 khung hình Side-by-Side."""
    f1 = get_frame_for_cam('astra')
    f2 = get_frame_for_cam('oakd')
    f1_s = cv2.resize(f1, (480, 360))
    f2_s = cv2.resize(f2, (480, 360))
    combined = np.hstack([f1_s, f2_s])
    cv2.line(combined, (480, 0), (480, 360), (0, 229, 255), 2)
    return combined


def jpeg_encoder_loop():
    """Nén trước toàn bộ các khung hình lên RAM ở tốc độ ~22 FPS."""
    global cached_jpeg_astra, cached_jpeg_oakd, cached_jpeg_combined, cached_jpeg_map
    last_frames = {'astra': None, 'oakd': None, 'map': None}
    while not rospy.is_shutdown():
        try:
            # 1. Astra
            f_astra = None
            with frame_lock:
                if display_frame_astra is not None:
                    f_astra = display_frame_astra
                elif raw_frame_astra is not None:
                    f_astra = raw_frame_astra

            if f_astra is not None and f_astra is not last_frames['astra']:
                # A 480x360 web feed is visually smooth while taking much
                # less CPU than repeatedly encoding the full Astra frame.
                web_astra = cv2.resize(f_astra, (480, 360))
                ret, jpg = cv2.imencode('.jpg', web_astra, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                if ret:
                    with jpeg_lock:
                        cached_jpeg_astra = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()
                        jpeg_versions['astra'] += 1
                    last_frames['astra'] = f_astra

            # 2. OAK-D
            f_oakd = None
            with frame_lock:
                if display_frame_oakd is not None:
                    f_oakd = display_frame_oakd
                elif raw_frame_oakd is not None:
                    f_oakd = raw_frame_oakd

            if f_oakd is not None and f_oakd is not last_frames['oakd']:
                ret, jpg = cv2.imencode('.jpg', f_oakd, [int(cv2.IMWRITE_JPEG_QUALITY), 55])
                if ret:
                    with jpeg_lock:
                        cached_jpeg_oakd = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()
                        jpeg_versions['oakd'] += 1
                    last_frames['oakd'] = f_oakd

            # 3. Map 2D.  The dashboard does not display the combined feed,
            # so do not spend CPU encoding it on every cycle.
            f_map = None
            with frame_lock:
                if display_frame_map is not None:
                    f_map = display_frame_map

            if f_map is not None and f_map is not last_frames['map']:
                ret, jpg = cv2.imencode('.jpg', f_map, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                if ret:
                    with jpeg_lock:
                        cached_jpeg_map = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()
                        jpeg_versions['map'] += 1
                    last_frames['map'] = f_map

        except Exception:
            pass

        time.sleep(0.050)


# ==========================================================
# 3. XỬ LÝ VẬT CẢN OAK-D STEREO DEPTH
# ==========================================================
def active_oakd_distances(info, now):
    """Use OAK-D for motion only when explicitly enabled and fresh."""
    if not USE_OAKD_FOR_AVOIDANCE or now - info['last_update'] >= 1.0:
        return 999.0, 999.0, 999.0
    return info['dist_l'], info['dist_c'], info['dist_r']


def process_obstacle_depth(depth_frame):
    if depth_frame is None or depth_frame.size == 0:
        return 999.0, 999.0, 999.0

    h, w = depth_frame.shape[:2]
    roi = depth_frame[int(h * 0.35):int(h * 0.90), :]
    col_w = w // 3
    zone_l = roi[:, :col_w]
    zone_c = roi[:, col_w:col_w * 2]
    zone_r = roi[:, col_w * 2:]

    def calc_dist(z):
        valid = z[(z > 150) & (z < 3000)]
        if len(valid) < 60:
            return 999.0
        return float(np.percentile(valid, 10)) / 10.0

    return calc_dist(zone_l), calc_dist(zone_c), calc_dist(zone_r)


def estimate_astra_distance_cm(depth_frame, rgb_shape, box):
    """Median Astra depth at the YOLO box centre; returns None if unavailable."""
    if depth_frame is None or depth_frame.size == 0 or rgb_shape is None:
        return None
    rgb_h, rgb_w = rgb_shape[:2]
    dep_h, dep_w = depth_frame.shape[:2]
    bx, by, bw, bh = box
    cx = int((bx + bw * 0.5) * dep_w / float(rgb_w))
    cy = int((by + bh * 0.5) * dep_h / float(rgb_h))
    rx = max(4, int(max(bw * 0.12, 12) * dep_w / float(rgb_w)))
    ry = max(4, int(max(bh * 0.12, 12) * dep_h / float(rgb_h)))
    roi = depth_frame[max(0, cy - ry):min(dep_h, cy + ry + 1),
                      max(0, cx - rx):min(dep_w, cx + rx + 1)]
    valid = roi[(roi > 150) & (roi < 8000)]
    if valid.size < 12:
        return None
    return float(np.median(valid)) / 10.0


def render_oakd_hud(frame, dl, dc, dr, strafe_dir="NONE"):
    h, w = frame.shape[:2]
    out = frame.copy()
    if not USE_OAKD_FOR_AVOIDANCE:
        cv2.rectangle(out, (0, 0), (w, 42), (15, 23, 42), -1)
        cv2.putText(out, "OAK-D CAMERA | LIDAR NAV", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 229, 255), 2)
        return out
    col_w = w // 3
    y_start = int(h * 0.42)
    y_end = int(h * 0.94)

    def get_color(dist):
        if dist < 30.0:
            return (0, 0, 255)
        elif dist < 60.0:
            return (0, 215, 255)
        else:
            return (0, 255, 0)

    zones = [(0, col_w, dl, "TRAI"), (col_w, col_w * 2, dc, "GIUA"), (col_w * 2, w, dr, "PHAI")]
    for x1, x2, dist, name in zones:
        c = get_color(dist)
        cv2.rectangle(out, (x1 + 4, y_start), (x2 - 4, y_end), c, 2)
        txt = "{}: {:.0f}cm".format(name, dist) if dist < 800 else "{}: THOANG".format(name)
        cv2.putText(out, txt, (x1 + 10, y_end - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2)

    cv2.rectangle(out, (0, 0), (w, 42), (15, 23, 42), -1)
    status_text = "OAK-D RADAR"
    if strafe_dir == "LEFT":
        status_text += " | <<< TRUOT TRAI NE"
        banner_c = (0, 215, 255)
    elif strafe_dir == "RIGHT":
        status_text += " | TRUOT PHAI NE >>>"
        banner_c = (0, 215, 255)
    elif dc < 30.0:
        status_text += " | PHANH KHAN CAP (<30cm)!"
        banner_c = (0, 0, 255)
    else:
        status_text += " | DUONG THOANG"
        banner_c = (0, 255, 0)

    cv2.putText(out, status_text, (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, banner_c, 2)
    return out


# ==========================================================
# 4. WEB SERVER & GIAO DIỆN CYBER DASHBOARD (KÈM BẢN ĐỒ 2D)
# ==========================================================
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class CamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def _stream_mjpeg(self, cam_name='astra'):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        try:
            last_version = -1
            while True:
                raw = None
                with jpeg_lock:
                    if cam_name == 'astra':
                        raw = cached_jpeg_astra
                    elif cam_name == 'oakd':
                        raw = cached_jpeg_oakd
                    elif cam_name == 'combined':
                        raw = cached_jpeg_combined
                    elif cam_name == 'map':
                        raw = cached_jpeg_map
                    version = jpeg_versions.get(cam_name, 0)

                if raw is not None and version == last_version:
                    time.sleep(0.025)
                    continue

                if raw is None:
                    frame = get_frame_for_cam(cam_name)
                    ret, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                    if ret:
                        raw = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()

                if raw is not None:
                    packet = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + raw + b'\r\n'
                    self.wfile.write(packet)
                    self.wfile.flush()
                    last_version = version
                time.sleep(0.025)
        except Exception:
            pass

    def do_POST(self):
        global TARGET_CLASS, IS_PAUSED, seeker_instance
        content_length = int(self.headers.get('Content-Length', 0))
        post_data = self.rfile.read(content_length) if content_length > 0 else b'{}'
        import json
        try:
            data = json.loads(post_data.decode('utf-8'))
        except Exception:
            data = {}

        if self.path == '/set_target':
            new_target = data.get('target', '').strip().lower()
            if new_target:
                TARGET_CLASS = new_target
                with target_lock:
                    target_memory.update(seen=False, x=None, y=None,
                                         last_seen_time=0.0, distance=0.0)
                if seeker_instance:
                    seeker_instance.set_desired_velocity(0.0, 0.0, 0.0)
                    seeker_instance.explorer.reset_scan(clear_visited=True)
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status": "ok"}')
                return

        elif self.path in ('/pause', '/stop'):
            IS_PAUSED = True
            if seeker_instance:
                seeker_instance.set_desired_velocity(0.0, 0.0, 0.0)
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"paused": true}')
            return

        elif self.path == '/resume':
            if IS_PAUSED and seeker_instance:
                seeker_instance.set_desired_velocity(0.0, 0.0, 0.0)
                seeker_instance.explorer.reset_scan()
            IS_PAUSED = False
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"paused": false}')
            return

        elif self.path == '/toggle_pause':
            IS_PAUSED = not IS_PAUSED
            if IS_PAUSED and seeker_instance:
                seeker_instance.set_desired_velocity(0.0, 0.0, 0.0)
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'paused': IS_PAUSED}).encode())
            return

        elif self.path == '/manual_move':
            direction = data.get('direction', 'stop').strip().lower()
            if seeker_instance:
                seeker_instance.manual_move(direction)
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'ok', 'direction': direction}).encode())
                return

        self.send_response(400)
        self.end_headers()

    def do_GET(self):
        import json
        if self.path.startswith('/map_feed'):
            self._stream_mjpeg('map')
        elif self.path.startswith('/cam_combined'):
            self._stream_mjpeg('combined')
        elif self.path.startswith('/cam_oakd'):
            self._stream_mjpeg('oakd')
        elif self.path.startswith('/cam_astra') or self.path.startswith('/cam.mjpg'):
            self._stream_mjpeg('astra')
        elif self.path.startswith('/status'):
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            with obstacle_lock:
                obs_copy = dict(obstacle_info)
            with pose_lock:
                p_copy = dict(robot_pose)
            with battery_lock:
                b_copy = dict(battery_state)
            if b_copy['last_update'] and time.time() - b_copy['last_update'] > 5.0:
                b_copy.update(voltage=None, percent=None)
            with target_lock:
                t_copy = dict(target_memory)
            with lidar_lock:
                scan_copy = dict(lidar_info)
            seeker = seeker_instance
            explorer = getattr(seeker, 'explorer', None) if seeker else None
            explore_goal = getattr(explorer, 'goal', None)
            ai_thread = getattr(seeker, 'ai_thread', None) if seeker else None

            status_data = {
                'target': TARGET_CLASS,
                'paused': IS_PAUSED,
                'camera_age': round(time.time() - astra_frame_time, 1) if astra_frame_time else None,
                'lidar_age': round(time.time() - lidar_info['last_update'], 1) if lidar_info['last_update'] else None,
                'lidar_scan_span_deg': round(lidar_info['scan_span_deg'], 0),
                'lidar_turn_clearance_cm': round(lidar_info['turn_clearance'], 0),
                'odom_age': round(time.time() - p_copy['last_update'], 1) if p_copy['last_update'] else None,
                'search_goal_x': round(explore_goal[0], 2) if explore_goal else None,
                'search_goal_y': round(explore_goal[1], 2) if explore_goal else None,
                'mapping_status': getattr(seeker, 'mapping_status', 'INIT'),
                'mapping_wait_reason': getattr(seeker, 'mapping_wait_reason', ''),
                'map_age': round(time.monotonic() - grid_mapper.last_update, 2) if grid_mapper.last_update else None,
                'footprint_radius_cm': round((FOOTPRINT.radius + FOOTPRINT.margin) * 100, 1),
                'body_length_cm': NAV_CONFIG['body_length_m'] * 100 if NAV_CONFIG['body_length_m'] else None,
                'body_width_cm': NAV_CONFIG['body_width_m'] * 100 if NAV_CONFIG['body_width_m'] else None,
                'footprint_calibrated': FOOTPRINT.calibrated,
                'footprint_source': getattr(seeker, 'footprint_source', 'UNCONFIGURED'),
                'depth_clearance': seeker.depth_guard.status,
                'depth_clearance_enabled': seeker.depth_guard.enabled,
                'explorer_phase': getattr(explorer, 'phase', 'INIT'),
                'scan_swept_deg': round(math.degrees(getattr(explorer, 'swept_angle', 0.0)), 1),
                'lidar_front_cm': round(scan_copy['front'], 1),
                'lidar_left_cm': round(scan_copy['left'], 1),
                'lidar_right_cm': round(scan_copy['right'], 1),
                'lidar_rear_cm': round(scan_copy['rear'], 1),
                'astra_depth_age': round(time.time() - astra_depth_time, 2) if astra_depth_time else None,
                'astra_depth_registered': astra_depth_registered,
                'ai_alive': bool(ai_thread and ai_thread.is_alive()),
                'ai_fps': round(seeker_instance.ai_fps, 1) if seeker_instance else 0.0,
                'camera_fps': round(seeker_instance.camera_fps, 1) if seeker_instance else 0.0,
                'inference_ms': round(seeker_instance.inference_ms, 0) if seeker_instance else 0.0,
                'action_text': seeker_instance.last_action_text if seeker_instance else "CHO KHOI DONG",
                'search_mode': seeker_instance.search_mode if seeker_instance else "INIT",
                'linear_x': round(seeker_instance.published_linear_x, 2) if seeker_instance else 0.0,
                'linear_y': round(seeker_instance.published_linear_y, 2) if seeker_instance else 0.0,
                'angular_z': round(seeker_instance.published_angular_z, 1) if seeker_instance else 0.0,
                'motion_block': seeker_instance.motion_block_reason if seeker_instance else '',
                'dist_left': round(obs_copy['dist_l'], 1),
                'dist_center': round(obs_copy['dist_c'], 1),
                'dist_right': round(obs_copy['dist_r'], 1),
                'strafe_dir': obs_copy['strafe_dir'],
                'obs_status': obs_copy['status'],
                'oakd_avoidance_enabled': USE_OAKD_FOR_AVOIDANCE,
                'pose_x': round(p_copy['x'], 2),
                'pose_y': round(p_copy['y'], 2),
                'pose_yaw': round(math.degrees(p_copy['yaw']), 1),
                'battery_voltage': b_copy['voltage'],
                'battery_percent': b_copy['percent'],
                'battery_age': round(time.time() - b_copy['last_update'], 1) if b_copy['last_update'] else None,
                'target_x': round(t_copy['x'], 2) if t_copy['x'] is not None else None,
                'target_y': round(t_copy['y'], 2) if t_copy['y'] is not None else None,
                'target_seen': t_copy['seen']
            }
            status_data.update({
                'announcement': seeker.target_announcement.message,
                'greeting_event_id': getattr(seeker, 'greeting_event_id', 0),
                'voice_error': getattr(getattr(seeker, 'robot_voice', None), 'error', ''),
            })
            self.wfile.write(json.dumps(status_data).encode())
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(self._render_html().encode('utf-8'))

    def _render_html(self):
        from html import escape
        template_path = os.path.join(os.path.dirname(__file__), 'templates', 'smart_dashboard.html')
        with open(template_path, 'r') as template_file:
            return template_file.read().replace('__TARGET__', escape(TARGET_CLASS.upper()))



def start_web_server():
    port = 5005
    server = None
    while port <= 5100:
        try:
            server = ThreadedHTTPServer(('0.0.0.0', port), CamHandler)
            break
        except Exception:
            port += 1

    if server:
        print("\n=======================================================")
        print(">> [SMART SEARCH WEB SERVER] Da khoi dong! Truy cap:")
        print(">> http://192.168.149.1:" + str(port))
        print("=======================================================\n")
        server.serve_forever()


class SmartObjectSeeker(object):
    def __init__(self):
        global seeker_instance, NAV_CONFIG, FOOTPRINT, ROBOT_CLEARANCE_M
        seeker_instance = self

        rospy.init_node('jetauto_smart_seeker', anonymous=True)
        NAV_CONFIG, self.footprint_source = discover_footprint(NAV_CONFIG, rospy.get_param)
        FOOTPRINT = FootprintSafety(NAV_CONFIG)
        ROBOT_CLEARANCE_M = FOOTPRINT.radius + FOOTPRINT.margin
        self.footprint_frame_valid = not bool(NAV_CONFIG.get('footprint_frame'))
        description = rospy.get_param('/robot_description', '')
        self.accepted_footprint_frames = set(frame for frame in ('base_link', 'base_footprint')
                                             if same_planar_frame(NAV_CONFIG.get('footprint_frame', ''), frame, description))
        rospy.loginfo('Navigation body: length=%s width=%s source=%s',
                      NAV_CONFIG['body_length_m'], NAV_CONFIG['body_width_m'], self.footprint_source)

        self.cmd_publishers = []
        for top in CMD_VEL_TOPICS:
            self.cmd_publishers.append(rospy.Publisher(top, Twist, queue_size=1))

        self.motion_lock = threading.Lock()
        self.desired_linear_x = 0.0
        self.desired_linear_y = 0.0
        self.desired_angular_z = 0.0
        self.published_linear_x = 0.0
        self.published_linear_y = 0.0
        self.published_angular_z = 0.0
        self.motion_block_reason = ''
        self.manual_override_until = 0.0
        self.explorer = FrontierExplorer(grid_mapper)
        self.last_ai_update = 0.0
        self.last_motion_update = 0.0
        self._last_map_render = 0.0
        self._astra_registered_depth_time = 0.0
        self.pose_history = PoseHistory()
        self.pending_scans = deque(maxlen=6)
        self.scan_processing_lock = threading.Lock()
        self.mapping_status = 'WAITING FOR SYNC'
        self.mapping_wait_reason = ''
        self.scan_stamp = 0.0
        self.depth_guard = DepthClearance(NAV_CONFIG, FOOTPRINT)
        self.depth_camera_info = None
        self.depth_info_lock = threading.Lock()
        # Retain all Subscriber objects.  Otherwise some rospy versions may
        # unregister them before an image/scan callback is received.
        self.astra_subscribers = []
        self.oakd_subscribers = []
        self.navigation_subscribers = []

        # Chế độ tìm kiếm: 'TRACKING' (Bám đuổi), 'INTERCEPT' (Đón đầu), 'EXPLORE' (Khám phá)
        self.search_mode = "EXPLORE"

        # YOLO Model
        self.classes = self.load_classes()
        model_path = os.path.join(os.path.dirname(__file__), 'yolov5n.onnx')
        if not os.path.exists(model_path):
            rospy.logerr(">> Khong tim thay file model: " + model_path)
            sys.exit(1)

        try:
            import onnxruntime as ort
            sess_opts = ort.SessionOptions()
            sess_opts.intra_op_num_threads = 4
            avail = ort.get_available_providers()
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if 'CUDAExecutionProvider' in avail else ['CPUExecutionProvider']
            self.session = ort.InferenceSession(model_path, sess_options=sess_opts, providers=providers)
            self.input_name = self.session.get_inputs()[0].name
            self.use_ort = True
        except Exception:
            self.net = cv2.dnn.readNetFromONNX(model_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.use_ort = False

        self.state_lock = threading.Lock()
        self.last_action_text = "DANG KHOI TAO..."
        self.last_status_color = (0, 165, 255)
        self.latest_detections = []
        self.latest_target_center = None
        self.ai_fps = 0.0
        self.camera_fps = 0.0
        self.inference_ms = 0.0
        self._camera_frame_count = 0
        self._camera_fps_start = time.time()
        self.greeting_event_id = 0
        self.robot_voice = RobotVoice(rospy.logwarn)
        self.target_announcement = TargetAnnouncement(self.robot_voice)

        rospy.on_shutdown(self.stop)

        # 1. Thread điều khiển động cơ 10Hz
        self.motion_thread = threading.Thread(target=self.motion_heartbeat_loop)
        self.motion_thread.daemon = True
        self.motion_thread.start()

        # 2. Camera Astra ROS Subscriber
        self.astra_thread = threading.Thread(target=self.init_astra_capture)
        self.astra_thread.daemon = True
        self.astra_thread.start()

        # 3. Camera OAK-D Stereo Depth Pipeline
        self.oakd_thread = threading.Thread(target=self.init_oakd_capture)
        self.oakd_thread.daemon = True
        self.oakd_thread.start()

        # 4. LiDAR & Odometry Subscribers cho Bản đồ 2D
        self.navigation_subscribers = [
            rospy.Subscriber('/scan', LaserScan, self._callback_scan, queue_size=1),
            rospy.Subscriber('/odom', Odometry, self._callback_odom, queue_size=1),
            rospy.Subscriber('/ros_robot_controller/battery', UInt16,
                             self._callback_battery, queue_size=1),
        ]

        # 5. Thread nén JPEG cache
        self.encoder_thread = threading.Thread(target=jpeg_encoder_loop)
        self.encoder_thread.daemon = True
        self.encoder_thread.start()

        # 6. Thread trung tâm ra quyết định
        self.ai_thread = threading.Thread(target=self.smart_search_loop)
        self.ai_thread.daemon = True
        self.ai_thread.start()

    def load_classes(self):
        path = os.path.join(os.path.dirname(__file__), 'coco.names')
        if os.path.exists(path):
            with open(path, 'r') as f:
                return [c.strip() for c in f.readlines() if c.strip()]
        return ['person']

    def set_desired_velocity(self, linear_x=0.0, linear_y=0.0, angular_z_deg=0.0):
        with self.motion_lock:
            self.desired_linear_x = float(linear_x)
            self.desired_linear_y = float(linear_y)
            self.desired_angular_z = float(angular_z_deg)
            self.last_motion_update = time.time()

    def manual_move(self, direction):
        if direction == 'forward':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.18, 0.0, 0.0)
        elif direction == 'backward':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(-0.18, 0.0, 0.0)
        elif direction == 'strafe_left':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.0, 0.18, 0.0)
        elif direction == 'strafe_right':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.0, -0.18, 0.0)
        elif direction == 'left':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.0, 0.0, 35.0)
        elif direction == 'right':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.0, 0.0, -35.0)
        else:
            self.manual_override_until = 0.0
            self.set_desired_velocity(0.0, 0.0, 0.0)

    def motion_heartbeat_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.motion_lock:
                vx = self.desired_linear_x
                vy = self.desired_linear_y
                wz = self.desired_angular_z
                paused = IS_PAUSED
                command_age = time.time() - self.last_motion_update
                manual_active = time.time() < self.manual_override_until
                manual_command = (self.manual_override_until > 0.0 and
                                  self.last_motion_update <= self.manual_override_until)
            block_reason = ''
            if not self.footprint_frame_valid:
                vx, vy, wz = 0.0, 0.0, 0.0
                block_reason = 'FOOTPRINT FRAME DOES NOT MATCH ODOM BASE'

            # A dead AI thread, stalled camera or released manual key must not
            # leave the chassis repeating its last nonzero velocity.
            if (paused or command_age > AI_COMMAND_TIMEOUT_S or
                    (manual_command and (command_age > 0.7 or not manual_active)) or
                    (manual_active and time.time() - self.last_ai_update > 1.5)):
                vx, vy, wz = 0.0, 0.0, 0.0
                block_reason = 'TẠM DỪNG' if paused else 'LỆNH ĐIỀU KHIỂN HẾT HẠN'
            if time.time() - astra_frame_time > 1.5 and not manual_active:
                vx, vy, wz = 0.0, 0.0, 0.0
                block_reason = 'MẤT CAMERA ASTRA'
            if time.time() - robot_pose['last_update'] > 1.5 and not manual_active:
                vx, vy, wz = 0.0, 0.0, 0.0
                block_reason = 'MẤT ODOMETRY'
            if (not manual_active and self.search_mode == 'EXPLORE'
                    and time.monotonic() - grid_mapper.last_update > 1.5):
                vx, vy, wz = 0.0, 0.0, 0.0
                block_reason = 'CHO BAN DO DONG BO'
            if time.time() - lidar_info['last_update'] > 1.5:
                vx, vy, wz = 0.0, 0.0, 0.0
                block_reason = 'MẤT LIDAR'
            else:
                allowed, reason = FOOTPRINT.check(vx, vy, wz, time.monotonic())
                if allowed and abs(vx) + abs(vy) + abs(wz) > 0:
                    allowed, reason = self.depth_guard.check(vx, vy, wz, time.monotonic())
                if not allowed:
                    vx, vy, wz = 0.0, 0.0, 0.0
                    block_reason = reason

            self.published_linear_x = vx
            self.published_linear_y = vy
            self.published_angular_z = wz
            self.motion_block_reason = block_reason

            twist_msg = Twist()
            twist_msg.linear.x = float(vx)
            twist_msg.linear.y = float(vy)
            twist_msg.angular.z = float(math.radians(wz))

            for pub in self.cmd_publishers:
                try:
                    pub.publish(twist_msg)
                except Exception:
                    pass

            rate.sleep()

    # ==========================================================
    # LIDAR & ODOMETRY CALLBACKS
    # ==========================================================
    def _callback_odom(self, msg):
        global robot_pose
        footprint_frame = NAV_CONFIG.get('footprint_frame')
        if footprint_frame:
            # A polygon in a different frame must not silently change the collision geometry.
            self.footprint_frame_valid = msg.child_frame_id.lstrip('/') in self.accepted_footprint_frames
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        generation = self.pose_history.generation
        stamp = msg.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if age > 0.5 or age < -0.05:
            return
        if not self.pose_history.add(stamp, px, py, yaw):
            return
        if self.pose_history.generation != generation:
            with self.scan_processing_lock:
                grid_mapper.clear()
                self.pending_scans.clear()
            self.scan_stamp = 0.0
            self.explorer.reset_scan(clear_visited=True)
            self.set_desired_velocity(0.0, 0.0, 0.0)
            with target_lock:
                target_memory.update(seen=False, x=None, y=None, last_seen_time=0.0)
        with pose_lock:
            robot_pose['x'] = px
            robot_pose['y'] = py
            robot_pose['yaw'] = yaw
            robot_pose['last_update'] = time.time()
        self._process_pending_scans()

    @staticmethod
    def _callback_battery(msg):
        # Hiwonder publishes this 3-cell pack voltage in millivolts (UInt16).
        millivolts = int(msg.data)
        if millivolts < 6000 or millivolts > 15000:
            return
        voltage = millivolts / 1000.0
        # Approximate display: vendor's 10.0 V warning to 12.6 V full charge.
        percent = int(round(max(0.0, min(100.0, (voltage - 10.0) / 2.6 * 100.0))))
        with battery_lock:
            battery_state.update(voltage=voltage, percent=percent, last_update=time.time())

    def _process_pending_scans(self):
        if not self.scan_processing_lock.acquire(False):
            return
        try:
            while self.pending_scans:
                msg, queued = self.pending_scans[0]
                stamps = msg.header.stamp.to_sec() + np.arange(len(msg.ranges)) * msg.time_increment
                poses = self.pose_history.interpolate(stamps)
                if poses is None:
                    self.mapping_wait_reason = self.pose_history.describe_wait(stamps)
                    if time.monotonic() - grid_mapper.last_update > 1.5:
                        self.mapping_status = 'WAITING FOR STAMPED ODOMETRY'
                    if time.monotonic() - queued > 0.5:
                        self.pending_scans.popleft()
                        continue
                    break
                self.pending_scans.popleft()
                self.mapping_wait_reason = ''
                origins, ends, hits = scan_geometry(msg.ranges, msg.angle_min, msg.angle_increment,
                                                  poses, NAV_CONFIG, max(0.12, msg.range_min), msg.range_max)
                grid_mapper.update_world_scan(origins, ends, hits)
                self.mapping_status = ('SYNCED' if NAV_CONFIG['lidar_extrinsics_calibrated']
                                       else 'SYNCED; LIDAR EXTRINSICS UNCALIBRATED')
        finally:
            self.scan_processing_lock.release()

    def _callback_scan(self, msg):
        global display_frame_map, lidar_info
        with pose_lock:
            rx = robot_pose['x']
            ry = robot_pose['y']
            ryaw = robot_pose['yaw']

        stamp = msg.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if (stamp <= self.scan_stamp or stamp <= 0 or age > 0.5 or age < -0.05
                or not math.isfinite(msg.time_increment) or msg.time_increment < 0
                or len(msg.ranges) < 10 or not math.isfinite(msg.angle_increment)
                or msg.angle_increment == 0):
            self.mapping_status = 'STALE OR OUT OF ORDER SCAN'
            return
        self.scan_stamp = stamp
        FOOTPRINT.update(msg.ranges, msg.angle_min, msg.angle_increment,
                         msg.range_min, msg.range_max, time.monotonic() - max(0, age))
        with self.scan_processing_lock:
            self.pending_scans.append((msg, time.monotonic()))
        self._process_pending_scans()

        # Split the 360° scan into safety sectors.  Use a low percentile
        # instead of one noisy beam, but retain genuinely close obstacles.
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        angles = (LIDAR_YAW_OFFSET_RAD + msg.angle_min +
                  np.arange(ranges.size) * msg.angle_increment)
        angles = np.arctan2(np.sin(angles), np.cos(angles))

        def sector_distance(center, half_width):
            delta = np.arctan2(np.sin(angles - center), np.cos(angles - center))
            sector = ranges[np.abs(delta) <= half_width]
            measured = sector[np.isfinite(sector) &
                              (sector >= max(0.12, msg.range_min)) &
                              (sector <= msg.range_max)]
            # An infinite return means no hit within the LiDAR range; NaN or
            # an empty sector means unknown and must not be treated as clear.
            clear_count = int(np.count_nonzero(np.isposinf(sector)))
            if measured.size + clear_count < 3:
                return 0.0
            clear_range = min(4.0, msg.range_max)
            distances = np.concatenate((np.minimum(measured, clear_range),
                                        np.full(clear_count, clear_range, dtype=np.float32)))
            nearest_cluster = measured[measured < 0.60]
            if nearest_cluster.size >= 2:
                return float(np.mean(np.partition(nearest_cluster, 1)[:2])) * 100.0
            return float(np.percentile(distances, 10)) * 100.0

        with lidar_lock:
            lidar_info['front'] = sector_distance(0.0, math.radians(28))
            lidar_info['left'] = sector_distance(math.pi / 2.0, math.radians(45))
            lidar_info['right'] = sector_distance(-math.pi / 2.0, math.radians(45))
            lidar_info['rear'] = sector_distance(math.pi, math.radians(28))
            valid = (np.isposinf(ranges) |
                     (np.isfinite(ranges) & (ranges >= max(0.12, msg.range_min)) &
                      (ranges <= msg.range_max)))
            close_returns = ranges[valid & np.isfinite(ranges)]
            lidar_info['scan_span_deg'] = math.degrees(abs(msg.angle_increment) * max(0, ranges.size - 1))
            if np.count_nonzero(valid) < 10 or lidar_info['scan_span_deg'] < 90.0:
                lidar_info['turn_clearance'] = 0.0
            elif close_returns.size:
                lidar_info['turn_clearance'] = float(np.min(close_returns)) * 100.0
            else:
                lidar_info['turn_clearance'] = min(4.0, msg.range_max) * 100.0
            lidar_info['last_update'] = time.time()

        # A map is useful at 2 FPS; camera and YOLO get the saved CPU time.
        now = time.time()
        if now - self._last_map_render >= 0.50:
            self._last_map_render = now
            with target_lock:
                t_copy = dict(target_memory)
            rendered_map = grid_mapper.render_map_image(
                rx, ry, ryaw, t_copy, self.search_mode, self.explorer.goal)
            with frame_lock:
                display_frame_map = rendered_map

    # ==========================================================
    # CAMERA CAPTURE (ASTRA & OAK-D)
    # ==========================================================
    def init_astra_capture(self):
        global cam_source_astra
        cam_source_astra = "Dang cho Astra..."
        if RosImage is not None:
            self.astra_subscribers = [
                rospy.Subscriber(NAV_CONFIG['depth_info_topic'], CameraInfo, self._cb_clearance_info, queue_size=1),
                rospy.Subscriber(NAV_CONFIG['depth_topic'], RosImage, self._cb_clearance_depth, queue_size=1, buff_size=2**24),
                rospy.Subscriber('/depth_cam/rgb/image_raw', RosImage,
                                 self._cb_astra, queue_size=1, buff_size=2**24),
                rospy.Subscriber('/camera/rgb/image_raw', RosImage,
                                 self._cb_astra, queue_size=1, buff_size=2**24),
                rospy.Subscriber('/depth_cam/depth/image_raw', RosImage,
                                 self._cb_astra_depth, queue_size=1, buff_size=2**24),
                rospy.Subscriber('/depth_cam/depth_registered/image_raw', RosImage,
                                 self._cb_astra_depth, callback_args=True,
                                 queue_size=1, buff_size=2**24),
                rospy.Subscriber('/camera/depth/image_raw', RosImage,
                                 self._cb_astra_depth, queue_size=1, buff_size=2**24),
            ]
        else:
            cam_source_astra = "Khong co sensor_msgs/Image"
            rospy.logerr(">> [ASTRA] Khong import duoc sensor_msgs/Image")

    def _cb_clearance_info(self, msg):
        with self.depth_info_lock:
            self.depth_camera_info = msg

    def _cb_clearance_depth(self, msg):
        if not self.depth_guard.enabled:
            return
        with self.depth_info_lock:
            info = self.depth_camera_info
        if (info is None or info.header.frame_id != msg.header.frame_id
                or info.width != msg.width or info.height != msg.height
                or msg.encoding not in ('16UC1', 'mono16', '32FC1')
                or info.K[0] <= 0 or info.K[4] <= 0
                or any(abs(v) > 1e-6 for v in info.D)):
            self.depth_guard.status = 'DEPTH NEED MATCHING RECTIFIED CAMERA INFO'
            return
        age = rospy.Time.now().to_sec() - msg.header.stamp.to_sec()
        if age < -0.05 or age > 0.4:
            return
        try:
            is_float = msg.encoding == '32FC1'
            dtype = np.dtype(('>' if msg.is_bigendian else '<') + ('f4' if is_float else 'u2'))
            row = msg.step // dtype.itemsize
            depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row)[:, :msg.width].astype(np.float32)
            if is_float:
                depth *= 1000.0
            self.depth_guard.update(depth, (info.K[0], info.K[4], info.K[2], info.K[5]),
                                    time.monotonic() - max(0, age))
        except (ValueError, TypeError) as error:
            self.depth_guard.status = 'DEPTH DECODE ERROR: ' + str(error)

    def _cb_astra(self, msg):
        global raw_frame_astra, display_frame_astra, cam_source_astra, astra_frame_time, astra_frame_seq
        frame = self._decode_ros_image(msg)
        if frame is not None:
            cam_source_astra = "Astra Pro (ROS)"
            self._camera_frame_count += 1
            frame_time = time.time()
            if frame_time - self._camera_fps_start >= 1.0:
                self.camera_fps = self._camera_frame_count / (frame_time - self._camera_fps_start)
                self._camera_frame_count = 0
                self._camera_fps_start = frame_time
            # Render the newest known inference result on each arriving Astra
            # frame.  Camera/web FPS is no longer limited by YOLO FPS.
            with self.state_lock:
                detections = list(self.latest_detections)
                target_center = self.latest_target_center
                action_text = self.last_action_text
                status_color = self.last_status_color
            rendered = self._render_ai_overlay(frame.copy(), detections, action_text,
                                               status_color, target_center)
            with frame_lock:
                raw_frame_astra = frame
                display_frame_astra = rendered
                astra_frame_time = time.time()
                astra_frame_seq += 1

    def _cb_astra_depth(self, msg, registered=False):
        global raw_depth_astra, astra_depth_time, astra_depth_registered
        try:
            # RGB and depth pixels are directly comparable only for the
            # registered stream.  Fall back to raw depth only when that stream
            # is unavailable.
            now = time.time()
            if not registered and now - self._astra_registered_depth_time < 1.0:
                return
            if now - astra_depth_time < 0.12 and (not registered or astra_depth_registered):
                return
            if msg.encoding not in ('16UC1', 'mono16', '16SC1'):
                return
            row_width = int(msg.step // 2) if msg.step else int(msg.width)
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, row_width))[:, :msg.width]
            with frame_lock:
                raw_depth_astra = depth.copy()
                astra_depth_time = now
                astra_depth_registered = registered
            if registered:
                self._astra_registered_depth_time = now
        except Exception as depth_err:
            rospy.logwarn_throttle(5.0, ">> [ASTRA] Loi doc depth: " + str(depth_err))

    def init_oakd_capture(self):
        global raw_frame_oakd, display_frame_oakd, cam_source_oakd, obstacle_info
        cam_source_oakd = "Dang ket noi DepthAI..."
        if dai is not None:
            try:
                devices = dai.Device.getAllAvailableDevices()
                if devices:
                    pipeline = dai.Pipeline()
                    cam_rgb = pipeline.createColorCamera()
                    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
                    cam_rgb.setPreviewSize(480, 360)
                    cam_rgb.setInterleaved(False)
                    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
                    cam_rgb.setFps(10)
                    xout_rgb = pipeline.createXLinkOut()
                    xout_rgb.setStreamName("rgb")
                    cam_rgb.preview.link(xout_rgb.input)

                    has_stereo = False
                    if USE_OAKD_FOR_AVOIDANCE:
                        try:
                            mono_l = pipeline.createMonoCamera()
                            mono_l.setBoardSocket(dai.CameraBoardSocket.LEFT)
                            mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
                            mono_l.setFps(10)
                            mono_r = pipeline.createMonoCamera()
                            mono_r.setBoardSocket(dai.CameraBoardSocket.RIGHT)
                            mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
                            mono_r.setFps(10)
                            stereo = pipeline.createStereoDepth()
                            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
                            stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
                            mono_l.out.link(stereo.left)
                            mono_r.out.link(stereo.right)
                            xout_d = pipeline.createXLinkOut()
                            xout_d.setStreamName("depth")
                            stereo.depth.link(xout_d.input)
                            has_stereo = True
                        except Exception as stereo_err:
                            rospy.logwarn(">> [OAK-D] Khong khoi tao Stereo Depth, dung RGB: " + str(stereo_err))

                    device = None
                    try:
                        device = dai.Device(pipeline)
                    except Exception:
                        time.sleep(2)
                        device = dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)

                    q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                    q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False) if has_stereo else None
                    cam_source_oakd = "OAK-D {} ({})".format(
                        "Spatial" if has_stereo else "RGB", device.getUsbSpeed().name)

                    cur_dl, cur_dc, cur_dr = 999.0, 999.0, 999.0
                    while not rospy.is_shutdown():
                        if q_depth is not None:
                            in_d = q_depth.tryGet()
                            if in_d is not None:
                                cur_dl, cur_dc, cur_dr = process_obstacle_depth(in_d.getFrame())
                                with obstacle_lock:
                                    obstacle_info['dist_l'] = cur_dl
                                    obstacle_info['dist_c'] = cur_dc
                                    obstacle_info['dist_r'] = cur_dr
                                    obstacle_info['status'] = 'NGUY HIEM' if cur_dc < 30.0 else ('CANH BAO' if cur_dc < 60.0 else 'AN TOAN')
                                    obstacle_info['last_update'] = time.time()

                        if q_rgb is not None:
                            in_rgb = q_rgb.tryGet()
                            if in_rgb is not None:
                                f = in_rgb.getCvFrame()
                                if f is not None:
                                    with obstacle_lock:
                                        s_dir = obstacle_info.get('strafe_dir', 'NONE')
                                    rendered = render_oakd_hud(f, cur_dl, cur_dc, cur_dr, s_dir)
                                    with frame_lock:
                                        raw_frame_oakd = f
                                        display_frame_oakd = rendered
                        time.sleep(0.01)
                    return
            except Exception as oakd_err:
                rospy.logwarn(">> [OAK-D] DepthAI loi, thu ROS fallback: " + str(oakd_err))

        cam_source_oakd = "Fallback ROS Topic"
        if RosImage is not None:
            self.oakd_subscribers = [
                rospy.Subscriber('/oakd/rgb/image_raw', RosImage,
                                 self._cb_oakd_ros, queue_size=1, buff_size=2**24)
            ]
        else:
            cam_source_oakd = "Khong co sensor_msgs/Image"

    def _cb_oakd_ros(self, msg):
        global raw_frame_oakd, display_frame_oakd
        frame = self._decode_ros_image(msg)
        if frame is not None:
            rendered = render_oakd_hud(frame, 999.0, 999.0, 999.0, "NONE")
            with frame_lock:
                raw_frame_oakd = frame
                display_frame_oakd = rendered

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

    # ==========================================================
    # YOLO & NHẬN DIỆN MỤC TIÊU
    # ==========================================================
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
            obj_mask = arr[:, 4] > 0.25
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

        conf_mask = confidences > 0.25
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
            indices = cv2.dnn.NMSBoxes(boxes, conf_list, 0.25, 0.45)
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

    # ==========================================================
    # BỘ NÃO TÌM KIẾM ĐÓN ĐẦU (SMART ARBITRATION + OBJECT PERMANENCE)
    # ==========================================================
    def smart_search_loop(self):
        global raw_frame_astra, display_frame_astra, target_memory
        frame_counter = 0
        last_fps_time = time.time()
        last_frame_seq = -1
        last_target_center = None
        last_target_time = 0.0
        search_target = TARGET_CLASS

        while not rospy.is_shutdown():
            raw_frame = None
            with frame_lock:
                frame_seq = astra_frame_seq
                if raw_frame_astra is not None and frame_seq != last_frame_seq:
                    raw_frame = raw_frame_astra.copy()

            if raw_frame is None:
                if time.time() - astra_frame_time > 1.5:
                    with self.state_lock:
                        self.last_action_text = "CHO CAMERA ASTRA"
                time.sleep(0.04)
                continue
            last_frame_seq = frame_seq
            self.last_ai_update = time.time()

            frame_counter += 1
            now = time.time()
            if TARGET_CLASS != search_target:
                search_target = TARGET_CLASS
                last_target_center = None
                last_target_time = 0.0
                self.explorer.reset_scan(clear_visited=True)
            if now - last_fps_time >= 1.0:
                self.ai_fps = frame_counter / (now - last_fps_time)
                frame_counter = 0
                last_fps_time = now

            h, w = raw_frame.shape[:2]
            center_screen_x = w // 2

            # 1. Nhận diện YOLO
            inference_started = time.time()
            detections = self.run_yolo(raw_frame)
            self.inference_ms = (time.time() - inference_started) * 1000.0
            target_detections = [d for d in detections if d['class_name'] == search_target]
            target_found = len(target_detections) > 0
            now = time.time()
            if self.target_announcement.update(
                    search_target, target_found, time.monotonic(),
                    enabled=not IS_PAUSED and now >= self.manual_override_until):
                self.greeting_event_id += 1

            with pose_lock:
                cur_rx = robot_pose['x']
                cur_ry = robot_pose['y']
                cur_yaw = robot_pose['yaw']
                odom_fresh = now - robot_pose['last_update'] < 1.5

            target_center = None
            est_dist = 0.0
            astra_distance_cm = None

            # 2. CẬP NHẬT BỘ NHỚ KHÔNG GIAN (OBJECT PERMANENCE)
            if target_found:
                # Keep following the same image location when several objects
                # of the selected class are visible.
                if last_target_center is not None and now - last_target_time < 1.0:
                    def target_score(det):
                        x, y, bw, bh = det['box']
                        dx = x + bw * 0.5 - last_target_center[0]
                        dy = y + bh * 0.5 - last_target_center[1]
                        return det['conf'] - 0.5 * math.hypot(dx, dy) / max(w, h)
                    best = max(target_detections, key=target_score)
                else:
                    best = max(target_detections, key=lambda d: d['conf'])
                bx, by, bw, bh = best['box']
                target_center = (bx + bw // 2, by + bh // 2)
                last_target_center = target_center
                last_target_time = now

                # Ước lượng khoảng cách từ kích thước Bounding Box
                # Đối với người/vật thể, tỉ lệ width_ratio nghịch với khoảng cách
                width_ratio = bw / float(w)
                # Target detection and range use the same Astra camera.  The
                # box-size formula below is only a fallback for invalid depth.
                with frame_lock:
                    astra_depth = (raw_depth_astra.copy() if raw_depth_astra is not None
                                   and astra_depth_registered and
                                   abs(astra_depth_time - astra_frame_time) < 0.25 else None)
                astra_distance_cm = estimate_astra_distance_cm(astra_depth, raw_frame.shape, best['box'])
                est_dist = (astra_distance_cm / 100.0 if astra_distance_cm is not None
                            else max(0.4, min(5.0, 0.45 / max(width_ratio, 0.08))))

                # Tính góc lệch theo FOV ngang của camera Astra (~60 độ)
                error_x = target_center[0] - center_screen_x
                angle_offset = - (error_x / float(center_screen_x)) * math.radians(30.0)

                # Tính tọa độ thực (World X, Y) của mục tiêu
                target_world_x = cur_rx + est_dist * math.cos(cur_yaw + angle_offset)
                target_world_y = cur_ry + est_dist * math.sin(cur_yaw + angle_offset)

                with target_lock:
                    target_memory['seen'] = True
                    target_memory['x'] = target_world_x
                    target_memory['y'] = target_world_y
                    target_memory['last_seen_time'] = now
                    target_memory['distance'] = est_dist
            else:
                with target_lock:
                    target_memory['seen'] = False

            # 3. MÁY TRẠNG THÁI TÌM KIẾM THÔNG MINH
            with target_lock:
                has_last_pos = (target_memory['x'] is not None)
                time_since_seen = now - target_memory['last_seen_time']
                last_tx = target_memory['x']
                last_ty = target_memory['y']

            base_vx = 0.0
            heading_wz = 0.0
            action_text = ""
            status_color = (0, 255, 0)

            if target_found:
                # CHẾ ĐỘ 1: BÁM ĐUỔI TRỰC DIỆN (TRACKING)
                self.search_mode = "TRACKING"
                error_x = target_center[0] - center_screen_x
                heading_wz = max(min(float(-error_x * 0.07), 25.0), -25.0)

                if ((astra_distance_cm is not None and astra_distance_cm < (PERSON_STOP_CM if TARGET_CLASS == 'person' else 55.0)) or
                        (astra_distance_cm is None and width_ratio > 0.45)):
                    base_vx = 0.0
                    heading_wz = 0.0
                    action_text = "ĐÃ ĐẾN GẦN MỤC TIÊU! DỪNG LẠI"
                else:
                    base_vx = 0.15 if abs(error_x) < 80 else 0.08
                    action_text = "BÁM MỤC TIÊU {} (Cách {:.1f}m)".format(TARGET_CLASS.upper(), est_dist)

            elif has_last_pos and time_since_seen < TARGET_REACQUIRE_S:
                self.search_mode = "REACQUIRE"
                dx, dy = last_tx - cur_rx, last_ty - cur_ry
                yaw_diff = (math.atan2(dy, dx) - cur_yaw + math.pi) % (2 * math.pi) - math.pi
                heading_wz = 18.0 if yaw_diff >= 0 else -18.0
                base_vx = 0.0
                action_text = "MAT DAU -> QUET TIM LAI (1.5s)"
                status_color = (0, 215, 255)

            else:
                # CHẾ ĐỘ 3: Quét 360 độ rồi đi đến rìa vùng chưa biết có
                # đường đi qua các ô đã đo là trống.
                if self.search_mode != "EXPLORE":
                    self.explorer.reset_scan()
                self.search_mode = "EXPLORE"
                with lidar_lock:
                    scan_fresh = now - lidar_info['last_update'] < 1.5
                if IS_PAUSED or now < self.manual_override_until:
                    self.explorer.reset_scan()
                    action_text = "TẠM DỪNG TÌM KIẾM"
                elif not odom_fresh or not scan_fresh:
                    self.explorer.reset_scan()
                    action_text = ("CHO DU LIEU ODOMETRY" if not odom_fresh
                                   else "CHO DU LIEU LIDAR")
                    status_color = (0, 0, 255)
                else:
                    base_vx, heading_wz, action_text = self.explorer.step(
                        cur_rx, cur_ry, cur_yaw, now)
                    status_color = (148, 163, 184)

            # 4. LiDAR safety; optional OAK-D depth can add close-range avoidance.
            with obstacle_lock:
                d_l, d_c, d_r = active_oakd_distances(obstacle_info, time.time())
            with lidar_lock:
                lidar = dict(lidar_info)

            cmd_vx = 0.0
            cmd_vy = 0.0
            cmd_wz = 0.0
            strafe_name = "NONE"

            if time.time() < self.manual_override_until:
                action_text = "LÁI THỦ CÔNG (MANUAL)"
                status_color = (0, 229, 255)
            elif IS_PAUSED:
                action_text = "TẠM DỪNG"
                status_color = (0, 165, 255)
                self.set_desired_velocity(0.0, 0.0, 0.0)
            elif not odom_fresh or time.time() - lidar['last_update'] > 1.5:
                action_text = ("CHO DU LIEU ODOMETRY" if not odom_fresh
                               else "CHO DU LIEU LIDAR")
                status_color = (0, 0, 255)
                self.set_desired_velocity(0.0, 0.0, 0.0)
            else:
                arrived = target_found and base_vx == 0.0 and heading_wz == 0.0
                if arrived:
                    action_text = "ĐÃ ĐẾN GẦN MỤC TIÊU! DỪNG LẠI"
                elif self.search_mode == "EXPLORE" and d_c < 60.0:
                    # Do not leave the planned, known-free path to sidestep.
                    action_text = "OAK-D BAO VAT CAN, DUNG DE LAP DUONG LAI"
                    status_color = (0, 0, 255)
                elif d_c < 30.0:
                    # PHANH CỨNG TIẾN THẲNG, TRƯỢT NGANG NÉ
                    cmd_vx = 0.0
                    cmd_vy = 0.15 if d_l > d_r else -0.15
                    strafe_name = "LEFT" if d_l > d_r else "RIGHT"
                    cmd_wz = heading_wz if target_found else 0.0
                    action_text = "PHANH! TRƯỢT {} NÉ ({:.0f}cm)".format("TRÁI" if d_l > d_r else "PHẢI", d_c)
                    status_color = (0, 0, 255)
                elif d_c < 60.0:
                    # TRƯỢT NGANG MECANUM LÁCH VÒNG QUA VẬT CẢN
                    cmd_vx = 0.04
                    cmd_vy = 0.13 if d_l > d_r else -0.13
                    strafe_name = "LEFT" if d_l > d_r else "RIGHT"
                    cmd_wz = heading_wz
                    action_text = "LÁCH {} NÉ ({:.0f}cm)".format("TRÁI" if d_l > d_r else "PHẢI", d_c)
                    status_color = (0, 215, 255)
                else:
                    cmd_vx = base_vx
                    cmd_vy = 0.0
                    cmd_wz = heading_wz

                allowed, reason = FOOTPRINT.check(cmd_vx, cmd_vy, cmd_wz, time.monotonic())
                if not allowed:
                    original_turn = cmd_wz
                    cmd_vx = cmd_vy = cmd_wz = 0.0
                    action_text = reason
                    # Reactive target following may select a verified turn. Exploration
                    # recovery owns its own retries; do not bypass its trajectory.
                    if self.search_mode in ('TRACKING', 'REACQUIRE'):
                        preferred = 18.0 if original_turn >= 0 else -18.0
                        for candidate in (preferred, -preferred):
                            if FOOTPRINT.check(0.0, 0.0, candidate, time.monotonic())[0]:
                                cmd_wz = candidate
                                action_text = 'XOAY NE THEO VUNG THAN XE'
                                break

                with obstacle_lock:
                    obstacle_info['strafe_dir'] = strafe_name

                self.set_desired_velocity(cmd_vx, cmd_vy, cmd_wz)

            # Vẽ Overlay lên khung hình Astra
            rendered_astra = self._render_ai_overlay(raw_frame, detections, action_text, status_color, target_center)
            with frame_lock:
                display_frame_astra = rendered_astra

            with self.state_lock:
                self.last_action_text = action_text
                self.last_status_color = status_color
                self.latest_detections = list(detections)
                self.latest_target_center = target_center

            time.sleep(0.015)

    def _render_ai_overlay(self, frame, detections, action_text, status_color, target_center):
        h, w = frame.shape[:2]
        center_screen_x = w // 2
        cv2.line(frame, (center_screen_x, 0), (center_screen_x, h), (255, 255, 255), 1)

        for det in detections:
            bx, by, bw, bh = det['box']
            color = (0, 255, 0) if det['class_name'] == TARGET_CLASS else (100, 100, 100)
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), color, 2)
            cv2.putText(frame, "{} {:.0f}%".format(det['class_name'], det['conf'] * 100), (bx, by - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

        if target_center is not None:
            cv2.circle(frame, target_center, 6, (0, 0, 255), -1)

        cv2.putText(frame, action_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 2)
        cv2.putText(frame, "[SMART SEEKER] ASTRA AI", (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 229, 255), 2)
        cv2.putText(frame, "AI FPS: {:.1f}".format(self.ai_fps), (w - 140, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return frame

    def stop(self):
        self.set_desired_velocity(0.0, 0.0, 0.0)
        zero = Twist()
        for pub in self.cmd_publishers:
            try:
                pub.publish(zero)
            except Exception:
                pass


# ==========================================================
# MAIN ENTRY POINT
# ==========================================================
def sigint_handler(signum, frame):
    print("\n>> Nhan tin hieu Ctrl+C! Dang dung dong co va thoat an toan...", flush=True)
    if seeker_instance:
        seeker_instance.set_desired_velocity(0.0, 0.0, 0.0)
    try:
        rospy.signal_shutdown("Ctrl+C pressed")
    except Exception:
        pass


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    try:
        seeker = SmartObjectSeeker()
        web_thread = threading.Thread(target=start_web_server)
        web_thread.daemon = True
        web_thread.start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sigint_handler(None, None)
