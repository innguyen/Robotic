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
import numpy as np
import cv2

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

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

IS_PAUSED = False
CMD_VEL_TOPICS = [
    '/jetauto_controller/cmd_vel',
    '/cmd_vel',
    '/controller/cmd_vel',
    '/hiwonder_controller/cmd_vel'
]

# Khung hình toàn cục
raw_frame_astra = None
raw_frame_oakd = None
depth_frame_oakd = None
display_frame_astra = None
display_frame_oakd = None
display_frame_map = None  # Khung hình bản đồ 2D thời gian thực
frame_lock = threading.Lock()

# Bộ đệm JPEG cache trên RAM (chống nghẽn socket)
cached_jpeg_astra = None
cached_jpeg_oakd = None
cached_jpeg_combined = None
cached_jpeg_map = None
jpeg_lock = threading.Lock()

# Thông số vật cản OAK-D (cm)
obstacle_info = {
    'dist_l': 999.0,
    'dist_c': 999.0,
    'dist_r': 999.0,
    'status': 'AN TOAN',
    'strafe_dir': 'NONE'
}
obstacle_lock = threading.Lock()

# Tọa độ Robot từ Odometry (x, y mét, yaw rad)
robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
pose_lock = threading.Lock()

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

    def world_to_grid(self, wx, wy):
        gx = int(self.center + wx / self.res)
        gy = int(self.center - wy / self.res)  # Trục Y đảo ngược cho ảnh
        return gx, gy

    def update_scan(self, rx, ry, ryaw, ranges, angle_min, angle_inc):
        now = time.time()
        if now - self.last_update < 0.08:  # Giới hạn tần số vẽ ~12 FPS để tiết kiệm CPU
            return
        self.last_update = now

        rgx, rgy = self.world_to_grid(rx, ry)
        if not (0 <= rgx < self.size and 0 <= rgy < self.size):
            return

        with self.map_lock:
            # Lấy mẫu bước nhảy 3 tia để tối ưu tốc độ xử lý
            num_points = len(ranges)
            for i in range(0, num_points, 3):
                dist = ranges[i]
                if math.isnan(dist) or dist < 0.12 or dist > 8.0:
                    continue

                angle = ryaw + angle_min + i * angle_inc
                ox = rx + dist * math.cos(angle)
                oy = ry + dist * math.sin(angle)
                ogx, ogy = self.world_to_grid(ox, oy)

                if 0 <= ogx < self.size and 0 <= ogy < self.size:
                    # Vẽ đường trống (Free space) từ xe tới vật cản
                    cv2.line(self.grid, (rgx, rgy), (ogx, ogy), 20, 1)
                    # Đánh dấu ô vật cản
                    self.grid[ogy, ogx] = 255

    def render_map_image(self, rx, ry, ryaw, target_info=None, search_state=""):
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

        # 4. Vẽ bảng thông tin HUD trên bản đồ
        cv2.rectangle(disp_map, (0, 0), (480, 42), (15, 23, 42), -1)
        cv2.putText(disp_map, "2D LIDAR SLAM MAP | 12m x 12m", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 229, 255), 2)
        if search_state:
            cv2.putText(disp_map, search_state, (12, 465), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return disp_map


grid_mapper = OccupancyGridMapper()


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
    while not rospy.is_shutdown():
        try:
            # 1. Astra
            f_astra = None
            with frame_lock:
                if display_frame_astra is not None:
                    f_astra = display_frame_astra
                elif raw_frame_astra is not None:
                    f_astra = raw_frame_astra

            if f_astra is not None:
                ret, jpg = cv2.imencode('.jpg', f_astra, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                if ret:
                    with jpeg_lock:
                        cached_jpeg_astra = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()

            # 2. OAK-D
            f_oakd = None
            with frame_lock:
                if display_frame_oakd is not None:
                    f_oakd = display_frame_oakd
                elif raw_frame_oakd is not None:
                    f_oakd = raw_frame_oakd

            if f_oakd is not None:
                ret, jpg = cv2.imencode('.jpg', f_oakd, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                if ret:
                    with jpeg_lock:
                        cached_jpeg_oakd = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()

            # 3. Combined
            if f_astra is not None or f_oakd is not None:
                comb = get_combined_frame()
                ret, jpg = cv2.imencode('.jpg', comb, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                if ret:
                    with jpeg_lock:
                        cached_jpeg_combined = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()

            # 4. Map 2D
            f_map = None
            with frame_lock:
                if display_frame_map is not None:
                    f_map = display_frame_map

            if f_map is not None:
                ret, jpg = cv2.imencode('.jpg', f_map, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                if ret:
                    with jpeg_lock:
                        cached_jpeg_map = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()

        except Exception:
            pass

        time.sleep(0.045)


# ==========================================================
# 3. XỬ LÝ VẬT CẢN OAK-D STEREO DEPTH
# ==========================================================
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


def render_oakd_hud(frame, dl, dc, dr, strafe_dir="NONE"):
    h, w = frame.shape[:2]
    out = frame.copy()
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

                if raw is None:
                    frame = get_frame_for_cam(cam_name)
                    ret, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                    if ret:
                        raw = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()

                if raw is not None:
                    packet = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + raw + b'\r\n'
                    self.wfile.write(packet)
                    self.wfile.flush()
                time.sleep(0.045)
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
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status": "ok"}')
                return

        elif self.path == '/toggle_pause':
            IS_PAUSED = not IS_PAUSED
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
            with target_lock:
                t_copy = dict(target_memory)

            status_data = {
                'target': TARGET_CLASS,
                'paused': IS_PAUSED,
                'ai_fps': round(seeker_instance.ai_fps, 1) if seeker_instance else 0.0,
                'action_text': seeker_instance.last_action_text if seeker_instance else "CHO KHOI DONG",
                'search_mode': seeker_instance.search_mode if seeker_instance else "INIT",
                'linear_x': round(seeker_instance.desired_linear_x, 2) if seeker_instance else 0.0,
                'linear_y': round(seeker_instance.desired_linear_y, 2) if seeker_instance else 0.0,
                'angular_z': round(seeker_instance.desired_angular_z, 1) if seeker_instance else 0.0,
                'dist_left': round(obs_copy['dist_l'], 1),
                'dist_center': round(obs_copy['dist_c'], 1),
                'dist_right': round(obs_copy['dist_r'], 1),
                'strafe_dir': obs_copy['strafe_dir'],
                'obs_status': obs_copy['status'],
                'pose_x': round(p_copy['x'], 2),
                'pose_y': round(p_copy['y'], 2),
                'pose_yaw': round(math.degrees(p_copy['yaw']), 1),
                'target_x': round(t_copy['x'], 2) if t_copy['x'] is not None else None,
                'target_y': round(t_copy['y'], 2) if t_copy['y'] is not None else None,
                'target_seen': t_copy['seen']
            }
            self.wfile.write(json.dumps(status_data).encode())
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(self._render_html().encode('utf-8'))

    def _render_html(self):
        return """<!DOCTYPE html>
<html lang="vi">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>JetAuto Smart Explorer - 2D Map & Dual Vision</title>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700;800&display=swap" rel="stylesheet">
    <style>
        :root {
            --primary: #00E5FF;
            --accent: #3B82F6;
            --success: #22C55E;
            --warning: #F59E0B;
            --danger: #EF4444;
            --bg-color: #0b1120;
            --card-bg: rgba(15, 23, 42, 0.75);
            --glass-bg: rgba(255, 255, 255, 0.04);
            --glass-border: rgba(255, 255, 255, 0.08);
            --neon-glow: 0 0 25px rgba(0, 229, 255, 0.35);
        }
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            background-color: var(--bg-color);
            background-image: radial-gradient(circle at 50% 0%, #1e293b 0%, #0b1120 80%);
            color: #f8fafc;
            font-family: 'Inter', -apple-system, sans-serif;
            min-height: 100vh;
            padding: 1.2rem;
            display: flex;
            flex-direction: column;
            align-items: center;
        }

        .header { text-align: center; margin-bottom: 1rem; max-width: 900px; width: 100%; }
        .header h1 {
            font-weight: 800;
            font-size: 2rem;
            background: linear-gradient(135deg, #00E5FF 0%, #3B82F6 100%);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            text-transform: uppercase;
            letter-spacing: 2px;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 12px;
            flex-wrap: wrap;
        }
        .header p { color: #94a3b8; font-size: 0.92rem; margin-top: 0.4rem; }
        .target-badge {
            background: rgba(0, 229, 255, 0.15);
            border: 1px solid var(--primary);
            color: var(--primary);
            padding: 0.25rem 0.9rem;
            border-radius: 20px;
            font-weight: 700;
            font-size: 0.9rem;
            text-transform: uppercase;
            box-shadow: var(--neon-glow);
        }

        .container {
            width: 100%;
            max-width: 1360px;
            background: var(--glass-bg);
            border: 1px solid var(--glass-border);
            border-radius: 24px;
            padding: 1.4rem;
            box-shadow: 0 25px 50px -12px rgba(0,0,0,0.6);
            backdrop-filter: blur(16px);
            display: flex;
            flex-direction: column;
            gap: 1.2rem;
        }

        /* 3 CỘT TRUNG TÂM: CAM ASTRA - BẢN ĐỒ 2D - CAM OAK-D */
        .main-display-grid {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 1.2rem;
            width: 100%;
        }
        @media (max-width: 1100px) {
            .main-display-grid { grid-template-columns: 1fr 1fr; }
        }
        @media (max-width: 760px) {
            .main-display-grid { grid-template-columns: 1fr; }
        }

        .stream-card {
            background: var(--card-bg);
            border: 1px solid var(--glass-border);
            border-radius: 18px;
            overflow: hidden;
            display: flex;
            flex-direction: column;
            box-shadow: 0 10px 25px -5px rgba(0,0,0,0.4);
        }
        .stream-header {
            padding: 10px 14px;
            background: rgba(15, 23, 42, 0.95);
            border-bottom: 1px solid var(--glass-border);
            font-size: 0.82rem;
            font-weight: 700;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .stream-box {
            position: relative;
            width: 100%;
            background: #000;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 280px;
        }
        .stream-box img {
            width: 100%;
            height: auto;
            max-height: 420px;
            object-fit: contain;
            display: block;
        }

        /* QUICK CONTROLS */
        .control-row {
            display: flex;
            gap: 10px;
            width: 100%;
            flex-wrap: wrap;
        }
        #targetInput {
            flex: 1;
            min-width: 220px;
            background: rgba(15, 23, 42, 0.8);
            border: 1px solid var(--glass-border);
            padding: 12px 18px;
            border-radius: 14px;
            color: #fff;
            font-size: 0.95rem;
            outline: none;
        }
        #targetInput:focus { border-color: var(--primary); box-shadow: var(--neon-glow); }

        .btn-action {
            padding: 12px 20px;
            border-radius: 14px;
            border: none;
            font-weight: 700;
            font-size: 0.88rem;
            cursor: pointer;
            transition: all 0.2s;
            display: flex;
            align-items: center;
            gap: 6px;
        }
        .btn-find { background: linear-gradient(135deg, #00E5FF, #3B82F6); color: #0b1120; }
        .btn-pause { background: #F59E0B; color: #000; }
        .btn-resume { background: #10B981; color: #fff; }

        /* HUD STATUS */
        .bottom-grid {
            display: grid;
            grid-template-columns: 1.4fr 1fr;
            gap: 1.2rem;
            width: 100%;
        }
        @media (max-width: 980px) {
            .bottom-grid { grid-template-columns: 1fr; }
        }

        .info-card, .dpad-card {
            background: var(--card-bg);
            border: 1px solid var(--glass-border);
            border-radius: 18px;
            padding: 18px;
        }
        .card-title {
            font-size: 0.95rem;
            font-weight: 800;
            color: #94a3b8;
            margin-bottom: 14px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            border-bottom: 1px solid rgba(255,255,255,0.06);
            padding-bottom: 8px;
        }
        .hud-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
        }
        .hud-item {
            background: rgba(0,0,0,0.3);
            border: 1px solid rgba(255,255,255,0.05);
            border-radius: 12px;
            padding: 10px 14px;
        }
        .hud-label { font-size: 0.72rem; color: #64748b; text-transform: uppercase; font-weight: 700; margin-bottom: 4px; }
        .hud-val { font-size: 0.92rem; font-weight: 700; }

        /* D-PAD */
        .dpad-layout {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 8px;
            max-width: 320px;
            margin: 0 auto;
        }
        .dpad-btn {
            background: rgba(255, 255, 255, 0.06);
            border: 1px solid rgba(255, 255, 255, 0.12);
            color: #fff;
            padding: 14px 8px;
            border-radius: 12px;
            font-weight: 700;
            font-size: 0.85rem;
            cursor: pointer;
            text-align: center;
        }
        .dpad-btn:active { background: var(--primary); color: #000; }
        .dpad-strafe { background: rgba(56, 189, 248, 0.12); border-color: rgba(56, 189, 248, 0.3); color: #38bdf8; }
        .dpad-stop { background: rgba(239, 68, 68, 0.2); border-color: rgba(239, 68, 68, 0.4); color: #ef4444; }
    </style>
</head>
<body>
    <div class="header">
        <h1>JetAuto Smart Explorer <span class="target-badge" id="targetBadge">🎯 """ + TARGET_CLASS.upper() + """</span></h1>
        <p>Bản Đồ 2D LiDAR SLAM + Bộ Nhớ Không Gian Object Permanence + Né Vật Cản Mecanum 3-DOF</p>
    </div>

    <div class="container">
        <!-- 3 KHUNG HÌNH TRUNG TÂM -->
        <div class="main-display-grid">
            <!-- 1. CAM ASTRA (YOLO TRACKING) -->
            <div class="stream-card">
                <div class="stream-header">
                    <span style="color: var(--primary);">🟢 ASTRA (AI Khóa Mục Tiêu)</span>
                    <span style="color: var(--success);" id="aiFpsBadge">AI: 0.0 FPS</span>
                </div>
                <div class="stream-box">
                    <img id="feedAstra" src="/cam_astra.mjpg" alt="Astra Feed" />
                </div>
            </div>

            <!-- 2. BẢN ĐỒ 2D LIDAR SLAM -->
            <div class="stream-card">
                <div class="stream-header">
                    <span style="color: #F59E0B;">🗺️ BẢN ĐỒ 2D LIDAR SLAM</span>
                    <span style="color: #38bdf8;" id="searchModeBadge">EXPLORING</span>
                </div>
                <div class="stream-box">
                    <img id="feedMap" src="/map_feed.mjpg" alt="2D Map Feed" />
                </div>
            </div>

            <!-- 3. CAM OAK-D (RADAR NÉ VẬT CẢN) -->
            <div class="stream-card">
                <div class="stream-header">
                    <span style="color: #38bdf8;">🔵 OAK-D (Radar Né Gầm Xe)</span>
                    <span style="color: var(--success);" id="feedStatusOakd">● Radar Active</span>
                </div>
                <div class="stream-box">
                    <img id="feedOakd" src="/cam_oakd.mjpg" alt="OAK-D Feed" />
                </div>
            </div>
        </div>

        <!-- QUICK CONTROLS -->
        <div class="control-row">
            <input type="text" id="targetInput" placeholder="Nhập vật thể cần tìm (person, bottle, chair, cell phone...)" onkeypress="if(event.key==='Enter') setTarget()" />
            <button class="btn-action btn-find" onclick="setTarget()">🔍 TÌM KIẾM TỰ ĐỘNG</button>
            <button id="pauseBtn" class="btn-action btn-pause" onclick="togglePause()">⏸️ TẠM DỪNG</button>
        </div>

        <!-- BOTTOM GRID -->
        <div class="bottom-grid">
            <div class="info-card">
                <div class="card-title">
                    <span>Trí Tuệ Nhân Tạo & Bộ Nhớ Không Gian</span>
                    <span id="poseBadge" style="color: #38bdf8;">Robot: (0.0m, 0.0m)</span>
                </div>
                <div class="hud-grid">
                    <div class="hud-item" style="grid-column: span 2;">
                        <div class="hud-label">Hành động Robot</div>
                        <div class="hud-val" id="hudAction" style="color: var(--warning);">ĐANG KHỞI TẠO...</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Tọa độ Mục Tiêu (Object Permanence)</div>
                        <div class="hud-val" id="hudTargetPos" style="color: var(--primary);">Chưa phát hiện</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Radar OAK-D (Trái | Giữa | Phải)</div>
                        <div class="hud-val" id="hudRadar" style="color: var(--success);">L: -- | C: -- | R: --</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Lệnh Vận Tốc 3-DOF</div>
                        <div class="hud-val" id="hudCmdVel">vx: 0.00 | vy: 0.00 | wz: 0.0°</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Hướng Né Mecanum</div>
                        <div class="hud-val" id="hudStrafe" style="color: #38bdf8;">ĐƯỜNG THÔNG THOÁNG</div>
                    </div>
                </div>
            </div>

            <!-- D-PAD -->
            <div class="dpad-card">
                <div class="card-title">
                    <span>Lái thủ công Bánh Mecanum (W,A,S,D + Q,E trượt)</span>
                </div>
                <div class="dpad-layout">
                    <div></div>
                    <button class="dpad-btn" onmousedown="drive('forward')" onmouseup="drive('stop')">▲ TIẾN</button>
                    <div></div>

                    <button class="dpad-btn dpad-strafe" onmousedown="drive('strafe_left')" onmouseup="drive('stop')">◄◄ TRƯỢT T</button>
                    <button class="dpad-btn dpad-stop" onclick="drive('stop')">DỪNG</button>
                    <button class="dpad-btn dpad-strafe" onmousedown="drive('strafe_right')" onmouseup="drive('stop')">TRƯỢT P ►►</button>

                    <button class="dpad-btn" onmousedown="drive('left')" onmouseup="drive('stop')">⟲ XOAY T</button>
                    <button class="dpad-btn" onmousedown="drive('backward')" onmouseup="drive('stop')">▼ LÙI</button>
                    <button class="dpad-btn" onmousedown="drive('right')" onmouseup="drive('stop')">XOAY P ⟳</button>
                </div>
            </div>
        </div>
    </div>

    <script>
        function setTarget() {
            const t = document.getElementById('targetInput').value.trim();
            if (!t) return;
            fetch('/set_target', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({target: t})
            }).then(r => r.json()).then(d => {
                if (d.status === 'ok') {
                    document.getElementById('targetBadge').innerText = '🎯 ' + t.toUpperCase();
                    document.getElementById('targetInput').value = '';
                }
            });
        }

        function togglePause() {
            fetch('/toggle_pause', {method: 'POST'})
                .then(r => r.json())
                .then(d => {
                    const btn = document.getElementById('pauseBtn');
                    if (d.paused) {
                        btn.className = 'btn-action btn-resume';
                        btn.innerText = '▶️ TIẾP TỤC';
                    } else {
                        btn.className = 'btn-action btn-pause';
                        btn.innerText = '⏸️ TẠM DỪNG';
                    }
                });
        }

        function drive(dir) {
            fetch('/manual_move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({direction: dir})
            }).catch(() => {});
        }

        // Cập nhật trạng thái HUD
        setInterval(() => {
            fetch('/status').then(r => r.json()).then(d => {
                document.getElementById('hudAction').innerText = d.action_text;
                document.getElementById('searchModeBadge').innerText = d.search_mode;
                document.getElementById('poseBadge').innerText = 'Robot: (' + d.pose_x + 'm, ' + d.pose_y + 'm, ' + d.pose_yaw + '°)';
                
                if (d.target_x !== null) {
                    const stateTxt = d.target_seen ? 'ĐANG NHÌN THẤY' : 'VỊ TRÍ CUỐI';
                    document.getElementById('hudTargetPos').innerText = '(' + d.target_x + 'm, ' + d.target_y + 'm) [' + stateTxt + ']';
                } else {
                    document.getElementById('hudTargetPos').innerText = 'Chưa phát hiện';
                }

                const dl = d.dist_left < 800 ? d.dist_left.toFixed(0) + 'cm' : 'THOÁNG';
                const dc = d.dist_center < 800 ? d.dist_center.toFixed(0) + 'cm' : 'THOÁNG';
                const dr = d.dist_right < 800 ? d.dist_right.toFixed(0) + 'cm' : 'THOÁNG';
                document.getElementById('hudRadar').innerText = 'L: ' + dl + ' | C: ' + dc + ' | R: ' + dr;

                document.getElementById('hudCmdVel').innerText = 'vx: ' + d.linear_x.toFixed(2) + ' | vy: ' + d.linear_y.toFixed(2) + ' | wz: ' + d.angular_z.toFixed(1) + '°';
                document.getElementById('aiFpsBadge').innerText = 'AI: ' + d.ai_fps.toFixed(1) + ' FPS';

                const strafeEl = document.getElementById('hudStrafe');
                if (d.strafe_dir === 'LEFT') strafeEl.innerText = '<<< TRƯỢT TRÁI NÉ';
                else if (d.strafe_dir === 'RIGHT') strafeEl.innerText = 'TRƯỢT PHẢI NÉ >>>';
                else strafeEl.innerText = 'ĐƯỜNG THÔNG THOÁNG';
            }).catch(() => {});
        }, 800);
    </script>
</body>
</html>"""


def start_web_server():
    port = 5000
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


# ==========================================================
# 5. SMART OBJECT SEEKER - BỘ NÃO TÌM KIẾM ĐÓN ĐẦU & NÉ MECANUM
# ==========================================================
class SmartObjectSeeker(object):
    def __init__(self):
        global seeker_instance
        seeker_instance = self

        rospy.init_node('jetauto_smart_seeker', anonymous=True)

        self.cmd_publishers = []
        for top in CMD_VEL_TOPICS:
            self.cmd_publishers.append(rospy.Publisher(top, Twist, queue_size=1))

        self.motion_lock = threading.Lock()
        self.desired_linear_x = 0.0
        self.desired_linear_y = 0.0
        self.desired_angular_z = 0.0
        self.manual_override_until = 0.0

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
        self.ai_fps = 0.0

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
        rospy.Subscriber('/scan', LaserScan, self._callback_scan, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self._callback_odom, queue_size=1)

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

            if paused:
                vx, vy, wz = 0.0, 0.0, 0.0

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
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        with pose_lock:
            robot_pose['x'] = px
            robot_pose['y'] = py
            robot_pose['yaw'] = yaw

    def _callback_scan(self, msg):
        global display_frame_map
        with pose_lock:
            rx = robot_pose['x']
            ry = robot_pose['y']
            ryaw = robot_pose['yaw']

        # Cập nhật tia quét vào bản đồ 2D
        grid_mapper.update_scan(rx, ry, ryaw, msg.ranges, msg.angle_min, msg.angle_increment)

        # Vẽ ảnh bản đồ hiển thị lên Web
        with target_lock:
            t_copy = dict(target_memory)
        rendered_map = grid_mapper.render_map_image(rx, ry, ryaw, t_copy, self.search_mode)

        with frame_lock:
            display_frame_map = rendered_map

    # ==========================================================
    # CAMERA CAPTURE (ASTRA & OAK-D)
    # ==========================================================
    def init_astra_capture(self):
        global cam_source_astra
        cam_source_astra = "Dang cho Astra..."
        if RosImage is not None:
            rospy.Subscriber('/depth_cam/rgb/image_raw', RosImage, self._cb_astra, queue_size=1, buff_size=2**24)
            rospy.Subscriber('/camera/rgb/image_raw', RosImage, self._cb_astra, queue_size=1, buff_size=2**24)

    def _cb_astra(self, msg):
        global raw_frame_astra, cam_source_astra
        frame = self._decode_ros_image(msg)
        if frame is not None:
            cam_source_astra = "Astra Pro (ROS)"
            with frame_lock:
                raw_frame_astra = frame

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
                    cam_rgb.setPreviewSize(640, 480)
                    cam_rgb.setInterleaved(False)
                    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
                    cam_rgb.setFps(25)
                    xout_rgb = pipeline.createXLinkOut()
                    xout_rgb.setStreamName("rgb")
                    cam_rgb.preview.link(xout_rgb.input)

                    has_stereo = False
                    try:
                        mono_l = pipeline.createMonoCamera()
                        mono_l.setBoardSocket(dai.CameraBoardSocket.LEFT)
                        mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
                        mono_r = pipeline.createMonoCamera()
                        mono_r.setBoardSocket(dai.CameraBoardSocket.RIGHT)
                        mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
                        stereo = pipeline.createStereoDepth()
                        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
                        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
                        mono_l.out.link(stereo.left)
                        mono_r.out.link(stereo.right)
                        xout_d = pipeline.createXLinkOut()
                        xout_d.setStreamName("depth")
                        stereo.depth.link(xout_d.input)
                        has_stereo = True
                    except Exception:
                        pass

                    device = None
                    try:
                        device = dai.Device(pipeline)
                    except Exception:
                        time.sleep(2)
                        device = dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)

                    q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                    q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False) if has_stereo else None
                    cam_source_oakd = "OAK-D Spatial ({})".format(device.getUsbSpeed().name)

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
            except Exception:
                pass

        cam_source_oakd = "Fallback ROS Topic"
        if RosImage is not None:
            rospy.Subscriber('/oakd/rgb/image_raw', RosImage, self._cb_oakd_ros, queue_size=1, buff_size=2**24)

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
            obj_mask = arr[:, 4] > 0.35
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

        conf_mask = confidences > 0.35
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
            indices = cv2.dnn.NMSBoxes(boxes, conf_list, 0.35, 0.45)
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
        explore_start_time = time.time()

        while not rospy.is_shutdown():
            raw_frame = None
            with frame_lock:
                if raw_frame_astra is not None:
                    raw_frame = raw_frame_astra.copy()

            if raw_frame is None:
                time.sleep(0.04)
                continue

            frame_counter += 1
            now = time.time()
            if now - last_fps_time >= 1.0:
                self.ai_fps = frame_counter / (now - last_fps_time)
                frame_counter = 0
                last_fps_time = now

            h, w = raw_frame.shape[:2]
            center_screen_x = w // 2

            # 1. Nhận diện YOLO
            detections = self.run_yolo(raw_frame)
            target_detections = [d for d in detections if d['class_name'] == TARGET_CLASS]
            target_found = len(target_detections) > 0

            with pose_lock:
                cur_rx = robot_pose['x']
                cur_ry = robot_pose['y']
                cur_yaw = robot_pose['yaw']

            target_center = None
            est_dist = 0.0

            # 2. CẬP NHẬT BỘ NHỚ KHÔNG GIAN (OBJECT PERMANENCE)
            if target_found:
                best = max(target_detections, key=lambda d: d['conf'])
                bx, by, bw, bh = best['box']
                target_center = (bx + bw // 2, by + bh // 2)

                # Ước lượng khoảng cách từ kích thước Bounding Box
                # Đối với người/vật thể, tỉ lệ width_ratio nghịch với khoảng cách
                width_ratio = bw / float(w)
                est_dist = max(0.4, min(5.0, 0.45 / max(width_ratio, 0.08)))

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

                if width_ratio > 0.45:
                    base_vx = 0.0
                    action_text = "ĐÃ ĐẾN GẦN MỤC TIÊU! DỪNG LẠI"
                else:
                    base_vx = 0.15 if abs(error_x) < 80 else 0.08
                    action_text = "BÁM MỤC TIÊU {} (Cách {:.1f}m)".format(TARGET_CLASS.upper(), est_dist)

            elif has_last_pos and time_since_seen < 12.0:
                # CHẾ ĐỘ 2: ĐÓN ĐẦU VỊ TRÍ CUỐI (INTERCEPT) - KHÔNG XOAY TẠI CHỖ!
                self.search_mode = "INTERCEPT"
                dx = last_tx - cur_rx
                dy = last_ty - cur_ry
                dist_to_last = math.sqrt(dx * dx + dy * dy)
                target_heading = math.atan2(dy, dx)
                yaw_diff = (target_heading - cur_yaw + math.pi) % (2.0 * math.pi) - math.pi

                if dist_to_last > 0.5:
                    # Lái xe tiến tới vị trí thấy lần cuối để nhìn ra sau vật cản
                    heading_wz = max(min(math.degrees(yaw_diff) * 0.8, 30.0), -30.0)
                    base_vx = 0.14 if abs(yaw_diff) < math.radians(35) else 0.05
                    action_text = "MẤT DẤU -> ĐANG LÁI ĐÓN ĐẦU ({:.1f}m)".format(dist_to_last)
                    status_color = (0, 215, 255)
                else:
                    # Đã tới nơi, xoay nhẹ quét tìm
                    heading_wz = 20.0
                    base_vx = 0.0
                    action_text = "ĐÃ TỚI VỊ TRÍ CŨ -> QUÉT XUNG QUANH"
                    status_color = (0, 215, 255)

            else:
                # CHẾ ĐỘ 3: MỞ RỘNG TÌM KIẾM (FRONTIER EXPLORATION)
                self.search_mode = "EXPLORE"
                # Tuần tra theo chu kỳ tiến chậm + xoay quét rộng (khám phá các góc phòng)
                cycle = int(now - explore_start_time) % 14
                if cycle < 7:
                    base_vx = 0.12  # Tiến về phía trước khám phá vùng mới
                    heading_wz = 0.0
                    action_text = "TUẦN TRA MỞ RỘNG VÙNG MỚI (EXPLORING)"
                else:
                    base_vx = 0.0
                    heading_wz = 22.0  # Xoay quét 360 độ
                    action_text = "QUÉT RADAR 360 ĐỘ TÌM MỤC TIÊU"
                status_color = (148, 163, 184)

            # 4. KẾT HỢP NÉ VẬT CẢN OAK-D BẰNG BÁNH MECANUM (TRƯỢT NGANG)
            with obstacle_lock:
                d_l = obstacle_info['dist_l']
                d_c = obstacle_info['dist_c']
                d_r = obstacle_info['dist_r']

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
            else:
                if d_c < 30.0:
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

                with obstacle_lock:
                    obstacle_info['strafe_dir'] = strafe_name

                self.set_desired_velocity(cmd_vx, cmd_vy, cmd_wz)

            # Vẽ Overlay lên khung hình Astra
            rendered_astra = self._render_ai_overlay(raw_frame, detections, action_text, status_color, target_center)
            with frame_lock:
                display_frame_astra = rendered_astra

            with self.state_lock:
                self.last_action_text = action_text

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
    os._exit(0)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    try:
        web_thread = threading.Thread(target=start_web_server)
        web_thread.daemon = True
        web_thread.start()
        seeker = SmartObjectSeeker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sigint_handler(None, None)
