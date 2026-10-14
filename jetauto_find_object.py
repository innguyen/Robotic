#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
JetAuto Pro - AI Tracker & Mecanum Obstacle Avoidance Controller
- Mắt chính (Astra Pro Plus): YOLOv5 nhận diện + khóa mục tiêu; ảnh độ sâu đo khoảng cách tới mục tiêu
  và phát hiện vật cản 3D (mặt bàn, vật thấp dưới tầm LiDAR, mép bàn / bậc thang).
- LiDAR G4: radar né vật cản 360° (góc phía sau bị khung xe che đã được tính tới).
- Bộ né vật cản (obstacle_avoidance.py): bản đồ nhớ ngắn hạn + chọn hướng kiểu VFH cho bánh Mecanum,
  trượt ngang / chéo luồn lách mà ĐẦU XE VẪN NHÌN MỤC TIÊU, chốt bên né để không lắc trái-phải.
- OAK-D: tạm tắt (USE_OAKD = False), đổi True để bật lại.
- Bộ đệm nén JPEG nền + giao diện Web: camera Astra, radar né vật cản, lái tay an toàn, chạy khô.
"""

from __future__ import print_function
import sys
import os
import time
import math
import signal
import threading
from collections import deque
import numpy as np
import cv2

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CameraInfo

import obstacle_avoidance as oa

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
# Chạy khô: vẫn tính toán và hiển thị lệnh nhưng KHÔNG gửi ra động cơ
# (bật bằng biến môi trường JETAUTO_DRY_RUN=1 hoặc nút trên web)
DRY_RUN = os.environ.get('JETAUTO_DRY_RUN', '0') == '1'
# OAK-D tạm tắt: né vật cản dùng LiDAR + độ sâu Astra. Đổi True để bật lại camera OAK-D.
USE_OAKD = False
# Node khung gầm (jetauto_odom_publisher) nghe cả /jetauto_controller/cmd_vel lẫn /cmd_vel; gửi cả hai
# làm động cơ nhận mỗi lệnh 2 lần (nhánh /cmd_vel còn bị kẹp tốc độ khác) -> chỉ gửi một topic.
CMD_VEL_TOPICS = [
    '/jetauto_controller/cmd_vel',
]
AVOID_CFG = oa.AvoidanceConfig()
NAV_RATE_HZ = 10.0
COMMAND_TIMEOUT_S = 0.4      # lệnh né vật cản cũ hơn mức này -> động cơ dừng
SENSOR_TIMEOUT_S = 0.5       # LiDAR / odom mất tín hiệu quá mức này -> coi như mất
DETECTION_CONFIDENCE = 0.35  # Vẫn vẽ detection yếu để chẩn đoán, nhưng chưa dùng để lái.
TARGET_CONFIDENCE_DEFAULT = 0.45
TARGET_CONFIDENCE = {
    'person': 0.40,
    'bottle': 0.45,
    'cup': 0.45,
    'cell phone': 0.50,
    'remote': 0.50,
}
DEPTH_SYNC_TOLERANCE_S = 0.08
# Lệnh lái tay (vx m/s, vy m/s, wz độ/s). Lùi chậm vì phía sau LiDAR bị khung xe che.
MANUAL_COMMANDS = {
    'forward': (0.15, 0.0, 0.0),
    'backward': (-0.08, 0.0, 0.0),
    'strafe_left': (0.0, 0.12, 0.0),
    'strafe_right': (0.0, -0.12, 0.0),
    'left': (0.0, 0.0, 35.0),
    'right': (0.0, 0.0, -35.0),
}

# Khung hình toàn cục cho cả 2 Camera
raw_frame_astra = None
raw_frame_oakd = None
depth_frame_oakd = None
display_frame_astra = None  # Astra: Bounding Box AI & Khóa mục tiêu
display_frame_oakd = None   # Khung thứ 2: radar né vật cản (hoặc OAK-D nếu USE_OAKD)
frame_lock = threading.Lock()

# Bộ đệm JPEG cache nén sẵn trên RAM (chống nghẽn socket và khóa GIL của Python)
cached_jpeg_astra = None
cached_jpeg_oakd = None
cached_jpeg_combined = None
jpeg_lock = threading.Lock()
combined_last_request = 0.0  # chỉ nén khung ghép khi có người đang xem

# Khoảng trống quanh xe (cm) từ bộ né vật cản LiDAR + Astra (giữ tên cũ cho giao diện)
obstacle_info = {
    'dist_l': 999.0,
    'dist_c': 999.0,
    'dist_r': 999.0,
    'status': 'CHO LIDAR',
    'strafe_dir': 'NONE'
}
obstacle_lock = threading.Lock()

seeker_instance = None
cam_source_astra = "Khoi tao..."
cam_source_oakd = "Khoi tao..."


# ==========================================================
# CÁC HÀM XỬ LÝ KHUNG HÌNH VÀ BỘ ĐỆM JPEG
# ==========================================================
def get_frame_for_cam(cam_name='astra'):
    """Lấy khung hình cho từng camera cụ thể hoặc khung hình ghép."""
    if cam_name == 'combined':
        return get_combined_frame()

    with frame_lock:
        if cam_name == 'astra':
            if display_frame_astra is not None:
                return display_frame_astra.copy()
            if raw_frame_astra is not None:
                return raw_frame_astra.copy()
            title = "ASTRA PRO PLUS (AI TARGET TRACKER)"
            status = cam_source_astra
        else:
            if display_frame_oakd is not None:
                return display_frame_oakd.copy()
            if raw_frame_oakd is not None:
                return raw_frame_oakd.copy()
            title = "LUXONIS OAK-D (OBSTACLE RADAR)" if USE_OAKD else "RADAR NE VAT CAN (LIDAR + ASTRA)"
            status = cam_source_oakd

    f = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(f, (0, 0), (640, 480), (15, 23, 42), -1)
    cv2.putText(f, title, (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 229, 255), 2)
    cv2.putText(f, "DANG CHO TIN HIEU CAMERA...", (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
    cv2.putText(f, status, (30, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (148, 163, 184), 1)
    return f


def get_combined_frame():
    """Ghép 2 camera thành 1 khung hình Side-by-Side (Astra bên trái, OAK-D bên phải)."""
    f1 = get_frame_for_cam('astra')
    f2 = get_frame_for_cam('oakd')
    f1_s = cv2.resize(f1, (480, 360))
    f2_s = cv2.resize(f2, (480, 360))
    combined = np.hstack([f1_s, f2_s])
    # Vẽ đường phân cách
    cv2.line(combined, (480, 0), (480, 360), (0, 229, 255), 2)
    return combined


def jpeg_encoder_loop():
    """Thread chuyên trách nén JPEG sẵn vào RAM ở tốc độ ~22 FPS (chỉ nén lại khi ảnh đổi)."""
    global cached_jpeg_astra, cached_jpeg_oakd, cached_jpeg_combined
    last_astra = last_oakd = None
    while not rospy.is_shutdown():
        try:
            # 1. Nén ảnh Astra
            f_astra = None
            with frame_lock:
                if display_frame_astra is not None:
                    f_astra = display_frame_astra
                elif raw_frame_astra is not None:
                    f_astra = raw_frame_astra

            if f_astra is not None and f_astra is not last_astra:
                ret, jpg = cv2.imencode('.jpg', f_astra, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                if ret:
                    raw_bytes = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()
                    with jpeg_lock:
                        cached_jpeg_astra = raw_bytes
                    last_astra = f_astra

            # 2. Nén ảnh khung thứ 2 (radar né vật cản / OAK-D)
            f_oakd = None
            with frame_lock:
                if display_frame_oakd is not None:
                    f_oakd = display_frame_oakd
                elif raw_frame_oakd is not None:
                    f_oakd = raw_frame_oakd

            if f_oakd is not None and f_oakd is not last_oakd:
                ret, jpg = cv2.imencode('.jpg', f_oakd, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                if ret:
                    raw_bytes = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()
                    with jpeg_lock:
                        cached_jpeg_oakd = raw_bytes
                    last_oakd = f_oakd

            # 3. Nén ảnh Ghép Side-by-Side (chỉ khi có người xem trong 5 giây gần đây)
            if (f_astra is not None or f_oakd is not None) and time.time() - combined_last_request < 5.0:
                comb = get_combined_frame()
                ret, jpg = cv2.imencode('.jpg', comb, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                if ret:
                    raw_bytes = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()
                    with jpeg_lock:
                        cached_jpeg_combined = raw_bytes

        except Exception:
            pass

        time.sleep(0.045)


# ==========================================================
# THUẬT TOÁN ĐO KHOẢNG CÁCH VẬT CẢN TỪ STEREO DEPTH OAK-D
# ==========================================================
def process_obstacle_depth(depth_frame):
    """
    Phân tích ma trận độ sâu OAK-D (đơn vị mm, uint16).
    Chia nửa dưới khung hình (mặt sàn) thành 3 vùng: Trái, Giữa, Phải.
    Trả về: (dist_l_cm, dist_c_cm, dist_r_cm)
    """
    if depth_frame is None or depth_frame.size == 0:
        return 999.0, 999.0, 999.0

    h, w = depth_frame.shape[:2]
    # Chỉ quét khu vực từ 35% đến 90% chiều cao (vùng có vật cản gầm xe)
    roi = depth_frame[int(h * 0.35):int(h * 0.90), :]

    col_w = w // 3
    zone_l = roi[:, :col_w]
    zone_c = roi[:, col_w:col_w * 2]
    zone_r = roi[:, col_w * 2:]

    def calc_dist(z):
        # Lấy các điểm trong tầm đo hiệu dụng từ 15cm đến 3m
        valid = z[(z > 150) & (z < 3000)]
        if len(valid) < 60:
            return 999.0  # Thông thoáng
        # Dùng phân vị thứ 10 để loại trừ pixel nhiễu
        return float(np.percentile(valid, 10)) / 10.0  # mm sang cm

    return calc_dist(zone_l), calc_dist(zone_c), calc_dist(zone_r)


def render_oakd_hud(frame, dl, dc, dr, strafe_dir="NONE"):
    """Vẽ giao diện radar và thước đo khoảng cách 3 vùng lên video OAK-D."""
    h, w = frame.shape[:2]
    out = frame.copy()
    col_w = w // 3
    y_start = int(h * 0.42)
    y_end = int(h * 0.94)

    def get_color(dist):
        if dist < 30.0:
            return (0, 0, 255)      # Đỏ: Nguy hiểm (< 30cm)
        elif dist < 60.0:
            return (0, 215, 255)    # Vàng: Cảnh báo (30-60cm)
        else:
            return (0, 255, 0)      # Xanh: An toàn (> 60cm)

    # Vẽ 3 khung radar
    zones = [
        (0, col_w, dl, "TRAI"),
        (col_w, col_w * 2, dc, "GIUA"),
        (col_w * 2, w, dr, "PHAI")
    ]
    for x1, x2, dist, name in zones:
        c = get_color(dist)
        cv2.rectangle(out, (x1 + 4, y_start), (x2 - 4, y_end), c, 2)
        txt = "{}: {:.0f}cm".format(name, dist) if dist < 800 else "{}: THOANG".format(name)
        cv2.putText(out, txt, (x1 + 10, y_end - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2)

    # Thanh trạng thái phía trên
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
# WEB SERVER & REQUEST HANDLERS
# ==========================================================
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request, client_address):
        # Trình duyệt đóng tab / đổi luồng giữa chừng là chuyện bình thường, không in traceback
        if isinstance(sys.exc_info()[1], (BrokenPipeError, ConnectionResetError)):
            return
        HTTPServer.handle_error(self, request, client_address)


class CamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def _stream_mjpeg(self, cam_name='astra'):
        global combined_last_request
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        try:
            while True:
                if cam_name == 'combined':
                    combined_last_request = time.time()
                raw = None
                with jpeg_lock:
                    if cam_name == 'astra' and cached_jpeg_astra is not None:
                        raw = cached_jpeg_astra
                    elif cam_name == 'oakd' and cached_jpeg_oakd is not None:
                        raw = cached_jpeg_oakd
                    elif cam_name == 'combined' and cached_jpeg_combined is not None:
                        raw = cached_jpeg_combined

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

    def _serve_snapshot(self, cam_name='astra'):
        global combined_last_request
        if cam_name == 'combined':
            combined_last_request = time.time()
        raw = None
        with jpeg_lock:
            if cam_name == 'astra' and cached_jpeg_astra is not None:
                raw = cached_jpeg_astra
            elif cam_name == 'oakd' and cached_jpeg_oakd is not None:
                raw = cached_jpeg_oakd
            elif cam_name == 'combined' and cached_jpeg_combined is not None:
                raw = cached_jpeg_combined

        if raw is None:
            frame = get_frame_for_cam(cam_name)
            ret, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ret:
                raw = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()

        if raw is not None:
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(raw)
        else:
            self.send_response(500)
            self.end_headers()

    def _send_json(self, data):
        import json
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def do_POST(self):
        global TARGET_CLASS, IS_PAUSED, DRY_RUN, seeker_instance
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
                if seeker_instance:
                    seeker_instance.reset_target(new_target)
                rospy.loginfo(">> [WEB] Doi muc tieu sang: " + TARGET_CLASS)
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status": "ok"}')
                return

        elif self.path == '/toggle_pause':
            IS_PAUSED = not IS_PAUSED
            rospy.loginfo(">> [WEB] Trang thai tam dung: " + str(IS_PAUSED))
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({'paused': IS_PAUSED}).encode())
            return

        # Lệnh trạng thái rõ ràng: gửi trùng hay trễ vẫn không biến "dừng" thành "chạy"
        elif self.path in ('/pause', '/stop'):
            IS_PAUSED = True
            if seeker_instance:
                seeker_instance.manual_move('stop')
            rospy.loginfo(">> [WEB] Dung robot theo lenh")
            self._send_json({'paused': True})
            return

        elif self.path == '/resume':
            IS_PAUSED = False
            rospy.loginfo(">> [WEB] Tiep tuc tu dong")
            self._send_json({'paused': False})
            return

        elif self.path == '/toggle_dry_run':
            DRY_RUN = not DRY_RUN
            rospy.loginfo(">> [WEB] Chay kho (khong gui lenh dong co): " + str(DRY_RUN))
            self._send_json({'dry_run': DRY_RUN})
            return

        elif self.path == '/calibrate_astra':
            if seeker_instance:
                seeker_instance.request_astra_calibration()
            self._send_json({'status': 'ok', 'msg': 'Dang hieu chuan mat san Astra - de robot nhin thay san truoc mat'})
            return

        elif self.path == '/test_motors':
            if seeker_instance:
                seeker_instance.trigger_motor_test()
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status": "ok", "msg": "Dang test dong co 3-DOF trong 2 giay!"}')
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
        if self.path.startswith('/cam_combined') or self.path.startswith('/video_feed_combined'):
            self._stream_mjpeg('combined')
        elif self.path.startswith('/cam_oakd') or self.path.startswith('/video_feed_oakd'):
            self._stream_mjpeg('oakd')
        elif self.path.startswith('/cam_astra') or self.path.startswith('/video_feed_astra') or self.path.startswith('/cam.mjpg'):
            self._stream_mjpeg('astra')
        elif self.path.startswith('/snapshot_combined'):
            self._serve_snapshot('combined')
        elif self.path.startswith('/snapshot_oakd'):
            self._serve_snapshot('oakd')
        elif self.path.startswith('/snapshot_astra') or self.path.startswith('/snapshot'):
            self._serve_snapshot('astra')
        elif self.path.startswith('/status'):
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            with obstacle_lock:
                obs_copy = dict(obstacle_info)

            status_data = {
                'target': TARGET_CLASS,
                'paused': IS_PAUSED,
                'ai_fps': round(seeker_instance.ai_fps, 1) if seeker_instance else 0.0,
                'action_text': seeker_instance.last_action_text if seeker_instance else "CHO KHOI DONG",
                'source_astra': cam_source_astra,
                'source_oakd': cam_source_oakd,
                'linear_x': round(seeker_instance.desired_linear_x, 2) if seeker_instance else 0.0,
                'linear_y': round(seeker_instance.desired_linear_y, 2) if seeker_instance else 0.0,
                'angular_z': round(seeker_instance.desired_angular_z, 1) if seeker_instance else 0.0,
                'dist_left': round(obs_copy['dist_l'], 1),
                'dist_center': round(obs_copy['dist_c'], 1),
                'dist_right': round(obs_copy['dist_r'], 1),
                'strafe_dir': obs_copy['strafe_dir'],
                'obs_status': obs_copy['status'],
                'dry_run': DRY_RUN,
            }
            if seeker_instance:
                status_data.update(seeker_instance.nav_status())
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
    <title>JetAuto Dual Vision - Mecanum Tracker</title>
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
            max-width: 1320px;
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

        /* TOOLBAR TOP */
        .top-toolbar {
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: rgba(15, 23, 42, 0.85);
            border: 1px solid var(--glass-border);
            padding: 10px 18px;
            border-radius: 16px;
            flex-wrap: wrap;
            gap: 12px;
        }
        .group-left, .group-right { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }

        .btn-toggle {
            background: rgba(255, 255, 255, 0.06);
            border: 1px solid rgba(255, 255, 255, 0.1);
            color: #cbd5e1;
            padding: 8px 14px;
            border-radius: 10px;
            font-weight: 600;
            font-size: 0.82rem;
            cursor: pointer;
            transition: all 0.2s;
        }
        .btn-toggle:hover { background: rgba(255, 255, 255, 0.12); color: white; }
        .btn-toggle.active {
            background: linear-gradient(135deg, #00E5FF, #3B82F6);
            color: #0b1120;
            border-color: transparent;
            font-weight: 800;
        }

        /* CAMERA GRIDS */
        .camera-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 1.2rem;
            width: 100%;
        }
        @media (max-width: 860px) {
            .camera-grid { grid-template-columns: 1fr; }
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
            max-height: 460px;
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
        .btn-test { background: rgba(255, 255, 255, 0.1); color: #fff; border: 1px solid var(--glass-border); }
        .btn-stop { background: #EF4444; color: #fff; }
        .btn-dry-on { background: #F59E0B; color: #000; }
        .legend { display: flex; flex-wrap: wrap; gap: 10px; padding: 8px 12px; font-size: 0.72rem; color: #94a3b8; border-top: 1px solid var(--glass-border); }
        .legend i { display: inline-block; width: 10px; height: 10px; border-radius: 2px; margin-right: 5px; vertical-align: -1px; }

        .tags-bar { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
        .tag-btn {
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid rgba(255, 255, 255, 0.1);
            color: #cbd5e1;
            padding: 5px 12px;
            border-radius: 16px;
            font-size: 0.8rem;
            cursor: pointer;
        }
        .tag-btn:hover { background: rgba(0, 229, 255, 0.2); border-color: var(--primary); color: #fff; }

        /* BOTTOM HUD & D-PAD */
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

        /* MECANUM D-PAD */
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
            transition: all 0.15s;
            user-select: none;
            touch-action: none;
            text-align: center;
        }
        .dpad-btn:active { background: var(--primary); color: #000; }
        .dpad-strafe { background: rgba(56, 189, 248, 0.12); border-color: rgba(56, 189, 248, 0.3); color: #38bdf8; font-size: 0.78rem; }
        .dpad-stop { background: rgba(239, 68, 68, 0.2); border-color: rgba(239, 68, 68, 0.4); color: #ef4444; }
    </style>
</head>
<body>
    <div class="header">
        <h1>JetAuto Dual Vision <span class="target-badge" id="targetBadge">🎯 """ + TARGET_CLASS.upper() + """</span></h1>
        <p>Cam Astra: YOLO Khóa Mục Tiêu + Độ Sâu | LiDAR G4 + Astra: Né Vật Cản Mecanum (đầu xe vẫn nhìn mục tiêu)</p>
    </div>

    <div class="container">
        <!-- TOP TOOLBAR -->
        <div class="top-toolbar">
            <div class="group-left">
                <span style="color: #64748b; font-weight: 700; font-size: 0.8rem; text-transform: uppercase;">Bố cục:</span>
                <button class="btn-toggle active" id="btnLayoutDual" onclick="setLayout('dual')">⊞ 2 Màn Hình (Cards)</button>
                <button class="btn-toggle" id="btnLayoutCombined" onclick="setLayout('combined')">🔲 Ghép 1 Luồng (Unified)</button>
                <button class="btn-toggle" id="btnLayoutAstra" onclick="setLayout('astra')">🔲 Chỉ Cam Astra</button>
                <button class="btn-toggle" id="btnLayoutOakd" onclick="setLayout('oakd')">🔲 Chỉ Radar</button>
            </div>
            <div class="group-right">
                <span style="color: #64748b; font-weight: 700; font-size: 0.8rem; text-transform: uppercase;">Chế độ:</span>
                <button class="btn-toggle active" id="btnFeedStream" onclick="setFeedMode('stream')">📡 Stream Video</button>
                <button class="btn-toggle" id="btnFeedSnapshot" onclick="setFeedMode('snapshot')">⚡ Snapshot</button>
            </div>
        </div>

        <!-- DUAL CAMERA GRID -->
        <div class="camera-grid" id="cameraGrid">
            <!-- CAM 1: ASTRA PRO PLUS (YOLO AI) -->
            <div class="stream-card" id="cardAstra">
                <div class="stream-header">
                    <span style="color: var(--primary);">🟢 ASTRA PRO PLUS (YOLO Khóa Mục Tiêu)</span>
                    <span style="color: var(--success);" id="feedStatusAstra">● AI Active</span>
                </div>
                <div class="stream-box">
                    <img id="feedAstra" src="/cam_astra.mjpg" alt="Astra Feed" />
                </div>
            </div>

            <!-- KHUNG 2: RADAR NÉ VẬT CẢN (LiDAR + Astra, nhìn từ trên xuống, đầu xe hướng lên) -->
            <div class="stream-card" id="cardOakd">
                <div class="stream-header">
                    <span style="color: #38bdf8;">🧭 RADAR NÉ VẬT CẢN (LiDAR + Astra)</span>
                    <span style="color: var(--success);" id="feedStatusOakd">● Đang khởi động</span>
                </div>
                <div class="stream-box">
                    <img id="feedOakd" src="/cam_oakd.mjpg" alt="Radar ne vat can" />
                </div>
                <div class="legend">
                    <span><i style="background:#00dcff"></i>LiDAR</span>
                    <span><i style="background:#ff8c00"></i>Astra</span>
                    <span><i style="background:#ff00ff"></i>Mép / hố</span>
                    <span><i style="background:#20283a"></i>Đã thấy trống</span>
                    <span><i style="background:#6e2828"></i>Góc mù LiDAR</span>
                    <span><i style="background:#ffff00"></i>Hướng đi</span>
                </div>
            </div>
        </div>

        <!-- UNIFIED COMBINED VIEW -->
        <div class="stream-card" id="cardCombined" style="display: none; width: 100%;">
            <div class="stream-header">
                <span style="color: var(--primary);">⊞ GHÉP CAMERA ASTRA + RADAR NÉ VẬT CẢN</span>
                <span style="color: var(--success);">● 0% Conflict Stream</span>
            </div>
            <div class="stream-box">
                <img id="feedCombined" src="/cam_combined.mjpg" alt="Combined Feed" />
            </div>
        </div>

        <!-- QUICK CONTROLS -->
        <div class="control-row">
            <input type="text" id="targetInput" placeholder="Nhập tên vật thể cần tìm (person, bottle, cup, chair, cell phone...)" onkeypress="if(event.key==='Enter') setTarget()" />
            <button class="btn-action btn-find" onclick="setTarget()">🔍 TÌM KIẾM</button>
            <button id="pauseBtn" class="btn-action btn-pause" onclick="togglePause()">⏸️ TẠM DỪNG</button>
            <button class="btn-action btn-stop" onclick="emergencyStop()">🛑 DỪNG KHẨN</button>
            <button id="dryRunBtn" class="btn-action btn-test" onclick="toggleDryRun()" title="Tính toán và hiển thị lệnh nhưng không gửi ra động cơ">🧪 CHẠY KHÔ: TẮT</button>
            <button class="btn-action btn-test" onclick="calibrateAstra()" title="Đặt robot nhìn thấy sàn trống phía trước rồi bấm">📐 HIỆU CHUẨN SÀN</button>
            <button class="btn-action btn-test" onclick="testMotors()">🛠️ TEST MECANUM 3-DOF</button>
        </div>

        <!-- QUICK TAGS -->
        <div class="tags-bar">
            <span style="font-size: 0.8rem; color: #64748b; font-weight: 700;">Gợi ý:</span>
            <button class="tag-btn" onclick="quickSelect('person')">👤 person</button>
            <button class="tag-btn" onclick="quickSelect('bottle')">🍾 bottle</button>
            <button class="tag-btn" onclick="quickSelect('cup')">☕ cup</button>
            <button class="tag-btn" onclick="quickSelect('cell phone')">📱 cell phone</button>
            <button class="tag-btn" onclick="quickSelect('chair')">🪑 chair</button>
            <button class="tag-btn" onclick="quickSelect('backpack')">🎒 backpack</button>
            <button class="tag-btn" onclick="quickSelect('sports ball')">⚽ ball</button>
        </div>

        <!-- BOTTOM GRID: STATUS HUD & D-PAD -->
        <div class="bottom-grid">
            <div class="info-card">
                <div class="card-title">
                    <span>Trạng thái Hệ thống & Né Vật Cản</span>
                    <span id="aiFpsBadge" style="color: var(--primary);">AI: 0.0 FPS</span>
                </div>
                <div class="hud-grid">
                    <div class="hud-item" style="grid-column: span 2;">
                        <div class="hud-label">Hành động Robot</div>
                        <div class="hud-val" id="hudAction" style="color: var(--warning);">ĐANG KHỞI TẠO...</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Khoảng trống quanh xe (Trái | Trước | Phải)</div>
                        <div class="hud-val" id="hudRadar" style="color: var(--success);">L: -- | C: -- | R: --</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Chiến thuật Mecanum</div>
                        <div class="hud-val" id="hudStrafe" style="color: #38bdf8;">--</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Lệnh Vận Tốc 3-DOF (10Hz)</div>
                        <div class="hud-val" id="hudCmdVel">vx: 0.00 | vy: 0.00 | wz: 0.0°</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Mục tiêu</div>
                        <div class="hud-val" id="hudTarget" style="color: #94a3b8;">Chưa thấy</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Cảm biến</div>
                        <div class="hud-val" id="hudSources" style="font-size: 0.8rem; color: #94a3b8;">--</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Hiệu chuẩn Astra (mặt sàn)</div>
                        <div class="hud-val" id="hudCalib" style="font-size: 0.8rem; color: #94a3b8;">--</div>
                    </div>
                </div>
            </div>

            <!-- MECANUM D-PAD MANUAL DRIVE -->
            <div class="dpad-card">
                <div class="card-title">
                    <span>Lái thủ công Bánh Mecanum (W,A,S,D + Q,E trượt)</span>
                </div>
                <div class="dpad-layout">
                    <div></div>
                    <button class="dpad-btn" data-drive="forward">▲ TIẾN</button>
                    <div></div>

                    <button class="dpad-btn dpad-strafe" data-drive="strafe_left">◄◄ TRƯỢT T</button>
                    <button class="dpad-btn dpad-stop" data-drive="stop">DỪNG</button>
                    <button class="dpad-btn dpad-strafe" data-drive="strafe_right">TRƯỢT P ►►</button>

                    <button class="dpad-btn" data-drive="left">⟲ XOAY T</button>
                    <button class="dpad-btn" data-drive="backward">▼ LÙI</button>
                    <button class="dpad-btn" data-drive="right">XOAY P ⟳</button>
                </div>
                <p style="font-size: 0.72rem; color: #64748b; text-align: center; margin-top: 10px;">Nhấn giữ để chạy, nhả ra là dừng. Lệnh lái tay vẫn được kiểm tra va chạm.</p>
            </div>
        </div>
    </div>

    <script>
        let currentLayout = 'dual';
        let currentFeedMode = 'stream';
        let snapshotTimer = null;

        function setLayout(mode) {
            currentLayout = mode;
            const grid = document.getElementById('cameraGrid');
            const cardAstra = document.getElementById('cardAstra');
            const cardOakd = document.getElementById('cardOakd');
            const cardCombined = document.getElementById('cardCombined');

            document.getElementById('btnLayoutDual').className = 'btn-toggle ' + (mode === 'dual' ? 'active' : '');
            document.getElementById('btnLayoutCombined').className = 'btn-toggle ' + (mode === 'combined' ? 'active' : '');
            document.getElementById('btnLayoutAstra').className = 'btn-toggle ' + (mode === 'astra' ? 'active' : '');
            document.getElementById('btnLayoutOakd').className = 'btn-toggle ' + (mode === 'oakd' ? 'active' : '');

            if (mode === 'dual') {
                grid.style.display = 'grid';
                grid.style.gridTemplateColumns = '1fr 1fr';
                cardAstra.style.display = 'block';
                cardOakd.style.display = 'block';
                cardCombined.style.display = 'none';
            } else if (mode === 'combined') {
                grid.style.display = 'none';
                cardCombined.style.display = 'block';
            } else if (mode === 'astra') {
                grid.style.display = 'grid';
                grid.style.gridTemplateColumns = '1fr';
                cardAstra.style.display = 'block';
                cardOakd.style.display = 'none';
                cardCombined.style.display = 'none';
            } else if (mode === 'oakd') {
                grid.style.display = 'grid';
                grid.style.gridTemplateColumns = '1fr';
                cardAstra.style.display = 'none';
                cardOakd.style.display = 'block';
                cardCombined.style.display = 'none';
            }
        }

        function setFeedMode(mode) {
            currentFeedMode = mode;
            document.getElementById('btnFeedStream').className = 'btn-toggle ' + (mode === 'stream' ? 'active' : '');
            document.getElementById('btnFeedSnapshot').className = 'btn-toggle ' + (mode === 'snapshot' ? 'active' : '');
            
            const imgAstra = document.getElementById('feedAstra');
            const imgOakd = document.getElementById('feedOakd');
            const imgCombined = document.getElementById('feedCombined');

            if (mode === 'snapshot') {
                startSnapshotLoop();
            } else {
                stopSnapshotLoop();
                imgAstra.src = '/cam_astra.mjpg?t=' + Date.now();
                imgOakd.src = '/cam_oakd.mjpg?t=' + Date.now();
                imgCombined.src = '/cam_combined.mjpg?t=' + Date.now();
            }
        }

        function startSnapshotLoop() {
            stopSnapshotLoop();
            function poll() {
                const now = Date.now();
                if (currentLayout === 'combined') {
                    const imgCombined = document.getElementById('feedCombined');
                    const nC = new Image();
                    nC.onload = () => { imgCombined.src = nC.src; };
                    nC.src = '/snapshot_combined.jpg?t=' + now;
                } else {
                    const imgAstra = document.getElementById('feedAstra');
                    const imgOakd = document.getElementById('feedOakd');
                    if (currentLayout === 'dual' || currentLayout === 'astra') {
                        const nA = new Image();
                        nA.onload = () => { imgAstra.src = nA.src; };
                        nA.src = '/snapshot_astra.jpg?t=' + now;
                    }
                    if (currentLayout === 'dual' || currentLayout === 'oakd') {
                        const nO = new Image();
                        nO.onload = () => { imgOakd.src = nO.src; };
                        nO.src = '/snapshot_oakd.jpg?t=' + now;
                    }
                }
                snapshotTimer = setTimeout(poll, 120);
            }
            poll();
        }

        function stopSnapshotLoop() {
            if (snapshotTimer) {
                clearTimeout(snapshotTimer);
                snapshotTimer = null;
            }
        }

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

        function quickSelect(t) {
            document.getElementById('targetInput').value = t;
            setTarget();
        }

        let lastPaused = false;
        function togglePause() {
            // Gửi trạng thái rõ ràng: bấm đúp hay mạng trễ cũng không biến "dừng" thành "chạy"
            fetch(lastPaused ? '/resume' : '/pause', {method: 'POST'})
                .then(r => r.json())
                .then(d => updatePauseUI(d.paused));
        }

        function emergencyStop() {
            stopDrive();
            fetch('/pause', {method: 'POST'}).then(r => r.json()).then(d => updatePauseUI(d.paused)).catch(() => {});
        }

        function updateDryRunUI(dry) {
            const btn = document.getElementById('dryRunBtn');
            btn.className = 'btn-action ' + (dry ? 'btn-dry-on' : 'btn-test');
            btn.innerText = dry ? '🧪 CHẠY KHÔ: BẬT (động cơ đứng yên)' : '🧪 CHẠY KHÔ: TẮT';
        }

        function toggleDryRun() {
            fetch('/toggle_dry_run', {method: 'POST'}).then(r => r.json()).then(d => updateDryRunUI(d.dry_run)).catch(() => {});
        }

        function calibrateAstra() {
            fetch('/calibrate_astra', {method: 'POST'})
                .then(r => r.json())
                .then(d => alert(">> " + d.msg))
                .catch(e => alert("Lỗi kết nối: " + e));
        }

        function updatePauseUI(paused) {
            lastPaused = paused;
            const btn = document.getElementById('pauseBtn');
            if (paused) {
                btn.className = 'btn-action btn-resume';
                btn.innerText = '▶️ TIẾP TỤC';
            } else {
                btn.className = 'btn-action btn-pause';
                btn.innerText = '⏸️ TẠM DỪNG';
            }
        }

        function testMotors() {
            fetch('/test_motors', {method: 'POST'})
                .then(r => r.json())
                .then(d => alert(">> " + d.msg))
                .catch(e => alert("Lỗi kết nối: " + e));
        }

        // Lái tay: giữ nút / phím thì gửi lặp lại mỗi 150 ms, nhả ra là dừng ngay
        // (robot tự dừng nếu 0.5 s không nhận lệnh, nên rớt mạng cũng không chạy mãi)
        let driveTimer = null, driveDir = null;
        function sendDrive(dir) {
            return fetch('/manual_move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({direction: dir})
            }).catch(() => {});
        }
        function startDrive(dir) {
            if (dir === 'stop') { stopDrive(); return; }
            if (driveDir === dir) return;
            if (driveTimer) clearInterval(driveTimer);
            driveDir = dir;
            sendDrive(dir);
            driveTimer = setInterval(() => sendDrive(dir), 150);
        }
        function stopDrive() {
            if (driveTimer) clearInterval(driveTimer);
            driveTimer = null;
            driveDir = null;
            sendDrive('stop');
        }
        document.querySelectorAll('[data-drive]').forEach(btn => {
            const dir = btn.dataset.drive;
            btn.addEventListener('pointerdown', e => { e.preventDefault(); btn.setPointerCapture(e.pointerId); startDrive(dir); });
            ['pointerup', 'pointercancel', 'lostpointercapture'].forEach(ev =>
                btn.addEventListener(ev, () => { if (dir !== 'stop' && driveDir === dir) stopDrive(); }));
            btn.addEventListener('contextmenu', e => e.preventDefault());
        });

        // Bàn phím: W/S tiến lùi, A/D xoay, Q/E trượt ngang, Space dừng
        const KEYS = {ArrowUp: 'forward', w: 'forward', W: 'forward', ArrowDown: 'backward', s: 'backward', S: 'backward',
                      q: 'strafe_left', Q: 'strafe_left', e: 'strafe_right', E: 'strafe_right',
                      ArrowLeft: 'left', a: 'left', A: 'left', ArrowRight: 'right', d: 'right', D: 'right'};
        window.addEventListener('keydown', e => {
            if (['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
            if (e.key === ' ') { e.preventDefault(); stopDrive(); return; }
            const dir = KEYS[e.key];
            if (dir) { e.preventDefault(); startDrive(dir); }
        });
        window.addEventListener('keyup', e => {
            if (['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
            if (KEYS[e.key] && KEYS[e.key] === driveDir) stopDrive();
        });
        window.addEventListener('blur', () => { if (driveDir) stopDrive(); });

        const NAV_TEXT = {
            BAM: ['BÁM THẲNG MỤC TIÊU', 'var(--success)'],
            NE_TRAI: ['<<< LÁCH TRÁI (đầu vẫn nhìn mục tiêu)', 'var(--warning)'],
            NE_PHAI: ['LÁCH PHẢI >>> (đầu vẫn nhìn mục tiêu)', 'var(--warning)'],
            NHO_BAM: ['MỤC TIÊU BỊ CHE · ĐI THEO VỊ TRÍ NHỚ', '#38bdf8'],
            NHO_NE_TRAI: ['BỊ CHE · LÁCH TRÁI THEO VỊ TRÍ NHỚ', '#38bdf8'],
            NHO_NE_PHAI: ['BỊ CHE · LÁCH PHẢI THEO VỊ TRÍ NHỚ', '#38bdf8'],
            DA_TOI: ['ĐÃ TỚI GẦN MỤC TIÊU · DỪNG', 'var(--success)'],
            BI_CHAN: ['BỊ CHẶN · CHỜ LỐI ĐI AN TOÀN', 'var(--danger)'],
            NHIN_QUANH: ['XOAY NHÌN VÙNG LIDAR CHƯA THẤY', '#38bdf8'],
            DUNG_AN_TOAN: ['DỪNG AN TOÀN (sắp va chạm)', 'var(--danger)'],
            TIM_XOAY: ['QUAY QUÉT TÌM MỤC TIÊU', '#94a3b8'],
            TIM_DI: ['ĐI TỚI CHỖ THOÁNG ĐỂ TÌM', '#94a3b8'],
            TIM_CHO: ['CHỜ LỐI ĐI ĐỂ TÌM TIẾP', 'var(--warning)'],
            TAM_DUNG: ['TẠM DỪNG', 'var(--warning)'],
            LAI_TAY: ['LÁI TAY', 'var(--primary)'],
            LAI_TAY_CHAN: ['LÁI TAY BỊ CHẶN (vật cản)', 'var(--danger)'],
            MAT_LIDAR: ['MẤT LIDAR · DỪNG XE', 'var(--danger)'],
            MAT_CAMERA: ['MẤT CAMERA · DỪNG XE', 'var(--danger)'],
            KHOI_DONG: ['ĐANG KHỞI ĐỘNG', 'var(--warning)']
        };
        const age = v => (v === null || v === undefined) ? 'MẤT' : (v < 1.0 ? 'OK' : v.toFixed(1) + 's');

        // Cập nhật trạng thái HUD mỗi giây
        setInterval(() => {
            fetch('/status').then(r => r.json()).then(d => {
                document.getElementById('hudAction').innerText = d.action_text;

                // Khoảng trống quanh xe theo 3 hướng
                const dl = d.dist_left < 800 ? d.dist_left.toFixed(0) + 'cm' : 'THOÁNG';
                const dc = d.dist_center < 800 ? d.dist_center.toFixed(0) + 'cm' : 'THOÁNG';
                const dr = d.dist_right < 800 ? d.dist_right.toFixed(0) + 'cm' : 'THOÁNG';
                const radarEl = document.getElementById('hudRadar');
                radarEl.innerText = 'L: ' + dl + ' | C: ' + dc + ' | R: ' + dr;
                if (d.obs_status === 'NGUY HIEM' || d.obs_status === 'MAT LIDAR') radarEl.style.color = 'var(--danger)';
                else if (d.obs_status === 'CANH BAO') radarEl.style.color = 'var(--warning)';
                else radarEl.style.color = 'var(--success)';

                // Chiến thuật né
                const nav = NAV_TEXT[d.nav_state] || [d.nav_state || '--', '#94a3b8'];
                const strafeEl = document.getElementById('hudStrafe');
                strafeEl.innerText = nav[0];
                strafeEl.style.color = nav[1];
                const feedEl = document.getElementById('feedStatusOakd');
                feedEl.innerText = '● ' + nav[0];
                feedEl.style.color = nav[1];

                document.getElementById('hudCmdVel').innerText = 'vx: ' + d.linear_x.toFixed(2) + ' | vy: ' + d.linear_y.toFixed(2) +
                    ' | wz: ' + d.angular_z.toFixed(1) + '°' + (d.dry_run ? '  (CHẠY KHÔ)' : '');

                const tgt = document.getElementById('hudTarget');
                if (d.target_visible) {
                    tgt.innerText = 'Đang thấy · lệch ' + d.target_bearing_deg.toFixed(0) + '°' +
                        (d.target_distance_m !== null ? ' · cách ' + d.target_distance_m.toFixed(2) + ' m (' + d.target_source + ')' : '');
                    tgt.style.color = 'var(--success)';
                } else if (d.target_remembered) {
                    tgt.innerText = 'Bị che · nhớ hướng ' + d.target_bearing_deg.toFixed(0) + '°';
                    tgt.style.color = '#38bdf8';
                } else {
                    tgt.innerText = 'Chưa thấy';
                    tgt.style.color = '#94a3b8';
                }

                document.getElementById('hudSources').innerText = 'Astra ' + age(d.camera_age) + ' · Độ sâu ' + age(d.depth_age) +
                    ' · LiDAR ' + age(d.lidar_age) + ' · Odom ' + age(d.odom_age) + ' · Vòng né ' + (d.nav_loop_ms || 0).toFixed(0) + ' ms';
                document.getElementById('hudCalib').innerText = d.astra_height_cm !== undefined ?
                    'Cao ' + d.astra_height_cm.toFixed(1) + ' cm · chúi ' + d.astra_pitch_deg.toFixed(1) + '° · ' + d.astra_calib : '--';
                document.getElementById('aiFpsBadge').innerText = 'AI: ' + d.ai_fps.toFixed(1) + ' FPS';
                updatePauseUI(d.paused);
                updateDryRunUI(d.dry_run);
            }).catch(() => {});
        }, 1000);
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
        print(">> [WEB SERVER] Da khoi dong! Truy cap vao:")
        print(">> http://192.168.149.1:" + str(port))
        print("=======================================================\n")
        server.serve_forever()


# ==========================================================
# OBJECT SEEKER - BỘ NÃO ĐIỀU PHỐI TRUNG TÂM
# ==========================================================
class ObjectSeeker(object):
    def __init__(self):
        global seeker_instance, cam_source_oakd
        seeker_instance = self

        rospy.init_node('jetauto_object_seeker', anonymous=True)

        # Publisher cho topic cmd_vel cua khung gam JetAuto
        self.cmd_publishers = []
        for top in CMD_VEL_TOPICS:
            self.cmd_publishers.append(rospy.Publisher(top, Twist, queue_size=1))

        # Bien van toc mong muon 3-DOF (vong ne vat can dat, Heartbeat 10Hz gui di)
        self.motion_lock = threading.Lock()
        self.desired_linear_x = 0.0
        self.desired_linear_y = 0.0  # MECANUM STRAFE (TRUOT NGANG)
        self.desired_angular_z = 0.0
        self.desired_time = 0.0
        self.published = (0.0, 0.0, 0.0)
        self.block_reason = ''

        # Lai tay: lenh dang giu + han dung (web gui lap lai moi 150 ms khi giu nut)
        self.manual_override_until = 0.0
        self.manual_command = (0.0, 0.0, 0.0)

        # Bo ne vat can LiDAR + do sau Astra (obstacle_avoidance.py)
        self.avoid = AVOID_CFG
        self.lidar_model = oa.LidarModel(self.avoid)
        self.grid = oa.LocalGrid(self.avoid)
        self.planner = oa.MotionPlanner(self.avoid)
        self.tracker = oa.TargetTracker(self.avoid)
        self.astra = oa.AstraDepth(self.avoid)
        self.sensor_lock = threading.Lock()
        self.target_lock = threading.Lock()
        self.astra_lock = threading.Lock()
        self.latest_scan = None
        self.scan_time = 0.0
        self.astra_result = None
        self.astra_time = 0.0
        self.astra_processed = 0.0
        self.depth_frame = None
        self.depth_time = 0.0
        self.depth_buffer = deque(maxlen=12)
        self.depth_intrinsics = None
        self.rgb_intrinsics = None
        self.rgb_distortion = None
        self.rgb_stamp = None
        self.odom_pose = None
        self.odom_time = 0.0
        self.camera_time = 0.0
        self.frame_seq = 0
        self.calibrate_attempts = 0
        self.calibrate_next = 0.0
        self.nav_cmd = oa.NavCommand(state='KHOI_DONG', reason='DANG KHOI DONG')
        self.nav_target = oa.TargetEstimate()
        self.safety_snapshot = (np.zeros((0, 2)), np.zeros((0, 2)), 0.0)
        self.nav_loop_ms = 0.0
        self.planner_reset_request = False
        self.ros_subscribers = []   # giu handle de rospy khong huy Subscriber som

        # Trang thai AI (khai bao truoc khi tai model: web /status co the hoi ngay luc khoi dong)
        self.state_lock = threading.Lock()
        self.last_detections = []
        self.last_action_text = "DANG KHOI DONG..."
        self.last_status_color = (0, 165, 255)
        self.best_target_center = None
        self.last_target_confidence = None
        self.last_target_box = None
        self.last_depth_sync_ms = None
        self.ai_fps = 0.0

        # Load YOLO Model
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
            used = self.session.get_providers()[0]
            rospy.loginfo(">> [JetAuto] Tai model bang onnxruntime ({}) thanh cong!".format(used))
        except Exception as e:
            rospy.logwarn(">> [JetAuto] Dung cv2.dnn thay onnxruntime: " + str(e))
            self.net = cv2.dnn.readNetFromONNX(model_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.use_ort = False

        rospy.on_shutdown(self.stop)

        # 1. KHOI DONG DONG CO HEARTBEAT LOOP (10Hz)
        self.motion_thread = threading.Thread(target=self.motion_heartbeat_loop)
        self.motion_thread.daemon = True
        self.motion_thread.start()
        rospy.loginfo(">> [JetAuto] Da khoi dong Thread dieu khien Dong Co 10Hz Heartbeat (Mecanum 3-DOF)!")

        # 2. KHOI DONG CAMERA ASTRA (RGB + DO SAU) VA LIDAR / ODOMETRY
        self.init_astra_capture()
        self.init_navigation_inputs()

        # 3. CAMERA OAK-D (dang tat: USE_OAKD = False)
        if USE_OAKD:
            self.oakd_thread = threading.Thread(target=self.init_oakd_capture)
            self.oakd_thread.daemon = True
            self.oakd_thread.start()
        else:
            cam_source_oakd = "Radar LiDAR + Astra"

        # 4. KHOI DONG THREAD NEN JPEG NEN (TIET KIEM CPU & GIL CHO WEB)
        self.encoder_thread = threading.Thread(target=jpeg_encoder_loop)
        self.encoder_thread.daemon = True
        self.encoder_thread.start()

        # 5. KHOI DONG AI YOLO (chi nhan dien + dinh vi muc tieu)
        self.ai_thread = threading.Thread(target=self.ai_worker_loop)
        self.ai_thread.daemon = True
        self.ai_thread.start()

        # 6. KHOI DONG VONG NE VAT CAN 10Hz (ra quyet dinh van toc)
        self.nav_thread = threading.Thread(target=self.navigation_loop)
        self.nav_thread.daemon = True
        self.nav_thread.start()
        rospy.loginfo(">> [JetAuto] Ne vat can: LiDAR + do sau Astra | OAK-D: {} | Chay kho: {}".format(
            'BAT' if USE_OAKD else 'TAT', 'BAT' if DRY_RUN else 'TAT'))

    def load_classes(self):
        path = os.path.join(os.path.dirname(__file__), 'coco.names')
        if os.path.exists(path):
            with open(path, 'r') as f:
                return [c.strip() for c in f.readlines() if c.strip()]
        return ['person']

    def set_desired_velocity(self, linear_x=0.0, linear_y=0.0, angular_z_deg=0.0):
        with self.motion_lock:
            self.desired_linear_x = float(linear_x)
            self.desired_linear_y = float(linear_y)  # MECANUM STRAFE
            self.desired_angular_z = float(angular_z_deg)
            self.desired_time = time.monotonic()

    def trigger_motor_test(self):
        """Kiem tra dong co 3-DOF: Tien -> Truot Trai -> Truot Phai -> Xoay -> Dung (van kiem tra va cham)."""
        def run_test():
            rospy.loginfo(">> [TEST MECANUM] Tien 0.5s -> Truot Trai 0.5s -> Truot Phai 0.5s -> Xoay 0.5s -> Dung!")
            for command in ((0.15, 0.0, 0.0), (0.0, 0.12, 0.0), (0.0, -0.12, 0.0), (0.0, 0.0, 35.0)):
                end = time.monotonic() + 0.5
                while time.monotonic() < end and not rospy.is_shutdown():
                    self.manual_command = command
                    self.manual_override_until = time.monotonic() + 0.3
                    time.sleep(0.1)
            self.manual_move('stop')
            rospy.loginfo(">> [TEST MECANUM] Hoan tat test 3-DOF!")

        t = threading.Thread(target=run_test)
        t.daemon = True
        t.start()

    def manual_move(self, direction):
        """Lenh lai tay tu web; vong ne vat can van kiem tra va cham truoc khi cho chay."""
        command = MANUAL_COMMANDS.get(direction)
        if command is None:
            self.manual_command = (0.0, 0.0, 0.0)
            self.manual_override_until = 0.0
            self.set_desired_velocity(0.0, 0.0, 0.0)
            return
        self.manual_command = command
        self.manual_override_until = time.monotonic() + 0.5   # web gui lai moi 150 ms khi dang giu

    def motion_heartbeat_loop(self):
        """Vong lap 10Hz gui Twist Mecanum 3-DOF. Driver khung gam KHONG tu dung khi mat lenh, nen o day:
        lenh cu / tam dung / chay kho / sap va cham -> gui van toc 0."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = time.monotonic()
            with self.motion_lock:
                vx = self.desired_linear_x
                vy = self.desired_linear_y
                wz = self.desired_angular_z
                age = now - self.desired_time
                obstacles, _, snap_time = self.safety_snapshot
            reason = ''
            if IS_PAUSED:
                vx = vy = wz = 0.0
                reason = 'TAM DUNG'
            elif age > COMMAND_TIMEOUT_S:
                vx = vy = wz = 0.0
                reason = 'LENH HET HAN'
            elif abs(vx) + abs(vy) + abs(wz) > 1e-6:
                if now - snap_time > COMMAND_TIMEOUT_S:
                    vx = vy = wz = 0.0
                    reason = 'DU LIEU VAT CAN CU'
                else:
                    ok, why = oa.motion_is_safe(obstacles, vx, vy, wz, self.avoid)
                    if not ok:
                        vx = vy = wz = 0.0
                        reason = why
            if DRY_RUN:
                send = (0.0, 0.0, 0.0)
                reason = reason or 'CHAY KHO'
            else:
                send = (vx, vy, wz)

            twist_msg = Twist()
            twist_msg.linear.x = float(send[0])
            twist_msg.linear.y = float(send[1])  # MECANUM STRAFING (TRUOT NGANG)
            twist_msg.angular.z = float(math.radians(send[2]))

            for pub in self.cmd_publishers:
                try:
                    pub.publish(twist_msg)
                except Exception:
                    pass
            self.published = send
            self.block_reason = reason

            if abs(send[0]) > 0.001 or abs(send[1]) > 0.001 or abs(send[2]) > 0.001:
                rospy.loginfo_throttle(2.5, ">> [MECANUM 3-DOF] vx={:.2f} m/s, vy={:.2f} m/s (strafe), wz={:.1f} deg/s".format(*send))

            rate.sleep()

    # ==========================================================
    # MODUL CAMERA 1: ASTRA PRO PLUS (ROS Topic: RGB + DO SAU)
    # ==========================================================
    def init_astra_capture(self):
        global cam_source_astra
        rospy.loginfo(">> [ASTRA] Dang ket noi Camera Astra Pro Plus qua ROS topic...")
        cam_source_astra = "Dang cho tin hieu Astra..."

        if RosImage is not None:
            self.ros_subscribers += [
                rospy.Subscriber('/depth_cam/rgb/image_raw', RosImage, self._callback_ros_astra, queue_size=1, buff_size=2**24),
                rospy.Subscriber('/depth_cam/depth/image_raw', RosImage, self._callback_depth, queue_size=1, buff_size=2**24),
                rospy.Subscriber('/depth_cam/depth/camera_info', CameraInfo, self._callback_depth_info, queue_size=1),
                rospy.Subscriber('/depth_cam/rgb/camera_info', CameraInfo, self._callback_rgb_info, queue_size=1),
            ]

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

    def _callback_depth_info(self, msg):
        with self.astra_lock:
            self.astra.set_intrinsics(msg.K[0], msg.K[4], msg.K[2], msg.K[5], msg.width, msg.height)
        self.depth_intrinsics = (msg.K[0], msg.K[4], msg.K[2], msg.K[5])

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

    def _callback_depth(self, msg):
        now = time.monotonic()
        depth = self._decode_depth(msg)
        if depth is None:
            return
        stamp = msg.header.stamp.to_sec() if msg.header.stamp else 0.0
        with self.sensor_lock:
            self.depth_frame = depth
            self.depth_time = now
            self.depth_buffer.append((stamp, depth))
        # Tach vat can tu do sau ~6 lan/giay (du cho xe 0.18 m/s, tiet kiem CPU Jetson)
        if now - self.astra_processed < 0.16 or self.depth_intrinsics is None:
            return
        self.astra_processed = now
        with self.astra_lock:
            if self.calibrate_attempts < 15 and now >= self.calibrate_next and self.astra.source != 'RANSAC':
                # Hieu chuan mat san (do cao + goc chui) ngay khi thay san; that bai thi thu lai sau 2 s
                self.calibrate_attempts += 1
                self.calibrate_next = now + 2.0
                if self.astra.calibrate(depth, now):
                    rospy.loginfo(">> [ASTRA] Hieu chuan mat san: cao {:.1f} cm, chui {:.1f} do".format(
                        100 * self.astra.height, self.astra.pitch_deg))
            result = self.astra.process(depth, now)
        with self.sensor_lock:
            self.astra_result = result
            self.astra_time = now

    def request_astra_calibration(self):
        with self.astra_lock:
            self.astra.source = 'CHO HIEU CHUAN'
            self.calibrate_attempts = 0
            self.calibrate_next = 0.0

    # ==========================================================
    # LIDAR + ODOMETRY CHO BO NE VAT CAN
    # ==========================================================
    def init_navigation_inputs(self):
        self.ros_subscribers += [
            rospy.Subscriber('/scan', LaserScan, self._callback_scan, queue_size=1),
            rospy.Subscriber('/odom', Odometry, self._callback_odom, queue_size=1),
        ]

    def _callback_scan(self, msg):
        scan = self.lidar_model.process(msg.ranges, msg.angle_min, msg.angle_increment,
                                        msg.range_min, msg.range_max, time.monotonic())
        with self.sensor_lock:
            self.latest_scan = scan
            self.scan_time = scan.stamp

    def _callback_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self.sensor_lock:
            self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
            self.odom_time = time.monotonic()

    # ==========================================================
    # MODUL CAMERA 2: LUXONIS OAK-D (DepthAI Stereo Depth & RGB)
    # ==========================================================
    def init_oakd_capture(self):
        global raw_frame_oakd, display_frame_oakd, cam_source_oakd, depth_frame_oakd, obstacle_info
        rospy.loginfo(">> [OAK-D] Dang khoi dong Luxonis OAK-D (RGB + Stereo Depth)...")
        cam_source_oakd = "Dang ket noi DepthAI..."

        if dai is not None:
            try:
                devices = dai.Device.getAllAvailableDevices()
                if devices:
                    rospy.loginfo(">> [OAK-D] Tim thay {} thiet bi OAK-D. Tao Pipeline Stereo Depth...".format(len(devices)))
                    pipeline = dai.Pipeline()

                    # 1. Cam RGB
                    cam_rgb = pipeline.createColorCamera()
                    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
                    cam_rgb.setPreviewSize(640, 480)
                    cam_rgb.setInterleaved(False)
                    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
                    cam_rgb.setFps(25)
                    xout_rgb = pipeline.createXLinkOut()
                    xout_rgb.setStreamName("rgb")
                    cam_rgb.preview.link(xout_rgb.input)

                    # 2. Stereo Depth
                    has_stereo = False
                    try:
                        mono_left = pipeline.createMonoCamera()
                        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
                        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

                        mono_right = pipeline.createMonoCamera()
                        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
                        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

                        stereo = pipeline.createStereoDepth()
                        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
                        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
                        mono_left.out.link(stereo.left)
                        mono_right.out.link(stereo.right)

                        xout_depth = pipeline.createXLinkOut()
                        xout_depth.setStreamName("depth")
                        stereo.depth.link(xout_depth.input)
                        has_stereo = True
                    except Exception as stereo_err:
                        rospy.logwarn(">> [OAK-D] Setup Stereo Depth gap loi (chay che do RGB don): " + str(stereo_err))

                    # 3. Ket noi Device
                    device = None
                    try:
                        device = dai.Device(pipeline)
                    except Exception:
                        time.sleep(2)
                        device = dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)

                    q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                    q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False) if has_stereo else None

                    cam_source_oakd = "OAK-D Spatial ({})".format(device.getUsbSpeed().name)
                    rospy.loginfo(">> [OAK-D] Da ket noi thanh cong OAK-D Spatial Radar!")

                    cur_dl, cur_dc, cur_dr = 999.0, 999.0, 999.0

                    while not rospy.is_shutdown():
                        # Doc Depth de cap nhat khoang cach vat can
                        if q_depth is not None:
                            in_depth = q_depth.tryGet()
                            if in_depth is not None:
                                d_frame = in_depth.getFrame()
                                cur_dl, cur_dc, cur_dr = process_obstacle_depth(d_frame)
                                with obstacle_lock:
                                    obstacle_info['dist_l'] = cur_dl
                                    obstacle_info['dist_c'] = cur_dc
                                    obstacle_info['dist_r'] = cur_dr
                                    if cur_dc < 30.0:
                                        obstacle_info['status'] = 'NGUY HIEM'
                                    elif cur_dc < 60.0:
                                        obstacle_info['status'] = 'CANH BAO'
                                    else:
                                        obstacle_info['status'] = 'AN TOAN'

                        # Doc RGB de hien thi len Web
                        if q_rgb is not None:
                            in_rgb = q_rgb.tryGet()
                            if in_rgb is not None:
                                frame = in_rgb.getCvFrame()
                                if frame is not None:
                                    with obstacle_lock:
                                        s_dir = obstacle_info.get('strafe_dir', 'NONE')
                                    rendered_oakd = render_oakd_hud(frame, cur_dl, cur_dc, cur_dr, s_dir)
                                    with frame_lock:
                                        raw_frame_oakd = frame
                                        display_frame_oakd = rendered_oakd

                        time.sleep(0.01)
                    return
            except Exception as e:
                rospy.logwarn(">> [OAK-D] DepthAI gap su co: {}. Thu fallback ROS Topic...".format(e))

        cam_source_oakd = "Fallback ROS Topic"
        if RosImage is not None:
            rospy.Subscriber('/oakd/rgb/image_raw', RosImage, self._callback_ros_oakd, queue_size=1, buff_size=2**24)

    def _callback_ros_oakd(self, msg):
        global raw_frame_oakd, display_frame_oakd, cam_source_oakd
        frame = self._decode_ros_image(msg)
        if frame is not None:
            cam_source_oakd = "OAK-D (ROS Topic)"
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
    # MODUL AI YOLO & VISION SEEKING
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

    # Màu chữ trạng thái điều hướng trên ảnh camera (BGR)
    STATE_COLORS = {
        'BAM': (0, 255, 0), 'DA_TOI': (0, 255, 0), 'NE_TRAI': (0, 215, 255), 'NE_PHAI': (0, 215, 255),
        'BI_CHAN': (0, 0, 255), 'DUNG_AN_TOAN': (0, 0, 255), 'MAT_LIDAR': (0, 0, 255), 'MAT_CAMERA': (0, 0, 255),
        'LAI_TAY': (255, 229, 0), 'LAI_TAY_CHAN': (0, 0, 255), 'TAM_DUNG': (0, 165, 255),
        'NHIN_QUANH': (255, 200, 80),
    }

    def _state_color(self, state):
        if state.startswith('NHO_'):
            return (255, 200, 80)
        if state.startswith('TIM'):
            return (200, 180, 150)
        return self.STATE_COLORS.get(state, (0, 165, 255))

    def _render_ai_overlay(self, frame, cam_name, detections, action_text, status_color, target_center, target_info=None):
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
            if target_info and target_info.get('distance') is not None:
                cv2.putText(frame, "{:.2f} m ({})".format(target_info['distance'], target_info['source']),
                            (target_center[0] + 10, target_center[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        # Dải nền tối phía trên để chữ trạng thái đọc được cả khi ảnh sáng
        band = frame[0:78, :]
        band[:] = (band * 0.35).astype(np.uint8)
        cv2.putText(frame, action_text[:46], (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)
        cv2.putText(frame, cam_name, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 229, 255), 2)
        cv2.putText(frame, "AI FPS: {:.1f}".format(self.ai_fps), (w - 140, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        if DRY_RUN:
            cv2.putText(frame, "CHAY KHO - DONG CO KHONG CHAY", (20, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        return frame

    # ==========================================================
    # AI YOLO: NHẬN DIỆN + ĐỊNH VỊ MỤC TIÊU
    # (vận tốc do vòng né vật cản 10Hz quyết định, không phụ thuộc tốc độ YOLO)
    # ==========================================================
    def ai_worker_loop(self):
        global display_frame_astra
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

            target_center = None
            target_info = None
            if best is not None:
                # Hướng + khoảng cách tới tâm xe (đo bằng độ sâu Astra, không có thì ước lượng theo kích thước)
                bearing, distance, source = self._locate_target(best['box'], raw_frame.shape,
                                                                 depth_frame=matched_depth)
                with self.sensor_lock:
                    pose = self.odom_pose if (self.odom_pose is not None and now - self.odom_time < SENSOR_TIMEOUT_S) else None
                with self.target_lock:
                    if self.tracker.label != label:
                        self.tracker.reset(label)
                    self.tracker.observe(label, bearing, distance, now, pose, source)
                bx, by, bw, bh = best['box']
                target_center = (bx + bw // 2, by + bh // 2)
                target_info = {'distance': distance, 'source': source}
            else:
                with self.target_lock:
                    self.tracker.missed(now)

            cmd = self.nav_cmd
            rendered = self._render_ai_overlay(raw_frame, "[AI TRACK] ASTRA | " + label.upper(), detections,
                                               cmd.reason or cmd.state, self._state_color(cmd.state),
                                               target_center, target_info)

            with frame_lock:
                display_frame_astra = rendered

            with self.state_lock:
                self.last_detections = detections
                self.best_target_center = target_center
                candidate = max(raw_targets, key=lambda d: d['conf']) if raw_targets else None
                self.last_target_confidence = candidate['conf'] if candidate is not None else None
                self.last_target_box = candidate['box'] if candidate is not None else None
                self.last_depth_sync_ms = depth_sync_ms
                self.ai_fps = current_ai_fps

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
            return oa.locate_target(box, self._rgb_intrinsics(shape), self.astra, depth, self.depth_intrinsics,
                                    self.avoid, TARGET_CLASS, self.rgb_distortion)

    def _pick_target(self, targets, shape):
        """Nhiều vật cùng lớp: ưu tiên vật gần hướng đang bám (không nhảy qua lại), rồi tới độ tin cậy."""
        with self.target_lock:
            recent = self.tracker.last_time is not None and time.monotonic() - self.tracker.last_time < 1.5
            last = self.tracker.bearing if recent else None
        if last is None or len(targets) == 1:
            return max(targets, key=lambda d: d['conf'])
        return max(targets, key=lambda d: d['conf'] - 0.8 * abs(oa.wrap_scalar(
            self._locate_target(d['box'], shape, with_depth=False)[0] - last)))

    def reset_target(self, label):
        """Đổi mục tiêu từ web: quên mục tiêu cũ, vòng né vật cản tự reset bộ lập đường."""
        with self.target_lock:
            self.tracker.reset(label)
        self.planner_reset_request = True

    # ==========================================================
    # VÒNG NÉ VẬT CẢN 10Hz: BẢN ĐỒ NHỚ -> LẬP ĐƯỜNG -> VẬN TỐC MONG MUỐN
    # ==========================================================
    def navigation_loop(self):
        period = 1.0 / NAV_RATE_HZ
        cycle = 0
        state = {'pose': (0.0, 0.0, 0.0), 'scan_used': 0.0, 'astra_used': 0.0}
        while not rospy.is_shutdown():
            started = time.monotonic()
            try:
                self._navigation_step(started, state, cycle)
            except Exception as error:
                # Lỗi bất ngờ: dừng xe (Heartbeat cũng tự dừng khi lệnh hết hạn) và ghi log
                rospy.logerr_throttle(5.0, ">> [NE VAT CAN] Loi: {}".format(error))
                self.set_desired_velocity(0.0, 0.0, 0.0)
            cycle += 1
            self.nav_loop_ms = 1000.0 * (time.monotonic() - started)
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    def _navigation_step(self, now, state, cycle):
        with self.sensor_lock:
            scan, scan_time = self.latest_scan, self.scan_time
            astra, astra_time = self.astra_result, self.astra_time
            odom, odom_time = self.odom_pose, self.odom_time
        if getattr(self, 'planner_reset_request', False):
            self.planner_reset_request = False
            self.planner.reset()

        odom_ok = odom is not None and now - odom_time < SENSOR_TIMEOUT_S
        if odom_ok:
            pose = state['pose'] = odom
        else:
            pose = state['pose']
            # Mất odom mà xe đang chạy -> bản đồ nhớ sẽ lệch: chỉ dùng dữ liệu cảm biến hiện tại
            if any(abs(v) > 1e-3 for v in self.published):
                self.grid.clear()
        scan_ok = scan is not None and now - scan_time < SENSOR_TIMEOUT_S

        grid = self.grid
        grid.recenter(pose[0], pose[1])
        grid.mark_body(pose, now)
        if scan_ok and scan_time > state['scan_used']:
            grid.update_lidar(pose, scan, now)
            state['scan_used'] = scan_time
        if astra is not None and astra_time > state['astra_used'] and now - astra_time < 1.0:
            grid.update_astra(pose, astra['obstacles'], astra['floor'], astra['cliffs'], now)
            state['astra_used'] = astra_time
        obstacles, blind = grid.query(pose, now, lidar_model=self.lidar_model)
        if scan_ok:
            obstacles = oa._concat(obstacles, scan.points)
        with self.target_lock:
            target = self.tracker.estimate(now, pose if odom_ok else None)

        if IS_PAUSED:
            self.planner.reset()
            cmd = oa.NavCommand(state='TAM_DUNG', reason='TAM DUNG THEO LENH')
        elif now < self.manual_override_until:
            self.planner.reset()
            cmd = self._manual_command(obstacles, blind, scan_ok)
        elif not scan_ok:
            self.planner.reset()
            cmd = oa.NavCommand(state='MAT_LIDAR', reason='MAT TIN HIEU LIDAR /scan - DUNG XE')
        elif now - self.camera_time > 2.0:
            self.planner.reset()
            cmd = oa.NavCommand(state='MAT_CAMERA', reason='MAT CAMERA ASTRA - DUNG XE')
        else:
            cmd = self.planner.plan(obstacles, blind, target, now, pose if odom_ok else None)

        self.nav_cmd = cmd
        self.nav_target = target
        with self.motion_lock:
            # Lệnh + dữ liệu vật cản đã dùng để kiểm tra nó được cập nhật cùng lúc,
            # Heartbeat không bao giờ ghép lệnh cũ với dữ liệu mới (hoặc ngược lại)
            self.safety_snapshot = (obstacles, blind, now)
            self.desired_linear_x, self.desired_linear_y, self.desired_angular_z = cmd.vx, cmd.vy, cmd.wz
            self.desired_time = time.monotonic()
        with self.state_lock:
            self.last_action_text = cmd.reason or cmd.state
        if cycle % 2 == 0:
            # 5 lần/giây: cập nhật khoảng trống cho giao diện + vẽ radar
            if cmd.clearance is None or cmd.v_allow is None:
                cmd.clearance, cmd.v_allow = self.planner.evaluate(obstacles, blind)
            self._update_obstacle_info(cmd, obstacles, scan_ok)
            self._render_radar(pose, now, cmd, target, odom_ok, scan_ok)

    def _manual_command(self, obstacles, blind, scan_ok):
        """Lái tay vẫn an toàn: chặn lệnh sắp đâm vật; đi vào vùng LiDAR không thấy thì chạy chậm."""
        vx, vy, wz = self.manual_command
        moving = abs(vx) + abs(vy) > 1e-6
        if not scan_ok and moving:
            return oa.NavCommand(0.0, 0.0, 0.0, 'LAI_TAY_CHAN', 'MAT LIDAR: CHI CHO XOAY TAI CHO')
        ok, why = oa.motion_is_safe(obstacles, vx, vy, wz, self.avoid)
        if not ok:
            return oa.NavCommand(0.0, 0.0, 0.0, 'LAI_TAY_CHAN', 'CHAN LAI TAY: ' + why)
        reason = 'LAI THU CONG'
        if moving and len(blind) and not oa.motion_is_safe(blind, vx, vy, 0.0, self.avoid)[0]:
            scale = min(1.0, 0.06 / math.hypot(vx, vy))
            vx, vy = vx * scale, vy * scale
            reason = 'LAI TAY VAO VUNG LIDAR KHONG THAY - CHAY CHAM'
        return oa.NavCommand(vx, vy, wz, 'LAI_TAY', reason)

    def _update_obstacle_info(self, cmd, obstacles, scan_ok):
        """Khoảng trống (cm) trái / trước / phải theo vật cản thật (không tính vùng mù) cho giao diện."""
        clear = self.planner.clearance(oa.thin_points(obstacles)) if len(obstacles) else None
        dirs = self.planner.dirs

        def sector(centre_deg):
            if clear is None:
                return 999.0
            value = float(np.min(clear[np.abs(oa.wrap(dirs - math.radians(centre_deg))) <= math.radians(20)]))
            return 999.0 if not np.isfinite(value) else min(999.0, 100.0 * value)

        dl, dc, dr = sector(90.0), sector(0.0), sector(-90.0)
        status = 'MAT LIDAR' if not scan_ok else ('NGUY HIEM' if dc < 15 else ('CANH BAO' if dc < 40 else 'AN TOAN'))
        with obstacle_lock:
            obstacle_info.update(dist_l=dl, dist_c=dc, dist_r=dr, status=status,
                                 strafe_dir='LEFT' if cmd.vy > 0.01 else ('RIGHT' if cmd.vy < -0.01 else 'NONE'))

    def _render_radar(self, pose, now, cmd, target, odom_ok, scan_ok):
        global raw_frame_oakd, display_frame_oakd
        if USE_OAKD:
            return
        with self.astra_lock:
            calib = 'ASTRA {:.0f}cm {:.1f}do {}'.format(100 * self.astra.height, self.astra.pitch_deg, self.astra.source)
        lines = [
            (cmd.reason or cmd.state)[:56],
            'vx {:+.2f} vy {:+.2f} m/s  wz {:+.0f} do/s{}'.format(cmd.vx, cmd.vy, cmd.wz, '  CHAY KHO' if DRY_RUN else ''),
            '{} | ODOM {} | LIDAR {}'.format(calib, 'OK' if odom_ok else 'MAT', 'OK' if scan_ok else 'MAT'),
        ]
        img = oa.render_radar(self.avoid, self.grid, pose, now, cmd, target, self.lidar_model, lines=lines,
                              dry_run=DRY_RUN)
        with frame_lock:
            raw_frame_oakd = img
            display_frame_oakd = img

    def nav_status(self):
        """Trạng thái né vật cản cho /status (giao diện web)."""
        now = time.monotonic()
        cmd, tgt = self.nav_cmd, self.nav_target

        def age(stamp):
            return round(now - stamp, 1) if stamp else None

        with self.astra_lock:
            astra = {'astra_height_cm': round(100 * self.astra.height, 1),
                     'astra_pitch_deg': round(self.astra.pitch_deg, 1),
                     'astra_calib': self.astra.source,
                     'astra_quality': round(float(self.astra.quality), 2)}
        scan = self.latest_scan
        with self.state_lock:
            detections = [
                {'class_name': d['class_name'], 'confidence': round(float(d['conf']), 3),
                 'box': [int(v) for v in d['box']]}
                for d in sorted(self.last_detections, key=lambda d: d['conf'], reverse=True)[:8]
            ]
            candidate_confidence = self.last_target_confidence
            candidate_box = list(self.last_target_box) if self.last_target_box is not None else None
            depth_sync_ms = self.last_depth_sync_ms
        with self.target_lock:
            tracker_hits = self.tracker.hits
            tracker_confirmed = self.tracker.confirmed
        status = {
            'nav_state': cmd.state,
            'nav_reason': cmd.reason,
            'block_reason': self.block_reason,
            'published': [round(float(v), 3) for v in self.published],
            'target_visible': bool(tgt.valid and tgt.visible),
            'target_remembered': bool(tgt.valid and not tgt.visible),
            'target_bearing_deg': round(math.degrees(tgt.bearing), 1) if tgt.valid else None,
            'target_distance_m': round(float(tgt.distance), 2) if tgt.valid and tgt.distance is not None else None,
            'target_source': tgt.source if tgt.valid else '',
            'camera_age': age(self.camera_time),
            'depth_age': age(self.depth_time),
            'lidar_age': age(self.scan_time),
            'odom_age': age(self.odom_time),
            'lidar_valid_pct': round(100 * scan.valid_fraction) if scan is not None else None,
            'nav_loop_ms': round(self.nav_loop_ms, 1),
            'manual_active': now < self.manual_override_until,
            'detections': detections,
            'target_candidate_confidence': (round(float(candidate_confidence), 3)
                                            if candidate_confidence is not None else None),
            'target_candidate_box': candidate_box,
            'target_confidence_required': TARGET_CONFIDENCE.get(TARGET_CLASS, TARGET_CONFIDENCE_DEFAULT),
            'tracker_hits': tracker_hits,
            'tracker_confirmed': bool(tracker_confirmed),
            'depth_sync_ms': round(float(depth_sync_ms), 1) if depth_sync_ms is not None else None,
        }
        status.update(astra)
        return status

    def stop(self):
        rospy.loginfo(">> [JetAuto Pro] Dang tat va dung robot an toan...")
        self.manual_override_until = 0.0
        self.set_desired_velocity(0.0, 0.0, 0.0)
        # Driver khung gầm KHÔNG tự dừng khi mất lệnh -> chủ động gửi vận tốc 0 vài lần trước khi thoát
        zero = Twist()
        for _ in range(3):
            for pub in self.cmd_publishers:
                try:
                    pub.publish(zero)
                except Exception:
                    pass
            time.sleep(0.05)


# ==========================================================
# MAIN ENTRY POINT
# ==========================================================
def sigint_handler(signum, frame):
    print("\n>> Nhan tin hieu Ctrl+C! Dang dung dong co va thoat an toan...", flush=True)
    if seeker_instance:
        seeker_instance.stop()   # gui van toc 0 truoc khi thoat (driver khong tu dung)
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
        seeker = ObjectSeeker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sigint_handler(None, None)
