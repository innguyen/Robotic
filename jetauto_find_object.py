#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
JetAuto Pro - Dual Vision AI Tracker & Mecanum Obstacle Avoidance Controller
- Mắt 1 (Astra Pro Plus): Chạy YOLOv5 AI nhận diện và khóa mục tiêu (Heading lock).
- Mắt 2 (Luxonis OAK-D): Quét độ sâu Stereo Depth gầm xe, tính khoảng cách vật cản 3 hướng (Trái / Giữa / Phải).
- Chiến thuật Mecanum Strafing: Tự động trượt ngang (linear.y) luồn lách qua chướng ngại vật mà ĐẦU XE KHÔNG QUAY ĐI, giữ mục tiêu 100% trong khung hình!
- Bộ đệm nén JPEG nền (Background JPEG Cache): Stream 2 camera đồng thời mượt mà không khóa GIL hay ngốn CPU.
- Giao diện Web: Xem độc lập 2 Card hoặc xem Ghép 1 luồng Unified Side-by-Side.
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

# Khung hình toàn cục cho cả 2 Camera
raw_frame_astra = None
raw_frame_oakd = None
depth_frame_oakd = None
display_frame_astra = None  # Astra: Bounding Box AI & Khóa mục tiêu
display_frame_oakd = None   # OAK-D: FPV Gầm xe + Thước đo Radar vật cản
frame_lock = threading.Lock()

# Bộ đệm JPEG cache nén sẵn trên RAM (chống nghẽn socket và khóa GIL của Python)
cached_jpeg_astra = None
cached_jpeg_oakd = None
cached_jpeg_combined = None
jpeg_lock = threading.Lock()

# Thông số vật cản từ OAK-D (Khoảng cách cm)
obstacle_info = {
    'dist_l': 999.0,
    'dist_c': 999.0,
    'dist_r': 999.0,
    'status': 'AN TOAN',
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
            title = "LUXONIS OAK-D (OBSTACLE RADAR)"
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
    """Thread chuyên trách nén JPEG sẵn vào RAM ở tốc độ ~22 FPS."""
    global cached_jpeg_astra, cached_jpeg_oakd, cached_jpeg_combined
    while not rospy.is_shutdown():
        try:
            # 1. Nén ảnh Astra
            f_astra = None
            with frame_lock:
                if display_frame_astra is not None:
                    f_astra = display_frame_astra
                elif raw_frame_astra is not None:
                    f_astra = raw_frame_astra

            if f_astra is not None:
                ret, jpg = cv2.imencode('.jpg', f_astra, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                if ret:
                    raw_bytes = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()
                    with jpeg_lock:
                        cached_jpeg_astra = raw_bytes

            # 2. Nén ảnh OAK-D
            f_oakd = None
            with frame_lock:
                if display_frame_oakd is not None:
                    f_oakd = display_frame_oakd
                elif raw_frame_oakd is not None:
                    f_oakd = raw_frame_oakd

            if f_oakd is not None:
                ret, jpg = cv2.imencode('.jpg', f_oakd, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                if ret:
                    raw_bytes = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()
                    with jpeg_lock:
                        cached_jpeg_oakd = raw_bytes

            # 3. Nén ảnh Ghép Side-by-Side
            if f_astra is not None or f_oakd is not None:
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
                'obs_status': obs_copy['status']
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
        <p>Cam Astra: YOLO Khóa Mục Tiêu | Cam OAK-D: Stereo Depth & Trượt Ngang Né Vật Cản</p>
    </div>

    <div class="container">
        <!-- TOP TOOLBAR -->
        <div class="top-toolbar">
            <div class="group-left">
                <span style="color: #64748b; font-weight: 700; font-size: 0.8rem; text-transform: uppercase;">Bố cục:</span>
                <button class="btn-toggle active" id="btnLayoutDual" onclick="setLayout('dual')">⊞ 2 Màn Hình (Cards)</button>
                <button class="btn-toggle" id="btnLayoutCombined" onclick="setLayout('combined')">🔲 Ghép 1 Luồng (Unified)</button>
                <button class="btn-toggle" id="btnLayoutAstra" onclick="setLayout('astra')">🔲 Chỉ Cam Astra</button>
                <button class="btn-toggle" id="btnLayoutOakd" onclick="setLayout('oakd')">🔲 Chỉ Cam OAK-D</button>
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

            <!-- CAM 2: LUXONIS OAK-D (OBSTACLE RADAR) -->
            <div class="stream-card" id="cardOakd">
                <div class="stream-header">
                    <span style="color: #38bdf8;">🔵 LUXONIS OAK-D (Stereo Depth & Radar Gầm Xe)</span>
                    <span style="color: var(--success);" id="feedStatusOakd">● Radar Active</span>
                </div>
                <div class="stream-box">
                    <img id="feedOakd" src="/cam_oakd.mjpg" alt="OAK-D Feed" />
                </div>
            </div>
        </div>

        <!-- UNIFIED COMBINED VIEW -->
        <div class="stream-card" id="cardCombined" style="display: none; width: 100%;">
            <div class="stream-header">
                <span style="color: var(--primary);">⊞ GHÉP ĐỒNG THỜI 2 CAMERA (Side-by-Side Unified Stream)</span>
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
                    <span>Trạng thái Hệ thống & Radar Né Vật Cản</span>
                    <span id="aiFpsBadge" style="color: var(--primary);">AI: 0.0 FPS</span>
                </div>
                <div class="hud-grid">
                    <div class="hud-item" style="grid-column: span 2;">
                        <div class="hud-label">Hành động Robot</div>
                        <div class="hud-val" id="hudAction" style="color: var(--warning);">ĐANG KHỞI TẠO...</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Radar OAK-D (Trái | Giữa | Phải)</div>
                        <div class="hud-val" id="hudRadar" style="color: var(--success);">L: -- | C: -- | R: --</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Hướng Né Mecanum</div>
                        <div class="hud-val" id="hudStrafe" style="color: #38bdf8;">ĐƯỜNG THÔNG THOÁNG</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Lệnh Vận Tốc 3-DOF (10Hz)</div>
                        <div class="hud-val" id="hudCmdVel">vx: 0.00 | vy: 0.00 | wz: 0.0°</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Nguồn 2 Camera</div>
                        <div class="hud-val" id="hudSources" style="font-size: 0.8rem; color: #94a3b8;">Astra Pro + OAK-D</div>
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
                    <button class="dpad-btn" onmousedown="drive('forward')" onmouseup="drive('stop')" ontouchstart="drive('forward')" ontouchend="drive('stop')">▲ TIẾN</button>
                    <div></div>

                    <button class="dpad-btn dpad-strafe" onmousedown="drive('strafe_left')" onmouseup="drive('stop')" ontouchstart="drive('strafe_left')" ontouchend="drive('stop')">◄◄ TRƯỢT T</button>
                    <button class="dpad-btn dpad-stop" onclick="drive('stop')">DỪNG</button>
                    <button class="dpad-btn dpad-strafe" onmousedown="drive('strafe_right')" onmouseup="drive('stop')" ontouchstart="drive('strafe_right')" ontouchend="drive('stop')">TRƯỢT P ►►</button>

                    <button class="dpad-btn" onmousedown="drive('left')" onmouseup="drive('stop')" ontouchstart="drive('left')" ontouchend="drive('stop')">⟲ XOAY T</button>
                    <button class="dpad-btn" onmousedown="drive('backward')" onmouseup="drive('stop')" ontouchstart="drive('backward')" ontouchend="drive('stop')">▼ LÙI</button>
                    <button class="dpad-btn" onmousedown="drive('right')" onmouseup="drive('stop')" ontouchstart="drive('right')" ontouchend="drive('stop')">XOAY P ⟳</button>
                </div>
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

        function togglePause() {
            fetch('/toggle_pause', {method: 'POST'})
                .then(r => r.json())
                .then(d => updatePauseUI(d.paused));
        }

        function updatePauseUI(paused) {
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

        function drive(dir) {
            fetch('/manual_move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({direction: dir})
            }).catch(() => {});
        }

        // Hỗ trợ bàn phím W/A/S/D + Q/E trượt ngang
        window.addEventListener('keydown', e => {
            if (['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
            if (e.key === 'ArrowUp' || e.key === 'w' || e.key === 'W') drive('forward');
            else if (e.key === 'ArrowDown' || e.key === 's' || e.key === 'S') drive('backward');
            else if (e.key === 'q' || e.key === 'Q') drive('strafe_left');
            else if (e.key === 'e' || e.key === 'E') drive('strafe_right');
            else if (e.key === 'ArrowLeft' || e.key === 'a' || e.key === 'A') drive('left');
            else if (e.key === 'ArrowRight' || e.key === 'd' || e.key === 'D') drive('right');
            else if (e.key === ' ') drive('stop');
        });

        window.addEventListener('keyup', e => {
            if (['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
            if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','w','a','s','d','q','e','W','A','S','D','Q','E'].includes(e.key)) {
                drive('stop');
            }
        });

        // Cập nhật trạng thái HUD mỗi giây
        setInterval(() => {
            fetch('/status').then(r => r.json()).then(d => {
                document.getElementById('hudAction').innerText = d.action_text;
                
                // Hiển thị radar khoảng cách 3 hướng
                const dl = d.dist_left < 800 ? d.dist_left.toFixed(0) + 'cm' : 'THOÁNG';
                const dc = d.dist_center < 800 ? d.dist_center.toFixed(0) + 'cm' : 'THOÁNG';
                const dr = d.dist_right < 800 ? d.dist_right.toFixed(0) + 'cm' : 'THOÁNG';
                const radarEl = document.getElementById('hudRadar');
                radarEl.innerText = 'L: ' + dl + ' | C: ' + dc + ' | R: ' + dr;
                if (d.obs_status === 'NGUY HIEM') radarEl.style.color = 'var(--danger)';
                else if (d.obs_status === 'CANH BAO') radarEl.style.color = 'var(--warning)';
                else radarEl.style.color = 'var(--success)';

                // Hướng né
                const strafeEl = document.getElementById('hudStrafe');
                if (d.strafe_dir === 'LEFT') {
                    strafeEl.innerText = '<<< TRƯỢT TRÁI NÉ';
                    strafeEl.style.color = 'var(--warning)';
                } else if (d.strafe_dir === 'RIGHT') {
                    strafeEl.innerText = 'TRƯỢT PHẢI NÉ >>>';
                    strafeEl.style.color = 'var(--warning)';
                } else {
                    strafeEl.innerText = 'ĐƯỜNG THÔNG THOÁNG';
                    strafeEl.style.color = 'var(--success)';
                }

                document.getElementById('hudCmdVel').innerText = 'vx: ' + d.linear_x.toFixed(2) + ' | vy: ' + d.linear_y.toFixed(2) + ' | wz: ' + d.angular_z.toFixed(1) + '°';
                document.getElementById('hudSources').innerText = d.source_astra + ' + ' + d.source_oakd;
                document.getElementById('aiFpsBadge').innerText = 'AI: ' + d.ai_fps.toFixed(1) + ' FPS';
                updatePauseUI(d.paused);
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
        global seeker_instance
        seeker_instance = self

        rospy.init_node('jetauto_object_seeker', anonymous=True)

        # Publishers cho tat ca cac topic cmd_vel cua JetAuto
        self.cmd_publishers = []
        for top in CMD_VEL_TOPICS:
            self.cmd_publishers.append(rospy.Publisher(top, Twist, queue_size=1))

        # Bien van toc mong muon 3-DOF (cho Motion Heartbeat Loop 10Hz)
        self.motion_lock = threading.Lock()
        self.desired_linear_x = 0.0
        self.desired_linear_y = 0.0  # MECANUM STRAFE (TRUOT NGANG)
        self.desired_angular_z = 0.0
        self.smooth_x = None
        self.smooth_w = None

        self.manual_override_until = 0.0

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

        # Trang thai AI
        self.state_lock = threading.Lock()
        self.last_detections = []
        self.last_action_text = "DANG KHOI DONG..."
        self.last_status_color = (0, 165, 255)
        self.best_target_center = None
        self.ai_fps = 0.0

        rospy.on_shutdown(self.stop)

        # 1. KHOI DONG DONG CO HEARTBEAT LOOP (10Hz)
        self.motion_thread = threading.Thread(target=self.motion_heartbeat_loop)
        self.motion_thread.daemon = True
        self.motion_thread.start()
        rospy.loginfo(">> [JetAuto] Da khoi dong Thread dieu khien Dong Co 10Hz Heartbeat (Mecanum 3-DOF)!")

        # 2. KHOI DONG CAMERA ASTRA (ROS Topic /depth_cam/rgb/image_raw)
        self.astra_thread = threading.Thread(target=self.init_astra_capture)
        self.astra_thread.daemon = True
        self.astra_thread.start()

        # 3. KHOI DONG CAMERA OAK-D (Stereo Depth + RGB qua DepthAI)
        self.oakd_thread = threading.Thread(target=self.init_oakd_capture)
        self.oakd_thread.daemon = True
        self.oakd_thread.start()

        # 4. KHOI DONG THREAD NEN JPEG NEN (TIET KIEM CPU & GIL CHO WEB)
        self.encoder_thread = threading.Thread(target=jpeg_encoder_loop)
        self.encoder_thread.daemon = True
        self.encoder_thread.start()

        # 5. KHOI DONG AI CENTRAL ARBITRATION WORKER LOOP
        self.ai_thread = threading.Thread(target=self.ai_worker_loop)
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
            self.desired_linear_y = float(linear_y)  # MECANUM STRAFE
            self.desired_angular_z = float(angular_z_deg)

    def trigger_motor_test(self):
        """Kiem tra dong co 3-DOF: Tien -> Truot Trai -> Truot Phai -> Xoay -> Dung!"""
        def run_test():
            rospy.loginfo(">> [TEST MECANUM] Tien 0.5s -> Truot Trai 0.5s -> Truot Phai 0.5s -> Xoay 0.5s -> Dung!")
            self.manual_override_until = time.time() + 3.0
            # 1. Tien
            self.set_desired_velocity(0.18, 0.0, 0.0)
            time.sleep(0.5)
            # 2. Truot trai
            self.set_desired_velocity(0.0, 0.18, 0.0)
            time.sleep(0.5)
            # 3. Truot phai
            self.set_desired_velocity(0.0, -0.18, 0.0)
            time.sleep(0.5)
            # 4. Xoay tai cho
            self.set_desired_velocity(0.0, 0.0, 35.0)
            time.sleep(0.5)
            # 5. Dung
            self.set_desired_velocity(0.0, 0.0, 0.0)
            self.manual_override_until = 0.0
            rospy.loginfo(">> [TEST MECANUM] Hoan tat test 3-DOF!")

        t = threading.Thread(target=run_test)
        t.daemon = True
        t.start()

    def manual_move(self, direction):
        if direction == 'forward':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.18, 0.0, 0.0)
        elif direction == 'backward':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(-0.18, 0.0, 0.0)
        elif direction == 'strafe_left':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.0, 0.18, 0.0)  # Truot trai
        elif direction == 'strafe_right':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.0, -0.18, 0.0)  # Truot phai
        elif direction == 'left':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.0, 0.0, 35.0)   # Xoay trai
        elif direction == 'right':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.0, 0.0, -35.0)  # Xoay phai
        else:
            self.manual_override_until = 0.0
            self.set_desired_velocity(0.0, 0.0, 0.0)

    def motion_heartbeat_loop(self):
        """Vong lap 10Hz lien tuc gui Twist lenh dong co Mecanum 3-DOF de duy tri watchdog STM32."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.motion_lock:
                vx = self.desired_linear_x
                vy = self.desired_linear_y
                wz = self.desired_angular_z
                paused = IS_PAUSED

            if paused:
                vx = 0.0
                vy = 0.0
                wz = 0.0

            twist_msg = Twist()
            twist_msg.linear.x = float(vx)
            twist_msg.linear.y = float(vy)  # MECANUM STRAFING (TRUOT NGANG)
            twist_msg.angular.z = float(math.radians(wz))

            for pub in self.cmd_publishers:
                try:
                    pub.publish(twist_msg)
                except Exception:
                    pass

            if abs(vx) > 0.001 or abs(vy) > 0.001 or abs(wz) > 0.001:
                rospy.loginfo_throttle(2.5, ">> [MECANUM 3-DOF] vx={:.2f} m/s, vy={:.2f} m/s (strafe), wz={:.1f} deg/s".format(vx, vy, wz))

            rate.sleep()

    # ==========================================================
    # MODUL CAMERA 1: ASTRA PRO PLUS (ROS Topic)
    # ==========================================================
    def init_astra_capture(self):
        global cam_source_astra
        rospy.loginfo(">> [ASTRA] Dang ket noi Camera Astra Pro Plus qua ROS topic...")
        cam_source_astra = "Dang cho tin hieu Astra..."

        if RosImage is not None:
            rospy.Subscriber('/depth_cam/rgb/image_raw', RosImage, self._callback_ros_astra, queue_size=1, buff_size=2**24)
            rospy.Subscriber('/camera/rgb/image_raw', RosImage, self._callback_ros_astra, queue_size=1, buff_size=2**24)

    def _callback_ros_astra(self, msg):
        global raw_frame_astra, cam_source_astra
        frame = self._decode_ros_image(msg)
        if frame is not None:
            cam_source_astra = "Astra Pro (ROS)"
            with frame_lock:
                raw_frame_astra = frame

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

    def _render_ai_overlay(self, frame, cam_name, detections, action_text, status_color, target_center):
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
        cv2.putText(frame, cam_name, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 229, 255), 2)
        cv2.putText(frame, "AI FPS: {:.1f}".format(self.ai_fps), (w - 140, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return frame

    # ==========================================================
    # CENTRAL ARBITRATION: DUNG HỢP ASTRA + OAK-D (MECANUM STRAFING)
    # ==========================================================
    def ai_worker_loop(self):
        global raw_frame_astra, display_frame_astra, obstacle_info
        frame_counter = 0
        last_fps_time = time.time()
        current_ai_fps = 0.0

        while not rospy.is_shutdown():
            raw_frame = None
            with frame_lock:
                if raw_frame_astra is not None:
                    raw_frame = raw_frame_astra.copy()

            if raw_frame is None:
                if not IS_PAUSED and time.time() >= self.manual_override_until:
                    self.set_desired_velocity(0.0, 0.0, 18.0)
                time.sleep(0.04)
                continue

            frame_counter += 1
            now = time.time()
            if now - last_fps_time >= 1.0:
                current_ai_fps = frame_counter / (now - last_fps_time)
                frame_counter = 0
                last_fps_time = now

            h, w = raw_frame.shape[:2]
            center_screen_x = w // 2

            detections = self.run_yolo(raw_frame)
            target_detections = [d for d in detections if d['class_name'] == TARGET_CLASS]
            target_found = len(target_detections) > 0

            target_center = None
            width_ratio = 0.0

            # 1. TÍNH TOÁN HƯỚNG QUAY KHÓA MỤC TIÊU TỪ CAMERA ASTRA
            base_vx = 0.0
            heading_wz = 0.0

            if target_found:
                best = max(target_detections, key=lambda d: d['conf'])
                bx, by, bw, bh = best['box']
                target_center = (bx + bw // 2, by + bh // 2)

                if self.smooth_x is None:
                    self.smooth_x = float(target_center[0])
                    self.smooth_w = float(bw)
                else:
                    self.smooth_x = 0.7 * self.smooth_x + 0.3 * float(target_center[0])
                    self.smooth_w = 0.7 * self.smooth_w + 0.3 * float(bw)

                error_x = self.smooth_x - center_screen_x
                width_ratio = self.smooth_w / float(w)

                if width_ratio > 0.45:
                    base_vx = 0.0
                    heading_wz = 0.0
                else:
                    # Bộ điều khiển P tinh chỉnh góc quay để mục tiêu luôn ở giữa màn hình
                    heading_wz = float(-error_x * 0.07)
                    heading_wz = max(min(heading_wz, 25.0), -25.0)
                    base_vx = 0.15 if abs(error_x) < 80 else 0.08
            else:
                self.smooth_x = None
                base_vx = 0.0
                heading_wz = 20.0  # Xoay tìm kiếm nếu chưa thấy

            # 2. ĐỌC THÔNG SỐ VẬT CẢN TỪ CAMERA OAK-D
            with obstacle_lock:
                d_l = obstacle_info['dist_l']
                d_c = obstacle_info['dist_c']
                d_r = obstacle_info['dist_r']

            # 3. BỘ NÃO ĐIỀU PHỐI (CENTRAL ARBITRATION VOI BÁNH MECANUM)
            cmd_vx = 0.0
            cmd_vy = 0.0  # Mecanum Strafe
            cmd_wz = 0.0
            strafe_name = "NONE"
            action_text = ""
            status_color = (0, 255, 0)

            if time.time() < self.manual_override_until:
                action_text = "LAI THU CONG (MANUAL)"
                status_color = (0, 229, 255)
            elif IS_PAUSED:
                action_text = "TAM DUNG THEO LENH"
                status_color = (0, 165, 255)
                self.set_desired_velocity(0.0, 0.0, 0.0)
            else:
                if d_c < 30.0:
                    # CẤP 3: VẬT CẢN QUÁ GẦN (< 30cm) -> PHANH KHẨN CẤP TIẾN THẲNG, TRƯỢT NGANG NÉ
                    cmd_vx = 0.0
                    if d_l > d_r:
                        cmd_vy = 0.15  # Trượt sang trái
                        strafe_name = "LEFT"
                        action_text = "PHANH! TRUOT TRAI NE VAT CAN ({:.0f}cm)".format(d_c)
                    else:
                        cmd_vy = -0.15  # Trượt sang phải
                        strafe_name = "RIGHT"
                        action_text = "PHANH! TRUOT PHAI NE VAT CAN ({:.0f}cm)".format(d_c)
                    cmd_wz = heading_wz if target_found else 0.0
                    status_color = (0, 0, 255)

                elif d_c < 60.0:
                    # CẤP 2: VẬT CẢN CHẮN ĐƯỜNG TIẾN (30cm - 60cm) -> TRƯỢT NGANG MECANUM LÁCH VÒNG
                    cmd_vx = 0.04  # Bò chậm tới trước
                    if d_l > d_r:
                        cmd_vy = 0.13  # Trượt sang trái
                        strafe_name = "LEFT"
                        action_text = "LACH TRUOT TRAI NE VAT CAN ({:.0f}cm)".format(d_c)
                    else:
                        cmd_vy = -0.13  # Trượt sang phải
                        strafe_name = "RIGHT"
                        action_text = "LACH TRUOT PHAI NE VAT CAN ({:.0f}cm)".format(d_c)
                    cmd_wz = heading_wz  # Luôn giữ đầu xe nhìn thẳng vào target!
                    status_color = (0, 215, 255)

                else:
                    # CẤP 1: ĐƯỜNG THÔNG THOÁNG HOÀN TOÀN (> 60cm)
                    cmd_vx = base_vx
                    cmd_vy = 0.0
                    strafe_name = "NONE"
                    cmd_wz = heading_wz

                    if target_found:
                        if width_ratio > 0.45:
                            action_text = "DA DEN GAN MUC TIEU! DUNG LAI"
                            status_color = (0, 255, 0)
                        else:
                            action_text = "BAM THEO {} (THOANG: {:.0f}cm)".format(TARGET_CLASS.upper(), d_c)
                            status_color = (0, 255, 0)
                    else:
                        action_text = "DANG TIM KIEM: {}".format(TARGET_CLASS.upper())
                        status_color = (0, 0, 255)

                with obstacle_lock:
                    obstacle_info['strafe_dir'] = strafe_name

                self.set_desired_velocity(cmd_vx, cmd_vy, cmd_wz)

            cam_label = "[AI TRACK] ASTRA PRO PLUS"
            rendered = self._render_ai_overlay(raw_frame, cam_label, detections, action_text, status_color, target_center)

            with frame_lock:
                display_frame_astra = rendered

            with self.state_lock:
                self.last_detections = detections
                self.last_action_text = action_text
                self.last_status_color = status_color
                self.best_target_center = target_center
                self.ai_fps = current_ai_fps

            time.sleep(0.015)

    def stop(self):
        rospy.loginfo(">> [JetAuto Pro] Dang tat va dung robot an toan...")
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
        seeker = ObjectSeeker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        sigint_handler(None, None)
