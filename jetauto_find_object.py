#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
JetAuto Pro - AI Vision Tracker & Single Camera Controller
- Giao dien Web don camera sac net, nhe CPU (tiet kiem tai toi da cho Jetson Nano).
- Camera chinh mac dinh: Luxonis OAK-D RGB (DepthAI truc tiep, khong do tre).
- Ho tro doi sang Astra Pro Plus bat cu luc nao.
- He thong lai 10Hz Heartbeat duy tri an toan voi STM32 watchdog.
- Ho tro che do Stream MJPEG va Live Snapshot Polling chong den man hinh.
- Tich hop D-Pad lai thu cong va nut Test Dong Co 2 giay.
"""

import sys
import rospy
import cv2
import numpy as np
import math
import threading
import time
import os
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
# CAU HINH HE THONG
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

# Khung hinh toan cuc cho ca 2 Camera
raw_frame_astra = None
raw_frame_oakd = None
display_frame_astra = None  # Astra: Co Bounding Box AI & HUD Dieu Khien
display_frame_oakd = None   # OAK-D: Luong Video FPV Sach Net (Khong chay Detect)
frame_lock = threading.Lock()
seeker_instance = None

cam_source_astra = "Khoi tao..."
cam_source_oakd = "Khoi tao..."


def get_frame_for_cam(cam_name='astra'):
    """Lay khung hinh cho tung camera cu the (Astra hoac OAK-D)."""
    with frame_lock:
        if cam_name == 'astra':
            if display_frame_astra is not None:
                return display_frame_astra.copy()
            if raw_frame_astra is not None:
                return raw_frame_astra.copy()
            title = "ASTRA PRO PLUS (AI DETECT & TRACK)"
            status = cam_source_astra
        else:
            if display_frame_oakd is not None:
                return display_frame_oakd.copy()
            if raw_frame_oakd is not None:
                return raw_frame_oakd.copy()
            title = "LUXONIS OAK-D (LIVE FPV STREAM)"
            status = cam_source_oakd

    f = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.rectangle(f, (0, 0), (640, 480), (15, 23, 42), -1)
    cv2.putText(f, title, (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 229, 255), 2)
    cv2.putText(f, "DANG CHO TIN HIEU CAMERA...", (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
    cv2.putText(f, status, (30, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (148, 163, 184), 1)
    return f


def get_active_frame():
    """Fallback lay frame Astra mac dinh."""
    return get_frame_for_cam('astra')


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
                frame = get_frame_for_cam(cam_name)
                ret, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                if ret:
                    raw = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()
                    packet = b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + raw + b'\r\n'
                    self.wfile.write(packet)
                    self.wfile.flush()
                time.sleep(0.04)
        except Exception:
            pass

    def _serve_snapshot(self, cam_name='astra'):
        frame = get_frame_for_cam(cam_name)
        ret, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ret:
            raw = jpg.tobytes() if sys.version_info[0] == 3 else jpg.tostring()
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
                IS_PAUSED = False
                rospy.loginfo(">> [WEB] Da doi muc tieu sang: " + TARGET_CLASS)
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

        elif self.path == '/switch_camera':
            cam_choice = data.get('cam', 'oakd').strip().lower()
            if seeker_instance:
                ok, msg = seeker_instance.switch_camera(cam_choice)
                self.send_response(200 if ok else 400)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({'status': 'ok' if ok else 'error', 'msg': msg, 'current': seeker_instance.active_ai_cam}).encode())
                return

        elif self.path == '/test_motors':
            if seeker_instance:
                seeker_instance.trigger_motor_test()
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status": "ok", "msg": "Dang test dong co trong 2 giay!"}')
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
        if self.path.startswith('/cam_oakd') or self.path.startswith('/video_feed_oakd'):
            self._stream_mjpeg('oakd')
        elif self.path.startswith('/cam_astra') or self.path.startswith('/video_feed_astra'):
            self._stream_mjpeg('astra')
        elif self.path.startswith('/video_feed') or self.path.startswith('/cam_combined.mjpg') or self.path.startswith('/cam_single.mjpg'):
            self._stream_mjpeg('astra')
        elif self.path.startswith('/snapshot_oakd'):
            self._serve_snapshot('oakd')
        elif self.path.startswith('/snapshot_astra'):
            self._serve_snapshot('astra')
        elif self.path.startswith('/snapshot'):
            self._serve_snapshot('astra')
        elif self.path.startswith('/status'):
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            status_data = {
                'target': TARGET_CLASS,
                'paused': IS_PAUSED,
                'ai_fps': seeker_instance.ai_fps if seeker_instance else 0.0,
                'action_text': seeker_instance.last_action_text if seeker_instance else "CHO KHOI DONG",
                'source_astra': cam_source_astra,
                'source_oakd': cam_source_oakd,
                'linear_x': seeker_instance.desired_linear_x if seeker_instance else 0.0,
                'angular_z': seeker_instance.desired_angular_z if seeker_instance else 0.0
            }
            self.wfile.write(json.dumps(status_data).encode())
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(self._render_html().encode('utf-8'))

    def _render_html(self, active_cam='astra'):
        return """<!DOCTYPE html>
<html lang="vi">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>JetAuto Vision Controller</title>
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
            font-size: 2.1rem;
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
        .header p { color: #94a3b8; font-size: 0.95rem; margin-top: 0.4rem; }
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

        .camera-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 1.2rem;
            width: 100%;
        }
        @media (max-width: 860px) {
            .camera-grid { grid-template-columns: 1fr; }
        }

        /* TOOLBAR TOP */
        .top-toolbar {
            display: flex;
            justify-content: space-between;
            align-items: center;
            background: rgba(15, 23, 42, 0.85);
            border: 1px solid var(--glass-border);
            border-radius: 16px;
            padding: 10px 16px;
            flex-wrap: wrap;
            gap: 12px;
        }
        .group-left, .group-right { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }

        .btn-toggle {
            background: rgba(255, 255, 255, 0.06);
            border: 1px solid rgba(255, 255, 255, 0.1);
            color: #cbd5e1;
            padding: 8px 14px;
            border-radius: 10px;
            font-weight: 600;
            font-size: 0.85rem;
            cursor: pointer;
            transition: all 0.2s;
        }
        .btn-toggle:hover { background: rgba(255, 255, 255, 0.12); color: white; }
        .btn-toggle.active {
            background: linear-gradient(135deg, #00E5FF, #3B82F6);
            color: #0b1120;
            border-color: transparent;
            font-weight: 800;
            box-shadow: 0 2px 10px rgba(0, 229, 255, 0.35);
        }

        /* SINGLE CAMERA CARD */
        .stream-card {
            background: var(--card-bg);
            border: 1px solid var(--glass-border);
            border-radius: 20px;
            padding: 14px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.4);
            position: relative;
        }
        .stream-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
            font-size: 0.9rem;
            color: #94a3b8;
            font-weight: 700;
        }
        .stream-box {
            position: relative;
            width: 100%;
            aspect-ratio: 4/3;
            max-height: 580px;
            background: #000;
            border-radius: 16px;
            overflow: hidden;
            box-shadow: inset 0 0 0 1px rgba(255,255,255,0.05);
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .stream-box img {
            width: 100%;
            height: 100%;
            object-fit: contain;
            display: block;
        }

        /* CONTROLS */
        .control-row {
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
        }
        input[type="text"] {
            flex: 1;
            min-width: 240px;
            padding: 12px 18px;
            border-radius: 12px;
            border: 1px solid var(--glass-border);
            background: rgba(0,0,0,0.4);
            color: white;
            font-size: 0.95rem;
            outline: none;
        }
        input[type="text"]:focus { border-color: var(--primary); box-shadow: 0 0 10px rgba(0, 229, 255, 0.2); }

        .btn-action {
            padding: 12px 20px;
            border-radius: 12px;
            border: none;
            font-weight: 800;
            font-size: 0.95rem;
            cursor: pointer;
            transition: all 0.2s;
            display: inline-flex;
            align-items: center;
            gap: 8px;
        }
        .btn-action:hover { transform: translateY(-2px); }
        .btn-find { background: linear-gradient(135deg, #00E5FF, #3B82F6); color: #0b1120; }
        .btn-pause { background: linear-gradient(135deg, #EF4444, #DC2626); color: white; }
        .btn-resume { background: linear-gradient(135deg, #22C55E, #16A34A); color: white; }
        .btn-test { background: linear-gradient(135deg, #F59E0B, #D97706); color: white; }

        /* QUICK TAGS */
        .tags-bar { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; }
        .tag-btn {
            background: rgba(255, 255, 255, 0.05);
            border: 1px solid rgba(255, 255, 255, 0.08);
            color: #94a3b8;
            padding: 6px 14px;
            border-radius: 20px;
            font-size: 0.8rem;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.2s;
        }
        .tag-btn:hover { background: rgba(0, 229, 255, 0.15); color: var(--primary); border-color: var(--primary); }

        /* BOTTOM GRID: STATUS HUD & D-PAD */
        .bottom-grid {
            display: grid;
            grid-template-columns: 3fr 2fr;
            gap: 1.2rem;
        }
        @media (max-width: 768px) { .bottom-grid { grid-template-columns: 1fr; } }

        .info-card, .dpad-card {
            background: rgba(15, 23, 42, 0.65);
            border: 1px solid var(--glass-border);
            border-radius: 18px;
            padding: 16px;
        }
        .card-title {
            font-size: 0.85rem;
            font-weight: 700;
            color: #94a3b8;
            text-transform: uppercase;
            letter-spacing: 1px;
            margin-bottom: 12px;
            display: flex;
            align-items: center;
            justify-content: space-between;
        }

        .dpad-layout {
            display: grid;
            grid-template-columns: 60px 60px 60px;
            grid-template-rows: 60px 60px 60px;
            gap: 8px;
            justify-content: center;
            margin: 6px auto;
        }
        .dpad-btn {
            background: rgba(255, 255, 255, 0.07);
            border: 1px solid rgba(255, 255, 255, 0.15);
            color: #f8fafc;
            border-radius: 12px;
            font-size: 1.2rem;
            font-weight: 700;
            display: flex;
            align-items: center;
            justify-content: center;
            cursor: pointer;
            user-select: none;
            transition: all 0.15s;
        }
        .dpad-btn:active, .dpad-btn.pressed { background: var(--primary); color: #0b1120; transform: scale(0.94); }
        .dpad-stop { background: rgba(239, 68, 68, 0.2); border-color: rgba(239, 68, 68, 0.4); color: var(--danger); font-size: 0.8rem; }

        .hud-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            font-size: 0.85rem;
        }
        .hud-item {
            background: rgba(0,0,0,0.3);
            border: 1px solid var(--glass-border);
            border-radius: 10px;
            padding: 10px;
        }
        .hud-label { color: #64748b; font-size: 0.75rem; text-transform: uppercase; font-weight: 700; }
        .hud-val { color: #f8fafc; font-weight: 700; margin-top: 4px; font-size: 0.95rem; }
    </style>
</head>
<body>
    <div class="header">
        <h1>JetAuto Dual Vision <span class="target-badge" id="targetBadge">🎯 """ + TARGET_CLASS.upper() + """</span></h1>
        <p>Cam Astra: YOLO AI Detect & Bam Duoi | Cam OAK-D: Live FPV Video (Khong Detect)</p>
    </div>

    <div class="container">
        <!-- TOP TOOLBAR -->
        <div class="top-toolbar">
            <div class="group-left">
                <span style="color: #64748b; font-weight: 700; font-size: 0.8rem; text-transform: uppercase;">Bo cuc:</span>
                <button class="btn-toggle active" id="btnLayoutDual" onclick="setLayout('dual')">⊞ 2 Cam Song Song (Dual)</button>
                <button class="btn-toggle" id="btnLayoutAstra" onclick="setLayout('astra')">🔲 Chi Cam Astra (AI)</button>
                <button class="btn-toggle" id="btnLayoutOakd" onclick="setLayout('oakd')">🔲 Chi Cam OAK-D (FPV)</button>
            </div>
            <div class="group-right">
                <span style="color: #64748b; font-weight: 700; font-size: 0.8rem; text-transform: uppercase;">Che do tai:</span>
                <button class="btn-toggle active" id="btnFeedStream" onclick="setFeedMode('stream')">📡 Stream (MJPEG)</button>
                <button class="btn-toggle" id="btnFeedSnapshot" onclick="setFeedMode('snapshot')">⚡ Snapshot (Live)</button>
            </div>
        </div>

        <!-- DUAL CAMERA GRID -->
        <div class="camera-grid" id="cameraGrid">
            <!-- CAM 1: ASTRA PRO PLUS (YOLO AI) -->
            <div class="stream-card" id="cardAstra">
                <div class="stream-header">
                    <span style="color: var(--primary);">🟢 ASTRA PRO PLUS (YOLO AI & Dieu Khien)</span>
                    <span style="color: var(--success);" id="feedStatusAstra">● AI Active</span>
                </div>
                <div class="stream-box">
                    <img id="feedAstra" src="/cam_astra.mjpg" alt="Astra AI Feed" onerror="handleStreamError('astra')" />
                </div>
            </div>

            <!-- CAM 2: LUXONIS OAK-D (CLEAN FPV STREAM) -->
            <div class="stream-card" id="cardOakd">
                <div class="stream-header">
                    <span style="color: #38bdf8;">🔵 LUXONIS OAK-D (Live FPV Stream - Khong Detect)</span>
                    <span style="color: var(--success);" id="feedStatusOakd">● Video Sac Net</span>
                </div>
                <div class="stream-box">
                    <img id="feedOakd" src="/cam_oakd.mjpg" alt="OAK-D FPV Feed" onerror="handleStreamError('oakd')" />
                </div>
            </div>
        </div>

        <!-- QUICK CONTROLS -->
        <div class="control-row">
            <input type="text" id="targetInput" placeholder="Nhap ten vat the can tim (person, bottle, cup, chair, cell phone...)" onkeypress="if(event.key==='Enter') setTarget()" />
            <button class="btn-action btn-find" onclick="setTarget()">🔍 TIM KIEM</button>
            <button id="pauseBtn" class="btn-action btn-pause" onclick="togglePause()">⏸️ TAM DUNG</button>
            <button class="btn-action btn-test" onclick="testMotors()">🛠️ TEST DONG CO (2S)</button>
        </div>

        <!-- QUICK TAGS -->
        <div class="tags-bar">
            <span style="font-size: 0.8rem; color: #64748b; font-weight: 700;">Goi y:</span>
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
                    <span>Trang thai he thong & Dong co</span>
                    <span id="aiFpsBadge" style="color: var(--primary);">AI: 0.0 FPS</span>
                </div>
                <div class="hud-grid">
                    <div class="hud-item">
                        <div class="hud-label">Hanh dong Robot</div>
                        <div class="hud-val" id="hudAction" style="color: var(--warning);">DANG KHOI TAO...</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Lenh lai 10Hz</div>
                        <div class="hud-val" id="hudCmdVel">vx: 0.00 | wz: 0.0°</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Cam 1 (AI Detect)</div>
                        <div class="hud-val" id="hudSourceAstra" style="color: var(--primary);">Astra Pro Plus</div>
                    </div>
                    <div class="hud-item">
                        <div class="hud-label">Cam 2 (Clean FPV)</div>
                        <div class="hud-val" id="hudSourceOakd" style="color: #38bdf8;">DepthAI ColorCamera</div>
                    </div>
                </div>
            </div>

            <!-- D-PAD MANUAL DRIVE -->
            <div class="dpad-card">
                <div class="card-title">
                    <span>Lai thu cong (D-Pad / W,A,S,D)</span>
                </div>
                <div class="dpad-layout">
                    <div></div>
                    <button class="dpad-btn" onmousedown="drive('forward')" onmouseup="drive('stop')" ontouchstart="drive('forward')" ontouchend="drive('stop')">▲</button>
                    <div></div>

                    <button class="dpad-btn" onmousedown="drive('left')" onmouseup="drive('stop')" ontouchstart="drive('left')" ontouchend="drive('stop')">◀</button>
                    <button class="dpad-btn dpad-stop" onclick="drive('stop')">DUNG</button>
                    <button class="dpad-btn" onmousedown="drive('right')" onmouseup="drive('stop')" ontouchstart="drive('right')" ontouchend="drive('stop')">▶</button>

                    <div></div>
                    <button class="dpad-btn" onmousedown="drive('backward')" onmouseup="drive('stop')" ontouchstart="drive('backward')" ontouchend="drive('stop')">▼</button>
                    <div></div>
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

            document.getElementById('btnLayoutDual').className = 'btn-toggle ' + (mode === 'dual' ? 'active' : '');
            document.getElementById('btnLayoutAstra').className = 'btn-toggle ' + (mode === 'astra' ? 'active' : '');
            document.getElementById('btnLayoutOakd').className = 'btn-toggle ' + (mode === 'oakd' ? 'active' : '');

            if (mode === 'dual') {
                grid.style.gridTemplateColumns = '1fr 1fr';
                cardAstra.style.display = 'block';
                cardOakd.style.display = 'block';
            } else if (mode === 'astra') {
                grid.style.gridTemplateColumns = '1fr';
                cardAstra.style.display = 'block';
                cardOakd.style.display = 'none';
            } else if (mode === 'oakd') {
                grid.style.gridTemplateColumns = '1fr';
                cardAstra.style.display = 'none';
                cardOakd.style.display = 'block';
            }
        }

        function setFeedMode(mode) {
            currentFeedMode = mode;
            document.getElementById('btnFeedStream').className = 'btn-toggle ' + (mode === 'stream' ? 'active' : '');
            document.getElementById('btnFeedSnapshot').className = 'btn-toggle ' + (mode === 'snapshot' ? 'active' : '');
            
            const imgAstra = document.getElementById('feedAstra');
            const imgOakd = document.getElementById('feedOakd');

            if (mode === 'snapshot') {
                document.getElementById('feedStatusAstra').innerText = '● Snapshot';
                document.getElementById('feedStatusOakd').innerText = '● Snapshot';
                startSnapshotLoop();
            } else {
                document.getElementById('feedStatusAstra').innerText = '● AI Active';
                document.getElementById('feedStatusOakd').innerText = '● Video Active';
                stopSnapshotLoop();
                imgAstra.src = '/cam_astra.mjpg?t=' + Date.now();
                imgOakd.src = '/cam_oakd.mjpg?t=' + Date.now();
            }
        }

        function handleStreamError(cam) {
            console.warn("MJPEG stream loi hoac bi chan o " + cam + ", chuyen sang Snapshot Polling...");
            setFeedMode('snapshot');
        }

        function startSnapshotLoop() {
            stopSnapshotLoop();
            function poll() {
                const imgAstra = document.getElementById('feedAstra');
                const imgOakd = document.getElementById('feedOakd');
                const cardAstra = document.getElementById('cardAstra');
                const cardOakd = document.getElementById('cardOakd');

                if (cardAstra && cardAstra.style.display !== 'none') {
                    const nA = new Image();
                    nA.onload = () => { imgAstra.src = nA.src; };
                    nA.src = '/snapshot_astra.jpg?t=' + Date.now();
                }
                if (cardOakd && cardOakd.style.display !== 'none') {
                    const nO = new Image();
                    nO.onload = () => { imgOakd.src = nO.src; };
                    nO.src = '/snapshot_oakd.jpg?t=' + Date.now();
                }
                snapshotTimer = setTimeout(poll, 70);
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
                btn.innerText = '▶️ TIEP TUC';
            } else {
                btn.className = 'btn-action btn-pause';
                btn.innerText = '⏸️ TAM DUNG';
            }
        }

        function testMotors() {
            fetch('/test_motors', {method: 'POST'})
                .then(r => r.json())
                .then(d => alert(">> " + d.msg))
                .catch(e => alert("Loi ket noi: " + e));
        }

        function drive(dir) {
            fetch('/manual_move', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({direction: dir})
            }).catch(() => {});
        }

        // Ho tro ban phim W/A/S/D hoac mui ten
        window.addEventListener('keydown', e => {
            if (['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
            if (e.key === 'ArrowUp' || e.key === 'w' || e.key === 'W') drive('forward');
            else if (e.key === 'ArrowDown' || e.key === 's' || e.key === 'S') drive('backward');
            else if (e.key === 'ArrowLeft' || e.key === 'a' || e.key === 'A') drive('left');
            else if (e.key === 'ArrowRight' || e.key === 'd' || e.key === 'D') drive('right');
            else if (e.key === ' ') drive('stop');
        });

        window.addEventListener('keyup', e => {
            if (['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) return;
            if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','w','a','s','d','W','A','S','D'].includes(e.key)) {
                drive('stop');
            }
        });

        // Cap nhat HUD status moi giay
        setInterval(() => {
            fetch('/status').then(r => r.json()).then(d => {
                document.getElementById('hudAction').innerText = d.action_text;
                document.getElementById('hudCmdVel').innerText = 'vx: ' + d.linear_x.toFixed(2) + ' | wz: ' + d.angular_z.toFixed(1) + '°';
                document.getElementById('hudSourceAstra').innerText = d.source_astra;
                document.getElementById('hudSourceOakd').innerText = d.source_oakd;
                document.getElementById('aiFpsBadge').innerText = 'AI: ' + d.ai_fps.toFixed(1) + ' FPS';
                updatePauseUI(d.paused);
            }).catch(() => {});
        }, 1000);
    </script>
</body>
</html>"""


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


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


class ObjectSeeker(object):
    def __init__(self):
        global seeker_instance
        seeker_instance = self

        rospy.init_node('jetauto_object_seeker', anonymous=True)

        # Publishers cho tat ca cac topic cmd_vel cua JetAuto
        self.cmd_publishers = []
        for top in CMD_VEL_TOPICS:
            self.cmd_publishers.append(rospy.Publisher(top, Twist, queue_size=1))

        # Bien van toc mong muon (cho Motion Heartbeat Loop 10Hz)
        self.motion_lock = threading.Lock()
        self.desired_linear_x = 0.0
        self.desired_angular_z = 0.0
        self.smooth_x = None
        self.smooth_w = None

        self.manual_override_until = 0.0

        # Camera AI mac dinh: Astra Pro Plus
        self.active_ai_cam = 'astra'

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
        rospy.loginfo(">> [JetAuto] Da khoi dong Thread dieu khien Dong Co 10Hz Heartbeat!")

        # 2. KHOI DONG CAMERA ASTRA (ROS Topic)
        self.astra_thread = threading.Thread(target=self.init_astra_capture)
        self.astra_thread.daemon = True
        self.astra_thread.start()

        # 3. KHOI DONG CAMERA OAK-D (DepthAI truc tiep)
        self.oakd_thread = threading.Thread(target=self.init_oakd_capture)
        self.oakd_thread.daemon = True
        self.oakd_thread.start()

        # 4. KHOI DONG AI WORKER LOOP
        self.ai_thread = threading.Thread(target=self.ai_worker_loop)
        self.ai_thread.daemon = True
        self.ai_thread.start()

    def switch_camera(self, cam_choice):
        self.active_ai_cam = 'oakd' if cam_choice == 'oakd' else 'astra'
        with frame_lock:
            global display_frame
            display_frame = None
        rospy.loginfo(">> [CAMERA] Chuyen camera AI sang: " + self.active_ai_cam.upper())
        return True, "Da chon " + ("OAK-D" if self.active_ai_cam == 'oakd' else "Astra Pro")

    def load_classes(self):
        path = os.path.join(os.path.dirname(__file__), 'coco.names')
        if os.path.exists(path):
            with open(path, 'r') as f:
                return [x.strip() for x in f if x.strip()]
        return ['person', 'bottle', 'chair', 'cell phone', 'cup']

    def set_desired_velocity(self, linear_x=0.0, angular_z_deg=0.0):
        with self.motion_lock:
            self.desired_linear_x = float(linear_x)
            self.desired_angular_z = float(angular_z_deg)

    def trigger_motor_test(self):
        """Kiem tra dong co trong 2 giay bang cach gui chuoi lenh di chuyen."""
        def run_test():
            rospy.loginfo(">> [TEST DONG CO] Bat dau test: Tien 0.6s -> Re Trai 0.6s -> Re Phai 0.6s -> Dung!")
            self.manual_override_until = time.time() + 3.0
            self.set_desired_velocity(0.18, 0.0)
            time.sleep(0.6)
            self.set_desired_velocity(0.0, 35.0)
            time.sleep(0.6)
            self.set_desired_velocity(0.0, -35.0)
            time.sleep(0.6)
            self.set_desired_velocity(0.0, 0.0)
            self.manual_override_until = 0.0
            rospy.loginfo(">> [TEST DONG CO] Test dong co hoan tat!")

        t = threading.Thread(target=run_test)
        t.daemon = True
        t.start()

    def manual_move(self, direction):
        if direction == 'forward':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.18, 0.0)
        elif direction == 'backward':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(-0.18, 0.0)
        elif direction == 'left':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.0, 35.0)
        elif direction == 'right':
            self.manual_override_until = time.time() + 0.8
            self.set_desired_velocity(0.0, -35.0)
        else:
            self.manual_override_until = 0.0
            self.set_desired_velocity(0.0, 0.0)

    def motion_heartbeat_loop(self):
        """Vong lap 10Hz lien tuc gui Twist lenh dong co de duy tri watchdog STM32."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.motion_lock:
                vx = self.desired_linear_x
                wz = self.desired_angular_z
                paused = IS_PAUSED

            if paused:
                vx = 0.0
                wz = 0.0

            twist_msg = Twist()
            twist_msg.linear.x = float(vx)
            twist_msg.angular.z = float(math.radians(wz))

            for pub in self.cmd_publishers:
                try:
                    pub.publish(twist_msg)
                except Exception:
                    pass

            if abs(vx) > 0.001 or abs(wz) > 0.001:
                rospy.loginfo_throttle(2.0, ">> [DONG CO] Dang lai 10Hz: vx={:.2f} m/s, wz={:.1f} deg/s".format(vx, wz))

            rate.sleep()

    # ==========================================================
    # MODUL CAMERA 1: ASTRA PRO PLUS (ROS Topic /depth_cam/rgb/image_raw)
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
            cam_source_astra = "Astra Pro (ROS Topic)"
            with frame_lock:
                raw_frame_astra = frame

    # ==========================================================
    # MODUL CAMERA 2: LUXONIS OAK-D (DepthAI ColorCamera / OpenCV) - CLEAN FPV STREAM
    # ==========================================================
    def init_oakd_capture(self):
        global raw_frame_oakd, display_frame_oakd, cam_source_oakd
        rospy.loginfo(">> [OAK-D] Dang kiem tra phan cung Luxonis OAK-D...")
        cam_source_oakd = "Dang ket noi DepthAI..."

        if dai is not None:
            try:
                devices = dai.Device.getAllAvailableDevices()
                if devices:
                    rospy.loginfo(">> [OAK-D] Phat hien {} thiet bi OAK-D. Khoi tao ColorCamera pipeline...".format(len(devices)))
                    pipeline = dai.Pipeline()
                    cam_rgb = pipeline.createColorCamera()
                    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
                    cam_rgb.setPreviewSize(640, 480)
                    cam_rgb.setInterleaved(False)
                    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
                    cam_rgb.setFps(30)

                    xout = pipeline.createXLinkOut()
                    xout.setStreamName("rgb")
                    cam_rgb.preview.link(xout.input)

                    device = None
                    try:
                        device = dai.Device(pipeline)
                    except Exception:
                        time.sleep(2)
                        device = dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)

                    q = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                    cam_source_oakd = "DepthAI ColorCamera ({})".format(device.getUsbSpeed().name)
                    rospy.loginfo(">> [OAK-D] Da ket noi thanh cong DepthAI ColorCamera (Clean FPV)!")

                    while not rospy.is_shutdown():
                        in_rgb = q.tryGet()
                        if in_rgb is not None:
                            f = in_rgb.getCvFrame()
                            if f is not None:
                                f_clean = f.copy()
                                cv2.putText(f_clean, "LUXONIS OAK-D (LIVE FPV)", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 229, 255), 2)
                                with frame_lock:
                                    raw_frame_oakd = f
                                    display_frame_oakd = f_clean
                        time.sleep(0.01)
                    return
            except Exception as e:
                rospy.logwarn(">> [OAK-D] DepthAI gap su co: {}. Fallback ROS Topic...".format(e))

        cam_source_oakd = "Fallback ROS Topic"
        if RosImage is not None:
            rospy.Subscriber('/oakd/rgb/image_raw', RosImage, self._callback_ros_oakd, queue_size=1, buff_size=2**24)

    def _callback_ros_oakd(self, msg):
        global raw_frame_oakd, display_frame_oakd, cam_source_oakd
        frame = self._decode_ros_image(msg)
        if frame is not None:
            cam_source_oakd = "ROS Topic"
            f_clean = frame.copy()
            cv2.putText(f_clean, "LUXONIS OAK-D (LIVE FPV)", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 229, 255), 2)
            with frame_lock:
                raw_frame_oakd = frame
                display_frame_oakd = f_clean

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

        blob = cv2.dnn.blobFromImage(frame, 1.0/255.0, (INPUT_SIZE, INPUT_SIZE), swapRB=True, crop=False)

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
            cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), color, 2)
            cv2.putText(frame, "{} {:.0f}%".format(det['class_name'], det['conf']*100), (bx, by-8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)

        if target_center is not None:
            cv2.circle(frame, target_center, 6, (0, 0, 255), -1)

        cv2.putText(frame, action_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, status_color, 2)
        cv2.putText(frame, cam_name, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 229, 255), 2)
        cv2.putText(frame, "AI FPS: {:.1f}".format(self.ai_fps), (w - 140, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return frame

    def ai_worker_loop(self):
        global raw_frame_astra, display_frame_astra
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
                    self.set_desired_velocity(0.0, 18.0)
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

            action_text = "TIM {}".format(TARGET_CLASS.upper())
            status_color = (0, 0, 255)
            target_center = None

            if time.time() < self.manual_override_until:
                action_text = "LAI THU CONG (MANUAL)"
                status_color = (0, 229, 255)
            elif IS_PAUSED:
                action_text = "TAM DUNG THEO LENH"
                status_color = (0, 165, 255)
                self.set_desired_velocity(0.0, 0.0)
            elif target_found:
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
                    action_text = "DA DEN GAN! DUNG LAI"
                    status_color = (255, 0, 0)
                    self.set_desired_velocity(0.0, 0.0)
                else:
                    if error_x < -40:
                        action_text = "LECH TRAI -> RE TRAI"
                        status_color = (0, 255, 255)
                        self.set_desired_velocity(0.08, 15.0)
                    elif error_x > 40:
                        action_text = "LECH PHAI -> RE PHAI"
                        status_color = (0, 255, 255)
                        self.set_desired_velocity(0.08, -15.0)
                    else:
                        action_text = "CHINH GIUA -> TIEN THANG!"
                        status_color = (0, 255, 0)
                        self.set_desired_velocity(0.15, 0.0)
            else:
                self.smooth_x = None
                action_text = "TIM {}".format(TARGET_CLASS.upper())
                status_color = (0, 0, 255)
                self.set_desired_velocity(0.0, 20.0)

            cam_label = "[AI DETECT & TRACK] ASTRA PRO PLUS"
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
        self.set_desired_velocity(0.0, 0.0)


if __name__ == '__main__':
    try:
        web_thread = threading.Thread(target=start_web_server)
        web_thread.daemon = True
        web_thread.start()
        seeker = ObjectSeeker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
