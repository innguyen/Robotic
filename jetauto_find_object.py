#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
JetAuto Pro - Chuong trinh tu dong tim kiem va tien lai gan vat the
- Tich hop AI YOLO11n thay vi nhan dien mau sac.
- Ho tro truyen ten vat the tu dong lenh: python jetauto_find_object.py bottle
"""

import sys
import rospy
import cv2
import numpy as np
import math
import threading

import time
import os
import signal
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Tu dong them cac duong dan chua jetauto_sdk vao sys.path
for extra_p in [
    '/home/jetauto/jetauto_ws/src/jetauto_sdk/src',
    '/home/jetauto/jetauto_ws/devel/lib/python2.7/dist-packages',
    '/home/jetauto/jetauto_ws/devel/lib/python3/dist-packages',
    '/home/jetauto/catkin_ws/src/jetauto_sdk/src',
    '/home/jetauto/catkin_ws/devel/lib/python2.7/dist-packages',
    '/home/jetauto/catkin_ws/devel/lib/python3/dist-packages',
    '/home/jetauto/ros_ws/src/jetauto_sdk/src',
    '/home/jetauto/ros_ws/devel/lib/python2.7/dist-packages',
    '/home/jetauto/ros_ws/devel/lib/python3/dist-packages',
]:
    if os.path.exists(extra_p) and extra_p not in sys.path:
        sys.path.insert(0, extra_p)

if sys.version_info[0] == 2:
    from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
    from SocketServer import ThreadingMixIn
else:
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from socketserver import ThreadingMixIn

# ==========================================================
# CAU HINH HE THONG
# ==========================================================
# Lay ten vat the tu dong lenh (Mac dinh la person)
TARGET_CLASS = 'person'
if len(sys.argv) > 1:
    TARGET_CLASS = sys.argv[1].strip().lower()

IS_PAUSED = False

CMD_VEL_TOPIC = '/jetauto_controller/cmd_vel'
# Topic camera Astra (RGB). Thu cac ten pho bien:
# /camera/rgb/image_raw  (Astra voi astra_camera package)
# /depth_cam/rgb/image_raw (Astra voi depth_cam.launch)
CAMERA_TOPIC = '/depth_cam/rgb/image_raw'

global_frame = None
frame_lock = threading.Lock()
_frame_count = 0   # Dem frame de skip YOLO, tranh lag

class CamHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        global TARGET_CLASS, IS_PAUSED
        if self.path == '/set_target':
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = self.rfile.read(content_length)
            try:
                import json
                data = json.loads(post_data)
                new_target = data.get('target', '').strip().lower()
                if new_target:
                    TARGET_CLASS = new_target
                    IS_PAUSED = False  # Tu dong chay tiep khi doi muc tieu
                    rospy.loginfo(">> [WEB] Da doi muc tieu sang: " + TARGET_CLASS)
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    self.wfile.write(b'{"status": "ok"}')
                    return
            except Exception as e:
                print("Loi POST:", e)
            self.send_response(400)
            self.end_headers()
            
        elif self.path == '/toggle_pause':
            IS_PAUSED = not IS_PAUSED
            rospy.loginfo(">> [WEB] Trang thai tam dung: " + str(IS_PAUSED))
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            import json
            self.wfile.write(json.dumps({'paused': IS_PAUSED}).encode())
            return

    def do_GET(self):
        if self.path.endswith('.mjpg'):
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            try:
                while True:
                    frame = None
                    with frame_lock:
                        if global_frame is not None:
                            frame = global_frame.copy()

                    if frame is None:
                        # Tao frame thong bao ro rang khi chua co tin hieu camera
                        frame = np.zeros((480, 640, 3), dtype=np.uint8)
                        frame[:] = (35, 30, 25)
                        cv2.putText(frame, "DANG CHO TIN HIEU CAMERA...", (60, 200),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 220, 255), 2)
                        cv2.putText(frame, "Topic: " + str(CAMERA_TOPIC), (60, 240),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 128), 1)
                        cv2.putText(frame, "Kiem tra: Den camera Astra co sang khong?", (60, 280),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
                        cv2.putText(frame, "Neu den tat: Rut ra cam lai cong USB camera!", (60, 315),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 1)

                    ret, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                    if ret:
                        self.wfile.write("--jpgboundary\r\n".encode())
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', str(len(jpg)))
                        self.end_headers()
                        if sys.version_info[0] == 3:
                            self.wfile.write(jpg.tobytes())
                        else:
                            self.wfile.write(jpg.tostring())
                        self.wfile.write('\r\n'.encode())
                    time.sleep(0.05) # 20 FPS
            except Exception:
                pass
        else:
            self.send_response(200)
            self.send_header('Content-type', 'text/html; charset=utf-8')
            self.end_headers()
            html = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>JetAuto AI Tracker</title>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;600;800&display=swap" rel="stylesheet">
    <style>
        :root { --primary: #00E5FF; --bg-color: #0f172a; --glass-bg: rgba(255, 255, 255, 0.05); --glass-border: rgba(255, 255, 255, 0.1); }
        body { margin: 0; padding: 0; background-color: var(--bg-color); background-image: radial-gradient(circle at top right, #1e293b, #0f172a); color: #f8fafc; font-family: 'Inter', sans-serif; min-height: 100vh; display: flex; flex-direction: column; align-items: center; }
        .header { margin-top: 2rem; text-align: center; animation: fadeIn 1s ease-out; }
        .header h1 { font-weight: 800; font-size: 2.5rem; margin: 0; background: linear-gradient(to right, #00E5FF, #3B82F6); -webkit-background-clip: text; -webkit-text-fill-color: transparent; text-transform: uppercase; letter-spacing: 2px; }
        .header p { color: #94a3b8; font-size: 1.1rem; margin-top: 0.5rem; }
        .target-badge { display: inline-block; background: rgba(0, 229, 255, 0.15); border: 1px solid var(--primary); color: var(--primary); padding: 0.3rem 1rem; border-radius: 20px; font-weight: 600; font-size: 0.9rem; margin-left: 10px; text-transform: uppercase; box-shadow: 0 0 10px rgba(0, 229, 255, 0.2); vertical-align: middle; transition: all 0.3s ease; }
        .container { margin-top: 1.5rem; padding: 1.5rem; background: var(--glass-bg); border: 1px solid var(--glass-border); border-radius: 24px; box-shadow: 0 25px 50px -12px rgba(0, 0, 0, 0.5); backdrop-filter: blur(16px); -webkit-backdrop-filter: blur(16px); width: 90%; max-width: 800px; animation: slideUp 0.8s ease-out; }
        .stream-container { position: relative; width: 100%; min-height: 380px; border-radius: 16px; overflow: hidden; box-shadow: inset 0 0 0 1px rgba(255,255,255,0.1); background: #0b0f19; display: flex; justify-content: center; align-items: center; }
        .stream-img { width: 100%; height: auto; display: block; border-radius: 16px; transition: transform 0.3s ease; }
        .control-panel { margin-top: 1.5rem; display: flex; gap: 10px; }
        input[type="text"] { flex: 1; padding: 12px 20px; border-radius: 12px; border: 1px solid var(--glass-border); background: rgba(0,0,0,0.4); color: white; font-size: 1rem; outline: none; transition: border 0.3s; }
        input[type="text"]:focus { border-color: var(--primary); }
        button { padding: 12px 24px; border-radius: 12px; border: none; background: linear-gradient(to right, #00E5FF, #3B82F6); color: #000; font-weight: 800; font-size: 1rem; cursor: pointer; transition: transform 0.2s, box-shadow 0.2s; box-shadow: 0 4px 15px rgba(0, 229, 255, 0.3); }
        button:hover { transform: translateY(-2px); box-shadow: 0 6px 20px rgba(0, 229, 255, 0.4); }
        .btn-pause { background: linear-gradient(to right, #f43f5e, #e11d48); color: white; box-shadow: 0 4px 15px rgba(225, 29, 72, 0.3); }
        .btn-pause:hover { box-shadow: 0 6px 20px rgba(225, 29, 72, 0.4); }
        .btn-resume { background: linear-gradient(to right, #22c55e, #16a34a); color: white; box-shadow: 0 4px 15px rgba(34, 197, 94, 0.3); }
        .status-bar { margin-top: 1.5rem; display: flex; justify-content: space-between; align-items: center; padding: 1rem 1.5rem; background: rgba(0, 0, 0, 0.3); border-radius: 12px; border: 1px solid var(--glass-border); }
        .status-indicator { display: flex; align-items: center; gap: 10px; font-weight: 600; color: #cbd5e1; }
        .dot { width: 12px; height: 12px; background-color: #22c55e; border-radius: 50%; box-shadow: 0 0 10px #22c55e; animation: pulse 2s infinite; }
        @keyframes pulse { 0% { transform: scale(0.95); box-shadow: 0 0 0 0 rgba(34, 197, 94, 0.7); } 70% { transform: scale(1); box-shadow: 0 0 0 10px rgba(34, 197, 94, 0); } 100% { transform: scale(0.95); box-shadow: 0 0 0 0 rgba(34, 197, 94, 0); } }
        @keyframes fadeIn { from { opacity: 0; } to { opacity: 1; } }
        @keyframes slideUp { from { opacity: 0; transform: translateY(30px); } to { opacity: 1; transform: translateY(0); } }
    </style>
</head>
<body>
    <div class="header">
        <h1>JetAuto Vision<span class="target-badge" id="targetBadge">🎯 """ + TARGET_CLASS.upper() + """</span></h1>
        <p>Hệ thống AI nhận diện và bám đuổi mục tiêu thông minh</p>
    </div>
    <div class="container">
        <div class="stream-container">
            <img class="stream-img" id="streamImg" src="/cam.mjpg" alt="Live Camera Stream" onerror="setTimeout(function(){ document.getElementById('streamImg').src = '/cam.mjpg?t=' + new Date().getTime(); }, 1000);" />
        </div>
        <div class="control-panel">
            <input type="text" id="targetInput" placeholder="Nhập tiếng Anh (VD: bottle, cup, cat, phone...)" onkeypress="if(event.key==='Enter') setTarget()" />
            <button onclick="setTarget()">TÌM KIẾM</button>
            <button id="pauseBtn" class="btn-pause" onclick="togglePause()">TẠM DỪNG</button>
        </div>
        <div class="status-bar">
            <div class="status-indicator"><div class="dot" id="statusDot"></div><span id="statusText">Hệ thống đang hoạt động</span></div>
            <div class="status-indicator" style="color: var(--primary);">YOLO11 AI Engine</div>
        </div>
    </div>
    <script>
        function setTarget() {
            const target = document.getElementById('targetInput').value.trim();
            if (!target) return;
            fetch('/set_target', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({target: target})
            }).then(res => res.json()).then(data => {
                if(data.status === 'ok') {
                    document.getElementById('targetBadge').innerText = '🎯 ' + target.toUpperCase();
                    document.getElementById('targetInput').value = '';
                    updatePauseUI(false);
                }
            }).catch(err => alert("Loi ket noi: " + err));
        }
        function togglePause() {
            fetch('/toggle_pause', {method: 'POST'})
                .then(res => res.json())
                .then(data => updatePauseUI(data.paused))
                .catch(err => alert("Loi ket noi: " + err));
        }
        function updatePauseUI(isPaused) {
            const btn = document.getElementById('pauseBtn');
            const dot = document.getElementById('statusDot');
            const txt = document.getElementById('statusText');
            if(isPaused) {
                btn.className = 'btn-resume';
                btn.innerText = 'TIẾP TỤC';
                dot.style.backgroundColor = '#f59e0b';
                dot.style.boxShadow = '0 0 10px #f59e0b';
                txt.innerText = 'Đã tạm dừng';
            } else {
                btn.className = 'btn-pause';
                btn.innerText = 'TẠM DỪNG';
                dot.style.backgroundColor = '#22c55e';
                dot.style.boxShadow = '0 0 10px #22c55e';
                txt.innerText = 'Hệ thống đang hoạt động';
            }
        }
    </script>
</body>
</html>"""
            self.wfile.write(html.encode())

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass

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

class ObjectSeeker:
    def __init__(self):
        rospy.init_node('jetauto_object_seeker', anonymous=True)
        # Publisher van toc cho dong co banh xe (phat ra ca 3 topic pho bien)
        self.vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)
        self.vel_pub_std = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel_pub_ctrl = rospy.Publisher('/controller/cmd_vel', Twist, queue_size=1)
        
        # Ket noi truc tiep voi dong co chassis qua jetauto_sdk neu co
        self.chassis_driver = None
        try:
            from jetauto_sdk.mecanum import MecanumChassis
            try:
                self.chassis_driver = MecanumChassis()
                rospy.loginfo(">> [JetAuto] Da ket noi truc tiep dong co: MecanumChassis()")
            except TypeError:
                try:
                    self.chassis_driver = MecanumChassis(a=103, b=97, wheel_diameter=96.5, pulse_per_cycle=4320)
                    rospy.loginfo(">> [JetAuto] Da ket noi truc tiep dong co: MecanumChassis(params)")
                except Exception as ex_init:
                    rospy.logwarn(">> [JetAuto] Loi khoi tao MecanumChassis: " + str(ex_init))
        except Exception as ex_import:
            rospy.loginfo(">> [JetAuto] Khong import duoc jetauto_sdk.mecanum: " + str(ex_import))

        self.chassis_connected = (self.chassis_driver is not None)
        self.smooth_x = None
        self.smooth_w = None
        self._last_detections = []

        # --- TU DONG TIM TOPIC CAMERA ---
        camera_topic = self._find_camera_topic()
        rospy.loginfo(">>[JetAuto] Dung camera topic: " + camera_topic)

        # --- KHOI TAO YOLO ---
        self.classes = self.load_classes()
        model_path = os.path.join(os.path.dirname(__file__), 'yolov5n.onnx')
        if not os.path.exists(model_path):
            rospy.logerr(">> Khong tim thay file model: " + model_path)
            sys.exit(1)
        
        try:
            import onnxruntime as ort
            sess_opts = ort.SessionOptions()
            sess_opts.intra_op_num_threads = 4
            sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            avail = ort.get_available_providers()
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider'] if 'CUDAExecutionProvider' in avail else ['CPUExecutionProvider']
            self.session = ort.InferenceSession(model_path, sess_options=sess_opts, providers=providers)
            self.input_name = self.session.get_inputs()[0].name
            self.use_ort = True
            used = self.session.get_providers()[0]
            rospy.loginfo(">>[JetAuto] Tai model bang onnxruntime ({}) thanh cong!".format(used))
        except ImportError:
            rospy.logwarn(">>[JetAuto] Khong co onnxruntime, thu dung cv2.dnn...")
            self.net = cv2.dnn.readNetFromONNX(model_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.use_ort = False
        rospy.loginfo(">>[JetAuto] San sang!")

        # Trang thai nhan dien chia se giua AI va luong render video 30 FPS
        self.state_lock = threading.Lock()
        self.last_detections = []
        self.last_action_text = "DANG TIM KIEM..."
        self.last_status_color = (0, 165, 255)
        self.best_target_center = None
        self.ai_fps = 0.0

        # Bo nho duy tri muc tieu (Chong mat target dot ngot)
        self.last_seen_time = 0.0
        self.tracking_active = False
        self.last_target_box = None

        # Buffer frame moi nhat cho AI loop
        self.latest_raw_frame = None
        self.frame_sync_lock = threading.Lock()

        rospy.sleep(0.5)
        rospy.on_shutdown(self.stop)
        
        # queue_size=1 va buff_size=16MB
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        
        # Thread rieng xu ly AI chay ngam
        self.worker_thread = threading.Thread(target=self.ai_worker_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()
        
        rospy.loginfo(">>[JetAuto] He thong san sang tim vat the: {}".format(TARGET_CLASS.upper()))

    def _find_camera_topic(self):
        """Tu dong tim topic camera dang duoc publish"""
        global CAMERA_TOPIC
        PRIORITY_TOPICS = [
            '/depth_cam/rgb/image_raw',     # Astra Pro Plus (chinh xac tren JetAuto)
            '/depth_cam/color/image_raw',
            '/camera/rgb/image_raw',
            '/camera/color/image_raw',
            '/depth_cam/image_raw',
            '/usb_cam/image_raw',
            '/image_raw',
        ]
        rospy.loginfo(">>[JetAuto] Dang tim topic camera (cho toi da 15 giay)...")
        deadline = rospy.Time.now() + rospy.Duration(15.0)
        while rospy.Time.now() < deadline:
            try:
                published = [t for t, _ in rospy.get_published_topics()]
                for topic in PRIORITY_TOPICS:
                    if topic in published:
                        CAMERA_TOPIC = topic
                        rospy.loginfo(">>[JetAuto] Da tim thay va dung camera topic: " + topic)
                        return topic
            except Exception:
                pass
            rospy.sleep(0.5)
        rospy.logwarn(">>[JetAuto] Khong tim thay topic camera dang publish! Dung mac dinh: " + CAMERA_TOPIC)
        return CAMERA_TOPIC

    def load_classes(self):

        path = os.path.join(os.path.dirname(__file__), 'coco.names')
        if os.path.exists(path):
            with open(path, 'r') as f:
                return [x.strip() for x in f if x.strip()]
        return ['person','bicycle','car','motorcycle','airplane','bus','train','truck','boat','traffic light','fire hydrant','stop sign','parking meter','bench','bird','cat','dog','horse','sheep','cow','elephant','bear','zebra','giraffe','backpack','umbrella','handbag','tie','suitcase','frisbee','skis','snowboard','sports ball','kite','baseball bat','baseball glove','skateboard','surfboard','tennis racket','bottle','wine glass','cup','fork','knife','spoon','bowl','banana','apple','sandwich','orange','broccoli','carrot','hot dog','pizza','donut','cake','chair','couch','potted plant','bed','dining table','toilet','tv','laptop','mouse','remote','keyboard','cell phone','microwave','oven','toaster','sink','refrigerator','book','clock','vase','scissors','teddy bear','hair drier','toothbrush']

    def set_velocity(self, linear_x=0.0, angular_z_deg=0.0):
        # 1. Truyen truc tiep den dong co qua SDK neu co
        if self.chassis_driver is not None:
            rad_z = math.radians(angular_z_deg)
            try:
                # Thu chuan 1: set_velocity(linear_x, linear_y, angular_z)
                self.chassis_driver.set_velocity(float(linear_x), 0.0, float(rad_z))
            except TypeError:
                try:
                    # Thu chuan 2: set_velocity(speed, direction, angular_z)
                    spd = int(abs(linear_x) * 100) # cm/s
                    direction = 0 if linear_x >= 0 else 180
                    self.chassis_driver.set_velocity(spd, direction, float(angular_z_deg))
                except Exception:
                    pass
            except Exception:
                pass

        # 2. Dong thoi phat qua tat ca cac topic ROS
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.angular.z = float(math.radians(angular_z_deg))
        self.vel_pub.publish(twist_msg)
        self.vel_pub_std.publish(twist_msg)
        self.vel_pub_ctrl.publish(twist_msg)

        # Kiem tra xem co node chassis nao dang lang nghe khong (hoac da co SDK ket noi)
        subs = self.vel_pub.get_num_connections() + self.vel_pub_std.get_num_connections() + self.vel_pub_ctrl.get_num_connections()
        self.chassis_connected = (subs > 0) or (self.chassis_driver is not None)
        if not self.chassis_connected and (abs(linear_x) > 0.001 or abs(angular_z_deg) > 0.001):
            rospy.logwarn_throttle(5.0, ">> [JetAuto CANH BAO] Dang phat lenh chay nhung chua co driver khung gam nao ket noi!")

    def run_yolo(self, frame):
        INPUT_SIZE = 640
        h, w = frame.shape[:2]
        x_scale = w / float(INPUT_SIZE)
        y_scale = h / float(INPUT_SIZE)

        # 1. Chuan bi anh bang OpenCV C++ blobFromImage (cuc nhanh, ~2ms thay vi 80ms)
        blob = cv2.dnn.blobFromImage(frame, 1.0/255.0, (INPUT_SIZE, INPUT_SIZE), swapRB=True, crop=False)

        # 2. Chay inference
        if self.use_ort:
            outputs = self.session.run(None, {self.input_name: blob})
            arr = outputs[0]
        else:
            self.net.setInput(blob)
            arr = self.net.forward()

        # 3. Giai ma YOLO bang Vectorized NumPy (nhanh gap 1000 lan for-loop Python)
        arr = np.asarray(arr)
        if arr.ndim == 3:
            arr = np.squeeze(arr, axis=0)
        if arr.shape[0] < arr.shape[1]:
            arr = arr.T

        if len(arr) == 0:
            return []

        # Loc truoc cac anchor box bang Vectorized array
        if arr.shape[1] == 85: # YOLOv5 format [25200, 85]: x, y, w, h, obj_conf, 80 scores
            obj_mask = arr[:, 4] > 0.25
            arr = arr[obj_mask]
            if len(arr) == 0:
                return []
            scores = arr[:, 5:]
            class_ids = np.argmax(scores, axis=1)
            confidences = np.max(scores, axis=1) * arr[:, 4]
        else: # YOLOv8/11 format [8400, 84]
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

    def decode_ros_image(self, msg):
        enc = msg.encoding.lower() if hasattr(msg, 'encoding') else 'rgb8'
        try:
            if enc == 'bgr8':
                return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            elif enc == 'rgb8':
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
            elif enc in ['bgra8', 'bgrx8']:
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
                return cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
            elif enc in ['rgba8', 'rgbx8']:
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 4))
                return cv2.cvtColor(raw, cv2.COLOR_RGBA2BGR)
            elif enc in ['yuv422', 'yuyv']:
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 2))
                return cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_YUYV)
            elif enc == 'uyvy':
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 2))
                return cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_UYVY)
            elif enc == 'mono8':
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                return cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
            else:
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                if arr.size == msg.height * msg.width * 3:
                    return arr.reshape((msg.height, msg.width, 3))
                elif arr.size == msg.height * msg.width * 4:
                    return cv2.cvtColor(arr.reshape((msg.height, msg.width, 4)), cv2.COLOR_BGRA2BGR)
                elif arr.size == msg.height * msg.width * 2:
                    return cv2.cvtColor(arr.reshape((msg.height, msg.width, 2)), cv2.COLOR_YUV2BGR_YUYV)
                return arr.reshape((msg.height, msg.width, -1))
        except Exception as e:
            rospy.logerr_throttle(2.0, ">> [JetAuto] Loi decode frame (encoding: {}): {}".format(enc, e))
            return None

    def image_callback(self, msg):
        global global_frame
        
        if not hasattr(self, '_first_frame_logged'):
            self._first_frame_logged = True
            rospy.loginfo(">> [JetAuto] DA NHAN FRAME DAU TIEN! Size: {}x{}, Encoding: {}".format(msg.width, msg.height, msg.encoding))

        # 1. Luu frame moi nhat cho AI loop xu ly
        with self.frame_sync_lock:
            self.latest_raw_frame = msg

        # 2. Render luong video truc tiep 30 FPS cho Web voi do tre 0ms
        try:
            frame = self.decode_ros_image(msg)
            if frame is None:
                return

            h, w = frame.shape[:2]
            center_screen_x = w // 2
            cv2.line(frame, (center_screen_x, 0), (center_screen_x, h), (255, 255, 255), 1)

            # Lay trang thai moi nhat tu AI worker
            with self.state_lock:
                detections = list(self.last_detections)
                action_text = self.last_action_text
                status_color = self.last_status_color
                target_center = self.best_target_center
                ai_fps = self.ai_fps

            for det in detections:
                bx, by, bw, bh = det['box']
                color = (0, 255, 0) if det['class_name'] == TARGET_CLASS else (100, 100, 100)
                cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), color, 2)
                cv2.putText(frame, "{} {:.0f}%".format(det['class_name'], det['conf']*100), (bx, by-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            if target_center is not None:
                cv2.circle(frame, target_center, 5, (0, 0, 255), -1)

            cv2.putText(frame, action_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            cv2.putText(frame, "AI FPS: {:.1f}".format(ai_fps), (w - 140, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            if hasattr(self, 'chassis_connected') and not self.chassis_connected:
                cv2.putText(frame, "! CHASSIS (DONG CO) CHUA BAT !", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

            with frame_lock:
                global_frame = frame
        except Exception as e:
            rospy.logerr_throttle(2.0, ">> [JetAuto LOI image_callback]: " + str(e))

    def ai_worker_loop(self):
        frame_counter = 0
        last_fps_time = time.time()
        current_ai_fps = 0.0

        while not rospy.is_shutdown():
            loop_start = time.time()
            msg = None
            with self.frame_sync_lock:
                if self.latest_raw_frame is not None:
                    msg = self.latest_raw_frame
                    self.latest_raw_frame = None
            
            if msg is None:
                time.sleep(0.005)
                continue

            frame_counter += 1
            now = time.time()
            if now - last_fps_time >= 1.0:
                current_ai_fps = frame_counter / (now - last_fps_time)
                frame_counter = 0
                last_fps_time = now

            frame = self.decode_ros_image(msg)
            if frame is None:
                continue

            h, w = frame.shape[:2]
            center_screen_x = w // 2

            # Chay YOLO
            detections = self.run_yolo(frame)

            # NEU DANG TAM DUNG
            if IS_PAUSED:
                self.set_velocity(0.0, 0.0)
                with self.state_lock:
                    self.last_detections = detections
                    self.last_action_text = "TAM DUNG (PAUSED)"
                    self.last_status_color = (0, 165, 255)
                    self.best_target_center = None
                    self.ai_fps = current_ai_fps
                continue

            target_found = False
            action_text = "DANG TIM KIEM..."
            status_color = (0, 165, 255)
            best_target = None
            max_score = -1.0

            # Uu tien target o gan vi tri cu de tranh bi nhay giua cac muc tieu
            last_cx, last_cy = None, None
            if self.last_target_box is not None:
                lx, ly, lw, lh = self.last_target_box
                last_cx, last_cy = lx + lw // 2, ly + lh // 2

            for det in detections:
                cls_name = det['class_name']
                if cls_name == TARGET_CLASS:
                    bx, by, bw, bh = det['box']
                    area = bw * bh
                    if last_cx is not None:
                        dcx = (bx + bw // 2) - last_cx
                        dcy = (by + bh // 2) - last_cy
                        dist = math.sqrt(dcx * dcx + dcy * dcy)
                        score = float(area) / (1.0 + dist * 0.3)
                    else:
                        score = float(area)

                    if score > max_score:
                        max_score = score
                        best_target = det

            target_center = None
            if best_target:
                target_found = True
                self.last_seen_time = time.time()
                self.tracking_active = True
                self.last_target_box = best_target['box']

                bx, by, bw, bh = best_target['box']
                cx = bx + bw // 2
                cy = by + bh // 2
                target_center = (cx, cy)

                if self.smooth_x is None:
                    self.smooth_x = cx
                    self.smooth_w = bw
                else:
                    self.smooth_x = 0.65 * self.smooth_x + 0.35 * cx
                    self.smooth_w = 0.7 * self.smooth_w + 0.3 * bw

                error_x = self.smooth_x - center_screen_x
                width_ratio = self.smooth_w / float(w)

                if width_ratio > 0.45:
                    action_text = "DA DEN GAN! DUNG LAI"
                    status_color = (255, 0, 0)
                    self.set_velocity(linear_x=0.0, angular_z_deg=0.0)
                else:
                    if error_x < -35:
                        action_text = "LECH TRAI -> RE TRAI"
                        status_color = (0, 255, 255)
                        self.set_velocity(linear_x=0.06, angular_z_deg=14.0)
                    elif error_x > 35:
                        action_text = "LECH PHAI -> RE PHAI"
                        status_color = (0, 255, 255)
                        self.set_velocity(linear_x=0.06, angular_z_deg=-14.0)
                    else:
                        action_text = "CHINH GIUA -> TIEN THANG!"
                        status_color = (0, 255, 0)
                        self.set_velocity(linear_x=0.15, angular_z_deg=0.0)
            else:
                time_since_seen = time.time() - self.last_seen_time if self.tracking_active else 999.0

                # GIAI DOAN 1: Mat target trong vong 1.5s -> DUNG CHO, KHONG QUAY GIAT MINH!
                if self.tracking_active and time_since_seen < 1.5:
                    action_text = "GIU MUC TIEU ({:.1f}s)...".format(1.5 - time_since_seen)
                    status_color = (0, 215, 255)  # Vang cam
                    self.set_velocity(linear_x=0.0, angular_z_deg=0.0)
                    if self.last_target_box is not None:
                        lx, ly, lw, lh = self.last_target_box
                        target_center = (lx + lw // 2, ly + lh // 2)

                # GIAI DOAN 2: Thuc su mat qua 1.5s -> Xac nhan mat, moi bat dau xoay tim
                else:
                    self.tracking_active = False
                    self.smooth_x = None
                    self.last_target_box = None
                    action_text = "TIM {}".format(TARGET_CLASS.upper())
                    status_color = (0, 0, 255)
                    self.set_velocity(linear_x=0.0, angular_z_deg=22.0)

            with self.state_lock:
                self.last_detections = detections
                self.last_action_text = action_text
                self.last_status_color = status_color
                self.best_target_center = target_center
                self.ai_fps = current_ai_fps

            # Gioi han toc do loop o muc 20 FPS (50ms) theo yeu cau
            elapsed = time.time() - loop_start
            if elapsed < 0.05:
                time.sleep(0.05 - elapsed)

    def stop(self):
        rospy.loginfo(">> [JetAuto Pro] Dang dung robot an toan...")
        if self.chassis_driver is not None:
            try:
                self.chassis_driver.set_velocity(0.0, 0.0, 0.0)
            except Exception:
                pass
        self.set_velocity(linear_x=0.0, angular_z_deg=0.0)

if __name__ == '__main__':
    seeker = None

    def handle_exit(sig, frame):
        if seeker is not None:
            try:
                seeker.stop()
            except Exception:
                pass
        os._exit(0)

    try:
        signal.signal(signal.SIGINT, handle_exit)
        signal.signal(signal.SIGTERM, handle_exit)
    except Exception:
        pass

    try:
        web_thread = threading.Thread(target=start_web_server)
        web_thread.daemon = True
        web_thread.start()
        seeker = ObjectSeeker()
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    finally:
        if seeker is not None:
            try:
                seeker.stop()
            except Exception:
                pass
        os._exit(0)
