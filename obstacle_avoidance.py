#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Né vật cản cho JetAuto Pro (bánh Mecanum) - LiDAR G4 + camera độ sâu Astra Pro Plus.
Module không phụ thuộc ROS để kiểm thử được trên máy tính (test_obstacle_avoidance.py).

Hệ tọa độ xe: x tiến, y sang trái, z lên trên (mét, radian, góc dương = quay trái).
Bộ nhớ ngắn hạn lưu theo khung odom để vật cản không "biến mất" khi lọt vào vùng mù.

Luồng xử lý mỗi chu kỳ (10 Hz):
  LidarModel     : tia LiDAR -> điểm vật cản + tự học các góc bị khung xe che (góc mù)
  AstraDepth     : ảnh độ sâu -> tự hiệu chuẩn mặt sàn -> điểm vật cản / sàn trống / hố (mép bàn)
  LocalGrid      : bản đồ nhớ quanh xe, cảm biến nào chỉ xóa vật cản của chính cảm biến đó
  MotionPlanner  : chọn hướng tịnh tiến kiểu VFH cho Mecanum, đầu xe vẫn nhìn mục tiêu,
                   chốt bên né để không lắc trái-phải, tự thoát khi lỡ đứng quá gần vật
  motion_is_safe : mô phỏng quét thân xe chữ nhật lần cuối trước khi gửi lệnh động cơ

Các số đo mặc định lấy trên robot ngày 05/10/2026 (xem AvoidanceConfig).
"""
from __future__ import division, print_function
import math

import numpy as np

try:
    import cv2
except ImportError:          # chỉ phần vẽ radar / lọc nhiễu cần OpenCV
    cv2 = None

TWO_PI = 2.0 * math.pi


def wrap(angle):
    """Đưa góc (số hoặc mảng) về khoảng [-pi, pi)."""
    return (np.asarray(angle, dtype=float) + math.pi) % TWO_PI - math.pi


def wrap_scalar(angle):
    return (angle + math.pi) % TWO_PI - math.pi


# ==========================================================
# CẤU HÌNH (ĐO THỰC TẾ TRÊN ROBOT)
# ==========================================================
class AvoidanceConfig(object):
    """Thông số né vật cản. Truyền keyword để ghi đè, ví dụ AvoidanceConfig(margin=0.08)."""

    def __init__(self, **overrides):
        # Thân xe đo được 32.4 x 26.0 cm, tâm trùng base_footprint
        self.body_length = 0.324
        self.body_width = 0.260
        self.margin = 0.06           # khoảng hở muốn giữ khi lập đường đi
        self.final_margin = 0.025    # khoảng hở tối thiểu ở bước kiểm tra cuối cùng
        self.self_filter = 0.03      # điểm sát thân hơn mức này là chính robot (tay máy, dây cáp)
        # LiDAR G4 (URDF): trước tâm xe 9.6 cm, mặt quét cao ~13 cm
        self.lidar_x = 0.096
        self.lidar_y = 0.0
        self.lidar_yaw = 0.0
        self.lidar_max_range = 6.0
        # Góc LiDAR bị khung/nắp xe che (đo 05/10/2026: 0% tia hợp lệ từ +120° qua 180° tới -90°).
        # Cấu hình cố định vì "không có tia về" ở chỗ khác có thể chỉ là không gian trống / kính.
        self.lidar_blind_deg = ((120.0, 180.0), (-180.0, -90.0))
        self.lidar_learn_blind = False      # True: thêm cả các góc tự học được là luôn mù
        self.lidar_blind_threshold = 0.04   # ngưỡng tự học: < 4% tia hợp lệ trong thời gian dài
        # Astra (URDF JetAuto Pro): sau tâm 13.1 cm; độ cao / góc chúi tự hiệu chuẩn theo mặt sàn
        self.astra_x = -0.131
        self.astra_y = 0.02
        self.astra_height = 0.462
        self.astra_pitch_deg = 7.9
        self.astra_roll_deg = 0.0
        # RANSAC chỉ được tinh chỉnh quanh tư thế gá đã đo. Nếu camera nhìn ít sàn,
        # mặt ghế/bàn có thể chiếm đa số ảnh và bị nhận nhầm là sàn; không có giới
        # hạn tuyệt đối thì bộ bám mặt phẳng còn tích lũy sai số qua từng frame.
        self.astra_calib_tilt_tolerance_deg = 10.0
        self.astra_calib_height_tolerance = 0.10
        self.astra_track_tilt_tolerance_deg = 5.0
        self.astra_track_height_tolerance = 0.05
        self.astra_stride = 4
        self.astra_min_depth = 0.30
        self.astra_max_range = 3.2
        self.obstacle_min_height = 0.045
        self.obstacle_max_height = 0.65
        self.floor_tolerance = 0.03
        # Giới hạn chuyển động (m/s, độ/s)
        self.max_forward = 0.18
        self.max_lateral = 0.12
        self.max_backward = 0.06
        self.min_speed = 0.035
        self.max_turn_deg = 30.0
        self.min_turn_deg = 8.0
        self.brake_decel = 0.35
        self.reaction_time = 0.25
        self.accel = 0.40
        self.turn_accel_deg = 90.0
        self.lookahead = 0.70        # vật cản gần hơn mức này trên hướng đi -> bắt đầu lách sớm
        self.plan_radius = 1.6       # chỉ xét vật trong bán kính này (xa hơn không đổi quyết định)
        # Bộ nhớ (giây). Vật cản được XÓA ngay khi cảm biến nhìn lại chỗ đó thấy trống; thời hạn
        # dưới đây chỉ là mức trần cho vùng không thể nhìn lại (góc mù LiDAR, sát mũi xe dưới
        # khung hình Astra) - nếu quên sớm, xe sẽ đâm vào vật vừa thấy cách đây vài giây.
        self.lidar_memory_s = 30.0
        self.astra_memory_s = 60.0
        self.free_memory_s = 30.0
        self.target_memory_s = 5.0
        # Khoảng dừng trước mục tiêu: tính từ mép trước xe tới bề mặt mục tiêu (m)
        self.standoff = {'person': 0.55, 'chair': 0.40, 'couch': 0.45, 'bed': 0.45,
                         'dining table': 0.40, 'tv': 0.50, 'refrigerator': 0.40}
        self.default_standoff = 0.35
        # Bề rộng thật của vật (m) để ước lượng khoảng cách khi không có độ sâu
        self.target_width = {'person': 0.42, 'chair': 0.48, 'couch': 1.5, 'dining table': 0.9,
                             'suitcase': 0.45, 'cell phone': 0.075, 'bottle': 0.07, 'cup': 0.09,
                             'remote': 0.05, 'laptop': 0.33, 'tv': 0.75, 'book': 0.16,
                             'backpack': 0.30, 'sports ball': 0.22, 'teddy bear': 0.25}
        for key, value in overrides.items():
            if not hasattr(self, key):
                raise KeyError('Unknown avoidance setting: ' + key)
            setattr(self, key, value)

    @property
    def hx(self):
        return self.body_length / 2.0

    @property
    def hy(self):
        return self.body_width / 2.0

    def standoff_for(self, label):
        return self.standoff.get(label, self.default_standoff)


# ==========================================================
# HÌNH HỌC THÂN XE CHỮ NHẬT
# ==========================================================
def transform_points(pose, points):
    """Điểm trong khung xe -> khung odom. pose = (x, y, yaw)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((x + c * pts[:, 0] - s * pts[:, 1],
                            y + s * pts[:, 0] + c * pts[:, 1]))


def inverse_transform_points(pose, points):
    """Điểm trong khung odom -> khung xe."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = pts[:, 0] - x, pts[:, 1] - y
    return np.column_stack((c * dx + s * dy, -s * dx + c * dy))


def body_clearance(points, hx, hy):
    """Khoảng cách từ điểm (khung xe) tới mép thân xe; bằng 0 nếu nằm trong thân."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    dx = np.maximum(np.abs(pts[:, 0]) - hx, 0.0)
    dy = np.maximum(np.abs(pts[:, 1]) - hy, 0.0)
    return np.hypot(dx, dy)


def box_sweep_distance(points, directions, hx, hy):
    """Quãng đường thân xe (nửa kích thước hx, hy) tịnh tiến được theo từng hướng
    trước khi chạm điểm gần nhất. Trả về mảng (K,): 0 nếu điểm đã nằm trong, inf nếu không chạm."""
    k = len(directions)
    if points is None or len(points) == 0:
        return np.full(k, np.inf)
    px = points[:, 0][None, :]
    py = points[:, 1][None, :]
    ux = np.cos(directions)
    uy = np.sin(directions)
    # Thành phần bằng 0 -> số rất nhỏ: phép chia cho ra +-vô cùng đúng ý nghĩa hình học
    ux = np.where(np.abs(ux) < 1e-12, 1e-12, ux)[:, None]
    uy = np.where(np.abs(uy) < 1e-12, 1e-12, uy)[:, None]
    # Phương pháp slab: thời điểm điểm đi vào / ra khỏi hộp đang tịnh tiến theo u
    inv_x, inv_y = 1.0 / ux, 1.0 / uy
    sx, sy = np.sign(ux), np.sign(uy)
    t_in = np.maximum((px - hx * sx) * inv_x, (py - hy * sy) * inv_y)
    t_out = np.minimum((px + hx * sx) * inv_x, (py + hy * sy) * inv_y)
    t_in[(t_in > t_out) | (t_out < 0.0)] = np.inf
    np.maximum(t_in, 0.0, out=t_in)
    return t_in.min(axis=1)


def thin_points(points, cell=0.03):
    """Gộp các điểm rơi cùng một ô (giảm tải cho bộ lập đường)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) < 2:
        return pts
    keys = np.floor(pts / cell).astype(np.int64)
    _, first = np.unique(keys[:, 0] * 1000003 + keys[:, 1], return_index=True)
    return pts[np.sort(first)]


def speed_for_clearance(clearance, decel, reaction):
    """Tốc độ lớn nhất còn phanh kịp trong quãng 'clearance' (tính cả thời gian phản ứng)."""
    d = np.maximum(np.asarray(clearance, dtype=float), 0.0)
    d = np.where(np.isfinite(d), d, 1e3)
    return decel * (np.sqrt(reaction * reaction + 2.0 * d / decel) - reaction)


def direction_speed_cap(directions, cfg):
    """Giới hạn tốc độ theo hướng (elip): tiến nhanh, đi ngang vừa, lùi chậm."""
    c, s = np.cos(directions), np.sin(directions)
    vx_lim = np.where(c >= 0.0, cfg.max_forward, cfg.max_backward)
    return 1.0 / np.sqrt((c / vx_lim) ** 2 + (s / cfg.max_lateral) ** 2)


def motion_is_safe(points, vx, vy, wz_deg, cfg, margin=None, horizon=None, steps=10):
    """Mô phỏng thân xe chữ nhật chạy theo (vx, vy, wz) trong thời gian phản ứng + phanh.

    Trả về (an_toan, ly_do). Điểm đã nằm trong vùng đệm từ trước (xe lỡ đứng quá gần)
    chỉ được phép xa dần, nhờ vậy xe luôn tự thoát ra được mà không đâm thêm."""
    if abs(vx) + abs(vy) < 1e-6 and abs(wz_deg) < 1e-6:
        return True, ''
    if points is None or len(points) == 0:
        return True, ''
    margin = cfg.final_margin if margin is None else margin
    hx, hy = cfg.hx, cfg.hy
    speed = math.hypot(vx, vy)
    wz = math.radians(wz_deg)
    edge_speed = speed + abs(wz) * math.hypot(hx, hy)
    if horizon is None:
        horizon = cfg.reaction_time + edge_speed / cfg.brake_decel + 0.15
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    reach = math.hypot(hx, hy) + margin + edge_speed * horizon + 0.05
    pts = pts[(np.abs(pts[:, 0]) <= reach) & (np.abs(pts[:, 1]) <= reach)]
    if not len(pts):
        return True, ''
    clear0 = body_clearance(pts, hx, hy)
    inside0 = clear0 < margin
    for t in np.linspace(0.0, horizon, steps)[1:]:
        th = wz * t
        if abs(wz) > 1e-6:
            x = (vx * math.sin(th) + vy * (math.cos(th) - 1.0)) / wz
            y = (vx * (1.0 - math.cos(th)) + vy * math.sin(th)) / wz
        else:
            x, y = vx * t, vy * t
        c, s = math.cos(th), math.sin(th)
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        clear = body_clearance(np.column_stack((c * dx + s * dy, -s * dx + c * dy)), hx, hy)
        if np.any(clear <= 1e-4):
            p = pts[int(np.argmin(clear))]
            return False, 'CHAM THAN XE X={:.0f} Y={:.0f}CM'.format(p[0] * 100, p[1] * 100)
        bad = (clear < margin) & (~inside0 | (clear < clear0 - 0.005))
        if np.any(bad):
            p = pts[bad][0]
            return False, 'VAT CAN X={:.0f} Y={:.0f}CM'.format(p[0] * 100, p[1] * 100)
    return True, ''


# ==========================================================
# LIDAR: ĐIỂM VẬT CẢN + HỌC GÓC MÙ DO KHUNG XE CHE
# ==========================================================
class LidarScan(object):
    __slots__ = ('points', 'origin', 'stamp', 'valid_fraction', 'fan')

    def __init__(self, points, origin, stamp, valid_fraction, fan=None):
        self.points = points            # điểm chạm vật (khung xe)
        self.origin = origin
        self.stamp = stamp
        self.valid_fraction = valid_fraction
        self.fan = fan                  # đa giác vùng nhìn thấy trống (khung xe)


class LidarModel(object):
    """Đổi /scan thành điểm khung xe. Tia = 0 nghĩa là KHÔNG BIẾT (không phải thoáng).
    Tỉ lệ tia hợp lệ theo từng góc được làm mượt; góc gần như luôn = 0 là góc bị khung xe che."""

    def __init__(self, cfg, bin_deg=5.0):
        self.cfg = cfg
        self.bins = int(round(360.0 / bin_deg))
        self.valid_ema = None
        self.updates = 0
        centres = np.degrees(-math.pi + (np.arange(self.bins) + 0.5) * TWO_PI / self.bins)
        self.configured = np.zeros(self.bins, dtype=bool)
        for lo, hi in cfg.lidar_blind_deg:
            self.configured |= (centres >= lo) & (centres <= hi)

    def bin_index(self, bearings):
        b = wrap(bearings)
        return np.floor((b + math.pi) / TWO_PI * self.bins).astype(int) % self.bins

    def learned_blind_bins(self):
        """Góc gần như chưa bao giờ có tia hợp lệ (chẩn đoán: dây cáp, nắp xe che...)."""
        if self.valid_ema is None or self.updates < 20:
            return np.zeros(self.bins, dtype=bool)
        return self.valid_ema < self.cfg.lidar_blind_threshold

    def blind_bins(self):
        if self.cfg.lidar_learn_blind:
            return self.configured | self.learned_blind_bins()
        return self.configured.copy()

    def is_blind(self, points):
        """Điểm (khung xe) có nằm trong góc LiDAR không nhìn thấy không."""
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if not len(pts):
            return np.zeros(0, dtype=bool)
        cfg = self.cfg
        bearing = np.arctan2(pts[:, 1] - cfg.lidar_y, pts[:, 0] - cfg.lidar_x) - cfg.lidar_yaw
        return self.blind_bins()[self.bin_index(bearing)]

    def process(self, ranges, angle_min, angle_increment, range_min, range_max, stamp):
        cfg = self.cfg
        r = np.asarray(ranges, dtype=float)
        a = angle_min + np.arange(r.size) * angle_increment
        valid = np.isfinite(r) & (r >= max(range_min, 0.10)) & (r <= range_max)
        idx = self.bin_index(a)
        total = np.bincount(idx, minlength=self.bins).astype(float)
        good = np.bincount(idx, weights=valid.astype(float), minlength=self.bins)
        frac = np.where(total > 0, good / np.maximum(total, 1.0), 0.0)
        self.valid_ema = frac if self.valid_ema is None else 0.95 * self.valid_ema + 0.05 * frac
        self.updates += 1
        ang = a + cfg.lidar_yaw
        rr = np.where(valid, r, 0.0)
        pts = np.column_stack((cfg.lidar_x + rr * np.cos(ang), cfg.lidar_y + rr * np.sin(ang)))
        # Bỏ tia chạm chính thân xe (dây cáp, cột đỡ); sau chỗ đó là vùng không biết
        own = valid & (body_clearance(pts, cfg.hx, cfg.hy) < cfg.self_filter)
        seen = valid & ~own
        hits = seen & (r <= cfg.lidar_max_range)
        # Đa giác vùng trống: theo thứ tự góc, tia không hợp lệ co về gốc LiDAR; lấy tầm ngắn
        # nhất của 3 tia kề nhau và lùi 6 cm để không "xóa" lấn sang vật ở mép tia
        order = np.argsort(wrap(ang))
        reach = np.where(seen, np.minimum(r, cfg.lidar_max_range), np.inf)[order]
        reach = np.minimum(np.minimum(reach, np.roll(reach, 1)), np.roll(reach, -1))
        reach = np.where(seen[order], np.maximum(reach - 0.06, 0.0), 0.0)
        ao = ang[order]
        fan = np.column_stack((cfg.lidar_x + reach * np.cos(ao), cfg.lidar_y + reach * np.sin(ao)))
        return LidarScan(pts[hits], (cfg.lidar_x, cfg.lidar_y), stamp,
                         float(valid.mean()) if r.size else 0.0, fan)


# ==========================================================
# ASTRA: TỰ HIỆU CHUẨN MẶT SÀN + PHÂN LOẠI ĐIỂM ĐỘ SÂU
# ==========================================================
class AstraDepth(object):
    """Ảnh độ sâu (mm) -> điểm khung xe: vật cản (cao 4.5..65 cm), sàn trống, hố/mép bàn.

    Mặt sàn được theo dõi liên tục (bình phương tối thiểu trên các điểm gần mặt sàn cũ), nên
    nếu ai đó chỉnh góc camera thì chỉ cần gọi calibrate() khi robot nhìn thấy sàn."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.intrinsics = None
        self.shape = None
        self._rays = None
        pitch = math.radians(cfg.astra_pitch_deg)
        roll = math.radians(cfg.astra_roll_deg)
        # Vector "lên trên" trong khung quang học (x phải, y xuống, z tới trước)
        up = np.array([math.cos(pitch) * math.sin(roll), -math.cos(pitch) * math.cos(roll), -math.sin(pitch)])
        self.nominal_up = up.copy()
        self.nominal_height = float(cfg.astra_height)
        self.set_plane(up, cfg.astra_height)
        self.quality = 0.0
        self.source = 'MAC DINH'
        self.calibrated_at = 0.0

    def set_plane(self, up, height):
        up = np.asarray(up, dtype=float)
        up /= np.linalg.norm(up)
        self.up = up
        self.height = float(height)
        forward = np.array([0.0, 0.0, 1.0]) - up[2] * up
        forward /= np.linalg.norm(forward)
        self.forward = forward
        self.left = np.cross(up, forward)

    def _plane_is_plausible(self, up, height, tracking=False):
        """Giữ hiệu chuẩn gần tư thế gá thật, không bám nhầm mặt ghế/bàn."""
        up = np.asarray(up, dtype=float)
        norm = np.linalg.norm(up)
        if norm < 1e-9 or not np.isfinite(height):
            return False
        dot = float(np.dot(up / norm, self.nominal_up))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
        tilt_limit = (self.cfg.astra_track_tilt_tolerance_deg if tracking
                      else self.cfg.astra_calib_tilt_tolerance_deg)
        height_limit = (self.cfg.astra_track_height_tolerance if tracking
                        else self.cfg.astra_calib_height_tolerance)
        return (tilt <= tilt_limit
                and abs(float(height) - self.nominal_height) <= height_limit)

    @property
    def pitch_deg(self):
        return math.degrees(math.asin(max(-1.0, min(1.0, -self.up[2]))))

    @property
    def roll_deg(self):
        return math.degrees(math.atan2(self.up[0], -self.up[1]))

    def set_intrinsics(self, fx, fy, cx, cy, width, height):
        intr = (float(fx), float(fy), float(cx), float(cy))
        if intr == self.intrinsics and self.shape == (height, width):
            return
        self.intrinsics = intr
        self.shape = (height, width)
        step = self.cfg.astra_stride
        v, u = np.mgrid[0:height:step, 0:width:step]
        self._rays = np.dstack(((u - cx) / fx, (v - cy) / fy)).astype(np.float32)

    def to_base(self, pts_cam):
        """Điểm khung quang học -> (x, y, cao) khung xe."""
        p = np.asarray(pts_cam, dtype=float).reshape(-1, 3)
        return np.column_stack((self.cfg.astra_x + p.dot(self.forward),
                                self.cfg.astra_y + p.dot(self.left),
                                p.dot(self.up) + self.height))

    def _points(self, depth_mm, max_range=None):
        step = self.cfg.astra_stride
        d = np.asarray(depth_mm)[::step, ::step]
        if d.dtype != np.uint16:
            d = np.clip(d, 0, 65535).astype(np.uint16)
        # Loại điểm "bay" ở mép vật (độ sâu nhảy giữa tiền cảnh và hậu cảnh)
        med = cv2.medianBlur(d, 3).astype(np.float32) if cv2 is not None else d.astype(np.float32)
        z = d.astype(np.float32) / 1000.0
        far = self.cfg.astra_max_range if max_range is None else max_range
        valid = ((z >= self.cfg.astra_min_depth) & (z <= far)
                 & (np.abs(d.astype(np.float32) - med) <= 0.04 * med + 5.0))
        rays = self._rays[:z.shape[0], :z.shape[1]]
        zz = z[valid]
        pts = np.column_stack((rays[..., 0][valid] * zz, rays[..., 1][valid] * zz, zz))
        return pts

    def calibrate(self, depth_mm, now=0.0, iterations=300):
        """Tìm sàn bằng RANSAC, chỉ tinh chỉnh quanh tư thế gá đã đo. Trả về True nếu thành công."""
        if self._rays is None:
            return False
        pts = self._points(depth_mm)
        if len(pts) < 500:
            return False
        rng = np.random.RandomState(7)
        sample = pts[rng.choice(len(pts), min(len(pts), 6000), replace=False)]
        best, best_count = None, 0
        for _ in range(iterations):
            a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
            n = np.cross(b - a, c - a)
            norm = np.linalg.norm(n)
            if norm < 1e-9:
                continue
            n /= norm
            h = -float(n.dot(a))
            if h < 0:
                n, h = -n, -h
            if (-n[1] < math.cos(math.radians(35)) or not 0.25 <= h <= 0.8
                    or not self._plane_is_plausible(n, h)):
                continue
            count = int((np.abs(sample.dot(n) + h) < 0.02).sum())
            if count > best_count:
                best, best_count = (n, h), count
        if best is None or best_count < 0.12 * len(sample):
            return False
        n, h = best
        inl = sample[np.abs(sample.dot(n) + h) < 0.02]
        n, h = self._fit_plane(inl, n)
        if not self._plane_is_plausible(n, h):
            return False
        self.set_plane(n, h)
        self.quality = best_count / float(len(sample))
        self.source = 'RANSAC'
        self.calibrated_at = now
        return True

    @staticmethod
    def _fit_plane(points, reference_up):
        centroid = points.mean(axis=0)
        _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
        n = vt[-1]
        if n.dot(reference_up) < 0:
            n = -n
        return n, -float(n.dot(centroid))

    def process(self, depth_mm, now=0.0):
        """Trả về dict các mảng (N,2) khung xe: 'obstacles', 'floor', 'cliffs' (+ 'heights')."""
        empty = np.zeros((0, 2))
        result = {'obstacles': empty, 'floor': empty, 'cliffs': empty}
        if self._rays is None:
            return result
        cfg = self.cfg
        everything = self._points(depth_mm, max_range=8.0)
        if not len(everything):
            return result
        pts = everything[everything[:, 2] <= cfg.astra_max_range]
        base = self.to_base(pts)
        horiz = np.hypot(base[:, 0] - cfg.astra_x, base[:, 1] - cfg.astra_y)
        # Bám mặt sàn: điểm gần mặt sàn cũ, cách camera 0.5..3 m
        cand = (np.abs(base[:, 2]) < 0.05) & (horiz > 0.5) & (horiz < 3.0)
        if cand.sum() >= max(300, 0.08 * len(pts)):
            sel = pts[cand]
            if len(sel) > 3000:
                sel = sel[np.random.RandomState(int(now * 10) % 1000).choice(len(sel), 3000, replace=False)]
            n, h = self._fit_plane(sel, self.up)
            angle = math.degrees(math.acos(max(-1.0, min(1.0, float(n.dot(self.up))))))
            if (angle < 4.0 and abs(h - self.height) < 0.05
                    and self._plane_is_plausible(n, h, tracking=True)):
                up = 0.8 * self.up + 0.2 * n
                self.set_plane(up, 0.8 * self.height + 0.2 * h)
                self.quality = 0.9 * self.quality + 0.1 * (cand.sum() / float(len(pts)))
                if self.source == 'MAC DINH':
                    self.source = 'BAM SAN'
                self.calibrated_at = now
                base = self.to_base(pts)
                horiz = np.hypot(base[:, 0] - cfg.astra_x, base[:, 1] - cfg.astra_y)
        z = base[:, 2]
        noise = 0.012 * np.maximum(horiz - 0.8, 0.0)          # sai số tăng theo khoảng cách
        floor = np.abs(z) < cfg.floor_tolerance + noise
        obst = (z >= cfg.obstacle_min_height + noise) & (z <= cfg.obstacle_max_height)
        xy = base[:, :2]
        # Hố / mép bàn / bậc xuống: tia lẽ ra chạm sàn ở quãng t_floor nhưng đo được xa hơn hẳn
        # -> chỗ tia cắt mặt sàn là nơi "mất sàn". Xét cả điểm xa tới 8 m vì tia qua mép bàn
        # thường chạm nền nhà ở rất xa.
        cliffs = empty
        rng = np.linalg.norm(everything, axis=1)
        rc = everything / rng[:, None]
        down = -rc.dot(self.up)
        t_floor = self.height / np.maximum(down, 1e-6)
        hole = (down > 0.02) & (t_floor < 2.6) & (rng > 1.08 * t_floor + 0.10)
        if np.any(hole):
            cliffs = self.to_base(rc[hole] * t_floor[hole][:, None])[:, :2]
        keep = body_clearance(xy, cfg.hx, cfg.hy) >= cfg.self_filter
        result['obstacles'] = xy[obst & keep]
        result['floor'] = xy[floor & keep]
        if len(cliffs):
            cliffs = cliffs[body_clearance(cliffs, cfg.hx, cfg.hy) >= cfg.self_filter]
        result['cliffs'] = cliffs
        return result


# ==========================================================
# BẢN ĐỒ NHỚ NGẮN HẠN QUANH XE (KHUNG ODOM)
# ==========================================================
def _shift(arr, di, dj):
    out = np.zeros_like(arr)
    n0, n1 = arr.shape
    si0, si1 = max(0, di), min(n0, n0 + di)
    sj0, sj1 = max(0, dj), min(n1, n1 + dj)
    if si1 > si0 and sj1 > sj0:
        di0, dj0 = max(0, -di), max(0, -dj)
        out[di0:di0 + si1 - si0, dj0:dj0 + sj1 - sj0] = arr[si0:si1, sj0:sj1]
    return out


class LocalGrid(object):
    """Lưới 6x6 m, ô 4 cm, lưu THỜI ĐIỂM thấy bận / thấy trống của từng cảm biến.

    - LiDAR chỉ xóa vật cản LiDAR (tia quét ở 13 cm đi qua gầm bàn không được xóa mặt bàn Astra thấy).
    - Astra xóa vật cản Astra khi nhìn thấy lại mặt sàn ở ô đó.
    - Vật Astra thấy được nhớ lâu hơn vì khi xe tới gần nó lọt xuống dưới khung hình."""

    def __init__(self, cfg, size_m=6.0, resolution=0.04):
        self.cfg = cfg
        self.res = resolution
        self.n = int(round(size_m / resolution))
        self.cx = 0.0
        self.cy = 0.0
        shape = (self.n, self.n)
        self.lidar_occ = np.zeros(shape)
        self.lidar_free = np.zeros(shape)
        self.astra_occ = np.zeros(shape)
        self.astra_free = np.zeros(shape)
        self.cliff = np.zeros(shape, dtype=bool)
        # Lùi vào nửa ô để không xóa nhầm vật cản nằm sát mép xe
        step, inset = resolution * 0.5, resolution * 0.5
        gx, gy = np.meshgrid(np.arange(-cfg.hx + inset, cfg.hx - inset + 1e-9, step),
                             np.arange(-cfg.hy + inset, cfg.hy - inset + 1e-9, step))
        self._body = np.column_stack((gx.ravel(), gy.ravel()))

    def clear(self):
        for arr in (self.lidar_occ, self.lidar_free, self.astra_occ, self.astra_free):
            arr.fill(0.0)
        self.cliff.fill(False)

    def recenter(self, x, y):
        if abs(x - self.cx) < 1.0 and abs(y - self.cy) < 1.0:
            return
        dj = int(round((x - self.cx) / self.res))
        di = int(round((y - self.cy) / self.res))
        self.lidar_occ = _shift(self.lidar_occ, di, dj)
        self.lidar_free = _shift(self.lidar_free, di, dj)
        self.astra_occ = _shift(self.astra_occ, di, dj)
        self.astra_free = _shift(self.astra_free, di, dj)
        self.cliff = _shift(self.cliff, di, dj)
        self.cx += dj * self.res
        self.cy += di * self.res

    def cells(self, xy):
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        j = np.floor((xy[:, 0] - self.cx) / self.res + self.n / 2.0).astype(int)
        i = np.floor((xy[:, 1] - self.cy) / self.res + self.n / 2.0).astype(int)
        inside = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        return i[inside], j[inside]

    def centers(self, i, j):
        return np.column_stack((self.cx + (j + 0.5 - self.n / 2.0) * self.res,
                                self.cy + (i + 0.5 - self.n / 2.0) * self.res))

    def update_lidar(self, pose, scan, now):
        if scan is None:
            return
        hit = np.zeros((self.n, self.n), dtype=bool)
        if len(scan.points):
            hi, hj = self.cells(transform_points(pose, scan.points))
            hit[hi, hj] = True
        if scan.fan is not None and len(scan.fan) >= 3 and cv2 is not None:
            # Nhanh: tô đa giác vùng nhìn thấy (~0.2 ms thay vì lấy mẫu từng tia)
            fan = transform_points(pose, scan.fan)
            gx = (fan[:, 0] - self.cx) / self.res + self.n / 2.0 - 0.5
            gy = (fan[:, 1] - self.cy) / self.res + self.n / 2.0 - 0.5
            poly = np.round(np.column_stack((gx, gy)) * 16.0).astype(np.int32)
            mask = np.zeros((self.n, self.n), dtype=np.uint8)
            cv2.fillPoly(mask, [poly], 1, cv2.LINE_8, 4)
            self.lidar_free[(mask > 0) & ~hit] = now
            self.lidar_occ[hit] = now
            return
        if not len(scan.points):
            return
        ends = transform_points(pose, scan.points)
        origin = transform_points(pose, [scan.origin])[0]
        d = ends - origin
        length = np.hypot(d[:, 0], d[:, 1])
        half = self.n * self.res / 2.0
        reach = np.minimum(length - 1.5 * self.res, half)
        if np.any(reach > 0):
            step = self.res * 0.5
            k = np.arange(int(math.ceil(max(reach.max(), 0.0) / step)) + 1) * step
            keep = k[None, :] < reach[:, None]
            frac = k[None, :] / np.maximum(length, 1e-6)[:, None]
            samples = origin + frac[..., None] * d[:, None, :]
            fi, fj = self.cells(samples[keep])
            free = np.zeros((self.n, self.n), dtype=bool)
            free[fi, fj] = True
            free &= ~hit
            self.lidar_free[free] = now
        self.lidar_occ[hit] = now

    def update_astra(self, pose, obstacles, floor, cliffs, now):
        hit = np.zeros((self.n, self.n), dtype=bool)
        for pts, is_cliff, need in ((obstacles, False, 2), (cliffs, True, 2)):
            if pts is None or not len(pts):
                continue
            i, j = self.cells(transform_points(pose, pts))
            if not len(i):
                continue
            flat = i * self.n + j
            counts = np.bincount(flat, minlength=self.n * self.n).reshape(self.n, self.n)
            cell = counts >= need
            self.astra_occ[cell] = now
            self.cliff[cell] = is_cliff
            hit |= cell
        if floor is not None and len(floor):
            i, j = self.cells(transform_points(pose, floor))
            free = np.zeros((self.n, self.n), dtype=bool)
            free[i, j] = True
            self.astra_free[free & ~hit] = now

    def mark_body(self, pose, now):
        """Chỗ xe đang đứng chắc chắn trống (giúp xe lùi lại được theo đường vừa đi)."""
        i, j = self.cells(transform_points(pose, self._body))
        self.lidar_free[i, j] = now
        self.astra_free[i, j] = now
        self.lidar_occ[i, j] = 0.0
        self.astra_occ[i, j] = 0.0

    def _window(self, pose, radius):
        x, y = pose[0], pose[1]
        r = int(math.ceil(radius / self.res)) + 1
        cj = int(math.floor((x - self.cx) / self.res + self.n / 2.0))
        ci = int(math.floor((y - self.cy) / self.res + self.n / 2.0))
        return (max(0, ci - r), min(self.n, ci + r + 1), max(0, cj - r), min(self.n, cj + r + 1))

    def occupied_mask(self, now, window=None):
        cfg = self.cfg
        sl = (slice(None), slice(None)) if window is None else (slice(window[0], window[1]), slice(window[2], window[3]))
        lo, lf = self.lidar_occ[sl], self.lidar_free[sl]
        ao, af = self.astra_occ[sl], self.astra_free[sl]
        lidar = (lo > 0) & (now - lo <= cfg.lidar_memory_s) & (lo >= lf)
        astra = (ao > 0) & (now - ao <= cfg.astra_memory_s) & (ao >= af)
        return lidar, astra

    def free_mask(self, now, window=None):
        cfg = self.cfg
        sl = (slice(None), slice(None)) if window is None else (slice(window[0], window[1]), slice(window[2], window[3]))
        lf, af = self.lidar_free[sl], self.astra_free[sl]
        return (((lf > 0) & (now - lf <= cfg.free_memory_s) & (lf > self.lidar_occ[sl]))
                | ((af > 0) & (now - af <= cfg.free_memory_s) & (af > self.astra_occ[sl])))

    def query(self, pose, now, radius=1.9, lidar_model=None, blind_radius=0.35):
        """Trả về (vật cản, điểm vùng mù chưa biết) trong khung xe, chỉ trong bán kính cần cho lập đường."""
        win = self._window(pose, radius)
        lidar, astra = self.occupied_mask(now, win)
        ii, jj = np.nonzero(lidar | astra)
        obstacles = inverse_transform_points(pose, self.centers(ii + win[0], jj + win[2]))
        if len(obstacles):
            obstacles = obstacles[np.hypot(obstacles[:, 0], obstacles[:, 1]) <= radius]
        blind = np.zeros((0, 2))
        if lidar_model is not None:
            bw = self._window(pose, math.hypot(self.cfg.hx, self.cfg.hy) + blind_radius)
            unknown = ~self.free_mask(now, bw)
            occ_l, occ_a = self.occupied_mask(now, bw)
            ui, uj = np.nonzero(unknown & ~occ_l & ~occ_a)
            if len(ui):
                pts = inverse_transform_points(pose, self.centers(ui + bw[0], uj + bw[2]))
                gap = body_clearance(pts, self.cfg.hx, self.cfg.hy)
                pts = pts[(gap <= blind_radius) & (gap >= 0.005)]
                blind = pts[lidar_model.is_blind(pts)]
        return obstacles, blind


# ==========================================================
# MỤC TIÊU: VỊ TRÍ, XÁC NHẬN, GHI NHỚ KHI BỊ CHE KHUẤT
# ==========================================================
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
    """Cần 2 lần thấy liên tiếp mới coi là tìm thấy; mất dấu thì nhớ vị trí trong khung odom vài giây."""

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
        self.hits = self.hits + 1 if recent else 1
        if self.hits >= 2:
            self.confirmed = True
        if self.bearing is None or not recent:
            self.bearing = bearing
        else:
            self.bearing = wrap_scalar(self.bearing + 0.6 * wrap_scalar(bearing - self.bearing))
        if distance is not None:
            if self.distance is None or self.distance_time is None or now - self.distance_time > 1.0:
                self.distance = distance
            else:
                self.distance += 0.5 * (distance - self.distance)
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
        if age > self.cfg.target_memory_s:
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
                p = astra.to_base([[xn * z, yn * z, z]])[0]
                return math.atan2(p[1], p[0]), math.hypot(p[0], p[1]), 'DO SAU'
    # Không có độ sâu: hướng theo tia camera, khoảng cách theo bề rộng vật
    ray = np.array([xn, yn, 1.0])
    fwd, lft = float(ray.dot(astra.forward)), float(ray.dot(astra.left))
    cam_bearing = math.atan2(lft, fwd)
    width = cfg.target_width.get(label)
    if width is None or bw < 2:
        return cam_bearing, None, 'HUONG'
    dist_cam = max(0.3, min(6.0, fx * width / bw))
    px = cfg.astra_x + dist_cam * math.cos(cam_bearing)
    py = cfg.astra_y + dist_cam * math.sin(cam_bearing)
    return math.atan2(py, px), math.hypot(px, py), 'KICH THUOC'


# ==========================================================
# BỘ LẬP ĐƯỜNG ĐI (VFH CHO MECANUM)
# ==========================================================
class NavCommand(object):
    __slots__ = ('vx', 'vy', 'wz', 'state', 'reason', 'direction', 'clearance', 'v_allow', 'goal', 'side')

    def __init__(self, vx=0.0, vy=0.0, wz=0.0, state='DUNG', reason=''):
        self.vx = vx
        self.vy = vy
        self.wz = wz
        self.state = state
        self.reason = reason
        self.direction = None
        self.clearance = None
        self.v_allow = None
        self.goal = None
        self.side = 0


def _concat(*arrays):
    parts = [np.asarray(a, dtype=float).reshape(-1, 2) for a in arrays if a is not None and len(a)]
    return np.vstack(parts) if parts else np.zeros((0, 2))


class SearchBehavior(object):
    """Chưa thấy mục tiêu: xoay 1 vòng quét, rồi đi tới chỗ thoáng nhất ~0.9 m, rồi xoay tiếp."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.phase = 'XOAY'
        self.turned = 0.0
        self.last_yaw = None
        self.phase_start = None
        self.last_time = None
        self.spin_dir = 1.0
        self.move_index = None
        self.move_from = None
        self.moved = 0.0
        self.last_dirs = []

    def _start_spin(self, now):
        self.phase = 'XOAY'
        self.turned = 0.0
        self.last_yaw = None
        self.phase_start = now

    def step(self, dirs, clear, v_allow, can_turn, now, yaw=None, xy=None):
        cfg = self.cfg
        if self.phase_start is None:
            self.phase_start = now
        dt = 0.0 if self.last_time is None else min(0.5, now - self.last_time)
        self.last_time = now
        if self.phase == 'XOAY':
            turn = 25.0 * self.spin_dir
            if yaw is not None:
                if self.last_yaw is not None:
                    self.turned += abs(wrap_scalar(yaw - self.last_yaw))
                self.last_yaw = yaw
            done = self.turned >= math.radians(330) or now - self.phase_start > 25.0
            if not done:
                for candidate in (turn, -turn):
                    if can_turn(candidate):
                        if candidate != turn:
                            self.spin_dir = -self.spin_dir
                        if yaw is None:
                            self.turned += math.radians(abs(candidate)) * dt
                        return 0.0, 0.0, candidate, 'TIM_XOAY', 'QUAY QUET TIM MUC TIEU ({:.0f} DO)'.format(
                            math.degrees(self.turned))
                if now - self.phase_start < 3.0:
                    return 0.0, 0.0, 0.0, 'TIM_CHO', 'KHONG XOAY DUOC, CHO LOI DI'
            # Chọn chỗ thoáng nhất để đi tiếp (ưu tiên phía trước, tránh lặp lại hướng cũ)
            score = np.minimum(clear, 2.0) + 0.4 * np.cos(dirs) - 1.5 * (np.cos(dirs) < -0.3)
            score = np.where((v_allow >= 0.06) & (clear >= 0.5), score, -np.inf)
            if not np.isfinite(score).any():
                self._start_spin(now)
                self.spin_dir = -self.spin_dir
                return 0.0, 0.0, 0.0, 'TIM_CHO', 'XUNG QUANH CHAT, CHO LOI DI'
            self.phase = 'DI'
            self.move_index = int(np.argmax(score))
            self.move_from = xy
            self.moved = 0.0
            self.phase_start = now
        idx = self.move_index
        if xy is not None and self.move_from is not None:
            self.moved = math.hypot(xy[0] - self.move_from[0], xy[1] - self.move_from[1])
        else:
            self.moved += 0.12 * dt
        if (self.moved >= 0.9 or now - self.phase_start > 8.0 or clear[idx] < 0.35
                or v_allow[idx] < cfg.min_speed):
            self._start_spin(now)
            return 0.0, 0.0, 0.0, 'TIM_XOAY', 'TOI CHO MOI, QUET LAI'
        speed = min(float(v_allow[idx]), 0.14)
        return (speed * math.cos(dirs[idx]), speed * math.sin(dirs[idx]), 0.0,
                'TIM_DI', 'DI TOI CHO THOANG DE TIM ({:.1f} m)'.format(self.moved))


class MotionPlanner(object):
    """Chọn (vx, vy, wz) mỗi chu kỳ.

    - Tịnh tiến: 72 hướng (5°). Hướng "mở" khi thân xe đi được >= lookahead mà không chạm vật;
      chọn hướng mở có chi phí nhỏ nhất (lệch mục tiêu, đổi hướng đột ngột, đi ngang, đi lùi).
    - Quay: luôn giữ mục tiêu ở giữa ảnh (Mecanum đi chéo được nên đầu xe không cần quay đi khi né).
    - Chốt bên né: đã lách trái thì tiếp tục trái cho tới khi đường thẳng tới mục tiêu thông,
      chỉ đổi bên khi bên đang lách bị chặn hẳn -> hết lắc qua lắc lại.
    - Vùng mù LiDAR (phía sau) chỉ chặn tịnh tiến, không chặn xoay tại chỗ."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.dirs = np.radians(np.arange(-180.0, 180.0, 5.0))
        self.cap = direction_speed_cap(self.dirs, cfg)
        self.search = SearchBehavior(cfg)
        self.reset()

    def reset(self):
        self.prev_dir = None
        self.side = 0
        self.side_bearing = None
        self.goal_open_since = None
        self.last_v = np.zeros(2)
        self.last_wz = 0.0
        self.last_time = None
        self.arrived = False
        self.peek = None
        self.peek_cooldown = 0.0
        self.search.reset()

    # ---------------- tiện ích ----------------
    def clearance(self, points):
        """Quãng tịnh tiến an toàn theo từng hướng, đã giữ khoảng đệm 'margin'.
        Điểm đã lọt vào vùng đệm: chỉ cấm các hướng tiến lại gần nó (để xe tự thoát ra)."""
        cfg = self.cfg
        hx, hy, m = cfg.hx, cfg.hy, cfg.margin
        if points is None or len(points) == 0:
            return np.full(len(self.dirs), np.inf)
        near = (np.abs(points[:, 0]) <= hx + m) & (np.abs(points[:, 1]) <= hy + m)
        dist = box_sweep_distance(points[~near], self.dirs, hx + m, hy + m)
        if np.any(near):
            p = points[near]
            nx = p[:, 0] - np.clip(p[:, 0], -hx, hx)
            ny = p[:, 1] - np.clip(p[:, 1], -hy, hy)
            nn = np.hypot(nx, ny)
            ok = nn > 1e-6
            if np.any(ok):
                nx, ny = nx[ok] / nn[ok], ny[ok] / nn[ok]
                approach = np.cos(self.dirs)[:, None] * nx[None, :] + np.sin(self.dirs)[:, None] * ny[None, :]
                contact = box_sweep_distance(p[ok], self.dirs, hx, hy)
                dist = np.minimum(dist, np.where((approach > 0.2).any(axis=1), 0.0, contact))
        return dist

    def evaluate(self, obstacles, blind=None):
        """(quãng trống, tốc độ cho phép) theo 72 hướng mà không lập đường (cho radar khi lái tay)."""
        pts = _concat(thin_points(_concat(obstacles)), blind)
        if len(pts):
            pts = pts[np.hypot(pts[:, 0], pts[:, 1]) <= self.cfg.plan_radius]
        clear = self.clearance(pts)
        v_allow = np.minimum(self.cap, speed_for_clearance(clear - 0.02, self.cfg.brake_decel,
                                                           self.cfg.reaction_time))
        return clear, v_allow

    def _turn(self, bearing, visible):
        cfg = self.cfg
        deg = math.degrees(bearing)
        if abs(deg) <= (3.0 if visible else 8.0):
            return 0.0
        wz = max(-cfg.max_turn_deg, min(cfg.max_turn_deg, 1.1 * deg))
        if not visible:
            wz = max(-20.0, min(20.0, wz))
        if abs(wz) < cfg.min_turn_deg:
            wz = math.copysign(cfg.min_turn_deg, wz)
        return wz

    def safe(self, obstacles, blind, vx, vy, wz):
        ok, why = motion_is_safe(obstacles, vx, vy, wz, self.cfg)
        if ok and abs(vx) + abs(vy) > 1e-6 and blind is not None and len(blind):
            ok, why = motion_is_safe(blind, vx, vy, 0.0, self.cfg)
            if not ok:
                why = 'VUNG MU LIDAR: ' + why
        return ok, why

    # ---------------- lập kế hoạch ----------------
    def plan(self, obstacles, blind, target, now, pose=None):
        cfg = self.cfg
        obstacles = thin_points(_concat(obstacles))
        if len(obstacles):
            obstacles = obstacles[np.hypot(obstacles[:, 0], obstacles[:, 1]) <= cfg.plan_radius]
        blind = _concat(blind)
        moving_pts = _concat(obstacles, blind)
        clear = self.clearance(moving_pts)
        v_allow = np.minimum(self.cap, speed_for_clearance(clear - 0.02, cfg.brake_decel, cfg.reaction_time))
        dt = 0.1 if self.last_time is None else max(0.02, min(0.5, now - self.last_time))
        self.last_time = now
        yaw = pose[2] if pose is not None else None
        xy = (pose[0], pose[1]) if pose is not None else None

        def can_turn(wz):
            return motion_is_safe(obstacles, 0.0, 0.0, wz, cfg)[0]

        if target is None or not target.valid:
            self.prev_dir = None
            self.side = 0
            self.arrived = False
            vx, vy, wz, state, reason = self.search.step(self.dirs, clear, v_allow, can_turn, now, yaw, xy)
            goal = None
        else:
            self.search.reset()
            vx, vy, wz, state, reason = self._track(obstacles, blind, clear, v_allow, target, now)
            goal = target.bearing
        cmd = self._finalize(obstacles, blind, vx, vy, wz, state, reason, dt)
        cmd.clearance = clear
        cmd.v_allow = v_allow
        cmd.goal = goal
        cmd.side = self.side
        return cmd

    def _blind_limited_direction(self, obstacles, bearing):
        """Hướng đi được nếu chỉ tính vật cản THẬT; dùng khi xe bị chặn bởi vùng LiDAR chưa thấy."""
        cfg = self.cfg
        v_real = np.minimum(self.cap, speed_for_clearance(self.clearance(obstacles) - 0.02,
                                                          cfg.brake_decel, cfg.reaction_time))
        dev = np.abs(wrap(self.dirs - bearing))
        ok = (v_real >= cfg.min_speed) & (dev <= math.radians(115))
        if not ok.any():
            return None
        return float(self.dirs[int(np.argmin(np.where(ok, dev, np.inf)))])

    def _track(self, obstacles, blind, clear, v_allow, target, now):
        cfg = self.cfg
        bearing = target.bearing
        dist = target.distance
        # Đang xoay nhìn quanh (cho LiDAR quét hông sau xe) -> làm nốt rồi mới lập đường lại
        if self.peek is not None:
            if now < self.peek[1]:
                return 0.0, 0.0, self.peek[0] * 30.0, 'NHIN_QUANH', 'XOAY NHIN VUNG LIDAR CHUA THAY ({})'.format(
                    'TRAI' if self.peek[0] > 0 else 'PHAI')
            self.peek = None
            self.peek_cooldown = now + 6.0
        wz = self._turn(bearing, target.visible)
        standoff = cfg.hx + target.standoff
        if dist is not None:
            if dist <= standoff + (0.06 if self.arrived else 0.0):
                self.arrived = True
                self.prev_dir = None
                self.side = 0
                centre = wz if abs(math.degrees(bearing)) > 6.0 else 0.0
                return 0.0, 0.0, centre, 'DA_TOI', 'DA DEN GAN MUC TIEU ({:.0f} CM)'.format(
                    100 * max(0.0, dist - cfg.hx))
        self.arrived = False
        if dist is not None:
            v_goal = float(np.clip(0.55 * (dist - standoff), cfg.min_speed, cfg.max_forward))
        else:
            v_goal = cfg.max_forward * 0.8
        if not target.visible:
            v_goal = min(v_goal, 0.10)
        elif abs(bearing) > math.radians(35):
            v_goal = min(v_goal, 0.06)          # mục tiêu lệch xa: quay là chính, đi chậm
        # Lập đường bỏ qua chính mục tiêu (người/ghế cũng là vật cản trong LiDAR)
        # Chọn hướng chiến lược chỉ theo vật cản THẬT; vùng mù (chưa biết) chỉ giới hạn tốc độ
        # tức thời qua v_allow, không được làm xe bỏ một lối đi tốt.
        real = obstacles
        if dist is not None and len(obstacles):
            tx, ty = dist * math.cos(bearing), dist * math.sin(bearing)
            real = obstacles[np.hypot(obstacles[:, 0] - tx, obstacles[:, 1] - ty) > target.radius]
        nav_clear = self.clearance(real)
        need = cfg.lookahead if dist is None else min(cfg.lookahead, max(0.15, dist - standoff))
        signed = wrap(self.dirs - bearing)
        dev = np.abs(signed)
        feasible = (v_allow >= cfg.min_speed) & (dev <= math.radians(115))
        open_ = feasible & (nav_clear >= need)
        goal_open = bool(np.any(open_ & (dev <= math.radians(7.5))))
        # Nhả chốt bên né khi đường thẳng tới mục tiêu đã thông ổn định, hoặc mục tiêu đổi chỗ hẳn
        if goal_open:
            if self.goal_open_since is None:
                self.goal_open_since = now
            if now - self.goal_open_since >= 0.5:
                self.side = 0
        else:
            self.goal_open_since = None
        if self.side and self.side_bearing is not None and abs(wrap_scalar(bearing - self.side_bearing)) > math.radians(60):
            self.side = 0
        candidates = open_ if open_.any() else feasible
        if not candidates.any():
            self.prev_dir = None
            # Bị chặn chỉ vì hông sau xe chưa từng được LiDAR nhìn thấy (vừa khởi động / đứng yên lâu):
            # xoay ~50° về phía hướng định đi để quét vùng đó, thay vì đứng chết một chỗ.
            if blind is not None and len(blind) and now >= self.peek_cooldown:
                wanted = self._blind_limited_direction(obstacles, bearing)
                if wanted is not None:
                    turn = 1 if math.sin(wanted) >= 0 else -1
                    if motion_is_safe(obstacles, 0.0, 0.0, turn * 30.0, cfg)[0]:
                        self.peek = (turn, now + 1.8)
                        return 0.0, 0.0, turn * 30.0, 'NHIN_QUANH', 'XOAY NHIN VUNG LIDAR CHUA THAY ({})'.format(
                            'TRAI' if turn > 0 else 'PHAI')
            return 0.0, 0.0, wz, 'BI_CHAN', 'KHONG CO LOI DI AN TOAN TOI MUC TIEU'
        cost = dev / math.pi
        if self.prev_dir is not None:
            cost = cost + 0.3 * np.abs(wrap(self.dirs - self.prev_dir)) / math.pi
        cost = cost + 0.15 * np.abs(np.sin(self.dirs))                  # đi ngang kém chính xác
        cost = cost + 0.8 * np.maximum(0.0, -np.cos(self.dirs))         # lùi = đi vào vùng mù
        cost = cost + 0.3 * (1.0 - np.minimum(nav_clear, 1.2) / 1.2)    # ưu tiên chỗ rộng
        if self.side:
            wrong = (np.sign(signed) == -self.side) & (dev > math.radians(12))
            cost = cost + np.where(wrong, 5.0, 0.0)
        idx = int(np.argmin(np.where(candidates, cost, np.inf)))
        chosen = float(self.dirs[idx])
        off = float(signed[idx])
        if abs(off) > math.radians(12) and not goal_open:
            side = 1 if off > 0 else -1
            if self.side != side:
                # Bắt đầu né, hoặc bên đang né đã bị chặn hẳn -> chuyển hẳn sang bên kia
                self.side = side
                self.side_bearing = bearing
        speed = min(float(v_allow[idx]), v_goal)
        if not open_.any():
            speed = min(speed, 0.08)            # chỗ chật: chỉ bò chậm
        self.prev_dir = chosen
        if abs(off) <= math.radians(12):
            state, reason = 'BAM', 'BAM THEO MUC TIEU'
        else:
            state = 'NE_TRAI' if off > 0 else 'NE_PHAI'
            reason = 'LACH {} VAT CAN ({:.0f} DO)'.format('TRAI' if off > 0 else 'PHAI', abs(math.degrees(off)))
        if dist is not None:
            reason += ' | CACH {:.2f} M'.format(max(0.0, dist - cfg.hx))
        if not target.visible:
            state = 'NHO_' + state
            reason = 'MAT DAU, DI THEO VI TRI NHO | ' + reason
        return speed * math.cos(chosen), speed * math.sin(chosen), wz, state, reason

    def _finalize(self, obstacles, blind, vx, vy, wz, state, reason, dt):
        """Giới hạn gia tốc (phanh thì tức thì) + kiểm tra quét thân xe; lỗi thì hạ dần lệnh."""
        cfg = self.cfg
        target_v = np.array([vx, vy])
        v = target_v
        if np.linalg.norm(target_v) > np.linalg.norm(self.last_v) + 1e-9:
            step = target_v - self.last_v
            limit = cfg.accel * dt
            if np.linalg.norm(step) > limit:
                v = self.last_v + step * (limit / np.linalg.norm(step))
        w_lim = cfg.turn_accel_deg * dt
        if abs(wz) < 1e-6:
            w = 0.0                             # dừng quay thì dừng ngay
        else:
            w = self.last_wz + max(-w_lim, min(w_lim, wz - self.last_wz))
            if abs(w) < cfg.min_turn_deg or w * wz < 0:
                w = math.copysign(cfg.min_turn_deg, wz)   # dưới ngưỡng này động cơ không nhúc nhích
        options = [(v[0], v[1], w), (vx, vy, wz), (v[0], v[1], 0.0), (0.5 * v[0], 0.5 * v[1], 0.0), (0.0, 0.0, w)]
        why = ''
        for i, (cx, cy, cw) in enumerate(options):
            ok, reason_bad = self.safe(obstacles, blind, cx, cy, cw)
            if ok:
                if i >= 2:
                    reason = reason + ' | GIAM LENH: ' + why
                cmd = NavCommand(cx, cy, cw, state, reason)
                break
            why = why or reason_bad
        else:
            cmd = NavCommand(0.0, 0.0, 0.0, 'DUNG_AN_TOAN', 'DUNG: ' + why)
        if abs(cmd.vx) + abs(cmd.vy) > 1e-6:
            cmd.direction = math.atan2(cmd.vy, cmd.vx)
        self.last_v = np.array([cmd.vx, cmd.vy])
        self.last_wz = cmd.wz
        return cmd


# ==========================================================
# VẼ RADAR CHO WEB (TỪ TRÊN NHÌN XUỐNG, ĐẦU XE HƯỚNG LÊN)
# ==========================================================
def render_radar(cfg, grid, pose, now, cmd=None, target=None, lidar_model=None, lines=(), size=480, meters=2.4,
                 dry_run=False):
    unknown = (24, 18, 15)
    # Ảnh màu của lưới (khung odom), sau đó xoay/dịch về khung xe bằng một phép affine
    lidar, astra = grid.occupied_mask(now)
    cells = np.empty((grid.n, grid.n, 3), dtype=np.uint8)
    cells[:] = unknown
    cells[grid.free_mask(now)] = (58, 44, 32)
    cells[astra] = (0, 140, 255)
    cells[astra & grid.cliff] = (255, 0, 255)
    cells[lidar] = (255, 220, 0)
    s = size / (2.0 * meters)
    c0 = size // 2
    x, y, yaw = pose
    c, sn = math.cos(yaw), math.sin(yaw)
    ex = grid.cx - x + grid.res * (0.5 - grid.n / 2.0)
    ey = grid.cy - y + grid.res * (0.5 - grid.n / 2.0)
    a = grid.res * s
    warp = np.array([[sn * a, -c * a, c0 + s * (sn * ex - c * ey)],
                     [-c * a, -sn * a, c0 - s * (c * ex + sn * ey)]], dtype=np.float64)
    img = cv2.warpAffine(cells, warp, (size, size), flags=cv2.INTER_NEAREST,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=unknown)

    def to_px(x, y):
        return int(round(c0 - y * s)), int(round(c0 - x * s))

    for ring in (0.5, 1.0, 1.5, 2.0):
        cv2.circle(img, (c0, c0), int(ring * s), (60, 60, 60), 1)
    # Góc mù LiDAR
    if lidar_model is not None and lidar_model.valid_ema is not None:
        blind = lidar_model.blind_bins()
        width = TWO_PI / lidar_model.bins
        ox, oy = to_px(cfg.lidar_x, cfg.lidar_y)
        for b in np.flatnonzero(blind):
            a = -math.pi + (b + 0.5) * width + cfg.lidar_yaw
            ex, ey = to_px(cfg.lidar_x + 0.45 * math.cos(a), cfg.lidar_y + 0.45 * math.sin(a))
            cv2.line(img, (ox, oy), (ex, ey), (40, 40, 110), 2)
    # Các hướng: dài theo quãng trống, màu theo tốc độ cho phép
    if cmd is not None and cmd.clearance is not None and cmd.v_allow is not None:
        planner_dirs = np.radians(np.arange(-180.0, 180.0, 5.0))
        cap = direction_speed_cap(planner_dirs, cfg)
        for a, d, v, cp in zip(planner_dirs, cmd.clearance, cmd.v_allow, cap):
            length = min(float(d), 1.2) + cfg.hx
            colour = (0, 200, 0) if v >= 0.8 * cp else ((0, 200, 255) if v >= cfg.min_speed else (0, 0, 200))
            cv2.line(img, (c0, c0), to_px(length * math.cos(a), length * math.sin(a)), colour, 1)
    # Thân xe + vùng đệm
    corners = [(cfg.hx, cfg.hy), (cfg.hx, -cfg.hy), (-cfg.hx, -cfg.hy), (-cfg.hx, cfg.hy)]
    cv2.polylines(img, [np.array([to_px(x, y) for x, y in corners], np.int32)], True, (0, 255, 0), 2)
    m = cfg.margin
    marg = [(cfg.hx + m, cfg.hy + m), (cfg.hx + m, -cfg.hy - m), (-cfg.hx - m, -cfg.hy - m), (-cfg.hx - m, cfg.hy + m)]
    cv2.polylines(img, [np.array([to_px(x, y) for x, y in marg], np.int32)], True, (0, 110, 0), 1)
    cv2.arrowedLine(img, (c0, c0), to_px(cfg.hx + 0.08, 0.0), (0, 255, 0), 2, tipLength=0.4)
    # Mục tiêu
    if target is not None and target.valid:
        colour = (0, 255, 0) if target.visible else (0, 165, 255)
        dist = target.distance if target.distance is not None else 1.6
        tp = to_px(min(dist, meters * 0.95) * math.cos(target.bearing), min(dist, meters * 0.95) * math.sin(target.bearing))
        cv2.line(img, (c0, c0), tp, colour, 1, cv2.LINE_AA)
        cv2.circle(img, tp, 10, colour, 2)
        cv2.putText(img, target.label.upper(), (tp[0] + 12, tp[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)
    # Lệnh đang chạy
    if cmd is not None and (abs(cmd.vx) + abs(cmd.vy)) > 1e-6:
        k = 0.45 / max(cfg.max_forward, 1e-6) + 2.0
        cv2.arrowedLine(img, (c0, c0), to_px(cmd.vx * k, cmd.vy * k), (0, 255, 255), 3, tipLength=0.3)
    if cmd is not None and abs(cmd.wz) > 1e-6:
        r = int(0.30 * s)
        start = -90 - (40 if cmd.wz > 0 else -40)
        cv2.ellipse(img, (c0, c0), (r, r), 0, -90, start, (255, 255, 0), 2)
    cv2.rectangle(img, (0, 0), (size, 40), (15, 23, 42), -1)
    title = 'RADAR NE VAT CAN | LiDAR + Astra'
    if dry_run:
        title += ' | CHAY KHO'
    cv2.putText(img, title, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 229, 255) if not dry_run else (0, 165, 255), 2)
    y = size - 12 - 18 * (len(lines) - 1)
    for line in lines:
        cv2.putText(img, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1)
        y += 18
    return img
