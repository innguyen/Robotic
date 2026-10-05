#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Kiểm thử obstacle_avoidance.py (không cần ROS, chạy được trên máy tính và trên robot):
    python -m unittest test_obstacle_avoidance -v

Gồm test từng khối + mô phỏng vòng kín: xe Mecanum ảo, LiDAR ảo (mù phía sau giống robot
thật), Astra ảo (chỉ thấy vật từ ~0.75 m), mục tiêu bị che khuất... Mỗi kịch bản kiểm tra:
không va chạm, tới được mục tiêu, không lắc trái-phải.
"""
from __future__ import division, print_function
import math
import unittest

import numpy as np

import obstacle_avoidance as oa


CFG = oa.AvoidanceConfig()


# ==========================================================
# MÔ PHỎNG THẾ GIỚI 2D
# ==========================================================
class World(object):
    """Vật cản: hình tròn (cx, cy, r, cao) và hình chữ nhật (x0, y0, x1, y1, cao)."""

    def __init__(self, circles=(), boxes=()):
        self.circles = [tuple(c) for c in circles]
        self.boxes = [tuple(b) for b in boxes]

    def ray_cast(self, origin, angles, max_range, height):
        """Khoảng cách tới vật cản cao hơn 'height' theo từng tia (inf nếu không chạm)."""
        ox, oy = origin
        dx, dy = np.cos(angles), np.sin(angles)
        best = np.full(len(angles), np.inf)
        for cx, cy, r, h in self.circles:
            if h < height:
                continue
            fx, fy = ox - cx, oy - cy
            b = dx * fx + dy * fy
            c = fx * fx + fy * fy - r * r
            disc = b * b - c
            ok = disc >= 0
            t = -b - np.sqrt(np.where(ok, disc, 0.0))
            t = np.where(ok & (t > 0), t, np.inf)
            best = np.minimum(best, t)
        for x0, y0, x1, y1, h in self.boxes:
            if h < height:
                continue
            with np.errstate(divide='ignore', invalid='ignore'):
                tx1, tx2 = (x0 - ox) / dx, (x1 - ox) / dx
                ty1, ty2 = (y0 - oy) / dy, (y1 - oy) / dy
            tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
            tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
            t = np.where((tmax >= tmin) & (tmax > 0), np.maximum(tmin, 0.0), np.inf)
            best = np.minimum(best, t)
        return np.where(best <= max_range, best, np.inf)

    def collides(self, pose):
        """Thân xe chữ nhật có chạm vật cản nào không."""
        x, y, yaw = pose
        corners = oa.transform_points(pose, [(CFG.hx, CFG.hy), (CFG.hx, -CFG.hy),
                                             (-CFG.hx, -CFG.hy), (-CFG.hx, CFG.hy)])
        for cx, cy, r, _ in self.circles:
            local = oa.inverse_transform_points(pose, [(cx, cy)])
            if oa.body_clearance(local, CFG.hx, CFG.hy)[0] < r - 1e-3:
                return True
        for x0, y0, x1, y1, _ in self.boxes:
            if (np.any((corners[:, 0] > x0) & (corners[:, 0] < x1) & (corners[:, 1] > y0) & (corners[:, 1] < y1))):
                return True
            # góc hộp nằm trong thân xe
            local = oa.inverse_transform_points(pose, [(x0, y0), (x0, y1), (x1, y0), (x1, y1)])
            if np.any((np.abs(local[:, 0]) < CFG.hx) & (np.abs(local[:, 1]) < CFG.hy)):
                return True
        return False

    def min_clearance(self, pose):
        best = np.inf
        for cx, cy, r, _ in self.circles:
            local = oa.inverse_transform_points(pose, [(cx, cy)])
            best = min(best, oa.body_clearance(local, CFG.hx, CFG.hy)[0] - r)
        for x0, y0, x1, y1, _ in self.boxes:
            xs = np.linspace(x0, x1, 40)
            ys = np.linspace(y0, y1, 40)
            edge = np.vstack([np.column_stack((xs, np.full(40, y0))), np.column_stack((xs, np.full(40, y1))),
                              np.column_stack((np.full(40, x0), ys)), np.column_stack((np.full(40, x1), ys))])
            best = min(best, oa.body_clearance(oa.inverse_transform_points(pose, edge), CFG.hx, CFG.hy).min())
        return best


LIDAR_ANGLES = np.radians(np.arange(-180.0, 180.0, 0.477))


def lidar_blind(rel):
    """Góc mù đo thực tế: từ +120° qua 180° tới -90° (phía sau và sau-phải)."""
    deg = np.degrees(oa.wrap(rel))
    return (deg >= 120.0) | (deg <= -90.0)


def simulate_scan(world, pose):
    lx, ly = oa.transform_points(pose, [(CFG.lidar_x, CFG.lidar_y)])[0]
    ranges = world.ray_cast((lx, ly), LIDAR_ANGLES + pose[2], 16.0, 0.13)
    ranges = np.where(np.isfinite(ranges), ranges, 0.0)       # driver trả 0 khi không có tia về
    ranges = np.where(lidar_blind(LIDAR_ANGLES), 0.0, ranges)  # bị khung xe che
    ranges = np.where(ranges < 0.10, 0.0, ranges)
    return ranges


def simulate_astra(world, pose, rng=None):
    """Kết quả AstraDepth.process giả lập: thấy vật cao 4.5 cm trở lên trong FOV ±29°,
    cách camera 0.75..3.0 m (giống camera đã chúi ~8°), kèm điểm sàn trống."""
    cam = oa.transform_points(pose, [(CFG.astra_x, CFG.astra_y)])[0]
    rays = np.radians(np.linspace(-29.0, 29.0, 59)) + pose[2]
    hit = world.ray_cast(cam, rays, 3.0, 0.045)
    obstacles, floor = [], []
    for a, t in zip(rays, hit):
        if np.isfinite(t) and t >= 0.75:
            obstacles.append((cam[0] + t * math.cos(a), cam[1] + t * math.sin(a)))
        for dist in np.arange(0.95, min(3.0, t if np.isfinite(t) else 3.0) - 0.05, 0.08):
            floor.append((cam[0] + dist * math.cos(a), cam[1] + dist * math.sin(a)))
    obs = oa.inverse_transform_points(pose, obstacles) if obstacles else np.zeros((0, 2))
    flo = oa.inverse_transform_points(pose, floor) if floor else np.zeros((0, 2))
    # Mỗi điểm vật cản lặp 2 lần (Astra thấy nhiều pixel trên một ô)
    return {'obstacles': np.vstack([obs, obs]) if len(obs) else obs, 'floor': flo, 'cliffs': np.zeros((0, 2))}


def target_observation(world, pose, target_xy, label='person', fov_deg=30.0):
    """Camera Astra (sau tâm xe) nhìn thấy mục tiêu nếu trong FOV, không bị che, cách 0.4..5 m."""
    cam = oa.transform_points(pose, [(CFG.astra_x, CFG.astra_y)])[0]
    dx, dy = target_xy[0] - cam[0], target_xy[1] - cam[1]
    dist_cam = math.hypot(dx, dy)
    ang = oa.wrap_scalar(math.atan2(dy, dx) - pose[2])
    if abs(math.degrees(ang)) > fov_deg or not 0.4 <= dist_cam <= 5.0:
        return None
    # Chỉ vật cao >= 1 m mới che được người (camera ở 0.46 m nhìn thấy thân trên qua hộp thấp)
    blocked = World([c for c in world.circles if math.hypot(c[0] - target_xy[0], c[1] - target_xy[1]) > 0.05],
                    world.boxes).ray_cast(cam, np.array([math.atan2(dy, dx)]), dist_cam - 0.25, 1.0)[0]
    if np.isfinite(blocked):
        return None
    local = oa.inverse_transform_points(pose, [target_xy])[0]
    # Đo độ sâu tới bề mặt trước của người (bán kính 0.2 m)
    d = math.hypot(local[0], local[1]) - (0.2 if label == 'person' else 0.0)
    return math.atan2(local[1], local[0]), d


ROOM = [(-3.3, -2.5, 4.7, -2.4, 1.0), (-3.3, 2.4, 4.7, 2.5, 1.0),     # tường phòng 8 x 4.8 m
        (-3.4, -2.5, -3.3, 2.5, 1.0), (4.7, -2.5, 4.8, 2.5, 1.0)]


def run_scenario(world, start, target_xy, label='person', seconds=60.0, dt=0.1, slip=0.0, seed=0,
                 target_is_obstacle=True, room=True, lateral_gain=1.0, odom_from_commands=False):
    """lateral_gain < 1: bánh Mecanum trượt ngang có hệ thống. odom_from_commands: odom tích phân
    vận tốc LỆNH giống jetauto_controller thật (không biết xe bị trượt) -> bản đồ nhớ bị lệch dần."""
    rng = np.random.RandomState(seed)
    if target_is_obstacle and label == 'person':
        world = World(list(world.circles) + [(target_xy[0], target_xy[1], 0.2, 1.7)], world.boxes)
    if room:
        world = World(world.circles, list(world.boxes) + ROOM)
    cfg = CFG
    lidar = oa.LidarModel(cfg)
    grid = oa.LocalGrid(cfg)
    planner = oa.MotionPlanner(cfg)
    tracker = oa.TargetTracker(cfg)
    pose = np.array(start, dtype=float)
    odom = pose.copy()
    log = {'collided': False, 'reached': False, 'min_clear': np.inf, 'sides': [], 'states': [], 'time': 0.0}
    now = 100.0
    for step in range(int(seconds / dt)):
        now += dt
        est_pose = tuple(odom) if odom_from_commands else tuple(pose)
        ranges = simulate_scan(world, pose)           # cảm biến luôn đo từ vị trí THẬT
        scan = lidar.process(ranges, LIDAR_ANGLES[0], LIDAR_ANGLES[1] - LIDAR_ANGLES[0], 0.1, 16.0, now)
        grid.recenter(est_pose[0], est_pose[1])
        grid.mark_body(est_pose, now)
        grid.update_lidar(est_pose, scan, now)
        astra = simulate_astra(world, pose)
        grid.update_astra(est_pose, astra['obstacles'], astra['floor'], astra['cliffs'], now)
        obstacles, blind = grid.query(est_pose, now, lidar_model=lidar)
        obstacles = oa._concat(obstacles, scan.points)
        obs = target_observation(world, pose, target_xy, label)
        if obs is not None:
            tracker.observe(label, obs[0], obs[1], now, est_pose, 'SIM')
        else:
            tracker.missed(now)
        est = tracker.estimate(now, est_pose)
        cmd = planner.plan(obstacles, blind, est, now, est_pose)
        log['sides'].append(planner.side)
        log['states'].append(cmd.state)
        # Động học Mecanum (có thể thêm trượt bánh)
        vx = cmd.vx * (1.0 + slip * rng.randn())
        vy = cmd.vy * lateral_gain * (1.0 + slip * rng.randn())
        wz = math.radians(cmd.wz)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        pose = pose + np.array([(c * vx - s * vy) * dt, (s * vx + c * vy) * dt, wz * dt])
        pose[2] = oa.wrap_scalar(pose[2])
        c, s = math.cos(odom[2]), math.sin(odom[2])
        odom = odom + np.array([(c * cmd.vx - s * cmd.vy) * dt, (s * cmd.vx + c * cmd.vy) * dt, wz * dt])
        odom[2] = oa.wrap_scalar(odom[2])
        if world.collides(pose):
            log['collided'] = True
            break
        log['min_clear'] = min(log['min_clear'], world.min_clearance(pose))
        if cmd.state == 'DA_TOI':
            local = oa.inverse_transform_points(pose, [target_xy])[0]
            log['reached'] = True
            log['final_gap'] = math.hypot(local[0], local[1]) - CFG.hx - (0.2 if label == 'person' else 0.0)
            log['time'] = (step + 1) * dt
            break
    log['pose'] = pose
    return log


def side_flips(sides):
    nonzero = [s for s in sides if s]
    return sum(1 for a, b in zip(nonzero, nonzero[1:]) if a != b)


# ==========================================================
# TEST TỪNG KHỐI
# ==========================================================
class GeometryTest(unittest.TestCase):
    def test_sweep_distance_ahead(self):
        pts = np.array([[1.0, 0.0]])
        dirs = np.radians([0.0, 90.0, 180.0])
        d = oa.box_sweep_distance(pts, dirs, CFG.hx, CFG.hy)
        self.assertAlmostEqual(d[0], 1.0 - CFG.hx, places=6)
        self.assertTrue(np.isinf(d[1]) and np.isinf(d[2]))

    def test_sweep_distance_inside_is_zero(self):
        d = oa.box_sweep_distance(np.array([[0.1, 0.0]]), np.radians([0.0, 180.0]), CFG.hx, CFG.hy)
        self.assertEqual(d[0], 0.0)
        self.assertEqual(d[1], 0.0)

    def test_speed_drops_near_obstacle(self):
        v = oa.speed_for_clearance(np.array([0.0, 0.05, 0.3, np.inf]), CFG.brake_decel, CFG.reaction_time)
        self.assertEqual(v[0], 0.0)
        self.assertTrue(v[0] < v[1] < v[2] < v[3])

    def test_turn_blocked_by_wall_beside_corner(self):
        # Tường song song cách hông xe 3 cm: xoay sẽ quẹt góc xe
        wall = np.column_stack((np.linspace(-0.4, 0.4, 41), np.full(41, CFG.hy + 0.03)))
        self.assertFalse(oa.motion_is_safe(wall, 0.0, 0.0, 25.0, CFG)[0])
        # Nhưng đi thẳng song song tường thì được
        self.assertTrue(oa.motion_is_safe(wall, 0.15, 0.0, 0.0, CFG)[0])

    def test_escape_only_away(self):
        # Vật lọt vào vùng đệm phía trước: không được tiến, được lùi / đi ngang
        p = np.array([[CFG.hx + 0.015, 0.0]])
        self.assertFalse(oa.motion_is_safe(p, 0.1, 0.0, 0.0, CFG)[0])
        self.assertTrue(oa.motion_is_safe(p, -0.05, 0.0, 0.0, CFG)[0])
        self.assertTrue(oa.motion_is_safe(p, 0.0, 0.1, 0.0, CFG)[0])
        planner = oa.MotionPlanner(CFG)
        clear = planner.clearance(p)
        fwd = int(np.argmin(np.abs(planner.dirs)))
        back = int(np.argmin(np.abs(oa.wrap(planner.dirs - math.pi))))
        self.assertEqual(clear[fwd], 0.0)
        self.assertGreater(clear[back], 1.0)


class LidarTest(unittest.TestCase):
    def test_learns_blind_sector(self):
        model = oa.LidarModel(CFG)
        world = World(boxes=[(1.5, -3, 1.7, 3, 1.0), (-1.7, -3, -1.5, 3, 1.0),
                             (-3, 1.5, 3, 1.7, 1.0), (-3, -1.7, 3, -1.5, 1.0)])
        for k in range(40):
            scan = model.process(simulate_scan(world, (0.0, 0.0, 0.0)), LIDAR_ANGLES[0],
                                 LIDAR_ANGLES[1] - LIDAR_ANGLES[0], 0.1, 16.0, k * 0.1)
        self.assertTrue(model.is_blind(np.array([[-0.5, 0.0]]))[0])     # phía sau
        self.assertFalse(model.is_blind(np.array([[0.8, 0.0]]))[0])     # phía trước
        self.assertFalse(model.is_blind(np.array([[0.1, 0.6]]))[0])     # bên trái
        self.assertGreater(len(scan.points), 200)
        learned = model.learned_blind_bins()
        rear = model.bin_index(np.array([math.pi - 0.1]))[0]
        front = model.bin_index(np.array([0.0]))[0]
        self.assertTrue(learned[rear] and not learned[front])

    def test_open_space_is_not_blind(self):
        # Phòng trống không có tia về: KHÔNG được coi là bị che (trước đây chặn hết mọi hướng)
        model = oa.LidarModel(oa.AvoidanceConfig())
        for k in range(30):
            model.process(simulate_scan(World(), (0.0, 0.0, 0.0)), LIDAR_ANGLES[0],
                          LIDAR_ANGLES[1] - LIDAR_ANGLES[0], 0.1, 16.0, k * 0.1)
        self.assertFalse(model.is_blind(np.array([[1.0, 0.0], [0.0, 1.0]])).any())


class GridTest(unittest.TestCase):
    def test_lidar_marks_and_clears(self):
        grid = oa.LocalGrid(CFG)
        model = oa.LidarModel(CFG)
        world = World(circles=[(1.0, 0.0, 0.1, 1.0)])
        pose = (0.0, 0.0, 0.0)
        scan = model.process(simulate_scan(world, pose), LIDAR_ANGLES[0], LIDAR_ANGLES[1] - LIDAR_ANGLES[0],
                             0.1, 16.0, 1.0)
        grid.update_lidar(pose, scan, 1.0)
        obstacles, _ = grid.query(pose, 1.0)
        self.assertTrue(np.any(np.hypot(obstacles[:, 0] - 0.9, obstacles[:, 1]) < 0.06))
        # Vật biến mất: tia mới xuyên qua -> ô được xóa
        empty = World(boxes=[(2.5, -3, 2.7, 3, 1.0)])
        scan = model.process(simulate_scan(empty, pose), LIDAR_ANGLES[0], LIDAR_ANGLES[1] - LIDAR_ANGLES[0],
                             0.1, 16.0, 1.2)
        grid.update_lidar(pose, scan, 1.2)
        obstacles, _ = grid.query(pose, 1.2)
        self.assertFalse(np.any(np.hypot(obstacles[:, 0] - 0.9, obstacles[:, 1]) < 0.06))

    def test_lidar_does_not_clear_astra_obstacle(self):
        grid = oa.LocalGrid(CFG)
        pose = (0.0, 0.0, 0.0)
        table = np.array([[0.8, y] for y in np.linspace(-0.2, 0.2, 11)] * 2)
        grid.update_astra(pose, table, None, None, 1.0)
        model = oa.LidarModel(CFG)
        scan = model.process(simulate_scan(World(boxes=[(2.5, -3, 2.7, 3, 1.0)]), pose), LIDAR_ANGLES[0],
                             LIDAR_ANGLES[1] - LIDAR_ANGLES[0], 0.1, 16.0, 1.5)
        grid.update_lidar(pose, scan, 1.5)          # LiDAR quét dưới gầm "bàn"
        obstacles, _ = grid.query(pose, 1.5)
        self.assertTrue(np.any(np.hypot(obstacles[:, 0] - 0.8, obstacles[:, 1]) < 0.05))

    def test_memory_survives_recentering_and_motion(self):
        grid = oa.LocalGrid(CFG)
        pts = np.array([[1.2, 0.3]] * 2)
        grid.update_astra((0.0, 0.0, 0.0), pts, None, None, 1.0)
        pose = (0.9, 0.4, math.radians(30))
        grid.recenter(pose[0] + 0.8, pose[1])      # buộc dịch lưới
        obstacles, _ = grid.query(pose, 2.0, radius=1.5)
        expected = oa.inverse_transform_points(pose, [(1.2, 0.3)])[0]
        self.assertTrue(np.any(np.hypot(obstacles[:, 0] - expected[0], obstacles[:, 1] - expected[1]) < 0.05))

    def test_astra_memory_expires(self):
        grid = oa.LocalGrid(CFG)
        grid.update_astra((0.0, 0.0, 0.0), np.array([[0.8, 0.0]] * 2), None, None, 1.0)
        self.assertEqual(len(grid.query((0.0, 0.0, 0.0), 1.0 + CFG.astra_memory_s + 0.5)[0]), 0)


class AstraTest(unittest.TestCase):
    @staticmethod
    def synthetic_depth(height, pitch_deg, box=None, cliff_x=None):
        """Ảnh độ sâu 640x480 của mặt sàn (+ hộp vật cản, + mép bàn) với camera cao height, chúi pitch."""
        fx = fy = 574.1
        cx, cy = 320.0, 237.9
        v, u = np.mgrid[0:480, 0:640]
        r = np.dstack(((u - cx) / fx, (v - cy) / fy, np.ones((480, 640))))
        p = math.radians(pitch_deg)
        up = np.array([0.0, -math.cos(p), -math.sin(p)])
        fwd = np.array([0.0, math.sin(p), math.cos(p)])
        fwd = fwd - fwd.dot(up) * up
        fwd /= np.linalg.norm(fwd)
        left = np.cross(up, fwd)
        dz = r.dot(up)
        t = np.where(dz < -1e-6, height / -np.where(dz < -1e-6, dz, -1.0), np.inf)
        dx, dy = r.dot(fwd), r.dot(left)
        if cliff_x is not None:
            # Sau mép bàn (x > cliff_x) sàn tụt xuống 0.7 m
            gx = CFG.astra_x + t * dx
            far = np.where(dz < -1e-6, (height + 0.7) / -np.where(dz < -1e-6, dz, -1.0), np.inf)
            t = np.where(gx > cliff_x, far, t)
        if box is not None:
            x0, x1, y0, y1, top = box
            ox, oy, oz = CFG.astra_x, CFG.astra_y, height
            with np.errstate(divide='ignore', invalid='ignore'):
                tx = np.stack([(x0 - ox) / dx, (x1 - ox) / dx])
                ty = np.stack([(y0 - oy) / dy, (y1 - oy) / dy])
                tz = np.stack([(0.0 - oz) / dz, (top - oz) / dz])
            tmin = np.maximum(np.maximum(tx.min(0), ty.min(0)), tz.min(0))
            tmax = np.minimum(np.minimum(tx.max(0), ty.max(0)), tz.max(0))
            tb = np.where((tmax >= tmin) & (tmax > 0), np.maximum(tmin, 0), np.inf)
            t = np.minimum(t, tb)
        depth = np.where(np.isfinite(t) & (t < 6.0) & (t > 0.6), t * 1000.0, 0.0)
        return depth.astype(np.uint16), (fx, fy, cx, cy)

    def make(self, depth, intr, **cfg):
        astra = oa.AstraDepth(oa.AvoidanceConfig(**cfg))
        astra.set_intrinsics(intr[0], intr[1], intr[2], intr[3], 640, 480)
        return astra

    def test_calibrates_floor(self):
        depth, intr = self.synthetic_depth(0.47, 11.0)
        astra = self.make(depth, intr, astra_pitch_deg=4.0, astra_height=0.42)
        self.assertTrue(astra.calibrate(depth))
        self.assertAlmostEqual(astra.height, 0.47, delta=0.01)
        self.assertAlmostEqual(astra.pitch_deg, 11.0, delta=0.5)

    def test_detects_box_and_ignores_floor(self):
        depth, intr = self.synthetic_depth(0.462, 7.9, box=(1.2, 1.4, -0.15, 0.15, 0.25))
        astra = self.make(depth, intr)
        out = astra.process(depth)
        obs = out['obstacles']
        self.assertGreater(len(obs), 20)
        self.assertTrue(np.all(obs[:, 0] > 1.15) and np.all(obs[:, 0] < 1.45))
        self.assertTrue(np.all(np.abs(obs[:, 1]) < 0.2))
        self.assertGreater(len(out['floor']), 500)

    def test_detects_cliff(self):
        depth, intr = self.synthetic_depth(0.462, 7.9, cliff_x=1.5)
        astra = self.make(depth, intr)
        out = astra.process(depth)
        self.assertGreater(len(out['cliffs']), 50)
        self.assertTrue(np.all(out['cliffs'][:, 0] > 1.4))
        self.assertEqual(len(out['obstacles']), 0)

    def test_locate_target_with_depth(self):
        depth, intr = self.synthetic_depth(0.462, 7.9, box=(1.8, 2.0, 0.3, 0.6, 1.2))
        astra = self.make(depth, intr)
        rgb = (546.2, 546.2, 318.6, 238.1)
        # Chiếu tâm mặt trước hộp (x=1.8, y=0.45, z=0.5) lên ảnh RGB
        p = np.array([1.8 - CFG.astra_x, 0.45 - CFG.astra_y, 0.5 - 0.462])
        cam = np.array([-p.dot([0, 1, 0]), 0, 0])
        rc = p[0] * astra.forward + p[1] * astra.left + p[2] * astra.up
        u = rgb[2] + rgb[0] * rc[0] / rc[2]
        v = rgb[3] + rgb[1] * rc[1] / rc[2]
        bearing, dist, source = oa.locate_target((u - 40, v - 60, 80, 120), rgb, astra, depth, intr, CFG, 'box')
        self.assertEqual(source, 'DO SAU')
        self.assertAlmostEqual(dist, math.hypot(1.8, 0.45), delta=0.08)
        self.assertAlmostEqual(math.degrees(bearing), math.degrees(math.atan2(0.45, 1.8)), delta=2.0)
        del cam


class RenderTest(unittest.TestCase):
    def test_radar_places_obstacle_correctly(self):
        grid = oa.LocalGrid(CFG)
        pose = (0.7, -0.4, math.radians(35))
        grid.recenter(pose[0] + 1.2, pose[1])
        local = (1.05, 0.62)                      # vị trí vật trong khung xe
        world_pt = oa.transform_points(pose, [local])
        grid.update_astra(pose, np.vstack([oa.inverse_transform_points(pose, world_pt)] * 2), None, None, 5.0)
        img = oa.render_radar(CFG, grid, pose, 5.0, size=480, meters=2.4)
        s = 480 / 4.8
        px, py = int(round(240 - local[1] * s)), int(round(240 - local[0] * s))
        patch = img[py - 1:py + 2, px - 1:px + 2].reshape(-1, 3)
        self.assertTrue(any(tuple(p) == (0, 140, 255) for p in patch), patch)


class TrackerTest(unittest.TestCase):
    def test_needs_two_hits_and_remembers(self):
        tracker = oa.TargetTracker(CFG)
        tracker.observe('person', 0.2, 2.0, 1.0, (0.0, 0.0, 0.0))
        self.assertFalse(tracker.estimate(1.0).valid)
        tracker.observe('person', 0.2, 2.0, 1.2, (0.0, 0.0, 0.0))
        est = tracker.estimate(1.2)
        self.assertTrue(est.valid and est.visible)
        # Xe quay trái 90°: mục tiêu phải nằm bên phải (góc âm)
        est = tracker.estimate(3.0, (0.0, 0.0, math.radians(90)))
        self.assertTrue(est.valid and not est.visible)
        self.assertAlmostEqual(math.degrees(est.bearing), math.degrees(0.2) - 90.0, delta=1.0)
        self.assertFalse(tracker.estimate(1.2 + CFG.target_memory_s + 0.5).valid)


class PlannerTest(unittest.TestCase):
    def target(self, bearing_deg, dist, visible=True, label='person'):
        est = oa.TargetEstimate()
        est.valid, est.visible = True, visible
        est.bearing, est.distance = math.radians(bearing_deg), dist
        est.standoff, est.label = CFG.standoff_for(label), label
        est.radius = 0.35
        return est

    def test_open_space_goes_straight(self):
        planner = oa.MotionPlanner(CFG)
        for k in range(15):
            cmd = planner.plan(np.zeros((0, 2)), None, self.target(0, 3.0), 1.0 + k * 0.1)
        self.assertGreater(cmd.vx, 0.15)
        self.assertAlmostEqual(cmd.vy, 0.0, places=6)
        self.assertEqual(cmd.state, 'BAM')

    def test_stops_at_standoff(self):
        planner = oa.MotionPlanner(CFG)
        cmd = planner.plan(np.zeros((0, 2)), None, self.target(0, CFG.hx + 0.5), 1.0)
        self.assertEqual(cmd.state, 'DA_TOI')
        self.assertEqual((cmd.vx, cmd.vy), (0.0, 0.0))

    def test_wall_ahead_detours_and_keeps_side(self):
        planner = oa.MotionPlanner(CFG)
        rng = np.random.RandomState(3)
        wall = np.column_stack((np.full(31, 0.75), np.linspace(-0.6, 0.6, 31)))
        sides = []
        for k in range(40):
            noisy = wall + rng.normal(0, 0.01, wall.shape)
            cmd = planner.plan(noisy, None, self.target(rng.normal(0, 2.0), 2.5), 1.0 + k * 0.1)
            sides.append(planner.side)
        self.assertNotEqual(sides[-1], 0)
        self.assertEqual(side_flips(sides), 0)
        self.assertGreater(abs(cmd.vy), 0.02)

    def test_peeks_when_only_unknown_space_blocks(self):
        # Tường ngay trước, hông sau xe chưa từng thấy (vùng mù): phải xoay nhìn quanh, không đứng chết
        planner = oa.MotionPlanner(CFG)
        wall = np.column_stack((np.full(41, CFG.hx + 0.07), np.linspace(-0.8, 0.8, 41)))
        side = np.vstack([np.column_stack((np.linspace(-CFG.hx, 0.0, 9), np.full(9, s * (CFG.hy + 0.04))))
                          for s in (1, -1)])
        cmd = planner.plan(wall, side, self.target(0, 2.0), 1.0)
        self.assertEqual(cmd.state, 'NHIN_QUANH')
        self.assertGreater(abs(cmd.wz), 0.0)
        self.assertEqual((cmd.vx, cmd.vy), (0.0, 0.0))

    def test_blind_rear_blocks_backing_not_turning(self):
        planner = oa.MotionPlanner(CFG)
        blind = np.column_stack((np.full(20, -CFG.hx - 0.05), np.linspace(-0.1, 0.1, 20)))
        front = np.column_stack((np.full(20, CFG.hx + 0.04), np.linspace(-0.2, 0.2, 20)))
        cmd = planner.plan(front, blind, self.target(170, 2.0), 1.0)
        self.assertLessEqual(cmd.vx, 1e-9)
        self.assertGreater(abs(cmd.wz), 0.0)
        self.assertTrue(oa.motion_is_safe(front, 0.0, 0.0, cmd.wz, CFG)[0])


# ==========================================================
# MÔ PHỎNG VÒNG KÍN
# ==========================================================
class ClosedLoopTest(unittest.TestCase):
    def check(self, log, max_flips=2, reach=True):
        self.assertFalse(log['collided'], 'va cham tai {}'.format(log['pose']))
        if reach:
            self.assertTrue(log['reached'], 'khong toi duoc muc tieu, dung o {} trang thai {}'.format(
                log['pose'], log['states'][-5:]))
            self.assertLess(abs(log['final_gap'] - CFG.standoff_for('person')), 0.25)
        self.assertLessEqual(side_flips(log['sides']), max_flips)
        self.assertGreater(log['min_clear'], 0.0)

    def test_open_space(self):
        self.check(run_scenario(World(), (0, 0, 0), (3.0, 0.3)))

    def test_box_between_robot_and_target(self):
        world = World(boxes=[(1.3, -0.35, 1.7, 0.35, 0.5)])
        self.check(run_scenario(world, (0, 0, 0), (3.2, 0.0)))

    def test_gap_in_wall(self):
        world = World(boxes=[(1.4, -3.0, 1.55, -0.30, 0.6), (1.4, 0.30, 1.55, 3.0, 0.6)])
        self.check(run_scenario(world, (0, 0, 0), (3.0, 0.1)))

    def test_too_narrow_gap_never_collides(self):
        world = World(boxes=[(1.4, -3.0, 1.55, -0.14, 0.6), (1.4, 0.14, 1.55, 3.0, 0.6)])
        log = run_scenario(world, (0, 0, 0), (3.0, 0.0), seconds=25.0)
        self.check(log, max_flips=4, reach=False)
        self.assertLess(log['pose'][0], 1.4)

    def test_low_obstacle_only_astra_sees(self):
        # Hộp thấp 8 cm: LiDAR (13 cm) không thấy, Astra thấy từ xa rồi nhớ khi tới gần
        world = World(boxes=[(1.2, -0.3, 1.5, 0.3, 0.08)])
        self.check(run_scenario(world, (0, 0, 0), (3.0, 0.0)))

    def test_start_right_in_front_of_wall(self):
        # Khởi động sát tường (chưa biết gì về hông sau xe), mục tiêu ở sau tường
        world = World(boxes=[(0.40, -0.7, 0.50, 0.7, 0.6)])
        self.check(run_scenario(world, (0, 0, 0), (2.4, 0.0)))

    def test_target_behind_robot_found_by_search(self):
        self.check(run_scenario(World(), (0, 0, 0), (-2.5, 0.5)))

    def test_occluded_target_reached(self):
        world = World(circles=[(1.4, 0.0, 0.25, 1.0)])
        self.check(run_scenario(world, (0, 0, 0), (3.0, 0.0)))

    def test_wheel_slip(self):
        world = World(boxes=[(1.3, -0.35, 1.7, 0.35, 0.5)])
        self.check(run_scenario(world, (0, 0, 0), (3.2, 0.0), slip=0.15, seed=4))

    def test_odom_drift_from_lateral_slip(self):
        # Đi ngang chỉ đạt 75% lệnh nhưng odom vẫn tính theo lệnh (giống robot thật)
        world = World(boxes=[(1.3, -0.35, 1.7, 0.35, 0.5), (2.3, 0.3, 2.5, 1.0, 0.5)])
        log = run_scenario(world, (0, 0, 0), (3.6, 0.2), lateral_gain=0.75, odom_from_commands=True,
                           slip=0.1, seed=2)
        self.check(log)

    def test_random_clutter_no_collision(self):
        reached = 0
        for seed in range(8):
            rng = np.random.RandomState(seed)
            circles = [(rng.uniform(0.8, 2.6), rng.uniform(-1.2, 1.2), rng.uniform(0.06, 0.18), 0.6)
                       for _ in range(7)]
            circles = [c for c in circles if math.hypot(c[0], c[1]) > 0.6 and math.hypot(c[0] - 3.4, c[1]) > 0.7]
            log = run_scenario(World(circles=circles), (0, 0, 0), (3.4, 0.0), seconds=60.0, seed=seed)
            self.assertFalse(log['collided'], 'seed {} va cham'.format(seed))
            reached += log['reached']
        self.assertGreaterEqual(reached, 6)


if __name__ == '__main__':
    unittest.main(verbosity=2)
