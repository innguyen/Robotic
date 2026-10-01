"""ROS-independent navigation geometry. SI units; base x forward, y left, z up."""
import json
import math
import threading
import ast
from collections import deque

import numpy as np


def angle_delta(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def load_config(path):
    with open(path, encoding='utf-8') as stream:
        cfg = json.load(stream)
    for key in ('robot_height_m', 'height_margin_m', 'horizontal_margin_m',
                'fallback_radius_m', 'reaction_time_s', 'braking_accel_mps2'):
        if not isinstance(cfg[key], (int, float)) or not math.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('Invalid navigation setting: ' + key)
    for key in ('body_length_m', 'body_width_m'):
        if cfg[key] is not None and (not math.isfinite(cfg[key]) or cfg[key] <= 0):
            raise ValueError('Invalid body size: ' + key)
    for key in ('body_center_x_m', 'body_center_y_m', 'lidar_x_m', 'lidar_y_m', 'lidar_yaw_deg'):
        if not math.isfinite(cfg[key]):
            raise ValueError('Invalid extrinsics: ' + key)
    radius = cfg.get('body_radius_m')
    if radius is not None and (not isinstance(radius, (int, float)) or not math.isfinite(radius) or radius <= 0):
        raise ValueError('Invalid body size: body_radius_m')
    return cfg


def footprint_config(cfg, polygon, source, frame):
    """Use an existing navigation footprint, never infer body size from free space."""
    if isinstance(polygon, str):
        polygon = ast.literal_eval(polygon)
    points = np.asarray(polygon, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 3
            or not np.all(np.isfinite(points))):
        raise ValueError('Invalid robot footprint polygon')
    low, high = points.min(axis=0), points.max(axis=0)
    size = high - low
    if np.any(size < .10) or np.any(size > 2.0) or np.any(low >= 0) or np.any(high <= 0):
        raise ValueError('Footprint must surround base origin and use metres')
    if frame.lstrip('/') not in ('base_link', 'base_footprint'):
        raise ValueError('Unknown footprint frame')
    updated = dict(cfg)
    updated.update(body_length_m=float(size[0]), body_width_m=float(size[1]),
                   body_center_x_m=float((low[0] + high[0]) / 2),
                   body_center_y_m=float((low[1] + high[1]) / 2),
                   footprint_source=source, footprint_frame=frame.lstrip('/'))
    return updated


def discover_footprint(cfg, get_param):
    """ROS navigation parameters first, then the installed navigation package YAML."""
    if cfg['body_length_m'] and cfg['body_width_m']:
        return cfg, 'navigation_config.json'
    if cfg.get('body_radius_m'):
        return cfg, 'navigation_config.json: measured body radius'
    for namespace in ('/move_base/local_costmap', '/move_base/global_costmap',
                      '/local_costmap', '/global_costmap'):
        polygon = get_param(namespace + '/footprint', None)
        frame = get_param(namespace + '/robot_base_frame', '')
        if polygon is not None and frame:
            try:
                result = footprint_config(cfg, polygon, namespace + '/footprint', frame)
                return result, result['footprint_source']
            except (ValueError, TypeError, SyntaxError):
                continue
    try:
        import os
        import glob
        import yaml
        import rospkg
        root = rospkg.RosPack().get_path('jetauto_navigation')
        paths = sorted(glob.glob(os.path.join(root, '**', '*costmap*.yaml'), recursive=True))
        # Only accept an unambiguous footprint/frame pair within the same directory.
        choices = []
        for directory in sorted(set(os.path.dirname(path) for path in paths)):
            merged = {}
            def collect(node):
                if isinstance(node, dict):
                    for key, value in node.items():
                        if key in ('footprint', 'robot_base_frame'):
                            merged.setdefault(key, []).append(value)
                        elif isinstance(value, dict):
                            collect(value)
            for path in paths:
                if os.path.dirname(path) == directory:
                    with open(path, encoding='utf-8') as stream:
                        collect(yaml.safe_load(stream))
            frames = set(str(f).lstrip('/') for f in merged.get('robot_base_frame', []))
            if len(frames) != 1:
                continue
            for polygon in merged.get('footprint', []):
                choices.append(footprint_config(cfg, polygon, directory, next(iter(frames))))
        keys = ('body_length_m', 'body_width_m', 'body_center_x_m', 'body_center_y_m', 'footprint_frame')
        unique = {tuple(choice[key] for key in keys) for choice in choices}
        if len(unique) == 1:
            return choices[0], choices[0]['footprint_source']
    except Exception:
        pass
    return cfg, 'UNCONFIGURED: NEED BODY DIMENSIONS'


def same_planar_frame(first, second, description):
    first, second = first.lstrip('/'), second.lstrip('/')
    if first == second:
        return True
    try:
        import xml.etree.ElementTree as ET
        graph = {}
        for joint in ET.fromstring(description).findall('joint'):
            if joint.get('type') != 'fixed':
                continue
            origin = joint.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0, 0, 0]
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0, 0, 0]
            if len(xyz) != 3 or len(rpy) != 3 or any(not math.isfinite(v) or abs(v) > 1e-6 for v in xyz[:2] + rpy):
                continue
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            graph.setdefault(parent, []).append(child)
            graph.setdefault(child, []).append(parent)
        pending, seen = [first], set()
        while pending:
            node = pending.pop()
            if node == second:
                return True
            if node not in seen:
                seen.add(node)
                pending.extend(graph.get(node, []))
    except Exception:
        pass
    return False


class PoseHistory(object):
    """Interpolate only bracketed ROS timestamps; never extrapolate stale poses."""
    def __init__(self):
        self.samples = deque(maxlen=600)
        self.lock = threading.Lock()
        self.generation = 0

    def add(self, stamp, x, y, yaw):
        if stamp <= 0 or not all(math.isfinite(v) for v in (stamp, x, y, yaw)):
            return False
        with self.lock:
            if self.samples:
                old = self.samples[-1]
                dt = stamp - old[0]
                if dt == 0:
                    return False
                if (dt < 0 or math.hypot(x - old[1], y - old[2]) > max(0.3, dt * 1.5)
                        or abs(angle_delta(yaw, old[3])) > max(0.5, dt * 3.0)):
                    self.samples.clear()
                    self.generation += 1
            self.samples.append((stamp, x, y, yaw))
        return True

    def interpolate(self, stamps):
        with self.lock:
            data = np.asarray(self.samples, dtype=float)
        stamps = np.asarray(stamps, dtype=float)
        if len(data) < 2 or stamps.size == 0:
            return None
        idx = np.searchsorted(data[:, 0], stamps, side='right')
        idx = np.clip(idx, 1, len(data) - 1)
        lo, hi = data[idx - 1], data[idx]
        gap = hi[:, 0] - lo[:, 0]
        if (np.any(stamps < data[0, 0]) or np.any(stamps > data[-1, 0])
                or np.any(gap > 0.20)):
            return None
        weight = (stamps - lo[:, 0]) / gap
        yaw_diff = (hi[:, 3] - lo[:, 3] + math.pi) % (2 * math.pi) - math.pi
        return np.column_stack((lo[:, 1] + weight * (hi[:, 1] - lo[:, 1]),
                                lo[:, 2] + weight * (hi[:, 2] - lo[:, 2]),
                                lo[:, 3] + weight * yaw_diff))

    def describe_wait(self, stamps):
        with self.lock:
            data = list(self.samples)
        if len(data) < 2:
            return 'ODOM HISTORY: {} SAMPLES'.format(len(data))
        if len(stamps) == 0:
            return 'EMPTY SCAN'
        if stamps[0] < data[0][0]:
            return 'SCAN BEFORE ODOM HISTORY BY {:.3f}S'.format(data[0][0] - stamps[0])
        if stamps[-1] > data[-1][0]:
            return 'WAIT ODOM FOR LAST RAY: {:.3f}S'.format(stamps[-1] - data[-1][0])
        return 'ODOM GAP EXCEEDS 0.20S'


def scan_geometry(ranges, angle_min, angle_increment, poses, cfg, range_min, range_max):
    """Deskew each ray using its own pose, including laser-to-base extrinsics."""
    ranges = np.asarray(ranges, dtype=float)
    valid = np.isposinf(ranges) | (np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max))
    ranges = ranges[valid]
    poses = poses[valid]
    angles = angle_min + np.flatnonzero(valid) * angle_increment
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    origins = poses[:, :2] + np.column_stack((c * cfg['lidar_x_m'] - s * cfg['lidar_y_m'],
                                             s * cfg['lidar_x_m'] + c * cfg['lidar_y_m']))
    directions = poses[:, 2] + math.radians(cfg['lidar_yaw_deg']) + angles
    lengths = np.minimum(ranges, min(4.0, range_max))
    ends = origins + lengths[:, None] * np.column_stack((np.cos(directions), np.sin(directions)))
    hits = np.isfinite(ranges) & (ranges <= min(4.0, range_max))
    return origins, ends, hits


class ScanRecovery(object):
    """Count net actual rotation, retry twice, then try planning in observed space."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous_yaw = None
        self.net_angle = 0.0
        self.direction = 1
        self.attempt = 0
        self.started = None
        self.progress_time = None
        self.best = 0.0
        self.wait_until = None

    def step(self, yaw, now):
        if self.started is None:
            self.started = self.progress_time = now
            self.previous_yaw = yaw
        delta = angle_delta(yaw, self.previous_yaw)
        self.previous_yaw = yaw
        self.net_angle += delta
        progress = abs(self.net_angle)
        if progress >= 2 * math.pi - 0.20:
            return 'PLAN', 0.0
        if self.wait_until is not None:
            if now < self.wait_until:
                return 'RECOVER', 0.0
            self.wait_until = None
            self.progress_time = now
        if progress > self.best + math.radians(3):
            self.best = progress
            self.progress_time = now
        if now - self.progress_time > 4.0 or now - self.started > 40.0:
            if self.attempt >= 2:
                return 'PLAN_PARTIAL', 0.0
            self.attempt += 1
            self.direction *= -1
            self.started = self.progress_time = now
            self.net_angle = self.best = 0.0
            self.wait_until = now + 2.0
            return 'RECOVER', 0.0
        return 'SCAN', self.direction * 18.0

    def partial_probe_due(self, threshold=math.pi / 2.0):
        """Allow a short safe translation before the full rotation completes."""
        return abs(self.net_angle) >= threshold


class FootprintSafety(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.rectangular = bool(cfg['body_length_m'] and cfg['body_width_m'])
        self.calibrated = self.rectangular or bool(cfg.get('body_radius_m'))
        circle_radius = cfg.get('body_radius_m') or cfg['fallback_radius_m']
        self.hx = cfg['body_length_m'] / 2 if self.rectangular else circle_radius
        self.hy = cfg['body_width_m'] / 2 if self.rectangular else circle_radius
        self.cx, self.cy = cfg['body_center_x_m'], cfg['body_center_y_m']
        self.radius = math.hypot(self.hx + abs(self.cx), self.hy + abs(self.cy)) if self.rectangular else self.hx
        self.margin = cfg['horizontal_margin_m']
        self.snapshot = None
        self.coverage_lookup = None
        self.lock = threading.Lock()

    def update(self, ranges, amin, inc, rmin, rmax, received):
        values = np.asarray(ranges, dtype=float)
        angles = amin + np.arange(len(values)) * inc
        valid = np.isposinf(values) | (np.isfinite(values) & (values >= max(0.12, rmin)) & (values <= rmax))
        # Keep invalid rays unknown. Never fill missing sectors by interpolation.
        a = angles + math.radians(self.cfg['lidar_yaw_deg'])
        finite = valid & np.isfinite(values)
        points = np.column_stack((self.cfg['lidar_x_m'] + values[finite] * np.cos(a[finite]),
                                  self.cfg['lidar_y_m'] + values[finite] * np.sin(a[finite])))
        normalized = (angles + math.pi) % (2 * math.pi) - math.pi
        order = np.argsort(normalized)
        sorted_angles = normalized[order]
        lookup_angles = np.concatenate((sorted_angles[-1:] - 2 * math.pi,
                                        sorted_angles, sorted_angles[:1] + 2 * math.pi))
        lookup_idx = np.concatenate((order[-1:], order, order[:1]))
        with self.lock:
            self.snapshot = (values, angles, valid, points, received, rmax, abs(inc), lookup_angles, lookup_idx)

    def inside(self, points, x=0.0, y=0.0, yaw=0.0, margin=0.0):
        dx, dy = points[:, 0] - x, points[:, 1] - y
        c, s = math.cos(yaw), math.sin(yaw)
        bx, by = c * dx + s * dy - self.cx, -s * dx + c * dy - self.cy
        if self.rectangular:
            return (np.abs(bx) <= self.hx + margin) & (np.abs(by) <= self.hy + margin)
        return bx * bx + by * by <= (self.radius + margin) ** 2

    def perimeter(self):
        if not self.rectangular:
            a = np.linspace(-math.pi, math.pi, 100, endpoint=False)
            return np.column_stack(((self.radius + self.margin) * np.cos(a) + self.cx,
                                    (self.radius + self.margin) * np.sin(a) + self.cy))
        x = np.linspace(-self.hx - self.margin, self.hx + self.margin, 35)
        y = np.linspace(-self.hy - self.margin, self.hy + self.margin, 35)
        return np.vstack((np.column_stack((x, np.full_like(x, self.hy + self.margin))),
                          np.column_stack((x, np.full_like(x, -self.hy - self.margin))),
                          np.column_stack((np.full_like(y, self.hx + self.margin), y)),
                          np.column_stack((np.full_like(y, -self.hx - self.margin), y)))) + [self.cx, self.cy]

    def check(self, vx, vy, wz_deg, now):
        if abs(vx) + abs(vy) + abs(wz_deg) < 1e-8:
            return True, ''
        with self.lock:
            snapshot = self.snapshot
        if snapshot is None or now - snapshot[4] > 0.5:
            return False, 'LIDAR STALE'
        values, angles, valid, points, received, rmax, inc, lookup_angles, lookup_idx = snapshot
        if len(values) < 10 or inc <= 0:
            return False, 'LIDAR INVALID'
        speed = math.hypot(vx, vy)
        rotation_only = speed < 1e-8 and abs(wz_deg) > 1e-8
        if rotation_only and not self.rectangular:
            # A centered circular body occupies the same space at every yaw.
            # Turning needs neither rear coverage nor a simulated trajectory.
            occupied = self.inside(points, margin=0.0)
            if np.any(occupied):
                return False, 'FOOTPRINT COLLISION: PHYSICAL BODY'
            return True, ''
        omega = math.radians(wz_deg)
        edge_speed = speed + abs(omega) * self.radius
        # Constant commanded speed throughout reaction + full braking time is conservative.
        horizon = self.cfg['reaction_time_s'] + edge_speed / self.cfg['braking_accel_mps2'] + max(0, now - received)
        samples = max(3, int(math.ceil(edge_speed * horizon / 0.02)) + 1)
        boundary = self.perimeter()
        # A return inside the safety margin can be escaped, but a return
        # inside the measured physical body or a move deeper into the margin
        # must still stop the chassis.
        initially_close = self.inside(points, margin=self.margin)
        if not rotation_only and not self.rectangular:
            initial_distance = np.linalg.norm(points - [self.cx, self.cy], axis=1)
            if np.any(initial_distance <= self.radius):
                return False, 'FOOTPRINT COLLISION: PHYSICAL BODY'
            previous_distance = initial_distance.copy()
        for t in np.linspace(0, horizon, samples):
            theta = omega * t
            if abs(omega) > 1e-7:
                x = (vx * math.sin(theta) + vy * (math.cos(theta) - 1)) / omega
                y = (vx * (1 - math.cos(theta)) + vy * math.sin(theta)) / omega
            else:
                x, y = vx * t, vy * t
            collision_margin = 0.0 if rotation_only and not self.rectangular else self.margin
            blocked = self.inside(points, x, y, theta, collision_margin)
            if not rotation_only and not self.rectangular:
                center = [x + self.cx * math.cos(theta) - self.cy * math.sin(theta),
                          y + self.cx * math.sin(theta) + self.cy * math.cos(theta)]
                distance = np.linalg.norm(points - center, axis=1)
                # Existing safety-margin overlap is acceptable only while
                # the commanded movement increases clearance monotonically.
                recovering = initially_close & (distance >= previous_distance - 1e-6)
                blocked &= ~recovering
                previous_distance = distance
            if np.any(blocked):
                obstacle = points[blocked][0]
                shape = ('BODY' if self.calibrated else
                         'UNCALIBRATED R={:.0f}CM'.format(100 * (self.radius + self.margin)))
                return False, 'FOOTPRINT COLLISION: {} X={:.0f} Y={:.0f}CM'.format(
                    shape, obstacle[0] * 100, obstacle[1] * 100)
            # In-place turns may proceed with incomplete coverage. Still check
            # every predicted pose against measured obstacles above.
            if rotation_only:
                continue
            c, s = math.cos(theta), math.sin(theta)
            perimeter = boundary.dot(np.array([[c, s], [-s, c]])) + [x, y]
            # Only the advancing half enters new space. Missing rays behind
            # the chassis cannot veto a forward or sideways translation.
            moving = np.array([c * vx - s * vy, s * vx + c * vy])
            advancing = np.sum((perimeter - [x, y]) * moving, axis=1) > 0
            perimeter = perimeter[advancing]
            # Coverage is needed only for newly entered space. Testing the
            # original footprint also vetoed valid moves away from a margin
            # overlap that the collision test above explicitly allows.
            perimeter = perimeter[~self.inside(perimeter, margin=self.margin + 1e-6)]
            if not len(perimeter):
                continue
            rel = perimeter - [self.cfg['lidar_x_m'], self.cfg['lidar_y_m']]
            bearing = np.arctan2(rel[:, 1], rel[:, 0]) - math.radians(self.cfg['lidar_yaw_deg'])
            bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
            upper = np.clip(np.searchsorted(lookup_angles, bearing), 1, len(lookup_angles) - 1)
            lower = upper - 1
            nearest = np.where(np.abs(lookup_angles[lower] - bearing) <= np.abs(lookup_angles[upper] - bearing), lower, upper)
            idx = lookup_idx[nearest]
            covered = np.abs(lookup_angles[nearest] - bearing) <= min(0.025, inc * 1.6)
            measured = np.where(np.isposinf(values[idx]), rmax, values[idx])
            known = covered & valid[idx]
            # Bridge only sub-degree dropouts bracketed by real valid rays.
            # Use the smaller range; never extend across an unobserved sector.
            valid_lookup = valid[lookup_idx]
            va = lookup_angles[valid_lookup]
            vi = lookup_idx[valid_lookup]
            if len(va) >= 2 and np.any(~known):
                hi = np.clip(np.searchsorted(va, bearing), 1, len(va) - 1)
                lo = hi - 1
                bracketed = ((va[lo] <= bearing) & (bearing <= va[hi]) &
                             (va[hi] - va[lo] <= math.radians(1.0) + 1e-9))
                conservative = np.minimum(np.where(np.isposinf(values[vi[lo]]), rmax, values[vi[lo]]),
                                          np.where(np.isposinf(values[vi[hi]]), rmax, values[vi[hi]]))
                fill = ~known & bracketed & (np.isnan(values[idx]) | (values[idx] == 0))
                measured = np.where(fill, conservative, measured)
                known |= fill
            clear = known & (measured >= np.linalg.norm(rel, axis=1))
            # Use recently measured free cells to cover intermittent missing
            # rays, never to override a current measured obstruction.
            missing = ~known & (np.isnan(values[idx]) | (values[idx] == 0))
            if np.any(missing) and self.coverage_lookup is not None:
                try:
                    recent = np.asarray(self.coverage_lookup(perimeter[missing], now), dtype=bool)
                    if recent.shape == (int(np.count_nonzero(missing)),):
                        if np.any(recent) and speed > .081:
                            return False, 'RECENT COVERAGE: SLOW TO 8CM/S'
                        clear[missing] = recent
                except Exception:
                    pass  # Failure to obtain evidence never authorizes motion.
            if not np.all(clear):
                failure = int(np.flatnonzero(~clear)[0])
                kind = 'UNKNOWN' if not known[failure] else 'BLOCKED'
                return False, 'SWEPT AREA {}: BEARING={:.1f} DEG'.format(
                    kind, math.degrees(bearing[failure]))
        if not rotation_only and not self.rectangular and np.any(initially_close):
            if np.any(previous_distance[initially_close] < initial_distance[initially_close] + 0.005):
                return False, 'FOOTPRINT COLLISION: NO CLEARANCE GAIN'
        return True, ''


class DepthClearance(object):
    """Prove a forward volume free using calibrated optical depth; unknown stays blocked."""
    def __init__(self, cfg, footprint):
        self.cfg, self.footprint = cfg, footprint
        self.snapshot = None
        self.lock = threading.Lock()
        self.status = 'DISABLED: NEED CALIBRATION'
        self.enabled = bool(cfg['depth_enabled'])
        keys = ('camera_x_m', 'camera_y_m', 'camera_height_m', 'camera_pitch_down_deg', 'camera_yaw_deg')
        self.ready = (footprint.calibrated and cfg['depth_extrinsics_calibrated']
                      and all(isinstance(cfg[k], (int, float)) and math.isfinite(cfg[k]) for k in keys))

    def update(self, depth, intrinsics, received):
        with self.lock:
            self.snapshot = (depth.copy(), tuple(intrinsics), received)

    def check(self, vx, vy, wz, now):
        if abs(vx) + abs(vy) + abs(wz) < 1e-8:
            return True, ''
        if not self.enabled:
            return True, self.status
        if not self.ready:
            self.status = 'DEPTH NEED CALIBRATION'
            return False, self.status
        if vx <= 0 or abs(vy) > 0.001 or abs(wz) > 0.5:
            self.status = 'DEPTH ONLY CERTIFIES STRAIGHT FORWARD'
            return False, self.status
        with self.lock:
            data = self.snapshot
        if data is None or now - data[2] > 0.4:
            self.status = 'DEPTH STALE'
            return False, self.status
        depth, (fx, fy, cx, cy), received = data
        cfg, fp = self.cfg, self.footprint
        stop = vx * (cfg['reaction_time_s'] + now - received) + vx * vx / (2 * cfg['braking_accel_mps2'])
        front = fp.cx + fp.hx + fp.margin
        # Sample the entire volume the front will enter, not just a centre pixel.
        xs = np.arange(front, front + max(0.20, stop) + 0.021, 0.02)
        ys = np.arange(fp.cy - fp.hy - fp.margin, fp.cy + fp.hy + fp.margin + 0.021, 0.02)
        zs = np.arange(0.03, cfg['robot_height_m'] + cfg['height_margin_m'] + 0.021, 0.02)
        x, y, z = np.meshgrid(xs, ys, zs, indexing='ij')
        rel = np.column_stack((x.ravel() - cfg['camera_x_m'], y.ravel() - cfg['camera_y_m'], z.ravel() - cfg['camera_height_m']))
        yaw, pitch = math.radians(cfg['camera_yaw_deg']), math.radians(cfg['camera_pitch_down_deg'])
        forward = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch)])
        right = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
        down = np.cross(forward, right)
        optical_z = rel.dot(forward)
        with np.errstate(divide='ignore', invalid='ignore'):
            u = np.rint(fx * rel.dot(right) / optical_z + cx)
            v = np.rint(fy * rel.dot(down) / optical_z + cy)
        h, w = depth.shape
        visible = (optical_z > 0.15) & np.isfinite(u) & np.isfinite(v) & (u >= 1) & (u < w - 1) & (v >= 1) & (v < h - 1)
        if not np.all(visible):
            self.status = 'DEPTH VOLUME OUTSIDE VIEW'
            return False, self.status
        u, v = u.astype(int), v.astype(int)
        # Invalid pixels in the projected swept volume are not evidence of free space.
        for du, dv in ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)):
            distance = depth[v + dv, u + du] * 0.001
            if np.any(~np.isfinite(distance) | (distance < 0.15)):
                self.status = 'DEPTH UNKNOWN PIXELS'
                return False, self.status
            if np.any(distance < optical_z + 0.02):
                self.status = 'DEPTH BODY VOLUME BLOCKED'
                return False, self.status
        self.status = 'DEPTH CLEAR TO {:.0f}CM HEIGHT'.format(100 * (cfg['robot_height_m'] + cfg['height_margin_m']))
        return True, self.status
