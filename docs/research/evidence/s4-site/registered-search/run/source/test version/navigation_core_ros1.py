"""Small ROS-independent navigation core: rolling costmap, A*, holonomic control.

Original implementation inspired by Nav2's planner/controller/costmap separation.
No upstream Nav2 code or ROS 2 dependency. Positions are meters in odom.
"""
import heapq
import math
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from collision_monitor_ros1 import Config, in_sector, translation_sweep_clear


class NavConfig(object):
    DEFAULTS = dict(resolution=0.06, inflation_radius=0.06, grid_size=81, memory_s=4.0, ray_range=3.0,
                    planner_hz=2.0, control_hz=10.0, goal_tolerance=0.10,
                    lookahead=0.20, target_timeout=1.2, odom_timeout=0.5,
                    pose_sync_tolerance=0.15, acceleration=0.25,
                    progress_timeout=8.0, progress_distance=0.06,
                    default_standoff=0.35, heading_gain=1.0,
                    rotate_in_place_deg=45.0, align_tolerance_deg=15.0,
                    heading_tolerance_deg=25.0, recovery_offset_deg=20.0)

    def __init__(self, **kwargs):
        if set(kwargs) - set(self.DEFAULTS):
            raise ValueError('Unknown navigation parameter')
        for key, value in self.DEFAULTS.items():
            value = float(kwargs.get(key, value))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(key + ' must be positive and finite')
            setattr(self, key, value)
        if self.grid_size != int(self.grid_size) or not 11 <= self.grid_size <= 201:
            raise ValueError('grid_size must be an integer from 11 to 201')
        self.grid_size = int(self.grid_size)
        if self.grid_size % 2 == 0:
            raise ValueError('grid_size must be odd')


def to_world(points, pose):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((x + c * points[:, 0] - s * points[:, 1],
                            y + s * points[:, 0] + c * points[:, 1]))


def nearest_pose(history, stamp, tolerance):
    if not history:
        return None
    sample = min(history, key=lambda item: abs(item[0] - stamp))
    return sample[1] if abs(sample[0] - stamp) <= tolerance else None


def dilate(mask, radius):
    """Circular dilation without SciPy or OpenCV dependencies."""
    out = mask.copy()
    n, m = mask.shape
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            if di * di + dj * dj > radius * radius:
                continue
            src_i = slice(max(0, -di), min(n, n-di))
            src_j = slice(max(0, -dj), min(m, m-dj))
            dst_i = slice(max(0, di), min(n, n+di))
            dst_j = slice(max(0, dj), min(m, m+dj))
            out[dst_i, dst_j] |= mask[src_i, src_j]
    return out


def footprint_mask(points, origin, shape, yaw, hx, hy, resolution):
    """Test center nodes against measured hits, without blanket cell padding."""
    out = np.zeros(shape, dtype=bool)
    n, m = shape
    c, s = math.cos(yaw), math.sin(yaw)
    extent_x, extent_y = abs(c)*hx+abs(s)*hy, abs(s)*hx+abs(c)*hy
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    # Batch local candidate cells for all hits. Preserve exact measured points
    # and the same rectangle test; do not add padding or discard scan hits.
    rx, ry = int(math.ceil(extent_x/resolution))+1, int(math.ceil(extent_y/resolution))+1
    ox, oy = np.meshgrid(np.arange(-rx, rx+1), np.arange(-ry, ry+1))
    offsets = np.column_stack((ox.ravel(), oy.ravel()))
    for begin in range(0, len(points), 256):
        batch = points[begin:begin+256]
        cells = np.floor(batch/resolution).astype(np.int64)[:, None, :]+offsets[None, :, :]
        delta = (cells+.5)*resolution-batch[:, None, :]
        cols, rows = cells[:, :, 0]-origin[0], cells[:, :, 1]-origin[1]
        inside = ((np.abs(c*delta[:, :, 0]+s*delta[:, :, 1]) <= hx) &
                  (np.abs(-s*delta[:, :, 0]+c*delta[:, :, 1]) <= hy) &
                  (cols >= 0) & (cols < m) & (rows >= 0) & (rows < n))
        out[rows[inside].astype(int), cols[inside].astype(int)] = True
    return out


# One-cell (drow, dcol) moves; the opposite moves reuse these masks from the other end.
SWEEP_MOVES = ((0, 1), (1, 0), (1, 1), (1, -1))


def sweep_mask(points, origin, shape, yaw, hx, hy, resolution, move):
    """Start cells whose fixed-heading rectangle sweep by one cell `move` hits a point."""
    out = np.zeros(shape, dtype=bool)
    n, m = shape
    c, s = math.cos(yaw), math.sin(yaw)
    world = (move[1]*resolution, move[0]*resolution)
    mx, my = c*world[0]+s*world[1], -s*world[0]+c*world[1]
    extent_x, extent_y = abs(c)*hx+abs(s)*hy, abs(s)*hx+abs(c)*hy
    rx = int(math.ceil((extent_x+abs(world[0]))/resolution))+1
    ry = int(math.ceil((extent_y+abs(world[1]))/resolution))+1
    ox, oy = np.meshgrid(np.arange(-rx, rx+1), np.arange(-ry, ry+1))
    offsets = np.column_stack((ox.ravel(), oy.ravel()))
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    for begin in range(0, len(points), 256):
        batch = points[begin:begin+256]
        cells = np.floor(batch/resolution).astype(np.int64)[:, None, :]+offsets[None, :, :]
        delta = batch[:, None, :]-(cells+.5)*resolution
        start = (c*delta[:, :, 0]+s*delta[:, :, 1], -s*delta[:, :, 0]+c*delta[:, :, 1])
        enter, leave = np.zeros(start[0].shape), np.ones(start[0].shape)
        for local, direction, half in ((start[0], mx, hx), (start[1], my, hy)):
            if abs(direction) < 1e-12:
                enter[np.abs(local) > half] = float('inf')
            else:
                t1, t2 = (local-half)/direction, (local+half)/direction
                enter = np.maximum(enter, np.minimum(t1, t2))
                leave = np.minimum(leave, np.maximum(t1, t2))
        cols, rows = cells[:, :, 0]-origin[0], cells[:, :, 1]-origin[1]
        hit = (enter <= leave) & (cols >= 0) & (cols < m) & (rows >= 0) & (rows < n)
        out[rows[hit].astype(int), cols[hit].astype(int)] = True
    return out


class Costmap(object):
    def __init__(self, cfg, collision_cfg):
        self.cfg, self.collision_cfg = cfg, collision_cfg
        self.cells = {}  # fixed odom cell -> (occupied, monotonic observation time)
        self.obstacle_points = {}  # preserve real hits instead of moving them to cell centers
        self.last_prune = float('-inf')

    def update(self, ranges, angle_min, angle_increment, range_min, range_max, pose, now):
        cc, cfg = self.collision_cfg, self.cfg
        values = np.asarray(ranges, dtype=float)
        if (values.ndim != 1 or len(values) < 3 or not np.isfinite(pose).all()
                or not all(math.isfinite(v) for v in (angle_min, angle_increment, range_min, range_max))
                or angle_increment == 0 or not 0 <= range_min < range_max):
            raise ValueError('Invalid scan or pose')
        angles = angle_min + np.arange(len(values)) * angle_increment
        finite = np.isfinite(values) & (values >= range_min) & (values <= range_max) & (values > 0)
        deg = (np.degrees(angles)+180.) % 360.-180.
        usable = np.ones(len(values), dtype=bool)
        for lo, hi in cc.blind_sectors_deg:
            blind = ((deg >= lo) & (deg <= hi)) if lo <= hi else ((deg >= lo) | (deg <= hi))
            usable &= ~blind
        valid = usable & (finite | np.isposinf(values))
        if not usable.any() or valid.sum() / float(usable.sum()) < cc.min_valid_fraction:
            raise ValueError('Insufficient usable scan rays')
        distances = np.minimum(values[valid], min(range_max, cfg.ray_range))
        bearings = angles[valid] + cc.scan_yaw
        local = np.column_stack((cc.scan_x + distances * np.cos(bearings),
                                 cc.scan_y + distances * np.sin(bearings)))
        endpoints = to_world(local, pose)
        origin = to_world([[cc.scan_x, cc.scan_y]], pose)[0]
        # Ray clearing first, then every actual hit; a free ray cannot erase another
        # obstacle endpoint in the same scan. +Inf is clear only out to ray_range.
        selected = endpoints[::max(1, int(math.ceil(len(endpoints) / 360.0)))]
        max_length = float(np.linalg.norm(selected-origin, axis=1).max())
        count = max(2, int(math.ceil(max_length / (cfg.resolution * 0.5))) + 1)
        fractions = np.linspace(0., 1., count)[None, :, None]
        samples = origin + (selected-origin)[:, None, :]*fractions
        cells = np.floor(samples.reshape(-1, 2)/cfg.resolution).astype(np.int64)
        base = cells.min(axis=0)
        width = int(cells[:, 1].max()-base[1])+1
        # Deduplicate in NumPy instead of inserting tens of thousands of duplicate
        # ray cells in Python, which delays odometry on the Nano.
        keys = np.unique((cells[:, 0]-base[0])*width+cells[:, 1]-base[1])
        # dict.fromkeys/update run in C; stale obstacle points of cleared cells are
        # dropped by the occupied-cell filter at the end of this update.
        free_value = (False, now)
        self.cells.update(dict.fromkeys(zip((keys//width+base[0]).tolist(), (keys % width+base[1]).tolist()), free_value))
        hits = finite[valid] & (values[valid] <= cfg.ray_range)
        current_hits = {}
        for point, (i, j) in zip(endpoints[hits].tolist(), np.floor(endpoints[hits] / cfg.resolution).astype(int).tolist()):
            cell = (i, j)
            self.cells[cell] = (True, now)
            current_hits.setdefault(cell, []).append((float(point[0]), float(point[1])))
        self.obstacle_points.update(current_hits)
        # The occupied robot footprint is known space; never erase actual hits.
        xs = np.arange(-cc.body_length/2, cc.body_length/2 + cfg.resolution/2, cfg.resolution/2)
        ys = np.arange(-cc.body_width/2, cc.body_width/2 + cfg.resolution/2, cfg.resolution/2)
        body = to_world(np.array([(x, y) for x in xs for y in ys]), pose)
        for i, j in np.floor(body / cfg.resolution).astype(int):
            key = (int(i), int(j))
            if not self.cells.get(key, (False, 0))[0]:
                self.cells[key] = (False, now)
        # Expired/out-of-window cells are already ignored by snapshot(); pruning the
        # whole dict every scan is slow on the Nano, so do it a few times per second.
        if now-self.last_prune >= .25 or now < self.last_prune:
            self.last_prune = now
            center = np.floor(np.asarray(pose[:2]) / cfg.resolution).astype(int)
            half = cfg.grid_size // 2 + 2
            cx, cy = int(center[0]), int(center[1])
            self.cells = {key: value for key, value in self.cells.items()
                          if 0 <= now-value[1] <= cfg.memory_s
                          and abs(key[0]-cx) <= half and abs(key[1]-cy) <= half}
        self.obstacle_points = {key: points for key, points in self.obstacle_points.items()
                                if self.cells.get(key, (False,))[0]}

    def snapshot(self, pose, now):
        cfg, cc = self.cfg, self.collision_cfg
        origin = np.floor(np.asarray(pose[:2]) / cfg.resolution).astype(int) - cfg.grid_size // 2
        raw = np.full((cfg.grid_size, cfg.grid_size), -1, dtype=np.int8)
        obstacles = []
        origin_x, origin_y = int(origin[0]), int(origin[1])
        if self.cells:
            # Bulk conversion instead of a per-cell Python loop (dominant cost on the Nano).
            keys = np.array(list(self.cells.keys()), dtype=np.int64).reshape(-1, 2)
            values = np.array(list(self.cells.values()), dtype=float).reshape(-1, 2)
            age = now-values[:, 1]
            fresh = (age >= 0) & (age <= cfg.memory_s)
            occupied = fresh & (values[:, 0] > .5)
            rows, cols = keys[:, 1]-origin_y, keys[:, 0]-origin_x
            inside = fresh & (rows >= 0) & (rows < cfg.grid_size) & (cols >= 0) & (cols < cfg.grid_size)
            raw[rows[inside], cols[inside]] = np.where(occupied[inside], 100, 0)
            for i, j in keys[occupied].tolist():
                obstacles.extend(self.obstacle_points.get((i, j), [((i+.5)*cfg.resolution, (j+.5)*cfg.resolution)]))
        obstacles = np.asarray(obstacles, dtype=float).reshape(-1, 2)
        # The visible buffer is 6 cm by default. Chassis clearance is checked
        # separately using its actual rectangle, not a 36 cm circular buffer.
        inflated = dilate(raw == 100, int(math.ceil(cfg.inflation_radius / cfg.resolution)))
        hx, hy = cc.body_length/2+cfg.inflation_radius, cc.body_width/2+cfg.inflation_radius
        body_blocked = footprint_mask(obstacles, origin, raw.shape, pose[2], hx, hy, cfg.resolution)
        allowed = (raw == 0) & ~body_blocked
        return Grid(raw, allowed, origin, cfg.resolution, inflated, obstacles, (hx, hy, pose[2]))


class Grid(object):
    def __init__(self, raw, allowed, origin, resolution, inflated=None, obstacles=None, footprint=None):
        self.raw, self.allowed = raw, allowed
        self.origin, self.resolution = np.asarray(origin), resolution
        self.inflated = ~allowed if inflated is None else inflated
        self.obstacles, self.footprint = obstacles, footprint
        self._buckets = {}
        self._edge_cache = {}
        self._sweep_blocked = None
        if footprint is not None and len(obstacles):
            # Vectorized bucketing: per-point Python/NumPy calls are very slow on the Nano.
            points = np.asarray(obstacles, dtype=float).reshape(-1, 2)
            keys = np.floor(points/.3).astype(np.int64)
            order = np.lexsort((keys[:, 1], keys[:, 0]))
            keys, points = keys[order], points[order]
            starts = np.flatnonzero(np.r_[True, np.any(keys[1:] != keys[:-1], axis=1)])
            for begin, end in zip(starts.tolist(), np.r_[starts[1:], len(keys)].tolist()):
                self._buckets[(int(keys[begin, 0]), int(keys[begin, 1]))] = points[begin:end]

    def sweep_blocked(self):
        """Per-direction masks of blocked one-cell moves, computed once per grid.

        Same slab test as translation_sweep_clear, batched over all obstacle points
        and start cells: per-edge NumPy calls made A* take seconds on the Nano.
        """
        if self._sweep_blocked is None:
            hx, hy, yaw = self.footprint
            self._sweep_blocked = {move: sweep_mask(self.obstacles, self.origin, self.raw.shape,
                                                    yaw, hx, hy, self.resolution, move)
                                   for move in SWEEP_MOVES}
        return self._sweep_blocked

    def nearby(self, a, b):
        hx, hy, yaw = self.footprint
        c, s = math.cos(yaw), math.sin(yaw)
        extent = (abs(c)*hx+abs(s)*hy, abs(s)*hx+abs(c)*hy)
        lower = np.floor((np.minimum(a, b)-extent)/.3).astype(int)
        upper = np.floor((np.maximum(a, b)+extent)/.3).astype(int)
        pieces = [self._buckets[(i, j)] for i in range(lower[0], upper[0]+1)
                  for j in range(lower[1], upper[1]+1) if (i, j) in self._buckets]
        return np.concatenate(pieces) if pieces else np.empty((0, 2))

    def cell(self, point):
        i, j = np.floor(np.asarray(point) / self.resolution).astype(int) - self.origin
        return int(j), int(i)

    def point(self, cell):
        row, col = cell
        return (np.array([col, row]) + self.origin + 0.5) * self.resolution

    def free(self, cell):
        i, j = cell
        return 0 <= i < self.allowed.shape[0] and 0 <= j < self.allowed.shape[1] and self.allowed[i, j]

    def pose_free(self, point, yaw):
        """Inflated footprint at another heading clear of obstacle points (for turning)."""
        if self.footprint is None or self.obstacles is None or not len(self.obstacles):
            return True
        hx, hy, _ = self.footprint
        delta = np.asarray(self.obstacles)-np.asarray(point, dtype=float)
        delta = delta[np.abs(delta).max(axis=1) <= math.hypot(hx, hy)]
        c, s = math.cos(yaw), math.sin(yaw)
        return not np.any((np.abs(c*delta[:, 0]+s*delta[:, 1]) <= hx)
                          & (np.abs(-s*delta[:, 0]+c*delta[:, 1]) <= hy))

    def inside(self, point):
        i, j = self.cell(point)
        return 0 <= i < self.raw.shape[0] and 0 <= j < self.raw.shape[1]

    def known_blocked(self, point):
        """Observed space where the robot cannot stand. Unknown/out-of-window is not blocked:
        such goals are explored through observed frontiers instead of being rejected."""
        i, j = self.cell(point)
        if not (0 <= i < self.raw.shape[0] and 0 <= j < self.raw.shape[1]) or self.raw[i, j] < 0:
            return False
        return not self.point_free(point)

    def point_free(self, point):
        if self.footprint is None:
            return self.free(self.cell(point))
        i, j = self.cell(point)
        if not (0 <= i < self.raw.shape[0] and 0 <= j < self.raw.shape[1] and self.raw[i, j] == 0):
            return False
        hx, hy, yaw = self.footprint
        delta = self.nearby(point, point)-np.asarray(point)
        c, s = math.cos(yaw), math.sin(yaw)
        return not np.any((np.abs(c*delta[:, 0]+s*delta[:, 1]) <= hx)
                          & (np.abs(-s*delta[:, 0]+c*delta[:, 1]) <= hy))

    def edge_free(self, cell, nxt):
        if self.footprint is not None:
            hx, hy, yaw = self.footprint
            if self.obstacles is None or not len(self.obstacles):
                return True
            move = (nxt[0]-cell[0], nxt[1]-cell[1])
            start = cell
            if move not in SWEEP_MOVES:
                move, start = (-move[0], -move[1]), nxt
            n, m = self.raw.shape
            if move in SWEEP_MOVES and 0 <= start[0] < n and 0 <= start[1] < m:
                return not self.sweep_blocked()[move][start]
            key = (cell, nxt) if cell < nxt else (nxt, cell)
            if key not in self._edge_cache:
                a, b = self.point(cell), self.point(nxt)
                self._edge_cache[key] = translation_sweep_clear(self.nearby(a, b), a, b, yaw, hx, hy)
            return self._edge_cache[key]
        if cell[0] != nxt[0] and cell[1] != nxt[1]:
            return self.free((cell[0], nxt[1])) and self.free((nxt[0], cell[1]))
        return True

    def segment_free(self, a, b):
        if self.footprint is not None:
            # Exact swept rectangle for constant-heading translation. This avoids
            # rounding the continuous robot pose back to a blocked cell center.
            hx, hy, yaw = self.footprint
            if not translation_sweep_clear(self.nearby(a, b), a, b, yaw, hx, hy):
                return False
        distance = float(np.linalg.norm(np.asarray(b)-a))
        count = max(2, int(math.ceil(distance / (self.resolution * .25))) + 1)
        previous = None
        for point in np.linspace(a, b, count):
            cell = self.cell(point)
            if self.footprint is None:
                if not self.free(cell):
                    return False
            elif not (0 <= cell[0] < self.raw.shape[0] and 0 <= cell[1] < self.raw.shape[1] and self.raw[cell] == 0):
                return False
            if self.footprint is None and previous and cell[0] != previous[0] and cell[1] != previous[1]:
                if not self.free((cell[0], previous[1])) or not self.free((previous[0], cell[1])):
                    return False
            previous = cell
        return True


def astar(grid, start, goal, yaw=None, collision_cfg=None):
    start, goal = grid.cell(start), grid.cell(goal)
    if not grid.free(start) or not grid.free(goal):
        return []
    queue = [(math.hypot(goal[0]-start[0], goal[1]-start[1]), 0., start)]
    costs, parent = {start: 0.}, {}
    moves = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1) if i or j]
    while queue:
        _, cost, cell = heapq.heappop(queue)
        if cost > costs[cell] + 1e-9:
            continue
        if cell == goal:
            path = [cell]
            while cell in parent:
                cell = parent[cell]
                path.append(cell)
            return [grid.point(c) for c in reversed(path)]
        for di, dj in moves:
            if yaw is not None and collision_cfg is not None and collision_cfg.block_blind_motion:
                angle = math.degrees(math.atan2(di, dj)-yaw-collision_cfg.scan_yaw)
                if any(in_sector(angle, lo, hi) for lo, hi in collision_cfg.blind_sectors_deg):
                    continue
            nxt = (cell[0]+di, cell[1]+dj)
            if not grid.free(nxt):
                continue
            if not grid.edge_free(cell, nxt):
                continue
            new_cost = cost + math.hypot(di, dj)
            if new_cost < costs.get(nxt, float('inf')):
                costs[nxt], parent[nxt] = new_cost, cell
                heuristic = math.hypot(goal[0]-nxt[0], goal[1]-nxt[1])
                heapq.heappush(queue, (new_cost+heuristic, new_cost, nxt))
    return []


def plan_approach(grid, start, target, distance, yaw, cc, exact_goal=False):
    """Multi-goal A*: any reachable point on the complete standoff ring.

    If the ring is unseen, an observed frontier that makes real progress may be
    used as a temporary observation point. Unknown cells are never traversed.
    """
    start_cell = grid.cell(start)
    if not grid.point_free(start):
        return [], 'START_BLOCKED', 0
    # An exact goal in unobserved space (e.g. behind the robot) is approached via
    # observed frontiers until LiDAR sees it; only observed-blocked goals are refused.
    explore = exact_goal and not grid.point_free(target)
    if explore and grid.known_blocked(target):
        return [], 'GOAL_BLOCKED_OR_UNKNOWN', 0
    target = np.asarray(target)
    start_distance = float(np.linalg.norm(target-start))
    res = grid.resolution
    origin_x, origin_y = float(grid.origin[0]), float(grid.origin[1])
    target_x, target_y = float(target[0]), float(target[1])
    def remaining_at(cell):
        return math.hypot((cell[1]+origin_x+.5)*res-target_x,
                          (cell[0]+origin_y+.5)*res-target_y)
    def heuristic(cell):
        return max(0., remaining_at(cell)-distance)/res
    queue = [(heuristic(start_cell), 0., start_cell)]
    costs, parent, visited = {start_cell: 0.}, {}, set()
    best_frontier, frontier_score = None, start_distance-.10
    moves = [(i, j) for i in (-1, 0, 1) for j in (-1, 0, 1) if i or j]
    allowed_moves = []
    for di, dj in moves:
        angle = math.degrees(math.atan2(di, dj)-yaw-cc.scan_yaw)
        if cc.block_blind_motion and any(in_sector(angle, lo, hi) for lo, hi in cc.blind_sectors_deg):
            continue
        allowed_moves.append((di, dj))

    def reconstruct(cell):
        result = [cell]
        while cell in parent:
            cell = parent[cell]
            result.append(cell)
        path = [grid.point(c) for c in reversed(result)]
        if len(path) == 1:
            return [np.asarray(start, dtype=float).copy(), path[0]]
        path[0] = np.asarray(start, dtype=float).copy()
        return path

    while queue:
        _, cost, cell = heapq.heappop(queue)
        if cell in visited or cost > costs[cell]+1e-9:
            continue
        visited.add(cell)
        remaining = remaining_at(cell)
        if exact_goal and remaining <= res*1.5:
            endpoint = grid.point(cell) if cell != start_cell else np.asarray(start)
            delta = np.asarray(target)-endpoint
            angle = math.degrees(math.atan2(delta[1], delta[0])-yaw-cc.scan_yaw)
            direction_ok = (np.linalg.norm(delta) < 1e-9 or not cc.block_blind_motion or
                            not any(in_sector(angle, lo, hi) for lo, hi in cc.blind_sectors_deg))
            if direction_ok and grid.segment_free(endpoint, target):
                return reconstruct(cell)+[np.asarray(target).copy()], 'MAP_POINT', len(visited)
        if not exact_goal and distance <= remaining <= distance+res*1.5:
            delta = grid.point(cell)-start
            angle = math.degrees(math.atan2(delta[1], delta[0])-yaw-cc.scan_yaw)
            blocked_start_connection = (cell == start_cell and cc.block_blind_motion
                and any(in_sector(angle, lo, hi) for lo, hi in cc.blind_sectors_deg))
            if not blocked_start_connection:
                return reconstruct(cell), 'APPROACH_RING', len(visited)
        # A frontier can expose the occluded approach on a subsequent scan. This
        # permits a known-space detour instead of insisting the final goal be seen.
        if cost*res >= .20 and remaining >= distance and remaining <= start_distance-.10:
            frontier = any(not (0 <= cell[0]+di < grid.raw.shape[0] and 0 <= cell[1]+dj < grid.raw.shape[1])
                           or grid.raw[cell[0]+di, cell[1]+dj] < 0 for di, dj in moves)
            score = remaining+.35*cost*res
            if frontier and score < frontier_score:
                best_frontier, frontier_score = cell, score
        for di, dj in (moves if cell == start_cell else allowed_moves):
            nxt = (cell[0]+di, cell[1]+dj)
            if not grid.free(nxt):
                continue
            if cell != start_cell and not grid.edge_free(cell, nxt):
                continue
            step_cost = math.hypot(di, dj)
            if cell == start_cell:
                # The robot is not exactly at its cell center. Connect from its
                # actual position rather than commanding a small backward step
                # to a discretized start point in a blind direction.
                delta = grid.point(nxt)-start
                angle = math.degrees(math.atan2(delta[1], delta[0])-yaw-cc.scan_yaw)
                if cc.block_blind_motion and any(in_sector(angle, lo, hi) for lo, hi in cc.blind_sectors_deg):
                    continue
                if not grid.segment_free(start, grid.point(nxt)):
                    continue
                step_cost = float(np.linalg.norm(delta))/res
            new_cost = cost+step_cost
            if new_cost < costs.get(nxt, float('inf')):
                costs[nxt], parent[nxt] = new_cost, cell
                heapq.heappush(queue, (new_cost+heuristic(nxt), new_cost, nxt))
    if best_frontier is not None and (not exact_goal or explore):
        return reconstruct(best_frontier), 'OBSERVED_SUBGOAL', len(visited)
    return [], 'NO_REACHABLE_APPROACH', len(visited)


class Navigator(object):
    def __init__(self, cfg=None, collision_cfg=None, async_planning=False):
        self.cfg, self.cc = cfg or NavConfig(), collision_cfg or Config()
        self.costmap = Costmap(self.cfg, self.cc)
        self.path, self.grid = [], None
        self.plan_time = None
        self.plan_goal = None
        self.previous_velocity = np.zeros(2)
        self.progress_anchor = None
        self.progress_time = None
        self.stuck = False
        self.route_target = None
        self.route_strategy = ''
        self.planning_info = {}
        self.executor = ThreadPoolExecutor(max_workers=1) if async_planning else None
        self.plan_future = None
        self.generation = 0
        self.future_generation = None
        self.route_mode = False
        self.planner_ms = 0.
        self.recovery_ref = self.recovery_wait = self.recovery_started = None
        self.recovery_step = 0

    def close(self):
        if self.executor is not None:
            self.executor.shutdown(wait=True)

    def reset(self):
        self.generation += 1
        self.path = []
        self.plan_time = self.plan_goal = None
        self.previous_velocity[:] = 0
        self.progress_anchor = self.progress_time = None
        self.stuck = False
        self.route_target = None
        self.route_strategy = ''
        self.planning_info = {}
        self.recovery_ref = self.recovery_wait = self.recovery_started = None
        self.recovery_step = 0

    def _recover(self, pose, now):
        """Small spin recovery: try headings target+-offset so a blind sector moves off the route.

        Offsets stay inside the camera view, so the vision target is not lost. Each
        heading is held for two planner periods; after all offsets, give up (bounded).
        """
        cfg, cc = self.cfg, self.cc
        offsets = (-cfg.recovery_offset_deg, cfg.recovery_offset_deg)
        while self.recovery_step < len(offsets):
            if self.recovery_started is None:
                self.recovery_started = now
            goal = self.recovery_ref+math.radians(offsets[self.recovery_step])
            error = (goal-pose[2]+math.pi) % (2*math.pi)-math.pi
            # Turning in place is not lack of progress.
            self.progress_anchor, self.progress_time = np.asarray(pose[:2], dtype=float).copy(), now
            if abs(error) > math.radians(3.) and now-self.recovery_started <= 6.:
                self.recovery_wait = None
                speed = min(cc.max_angular, max(.15, cfg.heading_gain*abs(error)))
                return (0., 0., math.copysign(speed, error)), 'RECOVERY_ROTATE'
            if self.recovery_wait is None:
                self.recovery_wait = now
            if now-self.recovery_wait <= 2./cfg.planner_hz:
                return (0., 0., 0.), 'RECOVERY_WAIT'
            self.recovery_step += 1
            self.recovery_wait = self.recovery_started = None
        return None

    def _escape(self, position, yaw, speed=.05):
        """Move directly away from the nearest obstacle point, if that is observed space."""
        cc, grid = self.cc, self.grid
        if grid.obstacles is None or not len(grid.obstacles):
            return None
        i, j = grid.cell(position)
        if not (0 <= i < grid.raw.shape[0] and 0 <= j < grid.raw.shape[1] and grid.raw[i, j] == 0):
            return None
        delta = grid.obstacles-position
        distance = np.linalg.norm(delta, axis=1)
        nearest = int(np.argmin(distance))
        if distance[nearest] < 1e-6 or distance[nearest] > .6:
            return None
        away = -delta[nearest]/distance[nearest]
        c, s = math.cos(yaw), math.sin(yaw)
        base = math.atan2(-s*away[0]+c*away[1], c*away[0]+s*away[1])   # body frame
        # Straight away first; if that points into the LiDAR blind sector (common right
        # after a spin reveals a rear obstacle), try directions still moving away from it.
        for offset in (0., 45., -45., 70., -70.):
            heading = base+math.radians(offset)
            angle = math.degrees(heading-cc.scan_yaw)
            if not (cc.block_blind_motion and any(in_sector(angle, lo, hi) for lo, hi in cc.blind_sectors_deg)):
                return (float(math.cos(heading)*speed), float(math.sin(heading)*speed), 0.)
        return None

    def step(self, pose, target, standoff, now, dt, exact_goal=False):
        cfg, cc = self.cfg, self.cc
        zero = (0., 0., 0.)
        position = np.asarray(pose[:2], dtype=float)
        target = np.asarray(target, dtype=float)
        if (not np.isfinite(pose).all() or not np.isfinite(target).all()
                or not math.isfinite(standoff) or standoff < 0):
            self.reset()
            return zero, 'INVALID_GOAL'
        offset = target-position
        distance = float(np.linalg.norm(offset))
        # Face the goal: keeps a vision target in the camera; turns toward map points (also behind).
        heading_error = (math.atan2(offset[1], offset[0])-pose[2]+math.pi) % (2*math.pi)-math.pi
        if exact_goal and distance < .30:
            # Map points: face the travel direction, but do not spin on the last few cm.
            heading_error = 0.
        turn = float(np.clip(cfg.heading_gain*heading_error, -cc.max_angular, cc.max_angular))
        stopping_distance = 0. if exact_goal else cc.body_length/2 + standoff
        if distance <= stopping_distance + cfg.goal_tolerance:
            if not exact_goal and abs(heading_error) > math.radians(cfg.align_tolerance_deg):
                self.previous_velocity[:] = 0
                return (0., 0., turn), 'ALIGN'
            self.reset()
            return zero, 'ARRIVED'
        # Preserve minimum target clearance as well as the requested standoff.
        if not exact_goal:
            stopping_distance = max(stopping_distance, math.hypot(cc.body_length, cc.body_width)/2 + cc.stop_margin + cfg.resolution*2)
        self.grid = self.costmap.snapshot(pose, now)
        if not self.grid.point_free(position):
            self.previous_velocity[:] = 0
            self.path = []
            escape = self._escape(position, pose[2])
            if escape is not None:
                # Inside the planning buffer only: back away slowly; the monitor still checks it.
                self.progress_anchor, self.progress_time = position.copy(), now
                return escape, 'ESCAPE'
            return zero, 'START_BLOCKED'
        if (abs(heading_error) > math.radians(cfg.rotate_in_place_deg)
                and self.grid.pose_free(position, pose[2]+turn*.5)):
            # Turn first: a target behind or beside the robot lies in blind/unplanned space.
            # Not when turning would swing a corner into the buffer: move away first.
            # Rotation is not lack of progress, so keep the stuck timer fresh.
            self.previous_velocity[:] = 0
            self.path = []
            self.progress_anchor, self.progress_time = position.copy(), now
            return (0., 0., turn), 'ROTATE'
        changed =self.route_target is None or np.linalg.norm(target-self.route_target) > .15 or self.route_mode != exact_goal
        if changed:
            self.stuck = False
            self.progress_anchor, self.progress_time = position.copy(), now
            self.recovery_ref = self.recovery_wait = self.recovery_started = None
            self.recovery_step = 0
        if self.stuck:
            return zero, 'STUCK'
        if self.progress_anchor is None or np.linalg.norm(position-self.progress_anchor) >= cfg.progress_distance:
            self.progress_anchor, self.progress_time = position.copy(), now
        if self.progress_time is not None and now-self.progress_time > cfg.progress_timeout:
            self.stuck = True
            self.previous_velocity[:] = 0
            return zero, 'STUCK'
        # Path ends are real positions (robot start, exact goal), not cell centres: a
        # blocked centre cell must fall back to the exact footprint test, otherwise the
        # async planner discards every path near obstacles and the robot crawls to STUCK.
        invalid_path = bool(self.path) and any(
            not self.grid.free(self.grid.cell(p)) and not self.grid.point_free(p) for p in self.path)
        if self.executor is not None:
            if changed:
                self.generation += 1
                self.path = []
                self.plan_time = None
                self.route_target, self.route_mode = target.copy(), exact_goal
            if invalid_path:
                self.path = []
            if self.plan_future is not None and self.plan_future.done():
                result, self.planner_ms = self.plan_future.result()
                if self.future_generation == self.generation:
                    self.path, self.route_strategy, reached = result
                    self.plan_time = now
                    self.plan_goal = self.path[-1].copy() if self.path else None
                    self.planning_info = dict(strategy=self.route_strategy, reachable_cells=reached,
                        approach_point=self.plan_goal.tolist() if self.plan_goal is not None else None)
                self.plan_future = None
            due = self.plan_time is None or now-self.plan_time >= 1./cfg.planner_hz or invalid_path
            if self.plan_future is None and due:
                # The worker owns an immutable grid snapshot. It never writes the
                # live map, pose, path or command; only this control loop accepts results.
                self.future_generation = self.generation
                def compute(grid, position, goal, heading):
                    started = time.monotonic()
                    result = plan_approach(grid, position, goal, stopping_distance, heading, cc, exact_goal)
                    return result, (time.monotonic()-started)*1000.
                self.plan_future = self.executor.submit(compute, self.grid, position.copy(), target.copy(), pose[2])
            if not self.path and self.plan_future is not None:
                self.previous_velocity[:] = 0
                recovery = self._recover(pose, now) if self.recovery_ref is not None else None
                return recovery if recovery is not None else (zero, 'PLANNING')
        elif changed or invalid_path or self.plan_time is None or now-self.plan_time >= 1./cfg.planner_hz:
            self.path, self.route_strategy, reached = plan_approach(self.grid, position, target, stopping_distance, pose[2], cc, exact_goal)
            self.plan_time, self.route_target = now, target.copy()
            self.route_mode = exact_goal
            self.plan_goal = self.path[-1].copy() if self.path else None
            self.planning_info = dict(strategy=self.route_strategy, reachable_cells=reached,
                                     approach_point=self.plan_goal.tolist() if self.plan_goal is not None else None)
        if not self.path:
            self.previous_velocity[:] = 0
            if self.recovery_ref is None:
                self.recovery_ref = pose[2]+heading_error
            recovery = self._recover(pose, now)
            return recovery if recovery is not None else (zero, 'NO_REACHABLE_APPROACH')
        self.recovery_wait = self.recovery_started = None
        partial = self.route_strategy == 'OBSERVED_SUBGOAL'
        # Skip passed points and follow only a visible segment of the A* path.
        nearest = int(np.argmin(np.linalg.norm(np.asarray(self.path)-position, axis=1)))
        self.path = self.path[nearest:]
        waypoint = None
        for point in self.path:
            delta = point-position
            if np.linalg.norm(delta) < cfg.resolution*.25:
                continue
            local_angle = math.degrees(math.atan2(delta[1], delta[0])-pose[2]-cc.scan_yaw)
            if cc.block_blind_motion and any(in_sector(local_angle, lo, hi) for lo, hi in cc.blind_sectors_deg):
                continue
            if not self.grid.segment_free(position, point):
                break
            waypoint = point
            if np.linalg.norm(point-position) >= cfg.lookahead:
                break
        if waypoint is None:
            self.previous_velocity[:] = 0
            self.path = []
            return zero, 'PATH_BLOCKED'
        error = waypoint-position
        yaw = pose[2]
        vx = math.cos(yaw)*error[0] + math.sin(yaw)*error[1]
        vy = -math.sin(yaw)*error[0] + math.cos(yaw)*error[1]
        # Holonomic translation along the path plus a heading correction toward the target.
        angle =math.degrees(math.atan2(vy, vx)-cc.scan_yaw)
        if cc.block_blind_motion and any(in_sector(angle, lo, hi) for lo, hi in cc.blind_sectors_deg):
            self.previous_velocity[:] = 0
            return zero, 'UNOBSERVED_DIRECTION'
        scale = min(1., (cc.max_forward if vx >= 0 else cc.max_backward)/max(abs(vx), 1e-12),
                    cc.max_lateral/max(abs(vy), 1e-12))
        desired = np.array([vx, vy])*scale
        delta = desired-self.previous_velocity
        # Acceleration only. A stop is immediate, and no smoothing may cut a corner.
        max_delta = cfg.acceleration*min(max(dt, .001), .2)
        if np.linalg.norm(delta) > max_delta:
            desired = self.previous_velocity + delta/np.linalg.norm(delta)*max_delta
        c, s = math.cos(yaw), math.sin(yaw)
        predicted = position + np.array([c*desired[0]-s*desired[1], s*desired[0]+c*desired[1]])*.3
        if not self.grid.segment_free(position, predicted):
            self.previous_velocity[:] = 0
            return zero, 'CONTROL_BLOCKED'
        self.previous_velocity = desired
        # Small errors keep the target in the camera view; leaving wz at zero keeps the
        # monitor on its exact translation sweep when passing close to obstacles.
        if (abs(heading_error) <= math.radians(cfg.heading_tolerance_deg)
                or not self.grid.pose_free(position, pose[2]+turn*.5)):
            # Also hold heading beside obstacles: a turning corner would enter the buffer
            # and leave the robot START_BLOCKED.
            turn = 0.
        return (float(desired[0]), float(desired[1]), turn), 'FOLLOW_PARTIAL' if partial else 'FOLLOW'
