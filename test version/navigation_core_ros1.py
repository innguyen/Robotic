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
                    default_standoff=0.35)

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


class Costmap(object):
    def __init__(self, cfg, collision_cfg):
        self.cfg, self.collision_cfg = cfg, collision_cfg
        self.cells = {}  # fixed odom cell -> (occupied, monotonic observation time)
        self.obstacle_points = {}  # preserve real hits instead of moving them to cell centers

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
        unique_cells = np.column_stack((keys//width+base[0], keys % width+base[1])).tolist()
        for i, j in unique_cells:
            cell = (i, j)
            self.cells[cell] = (False, now)
            self.obstacle_points.pop(cell, None)
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
        center = np.floor(np.asarray(pose[:2]) / cfg.resolution).astype(int)
        half = cfg.grid_size // 2 + 2
        self.cells = {key: value for key, value in self.cells.items()
                      if 0 <= now-value[1] <= cfg.memory_s
                      and max(abs(key[0]-center[0]), abs(key[1]-center[1])) <= half}
        self.obstacle_points = {key: points for key, points in self.obstacle_points.items()
                                if key in self.cells and self.cells[key][0]}

    def snapshot(self, pose, now):
        cfg, cc = self.cfg, self.collision_cfg
        origin = np.floor(np.asarray(pose[:2]) / cfg.resolution).astype(int) - cfg.grid_size // 2
        raw = np.full((cfg.grid_size, cfg.grid_size), -1, dtype=np.int8)
        obstacles = []
        origin_x, origin_y = int(origin[0]), int(origin[1])
        for (i, j), (occupied, stamp) in self.cells.items():
            row, col = j-origin_y, i-origin_x
            if 0 <= row < cfg.grid_size and 0 <= col < cfg.grid_size and 0 <= now-stamp <= cfg.memory_s:
                raw[row, col] = 100 if occupied else 0
            if occupied and 0 <= now-stamp <= cfg.memory_s:
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
        if footprint is not None:
            for point in obstacles:
                key = (int(math.floor(point[0]/.3)), int(math.floor(point[1]/.3)))
                self._buckets.setdefault(key, []).append(point)
            self._buckets = {key: np.asarray(value) for key, value in self._buckets.items()}

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
    if exact_goal and not grid.point_free(target):
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
    if best_frontier is not None and not exact_goal:
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
        stopping_distance = 0. if exact_goal else cc.body_length/2 + standoff
        if distance <= stopping_distance + cfg.goal_tolerance:
            self.reset()
            return zero, 'ARRIVED'
        # Preserve minimum target clearance as well as the requested standoff.
        if not exact_goal:
            stopping_distance = max(stopping_distance, math.hypot(cc.body_length, cc.body_width)/2 + cc.stop_margin + cfg.resolution*2)
        self.grid = self.costmap.snapshot(pose, now)
        if not self.grid.point_free(position):
            self.previous_velocity[:] = 0
            self.path = []
            return zero, 'START_BLOCKED'
        changed = self.route_target is None or np.linalg.norm(target-self.route_target) > .15 or self.route_mode != exact_goal
        if changed:
            self.stuck = False
            self.progress_anchor, self.progress_time = position.copy(), now
        if self.stuck:
            return zero, 'STUCK'
        if self.progress_anchor is None or np.linalg.norm(position-self.progress_anchor) >= cfg.progress_distance:
            self.progress_anchor, self.progress_time = position.copy(), now
        if self.progress_time is not None and now-self.progress_time > cfg.progress_timeout:
            self.stuck = True
            self.previous_velocity[:] = 0
            return zero, 'STUCK'
        invalid_path = bool(self.path) and any(not self.grid.free(self.grid.cell(p)) for p in self.path)
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
                return zero, 'PLANNING'
        elif changed or invalid_path or self.plan_time is None or now-self.plan_time >= 1./cfg.planner_hz:
            self.path, self.route_strategy, reached = plan_approach(self.grid, position, target, stopping_distance, pose[2], cc, exact_goal)
            self.plan_time, self.route_target = now, target.copy()
            self.route_mode = exact_goal
            self.plan_goal = self.path[-1].copy() if self.path else None
            self.planning_info = dict(strategy=self.route_strategy, reachable_cells=reached,
                                     approach_point=self.plan_goal.tolist() if self.plan_goal is not None else None)
        if not self.path:
            self.previous_velocity[:] = 0
            return zero, 'NO_REACHABLE_APPROACH'
        partial = self.route_strategy == 'OBSERVED_SUBGOAL'
        # Skip passed points and follow only a visible segment of the A* path.
        nearest = int(np.argmin([np.linalg.norm(p-position) for p in self.path]))
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
        # Fixed-heading holonomic control avoids commanding rotation through blind sectors.
        angle = math.degrees(math.atan2(vy, vx)-cc.scan_yaw)
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
        return (float(desired[0]), float(desired[1]), 0.), 'FOLLOW_PARTIAL' if partial else 'FOLLOW'
