"""Persistent OccupancyGrid layer and bounded global routing, also used offline.

The ROS map is owned by map_server / SLAM. This module never ages or ray-clears it.
Coordinates passed to MappedNavigator are in map, including search coverage.
"""
import math

import numpy as np

from collision_monitor_ros1 import Config
from navigation_core_ros1 import Navigator, Grid, dilate, plan_approach, to_world


class OccupancyMap(object):
    def __init__(self, raw, resolution, origin=(0., 0., 0.)):
        values = np.asarray(raw)
        if (values.ndim != 2 or not values.size or values.size > 2000000 or
                not np.isfinite(values).all() or np.any((values < -1) | (values > 100)) or
                not math.isfinite(resolution) or resolution <= 0 or
                len(origin) != 3 or not np.isfinite(origin).all()):
            raise ValueError('INVALID_GLOBAL_MAP')
        self.raw = np.where(values < 0, -1, np.where(values >= 65, 100,
                            np.where(values <= 19, 0, -1))).astype(np.int8)
        self.raw.setflags(write=False)
        self.resolution, self.origin = float(resolution), tuple(origin)
        self._planning_grids = {}
        cells = np.argwhere(self.raw == 100)[:, ::-1]
        self.obstacles = to_world((cells+.5)*resolution, origin)

    def local(self, points):
        xy = np.asarray(points, dtype=float).reshape(-1, 2)-self.origin[:2]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return np.column_stack((c*xy[:, 0]+s*xy[:, 1], -s*xy[:, 0]+c*xy[:, 1]))

    def sample(self, points):
        cells = np.floor(self.local(points)/self.resolution).astype(np.int64)
        cols, rows = cells[:, 0], cells[:, 1]
        inside = (rows >= 0) & (rows < self.raw.shape[0]) & (cols >= 0) & (cols < self.raw.shape[1])
        out = np.full(len(cells), -1, dtype=np.int8)
        out[inside] = self.raw[rows[inside], cols[inside]]
        return out

    def obstacles_near(self, point, radius):
        return self.obstacles[np.max(np.abs(self.obstacles-point), axis=1) <= radius]

    def planning_grid(self, radius):
        key = int(math.ceil(radius/self.resolution))
        if key not in self._planning_grids:
            blocked = dilate(self.raw != 0, key)
            # The map boundary is unknown too, including the full robot radius.
            blocked[:key, :] = blocked[-key:, :] = True
            blocked[:, :key] = blocked[:, -key:] = True
            self._planning_grids[key] = Grid(self.raw, ~blocked, (0, 0), self.resolution)
        return self._planning_grids[key]

    def plan(self, pose, target, standoff, cfg, cc, exact):
        # A circular global envelope allows heading changes along the route.
        # The local controller checks the full oriented rectangle and swept edges.
        grid = self.planning_grid(math.hypot(cc.body_length, cc.body_width)/2+cfg.inflation_radius)
        start, goal = self.local([pose[:2], target])
        distance = 0. if exact else max(cc.body_length/2+standoff,
                                      math.hypot(cc.body_length, cc.body_width)/2+cc.stop_margin+cfg.resolution*2)
        # Blind directions constrain instantaneous local motion, not a route that
        # will be followed with changing heading across the room.
        global_cc = Config(block_blind_motion=False)
        path, reason, visited = plan_approach(grid, start, goal, distance,
            pose[2]-self.origin[2], global_cc, exact, cfg.max_plan_expansions)
        return to_world(path, self.origin).tolist(), reason, visited

    def world_grid(self, resolution, radius):
        """Axis-aligned map-frame grid for camera coverage / exploration scoring."""
        h, w = self.raw.shape
        corners = to_world([[0, 0], [w*self.resolution, 0], [0, h*self.resolution],
                            [w*self.resolution, h*self.resolution]], self.origin)
        origin = np.floor(corners.min(axis=0)/resolution).astype(int)
        shape = np.ceil(corners.max(axis=0)/resolution).astype(int)-origin
        if int(shape[0])*int(shape[1]) > 250000:
            raise ValueError('EXPLORATION_MAP_TOO_LARGE')
        rows, cols = np.indices((shape[1], shape[0]))
        raw = self.sample((np.column_stack((cols.ravel(), rows.ravel()))+origin+.5)*resolution).reshape(rows.shape)
        blocked = dilate(raw != 0, int(math.ceil(radius/resolution)))
        return Grid(raw, (raw == 0) & ~blocked, origin, resolution)


class MappedNavigator(Navigator):
    def __init__(self, *args, **kwargs):
        super(MappedNavigator, self).__init__(*args, **kwargs)
        self.global_path, self.global_target = [], None
        self.global_future = None
        self.global_reason = ''
        self.global_generation = 0
        self.global_future_generation = None
        self.global_exact = None
        self.global_retry = float('-inf')
        self._local_step = False
        self.global_stuck = False
        self.global_best, self.global_progress = float('inf'), None
        self._exploration = None

    def reset(self):
        super(MappedNavigator, self).reset()
        if not self._local_step:
            self.global_generation += 1
            self.global_path, self.global_target = [], None
            self.global_reason = ''
            self.global_stuck = False
            self.global_best, self.global_progress = float('inf'), None

    def set_global_map(self, occupancy):
        old = self.costmap.global_map
        if (old is not None and old.resolution == occupancy.resolution and old.origin == occupancy.origin
                and np.array_equal(old.raw, occupancy.raw)):
            return
        self.costmap.global_map = occupancy
        self._exploration = None
        if self.global_path:
            radius = math.hypot(self.cc.body_length, self.cc.body_width)/2+self.cfg.inflation_radius
            grid = occupancy.planning_grid(radius)
            native = occupancy.local(self.global_path)
            keep = all(grid.free(grid.cell(p)) for p in native)
            if keep:
                return  # New SLAM observations need not restart a valid route.
        self.reset()

    def exploration_grid(self, pose, now):
        if self.costmap.global_map is None:
            return self.costmap.snapshot(pose, now)
        if self._exploration is None:
            radius = math.hypot(self.cc.body_length, self.cc.body_width)/2+self.cfg.inflation_radius
            self._exploration = self.costmap.global_map.world_grid(self.cfg.resolution, radius)
        return self._exploration

    def step(self, pose, target, standoff, now, dt, exact_goal=False):
        try:
            pose, target = np.asarray(pose, dtype=float), np.asarray(target, dtype=float)
            valid = (pose.shape == (3,) and target.shape == (2,) and
                     np.isfinite(pose).all() and np.isfinite(target).all() and
                     math.isfinite(standoff) and standoff >= 0)
        except (ValueError, TypeError, OverflowError):
            valid = False
        if not valid:
            self.reset()
            return (0., 0., 0.), 'INVALID_GOAL'
        occupancy = self.costmap.global_map
        if occupancy is None:
            return super(MappedNavigator, self).step(pose, target, standoff, now, dt, exact_goal)
        changed = (self.global_target is None or self.global_exact != exact_goal or
                   np.linalg.norm(np.asarray(target)-self.global_target) > .20)
        if changed:
            self.reset()
            self.global_target, self.global_exact = np.asarray(target).copy(), exact_goal
            self.global_retry = float('-inf')
        if self.global_stuck:
            return (0., 0., 0.), 'STUCK'
        if self.global_future is not None and self.global_future.done():
            path, reason, _ = self.global_future.result()
            if self.global_future_generation == self.global_generation:
                self.global_path = [np.asarray(p) for p in path]
                self.global_reason = reason
            self.global_future = None
        if not self.global_path and self.global_future is None and now-self.global_retry >= 2.:
            self.global_retry = now
            if self.executor is None:
                path, self.global_reason, _ = occupancy.plan(pose, target, standoff, self.cfg, self.cc, exact_goal)
                self.global_path = [np.asarray(p) for p in path]
            else:
                self.global_future_generation = self.global_generation
                self.global_future = self.executor.submit(occupancy.plan, tuple(pose), tuple(target),
                                                          standoff, self.cfg, self.cc, exact_goal)
        if not self.global_path:
            if self.global_reason == 'START_BLOCKED':
                # A circle can reject a pose where the oriented rectangle fits
                # (e.g. beside a chair). Let the exact local geometry move out;
                # retry global routing at the new pose without erasing the map.
                self._local_step = True
                try:
                    return super(MappedNavigator, self).step(pose, target, standoff, now, dt, exact_goal)
                finally:
                    self._local_step = False
            return (0., 0., 0.), 'GLOBAL_PLANNING' if self.global_future is not None else 'NO_GLOBAL_PATH'
        self.global_path = self.trim_path(self.global_path, pose[:2])
        remaining = self.path_length(self.global_path, pose[:2])
        if remaining < self.global_best-.10:
            self.global_best, self.global_progress = remaining, now
        elif self.global_progress is not None and now-self.global_progress > 30.:
            self.global_stuck = True
            return (0., 0., 0.), 'STUCK'
        endpoint = self.global_path[-1]
        final = np.linalg.norm(endpoint-np.asarray(pose[:2])) <= 1.1
        if final:
            local_target, local_exact = target, exact_goal
        else:
            # Keep the local goal on the global route, within the rolling map.
            local_grid = self.costmap.snapshot(pose, now)
            local_target = self.global_path[0]
            for point in self.global_path:
                if np.linalg.norm(point-np.asarray(pose[:2])) >= 1.0:
                    break
                if local_grid.point_free(point):
                    local_target = point
            local_exact = True
        self._local_step = True
        try:
            velocity, reason = super(MappedNavigator, self).step(pose, local_target, standoff,
                                                               now, dt, local_exact)
        finally:
            self._local_step = False
        if reason == 'STUCK':
            self.global_stuck = True
        if reason == 'ARRIVED' and not final:
            reason = 'FOLLOW_PARTIAL'
        self.planning_info.update(global_strategy=self.global_reason, global_remaining_m=round(remaining, 2))
        return velocity, reason
