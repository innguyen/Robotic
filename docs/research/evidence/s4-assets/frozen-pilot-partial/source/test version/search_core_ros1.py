"""Search-and-find mission on top of Navigator: spin scans, camera-coverage exploration,
approach on detection, revisit the last sighting, report the found position.

ROS-independent; shared by navigation_ros1.py and the desktop simulation so both run
the same mission logic. Positions are metres in odom.
"""
import math

import numpy as np

from navigation_core_ros1 import dilate, to_world


class SearchConfig(object):
    DEFAULTS = dict(spin_step_deg=45.0, spin_dwell_s=1.2, camera_fov_deg=58.0, camera_range=3.5,
                    min_goal_distance=0.5, goal_timeout_s=45.0, revisit_radius=0.7,
                    blacklist_radius=0.4, lost_standoff=0.6, min_gain_cells=12.0,
                    fail_hold_s=2.5, stall_s=15.0, stall_distance=0.10, explore_arrive_radius=0.30)

    def __init__(self, **kwargs):
        if set(kwargs) - set(self.DEFAULTS):
            raise ValueError('Unknown search parameter')
        for key, value in self.DEFAULTS.items():
            value = float(kwargs.get(key, value))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(key + ' must be positive and finite')
            setattr(self, key, value)


def wrap(angle):
    return (angle+math.pi) % (2*math.pi)-math.pi


# Navigator reasons meaning the current goal will not be reached. START_BLOCKED is not
# here: it is about the robot's own position (handled by ESCAPE), not the goal, and
# counting it burned through every candidate while the robot sat in a buffer.
GOAL_FAILURES = ('NO_REACHABLE_APPROACH', 'GOAL_BLOCKED_OR_UNKNOWN', 'STUCK',
                 'PATH_BLOCKED', 'INVALID_GOAL')
# Only real motion toward the goal clears a failure. The async planner interleaves
# PLANNING (and recovery spins) between failures; letting those reset the timer kept
# an unreachable goal alive until STUCK (~15 s of standing still).
GOAL_PROGRESS = ('FOLLOW', 'FOLLOW_PARTIAL', 'ROTATE', 'ESCAPE')


class SearchMission(object):
    def __init__(self, nav, cfg=None):
        self.nav, self.cfg = nav, cfg or SearchConfig()
        self.covered = set()     # odom cells the camera has looked at (not expiring)
        self.found = None
        self.state = 'SPIN'
        self.spin_yaws, self.spin_since, self.dwell_since = None, None, None
        self.goal, self.goal_kind, self.goal_since, self.fail_since = None, '', None, None
        self.best_distance, self.best_since = float('inf'), None
        self.blacklist, self.spun_at = [], []
        self.last_target, self.lost_pending = None, False
        # Seen but not reachable from here: ignore it briefly and explore near it.
        self.approach_fail_since, self.ignore_target_until = None, float('-inf')
        self.focus, self.focus_until = None, float('-inf')
        self.nav_reason = ''

    def info(self):
        return dict(state=self.state, found=self.found, goal=None if self.goal is None else [float(v) for v in self.goal],
                    goal_kind=self.goal_kind, covered_cells=len(self.covered),
                    covered_m2=round(len(self.covered)*self.nav.cfg.resolution**2, 2), spots=len(self.spun_at),
                    nav_reason=self.nav_reason)

    def update_coverage(self, ranges, angle_min, angle_increment, range_min, pose):
        """Mark cells inside the camera's horizontal view, cut at the first LiDAR hit."""
        cfg, cc, res = self.cfg, self.nav.cc, self.nav.cfg.resolution
        values = np.asarray(ranges, dtype=float)
        bearings = (angle_min+np.arange(len(values))*angle_increment+cc.scan_yaw+math.pi) % (2*math.pi)-math.pi
        view = np.abs(bearings) <= math.radians(cfg.camera_fov_deg)/2
        if not view.any():
            return
        reach = values[view]
        reach = np.where(np.isposinf(reach), cfg.camera_range,
                         np.where(np.isfinite(reach) & (reach >= range_min), reach, 0.))
        reach = np.minimum(reach, cfg.camera_range)
        origin = to_world([[cc.scan_x, cc.scan_y]], pose)[0]
        steps = np.arange(0., cfg.camera_range+res/2, res/2)
        angles = bearings[view]+pose[2]
        mask = steps[None, :] <= reach[:, None]
        xs = origin[0]+np.cos(angles)[:, None]*steps[None, :]
        ys = origin[1]+np.sin(angles)[:, None]*steps[None, :]
        cells = np.floor(np.column_stack((xs[mask], ys[mask]))/res).astype(np.int64)
        if len(cells):
            self.covered.update(map(tuple, np.unique(cells, axis=0).tolist()))

    def _coverage_mask(self, grid):
        mask = np.zeros(grid.raw.shape, dtype=bool)
        if self.covered:
            cells = np.array(list(self.covered), dtype=np.int64)
            rows, cols = cells[:, 1]-int(grid.origin[1]), cells[:, 0]-int(grid.origin[0])
            inside = (rows >= 0) & (rows < mask.shape[0]) & (cols >= 0) & (cols < mask.shape[1])
            mask[rows[inside], cols[inside]] = True
        return mask

    def choose_goal(self, pose, grid, now=None):
        """Standable observed cell with the most unseen space (unknown or not yet looked at
        by the camera) within camera range, discounted by distance."""
        cfg, res = self.cfg, grid.resolution
        interest = (grid.raw < 0) | ((grid.raw == 0) & ~self._coverage_mask(grid))
        # Integral image: interest count in a square of camera range around each cell.
        r = int(cfg.camera_range/res)
        padded = np.pad(interest.astype(np.int32), r+1)
        integral = padded.cumsum(0).cumsum(1)
        n, m = interest.shape
        rows, cols = np.mgrid[0:n, 0:m]
        a, b = rows+1, cols+1
        gain = (integral[a+2*r, b+2*r]-integral[a-1, b+2*r]-integral[a+2*r, b-1]+integral[a-1, b-1]).astype(float)
        # One cell inside the standable area: a goal on its edge is often beside an
        # obstacle face not seen yet, and becomes unreachable once that face appears.
        inner = grid.allowed & ~dilate(~grid.allowed, 1)
        candidates = inner & dilate(interest, 2)
        position = np.asarray(pose[:2], dtype=float)
        best, best_score = None, 0.
        for row, col in np.argwhere(candidates):
            if gain[row, col] < cfg.min_gain_cells:
                continue
            point = grid.point((row, col))
            distance = float(np.linalg.norm(point-position))
            if distance < cfg.min_goal_distance:
                continue
            if any(np.linalg.norm(point-p) < cfg.blacklist_radius for p in self.blacklist):
                continue
            if any(np.linalg.norm(point-p) < cfg.revisit_radius for p in self.spun_at):
                continue
            score = gain[row, col]/(distance+.5)
            if self.focus is not None and now is not None and now < self.focus_until:
                # Look at an unreachable sighting from another side.
                score /= 1.+2.*float(np.linalg.norm(point-self.focus))
            if score > best_score:
                best, best_score = point, score
        return best

    def _start_spin(self, pose):
        steps = int(round(360./self.cfg.spin_step_deg))
        self.spin_yaws = [pose[2]+math.radians(self.cfg.spin_step_deg)*k for k in range(steps)]
        self.state, self.spin_since, self.dwell_since = 'SPIN', None, None
        self.nav.reset()

    def _goto(self, goal, kind, now):
        self.goal, self.goal_kind, self.goal_since, self.fail_since = np.asarray(goal, dtype=float), kind, now, None
        self.best_distance, self.best_since = float('inf'), now
        self.state = 'GOTO'
        self.nav.reset()

    def _next_goal(self, pose, now):
        goal = self.choose_goal(pose, self.nav.grid, now)
        if goal is None:
            self.state, self.goal = 'COMPLETE', None
            return (0., 0., 0.), 'SEARCH_COMPLETE'
        self._goto(goal, 'EXPLORE', now)
        return (0., 0., 0.), 'SEARCH_EXPLORE'

    def _spin(self, pose, now):
        cfg, cc = self.cfg, self.nav.cc
        if self.spin_yaws is None:
            self._start_spin(pose)
        while self.spin_yaws:
            if self.spin_since is None:
                self.spin_since = now
            error = wrap(self.spin_yaws[0]-pose[2])
            if abs(error) > math.radians(5.) and now-self.spin_since <= 8.:
                speed = min(cc.max_angular, max(.15, self.nav.cfg.heading_gain*abs(error)))
                turn = math.copysign(speed, error)
                if self.nav.grid.pose_free(pose[:2], pose[2]+turn*.5):
                    self.dwell_since = None
                    return (0., 0., turn), 'SEARCH_SPIN'
            elif self.dwell_since is None or now-self.dwell_since < cfg.spin_dwell_s:
                # Hold still so YOLO can confirm a target in this view.
                self.dwell_since = self.dwell_since if self.dwell_since is not None else now
                return (0., 0., 0.), 'SEARCH_SPIN'
            # Reached and dwelt, timed out, or turning would swing into the buffer.
            self.spin_yaws.pop(0)
            self.spin_since = self.dwell_since = None
        self.spun_at.append(np.asarray(pose[:2], dtype=float).copy())
        self.spin_yaws = None
        return self._next_goal(pose, now)

    def step(self, pose, target, standoff, now, dt):
        """One control cycle. target: odom (x, y) of a confirmed fresh detection, or None."""
        zero = (0., 0., 0.)
        nav = self.nav
        if self.found is not None:
            self.state = 'FOUND'
            return zero, 'FOUND'
        position = np.asarray(pose[:2], dtype=float)
        if target is not None and now < self.ignore_target_until:
            target = None
        if target is not None:
            if self.state != 'APPROACH':
                nav.reset()
                self.approach_fail_since = None
            self.state, self.last_target, self.lost_pending = 'APPROACH', np.asarray(target, dtype=float), True
            velocity, self.nav_reason = nav.step(pose, target, standoff, now, dt)
            if self.nav_reason == 'ARRIVED':
                self.found = [float(target[0]), float(target[1])]
                self.state = 'FOUND'
                return zero, 'FOUND'
            if self.nav_reason in GOAL_PROGRESS:
                self.approach_fail_since = None
            elif self.nav_reason in GOAL_FAILURES and self.approach_fail_since is None:
                self.approach_fail_since = now
            if self.nav_reason == 'STUCK' or (self.approach_fail_since is not None
                                              and now-self.approach_fail_since >= self.cfg.fail_hold_s):
                hold = 15.
                self.ignore_target_until = self.focus_until = now+hold
                self.focus, self.lost_pending = self.last_target.copy(), False
                nav.grid = nav.costmap.snapshot(pose, now)
                return self._next_goal(pose, now)
            return velocity, self.nav_reason
        if self.lost_pending:
            # Lost during approach: look from just before the last sighting first.
            self.lost_pending = False
            offset = self.last_target-position
            distance = float(np.linalg.norm(offset))
            if distance > self.cfg.lost_standoff+.1:
                self._goto(self.last_target-offset/distance*self.cfg.lost_standoff, 'LAST_SEEN', now)
            else:
                self._start_spin(pose)
        if self.state == 'COMPLETE':
            return zero, 'SEARCH_COMPLETE'
        if self.state in ('SPIN', 'APPROACH'):
            nav.grid = nav.costmap.snapshot(pose, now)
            self.nav_reason = ''
            return self._spin(pose, now)
        distance = float(np.linalg.norm(self.goal-position))
        if self.goal_kind == 'EXPLORE' and distance <= self.cfg.explore_arrive_radius:
            # Close enough to look around; the last centimetres beside furniture are
            # where the planner dithers between ROTATE, recovery and FOLLOW.
            self._start_spin(pose)
            return zero, 'SEARCH_SPIN'
        velocity, self.nav_reason = nav.step(pose, self.goal, 0., now, dt, exact_goal=True)
        if nav.path:
            # Remaining route length, not straight-line distance: a detour around
            # furniture first moves away from the goal.
            route = np.vstack((position, np.asarray(nav.path, dtype=float)))
            remaining = float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum())
            if remaining < self.best_distance-self.cfg.stall_distance:
                self.best_distance, self.best_since = remaining, now
        if self.nav_reason == 'ARRIVED':
            self._start_spin(pose)
            return zero, 'SEARCH_SPIN'
        if self.nav_reason in GOAL_PROGRESS:
            self.fail_since = None
        elif self.nav_reason in GOAL_FAILURES and self.fail_since is None:
            self.fail_since = now
        # Turning and creeping keep the navigator from STUCK, but a route that has not
        # become shorter for stall_s will not be finished from here.
        if (self.nav_reason == 'STUCK' or now-self.goal_since > self.cfg.goal_timeout_s
                or now-self.best_since > self.cfg.stall_s
                or (self.fail_since is not None and now-self.fail_since >= self.cfg.fail_hold_s)):
            self.blacklist.append(self.goal.copy())
            nav.grid = nav.costmap.snapshot(pose, now)
            return self._next_goal(pose, now)
        return velocity, 'SEARCH_EXPLORE' if self.goal_kind == 'EXPLORE' else 'SEARCH_LAST_SEEN'
