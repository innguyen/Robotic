"""Official T1 exploration selectors. Inputs contain observed state only.

Mission, approach, planner and reporting remain in SearchMission/the shared adapter.
The grid is Navigator's sensor costmap, never an evaluator geometry map.
"""
from dataclasses import dataclass
import math

import numpy as np
from navigation_core_ros1 import dilate

B0 = 'b0-fixed-order-v2'
B1 = 'b1-gain-distance-focus-v1'
OFFICIAL_POLICIES = (B0, B1)
DOMAIN_VERSION = 'observed-standable-v1'


@dataclass(frozen=True, slots=True)
class SelectionState:
    pose: tuple
    now: float | None
    covered: frozenset
    blacklist: tuple
    spun_at: tuple
    focus: object
    focus_until: float


def coverage_mask(grid, covered):
    mask = np.zeros(grid.raw.shape, dtype=bool)
    if covered:
        cells = np.array(list(covered), dtype=np.int64)
        rows, cols = cells[:, 1]-int(grid.origin[1]), cells[:, 0]-int(grid.origin[0])
        inside = (rows >= 0) & (rows < mask.shape[0]) & (cols >= 0) & (cols < mask.shape[1])
        mask[rows[inside], cols[inside]] = True
    return mask


class CandidateDomain:
    """Common §5.2 mask, then exact-point filters (also for off-centre B0 points)."""
    def __init__(self, grid, state, cfg, bounds, body):
        self.grid, self.state, self.cfg, self.bounds = grid, state, cfg, bounds
        c, s = abs(math.cos(state.pose[2])), abs(math.sin(state.pose[2]))
        self.hx, self.hy = (c*body[0]+s*body[1])/2, (s*body[0]+c*body[1])/2
        rows, cols = np.indices(grid.raw.shape)
        x = (cols+grid.origin[0]+.5)*grid.resolution
        y = (rows+grid.origin[1]+.5)*grid.resolution
        a, b, aa, bb = bounds
        allowed = grid.allowed & (grid.raw == 0) & (x-self.hx >= a) & (x+self.hx <= aa) & (y-self.hy >= b) & (y+self.hy <= bb)
        self.interest = (grid.raw < 0) | ((grid.raw == 0) & ~coverage_mask(grid, state.covered))
        self.mask = allowed & ~dilate(~allowed, 1) & dilate(self.interest, 2)

    def rejection(self, point):
        g, state, cfg = self.grid, self.state, self.cfg
        if not g.inside(point):return 'outside_costmap'
        cell = g.cell(point)
        if g.raw[cell] < 0:return 'unknown'
        if g.raw[cell] != 0:return 'occupied'
        if not self.mask[cell]:return 'outside_candidate_domain'
        a, b, aa, bb = self.bounds
        if not (a <= point[0]-self.hx and point[0]+self.hx <= aa and b <= point[1]-self.hy and point[1]+self.hy <= bb):return 'bounds'
        if not g.pose_free(point, state.pose[2]):return 'footprint'
        point = np.asarray(point, dtype=float)
        if float(np.linalg.norm(point-np.asarray(state.pose[:2]))) < cfg.min_goal_distance:return 'min_distance'
        if any(np.linalg.norm(point-p) < cfg.blacklist_radius for p in state.blacklist):return 'blacklist'
        if any(np.linalg.norm(point-p) < cfg.revisit_radius for p in state.spun_at):return 'revisit'
        return None

    def points(self):
        for row, col in np.argwhere(self.mask):
            point = self.grid.point((row, col))
            if self.rejection(point) is None:yield (int(row), int(col)), point


@dataclass(frozen=True, slots=True)
class Selection:
    point: tuple
    kind: str
    waypoint_index: object = None


class GoalSelector:
    """select(domain) -> Selection | None; finish() records mission outcomes."""
    def __init__(self):
        self.pending = []
        self.active = None
        self.goals_selected = self.fallback_goals = 0

    def selected(self, selection, skipped=()):
        self.active = selection
        self.goals_selected += 1
        self.fallback_goals += selection.kind == 'fallback'
        self.pending.append(('SELECTOR_DECISION', dict(selection=dict(point=selection.point, kind=selection.kind,
            waypoint_index=selection.waypoint_index), skipped=list(skipped))))
        return selection

    def finish(self, outcome, reason):
        if self.active is not None:
            self.pending.append(('WAYPOINT_OUTCOME', dict(point=self.active.point, kind=self.active.kind,
                waypoint_index=self.active.waypoint_index, outcome=outcome, reason=reason)))
            self.active = None

    def drain(self):
        events, self.pending = self.pending, []
        return events

    def stats(self):
        return dict(goals_selected=self.goals_selected, fallback_goals=self.fallback_goals, unresolved_waypoints=None)


class FixedOrderSelector(GoalSelector):
    def __init__(self, start, bounds, spacing):
        super().__init__()
        self.start, self.spacing = tuple(start), spacing
        self.c, self.s = math.cos(start[2]), math.sin(start[2])
        a, b, aa, bb = bounds
        corners = [self.local((x, y)) for x in (a, aa) for y in (b, bb)]
        lo = [math.floor(min(p[i] for p in corners))-1 for i in (0, 1)]
        hi = [math.ceil(max(p[i] for p in corners))+1 for i in (0, 1)]
        indices = [(kx, ky) for kx in range(lo[0], hi[0]+1) for ky in range(lo[1], hi[1]+1) if (kx, ky) != (0, 0)]
        indices.sort(key=lambda p: (max(abs(p[0]), abs(p[1])), math.atan2(p[1], p[0]) % (2*math.pi), *p))
        self.waypoints = tuple((kx, ky, (start[0]+spacing*(self.c*kx-self.s*ky), start[1]+spacing*(self.s*kx+self.c*ky)))
            for kx, ky in indices if a <= start[0]+spacing*(self.c*kx-self.s*ky) <= aa and b <= start[1]+spacing*(self.s*kx+self.c*ky) <= bb)
        self.assigned, self.outcomes = set(), {}

    def local(self, point):
        x, y = point[0]-self.start[0], point[1]-self.start[1]
        return (self.c*x+self.s*y)/self.spacing, (-self.s*x+self.c*y)/self.spacing

    def select(self, domain):
        skipped = []
        for index, (_, _, point) in enumerate(self.waypoints):
            reason = 'already_assigned' if index in self.assigned else domain.rejection(point)
            if reason is not None:
                skipped.append(dict(index=index, reason=reason))
                continue
            self.assigned.add(index)
            return self.selected(Selection(point, 'fixed', index), skipped)
        candidates = list(domain.points())
        def key(item):
            cell, point = item
            u, v = self.local(point)
            row, col = cell
            return max(abs(u), abs(v)), math.atan2(v, u) % (2*math.pi), int(col+domain.grid.origin[0]), int(row+domain.grid.origin[1])
        if candidates:
            _, point = min(candidates, key=key)
            return self.selected(Selection(tuple(float(v) for v in point), 'fallback'), skipped)
        self.pending.append(('SELECTOR_DECISION', dict(selection=None, skipped=skipped)))
        return None

    def finish(self, outcome, reason):
        if self.active is not None and self.active.waypoint_index is not None:
            self.outcomes[self.active.waypoint_index] = outcome
        super().finish(outcome, reason)

    def stats(self):
        return dict(super().stats(), unresolved_waypoints=len(self.waypoints)-sum(v == 'reached' for v in self.outcomes.values()))


class GainDistanceFocusSelector(GoalSelector):
    def select(self, domain):
        # Same integral image, operation order, threshold, score and strict tie
        # rule as SearchMission.choose_goal(). Only the shared §5.2 domain differs.
        cfg, grid, state = domain.cfg, domain.grid, domain.state
        r = int(cfg.camera_range/grid.resolution)
        padded = np.pad(domain.interest.astype(np.int32), r+1)
        integral = padded.cumsum(0).cumsum(1)
        n, m = domain.interest.shape
        rows, cols = np.mgrid[0:n, 0:m]
        a, b = rows+1, cols+1
        gain = (integral[a+2*r, b+2*r]-integral[a-1, b+2*r]-integral[a+2*r, b-1]+integral[a-1, b-1]).astype(float)
        best, best_score = None, 0.
        for (row, col), point in domain.points():
            if gain[row, col] < cfg.min_gain_cells:continue
            distance = float(np.linalg.norm(point-np.asarray(state.pose[:2], dtype=float)))
            score = gain[row, col]/(distance+.5)
            if state.focus is not None and state.now is not None and state.now < state.focus_until:
                score /= 1.+2.*float(np.linalg.norm(point-np.asarray(state.focus)))
            if score > best_score:best, best_score = point, score
        if best is None:
            self.pending.append(('SELECTOR_DECISION', dict(selection=None, skipped=[])))
            return None
        return self.selected(Selection(tuple(float(v) for v in best), 'gain'))


def selection_diagnostics(records, elapsed, unknown=False):
    """Reconstruct selector counters/stalls from public events, including recovery."""
    latest = None; first = None; start = None; stalled = 0.
    for record in records:
        if record['event_type'] != 'SELECTOR_STATUS':continue
        latest = record['payload']; now = record['sim_time_s']
        if latest['stalled'] and start is None:
            start = now
            if first is None:first = now
        elif not latest['stalled'] and start is not None:
            stalled += now-start; start = None
    if latest is None:
        return dict.fromkeys(('goals_selected','fallback_goals','unresolved_waypoints','first_stalled_s','stalled_sim_s')) if unknown else {}
    if start is not None:stalled += max(0., elapsed-start)
    return {**{k:latest[k] for k in ('goals_selected','fallback_goals','unresolved_waypoints')},
            'first_stalled_s':first, 'stalled_sim_s':stalled}
