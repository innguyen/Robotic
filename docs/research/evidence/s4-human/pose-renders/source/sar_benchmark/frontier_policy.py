"""Classical free-space frontiers and an input-matched active verification arm."""
from collections import deque
import math

from .core_policy import CoreSearchPolicy
from .policies import SearchPolicy, detections
from .tracking import ReportTracker


class FrontierPolicy(CoreSearchPolicy):
    def __init__(self, seed=0, detector=None, verify=False):
        super().__init__(seed, detector)
        self.reporter = ReportTracker(verify)
        self.free, self.occupied, self.covered = set(), set(), set()
        self.goal = None
        self.goal_tick = 0
        self.verification_goal = None
        self.verify = verify

    @staticmethod
    def cell(x, y): return (math.floor(x/.3), math.floor(y/.3))

    def update_map(self, obs):
        pose, scan = obs["odom"]["pose"], obs["scan"]
        for index in range(0, len(scan["ranges_m"]), 4):
            distance = scan["ranges_m"][index]
            angle = scan["angle_min"]+index*scan["angle_increment"]+pose[2]
            extent = min(scan["range_max_m"], distance if distance is not None else scan["range_max_m"])
            for step in range(max(0, int((extent-.08)/.15))):
                self.free.add(self.cell(pose[0]+step*.15*math.cos(angle), pose[1]+step*.15*math.sin(angle)))
            if distance is not None:
                self.occupied.add(self.cell(pose[0]+distance*math.cos(angle), pose[1]+distance*math.sin(angle)))
        self.free.difference_update(self.occupied)
        self.covered.add(self.cell(*pose[:2]))

    def choose_frontier(self, pose):
        start = self.cell(*pose[:2])
        blocked = {(x+dx,y+dy) for x,y in self.occupied for dx in (-1,0,1) for dy in (-1,0,1)}
        reachable = {start: 0}; queue = deque([start]); candidates = []
        while queue and len(reachable) < 4000:
            x,y = queue.popleft(); distance = reachable[(x,y)]
            neighbors = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
            unknown = sum(n not in self.free and n not in self.occupied for n in neighbors)
            if unknown and distance >= 2:
                candidates.append((unknown/(1+.15*distance) - .15*((x,y) in self.covered), -(distance), x,y))
            for item in neighbors:
                if item in self.free and item not in blocked and item not in reachable:
                    reachable[item] = distance+1; queue.append(item)
        if not candidates: return None
        _,_,x,y = max(candidates)
        return [(x+.5)*.3,(y+.5)*.3]

    def act(self, obs):
        import numpy as np
        # Use the same detector/depth estimator for frontier and verification.
        found = detections(obs,self.detector)
        reports = self.reporter.update(obs,found)
        tick, pose, scan = obs["tick"], obs["odom"]["pose"], obs["scan"]
        action = {"epoch":obs["epoch"],"sequence":tick,"tick":tick,"reports":reports,
                  "twist":[0.,0.,0.],"finish":obs["sim_time_ns"] >= obs["budget_ns"]-100_000_000}
        if not obs["odom"]["valid"] or not scan["valid"] or obs["sim_time_ns"]-scan["capture_ns"] > 300_000_000:
            return action
        self.update_map(obs)
        now = obs["sim_time_ns"]/1e9
        values = np.array([v if v is not None else math.inf for v in scan["ranges_m"]])
        self.nav.costmap.update(values,scan["angle_min"],scan["angle_increment"],.05,8.,pose,now)
        self.nav.grid = self.nav.costmap.snapshot(pose,now)
        if self.verify and self.reporter.pending is not None:
            if self.verification_goal is None:
                dx,dy = self.reporter.pending[0]-pose[0],self.reporter.pending[1]-pose[1]
                length = max(.1,math.hypot(dx,dy))
                self.verification_goal = [pose[0]-.30*dy/length,pose[1]+.30*dx/length]
            twist,reason = self.nav.step(pose,self.verification_goal,0.,now,.1,exact_goal=True)
            action["twist"] = [float(v) for v in twist]
            if reason in ("ARRIVED","STUCK"): self.verification_goal = None
            return action
        self.verification_goal = None
        # One complete initial visual sweep; range scan alone is not RGB coverage.
        if tick < 130 or tick % 260 >= 220:
            action["twist"] = [0.,0.,.5]
            return action
        if self.goal is None or math.dist(pose[:2],self.goal) < .3 or tick-self.goal_tick > 100:
            self.goal,self.goal_tick = self.choose_frontier(pose),tick
            self.nav.reset()
        if self.goal is not None:
            twist,reason = self.nav.step(pose,self.goal,0.,now,.1,exact_goal=True)
            action["twist"] = [float(v) for v in twist]
            if reason == "STUCK": self.goal = None
        else:
            action["twist"] = [.08,0.,.3]
        return action
