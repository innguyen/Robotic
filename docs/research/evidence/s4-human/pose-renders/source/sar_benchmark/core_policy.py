"""S2 core algorithm reuse through public measurements, with no oracle inputs."""
import math
from pathlib import Path
import sys

from .policies import SearchPolicy, detections


class CoreSearchPolicy(SearchPolicy):
    def __init__(self, seed=0, detector=None):
        super().__init__("sweep", seed, detector)
        core = Path(__file__).resolve().parents[1] / "test version"
        sys.path.insert(0, str(core))
        from collision_monitor_ros1 import Config
        from navigation_core_ros1 import NavConfig, Navigator
        from search_core_ros1 import SearchConfig, SearchMission
        self.SearchMission, self.search_config = SearchMission, SearchConfig(camera_fov_deg=90., camera_range=8.)
        self.nav = Navigator(NavConfig(control_hz=10), Config(body_length=.4, body_width=.4,
                             scan_x=0., scan_y=0., scan_yaw=0., blind_sectors_deg=[]), async_planning=False)
        self.mission = SearchMission(self.nav, self.search_config)
        if detector is not None:
            from .tracking import ReportTracker
            self.human_reporter = ReportTracker()

    def act(self, obs):
        import numpy as np
        action = super().act(obs)
        if self.detector is not None:
            action["reports"] = self.human_reporter.update(obs,self.current_detections)
            self.reported = self.human_reporter.reported
        now = obs["sim_time_ns"] / 1e9
        pose, scan = obs["odom"]["pose"], obs["scan"]
        if not obs["odom"]["valid"] or not scan["valid"] or any(status in ("invalid", "dropped") for status in scan["status"]):
            action["twist"] = [0., 0., 0.]
            return action
        values = np.array([value if value is not None else math.inf for value in scan["ranges_m"]])
        self.nav.costmap.update(values, scan["angle_min"], scan["angle_increment"], .05, 8., pose, now)
        self.mission.update_coverage(values, scan["angle_min"], scan["angle_increment"], .05, pose)
        self.nav.grid = self.nav.costmap.snapshot(pose, now)
        if action["reports"]:
            self.mission = self.SearchMission(self.nav, self.search_config)
        candidates = [d["position"] for d in self.current_detections
                      if not any(math.dist(d["position"][:2], p[:2]) < .8 for p in self.reported)]
        target = min(candidates, key=lambda p: math.dist(p[:2], pose[:2]))[:2] if candidates else None
        twist, _ = self.mission.step(pose, target, .55, now, .1)
        action["twist"] = [float(v) for v in twist]
        return action

    def close(self):
        self.nav.close()
