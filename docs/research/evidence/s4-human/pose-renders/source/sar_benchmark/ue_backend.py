"""Single UE5 native editor integration, with private sensor response/GT lanes."""
import copy
import hashlib
import math
import random

from . import PROTOCOL
from .contracts import DT_NS, canonical, require
from .scenario_contract import validate_scenario


class UnrealBackend:
    def __init__(self, client, raw_sink=None):
        self.client, self.raw_sink = client, raw_sink
        self.config = None
        self.last_world_pose = None
        self.estimated_pose = [0.0, 0.0, 0.0]
        self.host_info = self.client.call("hello")
        require(self.host_info["rendered"] and self.host_info["clock_profile"] == "static_kinematic_transactions_not_Chaos_lockstep", "unsupported host")

    def reset(self, scenario, budget_ticks):
        validate_scenario(scenario)
        self.config = copy.deepcopy(scenario)
        self.budget_ticks = budget_ticks
        self.last_world_pose = None
        self.estimated_pose = [0.0, 0.0, 0.0]
        raw = self.client.call("reset", scenario)
        require(raw["tick"] == 0 and not raw["events"], "reset integrity failure")
        return self._convert(raw, [0.0] * 3)

    def advance(self, action):
        return self._convert(self.client.call("step", {"twist": action}), action)

    def point_to_episode(self, p):
        start, yaw = self.config["start_m"], self.config["start_yaw"]
        dx, dy = p[0] - start[0], p[1] - start[1]
        return [math.cos(yaw) * dx + math.sin(yaw) * dy,
                -math.sin(yaw) * dx + math.cos(yaw) * dy, p[2]]

    def _convert(self, raw, applied):
        if self.raw_sink:
            self.raw_sink(raw)
        if "rgb_png" in raw:
            from .capture import decode_human_capture
            raw = decode_human_capture(raw)
        tick, epoch = raw["tick"], raw["epoch"]
        now = tick * DT_NS
        pose = raw["pose"]
        ground = self.point_to_episode(pose[:2] + [0])[:2] + [(pose[2] - self.config["start_yaw"] + math.pi) % math.tau - math.pi]
        if self.last_world_pose is not None:
            previous = self.last_world_pose
            dx, dy, da = [ground[i] - previous[i] for i in range(3)]
            da = (da + math.pi) % math.tau - math.pi
            severity = self.config["odometry"]
            # Synthetic measurement drift from self-motion only. No victim data
            # participates. This is not a wheel/encoder simulator or real IMU.
            distance = math.hypot(dx, dy)
            if self.config.get("profile"):
                # Synthetic body-frame encoder increments integrated using the
                # estimated heading, so heading error also drifts XY over travel.
                pc,ps = math.cos(previous[2]),math.sin(previous[2])
                forward,left = (pc*dx+ps*dy)*(1+.03*severity),(-ps*dx+pc*dy)*(1+.03*severity)
                ec,es = math.cos(self.estimated_pose[2]),math.sin(self.estimated_pose[2])
                self.estimated_pose[0] += ec*forward-es*left
                self.estimated_pose[1] += es*forward+ec*left
            else:
                self.estimated_pose[0] += dx * (1 + .03 * severity)
                self.estimated_pose[1] += dy * (1 + .03 * severity)
            self.estimated_pose[2] += da * (1 + .10 * severity) + severity * (math.radians(2) * distance + .001)
        self.last_world_pose = ground
        rgb, depth = list(raw["rgb"]), list(raw["depth_m"])
        severity = self.config["perception"]
        seed = int.from_bytes(hashlib.sha256(canonical([self.config["seed"], "perception", tick])).digest()[:8], "big")
        rng = random.Random(seed)
        if severity:
            rgb = [round(value * (.75 if self.config.get("profile") else .28)) for value in rgb]
            if not self.config.get("profile"):
                depth = [None if value is None or rng.random() < .12 else min(10.0, max(.01, value + .35 + rng.gauss(0, .06))) for value in depth]
        if self.config.get("profile"):
            from .sensor_response import depth_response
            depth = depth_response(depth,raw["camera"]["width"],self.config.get("depth_model","structured"),self.config["seed"],tick,1+severity)
        if self.config["sensor_black"]:
            rgb, depth = [0] * len(rgb), [None] * len(depth)
        def packet(name, data):
            return {"epoch": epoch, "sequence": tick, "capture_ns": now, "available_ns": now,
                    "valid": True, "frame_id": name, **data}
        obs = {"protocol": PROTOCOL, "episode_id": "episode", "epoch": epoch, "tick": tick,
               "sim_time_ns": now, "sequence": tick, "budget_ns": self.budget_ticks * DT_NS,
               "profile": "ue5_sar_human_rgbd_scan_v1" if self.config.get("profile") else "ue5_static_proxy_rgbd_scan_v1", "camera": raw["camera"],
               "rgb": packet("rgb_optical", {"data": rgb}),
               "depth_m": packet("rgb_optical", {"data": depth}),
               "scan": packet("base_flu", {"ranges_m": raw["scan_m"],
                       "status": ["no_return" if value is None else "hit" for value in raw["scan_m"]],
                       "angle_min": -math.pi, "angle_increment": math.tau / len(raw["scan_m"]), "range_max_m": 8.0}),
               "odom": packet("episode", {"pose": list(self.estimated_pose)}),
               "limits": {"vx": [-.1, .3], "vy": [-.2, .2], "wz": [-.6, .6]}}
        truth = {"victims": [{"gt_id": item["gt_id"], "anchor_m": self.point_to_episode(item["anchor_m"])} for item in raw["victims"]],
                 "path_m": raw["path_m"], "safety_events": raw["events"], "applied": applied,
                 "frame": {"epoch": epoch, "capture_ns": now, "available_ns": now,
                           "width": raw["camera"]["width"], "height": raw["camera"]["height"],
                           "valid": True, "visible": raw["visible"],
                           "delivered_rgb_sha256": hashlib.sha256(canonical(rgb)).hexdigest()}}
        return obs, truth

    def stop(self):
        return self.client.call("stop")

    def close(self):
        self.stop()
