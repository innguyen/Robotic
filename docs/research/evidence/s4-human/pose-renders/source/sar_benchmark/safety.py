"""Common actuator/freshness gate. All policies use the same checked command."""
import math

from .contracts import DT_NS, finite


class SafetyGate:
    def __init__(self):
        self.previous = [0.0, 0.0, 0.0]
        self.last_limited = [0.0, 0.0, 0.0]

    def check(self, requested, obs):
        if len(requested) != 3 or not all(finite(x) for x in requested):
            self.previous = [0.0] * 3
            return [0.0] * 3, "INVALID_COMMAND"
        now = obs["sim_time_ns"]
        for name in ("scan", "odom"):
            packet = obs[name]
            if not packet["valid"] or now - packet["capture_ns"] > 300_000_000:
                self.previous = [0.0] * 3
                return [0.0] * 3, "STALE_OR_INVALID_" + name.upper()
        limited = [max(obs["limits"][k][0], min(obs["limits"][k][1], value))
                   for k, value in zip(("vx", "vy", "wz"), requested)]
        self.last_limited = list(limited)
        # Cap linear acceleration as a vector, rather than independently per axis.
        delta = [limited[i] - self.previous[i] for i in range(2)]
        length = math.hypot(*delta)
        scale = min(1.0, 0.5 * DT_NS / 1e9 / length) if length else 1.0
        applied = [self.previous[i] + delta[i] * scale for i in range(2)]
        applied.append(self.previous[2] + max(-.1, min(.1, limited[2] - self.previous[2])))
        if obs["profile"] == "ue5_sar_human_rgbd_scan_v1":
            from .embodiment import MecanumGeometry
            applied = MecanumGeometry().limit(applied)
        scan = obs["scan"]
        if any(status in ("invalid", "dropped") for status in scan["status"]):
            self.previous = [0.0] * 3
            return [0.0] * 3, "INVALID_RAYS"
        # Circumscribed .4m square plus movement over command TTL and margin.
        speed = math.hypot(*applied[:2])
        stop_radius = math.sqrt(2) * .2 + speed * .1 + .08
        if any(value is not None and value <= stop_radius for value in scan["ranges_m"]):
            self.previous = [0.0] * 3
            return [0.0] * 3, "CLEARANCE_STOP"
        self.previous = applied
        return applied, "OK"
