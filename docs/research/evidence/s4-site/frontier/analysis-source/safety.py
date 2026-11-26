"""Common actuator/freshness gate. All policies use the same checked command."""
import math

from .contracts import DT_NS, finite


def directional_clearance(twist, scan, dt):
    """Conservative 2D circle sweep over the same .01 s proxy increments.

    A nearby obstacle must not prevent a zero-yaw retreat that strictly improves
    clearance. This is measured-ray protection, not a 3D collision certificate.
    """
    vx,vy,wz = twist
    if max(abs(value) for value in twist) < 1e-12: return True
    speed = math.hypot(vx,vy)
    physical_radius = math.sqrt(2)*.2
    radius = physical_radius+.08+speed*.005
    positions = []
    x=y=yaw=0.
    for _ in range(round(dt/.01)):
        x += (math.cos(yaw)*vx-math.sin(yaw)*vy)*.01
        y += (math.sin(yaw)*vx+math.cos(yaw)*vy)*.01
        yaw += wz*.01
        positions.append((x,y))
    for index,distance in enumerate(scan["ranges_m"]):
        if distance is None: continue
        angle = scan["angle_min"]+index*scan["angle_increment"]
        px,py = distance*math.cos(angle),distance*math.sin(angle)
        if distance < radius:
            if distance <= physical_radius+.001 or abs(wz)>1e-12 or speed<1e-12: return False
            previous = distance
            for x,y in positions:
                actual = math.hypot(px-x,py-y)
                if actual < previous-1e-9: return False
                previous = actual
            if previous <= distance+1e-6: return False
        elif any(math.hypot(px-x,py-y) <= radius for x,y in positions):
            return False
    return True


class SafetyGate:
    def __init__(self):
        self.previous = [0.0, 0.0, 0.0]
        self.last_limited = [0.0, 0.0, 0.0]

    def check(self, requested, obs):
        if len(requested) != 3 or not all(finite(x) for x in requested):
            self.previous = [0.0] * 3
            return [0.0] * 3, "INVALID_COMMAND"
        now = obs["sim_time_ns"]
        dt = obs.get("action_dt_ns",DT_NS)/1e9
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
        scale = min(1.0, 0.5 * dt / length) if length else 1.0
        applied = [self.previous[i] + delta[i] * scale for i in range(2)]
        applied.append(self.previous[2] + max(-dt, min(dt, limited[2] - self.previous[2])))
        if obs["profile"] == "ue5_sar_human_rgbd_scan_v1":
            from .embodiment import MecanumGeometry
            applied = MecanumGeometry().limit(applied)
        scan = obs["scan"]
        if any(status in ("invalid", "dropped") for status in scan["status"]):
            self.previous = [0.0] * 3
            return [0.0] * 3, "INVALID_RAYS"
        # Circumscribed .4m square plus movement over command TTL and margin.
        speed = math.hypot(*applied[:2])
        stop_radius = math.sqrt(2) * .2 + speed * dt + .08
        clear = directional_clearance(applied,scan,dt) if obs["profile"]=="ue5_sar_human_rgbd_scan_v1" else not any(value is not None and value <= stop_radius for value in scan["ranges_m"])
        if not clear:
            self.previous = [0.0] * 3
            return [0.0] * 3, "CLEARANCE_STOP"
        self.previous = applied
        return applied, "OK"
