"""Evaluator-owned oracle; only frozen Observation values may cross to policy."""
from dataclasses import dataclass
import math
from .geometry import ray_hit


def visible(pose, target, obstacles, camera):
    x,y,yaw = pose
    c,s = math.cos(yaw),math.sin(yaw)
    origin = (x+c*camera['x_m']-s*camera['y_m'], y+s*camera['x_m']+c*camera['y_m'])
    dx,dy = target['x_m']-origin[0],target['y_m']-origin[1]
    angle = (math.atan2(dy,dx)-yaw-camera['yaw_rad']+math.pi)%(2*math.pi)-math.pi
    return (math.hypot(dx,dy) <= camera['range_m'] and abs(angle) <= math.radians(camera['fov_deg'])/2
            and not any(ray_hit(origin,(target['x_m'],target['y_m']),o['bounds_m']) for o in obstacles))


@dataclass(frozen=True, slots=True)
class Observation:
    observation_id: str
    label: str
    x_m: float
    y_m: float
    frame_id: str
    last_confirmed_seen_s: float
    visible: bool
    confirmed: bool


class Oracle:
    def __init__(self, camera, dt):
        self.camera, self.dt = dict(camera), dt
        self.last_step = -1
        self.count = self.sequence = 0
        self.last = None

    def step(self, step_index, pose, target, obstacles):
        if type(step_index) is not int or step_index != self.last_step+1:
            raise ValueError('oracle requires consecutive ticks starting at zero')
        if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
            raise ValueError('invalid pose')
        self.last_step = step_index
        now = step_index*self.dt
        raw = target is not None and visible(pose,target,obstacles,self.camera)
        self.count = self.count+1 if raw else 0
        if self.count >= self.camera['confirm_frames']:
            self.sequence += 1
            self.last = Observation('obs-%06d'%self.sequence,'person',target['x_m'],target['y_m'],
                                    'benchmark_map',now,True,True)
        if self.last is None or now-self.last.last_confirmed_seen_s > self.camera['memory_s']:
            return raw, None
        o = self.last
        return raw, Observation(o.observation_id,o.label,o.x_m,o.y_m,o.frame_id,o.last_confirmed_seen_s,raw,True)
