"""Explicit ideal proxy / Mecanum kinematic extension, without traction claims."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MecanumGeometry:
    wheel_radius_m: float = .05
    half_length_m: float = .13
    half_width_m: float = .17
    max_wheel_rad_s: float = 12.

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (self.wheel_radius_m,self.half_length_m,self.half_width_m,self.max_wheel_rad_s)):
            raise ValueError("invalid Mecanum geometry")

    def wheels(self, twist):
        x,y,w = twist; a = self.half_length_m+self.half_width_m; r = self.wheel_radius_m
        # FL, FR, RL, RR with X roller arrangement, base FLU.
        return [(x-y-a*w)/r,(x+y+a*w)/r,(x+y-a*w)/r,(x-y+a*w)/r]

    def body(self, wheels):
        fl,fr,rl,rr = wheels; r = self.wheel_radius_m; a = self.half_length_m+self.half_width_m
        return [r*(fl+fr+rl+rr)/4,r*(-fl+fr+rl-rr)/4,r*(-fl+fr-rl+rr)/(4*a)]

    def limit(self, twist):
        wheels = self.wheels(twist)
        scale = min(1.,self.max_wheel_rad_s/max(max(abs(v) for v in wheels),1e-12))
        return self.body([v*scale for v in wheels])
