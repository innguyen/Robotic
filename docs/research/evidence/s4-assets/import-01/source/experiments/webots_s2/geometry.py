"""Independent assertion geometry, never used to select policy targets/commands."""
import math


def footprint_separation(position, yaw, box):
    """SAT gap for a .4m square robot vs axis-aligned box; negative means overlap.

    Bounding the rotated robot with a world AABB gives false corner collisions.
    Test all four rectangle axes instead. Positive gap is a lower bound on clearance.
    """
    bx, by, sx, sy = box[:4]
    c, s = math.cos(yaw), math.sin(yaw)
    delta = (position[0]-bx, position[1]-by)
    gaps = []
    for ax, ay in ((1., 0.), (0., 1.), (c, s), (-s, c)):
        robot_radius = .2*(abs(ax*c+ay*s)+abs(-ax*s+ay*c))
        box_radius = abs(ax)*sx/2 + abs(ay)*sy/2
        gaps.append(abs(ax*delta[0]+ay*delta[1])-robot_radius-box_radius)
    return max(gaps)
