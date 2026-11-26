"""Canonical metres, radians, episode FLU and optical coordinates."""
import math


def ue_to_canonical(position_cm):
    return [position_cm[0] / 100, -position_cm[1] / 100, position_cm[2] / 100]


def canonical_to_ue(position_m):
    return [100 * position_m[0], -100 * position_m[1], 100 * position_m[2]]


def ue_rotation_to_canonical(rotation):
    signs = [1, -1, 1]
    return [[signs[i] * rotation[i][j] * signs[j] for j in range(3)] for i in range(3)]


def localize(u, v, depth_z, camera, estimated_pose):
    optical = [(u - camera["cx"]) * depth_z / camera["fx"],
               (v - camera["cy"]) * depth_z / camera["fy"], depth_z, 1]
    transform = camera["T_base_optical"]
    base = [sum(row[i] * optical[i] for i in range(4)) for row in transform[:3]]
    x, y, yaw = estimated_pose
    c, s = math.cos(yaw), math.sin(yaw)
    return [x + c * base[0] - s * base[1], y + s * base[0] + c * base[1], base[2]]


def iou(a, b):
    intersection = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersection
    return intersection / union if union > 0 else 0.0
