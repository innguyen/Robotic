"""Strict JSON wire contracts; the agent never receives a backend/environment object."""
import copy
import json
import math
import re

from . import PROTOCOL

DT_NS = 100_000_000
MAX_REPORTS = 5
ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,79}$")


class ContractError(ValueError):
    pass


def require(test, message):
    if not test:
        raise ContractError(message)


def finite(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def integer(value, minimum=0):
    return type(value) is int and value >= minimum


def fields(value, required, optional=()):
    require(type(value) is dict, "expected object")
    require(set(required) <= value.keys() <= set(required) | set(optional), "unknown/missing fields")


def vector(value, size):
    return type(value) is list and len(value) == size and all(finite(x) for x in value)


def identifier(value):
    return type(value) is str and ID.fullmatch(value) is not None


def strict_loads(data):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, "duplicate JSON key")
            result[key] = value
        return result
    try:
        return json.loads(data, object_pairs_hook=unique,
                          parse_constant=lambda _: (_ for _ in ()).throw(ContractError("non-finite JSON")))
    except (ValueError, TypeError, RecursionError) as exc:
        raise ContractError(str(exc)) from exc


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def covariance_valid(matrix):
    if not (type(matrix) is list and len(matrix) == 3 and all(vector(row, 3) for row in matrix)):
        return False
    if any(abs(matrix[i][j] - matrix[j][i]) > 1e-10 for i in range(3) for j in range(3)):
        return False
    # Cholesky checks positive definiteness, including all off-diagonal terms.
    lower = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(i + 1):
            value = matrix[i][j] - sum(lower[i][k] * lower[j][k] for k in range(j))
            if not math.isfinite(value):
                return False
            if i == j:
                if value <= 0:
                    return False
                lower[i][j] = math.sqrt(value)
            else:
                lower[i][j] = value / lower[j][j]
    return True


def validate_report(report):
    fields(report, ("report_id", "epoch", "tick", "label", "position_episode_m", "confidence", "evidence"),
           ("position_covariance_m2",))
    require(identifier(report["report_id"]), "invalid report_id")
    require(integer(report["epoch"], 1) and integer(report["tick"]), "invalid report header")
    require(report["label"] == "victim_person", "unsupported label")
    require(vector(report["position_episode_m"], 3), "invalid position")
    require(finite(report["confidence"]) and 0 <= report["confidence"] <= 1, "invalid confidence")
    if "position_covariance_m2" in report:
        require(covariance_valid(report["position_covariance_m2"]), "invalid covariance")
    require(type(report["evidence"]) is list and 2 <= len(report["evidence"]) <= 8, "need 2..8 evidence frames")
    for evidence in report["evidence"]:
        fields(evidence, ("rgb_sequence", "bbox_xyxy_px"))
        require(integer(evidence["rgb_sequence"]), "invalid evidence sequence")
        require(vector(evidence["bbox_xyxy_px"], 4), "invalid bbox")
    return report


def validate_action(action, epoch, sequence, tick):
    fields(action, ("epoch", "sequence", "tick", "twist", "reports", "finish"))
    require(integer(action["epoch"], 1) and action["epoch"] == epoch, "wrong action epoch")
    require(integer(action["sequence"]) and action["sequence"] == sequence, "wrong action sequence")
    require(integer(action["tick"]) and action["tick"] == tick, "wrong action tick")
    require(vector(action["twist"], 3), "invalid twist")
    require(type(action["reports"]) is list and len(action["reports"]) <= MAX_REPORTS, "invalid report envelope")
    require(type(action["finish"]) is bool, "invalid finish")
    return action


OBS_FIELDS = ("protocol", "episode_id", "epoch", "tick", "sim_time_ns", "sequence", "budget_ns",
              "profile", "camera", "rgb", "depth_m", "scan", "odom", "limits")


def validate_observation(obs):
    fields(obs, OBS_FIELDS)
    require(obs["protocol"] == PROTOCOL and identifier(obs["episode_id"]), "invalid observation identity")
    require(integer(obs["epoch"], 1) and integer(obs["tick"]) and integer(obs["sequence"]), "invalid header")
    require(integer(obs["sim_time_ns"]) and obs["sim_time_ns"] == obs["tick"] * DT_NS, "invalid clock")
    require(integer(obs["budget_ns"], DT_NS), "invalid budget")
    require(obs["profile"] == "ue5_static_proxy_rgbd_scan_v1", "unsupported profile")
    camera = obs["camera"]
    fields(camera, ("width", "height", "fx", "fy", "cx", "cy", "T_base_optical"))
    require(integer(camera["width"], 1) and integer(camera["height"], 1), "invalid image size")
    require(camera["width"] * camera["height"] <= 640 * 480, "image too large")
    require(all(finite(camera[k]) for k in ("fx", "fy", "cx", "cy")) and camera["fx"] > 0 and camera["fy"] > 0, "invalid intrinsics")
    transform = camera["T_base_optical"]
    require(type(transform) is list and len(transform) == 4 and all(vector(row, 4) for row in transform), "invalid extrinsic")
    require(transform[3] == [0, 0, 0, 1], "invalid homogeneous row")
    for i in range(3):
        for j in range(3):
            require(abs(sum(transform[k][i] * transform[k][j] for k in range(3)) - (i == j)) < 1e-6, "non-rigid transform")
    determinant = sum(transform[0][i] * (transform[1][(i + 1) % 3] * transform[2][(i + 2) % 3] -
                      transform[1][(i + 2) % 3] * transform[2][(i + 1) % 3]) for i in range(3))
    require(abs(determinant - 1) < 1e-6, "left-handed transform")
    pixels = camera["width"] * camera["height"]
    for name in ("rgb", "depth_m", "scan", "odom"):
        packet = obs[name]
        body = {"rgb": ("data",), "depth_m": ("data",), "scan": ("ranges_m", "status", "angle_min", "angle_increment", "range_max_m"), "odom": ("pose",)}[name]
        fields(packet, ("epoch", "sequence", "capture_ns", "available_ns", "valid", "frame_id") + body)
        require(integer(packet["epoch"], 1) and packet["epoch"] == obs["epoch"], "stale epoch")
        require(integer(packet["sequence"]) and packet["sequence"] <= obs["sequence"], "invalid sensor sequence")
        require(integer(packet["capture_ns"]) and integer(packet["available_ns"]) and
                packet["capture_ns"] <= packet["available_ns"] <= obs["sim_time_ns"], "invalid sensor timestamps")
        require(type(packet["valid"]) is bool, "invalid sensor flag")
        require(packet["frame_id"] == {"rgb": "rgb_optical", "depth_m": "rgb_optical", "scan": "base_flu", "odom": "episode"}[name], "invalid frame")
    require(type(obs["rgb"]["data"]) is list and len(obs["rgb"]["data"]) == pixels * 3 and
            all(type(x) is int and 0 <= x <= 255 for x in obs["rgb"]["data"]), "invalid RGB8")
    require(type(obs["depth_m"]["data"]) is list and len(obs["depth_m"]["data"]) == pixels and
            all(x is None or finite(x) and 0 < x <= 100 for x in obs["depth_m"]["data"]), "invalid optical depth")
    scan = obs["scan"]
    require(type(scan["ranges_m"]) is list and 8 <= len(scan["ranges_m"]) <= 720 and
            len(scan["ranges_m"]) == len(scan["status"]), "invalid scan length")
    require(all(finite(scan[k]) for k in ("angle_min", "angle_increment", "range_max_m")) and
            scan["angle_increment"] > 0 and scan["range_max_m"] > 0, "invalid scan geometry")
    for value, status in zip(scan["ranges_m"], scan["status"]):
        require(status in ("hit", "no_return", "invalid", "dropped"), "invalid ray status")
        require((status == "hit" and finite(value) and 0 <= value <= scan["range_max_m"]) or
                (status != "hit" and value is None), "invalid ray value")
    require(vector(obs["odom"]["pose"], 3), "invalid odometry")
    fields(obs["limits"], ("vx", "vy", "wz"))
    require(all(vector(obs["limits"][k], 2) and obs["limits"][k][0] <= 0 <= obs["limits"][k][1]
                for k in obs["limits"]), "invalid limits")
    return obs


def public_copy(measurements):
    """Fail closed on nested canaries too; never forward upstream info/rewards."""
    validate_observation(measurements)
    return copy.deepcopy(measurements)
