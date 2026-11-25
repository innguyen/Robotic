"""Privileged engineering checks, separate from policies and benchmark scores."""
import copy
import hashlib
import math

from .contracts import canonical
from .scenarios import scenario
from .safety import SafetyGate


def calibrate(backend):
    records, checks = [], {}
    config = scenario()
    config["start_yaw"] = 0.
    for axis, action, expected in (("forward", [.3, 0, 0], [.03, 0, 0]),
                                   ("left", [0, .2, 0], [0, .02, 0]),
                                   ("yaw_left", [0, 0, .6], [0, 0, .06])):
        obs, truth = backend.reset(config, 30)
        after, _ = backend.advance(action)
        actual = after["odom"]["pose"]
        checks[axis] = max(abs(a - b) for a, b in zip(actual, expected)) < 1e-5
        stopped, _ = backend.advance([0, 0, 0])
        checks[axis + "_zero"] = max(abs(a - b) for a, b in zip(actual, stopped["odom"]["pose"])) < 1e-6
        records.append({"axis": axis, "expected": expected, "actual": actual})
    initial = []
    epochs = []
    for _ in range(10):
        obs, truth = backend.reset(config, 30)
        initial.append({"pose": obs["odom"]["pose"], "rgb_sha256": hashlib.sha256(canonical(obs["rgb"]["data"])).hexdigest(),
                        "scan_sha256": hashlib.sha256(canonical(obs["scan"]["ranges_m"])).hexdigest()})
        epochs.append(obs["epoch"])
    checks["ten_reset_epochs"] = len(set(epochs)) == 10 and epochs == sorted(epochs)
    checks["ten_reset_pose_scan_repeat"] = len({canonical([item["pose"], item["scan_sha256"]]) for item in initial}) == 1
    checks["ten_reset_rgb_repeat"] = len({item["rgb_sha256"] for item in initial}) == 1
    blocked = copy.deepcopy(config)
    blocked["boxes"].append({"id": "calibration_wall", "center_m": [.6, 0, .5], "size_m": [.1, 2., 1.]})
    obs, _ = backend.reset(blocked, 30)
    before = list(obs["odom"]["pose"])
    # This intentionally bypasses the common safety gate to test the engine's
    # swept collision at low speed in a disposable scene. Never a policy run.
    contact = False
    for _ in range(20):
        obs, truth = backend.advance([.3, 0, 0])
        if truth["safety_events"]:
            contact = "CONTACT" in truth["safety_events"]
            break
    checks["engine_sweep_blocks_wall"] = contact and .30 <= obs["odom"]["pose"][0] <= .351
    checked, reason = SafetyGate().check([.3, 0, 0], obs)
    checks["common_gate_stops_near_wall"] = checked == [0, 0, 0] and reason == "CLEARANCE_STOP"
    backend.stop()
    return {"status": "PASS_ENGINE_CALIBRATION" if all(checks.values()) else "FAIL_ENGINE_CALIBRATION",
            "checks": checks, "axes": records, "reset_initial_states": initial, "epochs": epochs,
            "collision_final_pose": obs["odom"]["pose"], "collision_events": truth["safety_events"],
            "scope": "static proxy axes, collision, zero and same-machine reset repeat only"}
