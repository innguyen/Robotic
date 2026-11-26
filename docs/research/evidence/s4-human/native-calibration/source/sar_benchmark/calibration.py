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


def calibrate_human(backend):
    from .scenarios import sar_scenario
    records,checks = [],{}
    for distance in (.6,1.,2.,4.):
        config = sar_scenario("absent",environment="outdoor",depth_model="ideal")
        config["boxes"] = [{"id":"calibration_plane","center_m":[distance+.26,0,2.],"size_m":[.02,10.,6.]}]
        obs,_ = backend.reset(config,30)
        width,height = obs["camera"]["width"],obs["camera"]["height"]
        samples = [obs["depth_m"]["data"][(height//2)*width+col] for col in (width//4,width//2,3*width//4)]
        error = max(abs(value-distance) for value in samples if value is not None)
        checks[f"plane_{distance}m"] = len(samples)==3 and all(v is not None for v in samples) and error < .01
        records.append({"distance_m":distance,"optical_Z_samples_m":samples,"max_error_m":error})
    config = sar_scenario("clean",environment="outdoor",depth_model="ideal")
    states = []
    for _ in range(10):
        obs,truth = backend.reset(config,30)
        states.append({"epoch":obs["epoch"],"pose":obs["odom"]["pose"],"scan_hash":hashlib.sha256(canonical(obs["scan"]["ranges_m"])).hexdigest(),
                       "rgb_hash":hashlib.sha256(canonical(obs["rgb"]["data"])).hexdigest(),"visible":truth["frame"]["visible"]})
    checks["epoch_unique"] = len({s["epoch"] for s in states})==10
    checks["reset_pose_scan_repeat"] = len({canonical([s["pose"],s["scan_hash"]]) for s in states})==1
    checks["reset_render_repeat"] = len({s["rgb_hash"] for s in states})==1
    checks["visible_rendered_human"] = all(s["visible"].get("v0",{}).get("pixels",0)>=25 for s in states)
    checks["scan_360"] = len(obs["scan"]["ranges_m"])==360
    for axis,action,expected in (("forward",[.3,0,0],[.03,0,0]),("left",[0,.2,0],[0,.02,0]),("yaw",[0,0,.6],[0,0,.06])):
        obs,_ = backend.reset(config,30)
        moved,_ = backend.advance(action)
        checks[axis] = max(abs(a-b) for a,b in zip(moved["odom"]["pose"],expected))<1e-5
        stopped,_ = backend.advance([0,0,0])
        checks[axis+"_zero"] = max(abs(a-b) for a,b in zip(moved["odom"]["pose"],stopped["odom"]["pose"]))<1e-6
    backend.stop()
    return {"status":"PASS_ENGINE_CALIBRATION" if all(checks.values()) else "FAIL_ENGINE_CALIBRATION", "checks":checks,"planes":records,
            "resets":states,"scope":"native PNG/EXR depth Z, renders, scan, epochs and ideal actuation; not empirical sensor or wheel dynamics calibration"}
