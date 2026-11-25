"""Independent observer. Supervisor target truth is assertions-only, never policy input."""
import json
import math
from pathlib import Path
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from controller import Supervisor
from runtime import JsonLog
from geometry import footprint_separation


def main(sim, config):
    out = Path(config["out"])
    log = JsonLog(out / "audit.jsonl")
    scenario = config["scenario"]
    robot = sim.getFromDef("S2_ROBOT")
    record_path = out / "result.json"
    saved = json.loads(record_path.read_text(encoding="utf-8")) if record_path.exists() else dict(episodes=[])
    trial = len(saved["episodes"])
    last_stamp, seen, reasons, monitors, faults, states = None, False, set(), set(), set(), set()
    samples = fault_physical_stop_checks = 0
    max_lateral = max_detour = 0.
    done_time = done_position = None
    minimum_clearance = math.inf
    start = robot.getPosition()
    initial_yaw = math.atan2(robot.getOrientation()[3], robot.getOrientation()[0])
    if math.dist(start, [*scenario["start"][:2], .1]) > 1e-8 or abs(initial_yaw-scenario["start"][2]) > 1e-8:
        raise AssertionError("Initial fixture pose was not restored")
    log.emit(event="AUDIT_START", trial=trial, initial_position=start, initial_yaw=initial_yaw, sim_s=sim.getTime())
    while sim.step(32) != -1:
        contacts = [list(p.getPoint()) for p in robot.getContactPoints(True) if p.getPoint()[2] > .03]
        if contacts:
            raise AssertionError("Non-floor physical collision: " + repr(contacts))
        raw = robot.getField("customData").getSFString()
        if not raw:
            if sim.getTime() > 2:
                raise AssertionError("No core telemetry")
            continue
        obs = json.loads(raw)
        if obs["fault"] and sim.getTime()-obs["sim_s"] >= .032-1e-8:
            if max(map(abs, robot.getVelocity()[:2])) > .001 or abs(robot.getVelocity()[5]) > .001:
                raise AssertionError("Fault gate issued zero but robot did not physically stop")
            fault_physical_stop_checks += 1
        if obs["sim_s"] > sim.getTime() or obs["sim_s"] == last_stamp:
            continue
        # API reset preserves customData; reject the old epoch until the core resets.
        if obs["epoch"] != trial:
            continue
        last_stamp = obs["sim_s"]
        samples += 1
        reasons.add(obs["reason"])
        monitors.add(obs["monitor"])
        states.add(obs["mission"]["state"])
        applied = obs["applied"]
        if obs["monitor_steps"] < 1 or any(abs(v) > limit+1e-9 for v, limit in zip(applied, (.18, .12, math.pi/6))):
            raise AssertionError("Missing monitor or actuator bound violation")
        max_lateral = max(max_lateral, abs(applied[1]))
        pos = robot.getPosition()
        max_detour = max(max_detour, abs(pos[1]-scenario["start"][1]))
        # Exact oriented footprint-vs-AABB geometric checks independent of LiDAR.
        yaw = math.atan2(robot.getOrientation()[3], robot.getOrientation()[0])
        for box in scenario["boxes"]:
            clearance = footprint_separation(pos, yaw, box)
            minimum_clearance = min(minimum_clearance, clearance)
            if clearance < -.005:
                raise AssertionError("Robot footprint penetrated obstacle bounding box")
        if obs["fault"]:
            faults.add(obs["fault"])
            if any(applied) or not ("INVALID" in obs["reason"] or "STALE" in obs["reason"]):
                raise AssertionError("Injected fault did not stop via invalid/stale gate")
        if obs["target"] is not None:
            if not obs["recognition_visible"]:
                raise AssertionError("Policy received a target without camera recognition")
            seen = True
        if scenario["expect"] == "NO_FALSE_FOUND" and (seen or obs["mission"]["state"] == "FOUND"):
            raise AssertionError("Found a person in an absent-person fixture")
        if config["name"] == "occluded" and obs["sim_s"]-obs["episode_started_sim_s"] < .5:
            if obs["recognition_visible"] or obs["target"] is not None:
                raise AssertionError("Oracle saw through initial wall")
        if not obs["done"]:
            continue
        expected_found = scenario["expect"] == "FOUND"
        if expected_found and obs["done"] != "FOUND":
            raise AssertionError("Mission ended without FOUND: " + obs["done"])
        if any(applied):
            raise AssertionError("Nonzero command in terminal state")
        if done_time is None:
            done_time, done_position = obs["sim_s"], robot.getPosition()
            if expected_found:
                if not seen or obs["navigation_steps"] < 1:
                    raise AssertionError("No actual Navigator approach/recognition")
                target = sim.getFromDef("PERSON").getPosition()  # assertions only
                distance = math.dist(robot.getPosition()[:2], target[:2])
                if not .55 < distance < 1.05:
                    raise AssertionError("Wrong target standoff: " + str(distance))
        if math.dist(done_position[:2], pos[:2]) > .005:
            raise AssertionError("Terminal state did not hold position")
        if obs["sim_s"]-done_time < 1.:
            continue
        if max(map(abs, robot.getVelocity()[:2])) > .001 or abs(robot.getVelocity()[5]) > .001:
            raise AssertionError("Robot not physically stopped")
        if config["name"] == "obstacle" and (max_detour < .45 or max_lateral < .01):
            raise AssertionError("Did not demonstrate holonomic obstacle detour")
        if config["name"] == "faults" and faults != {f[2] for f in scenario["faults"]}:
            raise AssertionError("Some injected faults never ran")
        result = dict(trial=trial, done=obs["done"], done_sim_s=done_time,
                      elapsed_s=done_time-obs["episode_started_sim_s"], states=sorted(states),
                      reasons=sorted(reasons), monitors=sorted(monitors), faults=sorted(faults),
                      pose=pos, samples=samples, max_lateral_m_s=max_lateral, max_detour_m=max_detour,
                      min_box_clearance_m=minimum_clearance if math.isfinite(minimum_clearance) else None,
                      policy_steps=obs["policy_steps"], navigation_steps=obs["navigation_steps"],
                      hold_s=obs["sim_s"]-done_time, closed_workers=obs["closed_workers"],
                      fault_physical_stop_checks=fault_physical_stop_checks)
        saved["episodes"].append(result)
        record_path.write_text(json.dumps(saved, indent=2) + "\n", encoding="utf-8")
        log.emit(event="EPISODE_PASS", **result)
        if scenario.get("repeat") and trial < 2:
            if trial == 0:
                log.emit(event="RESET_REQUEST", method="simulationReset_no_controller_restart")
                sim.simulationReset()  # deliberately exercise in-process adapter cleanup
                sim.getSelf().restartController()
            else:
                log.emit(event="RESET_REQUEST", method="worldReload")
                # Runtime config gives the reloaded controller a unique next epoch.
                policy_config = Path(config["policy_config"])
                data = json.loads(policy_config.read_text(encoding="utf-8"))
                data["epoch"] = 2
                policy_config.write_text(json.dumps(data), encoding="utf-8")
                sim.worldReload()
        else:
            if scenario.get("repeat"):
                first = saved["episodes"][0]
                for repeat in saved["episodes"][1:]:
                    if math.dist(first["pose"][:2], repeat["pose"][:2]) > .02 or abs(first["elapsed_s"]-repeat["elapsed_s"]) > .3:
                        raise AssertionError("Reset/reload results differ beyond declared tolerances")
            log.emit(event="SUITE_PASS", name=config["name"])
            sim.simulationQuit(0)
        sim.step(32)
        return


if __name__ == "__main__":
    sim = Supervisor()
    config = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    try:
        main(sim, config)
    except Exception:
        JsonLog(Path(config["out"]) / "audit.jsonl").emit(event="AUDIT_ERROR", traceback=traceback.format_exc())
        sim.simulationQuit(1)
        sim.step(32)
        raise
