"""Test-only Supervisor: three mission trials through load, reset and reload.

Ground truth is used for assertions/logs only, never supplied to s0_smoke.
The runner adds this observer to a disposable copy of the canonical world.
"""
import json
import math
from pathlib import Path
import sys

from controller import Supervisor


def emit(**fields):
    line = json.dumps(fields, allow_nan=False)
    print(line, flush=True)
    with Path(sys.argv[1]).with_name("repeat-supervisor.jsonl").open("a", encoding="utf-8") as stream:
        stream.write(line + "\n")


def main(sim):
    state_path = Path(sys.argv[1])
    saved = json.loads(state_path.read_text(encoding="utf-8"))
    trial = len(saved["trials"])
    robot = sim.getFromDef("S0_ROBOT")
    initial = robot.getPosition()
    orientation = robot.getOrientation()
    yaw = math.atan2(orientation[3], orientation[0])
    initial_time = sim.getTime()
    emit(event="REPEAT_START", trial=trial, via=("load", "simulationReset+restartController", "worldReload")[trial],
         sim_s=sim.getTime(), initial_position_m=initial, initial_yaw_rad=yaw,
         initial_velocity=robot.getVelocity(), pose_source="supervisor_ground_truth_assertions_only")
    # Explicit restart requests take one basic tick to be serviced on R2025a.
    # Still require the exact untouched physical initial state, not a near-end pose.
    expected_time = sim.getBasicTimeStep()/1000 if trial == 1 else 0
    if abs(initial_time - expected_time) > 1e-10 or any(abs(a-b) > 1e-8 for a, b in zip(initial, (-2, -1, .1))):
        raise AssertionError("World did not restore initial position/time")
    if abs(yaw + math.pi / 2) > 1e-8 or max(map(abs, robot.getVelocity())) > 1e-8:
        raise AssertionError("World did not restore heading/zero velocity")
    sequence, valid_samples = [], 0
    last_sample = None
    found_time = found_position = None
    while sim.step(96) != -1:
        now = sim.getTime()
        if now - initial_time > 90:
            raise AssertionError("Mission exceeded 90 s")
        raw = robot.getField("customData").getSFString()
        if not raw:
            if now > 1:
                raise AssertionError("Missing smoke-controller telemetry")
            continue
        obs = json.loads(raw)
        # Supervisor reset preserves customData. Never consume a previous epoch's
        # FOUND or count the same controller observation twice.
        if (obs["epoch_started_sim_s"] != initial_time
                or not initial_time < obs["sim_s"] <= now
                or obs["sim_s"] == last_sample):
            if valid_samples == 0 and now - initial_time > 1:
                raise AssertionError("No fresh telemetry for this reset epoch")
            continue
        last_sample = obs["sim_s"]
        if not obs["valid"]:
            raise AssertionError("Invalid camera/LiDAR sample")
        valid_samples += 1
        state = obs["state"]
        if not sequence or sequence[-1] != state:
            sequence.append(state)
        if state in ("TIMEOUT", "INVALID_SENSOR", "OBSTACLE_STOP") or now > 90:
            raise AssertionError(f"Mission failed at {now}: {state}")
        if state != "FOUND":
            continue
        if any(obs["command"]):
            raise AssertionError("Nonzero command after FOUND")
        if found_time is None:
            found_time, found_position = now, robot.getPosition()
        drift = math.dist(found_position, robot.getPosition())
        if drift > .005:
            raise AssertionError("Robot drifted more than 5 mm after FOUND")
        if now - found_time < 2:
            continue
        if sequence != ["SEARCH_SPIN", "APPROACH", "FOUND"] or not 3 < found_time < 90:
            raise AssertionError("Did not execute a complete search from initial pose")
        speed = max(map(abs, robot.getVelocity()))
        if speed > .001:
            raise AssertionError("Robot not physically stopped")
        result = dict(trial=trial, initial_sim_s=initial_time, initial_position_m=initial, initial_yaw_rad=yaw,
                      states=sequence, found_observed_sim_s=found_time,
                      stopped_position_m=robot.getPosition(), hold_s=now-found_time,
                      hold_drift_m=drift, stopped_max_velocity=speed, valid_samples=valid_samples)
        saved["trials"].append(result)
        state_path.write_text(json.dumps(saved, indent=2) + "\n", encoding="utf-8")
        emit(event="REPEAT_PASS", **result)
        if trial == 0:
            # Unlike the GUI Reset button, this API preserves controller state.
            # Restart both controllers explicitly to emulate a full mission reset.
            emit(event="RESET_REQUEST", method="simulationReset+restartController")
            sim.simulationReset()
            robot.restartController()
            sim.getSelf().restartController()
        elif trial == 1:
            emit(event="RESET_REQUEST", method="worldReload")
            sim.worldReload()
        else:
            # Explicit tolerances; not a claim of cross-machine bitwise determinism.
            trials = saved["trials"]
            if max(t["found_observed_sim_s"] for t in trials) - min(t["found_observed_sim_s"] for t in trials) > .192:
                raise AssertionError("Repeat mission time differs by more than two samples")
            if any(math.dist(trials[0]["stopped_position_m"], t["stopped_position_m"]) > .01 for t in trials[1:]):
                raise AssertionError("Repeat stop pose differs by more than 1 cm")
            emit(event="REPEAT_SUITE_PASS", trials=3)
            sim.simulationQuit(0)
        sim.step(32)  # Let Webots process the reset/reload/quit request.
        return


if __name__ == "__main__":
    sim = Supervisor()
    try:
        main(sim)
    except Exception as exc:
        emit(event="AUDIT_FAIL", error=repr(exc))
        sim.simulationQuit(1)
        sim.step(32)
        raise
