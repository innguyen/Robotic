"""Stationary blocked/open/reblocked test of Webots recognition, not image AI."""
import json
import math
from pathlib import Path
import sys

from controller import Supervisor

PERIOD_MS = 96


def emit(**fields):
    line = json.dumps(fields, allow_nan=False)
    print(line, flush=True)
    if len(sys.argv) > 1:
        with (Path(sys.argv[1]) / "occlusion-controller.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(line + "\n")


def main(sim):
    for name in ("left wheel", "right wheel"):
        motor = sim.getDevice(name)
        motor.setPosition(math.inf)
        motor.setVelocity(0)
    camera, lidar = sim.getDevice("camera"), sim.getDevice("lidar")
    camera.enable(PERIOD_MS)
    camera.recognitionEnable(PERIOD_MS)
    lidar.enable(PERIOD_MS)
    wall = sim.getFromDef("OCCLUDER").getField("translation")
    person = sim.getFromDef("PERSON")
    evidence = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    start = sim.getSelf().getPosition()
    counts = {}
    emit(event="OCCLUSION_START", detector="webots_recognition_oracle", occlusion=1,
         robot_position_m=start, ground_truth_use="assertions_only", period_ms=PERIOD_MS)
    for phase, wall_y, expected in (("blocked", -1, False), ("open", 2, True), ("reblocked", -1, False)):
        wall.setSFVec3f([-.5, wall_y, 1.1])
        # Flush old camera buffers after changing geometry; sensor remains enabled.
        for _ in range(3):
            if sim.step(PERIOD_MS) == -1:
                raise RuntimeError("Stopped during warmup")
        visible = 0
        for sample in range(12):
            if sim.step(PERIOD_MS) == -1:
                raise RuntimeError("Stopped during occlusion test")
            image, ranges = camera.getImage(), lidar.getRangeImage()
            if image is None or len(image) != 76800 or len(ranges) != 180:
                raise AssertionError("Invalid camera/scan dimensions")
            if not all((math.isfinite(r) and .05 <= r <= 6) or r == math.inf for r in ranges):
                raise AssertionError("Invalid scan range")
            objects = [o for o in camera.getRecognitionObjects() if o.getModel() == "s0_person"]
            seen = bool(objects)
            visible += int(seen)
            # Ground truth independently proves the target exists inside FOV/range.
            relative = person.getPosition()  # world position for assertions only
            pose = sim.getSelf().getPosition()
            orientation = sim.getSelf().getOrientation()
            delta = [relative[i] - pose[i] for i in range(3)]
            base = [sum(orientation[3*j+i] * delta[j] for j in range(3)) for i in range(3)]
            camera_gt = [base[0] - .12, base[1], base[2] - .55]
            if camera_gt[0] <= 0 or abs(math.atan2(camera_gt[1], camera_gt[0])) > camera.getFov()/2:
                raise AssertionError("Target no longer inside horizontal FOV")
            if math.sqrt(sum(v*v for v in camera_gt)) >= 3.5:
                raise AssertionError("Target outside recognition range")
            front = min(ranges[88:92])
            if not math.isfinite(front) or (phase != "open" and not 1.28 < front < 1.38):
                raise AssertionError("Opaque wall not present in front LiDAR rays")
            if phase == "open" and front <= 2:
                raise AssertionError("Opening did not clear the wall")
            emit(event="OCCLUSION_SAMPLE", phase=phase, sample=sample, sim_s=sim.getTime(),
                 person_visible=seen, recognized_camera_xyz_m=list(objects[0].getPosition()) if seen else None,
                 ground_truth_camera_xyz_m=camera_gt, front_min_m=front, camera_bytes=len(image))
            if seen != expected:
                raise AssertionError("Recognition disagrees with occlusion phase: " + phase)
            if math.dist(start[:2], pose[:2]) > .005:
                raise AssertionError("Stationary robot moved horizontally")
        counts[phase] = visible
        if evidence:
            path = evidence / f"occlusion-{phase}.png"
            if camera.saveImage(str(path), 100) != 0:
                raise RuntimeError("Could not save " + str(path))
        emit(event="OCCLUSION_PHASE_PASS", phase=phase, visible=visible, samples=12)
    emit(event="OCCLUSION_SUITE_PASS", counts=counts)
    if evidence:
        (evidence / "occlusion-result.json").write_text(json.dumps(counts, indent=2) + "\n", encoding="utf-8")
        sim.simulationQuit(0)
    else:
        # Interactive use: return to the initial blocked setup, leave sensors live.
        wall.setSFVec3f([-.5, -1, 1.1])
    while sim.step(PERIOD_MS) != -1:
        pass


if __name__ == "__main__":
    sim = Supervisor()
    try:
        main(sim)
    except Exception as exc:
        emit(event="AUDIT_FAIL", error=repr(exc))
        if len(sys.argv) > 1:
            sim.simulationQuit(1)
            sim.step(PERIOD_MS)
        raise
