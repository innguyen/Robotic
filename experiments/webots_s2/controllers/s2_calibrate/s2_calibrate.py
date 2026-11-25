"""Assertions-only calibration. Scene target access never enters the mission controller."""
import json
import math
from pathlib import Path
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from controller import Supervisor
from adapter import ANGLE_INCREMENT, ANGLE_MIN, RAYS, CoreAdapter, ZERO, camera_target, odom_pose, rotate, wrap
from runtime import JsonLog, WebotsIO


def main(sim, cfg):
    out = Path(cfg["out"])
    log = JsonLog(out / "calibration.jsonl")
    io = WebotsIO(sim)
    io.emergency_zero()
    body = io.body
    # Independent analytic ranges to the four inside wall faces (+/-2.95 m).
    for _ in range(3):
        sim.step(96)
    ranges = io.lidar.getRangeImage()
    log.emit(event="SCAN_RAW", values_m=[v if math.isfinite(v) else None for v in ranges],
             body_position=body.getPosition(), lidar_translation=[.12, 0, .15])
    io.camera.saveImage(str(out / "initial-camera.png"), 100)
    errors, ordinary, cardinal = [], [], []
    for i, observed in enumerate(ranges):
        angle = math.pi-(i+.5)*ANGLE_INCREMENT
        dx, dy = math.cos(angle), math.sin(angle)
        candidates = [((2.95 if dx > 0 else -2.95)-.12)/dx if abs(dx) > 1e-9 else math.inf,
                      2.95/abs(dy) if abs(dy) > 1e-9 else math.inf]
        error = observed-min(candidates)
        errors.append(error)
        seam_distance = min(abs(wrap(angle-(math.pi/4+k*math.pi/2))) for k in range(4))
        if seam_distance > ANGLE_INCREMENT:
            ordinary.append(abs(error))
        if min(abs(wrap(angle-k*math.pi/2)) for k in range(4)) < 2*ANGLE_INCREMENT:
            cardinal.append(abs(error))
    # R2025a cylindrical projection underestimates depth next to cube seams.
    # Bound and report that bias separately; do not hide missing rays or allow
    # arbitrary errors elsewhere. Overestimated free space is limited to 1 cm.
    if (not all(math.isfinite(v) for v in errors) or max(map(abs, errors)) > .15
            or max(errors) > .01 or max(ordinary) > .005 or max(cardinal) > .005):
        raise AssertionError("Analytic wall scan mismatch: " + repr(errors))
    log.emit(event="WALL_SCAN_PASS", checked_rays=len(errors), max_error_m=max(errors),
             max_abs_error_m=max(map(abs, errors)), min_signed_error_m=min(errors),
             max_nonseam_error_m=max(ordinary), max_cardinal_error_m=max(cardinal),
             raw_angle_first_rad=math.pi-ANGLE_INCREMENT/2, normalized_angle_min_rad=ANGLE_MIN,
             angle_increment_rad=ANGLE_INCREMENT, cardinal_directions=["front", "left", "right", "rear"])
    io.lidar.enablePointCloud()
    sim.step(96)
    cloud = io.lidar.getPointCloud()
    angle_errors = []
    for i, point in enumerate(cloud):
        if math.isfinite(point.x) and math.isfinite(point.y):
            angle_errors.append(abs(wrap(math.atan2(point.y, point.x)-(math.pi-(i+.5)*ANGLE_INCREMENT))))
    if len(angle_errors) != RAYS or max(angle_errors) > 1e-5:
        raise AssertionError("Point cloud and scan angle mapping differ")
    io.lidar.disablePointCloud()
    log.emit(event="POINT_CLOUD_ANGLES_PASS", points=len(angle_errors), max_error_rad=max(angle_errors))
    person = sim.getFromDef("PERSON")
    for yaw in (0., math.pi/2, -math.pi/2, math.pi):
        body.getField("translation").setSFVec3f([0, 0, .1])
        body.getField("rotation").setSFRotation([0, 0, 1, yaw])
        body.resetPhysics()
        tx, ty = rotate(1.5, .2, yaw)
        person.getField("translation").setSFVec3f([tx, ty, .65])
        for _ in range(3):
            sim.step(96)
        objects = [o for o in io.camera.getRecognitionObjects() if o.getModel() == "s0_person"]
        if len(objects) != 1:
            raise AssertionError("Missing calibration oracle target")
        pose = io.pose()
        frame_origin = (-.3, .2, -.7)
        observed = camera_target(objects[0].getPosition(), odom_pose(pose, frame_origin))
        expected = odom_pose((tx, ty, 0), frame_origin)[:2]
        error = math.dist(observed, expected)
        if error > .035:
            raise AssertionError("Camera->base->odom transform error: " + str(error))
        log.emit(event="CAMERA_FRAME_PASS", yaw=yaw, error_m=error, target_odom=observed, expected_odom=expected)
    # Remove person from the motion test's path (test setup only).
    person.getField("translation").setSFVec3f([2.6, 2.6, .65])
    for command in ((.12, 0., 0.), (0., .1, 0.), (0., 0., .3), (.06, .04, .2)):
        for yaw in (0., math.pi/2):
            body.getField("translation").setSFVec3f([0, 0, .1])
            body.getField("rotation").setSFRotation([0, 0, 1, yaw])
            body.resetPhysics()
            adapter = CoreAdapter((0., 0., yaw))
            for _ in range(3):
                sim.step(96)
            p0, a0 = body.getPosition(), io.pose()[2]
            for _ in range(10):
                obs = io.read(0)
                result = adapter.step(obs, test_command=command)
                if math.dist(result["applied"], command) > 1e-8:
                    raise AssertionError("Calibration command blocked/clipped unexpectedly")
                io.apply(result)
                sim.step(96)
            p1, a1 = body.getPosition(), io.pose()[2]
            duration = .96
            if abs(command[2]) < 1e-9:
                dx, dy = command[0]*duration, command[1]*duration
            else:
                theta = command[2]*duration
                dx = (command[0]*math.sin(theta)+command[1]*(math.cos(theta)-1))/command[2]
                dy = (command[0]*(1-math.cos(theta))+command[1]*math.sin(theta))/command[2]
            ex, ey = rotate(dx, dy, a0)
            position_error = math.dist((p1[0]-p0[0], p1[1]-p0[1]), (ex, ey))
            yaw_error = abs(wrap(a1-a0-command[2]*duration))
            # Piecewise-constant world velocity integration at 96 ms, no traction noise.
            if position_error > .006 or yaw_error > .005:
                raise AssertionError("Omni actuator mapping mismatch")
            io.emergency_zero()
            sim.step(96)
            if max(map(abs, body.getVelocity()[:2])) > .001 or abs(body.getVelocity()[5]) > .001:
                raise AssertionError("Omni zero command did not stop")
            log.emit(event="ACTUATOR_PASS", command=command, start_yaw=yaw, position_error_m=position_error,
                     yaw_error_rad=yaw_error, monitor=result["monitor"], stopped=True)
            adapter.close()
    # Actual STOP_ZONE on rendered LiDAR, followed by a native sensor-disable test.
    body.getField("translation").setSFVec3f([0, 0, .1])
    body.getField("rotation").setSFRotation([0, 0, 1, 0])
    body.resetPhysics()
    sim.getFromDef("MONITOR_BLOCK").getField("translation").setSFVec3f([.3, 0, .4])
    for _ in range(3):
        sim.step(96)
    adapter = CoreAdapter((0., 0., 0.))
    before = body.getPosition()
    for _ in range(6):
        result = adapter.step(io.read(0), test_command=(.1, 0., 0.))
        if result["monitor"] != "STOP_ZONE" or any(result["applied"]):
            raise AssertionError("Rendered obstacle did not trigger STOP_ZONE")
        io.apply(result)
        sim.step(96)
    if math.dist(before[:2], body.getPosition()[:2]) > .001:
        raise AssertionError("STOP_ZONE did not hold physical position")
    log.emit(event="MONITOR_STOP_PASS", raw=result["raw"], checked=result["checked"],
             drift_m=math.dist(before[:2], body.getPosition()[:2]))
    io.lidar.disable()
    stale_reasons = set()
    for _ in range(8):
        sim.step(96)
        result = adapter.step(io.read(0), test_command=(.1, 0., 0.))
        io.apply(result)
        stale_reasons.add(result["reason"])
        if any(result["applied"]):
            raise AssertionError("Disabled LiDAR did not stop")
    if "STALE_OR_FUTURE_SCAN" not in stale_reasons:
        raise AssertionError("Disabled LiDAR got a fake fresh timestamp")
    log.emit(event="NATIVE_LIDAR_DISABLE_PASS", reasons=sorted(stale_reasons), applied=result["applied"])
    io.enable()
    sim.step(96)
    result = adapter.step(io.read(0), test_command=ZERO)
    io.apply(result)
    if result["monitor"] != "IDLE":
        raise AssertionError("Sensors did not recover after re-enable")
    adapter.close()
    log.emit(event="SUITE_PASS", name="calibration")
    (out / "result.json").write_text('{"calibration": "PASS"}\n', encoding="utf-8")
    sim.simulationQuit(0)
    sim.step(32)


if __name__ == "__main__":
    sim = Supervisor()
    cfg = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
    try:
        main(sim, cfg)
    except Exception:
        JsonLog(Path(cfg["out"]) / "calibration.jsonl").emit(event="CALIBRATION_ERROR", traceback=traceback.format_exc())
        sim.getSelf().setVelocity([0.]*6)
        sim.simulationQuit(1)
        sim.step(32)
        raise
