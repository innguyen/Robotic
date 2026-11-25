"""Isolated Webots feasibility controller; no ROS, YOLO or production imports.

Recognition is simulator-provided semantic truth, not a trained detector.
JSON lines on stdout are observations, not automatic hardware acceptance.
"""
import json
import math
from pathlib import Path
import sys
import time

from controller import Robot


SENSOR_MS = 96  # 3 physics steps; 10.4167 samples / simulated second.
RADIUS_M = 0.1
TRACK_M = 0.36
STOP_RANGE_M = 0.8  # Camera origin to recognition centre, ground-plane distance.
TIMEOUT_S = 90.0
_audit_log = None


def emit(**fields):
    line = json.dumps(fields, allow_nan=False)
    print(line, flush=True)
    if _audit_log is not None:
        with _audit_log.open("a", encoding="utf-8") as stream:
            stream.write(line + "\n")


def drive(left, right, vx, wz):
    left.setVelocity((vx - wz * TRACK_M / 2) / RADIUS_M)
    right.setVelocity((vx + wz * TRACK_M / 2) / RADIUS_M)


def main():
    global _audit_log
    robot = Robot()
    audit = "--audit" in sys.argv
    if "--audit-log" in sys.argv:
        _audit_log = Path(sys.argv[sys.argv.index("--audit-log") + 1])
    if SENSOR_MS % int(robot.getBasicTimeStep()):
        raise ValueError("Sensor period must be a multiple of basicTimeStep")
    left = robot.getDevice("left wheel")
    right = robot.getDevice("right wheel")
    for motor in (left, right):
        motor.setPosition(float("inf"))
        motor.setVelocity(0.0)
    camera = robot.getDevice("camera")
    lidar = robot.getDevice("lidar")
    camera.enable(SENSOR_MS)
    camera.recognitionEnable(SENSOR_MS)
    lidar.enable(SENSOR_MS)
    emit(event="START", sensor_period_ms=SENSOR_MS,
         basic_time_step_ms=robot.getBasicTimeStep(),
         camera=[camera.getWidth(), camera.getHeight()],
         camera_fov_rad=camera.getFov(), camera_period_ms=camera.getSamplingPeriod(),
         recognition_period_ms=camera.getRecognitionSamplingPeriod(),
         rays=lidar.getHorizontalResolution(), layers=lidar.getNumberOfLayers(),
         lidar_fov_rad=lidar.getFov(), lidar_period_ms=lidar.getSamplingPeriod(),
         lidar_range_m=[lidar.getMinRange(), lidar.getMaxRange()],
         detector="webots_recognition_oracle")
    started_sim = window_sim = robot.getTime()
    window_wall = time.perf_counter()
    samples = 0
    confirmed = 0
    found = False
    timed_out = False
    last_state = None
    try:
        while robot.step(SENSOR_MS) != -1:
            now = robot.getTime()
            image = camera.getImage()
            ranges = list(lidar.getRangeImage())
            valid = (image is not None and len(image) == camera.getWidth() * camera.getHeight() * 4
                     and len(ranges) == lidar.getHorizontalResolution()
                     and all((math.isfinite(r) and lidar.getMinRange() <= r <= lidar.getMaxRange())
                             or r == math.inf for r in ranges))
            # Full-room fixture must contain returns, not merely all +Inf.
            finite = [r for r in ranges if math.isfinite(r) and r >= lidar.getMinRange()]
            valid = valid and bool(finite)
            objects = [obj for obj in camera.getRecognitionObjects() if obj.getModel() == "s0_person"]
            person = objects[0] if objects else None
            confirmed = confirmed + 1 if person is not None and valid else 0
            distance = None
            vx, wz = 0.0, 0.0
            if now - started_sim >= TIMEOUT_S:
                timed_out = True
            if not valid:
                state = "INVALID_SENSOR"
            elif found:
                state = "FOUND"
            elif timed_out:
                state = "TIMEOUT"
            elif confirmed < 2:
                state, wz = "SEARCH_SPIN", 0.35
            else:
                x, y, _ = person.getPosition()  # Webots camera: +X forward, +Y left.
                distance = math.hypot(x, y)
                bearing = math.atan2(y, x)
                if distance <= STOP_RANGE_M:
                    found, state = True, "FOUND"
                    emit(event="FOUND", sim_s=now, camera_planar_range_m=distance,
                         detector="webots_recognition_oracle")
                else:
                    state = "APPROACH"
                    wz = max(-0.4, min(0.4, bearing))
                    vx = 0.12 if abs(bearing) < 0.2 else 0.0
            # Webots ranges run left-to-right; take the centre +/-30 deg.
            n = len(ranges)
            front = ranges[n // 2 - n // 12:n // 2 + n // 12 + 1]
            front_min = min(front, default=0.0)
            if vx > 0 and front_min < 0.45:
                vx, wz, state = 0.0, 0.0, "OBSTACLE_STOP"
            drive(left, right, vx, wz)
            if audit:
                # Test telemetry only. The policy never reads Supervisor ground truth.
                robot.setCustomData(json.dumps(dict(state=state, valid=valid,
                    sim_s=now, epoch_started_sim_s=started_sim,
                    command=[vx, 0.0, wz], range_m=distance)))
            if state != last_state:
                emit(event="STATE", sim_s=now, state=state, command=[vx, 0.0, wz])
                last_state = state
            samples += int(valid)
            wall_now = time.perf_counter()
            wall_elapsed = wall_now - window_wall
            if wall_elapsed >= 5.0:
                emit(event="SAMPLE", sim_s=now, state=state, valid_sensor=valid,
                     camera_bytes=0 if image is None else len(image), rays=n,
                     finite_rays=len(finite), lidar_min_m=min(finite) if finite else None,
                     front_min_m=front_min if math.isfinite(front_min) else None,
                     person_visible=person is not None, target_range_m=distance,
                     rtf=(now - window_sim) / wall_elapsed,
                     valid_sensor_samples_per_wall_s=samples / wall_elapsed)
                window_sim, window_wall, samples = now, wall_now, 0
    finally:
        drive(left, right, 0.0, 0.0)


if __name__ == "__main__":
    main()
