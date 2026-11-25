"""Webots device/actuator plumbing. Reads only self pose and on-board sensors."""
from dataclasses import replace
import json
import math
from pathlib import Path
import sys

from adapter import (ANGLE_INCREMENT, ANGLE_MIN, CoreAdapter, Observation, PERIOD_MS, RAYS,
                     ZERO, actuator_limit, world_velocity)


class JsonLog:
    def __init__(self, path=None):
        self.path = Path(path) if path else None

    def emit(self, **record):
        line = json.dumps(record, allow_nan=False)
        print(line, flush=True)
        if self.path:
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(line + "\n")


class WebotsIO:
    def __init__(self, sim):
        self.sim, self.body = sim, sim.getSelf()
        self.camera, self.lidar = sim.getDevice("camera"), sim.getDevice("lidar")
        self.enable()

    def enable(self):
        self.camera.enable(PERIOD_MS)
        self.camera.recognitionEnable(PERIOD_MS)
        self.lidar.enable(PERIOD_MS)
        self.lidar.disablePointCloud()
        self.scan_stamp = self.camera_stamp = None
        self.scan_cache = self.camera_cache = None
        if (self.camera.getWidth(), self.camera.getHeight(), self.lidar.getHorizontalResolution(),
                self.lidar.getNumberOfLayers()) != (160, 120, RAYS, 1):
            raise ValueError("DEVICE_CONTRACT_MISMATCH")
        if not math.isclose(self.lidar.getFov(), 2*math.pi) or self.sim.getBasicTimeStep() != 32:
            raise ValueError("DEVICE_TIMING_OR_FOV_MISMATCH")

    def pose(self):
        position, matrix = self.body.getPosition(), self.body.getOrientation()
        # The planar core cannot safely represent a tipped robot.
        roll = math.atan2(matrix[7], matrix[8])
        pitch = math.asin(max(-1., min(1., -matrix[6])))
        if max(abs(roll), abs(pitch)) > math.radians(5):
            raise ValueError("NONPLANAR_POSE")
        return (position[0], position[1], math.atan2(matrix[3], matrix[0]))

    def read(self, epoch, fault=None):
        now = self.sim.getTime()
        try:
            pose = self.pose()
        except ValueError:
            pose = (math.nan,)*3
        if self.camera.getSamplingPeriod() == PERIOD_MS and self.camera.getRecognitionSamplingPeriod() == PERIOD_MS:
            image = self.camera.getImage()
            valid = image is not None and len(image) == 76800
            recognized = tuple((obj.getId(), tuple(obj.getPosition())) for obj in self.camera.getRecognitionObjects()
                               if obj.getModel() == "s0_person")
            self.camera_cache, self.camera_stamp = (valid, recognized), now
        valid, recognized = self.camera_cache if self.camera_cache else (False, ())
        if self.lidar.getSamplingPeriod() == PERIOD_MS:
            self.scan_cache, self.scan_stamp = tuple(self.lidar.getRangeImage()), now
        ranges = self.scan_cache if self.scan_cache else ()
        # Stamps advance only on enabled synchronous device ticks, not on receipts
        # of cached data from a disabled/misconfigured device.
        obs = Observation(epoch, now, pose, now, ranges,
                          self.scan_stamp if self.scan_stamp is not None else -1., valid,
                          self.camera_stamp if self.camera_stamp is not None else -1., recognized)
        # Deliberate, logged fault injection after acquiring actual sensor data.
        # Stale buffers retain an OLD stamp, never a fake fresh receipt timestamp.
        if fault == "invalid_scan":
            obs = replace(obs, ranges=(math.nan,)*RAYS)
        elif fault == "stale_scan":
            obs = replace(obs, scan_stamp=now-.7)
        elif fault == "invalid_camera":
            obs = replace(obs, camera_valid=False, recognition=())
        elif fault == "stale_camera":
            obs = replace(obs, camera_stamp=now-.7)
        elif fault == "invalid_pose":
            obs = replace(obs, world_pose=(math.nan,)*3)
        elif fault == "stale_pose":
            obs = replace(obs, pose_stamp=now-.7)
        elif fault:
            raise ValueError("Unknown injected fault: " + fault)
        return obs

    def apply(self, result):
        command = tuple(result["applied"])
        if result["monitor_steps"] < 1 or actuator_limit(command) != command:
            raise ValueError("UNFILTERED_ACTUATOR_COMMAND")
        # Full holonomic Twist. Physics resolves contacts; no pose teleportation.
        yaw = self.pose()[2] if command != ZERO else 0
        self.body.setVelocity(world_velocity(command, yaw))

    def emergency_zero(self):
        # Shutdown/startup only; zero is fail-safe even without a valid observation.
        self.body.setVelocity([0.]*6)


def default_config():
    return dict(log=None, status=None, limit_s=180, faults=[], epoch=0, async_planning=False)


def run_core(sim, config):
    log = JsonLog(config.get("log"))
    io = WebotsIO(sim)
    io.emergency_zero()
    epoch = config.get("epoch", 0)
    started = sim.getTime()
    adapter = CoreAdapter(io.pose(), epoch, config.get("async_planning", False))
    def announce():
        log.emit(event="CORE_START", epoch=adapter.epoch, sim_s=sim.getTime(),
                 classes=[type(adapter.mission).__module__+".SearchMission",
                          type(adapter.nav).__module__+".Navigator",
                          type(adapter.nav.costmap).__module__+".Costmap",
                          type(adapter.monitor).__module__+".Monitor"],
                 core_paths={name: sys.modules[name].__file__ for name in
                             ("search_core_ros1", "navigation_core_ros1", "collision_monitor_ros1")},
                 model="ideal_holonomic_velocity_rigid_body", rays=RAYS,
                 angle_min=ANGLE_MIN, angle_increment=ANGLE_INCREMENT,
                 async_planning=adapter.async_planning)
    announce()
    last_time = started
    done = None
    try:
        while sim.step(PERIOD_MS) != -1:
            now = sim.getTime()
            if now < last_time:
                # Supervisor API reset keeps this Python process alive. Close the
                # old Navigator, discard map/coverage/targets, and re-enable sensors.
                io.emergency_zero()
                epoch += 1
                adapter.new_episode(io.pose(), epoch)
                io.enable()
                started, done = now, None
                log.emit(event="EPISODE_RESET", epoch=epoch, sim_s=now,
                         closed_workers=adapter.closed_workers)
                announce()
                last_time = now
                continue  # first new sensor frame arrives only after a period
            last_time = now
            age = now-started
            fault = next((kind for begin, end, kind in config.get("faults", []) if begin <= age < end), None)
            stop = "EPISODE_LIMIT" if age >= config["limit_s"] else done
            obs = io.read(epoch, fault)
            result = adapter.step(obs, safety_stop=stop)
            if result["mission"]["state"] in ("FOUND", "COMPLETE"):
                done = result["mission"]["state"]
            if stop == "EPISODE_LIMIT":
                done = stop
            io.apply(result)
            result.update(event="CONTROL", episode_started_sim_s=started,
                          fault=fault, done=done, recognition_visible=bool(obs.recognition),
                          world_pose=list(obs.world_pose) if all(math.isfinite(v) for v in obs.world_pose) else None,
                          closed_workers=adapter.closed_workers)
            log.emit(**result)
            # Separate observer receives telemetry only, never supplies a target.
            sim.setCustomData(json.dumps(result, allow_nan=False))
    finally:
        io.emergency_zero()
        adapter.close()
        log.emit(event="CORE_CLOSED", epoch=epoch, closed_workers=adapter.closed_workers)
