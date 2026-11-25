"""ROS-free bridge to the repository's actual cores. No Webots/Supervisor imports.

The only target input is a camera recognition observation. Ideal robot pose is
explicit; hidden scene objects never enter this interface. Units: m, rad, sim s.
"""
from dataclasses import dataclass
import math
from pathlib import Path
import sys

import numpy as np

CORE_DIR = Path(__file__).resolve().parents[2] / "test version"
if not CORE_DIR.is_dir():
    # Disposable Webots projects keep a pointer to the original, unmodified core.
    CORE_DIR = Path((Path(__file__).parent / "core-path.txt").read_text(encoding="utf-8").strip())
sys.path.insert(0, str(CORE_DIR))
from collision_monitor_ros1 import Config, Monitor, scan_points
from navigation_core_ros1 import NavConfig, Navigator
from search_core_ros1 import SearchConfig, SearchMission

ZERO = (0., 0., 0.)
PERIOD_MS = 96
RAYS = 192  # even pixels per cube face avoid the 180-ray cylindrical seam dropouts
FOV = 2 * math.pi
ANGLE_INCREMENT = FOV / RAYS
ANGLE_MIN = -FOV/2 + ANGLE_INCREMENT/2
CAMERA_X = .12


def wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*x-s*y, s*x+c*y


def odom_pose(world, origin):
    x, y = rotate(world[0]-origin[0], world[1]-origin[1], -origin[2])
    return (x, y, wrap(world[2]-origin[2]))


def camera_target(camera_xyz, pose):
    if len(camera_xyz) != 3 or not all(math.isfinite(v) for v in camera_xyz):
        raise ValueError("INVALID_TARGET_VECTOR")
    x, y, _ = camera_xyz
    if x <= 0:
        raise ValueError("TARGET_BEHIND_CAMERA")
    # Webots filters visibility using object geometry, not just its centre.
    # A partly visible object's centre can lie outside the image FOV. Reapplying
    # a centre-only FOV test here would freeze valid approaches near an edge.
    dx, dy = rotate(x+CAMERA_X, y, pose[2])
    return (pose[0]+dx, pose[1]+dy)


def normalize_scan(ranges):
    values = np.asarray(ranges, dtype=float)
    if values.shape != (RAYS,):
        raise ValueError("SCAN_DIMENSIONS")
    # Fail closed on corrupt rays; +Inf remains an observed no-return.
    if not np.all((np.isfinite(values) & (values >= .05) & (values <= 6)) | np.isposinf(values)):
        raise ValueError("INVALID_SCAN_RAY")
    return values[::-1].copy()


def actuator_limit(command):
    """Common scaling preserves the body Twist; no component (including vy) is dropped."""
    if len(command) != 3 or not all(math.isfinite(v) for v in command):
        raise ValueError("INVALID_ACTUATOR_COMMAND")
    vx, vy, wz = command
    scale = min(1., (.18 if vx >= 0 else .06)/max(abs(vx), 1e-12),
                .12/max(abs(vy), 1e-12), (math.pi/6)/max(abs(wz), 1e-12))
    return tuple(float(v*scale) for v in command)


def world_velocity(command, world_yaw):
    x, y = rotate(command[0], command[1], world_yaw)
    return (x, y, 0., 0., 0., command[2])


@dataclass(frozen=True)
class Observation:
    epoch: int
    now: float
    world_pose: tuple
    pose_stamp: float
    ranges: tuple
    scan_stamp: float
    camera_valid: bool
    camera_stamp: float
    # (recognition ID, camera-frame xyz). No scene target/world coordinates.
    recognition: tuple = ()


class CoreAdapter:
    def __init__(self, origin, epoch=0, async_planning=False):
        self.async_planning = async_planning
        self.closed_workers = 0
        self.nav = None
        self.new_episode(origin, epoch)

    def close(self):
        if self.nav is not None:
            self.nav.close()  # shutdown(wait=True): no stale worker crosses an episode.
            self.closed_workers += int(self.nav.executor is not None)
            self.nav = None

    def new_episode(self, origin, epoch):
        self.close()
        if len(origin) != 3 or not all(math.isfinite(v) for v in origin):
            raise ValueError("INVALID_ORIGIN")
        self.origin, self.epoch = tuple(origin), epoch
        self.cc = Config(body_length=.4, body_width=.4, scan_x=.12, scan_y=0,
                         scan_yaw=0, blind_sectors_deg=[], rate=1000/PERIOD_MS,
                         min_valid_fraction=1.)
        self.nav = Navigator(NavConfig(control_hz=1000/PERIOD_MS), self.cc,
                             async_planning=self.async_planning)
        self.mission = SearchMission(self.nav, SearchConfig())
        self.monitor = Monitor(self.cc)
        self.last_now = None
        self.last_stamps = dict(pose=None, scan=None, camera=None)
        self.confirmed_id, self.confirmed_count = None, 0
        self.last_camera_stamp = None
        self.policy_steps = self.navigation_steps = self.monitor_steps = 0
        navigation_step = self.nav.step

        def counted_navigation(*args, **kwargs):
            self.navigation_steps += 1
            return navigation_step(*args, **kwargs)

        self.nav.step = counted_navigation

    def _validate(self, obs):
        if obs.epoch != self.epoch:
            raise ValueError("EPOCH_MISMATCH")
        if not math.isfinite(obs.now) or obs.now < 0:
            raise ValueError("INVALID_CLOCK")
        if self.last_now is not None and obs.now <= self.last_now:
            raise ValueError("CLOCK_ROLLBACK_OR_DUPLICATE")
        if len(obs.world_pose) != 3 or not all(math.isfinite(v) for v in obs.world_pose):
            raise ValueError("INVALID_POSE")
        for name, stamp, timeout in (("pose", obs.pose_stamp, .5), ("scan", obs.scan_stamp, .5),
                                      ("camera", obs.camera_stamp, .5)):
            if not math.isfinite(stamp) or not 0 <= obs.now-stamp <= timeout:
                raise ValueError("STALE_OR_FUTURE_" + name.upper())
            previous = self.last_stamps[name]
            if previous is not None and stamp < previous:
                raise ValueError("NONMONOTONIC_" + name.upper())
        # All devices use the same sampling tick in this fixture. Do not attach
        # current pose to a held/older camera or LiDAR buffer while still moving.
        if max(obs.pose_stamp, obs.scan_stamp, obs.camera_stamp) - min(
                obs.pose_stamp, obs.scan_stamp, obs.camera_stamp) > 1e-8:
            raise ValueError("UNMATCHED_SENSOR_POSE_STAMPS")
        if not obs.camera_valid:
            raise ValueError("INVALID_CAMERA")
        if all(self.last_stamps[name] == getattr(obs, name+"_stamp") for name in self.last_stamps):
            raise ValueError("NO_NEW_SENSOR_SAMPLE")
        return normalize_scan(obs.ranges)

    def _target(self, obs, pose):
        if self.last_camera_stamp == obs.camera_stamp:
            return None  # a repeated buffer cannot confirm another detection
        self.last_camera_stamp = obs.camera_stamp
        if not obs.recognition:
            self.confirmed_id, self.confirmed_count = None, 0
            return None
        identity, xyz = obs.recognition[0]
        point = camera_target(xyz, pose)
        self.confirmed_count = self.confirmed_count+1 if identity == self.confirmed_id else 1
        self.confirmed_id = identity
        return point if self.confirmed_count >= 2 else None

    def step(self, obs, test_command=None, safety_stop=None):
        """test_command is calibration-only; mission runs use None exclusively."""
        raw = limited = checked = applied = ZERO
        reason, state, target = "", "IDLE", None
        pose = None
        try:
            scan = self._validate(obs)
            pose = odom_pose(obs.world_pose, self.origin)
            target = self._target(obs, pose)
            if self.last_stamps["scan"] != obs.scan_stamp:
                points = scan_points(scan, ANGLE_MIN, ANGLE_INCREMENT, .05, 6., self.cc)
                self.monitor.set_scan(points, obs.scan_stamp)
                self.nav.costmap.update(scan, ANGLE_MIN, ANGLE_INCREMENT, .05, 6., pose, obs.scan_stamp)
                self.mission.update_coverage(scan, ANGLE_MIN, ANGLE_INCREMENT, .05, pose)
            self.nav.grid = self.nav.costmap.snapshot(pose, obs.now)
            dt = PERIOD_MS/1000 if self.last_now is None else obs.now-self.last_now
            if dt > .5:
                raise ValueError("CONTROL_GAP")
            if safety_stop:
                raw, reason = ZERO, safety_stop
            elif test_command is None:
                raw, reason = self.mission.step(pose, target, .55, obs.now, dt)
                self.policy_steps += 1
            else:
                raw, reason = tuple(test_command), "CALIBRATION_COMMAND"
            limited = actuator_limit(raw)
            self.monitor.set_command(*limited, now=obs.now)
            checked, state = self.monitor.evaluate(obs.now)
            self.monitor_steps += 1
            applied = actuator_limit(checked)
            if applied != checked:
                self.monitor.set_command(*applied, now=obs.now)
                applied, state = self.monitor.evaluate(obs.now)
                self.monitor_steps += 1
            for name in self.last_stamps:
                self.last_stamps[name] = getattr(obs, name+"_stamp")
        except (ValueError, TypeError, OverflowError) as exc:
            reason = str(exc)
            self.confirmed_id, self.confirmed_count = None, 0
            self.monitor.invalidate_scan(reason)
            self.monitor.set_command(*ZERO, now=obs.now)
            checked, state = self.monitor.evaluate(obs.now)
            self.monitor_steps += 1
            applied = actuator_limit(checked)
            self.nav.previous_velocity[:] = 0
        if math.isfinite(obs.now):
            self.last_now = obs.now
        return dict(epoch=self.epoch, sim_s=obs.now, pose=pose, raw=list(raw), limited=list(limited),
                    checked=list(checked), applied=list(applied), reason=reason, monitor=state,
                    target=list(target) if target is not None else None,
                    target_source="webots_recognition_oracle", pose_source="supervisor_ideal_pose",
                    mission=self.mission.info(), policy_steps=self.policy_steps,
                    navigation_steps=self.navigation_steps, monitor_steps=self.monitor_steps,
                    stamps=dict(pose=obs.pose_stamp, scan=obs.scan_stamp, camera=obs.camera_stamp))
