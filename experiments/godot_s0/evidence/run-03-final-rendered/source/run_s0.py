"""Launch actual Godot, assert feasibility, and keep raw evidence. No third-party Python deps."""
import argparse
import base64
import ctypes
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import socket
import statistics
import struct
import subprocess
import sys
import time
import traceback
import zlib

from bridge import BridgeError, GodotBridge, red_detection

ROOT = Path(__file__).resolve().parent


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def identity():
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(ROOT.iterdir())
            if p.suffix in (".py", ".gd", ".godot", ".tscn")}


def protected_identity():
    repo = ROOT.parents[1]
    tracked = subprocess.check_output(["git", "ls-files", "-z"], cwd=repo).decode().split("\0")
    prefixes = ("experiments/webots_s0/", "experiments/webots_s2/", "test version/")
    # Include tracked evidence/worlds too, preserving pre-existing user edits as bytes.
    return {name: hashlib.sha256((repo / name).read_bytes()).hexdigest() for name in tracked
            if name and (name.startswith(prefixes) or ("/" not in name and name.endswith((".py", ".yaml", ".sh"))))
            and (repo / name).is_file()}


def host_memory():
    if os.name != "nt":
        return None
    class MemoryStatus(ctypes.Structure):
        _fields_ = [("length", ctypes.c_ulong), ("load", ctypes.c_ulong)] + [(name, ctypes.c_ulonglong) for name in
            ("total_physical", "available_physical", "total_pagefile", "available_pagefile", "total_virtual", "available_virtual", "extended_virtual")]
    value = MemoryStatus()
    value.length = ctypes.sizeof(value)
    if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(value)):
        return None
    return dict(total_physical_bytes=value.total_physical, available_physical_bytes=value.available_physical)


def png(path, camera):
    raw = base64.b64decode(camera["data"])
    width, height = camera["width"], camera["height"]
    def chunk(kind, content):
        return struct.pack(">I", len(content)) + kind + content + struct.pack(">I", zlib.crc32(kind + content))
    rows = b"".join(b"\0" + raw[y * width * 3:(y + 1) * width * 3] for y in range(height))
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">2I5B", width, height, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))


def windows_memory(pid):
    if os.name != "nt":
        return None
    class Counters(ctypes.Structure):
        _fields_ = [("cb", ctypes.c_ulong), ("PageFaultCount", ctypes.c_ulong)] + [
            (name, ctypes.c_size_t) for name in ("PeakWorkingSetSize", "WorkingSetSize", "QuotaPeakPagedPoolUsage",
            "QuotaPagedPoolUsage", "QuotaPeakNonPagedPoolUsage", "QuotaNonPagedPoolUsage", "PagefileUsage", "PeakPagefileUsage")]
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.OpenProcess.restype = ctypes.c_void_p
    kernel.OpenProcess.argtypes = [ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
    kernel.CloseHandle.argtypes = [ctypes.c_void_p]
    handle = kernel.OpenProcess(0x410, False, pid)
    if not handle:
        return None
    try:
        counter = Counters()
        counter.cb = ctypes.sizeof(counter)
        psapi = ctypes.WinDLL("psapi", use_last_error=True)
        psapi.GetProcessMemoryInfo.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong]
        if not psapi.GetProcessMemoryInfo(handle, ctypes.byref(counter), counter.cb):
            return None
        return dict(working_set_bytes=counter.WorkingSetSize, peak_working_set_bytes=counter.PeakWorkingSetSize)
    finally:
        kernel.CloseHandle(handle)


class Audit:
    def __init__(self, bridge, out, process, rendered):
        self.bridge, self.out, self.process, self.rendered = bridge, out, process, rendered
        self.log = (out / "observations.jsonl").open("w", encoding="utf-8")
        self.results = []
        self.case = "startup"
        self.latencies = []

    def call(self, method, *args, **kwargs):
        start = time.perf_counter()
        obs = getattr(self.bridge, method)(*args, **kwargs)
        elapsed = time.perf_counter() - start
        self.latencies.append(elapsed)
        record = json.loads(json.dumps(obs))
        data = record["camera"].pop("data")
        record["camera"]["sha256"] = hashlib.sha256(base64.b64decode(data)).hexdigest() if data else None
        record.update(case=self.case, request=dict(method=method, args=args, kwargs=kwargs), wall_seconds=elapsed)
        self.log.write(json.dumps(record, allow_nan=False) + "\n")
        self.log.flush()
        return obs

    def frame(self, name, obs):
        if obs["camera"]["valid"]:
            png(self.out / (name + ".png"), obs["camera"])

    def check(self, name, function):
        self.case = name
        start = time.perf_counter()
        try:
            metrics = function()
            result = dict(name=name, status="MEASURED" if name == "performance" else "PASS", metrics=metrics)
        except Exception as error:
            result = dict(name=name, status="FAIL", error=str(error), traceback=traceback.format_exc())
        result["wall_seconds"] = time.perf_counter() - start
        self.results.append(result)
        write_json(self.out / "results.json", self.results)
        print(name, result["status"], result.get("error", ""), flush=True)

    def clock_and_reset(self):
        a = self.call("reset", "calibration")
        time.sleep(0.25)
        b = self.call("observe")
        assert a["tick"] == b["tick"] == 0 and a["pose"] == b["pose"]
        c = self.call("step", .3, .1, .2, ticks=60)
        assert c["tick"] == 60 and c["simulation_timestamp"] == 1
        d = self.call("reset", "calibration")
        assert d["epoch"] == a["epoch"] + 1 and d["tick"] == 0
        assert d["pose"]["x"] == d["pose"]["y"] == d["pose"]["yaw"] == 0
        assert d["diagnostics"]["collision_count"] == 0
        assert all((x is None) == (y is None) for x, y in zip(a["scan"]["ranges"], d["scan"]["ranges"]))
        assert max(abs(x - y) for x, y in zip(a["scan"]["ranges"], d["scan"]["ranges"]) if x is not None) < 1e-5
        assert a["camera"]["valid"] == self.rendered
        if self.rendered:
            assert a["camera"]["data"] == d["camera"]["data"]
        return dict(idle_ticks=0, advanced_ticks=60, reset_epoch=d["epoch"], synchronized_stamps=True)

    def calibration(self):
        a = self.call("reset", "calibration")
        scan = a["scan"]
        errors = []
        for i, actual in enumerate(scan["ranges"]):
            angle = scan["angle_min"] + i * scan["angle_increment"]
            dx, dy = math.cos(angle), math.sin(angle)
            distances = [((4.95 if dx > 0 else -4.95) - .12) / dx,
                         (4.95 if dy > 0 else -4.95) / dy]
            expected = min(distances)
            if expected > 6:
                assert actual is None
            else:
                assert actual is not None
                errors.append(abs(actual - expected))
        assert max(errors) < .002, max(errors)
        actuator_errors = []
        for initial_yaw in (0, math.pi / 2):
            for vx, vy, wz in ((.3, 0, 0), (0, .2, 0), (0, 0, .4), (.3, .2, .4)):
                self.call("reset", "calibration")
                if initial_yaw:
                    self.call("step", wz=math.pi / 4, ticks=120)
                b = self.call("step", vx, vy, wz, ticks=60)
                ex = ey = 0.0
                for tick in range(1, 61):
                    angle = initial_yaw + wz * tick / 60
                    ex += (math.cos(angle) * vx - math.sin(angle) * vy) / 60
                    ey += (math.sin(angle) * vx + math.cos(angle) * vy) / 60
                error = math.hypot(b["pose"]["x"] - ex, b["pose"]["y"] - ey)
                actuator_errors.append(error)
                assert error < .002 and abs(b["pose"]["yaw"] - initial_yaw - wz) < .0001
        return dict(range_max_error_m=max(errors), finite_rays=len(errors), actuator_max_error_m=max(actuator_errors), trials=8)

    def collision(self):
        self.call("reset", "obstacle")
        a = self.call("step", .6, ticks=240)
        assert 1.20 < a["pose"]["x"] < 1.25, a["pose"]
        assert a["diagnostics"]["collision_count"] > 0
        b = self.call("step", ticks=60)
        assert a["pose"]["x"] == b["pose"]["x"]
        return dict(stop_x=a["pose"]["x"], collision_count=a["diagnostics"]["collision_count"], zero_hold_drift_m=0)

    def repeat(self):
        traces = []
        for _ in range(3):
            self.call("reset", "calibration", seed=7)
            trace = [self.call("step", *v, ticks=30) for v in ((.3, .1, .3), (.2, -.1, -.2), (0, 0, 0))]
            traces.append(trace)
        pose_error = scan_error = 0
        for other in traces[1:]:
            for a, b in zip(traces[0], other):
                assert a["tick"] == b["tick"]
                pose_error = max(pose_error, *(abs(a["pose"][key] - b["pose"][key]) for key in ("x", "y", "yaw")))
                for ra, rb in zip(a["scan"]["ranges"], b["scan"]["ranges"]):
                    assert (ra is None) == (rb is None)
                    if ra is not None:
                        scan_error = max(scan_error, abs(ra - rb))
                if self.rendered:
                    assert a["camera"]["data"] == b["camera"]["data"]
        assert pose_error < 1e-5 and scan_error < 1e-5
        return dict(episodes=3, max_pose_error=pose_error, max_scan_error=scan_error, rgb_identical=self.rendered)

    def protocol(self):
        a = self.call("reset", "calibration")
        rejected = []
        for fields in (dict(epoch=a["epoch"] - 1, velocity=[.1, 0, 0], ticks=1),
                       dict(epoch=a["epoch"], velocity=[99, 0, 0], ticks=1),
                       dict(epoch=a["epoch"], velocity=[0, 0, 0], ticks=1.5),
                       dict(epoch=a["epoch"], velocity=[0, 0], ticks=1)):
            try:
                self.bridge.request("step", **fields)
            except BridgeError as error:
                rejected.append(str(error))
            else:
                raise AssertionError("Invalid request accepted")
        b = self.call("observe")
        assert a["tick"] == b["tick"] and a["pose"] == b["pose"]
        self.call("step", .3, ticks=12)
        port = self.bridge.socket.getpeername()[1]
        self.bridge.close()
        time.sleep(.15)
        self.bridge = GodotBridge(port=port)
        c = self.call("observe")
        time.sleep(.15)
        d = self.call("observe")
        assert c["pose"] == d["pose"] and c["tick"] == d["tick"] == 12
        return dict(rejected=rejected, reconnect_pose_held=True)

    def visibility(self):
        a = self.call("reset", "occluded")
        self.frame("occluded-before", a)
        assert red_detection(a["camera"]) is None
        assert min(x for x in a["scan"]["ranges"] if x is not None) < 1.4
        self.call("step", vy=.4, ticks=300) # scripted 2m lateral reveal; not a planner
        b = self.call("step", wz=-.6, ticks=60)
        self.frame("occluded-after", b)
        detected = red_detection(b["camera"])
        assert detected is not None
        c = self.call("reset", "absent")
        self.frame("absent", c)
        assert red_detection(c["camera"]) is None
        assert b["diagnostics"]["collision_count"] == 0
        return dict(hidden_initial=True, revealed_rgb=detected, absent_false_detection=False, reveal="scripted 2m lateral maneuver")

    def mission(self):
        obs = self.call("reset", "clear")
        self.frame("mission-start", obs)
        assert red_detection(obs["camera"]) is None
        states = []
        confirmations = 0
        for iteration in range(400):
            detection = red_detection(obs["camera"])
            if detection is None:
                state, velocity = "SEARCH", (0, 0, .6)
                confirmations = 0
            else:
                confirmations += 1
                bearing = detection["bearing"]
                ranges = [r for i, r in enumerate(obs["scan"]["ranges"]) if r is not None
                          and abs(obs["scan"]["angle_min"] + i * obs["scan"]["angle_increment"] - bearing) < .08]
                nearest = min(ranges, default=6)
                if confirmations >= 2 and abs(bearing) < .08 and nearest <= .75:
                    states.append("STOP")
                    break
                state = "DETECT" if confirmations < 2 else "APPROACH"
                velocity = (.5 if confirmations >= 2 and abs(bearing) < .15 else 0, 0,
                            max(-.7, min(.7, 1.8 * bearing)))
            states.append(state)
            obs = self.call("step", *velocity, ticks=6)
        else:
            raise AssertionError("Mission exceeded 40 simulation seconds")
        self.frame("mission-stop", obs)
        distance = math.hypot(3 - obs["pose"]["x"], obs["pose"]["y"]) # observer ONLY
        assert .85 < distance < 1.2, distance
        assert obs["diagnostics"]["collision_count"] == 0
        assert all(state in states for state in ("SEARCH", "DETECT", "APPROACH", "STOP"))
        held = self.call("step", ticks=60)
        drift = math.hypot(held["pose"]["x"] - obs["pose"]["x"], held["pose"]["y"] - obs["pose"]["y"])
        assert drift < .001
        write_json(self.out / "mission-states.json", states)
        return dict(elapsed_sim_s=obs["simulation_timestamp"], states=sorted(set(states)),
                    distance_base_to_person_m=distance, hold_drift_m=drift, policy_steps=iteration,
                    perception="RGB red pixels + angularly matched range rays; no target pose input")

    def performance(self):
        self.call("reset", "calibration")
        samples = []
        started = time.perf_counter()
        for _ in range(100):
            before = time.perf_counter()
            obs = self.call("step", ticks=6)
            samples.append(dict(wall_s=time.perf_counter() - before, fps=obs["diagnostics"]["fps"],
                                engine_static_bytes=obs["diagnostics"]["static_memory_bytes"],
                                process_memory=windows_memory(self.process.pid)))
        wall = time.perf_counter() - started
        write_json(self.out / "performance-samples.json", samples)
        latencies = sorted(s["wall_s"] for s in samples)
        return dict(simulated_s=10, wall_s=wall, end_to_end_rtf=10 / wall,
                    request_median_ms=statistics.median(latencies) * 1000, request_p95_ms=latencies[94] * 1000,
                    reported_fps_min=min(s["fps"] for s in samples), reported_fps_max=max(s["fps"] for s in samples),
                    peak_working_set_bytes=max((s["process_memory"]["peak_working_set_bytes"] for s in samples if s["process_memory"]), default=None),
                    scope="100 lockstep requests, idle robot, full scan/RGB readback+JSON+logging; not standalone engine benchmark",
                    hardware_performance_acceptance="UNVERIFIED: short sample only, no matched Webots measurement or long soak")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--godot", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    with socket.socket() as port_socket:
        port_socket.bind(("127.0.0.1", 0))
        port = port_socket.getsockname()[1]
    cmd = [str(args.godot.resolve()), "--path", str(ROOT), "--audio-driver", "Dummy", "--log-file", str(out / "engine.log")]
    if args.headless:
        cmd += ["--headless"]
    cmd += ["--", f"--port={port}"]
    invocation = dict(started_utc=datetime.now(timezone.utc).isoformat(), command=cmd, python=sys.version,
                      python_executable=sys.executable, platform=platform.platform(), identity_before=identity(),
                      host_memory=host_memory(), protected_identity_before=protected_identity(),
                      rendered=not args.headless, hardware_claim="Windows 11 / Intel Iris Xe / 8 GB supplied by user; GPU verified in engine log when available")
    write_json(out / "invocation.json", invocation)
    (out / "source").mkdir()
    for filename in invocation["identity_before"]:
        shutil.copy2(ROOT / filename, out / "source" / filename)
    status = 1
    bridge = audit = None
    started = time.perf_counter()
    with (out / "stdout.txt").open("w", encoding="utf-8") as stdout, (out / "stderr.txt").open("w", encoding="utf-8") as stderr:
        process = subprocess.Popen(cmd, stdout=stdout, stderr=stderr)
        invocation["pid"] = process.pid
        write_json(out / "invocation.json", invocation)
        try:
            deadline = time.monotonic() + 45
            while True:
                if process.poll() is not None:
                    raise RuntimeError(f"Godot exited at startup: {process.returncode}")
                try:
                    bridge = GodotBridge(port=port)
                    break
                except OSError:
                    if time.monotonic() > deadline:
                        raise TimeoutError("Godot TCP startup timeout")
                    time.sleep(.1)
            audit = Audit(bridge, out, process, not args.headless)
            for name in ("clock_and_reset", "calibration", "collision", "repeat", "protocol"):
                audit.check(name, getattr(audit, name))
            if not args.headless:
                for name in ("visibility", "mission"):
                    audit.check(name, getattr(audit, name))
            else:
                audit.results += [dict(name=name, status="UNVERIFIED", reason="headless dummy renderer has no RGB") for name in ("visibility", "mission")]
            audit.check("performance", audit.performance)
            bridge = audit.bridge
            bridge.request("quit")
            process.wait(timeout=10)
            status = int(any(r["status"] == "FAIL" for r in audit.results) or process.returncode != 0)
        except Exception:
            invocation["error"] = traceback.format_exc()
            print(invocation["error"], flush=True)
        finally:
            if audit:
                audit.log.close()
                write_json(out / "results.json", audit.results)
            if bridge:
                bridge.close()
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            invocation.update(exit_code=process.returncode, runner_exit=status, wall_seconds=time.perf_counter() - started,
                              identity_after=identity(), protected_identity_after=protected_identity(), process_memory=windows_memory(process.pid))
            if invocation["identity_before"] != invocation["identity_after"] or invocation["protected_identity_before"] != invocation["protected_identity_after"]:
                status = invocation["runner_exit"] = 1
                invocation["identity_error"] = "Source or protected files changed during execution"
            write_json(out / "invocation.json", invocation)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
