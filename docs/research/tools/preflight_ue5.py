"""Read-only UE discovery; never launches an engine or connects to hardware.

Run from any directory with --out pointing to a NEW JSON file. An installed
editor is only a discovery result, never a rendered-sensor acceptance result.
"""
import argparse
import ctypes
import datetime
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[3]


def command(args):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=20)
        return {"args": args, "returncode": result.returncode,
                "stdout": result.stdout.strip(), "stderr": result.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"args": args, "error": str(error)}


def probe_path(path):
    try:
        # stat distinguishes inaccessible from not found; exists() may hide it.
        Path(path).stat()
        return {"path": str(path), "status": "found"}
    except FileNotFoundError:
        return {"path": str(path), "status": "not_found"}
    except OSError as error:
        return {"path": str(path), "status": "unverified", "error": str(error)}


def memory():
    if os.name != "nt":
        return {"status": "unverified", "reason": "Windows memory probe only"}

    class MemoryStatus(ctypes.Structure):
        _fields_ = [("length", ctypes.c_uint32), ("load", ctypes.c_uint32)] + [
            (name, ctypes.c_uint64) for name in (
                "total_physical", "available_physical", "total_pagefile",
                "available_pagefile", "total_virtual", "available_virtual",
                "available_extended_virtual")]

    data = MemoryStatus()
    data.length = ctypes.sizeof(data)
    try:
        if not ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(data)):
            return {"status": "unverified", "error": "GlobalMemoryStatusEx failed"}
        return {"status": "observed_local", "total_physical_bytes": data.total_physical,
                "available_physical_bytes": data.available_physical}
    except (OSError, AttributeError) as error:
        return {"status": "unverified", "error": str(error)}


def collect():
    names = ("UnrealEditor", "UnrealEditor-Cmd", "nvidia-smi")
    programs = {name: shutil.which(name) for name in names}
    env = {key: os.environ.get(key) for key in ("UE_ROOT", "UE_ENGINE_ROOT", "UNREAL_ENGINE")}
    paths = ["C:/Program Files/Epic Games", "C:/Epic Games", "C:/UnrealEngine",
             "D:/Epic Games", "C:/ProgramData/Epic/UnrealEngineLauncher/LauncherInstalled.dat"]
    paths.extend(value for value in env.values() if value)
    probes = [probe_path(path) for path in paths]
    editors = []
    for probe in probes:
        if probe["status"] != "found":
            continue
        base = Path(probe["path"])
        if not base.is_dir():
            continue
        try:
            # Bounded discovery: an engine root or its immediate UE_* children.
            candidates = [base] + list(base.glob("UE_*"))
            for candidate in candidates:
                for name in ("UnrealEditor.exe", "UnrealEditor-Cmd.exe"):
                    item = probe_path(candidate / "Engine/Binaries/Win64" / name)
                    if item["status"] != "not_found":
                        editors.append(item)
        except OSError as error:
            editors.append({"path": str(base), "status": "unverified", "error": str(error)})
    data = {
        "recorded_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "user_date": datetime.datetime.now(
            datetime.timezone(datetime.timedelta(hours=7))).date().isoformat(),
        "user_timezone": "Asia/Saigon",
        "platform": platform.platform(), "python": sys.version,
        "python_executable": sys.executable,
        "discovery_scope": "PATH, three UE environment variables, common paths, tracked uprojects",
        "programs": programs, "engine_environment": env, "paths": probes,
        "editor_candidates": editors, "memory": memory(),
        "gpu": {"status": "unverified", "graphics_context_test": "not_run"},
        "rendered_camera_depth_control_reset_evaluator": "UNVERIFIED_NOT_RUN",
    }
    powershell = shutil.which("pwsh") or shutil.which("powershell")
    if powershell:
        data["gpu"]["cim_probe"] = command([
            powershell, "-NoProfile", "-NonInteractive", "-Command",
            "$ErrorActionPreference='Stop'; Get-CimInstance Win32_VideoController | "
            "Select-Object Name,DriverVersion,AdapterRAM | ConvertTo-Json -Compress"])
    if programs["nvidia-smi"]:
        data["gpu"]["nvidia_probe"] = command([
            programs["nvidia-smi"], "--query-gpu=name,driver_version,memory.total",
            "--format=csv,noheader"])
    tracked = command(["git", "-C", str(ROOT), "ls-files", "--", "*.uproject"])
    data["tracked_uproject_probe"] = tracked
    data["engine_discovery"] = (
        "candidate_found_runtime_unverified"
        if any(programs.get(name) for name in ("UnrealEditor", "UnrealEditor-Cmd"))
        or any(item["status"] == "found" for item in editors)
        else "not_detected_in_checked_locations")
    prior = ROOT / "experiments/godot_s0/evidence/run-05-final-rendered/invocation.json"
    if prior.is_file():
        content = json.loads(prior.read_text(encoding="utf-8"))
        data["historical_godot_evidence"] = {
            "status": "reported_prior_not_live_gpu_probe", "path": str(prior.relative_to(ROOT)),
            "started_utc": content.get("started_utc"),
            "host_memory": content.get("host_memory"),
            "gpu_source": "experiments/godot_s0/evidence/run-05-final-rendered/stdout.txt"}
    return data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        parser.error("Output already exists; use a new evidence filename")
    result = collect()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(result["engine_discovery"])
    print("Rendered UE5 vertical slice: UNVERIFIED_NOT_RUN")
    print(f"Evidence: {args.out}")
