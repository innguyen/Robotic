"""Static check or actual Webots S1 run. Standard library only; no production imports."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

from s1_fixture import BASE, OCCLUSION, ROOT, make_occlusion, validate_files


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_webots(exe, world, out, label, env, timeout):
    command = [str(exe), "--batch", "--minimize", "--no-rendering", "--mode=fast",
               "--stdout", "--stderr", str(world)]
    metadata = dict(command=command, started_utc=datetime.now(timezone.utc).isoformat(),
                    python=sys.executable, python_version=sys.version, main_view_rendering=False,
                    sensor_rendering=True, performance_measurement=False,
                    runtime_world_sha256=sha(world),
                    controller_sha256={p.relative_to(world.parent.parent).as_posix(): sha(p)
                                       for p in (world.parent.parent / "controllers").rglob("*.py")})
    (out / f"{label}-invocation.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"Running {label}: {world}", flush=True)
    with (out / f"{label}-stdout.txt").open("w", encoding="utf-8") as stdout, \
            (out / f"{label}-stderr.txt").open("w", encoding="utf-8") as stderr:
        started = time.perf_counter()
        process = subprocess.Popen(command, stdout=stdout, stderr=stderr, env=env)
        metadata["pid"] = process.pid
        try:
            returncode = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            metadata["timed_out"] = True
            # Kill only this launcher's tree; never match all Webots processes.
            if os.name == "nt":
                cleanup = subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                                         capture_output=True, text=True)
                metadata["timeout_cleanup"] = cleanup.stdout + cleanup.stderr
            else:
                process.kill()
            returncode = process.wait()
        metadata.update(returncode=returncode, wall_elapsed_s=time.perf_counter()-started,
                        ended_utc=datetime.now(timezone.utc).isoformat())
        (out / f"{label}-invocation.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    stdout_text = (out / f"{label}-stdout.txt").read_text(encoding="utf-8")
    stderr_text = (out / f"{label}-stderr.txt").read_text(encoding="utf-8")
    marker = "REPEAT_SUITE_PASS" if label == "repeat" else "OCCLUSION_SUITE_PASS"
    # Windows launcher output forwarding may be empty when no console is attached.
    # Controller-owned append-only JSONL is the authoritative raw evidence.
    log_path = out / ("repeat-supervisor.jsonl" if label == "repeat" else "occlusion-controller.jsonl")
    controller_text = log_path.read_text(encoding="utf-8") if log_path.exists() else ""
    if returncode != 0 or marker not in controller_text or "AUDIT_FAIL" in stdout_text + stderr_text + controller_text:
        raise RuntimeError(f"{label} failed (exit {returncode}); inspect raw logs in {out}")
    print(f"PASS {label} (Webots exit {returncode})", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write-occlusion", action="store_true")
    parser.add_argument("--real", action="store_true", help="Actually launch installed Webots")
    parser.add_argument("--webots", type=Path, default=Path("C:/Program Files/Webots/msys64/mingw64/bin/webots.exe"))
    parser.add_argument("--evidence", type=Path)
    parser.add_argument("--timeout", type=int, default=180, help="Wall seconds per Webots invocation")
    args = parser.parse_args()
    if args.write_occlusion:
        OCCLUSION.write_text(make_occlusion(BASE.read_text(encoding="utf-8")), encoding="utf-8", newline="\n")
    validate_files()
    print("PASS static fixtures, initial state and sensor contract", flush=True)
    if not args.real:
        return
    if args.evidence is None:
        parser.error("--real requires --evidence (a new output directory)")
    out = args.evidence.resolve()
    out.mkdir(parents=True, exist_ok=False)  # Never overwrite earlier evidence.
    version = subprocess.run([str(args.webots), "--version"], capture_output=True, text=True, check=True)
    (out / "webots-version.txt").write_text(version.stdout + version.stderr, encoding="utf-8")
    if "R2025a" not in version.stdout + version.stderr:
        raise RuntimeError("S1 is pinned to R2025a")
    before = {str(p.relative_to(ROOT)): sha(p) for p in (BASE, OCCLUSION)}
    runtime_root = ROOT / ".s1-runs"
    runtime_root.mkdir(exist_ok=True)
    # Keep runtime project for failed-run diagnosis. It is ignored by Git.
    project = Path(tempfile.mkdtemp(prefix="audit-", dir=runtime_root))
    shutil.copytree(ROOT / "controllers", project / "controllers", ignore=shutil.ignore_patterns("__pycache__"))
    (project / "worlds").mkdir()
    state = out / "repeat-result.json"
    state.write_text('{"trials": []}\n', encoding="utf-8")
    base = BASE.read_text(encoding="utf-8").replace('controller "s0_smoke"',
        'controller "s0_smoke"\n  controllerArgs ["--audit" "--audit-log" ' + json.dumps((out / "repeat-smoke.jsonl").as_posix()) + ']', 1)
    base += '\nRobot {\n  supervisor TRUE\n  controller "s1_repeat"\n  controllerArgs [' + json.dumps(state.as_posix()) + ']\n}\n'
    repeat_world = project / "worlds/s1_repeat.wbt"
    repeat_world.write_text(base, encoding="utf-8")
    occlusion_world = project / "worlds/s1_occlusion.wbt"
    occlusion_world.write_text(OCCLUSION.read_text(encoding="utf-8").replace(
        'controller "s1_occlusion"', 'controller "s1_occlusion"\n  controllerArgs [' + json.dumps(out.as_posix()) + ']', 1), encoding="utf-8")
    env = os.environ.copy()
    env["PATH"] = str(Path(sys.executable).parent) + os.pathsep + env.get("PATH", "")
    env["PYTHONIOENCODING"] = "utf-8"
    try:
        run_webots(args.webots, repeat_world, out, "repeat", env, args.timeout)
        run_webots(args.webots, occlusion_world, out, "occlusion", env, args.timeout)
    finally:
        after = {str(p.relative_to(ROOT)): sha(p) for p in (BASE, OCCLUSION)}
        (out / "fixture-hashes.json").write_text(json.dumps(dict(before=before, after=after), indent=2) + "\n", encoding="utf-8")
        if before != after:
            raise RuntimeError("Canonical fixture was modified during the run")
    print(f"S1 actual Webots checks passed; evidence: {out}", flush=True)


if __name__ == "__main__":
    main()
