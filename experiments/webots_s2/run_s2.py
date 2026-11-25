"""Generate/check S2 fixtures or run actual Webots, preserving every invocation's evidence."""
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

from fixtures import ROOT, SCENARIOS, world

CORES = ["search_core_ros1.py", "navigation_core_ros1.py", "collision_monitor_ros1.py"]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_identity():
    repo = ROOT.parents[1]
    files = list(ROOT.glob("*.py")) + list((ROOT / "controllers").rglob("*.py"))
    files += [ROOT / "scenarios.json", ROOT.parent / "webots_s0/worlds/s0_search.wbt"]
    files += [repo / "test version" / name for name in CORES]
    return {p.relative_to(repo).as_posix(): digest(p) for p in files}


def run_case(name, out, exe, timeout, async_planning):
    out.mkdir()
    runtime = ROOT / ".runs"
    runtime.mkdir(exist_ok=True)
    project = Path(tempfile.mkdtemp(prefix=name+"-", dir=runtime))
    for filename in ("adapter.py", "runtime.py", "geometry.py"):
        shutil.copy2(ROOT / filename, project / filename)
    shutil.copytree(ROOT / "controllers", project / "controllers", ignore=shutil.ignore_patterns("__pycache__"))
    (project / "core-path.txt").write_text(str(ROOT.parents[1] / "test version"), encoding="utf-8")
    (project / "worlds").mkdir()
    scenario = SCENARIOS[name]
    # Policy configuration intentionally contains no target/obstacle ground truth.
    policy_config = project / "policy.json"
    policy = dict(log=(out / "control.jsonl").as_posix(), limit_s=scenario["limit_s"],
                  faults=scenario.get("faults", []), epoch=0, async_planning=async_planning)
    policy_config.write_text(json.dumps(policy, indent=2), encoding="utf-8")
    audit_config = project / "audit.json"
    audit = dict(out=out.as_posix(), name=name, scenario=scenario, policy_config=policy_config.as_posix())
    audit_config.write_text(json.dumps(audit, indent=2), encoding="utf-8")
    calibration_config = project / "calibration.json"
    calibration_config.write_text(json.dumps(dict(out=out.as_posix())), encoding="utf-8")
    text = world(name, [calibration_config.as_posix()] if name == "calibration" else [policy_config.as_posix()],
                 [] if name == "calibration" else [audit_config.as_posix()])
    fixture = project / "worlds" / (name+".wbt")
    fixture.write_text(text, encoding="utf-8", newline="\n")
    shutil.copy2(fixture, out / "fixture.wbt")
    (out / "policy-config.json").write_text(json.dumps(policy, indent=2), encoding="utf-8")
    (out / "scenario.json").write_text(json.dumps(scenario, indent=2), encoding="utf-8")
    command = [str(exe), "--batch", "--minimize", "--no-rendering", "--mode=fast", "--stdout", "--stderr", str(fixture)]
    invocation = dict(command=command, started_utc=datetime.now(timezone.utc).isoformat(),
                      fixture_sha256=digest(fixture), identity=source_identity(),
                      python=sys.version, executable=sys.executable,
                      async_planning=async_planning, model="ideal_holonomic_velocity_rigid_body",
                      sensor_rendering=True, performance_measurement=False)
    path = out / "invocation.json"
    path.write_text(json.dumps(invocation, indent=2), encoding="utf-8")
    env = os.environ.copy()
    env["PATH"] = str(Path(sys.executable).parent) + os.pathsep + env.get("PATH", "")
    env["PYTHONIOENCODING"] = "utf-8"
    print("Running Webots: " + name, flush=True)
    started = time.perf_counter()
    with (out / "stdout.txt").open("w", encoding="utf-8") as stdout, (out / "stderr.txt").open("w", encoding="utf-8") as stderr:
        process = subprocess.Popen(command, stdout=stdout, stderr=stderr, env=env)
        invocation["pid"] = process.pid
        path.write_text(json.dumps(invocation, indent=2), encoding="utf-8")
        try:
            code = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            invocation["timeout"] = True
            if os.name == "nt":
                cleanup = subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], capture_output=True, text=True)
                invocation["cleanup"] = cleanup.stdout+cleanup.stderr
                if cleanup.returncode and process.poll() is None:
                    raise RuntimeError("Failed to stop this Webots process tree")
            else:
                process.kill()
            code = process.wait(timeout=10)
    invocation.update(returncode=code, ended_utc=datetime.now(timezone.utc).isoformat(),
                      wall_elapsed_s=time.perf_counter()-started, identity_after=source_identity())
    log = out / ("calibration.jsonl" if name == "calibration" else "audit.jsonl")
    records = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()] if log.exists() else []
    passed = code == 0 and any(r["event"] == "SUITE_PASS" for r in records)
    passed &= invocation["identity"] == invocation["identity_after"]
    invocation["passed"] = passed
    path.write_text(json.dumps(invocation, indent=2) + "\n", encoding="utf-8")
    print(("PASS " if passed else "FAIL ") + name + " (exit " + str(code) + ")", flush=True)
    return passed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real", action="store_true")
    parser.add_argument("--evidence", type=Path)
    parser.add_argument("--webots", type=Path, default=Path("C:/Program Files/Webots/msys64/mingw64/bin/webots.exe"))
    parser.add_argument("--scenarios", nargs="+", choices=list(SCENARIOS), default=list(SCENARIOS))
    parser.add_argument("--timeout", type=int, default=180)
    parser.add_argument("--async-planning", action="store_true")
    parser.add_argument("--write-world", action="store_true")
    args = parser.parse_args()
    if args.write_world:
        (ROOT / "worlds").mkdir(exist_ok=True)
        (ROOT / "worlds/s2_search.wbt").write_text(world("found"), encoding="utf-8", newline="\n")
    if (ROOT / "worlds/s2_search.wbt").read_text(encoding="utf-8") != world("found"):
        raise ValueError("Canonical S2 world differs; regenerate explicitly with --write-world")
    if not args.real:
        print("PASS canonical S2 fixture; use --real to execute Webots")
        return
    if args.evidence is None:
        parser.error("--real requires a new --evidence directory")
    out = args.evidence.resolve()
    out.mkdir(parents=True, exist_ok=False)
    version = subprocess.run([str(args.webots), "--version"], capture_output=True, text=True, check=True)
    (out / "webots-version.txt").write_text(version.stdout+version.stderr, encoding="utf-8")
    if "R2025a" not in version.stdout+version.stderr:
        raise ValueError("Webots R2025a required")
    results = {name: run_case(name, out / name, args.webots, args.timeout, args.async_planning) for name in args.scenarios}
    (out / "summary.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    if not all(results.values()):
        raise SystemExit("Some actual Webots cases failed; inspect preserved raw logs")


if __name__ == "__main__":
    main()
