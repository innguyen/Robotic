"""Reproducible local UE pilot runner. Use a NEW output directory each time."""
import argparse
import datetime
import gzip
import json
import os
from pathlib import Path
import queue
import secrets
import shutil
import socket
import subprocess
import sys
import threading
import time
import traceback

from . import PROTOCOL
from .artifacts import RunStore, digest, write_json
from .contracts import ContractError, canonical, strict_loads
from .episode import Episode, Phase
from .evaluator import evaluate, reference_match
from .scenarios import scenario, sar_scenario
from .statistics import rq1
from .transport import HostClient
from .ue_backend import UnrealBackend

ROOT = Path(__file__).resolve().parents[1]


class PolicyProcess:
    def __init__(self, method, seed, source_root=ROOT, model=None, confidence=.4):
        # The token, engine endpoint, scenario and GT paths are never forwarded.
        env = {key: value for key, value in os.environ.items() if not key.startswith("SAR_")}
        command = [sys.executable, "-m", "sar_benchmark.agent_worker", "--method", method, "--seed", str(seed)]
        if model: command += ["--model", str(model), "--confidence", str(confidence)]
        self.process = subprocess.Popen(command, cwd=source_root, env=env,
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.responses = queue.Queue()
        def read():
            try:
                while line := self.process.stdout.readline(65537):
                    self.responses.put(line)
            finally:
                self.responses.put(None)
        self.reader = threading.Thread(target=read, daemon=True)
        self.reader.start()

    def act(self, obs):
        self.process.stdin.write(canonical(obs) + b"\n")
        self.process.stdin.flush()
        try:
            line = self.responses.get(timeout=30)
        except queue.Empty as exc:
            raise TimeoutError("agent action watchdog") from exc
        if line is None:
            raise RuntimeError("agent worker exited")
        if len(line) > 65536 or not line.endswith(b"\n"):
            raise ContractError("oversized/unparseable agent message")
        return strict_loads(line)

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        self.reader.join(timeout=2)
        stderr = self.process.stderr.read().decode("utf-8", errors="replace")
        self.process.stdout.close()
        self.process.stderr.close()
        return {"exit_code": self.process.returncode, "stderr": stderr}


def source_hashes():
    paths = sorted((ROOT / "sar_benchmark").glob("*.py")) + sorted((ROOT / "experiments/ue5_s4").glob("*.py"))
    paths += sorted((ROOT / "experiments/ue5_s4/editor_host/Config").glob("*.ini"))
    paths += sorted((ROOT / "experiments/ue5_s4/editor_host").glob("*.uproject"))
    paths += [ROOT / "experiments/webots_s2/geometry.py"]
    paths += [ROOT / "test version" / name for name in
              ("collision_monitor_ros1.py", "navigation_core_ros1.py", "search_core_ros1.py")]
    return {str(path.relative_to(ROOT)).replace("\\", "/"): digest(path) for path in paths}


def schedule(suite, seeds, environments=("indoor", "outdoor"), cases=("clean", "occluded", "absent", "multi", "obstacles", "blocked"), postures=("standing",)):
    items = []
    for seed in range(seeds):
        if suite.startswith("sar"):
            for environment in environments:
                for posture in postures:
                    for kind in cases:
                        items.append(sar_scenario(kind, seed, environment=environment, posture=posture))
                    if suite == "sar-pilot":
                        for p, n in ((1,0),(0,1),(1,1)):
                            items.append(sar_scenario("clean",seed,p,n,environment,posture))
            continue
        if suite == "smoke":
            items.append(scenario("clean", seed))
        else:
            for p, n in ((0, 0), (1, 0), (0, 1), (1, 1)):
                items.append(scenario("clean", seed, p, n))
            items += [scenario(kind, seed) for kind in ("occluded", "absent", "multi", "sensor_black")]
    return items


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--editor", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--suite", choices=("smoke", "pilot", "sar-smoke", "sar-pilot"), default="smoke")
    parser.add_argument("--environments", nargs="+", choices=("indoor","outdoor"), default=["indoor","outdoor"])
    parser.add_argument("--cases", nargs="+", choices=("clean","occluded","absent","multi","obstacles","blocked"), default=["clean","occluded","absent","multi","obstacles","blocked"])
    parser.add_argument("--postures", nargs="+", choices=("standing","seated","prone"), default=["standing"])
    parser.add_argument("--size", default="320x240", choices=("160x120","320x240","640x480"))
    parser.add_argument("--model", type=Path)
    parser.add_argument("--confidence", type=float, default=.4)
    parser.add_argument("--ticks", type=int, default=30)
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--methods", nargs="+", choices=("random", "sweep", "core"), default=["random", "sweep"])
    parser.add_argument("--calibrate", action="store_true", help="also run privileged static engineering checks")
    args = parser.parse_args()
    human = args.suite.startswith("sar")
    if human:
        args.model = (args.model or ROOT / "experiments/ue5_s4/assets/source/yolov5n-v7.onnx").resolve()
        if not args.model.is_file(): parser.error("run prepare_assets.py first")
    if not args.editor.is_file() or not 2 <= args.ticks <= 3000 or not 1 <= args.seeds <= 10:
        parser.error("invalid editor, ticks or seed count")
    store = RunStore(args.out.resolve())
    snapshot = store.path / "source"
    hashes = source_hashes()
    for relative in hashes:
        destination = snapshot / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / relative, destination)
    rows = []
    engine = client = None
    raw_stream = public_stream = None
    token = secrets.token_hex(32)
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    env = dict(os.environ)
    env.update(SAR_HOST_TOKEN=token, SAR_HOST_PORT=str(port), SAR_HOST_READY=str(store.path / "host-ready.json"))
    if human:
        env.update(SAR_PROFILE="human", SAR_ASSET_ROOT=str(ROOT / "experiments/ue5_s4/assets"), SAR_RGB_SIZE=args.size)
    command = [str(args.editor.resolve()), str(ROOT / "experiments/ue5_s4/editor_host/SarEditorSlice.uproject"),
               "-unattended", "-nosplash", "-nosound", "-d3d11", "-AllowCommandletRendering",
               "-ExecutePythonScript=" + str(snapshot / "experiments/ue5_s4/editor_server.py"),
               "-abslog=" + str(store.path / "engine.log")]
    manifest = {"protocol": PROTOCOL, "track": "diagnostic_primitive_fixture_not_main_research", "date_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "timezone": "Asia/Saigon", "command": command, "editor_sha256": digest(args.editor),
                "python": sys.version, "source_sha256": hashes, "suite": args.suite, "ticks": args.ticks,
                "seeds": args.seeds, "methods": args.methods, "threshold": .5, "max_reports": 5,
                "rgbd_size": [80, 60], "detector": "synthetic_red_components_no_person_model",
                "sensor_model": "synthetic", "odometry": "corrupted_GT_self_motion_increments_not_wheel_encoders",
                "isolation": "honest_local_policy_processes_only_NOT_hostile_agent_sandbox",
                "clock": "static_kinematic_transactions_not_Chaos_lockstep", "calibrate": args.calibrate, "status": "STARTED"}
    manifest["scientific_main_eligible"] = False
    if human:
        manifest.update(track="sar_human_dev_static_synthetic", rgbd_size=[int(x) for x in args.size.split("x")],
                        detector="official_yolov5n_v7_person", model_sha256=digest(args.model), confidence=args.confidence,
                        asset_manifest_sha256=digest(ROOT / "experiments/ue5_s4/assets/manifest.json"))
        shutil.copyfile(ROOT / "experiments/ue5_s4/assets/manifest.json", store.path / "asset-manifest.json")
    write_json(store.path / "manifest.json", manifest)
    output = (store.path / "engine-stdout.txt").open("xb")
    exit_code = 1
    result = None
    wall_started = time.perf_counter()
    try:
        engine = subprocess.Popen(command, cwd=store.path, env=env, stdout=output, stderr=subprocess.STDOUT,
                                  creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        store.event({"type": "engine_start", "pid": engine.pid})
        start = time.monotonic()
        ready_path = store.path / "host-ready.json"
        while not ready_path.exists():
            if engine.poll() is not None:
                raise RuntimeError(f"engine exited before ready: {engine.returncode}")
            if time.monotonic() - start > 300:
                raise TimeoutError("engine startup watchdog")
            time.sleep(.2)
        ready = strict_loads(ready_path.read_bytes())
        if ready["status"] != "READY":
            raise RuntimeError(str(ready))
        client = HostClient(port, token, timeout=60)
        raw_stream = gzip.open(store.path / "private/host.jsonl.gz", "xb")
        public_stream = gzip.open(store.path / "public/observations.jsonl.gz", "xb")
        backend = UnrealBackend(client, raw_sink=lambda raw: raw_stream.write(canonical(raw) + b"\n"))
        write_json(store.path / "host-capabilities.json", backend.host_info)
        for number, (config, method) in enumerate((item, method) for item in schedule(args.suite, args.seeds,args.environments,args.cases,args.postures) for method in args.methods):
            prefix = f"e{number:03d}"
            write_json(store.path / f"private/{prefix}-scenario.json", config)
            episode = Episode(backend, config, args.ticks)
            policy = PolicyProcess(method, config["seed"], snapshot, args.model, args.confidence)
            timings = []
            try:
                obs = episode.reset()
                store.event({"type": "episode_start", "id": prefix, "epoch": obs["epoch"], "method": method})
                deadline = time.monotonic() + 1800
                while obs is not None:
                    public_stream.write(canonical({"episode": prefix, "observation": obs}) + b"\n")
                    public_stream.flush()
                    start = time.perf_counter()
                    action = policy.act(obs)
                    timings.append(time.perf_counter() - start)
                    obs, ack = episode.submit(action)
                    store.event({"type": "agent_submission", "episode": prefix, "action": action, "ack": ack})
                    if time.monotonic() > deadline:
                        episode.terminate("AGENT_TIMEOUT")
                        break
            except Exception as exc:
                if episode.phase == Phase.RUNNING:
                    episode.terminate("AGENT_TIMEOUT" if isinstance(exc, TimeoutError) else "AGENT_ERROR")
                store.event({"type": "episode_exception", "episode": prefix, "traceback": traceback.format_exc()})
                if episode.ledger is None or episode.ledger["terminal_reason"] == "INFRA_ERROR":
                    raise
            finally:
                worker = policy.close()
                write_json(store.path / f"private/{prefix}-worker.json", worker)
                if episode.ledger is not None:
                    write_json(store.path / f"private/{prefix}-ledger.json", episode.ledger)
                    write_json(store.path / f"private/{prefix}-commands.json", episode.commands)
            metrics = evaluate(episode.ledger)
            reference = evaluate(episode.ledger, matcher=reference_match)
            if metrics != reference:
                raise RuntimeError("independent reference evaluator disagreement")
            row = {"episode": prefix, "method": method, "family": config["family"], "pair_id": config["pair_id"],
                   "kind": config["kind"], "perception": config["perception"], "odometry": config["odometry"],
                   "metrics": metrics, "agent_latency_s": timings}
            rows.append(row)
            write_json(store.path / f"{prefix}-metrics.json", row)
            store.event({"type": "episode_end", "id": prefix, "metrics": metrics})
            print(prefix, method, config["kind"], config["perception"], config["odometry"], metrics["terminal_reason"], "TP", metrics["tp"], "FP", metrics["fp"], flush=True)
        interactions = rq1([r for r in rows if r["kind"] == "clean"]) if args.suite in ("pilot","sar-pilot") else None
        if args.calibrate:
            from .calibration import calibrate
            calibration = calibrate(backend)
            write_json(store.path / "calibration.json", calibration)
            if calibration["status"] != "PASS_ENGINE_CALIBRATION":
                raise RuntimeError("engine calibration failed; see calibration.json")
        result = {"rows": rows, "rq1": interactions,
                  "scope": "Human SAR dev scenes; no statistical generalization/empirical sensor fidelity claim" if human else "Rendered primitive fixture only; no scientific RQ/physical fidelity conclusion"}
        exit_code = 0
    except Exception:
        write_json(store.path / "failure.json", {"status": "FAILED", "traceback": traceback.format_exc(), "completed_episodes": len(rows)})
        traceback.print_exc()
    finally:
        if client:
            try:
                client.call("shutdown")
            except Exception as error:
                store.event({"type": "shutdown_error", "error": str(error)})
            client.close()
        if engine:
            try:
                engine.wait(timeout=30)
            except subprocess.TimeoutExpired:
                # Only the process created above is terminated.
                engine.kill()
                engine.wait(timeout=10)
            store.event({"type": "engine_exit", "exit_code": engine.returncode})
            if engine.returncode != 0:
                exit_code = 1
        output.close()
        if (store.path / "engine.log").is_file():
            shutil.copyfile(store.path / "engine.log", store.path / "engine-log.txt")
        for stream in (raw_stream, public_stream):
            if stream:
                stream.close()
        if result is not None:
            result.update(status="COMPLETED_DIAGNOSTIC_UE_RUNTIME" if exit_code == 0 else "FAILED_ENGINE_EXIT",
                          engine_exit_code=engine.returncode if engine else None,
                          wall_duration_s=time.perf_counter() - wall_started)
            write_json(store.path / "results.json", result)
        store.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
