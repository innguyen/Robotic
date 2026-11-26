"""Reproducible local UE pilot runner. Use a NEW output directory each time."""
import argparse
import base64
import datetime
import gzip
import json
import math
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
        self.exchange = None

    def act(self, obs, timeout=30):
        # A worker stuck before reading stdin can fill the pipe. The watchdog
        # must cover serialization/write as well as the response wait.
        completed = queue.Queue()
        def exchange():
            try:
                self.last_timing = {"input_write_start_wall_ns":time.monotonic_ns()}
                self.process.stdin.write(canonical(obs) + b"\n")
                self.process.stdin.flush()
                self.last_timing["input_write_complete_wall_ns"] = time.monotonic_ns()
                line = self.responses.get()
                if line is None: raise RuntimeError("agent worker exited")
                if len(line) > 65536 or not line.endswith(b"\n"):
                    raise ContractError("oversized/unparseable agent message")
                result = strict_loads(line)
                self.last_timing["response_received_wall_ns"] = time.monotonic_ns()
                completed.put((True,result))
            except Exception as error:
                completed.put((False,error))
        self.exchange = threading.Thread(target=exchange,daemon=True)
        self.exchange.start()
        try:
            success,result = completed.get(timeout=timeout)
        except queue.Empty as exc:
            # Terminating this owned worker releases blocked pipe writes before
            # close() tries to acquire the BufferedWriter lock.
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait(timeout=3)
            self.exchange.join(timeout=2)
            raise TimeoutError("agent action watchdog") from exc
        self.exchange.join(timeout=2)
        if not success: raise result
        return result

    def close(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        self.reader.join(timeout=2)
        try:
            self.process.stdin.close()
        except BrokenPipeError:
            pass  # Expected when a terminated worker had buffered input.
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


def schedule(suite, seeds, environments=("indoor", "outdoor"), cases=("clean", "occluded", "absent", "multi", "obstacles", "blocked"), postures=("standing",), depth_model="structured", scene_set="legacy"):
    items = []
    for seed in range(seeds):
        if suite.startswith("sar"):
            for environment in environments:
                for posture in postures:
                    for kind in cases:
                        items.append(sar_scenario(kind, seed, environment=environment, posture=posture, depth_model=depth_model,scene_set=scene_set))
                    if suite == "sar-pilot":
                        for p, n in ((1,0),(0,1),(1,1)):
                            items.append(sar_scenario("clean",seed,p,n,environment,posture,depth_model,scene_set))
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
    parser.add_argument("--scene-set",choices=("legacy","assets"),default="legacy")
    parser.add_argument("--cases", nargs="+", choices=("clean","occluded","absent","multi","obstacles","blocked"), default=["clean","occluded","absent","multi","obstacles","blocked"])
    parser.add_argument("--postures", nargs="+", choices=("standing","seated","prone"), default=["standing"])
    parser.add_argument("--size", default="320x240", choices=("160x120","320x240","640x480"))
    parser.add_argument("--scan-rays", type=int, choices=(360,720), default=360)
    parser.add_argument("--depth-model", choices=("ideal","gaussian","structured"), default="structured")
    parser.add_argument("--render-quality",choices=("preview","standard"),default="preview")
    parser.add_argument("--rhi",choices=("d3d11","d3d12"),default="d3d11")
    parser.add_argument("--model", type=Path)
    parser.add_argument("--confidence", type=float, default=.4)
    parser.add_argument("--action-dt", type=float, choices=(.1,.5), default=.1)
    parser.add_argument("--ticks", type=int, default=30)
    parser.add_argument("--seeds", type=int, default=1)
    parser.add_argument("--methods", nargs="+", choices=("random", "sweep", "core", "frontier", "verify"), default=["random", "sweep"])
    parser.add_argument("--calibrate", action="store_true", help="also run privileged static engineering checks")
    parser.add_argument("--admissibility", action="store_true", help="pre-policy native visibility and conservative 2D reachability probes")
    parser.add_argument("--overview",action="store_true",help="save a private robot/scene QA view; never forwarded to policy")
    args = parser.parse_args()
    if not math.isfinite(args.confidence) or not 0 <= args.confidence <= 1:
        parser.error("confidence must be finite in [0,1]")
    for name in ("methods","environments","cases","postures"):
        values = getattr(args,name)
        if len(values) != len(set(values)): parser.error("duplicate "+name)
    human = args.suite.startswith("sar")
    if args.scene_set=="assets" and not human:parser.error("asset scenes require human SAR suite")
    if args.overview and not human: parser.error("overview requires a human SAR suite")
    if human:
        args.model = (args.model or ROOT / "experiments/ue5_s4/assets/source/yolov5n-v7.onnx").resolve()
        if not args.model.is_file(): parser.error("run prepare_assets.py first")
        from experiments.ue5_s4.prepare_assets import EXPECTED
        verified_model = digest(args.model) == EXPECTED["yolov5n-v7.onnx"]
        if args.model == ROOT / "experiments/ue5_s4/assets/source/yolov5n-v7.onnx" and not verified_model:
            parser.error("pinned default model hash mismatch; restore with prepare_assets.py")
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
    memory_sampler = None
    raw_stream = public_stream = None
    token = secrets.token_hex(32)
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    env = dict(os.environ)
    env.update(SAR_HOST_TOKEN=token, SAR_HOST_PORT=str(port), SAR_HOST_READY=str(store.path / "host-ready.json"))
    env["SAR_ACTION_DT_NS"] = str(round(args.action_dt*1e9))
    env["SAR_RENDER_QUALITY"] = args.render_quality
    if human:
        env.update(SAR_PROFILE="human", SAR_ASSET_ROOT=str(ROOT / "experiments/ue5_s4/assets"), SAR_RGB_SIZE=args.size,
                   SAR_CAPTURE_DIR=str(store.path / "private/capture-tmp"),SAR_SCAN_RAYS=str(args.scan_rays))
        env["SAR_SCENE_SET"]=args.scene_set
    command = [str(args.editor.resolve()), str(ROOT / "experiments/ue5_s4/editor_host/SarEditorSlice.uproject"),
               "-unattended", "-nosplash", "-nosound", "-"+args.rhi, "-AllowCommandletRendering",
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
    manifest["action_dt_ns"] = round(args.action_dt*1e9)
    manifest["editor_render_fps_cap"] = 10
    manifest["render_quality"] = args.render_quality
    manifest["requested_rhi"] = args.rhi
    manifest["pre_policy_admissibility"] = args.admissibility
    manifest["private_overview_requested"] = args.overview
    manifest["runner_arguments"] = {name:getattr(args,name) for name in
        ("suite","environments","cases","postures","size","scan_rays","depth_model","confidence","action_dt",
         "ticks","seeds","methods","calibrate","admissibility","overview","render_quality","rhi","scene_set")}
    clock = time.get_clock_info("monotonic")
    manifest["runner_monotonic_clock"] = {"implementation":clock.implementation,"resolution_s":clock.resolution}
    if human:
        manifest.update(track="sar_human_dev_static_synthetic", rgbd_size=[int(x) for x in args.size.split("x")],
                        detector="official_yolov5n_v7_person" if verified_model else "user_supplied_coco85_ONNX_unverified_provenance", model_sha256=digest(args.model), confidence=args.confidence,
                        scan_rays=args.scan_rays, depth_model=args.depth_model, asset_manifest_sha256=digest(ROOT / "experiments/ue5_s4/assets/manifest.json"),
                        derived_pose_manifest_sha256=digest(ROOT / "experiments/ue5_s4/assets/derived/poses.json"))
        shutil.copyfile(ROOT / "experiments/ue5_s4/assets/manifest.json", store.path / "asset-manifest.json")
        shutil.copyfile(ROOT / "experiments/ue5_s4/assets/derived/poses.json", store.path / "human-pose-manifest.json")
        if args.scene_set=="assets":
            manifest["scene_set"]="assets"
            manifest["scene_asset_manifests"]={}
            for relative in ("environment/lock.json","environment/meshes.json","anatomy/lock.json","anatomy/derived/poses.json"):
                name=relative.replace("/","-")
                shutil.copyfile(ROOT/"experiments/ue5_s4/assets"/relative,store.path/name)
                manifest["scene_asset_manifests"][name]=digest(store.path/name)
            # Exact bytes used by the importer are retained, not just live paths.
            for directory in ("environment","anatomy"):
                shutil.copytree(ROOT/"experiments/ue5_s4/assets"/directory,store.path/"asset-inputs"/directory,
                                ignore=shutil.ignore_patterns("__pycache__"))
            shutil.copytree(ROOT/"experiments/ue5_s4/assets/derived",store.path/"asset-inputs/derived")
            env["SAR_ASSET_ROOT"]=str(store.path/"asset-inputs")
    write_json(store.path / "manifest.json", manifest)
    output = (store.path / "engine-stdout.txt").open("xb")
    exit_code = 1
    result = None
    failure_phase = "runtime"
    wall_started = time.perf_counter()
    try:
        engine = subprocess.Popen(command, cwd=store.path, env=env, stdout=output, stderr=subprocess.STDOUT,
                                  creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        from .runtime_metrics import MemorySampler
        memory_sampler=MemorySampler(engine)
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
        raw_stream = gzip.open(store.path / "private/host.jsonl.gz", "xb",compresslevel=1)
        public_stream = gzip.open(store.path / "public/observations.jsonl.gz", "xb",compresslevel=1)
        backend = UnrealBackend(client, raw_sink=lambda raw: raw_stream.write(canonical(raw) + b"\n"))
        write_json(store.path / "host-capabilities.json", backend.host_info)
        scheduled = schedule(args.suite, args.seeds,args.environments,args.cases,args.postures,args.depth_model,args.scene_set)
        # Preserve rejected authoring inputs too, before any admission probe.
        write_json(store.path / "private/scheduled-scenarios.json",scheduled)
        if args.admissibility:
            failure_phase = "admissibility"
            if not human: raise ValueError("native mask admissibility requires sar suite")
            from .admissibility import probe
            for index,config in enumerate(scheduled):
                admission = probe(backend,config)
                write_json(store.path / f"private/admission-{index:03d}.json",admission)
                if not admission["admissible"] and config["kind"] != "blocked":
                    raise RuntimeError(f"scenario {index} rejected by pre-policy admissibility")
        failure_phase = "runtime"
        for number, (config, method) in enumerate((item, method) for item in scheduled for method in args.methods):
            prefix = f"e{number:03d}"
            write_json(store.path / f"private/{prefix}-scenario.json", config)
            episode = Episode(backend, config, args.ticks)
            policy = PolicyProcess(method, config["seed"], snapshot, args.model, args.confidence)
            timings = []
            try:
                obs = episode.reset()
                if args.overview:
                    try:
                        qa = client.call("overview")
                        qa_png = store.path / ("private/overview.png" if number==0 else f"private/{prefix}-overview.png")
                        qa_png.write_bytes(base64.b64decode(qa.pop("rgb_png"),validate=True))
                        write_json(qa_png.with_name(qa_png.stem+"-camera.json"),qa)
                        store.event({"type":"private_qa_overview","png_sha256":digest(qa_png)})
                    except Exception:
                        episode.terminate("INFRA_ERROR")
                        raise
                store.event({"type": "episode_start", "id": prefix, "epoch": obs["epoch"], "method": method})
                deadline = time.monotonic() + 1800
                while obs is not None:
                    public_stream.write(canonical({"episode": prefix, "observation": obs}) + b"\n")
                    public_stream.flush()
                    start = time.perf_counter()
                    capture_timing = dict(backend.last_capture_timing)
                    action = policy.act(obs)
                    timings.append(time.perf_counter() - start)
                    obs, ack = episode.submit(action)
                    store.event({"type": "agent_submission", "episode": prefix, "action": action, "ack": ack,
                                 "capture_timing":capture_timing,"policy_timing":policy.last_timing})
                    if time.monotonic() > deadline:
                        episode.terminate("AGENT_TIMEOUT")
                        break
            except Exception as exc:
                if episode.phase == Phase.RUNNING:
                    episode.terminate("AGENT_TIMEOUT" if isinstance(exc, TimeoutError) else
                                      "AGENT_PROTOCOL_ERROR" if isinstance(exc,ContractError) else "AGENT_ERROR")
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
        result = {"rows": rows, "rq1": interactions,
                  "scope": "Human SAR dev scenes; no statistical generalization/empirical sensor fidelity claim" if human else "Rendered primitive fixture only; no scientific RQ/physical fidelity conclusion"}
        if args.calibrate:
            failure_phase = "calibration"
            from .calibration import calibrate, calibrate_human
            calibration = calibrate_human(backend) if human else calibrate(backend)
            if args.scene_set=="assets":
                from .calibration import calibrate_asset_scenes
                asset_calibration=calibrate_asset_scenes(backend)
                write_json(store.path/"asset-calibration.json",asset_calibration)
                calibration["checks"]["asset_scenes_reset_visibility"]=asset_calibration["status"]=="PASS_ENGINE_CALIBRATION"
                calibration["status"]="PASS_ENGINE_CALIBRATION" if all(calibration["checks"].values()) else "FAIL_ENGINE_CALIBRATION"
            write_json(store.path / "calibration.json", calibration)
            if calibration["status"] != "PASS_ENGINE_CALIBRATION":
                raise RuntimeError("engine calibration failed; see calibration.json")
        exit_code = 0
    except Exception:
        write_json(store.path / "failure.json", {"status": "FAILED", "phase":failure_phase, "traceback": traceback.format_exc(), "completed_episodes": len(rows)})
        traceback.print_exc()
    finally:
        if memory_sampler:write_json(store.path/"runtime-memory.json",memory_sampler.close())
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
            failed_status = "FAILED_ENGINE_EXIT" if engine and engine.returncode != 0 else "FAILED_CALIBRATION" if failure_phase=="calibration" else "FAILED_RUNTIME"
            result.update(status=("COMPLETED_HUMAN_DEV_UE_RUNTIME" if human else "COMPLETED_DIAGNOSTIC_UE_RUNTIME") if exit_code == 0 else failed_status,
                          engine_exit_code=engine.returncode if engine else None,
                          wall_duration_s=time.perf_counter() - wall_started)
            write_json(store.path / "results.json", result)
        store.close()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
