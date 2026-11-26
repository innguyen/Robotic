"""Offline recomputation, independent reference and adversarial real-trace replay."""
import argparse
import copy
import gzip
import hashlib
import json
import math
import statistics
from pathlib import Path

from .artifacts import digest, verify_inventory, write_json
from .contracts import canonical, public_copy, strict_loads
from .evaluator import evaluate
from .reference import reference_score


def audit(path):
    path = Path(path)
    required=("manifest.json","inventory.json","journal.jsonl","host-capabilities.json",
              "public/observations.jsonl.gz","private/host.jsonl.gz")
    missing=[name for name in required if not (path/name).is_file()]
    if missing:
        return {"status":"FAIL","data_integrity_status":"INCOMPLETE_UNACCEPTED",
            "missing_artifacts":missing,"reference_episodes":0,"public_packets":0,
            "recorded_ledgers":len(list((path/"private").glob("*-ledger.json"))),
            "errors":["required acceptance artifacts missing; no successful audit inferred from scheduled or recorded episode count"],
            "requested_calibration_status":"NOT_AUDITED","engine_clean_exit":False,
            "scope":"Incomplete attempt; recorded_ledgers is a file count, not independently accepted episodes"}
    checks = {"inventory": verify_inventory(path), "reference_episodes": 0, "public_packets": 0,
              "source_snapshot": True, "errors": []}
    manifest = strict_loads((path / "manifest.json").read_bytes())
    for name, sha in manifest["source_sha256"].items():
        snapshot = path / "source" / name
        if not snapshot.is_file() or digest(snapshot) != sha:
            checks["source_snapshot"] = False
    previous = "0" * 64
    engine_exit = None
    delivery_packets = 0
    host = strict_loads((path / "host-capabilities.json").read_bytes())
    same_clock = host.get("monotonic_clock",{}).get("implementation") == manifest.get("runner_monotonic_clock",{}).get("implementation") and "runner_monotonic_clock" in manifest
    for line in (path / "journal.jsonl").read_text(encoding="utf-8").splitlines():
        record = strict_loads(line)
        if record["previous_sha256"] != previous:
            checks["errors"].append("journal chain discontinuity")
        actual = hashlib.sha256(canonical({"previous_sha256": previous, "event": record["event"]})).hexdigest()
        if actual != record["sha256"]:
            checks["errors"].append("journal hash mismatch")
        previous = record["sha256"]
        if record["event"].get("type") == "engine_exit":
            engine_exit = record["event"]["exit_code"]
        event = record["event"]
        if event.get("type") == "agent_submission" and "policy_timing" in event:
            capture,policy = event["capture_timing"],event["policy_timing"]
            times = [capture[k] for k in ("rpc_decode_complete_wall_ns","measurements_ready_wall_ns")]
            times += [policy[k] for k in ("input_write_start_wall_ns","input_write_complete_wall_ns","response_received_wall_ns")]
            if same_clock:
                times = [capture[k] for k in ("capture_start_wall_ns","capture_end_wall_ns","available_wall_ns")]+times
            if times != sorted(times): checks["errors"].append("capture-to-policy delivery timing order")
            delivery_packets += 1
    checks["engine_exit_code"] = engine_exit
    checks["engine_clean_exit"] = engine_exit == 0
    if manifest.get("private_overview_requested"):
        for name in ("private/overview.png","private/overview-camera.json"):
            if not (path / name).is_file(): checks["errors"].append("requested QA artifact missing: "+name)
        camera_file = path / "private/overview-camera.json"
        if camera_file.is_file():
            qa = strict_loads(camera_file.read_bytes())
            if "robot_geometry_recipe" in qa:
                recipe = qa["robot_geometry_recipe"]; x0,y0,yaw = qa["robot_pose_m"]
                c,s = math.cos(yaw),math.sin(yaw); length,width = recipe["half_length_m"],recipe["half_width_m"]
                actual = {item["label"]:item for item in qa["robot_visual_poses"]}
                expected = {f"wheel_{i}":([x0+c*x-s*y,y0+s*x+c*y,recipe["wheel_radius_m"]],
                            [2*recipe["wheel_radius_m"],2*recipe["wheel_radius_m"],recipe["wheel_thickness_m"]])
                            for i,(x,y) in enumerate(((-length,-width),(-length,width),(length,-width),(length,width)))}
                expected["sensor_mast"] = ([x0,y0,recipe["mast_center_z_m"]],recipe["mast_size_m"])
                checks["robot_visual_geometry"] = set(actual)==set(expected) and all(
                    max(abs(a-b) for a,b in zip(actual[label][field],values)) < 1e-5
                    for label,pair in expected.items() for field,values in zip(("position_m","world_scale"),pair))
                if not checks["robot_visual_geometry"]: checks["errors"].append("robot visual geometry differs from metric recipe")
    checks["delivery_timing_packets"] = delivery_packets
    checks["host_runner_clock_implementation_matches"] = same_clock
    inventory = strict_loads((path / "inventory.json").read_bytes())
    if previous != inventory["journal_head"]:
        checks["errors"].append("journal final head mismatch")
    delivered = {}
    with gzip.open(path / "public/observations.jsonl.gz", "rt", encoding="utf-8") as stream:
        for line in stream:
            packet = strict_loads(line)
            public_copy(packet["observation"])
            obs = packet["observation"]
            if [obs["camera"]["width"],obs["camera"]["height"]] != manifest["rgbd_size"]:
                checks["errors"].append("public image size disagrees with manifest")
            if "scan_rays" in manifest and len(obs["scan"]["ranges_m"]) != manifest["scan_rays"]:
                checks["errors"].append("public scan count disagrees with manifest")
            if obs.get("action_dt_ns",100_000_000) != manifest.get("action_dt_ns",100_000_000):
                checks["errors"].append("public action clock disagrees with manifest")
            identity = (obs["epoch"],obs["sequence"])
            if identity in delivered:
                checks["errors"].append("duplicate delivered frame")
            delivered[identity] = hashlib.sha256(canonical(obs["rgb"]["data"])).hexdigest()
            checks["public_packets"] += 1
    ledgers = []
    for file in sorted((path / "private").glob("*-ledger.json")):
        data = strict_loads(file.read_bytes())
        score, reference = evaluate(data), reference_score(data)
        for key in reference:
            if score[key] != reference[key]:
                checks["errors"].append(f"{file.name}/{key}: reference disagreement")
        checks["reference_episodes"] += 1
        prefix=file.name.removesuffix("-ledger.json")
        metrics_file=path/(prefix+"-metrics.json")
        if not metrics_file.is_file() or strict_loads(metrics_file.read_bytes()).get("metrics")!=score:
            checks["errors"].append(prefix+": stored metrics differ from ledger recomputation")
        for sequence,frame in data["frames"].items():
            # A final safety-abort/timeout capture can remain private, while
            # report evidence must always refer to a delivered public capture.
            key = (frame["epoch"],int(sequence))
            if key in delivered and frame["delivered_rgb_sha256"] != delivered[key]:
                checks["errors"].append(f"{file.name}/{sequence}: delivered RGB hash mismatch")
        for report_index,_ in score["matches"]:
            for evidence in data["reports"][report_index]["payload"]["evidence"]:
                if (data["epoch"],evidence["rgb_sequence"]) not in delivered:
                    checks["errors"].append(f"{file.name}: undelivered report evidence")
        worker = strict_loads(file.with_name(file.name.replace("-ledger", "-worker")).read_bytes())
        if worker["exit_code"] != 0 and data["terminal_reason"] not in ("AGENT_ERROR","AGENT_TIMEOUT","AGENT_PROTOCOL_ERROR"):
            checks["errors"].append(f"{file.name}: policy worker exit {worker['exit_code']}")
        ledgers.append((file.stem, data, score))
    # Software readback intervals and availability are real monotonic clocks;
    # they do not claim per-exposure hardware time or global physics lockstep.
    intervals, previous_end, raw_count, first_start = [], -1, 0, None
    with gzip.open(path / "private/host.jsonl.gz", "rt", encoding="utf-8") as stream:
        for line in stream:
            raw = strict_loads(line)
            start,end,available = (raw.get(key) for key in ("capture_start_wall_ns","capture_end_wall_ns","available_wall_ns"))
            if not all(type(value) is int for value in (start,end,available)) or not previous_end <= start <= end <= available:
                checks["errors"].append(f"capture {raw_count}: non-monotonic readback/availability")
            else:
                intervals.append((end-start)/1e9)
                if first_start is None: first_start = start
                previous_end = available
            raw_count += 1
    checks["native_capture_packets"] = raw_count
    checks["native_capture_window_wall_s"] = (previous_end-first_start)/1e9 if first_start is not None else None
    checks["readback_interval_s"] = {"median":statistics.median(intervals),"max":max(intervals)} if intervals else None
    if manifest.get("track") == "sar_human_dev_static_synthetic":
        checks["asset_manifest_checks"] = {}
        for name, field in (("asset-manifest.json","asset_manifest_sha256"),("human-pose-manifest.json","derived_pose_manifest_sha256")):
            if field not in manifest:
                checks["asset_manifest_checks"][name] = "NOT_RECORDED_BY_EARLIER_RUN"
            elif not (path / name).is_file() or digest(path / name) != manifest[field]:
                checks["errors"].append(name+": missing or manifest mismatch")
                checks["asset_manifest_checks"][name] = "FAIL"
            else:
                checks["asset_manifest_checks"][name] = "PASS"
    for name,sha in manifest.get("scene_asset_manifests",{}).items():
        file=path/name
        if not file.is_file() or digest(file)!=sha:
            checks["errors"].append(name+": scene asset manifest mismatch")
    if manifest.get("scene_set")=="assets":
        checks["scene_input_bytes_verified"]=0
        for category in ("environment","anatomy"):
            root=path/"asset-inputs"/category
            metadata = "meshes.json" if category == "environment" else "derived/poses.json"
            for relative in ("lock.json", metadata):
                recorded = (category + "/" + relative).replace("/", "-")
                actual = root / relative
                if not actual.is_file() or digest(actual) != manifest.get("scene_asset_manifests", {}).get(recorded):
                    checks["errors"].append(recorded + ": frozen importer metadata differs from launch manifest")
            if not (root/"lock.json").is_file() or not (root/metadata).is_file():
                continue
            lock=strict_loads((root/"lock.json").read_bytes())
            files=lock["files"]
            if type(files) is dict:files=[{"file":name,**item} for name,item in files.items()]
            for item in files:
                source=(root/"source"/item["file"]).resolve()
                if not source.is_relative_to((root/"source").resolve()) or not source.is_file() or digest(source)!=item["sha256"]:
                    checks["errors"].append(category+": locked input mismatch: "+item["file"])
                else:checks["scene_input_bytes_verified"]+=1
            poses=strict_loads((root/metadata).read_bytes())
            for item in poses.values():
                source=root/"derived"/item["file"]
                if not source.is_file() or digest(source)!=item["sha256"]:
                    checks["errors"].append(category+": derived mesh mismatch: "+item["file"])
                else:checks["scene_input_bytes_verified"]+=1
        if manifest.get("human_asset_profile") == "makehuman_dressed_v2":
            folder = path/"asset-inputs/anatomy/dressed"
            expected = manifest.get("scene_asset_manifests",{}).get("anatomy-dressed-poses.json")
            poses_file = folder/"poses.json"
            if not poses_file.is_file() or digest(poses_file) != expected or expected != manifest.get("derived_pose_manifest_sha256"):
                checks["errors"].append("dressed pose metadata differs from launch manifest")
            else:
                for item in strict_loads(poses_file.read_bytes()).values():
                    obj = folder/item["file"]
                    if not obj.is_file() or digest(obj) != item["sha256"]:
                        checks["errors"].append("dressed pose mesh mismatch: "+item["file"])
                    else:
                        checks["scene_input_bytes_verified"] += 1
    base = next(((name, data, score) for name, data, score in ledgers if score["tp"] == 1 and score["victims"] == 1), None)
    controls = {}
    if base:
        name, data, score = base
        report_index = score["matches"][0][0]
        slot = copy.deepcopy(data["reports"][report_index])
        duplicate = copy.deepcopy(slot)
        duplicate["payload"]["report_id"] = "diagnostic_duplicate"
        false = copy.deepcopy(slot)
        false["payload"]["report_id"] = "diagnostic_false"
        false["payload"]["position_episode_m"][0] += 20
        replay = copy.deepcopy(data)
        replay["reports"] = [slot, duplicate, false]
        replay_score = evaluate(replay)
        controls["duplicate_false"] = {"source": name, "expected_tp_fp": [1, 2], "actual_tp_fp": [replay_score["tp"], replay_score["fp"]],
                                      "ledger": replay, "metrics": replay_score}
        cells = {}
        for p, n in ((0, 0), (1, 0), (0, 1), (1, 1)):
            replay = copy.deepcopy(data)
            replay["reports"] = [copy.deepcopy(slot)]
            replay["reports"][0]["payload"]["position_episode_m"][0] += .7 * (p + n)
            value = evaluate(replay)["safe_localization_recall"]
            cells[f"{p}{n}"] = value
        controls["rq1_metric_sensitivity"] = {"source": name, "cells": cells,
            "interaction": cells["11"] - cells["10"] - cells["01"] + cells["00"],
            "scope": "OFFLINE REPORT-COORDINATE PERTURBATION ONLY, not a rendered sensor intervention or scientific RQ1 result"}
        attacks = {}
        for attack in ("oracle_position_forged_frames","wrong_epoch","backdated","malformed_confidence","repeated_capture","out_of_image","stale_evidence"):
            trial = copy.deepcopy(data); candidate = copy.deepcopy(slot)
            payload = candidate["payload"]
            if attack == "oracle_position_forged_frames":
                payload["position_episode_m"] = data["victims"][0]["anchor_m"][:]
                for e in payload["evidence"]: e["rgb_sequence"] = 999999
            elif attack == "wrong_epoch": payload["epoch"] += 1
            elif attack == "backdated": payload["tick"] += 1
            elif attack == "malformed_confidence": payload["confidence"] = "invalid"
            elif attack == "repeated_capture": payload["evidence"][1] = copy.deepcopy(payload["evidence"][0])
            elif attack == "out_of_image": payload["evidence"][0]["bbox_xyxy_px"][2] = 999999
            elif attack == "stale_evidence":
                # Extend only this synthetic replay clock; no engine outcome.
                trial["budget_ns"] += 3_000_000_000
                trial["terminal_ns"] += 3_000_000_000
                candidate["accepted_ns"] = trial["terminal_ns"]
                payload["tick"] = candidate["accepted_ns"]//trial.get("action_dt_ns",100_000_000)
            trial["reports"] = [candidate]
            measured, independent = evaluate(trial), reference_score(trial)
            outcome = [measured["tp"],measured["fp"]]
            attacks[attack] = {"expected_tp_fp":[0,1],"actual_tp_fp":outcome,
                              "independent_tp_fp":[independent["tp"],independent["fp"]],"candidate":candidate,
                              "terminal_ns":trial["terminal_ns"],"budget_ns":trial["budget_ns"]}
            if outcome != [0,1] or outcome != [independent["tp"],independent["fp"]]:
                checks["errors"].append("adversarial replay: "+attack)
        controls["adversarial_real_trace"] = {"source":name,"attacks":attacks,"scope":"offline mutations of actual delivered evidence, not additional UE episodes"}
        if [replay_score["tp"],replay_score["fp"]] != [1,2]: checks["errors"].append("duplicate/false control mismatch")
    checks["controls"] = controls
    integrity_ok = checks["inventory"] and checks["source_snapshot"] and checks["engine_clean_exit"] and ledgers and not checks["errors"]
    checks["data_integrity_status"] = "PASS_OFFLINE_DATA_AUDIT" if integrity_ok else "FAIL"
    checks["requested_calibration_status"] = "NOT_REQUESTED"
    if manifest.get("calibrate"):
        calibration_file = path / "calibration.json"
        calibration = strict_loads(calibration_file.read_bytes()) if calibration_file.is_file() else {}
        checks["requested_calibration_status"] = calibration.get("status", "MISSING")
        if checks["requested_calibration_status"] == "PASS_ENGINE_CALIBRATION":
            components = calibration.get("checks", {})
            valid = bool(components) and all(value is True for value in components.values())
            for component, filename in (("asset_scenes_reset_visibility", "asset-calibration.json"),
                                        ("asset_native_geometry_probe", "asset-geometry-calibration.json")):
                if component in components:
                    evidence = path / filename
                    record = strict_loads(evidence.read_bytes()) if evidence.is_file() else {}
                    flags = record.get("checks", {})
                    valid = valid and record.get("status") == "PASS_ENGINE_CALIBRATION" and bool(flags) and all(v is True for v in flags.values())
            if not valid:
                checks["requested_calibration_status"] = "FAIL_INCONSISTENT_CALIBRATION"
        if checks["requested_calibration_status"] != "PASS_ENGINE_CALIBRATION":
            checks["errors"].append("requested native calibration did not pass")
    result_file=path/"results.json"
    checks["run_acceptance_errors"]=[]
    if not result_file.is_file():
        checks["run_acceptance_errors"].append("results missing; scheduled/partial episodes do not establish completion")
    else:
        result=strict_loads(result_file.read_bytes())
        if result.get("status") not in ("COMPLETED_HUMAN_DEV_UE_RUNTIME","COMPLETED_DIAGNOSTIC_UE_RUNTIME"):
            checks["run_acceptance_errors"].append("run status is not completed")
        if result.get("engine_exit_code")!=engine_exit:
            checks["errors"].append("results engine exit disagrees with journal")
        expected={name.removesuffix("-ledger"):score for name,data,score in ledgers}
        actual={row["episode"]:row["metrics"] for row in result.get("rows",[])}
        if expected!=actual or len(actual)!=len(result.get("rows",[])):
            checks["errors"].append("results rows differ from independently recomputed ledgers")
        scheduled_file=path/"private/scheduled-scenarios.json"
        checks["scheduled_episode_count"]="NOT_RECORDED_BY_EARLIER_RUN"
        if scheduled_file.is_file():
            count=len(strict_loads(scheduled_file.read_bytes()))*len(manifest["methods"])
            checks["scheduled_episode_count"]=count
            if len(ledgers)!=count:checks["run_acceptance_errors"].append("completed ledger count differs from scheduled episodes")
    checks["data_integrity_status"]="PASS_OFFLINE_DATA_AUDIT" if integrity_ok and not [e for e in checks["errors"] if e!="requested native calibration did not pass"] else "FAIL"
    checks["status"] = "PASS_OFFLINE_AUDIT" if integrity_ok and not checks["errors"] and not checks["run_acceptance_errors"] else "FAIL"
    return checks


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.run)
    write_json(args.out, result)
    print(result["status"], "reference episodes:", result["reference_episodes"], "packets:", result["public_packets"])
    raise SystemExit(result["status"] != "PASS_OFFLINE_AUDIT")
