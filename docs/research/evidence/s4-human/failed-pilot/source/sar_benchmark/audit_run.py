"""Offline recomputation, independent reference and adversarial real-trace replay."""
import argparse
import copy
import gzip
import hashlib
import json
import statistics
from pathlib import Path

from .artifacts import digest, verify_inventory, write_json
from .contracts import canonical, public_copy, strict_loads
from .evaluator import evaluate
from .reference import reference_score


def audit(path):
    path = Path(path)
    checks = {"inventory": verify_inventory(path), "reference_episodes": 0, "public_packets": 0,
              "source_snapshot": True, "errors": []}
    manifest = strict_loads((path / "manifest.json").read_bytes())
    for name, sha in manifest["source_sha256"].items():
        snapshot = path / "source" / name
        if not snapshot.is_file() or digest(snapshot) != sha:
            checks["source_snapshot"] = False
    previous = "0" * 64
    engine_exit = None
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
    checks["engine_exit_code"] = engine_exit
    checks["engine_clean_exit"] = engine_exit == 0
    inventory = strict_loads((path / "inventory.json").read_bytes())
    if previous != inventory["journal_head"]:
        checks["errors"].append("journal final head mismatch")
    delivered = {}
    with gzip.open(path / "public/observations.jsonl.gz", "rt", encoding="utf-8") as stream:
        for line in stream:
            packet = strict_loads(line)
            public_copy(packet["observation"])
            obs = packet["observation"]
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
        for sequence,frame in data["frames"].items():
            # A final safety-abort/timeout capture can remain private, while
            # report evidence must always refer to a delivered public capture.
            key = (frame["epoch"],int(sequence))
            if key in delivered and frame["delivered_rgb_sha256"] != delivered[key]:
                checks["errors"].append(f"{file.name}/{sequence}: delivered RGB hash mismatch")
        worker = strict_loads(file.with_name(file.name.replace("-ledger", "-worker")).read_bytes())
        if worker["exit_code"] != 0:
            checks["errors"].append(f"{file.name}: policy worker exit {worker['exit_code']}")
        ledgers.append((file.stem, data, score))
    # Software readback intervals and availability are real monotonic clocks;
    # they do not claim per-exposure hardware time or global physics lockstep.
    intervals, previous_end, raw_count = [], -1, 0
    with gzip.open(path / "private/host.jsonl.gz", "rt", encoding="utf-8") as stream:
        for line in stream:
            raw = strict_loads(line)
            start,end,available = (raw.get(key) for key in ("capture_start_wall_ns","capture_end_wall_ns","available_wall_ns"))
            if not all(type(value) is int for value in (start,end,available)) or not previous_end <= start <= end <= available:
                checks["errors"].append(f"capture {raw_count}: non-monotonic readback/availability")
            else:
                intervals.append((end-start)/1e9)
                previous_end = available
            raw_count += 1
    checks["native_capture_packets"] = raw_count
    checks["readback_interval_s"] = {"median":statistics.median(intervals),"max":max(intervals)} if intervals else None
    if manifest.get("track") == "sar_human_dev_static_synthetic":
        for name, field in (("asset-manifest.json","asset_manifest_sha256"),("human-pose-manifest.json","derived_pose_manifest_sha256")):
            if digest(path / name) != manifest[field]: checks["errors"].append(name+": manifest mismatch")
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
    checks["controls"] = controls
    checks["status"] = "PASS_OFFLINE_AUDIT" if checks["inventory"] and checks["source_snapshot"] and checks["engine_clean_exit"] and ledgers and not checks["errors"] else "FAIL"
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
