"""Offline recomputation, independent reference and adversarial real-trace replay."""
import argparse
import copy
import gzip
import hashlib
import json
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
    for line in (path / "journal.jsonl").read_text(encoding="utf-8").splitlines():
        record = strict_loads(line)
        if record["previous_sha256"] != previous:
            checks["errors"].append("journal chain discontinuity")
        actual = hashlib.sha256(canonical({"previous_sha256": previous, "event": record["event"]})).hexdigest()
        if actual != record["sha256"]:
            checks["errors"].append("journal hash mismatch")
        previous = record["sha256"]
    with gzip.open(path / "public/observations.jsonl.gz", "rt", encoding="utf-8") as stream:
        for line in stream:
            packet = strict_loads(line)
            public_copy(packet["observation"])
            checks["public_packets"] += 1
    ledgers = []
    for file in sorted((path / "private").glob("*-ledger.json")):
        data = strict_loads(file.read_bytes())
        score, reference = evaluate(data), reference_score(data)
        for key in reference:
            if score[key] != reference[key]:
                checks["errors"].append(f"{file.name}/{key}: reference disagreement")
        checks["reference_episodes"] += 1
        ledgers.append((file.stem, data, score))
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
    checks["status"] = "PASS_OFFLINE_AUDIT" if checks["inventory"] and checks["source_snapshot"] and not checks["errors"] else "FAIL"
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
