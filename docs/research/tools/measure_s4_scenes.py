"""Measure per-scene performance within scored UE episodes from frozen traces."""
import argparse
import gzip
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from sar_benchmark.artifacts import digest, verify_inventory, write_json


def measure(run):
    if not verify_inventory(run):
        raise ValueError(f"inventory mismatch in {run}")
    manifest = json.loads((run / "manifest.json").read_bytes())
    host = json.loads((run / "host-capabilities.json").read_bytes())
    clock = host.get("monotonic_clock", {}).get("implementation")
    shared_clock = bool(clock) and clock == manifest.get("runner_monotonic_clock", {}).get("implementation")
    samples = json.loads((run / "runtime-memory.json").read_bytes())["samples"]
    epochs = {}
    for ledger in sorted((run / "private").glob("*-ledger.json")):
        epoch = json.loads(ledger.read_bytes())["epoch"]
        config = ledger.with_name(ledger.name.replace("-ledger.json", "-scenario.json"))
        epochs[epoch] = json.loads(config.read_bytes())["environment"]

    windows = {}
    with gzip.open(run / "private/host.jsonl.gz", "rt", encoding="utf-8") as stream:
        for line in stream:
            raw = json.loads(line)
            epoch = raw["epoch"]
            if epoch in epochs:
                frame = (raw["capture_start_wall_ns"], raw["render_loop"]["frames"])
                first, _, count = windows.get(epoch, (frame, frame, 0))
                windows[epoch] = (first, frame, count + 1)
    if epochs.keys() != windows.keys():
        raise ValueError(f"missing native frames for a scored epoch in {run}")

    rows = []
    for scene in sorted(set(epochs.values())):
        spans = [window for epoch, window in windows.items() if epochs[epoch] == scene]
        seconds = sum(last[0] - first[0] for first, last, _ in spans) / 1e9
        callbacks = sum(last[1] - first[1] for first, last, _ in spans)
        memory = [s for s in samples if shared_clock and s["status"] == "MEASURED"
                  and any(first[0] <= s["monotonic_ns"] <= last[0] for first, last, _ in spans)]
        rows.append({"environment": scene, "episodes": len(spans),
                     "captures": sum(count for _, _, count in spans),
                     "wall_s": seconds, "engine_callbacks": callbacks,
                     "callback_fps": callbacks / seconds if seconds > 0 else None,
                     "memory_samples": len(memory),
                     "max_sampled_working_set_bytes": max((s["working_set_bytes"] for s in memory), default=None),
                     "max_sampled_private_commit_bytes": max((s["private_commit_bytes"] for s in memory), default=None)})
    return {"run": run.name, "inventory_sha256": digest(run / "inventory.json"),
            "host_runner_clock_matches": shared_clock, "scenes": rows}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", type=Path, nargs="+")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    write_json(args.out, {"runs": [measure(run) for run in args.runs],
                          "scope": "Sum of first-to-last capture windows of scored epochs only. "
                                   "Excludes setup, admission, inter-episode gaps and calibration. "
                                   "Memory is sampled UE process memory, not GPU or lifetime peaks."})
