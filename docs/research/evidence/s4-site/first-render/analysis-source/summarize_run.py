"""Descriptive dev-pilot summaries, retaining abstention and auxiliary failures."""
import argparse
from collections import defaultdict
from pathlib import Path
import statistics

from .artifacts import digest, verify_inventory, write_json
from .contracts import strict_loads
from .statistics import rq1


def percentile(values, fraction):
    if not values: return None
    ordered = sorted(values); index = (len(ordered)-1)*fraction
    low = int(index); high = min(low+1,len(ordered)-1)
    return ordered[low]+(ordered[high]-ordered[low])*(index-low)


def summarize(run, include_failed=False):
    if not verify_inventory(run): raise ValueError("input inventory failed")
    manifest = strict_loads((run / "manifest.json").read_bytes())
    result_file = run / "results.json"
    if result_file.is_file():
        result = strict_loads(result_file.read_bytes())
    elif include_failed and (run / "failure.json").is_file():
        failure = strict_loads((run / "failure.json").read_bytes())
        rows = [strict_loads(p.read_bytes()) for p in sorted(run.glob("e*-metrics.json"))]
        if len(rows) != failure["completed_episodes"]: raise ValueError("failed-run episode count mismatch")
        exits = [strict_loads(line)["event"] for line in (run / "journal.jsonl").read_text().splitlines()
                 if strict_loads(line)["event"].get("type")=="engine_exit"]
        calibration = strict_loads((run / "calibration.json").read_bytes()) if (run / "calibration.json").exists() else None
        result = {"rows":rows,"status":"FAILED_CALIBRATION" if calibration and calibration["status"]!="PASS_ENGINE_CALIBRATION" else "FAILED_RECORDED_ATTEMPT",
                  "engine_exit_code":exits[-1]["exit_code"] if exits else None,"wall_duration_s":None,
                  "rq1":rq1([r for r in rows if r["kind"]=="clean"]) if manifest["suite"] in ("pilot","sar-pilot") else None}
    else:
        raise ValueError("no completed result; use --include-failed for explicitly failed-run outcomes")
    if not include_failed and (not result["status"].startswith("COMPLETED_") or result["engine_exit_code"] != 0):
        raise ValueError("summary requires a completed clean-exit run")
    groups = defaultdict(list)
    for row in result["rows"]:
        key = (row["method"],row["kind"],row["perception"],row["odometry"])
        groups[key].append(row)
    table = []
    for (method,kind,p,n),rows in sorted(groups.items()):
        metrics = [r["metrics"] for r in rows]
        slr = [m["safe_localization_recall"] for m in metrics if m["safe_localization_recall"] is not None]
        errors = [v for m in metrics for v in m["localization_xy_m"]]
        latencies = [v for r in rows for v in r["agent_latency_s"]]
        table.append({"method":method,"kind":kind,"P":p,"N":n,"episodes":len(rows),
            "auxiliary_failure_challenge":kind=="blocked","safe_localization_recall":statistics.mean(slr) if slr else None,
            "tp":sum(m["tp"] for m in metrics),"fp":sum(m["fp"] for m in metrics),
            "unsafe_episodes":sum(m["unsafe"] for m in metrics),
            "absent_false_alarm_rate":statistics.mean(m["absent_false_alarm"] for m in metrics) if kind=="absent" else None,
            "path_mean_m":statistics.mean(m["path_m"] for m in metrics),
            "localization_matched_count":len(errors),"localization_median_m":percentile(errors,.5),"localization_p90_m":percentile(errors,.9),
            "time_integrated_recall":statistics.mean(m["time_integrated_recall"] for m in metrics if m["time_integrated_recall"] is not None) if slr else None,
            "action_latency_p50_s":percentile(latencies,.5),"action_latency_p95_s":percentile(latencies,.95)})
    paired = defaultdict(dict)
    for row in result["rows"]:
        if row["kind"] == "clean" and row["perception"]==1 and row["odometry"]==1 and row["method"] in ("frontier","verify"):
            paired[row["family"],row["pair_id"]][row["method"]] = row["metrics"]["safe_localization_recall"]
    families = defaultdict(list)
    for (family,pair),values in paired.items():
        if set(values) != {"frontier","verify"}: raise ValueError("incomplete RQ2 paired block")
        families[family].append(values["verify"]-values["frontier"])
    family_deltas = {name:statistics.mean(values) for name,values in families.items()}
    return {"scope":"descriptive development pilot only; no significance, generalization or fixed-FAR claim",
            "run_status":result["status"],"completed_episodes":len(result["rows"]),"engine_exit_code":result["engine_exit_code"],
            "results_sha256":digest(result_file) if result_file.is_file() else None,"source_inventory_sha256":digest(run / "inventory.json"),"table":table,"rq1":result.get("rq1"),
            "rq2_joint_verify_minus_frontier":{"family_deltas":family_deltas,"mean":statistics.mean(family_deltas.values()) if family_deltas else None,
                "ci95":None,"reason":"dev corpus is not an independent held-out family sample"},
            "wall_duration_s":result["wall_duration_s"],"task_budget_s":manifest["ticks"]*manifest.get("action_dt_ns",100_000_000)/1e9,
            "action_dt_ns":manifest.get("action_dt_ns",100_000_000),"scientific_main_eligible":manifest["scientific_main_eligible"]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path);parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--include-failed",action="store_true",help="retain failed-run outcomes, explicitly labelled; never grants acceptance")
    args = parser.parse_args(); value = summarize(args.run,args.include_failed)
    write_json(args.out,value)
    print("descriptive groups:",len(value["table"]))
