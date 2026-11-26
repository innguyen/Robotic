"""Index an interrupted attempt without repairing or changing its evidence."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from sar_benchmark.artifacts import digest,write_json
from sar_benchmark.contracts import canonical,strict_loads
from sar_benchmark.evaluator import evaluate
from sar_benchmark.reference import reference_score


def inspect(run,target):
    run,target=Path(run).resolve(),Path(target).resolve()
    target.mkdir(parents=True,exist_ok=False)
    manifest=strict_loads((run/"manifest.json").read_bytes())
    source_ok=all((run/"source"/name).is_file() and digest(run/"source"/name)==sha for name,sha in manifest["source_sha256"].items())
    previous="0"*64;events=[];journal_errors=[]
    for line in (run/"journal.jsonl").read_text().splitlines():
        try:record=strict_loads(line)
        except ValueError as error:
            journal_errors.append(str(error));break
        actual=hashlib.sha256(canonical({"previous_sha256":previous,"event":record["event"]})).hexdigest()
        if record["previous_sha256"]!=previous or actual!=record["sha256"]:journal_errors.append("chain/hash mismatch")
        previous=record["sha256"];events.append(record["event"])
    rows=[]
    unscored_ledgers=[]
    for file in sorted((run/"private").glob("*-ledger.json")):
        episode=file.name.removesuffix("-ledger.json")
        metric_file=run/(episode+"-metrics.json")
        if not metric_file.exists():
            unscored_ledgers.append(episode)
            continue
        ledger=strict_loads(file.read_bytes());score=evaluate(ledger);reference=reference_score(ledger)
        row=strict_loads(metric_file.read_bytes())
        rows.append({"episode":row["episode"],"method":row["method"],"kind":row["kind"],
            "metrics_equal_recomputed":row["metrics"]==score,"independent_reference_equal":all(score[k]==v for k,v in reference.items()),"metrics":score})
    streams={}
    for name in ("public/observations.jsonl.gz","private/host.jsonl.gz"):
        count=0;last=None;error=None
        try:
            with gzip.open(run/name,"rt",encoding="utf-8") as stream:
                for line in stream:
                    record=json.loads(line);count+=1
                    value=record.get("observation",record)
                    last={k:value.get(k) for k in ("epoch","tick")}
        except (EOFError,OSError,ValueError) as exc:error=type(exc).__name__+": "+str(exc)
        streams[name]={"bytes":(run/name).stat().st_size,"sha256":digest(run/name),
                       "parseable_records":count,"last_record":last,"stream_complete":error is None,"error":error}
    inventory={str(p.relative_to(run)).replace("\\","/"):{"sha256":digest(p),"bytes":p.stat().st_size}
               for p in sorted(run.rglob("*")) if p.is_file() and not {"__pycache__","capture-tmp"}.intersection(p.parts)}
    # Metadata/source are copied, but large raw bytes remain at the indexed
    # original path. This is deliberately NOT an accepted RunStore inventory.
    for relative in inventory:
        if relative in streams or relative.endswith(".log"):continue
        destination=target/relative;destination.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(run/relative,destination)
    scheduled_file=run/"private/scheduled-scenarios.json"
    scheduled=len(strict_loads(scheduled_file.read_bytes()))*len(manifest["methods"]) if scheduled_file.exists() else None
    report={"status":"INTERRUPTED_PARTIAL_NOT_ACCEPTED","source_run_absolute":str(run),
        "scheduled_episodes":scheduled,"completed_ledgers_independently_recomputed":len(rows),
        "ledgers_without_final_metrics":unscored_ledgers,
        "source_snapshot_matches_manifest":source_ok,"journal_chain_errors":journal_errors,
        "engine_exit_events":[event for event in events if event.get("type")=="engine_exit"],
        "last_journal_event":events[-1] if events else None,"calibration_present":(run/"calibration.json").exists(),
        "results_present":(run/"results.json").exists(),"original_inventory_present":(run/"inventory.json").exists(),
        "streams":streams,"episodes":rows,"all_original_file_hashes":inventory,
        "archive_kind":"PARTIAL_LOCAL_RAW_INDEX_NO_ACCEPTANCE_INVENTORY",
        "note":"Original attempt unchanged; parseable records/recomputed metrics do not establish clean lifecycle, full delivery audit or run acceptance. Interruption cause is unknown."}
    write_json(target/"partial-inspection.json",report)
    print(report["status"],len(rows),"/",scheduled)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("run",type=Path);parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args();inspect(args.run,args.out)
