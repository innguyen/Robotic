"""Archive completed S4 attempts, including failed startup/visual/search attempts.

Never repairs original runs, overwrites an archive, or converts a partial index
into portable evidence. Execute after the sequential UE runtime queue finishes.
"""
import argparse
import datetime
import json
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from sar_benchmark.artifacts import digest,verify_inventory,write_json
from sar_benchmark.archive_run import archive
from sar_benchmark.audit_run import audit
from sar_benchmark.summarize_run import summarize
from sar_benchmark.visual_qa import export

ATTEMPTS=(
    ("s4-assets-before","startup-sandbox-failure",False),
    ("s4-assets-before-authorized","baseline-before",False),
    ("s4-assets-import-01","import-01",False),
    ("s4-assets-visual-02","visual-02",False),
    ("s4-assets-search-90s","search-open-90s",True),
    ("s4-assets-search-closed-90s","search-closed-90s",True),
    ("s4-assets-final-poses","final-poses",True),
    ("s4-legacy-comparison","legacy-comparison",True),
)

CONTINUATION=(
    ("s4-assets-native-geometry-retry","outdoor-native",True),
    ("s4-assets-seated-alignment","seated-alignment",True),
    ("s4-legacy-pilot-revalidation","legacy-pilot-54",False),
    ("s4-assets-indoor-native-final","indoor-native",True),
)

def collect(target, attempts=ATTEMPTS):
    import cv2
    import numpy
    target=Path(target);target.mkdir(parents=True,exist_ok=True)
    for name,label,frames in attempts:
        if not (ROOT/"experiments/ue5_s4/runs"/name/"inventory.json").exists():
            raise ValueError("attempt still running or unclosed: "+name)
    catalogue=[]
    for name,label,frames in attempts:
        run=ROOT/"experiments/ue5_s4/runs"/name
        audit_file=target/(label+"-final-audit.json")
        if not audit_file.exists():write_json(audit_file,audit(run))
        result=json.loads(audit_file.read_text())
        bundle=target/label
        if not bundle.exists():
            if not (run/"private/host.jsonl.gz").exists():
                if not verify_inventory(run):raise ValueError("failed attempt inventory corrupt")
                shutil.copytree(run,bundle,ignore=shutil.ignore_patterns("__pycache__","capture-tmp","*.log"))
                write_json(bundle/"archive-provenance.json",{"kind":"FAILED_STARTUP_METADATA_COPY",
                    "source_run_absolute":str(run),"source_inventory_sha256":digest(run/"inventory.json"),
                    "note":"No sensor stream was created; this is failed startup evidence, never an accepted episode."})
            else:
                # Keep final acceptance raw portable when each gzip remains
                # below the repository service's individual-file limit.
                raw_files=[run/"public/observations.jsonl.gz",run/"private/host.jsonl.gz"]
                final=label in ("search-closed-90s","final-poses","outdoor-native","seated-alignment","indoor-native")
                cap=160 if final and all(p.stat().st_size<95*1024**2 for p in raw_files) else 90
                archive(run,bundle,max_raw_mib=cap)
        summary=None
        if (run/"results.json").exists():
            summary_file=target/(label+"-summary.json")
            if not summary_file.exists():write_json(summary_file,summarize(run,include_failed=True))
            summary=json.loads(summary_file.read_text())
        if frames and (run/"private/host.jsonl.gz").exists():
            frame_folder=target/(label+"-frames")
            if not frame_folder.exists():export(run,frame_folder,ROOT/"experiments/ue5_s4/assets/source/yolov5n-v7.onnx")
        catalogue.append({"run":name,"label":label,"audit":result["status"],
            "data_integrity":result["data_integrity_status"],"calibration":result.get("requested_calibration_status"),
            "summary":summary,"archive_provenance":json.loads((bundle/"archive-provenance.json").read_text())})
        print(label,result["status"],flush=True)
    analysis=target/"analysis-source";analysis.mkdir(exist_ok=False)
    for file in sorted((ROOT/"sar_benchmark").glob("*.py")):
        shutil.copyfile(file,analysis/file.name)
    write_json(target/"catalogue.json",{"created_utc":datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "python":sys.version,"numpy":numpy.__version__,"opencv":cv2.__version__,"attempts":catalogue,
        "analysis_source_sha256":{p.name:digest(p) for p in sorted(analysis.glob("*.py"))},
        "scope":"All new attempts retained; audit PASS is trace/runtime integrity, not detector coverage, photorealism, full frozen matrix or S3/S5 acceptance"})


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--continuation",action="store_true",help="collect the native-geometry, seated alignment and legacy revalidation runs")
    args=parser.parse_args();collect(args.out, CONTINUATION if args.continuation else ATTEMPTS)
