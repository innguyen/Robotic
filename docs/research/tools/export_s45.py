"""Export sealed S4.5 runs without duplicating their frozen source and raw streams."""
import argparse
import json
import os
from pathlib import Path
import shutil
import sys
from tempfile import TemporaryDirectory

os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR","1")

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from docs.research.tools.measure_s4_scenes import measure
from sar_benchmark.artifacts import digest, verify_inventory, write_json
from sar_benchmark.audit_run import audit
from sar_benchmark.episode_video import export as video
from sar_benchmark.visual_qa import export as frames


def export(run, dest, films):
    if not verify_inventory(run):
        raise ValueError("source run inventory mismatch")
    dest.mkdir(parents=True,exist_ok=False)
    read = lambda path: json.loads(path.read_bytes())
    manifest = read(run/"manifest.json")
    if manifest.get("scene_set") != "reference":
        raise ValueError("expected a reference-site run")
    write_json(dest/"audit.json",audit(run))
    for name in ["manifest.json","results.json","failure.json","calibration.json",
                 "asset-calibration.json","asset-geometry-calibration.json","host-capabilities.json"]:
        if (run/name).exists():
            shutil.copyfile(run/name,dest/name)
    for path in sorted((run/"private").glob("admission-*.json")):
        shutil.copyfile(path,dest/path.name)
    for path in sorted((run/"private").glob("*-routes.json")):
        shutil.copyfile(path,dest/path.name)
    if (run/"results.json").is_file() and list((run/"private").glob("*-ledger.json")):
        perf = measure(run)
        # Consecutive scored native capture starts include policy/IPC wait.
        for row in perf["scenes"]:
            row["observed_intervals_per_wall_s"] = ((row["captures"]-row["episodes"])/row["wall_s"]
                                                     if row["wall_s"] else None)
        write_json(dest/"performance.json",perf)
        frames(run,dest/"frames",ROOT/"experiments/ue5_s4/assets/source/yolov5n-v7.onnx")
        for episode in films:
            video(run,episode,dest/(episode+"-video"))
    files = read(run/"inventory.json")["files"]
    write_json(dest/"provenance.json",dict(
        original_run=run.relative_to(ROOT).as_posix(),inventory_sha256=digest(run/"inventory.json"),
        raw={n:{"sha256":s,"bytes":(run/n).stat().st_size} for n,s in files.items() if n.endswith(".gz")},
        retained="Selected runtime evidence. Full raw streams and source snapshots live in original_run."))
    write_json(dest/"files.json",{p.relative_to(dest).as_posix():digest(p) for p in sorted(dest.rglob("*")) if p.is_file()})


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    parser.add_argument("dest",type=Path)
    parser.add_argument("--video",nargs="*",default=[])
    args=parser.parse_args()
    args.dest.parent.mkdir(parents=True,exist_ok=True)
    if args.dest.exists():
        parser.error("destination must be new")
    with TemporaryDirectory(dir=args.dest.parent) as staging:
        folder=Path(staging)/"export"
        export(args.run.resolve(),folder,args.video)
        folder.rename(args.dest)
