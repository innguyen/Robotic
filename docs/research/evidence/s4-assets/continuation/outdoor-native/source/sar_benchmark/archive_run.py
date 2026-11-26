"""Create a new evidence archive without changing the original run.

Large raw streams stay at their recorded local path, with byte hashes. Such an
index is explicitly partial and cannot be mistaken for a portable audited run.
"""
import argparse
import json
from pathlib import Path
import shutil

from .artifacts import digest, verify_inventory, write_json


def archive(source, target, max_raw_mib=64):
    source,target = Path(source).resolve(),Path(target).resolve()
    if target == source or source in target.parents: raise ValueError("archive must be outside source run")
    if not verify_inventory(source): raise ValueError("source inventory failed")
    inventory = json.loads((source / "inventory.json").read_text(encoding="utf-8"))
    raw_names = ("public/observations.jsonl.gz","private/host.jsonl.gz")
    partial = sum((source / name).stat().st_size for name in raw_names) > max_raw_mib*1024**2
    target.mkdir(parents=True,exist_ok=False)
    omitted = {}
    for name,sha in inventory["files"].items():
        if "capture-tmp" in Path(name).parts or partial and name in raw_names:
            omitted[name] = {"sha256":sha,"bytes":(source / name).stat().st_size}
            continue
        destination = target / name; destination.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(source / name,destination)
    shutil.copyfile(source / "inventory.json",target / "original-run-inventory.json")
    write_json(target / "archive-provenance.json",{
        "kind":"PARTIAL_LOCAL_RAW_INDEX" if partial else "PORTABLE_LOSSLESS_RAW_ARCHIVE",
        "source_run_absolute":str(source),"source_inventory_sha256":digest(source / "inventory.json"),
        "source_run_repository_relative":str(source.relative_to(Path(__file__).resolve().parents[1])).replace("\\","/") if source.is_relative_to(Path(__file__).resolve().parents[1]) else None,
        "omitted":omitted,"raw_streams_included":not partial,
        "note":"Original run untouched. Scratch captures are already losslessly embedded in raw host stream. A partial index requires original local raw files for audit/replay."})
    archived = {str(p.relative_to(target)).replace("\\","/"):digest(p) for p in sorted(target.rglob("*")) if p.is_file()}
    if not partial:
        write_json(target / "inventory.json",{"files":archived,"journal_head":inventory["journal_head"],
                   "excluded_generated":["capture-tmp","__pycache__","*.log retained as engine-log.txt"]})
    else:
        write_json(target / "partial-archive-inventory.json",{"files":archived,"journal_head":inventory["journal_head"]})
    return {"partial":partial,"files":len(archived),"omitted":len(omitted),"target":str(target)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source",type=Path); parser.add_argument("target",type=Path)
    parser.add_argument("--max-raw-mib",type=float,default=64)
    args = parser.parse_args()
    if args.max_raw_mib < 0: parser.error("nonnegative raw cap required")
    print(json.dumps(archive(args.source,args.target,args.max_raw_mib),indent=2))
