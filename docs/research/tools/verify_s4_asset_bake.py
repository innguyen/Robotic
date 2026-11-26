"""Rebuild both asset libraries from locked inputs and compare exact bytes."""
import argparse
import contextlib
import io
from pathlib import Path
import shutil
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from experiments.ue5_s4 import bake_anatomy, bake_environment
from sar_benchmark.artifacts import digest, write_json


def verify(module, fresh, groups, manifests):
    previous = module.BASE
    source = previous.resolve()
    shutil.copytree(source/"source",fresh/"source")
    shutil.copyfile(source/"lock.json",fresh/"lock.json")
    try:
        module.BASE = fresh
        with contextlib.redirect_stdout(io.StringIO()):
            module.bake()
    finally:
        module.BASE = previous
    def outputs(root):
        return {f.relative_to(root) for g in groups for f in (root/g).iterdir() if f.is_file()} | manifests
    if outputs(source) != outputs(fresh):
        raise ValueError(f"derived file sets differ for {source.name}")
    hashes = {}
    for name in sorted(outputs(fresh)):
        sha = digest(fresh/name)
        if sha != digest(source/name):
            raise ValueError(f"rebake changed {name}")
        hashes[name.as_posix()] = sha
    return hashes


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out",type=Path,required=True)
    args = parser.parse_args()
    if args.out.exists():
        parser.error("output must be new")
    with tempfile.TemporaryDirectory() as folder:
        fresh = Path(folder)
        records = {"environment":verify(bake_environment,fresh/"environment",["derived"],{Path("meshes.json")}),
                   "anatomy":verify(bake_anatomy,fresh/"anatomy",["derived","dressed"],set())}
    write_json(args.out,{"status":"PASS","sha256":records})
    print("Reproduced",sum(map(len,records.values())),"files")
