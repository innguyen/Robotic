"""Restore a small, reviewed CC0 asset selection; never download a scene pack.

The committed lock pins every byte. --create-lock is an explicit maintainer
operation, separate from restoration. Total input cap: 32 MiB; per file: 5 MiB.
"""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

BASE = Path(__file__).parent / "assets/environment"
MODELS = ("concrete_road_barrier_02", "painted_wooden_chair_02", "wooden_crate_01")
TEXTURES = ("concrete_debris", "broken_brick_wall", "cracked_concrete")
LIMIT = 5 * 1024**2
TOTAL = 32 * 1024**2


def fetch(url, cap):
    if not url.startswith(("https://api.polyhaven.com/", "https://dl.polyhaven.org/")):
        raise ValueError("unreviewed download host")
    request = urllib.request.Request(url, headers={"User-Agent": "Robotic-S4-Research/1.0"})
    with urllib.request.urlopen(request, timeout=45) as response:
        data = response.read(cap + 1)
    if len(data) > cap:
        raise ValueError("small asset cap exceeded")
    return data


def create_lock():
    lock = {"license": "CC0-1.0", "license_url": "https://polyhaven.com/license",
            "review_date": "2026-11-26", "resolution": "1k", "files": [], "assets": {}}
    planned = []
    for name in MODELS + TEXTURES:
        api = "https://api.polyhaven.com/files/" + name
        document = json.loads(fetch(api, 200_000))
        lock["assets"][name] = {"source": "https://polyhaven.com/a/" + name, "api": api,
                                "kind": "model" if name in MODELS else "texture"}
        if name in MODELS:
            item = document["gltf"]["1k"]["gltf"]
            planned.append((name + "/model.gltf", item))
            # Only mesh binary is needed; textures are explicitly chosen below.
            for relative, part in item["include"].items():
                if relative.endswith(".bin"):
                    planned.append((name + "/" + Path(relative).name, part))
        for channel in ("Diffuse", "nor_dx", "Rough"):
            planned.append((name + "/" + channel + ".jpg", document[channel]["1k"]["jpg"]))
    if sum(item["size"] for _, item in planned) > TOTAL:
        raise ValueError("total asset plan exceeds 32 MiB")
    for relative, item in planned:
        if item["size"] > LIMIT:
            raise ValueError("oversized planned asset")
        data = fetch(item["url"], item["size"])
        if len(data) != item["size"] or hashlib.md5(data).hexdigest() != item["md5"]:
            raise ValueError("upstream size/MD5 mismatch: " + relative)
        target = BASE / "source" / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        lock["files"].append({"file": relative, "url": item["url"], "bytes": len(data),
                              "upstream_md5": item["md5"], "sha256": hashlib.sha256(data).hexdigest()})
        print(relative, len(data), flush=True)
    with (BASE / "lock.json").open("x", encoding="utf-8") as stream:
        json.dump(lock, stream, indent=2)
        stream.write("\n")


def restore():
    lock = json.loads((BASE / "lock.json").read_text())
    if sum(item["bytes"] for item in lock["files"]) > TOTAL:
        raise ValueError("total cap exceeded")
    for item in lock["files"]:
        target = (BASE / "source" / item["file"]).resolve()
        if not target.is_relative_to((BASE / "source").resolve()) or not 0 < item["bytes"] <= LIMIT:
            raise ValueError("invalid locked path/size")
        data = target.read_bytes() if target.exists() else fetch(item["url"], item["bytes"])
        if len(data) != item["bytes"] or hashlib.sha256(data).hexdigest() != item["sha256"]:
            raise ValueError("locked asset mismatch: " + item["file"])
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
    print("CC0 environment inputs verified:", len(lock["files"]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--create-lock", action="store_true")
    args = parser.parse_args()
    BASE.mkdir(parents=True, exist_ok=True)
    if args.create_lock:
        if (BASE / "lock.json").exists():
            parser.error("lock already exists; restoration cannot update provenance")
        create_lock()
    else:
        restore()
