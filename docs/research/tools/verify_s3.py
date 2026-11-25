"""Check S3 documents, JSON semantics and preserved tracked bytes.

This is artifact QA, not a full JSON Schema validator or UE5 runtime test.
Use PowerShell Test-Json for the separate schema validation recorded in audit.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[3]
RESEARCH = ROOT / "docs/research"


def no_duplicates(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON key: {key}")
        result[key] = value
    return result


def invalid_constant(value):
    raise ValueError(f"Non-finite JSON value: {value}")


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"),
                      object_pairs_hook=no_duplicates, parse_constant=invalid_constant)


def verify():
    errors = []
    baseline = read_json(RESEARCH / "evidence/protected-before.json")
    checked = 0
    for name, digest in baseline["files"].items():
        path = ROOT / name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            errors.append(f"Protected file changed/missing: {name}")
        checked += 1
    current_head = subprocess.check_output(
        ["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip()
    if current_head != baseline["head"]:
        errors.append("Git HEAD changed during S3")
    links = 0
    docs = sorted(RESEARCH.glob("*.md"))
    for doc in docs:
        content = doc.read_text(encoding="utf-8")
        if content.count("```") % 2:
            errors.append(f"Unbalanced code fences: {doc.name}")
        # All artifact links here use ordinary Markdown (no parentheses in URLs).
        for target in re.findall(r"\]\(([^)]+)\)", content):
            url = urlsplit(target.strip("<>"))
            if url.scheme or not url.path:
                continue
            resolved = (doc.parent / unquote(url.path)).resolve()
            if not resolved.is_relative_to(ROOT) or not resolved.exists():
                errors.append(f"Broken/outside-workspace link: {doc.name}: {target}")
            links += 1
    json_files = sorted((RESEARCH / "contracts").glob("*.json"))
    for path in json_files:
        read_json(path)
    sample = read_json(RESEARCH / "contracts/scenario.example.json")
    public = sample["public_profile"]
    private = sample["private_scenario"]
    if public["action_dt_ns"] % public["physics_dt_ns"]:
        errors.append("Action tick is not an integer number of physics ticks")
    if public["budget_ns"] % public["action_dt_ns"]:
        errors.append("Budget is not an integer number of actions")
    xmin, ymin, xmax, ymax = private["operating_bounds_xy_m"]
    if not (xmin < xmax and ymin < ymax):
        errors.append("Invalid operating bounds")
    victim_ids = [item["gt_id"] for item in private["victims"]]
    if len(victim_ids) != len(set(victim_ids)):
        errors.append("Duplicate victim IDs")
    if not sample["design_only"] or private["admissibility"] != "unverified":
        errors.append("Example misrepresents runtime/admissibility status")
    forbidden_public = {"victims", "seed_streams", "condition", "scene_id", "scene_family_id",
                        "source_site_id", "target_pose", "segmentation", "navmesh"}
    if forbidden_public.intersection(public):
        errors.append("Private authoring metadata leaked into public profile")
    if sample["provenance"]["binary_sha256"] is not None:
        errors.append("Design example must not invent a UE binary hash")
    for path in (RESEARCH / "tools").glob("*.py"):
        compile(path.read_text(encoding="utf-8"), str(path), "exec")
    authored = [path for path in RESEARCH.rglob("*")
                if path.is_file() and "evidence" not in path.relative_to(RESEARCH).parts]
    for path in authored:
        lines = path.read_text(encoding="utf-8").splitlines()
        if any(line.rstrip() != line for line in lines):
            errors.append(f"Trailing whitespace: {path.relative_to(ROOT)}")
    artifact_hashes = {str(path.relative_to(ROOT)).replace("\\", "/"):
                       hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(authored)}
    result = {
        "status": "PASS" if not errors else "FAIL",
        "scope": "artifact QA; not JSON Schema or UE5 runtime acceptance",
        "protected_files_checked": checked, "head_unchanged": current_head == baseline["head"],
        "markdown_documents_checked": len(docs), "local_links_checked": links,
        "contract_json_files_checked": len(json_files),
        "authored_files_sha256": artifact_hashes,
        "errors": errors,
        "ue5_runtime": "UNVERIFIED_NOT_RUN",
    }
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    if args.out.exists():
        parser.error("Output exists; choose a new verification filename")
    result = verify()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["status"] == "PASS" else 1)
