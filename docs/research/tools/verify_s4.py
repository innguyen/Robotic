"""Verify existing tracked files against HEAD and compile S4 source, without UE."""
import argparse
import hashlib
import json
import re
from pathlib import Path
import subprocess
import sys
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from sar_benchmark.artifacts import write_json


def collect():
    tracked = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT).decode().split("\0")
    changed = subprocess.check_output(["git", "diff", "--name-only", "HEAD"], cwd=ROOT).decode().splitlines()
    protected = [name for name in tracked if name and not name.startswith("docs/research/")]
    altered = [name for name in changed if name in protected]
    digest = hashlib.sha256()
    for name in protected:
        data = (ROOT / name).read_bytes()
        digest.update(name.encode() + b"\0" + hashlib.sha256(data).digest())
    source = list((ROOT / "sar_benchmark").glob("*.py")) + list((ROOT / "experiments/ue5_s4").glob("*.py"))
    source += list((ROOT / "tests/sar_benchmark").glob("*.py"))
    for path in source:
        compile(path.read_text(encoding="utf-8"), str(path), "exec")
    docs = list((ROOT / "docs/research").glob("*.md"))
    docs += [ROOT / "sar_benchmark/README.md", ROOT / "experiments/ue5_s4/README.md"]
    links, doc_errors = 0, []
    for doc in docs:
        content = doc.read_text(encoding="utf-8")
        if content.count("```") % 2:
            doc_errors.append(str(doc.relative_to(ROOT)) + ": unbalanced fences")
        for target in re.findall(r"\]\(([^)]+)\)", content):
            url = urlsplit(target.strip("<>"))
            if url.scheme or not url.path:
                continue
            links += 1
            resolved = (doc.parent / unquote(url.path)).resolve()
            if not resolved.is_relative_to(ROOT) or not resolved.exists():
                doc_errors.append(str(doc.relative_to(ROOT)) + ": missing " + target)
    return {"protected_tracked_files": len(protected), "protected_sha256_inventory_digest": digest.hexdigest(),
            "protected_files_changed_from_HEAD": altered, "compiled_source_files": len(source),
            "local_links_checked": links, "documentation_errors": doc_errors,
            "status": "PASS" if not altered and not doc_errors else "FAIL",
            "scope": "source preservation/syntax/links only, not engine validation"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = collect()
    write_json(args.out, result)
    print(result["status"], result["protected_tracked_files"], "protected files")
    raise SystemExit(result["status"] != "PASS")
