"""Exclusive run folders, lossless public/private logs and SHA256 inventories."""
import hashlib
import json
from pathlib import Path

from .contracts import canonical


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


class RunStore:
    def __init__(self, path):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / "public").mkdir()
        (self.path / "private").mkdir()
        self.chain = "0" * 64
        self.journal = (self.path / "journal.jsonl").open("x", encoding="utf-8", newline="\n")

    def event(self, event):
        record = {"previous_sha256": self.chain, "event": event}
        self.chain = hashlib.sha256(canonical(record)).hexdigest()
        self.journal.write(canonical({**record, "sha256": self.chain}).decode() + "\n")
        self.journal.flush()

    def close(self):
        self.journal.close()
        files = {str(path.relative_to(self.path)).replace("\\", "/"): digest(path)
                 for path in sorted(self.path.rglob("*")) if path.is_file() and path.name != "inventory.json"}
        write_json(self.path / "inventory.json", {"files": files, "journal_head": self.chain})


def verify_inventory(path):
    path = Path(path)
    inventory = json.loads((path / "inventory.json").read_text(encoding="utf-8"))
    return all((path / name).is_file() and digest(path / name) == value for name, value in inventory["files"].items())
