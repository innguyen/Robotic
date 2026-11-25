"""Independent post-run checks; run from any cwd with Python stdlib."""
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import zlib

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parent
REPO = PROJECT.parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def png_rgb(path):
    data = path.read_bytes()
    assert data.startswith(b"\x89PNG\r\n\x1a\n")
    offset, compressed, width, height = 8, b"", None, None
    while offset < len(data):
        size = struct.unpack_from(">I", data, offset)[0]
        kind = data[offset + 4:offset + 8]
        content = data[offset + 8:offset + 8 + size]
        crc = struct.unpack_from(">I", data, offset + 8 + size)[0]
        assert zlib.crc32(kind + content) == crc
        if kind == b"IHDR":
            width, height, depth, color, *_ = struct.unpack(">2I5B", content)
            assert (width, height, depth, color) == (160, 120, 8, 2)
        elif kind == b"IDAT":
            compressed += content
        offset += size + 12
    scanlines = zlib.decompress(compressed)
    assert len(scanlines) == height * (width * 3 + 1)
    assert all(scanlines[y * (width * 3 + 1)] == 0 for y in range(height))
    return b"".join(scanlines[y * (width * 3 + 1) + 1:(y + 1) * (width * 3 + 1)] for y in range(height))


def verify_run(name):
    directory = HERE / name
    invocation = json.loads((directory / "invocation.json").read_text(encoding="utf-8"))
    assert invocation["exit_code"] == invocation["runner_exit"] == 0
    assert invocation["identity_before"] == invocation["identity_after"]
    assert invocation["protected_identity_before"] == invocation["protected_identity_after"]
    for filename, value in invocation["identity_before"].items():
        assert digest(PROJECT / filename) == digest(directory / "source" / filename) == value
    for filename, value in invocation["protected_identity_after"].items():
        assert digest(REPO / filename) == value, filename
    results = json.loads((directory / "results.json").read_text(encoding="utf-8"))
    rendered = invocation["rendered"]
    assert sum(r["status"] == "PASS" for r in results) == (7 if rendered else 5)
    assert sum(r["status"] == "UNVERIFIED" for r in results) == (0 if rendered else 2)
    assert next(r for r in results if r["name"] == "performance")["status"] == "MEASURED"
    records = [json.loads(line) for line in (directory / "observations.jsonl").read_text().splitlines()]
    last_epoch, last_tick, hashes = None, None, set()
    for row in records:
        assert row["protocol"] == 1 and abs(row["dt"] - 1 / 60) < 1e-12
        assert abs(row["simulation_timestamp"] - row["tick"] / 60) < 1e-9
        assert all(row[key]["stamp"] == row["simulation_timestamp"] for key in ("pose", "scan", "camera"))
        assert len(row["scan"]["ranges"]) == 192
        assert row["scan"]["origin"] == [.12, 0, .14]
        assert row["camera"]["origin"] == [.12, 0, .44]
        assert abs(row["pose"]["z"] + row["scan"]["origin"][2] - .25) < 1e-6
        assert abs(row["pose"]["z"] + row["camera"]["origin"][2] - .55) < 1e-6
        assert all(r is None or math.isfinite(r) and 0 <= r <= 6.00001 for r in row["scan"]["ranges"])
        assert not any(key in row for key in ("target", "target_pose", "person_position", "recognition"))
        if row["epoch"] == last_epoch:
            assert row["tick"] >= last_tick
        else:
            assert row["tick"] == 0
        last_epoch, last_tick = row["epoch"], row["tick"]
        assert row["camera"]["valid"] == rendered
        if row["camera"]["valid"]:
            hashes.add(row["camera"]["sha256"])
        request = row["request"]
        if request["method"] == "step":
            ticks = request["kwargs"].get("ticks", 6)
            assert isinstance(ticks, int) and 1 <= ticks <= 600
            values = dict(zip(("vx", "vy", "wz"), request["args"]))
            values.update(request["kwargs"])
            assert all(abs(values.get(key, 0)) <= limit for key, limit in (("vx", .6), ("vy", .4), ("wz", 1)))
    images = list(directory.glob("*.png"))
    for image in images:
        assert hashlib.sha256(png_rgb(image)).hexdigest() in hashes, image
    stderr = (directory / "stderr.txt").read_text(encoding="utf-8")
    assert "SCRIPT ERROR" not in stderr and "Parse Error" not in stderr
    return dict(status="PASS", accepted_observations=len(records), png_sensor_hashes_verified=len(images),
                source_files_verified=len(invocation["identity_before"]), protected_files_verified=len(invocation["protected_identity_before"]),
                functional_pass=sum(r["status"] == "PASS" for r in results),
                unverified=sum(r["status"] == "UNVERIFIED" for r in results), performance="MEASURED")


def main():
    report = {name: verify_run(name) for name in ("run-05-final-rendered", "run-06-final-headless")}
    missing_links = []
    for path in (REPO / "docs/godot-s0-feasibility.md", PROJECT / "README.md"):
        for target in re.findall(r"\]\(([^)]+)\)", path.read_text(encoding="utf-8")):
            if "://" not in target and not (path.parent / target).exists():
                missing_links.append(dict(document=str(path), target=target))
    assert not missing_links, missing_links
    report["documentation_links"] = "PASS"
    report["verifier_sha256"] = digest(Path(__file__))
    (HERE / "verification.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
