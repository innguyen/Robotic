"""Acquire small, pinned research inputs; never downloads an environment pack.

Cesium Man: Cesium, CC BY 4.0 (logo removed from derived local materials).
YOLOv5n: official v7.0 release; source distribution declares GPL-3.0.
Every response is capped; the existing production checkpoint is untouched.
"""
import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[2]
DEST = ROOT / "experiments/ue5_s4/assets/source"
CESIUM_REF = "e124e8d8d7330a5372bc2b4d00d39a4a9d7997ea"


def acquire(url, name, limit):
    path = DEST / name
    if not path.is_file():
        with urllib.request.urlopen(url, timeout=60) as response:
            data = response.read(limit + 1)
        if len(data) > limit:
            raise RuntimeError("download exceeds small-input cap: " + name)
        path.write_bytes(data)
    data = path.read_bytes()
    if len(data) > limit:
        raise RuntimeError("oversized cached input")
    return {"file": name, "url": url, "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}


if __name__ == "__main__":
    DEST.mkdir(parents=True, exist_ok=True)
    base = f"https://raw.githubusercontent.com/KhronosGroup/glTF-Sample-Assets/{CESIUM_REF}/Models/CesiumMan/"
    files = [acquire(base + "glTF/" + name, name, 2_000_000)
             for name in ("CesiumMan.gltf", "CesiumMan_data.bin", "CesiumMan_img0.jpg")]
    files += [acquire(base + "README.md", "CesiumMan-LICENSE-source.md", 100_000)]
    files += [acquire("https://raw.githubusercontent.com/ultralytics/yolov5/915bbf294bb74c859f0b41f1c23bc395014ea679/LICENSE", "YOLOv5-v7-LICENSE.txt", 100_000)]
    files += [acquire("https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.onnx", "yolov5n-v7.onnx", 10_000_000)]
    (DEST.parent / "manifest.json").write_text(json.dumps({"files": files, "human": {"author": "Cesium", "license": "CC-BY-4.0", "source_commit": CESIUM_REF,
        "modifications": "Offline pose baking, coordinate conversion, replacement clothing/skin materials; original logo texture not used for rendering"},
        "detector": {"source_commit": "915bbf294bb74c859f0b41f1c23bc395014ea679", "release": "v7.0", "upstream_license": "GPL-3.0", "purpose": "local research inference; no relicensing claim"}}, indent=2), encoding="utf-8")
    print(json.dumps(files, indent=2))
