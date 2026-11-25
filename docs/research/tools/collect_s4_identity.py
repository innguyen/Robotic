"""Hash installed engine/plugin/mesh files without copying them into the repo."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from sar_benchmark.artifacts import digest, write_json


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    names = ["Engine/Build/Build.version", "Engine/Binaries/Win64/UnrealEditor-Cmd.exe",
             "Engine/Binaries/Win64/UnrealEditor-D3D11RHI.dll",
             "Engine/Plugins/Experimental/PythonScriptPlugin/PythonScriptPlugin.uplugin",
             "Engine/Plugins/Experimental/PythonScriptPlugin/Binaries/Win64/UnrealEditor-PythonScriptPlugin.dll",
             "Engine/Content/BasicShapes/Cube.uasset", "Engine/Content/BasicShapes/Sphere.uasset"]
    files = {name: {"sha256": digest(args.engine / name), "bytes": (args.engine / name).stat().st_size}
             for name in names if (args.engine / name).is_file()}
    write_json(args.out, {"engine_root": str(args.engine), "build": json.loads((args.engine / names[0]).read_text()),
                         "files": files, "missing": [name for name in names if name not in files],
                         "assets": "Engine files remain local; only hashes and scene-generation recipe are delivered",
                         "model_not_used_in_fixture_pilot": {"path": "models/yolov5n.onnx",
                             "sha256": digest(ROOT / "models/yolov5n.onnx"), "provenance_license": "unverified"}})
