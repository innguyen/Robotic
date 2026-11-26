"""Render a readable RGB/depth/visibility panel from a verified QA export."""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from sar_benchmark.artifacts import digest, write_json


def render(gallery, label, out):
    index = json.loads((gallery / "index.json").read_bytes())
    frame = next(item for item in index["frames"] if item["label"] == label)
    if "delivered_png_sha256" not in frame:
        raise ValueError("panel requires a delivered robot frame")
    inventory = json.loads((gallery / "inventory.json").read_bytes())
    names = [label + suffix for suffix in ("-robot.png", "-depth-mm.png", "-v0-visible.png")]
    sources = {name: digest(gallery / name) for name in names}
    if any(sha != inventory[name] for name, sha in sources.items()):
        raise ValueError("source frame differs from gallery inventory")
    rgb, depth, mask = [cv2.imread(str(gallery / name), cv2.IMREAD_UNCHANGED) for name in names]
    if depth.dtype != np.uint16 or depth.shape != mask.shape or rgb.shape[:2] != depth.shape:
        raise ValueError("expected matching RGB, native uint16 millimetres and binary mask")
    maximum_mm = 10000
    intensity = np.rint(depth.astype(float).clip(0, maximum_mm)*255/maximum_mm).astype(np.uint8)
    z = cv2.applyColorMap(intensity, cv2.COLORMAP_VIRIDIS)
    z[depth == 0] = 0
    height, width = depth.shape
    canvas = np.full((height + 88, 3*width, 3), 28, np.uint8)
    for column, (picture, title) in enumerate(zip(
            (rgb, z, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)),
            ("Delivered RGB", "Native optical Z (m)", "Private visible mask: v0"))):
        left = column*width
        canvas[32:32+height, left:left+width] = picture
        cv2.putText(canvas, title, (left+7, 22), cv2.FONT_HERSHEY_SIMPLEX, .51, (245,245,245), 1, cv2.LINE_AA)
    gradient = np.linspace(0,255,width-20).astype(np.uint8)[None, :]
    canvas[height+41:height+51, width+10:2*width-10] = cv2.applyColorMap(gradient,cv2.COLORMAP_VIRIDIS)
    info = f"Epoch {frame['epoch']}  tick {frame['tick']}  |  scan hits {frame['scan_hits']}"
    for text, x in ((info, 7), ("0 m       black = invalid       10 m", width+7),
                    (f"Visible pixels: {np.count_nonzero(mask)}", 2*width+7)):
        cv2.putText(canvas, text, (x,height+74), cv2.FONT_HERSHEY_SIMPLEX, .44, (234,234,234), 1, cv2.LINE_AA)
    out.parent.mkdir(parents=True,exist_ok=True)
    ok, png = cv2.imencode(".png", canvas)
    if not ok:
        raise RuntimeError("PNG encoding failed")
    with out.open("xb") as stream:
        stream.write(png.tobytes())
    write_json(out.with_suffix(".json"), {"source_gallery":str(gallery.relative_to(ROOT)),
        "source_inventory_sha256":digest(gallery/"inventory.json"),"input_png_sha256":sources,
        "epoch":frame["epoch"],"tick":frame["tick"],"frame_role":frame["role"],
        "depth_scale_mm":[0,maximum_mm],"invalid_depth_color":"black","output_sha256":digest(out)})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gallery",type=Path)
    parser.add_argument("label")
    parser.add_argument("--out",type=Path,required=True)
    args = parser.parse_args()
    render(args.gallery.resolve(),args.label,args.out)
