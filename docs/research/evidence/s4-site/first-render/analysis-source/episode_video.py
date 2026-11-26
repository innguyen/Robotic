"""Export continuous, task-clock RGB video from sealed public observations."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

from .artifacts import digest, verify_inventory, write_json
from .contracts import canonical


def export(run, episode, target):
    if not verify_inventory(run):
        raise ValueError("run inventory mismatch")
    ledger = json.loads((run/"private"/(episode+"-ledger.json")).read_bytes())
    config = json.loads((run/"private"/(episode+"-scenario.json")).read_bytes())
    manifest = json.loads((run/"manifest.json").read_bytes())
    width, height = manifest["rgbd_size"]
    fps = 1e9/manifest["action_dt_ns"]
    target.mkdir(parents=True,exist_ok=False)
    video = target/"robot.avi"
    writer = cv2.VideoWriter(str(video),cv2.VideoWriter_fourcc(*"MJPG"),fps,(width,height))
    if not writer.isOpened():
        raise RuntimeError("Motion JPEG encoder unavailable")
    hashes = []
    try:
        with gzip.open(run/"public/observations.jsonl.gz","rt",encoding="utf-8") as stream:
            for line in stream:
                packet = json.loads(line)
                if packet["episode"] != episode:
                    continue
                obs = packet["observation"]
                tick = len(hashes)
                if obs["epoch"] != ledger["epoch"] or obs["tick"] != tick or obs["sim_time_ns"] != tick*manifest["action_dt_ns"]:
                    raise ValueError("discontinuous episode clock")
                values = obs["rgb"]["data"]
                sha = hashlib.sha256(canonical(values)).hexdigest()
                if sha != ledger["frames"][str(obs["sequence"])]["delivered_rgb_sha256"]:
                    raise ValueError("delivered RGB differs from scored ledger")
                rgb = np.array(values,dtype=np.uint8).reshape(height,width,3)
                writer.write(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
                hashes.append(sha)
    finally:
        writer.release()
    capture = cv2.VideoCapture(str(video))
    count = 0
    try:
        if not math.isclose(capture.get(cv2.CAP_PROP_FPS),fps):
            raise ValueError("incorrect playback clock")
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame.shape != (height,width,3):
                raise ValueError("incorrect playback size")
            count += 1
    finally:
        capture.release()
    if not hashes or count != len(hashes):
        raise ValueError("video playback verification failed")
    metrics = json.loads((run/(episode+"-metrics.json")).read_bytes())["metrics"]
    write_json(target/"video.json",{"source_run":run.name,"episode":episode,"environment":config["environment"],
        "family":config["family"],"run_inventory_sha256":digest(run/"inventory.json"),
        "codec":"MJPG","frames_decoded":count,"size":[width,height],"fps_task_clock":fps,
        "first_tick":0,"last_tick":count-1,"delivered_rgb_sha256_in_order":hashes,
        "file_sha256":digest(video),"metrics":metrics,
        "scope":"Every delivered policy RGB frame, in order, task clock. Motion JPEG is lossy; raw stream retains original bytes."})
    print(episode,config["family"],count,"verified frames",flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    parser.add_argument("episode")
    parser.add_argument("--out",type=Path,required=True)
    args = parser.parse_args()
    export(args.run,args.episode,args.out)
