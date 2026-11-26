"""Secondary fixed-trajectory synthetic-depth sensitivity, never an RQ3 trial.

RGB, detections and estimated poses stay fixed. Native clean Z is the numerical
reference, not a real Astra measurement. Private data is used only for analysis.
"""
import argparse
import gzip
import math
from pathlib import Path

from .artifacts import digest, verify_inventory, write_json
from .capture import decode_human_capture
from .contracts import strict_loads
from .perception import OnnxPersonDetector
from .policies import detections
from .sensor_response import depth_response
from .runner import source_hashes


class FrozenDetections:
    def __init__(self, boxes):
        self.boxes = boxes

    def detect(self, rgb, width, height):
        return self.boxes


def study(run, model, stride=10, frames_per_episode=3):
    if not verify_inventory(run): raise ValueError("replay input inventory failed")
    manifest = strict_loads((run / "manifest.json").read_bytes())
    detector = OnnxPersonDetector(model, manifest.get("confidence", .4))
    if detector.model_sha256 != manifest.get("model_sha256"):
        raise ValueError("replay requires the recorded detector weights")
    selected, counts = {}, {}
    with gzip.open(run / "public/observations.jsonl.gz", "rt", encoding="utf-8") as stream:
        for line in stream:
            packet = strict_loads(line); obs = packet["observation"]
            epoch = obs["epoch"]
            if obs["tick"] % stride == 0 and counts.get(epoch,0) < frames_per_episode:
                selected[epoch,obs["tick"]] = (packet["episode"],obs)
                counts[epoch] = counts.get(epoch,0)+1
    totals = {mode:{"squared_error":0.,"valid_native":0,"retained":0,"localized":0,"detected":0}
              for mode in ("ideal","gaussian","structured")}
    frames = []
    with gzip.open(run / "private/host.jsonl.gz", "rt", encoding="utf-8") as stream:
        for line in stream:
            raw = strict_loads(line)
            item = selected.get((raw["epoch"],raw["tick"]))
            if item is None: continue
            episode,obs = item
            raw = decode_human_capture(raw)
            config = strict_loads((run / f"private/{episode}-scenario.json").read_bytes())
            boxes = detector.detect(obs["rgb"]["data"],obs["camera"]["width"],obs["camera"]["height"])
            per_frame = {"episode":episode,"epoch":raw["epoch"],"tick":raw["tick"],"boxes":boxes,"models":{}}
            for mode,aggregate in totals.items():
                depth = depth_response(raw["depth_m"],obs["camera"]["width"],mode,config["seed"],raw["tick"],1+config["perception"])
                replay = dict(obs); replay["depth_m"] = {**obs["depth_m"],"data":depth}
                found = detections(replay,FrozenDetections(boxes))
                aggregate["detected"] += len(boxes); aggregate["localized"] += len(found)
                for clean,value in zip(raw["depth_m"],depth):
                    if clean is None: continue
                    aggregate["valid_native"] += 1
                    if value is not None:
                        aggregate["retained"] += 1
                        aggregate["squared_error"] += (value-clean)**2
                per_frame["models"][mode] = {"localized_candidates":found}
            frames.append(per_frame)
    for aggregate in totals.values():
        retained,valid = aggregate["retained"],aggregate["valid_native"]
        aggregate["rmse_vs_native_Z_m"] = math.sqrt(aggregate["squared_error"]/retained) if retained else None
        aggregate["invalid_fraction_of_native_valid"] = 1-retained/valid if valid else None
    return {"scope":"OFFLINE fixed captured trajectories, RGB detections and estimated pose; synthetic sensitivity only, not closed-loop method ranking or real Astra validation",
            "run_manifest_sha256":digest(run / "manifest.json"),"detector_sha256":detector.model_sha256,
            "analysis_source_sha256":source_hashes(),"opencv_version":detector.cv2.__version__,"opencv_threads":detector.cv2.getNumThreads(),
            "stride":stride,"frames_per_episode":frames_per_episode,"selected_frames":len(frames),
            "models":totals,"frames":frames,"gaussian_rmse_matched_to_structured":False,
            "limitations":["Gaussian and structured profiles are uncalibrated and not RMSE matched","Native half-float Z is a simulation reference","Sampling is from policy paths; no held-out image corpus"]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    parser.add_argument("--model",type=Path,required=True)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--stride",type=int,default=10)
    parser.add_argument("--frames-per-episode",type=int,default=3)
    args = parser.parse_args()
    if args.stride < 1 or args.frames_per_episode < 1: parser.error("positive sampling limits required")
    result = study(args.run,args.model,args.stride,args.frames_per_episode)
    write_json(args.out,result)
    print("fixed-trajectory frames:",result["selected_frames"])
