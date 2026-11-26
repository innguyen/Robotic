"""Export actual native/robot images with hashes and explicit viewpoint labels.

Read-only analysis of an inventoried run. No generated/concept imagery. Initial
policy frames are fixed at tick 0; calibration images are privileged and labelled.
"""
import argparse
import base64
import gzip
import json
from pathlib import Path
import shutil
import statistics

from .artifacts import digest, verify_inventory, write_json
from .capture import decode_human_capture
from .contracts import strict_loads
from .perception import OnnxPersonDetector
from .policies import detections


def export(run, target, model):
    import cv2
    import numpy as np
    run,target=Path(run),Path(target)
    if not verify_inventory(run):raise ValueError("run inventory failed")
    target.mkdir(parents=True,exist_ok=False)
    manifest=strict_loads((run/"manifest.json").read_bytes())
    detector=OnnxPersonDetector(model,manifest.get("confidence",.4))
    if detector.model_sha256!=manifest["model_sha256"]:raise ValueError("QA detector differs from recorded model")
    selected={};episodes={}
    for file in sorted((run/"private").glob("*-ledger.json")):
        episode=file.name.removesuffix("-ledger.json")
        ledger=strict_loads(file.read_bytes())
        config=strict_loads((run/"private"/(episode+"-scenario.json")).read_bytes())
        episodes[ledger["epoch"]]={"episode":episode,"config":config,"metrics":strict_loads((run/(episode+"-metrics.json")).read_bytes())["metrics"]}
    with gzip.open(run/"public/observations.jsonl.gz","rt",encoding="utf-8") as stream:
        for line in stream:
            packet=strict_loads(line);obs=packet["observation"]
            if obs["tick"]==0:selected[obs["epoch"],0]={"observation":obs,"role":"INITIAL_POLICY_FRAME","label":packet["episode"]}
    calibration_file=run/"asset-calibration.json"
    if calibration_file.is_file():
        for index,record in enumerate(strict_loads(calibration_file.read_bytes())["records"]):
            epoch=record["states"][0]["epoch"]
            selected[epoch,0]={"role":"PRIVILEGED_CALIBRATION_VIEW_NEVER_DELIVERED_TO_POLICY",
                "label":f"qa-{index:02d}-{record['environment']}-{record['posture']}-P{record['perception']}","config":record["config"]}
    frames=[];native_count=0;render_first=render_last=None;wall_first=wall_last=None;light_counts=set()
    with gzip.open(run/"private/host.jsonl.gz","rt",encoding="utf-8") as stream:
        for line in stream:
            raw=strict_loads(line);native_count+=1
            if "render_loop" in raw:
                render_first=raw["render_loop"]["frames"] if render_first is None else render_first
                render_last=raw["render_loop"]["frames"]
                wall_first=raw["capture_start_wall_ns"] if wall_first is None else wall_first
                wall_last=raw["available_wall_ns"]
                light_counts.add(raw["lighting"]["directional_count"])
            item=selected.get((raw["epoch"],raw["tick"]))
            if item is None:continue
            label=item["label"];decoded=decode_human_capture(raw)
            native=target/(label+"-native.png");native.write_bytes(base64.b64decode(raw["rgb_png"],validate=True))
            depth=np.array([0 if v is None else round(v*1000) for v in decoded["depth_m"]],dtype=np.uint16).reshape(raw["camera"]["height"],raw["camera"]["width"])
            cv2.imwrite(str(target/(label+"-depth-mm.png")),depth)
            # A depth-tested mask visualization uses the same independently
            # rendered isolated/full Z images as the trusted capture decoder.
            def exr(payload):return cv2.imdecode(np.frombuffer(base64.b64decode(payload),np.uint8),cv2.IMREAD_UNCHANGED)[:,:,2].astype(np.float64)/100
            z=exr(raw["depth_exr"])
            for identity,payload in raw["isolated_depth_exr"].items():
                isolated=exr(payload)
                keep=np.isfinite(z)&(z>=.01)&(z<=10)&np.isfinite(isolated)&(isolated>.01)&(isolated<10)&(np.abs(isolated-z)<=np.maximum(.001,np.abs(isolated)*.0007))
                cv2.imwrite(str(target/(label+"-"+identity+"-visible.png")),keep.astype(np.uint8)*255)
            entry={"label":label,"role":item["role"],"epoch":raw["epoch"],"tick":raw["tick"],"camera_pose_world_m":raw["pose"],
                "native_png_sha256":digest(native),"visible":decoded["visible"],"lighting":raw.get("lighting"),"scan_hits":sum(v is not None for v in raw["scan_m"])}
            if "observation" in item:
                obs=item["observation"]
                delivered=target/(label+"-robot.png")
                cv2.imwrite(str(delivered),np.array(obs["rgb"]["data"],np.uint8).reshape(raw["camera"]["height"],raw["camera"]["width"],3)[:,:,::-1])
                found=detections(obs,detector)
                entry.update(offline_detection_localization=found,delivered_png_sha256=digest(delivered),**episodes[raw["epoch"]])
            else:
                # QA inference uses the original view/ideal depth, with no
                # episode scoring or claim of closed-loop detections.
                entry["config"]=item["config"]
                entry["offline_person_boxes"]=detector.detect(decoded["rgb"],raw["camera"]["width"],raw["camera"]["height"])
            frames.append(entry)
    for file in sorted((run/"private").glob("*overview*")):
        if file.is_file():shutil.copyfile(file,target/file.name)
    duration=(wall_last-wall_first)/1e9 if wall_first is not None else None
    memory=strict_loads((run/"runtime-memory.json").read_bytes()) if (run/"runtime-memory.json").is_file() else {"status":"NOT_RECORDED"}
    report={"source_run":str(run.resolve()),"run_inventory_sha256":digest(run/"inventory.json"),"frames":frames,
        "native_captures":native_count,"engine_render_loop_fps":(render_last-render_first)/duration if duration else None,
        "fps_scope":"Average engine callback frames per wall second across captures/inference/resets; not GPU profiler FPS or sensor Hz",
        "directional_light_counts":sorted(light_counts),"engine_peak_working_set_bytes":memory.get("engine_peak_working_set_bytes"),
        "engine_max_private_commit_bytes":memory.get("engine_max_private_commit_bytes"),"memory_status":memory["status"],
        "scope":"Actual UE PNG bytes; robot images are delivered RGB; depth PNG is measured optical Z in mm (0 invalid); masks are depth-tested visualizations; offline QA inference is not a scored episode"}
    write_json(target/"index.json",report)
    write_json(target/"inventory.json",{str(p.relative_to(target)):digest(p) for p in sorted(target.rglob("*")) if p.is_file()})
    print("QA frames:",len(frames))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path);parser.add_argument("--out",type=Path,required=True);parser.add_argument("--model",type=Path,required=True)
    args=parser.parse_args();export(args.run,args.out,args.model)
