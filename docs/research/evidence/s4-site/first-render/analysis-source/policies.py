"""Two input-matched diagnostic baselines; no imports from private scenarios/GT.

Red-component detection is deliberately a synthetic fixture detector. It is
image-derived, but cannot support human-perception or photorealism claims.
"""
import math
import random
import statistics

from .geometry import localize


def detections(obs, detector=None):
    if not obs["rgb"]["valid"] or not obs["depth_m"]["valid"] or not obs["odom"]["valid"]:
        return []
    if obs["sim_time_ns"] - obs["odom"]["capture_ns"] > 300_000_000:
        return []
    if abs(obs["rgb"]["capture_ns"] - obs["depth_m"]["capture_ns"]) > 30_000_000 or obs["sim_time_ns"] - obs["rgb"]["capture_ns"] > 500_000_000:
        return []
    width, height = obs["camera"]["width"], obs["camera"]["height"]
    rgb = obs["rgb"]["data"]
    if detector is not None:
        results = []
        for detection in detector.detect(rgb, width, height):
            box = detection["box"]
            # Torso depth from the central box area, retaining holes as invalid.
            u, v = (box[0]+box[2])/2, box[1]+.40*(box[3]-box[1])
            radius = max(1, round(min(box[2]-box[0],box[3]-box[1])*.12))
            values = [obs["depth_m"]["data"][y*width+x]
                      for y in range(max(0,round(v)-radius), min(height,round(v)+radius+1))
                      for x in range(max(0,round(u)-radius), min(width,round(u)+radius+1))
                      if obs["depth_m"]["data"][y*width+x] is not None]
            if len(values) < 3: continue
            z = statistics.median(values)
            position = localize(u, v, z, obs["camera"], obs["odom"]["pose"])
            spread = statistics.median(abs(value-z) for value in values)
            results.append({**detection, "position": position, "depth_spread_m": spread})
        return results
    foreground = {i for i in range(width * height)
                  if rgb[3 * i] >= 70 and rgb[3 * i] > 1.8 * max(rgb[3 * i + 1], rgb[3 * i + 2])}
    components = []
    while foreground:
        first = min(foreground)
        foreground.remove(first)
        todo, group = [first], []
        while todo:
            p = todo.pop()
            group.append(p)
            x, y = p % width, p // width
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                q = (y + dy) * width + x + dx
                if 0 <= x + dx < width and 0 <= y + dy < height and q in foreground:
                    foreground.remove(q)
                    todo.append(q)
        if len(group) < 25:
            continue
        xs, ys = [p % width for p in group], [p // width for p in group]
        box = [min(xs), min(ys), max(xs) + 1, max(ys) + 1]
        if box[0] == 0 or box[1] == 0 or box[2] == width or box[3] == height:
            continue
        # Use the central torso area, excluding head/leg extremes.
        u, v = (box[0] + box[2] - 1) / 2, box[1] + .52 * (box[3] - box[1])
        values = [obs["depth_m"]["data"][p] for p in group
                  if abs(p % width - u) <= 2 and abs(p // width - v) <= 2 and obs["depth_m"]["data"][p] is not None]
        if len(values) < 3:
            continue
        z = statistics.median(values)
        position = localize(u, v, z, obs["camera"], obs["odom"]["pose"])
        components.append({"box": box, "position": position, "confidence": .9})
    return components


class SearchPolicy:
    def __init__(self, method="sweep", seed=0, detector=None):
        if method not in ("random", "sweep"):
            raise ValueError("unknown policy")
        self.method = method
        self.rng = random.Random(seed)
        self.tracks, self.reported = [], []
        self.report_count = 0
        self.detector = detector
        if detector is not None:
            from .tracking import ReportTracker
            self.reporter = ReportTracker()

    def act(self, obs):
        reports = []
        self.current_detections = detections(obs, self.detector)
        if self.detector is not None:
            reports = self.reporter.update(obs,self.current_detections)
            self.reported = self.reporter.reported
        elif obs["odom"]["valid"] and obs["sim_time_ns"] - obs["odom"]["capture_ns"] <= 300_000_000:
            for detection in self.current_detections:
                position = detection["position"]
                if any(math.dist(position[:2], previous[:2]) < .8 for previous in self.reported):
                    continue
                best = next((track for track in self.tracks if math.dist(track["position"][:2], position[:2]) < .8
                             and 0 < obs["rgb"]["capture_ns"]-track["stamp"] <= 1_500_000_000), None)
                evidence = {"rgb_sequence": obs["rgb"]["sequence"], "bbox_xyxy_px": detection["box"]}
                if best is None:
                    self.tracks.append({"position": position, "evidence": evidence, "stamp": obs["rgb"]["capture_ns"]})
                elif obs["rgb"]["capture_ns"] - best["stamp"] >= 100_000_000 and self.report_count < 5:
                    self.report_count += 1
                    reports.append({"report_id": f"r{self.report_count}", "epoch": obs["epoch"], "tick": obs["tick"],
                                    "label": "victim_person", "position_episode_m": position, "confidence": detection["confidence"],
                                    "evidence": [best["evidence"], evidence]})
                    self.reported.append(position)
        tick = obs["tick"]
        if self.method == "random":
            phase = obs["sim_time_ns"]//1_000_000_000
            if getattr(self,"motion_phase",None) != phase:
                self.motion = [self.rng.uniform(.05, .2), 0.0, self.rng.choice([-.5, .5])]
                self.motion_phase = phase
            twist = list(self.motion)
        else:
            # Full sensor-only sweep, then short translation, repeated. No map,
            # victim count, hidden target pose or success acknowledgement.
            twist = [0.0, 0.0, .5] if obs["sim_time_ns"] % 14_000_000_000 < 12_500_000_000 else [.15, 0.0, 0.0]
        return {"epoch": obs["epoch"], "sequence": tick, "tick": tick, "twist": twist,
                "reports": reports, "finish": obs["sim_time_ns"] >= obs["budget_ns"] - obs.get("action_dt_ns",100_000_000)}
