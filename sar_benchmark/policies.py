"""Two input-matched diagnostic baselines; no imports from private scenarios/GT.

Red-component detection is deliberately a synthetic fixture detector. It is
image-derived, but cannot support human-perception or photorealism claims.
"""
import math
import random
import statistics

from .geometry import localize


def detections(obs):
    if not obs["rgb"]["valid"] or not obs["depth_m"]["valid"]:
        return []
    if abs(obs["rgb"]["capture_ns"] - obs["depth_m"]["capture_ns"]) > 30_000_000 or obs["sim_time_ns"] - obs["rgb"]["capture_ns"] > 500_000_000:
        return []
    width, height = obs["camera"]["width"], obs["camera"]["height"]
    rgb = obs["rgb"]["data"]
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
    def __init__(self, method="sweep", seed=0):
        if method not in ("random", "sweep"):
            raise ValueError("unknown policy")
        self.method = method
        self.rng = random.Random(seed)
        self.tracks, self.reported = [], []
        self.report_count = 0

    def act(self, obs):
        reports = []
        if obs["odom"]["valid"] and obs["sim_time_ns"] - obs["odom"]["capture_ns"] <= 300_000_000:
            for detection in detections(obs):
                position = detection["position"]
                if any(math.dist(position[:2], previous[:2]) < .8 for previous in self.reported):
                    continue
                best = next((track for track in self.tracks if math.dist(track["position"][:2], position[:2]) < .8), None)
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
            if tick % 10 == 0:
                self.motion = [self.rng.uniform(.05, .2), 0.0, self.rng.choice([-.5, .5])]
            twist = list(self.motion)
        else:
            # Full sensor-only sweep, then short translation, repeated. No map,
            # victim count, hidden target pose or success acknowledgement.
            twist = [0.0, 0.0, .5] if tick % 140 < 125 else [.15, 0.0, 0.0]
        return {"epoch": obs["epoch"], "sequence": tick, "tick": tick, "twist": twist,
                "reports": reports, "finish": tick * 100_000_000 >= obs["budget_ns"] - 100_000_000}
