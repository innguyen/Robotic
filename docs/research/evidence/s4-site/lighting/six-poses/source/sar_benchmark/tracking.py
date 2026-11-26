"""Public RGB-D multi-target association and optional translated-view verification."""
import math
import statistics


class ReportTracker:
    def __init__(self, verify=False, operating_threshold=.5):
        self.verify = verify
        self.tracks, self.reported = [], []
        self.count = 0
        self.pending = None
        self.operating_threshold = operating_threshold

    def update(self, obs, detections):
        now = obs["rgb"]["capture_ns"]
        self.pending = None
        reports = []
        used = set()
        for detection in sorted(detections, key=lambda d: -d["confidence"]):
            position = detection["position"]
            if any(math.dist(position[:2], previous[:2]) < .65 for previous in self.reported): continue
            candidates = [(math.dist(t["history"][-1]["position"][:2], position[:2]), i, t)
                          for i, t in enumerate(self.tracks) if i not in used and now-t["history"][-1]["stamp"] <= 2_000_000_000]
            best = min(candidates, default=None, key=lambda item: (item[0],item[1]))
            if best is None or best[0] > .65:
                track = {"history": []}; self.tracks.append(track); index = len(self.tracks)-1
            else:
                _, index, track = best
            used.add(index)
            if track["history"] and now <= track["history"][-1]["stamp"]: continue
            track["history"].append({"stamp": now, "position": position, "pose": obs["odom"]["pose"][:2],
                "confidence": detection["confidence"], "evidence": {"rgb_sequence": obs["rgb"]["sequence"], "bbox_xyxy_px": detection["box"]}})
            track["history"] = [h for h in track["history"] if now-h["stamp"] <= 2_000_000_000]
            history = track["history"]
            translated = max((math.dist(h["pose"], history[-1]["pose"]) for h in history), default=0.)
            stable = max((math.dist(h["position"][:2], position[:2]) for h in history), default=0.) <= .5
            ready = len(history) >= (3 if self.verify else 2) and history[-1]["stamp"]-history[-2]["stamp"] >= 100_000_000
            ready = ready and statistics.mean(h["confidence"] for h in history) >= self.operating_threshold
            if self.verify and not (translated >= .2 and stable):
                self.pending = position
                ready = False
            if ready and self.count < 5:
                self.count += 1
                estimate = [statistics.median(h["position"][axis] for h in history) for axis in range(3)]
                variance = [max(.01, statistics.pvariance(h["position"][axis] for h in history)) for axis in range(3)]
                reports.append({"report_id": f"r{self.count}", "epoch": obs["epoch"], "tick": obs["tick"],
                    "label": "victim_person", "position_episode_m": estimate,
                    "confidence": statistics.mean(h["confidence"] for h in history),
                    "position_covariance_m2": [[variance[i] if i==j else 0. for j in range(3)] for i in range(3)],
                    "evidence": [h["evidence"] for h in history[-2:]]})
                self.reported.append(estimate)
        return reports
