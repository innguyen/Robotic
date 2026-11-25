"""Small independently written reference scorer for counts and time credit.

It deliberately repeats the eligibility calculation, shares only wire-schema
validation, and exhaustively enumerates assignments (<=3 victims, <=5 slots).
It does not call evaluator.evidence_candidates or the DP matcher.
"""
import math

from .contracts import ContractError, DT_NS, finite, validate_report


def reference_score(data, threshold=.5):
    graph, included, used_names = [], [], set()
    for slot in data["reports"]:
        p = slot["payload"]
        confidence = p.get("confidence") if isinstance(p, dict) else None
        included.append(confidence >= threshold if finite(confidence) and 0 <= confidence <= 1 else True)
        allowed = []
        try:
            validate_report(p)
            if p["report_id"] in used_names:
                raise ContractError("id repeated")
            used_names.add(p["report_id"])
            if (p["epoch"] != data["epoch"] or p["tick"] * DT_NS != slot["accepted_ns"] or
                    slot["accepted_ns"] > data["terminal_ns"]):
                raise ContractError("header mismatch")
            evidence_frames = []
            sequences = set()
            for e in p["evidence"]:
                number = e["rgb_sequence"]
                if number in sequences or str(number) not in data["frames"]:
                    raise ContractError("frame mismatch")
                sequences.add(number)
                frame = data["frames"][str(number)]
                x1, y1, x2, y2 = e["bbox_xyxy_px"]
                if not (0 <= x1 < x2 <= frame["width"] and 0 <= y1 < y2 <= frame["height"] and
                        frame["valid"] and frame["epoch"] == data["epoch"] and
                        0 <= frame["capture_ns"] <= frame["available_ns"] <= slot["accepted_ns"]):
                    raise ContractError("invalid frame")
                evidence_frames.append((e["bbox_xyxy_px"], frame))
            stamps = [f["capture_ns"] for _, f in evidence_frames]
            if len(set(stamps)) != len(stamps) or max(stamps) - min(stamps) < DT_NS or slot["accepted_ns"] - max(stamps) > 2_000_000_000:
                raise ContractError("frame timing mismatch")
            for j, victim in enumerate(data["victims"]):
                valid = True
                for box, frame in evidence_frames:
                    annotation = frame["visible"].get(victim["gt_id"])
                    if annotation is None or annotation["pixels"] < 25:
                        valid = False
                        break
                    other = annotation["bbox"]
                    overlap = max(0, min(box[2], other[2]) - max(box[0], other[0])) * max(0, min(box[3], other[3]) - max(box[1], other[1]))
                    total = (box[2] - box[0]) * (box[3] - box[1]) + (other[2] - other[0]) * (other[3] - other[1]) - overlap
                    if total <= 0 or overlap / total < .25:
                        valid = False
                        break
                error = sum((p["position_episode_m"][i] - victim["anchor_m"][i]) ** 2 for i in (0, 1))
                if valid and error <= data.get("xy_threshold_m", 1.) ** 2 and abs(p["position_episode_m"][2] - victim["anchor_m"][2]) <= .75:
                    allowed.append((j, error))
        except ContractError:
            allowed = []
        graph.append(allowed if included[-1] else [])
    def solve(deadline):
        options = []
        def recurse(index, taken, choices, cost):
            if index == len(graph):
                options.append((-len(choices), cost, tuple(choices)))
                return
            recurse(index + 1, taken, choices, cost)
            if data["reports"][index]["accepted_ns"] <= deadline:
                for target, error in graph[index]:
                    if target not in taken:
                        recurse(index + 1, taken | {target}, choices + [(index, target)], cost + error)
        recurse(0, set(), [], 0.)
        return min(options)[2]
    pairs = solve(data["budget_ns"])
    stamps = sorted({0, data["budget_ns"]} | {s["accepted_ns"] for s in data["reports"]})
    integral = sum((right - left) * len(solve(left)) for left, right in zip(stamps, stamps[1:]))
    k = len(data["victims"])
    safe = not data["safety_events"] and data["terminal_reason"] not in ("AGENT_ERROR", "AGENT_TIMEOUT", "AGENT_PROTOCOL_ERROR", "INFRA_ERROR")
    return {"tp": len(pairs), "fp": sum(included) - len(pairs), "matches": [list(p) for p in pairs],
            "time_integrated_recall": integral / (data["budget_ns"] * k) if k else None,
            "safe_localization_recall": len(pairs) / k if k and safe else (0. if k else None)}
