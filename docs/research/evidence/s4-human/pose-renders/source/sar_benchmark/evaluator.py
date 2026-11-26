"""Independent offline evaluator. Imports neither policy nor backend nor detector.

Frame truth contains actual visible pixel boxes, not projected full actor boxes.
All report slots are retained, including schema-invalid predictions.
"""
import math
import statistics

from .contracts import DT_NS, MAX_REPORTS, ContractError, finite, integer, vector, validate_report
from .geometry import iou


def match(edges, count):
    """DP: maximum cardinality, then minimum squared XY cost, then stable order."""
    states = {0: (0.0, ())}
    for report_index, candidates in enumerate(edges):
        updated = dict(states)
        for mask, (cost, pairs) in states.items():
            for victim_index, squared_error in candidates:
                if mask & (1 << victim_index):
                    continue
                key = mask | (1 << victim_index)
                item = (cost + squared_error, pairs + ((report_index, victim_index),))
                if key not in updated or item < updated[key]:
                    updated[key] = item
        states = updated
    return min(states.items(), key=lambda item: (-item[0].bit_count(), item[1]))[1][1]


def reference_match(edges, count):
    """Deliberately exhaustive independent implementation for small K<=3, R<=5."""
    best = None
    def visit(index, used, pairs, cost):
        nonlocal best
        if index == len(edges):
            candidate = (-len(pairs), cost, tuple(pairs))
            if best is None or candidate < best:
                best = candidate
            return
        visit(index + 1, used, pairs, cost)
        for victim, error in edges[index]:
            if victim not in used:
                visit(index + 1, used | {victim}, pairs + [(index, victim)], cost + error)
    visit(0, set(), [], 0.0)
    return best[2]


def evidence_candidates(report, episode):
    validate_report(report)
    if report["epoch"] != episode["epoch"] or report["tick"] < 0 or report["tick"] * DT_NS > episode["terminal_ns"]:
        raise ContractError("report outside epoch/deadline")
    frames = episode["frames"]
    selected = []
    seen = set()
    for evidence in report["evidence"]:
        seq = evidence["rgb_sequence"]
        if seq in seen or str(seq) not in frames:
            raise ContractError("repeated/undelivered evidence")
        seen.add(seq)
        frame = frames[str(seq)]
        stamp = frame["capture_ns"]
        if not frame["valid"] or frame["epoch"] != episode["epoch"] or stamp > report["tick"] * DT_NS or frame["available_ns"] > report["tick"] * DT_NS:
            raise ContractError("invalid/future evidence")
        box = evidence["bbox_xyxy_px"]
        if not (0 <= box[0] < box[2] <= frame["width"] and 0 <= box[1] < box[3] <= frame["height"]):
            raise ContractError("out-of-image bbox")
        selected.append((stamp, box, frame))
    stamps = sorted(x[0] for x in selected)
    if len(set(stamps)) != len(stamps) or stamps[-1] - stamps[0] < DT_NS or report["tick"] * DT_NS - stamps[-1] > 2_000_000_000:
        raise ContractError("buffered/too close/stale evidence")
    candidates = []
    for index, victim in enumerate(episode["victims"]):
        if not all((item := frame["visible"].get(victim["gt_id"])) is not None and
                   item["pixels"] >= 25 and iou(box, item["bbox"]) >= 0.25
                   for _, box, frame in selected):
            continue
        position = report["position_episode_m"]
        anchor = victim["anchor_m"]
        dx, dy = position[0] - anchor[0], position[1] - anchor[1]
        error2 = dx * dx + dy * dy
        if error2 <= episode.get("xy_threshold_m", 1.0) ** 2 and abs(position[2] - anchor[2]) <= 0.75:
            candidates.append((index, error2))
    return candidates


def evaluate(episode, threshold=0.5, matcher=match):
    if not finite(threshold) or not 0 <= threshold <= 1:
        raise ContractError("invalid operating threshold")
    victims, slots = episode["victims"], episode["reports"]
    if not (integer(episode["epoch"], 1) and integer(episode["budget_ns"], DT_NS) and
            integer(episode["terminal_ns"]) and episode["terminal_ns"] <= episode["budget_ns"] and
            finite(episode["path_m"]) and episode["path_m"] >= 0):
        raise ContractError("invalid trusted episode clock/path")
    if not 0 <= len(victims) <= 3 or len({x["gt_id"] for x in victims}) != len(victims):
        raise ContractError("invalid private victim set")
    if not all(vector(v["anchor_m"], 3) for v in victims):
        raise ContractError("invalid private anchor")
    if len(slots) > MAX_REPORTS:
        raise ContractError("trusted ledger exceeded report cap")
    edges, active, reasons, seen_ids = [], [], [], set()
    for slot in slots:
        if not integer(slot["accepted_ns"]) or slot["accepted_ns"] > episode["terminal_ns"]:
            raise ContractError("invalid trusted acceptance timestamp")
        report = slot["payload"]
        confidence = report.get("confidence") if type(report) is dict else None
        # Malformed confidence must never create an abstention loophole.
        confidence = confidence if finite(confidence) and 0 <= confidence <= 1 else 1.0
        active.append(confidence >= threshold)
        candidates, reason = [], None
        try:
            validate_report(report)
            if report["report_id"] in seen_ids:
                raise ContractError("duplicate report_id")
            seen_ids.add(report["report_id"])
            if report["tick"] * DT_NS != slot["accepted_ns"]:
                raise ContractError("backdated report")
            candidates = evidence_candidates(report, episode)
        except ContractError as exc:
            reason = str(exc)
        edges.append(candidates if active[-1] else [])
        reasons.append(reason)
    assignment = matcher(edges, len(victims))
    tp, fp = len(assignment), sum(active) - len(assignment)
    unsafe = bool(episode["safety_events"])
    fatal = episode["terminal_reason"] in ("AGENT_ERROR", "AGENT_TIMEOUT", "AGENT_PROTOCOL_ERROR", "INFRA_ERROR")
    safe = not unsafe and not fatal
    k = len(victims)
    budget = episode["budget_ns"]
    times = sorted({slots[i]["accepted_ns"] for i, enabled in enumerate(active) if enabled})
    area, previous, previous_tp = 0, 0, 0
    first = None
    # Recompute prefix matching. A final minimum-cost reassignment must not
    # rewrite when a victim was first correctly reported.
    for stamp in times + [budget]:
        area += (stamp - previous) * previous_tp
        prefix = [edge if slots[i]["accepted_ns"] <= stamp else [] for i, edge in enumerate(edges)]
        previous_tp = len(matcher(prefix, k))
        if previous_tp and first is None:
            first = stamp
        previous = stamp
    errors = [math.sqrt(dict(edges[i])[j]) for i, j in assignment]
    p90 = sorted(errors)[max(0, math.ceil(.9 * len(errors)) - 1)] if errors else None
    covariance_scores = []
    for report_index, victim_index in assignment:
        report = slots[report_index]["payload"]
        covariance = report.get("position_covariance_m2")
        if covariance is None:
            continue
        dx, dy = [report["position_episode_m"][i] - victims[victim_index]["anchor_m"][i] for i in range(2)]
        a, b, d = covariance[0][0], covariance[0][1], covariance[1][1]
        det = a * d - b * b
        if det <= 0 or not math.isfinite(det):
            continue
        mahalanobis2 = (d * dx * dx - 2 * b * dx * dy + a * dy * dy) / det
        covariance_scores.append({"report": report_index, "mahalanobis2": mahalanobis2,
                                  "nll_xy": math.log(2 * math.pi) + .5 * math.log(det) + .5 * mahalanobis2,
                                  "within_50": mahalanobis2 <= 1.38629436112,
                                  "within_90": mahalanobis2 <= 4.60517018599,
                                  "within_95": mahalanobis2 <= 5.99146454711})
    duplicate_count = sum(active[i] and bool(edges[i]) and i not in {p[0] for p in assignment} for i in range(len(slots)))
    confidence_labels = [(slot["payload"]["confidence"], i in {p[0] for p in assignment})
                         for i, slot in enumerate(slots) if type(slot["payload"]) is dict and
                         finite(slot["payload"].get("confidence")) and 0 <= slot["payload"]["confidence"] <= 1]
    brier = statistics.mean((confidence - label) ** 2 for confidence, label in confidence_labels) if confidence_labels else None
    return {"tp": tp, "fp": fp, "victims": k, "matches": [list(pair) for pair in assignment],
            "recall": tp / k if k else None, "safe_localization_recall": tp / k if k and safe else (0.0 if k else None),
            "precision": tp / (tp + fp) if tp + fp else None,
            "duplicate_reports": duplicate_count, "invalid_reports": sum(x is not None for x in reasons),
            "report_reasons": reasons, "safe": safe, "unsafe": unsafe,
            "mission_complete": safe and tp == k and fp == 0 if k else None,
            "absent_false_alarm": bool(sum(active)) if not k else None,
            "absent_specificity": safe and not sum(active) and episode["terminal_reason"] == "FINISH" and episode["terminal_ns"] < budget if not k else None,
            "time_integrated_recall": area / (budget * k) if k else None,
            "safe_time_integrated_recall": area / (budget * k) if k and safe else (0.0 if k else None),
            "first_correct_ns": first, "restricted_first_ns": first if first is not None else budget,
            "localization_xy_m": errors, "localization_median_m": statistics.median(errors) if errors else None,
            "localization_p90_m": p90, "covariance_matched_inliers_only": covariance_scores,
            "uncertainty_unavailable_count": sum("position_covariance_m2" not in slots[i]["payload"] for i, _ in assignment),
            "path_m": episode["path_m"], "brier": brier, "terminal_reason": episode["terminal_reason"]}
