import copy
import math
import random
import unittest

from sar_benchmark.contracts import ContractError, DT_NS
from sar_benchmark.evaluator import evaluate, match, reference_match
from sar_benchmark.reference import reference_score


def ledger(count=1):
    victims = [{"gt_id": f"v{i}", "anchor_m": [i * .8, 0, .65]} for i in range(count)]
    frames = {str(i): {"epoch": 1, "capture_ns": i * DT_NS, "available_ns": i * DT_NS,
                       "valid": True, "width": 80, "height": 60,
                       "visible": {v["gt_id"]: {"bbox": [20, 10, 40, 40], "pixels": 200} for v in victims}}
              for i in range(5)}
    return {"epoch": 1, "budget_ns": 10 * DT_NS, "terminal_ns": 5 * DT_NS,
            "terminal_reason": "FINISH", "victims": victims, "frames": frames,
            "reports": [], "safety_events": [], "path_m": 1.2}


def report(position=None, name="r1", tick=2):
    return {"report_id": name, "epoch": 1, "tick": tick, "label": "victim_person",
            "position_episode_m": position or [0.0, 0.0, .65], "confidence": .9,
            "evidence": [{"rgb_sequence": 0, "bbox_xyxy_px": [20, 10, 40, 40]},
                         {"rgb_sequence": 1, "bbox_xyxy_px": [20, 10, 40, 40]}]}


def add(episode, prediction):
    episode["reports"].append({"payload": prediction, "accepted_ns": 2 * DT_NS})


class EvaluatorTests(unittest.TestCase):
    def test_positive_exact(self):
        data = ledger()
        add(data, report())
        score = evaluate(data)
        self.assertEqual((score["tp"], score["fp"], score["safe_localization_recall"]), (1, 0, 1))
        self.assertAlmostEqual(score["time_integrated_recall"], .8)
        self.assertEqual(score, evaluate(data, matcher=reference_match))

    def test_absent_no_report_is_not_recall_one(self):
        score = evaluate(ledger(0))
        self.assertIsNone(score["recall"])
        self.assertTrue(score["absent_specificity"])
        self.assertIsNone(score["precision"])

    def test_absent_false_alarm(self):
        data = ledger(0)
        add(data, report())
        score = evaluate(data)
        self.assertEqual((score["tp"], score["fp"]), (0, 1))
        self.assertTrue(score["absent_false_alarm"])

    def test_duplicate_report(self):
        data = ledger()
        add(data, report())
        add(data, report(name="r2"))
        score = evaluate(data)
        self.assertEqual((score["tp"], score["fp"], score["duplicate_reports"]), (1, 1, 1))

    def test_duplicate_ids_penalized(self):
        data = ledger()
        add(data, report())
        add(data, report())
        score = evaluate(data)
        self.assertEqual((score["tp"], score["fp"], score["invalid_reports"]), (1, 1, 1))

    def test_three_victims(self):
        data = ledger(3)
        for i in range(3):
            add(data, report([i * .8, 0, .65], f"r{i}"))
        self.assertEqual(evaluate(data)["tp"], 3)

    def test_crossed_assignment_maximizes_cardinality(self):
        edges = [[(0, .01), (1, .5)], [(0, .2)]]
        self.assertEqual(match(edges, 2), ((0, 1), (1, 0)))

    def test_equal_cost_tie_is_stable(self):
        self.assertEqual(match([[(0, .2), (1, .2)], [(0, .2), (1, .2)]], 2), ((0, 0), (1, 1)))

    def test_random_graphs_against_exhaustive(self):
        rng = random.Random(221)
        for _ in range(500):
            count = rng.randrange(4)
            edges = [[(j, rng.choice([0., .1, .2, 1.])) for j in range(count) if rng.random() < .6]
                     for i in range(rng.randrange(6))]
            self.assertEqual(match(edges, count), reference_match(edges, count))

    def test_errors_and_threshold_equality(self):
        for position, expected in (([1., 0, .65], 1), ([1.00001, 0, .65], 0), ([0., 0, 1.4], 1), ([0, 0, 1.400001], 0)):
            data = ledger()
            prediction = report(position)
            prediction["confidence"] = .5
            add(data, prediction)
            self.assertEqual(evaluate(data)["tp"], expected)

    def test_adversarial_schema_and_evidence(self):
        changes = [lambda r: r.update(target_pose=[0, 0, 0]),
                   lambda r: r.update(epoch=2), lambda r: r.update(tick=-1),
                   lambda r: r.update(tick=3), lambda r: r.update(label="poster"),
                   lambda r: r.update(position_episode_m=[math.nan, 0, 0]),
                   lambda r: r.update(position_episode_m=[True, 0, 0]),
                   lambda r: r.update(position_covariance_m2=[[1, 2, 0], [2, 1, 0], [0, 0, 1]]),
                   lambda r: r["evidence"][0].update(rgb_sequence=99),
                   lambda r: r["evidence"][1].update(rgb_sequence=0),
                   lambda r: r["evidence"][0].update(bbox_xyxy_px=[-1, 0, 1, 1]),
                   lambda r: r["evidence"][0].update(bbox_xyxy_px=[1, 1, 1, 1]),
                   lambda r: r["evidence"][0].update(bbox_xyxy_px=[0, 0, 3, 3]),
                   lambda r: r.update(evidence=r["evidence"][:1])]
        for change in changes:
            with self.subTest(change=change):
                data, prediction = ledger(), report()
                change(prediction)
                add(data, prediction)
                result = evaluate(data)
                self.assertEqual((result["tp"], result["fp"]), (0, 1))

    def test_invalid_confidence_cannot_hide_fp(self):
        for value in (None, math.nan, math.inf, True, -1, 2, 1e308, 10 ** 1000, "0"):
            data, prediction = ledger(), report()
            prediction["confidence"] = value
            add(data, prediction)
            self.assertEqual(evaluate(data, threshold=1)["fp"], 1)

    def test_extreme_finite_position_is_false_report_not_evaluator_crash(self):
        data = ledger()
        add(data, report([1e308, -1e308, .65]))
        self.assertEqual(evaluate(data)["fp"], 1)
        self.assertEqual(reference_score(data)["fp"], 1)

    def test_low_confidence_abstention(self):
        data, prediction = ledger(), report()
        prediction["confidence"] = .49
        add(data, prediction)
        self.assertEqual((evaluate(data)["tp"], evaluate(data)["fp"]), (0, 0))

    def test_held_and_frozen_buffers(self):
        for field, value in (("capture_ns", 0), ("available_ns", 4 * DT_NS), ("valid", False), ("epoch", 2)):
            data = ledger()
            data["frames"]["1"][field] = value
            add(data, report())
            self.assertEqual(evaluate(data)["tp"], 0)

    def test_visible_pixels_and_association_same_victim(self):
        data = ledger(2)
        data["frames"]["0"]["visible"].pop("v1")
        data["frames"]["1"]["visible"].pop("v0")
        add(data, report())
        self.assertEqual(evaluate(data)["tp"], 0)
        data = ledger()
        data["frames"]["1"]["visible"]["v0"]["pixels"] = 24
        add(data, report())
        self.assertEqual(evaluate(data)["tp"], 0)

    def test_safety_abort_keeps_denominator(self):
        data = ledger()
        add(data, report())
        data["safety_events"] = ["CONTACT"]
        result = evaluate(data)
        self.assertEqual(result["recall"], 1)
        self.assertEqual(result["safe_localization_recall"], 0)
        self.assertEqual(result["safe_time_integrated_recall"], 0)

    def test_agent_error_does_not_get_primary_credit(self):
        data = ledger()
        add(data, report())
        data["terminal_reason"] = "AGENT_PROTOCOL_ERROR"
        self.assertEqual(evaluate(data)["safe_localization_recall"], 0)

    def test_prefix_matching_time_credit(self):
        data = ledger(2)
        add(data, report([.1, 0, .65]))
        second = report([0, 0, .65], "r2", tick=4)
        data["reports"].append({"payload": second, "accepted_ns": 4 * DT_NS})
        self.assertAlmostEqual(evaluate(data)["time_integrated_recall"], .7)

    def test_deadline_evidence_and_backdated_reports(self):
        data = ledger()
        data["terminal_ns"] = data["budget_ns"] = 2 * DT_NS
        add(data, report())
        self.assertEqual(evaluate(data)["tp"], 1)
        data["reports"][0]["payload"]["tick"] = 1
        self.assertEqual(evaluate(data)["tp"], 0)

    def test_invalid_trusted_ledger_rejected(self):
        for key, value in (("terminal_ns", 2_000_000_000), ("path_m", math.nan), ("epoch", True)):
            data = ledger()
            data[key] = value
            with self.assertRaises(ContractError):
                evaluate(data)

    def test_independent_reference_eligibility_and_metrics(self):
        rng = random.Random(8821)
        for _ in range(150):
            data = ledger(rng.randrange(4))
            for index in range(rng.randrange(6)):
                prediction = report([rng.uniform(-1, 3), rng.uniform(-.5, .5), .65], f"r{index}")
                prediction["confidence"] = rng.choice([.2, .5, .9])
                if rng.random() < .15:
                    prediction["evidence"][0]["rgb_sequence"] = 999
                if rng.random() < .15:
                    prediction["target_pose"] = [4242, 4242, 4242]
                add(data, prediction)
            actual, expected = evaluate(data), reference_score(data)
            for key, value in expected.items():
                self.assertEqual(actual[key], value, key)

    def test_covariance_nll_and_coverage(self):
        data, prediction = ledger(), report([.5, 0, .65])
        prediction["position_covariance_m2"] = [[.25, 0, 0], [0, .25, 0], [0, 0, .1]]
        add(data, prediction)
        item = evaluate(data)["covariance_matched_inliers_only"][0]
        self.assertAlmostEqual(item["mahalanobis2"], 1.)
        self.assertTrue(item["within_50"])
        self.assertAlmostEqual(item["nll_xy"], math.log(2 * math.pi) + .5 * math.log(.0625) + .5)


if __name__ == "__main__":
    unittest.main()
