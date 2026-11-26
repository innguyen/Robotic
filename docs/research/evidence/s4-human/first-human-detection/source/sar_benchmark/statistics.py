"""Family-weighted descriptive factorial analysis; missing cells fail closed."""
import random
import statistics


def rq1(rows, draws=10000, seed=17):
    grouped = {}
    for row in rows:
        value = row["metrics"]["safe_localization_recall"]
        if value is None:
            continue
        key = (row["method"], row["family"], row["pair_id"])
        cell = (row["perception"], row["odometry"])
        if cell in grouped.setdefault(key, {}):
            raise ValueError("duplicate paired cell")
        grouped[key][cell] = value
    methods = {}
    for (method, family, pair), cells in grouped.items():
        if set(cells) != {(0, 0), (1, 0), (0, 1), (1, 1)}:
            raise ValueError(f"incomplete factorial block: {method}/{family}/{pair}")
        methods.setdefault(method, {}).setdefault(family, []).append(cells)
    results = {}
    for method, families in methods.items():
        family_cells = [{str(cell): statistics.mean(block[cell] for block in blocks)
                         for cell in ((0, 0), (1, 0), (0, 1), (1, 1))} for blocks in families.values()]
        interactions = [c["(1, 1)"] - c["(1, 0)"] - c["(0, 1)"] + c["(0, 0)"] for c in family_cells]
        estimate = statistics.mean(interactions)
        ci = None
        if len(interactions) >= 2:
            rng = random.Random(seed)
            samples = sorted(statistics.mean(rng.choices(interactions, k=len(interactions))) for _ in range(draws))
            ci = [samples[int(0.025 * draws)], samples[min(draws - 1, int(0.975 * draws))]]
        results[method] = {"family_count": len(interactions), "interaction": estimate, "ci95": ci,
                           "cells": {cell: statistics.mean(c[cell] for c in family_cells) for cell in family_cells[0]},
                           "interpretation": "diagnostic pilot; no confirmatory claim", "bootstrap_draws": draws if ci else 0}
    return results
