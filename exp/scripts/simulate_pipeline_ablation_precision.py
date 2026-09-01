#!/usr/bin/env python3
"""Scenario-based precision simulation for the pipeline 2x2 authoring study."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import statistics
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = (
    REPO_ROOT / "exp/reference/pipeline_ablation_precision_scenarios_v1.json"
)
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/pipeline_ablation_v1/analysis/precision_simulation.json"
ARMS = ((0, 0), (1, 0), (0, 1), (1, 1))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def logistic(value: float) -> float:
    if value >= 0:
        inverse = math.exp(-value)
        return 1.0 / (1.0 + inverse)
    exponent = math.exp(value)
    return exponent / (1.0 + exponent)


def logit(probability: float) -> float:
    return math.log(probability / (1.0 - probability))


def latent_intercept_sd(icc: float) -> float:
    return math.sqrt((icc * math.pi * math.pi / 3.0) / (1.0 - icc))


def validate_config(config: dict[str, Any]) -> None:
    if config.get("schema_version") != 1:
        raise ValueError("schema_version must be 1")
    if int(config.get("simulations_per_cell", 0)) < 100:
        raise ValueError("simulations_per_cell must be at least 100")
    if int(config.get("repeats_per_task_arm", 0)) < 1:
        raise ValueError("repeats_per_task_arm must be positive")
    task_counts = config.get("task_counts")
    if not isinstance(task_counts, list) or not task_counts:
        raise ValueError("task_counts must be a nonempty list")
    if any(not isinstance(value, int) or value < 4 for value in task_counts):
        raise ValueError("every task count must be an integer >= 4")
    iccs = config.get("task_icc")
    if not isinstance(iccs, list) or not iccs:
        raise ValueError("task_icc must be a nonempty list")
    if any(not isinstance(value, (int, float)) or not 0 <= value < 1 for value in iccs):
        raise ValueError("every task ICC must be in [0, 1)")
    scenarios = config.get("scenarios")
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError("scenarios must be a nonempty list")
    for scenario in scenarios:
        probability = scenario.get("baseline_probability")
        if not isinstance(probability, (int, float)) or not 0 < probability < 1:
            raise ValueError("baseline_probability must be in (0, 1)")
        for key in (
            "source_odds_ratio",
            "design_odds_ratio",
            "interaction_odds_ratio",
        ):
            value = scenario.get(key)
            if not isinstance(value, (int, float)) or value <= 0:
                raise ValueError(f"{key} must be positive")


def task_contrasts(
    rng: random.Random,
    *,
    scenario: dict[str, Any],
    task_count: int,
    repeats: int,
    icc: float,
) -> dict[str, list[float]]:
    base = logit(float(scenario["baseline_probability"]))
    beta_source = math.log(float(scenario["source_odds_ratio"]))
    beta_design = math.log(float(scenario["design_odds_ratio"]))
    beta_interaction = math.log(float(scenario["interaction_odds_ratio"]))
    intercept_sd = latent_intercept_sd(icc)
    contrasts = {"source": [], "design": [], "interaction": [], "package": []}

    for _ in range(task_count):
        random_intercept = rng.gauss(0.0, intercept_sd)
        arm_means: dict[tuple[int, int], float] = {}
        for source, design in ARMS:
            linear = (
                base
                + random_intercept
                + beta_source * source
                + beta_design * design
                + beta_interaction * source * design
            )
            probability = logistic(linear)
            successes = sum(rng.random() < probability for _ in range(repeats))
            arm_means[(source, design)] = successes / repeats
        p00 = arm_means[(0, 0)]
        p10 = arm_means[(1, 0)]
        p01 = arm_means[(0, 1)]
        p11 = arm_means[(1, 1)]
        contrasts["source"].append(((p10 - p00) + (p11 - p01)) / 2.0)
        contrasts["design"].append(((p01 - p00) + (p11 - p10)) / 2.0)
        contrasts["interaction"].append(p11 - p10 - p01 + p00)
        contrasts["package"].append(p11 - p00)
    return contrasts


def normal_interval(values: list[float], alpha: float) -> tuple[float, float, float]:
    # The frozen protocol uses alpha=0.05. Keeping this explicit avoids silently
    # presenting an unsupported arbitrary-normal quantile implementation.
    if not math.isclose(alpha, 0.05, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError("only alpha_two_sided=0.05 is supported")
    estimate = statistics.fmean(values)
    standard_error = statistics.stdev(values) / math.sqrt(len(values))
    half_width = 1.959963984540054 * standard_error
    return estimate, estimate - half_width, estimate + half_width


def quantile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def simulate(config: dict[str, Any]) -> dict[str, Any]:
    validate_config(config)
    base_seed = int(config["simulation_seed"])
    simulations = int(config["simulations_per_cell"])
    repeats = int(config["repeats_per_task_arm"])
    alpha = float(config["alpha_two_sided"])
    cells: list[dict[str, Any]] = []

    for scenario_index, scenario in enumerate(config["scenarios"]):
        for task_count in config["task_counts"]:
            for icc_index, icc in enumerate(config["task_icc"]):
                seed = base_seed + scenario_index * 1_000_000 + task_count * 1_000 + icc_index
                rng = random.Random(seed)
                accumulator = {
                    name: {"estimates": [], "half_widths": [], "rejects": 0}
                    for name in ("source", "design", "interaction", "package")
                }
                for _ in range(simulations):
                    contrasts = task_contrasts(
                        rng,
                        scenario=scenario,
                        task_count=task_count,
                        repeats=repeats,
                        icc=float(icc),
                    )
                    for name, values in contrasts.items():
                        estimate, lower, upper = normal_interval(values, alpha)
                        row = accumulator[name]
                        row["estimates"].append(estimate)
                        row["half_widths"].append((upper - lower) / 2.0)
                        row["rejects"] += lower > 0.0 or upper < 0.0

                endpoints: dict[str, Any] = {}
                for name, values in accumulator.items():
                    endpoints[name] = {
                        "mean_estimated_risk_difference": statistics.fmean(values["estimates"]),
                        "median_ci95_half_width": quantile(values["half_widths"], 0.5),
                        "p90_ci95_half_width": quantile(values["half_widths"], 0.9),
                        "two_sided_rejection_probability": values["rejects"] / simulations,
                    }
                cells.append(
                    {
                        "scenario": scenario["name"],
                        "task_count": task_count,
                        "repeats_per_task_arm": repeats,
                        "task_icc": icc,
                        "simulation_seed": seed,
                        "endpoints": endpoints,
                    }
                )

    return {
        "schema_version": 1,
        "status": "PASS",
        "estimand": (
            "paired task-level factorial risk-difference contrasts; normal CI used only "
            "for design-stage sensitivity, not final inference"
        ),
        "claim_boundary": config["claim_boundary"],
        "cell_count": len(cells),
        "cells": cells,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    result = simulate(config)
    result["config_path"] = str(args.config.resolve())
    result["config_sha256"] = sha256(args.config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({"status": result["status"], "cell_count": result["cell_count"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
