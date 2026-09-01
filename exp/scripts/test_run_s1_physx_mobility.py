#!/usr/bin/env python3
"""Behavior tests for the PhysX-Mobility Supplementary Table S1 runner."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


REPO = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO / "exp/scripts/run_s1_physx_mobility.py"
SPEC = importlib.util.spec_from_file_location("run_s1_physx_mobility_test_target", RUNNER_PATH)
assert SPEC is not None and SPEC.loader is not None
runner = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = runner
SPEC.loader.exec_module(runner)


def _record(*, strict: bool, eligible_pairs: int = 0) -> dict:
    return {
        "status": "completed",
        "binding_verified": True,
        "declared_collision_elements": 0,
        "receipt_bound": False,
        "receipt_replay_pass": False,
        "rebuild_eligible": False,
        "rebuild_match": None,
        "allowance_complete": True,
        "registered_allowance_pairs": 0,
        "eligible_nonadjacent_pairs": eligible_pairs,
        "strict_pass_no_method_allowance": strict,
        "registered_allowance_strict_pass": strict,
    }


def test_zero_collision_pair_denominator_is_not_reported_as_zero_percent() -> None:
    summary = runner.aggregate([_record(strict=True), _record(strict=False)], intended_assets=2)
    allowance = summary["metrics"]["allowance_density"]

    assert allowance == {
        "status": "N/E",
        "registered_pairs": 0,
        "eligible_pairs": 0,
        "rate": None,
        "percentage": None,
        "measured_assets": 2,
        "intended_assets": 2,
        "reason": "no eligible non-adjacent collision-bearing source-link pairs",
    }
    assert "0 / 0 (N/E)" in runner.render_summary(summary)


def test_incomplete_table4_state_execution_fails_closed() -> None:
    item = {
        "dataset_id": "7",
        "order": 0,
        "protocol_id": "table4-test",
        "movable_dof_count": 1,
        "rest_state_expected": 1,
        "single_state_expected": 1,
        "sobol_state_expected": 1,
    }
    states = [
        {
            "dataset_id": "7",
            "order": 0,
            "protocol_id": "table4-test",
            "phase": "rest",
            "non_adjacent_illegal_penetration_count": 0,
        },
        {
            "dataset_id": "7",
            "order": 0,
            "protocol_id": "table4-test",
            "phase": "single_joint_sweep",
            "non_adjacent_illegal_penetration_count": 0,
        },
    ]
    asset = {
        **item,
        "range_evaluable_dof_count": 1,
        "state_records_sha256": runner.canonical_sha256(states),
        "rest_state_executed": 1,
        "rest_non_adjacent_free": 1,
        "rest_non_adjacent_cf": True,
        "single_state_executed": 1,
        "single_non_adjacent_free": 1,
        "single_joint_sweep_cf": True,
        "sobol_state_executed": 0,
        "sobol_non_adjacent_free": 0,
        "multi_joint_sobol_cf": False,
        "measurement_complete": False,
        "strict_collision_pass": False,
    }

    result = runner.reaggregate_table4_asset(item, asset, states)

    assert result["measurement_complete"] is False
    assert result["strict_collision_pass"] is False


def test_strict_rate_keeps_full_intended_denominator() -> None:
    summary = runner.aggregate(
        [_record(strict=True), _record(strict=False), {"status": "failed"}],
        intended_assets=3,
    )

    assert summary["metrics"]["strict_pass_no_method_allowance"] == {
        "passed": 1,
        "denominator": 3,
        "rate": 1 / 3,
        "percentage": 100 / 3,
    }
