#!/usr/bin/env python3
"""Behavior tests for the PhysX-Mobility Table 4b runner."""

from __future__ import annotations

from copy import deepcopy
import importlib.util
from pathlib import Path
import sys


REPO = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO / "exp/scripts/run_urdf_table4b_physx_mobility.py"
SPEC = importlib.util.spec_from_file_location(
    "run_urdf_table4b_physx_mobility_test_target", RUNNER_PATH
)
assert SPEC is not None and SPEC.loader is not None
runner = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = runner
SPEC.loader.exec_module(runner)


def _record(index: int) -> dict:
    unavailable = {"status": "N/E", "value": None}
    return {
        "selection_index": index,
        "dataset_id": str(index),
        "status": "completed",
        "declared_collision_element_count": 0,
        "loadable_collision_element_count": 0,
        "analytic_collision_element_count": 0,
        "d_visual_status": "COMPLETE",
        "d_visual": 1.0,
        "visual_to_collision_p95_normalized": deepcopy(unavailable),
        "collision_to_visual_p95_normalized": deepcopy(unavailable),
        "collision_load_time_seconds": deepcopy(unavailable),
        "shapes_per_visual_bearing_link": deepcopy(unavailable),
        "collision_mesh_triangles_per_asset": deepcopy(unavailable),
        "intra_link_redundancy": {
            "status": "N/E",
            "measured_link_count": 0,
            "intended_link_count": 0,
            "redundant_volume_m3": None,
            "shape_volume_m3": None,
        },
    }


def test_verifier_accepts_fail_closed_partial_d_visual_coverage() -> None:
    rows = [{"dataset_id": index} for index in range(runner.N_EVAL)]
    records = [_record(index) for index in range(runner.N_EVAL)]
    records[-3]["d_visual_status"] = "PARTIAL"
    records[-2]["d_visual_status"] = "PARTIAL"
    records[-1]["d_visual_status"] = "N/E"
    records[-1]["d_visual"] = None
    aggregates = runner.aggregate(records)

    verification = runner.verify_run(rows, records, aggregates)

    assert aggregates["d_visual_coverage"] == {"complete": 797, "intended": 800}
    assert verification["all_pass"] is True


def test_verifier_rejects_tampered_d_visual_coverage() -> None:
    rows = [{"dataset_id": index} for index in range(runner.N_EVAL)]
    records = [_record(index) for index in range(runner.N_EVAL)]
    records[-3]["d_visual_status"] = "PARTIAL"
    records[-2]["d_visual_status"] = "PARTIAL"
    records[-1]["d_visual_status"] = "N/E"
    records[-1]["d_visual"] = None
    aggregates = runner.aggregate(records)
    aggregates["d_visual_coverage"] = {"complete": 800, "intended": 800}

    verification = runner.verify_run(rows, records, aggregates)
    checks = {item["check"]: item["pass"] for item in verification["checks"]}

    assert verification["all_pass"] is False
    assert checks["aggregate_recomputation_matches"] is False
