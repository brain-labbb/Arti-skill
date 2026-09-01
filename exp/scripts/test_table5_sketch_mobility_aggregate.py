#!/usr/bin/env python3
"""Focused regression tests for SketchMobility Table 5 aggregation."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


REPO = Path(__file__).resolve().parents[2]
AGGREGATE = REPO / "exp/scripts/aggregate_table5_sketch_mobility.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"cannot load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class SketchMobilityTable5AggregateTests(unittest.TestCase):
    def test_pose_diagnostics_excludes_units_when_bbox_is_missing(self) -> None:
        aggregate = load_module(AGGREGATE, "table5_sketch_aggregate_bbox_target")
        dataset_id = "sketch_0001"
        row = {
            "dataset_id": dataset_id,
            "asset_id": "data/Shape2Motion/example",
            "manifest_root": "data/Shape2Motion/example",
            "bounding_box_diagonal": None,
            "preflight": {
                "status": "failed",
                "issues": ["missing_bounding_box"],
                "simulator_eligible": False,
            },
            "joint_tree": {
                "links": ["base", "door"],
                "root_links": ["base"],
                "joints": [
                    {
                        "name": "hinge",
                        "type": "revolute",
                        "parent": "base",
                        "child": "door",
                    }
                ],
            },
        }
        protocol = {
            "cross_simulator": {
                "thresholds": {
                    "translation_over_bbox_diagonal": 0.01,
                    "rotation_rad": 0.01,
                }
            }
        }
        indexed = {simulator: {} for simulator in aggregate.SIMULATORS}

        diagnostics, inventory = aggregate._pose_diagnostics(
            protocol, {dataset_id: row}, indexed, [dataset_id]
        )

        self.assertEqual(1, diagnostics["candidate_units"])
        self.assertEqual(0, diagnostics["evaluable_units"])
        self.assertEqual(1, diagnostics["excluded_units"])
        self.assertEqual(
            [
                {
                    "dataset_id": dataset_id,
                    "asset_id": row["manifest_root"],
                    "manifest_root": row["manifest_root"],
                    "joint_name": "hinge",
                    "descendant_link": "door",
                    "reason": "missing_bounding_box",
                }
            ],
            inventory,
        )


if __name__ == "__main__":
    unittest.main()
