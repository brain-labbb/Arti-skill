#!/usr/bin/env python3

from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
import sys
import unittest
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_stable_v2_aggregate as aggregate
import table5_stable_v2_runtime as runtime


ROW = {
    "bounding_box_diagonal": 1.0,
    "scalar_joints": [
        {
            "name": "hinge",
            "type": "revolute",
            "lower": 0.0,
            "upper": 1.0,
            "parent": "base",
            "child": "door",
            "axis": [0.0, 0.0, 1.0],
            "origin_xyz": [0.0, 0.0, 0.0],
            "origin_rpy": [0.0, 0.0, 0.0],
            "fk_supported": True,
        }
    ],
    "joint_tree": {
        "links": ["base", "door"],
        "root_links": ["base"],
        "joints": [
            {
                "name": "hinge",
                "type": "revolute",
                "lower": 0.0,
                "upper": 1.0,
                "parent": "base",
                "child": "door",
                "axis": [0.0, 0.0, 1.0],
                "origin_xyz": [0.0, 0.0, 0.0],
                "origin_rpy": [0.0, 0.0, 0.0],
                "fk_supported": True,
            }
        ],
    },
}


class FakeAdapter:
    mapped_joint_names = ["hinge"]
    observed_joint_names = ["hinge"]
    observed_link_names = ["base", "door"]

    def __init__(self, mode: str = "bounded_motion") -> None:
        self.mode = mode
        self.position = 0.5
        self.speed = 0.0
        self.step_count = 0

    def reset(self, positions: dict[str, float]) -> None:
        self.position = positions["hinge"]
        self.speed = 0.0
        self.step_count = 0

    def state(self) -> dict[str, dict[str, float]]:
        return {"hinge": {"q": self.position, "qdot": self.speed}}

    def step(self, efforts: dict[str, float]) -> None:
        self.step_count += 1
        if self.mode == "bounded_motion":
            self.speed = 1.0
        elif self.mode == "over_limit":
            self.position = 1.01
            self.speed = 0.0
        elif self.mode == "over_speed":
            self.speed = runtime.MAX_REVOLUTE_SPEED_RAD_PER_S + 0.1
        elif self.mode == "nonfinite" and self.step_count >= 3:
            self.position = math.nan

    def link_poses(self) -> dict[str, dict[str, list[float]]]:
        return {
            "base": {
                "translation": [0.0, 0.0, 0.0],
                "rotation": [1.0, 0.0, 0.0, 0.0],
            },
            "door": {
                "translation": [0.0, 0.0, 0.0],
                "rotation": [
                    math.cos(self.position / 2.0),
                    0.0,
                    0.0,
                    math.sin(self.position / 2.0),
                ],
            },
        }


def mapping_receipt(adapter: FakeAdapter) -> dict[str, object]:
    return runtime._core._mapping_receipt(adapter, ROW)


class StableV2RuntimeTests(unittest.TestCase):
    def test_source_protocol_validates_data_contract_not_legacy_code_hash(self) -> None:
        manifest = runtime._core._read_json(
            SCRIPT_DIR.parent
            / "runtime"
            / "table5_v2_articraft_diagnostic_resample_20260830"
            / "prepared"
            / "manifest.json",
            "test prepared manifest",
        )
        validated = runtime._validate_source_protocol(manifest["protocol"])
        self.assertEqual(validated, manifest["protocol"])

        invalid = deepcopy(manifest["protocol"])
        invalid["runtime"]["timestep_s"] = {"numerator": 1, "denominator": 120}
        invalid["protocol_sha256"] = runtime._core._runtime.canonical_sha256(
            invalid, exclude_fields=("protocol_sha256",)
        )
        with self.assertRaisesRegex(runtime.RuntimeErrorV2, "1/240"):
            runtime._validate_source_protocol(invalid)

    def test_persistent_bounded_motion_passes(self) -> None:
        adapter = FakeAdapter()
        with mock.patch.object(runtime, "MAXIMUM_STEPS", 8), mock.patch.object(
            runtime, "DRIFT_SAMPLE_EVERY_STEPS", 2
        ):
            receipt = runtime.stable_v2_rollout(adapter, ROW, mapping_receipt(adapter))
        self.assertTrue(receipt["passed"])
        self.assertEqual(
            [trial["steps_completed"] for trial in receipt["trials"]], [8] * 3
        )
        self.assertTrue(
            all(
                trial["revolute_speed_rad_per_s_max"] == 1.0
                for trial in receipt["trials"]
            )
        )
        drift = receipt["constraint_drift"]["joints"][0]
        self.assertEqual(drift["expected_samples"], 12)
        self.assertTrue(drift["position_fully_evaluated"])
        self.assertTrue(drift["rotation_fully_evaluated"])
        self.assertAlmostEqual(drift["max_position_error_over_bbox"], 0.0)
        self.assertAlmostEqual(drift["max_rotation_error_rad"], 0.0)

    def test_limit_failure_still_runs_full_horizon(self) -> None:
        adapter = FakeAdapter("over_limit")
        mapping = mapping_receipt(adapter)
        eligible = runtime._eligible_joints(ROW, ["hinge"])
        with mock.patch.object(runtime, "MAXIMUM_STEPS", 8):
            trial = runtime._trial(adapter, ROW, mapping, eligible, 0.5)
        self.assertFalse(trial["passed"])
        self.assertEqual(trial["steps_completed"], 8)
        self.assertFalse(trial["checks"]["limit_compliant"])

    def test_nonfinite_state_fails_closed(self) -> None:
        adapter = FakeAdapter("nonfinite")
        mapping = mapping_receipt(adapter)
        eligible = runtime._eligible_joints(ROW, ["hinge"])
        with mock.patch.object(runtime, "MAXIMUM_STEPS", 8):
            trial = runtime._trial(adapter, ROW, mapping, eligible, 0.5)
        self.assertFalse(trial["passed"])
        self.assertEqual(trial["steps_completed"], 2)
        self.assertFalse(trial["checks"]["full_horizon_finite_rollout"])

    def test_no_bounded_hinge_or_slide_fails(self) -> None:
        adapter = FakeAdapter()
        row = {
            "scalar_joints": [
                {
                    "name": "hinge",
                    "type": "continuous",
                    "lower": None,
                    "upper": None,
                }
            ]
        }
        receipt = runtime.stable_v2_rollout(
            adapter, row, runtime._core._mapping_receipt(adapter, row)
        )
        self.assertFalse(receipt["passed"])
        self.assertEqual(receipt["trials"], [])

    def test_protocol_does_not_require_rest(self) -> None:
        protocol = runtime._stable_protocol()
        self.assertIs(protocol["rest_state_required"], False)


class StableV2AggregateTests(unittest.TestCase):
    def test_passed_receipt_requires_three_mapping_trials(self) -> None:
        adapter = FakeAdapter()
        with mock.patch.object(runtime, "MAXIMUM_STEPS", 8):
            receipt = runtime.stable_v2_rollout(adapter, ROW, mapping_receipt(adapter))
            receipt["trials"][0] = "not-a-trial"
            receipt["receipt_sha256"] = runtime._core._runtime.canonical_sha256(
                receipt, exclude_fields=("receipt_sha256",)
            )
            record = {
                "terminal_status": "completed",
                "identity": {
                    "simulator": "genesis",
                    "dataset_slug": "pva",
                },
                "evaluation": {
                    "stable_v2": {
                        "schema_version": runtime.EVIDENCE_SCHEMA,
                        "protocol": runtime._stable_protocol(),
                        "rollout": deepcopy(receipt),
                    }
                },
            }
            passed, reason = aggregate._receipt_pass(record, "genesis", "pva")
        self.assertFalse(passed)
        self.assertEqual(reason, "passed_receipt_trial_count_mismatch")


if __name__ == "__main__":
    unittest.main()
