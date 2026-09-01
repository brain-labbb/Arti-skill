#!/usr/bin/env python3

from __future__ import annotations

import math
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_passive_stability_aggregate as aggregate
import table5_passive_stability_runtime as runtime


class PassiveStabilityTest(unittest.TestCase):
    def test_trial_specs_release_each_joint_separately(self) -> None:
        row = {
            "scalar_joints": [
                {"name": "door", "type": "revolute", "lower": 0.0, "upper": 2.0},
                {"name": "knob", "type": "continuous", "lower": None, "upper": None},
                {"name": "bad_slide", "type": "prismatic", "lower": None, "upper": None},
            ]
        }
        self.assertEqual(
            [item["trial_id"] for item in runtime.trial_specs(row)],
            ["joint:door", "joint:knob"],
        )

    def test_wrapped_endpoint_distance(self) -> None:
        spec = {"scale": 2.0 * math.pi, "distance_mode": "wrapped_angle"}
        distance = runtime.normalized_distance(math.pi - 0.01, -math.pi + 0.01, spec)
        self.assertAlmostEqual(distance, 0.02 / (2.0 * math.pi))

    def test_authored_damping_parser(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.urdf"
            path.write_text(
                '<robot name="x"><joint name="door" type="revolute">'
                '<dynamics damping="0.4"/></joint></robot>',
                encoding="ascii",
            )
            self.assertEqual(runtime._authored_damping(path), {"door": 0.4})

    def test_settle_window_uses_speed_span_and_limits(self) -> None:
        spec = {
            "joint_name": "door",
            "joint_type": "revolute",
            "bounded": True,
            "lower": 0.0,
            "upper": 1.0,
            "scale": 1.0,
            "distance_mode": "linear",
        }
        quiet = [
            {"door": {"q": 0.5 + index * 1.0e-6, "qdot": 1.0e-4}}
            for index in range(runtime.SETTLE_WINDOW_STEPS)
        ]
        moving = [
            {"door": {"q": 0.5 + index * 1.0e-4, "qdot": 0.02}}
            for index in range(runtime.SETTLE_WINDOW_STEPS)
        ]
        self.assertTrue(runtime._window_joint_assessment(quiet, spec)["passed"])
        self.assertFalse(runtime._window_joint_assessment(moving, spec)["passed"])

    def test_endpoint_reduction_uses_pairwise_median(self) -> None:
        endpoints = [
            {"joint_name": "door", "distance_mode": "linear", "scale": 2.0, "q": value}
            for value in (0.0, 0.2, 0.6)
        ]
        self.assertAlmostEqual(aggregate._endpoint_distance(endpoints), 0.2)

    def test_percentile_interpolates(self) -> None:
        self.assertEqual(aggregate._percentile([], 95.0), None)
        self.assertAlmostEqual(aggregate._percentile([0.0, 1.0], 95.0), 0.95)


if __name__ == "__main__":
    unittest.main()
