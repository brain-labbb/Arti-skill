#!/usr/bin/env python3

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_stable_v2_1_runtime as runtime
import table5_stable_v2_1_aggregate as aggregate
import table5_stable_v2_runtime as strict_runtime


def joint(name: str, kind: str, child: str) -> dict[str, object]:
    bounded = kind in {"revolute", "prismatic"}
    return {
        "name": name,
        "type": kind,
        "lower": 0.0 if bounded else None,
        "upper": 1.0 if bounded else None,
        "parent": "base",
        "child": child,
        "axis": [0.0, 0.0, 1.0],
        "origin_xyz": [0.0, 0.0, 0.0],
        "origin_rpy": [0.0, 0.0, 0.0],
        "fk_supported": True,
    }


def row(*joints: dict[str, object]) -> dict[str, object]:
    return {
        "bounding_box_diagonal": 1.0,
        "scalar_joints": list(joints),
        "joint_tree": {
            "links": ["base", *(str(item["child"]) for item in joints)],
            "root_links": ["base"],
            "joints": list(joints),
        },
    }


class FakeAdapter:
    def __init__(self, source_row: dict[str, object], mode: str = "finite") -> None:
        names = [str(item["name"]) for item in source_row["scalar_joints"]]
        children = [str(item["child"]) for item in source_row["scalar_joints"]]
        self.mapped_joint_names = names
        self.observed_joint_names = names
        self.observed_link_names = ["base", *children]
        self.mode = mode
        self.positions = {name: 0.0 for name in names}
        self.speeds = {name: 0.0 for name in names}
        self.step_count = 0

    def reset(self, positions: dict[str, float]) -> None:
        if (
            self.mode == "dependent"
            and sum(value > 0.0 for value in positions.values()) > 1
        ):
            raise RuntimeError("cross-joint configuration is unreachable")
        self.positions = dict(positions)
        self.speeds = {name: 0.0 for name in positions}
        self.step_count = 0

    def state(self) -> dict[str, dict[str, float]]:
        return {
            name: {"q": self.positions[name], "qdot": self.speeds[name]}
            for name in self.mapped_joint_names
        }

    def step(self, efforts: dict[str, float]) -> None:
        self.step_count += 1
        if self.mode == "strict_diagnostic_failure":
            first = self.mapped_joint_names[0]
            self.positions[first] = 1.02
            self.speeds[first] = math.radians(400.0)
        elif self.mode == "nonfinite" and self.step_count == 3:
            self.positions[self.mapped_joint_names[0]] = math.nan

    def link_poses(self) -> dict[str, dict[str, list[float]]]:
        poses = {
            "base": {
                "translation": [0.0, 0.0, 0.0],
                "rotation": [1.0, 0.0, 0.0, 0.0],
            }
        }
        for name in self.observed_link_names[1:]:
            poses[name] = {
                "translation": [0.0, 0.0, 0.0],
                "rotation": [1.0, 0.0, 0.0, 0.0],
            }
        return poses


def mapping(adapter: FakeAdapter, source_row: dict[str, object]) -> dict[str, object]:
    return runtime._core._mapping_receipt(adapter, source_row)


class StableV21Tests(unittest.TestCase):
    def test_report_separates_genesis_and_cross_simulator_tables(self) -> None:
        rate = {"percentage": 100.0}
        drift = {
            "position_p95": {
                "p95": 0.1,
                "evaluated_units": 2,
                "candidate_units": 2,
            },
            "rotation_p95": {
                "p95": 0.2,
                "evaluated_units": 2,
                "candidate_units": 2,
            },
        }
        distribution = {
            "median": 0.0,
            "p95": 0.1,
            "evaluated_units": 2,
            "candidate_units": 2,
        }
        summary = {
            "classification": "COMPLETE",
            "datasets": [
                {
                    "dataset_name": "Example",
                    "n": 200,
                    "primary_existing": {
                        "import_success": rate,
                        "dof_mapping": rate,
                        "actuated_trajectory_coverage": rate,
                    },
                    "inertial_assets": {
                        "mathematically_valid": rate,
                        "exact_unit_placeholder": {"percentage": 0.0},
                        "complete_non_placeholder": rate,
                    },
                    "complete_genesis_readiness": rate,
                    "dependency_aware_functional_motion": {
                        "asset_success": rate,
                        "joint_completion": rate,
                        "evaluable_asset_coverage": rate,
                    },
                    "finite_rollout_v2_1": {
                        "genesis": rate,
                        "pybullet": rate,
                        "mujoco": rate,
                    },
                    "all_three_finite_rollout_v2_1": rate,
                    "all_three_readiness": {
                        "import_success": rate,
                        "dof_mapping": rate,
                    },
                    "constraint_drift": {
                        "genesis": drift,
                        "pybullet": drift,
                        "mujoco": drift,
                    },
                    "neutral_long_horizon_diagnostics": {
                        simulator: {"limit_violation_p95": distribution}
                        for simulator in ("genesis", "pybullet", "mujoco")
                    },
                    "supplementary_existing": {
                        "tracking_nrmse_p95": drift["position_p95"],
                        "limit_violation_p95": drift["position_p95"],
                    },
                }
            ],
        }

        report = aggregate._report(summary)

        self.assertIn("Table 5a: Genesis single-simulator evaluation", report)
        self.assertIn("Genesis 10 s Simulation Validity", report)
        self.assertIn("Complete Genesis Readiness", report)
        self.assertNotIn("Dependency-aware Functional Motion Success", report)
        self.assertNotIn("Drift Pos P95", report)
        self.assertIn("Table 5b: Cross-simulator evaluation", report)
        self.assertIn("All-3 Import", report)
        self.assertIn("All-3 DoF Mapping", report)
        self.assertIn("All-3 Stable", report)
        self.assertNotIn("PyBullet Validity", report)

        supplementary = aggregate._supplementary_report(summary)
        self.assertIn("Kinematic drift numerical diagnostics", supplementary)
        self.assertIn("do not participate in Table 5 ranking", supplementary)
        self.assertIn("Dependency-aware motion diagnostic", supplementary)
        self.assertIn("excluded from Table 5 ranking", supplementary)

    def test_all_three_readiness_uses_exact_intersections(self) -> None:
        rows = [
            {"scalar_joints": [{"name": "door"}, {"name": "drawer"}]},
            {"scalar_joints": [{"name": "lid"}]},
        ]
        evidence = {
            "genesis": [(True, {"door", "drawer"}), (True, {"lid"})],
            "pybullet": [(True, {"door", "drawer"}), (True, {"lid"})],
            "mujoco": [(True, {"door"}), (False, set())],
        }

        readiness = aggregate._intersect_readiness(rows, evidence)

        self.assertEqual(readiness["import_success"]["percentage"], 50.0)
        self.assertEqual(readiness["dof_mapping"]["passed"], 1)
        self.assertEqual(readiness["dof_mapping"]["denominator"], 3)
        self.assertAlmostEqual(readiness["dof_mapping"]["percentage"], 100.0 / 3.0)

    def test_drift_cell_preserves_visible_small_values(self) -> None:
        value = {
            "p95": 0.000013071263977148262,
            "evaluated_units": 624,
            "candidate_units": 628,
        }

        self.assertEqual(aggregate._drift_cell(value), "1.31e-05 (624/628)")

    def test_exact_unit_inertial_placeholder_is_excluded(self) -> None:
        placeholder = {
            "physics": {
                "status": "ready",
                "links": [
                    {
                        "required_for_fixed_base_dynamics": True,
                        "details": {
                            "mass_kg": 1.0,
                            "center_of_mass_xyz": [0.0, 0.0, 0.0],
                            "inertia_eigenvalues_kg_m2": [1.0, 1.0, 1.0],
                        },
                    }
                ],
            }
        }
        geometry_related = {
            "physics": {
                "status": "ready",
                "links": [
                    {
                        "required_for_fixed_base_dynamics": True,
                        "details": {
                            "mass_kg": 1.0,
                            "center_of_mass_xyz": [0.0, 0.0, 0.0],
                            "inertia_eigenvalues_kg_m2": [0.1, 0.2, 0.25],
                        },
                    }
                ],
            }
        }
        overlay = {"physics": {"status": "ready", "policy_id": "overlay"}}

        self.assertTrue(aggregate._is_exact_unit_inertial_placeholder(placeholder))
        self.assertFalse(
            aggregate._is_exact_unit_inertial_placeholder(geometry_related)
        )
        self.assertFalse(aggregate._is_exact_unit_inertial_placeholder(overlay))

    def test_overlay_plan_uses_same_placeholder_rule(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            plan_path = Path(directory) / "physics_plan.json"
            plan_path.write_text(
                json.dumps(
                    {
                        "links": [
                            {
                                "link_name": "base",
                                "inertial": {
                                    "mass_kg": 2.0,
                                    "inertia_eigenvalues_kg_m2": [0.1, 0.2, 0.25],
                                },
                            },
                            {
                                "link_name": "door",
                                "inertial": {
                                    "mass_kg": 1.0,
                                    "inertia_eigenvalues_kg_m2": [1.0, 1.0, 1.0],
                                },
                            },
                        ]
                    }
                ),
                encoding="utf-8",
            )
            overlay = {
                "physics": {
                    "status": "ready",
                    "physics_plan_path": str(plan_path),
                    "physics_plan_sha256": runtime._core._runtime.sha256_file(
                        plan_path
                    ),
                },
                "joint_tree": {
                    "links": ["base", "door"],
                    "root_links": ["base"],
                },
            }
            self.assertTrue(aggregate._is_exact_unit_inertial_placeholder(overlay))

    def test_trial_max_requires_complete_passed_receipt(self) -> None:
        receipt = {
            "passed": True,
            "trials": [
                {"limit_violation_over_range_max": 0.001},
                {"limit_violation_over_range_max": 0.007},
                {"limit_violation_over_range_max": 0.002},
            ],
        }
        self.assertEqual(
            aggregate._trial_max(receipt, "limit_violation_over_range_max"), 0.007
        )
        receipt["passed"] = False
        self.assertIsNone(
            aggregate._trial_max(receipt, "limit_violation_over_range_max")
        )

    def test_protocol_separates_stability_from_strict_diagnostics(self) -> None:
        protocol = runtime._stable_protocol()
        self.assertFalse(
            protocol["initial_state"]["cross_joint_fractional_combinations"]
        )
        self.assertFalse(
            protocol["supplementary_strict_diagnostics"]["stable_pass_gate"]
        )
        self.assertEqual(protocol["maximum_steps"] * protocol["timestep_s"], 10.0)

    def test_neutral_reset_avoids_unreachable_cross_joint_pose(self) -> None:
        source_row = row(
            joint("door", "revolute", "door_link"),
            joint("drawer", "prismatic", "drawer_link"),
        )
        adapter = FakeAdapter(source_row, "dependent")
        with mock.patch.object(runtime, "MAXIMUM_STEPS", 4), mock.patch.object(
            runtime, "DRIFT_SAMPLE_EVERY_STEPS", 2
        ):
            receipt = runtime.stable_v2_1_rollout(
                adapter, source_row, mapping(adapter, source_row)
            )
        self.assertTrue(receipt["passed"])
        self.assertEqual(
            [trial["steps_completed"] for trial in receipt["trials"]], [4] * 3
        )

        strict_adapter = FakeAdapter(source_row, "dependent")
        with mock.patch.object(strict_runtime, "MAXIMUM_STEPS", 4):
            strict_receipt = strict_runtime.stable_v2_rollout(
                strict_adapter,
                source_row,
                strict_runtime._core._mapping_receipt(strict_adapter, source_row),
            )
        self.assertFalse(strict_receipt["passed"])

    def test_limit_and_speed_are_diagnostics_not_stability_gates(self) -> None:
        source_row = row(joint("hinge", "revolute", "door"))
        adapter = FakeAdapter(source_row, "strict_diagnostic_failure")
        with mock.patch.object(runtime, "MAXIMUM_STEPS", 4):
            receipt = runtime.stable_v2_1_rollout(
                adapter, source_row, mapping(adapter, source_row)
            )
        self.assertTrue(receipt["passed"])
        for trial in receipt["trials"]:
            diagnostics = trial["supplementary_strict_diagnostics"]
            self.assertFalse(diagnostics["strict_limit_compliant"])
            self.assertFalse(diagnostics["strict_joint_speed_bounded"])

    def test_continuous_only_asset_is_evaluated(self) -> None:
        source_row = row(joint("yaw", "continuous", "head"))
        adapter = FakeAdapter(source_row)
        with mock.patch.object(runtime, "MAXIMUM_STEPS", 4):
            receipt = runtime.stable_v2_1_rollout(
                adapter, source_row, mapping(adapter, source_row)
            )
        self.assertTrue(receipt["passed"])
        self.assertEqual(receipt["mapped_eligible_joint_count"], 0)
        self.assertEqual(len(receipt["trials"]), 3)

    def test_nonfinite_rollout_fails_closed(self) -> None:
        source_row = row(joint("hinge", "revolute", "door"))
        adapter = FakeAdapter(source_row, "nonfinite")
        with mock.patch.object(runtime, "MAXIMUM_STEPS", 4):
            receipt = runtime.stable_v2_1_rollout(
                adapter, source_row, mapping(adapter, source_row)
            )
        self.assertFalse(receipt["passed"])
        self.assertTrue(
            all(trial["steps_completed"] == 2 for trial in receipt["trials"])
        )


if __name__ == "__main__":
    unittest.main()
