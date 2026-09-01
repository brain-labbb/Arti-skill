from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4_partnet_mobility.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("urdf_table4_partnet", RUNNER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {RUNNER}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Table4ProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.runner = load_runner()

    def test_selection_is_salted_identity_rank_without_outcome_filtering(self) -> None:
        candidates = [
            {"dataset_id": "148", "category": "Faucet"},
            {"dataset_id": "100599", "category": "Table"},
            {"dataset_id": "103234", "category": "Chair"},
            {"dataset_id": "4628", "category": "Door"},
            {"dataset_id": "102985", "category": "StorageFurniture"},
        ]

        selected = self.runner.select_candidates(
            candidates,
            sample_size=3,
            salt="urdf-sim-ready-table4-partnet-mobility-n800-v1:20260813",
        )

        self.assertEqual(
            [row["dataset_id"] for row in selected],
            ["100599", "103234", "4628"],
        )

    def test_child_python_keeps_virtualenv_symlink_path(self) -> None:
        relative = Path("exp/.venv_low_medium/bin/python")

        normalized = self.runner.normalize_executable_path(relative, REPO)

        self.assertEqual(normalized, REPO / relative)
        self.assertNotEqual(normalized, Path("/usr/bin/python3.12"))

    def test_formal_selection_contract_locks_release_and_ordered_n800(self) -> None:
        release_ids = self.runner.discover_release_ids(
            REPO / "exp/PartNet-Mobility/data/dataset"
        )

        contract = self.runner.selection_contract(
            release_ids, sample_size=800, qualification_smoke=False
        )

        self.assertEqual(
            contract["candidate_pool_identity_sha256"],
            "0203a510202510cea7e469048e84b133bd65ccbc6e1e3aa90c9bfeea7807959d",
        )
        self.assertEqual(
            contract["ordered_selected_ids_sha256"],
            "ef6cb964e50dc712280256c5b2f675cc2c957095c3553b21845d3562a5011883",
        )
        self.assertEqual(contract["protocol_id"], self.runner.PROTOCOL_ID)
        self.assertEqual(contract["cohort_label"], "PartNet-Mobility N=800 sampled release cohort")

        with self.assertRaisesRegex(ValueError, "formal protocol requires sample_size=800"):
            self.runner.selection_contract(
                release_ids, sample_size=3, qualification_smoke=False
            )

        smoke = self.runner.selection_contract(
            release_ids, sample_size=3, qualification_smoke=True
        )
        self.assertEqual(smoke["protocol_id"], self.runner.QUALIFICATION_PROTOCOL_ID)
        self.assertEqual(smoke["cohort_label"], "PartNet-Mobility qualification smoke N=3")

    def test_frozen_archive_identity_rejects_in_place_replacement(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            archive = Path(temporary) / "release.zip"
            archive.write_bytes(b"frozen archive")
            digest = self.runner.sha256_file(archive)
            frozen = {
                "path": str(archive),
                "size_bytes": archive.stat().st_size,
                "sha256": digest,
                "matches_expected_sha256": True,
            }

            self.runner.validate_frozen_archive(
                frozen, archive, expected_sha256=digest
            )
            archive.write_bytes(b"changed archive")

            with self.assertRaisesRegex(RuntimeError, "archive (size|SHA256) drift"):
                self.runner.validate_frozen_archive(
                    frozen, archive, expected_sha256=digest
                )

    def test_nonzero_child_exit_overwrites_stale_success_receipt(self) -> None:
        item = {
            "protocol_id": self.runner.PROTOCOL_ID,
            "order": 0,
            "dataset_id": "1",
            "category": "Fixture",
            "movable_dof_count": 1,
            "range_evaluable_dof_count": 1,
            "rest_state_expected": 1,
            "single_state_expected": 21,
            "sobol_state_expected": 64,
            "input_identity_sha256": "input-hash",
        }
        stale = {
            **self.runner.failure_record(item, "placeholder"),
            "load_success": True,
            "measurement_complete": True,
            "strict_collision_pass": True,
            "runner_sha256": "runner-hash",
            "input_identity_sha256": "input-hash",
        }
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            root = Path(temporary)
            item_path = root / "item.json"
            result_path = root / "result.json"
            log_path = root / "child.log"
            item_path.write_text(json.dumps(item), encoding="utf-8")
            result_path.write_text(json.dumps(stale), encoding="utf-8")
            with mock.patch.object(
                self.runner.subprocess,
                "run",
                return_value=mock.Mock(returncode=7),
            ):
                result = self.runner.run_one_subprocess(
                    item_path,
                    item,
                    root,
                    result_path,
                    log_path,
                    30,
                    Path("/usr/bin/python3"),
                    "different-runner-hash",
                )

        self.assertFalse(result["load_success"])
        self.assertFalse(result["measurement_complete"])
        self.assertFalse(result["strict_collision_pass"])
        self.assertEqual(result["issues"], ["child_exit_7"])
        self.assertEqual(result["child_returncode"], 7)

    def test_frozen_collision_mesh_inventory_rejects_content_drift(self) -> None:
        urdf = """<?xml version="1.0"?>
<robot name="mesh_binding_fixture">
  <link name="base">
    <collision><geometry><mesh filename="mesh.obj"/></geometry></collision>
  </link>
</robot>
"""
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            asset_dir = Path(temporary)
            urdf_path = asset_dir / "mobility.urdf"
            mesh_path = asset_dir / "mesh.obj"
            urdf_path.write_text(urdf, encoding="utf-8")
            mesh_path.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")
            inventory = self.runner.collision_mesh_inventory(asset_dir, urdf_path)
            item = {
                "dataset_id": "fixture",
                "urdf_sha256": self.runner.sha256_file(urdf_path),
                "bounding_box_sha256": None,
                "collision_mesh_files": inventory,
                "collision_mesh_inventory_sha256": self.runner.canonical_sha256(inventory),
            }

            self.runner.validate_frozen_asset_files(item, asset_dir)
            mesh_path.write_text("v 0 0 0\nv 2 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")

            with self.assertRaisesRegex(RuntimeError, "collision mesh inventory drift"):
                self.runner.validate_frozen_asset_files(item, asset_dir)

    def test_cache_reuse_rehashes_frozen_asset_files_first(self) -> None:
        urdf = """<?xml version="1.0"?>
<robot name="cache_fixture">
  <link name="base"><collision><geometry><mesh filename="mesh.obj"/></geometry></collision></link>
</robot>
"""
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            root = Path(temporary)
            asset_dir = root / "1"
            asset_dir.mkdir()
            urdf_path = asset_dir / "mobility.urdf"
            mesh_path = asset_dir / "mesh.obj"
            urdf_path.write_text(urdf, encoding="utf-8")
            mesh_path.write_text("v 0 0 0\n", encoding="utf-8")
            inventory = self.runner.collision_mesh_inventory(asset_dir, urdf_path)
            item = {
                "protocol_id": self.runner.PROTOCOL_ID,
                "order": 0,
                "dataset_id": "1",
                "category": "Fixture",
                "package_audit_success": True,
                "movable_dof_count": 0,
                "range_evaluable_dof_count": 0,
                "joint_specs": [],
                "rest_state_expected": 1,
                "single_state_expected": 0,
                "sobol_state_expected": 0,
                "object_bbox_diagonal_m": 1.0,
                "urdf_sha256": self.runner.sha256_file(urdf_path),
                "bounding_box_sha256": None,
                "collision_mesh_files": inventory,
                "collision_mesh_inventory_sha256": self.runner.canonical_sha256(
                    inventory
                ),
                "input_identity_sha256": "frozen-input",
            }
            cached = self.runner.failure_record(item, "previous_failure")
            cached.update(
                {
                    "runner_sha256": "runner-hash",
                    "input_identity_sha256": "frozen-input",
                }
            )
            item_path = root / "item.json"
            result_path = root / "result.json"
            log_path = root / "child.log"
            item_path.write_text(json.dumps(item), encoding="utf-8")
            result_path.write_text(json.dumps(cached), encoding="utf-8")
            mesh_path.write_text("v 1 0 0\n", encoding="utf-8")

            with mock.patch.object(self.runner.subprocess, "run") as run:
                result = self.runner.run_one_subprocess(
                    item_path,
                    item,
                    root,
                    result_path,
                    log_path,
                    30,
                    Path("/usr/bin/python3"),
                    "runner-hash",
                )

        run.assert_not_called()
        self.assertFalse(result["cache_reused"])
        self.assertIn("frozen_asset_files_drift", result["issues"][0])

    def test_state_closure_rejects_missing_or_duplicated_state_rows(self) -> None:
        item = {
            "protocol_id": self.runner.PROTOCOL_ID,
            "dataset_id": "1",
            "category": "Fixture",
            "rest_state_expected": 1,
            "single_state_expected": 0,
            "sobol_state_expected": 0,
        }
        state = {
            "dataset_id": "1",
            "category": "Fixture",
            "phase": "rest",
            "sample_index": 0,
            "joint_name": None,
            "joint_values_sha256": self.runner.canonical_sha256([]),
            "reset_readback_max_abs_error": 0.0,
            "all_pair_contact_count": 1,
            "all_pair_illegal_penetration_count": 1,
            "all_pair_max_penetration_m": 0.2,
            "non_adjacent_contact_count": 0,
            "non_adjacent_illegal_penetration_count": 0,
            "non_adjacent_max_penetration_m": 0.0,
            "metric_max_penetration_m": 0.2,
        }
        record = {
            **self.runner.failure_record(item, "placeholder"),
            "load_success": True,
            "measurement_complete": True,
            "rest_state_executed": 1,
            "rest_non_adjacent_free": 1,
            "rest_all_pair_cf": False,
            "rest_non_adjacent_cf": True,
            "single_joint_sweep_cf": True,
            "strict_collision_pass": False,
            "max_penetration_m": 0.2,
            "max_penetration_normalized": 0.1,
            "object_bbox_diagonal_m": 2.0,
            "max_reset_readback_error": 0.0,
            "state_records": [state],
        }
        record["state_records_sha256"] = self.runner.canonical_sha256([state])

        self.runner.validate_state_closure(record)
        with self.assertRaisesRegex(RuntimeError, "state record count"):
            self.runner.validate_state_closure({**record, "state_records": []})
        with self.assertRaisesRegex(RuntimeError, "state record count"):
            self.runner.validate_state_closure(
                {**record, "state_records": [state, state]}
            )

        exported_record = {key: value for key, value in record.items() if key != "state_records"}
        self.assertTrue(self.runner._result_counters_valid(exported_record, [state]))
        self.assertFalse(
            self.runner._result_counters_valid(
                {**exported_record, "strict_collision_pass": True}, [state]
            )
        )
        mutated = [{**state, "metric_max_penetration_m": 0.1}]
        with self.assertRaisesRegex(RuntimeError, "policy mismatch|digest mismatch"):
            self.runner.validate_state_closure(exported_record, mutated)

    def test_state_identity_matches_frozen_category(self) -> None:
        item = {
            "protocol_id": self.runner.PROTOCOL_ID,
            "dataset_id": "1",
            "category": "Fixture",
            "joint_specs": [],
            "rest_state_expected": 1,
            "single_state_expected": 0,
            "sobol_state_expected": 0,
        }
        state = {
            "dataset_id": "1",
            "category": "Other",
            "phase": "rest",
            "sample_index": 0,
            "joint_name": None,
            "reset_readback_max_abs_error": 0.0,
            "all_pair_illegal_penetration_count": 0,
            "all_pair_max_penetration_m": 0.0,
            "non_adjacent_illegal_penetration_count": 0,
            "non_adjacent_max_penetration_m": 0.0,
            "metric_max_penetration_m": 0.0,
        }
        record = {
            **self.runner.failure_record(item, "placeholder"),
            "rest_state_executed": 1,
            "rest_non_adjacent_free": 1,
            "rest_all_pair_cf": True,
            "rest_non_adjacent_cf": True,
            "max_penetration_m": 0.0,
            "max_penetration_normalized": 0.0,
            "max_reset_readback_error": 0.0,
            "object_bbox_diagonal_m": 1.0,
        }
        record["state_records_sha256"] = self.runner.canonical_sha256([state])

        with self.assertRaisesRegex(RuntimeError, "category identity mismatch"):
            self.runner.validate_state_closure(record, [state], item)

    def test_rest_all_pair_penetration_contributes_to_maximum(self) -> None:
        urdf = """<?xml version="1.0"?>
<robot name="rest_penetration_fixture">
  <link name="base"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="child"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="child"/><axis xyz="0 0 1"/>
    <limit lower="-0.1" upper="0.1" effort="1" velocity="1"/>
  </joint>
</robot>
"""
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            asset_dir = dataset_root / "1"
            asset_dir.mkdir()
            urdf_path = asset_dir / "mobility.urdf"
            urdf_path.write_text(urdf, encoding="utf-8")
            inventory = self.runner.collision_mesh_inventory(asset_dir, urdf_path)
            item = {
                "protocol_id": self.runner.PROTOCOL_ID,
                "order": 0,
                "dataset_id": "1",
                "category": "Fixture",
                "package_audit_success": True,
                "missing_collision_mesh_reference_count": 0,
                "movable_dof_count": 1,
                "range_evaluable_dof_count": 1,
                "object_bbox_diagonal_m": 2.0,
                "urdf_sha256": self.runner.sha256_file(urdf_path),
                "bounding_box_sha256": None,
                "collision_mesh_files": inventory,
                "collision_mesh_inventory_sha256": self.runner.canonical_sha256(inventory),
                "rest_state_expected": 1,
                "single_state_expected": 21,
                "sobol_state_expected": 64,
                "input_identity_sha256": "fixture",
            }

            result = self.runner.evaluate_asset(item, dataset_root)

        self.assertFalse(result["rest_all_pair_cf"])
        self.assertTrue(result["rest_non_adjacent_cf"])
        self.assertGreater(result["max_penetration_m"], 0.0)
        self.assertAlmostEqual(
            result["max_penetration_normalized"],
            result["max_penetration_m"] / 2.0,
        )

    def test_single_joint_grid_has_21_states_and_both_endpoints(self) -> None:
        values = self.runner.single_joint_values(
            {"type": "revolute", "lower": -1.0, "upper": 1.0}
        )

        self.assertEqual(len(values), 21)
        self.assertAlmostEqual(values[0], -1.0)
        self.assertAlmostEqual(values[10], 0.0)
        self.assertAlmostEqual(values[-1], 1.0)

    def test_seeded_sobol_has_64_states_for_a_one_joint_asset(self) -> None:
        values = self.runner.sobol_joint_values(
            [{"type": "continuous", "lower": None, "upper": None}],
            seed=20260813,
        )

        self.assertEqual(len(values), 64)
        self.assertEqual(len(values[0]), 1)
        self.assertAlmostEqual(values[0][0], -1.5186851542811064)
        self.assertTrue(all(-math.pi <= row[0] <= math.pi for row in values))

    def test_seeded_sobol_is_frozen_for_two_joint_ranges(self) -> None:
        values = self.runner.sobol_joint_values(
            [
                {"type": "revolute", "lower": -1.0, "upper": 1.0},
                {"type": "continuous", "lower": None, "upper": None},
            ],
            seed=20260813,
        )

        self.assertEqual(len(values), 64)
        self.assertEqual(len(values[0]), 2)
        self.assertAlmostEqual(values[0][0], -0.48341249860823154)
        self.assertAlmostEqual(values[0][1], 2.5997062687267825)

    def test_penetration_threshold_allows_surface_and_exact_boundary(self) -> None:
        self.assertFalse(self.runner.penetration_is_illegal(0.0))
        self.assertFalse(
            self.runner.penetration_is_illegal(-self.runner.PENETRATION_THRESHOLD_M)
        )
        self.assertTrue(
            self.runner.penetration_is_illegal(
                -self.runner.PENETRATION_THRESHOLD_M - 1e-12
            )
        )

    def test_failures_remain_in_asset_and_state_denominators(self) -> None:
        manifest = {
            "sample_size": 2,
            "items": [
                {"dataset_id": "1", "category": "Door", "movable_dof_count": 1},
                {"dataset_id": "2", "category": "Door", "movable_dof_count": 2},
            ],
        }
        records = [
            {
                "dataset_id": "1",
                "category": "Door",
                "load_success": True,
                "measurement_complete": True,
                "rest_all_pair_cf": True,
                "rest_non_adjacent_cf": True,
                "single_joint_sweep_cf": True,
                "multi_joint_sobol_cf": True,
                "strict_collision_pass": True,
                "single_state_expected": 21,
                "single_state_executed": 21,
                "single_non_adjacent_free": 21,
                "sobol_state_expected": 64,
                "sobol_state_executed": 64,
                "sobol_non_adjacent_free": 64,
                "rest_state_executed": 1,
                "rest_non_adjacent_free": 1,
                "max_penetration_normalized": 0.0,
                "child_timed_out": False,
                "child_returncode": 0,
            },
            self.runner.failure_record(manifest["items"][1], "child_timeout", timed_out=True),
        ]

        summary = self.runner.summarize_records(manifest, records)

        self.assertEqual(summary["cohort"]["selected"], 2)
        self.assertEqual(summary["metrics"]["strict_collision_pass"], {"passed": 1, "denominator": 2, "rate": 0.5})
        self.assertEqual(summary["metrics"]["collision_state_rate"]["denominator"], 1 + 21 + 64 + 1 + 42 + 64)
        self.assertEqual(summary["metrics"]["collision_state_rate"]["collision_states"], 1 + 42 + 64)
        self.assertEqual(summary["metrics"]["collision_state_rate"]["observed_collision_states"], 0)
        self.assertEqual(summary["metrics"]["collision_state_rate"]["unexecuted_states"], 1 + 42 + 64)
        self.assertEqual(summary["metrics"]["collision_free_range"], {"passed_states": 21, "denominator": 63, "rate": 1 / 3})

    def test_observed_collision_count_survives_state_record_extraction(self) -> None:
        manifest = {
            "sample_size": 1,
            "items": [{"dataset_id": "1", "category": "Door", "movable_dof_count": 1}],
        }
        record = {
            "dataset_id": "1",
            "category": "Door",
            "load_success": True,
            "measurement_complete": True,
            "rest_all_pair_cf": False,
            "rest_non_adjacent_cf": False,
            "single_joint_sweep_cf": False,
            "multi_joint_sobol_cf": False,
            "strict_collision_pass": False,
            "rest_state_expected": 1,
            "rest_state_executed": 1,
            "rest_non_adjacent_free": 0,
            "single_state_expected": 21,
            "single_state_executed": 21,
            "single_non_adjacent_free": 20,
            "sobol_state_expected": 64,
            "sobol_state_executed": 64,
            "sobol_non_adjacent_free": 62,
            "max_penetration_normalized": 0.01,
            "child_timed_out": False,
            "child_returncode": 0,
        }

        collision = self.runner.summarize_records(manifest, [record])["metrics"]["collision_state_rate"]

        self.assertEqual(collision["observed_collision_states"], 4)
        self.assertEqual(collision["unexecuted_states"], 0)
        self.assertEqual(collision["collision_states"], 4)

    def test_report_exposes_partial_max_penetration_denominator(self) -> None:
        summary = {
            "status": "COMPLETE_WITH_RETAINED_FAILURES",
            "cohort": {"label": "PartNet-Mobility qualification smoke N=2"},
            "metrics": {
                key: {"passed": 1, "denominator": 2, "rate": 0.5}
                for key in (
                    "rest_all_pair_cf",
                    "rest_non_adjacent_cf",
                    "single_joint_sweep_cf",
                    "multi_joint_sobol_cf",
                    "strict_collision_pass",
                )
            },
        }
        summary["metrics"].update(
            {
                "collision_state_rate": {
                    "collision_states": 1,
                    "denominator": 2,
                    "rate": 0.5,
                },
                "aor": {"status": "N/E"},
                "max_penetration": {
                    "maximum_observed_normalized": 0.25,
                    "observed_assets": 1,
                    "fully_measured_assets": 1,
                    "denominator": 2,
                    "status": "PARTIAL",
                },
                "collision_free_range": {
                    "passed_states": 1,
                    "denominator": 2,
                    "rate": 0.5,
                },
            }
        )
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            output = Path(temporary)
            self.runner.render_report(summary, output)
            report = (output / "report.md").read_text(encoding="utf-8")

        self.assertIn("qualification smoke N=2", report)
        self.assertIn("fully measured 1/2; observed 1/2; PARTIAL", report)

    def test_partial_asset_does_not_make_max_penetration_complete(self) -> None:
        manifest = {"sample_size": 1, "items": [{"movable_dof_count": 1}]}
        record = {
            **self.runner.failure_record(
                {
                    "dataset_id": "1",
                    "category": "Fixture",
                    "movable_dof_count": 1,
                },
                "mid_sweep_exception",
            ),
            "load_success": True,
            "rest_state_executed": 1,
            "rest_non_adjacent_free": 1,
            "rest_all_pair_cf": True,
            "rest_non_adjacent_cf": True,
            "max_penetration_normalized": 0.01,
        }

        maximum = self.runner.summarize_records(manifest, [record])["metrics"][
            "max_penetration"
        ]

        self.assertEqual(maximum["observed_assets"], 1)
        self.assertEqual(maximum["fully_measured_assets"], 0)
        self.assertEqual(maximum["status"], "PARTIAL")

    def test_runtime_identity_mismatch_is_fail_closed_before_execution(self) -> None:
        expected = {"python_version": "3.12", "pybullet_module_sha256": "a"}
        observed = {"python_version": "3.12", "pybullet_module_sha256": "b"}

        with self.assertRaisesRegex(RuntimeError, "child runtime identity mismatch"):
            self.runner.require_runtime_match(expected, observed)

    def test_verify_rejects_mutated_report_end_to_end(self) -> None:
        urdf = """<?xml version="1.0"?>
<robot name="verify_fixture"><link name="base"/></robot>
"""
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            output = Path(temporary)
            dataset_root = output / "dataset"
            asset_dir = dataset_root / "1"
            asset_dir.mkdir(parents=True)
            urdf_path = asset_dir / "mobility.urdf"
            urdf_path.write_text(urdf, encoding="utf-8")
            inventory = self.runner.collision_mesh_inventory(asset_dir, urdf_path)
            item = {
                "protocol_id": self.runner.QUALIFICATION_PROTOCOL_ID,
                "order": 0,
                "dataset_id": "1",
                "category": "Fixture",
                "input_identity_sha256": "fixture-input",
                "movable_dof_count": 0,
                "range_evaluable_dof_count": 0,
                "joint_specs": [],
                "joint_specs_sha256": self.runner.canonical_sha256([]),
                "rest_state_expected": 1,
                "single_state_expected": 0,
                "sobol_state_expected": 0,
                "object_bbox_diagonal_m": 1.0,
                "urdf_sha256": self.runner.sha256_file(urdf_path),
                "bounding_box_sha256": None,
                "collision_mesh_files": inventory,
                "collision_mesh_inventory_sha256": self.runner.canonical_sha256(
                    inventory
                ),
            }
            runner_hash = self.runner.sha256_file(RUNNER)
            runtime_identity = self.runner.current_runtime_identity()
            manifest = {
                "protocol_id": self.runner.QUALIFICATION_PROTOCOL_ID,
                "sample_size": 1,
                "cohort_label": "PartNet-Mobility qualification smoke N=1",
                "dataset_root": str(dataset_root),
                "runtime": {
                    "runner_sha256": runner_hash,
                    "child": runtime_identity,
                },
                "items": [item],
            }
            state = {
                "dataset_id": "1",
                "category": "Fixture",
                "phase": "rest",
                "sample_index": 0,
                "joint_name": None,
                "joint_values_sha256": self.runner.canonical_sha256([]),
                "reset_readback_max_abs_error": 0.0,
                "all_pair_contact_count": 0,
                "all_pair_illegal_penetration_count": 0,
                "all_pair_max_penetration_m": 0.0,
                "non_adjacent_contact_count": 0,
                "non_adjacent_illegal_penetration_count": 0,
                "non_adjacent_max_penetration_m": 0.0,
                "metric_max_penetration_m": 0.0,
            }
            record = {
                **self.runner.failure_record(item, "placeholder"),
                "runner_sha256": runner_hash,
                "load_success": True,
                "measurement_complete": True,
                "rest_state_executed": 1,
                "rest_non_adjacent_free": 1,
                "rest_all_pair_cf": True,
                "rest_non_adjacent_cf": True,
                "single_joint_sweep_cf": True,
                "multi_joint_sobol_cf": False,
                "strict_collision_pass": False,
                "max_penetration_m": 0.0,
                "max_penetration_normalized": 0.0,
                "max_reset_readback_error": 0.0,
                "issues": [],
            }
            record.pop("state_records")
            record["state_records_sha256"] = self.runner.canonical_sha256([state])
            summary = self.runner.summarize_records(manifest, [record])
            self.runner.atomic_json(output / "frozen_manifest.json", manifest)
            self.runner.atomic_json(
                output / "child_runtime_probe.json", runtime_identity
            )
            self.runner.atomic_json(output / "asset_records.json", [record])
            self.runner.atomic_jsonl(output / "state_records.jsonl", [state])
            self.runner.atomic_json(output / "summary.json", summary)
            self.runner.render_report(summary, output)
            self.runner.run_pair_policy_smoke(output)
            (output / "report.md").write_text(
                "# mutated report\n", encoding="utf-8"
            )

            with self.assertRaisesRegex(RuntimeError, "verification failed"):
                self.runner.verify(manifest, output)
            receipt = self.runner.read_json(output / "verification.json")

        self.assertEqual(receipt["status"], "FAIL")
        self.assertFalse(receipt["checks"]["report_recomputes_exactly"])

    def test_pair_policy_smoke_distinguishes_direct_parent_contact(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            result = self.runner.run_pair_policy_smoke(Path(temporary))

        self.assertEqual(result["status"], "PASS")
        self.assertGreater(result["all_pair_illegal_penetration_count"], 0)
        self.assertEqual(result["non_adjacent_illegal_penetration_count"], 0)

    def test_missing_collision_mesh_is_retained_as_preload_failure(self) -> None:
        asset_dir = REPO / "exp/PartNet-Mobility/data/dataset/2780"
        audit = self.runner.package_audit(asset_dir)
        item = {
            "order": 0,
            "dataset_id": "2780",
            **audit,
            "rest_state_expected": 1,
            "single_state_expected": 21 * audit["movable_dof_count"],
            "sobol_state_expected": 64,
            "input_identity_sha256": "fixture",
        }

        result = self.runner.evaluate_asset(
            item, REPO / "exp/PartNet-Mobility/data/dataset"
        )

        self.assertEqual(audit["missing_collision_mesh_reference_count"], 3)
        self.assertFalse(result["load_success"])
        self.assertFalse(result["measurement_complete"])
        self.assertIn("missing_collision_mesh_references:3", result["issues"])
        self.assertEqual(result["single_state_expected"], 273)
        self.assertEqual(result["sobol_state_expected"], 64)

    def test_sobol_asset_pass_requires_every_declared_joint_range(self) -> None:
        urdf = """<?xml version="1.0"?>
<robot name="invalid_range_fixture">
  <link name="base"/>
  <link name="valid_child"/>
  <link name="zero_width_child"/>
  <joint name="valid" type="revolute">
    <parent link="base"/><child link="valid_child"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="zero_width" type="revolute">
    <parent link="base"/><child link="zero_width_child"/>
    <axis xyz="0 1 0"/><limit lower="0" upper="0" effort="1" velocity="1"/>
  </joint>
</robot>
"""
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            asset_dir = dataset_root / "1"
            asset_dir.mkdir()
            urdf_path = asset_dir / "mobility.urdf"
            urdf_path.write_text(urdf, encoding="utf-8")
            inventory = self.runner.collision_mesh_inventory(asset_dir, urdf_path)
            item = {
                "protocol_id": self.runner.PROTOCOL_ID,
                "order": 0,
                "dataset_id": "1",
                "category": "Fixture",
                "package_audit_success": True,
                "missing_collision_mesh_reference_count": 0,
                "movable_dof_count": 2,
                "range_evaluable_dof_count": 1,
                "object_bbox_diagonal_m": 1.0,
                "urdf_sha256": self.runner.sha256_file(urdf_path),
                "bounding_box_sha256": None,
                "collision_mesh_files": inventory,
                "collision_mesh_inventory_sha256": self.runner.canonical_sha256(
                    inventory
                ),
                "rest_state_expected": 1,
                "single_state_expected": 42,
                "sobol_state_expected": 64,
                "input_identity_sha256": "fixture",
            }

            result = self.runner.evaluate_asset(item, dataset_root)

        self.assertEqual(result["sobol_state_executed"], 0)
        self.assertEqual(result["sobol_non_adjacent_free"], 0)
        self.assertEqual(result["sobol_state_expected"], 64)
        self.assertFalse(result["multi_joint_sobol_cf"])
        self.assertFalse(result["measurement_complete"])
        self.assertIn("joint_range_not_evaluable:zero_width", result["issues"])


if __name__ == "__main__":
    unittest.main()
