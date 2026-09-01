from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4_artiverse.py"
TABLE1_MANIFEST = REPO / "exp/runtime/table1_artiverse/manifest.json"
DATASET_ROOT = REPO / "exp/artiverse"


def load_runner():
    spec = importlib.util.spec_from_file_location("urdf_table4_artiverse", RUNNER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {RUNNER}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ArtiverseTable4ProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.runner = load_runner()

    def test_formal_cohort_is_exact_table1_order_without_resampling(self) -> None:
        contract = self.runner.load_table1_cohort(
            TABLE1_MANIFEST, sample_size=800, qualification_smoke=False
        )

        self.assertEqual(len(contract["selected"]), 800)
        self.assertEqual(
            contract["selected"][0]["manifest_root"],
            "data/nightstand/3dfModel/06d0beb2-9724-4e98-929d-80b23a31775a",
        )
        self.assertEqual(
            contract["selected"][-1]["manifest_root"],
            "data/switch/3dw/c1c44f9e0a20fa4aeba36c406fb6df1a",
        )
        self.assertEqual(contract["source_manifest_sha256"], self.runner.EXPECTED_TABLE1_MANIFEST_SHA256)
        self.assertEqual(contract["protocol_id"], self.runner.PROTOCOL_ID)
        self.assertEqual(
            contract["ordered_asset_identities_sha256"],
            self.runner.EXPECTED_ORDERED_ASSET_IDENTITIES_SHA256,
        )
        self.assertEqual(
            [row["selection_rank"] for row in contract["selected"]],
            list(range(1, 801)),
        )

        with self.assertRaisesRegex(ValueError, "formal protocol requires sample_size=800"):
            self.runner.load_table1_cohort(
                TABLE1_MANIFEST, sample_size=3, qualification_smoke=False
            )

    def test_qualification_uses_prefix_of_frozen_table1_order(self) -> None:
        contract = self.runner.load_table1_cohort(
            TABLE1_MANIFEST, sample_size=3, qualification_smoke=True
        )

        self.assertEqual(
            [row["selection_rank"] for row in contract["selected"]], [1, 2, 3]
        )
        self.assertEqual(contract["protocol_id"], self.runner.QUALIFICATION_PROTOCOL_ID)
        self.assertTrue(contract["qualification_smoke"])

    def test_asset_audit_resolves_nested_urdf_and_collision_mesh(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            asset_root = dataset_root / "data/door/source/model"
            urdf_dir = asset_root / "urdf_w_collider"
            mesh_dir = urdf_dir / "objs"
            mesh_dir.mkdir(parents=True)
            urdf_path = urdf_dir / "model.urdf"
            urdf_path.write_text(
                """<?xml version="1.0"?>
<robot name="fixture">
  <link name="base"><collision><geometry><mesh filename="./objs/mesh.obj"/></geometry></collision></link>
  <link name="door"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="door"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
""",
                encoding="utf-8",
            )
            mesh_path = mesh_dir / "mesh.obj"
            mesh_path.write_text(
                "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n"
                "f 1 2 3\nf 1 2 4\nf 1 3 4\nf 2 3 4\n",
                encoding="utf-8",
            )
            source_row = {
                "asset_id": "data/door/source/model",
                "manifest_root": "data/door/source/model",
                "model_id": "model",
                "raw_category": "door",
                "source": "source",
                "chunk_archive": "chunk.tar.gz",
                "selection_hash": "abc",
                "selection_rank": 1,
            }

            audit = self.runner.audit_asset(dataset_root, source_row)

        self.assertTrue(audit["package_audit_success"])
        self.assertEqual(
            audit["primary_urdf_relpath"],
            "data/door/source/model/urdf_w_collider/model.urdf",
        )
        self.assertEqual(audit["movable_dof_count"], 1)
        self.assertEqual(audit["range_evaluable_dof_count"], 1)
        self.assertEqual(audit["missing_collision_mesh_reference_count"], 0)
        self.assertEqual(audit["unsafe_collision_mesh_reference_count"], 0)
        self.assertEqual(len(audit["collision_mesh_files"]), 1)
        self.assertEqual(audit["collision_mesh_files"][0]["path"], "./objs/mesh.obj")
        self.assertTrue(audit["collision_mesh_files"][0]["exists"])
        self.assertEqual(audit["scale_derivation"]["status"], "PASS")
        self.assertGreater(audit["object_bbox_diagonal_m"], 0.0)

    def test_collision_mesh_escape_is_a_retained_package_failure(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            asset_root = dataset_root / "data/door/source/model"
            urdf_dir = asset_root / "urdf_w_collider"
            urdf_dir.mkdir(parents=True)
            (asset_root / "outside.obj").write_text("v 0 0 0\n", encoding="utf-8")
            (urdf_dir / "model.urdf").write_text(
                """<robot name="fixture"><link name="base"><collision><geometry>
<mesh filename="../outside.obj"/></geometry></collision></link></robot>""",
                encoding="utf-8",
            )
            source_row = {
                "asset_id": "data/door/source/model",
                "manifest_root": "data/door/source/model",
                "model_id": "model",
                "raw_category": "door",
                "source": "source",
                "chunk_archive": "chunk.tar.gz",
                "selection_hash": "abc",
                "selection_rank": 1,
            }

            audit = self.runner.audit_asset(dataset_root, source_row)

        self.assertFalse(audit["package_audit_success"])
        self.assertEqual(audit["unsafe_collision_mesh_reference_count"], 1)
        self.assertIn("unsafe collision mesh reference", audit["audit_issue"])

    def test_collision_aabb_uses_loaded_rest_collision_geometry(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            urdf_path = Path(temporary) / "box.urdf"
            urdf_path.write_text(
                """<robot name="box"><link name="base"><collision><geometry>
<box size="2 4 6"/></geometry></collision></link></robot>""",
                encoding="utf-8",
            )

            scale = self.runner.derive_collision_aabb(urdf_path)

        self.assertEqual(scale["status"], "PASS")
        self.assertEqual(scale["collision_link_indices"], [-1])
        self.assertTrue(
            math.isclose(scale["diagonal_m"], math.sqrt(56.0), abs_tol=1e-9)
        )
        self.assertEqual(scale["protocol"], "pybullet_q0_collision_shape_union_aabb_v1")

    def test_frozen_source_validation_rejects_mesh_content_drift(self) -> None:
        source_row = json.loads(TABLE1_MANIFEST.read_text(encoding="utf-8"))["assets"][0]
        audit = self.runner.audit_asset(DATASET_ROOT, source_row)
        item = {
            "asset_root_relpath": source_row["manifest_root"],
            **audit,
        }
        mesh = next(row for row in item["collision_mesh_files"] if row["exists"])
        original_sha = mesh["sha256"]
        mesh["sha256"] = "0" * 64
        self.assertNotEqual(original_sha, mesh["sha256"])

        with self.assertRaisesRegex(RuntimeError, "collision mesh inventory drift"):
            self.runner.validate_frozen_asset_files(item, DATASET_ROOT)

    def test_report_names_artiverse_and_preserves_claim_boundary(self) -> None:
        summary = {
            "status": "COMPLETE_WITH_RETAINED_FAILURES",
            "cohort": {"label": "Artiverse Table 1 fixed N=800 cohort"},
            "metrics": {
                "rest_all_pair_cf": {"passed": 1, "denominator": 3, "rate": 1 / 3},
                "rest_non_adjacent_cf": {"passed": 2, "denominator": 3, "rate": 2 / 3},
                "single_joint_sweep_cf": {"passed": 1, "denominator": 3, "rate": 1 / 3},
                "multi_joint_sobol_cf": {"passed": 1, "denominator": 3, "rate": 1 / 3},
                "collision_state_rate": {
                    "collision_states": 4,
                    "denominator": 10,
                    "rate": 0.4,
                },
                "aor": {"status": "N/E"},
                "max_penetration": {
                    "maximum_observed_normalized": 0.25,
                    "fully_measured_assets": 2,
                    "observed_assets": 2,
                    "denominator": 3,
                    "status": "PARTIAL",
                },
                "collision_free_range": {
                    "passed_states": 5,
                    "denominator": 9,
                    "rate": 5 / 9,
                },
                "strict_collision_pass": {"passed": 1, "denominator": 3, "rate": 1 / 3},
            },
        }

        report = self.runner.report_text(summary)

        self.assertIn("Artiverse Table 1 fixed N=800 cohort", report)
        self.assertIn("pre-release 3,544-asset universe", report)
        self.assertIn("AOR | N/E", report)
        self.assertNotIn("PartNet", report)
        self.assertNotIn("2,347", report)

    def test_frozen_invalid_tree_retains_declared_state_denominators(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            asset_root = dataset_root / "data/desk/source/cycle"
            package = asset_root / "urdf_w_collider"
            package.mkdir(parents=True)
            (package / "cycle.urdf").write_text(
                """<robot name="cycle">
<link name="a"/><link name="b"/>
<joint name="forward" type="revolute"><parent link="a"/><child link="b"/>
<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
<joint name="back" type="fixed"><parent link="b"/><child link="a"/></joint>
</robot>""",
                encoding="utf-8",
            )
            source_row = {
                "asset_id": "data/desk/source/cycle",
                "manifest_root": "data/desk/source/cycle",
                "model_id": "cycle",
                "raw_category": "desk",
                "source": "source",
                "chunk_archive": "chunk.tar.gz",
                "selection_hash": "abc",
                "selection_rank": 1,
            }

            audit = self.runner.audit_asset(dataset_root, source_row)
            item = self.runner.freeze_item(
                source_row,
                audit,
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity={"fixture": True},
            )

        self.assertFalse(item["package_audit_success"])
        self.assertEqual(item["movable_dof_count"], 1)
        self.assertEqual(item["rest_state_expected"], 1)
        self.assertEqual(item["single_state_expected"], 21)
        self.assertEqual(item["sobol_state_expected"], 64)
        self.assertEqual(item["asset_id"], source_row["asset_id"])
        self.assertEqual(item["dataset_id"], "artiverse_0000")

    def test_evaluate_asset_runs_frozen_states_and_preserves_source_identity(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            asset_root = dataset_root / "data/door/source/model"
            package = asset_root / "urdf_w_collider"
            package.mkdir(parents=True)
            (package / "model.urdf").write_text(
                """<robot name="fixture">
<link name="base"><collision><origin xyz="0 0 0"/><geometry><box size="1 1 1"/></geometry></collision></link>
<link name="door"><collision><origin xyz="0 0 0"/><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
<joint name="slide" type="prismatic"><parent link="base"/><child link="door"/>
<origin xyz="2 0 0"/><axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint>
</robot>""",
                encoding="utf-8",
            )
            source_row = {
                "asset_id": "data/door/source/model",
                "manifest_root": "data/door/source/model",
                "model_id": "model",
                "raw_category": "door",
                "source": "source",
                "chunk_archive": "chunk.tar.gz",
                "selection_hash": "abc",
                "selection_rank": 1,
            }
            audit = self.runner.audit_asset(dataset_root, source_row)
            runtime = self.runner.current_runtime_identity()
            item = self.runner.freeze_item(
                source_row,
                audit,
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity=runtime,
            )

            result = self.runner.evaluate_asset(item, dataset_root)

        self.assertTrue(result["load_success"])
        self.assertTrue(result["measurement_complete"])
        self.assertEqual(result["asset_id"], source_row["asset_id"])
        self.assertEqual(result["manifest_root"], source_row["manifest_root"])
        self.assertEqual(result["selection_rank"], 1)
        self.assertEqual(result["rest_state_executed"], 1)
        self.assertEqual(result["single_state_executed"], 21)
        self.assertEqual(result["sobol_state_executed"], 64)
        self.assertEqual(len(result["state_records"]), 86)
        self.assertEqual(
            result["state_records_sha256"],
            self.runner.canonical_sha256(result["state_records"]),
        )

    def test_package_failure_executes_zero_states_but_keeps_expected_counts(self) -> None:
        source_row = {
            "asset_id": "data/desk/source/cycle",
            "manifest_root": "data/desk/source/cycle",
            "model_id": "cycle",
            "raw_category": "desk",
            "source": "source",
            "chunk_archive": "chunk.tar.gz",
            "selection_hash": "abc",
            "selection_rank": 1,
        }
        cycle_urdf = """<?xml version="1.0"?>
<robot name="cycle">
  <link name="a"/><link name="b"/>
  <joint name="hinge" type="revolute">
    <parent link="a"/><child link="b"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="back" type="fixed"><parent link="b"/><child link="a"/></joint>
</robot>
"""
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = (
                dataset_root / source_row["manifest_root"] / "urdf_w_collider"
            )
            package.mkdir(parents=True)
            (package / "cycle.urdf").write_text(cycle_urdf, encoding="utf-8")
            audit = self.runner.audit_asset(dataset_root, source_row)
            item = self.runner.freeze_item(
                source_row,
                audit,
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity={"fixture": True},
            )

            result = self.runner.evaluate_asset(item, dataset_root)

        self.assertFalse(result["load_success"])
        self.assertFalse(result["measurement_complete"])
        self.assertEqual(result["rest_state_expected"], 1)
        self.assertEqual(result["single_state_expected"], 21)
        self.assertEqual(result["sobol_state_expected"], 64)
        self.assertEqual(result["rest_state_executed"], 0)
        self.assertEqual(result["single_state_executed"], 0)
        self.assertEqual(result["sobol_state_executed"], 0)
        self.assertIn("valid rooted tree", result["issues"][0])

    def test_package_failure_snapshot_replays_full_audit(self) -> None:
        source_row = {
            "asset_id": "data/desk/source/model",
            "manifest_root": "data/desk/source/model",
            "model_id": "model",
            "raw_category": "desk",
            "source": "source",
            "chunk_archive": "chunk.tar.gz",
            "selection_hash": "abc",
            "selection_rank": 1,
        }
        audit = self.runner._empty_audit()
        audit.update(
            {
                "primary_urdf_relpath": "data/desk/source/model/urdf_w_collider/model.urdf",
                "audit_issue": "RuntimeError: transient scale failure",
            }
        )
        item = self.runner.freeze_item(
            source_row,
            audit,
            order=0,
            protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
            runtime_identity={"fixture": True},
        )
        recovered = {**audit, "package_audit_success": True, "audit_issue": None}

        with mock.patch.object(self.runner, "audit_asset", return_value=recovered):
            with self.assertRaisesRegex(RuntimeError, "package audit snapshot drift"):
                self.runner.validate_frozen_source_snapshot(item, Path("/fixture"))

    def test_build_qualification_manifest_binds_real_table1_prefix(self) -> None:
        runtime = self.runner.current_runtime_identity()

        manifest = self.runner.build_manifest(
            DATASET_ROOT,
            TABLE1_MANIFEST,
            sample_size=3,
            qualification_smoke=True,
            child_runtime=runtime,
            workers=2,
        )

        self.assertEqual(manifest["sample_size"], 3)
        self.assertEqual(manifest["release_asset_count"], 3544)
        self.assertEqual(
            [item["asset_id"] for item in manifest["items"]],
            [
                row["asset_id"]
                for row in json.loads(TABLE1_MANIFEST.read_text(encoding="utf-8"))[
                    "assets"
                ][:3]
            ],
        )
        self.assertEqual(
            manifest["source"]["table1_manifest_sha256"],
            self.runner.EXPECTED_TABLE1_MANIFEST_SHA256,
        )
        self.assertEqual(
            manifest["runtime"]["collision_core_sha256"],
            self.runner.EXPECTED_CORE_SHA256,
        )
        self.assertEqual(
            manifest["items_sha256"],
            self.runner.canonical_sha256(manifest["items"]),
        )

    def test_result_binding_rejects_changed_source_identity(self) -> None:
        source_row = {
            "asset_id": "data/desk/source/model",
            "manifest_root": "data/desk/source/model",
            "model_id": "model",
            "raw_category": "desk",
            "source": "source",
            "chunk_archive": "chunk.tar.gz",
            "selection_hash": "abc",
            "selection_rank": 1,
        }
        item = self.runner.freeze_item(
            source_row,
            self.runner._empty_audit(),
            order=0,
            protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
            runtime_identity={"fixture": True},
        )
        result = self.runner.failure_record(item, "fixture_failure")
        runner_hash = result["runner_sha256"]

        self.assertTrue(self.runner.result_matches_item(result, item, runner_hash))
        result["asset_id"] = "data/desk/source/other"
        self.assertFalse(self.runner.result_matches_item(result, item, runner_hash))

    def test_summarize_records_counts_unexecuted_states_fail_closed(self) -> None:
        rows = []
        items = []
        for order in range(2):
            source_row = {
                "asset_id": f"data/desk/source/model{order}",
                "manifest_root": f"data/desk/source/model{order}",
                "model_id": f"model{order}",
                "raw_category": "desk",
                "source": "source",
                "chunk_archive": "chunk.tar.gz",
                "selection_hash": f"hash{order}",
                "selection_rank": order + 1,
            }
            audit = self.runner._empty_audit()
            audit.update(
                {
                    "movable_dof_count": 1,
                    "range_evaluable_dof_count": 1,
                    "object_bbox_diagonal_m": 1.0,
                }
            )
            item = self.runner.freeze_item(
                source_row,
                audit,
                order=order,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity={"fixture": True},
            )
            items.append(item)
            rows.append(self.runner.failure_record(item, "fixture_failure"))
        manifest = {
            "protocol_id": self.runner.QUALIFICATION_PROTOCOL_ID,
            "cohort_label": "Artiverse Table 1 qualification N=2",
            "sample_size": 2,
            "items": items,
        }

        summary = self.runner.summarize_records(manifest, rows)

        self.assertEqual(summary["cohort"]["selected"], 2)
        self.assertEqual(summary["metrics"]["collision_state_rate"]["denominator"], 172)
        self.assertEqual(summary["metrics"]["collision_state_rate"]["collision_states"], 172)
        self.assertEqual(summary["metrics"]["collision_state_rate"]["unexecuted_states"], 172)
        self.assertEqual(
            summary["metrics"]["max_penetration"]["normalization"],
            "PyBullet q=0 collision-shape union AABB diagonal",
        )

    def test_state_closure_rejects_artiverse_source_identity_drift(self) -> None:
        source_row = {
            "asset_id": "data/desk/source/model",
            "manifest_root": "data/desk/source/model",
            "model_id": "model",
            "raw_category": "desk",
            "source": "source",
            "chunk_archive": "chunk.tar.gz",
            "selection_hash": "abc",
            "selection_rank": 1,
        }
        audit = self.runner._empty_audit()
        audit["object_bbox_diagonal_m"] = 2.0
        item = self.runner.freeze_item(
            source_row,
            audit,
            order=0,
            protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
            runtime_identity={"fixture": True},
        )
        state = {
            "dataset_id": item["dataset_id"],
            **{key: item[key] for key in self.runner.IDENTITY_FIELDS},
            "category": item["category"],
            "protocol_id": item["protocol_id"],
            "order": item["order"],
            "input_identity_sha256": item["input_identity_sha256"],
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
        record = self.runner.failure_record(item, "fixture")
        record.update(
            {
                "load_success": True,
                "measurement_complete": True,
                "rest_state_executed": 1,
                "rest_non_adjacent_free": 1,
                "rest_all_pair_cf": True,
                "rest_non_adjacent_cf": True,
                "single_joint_sweep_cf": True,
                "max_penetration_m": 0.0,
                "max_penetration_normalized": 0.0,
                "max_reset_readback_error": 0.0,
                "state_records_sha256": self.runner.canonical_sha256([state]),
            }
        )

        self.runner.validate_state_closure(record, [state], item)
        mutated = [{**state, "asset_id": "data/desk/source/other"}]
        with self.assertRaisesRegex(RuntimeError, "source identity mismatch"):
            self.runner.validate_state_closure(record, mutated, item)

    def test_manifest_validation_rejects_mutated_table1_identity(self) -> None:
        runtime = self.runner.current_runtime_identity()
        manifest = self.runner.build_manifest(
            DATASET_ROOT,
            TABLE1_MANIFEST,
            sample_size=1,
            qualification_smoke=True,
            child_runtime=runtime,
            workers=1,
        )
        manifest["items"][0]["asset_id"] = "data/desk/source/forged"
        manifest["items_sha256"] = self.runner.canonical_sha256(manifest["items"])

        with self.assertRaisesRegex(RuntimeError, "source identity|item identity"):
            self.runner.validate_manifest(
                manifest,
                DATASET_ROOT,
                TABLE1_MANIFEST,
                qualification_smoke=True,
                child_runtime=runtime,
            )

    def test_n1_fail_closed_artifacts_verify_and_report_tamper_fails(self) -> None:
        runtime = self.runner.current_runtime_identity()
        manifest = self.runner.build_manifest(
            DATASET_ROOT,
            TABLE1_MANIFEST,
            sample_size=1,
            qualification_smoke=True,
            child_runtime=runtime,
            workers=1,
        )
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            output = Path(temporary)
            self.runner.atomic_json(output / "frozen_manifest.json", manifest)
            self.runner.atomic_json(output / "child_runtime_probe.json", runtime)
            result = self.runner.failure_record(
                manifest["items"][0], "intentional_test_failure"
            )
            child_path = self.runner.child_result_path(output, manifest["items"][0])
            self.runner.atomic_json(child_path, result)
            self.runner.run_pair_policy_smoke(output)

            summary = self.runner.summarize(manifest, output)
            receipt = self.runner.verify(manifest, output)

            self.assertEqual(receipt["status"], "PASS")
            self.assertEqual(
                summary["metrics"]["collision_state_rate"]["unexecuted_states"],
                summary["metrics"]["collision_state_rate"]["denominator"],
            )
            (output / "report.md").write_text("# tampered\n", encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "verification failed"):
                self.runner.verify(manifest, output)
            failed = self.runner.read_json(output / "verification.json")

        self.assertEqual(failed["status"], "FAIL")
        self.assertFalse(failed["checks"]["report_recomputes_exactly"])

    def test_prepare_rejects_existing_manifest_with_different_sample_size(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            output = Path(temporary)
            python = Path(self.runner.sys.executable)
            self.runner.prepare(
                DATASET_ROOT,
                TABLE1_MANIFEST,
                output,
                sample_size=1,
                qualification_smoke=True,
                child_python=python,
                workers=1,
            )

            with self.assertRaisesRegex(RuntimeError, "requested sample size"):
                self.runner.prepare(
                    DATASET_ROOT,
                    TABLE1_MANIFEST,
                    output,
                    sample_size=2,
                    qualification_smoke=True,
                    child_python=python,
                    workers=1,
                )


if __name__ == "__main__":
    unittest.main()
