from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
import copy
from pathlib import Path
import subprocess
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4_articraft10k.py"
SOURCE_MANIFEST = (
    REPO
    / "exp/runtime/table2_urdf_articraft10k_n800_seed20260813_20260813T145915Z/manifest.json"
)
DATASET_ROOT = REPO / "exp/Articraft-10K"
FROZEN_PROTOCOL_DOCUMENT = (
    REPO
    / "exp/runtime/urdf_table4_articraft10k_n800_20260814/protocol_document_at_freeze.md"
)


def load_runner():
    spec = importlib.util.spec_from_file_location("urdf_table4_articraft10k", RUNNER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {RUNNER}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def canonical_sha256(value) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def package_binding(package: Path) -> dict:
    files = []
    for current_raw, directories, names in os.walk(package, followlinks=False):
        current = Path(current_raw)
        directories.sort()
        names.sort()
        for name in names:
            path = current / name
            relative = path.relative_to(package).as_posix()
            files.append(
                {
                    "path": relative,
                    "bytes": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )
    return {
        "file_count": len(files),
        "total_bytes": sum(row["bytes"] for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def write_package(dataset_root: Path, asset_id: str, urdf: str) -> Path:
    package = dataset_root / "released_urdf" / asset_id
    package.mkdir(parents=True)
    (package / "model.urdf").write_text(urdf, encoding="utf-8")
    (package / "compile_report.json").write_text(
        json.dumps({"schema_version": 1, "record_id": asset_id, "status": "success"})
        + "\n",
        encoding="utf-8",
    )
    return package


def source_row(package: Path, selection_index: int = 0) -> dict:
    binding = package_binding(package)
    return {
        "asset_id": package.name,
        "selection_index": selection_index,
        "package": str(package.resolve()),
        "package_relpath": f"released_urdf/{package.name}",
        "model_urdf_sha256": sha256_file(package / "model.urdf"),
        "package_binding": binding,
        "package_content_manifest_sha256": binding["content_manifest_sha256"],
        "source_record_sha256": canonical_sha256(
            {
                "asset_id": package.name,
                "selection_index": selection_index,
                "package": str(package.resolve()),
                "model_urdf_sha256": sha256_file(package / "model.urdf"),
                "package_binding": binding,
            }
        ),
    }


VALID_URDF = """<robot name="fixture">
<link name="base"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
<link name="door"><collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision></link>
<joint name="slide" type="prismatic"><parent link="base"/><child link="door"/>
<origin xyz="2 0 0"/><axis xyz="1 0 0"/>
<limit lower="0" upper="1" effort="1" velocity="1"/></joint>
</robot>
"""

OVERLAPPING_URDF = """<robot name="overlapping_fixture">
<link name="base"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
<link name="door"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
<joint name="slide" type="prismatic"><parent link="base"/><child link="door"/>
<origin xyz="0 0 0"/><axis xyz="1 0 0"/>
<limit lower="0" upper="0.1" effort="1" velocity="1"/></joint>
</robot>
"""

VISUAL_ONLY_URDF = """<robot name="visual_only">
<link name="base"><visual><geometry><box size="4 4 4"/></geometry></visual></link>
<link name="door"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
<joint name="hinge" type="revolute"><parent link="base"/><child link="door"/>
<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>
"""

PARTIAL_COLLISION_URDF = """<robot name="partial">
<link name="base"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
<link name="door"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
<joint name="hinge" type="revolute"><parent link="base"/><child link="door"/>
<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
</robot>
"""


class Articraft10KTable4ProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.runner = load_runner()
        if (
            sha256_file(FROZEN_PROTOCOL_DOCUMENT)
            != cls.runner.EXPECTED_PROTOCOL_DOCUMENT_SHA256
        ):
            raise RuntimeError("frozen Table 4 protocol fixture hash mismatch")
        cls.runner.PROTOCOL_DOCUMENT = FROZEN_PROTOCOL_DOCUMENT

    def test_formal_cohort_is_exact_table2_order_without_resampling(self) -> None:
        contract = self.runner.load_table2_cohort(
            SOURCE_MANIFEST,
            DATASET_ROOT,
            sample_size=800,
            qualification_smoke=False,
        )

        self.assertEqual(len(contract["selected"]), 800)
        self.assertEqual(
            contract["selected"][0]["asset_id"],
            "rec_fidget_toy_d3e5dba051334aca88e7cba99ac794df",
        )
        self.assertEqual(
            contract["selected"][-1]["asset_id"],
            "rec_bell_tower_with_swinging_bell_c2ebae809533486fba06f1158cfe13c2",
        )
        self.assertEqual(
            [row["selection_index"] for row in contract["selected"]],
            list(range(800)),
        )
        source_records = json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))[
            "records"
        ]
        self.assertEqual(
            [row["asset_id"] for row in contract["selected"]],
            [row["asset_id"] for row in source_records],
        )
        self.assertEqual(
            [row["model_urdf_sha256"] for row in contract["selected"]],
            [row["model_urdf_sha256"] for row in source_records],
        )
        self.assertEqual(
            [row["package_binding"] for row in contract["selected"]],
            [row["package_binding"] for row in source_records],
        )
        self.assertEqual(
            contract["source_manifest_sha256"],
            "13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d",
        )
        self.assertEqual(
            contract["source_manifest_content_sha256"],
            "576852cb6da00775e1c51360b82b4be40e0a614e4fb0cfb1bae066912eed56a3",
        )
        self.assertEqual(
            contract["selected_asset_ids_sha256"],
            "79c44441600077513d3cde1cda8fef38324e1a0ee660730b860d5313f0ae9784",
        )
        self.assertEqual(contract["release_asset_count"], 9996)
        with self.assertRaisesRegex(ValueError, "formal protocol requires sample_size=800"):
            self.runner.load_table2_cohort(
                SOURCE_MANIFEST,
                DATASET_ROOT,
                sample_size=3,
                qualification_smoke=False,
            )

    def test_complete_collision_package_audit_derives_collision_scale(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(dataset_root, "rec_fixture", VALID_URDF)
            row = source_row(package)

            audit = self.runner.audit_asset(dataset_root, row)

        self.assertTrue(audit["package_audit_success"])
        self.assertTrue(audit["collision_coverage_complete"])
        self.assertEqual(audit["link_count"], 2)
        self.assertEqual(audit["collision_covered_link_count"], 2)
        self.assertEqual(audit["movable_dof_count"], 1)
        self.assertEqual(audit["range_evaluable_dof_count"], 1)
        self.assertEqual(audit["primary_urdf_relpath"], "released_urdf/rec_fixture/model.urdf")
        self.assertEqual(audit["scale_derivation"]["status"], "PASS")
        self.assertGreater(audit["object_bbox_diagonal_m"], 0.0)

    def test_visual_only_package_is_fail_closed_without_scale_fallback(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(dataset_root, "rec_visual", VISUAL_ONLY_URDF)
            row = source_row(package)
            audit = self.runner.audit_asset(dataset_root, row)
            item = self.runner.freeze_item(
                row,
                audit,
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity={"fixture": True},
            )

        self.assertFalse(item["package_audit_success"])
        self.assertFalse(item["collision_coverage_complete"])
        self.assertEqual(item["collision_element_count"], 0)
        self.assertIsNone(item["object_bbox_diagonal_m"])
        self.assertEqual(item["scale_derivation"]["status"], "N/E")
        self.assertIn("collision coverage incomplete", item["audit_issue"])
        self.assertEqual(item["rest_state_expected"], 1)
        self.assertEqual(item["single_state_expected"], 21)
        self.assertEqual(item["sobol_state_expected"], 64)

    def test_partial_collision_coverage_is_a_retained_failure(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(dataset_root, "rec_partial", PARTIAL_COLLISION_URDF)
            row = source_row(package)

            audit = self.runner.audit_asset(dataset_root, row)

        self.assertFalse(audit["package_audit_success"])
        self.assertEqual(audit["link_count"], 2)
        self.assertEqual(audit["collision_covered_link_count"], 1)
        self.assertEqual(audit["collision_element_count"], 1)
        self.assertIsNone(audit["object_bbox_diagonal_m"])
        self.assertEqual(audit["scale_derivation"]["status"], "N/E")
        self.assertIn("collision coverage incomplete: 1/2", audit["audit_issue"])

    def test_partial_collision_failure_keeps_states_fail_closed_end_to_end(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(
                dataset_root, "rec_partial_chain", PARTIAL_COLLISION_URDF
            )
            row = source_row(package)
            audit = self.runner.audit_asset(dataset_root, row)
            item = self.runner.freeze_item(
                row,
                audit,
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity={"fixture": True},
            )
            result = self.runner.evaluate_asset(item, dataset_root)
            summary = self.runner.summarize_records(
                {
                    "protocol_id": self.runner.QUALIFICATION_PROTOCOL_ID,
                    "cohort_label": "Articraft-10K qualification N=1",
                    "sample_size": 1,
                    "items": [item],
                },
                [result],
            )

        self.assertFalse(result["load_success"])
        self.assertFalse(result["measurement_complete"])
        self.assertEqual(item["movable_dof_count"], 1)
        self.assertEqual(result["rest_state_expected"], 1)
        self.assertEqual(result["single_state_expected"], 21)
        self.assertEqual(result["sobol_state_expected"], 64)
        self.assertEqual(result["rest_state_executed"], 0)
        self.assertEqual(result["single_state_executed"], 0)
        self.assertEqual(result["sobol_state_executed"], 0)
        self.assertFalse(result["strict_collision_pass"])
        self.assertEqual(
            summary["metrics"]["collision_state_rate"]["denominator"], 86
        )
        self.assertEqual(
            summary["metrics"]["collision_state_rate"]["collision_states"], 86
        )
        self.assertEqual(
            summary["metrics"]["collision_state_rate"]["unexecuted_states"], 86
        )

    def test_full_package_snapshot_detects_unreferenced_file_drift(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(dataset_root, "rec_drift", VALID_URDF)
            row = source_row(package)
            audit = self.runner.audit_asset(dataset_root, row)
            item = self.runner.freeze_item(
                row,
                audit,
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity={"fixture": True},
            )
            (package / "compile_report.json").write_text("{}\n", encoding="utf-8")

            with self.assertRaisesRegex(RuntimeError, "package binding drift"):
                self.runner.validate_frozen_source_snapshot(item, dataset_root)

    def test_package_binding_rejects_symlink_entries(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            package = Path(temporary) / "package"
            package.mkdir()
            target = package / "target.txt"
            target.write_text("bound content\n", encoding="utf-8")
            (package / "alias.txt").symlink_to(target.name)

            with self.assertRaisesRegex(RuntimeError, "file symlink"):
                self.runner.package_binding(package)

    def test_failed_package_snapshot_is_not_exempt_from_drift_checks(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(dataset_root, "rec_failed_drift", VISUAL_ONLY_URDF)
            row = source_row(package)
            audit = self.runner.audit_asset(dataset_root, row)
            item = self.runner.freeze_item(
                row,
                audit,
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity={"fixture": True},
            )
            self.assertFalse(item["package_audit_success"])
            (package / "compile_report.json").write_text("{}\n", encoding="utf-8")

            with self.assertRaisesRegex(RuntimeError, "package binding drift"):
                self.runner.validate_frozen_source_snapshot(item, dataset_root)

    def test_evaluate_asset_runs_all_frozen_states_and_binds_source_identity(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(dataset_root, "rec_evaluate", VALID_URDF)
            row = source_row(package)
            audit = self.runner.audit_asset(dataset_root, row)
            item = self.runner.freeze_item(
                row,
                audit,
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity=self.runner.current_runtime_identity(),
            )

            result = self.runner.evaluate_asset(item, dataset_root)

        self.assertTrue(result["load_success"])
        self.assertTrue(result["measurement_complete"])
        self.assertEqual(result["asset_id"], "rec_evaluate")
        self.assertEqual(result["package_relpath"], "released_urdf/rec_evaluate")
        self.assertEqual(result["selection_index"], 0)
        self.assertEqual(result["rest_state_executed"], 1)
        self.assertEqual(result["single_state_executed"], 21)
        self.assertEqual(result["sobol_state_executed"], 64)
        self.assertEqual(len(result["state_records"]), 86)
        self.assertEqual(
            result["state_records_sha256"],
            self.runner.canonical_sha256(result["state_records"]),
        )

    def test_run_one_finalizes_raw_child_and_retains_terminal_timeout(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            root = Path(temporary)
            dataset_root = root / "dataset"
            output = root / "output"
            package = write_package(dataset_root, "rec_child", VALID_URDF)
            row = source_row(package)
            item = self.runner.freeze_item(
                row,
                self.runner.audit_asset(dataset_root, row),
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity=self.runner.current_runtime_identity(),
            )
            item_path = output / "inputs" / "item.json"
            self.runner.atomic_json(item_path, item)
            child_path = self.runner.child_result_path(output, item)
            child_log = (
                output / "child_logs" / f"{self.runner._job_prefix(item)}.log"
            )
            runner_hash = self.runner.sha256_file(RUNNER)
            launcher = self.runner.frozen_launcher_binding(
                self.runner.DEFAULT_CHILD_PYTHON, REPO
            )

            result = self.runner.run_one_subprocess(
                item_path,
                item,
                dataset_root,
                child_path,
                child_log,
                self.runner.CHILD_TIMEOUT_SECONDS,
                self.runner.DEFAULT_CHILD_PYTHON,
                launcher,
                runner_hash,
            )

            self.assertEqual(result["child_returncode"], 0)
            self.assertTrue(result["measurement_complete"])
            self.assertTrue(
                self.runner._valid_terminal_child(child_path, item, runner_hash)
            )
            self.assertTrue(
                self.runner._valid_cached_child(child_path, item, runner_hash)
            )

            timeout_result = self.runner.failure_record(
                item, "child_timeout", timed_out=True
            )
            timeout_result.update(
                {
                    "child_returncode": -9,
                    "child_timed_out": True,
                    "child_log": str(child_log),
                    "cache_reused": False,
                }
            )
            self.runner.atomic_json(child_path, timeout_result)
            self.assertTrue(
                self.runner._valid_terminal_child(child_path, item, runner_hash)
            )
            self.assertFalse(
                self.runner._valid_cached_child(child_path, item, runner_hash)
            )
            replay = self.runner._replay_frozen_measurements(
                {"sample_size": 1, "items": [item]}, output, dataset_root
            )
            self.assertEqual(replay[0]["replay_mode"], "canonical_terminal_failure")

    def test_fail_closed_summary_keeps_all_unexecuted_configurations(self) -> None:
        rows = []
        items = []
        for order in range(2):
            row = {
                "asset_id": f"rec_failure_{order}",
                "selection_index": order,
                "package": f"/fixture/rec_failure_{order}",
                "package_relpath": f"released_urdf/rec_failure_{order}",
                "model_urdf_sha256": str(order) * 64,
                "package_binding": {
                    "file_count": 0,
                    "total_bytes": 0,
                    "files": [],
                    "content_manifest_sha256": self.runner.canonical_sha256([]),
                },
                "package_content_manifest_sha256": self.runner.canonical_sha256([]),
                "source_record_sha256": self.runner.canonical_sha256({"order": order}),
            }
            audit = self.runner._empty_audit()
            audit.update({"movable_dof_count": 1, "range_evaluable_dof_count": 1})
            item = self.runner.freeze_item(
                row,
                audit,
                order=order,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity={"fixture": True},
            )
            items.append(item)
            rows.append(self.runner.failure_record(item, "fixture_failure"))
        manifest = {
            "protocol_id": self.runner.QUALIFICATION_PROTOCOL_ID,
            "cohort_label": "Articraft-10K qualification N=2",
            "sample_size": 2,
            "items": items,
        }
        rows[0]["max_penetration_normalized"] = 999.0
        rows[0]["measurement_complete"] = False
        rows[1]["max_penetration_normalized"] = 0.25
        rows[1]["measurement_complete"] = True

        summary = self.runner.summarize_records(manifest, rows)

        self.assertEqual(summary["cohort"]["selected"], 2)
        self.assertEqual(summary["cohort"]["category_count"], 0)
        self.assertEqual(summary["category_macro"], {"status": "N/E", "reason": "no authoritative category labels"})
        self.assertEqual(summary["metrics"]["collision_state_rate"]["denominator"], 172)
        self.assertEqual(summary["metrics"]["collision_state_rate"]["collision_states"], 172)
        self.assertEqual(summary["metrics"]["collision_state_rate"]["unexecuted_states"], 172)
        self.assertEqual(
            summary["metrics"]["max_penetration"]["normalization"],
            "PyBullet q=0 collision-shape union AABB diagonal",
        )
        self.assertEqual(
            summary["metrics"]["max_penetration"]["maximum_observed_normalized"],
            0.25,
        )
        self.assertEqual(
            summary["metrics"]["max_penetration"]["observed_assets"], 1
        )

    def test_qualification_manifest_binds_real_table2_prefix_and_source(self) -> None:
        runtime = self.runner.current_runtime_identity()

        manifest = self.runner.build_manifest(
            DATASET_ROOT,
            SOURCE_MANIFEST,
            sample_size=3,
            qualification_smoke=True,
            child_runtime=runtime,
            workers=2,
        )

        source = json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(manifest["sample_size"], 3)
        self.assertEqual(manifest["release_asset_count"], 9996)
        self.assertEqual(
            [item["asset_id"] for item in manifest["items"]],
            [row["asset_id"] for row in source["records"][:3]],
        )
        self.assertEqual(
            manifest["source"]["table2_manifest_sha256"],
            "13c47e2b2affadb951a01cab826bae139852fca5769e99ec081cc916ffa6373d",
        )
        self.assertEqual(
            manifest["source"]["protocol_document_sha256_at_freeze"],
            "be3813e1b40b4fb8e2ee5cf9bec89aa3b83d7dcca3050a0c6c3eeb3097c36ed1",
        )
        self.assertEqual(
            manifest["execution_policy"], self.runner._execution_policy()
        )
        self.assertEqual(
            manifest["runtime"]["collision_core_sha256"],
            self.runner.EXPECTED_CORE_SHA256,
        )
        self.assertEqual(
            manifest["runtime"]["child_launcher"],
            self.runner.frozen_launcher_binding(
                self.runner.DEFAULT_CHILD_PYTHON, REPO
            ),
        )
        self.assertEqual(manifest["items_sha256"], self.runner.canonical_sha256(manifest["items"]))

        rebound_source = copy.deepcopy(manifest)
        rebound_source["source"]["revision"] = "0" * 40
        rebound_source["manifest_content_sha256"] = self.runner.manifest_self_hash(
            rebound_source
        )
        with self.assertRaisesRegex(RuntimeError, "Table 2 source closure mismatch"):
            self.runner.validate_manifest(
                rebound_source,
                DATASET_ROOT,
                SOURCE_MANIFEST,
                qualification_smoke=True,
                child_runtime=runtime,
            )

        rebound_item = copy.deepcopy(manifest)
        rebound_item["items"][0]["package_relpath"] = "released_urdf/rec_other"
        rebound_item["items"][0]["input_identity_sha256"] = (
            self.runner._input_identity_sha256(rebound_item["items"][0])
        )
        rebound_item["items_sha256"] = self.runner.canonical_sha256(
            rebound_item["items"]
        )
        rebound_item["manifest_content_sha256"] = self.runner.manifest_self_hash(
            rebound_item
        )
        with self.assertRaisesRegex(RuntimeError, "source identity mismatch"):
            self.runner.validate_manifest(
                rebound_item,
                DATASET_ROOT,
                SOURCE_MANIFEST,
                qualification_smoke=True,
                child_runtime=runtime,
            )

        semantic_tamper = copy.deepcopy(manifest)
        semantic_tamper["collision_policy"]["continuous_collision_detection"] = "run"
        semantic_tamper["manifest_content_sha256"] = self.runner.manifest_self_hash(
            semantic_tamper
        )
        with self.assertRaisesRegex(RuntimeError, "collision policy mismatch"):
            self.runner._validate_manifest_semantics(
                semantic_tamper, qualification_smoke=True
            )

        cohort_tamper = copy.deepcopy(manifest)
        cohort_tamper["cohort_boundary"]["is_full_release_cohort"] = True
        cohort_tamper["manifest_content_sha256"] = self.runner.manifest_self_hash(
            cohort_tamper
        )
        with self.assertRaisesRegex(RuntimeError, "cohort boundary mismatch"):
            self.runner._validate_manifest_semantics(
                cohort_tamper, qualification_smoke=True
            )

        execution_tamper = copy.deepcopy(manifest)
        execution_tamper["execution_policy"]["child_timeout_seconds"] = 1
        execution_tamper["manifest_content_sha256"] = self.runner.manifest_self_hash(
            execution_tamper
        )
        with self.assertRaisesRegex(RuntimeError, "execution policy mismatch"):
            self.runner.validate_manifest(
                execution_tamper,
                DATASET_ROOT,
                SOURCE_MANIFEST,
                qualification_smoke=True,
                child_runtime=runtime,
            )

        launcher_tamper = copy.deepcopy(manifest)
        launcher_tamper["runtime"]["child_launcher"][
            "resolved_executable_sha256"
        ] = "0" * 64
        launcher_tamper["manifest_content_sha256"] = self.runner.manifest_self_hash(
            launcher_tamper
        )
        with self.assertRaisesRegex(RuntimeError, "launcher binding"):
            self.runner.validate_manifest(
                launcher_tamper,
                DATASET_ROOT,
                SOURCE_MANIFEST,
                qualification_smoke=True,
                child_runtime=runtime,
            )

        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            output = Path(temporary)
            self.runner.write_protocol_document_snapshot(output, manifest)
            self.runner.validate_protocol_document_snapshot(output, manifest)
            (output / self.runner.PROTOCOL_DOCUMENT_SNAPSHOT).write_text(
                "tampered\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(RuntimeError, "protocol document snapshot"):
                self.runner.validate_protocol_document_snapshot(output, manifest)

    def test_formal_static_denominators_are_frozen_invariants(self) -> None:
        expected = dict(self.runner.EXPECTED_FORMAL_AUDIT_SUMMARY)
        self.runner._validate_formal_audit_summary(
            expected, qualification_smoke=False
        )
        wrong = dict(expected)
        wrong["retained_package_failures"] -= 1
        with self.assertRaisesRegex(RuntimeError, "formal static audit invariant"):
            self.runner._validate_formal_audit_summary(
                wrong, qualification_smoke=False
            )
        self.runner._validate_formal_audit_summary(
            wrong, qualification_smoke=True
        )

    def test_state_closure_rejects_articraft_source_identity_drift(self) -> None:
        row = {
            "asset_id": "rec_state",
            "selection_index": 0,
            "package": "/fixture/rec_state",
            "package_relpath": "released_urdf/rec_state",
            "model_urdf_sha256": "1" * 64,
            "package_binding": {"file_count": 0, "total_bytes": 0, "files": [], "content_manifest_sha256": canonical_sha256([])},
            "package_content_manifest_sha256": canonical_sha256([]),
            "source_record_sha256": canonical_sha256({"asset_id": "rec_state"}),
        }
        audit = self.runner._empty_audit()
        audit["object_bbox_diagonal_m"] = 2.0
        item = self.runner.freeze_item(
            row,
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
        for field in self.runner.IDENTITY_FIELDS:
            with self.subTest(field=field):
                value = state[field]
                replacement = value + 1 if isinstance(value, int) else f"{value}_other"
                mutated = [{**state, field: replacement}]
                with self.assertRaisesRegex(RuntimeError, "source identity mismatch"):
                    self.runner.validate_state_closure(record, mutated, item)

        impossible_penetration = copy.deepcopy(state)
        impossible_penetration["all_pair_max_penetration_m"] = 0.5
        impossible_penetration["metric_max_penetration_m"] = 0.5
        impossible_record = copy.deepcopy(record)
        impossible_record["max_penetration_m"] = 0.5
        impossible_record["max_penetration_normalized"] = 0.25
        impossible_record["state_records_sha256"] = self.runner.canonical_sha256(
            [impossible_penetration]
        )
        with self.assertRaisesRegex(RuntimeError, "penetration threshold"):
            self.runner.validate_state_closure(
                impossible_record, [impossible_penetration], item
            )

        impossible_readback = copy.deepcopy(state)
        impossible_readback["reset_readback_max_abs_error"] = 0.1
        readback_record = copy.deepcopy(record)
        readback_record["max_reset_readback_error"] = 0.1
        readback_record["state_records_sha256"] = self.runner.canonical_sha256(
            [impossible_readback]
        )
        with self.assertRaisesRegex(RuntimeError, "reset readback"):
            self.runner.validate_state_closure(
                readback_record, [impossible_readback], item
            )

    def test_cached_child_requires_exact_frozen_and_runtime_identity(self) -> None:
        row = {
            "asset_id": "rec_cache",
            "selection_index": 0,
            "package": "/fixture/rec_cache",
            "package_relpath": "released_urdf/rec_cache",
            "model_urdf_sha256": "1" * 64,
            "package_binding": {
                "file_count": 0,
                "total_bytes": 0,
                "files": [],
                "content_manifest_sha256": canonical_sha256([]),
            },
            "package_content_manifest_sha256": canonical_sha256([]),
            "source_record_sha256": canonical_sha256({"asset_id": "rec_cache"}),
        }
        item = self.runner.freeze_item(
            row,
            self.runner._empty_audit(),
            order=0,
            protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
            runtime_identity=self.runner.current_runtime_identity(),
        )
        runner_hash = self.runner.sha256_file(RUNNER)
        result = self.runner.failure_record(item, "fixture_failure")
        forged_failure_metric = copy.deepcopy(result)
        forged_failure_metric["max_penetration_normalized"] = 123.456
        self.assertFalse(
            self.runner.result_matches_item(
                forged_failure_metric, item, runner_hash
            )
        )
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            output = Path(temporary)
            path = self.runner.child_result_path(output, item)
            self.runner.atomic_json(path, result)
            self.assertFalse(self.runner._valid_cached_child(path, item, runner_hash))

            result.update(
                {
                    "child_returncode": 0,
                    "child_timed_out": False,
                    "child_log": str(
                        output
                        / "child_logs"
                        / f"{self.runner._job_prefix(item)}.log"
                    ),
                    "cache_reused": False,
                }
            )
            self.runner.atomic_json(path, result)
            self.assertTrue(self.runner._valid_cached_child(path, item, runner_hash))

            invalid_raw = self.runner.failure_record(
                item, "child_invalid_result_returncode_0"
            )
            invalid_raw.update(
                {
                    "child_returncode": 0,
                    "child_timed_out": False,
                    "child_log": result["child_log"],
                    "cache_reused": False,
                }
            )
            self.runner.atomic_json(path, invalid_raw)
            self.assertTrue(
                self.runner._valid_terminal_child(path, item, runner_hash)
            )
            self.assertFalse(
                self.runner._valid_cached_child(path, item, runner_hash)
            )

            stale_source = copy.deepcopy(result)
            stale_source["package_relpath"] = "released_urdf/rec_other"
            self.runner.atomic_json(path, stale_source)
            self.assertFalse(self.runner._valid_cached_child(path, item, runner_hash))

            stale_runtime = copy.deepcopy(result)
            stale_runtime["runtime_identity"] = {"fixture": "wrong"}
            self.runner.atomic_json(path, stale_runtime)
            self.assertFalse(self.runner._valid_cached_child(path, item, runner_hash))

            self.runner.atomic_json(path, [])
            self.assertFalse(self.runner._valid_cached_child(path, item, runner_hash))

    def test_collision_core_executes_hashed_source_without_pyc_cache(self) -> None:
        core = self.runner._load_core()

        self.assertIsNone(getattr(core, "__cached__", None))
        self.assertEqual(
            core.__source_sha256__, self.runner.EXPECTED_CORE_SHA256
        )

    def test_child_launcher_is_frozen_to_exact_venv_entrypoint(self) -> None:
        binding = self.runner.frozen_launcher_binding(
            self.runner.DEFAULT_CHILD_PYTHON, REPO
        )

        self.assertEqual(
            binding["launch_path"], str(self.runner.DEFAULT_CHILD_PYTHON)
        )
        self.assertEqual(binding["symlink_target"], "/usr/bin/python3.12")
        self.assertEqual(
            binding["resolved_executable_sha256"],
            "c0736aec631466e7bc4f5541b67358543193b8922ec3b63f6c1b247d70716591",
        )
        self.assertEqual(
            binding["pyvenv_cfg_sha256"],
            "a0151eba26bcc62dbba81f521d095126e9c3ab83db1471f592c4e14961b6341f",
        )
        self.runner.validate_frozen_launcher_binding(binding)

        tampered = copy.deepcopy(binding)
        tampered["resolved_executable_sha256"] = "0" * 64
        with self.assertRaisesRegex(RuntimeError, "launcher binding"):
            self.runner.validate_frozen_launcher_binding(tampered)

        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            shim = Path(temporary) / "python"
            shim.symlink_to("/usr/bin/python3.12")
            with self.assertRaisesRegex(RuntimeError, "launcher path"):
                self.runner.frozen_launcher_binding(shim, REPO)

    def test_verifier_replays_every_frozen_child_measurement_exactly(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            root = Path(temporary)
            dataset_root = root / "dataset"
            output = root / "output"
            package = write_package(dataset_root, "rec_replay", OVERLAPPING_URDF)
            row = source_row(package)
            audit = self.runner.audit_asset(dataset_root, row)
            item = self.runner.freeze_item(
                row,
                audit,
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity=self.runner.current_runtime_identity(),
            )
            failed_package = write_package(
                dataset_root, "rec_replay_failed", VISUAL_ONLY_URDF
            )
            failed_row = source_row(failed_package, selection_index=1)
            failed_item = self.runner.freeze_item(
                failed_row,
                self.runner.audit_asset(dataset_root, failed_row),
                order=1,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity=self.runner.current_runtime_identity(),
            )
            manifest = {
                "runtime": {"runner_sha256": self.runner.sha256_file(RUNNER)},
                "sample_size": 2,
                "items": [item, failed_item],
            }
            result = self.runner.evaluate_asset(item, dataset_root)
            result.update(
                {
                    "child_returncode": 0,
                    "child_timed_out": False,
                    "child_log": str(
                        output
                        / "child_logs"
                        / f"{self.runner._job_prefix(item)}.log"
                    ),
                    "cache_reused": False,
                }
            )
            child_path = self.runner.child_result_path(output, item)
            self.runner.atomic_json(child_path, result)
            failed_result = self.runner.evaluate_asset(failed_item, dataset_root)
            failed_result.update(
                {
                    "child_returncode": 0,
                    "child_timed_out": False,
                    "child_log": str(
                        output
                        / "child_logs"
                        / f"{self.runner._job_prefix(failed_item)}.log"
                    ),
                    "cache_reused": False,
                }
            )
            self.runner.atomic_json(
                self.runner.child_result_path(output, failed_item), failed_result
            )

            bindings = self.runner._replay_frozen_measurements(
                manifest, output, dataset_root
            )

            self.assertEqual(len(bindings), 2)
            self.assertEqual(
                [binding["dataset_id"] for binding in bindings],
                [item["dataset_id"], failed_item["dataset_id"]],
            )
            forged = copy.deepcopy(result)
            self.assertTrue(
                any(
                    state["all_pair_illegal_penetration_count"] > 0
                    for state in forged["state_records"]
                )
            )
            for state in forged["state_records"]:
                for prefix in ("all_pair", "non_adjacent"):
                    state[f"{prefix}_contact_count"] = 0
                    state[f"{prefix}_illegal_penetration_count"] = 0
                    state[f"{prefix}_max_penetration_m"] = 0.0
                state["metric_max_penetration_m"] = 0.0
                state["reset_readback_max_abs_error"] = 0.0
            forged.update(
                {
                    "rest_all_pair_cf": True,
                    "rest_non_adjacent_cf": True,
                    "rest_non_adjacent_free": forged["rest_state_executed"],
                    "single_non_adjacent_free": forged["single_state_executed"],
                    "joint_single_sweep_cf_passed": forged[
                        "range_evaluable_dof_count"
                    ],
                    "single_joint_sweep_cf": True,
                    "sobol_non_adjacent_free": forged["sobol_state_executed"],
                    "multi_joint_sobol_cf": True,
                    "strict_collision_pass": True,
                    "max_penetration_m": 0.0,
                    "max_penetration_normalized": 0.0,
                    "max_reset_readback_error": 0.0,
                    "state_records_sha256": self.runner.canonical_sha256(
                        forged["state_records"]
                    ),
                }
            )
            self.assertTrue(
                self.runner.result_matches_item(
                    forged,
                    item,
                    manifest["runtime"]["runner_sha256"],
                    forged["state_records"],
                )
            )
            self.runner.atomic_json(child_path, forged)
            with self.assertRaisesRegex(RuntimeError, "measurement replay mismatch"):
                self.runner._replay_frozen_measurements(
                    manifest, output, dataset_root
                )

    def test_n1_fail_closed_artifacts_verify_and_report_tamper_fails(self) -> None:
        runtime = self.runner.current_runtime_identity()
        manifest = self.runner.build_manifest(
            DATASET_ROOT,
            SOURCE_MANIFEST,
            sample_size=1,
            qualification_smoke=True,
            child_runtime=runtime,
            workers=1,
        )
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            output = Path(temporary)
            self.runner.atomic_json(output / "frozen_manifest.json", manifest)
            self.runner.write_protocol_document_snapshot(output, manifest)
            self.runner.atomic_json(output / "child_runtime_probe.json", runtime)
            item = manifest["items"][0]
            result = self.runner.evaluate_asset(item, DATASET_ROOT)
            result.update(
                {
                    "child_returncode": 0,
                    "child_timed_out": False,
                    "child_log": str(
                        output
                        / "child_logs"
                        / f"{self.runner._job_prefix(item)}.log"
                    ),
                    "cache_reused": False,
                }
            )
            self.runner.atomic_json(
                self.runner.child_result_path(output, item), result
            )
            self.runner.run_pair_policy_smoke(output)

            summary = self.runner.summarize(manifest, output)
            receipt = self.runner.verify(manifest, output)

            self.assertEqual(receipt["status"], "PASS")
            self.assertTrue(receipt["qualification_smoke"])
            self.assertFalse(receipt["formal_evaluation"])
            self.assertEqual(receipt["sample_size"], 1)
            self.assertEqual(
                receipt["evaluated_protocol_id"],
                self.runner.QUALIFICATION_PROTOCOL_ID,
            )
            self.assertEqual(
                receipt["manifest_content_sha256"],
                manifest["manifest_content_sha256"],
            )
            self.assertEqual(
                receipt["experiment_status"], "COMPLETE_WITH_RETAINED_FAILURES"
            )
            self.assertEqual(receipt["retained_failure_assets"], 1)
            self.assertEqual(
                receipt["unexecuted_states"],
                summary["metrics"]["collision_state_rate"]["denominator"],
            )
            self.assertEqual(
                summary["metrics"]["collision_state_rate"]["unexecuted_states"],
                summary["metrics"]["collision_state_rate"]["denominator"],
            )

            forged_records = self.runner.read_json(output / "asset_records.json")
            forged_records[0]["issues"] = ["forged_aggregate_issue"]
            self.runner.atomic_json(output / "asset_records.json", forged_records)
            forged_summary = self.runner.summarize_records(manifest, forged_records)
            self.runner.atomic_json(output / "summary.json", forged_summary)
            self.runner.render_report(forged_summary, output)
            with self.assertRaisesRegex(RuntimeError, "verification failed"):
                self.runner.verify(manifest, output)
            failed = self.runner.read_json(output / "verification.json")
            self.assertFalse(
                failed["checks"]["aggregates_match_authoritative_children"]
            )
            summary = self.runner.summarize(manifest, output)

            (output / "report.md").write_text("# tampered\n", encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "verification failed"):
                self.runner.verify(manifest, output)
            failed = self.runner.read_json(output / "verification.json")
            self.assertEqual(failed["status"], "FAIL")
            self.assertFalse(failed["checks"]["report_recomputes_exactly"])

            self.runner.atomic_text(output / "report.md", self.runner.report_text(summary))
            tampered_summary = copy.deepcopy(summary)
            tampered_summary["metrics"]["rest_all_pair_cf"]["passed"] = 1
            self.runner.atomic_json(output / "summary.json", tampered_summary)
            with self.assertRaisesRegex(RuntimeError, "verification failed"):
                self.runner.verify(manifest, output)
            failed = self.runner.read_json(output / "verification.json")
            self.assertFalse(failed["checks"]["summary_recomputes_exactly"])
            self.assertTrue(failed["checks"]["report_recomputes_exactly"])

            self.runner.atomic_json(output / "summary.json", summary)
            pair_receipt = self.runner.read_json(output / "pair_policy_smoke.json")
            pair_receipt["non_adjacent_illegal_penetration_count"] = 1
            self.runner.atomic_json(output / "pair_policy_smoke.json", pair_receipt)
            with self.assertRaisesRegex(RuntimeError, "verification failed"):
                self.runner.verify(manifest, output)
            failed = self.runner.read_json(output / "verification.json")
            self.assertFalse(failed["checks"]["pair_policy_smoke_semantics_pass"])

            forged_pair_receipt = {
                "protocol_id": "urdf_table4_pybullet_pair_policy_smoke_v1",
                "status": "PASS",
                "pybullet_api_version": runtime["pybullet_api_version"],
                "all_pair_illegal_penetration_count": 1,
                "non_adjacent_illegal_penetration_count": 0,
            }
            self.runner.atomic_json(
                output / "pair_policy_smoke.json", forged_pair_receipt
            )
            with self.assertRaisesRegex(RuntimeError, "verification failed"):
                self.runner.verify(manifest, output)
            failed = self.runner.read_json(output / "verification.json")
            self.assertFalse(
                failed["checks"]["pair_policy_smoke_reproduces_exactly"]
            )

            self.runner.atomic_json(
                output / "verification.json", {"status": "PASS", "marker": "stale"}
            )
            self.runner.atomic_json(output / "asset_records.json", [None])
            malformed_manifest = copy.deepcopy(manifest)
            malformed_manifest["items"] = [None]
            malformed_manifest["items_sha256"] = self.runner.canonical_sha256(
                malformed_manifest["items"]
            )
            malformed_manifest["manifest_content_sha256"] = (
                self.runner.manifest_self_hash(malformed_manifest)
            )
            with self.assertRaisesRegex(RuntimeError, "verification aborted"):
                self.runner.verify(malformed_manifest, output)
            fatal = self.runner.read_json(output / "verification.json")
            self.assertEqual(fatal["status"], "FAIL")
            self.assertNotIn("marker", fatal)
            self.assertIn("fatal_verification_error", fatal["errors"])

    def test_report_names_articraft_and_preserves_claim_boundaries(self) -> None:
        summary = {
            "status": "COMPLETE_WITH_RETAINED_FAILURES",
            "cohort": {"label": "Articraft-10K fixed N=800 cohort"},
            "metrics": {
                "rest_all_pair_cf": {"passed": 1, "denominator": 3, "rate": 1 / 3},
                "rest_non_adjacent_cf": {"passed": 2, "denominator": 3, "rate": 2 / 3},
                "single_joint_sweep_cf": {"passed": 1, "denominator": 3, "rate": 1 / 3},
                "multi_joint_sobol_cf": {"passed": 1, "denominator": 3, "rate": 1 / 3},
                "collision_state_rate": {"collision_states": 4, "denominator": 10, "rate": 0.4},
                "aor": {"status": "N/E"},
                "max_penetration": {
                    "maximum_observed_normalized": 0.25,
                    "fully_measured_assets": 2,
                    "observed_assets": 2,
                    "denominator": 3,
                    "status": "PARTIAL",
                },
                "collision_free_range": {"passed_states": 5, "denominator": 9, "rate": 5 / 9},
                "strict_collision_pass": {"passed": 1, "denominator": 3, "rate": 1 / 3},
            },
        }

        report = self.runner.report_text(summary)

        self.assertIn("Articraft-10K fixed N=800 cohort", report)
        self.assertIn("9,996-asset release universe", report)
        self.assertIn("AOR | N/E", report)
        self.assertIn("Category macro average: N/E", report)
        self.assertNotIn("Artiverse", report)
        self.assertNotIn("PartNet", report)

    def test_verify_cli_overwrites_stale_pass_when_manifest_is_malformed(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            output = Path(temporary)
            (output / "frozen_manifest.json").write_text(
                "{malformed\n", encoding="utf-8"
            )
            self.runner.atomic_json(
                output / "verification.json", {"status": "PASS", "marker": "stale"}
            )
            completed = subprocess.run(
                [
                    str(REPO / "exp/.venv_low_medium/bin/python"),
                    str(RUNNER),
                    "--phase",
                    "verify",
                    "--dataset-root",
                    str(DATASET_ROOT),
                    "--table2-manifest",
                    str(SOURCE_MANIFEST),
                    "--output",
                    str(output),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
                timeout=120,
            )
            receipt = self.runner.read_json(output / "verification.json")

            alternate_launcher = output / "python"
            alternate_launcher.symlink_to("/usr/bin/python3.12")
            self.runner.atomic_json(
                output / "verification.json", {"status": "PASS", "marker": "stale"}
            )
            shim_completed = subprocess.run(
                [
                    str(REPO / "exp/.venv_low_medium/bin/python"),
                    str(RUNNER),
                    "--phase",
                    "verify",
                    "--python",
                    str(alternate_launcher),
                    "--dataset-root",
                    str(DATASET_ROOT),
                    "--table2-manifest",
                    str(SOURCE_MANIFEST),
                    "--output",
                    str(output),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
                timeout=120,
            )
            shim_receipt = self.runner.read_json(output / "verification.json")

        self.assertNotEqual(completed.returncode, 0)
        self.assertEqual(receipt["status"], "FAIL")
        self.assertNotIn("marker", receipt)
        self.assertIn("fatal_verification_error", receipt["errors"])
        self.assertNotEqual(shim_completed.returncode, 0)
        self.assertEqual(shim_receipt["status"], "FAIL")
        self.assertNotIn("marker", shim_receipt)
        self.assertIn("launcher path", shim_receipt["errors"]["fatal_verification_error"])


if __name__ == "__main__":
    unittest.main()
