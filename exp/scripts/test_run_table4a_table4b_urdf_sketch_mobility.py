#!/usr/bin/env python3
"""Contract tests for SketchMobility Table 4a and Table 4b adapters."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[2]
TABLE4A = REPO / "exp/scripts/run_table4a_urdf_sketch_mobility.py"
TABLE4B = REPO / "exp/scripts/run_table4b_urdf_sketch_mobility.py"
VERIFY4A = REPO / "exp/scripts/verify_table4a_urdf_sketch_mobility.py"
VERIFY4B = REPO / "exp/scripts/verify_table4b_urdf_sketch_mobility.py"


def load_module(path: Path, name: str):
    if not path.is_file():
        raise AssertionError(f"required module is missing: {path}")
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"cannot load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class SketchMobilityTable4AdaptersTests(unittest.TestCase):
    def test_receipt_closure_rejects_evidence_tampering(self) -> None:
        adapter = load_module(TABLE4A, "table4a_sketch_closure_fixture")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            evidence = root / "asset_records.jsonl"
            evidence.write_text('{"asset_id":"before"}\n', encoding="utf-8")
            adapter.common.write_receipt_closure(
                root, adapter.base.atomic_write_json
            )
            self.assertTrue(adapter.common.validate_receipt_closure(root))
            evidence.write_text('{"asset_id":"after"}\n', encoding="utf-8")
            self.assertFalse(adapter.common.validate_receipt_closure(root))

    def test_table4a_preserves_full_denominators_and_311_asset_gate(self) -> None:
        adapter = load_module(TABLE4A, "table4a_sketch_test_target")

        manifest = adapter.load_source_manifest()
        table3, joint_count = adapter.load_table3_joint_pass()
        jobs = adapter.build_jobs(
            manifest, table3, adapter.load_table4_state_hashes()
        )

        self.assertEqual(800, len(jobs))
        self.assertEqual(1824, joint_count)
        self.assertEqual(1824, sum(job["expected_movable_dof"] for job in jobs))
        self.assertEqual(311, sum(bool(job["genesis_eligible"]) for job in jobs))
        self.assertEqual(4, adapter.WORKERS)
        self.assertEqual(
            "data/Shape2Motion/Kettle/kettle_0057", jobs[0]["dataset_id"]
        )
        self.assertEqual("mobility.urdf", Path(jobs[0]["urdf_path"]).name)

        failed_job = next(job for job in jobs if not job["genesis_eligible"])
        failed = adapter.gate_failed_record(failed_job)
        self.assertEqual("error", failed["status"])
        self.assertEqual(failed_job["expected_movable_dof"], len(failed["joint_records"]))
        self.assertEqual(
            21 * failed_job["expected_movable_dof"], failed["states_intended"]
        )
        aggregate = adapter.base.aggregate(
            [adapter.gate_failed_record(job) for job in jobs],
            adapter.load_table4_strict_pass(),
        )
        self.assertEqual(1824, aggregate["joint_level_full_range_cf"]["denominator"])
        self.assertEqual(1824, aggregate["collision_safe_dof_retention"]["denominator"])

        smoke_records = [adapter.gate_failed_record(job) for job in jobs[:5]]
        smoke = adapter.base.aggregate(
            smoke_records, adapter.load_table4_strict_pass()
        )
        self.assertEqual(5, smoke["joint_level_full_range_cf"]["denominator"])
        self.assertEqual(5, smoke["collision_safe_dof_retention"]["denominator"])

    def test_table4b_preserves_exact_frozen_cohort_order_and_identity(self) -> None:
        adapter = load_module(TABLE4B, "table4b_sketch_test_target")

        manifest = adapter.load_source_manifest()
        jobs = adapter.build_jobs(manifest)

        self.assertEqual(800, len(jobs))
        self.assertEqual(
            "data/Shape2Motion/Kettle/kettle_0057", jobs[0]["asset_id"]
        )
        self.assertEqual(0, jobs[0]["selection_index"])
        self.assertEqual("mobility.urdf", Path(jobs[0]["urdf_path"]).name)
        self.assertEqual(
            "150fb5b16442ad363223d045fcddfa385d1d164851c6f37602a1c5cb64602711",
            jobs[0]["expected_urdf_sha256"],
        )
        self.assertEqual(
            "a88506e1da8e7e8b61a740965dea2faba4e9ab8280f47417e17550024b6dde17",
            adapter.EXPECTED_ORDERED_IDS_SHA256,
        )

        failed = adapter.base._failed_asset_record(jobs[0], "synthetic")
        metrics = adapter.base.aggregate([failed])
        self.assertEqual("N/E", metrics["analytic_collision_share"]["status"])
        self.assertIsNone(metrics["analytic_collision_share"]["percent"])

    def test_standalone_verifiers_reaggregate_fail_closed_records(self) -> None:
        table4a = load_module(TABLE4A, "table4a_sketch_verifier_fixture")
        verify4a = load_module(VERIFY4A, "table4a_sketch_verifier_test_target")
        verify4b = load_module(VERIFY4B, "table4b_sketch_verifier_test_target")
        manifest = table4a.load_source_manifest()
        table3, _ = table4a.load_table3_joint_pass()
        jobs = table4a.build_jobs(
            manifest, table3, table4a.load_table4_state_hashes()
        )
        records = []
        for job in jobs:
            if job["genesis_eligible"]:
                record = table4a.base._failed_asset_record(
                    job, "synthetic_eligible_failure"
                )
                record["expected_package_content_manifest_sha256"] = job[
                    "expected_package_content_manifest_sha256"
                ]
                record["package_content_manifest_sha256"] = job[
                    "expected_package_content_manifest_sha256"
                ]
            else:
                record = table4a.gate_failed_record(job)
            records.append(record)
        aggregates = table4a.base.aggregate(
            records, table4a.load_table4_strict_pass()
        )

        result = verify4a.verify_records(records, aggregates)

        self.assertEqual("PASS", result["status"])
        self.assertTrue(all(result["checks"].values()))
        rejected = verify4b.verify_records([], {})
        self.assertEqual("FAIL", rejected["status"])
        self.assertFalse(rejected["checks"]["formal_record_count"])


if __name__ == "__main__":
    unittest.main()
