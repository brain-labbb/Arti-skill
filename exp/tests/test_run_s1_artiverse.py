from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_s1_artiverse.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("s1_artiverse_runner_under_test", RUNNER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def process_id(_: int) -> int:
    return os.getpid()


class S1ArtiverseRunnerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.runner = load_runner()

    def test_aggregate_no_published_evidence_is_fail_closed(self) -> None:
        records = [
            {
                "asset_id": "asset-a",
                "status": "completed",
                "strict_pass_no_method_allowance": True,
                "s1_evidence": {
                    "receipt": {"receipt_bound_asset": 0},
                    "receipt_replay": {"passed": None, "status": "NOT_RUN_NO_VALID_RECEIPT"},
                    "rebuild": {"eligible_asset": 0, "status": "N/E"},
                    "allowance": {
                        "status": "COMPLETE",
                        "registered_excluded_pair_count": 0,
                        "eligible_nonadjacent_pair_count": 5,
                    },
                },
            },
            {
                "asset_id": "asset-b",
                "status": "completed",
                "strict_pass_no_method_allowance": False,
                "s1_evidence": {
                    "receipt": {"receipt_bound_asset": 0},
                    "receipt_replay": {"passed": None, "status": "NOT_RUN_NO_VALID_RECEIPT"},
                    "rebuild": {"eligible_asset": 0, "status": "N/E"},
                    "allowance": {
                        "status": "COMPLETE",
                        "registered_excluded_pair_count": 0,
                        "eligible_nonadjacent_pair_count": 1,
                    },
                },
            },
        ]

        summary = self.runner.aggregate(records, intended_assets=2)
        metrics = summary["metrics"]

        self.assertEqual(metrics["receipt_bound_assets"], {
            "passed": 0, "denominator": 2, "rate": 0.0, "percentage": 0.0,
        })
        self.assertEqual(metrics["receipt_replay_pass"], {
            "passed": 0, "denominator": 2, "rate": 0.0, "percentage": 0.0,
        })
        self.assertEqual(metrics["deterministic_rebuild_match"]["status"], "N/E")
        self.assertEqual(metrics["deterministic_rebuild_match"]["eligible_assets"], 0)
        self.assertEqual(metrics["deterministic_rebuild_match"]["asset_denominator"], 2)
        self.assertEqual(metrics["allowance_density"]["registered_pairs"], 0)
        self.assertEqual(metrics["allowance_density"]["eligible_pairs"], 6)
        self.assertEqual(metrics["allowance_density"]["percentage"], 0.0)
        self.assertEqual(metrics["strict_pass_no_method_allowance"]["passed"], 1)
        self.assertEqual(metrics["strict_pass_no_method_allowance"]["denominator"], 2)
        self.assertEqual(metrics["registered_allowance_gain_pp"]["status"], "COMPLETE")
        self.assertEqual(metrics["registered_allowance_gain_pp"]["value"], 0.0)

    def test_parallel_map_executes_work_in_multiple_processes(self) -> None:
        worker_pids = self.runner.multiprocessing_map(process_id, list(range(32)), workers=2)

        self.assertGreaterEqual(len(set(worker_pids)), 2)
        self.assertNotIn(os.getpid(), worker_pids)

    def test_allowance_pair_policy_excludes_reversed_parent_child_pair(self) -> None:
        root = ET.fromstring("""
        <robot name="fixture">
          <link name="z"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
          <link name="a"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
          <link name="m"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
          <joint name="za" type="fixed"><parent link="z"/><child link="a"/></joint>
          <joint name="am" type="fixed"><parent link="a"/><child link="m"/></joint>
        </robot>
        """)

        eligible, issues = self.runner.static._eligible_nonadjacent_pairs(root)

        self.assertEqual(issues, [])
        self.assertEqual(eligible, {("m", "z")})

    def test_allowance_pair_policy_ignores_links_without_collision(self) -> None:
        root = ET.fromstring("""
        <robot name="fixture">
          <link name="base"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
          <link name="door"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
          <link name="marker"/>
          <joint name="hinge" type="fixed"><parent link="base"/><child link="door"/></joint>
          <joint name="tag" type="fixed"><parent link="door"/><child link="marker"/></joint>
        </robot>
        """)

        eligible, issues = self.runner.static._eligible_nonadjacent_pairs(root)

        self.assertEqual(issues, [])
        self.assertEqual(eligible, set())

    def test_live_allowance_file_is_not_treated_as_preregistered(self) -> None:
        with tempfile.TemporaryDirectory(prefix="s1_allowance_boundary_") as temporary:
            dataset_root = Path(temporary)
            package = dataset_root / "data/cat/src/a/urdf_w_collider"
            package.mkdir(parents=True)
            urdf = package / "a.urdf"
            urdf.write_text("""
            <robot name="fixture">
              <link name="base"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
              <link name="door"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
              <link name="handle"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
              <joint name="hinge" type="fixed"><parent link="base"/><child link="door"/></joint>
              <joint name="mount" type="fixed"><parent link="door"/><child link="handle"/></joint>
            </robot>
            """, encoding="utf-8")
            (package / "allowance.json").write_text(json.dumps({
                "excluded_non_adjacent_pairs": [["base", "handle"]],
            }), encoding="utf-8")
            item = {
                "selection_index": 0,
                "asset_id": "data/cat/src/a",
                "manifest_root": "data/cat/src/a",
                "dataset_id": "artiverse_0000",
                "model_id": "a",
                "raw_category": "cat",
                "source": "src",
                "selection_rank": 1,
                "package": package.as_posix(),
                "primary_urdf_relative_path": "a.urdf",
                "urdf_sha256_expected": self.runner.sha256_file(urdf),
                "collision_mesh_files_expected": [],
                "table4_input_identity_sha256": "1" * 64,
                "strict_pass_no_method_allowance": False,
            }
            item["s1_input_identity_sha256"] = self.runner.canonical_sha256(item)

            record = self.runner.evaluate_asset(item, dataset_root=dataset_root)

            (package / "allowance.json").write_text("{malformed", encoding="utf-8")
            malformed_record = self.runner.evaluate_asset(item, dataset_root=dataset_root)

        allowance = record["s1_evidence"]["allowance"]
        self.assertEqual(allowance["candidate_file_count"], 1)
        self.assertEqual(allowance["discovered_unregistered_pair_count"], 1)
        self.assertEqual(allowance["registered_excluded_pair_count"], 0)
        self.assertEqual(record["registered_allowance_strict_pass"], False)
        malformed_allowance = malformed_record["s1_evidence"]["allowance"]
        self.assertEqual(malformed_allowance["status"], "COMPLETE")
        self.assertEqual(malformed_allowance["registered_excluded_pair_count"], 0)
        self.assertEqual(malformed_allowance["eligible_nonadjacent_pair_count"], 1)
        self.assertTrue(malformed_allowance["discovery_issues"])

    def test_registered_allowance_without_replay_cannot_claim_zero_gain(self) -> None:
        records = [{
            "asset_id": "asset-a",
            "status": "completed",
            "strict_pass_no_method_allowance": False,
            "registered_allowance_strict_pass": None,
            "s1_evidence": {
                "receipt": {"receipt_bound_asset": 0},
                "receipt_replay": {"passed": None, "status": "NOT_RUN_NO_VALID_RECEIPT"},
                "rebuild": {"eligible_asset": 0, "status": "N/E"},
                "allowance": {
                    "status": "COMPLETE",
                    "registered_excluded_pair_count": 1,
                    "eligible_nonadjacent_pair_count": 3,
                },
            },
        }]

        metrics = self.runner.aggregate(records, intended_assets=1)["metrics"]

        self.assertEqual(metrics["allowance_density"]["registered_pairs"], 1)
        self.assertEqual(metrics["allowance_density"]["eligible_pairs"], 3)
        self.assertEqual(metrics["registered_allowance_gain_pp"]["status"], "NOT_EVALUABLE")
        self.assertIsNone(metrics["registered_allowance_gain_pp"]["value"])

    def test_binding_failure_cannot_inherit_historical_strict_pass(self) -> None:
        records = [{
            "asset_id": "asset-a",
            "status": "binding_failed",
            "binding": {"verified": False, "issues": ["primary_urdf_sha256_mismatch"]},
            "strict_pass_no_method_allowance": True,
            "registered_allowance_strict_pass": None,
            "s1_evidence": {
                "receipt": {"receipt_bound_asset": 0},
                "receipt_replay": {"passed": False},
                "rebuild": {"eligible_asset": 0, "status": "N/E"},
                "allowance": {
                    "status": "NOT_EVALUABLE",
                    "registered_excluded_pair_count": None,
                    "eligible_nonadjacent_pair_count": None,
                },
            },
        }]

        metrics = self.runner.aggregate(records, intended_assets=1)["metrics"]

        self.assertEqual(metrics["strict_pass_no_method_allowance"]["passed"], 0)
        self.assertEqual(metrics["allowance_density"]["measured_assets"], 0)
        self.assertEqual(metrics["allowance_density"]["intended_assets"], 1)

        rendered = self.runner.render_summary({
            "dataset": "Artiverse",
            "protocol_id": self.runner.PROTOCOL_ID,
            "n_eval": 1,
            "status_counts": {"binding_failed": 1},
            "metrics": metrics,
        })
        self.assertIn("PARTIAL", rendered)

    def test_manifest_closes_over_verification_and_environment(self) -> None:
        with tempfile.TemporaryDirectory(prefix="s1_manifest_") as temporary:
            output = Path(temporary)
            names = (
                "frozen_config.json", "environment.json", "evidence_inventory.json",
                "asset_records.jsonl", "summary.json", "summary.md",
                "protocol_snapshot.md", "verification.json",
            )
            for name in names:
                (output / name).write_text(name, encoding="utf-8")

            manifest = self.runner.build_manifest(
                output,
                classification="FORMAL",
                completed_at="2026-08-21T00:00:00Z",
                command=["run_s1_artiverse.py"],
                verification_status="PASS",
            )

        self.assertEqual(set(manifest["outputs"]), set(names))
        self.assertEqual(manifest["verifier"]["path"], str(self.runner.SCRIPT.with_name("verify_s1_artiverse.py")))
        self.assertEqual(len(manifest["verifier"]["sha256"]), 64)

    def test_freeze_cohort_preserves_table1_order_and_table4_binding(self) -> None:
        with tempfile.TemporaryDirectory(prefix="s1_artiverse_cohort_") as temporary:
            root = Path(temporary)
            table1_path = root / "table1.json"
            table4_path = root / "table4.json"
            table4_records_path = root / "table4_records.json"
            table4_verification_path = root / "verification.json"
            table1_path.write_text(json.dumps({
                "dataset": "Artiverse",
                "assets": [
                    {"manifest_root": "data/cat/src/a", "selection_rank": 7},
                    {"manifest_root": "data/cat/src/b", "selection_rank": 9},
                ],
            }), encoding="utf-8")
            table4_path.write_text(json.dumps({
                "protocol_id": "strict-v1",
                "sample_size": 2,
                "items": [
                    {
                        "asset_id": "artiverse_0000",
                        "dataset_id": "artiverse_0000",
                        "manifest_root": "data/cat/src/a",
                        "model_id": "a",
                        "raw_category": "cat",
                        "source": "src",
                        "primary_urdf_relpath": "data/cat/src/a/urdf_w_collider/a.urdf",
                        "urdf_sha256": "a" * 64,
                        "input_identity_sha256": "1" * 64,
                    },
                    {
                        "asset_id": "artiverse_0001",
                        "dataset_id": "artiverse_0001",
                        "manifest_root": "data/cat/src/b",
                        "model_id": "b",
                        "raw_category": "cat",
                        "source": "src",
                        "primary_urdf_relpath": "data/cat/src/b/urdf_w_collider/b.urdf",
                        "urdf_sha256": "b" * 64,
                        "input_identity_sha256": "2" * 64,
                    },
                ],
            }), encoding="utf-8")
            table4_records_path.write_text(json.dumps([
                {
                    "dataset_id": "artiverse_0000",
                    "manifest_root": "data/cat/src/a",
                    "input_identity_sha256": "1" * 64,
                    "order": 0,
                    "protocol_id": "strict-v1",
                    "strict_collision_pass": True,
                },
                {
                    "dataset_id": "artiverse_0001",
                    "manifest_root": "data/cat/src/b",
                    "input_identity_sha256": "2" * 64,
                    "order": 1,
                    "protocol_id": "strict-v1",
                    "strict_collision_pass": False,
                },
            ]), encoding="utf-8")
            table4_verification_path.write_text(json.dumps({
                "status": "PASS",
                "artifact_sha256": {
                    "frozen_manifest.json": self.runner.sha256_file(table4_path),
                    "asset_records.json": self.runner.sha256_file(table4_records_path),
                },
            }), encoding="utf-8")

            items, provenance = self.runner.freeze_cohort(
                table1_manifest=table1_path,
                expected_table1_sha256=self.runner.sha256_file(table1_path),
                table4_manifest=table4_path,
                expected_table4_sha256=self.runner.sha256_file(table4_path),
                table4_records=table4_records_path,
                expected_table4_records_sha256=self.runner.sha256_file(table4_records_path),
                table4_verification=table4_verification_path,
                expected_table4_verification_sha256=self.runner.sha256_file(table4_verification_path),
                dataset_root=root / "dataset",
                expected_size=2,
                limit=None,
            )

        self.assertEqual([row["manifest_root"] for row in items], [
            "data/cat/src/a", "data/cat/src/b",
        ])
        self.assertEqual([row["selection_index"] for row in items], [0, 1])
        self.assertEqual([row["strict_pass_no_method_allowance"] for row in items], [True, False])
        self.assertEqual(items[0]["package"], (root / "dataset/data/cat/src/a/urdf_w_collider").as_posix())
        self.assertEqual(provenance["strict_passed"], 1)
        self.assertEqual(provenance["n_eval"], 2)

    def test_freeze_cohort_rejects_table4_result_identity_drift(self) -> None:
        with tempfile.TemporaryDirectory(prefix="s1_artiverse_drift_") as temporary:
            root = Path(temporary)
            table1 = root / "table1.json"
            table4 = root / "frozen_manifest.json"
            records = root / "asset_records.json"
            verification = root / "verification.json"
            table1.write_text(json.dumps({
                "dataset": "Artiverse",
                "assets": [{"manifest_root": "data/cat/src/a"}],
            }), encoding="utf-8")
            table4.write_text(json.dumps({
                "protocol_id": "strict-v1",
                "sample_size": 1,
                "items": [{
                    "asset_id": "artiverse_0000",
                    "dataset_id": "artiverse_0000",
                    "manifest_root": "data/cat/src/a",
                    "model_id": "a",
                    "raw_category": "cat",
                    "source": "src",
                    "primary_urdf_relpath": "data/cat/src/a/urdf_w_collider/a.urdf",
                    "urdf_sha256": "a" * 64,
                    "input_identity_sha256": "1" * 64,
                    "order": 0,
                    "protocol_id": "strict-v1",
                }],
            }), encoding="utf-8")
            records.write_text(json.dumps([{
                "dataset_id": "artiverse_0000",
                "manifest_root": "data/cat/src/other",
                "input_identity_sha256": "1" * 64,
                "order": 0,
                "protocol_id": "strict-v1",
                "strict_collision_pass": True,
            }]), encoding="utf-8")
            verification.write_text(json.dumps({
                "status": "PASS",
                "artifact_sha256": {
                    "frozen_manifest.json": self.runner.sha256_file(table4),
                    "asset_records.json": self.runner.sha256_file(records),
                },
            }), encoding="utf-8")

            with self.assertRaisesRegex(self.runner.ProtocolViolation, "identity mismatch"):
                self.runner.freeze_cohort(
                    table1_manifest=table1,
                    expected_table1_sha256=self.runner.sha256_file(table1),
                    table4_manifest=table4,
                    expected_table4_sha256=self.runner.sha256_file(table4),
                    table4_records=records,
                    expected_table4_records_sha256=self.runner.sha256_file(records),
                    table4_verification=verification,
                    expected_table4_verification_sha256=self.runner.sha256_file(verification),
                    dataset_root=root / "dataset",
                    expected_size=1,
                    limit=None,
                )


if __name__ == "__main__":
    unittest.main()
