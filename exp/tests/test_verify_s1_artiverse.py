from __future__ import annotations

import importlib.util
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[2]
VERIFIER = REPO / "exp/scripts/verify_s1_artiverse.py"


def load_verifier():
    spec = importlib.util.spec_from_file_location("s1_artiverse_verifier_under_test", VERIFIER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def records_fixture() -> list[dict]:
    evidence_a = {
        "receipt": {"receipt_bound_asset": 0},
        "receipt_replay": {"passed": False},
        "rebuild": {"eligible_asset": 0},
        "allowance": {
            "status": "COMPLETE",
            "registered_excluded_pair_count": 0,
            "eligible_nonadjacent_pair_count": 4,
        },
    }
    evidence_b = {
        "receipt": {"receipt_bound_asset": 0},
        "receipt_replay": {"passed": False},
        "rebuild": {"eligible_asset": 0},
        "allowance": {
            "status": "COMPLETE",
            "registered_excluded_pair_count": 0,
            "eligible_nonadjacent_pair_count": 2,
        },
    }
    return [
        {
            "selection_index": 0,
            "asset_id": "a",
            "status": "completed",
            "strict_pass_no_method_allowance": True,
            "registered_allowance_strict_pass": True,
            "s1_evidence": evidence_a,
        },
        {
            "selection_index": 1,
            "asset_id": "b",
            "status": "completed",
            "strict_pass_no_method_allowance": False,
            "registered_allowance_strict_pass": False,
            "s1_evidence": evidence_b,
        },
    ]


class S1ArtiverseVerifierTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.verifier = load_verifier()

    def test_recomputes_six_metrics_from_atomic_records(self) -> None:
        metrics = self.verifier.recompute_metrics(records_fixture(), intended_assets=2)

        self.assertEqual(metrics["receipt_bound_assets"]["passed"], 0)
        self.assertEqual(metrics["receipt_replay_pass"]["passed"], 0)
        self.assertEqual(metrics["deterministic_rebuild_match"]["status"], "N/E")
        self.assertEqual(metrics["allowance_density"]["registered_pairs"], 0)
        self.assertEqual(metrics["allowance_density"]["eligible_pairs"], 6)
        self.assertEqual(metrics["strict_pass_no_method_allowance"]["passed"], 1)
        self.assertEqual(metrics["registered_allowance_gain_pp"]["value"], 0.0)

    def test_relative_and_absolute_paths_resolve_to_same_artifact(self) -> None:
        with tempfile.TemporaryDirectory(prefix="s1_same_path_", dir=REPO / "exp/runtime") as temporary:
            absolute = Path(temporary) / "artifact.json"
            absolute.write_text("{}", encoding="utf-8")
            relative = Path(os.path.relpath(absolute, REPO))

            self.assertTrue(self.verifier.same_resolved_path(absolute, REPO / relative))

    def test_summary_mismatch_fails_verification(self) -> None:
        records = records_fixture()
        expected = self.verifier.recompute_metrics(records, intended_assets=2)
        observed = {key: dict(value) for key, value in expected.items()}
        observed["strict_pass_no_method_allowance"]["passed"] = 2

        checks = self.verifier.verify_aggregates(
            records,
            {"n_eval": 2, "metrics": observed},
            expected_n=2,
        )

        self.assertFalse(checks["summary_metrics_recompute_exactly"])
        self.assertTrue(checks["record_count_matches_n_eval"])
        self.assertTrue(checks["selection_order_is_exact"])

    def test_formal_verification_rejects_empty_or_prefix_cohort(self) -> None:
        empty_metrics = self.verifier.recompute_metrics([], intended_assets=0)

        checks = self.verifier.verify_aggregates(
            [],
            {"n_eval": 0, "metrics": empty_metrics},
            expected_n=800,
        )

        self.assertFalse(checks["record_count_matches_n_eval"])

    def test_atomic_source_verification_recomputes_eligible_pairs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="s1_verify_source_") as temporary:
            dataset_root = Path(temporary)
            package = dataset_root / "data/cat/src/a/urdf_w_collider"
            package.mkdir(parents=True)
            urdf = package / "a.urdf"
            urdf.write_text("""
            <robot name="fixture">
              <link name="z"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
              <link name="a"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
              <link name="m"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
              <joint name="za" type="fixed"><parent link="z"/><child link="a"/></joint>
              <joint name="am" type="fixed"><parent link="a"/><child link="m"/></joint>
            </robot>
            """, encoding="utf-8")
            urdf_sha = hashlib.sha256(urdf.read_bytes()).hexdigest()
            identity = {
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
                "urdf_sha256_expected": urdf_sha,
                "collision_mesh_files_expected": [],
                "table4_input_identity_sha256": "1" * 64,
                "strict_pass_no_method_allowance": True,
            }
            record = {
                **identity,
                "s1_input_identity_sha256": hashlib.sha256(
                    json.dumps(identity, sort_keys=True, separators=(",", ":")).encode()
                ).hexdigest(),
                "status": "completed",
                "binding": {"verified": True, "issues": []},
                "resource_closure": {
                    "complete": True,
                    "sha256": hashlib.sha256(json.dumps(
                        [{"path": "a.urdf", "sha256": urdf_sha}],
                        sort_keys=True, separators=(",", ":"),
                    ).encode()).hexdigest(),
                    "files": [{"path": "a.urdf", "sha256": urdf_sha}],
                },
                "s1_evidence": {
                    "receipt": {"candidate_count": 0, "records": []},
                    "rebuild": {"candidate_recipe_count": 0, "recipes": []},
                    "allowance": {
                        "candidate_file_count": 0,
                        "records": [],
                        "registered_excluded_pair_count": 0,
                        "eligible_nonadjacent_pair_count": 2,
                    },
                },
            }
            frozen = {
                "dataset_id": "artiverse_0000",
                "manifest_root": "data/cat/src/a",
                "input_identity_sha256": "1" * 64,
                "order": 0,
                "protocol_id": "strict-v1",
                "model_id": "a",
                "urdf_sha256": urdf_sha,
                "collision_mesh_files": [],
            }

            checks = self.verifier.verify_atomic_record(record, frozen, dataset_root)

        self.assertTrue(checks["source_bytes_match"])
        self.assertTrue(checks["s1_input_identity_matches"])
        self.assertTrue(checks["evidence_inventory_matches"])
        self.assertFalse(checks["eligible_pair_count_matches"])

    def test_atomic_verification_rejects_fabricated_receipt_and_rebuild_results(self) -> None:
        with tempfile.TemporaryDirectory(prefix="s1_verify_evidence_") as temporary:
            dataset_root = Path(temporary)
            package = dataset_root / "data/cat/src/a/urdf_w_collider"
            package.mkdir(parents=True)
            urdf = package / "a.urdf"
            urdf.write_text(
                '<robot name="fixture"><link name="base"/></robot>',
                encoding="utf-8",
            )
            urdf_sha = hashlib.sha256(urdf.read_bytes()).hexdigest()
            identity = {
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
                "urdf_sha256_expected": urdf_sha,
                "collision_mesh_files_expected": [],
                "table4_input_identity_sha256": "1" * 64,
                "strict_pass_no_method_allowance": False,
            }
            closure_files = [{"path": "a.urdf", "sha256": urdf_sha}]
            record = {
                **identity,
                "s1_input_identity_sha256": hashlib.sha256(
                    json.dumps(identity, sort_keys=True, separators=(",", ":")).encode()
                ).hexdigest(),
                "status": "completed",
                "binding": {"verified": True, "issues": []},
                "resource_closure": {
                    "status": "COMPLETE",
                    "complete": True,
                    "file_count": 1,
                    "sha256": hashlib.sha256(json.dumps(
                        closure_files, sort_keys=True, separators=(",", ":"),
                    ).encode()).hexdigest(),
                    "files": closure_files,
                    "issues": [],
                },
                "s1_evidence": {
                    "receipt": {
                        "candidate_count": 0,
                        "valid_mechanical_receipt_count": 1,
                        "receipt_bound_asset": 1,
                        "records": [],
                        "issues": [],
                    },
                    "receipt_replay": {
                        "eligible_receipt_count": 1,
                        "attempted": 0,
                        "passed": False,
                        "status": "NOT_EVALUABLE_NO_REGISTERED_REPLAY_BACKEND",
                    },
                    "rebuild": {
                        "status": "ELIGIBLE_NOT_RUN",
                        "eligible_asset": 1,
                        "candidate_recipe_count": 0,
                        "valid_recipe_count": 1,
                        "recipes": [],
                    },
                    "allowance": {
                        "candidate_file_count": 0,
                        "records": [],
                        "registered_excluded_pair_count": 0,
                        "eligible_nonadjacent_pair_count": 0,
                        "registration_status": "NO_PREREGISTERED_METHOD_SPECIFIC_REGISTRY",
                    },
                },
            }
            frozen = {
                "dataset_id": "artiverse_0000",
                "manifest_root": "data/cat/src/a",
                "input_identity_sha256": "1" * 64,
                "order": 0,
                "urdf_sha256": urdf_sha,
                "collision_mesh_files": [],
            }

            checks = self.verifier.verify_atomic_record(record, frozen, dataset_root)

        self.assertFalse(checks["evidence_atomic_results_match"])

    def test_atomic_verification_rederives_resource_closure_members(self) -> None:
        with tempfile.TemporaryDirectory(prefix="s1_verify_closure_") as temporary:
            dataset_root = Path(temporary)
            package = dataset_root / "data/cat/src/a/urdf_w_collider"
            (package / "meshes").mkdir(parents=True)
            mesh = package / "meshes/part.obj"
            mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")
            urdf = package / "a.urdf"
            urdf.write_text(
                '<robot name="fixture"><link name="base"><collision><geometry>'
                '<mesh filename="meshes/part.obj"/></geometry></collision></link></robot>',
                encoding="utf-8",
            )
            urdf_sha = hashlib.sha256(urdf.read_bytes()).hexdigest()
            identity = {
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
                "urdf_sha256_expected": urdf_sha,
                "collision_mesh_files_expected": [],
                "table4_input_identity_sha256": "1" * 64,
                "strict_pass_no_method_allowance": False,
            }
            incomplete_files = [{"path": "a.urdf", "sha256": urdf_sha}]
            record = {
                **identity,
                "s1_input_identity_sha256": hashlib.sha256(
                    json.dumps(identity, sort_keys=True, separators=(",", ":")).encode()
                ).hexdigest(),
                "binding": {"verified": True},
                "resource_closure": {
                    "status": "COMPLETE",
                    "complete": True,
                    "file_count": 1,
                    "sha256": hashlib.sha256(json.dumps(
                        incomplete_files, sort_keys=True, separators=(",", ":"),
                    ).encode()).hexdigest(),
                    "files": incomplete_files,
                    "issues": [],
                },
                "s1_evidence": {
                    "receipt": {"records": []},
                    "rebuild": {"recipes": []},
                    "allowance": {
                        "records": [],
                        "registered_excluded_pair_count": 0,
                        "eligible_nonadjacent_pair_count": 0,
                        "registration_status": "NO_PREREGISTERED_METHOD_SPECIFIC_REGISTRY",
                    },
                },
            }
            frozen = {
                "dataset_id": "artiverse_0000",
                "manifest_root": "data/cat/src/a",
                "input_identity_sha256": "1" * 64,
                "order": 0,
                "urdf_sha256": urdf_sha,
                "collision_mesh_files": [],
            }

            checks = self.verifier.verify_atomic_record(record, frozen, dataset_root)

        self.assertFalse(checks["resource_closure_bytes_match"])

    def test_scan_evidence_candidates_uses_shared_rebuild_filenames(self) -> None:
        with tempfile.TemporaryDirectory(prefix="s1_verify_recipe_names_") as temporary:
            package = Path(temporary)
            for name in ("build-recipe.json", "rebuild-recipe.json"):
                (package / name).write_text("{}", encoding="utf-8")

            inventory = self.verifier.scan_evidence_candidates(package)

        self.assertEqual(
            [row["path"] for row in inventory["rebuild_recipe_candidates"]],
            ["build-recipe.json", "rebuild-recipe.json"],
        )


if __name__ == "__main__":
    unittest.main()
