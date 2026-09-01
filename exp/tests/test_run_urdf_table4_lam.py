from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from unittest import mock


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/run_urdf_table4_lam.py"
DATASET_ROOT = REPO / "exp/Articulated-Object-Code"
SOURCE_RECORDS = (
    REPO
    / "exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3/asset_records.jsonl"
)
SOURCE_MANIFEST = SOURCE_RECORDS.parent / "manifest.json"
FORMAL_PROTOCOL_SNAPSHOT = (
    REPO
    / "exp/runtime/urdf_table4_lam_n800_20260814/protocol_document_at_freeze.md"
)


def load_runner():
    spec = importlib.util.spec_from_file_location("urdf_table4_lam", RUNNER)
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


def write_package(
    dataset_root: Path,
    object_release_id: str,
    urdf: str,
    *,
    category: str = "fixture_category",
) -> Path:
    package = (
        dataset_root
        / "released_outputs"
        / "objects"
        / category
        / object_release_id
    )
    package.mkdir(parents=True)
    (package / "generated.urdf").write_text(urdf, encoding="utf-8")
    (package / "fixture_metadata.json").write_text(
        json.dumps({"object_release_id": object_release_id, "category": category})
        + "\n",
        encoding="utf-8",
    )
    return package


def source_row(package: Path, selection_rank: int = 1, *, tier: str = "viable") -> dict:
    released_outputs = next(
        parent for parent in package.parents if parent.name == "released_outputs"
    )
    rel_path = package.relative_to(released_outputs).as_posix()
    category = Path(rel_path).parts[1]
    binding = package_binding(package)
    source_identity = {
        "asset_key": f"{tier}:{rel_path}",
        "selection_rank": selection_rank,
        "selection_hash": canonical_sha256(
            {"asset_key": f"{tier}:{rel_path}", "selection_rank": selection_rank}
        ),
        "tier": tier,
        "rel_path": rel_path,
        "object_release_id": package.name,
        "category": category,
        "urdf_sha256": sha256_file(package / "generated.urdf"),
    }
    manifest_identity = {
        **source_identity,
        "urdf_path": str((package / "generated.urdf").resolve()),
        "urdf_exists": True,
    }
    return {
        **{
            key: source_identity[key]
            for key in (
                "asset_key",
                "selection_rank",
                "selection_hash",
                "tier",
                "rel_path",
                "object_release_id",
            )
        },
        "category": category,
        "package": str(package.resolve()),
        "package_relpath": f"released_outputs/{rel_path}",
        "model_urdf_sha256": source_identity["urdf_sha256"],
        "package_binding": binding,
        "package_content_manifest_sha256": binding["content_manifest_sha256"],
        "source_record_sha256": canonical_sha256(source_identity),
        "source_manifest_record_sha256": canonical_sha256(manifest_identity),
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


class LamTable4ProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.runner = load_runner()
        snapshot_sha256 = sha256_file(FORMAL_PROTOCOL_SNAPSHOT)
        if snapshot_sha256 != cls.runner.EXPECTED_PROTOCOL_DOCUMENT_SHA256:
            raise AssertionError(
                "formal protocol snapshot SHA256 drift: "
                f"expected {cls.runner.EXPECTED_PROTOCOL_DOCUMENT_SHA256}, "
                f"got {snapshot_sha256}"
            )
        cls._original_protocol_document = cls.runner.PROTOCOL_DOCUMENT
        cls.runner.PROTOCOL_DOCUMENT = FORMAL_PROTOCOL_SNAPSHOT
        cls._table3_contract = None

    @classmethod
    def tearDownClass(cls) -> None:
        cls.runner.PROTOCOL_DOCUMENT = cls._original_protocol_document

    @classmethod
    def get_table3_contract(cls) -> dict:
        # Keep the expensive release/archive and 800-package re-hash to one pass,
        # while allowing purely synthetic tests to run without paying that cost.
        if cls._table3_contract is None:
            cls._table3_contract = cls.runner.load_table3_cohort(
                SOURCE_RECORDS,
                SOURCE_MANIFEST,
                DATASET_ROOT,
                sample_size=800,
                qualification_smoke=False,
            )
        return cls._table3_contract

    @classmethod
    def qualification_contract(cls, sample_size: int) -> dict:
        contract = copy.deepcopy(cls.get_table3_contract())
        contract["protocol_id"] = cls.runner.QUALIFICATION_PROTOCOL_ID
        contract["qualification_smoke"] = True
        contract["cohort_label"] = f"LAM released outputs qualification N={sample_size}"
        contract["selected"] = contract["selected"][:sample_size]
        return contract

    def test_exact_table3_cohort_is_reconstructed_by_selection_rank(self) -> None:
        contract = self.get_table3_contract()

        selected = contract["selected"]
        self.assertEqual(len(selected), 800)
        self.assertEqual([row["selection_rank"] for row in selected], list(range(1, 801)))
        self.assertEqual(
            selected[0]["asset_key"],
            "viable:objects/adjustable_wrench/adjustable_wrench_027",
        )
        self.assertEqual(
            selected[-1]["asset_key"],
            "viable:objects/cabinet_with_a_drawer/cabinet_with_a_drawer_017",
        )
        self.assertEqual(
            contract["selected_asset_keys_sha256"],
            "643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3",
        )
        self.assertEqual(
            contract["ordered_asset_identities_sha256"],
            self.runner.EXPECTED_ORDERED_ASSET_IDENTITIES_SHA256,
        )
        self.assertEqual(contract["release_asset_count"], 3217)
        self.assertEqual(contract["source_records_sha256"], "7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94")
        self.assertEqual(contract["source_manifest_sha256"], "7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951")

        jsonl_rows = [
            json.loads(line)
            for line in SOURCE_RECORDS.read_text(encoding="utf-8").splitlines()
        ]
        self.assertNotEqual(
            [row["selection_rank"] for row in jsonl_rows], list(range(1, 801))
        )
        self.assertEqual(
            [row["asset_key"] for row in sorted(jsonl_rows, key=lambda row: row["selection_rank"])],
            [row["asset_key"] for row in selected],
        )
        self.assertEqual(
            set(self.runner.IDENTITY_FIELDS),
            {
                "asset_key",
                "selection_rank",
                "selection_hash",
                "tier",
                "rel_path",
                "object_release_id",
                "package_relpath",
                "model_urdf_sha256",
                "package_content_manifest_sha256",
                "source_record_sha256",
                "source_manifest_record_sha256",
            },
        )

    def test_table3_source_bytes_and_duplicate_ranks_are_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            root = Path(temporary)
            tampered = root / "asset_records.jsonl"
            shutil.copyfile(SOURCE_RECORDS, tampered)
            with tampered.open("a", encoding="utf-8") as stream:
                stream.write("\n")
            with self.assertRaisesRegex(RuntimeError, "SHA256"):
                self.runner.load_table3_cohort(
                    tampered,
                    SOURCE_MANIFEST,
                    DATASET_ROOT,
                    sample_size=800,
                    qualification_smoke=False,
                )

            rows = [
                json.loads(line)
                for line in SOURCE_RECORDS.read_text(encoding="utf-8").splitlines()
            ]
            rows[0]["selection_rank"] = rows[1]["selection_rank"]
            duplicate = root / "duplicate_rank.jsonl"
            duplicate.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows),
                encoding="utf-8",
            )
            with (
                mock.patch.object(
                    self.runner,
                    "EXPECTED_SOURCE_RECORDS_SHA256",
                    sha256_file(duplicate),
                ),
                mock.patch.object(
                    self.runner, "_lam_source_provenance", return_value={"fixture": True}
                ),
            ):
                with self.assertRaisesRegex(RuntimeError, "selection ranks are invalid"):
                    self.runner.load_table3_cohort(
                        duplicate,
                        SOURCE_MANIFEST,
                        DATASET_ROOT,
                        sample_size=800,
                        qualification_smoke=False,
                    )

    def test_lam_relative_paths_and_generated_urdf_are_contained(self) -> None:
        self.assertEqual(
            self.runner._safe_lam_rel_path("objects/chair/chair_001"),
            "objects/chair/chair_001",
        )
        for value in (
            "../objects/chair/chair_001",
            "/objects/chair/chair_001",
            "objects/chair",
            "other/chair/chair_001",
            "objects/./chair_001",
        ):
            with self.subTest(value=value):
                with self.assertRaisesRegex(RuntimeError, "invalid LAM release"):
                    self.runner._safe_lam_rel_path(value)

        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(dataset_root, "fixture_primary", VALID_URDF)
            row = source_row(package)
            resolved_package, urdf_path = self.runner._resolve_primary_urdf(
                dataset_root, row
            )
            self.assertEqual(resolved_package, package.resolve())
            self.assertEqual(urdf_path.name, "generated.urdf")
            (package / "generated.urdf").rename(package / "model.urdf")
            with self.assertRaisesRegex(FileNotFoundError, "generated.urdf"):
                self.runner._resolve_primary_urdf(dataset_root, row)

            escape = dict(row)
            escape["package_relpath"] = "../outside"
            with self.assertRaisesRegex(ValueError, "package_relpath"):
                self.runner._resolve_primary_urdf(dataset_root, escape)

    def test_complete_collision_package_audit_derives_collision_scale(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(dataset_root, "fixture_complete", VALID_URDF)
            row = source_row(package)
            audit = self.runner.audit_asset(dataset_root, row)

        self.assertTrue(audit["package_audit_success"])
        self.assertTrue(audit["collision_coverage_complete"])
        self.assertEqual(audit["link_count"], 2)
        self.assertEqual(audit["collision_covered_link_count"], 2)
        self.assertEqual(audit["movable_dof_count"], 1)
        self.assertEqual(audit["range_evaluable_dof_count"], 1)
        self.assertEqual(
            audit["primary_urdf_relpath"],
            "released_outputs/objects/fixture_category/fixture_complete/generated.urdf",
        )
        self.assertEqual(audit["scale_derivation"]["status"], "PASS")
        self.assertGreater(audit["object_bbox_diagonal_m"], 0.0)

    def test_visual_only_package_is_fail_closed_without_scale_fallback(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(dataset_root, "fixture_visual", VISUAL_ONLY_URDF)
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

    def test_partial_collision_failure_retains_all_frozen_denominators(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(dataset_root, "fixture_partial", PARTIAL_COLLISION_URDF)
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
                    "cohort_label": "LAM released outputs qualification N=1",
                    "sample_size": 1,
                    "items": [item],
                },
                [result],
            )

        self.assertFalse(audit["package_audit_success"])
        self.assertEqual(audit["collision_covered_link_count"], 1)
        self.assertIn("collision coverage incomplete: 1/2", audit["audit_issue"])
        self.assertFalse(result["load_success"])
        self.assertFalse(result["measurement_complete"])
        self.assertEqual(result["rest_state_expected"], 1)
        self.assertEqual(result["single_state_expected"], 21)
        self.assertEqual(result["sobol_state_expected"], 64)
        self.assertEqual(result["rest_state_executed"], 0)
        self.assertEqual(result["single_state_executed"], 0)
        self.assertEqual(result["sobol_state_executed"], 0)
        self.assertEqual(summary["metrics"]["collision_state_rate"]["denominator"], 86)
        self.assertEqual(summary["metrics"]["collision_state_rate"]["collision_states"], 86)
        self.assertEqual(summary["metrics"]["collision_state_rate"]["unexecuted_states"], 86)

    def test_full_package_snapshot_detects_drift_even_for_failed_assets(self) -> None:
        for object_id, urdf in (
            ("fixture_drift_complete", VALID_URDF),
            ("fixture_drift_failed", VISUAL_ONLY_URDF),
        ):
            with self.subTest(object_id=object_id):
                with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
                    dataset_root = Path(temporary)
                    package = write_package(dataset_root, object_id, urdf)
                    row = source_row(package)
                    item = self.runner.freeze_item(
                        row,
                        self.runner.audit_asset(dataset_root, row),
                        order=0,
                        protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                        runtime_identity={"fixture": True},
                    )
                    (package / "fixture_metadata.json").write_text(
                        "{}\n", encoding="utf-8"
                    )
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

    def test_evaluate_asset_runs_all_states_and_binds_lam_source_identity(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(
                dataset_root,
                "fixture_evaluate",
                VALID_URDF,
                category="adjustable_wrench",
            )
            row = source_row(package, selection_rank=17)
            item = self.runner.freeze_item(
                row,
                self.runner.audit_asset(dataset_root, row),
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity=self.runner.current_runtime_identity(),
            )

            result = self.runner.evaluate_asset(item, dataset_root)

        self.assertTrue(result["load_success"])
        self.assertTrue(result["measurement_complete"])
        self.assertEqual(result["dataset_id"], "lam_0000")
        self.assertEqual(result["asset_key"], row["asset_key"])
        self.assertEqual(result["selection_rank"], 17)
        self.assertEqual(result["category"], "adjustable_wrench")
        self.assertEqual(
            result["package_relpath"],
            "released_outputs/objects/adjustable_wrench/fixture_evaluate",
        )
        self.assertEqual(result["rest_state_executed"], 1)
        self.assertEqual(result["single_state_executed"], 21)
        self.assertEqual(result["sobol_state_executed"], 64)
        self.assertEqual(len(result["state_records"]), 86)
        self.assertEqual(
            result["state_records_sha256"],
            self.runner.canonical_sha256(result["state_records"]),
        )
        for state in result["state_records"]:
            for field in self.runner.IDENTITY_FIELDS:
                self.assertEqual(state[field], item[field])

    def test_category_macro_is_authoritative_and_fail_closed(self) -> None:
        items = []
        rows = []
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            for order, category in enumerate(("category_a", "category_a", "category_b")):
                package = write_package(
                    dataset_root,
                    f"fixture_macro_{order}",
                    VALID_URDF,
                    category=category,
                )
                source = source_row(package, selection_rank=order + 1)
                audit = self.runner._empty_audit()
                audit.update(
                    {
                        "movable_dof_count": 1,
                        "range_evaluable_dof_count": 1,
                    }
                )
                item = self.runner.freeze_item(
                    source,
                    audit,
                    order=order,
                    protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                    runtime_identity={"fixture": True},
                )
                items.append(item)
                rows.append(self.runner.failure_record(item, "fixture_failure"))

        rows[0]["max_penetration_normalized"] = 999.0
        for index, maximum in ((1, 0.25), (2, 0.5)):
            rows[index].update(
                {
                    "load_success": True,
                    "measurement_complete": True,
                    "rest_state_executed": 1,
                    "single_state_executed": 21,
                    "sobol_state_executed": 64,
                    "rest_non_adjacent_free": 1,
                    "single_non_adjacent_free": 21,
                    "sobol_non_adjacent_free": 64,
                    "joint_single_sweep_cf_passed": 1,
                    "rest_all_pair_cf": True,
                    "rest_non_adjacent_cf": True,
                    "single_joint_sweep_cf": True,
                    "multi_joint_sobol_cf": True,
                    "strict_collision_pass": True,
                    "max_penetration_normalized": maximum,
                }
            )
        manifest = {
            "protocol_id": self.runner.QUALIFICATION_PROTOCOL_ID,
            "cohort_label": "LAM released outputs qualification N=3",
            "sample_size": 3,
            "items": items,
        }

        summary = self.runner.summarize_records(manifest, rows)

        self.assertEqual(summary["cohort"]["selected"], 3)
        self.assertEqual(summary["cohort"]["category_count"], 2)
        self.assertEqual(set(summary["category_results"]), {"category_a", "category_b"})
        self.assertAlmostEqual(
            summary["category_results"]["category_a"]["collision_state_rate"]["rate"],
            0.5,
        )
        self.assertAlmostEqual(
            summary["category_results"]["category_b"]["collision_state_rate"]["rate"],
            0.0,
        )
        self.assertAlmostEqual(summary["category_macro"]["collision_state_rate"], 0.25)
        self.assertEqual(
            summary["metrics"]["max_penetration"]["maximum_observed_normalized"],
            0.5,
        )
        self.assertEqual(summary["metrics"]["max_penetration"]["observed_assets"], 2)
        self.assertEqual(
            summary["metrics"]["max_penetration"]["normalization"],
            "PyBullet q=0 collision-shape union AABB diagonal",
        )
        self.assertEqual(summary["metrics"]["aor"]["status"], "N/E")

    def test_qualification_manifest_binds_table3_jsonl_manifest_and_categories(self) -> None:
        runtime = self.runner.current_runtime_identity()
        contract = self.qualification_contract(2)
        with mock.patch.object(
            self.runner, "load_table3_cohort", return_value=contract
        ):
            manifest = self.runner.build_manifest(
                DATASET_ROOT,
                SOURCE_RECORDS,
                SOURCE_MANIFEST,
                sample_size=2,
                qualification_smoke=True,
                child_runtime=runtime,
                workers=2,
            )

            self.runner.validate_manifest(
                manifest,
                DATASET_ROOT,
                SOURCE_RECORDS,
                SOURCE_MANIFEST,
                qualification_smoke=True,
                child_runtime=runtime,
            )

            rebound_source = copy.deepcopy(manifest)
            rebound_source["source"]["revision"] = "0" * 40
            rebound_source["manifest_content_sha256"] = self.runner.manifest_self_hash(
                rebound_source
            )
            with self.assertRaisesRegex(RuntimeError, "Table 3 source closure mismatch"):
                self.runner.validate_manifest(
                    rebound_source,
                    DATASET_ROOT,
                    SOURCE_RECORDS,
                    SOURCE_MANIFEST,
                    qualification_smoke=True,
                    child_runtime=runtime,
                )

            rebound_item = copy.deepcopy(manifest)
            rebound_item["items"][0]["rel_path"] = "objects/other/other_001"
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
                    SOURCE_RECORDS,
                    SOURCE_MANIFEST,
                    qualification_smoke=True,
                    child_runtime=runtime,
                )

        self.assertEqual(manifest["sample_size"], 2)
        self.assertEqual(manifest["release_asset_count"], 3217)
        self.assertEqual(
            [item["asset_key"] for item in manifest["items"]],
            [row["asset_key"] for row in contract["selected"]],
        )
        self.assertEqual(
            [item["category"] for item in manifest["items"]],
            [row["category"] for row in contract["selected"]],
        )
        self.assertEqual(
            manifest["source"]["table3_asset_records_sha256"],
            self.runner.EXPECTED_SOURCE_RECORDS_SHA256,
        )
        self.assertEqual(
            manifest["source"]["table3_manifest_sha256"],
            self.runner.EXPECTED_SOURCE_MANIFEST_SHA256,
        )
        self.assertTrue(
            manifest["cohort_boundary"]["authoritative_category_labels_available"]
        )
        self.assertFalse(manifest["cohort_boundary"]["is_full_release_cohort"])
        self.assertFalse(
            manifest["cohort_boundary"]["is_shared_category_balanced_cohort"]
        )
        self.assertEqual(
            manifest["items_sha256"], self.runner.canonical_sha256(manifest["items"])
        )

    def test_manifest_semantics_and_formal_denominators_are_frozen(self) -> None:
        expected = dict(self.runner.EXPECTED_FORMAL_AUDIT_SUMMARY)
        self.assertEqual(expected["movable_dof_count"], 2395)
        self.assertEqual(expected["range_evaluable_dof_count"], 2382)
        self.assertEqual(expected["rest_state_expected"], 800)
        self.assertEqual(expected["single_state_expected"], 50295)
        self.assertEqual(expected["sobol_state_expected"], 49536)
        self.runner._validate_formal_audit_summary(expected, qualification_smoke=False)
        wrong = dict(expected)
        wrong["single_state_expected"] -= 21
        with self.assertRaisesRegex(RuntimeError, "formal static audit invariant"):
            self.runner._validate_formal_audit_summary(
                wrong, qualification_smoke=False
            )
        self.runner._validate_formal_audit_summary(wrong, qualification_smoke=True)

        contract = self.qualification_contract(1)
        with mock.patch.object(
            self.runner, "load_table3_cohort", return_value=contract
        ):
            manifest = self.runner.build_manifest(
                DATASET_ROOT,
                SOURCE_RECORDS,
                SOURCE_MANIFEST,
                sample_size=1,
                qualification_smoke=True,
                child_runtime=self.runner.current_runtime_identity(),
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

    def test_state_closure_rejects_every_lam_identity_field_drift(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            dataset_root = Path(temporary)
            package = write_package(
                dataset_root,
                "fixture_state",
                VALID_URDF,
                category="state_category",
            )
            row = source_row(package, selection_rank=23)
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
                with self.assertRaisesRegex(RuntimeError, "source identity mismatch"):
                    self.runner.validate_state_closure(
                        record, [{**state, field: replacement}], item
                    )

        impossible_penetration = copy.deepcopy(state)
        impossible_penetration["all_pair_contact_count"] = 1
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

    def test_cached_and_terminal_children_require_exact_identity(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            root = Path(temporary)
            dataset_root = root / "dataset"
            output = root / "output"
            package = write_package(dataset_root, "fixture_cache", VISUAL_ONLY_URDF)
            row = source_row(package)
            item = self.runner.freeze_item(
                row,
                self.runner.audit_asset(dataset_root, row),
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity=self.runner.current_runtime_identity(),
            )
            runner_hash = self.runner.sha256_file(RUNNER)
            result = self.runner.failure_record(item, "fixture_failure")
            forged_metric = copy.deepcopy(result)
            forged_metric["max_penetration_normalized"] = 123.456
            self.assertFalse(
                self.runner.result_matches_item(forged_metric, item, runner_hash)
            )

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
            self.assertTrue(self.runner._valid_terminal_child(path, item, runner_hash))
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
            self.assertTrue(self.runner._valid_terminal_child(path, item, runner_hash))
            self.assertFalse(self.runner._valid_cached_child(path, item, runner_hash))

            for field in ("asset_key", "selection_rank", "category", "runtime_identity"):
                stale = copy.deepcopy(result)
                value = stale[field]
                stale[field] = value + 1 if isinstance(value, int) else (
                    {"fixture": "wrong"}
                    if isinstance(value, dict)
                    else f"{value}_other"
                )
                self.runner.atomic_json(path, stale)
                self.assertFalse(
                    self.runner._valid_cached_child(path, item, runner_hash), field
                )

    def test_run_one_finalizes_raw_child_and_retains_terminal_timeout(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            root = Path(temporary)
            dataset_root = root / "dataset"
            output = root / "output"
            package = write_package(dataset_root, "fixture_child", VALID_URDF)
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
            child_log = output / "child_logs" / f"{self.runner._job_prefix(item)}.log"
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
            self.assertTrue(self.runner._valid_cached_child(child_path, item, runner_hash))

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
            self.assertFalse(self.runner._valid_cached_child(child_path, item, runner_hash))
            replay = self.runner._replay_frozen_measurements(
                {"sample_size": 1, "items": [item]}, output, dataset_root
            )
            self.assertEqual(replay[0]["replay_mode"], "canonical_terminal_failure")

    def test_verifier_replays_success_and_failure_measurements_exactly(self) -> None:
        with tempfile.TemporaryDirectory(dir=REPO / "exp/runtime") as temporary:
            root = Path(temporary)
            dataset_root = root / "dataset"
            output = root / "output"
            package = write_package(dataset_root, "fixture_replay", OVERLAPPING_URDF)
            row = source_row(package)
            item = self.runner.freeze_item(
                row,
                self.runner.audit_asset(dataset_root, row),
                order=0,
                protocol_id=self.runner.QUALIFICATION_PROTOCOL_ID,
                runtime_identity=self.runner.current_runtime_identity(),
            )
            failed_package = write_package(
                dataset_root, "fixture_replay_failed", VISUAL_ONLY_URDF
            )
            failed_row = source_row(failed_package, selection_rank=2)
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
            results = [
                self.runner.evaluate_asset(item, dataset_root),
                self.runner.evaluate_asset(failed_item, dataset_root),
            ]
            for frozen, result in zip((item, failed_item), results):
                result.update(
                    {
                        "child_returncode": 0,
                        "child_timed_out": False,
                        "child_log": str(
                            output
                            / "child_logs"
                            / f"{self.runner._job_prefix(frozen)}.log"
                        ),
                        "cache_reused": False,
                    }
                )
                self.runner.atomic_json(
                    self.runner.child_result_path(output, frozen), result
                )

            bindings = self.runner._replay_frozen_measurements(
                manifest, output, dataset_root
            )
            self.assertEqual(len(bindings), 2)
            self.assertEqual(
                [binding["dataset_id"] for binding in bindings],
                [item["dataset_id"], failed_item["dataset_id"]],
            )

            forged = copy.deepcopy(results[0])
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
            self.runner.atomic_json(
                self.runner.child_result_path(output, item), forged
            )
            with self.assertRaisesRegex(RuntimeError, "measurement replay mismatch"):
                self.runner._replay_frozen_measurements(
                    manifest, output, dataset_root
                )

    def test_collision_core_and_child_launcher_are_exactly_pinned(self) -> None:
        core = self.runner._load_core()
        self.assertIsNone(getattr(core, "__cached__", None))
        self.assertEqual(core.__source_sha256__, self.runner.EXPECTED_CORE_SHA256)

        binding = self.runner.frozen_launcher_binding(
            self.runner.DEFAULT_CHILD_PYTHON, REPO
        )
        self.assertEqual(binding["launch_path"], str(self.runner.DEFAULT_CHILD_PYTHON))
        self.assertEqual(
            binding["symlink_target"],
            self.runner.EXPECTED_CHILD_PYTHON_SYMLINK_TARGET,
        )
        self.assertEqual(
            binding["resolved_executable_sha256"],
            self.runner.EXPECTED_CHILD_PYTHON_SHA256,
        )
        self.assertEqual(
            binding["pyvenv_cfg_sha256"], self.runner.EXPECTED_PYVENV_CFG_SHA256
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

    def test_n1_fail_closed_artifacts_verify_and_report_tamper_fails(self) -> None:
        runtime = self.runner.current_runtime_identity()
        contract = self.qualification_contract(1)
        with mock.patch.object(
            self.runner, "load_table3_cohort", return_value=contract
        ):
            manifest = self.runner.build_manifest(
                DATASET_ROOT,
                SOURCE_RECORDS,
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
                    receipt["experiment_status"],
                    "COMPLETE_WITH_RETAINED_FAILURES",
                )
                self.assertEqual(receipt["retained_failure_assets"], 1)
                self.assertEqual(
                    receipt["unexecuted_states"],
                    summary["metrics"]["collision_state_rate"]["denominator"],
                )
                self.assertEqual(summary["cohort"]["category_count"], 1)
                self.assertIn(item["category"], summary["category_results"])

                (output / "report.md").write_text("# tampered\n", encoding="utf-8")
                with self.assertRaisesRegex(RuntimeError, "verification failed"):
                    self.runner.verify(manifest, output)
                failed = self.runner.read_json(output / "verification.json")
                self.assertEqual(failed["status"], "FAIL")
                self.assertFalse(failed["checks"]["report_recomputes_exactly"])

                self.runner.atomic_json(
                    output / "verification.json", {"status": "PASS", "marker": "stale"}
                )
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

    def test_report_names_lam_and_preserves_claim_boundaries(self) -> None:
        summary = {
            "status": "COMPLETE_WITH_RETAINED_FAILURES",
            "cohort": {
                "label": "LAM released outputs fixed Table 3 N=800 cohort",
                "category_count": 305,
            },
            "category_macro": {
                "rest_all_pair_cf": 0.25,
                "collision_state_rate": 0.125,
            },
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
                "strict_collision_pass": {
                    "passed": 1,
                    "denominator": 3,
                    "rate": 1 / 3,
                },
            },
        }

        report = self.runner.report_text(summary)

        self.assertIn("LAM released outputs fixed Table 3 N=800 cohort", report)
        self.assertIn("3,217-asset LAM release universe", report)
        self.assertIn("Observed authoritative categories: 305", report)
        self.assertIn("collision_state_rate=12.500%", report)
        self.assertIn("AOR | N/E", report)
        self.assertNotIn("Articraft-10K", report)
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
                    "--source-records",
                    str(SOURCE_RECORDS),
                    "--table3-manifest",
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

        self.assertNotEqual(completed.returncode, 0)
        self.assertEqual(receipt["status"], "FAIL")
        self.assertNotIn("marker", receipt)
        self.assertIn("fatal_verification_error", receipt["errors"])


if __name__ == "__main__":
    unittest.main()
