#!/usr/bin/env python3
"""Contract tests for the PV-A/Ours Table 2 supplementary adapter."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from exp.scripts import run_urdf_table2sup_ours_pva as runner


class PvaTable2SupplementaryTests(unittest.TestCase):
    def test_frozen_cohort_has_full_denominator_and_category_layout(self) -> None:
        cohort = runner.load_frozen_inputs(validate_packages=False)

        self.assertEqual(len(cohort.items), runner.EXPECTED_N)
        self.assertEqual(cohort.j_eval, runner.EXPECTED_J_EVAL)
        self.assertEqual(len(cohort.categories), runner.EXPECTED_CATEGORIES)
        self.assertEqual(set(cohort.category_counts.values()), {runner.PER_CLASS})

    def test_binding_check_rejects_urdf_drift(self) -> None:
        with tempfile.TemporaryDirectory(dir=runner.PVA_ROOT) as temporary:
            package = Path(temporary)
            urdf = package / "model.urdf"
            urdf.write_text('<robot name="x"><link name="base" /></robot>\n', encoding="utf-8")
            binding = runner.package_binding(package)
            item = {
                "package": str(package),
                "primary_urdf_relative_path": "model.urdf",
                "model_urdf_sha256_expected": runner.sha256_file(urdf),
                "package_content_manifest_sha256_expected": binding["content_manifest_sha256"],
                "package_binding_expected": binding,
            }
            urdf.write_text('<robot name="changed"><link name="base" /></robot>\n', encoding="utf-8")

            result = runner.verify_binding(item)

        self.assertFalse(result["verified"])
        self.assertTrue(any("model_urdf_sha256_mismatch" in issue for issue in result["issues"]))

    def test_frozen_manifest_identity_hash_is_canonical(self) -> None:
        item = {
            "selection_index": 0,
            "asset_id": "PV-A/test/seed_0000",
            "asset_root": "/tmp/package",
            "raw_category": "test",
            "seed_name": "seed_0000",
            "selection_rank": 1,
            "package": "/tmp/package",
            "primary_urdf_relative_path": "model.urdf",
            "expected_declared_joint_count": 1,
            "model_urdf_sha256_expected": "a" * 64,
            "package_content_manifest_sha256_expected": "b" * 64,
            "package_binding_files_expected": [],
        }
        identity = runner.input_identity(item)

        expected = runner.canonical_sha256(
            {field: item[field] for field in runner.INPUT_IDENTITY_FIELDS}
        )
        self.assertEqual(identity, expected)


if __name__ == "__main__":
    unittest.main()
