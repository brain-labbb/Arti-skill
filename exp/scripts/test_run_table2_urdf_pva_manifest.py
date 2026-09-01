#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from exp.scripts.run_table2_urdf_pva_manifest import (
    DEFAULT_INPUT,
    EXPECTED_N,
    build_jobs,
    load_cohort,
    parse_args,
)


class CohortAdapterTests(unittest.TestCase):
    def make_manifest(self, root: Path) -> Path:
        assets = []
        for index, category in enumerate(("Alpha", "Beta")):
            package = root / category / "seed_0000"
            package.mkdir(parents=True)
            urdf = package / "model.urdf"
            urdf.write_text(
                f'<robot name="{category}"><link name="base" /></robot>',
                encoding="utf-8",
            )
            from exp.scripts.run_table2_urdf_pva_manifest import TABLE2

            binding = TABLE2.package_binding(package)
            urdf_hash = TABLE2.sha256_file(urdf)
            assets.append(
                {
                    "selection_index": index,
                    "dataset_id": f"PV-A/{category}/seed_0000",
                    "asset_id": "seed_0000",
                    "category": category,
                    "package": str(package),
                    "primary_urdf_relative_path": "model.urdf",
                    "urdf_sha256": urdf_hash,
                    "package_binding": binding,
                }
            )
        manifest = {
            "schema_version": "test",
            "n_eval": 2,
            "class_count": 2,
            "per_class": 1,
            "assets": assets,
        }
        manifest["manifest_content_sha256"] = hashlib.sha256(
            json.dumps(
                manifest,
                sort_keys=True,
                separators=(",", ":"),
                ensure_ascii=True,
            ).encode()
        ).hexdigest()
        path = root / "manifest.json"
        path.write_text(json.dumps(manifest), encoding="utf-8")
        return path

    def test_loads_globally_unique_dataset_ids_and_accepts_repeated_seed_names(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = self.make_manifest(Path(temporary))
            cohort = load_cohort(path, expected_n=2, validate_packages=True)

            self.assertEqual(
                [row["dataset_id"] for row in cohort["assets"]],
                ["PV-A/Alpha/seed_0000", "PV-A/Beta/seed_0000"],
            )

    def test_build_jobs_uses_dataset_id_as_child_asset_key(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = self.make_manifest(Path(temporary))
            cohort = load_cohort(path, expected_n=2, validate_packages=True)
            jobs = build_jobs(
                cohort["assets"],
                manifest_content_sha256="a" * 64,
                run_standard_parser=False,
            )

            self.assertEqual(
                [job["asset_id"] for job in jobs],
                ["PV-A/Alpha/seed_0000", "PV-A/Beta/seed_0000"],
            )
            self.assertEqual(jobs[0]["primary_urdf_relative_path"], "model.urdf")
            self.assertEqual(jobs[0]["model_urdf_sha256"], cohort["assets"][0]["urdf_sha256"])

    def test_cli_freezes_source_denominator(self) -> None:
        with self.assertRaises(SystemExit):
            parse_args(["--expected-n", "2"])

    def test_frozen_pv_a_layout_is_checked(self) -> None:
        cohort = load_cohort(DEFAULT_INPUT, expected_n=EXPECTED_N)
        self.assertEqual(cohort["manifest"]["class_count"], 531)
        self.assertEqual(cohort["manifest"]["per_class"], 5)


if __name__ == "__main__":
    unittest.main()
