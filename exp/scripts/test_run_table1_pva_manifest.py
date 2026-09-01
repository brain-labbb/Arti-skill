#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from contextlib import redirect_stderr

from exp.scripts.run_table1_pva_manifest import load_cohort, parse_args


def canonical_hash(value: object) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )
    return hashlib.sha256(payload.encode()).hexdigest()


class LoadCohortTests(unittest.TestCase):
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
            files = [
                {
                    "path": "model.urdf",
                    "bytes": urdf.stat().st_size,
                    "sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
                }
            ]
            assets.append(
                {
                    "selection_index": index,
                    "dataset_id": f"PV-A/{category}/seed_0000",
                    "asset_id": "seed_0000",
                    "category": category,
                    "package": str(package),
                    "primary_urdf_relative_path": "model.urdf",
                    "urdf_sha256": files[0]["sha256"],
                    "package_binding": {
                        "file_count": 1,
                        "total_bytes": files[0]["bytes"],
                        "files": files,
                        "content_manifest_sha256": canonical_hash(files),
                    },
                }
            )
        manifest = {
            "schema_version": "test",
            "n_eval": 2,
            "class_count": 2,
            "per_class": 1,
            "assets": assets,
        }
        manifest["manifest_content_sha256"] = canonical_hash(manifest)
        path = root / "manifest.json"
        path.write_text(json.dumps(manifest), encoding="utf-8")
        return path

    def test_uses_globally_unique_dataset_id_and_preserves_order(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = self.make_manifest(Path(temporary))

            cohort = load_cohort(path, expected_n=2, validate_packages=True)

        self.assertEqual(
            [row["dataset_id"] for row in cohort["assets"]],
            ["PV-A/Alpha/seed_0000", "PV-A/Beta/seed_0000"],
        )

    def test_rejects_manifest_self_hash_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = self.make_manifest(Path(temporary))
            manifest = json.loads(path.read_text())
            manifest["class_count"] = 999
            path.write_text(json.dumps(manifest), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "self-hash"):
                load_cohort(path, expected_n=2, validate_packages=False)

    def test_rejects_class_count_that_disagrees_with_assets(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = self.make_manifest(Path(temporary))
            manifest = json.loads(path.read_text())
            manifest["class_count"] = 1
            manifest.pop("manifest_content_sha256")
            manifest["manifest_content_sha256"] = canonical_hash(manifest)
            path.write_text(json.dumps(manifest), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "class_count"):
                load_cohort(path, expected_n=2, validate_packages=False)

    def test_cli_rejects_changing_the_frozen_source_denominator(self) -> None:
        with redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parse_args(["--expected-n", "2"])


if __name__ == "__main__":
    unittest.main()
