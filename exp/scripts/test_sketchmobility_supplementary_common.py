#!/usr/bin/env python3
"""Behavior tests for shared SketchMobility supplementary receipt contracts."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[2]
MODULE_PATH = REPO / "exp/scripts/sketchmobility_supplementary_common.py"


class SketchMobilitySupplementaryCommonTests(unittest.TestCase):
    def load_common(self):
        if not MODULE_PATH.is_file():
            self.fail(f"shared SketchMobility contract is missing: {MODULE_PATH}")
        spec = importlib.util.spec_from_file_location(
            "sketchmobility_supplementary_common_test_target", MODULE_PATH
        )
        if spec is None or spec.loader is None:
            self.fail(f"cannot load module: {MODULE_PATH}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module

    def test_loads_exact_frozen_table4_cohort(self) -> None:
        common = self.load_common()

        cohort = common.load_frozen_cohort(formal=True)

        self.assertEqual(800, len(cohort["rows"]))
        self.assertEqual(1824, cohort["joint_count"])
        self.assertEqual(
            "a88506e1da8e7e8b61a740965dea2faba4e9ab8280f47417e17550024b6dde17",
            cohort["ordered_asset_ids_sha256"],
        )
        self.assertEqual(1, cohort["rows"][0]["selection_rank"])
        self.assertEqual(
            "data/Shape2Motion/Kettle/kettle_0057",
            cohort["rows"][0]["asset_id"],
        )
        self.assertEqual(800, cohort["rows"][-1]["selection_rank"])

    def test_rejects_stored_order_drift(self) -> None:
        common = self.load_common()
        source = common.DEFAULT_TABLE4_RECEIPT / "manifest.json"
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="sketch-common-order-", dir=runtime_root
        ) as temporary:
            receipt = Path(temporary)
            manifest = json.loads(source.read_text("utf-8"))
            manifest["items"][0], manifest["items"][1] = (
                manifest["items"][1],
                manifest["items"][0],
            )
            (receipt / "manifest.json").write_text(
                json.dumps(manifest), encoding="utf-8"
            )

            with self.assertRaisesRegex(ValueError, "order|ordered asset"):
                common.load_frozen_cohort(receipt=receipt, formal=False)

    def test_audit_package_matches_recursive_frozen_binding(self) -> None:
        common = self.load_common()
        row = common.load_frozen_cohort(formal=True)["rows"][0]

        audit = common.audit_package(row, common.DEFAULT_DATASET_ROOT)

        self.assertEqual(
            "150fb5b16442ad363223d045fcddfa385d1d164851c6f37602a1c5cb64602711",
            audit["urdf_sha256"],
        )
        self.assertEqual(
            "b1e55aa48e8120a9e94e82d4400881054adf749c2cbcf09b7dcb7a0d301c1eae",
            audit["package_content_manifest_sha256"],
        )
        self.assertEqual(6, audit["package_file_count"])

    def test_rejects_symlink_in_package_closure(self) -> None:
        common = self.load_common()
        source_row = common.load_frozen_cohort(formal=True)["rows"][0]
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="sketch-common-symlink-", dir=runtime_root
        ) as temporary:
            root = Path(temporary)
            package = root / source_row["asset_root_relpath"]
            package.parent.mkdir(parents=True)
            source_package = common.DEFAULT_DATASET_ROOT / source_row["asset_root_relpath"]
            shutil.copytree(source_package, package)
            (package / "escape").symlink_to("/tmp")
            row = copy.deepcopy(source_row)

            with self.assertRaisesRegex(ValueError, "symlink"):
                common.audit_package(row, root)


if __name__ == "__main__":
    unittest.main()
