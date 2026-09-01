#!/usr/bin/env python3
"""Behavior tests for the SketchMobility Table 2 adapter."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO / "exp/scripts/run_table2_urdf_sketch_mobility.py"


class SketchMobilityTable2AdapterTests(unittest.TestCase):
    def load_runner(self):
        if not RUNNER_PATH.is_file():
            self.fail(f"SketchMobility Table 2 runner is missing: {RUNNER_PATH}")
        spec = importlib.util.spec_from_file_location(
            "run_table2_urdf_sketch_mobility_test_target", RUNNER_PATH
        )
        if spec is None or spec.loader is None:
            self.fail(f"cannot load runner: {RUNNER_PATH}")
        runner = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = runner
        spec.loader.exec_module(runner)
        return runner

    def test_loads_exact_frozen_table1_cohort_in_rank_order(self) -> None:
        runner = self.load_runner()
        cohort = runner.load_table1_cohort(
            runner.DEFAULT_TABLE1_RECEIPT,
            runner.DEFAULT_DATASET_ROOT,
        )

        self.assertEqual(4956, cohort["release_asset_count"])
        self.assertEqual(70, cohort["release_category_count"])
        self.assertEqual(800, len(cohort["rows"]))
        self.assertEqual(
            "data/Shape2Motion/Kettle/kettle_0057",
            cohort["rows"][0]["asset_id"],
        )
        self.assertEqual(1, cohort["rows"][0]["selection_rank"])
        self.assertEqual(
            "data/Shape2Motion/Toilet/toilet_0117",
            cohort["rows"][-1]["asset_id"],
        )
        self.assertEqual(800, cohort["rows"][-1]["selection_rank"])

    def test_builds_job_for_mobility_urdf_inside_asset_package(self) -> None:
        runner = self.load_runner()
        cohort = runner.load_table1_cohort(
            runner.DEFAULT_TABLE1_RECEIPT,
            runner.DEFAULT_DATASET_ROOT,
        )

        jobs = runner.build_jobs(
            cohort["rows"][:1],
            runner.DEFAULT_DATASET_ROOT,
            manifest_content_sha256="a" * 64,
            run_standard_parser=True,
        )

        self.assertEqual(1, len(jobs))
        job = jobs[0]
        self.assertEqual("mobility.urdf", job["primary_urdf_relative_path"])
        self.assertEqual("Shape2Motion/Kettle", job["raw_category"])
        self.assertEqual(1, job["table1_selection_rank"])
        self.assertEqual(
            str(
                (
                    runner.DEFAULT_DATASET_ROOT
                    / "data/Shape2Motion/Kettle/kettle_0057"
                ).resolve()
            ),
            job["package"],
        )
        self.assertEqual(
            "150fb5b16442ad363223d045fcddfa385d1d164851c6f37602a1c5cb64602711",
            job["primary_urdf_sha256"],
        )

    def test_rejects_live_primary_urdf_hash_drift(self) -> None:
        runner = self.load_runner()
        cohort = runner.load_table1_cohort(
            runner.DEFAULT_TABLE1_RECEIPT,
            runner.DEFAULT_DATASET_ROOT,
        )
        tampered = dict(cohort["rows"][0])
        tampered["primary_urdf_sha256"] = "0" * 64

        with self.assertRaisesRegex(ValueError, "mobility.urdf hash mismatch"):
            runner.build_jobs(
                [tampered],
                runner.DEFAULT_DATASET_ROOT,
                manifest_content_sha256="b" * 64,
                run_standard_parser=True,
            )

    def test_published_run_retains_valid_protocol_snapshot_binding(self) -> None:
        runner = self.load_runner()
        runtime_root = REPO / "exp/runtime"
        runtime_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(
            prefix="table2-sketch-protocol-test-", dir=runtime_root
        ) as temporary:
            output = Path(temporary) / "published"
            args = runner.parse_args(
                [
                    "--limit",
                    "1",
                    "--workers",
                    "1",
                    "--output",
                    str(output),
                ]
            )

            runner.run(args)

            published = output.resolve(strict=True)
            manifest = json.loads((published / "manifest.json").read_text("utf-8"))
            runner.TABLE2.validate_protocol_snapshot_binding(
                published,
                manifest["evaluation"],
            )

    def test_release_manifest_identity_rejects_byte_drift(self) -> None:
        runner = self.load_runner()
        self.assertEqual(
            runner.EXPECTED_RELEASE_MANIFEST_SHA256,
            runner.validate_release_manifest_identity(runner.DEFAULT_DATASET_ROOT),
        )

        runtime_root = REPO / "exp/runtime"
        runtime_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(
            prefix="table2-sketch-release-test-", dir=runtime_root
        ) as temporary:
            dataset_root = Path(temporary)
            manifest_dir = dataset_root / "dataset_chunks"
            manifest_dir.mkdir()
            source = runner.DEFAULT_DATASET_ROOT / "dataset_chunks/manifest.json"
            altered = json.loads(source.read_text("utf-8"))
            altered["chunks"] = altered["chunks"][:-1]
            (manifest_dir / "manifest.json").write_text(
                json.dumps(altered), encoding="utf-8"
            )

            with self.assertRaisesRegex(ValueError, "release manifest hash mismatch"):
                runner.validate_release_manifest_identity(dataset_root)

    def test_formal_runtime_requires_pinned_python_and_urdfpy(self) -> None:
        runner = self.load_runner()
        runner.validate_formal_runtime("3.12.3", "0.0.22")

        with self.assertRaisesRegex(ValueError, "Python 3.12.3"):
            runner.validate_formal_runtime("3.13.2", "0.0.22")
        with self.assertRaisesRegex(ValueError, "urdfpy 0.0.22"):
            runner.validate_formal_runtime("3.12.3", "0.0.21")


if __name__ == "__main__":
    unittest.main()
