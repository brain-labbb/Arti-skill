#!/usr/bin/env python3
"""Behavior tests for the SketchMobility Table 3 adapter."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


REPO = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO / "exp/scripts/run_table3_urdf_sketch_mobility.py"
VERIFIER_PATH = REPO / "exp/scripts/verify_table3_urdf_sketch_mobility.py"


class SketchMobilityTable3AdapterTests(unittest.TestCase):
    def load_runner(self):
        if not RUNNER_PATH.is_file():
            self.fail(f"SketchMobility Table 3 runner is missing: {RUNNER_PATH}")
        spec = importlib.util.spec_from_file_location(
            "run_table3_urdf_sketch_mobility_test_target", RUNNER_PATH
        )
        if spec is None or spec.loader is None:
            self.fail(f"cannot load runner: {RUNNER_PATH}")
        runner = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = runner
        spec.loader.exec_module(runner)
        return runner

    def load_verifier(self):
        if not VERIFIER_PATH.is_file():
            self.fail(f"SketchMobility Table 3 verifier is missing: {VERIFIER_PATH}")
        spec = importlib.util.spec_from_file_location(
            "verify_table3_urdf_sketch_mobility_test_target", VERIFIER_PATH
        )
        if spec is None or spec.loader is None:
            self.fail(f"cannot load verifier: {VERIFIER_PATH}")
        verifier = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = verifier
        spec.loader.exec_module(verifier)
        return verifier

    def test_loads_exact_table2_cohort_in_table1_rank_order(self) -> None:
        runner = self.load_runner()

        cohort = runner.load_frozen_cohort(
            runner.DEFAULT_TABLE2_MANIFEST,
            runner.DEFAULT_TABLE1_RECEIPT,
            runner.DEFAULT_DATASET_ROOT,
            formal=True,
        )

        self.assertEqual(4956, cohort["n_release"])
        self.assertEqual(70, cohort["release_category_count"])
        self.assertEqual(67, cohort["eval_category_count"])
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

    def test_builds_asset_for_mobility_urdf_and_full_package_binding(self) -> None:
        runner = self.load_runner()
        cohort = runner.load_frozen_cohort(
            runner.DEFAULT_TABLE2_MANIFEST,
            runner.DEFAULT_TABLE1_RECEIPT,
            runner.DEFAULT_DATASET_ROOT,
            formal=True,
        )

        assets = runner.build_assets(cohort["rows"][:1], runner.DEFAULT_DATASET_ROOT)

        self.assertEqual(1, len(assets))
        asset = assets[0]
        self.assertEqual("Shape2Motion/Kettle", asset["raw_category"])
        self.assertEqual(1, asset["selection_rank"])
        self.assertEqual(1, asset["declared_joint_count_hint"])
        self.assertEqual(
            str(
                (
                    runner.DEFAULT_DATASET_ROOT
                    / "data/Shape2Motion/Kettle/kettle_0057/mobility.urdf"
                ).resolve()
            ),
            asset["urdf_path"],
        )
        self.assertEqual(
            "150fb5b16442ad363223d045fcddfa385d1d164851c6f37602a1c5cb64602711",
            asset["urdf_sha256"],
        )
        self.assertGreater(asset["package_binding"]["file_count"], 2)

    def test_rejects_table2_manifest_byte_drift(self) -> None:
        runner = self.load_runner()
        runtime_root = REPO / "exp/runtime"
        runtime_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(
            prefix="table3-sketch-cohort-test-", dir=runtime_root
        ) as temporary:
            altered_path = Path(temporary) / "manifest.json"
            altered = json.loads(runner.DEFAULT_TABLE2_MANIFEST.read_text("utf-8"))
            altered["assets"] = altered["assets"][:-1]
            altered_path.write_text(json.dumps(altered), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "Table 2 manifest file hash mismatch"):
                runner.load_frozen_cohort(
                    altered_path,
                    runner.DEFAULT_TABLE1_RECEIPT,
                    runner.DEFAULT_DATASET_ROOT,
                    formal=True,
                )

    def test_formal_runtime_requires_pinned_python_and_numpy(self) -> None:
        runner = self.load_runner()
        runner.validate_formal_runtime("3.12.3", "2.5.1")

        with self.assertRaisesRegex(ValueError, "Python 3.12.3"):
            runner.validate_formal_runtime("3.13.2", "2.5.1")
        with self.assertRaisesRegex(ValueError, "numpy 2.5.1"):
            runner.validate_formal_runtime("3.12.3", "2.4.4")

    def test_published_smoke_retains_protocol_and_artifact_bindings(self) -> None:
        runner = self.load_runner()
        runtime_root = REPO / "exp/runtime"
        runtime_root.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(
            prefix="table3-sketch-publication-test-", dir=runtime_root
        ) as temporary:
            output = Path(temporary) / "published"
            args = runner.parse_args(
                [
                    "--mode",
                    "smoke",
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
            runner.validate_protocol_snapshot_binding(
                published, manifest["evaluation"]
            )
            runner.verify_artifacts(published)
            summary = json.loads((published / "summary.json").read_text("utf-8"))
            self.assertEqual(1, summary["n_eval"])
            self.assertEqual({"completed": 1}, summary["status_counts"])
            verification = json.loads(
                (published / "verification.json").read_text("utf-8")
            )
            self.assertEqual("PASS", verification["status"])
            self.assertTrue(all(verification["checks"].values()))
            source_snapshot = published / "source_snapshot"
            for name in (
                "run_table3_urdf_sketch_mobility.py",
                "run_urdf_table3_ours_500k.py",
                "run_urdf_table3_lam.py",
                "verify_table3_urdf_sketch_mobility.py",
            ):
                self.assertTrue((source_snapshot / name).is_file())
            self.assertFalse((source_snapshot / "__pycache__").exists())
            self.assertEqual(
                {
                    "OMP_NUM_THREADS": "1",
                    "OPENBLAS_NUM_THREADS": "1",
                    "MKL_NUM_THREADS": "1",
                    "NUMEXPR_NUM_THREADS": "1",
                },
                manifest["evaluation"]["effective_child_environment"],
            )

            with self.assertRaisesRegex(RuntimeError, "output already exists"):
                runner.run(args)

            (published / "unexpected.txt").write_text("drift", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unexpected receipt files"):
                runner.verify_artifacts(published)

    def test_independent_verifier_rejects_summary_tampering(self) -> None:
        runner = self.load_runner()
        verifier = self.load_verifier()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table3-sketch-verifier-test-", dir=runtime_root
        ) as temporary:
            output = Path(temporary) / "published"
            args = runner.parse_args(
                [
                    "--mode",
                    "smoke",
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
            summary_path = published / "summary.json"
            summary = json.loads(summary_path.read_text("utf-8"))
            summary["j_eval"] += 1
            summary_path.write_text(json.dumps(summary), encoding="utf-8")

            receipt = verifier.verify_output(published, write_receipt=False)

            self.assertEqual("FAIL", receipt["status"])
            self.assertFalse(receipt["checks"]["summary_matches_reaggregation"])

    def test_verifier_formal_runtime_and_provenance_contract(self) -> None:
        verifier = self.load_verifier()
        smoke = (
            REPO
            / "exp/runtime/urdf_table3_sketch_mobility_smoke_n5_20260821T060959Z"
        ).resolve(strict=True)
        manifest = json.loads((smoke / "manifest.json").read_text("utf-8"))

        verifier.validate_formal_runtime_environment(
            manifest["evaluation"]["environment"]
        )
        parsed = verifier.parse_args(["--output-root", str(smoke)])
        self.assertFalse(parsed.write_receipt)
        frozen_verifier = Path(
            manifest["evaluation"]["source_snapshots"]["independent_verifier"][
                "path"
            ]
        )
        with mock.patch.object(verifier, "SCRIPT_PATH", frozen_verifier):
            verifier._validate_sources(smoke, manifest)
            manifest["evaluation"]["adapter_sha256"] = "0" * 64
            with self.assertRaisesRegex(ValueError, "flat source binding mismatch"):
                verifier._validate_sources(smoke, manifest)

    def test_resume_reuses_hash_bound_journal_records(self) -> None:
        runner = self.load_runner()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table3-sketch-resume-test-", dir=runtime_root
        ) as temporary:
            output = Path(temporary) / "published"
            args = runner.parse_args(
                [
                    "--mode",
                    "smoke",
                    "--limit",
                    "2",
                    "--workers",
                    "1",
                    "--output",
                    str(output),
                ]
            )
            original = runner._execute_jobs

            def interrupt_after_first(*call_args, **call_kwargs):
                call_kwargs["stop_after"] = 1
                return original(*call_args, **call_kwargs)

            with mock.patch.object(runner, "_execute_jobs", interrupt_after_first):
                with self.assertRaisesRegex(KeyboardInterrupt, "test interruption"):
                    runner.run(args)

            self.assertFalse(output.exists())
            resume_args = runner.parse_args(
                [
                    "--mode",
                    "smoke",
                    "--limit",
                    "2",
                    "--workers",
                    "1",
                    "--resume",
                    "--output",
                    str(output),
                ]
            )
            runner.run(resume_args)
            records = [
                json.loads(line)
                for line in (output / "asset_records.jsonl").read_text("utf-8").splitlines()
            ]
            self.assertEqual([1, 2], [record["selection_rank"] for record in records])
            self.assertEqual(2, len(records))


if __name__ == "__main__":
    unittest.main()
