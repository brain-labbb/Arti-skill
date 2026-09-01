#!/usr/bin/env python3
"""Behavior tests for the SketchMobility Table 4 adapter."""

from __future__ import annotations

import importlib.util
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest import mock


REPO = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO / "exp/scripts/run_table4_urdf_sketch_mobility.py"
VERIFIER_PATH = REPO / "exp/scripts/verify_table4_urdf_sketch_mobility.py"


class SketchMobilityTable4AdapterTests(unittest.TestCase):
    def load_runner(self):
        if not RUNNER_PATH.is_file():
            self.fail(f"SketchMobility Table 4 runner is missing: {RUNNER_PATH}")
        spec = importlib.util.spec_from_file_location(
            "run_table4_urdf_sketch_mobility_test_target", RUNNER_PATH
        )
        if spec is None or spec.loader is None:
            self.fail(f"cannot load runner: {RUNNER_PATH}")
        runner = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = runner
        spec.loader.exec_module(runner)
        return runner

    def load_verifier(self):
        if not VERIFIER_PATH.is_file():
            self.fail(f"SketchMobility Table 4 verifier is missing: {VERIFIER_PATH}")
        spec = importlib.util.spec_from_file_location(
            "verify_table4_urdf_sketch_mobility_test_target", VERIFIER_PATH
        )
        if spec is None or spec.loader is None:
            self.fail(f"cannot load verifier: {VERIFIER_PATH}")
        verifier = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = verifier
        spec.loader.exec_module(verifier)
        return verifier

    def test_loads_exact_table3_cohort_and_rejects_record_drift(self) -> None:
        runner = self.load_runner()
        loaded = runner.load_frozen_cohort(
            runner.DEFAULT_TABLE3_RECEIPT,
            runner.DEFAULT_TABLE2_MANIFEST,
            runner.DEFAULT_TABLE1_RECEIPT,
            runner.DEFAULT_DATASET_ROOT,
            formal=True,
        )

        self.assertEqual(800, len(loaded["rows"]))
        self.assertEqual(1824, loaded["declared_joint_count"])
        self.assertEqual(
            "data/Shape2Motion/Kettle/kettle_0057",
            loaded["rows"][0]["asset_id"],
        )
        self.assertEqual(
            "data/Shape2Motion/Toilet/toilet_0117",
            loaded["rows"][-1]["asset_id"],
        )

        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table4-sketch-cohort-drift-", dir=runtime_root
        ) as temporary:
            altered = Path(temporary) / "table3"
            altered.mkdir()
            shutil.copy2(runner.DEFAULT_TABLE3_RECEIPT / "manifest.json", altered)
            rows = [
                json.loads(line)
                for line in (runner.DEFAULT_TABLE3_RECEIPT / "asset_records.jsonl")
                .read_text("utf-8")
                .splitlines()
            ]
            rows[0]["urdf_sha256"] = "0" * 64
            (altered / "asset_records.jsonl").write_text(
                "".join(
                    json.dumps(row, sort_keys=True, ensure_ascii=True) + "\n"
                    for row in rows
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(ValueError, "Table 3 record binding mismatch"):
                runner.load_frozen_cohort(
                    altered,
                    runner.DEFAULT_TABLE2_MANIFEST,
                    runner.DEFAULT_TABLE1_RECEIPT,
                    runner.DEFAULT_DATASET_ROOT,
                    formal=False,
                )

    def test_audit_binds_mobility_urdf_collision_inventory_and_full_package(self) -> None:
        runner = self.load_runner()
        loaded = runner.load_frozen_cohort(
            runner.DEFAULT_TABLE3_RECEIPT,
            runner.DEFAULT_TABLE2_MANIFEST,
            runner.DEFAULT_TABLE1_RECEIPT,
            runner.DEFAULT_DATASET_ROOT,
            formal=True,
        )

        item = runner.audit_asset(
            loaded["rows"][0], runner.DEFAULT_DATASET_ROOT
        )

        self.assertEqual("mobility.urdf", item["primary_urdf_relative_path"])
        self.assertEqual(
            "150fb5b16442ad363223d045fcddfa385d1d164851c6f37602a1c5cb64602711",
            item["urdf_sha256"],
        )
        self.assertEqual(
            "b1e55aa48e8120a9e94e82d4400881054adf749c2cbcf09b7dcb7a0d301c1eae",
            item["package_content_manifest_sha256"],
        )
        self.assertEqual(1, item["movable_dof_count"])
        self.assertGreaterEqual(len(item["collision_mesh_files"]), 2)

    def test_formal_runtime_rejects_environment_or_core_drift(self) -> None:
        runner = self.load_runner()
        identity = runner.current_runtime_identity()
        runner.validate_formal_runtime(identity)

        for key, wrong in (
            ("python_version", "3.13.0"),
            ("pybullet_version", "3.2.6"),
            ("numpy_version", "2.4.0"),
            ("scipy_version", "1.17.0"),
            ("collision_core_sha256", "0" * 64),
        ):
            changed = dict(identity)
            changed[key] = wrong
            with self.subTest(key=key), self.assertRaisesRegex(
                ValueError, "formal runtime mismatch"
            ):
                runner.validate_formal_runtime(changed)

    def test_smoke_publishes_write_once_semantically_closed_receipt(self) -> None:
        runner = self.load_runner()
        verifier = self.load_verifier()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table4-sketch-publication-", dir=runtime_root
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
                    "--audit-workers",
                    "1",
                    "--output",
                    str(output),
                ]
            )

            result = runner.run(args)

            published = output.resolve(strict=True)
            self.assertEqual("PASS", result["verification"]["status"])
            self.assertEqual(1, result["summary"]["n_eval"])
            self.assertEqual(1, result["summary"]["cohort"]["measurement_complete"])
            self.assertTrue((published / "state_records.jsonl").is_file())
            runner.verify_artifacts(published)
            receipt = verifier.verify_output(published, write_receipt=False)
            self.assertEqual("PASS", receipt["status"])
            self.assertTrue(all(receipt["checks"].values()))

            with self.assertRaisesRegex(RuntimeError, "output already exists"):
                runner.run(args)

            (published / "unexpected.txt").write_text("drift", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unexpected receipt files"):
                runner.verify_artifacts(published)
            (published / "unexpected.txt").unlink()
            (published / "dangling-link").symlink_to("missing-target")
            with self.assertRaisesRegex(ValueError, "symlink"):
                runner.verify_artifacts(published)

    def test_independent_verifier_rejects_summary_tampering(self) -> None:
        runner = self.load_runner()
        verifier = self.load_verifier()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table4-sketch-verifier-", dir=runtime_root
        ) as temporary:
            output = Path(temporary) / "published"
            runner.run(
                runner.parse_args(
                    [
                        "--mode",
                        "smoke",
                        "--limit",
                        "1",
                        "--workers",
                        "1",
                        "--audit-workers",
                        "1",
                        "--output",
                        str(output),
                    ]
                )
            )
            published = output.resolve(strict=True)
            summary_path = published / "summary.json"
            summary = json.loads(summary_path.read_text("utf-8"))
            summary["metrics"]["strict_collision_pass"]["passed"] += 1
            summary_path.write_text(json.dumps(summary), encoding="utf-8")

            receipt = verifier.verify_output(published, write_receipt=False)

            self.assertEqual("FAIL", receipt["status"])
            self.assertFalse(receipt["checks"]["summary_matches_reaggregation"])

    def test_resume_reuses_hash_bound_journal_record(self) -> None:
        runner = self.load_runner()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table4-sketch-resume-", dir=runtime_root
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
                    "--audit-workers",
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
            resume = runner.parse_args(
                [
                    "--mode",
                    "smoke",
                    "--limit",
                    "2",
                    "--workers",
                    "1",
                    "--audit-workers",
                    "1",
                    "--resume",
                    "--output",
                    str(output),
                ]
            )
            result = runner.run(resume)
            self.assertEqual(2, result["summary"]["n_eval"])
            records = [
                json.loads(line)
                for line in (output / "asset_records.jsonl")
                .read_text("utf-8")
                .splitlines()
            ]
            self.assertEqual([1, 2], [row["selection_rank"] for row in records])

    def test_consistently_rehashed_fabricated_state_identity_is_rejected(self) -> None:
        runner = self.load_runner()
        verifier = self.load_verifier()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table4-sketch-state-tamper-", dir=runtime_root
        ) as temporary:
            output = Path(temporary) / "published"
            runner.run(
                runner.parse_args(
                    [
                        "--mode",
                        "smoke",
                        "--limit",
                        "1",
                        "--workers",
                        "1",
                        "--audit-workers",
                        "1",
                        "--output",
                        str(output),
                    ]
                )
            )
            published = output.resolve(strict=True)
            states_path = published / "state_records.jsonl"
            states = [json.loads(line) for line in states_path.read_text("utf-8").splitlines()]
            self.assertGreater(len(states), 1)
            states[1]["joint_values_sha256"] = "0" * 64
            states_path.write_text(
                "".join(json.dumps(row, sort_keys=True) + "\n" for row in states),
                encoding="utf-8",
            )
            records_path = published / "asset_records.jsonl"
            records = [json.loads(line) for line in records_path.read_text("utf-8").splitlines()]
            payload = json.dumps(
                states, sort_keys=True, separators=(",", ":"), ensure_ascii=True
            ).encode("utf-8")
            records[0]["state_records_sha256"] = hashlib.sha256(payload).hexdigest()
            records_path.write_text(
                json.dumps(records[0], sort_keys=True) + "\n", encoding="utf-8"
            )

            receipt = verifier.verify_output(published, write_receipt=False)

            self.assertEqual("FAIL", receipt["status"])
            self.assertFalse(receipt["checks"]["record_and_state_semantic_closure"])

    def test_formal_contract_freezes_timeout_and_requires_passing_smoke(self) -> None:
        runner = self.load_runner()
        output = REPO / "exp/runtime/formal-contract-test-does-not-run"
        missing_smoke = REPO / "exp/runtime/nonexistent-table4-smoke"
        changed_timeout = runner.parse_args(
            [
                "--mode",
                "formal",
                "--output",
                str(output),
                "--child-timeout-seconds",
                "1",
                "--smoke-receipt",
                str(missing_smoke),
            ]
        )
        with self.assertRaisesRegex(ValueError, "frozen formal configuration"):
            runner.validate_contract(changed_timeout)

        no_smoke = runner.parse_args(["--mode", "formal", "--output", str(output)])
        with self.assertRaisesRegex(ValueError, "passing smoke receipt"):
            runner.validate_contract(no_smoke)

    def test_formal_gate_replays_frozen_verifier_on_smoke_receipt(self) -> None:
        runner = self.load_runner()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table4-sketch-smoke-gate-", dir=runtime_root
        ) as temporary:
            output = Path(temporary) / "published"
            runner.run(
                runner.parse_args(
                    [
                        "--mode",
                        "smoke",
                        "--limit",
                        "5",
                        "--workers",
                        "1",
                        "--audit-workers",
                        "1",
                        "--child-timeout-seconds",
                        "900",
                        "--output",
                        str(output),
                    ]
                )
            )
            published = output.resolve(strict=True)
            summary_path = published / "summary.json"
            summary = json.loads(summary_path.read_text("utf-8"))
            summary["n_eval"] = 999
            summary_path.write_text(
                json.dumps(summary, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            runner._write_artifact_manifest(published)

            with self.assertRaisesRegex(ValueError, "smoke receipt semantic"):
                runner.smoke_receipt_binding(published)

    def test_source_pin_rejects_drift_and_resume_uses_only_snapshots(self) -> None:
        runner = self.load_runner()
        runner.validate_source_pins(runner.DEFAULT_SOURCE_PINS)
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table4-sketch-source-pin-", dir=runtime_root
        ) as temporary:
            temporary_root = Path(temporary)
            altered_pin = temporary_root / "source_pins.json"
            pin = json.loads(runner.DEFAULT_SOURCE_PINS.read_text("utf-8"))
            pin["sources"]["adapter"] = "0" * 64
            altered_pin.write_text(json.dumps(pin), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "source pin mismatch"):
                runner.validate_source_pins(altered_pin)

            output = temporary_root / "snapshot"
            output.mkdir()
            declared = runner.freeze_source_snapshots(output)
            drifted = temporary_root / runner.SCRIPT_PATH.name
            drifted.write_text("drift", encoding="utf-8")
            live_paths = runner._source_paths()
            live_paths["adapter"] = drifted
            with mock.patch.object(runner, "_source_paths", return_value=live_paths):
                runner.validate_source_snapshots(
                    output, declared, require_live_match=False
                )


if __name__ == "__main__":
    unittest.main()
