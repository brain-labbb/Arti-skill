#!/usr/bin/env python3
"""Behavior tests for the SketchMobility Table 2 supplementary adapter."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest import mock


REPO = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO / "exp/scripts/run_table2sup_urdf_sketch_mobility.py"
VERIFIER_PATH = REPO / "exp/scripts/verify_table2sup_urdf_sketch_mobility.py"


def load_module(path: Path, name: str):
    if not path.is_file():
        raise AssertionError(f"required module is missing: {path}")
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"cannot load module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class SketchMobilityTable2SupplementaryTests(unittest.TestCase):
    def load_runner(self):
        return load_module(RUNNER_PATH, "table2sup_sketch_runner_test_target")

    def load_verifier(self):
        return load_module(VERIFIER_PATH, "table2sup_sketch_verifier_test_target")

    def test_loads_exact_frozen_intent_and_builds_mobility_job(self) -> None:
        runner = self.load_runner()

        cohort = runner.load_formal_cohort(formal=True)
        jobs = runner.build_jobs(cohort["rows"])

        self.assertEqual(800, len(jobs))
        self.assertEqual(1824, sum(job["expected_movable_joints"] for job in jobs))
        self.assertEqual(0, jobs[0]["selection_index"])
        self.assertEqual(1, jobs[0]["selection_rank"])
        self.assertEqual(
            "data/Shape2Motion/Kettle/kettle_0057", jobs[0]["asset_id"]
        )
        self.assertEqual("mobility.urdf", jobs[0]["urdf_relative_path"])
        self.assertEqual(
            "b1e55aa48e8120a9e94e82d4400881054adf749c2cbcf09b7dcb7a0d301c1eae",
            jobs[0]["expected_package_content_manifest_sha256"],
        )

    def test_pins_current_static_evaluator_source(self) -> None:
        runner = self.load_runner()

        identity = runner.current_evaluator_identity()

        self.assertEqual("lam-supplementary-static/v1.2", identity["protocol_version"])
        self.assertEqual(
            "4701415dad8a5c0a434c16887979bcb70c250ba0b25772014e8db73789098e5f",
            identity["static_module_sha256"],
        )

    def test_live_package_drift_invalidates_run_before_children(self) -> None:
        runner = self.load_runner()
        job = runner.build_jobs(runner.load_formal_cohort(formal=True)["rows"])[0]
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table2sup-sketch-input-drift-", dir=runtime_root
        ) as temporary:
            package = Path(temporary) / "package"
            shutil.copytree(Path(job["package"]), package)
            (package / "drift.txt").write_text("drift", encoding="utf-8")
            changed = dict(job)
            changed["package"] = str(package)

            self.assertTrue(
                hasattr(runner, "validate_live_inputs"),
                "run-level live input validation is missing",
            )
            with self.assertRaisesRegex(ValueError, "package binding drift"):
                runner.validate_live_inputs([changed], workers=1)

    def test_atom_recompute_binds_category_authority(self) -> None:
        runner = self.load_runner()
        verifier = self.load_verifier()
        item = runner.build_jobs(runner.load_formal_cohort(formal=True)["rows"])[0]
        record = runner.atoms.audit_partnet_mobility_asset(item)
        record["package_content_manifest_sha256"] = item[
            "expected_package_content_manifest_sha256"
        ]
        self.assertTrue(verifier._recompute_record_atoms(runner.atoms, item, record))
        record["category"] = "forged/category"

        self.assertFalse(verifier._recompute_record_atoms(runner.atoms, item, record))

    def test_parent_runtime_failure_cannot_be_published_without_external_anchor(self) -> None:
        runner = self.load_runner()
        verifier = self.load_verifier()
        item = runner.build_jobs(runner.load_formal_cohort(formal=True)["rows"])[0]
        self.assertTrue(
            hasattr(runner, "_parent_failure_record"),
            "parent runtime failure attestation is missing",
        )
        snapshots = runner.current_source_hashes()
        record = runner._parent_failure_record(
            item,
            manifest_hash="a" * 64,
            source_snapshots=snapshots,
            failure_kind="timeout",
            returncode=None,
        )

        self.assertFalse(
            verifier._verifiable_runtime_failure(
                item,
                record,
                snapshots=snapshots,
                manifest_hash="a" * 64,
            )
        )
        self.assertEqual(
            item["expected_movable_joints"],
            record["table2_supplementary"]["joint_limit_portability"][
                "joints_intended"
            ],
        )

    def test_parent_runtime_failure_blocks_publish_and_requires_resume(self) -> None:
        runner = self.load_runner()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table2sup-sketch-runtime-failure-", dir=runtime_root
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

            def fail_child(job, work, manifest_hash, frozen_runner, snapshots):
                del frozen_runner
                record = runner._parent_failure_record(
                    job,
                    manifest_hash=manifest_hash,
                    source_snapshots=snapshots,
                    failure_kind="timeout",
                    returncode=None,
                )
                runner.atomic_write_json(
                    work / "children/rank_0001.json", record
                )
                return record

            with mock.patch.object(runner, "_execute_job", fail_child):
                with self.assertRaisesRegex(RuntimeError, "rerun with --resume"):
                    runner.run(args)

            self.assertFalse(output.exists())
            work = output.with_name(f".{output.name}.work")
            checkpoint = json.loads(
                (work / "checkpoint.json").read_text(encoding="utf-8")
            )
            self.assertEqual("runtime_failures_require_resume", checkpoint["state"])
            manifest = json.loads((work / "manifest.json").read_text("utf-8"))
            self.assertIsNone(
                runner._load_resume_record(
                    work / "children/rank_0001.json",
                    manifest["items"][0],
                    manifest["manifest_content_sha256"],
                    manifest["source_snapshots"][
                        "exp/scripts/run_table2sup_urdf_sketch_mobility.py"
                    ],
                )
            )

    def test_formal_contract_requires_exact_passing_n5_smoke(self) -> None:
        runner = self.load_runner()
        self.assertTrue(
            hasattr(runner, "validate_contract"),
            "formal validation contract is missing",
        )
        args = runner.parse_args(
            ["--mode", "formal", "--output", "/tmp/not-created"]
        )

        with self.assertRaisesRegex(ValueError, "smoke receipt"):
            runner.validate_contract(args)

    def test_smoke_writes_verified_fail_closed_receipt(self) -> None:
        runner = self.load_runner()
        verifier = self.load_verifier()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table2sup-sketch-smoke-", dir=runtime_root
        ) as temporary:
            output = Path(temporary) / "published"
            result = runner.run(
                runner.parse_args(
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
            )

            self.assertEqual(1, result["summary"]["n_eval"])
            self.assertEqual(1, result["summary"]["j_eval"])
            self.assertEqual(
                "N/E",
                result["summary"]["metrics"]["placeholder_mass_incidence"]["status"],
            )
            self.assertEqual("PASS", result["verification"]["status"])
            replay = verifier.verify_output(output, write_receipt=False)
            self.assertEqual("PASS", replay["status"])
            self.assertTrue(all(replay["checks"].values()))
            self.assertTrue(
                (output / "receipt_digest.json").is_file(),
                "whole-receipt digest is missing",
            )
            before_replay = {
                path.relative_to(output).as_posix(): runner.sha256_file(path)
                for path in output.rglob("*")
                if path.is_file()
            }
            verifier.verify_output(output, write_receipt=False)
            after_replay = {
                path.relative_to(output).as_posix(): runner.sha256_file(path)
                for path in output.rglob("*")
                if path.is_file()
            }
            self.assertEqual(before_replay, after_replay)
            manifest = json.loads((output / "manifest.json").read_text("utf-8"))
            record = json.loads(
                (output / "asset_records.jsonl").read_text("utf-8").splitlines()[0]
            )
            frozen_runner_key = (
                "exp/scripts/run_table2sup_urdf_sketch_mobility.py"
            )
            self.assertIn(
                frozen_runner_key,
                manifest["source_snapshots"],
                "receipt does not bind an executable frozen runner",
            )
            self.assertEqual(
                manifest["source_snapshots"][frozen_runner_key],
                record["child"]["executed_runner_sha256"],
            )
            self.assertIn(
                "executed_source_snapshots",
                record["child"],
                "child dependency source attestation is missing",
            )
            self.assertEqual(
                manifest["source_snapshots"],
                record["child"]["executed_source_snapshots"],
            )
            for required in (
                "table2_manifest_sha256",
                "table2_records_sha256",
                "table4_asset_records_sha256",
                "table4_state_records_sha256",
            ):
                self.assertIn(required, manifest["source"])
            self.assertTrue(
                hasattr(verifier, "_verify_formal_smoke_binding"),
                "standalone formal smoke verification is missing",
            )
            frozen_verifier = (
                output
                / "source_snapshots/exp/scripts/verify_table2sup_urdf_sketch_mobility.py"
            )
            receipt_digest = json.loads(
                (output / "receipt_digest.json").read_text("utf-8")
            )
            valid_n1_binding = {
                "path": str(output),
                "manifest_sha256": runner.sha256_file(output / "manifest.json"),
                "summary_sha256": runner.sha256_file(output / "summary.json"),
                "asset_records_sha256": runner.sha256_file(
                    output / "asset_records.jsonl"
                ),
                "artifact_manifest_sha256": runner.sha256_file(
                    output / "artifact_manifest.json"
                ),
                "verification_sha256": runner.sha256_file(
                    output / "verification.json"
                ),
                "receipt_digest_sha256": runner.sha256_file(
                    output / "receipt_digest.json"
                ),
                "receipt_tree_sha256": receipt_digest["tree_sha256"],
                "frozen_verifier_sha256": runner.sha256_file(frozen_verifier),
                "source_snapshots": manifest["source_snapshots"],
            }
            self.assertFalse(
                verifier._verify_formal_smoke_binding(
                    valid_n1_binding,
                    snapshots=manifest["source_snapshots"],
                ),
                "formal verifier accepted a valid but non-N=5 smoke receipt",
            )
            self.assertFalse(
                verifier._verify_formal_smoke_binding(
                    {
                        "path": str(output),
                        "manifest_sha256": "0" * 64,
                        "summary_sha256": "0" * 64,
                        "asset_records_sha256": "0" * 64,
                        "artifact_manifest_sha256": "0" * 64,
                        "verification_sha256": "0" * 64,
                        "receipt_digest_sha256": "0" * 64,
                        "receipt_tree_sha256": "0" * 64,
                        "source_snapshots": manifest["source_snapshots"],
                    },
                    snapshots=manifest["source_snapshots"],
                )
            )

            with self.assertRaisesRegex(ValueError, "N=5|configuration"):
                runner.smoke_receipt_binding(output)

            with self.assertRaisesRegex(RuntimeError, "output already exists"):
                runner.run(
                    runner.parse_args(
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
                )
            (output / "receipt_digest.json").unlink()
            missing_digest = verifier.verify_output(output, write_receipt=False)
            self.assertEqual("FAIL", missing_digest["status"])
            self.assertFalse(missing_digest["checks"]["whole_receipt_digest"])

    def test_independent_verifier_rejects_summary_tampering(self) -> None:
        runner = self.load_runner()
        verifier = self.load_verifier()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table2sup-sketch-tamper-", dir=runtime_root
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
                        "--output",
                        str(output),
                    ]
                )
            )
            summary_path = output / "summary.json"
            summary = json.loads(summary_path.read_text("utf-8"))
            summary["metrics"]["visual_bearing_collision_coverage"]["passed"] += 1
            summary_path.write_text(json.dumps(summary), encoding="utf-8")

            replay = verifier.verify_output(output, write_receipt=False)

            self.assertEqual("FAIL", replay["status"])
            self.assertFalse(replay["checks"]["summary_matches_reaggregation"])

    def test_verifier_rejects_fail_closed_denominator_shrink_with_fresh_hashes(self) -> None:
        runner = self.load_runner()
        verifier = self.load_verifier()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table2sup-sketch-denominator-tamper-", dir=runtime_root
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
                        "--output",
                        str(output),
                    ]
                )
            )
            records_path = output / "asset_records.jsonl"
            record = json.loads(records_path.read_text("utf-8").splitlines()[0])
            portability = record["table2_supplementary"]["joint_limit_portability"]
            dynamics = record["table2_supplementary"]["joint_dynamics_coverage"]
            portability.update(
                {
                    "joints_intended": 0,
                    "joints_extracted": 0,
                    "joints_passed": 0,
                    "joint_records": [],
                }
            )
            dynamics.update(
                {
                    "joints_intended": 0,
                    "joints_extracted": 0,
                    "joints_covered": 0,
                    "joint_records": [],
                }
            )
            records_path.write_text(
                json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n",
                encoding="utf-8",
            )
            summary_path = output / "summary.json"
            summary = json.loads(summary_path.read_text("utf-8"))
            aggregate = runner.aggregate_records([record])
            for key, value in aggregate.items():
                summary[key] = value
            summary_path.write_text(
                json.dumps(summary, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            artifact_path = output / "artifact_manifest.json"
            artifact = json.loads(artifact_path.read_text("utf-8"))
            for entry in artifact["files"]:
                if entry["path"] in {"asset_records.jsonl", "summary.json"}:
                    changed = output / entry["path"]
                    entry["bytes"] = changed.stat().st_size
                    entry["sha256"] = runner.sha256_file(changed)
            artifact_path.write_text(
                json.dumps(artifact, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )

            replay = verifier.verify_output(output, write_receipt=False)

            self.assertEqual("FAIL", replay["status"])
            self.assertFalse(replay["checks"]["record_atoms_recomputed"])

            manifest = json.loads((output / "manifest.json").read_text("utf-8"))
            forged = runner._failed_record(manifest["items"][0], "forged_runtime_error")
            forged["manifest_content_sha256"] = manifest["manifest_content_sha256"]
            forged["child"] = record["child"]
            records_path.write_text(
                json.dumps(forged, sort_keys=True, separators=(",", ":")) + "\n",
                encoding="utf-8",
            )
            summary = json.loads(summary_path.read_text("utf-8"))
            for key, value in runner.aggregate_records([forged]).items():
                summary[key] = value
            summary_path.write_text(
                json.dumps(summary, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            artifact = json.loads(artifact_path.read_text("utf-8"))
            for entry in artifact["files"]:
                if entry["path"] in {"asset_records.jsonl", "summary.json"}:
                    changed = output / entry["path"]
                    entry["bytes"] = changed.stat().st_size
                    entry["sha256"] = runner.sha256_file(changed)
            artifact_path.write_text(
                json.dumps(artifact, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            (output / "receipt_digest.json").write_text(
                json.dumps(runner._receipt_digest(output), indent=2, sort_keys=True)
                + "\n",
                encoding="utf-8",
            )

            forged_replay = verifier.verify_output(output, write_receipt=False)

            self.assertEqual("FAIL", forged_replay["status"])
            self.assertFalse(forged_replay["checks"]["record_atoms_recomputed"])

    def test_verifier_rejects_snapshot_tamper_even_with_updated_artifact_hash(self) -> None:
        runner = self.load_runner()
        verifier = self.load_verifier()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table2sup-sketch-source-tamper-", dir=runtime_root
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
                        "--output",
                        str(output),
                    ]
                )
            )
            snapshot = (
                output
                / "source_snapshots/exp/scripts/sketchmobility_supplementary_common.py"
            )
            snapshot.write_text(snapshot.read_text("utf-8") + "\n# tampered\n", encoding="utf-8")
            artifact_path = output / "artifact_manifest.json"
            artifact = json.loads(artifact_path.read_text("utf-8"))
            entry = next(
                row
                for row in artifact["files"]
                if row["path"]
                == "source_snapshots/exp/scripts/sketchmobility_supplementary_common.py"
            )
            entry["bytes"] = snapshot.stat().st_size
            entry["sha256"] = runner.sha256_file(snapshot)
            artifact_path.write_text(
                json.dumps(artifact, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )

            replay = verifier.verify_output(output, write_receipt=False)

            self.assertEqual("FAIL", replay["status"])
            self.assertFalse(replay["checks"]["source_snapshot_bindings"])

    def test_interrupted_smoke_strictly_resumes_hash_bound_journal(self) -> None:
        runner = self.load_runner()
        runtime_root = REPO / "exp/runtime"
        with tempfile.TemporaryDirectory(
            prefix="table2sup-sketch-resume-", dir=runtime_root
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
            self.assertTrue(hasattr(args, "resume"), "strict resume option is missing")
            original = runner._execute_job

            def interrupt_rank_two(job, *call_args, **call_kwargs):
                if int(job["selection_rank"]) == 2:
                    raise KeyboardInterrupt("test interruption")
                return original(job, *call_args, **call_kwargs)

            with mock.patch.object(runner, "_execute_job", interrupt_rank_two):
                with self.assertRaisesRegex(KeyboardInterrupt, "test interruption"):
                    runner.run(args)
            work = output.with_name(f".{output.name}.work")
            first = work / "children/rank_0001.json"
            self.assertTrue(first.is_file())
            first_hash = runner.sha256_file(first)

            resume = runner.parse_args(
                [
                    "--mode",
                    "smoke",
                    "--limit",
                    "2",
                    "--workers",
                    "1",
                    "--output",
                    str(output),
                    "--resume",
                ]
            )
            result = runner.run(resume)

            self.assertEqual(2, result["summary"]["n_eval"])
            self.assertEqual(first_hash, runner.sha256_file(output / "children/rank_0001.json"))


if __name__ == "__main__":
    unittest.main()
