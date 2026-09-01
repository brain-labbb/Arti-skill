#!/usr/bin/env python3
"""Focused contract tests for the shared Table 5 N=200 runtime."""

from __future__ import annotations

import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import textwrap
import threading
import time
import unittest
from unittest import mock


REPO = Path(__file__).resolve().parents[2]
RUNNER = REPO / "exp/scripts/table5_n200_runtime.py"


def load_runner():
    name = "table5_n200_runtime_test_target"
    spec = importlib.util.spec_from_file_location(name, RUNNER)
    if spec is None or spec.loader is None:
        raise AssertionError(f"cannot load {RUNNER}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


runtime = load_runner()


def protocol() -> dict:
    value = {
        "runtime": {
            "gravity_m_per_s2": [0.0, 0.0, -9.81],
            "timestep_s": {"numerator": 1, "denominator": 240},
            "solver_iterations": 50,
            "reset_repetitions": 3,
            "passive_settling": {"steps": 240},
            "actuation": {"trajectory": {}},
            "limit_enforcement": {},
            "thread_caps": {},
            "child_timeout_s": 300,
        },
        "metrics": {"simulator_pass": {"logical_and": list(runtime.METRIC_NAMES[:-1])}},
        "cross_simulator": {},
        "adapters": {
            "pybullet": {},
            "mujoco": {},
            "genesis": {},
        },
    }
    value["protocol_sha256"] = runtime.canonical_sha256(
        value, exclude_fields=("protocol_sha256",)
    )
    return value


def metric_result(value: bool = True) -> dict:
    return {
        "metrics": {name: value for name in runtime.METRIC_NAMES},
        "diagnostics": {"raw_marker": "preserved"},
    }


class RuntimeFixture:
    def __init__(self, root: Path, *, slug: str = "articraft-10k") -> None:
        self.root = root
        self.slug = slug
        self.urdf = root / slug / "asset-0000" / "model.urdf"
        self.urdf.parent.mkdir(parents=True)
        self.urdf.write_text("<robot name='fixture'/>", encoding="utf-8")
        self.row = {
            "asset_id": f"{slug}/source-0000",
            "dataset_id": "asset-0000",
            "category": "fixture",
            "package_root": str(self.urdf.parent),
            "urdf_path": str(self.urdf),
            "urdf_sha256": runtime.sha256_file(self.urdf),
            "source_parent": None,
            "joint_tree": {"root_links": ["base"], "joints": []},
            "scalar_joints": [],
            "bounding_box_diagonal": 1.0,
            "preflight": {
                "status": "pass",
                "issues": [],
                "simulator_eligible": True,
            },
            "strict_gates": {"table2": None, "table3": None, "table4": None},
        }
        self.manifest = root / "manifest.json"
        self.write_manifest()

    def write_manifest(self, *, groups: list[dict] | None = None) -> None:
        if groups is None:
            groups = [
                {
                    "dataset_slug": self.slug,
                    "dataset_name": self.slug,
                    "rows": [self.row],
                }
            ]
        groups = json.loads(json.dumps(groups))
        for group in groups:
            for row in group["rows"]:
                row["row_sha256"] = runtime.canonical_sha256(
                    row, exclude_fields=("row_sha256",)
                )
        frozen_protocol = protocol()
        value = {
            "schema_version": "table5_n200_manifest_v1",
            "ordered_dataset_slugs": [group["dataset_slug"] for group in groups],
            "datasets": groups,
            "protocol": frozen_protocol,
            "protocol_sha256": frozen_protocol["protocol_sha256"],
        }
        value["manifest_sha256"] = runtime.canonical_sha256(
            value, exclude_fields=("manifest_sha256",)
        )
        self.manifest.write_text(json.dumps(value), encoding="utf-8")


class Table5N200RuntimeTests(unittest.TestCase):
    def test_loads_six_grouped_datasets_in_frozen_order(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = RuntimeFixture(root)
            slugs = (
                "articraft-10k",
                "lam",
                "artiverse",
                "partnet-mobility",
                "physx-mobility",
                "sketchmobility",
            )
            groups = []
            for index, slug in enumerate(slugs):
                urdf = root / slug / "model.urdf"
                urdf.parent.mkdir(parents=True, exist_ok=True)
                urdf.write_text(f"<robot name='{slug}'/>", encoding="utf-8")
                row = dict(fixture.row)
                row.update(
                    {
                        "asset_id": f"source/{slug}",
                        "dataset_id": f"asset-{index:04d}",
                        "package_root": str(urdf.parent),
                        "urdf_path": str(urdf),
                        "urdf_sha256": runtime.sha256_file(urdf),
                    }
                )
                groups.append(
                    {"dataset_slug": slug, "dataset_name": slug, "rows": [row]}
                )
            fixture.write_manifest(groups=groups)

            loaded = runtime.load_manifest(fixture.manifest)

            self.assertEqual(slugs, tuple(dataset.slug for dataset in loaded.datasets))
            self.assertEqual(
                "physx-mobility", loaded.datasets[4].rows[0]["dataset_slug"]
            )
            self.assertEqual(protocol(), loaded.protocol)

    def test_success_is_atomic_at_required_path_and_preserves_evaluation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = RuntimeFixture(root)
            calls = []

            def launch(**kwargs):
                calls.append(kwargs)
                return runtime.WorkerOutcome(
                    returncode=0,
                    duration_s=0.25,
                    response=metric_result(),
                    command=[kwargs["executable"], "worker"],
                )

            output = root / "runtime"
            summary = runtime.run_manifest(
                fixture.manifest,
                output,
                simulators=("pybullet",),
                executables={"pybullet": "/opt/pybullet/bin/python"},
                launcher=launch,
            )

            path = output / fixture.slug / "pybullet/assets/asset-0000.json"
            record = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(runtime.ASSET_SCHEMA, record["schema_version"])
            self.assertEqual("completed", record["terminal_status"])
            self.assertEqual(metric_result(), record["evaluation"])
            self.assertTrue(all(record["metrics"].values()))
            self.assertEqual(1, summary["terminal_count"])
            self.assertEqual("/opt/pybullet/bin/python", calls[0]["executable"])
            self.assertEqual(protocol(), calls[0]["request"]["protocol"])
            self.assertFalse(list(path.parent.glob("*.tmp")))

    def test_preflight_failure_is_terminal_and_never_launches(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = RuntimeFixture(root)
            fixture.row["preflight"] = {
                "status": "failed",
                "issues": ["invalid_xml"],
                "simulator_eligible": False,
            }
            fixture.row["joint_tree"] = None
            fixture.write_manifest()

            def forbidden(**_kwargs):
                raise AssertionError("failed preflight must not launch a simulator")

            output = root / "runtime"
            runtime.run_manifest(
                fixture.manifest,
                output,
                simulators=("mujoco",),
                launcher=forbidden,
            )

            record = json.loads(
                (output / fixture.slug / "mujoco/assets/asset-0000.json").read_text()
            )
            self.assertEqual("preflight_failure", record["terminal_status"])
            self.assertFalse(any(record["metrics"].values()))
            self.assertIn("invalid_xml", record["failure"]["message"])

    def test_timeout_and_malformed_response_are_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = RuntimeFixture(root)
            outputs = iter(
                [
                    runtime.WorkerOutcome(
                        returncode=-15,
                        timed_out=True,
                        response=metric_result(),
                    ),
                    runtime.WorkerOutcome(
                        returncode=0,
                        response_error="invalid JSON",
                    ),
                ]
            )

            def launch(**_kwargs):
                return next(outputs)

            output = root / "runtime"
            runtime.run_manifest(
                fixture.manifest,
                output,
                simulators=("pybullet",),
                launcher=launch,
            )
            timeout_record = json.loads(
                (output / fixture.slug / "pybullet/assets/asset-0000.json").read_text()
            )
            self.assertEqual("timeout", timeout_record["terminal_status"])
            self.assertFalse(any(timeout_record["metrics"].values()))

            # A different simulator gets an independent terminal denominator row.
            runtime.run_manifest(
                fixture.manifest,
                output,
                simulators=("mujoco",),
                launcher=launch,
            )
            malformed = json.loads(
                (output / fixture.slug / "mujoco/assets/asset-0000.json").read_text()
            )
            self.assertEqual("malformed_response", malformed["terminal_status"])
            self.assertFalse(any(malformed["metrics"].values()))

    def test_resume_requires_exact_identity_and_never_overwrites(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = RuntimeFixture(root)
            count = 0

            def launch(**_kwargs):
                nonlocal count
                count += 1
                return runtime.WorkerOutcome(returncode=0, response=metric_result())

            output = root / "runtime"
            runtime.run_manifest(
                fixture.manifest,
                output,
                simulators=("pybullet",),
                timeout_s=300,
                launcher=launch,
            )
            path = output / fixture.slug / "pybullet/assets/asset-0000.json"
            original = path.read_bytes()
            resumed = runtime.run_manifest(
                fixture.manifest,
                output,
                simulators=("pybullet",),
                timeout_s=300,
                launcher=launch,
            )
            self.assertEqual(1, count)
            self.assertEqual(1, resumed["runs"][0]["resumed_count"])
            self.assertEqual(original, path.read_bytes())

            with self.assertRaisesRegex(
                runtime.RuntimeContractError, "identity does not match exactly"
            ):
                runtime.run_manifest(
                    fixture.manifest,
                    output,
                    simulators=("pybullet",),
                    timeout_s=300,
                    executables={"pybullet": "/different/python"},
                    launcher=launch,
                )
            self.assertEqual(original, path.read_bytes())
            self.assertEqual(1, count)

    def test_manifest_and_row_self_hashes_are_verified(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = RuntimeFixture(root)
            value = json.loads(fixture.manifest.read_text(encoding="utf-8"))
            value["datasets"][0]["rows"][0]["category"] = "tampered"
            value["manifest_sha256"] = runtime.canonical_sha256(
                value, exclude_fields=("manifest_sha256",)
            )
            fixture.manifest.write_text(json.dumps(value), encoding="utf-8")

            with self.assertRaisesRegex(
                runtime.RuntimeContractError, "row_sha256 self-check failed"
            ):
                runtime.load_manifest(fixture.manifest)

    def test_failed_preflight_may_retain_a_null_urdf_hash(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = RuntimeFixture(root)
            fixture.row["urdf_sha256"] = None
            fixture.row["joint_tree"] = None
            fixture.row["preflight"] = {
                "status": "failed",
                "issues": ["urdf_missing"],
                "simulator_eligible": False,
            }
            fixture.write_manifest()
            loaded = runtime.load_manifest(fixture.manifest)
            self.assertIsNone(loaded.datasets[0].rows[0]["urdf_sha256"])

            def forbidden(**_kwargs):
                raise AssertionError("retained failed row must not launch")

            output = root / "runtime"
            runtime.run_manifest(
                fixture.manifest,
                output,
                simulators=("pybullet",),
                launcher=forbidden,
            )
            record = json.loads(
                (output / fixture.slug / "pybullet/assets/asset-0000.json").read_text()
            )
            self.assertIsNone(record["identity"]["urdf_sha256"])
            self.assertEqual("preflight_failure", record["terminal_status"])

    def test_timeout_must_equal_the_frozen_protocol(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = RuntimeFixture(root)
            with self.assertRaisesRegex(
                runtime.RuntimeContractError, "does not match frozen protocol"
            ):
                runtime.run_manifest(
                    fixture.manifest,
                    root / "runtime",
                    simulators=("pybullet",),
                    timeout_s=1,
                    launcher=lambda **_kwargs: runtime.WorkerOutcome(),
                )

    def test_genesis_workers_are_device_limited_and_serialized_per_gpu(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = RuntimeFixture(root)
            rows = []
            for index in range(6):
                row = dict(fixture.row)
                row["asset_id"] = f"source-{index}"
                row["dataset_id"] = f"asset-{index:04d}"
                rows.append(row)
            fixture.write_manifest(
                groups=[
                    {
                        "dataset_slug": fixture.slug,
                        "dataset_name": fixture.slug,
                        "rows": rows,
                    }
                ]
            )
            state_lock = threading.Lock()
            active = {"0": 0, "1": 0}
            overlaps = []

            def launch(**kwargs):
                gpu = kwargs["gpu_binding"]
                with state_lock:
                    active[gpu] += 1
                    if active[gpu] > 1:
                        overlaps.append(gpu)
                time.sleep(0.01)
                with state_lock:
                    active[gpu] -= 1
                return runtime.WorkerOutcome(returncode=0, response=metric_result())

            summary = runtime.run_manifest(
                fixture.manifest,
                root / "runtime",
                simulators=("genesis",),
                workers={"genesis": 8},
                gpu_bindings=("0", "1"),
                launcher=launch,
            )

            self.assertFalse(overlaps)
            self.assertEqual(8, summary["runs"][0]["requested_workers"])
            self.assertEqual(2, summary["runs"][0]["effective_workers"])
            record = json.loads(
                (
                    root / "runtime" / fixture.slug / "genesis/assets/asset-0000.json"
                ).read_text()
            )
            self.assertEqual(2, record["identity"]["effective_workers"])

    def test_diagnostic_failure_keeps_evidence_and_closed_metrics(self) -> None:
        identity = {"timeout_s": 300.0}
        evidence = {
            "reason": "simulator_asset_load_rejected",
            "message": "unsupported mesh",
        }
        record = runtime._terminal_record(
            identity,
            runtime.WorkerOutcome(
                returncode=0,
                response={
                    "diagnostic_failure": evidence,
                    "metrics": runtime._false_metrics(),
                },
            ),
        )
        self.assertEqual("diagnostic_failure", record["terminal_status"])
        self.assertEqual(evidence, record["evaluation"]["diagnostic_failure"])
        self.assertFalse(any(record["metrics"].values()))

    def test_bbox_ne_runs_evaluator_but_forces_drift_and_total_false(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = RuntimeFixture(root)
            row = dict(fixture.row)
            row["bounding_box_diagonal"] = "N/E"
            request = {
                "schema_version": runtime.WORKER_REQUEST_SCHEMA,
                "simulator": "mujoco",
                "row": row,
                "protocol": protocol(),
                "urdf_path": str(fixture.urdf),
            }
            request_path, response_path = root / "request.json", root / "response.json"
            request_path.write_text(json.dumps(request), encoding="utf-8")

            class Adapter:
                def close(self):
                    pass

            def evaluator(_adapter, evaluated_row, _protocol):
                self.assertEqual(1.0, evaluated_row["bounding_box_diagonal"])
                return metric_result()

            with mock.patch.object(
                runtime, "_make_adapter", return_value=Adapter()
            ), mock.patch.object(runtime, "evaluate_asset", side_effect=evaluator):
                self.assertEqual(0, runtime.worker_main(request_path, response_path))

            response = json.loads(response_path.read_text(encoding="utf-8"))
            self.assertTrue(response["metrics"]["load"])
            self.assertTrue(response["metrics"]["limit_enforcement"])
            self.assertFalse(response["metrics"]["constraint_drift"])
            self.assertFalse(response["metrics"]["simulator_pass"])
            self.assertTrue(response["missing_bbox_normalizer"])
            self.assertEqual(
                "missing_bbox_normalizer",
                response["diagnostics"]["missing_bbox_normalizer"]["reason"],
            )

    def test_configured_executable_runs_isolated_child_with_dynamic_gpu(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            executable = root / "fake-python"
            executable.write_text(
                textwrap.dedent(
                    f"""\
                    #!{sys.executable}
                    import json
                    import os
                    import pathlib
                    import sys

                    response = pathlib.Path(sys.argv[sys.argv.index('--response') + 1])
                    response.write_text(json.dumps({{
                        'metrics': {{name: True for name in {runtime.METRIC_NAMES!r}}},
                        'seen_gpu': os.environ.get('CUDA_VISIBLE_DEVICES'),
                    }}))
                    """
                ),
                encoding="utf-8",
            )
            executable.chmod(0o755)
            request = {
                "schema_version": runtime.WORKER_REQUEST_SCHEMA,
                "simulator": "genesis",
                "row": {"dataset_id": "asset-0000"},
                "protocol": protocol(),
            }

            outcome = runtime.spawn_worker_process(
                request=request,
                executable=str(executable),
                timeout_s=2,
                gpu_binding="GPU-test-token",
                work_root=root / "runtime",
            )

            self.assertEqual(0, outcome.returncode)
            self.assertFalse(outcome.timed_out)
            self.assertEqual("GPU-test-token", outcome.response["seen_gpu"])
            self.assertEqual(str(executable), outcome.command[0])

    def test_isolated_child_timeout_terminates_the_process_group(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            executable = root / "slow-python"
            executable.write_text(
                textwrap.dedent(
                    f"""\
                    #!{sys.executable}
                    import time
                    time.sleep(10)
                    """
                ),
                encoding="utf-8",
            )
            executable.chmod(0o755)
            request = {
                "schema_version": runtime.WORKER_REQUEST_SCHEMA,
                "simulator": "pybullet",
                "row": {"dataset_id": "asset-0000"},
                "protocol": protocol(),
            }

            outcome = runtime.spawn_worker_process(
                request=request,
                executable=str(executable),
                timeout_s=0.05,
                gpu_binding=None,
                work_root=root / "runtime",
            )

            self.assertTrue(outcome.timed_out)
            self.assertIsNotNone(outcome.returncode)
            self.assertNotEqual(0, outcome.returncode)

    def test_reuses_established_evaluator_and_cpu_adapters(self) -> None:
        self.assertIs(runtime.evaluate_asset, runtime._legacy.evaluate_asset)
        self.assertIs(runtime.PyBulletAdapter, runtime._legacy.PyBulletAdapter)
        self.assertIs(runtime.MuJoCoAdapter, runtime._legacy.MuJoCoAdapter)


if __name__ == "__main__":
    unittest.main()
