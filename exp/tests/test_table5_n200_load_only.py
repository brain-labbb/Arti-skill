#!/usr/bin/env python3
"""Focused tests for the independent Table 5 strict-load runner."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import textwrap
import unittest
from unittest import mock


REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / "exp/scripts/table5_n200_load_only.py"


def load_module():
    name = "table5_n200_load_only_test_target"
    spec = importlib.util.spec_from_file_location(name, SCRIPT)
    if spec is None or spec.loader is None:
        raise AssertionError(f"cannot load {SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


runner = load_module()


def frozen_protocol() -> dict:
    protocol = {
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
        "metrics": {
            "simulator_pass": {
                "logical_and": [
                    "load",
                    "reset",
                    "settling",
                    "actuation",
                    "limit_enforcement",
                    "constraint_drift",
                ]
            }
        },
        "cross_simulator": {},
        "adapters": {
            "pybullet": {},
            "mujoco": {},
            "genesis": {},
        },
    }
    protocol["protocol_sha256"] = runner._base.canonical_sha256(
        protocol, exclude_fields=("protocol_sha256",)
    )
    return protocol


def evaluation(load: bool = True) -> dict:
    return {
        "schema_version": runner.EVALUATION_SCHEMA,
        **runner._profile_fields(),
        "metrics": {"load": load},
        "load": {"strict_load": load},
        "support": {"joints": []},
        "diagnostics": {"warnings": [], "errors": []},
        "failure": None,
    }


class Fixture:
    def __init__(self, root: Path) -> None:
        self.root = root
        self.slug = "artiverse"
        self.urdf = root / "asset" / "model.urdf"
        self.urdf.parent.mkdir(parents=True)
        self.urdf.write_text("<robot name='fixture'/>", encoding="utf-8")
        joint = {
            "name": "hinge",
            "type": "revolute",
            "parent": "base",
            "child": "door",
            "lower": 0.0,
            "upper": 1.0,
            "effort": 1.0,
            "velocity": 1.0,
        }
        self.row = {
            "dataset_slug": self.slug,
            "dataset_name": "Artiverse",
            "asset_id": "source/asset-0000",
            "dataset_id": "asset-0000",
            "category": "cabinet",
            "package_root": str(self.urdf.parent),
            "urdf_path": str(self.urdf),
            "urdf_sha256": runner.sha256_file(self.urdf),
            "source_parent": None,
            "joint_tree": {
                "links": ["base", "door"],
                "root_links": ["base"],
                "joints": [joint],
            },
            "scalar_joints": [joint],
            "bounding_box_diagonal": 1.0,
            "preflight": {
                "status": "pass",
                "issues": [],
                "simulator_eligible": True,
            },
            "strict_gates": {},
        }
        self.manifest = root / "manifest.json"
        self.write_manifest()

    def write_manifest(self) -> None:
        row = json.loads(json.dumps(self.row))
        row["row_sha256"] = runner._base.canonical_sha256(
            row, exclude_fields=("row_sha256",)
        )
        protocol = frozen_protocol()
        manifest = {
            "schema_version": "table5_six_dataset_prefix_manifest_v1",
            "protocol": protocol,
            "protocol_sha256": protocol["protocol_sha256"],
            "ordered_dataset_slugs": [self.slug],
            "datasets": [
                {
                    "dataset_slug": self.slug,
                    "dataset_name": "Artiverse",
                    "rows": [row],
                }
            ],
        }
        manifest["manifest_sha256"] = runner._base.canonical_sha256(
            manifest, exclude_fields=("manifest_sha256",)
        )
        self.manifest.write_text(json.dumps(manifest), encoding="utf-8")


class Table5N200LoadOnlyTests(unittest.TestCase):
    def test_success_schema_path_identity_and_metric_scope(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = Fixture(root)
            calls = []

            def launch(**kwargs):
                calls.append(kwargs)
                return runner.WorkerOutcome(returncode=0, response=evaluation(True))

            output = root / "load-runtime"
            summary = runner.run_manifest(
                fixture.manifest,
                output,
                simulators=("pybullet",),
                workers={"pybullet": 2},
                executables={"pybullet": "/opt/pybullet/python"},
                launcher=launch,
            )

            path = output / "artiverse/pybullet/assets/asset-0000.json"
            record = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(runner.ASSET_SCHEMA, record["schema_version"])
            self.assertEqual({"load": True}, record["metrics"])
            self.assertEqual(["load"], record["planned_metrics"])
            self.assertEqual(
                list(runner.NOT_EVALUATED_METRICS), record["not_evaluated_metrics"]
            )
            self.assertEqual(
                runner.EXECUTION_PROFILE, record["identity"]["execution_profile"]
            )
            self.assertEqual(
                runner.EXECUTION_PROFILE_SHA256,
                record["identity"]["execution_profile_sha256"],
            )
            self.assertEqual(
                runner.sha256_file(SCRIPT), record["identity"]["runner_source_sha256"]
            )
            self.assertEqual(1, summary["terminal_count"])
            self.assertEqual(2, calls[0]["request"]["identity"]["effective_workers"])

    def test_preflight_failure_retains_denominator_without_launch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = Fixture(root)
            fixture.row["joint_tree"] = None
            fixture.row["scalar_joints"] = []
            fixture.row["urdf_sha256"] = None
            fixture.row["preflight"] = {
                "status": "failed",
                "issues": ["invalid_xml"],
                "simulator_eligible": False,
            }
            fixture.write_manifest()

            def forbidden(**_kwargs):
                raise AssertionError("failed preflight must not launch")

            output = root / "load-runtime"
            summary = runner.run_manifest(
                fixture.manifest,
                output,
                simulators=("mujoco",),
                launcher=forbidden,
            )
            record = json.loads(
                (output / "artiverse/mujoco/assets/asset-0000.json").read_text()
            )
            self.assertEqual("preflight_failure", record["terminal_status"])
            self.assertEqual({"load": False}, record["metrics"])
            self.assertEqual(1, summary["intent_count"])
            self.assertEqual(1, summary["terminal_count"])
            self.assertEqual(0, summary["runs"][0]["load_pass_count"])

    def test_resume_requires_exact_identity_and_never_overwrites(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = Fixture(root)
            count = 0

            def launch(**_kwargs):
                nonlocal count
                count += 1
                return runner.WorkerOutcome(returncode=0, response=evaluation(True))

            output = root / "load-runtime"
            runner.run_manifest(
                fixture.manifest,
                output,
                simulators=("pybullet",),
                launcher=launch,
            )
            path = output / "artiverse/pybullet/assets/asset-0000.json"
            original = path.read_bytes()
            resumed = runner.run_manifest(
                fixture.manifest,
                output,
                simulators=("pybullet",),
                launcher=launch,
            )
            self.assertEqual(1, count)
            self.assertEqual(1, resumed["runs"][0]["resumed_count"])
            self.assertEqual(original, path.read_bytes())
            with self.assertRaisesRegex(
                runner.LoadOnlyContractError, "identity does not match exactly"
            ):
                runner.run_manifest(
                    fixture.manifest,
                    output,
                    simulators=("pybullet",),
                    timeout_s=299,
                    launcher=launch,
                )
            self.assertEqual(original, path.read_bytes())
            self.assertEqual(1, count)

    def test_evaluator_only_computes_strict_load(self) -> None:
        fixture_row = {
            "joint_tree": {
                "links": ["base", "door"],
                "root_links": ["base"],
                "joints": [
                    {
                        "name": "hinge",
                        "type": "revolute",
                        "parent": "base",
                        "child": "door",
                        "lower": 0.0,
                        "upper": 1.0,
                        "effort": 1.0,
                        "velocity": 1.0,
                    }
                ],
            }
        }

        class Adapter:
            observed_link_names = ["base", "door"]
            observed_joint_names = ["hinge"]
            mapped_joint_names = ["hinge"]
            warnings = []

            def reset(self, *_args):
                raise AssertionError("load-only must not reset")

            def state(self):
                raise AssertionError("load-only must not read state")

            def step(self, *_args):
                raise AssertionError("load-only must not step")

            def link_poses(self):
                raise AssertionError("load-only must not read poses")

        result = runner.evaluate_load_only(Adapter(), fixture_row)
        self.assertEqual({"load": True}, result["metrics"])
        self.assertTrue(result["load"]["strict_load"])

    def test_pybullet_adapter_bypasses_only_the_actual_state_capacity_gate(
        self,
    ) -> None:
        original = runner._legacy._require_pybullet_actual_state_capacity

        class FakePyBulletAdapter:
            def __init__(self, *_args):
                runner._legacy._require_pybullet_actual_state_capacity(999)

        with mock.patch.object(runner, "PyBulletAdapter", FakePyBulletAdapter):
            adapter = runner._make_load_only_adapter(
                "pybullet", Path("model.urdf"), {}, frozen_protocol()
            )
        self.assertIsInstance(adapter, FakePyBulletAdapter)
        self.assertIs(original, runner._legacy._require_pybullet_actual_state_capacity)

    def test_worker_writes_native_strict_load_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fixture = Fixture(root)
            bundle = runner._base.load_manifest(fixture.manifest)
            row = bundle.datasets[0].rows[0]
            identity = runner._identity(
                bundle,
                row,
                simulator="mujoco",
                executable=sys.executable,
                timeout_s=300,
                effective_workers=1,
            )
            request = {
                "schema_version": runner.WORKER_REQUEST_SCHEMA,
                **runner._profile_fields(),
                "simulator": "mujoco",
                "identity": identity,
                "row": row,
                "protocol": bundle.protocol,
                "urdf_path": identity["urdf_path"],
            }
            request_path, response_path = root / "request.json", root / "response.json"
            request_path.write_text(json.dumps(request), encoding="utf-8")

            class Adapter:
                observed_link_names = ["base", "door"]
                observed_joint_names = ["hinge"]
                mapped_joint_names = ["hinge"]
                warnings = []

                def close(self):
                    pass

            with mock.patch.object(
                runner, "_make_load_only_adapter", return_value=Adapter()
            ):
                self.assertEqual(0, runner.worker_main(request_path, response_path))
            response = json.loads(response_path.read_text(encoding="utf-8"))
            self.assertEqual({"load": True}, response["metrics"])
            self.assertTrue(response["load"]["strict_load"])

    def test_timeout_and_malformed_response_are_fail_closed(self) -> None:
        identity = {"timeout_s": 1.0}
        timeout = runner._terminal_record(
            identity,
            runner.WorkerOutcome(
                returncode=-15, timed_out=True, response=evaluation(True)
            ),
        )
        malformed = runner._terminal_record(
            identity,
            runner.WorkerOutcome(returncode=0, response_error="bad JSON"),
        )
        self.assertEqual("timeout", timeout["terminal_status"])
        self.assertEqual({"load": False}, timeout["metrics"])
        self.assertEqual("malformed_response", malformed["terminal_status"])
        self.assertEqual({"load": False}, malformed["metrics"])

    def test_configured_executable_is_a_real_isolated_child(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            executable = root / "fake-python"
            executable.write_text(
                textwrap.dedent(
                    f"""\
                    #!{sys.executable}
                    import json
                    import pathlib
                    import sys
                    response = pathlib.Path(sys.argv[sys.argv.index('--response') + 1])
                    response.write_text(json.dumps({evaluation(True)!r}))
                    """
                ),
                encoding="utf-8",
            )
            executable.chmod(0o755)
            outcome = runner.spawn_worker_process(
                request={
                    "row": {"dataset_id": "asset-0000"},
                    "protocol": frozen_protocol(),
                },
                executable=str(executable),
                timeout_s=2,
                work_root=root / "runtime",
            )
            self.assertEqual(0, outcome.returncode)
            self.assertEqual({"load": True}, outcome.response["metrics"])
            self.assertEqual(str(executable), outcome.command[0])

    def test_rejects_genesis(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            fixture = Fixture(Path(temporary))
            with self.assertRaisesRegex(
                runner.LoadOnlyContractError, "subset of pybullet,mujoco"
            ):
                runner.run_manifest(
                    fixture.manifest,
                    Path(temporary) / "runtime",
                    simulators=("genesis",),
                    launcher=lambda **_kwargs: runner.WorkerOutcome(),
                )


if __name__ == "__main__":
    unittest.main()
