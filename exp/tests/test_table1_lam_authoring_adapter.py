from __future__ import annotations

import importlib.util
import json
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock


EXP_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = EXP_ROOT.parent


def load_runner():
    path = EXP_ROOT / "scripts/run_table1_lam_authoring.py"
    spec = importlib.util.spec_from_file_location("table1_lam_runner_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


runner = load_runner()


def iso(second: int) -> str:
    return f"2026-08-12T00:00:{second:02d}Z"


class Table1LamAuthoringAdapterTest(unittest.TestCase):
    def setUp(self):
        runtime = EXP_ROOT / "runtime/table1_reliability"
        runtime.mkdir(parents=True, exist_ok=True)
        self.tmp = tempfile.TemporaryDirectory(dir=runtime, prefix="lam_adapter_test_")
        self.root = Path(self.tmp.name)
        self.task = {
            "task_id": "FUR-L1-01",
            "prompt": "Build an articulated cabinet with one hinged door.",
            "prompt_sha256": runner.sha256_bytes(
                b"Build an articulated cabinet with one hinged door."
            ),
            "hidden_spec_sha256": "f" * 64,
        }
        self.bindings = {
            "protocol_sha256": "1" * 64,
            "manifest_sha256": "2" * 64,
            "hidden_specs_sha256": "3" * 64,
            "package_schema_sha256": runner.sha256_file(runner.PACKAGE_SCHEMA),
            "result_schema_sha256": runner.sha256_file(runner.RESULT_SCHEMA),
            "common_evaluator_sha256": runner.sha256_file(runner.COMMON_EVALUATOR),
        }
        self.protocol = {
            "protocol_id": "table1_reliability_protocol_v1",
            "max_common_repair_turns": 3,
            "methods": {
                "lam": {
                    "provider": "openai",
                    "model": "gpt-5",
                    "reasoning_effort": "high",
                    "request_parameters": {
                        "temperature": {
                            "sent": False,
                            "value": None,
                            "reason": "GPT-5 Responses handler omits this field",
                        },
                        "top_p": {
                            "sent": False,
                            "value": None,
                            "reason": "GPT-5 Responses handler omits this field",
                        },
                        "max_output_tokens": {"sent": True, "value": 64000},
                        "verbosity": {"sent": True, "value": "medium"},
                    },
                    "native_settings": {
                        "openai_sdk_max_retries": 0,
                        "outer_native_retry_limit": 2,
                        "base_agent_parse_max_attempts": 3,
                        "shape_export_validation_max_retries": 3,
                        "vlm_critic_enabled": True,
                        "vlm_critic_max_iterations": 4,
                        "articulation_feedback_enabled": True,
                        "articulation_feedback_max_iterations": 3,
                        "pointllm_critic_enabled": False,
                        "feedback_fusion_enabled": False,
                        "num_executions": 1,
                        "parallel_workers": 0,
                    },
                }
            },
        }

    def tearDown(self):
        self.tmp.cleanup()

    def native_fixture(self, calls, *, first_transport_failure=False):
        def invoke(
            task,
            attempt_index,
            native_retry_index,
            description,
            invocation_root,
            settings,
            timeout,
            python_executable,
        ):
            del task, settings, timeout, python_executable
            invocation_root.mkdir(parents=True, exist_ok=False)
            (invocation_root / "partial.txt").write_text(
                f"attempt={attempt_index} retry={native_retry_index}\n", encoding="utf-8"
            )
            calls.append((attempt_index, native_retry_index, description, invocation_root))
            transport_failure = (
                first_transport_failure
                and attempt_index == 0
                and native_retry_index == 0
            )
            trace = {
                "schema_version": 1,
                "started_at_utc": iso(attempt_index * 10 + native_retry_index),
                "finished_at_utc": iso(attempt_index * 10 + native_retry_index + 1),
                "wall_time_seconds": 1.25,
                "exit_code": 70 if transport_failure else 0,
                "timed_out": False,
                "provider_call_count": 0,
                "emitted_model_output": not transport_failure,
                "model_response_sha256": None if transport_failure else "a" * 64,
                "credential_values_recorded": False,
            }
            runner.write_json(invocation_root / "native_trace.json", trace)
            return trace

        return invoke

    def evaluator_fixture(self, verdicts):
        def evaluate(
            run_root,
            attempt_root,
            task,
            repeat_id,
            attempt_index,
            bindings,
            timeout,
            harness_python,
        ):
            del timeout
            self.assertEqual(harness_python, runner.DEFAULT_HARNESS_PYTHON)
            product = attempt_root / "normalized_product"
            product.mkdir(parents=True, exist_ok=False)
            source = product / "export.js"
            urdf = product / "generated.urdf"
            source.write_text("console.log('fixture');\n", encoding="utf-8")
            urdf.write_text(
                "<robot name='fixture'><link name='cabinet'/></robot>\n",
                encoding="utf-8",
            )
            passes = verdicts[attempt_index]
            report = {
                "verdicts": {
                    "executable": True,
                    "artifact_saved": True,
                    "urdf_tree_pass": True,
                    "semantic_roles_pass": passes,
                    "joint_spec_pass": passes,
                    "common_qc_pass": passes,
                },
                "binding_checks": {"fixture": True},
                "protocol_checks": {"fixture": True},
                "task_checks": {"fixture": True},
                "feedback": {
                    "schema_version": 1,
                    "task_id": task["task_id"],
                    "attempt_index": attempt_index,
                    "failure_codes": [] if passes else ["JOINT_SPEC_FAILED"],
                    "bounded_diagnostics": {
                        "tree_root_count": 1,
                        "tree_connected_link_count": 1,
                        "tree_link_count": 1,
                    },
                    "policy": "hidden expected values withheld",
                },
            }
            runner.write_json(attempt_root / "common_evaluator_report.json", report)
            package = {
                "artifacts": {
                    "source": {
                        "path": runner.relative(source, run_root),
                        "sha256": runner.sha256_file(source),
                    },
                    "urdf": {
                        "path": runner.relative(urdf, run_root),
                        "sha256": runner.sha256_file(urdf),
                    },
                },
                "execution_probe": {
                    "started_at_utc": iso(attempt_index * 10 + 2),
                    "finished_at_utc": iso(attempt_index * 10 + 3),
                },
                "bindings": bindings,
                "repeat_id": repeat_id,
            }
            return report, package

        return evaluate

    def execute_task(self, task, repeat_id, run_root, native, evaluator):
        return runner.execute_job(
            task,
            repeat_id,
            run_root,
            self.protocol,
            self.bindings,
            Path(os.sys.executable),
            1800.0,
            180.0,
            runner.DEFAULT_HARNESS_PYTHON,
            native_invoker=native,
            attempt_evaluator=evaluator,
        )

    def execute(self, run_root, native, evaluator):
        return self.execute_task(self.task, "r0", run_root, native, evaluator)

    def test_attempt_zero_then_one_common_repair_success_and_resume(self):
        calls = []
        run_root = self.root / "repair_success"
        result = self.execute(
            run_root,
            self.native_fixture(calls),
            self.evaluator_fixture({0: False, 1: True}),
        )

        runner.validate_result(result, runner.DEFAULT_HARNESS_PYTHON)
        self.assertEqual([row["attempt_index"] for row in result["attempts"]], [0, 1])
        self.assertEqual(
            [row["attempt_kind"] for row in result["attempts"]],
            ["attempt_0", "common_repair"],
        )
        self.assertFalse(result["summary"]["first_shot"])
        self.assertTrue(result["summary"]["final_success"])
        self.assertEqual(result["summary"]["repair_turns"], 1)
        self.assertTrue((run_root / "attempts/a0/native/r0/partial.txt").is_file())
        self.assertTrue((run_root / "attempts/a1/native/r0/partial.txt").is_file())
        self.assertIn("JOINT_SPEC_FAILED", calls[1][2])
        self.assertNotIn("hidden_spec_sha256", calls[1][2])
        self.assertEqual(result["attempts"][1]["repair_feedback_sha256"], runner.sha256_file(
            run_root / "attempts/a0/repair_feedback.json"
        ))

        call_count = len(calls)
        resumed = self.execute(
            run_root,
            self.native_fixture(calls),
            self.evaluator_fixture({0: False, 1: True}),
        )
        self.assertEqual(resumed, result)
        self.assertEqual(len(calls), call_count)

        (run_root / "result.json").unlink()
        (run_root / "result_seal.json").unlink(missing_ok=True)
        resumed_from_checkpoint = self.execute(
            run_root,
            self.native_fixture(calls),
            self.evaluator_fixture({0: False, 1: True}),
        )
        self.assertEqual(resumed_from_checkpoint["attempts"], result["attempts"])
        self.assertEqual(len(calls), call_count)

    def test_completed_resume_requires_exact_identity_and_seal_pair(self):
        mutations = {
            "run_id": "lam__OTHER-L1-01__r0",
            "method_id": "pva",
            "task_id": "OTHER-L1-01",
            "repeat_id": "r1",
        }
        for field, replacement in mutations.items():
            with self.subTest(field=field):
                calls = []
                run_root = self.root / f"identity_{field}"
                result = self.execute(
                    run_root,
                    self.native_fixture(calls),
                    self.evaluator_fixture({0: True}),
                )
                result[field] = replacement
                runner.write_json(run_root / "result.json", result)
                with self.assertRaisesRegex(RuntimeError, "stale completed result identity"):
                    self.execute(
                        run_root,
                        self.native_fixture(calls),
                        self.evaluator_fixture({0: True}),
                    )

        incomplete_root = self.root / "incomplete_seal_pair"
        calls = []
        self.execute(
            incomplete_root,
            self.native_fixture(calls),
            self.evaluator_fixture({0: True}),
        )
        result_seal = incomplete_root / "result_seal.json"
        self.assertTrue(result_seal.is_file())
        result_seal.unlink()
        with self.assertRaisesRegex(RuntimeError, "result/resume seal pair incomplete"):
            self.execute(
                incomplete_root,
                self.native_fixture(calls),
                self.evaluator_fixture({0: True}),
            )

    def test_completed_resume_rejects_cross_job_attempt_swap(self):
        source_calls = []
        source_root = self.root / "source_job"
        self.execute(
            source_root,
            self.native_fixture(source_calls),
            self.evaluator_fixture({0: True}),
        )

        other_task = {
            **self.task,
            "task_id": "KIT-L1-01",
            "hidden_spec_sha256": "e" * 64,
        }
        other_calls = []
        other_root = self.root / "other_job"
        self.execute_task(
            other_task,
            "r0",
            other_root,
            self.native_fixture(other_calls),
            self.evaluator_fixture({0: True}),
        )

        swapped_root = self.root / "swapped_job"
        shutil.copytree(source_root, swapped_root)
        shutil.rmtree(swapped_root / "attempts/a0")
        shutil.copytree(other_root / "attempts/a0", swapped_root / "attempts/a0")
        (swapped_root / "result_seal.json").unlink()
        runner.seal_result(
            swapped_root,
            swapped_root / "result.json",
            runner.result_bindings(self.bindings),
            runner.expected_run_identity(self.task, "r0"),
        )
        with self.assertRaisesRegex(RuntimeError, "immutable attempt output"):
            self.execute(
                swapped_root,
                self.native_fixture([]),
                self.evaluator_fixture({0: True}),
            )

    def test_completed_resume_rejects_result_tamper(self):
        calls = []
        run_root = self.root / "result_tamper"
        result = self.execute(
            run_root,
            self.native_fixture(calls),
            self.evaluator_fixture({0: True}),
        )
        result["summary"]["artifact_saved"] = False
        runner.write_json(run_root / "result.json", result)
        with self.assertRaisesRegex(RuntimeError, "changed or stale result output"):
            self.execute(
                run_root,
                self.native_fixture(calls),
                self.evaluator_fixture({0: True}),
            )

    def test_completed_resume_rejects_attempt_evidence_tamper(self):
        calls = []
        source_root = self.root / "evidence_source"
        self.execute(
            source_root,
            self.native_fixture(calls),
            self.evaluator_fixture({0: True}),
        )
        tampered_root = self.root / "tampered_job"
        shutil.copytree(source_root, tampered_root)
        (tampered_root / "attempts/a0/native/r0/partial.txt").write_text(
            "tampered after completion\n", encoding="utf-8"
        )
        (tampered_root / "result_seal.json").unlink()
        runner.seal_result(
            tampered_root,
            tampered_root / "result.json",
            runner.result_bindings(self.bindings),
            runner.expected_run_identity(self.task, "r0"),
        )
        with self.assertRaisesRegex(RuntimeError, "immutable attempt output"):
            self.execute(
                tampered_root,
                self.native_fixture([]),
                self.evaluator_fixture({0: True}),
            )

    def test_output_free_native_retry_is_separate_and_preserved(self):
        calls = []
        run_root = self.root / "native_retry"
        result = self.execute(
            run_root,
            self.native_fixture(calls, first_transport_failure=True),
            self.evaluator_fixture({0: True}),
        )

        runner.validate_result(result, runner.DEFAULT_HARNESS_PYTHON)
        self.assertEqual([(row[0], row[1]) for row in calls], [(0, 0), (0, 1)])
        self.assertEqual(result["attempts"][0]["native_retry_count"], 1)
        self.assertEqual(result["attempts"][0]["native_retry_index"], 1)
        self.assertTrue((run_root / "attempts/a0/native/r0/partial.txt").is_file())
        self.assertTrue((run_root / "attempts/a0/native/r1/partial.txt").is_file())
        self.assertEqual(len(result["attempts"]), 1)

    def test_stale_result_and_checkpoint_bindings_fail_closed(self):
        calls = []
        result_root = self.root / "stale_result"
        result = self.execute(
            result_root,
            self.native_fixture(calls),
            self.evaluator_fixture({0: True}),
        )
        result["bindings"]["protocol_sha256"] = "9" * 64
        runner.write_json(result_root / "result.json", result)
        with self.assertRaisesRegex(RuntimeError, "stale result"):
            self.execute(
                result_root,
                self.native_fixture(calls),
                self.evaluator_fixture({0: True}),
            )

        checkpoint_root = self.root / "stale_checkpoint"
        checkpoint_root.mkdir()
        checkpoint = runner.checkpoint_payload(
            self.protocol,
            self.task,
            "r0",
            self.bindings,
            iso(0),
            [],
        )
        checkpoint["bindings"]["manifest_sha256"] = "8" * 64
        runner.write_json(checkpoint_root / "run_state.json", checkpoint)
        with self.assertRaisesRegex(RuntimeError, "stale checkpoint"):
            self.execute(
                checkpoint_root,
                self.native_fixture(calls),
                self.evaluator_fixture({0: True}),
            )

    def test_generated_config_has_placeholders_not_credentials(self):
        config = self.root / "config.yaml"
        settings = self.protocol["methods"]["lam"]["native_settings"]
        with mock.patch.dict(os.environ, {runner.OPENAI_ENV_KEY: "fixture-secret-value"}):
            runner.no_secret_config(
                config,
                self.root / "native_output",
                self.protocol["methods"]["lam"],
                settings,
            )
        rendered = config.read_text(encoding="utf-8")
        self.assertNotIn("fixture-secret-value", rendered)
        self.assertIn("INHERIT_OPENAI_API_KEY_FROM_ENVIRONMENT", rendered)
        payload = runner.yaml.safe_load(rendered)
        self.assertEqual(set(payload["api"]["agents"]), set(runner.EXPECTED_ROLES))
        self.assertTrue(all(value == "gpt-5" for value in payload["api"]["agents"].values()))
        self.assertNotIn("temperature", payload["api"]["defaults"])
        self.assertNotIn("top_p", payload["api"]["defaults"])
        self.assertEqual(payload["api"]["defaults"]["max_tokens"], 64000)
        self.assertEqual(payload["api"]["overrides"]["gpt-5"]["verbosity"], "medium")

    def test_generated_shim_disables_openai_sdk_retry_at_handler_init(self):
        checkout = self.root / "fake_lam_checkout"
        for package in ("utils", "agents", "providers"):
            (checkout / package).mkdir(parents=True, exist_ok=True)
            (checkout / package / "__init__.py").write_text("", encoding="utf-8")
        (checkout / "utils/pipeline_stage_runner.py").write_text(
            "def cleanup_output(paths):\n    return paths\n", encoding="utf-8"
        )
        (checkout / "utils/pipeline_execution_runner.py").write_text(
            "def cleanup_output(paths):\n    return paths\n", encoding="utf-8"
        )
        (checkout / "agents/shape_generator_agent.py").write_text(
            "class ShapeGeneratorAgent:\n"
            "    def generate_with_export_validation(self, *args, **kwargs):\n"
            "        return kwargs.get('max_retries')\n",
            encoding="utf-8",
        )
        (checkout / "utils/stage_recorder.py").write_text(
            "class StageRecorder:\n"
            "    def _resolve_stage_paths(self, output_folder, stage_num):\n"
            "        return None, output_folder, None\n"
            "    def save_stage_output_immediate(self, output_folder, raw_response, stage_num=None):\n"
            "        return None\n",
            encoding="utf-8",
        )
        (checkout / "providers/openai_handler.py").write_text(
            "class Client:\n"
            "    def __init__(self, max_retries=2):\n"
            "        self.max_retries = max_retries\n"
            "    def with_options(self, *, max_retries):\n"
            "        return Client(max_retries=max_retries)\n"
            "class OpenAIHandler:\n"
            "    def __init__(self, config):\n"
            "        self.client = Client()\n"
            "        self.model_name = 'gpt-5'\n"
            "    def invoke(self, prompt, system_prompt=None, **kwargs):\n"
            "        raise AssertionError('provider must not be called')\n",
            encoding="utf-8",
        )
        (checkout / "run_pipeline.py").write_text(
            "import json\n"
            "from pathlib import Path\n"
            "from providers.openai_handler import OpenAIHandler\n"
            "def main():\n"
            "    handler = OpenAIHandler({})\n"
            "    Path('sdk_retry_fixture.json').write_text(json.dumps({\n"
            "        'max_retries': handler.client.max_retries,\n"
            "    }))\n",
            encoding="utf-8",
        )
        shim = self.root / "sdk_retry_shim.py"
        shim.write_text(
            runner.native_shim_text(
                checkout, self.protocol["methods"]["lam"]["native_settings"]
            ),
            encoding="utf-8",
        )

        completed = subprocess.run(
            [os.sys.executable, str(shim)],
            cwd=self.root,
            env={"PATH": "/usr/bin:/bin"},
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30.0,
            check=False,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertEqual(
            json.loads((self.root / "sdk_retry_fixture.json").read_text()),
            {"max_retries": 0},
        )

    def test_source_probe_generated_js_receives_only_exact_safe_environment(self):
        probe_root = self.root / "source_probe"
        environment_output = probe_root / "visible_environment.json"
        source = self.root / "environment_probe.mjs"
        three_module = (
            runner.LAM_CHECKOUT / "node_modules/three/build/three.module.js"
        ).resolve().as_uri()
        source.write_text(
            "\n".join(
                (
                    f"import * as THREE from {json.dumps(three_module)};",
                    "import fs from 'node:fs';",
                    f"fs.writeFileSync({json.dumps(str(environment_output))}, JSON.stringify(process.env));",
                    "export function createScene() {",
                    "  const root = new THREE.Group();",
                    "  root.name = 'root';",
                    "  const link = new THREE.Group();",
                    "  link.name = 'link';",
                    "  link.add(new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1)));",
                    "  root.add(link);",
                    "  return root;",
                    "}",
                )
            )
            + "\n",
            encoding="utf-8",
        )
        inherited = {
            runner.OPENAI_ENV_KEY: "single-secret",
            "OPENAI_API_KEYS": "multi-secret",
            "TABLE1_HOST_SENTINEL": "host-secret",
            "HTTPS_PROXY": "http://proxy.invalid",
            "HOME": "/host/home/should-not-leak",
        }
        with mock.patch.dict(os.environ, inherited, clear=False):
            probe = runner.run_source_probe(source, probe_root, 30.0)

        self.assertEqual(probe["exit_code"], 0, (probe_root / "stderr.txt").read_text())
        visible = json.loads(environment_output.read_text(encoding="utf-8"))
        self.assertEqual(
            visible,
            {
                "PATH": "/usr/bin:/bin",
                "NODE_PATH": str(runner.LAM_CHECKOUT / "node_modules"),
                "OPENBLAS_NUM_THREADS": "1",
                "OMP_NUM_THREADS": "1",
                "MKL_NUM_THREADS": "1",
                "NUMEXPR_NUM_THREADS": "1",
            },
        )
        self.assertEqual(
            set(visible),
            {"PATH", "NODE_PATH", *runner.THREAD_ENV},
        )
        self.assertNotIn(runner.OPENAI_ENV_KEY, visible)
        self.assertNotIn("OPENAI_API_KEYS", visible)
        self.assertNotIn("TABLE1_HOST_SENTINEL", visible)
        self.assertNotIn("HTTPS_PROXY", visible)
        self.assertNotIn("HOME", visible)

    def test_all_four_failed_attempts_remain_schema_valid_and_isolated(self):
        calls = []
        run_root = self.root / "exhausted"
        result = self.execute(
            run_root,
            self.native_fixture(calls),
            self.evaluator_fixture({0: False, 1: False, 2: False, 3: False}),
        )

        runner.validate_result(result, runner.DEFAULT_HARNESS_PYTHON)
        self.assertEqual(len(result["attempts"]), 4)
        self.assertFalse(result["summary"]["first_shot"])
        self.assertFalse(result["summary"]["final_success"])
        self.assertEqual(result["summary"]["repair_turns"], 3)
        roots = [run_root / f"attempts/a{index}/native/r0" for index in range(4)]
        self.assertEqual(len({path.resolve() for path in roots}), 4)
        self.assertTrue(all((path / "partial.txt").is_file() for path in roots))

    def test_common_evaluator_exceptions_seal_terminal_not_evaluable_and_resume(self):
        error_cases = {
            "timeout": subprocess.TimeoutExpired(
                ["common-evaluator"], 180.0, stderr="internal-sensitive-detail"
            ),
            "json": json.JSONDecodeError(
                "internal-sensitive-detail", "not-json", 0
            ),
            "runtime": RuntimeError("internal-sensitive-detail"),
        }
        for name, error in error_cases.items():
            with self.subTest(name=name):
                native_calls = []
                evaluator_calls = []

                def failing_evaluator(*args):
                    evaluator_calls.append(args[4])
                    raise error

                run_root = self.root / f"evaluator_{name}"
                result = self.execute(
                    run_root,
                    self.native_fixture(native_calls),
                    failing_evaluator,
                )

                runner.validate_result(result, runner.DEFAULT_HARNESS_PYTHON)
                self.assertEqual(result["status"], "failed")
                self.assertEqual(result["summary"]["state"], "not_evaluable")
                self.assertTrue(
                    all(
                        result["summary"][key] is None
                        for key in (
                            "executable",
                            "artifact_saved",
                            "first_shot",
                            "final_success",
                            "repair_turns",
                        )
                    )
                )
                self.assertEqual(result["error"]["code"], "COMMON_EVALUATOR_NOT_EVALUABLE")
                self.assertEqual(len(result["attempts"]), 1)
                attempt = result["attempts"][0]
                self.assertEqual(attempt["failure_class"], "common_evaluator_failure")
                self.assertEqual(attempt["evaluation"]["state"], "not_evaluable")
                self.assertIsNone(attempt["evaluation"]["common_qc_pass"])
                self.assertEqual(evaluator_calls, [0])
                self.assertEqual(len(native_calls), 1)
                self.assertFalse((run_root / "attempts/a1").exists())
                self.assertFalse((run_root / "attempts/a0/repair_feedback.json").exists())
                self.assertTrue((run_root / "attempts/a0/common_evaluator_failure.json").is_file())
                self.assertTrue((run_root / "attempts/a0/attempt_seal.json").is_file())
                self.assertTrue((run_root / "result_seal.json").is_file())
                emitted = "\n".join(
                    path.read_text(encoding="utf-8", errors="replace")
                    for path in run_root.rglob("*")
                    if path.is_file()
                )
                self.assertNotIn("internal-sensitive-detail", emitted)

                resumed = self.execute(
                    run_root,
                    self.native_fixture(native_calls),
                    failing_evaluator,
                )
                self.assertEqual(resumed, result)
                self.assertEqual(evaluator_calls, [0])
                self.assertEqual(len(native_calls), 1)

    def test_failed_provider_call_makes_retry_telemetry_unreported(self):
        first = self.root / "telemetry_r0"
        second = self.root / "telemetry_r1"
        first.mkdir()
        second.mkdir()
        runner.write_json(
            first / "native_trace.json",
            {"provider_call_count": 1},
        )
        runner.write_text_atomic(
            first / "provider_calls.jsonl",
            json.dumps(
                {
                    "status": "exception",
                    "input_tokens": None,
                    "output_tokens": None,
                    "api_cost_usd": None,
                }
            )
            + "\n",
        )
        runner.write_json(second / "native_trace.json", {"provider_call_count": 1})
        runner.write_text_atomic(
            second / "provider_calls.jsonl",
            json.dumps(
                {
                    "status": "completed",
                    "input_tokens": 10,
                    "output_tokens": 20,
                    "api_cost_usd": 0.5,
                }
            )
            + "\n",
        )

        telemetry = runner.telemetry_from_native([first, second])
        self.assertIsNone(telemetry["input_tokens"])
        self.assertIsNone(telemetry["output_tokens"])
        self.assertIsNone(telemetry["api_cost_usd"])
        self.assertIn("partial telemetry is not imputed", telemetry["reason"])

    def test_aggregate_headline_metrics_use_intended_denominator(self):
        observed = {
            "protocol_id": self.protocol["protocol_id"],
            "summary": {
                "state": "observed",
                "executable": True,
                "artifact_saved": True,
                "first_shot": False,
                "final_success": True,
                "repair_turns": 1,
            },
            "attempts": [],
        }
        not_evaluable = {
            "protocol_id": self.protocol["protocol_id"],
            "summary": {
                "state": "not_evaluable",
                "executable": None,
                "artifact_saved": None,
                "first_shot": None,
                "final_success": None,
                "repair_turns": None,
            },
            "attempts": [],
        }

        summary = runner.aggregate_results([observed, not_evaluable], intended_runs=3)

        self.assertTrue(
            all(metric["denominator"] == 3 for metric in summary["metrics"].values())
        )
        self.assertEqual(summary["strict_denominator"], 3)
        self.assertEqual(summary["completed_results"], 2)
        self.assertEqual(summary["observed_denominator"], 1)
        self.assertEqual(summary["evaluable_denominator"], 1)


if __name__ == "__main__":
    unittest.main()
