from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "exp" / "scripts" / "run_naive_same_llm_editability_v2.py"
CONTRACT_PATH = (
    REPO_ROOT
    / "exp"
    / "reference"
    / "editability_v2"
    / "naive_same_llm_prompt_contract_v1.json"
)
TEST_TMP_ROOT = REPO_ROOT / "exp" / "runtime" / "nano3d_editability_v2" / "test_tmp"


def load_runner():
    spec = importlib.util.spec_from_file_location("run_naive_same_llm_editability", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to load runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


runner = load_runner()


class NaiveSameLlmEditabilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        TEST_TMP_ROOT.mkdir(parents=True, exist_ok=True)

    @classmethod
    def tearDownClass(cls) -> None:
        if TEST_TMP_ROOT.exists():
            shutil.rmtree(TEST_TMP_ROOT)

    def test_prompt_contains_full_source_but_not_gold(self) -> None:
        source = "from sdk import Asset\n\nvalue = 'complete-parent-source'\n"
        task = {
            "task_id": "case_alpha",
            "original_object_prompt": "Make an object.",
            "normalized_edit_instruction": "Add a handle.",
            "gold": {"private_sentinel": "DO_NOT_LEAK_THIS_GOLD"},
        }
        digest = hashlib.sha256(source.encode("utf-8")).hexdigest()

        message = runner.render_user_message(task, source, digest)

        self.assertIn(source, message)
        self.assertIn(digest, message)
        self.assertNotIn("DO_NOT_LEAK_THIS_GOLD", message)

    def test_output_root_lock_rejects_concurrent_owner(self) -> None:
        with tempfile.TemporaryDirectory(dir=TEST_TMP_ROOT) as temp:
            output = Path(temp) / "locked-output"
            with runner.output_root_lock(
                output,
                mode="execute",
                expected_manifest_sha256="a" * 64,
            ) as first_owner:
                self.assertEqual(first_owner["status"], "ACTIVE")
                owner_path = output / ".runner.lock.owner.json"
                persisted = json.loads(owner_path.read_text(encoding="utf-8"))
                self.assertEqual(persisted["pid"], first_owner["pid"])
                self.assertEqual(persisted["runner_sha256"], runner.sha256(SCRIPT_PATH))
                with self.assertRaisesRegex(RuntimeError, "OUTPUT_ROOT_LOCK_HELD"):
                    with runner.output_root_lock(
                        output,
                        mode="execute",
                        expected_manifest_sha256="a" * 64,
                    ):
                        self.fail("a second owner acquired the same output-root lock")

            released = json.loads(owner_path.read_text(encoding="utf-8"))
            self.assertEqual(released["status"], "RELEASED_COMPLETE")
            self.assertIn("finished_at_utc", released)

    def test_response_assessment_is_fail_closed(self) -> None:
        accepted = runner.assess_response(
            {
                "content": "```python\nvalue = 1\n```",
                "tool_calls": [],
                "finish_reason": "stop",
                "usage": {
                    "prompt_tokens": 100,
                    "cached_tokens": 0,
                    "candidates_tokens": 10,
                    "total_tokens": 110,
                },
            }
        )
        truncated = runner.assess_response(
            {
                "content": "```python\nvalue = 1\n```",
                "tool_calls": [],
                "finish_reason": "length",
                "usage": {"candidates_tokens": 65_536},
            }
        )
        tool_call = runner.assess_response(
            {
                "content": "```python\nvalue = 1\n```",
                "tool_calls": [{"name": "compile_model"}],
                "finish_reason": "stop",
                "usage": {},
            }
        )

        self.assertTrue(accepted["accepted"])
        self.assertEqual(truncated["status"], "OUTPUT_TRUNCATED_OR_INVALID")
        self.assertFalse(truncated["accepted"])
        self.assertEqual(tool_call["status"], "PROTOCOL_VIOLATION_TOOL_CALL")

    def test_context_pressure_blocks_without_source_truncation(self) -> None:
        with tempfile.TemporaryDirectory(dir=TEST_TMP_ROOT) as temp:
            output = Path(temp) / "out"
            source = "value = '" + ("x" * 20_000) + "'\n"
            task = {
                "task_id": "long_case",
                "parent_model_path": "unused/model.py",
                "parent_record_id": "parent_long",
                "edit_class": "numeric",
                "gold_sha256": "0" * 64,
                "original_object_prompt": "Make a long object.",
                "normalized_edit_instruction": "Change one property.",
                "source": source,
                "source_sha256": hashlib.sha256(source.encode()).hexdigest(),
                "source_utf8_bytes": len(source.encode()),
            }
            with mock.patch.dict(
                runner.FROZEN_SETTINGS,
                {"context_window_tokens": 1_000, "max_output_tokens": 512},
            ):
                rows, errors = runner.build_previews(
                    [task], system_prompt="system", output_dir=output
                )

            self.assertTrue(rows[0]["full_source_present"])
            self.assertFalse(rows[0]["output_cap_preserved"])
            self.assertIn("long_case:OUTPUT_CAP_REDUCED_BY_CONTEXT", errors)

    def test_mock_end_to_end_uses_18_manifest_tasks_without_api(self) -> None:
        with tempfile.TemporaryDirectory(dir=TEST_TMP_ROOT) as temp:
            root = Path(temp)
            sources = root / "sources"
            mocks = root / "mocks"
            output = root / "output"
            sources.mkdir()
            mocks.mkdir()
            contract_sha = hashlib.sha256(CONTRACT_PATH.read_bytes()).hexdigest()
            tasks = []
            classes = ["numeric"] * 6 + ["component"] * 6 + ["structure"] * 6
            for index, edit_class in enumerate(classes, start=1):
                task_id = f"new_case_{index:02d}"
                model_dir = sources / task_id
                model_dir.mkdir()
                model_path = model_dir / "model.py"
                source = f"value_{index} = {index}\n"
                model_path.write_text(source, encoding="utf-8")
                tasks.append(
                    {
                        "task_id": task_id,
                        "parent_record_id": f"parent_{index:02d}",
                        "parent_model_path": str(model_path.relative_to(runner.WORKSPACE_ROOT)),
                        "parent_model_sha256": hashlib.sha256(source.encode()).hexdigest(),
                        "original_object_prompt": f"Build object {index}.",
                        "normalized_edit_instruction": f"Apply edit {index}.",
                        "edit_class": edit_class,
                        "gold": {"private_sentinel": f"GOLD_MUST_NOT_LEAK_{index:02d}"},
                    }
                )
                (mocks / f"{task_id}.json").write_text(
                    json.dumps(
                        {
                            "content": f"```python\nvalue_{index} = {index + 1}\n```",
                            "tool_calls": [],
                            "finish_reason": "stop",
                            "usage": {
                                "prompt_tokens": 100 + index,
                                "cached_tokens": 0,
                                "candidates_tokens": 20,
                                "total_tokens": 120 + index,
                            },
                        }
                    ),
                    encoding="utf-8",
                )
            manifest = {
                "schema_version": 1,
                "protocol_id": "nano3d_table5_editability_shared_v2",
                "prompt_contract_sha256": contract_sha,
                "method_protocols": {
                    "naive_one_shot": dict(runner.FROZEN_SETTINGS),
                },
                "pricing_snapshot": dict(runner.FROZEN_PRICING),
                "cohort_distribution": {"numeric": 6, "component": 6, "structure": 6},
                "tasks": tasks,
            }
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
            manifest_sha = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
            argv = [
                str(SCRIPT_PATH),
                "--mode",
                "mock",
                "--manifest",
                str(manifest_path),
                "--expected-manifest-sha256",
                manifest_sha,
                "--mock-responses",
                str(mocks),
                "--output-dir",
                str(output),
            ]
            with mock.patch.object(sys, "argv", argv), contextlib.redirect_stdout(io.StringIO()):
                returncode = runner.main()

            summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(returncode, 0)
            self.assertEqual(summary["status"], "MOCK_COMPLETE")
            self.assertFalse(summary["network_accessed"])
            self.assertFalse(summary["paid_api_called"])
            self.assertEqual(len(summary["records"]), 18)
            self.assertEqual(summary["requirements"]["preview_gates_passed"], 18)
            for index in range(1, 19):
                task_id = f"new_case_{index:02d}"
                preview = (output / "request_previews" / f"{task_id}.json").read_text(
                    encoding="utf-8"
                )
                self.assertNotIn(f"GOLD_MUST_NOT_LEAK_{index:02d}", preview)
                self.assertTrue((output / "tasks" / task_id / "model.py").is_file())


if __name__ == "__main__":
    unittest.main()
