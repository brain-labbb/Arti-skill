from __future__ import annotations

import importlib.util
import io
import json
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from unittest import mock

EXP_ROOT = Path(__file__).resolve().parents[1]


def load_helper():
    path = EXP_ROOT / "scripts/pipeline_ablation_authoring_isolation.py"
    spec = importlib.util.spec_from_file_location("pipeline_ablation_authoring_isolation", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


isolation = load_helper()


class PipelineAblationAuthoringIsolationTest(unittest.TestCase):
    def test_provider_surface_is_exactly_packet_read_and_single_submission(self):
        session = isolation.AuthoringIsolationSession('{"task_id":"F-001"}')
        schemas = session.get_tool_schemas()

        names = {row["function"]["name"] for row in schemas}
        self.assertEqual(names, {"read_authoring_packet", "submit_template"})
        self.assertTrue(
            names.isdisjoint(
                {
                    "read_file",
                    "write_file",
                    "replace",
                    "apply_patch",
                    "compile_model",
                    "probe_model",
                    "find_examples",
                    "shell",
                }
            )
        )
        read_schema = next(
            row for row in schemas if row["function"]["name"] == "read_authoring_packet"
        )
        submit_schema = next(row for row in schemas if row["function"]["name"] == "submit_template")
        self.assertEqual(read_schema["function"]["parameters"]["properties"], {})
        self.assertFalse(read_schema["function"]["parameters"]["additionalProperties"])
        self.assertEqual(submit_schema["function"]["parameters"]["properties"].keys(), {"template"})
        self.assertEqual(submit_schema["function"]["parameters"]["required"], ["template"])
        self.assertFalse(submit_schema["function"]["parameters"]["additionalProperties"])

        schemas[0]["function"]["name"] = "read_file"
        self.assertEqual(session.get_all_tool_names(), ("read_authoring_packet", "submit_template"))

    def test_canary_file_and_arbitrary_read_capabilities_are_denied_without_io(self):
        allowed_packet = '{"task_id":"F-001","prompt":"author allowed object"}'
        secret = "PIPELINE_ABLATION_CANARY_7a2d91"
        session = isolation.AuthoringIsolationSession(allowed_packet)

        with tempfile.TemporaryDirectory() as directory:
            canary_path = Path(directory) / "hidden_gold_canary.txt"
            canary_path.write_text(secret, encoding="utf-8")
            attempted_calls = [
                ("read_authoring_packet", {"path": str(canary_path)}),
                ("read_file", {"path": str(canary_path)}),
                ("compile_model", {"path": str(canary_path)}),
                ("probe_model", {"code": f"open({str(canary_path)!r}).read()"}),
                ("find_examples", {"query": secret}),
                ("shell", {"command": f"cat {canary_path}"}),
            ]

            denial = AssertionError("filesystem I/O attempted")
            with (
                mock.patch("builtins.open", side_effect=denial),
                mock.patch.object(io, "open", side_effect=denial),
            ):
                denied = [session.dispatch(name, args) for name, args in attempted_calls]
                allowed = session.dispatch("read_authoring_packet", {})

        self.assertTrue(all(not result.ok for result in denied))
        self.assertTrue(allowed.ok)
        self.assertEqual(allowed.output["packet"], allowed_packet)
        rendered = json.dumps(
            [result.provider_payload() for result in denied] + [allowed.provider_payload()],
            ensure_ascii=False,
        )
        self.assertNotIn(secret, rendered)
        self.assertNotIn(str(canary_path), rendered)

    def test_normalized_provider_call_and_single_exact_template_submission(self):
        session = isolation.AuthoringIsolationSession('{"task_id":"F-001"}')
        template = "from sdk import *\n\ndef build_fixture():\n    return None\n"

        read_result = session.dispatch_provider_call(
            {
                "id": "call_read",
                "type": "function",
                "function": {"name": "read_authoring_packet", "arguments": "{}"},
            }
        )
        submit_result = session.dispatch_provider_call(
            {
                "id": "call_submit",
                "type": "function",
                "function": {
                    "name": "submit_template",
                    "arguments": json.dumps({"template": template}),
                },
            }
        )
        duplicate = session.dispatch(
            "submit_template",
            {"template": "different text"},
            tool_call_id="call_duplicate",
        )
        post_submit_read = session.dispatch("read_authoring_packet", {})

        self.assertTrue(read_result.ok)
        self.assertTrue(submit_result.ok)
        self.assertFalse(duplicate.ok)
        self.assertIn(duplicate.error_code, {"already_submitted", "session_closed"})
        self.assertFalse(post_submit_read.ok)
        self.assertEqual(post_submit_read.error_code, "session_closed")
        accepted = session.require_submission()
        self.assertEqual(accepted.text, template)
        self.assertEqual(submit_result.output["template_sha256"], accepted.sha256)
        self.assertNotIn(template, submit_result.provider_message()["content"])

    def test_concurrent_submissions_accept_exactly_one(self):
        session = isolation.AuthoringIsolationSession('{"task_id":"F-001"}')
        barrier = threading.Barrier(12)
        results = []
        results_lock = threading.Lock()

        def submit(index: int) -> None:
            barrier.wait()
            result = session.dispatch(
                "submit_template",
                {"template": f"template-{index}"},
                tool_call_id=f"call-{index}",
            )
            with results_lock:
                results.append(result)

        threads = [threading.Thread(target=submit, args=(index,)) for index in range(12)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=5)

        self.assertTrue(all(not thread.is_alive() for thread in threads))
        self.assertEqual(sum(result.ok for result in results), 1)
        accepted = session.require_submission()
        self.assertIn(accepted.text, {f"template-{index}" for index in range(12)})

    def test_invalid_calls_do_not_count_as_submission(self):
        session = isolation.AuthoringIsolationSession(
            '{"task_id":"F-001"}',
            max_template_bytes=8,
        )

        invalid = [
            session.dispatch("submit_template", {"template": ""}),
            session.dispatch("submit_template", {"template": "123456789"}),
            session.dispatch("submit_template", {"template": "ok", "path": "model.py"}),
            session.dispatch("submit_template", "not-json"),
            session.dispatch_provider_call(
                {
                    "id": "custom-call",
                    "type": "custom",
                    "function": {"name": "submit_template", "arguments": "{}"},
                }
            ),
        ]

        self.assertTrue(all(not result.ok for result in invalid))
        self.assertFalse(session.has_submission)
        with self.assertRaises(isolation.MissingTemplateSubmission):
            session.require_submission()

    def test_submitted_text_is_never_executed_or_written_by_the_helper(self):
        session = isolation.AuthoringIsolationSession('{"task_id":"F-001"}')
        inert_text = "open('/tmp/should-not-exist', 'w').write('bad')\n"

        denial = AssertionError("template was executed")
        with (
            mock.patch("builtins.open", side_effect=denial),
            mock.patch.object(io, "open", side_effect=denial),
        ):
            result = session.dispatch("submit_template", {"template": inert_text})

        self.assertTrue(result.ok)
        self.assertEqual(session.require_submission().text, inert_text)


if __name__ == "__main__":
    unittest.main()
