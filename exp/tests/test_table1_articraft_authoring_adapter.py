#!/usr/bin/env python3
"""Zero-provider focused tests for the Table 1 Articraft outer adapter."""

from __future__ import annotations

import importlib.util
import json
import os
import shutil
import subprocess
import sys
import time
import uuid
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = REPO_ROOT / "exp/scripts/run_table1_articraft_authoring.py"
TEST_ROOT = REPO_ROOT / "exp/runtime/table1_reliability/articraft_adapter_mock_tests"

spec = importlib.util.spec_from_file_location("table1_articraft_runner", RUNNER_PATH)
assert spec is not None and spec.loader is not None
runner = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = runner
spec.loader.exec_module(runner)


def check(value: bool, message: str) -> None:
    if not value:
        raise AssertionError(message)


def task(task_id: str) -> dict[str, Any]:
    prompt = f"Build articulated test asset {task_id}."
    return {
        "task_id": task_id,
        "prompt": prompt,
        "prompt_sha256": runner.sha256_bytes(prompt.encode("utf-8")),
    }


def protocol() -> dict[str, Any]:
    return {
        "protocol_id": "table1_reliability_protocol_v1",
        "max_common_repair_turns": 3,
        "methods": {
            "articraft": {
                "provider": "openai",
                "model": "gpt-5",
                "reasoning_effort": "high",
                "native_settings": {
                    "max_turns": 100,
                    "openai_transport": "http",
                    "sdk_package": "sdk",
                    "max_cost_usd": None,
                    "cost_policy": "formal_global_budget_only",
                    "python_executable": "articraft_data/.venv/bin/python",
                    "python_version": "3.12.3",
                    "pyproject_sha256": "fd2cf4ddff0d8aaac5052bbfcadf09114cd70f3a1e9c9318936af22ef6c526be",
                    "uv_lock_sha256": "b58b12834c30a894ce4d7fdf6ae41e0fc2947fb3a10ff6acf653344223b9a0fc",
                },
                "request_parameters": {
                    "temperature": {
                        "sent": False,
                        "value": None,
                        "reason": "Articraft Responses provider does not expose or send this parameter",
                    },
                    "top_p": {
                        "sent": False,
                        "value": None,
                        "reason": "Articraft Responses provider does not expose or send this parameter",
                    },
                    "max_output_tokens": {
                        "sent": False,
                        "value": None,
                        "reason": "Articraft Responses provider does not expose or send an output-token cap",
                    },
                    "parallel_tool_calls": {"sent": True, "value": True},
                    "reasoning_summary": {"sent": True, "value": "auto"},
                    "store": {"sent": True, "value": False},
                },
            }
        },
        "timeouts": {
            "model_response_seconds": 1800,
            "execution_seconds_per_attempt": 180,
            "common_evaluator_seconds_per_attempt": 180,
            "native_retry_limit_per_attempt": 2,
        },
    }


def bindings(seed: str) -> dict[str, str]:
    return {
        key: runner.sha256_bytes(f"{seed}:{key}".encode("utf-8"))
        for key in (
            "protocol_sha256",
            "manifest_sha256",
            "hidden_specs_sha256",
            "package_schema_sha256",
            "result_schema_sha256",
            "common_evaluator_sha256",
            "adapter_sha256",
        )
    }


class MockExecutor:
    def __init__(self, statuses: list[str] | None = None, *, timeout: bool = False) -> None:
        self.statuses = statuses or ["success"]
        self.timeout = timeout
        self.calls: list[dict[str, Any]] = []

    def run(self, **kwargs: Any) -> Any:
        self.calls.append(kwargs)
        settings = kwargs["settings"]
        check(settings["provider"] == "openai", "provider drift")
        check(settings["model"] == "gpt-5", "model drift")
        check(settings["reasoning_effort"] == "high", "reasoning drift")
        check(settings["max_turns"] == 100, "max_turns drift")
        check(settings["native_retry_limit"] == 2, "native retry drift")
        if self.timeout:
            raise TimeoutError("mock timeout")
        index = kwargs["attempt_index"]
        status = self.statuses[min(index, len(self.statuses) - 1)]
        native_root = kwargs["native_root"]
        if kwargs["kind"] == "repair":
            check(kwargs["prior_native_root"] != native_root, "repair reused prior storage")
            check(kwargs["prior_record_id"] is not None, "repair lost parent record")
            check("bounded_cumulative_feedback" in kwargs["repair_prompt"], "repair lost feedback")
        native_root.mkdir(parents=True, exist_ok=False)
        source = native_root / "record/model.py"
        urdf = native_root / "cache/model.urdf"
        mesh = native_root / "cache/assets/meshes/part.stl"
        source.parent.mkdir(parents=True)
        urdf.parent.mkdir(parents=True)
        mesh.parent.mkdir(parents=True)
        source.write_text("VALUE = 1\n", encoding="utf-8")
        urdf.write_text(
            '<robot name="mock"><link name="base"><visual><geometry>'
            '<mesh filename="assets/meshes/part.stl"/>'
            '</geometry></visual></link></robot>\n',
            encoding="utf-8",
        )
        mesh.write_bytes(b"solid p\nendsolid p\n")
        now = runner.utc_now()
        return runner.NativeResult(
            status=status,
            exit_code=0 if status == "success" else 2,
            message=None if status == "success" else "mock native failure",
            run_id=f"run_a{index}",
            record_id=f"rec_a{index}",
            source_path=str(source),
            urdf_path=str(urdf),
            materialization_assets_path=str(native_root / "cache/assets"),
            record_assets_path=None,
            cost_path=None,
            request_started_at_utc=now,
            response_completed_at_utc=now,
            wall_time_seconds=0.01,
            native_retry_count=0,
        )


def mock_probe(source: Path | None, attempt_root: Path, timeout: float) -> dict[str, Any]:
    del timeout
    now = runner.utc_now()
    source_ok = source is not None and source.is_file()
    return {
        "started_at_utc": now,
        "finished_at_utc": now,
        "wall_time_s": 0.01,
        "exit_code": 0 if source_ok else 1,
        "timed_out": False,
        "source_sha256": runner.sha256_file(source) if source_ok else "0" * 64,
        "stdout_sha256": "0" * 64,
        "stderr_sha256": "1" * 64,
    }


class MockEvaluator:
    def __init__(self, verdicts: list[bool]) -> None:
        self.verdicts = verdicts
        self.calls = 0

    def __call__(self, package: Path, report: Path, timeout: float) -> dict[str, Any]:
        del timeout
        package_value = runner.read_object(package)
        passed = self.verdicts[min(self.calls, len(self.verdicts) - 1)]
        self.calls += 1
        value = {
            "binding_checks": {"all": True},
            "protocol_checks": {"all": True},
            "task_checks": {"all": True},
            "verdicts": {
                "executable": True,
                "artifact_saved": True,
                "urdf_tree_pass": passed,
                "semantic_roles_pass": passed,
                "joint_spec_pass": passed,
                "common_qc_pass": passed,
            },
            "feedback": {
                "schema_version": 1,
                "task_id": package_value["task_id"],
                "attempt_index": package_value["attempt_index"],
                "failure_codes": [] if passed else ["SEMANTIC_ROLES_FAILED"],
                "bounded_diagnostics": {"tree_link_count": 1},
                "policy": "bounded output-derived diagnostics only",
            },
        }
        runner.write_json(report, value)
        return value


def run_case(
    name: str,
    executor: MockExecutor,
    evaluator: MockEvaluator,
    *,
    pause_after: int | None = None,
) -> tuple[Path, dict[str, Any]]:
    root = TEST_ROOT / name
    value = runner.execute_job(
        task=task(name),
        repeat_id="r0",
        job_root=root,
        protocol=protocol(),
        bindings=bindings(name),
        executor=executor,
        evaluator_func=evaluator,
        probe_func=mock_probe,
        pause_after_new_attempts=pause_after,
        allow_test_unready=True,
    )
    runner.validate_schema(value, runner.RESULT_SCHEMA)
    return root, value


def main() -> int:
    if TEST_ROOT.exists():
        shutil.rmtree(TEST_ROOT)
    TEST_ROOT.mkdir(parents=True)
    checks: dict[str, bool] = {}
    try:
        fixture = subprocess.run(
            [
                str(runner.ARTICRAFT_PYTHON),
                "-c",
                """
import asyncio, json
from pathlib import Path
from agent.tools.write_code import WriteFileTool
from agent.workspace_docs import DocsBundle, VirtualWorkspace, VirtualWorkspaceFile
model = Path(__import__('sys').argv[1])
model.parent.mkdir(parents=True, exist_ok=True)
model.write_text('VALUE = 1\\n', encoding='utf-8')
router = VirtualWorkspaceFile(virtual_path='docs/sdk/references/quickstart.md', content='docs')
workspace = VirtualWorkspace(model_file_path=model, docs_bundle=DocsBundle(router=router, files_by_path={router.virtual_path: router}))
blocked = 0
for candidate in ('/mnt/zsn/lyb/arti-skill/exp/reference/table1_reliability_hidden_specs_v1.json', '../exp/reference/table1_reliability_hidden_specs_v1.json', 'exp/reference/table1_reliability_hidden_specs_v1.json'):
    try: workspace.resolve(candidate)
    except (ValueError, FileNotFoundError): blocked += 1
write_blocked = False
try: asyncio.run(WriteFileTool().build({'content':'VALUE = 2','path':'../hidden_specs.json'}))
except ValueError: write_blocked = True
print(json.dumps({'blocked_reads': blocked, 'write_blocked': write_blocked}))
""",
                str(TEST_ROOT / "tool_scope/model.py"),
            ],
            cwd=runner.ARTICRAFT_ROOT,
            env={**os.environ, **runner.THREAD_ENV, "PYTHONPATH": str(runner.ARTICRAFT_ROOT)},
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30,
            check=True,
        )
        fixture_value = json.loads(fixture.stdout)
        checks["native_read_tool_rejects_hidden_paths"] = fixture_value["blocked_reads"] == 3
        checks["native_write_tool_rejects_path_escape"] = fixture_value["write_blocked"] is True
        access = runner.author_access_audit()
        checks["model_execution_layer_remains_blocked"] = (
            access["virtual_read_tool_scope_restricted"] is True
            and access["authored_model_executes_with_regular_python"] is True
            and access["strong_read_isolation"] is False
        )
        checks["authored_python_host_and_credential_isolation_reported"] = (
            access["worker_environment_isolation"] is False
            and "credential" in access["reason"].lower()
            and "regular Python" in access["reason"]
        )
        retry_audit = runner.native_retry_contract_audit()
        checks["native_retry_contract_gap_reported"] = (
            retry_audit["per_common_attempt_limit_enforced"] is False
            and retry_audit["identical_request_retries_only"] is False
            and retry_audit["post_hoc_count_can_exceed_before_refusal"] is True
            and retry_audit["reasoning_summary_fallback_changes_request"] is True
        )
        ready_fixture = runner.execution_readiness(
            checks={},
            implementation={"checks": {}},
            protocol={
                "execution_ready": False,
                "execution_readiness": {
                    "status": "BLOCKED_CAPABILITIES",
                    "method_adapters_ready": {"articraft": False},
                },
            },
            access=access,
            retry_audit=retry_audit,
        )
        blocker_codes = {row["code"] for row in ready_fixture["blockers"]}
        checks["readiness_contains_both_capability_blockers"] = (
            "ARTICRAFT_AUTHOR_READ_ISOLATION_NOT_ENFORCED" in blocker_codes
            and "ARTICRAFT_NATIVE_RETRY_CONTRACT_NOT_ENFORCED" in blocker_codes
        )
        direct_gate_root = TEST_ROOT / "direct_readiness_gate"
        direct_gate_refused = False
        direct_gate_executor = MockExecutor()
        try:
            runner.execute_job(
                task=task("direct_readiness_gate"),
                repeat_id="r0",
                job_root=direct_gate_root,
                protocol=protocol(),
                bindings=bindings("direct_readiness_gate"),
                executor=direct_gate_executor,
                evaluator_func=MockEvaluator([True]),
                probe_func=mock_probe,
            )
        except RuntimeError:
            direct_gate_refused = True
        checks["direct_execution_requires_protocol_ready"] = (
            direct_gate_refused and not direct_gate_executor.calls and not direct_gate_root.exists()
        )

        first_root, first = run_case(
            "first_shot", MockExecutor(), MockEvaluator([True])
        )
        checks["first_shot_success"] = (
            first["summary"]["first_shot"] is True
            and first["summary"]["repair_turns"] == 0
            and (first_root / "attempts/a0/attempt_seal.json").is_file()
        )

        repair_executor = MockExecutor()
        repair_root, repaired = run_case(
            "repair_success", repair_executor, MockEvaluator([False, True])
        )
        checks["repair_success"] = (
            repaired["summary"]["final_success"] is True
            and repaired["summary"]["repair_turns"] == 1
            and [row["kind"] for row in repair_executor.calls] == ["initial", "repair"]
            and (repair_root / "attempts/a1/repair_input.json").is_file()
        )
        a0_source = repair_root / "attempts/a0/normalized/model.py"
        a0_hash = runner.sha256_file(a0_source)
        (repair_root / "attempts/a1/normalized/model.py").read_text(encoding="utf-8")
        checks["prior_attempt_immutable"] = runner.sha256_file(a0_source) == a0_hash

        failed_root, failed = run_case(
            "native_failure", MockExecutor(["failed"]), MockEvaluator([False])
        )
        checks["failure_retained"] = (
            failed["summary"]["final_success"] is False
            and (failed_root / "attempts/a0/failure.json").is_file()
            and (failed_root / "attempts/a0/attempt_seal.json").is_file()
        )

        timeout_root, timed_out = run_case(
            "model_timeout", MockExecutor(timeout=True), MockEvaluator([False])
        )
        checks["timeout_retained"] = (
            timed_out["attempts"][0]["failure_class"] == "model_timeout"
            and (timeout_root / "attempts/a0/failure.json").is_file()
        )

        resume_name = "resume"
        resume_root = TEST_ROOT / resume_name
        resume_exec = MockExecutor()
        resume_eval = MockEvaluator([False, True])
        try:
            runner.execute_job(
                task=task(resume_name), repeat_id="r0", job_root=resume_root,
                protocol=protocol(), bindings=bindings(resume_name), executor=resume_exec,
                evaluator_func=resume_eval, probe_func=mock_probe,
                pause_after_new_attempts=1,
                allow_test_unready=True,
            )
            raise AssertionError("checkpoint did not pause")
        except runner.ExecutionPaused:
            pass
        resumed = runner.execute_job(
            task=task(resume_name), repeat_id="r0", job_root=resume_root,
            protocol=protocol(), bindings=bindings(resume_name), executor=resume_exec,
            evaluator_func=resume_eval, probe_func=mock_probe,
            allow_test_unready=True,
        )
        checks["partial_resume"] = (
            resumed["summary"]["final_success"] is True
            and len(resumed["attempts"]) == 2
            and len(resume_exec.calls) == 2
        )
        resumed_again = runner.execute_job(
            task=task(resume_name), repeat_id="r0", job_root=resume_root,
            protocol=protocol(), bindings=bindings(resume_name), executor=MockExecutor(),
            evaluator_func=MockEvaluator([True]), probe_func=mock_probe,
            allow_test_unready=True,
        )
        checks["completed_resume_zero_execution"] = resumed_again == resumed

        stale_refused = False
        try:
            runner.execute_job(
                task=task(resume_name), repeat_id="r0", job_root=resume_root,
                protocol=protocol(), bindings=bindings("different"), executor=MockExecutor(),
                evaluator_func=MockEvaluator([True]), probe_func=mock_probe,
                allow_test_unready=True,
            )
        except RuntimeError:
            stale_refused = True
        checks["stale_binding_refused"] = stale_refused

        swapped_source_root, _ = run_case(
            "swapped_source", MockExecutor(), MockEvaluator([True])
        )
        swapped_bindings = bindings("swapped_source")
        swapped_target_root = TEST_ROOT / "swapped_target"
        swapped_source_root.rename(swapped_target_root)
        swapped_executor = MockExecutor()
        swapped_refused = False
        try:
            runner.execute_job(
                task=task("swapped_target"), repeat_id="r0", job_root=swapped_target_root,
                protocol=protocol(), bindings=swapped_bindings, executor=swapped_executor,
                evaluator_func=MockEvaluator([True]), probe_func=mock_probe,
                allow_test_unready=True,
            )
        except RuntimeError:
            swapped_refused = True
        checks["cross_job_swapped_sealed_result_refused"] = (
            swapped_refused and not swapped_executor.calls
        )

        collision = TEST_ROOT / "collision"
        collision.mkdir()
        (collision / "foreign.txt").write_text("foreign\n", encoding="utf-8")
        collision_refused = False
        try:
            runner.execute_job(
                task=task("collision"), repeat_id="r0", job_root=collision,
                protocol=protocol(), bindings=bindings("collision"), executor=MockExecutor(),
                evaluator_func=MockEvaluator([True]), probe_func=mock_probe,
                allow_test_unready=True,
            )
        except RuntimeError:
            collision_refused = True
        checks["path_collision_refused"] = collision_refused

        modified_refused = False
        (resume_root / "attempts/a0/bounded_feedback.json").write_text("{}\n", encoding="utf-8")
        try:
            runner.execute_job(
                task=task(resume_name), repeat_id="r0", job_root=resume_root,
                protocol=protocol(), bindings=bindings(resume_name), executor=MockExecutor(),
                evaluator_func=MockEvaluator([True]), probe_func=mock_probe,
                allow_test_unready=True,
            )
        except RuntimeError:
            modified_refused = True
        checks["modified_output_refused"] = modified_refused

        denominator_fixture = runner.build_authoring_summary(
            status="EXECUTED",
            results=[
                {"summary": {"state": "observed", "final_success": True}},
                {"summary": {"state": "not_evaluable", "final_success": None}},
            ],
            prepared_jobs=runner.EXPECTED_JOB_COUNT,
            execution_ready=True,
            blocker_codes=[],
        )
        final_success = denominator_fixture["metrics"]["final_success"]
        checks["final_success_uses_intent_denominator"] = (
            final_success["numerator"] == 1
            and final_success["denominator"] == 162
            and final_success["value"] == 1 / 162
        )
        checks["attempted_and_evaluable_reported_separately"] = (
            denominator_fixture["attempted_jobs"] == 2
            and denominator_fixture["evaluable_jobs"] == 1
        )
        checks["summary_has_accurate_claim_boundary"] = (
            isinstance(denominator_fixture["claim_boundary"], str)
            and "162 intent runs" in denominator_fixture["claim_boundary"]
            and "attempted_jobs" in denominator_fixture["claim_boundary"]
            and "evaluable_jobs" in denominator_fixture["claim_boundary"]
        )

        check(all(checks.values()), json.dumps(checks, sort_keys=True))
        print(json.dumps({
            "status": "PASS",
            "provider_calls_made": 0,
            "checks_passed": sum(checks.values()),
            "checks_total": len(checks),
            "checks": checks,
        }, sort_keys=True))
        return 0
    finally:
        shutil.rmtree(TEST_ROOT, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
