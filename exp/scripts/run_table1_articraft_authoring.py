#!/usr/bin/env python3
"""Prepare or execute the frozen Nano3D Table 1 Articraft authoring arm.

Preparation is local and makes no provider calls. Paid execution requires both
``--execute`` and ``--acknowledge-paid-run`` and remains fail-closed until the
frozen protocol marks the Articraft adapter ready. Each native attempt uses an
independent experiment repository root; the pinned Articraft checkout is only
imported as code and is never used as run storage.
"""

from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import logging
import os
import re
import shutil
import subprocess
import sys
import time
import traceback
import uuid
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Protocol


REPO_ROOT = Path(__file__).resolve().parents[2]
EXP_ROOT = REPO_ROOT / "exp"
DEFAULT_MANIFEST = EXP_ROOT / "reference/table1_reliability_common_authoring_v1.json"
DEFAULT_PROTOCOL = EXP_ROOT / "reference/table1_reliability_protocol_v1.json"
DEFAULT_OUT = EXP_ROOT / "runtime/table1_reliability/articraft_authoring_v1"
DEFAULT_RUN_ROOT = EXP_ROOT / "runtime/table1_reliability/common_authoring/articraft"
ARTICRAFT_ROOT = REPO_ROOT / "articraft_data"
ARTICRAFT_ENTRYPOINT = ARTICRAFT_ROOT / "agent/runner_cli.py"
ARTICRAFT_PYTHON = ARTICRAFT_ROOT / ".venv/bin/python"
ARTICRAFT_PYTHON_BINDING = "articraft_data/.venv/bin/python"
ARTICRAFT_PYPROJECT = ARTICRAFT_ROOT / "pyproject.toml"
ARTICRAFT_UV_LOCK = ARTICRAFT_ROOT / "uv.lock"
COMMON_EVALUATOR = EXP_ROOT / "scripts/evaluate_table1_authoring_common.py"
PACKAGE_SCHEMA = EXP_ROOT / "reference/table1_authoring_package_schema_v1.json"
RESULT_SCHEMA = EXP_ROOT / "reference/table1_authoring_result_schema_v1.json"

METHOD_ID = "articraft"
EXPECTED_TASK_COUNT = 54
EXPECTED_REPEAT_IDS = ["r0", "r1", "r2"]
EXPECTED_JOB_COUNT = EXPECTED_TASK_COUNT * len(EXPECTED_REPEAT_IDS)
EXPECTED_PROVIDER = "openai"
EXPECTED_MODEL = "gpt-5"
EXPECTED_REASONING_EFFORT = "high"
EXPECTED_MAX_TURNS = 100
EXPECTED_PYTHON_VERSION = "3.12.3"
EXPECTED_PYPROJECT_SHA256 = "fd2cf4ddff0d8aaac5052bbfcadf09114cd70f3a1e9c9318936af22ef6c526be"
EXPECTED_UV_LOCK_SHA256 = "b58b12834c30a894ce4d7fdf6ae41e0fc2947fb3a10ff6acf653344223b9a0fc"
EXPECTED_REPAIR_TURNS = 3
EXPECTED_ARTICRAFT_COMMIT = "06cd75fbc9e90fae33f127b494d13c35090356be"
EXPECTED_ARTICRAFT_TREE = "e9cfec5562e4a518bf8f57ee091c7255249431ea"
THREAD_ENV = {
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}
HEX64 = re.compile(r"^[0-9a-f]{64}$")
SAFE_COMPONENT = re.compile(r"^[A-Za-z0-9_.-]+$")
NATIVE_WORKER_ENV = "NANO3D_TABLE1_ARTICRAFT_WORKER_TOKEN"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def contained_repo(path: Path, *, must_exist: bool = False) -> Path:
    resolved = path.expanduser().resolve(strict=must_exist)
    if resolved != REPO_ROOT and REPO_ROOT not in resolved.parents:
        raise ValueError(f"path escapes repository root: {resolved}")
    return resolved


def contained_runtime(path: Path, *, must_exist: bool = False) -> Path:
    resolved = contained_repo(path, must_exist=must_exist)
    runtime_root = (EXP_ROOT / "runtime/table1_reliability").resolve()
    if resolved != runtime_root and runtime_root not in resolved.parents:
        raise ValueError(f"output path escapes Table 1 runtime root: {resolved}")
    return resolved


def contained_child(path: Path, root: Path, *, must_exist: bool = False) -> Path:
    resolved = path.resolve(strict=must_exist)
    resolved.relative_to(root.resolve())
    return resolved


def rel(path: Path, root: Path = REPO_ROOT) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    raw = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return sha256_bytes(raw)


def read_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def read_json_snapshot(path: Path) -> tuple[dict[str, Any], bytes, str]:
    path = contained_repo(path, must_exist=True)
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"expected regular non-symlink JSON file: {path}")
    raw = path.read_bytes()
    value = json.loads(raw.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value, raw, sha256_bytes(raw)


def write_text_atomic(path: Path, value: str) -> None:
    path = contained_runtime(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + f".tmp.{os.getpid()}.{uuid.uuid4().hex}")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def write_json(path: Path, value: Any) -> None:
    write_text_atomic(
        path,
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    )


def validate_schema(value: dict[str, Any], schema_path: Path) -> None:
    import jsonschema  # type: ignore[import-not-found]

    schema = read_object(schema_path)
    jsonschema.Draft202012Validator.check_schema(schema)
    jsonschema.Draft202012Validator(schema).validate(value)


def git_value(root: Path, args: list[str]) -> str | None:
    try:
        completed = subprocess.run(
            ["git", *args],
            cwd=root,
            env={**os.environ, **THREAD_ENV},
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return completed.stdout.strip() if completed.returncode == 0 else None


def is_hash(value: Any) -> bool:
    return isinstance(value, str) and HEX64.fullmatch(value) is not None


def selected_method(protocol: dict[str, Any]) -> dict[str, Any]:
    methods = protocol.get("methods")
    if isinstance(methods, dict) and isinstance(methods.get(METHOD_ID), dict):
        return methods[METHOD_ID]
    if isinstance(methods, list):
        for row in methods:
            if isinstance(row, dict) and row.get("method_id") == METHOD_ID:
                return row
    raise ValueError("protocol has no Articraft method binding")


def binding_path_and_hash(section: Any) -> tuple[str | None, str | None]:
    if not isinstance(section, dict):
        return None, None
    raw_path = section.get("path", section.get("entrypoint"))
    raw_hash = section.get("sha256", section.get("expected_sha256"))
    return (
        raw_path if isinstance(raw_path, str) else None,
        raw_hash if isinstance(raw_hash, str) else None,
    )


def expected_binding(protocol: dict[str, Any], name: str, path: Path) -> bool:
    bound_path, bound_hash = binding_path_and_hash(protocol.get(name))
    return bound_path == rel(path) and bound_hash == sha256_file(path)


def model_binding(protocol: dict[str, Any], method: dict[str, Any]) -> dict[str, Any]:
    common = protocol.get("common_model_binding")
    common = common if isinstance(common, dict) else {}
    return {
        "provider": method.get("provider", common.get("provider")),
        "model": method.get(
            "model", method.get("model_id", common.get("model", common.get("model_id")))
        ),
        "reasoning_effort": method.get(
            "reasoning_effort",
            method.get("reasoning", common.get("reasoning_effort", common.get("reasoning"))),
        ),
    }


def native_settings(method: dict[str, Any]) -> dict[str, Any]:
    value = method.get("native_settings")
    return value if isinstance(value, dict) else {}


def exact_parameter(
    value: Any, *, sent: bool, expected: Any, reason: str | None = None
) -> bool:
    expected_keys = {"sent", "value"} if sent else {"sent", "value", "reason"}
    return (
        isinstance(value, dict)
        and value.get("sent") is sent
        and value.get("value") == expected
        and (sent or value.get("reason") == reason)
        and set(value) == expected_keys
    )


def python_version(python: Path) -> str | None:
    if not python.is_file():
        return None
    try:
        completed = subprocess.run(
            [str(python), "-c", "import platform; print(platform.python_version())"],
            cwd=ARTICRAFT_ROOT,
            env={**os.environ, **THREAD_ENV},
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    return completed.stdout.strip() if completed.returncode == 0 else None


def frozen_checks(
    manifest: dict[str, Any], manifest_sha: str, protocol: dict[str, Any], method: dict[str, Any]
) -> dict[str, bool]:
    tasks = manifest.get("tasks")
    task_ids = [
        row.get("task_id")
        for row in tasks or []
        if isinstance(row, dict) and isinstance(row.get("task_id"), str)
    ]
    prompts_valid = isinstance(tasks, list) and bool(tasks) and all(
        isinstance(row, dict)
        and isinstance(row.get("prompt"), str)
        and bool(row["prompt"].strip())
        and row.get("prompt_sha256") == sha256_bytes(row["prompt"].encode("utf-8"))
        for row in tasks
    )
    model = model_binding(protocol, method)
    settings = native_settings(method)
    parameters = method.get("request_parameters")
    parameters = parameters if isinstance(parameters, dict) else {}
    return {
        "manifest_frozen": manifest.get("frozen") is True,
        "manifest_task_count_54": isinstance(tasks, list)
        and len(tasks) == EXPECTED_TASK_COUNT
        and manifest.get("task_count") == EXPECTED_TASK_COUNT,
        "manifest_task_ids_unique": len(task_ids) == EXPECTED_TASK_COUNT
        and len(set(task_ids)) == EXPECTED_TASK_COUNT,
        "manifest_prompt_hashes": prompts_valid,
        "manifest_repeat_ids_exact": manifest.get("repeat_ids") == EXPECTED_REPEAT_IDS,
        "protocol_manifest_hash_bound": protocol.get("manifest", {}).get("sha256")
        == manifest_sha,
        "protocol_frozen_design": protocol.get("frozen_design") is True,
        "protocol_expected_task_count_54": protocol.get("expected_task_count")
        == EXPECTED_TASK_COUNT,
        "protocol_repeat_ids_exact": protocol.get("repeat_ids") == EXPECTED_REPEAT_IDS,
        "protocol_independent_runs_3": protocol.get("independent_runs_per_task") == 3,
        "protocol_expected_runs_162": protocol.get("expected_runs_per_method")
        == EXPECTED_JOB_COUNT,
        "protocol_repair_budget_3": protocol.get("max_common_repair_turns")
        == EXPECTED_REPAIR_TURNS,
        "protocol_provider_openai": model["provider"] == EXPECTED_PROVIDER,
        "protocol_model_gpt5": model["model"] == EXPECTED_MODEL,
        "protocol_reasoning_high": model["reasoning_effort"]
        == EXPECTED_REASONING_EFFORT,
        "protocol_native_max_turns_100": settings.get("max_turns")
        == EXPECTED_MAX_TURNS,
        "protocol_native_transport_http": settings.get("openai_transport", "http")
        == "http",
        "protocol_native_sdk_package": settings.get("sdk_package", "sdk") == "sdk",
        "protocol_native_cost_policy": settings.get("max_cost_usd") is None
        and settings.get("cost_policy") == "formal_global_budget_only",
        "protocol_native_python_executable": settings.get("python_executable")
        == ARTICRAFT_PYTHON_BINDING
        and ARTICRAFT_PYTHON.is_file(),
        "protocol_native_python_version": settings.get("python_version")
        == EXPECTED_PYTHON_VERSION
        == python_version(ARTICRAFT_PYTHON),
        "protocol_native_pyproject_hash": settings.get("pyproject_sha256")
        == EXPECTED_PYPROJECT_SHA256
        == sha256_file(ARTICRAFT_PYPROJECT),
        "protocol_native_uv_lock_hash": settings.get("uv_lock_sha256")
        == EXPECTED_UV_LOCK_SHA256
        == sha256_file(ARTICRAFT_UV_LOCK),
        "protocol_request_temperature_unset": exact_parameter(
            parameters.get("temperature"),
            sent=False,
            expected=None,
            reason="Articraft Responses provider does not expose or send this parameter",
        ),
        "protocol_request_top_p_unset": exact_parameter(
            parameters.get("top_p"),
            sent=False,
            expected=None,
            reason="Articraft Responses provider does not expose or send this parameter",
        ),
        "protocol_request_max_output_tokens_unset": exact_parameter(
            parameters.get("max_output_tokens"),
            sent=False,
            expected=None,
            reason="Articraft Responses provider does not expose or send an output-token cap",
        ),
        "protocol_request_parallel_tool_calls_true": exact_parameter(
            parameters.get("parallel_tool_calls"), sent=True, expected=True
        ),
        "protocol_request_reasoning_summary_auto": exact_parameter(
            parameters.get("reasoning_summary"), sent=True, expected="auto"
        ),
        "protocol_request_store_false": exact_parameter(
            parameters.get("store"), sent=True, expected=False
        ),
        "protocol_native_retry_limit_2": protocol.get("timeouts", {}).get(
            "native_retry_limit_per_attempt"
        )
        == 2,
        "protocol_hidden_specs_hash_frozen": is_hash(
            protocol.get("hidden_specs", {}).get("sha256")
        ),
        "protocol_hidden_specs_evaluator_only": protocol.get("hidden_specs", {}).get(
            "visibility"
        )
        == "evaluator_only"
        or protocol.get("hidden_specs", {}).get("withheld_from_author") is True,
        "protocol_common_evaluator_bound": expected_binding(
            protocol, "common_evaluator", COMMON_EVALUATOR
        ),
        "protocol_package_schema_bound": expected_binding(
            protocol, "package_schema", PACKAGE_SCHEMA
        ),
        "protocol_result_schema_bound": expected_binding(protocol, "result_schema", RESULT_SCHEMA),
    }


def implementation_binding(method: dict[str, Any], adapter_path: Path) -> dict[str, Any]:
    implementation = method.get("implementation")
    implementation = implementation if isinstance(implementation, dict) else {}
    actual = {
        "commit": git_value(ARTICRAFT_ROOT, ["rev-parse", "HEAD"]),
        "git_tree": git_value(ARTICRAFT_ROOT, ["rev-parse", "HEAD^{tree}"]),
        "tracked_status": git_value(
            ARTICRAFT_ROOT, ["status", "--porcelain", "--untracked-files=no"]
        ),
        "adapter_sha256": sha256_file(adapter_path),
    }
    checks = {
        "implementation_checkout_path": implementation.get("checkout_path")
        == rel(ARTICRAFT_ROOT),
        "implementation_commit": implementation.get("commit")
        == EXPECTED_ARTICRAFT_COMMIT
        == actual["commit"],
        "implementation_git_tree": implementation.get("git_tree")
        == EXPECTED_ARTICRAFT_TREE
        == actual["git_tree"],
        "implementation_entrypoint": implementation.get("entrypoint")
        == rel(ARTICRAFT_ENTRYPOINT)
        and ARTICRAFT_ENTRYPOINT.is_file()
        and not ARTICRAFT_ENTRYPOINT.is_symlink(),
        "implementation_tracked_clean": implementation.get("tracked_clean_at_freeze")
        is True
        and actual["tracked_status"] == "",
        "implementation_provenance": isinstance(implementation.get("provenance"), str)
        and bool(implementation.get("provenance", "").strip()),
        "adapter_entrypoint": method.get("adapter_entrypoint") == rel(adapter_path),
        "adapter_sha256": method.get("adapter_sha256") == actual["adapter_sha256"],
    }
    return {
        "declared": {
            **implementation,
            "adapter_entrypoint": method.get("adapter_entrypoint"),
            "adapter_sha256": method.get("adapter_sha256"),
        },
        "actual": actual,
        "checks": checks,
    }


def author_access_audit() -> dict[str, Any]:
    workspace_path = ARTICRAFT_ROOT / "agent/workspace_docs.py"
    compiler_path = ARTICRAFT_ROOT / "agent/compiler.py"
    read_tool_path = ARTICRAFT_ROOT / "agent/tools/read_file.py"
    workspace = workspace_path.read_text(encoding="utf-8")
    compiler = compiler_path.read_text(encoding="utf-8")
    read_tool = read_tool_path.read_text(encoding="utf-8")
    virtual_read_scope = all(
        token in workspace
        for token in (
            'candidate.startswith("/")',
            'any(part == ".." for part in parts)',
            "if normalized == _MODEL_VIRTUAL_PATH",
            "return self.docs_bundle.resolve(normalized)",
        )
    ) and "self.virtual_workspace.resolve(self.params.path)" in read_tool
    authored_code_regular_python = all(
        token in compiler
        for token in (
            "os.chdir(script_path.parent)",
            "runpy.run_path(script_path.name)",
        )
    )
    return {
        "virtual_read_tool_scope_restricted": virtual_read_scope,
        "virtual_read_tool_absolute_and_parent_paths_rejected": virtual_read_scope,
        "authored_model_executes_with_regular_python": authored_code_regular_python,
        "worker_environment_isolation": False,
        "worker_os_filesystem_read_sandbox": False,
        "generation_access_to_evaluator_only_files_prevented": False,
        "strong_read_isolation": False,
        "evidence": {
            "virtual_workspace": {
                "path": rel(workspace_path),
                "sha256": sha256_file(workspace_path),
            },
            "read_tool": {
                "path": rel(read_tool_path),
                "sha256": sha256_file(read_tool_path),
            },
            "compiler": {
                "path": rel(compiler_path),
                "sha256": sha256_file(compiler_path),
            },
        },
        "reason": (
            "The author-facing read_file tool is virtual and contained, but compile/probe "
            "executes authored model.py with regular Python in a worker that inherits provider "
            "credentials. Without an OS read sandbox, authored code can read evaluator-only, "
            "credential, or other-method paths and write outside its attempt root."
        ),
    }


def request_payload_audit() -> dict[str, Any]:
    provider_path = ARTICRAFT_ROOT / "agent/providers/openai.py"
    source = provider_path.read_text(encoding="utf-8")
    start = source.index("    def _build_request_payload(")
    end = source.index("    def _build_token_count_payload(", start)
    payload_source = source[start:end]
    checks = {
        "temperature_not_sent": '"temperature"' not in payload_source,
        "top_p_not_sent": '"top_p"' not in payload_source,
        "max_output_tokens_not_sent": '"max_output_tokens"' not in payload_source,
        "parallel_tool_calls_true": '"parallel_tool_calls": True' in payload_source,
        "reasoning_summary_sent_when_auto": (
            'if self.reasoning_summary:' in payload_source
            and 'reasoning["summary"] = self.reasoning_summary' in payload_source
        ),
        "store_uses_transport_default": (
            '"store": self.store' in payload_source
            and 'self.store = self.transport == "websocket" if store is None else bool(store)'
            in source
        ),
    }
    return {
        "path": rel(provider_path),
        "sha256": sha256_file(provider_path),
        "checks": checks,
        "pass": all(checks.values()),
    }


def native_retry_contract_audit() -> dict[str, Any]:
    provider_path = ARTICRAFT_ROOT / "agent/providers/openai.py"
    runner_source = Path(__file__).resolve().read_text(encoding="utf-8")
    provider_source = provider_path.read_text(encoding="utf-8")
    per_request_retry = (
        "response = await _async_retry(" in provider_source
        and "max_attempts=self.max_attempts" in provider_source
        and 'env["OPENAI_MAX_ATTEMPTS"] = str(int(settings["native_retry_limit"]) + 1)'
        in runner_source
    )
    post_hoc_rejection = (
        'if native.native_retry_count > execution_settings["native_retry_limit"]:'
        in runner_source
    )
    summary_fallback = all(
        token in provider_source
        for token in (
            "_is_reasoning_summary_unsupported_error",
            "_payload_without_reasoning_summary(request_payload)",
            "return await self._request_with_transport(",
        )
    )
    return {
        "per_provider_request_retry_limit": per_request_retry,
        "per_common_attempt_limit_enforced": False,
        "post_hoc_count_can_exceed_before_refusal": post_hoc_rejection,
        "reasoning_summary_fallback_changes_request": summary_fallback,
        "identical_request_retries_only": False,
        "evidence": {
            "provider_path": rel(provider_path),
            "provider_sha256": sha256_file(provider_path),
            "adapter_sha256": sha256_file(Path(__file__).resolve()),
        },
        "reason": (
            "Articraft applies the frozen retry limit per provider request rather than per "
            "common attempt, checks cumulative retries only after the native session, and can "
            "resend a different payload without reasoning.summary; the total identical-request "
            "native-retry budget is not enforced."
        ),
    }


def protocol_method_ready(protocol: dict[str, Any]) -> bool:
    readiness = protocol.get("execution_readiness")
    readiness = readiness if isinstance(readiness, dict) else {}
    methods = readiness.get("method_adapters_ready")
    return (
        protocol.get("execution_ready") is True
        and readiness.get("status") in {"READY", "EXECUTION_READY"}
        and isinstance(methods, dict)
        and methods.get(METHOD_ID) is True
    )


def build_bindings(
    manifest_sha: str, protocol_sha: str, protocol: dict[str, Any]
) -> dict[str, str]:
    return {
        "protocol_sha256": protocol_sha,
        "manifest_sha256": manifest_sha,
        "hidden_specs_sha256": str(protocol.get("hidden_specs", {}).get("sha256") or ""),
        "package_schema_sha256": sha256_file(PACKAGE_SCHEMA),
        "result_schema_sha256": sha256_file(RESULT_SCHEMA),
        "common_evaluator_sha256": sha256_file(COMMON_EVALUATOR),
        "adapter_sha256": sha256_file(Path(__file__).resolve()),
    }


def attempt_plan(job_root: Path, max_repairs: int) -> list[dict[str, Any]]:
    return [
        {
            "attempt_index": index,
            "attempt_kind": "attempt_0" if index == 0 else "common_repair",
            "attempt_root": rel(job_root / "attempts" / f"a{index}"),
            "native_repo_root": rel(job_root / "attempts" / f"a{index}" / "native_repo"),
            "normalized_source": rel(
                job_root / "attempts" / f"a{index}" / "normalized/model.py"
            ),
            "normalized_urdf": rel(
                job_root / "attempts" / f"a{index}" / "normalized/model.urdf"
            ),
            "package": rel(job_root / "attempts" / f"a{index}" / "package.json"),
            "common_evaluator_report": rel(
                job_root / "attempts" / f"a{index}" / "common_evaluator_report.json"
            ),
            "bounded_feedback": rel(
                job_root / "attempts" / f"a{index}" / "bounded_feedback.json"
            ),
            "attempt_seal": rel(
                job_root / "attempts" / f"a{index}" / "attempt_seal.json"
            ),
            "immutable_after_attempt": True,
        }
        for index in range(max_repairs + 1)
    ]


def build_jobs(
    tasks: list[dict[str, Any]], repeat_ids: list[str], run_root: Path,
    bindings: dict[str, str], method_provenance: dict[str, Any], max_repairs: int,
) -> list[dict[str, Any]]:
    jobs: list[dict[str, Any]] = []
    for ordinal, (task, repeat_id) in enumerate(
        (task, repeat_id) for task in tasks for repeat_id in repeat_ids
    ):
        job_id = f"articraft__{task['task_id']}__{repeat_id}"
        job_root = run_root / task["task_id"] / repeat_id
        jobs.append(
            {
                "ordinal": ordinal,
                "job_id": job_id,
                "method_id": METHOD_ID,
                "task_id": task["task_id"],
                "repeat_id": repeat_id,
                "prompt_sha256": task["prompt_sha256"],
                "run_root": rel(job_root),
                "result": rel(job_root / "result.json"),
                "bindings": bindings,
                "method_provenance": method_provenance,
                "attempts": attempt_plan(job_root, max_repairs),
            }
        )
    return jobs


def validate_job_plan(jobs: list[dict[str, Any]]) -> dict[str, bool]:
    ids = [row["job_id"] for row in jobs]
    roots = [row["run_root"] for row in jobs]
    attempts = [a["attempt_root"] for row in jobs for a in row["attempts"]]
    native = [a["native_repo_root"] for row in jobs for a in row["attempts"]]
    return {
        "job_count_162": len(jobs) == EXPECTED_JOB_COUNT,
        "job_ids_unique": len(ids) == len(set(ids)),
        "job_roots_unique": len(roots) == len(set(roots)),
        "four_attempts_per_job": all(len(row["attempts"]) == 4 for row in jobs),
        "attempt_roots_unique": len(attempts) == len(set(attempts)),
        "native_repo_roots_unique": len(native) == len(set(native)),
        "attempts_immutable": all(
            attempt["immutable_after_attempt"] for row in jobs for attempt in row["attempts"]
        ),
    }


def output_collisions(jobs: list[dict[str, Any]]) -> list[str]:
    collisions = []
    for job in jobs:
        root = contained_runtime(REPO_ROOT / job["run_root"])
        if root.is_symlink() or (root.exists() and (not root.is_dir() or any(root.iterdir()))):
            collisions.append(job["run_root"])
    return collisions


def tree_manifest(root: Path, *, excluded: set[str] | None = None) -> dict[str, str]:
    excluded = excluded or set()
    rows: dict[str, str] = {}
    if not root.is_dir() or root.is_symlink():
        raise ValueError(f"expected regular directory: {root}")
    for path in sorted(root.rglob("*")):
        relative = rel(path, root)
        if relative in excluded:
            continue
        if path.is_symlink():
            raise ValueError(f"symlinks are forbidden in sealed output: {path}")
        if path.is_file():
            rows[relative] = sha256_file(path)
    return rows


def tree_digest(rows: dict[str, str]) -> str:
    return canonical_sha256(rows)


def seal_attempt(
    attempt_root: Path, bindings: dict[str, str], attempt_record_sha256: str
) -> dict[str, Any]:
    seal_path = attempt_root / "attempt_seal.json"
    if seal_path.exists():
        raise RuntimeError(f"attempt already sealed: {attempt_root}")
    files = tree_manifest(attempt_root, excluded={"attempt_seal.json"})
    seal = {
        "schema_version": "table1_articraft_attempt_seal_v1",
        "sealed_at_utc": utc_now(),
        "bindings": bindings,
        "attempt_record_sha256": attempt_record_sha256,
        "files": files,
        "output_tree_sha256": tree_digest(files),
    }
    write_json(seal_path, seal)
    return seal


def validate_attempt_seal(attempt_root: Path, bindings: dict[str, str]) -> dict[str, Any]:
    seal_path = attempt_root / "attempt_seal.json"
    if not seal_path.is_file() or seal_path.is_symlink():
        raise RuntimeError(f"missing immutable attempt seal: {seal_path}")
    seal = read_object(seal_path)
    if seal.get("bindings") != bindings:
        raise RuntimeError(f"stale attempt bindings: {attempt_root}")
    actual = tree_manifest(attempt_root, excluded={"attempt_seal.json"})
    if seal.get("files") != actual or seal.get("output_tree_sha256") != tree_digest(actual):
        raise RuntimeError(f"immutable attempt output changed: {attempt_root}")
    return seal


def seal_result(job_root: Path, result_path: Path, bindings: dict[str, str]) -> None:
    files = tree_manifest(job_root, excluded={"result_seal.json"})
    write_json(
        job_root / "result_seal.json",
        {
            "schema_version": "table1_articraft_result_seal_v1",
            "sealed_at_utc": utc_now(),
            "bindings": bindings,
            "result_sha256": sha256_file(result_path),
            "files": files,
            "output_tree_sha256": tree_digest(files),
        },
    )


def validate_completed_result(
    job_root: Path,
    bindings: dict[str, str],
    *,
    expected_run_id: str,
    expected_method_id: str,
    expected_task_id: str,
    expected_repeat_id: str,
) -> dict[str, Any]:
    result_path = job_root / "result.json"
    seal_path = job_root / "result_seal.json"
    if not result_path.is_file() or not seal_path.is_file():
        raise RuntimeError(f"result/resume seal pair incomplete: {job_root}")
    result = read_object(result_path)
    validate_schema(result, RESULT_SCHEMA)
    if result.get("bindings") != bindings:
        raise RuntimeError(f"refusing stale result bindings: {result_path}")
    expected_identity = {
        "run_id": expected_run_id,
        "method_id": expected_method_id,
        "task_id": expected_task_id,
        "repeat_id": expected_repeat_id,
    }
    actual_identity = {key: result.get(key) for key in expected_identity}
    if actual_identity != expected_identity:
        raise RuntimeError(f"refusing result for a different job: {result_path}")
    seal = read_object(seal_path)
    actual = tree_manifest(job_root, excluded={"result_seal.json"})
    if (
        seal.get("bindings") != bindings
        or seal.get("result_sha256") != sha256_file(result_path)
        or seal.get("files") != actual
        or seal.get("output_tree_sha256") != tree_digest(actual)
    ):
        raise RuntimeError(f"refusing changed or stale result output: {job_root}")
    for attempt in result["attempts"]:
        validate_attempt_seal(
            job_root / "attempts" / f"a{attempt['attempt_index']}", bindings
        )
    return result


@dataclass
class NativeResult:
    status: str
    exit_code: int
    message: str | None
    run_id: str
    record_id: str
    source_path: str | None
    urdf_path: str | None
    materialization_assets_path: str | None
    record_assets_path: str | None
    cost_path: str | None
    request_started_at_utc: str
    response_completed_at_utc: str
    wall_time_seconds: float
    native_retry_count: int
    timed_out: bool = False


class NativeExecutor(Protocol):
    def run(
        self, *, kind: str, native_root: Path, task: dict[str, Any], repeat_id: str,
        attempt_index: int, prior_native_root: Path | None, prior_record_id: str | None,
        repair_prompt: str | None, settings: dict[str, Any], timeout: float,
    ) -> NativeResult: ...


class ExecutionPaused(RuntimeError):
    """Test/operations checkpoint after a fully sealed attempt."""


def _native_worker(request_path: Path) -> int:
    request = read_object(contained_runtime(request_path, must_exist=True))
    token = os.environ.get(NATIVE_WORKER_ENV)
    if not token or request.get("worker_token_sha256") != sha256_bytes(token.encode("utf-8")):
        raise RuntimeError("native worker paid-execution token missing or mismatched")
    native_root = contained_runtime(Path(str(request["native_root"])))
    response_path = contained_runtime(Path(str(request["response_path"])))
    kind = str(request["kind"])
    if kind == "initial":
        native_root.mkdir(parents=True, exist_ok=False)
    elif kind == "repair":
        if not native_root.is_dir() or native_root.is_symlink():
            raise RuntimeError("repair worker requires a copied independent native repo")
    else:
        raise ValueError(f"unknown native worker kind: {kind}")
    sys.path.insert(0, str(ARTICRAFT_ROOT))
    started = utc_now()
    start_wall = time.monotonic()
    retry_count = 0
    try:
        from agent.edit import edit_record
        from agent.run_context import _read_logged_cost_totals
        from agent.single_run import run_from_input_impl
        from agent.tools import build_initial_user_content
        from storage.repo import StorageRepo
        from storage.revisions import active_cost_path, active_model_path

        class RetryCounter(logging.Handler):
            count = 0

            def emit(self, record: logging.LogRecord) -> None:
                message = record.getMessage()
                if "failed (attempt " in message and "retrying in " in message:
                    self.count += 1

        retry_counter = RetryCounter()
        logging.getLogger("agent.providers.openai").addHandler(retry_counter)
        record_id = str(request["record_id"])
        run_id = str(request["run_id"])
        settings = request["settings"]
        if kind == "initial":
            prompt = str(request["prompt"])
            outcome = asyncio.run(
                run_from_input_impl(
                    build_initial_user_content(prompt),
                    prompt_text=prompt,
                    display_prompt=prompt,
                    repo_root=native_root,
                    image_path=None,
                    provider=EXPECTED_PROVIDER,
                    model_id=EXPECTED_MODEL,
                    openai_transport="http",
                    thinking_level=EXPECTED_REASONING_EFFORT,
                    max_turns=int(settings["max_turns"]),
                    system_prompt_path="designer_system_prompt.txt",
                    display_enabled=False,
                    sdk_package="sdk",
                    openai_reasoning_summary="auto",
                    label=f"Nano3D Table1 {request['task_id']} {request['repeat_id']}",
                    tags=["nano3d-table1", str(request["task_id"]), str(request["repeat_id"])],
                    collection="workbench",
                    record_id=record_id,
                    run_id=run_id,
                    record_author="nano3d-table1",
                )
            )
        elif kind == "repair":
            parent_record_id = str(request["prior_record_id"])
            outcome = asyncio.run(
                edit_record(
                    repo_root=native_root,
                    parent_record_id=parent_record_id,
                    edit_prompt=str(request["repair_prompt"]),
                    provider=EXPECTED_PROVIDER,
                    model_id=EXPECTED_MODEL,
                    thinking_level=EXPECTED_REASONING_EFFORT,
                    max_turns=int(settings["max_turns"]),
                    sdk_package="sdk",
                    record_id=record_id,
                    label=f"Nano3D Table1 repair {request['attempt_index']}",
                    tags=["nano3d-table1", "common-repair"],
                    display_enabled=False,
                    allow_variant_parent=True,
                    resolve_record_author_func=lambda _: "nano3d-table1",
                )
            )
        else:
            raise ValueError(f"unknown native worker kind: {kind}")
        retry_count = retry_counter.count

        repo = StorageRepo(native_root)
        record = repo.read_json(repo.layout.record_metadata_path(outcome.record_id))
        source = (
            active_model_path(repo, outcome.record_id, record=record)
            if isinstance(record, dict)
            else None
        )
        urdf = repo.layout.record_materialization_urdf_path(outcome.record_id)
        assets = repo.layout.record_materialization_assets_dir(outcome.record_id)
        record_assets = repo.layout.record_assets_dir(outcome.record_id)
        cost = (
            active_cost_path(repo, outcome.record_id, record=record)
            if isinstance(record, dict)
            else None
        )
        if cost is not None and cost.is_file():
            _read_logged_cost_totals(cost)
        response = NativeResult(
            status=str(outcome.status),
            exit_code=int(outcome.exit_code),
            message=outcome.message,
            run_id=str(outcome.run_id or run_id),
            record_id=str(outcome.record_id or record_id),
            source_path=str(source) if source is not None and source.is_file() else None,
            urdf_path=str(urdf) if urdf.is_file() else None,
            materialization_assets_path=str(assets) if assets.is_dir() else None,
            record_assets_path=str(record_assets) if record_assets.is_dir() else None,
            cost_path=str(cost) if cost is not None and cost.is_file() else None,
            request_started_at_utc=started,
            response_completed_at_utc=utc_now(),
            wall_time_seconds=time.monotonic() - start_wall,
            native_retry_count=retry_count,
        )
    except BaseException as exc:
        response = NativeResult(
            status="failed",
            exit_code=2,
            message=f"{type(exc).__name__}: {exc}",
            run_id=str(request.get("run_id") or ""),
            record_id=str(request.get("record_id") or ""),
            source_path=None,
            urdf_path=None,
            materialization_assets_path=None,
            record_assets_path=None,
            cost_path=None,
            request_started_at_utc=started,
            response_completed_at_utc=utc_now(),
            wall_time_seconds=time.monotonic() - start_wall,
            native_retry_count=retry_count,
        )
        write_text_atomic(response_path.with_suffix(".traceback.txt"), traceback.format_exc())
    write_json(response_path, asdict(response))
    return 0 if response.exit_code == 0 else 2


class ProductionNativeExecutor:
    def run(
        self, *, kind: str, native_root: Path, task: dict[str, Any], repeat_id: str,
        attempt_index: int, prior_native_root: Path | None, prior_record_id: str | None,
        repair_prompt: str | None, settings: dict[str, Any], timeout: float,
    ) -> NativeResult:
        attempt_root = native_root.parent
        if native_root.exists():
            raise RuntimeError(f"native output root already exists: {native_root}")
        if kind == "repair":
            if prior_native_root is None or prior_record_id is None:
                raise RuntimeError("repair requires a prior native repo and record")
            contained_runtime(prior_native_root, must_exist=True)
            shutil.copytree(prior_native_root, native_root, symlinks=False)
        token = uuid.uuid4().hex + uuid.uuid4().hex
        request_path = attempt_root / "native_worker_request.json"
        response_path = attempt_root / "native_worker_response.json"
        record_id = f"rec_table1_{task['task_id']}_{repeat_id}_a{attempt_index}"
        run_id = f"run_table1_{task['task_id']}_{repeat_id}_a{attempt_index}"
        write_json(
            request_path,
            {
                "schema_version": "table1_articraft_native_worker_request_v1",
                "worker_token_sha256": sha256_bytes(token.encode("utf-8")),
                "kind": kind,
                "native_root": str(native_root),
                "response_path": str(response_path),
                "task_id": task["task_id"],
                "repeat_id": repeat_id,
                "attempt_index": attempt_index,
                "prompt": task["prompt"],
                "repair_prompt": repair_prompt,
                "prior_record_id": prior_record_id,
                "record_id": record_id,
                "run_id": run_id,
                "settings": settings,
            },
        )
        python = ARTICRAFT_PYTHON if ARTICRAFT_PYTHON.is_file() else Path(sys.executable)
        env = {**os.environ, **THREAD_ENV, NATIVE_WORKER_ENV: token}
        env.pop("ARTICRAFT_MAX_COST_USD", None)
        env.pop("OPENAI_BASE_URL", None)
        env["OPENAI_PROMPT_CACHE_KEY_STRATEGY"] = "off"
        env["OPENAI_PROMPT_CACHE_RETENTION"] = "off"
        env["OPENAI_PROMPT_CACHE_KEY_PREFIX"] = ""
        env["PYTHONPATH"] = str(ARTICRAFT_ROOT)
        env["OPENAI_MAX_ATTEMPTS"] = str(int(settings["native_retry_limit"]) + 1)
        env["OPENAI_REQUEST_TIMEOUT_SECONDS"] = str(max(1, int(timeout)))
        started = utc_now()
        start_wall = time.monotonic()
        try:
            completed = subprocess.run(
                [str(python), str(Path(__file__).resolve()), "--native-worker", str(request_path)],
                cwd=ARTICRAFT_ROOT,
                env=env,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout,
                check=False,
            )
            write_text_atomic(attempt_root / "native_worker.stdout.txt", completed.stdout)
            write_text_atomic(attempt_root / "native_worker.stderr.txt", completed.stderr)
            if not response_path.is_file():
                raise RuntimeError(
                    f"native worker exit {completed.returncode} produced no response"
                )
            return NativeResult(**read_object(response_path))
        except subprocess.TimeoutExpired as exc:
            write_text_atomic(
                attempt_root / "native_worker.stdout.txt",
                exc.stdout if isinstance(exc.stdout, str) else "",
            )
            write_text_atomic(
                attempt_root / "native_worker.stderr.txt",
                exc.stderr if isinstance(exc.stderr, str) else "",
            )
            return NativeResult(
                status="failed",
                exit_code=124,
                message=f"native model attempt exceeded {timeout:g} seconds",
                run_id=run_id,
                record_id=record_id,
                source_path=None,
                urdf_path=None,
                materialization_assets_path=None,
                record_assets_path=None,
                cost_path=None,
                request_started_at_utc=started,
                response_completed_at_utc=utc_now(),
                wall_time_seconds=time.monotonic() - start_wall,
                native_retry_count=0,
                timed_out=True,
            )


def copy_tree_contents(source: Path, destination: Path) -> None:
    if not source.is_dir():
        return
    for path in sorted(source.rglob("*")):
        relative = path.relative_to(source)
        target = destination / relative
        if path.is_symlink():
            raise ValueError(f"native output symlink forbidden: {path}")
        if path.is_dir():
            target.mkdir(parents=True, exist_ok=True)
        elif path.is_file():
            target.parent.mkdir(parents=True, exist_ok=True)
            if target.exists() and sha256_file(target) != sha256_file(path):
                raise RuntimeError(
                    f"conflicting native asset normalization target: {target}"
                )
            shutil.copy2(path, target)


def normalize_native_output(native: NativeResult, native_root: Path, attempt_root: Path) -> dict[str, Path | None]:
    normalized = attempt_root / "normalized"
    source_out = normalized / "model.py"
    urdf_out = normalized / "model.urdf"
    assets_out = normalized / "assets"
    source_in = Path(native.source_path) if native.source_path else None
    urdf_in = Path(native.urdf_path) if native.urdf_path else None
    if source_in is not None:
        contained_child(source_in, native_root, must_exist=True)
        source_out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_in, source_out)
    if urdf_in is not None:
        contained_child(urdf_in, native_root, must_exist=True)
        urdf_out.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(urdf_in, urdf_out)
    for raw in (native.materialization_assets_path, native.record_assets_path):
        if raw:
            source = contained_child(Path(raw), native_root, must_exist=True)
            copy_tree_contents(source, assets_out)
    return {
        "source": source_out if source_out.is_file() else None,
        "urdf": urdf_out if urdf_out.is_file() else None,
        "assets": assets_out if assets_out.is_dir() else None,
    }


def read_cost(path: str | None, native_root: Path) -> tuple[int | None, int | None, float | None]:
    if not path:
        return None, None, None
    cost_path = contained_child(Path(path), native_root, must_exist=True)
    try:
        payload = read_object(cost_path)
    except (OSError, ValueError, json.JSONDecodeError):
        return None, None, None
    total = payload.get("all_in_total") or payload.get("total")
    total = total if isinstance(total, dict) else {}
    tokens = total.get("tokens") if isinstance(total.get("tokens"), dict) else {}
    costs = total.get("costs_usd") if isinstance(total.get("costs_usd"), dict) else {}
    inp = tokens.get("prompt_tokens")
    out = tokens.get("candidates_tokens")
    cost = costs.get("total")
    return (
        inp if isinstance(inp, int) and inp >= 0 else None,
        out if isinstance(out, int) and out >= 0 else None,
        float(cost) if isinstance(cost, (int, float)) and cost >= 0 else None,
    )


def run_compile_probe(source: Path | None, attempt_root: Path, timeout: float) -> dict[str, Any]:
    stdout_path = attempt_root / "execution_probe.stdout.txt"
    stderr_path = attempt_root / "execution_probe.stderr.txt"
    started = utc_now()
    start_wall = time.monotonic()
    timed_out = False
    exit_code = 1
    stdout = ""
    stderr = "source output unavailable"
    if source is not None:
        python = ARTICRAFT_PYTHON if ARTICRAFT_PYTHON.is_file() else Path(sys.executable)
        code = (
            "from pathlib import Path; "
            "from agent.compiler import compile_urdf_report_maybe_timeout; "
            "r=compile_urdf_report_maybe_timeout(Path(__import__('sys').argv[1]), sdk_package='sdk'); "
            "print(len(r.urdf_xml))"
        )
        try:
            completed = subprocess.run(
                [str(python), "-c", code, str(source)],
                cwd=ARTICRAFT_ROOT,
                env={**os.environ, **THREAD_ENV, "PYTHONPATH": str(ARTICRAFT_ROOT)},
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout,
                check=False,
            )
            exit_code = completed.returncode
            stdout = completed.stdout
            stderr = completed.stderr
        except subprocess.TimeoutExpired as exc:
            timed_out = True
            exit_code = 124
            stdout = exc.stdout if isinstance(exc.stdout, str) else ""
            stderr = exc.stderr if isinstance(exc.stderr, str) else ""
    write_text_atomic(stdout_path, stdout)
    write_text_atomic(stderr_path, stderr)
    return {
        "started_at_utc": started,
        "finished_at_utc": utc_now(),
        "wall_time_s": time.monotonic() - start_wall,
        "exit_code": exit_code,
        "timed_out": timed_out,
        "source_sha256": sha256_file(source) if source is not None else "0" * 64,
        "stdout_sha256": sha256_file(stdout_path),
        "stderr_sha256": sha256_file(stderr_path),
    }


def run_common_evaluator(package: Path, report: Path, timeout: float) -> dict[str, Any]:
    completed = subprocess.run(
        [
            sys.executable,
            str(COMMON_EVALUATOR),
            "--package-manifest",
            str(package),
            "--output",
            str(report),
        ],
        cwd=REPO_ROOT,
        env={**os.environ, **THREAD_ENV},
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )
    write_text_atomic(report.with_suffix(".stdout.txt"), completed.stdout)
    write_text_atomic(report.with_suffix(".stderr.txt"), completed.stderr)
    if not report.is_file():
        raise RuntimeError(
            f"common evaluator exit {completed.returncode} produced no report: "
            f"{completed.stderr[-2000:]}"
        )
    return read_object(report)


ProbeFunc = Callable[[Path | None, Path, float], dict[str, Any]]
EvaluatorFunc = Callable[[Path, Path, float], dict[str, Any]]


def evaluation_record(report: dict[str, Any], report_path: Path, job_root: Path) -> dict[str, Any]:
    verdicts = report.get("verdicts")
    verdicts = verdicts if isinstance(verdicts, dict) else {}
    binding_pass = all(report.get("binding_checks", {}).values())
    binding_pass = binding_pass and all(report.get("protocol_checks", {}).values())
    binding_pass = binding_pass and all(report.get("task_checks", {}).values())
    return {
        "state": "observed",
        "executable": bool(verdicts.get("executable")),
        "artifact_saved": bool(verdicts.get("artifact_saved")),
        "common_qc_pass": bool(verdicts.get("common_qc_pass")),
        "urdf_tree_pass": bool(verdicts.get("urdf_tree_pass")),
        "semantic_roles_pass": bool(verdicts.get("semantic_roles_pass")),
        "joint_spec_pass": bool(verdicts.get("joint_spec_pass")),
        "input_bindings_pass": binding_pass,
        "common_evaluator_report_path": rel(report_path, job_root),
        "common_evaluator_report_sha256": sha256_file(report_path),
        "reason": None,
    }


def bounded_feedback(report: dict[str, Any]) -> dict[str, Any]:
    feedback = report.get("feedback")
    feedback = dict(feedback) if isinstance(feedback, dict) else {}
    allowed = {"schema_version", "task_id", "attempt_index", "failure_codes", "bounded_diagnostics", "policy"}
    filtered = {key: feedback[key] for key in allowed if key in feedback}
    filtered["common_qc_pass"] = bool(report.get("verdicts", {}).get("common_qc_pass"))
    return filtered


def repair_request(prior_feedback: list[dict[str, Any]], prior_source_sha: str) -> dict[str, Any]:
    return {
        "schema_version": "table1_articraft_common_repair_input_v1",
        "prior_source_sha256": prior_source_sha,
        "bounded_cumulative_feedback": prior_feedback,
        "instruction": (
            "Edit the staged parent model.py. Preserve correct content and make the smallest "
            "coherent fixes for every listed common-evaluator failure. The feedback is cumulative."
        ),
    }


def native_failure_class(native: NativeResult) -> str | None:
    if native.timed_out:
        return "model_timeout"
    if native.exit_code != 0 or native.status != "success":
        return "native_authoring_failure"
    return None


def attempt_output_record(paths: dict[str, Path | None], job_root: Path) -> dict[str, Any]:
    source = paths["source"]
    urdf = paths["urdf"]
    return {
        "template_path": rel(source, job_root) if source is not None else None,
        "template_sha256": sha256_file(source) if source is not None else None,
        "artifact_path": rel(urdf, job_root) if urdf is not None else None,
        "artifact_sha256": sha256_file(urdf) if urdf is not None else None,
    }


def execute_job(
    *, task: dict[str, Any], repeat_id: str, job_root: Path, protocol: dict[str, Any],
    bindings: dict[str, str], executor: NativeExecutor | None = None,
    evaluator_func: EvaluatorFunc = run_common_evaluator,
    probe_func: ProbeFunc = run_compile_probe,
    pause_after_new_attempts: int | None = None,
    allow_test_unready: bool = False,
) -> dict[str, Any]:
    job_root = contained_runtime(job_root)
    if not allow_test_unready and not protocol_method_ready(protocol):
        raise RuntimeError("protocol Articraft execution readiness is not enabled")
    if not allow_test_unready and not author_access_audit()["strong_read_isolation"]:
        raise RuntimeError("Articraft author read isolation is not enforced")
    result_path = job_root / "result.json"
    result_seal = job_root / "result_seal.json"
    if result_path.exists() or result_seal.exists():
        return validate_completed_result(
            job_root,
            bindings,
            expected_run_id=f"articraft__{task['task_id']}__{repeat_id}",
            expected_method_id=METHOD_ID,
            expected_task_id=task["task_id"],
            expected_repeat_id=repeat_id,
        )
    state_path = job_root / "run_state.json"
    attempts_root = job_root / "attempts"
    if job_root.exists() and not job_root.is_dir():
        raise RuntimeError(f"job output collision: {job_root}")
    job_root.mkdir(parents=True, exist_ok=True)
    settings = native_settings(selected_method(protocol))
    timeouts = protocol.get("timeouts")
    timeouts = timeouts if isinstance(timeouts, dict) else {}
    execution_settings = {
        "provider": EXPECTED_PROVIDER,
        "model": EXPECTED_MODEL,
        "reasoning_effort": EXPECTED_REASONING_EFFORT,
        "max_turns": int(settings["max_turns"]),
        "openai_transport": str(settings.get("openai_transport", "http")),
        "sdk_package": str(settings.get("sdk_package", "sdk")),
        "native_retry_limit": int(timeouts["native_retry_limit_per_attempt"]),
    }
    if execution_settings != {
        "provider": "openai",
        "model": "gpt-5",
        "reasoning_effort": "high",
        "max_turns": 100,
        "openai_transport": "http",
        "sdk_package": "sdk",
        "native_retry_limit": 2,
    }:
        raise RuntimeError(f"non-frozen native settings: {execution_settings}")
    model_timeout = float(timeouts["model_response_seconds"])
    execution_timeout = float(timeouts["execution_seconds_per_attempt"])
    evaluator_timeout = float(timeouts["common_evaluator_seconds_per_attempt"])
    executor = executor or ProductionNativeExecutor()
    signature = {
        "schema_version": "table1_articraft_run_state_v1",
        "protocol_id": protocol["protocol_id"],
        "run_id": f"articraft__{task['task_id']}__{repeat_id}",
        "task_id": task["task_id"],
        "repeat_id": repeat_id,
        "prompt_sha256": task["prompt_sha256"],
        "bindings": bindings,
        "execution_settings": execution_settings,
    }
    if state_path.exists():
        state = read_object(state_path)
        if {key: state.get(key) for key in signature} != signature:
            raise RuntimeError(f"refusing stale partial run state: {state_path}")
        attempts = state.get("attempts")
        if not isinstance(attempts, list):
            raise RuntimeError(f"invalid partial run attempts: {state_path}")
        for row in attempts:
            seal = validate_attempt_seal(
                attempts_root / f"a{int(row['attempt_index'])}", bindings
            )
            state_record = {
                key: value
                for key, value in row.items()
                if key != "attempt_output_tree_sha256"
            }
            if (
                row.get("attempt_output_tree_sha256") != seal["output_tree_sha256"]
                or canonical_sha256(state_record) != seal.get("attempt_record_sha256")
            ):
                raise RuntimeError("partial run state does not match immutable attempt seal")
    else:
        if any(job_root.iterdir()):
            raise RuntimeError(f"preexisting job output collision without state: {job_root}")
        state = {**signature, "started_at_utc": utc_now(), "attempts": []}
        attempts = state["attempts"]
        write_json(state_path, state)

    if len(attempts) > EXPECTED_REPAIR_TURNS + 1:
        raise RuntimeError("partial state exceeds repair budget")
    prior_feedback: list[dict[str, Any]] = []
    for index, row in enumerate(attempts):
        if row.get("attempt_index") != index:
            raise RuntimeError("partial state attempts are non-contiguous")
        feedback_path = attempts_root / f"a{index}" / "bounded_feedback.json"
        if feedback_path.is_file():
            prior_feedback.append(read_object(feedback_path))

    new_attempts = 0
    for attempt_index in range(len(attempts), EXPECTED_REPAIR_TURNS + 1):
        if attempts and attempts[-1]["evaluation"].get("common_qc_pass") is True:
            break
        attempt_root = attempts_root / f"a{attempt_index}"
        if attempt_root.exists():
            raise RuntimeError(f"unsealed or colliding attempt output: {attempt_root}")
        attempt_root.mkdir(parents=True)
        native_root = attempt_root / "native_repo"
        kind = "initial" if attempt_index == 0 else "repair"
        prior_native_root = attempts_root / f"a{attempt_index - 1}" / "native_repo" if attempt_index else None
        prior_record_id = attempts[-1].get("native_record_id") if attempts else None
        repair_payload = None
        repair_sha = None
        if attempt_index > 0:
            prior_source = attempts_root / f"a{attempt_index - 1}" / "normalized/model.py"
            if not prior_source.is_file() or not prior_record_id:
                break
            repair_payload = repair_request(prior_feedback, sha256_file(prior_source))
            write_json(attempt_root / "repair_input.json", repair_payload)
            repair_sha = sha256_file(attempt_root / "repair_input.json")
        try:
            native = executor.run(
                kind=kind,
                native_root=native_root,
                task=task,
                repeat_id=repeat_id,
                attempt_index=attempt_index,
                prior_native_root=prior_native_root,
                prior_record_id=str(prior_record_id) if prior_record_id else None,
                repair_prompt=(
                    json.dumps(repair_payload, ensure_ascii=False, sort_keys=True)
                    if repair_payload is not None
                    else None
                ),
                settings=execution_settings,
                timeout=model_timeout,
            )
        except TimeoutError:
            native = NativeResult(
                status="failed", exit_code=124, message="native executor timeout",
                run_id=f"run_table1_{task['task_id']}_{repeat_id}_a{attempt_index}",
                record_id=f"rec_table1_{task['task_id']}_{repeat_id}_a{attempt_index}",
                source_path=None, urdf_path=None, materialization_assets_path=None,
                record_assets_path=None, cost_path=None, request_started_at_utc=utc_now(),
                response_completed_at_utc=utc_now(), wall_time_seconds=model_timeout,
                native_retry_count=0, timed_out=True,
            )
        if native.native_retry_count > execution_settings["native_retry_limit"]:
            raise RuntimeError("native retry count exceeds frozen limit")
        if not native_root.exists():
            native_root.mkdir(parents=True)
        paths = normalize_native_output(native, native_root, attempt_root)
        probe = probe_func(paths["source"], attempt_root, execution_timeout)
        package_path = attempt_root / "package.json"
        source = paths["source"]
        urdf = paths["urdf"]
        package = {
            "schema_version": "table1_authoring_package_v1",
            "run_id": signature["run_id"],
            "method_id": METHOD_ID,
            "task_id": task["task_id"],
            "repeat_id": repeat_id,
            "attempt_index": attempt_index,
            "run_root": str(job_root),
            "bindings": {
                key: bindings[key]
                for key in (
                    "protocol_sha256", "manifest_sha256", "hidden_specs_sha256",
                    "common_evaluator_sha256", "package_schema_sha256",
                )
            },
            "artifacts": {
                "source": {
                    "path": rel(source, attempt_root) if source is not None else "normalized/model.py",
                    "sha256": sha256_file(source) if source is not None else "0" * 64,
                },
                "urdf": {
                    "path": rel(urdf, attempt_root) if urdf is not None else "normalized/model.urdf",
                    "sha256": sha256_file(urdf) if urdf is not None else "0" * 64,
                },
            },
            "execution_probe": probe,
        }
        validate_schema(package, PACKAGE_SCHEMA)
        write_json(package_path, package)
        report_path = attempt_root / "common_evaluator_report.json"
        evaluator_error = None
        try:
            report = evaluator_func(package_path, report_path, evaluator_timeout)
            if not report_path.is_file():
                write_json(report_path, report)
            evaluation = evaluation_record(report, report_path, job_root)
            feedback = bounded_feedback(report)
        except (subprocess.TimeoutExpired, TimeoutError) as exc:
            evaluator_error = f"common evaluator timeout: {exc}"
            report = {}
            evaluation = {
                "state": "not_evaluable", "executable": None, "artifact_saved": None,
                "common_qc_pass": None, "urdf_tree_pass": None,
                "semantic_roles_pass": None, "joint_spec_pass": None,
                "input_bindings_pass": None, "common_evaluator_report_path": None,
                "common_evaluator_report_sha256": None, "reason": evaluator_error,
            }
            feedback = {
                "schema_version": 1, "task_id": task["task_id"],
                "attempt_index": attempt_index, "common_qc_pass": False,
                "failure_codes": ["COMMON_EVALUATOR_TIMEOUT"],
                "bounded_diagnostics": {},
                "policy": "No hidden specification content is exposed.",
            }
        except Exception as exc:  # noqa: BLE001
            evaluator_error = f"common evaluator error: {type(exc).__name__}: {exc}"
            report = {}
            evaluation = {
                "state": "not_evaluable", "executable": None, "artifact_saved": None,
                "common_qc_pass": None, "urdf_tree_pass": None,
                "semantic_roles_pass": None, "joint_spec_pass": None,
                "input_bindings_pass": None, "common_evaluator_report_path": None,
                "common_evaluator_report_sha256": None, "reason": evaluator_error,
            }
            feedback = {
                "schema_version": 1, "task_id": task["task_id"],
                "attempt_index": attempt_index, "common_qc_pass": False,
                "failure_codes": ["COMMON_EVALUATOR_ERROR"],
                "bounded_diagnostics": {},
                "policy": "No hidden specification content is exposed.",
            }
        feedback_path = attempt_root / "bounded_feedback.json"
        write_json(feedback_path, feedback)
        prior_feedback.append(feedback)
        input_tokens, output_tokens, api_cost = read_cost(native.cost_path, native_root)
        failure_class = native_failure_class(native)
        if failure_class is None and evaluator_error:
            failure_class = "common_evaluator_failure"
        elif failure_class is None and evaluation["common_qc_pass"] is not True:
            failure_class = "common_evaluator_rejection"
        attempt_row = {
            "attempt_index": attempt_index,
            "attempt_kind": "attempt_0" if attempt_index == 0 else "common_repair",
            "method_id": METHOD_ID,
            "task_id": task["task_id"],
            "repeat_id": repeat_id,
            "native_retry_count": native.native_retry_count,
            "native_retry_index": native.native_retry_count,
            "request_started_at_utc": native.request_started_at_utc,
            "response_completed_at_utc": native.response_completed_at_utc,
            "execution_started_at_utc": probe["started_at_utc"],
            "execution_completed_at_utc": probe["finished_at_utc"],
            "failure_class": failure_class,
            "model_response_sha256": None,
            "repair_feedback_sha256": repair_sha,
            "output": attempt_output_record(paths, job_root),
            "evaluation": evaluation,
            "telemetry": {
                "wall_time_seconds": native.wall_time_seconds,
                "input_tokens": input_tokens,
                "output_tokens": output_tokens,
                "provider_request_id_hash": None,
                "api_cost_usd": api_cost,
                "missing_reasons": {
                    "input_tokens": None if input_tokens is not None else "Articraft cost log did not report prompt tokens",
                    "output_tokens": None if output_tokens is not None else "Articraft cost log did not report candidate tokens",
                    "provider_request_id_hash": "Pinned Articraft does not persist provider request identifiers",
                    "api_cost_usd": None if api_cost is not None else "Articraft cost log did not report total API cost",
                },
            },
        }
        if evaluation["common_qc_pass"] is not True:
            write_json(
                attempt_root / "failure.json",
                {
                    "schema_version": "table1_articraft_attempt_failure_v1",
                    "failure_class": failure_class,
                    "native_exit_code": native.exit_code,
                    "native_status": native.status,
                    "native_message": native.message,
                    "evaluator_error": evaluator_error,
                    "bounded_feedback_sha256": sha256_file(feedback_path),
                },
            )
        state_attempt = {
            **attempt_row,
            "native_record_id": native.record_id,
        }
        seal = seal_attempt(attempt_root, bindings, canonical_sha256(state_attempt))
        state_attempt["attempt_output_tree_sha256"] = seal["output_tree_sha256"]
        attempts.append(state_attempt)
        new_attempts += 1
        state["attempts"] = attempts
        state["updated_at_utc"] = utc_now()
        write_json(state_path, state)
        if evaluation["common_qc_pass"] is True or evaluator_error is not None:
            break
        if source is None or native.exit_code != 0:
            break
        if pause_after_new_attempts is not None and new_attempts >= pause_after_new_attempts:
            raise ExecutionPaused(f"paused after {new_attempts} newly sealed attempt(s)")

    if not attempts:
        raise RuntimeError("no attempt could be executed or resumed")
    schema_attempts = [
        {key: value for key, value in row.items() if key not in {"native_record_id", "attempt_output_tree_sha256"}}
        for row in attempts
    ]
    first = schema_attempts[0]["evaluation"]
    final = schema_attempts[-1]["evaluation"]
    observed = final["state"] == "observed"
    result = {
        "schema_version": "table1_authoring_result_v1",
        "protocol_id": protocol["protocol_id"],
        "bindings": bindings,
        "run_id": signature["run_id"],
        "method_id": METHOD_ID,
        "task_id": task["task_id"],
        "repeat_id": repeat_id,
        "status": "completed" if observed else "failed",
        "started_at_utc": state["started_at_utc"],
        "finished_at_utc": utc_now(),
        "attempts": schema_attempts,
        "summary": {
            "state": "observed" if observed else "not_evaluable",
            "executable": final["executable"] if observed else None,
            "artifact_saved": final["artifact_saved"] if observed else None,
            "first_shot": first["common_qc_pass"] if observed else None,
            "final_success": final["common_qc_pass"] if observed else None,
            "repair_turns": len(schema_attempts) - 1 if observed else None,
            "reason": None if observed else final["reason"],
        },
        "error": None if observed else {
            "code": "COMMON_EVALUATOR_NOT_EVALUABLE",
            "message": str(final["reason"] or "common evaluator produced no evaluable verdict"),
        },
    }
    validate_schema(result, RESULT_SCHEMA)
    write_json(result_path, result)
    seal_result(job_root, result_path, bindings)
    return result


def execution_readiness(
    checks: dict[str, bool], implementation: dict[str, Any], protocol: dict[str, Any],
    access: dict[str, Any], retry_audit: dict[str, Any],
) -> dict[str, Any]:
    blockers: list[dict[str, str]] = []
    failed_frozen = sorted(key for key, value in checks.items() if not value)
    if failed_frozen:
        blockers.append({"code": "FROZEN_INPUT_BINDING_FAILED", "detail": ", ".join(failed_frozen)})
    failed_impl = sorted(key for key, value in implementation["checks"].items() if not value)
    if failed_impl:
        blockers.append({"code": "IMPLEMENTATION_BINDING_FAILED", "detail": ", ".join(failed_impl)})
    if not access["strong_read_isolation"]:
        blockers.append({
            "code": "ARTICRAFT_AUTHOR_READ_ISOLATION_NOT_ENFORCED",
            "detail": access["reason"],
        })
    if not (
        retry_audit["per_common_attempt_limit_enforced"]
        and retry_audit["identical_request_retries_only"]
    ):
        blockers.append({
            "code": "ARTICRAFT_NATIVE_RETRY_CONTRACT_NOT_ENFORCED",
            "detail": retry_audit["reason"],
        })
    if not protocol_method_ready(protocol):
        blockers.append({
            "code": "PROTOCOL_METHOD_NOT_READY",
            "detail": "Frozen protocol has not enabled execution_readiness.method_adapters_ready.articraft.",
        })
    return {
        "frozen_inputs_ready": all(checks.values()),
        "implementation_contract_ready": all(implementation["checks"].values()),
        "planned_paths_unique": True,
        "output_dir_isolation_ready": True,
        "author_read_isolation_ready": access["strong_read_isolation"],
        "native_repair_ready": True,
        "normalization_ready": True,
        "adapter_gates": {
            "common_evaluator_adapter_implemented": True,
            "common_repair_adapter_implemented": True,
            "common_result_writer_implemented": True,
            "partial_resume_and_output_seals_implemented": True,
        },
        "execution_ready": not blockers,
        "blockers": blockers,
    }


def render_report(summary: dict[str, Any], ready: dict[str, Any]) -> str:
    lines = [
        "# Table 1 Articraft Authoring Adapter",
        "",
        f"Status: **{summary['status']}**",
        "",
        f"Prepared jobs: {summary['prepared_jobs']}/{EXPECTED_JOB_COUNT}; provider calls: {summary['provider_calls_made']}.",
        "",
        "## Execution Contract",
        "",
        f"- Native implementation: `{rel(ARTICRAFT_ROOT)}` at `{EXPECTED_ARTICRAFT_COMMIT}`.",
        f"- Provider/model/reasoning/max turns: `{EXPECTED_PROVIDER}` / `{EXPECTED_MODEL}` / `{EXPECTED_REASONING_EFFORT}` / `{EXPECTED_MAX_TURNS}`.",
        "- Every attempt owns an independent native StorageRepo root and immutable normalized output seal.",
        "- Common evaluator, cumulative bounded repair, result schema validation, failure retention, and hash-bound resume are implemented.",
        "",
        "## Execution Blockers",
        "",
    ]
    lines.extend(
        [f"- `{row['code']}`: {row['detail']}" for row in ready["blockers"]]
        or ["- None."]
    )
    lines.extend(
        [
            "",
            "## Claim Boundary",
            "",
            "Preparation and mock tests make zero provider calls. Numeric Table 1 claims require paid attempts plus hash-valid normalized results and common-evaluator reports.",
            "",
        ]
    )
    return "\n".join(lines)


def select_execution_rows(
    tasks: list[dict[str, Any]], repeat_ids: list[str], args: argparse.Namespace
) -> tuple[list[dict[str, Any]], list[str]]:
    task_filter = {item for item in args.task_ids.split(",") if item}
    repeat_filter = {item for item in args.repeat_ids.split(",") if item}
    selected_tasks = tasks if args.all_tasks else [row for row in tasks if row["task_id"] in task_filter]
    selected_repeats = repeat_ids if args.all_repeats else [item for item in repeat_ids if item in repeat_filter]
    unknown_tasks = task_filter - {row["task_id"] for row in tasks}
    unknown_repeats = repeat_filter - set(repeat_ids)
    if unknown_tasks or unknown_repeats:
        raise ValueError(f"unknown selectors: tasks={sorted(unknown_tasks)}, repeats={sorted(unknown_repeats)}")
    if not selected_tasks or not selected_repeats:
        raise ValueError("execute requires task/repeat selectors or --all-tasks/--all-repeats")
    return selected_tasks, selected_repeats


def build_authoring_summary(
    *,
    status: str,
    results: list[dict[str, Any]],
    prepared_jobs: int,
    execution_ready: bool,
    blocker_codes: list[str],
    generated_at_utc: str | None = None,
) -> dict[str, Any]:
    attempted = len(results)
    evaluable = sum(row["summary"]["state"] == "observed" for row in results)
    success = sum(row["summary"].get("final_success") is True for row in results)
    return {
        "schema_version": "table1_articraft_authoring_summary_v1",
        "generated_at_utc": generated_at_utc or utc_now(),
        "status": status,
        "method_id": METHOD_ID,
        "intent_denominator": EXPECTED_JOB_COUNT,
        "prepared_jobs": prepared_jobs,
        "attempted_jobs": attempted,
        "completed_jobs": len(results),
        "evaluable_jobs": evaluable,
        "strict_success_jobs": success,
        "provider_calls_made": None if attempted else 0,
        "api_or_network_accessed": True if attempted else False,
        "execution_ready": execution_ready,
        "blocker_codes": blocker_codes,
        "claim_boundary": (
            "Headline final_success is strict success over all 162 intent runs. "
            "attempted_jobs and evaluable_jobs are reported separately; neither replaces "
            "the intent denominator. Preparation alone is not experimental evidence."
        ),
        "metrics": {
            "final_success": {
                "value": success / EXPECTED_JOB_COUNT if attempted else None,
                "state": "observed" if attempted else "not_reported",
                "numerator": success if attempted else None,
                "denominator": EXPECTED_JOB_COUNT,
                "reason": None if attempted else "zero authoring attempts; prepare-only or fail-closed execution",
            }
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--run-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--task-ids", default="")
    parser.add_argument("--repeat-ids", default="")
    parser.add_argument("--all-tasks", action="store_true")
    parser.add_argument("--all-repeats", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--acknowledge-paid-run", action="store_true")
    parser.add_argument("--native-worker", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()

    for key, value in THREAD_ENV.items():
        os.environ[key] = value
    if args.native_worker is not None:
        return _native_worker(args.native_worker)
    if args.acknowledge_paid_run and not args.execute:
        parser.error("--acknowledge-paid-run is valid only with --execute")
    if args.execute and not args.acknowledge_paid_run:
        parser.error("--execute requires --acknowledge-paid-run")

    manifest_path = contained_repo(args.manifest, must_exist=True)
    protocol_path = contained_repo(args.protocol, must_exist=True)
    out = contained_runtime(args.out)
    run_root = contained_runtime(args.run_root)
    manifest, manifest_raw, manifest_sha = read_json_snapshot(manifest_path)
    protocol, protocol_raw, protocol_sha = read_json_snapshot(protocol_path)
    method = selected_method(protocol)
    checks = frozen_checks(manifest, manifest_sha, protocol, method)
    implementation = implementation_binding(method, Path(__file__).resolve())
    access = author_access_audit()
    request_audit = request_payload_audit()
    retry_audit = native_retry_contract_audit()
    if not request_audit["pass"]:
        checks["pinned_native_request_payload_matches_frozen_shape"] = False
    else:
        checks["pinned_native_request_payload_matches_frozen_shape"] = True
    ready = execution_readiness(
        checks, implementation, protocol, access, retry_audit
    )
    tasks = manifest.get("tasks") if isinstance(manifest.get("tasks"), list) else []
    repeat_ids = manifest.get("repeat_ids") if isinstance(manifest.get("repeat_ids"), list) else []
    bindings = build_bindings(manifest_sha, protocol_sha, protocol)
    provenance = {
        "implementation_checkout_path": rel(ARTICRAFT_ROOT),
        "implementation_checkout_head": implementation["actual"]["commit"],
        "implementation_checkout_tree": implementation["actual"]["git_tree"],
        "implementation_entrypoint": rel(ARTICRAFT_ENTRYPOINT),
        "provenance_class": "local_pipeline_mirror_of_mattzh72_articraft",
    }
    jobs = build_jobs(tasks, repeat_ids, run_root, bindings, provenance, EXPECTED_REPAIR_TURNS)
    plan_checks = validate_job_plan(jobs)
    collisions = output_collisions(jobs)
    stability = {
        "manifest_unchanged_during_prepare": manifest_path.read_bytes() == manifest_raw,
        "protocol_unchanged_during_prepare": protocol_path.read_bytes() == protocol_raw,
    }
    if not all(plan_checks.values()):
        ready["blockers"].append({"code": "INVALID_JOB_PLAN", "detail": ", ".join(k for k, v in plan_checks.items() if not v)})
    if not all(stability.values()):
        ready["blockers"].append({"code": "FROZEN_INPUT_CHANGED_DURING_PREPARE", "detail": ", ".join(k for k, v in stability.items() if not v)})
    ready["execution_ready"] = not ready["blockers"] and all(plan_checks.values()) and all(stability.values())
    generated_at = utc_now()
    status = "PREPARED_EXECUTION_READY" if ready["execution_ready"] else "PREPARED_EXECUTION_BLOCKED"
    selected_tasks: list[dict[str, Any]] = []
    selected_repeats: list[str] = []
    results: list[dict[str, Any]] = []
    refusal = None
    if args.execute:
        if not ready["execution_ready"]:
            status = "EXECUTION_REFUSED_FAIL_CLOSED"
            refusal = ready["blockers"]
        elif not (os.environ.get("OPENAI_API_KEY") or os.environ.get("OPENAI_API_KEYS")):
            status = "EXECUTION_REFUSED_FAIL_CLOSED"
            refusal = [{"code": "OPENAI_CREDENTIAL_MISSING", "detail": "No OpenAI credential is present in the execution environment."}]
        else:
            try:
                selected_tasks, selected_repeats = select_execution_rows(tasks, repeat_ids, args)
            except ValueError as exc:
                parser.error(str(exc))
            status = "EXECUTING"
            for task in selected_tasks:
                for repeat_id in selected_repeats:
                    results.append(
                        execute_job(
                            task=task,
                            repeat_id=repeat_id,
                            job_root=run_root / task["task_id"] / repeat_id,
                            protocol=protocol,
                            bindings=bindings,
                        )
                    )
            status = "EXECUTED"

    summary = build_authoring_summary(
        status=status,
        results=results,
        prepared_jobs=len(jobs),
        execution_ready=ready["execution_ready"],
        blocker_codes=[row["code"] for row in (refusal or ready["blockers"])],
        generated_at_utc=generated_at,
    )
    attempted = summary["attempted_jobs"]
    experiment_manifest = {
        "schema_version": "table1_articraft_authoring_experiment_manifest_v1",
        "experiment_id": "nano3d_table1_articraft_authoring_v1",
        "generated_at_utc": generated_at,
        "mode": "execute_requested" if args.execute else "prepare_only",
        "method_id": METHOD_ID,
        "provider": EXPECTED_PROVIDER,
        "model": EXPECTED_MODEL,
        "reasoning_effort": EXPECTED_REASONING_EFFORT,
        "native_max_turns": EXPECTED_MAX_TURNS,
        "bindings": bindings,
        "prepared_job_count": len(jobs),
        "job_ids": [row["job_id"] for row in jobs],
        "provider_calls_made": summary["provider_calls_made"],
        "api_or_network_accessed": summary["api_or_network_accessed"],
        "run_root_pattern": rel(run_root / "{task_id}" / "{repeat_id}"),
        "attempt_output_pattern": rel(run_root / "{task_id}" / "{repeat_id}" / "attempts/a{attempt_index}"),
        "native_storage_pattern": rel(run_root / "{task_id}" / "{repeat_id}" / "attempts/a{attempt_index}/native_repo/data"),
        "preexisting_output_policy": "resume_only_if_all_binding_and_output_seals_match",
        "failure_retention_policy": "immutable attempt directory and hash seal; never overwrite or delete",
        "method_provenance": provenance,
    }
    preflight = {
        "schema_version": "table1_articraft_authoring_preflight_v1",
        "generated_at_utc": generated_at,
        "status": status,
        "frozen_checks": checks,
        "frozen_checks_passed": sum(checks.values()),
        "frozen_checks_total": len(checks),
        "job_plan_checks": plan_checks,
        "output_preflight": {
            "checked_job_roots": len(jobs),
            "collision_count": len(collisions),
            "collision_paths": collisions,
            "policy": "existing roots are accepted only by execution-time hash-bound resume",
        },
        "input_stability": stability,
        "implementation_binding": implementation,
        "author_access_audit": access,
        "request_payload_audit": request_audit,
        "native_retry_contract_audit": retry_audit,
        "readiness": ready,
        "paid_execution_requested": args.execute,
        "paid_execution_acknowledged": args.acknowledge_paid_run,
        "provider_calls_made": summary["provider_calls_made"],
    }
    self_check_checks = {
        **checks,
        **implementation["checks"],
        **plan_checks,
        **stability,
        "provider_calls_zero_in_prepare": not args.execute and summary["provider_calls_made"] == 0,
        "native_storage_isolation_implemented": True,
        "author_virtual_read_tool_contained": access["virtual_read_tool_scope_restricted"],
        "author_read_isolation_blocks_formal_execution": not access["strong_read_isolation"],
        "native_retry_contract_gap_blocks_formal_execution": not (
            retry_audit["per_common_attempt_limit_enforced"]
            and retry_audit["identical_request_retries_only"]
        ),
        "common_evaluator_implemented": True,
        "common_repair_implemented": True,
        "normalized_result_writer_implemented": True,
        "resume_output_seals_implemented": True,
    }
    self_check = {
        "schema_version": "table1_articraft_authoring_self_check_v1",
        "generated_at_utc": generated_at,
        "scope": "adapter_contract_and_prepare_evidence",
        "pass": all(self_check_checks.values()),
        "execution_ready": ready["execution_ready"],
        "provider_calls_made": summary["provider_calls_made"],
        "api_or_network_accessed": summary["api_or_network_accessed"],
        "generated_assets": 0 if not args.execute else None,
        "hidden_specs_read_by_author": False,
        "credentials_persisted": False,
        "checks": self_check_checks,
    }
    jobs_document = {
        "schema_version": "table1_articraft_job_plan_v1",
        "generated_at_utc": generated_at,
        "method_id": METHOD_ID,
        "job_count": len(jobs),
        "attempts_per_job_max": EXPECTED_REPAIR_TURNS + 1,
        "jobs": jobs,
    }
    out.mkdir(parents=True, exist_ok=True)
    write_json(out / "experiment_manifest.json", experiment_manifest)
    write_json(out / "jobs.json", jobs_document)
    write_json(out / "preflight.json", preflight)
    write_json(out / "summary.json", summary)
    write_json(out / "self_check.json", self_check)
    write_text_atomic(out / "report.md", render_report(summary, ready))
    if refusal is not None:
        write_json(out / "execution_refusal.json", {
            "schema_version": "table1_articraft_execution_refusal_v1",
            "generated_at_utc": utc_now(),
            "status": "REFUSED_BEFORE_PROVIDER_CALL",
            "provider_calls_made": 0,
            "blockers": refusal,
        })
    print(json.dumps({
        "status": status,
        "prepared_jobs": len(jobs),
        "attempted_jobs": attempted,
        "provider_calls_made": summary["provider_calls_made"],
        "execution_ready": ready["execution_ready"],
        "blocker_codes": summary["blocker_codes"],
        "out": rel(out),
    }, sort_keys=True))
    return 2 if args.execute and refusal is not None else 0


if __name__ == "__main__":
    raise SystemExit(main())
