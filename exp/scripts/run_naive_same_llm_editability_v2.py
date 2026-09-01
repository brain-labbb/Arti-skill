#!/usr/bin/env python3
"""Run the frozen one-shot naive same-LLM Table 5 source-edit baseline.

Preflight and mock modes never contact a provider.  Real execution is guarded
by an exact manifest SHA and an explicit paid-API confirmation string.  Every
request has the full parent source, no Articraft system docs, no tools, no
compiler feedback, and no continuation or repair turn.
"""

from __future__ import annotations

import argparse
import ast
import asyncio
import fcntl
import hashlib
import json
import os
import re
import socket
import sys
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterator

SCRIPT_DIR = Path(__file__).resolve().parent
EXP_ROOT = SCRIPT_DIR.parent
REPO_ROOT = EXP_ROOT.parent
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
ARTICRAFT_ROOT = REPO_ROOT / "articraft_data"
DEFAULT_MANIFEST = EXP_ROOT / "reference" / "editability_v2" / "articraft_parent_manifest.json"
DEFAULT_CONTRACT = (
    EXP_ROOT / "reference" / "editability_v2" / "naive_same_llm_prompt_contract_v1.json"
)
DEFAULT_OUTPUT = EXP_ROOT / "runtime" / "nano3d_editability_v2" / "naive_same_llm"
PAID_CONFIRMATION = "RUN_FROZEN_ONE_SHOT_DASHSCOPE_18"
FROZEN_SETTINGS: dict[str, Any] = {
    "execution_mode": "single_provider_request_source_edit",
    "provider": "dashscope",
    "model_id": "qwen3.7-max-2026-05-20",
    "thinking_level": "high",
    "max_turns": 1,
    "provider_request_limit": 1,
    "context_window_tokens": 1_000_000,
    "max_output_tokens": 65_536,
    "output_safety_tokens": 1_024,
    "max_cost_usd": None,
    "tools": [],
    "automatic_retries": 0,
    "compile_feedback": False,
    "automatic_repair": False,
    "source_truncation": False,
}
FROZEN_PRICING: dict[str, Any] = {
    "currency": "CNY",
    "unit": "per_1m_tokens",
    "uncached_input": 12.0,
    "cached_input": 2.4,
    "output": 36.0,
    "retrieved_at": "2026-08-11",
    "usd_conversion": None,
}

sys.path.insert(0, str(ARTICRAFT_ROOT))


def contained(path: Path, *, strict: bool = False) -> Path:
    resolved = path.resolve(strict=strict)
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path outside authorized workspace: {resolved}")
    return resolved


def relative(path: Path) -> str:
    return str(contained(path).relative_to(WORKSPACE_ROOT))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path, strict=True).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def dump_json(path: Path, payload: Any) -> None:
    path = contained(path)
    contained(path.parent).mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def load_json(path: Path) -> Any:
    return json.loads(contained(path, strict=True).read_text(encoding="utf-8"))


@contextmanager
def output_root_lock(
    output_dir: Path,
    *,
    mode: str,
    expected_manifest_sha256: str | None,
) -> Iterator[dict[str, Any]]:
    """Hold a non-blocking OS lock for the complete output-root lifecycle."""
    output_dir = contained(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    lock_path = contained(output_dir / ".runner.lock")
    owner_path = contained(output_dir / ".runner.lock.owner.json")
    lock_handle = lock_path.open("a+", encoding="utf-8")
    try:
        try:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            try:
                owner = load_json(owner_path)
            except (OSError, json.JSONDecodeError):
                owner = {"status": "LOCKED_OWNER_METADATA_UNAVAILABLE"}
            raise RuntimeError(
                "OUTPUT_ROOT_LOCK_HELD:" + json.dumps(owner, sort_keys=True)
            ) from exc

        owner = {
            "schema_version": 1,
            "status": "ACTIVE",
            "pid": os.getpid(),
            "hostname": socket.gethostname(),
            "started_at_utc": datetime.now(timezone.utc).isoformat(),
            "mode": mode,
            "output_dir": relative(output_dir),
            "lock_file": relative(lock_path),
            "runner": relative(Path(__file__).resolve()),
            "runner_sha256": sha256(Path(__file__).resolve()),
            "expected_manifest_sha256": expected_manifest_sha256,
            "argv": sys.argv[1:],
        }
        dump_json(owner_path, owner)
        try:
            yield owner
        except BaseException:
            owner["status"] = "FAILED_OR_INTERRUPTED"
            owner["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            dump_json(owner_path, owner)
            raise
        else:
            owner["status"] = "RELEASED_COMPLETE"
            owner["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            dump_json(owner_path, owner)
    finally:
        try:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
        finally:
            lock_handle.close()


def resolve_manifest_path(value: Any) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("parent_model_path must be a non-empty string")
    path = Path(value)
    if not path.is_absolute():
        path = WORKSPACE_ROOT / path
    return contained(path, strict=True)


def render_user_message(task: dict[str, Any], source: str, source_hash: str) -> str:
    marker = f"PARENT_MODEL_PY_{source_hash.upper()}"
    if marker in source:
        raise ValueError(f"{task['task_id']}: source collides with deterministic delimiter")
    return (
        "Original object prompt:\n"
        f"{task['original_object_prompt'].strip()}\n\n"
        "Edit instruction:\n"
        f"{task['normalized_edit_instruction'].strip()}\n\n"
        f"Parent model.py SHA-256: {source_hash}\n"
        f"Parent model.py UTF-8 bytes: {len(source.encode('utf-8'))}\n\n"
        f"BEGIN_{marker}\n"
        f"{source}"
        f"{'' if source.endswith(chr(10)) else chr(10)}"
        f"END_{marker}\n"
    )


def validate_manifest(
    manifest: Any,
    *,
    manifest_path: Path,
    expected_manifest_sha256: str | None,
    contract_sha256: str,
) -> tuple[list[str], list[dict[str, Any]]]:
    errors: list[str] = []
    normalized_tasks: list[dict[str, Any]] = []
    actual_manifest_sha256 = sha256(manifest_path)
    if not expected_manifest_sha256:
        errors.append("EXPECTED_MANIFEST_SHA256_REQUIRED")
    elif expected_manifest_sha256.lower() != actual_manifest_sha256:
        errors.append("MANIFEST_SHA256_MISMATCH")
    if not isinstance(manifest, dict):
        return [*errors, "MANIFEST_NOT_AN_OBJECT"], normalized_tasks
    if manifest.get("schema_version") != 1:
        errors.append("MANIFEST_SCHEMA_VERSION_MISMATCH")
    if manifest.get("protocol_id") != "nano3d_table5_editability_shared_v2":
        errors.append("MANIFEST_PROTOCOL_ID_MISMATCH")
    if manifest.get("prompt_contract_sha256") != contract_sha256:
        errors.append("PROMPT_CONTRACT_SHA256_MISMATCH")

    method_protocols = manifest.get("method_protocols")
    settings = (
        method_protocols.get("naive_one_shot")
        if isinstance(method_protocols, dict)
        else None
    )
    if not isinstance(settings, dict):
        errors.append("NAIVE_METHOD_PROTOCOL_MISSING")
    else:
        for key, expected in FROZEN_SETTINGS.items():
            if settings.get(key) != expected:
                errors.append(f"SETTINGS_MISMATCH:{key}")
    pricing = manifest.get("pricing_snapshot")
    if not isinstance(pricing, dict):
        errors.append("PRICING_SNAPSHOT_MISSING")
    else:
        for key, expected in FROZEN_PRICING.items():
            if pricing.get(key) != expected:
                errors.append(f"PRICING_MISMATCH:{key}")

    tasks = manifest.get("tasks")
    if not isinstance(tasks, list):
        return [*errors, "TASKS_MISSING"], normalized_tasks
    task_ids = [str(task.get("task_id") or "").strip() for task in tasks if isinstance(task, dict)]
    if len(tasks) != 18:
        errors.append(f"TASK_COUNT:{len(tasks)}:EXPECTED:18")
    if len(task_ids) != 18 or any(not task_id for task_id in task_ids):
        errors.append("TASK_IDS_MUST_BE_18_NONEMPTY_VALUES")
    if len(set(task_ids)) != len(task_ids):
        errors.append("TASK_IDS_NOT_UNIQUE")
    required = {
        "task_id",
        "parent_record_id",
        "parent_model_path",
        "parent_model_sha256",
        "original_object_prompt",
        "normalized_edit_instruction",
        "edit_class",
        "gold",
    }
    class_counts: dict[str, int] = {}
    for index, task in enumerate(tasks):
        if not isinstance(task, dict):
            errors.append(f"TASK_NOT_OBJECT:{index}")
            continue
        task_id = str(task.get("task_id") or f"index_{index}")
        missing = sorted(required - set(task))
        if missing:
            errors.append(f"{task_id}:MISSING_FIELDS:{','.join(missing)}")
            continue
        if not str(task["original_object_prompt"]).strip():
            errors.append(f"{task_id}:EMPTY_ORIGINAL_OBJECT_PROMPT")
        if not str(task["normalized_edit_instruction"]).strip():
            errors.append(f"{task_id}:EMPTY_NORMALIZED_EDIT_INSTRUCTION")
        if not str(task["parent_record_id"]).strip():
            errors.append(f"{task_id}:EMPTY_PARENT_RECORD_ID")
        edit_class = str(task["edit_class"]).strip()
        if not edit_class:
            errors.append(f"{task_id}:EMPTY_EDIT_CLASS")
        else:
            class_counts[edit_class] = class_counts.get(edit_class, 0) + 1
        if not isinstance(task["gold"], dict) or not task["gold"]:
            errors.append(f"{task_id}:GOLD_MUST_BE_NONEMPTY_OBJECT")
        try:
            source_path = resolve_manifest_path(task["parent_model_path"])
        except (OSError, ValueError) as exc:
            errors.append(f"{task_id}:PARENT_MODEL_UNAVAILABLE:{exc}")
            continue
        if source_path.name != "model.py":
            errors.append(f"{task_id}:PARENT_BASENAME_NOT_MODEL_PY")
        actual_source_hash = sha256(source_path)
        if actual_source_hash != str(task["parent_model_sha256"]).lower():
            errors.append(f"{task_id}:PARENT_MODEL_SHA256_MISMATCH")
            continue
        source = source_path.read_text(encoding="utf-8")
        if not source.strip():
            errors.append(f"{task_id}:EMPTY_PARENT_MODEL")
            continue
        normalized_tasks.append(
            {
                **task,
                "parent_model_path": relative(source_path),
                "source": source,
                "source_sha256": actual_source_hash,
                "source_utf8_bytes": len(source.encode("utf-8")),
                "gold_sha256": hashlib.sha256(
                    json.dumps(
                        task["gold"], ensure_ascii=False, sort_keys=True, separators=(",", ":")
                    ).encode("utf-8")
                ).hexdigest(),
            }
        )
    declared_distribution = manifest.get("cohort_distribution")
    if not isinstance(declared_distribution, dict):
        errors.append("COHORT_DISTRIBUTION_MISSING")
    else:
        normalized_distribution = {
            str(key): value for key, value in declared_distribution.items()
        }
        if normalized_distribution != class_counts:
            errors.append("COHORT_DISTRIBUTION_MISMATCH")
    if sorted(class_counts.values()) != [6, 6, 6]:
        errors.append("EDIT_CLASS_DISTRIBUTION_MUST_BE_6_6_6")
    if len(normalized_tasks) != 18:
        errors.append(f"READY_TASK_COUNT:{len(normalized_tasks)}:EXPECTED:18")
    return sorted(set(errors)), normalized_tasks


def create_dashscope_client(*, dry_run: bool, output_dir: Path) -> Any:
    # Provider dependencies are optional for offline preflight and mock runs.
    # Import them only after all real-execution gates have passed.
    from agent.providers.dashscope import DashScopeLLM

    original_cwd = contained(Path.cwd(), strict=True)
    output_dir = contained(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    try:
        # DashScope's constructor checks cwd/.env. The experiment uses only
        # explicitly exported credentials, and preflight must not load secrets.
        os.chdir(output_dir)
        client = DashScopeLLM(
            model_id=FROZEN_SETTINGS["model_id"],
            thinking_level=FROZEN_SETTINGS["thinking_level"],
            dry_run=dry_run,
        )
    finally:
        os.chdir(original_cwd)
    client.max_tokens = int(FROZEN_SETTINGS["max_output_tokens"])
    client.context_tokens = int(FROZEN_SETTINGS["context_window_tokens"])
    client.output_safety_tokens = int(FROZEN_SETTINGS["output_safety_tokens"])
    client.max_attempts = 1

    original_convert = client._convert_response

    def convert_with_finish_reason(response: Any) -> dict[str, Any]:
        converted = original_convert(response)
        choices = response.get("choices") if isinstance(response, dict) else getattr(response, "choices", None)
        choice = choices[0] if isinstance(choices, list) and choices else None
        finish_reason = (
            choice.get("finish_reason") if isinstance(choice, dict) else getattr(choice, "finish_reason", None)
        )
        if finish_reason is not None:
            converted["finish_reason"] = str(finish_reason)
        return converted

    client._convert_response = convert_with_finish_reason
    return client


def estimated_prompt_tokens(preview: dict[str, Any]) -> int:
    messages = preview.get("messages")
    tools = preview.get("tools")
    text_size = len(json.dumps(messages, ensure_ascii=False, separators=(",", ":")))
    if tools is not None:
        text_size += len(json.dumps(tools, ensure_ascii=False, separators=(",", ":")))
    overhead = 128 + (len(messages) * 16 if isinstance(messages, list) else 0)
    if isinstance(tools, list):
        overhead += len(tools) * 128
    return max(1, (text_size + 2) // 3 + overhead)


def cny_cost(usage: dict[str, int]) -> dict[str, Any]:
    prompt_tokens = int(usage.get("prompt_tokens") or 0)
    cached_tokens = min(prompt_tokens, int(usage.get("cached_tokens") or 0))
    uncached_tokens = max(0, prompt_tokens - cached_tokens)
    output_tokens = int(usage.get("candidates_tokens") or 0)
    input_uncached_cost = uncached_tokens * float(FROZEN_PRICING["uncached_input"]) / 1_000_000
    input_cached_cost = cached_tokens * float(FROZEN_PRICING["cached_input"]) / 1_000_000
    output_cost = output_tokens * float(FROZEN_PRICING["output"]) / 1_000_000
    return {
        "currency": "CNY",
        "estimated": True,
        "input_uncached": round(input_uncached_cost, 8),
        "input_cached": round(input_cached_cost, 8),
        "output": round(output_cost, 8),
        "total": round(input_uncached_cost + input_cached_cost + output_cost, 8),
        "pricing_snapshot": FROZEN_PRICING,
        "usd": None,
    }


def build_previews(
    tasks: list[dict[str, Any]],
    *,
    system_prompt: str,
    output_dir: Path,
) -> tuple[list[dict[str, Any]], list[str]]:
    client = create_dashscope_client(dry_run=True, output_dir=output_dir)
    rows: list[dict[str, Any]] = []
    errors: list[str] = []
    previews_dir = contained(output_dir / "request_previews")
    previews_dir.mkdir(parents=True, exist_ok=True)
    for task in tasks:
        task_id = str(task["task_id"])
        user_message = render_user_message(task, task["source"], task["source_sha256"])
        messages = [{"role": "user", "content": user_message}]
        preview = client.build_request_preview(
            system_prompt=system_prompt,
            messages=messages,
            tools=[],
        )
        preview_messages = preview.get("messages")
        preview_user_content = None
        if isinstance(preview_messages, list):
            for message in preview_messages:
                if isinstance(message, dict) and message.get("role") == "user":
                    preview_user_content = message.get("content")
                    break
        full_source_present = (
            isinstance(preview_user_content, str)
            and preview_user_content == user_message
            and task["source"] in preview_user_content
        )
        output_cap_preserved = preview.get("max_tokens") == FROZEN_SETTINGS["max_output_tokens"]
        no_tools = "tools" not in preview or preview.get("tools") == []
        if not full_source_present:
            errors.append(f"{task_id}:FULL_SOURCE_NOT_IN_PROVIDER_PREVIEW")
        if not output_cap_preserved:
            errors.append(f"{task_id}:OUTPUT_CAP_REDUCED_BY_CONTEXT")
        if not no_tools:
            errors.append(f"{task_id}:TOOLS_PRESENT_IN_PROVIDER_PREVIEW")
        prompt_tokens = estimated_prompt_tokens(preview)
        projected_usage = {
            "prompt_tokens": prompt_tokens,
            "cached_tokens": 0,
            "candidates_tokens": int(FROZEN_SETTINGS["max_output_tokens"]),
            "total_tokens": prompt_tokens + int(FROZEN_SETTINGS["max_output_tokens"]),
        }
        preview_path = previews_dir / f"{task_id}.json"
        dump_json(preview_path, preview)
        rows.append(
            {
                "task_id": task_id,
                "parent_model_path": task["parent_model_path"],
                "parent_model_sha256": task["source_sha256"],
                "parent_record_id": task["parent_record_id"],
                "edit_class": task["edit_class"],
                "gold_sha256": task["gold_sha256"],
                "gold_in_request": False,
                "parent_model_utf8_bytes": task["source_utf8_bytes"],
                "request_preview": relative(preview_path),
                "request_preview_sha256": sha256(preview_path),
                "full_source_present": full_source_present,
                "output_cap_preserved": output_cap_preserved,
                "tools": [],
                "estimated_prompt_tokens_conservative": prompt_tokens,
                "projected_max_usage": projected_usage,
                "projected_max_cost_cny": cny_cost(projected_usage),
            }
        )
    return rows, errors


PYTHON_FENCE = re.compile(r"\A\s*```python[ \t]*\r?\n(.*?)\r?\n```\s*\Z", re.DOTALL)


def assess_response(response: dict[str, Any]) -> dict[str, Any]:
    content = response.get("content")
    content = content if isinstance(content, str) else ""
    tool_calls = response.get("tool_calls")
    tool_calls = tool_calls if isinstance(tool_calls, list) else []
    usage = response.get("usage")
    usage = usage if isinstance(usage, dict) else {}
    finish_reason = str(response.get("finish_reason") or "").lower()
    match = PYTHON_FENCE.fullmatch(content)
    code = match.group(1) if match else None
    syntax_valid = False
    syntax_error = None
    if code is not None:
        try:
            ast.parse(code)
            syntax_valid = True
        except SyntaxError as exc:
            syntax_error = f"{exc.msg} at line {exc.lineno}, column {exc.offset}"
    candidate_tokens = usage.get("candidates_tokens")
    token_cap_hit = isinstance(candidate_tokens, int) and candidate_tokens >= int(
        FROZEN_SETTINGS["max_output_tokens"]
    )
    provider_length_finish = finish_reason in {"length", "max_tokens", "max_output_tokens"}
    accepted = bool(
        match
        and syntax_valid
        and not tool_calls
        and not token_cap_hit
        and not provider_length_finish
    )
    if accepted:
        status = "OUTPUT_ACCEPTED"
    elif tool_calls:
        status = "PROTOCOL_VIOLATION_TOOL_CALL"
    elif token_cap_hit or provider_length_finish:
        status = "OUTPUT_TRUNCATED_OR_INVALID"
    elif not match:
        status = "OUTPUT_FORMAT_INVALID"
    else:
        status = "OUTPUT_SYNTAX_INVALID"
    return {
        "status": status,
        "accepted": accepted,
        "code": code,
        "exact_single_python_fence": bool(match),
        "syntax_valid": syntax_valid,
        "syntax_error": syntax_error,
        "tool_call_count": len(tool_calls),
        "finish_reason": finish_reason or None,
        "token_cap_hit": token_cap_hit,
        "usage": usage,
        "estimated_cost_cny": cny_cost(usage),
    }


def mock_response(mock_dir: Path, task_id: str) -> dict[str, Any]:
    path = contained(mock_dir / f"{task_id}.json", strict=True)
    payload = load_json(path)
    if not isinstance(payload, dict):
        raise ValueError(f"{task_id}: mock response must be a JSON object")
    return payload


async def run_requests(
    tasks: list[dict[str, Any]],
    *,
    system_prompt: str,
    output_dir: Path,
    mode: str,
    mock_dir: Path | None,
    resume: bool = False,
) -> list[dict[str, Any]]:
    client = None
    if mode == "execute":
        client = create_dashscope_client(dry_run=False, output_dir=output_dir)
    records: list[dict[str, Any]] = []
    try:
        for task in tasks:
            task_id = str(task["task_id"])
            task_dir = contained(output_dir / "tasks" / task_id)
            if task_dir.exists():
                record_path = contained(task_dir / "record.json")
                existing = list(task_dir.iterdir())
                if resume and record_path.is_file():
                    record = load_json(record_path)
                    if (
                        not isinstance(record, dict)
                        or record.get("task_id") != task_id
                        or record.get("mode") != mode
                        or int(record.get("provider_requests", -1)) != (0 if mode == "mock" else 1)
                    ):
                        raise RuntimeError(f"{task_id}: existing resume record failed validation")
                    records.append(record)
                    continue
                if not resume or existing:
                    raise RuntimeError(
                        f"{task_id}: output already exists; refusing a possible duplicate paid request"
                    )
            else:
                task_dir.mkdir(parents=True, exist_ok=False)
            user_message = render_user_message(task, task["source"], task["source_sha256"])
            messages = [{"role": "user", "content": user_message}]
            started = time.monotonic()
            try:
                if mode == "mock":
                    if mock_dir is None:
                        raise ValueError("--mock-responses is required in mock mode")
                    response = mock_response(mock_dir, task_id)
                else:
                    response = await client.generate_with_tools(system_prompt, messages, [])
            except Exception as exc:  # noqa: BLE001
                record = {
                    "task_id": task_id,
                    "mode": mode,
                    "status": "PROVIDER_ERROR",
                    "accepted": False,
                    "provider_requests": 0 if mode == "mock" else 1,
                    "turns": 1,
                    "tools_exposed": 0,
                    "compile_feedback": False,
                    "repair_turns": 0,
                    "elapsed_seconds": time.monotonic() - started,
                    "parent_model_path": task["parent_model_path"],
                    "parent_model_sha256": task["source_sha256"],
                    "parent_record_id": task["parent_record_id"],
                    "edit_class": task["edit_class"],
                    "gold_sha256": task["gold_sha256"],
                    "gold_in_request": False,
                    "error": f"{type(exc).__name__}: {str(exc)[:2000]}",
                    "usage": {},
                    "estimated_cost_cny": cny_cost({}),
                }
                dump_json(task_dir / "record.json", record)
                records.append(record)
                break
            elapsed = time.monotonic() - started
            raw_path = task_dir / "raw_response.json"
            dump_json(raw_path, response)
            assessment = assess_response(response)
            code = assessment.pop("code")
            model_path = None
            if isinstance(code, str):
                model_path = contained(task_dir / "model.py")
                model_path.write_text(code + ("" if code.endswith("\n") else "\n"), encoding="utf-8")
            record = {
                "task_id": task_id,
                "mode": mode,
                "provider_requests": 0 if mode == "mock" else 1,
                "turns": 1,
                "tools_exposed": 0,
                "compile_feedback": False,
                "repair_turns": 0,
                "elapsed_seconds": elapsed,
                "parent_model_path": task["parent_model_path"],
                "parent_model_sha256": task["source_sha256"],
                "parent_record_id": task["parent_record_id"],
                "edit_class": task["edit_class"],
                "gold_sha256": task["gold_sha256"],
                "gold_in_request": False,
                "raw_response": relative(raw_path),
                "raw_response_sha256": sha256(raw_path),
                "model_py": relative(model_path) if model_path else None,
                "model_py_sha256": sha256(model_path) if model_path else None,
                **assessment,
            }
            dump_json(task_dir / "record.json", record)
            records.append(record)
    finally:
        if client is not None:
            await client.close()
    return records


def totals(records: list[dict[str, Any]]) -> dict[str, Any]:
    token_keys = ("prompt_tokens", "cached_tokens", "candidates_tokens", "total_tokens")
    token_totals = {key: 0 for key in token_keys}
    total_cny = 0.0
    for record in records:
        usage = record.get("usage") if isinstance(record.get("usage"), dict) else {}
        for key in token_keys:
            value = usage.get(key)
            if isinstance(value, int):
                token_totals[key] += value
        cost = record.get("estimated_cost_cny")
        if isinstance(cost, dict) and isinstance(cost.get("total"), (int, float)):
            total_cny += float(cost["total"])
    return {
        "actual_tokens": token_totals,
        "estimated_cost_cny": round(total_cny, 8),
        "usd_cost": None,
    }


def write_report(output_dir: Path, summary: dict[str, Any]) -> None:
    requirements = summary.get("requirements", {})
    blockers = summary.get("blockers", [])
    report = f"""# Naive same-LLM Table 5 source-edit baseline

Status: **{summary['status']}** (`{summary['evidence_class']}`)

This baseline gives `dashscope/qwen3.7-max-2026-05-20` only the complete parent
`model.py`, the original object prompt, and one normalized edit instruction. It
exposes no Articraft system documentation, examples, harness, tools, compiler
feedback, continuation, or automatic repair.

| Frozen control | Value |
|---|---:|
| Thinking | high |
| Native Articraft turn cap (reference) | 30 |
| Naive effective/provider turn cap | 1 |
| Context window | 1,000,000 tokens |
| Max output | 65,536 tokens |
| Max cost USD | N/A (`null`) |
| Tools / compile feedback / repair | 0 / none / 0 |

Long sources are never truncated, summarized, or chunked. Preflight requires the
full source to appear in the exact provider preview and requires the adapter to
retain all 65,536 output tokens. Otherwise the task is blocked before an API
call. Length-capped output receives no continuation.

Pricing is frozen in CNY per 1M tokens: uncached input 12, cached input 2.4,
output 36 (retrieved 2026-08-11). Actual token usage and estimated CNY are
reported without inventing a USD conversion.

Manifest available: {requirements.get('manifest_available', False)}; manifest
SHA verified: {requirements.get('manifest_sha256_verified', False)}; ready tasks:
{requirements.get('ready_tasks', 0)}/18; preview/context gates:
{requirements.get('preview_gates_passed', 0)}/18.

Blockers: {', '.join(blockers) if blockers else 'none'}.
"""
    contained(output_dir / "report.md").write_text(report, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--expected-manifest-sha256")
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--mode", choices=("preflight", "mock"), default="preflight")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Make the one-shot paid provider requests after all frozen gates pass.",
    )
    parser.add_argument("--mock-responses", type=Path)
    parser.add_argument("--confirm-paid-api")
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Skip only tasks with a complete validated record.json; reuse only an empty next-task directory.",
    )
    return parser.parse_args()


def run_locked(args: argparse.Namespace, *, mode: str, output_dir: Path) -> int:
    contract_path = contained(args.contract, strict=True)
    contract = load_json(contract_path)
    contract_sha = sha256(contract_path)
    manifest_path = contained(args.manifest)
    blockers: list[str] = []
    tasks: list[dict[str, Any]] = []
    manifest = None
    manifest_sha = None
    manifest_sha_verified = False
    if not manifest_path.is_file() or manifest_path.is_symlink():
        blockers.append("FROZEN_MANIFEST_UNAVAILABLE")
    else:
        manifest = load_json(manifest_path)
        manifest_sha = sha256(manifest_path)
        validation_errors, tasks = validate_manifest(
            manifest,
            manifest_path=manifest_path,
            expected_manifest_sha256=args.expected_manifest_sha256,
            contract_sha256=contract_sha,
        )
        blockers.extend(validation_errors)
        manifest_sha_verified = bool(
            args.expected_manifest_sha256
            and args.expected_manifest_sha256.lower() == manifest_sha
        )

    preview_rows: list[dict[str, Any]] = []
    if not blockers:
        preview_rows, preview_errors = build_previews(
            tasks,
            system_prompt=str(contract["system_prompt"]),
            output_dir=output_dir,
        )
        blockers.extend(preview_errors)

    if args.execute and args.mode == "mock":
        blockers.append("EXECUTE_AND_MOCK_ARE_MUTUALLY_EXCLUSIVE")
    if mode == "execute" and args.confirm_paid_api != PAID_CONFIRMATION:
        blockers.append("PAID_API_CONFIRMATION_REQUIRED")
    if mode == "mock" and args.mock_responses is None:
        blockers.append("MOCK_RESPONSES_REQUIRED")

    records: list[dict[str, Any]] = []
    if not blockers and mode in {"mock", "execute"}:
        mock_dir = contained(args.mock_responses, strict=True) if args.mock_responses else None
        records = asyncio.run(
            run_requests(
                tasks,
                system_prompt=str(contract["system_prompt"]),
                output_dir=output_dir,
                mode=mode,
                mock_dir=mock_dir,
                resume=args.resume,
            )
        )

    if blockers:
        status = "BLOCKED"
        evidence_class = "PREFLIGHT_ONLY"
    elif mode == "preflight":
        status = "READY_NOT_RUN"
        evidence_class = "PREFLIGHT_ONLY"
    elif mode == "mock":
        status = "MOCK_COMPLETE"
        evidence_class = "MOCK_ONLY"
    elif all(record.get("accepted") for record in records) and len(records) == 18:
        status = "COMPLETE"
        evidence_class = "LOCAL_DIRECT"
    else:
        status = "COMPLETE_WITH_OUTPUT_FAILURES"
        evidence_class = "LOCAL_DIRECT"

    summary = {
        "schema_version": 1,
        "protocol_id": contract["protocol_id"],
        "baseline": "naive_same_llm",
        "status": status,
        "evidence_class": evidence_class,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "mode": mode,
        "network_accessed": mode == "execute" and bool(records),
        "paid_api_called": mode == "execute" and bool(records),
        "manifest": relative(manifest_path) if manifest_path.exists() else relative(manifest_path),
        "manifest_sha256": manifest_sha,
        "expected_manifest_sha256": args.expected_manifest_sha256,
        "contract": relative(contract_path),
        "contract_sha256": contract_sha,
        "frozen_settings": FROZEN_SETTINGS,
        "pricing_snapshot": FROZEN_PRICING,
        "requirements": {
            "manifest_available": manifest is not None,
            "manifest_sha256_verified": manifest_sha_verified,
            "ready_tasks": len(tasks),
            "preview_gates_passed": sum(
                row["full_source_present"] and row["output_cap_preserved"]
                for row in preview_rows
            ),
            "full_source_no_truncation": all(
                row["full_source_present"] for row in preview_rows
            ) if preview_rows else False,
            "tools_exposed": 0,
            "provider_request_limit": 1,
            "compile_feedback": False,
            "automatic_repair": False,
        },
        "blockers": sorted(set(blockers)),
        "previews": preview_rows,
        "records": records,
        "totals": totals(records),
        "run_policy": {
            "real_execution_requires_confirmation": PAID_CONFIRMATION,
            "duplicate_task_output_refused": True,
            "provider_transport_retries": 0,
            "continuation_on_truncation": False,
            "compile_or_syntax_feedback_to_model": False,
        },
    }
    dump_json(output_dir / "summary.json", summary)
    write_report(output_dir, summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 2 if blockers else 0


def main() -> int:
    args = parse_args()
    mode = "execute" if args.execute else args.mode
    output_dir = contained(args.output_dir)
    with output_root_lock(
        output_dir,
        mode=mode,
        expected_manifest_sha256=args.expected_manifest_sha256,
    ):
        return run_locked(args, mode=mode, output_dir=output_dir)


if __name__ == "__main__":
    raise SystemExit(main())
