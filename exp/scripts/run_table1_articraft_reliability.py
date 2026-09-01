#!/usr/bin/env python3
"""Fail-closed Articraft preflight for Nano3D Table 1 reliability.

This runner deliberately makes no model/API call.  A comparable Articraft
authoring result requires the cross-method Table 1 task manifest, exact method
configuration, attempt-boundary telemetry, and the common evaluator to be frozen
before generation.  Until those inputs exist, the runner audits local readiness
and emits N/R metrics rather than repurposing the paper's manual-retention rate or
an unrelated local authoring experiment.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import shutil
import statistics
import subprocess
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
DEFAULT_OUTPUT = REPO_ROOT / "exp/runtime/table1_reliability/articraft"
DEFAULT_PROTOCOL = REPO_ROOT / "exp/reference/table1_reliability_protocol_v1.json"
DEFAULT_COMMON_MANIFEST = REPO_ROOT / "exp/reference/table1_reliability_common_authoring_v1.json"
ARTICRAFT_ROOT = REPO_ROOT / "articraft_data"
ARTICRAFT_PYTHON = ARTICRAFT_ROOT / ".venv/bin/python"
ARTICRAFT_CLI = ARTICRAFT_ROOT / ".venv/bin/articraft"
OFFICIAL_RELEASE_ROOT = REPO_ROOT / "exp/baselines/Articraft-10K-official"

PAPER_RETENTION_NUMERATOR = 9731
PAPER_RETENTION_DENOMINATOR = 10611
EXPECTED_TASKS = 54
EXPECTED_REPEATS = 3
EXPECTED_RUNS = EXPECTED_TASKS * EXPECTED_REPEATS

PROVIDER_ENV_NAMES = {
    "openai": ("OPENAI_API_KEY",),
    "gemini": ("GEMINI_API_KEYS", "GEMINI_API_KEY"),
    "anthropic": ("ANTHROPIC_API_KEYS", "ANTHROPIC_API_KEY"),
    "dashscope": ("DASHSCOPE_API_KEY",),
    "openrouter": ("OPENROUTER_API_KEY",),
    "deepseek": ("DEEPSEEK_API_KEY",),
}
SENSITIVE_ASSIGNMENT_RE = re.compile(
    r"(?im)^\s*(?:OPENAI|GEMINI|ANTHROPIC|DASHSCOPE|OPENROUTER|DEEPSEEK)_[A-Z_]*KEYS?\s*="
)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def contained(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path is outside the authorized workspace: {resolved}")
    return resolved


def canonical_json(payload: Any) -> bytes:
    return json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_text_atomic(path: Path, text: str) -> None:
    path = contained(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def write_json(path: Path, payload: Any) -> None:
    write_text_atomic(
        path,
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
    )


def run_command(command: list[str], *, cwd: Path, timeout: float = 60.0) -> dict[str, Any]:
    started = datetime.now(timezone.utc)
    try:
        completed = subprocess.run(
            command,
            cwd=contained(cwd),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            check=False,
            env={**os.environ, "PYTHONNOUSERSITE": "1"},
        )
        return {
            "ok": completed.returncode == 0,
            "exit_code": completed.returncode,
            "stdout_tail": completed.stdout[-2000:],
            "stderr_tail": completed.stderr[-2000:],
            "started_at_utc": started.isoformat(),
        }
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {
            "ok": False,
            "exit_code": None,
            "error": type(exc).__name__,
            "started_at_utc": started.isoformat(),
        }


def git_snapshot(root: Path) -> dict[str, Any]:
    head = run_command(["git", "rev-parse", "HEAD"], cwd=root)
    status = run_command(
        ["git", "status", "--porcelain", "--untracked-files=no"], cwd=root
    )
    return {
        "path": str(contained(root)),
        "head": head.get("stdout_tail", "").strip() if head["ok"] else None,
        "tracked_worktree_dirty": bool(status.get("stdout_tail", "").strip())
        if status["ok"]
        else None,
        "git_available": bool(head["ok"] and status["ok"]),
    }


def env_key_presence(env_file: Path) -> dict[str, bool]:
    """Return key-presence booleans without returning or persisting values."""

    file_presence: dict[str, bool] = {}
    if env_file.is_file():
        for raw_line in env_file.read_text(encoding="utf-8", errors="replace").splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, raw_value = line.split("=", 1)
            key = key.strip()
            value_present = bool(raw_value.strip().strip("'\""))
            if key in {name for names in PROVIDER_ENV_NAMES.values() for name in names} | {
                "ARTICRAFT_MODEL",
                "ARTICRAFT_THINKING_LEVEL",
            }:
                file_presence[key] = value_present

    result: dict[str, bool] = {}
    for names in PROVIDER_ENV_NAMES.values():
        for name in names:
            result[name] = bool(os.environ.get(name)) or file_presence.get(name, False)
    result["ARTICRAFT_MODEL"] = bool(os.environ.get("ARTICRAFT_MODEL")) or file_presence.get(
        "ARTICRAFT_MODEL", False
    )
    result["ARTICRAFT_THINKING_LEVEL"] = bool(
        os.environ.get("ARTICRAFT_THINKING_LEVEL")
    ) or file_presence.get("ARTICRAFT_THINKING_LEVEL", False)
    return result


def dataset_audit(index_path: Path, records_root: Path) -> dict[str, Any]:
    rows = 0
    invalid_json_rows = 0
    record_ids: set[str] = set()
    if index_path.is_file():
        with index_path.open("r", encoding="utf-8", errors="replace") as handle:
            for line in handle:
                if not line.strip():
                    continue
                rows += 1
                try:
                    payload = json.loads(line)
                except json.JSONDecodeError:
                    invalid_json_rows += 1
                    continue
                if isinstance(payload, dict) and isinstance(payload.get("record_id"), str):
                    record_ids.add(payload["record_id"])

    hydrated_record_dirs = 0
    hydrated_record_json = 0
    if records_root.is_dir():
        for entry in os.scandir(records_root):
            if entry.is_dir(follow_symlinks=False):
                hydrated_record_dirs += 1
                if (Path(entry.path) / "record.json").is_file():
                    hydrated_record_json += 1
    return {
        "index_path": str(contained(index_path)),
        "index_sha256": sha256_file(index_path) if index_path.is_file() else None,
        "index_rows": rows,
        "unique_record_ids": len(record_ids),
        "invalid_json_rows": invalid_json_rows,
        "hydrated_record_directories": hydrated_record_dirs,
        "hydrated_record_json_files": hydrated_record_json,
        "note": "Current checkout inventory; not the paper's 10,611-generation denominator.",
    }


def jsonl_rows(path: Path) -> tuple[list[dict[str, Any]], int]:
    rows: list[dict[str, Any]] = []
    invalid = 0
    if not path.is_file():
        return rows, invalid
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                invalid += 1
                continue
            if isinstance(payload, dict):
                rows.append(payload)
            else:
                invalid += 1
    return rows, invalid


def field_coverage(rows: list[dict[str, Any]], field: str) -> dict[str, Any]:
    count = sum(row.get(field) is not None for row in rows)
    denominator = len(rows)
    return {
        "field": field,
        "count": count,
        "denominator": denominator,
        "rate": count / denominator if denominator else None,
    }


def path_has_any_file(path: Path) -> bool:
    if not path.is_dir():
        return False
    try:
        return any(entry.is_file(follow_symlinks=False) for entry in os.scandir(path))
    except OSError:
        return False


def numeric_telemetry(rows: list[dict[str, Any]], field: str) -> dict[str, Any]:
    values = [
        value
        for row in rows
        if isinstance((value := row.get(field)), (int, float))
        and not isinstance(value, bool)
    ]
    denominator = len(rows)
    result: dict[str, Any] = {
        "field": field,
        "count": len(values),
        "denominator": denominator,
        "coverage": len(values) / denominator if denominator else None,
        "sum": None,
        "mean": None,
        "median": None,
        "p95_linear": None,
        "minimum": None,
        "maximum": None,
    }
    if not values:
        return result
    ordered = sorted(values)
    position = (len(ordered) - 1) * 0.95
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    p95 = ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)
    result.update(
        {
            "sum": sum(values),
            "mean": statistics.fmean(values),
            "median": statistics.median(values),
            "p95_linear": p95,
            "minimum": min(values),
            "maximum": max(values),
        }
    )
    return result


def counter_from_field(rows: list[dict[str, Any]], field: str) -> dict[str, int]:
    counts = Counter("NULL" if row.get(field) is None else str(row[field]) for row in rows)
    return dict(sorted(counts.items()))


def rating_audit(rows: list[dict[str, Any]], field: str) -> dict[str, Any]:
    values = [
        value
        for row in rows
        if isinstance((value := row.get(field)), (int, float))
        and not isinstance(value, bool)
    ]
    retained = sum(value >= 4 for value in values)
    distribution = Counter(str(value) for value in values)
    return {
        "field": field,
        "rated_count": len(values),
        "release_index_rows": len(rows),
        "missing_count": len(rows) - len(values),
        "distribution": dict(sorted(distribution.items())),
        "rating_ge_4_count": retained,
        "rating_ge_4_denominator": len(values),
        "rating_ge_4_rate": retained / len(values) if values else None,
        "claim_boundary": (
            "Current official-release snapshot rating coverage only; not the paper's "
            "9,731/10,611 retention cohort and not a Table 1 success metric."
        ),
    }


def official_release_audit(root: Path) -> dict[str, Any]:
    root = contained(root)
    index_path = root / "records_index.jsonl"
    manifest_path = root / "records_manifest.jsonl"
    index_rows, index_invalid = jsonl_rows(index_path)
    manifest_rows, manifest_invalid = jsonl_rows(manifest_path)
    index_ids = {
        row["record_id"]
        for row in index_rows
        if isinstance(row.get("record_id"), str)
    }
    manifest_ids = {
        row["record_id"]
        for row in manifest_rows
        if isinstance(row.get("record_id"), str)
    }

    declared_flags: dict[str, Any] = {}
    for field in (
        "has_cost",
        "has_traces",
        "has_provenance",
        "has_history",
        "has_compile_report",
    ):
        declared_flags[field] = {
            "true": sum(row.get(field) is True for row in index_rows),
            "false": sum(row.get(field) is False for row in index_rows),
            "null_or_other": sum(
                row.get(field) is not True and row.get(field) is not False
                for row in index_rows
            ),
            "denominator": len(index_rows),
        }

    artifact_patterns = {
        "record_json": lambda record, revision: record / "record.json",
        "revision_json": lambda record, revision: revision / "revision.json",
        "model_py": lambda record, revision: revision / "model.py",
        "cost_json": lambda record, revision: revision / "cost.json",
        "provenance_json": lambda record, revision: revision / "provenance.json",
        "trajectory_jsonl_zst": lambda record, revision: revision
        / "traces/trajectory.jsonl.zst",
        "compile_report_json": lambda record, revision: revision / "compile_report.json",
    }
    verified_presence = {name: 0 for name in artifact_patterns}
    flag_path_mismatches = {
        "has_cost": 0,
        "has_traces": 0,
        "has_provenance": 0,
        "has_compile_report": 0,
    }
    for row in index_rows:
        record_id = row.get("record_id")
        revision_id = row.get("active_revision_id")
        if not isinstance(record_id, str) or not isinstance(revision_id, str):
            continue
        record_root = root / "records" / record_id
        revision_root = record_root / "revisions" / revision_id
        present: dict[str, bool] = {}
        for name, resolve in artifact_patterns.items():
            exists = resolve(record_root, revision_root).is_file()
            present[name] = exists
            verified_presence[name] += int(exists)
        comparisons = {
            "has_cost": present["cost_json"],
            "has_traces": path_has_any_file(revision_root / "traces"),
            "has_provenance": present["provenance_json"],
            "has_compile_report": present["compile_report_json"],
        }
        for flag, exists in comparisons.items():
            flag_path_mismatches[flag] += int(bool(row.get(flag)) != exists)

    run_status_counts = counter_from_field(index_rows, "run_status")
    telemetry = {
        field: numeric_telemetry(index_rows, field)
        for field in ("turn_count", "input_tokens", "output_tokens", "total_cost_usd")
    }
    exact_wall_time_field_count = sum(
        any(row.get(field) is not None for field in ("wall_time_s", "elapsed_s", "duration_s"))
        for row in index_rows
    )
    return {
        "schema_version": 1,
        "audit_id": "articraft_official_release_telemetry_v1",
        "evidence_class": "SUPPLEMENTARY_OFFICIAL_RELEASE_AUDIT",
        "included_in_common_authoring_metrics": False,
        "claim_boundary": (
            "Deterministic audit of fields in the local official Articraft-10K release "
            "snapshot. It is not a common-task rerun. run_status=success is not mapped "
            "to Table 1 Final Success, and rating>=4 is not mapped to First-shot or Final Success."
        ),
        "release_checkout": git_snapshot(root),
        "records_index": {
            "path": str(index_path),
            "sha256": sha256_file(index_path) if index_path.is_file() else None,
            "rows": len(index_rows),
            "invalid_json_rows": index_invalid,
            "unique_record_ids": len(index_ids),
        },
        "records_manifest": {
            "path": str(manifest_path),
            "sha256": sha256_file(manifest_path) if manifest_path.is_file() else None,
            "rows": len(manifest_rows),
            "invalid_json_rows": manifest_invalid,
            "unique_record_ids": len(manifest_ids),
        },
        "index_manifest_alignment": {
            "shared_record_ids": len(index_ids & manifest_ids),
            "index_only_count": len(index_ids - manifest_ids),
            "manifest_only_count": len(manifest_ids - index_ids),
            "index_only_record_ids": sorted(index_ids - manifest_ids),
            "manifest_only_record_ids": sorted(manifest_ids - index_ids),
        },
        "run_status": {
            "counts": run_status_counts,
            "denominator": len(index_rows),
            "table1_final_success_mapping": None,
            "mapping_note": "Release lifecycle status, not the frozen common evaluator verdict.",
        },
        "declared_artifact_flags": declared_flags,
        "verified_active_revision_artifacts": {
            "counts": verified_presence,
            "denominator": len(index_rows),
            "declared_flag_path_mismatch_counts": flag_path_mismatches,
            "path_contract": "records/<record_id>/revisions/<active_revision_id>/...",
        },
        "telemetry": telemetry,
        "time_coverage": {
            "created_at": field_coverage(index_rows, "created_at"),
            "updated_at": field_coverage(index_rows, "updated_at"),
            "promoted_at": field_coverage(index_rows, "promoted_at"),
            "exact_authoring_wall_time_field_count": exact_wall_time_field_count,
            "denominator": len(index_rows),
            "wall_time_summary": None,
            "note": (
                "created_at/updated_at/promoted_at are lifecycle timestamps; their differences "
                "are not assumed to equal authoring wall time."
            ),
        },
        "rating": {
            "primary": rating_audit(index_rows, "rating"),
            "secondary": rating_audit(index_rows, "secondary_rating"),
            "effective": rating_audit(index_rows, "effective_rating"),
        },
        "paper_retention_denominator_matches_index": len(index_rows)
        == PAPER_RETENTION_DENOMINATOR,
        "paper_retention_denominator_matches_manifest": len(manifest_rows)
        == PAPER_RETENTION_DENOMINATOR,
        "network_accessed": False,
    }


def load_json_object(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    if not path.is_file() or path.is_symlink():
        return None, "missing or non-regular file"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, f"{type(exc).__name__}: {exc}"
    if not isinstance(payload, dict):
        return None, "top-level JSON value is not an object"
    return payload, None


def inspect_common_manifest(path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {
        "path": str(contained(path)),
        "exists": path.is_file(),
        "sha256": None,
        "valid_json_object": False,
        "error": None,
        "manifest_id": None,
        "task_count": None,
        "declared_task_count": None,
        "declared_task_count_matches": False,
        "task_ids_unique": False,
        "prompts_nonempty": False,
        "prompt_sha256_matches": False,
        "frozen": False,
        "frozen_at_utc_nonempty": False,
        "repeat_ids": [],
        "repeat_count": None,
        "repeat_ids_unique": False,
        "expected_runs_per_method": None,
    }
    payload, error = load_json_object(path)
    if payload is None:
        result["error"] = error
        return result
    result["sha256"] = sha256_file(path)
    result["valid_json_object"] = True
    tasks = payload.get("tasks")
    repeats = payload.get("repeat_ids")
    result["task_count"] = len(tasks) if isinstance(tasks, list) else None
    result["declared_task_count"] = payload.get("task_count")
    result["declared_task_count_matches"] = (
        isinstance(tasks, list) and payload.get("task_count") == len(tasks)
    )
    result["repeat_count"] = len(repeats) if isinstance(repeats, list) else None
    normalized_repeats = (
        [str(value).strip() for value in repeats] if isinstance(repeats, list) else []
    )
    result["repeat_ids"] = normalized_repeats
    result["repeat_ids_unique"] = bool(normalized_repeats) and all(normalized_repeats) and (
        len(normalized_repeats) == len(set(normalized_repeats))
    )
    result["manifest_id"] = payload.get("manifest_id")
    result["frozen"] = payload.get("frozen") is True
    result["frozen_at_utc_nonempty"] = bool(str(payload.get("frozen_at_utc", "")).strip())
    result["expected_runs_per_method"] = payload.get("expected_runs_per_method")

    task_ids: list[str] = []
    prompts_nonempty = True
    prompt_sha256_matches = True
    if not isinstance(tasks, list) or not tasks:
        prompts_nonempty = False
        prompt_sha256_matches = False
    else:
        for task in tasks:
            if not isinstance(task, dict):
                prompts_nonempty = False
                prompt_sha256_matches = False
                continue
            task_id = str(task.get("task_id", "")).strip()
            prompt = str(task.get("prompt", "")).strip()
            if task_id:
                task_ids.append(task_id)
            prompts_nonempty &= bool(prompt)
            expected_sha = hashlib.sha256(prompt.encode("utf-8")).hexdigest() if prompt else None
            prompt_sha256_matches &= bool(
                expected_sha
                and isinstance(task.get("prompt_sha256"), str)
                and task["prompt_sha256"].lower() == expected_sha
            )
    result["task_ids_unique"] = bool(tasks) and len(task_ids) == len(tasks) and (
        len(task_ids) == len(set(task_ids))
    )
    result["prompts_nonempty"] = prompts_nonempty
    result["prompt_sha256_matches"] = prompt_sha256_matches
    return result


def inspect_protocol(path: Path, common: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {
        "path": str(contained(path)),
        "exists": path.is_file(),
        "sha256": None,
        "valid_json_object": False,
        "error": None,
        "protocol_id": None,
        "schema_version_present": False,
        "frozen_at_utc_nonempty": False,
        "frozen_before_first_run": False,
        "manifest_sha256_bound": False,
        "hidden_specs_sha256_frozen": False,
        "hidden_specs_withheld": False,
        "expected_task_count_matches": False,
        "independent_runs_per_task_matches": False,
        "repeat_ids_match_manifest": False,
        "repair_budget_frozen": False,
        "method_ids": [],
        "method_ids_exact": False,
        "articraft_method_present": False,
        "articraft_method_arm": None,
        "common_model_binding": {},
        "openai_gpt5_exact_binding": False,
        "articraft_roles_match_common_binding": False,
        "sampling_frozen": False,
        "attempt_boundary_frozen": False,
        "common_evaluator_frozen": False,
        "method_output_isolation_frozen": False,
        "telemetry_frozen": False,
        "timeouts_frozen": False,
        "execution_readiness_declared": False,
    }
    payload, error = load_json_object(path)
    if payload is None:
        result["error"] = error
        return result
    result["sha256"] = sha256_file(path)
    result["valid_json_object"] = True
    result["protocol_id"] = payload.get("protocol_id")
    result["schema_version_present"] = isinstance(payload.get("schema_version"), (int, str))
    result["frozen_at_utc_nonempty"] = bool(str(payload.get("frozen_at_utc", "")).strip())
    result["frozen_before_first_run"] = payload.get("frozen_before_first_run") is True or (
        payload.get("frozen_design") is True
    )

    legacy = payload.get("frozen_protocol")
    legacy = legacy if isinstance(legacy, dict) else {}
    manifest_binding = payload.get("manifest")
    bound_sha = manifest_binding.get("sha256") if isinstance(manifest_binding, dict) else None
    if not bound_sha:
        bound_sha = payload.get("manifest_sha256", legacy.get("manifest_sha256"))
    result["manifest_sha256_bound"] = bool(common.get("sha256")) and bound_sha == common["sha256"]
    hidden_specs = payload.get("hidden_specs")
    hidden_specs_sha = (
        hidden_specs.get("sha256")
        if isinstance(hidden_specs, dict)
        else payload.get("hidden_specs_sha256", legacy.get("hidden_specs_sha256"))
    )
    result["hidden_specs_sha256_frozen"] = isinstance(hidden_specs_sha, str) and bool(
        re.fullmatch(r"[0-9a-fA-F]{64}", hidden_specs_sha)
    )
    hidden_policy = payload.get("hidden_policy")
    result["hidden_specs_withheld"] = (
        isinstance(hidden_specs, dict) and hidden_specs.get("withheld_from_author") is True
    ) or (
        isinstance(hidden_specs, dict)
        and str(hidden_specs.get("visibility", "")).strip().lower() == "evaluator_only"
    ) or (
        isinstance(hidden_policy, dict)
        and hidden_policy.get("generation_access_to_hidden_specs") is False
        and hidden_policy.get("evaluator_only") is True
    ) or legacy.get("hidden_specs_withheld") is True
    result["expected_task_count_matches"] = (
        payload.get("expected_task_count", legacy.get("expected_task_count")) == EXPECTED_TASKS
    )
    result["independent_runs_per_task_matches"] = (
        payload.get("independent_runs_per_task", legacy.get("independent_runs_per_task"))
        == EXPECTED_REPEATS
    )
    protocol_repeat_ids = payload.get("repeat_ids", legacy.get("repeat_ids"))
    protocol_repeat_ids = (
        [str(value).strip() for value in protocol_repeat_ids]
        if isinstance(protocol_repeat_ids, list)
        else []
    )
    result["repeat_ids"] = protocol_repeat_ids
    result["repeat_ids_match_manifest"] = bool(protocol_repeat_ids) and (
        protocol_repeat_ids == common.get("repeat_ids")
    )
    repair_budget = payload.get(
        "max_common_repair_turns",
        legacy.get("max_common_repair_turns", legacy.get("max_repair_turns")),
    )
    result["repair_budget_frozen"] = repair_budget == 3

    common_binding = payload.get("common_model_binding", legacy.get("common_model_binding"))
    if not isinstance(common_binding, dict):
        common_binding = {
            "provider": payload.get("provider", legacy.get("provider")),
            "model_id": payload.get(
                "model_id",
                payload.get("exact_model_id", legacy.get("exact_model_id")),
            ),
            "sampling_config": payload.get(
                "sampling_config", legacy.get("sampling_config")
            ),
        }
    result["common_model_binding"] = common_binding
    result["openai_gpt5_exact_binding"] = (
        str(common_binding.get("provider", "")).strip().lower() == "openai"
        and str(
            common_binding.get(
                "model_id",
                common_binding.get("model", common_binding.get("exact_model_id", "")),
            )
        ).strip()
        == "gpt-5"
    )
    methods = payload.get("methods", legacy.get("methods", []))
    articraft_arm: dict[str, Any] | None = None
    method_ids: list[str] = []
    if isinstance(methods, list):
        for method in methods:
            method_id = (
                method.strip().lower()
                if isinstance(method, str)
                else str(method.get("method_id", "")).strip().lower()
                if isinstance(method, dict)
                else ""
            )
            if method_id:
                method_ids.append(method_id)
            if isinstance(method, str) and method_id == "articraft":
                result["articraft_method_present"] = True
                result["articraft_roles_match_common_binding"] = result[
                    "openai_gpt5_exact_binding"
                ]
            elif isinstance(method, dict) and method_id == "articraft":
                result["articraft_method_present"] = True
                articraft_arm = method
    elif isinstance(methods, dict):
        method_ids = [str(key).strip().lower() for key in methods if str(key).strip()]
        candidate = methods.get("articraft")
        if isinstance(candidate, dict):
            result["articraft_method_present"] = True
            articraft_arm = candidate
    result["method_ids"] = method_ids
    result["method_ids_exact"] = set(method_ids) == {"pva", "lam", "articraft"} and (
        len(method_ids) == 3
    )
    roles = articraft_arm.get("roles") if articraft_arm else None
    result["articraft_method_arm"] = articraft_arm
    request_parameters = (
        articraft_arm.get("request_parameters") if isinstance(articraft_arm, dict) else None
    )

    def frozen_request_parameter(key: str) -> bool:
        if not isinstance(request_parameters, dict):
            return False
        row = request_parameters.get(key)
        if not isinstance(row, dict) or not isinstance(row.get("sent"), bool):
            return False
        if row["sent"]:
            return row.get("value") is not None
        return row.get("value") is None and isinstance(row.get("reason"), str) and bool(
            row["reason"].strip()
        )

    result["sampling_frozen"] = all(
        frozen_request_parameter(key)
        for key in ("temperature", "top_p", "max_output_tokens")
    )
    if isinstance(roles, list) and roles:
        result["articraft_roles_match_common_binding"] = all(
            isinstance(role, dict)
            and str(role.get("provider", "")).strip().lower() == "openai"
            and str(role.get("model_id", role.get("exact_model_id", ""))).strip() == "gpt-5"
            for role in roles
        )
    elif articraft_arm:
        result["articraft_roles_match_common_binding"] = (
            str(articraft_arm.get("provider", "")).strip().lower() == "openai"
            and str(
                articraft_arm.get(
                    "model_id",
                    articraft_arm.get("model", articraft_arm.get("exact_model_id", "")),
                )
            ).strip()
            == "gpt-5"
        )

    definitions = payload.get("definitions", legacy.get("definitions"))
    definition_blob = json.dumps(
        {
            "definitions": definitions,
            "attempt_semantics": payload.get("attempt_semantics"),
            "metric_definitions": payload.get("metric_definitions"),
        },
        sort_keys=True,
    ).lower()
    required_definition_aliases = (
        ("attempt_0", "attempt0", "attempt_boundary_definition"),
        ("common_repair_turn", "common_repair", "repair_turn"),
        ("first_shot",),
        ("final_success",),
    )
    result["attempt_boundary_frozen"] = bool(definition_blob) and all(
        any(alias in definition_blob for alias in aliases)
        for aliases in required_definition_aliases
    )
    result["common_evaluator_frozen"] = bool(
        payload.get("common_evaluator", legacy.get("common_evaluator"))
    )
    result["method_output_isolation_frozen"] = bool(
        payload.get(
            "method_output_isolation",
            payload.get("output_isolation", legacy.get("method_output_isolation")),
        )
    )
    result["telemetry_frozen"] = bool(
        payload.get(
            "telemetry_fields",
            payload.get("telemetry_requirements", legacy.get("telemetry_fields")),
        )
    )
    result["timeouts_frozen"] = bool(
        payload.get("timeouts_s", payload.get("timeouts", legacy.get("timeouts_s")))
    )
    result["execution_readiness_declared"] = bool(
        payload.get("execution_readiness", legacy.get("execution_readiness"))
    )
    return result


def excluded_local_authoring_evidence(path: Path) -> dict[str, Any]:
    result: dict[str, Any] = {"path": str(contained(path)), "exists": path.is_file()}
    if not path.is_file():
        return result
    payload = json.loads(path.read_text(encoding="utf-8"))
    methods = payload.get("methods", []) if isinstance(payload, dict) else []
    result.update(
        {
            "sha256": sha256_file(path),
            "protocol": payload.get("protocol"),
            "task_count": payload.get("task_count"),
            "repeat_count": payload.get("repeat_count"),
            "completed_runs": payload.get("completed_runs"),
            "method_ids": [
                row.get("method_id") for row in methods if isinstance(row, dict)
            ],
            "articraft_method_present": any(
                isinstance(row, dict) and row.get("method_id") == "articraft"
                for row in methods
            ),
            "included_in_table1_articraft_metrics": False,
            "exclusion_reason": (
                "PV-A authoring ablation on six tasks and one repeat; it contains no "
                "native Articraft method arm and is not the cross-method Table 1 cohort."
            ),
        }
    )
    return result


def input_hashes(paths: list[Path]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in paths:
        rows.append(
            {
                "path": str(contained(path)),
                "exists": path.is_file(),
                "sha256": sha256_file(path) if path.is_file() else None,
            }
        )
    return rows


def report_markdown(summary: dict[str, Any], checks: dict[str, Any]) -> str:
    paper = summary["paper_context"]["manual_retention"]
    dataset = summary["local_readiness"]["dataset"]
    release = summary["supplementary_official_release_audit"]
    release_index = release["records_index"]
    release_manifest = release["records_manifest"]
    release_flags = release["declared_artifact_flags"]
    release_telemetry = release["telemetry"]
    effective_rating = release["rating"]["effective"]
    blockers = "\n".join(f"- {item}" for item in summary["blockers"])
    configured = summary["local_readiness"]["provider_configuration"][
        "configured_provider_names"
    ]
    return f"""# Articraft Table 1 Reliability Preflight

Status: **{summary['status']}** (`{summary['evidence_class']}`)

No common authoring generation was executed. The local checkout, SDK/runtime, CLI,
dataset index, and provider-name readiness were audited without making a network or
paid provider call. Table 1 authoring metrics remain **N/R**, not zero.

## Table 1 row

| Method | Evaluation Unit | N | Executable | Artifact Saved | First-shot | Final Success | Repair Turns | Wall Time | Tokens | API Cost |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Articraft | common text/image item -> SDK program + articulated URDF | N/R (preflight only) | N/R | N/R | N/R | N/R | N/R | N/R | N/R | N/R |

## Paper-only context

The paper-reported manual retention is `{paper['numerator']}/{paper['denominator']} = {paper['rate_percent_1dp']:.1f}%`.
It is a post-generation human-rating filter, not executable rate, artifact-saved rate,
first-shot success, or final repair-loop success. It is excluded from every local
Table 1 metric numerator and denominator.

## Local readiness

- Articraft checkout HEAD: `{summary['local_readiness']['checkout']['head']}`
- Python/runtime import smoke: `{summary['local_readiness']['runtime']['import_smoke_ok']}`
- `articraft generate --help`: `{summary['local_readiness']['runtime']['generate_cli_help_ok']}`
- Current `records_index.jsonl`: `{dataset['index_rows']}` rows, `{dataset['unique_record_ids']}` unique record IDs, `{dataset['invalid_json_rows']}` invalid JSON rows
- Configured provider names (presence only): `{', '.join(configured) if configured else 'none'}`
- Exact common model bound to `openai/gpt-5`: `{summary['common_protocol']['openai_gpt5_exact_binding']}`
- Network/API calls made by this runner: `0`

The current checkout inventory is not the paper denominator and is not a local
authoring-success cohort. The completed local T2 formal run is also excluded because
it is a six-task PV-A ablation with no native Articraft arm.

## Official-release telemetry (supplementary)

This panel audits the local official Articraft-10K release snapshot. It is not pooled
with the common authoring row above.

| Field | Direct count / denominator | Direct summary | Table 1 mapping |
|---|---:|---|---|
| records index | {release_index['rows']} / {release_index['rows']} | {release_index['unique_record_ids']} unique IDs; invalid JSON={release_index['invalid_json_rows']} | none |
| records manifest | {release_manifest['rows']} / {release_manifest['rows']} | {release_manifest['unique_record_ids']} unique IDs; invalid JSON={release_manifest['invalid_json_rows']} | none |
| run status | {release['run_status']['denominator']} / {release_index['rows']} | `{json.dumps(release['run_status']['counts'], sort_keys=True)}` | not Final Success |
| cost files declared | {release_flags['has_cost']['true']} / {release_flags['has_cost']['denominator']} | verified active-revision files={release['verified_active_revision_artifacts']['counts']['cost_json']} | supplementary only |
| traces declared | {release_flags['has_traces']['true']} / {release_flags['has_traces']['denominator']} | verified trajectories={release['verified_active_revision_artifacts']['counts']['trajectory_jsonl_zst']} | supplementary only |
| provenance declared | {release_flags['has_provenance']['true']} / {release_flags['has_provenance']['denominator']} | verified files={release['verified_active_revision_artifacts']['counts']['provenance_json']} | supplementary only |
| history declared | {release_flags['has_history']['true']} / {release_flags['has_history']['denominator']} | release flag only | no repair-turn inference |
| compile report declared | {release_flags['has_compile_report']['true']} / {release_flags['has_compile_report']['denominator']} | verified files={release['verified_active_revision_artifacts']['counts']['compile_report_json']} | no Executable inference |
| turn count | {release_telemetry['turn_count']['count']} / {release_telemetry['turn_count']['denominator']} | mean={release_telemetry['turn_count']['mean']:.3f}; median={release_telemetry['turn_count']['median']}; p95={release_telemetry['turn_count']['p95_linear']:.3f} | not Repair Turns |
| input tokens | {release_telemetry['input_tokens']['count']} / {release_telemetry['input_tokens']['denominator']} | mean={release_telemetry['input_tokens']['mean']:.3f}; median={release_telemetry['input_tokens']['median']}; total={release_telemetry['input_tokens']['sum']} | release telemetry only |
| output tokens | {release_telemetry['output_tokens']['count']} / {release_telemetry['output_tokens']['denominator']} | mean={release_telemetry['output_tokens']['mean']:.3f}; median={release_telemetry['output_tokens']['median']}; total={release_telemetry['output_tokens']['sum']} | release telemetry only |
| API cost USD | {release_telemetry['total_cost_usd']['count']} / {release_telemetry['total_cost_usd']['denominator']} | mean={release_telemetry['total_cost_usd']['mean']:.6f}; median={release_telemetry['total_cost_usd']['median']:.6f}; total={release_telemetry['total_cost_usd']['sum']:.6f} | release telemetry only |
| effective rating >=4 | {effective_rating['rating_ge_4_count']} / {effective_rating['rating_ge_4_denominator']} | {100 * effective_rating['rating_ge_4_rate']:.3f}% of rated release-index rows | not First-shot/Final Success |
| exact authoring wall time | {release['time_coverage']['exact_authoring_wall_time_field_count']} / {release['time_coverage']['denominator']} | N/R; lifecycle timestamps are not differenced | N/R |

The index and manifest differ by
`{release['index_manifest_alignment']['manifest_only_count']}` manifest-only record and
`{release['index_manifest_alignment']['index_only_count']}` index-only records. Neither
row count equals the paper's 10,611-generation denominator. `run_status=success`,
rating retention, turns, tokens, and cost are direct release telemetry, but the
release does not expose the frozen common-evaluator attempt boundary needed for
Table 1 First-shot or Final Success.

## Blockers

{blockers}

## Reproduce

```bash
python exp/scripts/run_table1_articraft_reliability.py \\
  --output-dir exp/runtime/table1_reliability/articraft
```

Use `--require-ready` to make the preflight return exit code 2 while any hard gate is
unmet. `ADAPTER_REQUIRED` means the frozen inputs and runtime gates pass but the
evaluator/repair/output-isolation adapters are still absent. A real rerun needs a
separate executor that consumes the frozen
common manifest and emits per-attempt telemetry plus common-evaluator verdicts.

Self-check: `{checks['passed']}/{checks['total']} passed`; internal artifact validity:
`{checks['all_passed']}`. This validates the fail-closed report, not the experiment.
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--common-manifest", type=Path, default=DEFAULT_COMMON_MANIFEST)
    parser.add_argument(
        "--require-ready",
        action="store_true",
        help="Exit 2 after writing evidence when the common rerun is not ready.",
    )
    args = parser.parse_args()

    output_dir = contained(args.output_dir)
    protocol_path = contained(args.protocol)
    common_manifest_path = contained(args.common_manifest)
    output_dir.mkdir(parents=True, exist_ok=True)

    table_definition = REPO_ROOT / "exp/Nano3d.md"
    table_results = REPO_ROOT / "exp/Nano3dresults.md"
    t2_protocol = REPO_ROOT / "exp/t2_authoring_pilot/protocol.json"
    t2_readme = REPO_ROOT / "exp/t2_authoring_pilot/README.md"
    t2_formal_summary = REPO_ROOT / "exp/runtime/t2_formal_v1/authoring/summary.json"
    records_index = ARTICRAFT_ROOT / "data/records_index.jsonl"

    checkout = git_snapshot(ARTICRAFT_ROOT)
    repository = git_snapshot(REPO_ROOT)
    import_smoke = run_command(
        [
            str(ARTICRAFT_PYTHON),
            "-c",
            "import agent, articraft, cli, sdk; print('imports_ok')",
        ],
        cwd=ARTICRAFT_ROOT,
    ) if ARTICRAFT_PYTHON.is_file() else {"ok": False, "exit_code": None, "error": "missing_python"}
    cli_help = run_command(
        [str(ARTICRAFT_CLI), "generate", "--help"], cwd=ARTICRAFT_ROOT
    ) if ARTICRAFT_CLI.is_file() else {"ok": False, "exit_code": None, "error": "missing_cli"}

    env_presence = env_key_presence(ARTICRAFT_ROOT / ".env")
    configured_providers = sorted(
        provider
        for provider, names in PROVIDER_ENV_NAMES.items()
        if any(env_presence.get(name, False) for name in names)
    )
    codex_cli = shutil.which("codex")
    if codex_cli:
        configured_providers.append("codex-cli-binary-present-auth-not-probed")

    dataset = dataset_audit(records_index, ARTICRAFT_ROOT / "data/records")
    release_audit = official_release_audit(OFFICIAL_RELEASE_ROOT)
    common = inspect_common_manifest(common_manifest_path)
    protocol = inspect_protocol(protocol_path, common)
    excluded_t2 = excluded_local_authoring_evidence(t2_formal_summary)
    table_text = table_results.read_text(encoding="utf-8")
    retention_formula_present = (
        "9,731/10,611" in table_text
        and "91.7%" in table_text
        and "retention" in table_text.lower()
    )

    contract_gates = {
        "common_manifest_exists": common["exists"],
        "common_manifest_valid_json_object": common["valid_json_object"],
        "common_manifest_id_nonempty": bool(common["manifest_id"]),
        "common_manifest_has_54_tasks": common["task_count"] == EXPECTED_TASKS,
        "common_manifest_declared_task_count_matches": common[
            "declared_task_count_matches"
        ],
        "common_manifest_has_3_repeats": common["repeat_count"] == EXPECTED_REPEATS,
        "common_manifest_repeat_ids_unique": common["repeat_ids_unique"],
        "common_manifest_expected_runs_per_method_162": common[
            "expected_runs_per_method"
        ]
        == EXPECTED_RUNS,
        "common_manifest_task_ids_unique": common["task_ids_unique"],
        "common_manifest_prompts_nonempty": common["prompts_nonempty"],
        "common_manifest_prompt_sha256_matches": common["prompt_sha256_matches"],
        "common_manifest_frozen": common["frozen"],
        "common_manifest_frozen_at_utc": common["frozen_at_utc_nonempty"],
        "protocol_exists": protocol["exists"],
        "protocol_valid_json_object": protocol["valid_json_object"],
        "protocol_id_nonempty": bool(protocol["protocol_id"]),
        "protocol_schema_version_present": protocol["schema_version_present"],
        "protocol_frozen_at_utc": protocol["frozen_at_utc_nonempty"],
        "protocol_frozen_before_first_run": protocol["frozen_before_first_run"],
        "protocol_manifest_sha256_bound": protocol["manifest_sha256_bound"],
        "protocol_hidden_specs_sha256_frozen": protocol["hidden_specs_sha256_frozen"],
        "protocol_hidden_specs_withheld": protocol["hidden_specs_withheld"],
        "protocol_expected_task_count_54": protocol["expected_task_count_matches"],
        "protocol_independent_runs_per_task_3": protocol[
            "independent_runs_per_task_matches"
        ],
        "protocol_repeat_ids_match_manifest": protocol["repeat_ids_match_manifest"],
        "protocol_has_articraft_method": protocol["articraft_method_present"],
        "protocol_method_ids_exact": protocol["method_ids_exact"],
        "openai_gpt5_exact_binding": protocol["openai_gpt5_exact_binding"],
        "articraft_roles_match_openai_gpt5_binding": protocol[
            "articraft_roles_match_common_binding"
        ],
        "sampling_configuration_frozen": protocol["sampling_frozen"],
        "repair_budget_frozen": protocol["repair_budget_frozen"],
        "first_shot_attempt_boundary_frozen": protocol["attempt_boundary_frozen"],
        "common_evaluator_frozen": protocol["common_evaluator_frozen"],
        "method_output_isolation_frozen": protocol["method_output_isolation_frozen"],
        "telemetry_schema_frozen": protocol["telemetry_frozen"],
        "timeouts_frozen": protocol["timeouts_frozen"],
        "execution_readiness_declared": protocol["execution_readiness_declared"],
    }
    runtime_gates = {
        "articraft_checkout_available": checkout["git_available"],
        "articraft_runtime_imports": import_smoke["ok"],
        "articraft_generate_cli_available": cli_help["ok"],
        "openai_provider_configuration_present": env_presence["OPENAI_API_KEY"],
    }
    adapter_gates = {
        "common_evaluator_adapter_available": False,
        "common_repair_adapter_available": False,
        "run_id_output_isolation": False,
    }
    hard_gates = {**contract_gates, **runtime_gates, **adapter_gates}

    blocker_labels = {
        "common_manifest_exists": "cross-method Table 1 common authoring manifest is absent",
        "common_manifest_valid_json_object": "common authoring manifest is not a valid JSON object",
        "common_manifest_id_nonempty": "common authoring manifest id is absent",
        "common_manifest_has_54_tasks": "common manifest does not freeze the required 54 tasks",
        "common_manifest_declared_task_count_matches": "common manifest task_count does not match tasks",
        "common_manifest_has_3_repeats": "common manifest does not freeze three independent repeats",
        "common_manifest_repeat_ids_unique": "common manifest repeat_ids are empty or non-unique",
        "common_manifest_expected_runs_per_method_162": (
            "common manifest does not bind expected_runs_per_method=162"
        ),
        "common_manifest_task_ids_unique": "common manifest task_ids are empty or non-unique",
        "common_manifest_prompts_nonempty": "common manifest contains an empty prompt",
        "common_manifest_prompt_sha256_matches": "common manifest prompt SHA256 binding failed",
        "common_manifest_frozen": "common authoring manifest is not marked frozen",
        "common_manifest_frozen_at_utc": "common authoring manifest has no frozen_at_utc",
        "protocol_exists": "cross-method Table 1 protocol is absent",
        "protocol_valid_json_object": "cross-method Table 1 protocol is not a valid JSON object",
        "protocol_id_nonempty": "cross-method Table 1 protocol id is absent",
        "protocol_schema_version_present": "cross-method Table 1 protocol schema_version is absent",
        "protocol_frozen_at_utc": "cross-method Table 1 protocol frozen_at_utc is absent",
        "protocol_frozen_before_first_run": "protocol is not marked frozen before the first run",
        "protocol_manifest_sha256_bound": "protocol manifest SHA256 binding failed",
        "protocol_hidden_specs_sha256_frozen": "protocol hidden-spec SHA256 is not frozen",
        "protocol_hidden_specs_withheld": "protocol does not withhold hidden specs from authors",
        "protocol_expected_task_count_54": "protocol does not bind expected_task_count=54",
        "protocol_independent_runs_per_task_3": "protocol does not bind three runs per task",
        "protocol_repeat_ids_match_manifest": "protocol repeat_ids do not match the manifest",
        "protocol_has_articraft_method": "protocol has no Articraft method arm",
        "protocol_method_ids_exact": "protocol method IDs are not exactly pva, lam, articraft",
        "openai_gpt5_exact_binding": "protocol does not bind provider=openai and model_id=gpt-5",
        "articraft_roles_match_openai_gpt5_binding": (
            "Articraft roles do not inherit or explicitly bind openai/gpt-5"
        ),
        "sampling_configuration_frozen": "common sampling configuration is not frozen",
        "repair_budget_frozen": "common maximum repair turns are not frozen",
        "first_shot_attempt_boundary_frozen": "first-shot attempt boundary/telemetry adapter is not frozen",
        "common_evaluator_frozen": "common executable/artifact/QC evaluator is not frozen",
        "method_output_isolation_frozen": "method-specific output isolation is not frozen",
        "telemetry_schema_frozen": "common attempt telemetry schema is not frozen",
        "timeouts_frozen": "common method timeouts are not frozen",
        "execution_readiness_declared": "protocol execution_readiness is not declared",
        "articraft_checkout_available": "Articraft checkout/git provenance is unavailable",
        "articraft_runtime_imports": "Articraft runtime package import smoke failed",
        "articraft_generate_cli_available": "Articraft generate CLI preflight failed",
        "openai_provider_configuration_present": "OpenAI provider credential is not configured",
        "common_evaluator_adapter_available": "Articraft common evaluator adapter is not implemented",
        "common_repair_adapter_available": "Articraft common repair adapter is not implemented",
        "run_id_output_isolation": "Articraft run-id output isolation is not implemented",
    }
    blockers = [blocker_labels[key] for key, value in hard_gates.items() if not value]
    environment_blockers = [
        blocker_labels[key]
        for key, value in {**contract_gates, **runtime_gates}.items()
        if not value
    ]
    adapter_blockers = [
        blocker_labels[key] for key, value in adapter_gates.items() if not value
    ]
    status = "BLOCKED" if environment_blockers else "ADAPTER_REQUIRED"

    manifest = {
        "schema_version": 1,
        "manifest_id": "nano3d_table1_articraft_reliability_preflight_v1",
        "manifest_kind": "fail_closed_preflight",
        "method_id": "articraft",
        "evaluation_unit": "common text/image item -> SDK program + articulated URDF",
        "intended_protocol": {
            "tasks": EXPECTED_TASKS,
            "independent_repeats_per_task": EXPECTED_REPEATS,
            "intended_generation_runs": EXPECTED_RUNS,
            "metrics": [
                "executable",
                "artifact_saved",
                "first_shot",
                "final_success",
                "repair_turns",
                "wall_time",
                "tokens",
                "api_cost",
            ],
            "paper_manual_retention_is_not_a_metric_substitute": True,
        },
        "common_protocol": protocol,
        "common_manifest": common,
        "execution_gates": adapter_gates,
        "generation_run_manifest": [],
        "actual_generation_runs": 0,
        "network_accessed": False,
        "paid_provider_calls": 0,
    }

    provider_presence_by_name = {
        provider: any(env_presence.get(name, False) for name in names)
        for provider, names in PROVIDER_ENV_NAMES.items()
    }
    summary = {
        "schema_version": 1,
        "protocol_id": manifest["manifest_id"],
        "generated_at_utc": utc_now(),
        "status": status,
        "evidence_class": "PREFLIGHT_ONLY",
        "claim_boundary": (
            "No common-prompt Articraft authoring rerun was executed. Local Table 1 "
            "success metrics are N/R; the paper manual-retention statistic and local "
            "non-common experiments are excluded."
        ),
        "hard_gates": hard_gates,
        "contract_gates": contract_gates,
        "runtime_gates": runtime_gates,
        "adapter_gates": adapter_gates,
        "common_protocol": protocol,
        "common_manifest": common,
        "local_rerun": {
            "attempted": False,
            "actual_generation_runs": 0,
            "intended_generation_runs": EXPECTED_RUNS,
            "metric_denominator": 0,
            "display_value": "N/R",
            "metrics": {
                "executable": None,
                "artifact_saved": None,
                "first_shot": None,
                "final_success": None,
                "repair_turns": None,
                "wall_time_s": None,
                "input_tokens": None,
                "output_tokens": None,
                "api_cost_usd": None,
            },
        },
        "paper_context": {
            "manual_retention": {
                "numerator": PAPER_RETENTION_NUMERATOR,
                "denominator": PAPER_RETENTION_DENOMINATOR,
                "rate": PAPER_RETENTION_NUMERATOR / PAPER_RETENTION_DENOMINATOR,
                "rate_percent_1dp": round(
                    100 * PAPER_RETENTION_NUMERATOR / PAPER_RETENTION_DENOMINATOR, 1
                ),
                "evidence_class": "PAPER_REPORTED_MANUAL_RETENTION",
                "table_document_formula_present": retention_formula_present,
                "used_as_executable": False,
                "used_as_artifact_saved": False,
                "used_as_first_shot": False,
                "used_as_final_success": False,
            }
        },
        "local_readiness": {
            "checkout": checkout,
            "runtime": {
                "python_path": str(ARTICRAFT_PYTHON),
                "python_exists": ARTICRAFT_PYTHON.is_file(),
                "import_smoke_ok": import_smoke["ok"],
                "import_smoke_exit_code": import_smoke.get("exit_code"),
                "generate_cli_path": str(ARTICRAFT_CLI),
                "generate_cli_exists": ARTICRAFT_CLI.is_file(),
                "generate_cli_help_ok": cli_help["ok"],
                "generate_cli_help_exit_code": cli_help.get("exit_code"),
            },
            "sdk_and_agent_entrypoints": {
                "sdk_package": str(ARTICRAFT_ROOT / "sdk"),
                "agent_runner": str(ARTICRAFT_ROOT / "agent/runner.py"),
                "agent_runner_cli": str(ARTICRAFT_ROOT / "agent/runner_cli.py"),
                "cli_main": str(ARTICRAFT_ROOT / "cli/main.py"),
                "all_present": all(
                    path.exists()
                    for path in (
                        ARTICRAFT_ROOT / "sdk",
                        ARTICRAFT_ROOT / "agent/runner.py",
                        ARTICRAFT_ROOT / "agent/runner_cli.py",
                        ARTICRAFT_ROOT / "cli/main.py",
                    )
                ),
            },
            "dataset": dataset,
            "provider_configuration": {
                "env_file_present": (ARTICRAFT_ROOT / ".env").is_file(),
                "provider_presence_by_name": provider_presence_by_name,
                "configured_provider_names": configured_providers,
                "articraft_model_env_present": env_presence["ARTICRAFT_MODEL"],
                "articraft_thinking_level_env_present": env_presence[
                    "ARTICRAFT_THINKING_LEVEL"
                ],
                "credential_values_read_into_outputs": False,
                "credential_validation_network_call_made": False,
                "note": "Presence-only audit; provider secrets and values are never emitted.",
            },
        },
        "supplementary_official_release_audit": release_audit,
        "excluded_local_evidence": {"t2_formal_v1": excluded_t2},
        "blockers": blockers,
        "environment_blockers": environment_blockers,
        "adapter_blockers": adapter_blockers,
    }

    provenance = {
        "schema_version": 1,
        "generated_at_utc": summary["generated_at_utc"],
        "command": (
            "python exp/scripts/run_table1_articraft_reliability.py "
            "--protocol exp/reference/table1_reliability_protocol_v1.json "
            "--common-manifest exp/reference/table1_reliability_common_authoring_v1.json "
            "--output-dir exp/runtime/table1_reliability/articraft"
        ),
        "runner": str(Path(__file__).resolve()),
        "runner_sha256": sha256_file(Path(__file__)),
        "repository": repository,
        "articraft_checkout": checkout,
        "environment": {
            "python": sys.version.split()[0],
            "python_executable": sys.executable,
            "platform": platform.platform(),
        },
        "inputs": input_hashes(
            [
                table_definition,
                table_results,
                protocol_path,
                common_manifest_path,
                t2_protocol,
                t2_readme,
                ARTICRAFT_ROOT / "README.md",
                ARTICRAFT_ROOT / "pyproject.toml",
                ARTICRAFT_ROOT / "agent/runner_cli.py",
                ARTICRAFT_ROOT / "cli/main.py",
                t2_formal_summary,
                OFFICIAL_RELEASE_ROOT / "records_index.jsonl",
                OFFICIAL_RELEASE_ROOT / "records_manifest.jsonl",
            ]
        ),
        "network_accessed": False,
        "paid_provider_calls": 0,
        "secret_values_emitted": False,
    }

    checks_list = [
        {
            "id": "fail_closed_status_domain",
            "pass": summary["status"] in {"BLOCKED", "ADAPTER_REQUIRED"},
        },
        {
            "id": "status_classification_consistent",
            "pass": (
                summary["status"] == "BLOCKED" and bool(environment_blockers)
            )
            or (
                summary["status"] == "ADAPTER_REQUIRED"
                and not environment_blockers
                and bool(adapter_blockers)
            ),
        },
        {
            "id": "adapter_gates_fail_closed",
            "pass": bool(adapter_gates) and not any(adapter_gates.values()),
        },
        {
            "id": "no_generation_runs_claimed",
            "pass": summary["local_rerun"]["actual_generation_runs"] == 0,
        },
        {
            "id": "local_metric_denominator_zero",
            "pass": summary["local_rerun"]["metric_denominator"] == 0,
        },
        {
            "id": "all_local_metrics_null",
            "pass": all(
                value is None for value in summary["local_rerun"]["metrics"].values()
            ),
        },
        {
            "id": "manual_retention_formula_exact",
            "pass": abs(
                summary["paper_context"]["manual_retention"]["rate"]
                - PAPER_RETENTION_NUMERATOR / PAPER_RETENTION_DENOMINATOR
            )
            < 1e-15,
        },
        {
            "id": "manual_retention_not_relabelled",
            "pass": not any(
                summary["paper_context"]["manual_retention"][key]
                for key in (
                    "used_as_executable",
                    "used_as_artifact_saved",
                    "used_as_first_shot",
                    "used_as_final_success",
                )
            ),
        },
        {
            "id": "table_document_retention_formula_found",
            "pass": retention_formula_present,
        },
        {
            "id": "dataset_index_valid_jsonl",
            "pass": dataset["index_rows"] > 0 and dataset["invalid_json_rows"] == 0,
        },
        {"id": "checkout_provenance_available", "pass": checkout["git_available"]},
        {"id": "runtime_import_smoke_passed", "pass": import_smoke["ok"]},
        {"id": "generate_cli_help_passed", "pass": cli_help["ok"]},
        {
            "id": "unrelated_t2_not_pooled",
            "pass": excluded_t2.get("included_in_table1_articraft_metrics") is False,
        },
        {
            "id": "release_audit_not_pooled",
            "pass": release_audit["included_in_common_authoring_metrics"] is False,
        },
        {
            "id": "official_release_jsonl_valid",
            "pass": release_audit["records_index"]["invalid_json_rows"] == 0
            and release_audit["records_manifest"]["invalid_json_rows"] == 0,
        },
        {
            "id": "release_status_not_mapped_to_final_success",
            "pass": release_audit["run_status"]["table1_final_success_mapping"] is None,
        },
        {
            "id": "release_denominators_not_paper_denominator",
            "pass": not release_audit["paper_retention_denominator_matches_index"]
            and not release_audit["paper_retention_denominator_matches_manifest"],
        },
        {
            "id": "release_flags_match_active_revision_paths",
            "pass": all(
                count == 0
                for count in release_audit["verified_active_revision_artifacts"][
                    "declared_flag_path_mismatch_counts"
                ].values()
            ),
        },
        {
            "id": "no_network_or_paid_calls",
            "pass": not provenance["network_accessed"] and provenance["paid_provider_calls"] == 0,
        },
    ]
    checks = {
        "schema_version": 1,
        "generated_at_utc": summary["generated_at_utc"],
        "checks": checks_list,
        "passed": sum(bool(row["pass"]) for row in checks_list),
        "total": len(checks_list),
    }
    checks["all_passed"] = checks["passed"] == checks["total"]

    write_json(output_dir / "manifest.json", manifest)
    write_json(output_dir / "provenance.json", provenance)
    write_json(output_dir / "release_telemetry.json", release_audit)
    write_json(output_dir / "summary.json", summary)
    write_json(output_dir / "self_check.json", checks)
    write_text_atomic(output_dir / "report.md", report_markdown(summary, checks))

    owned_outputs = [
        output_dir / "manifest.json",
        output_dir / "provenance.json",
        output_dir / "release_telemetry.json",
        output_dir / "summary.json",
        output_dir / "self_check.json",
        output_dir / "report.md",
    ]
    rendered_outputs = "\n".join(
        f"{sha256_file(path)}  {path.name}" for path in owned_outputs
    ) + "\n"
    write_text_atomic(output_dir / "SHA256SUMS", rendered_outputs)

    secret_assignment_found = any(
        SENSITIVE_ASSIGNMENT_RE.search(path.read_text(encoding="utf-8"))
        for path in owned_outputs + [output_dir / "SHA256SUMS"]
    )
    if secret_assignment_found or not checks["all_passed"]:
        return 1
    if args.require_ready and status != "READY":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
