#!/usr/bin/env python3
"""Fail-closed preflight for the frozen Table 1 common authoring protocol.

The command performs local, read-only validation of the public manifest, the
evaluator-only hidden specifications, and the protocol that binds them.  It
never invokes a model, API, method adapter, or generated template.  Method
adapter readiness is reported separately and is deliberately excluded from the
``--require-ready`` frozen-consistency gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = REPO_ROOT / "exp/reference/table1_reliability_common_authoring_v1.json"
DEFAULT_PROTOCOL = REPO_ROOT / "exp/reference/table1_reliability_protocol_v1.json"
DEFAULT_HIDDEN_SPECS = REPO_ROOT / "exp/reference/table1_reliability_hidden_specs_v1.json"
DEFAULT_RESULT_SCHEMA = REPO_ROOT / "exp/reference/table1_authoring_result_schema_v1.json"
DEFAULT_PACKAGE_SCHEMA = REPO_ROOT / "exp/reference/table1_authoring_package_schema_v1.json"
DEFAULT_EVALUATOR = REPO_ROOT / "exp/scripts/evaluate_table1_authoring_common.py"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "exp/runtime/table1_reliability/common"

EXPECTED_TASKS = 54
EXPECTED_REPEAT_IDS = ["r0", "r1", "r2"]
EXPECTED_METHODS = ["pva", "lam", "articraft"]
EXPECTED_DIFFICULTIES = ["L1", "L2", "L3"]
EXPECTED_SOURCE_PROVENANCE = "benchmark-authored_from_category_and_mechanical_priors"
EXPECTED_COORDINATE_CONVENTION = {
    "up_axis": "+Z",
    "front_axis": "+Y",
    "right_axis": "+X",
    "axis_frame": "parent_link",
    "units": {"length": "m", "angle": "rad"},
}
EXPECTED_REFERENCE_PATHS = {
    "manifest": "exp/reference/table1_reliability_common_authoring_v1.json",
    "hidden_specs": "exp/reference/table1_reliability_hidden_specs_v1.json",
    "package_schema": "exp/reference/table1_authoring_package_schema_v1.json",
    "result_schema": "exp/reference/table1_authoring_result_schema_v1.json",
    "common_evaluator": "exp/scripts/evaluate_table1_authoring_common.py",
}
EXPECTED_COMMON_MODEL_BINDING = {
    "provider": "openai",
    "model": "gpt-5",
    "reasoning_effort": "high",
}
EXPECTED_PVA_NATIVE_SETTINGS = {
    "adapter_schema": "pva_chroot_read_isolation_v1",
    "codex_cli": {
        "path": (
            "/mnt/zsn/miniconda3/lib/node_modules/@openai/codex/node_modules/"
            "@openai/codex-linux-x64/vendor/x86_64-unknown-linux-musl/bin/codex"
        ),
        "version": "codex-cli 0.147.0",
        "subcommand": "exec",
        "json_events": True,
        "ephemeral": True,
        "ignore_user_config": True,
        "ignore_rules": True,
        "external_isolation": "exclusive_single_job_chroot_nonroot_uid_65534",
        "internal_sandbox": "bypassed_inside_external_chroot",
        "shell_environment_inherit": "none",
    },
    "provider": {
        "model_provider": "pva_openai_env",
        "base_url": "https://api.openai.com/v1",
        "env_key": "OPENAI_API_KEY",
        "wire_api": "responses",
        "requires_openai_auth": False,
    },
    "native_retry_limit_per_attempt": 2,
    "model_response_timeout_seconds": 1800,
    "common_repair_turns": 3,
}
EXPECTED_LAM_NATIVE_SETTINGS = {
    "python_executable": "exp/runtime/table1_reliability/lam_env_v1/.venv/bin/python",
    "expected_python_version": "Python 3.12.3",
    "harness_python_executable": "/mnt/zsn/miniconda3/bin/python",
    "expected_harness_python_version": "Python 3.13.2",
    "harness_packages": {"jsonschema": "4.26.0", "trimesh": "4.12.2"},
    "node_executable": "/usr/bin/node",
    "expected_node_version": "v18.19.1",
    "npm_executable": "/usr/bin/npm",
    "expected_npm_version": "9.2.0",
    "requirements_sha256": "dbb9014ef1e86f7a9d31fc00b05b41fb0797abd0ea1ad54e25e3f2a25750f01c",
    "package_lock_sha256": "9a15deac933f4ef067d6645890c534c13f06f4610629fda5faed1f7f059039f7",
    "all_roles_use_method_model": True,
    "num_executions": 1,
    "parallel_workers": 0,
    "base_agent_parse_max_attempts": 3,
    "shape_export_validation_max_retries": 3,
    "vlm_critic_enabled": True,
    "vlm_critic_max_iterations": 4,
    "articulation_feedback_enabled": True,
    "articulation_feedback_max_iterations": 3,
    "pointllm_critic_enabled": False,
    "feedback_fusion_enabled": False,
    "outer_native_retry_limit": 2,
    "config_source": "adapter_generated_per_attempt_no_credentials",
}
EXPECTED_ARTICRAFT_NATIVE_SETTINGS = {
    "max_turns": 100,
    "openai_transport": "http",
    "sdk_package": "sdk",
    "max_cost_usd": None,
    "cost_policy": "formal_global_budget_only",
    "python_executable": "articraft_data/.venv/bin/python",
    "python_version": "3.12.3",
    "pyproject_sha256": "fd2cf4ddff0d8aaac5052bbfcadf09114cd70f3a1e9c9318936af22ef6c526be",
    "uv_lock_sha256": "b58b12834c30a894ce4d7fdf6ae41e0fc2947fb3a10ff6acf653344223b9a0fc",
}
EXPECTED_NATIVE_SETTINGS = {
    "pva": EXPECTED_PVA_NATIVE_SETTINGS,
    "lam": EXPECTED_LAM_NATIVE_SETTINGS,
    "articraft": EXPECTED_ARTICRAFT_NATIVE_SETTINGS,
}
EXPECTED_REQUEST_PARAMETERS = {
    "pva": {
        "reasoning_effort": {
            "value": "high",
            "configured": True,
            "transport": "cli_config",
        },
        "temperature": {
            "sent": False,
            "value": None,
            "reason": "Codex CLI exposes no temperature field for this adapter",
        },
        "top_p": {
            "sent": False,
            "value": None,
            "reason": "Codex CLI exposes no top_p field for this adapter",
        },
        "max_output_tokens": {
            "sent": False,
            "value": None,
            "reason": "Codex CLI exposes no output-token cap for this adapter",
        },
    },
    "lam": {
        "temperature": {
            "sent": False,
            "value": None,
            "reason": "LAM GPT-5 Responses handler omits temperature",
        },
        "top_p": {
            "sent": False,
            "value": None,
            "reason": "LAM GPT-5 Responses handler omits top_p",
        },
        "max_output_tokens": {"sent": True, "value": 64000},
        "verbosity": {"sent": True, "value": "medium"},
    },
    "articraft": {
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
DEFAULT_ADAPTER_PATHS = {
    method: REPO_ROOT / f"exp/scripts/run_table1_{method}_authoring.py"
    for method in EXPECTED_METHODS
}
THREAD_ENV = {
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}
HEX64 = re.compile(r"^[0-9a-f]{64}$")
GIT_OBJECT_ID = re.compile(r"^[0-9a-f]{40}$")
UTC_TIMESTAMP = re.compile(r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z$")
IMPLEMENTATION_KEYS = {
    "checkout_path",
    "commit",
    "git_tree",
    "entrypoint",
    "tracked_clean_at_freeze",
    "provenance",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def contained(path: Path, *, must_exist: bool = False) -> Path:
    resolved = path.expanduser().resolve(strict=must_exist)
    if resolved != REPO_ROOT and REPO_ROOT not in resolved.parents:
        raise ValueError(f"path escapes repository root: {resolved}")
    return resolved


def rel(path: Path) -> str:
    return contained(path).relative_to(REPO_ROOT).as_posix()


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path, must_exist=True).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def is_sha256(value: Any) -> bool:
    return isinstance(value, str) and HEX64.fullmatch(value) is not None


def is_git_object_id(value: Any) -> bool:
    return isinstance(value, str) and GIT_OBJECT_ID.fullmatch(value) is not None


def is_utc_timestamp(value: Any) -> bool:
    if not isinstance(value, str) or UTC_TIMESTAMP.fullmatch(value) is None:
        return False
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError:
        return False
    return parsed.utcoffset() == timezone.utc.utcoffset(parsed)


def load_json_object(path: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    path = contained(path)
    metadata: dict[str, Any] = {
        "path": rel(path),
        "exists": path.is_file(),
        "regular_file": path.is_file() and not path.is_symlink(),
        "size_bytes": path.stat().st_size if path.is_file() else None,
        "sha256": None,
        "valid_json_object": False,
        "error": None,
    }
    if not metadata["regular_file"]:
        metadata["error"] = "missing, non-regular, or symbolic-link input"
        return None, metadata
    try:
        metadata["sha256"] = sha256_file(path)
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        metadata["error"] = f"{type(exc).__name__}: {exc}"
        return None, metadata
    if not isinstance(payload, dict):
        metadata["error"] = "top-level JSON value is not an object"
        return None, metadata
    metadata["valid_json_object"] = True
    return payload, metadata


def write_text_atomic(path: Path, value: str) -> None:
    path = contained(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + f".tmp.{os.getpid()}")
    temporary.write_text(value, encoding="utf-8")
    temporary.replace(path)


def write_json(path: Path, value: Any) -> None:
    write_text_atomic(
        path,
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    )


def add_check(
    checks: list[dict[str, Any]],
    check_id: str,
    passed: bool,
    detail: str,
    *,
    scope: str = "frozen_consistency",
) -> bool:
    checks.append(
        {
            "check_id": check_id,
            "detail": detail,
            "passed": bool(passed),
            "scope": scope,
        }
    )
    return bool(passed)


def first_items(values: list[str], limit: int = 8) -> str:
    if not values:
        return "none"
    suffix = "" if len(values) <= limit else f" (+{len(values) - limit} more)"
    return ", ".join(values[:limit]) + suffix


def normalized_key(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", value.lower()).strip("_")


def declared_repo_path(value: Any) -> tuple[Path | None, str | None]:
    if not isinstance(value, str) or not value or Path(value).is_absolute():
        return None, "path must be a nonempty repository-relative string"
    try:
        resolved = contained(REPO_ROOT / value)
    except (OSError, ValueError) as exc:
        return None, str(exc)
    if rel(resolved) != value:
        return None, "path must be canonical and resolve to the declared repository-relative path"
    return resolved, None


def git_read(checkout: Path, arguments: list[str]) -> tuple[str | None, str | None]:
    try:
        completed = subprocess.run(
            ["git", "-C", str(checkout), *arguments],
            env={**os.environ, **THREAD_ENV, "GIT_OPTIONAL_LOCKS": "0"},
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, f"{type(exc).__name__}: {exc}"
    if completed.returncode != 0:
        detail = completed.stderr.strip() or f"git exited {completed.returncode}"
        return None, detail
    return completed.stdout.strip(), None


def manifest_hidden_leaks(payload: Any, prefix: str = "") -> list[str]:
    """Return public-manifest fields that could expose evaluator-only content."""

    leaks: list[str] = []
    if isinstance(payload, dict):
        for raw_key, value in payload.items():
            key = str(raw_key)
            path = f"{prefix}.{key}" if prefix else key
            normalized = normalized_key(key)
            if normalized != "hidden_spec_sha256":
                hidden_material = "hidden" in normalized and any(
                    token in normalized for token in ("path", "body", "content", "text", "spec")
                )
                explicit_spec_material = normalized in {
                    "spec",
                    "spec_body",
                    "spec_content",
                    "evaluator_spec",
                    "evaluator_only_path",
                }
                if hidden_material or explicit_spec_material:
                    leaks.append(path)
            leaks.extend(manifest_hidden_leaks(value, path))
    elif isinstance(payload, list):
        for index, value in enumerate(payload):
            leaks.extend(manifest_hidden_leaks(value, f"{prefix}[{index}]"))
    return leaks


def audit_manifest(
    payload: dict[str, Any] | None,
    metadata: dict[str, Any],
    checks: list[dict[str, Any]],
) -> dict[str, Any]:
    add_check(
        checks,
        "manifest.regular_json_object",
        payload is not None,
        metadata["error"] or "regular JSON object loaded",
    )
    result: dict[str, Any] = {
        "manifest_id": None,
        "domains": [],
        "task_ids": [],
        "task_count": 0,
        "public_hidden_hashes": {},
        "domain_counts": {},
        "difficulty_counts": {},
        "domain_difficulty_counts": {},
    }
    if payload is None:
        return result

    result["manifest_id"] = payload.get("manifest_id")
    add_check(checks, "manifest.schema_version", payload.get("schema_version") == 1, "schema_version must equal 1")
    add_check(
        checks,
        "manifest.manifest_id",
        isinstance(payload.get("manifest_id"), str) and bool(payload["manifest_id"].strip()),
        "manifest_id must be a nonempty string",
    )
    add_check(checks, "manifest.frozen", payload.get("frozen") is True, "frozen must be true")
    add_check(
        checks,
        "manifest.frozen_at_utc",
        is_utc_timestamp(payload.get("frozen_at_utc")),
        "frozen_at_utc must be an RFC 3339 UTC timestamp ending in Z",
    )
    add_check(
        checks,
        "manifest.task_count_declared",
        payload.get("task_count") == EXPECTED_TASKS,
        f"task_count must equal {EXPECTED_TASKS}",
    )
    add_check(
        checks,
        "manifest.repeat_ids",
        payload.get("repeat_ids") == EXPECTED_REPEAT_IDS,
        f"repeat_ids must equal {EXPECTED_REPEAT_IDS}",
    )

    raw_domains = payload.get("domains")
    domains: list[str] = []
    domain_records_valid = False
    if isinstance(raw_domains, list) and len(raw_domains) == 6:
        if all(isinstance(value, str) and value.strip() for value in raw_domains):
            domains = list(raw_domains)
            domain_records_valid = True
        elif all(isinstance(value, dict) for value in raw_domains):
            domain_records_valid = all(
                isinstance(value.get("domain"), str)
                and bool(value["domain"].strip())
                and value.get("task_count") == 9
                and isinstance(value.get("task_ids"), list)
                and len(value["task_ids"]) == 9
                and all(isinstance(task_id, str) and task_id.strip() for task_id in value["task_ids"])
                and len(set(value["task_ids"])) == 9
                for value in raw_domains
            )
            domains = [value["domain"] for value in raw_domains if isinstance(value.get("domain"), str)]
    domains_valid = domain_records_valid and len(domains) == 6 and len(set(domains)) == 6
    add_check(
        checks,
        "manifest.six_unique_domains",
        domains_valid,
        "domains must contain exactly six unique nonempty strings",
    )
    if domains_valid:
        result["domains"] = list(domains)

    tasks = payload.get("tasks")
    tasks_valid = isinstance(tasks, list) and len(tasks) == EXPECTED_TASKS
    add_check(
        checks,
        "manifest.tasks_54",
        tasks_valid,
        f"tasks must contain exactly {EXPECTED_TASKS} records",
    )
    if not isinstance(tasks, list):
        tasks = []

    required = {
        "task_id",
        "domain",
        "difficulty",
        "category",
        "input_modality",
        "prompt",
        "prompt_sha256",
        "hidden_spec_sha256",
        "contamination_flags",
        "source_provenance",
        "local_output_independent",
    }
    task_ids: list[str] = []
    public_hidden_hashes: dict[str, str] = {}
    invalid_shapes: list[str] = []
    invalid_domains: list[str] = []
    invalid_difficulties: list[str] = []
    invalid_categories: list[str] = []
    invalid_modalities: list[str] = []
    invalid_prompt_hashes: list[str] = []
    invalid_hidden_hashes: list[str] = []
    invalid_contamination: list[str] = []
    invalid_provenance: list[str] = []
    invalid_independence: list[str] = []
    domain_counts: Counter[str] = Counter()
    difficulty_counts: Counter[str] = Counter()
    cell_counts: Counter[tuple[str, str]] = Counter()
    for index, row in enumerate(tasks):
        fallback = f"index:{index}"
        if not isinstance(row, dict):
            invalid_shapes.append(fallback)
            continue
        raw_task_id = row.get("task_id")
        task_id = raw_task_id.strip() if isinstance(raw_task_id, str) else ""
        label = task_id or fallback
        if not required.issubset(row) or not task_id:
            invalid_shapes.append(label)
        if task_id:
            task_ids.append(task_id)
        domain = row.get("domain")
        if not isinstance(domain, str) or domain not in result["domains"]:
            invalid_domains.append(label)
        else:
            domain_counts[domain] += 1
        difficulty = row.get("difficulty")
        if difficulty not in EXPECTED_DIFFICULTIES:
            invalid_difficulties.append(label)
        else:
            difficulty_counts[difficulty] += 1
        if isinstance(domain, str) and domain in result["domains"] and difficulty in EXPECTED_DIFFICULTIES:
            cell_counts[(domain, difficulty)] += 1
        if not isinstance(row.get("category"), str) or not row["category"].strip():
            invalid_categories.append(label)
        if row.get("input_modality") != "text":
            invalid_modalities.append(label)
        prompt = row.get("prompt")
        prompt_hash = row.get("prompt_sha256")
        if (
            not isinstance(prompt, str)
            or not prompt.strip()
            or not is_sha256(prompt_hash)
            or sha256_text(prompt) != prompt_hash
        ):
            invalid_prompt_hashes.append(label)
        hidden_hash = row.get("hidden_spec_sha256")
        if not is_sha256(hidden_hash):
            invalid_hidden_hashes.append(label)
        elif task_id:
            public_hidden_hashes[task_id] = hidden_hash
        contamination = row.get("contamination_flags")
        if contamination != []:
            invalid_contamination.append(label)
        if row.get("source_provenance") != EXPECTED_SOURCE_PROVENANCE:
            invalid_provenance.append(label)
        if row.get("local_output_independent") is not True:
            invalid_independence.append(label)

    duplicate_ids = sorted(task_id for task_id, count in Counter(task_ids).items() if count != 1)
    add_check(
        checks,
        "manifest.task_shapes",
        not invalid_shapes and len(task_ids) == EXPECTED_TASKS,
        f"invalid task records: {first_items(invalid_shapes)}",
    )
    add_check(
        checks,
        "manifest.task_ids_unique",
        len(task_ids) == EXPECTED_TASKS and not duplicate_ids,
        f"duplicate task_ids: {first_items(duplicate_ids)}",
    )
    add_check(checks, "manifest.task_domains", not invalid_domains, f"invalid domain tasks: {first_items(invalid_domains)}")
    add_check(
        checks,
        "manifest.task_difficulties",
        not invalid_difficulties,
        f"invalid difficulty tasks: {first_items(invalid_difficulties)}",
    )
    add_check(checks, "manifest.task_categories", not invalid_categories, f"invalid category tasks: {first_items(invalid_categories)}")
    add_check(checks, "manifest.text_only", not invalid_modalities, f"non-text tasks: {first_items(invalid_modalities)}")
    add_check(
        checks,
        "manifest.prompt_hashes",
        not invalid_prompt_hashes,
        f"empty/mismatched prompt hashes: {first_items(invalid_prompt_hashes)}",
    )
    add_check(
        checks,
        "manifest.hidden_hash_fields",
        not invalid_hidden_hashes,
        f"invalid hidden_spec_sha256 tasks: {first_items(invalid_hidden_hashes)}",
    )
    add_check(
        checks,
        "manifest.contamination_flags",
        not invalid_contamination,
        f"contamination_flags must be empty arrays: {first_items(invalid_contamination)}",
    )
    add_check(
        checks,
        "manifest.source_provenance",
        not invalid_provenance,
        f"unexpected source_provenance tasks: {first_items(invalid_provenance)}",
    )
    add_check(
        checks,
        "manifest.local_output_independent",
        not invalid_independence,
        f"tasks without local_output_independent=true: {first_items(invalid_independence)}",
    )
    expected_cells = {
        (domain, difficulty): 3
        for domain in result["domains"]
        for difficulty in EXPECTED_DIFFICULTIES
    }
    bad_cells = sorted(
        f"{domain}/{difficulty}={cell_counts.get((domain, difficulty), 0)}"
        for domain, difficulty in expected_cells
        if cell_counts.get((domain, difficulty), 0) != 3
    )
    add_check(
        checks,
        "manifest.balanced_domain_difficulty_grid",
        domains_valid and not bad_cells,
        f"each domain/difficulty cell must have three tasks; bad cells: {first_items(bad_cells)}",
    )
    domain_membership_ok = True
    if isinstance(raw_domains, list) and raw_domains and isinstance(raw_domains[0], dict):
        task_ids_by_domain: dict[str, list[str]] = {}
        for task in tasks:
            if not isinstance(task, dict) or not isinstance(task.get("domain"), str) or not isinstance(task.get("task_id"), str):
                continue
            task_ids_by_domain.setdefault(task["domain"], []).append(task["task_id"])
        domain_membership_ok = all(
            isinstance(row, dict)
            and sorted(row.get("task_ids", [])) == sorted(task_ids_by_domain.get(str(row.get("domain", "")), []))
            for row in raw_domains
        )
    add_check(
        checks,
        "manifest.domain_records_match_tasks",
        domains_valid and domain_membership_ok,
        "declared domain task IDs must exactly match task records",
    )
    leaks = sorted(set(manifest_hidden_leaks(payload)))
    add_check(
        checks,
        "manifest.no_hidden_material",
        not leaks,
        f"forbidden evaluator-only public fields: {first_items(leaks)}",
    )

    result.update(
        {
            "task_ids": task_ids,
            "task_count": len(tasks),
            "public_hidden_hashes": public_hidden_hashes,
            "domain_counts": dict(sorted(domain_counts.items())),
            "difficulty_counts": dict(sorted(difficulty_counts.items())),
            "domain_difficulty_counts": {
                f"{domain}/{difficulty}": cell_counts[(domain, difficulty)]
                for domain, difficulty in sorted(cell_counts)
            },
        }
    )
    return result


def valid_role_rows(value: Any) -> bool:
    if not isinstance(value, list) or not value:
        return False
    for row in value:
        if not isinstance(row, dict):
            return False
        if not isinstance(row.get("role_id"), str) or not row["role_id"].strip():
            return False
        aliases = row.get("aliases")
        if not isinstance(aliases, list) or not aliases or not all(
            isinstance(alias, str) and alias.strip() for alias in aliases
        ):
            return False
        if not isinstance(row.get("min_count"), int) or isinstance(row.get("min_count"), bool) or row["min_count"] < 1:
            return False
    return True


def valid_joint_rows(value: Any) -> bool:
    if not isinstance(value, list) or not value:
        return False
    for row in value:
        if not isinstance(row, dict):
            return False
        for key in ("joint_id", "parent_role", "child_role", "type"):
            if not isinstance(row.get(key), str) or not row[key].strip():
                return False
        axis = row.get("axis")
        if not isinstance(axis, list) or len(axis) != 3 or not all(
            isinstance(item, (int, float)) and not isinstance(item, bool) for item in axis
        ):
            return False
        for key in ("lower", "upper"):
            if row.get(key) is not None and (
                not isinstance(row[key], (int, float)) or isinstance(row[key], bool)
            ):
                return False
        if "mimic" in row and not isinstance(row["mimic"], (dict, bool, type(None))):
            return False
    return True


def valid_constraint_rows(value: Any) -> bool:
    if not isinstance(value, list):
        return False
    for row in value:
        if not isinstance(row, dict):
            return False
        if not isinstance(row.get("constraint_id"), str) or not row["constraint_id"].strip():
            return False
        if not isinstance(row.get("description"), str) or not row["description"].strip():
            return False
        if row.get("table1_gate") is not False:
            return False
        if not isinstance(row.get("reserved_axis"), str) or not row["reserved_axis"].strip():
            return False
    return True


def audit_hidden_specs(
    payload: dict[str, Any] | None,
    metadata: dict[str, Any],
    public: dict[str, Any],
    checks: list[dict[str, Any]],
) -> dict[str, Any]:
    add_check(
        checks,
        "hidden.regular_json_object",
        payload is not None,
        metadata["error"] or "regular evaluator-only JSON object loaded",
    )
    result: dict[str, Any] = {
        "hidden_specs_id": None,
        "spec_count": 0,
        "task_ids_match_public": False,
        "canonical_hash_match_count": 0,
        "public_hash_match_count": 0,
    }
    if payload is None:
        return result

    result["hidden_specs_id"] = payload.get("hidden_specs_id")
    add_check(checks, "hidden.schema_version", payload.get("schema_version") == 1, "schema_version must equal 1")
    add_check(
        checks,
        "hidden.hidden_specs_id",
        payload.get("hidden_specs_id") == "table1_reliability_hidden_specs_v1",
        "hidden_specs_id must equal table1_reliability_hidden_specs_v1",
    )
    add_check(checks, "hidden.frozen", payload.get("frozen") is True, "frozen must be true")
    add_check(
        checks,
        "hidden.frozen_at_utc",
        is_utc_timestamp(payload.get("frozen_at_utc")),
        "frozen_at_utc must be an RFC 3339 UTC timestamp ending in Z",
    )
    add_check(
        checks,
        "hidden.task_count_declared",
        payload.get("task_count") == EXPECTED_TASKS,
        f"task_count must equal {EXPECTED_TASKS}",
    )
    add_check(
        checks,
        "hidden.coordinate_convention",
        payload.get("coordinate_convention") == EXPECTED_COORDINATE_CONVENTION,
        "coordinate_convention must match the frozen +Z/+Y/+X parent-link SI convention",
    )
    add_check(
        checks,
        "hidden.canonical_hash_rule",
        isinstance(payload.get("canonical_hash_rule"), str)
        and all(
            term in payload["canonical_hash_rule"].lower()
            for term in ("sort_keys", "separators", "ensure_ascii")
        ),
        "canonical_hash_rule must explicitly name sort_keys, separators, and ensure_ascii",
    )
    specs = payload.get("specs")
    add_check(
        checks,
        "hidden.specs_54",
        isinstance(specs, list) and len(specs) == EXPECTED_TASKS,
        f"specs must contain exactly {EXPECTED_TASKS} records",
    )
    if not isinstance(specs, list):
        specs = []

    task_ids: list[str] = []
    invalid_shapes: list[str] = []
    invalid_semantics: list[str] = []
    invalid_declared_hashes: list[str] = []
    public_hash_mismatches: list[str] = []
    canonical_match_count = 0
    public_match_count = 0
    required = {
        "task_id",
        "coordinate_convention",
        "difficulty_rationale",
        "required_parts",
        "required_joints",
        "measurable_constraints",
        "hidden_spec_sha256",
    }
    for index, row in enumerate(specs):
        fallback = f"index:{index}"
        if not isinstance(row, dict):
            invalid_shapes.append(fallback)
            continue
        raw_task_id = row.get("task_id")
        task_id = raw_task_id.strip() if isinstance(raw_task_id, str) else ""
        label = task_id or fallback
        if not required.issubset(row) or not task_id:
            invalid_shapes.append(label)
        if task_id:
            task_ids.append(task_id)
        semantic_ok = (
            row.get("coordinate_convention") == EXPECTED_COORDINATE_CONVENTION
            and isinstance(row.get("difficulty_rationale"), str)
            and bool(row["difficulty_rationale"].strip())
            and valid_role_rows(row.get("required_parts"))
            and valid_joint_rows(row.get("required_joints"))
            and valid_constraint_rows(row.get("measurable_constraints"))
        )
        if not semantic_ok:
            invalid_semantics.append(label)
        declared_hash = row.get("hidden_spec_sha256")
        value_for_hash = {key: value for key, value in row.items() if key != "hidden_spec_sha256"}
        computed_hash = hashlib.sha256(canonical_json_bytes(value_for_hash)).hexdigest()
        if not is_sha256(declared_hash) or declared_hash != computed_hash:
            invalid_declared_hashes.append(label)
        else:
            canonical_match_count += 1
        public_hash = public["public_hidden_hashes"].get(task_id)
        if public_hash != computed_hash or declared_hash != computed_hash:
            public_hash_mismatches.append(label)
        else:
            public_match_count += 1

    duplicates = sorted(task_id for task_id, count in Counter(task_ids).items() if count != 1)
    add_check(
        checks,
        "hidden.spec_shapes",
        not invalid_shapes and len(task_ids) == EXPECTED_TASKS,
        f"invalid hidden spec records: {first_items(invalid_shapes)}",
    )
    add_check(
        checks,
        "hidden.spec_semantics",
        not invalid_semantics,
        f"invalid hidden spec structures: {first_items(invalid_semantics)}",
    )
    add_check(
        checks,
        "hidden.task_ids_unique",
        len(task_ids) == EXPECTED_TASKS and not duplicates,
        f"duplicate hidden task_ids: {first_items(duplicates)}",
    )
    public_ids = public["task_ids"]
    ids_match = (
        len(task_ids) == EXPECTED_TASKS
        and len(public_ids) == EXPECTED_TASKS
        and task_ids == public_ids
    )
    add_check(
        checks,
        "hidden.task_ids_match_public_order",
        ids_match,
        "hidden specs task IDs and order must exactly match the public manifest",
    )
    add_check(
        checks,
        "hidden.canonical_item_hashes",
        not invalid_declared_hashes and len(specs) == EXPECTED_TASKS,
        f"canonical hash mismatches: {first_items(invalid_declared_hashes)}",
    )
    add_check(
        checks,
        "hidden.public_item_hash_bindings",
        not public_hash_mismatches and len(specs) == EXPECTED_TASKS,
        f"public/private hidden hash mismatches: {first_items(public_hash_mismatches)}",
    )
    result.update(
        {
            "spec_count": len(specs),
            "task_ids_match_public": ids_match,
            "canonical_hash_match_count": canonical_match_count,
            "public_hash_match_count": public_match_count,
        }
    )
    return result


def binding_values(section: Any) -> tuple[Any, Any]:
    if not isinstance(section, dict):
        return None, None
    return section.get("path", section.get("entrypoint")), section.get(
        "sha256", section.get("expected_sha256")
    )


def audit_file_binding(
    checks: list[dict[str, Any]],
    check_prefix: str,
    section: Any,
    expected_path: str,
    actual_sha256: str | None,
) -> None:
    bound_path, bound_hash = binding_values(section)
    add_check(
        checks,
        f"protocol.{check_prefix}_path",
        bound_path == expected_path,
        f"{check_prefix} path must equal {expected_path}",
    )
    add_check(
        checks,
        f"protocol.{check_prefix}_sha256",
        actual_sha256 is not None and bound_hash == actual_sha256,
        f"{check_prefix} SHA-256 must match current frozen input bytes",
    )


def extract_method_rows(value: Any) -> dict[str, dict[str, Any]]:
    rows: dict[str, dict[str, Any]] = {}
    if isinstance(value, dict):
        for method, row in value.items():
            if isinstance(method, str) and isinstance(row, dict):
                rows[method.strip().lower()] = row
    elif isinstance(value, list):
        for row in value:
            if isinstance(row, str):
                rows[row.strip().lower()] = {}
            elif isinstance(row, dict) and isinstance(row.get("method_id"), str):
                rows[row["method_id"].strip().lower()] = row
    return rows


def values_are_nonempty_strings(mapping: Any, keys: set[str]) -> bool:
    return isinstance(mapping, dict) and all(
        isinstance(mapping.get(key), str) and bool(mapping[key].strip()) for key in keys
    )


def audit_method_implementations(
    methods: dict[str, dict[str, Any]], checks: list[dict[str, Any]]
) -> dict[str, dict[str, Any]]:
    records: dict[str, dict[str, Any]] = {}
    for method in EXPECTED_METHODS:
        row = methods.get(method, {})
        implementation = row.get("implementation")
        shape_ok = isinstance(implementation, dict) and IMPLEMENTATION_KEYS <= set(
            implementation
        )
        add_check(
            checks,
            f"protocol.method.{method}.implementation_shape",
            shape_ok,
            "implementation must define checkout_path, commit, git_tree, entrypoint, "
            "tracked_clean_at_freeze, and provenance",
        )
        implementation = implementation if isinstance(implementation, dict) else {}

        checkout, checkout_error = declared_repo_path(implementation.get("checkout_path"))
        checkout_exists = bool(
            checkout is not None
            and checkout.is_dir()
            and not checkout.is_symlink()
            and (checkout / ".git").exists()
        )
        add_check(
            checks,
            f"protocol.method.{method}.checkout_path",
            checkout_exists,
            checkout_error
            or "checkout_path must name a canonical repository-contained Git checkout",
        )

        head: str | None = None
        tree: str | None = None
        tracked_status: str | None = None
        git_root: str | None = None
        git_errors: dict[str, str] = {}
        if checkout_exists and checkout is not None:
            for name, arguments in (
                ("git_root", ["rev-parse", "--show-toplevel"]),
                ("head", ["rev-parse", "HEAD"]),
                ("tree", ["rev-parse", "HEAD^{tree}"]),
                ("tracked_status", ["status", "--porcelain=v1", "--untracked-files=no"]),
            ):
                value, error = git_read(checkout, arguments)
                if error is not None:
                    git_errors[name] = error
                elif name == "git_root":
                    git_root = value
                elif name == "head":
                    head = value
                elif name == "tree":
                    tree = value
                else:
                    tracked_status = value

        exact_git_root = False
        if checkout is not None and git_root:
            try:
                exact_git_root = Path(git_root).resolve(strict=True) == checkout
            except OSError:
                exact_git_root = False
        add_check(
            checks,
            f"protocol.method.{method}.checkout_git_root",
            exact_git_root,
            git_errors.get("git_root")
            or "checkout_path must equal the checkout's Git top-level directory",
        )

        declared_commit = implementation.get("commit")
        add_check(
            checks,
            f"protocol.method.{method}.commit",
            is_git_object_id(declared_commit) and head == declared_commit,
            git_errors.get("head") or f"declared commit must equal actual HEAD {head or 'N/A'}",
        )
        declared_tree = implementation.get("git_tree")
        add_check(
            checks,
            f"protocol.method.{method}.git_tree",
            is_git_object_id(declared_tree) and tree == declared_tree,
            git_errors.get("tree")
            or f"declared git_tree must equal actual HEAD tree {tree or 'N/A'}",
        )
        actual_clean = tracked_status == "" if tracked_status is not None else None
        declared_clean = implementation.get("tracked_clean_at_freeze")
        add_check(
            checks,
            f"protocol.method.{method}.tracked_clean_at_freeze",
            declared_clean is True and actual_clean is True,
            git_errors.get("tracked_status")
            or "tracked_clean_at_freeze must be true and tracked Git status must be clean",
        )
        provenance = implementation.get("provenance")
        add_check(
            checks,
            f"protocol.method.{method}.provenance",
            isinstance(provenance, str) and bool(provenance.strip()),
            "implementation provenance must be a nonempty string",
        )

        entrypoint, entrypoint_error = declared_repo_path(implementation.get("entrypoint"))
        entrypoint_in_checkout = bool(
            checkout is not None
            and entrypoint is not None
            and entrypoint != checkout
            and checkout in entrypoint.parents
        )
        entrypoint_regular = bool(
            entrypoint_in_checkout
            and entrypoint is not None
            and entrypoint.is_file()
            and not entrypoint.is_symlink()
        )
        add_check(
            checks,
            f"protocol.method.{method}.entrypoint",
            entrypoint_regular,
            entrypoint_error
            or "entrypoint must be a canonical regular file beneath the declared checkout",
        )
        entrypoint_tracked = False
        entrypoint_matches_head = False
        entrypoint_git_error: str | None = None
        if entrypoint_regular and checkout is not None and entrypoint is not None:
            checkout_relative = entrypoint.relative_to(checkout).as_posix()
            tracked_value, tracked_error = git_read(
                checkout, ["ls-files", "--error-unmatch", "--", checkout_relative]
            )
            entrypoint_tracked = tracked_error is None and tracked_value == checkout_relative
            diff_value, diff_error = git_read(
                checkout, ["diff", "--name-only", "HEAD", "--", checkout_relative]
            )
            entrypoint_matches_head = diff_error is None and diff_value == ""
            entrypoint_git_error = tracked_error or diff_error
        add_check(
            checks,
            f"protocol.method.{method}.entrypoint_tracked",
            entrypoint_tracked,
            entrypoint_git_error or "entrypoint must be tracked by the declared checkout",
        )
        add_check(
            checks,
            f"protocol.method.{method}.entrypoint_matches_head",
            entrypoint_matches_head,
            entrypoint_git_error or "entrypoint working bytes must match the frozen HEAD tree",
        )

        adapter, adapter_error = declared_repo_path(row.get("adapter_entrypoint"))
        expected_adapter = DEFAULT_ADAPTER_PATHS[method]
        adapter_regular = bool(
            adapter is not None
            and adapter == expected_adapter
            and adapter.is_file()
            and not adapter.is_symlink()
        )
        add_check(
            checks,
            f"protocol.method.{method}.adapter_entrypoint",
            adapter_regular,
            adapter_error
            or f"adapter_entrypoint must equal {rel(expected_adapter)} and be a regular file",
        )
        actual_adapter_sha256 = sha256_file(adapter) if adapter_regular and adapter else None
        add_check(
            checks,
            f"protocol.method.{method}.adapter_sha256",
            is_sha256(row.get("adapter_sha256"))
            and row.get("adapter_sha256") == actual_adapter_sha256,
            "adapter_sha256 must match current adapter bytes",
        )

        records[method] = {
            "checkout_path": rel(checkout) if checkout is not None else None,
            "checkout_exists": checkout_exists,
            "git_root": rel(checkout) if exact_git_root and checkout is not None else git_root,
            "actual_commit": head,
            "declared_commit": declared_commit,
            "actual_git_tree": tree,
            "declared_git_tree": declared_tree,
            "actual_tracked_clean": actual_clean,
            "declared_tracked_clean_at_freeze": declared_clean,
            "entrypoint": rel(entrypoint) if entrypoint is not None else None,
            "entrypoint_regular": entrypoint_regular,
            "entrypoint_tracked": entrypoint_tracked,
            "entrypoint_matches_head": entrypoint_matches_head,
            "provenance": provenance if isinstance(provenance, str) else None,
            "adapter_entrypoint": rel(adapter) if adapter is not None else None,
            "actual_adapter_sha256": actual_adapter_sha256,
            "declared_adapter_sha256": row.get("adapter_sha256"),
        }
    return records


def audit_protocol(
    payload: dict[str, Any] | None,
    metadata: dict[str, Any],
    input_metadata: dict[str, dict[str, Any]],
    checks: list[dict[str, Any]],
) -> dict[str, Any]:
    add_check(
        checks,
        "protocol.regular_json_object",
        payload is not None,
        metadata["error"] or "regular JSON object loaded",
    )
    result: dict[str, Any] = {
        "protocol_id": None,
        "method_ids": [],
        "declared_execution_ready": None,
        "declared_method_adapters_ready": {},
        "method_rows": {},
        "method_implementations": {},
    }
    if payload is None:
        return result

    result["protocol_id"] = payload.get("protocol_id")
    add_check(
        checks,
        "protocol.schema_version",
        payload.get("schema_version") in (1, "table1_reliability_protocol_v1"),
        "schema_version must identify protocol v1",
    )
    add_check(
        checks,
        "protocol.protocol_id",
        isinstance(payload.get("protocol_id"), str) and bool(payload["protocol_id"].strip()),
        "protocol_id must be a nonempty string",
    )
    frozen_design = payload.get("frozen_design", payload.get("frozen"))
    add_check(checks, "protocol.frozen_design", frozen_design is True, "frozen_design must be true")
    if "frozen_at_utc" in payload:
        add_check(
            checks,
            "protocol.frozen_at_utc",
            is_utc_timestamp(payload.get("frozen_at_utc")),
            "frozen_at_utc must be an RFC 3339 UTC timestamp ending in Z",
        )

    audit_file_binding(
        checks,
        "manifest",
        payload.get("manifest"),
        EXPECTED_REFERENCE_PATHS["manifest"],
        input_metadata["manifest"].get("sha256"),
    )
    hidden_binding = payload.get("hidden_specs")
    audit_file_binding(
        checks,
        "hidden_specs",
        hidden_binding,
        EXPECTED_REFERENCE_PATHS["hidden_specs"],
        input_metadata["hidden_specs"].get("sha256"),
    )
    add_check(
        checks,
        "protocol.hidden_specs_withheld",
        isinstance(hidden_binding, dict)
        and (
            hidden_binding.get("withheld_from_author") is True
            or hidden_binding.get("visibility") == "evaluator_only"
        ),
        "hidden_specs must declare withheld_from_author=true or visibility=evaluator_only",
    )
    audit_file_binding(
        checks,
        "common_evaluator",
        payload.get("common_evaluator"),
        EXPECTED_REFERENCE_PATHS["common_evaluator"],
        input_metadata["common_evaluator"].get("sha256"),
    )
    audit_file_binding(
        checks,
        "package_schema",
        payload.get("package_schema"),
        EXPECTED_REFERENCE_PATHS["package_schema"],
        input_metadata["package_schema"].get("sha256"),
    )
    result_schema = payload.get("result_schema", payload.get("attempt_record_schema"))
    audit_file_binding(
        checks,
        "result_schema",
        result_schema,
        EXPECTED_REFERENCE_PATHS["result_schema"],
        input_metadata["result_schema"].get("sha256"),
    )

    add_check(
        checks,
        "protocol.expected_task_count",
        payload.get("expected_task_count") == EXPECTED_TASKS,
        f"expected_task_count must equal {EXPECTED_TASKS}",
    )
    add_check(
        checks,
        "protocol.independent_runs_per_task",
        payload.get("independent_runs_per_task") == len(EXPECTED_REPEAT_IDS),
        "independent_runs_per_task must equal 3",
    )
    add_check(
        checks,
        "protocol.repeat_ids",
        payload.get("repeat_ids") == EXPECTED_REPEAT_IDS,
        f"repeat_ids must equal {EXPECTED_REPEAT_IDS}",
    )
    add_check(
        checks,
        "protocol.max_common_repair_turns",
        payload.get("max_common_repair_turns") == 3,
        "max_common_repair_turns must equal 3",
    )

    methods = extract_method_rows(payload.get("methods"))
    result["method_ids"] = sorted(methods)
    result["method_rows"] = methods
    add_check(
        checks,
        "protocol.method_set",
        sorted(methods) == sorted(EXPECTED_METHODS),
        f"methods must contain exactly {EXPECTED_METHODS}",
    )
    result["method_implementations"] = audit_method_implementations(methods, checks)
    common_binding = payload.get("common_model_binding")
    common_ok = common_binding == EXPECTED_COMMON_MODEL_BINDING
    method_model_ok = bool(methods) and all(
        row.get("provider") == "openai"
        and row.get("model", row.get("model_id")) == "gpt-5"
        and row.get("reasoning_effort", row.get("reasoning")) == "high"
        for row in methods.values()
    )
    add_check(
        checks,
        "protocol.common_model_binding",
        common_ok and method_model_ok,
        "common binding must contain only provider=openai, model=gpt-5, reasoning_effort=high and every method must match",
    )
    def request_parameter_is_frozen(
        row: dict[str, Any], key: str, *, positive_integer: bool = False
    ) -> bool:
        parameters = row.get("request_parameters")
        if not isinstance(parameters, dict):
            return False
        parameter = parameters.get(key)
        if not isinstance(parameter, dict) or not isinstance(parameter.get("sent"), bool):
            return False
        value = parameter.get("value")
        if parameter["sent"]:
            if value is None:
                return False
            if positive_integer:
                return isinstance(value, int) and not isinstance(value, bool) and value > 0
            return True
        return value is None and isinstance(parameter.get("reason"), str) and bool(
            parameter["reason"].strip()
        )

    method_config_ok = bool(methods) and all(
        isinstance(row.get("role"), str)
        and bool(row["role"].strip())
        and request_parameter_is_frozen(row, "temperature")
        and request_parameter_is_frozen(row, "top_p")
        and request_parameter_is_frozen(row, "max_output_tokens", positive_integer=True)
        for row in methods.values()
    )
    add_check(
        checks,
        "protocol.method_sampling_bindings",
        method_config_ok,
        "each method must freeze role and sent/value/reason records for temperature, top_p, and max_output_tokens",
    )
    add_check(
        checks,
        "protocol.request_parameters_honest",
        method_config_ok,
        "unsupported or unexposed request parameters must be frozen as sent=false, value=null, with a reason",
    )
    for method in EXPECTED_METHODS:
        add_check(
            checks,
            f"protocol.method.{method}.request_contract",
            methods.get(method, {}).get("request_parameters")
            == EXPECTED_REQUEST_PARAMETERS[method],
            f"{method} request_parameters must match the audited provider payload contract",
        )
        add_check(
            checks,
            f"protocol.method.{method}.native_contract",
            methods.get(method, {}).get("native_settings")
            == EXPECTED_NATIVE_SETTINGS[method],
            f"{method} native_settings must match the audited exact contract",
        )

    attempt_semantics = payload.get("attempt_semantics", payload.get("attempt_schema"))
    add_check(
        checks,
        "protocol.attempt_semantics",
        values_are_nonempty_strings(attempt_semantics, {"attempt_0", "common_repair", "native_retry"}),
        "attempt semantics must define attempt_0, common_repair, and native_retry",
    )
    definitions = payload.get("metric_definitions", payload.get("definitions"))
    add_check(
        checks,
        "protocol.metric_definitions",
        values_are_nonempty_strings(
            definitions,
            {"executable", "artifact_saved", "common_qc", "first_shot", "final_success", "repair_turns"},
        ),
        "metric definitions must define executable, artifact_saved, common_qc, first_shot, final_success, and repair_turns",
    )

    telemetry = payload.get("telemetry_requirements", payload.get("telemetry_fields"))
    per_attempt: list[str] = []
    telemetry_policy_ok = False
    if isinstance(telemetry, dict):
        raw_fields = telemetry.get("per_attempt", telemetry.get("fields", []))
        if isinstance(raw_fields, list):
            per_attempt = [value for value in raw_fields if isinstance(value, str)]
        telemetry_policy_ok = (
            isinstance(telemetry.get("missing_value_policy"), str)
            and bool(telemetry["missing_value_policy"].strip())
            and telemetry.get("credential_values_forbidden") is True
        )
    elif isinstance(telemetry, list):
        per_attempt = [value for value in telemetry if isinstance(value, str)]
    normalized_fields = {normalized_key(value) for value in per_attempt}
    telemetry_fields_ok = (
        any(value in normalized_fields for value in ("wall_time", "wall_time_s", "wall_time_seconds"))
        and "input_tokens" in normalized_fields
        and "output_tokens" in normalized_fields
        and any(value in normalized_fields for value in ("api_cost", "api_cost_usd"))
    )
    add_check(
        checks,
        "protocol.telemetry_fields",
        telemetry_fields_ok,
        "per-attempt telemetry must include wall time, input tokens, output tokens, and API cost",
    )
    add_check(
        checks,
        "protocol.telemetry_policy",
        telemetry_policy_ok,
        "telemetry must freeze a missing-value policy and forbid credential values",
    )

    timeouts = payload.get("timeouts", payload.get("timeouts_s"))
    timeout_ok = False
    if isinstance(timeouts, dict):
        timeout_ok = all(
            isinstance(timeouts.get(key), (int, float))
            and not isinstance(timeouts.get(key), bool)
            and timeouts[key] > 0
            for key in (
                "model_response_seconds",
                "execution_seconds_per_attempt",
                "common_evaluator_seconds_per_attempt",
            )
        ) and (
            isinstance(timeouts.get("native_retry_limit_per_attempt"), int)
            and not isinstance(timeouts.get("native_retry_limit_per_attempt"), bool)
            and timeouts["native_retry_limit_per_attempt"] >= 0
        )
    add_check(
        checks,
        "protocol.timeouts",
        timeout_ok,
        "timeouts must freeze three positive attempt timeouts and a nonnegative native retry limit",
    )

    evaluator = payload.get("common_evaluator")
    evaluator_contract_ok = (
        isinstance(evaluator, dict)
        and evaluator.get("exists_at_freeze") is True
        and evaluator.get("required_before_execution") is True
        and isinstance(evaluator.get("table1_gate_scope"), list)
        and bool(evaluator["table1_gate_scope"])
        and all(isinstance(value, str) and value.strip() for value in evaluator["table1_gate_scope"])
        and isinstance(evaluator.get("excluded_from_table1_gate"), str)
        and bool(evaluator["excluded_from_table1_gate"].strip())
    )
    add_check(
        checks,
        "protocol.evaluator_contract",
        evaluator_contract_ok,
        "common evaluator must exist at freeze and define required-before-execution, included gate scope, and exclusions",
    )

    output_isolation = payload.get("output_isolation", payload.get("method_output_isolation"))
    output_isolation_ok = (
        isinstance(output_isolation, dict)
        and isinstance(output_isolation.get("root_pattern"), str)
        and bool(output_isolation["root_pattern"].strip())
        and output_isolation.get("one_method_task_repeat_per_directory") is True
        and output_isolation.get("cross_method_read_access") is False
        and isinstance(output_isolation.get("preexisting_output_policy"), str)
        and bool(output_isolation["preexisting_output_policy"].strip())
    )
    add_check(
        checks,
        "protocol.output_isolation",
        output_isolation_ok,
        "output isolation must freeze a root pattern, one run directory, no cross-method reads, and a preexisting-output policy",
    )

    hidden_policy = payload.get("hidden_policy")
    hidden_policy_ok = isinstance(hidden_policy, dict) and any(
        hidden_policy.get(key) is True
        for key in ("withheld_from_author", "hidden_specs_withheld_from_author", "evaluator_only")
    )
    add_check(
        checks,
        "protocol.hidden_policy",
        hidden_policy_ok,
        "hidden_policy must explicitly keep hidden specs evaluator-only",
    )

    execution = payload.get("execution_readiness")
    declared_adapter_states: dict[str, Any] = {}
    if isinstance(execution, dict):
        declared_adapter_states = (
            execution.get("method_adapters_ready")
            if isinstance(execution.get("method_adapters_ready"), dict)
            else {}
        )
        result["declared_method_adapters_ready"] = declared_adapter_states
    execution_contract_ok = (
        isinstance(payload.get("execution_ready"), bool)
        and isinstance(execution, dict)
        and isinstance(execution.get("status"), str)
        and bool(execution["status"].strip())
        and execution.get("frozen_inputs_complete") is True
        and execution.get("evaluator_bound") is True
        and execution.get("package_schema_bound") is True
        and execution.get("result_schema_bound") is True
        and sorted(declared_adapter_states) == sorted(EXPECTED_METHODS)
        and all(isinstance(declared_adapter_states[method], bool) for method in EXPECTED_METHODS)
        and isinstance(execution.get("blockers"), list)
        and all(isinstance(value, str) and value.strip() for value in execution["blockers"])
    )
    add_check(
        checks,
        "protocol.execution_readiness_contract",
        execution_contract_ok,
        "execution readiness must structurally declare frozen bindings, all three adapter booleans, and blockers",
    )
    declared_execution_ready = payload.get("execution_ready")
    method_states_valid = (
        set(declared_adapter_states) == set(EXPECTED_METHODS)
        and all(
            isinstance(declared_adapter_states.get(method), bool)
            for method in EXPECTED_METHODS
        )
    )
    add_check(
        checks,
        "protocol.execution_ready_matches_method_readiness",
        isinstance(declared_execution_ready, bool)
        and method_states_valid
        and declared_execution_ready is all(declared_adapter_states.values()),
        "execution_ready must equal the conjunction of all method adapter readiness states",
    )
    execution_status = execution.get("status") if isinstance(execution, dict) else None
    add_check(
        checks,
        "protocol.execution_status_matches_execution_ready",
        isinstance(declared_execution_ready, bool)
        and isinstance(execution_status, str)
        and bool(execution_status.strip())
        and (
            execution_status == "READY"
            if declared_execution_ready
            else execution_status.startswith("BLOCKED")
        ),
        "execution status must be READY when execution_ready is true and BLOCKED* otherwise",
    )
    method_blockers = (
        execution.get("method_blockers") if isinstance(execution, dict) else None
    )
    method_blockers_valid = (
        isinstance(method_blockers, dict)
        and set(method_blockers) == set(EXPECTED_METHODS)
        and all(
            isinstance(method_blockers.get(method), list)
            and all(
                isinstance(value, str) and bool(value.strip())
                for value in method_blockers[method]
            )
            for method in EXPECTED_METHODS
        )
    )
    add_check(
        checks,
        "protocol.method_blockers_match_method_readiness",
        method_states_valid
        and method_blockers_valid
        and all(
            bool(method_blockers[method]) is not declared_adapter_states[method]
            for method in EXPECTED_METHODS
        ),
        "each method must have blockers exactly when its adapter readiness is false",
    )
    global_blockers = execution.get("blockers") if isinstance(execution, dict) else None
    global_blockers_valid = isinstance(global_blockers, list) and all(
        isinstance(value, str) and bool(value.strip()) for value in global_blockers
    )
    add_check(
        checks,
        "protocol.global_blockers_match_execution_ready",
        isinstance(declared_execution_ready, bool)
        and global_blockers_valid
        and bool(global_blockers) is not declared_execution_ready,
        "global blockers must be nonempty exactly when execution_ready is false",
    )
    result["declared_execution_ready"] = declared_execution_ready
    return result


def audit_schema_document(
    payload: dict[str, Any] | None,
    metadata: dict[str, Any],
    check_prefix: str,
    checks: list[dict[str, Any]],
) -> None:
    add_check(
        checks,
        f"{check_prefix}.regular_json_object",
        payload is not None,
        metadata["error"] or "regular JSON schema object loaded",
    )
    if payload is None:
        return
    structurally_valid = (
        payload.get("$schema") == "https://json-schema.org/draft/2020-12/schema"
        and payload.get("type") == "object"
        and isinstance(payload.get("required"), list)
        and isinstance(payload.get("properties"), dict)
        and payload.get("additionalProperties") is False
    )
    add_check(
        checks,
        f"{check_prefix}.draft2020_structure",
        structurally_valid,
        "schema must be a closed Draft 2020-12 object with required/properties",
    )


def adapter_readiness(
    protocol: dict[str, Any] | None,
    protocol_audit: dict[str, Any],
    checks: list[dict[str, Any]],
) -> dict[str, Any]:
    declared = protocol_audit["declared_method_adapters_ready"]
    method_rows = protocol_audit["method_rows"]
    records: dict[str, Any] = {}
    for method in EXPECTED_METHODS:
        row = method_rows.get(method, {})
        raw_path = row.get("adapter_entrypoint", row.get("adapter_path"))
        expected_path = DEFAULT_ADAPTER_PATHS[method]
        bound_path, path_error = declared_repo_path(raw_path)
        path_matches = bound_path == expected_path
        exists = bool(
            path_matches
            and bound_path is not None
            and bound_path.is_file()
            and not bound_path.is_symlink()
        )
        actual_hash = sha256_file(bound_path) if exists and bound_path else None
        expected_hash = row.get("adapter_sha256")
        binding_present = path_matches and is_sha256(expected_hash)
        binding_matches = binding_present and actual_hash == expected_hash
        declared_ready = declared.get(method) is True
        ready = bool(exists and binding_matches and declared_ready)
        reasons: list[str] = []
        if path_error is not None or not path_matches:
            reasons.append("adapter path binding invalid")
        if not exists:
            reasons.append("adapter entrypoint missing")
        if not binding_present:
            reasons.append("adapter path/SHA binding absent")
        elif not binding_matches:
            reasons.append("adapter SHA binding mismatch")
        if not declared_ready:
            reasons.append("protocol does not declare adapter ready")
        records[method] = {
            "adapter_path": rel(expected_path),
            "actual_sha256": actual_hash,
            "binding_present": binding_present,
            "binding_matches": binding_matches,
            "declared_ready": declared_ready,
            "ready": ready,
            "reasons": reasons,
        }
        add_check(
            checks,
            f"adapter.{method}.ready",
            ready,
            "; ".join(reasons) if reasons else "adapter exists and is hash-bound",
            scope="adapter_readiness",
        )
    return {
        "declared_execution_ready": protocol_audit["declared_execution_ready"],
        "methods": records,
        "ready": all(record["ready"] for record in records.values()),
        "note": "Diagnostic only; excluded from --require-ready and frozen-consistency status.",
    }


def render_report(summary: dict[str, Any], evidence_manifest: dict[str, Any]) -> str:
    frozen = summary["frozen_consistency"]
    adapter = summary["execution_adapter_readiness"]
    lines = [
        "# Table 1 Common Authoring Preflight",
        "",
        f"Generated: `{summary['generated_at_utc']}`",
        "",
        f"Frozen consistency: **{frozen['status']}** ({frozen['passed_checks']}/{frozen['total_checks']} checks passed).",
        "",
        f"Execution adapters: **{'READY' if adapter['ready'] else 'NOT READY'}**. This is diagnostic only and does not affect `--require-ready`.",
        "",
        "## Frozen Inputs",
        "",
        "| Input | Exists | SHA-256 |",
        "|---|---:|---|",
    ]
    for name in ("manifest", "protocol", "hidden_specs", "common_evaluator", "package_schema", "result_schema"):
        row = evidence_manifest["inputs"][name]
        lines.append(f"| `{row['path']}` | {str(row['exists']).lower()} | `{row['sha256'] or 'N/A'}` |")
    lines.extend(["", "## Contract", ""])
    contract = summary["contract"]
    lines.extend(
        [
            f"- Public tasks: {contract['public_task_count']}/{EXPECTED_TASKS}.",
            f"- Hidden specs: {contract['hidden_spec_count']}/{EXPECTED_TASKS}.",
            f"- Canonical hidden hashes matched: {contract['canonical_hidden_hash_match_count']}/{EXPECTED_TASKS}.",
            f"- Public-to-private hidden hash bindings matched: {contract['public_hidden_hash_match_count']}/{EXPECTED_TASKS}.",
            f"- Design denominator: {EXPECTED_TASKS} tasks x {len(EXPECTED_REPEAT_IDS)} independent repeats = {EXPECTED_TASKS * len(EXPECTED_REPEAT_IDS)} runs per method.",
        ]
    )
    lines.extend(["", "## Frozen Blockers", ""])
    if frozen["blockers"]:
        lines.extend(f"- `{row['check_id']}`: {row['detail']}" for row in frozen["blockers"])
    else:
        lines.append("- None.")
    lines.extend(["", "## Adapter Readiness", ""])
    for method in EXPECTED_METHODS:
        row = adapter["methods"][method]
        reason = "; ".join(row["reasons"]) if row["reasons"] else "ready"
        lines.append(f"- `{method}`: {'READY' if row['ready'] else 'NOT READY'}; {reason}.")
    lines.extend(["", "## Frozen Implementations", ""])
    for method in EXPECTED_METHODS:
        row = contract["method_implementations"][method]
        lines.append(
            f"- `{method}`: checkout `{row['checkout_path'] or 'N/A'}`; "
            f"HEAD `{row['actual_commit'] or 'N/A'}`; tree "
            f"`{row['actual_git_tree'] or 'N/A'}`; tracked clean "
            f"`{row['actual_tracked_clean']}`; entrypoint tracked/matches HEAD "
            f"`{row['entrypoint_tracked']}/{row['entrypoint_matches_head']}`; adapter SHA "
            f"`{row['actual_adapter_sha256'] or 'N/A'}`."
        )
    lines.extend(
        [
            "",
            "## Claim Boundary",
            "",
            "This preflight makes no model/API calls and executes no authoring adapter. A READY frozen-consistency result proves only that the 54-task public manifest, evaluator-only hidden specifications, common protocol, evaluator, and record schemas are mutually hash-bound and structurally consistent. It is not a generation result.",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--hidden-specs", type=Path, default=DEFAULT_HIDDEN_SPECS)
    parser.add_argument("--result-schema", type=Path, default=DEFAULT_RESULT_SCHEMA)
    parser.add_argument("--package-schema", type=Path, default=DEFAULT_PACKAGE_SCHEMA)
    parser.add_argument("--common-evaluator", type=Path, default=DEFAULT_EVALUATOR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--require-ready",
        action="store_true",
        help="Exit 2 unless frozen inputs and bindings are consistent; adapter readiness is excluded.",
    )
    args = parser.parse_args()

    for name, value in THREAD_ENV.items():
        os.environ[name] = value

    try:
        paths = {
            "manifest": contained(args.manifest),
            "protocol": contained(args.protocol),
            "hidden_specs": contained(args.hidden_specs),
            "result_schema": contained(args.result_schema),
            "package_schema": contained(args.package_schema),
            "common_evaluator": contained(args.common_evaluator),
        }
        output_dir = contained(args.output_dir)
    except ValueError as exc:
        parser.error(str(exc))

    checks: list[dict[str, Any]] = []
    loaded: dict[str, dict[str, Any] | None] = {}
    input_metadata: dict[str, dict[str, Any]] = {}
    for name, path in paths.items():
        if name == "common_evaluator":
            exists = path.is_file() and not path.is_symlink()
            input_metadata[name] = {
                "path": rel(path),
                "exists": exists,
                "regular_file": exists,
                "size_bytes": path.stat().st_size if exists else None,
                "sha256": sha256_file(path) if exists else None,
                "valid_json_object": None,
                "error": None if exists else "missing, non-regular, or symbolic-link input",
            }
            loaded[name] = None
            add_check(
                checks,
                "evaluator.regular_file",
                exists,
                input_metadata[name]["error"] or "regular evaluator source loaded",
            )
            continue
        loaded[name], input_metadata[name] = load_json_object(path)

    public = audit_manifest(loaded["manifest"], input_metadata["manifest"], checks)
    hidden = audit_hidden_specs(loaded["hidden_specs"], input_metadata["hidden_specs"], public, checks)
    audit_schema_document(loaded["package_schema"], input_metadata["package_schema"], "package_schema", checks)
    audit_schema_document(loaded["result_schema"], input_metadata["result_schema"], "result_schema", checks)
    protocol = audit_protocol(loaded["protocol"], input_metadata["protocol"], input_metadata, checks)
    adapters = adapter_readiness(loaded["protocol"], protocol, checks)

    frozen_checks = [row for row in checks if row["scope"] == "frozen_consistency"]
    frozen_blockers = [row for row in frozen_checks if not row["passed"]]
    frozen_ready = bool(frozen_checks) and not frozen_blockers
    generated_at = utc_now()
    output_paths = {
        "manifest": output_dir / "manifest.json",
        "summary": output_dir / "summary.json",
        "self_check": output_dir / "self_check.json",
        "report": output_dir / "report.md",
    }
    evidence_manifest = {
        "schema_version": "table1_authoring_common_preflight_manifest_v1",
        "generated_at_utc": generated_at,
        "mode": "local_read_only_preflight_no_api",
        "inputs": input_metadata,
        "outputs": {name: rel(path) for name, path in output_paths.items()},
        "preflight_script": {
            "path": rel(Path(__file__).resolve()),
            "sha256": sha256_file(Path(__file__).resolve()),
        },
        "numeric_thread_environment": THREAD_ENV,
        "privacy": {
            "hidden_spec_content_persisted": False,
            "hidden_spec_task_records_persisted": False,
            "public_manifest_hidden_material_fields": False
            if next((row for row in checks if row["check_id"] == "manifest.no_hidden_material"), {"passed": False})["passed"]
            else None,
        },
    }
    summary = {
        "schema_version": "table1_authoring_common_preflight_summary_v1",
        "generated_at_utc": generated_at,
        "status": "READY" if frozen_ready else "BLOCKED_FROZEN_CONSISTENCY",
        "require_ready_requested": args.require_ready,
        "frozen_consistency": {
            "ready": frozen_ready,
            "status": "READY" if frozen_ready else "BLOCKED",
            "total_checks": len(frozen_checks),
            "passed_checks": sum(row["passed"] for row in frozen_checks),
            "failed_checks": len(frozen_blockers),
            "blockers": frozen_blockers,
        },
        "execution_adapter_readiness": adapters,
        "contract": {
            "manifest_id": public["manifest_id"],
            "protocol_id": protocol["protocol_id"],
            "hidden_specs_id": hidden["hidden_specs_id"],
            "domains": public["domains"],
            "domain_counts": public["domain_counts"],
            "difficulty_counts": public["difficulty_counts"],
            "domain_difficulty_counts": public["domain_difficulty_counts"],
            "public_task_count": public["task_count"],
            "hidden_spec_count": hidden["spec_count"],
            "canonical_hidden_hash_match_count": hidden["canonical_hash_match_count"],
            "public_hidden_hash_match_count": hidden["public_hash_match_count"],
            "repeat_ids": EXPECTED_REPEAT_IDS,
            "runs_per_method": EXPECTED_TASKS * len(EXPECTED_REPEAT_IDS),
            "methods": EXPECTED_METHODS,
            "method_implementations": protocol["method_implementations"],
        },
        "claim_boundary": "Frozen protocol consistency only; no model/API call, authoring attempt, or generation result.",
    }
    self_check = {
        "schema_version": "table1_authoring_common_preflight_self_check_v1",
        "generated_at_utc": generated_at,
        "pass": frozen_ready,
        "frozen_consistency_ready": frozen_ready,
        "adapter_readiness_excluded_from_gate": True,
        "input_files_modified": False,
        "network_or_api_calls": 0,
        "authoring_attempts": 0,
        "hidden_spec_content_persisted": False,
        "checks": checks,
        "check_counts": {
            "all": len(checks),
            "frozen_consistency": len(frozen_checks),
            "adapter_readiness": sum(row["scope"] == "adapter_readiness" for row in checks),
            "passed": sum(row["passed"] for row in checks),
            "failed": sum(not row["passed"] for row in checks),
        },
    }

    write_json(output_paths["summary"], summary)
    write_json(output_paths["self_check"], self_check)
    write_text_atomic(output_paths["report"], render_report(summary, evidence_manifest))
    write_json(output_paths["manifest"], evidence_manifest)

    emitted = {
        name: {"path": rel(path), "sha256": sha256_file(path), "size_bytes": path.stat().st_size}
        for name, path in output_paths.items()
    }
    print(
        json.dumps(
            {
                "adapter_ready": adapters["ready"],
                "frozen_consistency_ready": frozen_ready,
                "outputs": emitted,
                "status": summary["status"],
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 2 if args.require_ready and not frozen_ready else 0


if __name__ == "__main__":
    raise SystemExit(main())
