#!/usr/bin/env python3
"""Fail-closed LAM preflight and official-release telemetry audit for Table 1.

This command never invokes an API and never treats the released LAM dataset as
a common-protocol authoring rerun.  It is intentionally reusable: once the
common protocol and authoring manifest are frozen, rerunning this command will
check their binding and report whether the separate generation runner may be
started.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import pandas as pd
import yaml


WORKSPACE_ROOT = Path("/mnt/zsn/lyb").resolve()
REPO_ROOT = WORKSPACE_ROOT / "arti-skill"
DEFAULT_OUTPUT = REPO_ROOT / "exp" / "runtime" / "table1_reliability" / "lam"
DEFAULT_PROTOCOL = REPO_ROOT / "exp" / "reference" / "table1_reliability_protocol_v1.json"
DEFAULT_AUTHORING_MANIFEST = (
    REPO_ROOT / "exp" / "reference" / "table1_reliability_common_authoring_v1.json"
)
DEFAULT_LAM_CHECKOUT = REPO_ROOT / "exp" / "baselines" / "LAM-official"
DEFAULT_RELEASE_DATASET = REPO_ROOT / "exp" / "baselines" / "LAM-official-dataset"
DEFAULT_CREDENTIAL_ENV = REPO_ROOT / "articraft_data" / ".env"
EXPECTED_LAM_COMMIT = "0b3a87beb8c35273a5acf8681221791aff746d8e"
EXPECTED_RELEASE_ROWS = 3217
EXPECTED_TASKS = 54
EXPECTED_RUNS_PER_TASK = 3
EXPECTED_REPAIR_BUDGET = 3
MODEL_PREFIXES = ("gpt-", "o1", "o3", "o4", "gemini", "claude")
SECRET_ENV_NAMES = (
    "OPENAI_API_KEY",
    "GOOGLE_API_KEY",
    "GEMINI_API_KEY",
    "ANTHROPIC_API_KEY",
)
THREAD_ENV = {
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}


def contained(path: Path, *, must_exist: bool = False) -> Path:
    resolved = path.expanduser().resolve(strict=must_exist)
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path escapes authorized workspace: {resolved}")
    return resolved


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path, must_exist=True).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, payload: Any) -> None:
    target = contained(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def rel(path: Path) -> str:
    return contained(path).relative_to(WORKSPACE_ROOT).as_posix()


def run_command(argv: list[str], cwd: Path, *, timeout: int = 30) -> dict[str, Any]:
    env = os.environ.copy()
    env.update(THREAD_ENV)
    try:
        result = subprocess.run(
            argv,
            cwd=contained(cwd, must_exist=True),
            env=env,
            text=True,
            capture_output=True,
            timeout=timeout,
            check=False,
        )
        return {
            "argv": argv,
            "exit_code": result.returncode,
            "stdout_nonempty": bool(result.stdout.strip()),
            "stderr_nonempty": bool(result.stderr.strip()),
            "timed_out": False,
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "argv": argv,
            "exit_code": None,
            "stdout_nonempty": bool(exc.stdout),
            "stderr_nonempty": bool(exc.stderr),
            "timed_out": True,
        }
    except OSError as exc:
        return {
            "argv": argv,
            "exit_code": None,
            "stdout_nonempty": False,
            "stderr_nonempty": False,
            "timed_out": False,
            "launch_error": f"{type(exc).__name__}: {exc}",
        }


def git_value(checkout: Path, argv: list[str]) -> str | None:
    result = subprocess.run(
        ["git", *argv],
        cwd=contained(checkout, must_exist=True),
        text=True,
        capture_output=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else None


def flatten(payload: Any, prefix: str = "") -> Iterable[tuple[str, Any]]:
    if isinstance(payload, dict):
        for key, value in payload.items():
            child = f"{prefix}.{key}" if prefix else str(key)
            yield child, value
            yield from flatten(value, child)
    elif isinstance(payload, list):
        for index, value in enumerate(payload):
            child = f"{prefix}[{index}]"
            yield child, value
            yield from flatten(value, child)


def normalized_key(path: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", path.lower()).strip("_")


def scalar_matches(payload: Any, key_terms: tuple[str, ...], value: Any) -> bool:
    for path, candidate in flatten(payload):
        key = normalized_key(path)
        if all(term in key for term in key_terms) and candidate == value:
            return True
    return False


def has_sha_binding(payload: Any, target_sha: str | None) -> bool:
    if target_sha is None:
        return False
    for path, value in flatten(payload):
        if "sha256" in normalized_key(path) and value == target_sha:
            return True
    return False


def bound_manifest_sha256(payload: dict[str, Any]) -> str | None:
    manifest = payload.get("manifest")
    if isinstance(manifest, dict) and isinstance(manifest.get("sha256"), str):
        return manifest["sha256"]
    if isinstance(payload.get("manifest_sha256"), str):
        return payload["manifest_sha256"]
    frozen = payload.get("frozen_protocol")
    if isinstance(frozen, dict) and isinstance(frozen.get("manifest_sha256"), str):
        return frozen["manifest_sha256"]
    return None


def has_definition(payload: Any, aliases: tuple[str, ...]) -> bool:
    for path, value in flatten(payload):
        key = normalized_key(path)
        if any(alias in key for alias in aliases) and isinstance(value, str) and value.strip():
            return True
    return False


def has_hidden_spec_policy(payload: Any) -> bool:
    for path, value in flatten(payload):
        key = normalized_key(path)
        if "hidden" not in key or "spec" not in key:
            continue
        if value is False and any(token in key for token in ("visible", "provided", "exposed")):
            return True
        if value is True and any(token in key for token in ("withheld", "hidden", "blind")):
            return True
        if isinstance(value, str) and any(
            token in value.lower() for token in ("withheld", "not provided", "not exposed")
        ):
            return True
    return False


def extract_models(payload: Any) -> list[str]:
    models: set[str] = set()
    for path, value in flatten(payload):
        if "model" not in normalized_key(path) or not isinstance(value, str):
            continue
        candidate = value.strip()
        if candidate.lower().startswith(MODEL_PREFIXES):
            models.add(candidate)
    return sorted(models)


def model_provider(model: str) -> str | None:
    lowered = model.lower()
    if lowered.startswith(("gpt-", "o1", "o3", "o4")):
        return "openai"
    if lowered.startswith("gemini"):
        return "google"
    if lowered.startswith("claude"):
        return "anthropic"
    return None


def load_json_object(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    if not path.is_file() or path.is_symlink():
        return None, "missing or non-regular file"
    try:
        payload = json.loads(contained(path, must_exist=True).read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        return None, f"{type(exc).__name__}: {exc}"
    if not isinstance(payload, dict):
        return None, "top-level JSON value is not an object"
    return payload, None


def manifest_items(payload: dict[str, Any]) -> tuple[list[Any], str | None]:
    for key in ("tasks", "items", "authoring_tasks"):
        value = payload.get(key)
        if isinstance(value, list):
            return value, key
    return [], None


def audit_authoring_manifest(path: Path) -> dict[str, Any]:
    payload, error = load_json_object(path)
    if payload is None:
        return {
            "path": rel(path),
            "exists": path.is_file(),
            "sha256": None,
            "valid_json_object": False,
            "error": error,
            "manifest_id": None,
            "items_key": None,
            "task_count": 0,
            "declared_task_count": None,
            "declared_task_count_matches": False,
            "repeat_ids": [],
            "repeat_count": 0,
            "repeat_ids_unique": False,
            "expected_runs_per_method": None,
            "task_ids_unique": False,
            "task_ids_nonempty": False,
            "prompts_nonempty": False,
            "prompt_sha256_matches": False,
            "frozen_before_run": False,
            "frozen_at_utc_nonempty": False,
            "task_fingerprints": [],
        }

    items, items_key = manifest_items(payload)
    task_ids: list[str] = []
    prompt_hashes: list[dict[str, str]] = []
    prompts_nonempty = True
    prompt_sha256_matches = True
    task_ids_nonempty = True
    for index, item in enumerate(items):
        if not isinstance(item, dict):
            task_ids_nonempty = False
            prompts_nonempty = False
            continue
        task_id = next(
            (str(item[key]).strip() for key in ("task_id", "id", "item_id") if item.get(key)),
            "",
        )
        prompt = next(
            (
                str(item[key]).strip()
                for key in ("prompt", "description", "text_prompt", "input_prompt")
                if item.get(key)
            ),
            "",
        )
        task_ids_nonempty &= bool(task_id)
        prompts_nonempty &= bool(prompt)
        declared_prompt_sha = item.get("prompt_sha256")
        computed_prompt_sha = hashlib.sha256(prompt.encode("utf-8")).hexdigest() if prompt else None
        prompt_sha256_matches &= bool(
            computed_prompt_sha
            and isinstance(declared_prompt_sha, str)
            and declared_prompt_sha.lower() == computed_prompt_sha
        )
        if task_id:
            task_ids.append(task_id)
        if task_id and prompt:
            prompt_hashes.append(
                {
                    "task_id": task_id,
                    "prompt_sha256": computed_prompt_sha,
                }
            )

    frozen = any(
        scalar_matches(payload, terms, True)
        for terms in (
            ("frozen",),
            ("selection", "frozen"),
            ("frozen", "before", "run"),
        )
    )
    manifest_id = next(
        (str(payload[key]) for key in ("manifest_id", "protocol_id", "cohort_id") if payload.get(key)),
        None,
    )
    repeat_ids = payload.get("repeat_ids")
    normalized_repeat_ids = (
        [str(value).strip() for value in repeat_ids]
        if isinstance(repeat_ids, list)
        else []
    )
    declared_task_count = payload.get("task_count")
    expected_runs_per_method = payload.get("expected_runs_per_method")
    return {
        "path": rel(path),
        "exists": True,
        "sha256": sha256(path),
        "valid_json_object": True,
        "error": None,
        "manifest_id": manifest_id,
        "items_key": items_key,
        "task_count": len(items),
        "declared_task_count": declared_task_count,
        "declared_task_count_matches": declared_task_count == len(items),
        "repeat_ids": normalized_repeat_ids,
        "repeat_count": len(normalized_repeat_ids),
        "repeat_ids_unique": bool(normalized_repeat_ids)
        and all(normalized_repeat_ids)
        and len(set(normalized_repeat_ids)) == len(normalized_repeat_ids),
        "expected_runs_per_method": expected_runs_per_method,
        "task_ids_unique": bool(items) and len(task_ids) == len(items) and len(set(task_ids)) == len(items),
        "task_ids_nonempty": bool(items) and task_ids_nonempty and len(task_ids) == len(items),
        "prompts_nonempty": bool(items) and prompts_nonempty and len(prompt_hashes) == len(items),
        "prompt_sha256_matches": bool(items) and prompt_sha256_matches,
        "frozen_before_run": frozen,
        "frozen_at_utc_nonempty": bool(str(payload.get("frozen_at_utc", "")).strip()),
        "task_fingerprints": sorted(prompt_hashes, key=lambda row: row["task_id"]),
    }


def audit_protocol(path: Path, authoring: dict[str, Any]) -> dict[str, Any]:
    payload, error = load_json_object(path)
    if payload is None:
        return {
            "path": rel(path),
            "exists": path.is_file(),
            "sha256": None,
            "valid_json_object": False,
            "error": error,
            "protocol_id": None,
            "models": [],
            "method_id_present": False,
            "checks": {},
        }

    protocol_id = next(
        (str(payload[key]) for key in ("protocol_id", "experiment_id", "id") if payload.get(key)),
        None,
    )
    frozen_protocol = payload.get("frozen_protocol")
    legacy = frozen_protocol if isinstance(frozen_protocol, dict) else {}
    methods = payload.get("methods", legacy.get("methods", []))
    lam_arm: dict[str, Any] | None = None
    method_id_present = False
    method_ids: list[str] = []
    if isinstance(methods, list):
        for row in methods:
            method_id = (
                row.strip().lower()
                if isinstance(row, str)
                else str(row.get("method_id", "")).strip().lower()
                if isinstance(row, dict)
                else ""
            )
            if method_id:
                method_ids.append(method_id)
            if method_id == "lam":
                method_id_present = True
                if isinstance(row, dict):
                    lam_arm = row
    elif isinstance(methods, dict):
        method_ids = [str(key).strip().lower() for key in methods if str(key).strip()]
        candidate = methods.get("lam")
        if isinstance(candidate, dict):
            method_id_present = True
            lam_arm = candidate

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
    common_binding_exact = (
        str(common_binding.get("provider", "")).strip().lower() == "openai"
        and str(
            common_binding.get(
                "model_id",
                common_binding.get("model", common_binding.get("exact_model_id", "")),
            )
        ).strip()
        == "gpt-5"
    )
    lam_roles_match_common_binding = method_id_present and common_binding_exact
    if lam_arm:
        roles = lam_arm.get("roles")
        if isinstance(roles, list) and roles:
            lam_roles_match_common_binding = all(
                isinstance(role, dict)
                and str(role.get("provider", "")).strip().lower() == "openai"
                and str(
                    role.get(
                        "model_id", role.get("model", role.get("exact_model_id", ""))
                    )
                ).strip()
                == "gpt-5"
                for role in roles
            )
        elif any(
            key in lam_arm for key in ("provider", "model_id", "model", "exact_model_id")
        ):
            lam_roles_match_common_binding = (
                str(lam_arm.get("provider", "")).strip().lower() == "openai"
                and str(
                    lam_arm.get(
                        "model_id", lam_arm.get("model", lam_arm.get("exact_model_id", ""))
                    )
                ).strip()
                == "gpt-5"
            )
    models = extract_models(lam_arm) if lam_arm else []
    if not models:
        models = extract_models(common_binding)
    if not models and isinstance(legacy.get("exact_model_id"), str):
        models = [legacy["exact_model_id"]]

    definitions_payload = {
        "definitions": payload.get("definitions", legacy.get("definitions", {})),
        "attempt_semantics": payload.get("attempt_semantics", {}),
        "metric_definitions": payload.get("metric_definitions", {}),
    }
    definitions_text = json.dumps(definitions_payload, sort_keys=True).lower()
    definitions = {
        metric: any(alias in definitions_text for alias in aliases)
        or has_definition(payload, aliases)
        for metric, aliases in {
            "executable": ("executable_definition", "template_executable", "executable"),
            "artifact_saved": ("artifact_saved_definition", "artifact_saved"),
            "first_shot": ("first_shot_definition", "first_shot"),
            "final_success": ("final_success_definition", "final_success"),
        }.items()
    }
    telemetry_payload = payload.get(
        "telemetry_fields",
        payload.get("telemetry_requirements", legacy.get("telemetry_fields", {})),
    )
    telemetry_text = json.dumps(telemetry_payload, sort_keys=True).lower()
    telemetry = {
        metric: metric in telemetry_text
        or any(metric in normalized_key(key) for key, _ in flatten(payload))
        for metric in ("wall_time", "input_tokens", "output_tokens", "api_cost")
    }
    protocol_repeat_ids = payload.get("repeat_ids", legacy.get("repeat_ids"))
    protocol_repeat_ids = (
        [str(value).strip() for value in protocol_repeat_ids]
        if isinstance(protocol_repeat_ids, list)
        else []
    )
    common_evaluator = payload.get("common_evaluator", legacy.get("common_evaluator"))
    output_isolation = payload.get(
        "method_output_isolation",
        payload.get("output_isolation", legacy.get("method_output_isolation")),
    )
    attempt_schema = payload.get(
        "attempt_schema", payload.get("result_schema", legacy.get("attempt_schema"))
    )
    execution_readiness = payload.get(
        "execution_readiness", legacy.get("execution_readiness")
    )
    hidden_specs = payload.get("hidden_specs", legacy.get("hidden_specs"))
    hidden_specs_sha = (
        hidden_specs.get("sha256")
        if isinstance(hidden_specs, dict)
        else payload.get("hidden_specs_sha256", legacy.get("hidden_specs_sha256"))
    )
    checks = {
        "protocol_id_nonempty": bool(protocol_id),
        "schema_version_present": isinstance(payload.get("schema_version"), (int, str)),
        "frozen_at_utc_nonempty": bool(str(payload.get("frozen_at_utc", "")).strip()),
        "frozen_before_first_run": payload.get("frozen_before_first_run") is True
        or payload.get("frozen_design") is True,
        "manifest_sha256_bound": bool(authoring.get("sha256"))
        and bound_manifest_sha256(payload) == authoring["sha256"],
        "expected_task_count_54": scalar_matches(payload, ("task", "count"), EXPECTED_TASKS)
        or scalar_matches(payload, ("expected", "tasks"), EXPECTED_TASKS),
        "independent_runs_per_task_3": scalar_matches(payload, ("runs", "per", "task"), EXPECTED_RUNS_PER_TASK)
        or scalar_matches(payload, ("repetitions",), EXPECTED_RUNS_PER_TASK),
        "repeat_ids_match_manifest": bool(protocol_repeat_ids)
        and protocol_repeat_ids == authoring.get("repeat_ids"),
        "repair_budget_3": scalar_matches(payload, ("repair", "budget"), EXPECTED_REPAIR_BUDGET)
        or scalar_matches(payload, ("max", "common", "repair", "turns"), EXPECTED_REPAIR_BUDGET),
        "hidden_spec_withheld": has_hidden_spec_policy(payload),
        "hidden_specs_sha256_frozen": isinstance(hidden_specs_sha, str)
        and bool(re.fullmatch(r"[0-9a-fA-F]{64}", hidden_specs_sha)),
        "lam_method_arm_present": method_id_present,
        "openai_gpt5_exact_binding": common_binding_exact,
        "lam_roles_match_openai_gpt5_binding": lam_roles_match_common_binding,
        "method_ids_exact": set(method_ids) == {"pva", "lam", "articraft"}
        and len(method_ids) == 3,
        "success_definitions_complete": all(definitions.values()),
        "common_evaluator_frozen": isinstance(common_evaluator, (dict, str))
        and bool(common_evaluator),
        "method_output_isolation_frozen": isinstance(output_isolation, (dict, str))
        and bool(output_isolation),
        "attempt_schema_frozen": (
            isinstance(attempt_schema, (dict, str)) and bool(attempt_schema)
        )
        or (
            isinstance(telemetry_payload, dict)
            and isinstance(telemetry_payload.get("attempt_schema"), (dict, str))
            and bool(telemetry_payload["attempt_schema"])
        ),
        "resource_telemetry_complete": all(telemetry.values()),
        "timeout_frozen": any(
            "timeout" in normalized_key(key) and isinstance(value, (int, float)) and value > 0
            for key, value in flatten(payload)
        ),
        "execution_readiness_declared": isinstance(execution_readiness, (dict, str))
        and bool(execution_readiness),
    }
    return {
        "path": rel(path),
        "exists": True,
        "sha256": sha256(path),
        "valid_json_object": True,
        "error": None,
        "protocol_id": protocol_id,
        "models": sorted(set(models)),
        "method_id_present": method_id_present,
        "method_arm": lam_arm,
        "method_ids": method_ids,
        "repeat_ids": protocol_repeat_ids,
        "common_model_binding": common_binding,
        "openai_gpt5_exact_binding": common_binding_exact,
        "lam_roles_match_common_binding": lam_roles_match_common_binding,
        "definitions": definitions,
        "telemetry_fields": telemetry,
        "checks": checks,
    }


def dotenv_presence(path: Path) -> dict[str, bool]:
    present = {name: False for name in SECRET_ENV_NAMES}
    if not path.is_file() or path.is_symlink():
        return present
    for raw_line in contained(path, must_exist=True).read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip("\"'")
        if key in present and value and not value.upper().startswith(("YOUR_", "REPLACE_")):
            present[key] = True
    return present


def safe_find_spec(module: str) -> bool:
    try:
        return importlib.util.find_spec(module) is not None
    except (ImportError, ModuleNotFoundError, ValueError):
        return False


def audit_lam_config(path: Path, protocol_models: list[str]) -> dict[str, Any]:
    models: list[str] = []
    error: str | None = None
    if path.is_file() and not path.is_symlink():
        try:
            payload = yaml.safe_load(contained(path, must_exist=True).read_text(encoding="utf-8"))
            if isinstance(payload, dict):
                models = extract_models(payload)
            else:
                error = "top-level YAML value is not an object"
        except (OSError, UnicodeDecodeError, yaml.YAMLError) as exc:
            error = f"{type(exc).__name__}: {exc}"
    effective = sorted(set(protocol_models or models))
    return {
        "path": rel(path),
        "exists": path.is_file() and not path.is_symlink(),
        "parse_error": error,
        "models_source": "common_protocol" if protocol_models else ("lam_config" if models else None),
        "models": effective,
        "providers": sorted({provider for model in effective if (provider := model_provider(model))}),
        "model_binding_frozen": bool(effective) and (bool(protocol_models) or path.is_file()),
    }


def audit_runtime(
    checkout: Path,
    lam_config: Path,
    credential_env: Path,
    protocol_models: list[str],
) -> dict[str, Any]:
    head = git_value(checkout, ["rev-parse", "HEAD"])
    status = git_value(checkout, ["status", "--porcelain=v1", "--untracked-files=all"])
    origin = git_value(checkout, ["remote", "get-url", "origin"])
    config = audit_lam_config(lam_config, protocol_models)

    environment_presence = {name: bool(os.environ.get(name)) for name in SECRET_ENV_NAMES}
    file_presence = dotenv_presence(credential_env)
    provider_key = {
        "openai": environment_presence["OPENAI_API_KEY"] or file_presence["OPENAI_API_KEY"],
        "google": environment_presence["GOOGLE_API_KEY"]
        or environment_presence["GEMINI_API_KEY"]
        or file_presence["GOOGLE_API_KEY"]
        or file_presence["GEMINI_API_KEY"],
        "anthropic": environment_presence["ANTHROPIC_API_KEY"] or file_presence["ANTHROPIC_API_KEY"],
    }
    provider_packages = {
        "openai": safe_find_spec("openai") and safe_find_spec("tiktoken"),
        "google": safe_find_spec("google.genai"),
        "anthropic": safe_find_spec("anthropic"),
    }
    required_providers = config["providers"]
    provider_ready = {
        provider: provider_key.get(provider, False) and provider_packages.get(provider, False)
        for provider in required_providers
    }
    python_modules = {
        module: safe_find_spec(module)
        for module in (
            "anthropic",
            "google.genai",
            "numpy",
            "openai",
            "PIL",
            "pydantic",
            "pyrender",
            "tenacity",
            "tiktoken",
            "trimesh",
            "yaml",
        )
    }
    node_packages = {
        package: (checkout / "node_modules" / package / "package.json").is_file()
        for package in ("canvas", "jsdom", "puppeteer", "three")
    }
    cli_smoke = run_command([sys.executable, "run_pipeline.py", "--help"], checkout)
    return {
        "checkout": {
            "path": rel(checkout),
            "git_head": head,
            "expected_git_head": EXPECTED_LAM_COMMIT,
            "commit_match": head == EXPECTED_LAM_COMMIT,
            "worktree_clean": status == "",
            "origin": origin,
            "official_origin_match": origin == "https://github.com/gaoypeng/LAM.git",
        },
        "execution_config": config,
        "provider_audit": {
            "credential_env_path": rel(credential_env),
            "credential_env_exists": credential_env.is_file(),
            "environment_key_configured": environment_presence,
            "credential_file_key_configured": file_presence,
            "credential_values_recorded": False,
            "provider_package_ready": provider_packages,
            "required_providers": required_providers,
            "provider_ready": provider_ready,
        },
        "dependencies": {
            "python_modules": python_modules,
            "node_modules_directory": (checkout / "node_modules").is_dir(),
            "node_packages": node_packages,
            "cli_help_smoke": cli_smoke,
        },
    }


def nonempty_string_mask(series: pd.Series) -> pd.Series:
    return series.notna() & series.map(lambda value: isinstance(value, str) and bool(value.strip()))


def parse_json_column(series: pd.Series) -> tuple[int, list[str]]:
    parsed = 0
    failures: list[str] = []
    for index, value in series.items():
        if not isinstance(value, str) or not value.strip():
            continue
        try:
            json.loads(value)
            parsed += 1
        except json.JSONDecodeError:
            failures.append(str(index))
    return parsed, failures


def parse_urdf_column(series: pd.Series) -> tuple[int, list[str]]:
    parsed = 0
    failures: list[str] = []
    for index, value in series.items():
        if not isinstance(value, str) or not value.strip():
            continue
        try:
            root = ET.fromstring(value)
            if root.tag != "robot":
                raise ValueError("root is not robot")
            parsed += 1
        except (ET.ParseError, ValueError):
            failures.append(str(index))
    return parsed, failures


def audit_release(dataset: Path) -> dict[str, Any]:
    code_path = contained(dataset / "articulated_code.parquet", must_exist=True)
    manifest_path = contained(dataset / "manifest.parquet", must_exist=True)
    manifest_csv = contained(dataset / "manifest.csv", must_exist=True)
    readme = contained(dataset / "README.md", must_exist=True)
    api_record = contained(dataset / "dataset_api.json", must_exist=True)

    code = pd.read_parquet(code_path)
    manifest = pd.read_parquet(manifest_path)
    csv_manifest = pd.read_csv(manifest_csv)
    required = {
        "object_release_id",
        "rel_path",
        "caption",
        "status",
        "tier",
        "threejs_code",
        "urdf",
        "articulation_json",
        "workflow_json",
    }
    missing_columns = sorted(required - set(code.columns))
    if missing_columns:
        raise RuntimeError(f"release parquet misses required columns: {missing_columns}")

    field_coverage: dict[str, dict[str, Any]] = {}
    for field in ("caption", "threejs_code", "urdf", "articulation_json", "workflow_json"):
        mask = nonempty_string_mask(code[field])
        field_coverage[field] = {
            "numerator": int(mask.sum()),
            "denominator": len(code),
            "value": float(mask.mean()),
            "missing": int((~mask).sum()),
            "missing_rel_paths": sorted(code.loc[~mask, "rel_path"].astype(str).tolist()),
        }

    articulation_parsed, articulation_failures = parse_json_column(code["articulation_json"])
    workflow_parsed, workflow_failures = parse_json_column(code["workflow_json"])
    urdf_parsed, urdf_failures = parse_urdf_column(code["urdf"])
    all_core = (
        nonempty_string_mask(code["threejs_code"])
        & nonempty_string_mask(code["urdf"])
        & nonempty_string_mask(code["articulation_json"])
        & nonempty_string_mask(code["workflow_json"])
    )
    status_counts = {str(key): int(value) for key, value in code["status"].value_counts().items()}
    tier_counts = {str(key): int(value) for key, value in code["tier"].value_counts().items()}
    crosstab = pd.crosstab(code["tier"], code["status"], dropna=False)
    status_tier = {
        str(tier): {str(status): int(crosstab.loc[tier, status]) for status in crosstab.columns}
        for tier in crosstab.index
    }

    common_columns = ["rel_path", "status", "tier"]
    code_common = code[common_columns].sort_values("rel_path").reset_index(drop=True)
    manifest_common = manifest[common_columns].sort_values("rel_path").reset_index(drop=True)
    csv_common = csv_manifest[common_columns].sort_values("rel_path").reset_index(drop=True)
    api_payload = json.loads(api_record.read_text(encoding="utf-8"))
    return {
        "evidence_class": "OFFICIAL_RELEASE_AUDIT_ONLY",
        "claim_boundary": (
            "Supplementary released-record telemetry; not a common-prompt authoring rerun, "
            "not fresh execution, and not eligible for Table 1 first-shot/final-success cells."
        ),
        "network_accessed": False,
        "generated_code_executed": False,
        "release_intent_records": len(code),
        "release_record_denominator": len(code),
        "unique_rel_paths": int(code["rel_path"].nunique(dropna=False)),
        "unique_object_release_ids": int(code["object_release_id"].nunique(dropna=False)),
        "status_counts": status_counts,
        "tier_counts": tier_counts,
        "status_by_tier": status_tier,
        "field_coverage": field_coverage,
        "parse_checks": {
            "urdf_xml_robot": {
                "numerator": urdf_parsed,
                "denominator": len(code),
                "failure_indices": urdf_failures,
            },
            "articulation_json_among_all_records": {
                "numerator": articulation_parsed,
                "denominator": len(code),
                "parse_failures_among_nonempty": articulation_failures,
            },
            "workflow_json_among_all_records": {
                "numerator": workflow_parsed,
                "denominator": len(code),
                "parse_failures_among_nonempty": workflow_failures,
            },
        },
        "all_core_fields_nonempty": {
            "fields": ["threejs_code", "urdf", "articulation_json", "workflow_json"],
            "numerator": int(all_core.sum()),
            "denominator": len(code),
            "value": float(all_core.mean()),
        },
        "source_consistency": {
            "articulated_code_rows": len(code),
            "manifest_parquet_rows": len(manifest),
            "manifest_csv_rows": len(csv_manifest),
            "rel_path_status_tier_exact_manifest_parquet": code_common.equals(manifest_common),
            "rel_path_status_tier_exact_manifest_csv": code_common.equals(csv_common),
        },
        "dataset_api": {
            "dataset_id": api_payload.get("id"),
            "revision": api_payload.get("sha"),
            "last_modified": api_payload.get("lastModified"),
        },
        "input_hashes": {
            rel(code_path): sha256(code_path),
            rel(manifest_path): sha256(manifest_path),
            rel(manifest_csv): sha256(manifest_csv),
            rel(readme): sha256(readme),
            rel(api_record): sha256(api_record),
        },
        "common_metric_mapping": {
            "executable": {"state": "N/R", "value": None},
            "artifact_saved": {"state": "N/R", "value": None},
            "first_shot": {"state": "N/R", "value": None},
            "final_success": {"state": "N/R", "value": None},
            "repair_turns": {"state": "N/R", "value": None},
            "wall_time": {"state": "N/R", "value": None},
            "tokens": {"state": "N/R", "value": None},
            "api_cost": {"state": "N/R", "value": None},
            "reason": (
                "Release PASS/FAIL, tier, and surviving fields do not encode the frozen common "
                "success evaluator, pre-repair first attempt, repair budget, or complete run telemetry."
            ),
        },
    }


def readiness_blockers(
    protocol: dict[str, Any],
    authoring: dict[str, Any],
    runtime: dict[str, Any],
    execution_gates: dict[str, bool],
) -> tuple[list[str], list[str]]:
    blockers: list[str] = []
    if not protocol["valid_json_object"]:
        blockers.append("frozen common Table 1 reliability protocol is missing or invalid")
    else:
        for check, passed in protocol["checks"].items():
            if not passed:
                blockers.append(f"common protocol contract failed: {check}")
    if not authoring["valid_json_object"]:
        blockers.append("frozen common authoring task manifest is missing or invalid")
    else:
        manifest_checks = {
            "manifest_id_nonempty": bool(authoring["manifest_id"]),
            "task_count_54": authoring["task_count"] == EXPECTED_TASKS,
            "declared_task_count_matches": authoring["declared_task_count_matches"],
            "repeat_ids_3_unique": authoring["repeat_count"] == EXPECTED_RUNS_PER_TASK
            and authoring["repeat_ids_unique"],
            "expected_runs_per_method_162": authoring["expected_runs_per_method"]
            == EXPECTED_TASKS * EXPECTED_RUNS_PER_TASK,
            "task_ids_unique": authoring["task_ids_unique"],
            "task_ids_nonempty": authoring["task_ids_nonempty"],
            "prompts_nonempty": authoring["prompts_nonempty"],
            "prompt_sha256_matches": authoring["prompt_sha256_matches"],
            "frozen_before_run": authoring["frozen_before_run"],
            "frozen_at_utc_nonempty": authoring["frozen_at_utc_nonempty"],
        }
        for check, passed in manifest_checks.items():
            if not passed:
                blockers.append(f"authoring manifest contract failed: {check}")

    checkout = runtime["checkout"]
    if not checkout["commit_match"]:
        blockers.append("LAM checkout is not at the pinned official commit")
    if not checkout["worktree_clean"]:
        blockers.append("LAM official checkout is not clean")
    if not checkout["official_origin_match"]:
        blockers.append("LAM checkout origin is not the official public repository")
    config = runtime["execution_config"]
    if not config["exists"]:
        blockers.append("LAM execution config.yaml is absent")
    if config["parse_error"]:
        blockers.append("LAM execution config.yaml is invalid")
    if not config["model_binding_frozen"]:
        blockers.append("LAM model assignments are not frozen in the common protocol or execution config")
    providers = runtime["provider_audit"]
    for provider, ready in providers["provider_ready"].items():
        if not ready:
            blockers.append(f"required {provider} provider is not credential-and-package ready")
    dependencies = runtime["dependencies"]
    if not dependencies["node_modules_directory"] or not all(dependencies["node_packages"].values()):
        blockers.append("LAM Node dependencies are not installed")
    if dependencies["cli_help_smoke"].get("exit_code") != 0:
        blockers.append("LAM CLI help smoke did not exit successfully")

    adapter_labels = {
        "common_evaluator_adapter_available": "LAM common evaluator adapter is not implemented",
        "common_repair_adapter_available": "LAM common-evaluator repair adapter is not implemented",
        "failed_output_preservation": "LAM currently deletes failed generation outputs",
        "run_id_output_isolation": "LAM has no explicit task/repeat/attempt output binding",
        "normalized_attempt_telemetry_adapter_available": (
            "LAM normalized per-attempt telemetry adapter is not implemented"
        ),
    }
    adapter_blockers = [
        adapter_labels[key] for key, available in execution_gates.items() if not available
    ]
    return blockers, adapter_blockers


def report_text(summary: dict[str, Any], manifest: dict[str, Any], provenance: dict[str, Any]) -> str:
    release = summary["official_release_telemetry"]
    fields = release["field_coverage"]
    authoring = summary["common_authoring_rerun"]
    blockers = "\n".join(f"- {item}" for item in summary["blockers"])
    command = provenance["reproduction_command"]
    return f"""# Table 1 LAM reliability preflight

Status: **{summary['status']}**

No common-protocol LAM authoring generation was executed. The authoring intent
denominator is {authoring['intent_tasks']} tasks / {authoring['intent_runs']} runs,
and attempted authoring runs are {authoring['attempted_runs']}. All Table 1 common
reliability cells therefore remain `N/R`, not zero.

## Common authoring readiness

- Protocol: `{manifest['common_authoring']['protocol']['path']}`; present={manifest['common_authoring']['protocol']['exists']}.
- Authoring manifest: `{manifest['common_authoring']['authoring_manifest']['path']}`; present={manifest['common_authoring']['authoring_manifest']['exists']}.
- Frozen tasks discovered: {manifest['common_authoring']['authoring_manifest']['task_count']} (required {EXPECTED_TASKS}).
- LAM checkout: `{provenance['lam_checkout']['git_head']}`; pinned commit match={provenance['lam_checkout']['commit_match']}; clean={provenance['lam_checkout']['worktree_clean']}.
- LAM CLI help smoke exit code: {provenance['runtime']['dependencies']['cli_help_smoke']['exit_code']}.
- API calls made: none. Credential values recorded: no.

Blockers:

{blockers}

## Official release telemetry (supplementary only)

This is a direct audit of released records. It is not fresh execution, not a
common-prompt rerun, and not eligible to fill first-shot/final-success cells.

- Release intent records: {release['release_intent_records']}/{release['release_record_denominator']}.
- Release status: PASS={release['status_counts'].get('PASS', 0)}/{release['release_record_denominator']}; FAIL={release['status_counts'].get('FAIL', 0)}/{release['release_record_denominator']}.
- Release tier: viable={release['tier_counts'].get('viable', 0)}, loads_only={release['tier_counts'].get('loads_only', 0)}, broken={release['tier_counts'].get('broken', 0)}; denominator={release['release_record_denominator']}.
- Non-empty Three.js code: {fields['threejs_code']['numerator']}/{fields['threejs_code']['denominator']}.
- Non-empty URDF: {fields['urdf']['numerator']}/{fields['urdf']['denominator']}; XML `<robot>` parse: {release['parse_checks']['urdf_xml_robot']['numerator']}/{release['parse_checks']['urdf_xml_robot']['denominator']}.
- Non-empty articulation JSON: {fields['articulation_json']['numerator']}/{fields['articulation_json']['denominator']}.
- Non-empty workflow JSON: {fields['workflow_json']['numerator']}/{fields['workflow_json']['denominator']}.
- All four core fields non-empty: {release['all_core_fields_nonempty']['numerator']}/{release['all_core_fields_nonempty']['denominator']}.
- Non-empty release caption: {fields['caption']['numerator']}/{fields['caption']['denominator']}.

`PASS` is the release metadata status: all viable and loads-only records are
PASS, while all broken records are FAIL. It is not reinterpreted as common
`Executable`, `First-shot`, or `Final Success`. Release survival also cannot
prove artifact-saved rate over the original intent-to-run population.

## Reproduce

```bash
{command}
```

Outputs: `manifest.json`, `provenance.json`, `summary.json`, `self_check.json`,
and this `report.md` under `{rel(DEFAULT_OUTPUT)}`.
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--authoring-manifest", type=Path, default=DEFAULT_AUTHORING_MANIFEST)
    parser.add_argument("--lam-checkout", type=Path, default=DEFAULT_LAM_CHECKOUT)
    parser.add_argument("--release-dataset", type=Path, default=DEFAULT_RELEASE_DATASET)
    parser.add_argument("--credential-env-file", type=Path, default=DEFAULT_CREDENTIAL_ENV)
    parser.add_argument("--lam-config", type=Path)
    args = parser.parse_args()

    output = contained(args.output_dir)
    protocol_path = contained(args.protocol)
    authoring_manifest_path = contained(args.authoring_manifest)
    checkout = contained(args.lam_checkout, must_exist=True)
    release_dataset = contained(args.release_dataset, must_exist=True)
    credential_env = contained(args.credential_env_file)
    lam_config = contained(args.lam_config) if args.lam_config else checkout / "config.yaml"
    output.mkdir(parents=True, exist_ok=True)
    generated_at = utc_now()

    authoring = audit_authoring_manifest(authoring_manifest_path)
    protocol = audit_protocol(protocol_path, authoring)
    runtime = audit_runtime(checkout, lam_config, credential_env, protocol["models"])
    release = audit_release(release_dataset)
    execution_gates = {
        "common_evaluator_adapter_available": False,
        "common_repair_adapter_available": False,
        "failed_output_preservation": False,
        "run_id_output_isolation": False,
        "normalized_attempt_telemetry_adapter_available": False,
    }
    environment_blockers, adapter_blockers = readiness_blockers(
        protocol, authoring, runtime, execution_gates
    )
    blockers = environment_blockers + adapter_blockers
    status = "BLOCKED" if environment_blockers else "ADAPTER_REQUIRED"

    intent_tasks = authoring["task_count"] if authoring["valid_json_object"] else 0
    intent_runs = intent_tasks * EXPECTED_RUNS_PER_TASK if protocol["valid_json_object"] else 0
    metrics = {
        key: {"state": "N/R", "value": None}
        for key in (
            "executable",
            "artifact_saved",
            "first_shot",
            "final_success",
            "repair_turns",
            "wall_time",
            "tokens",
            "api_cost",
        )
    }
    manifest = {
        "protocol_id": "nano3d_table1_lam_reliability_preflight_v1",
        "generated_at_utc": generated_at,
        "method": "LAM",
        "mode": "preflight_plus_official_release_telemetry_audit",
        "common_authoring": {
            "evidence_class": "COMMON_PROTOCOL_AUTHORING_INTENT",
            "protocol": protocol,
            "authoring_manifest": authoring,
            "intent_tasks": intent_tasks,
            "intent_runs": intent_runs,
            "attempted_runs": 0,
            "execution_gates": execution_gates,
        },
        "official_release": {
            "evidence_class": release["evidence_class"],
            "release_dataset": rel(release_dataset),
            "release_records": release["release_intent_records"],
            "input_hashes": release["input_hashes"],
        },
        "claim_separation": {
            "release_is_common_rerun": False,
            "release_is_authoring_result": False,
            "release_status_mapped_to_first_shot": False,
            "release_status_mapped_to_final_success": False,
            "paper_values_reused": False,
        },
    }
    runner_path = Path(__file__).resolve()
    table_design = REPO_ROOT / "exp" / "Nano3d.md"
    table_results = REPO_ROOT / "exp" / "Nano3dresults.md"
    project_head = git_value(REPO_ROOT, ["rev-parse", "HEAD"])
    reproduction_command = (
        "OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 "
        "NUMEXPR_NUM_THREADS=1 python exp/scripts/preflight_table1_reliability_lam.py "
        "--output-dir exp/runtime/table1_reliability/lam"
    )
    provenance = {
        "protocol_id": "nano3d_table1_lam_reliability_provenance_v1",
        "generated_at_utc": generated_at,
        "network_accessed": False,
        "api_calls_made": 0,
        "credential_values_recorded": False,
        "workspace_root": str(WORKSPACE_ROOT),
        "project_git_head": project_head,
        "lam_checkout": runtime["checkout"],
        "runtime": runtime,
        "inputs": {
            rel(runner_path): sha256(runner_path),
            rel(table_design): sha256(table_design),
            rel(table_results): sha256(table_results),
            **release["input_hashes"],
        },
        "common_protocol_input": {
            "path": protocol["path"],
            "sha256": protocol["sha256"],
        },
        "authoring_manifest_input": {
            "path": authoring["path"],
            "sha256": authoring["sha256"],
        },
        "reproduction_command": reproduction_command,
    }
    summary = {
        "protocol_id": "nano3d_table1_lam_reliability_preflight_v1",
        "generated_at_utc": generated_at,
        "method": "LAM",
        "status": status,
        "blockers": blockers,
        "environment_blockers": environment_blockers,
        "adapter_blockers": adapter_blockers,
        "execution_gates": execution_gates,
        "common_authoring_rerun": {
            "evidence_class": "COMMON_PROTOCOL_AUTHORING_INTENT",
            "intent_tasks": intent_tasks,
            "intent_runs": intent_runs,
            "attempted_runs": 0,
            "completed_runs": 0,
            "metrics": metrics,
            "claim": "No result; preflight only.",
        },
        "official_release_telemetry": release,
    }

    manifest_path = output / "manifest.json"
    provenance_path = output / "provenance.json"
    summary_path = output / "summary.json"
    report_path = output / "report.md"
    self_check_path = output / "self_check.json"
    write_json(manifest_path, manifest)
    write_json(provenance_path, provenance)
    write_json(summary_path, summary)
    report_path.write_text(report_text(summary, manifest, provenance), encoding="utf-8")

    checks = {
        "authoring_and_release_denominators_separate": intent_tasks != release["release_intent_records"],
        "authoring_attempted_runs_zero": summary["common_authoring_rerun"]["attempted_runs"] == 0,
        "authoring_metrics_all_nr_null": all(
            row["state"] == "N/R" and row["value"] is None for row in metrics.values()
        ),
        "release_rows_expected_3217": release["release_intent_records"] == EXPECTED_RELEASE_ROWS,
        "release_status_conserved": sum(release["status_counts"].values()) == release["release_record_denominator"],
        "release_tier_conserved": sum(release["tier_counts"].values()) == release["release_record_denominator"],
        "release_status_domain_exact": set(release["status_counts"]) == {"PASS", "FAIL"},
        "release_tier_domain_exact": set(release["tier_counts"]) == {"viable", "loads_only", "broken"},
        "release_sources_row_aligned": all(
            release["source_consistency"][key]
            for key in (
                "rel_path_status_tier_exact_manifest_parquet",
                "rel_path_status_tier_exact_manifest_csv",
            )
        ),
        "release_urdf_parse_count_bounded": release["parse_checks"]["urdf_xml_robot"]["numerator"]
        <= release["release_record_denominator"],
        "release_never_mapped_to_common_metrics": all(
            row["state"] == "N/R" and row["value"] is None
            for key, row in release["common_metric_mapping"].items()
            if isinstance(row, dict)
        ),
        "credential_values_not_recorded": not provenance["credential_values_recorded"]
        and not runtime["provider_audit"]["credential_values_recorded"],
        "no_network_or_api": not provenance["network_accessed"] and provenance["api_calls_made"] == 0,
        "status_domain_fail_closed": status in {"BLOCKED", "ADAPTER_REQUIRED"},
        "status_classification_consistent": (
            (status == "BLOCKED" and bool(environment_blockers))
            or (status == "ADAPTER_REQUIRED" and not environment_blockers and bool(adapter_blockers))
        ),
        "adapter_gates_fail_closed": bool(execution_gates)
        and not any(execution_gates.values()),
    }
    self_check = {
        "protocol_id": "nano3d_table1_lam_reliability_self_check_v1",
        "generated_at_utc": generated_at,
        "status": "PASS" if all(checks.values()) else "FAIL",
        "checks": checks,
        "sha256": {
            "manifest.json": sha256(manifest_path),
            "provenance.json": sha256(provenance_path),
            "summary.json": sha256(summary_path),
            "report.md": sha256(report_path),
            rel(runner_path): sha256(runner_path),
        },
        "evidence_files": [rel(path) for path in (manifest_path, provenance_path, summary_path, report_path)],
    }
    write_json(self_check_path, self_check)

    print(
        json.dumps(
            {
                "status": status,
                "self_check": self_check["status"],
                "authoring_intent_tasks": intent_tasks,
                "authoring_attempted_runs": 0,
                "release_records": release["release_intent_records"],
                "blockers": len(blockers),
                "output_dir": str(output),
            },
            sort_keys=True,
        )
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
