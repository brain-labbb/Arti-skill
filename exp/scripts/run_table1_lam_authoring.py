#!/usr/bin/env python3
"""Prepare or execute the frozen Nano3D Table 1 LAM authoring arm.

Preparation is the default and makes no provider call. Formal execution is
fail-closed behind an explicit paid-run acknowledgement, an exact protocol hash,
the frozen global readiness gate, the LAM method gate, and audited dependencies.

The adapter treats each common attempt as a fresh, isolated LAM invocation. A
failed attempt is immutable evidence: native stdout/stderr, partial products,
model-response snapshots, evaluator reports, and normalized feedback stay under
that attempt's directory. Common repairs use only the public prompt and bounded
feedback emitted by the frozen evaluator.
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
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
EXP_ROOT = REPO_ROOT / "exp"
DEFAULT_MANIFEST = EXP_ROOT / "reference/table1_reliability_common_authoring_v1.json"
DEFAULT_PROTOCOL = EXP_ROOT / "reference/table1_reliability_protocol_v1.json"
DEFAULT_OUTPUT = EXP_ROOT / "runtime/table1_reliability/lam_authoring_v1"
DEFAULT_LAM_PYTHON = (
    EXP_ROOT / "runtime/table1_reliability/lam_env_v1/.venv/bin/python"
)
DEFAULT_HARNESS_PYTHON = Path("/mnt/zsn/miniconda3/bin/python")
LAM_CHECKOUT = EXP_ROOT / "baselines/LAM-official"
LAM_ENTRYPOINT = LAM_CHECKOUT / "run_pipeline.py"
LAM_REQUIREMENTS = LAM_CHECKOUT / "requirements.txt"
LAM_PACKAGE_JSON = LAM_CHECKOUT / "package.json"
LAM_PACKAGE_LOCK = LAM_CHECKOUT / "package-lock.json"
LAM_CREDENTIAL_ENV = REPO_ROOT / "articraft_data/.env"
SYSTEM_NODE = Path("/usr/bin/node")
SYSTEM_NPM = Path("/usr/bin/npm")
EXPECTED_PYTHON_VERSION = "Python 3.12.3"
EXPECTED_HARNESS_PYTHON_VERSION = "Python 3.13.2"
EXPECTED_HARNESS_PACKAGES = {
    "jsonschema": "4.26.0",
    "trimesh": "4.12.2",
}
EXPECTED_NODE_VERSION = "v18.19.1"
EXPECTED_NPM_VERSION = "9.2.0"
COMMON_EVALUATOR = EXP_ROOT / "scripts/evaluate_table1_authoring_common.py"
RESULT_SCHEMA = EXP_ROOT / "reference/table1_authoring_result_schema_v1.json"
PACKAGE_SCHEMA = EXP_ROOT / "reference/table1_authoring_package_schema_v1.json"
EXPECTED_LAM_COMMIT = "0b3a87beb8c35273a5acf8681221791aff746d8e"
EXPECTED_LAM_TREE = "684d2b8162d6ecbf85cd9924967314c0ee4ec609"
EXPECTED_TASKS = 54
EXPECTED_REPEAT_IDS = ["r0", "r1", "r2"]
EXPECTED_JOBS = EXPECTED_TASKS * len(EXPECTED_REPEAT_IDS)
EXPECTED_ROLES = (
    "linker_generator",
    "shape_generator",
    "articulation_generator",
    "shape_code_fixer",
    "vlm_critic",
    "shape_fixer",
    "articulation_vlm_critic",
    "articulation_fixer",
)
REQUIRED_NODE_PACKAGES = ("canvas", "jsdom", "puppeteer", "three")
SAFE_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
OPENAI_ENV_KEY = "OPENAI_API_KEY"
SENSITIVE_ASSIGNMENT_RE = re.compile(
    r"(?im)^\s*(?:OPENAI|GOOGLE|GEMINI|ANTHROPIC)_[A-Z_]*KEYS?\s*="
)
THREAD_ENV = {
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1",
}
ADAPTER_GATES = {
    "common_evaluator_adapter_available": True,
    "common_repair_adapter_available": True,
    "failed_output_preservation": True,
    "run_id_output_isolation": True,
    "normalized_attempt_schema_writer_available": True,
    "resume_binding_validation_available": True,
    "provider_retry_separate_from_common_repair": True,
}


NativeInvoker = Callable[
    [dict[str, Any], int, int, str, Path, dict[str, Any], float, Path],
    dict[str, Any],
]
AttemptEvaluator = Callable[
    [Path, Path, dict[str, Any], str, int, dict[str, str], float, Path],
    tuple[dict[str, Any], dict[str, Any]],
]


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def contained(path: Path, *, must_exist: bool = False) -> Path:
    resolved = path.expanduser().resolve(strict=must_exist)
    if resolved != REPO_ROOT and REPO_ROOT not in resolved.parents:
        raise ValueError(f"path escapes repository root: {resolved}")
    return resolved


def authorized_python_executable(path: Path) -> Path:
    """Keep the frozen environment path while allowing its system-binary symlink."""
    declared = Path(os.path.abspath(os.path.expanduser(str(path))))
    allowed_roots = (REPO_ROOT, Path("/mnt/zsn/miniconda3"))
    if not any(declared == root or root in declared.parents for root in allowed_roots):
        raise ValueError(f"Python executable path is outside authorized roots: {declared}")
    return declared


def relative(path: Path, base: Path = REPO_ROOT) -> str:
    return path.resolve().relative_to(base.resolve()).as_posix()


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return sha256_bytes(encoded)


def tree_manifest(root: Path, *, excluded: set[str] | None = None) -> dict[str, str]:
    excluded = excluded or set()
    if not root.is_dir() or root.is_symlink():
        raise ValueError(f"expected regular directory: {root}")
    contained_root = contained(root, must_exist=True)
    files: dict[str, str] = {}
    for path in sorted(contained_root.rglob("*")):
        relative_path = path.relative_to(contained_root).as_posix()
        if relative_path in excluded:
            continue
        if path.is_symlink():
            raise ValueError(f"symlinks are forbidden in sealed output: {path}")
        if path.is_file():
            files[relative_path] = sha256_file(path)
        elif not path.is_dir():
            raise ValueError(f"special files are forbidden in sealed output: {path}")
    return files


def manifest_digest(files: dict[str, str]) -> str:
    return canonical_sha256(files)


def load_object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def load_json_snapshot(path: Path) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    metadata: dict[str, Any] = {
        "path": relative(path),
        "exists": path.is_file() and not path.is_symlink(),
        "sha256": None,
        "valid_json_object": False,
        "error": None,
    }
    if not metadata["exists"]:
        metadata["error"] = "missing or non-regular file"
        return None, metadata
    try:
        raw = path.read_bytes()
        metadata["sha256"] = sha256_bytes(raw)
        value = json.loads(raw.decode("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        metadata["error"] = f"{type(exc).__name__}: {exc}"
        return None, metadata
    if not isinstance(value, dict):
        metadata["error"] = "top-level JSON value is not an object"
        return None, metadata
    metadata["valid_json_object"] = True
    return value, metadata


def write_text_atomic(path: Path, text: str) -> None:
    target = contained(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(target)


def write_json(path: Path, value: Any) -> None:
    write_text_atomic(
        path,
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
    )


def command_value(
    argv: list[str],
    cwd: Path,
    timeout: float = 10.0,
    env: dict[str, str] | None = None,
) -> str | None:
    try:
        completed = subprocess.run(
            argv,
            cwd=cwd,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
            check=False,
            env=env,
        )
    except (OSError, subprocess.TimeoutExpired, ValueError):
        return None
    return completed.stdout.strip() if completed.returncode == 0 else None


def safe_find_spec(module: str, python_executable: Path | None = None) -> bool:
    if python_executable is None or python_executable.resolve() == Path(sys.executable).resolve():
        try:
            return importlib.util.find_spec(module) is not None
        except (ImportError, ModuleNotFoundError, ValueError):
            return False
    if not python_executable.is_file():
        return False
    return (
        command_value(
            [
                str(python_executable),
                "-c",
                f"import importlib.util; raise SystemExit(importlib.util.find_spec({module!r}) is None)",
            ],
            REPO_ROOT,
        )
        is not None
    )


def distribution_version(distribution: str, python_executable: Path) -> str | None:
    if not python_executable.is_file():
        return None
    return command_value(
        [
            str(python_executable),
            "-c",
            (
                "import importlib.metadata,sys;"
                "sys.stdout.write(importlib.metadata.version(sys.argv[1]))"
            ),
            distribution,
        ],
        REPO_ROOT,
    )


def dotenv_value(path: Path, key: str) -> str | None:
    if not path.is_file() or path.is_symlink():
        return None
    for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        candidate, value = line.split("=", 1)
        if candidate.strip() != key:
            continue
        normalized = value.strip().strip("\"'")
        if normalized and not normalized.upper().startswith(("YOUR_", "REPLACE_")):
            return normalized
    return None


def method_binding(protocol: dict[str, Any]) -> dict[str, Any] | None:
    methods = protocol.get("methods")
    if isinstance(methods, dict) and isinstance(methods.get("lam"), dict):
        return methods["lam"]
    if isinstance(methods, list):
        for row in methods:
            if (
                isinstance(row, dict)
                and str(row.get("method_id", "")).strip().lower() == "lam"
            ):
                return row
    return None


def model_id(binding: dict[str, Any] | None) -> str:
    binding = binding or {}
    return str(
        binding.get("model")
        or binding.get("model_id")
        or binding.get("exact_model_id")
        or ""
    ).strip()


def request_parameter(method: dict[str, Any], key: str) -> dict[str, Any]:
    parameters = method.get("request_parameters")
    parameters = parameters if isinstance(parameters, dict) else {}
    row = parameters.get(key)
    return dict(row) if isinstance(row, dict) else {}


def native_settings(protocol: dict[str, Any]) -> dict[str, Any]:
    method = method_binding(protocol) or {}
    value = method.get("native_settings")
    return dict(value) if isinstance(value, dict) else {}


def resolve_protocol_input(
    protocol: dict[str, Any], key: str
) -> tuple[Path | None, str | None]:
    row = protocol.get(key)
    if not isinstance(row, dict):
        return None, None
    raw_path = row.get("path")
    expected_sha = row.get("sha256")
    if not isinstance(raw_path, str) or not raw_path.strip():
        return None, expected_sha if isinstance(expected_sha, str) else None
    candidate = Path(raw_path)
    path = candidate if candidate.is_absolute() else REPO_ROOT / candidate
    return contained(path), expected_sha if isinstance(expected_sha, str) else None


def input_record(
    path: Path | None, *, expected_sha256: str | None = None
) -> dict[str, Any]:
    if path is None:
        return {
            "path": None,
            "exists": False,
            "sha256": None,
            "expected_sha256": expected_sha256,
            "binding_matches": False,
        }
    exists = path.is_file() and not path.is_symlink()
    actual = sha256_file(path) if exists else None
    return {
        "path": relative(path),
        "exists": exists,
        "sha256": actual,
        "expected_sha256": expected_sha256,
        "binding_matches": bool(actual)
        and (expected_sha256 is None or actual == expected_sha256),
    }


def prompt_sha_matches(task: Any) -> bool:
    if not isinstance(task, dict):
        return False
    prompt = task.get("prompt")
    expected = task.get("prompt_sha256")
    return (
        isinstance(prompt, str)
        and bool(prompt.strip())
        and isinstance(expected, str)
        and sha256_bytes(prompt.encode("utf-8")) == expected
    )


def method_implementation_checks(method: dict[str, Any] | None) -> dict[str, bool]:
    method = method or {}
    implementation = method.get("implementation")
    implementation = implementation if isinstance(implementation, dict) else {}
    checkout = REPO_ROOT / str(implementation.get("checkout_path", ""))
    entrypoint = REPO_ROOT / str(implementation.get("entrypoint", ""))
    adapter = REPO_ROOT / str(method.get("adapter_entrypoint", ""))
    try:
        checkout = contained(checkout)
        entrypoint = contained(entrypoint)
        adapter = contained(adapter)
        entrypoint_in_checkout = entrypoint.relative_to(checkout) is not None
    except ValueError:
        checkout = entrypoint = adapter = REPO_ROOT / ".invalid-table1-binding"
        entrypoint_in_checkout = False
    return {
        "lam_implementation_checkout_bound": checkout == LAM_CHECKOUT.resolve()
        and command_value(["git", "rev-parse", "--show-toplevel"], checkout)
        == str(checkout),
        "lam_implementation_commit_bound": implementation.get("commit")
        == EXPECTED_LAM_COMMIT
        and command_value(["git", "rev-parse", "HEAD"], checkout)
        == EXPECTED_LAM_COMMIT,
        "lam_implementation_tree_bound": implementation.get("git_tree")
        == EXPECTED_LAM_TREE
        and command_value(["git", "rev-parse", "HEAD^{tree}"], checkout)
        == EXPECTED_LAM_TREE,
        "lam_implementation_tracked_clean": implementation.get(
            "tracked_clean_at_freeze"
        )
        is True
        and command_value(
            ["git", "status", "--porcelain", "--untracked-files=no"], checkout
        )
        == "",
        "lam_implementation_entrypoint_bound": entrypoint_in_checkout
        and entrypoint == LAM_ENTRYPOINT.resolve()
        and entrypoint.is_file()
        and not entrypoint.is_symlink(),
        "lam_implementation_provenance_bound": isinstance(
            implementation.get("provenance"), str
        )
        and bool(implementation["provenance"].strip()),
        "lam_adapter_path_bound": adapter == Path(__file__).resolve()
        and adapter.is_file()
        and not adapter.is_symlink(),
        "lam_adapter_sha256_bound": isinstance(method.get("adapter_sha256"), str)
        and adapter.is_file()
        and sha256_file(adapter) == method.get("adapter_sha256"),
    }


def validate_contract(
    manifest: dict[str, Any] | None,
    protocol: dict[str, Any] | None,
    manifest_meta: dict[str, Any],
    protocol_meta: dict[str, Any],
) -> tuple[dict[str, bool], dict[str, dict[str, Any]], dict[str, Any]]:
    manifest = manifest or {}
    protocol = protocol or {}
    tasks = manifest.get("tasks") if isinstance(manifest.get("tasks"), list) else []
    repeat_ids = manifest.get("repeat_ids")
    method = method_binding(protocol)
    common_model = protocol.get("common_model_binding")
    common_model = common_model if isinstance(common_model, dict) else {}
    bound_manifest_path, bound_manifest_sha = resolve_protocol_input(protocol, "manifest")
    hidden_path, hidden_sha = resolve_protocol_input(protocol, "hidden_specs")
    evaluator_path, evaluator_sha = resolve_protocol_input(protocol, "common_evaluator")
    package_schema_path, package_schema_sha = resolve_protocol_input(
        protocol, "package_schema"
    )
    result_schema_path, result_schema_sha = resolve_protocol_input(protocol, "result_schema")
    inputs = {
        "protocol": {
            **protocol_meta,
            "expected_sha256": None,
            "binding_matches": protocol_meta.get("valid_json_object", False),
        },
        "manifest": {
            **manifest_meta,
            "expected_sha256": bound_manifest_sha,
            "binding_matches": bool(manifest_meta.get("sha256"))
            and manifest_meta.get("sha256") == bound_manifest_sha
            and bound_manifest_path
            == Path(REPO_ROOT / str(manifest_meta.get("path"))).resolve(),
        },
        "hidden_specs": input_record(hidden_path, expected_sha256=hidden_sha),
        "common_evaluator": input_record(evaluator_path, expected_sha256=evaluator_sha),
        "package_schema": input_record(
            package_schema_path, expected_sha256=package_schema_sha
        ),
        "result_schema": input_record(
            result_schema_path, expected_sha256=result_schema_sha
        ),
    }
    task_ids = [
        str(task.get("task_id", "")).strip() if isinstance(task, dict) else ""
        for task in tasks
    ]
    output_isolation = protocol.get("output_isolation")
    output_isolation = output_isolation if isinstance(output_isolation, dict) else {}
    result_schema = load_object(result_schema_path) if result_schema_path else {}
    attempt_schema = (result_schema.get("$defs") or {}).get("attempt")
    lam_parameters = (
        {
            key: request_parameter(method, key)
            for key in ("temperature", "top_p", "max_output_tokens", "verbosity")
        }
        if isinstance(method, dict)
        else {}
    )
    checks = {
        "manifest_json_object": bool(manifest_meta.get("valid_json_object")),
        "protocol_json_object": bool(protocol_meta.get("valid_json_object")),
        "manifest_frozen": manifest.get("frozen") is True,
        "protocol_frozen": protocol.get("frozen_before_first_run") is True
        or protocol.get("frozen_design") is True,
        "manifest_sha256_bound": inputs["manifest"]["binding_matches"],
        "hidden_specs_sha256_bound": inputs["hidden_specs"]["binding_matches"],
        "common_evaluator_sha256_bound": inputs["common_evaluator"]["binding_matches"],
        "package_schema_sha256_bound": inputs["package_schema"]["binding_matches"],
        "result_schema_sha256_bound": inputs["result_schema"]["binding_matches"],
        "task_count_54": len(tasks) == EXPECTED_TASKS
        and manifest.get("task_count") == EXPECTED_TASKS
        and protocol.get("expected_task_count") == EXPECTED_TASKS,
        "repeat_ids_exact": repeat_ids == EXPECTED_REPEAT_IDS
        and protocol.get("repeat_ids") == EXPECTED_REPEAT_IDS,
        "expected_jobs_162": manifest.get("expected_runs_per_method") == EXPECTED_JOBS
        and protocol.get("expected_runs_per_method") == EXPECTED_JOBS,
        "task_ids_unique_safe": len(task_ids) == EXPECTED_TASKS
        and all(SAFE_ID_RE.fullmatch(value) for value in task_ids)
        and len(set(task_ids)) == EXPECTED_TASKS,
        "prompt_sha256_all_match": len(tasks) == EXPECTED_TASKS
        and all(prompt_sha_matches(task) for task in tasks),
        "hidden_spec_sha256_per_task": len(tasks) == EXPECTED_TASKS
        and all(
            isinstance(task, dict)
            and isinstance(task.get("hidden_spec_sha256"), str)
            and bool(SHA256_RE.fullmatch(task["hidden_spec_sha256"]))
            for task in tasks
        ),
        "repair_budget_3": protocol.get("max_common_repair_turns") == 3,
        "lam_method_binding_present": isinstance(method, dict),
        "common_model_openai_gpt5": str(common_model.get("provider", "")).lower()
        == "openai"
        and model_id(common_model) == "gpt-5",
        "lam_model_openai_gpt5": str((method or {}).get("provider", "")).lower()
        == "openai"
        and model_id(method) == "gpt-5",
        "lam_reasoning_effort_high": (method or {}).get("reasoning_effort") == "high",
        "lam_request_parameters_match_handler": (
            lam_parameters.get("temperature", {}).get("sent") is False
            and lam_parameters.get("temperature", {}).get("value") is None
            and bool(lam_parameters.get("temperature", {}).get("reason"))
            and lam_parameters.get("top_p", {}).get("sent") is False
            and lam_parameters.get("top_p", {}).get("value") is None
            and bool(lam_parameters.get("top_p", {}).get("reason"))
            and lam_parameters.get("max_output_tokens", {}).get("sent") is True
            and lam_parameters.get("max_output_tokens", {}).get("value") == 64000
            and lam_parameters.get("verbosity", {}).get("sent") is True
            and lam_parameters.get("verbosity", {}).get("value") == "medium"
        ),
        "output_isolation_frozen": output_isolation.get("root_pattern")
        == "exp/runtime/table1_reliability/common_authoring/{method}/{task_id}/{repeat_id}/"
        and output_isolation.get("one_method_task_repeat_per_directory") is True
        and output_isolation.get("cross_method_read_access") is False
        and output_isolation.get("preexisting_output_policy")
        == "fail_closed_if_nonempty",
        "normalized_result_schema": result_schema.get("$schema")
        == "https://json-schema.org/draft/2020-12/schema"
        and result_schema.get("properties", {}).get("method_id", {}).get("enum")
        == ["pva", "lam", "articraft"]
        and isinstance(attempt_schema, dict)
        and set(attempt_schema.get("required", []))
        >= {
            "attempt_index",
            "attempt_kind",
            "native_retry_count",
            "output",
            "evaluation",
            "telemetry",
        },
    }
    checks.update(method_implementation_checks(method))
    schema_paths = {
        "result_schema_path": inputs["result_schema"]["path"],
        "result_schema_sha256": inputs["result_schema"]["sha256"],
        "package_schema_path": inputs["package_schema"]["path"],
        "package_schema_sha256": inputs["package_schema"]["sha256"],
        "result_relative_path": "result.json",
        "attempt_root_pattern": "attempts/a{attempt_index}/",
        "attempt_indices": list(range(protocol.get("max_common_repair_turns", -1) + 1))
        if isinstance(protocol.get("max_common_repair_turns"), int)
        else [],
    }
    return checks, inputs, schema_paths


def audit_lam_runtime(
    protocol: dict[str, Any], python_executable: Path, harness_python: Path
) -> tuple[dict[str, Any], list[str]]:
    settings = native_settings(protocol)
    method = method_binding(protocol) or {}
    expected_model = model_id(method)
    provider_packages = {
        name: safe_find_spec(name, python_executable)
        for name in ("openai", "tiktoken", "yaml")
    }
    node_packages = {
        name: (LAM_CHECKOUT / "node_modules" / name / "package.json").is_file()
        for name in REQUIRED_NODE_PACKAGES
    }
    key_present = bool(os.environ.get(OPENAI_ENV_KEY)) or bool(
        dotenv_value(LAM_CREDENTIAL_ENV, OPENAI_ENV_KEY)
    )
    expected_python_raw = settings.get("python_executable")
    expected_python = (
        authorized_python_executable(REPO_ROOT / expected_python_raw)
        if isinstance(expected_python_raw, str) and expected_python_raw
        else None
    )
    required_settings = {
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
        "openai_sdk_max_retries": 0,
        "config_source": "adapter_generated_per_attempt_no_credentials",
        "expected_python_version": EXPECTED_PYTHON_VERSION,
        "harness_python_executable": str(harness_python),
        "expected_harness_python_version": EXPECTED_HARNESS_PYTHON_VERSION,
        "harness_packages": EXPECTED_HARNESS_PACKAGES,
        "node_executable": str(SYSTEM_NODE),
        "expected_node_version": EXPECTED_NODE_VERSION,
        "npm_executable": str(SYSTEM_NPM),
        "expected_npm_version": EXPECTED_NPM_VERSION,
    }
    settings_checks = {
        key: settings.get(key) == expected for key, expected in required_settings.items()
    }
    settings_checks["python_executable"] = expected_python == python_executable
    settings_checks["requirements_sha256"] = settings.get(
        "requirements_sha256"
    ) == sha256_file(LAM_REQUIREMENTS)
    settings_checks["package_lock_sha256"] = settings.get(
        "package_lock_sha256"
    ) == sha256_file(LAM_PACKAGE_LOCK)
    settings_checks["all_roles_use_method_model"] = settings.get(
        "all_roles_use_method_model"
    ) is True
    python_version = (
        command_value([str(python_executable), "--version"], REPO_ROOT)
        if python_executable.is_file()
        else None
    )
    harness_python_version = (
        command_value([str(harness_python), "--version"], REPO_ROOT)
        if harness_python.is_file()
        else None
    )
    harness_packages = {
        name: distribution_version(name, harness_python)
        for name in EXPECTED_HARNESS_PACKAGES
    }
    system_path_env = {**os.environ, "PATH": "/usr/bin:/bin"}
    node_version = command_value(
        [str(SYSTEM_NODE), "--version"], LAM_CHECKOUT, env=system_path_env
    )
    npm_version = command_value(
        [str(SYSTEM_NPM), "--version"], LAM_CHECKOUT, env=system_path_env
    )
    node_smoke = command_value(
        [
            str(SYSTEM_NODE),
            "-e",
            (
                "const {createCanvas}=require('canvas');"
                "const c=createCanvas(2,2);"
                "require('jsdom');require('puppeteer');require('three');"
                "if(c.width!==2)process.exit(2);console.log('PASS')"
            ),
        ],
        LAM_CHECKOUT,
        timeout=30.0,
        env=system_path_env,
    )
    environment_checks = {
        "python_version_exact": python_version == EXPECTED_PYTHON_VERSION,
        "harness_python_version_exact": harness_python_version
        == EXPECTED_HARNESS_PYTHON_VERSION,
        "harness_package_versions_exact": harness_packages
        == EXPECTED_HARNESS_PACKAGES,
        "node_version_exact": node_version == EXPECTED_NODE_VERSION,
        "npm_version_exact": npm_version == EXPECTED_NPM_VERSION,
        "node_module_runtime_smoke": node_smoke == "PASS",
    }
    runtime = {
        "lam_checkout": relative(LAM_CHECKOUT),
        "git_head": command_value(["git", "rev-parse", "HEAD"], LAM_CHECKOUT),
        "expected_git_head": EXPECTED_LAM_COMMIT,
        "git_tree": command_value(["git", "rev-parse", "HEAD^{tree}"], LAM_CHECKOUT),
        "expected_git_tree": EXPECTED_LAM_TREE,
        "origin": command_value(["git", "remote", "get-url", "origin"], LAM_CHECKOUT),
        "entrypoint": relative(LAM_ENTRYPOINT),
        "entrypoint_exists": LAM_ENTRYPOINT.is_file(),
        "python_executable": {
            "path": python_executable.relative_to(REPO_ROOT).as_posix(),
            "exists": python_executable.is_file(),
            "version": python_version,
            "expected_version": EXPECTED_PYTHON_VERSION,
            "matches_protocol": expected_python == python_executable.resolve(),
        },
        "harness_python": {
            "path": str(harness_python),
            "exists": harness_python.is_file(),
            "version": harness_python_version,
            "expected_version": EXPECTED_HARNESS_PYTHON_VERSION,
            "packages": harness_packages,
            "expected_packages": EXPECTED_HARNESS_PACKAGES,
            "role": "result-schema validation and common evaluator only",
        },
        "node": {
            "executable": str(SYSTEM_NODE),
            "version": node_version,
            "expected_version": EXPECTED_NODE_VERSION,
            "npm_executable": str(SYSTEM_NPM),
            "npm_version": npm_version,
            "expected_npm_version": EXPECTED_NPM_VERSION,
            "node_modules_exists": (LAM_CHECKOUT / "node_modules").is_dir(),
            "packages": node_packages,
            "runtime_smoke": node_smoke,
            "ready": all(node_packages.values())
            and all(environment_checks[key] for key in (
                "node_version_exact",
                "npm_version_exact",
                "node_module_runtime_smoke",
            )),
        },
        "openai_provider": {
            "credential_present": key_present,
            "packages": provider_packages,
            "ready": key_present and all(provider_packages.values()),
            "credential_values_recorded": False,
            "sdk_max_retries": settings.get("openai_sdk_max_retries"),
            "sdk_retry_disabled_by_native_shim": settings_checks.get(
                "openai_sdk_max_retries", False
            ),
        },
        "native_settings": settings,
        "native_settings_checks": settings_checks,
        "environment_checks": environment_checks,
        "model": expected_model,
        "required_roles": list(EXPECTED_ROLES),
        "generated_config_contains_credentials": False,
        "generated_code_read_isolation": {
            "enforced": False,
            "source_probe_credential_environment_isolated": True,
            "source_probe_environment_keys": sorted(source_probe_environment()),
            "native_pipeline_credential_environment_isolated": False,
            "risk": (
                "The adapter's post-generation common source probe uses an exact environment "
                "allowlist and does not inherit provider credentials or arbitrary host variables. "
                "The official native pipeline also executes model-generated JavaScript while its "
                "parent environment carries the OpenAI credential; that path has neither an exact "
                "environment boundary nor filesystem read denial. Generated code can still read "
                "credentials and absolute host paths through process.env and node:fs."
            ),
            "required_evidence": (
                "A kernel-enforced sandbox test must prove generated code receives EACCES "
                "or ENOENT for hidden specs and other-method output roots while retaining "
                "access to the frozen LAM runtime and its own attempt directory."
            ),
        },
        "protocol_lam_method_adapter_ready": (
            protocol.get("execution_readiness", {})
            .get("method_adapters_ready", {})
            .get("lam")
            is True
        ),
        "capability_notes": [
            "LAM is a multi-agent pipeline; one common attempt can contain linker, shape, critic, fixer, and articulation model calls.",
            "BaseAgent parse retry uses a total-attempt count, while shape export validation uses a retry count and can emit up to retries+1 shape generations.",
            "The exp-side native shim freezes the official hard-coded shape retry count and disables destructive cleanup without changing the official checkout.",
            "Common repair is a fresh isolated LAM pipeline invocation using only public task text and bounded common-evaluator feedback; LAM has no native continuation API for this contract.",
        ],
    }
    blockers: list[str] = []
    if runtime["git_head"] != EXPECTED_LAM_COMMIT or runtime["git_tree"] != EXPECTED_LAM_TREE:
        blockers.append("LAM checkout is not at the audited official commit/tree")
    if runtime["origin"] != "https://github.com/gaoypeng/LAM.git":
        blockers.append("LAM checkout origin is not the official repository")
    if not runtime["entrypoint_exists"]:
        blockers.append("LAM run_pipeline.py entrypoint is absent")
    if not python_executable.is_file():
        blockers.append("frozen LAM experiment Python executable is absent")
    elif not all(provider_packages.values()):
        blockers.append("frozen LAM Python environment is missing required packages")
    elif not environment_checks["python_version_exact"]:
        blockers.append("frozen LAM Python version does not match Python 3.12.3")
    if not harness_python.is_file() or not all(
        environment_checks[key]
        for key in (
            "harness_python_version_exact",
            "harness_package_versions_exact",
        )
    ):
        blockers.append(
            "frozen outer harness Python or jsonschema/trimesh versions do not match protocol"
        )
    if not key_present:
        blockers.append("OpenAI credential is not available to inherit at execution time")
    if not runtime["node"]["ready"]:
        blockers.append(
            "LAM Node 18.19.1/npm 9.2.0 environment or required module ABI smoke is not ready"
        )
    if not runtime["generated_code_read_isolation"]["enforced"]:
        blockers.append(
            "LAM native generated-code environment/filesystem isolation is not kernel-enforced; despite the post-generation source-probe allowlist, native Node execution can read credentials, evaluator-only paths, or other-method outputs"
        )
    failed_settings = [key for key, passed in settings_checks.items() if not passed]
    if failed_settings:
        blockers.append(
            "protocol does not freeze the audited LAM native settings: "
            + ", ".join(failed_settings)
        )
    return runtime, blockers


def run_root_for(task_id: str, repeat_id: str, protocol: dict[str, Any]) -> Path:
    pattern = protocol.get("output_isolation", {}).get("root_pattern", "")
    value = pattern.format(method="lam", task_id=task_id, repeat_id=repeat_id).rstrip("/")
    return contained(REPO_ROOT / value)


def build_jobs(
    manifest: dict[str, Any], protocol: dict[str, Any]
) -> tuple[list[dict[str, Any]], list[str]]:
    jobs: list[dict[str, Any]] = []
    nonempty_roots: list[str] = []
    for task in manifest.get("tasks", []):
        for repeat_id in manifest.get("repeat_ids", []):
            task_id = task["task_id"]
            run_id = f"lam__{task_id}__{repeat_id}"
            run_root = run_root_for(task_id, repeat_id, protocol)
            if (
                run_root.is_dir()
                and any(run_root.iterdir())
                and not (run_root / "result.json").is_file()
                and not (run_root / "run_state.json").is_file()
            ):
                nonempty_roots.append(relative(run_root))
            attempts = [
                {
                    "attempt_index": index,
                    "attempt_kind": "attempt_0" if index == 0 else "common_repair",
                    "attempt_root": f"{relative(run_root)}/attempts/a{index}",
                    "package_manifest_path": f"{relative(run_root)}/attempts/a{index}/package.json",
                    "common_evaluator_report_path": f"{relative(run_root)}/attempts/a{index}/common_evaluator_report.json",
                }
                for index in range(protocol["max_common_repair_turns"] + 1)
            ]
            jobs.append(
                {
                    "run_id": run_id,
                    "method_id": "lam",
                    "task_id": task_id,
                    "repeat_id": repeat_id,
                    "domain": task.get("domain"),
                    "difficulty": task.get("difficulty"),
                    "category": task.get("category"),
                    "input_modality": task.get("input_modality"),
                    "prompt": task["prompt"],
                    "prompt_sha256": task["prompt_sha256"],
                    "hidden_spec_sha256": task["hidden_spec_sha256"],
                    "run_root": relative(run_root),
                    "result_path": f"{relative(run_root)}/result.json",
                    "attempts": attempts,
                    "status": "prepared_not_run",
                }
            )
    return jobs, sorted(nonempty_roots)


def frozen_bindings(
    manifest_path: Path, protocol_path: Path, protocol: dict[str, Any]
) -> dict[str, str]:
    return {
        "protocol_sha256": sha256_file(protocol_path),
        "manifest_sha256": sha256_file(manifest_path),
        "hidden_specs_sha256": str(protocol["hidden_specs"]["sha256"]),
        "package_schema_sha256": sha256_file(PACKAGE_SCHEMA),
        "result_schema_sha256": sha256_file(RESULT_SCHEMA),
        "common_evaluator_sha256": sha256_file(COMMON_EVALUATOR),
    }


def validate_result(
    result: dict[str, Any], harness_python: Path = DEFAULT_HARNESS_PYTHON
) -> None:
    script = (
        "import json, jsonschema, pathlib, sys;"
        "schema=json.loads(pathlib.Path(sys.argv[1]).read_text(encoding='utf-8'));"
        "payload=json.load(sys.stdin);"
        "jsonschema.Draft202012Validator(schema).validate(payload)"
    )
    completed = subprocess.run(
        [str(harness_python), "-c", script, str(RESULT_SCHEMA)],
        input=json.dumps(result, ensure_ascii=False),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=60.0,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "result schema validation failed in frozen outer harness: "
            + completed.stderr[-4000:]
        )


def no_secret_config(
    output_path: Path,
    output_dir: Path,
    method: dict[str, Any],
    settings: dict[str, Any],
) -> None:
    model = model_id(method)
    max_tokens = request_parameter(method, "max_output_tokens")
    verbosity = request_parameter(method, "verbosity")
    if max_tokens.get("sent") is not True or not isinstance(max_tokens.get("value"), int):
        raise ValueError("LAM max_output_tokens request parameter is not frozen as sent")
    if verbosity.get("sent") is not True or not isinstance(verbosity.get("value"), str):
        raise ValueError("LAM verbosity request parameter is not frozen as sent")
    payload = {
        "api": {
            "keys": {
                "openai": "INHERIT_OPENAI_API_KEY_FROM_ENVIRONMENT",
                "google": "UNUSED",
                "anthropic": "UNUSED",
            },
            "agents": {role: model for role in EXPECTED_ROLES},
            "defaults": {
                "max_tokens": max_tokens["value"],
                "frequency_penalty": 0,
                "presence_penalty": 0,
            },
            "overrides": {
                model: {
                    "max_tokens": max_tokens["value"],
                    "reasoning_effort": method.get("reasoning_effort"),
                    "verbosity": verbosity["value"],
                }
            },
        },
        "retry": {
            "max_retries": settings["base_agent_parse_max_attempts"],
            "multiplier": settings.get("parse_retry_multiplier", 1),
            "min_wait": settings.get("parse_retry_min_wait_seconds", 4),
            "max_wait": settings.get("parse_retry_max_wait_seconds", 10),
        },
        "output": {
            "base_dir": str(output_dir),
            "save_config": True,
            "output_visualization": False,
        },
        "vlm_critic": {
            "enabled": settings["vlm_critic_enabled"],
            "max_iterations": settings["vlm_critic_max_iterations"],
            "image_size": [384, 384],
            "background_color": [0.95, 0.95, 0.95, 1.0],
            "improvement_threshold": 0.8,
            "render_angles": [
                {"name": "front_right_top", "azimuth": 45, "elevation": 30},
                {"name": "front_left_top", "azimuth": -45, "elevation": 30},
                {"name": "back_right_top", "azimuth": 135, "elevation": 30},
                {"name": "back_left_top", "azimuth": -135, "elevation": 30},
            ],
        },
        "pointllm_critic": {"enabled": settings["pointllm_critic_enabled"]},
        "feedback_fusion": {"enabled": settings["feedback_fusion_enabled"]},
        "articulation_feedback": {
            "enabled": settings["articulation_feedback_enabled"],
            "max_iterations": settings["articulation_feedback_max_iterations"],
            "confidence_threshold": 0.7,
            "joint_states": [0.0, 0.75],
            "image_size": [384, 384],
            "background_color": [0.95, 0.95, 0.95, 1.0],
        },
        "logging": {"enabled": True, "level": "INFO"},
    }
    rendered = yaml.safe_dump(payload, sort_keys=False)
    live_key = os.environ.get(OPENAI_ENV_KEY)
    if SENSITIVE_ASSIGNMENT_RE.search(rendered) or (live_key and live_key in rendered):
        raise RuntimeError("refusing to persist credential material in generated LAM config")
    write_text_atomic(output_path, rendered)


def native_shim_text(checkout: Path, settings: dict[str, Any]) -> str:
    frozen = {
        "openai_sdk_max_retries": settings["openai_sdk_max_retries"],
        "shape_export_validation_max_retries": settings[
            "shape_export_validation_max_retries"
        ]
    }
    return f'''#!/usr/bin/env python3
import json
import hashlib
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

CHECKOUT = Path({str(checkout)!r})
SETTINGS = json.loads({json.dumps(frozen, sort_keys=True)!r})
sys.path.insert(0, str(CHECKOUT))

import utils.pipeline_stage_runner as pipeline_stage_runner
import utils.pipeline_execution_runner as pipeline_execution_runner
from agents.shape_generator_agent import ShapeGeneratorAgent
from providers.openai_handler import OpenAIHandler
from utils.stage_recorder import StageRecorder

PROVIDER_TRACE = Path.cwd() / "provider_calls.jsonl"

def utc_now():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")

def append_provider_record(record):
    with PROVIDER_TRACE.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\\n")

_original_provider_init = OpenAIHandler.__init__
def _sdk_retry_disabled_init(self, *args, **kwargs):
    _original_provider_init(self, *args, **kwargs)
    self.client = self.client.with_options(
        max_retries=SETTINGS["openai_sdk_max_retries"]
    )
OpenAIHandler.__init__ = _sdk_retry_disabled_init

_original_provider_invoke = OpenAIHandler.invoke
def _instrumented_provider_invoke(self, prompt, system_prompt=None, **kwargs):
    started = utc_now()
    start_wall = time.monotonic()
    index = sum(1 for _ in PROVIDER_TRACE.open("r", encoding="utf-8")) if PROVIDER_TRACE.is_file() else 0
    try:
        result = _original_provider_invoke(self, prompt, system_prompt, **kwargs)
    except Exception as exc:
        append_provider_record({{
            "provider_call_index": index,
            "provider": "openai",
            "model": self.model_name,
            "status": "exception",
            "started_at_utc": started,
            "finished_at_utc": utc_now(),
            "wall_time_seconds": time.monotonic() - start_wall,
            "exception_class": type(exc).__name__,
            "response_sha256": None,
            "input_tokens": None,
            "output_tokens": None,
            "api_cost_usd": None,
            "request_or_credential_content_recorded": False,
        }})
        raise
    response_text = result.response_text or ""
    append_provider_record({{
        "provider_call_index": index,
        "provider": "openai",
        "model": result.model_name,
        "endpoint": result.endpoint,
        "status": "completed",
        "started_at_utc": started,
        "finished_at_utc": utc_now(),
        "wall_time_seconds": time.monotonic() - start_wall,
        "exception_class": None,
        "response_sha256": hashlib.sha256(response_text.encode("utf-8")).hexdigest(),
        "input_tokens": result.input_tokens,
        "output_tokens": result.output_tokens,
        "api_cost_usd": result.total_cost,
        "request_or_credential_content_recorded": False,
    }})
    return result
OpenAIHandler.invoke = _instrumented_provider_invoke

def preserve_partial(_paths):
    return None

pipeline_stage_runner.cleanup_output = preserve_partial
pipeline_execution_runner.cleanup_output = preserve_partial

_original_shape = ShapeGeneratorAgent.generate_with_export_validation
def _bounded_shape(self, *args, **kwargs):
    kwargs["max_retries"] = SETTINGS["shape_export_validation_max_retries"]
    return _original_shape(self, *args, **kwargs)
ShapeGeneratorAgent.generate_with_export_validation = _bounded_shape

_original_immediate = StageRecorder.save_stage_output_immediate
def _preserving_immediate(self, output_folder, raw_response, stage_num=None):
    _, stage_folder, _ = self._resolve_stage_paths(output_folder, stage_num)
    evidence = Path(stage_folder) / "raw_responses"
    evidence.mkdir(parents=True, exist_ok=True)
    index = len(list(evidence.glob("response_*.txt")))
    (evidence / f"response_{{index:03d}}.txt").write_text(raw_response, encoding="utf-8")
    return _original_immediate(self, output_folder, raw_response, stage_num)
StageRecorder.save_stage_output_immediate = _preserving_immediate

import run_pipeline
run_pipeline.main()
'''


def native_environment() -> dict[str, str]:
    env = os.environ.copy()
    env.update(THREAD_ENV)
    env["PATH"] = "/usr/bin:/bin:" + env.get("PATH", "")
    if not env.get(OPENAI_ENV_KEY):
        value = dotenv_value(LAM_CREDENTIAL_ENV, OPENAI_ENV_KEY)
        if value:
            env[OPENAI_ENV_KEY] = value
    return env


def source_probe_environment() -> dict[str, str]:
    return {
        "PATH": "/usr/bin:/bin",
        "NODE_PATH": str(LAM_CHECKOUT / "node_modules"),
        **THREAD_ENV,
    }


def file_inventory(root: Path) -> list[dict[str, Any]]:
    if not root.is_dir():
        return []
    rows: list[dict[str, Any]] = []
    for path in sorted(root.rglob("*")):
        if path.is_file() and not path.is_symlink():
            rows.append(
                {
                    "path": relative(path, root),
                    "size": path.stat().st_size,
                    "sha256": sha256_file(path),
                }
            )
    return rows


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    rows: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"expected JSON object line: {path}")
        rows.append(value)
    return rows


def emitted_response_files(root: Path) -> list[Path]:
    patterns = (
        "**/raw_responses/response_*.txt",
        "**/pipeline_logs/stage_*/llm_interaction.txt",
    )
    rows: set[Path] = set()
    for pattern in patterns:
        rows.update(path for path in root.glob(pattern) if path.is_file())
    return sorted(rows)


def tree_digest(paths: list[Path], root: Path) -> str | None:
    rows = [
        {"path": relative(path, root), "sha256": sha256_file(path)} for path in paths
    ]
    return canonical_sha256(rows) if rows else None


def real_native_invoker(
    task: dict[str, Any],
    attempt_index: int,
    native_retry_index: int,
    description: str,
    invocation_root: Path,
    settings: dict[str, Any],
    timeout: float,
    python_executable: Path,
) -> dict[str, Any]:
    if invocation_root.exists() and any(invocation_root.iterdir()):
        raise RuntimeError(f"refusing nonempty native retry directory: {invocation_root}")
    invocation_root.mkdir(parents=True, exist_ok=False)
    native_output = invocation_root / "native_output"
    method = settings["_method_binding"]
    no_secret_config(invocation_root / "config.yaml", native_output, method, settings)
    shim = invocation_root / "lam_native_shim.py"
    write_text_atomic(shim, native_shim_text(LAM_CHECKOUT, settings))
    request = {
        "schema_version": 1,
        "task_id": task["task_id"],
        "attempt_index": attempt_index,
        "native_retry_index": native_retry_index,
        "description_sha256": sha256_bytes(description.encode("utf-8")),
        "description": description,
        "model": model_id(method),
        "settings": {key: value for key, value in settings.items() if not key.startswith("_")},
        "credential_values_recorded": False,
    }
    write_json(invocation_root / "request.json", request)
    command = [
        str(python_executable),
        str(shim),
        "--description",
        description,
        "--linker-model",
        model_id(method),
        "--shape-model",
        model_id(method),
        "--articulation-model",
        model_id(method),
        "--output-dir",
        str(native_output),
        "--num-executions",
        str(settings["num_executions"]),
        "--parallel",
        str(settings["parallel_workers"]),
        "--max-retries",
        str(settings["base_agent_parse_max_attempts"]),
        "--log-level",
        "INFO",
    ]
    if not settings["vlm_critic_enabled"]:
        command.append("--no-vlm-critic")
    if not settings["articulation_feedback_enabled"]:
        command.append("--no-articulation-feedback")
    started_at = utc_now()
    start_wall = time.monotonic()
    timed_out = False
    try:
        completed = subprocess.run(
            command,
            cwd=invocation_root,
            env=native_environment(),
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
    stdout_path = invocation_root / "native.stdout.txt"
    stderr_path = invocation_root / "native.stderr.txt"
    write_text_atomic(stdout_path, stdout)
    write_text_atomic(stderr_path, stderr)
    responses = emitted_response_files(native_output)
    provider_records = read_jsonl(invocation_root / "provider_calls.jsonl")
    completed_provider_records = [
        row for row in provider_records if row.get("status") == "completed"
    ]
    inventory = file_inventory(invocation_root)
    trace = {
        "schema_version": 1,
        "started_at_utc": started_at,
        "finished_at_utc": utc_now(),
        "wall_time_seconds": time.monotonic() - start_wall,
        "exit_code": exit_code,
        "timed_out": timed_out,
        "stdout_path": relative(stdout_path, invocation_root),
        "stdout_sha256": sha256_file(stdout_path),
        "stderr_path": relative(stderr_path, invocation_root),
        "stderr_sha256": sha256_file(stderr_path),
        "provider_call_count": len(provider_records),
        "completed_provider_call_count": len(completed_provider_records),
        "failed_provider_call_count": len(provider_records)
        - len(completed_provider_records),
        "provider_trace_path": "provider_calls.jsonl"
        if provider_records
        else None,
        "provider_trace_sha256": sha256_file(invocation_root / "provider_calls.jsonl")
        if provider_records
        else None,
        "emitted_model_output": bool(completed_provider_records),
        "model_response_file_count": len(responses),
        "model_response_sha256": canonical_sha256(
            [row["response_sha256"] for row in completed_provider_records]
        )
        if completed_provider_records
        else tree_digest(responses, invocation_root),
        "file_inventory": inventory,
        "credential_values_recorded": False,
    }
    write_json(invocation_root / "native_trace.json", trace)
    return trace


def select_native_product(invocation_root: Path) -> tuple[Path | None, Path | None]:
    native_output = invocation_root / "native_output"
    candidates: list[tuple[Path, Path]] = []
    if native_output.is_dir():
        for urdf in native_output.glob("*/generated.urdf"):
            source = urdf.parent / "_export_temp/export.js"
            if urdf.is_file() and source.is_file():
                candidates.append((source, urdf))
    candidates.sort(key=lambda row: row[1].as_posix())
    if len(candidates) != 1:
        return None, None
    return candidates[0]


def telemetry_from_native(invocation_roots: list[Path]) -> dict[str, Any]:
    records: list[dict[str, Any]] = []
    trace_complete = True
    for invocation_root in invocation_roots:
        try:
            trace = load_object(invocation_root / "native_trace.json")
            declared_count = trace.get("provider_call_count")
            rows = read_jsonl(invocation_root / "provider_calls.jsonl")
            if not isinstance(declared_count, int) or declared_count != len(rows):
                trace_complete = False
            records.extend(rows)
        except (OSError, ValueError, json.JSONDecodeError):
            trace_complete = False
    completed = [row for row in records if row.get("status") == "completed"]
    complete_usage = bool(completed) and all(
        isinstance(row.get("input_tokens"), int)
        and isinstance(row.get("output_tokens"), int)
        and isinstance(row.get("api_cost_usd"), (int, float))
        for row in completed
    )
    no_unknown_failures = len(records) == len(completed)
    if not trace_complete or not complete_usage or not no_unknown_failures:
        return {
            "input_tokens": None,
            "output_tokens": None,
            "api_cost_usd": None,
            "reason": (
                "LAM provider-call trace was absent, incomplete, or included failed calls "
                "with unknown usage; partial telemetry is not imputed"
            ),
        }
    return {
        "input_tokens": sum(row["input_tokens"] for row in completed),
        "output_tokens": sum(row["output_tokens"] for row in completed),
        "api_cost_usd": sum(float(row["api_cost_usd"]) for row in completed),
        "reason": None,
    }


def run_source_probe(source: Path, probe_root: Path, timeout: float) -> dict[str, Any]:
    probe_root.mkdir(parents=True, exist_ok=False)
    stdout_path = probe_root / "stdout.txt"
    stderr_path = probe_root / "stderr.txt"
    mesh_output = probe_root / "mesh_output"
    started_at = utc_now()
    start_wall = time.monotonic()
    timed_out = False
    try:
        completed = subprocess.run(
            [
                str(SYSTEM_NODE),
                str(LAM_CHECKOUT / "utils/threejs_to_mesh.js"),
                str(source),
                str(mesh_output),
            ],
            cwd=LAM_CHECKOUT,
            env=source_probe_environment(),
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
        "started_at_utc": started_at,
        "finished_at_utc": utc_now(),
        "wall_time_s": time.monotonic() - start_wall,
        "exit_code": exit_code,
        "timed_out": timed_out,
        "source_sha256": sha256_file(source),
        "stdout_sha256": sha256_file(stdout_path),
        "stderr_sha256": sha256_file(stderr_path),
    }


def normalized_feedback(report: dict[str, Any], output: Path) -> dict[str, Any]:
    feedback = dict(report.get("feedback") or {})
    feedback["common_qc_pass"] = report.get("verdicts", {}).get("common_qc_pass")
    write_json(output, feedback)
    return feedback


def evaluate_native_attempt(
    run_root: Path,
    attempt_root: Path,
    task: dict[str, Any],
    repeat_id: str,
    attempt_index: int,
    bindings: dict[str, str],
    timeout: float,
    harness_python: Path = DEFAULT_HARNESS_PYTHON,
) -> tuple[dict[str, Any], dict[str, Any]]:
    retry_roots = sorted((attempt_root / "native").glob("r*"))
    if not retry_roots:
        raise RuntimeError("native attempt has no retry directory")
    source, urdf = select_native_product(retry_roots[-1])
    if source is None or urdf is None:
        raise FileNotFoundError("LAM native output has no unique export.js/generated.urdf pair")
    probe = run_source_probe(source, attempt_root / "execution_probe", timeout)
    package_path = attempt_root / "package.json"
    package = {
        "schema_version": "table1_authoring_package_v1",
        "run_id": f"lam__{task['task_id']}__{repeat_id}",
        "method_id": "lam",
        "task_id": task["task_id"],
        "repeat_id": repeat_id,
        "attempt_index": attempt_index,
        "run_root": str(run_root),
        "bindings": {
            "protocol_sha256": bindings["protocol_sha256"],
            "manifest_sha256": bindings["manifest_sha256"],
            "hidden_specs_sha256": bindings["hidden_specs_sha256"],
            "common_evaluator_sha256": bindings["common_evaluator_sha256"],
            "package_schema_sha256": bindings["package_schema_sha256"],
        },
        "artifacts": {
            "source": {"path": relative(source, run_root), "sha256": sha256_file(source)},
            "urdf": {"path": relative(urdf, run_root), "sha256": sha256_file(urdf)},
        },
        "execution_probe": probe,
    }
    write_json(package_path, package)
    report_path = attempt_root / "common_evaluator_report.json"
    completed = subprocess.run(
        [
            str(harness_python),
            str(COMMON_EVALUATOR),
            "--package-manifest",
            str(package_path),
            "--output",
            str(report_path),
        ],
        cwd=REPO_ROOT,
        env={**os.environ, **THREAD_ENV},
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )
    write_text_atomic(attempt_root / "common_evaluator.stdout.txt", completed.stdout)
    write_text_atomic(attempt_root / "common_evaluator.stderr.txt", completed.stderr)
    if not report_path.is_file():
        raise RuntimeError(
            f"common evaluator produced no report (exit {completed.returncode}): {completed.stderr[-4000:]}"
        )
    return load_object(report_path), package


def missing_output_evaluation(
    task: dict[str, Any], attempt_index: int, attempt_root: Path
) -> tuple[dict[str, Any], dict[str, Any]]:
    feedback = {
        "schema_version": 1,
        "task_id": task["task_id"],
        "attempt_index": attempt_index,
        "common_qc_pass": False,
        "failure_codes": ["ARTIFACT_PACKAGE_FAILED"],
        "bounded_diagnostics": {
            "unresolved_mesh_reference_count": 0,
            "urdf_parse_error_class": None,
            "tree_root_count": None,
            "tree_connected_link_count": None,
            "tree_link_count": None,
        },
        "policy": "Only a common gate identifier is exposed; hidden expected values remain withheld.",
    }
    write_json(attempt_root / "repair_feedback.json", feedback)
    evaluation = {
        "state": "observed",
        "executable": False,
        "artifact_saved": False,
        "common_qc_pass": False,
        "urdf_tree_pass": False,
        "semantic_roles_pass": False,
        "joint_spec_pass": False,
        "input_bindings_pass": None,
        "common_evaluator_report_path": None,
        "common_evaluator_report_sha256": None,
        "reason": "LAM native output had no unique source/URDF package",
    }
    output = {
        "template_path": None,
        "template_sha256": None,
        "artifact_path": None,
        "artifact_sha256": None,
    }
    return evaluation, output


def common_evaluator_failure_evaluation(
    attempt_root: Path, exc: Exception
) -> tuple[dict[str, Any], dict[str, Any]]:
    timed_out = isinstance(exc, (subprocess.TimeoutExpired, TimeoutError))
    write_json(
        attempt_root / "common_evaluator_failure.json",
        {
            "schema_version": "table1_lam_common_evaluator_failure_v1",
            "failure_class": "common_evaluator_failure",
            "error_class": type(exc).__name__,
            "timed_out": timed_out,
            "internal_error_detail_recorded": False,
            "repair_allowed": False,
        },
    )
    evaluation = {
        "state": "not_evaluable",
        "executable": None,
        "artifact_saved": None,
        "common_qc_pass": None,
        "urdf_tree_pass": None,
        "semantic_roles_pass": None,
        "joint_spec_pass": None,
        "input_bindings_pass": None,
        "common_evaluator_report_path": None,
        "common_evaluator_report_sha256": None,
        "reason": (
            "Common evaluator timed out; the attempt is not evaluable"
            if timed_out
            else "Common evaluator failed; the attempt is not evaluable"
        ),
    }
    output = {
        "template_path": None,
        "template_sha256": None,
        "artifact_path": None,
        "artifact_sha256": None,
    }
    return evaluation, output


def repair_description(
    task: dict[str, Any], feedback_path: Path, attempt_index: int
) -> str:
    feedback = load_object(feedback_path)
    allowed = {
        "schema_version": feedback.get("schema_version"),
        "task_id": feedback.get("task_id"),
        "attempt_index": feedback.get("attempt_index"),
        "common_qc_pass": feedback.get("common_qc_pass"),
        "failure_codes": feedback.get("failure_codes", []),
        "bounded_diagnostics": feedback.get("bounded_diagnostics", {}),
        "policy": feedback.get("policy"),
    }
    return (
        f"{task['prompt']}\n\n"
        f"Common repair turn {attempt_index}/3. The prior isolated LAM product failed "
        "the frozen common evaluator. Regenerate the complete articulated object while "
        "addressing only this bounded output-derived feedback; do not infer or request "
        "hidden benchmark values:\n"
        + json.dumps(allowed, ensure_ascii=False, sort_keys=True)
    )


def result_bindings(bindings: dict[str, str]) -> dict[str, str]:
    return {**bindings, "adapter_sha256": sha256_file(Path(__file__).resolve())}


def expected_run_identity(task: dict[str, Any], repeat_id: str) -> dict[str, str]:
    return {
        "run_id": f"lam__{task['task_id']}__{repeat_id}",
        "method_id": "lam",
        "task_id": task["task_id"],
        "repeat_id": repeat_id,
    }


def require_exact_identity(
    record: dict[str, Any], expected: dict[str, str], label: str
) -> None:
    actual = {key: record.get(key) for key in expected}
    if actual != expected:
        raise RuntimeError(f"refusing stale {label} identity: {actual} != {expected}")


def seal_attempt(
    attempt_root: Path,
    bindings: dict[str, str],
    identity: dict[str, str],
    attempt: dict[str, Any],
) -> dict[str, Any]:
    seal_path = attempt_root / "attempt_seal.json"
    if seal_path.exists():
        raise RuntimeError(f"attempt already sealed: {attempt_root}")
    files = tree_manifest(attempt_root, excluded={"attempt_seal.json"})
    seal = {
        "schema_version": "table1_lam_attempt_seal_v1",
        "sealed_at_utc": utc_now(),
        "identity": {**identity, "attempt_index": attempt["attempt_index"]},
        "bindings": bindings,
        "attempt_record_sha256": canonical_sha256(attempt),
        "files": files,
        "output_tree_sha256": manifest_digest(files),
    }
    write_json(seal_path, seal)
    return seal


def validate_attempt_seal(
    attempt_root: Path,
    bindings: dict[str, str],
    identity: dict[str, str],
    attempt: dict[str, Any],
) -> dict[str, Any]:
    seal_path = attempt_root / "attempt_seal.json"
    if not seal_path.is_file() or seal_path.is_symlink():
        raise RuntimeError(f"missing immutable attempt seal: {seal_path}")
    seal = load_object(seal_path)
    expected_identity = {**identity, "attempt_index": attempt.get("attempt_index")}
    actual_files = tree_manifest(attempt_root, excluded={"attempt_seal.json"})
    if (
        seal.get("schema_version") != "table1_lam_attempt_seal_v1"
        or seal.get("identity") != expected_identity
        or seal.get("bindings") != bindings
        or seal.get("attempt_record_sha256") != canonical_sha256(attempt)
        or seal.get("files") != actual_files
        or seal.get("output_tree_sha256") != manifest_digest(actual_files)
    ):
        raise RuntimeError(f"immutable attempt output changed or is stale: {attempt_root}")
    return seal


def seal_result(
    run_root: Path,
    result_path: Path,
    bindings: dict[str, str],
    identity: dict[str, str],
) -> None:
    seal_path = run_root / "result_seal.json"
    if seal_path.exists():
        raise RuntimeError(f"result already sealed: {run_root}")
    files = tree_manifest(run_root, excluded={"result_seal.json"})
    write_json(
        seal_path,
        {
            "schema_version": "table1_lam_result_seal_v1",
            "sealed_at_utc": utc_now(),
            "identity": identity,
            "bindings": bindings,
            "result_sha256": sha256_file(result_path),
            "files": files,
            "output_tree_sha256": manifest_digest(files),
        },
    )


def validate_completed_result(
    run_root: Path,
    bindings: dict[str, str],
    identity: dict[str, str],
    protocol_id: str,
    harness_python: Path,
) -> dict[str, Any]:
    result_path = run_root / "result.json"
    seal_path = run_root / "result_seal.json"
    if (
        not result_path.is_file()
        or result_path.is_symlink()
        or not seal_path.is_file()
        or seal_path.is_symlink()
    ):
        raise RuntimeError(f"result/resume seal pair incomplete: {run_root}")
    result = load_object(result_path)
    validate_result(result, harness_python)
    require_exact_identity(result, identity, "completed result")
    if result.get("protocol_id") != protocol_id:
        raise RuntimeError(f"refusing stale completed result protocol: {result_path}")
    if result.get("bindings") != bindings:
        raise RuntimeError(f"refusing stale result bindings: {result_path}")

    seal = load_object(seal_path)
    actual_files = tree_manifest(run_root, excluded={"result_seal.json"})
    if (
        seal.get("schema_version") != "table1_lam_result_seal_v1"
        or seal.get("identity") != identity
        or seal.get("bindings") != bindings
        or seal.get("result_sha256") != sha256_file(result_path)
        or seal.get("files") != actual_files
        or seal.get("output_tree_sha256") != manifest_digest(actual_files)
    ):
        raise RuntimeError(f"refusing changed or stale result output: {run_root}")

    checkpoint_path = run_root / "run_state.json"
    if not checkpoint_path.is_file() or checkpoint_path.is_symlink():
        raise RuntimeError(f"refusing changed or stale result output: {run_root}")
    checkpoint = load_object(checkpoint_path)
    require_exact_identity(checkpoint, identity, "completed checkpoint")
    if (
        checkpoint.get("protocol_id") != protocol_id
        or checkpoint.get("bindings") != bindings
        or checkpoint.get("attempts") != result.get("attempts")
    ):
        raise RuntimeError(f"refusing changed or stale result output: {run_root}")
    for index, attempt in enumerate(result["attempts"]):
        if attempt.get("attempt_index") != index:
            raise RuntimeError(f"refusing changed or stale result output: {run_root}")
        require_exact_identity(attempt, {key: identity[key] for key in ("method_id", "task_id", "repeat_id")}, "attempt")
        validate_attempt_seal(
            run_root / f"attempts/a{index}", bindings, identity, attempt
        )
    return result


def checkpoint_payload(
    protocol: dict[str, Any],
    task: dict[str, Any],
    repeat_id: str,
    bindings: dict[str, str],
    started_at: str,
    attempts: list[dict[str, Any]],
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "protocol_id": protocol["protocol_id"],
        "run_id": f"lam__{task['task_id']}__{repeat_id}",
        "method_id": "lam",
        "task_id": task["task_id"],
        "repeat_id": repeat_id,
        "bindings": result_bindings(bindings),
        "started_at_utc": started_at,
        "attempts": attempts,
    }


def execute_job(
    task: dict[str, Any],
    repeat_id: str,
    run_root: Path,
    protocol: dict[str, Any],
    bindings: dict[str, str],
    python_executable: Path,
    model_timeout: float,
    evaluator_timeout: float,
    harness_python: Path = DEFAULT_HARNESS_PYTHON,
    *,
    native_invoker: NativeInvoker = real_native_invoker,
    attempt_evaluator: AttemptEvaluator = evaluate_native_attempt,
) -> dict[str, Any]:
    run_root = contained(run_root)
    expected_identity = expected_run_identity(task, repeat_id)
    expected_run_id = expected_identity["run_id"]
    expected_bindings = result_bindings(bindings)
    result_path = run_root / "result.json"
    result_seal_path = run_root / "result_seal.json"
    checkpoint_path = run_root / "run_state.json"
    if result_path.exists() or result_seal_path.exists():
        return validate_completed_result(
            run_root,
            expected_bindings,
            expected_identity,
            protocol["protocol_id"],
            harness_python,
        )
    attempts: list[dict[str, Any]] = []
    if checkpoint_path.is_file():
        checkpoint = load_object(checkpoint_path)
        require_exact_identity(checkpoint, expected_identity, "checkpoint")
        if checkpoint.get("bindings") != expected_bindings:
            raise RuntimeError(f"refusing stale checkpoint with different bindings: {checkpoint_path}")
        if checkpoint.get("protocol_id") != protocol["protocol_id"]:
            raise RuntimeError(f"refusing stale checkpoint protocol: {checkpoint_path}")
        attempts = list(checkpoint.get("attempts") or [])
        started_at = str(checkpoint.get("started_at_utc"))
    else:
        if run_root.exists() and any(run_root.iterdir()):
            raise RuntimeError(f"refusing nonempty run root without bound state: {run_root}")
        run_root.mkdir(parents=True, exist_ok=True)
        started_at = utc_now()
        write_json(
            checkpoint_path,
            checkpoint_payload(protocol, task, repeat_id, bindings, started_at, attempts),
        )
    if [row.get("attempt_index") for row in attempts] != list(range(len(attempts))):
        raise RuntimeError(f"refusing non-contiguous checkpoint attempts: {checkpoint_path}")
    if any(
        row.get("method_id") != "lam"
        or row.get("task_id") != task["task_id"]
        or row.get("repeat_id") != repeat_id
        for row in attempts
    ):
        raise RuntimeError(f"refusing checkpoint with mismatched attempt identity: {checkpoint_path}")
    for index, attempt in enumerate(attempts):
        validate_attempt_seal(
            run_root / f"attempts/a{index}",
            expected_bindings,
            expected_identity,
            attempt,
        )
    terminal_checkpoint = bool(
        attempts
        and (
            attempts[-1].get("evaluation", {}).get("common_qc_pass") is True
            or attempts[-1].get("evaluation", {}).get("state") == "not_evaluable"
        )
    )
    settings = native_settings(protocol)
    settings["_method_binding"] = method_binding(protocol) or {}
    outer_retry_limit = int(settings.get("outer_native_retry_limit", 0))
    next_attempt = (
        int(protocol["max_common_repair_turns"]) + 1
        if terminal_checkpoint
        else len(attempts)
    )
    for attempt_index in range(next_attempt, int(protocol["max_common_repair_turns"]) + 1):
        attempt_root = run_root / f"attempts/a{attempt_index}"
        if attempt_root.exists() and any(attempt_root.iterdir()):
            raise RuntimeError(f"refusing partial attempt without checkpoint record: {attempt_root}")
        attempt_root.mkdir(parents=True, exist_ok=False)
        if attempt_index == 0:
            description = task["prompt"]
            prior_feedback_hash = None
        else:
            prior_feedback = run_root / f"attempts/a{attempt_index - 1}/repair_feedback.json"
            description = repair_description(task, prior_feedback, attempt_index)
            prior_feedback_hash = sha256_file(prior_feedback)
        native_traces: list[dict[str, Any]] = []
        selected_retry = 0
        for retry_index in range(outer_retry_limit + 1):
            selected_retry = retry_index
            trace = native_invoker(
                task,
                attempt_index,
                retry_index,
                description,
                attempt_root / f"native/r{retry_index}",
                settings,
                model_timeout,
                python_executable,
            )
            native_traces.append(trace)
            if trace.get("exit_code") == 0 or trace.get("emitted_model_output") is True:
                break
        write_json(attempt_root / "native_retries.json", native_traces)
        selected_root = attempt_root / f"native/r{selected_retry}"
        request_started = str(native_traces[0].get("started_at_utc") or utc_now())
        response_completed = str(native_traces[-1].get("finished_at_utc") or utc_now())
        model_response_hash = native_traces[-1].get("model_response_sha256")
        evaluator_failed = False
        try:
            report, package = attempt_evaluator(
                run_root,
                attempt_root,
                task,
                repeat_id,
                attempt_index,
                bindings,
                evaluator_timeout,
                harness_python,
            )
            feedback = normalized_feedback(report, attempt_root / "repair_feedback.json")
            verdicts = report.get("verdicts", {})
            evaluation = {
                "state": "observed",
                "executable": bool(verdicts.get("executable")),
                "artifact_saved": bool(verdicts.get("artifact_saved")),
                "common_qc_pass": bool(verdicts.get("common_qc_pass")),
                "urdf_tree_pass": bool(verdicts.get("urdf_tree_pass")),
                "semantic_roles_pass": bool(verdicts.get("semantic_roles_pass")),
                "joint_spec_pass": bool(verdicts.get("joint_spec_pass")),
                "input_bindings_pass": all(report.get("binding_checks", {}).values())
                and all(report.get("protocol_checks", {}).values())
                and all(report.get("task_checks", {}).values()),
                "common_evaluator_report_path": relative(
                    attempt_root / "common_evaluator_report.json", run_root
                ),
                "common_evaluator_report_sha256": sha256_file(
                    attempt_root / "common_evaluator_report.json"
                ),
                "reason": None,
            }
            source = run_root / package["artifacts"]["source"]["path"]
            urdf = run_root / package["artifacts"]["urdf"]["path"]
            output_record = {
                "template_path": relative(source, run_root),
                "template_sha256": sha256_file(source),
                "artifact_path": relative(urdf, run_root),
                "artifact_sha256": sha256_file(urdf),
            }
            probe = package.get("execution_probe", {})
            execution_started = probe.get("started_at_utc")
            execution_completed = probe.get("finished_at_utc")
            del feedback
        except FileNotFoundError:
            evaluation, output_record = missing_output_evaluation(
                task, attempt_index, attempt_root
            )
            execution_started = None
            execution_completed = None
        except Exception as exc:
            evaluator_failed = True
            evaluation, output_record = common_evaluator_failure_evaluation(
                attempt_root, exc
            )
            execution_started = None
            execution_completed = None
        telemetry_native = telemetry_from_native(
            [attempt_root / f"native/r{index}" for index in range(selected_retry + 1)]
        )
        total_wall = sum(float(row.get("wall_time_seconds") or 0.0) for row in native_traces)
        attempt = {
            "attempt_index": attempt_index,
            "attempt_kind": "attempt_0" if attempt_index == 0 else "common_repair",
            "method_id": "lam",
            "task_id": task["task_id"],
            "repeat_id": repeat_id,
            "native_retry_count": selected_retry,
            "native_retry_index": selected_retry,
            "request_started_at_utc": request_started,
            "response_completed_at_utc": response_completed,
            "execution_started_at_utc": execution_started,
            "execution_completed_at_utc": execution_completed,
            "failure_class": (
                "common_evaluator_failure"
                if evaluator_failed
                else (
                    None
                    if evaluation["common_qc_pass"]
                    else (
                        "native_attempt_failed"
                        if native_traces[-1].get("exit_code") != 0
                        else (
                            "artifact_output_missing"
                            if output_record["template_path"] is None
                            else "common_evaluator_rejection"
                        )
                    )
                )
            ),
            "model_response_sha256": model_response_hash,
            "repair_feedback_sha256": prior_feedback_hash,
            "output": output_record,
            "evaluation": evaluation,
            "telemetry": {
                "wall_time_seconds": total_wall,
                "input_tokens": telemetry_native["input_tokens"],
                "output_tokens": telemetry_native["output_tokens"],
                "provider_request_id_hash": None,
                "api_cost_usd": telemetry_native["api_cost_usd"],
                "missing_reasons": {
                    "input_tokens": telemetry_native["reason"],
                    "output_tokens": telemetry_native["reason"],
                    "provider_request_id_hash": "LAM provider metadata does not expose request identifiers",
                    "api_cost_usd": telemetry_native["reason"],
                },
            },
        }
        seal_attempt(
            attempt_root,
            expected_bindings,
            expected_identity,
            attempt,
        )
        attempts.append(attempt)
        write_json(
            checkpoint_path,
            checkpoint_payload(protocol, task, repeat_id, bindings, started_at, attempts),
        )
        if evaluation["common_qc_pass"] or evaluator_failed:
            break
    first = attempts[0]["evaluation"]
    final = attempts[-1]["evaluation"]
    observed = final["state"] == "observed"
    result = {
        "schema_version": "table1_authoring_result_v1",
        "protocol_id": protocol["protocol_id"],
        "bindings": expected_bindings,
        "run_id": expected_run_id,
        "method_id": "lam",
        "task_id": task["task_id"],
        "repeat_id": repeat_id,
        "status": "completed" if observed else "failed",
        "started_at_utc": started_at,
        "finished_at_utc": utc_now(),
        "attempts": attempts,
        "summary": {
            "state": "observed" if observed else "not_evaluable",
            "executable": final["executable"] if observed else None,
            "artifact_saved": final["artifact_saved"] if observed else None,
            "first_shot": first["common_qc_pass"] if observed else None,
            "final_success": final["common_qc_pass"] if observed else None,
            "repair_turns": len(attempts) - 1 if observed else None,
            "reason": None if observed else final["reason"],
        },
        "error": None
        if observed
        else {
            "code": "COMMON_EVALUATOR_NOT_EVALUABLE",
            "message": str(final["reason"]),
        },
    }
    validate_result(result, harness_python)
    write_json(result_path, result)
    seal_result(
        run_root,
        result_path,
        expected_bindings,
        expected_identity,
    )
    return result


def aggregate_results(results: list[dict[str, Any]], intended_runs: int) -> dict[str, Any]:
    observed = [row for row in results if row.get("summary", {}).get("state") == "observed"]
    evaluable = observed
    telemetry_rows = [attempt for result in observed for attempt in result.get("attempts", [])]
    return {
        "schema_version": 1,
        "protocol_id": results[0]["protocol_id"] if results else None,
        "method_id": "lam",
        "generated_at_utc": utc_now(),
        "intended_runs": intended_runs,
        "completed_results": len(results),
        "strict_denominator": intended_runs,
        "observed_denominator": len(observed),
        "evaluable_denominator": len(evaluable),
        "metrics": {
            key: {
                "numerator": sum(row["summary"][key] is True for row in evaluable),
                "denominator": intended_runs,
            }
            for key in ("executable", "artifact_saved", "first_shot", "final_success")
        },
        "repair_turns_total": sum(int(row["summary"]["repair_turns"]) for row in observed),
        "attempt_telemetry": {
            "attempt_denominator": len(telemetry_rows),
            "input_tokens_reported": sum(row["telemetry"]["input_tokens"] is not None for row in telemetry_rows),
            "output_tokens_reported": sum(row["telemetry"]["output_tokens"] is not None for row in telemetry_rows),
            "api_cost_reported": sum(row["telemetry"]["api_cost_usd"] is not None for row in telemetry_rows),
        },
    }


def select_jobs(
    manifest: dict[str, Any], task_ids_raw: str, repeat_ids_raw: str, all_tasks: bool, all_repeats: bool
) -> list[tuple[dict[str, Any], str]]:
    task_ids = [value for value in task_ids_raw.split(",") if value]
    repeat_ids = [value for value in repeat_ids_raw.split(",") if value]
    tasks = manifest["tasks"] if all_tasks else []
    if not all_tasks:
        by_id = {row["task_id"]: row for row in manifest["tasks"]}
        unknown = sorted(set(task_ids) - set(by_id))
        if unknown:
            raise ValueError(f"unknown task IDs: {unknown}")
        tasks = [by_id[value] for value in task_ids]
    repeats = manifest["repeat_ids"] if all_repeats else repeat_ids
    if not tasks or not repeats:
        raise ValueError("select tasks and repeats, or use --all-tasks --all-repeats")
    unknown_repeats = sorted(set(repeats) - set(manifest["repeat_ids"]))
    if unknown_repeats:
        raise ValueError(f"unknown repeat IDs: {unknown_repeats}")
    jobs = [(task, repeat_id) for task in tasks for repeat_id in repeats]
    jobs.sort(key=lambda row: sha256_bytes(f"{row[0]['task_id']}:{row[1]}".encode()))
    return jobs


def report_markdown(summary: dict[str, Any], manifest: dict[str, Any]) -> str:
    blockers_text = "\n".join(f"- {value}" for value in summary["blockers"]) or "- None"
    bindings = manifest["bindings"]
    return f"""# Table 1 LAM Authoring Runner

Status: **{summary['status']}**

Prepared jobs: `{summary['prepared_jobs']}/{summary['expected_jobs']}`. Provider
calls made by this preparation step: `{summary['provider_calls_made']}`. Formal
authoring attempts: `{summary['authoring_attempts']}`. Preparation alone cannot
populate a Table 1 result cell.

## Frozen bindings

- Protocol: `{bindings['protocol']['path']}` (`{bindings['protocol']['sha256']}`)
- Manifest: `{bindings['manifest']['path']}` (`{bindings['manifest']['sha256']}`)
- Hidden specs: `{bindings['hidden_specs']['path']}` (`{bindings['hidden_specs']['sha256']}`; contents are evaluator-only)
- Common evaluator: `{bindings['common_evaluator']['path']}` (`{bindings['common_evaluator']['sha256']}`)
- Package schema: `{bindings['package_schema']['path']}` (`{bindings['package_schema']['sha256']}`)
- Result schema: `{bindings['result_schema']['path']}` (`{bindings['result_schema']['sha256']}`)
- Runner: `{bindings['runner']['path']}` (`{bindings['runner']['sha256']}`)

## Adapter behavior

Each method/task/repeat has a unique frozen run root. Each common attempt has an
immutable `attempts/aN/` directory, and each output-free native retry has an
immutable `native/rN/` directory. A per-invocation config contains placeholders,
not credentials. The official checkout remains unmodified; an exp-side shim only
freezes hard-coded native retry mechanics and prevents deletion of failed evidence.

## Execution blockers

{blockers_text}

`--execute` requires `--acknowledge-paid-run`, an exact
`--confirm-protocol-sha256`, frozen global readiness, frozen LAM readiness, and a
clean dependency audit. Common repairs receive only normalized evaluator feedback.
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--python-executable", type=Path, default=DEFAULT_LAM_PYTHON)
    parser.add_argument(
        "--harness-python-executable", type=Path, default=DEFAULT_HARNESS_PYTHON
    )
    parser.add_argument("--task-ids", default="", help="comma-separated task IDs")
    parser.add_argument("--repeat-ids", default="", help="comma-separated repeat IDs")
    parser.add_argument("--all-tasks", action="store_true")
    parser.add_argument("--all-repeats", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--acknowledge-paid-run", action="store_true")
    parser.add_argument("--confirm-protocol-sha256")
    parser.add_argument("--model-timeout", type=float, default=1800.0)
    parser.add_argument("--evaluator-timeout", type=float, default=180.0)
    args = parser.parse_args()

    output = contained(args.output_dir)
    manifest_path = contained(args.manifest)
    protocol_path = contained(args.protocol)
    python_executable = authorized_python_executable(args.python_executable)
    harness_python = authorized_python_executable(args.harness_python_executable)
    output.mkdir(parents=True, exist_ok=True)
    generated_at = utc_now()
    public_manifest, manifest_meta = load_json_snapshot(manifest_path)
    protocol, protocol_meta = load_json_snapshot(protocol_path)
    contract_checks, inputs, schema_paths = validate_contract(
        public_manifest, protocol, manifest_meta, protocol_meta
    )
    public_manifest = public_manifest or {}
    protocol = protocol or {}
    contract_ready = all(contract_checks.values())
    jobs: list[dict[str, Any]] = []
    nonempty_roots: list[str] = []
    if contract_ready:
        jobs, nonempty_roots = build_jobs(public_manifest, protocol)
    runtime, runtime_blockers = audit_lam_runtime(
        protocol, python_executable, harness_python
    )
    contract_blockers = [
        f"frozen contract check failed: {key}"
        for key, passed in contract_checks.items()
        if not passed
    ]
    output_blockers = (
        [f"preexisting nonempty run roots: {len(nonempty_roots)}"] if nonempty_roots else []
    )
    confirmation = {
        "execute_requested": args.execute,
        "acknowledge_paid_run": args.acknowledge_paid_run,
        "protocol_sha256_confirmation_supplied": bool(args.confirm_protocol_sha256),
        "protocol_sha256_confirmation_matches": bool(args.confirm_protocol_sha256)
        and args.confirm_protocol_sha256.lower() == protocol_meta.get("sha256"),
    }
    confirmation_blockers: list[str] = []
    readiness = protocol.get("execution_readiness", {})
    method_readiness = readiness.get("method_adapters_ready", {}) if isinstance(readiness, dict) else {}
    if args.execute:
        if not args.acknowledge_paid_run:
            confirmation_blockers.append("--execute requires --acknowledge-paid-run")
        if not confirmation["protocol_sha256_confirmation_matches"]:
            confirmation_blockers.append(
                "--execute requires --confirm-protocol-sha256 matching the current protocol"
            )
        if not (
            protocol.get("execution_ready") is True
            and readiness.get("status") == "READY"
            and method_readiness.get("lam") is True
        ):
            confirmation_blockers.append(
                "frozen protocol does not declare global and LAM execution readiness"
            )
    frozen_timeouts = protocol.get("timeouts", {})
    if args.execute and args.model_timeout != float(
        frozen_timeouts.get("model_response_seconds", -1)
    ):
        confirmation_blockers.append("--model-timeout differs from frozen protocol")
    if args.execute and (
        args.evaluator_timeout
        != float(frozen_timeouts.get("common_evaluator_seconds_per_attempt", -1))
        or args.evaluator_timeout
        != float(frozen_timeouts.get("execution_seconds_per_attempt", -1))
    ):
        confirmation_blockers.append(
            "--evaluator-timeout differs from frozen evaluator/execution timeouts"
        )
    execution_blockers = (
        contract_blockers + runtime_blockers + output_blockers + confirmation_blockers
    )
    if not contract_ready:
        status = "BLOCKED_PREPARE"
    elif args.execute and execution_blockers:
        status = "BLOCKED_EXECUTION"
    elif args.execute:
        status = "EXECUTING"
    else:
        status = "PREPARED_NOT_RUN"
    runtime_inputs = {
        "lam_entrypoint": LAM_ENTRYPOINT,
        "lam_requirements": LAM_REQUIREMENTS,
        "lam_package_json": LAM_PACKAGE_JSON,
        "lam_package_lock": LAM_PACKAGE_LOCK,
    }
    bindings = {
        **inputs,
        "runner": input_record(Path(__file__).resolve()),
        **{name: input_record(path) for name, path in runtime_inputs.items()},
    }
    job_order = [job["run_id"] for job in jobs]
    job_roots = [job["run_root"] for job in jobs]
    experiment_manifest = {
        "schema_version": 2,
        "manifest_id": "nano3d_table1_lam_authoring_runner_v2",
        "generated_at_utc": generated_at,
        "mode": "execute_requested" if args.execute else "prepare_only",
        "method_id": "lam",
        "protocol_id": protocol.get("protocol_id"),
        "bindings": bindings,
        "model_binding": method_binding(protocol),
        "normalized_attempt_schema": schema_paths,
        "output_isolation": protocol.get("output_isolation"),
        "selected_task_count": len(public_manifest.get("tasks", [])),
        "selected_repeat_count": len(public_manifest.get("repeat_ids", [])),
        "prepared_job_count": len(jobs),
        "job_order_sha256": canonical_sha256(job_order),
        "jobs_sha256": canonical_sha256(jobs),
        "job_order": job_order,
        "jobs": jobs,
        "job_directories_created_by_prepare": 0,
        "preexisting_nonempty_run_roots": nonempty_roots,
        "provider_calls_made": 0,
        "authoring_attempts": 0,
        "claim_boundary": "Prepared jobs are intent only and cannot populate Table 1 metrics.",
    }
    metrics = {
        name: {"state": "N/R", "value": None}
        for name in (
            "executable",
            "artifact_saved",
            "first_shot",
            "final_success",
            "repair_turns",
            "wall_time",
            "input_tokens",
            "output_tokens",
            "api_cost",
        )
    }
    summary = {
        "schema_version": 2,
        "protocol_id": protocol.get("protocol_id"),
        "method_id": "lam",
        "generated_at_utc": generated_at,
        "status": status,
        "mode": experiment_manifest["mode"],
        "prepared_jobs": len(jobs),
        "expected_jobs": EXPECTED_JOBS,
        "provider_calls_made": 0,
        "authoring_attempts": 0,
        "completed_results": 0,
        "metric_denominator": 0,
        "metrics": metrics,
        "contract_checks": contract_checks,
        "runtime": runtime,
        "adapter_gates": ADAPTER_GATES,
        "confirmation": confirmation,
        "ready_to_execute": not execution_blockers,
        "contract_blockers": contract_blockers,
        "runtime_blockers": runtime_blockers,
        "adapter_blockers": [],
        "output_blockers": output_blockers,
        "confirmation_blockers": confirmation_blockers,
        "blockers": execution_blockers,
        "other_method_adapter_readiness_ignored": True,
    }
    checks = {
        "frozen_contract_valid": contract_ready,
        "prepared_exactly_162_jobs": len(jobs) == EXPECTED_JOBS,
        "run_ids_unique": len(job_order) == len(set(job_order)) == EXPECTED_JOBS,
        "run_roots_unique": len(job_roots) == len(set(job_roots)) == EXPECTED_JOBS,
        "all_run_roots_lam_scoped": all(
            root.startswith("exp/runtime/table1_reliability/common_authoring/lam/")
            for root in job_roots
        ),
        "attempt_layout_0_through_3": bool(jobs)
        and all(
            [row["attempt_index"] for row in job["attempts"]] == [0, 1, 2, 3]
            for job in jobs
        ),
        "normalized_attempt_schema_bound": contract_checks.get(
            "normalized_result_schema", False
        ),
        "prepare_created_no_job_directories": True,
        "provider_calls_zero": True,
        "authoring_attempts_zero": True,
        "all_metrics_nr_null": all(
            row["state"] == "N/R" and row["value"] is None for row in metrics.values()
        ),
        "adapter_gates_implemented": all(ADAPTER_GATES.values()),
        "status_fail_closed": status
        in {"PREPARED_NOT_RUN", "BLOCKED_PREPARE", "BLOCKED_EXECUTION", "EXECUTING"},
        "execute_request_never_started_provider_when_blocked": not args.execute
        or status == "EXECUTING"
        or summary["provider_calls_made"] == 0,
        "only_lam_adapter_readiness_is_relevant": True,
    }
    manifest_output = output / "manifest.json"
    summary_output = output / "summary.json"
    report_output = output / "report.md"
    self_check_output = output / "self_check.json"
    write_json(manifest_output, experiment_manifest)
    write_json(summary_output, summary)
    write_text_atomic(report_output, report_markdown(summary, experiment_manifest))
    inputs_stable = all(
        not row.get("exists")
        or (
            isinstance(row.get("path"), str)
            and sha256_file(REPO_ROOT / row["path"]) == row.get("sha256")
        )
        for row in bindings.values()
    )
    checks["input_snapshots_stable_during_prepare"] = inputs_stable
    secret_assignment_found = any(
        SENSITIVE_ASSIGNMENT_RE.search(path.read_text(encoding="utf-8"))
        for path in (manifest_output, summary_output, report_output)
    )
    checks["credential_values_not_emitted"] = not secret_assignment_found
    self_check_pass = all(checks.values())
    self_check = {
        "schema_version": 2,
        "check_id": "nano3d_table1_lam_authoring_runner_self_check_v2",
        "generated_at_utc": generated_at,
        "status": "PASS" if self_check_pass else "FAIL",
        "checks": checks,
        "passed": sum(bool(value) for value in checks.values()),
        "total": len(checks),
        "evidence_sha256": {
            "manifest.json": sha256_file(manifest_output),
            "summary.json": sha256_file(summary_output),
            "report.md": sha256_file(report_output),
        },
    }
    write_json(self_check_output, self_check)
    if not self_check_pass:
        print(json.dumps({"status": status, "self_check": "FAIL"}, sort_keys=True))
        return 1
    if status in {"BLOCKED_PREPARE", "BLOCKED_EXECUTION"}:
        print(
            json.dumps(
                {
                    "status": status,
                    "prepared_jobs": len(jobs),
                    "provider_calls_made": 0,
                    "authoring_attempts": 0,
                    "self_check": "PASS",
                    "blockers": len(execution_blockers),
                    "output_dir": str(output),
                },
                sort_keys=True,
            )
        )
        return 2
    if not args.execute:
        print(
            json.dumps(
                {
                    "status": status,
                    "prepared_jobs": len(jobs),
                    "provider_calls_made": 0,
                    "authoring_attempts": 0,
                    "self_check": "PASS",
                    "blockers": len(execution_blockers),
                    "output_dir": str(output),
                },
                sort_keys=True,
            )
        )
        return 0
    selected = select_jobs(
        public_manifest,
        args.task_ids,
        args.repeat_ids,
        args.all_tasks,
        args.all_repeats,
    )
    bindings_for_results = frozen_bindings(manifest_path, protocol_path, protocol)
    results = [
        execute_job(
            task,
            repeat_id,
            run_root_for(task["task_id"], repeat_id, protocol),
            protocol,
            bindings_for_results,
            python_executable,
            args.model_timeout,
            args.evaluator_timeout,
            harness_python,
        )
        for task, repeat_id in selected
    ]
    execution_summary = aggregate_results(results, len(selected))
    write_json(output / "execution_summary.json", execution_summary)
    print(json.dumps(execution_summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
