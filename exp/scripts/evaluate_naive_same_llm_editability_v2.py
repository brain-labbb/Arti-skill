#!/usr/bin/env python3
"""Compile and score frozen naive same-LLM Table 5 outputs.

This evaluator is deliberately separate from provider generation. It treats the
provider directory as immutable, materializes each accepted ``model.py`` as an
independent child record in an output-local Articraft snapshot, invokes the
official full compiler exactly once per selected parent and child, and calls the
same frozen ``score_record`` implementation used by the native-fork cohort.

The evaluation snapshot contains parents but no historical-gold records. Gold
metadata is never used to construct a child record or prompt.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# Limit native math-library thread pools before importing the scorer's NumPy
# and trimesh dependencies. The compiler itself remains single-process here.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import run_articraft_editability_v2 as scorer
import run_naive_same_llm_editability_v2 as naive_runner

SCRIPT_DIR = Path(__file__).resolve().parent
EXP_ROOT = SCRIPT_DIR.parent
REPO_ROOT = EXP_ROOT.parent
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
ARTICRAFT_ROOT = REPO_ROOT / "articraft_data"
DEFAULT_MANIFEST = EXP_ROOT / "reference" / "editability_v2" / "articraft_parent_manifest.json"
DEFAULT_SCORER_CONTRACT = (
    EXP_ROOT / "reference" / "editability_v2" / "articraft_scorer_contract_v2.json"
)
DEFAULT_PROMPT_CONTRACT = (
    EXP_ROOT / "reference" / "editability_v2" / "naive_same_llm_prompt_contract_v1.json"
)
DEFAULT_PROVIDER_OUTPUT = (
    EXP_ROOT
    / "runtime"
    / "nano3d_editability_v2"
    / "naive_same_llm_final_d8d6e47a"
)
DEFAULT_OUTPUT = (
    EXP_ROOT
    / "runtime"
    / "nano3d_editability_v2"
    / "naive_same_llm_eval_final_d8d6e47a"
)
MODEL_ID = "qwen3.7-max-2026-05-20"
PROVIDER = "dashscope"
REVISION_ID = "rev_000001"


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


def text_sha256(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def load_json(path: Path) -> Any:
    return json.loads(contained(path, strict=True).read_text(encoding="utf-8"))


def dump_json(path: Path, payload: Any) -> None:
    path = contained(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def child_record_id(task_id: str) -> str:
    return f"rec_table5_naive_same_llm_v2_{task_id.lower()}"


def selected_tasks(
    manifest: dict[str, Any], task_ids: list[str], allow_partial: bool
) -> list[dict[str, Any]]:
    tasks = manifest.get("tasks")
    if not isinstance(tasks, list) or len(tasks) != 18:
        raise RuntimeError("frozen manifest must contain exactly 18 tasks")
    by_id = {str(task["task_id"]): task for task in tasks}
    if task_ids:
        unknown = sorted(set(task_ids) - set(by_id))
        if unknown:
            raise RuntimeError(f"unknown task IDs: {', '.join(unknown)}")
        if not allow_partial and set(task_ids) != set(by_id):
            raise RuntimeError("a task subset requires --allow-partial")
        wanted = set(task_ids)
        return [task for task in tasks if task["task_id"] in wanted]
    return tasks


def provider_artifact_paths(provider_output: Path, task_id: str) -> dict[str, Path]:
    task_dir = contained(provider_output / "tasks" / task_id)
    return {
        "record": contained(task_dir / "record.json"),
        "model": contained(task_dir / "model.py"),
        "raw_response": contained(task_dir / "raw_response.json"),
        "request_preview": contained(provider_output / "request_previews" / f"{task_id}.json"),
    }


def fingerprint_provider_outputs(
    provider_output: Path, tasks: list[dict[str, Any]]
) -> dict[str, dict[str, str]]:
    fingerprints: dict[str, dict[str, str]] = {}
    for task in tasks:
        task_id = str(task["task_id"])
        paths = provider_artifact_paths(provider_output, task_id)
        missing = [name for name, path in paths.items() if not path.is_file()]
        if missing:
            raise RuntimeError(f"{task_id}: missing provider artifacts: {', '.join(missing)}")
        fingerprints[task_id] = {name: sha256(path) for name, path in paths.items()}
    return fingerprints


def validate_provider_cohort_summary(
    provider_output: Path,
    tasks: list[dict[str, Any]],
    *,
    manifest_hash: str,
    expected_summary_sha256: str | None,
    expected_runner_sha256: str | None,
) -> dict[str, Any]:
    summary_path = contained(provider_output / "summary.json", strict=True)
    summary = load_json(summary_path)
    errors: list[str] = []
    actual_summary_sha256 = sha256(summary_path)
    if expected_summary_sha256 and actual_summary_sha256 != expected_summary_sha256.lower():
        errors.append("PROVIDER_SUMMARY_SHA256_MISMATCH")
    if summary.get("status") != "COMPLETE":
        errors.append("PROVIDER_COHORT_NOT_COMPLETE")
    if summary.get("manifest_sha256") != manifest_hash:
        errors.append("PROVIDER_SUMMARY_MANIFEST_SHA256_MISMATCH")
    summary_records = summary.get("records")
    if not isinstance(summary_records, list) or len(summary_records) != 18:
        errors.append("PROVIDER_SUMMARY_RECORD_COUNT_NOT_18")
        summary_records = []
    summary_by_id = {
        str(record.get("task_id")): record
        for record in summary_records
        if isinstance(record, dict) and record.get("task_id")
    }
    mismatched_tasks: list[str] = []
    for task in tasks:
        task_id = str(task["task_id"])
        disk_record = load_json(provider_artifact_paths(provider_output, task_id)["record"])
        if summary_by_id.get(task_id) != disk_record:
            mismatched_tasks.append(task_id)
    if mismatched_tasks:
        errors.append("PROVIDER_SUMMARY_TASK_ARTIFACT_MISMATCH")

    owner_path = contained(provider_output / ".runner.lock.owner.json")
    owner = load_json(owner_path) if owner_path.is_file() else None
    if expected_runner_sha256:
        if not isinstance(owner, dict):
            errors.append("PROVIDER_LOCK_OWNER_METADATA_MISSING")
        else:
            if owner.get("status") != "RELEASED_COMPLETE":
                errors.append("PROVIDER_LOCK_OWNER_NOT_RELEASED_COMPLETE")
            if owner.get("runner_sha256") != expected_runner_sha256.lower():
                errors.append("PROVIDER_RUNNER_SHA256_MISMATCH")
            if owner.get("expected_manifest_sha256") != manifest_hash:
                errors.append("PROVIDER_OWNER_MANIFEST_SHA256_MISMATCH")
    return {
        "passed": not errors,
        "errors": sorted(set(errors)),
        "summary": relative(summary_path),
        "summary_sha256": actual_summary_sha256,
        "summary_task_count": len(summary_records),
        "summary_disk_mismatched_tasks": sorted(mismatched_tasks),
        "lock_owner": relative(owner_path) if owner_path.is_file() else None,
        "lock_owner_status": owner.get("status") if isinstance(owner, dict) else None,
        "provider_runner_sha256": owner.get("runner_sha256") if isinstance(owner, dict) else None,
    }


def validate_provider_output(
    task: dict[str, Any], provider_output: Path, prompt_contract: dict[str, Any]
) -> dict[str, Any]:
    task_id = str(task["task_id"])
    paths = provider_artifact_paths(provider_output, task_id)
    record = load_json(paths["record"])
    preview = load_json(paths["request_preview"])
    model_hash = sha256(paths["model"])
    parent_model = contained(WORKSPACE_ROOT / task["parent_model_path"], strict=True)
    parent_source = parent_model.read_text(encoding="utf-8")
    parent_hash = sha256(parent_model)
    expected_user_message = naive_runner.render_user_message(task, parent_source, parent_hash)
    expected_messages = [
        {"role": "system", "content": str(prompt_contract["system_prompt"])},
        {"role": "user", "content": expected_user_message},
    ]

    errors: list[str] = []
    expected_fields = {
        "task_id": task_id,
        "mode": "execute",
        "status": "OUTPUT_ACCEPTED",
        "accepted": True,
        "provider_requests": 1,
        "turns": 1,
        "tools_exposed": 0,
        "compile_feedback": False,
        "repair_turns": 0,
        "parent_record_id": task["parent_record_id"],
        "parent_model_sha256": task["parent_model_sha256"],
        "gold_in_request": False,
        "model_py_sha256": model_hash,
        "raw_response_sha256": sha256(paths["raw_response"]),
        "model_py": relative(paths["model"]),
        "raw_response": relative(paths["raw_response"]),
    }
    for key, expected in expected_fields.items():
        if record.get(key) != expected:
            errors.append(f"PROVIDER_RECORD_FIELD_MISMATCH:{key}")
    if record.get("tool_call_count") != 0:
        errors.append("PROVIDER_TOOL_CALL_COUNT_NOT_ZERO")
    if record.get("exact_single_python_fence") is not True:
        errors.append("PROVIDER_OUTPUT_NOT_EXACT_SINGLE_PYTHON_FENCE")
    if record.get("syntax_valid") is not True:
        errors.append("PROVIDER_OUTPUT_SYNTAX_INVALID")
    if parent_hash != task["parent_model_sha256"]:
        errors.append("PARENT_MODEL_SHA256_MISMATCH")
    if preview.get("messages") != expected_messages:
        errors.append("REQUEST_PREVIEW_MESSAGES_MISMATCH")
    if preview.get("model") != MODEL_ID:
        errors.append("REQUEST_PREVIEW_MODEL_MISMATCH")
    if preview.get("max_tokens") != 65_536:
        errors.append("REQUEST_PREVIEW_OUTPUT_CAP_MISMATCH")
    if preview.get("tools") not in (None, []):
        errors.append("REQUEST_PREVIEW_EXPOSED_TOOLS")

    return {
        "valid": not errors,
        "errors": sorted(set(errors)),
        "record": record,
        "paths": paths,
        "exact_user_message": expected_user_message,
        "system_prompt": str(prompt_contract["system_prompt"]),
        "request_preview_messages_exact": preview.get("messages") == expected_messages,
        "gold_in_request": False,
        "gold_fields_used_to_render_request": [],
    }


def copy_evaluation_snapshot(
    manifest: dict[str, Any], snapshot: Path, manifest_hash: str
) -> None:
    snapshot = contained(snapshot)
    if snapshot.exists():
        raise RuntimeError(f"evaluation snapshot already exists: {snapshot}")
    snapshot.mkdir(parents=True)
    for name in ("sdk", "agent", "articraft", "storage", "cli"):
        os.symlink(contained(ARTICRAFT_ROOT / name, strict=True), snapshot / name)
    for name in (
        "pyproject.toml",
        "uv.lock",
        ".python-version",
        "FORK_VARIANTS.md",
        "VARIANT_PIPELINE.md",
    ):
        source = contained(ARTICRAFT_ROOT / name)
        if source.exists():
            os.symlink(contained(source, strict=True), snapshot / name)

    source_data = contained(ARTICRAFT_ROOT / "data", strict=True)
    target_data = snapshot / "data"
    target_data.mkdir()
    for name in (
        "CATEGORY_PROMPT_GUIDE.md",
        "CATEGORY_SELECTION_REQUIREMENTS.md",
        "REJECTED_CATEGORIES.md",
        "supercategories.json",
    ):
        source = source_data / name
        if source.exists():
            shutil.copy2(contained(source, strict=True), target_data / name)
    for name in ("categories", "system_prompts"):
        source = source_data / name
        if source.exists():
            shutil.copytree(contained(source, strict=True), target_data / name)
    for name in ("records", "cache", "local", "index"):
        (target_data / name).mkdir()

    # Evaluation needs parent geometry only. Historical-gold records are
    # intentionally excluded from this snapshot.
    for parent_id in sorted({str(task["parent_record_id"]) for task in manifest["tasks"]}):
        shutil.copytree(
            contained(source_data / "records" / parent_id, strict=True),
            target_data / "records" / parent_id,
        )
    (snapshot / "MANIFEST.sha256").write_text(manifest_hash + "\n", encoding="utf-8")
    dump_json(
        snapshot / "EVALUATION_SNAPSHOT.json",
        {
            "schema_version": 1,
            "manifest_sha256": manifest_hash,
            "parents_copied": 18,
            "historical_gold_records_copied": 0,
            "purpose": "naive_same_llm_compile_and_score",
        },
    )


def materialize_child_record(
    task: dict[str, Any], validation: dict[str, Any], snapshot: Path
) -> dict[str, Any]:
    task_id = str(task["task_id"])
    record_id = child_record_id(task_id)
    record_dir = contained(snapshot / "data" / "records" / record_id)
    if record_dir.exists():
        raise RuntimeError(f"{task_id}: child record already exists: {record_id}")
    revision_dir = record_dir / "revisions" / REVISION_ID
    revision_dir.mkdir(parents=True)

    model_source = contained(validation["paths"]["model"], strict=True)
    model_target = contained(revision_dir / "model.py")
    shutil.copy2(model_source, model_target)
    exact_user_message = str(validation["exact_user_message"])
    prompt_target = contained(revision_dir / "prompt.txt")
    prompt_target.write_text(exact_user_message, encoding="utf-8")
    prompt_hash = text_sha256(exact_user_message)
    model_hash = sha256(model_target)
    timestamp = utc_now()
    parent_record = load_json(snapshot / "data" / "records" / task["parent_record_id"] / "record.json")
    source_run_id = f"run_table5_naive_same_llm_v2_{task_id.lower()}"
    artifacts = {
        "prompt_txt": f"revisions/{REVISION_ID}/prompt.txt",
        "prompt_series_json": None,
        "model_py": f"revisions/{REVISION_ID}/model.py",
        "provenance_json": f"revisions/{REVISION_ID}/provenance.json",
        "cost_json": None,
        "inputs_dir": None,
        "traces_dir": None,
    }
    hashes = {"prompt_sha256": prompt_hash, "model_py_sha256": model_hash}
    source = {
        "run_id": source_run_id,
        "prompt_batch_id": None,
        "batch_spec_id": None,
        "row_id": task_id,
        "prompt_index": None,
    }
    lineage = {
        "origin_record_id": parent_record.get("lineage", {}).get("origin_record_id")
        or task["parent_record_id"],
        "parent_record_id": task["parent_record_id"],
        "parent_revision_id": task["parent_revision_id"],
        "edit_mode": "copy",
    }
    run_summary = {
        "turn_count": 1,
        "tool_call_count": 0,
        "compile_attempt_count": 0,
        "final_status": "provider_output_accepted_pending_external_compile",
    }
    generation = {
        "provider": PROVIDER,
        "model_id": MODEL_ID,
        "thinking_level": "high",
        "openai_transport": None,
        "openai_reasoning_summary": None,
        "max_turns": 1,
        "max_cost_usd": None,
    }
    record = {
        "schema_version": 3,
        "record_id": record_id,
        "created_at": timestamp,
        "updated_at": timestamp,
        "rating": None,
        "secondary_rating": None,
        "author": None,
        "rated_by": None,
        "secondary_rated_by": None,
        "kind": "generated_model",
        "prompt_kind": "single_prompt",
        "category_slug": parent_record.get("category_slug"),
        "source": source,
        "sdk_package": "sdk",
        "provider": PROVIDER,
        "model_id": MODEL_ID,
        "display": {
            "title": f"Table 5 naive same-LLM edit {task_id}",
            "prompt_preview": str(task["normalized_edit_instruction"]),
        },
        "artifacts": artifacts,
        "hashes": hashes,
        "collections": [],
        "active_revision_id": REVISION_ID,
        "lineage": lineage,
    }
    provenance = {
        "schema_version": 2,
        "record_id": record_id,
        "generation": generation,
        "prompting": {
            "system_prompt_file": "naive_same_llm_prompt_contract_v1.json#system_prompt",
            "system_prompt_sha256": text_sha256(str(validation["system_prompt"])),
        },
        "sdk": {"sdk_package": "sdk", "sdk_version": "workspace", "sdk_fingerprint": None},
        "environment": {
            "python_version": platform.python_version(),
            "platform": platform.platform(),
            "evaluation_only": True,
            "provider_record_sha256": sha256(validation["paths"]["record"]),
            "provider_raw_response_sha256": sha256(validation["paths"]["raw_response"]),
            "request_preview_sha256": sha256(validation["paths"]["request_preview"]),
        },
        "run_summary": run_summary,
    }
    revision = {
        "schema_version": 1,
        "record_id": record_id,
        "revision_id": REVISION_ID,
        "created_at": timestamp,
        "prompt_kind": "single_prompt",
        "prompt_sha256": prompt_hash,
        "source": source,
        "generation": generation,
        "artifacts": artifacts,
        "hashes": hashes,
        "run_summary": run_summary,
        "parent": {"record_id": task["parent_record_id"], "revision_id": task["parent_revision_id"]},
        "seed": {
            "record_id": task["parent_record_id"],
            "revision_id": task["parent_revision_id"],
            "artifact": "model.py",
        },
        "inherited_inputs": [],
    }
    dump_json(record_dir / "record.json", record)
    dump_json(revision_dir / "provenance.json", provenance)
    dump_json(revision_dir / "revision.json", revision)
    return {
        "task_id": task_id,
        "record_id": record_id,
        "record": relative(record_dir / "record.json"),
        "model_py_sha256": model_hash,
        "prompt_sha256": prompt_hash,
        "parent_record_id": task["parent_record_id"],
        "lineage": lineage,
    }


def write_report(output: Path, summary: dict[str, Any]) -> None:
    rows = []
    for item in summary["records"]:
        compile_result = item.get("compile") or {}
        score = item.get("score") or {}
        rows.append(
            f"| {item['task_id']} | {item['edit_class']} | {compile_result.get('returncode', 'N/A')} "
            f"| {'PASS' if score.get('passed') else 'FAIL'} | {item['status']} |"
        )
    report = "\n".join(
        [
            "# Naive same-LLM Table 5 compile and score",
            "",
            f"Status: **{summary['status']}**",
            "",
            f"Manifest SHA-256: `{summary['manifest_sha256']}`",
            f"Scorer contract SHA-256: `{summary['scorer_contract_sha256']}`",
            f"Scorer implementation SHA-256: `{summary['scorer_implementation_sha256']}`",
            "",
            "Historical gold records in evaluation snapshot: **0**.",
            "Compiler/repair feedback returned to provider: **no**.",
            "",
            "| Task | Class | Compile RC | Score | Status |",
            "|---|---|---:|---|---|",
            *rows,
            "",
            f"Passed: **{summary['passed']}/{summary['task_count']}**.",
            f"Compile success: **{summary['compile_success']}/{summary['task_count']}**.",
            "",
        ]
    )
    contained(output / "report.md").write_text(report, encoding="utf-8")


def provider_totals(validations: dict[str, dict[str, Any]]) -> dict[str, Any]:
    token_keys = (
        "prompt_tokens",
        "cached_tokens",
        "candidates_tokens",
        "reasoning_tokens",
        "total_tokens",
    )
    tokens = {key: 0 for key in token_keys}
    estimated_cost_cny = 0.0
    elapsed_seconds = 0.0
    for validation in validations.values():
        record = validation["record"]
        usage = record.get("usage") if isinstance(record.get("usage"), dict) else {}
        for key in token_keys:
            value = usage.get(key)
            if isinstance(value, int):
                tokens[key] += value
        cost = record.get("estimated_cost_cny")
        if isinstance(cost, dict) and isinstance(cost.get("total"), (int, float)):
            estimated_cost_cny += float(cost["total"])
        elapsed = record.get("elapsed_seconds")
        if isinstance(elapsed, (int, float)):
            elapsed_seconds += float(elapsed)
    return {
        "actual_tokens": tokens,
        "estimated_cost_cny": round(estimated_cost_cny, 8),
        "summed_provider_elapsed_seconds": round(elapsed_seconds, 3),
        "usd_cost": None,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--expected-manifest-sha256", required=True)
    parser.add_argument("--scorer-contract", type=Path, default=DEFAULT_SCORER_CONTRACT)
    parser.add_argument("--expected-scorer-contract-sha256", required=True)
    parser.add_argument("--prompt-contract", type=Path, default=DEFAULT_PROMPT_CONTRACT)
    parser.add_argument("--provider-output", type=Path, default=DEFAULT_PROVIDER_OUTPUT)
    parser.add_argument("--expected-provider-summary-sha256")
    parser.add_argument("--expected-provider-runner-sha256")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--task-id", action="append", default=[])
    parser.add_argument("--allow-partial", action="store_true")
    parser.add_argument("--record-timeout-seconds", type=int, default=1800)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manifest_path = contained(args.manifest, strict=True)
    scorer_contract_path = contained(args.scorer_contract, strict=True)
    prompt_contract_path = contained(args.prompt_contract, strict=True)
    provider_output = contained(args.provider_output, strict=True)
    output = contained(args.output_dir)
    if output.exists():
        raise RuntimeError(f"output directory already exists; refusing overwrite: {output}")

    manifest_hash = sha256(manifest_path)
    scorer_contract_hash = sha256(scorer_contract_path)
    scorer_implementation_path = contained(Path(scorer.__file__), strict=True)
    scorer_implementation_hash = sha256(scorer_implementation_path)
    prompt_contract_hash = sha256(prompt_contract_path)
    manifest = load_json(manifest_path)
    scorer_contract = load_json(scorer_contract_path)
    prompt_contract = load_json(prompt_contract_path)

    blockers: list[str] = []
    if manifest_hash != args.expected_manifest_sha256.lower():
        blockers.append("MANIFEST_SHA256_MISMATCH")
    if scorer_contract_hash != args.expected_scorer_contract_sha256.lower():
        blockers.append("SCORER_CONTRACT_SHA256_MISMATCH")
    if manifest.get("scorer_sha256") != scorer_contract_hash:
        blockers.append("MANIFEST_SCORER_CONTRACT_SHA256_MISMATCH")
    if manifest.get("execution_runner_sha256") != scorer_implementation_hash:
        blockers.append("MANIFEST_SCORER_IMPLEMENTATION_SHA256_MISMATCH")
    if scorer_contract.get("semantic_rule_tables_sha256") != scorer.scorer_rule_tables_sha256():
        blockers.append("SCORER_SEMANTIC_RULE_TABLES_SHA256_MISMATCH")
    if manifest.get("prompt_contract_sha256") != prompt_contract_hash:
        blockers.append("MANIFEST_PROMPT_CONTRACT_SHA256_MISMATCH")
    blockers.extend(scorer.validate_manifest(manifest, manifest_path))
    if blockers:
        raise RuntimeError("frozen artifact validation failed: " + ", ".join(sorted(set(blockers))))

    tasks = selected_tasks(manifest, args.task_id, args.allow_partial)
    if not args.allow_partial and len(tasks) != 18:
        raise RuntimeError("formal evaluation requires all 18 tasks")
    provider_cohort_audit = validate_provider_cohort_summary(
        provider_output,
        tasks,
        manifest_hash=manifest_hash,
        expected_summary_sha256=args.expected_provider_summary_sha256,
        expected_runner_sha256=args.expected_provider_runner_sha256,
    )
    if not provider_cohort_audit["passed"]:
        raise RuntimeError(
            "provider cohort integrity failed: "
            + json.dumps(provider_cohort_audit["errors"], sort_keys=True)
        )
    before_fingerprints = fingerprint_provider_outputs(provider_output, tasks)
    validations = {
        str(task["task_id"]): validate_provider_output(task, provider_output, prompt_contract)
        for task in tasks
    }
    invalid = {
        task_id: row["errors"] for task_id, row in validations.items() if not row["valid"]
    }
    if invalid:
        raise RuntimeError("provider output validation failed: " + json.dumps(invalid, sort_keys=True))

    output.mkdir(parents=True)
    snapshot = output / "repo_snapshot"
    copy_evaluation_snapshot(manifest, snapshot, manifest_hash)
    frozen_dir = output / "frozen_scorer"
    frozen_dir.mkdir()
    shutil.copy2(scorer_contract_path, frozen_dir / scorer_contract_path.name)
    shutil.copy2(scorer_implementation_path, frozen_dir / scorer_implementation_path.name)
    shutil.copy2(prompt_contract_path, frozen_dir / prompt_contract_path.name)

    materialized = {
        str(task["task_id"]): materialize_child_record(
            task, validations[str(task["task_id"])], snapshot
        )
        for task in tasks
    }
    results: list[dict[str, Any]] = []
    for task in tasks:
        task_id = str(task["task_id"])
        parent_id = str(task["parent_record_id"])
        child_id = child_record_id(task_id)
        parent_compile = scorer.compile_record(
            snapshot,
            parent_id,
            args.record_timeout_seconds,
            output / "compile_logs" / "parents",
        )
        child_compile = scorer.compile_record(
            snapshot,
            child_id,
            args.record_timeout_seconds,
            output / "compile_logs" / "children",
        )
        item: dict[str, Any] = {
            "task_id": task_id,
            "edit_class": task["edit_class"],
            "parent_record_id": parent_id,
            "record_id": child_id,
            "provider_validation": {
                "passed": validations[task_id]["valid"],
                "request_preview_messages_exact": validations[task_id][
                    "request_preview_messages_exact"
                ],
                "gold_in_request": False,
                "gold_fields_used_to_render_request": [],
            },
            "provider_generation": {
                "status": validations[task_id]["record"]["status"],
                "elapsed_seconds": validations[task_id]["record"].get("elapsed_seconds"),
                "usage": validations[task_id]["record"].get("usage", {}),
                "estimated_cost_cny": validations[task_id]["record"].get(
                    "estimated_cost_cny", {}
                ),
                "model_py_sha256": validations[task_id]["record"]["model_py_sha256"],
                "raw_response_sha256": validations[task_id]["record"][
                    "raw_response_sha256"
                ],
            },
            "materialized": materialized[task_id],
            "parent_compile": parent_compile,
            "compile": child_compile,
            "compile_attempts": 1,
            "repair_attempts": 0,
            "compiler_feedback_returned_to_provider": False,
        }
        if parent_compile["returncode"] != 0:
            item["status"] = "PARENT_COMPILE_FAILED"
        elif child_compile["returncode"] != 0:
            item["status"] = "CHILD_COMPILE_FAILED"
        else:
            try:
                item["score"] = scorer.score_record(task, parent_id, child_id, snapshot)
                item["status"] = "SCORED_PASS" if item["score"]["passed"] else "SCORED_FAIL"
            except Exception as exc:  # noqa: BLE001
                item["status"] = "SCORER_ERROR"
                item["scorer_error"] = f"{type(exc).__name__}: {exc}"
        results.append(item)
        dump_json(output / "records" / f"{task_id}.json", item)

    after_fingerprints = fingerprint_provider_outputs(provider_output, tasks)
    provider_outputs_unchanged = before_fingerprints == after_fingerprints
    final_scorer_hash = sha256(scorer_implementation_path)
    final_contract_hash = sha256(scorer_contract_path)
    frozen_hashes_match = (
        final_scorer_hash == scorer_implementation_hash
        and final_contract_hash == scorer_contract_hash
        and sha256(frozen_dir / scorer_implementation_path.name) == scorer_implementation_hash
        and sha256(frozen_dir / scorer_contract_path.name) == scorer_contract_hash
    )
    passed = sum(bool(item.get("score", {}).get("passed")) for item in results)
    compile_success = sum(item["compile"]["returncode"] == 0 for item in results)
    summary = {
        "schema_version": 1,
        "protocol_id": "nano3d_table5_naive_same_llm_compile_score_v2",
        "status": (
            "COMPLETE"
            if provider_outputs_unchanged and frozen_hashes_match and len(results) == len(tasks)
            else "INTEGRITY_FAILURE"
        ),
        "evidence_class": "LOCAL_DIRECT",
        "generated_at_utc": utc_now(),
        "formal_full_cohort": not args.allow_partial and len(tasks) == 18,
        "manifest": relative(manifest_path),
        "manifest_sha256": manifest_hash,
        "scorer_contract": relative(scorer_contract_path),
        "scorer_contract_sha256": scorer_contract_hash,
        "scorer_implementation": relative(scorer_implementation_path),
        "scorer_implementation_sha256": scorer_implementation_hash,
        "prompt_contract_sha256": prompt_contract_hash,
        "provider_output": relative(provider_output),
        "provider_cohort_audit": provider_cohort_audit,
        "provider_outputs_unchanged": provider_outputs_unchanged,
        "provider_artifact_fingerprints": before_fingerprints,
        "frozen_scorer_hashes_match_after_run": frozen_hashes_match,
        "evaluation_snapshot": relative(snapshot),
        "historical_gold_records_in_snapshot": 0,
        "gold_used_to_construct_child_records": False,
        "gold_used_by_score_record": False,
        "task_count": len(tasks),
        "provider_totals": provider_totals(validations),
        "compile_success": compile_success,
        "passed": passed,
        "failed": len(tasks) - passed,
        "by_class": {
            edit_class: {
                "n": sum(item["edit_class"] == edit_class for item in results),
                "compile_success": sum(
                    item["edit_class"] == edit_class and item["compile"]["returncode"] == 0
                    for item in results
                ),
                "passed": sum(
                    item["edit_class"] == edit_class
                    and bool(item.get("score", {}).get("passed"))
                    for item in results
                ),
            }
            for edit_class in ("additive", "modified", "scale")
        },
        "records": results,
    }
    dump_json(output / "summary.json", summary)
    write_report(output, summary)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 0 if summary["status"] == "COMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
