#!/usr/bin/env python3
"""Fail-closed preflight for the LAM row of Nano3D Table 5.

The official LAM release is locally materialized, including generated Three.js
source and URDFs.  That does not by itself define a source-edit experiment.
This harness requires an explicit 18-task parent manifest and a callable LAM
source-edit adapter before any Table 5 metric can be reported.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[2]
WORKSPACE_ROOT = REPO_ROOT.parent.resolve()
EXP_ROOT = REPO_ROOT / "exp"
TASK_SOURCE = EXP_ROOT / "scripts" / "run_nano3d_paper_editability.py"
DEFAULT_MANIFEST = EXP_ROOT / "reference" / "editability_baselines" / "lam_manifest.jsonl"
DEFAULT_EDIT_RUNNER = EXP_ROOT / "scripts" / "lam_source_edit_adapter.py"
DEFAULT_OUTPUT = EXP_ROOT / "runtime" / "nano3d_editability_baselines" / "lam"
LAM_ROOT = REPO_ROOT / ".cache" / "table6_sources" / "lam"
LAM_CODE = LAM_ROOT / "code"
LAM_DATASET = LAM_ROOT / "dataset"
LAM_PARQUET = LAM_DATASET / "articulated_code.parquet"
TABLE6_PREFLIGHT = EXP_ROOT / "runtime" / "table6_lam" / "preflight.json"
TABLE6_SUMMARY = EXP_ROOT / "runtime" / "table6_lam" / "metadata_summary.json"
METHOD = "LAM"
PROTOCOL = "nano3d_table5_lam_source_edit_preflight_v1"
METRIC_NAMES = (
    "target_fulfilled",
    "anchor",
    "scale",
    "non_target_preserved",
    "geometry_locality",
    "structural_locality",
    "post_edit_constraint_pass",
    "regression_preservation",
    "final_pass",
    "edit_cost",
)


def contained(path: Path, *, strict: bool = False) -> Path:
    resolved = path.resolve(strict=strict)
    if resolved != WORKSPACE_ROOT and WORKSPACE_ROOT not in resolved.parents:
        raise ValueError(f"path is outside authorized workspace: {resolved}")
    return resolved


def relative(path: Path) -> str:
    return str(contained(path, strict=path.exists()).relative_to(WORKSPACE_ROOT))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with contained(path, strict=True).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(contained(path, strict=True).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def load_tasks() -> list[dict[str, Any]]:
    source = contained(TASK_SOURCE, strict=True).read_text(encoding="utf-8")
    module = ast.parse(source, filename=str(TASK_SOURCE))
    for node in module.body:
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            if node.target.id == "TASKS" and node.value is not None:
                value = ast.literal_eval(node.value)
                if not isinstance(value, tuple) or len(value) != 18:
                    raise ValueError("frozen TASKS must contain exactly 18 items")
                tasks = [dict(item) for item in value]
                ids = [str(item.get("task_id", "")) for item in tasks]
                if len(set(ids)) != 18 or any(not item for item in ids):
                    raise ValueError("frozen TASKS contain missing or duplicate task_id")
                return tasks
    raise ValueError("TASKS assignment not found")


def parquet_evidence() -> dict[str, Any]:
    evidence: dict[str, Any] = {
        "path": relative(LAM_PARQUET),
        "present": LAM_PARQUET.is_file() and not LAM_PARQUET.is_symlink(),
    }
    if not evidence["present"]:
        return evidence
    evidence["bytes"] = contained(LAM_PARQUET, strict=True).stat().st_size
    try:
        import pyarrow.parquet as pq

        parquet = pq.ParquetFile(contained(LAM_PARQUET, strict=True))
        columns = list(parquet.schema_arrow.names)
        required = ["threejs_code", "urdf", "articulation_json", "links_hierarchy_json"]
        evidence.update(
            {
                "rows": parquet.metadata.num_rows,
                "columns": columns,
                "required_source_columns": required,
                "required_source_columns_present": all(name in columns for name in required),
            }
        )
    except Exception as exc:  # noqa: BLE001
        evidence["metadata_error"] = f"{type(exc).__name__}: {exc}"
        evidence["required_source_columns_present"] = False
    return evidence


def load_manifest(path: Path) -> tuple[list[dict[str, Any]], list[str]]:
    rows: list[dict[str, Any]] = []
    errors: list[str] = []
    try:
        lines = contained(path, strict=True).read_text(encoding="utf-8").splitlines()
    except (OSError, ValueError) as exc:
        return rows, [f"LAM parent manifest is unreadable: {type(exc).__name__}: {exc}"]
    for line_number, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            errors.append(f"manifest line {line_number}: invalid JSON: {exc}")
            continue
        if not isinstance(value, dict):
            errors.append(f"manifest line {line_number}: expected a JSON object")
            continue
        rows.append(value)
    if not rows:
        errors.append("LAM parent manifest contains no records")
    return rows, errors


def validate_artifact(raw: Any, field: str, index: int, suffixes: set[str]) -> tuple[str | None, list[str]]:
    errors: list[str] = []
    if not isinstance(raw, str) or not raw.strip():
        return None, [f"record {index}: missing {field}"]
    candidate = Path(raw)
    if not candidate.is_absolute():
        candidate = WORKSPACE_ROOT / candidate
    try:
        resolved = contained(candidate, strict=True)
    except (OSError, ValueError) as exc:
        return None, [f"record {index}: invalid {field}: {exc}"]
    if not resolved.is_file() or resolved.is_symlink():
        errors.append(f"record {index}: {field} must be a regular non-symlink file: {resolved}")
    if resolved.suffix.casefold() not in suffixes:
        errors.append(f"record {index}: {field} has unsupported suffix: {resolved.suffix}")
    return str(resolved), errors


def validate_manifest(rows: list[dict[str, Any]], tasks: list[dict[str, Any]]) -> list[str]:
    errors: list[str] = []
    expected = {item["task_id"] for item in tasks}
    seen: set[str] = set()
    for index, row in enumerate(rows, 1):
        task_id = row.get("task_id")
        if not isinstance(task_id, str) or task_id not in expected:
            errors.append(f"record {index}: task_id must be one of the frozen 18 IDs")
        elif task_id in seen:
            errors.append(f"record {index}: duplicate task_id {task_id}")
        else:
            seen.add(task_id)
        if row.get("method") != METHOD:
            errors.append(f"record {index}: method must be exactly {METHOD!r}")
        provenance = row.get("provenance")
        if not isinstance(provenance, dict) or not all(
            str(provenance.get(key, "")).strip()
            for key in ("object_release_id", "dataset_revision", "generation_record")
        ):
            errors.append(
                f"record {index}: provenance requires object_release_id, dataset_revision, and generation_record"
            )
        for field, suffixes in (
            ("parent_source", {".js", ".mjs", ".cjs"}),
            ("parent_urdf", {".urdf"}),
            ("parent_glb", {".glb"}),
        ):
            _, field_errors = validate_artifact(row.get(field), field, index, suffixes)
            errors.extend(field_errors)
    missing = sorted(expected - seen)
    extra = sorted(seen - expected)
    if missing:
        errors.append("manifest is missing frozen task IDs: " + ", ".join(missing))
    if extra:
        errors.append("manifest has unexpected task IDs: " + ", ".join(extra))
    if len(rows) != 18:
        errors.append(f"manifest must contain exactly 18 records, found {len(rows)}")
    return errors


def edit_runner_evidence(path: Path) -> tuple[dict[str, Any], list[str]]:
    evidence = {"path": relative(path), "present": path.is_file() and not path.is_symlink()}
    if not evidence["present"]:
        return evidence, [f"LAM source-edit adapter is missing: {contained(path)}"]
    source = contained(path, strict=True).read_text(encoding="utf-8", errors="replace")
    required_flags = ["--parent-source", "--instruction", "--output-dir"]
    evidence["sha256"] = sha256(path)
    evidence["required_cli_flags"] = required_flags
    evidence["required_cli_flags_present"] = all(flag in source for flag in required_flags)
    if not evidence["required_cli_flags_present"]:
        return evidence, ["LAM source-edit adapter does not expose the required parent/instruction/output CLI"]
    return evidence, []


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--edit-runner", type=Path, default=DEFAULT_EDIT_RUNNER)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    output = contained(args.output)
    output.mkdir(parents=True, exist_ok=True)
    tasks = load_tasks()

    table6_preflight = read_json(TABLE6_PREFLIGHT) if TABLE6_PREFLIGHT.is_file() else {}
    table6_summary = read_json(TABLE6_SUMMARY) if TABLE6_SUMMARY.is_file() else {}
    parquet = parquet_evidence()
    run_pipeline = LAM_CODE / "run_pipeline.py"
    pipeline_source = (
        contained(run_pipeline, strict=True).read_text(encoding="utf-8")
        if run_pipeline.is_file()
        else ""
    )
    source_edit_flags = ["--edit", "--parent-source", "--source-program", "--edit-instruction"]
    official_evidence = {
        "code_root": relative(LAM_CODE),
        "code_present": LAM_CODE.is_dir() and not LAM_CODE.is_symlink(),
        "run_pipeline_present": run_pipeline.is_file(),
        "run_pipeline_sha256": sha256(run_pipeline) if run_pipeline.is_file() else None,
        "public_cli_mode": "text-to-new-asset generation",
        "public_source_edit_flags": {flag: flag in pipeline_source for flag in source_edit_flags},
        "public_source_edit_api_present": any(flag in pipeline_source for flag in source_edit_flags),
        "runtime_config_present": (LAM_CODE / "config.yaml").is_file(),
        "node_modules_present": (LAM_CODE / "node_modules").is_dir(),
        "dataset": parquet,
        "table6_preflight": relative(TABLE6_PREFLIGHT),
        "table6_preflight_sha256": sha256(TABLE6_PREFLIGHT) if TABLE6_PREFLIGHT.is_file() else None,
        "recorded_code_commit": table6_preflight.get("official_sources", {}).get("code", {}).get("commit"),
        "recorded_dataset_revision": table6_preflight.get("official_sources", {}).get("dataset", {}).get("revision"),
        "recorded_release_rows": table6_summary.get("scope", {}).get("all_release_rows"),
        "recorded_generated_code_executed": table6_summary.get("scope", {}).get("generated_code_executed"),
    }

    blockers: list[str] = []
    if not official_evidence["code_present"] or not official_evidence["run_pipeline_present"]:
        blockers.append("official LAM code materialization is missing or incomplete")
    if not parquet.get("present") or not parquet.get("required_source_columns_present"):
        blockers.append("official LAM generated-source release is missing or lacks required source/URDF columns")

    manifest = contained(args.manifest)
    rows: list[dict[str, Any]] = []
    manifest_errors: list[str] = []
    if not manifest.is_file() or manifest.is_symlink():
        blockers.append(f"LAM 18-task parent manifest is missing: {manifest}")
    else:
        rows, manifest_errors = load_manifest(manifest)
        manifest_errors.extend(validate_manifest(rows, tasks))
        blockers.extend(manifest_errors)

    edit_runner, runner_errors = edit_runner_evidence(contained(args.edit_runner))
    blockers.extend(runner_errors)
    if not official_evidence["public_source_edit_api_present"]:
        blockers.append(
            "official LAM public CLI exposes text-to-new-asset generation, not parent-source local editing"
        )

    task_records = [
        {
            "task_id": task["task_id"],
            "edit_class": task["edit_class"],
            "instruction": task["instruction"],
            "status": "N/R",
            "reason": "preflight blocked before source editing",
        }
        for task in tasks
    ]
    metrics = {name: "N/R" for name in METRIC_NAMES}
    metrics["16_seed_propagation"] = "N/A (LAM is a per-asset source-program method)"
    summary = {
        "protocol_id": PROTOCOL,
        "method": METHOD,
        "status": "BLOCKED" if blockers else "READY_NOT_RUN",
        "validated": False,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "workspace_root": str(WORKSPACE_ROOT),
        "network_accessed": False,
        "paper_values_reused": False,
        "other_method_artifacts_substituted": False,
        "frozen_task_source": relative(TASK_SOURCE),
        "frozen_task_source_sha256": sha256(TASK_SOURCE),
        "task_count": len(tasks),
        "edit_class_counts": {
            name: sum(task["edit_class"] == name for task in tasks)
            for name in ("additive", "modified-existing")
        },
        "official_lam_materialization": official_evidence,
        "manifest": {
            "path": relative(manifest),
            "present": manifest.is_file() and not manifest.is_symlink(),
            "record_count": len(rows),
            "validation_errors": manifest_errors,
            "required_fields": [
                "task_id", "method=LAM", "parent_source", "parent_urdf", "parent_glb",
                "provenance.object_release_id", "provenance.dataset_revision",
                "provenance.generation_record",
            ],
        },
        "edit_runner": edit_runner,
        "blockers": blockers,
        "metrics": metrics,
        "task_records": task_records,
        "missing_evaluation_inputs": [
            "frozen per-task semantic anchor and scale gold for LAM parent assets",
            "post-edit constraint specifications and measurement recipes",
            "independent historical regression manifest",
            "two blinded reviewer labels plus adjudication for semantic/locality/final-pass fields",
        ],
        "required_to_run": [
            "an explicit provenance-checked 18-row manifest mapping E001-E018 to LAM parent source/URDF/GLB",
            "a callable LAM source-edit adapter that consumes the parent source and frozen instruction and writes edited source/URDF/GLB",
            "a fully local runnable LAM environment for that adapter; credentials are neither searched nor inferred by this preflight",
        ],
        "disposition": (
            "No edit was launched and every measured Table 5 cell remains N/R. "
            "The official release source/URDF corpus is evidence of generation, not evidence of a local edit."
        ),
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    blocker_lines = "\n".join(f"- {item}" for item in blockers) or "- None"
    report = f"""# LAM Table 5 Editability Preflight

- Status: **{summary['status']}**
- Frozen tasks: `{len(tasks)}` (`13` additive + `5` modified-existing)
- Network accessed: `false`
- Paper values reused: `false`
- Other-method artifacts substituted: `false`

## What is locally available

The official LAM code and official Articulated-Object-Code release are locally materialized.
The parquet contains `{parquet.get('rows', 'N/R')}` rows and the generated `threejs_code`,
URDF, articulation, and hierarchy columns. Table 6 separately audited this release, but its
recorded `generated_code_executed` value is `{official_evidence['recorded_generated_code_executed']}`.

This availability does not establish a Table 5 edit run. The public `run_pipeline.py` CLI
generates a new asset from a text description; it does not accept a parent source program
and local edit instruction.

## Blocking reasons

{blocker_lines}

## Table 5 disposition

`Target Fulfilled`, `Anchor`, `Scale`, `Non-Target Preserved`, `Geometry Locality`,
`Structural Locality`, `Post-Edit Constraint Pass`, `Regression Preservation`,
`Final Pass`, and `Edit Cost` are all `N/R`. `16-Seed Propagation` is `N/A` because
LAM is evaluated as a per-asset method.

No edit command was launched. No LAM score was inferred from release URDFs, paper numbers,
or assets generated by another method.
"""
    (output / "report.md").write_text(report, encoding="utf-8")
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    return 2 if blockers else 0


if __name__ == "__main__":
    raise SystemExit(main())
