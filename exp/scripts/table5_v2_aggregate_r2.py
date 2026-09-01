#!/usr/bin/env python3
"""Aggregate revision-2 Table 5 records with independent metric boundaries."""

from __future__ import annotations

from pathlib import Path
import sys
from typing import Any, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_v2_aggregate as _core  # noqa: E402
import table5_v2_runtime_r2 as _runtime_r2  # noqa: E402


SCHEMA_VERSION = "table5_v2_aggregate_v3"
V2_EVIDENCE_SCHEMA = _runtime_r2.V2_EVIDENCE_SCHEMA
_ORIGINAL_LOAD_MANIFEST = _core._load_manifest
_ORIGINAL_DATASET_SUMMARY = _core._dataset_summary
_INSTALLED = False


def _valid_receipt(receipt: Any, schema: str) -> bool:
    return bool(
        isinstance(receipt, Mapping)
        and receipt.get("schema_version") == schema
        and receipt.get("receipt_sha256")
        == _core.canonical_sha256(receipt, exclude_fields=("receipt_sha256",))
    )


def _import_pass(record: Mapping[str, Any] | None) -> bool:
    evidence = _core._v2(record)
    imported = evidence.get("import")
    return bool(
        _valid_receipt(imported, _runtime_r2.IMPORT_RECEIPT_SCHEMA)
        and imported.get("passed") is True
        and imported.get("criterion") == "native_simulator_asset_load"
    )


def _stable_pass(record: Mapping[str, Any] | None) -> bool:
    evidence = _core._v2(record)
    stable = evidence.get("stable_rollout")
    if not _import_pass(record) or not _valid_receipt(
        stable, _runtime_r2.STABLE_RECEIPT_SCHEMA
    ):
        return False
    checks = stable.get("checks")
    return bool(
        stable.get("passed") is True
        and isinstance(checks, Mapping)
        and checks
        and all(value is True for value in checks.values())
        and stable.get("steps_completed") == stable.get("required_steps")
        and stable.get("finite_state_steps") == stable.get("required_steps")
        and stable.get("finite_pose_steps") == stable.get("required_steps")
        and stable.get("error") is None
    )


def _diagnostic_rows(
    record: Mapping[str, Any] | None, field: str
) -> dict[str, Mapping[str, Any]]:
    if not isinstance(record, Mapping) or not _import_pass(record):
        return {}
    evaluation = record.get("evaluation")
    diagnostics = (
        evaluation.get("diagnostics") if isinstance(evaluation, Mapping) else None
    )
    rows = diagnostics.get(field) if isinstance(diagnostics, Mapping) else None
    if not isinstance(rows, list):
        return {}
    result: dict[str, Mapping[str, Any]] = {}
    for row in rows:
        name = row.get("joint_name") if isinstance(row, Mapping) else None
        if isinstance(name, str) and name not in result:
            result[name] = row
    return result


def _actuation_by_name(
    record: Mapping[str, Any] | None,
) -> dict[str, Mapping[str, Any]]:
    return _diagnostic_rows(record, "actuation")


def _limit_by_name(
    record: Mapping[str, Any] | None,
) -> dict[str, Mapping[str, Any]]:
    return _diagnostic_rows(record, "limit_enforcement")


def _trajectory_candidate_count(rows: Sequence[Mapping[str, Any]]) -> int:
    return sum(
        1
        for row in rows
        for joint in row.get("scalar_joints", [])
        if isinstance(joint, Mapping) and joint.get("type") in {"revolute", "prismatic"}
    )


def _dataset_summary(
    dataset: Mapping[str, Any],
    records: Mapping[str, Sequence[dict[str, Any] | None]],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    summary = _ORIGINAL_DATASET_SUMMARY(dataset, records, protocol)
    candidate = _trajectory_candidate_count(dataset["rows"])
    tracking = summary["table5a"]["tracking_nrmse_p95"]
    limits = summary["table5a"]["limit_violation_p95"]
    tracking["candidate_units"] = candidate
    limits["candidate_units"] = candidate
    tracking["coverage_percentage"] = (
        0.0 if candidate == 0 else 100.0 * tracking["evaluated_units"] / candidate
    )
    limits["coverage_percentage"] = (
        0.0 if candidate == 0 else 100.0 * limits["evaluated_units"] / candidate
    )
    summary["declared_trajectory_joint_count"] = candidate
    summary["table5a"]["trajectory_coverage"] = _core._rate(
        tracking["evaluated_units"], candidate, ci=None
    )
    return summary


def _failure_reason(record: Mapping[str, Any] | None) -> str:
    if record is None:
        return "MISSING_OR_UNTRUSTED_RECORD"
    status = str(record.get("terminal_status") or "UNKNOWN").upper()
    if status != "COMPLETED":
        return status
    if not _import_pass(record):
        return "IMPORT_FAILED"
    if not _stable_pass(record):
        return "PASSIVE_STABILITY_FAILED"
    return "PASS"


def _load_manifest(path: Path) -> dict[str, Any]:
    manifest = _ORIGINAL_LOAD_MANIFEST(path)
    protocol = manifest.get("protocol")
    if (
        not isinstance(protocol, Mapping)
        or protocol.get("schema_version") != _runtime_r2.PROTOCOL_SCHEMA
        or protocol.get("protocol_id") != _runtime_r2.PROTOCOL_ID
        or protocol.get("v2_metrics", {}).get("metric_semantics_id")
        != _runtime_r2.METRIC_SEMANTICS_ID
    ):
        raise _core.AggregateError("prepared manifest does not use revision-2 metrics")
    implementation = protocol.get("implementation")
    expected_hash = _runtime_r2._hash(SCRIPT_PATH)
    if (
        not isinstance(implementation, Mapping)
        or implementation.get("aggregate_script_sha256") != expected_hash
    ):
        raise _core.AggregateError("aggregate implementation hash mismatch")
    return manifest


def _rate_cell(value: Mapping[str, Any]) -> str:
    return f"{float(value['percentage']):.2f}"


def _continuous_cell(value: Mapping[str, Any]) -> str:
    evaluated = int(value.get("evaluated_units", 0))
    candidate = int(value.get("candidate_units", 0))
    p95 = value.get("p95")
    if p95 is None:
        return f"N/E ({evaluated}/{candidate})"
    return f"{float(p95):.4f} ({evaluated}/{candidate})"


def render_markdown(summary: Mapping[str, Any]) -> str:
    lines = [
        "# Table 5 revision-2 results",
        "",
        f"Run classification: **{summary['classification']}**.",
        "",
        "## Table 5a: Genesis Simulation Readiness",
        "",
        "| Dataset | N | Import Success (%) ↑ | DoF Coverage (%) ↑ | Stable Rollout (%) ↑ | Trajectory Coverage (%) ↑ | Tracking NRMSE P95 (%) ↓ | Limit Violation P95 (%) ↓ |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for dataset in summary["datasets"]:
        table = dataset["table5a"]
        lines.append(
            f"| {dataset['dataset_name']} | {dataset['n']} | "
            f"{_rate_cell(table['import_success'])} | "
            f"{_rate_cell(table['dof_coverage'])} | "
            f"{_rate_cell(table['stable_rollout'])} | "
            f"{_rate_cell(table['trajectory_coverage'])} | "
            f"{_continuous_cell(table['tracking_nrmse_p95'])} | "
            f"{_continuous_cell(table['limit_violation_p95'])} |"
        )
    lines.extend(
        [
            "",
            "## Table 5b: Cross-Simulator Portability",
            "",
            "| Dataset | N | Genesis Import Success (%) ↑ | Genesis DoF (%) ↑ | Genesis Stable Rollout (%) ↑ | PyBullet Import Success (%) ↑ | PyBullet DoF (%) ↑ | PyBullet Stable Rollout (%) ↑ | MuJoCo Import Success (%) ↑ | MuJoCo DoF (%) ↑ | MuJoCo Stable Rollout (%) ↑ |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for dataset in summary["datasets"]:
        table = dataset["table5b"]
        lines.append(
            f"| {dataset['dataset_name']} | {dataset['n']} | "
            f"{_rate_cell(table['genesis']['import_success'])} | "
            f"{_rate_cell(table['genesis']['dof_coverage'])} | "
            f"{_rate_cell(table['genesis']['stable_rollout'])} | "
            f"{_rate_cell(table['pybullet']['import_success'])} | "
            f"{_rate_cell(table['pybullet']['dof_coverage'])} | "
            f"{_rate_cell(table['pybullet']['stable_rollout'])} | "
            f"{_rate_cell(table['mujoco']['import_success'])} | "
            f"{_rate_cell(table['mujoco']['dof_coverage'])} | "
            f"{_rate_cell(table['mujoco']['stable_rollout'])} |"
        )
    lines.extend(
        [
            "",
            "**Table 5b, kinematic diagnostics**",
            "",
            "| Dataset | Genesis FK Position P95 (% diag.) ↓ | PyBullet FK Position P95 (% diag.) ↓ | MuJoCo FK Position P95 (% diag.) ↓ | Genesis FK Rotation P95 (deg) ↓ | PyBullet FK Rotation P95 (deg) ↓ | MuJoCo FK Rotation P95 (deg) ↓ |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for dataset in summary["datasets"]:
        table = dataset["table5b"]
        lines.append(
            f"| {dataset['dataset_name']} | "
            f"{_continuous_cell(table['genesis']['fk_position_error_p95'])} | "
            f"{_continuous_cell(table['pybullet']['fk_position_error_p95'])} | "
            f"{_continuous_cell(table['mujoco']['fk_position_error_p95'])} | "
            f"{_continuous_cell(table['genesis']['fk_rotation_error_p95'])} | "
            f"{_continuous_cell(table['pybullet']['fk_rotation_error_p95'])} | "
            f"{_continuous_cell(table['mujoco']['fk_rotation_error_p95'])} |"
        )
    lines.extend(
        [
            "",
            "Import Success records native asset-load acceptance only. Stable Rollout is an independent fixed-step zero-force passive finite-state test.",
            "Continuous cells show `P95 (evaluated/candidate)`; N/E never counts as a successful continuous evaluation.",
            "",
        ]
    )
    return "\n".join(lines)


def install() -> None:
    global _INSTALLED
    if _INSTALLED:
        return
    _core.SCHEMA_VERSION = SCHEMA_VERSION
    _core.V2_EVIDENCE_SCHEMA = V2_EVIDENCE_SCHEMA
    _core._load_manifest = _load_manifest
    _core._import_pass = _import_pass
    _core._stable_pass = _stable_pass
    _core._actuation_by_name = _actuation_by_name
    _core._limit_by_name = _limit_by_name
    _core._dataset_summary = _dataset_summary
    _core._failure_reason = _failure_reason
    _core.render_markdown = render_markdown
    _INSTALLED = True


def main(argv: Sequence[str] | None = None) -> int:
    install()
    return _core.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
