#!/usr/bin/env python3
"""Aggregate the partial Table 5 plan: Genesis full, PyBullet/MuJoCo load-only.

Genesis records are read from::

    <run-root>/runtime/<dataset-slug>/genesis/assets/<dataset-id>.json

Strict load-only records are read from::

    <load-only-root>/<dataset-slug>/<simulator>/assets/<dataset-id>.json

Only metrics in the declared execution plan are scored. Missing or invalid planned
records fail closed without leaving the full dataset denominator. Metrics excluded
from the plan are reported as N/E rather than as zero-percent failures.
"""

from __future__ import annotations

import argparse
import csv
import io
import json
import os
import re
import sys
import tempfile
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_n200_aggregate as _full  # noqa: E402


SCHEMA_VERSION = "table5_n200_hybrid_aggregate_v1"
LOAD_ONLY_ASSET_SCHEMA = "table5_n200_load_only_asset_v1"
LOAD_ONLY_EVALUATION_SCHEMA = "table5_n200_load_only_evaluation_v1"
LOAD_ONLY_PROFILE_NAME = "strict_load_only_v1"
LOAD_ONLY_SIMULATORS = ("pybullet", "mujoco")
SIMULATORS = ("pybullet", "genesis", "mujoco")
TABLE5A_METRICS = _full.TABLE5A_METRICS
STRICT_GATE_NAMES = _full.STRICT_GATE_NAMES
NOT_EVALUATED_METRICS = tuple(metric for metric in TABLE5A_METRICS if metric != "load")
LOAD_ONLY_TERMINAL_STATUSES = {
    "completed",
    "preflight_failure",
    "diagnostic_failure",
    "timeout",
    "native_crash",
    "malformed_response",
    "worker_error",
    "parent_error",
}
LOAD_ONLY_PROFILE_CONTRACT = {
    "execution_profile": LOAD_ONLY_PROFILE_NAME,
    "simulators": list(LOAD_ONLY_SIMULATORS),
    "planned_metrics": ["load"],
    "not_evaluated_metrics": list(NOT_EVALUATED_METRICS),
    "load_definition": (
        "native adapter construction followed by exact observed/expected URDF "
        "link and joint name multisets"
    ),
    "pybullet_actual_state_capacity": "not_applicable_to_load_only",
    "failure_policy": "fail_closed_without_denominator_removal",
}
LOAD_ONLY_PROFILE_SHA256 = _full._canonical_sha256(LOAD_ONLY_PROFILE_CONTRACT)
_HEX_SHA256 = re.compile(r"[0-9a-f]{64}")

AggregateError = _full.AggregateError


@dataclass(frozen=True)
class LoadOnlyRecord:
    path: Path
    record: dict[str, Any] | None
    state: str
    terminal_status: str | None

    @property
    def completed(self) -> bool:
        return self.state == "valid" and self.terminal_status == "completed"


def _rate(passed: int, denominator: int) -> dict[str, Any]:
    return _full._rate(passed, denominator)


def _not_evaluable(denominator: int, reason: str) -> dict[str, Any]:
    return _full._not_evaluable(denominator, reason)


def _profile_fields_match(value: Mapping[str, Any]) -> bool:
    return bool(
        value.get("execution_profile") == LOAD_ONLY_PROFILE_NAME
        and value.get("execution_profile_sha256") == LOAD_ONLY_PROFILE_SHA256
        and value.get("planned_metrics") == ["load"]
        and value.get("not_evaluated_metrics") == list(NOT_EVALUATED_METRICS)
    )


def _read_load_only_record(
    path: Path,
    *,
    row: Mapping[str, Any],
    dataset_slug: str,
    dataset_name: str,
    simulator: str,
    manifest_sha256: str,
    protocol_sha256: str,
) -> LoadOnlyRecord:
    if not path.is_file():
        return LoadOnlyRecord(path, None, "missing", None)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return LoadOnlyRecord(path, None, "invalid_json", None)
    if not isinstance(value, dict):
        return LoadOnlyRecord(path, None, "malformed", None)
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError):
        return LoadOnlyRecord(path, value, "malformed", None)

    status = value.get("terminal_status")
    if (
        value.get("schema_version") != LOAD_ONLY_ASSET_SCHEMA
        or value.get("terminal") is not True
        or not isinstance(status, str)
        or status not in LOAD_ONLY_TERMINAL_STATUSES
        or not _profile_fields_match(value)
    ):
        return LoadOnlyRecord(path, value, "malformed", status)
    metrics = value.get("metrics")
    evaluation = value.get("evaluation")
    process = value.get("process")
    if (
        not isinstance(metrics, dict)
        or set(metrics) != {"load"}
        or not isinstance(metrics.get("load"), bool)
        or not isinstance(evaluation, dict)
        or evaluation.get("schema_version") != LOAD_ONLY_EVALUATION_SCHEMA
        or not _profile_fields_match(evaluation)
        or evaluation.get("metrics") != metrics
        or not isinstance(process, dict)
    ):
        return LoadOnlyRecord(path, value, "malformed", status)

    failure = value.get("failure")
    if status == "completed":
        load_evidence = evaluation.get("load")
        if (
            failure is not None
            or not isinstance(load_evidence, dict)
            or load_evidence.get("strict_load") is not metrics["load"]
        ):
            return LoadOnlyRecord(path, value, "malformed", status)
    elif metrics["load"] or not isinstance(failure, dict):
        return LoadOnlyRecord(path, value, "malformed", status)

    identity = value.get("identity")
    if not isinstance(identity, dict):
        return LoadOnlyRecord(path, value, "identity_mismatch", status)
    bindings: dict[str, Any] = {
        "dataset_slug": dataset_slug,
        "dataset_name": dataset_name,
        "dataset_id": row["dataset_id"],
        "simulator": simulator,
        "manifest_sha256": manifest_sha256,
        "protocol_sha256": protocol_sha256,
        "row_sha256": row.get("row_sha256"),
        "execution_profile": LOAD_ONLY_PROFILE_NAME,
        "execution_profile_sha256": LOAD_ONLY_PROFILE_SHA256,
    }
    for field in ("asset_id", "urdf_sha256"):
        if field in row:
            bindings[field] = row[field]
    required_string_fields = ("urdf_path", "executable", "runner_source_sha256")
    if (
        not isinstance(row.get("row_sha256"), str)
        or any(identity.get(field) != expected for field, expected in bindings.items())
        or not isinstance(identity.get("execution_profile_sha256"), str)
        or _HEX_SHA256.fullmatch(identity["execution_profile_sha256"]) is None
        or any(
            not isinstance(identity.get(field), str) or not identity[field]
            for field in required_string_fields
        )
        or _HEX_SHA256.fullmatch(identity["runner_source_sha256"]) is None
        or not _full._finite_number(identity.get("timeout_s"))
        or float(identity["timeout_s"]) <= 0
        or not isinstance(identity.get("effective_workers"), int)
        or isinstance(identity.get("effective_workers"), bool)
        or identity["effective_workers"] <= 0
    ):
        return LoadOnlyRecord(path, value, "identity_mismatch", status)
    return LoadOnlyRecord(path, value, "valid", status)


def _load_only_path(root: Path, slug: str, simulator: str, dataset_id: str) -> Path:
    return root / slug / simulator / "assets" / f"{dataset_id}.json"


def _load_pass(record: Any) -> bool:
    return bool(
        record.completed
        and record.record is not None
        and record.record["metrics"].get("load") is True
    )


def _record_completeness(
    records: Mapping[str, Mapping[str, Any]], denominator: int
) -> dict[str, Any]:
    by_simulator: dict[str, dict[str, Any]] = {}
    for simulator in SIMULATORS:
        simulator_records = list(records[simulator].values())
        states = Counter(record.state for record in simulator_records)
        statuses = Counter(
            record.terminal_status if record.state == "valid" else record.state
            for record in simulator_records
        )
        present = denominator - states["missing"]
        valid = states["valid"]
        terminal = valid
        by_simulator[simulator] = {
            "source_profile": (
                "full_table5_v1" if simulator == "genesis" else LOAD_ONLY_PROFILE_NAME
            ),
            "expected_records": denominator,
            "present_records": present,
            "valid_records": valid,
            "terminal_records": terminal,
            "classification": ("COMPLETE" if terminal == denominator else "INCOMPLETE"),
            "record_state_counts": dict(sorted(states.items())),
            "terminal_status_counts": dict(sorted(statuses.items())),
        }
    expected = len(SIMULATORS) * denominator
    present = sum(item["present_records"] for item in by_simulator.values())
    valid = sum(item["valid_records"] for item in by_simulator.values())
    terminal = sum(item["terminal_records"] for item in by_simulator.values())
    return {
        "classification": "COMPLETE" if terminal == expected else "INCOMPLETE",
        "expected_records": expected,
        "present_records": present,
        "valid_records": valid,
        "terminal_records": terminal,
        "missing_or_invalid_records": expected - terminal,
        "by_simulator": by_simulator,
        "rule": (
            "one schema-valid identity-bound terminal record per manifest row for "
            "Genesis full plus PyBullet/MuJoCo strict load-only; terminal failures "
            "count as complete"
        ),
    }


def _upstream_rates(rows: Sequence[Mapping[str, Any]]) -> dict[str, dict[str, Any]]:
    denominator = len(rows)
    rates: dict[str, dict[str, Any]] = {}
    for gate in STRICT_GATE_NAMES:
        values = [row["_strict_gates"][gate] for row in rows]
        known = [value for value in values if isinstance(value, bool)]
        if len(known) == denominator:
            rates[gate] = _rate(sum(known), denominator)
        else:
            rates[gate] = _full._not_evaluable(
                denominator,
                "upstream_gate_not_evaluable",
                known_passed=sum(known),
                known_denominator=len(known),
            )
    return rates


def _candidate_counts(rows: Sequence[Mapping[str, Any]]) -> tuple[dict[str, int], int]:
    joints = {"revolute": 0, "prismatic": 0}
    poses = 0
    for row in rows:
        for joint in row["scalar_joints"]:
            if not _full._cross_sim_eligible_joint(joint):
                continue
            joint_type = joint["type"]
            joints[joint_type] += 1
            poses += len(_full._descendants(row, joint.get("child")))
    return joints, poses


def _joint_diagnostic_ne(candidate_units: int, threshold: float) -> dict[str, Any]:
    reason = "cross_simulator_diagnostics_not_in_planned_scope"
    return {
        "status": "not_evaluable",
        "reason": reason,
        "candidate_units": candidate_units,
        "evaluable_units": None,
        "excluded_units": None,
        "available_pairs": None,
        "coverage": _not_evaluable(candidate_units, reason),
        "threshold": threshold,
        "units_within_threshold": None,
        "population_max": None,
        "mean": None,
        "median": None,
    }


def _pose_diagnostic_ne(
    candidate_units: int, thresholds: Mapping[str, float]
) -> dict[str, Any]:
    reason = "cross_simulator_diagnostics_not_in_planned_scope"
    return {
        "status": "not_evaluable",
        "reason": reason,
        "candidate_units": candidate_units,
        "evaluable_units": None,
        "excluded_units": None,
        "available_pairs": None,
        "coverage": _not_evaluable(candidate_units, reason),
        "translation_over_bbox_diagonal": {
            "status": "not_evaluable",
            "reason": reason,
            "threshold": thresholds["translation_over_bbox_diagonal"],
            "units_within_threshold": None,
            "population_max": None,
            "mean": None,
            "median": None,
        },
        "rotation_rad": {
            "status": "not_evaluable",
            "reason": reason,
            "threshold": thresholds["rotation_rad"],
            "units_within_threshold": None,
            "population_max": None,
            "mean": None,
            "median": None,
        },
    }


def _aggregate_dataset(
    slug: str,
    dataset: Mapping[str, Any],
    run_root: Path,
    load_only_root: Path,
    thresholds: Mapping[str, float],
    manifest_sha256: str,
    protocol_sha256: str,
) -> dict[str, Any]:
    rows: list[dict[str, Any]] = dataset["rows"]
    denominator = len(rows)
    records: dict[str, dict[str, Any]] = {simulator: {} for simulator in SIMULATORS}
    for row in rows:
        dataset_id = row["dataset_id"]
        records["genesis"][dataset_id] = _full._read_runtime_record(
            _full._record_path(run_root, slug, "genesis", dataset_id),
            row=row,
            dataset_slug=slug,
            dataset_name=dataset["name"],
            simulator="genesis",
            manifest_sha256=manifest_sha256,
            protocol_sha256=protocol_sha256,
        )
        for simulator in LOAD_ONLY_SIMULATORS:
            records[simulator][dataset_id] = _read_load_only_record(
                _load_only_path(load_only_root, slug, simulator, dataset_id),
                row=row,
                dataset_slug=slug,
                dataset_name=dataset["name"],
                simulator=simulator,
                manifest_sha256=manifest_sha256,
                protocol_sha256=protocol_sha256,
            )

    load_by_id = {
        row["dataset_id"]: {
            simulator: _load_pass(records[simulator][row["dataset_id"]])
            for simulator in SIMULATORS
        }
        for row in rows
    }
    table5a: dict[str, dict[str, dict[str, Any]]] = {}
    for simulator in SIMULATORS:
        table5a[simulator] = {}
        for metric in TABLE5A_METRICS:
            if simulator in LOAD_ONLY_SIMULATORS and metric != "load":
                table5a[simulator][metric] = _not_evaluable(
                    denominator, "metric_not_in_planned_scope"
                )
                continue
            passed = sum(
                (
                    load_by_id[row["dataset_id"]][simulator]
                    if metric == "load"
                    else _full._metric_pass(
                        records[simulator][row["dataset_id"]], metric
                    )
                )
                for row in rows
            )
            table5a[simulator][metric] = _rate(passed, denominator)

    all_three_load_by_id = {
        dataset_id: all(simulator_values.values())
        for dataset_id, simulator_values in load_by_id.items()
    }
    candidate_joints, candidate_poses = _candidate_counts(rows)
    joint_diagnostics = {
        joint_type: _joint_diagnostic_ne(
            candidate_joints[joint_type], thresholds["normalized_joint_rmse"]
        )
        for joint_type in ("revolute", "prismatic")
    }
    link_pose = _pose_diagnostic_ne(candidate_poses, thresholds)
    cross_sim_reason = "cross_simulator_diagnostics_not_in_planned_scope"
    runtime_reason = "required_runtime_metrics_not_in_planned_scope"
    strict_reason = "required_runtime_and_cross_simulator_metrics_not_in_planned_scope"
    upstream = _upstream_rates(rows)
    completeness = _record_completeness(records, denominator)

    table5b = {
        "per_simulator_pass": {
            "pybullet": _not_evaluable(denominator, "metric_not_in_planned_scope"),
            "genesis": table5a["genesis"]["simulator_pass"],
            "mujoco": _not_evaluable(denominator, "metric_not_in_planned_scope"),
        },
        "all_three_load": _rate(sum(all_three_load_by_id.values()), denominator),
        "all_three_runtime_pass": _not_evaluable(denominator, runtime_reason),
        "joint_normalized_trajectory_pairwise_max_rmse": joint_diagnostics,
        "final_descendant_link_pose_pairwise_max_error": link_pose,
        "coverage": {
            "runtime_records": {
                simulator: {
                    "valid_records": _rate(
                        completeness["by_simulator"][simulator]["valid_records"],
                        denominator,
                    ),
                    "completed_records": _rate(
                        sum(record.completed for record in records[simulator].values()),
                        denominator,
                    ),
                }
                for simulator in SIMULATORS
            },
            "joint_trajectory": {
                joint_type: joint_diagnostics[joint_type]["coverage"]
                for joint_type in ("revolute", "prismatic")
            },
            "descendant_link_pose": link_pose["coverage"],
        },
        "upstream_strict_gates": upstream,
        "strict_consistency": _not_evaluable(denominator, cross_sim_reason),
        "strict_sim_ready": _not_evaluable(denominator, strict_reason),
    }
    table5b["joint_rmse"] = joint_diagnostics
    table5b["link_pose_error"] = link_pose
    for gate in STRICT_GATE_NAMES:
        table5b[gate] = upstream[gate]

    asset_outcomes = []
    for row in rows:
        dataset_id = row["dataset_id"]
        asset_outcomes.append(
            {
                "dataset_id": dataset_id,
                "asset_id": row.get("asset_id"),
                "category": row["category"],
                "per_simulator_load": dict(load_by_id[dataset_id]),
                "per_simulator_pass": {
                    "pybullet": None,
                    "genesis": _full._metric_pass(
                        records["genesis"][dataset_id], "simulator_pass"
                    ),
                    "mujoco": None,
                },
                "all_three_load": all_three_load_by_id[dataset_id],
                "all_three_runtime_pass": None,
                "strict_consistency": None,
                "strict_gates": dict(row["_strict_gates"]),
                "strict_sim_ready": None,
            }
        )

    return {
        "name": dataset["name"],
        "slug": slug,
        "n": denominator,
        "classification": completeness["classification"],
        "table5_scope": "PARTIAL",
        "completeness": completeness,
        "categories": dict(sorted(Counter(row["category"] for row in rows).items())),
        "table5a": table5a,
        "table5b": table5b,
        "asset_outcomes": asset_outcomes,
    }


def aggregate_manifest(
    manifest: dict[str, Any],
    run_root: Path | str,
    load_only_root: Path | str,
) -> dict[str, Any]:
    manifest_sha256, protocol_sha256 = _full._authority_hashes(manifest)
    datasets_input = _full.validate_manifest(manifest)
    thresholds = _full._thresholds(manifest)
    datasets = {
        slug: _aggregate_dataset(
            slug,
            dataset,
            Path(run_root),
            Path(load_only_root),
            thresholds,
            manifest_sha256,
            protocol_sha256,
        )
        for slug, dataset in datasets_input.items()
    }
    expected = sum(
        item["completeness"]["expected_records"] for item in datasets.values()
    )
    present = sum(item["completeness"]["present_records"] for item in datasets.values())
    valid = sum(item["completeness"]["valid_records"] for item in datasets.values())
    terminal = sum(
        item["completeness"]["terminal_records"] for item in datasets.values()
    )
    completeness = {
        "classification": "COMPLETE" if terminal == expected else "INCOMPLETE",
        "expected_records": expected,
        "present_records": present,
        "valid_records": valid,
        "terminal_records": terminal,
        "missing_or_invalid_records": expected - terminal,
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "generated_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "manifest_sha256": manifest_sha256,
        "protocol_sha256": protocol_sha256,
        "classification": completeness["classification"],
        "table5_scope": "PARTIAL",
        "scope_reason": (
            "Genesis runs the full protocol; PyBullet and MuJoCo run strict load-only"
        ),
        "execution_plan": {
            "pybullet": {
                "profile": LOAD_ONLY_PROFILE_NAME,
                "profile_sha256": LOAD_ONLY_PROFILE_SHA256,
                "planned_metrics": ["load"],
            },
            "genesis": {
                "profile": "full_table5_v1",
                "planned_metrics": list(TABLE5A_METRICS),
            },
            "mujoco": {
                "profile": LOAD_ONLY_PROFILE_NAME,
                "profile_sha256": LOAD_ONLY_PROFILE_SHA256,
                "planned_metrics": ["load"],
            },
        },
        "simulators": list(SIMULATORS),
        "thresholds": thresholds,
        "completeness": completeness,
        "dataset_order": list(datasets),
        "datasets": datasets,
    }


aggregate = aggregate_manifest


def _rate_display(rate: Mapping[str, Any]) -> str:
    return _full._rate_display(rate)


def _rate_csv(row: dict[str, Any], prefix: str, rate: Mapping[str, Any]) -> None:
    _full._rate_csv(row, prefix, rate)


def render_table5a_csv(summary: Mapping[str, Any]) -> str:
    rows: list[dict[str, Any]] = []
    for slug in summary["dataset_order"]:
        dataset = summary["datasets"][slug]
        for simulator in SIMULATORS:
            row: dict[str, Any] = {
                "dataset_slug": slug,
                "dataset": dataset["name"],
                "simulator": simulator,
                "n": dataset["n"],
                "classification": dataset["classification"],
                "table5_scope": dataset["table5_scope"],
            }
            for metric in TABLE5A_METRICS:
                _rate_csv(row, metric, dataset["table5a"][simulator][metric])
            rows.append(row)
    fieldnames = (
        list(rows[0]) if rows else ["dataset_slug", "dataset", "simulator", "n"]
    )
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue()


def render_table5b_csv(summary: Mapping[str, Any]) -> str:
    rows: list[dict[str, Any]] = []
    for slug in summary["dataset_order"]:
        dataset = summary["datasets"][slug]
        table = dataset["table5b"]
        row: dict[str, Any] = {
            "dataset_slug": slug,
            "dataset": dataset["name"],
            "n": dataset["n"],
            "classification": dataset["classification"],
            "table5_scope": dataset["table5_scope"],
        }
        for simulator in SIMULATORS:
            _rate_csv(row, f"{simulator}_pass", table["per_simulator_pass"][simulator])
        _rate_csv(row, "all_three_load", table["all_three_load"])
        _rate_csv(row, "all_three_runtime_pass", table["all_three_runtime_pass"])
        for joint_type in ("revolute", "prismatic"):
            diagnostic = table["joint_rmse"][joint_type]
            row[f"{joint_type}_joint_status"] = diagnostic["status"]
            row[f"{joint_type}_joint_reason"] = diagnostic["reason"]
            row[f"{joint_type}_joint_candidate_units"] = diagnostic["candidate_units"]
            row[f"{joint_type}_joint_coverage"] = _rate_display(diagnostic["coverage"])
            row[f"{joint_type}_joint_pairwise_max_rmse"] = "N/E"
        pose = table["link_pose_error"]
        row["pose_status"] = pose["status"]
        row["pose_reason"] = pose["reason"]
        row["pose_candidate_units"] = pose["candidate_units"]
        row["pose_coverage"] = _rate_display(pose["coverage"])
        row["pose_pairwise_max_translation_over_bbox"] = "N/E"
        row["pose_pairwise_max_rotation_rad"] = "N/E"
        for gate in STRICT_GATE_NAMES:
            _rate_csv(row, gate, table["upstream_strict_gates"][gate])
        _rate_csv(row, "strict_consistency", table["strict_consistency"])
        _rate_csv(row, "strict_sim_ready", table["strict_sim_ready"])
        rows.append(row)
    fieldnames = list(rows[0]) if rows else ["dataset_slug", "dataset", "n"]
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue()


def render_report(summary: Mapping[str, Any]) -> str:
    completeness = summary["completeness"]
    completeness_line = (
        "> **INCOMPLETE: planned runtime records are missing or untrusted; planned "
        "rates remain provisional and fail closed.**"
        if completeness["classification"] == "INCOMPLETE"
        else "**Planned record completeness: COMPLETE.**"
    )
    lines = [
        "# Table 5 N=200 hybrid automatic evaluation",
        "",
        "> **TABLE 5 SCOPE: PARTIAL. Genesis runs the full protocol; PyBullet and "
        "MuJoCo run strict Load only. This report makes no full three-simulator "
        "Sim-ready claim.**",
        "",
        completeness_line,
        "",
        f"Trusted planned terminal coverage: {completeness['terminal_records']} / "
        f"{completeness['expected_records']} records.",
        "",
        "A missing or failed planned record remains in the full dataset denominator. "
        "Metrics excluded from the execution plan are N/E, never zero-percent failures.",
        "",
        "## Table 5a",
        "",
        "| Dataset | Record state | Simulator | Load | Reset | Settling | Actuation | Limits | Drift | Simulator pass |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for slug in summary["dataset_order"]:
        dataset = summary["datasets"][slug]
        for simulator in SIMULATORS:
            metrics = dataset["table5a"][simulator]
            cells = [_rate_display(metrics[metric]) for metric in TABLE5A_METRICS]
            state = dataset["completeness"]["by_simulator"][simulator]["classification"]
            lines.append(
                f"| {dataset['name']} | {state} | {simulator} | "
                + " | ".join(cells)
                + " |"
            )
    lines.extend(
        [
            "",
            "## Table 5b",
            "",
            "| Dataset | Record state | PyBullet pass | Genesis pass | MuJoCo pass | All-three load | All-three runtime | Joint RMSE | Link pose | Cross-sim coverage | Strict consistency | Strict Sim-ready |",
            "|---|---|---:|---:|---:|---:|---:|---|---|---|---:|---:|",
        ]
    )
    for slug in summary["dataset_order"]:
        dataset = summary["datasets"][slug]
        table = dataset["table5b"]
        cells = [
            _rate_display(table["per_simulator_pass"][simulator])
            for simulator in SIMULATORS
        ]
        cells.extend(
            [
                _rate_display(table["all_three_load"]),
                _rate_display(table["all_three_runtime_pass"]),
                "N/E",
                "N/E",
                "N/E",
                _rate_display(table["strict_consistency"]),
                _rate_display(table["strict_sim_ready"]),
            ]
        )
        lines.append(
            f"| {dataset['name']} | {dataset['classification']} | "
            + " | ".join(cells)
            + " |"
        )
    lines.extend(
        [
            "",
            "## Scope contract",
            "",
            f"Load-only profile: `{LOAD_ONLY_PROFILE_NAME}` "
            f"(`{LOAD_ONLY_PROFILE_SHA256}`).",
            "",
            "All-three Load is evaluated because Load is planned for every simulator. "
            "PyBullet/MuJoCo Pass, all-three Runtime, cross-simulator diagnostics, "
            "Strict consistency, and Strict Sim-ready are N/E by construction.",
            "",
        ]
    )
    return "\n".join(lines)


def _atomic_write_set(out: Path, payloads: Mapping[str, str]) -> None:
    out.mkdir(parents=True, exist_ok=True)
    staged: dict[str, Path] = {}
    try:
        for filename, content in payloads.items():
            descriptor, temporary = tempfile.mkstemp(
                dir=out, prefix=f".{filename}.", suffix=".tmp"
            )
            path = Path(temporary)
            staged[filename] = path
            with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
        for filename, path in staged.items():
            os.replace(path, out / filename)
        directory_fd = os.open(out, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        for path in staged.values():
            try:
                path.unlink()
            except FileNotFoundError:
                pass


def write_outputs(summary: Mapping[str, Any], out: Path | str) -> None:
    _atomic_write_set(
        Path(out),
        {
            "summary.json": json.dumps(
                summary,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                indent=2,
            )
            + "\n",
            "table5a.csv": render_table5a_csv(summary),
            "table5b.csv": render_table5b_csv(summary),
            "report.md": render_report(summary),
        },
    )


def _parse_args(argv: Iterable[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--load-only-root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        summary = aggregate_manifest(manifest, args.run_root, args.load_only_root)
        write_outputs(summary, args.out)
    except (AggregateError, OSError, UnicodeError, json.JSONDecodeError) as error:
        print(f"table5_n200_hybrid_aggregate: {error}", file=sys.stderr)
        return 2
    print(args.out / "summary.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
