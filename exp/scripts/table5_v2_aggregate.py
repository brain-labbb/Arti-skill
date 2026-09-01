#!/usr/bin/env python3
"""Aggregate frozen Table 5 v2 records into Table 5a and Table 5b.

Asset-level rates fail closed over N=200.  Continuous errors are conditional
and always carry explicit joint/sample coverage.  No missing runtime record is
silently removed from a success-rate denominator.
"""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import random
import tempfile
from typing import Any, Iterable, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SIMULATORS = ("genesis", "pybullet", "mujoco")
SCHEMA_VERSION = "table5_v2_aggregate_v2"
ASSET_RECORD_SCHEMA = "table5_n200_runtime_asset_v1"
V2_EVIDENCE_SCHEMA = "table5_v2_runtime_evidence_v2"
PREPARED_SCHEMA = "table5_v2_prepared_manifest_v1"
EXPECTED_N = 200


class AggregateError(ValueError):
    """Raised when frozen inputs or record bindings are malformed."""


def canonical_sha256(value: Any, *, exclude_fields: Iterable[str] = ()) -> str:
    excluded = set(exclude_fields)

    def filtered(item: Any) -> Any:
        if isinstance(item, dict):
            return {
                key: filtered(child)
                for key, child in item.items()
                if key not in excluded
            }
        if isinstance(item, list):
            return [filtered(child) for child in item]
        return item

    payload = json.dumps(
        filtered(value),
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _finite(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def percentile(values: Sequence[float], percent: float) -> float | None:
    """Return the linear-interpolated percentile (Hyndman-Fan type 7)."""

    if not values:
        return None
    if not 0.0 <= percent <= 100.0:
        raise AggregateError("percentile must be in [0, 100]")
    ordered = sorted(float(value) for value in values)
    if any(not math.isfinite(value) for value in ordered):
        raise AggregateError("percentile population contains non-finite values")
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _rate(passed: int, denominator: int, *, ci: tuple[float, float] | None) -> dict[str, Any]:
    if denominator < 0 or passed < 0 or passed > denominator:
        raise AggregateError("invalid rate numerator/denominator")
    return {
        "passed": passed,
        "denominator": denominator,
        "percentage": 0.0 if denominator == 0 else 100.0 * passed / denominator,
        "bootstrap_95_ci_percentage": (
            None if ci is None else [100.0 * ci[0], 100.0 * ci[1]]
        ),
    }


def _bootstrap_rate(
    outcomes: Sequence[bool], *, resamples: int, seed: str
) -> tuple[float, float] | None:
    if not outcomes:
        return None
    generator = random.Random(seed)
    count = len(outcomes)
    values = [
        sum(outcomes[generator.randrange(count)] for _ in range(count)) / count
        for _ in range(resamples)
    ]
    return float(percentile(values, 2.5)), float(percentile(values, 97.5))


def _bootstrap_asset_p95(
    by_asset: Sequence[Sequence[float]], *, resamples: int, seed: str
) -> tuple[float, float] | None:
    per_asset = [percentile(values, 95) for values in by_asset if values]
    clean = [float(value) for value in per_asset if value is not None]
    if not clean:
        return None
    generator = random.Random(seed)
    count = len(clean)
    estimates = [
        float(
            percentile(
                [clean[generator.randrange(count)] for _ in range(count)], 95
            )
        )
        for _ in range(resamples)
    ]
    return float(percentile(estimates, 2.5)), float(percentile(estimates, 97.5))


def _continuous(
    values_by_asset: Sequence[Sequence[float]],
    *,
    candidate_units: int,
    evaluated_units: int,
    scale: float,
    unit: str,
    resamples: int,
    seed: str,
) -> dict[str, Any]:
    values = [float(value) for asset in values_by_asset for value in asset]
    p95 = percentile(values, 95)
    median = percentile(values, 50)
    ci = _bootstrap_asset_p95(values_by_asset, resamples=resamples, seed=seed)
    return {
        "candidate_units": candidate_units,
        "evaluated_units": evaluated_units,
        "coverage_percentage": (
            0.0 if candidate_units == 0 else 100.0 * evaluated_units / candidate_units
        ),
        "sample_count": len(values),
        "median": None if median is None else scale * median,
        "p95": None if p95 is None else scale * p95,
        "p95_asset_cluster_bootstrap_95_ci": (
            None if ci is None else [scale * ci[0], scale * ci[1]]
        ),
        "unit": unit,
    }


def _load_manifest(path: Path) -> dict[str, Any]:
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise AggregateError(f"cannot load prepared manifest: {error}") from error
    if not isinstance(manifest, dict) or manifest.get("schema_version") != PREPARED_SCHEMA:
        raise AggregateError("prepared manifest schema mismatch")
    if manifest.get("manifest_sha256") != canonical_sha256(
        manifest, exclude_fields=("manifest_sha256",)
    ):
        raise AggregateError("prepared manifest hash mismatch")
    if manifest.get("sample_size") != EXPECTED_N:
        raise AggregateError("prepared sample size is not 200")
    return manifest


def _record_path(root: Path, slug: str, simulator: str, dataset_id: str) -> Path:
    return root / slug / simulator / "assets" / f"{dataset_id}.json"


def _expected_source(row: Mapping[str, Any], simulator: str) -> dict[str, Any]:
    sources = row.get("simulator_sources")
    source = sources.get(simulator) if isinstance(sources, Mapping) else None
    if not isinstance(source, Mapping):
        return {
            "format": "urdf",
            "sha256": row.get("urdf_sha256"),
            "representation": "released_or_prepared_canonical_urdf",
        }
    return {
        "format": source.get("format"),
        "sha256": source.get("sha256"),
        "representation": source.get("representation"),
    }


def _record(
    path: Path,
    *,
    manifest: Mapping[str, Any],
    row: Mapping[str, Any],
    simulator: str,
) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(record, dict) or record.get("schema_version") != ASSET_RECORD_SCHEMA:
        return None
    identity = record.get("identity")
    if not isinstance(identity, Mapping):
        return None
    source = _expected_source(row, simulator)
    expected = {
        "dataset_slug": row["dataset_slug"],
        "dataset_name": row["dataset_name"],
        "dataset_id": row["dataset_id"],
        "asset_id": row["asset_id"],
        "simulator": simulator,
        "manifest_sha256": manifest["manifest_sha256"],
        "protocol_sha256": manifest["protocol_sha256"],
        "row_sha256": row["row_sha256"],
        "urdf_sha256": source["sha256"],
        "source_sha256": source["sha256"],
        "source_format": source["format"],
        "source_representation": source["representation"],
        "canonical_urdf_sha256": row["urdf_sha256"],
        "prepared_cohort_sha256": manifest["prepared_cohort_sha256"],
    }
    if any(identity.get(field) != value for field, value in expected.items()):
        return None
    evaluation = record.get("evaluation")
    if not isinstance(evaluation, Mapping):
        return None
    v2 = evaluation.get("v2")
    if not isinstance(v2, Mapping) or v2.get("schema_version") != V2_EVIDENCE_SCHEMA:
        return None
    return record


def _v2(record: Mapping[str, Any] | None) -> Mapping[str, Any]:
    if not isinstance(record, Mapping):
        return {}
    evaluation = record.get("evaluation")
    return evaluation.get("v2", {}) if isinstance(evaluation, Mapping) else {}


def _import_pass(record: Mapping[str, Any] | None) -> bool:
    evidence = _v2(record)
    imported = evidence.get("import")
    mapping = evidence.get("dof_mapping")
    physics = evidence.get("physics")
    return bool(
        isinstance(imported, Mapping)
        and imported.get("passed") is True
        and isinstance(imported.get("first_step"), Mapping)
        and imported["first_step"].get("passed") is True
        and isinstance(mapping, Mapping)
        and isinstance(physics, Mapping)
    )


def _stable_pass(record: Mapping[str, Any] | None) -> bool:
    evidence = _v2(record)
    stability = evidence.get("stable_rollout")
    return bool(
        _import_pass(record)
        and isinstance(stability, Mapping)
        and stability.get("passed") is True
    )


def _mapped(record: Mapping[str, Any] | None) -> set[str]:
    if not _import_pass(record):
        return set()
    mapping = _v2(record).get("dof_mapping")
    names = (
        mapping.get("mapped_canonical_scalar_joint_names")
        if isinstance(mapping, Mapping)
        else None
    )
    return set(names) if isinstance(names, list) else set()


def _declared_names(row: Mapping[str, Any]) -> set[str]:
    return {
        str(joint["name"])
        for joint in row.get("scalar_joints", [])
        if isinstance(joint, Mapping) and isinstance(joint.get("name"), str)
    }


def _canonical_fk_candidate_count(
    row: Mapping[str, Any], protocol: Mapping[str, Any]
) -> int:
    tree = row.get("joint_tree")
    if not isinstance(tree, Mapping):
        return 0
    alphas = protocol["v2_metrics"]["fk_probe_alphas"]
    return sum(
        len(_descendant_names(tree, str(joint["child"]))) * len(alphas)
        for joint in row.get("scalar_joints", [])
        if _eligible_joint(joint)
    )


def _descendant_names(tree: Mapping[str, Any], child_name: str) -> set[str]:
    reachable = {child_name}
    joints = tree.get("joints", [])
    while True:
        expanded = reachable | {
            str(joint["child"])
            for joint in joints
            if isinstance(joint, Mapping)
            and joint.get("parent") in reachable
            and isinstance(joint.get("child"), str)
        }
        if expanded == reachable:
            return reachable
        reachable = expanded


def _eligible_joint(joint: Mapping[str, Any]) -> bool:
    return bool(
        joint.get("type") in {"revolute", "prismatic"}
        and _finite(joint.get("lower"))
        and _finite(joint.get("upper"))
        and float(joint["lower"]) < float(joint["upper"])
        and _finite(joint.get("effort"))
        and float(joint["effort"]) > 0
        and _finite(joint.get("velocity"))
        and float(joint["velocity"]) > 0
    )


def _actuation_by_name(record: Mapping[str, Any] | None) -> dict[str, Mapping[str, Any]]:
    if not isinstance(record, Mapping) or not _stable_pass(record):
        return {}
    evaluation = record.get("evaluation")
    diagnostics = evaluation.get("diagnostics") if isinstance(evaluation, Mapping) else None
    rows = diagnostics.get("actuation") if isinstance(diagnostics, Mapping) else None
    if not isinstance(rows, list):
        return {}
    result: dict[str, Mapping[str, Any]] = {}
    for row in rows:
        name = row.get("joint_name") if isinstance(row, Mapping) else None
        if isinstance(name, str) and name not in result:
            result[name] = row
    return result


def _tracking_nrmse(
    diagnostic: Mapping[str, Any], protocol: Mapping[str, Any]
) -> float | None:
    trajectory = diagnostic.get("trajectory")
    if not isinstance(trajectory, Mapping):
        return None
    steps = trajectory.get("sample_steps")
    measured = trajectory.get("normalized_positions")
    expected_steps = protocol["cross_simulator"]["joint_rmse"]["sample_steps"]
    if (
        steps != expected_steps
        or not isinstance(measured, list)
        or len(measured) != len(expected_steps)
        or any(not _finite(value) for value in measured)
    ):
        return None
    ramp = float(protocol["runtime"]["actuation"]["trajectory"]["ramp_steps"])
    target = [
        _legacy_minimum_jerk(step / ramp) if step <= ramp else 1.0
        for step in expected_steps
    ]
    return math.sqrt(
        sum((float(a) - float(b)) ** 2 for a, b in zip(measured, target))
        / len(target)
    )


def _legacy_minimum_jerk(progress: float) -> float:
    progress = max(0.0, min(1.0, float(progress)))
    return 10.0 * progress**3 - 15.0 * progress**4 + 6.0 * progress**5


def _limit_by_name(record: Mapping[str, Any] | None) -> dict[str, Mapping[str, Any]]:
    if not isinstance(record, Mapping) or not _stable_pass(record):
        return {}
    evaluation = record.get("evaluation")
    diagnostics = evaluation.get("diagnostics") if isinstance(evaluation, Mapping) else None
    rows = (
        diagnostics.get("limit_enforcement")
        if isinstance(diagnostics, Mapping)
        else None
    )
    if not isinstance(rows, list):
        return {}
    return {
        str(row["joint_name"]): row
        for row in rows
        if isinstance(row, Mapping) and isinstance(row.get("joint_name"), str)
    }


def _limit_violation(
    diagnostic: Mapping[str, Any], joint: Mapping[str, Any]
) -> float | None:
    targets = diagnostic.get("targets")
    if not isinstance(targets, list) or not targets:
        return None
    lower, upper = float(joint["lower"]), float(joint["upper"])
    span = upper - lower
    maximum = 0.0
    for target in targets:
        if not isinstance(target, Mapping):
            return None
        low, high = target.get("minimum_q"), target.get("maximum_q")
        if not _finite(low) or not _finite(high):
            return None
        maximum = max(maximum, lower - float(low), float(high) - upper, 0.0)
    return maximum / span


def _fk_values(record: Mapping[str, Any] | None, field: str) -> list[float]:
    if not _import_pass(record):
        return []
    probe = _v2(record).get("fk_probe")
    samples = probe.get("samples") if isinstance(probe, Mapping) else None
    if not isinstance(samples, list):
        return []
    return [
        float(sample[field])
        for sample in samples
        if isinstance(sample, Mapping)
        and sample.get("status") == "evaluated"
        and _finite(sample.get(field))
    ]


def _failure_reason(record: Mapping[str, Any] | None) -> str:
    if record is None:
        return "MISSING_OR_UNTRUSTED_RECORD"
    if _import_pass(record) and _stable_pass(record):
        return "PASS"
    evaluation = record.get("evaluation")
    if isinstance(evaluation, Mapping):
        if evaluation.get("physics_blocked"):
            return "PHYSICS_PREPARATION_BLOCKED"
        diagnostic = evaluation.get("diagnostic_failure")
        if isinstance(diagnostic, Mapping):
            stage = str(diagnostic.get("stage") or "UNKNOWN").upper()
            return f"DIAGNOSTIC_{stage}"
        if evaluation.get("worker_error"):
            return "WORKER_ERROR"
    return str(record.get("terminal_status") or "UNKNOWN").upper()


def _dataset_summary(
    dataset: Mapping[str, Any],
    records: Mapping[str, Sequence[dict[str, Any] | None]],
    protocol: Mapping[str, Any],
) -> dict[str, Any]:
    rows = dataset["rows"]
    if len(rows) != EXPECTED_N:
        raise AggregateError(f"{dataset['dataset_slug']} does not have N=200")
    resamples = int(protocol["v2_metrics"]["bootstrap"]["resamples"])
    bootstrap_seed = str(protocol["v2_metrics"]["bootstrap"]["seed"])
    declared_dofs = sum(len(row.get("scalar_joints", [])) for row in rows)

    import_outcomes = {
        simulator: [_import_pass(record) for record in records[simulator]]
        for simulator in SIMULATORS
    }
    stable_outcomes = {
        simulator: [_stable_pass(record) for record in records[simulator]]
        for simulator in SIMULATORS
    }
    mapped_counts = {
        simulator: sum(
            len(_mapped(record) & _declared_names(row))
            for row, record in zip(rows, records[simulator])
        )
        for simulator in SIMULATORS
    }
    genesis_records = records["genesis"]

    tracking_by_asset: list[list[float]] = []
    limit_by_asset: list[list[float]] = []
    tracking_candidates = 0
    tracking_evaluated = 0
    limit_evaluated = 0
    for row, record in zip(rows, genesis_records):
        actuation = _actuation_by_name(record)
        limits = _limit_by_name(record)
        tracking_values: list[float] = []
        limit_values: list[float] = []
        for joint in row.get("scalar_joints", []):
            if not _eligible_joint(joint):
                continue
            tracking_candidates += 1
            name = str(joint["name"])
            tracking = (
                _tracking_nrmse(actuation[name], protocol) if name in actuation else None
            )
            violation = (
                _limit_violation(limits[name], joint) if name in limits else None
            )
            if tracking is not None:
                tracking_values.append(tracking)
                tracking_evaluated += 1
            if violation is not None:
                limit_values.append(violation)
                limit_evaluated += 1
        tracking_by_asset.append(tracking_values)
        limit_by_asset.append(limit_values)

    genesis_import = import_outcomes["genesis"]
    genesis_stable = stable_outcomes["genesis"]
    table5a = {
        "import_success": _rate(
            sum(genesis_import),
            EXPECTED_N,
            ci=_bootstrap_rate(
                genesis_import,
                resamples=resamples,
                seed=f"{bootstrap_seed}:{dataset['dataset_slug']}:genesis_import",
            ),
        ),
        "dof_coverage": _rate(
            mapped_counts["genesis"], declared_dofs, ci=None
        ),
        "stable_rollout": _rate(
            sum(genesis_stable),
            EXPECTED_N,
            ci=_bootstrap_rate(
                genesis_stable,
                resamples=resamples,
                seed=f"{bootstrap_seed}:{dataset['dataset_slug']}:genesis_stable",
            ),
        ),
        "tracking_nrmse_p95": _continuous(
            tracking_by_asset,
            candidate_units=tracking_candidates,
            evaluated_units=tracking_evaluated,
            scale=100.0,
            unit="percent_joint_range",
            resamples=resamples,
            seed=f"{bootstrap_seed}:{dataset['dataset_slug']}:tracking",
        ),
        "limit_violation_p95": _continuous(
            limit_by_asset,
            candidate_units=tracking_candidates,
            evaluated_units=limit_evaluated,
            scale=100.0,
            unit="percent_joint_range",
            resamples=resamples,
            seed=f"{bootstrap_seed}:{dataset['dataset_slug']}:limits",
        ),
    }

    fk_position_by_simulator: dict[str, list[list[float]]] = {
        simulator: [[] for _ in rows] for simulator in SIMULATORS
    }
    fk_rotation_by_simulator: dict[str, list[list[float]]] = {
        simulator: [[] for _ in rows] for simulator in SIMULATORS
    }
    fk_candidate = sum(_canonical_fk_candidate_count(row, protocol) for row in rows)
    for index, row in enumerate(rows):
        for simulator in SIMULATORS:
            position_values = _fk_values(
                records[simulator][index], "position_error_over_bbox"
            )
            rotation_values = _fk_values(
                records[simulator][index], "rotation_error_rad"
            )
            fk_position_by_simulator[simulator][index].extend(position_values)
            fk_rotation_by_simulator[simulator][index].extend(rotation_values)

    table5b: dict[str, dict[str, Any]] = {}
    for simulator in SIMULATORS:
        position_by_asset = fk_position_by_simulator[simulator]
        rotation_by_asset = fk_rotation_by_simulator[simulator]
        position_evaluated = sum(len(values) for values in position_by_asset)
        rotation_evaluated = sum(len(values) for values in rotation_by_asset)
        if position_evaluated > fk_candidate or rotation_evaluated > fk_candidate:
            raise AggregateError(
                f"{simulator} FK evidence exceeds the canonical candidate denominator"
            )
        table5b[simulator] = {
            "import_success": _rate(
                sum(import_outcomes[simulator]),
                EXPECTED_N,
                ci=_bootstrap_rate(
                    import_outcomes[simulator],
                    resamples=resamples,
                    seed=f"{bootstrap_seed}:{dataset['dataset_slug']}:{simulator}:import",
                ),
            ),
            "dof_coverage": _rate(mapped_counts[simulator], declared_dofs, ci=None),
            "stable_rollout": _rate(
                sum(stable_outcomes[simulator]),
                EXPECTED_N,
                ci=_bootstrap_rate(
                    stable_outcomes[simulator],
                    resamples=resamples,
                    seed=f"{bootstrap_seed}:{dataset['dataset_slug']}:{simulator}:stable",
                ),
            ),
            "fk_position_error_p95": _continuous(
                position_by_asset,
                candidate_units=fk_candidate,
                evaluated_units=position_evaluated,
                scale=100.0,
                unit="percent_object_diagonal",
                resamples=resamples,
                seed=f"{bootstrap_seed}:{dataset['dataset_slug']}:{simulator}:fk_position",
            ),
            "fk_rotation_error_p95": _continuous(
                rotation_by_asset,
                candidate_units=fk_candidate,
                evaluated_units=rotation_evaluated,
                scale=180.0 / math.pi,
                unit="degree",
                resamples=resamples,
                seed=f"{bootstrap_seed}:{dataset['dataset_slug']}:{simulator}:fk_rotation",
            ),
        }
    failure_counts = {
        simulator: dict(
            sorted(Counter(_failure_reason(record) for record in records[simulator]).items())
        )
        for simulator in SIMULATORS
    }
    trusted = {
        simulator: sum(record is not None for record in records[simulator])
        for simulator in SIMULATORS
    }
    return {
        "dataset_slug": dataset["dataset_slug"],
        "dataset_name": dataset["dataset_name"],
        "n": EXPECTED_N,
        "classification": (
            "COMPLETE"
            if all(count == EXPECTED_N for count in trusted.values())
            else "INCOMPLETE"
        ),
        "trusted_terminal_records": trusted,
        "declared_scalar_dof_count": declared_dofs,
        "table5a": table5a,
        "table5b": table5b,
        "failure_reason_counts": failure_counts,
    }


def aggregate(
    prepared_path: Path,
    runtime_roots: Mapping[str, Path],
) -> dict[str, Any]:
    manifest = _load_manifest(prepared_path)
    protocol = manifest["protocol"]
    datasets: list[dict[str, Any]] = []
    for dataset in manifest["datasets"]:
        records = {
            simulator: [
                _record(
                    _record_path(
                        runtime_roots[simulator],
                        dataset["dataset_slug"],
                        simulator,
                        row["dataset_id"],
                    ),
                    manifest=manifest,
                    row=row,
                    simulator=simulator,
                )
                for row in dataset["rows"]
            ]
            for simulator in SIMULATORS
        }
        datasets.append(_dataset_summary(dataset, records, protocol))
    summary: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "prepared_manifest": str(prepared_path.resolve(strict=False)),
        "prepared_manifest_sha256": manifest["manifest_sha256"],
        "prepared_cohort_sha256": manifest["prepared_cohort_sha256"],
        "protocol_sha256": manifest["protocol_sha256"],
        "classification": (
            "COMPLETE"
            if all(dataset["classification"] == "COMPLETE" for dataset in datasets)
            else "INCOMPLETE"
        ),
        "dataset_order": [dataset["dataset_slug"] for dataset in datasets],
        "datasets": datasets,
    }
    summary["summary_sha256"] = canonical_sha256(
        summary, exclude_fields=("summary_sha256",)
    )
    return summary


def _rate_cell(value: Mapping[str, Any]) -> str:
    return f"{float(value['percentage']):.2f}"


def _continuous_cell(value: Mapping[str, Any]) -> str:
    p95 = value.get("p95")
    return "N/E" if p95 is None else f"{float(p95):.4f}"


def render_markdown(summary: Mapping[str, Any]) -> str:
    lines = [
        "# Table 5 v2 results",
        "",
        f"Run classification: **{summary['classification']}**.",
        "",
        "## Table 5a: Genesis Simulation Readiness",
        "",
        "| Dataset | N | Import Success (%) ↑ | DoF Coverage (%) ↑ | Stable Rollout (%) ↑ | Tracking NRMSE P95 (%) ↓ | Limit Violation P95 (%) ↓ |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for dataset in summary["datasets"]:
        table = dataset["table5a"]
        lines.append(
            f"| {dataset['dataset_name']} | {dataset['n']} | "
            f"{_rate_cell(table['import_success'])} | "
            f"{_rate_cell(table['dof_coverage'])} | "
            f"{_rate_cell(table['stable_rollout'])} | "
            f"{_continuous_cell(table['tracking_nrmse_p95'])} | "
            f"{_continuous_cell(table['limit_violation_p95'])} |"
        )
    lines.extend(
        [
            "",
            "## Table 5b: Cross-Simulator Portability",
            "",
            "| Dataset | N | Genesis Import (%) ↑ | Genesis DoF (%) ↑ | Genesis Stable Rollout (%) ↑ | PyBullet Import (%) ↑ | PyBullet DoF (%) ↑ | PyBullet Stable Rollout (%) ↑ | MuJoCo Import (%) ↑ | MuJoCo DoF (%) ↑ | MuJoCo Stable Rollout (%) ↑ |",
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
            "FK diagnostics are reported against canonical URDF kinematics per simulator; pairwise trajectory NRMSE is not measured.",
        ]
    )
    lines.extend(
        [
            "",
            "Continuous metric coverage, confidence intervals, and failure counts are retained in `summary.json` and the CSV files.",
            "",
        ]
    )
    return "\n".join(lines)


def _csv_text(summary: Mapping[str, Any], table_name: str) -> str:
    rows: list[dict[str, Any]] = []
    for dataset in summary["datasets"]:
        row: dict[str, Any] = {
            "dataset_slug": dataset["dataset_slug"],
            "dataset": dataset["dataset_name"],
            "n": dataset["n"],
            "classification": dataset["classification"],
        }
        def add_metric(prefix: str, value: Mapping[str, Any]) -> None:
            if "percentage" in value:
                row[f"{prefix}_percentage"] = value["percentage"]
                row[f"{prefix}_passed"] = value["passed"]
                row[f"{prefix}_denominator"] = value["denominator"]
                row[f"{prefix}_bootstrap_95_ci_percentage"] = json.dumps(
                    value["bootstrap_95_ci_percentage"]
                )
                return
            for field in (
                "p95",
                "median",
                "candidate_units",
                "evaluated_units",
                "coverage_percentage",
                "unit",
            ):
                row[f"{prefix}_{field}"] = value.get(field)
            row[f"{prefix}_bootstrap_95_ci"] = json.dumps(
                value.get("p95_asset_cluster_bootstrap_95_ci")
            )

        for name, value in dataset[table_name].items():
            if table_name == "table5b" and isinstance(value, Mapping) and "percentage" not in value:
                for metric_name, metric_value in value.items():
                    if isinstance(metric_value, Mapping):
                        add_metric(f"{name}_{metric_name}", metric_value)
            elif isinstance(value, Mapping):
                add_metric(name, value)
        rows.append(row)
    stream = io.StringIO(newline="")
    fieldnames = list(rows[0]) if rows else []
    writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue()


def _atomic_outputs(output: Path, payloads: Mapping[str, str]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    staged: dict[str, Path] = {}
    try:
        for name, payload in payloads.items():
            descriptor, temporary = tempfile.mkstemp(
                dir=output, prefix=f".{name}.", suffix=".tmp"
            )
            path = Path(temporary)
            staged[name] = path
            with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
        for name, path in staged.items():
            os.replace(path, output / name)
    finally:
        for path in staged.values():
            path.unlink(missing_ok=True)


def write_outputs(summary: Mapping[str, Any], output: Path) -> None:
    _atomic_outputs(
        output,
        {
            "summary.json": json.dumps(
                summary,
                ensure_ascii=True,
                allow_nan=False,
                sort_keys=True,
                indent=2,
            )
            + "\n",
            "table5a.csv": _csv_text(summary, "table5a"),
            "table5b.csv": _csv_text(summary, "table5b"),
            "report.md": render_markdown(summary),
        },
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepared", type=Path, required=True)
    parser.add_argument("--genesis", type=Path, required=True)
    parser.add_argument("--pybullet", type=Path, required=True)
    parser.add_argument("--mujoco", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    arguments = parser.parse_args(argv)
    try:
        summary = aggregate(
            arguments.prepared,
            {
                "genesis": arguments.genesis,
                "pybullet": arguments.pybullet,
                "mujoco": arguments.mujoco,
            },
        )
        write_outputs(summary, arguments.out)
    except (AggregateError, OSError, json.JSONDecodeError) as error:
        parser.error(str(error))
    print(
        json.dumps(
            {
                "classification": summary["classification"],
                "out": str(arguments.out.resolve(strict=False)),
                "summary_sha256": summary["summary_sha256"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
