#!/usr/bin/env python3
"""Aggregate Stable v2 results for the fixed eight Table 5 cohorts."""

from __future__ import annotations

import argparse
import csv
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from typing import Any, Mapping, Sequence


SCRIPT_PATH = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT_PATH.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import table5_stable_v2_runtime as _runtime  # noqa: E402


SCHEMA_VERSION = "table5_stable_v2_aggregate_v1"
SIMULATORS = ("genesis", "pybullet", "mujoco")
DATASET_ORDER = (
    "articraft_10k",
    "lam_released_outputs",
    "artiverse",
    "partnet_mobility",
    "physx_mobility",
    "sketchmobility",
    "infinigen_sim",
    "pva",
)
EXPECTED_N = 200


class AggregateError(ValueError):
    pass


def _trial_count() -> int:
    repetitions = getattr(_runtime, "RESET_REPETITIONS", None)
    if isinstance(repetitions, int) and not isinstance(repetitions, bool):
        return repetitions
    return len(_runtime.INITIAL_FRACTIONS)


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise AggregateError(f"cannot read {path}: {error}") from error
    if not isinstance(value, dict):
        raise AggregateError(f"{path} must contain a JSON object")
    return value


def _atomic_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
        delete=False,
    ) as handle:
        handle.write(value)
        temporary = Path(handle.name)
    temporary.replace(path)


def _atomic_json(path: Path, value: Mapping[str, Any]) -> None:
    _atomic_text(
        path,
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True) + "\n",
    )


def _datasets(manifest: Mapping[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for dataset in manifest.get("datasets", []):
        if not isinstance(dataset, dict):
            raise AggregateError("prepared manifest dataset is malformed")
        slug = dataset.get("dataset_slug")
        rows = dataset.get("rows")
        if not isinstance(slug, str) or not isinstance(rows, list):
            raise AggregateError("prepared manifest dataset identity is malformed")
        result[slug] = dataset
    return result


def _fixed_datasets(
    formal_manifest: Mapping[str, Any], articraft_manifest: Mapping[str, Any]
) -> list[dict[str, Any]]:
    formal = _datasets(formal_manifest)
    articraft = _datasets(articraft_manifest)
    if set(DATASET_ORDER) - set(formal):
        raise AggregateError("formal prepared manifest lacks required datasets")
    if "articraft_10k" not in articraft:
        raise AggregateError("Articraft resample prepared manifest is missing")
    result = [
        articraft[slug] if slug == "articraft_10k" else formal[slug]
        for slug in DATASET_ORDER
    ]
    for dataset in result:
        rows = dataset["rows"]
        if len(rows) != EXPECTED_N:
            raise AggregateError(
                f"{dataset['dataset_slug']} has {len(rows)} rows, expected {EXPECTED_N}"
            )
        identifiers = [str(row.get("dataset_id")) for row in rows]
        if len(set(identifiers)) != EXPECTED_N:
            raise AggregateError(f"{dataset['dataset_slug']} has duplicate dataset IDs")
    return result


def _record_path(
    run_root: Path, simulator: str, dataset_slug: str, dataset_id: str
) -> Path:
    source = "articraft" if dataset_slug == "articraft_10k" else "formal"
    return (
        run_root
        / simulator
        / source
        / dataset_slug
        / simulator
        / "assets"
        / f"{dataset_id}.json"
    )


def _validated_rollout(
    record: Mapping[str, Any], simulator: str, dataset_slug: str
) -> tuple[Mapping[str, Any] | None, str]:
    if record.get("terminal_status") != "completed":
        return None, str(record.get("terminal_status") or "missing_terminal_status")
    identity = record.get("identity")
    if not isinstance(identity, Mapping):
        return None, "missing_identity"
    if (
        identity.get("simulator") != simulator
        or identity.get("dataset_slug") != dataset_slug
    ):
        return None, "identity_mismatch"
    evaluation = record.get("evaluation")
    evidence = evaluation.get("stable_v2") if isinstance(evaluation, Mapping) else None
    if (
        not isinstance(evidence, Mapping)
        or evidence.get("schema_version") != _runtime.EVIDENCE_SCHEMA
    ):
        return None, "missing_stable_v2_evidence"
    protocol = evidence.get("protocol")
    expected_protocol = _runtime._stable_protocol()
    if protocol != expected_protocol:
        return None, "stable_v2_protocol_mismatch"
    receipt = evidence.get("rollout")
    if not isinstance(receipt, Mapping):
        return None, "missing_rollout_receipt"
    if (
        receipt.get("schema_version") != _runtime.RECEIPT_SCHEMA
        or receipt.get("protocol_id") != _runtime.PROTOCOL_ID
        or receipt.get("protocol_sha256") != expected_protocol["protocol_sha256"]
    ):
        return None, "rollout_receipt_identity_mismatch"
    expected_hash = _runtime._core._runtime.canonical_sha256(
        receipt, exclude_fields=("receipt_sha256",)
    )
    if receipt.get("receipt_sha256") != expected_hash:
        return None, "rollout_receipt_hash_mismatch"
    return receipt, "VALID"


def _receipt_pass(
    record: Mapping[str, Any], simulator: str, dataset_slug: str
) -> tuple[bool, str]:
    receipt, reason = _validated_rollout(record, simulator, dataset_slug)
    if receipt is None:
        return False, reason
    if receipt.get("passed") is True:
        trials = receipt.get("trials")
        if (
            not isinstance(trials, list)
            or len(trials) != _trial_count()
            or not all(isinstance(trial, Mapping) for trial in trials)
        ):
            return False, "passed_receipt_trial_count_mismatch"
        if any(
            trial.get("passed") is not True
            or trial.get("steps_completed") != _runtime.MAXIMUM_STEPS
            for trial in trials
        ):
            return False, "passed_receipt_trial_mismatch"
        return True, "PASS"
    checks = receipt.get("checks")
    if isinstance(checks, Mapping):
        for name, passed in checks.items():
            if passed is not True:
                return False, str(name)
    trials = receipt.get("trials")
    if isinstance(trials, list):
        for trial in trials:
            trial_checks = trial.get("checks") if isinstance(trial, Mapping) else None
            if isinstance(trial_checks, Mapping):
                for name, passed in trial_checks.items():
                    if passed is not True:
                        return False, str(name)
    return False, "rollout_failed"


def _drift_by_name(
    record: Mapping[str, Any], simulator: str, dataset_slug: str
) -> dict[str, Mapping[str, Any]]:
    receipt, _ = _validated_rollout(record, simulator, dataset_slug)
    if receipt is None:
        return {}
    drift = receipt.get("constraint_drift")
    if (
        not isinstance(drift, Mapping)
        or drift.get("schema_version") != _runtime.DRIFT_SCHEMA
        or drift.get("sample_every_steps") != _runtime.DRIFT_SAMPLE_EVERY_STEPS
        or drift.get("expected_samples_per_joint")
        != _trial_count()
        * _runtime.MAXIMUM_STEPS
        // _runtime.DRIFT_SAMPLE_EVERY_STEPS
    ):
        return {}
    joints = drift.get("joints")
    if not isinstance(joints, list):
        return {}
    expected_samples = int(drift["expected_samples_per_joint"])
    result: dict[str, Mapping[str, Any]] = {}
    for joint in joints:
        name = joint.get("joint_name") if isinstance(joint, Mapping) else None
        if not isinstance(name, str) or not name or name in result:
            return {}
        position_samples = joint.get("position_evaluated_samples")
        rotation_samples = joint.get("rotation_evaluated_samples")
        if (
            joint.get("expected_samples") != expected_samples
            or not isinstance(position_samples, int)
            or isinstance(position_samples, bool)
            or not 0 <= position_samples <= expected_samples
            or not isinstance(rotation_samples, int)
            or isinstance(rotation_samples, bool)
            or not 0 <= rotation_samples <= expected_samples
            or joint.get("position_fully_evaluated")
            is not (position_samples == expected_samples)
            or joint.get("rotation_fully_evaluated")
            is not (rotation_samples == expected_samples)
            or (
                position_samples == expected_samples
                and not _finite(joint.get("max_position_error_over_bbox"))
            )
            or (
                rotation_samples == expected_samples
                and not _finite(joint.get("max_rotation_error_rad"))
            )
        ):
            return {}
        result[name] = joint
    return result


def _finite(value: Any) -> bool:
    return bool(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    rank = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _continuous(
    values_by_asset: Sequence[Sequence[float]],
    *,
    candidate_units: int,
    scale: float,
    unit: str,
) -> dict[str, Any]:
    values = [float(value) for asset in values_by_asset for value in asset]
    p95 = _percentile(values, 95.0)
    median = _percentile(values, 50.0)
    evaluated = len(values)
    return {
        "candidate_units": candidate_units,
        "evaluated_units": evaluated,
        "coverage_percentage": (
            0.0 if candidate_units == 0 else 100.0 * evaluated / candidate_units
        ),
        "sample_count": evaluated,
        "median": None if median is None else scale * median,
        "p95": None if p95 is None else scale * p95,
        "unit": unit,
    }


def _declared_bounded_joint_names(row: Mapping[str, Any]) -> set[str]:
    return {
        str(joint["name"])
        for joint in row.get("scalar_joints", [])
        if isinstance(joint, Mapping)
        and isinstance(joint.get("name"), str)
        and joint.get("type") in {"revolute", "prismatic"}
        and _finite(joint.get("lower"))
        and _finite(joint.get("upper"))
        and float(joint["lower"]) < float(joint["upper"])
    }


def _old_summary(path: Path) -> dict[str, Mapping[str, Any]]:
    summary = _read_json(path)
    datasets = summary.get("datasets")
    if not isinstance(datasets, list):
        raise AggregateError("old summary datasets are missing")
    result = {
        str(dataset["dataset_slug"]): dataset
        for dataset in datasets
        if isinstance(dataset, Mapping)
    }
    if set(result) != set(DATASET_ORDER):
        raise AggregateError("old summary dataset set differs from Stable v2")
    return result


def _rate(passed: int) -> dict[str, Any]:
    return {
        "passed": passed,
        "denominator": EXPECTED_N,
        "percentage": 100.0 * passed / EXPECTED_N,
    }


def aggregate(
    formal_prepared: Path,
    articraft_prepared: Path,
    run_root: Path,
    old_summary_path: Path,
) -> dict[str, Any]:
    formal_manifest = _read_json(formal_prepared)
    articraft_manifest = _read_json(articraft_prepared)
    datasets = _fixed_datasets(formal_manifest, articraft_manifest)
    old = _old_summary(old_summary_path)
    output_datasets: list[dict[str, Any]] = []
    complete = True
    for dataset in datasets:
        slug = str(dataset["dataset_slug"])
        passes: dict[str, dict[str, bool]] = {simulator: {} for simulator in SIMULATORS}
        reasons: dict[str, dict[str, int]] = {simulator: {} for simulator in SIMULATORS}
        drift_position_by_asset: dict[str, list[list[float]]] = {
            simulator: [] for simulator in SIMULATORS
        }
        drift_rotation_by_asset: dict[str, list[list[float]]] = {
            simulator: [] for simulator in SIMULATORS
        }
        drift_candidate_units = sum(
            len(_declared_bounded_joint_names(row)) for row in dataset["rows"]
        )
        for row in dataset["rows"]:
            dataset_id = str(row["dataset_id"])
            declared_names = _declared_bounded_joint_names(row)
            for simulator in SIMULATORS:
                path = _record_path(run_root, simulator, slug, dataset_id)
                position_values: list[float] = []
                rotation_values: list[float] = []
                if not path.is_file():
                    passed, reason = False, "missing_record"
                    complete = False
                else:
                    record = _read_json(path)
                    passed, reason = _receipt_pass(record, simulator, slug)
                    drift = _drift_by_name(record, simulator, slug)
                    for name in declared_names:
                        joint = drift.get(name)
                        if not isinstance(joint, Mapping):
                            continue
                        if joint.get("position_fully_evaluated") is True and _finite(
                            joint.get("max_position_error_over_bbox")
                        ):
                            position_values.append(
                                float(joint["max_position_error_over_bbox"])
                            )
                        if joint.get("rotation_fully_evaluated") is True and _finite(
                            joint.get("max_rotation_error_rad")
                        ):
                            rotation_values.append(
                                float(joint["max_rotation_error_rad"])
                            )
                passes[simulator][dataset_id] = passed
                reasons[simulator][reason] = reasons[simulator].get(reason, 0) + 1
                drift_position_by_asset[simulator].append(position_values)
                drift_rotation_by_asset[simulator].append(rotation_values)
        old_dataset = old[slug]
        stable_v2 = {
            simulator: _rate(sum(passes[simulator].values()))
            for simulator in SIMULATORS
        }
        all_three = _rate(
            sum(
                all(
                    passes[simulator][str(row["dataset_id"])]
                    for simulator in SIMULATORS
                )
                for row in dataset["rows"]
            )
        )
        constraint_drift = {
            simulator: {
                "position_p95": _continuous(
                    drift_position_by_asset[simulator],
                    candidate_units=drift_candidate_units,
                    scale=100.0,
                    unit="percent_object_diagonal",
                ),
                "rotation_p95": _continuous(
                    drift_rotation_by_asset[simulator],
                    candidate_units=drift_candidate_units,
                    scale=180.0 / math.pi,
                    unit="degree",
                ),
            }
            for simulator in SIMULATORS
        }
        output_datasets.append(
            {
                "dataset_slug": slug,
                "dataset_name": dataset["dataset_name"],
                "n": EXPECTED_N,
                "primary_existing": {
                    "import_success": old_dataset["table5a"]["import_success"],
                    "dof_mapping": old_dataset["table5a"]["dof_coverage"],
                    "actuated_trajectory_coverage": old_dataset["table5a"][
                        "trajectory_coverage"
                    ],
                },
                "old_stable": {
                    "genesis": old_dataset["table5b"]["genesis"]["stable_rollout"],
                    "pybullet": old_dataset["table5b"]["pybullet"]["stable_rollout"],
                    "mujoco": old_dataset["table5b"]["mujoco"]["stable_rollout"],
                },
                "stable_v2": stable_v2,
                "all_three_stable_v2": all_three,
                "constraint_drift": constraint_drift,
                "supplementary_existing": {
                    "tracking_nrmse_p95": old_dataset["table5a"]["tracking_nrmse_p95"],
                    "limit_violation_p95": old_dataset["table5a"][
                        "limit_violation_p95"
                    ],
                    "fk_by_simulator": {
                        simulator: {
                            "position_p95": old_dataset["table5b"][simulator][
                                "fk_position_error_p95"
                            ],
                            "rotation_p95": old_dataset["table5b"][simulator][
                                "fk_rotation_error_p95"
                            ],
                        }
                        for simulator in SIMULATORS
                    },
                },
                "failure_reason_counts": reasons,
            }
        )
    protocol = _runtime._stable_protocol()
    summary: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "classification": "COMPLETE" if complete else "INCOMPLETE",
        "dataset_order": list(DATASET_ORDER),
        "simulators": list(SIMULATORS),
        "formal_prepared_manifest": str(formal_prepared.resolve()),
        "articraft_prepared_manifest": str(articraft_prepared.resolve()),
        "old_summary": str(old_summary_path.resolve()),
        "stable_v2_protocol": protocol,
        "metric_roles": {
            "primary": [
                "import_success",
                "dof_coverage",
                "stable_v2",
                "trajectory_coverage",
                "constraint_drift_position_p95",
                "constraint_drift_rotation_p95",
            ],
            "supplementary_existing_values_not_rerun": [
                "tracking_nrmse_p95",
                "limit_violation_p95",
                "fk_position_error_p95",
                "fk_rotation_error_p95",
            ],
        },
        "datasets": output_datasets,
    }
    summary["summary_sha256"] = _runtime._core._runtime.canonical_sha256(
        summary, exclude_fields=("summary_sha256",)
    )
    return summary


def _report(summary: Mapping[str, Any]) -> str:
    lines = [
        "# Table 5 primary results",
        "",
        f"Run classification: **{summary['classification']}**.",
        "",
        "Stable v2 uses gravity [0, 0, -9.81], contacts, self-collision, a fixed base, and each asset's manifest-bound physics. Each asset is reset to 25%, 50%, and 75% of every mapped bounded hinge/slide range. Each trial runs the full 10 s at 240 Hz under zero applied joint force. Passing requires finite states and poses, unchanged mapping, <=0.5% normalized limit violation, revolute speed <=300 deg/s, and prismatic speed <=5 m/s for all three trials.",
        "",
        "## Genesis simulation readiness",
        "",
        "| Dataset | N | Import (%) | DoF Mapping (%) | Stable v2 (%) | Actuated Trajectory Coverage (%) | Drift Pos P95 (% diag.) | Drift Rot P95 (deg) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for dataset in summary["datasets"]:
        existing = dataset["primary_existing"]
        position = dataset["constraint_drift"]["genesis"]["position_p95"]
        rotation = dataset["constraint_drift"]["genesis"]["rotation_p95"]
        lines.append(
            f"| {dataset['dataset_name']} | {dataset['n']} | "
            f"{existing['import_success']['percentage']:.2f} | "
            f"{existing['dof_mapping']['percentage']:.2f} | "
            f"{dataset['stable_v2']['genesis']['percentage']:.2f} | "
            f"{existing['actuated_trajectory_coverage']['percentage']:.2f} | "
            f"{_continuous_cell(position)} | {_continuous_cell(rotation)} |"
        )
    lines.extend(
        [
            "",
            "## Cross-simulator Stable v2",
            "",
            "| Dataset | Genesis (%) | PyBullet (%) | MuJoCo (%) | All three (%) |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for dataset in summary["datasets"]:
        stable = dataset["stable_v2"]
        lines.append(
            f"| {dataset['dataset_name']} | {stable['genesis']['percentage']:.2f} | {stable['pybullet']['percentage']:.2f} | {stable['mujoco']['percentage']:.2f} | {dataset['all_three_stable_v2']['percentage']:.2f} |"
        )
    lines.extend(
        [
            "",
            "## Long-horizon constraint drift",
            "",
            "Each cell is `P95 (evaluated/candidate joints)`. Translation is normalized by object bounding-box diagonal; rotation is quaternion geodesic error.",
            "",
            "| Dataset | Genesis Pos (% diag.) | Genesis Rot (deg) | PyBullet Pos (% diag.) | PyBullet Rot (deg) | MuJoCo Pos (% diag.) | MuJoCo Rot (deg) |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for dataset in summary["datasets"]:
        drift = dataset["constraint_drift"]
        lines.append(
            f"| {dataset['dataset_name']} | "
            f"{_continuous_cell(drift['genesis']['position_p95'])} | "
            f"{_continuous_cell(drift['genesis']['rotation_p95'])} | "
            f"{_continuous_cell(drift['pybullet']['position_p95'])} | "
            f"{_continuous_cell(drift['pybullet']['rotation_p95'])} | "
            f"{_continuous_cell(drift['mujoco']['position_p95'])} | "
            f"{_continuous_cell(drift['mujoco']['rotation_p95'])} |"
        )
    lines.append("")
    return "\n".join(lines)


def _supplementary_report(summary: Mapping[str, Any]) -> str:
    lines = [
        "# Table 5 supplementary diagnostics",
        "",
        "These values are retained from the fixed eight-dataset evaluation and were not rerun with Stable v2. They do not participate in the primary ranking.",
        "",
        "## Active-control diagnostics",
        "",
        "| Dataset | Tracking NRMSE P95 (%) | Limit Violation P95 (%) |",
        "|---|---:|---:|",
    ]
    for dataset in summary["datasets"]:
        diagnostics = dataset["supplementary_existing"]
        lines.append(
            f"| {dataset['dataset_name']} | "
            f"{_continuous_cell(diagnostics['tracking_nrmse_p95'])} | "
            f"{_continuous_cell(diagnostics['limit_violation_p95'])} |"
        )
    lines.extend(
        [
            "",
            "## Direct-reset FK diagnostics",
            "",
            "| Dataset | Genesis Pos (% diag.) | Genesis Rot (deg) | PyBullet Pos (% diag.) | PyBullet Rot (deg) | MuJoCo Pos (% diag.) | MuJoCo Rot (deg) |",
            "|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for dataset in summary["datasets"]:
        fk = dataset["supplementary_existing"]["fk_by_simulator"]
        lines.append(
            f"| {dataset['dataset_name']} | "
            f"{_continuous_cell(fk['genesis']['position_p95'])} | "
            f"{_continuous_cell(fk['genesis']['rotation_p95'])} | "
            f"{_continuous_cell(fk['pybullet']['position_p95'])} | "
            f"{_continuous_cell(fk['pybullet']['rotation_p95'])} | "
            f"{_continuous_cell(fk['mujoco']['position_p95'])} | "
            f"{_continuous_cell(fk['mujoco']['rotation_p95'])} |"
        )
    lines.append("")
    return "\n".join(lines)


def _continuous_cell(value: Mapping[str, Any]) -> str:
    evaluated = int(value.get("evaluated_units", 0))
    candidate = int(value.get("candidate_units", 0))
    p95 = value.get("p95")
    if p95 is None:
        return f"N/E ({evaluated}/{candidate})"
    return f"{float(p95):.4f} ({evaluated}/{candidate})"


def _csv(summary: Mapping[str, Any]) -> str:
    output = io.StringIO()
    fields = [
        "dataset_slug",
        "dataset",
        "n",
        "import_success_percentage",
        "dof_mapping_percentage",
        "actuated_trajectory_coverage_percentage",
        "genesis_stable_v1_percentage",
        "genesis_stable_v2_percentage",
        "pybullet_stable_v1_percentage",
        "pybullet_stable_v2_percentage",
        "mujoco_stable_v1_percentage",
        "mujoco_stable_v2_percentage",
        "all_three_stable_v2_percentage",
        "genesis_drift_position_p95_percent_diagonal",
        "genesis_drift_position_evaluated_units",
        "genesis_drift_rotation_p95_degree",
        "genesis_drift_rotation_evaluated_units",
        "pybullet_drift_position_p95_percent_diagonal",
        "pybullet_drift_position_evaluated_units",
        "pybullet_drift_rotation_p95_degree",
        "pybullet_drift_rotation_evaluated_units",
        "mujoco_drift_position_p95_percent_diagonal",
        "mujoco_drift_position_evaluated_units",
        "mujoco_drift_rotation_p95_degree",
        "mujoco_drift_rotation_evaluated_units",
        "drift_candidate_units",
    ]
    writer = csv.DictWriter(output, fieldnames=fields)
    writer.writeheader()
    for dataset in summary["datasets"]:
        writer.writerow(
            {
                "dataset_slug": dataset["dataset_slug"],
                "dataset": dataset["dataset_name"],
                "n": dataset["n"],
                "import_success_percentage": dataset["primary_existing"][
                    "import_success"
                ]["percentage"],
                "dof_mapping_percentage": dataset["primary_existing"]["dof_mapping"][
                    "percentage"
                ],
                "actuated_trajectory_coverage_percentage": dataset["primary_existing"][
                    "actuated_trajectory_coverage"
                ]["percentage"],
                "genesis_stable_v1_percentage": dataset["old_stable"]["genesis"][
                    "percentage"
                ],
                "genesis_stable_v2_percentage": dataset["stable_v2"]["genesis"][
                    "percentage"
                ],
                "pybullet_stable_v1_percentage": dataset["old_stable"]["pybullet"][
                    "percentage"
                ],
                "pybullet_stable_v2_percentage": dataset["stable_v2"]["pybullet"][
                    "percentage"
                ],
                "mujoco_stable_v1_percentage": dataset["old_stable"]["mujoco"][
                    "percentage"
                ],
                "mujoco_stable_v2_percentage": dataset["stable_v2"]["mujoco"][
                    "percentage"
                ],
                "all_three_stable_v2_percentage": dataset["all_three_stable_v2"][
                    "percentage"
                ],
                "genesis_drift_position_p95_percent_diagonal": dataset[
                    "constraint_drift"
                ]["genesis"]["position_p95"]["p95"],
                "genesis_drift_position_evaluated_units": dataset["constraint_drift"][
                    "genesis"
                ]["position_p95"]["evaluated_units"],
                "genesis_drift_rotation_p95_degree": dataset["constraint_drift"][
                    "genesis"
                ]["rotation_p95"]["p95"],
                "genesis_drift_rotation_evaluated_units": dataset["constraint_drift"][
                    "genesis"
                ]["rotation_p95"]["evaluated_units"],
                "pybullet_drift_position_p95_percent_diagonal": dataset[
                    "constraint_drift"
                ]["pybullet"]["position_p95"]["p95"],
                "pybullet_drift_position_evaluated_units": dataset["constraint_drift"][
                    "pybullet"
                ]["position_p95"]["evaluated_units"],
                "pybullet_drift_rotation_p95_degree": dataset["constraint_drift"][
                    "pybullet"
                ]["rotation_p95"]["p95"],
                "pybullet_drift_rotation_evaluated_units": dataset["constraint_drift"][
                    "pybullet"
                ]["rotation_p95"]["evaluated_units"],
                "mujoco_drift_position_p95_percent_diagonal": dataset[
                    "constraint_drift"
                ]["mujoco"]["position_p95"]["p95"],
                "mujoco_drift_position_evaluated_units": dataset["constraint_drift"][
                    "mujoco"
                ]["position_p95"]["evaluated_units"],
                "mujoco_drift_rotation_p95_degree": dataset["constraint_drift"][
                    "mujoco"
                ]["rotation_p95"]["p95"],
                "mujoco_drift_rotation_evaluated_units": dataset["constraint_drift"][
                    "mujoco"
                ]["rotation_p95"]["evaluated_units"],
                "drift_candidate_units": dataset["constraint_drift"]["genesis"][
                    "position_p95"
                ]["candidate_units"],
            }
        )
    return output.getvalue()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--formal-prepared", type=Path, required=True)
    parser.add_argument("--articraft-prepared", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--old-summary", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.out.exists():
        raise AggregateError(f"output already exists: {args.out}")
    summary = aggregate(
        args.formal_prepared,
        args.articraft_prepared,
        args.run_root,
        args.old_summary,
    )
    args.out.mkdir(parents=True)
    _atomic_json(args.out / "summary.json", summary)
    _atomic_text(args.out / "report.md", _report(summary))
    _atomic_text(
        args.out / "supplementary_diagnostics.md", _supplementary_report(summary)
    )
    _atomic_text(args.out / "stable_v2.csv", _csv(summary))
    print(
        json.dumps(
            {
                "classification": summary["classification"],
                "out": str(args.out.resolve()),
                "summary_sha256": summary["summary_sha256"],
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AggregateError as error:
        print(f"table5_stable_v2_aggregate: {error}", file=sys.stderr)
        raise SystemExit(2)
