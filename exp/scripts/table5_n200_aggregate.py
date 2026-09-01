#!/usr/bin/env python3
"""Fail-closed aggregation for the unified six-dataset Table 5 N=200 run.

The runtime layout consumed by this script is::

    <run-root>/runtime/<dataset-slug>/<simulator>/assets/<dataset-id>.json

Missing, malformed, and non-completed records remain failures in every asset-level
rate.  Cross-simulator diagnostics use their own explicit coverage denominators;
they never silently remove an asset from a Table 5a or strict Table 5b rate.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
import statistics
import sys
import tempfile
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


SCHEMA_VERSION = "table5_n200_aggregate_v1"
SIMULATORS = ("pybullet", "genesis", "mujoco")
SIMULATOR_PAIRS = (
    ("pybullet", "genesis"),
    ("pybullet", "mujoco"),
    ("genesis", "mujoco"),
)
TABLE5A_METRICS = (
    "load",
    "reset",
    "settling",
    "actuation",
    "limit_enforcement",
    "constraint_drift",
    "simulator_pass",
)
STRICT_GATE_NAMES = (
    "strict_urdf_pass",
    "strict_kinematic_pass",
    "strict_collision_pass",
)
DEFAULT_THRESHOLDS = {
    "normalized_joint_rmse": 0.10,
    "translation_over_bbox_diagonal": 0.02,
    "rotation_rad": 0.10,
}
TERMINAL_STATUSES = {
    "completed",
    "preflight_failure",
    "diagnostic_failure",
    "timeout",
    "native_crash",
    "missing_response",
    "malformed_response",
    "worker_error",
    "parent_error",
}


class AggregateError(ValueError):
    """Raised when the manifest, rather than a runtime result, is malformed."""


@dataclass(frozen=True)
class RuntimeRecord:
    path: Path
    record: dict[str, Any] | None
    state: str
    terminal_status: str | None

    @property
    def completed(self) -> bool:
        return self.state == "valid" and self.terminal_status == "completed"


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _rate(passed: int, denominator: int) -> dict[str, Any]:
    if denominator < 0 or passed < 0 or passed > denominator:
        raise AggregateError("invalid rate numerator or denominator")
    return {
        "status": "evaluated",
        "passed": passed,
        "denominator": denominator,
        "percentage": 0.0 if denominator == 0 else 100.0 * passed / denominator,
    }


def _coverage(passed: int, denominator: int) -> dict[str, Any]:
    if denominator == 0:
        return _not_evaluable(0, "no_candidate_units", known_denominator=0)
    return _rate(passed, denominator)


def _not_evaluable(
    denominator: int,
    reason: str,
    *,
    known_passed: int = 0,
    known_denominator: int = 0,
) -> dict[str, Any]:
    return {
        "status": "not_evaluable",
        "passed": None,
        "denominator": denominator,
        "percentage": None,
        "reason": reason,
        "known_passed": known_passed,
        "known_denominator": known_denominator,
    }


def _distribution(values: Sequence[float]) -> dict[str, float | None]:
    return {
        "population_max": max(values) if values else None,
        "mean": statistics.fmean(values) if values else None,
        "median": statistics.median(values) if values else None,
    }


def _canonical_sha256(value: Any, *, exclude_fields: Sequence[str] = ()) -> str:
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

    try:
        encoded = json.dumps(
            filtered(value),
            ensure_ascii=True,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise AggregateError(f"value is not finite canonical JSON: {error}") from error
    return hashlib.sha256(encoded).hexdigest()


def _nested(mapping: Mapping[str, Any], *keys: str) -> Any:
    current: Any = mapping
    for key in keys:
        if not isinstance(current, Mapping) or key not in current:
            return None
        current = current[key]
    return current


def _authority_hashes(manifest: Mapping[str, Any]) -> tuple[str, str]:
    protocol = manifest.get("protocol")
    if not isinstance(protocol, dict):
        raise AggregateError("manifest.protocol must be an embedded object")
    computed_protocol = _canonical_sha256(protocol, exclude_fields=("protocol_sha256",))
    declared_protocols = [
        value
        for value in (manifest.get("protocol_sha256"), protocol.get("protocol_sha256"))
        if value is not None
    ]
    if any(value != computed_protocol for value in declared_protocols):
        raise AggregateError("manifest protocol_sha256 self-check failed")
    computed_manifest = _canonical_sha256(manifest, exclude_fields=("manifest_sha256",))
    declared_manifest = manifest.get("manifest_sha256")
    if declared_manifest is not None and declared_manifest != computed_manifest:
        raise AggregateError("manifest manifest_sha256 self-check failed")
    return computed_manifest, computed_protocol


def _thresholds(manifest: Mapping[str, Any]) -> dict[str, float]:
    candidates = (
        _nested(manifest, "protocol", "cross_simulator", "thresholds"),
        _nested(manifest, "cross_simulator", "thresholds"),
        manifest.get("thresholds"),
    )
    source = next((item for item in candidates if isinstance(item, Mapping)), {})
    result: dict[str, float] = {}
    for name, default in DEFAULT_THRESHOLDS.items():
        value = source.get(name, default)
        if not _finite_number(value) or float(value) < 0:
            raise AggregateError(f"threshold {name!r} must be finite and non-negative")
        result[name] = float(value)
    return result


def _expected_sample_steps(manifest: Mapping[str, Any]) -> list[int] | None:
    candidates = (
        _nested(
            manifest,
            "protocol",
            "cross_simulator",
            "joint_rmse",
            "sample_steps",
        ),
        _nested(manifest, "cross_simulator", "joint_rmse", "sample_steps"),
    )
    value = next((item for item in candidates if item is not None), None)
    if value is None:
        return None
    if not (
        isinstance(value, list)
        and value
        and all(isinstance(step, int) and not isinstance(step, bool) for step in value)
    ):
        raise AggregateError(
            "cross-simulator sample_steps must be a non-empty integer list"
        )
    return list(value)


def _dataset_id(value: Any, *, location: str) -> str:
    if isinstance(value, bool) or not isinstance(value, (str, int)):
        raise AggregateError(f"{location}.dataset_id must be a string or integer")
    text = str(value)
    if not text or text in {".", ".."} or Path(text).name != text:
        raise AggregateError(f"{location}.dataset_id is not a safe filename stem")
    return text


def _joint_rows(row: Mapping[str, Any], *, location: str) -> list[dict[str, Any]]:
    tree = row.get("joint_tree")
    preflight = row.get("preflight")
    if (
        tree is None
        and isinstance(preflight, Mapping)
        and preflight.get("simulator_eligible") is False
    ):
        return []
    joints = tree.get("joints") if isinstance(tree, Mapping) else None
    if not isinstance(joints, list) or not all(
        isinstance(item, dict) for item in joints
    ):
        raise AggregateError(f"{location}.joint_tree.joints must be a list of objects")
    names: set[str] = set()
    for index, joint in enumerate(joints):
        name = joint.get("name")
        if not isinstance(name, str) or not name or name in names:
            raise AggregateError(
                f"{location}.joint_tree.joints[{index}] has an invalid name"
            )
        names.add(name)
    return joints


def _scalar_joints(row: Mapping[str, Any], *, location: str) -> list[dict[str, Any]]:
    joints = _joint_rows(row, location=location)
    by_name = {joint["name"]: joint for joint in joints}
    scalar = row.get("scalar_joints")
    if not isinstance(scalar, list):
        raise AggregateError(f"{location}.scalar_joints must be a list")
    if not joints and row.get("joint_tree") is None and scalar:
        raise AggregateError(
            f"{location}.scalar_joints must be empty when joint_tree is unavailable"
        )
    result: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, entry in enumerate(scalar):
        if isinstance(entry, str):
            source = by_name.get(entry)
            if source is None:
                raise AggregateError(
                    f"{location}.scalar_joints[{index}] is absent from joint_tree"
                )
            joint = source
        elif isinstance(entry, dict):
            name = entry.get("name")
            source = by_name.get(name) if isinstance(name, str) else None
            joint = {**(source or {}), **entry}
        else:
            raise AggregateError(f"{location}.scalar_joints[{index}] is malformed")
        name, joint_type = joint.get("name"), joint.get("type")
        if not isinstance(name, str) or not name or name in seen:
            raise AggregateError(
                f"{location}.scalar_joints[{index}] has an invalid name"
            )
        if joint_type not in {"revolute", "prismatic", "continuous"}:
            raise AggregateError(
                f"{location}.scalar_joints[{index}] must be revolute, prismatic, or continuous"
            )
        seen.add(name)
        result.append(dict(joint))
    return result


def _cross_sim_eligible_joint(joint: Mapping[str, Any]) -> bool:
    """Match evaluate_asset's bounded scalar-joint eligibility surface."""
    lower, upper = joint.get("lower"), joint.get("upper")
    effort, velocity = joint.get("effort"), joint.get("velocity")
    return bool(
        joint.get("type") in {"revolute", "prismatic"}
        and _finite_number(lower)
        and _finite_number(upper)
        and float(lower) < float(upper)
        and _finite_number(effort)
        and float(effort) > 0
        and _finite_number(velocity)
        and float(velocity) > 0
    )


def _gate_value(strict_gates: Mapping[str, Any], name: str) -> bool | None:
    if name in strict_gates:
        value = strict_gates[name]
    else:
        table_name = {
            "strict_urdf_pass": "table2",
            "strict_kinematic_pass": "table3",
            "strict_collision_pass": "table4",
        }[name]
        table = strict_gates.get(table_name)
        value = table.get(name) if isinstance(table, Mapping) else "__missing__"
    if value == "__missing__" or (value is not None and not isinstance(value, bool)):
        raise AggregateError(f"strict_gates.{name} must be boolean or null")
    return value


def _bbox_diagonal(row: Mapping[str, Any]) -> float | None:
    for key in (
        "bbox_diagonal",
        "bounding_box_diagonal",
        "object_bbox_diagonal_m",
    ):
        if key in row:
            value = row[key]
            if value is None or (
                isinstance(value, str) and value.strip().upper() == "N/E"
            ):
                return None
            if not _finite_number(value) or float(value) <= 0:
                raise AggregateError(f"{key} must be null or positive and finite")
            return float(value)
    for key in ("bbox", "bounding_box"):
        bbox = row.get(key)
        if not isinstance(bbox, Mapping):
            continue
        for diagonal_key in ("diagonal", "diagonal_m"):
            if diagonal_key in bbox:
                value = bbox[diagonal_key]
                if value is None or (
                    isinstance(value, str) and value.strip().upper() == "N/E"
                ):
                    return None
                if not _finite_number(value) or float(value) <= 0:
                    raise AggregateError(
                        "bbox diagonal must be null or positive and finite"
                    )
                return float(value)
        minimum, maximum = bbox.get("min"), bbox.get("max")
        if (
            isinstance(minimum, list)
            and isinstance(maximum, list)
            and len(minimum) == len(maximum) == 3
            and all(_finite_number(value) for value in minimum + maximum)
        ):
            diagonal = math.sqrt(
                sum(
                    (float(high) - float(low)) ** 2
                    for low, high in zip(minimum, maximum)
                )
            )
            if diagonal <= 0:
                raise AggregateError("bbox diagonal must be positive")
            return diagonal
    return None


def _dataset_entries(datasets: Any) -> list[tuple[str, str, list[Any]]]:
    """Return canonical entries while accepting the early mapping draft."""
    if isinstance(datasets, list):
        entries: list[tuple[str, str, list[Any]]] = []
        for index, dataset in enumerate(datasets):
            if not isinstance(dataset, dict):
                raise AggregateError(f"manifest.datasets[{index}] must be an object")
            slug = dataset.get("dataset_slug")
            name = dataset.get("dataset_name")
            rows = dataset.get("rows")
            if not isinstance(slug, str) or not slug:
                raise AggregateError(
                    f"manifest.datasets[{index}].dataset_slug must be non-empty"
                )
            if not isinstance(name, str) or not name:
                raise AggregateError(
                    f"manifest.datasets[{index}].dataset_name must be non-empty"
                )
            if not isinstance(rows, list):
                raise AggregateError(f"manifest.datasets[{index}].rows must be a list")
            entries.append((slug, name, rows))
        return entries
    if isinstance(datasets, dict):
        entries = []
        for slug, dataset in datasets.items():
            if not isinstance(dataset, dict):
                raise AggregateError(f"datasets.{slug} must be an object")
            name, rows = dataset.get("name"), dataset.get("rows")
            if not isinstance(name, str) or not name:
                raise AggregateError(f"datasets.{slug}.name must be a non-empty string")
            if not isinstance(rows, list):
                raise AggregateError(f"datasets.{slug}.rows must be a list")
            entries.append((slug, name, rows))
        return entries
    raise AggregateError("manifest.datasets must be a non-empty array")


def validate_manifest(manifest: Any) -> dict[str, dict[str, Any]]:
    if not isinstance(manifest, dict):
        raise AggregateError("manifest must be a JSON object")
    datasets = manifest.get("datasets")
    entries = _dataset_entries(datasets)
    if not entries:
        raise AggregateError("manifest.datasets must be non-empty")
    normalized: dict[str, dict[str, Any]] = {}
    for slug, name, rows in entries:
        if (
            not isinstance(slug, str)
            or not slug
            or slug in {".", ".."}
            or Path(slug).name != slug
        ):
            raise AggregateError("dataset slug must be a safe path component")
        if slug in normalized:
            raise AggregateError(f"duplicate dataset slug: {slug}")
        normalized_rows: list[dict[str, Any]] = []
        seen: set[str] = set()
        for index, source_row in enumerate(rows):
            location = f"datasets.{slug}.rows[{index}]"
            if not isinstance(source_row, dict):
                raise AggregateError(f"{location} must be an object")
            row = dict(source_row)
            declared_row_sha256 = row.get("row_sha256")
            if declared_row_sha256 is not None and (
                not isinstance(declared_row_sha256, str)
                or len(declared_row_sha256) != 64
                or _canonical_sha256(row, exclude_fields=("row_sha256",))
                != declared_row_sha256
            ):
                raise AggregateError(f"{location}.row_sha256 self-check failed")
            if row.get("dataset_slug", slug) != slug:
                raise AggregateError(
                    f"{location}.dataset_slug conflicts with its dataset"
                )
            row_name = row.get("dataset_name", row.get("name", name))
            if row_name != name:
                raise AggregateError(
                    f"{location}.dataset_name conflicts with its dataset"
                )
            row.setdefault("dataset_slug", slug)
            row.setdefault("dataset_name", name)
            row["dataset_id"] = _dataset_id(row.get("dataset_id"), location=location)
            if row["dataset_id"] in seen:
                raise AggregateError(
                    f"duplicate dataset_id in {slug}: {row['dataset_id']}"
                )
            seen.add(row["dataset_id"])
            if not isinstance(row.get("category"), str) or not row["category"]:
                raise AggregateError(f"{location}.category must be a non-empty string")
            _joint_rows(row, location=location)
            row["scalar_joints"] = _scalar_joints(row, location=location)
            strict_gates = row.get("strict_gates")
            if not isinstance(strict_gates, dict):
                raise AggregateError(f"{location}.strict_gates must be an object")
            row["_strict_gates"] = {
                name: _gate_value(strict_gates, name) for name in STRICT_GATE_NAMES
            }
            try:
                row["_bbox_diagonal"] = _bbox_diagonal(row)
            except AggregateError as error:
                raise AggregateError(f"{location}.{error}") from error
            normalized_rows.append(row)
        normalized[slug] = {"name": name, "rows": normalized_rows}
    sample_size = manifest.get("sample_size")
    if sample_size is not None:
        if (
            isinstance(sample_size, bool)
            or not isinstance(sample_size, int)
            or sample_size <= 0
        ):
            raise AggregateError("manifest.sample_size must be a positive integer")
        if any(len(dataset["rows"]) != sample_size for dataset in normalized.values()):
            raise AggregateError(
                "each dataset row count must equal manifest.sample_size"
            )
    if manifest.get("dataset_count", len(normalized)) != len(normalized):
        raise AggregateError("manifest.dataset_count mismatch")
    if manifest.get(
        "total_rows", sum(len(item["rows"]) for item in normalized.values())
    ) != sum(len(item["rows"]) for item in normalized.values()):
        raise AggregateError("manifest.total_rows mismatch")
    ordered = manifest.get("ordered_dataset_slugs")
    if ordered is not None and ordered != list(normalized):
        raise AggregateError("manifest.ordered_dataset_slugs mismatch")
    return normalized


def _record_path(run_root: Path, slug: str, simulator: str, dataset_id: str) -> Path:
    return run_root / "runtime" / slug / simulator / "assets" / f"{dataset_id}.json"


def _read_runtime_record(
    path: Path,
    *,
    row: Mapping[str, Any],
    dataset_slug: str,
    dataset_name: str,
    simulator: str,
    manifest_sha256: str,
    protocol_sha256: str,
) -> RuntimeRecord:
    dataset_id = row["dataset_id"]
    if not path.is_file():
        return RuntimeRecord(path, None, "missing", None)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError):
        return RuntimeRecord(path, None, "invalid_json", None)
    if not isinstance(value, dict):
        return RuntimeRecord(path, None, "malformed", None)
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError):
        return RuntimeRecord(path, value, "malformed", None)
    terminal = value.get("terminal_status")
    metrics, evaluation = value.get("metrics"), value.get("evaluation")
    if terminal not in TERMINAL_STATUSES:
        return RuntimeRecord(path, value, "malformed", None)
    record_schema = value.get("schema_version")
    if (
        record_schema is not None and record_schema != "table5_n200_runtime_asset_v1"
    ) or value.get("terminal", True) is not True:
        return RuntimeRecord(path, value, "malformed", terminal)
    if not isinstance(metrics, dict):
        return RuntimeRecord(path, value, "malformed", terminal)
    if any(not isinstance(metrics.get(metric), bool) for metric in TABLE5A_METRICS):
        return RuntimeRecord(path, value, "malformed", terminal)
    if terminal == "completed" and not isinstance(evaluation, dict):
        return RuntimeRecord(path, value, "malformed", terminal)
    evaluation_metrics = (
        evaluation.get("metrics") if isinstance(evaluation, dict) else None
    )
    if terminal == "completed" and (
        not isinstance(evaluation_metrics, dict)
        or any(
            evaluation_metrics.get(metric) is not metrics[metric]
            for metric in TABLE5A_METRICS
        )
    ):
        return RuntimeRecord(path, value, "malformed", terminal)
    identity = value.get("identity")
    if not isinstance(identity, dict):
        return RuntimeRecord(path, value, "identity_mismatch", terminal)
    bindings = {
        "dataset_slug": dataset_slug,
        "dataset_name": dataset_name,
        "dataset_id": dataset_id,
        "simulator": simulator,
        "manifest_sha256": manifest_sha256,
        "protocol_sha256": protocol_sha256,
    }
    asset_id = row.get("asset_id")
    urdf_sha256 = row.get("urdf_sha256")
    if "asset_id" in row:
        bindings["asset_id"] = asset_id
    if "urdf_sha256" in row:
        bindings["urdf_sha256"] = urdf_sha256
    if any(identity.get(field) != expected for field, expected in bindings.items()):
        return RuntimeRecord(path, value, "identity_mismatch", terminal)
    row_sha256 = identity.get("row_sha256")
    if not isinstance(row.get("row_sha256"), str) or row_sha256 != row["row_sha256"]:
        return RuntimeRecord(path, value, "identity_mismatch", terminal)
    return RuntimeRecord(path, value, "valid", terminal)


def _metric_pass(record: RuntimeRecord, metric: str) -> bool:
    return bool(
        record.completed
        and record.record is not None
        and record.record["metrics"].get(metric) is True
    )


def _actuation_map(record: RuntimeRecord) -> dict[str, dict[str, Any]]:
    if not record.completed or record.record is None:
        return {}
    diagnostics = record.record["evaluation"].get("diagnostics")
    actuation = diagnostics.get("actuation") if isinstance(diagnostics, dict) else None
    if not isinstance(actuation, list):
        return {}
    result: dict[str, dict[str, Any]] = {}
    for item in actuation:
        name = item.get("joint_name") if isinstance(item, dict) else None
        if not isinstance(name, str) or not name or name in result:
            return {}
        result[name] = item
    return result


def _support_allows(record: RuntimeRecord, joint_name: str, joint_type: str) -> bool:
    if not record.completed or record.record is None:
        return False
    support = record.record["evaluation"].get("support")
    entries = support.get("joints") if isinstance(support, dict) else None
    if not isinstance(entries, list):
        return True
    match = next(
        (
            entry
            for entry in entries
            if isinstance(entry, dict) and entry.get("name") == joint_name
        ),
        None,
    )
    return bool(
        isinstance(match, dict)
        and match.get("type", joint_type) == joint_type
        and match.get("eligible") is True
        and match.get("runtime_mapped") is True
    )


def _trajectory(
    record: RuntimeRecord,
    joint_name: str,
    joint_type: str,
    expected_steps: list[int] | None,
) -> tuple[list[float], list[int] | None] | None:
    if not _support_allows(record, joint_name, joint_type):
        return None
    diagnostic = _actuation_map(record).get(joint_name)
    if not isinstance(diagnostic, dict):
        return None
    if diagnostic.get("joint_type", joint_type) != joint_type:
        return None
    trajectory = diagnostic.get("trajectory")
    values = (
        trajectory.get("normalized_positions") if isinstance(trajectory, dict) else None
    )
    steps = trajectory.get("sample_steps") if isinstance(trajectory, dict) else None
    if not (
        isinstance(values, list)
        and values
        and all(_finite_number(value) for value in values)
    ):
        return None
    if steps is not None and not (
        isinstance(steps, list)
        and len(steps) == len(values)
        and all(isinstance(step, int) and not isinstance(step, bool) for step in steps)
    ):
        return None
    if expected_steps is not None and steps != expected_steps:
        return None
    return [float(value) for value in values], (
        list(steps) if steps is not None else None
    )


def _pose(
    record: RuntimeRecord, joint_name: str, joint_type: str, link_name: str
) -> dict[str, Any] | None:
    if not _support_allows(record, joint_name, joint_type):
        return None
    diagnostic = _actuation_map(record).get(joint_name)
    pose_map = (
        diagnostic.get("final_descendant_root_frame_poses")
        if isinstance(diagnostic, dict)
        else None
    )
    pose = pose_map.get(link_name) if isinstance(pose_map, dict) else None
    return pose if _valid_pose(pose) else None


def _valid_pose(pose: Any) -> bool:
    if not isinstance(pose, dict):
        return False
    translation, rotation = pose.get("translation"), pose.get("rotation")
    return bool(
        isinstance(translation, list)
        and len(translation) == 3
        and all(_finite_number(value) for value in translation)
        and isinstance(rotation, list)
        and len(rotation) == 4
        and all(_finite_number(value) for value in rotation)
        and math.sqrt(sum(float(value) ** 2 for value in rotation)) > 0
    )


def _rmse(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(statistics.fmean((a - b) ** 2 for a, b in zip(left, right)))


def _quaternion_error(left: Sequence[float], right: Sequence[float]) -> float:
    left_norm = math.sqrt(sum(float(value) ** 2 for value in left))
    right_norm = math.sqrt(sum(float(value) ** 2 for value in right))
    dot = abs(
        sum(float(a) * float(b) for a, b in zip(left, right)) / (left_norm * right_norm)
    )
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def _pose_error(
    left: Mapping[str, Any], right: Mapping[str, Any], bbox: float
) -> tuple[float, float]:
    translation = (
        math.sqrt(
            sum(
                (float(a) - float(b)) ** 2
                for a, b in zip(left["translation"], right["translation"])
            )
        )
        / bbox
    )
    rotation = _quaternion_error(left["rotation"], right["rotation"])
    return translation, rotation


def _descendants(row: Mapping[str, Any], child: Any) -> list[str]:
    if not isinstance(child, str) or not child:
        return []
    joints = row["joint_tree"]["joints"]
    reachable = {child}
    for _ in range(len(joints) + 1):
        expanded = reachable | {
            joint["child"]
            for joint in joints
            if joint.get("parent") in reachable
            and isinstance(joint.get("child"), str)
            and joint["child"]
        }
        if expanded == reachable:
            return sorted(reachable)
        reachable = expanded
    return []


def _joint_diagnostic(
    values: list[float], candidate_units: int, available_pairs: int, threshold: float
) -> dict[str, Any]:
    return {
        "candidate_units": candidate_units,
        "evaluable_units": len(values),
        "excluded_units": candidate_units - len(values),
        "available_pairs": available_pairs,
        "coverage": _coverage(len(values), candidate_units),
        "threshold": threshold,
        "units_within_threshold": sum(value <= threshold for value in values),
        **_distribution(values),
    }


def _aggregate_dataset(
    slug: str,
    dataset: Mapping[str, Any],
    run_root: Path,
    thresholds: Mapping[str, float],
    expected_steps: list[int] | None,
    manifest_sha256: str,
    protocol_sha256: str,
) -> dict[str, Any]:
    rows: list[dict[str, Any]] = dataset["rows"]
    denominator = len(rows)
    records: dict[str, dict[str, RuntimeRecord]] = {sim: {} for sim in SIMULATORS}
    record_states: dict[str, Counter[str]] = {sim: Counter() for sim in SIMULATORS}
    terminal_statuses: dict[str, Counter[str]] = {sim: Counter() for sim in SIMULATORS}
    for row in rows:
        dataset_id = row["dataset_id"]
        for simulator in SIMULATORS:
            record = _read_runtime_record(
                _record_path(run_root, slug, simulator, dataset_id),
                row=row,
                dataset_slug=slug,
                dataset_name=dataset["name"],
                simulator=simulator,
                manifest_sha256=manifest_sha256,
                protocol_sha256=protocol_sha256,
            )
            records[simulator][dataset_id] = record
            record_states[simulator][record.state] += 1
            terminal_statuses[simulator][
                record.terminal_status if record.state == "valid" else record.state
            ] += 1

    table5a = {
        simulator: {
            metric: _rate(
                sum(
                    _metric_pass(records[simulator][row["dataset_id"]], metric)
                    for row in rows
                ),
                denominator,
            )
            for metric in TABLE5A_METRICS
        }
        for simulator in SIMULATORS
    }
    all_three_load_by_id = {
        row["dataset_id"]: all(
            _metric_pass(records[simulator][row["dataset_id"]], "load")
            for simulator in SIMULATORS
        )
        for row in rows
    }
    all_three_runtime_by_id = {
        row["dataset_id"]: all(
            _metric_pass(records[simulator][row["dataset_id"]], "simulator_pass")
            for simulator in SIMULATORS
        )
        for row in rows
    }

    joint_values: dict[str, list[float]] = {"revolute": [], "prismatic": []}
    joint_candidates = {"revolute": 0, "prismatic": 0}
    joint_pair_counts = {"revolute": 0, "prismatic": 0}
    pose_translation_values: list[float] = []
    pose_rotation_values: list[float] = []
    pose_candidates = 0
    pose_pair_count = 0
    consistent_by_id: dict[str, bool] = {}
    consistency_reasons: Counter[str] = Counter()

    for row in rows:
        dataset_id = row["dataset_id"]
        bbox = row["_bbox_diagonal"]
        row_consistent = bbox is not None
        if bbox is None:
            consistency_reasons["missing_bbox"] += 1
        declared_scalar_joints = row["scalar_joints"]
        scalar_joints = [
            joint
            for joint in declared_scalar_joints
            if _cross_sim_eligible_joint(joint)
        ]
        unsupported_count = len(declared_scalar_joints) - len(scalar_joints)
        if unsupported_count:
            row_consistent = False
            consistency_reasons["unsupported_scalar_joint"] += unsupported_count
        if not scalar_joints:
            row_consistent = False
            consistency_reasons["no_cross_sim_eligible_scalar_joints"] += 1
        for joint in scalar_joints:
            name, joint_type = joint["name"], joint["type"]
            joint_candidates[joint_type] += 1
            traces: dict[str, tuple[list[float], list[int] | None]] = {}
            for simulator in SIMULATORS:
                trace = _trajectory(
                    records[simulator][dataset_id], name, joint_type, expected_steps
                )
                if trace is not None:
                    traces[simulator] = trace
            pair_values: list[float] = []
            for left, right in SIMULATOR_PAIRS:
                if left not in traces or right not in traces:
                    continue
                left_values, left_steps = traces[left]
                right_values, right_steps = traces[right]
                if len(left_values) != len(right_values) or left_steps != right_steps:
                    continue
                pair_values.append(_rmse(left_values, right_values))
            if pair_values:
                maximum = max(pair_values)
                joint_values[joint_type].append(maximum)
                joint_pair_counts[joint_type] += len(pair_values)
                if maximum > thresholds["normalized_joint_rmse"]:
                    row_consistent = False
                    consistency_reasons["joint_rmse_over_threshold"] += 1
            if len(pair_values) != len(SIMULATOR_PAIRS):
                row_consistent = False
                consistency_reasons["joint_trajectory_incomplete"] += 1

            descendants = _descendants(row, joint.get("child"))
            if not descendants:
                row_consistent = False
                consistency_reasons["descendant_topology_missing"] += 1
            for link_name in descendants:
                pose_candidates += 1
                if bbox is None:
                    continue
                poses = {
                    simulator: _pose(
                        records[simulator][dataset_id], name, joint_type, link_name
                    )
                    for simulator in SIMULATORS
                }
                available_poses = {
                    simulator: pose
                    for simulator, pose in poses.items()
                    if pose is not None
                }
                pair_errors = [
                    _pose_error(available_poses[left], available_poses[right], bbox)
                    for left, right in SIMULATOR_PAIRS
                    if left in available_poses and right in available_poses
                ]
                if len(pair_errors) != len(SIMULATOR_PAIRS):
                    row_consistent = False
                    consistency_reasons["descendant_pose_incomplete"] += 1
                if not pair_errors:
                    continue
                translation_max = max(value[0] for value in pair_errors)
                rotation_max = max(value[1] for value in pair_errors)
                pose_translation_values.append(translation_max)
                pose_rotation_values.append(rotation_max)
                pose_pair_count += len(pair_errors)
                if (
                    translation_max > thresholds["translation_over_bbox_diagonal"]
                    or rotation_max > thresholds["rotation_rad"]
                ):
                    row_consistent = False
                    consistency_reasons["descendant_pose_over_threshold"] += 1
        consistent_by_id[dataset_id] = row_consistent

    joint_diagnostics = {
        joint_type: _joint_diagnostic(
            joint_values[joint_type],
            joint_candidates[joint_type],
            joint_pair_counts[joint_type],
            thresholds["normalized_joint_rmse"],
        )
        for joint_type in ("revolute", "prismatic")
    }
    pose_evaluable = len(pose_translation_values)
    link_pose = {
        "candidate_units": pose_candidates,
        "evaluable_units": pose_evaluable,
        "excluded_units": pose_candidates - pose_evaluable,
        "available_pairs": pose_pair_count,
        "coverage": _coverage(pose_evaluable, pose_candidates),
        "translation_over_bbox_diagonal": {
            "threshold": thresholds["translation_over_bbox_diagonal"],
            "units_within_threshold": sum(
                value <= thresholds["translation_over_bbox_diagonal"]
                for value in pose_translation_values
            ),
            **_distribution(pose_translation_values),
        },
        "rotation_rad": {
            "threshold": thresholds["rotation_rad"],
            "units_within_threshold": sum(
                value <= thresholds["rotation_rad"] for value in pose_rotation_values
            ),
            **_distribution(pose_rotation_values),
        },
    }

    upstream: dict[str, dict[str, Any]] = {}
    for gate in STRICT_GATE_NAMES:
        values = [row["_strict_gates"][gate] for row in rows]
        known = [value for value in values if isinstance(value, bool)]
        if len(known) != denominator:
            upstream[gate] = _not_evaluable(
                denominator,
                "upstream_gate_not_evaluable",
                known_passed=sum(known),
                known_denominator=len(known),
            )
        else:
            upstream[gate] = _rate(sum(known), denominator)

    consistency = _rate(sum(consistent_by_id.values()), denominator)
    gates_complete = all(
        row["_strict_gates"][gate] is not None
        for row in rows
        for gate in STRICT_GATE_NAMES
    )
    if gates_complete:
        strict_passed = sum(
            all(row["_strict_gates"][gate] is True for gate in STRICT_GATE_NAMES)
            and all_three_runtime_by_id[row["dataset_id"]]
            and consistent_by_id[row["dataset_id"]]
            for row in rows
        )
        strict_sim_ready = _rate(strict_passed, denominator)
    else:
        applicable_rows = [
            row
            for row in rows
            if all(row["_strict_gates"][gate] is not None for gate in STRICT_GATE_NAMES)
        ]
        known_passed = sum(
            all(row["_strict_gates"][gate] is True for gate in STRICT_GATE_NAMES)
            and all_three_runtime_by_id[row["dataset_id"]]
            and consistent_by_id[row["dataset_id"]]
            for row in applicable_rows
        )
        strict_sim_ready = _not_evaluable(
            denominator,
            "one_or_more_upstream_strict_gates_not_evaluable",
            known_passed=known_passed,
            known_denominator=len(applicable_rows),
        )

    runtime_coverage = {
        simulator: {
            "valid_records": _rate(record_states[simulator]["valid"], denominator),
            "completed_records": _rate(
                terminal_statuses[simulator]["completed"], denominator
            ),
            "record_state_counts": dict(sorted(record_states[simulator].items())),
            "terminal_status_counts": dict(
                sorted(terminal_statuses[simulator].items())
            ),
        }
        for simulator in SIMULATORS
    }
    completeness_by_simulator: dict[str, dict[str, Any]] = {}
    for simulator in SIMULATORS:
        present = denominator - record_states[simulator]["missing"]
        valid = record_states[simulator]["valid"]
        terminal = sum(
            count
            for status, count in terminal_statuses[simulator].items()
            if status
            not in {"missing", "invalid_json", "malformed", "identity_mismatch"}
        )
        completeness_by_simulator[simulator] = {
            "expected_records": denominator,
            "present_records": present,
            "valid_records": valid,
            "terminal_records": terminal,
            "classification": "COMPLETE" if terminal == denominator else "INCOMPLETE",
        }
    expected_records = len(SIMULATORS) * denominator
    present_records = sum(
        item["present_records"] for item in completeness_by_simulator.values()
    )
    valid_records = sum(
        item["valid_records"] for item in completeness_by_simulator.values()
    )
    terminal_records = sum(
        item["terminal_records"] for item in completeness_by_simulator.values()
    )
    completeness = {
        "classification": (
            "COMPLETE" if terminal_records == expected_records else "INCOMPLETE"
        ),
        "expected_records": expected_records,
        "present_records": present_records,
        "valid_records": valid_records,
        "terminal_records": terminal_records,
        "missing_or_invalid_records": expected_records - terminal_records,
        "by_simulator": completeness_by_simulator,
        "rule": "every intent asset has one schema-valid identity-bound terminal record in every simulator; terminal failures count as complete",
    }
    table5b = {
        "per_simulator_pass": {
            simulator: table5a[simulator]["simulator_pass"] for simulator in SIMULATORS
        },
        "all_three_load": _rate(sum(all_three_load_by_id.values()), denominator),
        "all_three_runtime_pass": _rate(
            sum(all_three_runtime_by_id.values()), denominator
        ),
        "joint_normalized_trajectory_pairwise_max_rmse": joint_diagnostics,
        "final_descendant_link_pose_pairwise_max_error": link_pose,
        "coverage": {
            "runtime_records": runtime_coverage,
            "joint_trajectory": {
                joint_type: joint_diagnostics[joint_type]["coverage"]
                for joint_type in ("revolute", "prismatic")
            },
            "descendant_link_pose": link_pose["coverage"],
        },
        "upstream_strict_gates": upstream,
        "strict_consistency": consistency,
        "strict_sim_ready": strict_sim_ready,
    }
    # Short aliases match the established single-dataset Table 5 aggregate schema.
    table5b["joint_rmse"] = joint_diagnostics
    table5b["link_pose_error"] = link_pose
    for gate in STRICT_GATE_NAMES:
        table5b[gate] = upstream[gate]
    asset_outcomes = []
    for row in rows:
        dataset_id = row["dataset_id"]
        row_gates_complete = all(
            row["_strict_gates"][gate] is not None for gate in STRICT_GATE_NAMES
        )
        row_strict = (
            all(row["_strict_gates"][gate] is True for gate in STRICT_GATE_NAMES)
            and all_three_runtime_by_id[dataset_id]
            and consistent_by_id[dataset_id]
            if row_gates_complete
            else None
        )
        asset_outcomes.append(
            {
                "dataset_id": dataset_id,
                "asset_id": row.get("asset_id"),
                "category": row["category"],
                "per_simulator_pass": {
                    simulator: _metric_pass(
                        records[simulator][dataset_id], "simulator_pass"
                    )
                    for simulator in SIMULATORS
                },
                "all_three_load": all_three_load_by_id[dataset_id],
                "all_three_runtime_pass": all_three_runtime_by_id[dataset_id],
                "strict_consistency": consistent_by_id[dataset_id],
                "strict_gates": dict(row["_strict_gates"]),
                "strict_sim_ready": row_strict,
            }
        )
    return {
        "name": dataset["name"],
        "slug": slug,
        "n": denominator,
        "classification": completeness["classification"],
        "completeness": completeness,
        "categories": dict(sorted(Counter(row["category"] for row in rows).items())),
        "table5a": table5a,
        "table5b": table5b,
        "asset_outcomes": asset_outcomes,
        "diagnostics": {
            "consistency_failure_reason_counts": dict(
                sorted(consistency_reasons.items())
            )
        },
    }


def aggregate_manifest(
    manifest: dict[str, Any], run_root: Path | str
) -> dict[str, Any]:
    manifest_sha256, protocol_sha256 = _authority_hashes(manifest)
    normalized = validate_manifest(manifest)
    thresholds = _thresholds(manifest)
    expected_steps = _expected_sample_steps(manifest)
    root = Path(run_root)
    datasets = {
        slug: _aggregate_dataset(
            slug,
            dataset,
            root,
            thresholds,
            expected_steps,
            manifest_sha256,
            protocol_sha256,
        )
        for slug, dataset in normalized.items()
    }
    completeness = {
        "classification": (
            "COMPLETE"
            if all(
                dataset["completeness"]["classification"] == "COMPLETE"
                for dataset in datasets.values()
            )
            else "INCOMPLETE"
        ),
        "expected_records": sum(
            dataset["completeness"]["expected_records"] for dataset in datasets.values()
        ),
        "present_records": sum(
            dataset["completeness"]["present_records"] for dataset in datasets.values()
        ),
        "valid_records": sum(
            dataset["completeness"]["valid_records"] for dataset in datasets.values()
        ),
        "terminal_records": sum(
            dataset["completeness"]["terminal_records"] for dataset in datasets.values()
        ),
    }
    completeness["missing_or_invalid_records"] = (
        completeness["expected_records"] - completeness["terminal_records"]
    )
    return {
        "schema_version": SCHEMA_VERSION,
        "generated_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        "manifest_sha256": manifest_sha256,
        "protocol_sha256": protocol_sha256,
        "simulators": list(SIMULATORS),
        "thresholds": thresholds,
        "classification": completeness["classification"],
        "completeness": completeness,
        "dataset_order": list(datasets),
        "datasets": datasets,
    }


aggregate = aggregate_manifest


def _rate_display(rate: Mapping[str, Any]) -> str:
    if rate.get("status") != "evaluated":
        return "N/E"
    return (
        f"{rate['passed']} / {rate['denominator']} "
        f"({float(rate['percentage']):.3f}%)"
    )


def _rate_csv(row: dict[str, Any], prefix: str, rate: Mapping[str, Any]) -> None:
    row[f"{prefix}_status"] = rate.get("status")
    row[f"{prefix}_passed"] = rate.get("passed")
    row[f"{prefix}_denominator"] = rate.get("denominator")
    percentage = rate.get("percentage")
    row[f"{prefix}_percentage"] = (
        "" if percentage is None else f"{float(percentage):.6f}"
    )
    row[f"{prefix}_display"] = _rate_display(rate)


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
        }
        for simulator in SIMULATORS:
            _rate_csv(row, f"{simulator}_pass", table["per_simulator_pass"][simulator])
        _rate_csv(row, "all_three_load", table["all_three_load"])
        _rate_csv(row, "all_three_runtime_pass", table["all_three_runtime_pass"])
        for joint_type in ("revolute", "prismatic"):
            diagnostic = table["joint_normalized_trajectory_pairwise_max_rmse"][
                joint_type
            ]
            row[f"{joint_type}_joint_candidate_units"] = diagnostic["candidate_units"]
            row[f"{joint_type}_joint_evaluable_units"] = diagnostic["evaluable_units"]
            row[f"{joint_type}_joint_coverage"] = _rate_display(diagnostic["coverage"])
            row[f"{joint_type}_joint_pairwise_max_rmse"] = (
                ""
                if diagnostic["population_max"] is None
                else diagnostic["population_max"]
            )
        pose = table["final_descendant_link_pose_pairwise_max_error"]
        row["pose_candidate_units"] = pose["candidate_units"]
        row["pose_evaluable_units"] = pose["evaluable_units"]
        row["pose_coverage"] = _rate_display(pose["coverage"])
        row["pose_pairwise_max_translation_over_bbox"] = (
            ""
            if pose["translation_over_bbox_diagonal"]["population_max"] is None
            else pose["translation_over_bbox_diagonal"]["population_max"]
        )
        row["pose_pairwise_max_rotation_rad"] = (
            ""
            if pose["rotation_rad"]["population_max"] is None
            else pose["rotation_rad"]["population_max"]
        )
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
    thresholds = summary["thresholds"]
    completeness_label = (
        "> **INCOMPLETE: runtime records are missing or untrusted; all rates below are provisional fail-closed values.**"
        if summary["completeness"]["classification"] == "INCOMPLETE"
        else "**Run completeness: COMPLETE.**"
    )
    lines = [
        "# Table 5 N=200 automatic evaluation",
        "",
        completeness_label,
        "",
        f"Trusted terminal coverage: "
        f"({summary['completeness']['terminal_records']} / "
        f"{summary['completeness']['expected_records']} trusted terminal records).",
        "",
        "Missing, malformed, failed, crashed, and timed-out runtime records fail closed and remain in the full dataset denominator.",
        "",
        "## Table 5a",
        "",
        "| Dataset | State | Simulator | Load | Reset | Settling | Actuation | Limits | Drift | Simulator pass |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for slug in summary["dataset_order"]:
        dataset = summary["datasets"][slug]
        for simulator in SIMULATORS:
            metrics = dataset["table5a"][simulator]
            cells = [_rate_display(metrics[metric]) for metric in TABLE5A_METRICS]
            simulator_state = dataset["completeness"]["by_simulator"][simulator][
                "classification"
            ]
            lines.append(
                f"| {dataset['name']} | {simulator_state} | {simulator} | "
                + " | ".join(cells)
                + " |"
            )
    lines.extend(
        [
            "",
            "## Table 5b",
            "",
            "| Dataset | State | PyBullet pass | Genesis pass | MuJoCo pass | All-three load | All-three runtime | Joint pairwise-max RMSE | Link-pose pairwise-max error | Coverage | Strict Sim-ready |",
            "|---|---|---:|---:|---:|---:|---:|---|---|---|---:|",
        ]
    )
    for slug in summary["dataset_order"]:
        dataset = summary["datasets"][slug]
        table = dataset["table5b"]
        joint = table["joint_normalized_trajectory_pairwise_max_rmse"]
        pose = table["final_descendant_link_pose_pairwise_max_error"]
        joint_cell = (
            f"rev max {joint['revolute']['population_max']}; "
            f"prism max {joint['prismatic']['population_max']}"
        )
        pose_cell = (
            f"trans/bbox max {pose['translation_over_bbox_diagonal']['population_max']}; "
            f"rot max {pose['rotation_rad']['population_max']}"
        )
        coverage_cell = (
            f"rev {_rate_display(joint['revolute']['coverage'])}; "
            f"prism {_rate_display(joint['prismatic']['coverage'])}; "
            f"pose {_rate_display(pose['coverage'])}"
        )
        cells = [
            _rate_display(table["per_simulator_pass"][simulator])
            for simulator in SIMULATORS
        ]
        cells.extend(
            [
                _rate_display(table["all_three_load"]),
                _rate_display(table["all_three_runtime_pass"]),
                joint_cell,
                pose_cell,
                coverage_cell,
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
            "## Contract",
            "",
            f"Joint normalized RMSE threshold: `{thresholds['normalized_joint_rmse']}`.",
            "",
            f"Root-frame descendant translation/bbox threshold: `{thresholds['translation_over_bbox_diagonal']}`; rotation threshold: `{thresholds['rotation_rad']}` rad.",
            "",
            "Cross-simulator coverage is reported over eligible scalar-joint and descendant-link candidate units. A diagnostic unit is evaluable when at least one simulator pair is present and valid; Strict Sim-ready still requires complete all-three evidence.",
            "",
            "Strict Sim-ready is N/E whenever any required upstream strict gate is null; null is never converted to a failed zero-percent result.",
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
            temporary_path = Path(temporary)
            staged[filename] = temporary_path
            with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
        for filename, temporary_path in staged.items():
            os.replace(temporary_path, out / filename)
        directory_fd = os.open(out, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        for temporary_path in staged.values():
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass


def write_outputs(summary: Mapping[str, Any], out: Path | str) -> None:
    json_text = (
        json.dumps(
            summary, ensure_ascii=False, allow_nan=False, sort_keys=True, indent=2
        )
        + "\n"
    )
    _atomic_write_set(
        Path(out),
        {
            "summary.json": json_text,
            "table5a.csv": render_table5a_csv(summary),
            "table5b.csv": render_table5b_csv(summary),
            "report.md": render_report(summary),
        },
    )


def _parse_args(argv: Iterable[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        summary = aggregate_manifest(manifest, args.run_root)
        write_outputs(summary, args.out)
    except (AggregateError, OSError, UnicodeError, json.JSONDecodeError) as error:
        print(f"table5_n200_aggregate: {error}", file=sys.stderr)
        return 2
    print(args.out / "summary.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
