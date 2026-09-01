#!/usr/bin/env python3
"""Validate and aggregate frozen Artiverse Table 5 runtime records."""

from __future__ import annotations

import argparse
import copy
import ctypes
import errno
import fcntl
import json
import math
import os
import shutil
import stat
import statistics
import tempfile
from collections import Counter
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterable

from run_table5_ours import (
    METRIC_NAMES,
    RuntimeContractError,
    adapter_identity,
    quaternion_angular_error,
    validate_terminal_record,
)
from table5_ours_common import (
    ManifestError,
    atomic_write_json,
    canonical_sha256,
    sha256_file,
    validate_manifest,
    validate_protocol_schema,
    validate_receipt_set,
)

AGGREGATE_SCHEMA = "table5_ours_aggregate_v1"
INVENTORY_SCHEMA = "table5_ours_failure_inventory_v1"
SELF_CHECK_SCHEMA = "table5_ours_aggregate_self_check_v1"
PUBLICATION_SCHEMA = "table5_ours_aggregate_publication_v1"
SIMULATORS = ("pybullet", "genesis", "mujoco")
SIMULATOR_PAIRS = (
    ("pybullet", "genesis"),
    ("pybullet", "mujoco"),
    ("genesis", "mujoco"),
)
STRICT_METRICS = tuple(METRIC_NAMES)
AND_METRICS = (
    "load",
    "reset",
    "settling",
    "actuation",
    "limit_enforcement",
    "constraint_drift",
)
TABLE5A_METRICS = STRICT_METRICS + ("strict_collision_pass",)
TABLE5B_RATE_METRICS = (
    "all_three_load",
    "all_three_runtime_pass",
    "strict_urdf_pass",
    "strict_kinematic_pass",
    "strict_collision_pass",
    "strict_consistency",
    "strict_sim_ready",
)
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
SMALL_GROUP_THRESHOLD = 5


class AggregateContractError(ValueError):
    """Raised when runtime evidence cannot support a trustworthy aggregate."""


def _finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _json_finite(value: Any, location: str) -> None:
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise AggregateContractError(
            f"non-finite or non-JSON value at {location}: {error}"
        ) from error


def _rate(passed: int, denominator: int) -> dict[str, int | float]:
    if denominator < 0 or passed < 0 or passed > denominator:
        raise AggregateContractError("invalid rate numerator or denominator")
    return {
        "passed": passed,
        "denominator": denominator,
        "percentage": 0.0 if denominator == 0 else 100.0 * passed / denominator,
    }


def _percentile95(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(0.95 * len(ordered)) - 1)
    return ordered[index]


def _distribution(values: list[float]) -> dict[str, int | float | None]:
    return {
        "population_max": max(values) if values else None,
        "mean": statistics.fmean(values) if values else None,
        "median": statistics.median(values) if values else None,
        "p95": _percentile95(values),
    }


def _row_map(manifest: dict[str, Any]) -> dict[str, dict[str, Any]]:
    rows = manifest.get("rows")
    if not isinstance(rows, list):
        raise AggregateContractError("manifest rows must be a list")
    result: dict[str, dict[str, Any]] = {}
    for order, row in enumerate(rows):
        if not isinstance(row, dict):
            raise AggregateContractError("manifest row is malformed")
        dataset_id = row.get("dataset_id")
        if (
            not isinstance(dataset_id, str)
            or len(dataset_id) != len("artiverse_0000")
            or not dataset_id.startswith("artiverse_")
            or not dataset_id.removeprefix("artiverse_").isdigit()
        ):
            raise AggregateContractError("manifest dataset_id is malformed")
        if (
            dataset_id != f"artiverse_{order:04d}"
            or row.get("order") != order
            or row.get("selection_rank") != order + 1
        ):
            raise AggregateContractError(
                f"manifest dataset_id/order/rank binding mismatch at row {order}"
            )
        if dataset_id in result:
            raise AggregateContractError(f"duplicate manifest dataset ID: {dataset_id}")
        manifest_root = row.get("manifest_root")
        asset_id = row.get("asset_id")
        if (
            not isinstance(manifest_root, str)
            or not manifest_root
            or asset_id != manifest_root
        ):
            raise AggregateContractError(
                f"manifest_root/asset_id authority mismatch for {dataset_id}"
            )
        category = row.get("raw_category")
        if (
            not isinstance(category, str)
            or not category
            or row.get("category") != category
        ):
            raise AggregateContractError(
                f"manifest category is not an exact raw string for {dataset_id}"
            )
        preflight = row.get("preflight")
        if not (
            isinstance(preflight, dict)
            and preflight.get("status") in {"pass", "failed"}
            and isinstance(preflight.get("issues"), list)
            and all(isinstance(issue, str) and issue for issue in preflight["issues"])
            and isinstance(preflight.get("simulator_eligible"), bool)
            and (preflight["status"] == "pass") is preflight["simulator_eligible"]
            and (preflight["issues"] == []) is preflight["simulator_eligible"]
        ):
            raise AggregateContractError(
                f"manifest preflight is malformed for {dataset_id}"
            )
        diagonal = row.get("bounding_box_diagonal")
        preflight_eligible = preflight["simulator_eligible"]
        if preflight_eligible is True and (
            not _finite_number(diagonal) or float(diagonal) <= 0
        ):
            raise AggregateContractError(
                f"bounding box diagonal must be positive and finite for {dataset_id}"
            )
        if (
            preflight_eligible is False
            and diagonal is not None
            and (not _finite_number(diagonal) or float(diagonal) <= 0)
        ):
            raise AggregateContractError(
                f"preflight-failed bounding box is malformed for {dataset_id}"
            )
        strict_gates = row.get("strict_gates")
        gate_paths = (
            ("table2", "strict_urdf_pass"),
            ("table3", "strict_kinematic_pass"),
            ("table4", "strict_collision_pass"),
        )
        if not isinstance(strict_gates, dict) or any(
            not isinstance(strict_gates.get(table), dict)
            or not isinstance(strict_gates[table].get(field), bool)
            for table, field in gate_paths
        ):
            raise AggregateContractError(
                f"manifest strict gates are malformed for {dataset_id}"
            )
        result[dataset_id] = row
    return result


def _validate_protocol_manifest(
    protocol: dict[str, Any], manifest: dict[str, Any]
) -> dict[str, dict[str, Any]]:
    _json_finite(protocol, "protocol")
    _json_finite(manifest, "manifest")
    schema_protocol = copy.deepcopy(protocol)
    try:
        schema_protocol["cross_simulator"]["all_three_denominator"] = 800
        validate_protocol_schema(schema_protocol)
    except (ManifestError, KeyError, TypeError, ValueError) as error:
        raise AggregateContractError(
            f"protocol schema/semantic mismatch: {error}"
        ) from error
    rows = _row_map(manifest)
    selection = manifest.get("selection")
    if not isinstance(selection, dict) or selection.get("selected_count") != len(rows):
        raise AggregateContractError("manifest selected count mismatch")
    if protocol.get("selection", {}).get("selected_count") != len(rows):
        raise AggregateContractError("protocol selected count mismatch")
    if protocol.get("cross_simulator", {}).get("all_three_denominator") != len(rows):
        raise AggregateContractError("protocol all-three denominator mismatch")
    expected_protocol = canonical_sha256(
        protocol, exclude_fields={"protocol_sha256", "generated_at"}
    )
    if protocol.get("protocol_sha256") != expected_protocol:
        raise AggregateContractError("protocol self-hash mismatch")
    if manifest.get("protocol_sha256") != protocol.get("protocol_sha256"):
        raise AggregateContractError("protocol/manifest binding mismatch")
    expected_cohort = canonical_sha256(
        manifest, exclude_fields={"cohort_sha256", "generated_at"}
    )
    if manifest.get("cohort_sha256") != expected_cohort:
        raise AggregateContractError("manifest cohort hash mismatch")
    return rows


def _authoritative_receipt(
    manifest: dict[str, Any], receipt_info: dict[str, Any] | None
) -> dict[str, Any]:
    if receipt_info is None:
        receipt_info = {}
    if not isinstance(receipt_info, dict):
        raise AggregateContractError("receipt_info must be a JSON object")
    authoritative = {
        "protocol_sha256": manifest["protocol_sha256"],
        "cohort_sha256": manifest["cohort_sha256"],
    }
    for key, expected in authoritative.items():
        if key in receipt_info and receipt_info[key] != expected:
            raise AggregateContractError(
                f"receipt {key} conflicts with authoritative manifest binding"
            )
    supplemental = {
        key: copy.deepcopy(value)
        for key, value in receipt_info.items()
        if key not in authoritative
    }
    return {**authoritative, **supplemental}


def _validate_receipt_provenance(
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    receipt_info: dict[str, Any] | None,
    *,
    required: bool,
) -> bool:
    if receipt_info is None or receipt_info == {}:
        if required:
            raise AggregateContractError("authoritative receipt provenance is required")
        return False
    if not isinstance(receipt_info, dict):
        raise AggregateContractError("receipt_info must be a JSON object")
    provenance_fields = {
        "receipt_set_sha256",
        "protocol_file_sha256",
        "manifest_file_sha256",
        "receipt_root",
        "dataset_root",
        "table1_manifest",
        "upstream_roots",
        "runtime_inputs",
    }
    authoritative_fields = {"protocol_sha256", "cohort_sha256"}
    if frozenset(receipt_info) not in {
        frozenset(provenance_fields),
        frozenset(provenance_fields | authoritative_fields),
    }:
        raise AggregateContractError(
            "receipt provenance fields are incomplete or unexpected"
        )

    protocol_file_sha256 = canonical_sha256(protocol)
    manifest_file_sha256 = canonical_sha256(manifest)
    marker = {
        "schema_version": "table5_artiverse_receipt_set_v1",
        "protocol_sha256": protocol_file_sha256,
        "manifest_sha256": manifest_file_sha256,
    }
    expected_hashes = {
        "protocol_file_sha256": protocol_file_sha256,
        "manifest_file_sha256": manifest_file_sha256,
        "receipt_set_sha256": canonical_sha256(marker),
    }
    if any(
        receipt_info.get(field) != expected
        for field, expected in expected_hashes.items()
    ):
        raise AggregateContractError(
            "receipt provenance hash does not match authoritative inputs"
        )
    for field in ("receipt_root", "dataset_root", "table1_manifest"):
        value = receipt_info.get(field)
        if not isinstance(value, str) or not value or not Path(value).is_absolute():
            raise AggregateContractError(
                f"receipt provenance {field} must be an absolute path"
            )
    upstream_roots = receipt_info.get("upstream_roots")
    if not isinstance(upstream_roots, dict) or set(upstream_roots) != {
        "table2",
        "table3",
        "table4",
    }:
        raise AggregateContractError("receipt provenance upstream_roots is incomplete")
    if any(
        not isinstance(value, str) or not value or not Path(value).is_absolute()
        for value in upstream_roots.values()
    ):
        raise AggregateContractError(
            "receipt provenance upstream roots must be absolute paths"
        )
    return True


def _validate_source_authority(
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    receipt_info: dict[str, Any] | None,
    *,
    required: bool,
) -> bool:
    if receipt_info is None or receipt_info == {}:
        if required:
            raise AggregateContractError("authoritative source provenance is required")
        return False
    dataset_root = Path(receipt_info["dataset_root"])
    table1_manifest = Path(receipt_info["table1_manifest"])
    upstream_roots = {
        name: Path(value) for name, value in receipt_info["upstream_roots"].items()
    }
    expected_dataset_root = manifest.get("source_receipt", {}).get("dataset_root")
    expected_table1 = manifest.get("source_receipt", {}).get("table1_manifest_path")
    expected_upstream = {
        name: manifest.get("upstream_artifacts", {}).get(name, {}).get("root")
        for name in ("table2", "table3", "table4")
    }
    if str(dataset_root.resolve()) != expected_dataset_root:
        raise AggregateContractError(
            "dataset root does not match frozen manifest source receipt"
        )
    if str(table1_manifest.resolve()) != expected_table1:
        raise AggregateContractError(
            "Table 1 manifest path does not match frozen source receipt"
        )
    if any(
        str(upstream_roots[name].resolve()) != expected_upstream[name]
        for name in expected_upstream
    ):
        raise AggregateContractError(
            "upstream root does not match frozen manifest artifact receipt"
        )
    try:
        validate_manifest(
            manifest,
            dataset_root,
            table1_manifest,
            upstream_roots,
            protocol=protocol,
            formal=True,
        )
    except (ManifestError, OSError, KeyError, TypeError, ValueError) as error:
        raise AggregateContractError(
            f"authoritative source revalidation failed: {error}"
        ) from error
    return True


def _paths_overlap(left: Path, right: Path) -> bool:
    return left == right or left in right.parents or right in left.parents


def _validate_publication_location(
    output_root: Path,
    receipt_info: dict[str, Any],
    *,
    phase: str,
) -> Path:
    if phase not in {"formal", "qualification"}:
        raise AggregateContractError(
            "publication phase must be formal or qualification"
        )
    unresolved_output_root = Path(output_root)
    if unresolved_output_root.is_symlink():
        raise AggregateContractError(
            "refusing to publish through an aggregate output symlink"
        )
    output_root = unresolved_output_root.resolve()
    receipt_root = Path(receipt_info["receipt_root"]).resolve()
    expected_below_receipt = receipt_root / "aggregate" / phase
    if output_root != expected_below_receipt:
        raise AggregateContractError(
            f"aggregate output must resolve exactly to {expected_below_receipt}"
        )
    protected = {
        "dataset root": Path(receipt_info["dataset_root"]).resolve(),
        "Table 1 artifact root": Path(receipt_info["table1_manifest"]).resolve().parent,
        **{
            f"{name} upstream root": Path(path).resolve()
            for name, path in receipt_info["upstream_roots"].items()
        },
    }
    for label, root in protected.items():
        if _paths_overlap(output_root, root):
            raise AggregateContractError(
                f"aggregate output overlaps authoritative {label}: {root}"
            )
    return output_root


def _runtime_summary(
    records: list[dict[str, Any]],
    *,
    intent_count: int,
    simulator: str,
    phase: str,
    effective_workers: int,
) -> dict[str, Any]:
    if (
        not isinstance(effective_workers, int)
        or isinstance(effective_workers, bool)
        or effective_workers < 1
    ):
        raise AggregateContractError(
            f"runtime effective_workers is malformed for {simulator}"
        )
    return {
        "schema_version": "table5_artiverse_runtime_summary_v1",
        "run_phase": phase,
        "simulator": simulator,
        "effective_workers": effective_workers,
        "intent_count": intent_count,
        "terminal_count": len(records),
        "remaining_count": intent_count - len(records),
        "complete": len(records) == intent_count,
        "terminal_status_counts": dict(
            sorted(Counter(record["terminal_status"] for record in records).items())
        ),
        "metric_pass_counts": {
            metric: sum(record["metrics"][metric] is True for record in records)
            for metric in STRICT_METRICS
        },
        "metric_denominator": intent_count,
    }


def _consistent_record_effective_workers(
    records: list[dict[str, Any]],
    *,
    simulator: str,
) -> int | None:
    observed: set[int] = set()
    for record in records:
        identity = record.get("identity") if isinstance(record, dict) else None
        workers = (
            identity.get("effective_workers") if isinstance(identity, dict) else None
        )
        if not isinstance(workers, int) or isinstance(workers, bool) or workers < 1:
            raise AggregateContractError(
                f"terminal effective_workers is malformed in {simulator}"
            )
        observed.add(workers)
    if len(observed) > 1:
        raise AggregateContractError(
            f"mixed effective_workers in {simulator}: {sorted(observed)}"
        )
    return next(iter(observed)) if observed else None


def _expected_runtime_inputs(
    records_by_simulator: dict[str, list[dict[str, Any]]],
    *,
    phase: str,
    intent_count: int,
    effective_workers_by_simulator: dict[str, int] | None = None,
) -> dict[str, dict[str, Any]]:
    if effective_workers_by_simulator is None:
        effective_workers_by_simulator = {simulator: 1 for simulator in SIMULATORS}
    if set(effective_workers_by_simulator) != set(SIMULATORS):
        raise AggregateContractError(
            "runtime effective_workers must name exactly all three simulators"
        )
    result: dict[str, dict[str, Any]] = {}
    for simulator in SIMULATORS:
        records = sorted(
            records_by_simulator[simulator],
            key=lambda record: record["identity"]["dataset_id"],
        )
        record_workers = _consistent_record_effective_workers(
            records,
            simulator=simulator,
        )
        expected_workers = effective_workers_by_simulator[simulator]
        if record_workers is not None and record_workers != expected_workers:
            raise AggregateContractError(
                f"runtime effective_workers does not match terminal identities in {simulator}"
            )
        record_hashes = [
            {
                "filename": f"{record['identity']['dataset_id']}.json",
                "sha256": canonical_sha256(record),
            }
            for record in records
        ]
        implementation_hashes = {
            record["identity"]["adapter_implementation_sha256"] for record in records
        }
        if len(implementation_hashes) > 1:
            raise AggregateContractError(
                f"mixed adapter implementation receipts in {simulator}"
            )
        summary = _runtime_summary(
            records,
            intent_count=intent_count,
            simulator=simulator,
            phase=phase,
            effective_workers=effective_workers_by_simulator[simulator],
        )
        progress = copy.deepcopy(summary)
        progress["schema_version"] = "table5_artiverse_runtime_progress_v1"
        result[simulator] = {
            "present": True,
            "effective_workers": effective_workers_by_simulator[simulator],
            "intent_count": intent_count,
            "terminal_count": len(records),
            "complete": len(records) == intent_count,
            "record_file_hashes": {
                item["filename"]: item["sha256"] for item in record_hashes
            },
            "record_set_sha256": canonical_sha256(record_hashes),
            "summary_sha256": canonical_sha256(summary),
            "progress_sha256": canonical_sha256(progress),
            "adapter_implementation_sha256": (
                next(iter(implementation_hashes)) if implementation_hashes else None
            ),
        }
    return result


def _validate_runtime_receipt_info(
    receipt_info: dict[str, Any] | None,
    records_by_simulator: dict[str, list[dict[str, Any]]],
    *,
    phase: str,
    intent_count: int,
    required: bool,
) -> bool:
    runtime_inputs = (
        receipt_info.get("runtime_inputs") if isinstance(receipt_info, dict) else None
    )
    if runtime_inputs is None:
        if required:
            raise AggregateContractError(
                "authoritative runtime input receipt is required for publication"
            )
        return False
    if not isinstance(runtime_inputs, dict) or set(runtime_inputs) != set(SIMULATORS):
        raise AggregateContractError(
            "runtime input receipt must name exactly all three simulators"
        )
    effective_workers = {
        simulator: runtime_inputs[simulator].get("effective_workers")
        for simulator in SIMULATORS
        if isinstance(runtime_inputs.get(simulator), dict)
    }
    expected = _expected_runtime_inputs(
        records_by_simulator,
        phase=phase,
        intent_count=intent_count,
        effective_workers_by_simulator=effective_workers,
    )
    if runtime_inputs != expected:
        raise AggregateContractError(
            "runtime input receipt does not match records/summary/progress"
        )
    complete = all(
        runtime_inputs[simulator]["complete"] is True for simulator in SIMULATORS
    )
    if required and not complete:
        raise AggregateContractError(
            "authoritative runtime input receipt is incomplete"
        )
    return complete


def _expected_identity(
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    row: dict[str, Any],
    record: dict[str, Any],
    simulator: str,
    phase: str,
) -> dict[str, Any]:
    provenance = record.get("provenance")
    receipt = (
        provenance.get("adapter_implementation_receipt")
        if isinstance(provenance, dict)
        else None
    )
    if not isinstance(receipt, dict):
        raise AggregateContractError(
            f"terminal record lacks adapter implementation receipt for {row['dataset_id']}/{simulator}"
        )
    identity = record.get("identity")
    if not isinstance(identity, dict):
        raise AggregateContractError(
            f"terminal record identity is malformed for {row['dataset_id']}/{simulator}"
        )
    return adapter_identity(
        protocol,
        manifest,
        row,
        simulator,
        phase,
        implementation_receipt=receipt,
        workers=identity.get("effective_workers"),
    )


def _validate_record(
    record: Any,
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    row: dict[str, Any],
    simulator: str,
    phase: str,
) -> None:
    if not isinstance(record, dict):
        raise AggregateContractError(
            f"terminal record is malformed for {row['dataset_id']}/{simulator}"
        )
    if record.get("identity", {}).get("dataset_id") != row["dataset_id"]:
        raise AggregateContractError(
            f"unknown or binding-mismatched dataset ID for {simulator}"
        )
    expected = _expected_identity(protocol, manifest, row, record, simulator, phase)
    try:
        validate_terminal_record(record, expected, row, protocol)
    except RuntimeContractError as error:
        raise AggregateContractError(
            f"terminal record binding/schema failure for {row['dataset_id']}/{simulator}: {error}"
        ) from error
    status = record.get("terminal_status")
    if status not in TERMINAL_STATUSES:
        raise AggregateContractError(
            f"invalid terminal status for {row['dataset_id']}/{simulator}"
        )
    metrics = record["metrics"]
    if metrics.get("simulator_pass") != all(
        metrics.get(metric) is True for metric in AND_METRICS
    ):
        raise AggregateContractError(
            f"simulator_pass is not the six-metric AND for {row['dataset_id']}/{simulator}"
        )
    if metrics.get("load") != record["load"].get("strict_load"):
        raise AggregateContractError(
            f"load metric/detail mismatch for {row['dataset_id']}/{simulator}"
        )
    if status != "completed" and any(
        metrics.get(metric) is not False for metric in STRICT_METRICS
    ):
        raise AggregateContractError(
            f"terminal failure metrics must all be false for {row['dataset_id']}/{simulator}"
        )
    _json_finite(record, f"record {row['dataset_id']}/{simulator}")


def _index_records(
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    row_by_id: dict[str, dict[str, Any]],
    records_by_simulator: dict[str, list[dict[str, Any]]],
    phase: str,
    intent_ids: list[str],
) -> tuple[
    dict[str, dict[str, dict[str, Any]]],
    list[dict[str, Any]],
    dict[str, dict[str, Any]],
]:
    if set(records_by_simulator) != set(SIMULATORS):
        raise AggregateContractError(
            "records must name exactly pybullet, genesis, and mujoco"
        )
    intent_set = set(intent_ids)
    indexed: dict[str, dict[str, dict[str, Any]]] = {}
    inventory: list[dict[str, Any]] = []
    coverage: dict[str, dict[str, Any]] = {}
    for simulator in SIMULATORS:
        entries = records_by_simulator[simulator]
        if not isinstance(entries, list):
            raise AggregateContractError(f"records for {simulator} must be a list")
        _consistent_record_effective_workers(entries, simulator=simulator)
        by_id: dict[str, dict[str, Any]] = {}
        duplicates: list[str] = []
        implementation_hashes: set[str] = set()
        for record in entries:
            if not isinstance(record, dict):
                raise AggregateContractError(
                    f"malformed terminal record in {simulator}"
                )
            dataset_id = record.get("identity", {}).get("dataset_id")
            if not isinstance(dataset_id, str) or dataset_id not in row_by_id:
                raise AggregateContractError(
                    f"unknown or malformed dataset ID in {simulator}: {dataset_id}"
                )
            if dataset_id not in intent_set:
                raise AggregateContractError(
                    f"record ID is outside {phase} intent in {simulator}: {dataset_id}"
                )
            if dataset_id in by_id:
                duplicates.append(dataset_id)
                continue
            _validate_record(
                record, protocol, manifest, row_by_id[dataset_id], simulator, phase
            )
            by_id[dataset_id] = record
            implementation_hashes.add(
                record["identity"]["adapter_implementation_sha256"]
            )
        if duplicates:
            raise AggregateContractError(
                f"duplicate terminal records in {simulator}: {sorted(duplicates)}"
            )
        if len(implementation_hashes) > 1:
            raise AggregateContractError(
                f"mixed adapter implementation receipts in {simulator}"
            )
        missing = [dataset_id for dataset_id in intent_ids if dataset_id not in by_id]
        inventory.extend(
            {
                "dataset_id": dataset_id,
                "asset_id": row_by_id[dataset_id]["manifest_root"],
                "manifest_root": row_by_id[dataset_id]["manifest_root"],
                "simulator": simulator,
                "reason": "missing_record",
            }
            for dataset_id in missing
        )
        for dataset_id, record in sorted(by_id.items()):
            failures = [
                metric
                for metric in STRICT_METRICS
                if record["metrics"][metric] is False
            ]
            if record["terminal_status"] != "completed" or failures:
                entry = {
                    "dataset_id": dataset_id,
                    "asset_id": row_by_id[dataset_id]["manifest_root"],
                    "manifest_root": row_by_id[dataset_id]["manifest_root"],
                    "simulator": simulator,
                    "reason": (
                        "terminal_failure"
                        if record["terminal_status"] != "completed"
                        else "strict_metric_failure"
                    ),
                    "terminal_status": record["terminal_status"],
                    "failed_metrics": failures,
                }
                if record["terminal_status"] == "diagnostic_failure":
                    entry["diagnostic_failure"] = copy.deepcopy(
                        record["diagnostics"]["diagnostic_failure"]
                    )
                inventory.append(entry)
        indexed[simulator] = by_id
        coverage[simulator] = {
            "intent_count": len(intent_ids),
            "seen_count": len(by_id),
            "missing_count": len(missing),
            "missing_ids": missing,
            "duplicate_count": 0,
            "terminal_status_counts": dict(
                sorted(
                    Counter(
                        record["terminal_status"] for record in by_id.values()
                    ).items()
                )
            ),
        }
    return indexed, inventory, coverage


def _strict_inventory_entry(
    row: dict[str, Any], reason: str, **details: Any
) -> dict[str, Any]:
    return {
        "dataset_id": row["dataset_id"],
        "asset_id": row["manifest_root"],
        "manifest_root": row["manifest_root"],
        "reason": reason,
        **details,
    }


def _inventory_identity(row: dict[str, Any]) -> dict[str, str]:
    return {
        "dataset_id": row["dataset_id"],
        "asset_id": row["manifest_root"],
        "manifest_root": row["manifest_root"],
    }


def _strict_supported_joints(row: dict[str, Any]) -> list[dict[str, Any]]:
    joint_tree = row.get("joint_tree")
    if (
        joint_tree is None
        and row.get("preflight", {}).get("simulator_eligible") is False
    ):
        return []
    joints = joint_tree.get("joints", []) if isinstance(joint_tree, dict) else None
    if not isinstance(joints, list):
        raise AggregateContractError(
            f"manifest joint tree is malformed for {row['dataset_id']}"
        )
    supported: list[dict[str, Any]] = []
    for joint in joints:
        if not isinstance(joint, dict) or joint.get("type") not in {
            "revolute",
            "prismatic",
        }:
            continue
        lower, upper = joint.get("lower"), joint.get("upper")
        effort, velocity = joint.get("effort"), joint.get("velocity")
        if (
            _finite_number(lower)
            and _finite_number(upper)
            and float(lower) < float(upper)
            and _finite_number(effort)
            and float(effort) > 0
            and _finite_number(velocity)
            and float(velocity) > 0
        ):
            supported.append(joint)
    return supported


def _movable_joints(row: dict[str, Any]) -> list[dict[str, Any]]:
    joint_tree = row.get("joint_tree")
    if (
        joint_tree is None
        and row.get("preflight", {}).get("simulator_eligible") is False
    ):
        return []
    joints = joint_tree.get("joints", []) if isinstance(joint_tree, dict) else None
    if not isinstance(joints, list) or any(
        not isinstance(joint, dict) for joint in joints
    ):
        raise AggregateContractError(
            f"manifest joint tree is malformed for {row['dataset_id']}"
        )
    return [joint for joint in joints if joint.get("type") != "fixed"]


def _strict_asset_outcomes(
    protocol: dict[str, Any],
    row_by_id: dict[str, dict[str, Any]],
    indexed: dict[str, dict[str, dict[str, Any]]],
    intent_ids: list[str],
) -> tuple[dict[str, dict[str, bool]], list[dict[str, Any]]]:
    """Compute the per-asset publication gates without treating partial pairs as strict evidence."""
    thresholds = protocol["cross_simulator"]["thresholds"]
    joint_threshold = float(thresholds["normalized_joint_rmse"])
    translation_threshold = float(thresholds["translation_over_bbox_diagonal"])
    rotation_threshold = float(thresholds["rotation_rad"])
    sample_steps = protocol["cross_simulator"]["joint_rmse"]["sample_steps"]
    outcomes: dict[str, dict[str, bool]] = {}
    inventory: list[dict[str, Any]] = []
    for dataset_id in intent_ids:
        row = row_by_id[dataset_id]
        consistency = True
        records: dict[str, dict[str, Any]] = {}
        for simulator in SIMULATORS:
            record = indexed.get(simulator, {}).get(dataset_id)
            if record is None:
                consistency = False
                inventory.append(
                    _strict_inventory_entry(
                        row, "strict_missing_record", simulator=simulator
                    )
                )
                continue
            identity = record.get("identity", {})
            if not (
                identity.get("dataset_id") == dataset_id
                and identity.get("asset_id") == row["manifest_root"]
                and identity.get("manifest_root") == row["manifest_root"]
            ):
                consistency = False
                inventory.append(
                    _strict_inventory_entry(
                        row,
                        "strict_manifest_root_binding_mismatch",
                        simulator=simulator,
                    )
                )
                continue
            records[simulator] = record
            if record.get("terminal_status") != "completed":
                consistency = False
                inventory.append(
                    _strict_inventory_entry(
                        row,
                        "strict_terminal_not_completed",
                        simulator=simulator,
                        terminal_status=record.get("terminal_status"),
                    )
                )
                continue
            metrics = record.get("metrics")
            if (
                not isinstance(metrics, dict)
                or metrics.get("simulator_pass") is not True
            ):
                consistency = False
                failed_metrics = [
                    metric
                    for metric in STRICT_METRICS
                    if not isinstance(metrics, dict) or metrics.get(metric) is not True
                ]
                inventory.append(
                    _strict_inventory_entry(
                        row,
                        "strict_runtime_prerequisite_failed",
                        simulator=simulator,
                        failed_metrics=failed_metrics,
                    )
                )

        strict_supported_names = {
            joint["name"] for joint in _strict_supported_joints(row)
        }
        for joint in _movable_joints(row):
            joint_name = joint.get("name")
            joint_type = joint.get("type")
            for simulator in SIMULATORS:
                record = records.get(simulator)
                if record is None or record.get("terminal_status") != "completed":
                    continue
                support = _supported_by_name(record).get(joint_name)
                support_complete = (
                    joint_name in strict_supported_names
                    and isinstance(support, dict)
                    and support.get("type") == joint_type
                    and support.get("eligible") is True
                    and support.get("runtime_mapped") is True
                )
                if support_complete:
                    continue
                consistency = False
                if isinstance(support, dict):
                    unsupported_reason = support.get("unsupported_reason")
                    if (
                        not isinstance(unsupported_reason, str)
                        or not unsupported_reason
                    ):
                        unsupported_reason = "runtime_support_incomplete"
                elif joint_name not in strict_supported_names:
                    unsupported_reason = (
                        "manifest_joint_not_supported_by_strict_protocol"
                    )
                else:
                    unsupported_reason = "missing_support_entry"
                inventory.append(
                    _strict_inventory_entry(
                        row,
                        "strict_unsupported_movable_joint",
                        simulator=simulator,
                        joint_name=joint_name,
                        joint_type=joint_type,
                        unsupported_reason=unsupported_reason,
                    )
                )

        for joint in _strict_supported_joints(row):
            joint_name = joint["name"]
            joint_type = joint["type"]
            traces: dict[str, list[float]] = {}
            poses: dict[str, dict[str, dict[str, Any]]] = {}
            descendants = _descendant_names(row, joint["child"])
            for simulator in SIMULATORS:
                record = records.get(simulator)
                if record is None or record.get("terminal_status") != "completed":
                    continue
                support = _supported_by_name(record).get(joint_name)
                if not (
                    isinstance(support, dict)
                    and support.get("type") == joint_type
                    and support.get("eligible") is True
                    and support.get("runtime_mapped") is True
                ):
                    consistency = False
                    inventory.append(
                        _strict_inventory_entry(
                            row,
                            "strict_supported_joint_incomplete",
                            simulator=simulator,
                            joint_name=joint_name,
                            joint_type=joint_type,
                        )
                    )
                    continue
                diagnostic = _actuation_by_name(record).get(joint_name)
                trajectory = (
                    diagnostic.get("trajectory")
                    if isinstance(diagnostic, dict)
                    else None
                )
                values = (
                    trajectory.get("normalized_positions")
                    if isinstance(trajectory, dict)
                    else None
                )
                if not (
                    isinstance(trajectory, dict)
                    and trajectory.get("sample_steps") == sample_steps
                    and isinstance(values, list)
                    and len(values) == len(sample_steps)
                    and all(_finite_number(value) for value in values)
                ):
                    consistency = False
                    inventory.append(
                        _strict_inventory_entry(
                            row,
                            "strict_missing_joint_trajectory",
                            simulator=simulator,
                            joint_name=joint_name,
                            joint_type=joint_type,
                        )
                    )
                    continue
                traces[simulator] = [float(value) for value in values]
                pose_map = diagnostic.get("final_descendant_root_frame_poses")
                if not isinstance(pose_map, dict):
                    pose_map = {}
                poses[simulator] = pose_map
                for descendant in descendants:
                    if descendant not in pose_map:
                        consistency = False
                        inventory.append(
                            _strict_inventory_entry(
                                row,
                                "strict_missing_descendant_pose",
                                simulator=simulator,
                                joint_name=joint_name,
                                joint_type=joint_type,
                                link_name=descendant,
                            )
                        )

            if set(traces) != set(SIMULATORS):
                consistency = False
            else:
                for left, right in SIMULATOR_PAIRS:
                    rmse = math.sqrt(
                        statistics.fmean(
                            (a - b) ** 2 for a, b in zip(traces[left], traces[right])
                        )
                    )
                    if rmse > joint_threshold:
                        consistency = False
                        inventory.append(
                            _strict_inventory_entry(
                                row,
                                "strict_joint_rmse_over_threshold",
                                joint_name=joint_name,
                                joint_type=joint_type,
                                simulator_pair=[left, right],
                                rmse=rmse,
                                threshold=joint_threshold,
                            )
                        )
            for descendant in descendants:
                if not all(
                    descendant in poses.get(simulator, {}) for simulator in SIMULATORS
                ):
                    consistency = False
                    continue
                for left, right in SIMULATOR_PAIRS:
                    translation, rotation = _pose_error(
                        poses[left][descendant],
                        poses[right][descendant],
                        float(row["bounding_box_diagonal"]),
                    )
                    if (
                        translation > translation_threshold
                        or rotation > rotation_threshold
                    ):
                        consistency = False
                        inventory.append(
                            _strict_inventory_entry(
                                row,
                                "strict_pose_pair_over_threshold",
                                joint_name=joint_name,
                                joint_type=joint_type,
                                link_name=descendant,
                                simulator_pair=[left, right],
                                translation_over_bbox_diagonal=translation,
                                rotation_error_rad=rotation,
                                translation_threshold=translation_threshold,
                                rotation_threshold=rotation_threshold,
                            )
                        )

        gates = row["strict_gates"]
        strict_urdf = gates["table2"]["strict_urdf_pass"]
        strict_kinematic = gates["table3"]["strict_kinematic_pass"]
        strict_collision = gates["table4"]["strict_collision_pass"]
        all_three_runtime = all(
            indexed.get(simulator, {})
            .get(dataset_id, {})
            .get("metrics", {})
            .get("simulator_pass")
            is True
            for simulator in SIMULATORS
        )
        outcomes[dataset_id] = {
            "strict_urdf_pass": strict_urdf,
            "strict_kinematic_pass": strict_kinematic,
            "strict_collision_pass": strict_collision,
            "all_three_runtime_pass": all_three_runtime,
            "strict_consistency": consistency,
            "strict_sim_ready": all(
                (
                    strict_urdf,
                    strict_kinematic,
                    strict_collision,
                    all_three_runtime,
                    consistency,
                )
            ),
        }
    return outcomes, inventory


def _metric_tables(
    indexed: dict[str, dict[str, dict[str, Any]]],
    intent_ids: list[str],
    strict_outcomes: dict[str, dict[str, bool]],
) -> tuple[dict[str, Any], dict[str, Any]]:
    denominator = len(intent_ids)
    table5a: dict[str, Any] = {}
    for simulator in SIMULATORS:
        metrics = {
            metric: _rate(
                sum(
                    indexed[simulator]
                    .get(dataset_id, {})
                    .get("metrics", {})
                    .get(metric)
                    is True
                    for dataset_id in intent_ids
                ),
                denominator,
            )
            for metric in STRICT_METRICS
        }
        metrics["strict_collision_pass"] = _rate(
            sum(
                strict_outcomes[dataset_id]["strict_collision_pass"]
                for dataset_id in intent_ids
            ),
            denominator,
        )
        table5a[simulator] = metrics
    all_three_load = sum(
        all(
            indexed[simulator].get(dataset_id, {}).get("metrics", {}).get("load")
            is True
            for simulator in SIMULATORS
        )
        for dataset_id in intent_ids
    )
    all_three_runtime = sum(
        all(
            indexed[simulator]
            .get(dataset_id, {})
            .get("metrics", {})
            .get("simulator_pass")
            is True
            for simulator in SIMULATORS
        )
        for dataset_id in intent_ids
    )
    table5b = {
        "per_simulator_pass": {
            simulator: copy.deepcopy(table5a[simulator]["simulator_pass"])
            for simulator in SIMULATORS
        },
        "all_three_load": _rate(all_three_load, denominator),
        "all_three_runtime_pass": _rate(all_three_runtime, denominator),
        "strict_urdf_pass": _rate(
            sum(
                strict_outcomes[dataset_id]["strict_urdf_pass"]
                for dataset_id in intent_ids
            ),
            denominator,
        ),
        "strict_kinematic_pass": _rate(
            sum(
                strict_outcomes[dataset_id]["strict_kinematic_pass"]
                for dataset_id in intent_ids
            ),
            denominator,
        ),
        "strict_collision_pass": _rate(
            sum(
                strict_outcomes[dataset_id]["strict_collision_pass"]
                for dataset_id in intent_ids
            ),
            denominator,
        ),
        "strict_consistency": _rate(
            sum(
                strict_outcomes[dataset_id]["strict_consistency"]
                for dataset_id in intent_ids
            ),
            denominator,
        ),
        "strict_sim_ready": _rate(
            sum(
                strict_outcomes[dataset_id]["strict_sim_ready"]
                for dataset_id in intent_ids
            ),
            denominator,
        ),
    }
    return table5a, table5b


def _category_report(
    row_by_id: dict[str, dict[str, Any]],
    indexed: dict[str, dict[str, dict[str, Any]]],
    intent_ids: list[str],
    strict_outcomes: dict[str, dict[str, bool]],
) -> dict[str, Any]:
    groups: dict[str, list[str]] = {}
    for dataset_id in intent_ids:
        groups.setdefault(row_by_id[dataset_id]["raw_category"], []).append(dataset_id)

    def category_metrics(ids: list[str]) -> dict[str, Any]:
        table5a, table5b = _metric_tables(indexed, ids, strict_outcomes)
        return {
            "table5a": table5a,
            "table5b": {
                "per_simulator_pass": table5b["per_simulator_pass"],
                **{metric: table5b[metric] for metric in TABLE5B_RATE_METRICS},
            },
        }

    micro = category_metrics(intent_ids)
    group_rows: list[dict[str, Any]] = []
    for category, ids in groups.items():
        warnings: list[str] = []
        if len(ids) == 1:
            warnings.append("singleton")
        if len(ids) < SMALL_GROUP_THRESHOLD:
            warnings.append("small_group")
        group_rows.append(
            {
                "category": category,
                "size": len(ids),
                "warnings": warnings,
                **category_metrics(ids),
            }
        )
    macro = {
        "table5a": {
            simulator: {
                metric: {
                    "category_count": len(group_rows),
                    "percentage": (
                        statistics.fmean(
                            group["table5a"][simulator][metric]["percentage"]
                            for group in group_rows
                        )
                        if group_rows
                        else None
                    ),
                }
                for metric in TABLE5A_METRICS
            }
            for simulator in SIMULATORS
        },
        "table5b": {
            "per_simulator_pass": {
                simulator: {
                    "category_count": len(group_rows),
                    "percentage": (
                        statistics.fmean(
                            group["table5b"]["per_simulator_pass"][simulator][
                                "percentage"
                            ]
                            for group in group_rows
                        )
                        if group_rows
                        else None
                    ),
                }
                for simulator in SIMULATORS
            },
            **{
                metric: {
                    "category_count": len(group_rows),
                    "percentage": (
                        statistics.fmean(
                            group["table5b"][metric]["percentage"]
                            for group in group_rows
                        )
                        if group_rows
                        else None
                    ),
                }
                for metric in TABLE5B_RATE_METRICS
            },
        },
    }
    return {
        "headline": "micro",
        "small_group_threshold": SMALL_GROUP_THRESHOLD,
        "category_count": len(group_rows),
        "micro": micro,
        "macro": macro,
        "groups": group_rows,
    }


def _category_macro_reconciles(categories: dict[str, Any]) -> bool:
    groups = categories["groups"]
    count = len(groups)
    for simulator in SIMULATORS:
        for metric in TABLE5A_METRICS:
            expected = (
                statistics.fmean(
                    group["table5a"][simulator][metric]["percentage"]
                    for group in groups
                )
                if groups
                else None
            )
            if categories["macro"]["table5a"][simulator][metric] != {
                "category_count": count,
                "percentage": expected,
            }:
                return False
        expected = (
            statistics.fmean(
                group["table5b"]["per_simulator_pass"][simulator]["percentage"]
                for group in groups
            )
            if groups
            else None
        )
        if categories["macro"]["table5b"]["per_simulator_pass"][simulator] != {
            "category_count": count,
            "percentage": expected,
        }:
            return False
    for metric in TABLE5B_RATE_METRICS:
        expected = (
            statistics.fmean(group["table5b"][metric]["percentage"] for group in groups)
            if groups
            else None
        )
        if categories["macro"]["table5b"][metric] != {
            "category_count": count,
            "percentage": expected,
        }:
            return False
    return True


def _category_metadata(categories: dict[str, Any]) -> dict[str, Any]:
    return {
        "small_group_threshold": categories["small_group_threshold"],
        "category_count": categories["category_count"],
        "groups": [
            {
                "category": group["category"],
                "size": group["size"],
                "warnings": copy.deepcopy(group["warnings"]),
            }
            for group in categories["groups"]
        ],
    }


def _category_metadata_reconciles(categories: dict[str, Any]) -> bool:
    if categories.get("small_group_threshold") != SMALL_GROUP_THRESHOLD:
        return False
    groups = categories.get("groups")
    if not isinstance(groups, list) or categories.get("category_count") != len(groups):
        return False
    for group in groups:
        size = group.get("size")
        expected_warnings = []
        if size == 1:
            expected_warnings.append("singleton")
        if isinstance(size, int) and size < SMALL_GROUP_THRESHOLD:
            expected_warnings.append("small_group")
        if group.get("warnings") != expected_warnings:
            return False
    return True


def _supported_by_name(record: dict[str, Any]) -> dict[str, dict[str, Any]]:
    joints = record["support"]["joints"]
    return {
        entry["name"]: entry
        for entry in joints
        if isinstance(entry, dict) and isinstance(entry.get("name"), str)
    }


def _actuation_by_name(record: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {
        entry["joint_name"]: entry
        for entry in record["diagnostics"]["actuation"]
        if isinstance(entry, dict) and isinstance(entry.get("joint_name"), str)
    }


def _joint_exclusion_reason(
    record: dict[str, Any] | None, joint_name: str, joint_type: str
) -> str | None:
    if record is None:
        return "missing_record"
    if record["terminal_status"] != "completed":
        return f"terminal_{record['terminal_status']}"
    support = _supported_by_name(record).get(joint_name)
    if support is None:
        return "missing_support_entry"
    if support.get("type") != joint_type:
        return "joint_type_mismatch"
    if not support.get("eligible"):
        reason = support.get("unsupported_reason")
        return (
            reason
            if isinstance(reason, str) and reason
            else "unsupported_without_reason"
        )
    if not support.get("runtime_mapped"):
        return "eligible_not_runtime_mapped"
    return None


def _joint_diagnostics(
    protocol: dict[str, Any],
    row_by_id: dict[str, dict[str, Any]],
    indexed: dict[str, dict[str, dict[str, Any]]],
    intent_ids: list[str],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    steps = protocol["cross_simulator"]["joint_rmse"]["sample_steps"]
    results: dict[str, list[float]] = {"revolute": [], "prismatic": []}
    pair_counts = {"revolute": 0, "prismatic": 0}
    exclusions: list[dict[str, Any]] = []
    candidate_counts = {"revolute": 0, "prismatic": 0}
    evaluable_counts = {"revolute": 0, "prismatic": 0}
    threshold = float(
        protocol["cross_simulator"]["thresholds"]["normalized_joint_rmse"]
    )
    for dataset_id in intent_ids:
        row = row_by_id[dataset_id]
        joint_tree = row.get("joint_tree")
        if (
            joint_tree is None
            and row.get("preflight", {}).get("simulator_eligible") is False
        ):
            continue
        for joint in joint_tree["joints"]:
            joint_type = joint.get("type")
            if joint_type not in results:
                continue
            candidate_counts[joint_type] += 1
            traces: dict[str, list[float]] = {}
            for simulator in SIMULATORS:
                record = indexed[simulator].get(dataset_id)
                reason = _joint_exclusion_reason(record, joint["name"], joint_type)
                if reason is None:
                    diagnostic = _actuation_by_name(record).get(joint["name"])
                    if not isinstance(diagnostic, dict):
                        raise AggregateContractError(
                            f"eligible joint lacks mandatory diagnostic for {dataset_id}/{simulator}/{joint['name']}"
                        )
                    if diagnostic.get("joint_type") != joint_type:
                        raise AggregateContractError(
                            f"joint diagnostic type mismatch for {dataset_id}/{simulator}/{joint['name']}"
                        )
                    trajectory = diagnostic.get("trajectory")
                    if (
                        not isinstance(trajectory, dict)
                        or trajectory.get("sample_steps") != steps
                    ):
                        raise AggregateContractError(
                            f"eligible joint lacks mandatory trajectory for {dataset_id}/{simulator}/{joint['name']}"
                        )
                    values = trajectory.get("normalized_positions")
                    if (
                        not isinstance(values, list)
                        or len(values) != len(steps)
                        or not all(_finite_number(value) for value in values)
                    ):
                        raise AggregateContractError(
                            f"eligible joint has invalid or non-finite trajectory for {dataset_id}/{simulator}/{joint['name']}"
                        )
                    traces[simulator] = [float(value) for value in values]
                else:
                    exclusions.append(
                        {
                            **_inventory_identity(row),
                            "simulator": simulator,
                            "joint_name": joint["name"],
                            "joint_type": joint_type,
                            "reason": reason,
                        }
                    )
            pair_values: list[float] = []
            for left, right in SIMULATOR_PAIRS:
                if left not in traces or right not in traces:
                    continue
                rmse = math.sqrt(
                    statistics.fmean(
                        (a - b) ** 2 for a, b in zip(traces[left], traces[right])
                    )
                )
                pair_values.append(rmse)
                if rmse > threshold:
                    exclusions.append(
                        {
                            **_inventory_identity(row),
                            "joint_name": joint["name"],
                            "joint_type": joint_type,
                            "simulator_pair": [left, right],
                            "reason": "joint_rmse_over_threshold",
                            "rmse": rmse,
                            "threshold": threshold,
                        }
                    )
            if pair_values:
                evaluable_counts[joint_type] += 1
                pair_counts[joint_type] += len(pair_values)
                unit_max = max(pair_values)
                results[joint_type].append(unit_max)
                if unit_max > threshold:
                    exclusions.append(
                        {
                            **_inventory_identity(row),
                            "joint_name": joint["name"],
                            "joint_type": joint_type,
                            "reason": "joint_unit_max_over_threshold",
                            "max_rmse": unit_max,
                            "threshold": threshold,
                        }
                    )
            else:
                exclusions.append(
                    {
                        **_inventory_identity(row),
                        "joint_name": joint["name"],
                        "joint_type": joint_type,
                        "reason": "no_available_simulator_pair",
                    }
                )
    aggregate = {
        joint_type: {
            "candidate_units": candidate_counts[joint_type],
            "evaluable_units": evaluable_counts[joint_type],
            "available_pairs": pair_counts[joint_type],
            "excluded_units": candidate_counts[joint_type]
            - evaluable_counts[joint_type],
            "threshold": threshold,
            "units_within_threshold": sum(
                value <= threshold for value in results[joint_type]
            ),
            **_distribution(results[joint_type]),
        }
        for joint_type in ("revolute", "prismatic")
    }
    return aggregate, exclusions


def _pose_error(
    left: dict[str, Any], right: dict[str, Any], diagonal: float
) -> tuple[float, float]:
    left_translation, right_translation = left.get("translation"), right.get(
        "translation"
    )
    if not (
        isinstance(left_translation, list)
        and isinstance(right_translation, list)
        and len(left_translation) == len(right_translation) == 3
        and all(_finite_number(value) for value in left_translation + right_translation)
    ):
        raise AggregateContractError("pose translation is malformed or non-finite")
    translation = (
        math.sqrt(
            sum(
                (float(a) - float(b)) ** 2
                for a, b in zip(left_translation, right_translation)
            )
        )
        / diagonal
    )
    try:
        rotation = quaternion_angular_error(left.get("rotation"), right.get("rotation"))
    except RuntimeContractError as error:
        raise AggregateContractError(
            f"pose quaternion is malformed: {error}"
        ) from error
    return translation, rotation


def _descendant_names(row: dict[str, Any], child_name: str) -> list[str]:
    reachable = {child_name}
    while True:
        expanded = reachable | {
            joint["child"]
            for joint in row["joint_tree"]["joints"]
            if joint.get("parent") in reachable
        }
        if expanded == reachable:
            return sorted(reachable)
        reachable = expanded


def _pose_diagnostics(
    protocol: dict[str, Any],
    row_by_id: dict[str, dict[str, Any]],
    indexed: dict[str, dict[str, dict[str, Any]]],
    intent_ids: list[str],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    translation_maxima: list[float] = []
    rotation_maxima: list[float] = []
    available_pairs = 0
    candidate_units = 0
    evaluable_units = 0
    exclusions: list[dict[str, Any]] = []
    thresholds = protocol["cross_simulator"]["thresholds"]
    translation_threshold = float(thresholds["translation_over_bbox_diagonal"])
    rotation_threshold = float(thresholds["rotation_rad"])
    for dataset_id in intent_ids:
        row = row_by_id[dataset_id]
        joint_tree = row.get("joint_tree")
        if (
            joint_tree is None
            and row.get("preflight", {}).get("simulator_eligible") is False
        ):
            continue
        diagonal = float(row["bounding_box_diagonal"])
        for joint in joint_tree["joints"]:
            joint_type = joint.get("type")
            if joint_type not in {"revolute", "prismatic"}:
                continue
            descendants = _descendant_names(row, joint["child"])
            for link_name in descendants:
                candidate_units += 1
                poses: dict[str, dict[str, Any]] = {}
                for simulator in SIMULATORS:
                    record = indexed[simulator].get(dataset_id)
                    reason = _joint_exclusion_reason(record, joint["name"], joint_type)
                    if reason is None:
                        diagnostic = _actuation_by_name(record).get(joint["name"])
                        if not isinstance(diagnostic, dict):
                            raise AggregateContractError(
                                f"eligible joint lacks mandatory pose diagnostic for {dataset_id}/{simulator}/{joint['name']}"
                            )
                        pose_map = diagnostic.get("final_descendant_root_frame_poses")
                        missing = diagnostic.get("missing_descendant_link_names")
                        if not isinstance(pose_map, dict) or not isinstance(
                            missing, list
                        ):
                            raise AggregateContractError(
                                f"eligible joint has malformed pose surface for {dataset_id}/{simulator}/{joint['name']}"
                            )
                        if link_name in pose_map:
                            poses[simulator] = pose_map[link_name]
                        elif link_name in missing:
                            reason = "missing_descendant_link_pose"
                        else:
                            raise AggregateContractError(
                                f"eligible joint lacks mandatory pose or mapping-miss reason for "
                                f"{dataset_id}/{simulator}/{joint['name']}/{link_name}"
                            )
                    if reason is not None:
                        exclusions.append(
                            {
                                **_inventory_identity(row),
                                "simulator": simulator,
                                "joint_name": joint["name"],
                                "descendant_link": link_name,
                                "reason": reason,
                            }
                        )
                translation_pairs: list[float] = []
                rotation_pairs: list[float] = []
                for left, right in SIMULATOR_PAIRS:
                    if left not in poses or right not in poses:
                        continue
                    translation, rotation = _pose_error(
                        poses[left], poses[right], diagonal
                    )
                    translation_pairs.append(translation)
                    rotation_pairs.append(rotation)
                    exceeded = []
                    if translation > translation_threshold:
                        exceeded.append("translation_over_bbox_diagonal")
                    if rotation > rotation_threshold:
                        exceeded.append("rotation_rad")
                    if exceeded:
                        exclusions.append(
                            {
                                **_inventory_identity(row),
                                "joint_name": joint["name"],
                                "descendant_link": link_name,
                                "simulator_pair": [left, right],
                                "reason": "pose_pair_over_threshold",
                                "translation_over_bbox_diagonal": translation,
                                "rotation_rad": rotation,
                                "translation_threshold": translation_threshold,
                                "rotation_threshold": rotation_threshold,
                                "exceeded": exceeded,
                            }
                        )
                if translation_pairs:
                    evaluable_units += 1
                    available_pairs += len(translation_pairs)
                    translation_maximum = max(translation_pairs)
                    rotation_maximum = max(rotation_pairs)
                    translation_maxima.append(translation_maximum)
                    rotation_maxima.append(rotation_maximum)
                    exceeded = []
                    if translation_maximum > translation_threshold:
                        exceeded.append("translation_over_bbox_diagonal")
                    if rotation_maximum > rotation_threshold:
                        exceeded.append("rotation_rad")
                    if exceeded:
                        exclusions.append(
                            {
                                **_inventory_identity(row),
                                "joint_name": joint["name"],
                                "descendant_link": link_name,
                                "reason": "pose_unit_max_over_threshold",
                                "max_translation_over_bbox_diagonal": translation_maximum,
                                "max_rotation_rad": rotation_maximum,
                                "translation_threshold": translation_threshold,
                                "rotation_threshold": rotation_threshold,
                                "exceeded": exceeded,
                            }
                        )
                else:
                    exclusions.append(
                        {
                            **_inventory_identity(row),
                            "joint_name": joint["name"],
                            "descendant_link": link_name,
                            "reason": "no_available_simulator_pair",
                        }
                    )
    return {
        "evaluation_unit": ["asset", "tested_joint", "descendant_link"],
        "candidate_units": candidate_units,
        "evaluable_units": evaluable_units,
        "available_pairs": available_pairs,
        "excluded_units": candidate_units - evaluable_units,
        "translation_over_bbox_diagonal": {
            "threshold": translation_threshold,
            "units_within_threshold": sum(
                value <= translation_threshold for value in translation_maxima
            ),
            **_distribution(translation_maxima),
        },
        "rotation_rad": {
            "threshold": rotation_threshold,
            "units_within_threshold": sum(
                value <= rotation_threshold for value in rotation_maxima
            ),
            **_distribution(rotation_maxima),
        },
    }, exclusions


def _constraint_drift_inventory(
    protocol: dict[str, Any],
    row_by_id: dict[str, dict[str, Any]],
    indexed: dict[str, dict[str, dict[str, Any]]],
    intent_ids: list[str],
) -> list[dict[str, Any]]:
    translation_threshold = float(
        protocol["metrics"]["constraint_drift"]["translation_over_bbox_diagonal_max"]
    )
    rotation_threshold = float(
        protocol["metrics"]["constraint_drift"]["rotation_rad_max"]
    )
    inventory: list[dict[str, Any]] = []
    for simulator in SIMULATORS:
        for dataset_id in intent_ids:
            row = row_by_id[dataset_id]
            record = indexed[simulator].get(dataset_id)
            if record is None or record["terminal_status"] != "completed":
                continue
            for diagnostic in record["diagnostics"]["actuation"]:
                drift = diagnostic["constraint_drift"]
                if drift["finite"] is False:
                    inventory.append(
                        {
                            **_inventory_identity(row),
                            "simulator": simulator,
                            "joint_name": diagnostic["joint_name"],
                            "joint_type": diagnostic["joint_type"],
                            "reason": "constraint_drift_nonfinite",
                            "max_translation_over_bbox_diagonal": drift[
                                "max_translation_over_bbox_diagonal"
                            ],
                            "max_rotation_error_rad": drift["max_rotation_error_rad"],
                            "translation_threshold": translation_threshold,
                            "rotation_threshold": rotation_threshold,
                            "passed": drift["passed"],
                            "finite": False,
                            "threshold_evaluable": False,
                        }
                    )
                    continue
                translation = float(drift["max_translation_over_bbox_diagonal"])
                rotation = float(drift["max_rotation_error_rad"])
                exceeded = []
                if translation > translation_threshold:
                    exceeded.append("translation_over_bbox_diagonal")
                if rotation > rotation_threshold:
                    exceeded.append("rotation_rad")
                inventory.append(
                    {
                        **_inventory_identity(row),
                        "simulator": simulator,
                        "joint_name": diagnostic["joint_name"],
                        "joint_type": diagnostic["joint_type"],
                        "reason": "constraint_drift_worst_error",
                        "max_translation_over_bbox_diagonal": translation,
                        "max_rotation_error_rad": rotation,
                        "translation_threshold": translation_threshold,
                        "rotation_threshold": rotation_threshold,
                        "passed": drift["passed"],
                        "exceeded": exceeded,
                    }
                )
    return inventory


def aggregate_records(
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    records_by_simulator: dict[str, list[dict[str, Any]]],
    *,
    phase: str,
    intent_ids: list[str] | None = None,
    receipt_info: dict[str, Any] | None = None,
    _verify_bundle: bool = True,
) -> dict[str, Any]:
    if phase not in {"formal", "qualification"}:
        raise AggregateContractError("phase must be formal or qualification")
    row_by_id = _validate_protocol_manifest(protocol, manifest)
    authoritative_receipt = _authoritative_receipt(manifest, receipt_info)
    _validate_receipt_provenance(
        protocol,
        manifest,
        receipt_info,
        required=False,
    )
    selected_ids = list(row_by_id)
    if phase == "formal":
        if intent_ids is not None:
            raise AggregateContractError(
                "formal phase cannot accept explicit intent IDs"
            )
        if len(selected_ids) != 800:
            raise AggregateContractError(
                "formal Artiverse Table 5 requires the full N=800 cohort"
            )
        intent_ids = selected_ids
        report_kind = "formal"
    else:
        if not intent_ids:
            raise AggregateContractError(
                "qualification phase requires explicit unique intent IDs"
            )
        if len(intent_ids) != len(set(intent_ids)) or not set(intent_ids).issubset(
            row_by_id
        ):
            raise AggregateContractError(
                "qualification intent IDs must be unique selected manifest IDs"
            )
        intent_ids = [
            dataset_id for dataset_id in selected_ids if dataset_id in set(intent_ids)
        ]
        report_kind = "non_formal"
    indexed, record_inventory, coverage = _index_records(
        protocol, manifest, row_by_id, records_by_simulator, phase, intent_ids
    )
    record_coverage_complete = all(
        coverage[simulator]["missing_count"] == 0 for simulator in SIMULATORS
    )
    runtime_receipts_complete = _validate_runtime_receipt_info(
        receipt_info,
        records_by_simulator,
        phase=phase,
        intent_count=len(intent_ids),
        required=False,
    )
    complete = record_coverage_complete and runtime_receipts_complete
    strict_outcomes, strict_inventory = _strict_asset_outcomes(
        protocol, row_by_id, indexed, intent_ids
    )
    table5a, table5b = _metric_tables(indexed, intent_ids, strict_outcomes)
    joint_rmse, joint_inventory = _joint_diagnostics(
        protocol, row_by_id, indexed, intent_ids
    )
    poses, pose_inventory = _pose_diagnostics(protocol, row_by_id, indexed, intent_ids)
    drift_inventory = _constraint_drift_inventory(
        protocol, row_by_id, indexed, intent_ids
    )
    table5b["joint_rmse"] = joint_rmse
    table5b["link_pose_error"] = poses
    asset_outcomes = [
        {
            **_inventory_identity(row_by_id[dataset_id]),
            **strict_outcomes[dataset_id],
        }
        for dataset_id in intent_ids
    ]
    table5 = {
        "schema_version": AGGREGATE_SCHEMA,
        "method": "Artiverse Table 1 cohort contact-enabled cross-simulator readiness",
        "dataset": "Artiverse",
        "run_phase": phase,
        "report_kind": report_kind,
        "state": "complete" if complete else "incomplete",
        "formal_claim_complete": phase == "formal" and complete,
        "intent": {
            "count": len(intent_ids),
            "dataset_ids": intent_ids,
            "manifest_roots": [
                row_by_id[dataset_id]["manifest_root"] for dataset_id in intent_ids
            ],
        },
        "receipt": authoritative_receipt,
        "claim_boundary": {
            "contact_enabled_runtime_diagnostic": True,
            "upstream_gates_joined_by_manifest_root": True,
            "strict_consistency_requires_all_three_simulators": True,
            "strict_sim_ready_formula": (
                "strict_urdf AND strict_kinematic AND strict_collision AND "
                "all_three_runtime_pass AND strict_consistency"
            ),
        },
        "table5a": table5a,
        "table5b": table5b,
        "asset_outcomes": asset_outcomes,
        "categories": _category_report(row_by_id, indexed, intent_ids, strict_outcomes),
    }
    inventory = {
        "schema_version": INVENTORY_SCHEMA,
        "run_phase": phase,
        "records": record_inventory,
        "joint_diagnostics": joint_inventory,
        "pose_diagnostics": pose_inventory,
        "constraint_drift": drift_inventory,
        "strict_consistency": strict_inventory,
    }
    checks = {
        "phase_separation": True,
        "intent_unique_selected": len(intent_ids) == len(set(intent_ids))
        and set(intent_ids).issubset(row_by_id),
        "formal_full_cohort": phase != "formal"
        or (intent_ids == selected_ids and len(intent_ids) == 800),
        "full_n_denominators": all(
            table5a[simulator][metric]["denominator"] == len(intent_ids)
            for simulator in SIMULATORS
            for metric in TABLE5A_METRICS
        ),
        "all_three_denominators": all(
            table5b[metric]["denominator"] == len(intent_ids)
            for metric in TABLE5B_RATE_METRICS
        ),
        "metric_numerators_recomputed": all(
            table5a[simulator][metric]["passed"]
            == sum(
                indexed[simulator].get(dataset_id, {}).get("metrics", {}).get(metric)
                is True
                for dataset_id in intent_ids
            )
            for simulator in SIMULATORS
            for metric in STRICT_METRICS
        ),
        "intersections_recomputed": table5b["all_three_load"]["passed"]
        == sum(
            all(
                indexed[simulator].get(dataset_id, {}).get("metrics", {}).get("load")
                is True
                for simulator in SIMULATORS
            )
            for dataset_id in intent_ids
        )
        and table5b["all_three_runtime_pass"]["passed"]
        == sum(
            all(
                indexed[simulator]
                .get(dataset_id, {})
                .get("metrics", {})
                .get("simulator_pass")
                is True
                for simulator in SIMULATORS
            )
            for dataset_id in intent_ids
        ),
        "strict_rates_recomputed": all(
            table5b[metric]["passed"]
            == sum(strict_outcomes[dataset_id][metric] for dataset_id in intent_ids)
            for metric in (
                "all_three_runtime_pass",
                "strict_urdf_pass",
                "strict_kinematic_pass",
                "strict_collision_pass",
                "strict_consistency",
                "strict_sim_ready",
            )
        )
        and all(
            table5a[simulator]["strict_collision_pass"]
            == table5b["strict_collision_pass"]
            for simulator in SIMULATORS
        ),
        "strict_formula_recomputed": all(
            strict_outcomes[dataset_id]["strict_sim_ready"]
            == all(
                strict_outcomes[dataset_id][field]
                for field in (
                    "strict_urdf_pass",
                    "strict_kinematic_pass",
                    "strict_collision_pass",
                    "all_three_runtime_pass",
                    "strict_consistency",
                )
            )
            for dataset_id in intent_ids
        ),
        "asset_outcomes_recomputed": table5["asset_outcomes"]
        == [
            {
                **_inventory_identity(row_by_id[dataset_id]),
                **strict_outcomes[dataset_id],
            }
            for dataset_id in intent_ids
        ],
        "category_reconciliation": sum(
            group["size"] for group in table5["categories"]["groups"]
        )
        == len(intent_ids),
        "category_micro_reconciles": all(
            table5["categories"]["micro"]["table5a"][simulator][metric]
            == table5a[simulator][metric]
            for simulator in SIMULATORS
            for metric in TABLE5A_METRICS
        )
        and all(
            table5["categories"]["micro"]["table5b"][metric] == table5b[metric]
            for metric in ("per_simulator_pass", *TABLE5B_RATE_METRICS)
        ),
        "category_macro_reconciles": _category_macro_reconciles(table5["categories"]),
        "category_metadata_reconciles": _category_metadata_reconciles(
            table5["categories"]
        ),
        "inventory_reasons_complete": all(
            isinstance(entry, dict)
            and isinstance(entry.get("reason"), str)
            and bool(entry["reason"])
            for section in (
                "records",
                "joint_diagnostics",
                "pose_diagnostics",
                "constraint_drift",
                "strict_consistency",
            )
            for entry in inventory[section]
        ),
        "finite_json": True,
    }
    self_check = {
        "schema_version": SELF_CHECK_SCHEMA,
        "run_phase": phase,
        "coverage": coverage,
        "completion_rule": (
            "every intent asset has exactly one valid terminal record in every simulator and "
            "the exact runtime summary/progress receipts are complete"
        ),
        "completion": {
            "state": table5["state"],
            "formal_claim_complete": table5["formal_claim_complete"],
        },
        "input_receipt": copy.deepcopy(table5["receipt"]),
        "selected_count": len(intent_ids),
        "category_accounting": _category_metadata(table5["categories"]),
        "checks": checks,
        "diagnostic_counts": {
            "joint_rmse": {
                joint_type: {
                    key: joint_rmse[joint_type][key]
                    for key in (
                        "candidate_units",
                        "evaluable_units",
                        "available_pairs",
                        "excluded_units",
                    )
                }
                for joint_type in ("revolute", "prismatic")
            },
            "link_pose": {
                key: poses[key]
                for key in (
                    "candidate_units",
                    "evaluable_units",
                    "available_pairs",
                    "excluded_units",
                )
            },
            "joint_exclusion_entries": len(joint_inventory),
            "pose_exclusion_entries": len(pose_inventory),
            "constraint_drift_entries": len(drift_inventory),
            "strict_consistency_entries": len(strict_inventory),
        },
        "passed": all(checks.values()),
    }
    bundle = {
        "table5": table5,
        "failure_inventory": inventory,
        "self_check": self_check,
    }
    _json_finite(bundle, "aggregate bundle")
    if _verify_bundle:
        validate_aggregate_bundle(
            bundle,
            protocol,
            manifest,
            records_by_simulator,
            phase=phase,
            intent_ids=intent_ids if phase == "qualification" else None,
            receipt_info=receipt_info,
        )
    return bundle


def validate_aggregate_bundle(
    bundle: dict[str, Any],
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    records_by_simulator: dict[str, list[dict[str, Any]]],
    *,
    phase: str,
    intent_ids: list[str] | None = None,
    receipt_info: dict[str, Any] | None = None,
) -> None:
    if not isinstance(bundle, dict) or set(bundle) != {
        "table5",
        "failure_inventory",
        "self_check",
    }:
        raise AggregateContractError("aggregate self-check bundle shape mismatch")
    # Recompute independently through the same pure pipeline while suppressing recursive validation.
    expected = _aggregate_without_bundle_validation(
        protocol,
        manifest,
        records_by_simulator,
        phase=phase,
        intent_ids=intent_ids,
        receipt_info=receipt_info,
    )
    if bundle != expected:
        raise AggregateContractError("aggregate self-check recomputation mismatch")
    _json_finite(bundle, "validated aggregate bundle")


def _aggregate_without_bundle_validation(
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    records_by_simulator: dict[str, list[dict[str, Any]]],
    *,
    phase: str,
    intent_ids: list[str] | None,
    receipt_info: dict[str, Any] | None,
) -> dict[str, Any]:
    return aggregate_records(
        protocol,
        manifest,
        records_by_simulator,
        phase=phase,
        intent_ids=intent_ids,
        receipt_info=receipt_info,
        _verify_bundle=False,
    )


def _markdown(bundle: dict[str, Any]) -> str:
    table5 = bundle["table5"]
    label = "complete formal" if table5["formal_claim_complete"] else table5["state"]
    lines = [
        "# Artiverse Table 5",
        "",
        f"- Phase: `{table5['run_phase']}` ({table5['report_kind']})",
        f"- State: `{label}`",
        f"- full-N denominator: `{table5['intent']['count']}`",
        "- Scope: the exact ordered Artiverse Table 1 N=800 cohort with contact-enabled simulator adapters.",
        "- Strict Sim-ready joins frozen Table 2/3/4 gates by exact manifest_root and requires all-three runtime plus strict consistency.",
        "- Cross-simulator agreement is simulator evidence, not real-world dynamics validation.",
        "",
        "## Table 5a",
        "",
        "| Simulator | Load | Reset | Settling | Actuation | Limits | Drift pass | Simulator pass | Strict collision |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    labels = {
        "load": "Load",
        "reset": "Reset",
        "settling": "Settling",
        "actuation": "Actuation",
        "limit_enforcement": "Limits",
        "constraint_drift": "Drift",
        "simulator_pass": "Pass",
    }
    del labels
    for simulator in SIMULATORS:
        values = table5["table5a"][simulator]
        cells = [
            f"{values[metric]['passed']}/{values[metric]['denominator']} ({values[metric]['percentage']:.3f}%)"
            for metric in STRICT_METRICS
        ]
        strict_collision = values["strict_collision_pass"]
        cells.append(
            f"{strict_collision['passed']}/{strict_collision['denominator']} "
            f"({strict_collision['percentage']:.3f}%)"
        )
        lines.append(f"| {simulator} | " + " | ".join(cells) + " |")
    table5b = table5["table5b"]
    lines.extend(
        [
            "",
            "## Table 5b",
            "",
            f"All-three Load: {table5b['all_three_load']['passed']}/{table5b['all_three_load']['denominator']} "
            f"({table5b['all_three_load']['percentage']:.3f}%).",
            "",
            f"All-three Runtime Pass: {table5b['all_three_runtime_pass']['passed']}/"
            f"{table5b['all_three_runtime_pass']['denominator']} "
            f"({table5b['all_three_runtime_pass']['percentage']:.3f}%).",
            "",
            f"Upstream Strict URDF: {table5b['strict_urdf_pass']['passed']}/"
            f"{table5b['strict_urdf_pass']['denominator']} "
            f"({table5b['strict_urdf_pass']['percentage']:.3f}%).",
            "",
            f"Upstream Strict Kinematic: {table5b['strict_kinematic_pass']['passed']}/"
            f"{table5b['strict_kinematic_pass']['denominator']} "
            f"({table5b['strict_kinematic_pass']['percentage']:.3f}%).",
            "",
            f"Upstream Strict Collision: {table5b['strict_collision_pass']['passed']}/"
            f"{table5b['strict_collision_pass']['denominator']} "
            f"({table5b['strict_collision_pass']['percentage']:.3f}%).",
            "",
            f"Strict consistency: {table5b['strict_consistency']['passed']}/"
            f"{table5b['strict_consistency']['denominator']} "
            f"({table5b['strict_consistency']['percentage']:.3f}%).",
            "",
            f"Strict Sim-ready: {table5b['strict_sim_ready']['passed']}/"
            f"{table5b['strict_sim_ready']['denominator']} "
            f"({table5b['strict_sim_ready']['percentage']:.3f}%).",
            "",
            "Diagnostic evaluable denominators are separate from the full-N strict rates:",
            "",
            f"- Revolute joint RMSE: {table5b['joint_rmse']['revolute']['evaluable_units']} evaluable units; "
            f"population max {table5b['joint_rmse']['revolute']['population_max']}.",
            f"- Prismatic joint RMSE: {table5b['joint_rmse']['prismatic']['evaluable_units']} evaluable units; "
            f"population max {table5b['joint_rmse']['prismatic']['population_max']}.",
            f"- Link-pose error: {table5b['link_pose_error']['evaluable_units']} diagnostic evaluable units.",
            "",
            "## Categories",
            "",
            "The micro headline uses all intent assets. The macro supplement is the unweighted mean over exact raw category strings.",
            "",
            f"Small-group warning threshold: < {table5['categories']['small_group_threshold']} assets.",
            "",
        ]
    )
    if table5["state"] == "incomplete":
        lines.extend(
            [
                "> INCOMPLETE: this report preserves intent denominators but is not a completed formal Table 5 claim.",
                "",
            ]
        )
    return "\n".join(lines)


def _atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


@contextmanager
def _publication_lock(output_root: Path) -> Iterable[None]:
    output_root.parent.mkdir(parents=True, exist_ok=True)
    lock_path = output_root.parent / f".{output_root.name}.publish.lock"
    descriptor = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise AggregateContractError(
                f"aggregate output is already locked: {output_root}"
            ) from error
        yield
    finally:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)


def _link_directory_noreplace(source: Path, destination: Path) -> None:
    artifact_names = (
        "table5.json",
        "failure_inventory.json",
        "report.md",
        "self_check.json",
        "aggregate_set.json",
    )
    expected = set(artifact_names)
    actual = {entry.name for entry in source.iterdir()}
    if actual != expected:
        raise AggregateContractError(
            "aggregate staging directory has unexpected artifacts"
        )
    try:
        destination.mkdir(mode=0o700)
    except FileExistsError as error:
        raise AggregateContractError(
            f"refusing to replace existing aggregate output: {destination}"
        ) from error
    destination_stat = destination.stat(follow_symlinks=False)
    try:
        for name in artifact_names[:-1]:
            staged = source / name
            if staged.is_symlink() or not stat.S_ISREG(
                staged.stat(follow_symlinks=False).st_mode
            ):
                raise AggregateContractError(
                    f"aggregate staging artifact is not regular: {name}"
                )
            os.link(staged, destination / name, follow_symlinks=False)
        os.chmod(destination, 0o755)
        marker = source / artifact_names[-1]
        if marker.is_symlink() or not stat.S_ISREG(
            marker.stat(follow_symlinks=False).st_mode
        ):
            raise AggregateContractError("aggregate publication marker is not regular")
        os.link(marker, destination / marker.name, follow_symlinks=False)
    except BaseException:
        try:
            current = destination.stat(follow_symlinks=False)
            owns_destination = (
                stat.S_ISDIR(current.st_mode)
                and current.st_dev == destination_stat.st_dev
                and current.st_ino == destination_stat.st_ino
            )
            if owns_destination:
                for name in artifact_names:
                    staged = source / name
                    published = destination / name
                    try:
                        staged_stat = staged.stat(follow_symlinks=False)
                        published_stat = published.stat(follow_symlinks=False)
                    except FileNotFoundError:
                        continue
                    if (
                        stat.S_ISREG(staged_stat.st_mode)
                        and stat.S_ISREG(published_stat.st_mode)
                        and staged_stat.st_dev == published_stat.st_dev
                        and staged_stat.st_ino == published_stat.st_ino
                    ):
                        published.unlink()
                try:
                    destination.rmdir()
                except OSError:
                    pass
        finally:
            raise


def _rename_directory_noreplace(source: Path, destination: Path) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        _link_directory_noreplace(source, destination)
        return
    renameat2.argtypes = [
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    ]
    renameat2.restype = ctypes.c_int
    result = renameat2(
        -100,
        os.fsencode(source),
        -100,
        os.fsencode(destination),
        1,
    )
    if result == 0:
        return
    error_number = ctypes.get_errno()
    if error_number in {errno.EEXIST, errno.ENOTEMPTY}:
        raise AggregateContractError(
            f"refusing to replace existing aggregate output: {destination}"
        )
    if error_number in {errno.ENOSYS, errno.EINVAL, errno.ENOTSUP, errno.EOPNOTSUPP}:
        _link_directory_noreplace(source, destination)
        return
    raise OSError(error_number, os.strerror(error_number), str(destination))


def _published_self_check(bundle: dict[str, Any], stage_root: Path) -> dict[str, Any]:
    self_check = copy.deepcopy(bundle["self_check"])
    self_check["publication_schema_version"] = PUBLICATION_SCHEMA
    self_check["output_hashes"] = {
        name: sha256_file(stage_root / name)
        for name in ("table5.json", "failure_inventory.json", "report.md")
    }
    return self_check


def _publication_marker(stage_root: Path, bundle: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": PUBLICATION_SCHEMA,
        "run_phase": bundle["table5"]["run_phase"],
        "protocol_sha256": bundle["table5"]["receipt"]["protocol_sha256"],
        "cohort_sha256": bundle["table5"]["receipt"]["cohort_sha256"],
        "file_hashes": {
            name: sha256_file(stage_root / name)
            for name in (
                "table5.json",
                "failure_inventory.json",
                "report.md",
                "self_check.json",
            )
        },
    }


def _validate_publication_claim_state(
    table5: Any,
    *,
    phase: str,
) -> None:
    if not isinstance(table5, dict) or table5.get("run_phase") != phase:
        raise AggregateContractError("publication phase/aggregate binding mismatch")
    if table5.get("state") != "complete":
        raise AggregateContractError("refusing to publish an incomplete aggregate")
    if phase == "formal":
        if (
            table5.get("report_kind") != "formal"
            or table5.get("formal_claim_complete") is not True
        ):
            raise AggregateContractError(
                "formal publication requires a complete formal claim"
            )
    elif phase == "qualification":
        if (
            table5.get("report_kind") != "non_formal"
            or table5.get("formal_claim_complete") is not False
        ):
            raise AggregateContractError(
                "qualification publication must remain complete and nonformal"
            )
    else:
        raise AggregateContractError(
            "publication phase must be formal or qualification"
        )


def publish_aggregate(
    bundle: dict[str, Any],
    output_root: Path,
    *,
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    records_by_simulator: dict[str, list[dict[str, Any]]],
    phase: str,
    intent_ids: list[str] | None = None,
    receipt_info: dict[str, Any] | None = None,
) -> None:
    if bundle.get("self_check", {}).get("passed") is not True:
        raise AggregateContractError(
            "refusing to publish a failing aggregate self-check"
        )
    _validate_publication_claim_state(bundle.get("table5"), phase=phase)
    intent_count = len(manifest["rows"]) if phase == "formal" else len(intent_ids or [])
    _validate_receipt_provenance(
        protocol,
        manifest,
        receipt_info,
        required=True,
    )
    assert receipt_info is not None
    unresolved_root = Path(output_root)
    if unresolved_root.is_symlink():
        raise AggregateContractError(
            "refusing to publish through an aggregate output symlink"
        )
    output_root = _validate_publication_location(
        unresolved_root, receipt_info, phase=phase
    )
    _validate_source_authority(
        protocol,
        manifest,
        receipt_info,
        required=True,
    )
    _validate_runtime_receipt_info(
        receipt_info,
        records_by_simulator,
        phase=phase,
        intent_count=intent_count,
        required=True,
    )
    validate_aggregate_bundle(
        bundle,
        protocol,
        manifest,
        records_by_simulator,
        phase=phase,
        intent_ids=intent_ids,
        receipt_info=receipt_info,
    )
    with _publication_lock(output_root):
        if output_root.exists():
            raise AggregateContractError(
                f"refusing to replace existing aggregate output: {output_root}"
            )
        stage_root = Path(
            tempfile.mkdtemp(
                prefix=f".{output_root.name}.stage-", dir=output_root.parent
            )
        )
        committed = False
        try:
            atomic_write_json(stage_root / "table5.json", bundle["table5"])
            atomic_write_json(
                stage_root / "failure_inventory.json", bundle["failure_inventory"]
            )
            _atomic_write_text(stage_root / "report.md", _markdown(bundle))
            atomic_write_json(
                stage_root / "self_check.json",
                _published_self_check(bundle, stage_root),
            )
            atomic_write_json(
                stage_root / "aggregate_set.json",
                _publication_marker(stage_root, bundle),
            )
            validate_published_outputs(
                stage_root,
                protocol=protocol,
                manifest=manifest,
                records_by_simulator=records_by_simulator,
                phase=phase,
                intent_ids=intent_ids,
                receipt_info=receipt_info,
                _source_prevalidated=True,
            )
            if output_root.exists():
                raise AggregateContractError(
                    f"refusing to replace existing aggregate output: {output_root}"
                )
            _rename_directory_noreplace(stage_root, output_root)
            committed = True
        finally:
            if stage_root.exists():
                if committed:
                    try:
                        shutil.rmtree(stage_root)
                    except BaseException:
                        pass
                else:
                    shutil.rmtree(stage_root)


def _load_published_json(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise AggregateContractError(
            f"published artifact must be a regular file: {path.name}"
        )
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise AggregateContractError(
            f"published JSON is malformed: {path.name}: {error}"
        ) from error
    if not isinstance(value, dict):
        raise AggregateContractError(f"published JSON must be an object: {path.name}")
    _json_finite(value, path.name)
    return value


def validate_published_outputs(
    output_root: Path,
    *,
    protocol: dict[str, Any],
    manifest: dict[str, Any],
    records_by_simulator: dict[str, list[dict[str, Any]]],
    phase: str,
    intent_ids: list[str] | None = None,
    receipt_info: dict[str, Any] | None = None,
    _source_prevalidated: bool = False,
) -> None:
    intent_count = len(manifest["rows"]) if phase == "formal" else len(intent_ids or [])
    _validate_receipt_provenance(
        protocol,
        manifest,
        receipt_info,
        required=True,
    )
    if not _source_prevalidated:
        assert receipt_info is not None
        _validate_publication_location(output_root, receipt_info, phase=phase)
        _validate_source_authority(
            protocol,
            manifest,
            receipt_info,
            required=True,
        )
    _validate_runtime_receipt_info(
        receipt_info,
        records_by_simulator,
        phase=phase,
        intent_count=intent_count,
        required=True,
    )
    unresolved_root = Path(output_root)
    if unresolved_root.is_symlink():
        raise AggregateContractError("published aggregate root cannot be a symlink")
    output_root = unresolved_root.resolve()
    if not output_root.is_dir():
        raise AggregateContractError("published aggregate root is not a directory")
    expected = {
        "aggregate_set.json",
        "table5.json",
        "failure_inventory.json",
        "report.md",
        "self_check.json",
    }
    files = {path.name for path in output_root.iterdir()}
    if files != expected:
        raise AggregateContractError("published aggregate file set mismatch")
    for name in sorted(expected):
        _require_regular_file(output_root / name, f"published artifact {name}")
    marker = _load_published_json(output_root / "aggregate_set.json")
    table5 = _load_published_json(output_root / "table5.json")
    inventory = _load_published_json(output_root / "failure_inventory.json")
    self_check = _load_published_json(output_root / "self_check.json")
    if (
        self_check.get("publication_schema_version") != PUBLICATION_SCHEMA
        or self_check.get("passed") is not True
    ):
        raise AggregateContractError("published aggregate lacks passing self-check")
    hashes = self_check.get("output_hashes")
    expected_self_hash_keys = {"table5.json", "failure_inventory.json", "report.md"}
    if not isinstance(hashes, dict) or set(hashes) != expected_self_hash_keys:
        raise AggregateContractError("published aggregate lacks output hashes")
    for name in sorted(expected_self_hash_keys):
        if hashes.get(name) != sha256_file(output_root / name):
            raise AggregateContractError(f"published output hash mismatch: {name}")
    expected_marker = {
        "schema_version": PUBLICATION_SCHEMA,
        "run_phase": phase,
        "protocol_sha256": manifest["protocol_sha256"],
        "cohort_sha256": manifest["cohort_sha256"],
        "file_hashes": {
            name: sha256_file(output_root / name)
            for name in (
                "table5.json",
                "failure_inventory.json",
                "report.md",
                "self_check.json",
            )
        },
    }
    if marker != expected_marker:
        raise AggregateContractError("published aggregate marker/hash binding mismatch")
    _validate_publication_claim_state(table5, phase=phase)
    logical_self_check = copy.deepcopy(self_check)
    del logical_self_check["publication_schema_version"]
    del logical_self_check["output_hashes"]
    logical_bundle = {
        "table5": table5,
        "failure_inventory": inventory,
        "self_check": logical_self_check,
    }
    validate_aggregate_bundle(
        logical_bundle,
        protocol,
        manifest,
        records_by_simulator,
        phase=phase,
        intent_ids=intent_ids,
        receipt_info=receipt_info,
    )
    if (output_root / "report.md").is_symlink() or not (
        output_root / "report.md"
    ).is_file():
        raise AggregateContractError("published report must be a regular file")
    if (output_root / "report.md").read_text(encoding="utf-8") != _markdown(
        logical_bundle
    ):
        raise AggregateContractError(
            "published report does not match authoritative aggregate"
        )


def _require_regular_directory(path: Path, label: str) -> None:
    try:
        mode = path.stat(follow_symlinks=False).st_mode
    except FileNotFoundError as error:
        raise AggregateContractError(f"required {label} is missing: {path}") from error
    if path.is_symlink() or not stat.S_ISDIR(mode):
        raise AggregateContractError(
            f"{label} must be a regular directory, not a symlink: {path}"
        )


def _require_regular_file(path: Path, label: str) -> None:
    try:
        mode = path.stat(follow_symlinks=False).st_mode
    except FileNotFoundError as error:
        raise AggregateContractError(f"required {label} is missing: {path}") from error
    if path.is_symlink() or not stat.S_ISREG(mode):
        raise AggregateContractError(
            f"{label} must be a regular file, not a symlink: {path}"
        )


def _read_runtime_records(
    receipt_root: Path, phase: str
) -> tuple[dict[str, list[dict[str, Any]]], dict[str, dict[str, Any]]]:
    records: dict[str, list[dict[str, Any]]] = {}
    runtime_inputs: dict[str, dict[str, Any]] = {}
    phase_root = receipt_root / phase
    _require_regular_directory(phase_root, "runtime phase root")
    for simulator in SIMULATORS:
        simulator_root = phase_root / simulator
        assets_root = simulator_root / "assets"
        _require_regular_directory(simulator_root, f"{simulator} runtime root")
        _require_regular_directory(assets_root, f"{simulator} assets root")
        unexpected = [
            path
            for path in simulator_root.iterdir()
            if path.name
            not in {
                "assets",
                "progress.json",
                "summary.json",
                "worker_logs",
                ".worker_requests",
                ".worker_responses",
                ".prepare.lock",
            }
        ]
        if unexpected:
            raise AggregateContractError(
                f"unexpected runtime artifact in {simulator_root}: {unexpected[0].name}"
            )
        entries: list[dict[str, Any]] = []
        record_hashes: list[dict[str, str]] = []
        resolved_assets = assets_root.resolve()
        for path in sorted(assets_root.iterdir()):
            if path.suffix != ".json" or not (
                len(path.stem) == len("artiverse_0000")
                and path.stem.startswith("artiverse_")
                and path.stem.removeprefix("artiverse_").isdigit()
            ):
                raise AggregateContractError(f"unexpected runtime record path: {path}")
            canonical_name = f"{path.stem}.json"
            if path.name != canonical_name:
                raise AggregateContractError(
                    f"runtime record filename is not canonical: {path.name}"
                )
            _require_regular_file(path, "runtime record")
            if path.resolve().parent != resolved_assets:
                raise AggregateContractError(
                    f"runtime record resolves outside assets root: {path}"
                )
            try:
                record = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as error:
                raise AggregateContractError(
                    f"malformed runtime record {path}: {error}"
                ) from error
            if not isinstance(record, dict):
                raise AggregateContractError(
                    f"runtime record must be a JSON object: {path}"
                )
            if record.get("identity", {}).get("dataset_id") != path.stem:
                raise AggregateContractError(
                    f"runtime record filename does not match dataset ID: {path}"
                )
            entries.append(record)
            record_hashes.append({"filename": path.name, "sha256": sha256_file(path)})
        record_hashes.sort(key=lambda entry: Path(entry["filename"]).stem)
        records[simulator] = entries
        implementation_hashes = {
            record.get("identity", {}).get("adapter_implementation_sha256")
            for record in entries
        }
        if None in implementation_hashes or len(implementation_hashes) > 1:
            raise AggregateContractError(
                f"invalid or mixed adapter implementation hashes in {simulator}"
            )
        runtime_inputs[simulator] = {
            "record_file_hashes": {
                entry["filename"]: entry["sha256"] for entry in record_hashes
            },
            "record_set_sha256": canonical_sha256(record_hashes),
            "adapter_implementation_sha256": (
                next(iter(implementation_hashes)) if implementation_hashes else None
            ),
        }
    return records, runtime_inputs


def _validate_runtime_summaries(
    receipt_root: Path,
    phase: str,
    records: dict[str, list[dict[str, Any]]],
    intent_count: int,
    runtime_inputs: dict[str, dict[str, Any]],
) -> None:
    for simulator in SIMULATORS:
        simulator_root = receipt_root / phase / simulator
        summary_path = simulator_root / "summary.json"
        progress_path = simulator_root / "progress.json"
        _require_regular_file(summary_path, f"{simulator} runtime summary")
        _require_regular_file(progress_path, f"{simulator} runtime progress")
        documents: dict[str, dict[str, Any]] = {}
        for label, path in (("summary", summary_path), ("progress", progress_path)):
            try:
                value = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as error:
                raise AggregateContractError(
                    f"runtime {label} is malformed for {simulator}: {error}"
                ) from error
            if not isinstance(value, dict):
                raise AggregateContractError(
                    f"runtime {label} must be an object for {simulator}"
                )
            documents[label] = value
        status_counts = dict(
            sorted(
                Counter(
                    record["terminal_status"] for record in records[simulator]
                ).items()
            )
        )
        metric_counts = {
            metric: sum(
                record["metrics"][metric] is True for record in records[simulator]
            )
            for metric in STRICT_METRICS
        }
        expected_summary = {
            "schema_version": "table5_artiverse_runtime_summary_v1",
            "run_phase": phase,
            "simulator": simulator,
            "effective_workers": documents["summary"].get("effective_workers"),
            "intent_count": intent_count,
            "terminal_count": len(records[simulator]),
            "remaining_count": intent_count - len(records[simulator]),
            "complete": len(records[simulator]) == intent_count,
            "terminal_status_counts": status_counts,
            "metric_pass_counts": metric_counts,
            "metric_denominator": intent_count,
        }
        expected_progress = copy.deepcopy(expected_summary)
        expected_progress["schema_version"] = "table5_artiverse_runtime_progress_v1"
        effective_workers = expected_summary["effective_workers"]
        if (
            not isinstance(effective_workers, int)
            or isinstance(effective_workers, bool)
            or effective_workers < 1
        ):
            raise AggregateContractError(
                f"runtime effective_workers is malformed for {simulator}"
            )
        if documents["summary"] != expected_summary:
            raise AggregateContractError(
                f"runtime summary does not match records/intent for {simulator}"
            )
        if documents["progress"] != expected_progress:
            raise AggregateContractError(
                f"runtime progress does not match records/intent for {simulator}"
            )
        _json_finite(documents, f"runtime summary/progress {simulator}")
        runtime_inputs[simulator].update(
            {
                "present": True,
                "effective_workers": effective_workers,
                "intent_count": intent_count,
                "terminal_count": len(records[simulator]),
                "complete": len(records[simulator]) == intent_count,
                "summary_sha256": sha256_file(summary_path),
                "progress_sha256": sha256_file(progress_path),
            }
        )


def load_aggregate_inputs(
    receipt_root: Path,
    dataset_root: Path,
    table1_manifest: Path,
    upstream_roots: dict[str, Path],
    *,
    phase: str,
    intent_ids: list[str] | None = None,
) -> tuple[
    dict[str, Any], dict[str, Any], dict[str, list[dict[str, Any]]], dict[str, Any]
]:
    unresolved_receipt_root = Path(receipt_root)
    if unresolved_receipt_root.is_symlink():
        raise AggregateContractError("runtime receipt root cannot be a symlink")
    receipt_root = unresolved_receipt_root.resolve()
    _require_regular_directory(receipt_root, "runtime receipt root")
    for name in ("receipt_set.json", "protocol.json", "manifest.json"):
        _require_regular_file(receipt_root / name, f"receipt-set file {name}")
    dataset_root = Path(dataset_root).resolve()
    table1_manifest = Path(table1_manifest).resolve()
    if set(upstream_roots) != {"table2", "table3", "table4"}:
        raise AggregateContractError(
            "upstream roots must name table2, table3, and table4"
        )
    upstream_roots = {
        name: Path(path).resolve() for name, path in upstream_roots.items()
    }
    if phase == "qualification" and not intent_ids:
        raise AggregateContractError("qualification phase requires explicit --ids")
    if phase == "formal" and intent_ids is not None:
        raise AggregateContractError(
            "formal phase forbids --ids and always uses the full cohort"
        )
    try:
        protocol, manifest = validate_receipt_set(receipt_root)
        validate_manifest(
            manifest,
            dataset_root,
            table1_manifest,
            upstream_roots,
            protocol=protocol,
            formal=True,
        )
    except (ManifestError, OSError, KeyError, TypeError, ValueError) as error:
        raise AggregateContractError(
            f"receipt/source validation failed: {error}"
        ) from error
    row_by_id = _validate_protocol_manifest(protocol, manifest)
    selected_ids = list(row_by_id)
    if phase == "formal":
        effective_intent_ids = selected_ids
    else:
        assert intent_ids is not None
        if len(intent_ids) != len(set(intent_ids)) or not set(intent_ids).issubset(
            row_by_id
        ):
            raise AggregateContractError(
                "qualification intent IDs must be unique selected manifest IDs"
            )
        requested = set(intent_ids)
        effective_intent_ids = [
            dataset_id for dataset_id in selected_ids if dataset_id in requested
        ]
    records, runtime_inputs = _read_runtime_records(receipt_root, phase)
    _index_records(
        protocol,
        manifest,
        row_by_id,
        records,
        phase,
        effective_intent_ids,
    )
    intent_count = len(effective_intent_ids)
    _validate_runtime_summaries(
        receipt_root, phase, records, intent_count, runtime_inputs
    )
    receipt_info = {
        "receipt_set_sha256": sha256_file(receipt_root / "receipt_set.json"),
        "protocol_file_sha256": sha256_file(receipt_root / "protocol.json"),
        "manifest_file_sha256": sha256_file(receipt_root / "manifest.json"),
        "receipt_root": str(receipt_root),
        "dataset_root": str(dataset_root),
        "table1_manifest": str(table1_manifest),
        "upstream_roots": {name: str(path) for name, path in upstream_roots.items()},
        "runtime_inputs": runtime_inputs,
    }
    _validate_receipt_provenance(
        protocol,
        manifest,
        receipt_info,
        required=True,
    )
    return protocol, manifest, records, receipt_info


def _parse_ids(value: str | None) -> list[str] | None:
    if value is None:
        return None
    ids = value.split(",")
    if (
        not ids
        or any(not item for item in ids)
        or len(ids) != len(set(ids))
        or any(
            len(item) != len("artiverse_0000")
            or not item.startswith("artiverse_")
            or not item.removeprefix("artiverse_").isdigit()
            or int(item.removeprefix("artiverse_")) >= 800
            for item in ids
        )
    ):
        raise AggregateContractError(
            "--ids must contain unique canonical Artiverse dataset IDs"
        )
    return ids


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--receipt-root", required=True)
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--table1-manifest", required=True)
    parser.add_argument("--table2-root", required=True)
    parser.add_argument("--table3-root", required=True)
    parser.add_argument("--table4-root", required=True)
    parser.add_argument("--phase", choices=("formal", "qualification"), required=True)
    parser.add_argument("--ids")
    parser.add_argument("--out", required=True)
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = _parser().parse_args(list(argv) if argv is not None else None)
    intent_ids = _parse_ids(args.ids)
    protocol, manifest, records, receipt_info = load_aggregate_inputs(
        Path(args.receipt_root),
        Path(args.dataset_root),
        Path(args.table1_manifest),
        {
            "table2": Path(args.table2_root),
            "table3": Path(args.table3_root),
            "table4": Path(args.table4_root),
        },
        phase=args.phase,
        intent_ids=intent_ids,
    )
    unresolved_output_root = Path(args.out)
    _validate_publication_location(
        unresolved_output_root,
        receipt_info,
        phase=args.phase,
    )
    bundle = aggregate_records(
        protocol,
        manifest,
        records,
        phase=args.phase,
        intent_ids=intent_ids,
        receipt_info=receipt_info,
    )
    publish_aggregate(
        bundle,
        unresolved_output_root,
        protocol=protocol,
        manifest=manifest,
        records_by_simulator=records,
        phase=args.phase,
        intent_ids=intent_ids,
        receipt_info=receipt_info,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
