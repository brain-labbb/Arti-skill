#!/usr/bin/env python3
"""Read-only acceptance checks for the full-release Table 4 evaluation.

The evaluator is intentionally kept out of this module.  A Table 4 run is a
publication artifact, so this checker validates the frozen roster, per-asset
records, state accounting, receipt bindings, self hashes, and artifact
closure.  It also fails closed for packages without native collision
geometry: an empty contact query is never interpreted as collision-free.

The checker accepts the two record filenames used by the historical adapters
(``asset_records.jsonl`` and ``records.jsonl``) so that old smoke receipts can
be inspected while the full-release runner evolves.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Iterable, Mapping, Sequence

try:
    import table123_full_release_common as common
except ImportError:  # pragma: no cover
    from . import table123_full_release_common as common


class AutomationError(ValueError):
    """Raised when a full-release Table 4 contract is not closed."""


DATASETS: tuple[dict[str, Any], ...] = (
    {"slug": "articraft", "display": "Articraft-10K", "n_eval": 9996, "j_eval": 37144},
    {"slug": "lam", "display": "LAM released outputs", "aliases": ("LAM",), "n_eval": 3217, "j_eval": 10381},
    {"slug": "artiverse", "display": "Artiverse", "n_eval": 3544, "j_eval": 16332},
    {"slug": "partnet", "display": "PartNet-Mobility", "n_eval": 2347, "j_eval": 11971},
    {"slug": "physx", "display": "PhysX-Mobility", "n_eval": 2024, "j_eval": 9883},
    {"slug": "sketch", "display": "SketchMobility", "n_eval": 4956, "j_eval": 11009},
    {"slug": "infinite", "display": "Infinite Mobility", "n_eval": 720, "j_eval": 4723},
    {"slug": "infinigen", "display": "Infinigen-Sim", "n_eval": 8226, "j_eval": 31975},
)
DATASET_BY_SLUG = {item["slug"]: item for item in DATASETS}
DISPLAY_BY_SLUG = {
    item["slug"]: (item["display"], *item.get("aliases", ())) for item in DATASETS
}
SINGLE_SAMPLES = 21
SOBOL_SAMPLES = 64
SAMPLING_PROTOCOL_V1 = "independent_sampling_v1"
SAMPLING_PROTOCOL_V2 = "mimic_aware_independent_sampling_v2"
VALID_STATUS = {"completed", "error", "timeout", "blocked", "skipped"}
METRIC_KEYS = (
    "rest_all_pair_cf",
    "rest_non_adjacent_cf",
    "single_joint_sweep_cf",
    "multi_joint_sobol_cf",
    "collision_state_rate",
    "max_penetration",
    "collision_free_range",
    "strict_collision_pass",
    "aor",
)


def _json(path: Path) -> Any:
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise AutomationError(f"cannot read JSON {path}: {exc}") from exc


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with Path(path).open("rb") as handle:
            for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise AutomationError(f"cannot hash {path}: {exc}") from exc
    return digest.hexdigest()


def _without(value: Mapping[str, Any], field: str) -> dict[str, Any]:
    result = dict(value)
    result.pop(field, None)
    return result


def _canonical(value: Any) -> str:
    return common.canonical_sha256(value)


def _check_self_hash(
    value: Mapping[str, Any],
    label: str,
    *,
    field: str | None = None,
    required: bool = False,
) -> str | None:
    """Check a schema-specific content hash and return the declaration."""

    candidates = (
        "manifest_content_sha256",
        "summary_content_sha256",
        "checkpoint_content_sha256",
        "artifact_manifest_content_sha256",
        "receipt_content_sha256",
    )
    if field is None:
        field = next((name for name in candidates if name in value), None)
    if field is None:
        if required:
            raise AutomationError(f"{label} has no self-hash")
        return None
    declared = value.get(field)
    observed = _canonical(_without(value, field))
    if declared != observed:
        raise AutomationError(f"{label} self-hash mismatch")
    return str(declared)


def _records(path: Path) -> list[dict[str, Any]]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise AutomationError(f"cannot read records {path}: {exc}") from exc
    rows: list[dict[str, Any]] = []
    for number, line in enumerate(lines, 1):
        if not line.strip():
            raise AutomationError(f"blank record row: {path}:{number}")
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise AutomationError(f"invalid record JSON {path}:{number}: {exc}") from exc
        if not isinstance(value, dict):
            raise AutomationError(f"record is not an object: {path}:{number}")
        rows.append(value)
    return rows


def _integer(value: Any, label: str, *, allow_none: bool = False) -> int | None:
    if value is None and allow_none:
        return None
    if isinstance(value, bool):
        raise AutomationError(f"{label} is not an integer")
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise AutomationError(f"{label} is not an integer") from exc
    if parsed < 0:
        raise AutomationError(f"{label} is negative")
    return parsed


def _first(record: Mapping[str, Any], *names: str, default: Any = None) -> Any:
    for name in names:
        if name in record:
            return record[name]
    return default


def _bool(record: Mapping[str, Any], *names: str) -> bool | None:
    value = _first(record, *names)
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    if isinstance(value, str) and value.lower() in {"true", "false"}:
        return value.lower() == "true"
    raise AutomationError(f"boolean field {names[0]} has invalid value {value!r}")


def _rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def _metric_value(summary: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    metrics = summary.get("metrics")
    if not isinstance(metrics, Mapping):
        return {}
    value = metrics.get(key)
    return value if isinstance(value, Mapping) else {"value": value}


def _metric_pair(value: Mapping[str, Any]) -> tuple[int, int] | None:
    """Return a metric numerator/denominator, accepting common aliases."""

    if isinstance(value.get("asset"), Mapping):
        value = value["asset"]
    numerator = _first(value, "numerator", "passed", "collision_states", "passed_states")
    denominator = _first(value, "denominator", "expected", "total_expected")
    if numerator is None or denominator is None:
        return None
    try:
        n, d = int(numerator), int(denominator)
    except (TypeError, ValueError) as exc:
        raise AutomationError("metric numerator/denominator is not integer") from exc
    if n < 0 or d < 0 or n > d:
        raise AutomationError(f"invalid metric fraction {n}/{d}")
    return n, d


def _asset_expected(record: Mapping[str, Any], phase: str, dof: int) -> int:
    aliases = {
        "rest": ("rest_state_expected", "rest_expected"),
        "single": ("single_state_expected", "single_expected"),
        "sobol": ("sobol_state_expected", "sobol_expected"),
    }
    value = _first(record, *aliases[phase])
    if value is None:
        if phase == "rest":
            return 1
        if phase == "single":
            return SINGLE_SAMPLES * dof
        return SOBOL_SAMPLES if dof else 0
    result = _integer(value, f"{phase} expected")
    assert result is not None
    return result


def _asset_executed(record: Mapping[str, Any], phase: str) -> int:
    value = _first(record, f"{phase}_state_executed", f"{phase}_executed")
    result = _integer(value if value is not None else 0, f"{phase} executed")
    assert result is not None
    return result


def _asset_free(record: Mapping[str, Any], phase: str) -> int | None:
    names = {
        "rest": ("rest_non_adjacent_free", "rest_free", "rest_non_adjacent_free_states"),
        "single": ("single_non_adjacent_free", "single_free", "single_free_states"),
        "sobol": ("sobol_non_adjacent_free", "sobol_free", "sobol_free_states"),
    }
    value = _first(record, *names[phase])
    if value is None:
        return None
    result = _integer(value, f"{phase} free")
    assert result is not None
    return result


def _collision_status(record: Mapping[str, Any]) -> str:
    value = _first(record, "collision_metric_status", "collision_status", default="measured")
    return str(value).upper().replace(" ", "_")


def _sampling_metadata(
    record: Mapping[str, Any],
    index: int,
    *,
    expected_protocol: str | None = None,
    expected_declared_dof: int | None = None,
) -> dict[str, Any]:
    """Validate a record's frozen sampling denominator and plan binding.

    Historical v1 records remain governed by their declared movable-DoF
    denominator.  Mimic-aware v2 records must bind the declared and independent
    DoF counts separately; this prevents a summary from silently reusing the
    larger v1 denominator after follower joints stop being sampled as free DoF.
    """

    protocol = str(record.get("sampling_protocol") or SAMPLING_PROTOCOL_V1)
    if protocol not in {SAMPLING_PROTOCOL_V1, SAMPLING_PROTOCOL_V2}:
        raise AutomationError(
            f"record {index} has unknown sampling protocol: {protocol!r}"
        )
    if expected_protocol is not None and protocol != expected_protocol:
        raise AutomationError(
            f"record {index} sampling protocol mismatch: "
            f"{protocol!r} != {expected_protocol!r}"
        )
    declared = _integer(
        _first(
            record,
            "movable_dof_count",
            "expected_movable_joints",
            "dof_count",
            "range_evaluable_dof_count",
            default=0,
        ),
        f"record {index} declared dof",
    )
    assert declared is not None
    if expected_declared_dof is not None and declared != expected_declared_dof:
        raise AutomationError(
            f"record {index} declared DoF mismatch: "
            f"{declared} != {expected_declared_dof}"
        )
    if protocol == SAMPLING_PROTOCOL_V1:
        return {
            "protocol": protocol,
            "declared_dof": declared,
            "independent_dof": declared,
            "range_independent_dof": _integer(
                _first(record, "range_evaluable_dof_count", default=declared),
                f"record {index} range-evaluable dof",
            ),
            "mimic_joint_count": 0,
            "fixed_root_joint_count": 0,
            "plan_sha256": None,
        }

    independent = _integer(
        record.get("independent_dof_count"),
        f"record {index} independent dof",
    )
    range_independent = _integer(
        record.get("range_evaluable_independent_dof_count"),
        f"record {index} range-evaluable independent dof",
    )
    mimic = _integer(
        record.get("mimic_joint_count"),
        f"record {index} mimic joint count",
    )
    fixed_roots = _integer(
        record.get("fixed_root_joint_count", 0),
        f"record {index} fixed root joint count",
    )
    assert (
        independent is not None
        and range_independent is not None
        and mimic is not None
        and fixed_roots is not None
    )
    if independent + mimic + fixed_roots != declared:
        raise AutomationError(
            f"record {index} independent/mimic/fixed DoF do not close declared DoF"
        )
    if range_independent > independent:
        raise AutomationError(
            f"record {index} range-evaluable independent DoF exceeds independent DoF"
        )
    frozen_expected = {
        "rest": 1,
        "single": SINGLE_SAMPLES * independent,
        "sobol": SOBOL_SAMPLES if independent else 0,
    }
    for phase, expected in frozen_expected.items():
        observed = _asset_expected(record, phase, independent)
        if observed != expected:
            raise AutomationError(
                f"record {index} {phase} expected denominator mismatch: "
                f"{observed} != {expected}"
            )
    plan_hash = record.get("joint_sampling_plan_sha256")
    plan_error = record.get("sampling_plan_error")
    if plan_hash is None:
        if not isinstance(plan_error, str) or not plan_error.strip():
            raise AutomationError(f"record {index} has no v2 sampling-plan hash")
        if bool(_first(record, "measurement_complete", "measured", default=False)):
            raise AutomationError(
                f"record {index} is measurement-complete with an invalid v2 sampling plan"
            )
        if any(_asset_executed(record, phase) for phase in ("rest", "single", "sobol")):
            raise AutomationError(
                f"record {index} executed states with an invalid v2 sampling plan"
            )
    elif not isinstance(plan_hash, str) or re.fullmatch(r"[0-9a-f]{64}", plan_hash) is None:
        raise AutomationError(f"record {index} v2 sampling-plan hash is malformed")
    elif plan_error not in {None, ""}:
        raise AutomationError(
            f"record {index} has both a v2 sampling-plan hash and an error"
        )
    return {
        "protocol": protocol,
        "declared_dof": declared,
        "independent_dof": independent,
        "range_independent_dof": range_independent,
        "mimic_joint_count": mimic,
        "fixed_root_joint_count": fixed_roots,
        "plan_sha256": plan_hash,
    }


def aggregate_records(
    records: Sequence[Mapping[str, Any]],
    n_eval: int,
    j_eval: int,
) -> dict[str, Any]:
    """Recompute Table 4 headline metrics from immutable asset records.

    Missing collision observations are retained in the frozen state
    denominator.  If every asset is explicitly N/E for native collision
    geometry, collision-dependent headline metrics are N/E rather than
    vacuously passing.
    """

    if len(records) != n_eval:
        raise AutomationError(f"record count mismatch: {len(records)} != {n_eval}")
    status_counts: dict[str, int] = {}
    totals = {phase: {"expected": 0, "executed": 0, "free": 0} for phase in ("rest", "single", "sobol")}
    pass_counts = {"rest_all_pair_cf": 0, "rest_non_adjacent_cf": 0, "single_joint_sweep_cf": 0, "multi_joint_sobol_cf": 0, "strict_collision_pass": 0}
    # Asset-level pass metrics use the frozen release denominator.  A blocked
    # or failed asset contributes no passing boolean, but it must remain in
    # the denominator (the runner uses the same fail-closed convention).
    pass_denoms = {key: n_eval for key in pass_counts}
    max_values: list[float] = []
    measured_assets = 0
    collision_assets = 0
    categories: dict[str, list[Mapping[str, Any]]] = {}
    declared_dof_total = 0
    independent_dof_total = 0
    mimic_joint_total = 0
    fixed_root_joint_total = 0
    sampling_protocols: set[str] = set()
    for index, record in enumerate(records):
        status = str(record.get("status", ""))
        if status not in VALID_STATUS:
            raise AutomationError(f"record {index} has unknown status: {status!r}")
        status_counts[status] = status_counts.get(status, 0) + 1
        sampling = _sampling_metadata(record, index)
        # v1 samples every declared movable joint independently.  v2 samples
        # only independent roots and expands mimic followers from the frozen
        # affine plan.  Non-range-evaluable roots remain in either protocol's
        # fail-closed denominator.
        dof = int(sampling["independent_dof"])
        declared_dof_total += int(sampling["declared_dof"])
        independent_dof_total += dof
        mimic_joint_total += int(sampling["mimic_joint_count"])
        fixed_root_joint_total += int(sampling["fixed_root_joint_count"])
        sampling_protocols.add(str(sampling["protocol"]))
        for phase in totals:
            expected = _asset_expected(record, phase, dof)
            executed = _asset_executed(record, phase)
            free = _asset_free(record, phase)
            if executed > expected:
                raise AutomationError(f"record {index} {phase} executed exceeds expected")
            if free is not None and free > executed:
                raise AutomationError(f"record {index} {phase} free exceeds executed")
            totals[phase]["expected"] += expected
            totals[phase]["executed"] += executed
            totals[phase]["free"] += free if free is not None else 0
        native_elements = _integer(record.get("native_collision_elements", 0), f"record {index} native collision elements")
        assert native_elements is not None
        collision_ne = (
            native_elements == 0
            or _collision_status(record) in {"N/E", "NE", "BLOCKED", "NO_NATIVE_COLLISION", "NO_COLLISION_GEOMETRY"}
        )
        if collision_ne and bool(_first(record, "measurement_complete", "measured", default=False)):
            raise AutomationError(f"record {index} is measurement-complete without native collision geometry")
        native_zero_dof = dof == 0 and not collision_ne
        for key, aliases in {
            "rest_all_pair_cf": ("rest_all_pair_cf", "rest_all_pair_cf_passed"),
            "rest_non_adjacent_cf": ("rest_non_adjacent_cf", "rest_non_adjacent_cf_passed"),
            "single_joint_sweep_cf": ("single_joint_sweep_cf", "joint_single_sweep_cf", "single_joint_sweep_cf_passed"),
            "multi_joint_sobol_cf": ("multi_joint_sobol_cf", "multi_joint_sobol_cf_passed"),
            "strict_collision_pass": ("strict_collision_pass", "strict_collision_passed"),
        }.items():
            value = _bool(record, *aliases)
            if value is not None:
                # An N/E record may not silently publish a passing boolean.
                if collision_ne and value:
                    raise AutomationError(f"record {index} reports pass despite collision N/E")
                if native_zero_dof and key in {"multi_joint_sobol_cf", "strict_collision_pass"} and value:
                    raise AutomationError(f"record {index} reports vacuous zero-DoF pass for {key}")
                pass_counts[key] += int(value)
        raw_max = _first(record, "max_penetration_normalized", "max_penetration_norm")
        if raw_max is not None:
            try:
                maximum = float(raw_max)
            except (TypeError, ValueError) as exc:
                raise AutomationError(f"record {index} max penetration is not numeric") from exc
            if not math.isfinite(maximum) or maximum < 0:
                raise AutomationError(f"record {index} max penetration is invalid")
            max_values.append(maximum)
        if bool(_first(record, "measurement_complete", "measured", default=False)) and not collision_ne:
            measured_assets += 1
        if not collision_ne:
            collision_assets += 1
        category = str(record.get("category") or "__UNSPECIFIED__")
        categories.setdefault(category, []).append(record)

    if totals["single"]["expected"] and totals["single"]["expected"] < 0:
        raise AutomationError("invalid single-joint denominator")
    metrics: dict[str, Any] = {}
    for key in ("rest_all_pair_cf", "rest_non_adjacent_cf", "single_joint_sweep_cf", "multi_joint_sobol_cf", "strict_collision_pass"):
        denominator = pass_denoms[key]
        metrics[key] = {"numerator": pass_counts[key], "denominator": denominator, "rate": _rate(pass_counts[key], denominator)}
    total_expected = sum(totals[p]["expected"] for p in totals)
    total_executed = sum(totals[p]["executed"] for p in totals)
    total_free = sum(totals[p]["free"] for p in totals)
    if collision_assets == 0:
        # There is no measurable collision oracle at all.  Keep every
        # collision-dependent asset metric explicitly N/E, matching the
        # runner's fail-closed summary instead of publishing zero-valued
        # vacuous rates.
        for key in ("rest_all_pair_cf", "rest_non_adjacent_cf", "single_joint_sweep_cf", "multi_joint_sobol_cf", "strict_collision_pass"):
            metrics[key] = {"status": "N/E", "reason": "no_native_collision_geometry", "numerator": None, "denominator": None}
        metrics["collision_state_rate"] = {"status": "N/E", "reason": "no_native_collision_geometry", "numerator": None, "denominator": None}
        metrics["collision_free_range"] = {"status": "N/E", "reason": "no_native_collision_geometry", "numerator": None, "denominator": None}
    else:
        metrics["collision_state_rate"] = {
            "numerator": total_expected - total_free,
            "denominator": total_expected,
            "rate": _rate(total_expected - total_free, total_expected),
            "executed_states": total_executed,
            "unexecuted_states": total_expected - total_executed,
        }
        metrics["collision_free_range"] = {
            "numerator": totals["single"]["free"],
            "denominator": totals["single"]["expected"],
            "rate": _rate(totals["single"]["free"], totals["single"]["expected"]),
        }
    metrics["max_penetration"] = {
        "status": "N/E" if collision_assets == 0 or not max_values else ("COMPLETE" if measured_assets == n_eval else "PARTIAL"),
        "maximum_observed_normalized": max(max_values) if max_values and collision_assets else None,
        "observed_assets": len(max_values),
        "measured_assets": measured_assets,
        "denominator": n_eval,
    }
    metrics["aor"] = {"status": "N/E", "reason": "exact_overlap_volume_not_implemented"}
    if collision_assets == 0 and records and all(str(row.get("status")) == "blocked" for row in records):
        overall_status = "BLOCKED"
    elif all(str(row.get("status")) == "completed" for row in records):
        overall_status = "COMPLETE"
    else:
        overall_status = "COMPLETE_WITH_RETAINED_FAILURES"
    return {
        "n_eval": n_eval,
        "j_eval": j_eval,
        "declared_dof_count": declared_dof_total,
        "independent_dof_count": independent_dof_total,
        "mimic_joint_count": mimic_joint_total,
        "fixed_root_joint_count": fixed_root_joint_total,
        "sampling_protocol": (
            next(iter(sampling_protocols))
            if len(sampling_protocols) == 1
            else "mixed"
        ),
        "status": overall_status,
        "status_counts": dict(sorted(status_counts.items())),
        "expected_states": {phase: values["expected"] for phase, values in totals.items()},
        "executed_states": {phase: values["executed"] for phase, values in totals.items()},
        "metrics": metrics,
        "collision_geometry_assets": collision_assets,
        "category_count": len(categories),
    }


def _find_records(output: Path) -> Path:
    for name in ("asset_records.jsonl", "records.jsonl"):
        candidate = output / name
        if candidate.is_file():
            return candidate
    raise AutomationError(f"missing asset records JSONL: {output}")


def _resolve(root: Path, value: Any, *, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise AutomationError(f"{label} path is missing")
    path = Path(value)
    if not path.is_absolute():
        path = root / path
    try:
        return path.resolve(strict=True)
    except OSError as exc:
        raise AutomationError(f"{label} path is missing: {path}") from exc


def _load_roster(manifest: Mapping[str, Any], output: Path, root: Path) -> tuple[Mapping[str, Any], Path]:
    value = manifest.get("roster") or manifest.get("roster_path")
    if value is None:
        for candidate in (output / "roster_manifest.json", output / "full_release_manifest.json"):
            if candidate.is_file():
                value = str(candidate)
                break
    path = _resolve(root, value, label="roster")
    roster = _json(path)
    if not isinstance(roster, Mapping):
        raise AutomationError("roster manifest is not an object")
    schema = str(roster.get("schema_version", ""))
    if schema not in {"table123_full_release_manifest_v1", "table4_full_release_roster_v1"}:
        raise AutomationError(f"roster schema mismatch: {schema}")
    if "manifest_content_sha256" in roster:
        _check_self_hash(roster, "roster manifest", field="manifest_content_sha256", required=True)
    rows = roster.get("rows", roster.get("assets"))
    if not isinstance(rows, list):
        raise AutomationError("roster has no rows/assets list")
    if manifest.get("roster_sha256"):
        observed = _sha(path)
        if observed != manifest["roster_sha256"]:
            # Some runners bind the canonical rows rather than file bytes.
            if _canonical(rows) != str(manifest["roster_sha256"]):
                raise AutomationError("run manifest roster hash mismatch")
    return roster, path


def _verify_record_identity(record: Mapping[str, Any], row: Mapping[str, Any], index: int) -> None:
    expected_id = str(row.get("asset_id", row.get("dataset_id", "")))
    observed_id = str(_first(record, "asset_id", "dataset_id", default=""))
    if observed_id != expected_id:
        raise AutomationError(f"record identity mismatch at {index}: {observed_id!r} != {expected_id!r}")
    ordinal = _first(record, "selection_index", "ordinal", "order", default=index)
    try:
        parsed_ordinal = int(ordinal)
    except (TypeError, ValueError) as exc:
        raise AutomationError(f"record order is invalid at {index}") from exc
    if parsed_ordinal != index:
        raise AutomationError(f"record order mismatch at {index}")
    expected_hash = row.get("primary_urdf_sha256") or row.get("urdf_sha256")
    observed_hash = _first(record, "expected_primary_urdf_sha256", "primary_urdf_sha256", "urdf_sha256")
    if expected_hash and observed_hash and str(expected_hash) != str(observed_hash):
        raise AutomationError(f"record URDF hash binding mismatch at {index}")
    expected_joints = row.get("joint_count")
    if expected_joints is None and isinstance(row.get("non_fixed_joints"), list):
        expected_joints = len(row["non_fixed_joints"])
    observed_joints = _first(record, "roster_joint_count", "expected_movable_joints", "joint_count")
    if expected_joints is not None and observed_joints is not None and int(expected_joints) != int(observed_joints):
        raise AutomationError(f"record joint binding mismatch at {index}")


def _verify_state_records(
    output: Path,
    records: Sequence[Mapping[str, Any]],
    *,
    expected_sampling_protocol: str | None = None,
) -> dict[str, Any]:
    path = output / "state_records.jsonl"
    if not path.is_file():
        expected = sum(
            _asset_executed(record, phase)
            for record in records
            for phase in ("rest", "single", "sobol")
        )
        if expected:
            raise AutomationError("state_records.jsonl is missing despite executed states")
        # A blocked-only run may intentionally have no state rows.
        return {"present": False, "rows": 0}
    rows = _records(path)
    by_identity: dict[tuple[str, int], Mapping[str, Any]] = {}
    for index, record in enumerate(records):
        key = (str(_first(record, "dataset_id", "asset_id", default="")), int(_first(record, "order", default=index)))
        if key in by_identity:
            raise AutomationError(f"duplicate asset identity in state binding: {key}")
        by_identity[key] = record
    grouped: dict[tuple[str, int], list[Mapping[str, Any]]] = defaultdict(list)
    valid_phases = {"rest", "single_joint_sweep", "multi_joint_sobol"}
    for index, row in enumerate(rows):
        phase = row.get("phase", row.get("stage", ""))
        if not isinstance(phase, str) or phase not in valid_phases:
            raise AutomationError(f"state record {index} has no phase")
        asset_id = str(row.get("dataset_id", row.get("asset_id", "")))
        try:
            order = int(row.get("order", row.get("ordinal", -1)))
        except (TypeError, ValueError) as exc:
            raise AutomationError(f"state record {index} has invalid order") from exc
        record = by_identity.get((asset_id, order))
        if record is None:
            raise AutomationError(f"state record {index} is not bound to an asset")
        expected_identity = record.get("input_identity_sha256")
        observed_identity = row.get("input_identity_sha256")
        if expected_identity and observed_identity and str(expected_identity) != str(observed_identity):
            raise AutomationError(f"state record {index} input identity mismatch")
        protocol = str(record.get("sampling_protocol") or SAMPLING_PROTOCOL_V1)
        if expected_sampling_protocol is not None and protocol != expected_sampling_protocol:
            raise AutomationError(f"state record {index} parent protocol mismatch")
        if protocol == SAMPLING_PROTOCOL_V2:
            if row.get("schema_version") != "table4_state_v2":
                raise AutomationError(f"state record {index} v2 schema mismatch")
            if row.get("sampling_protocol") != protocol:
                raise AutomationError(f"state record {index} sampling protocol mismatch")
            if not expected_identity or observed_identity != expected_identity:
                raise AutomationError(f"state record {index} has no exact input identity binding")
            if row.get("joint_sampling_plan_sha256") != record.get(
                "joint_sampling_plan_sha256"
            ):
                raise AutomationError(f"state record {index} sampling-plan hash mismatch")
        grouped[(asset_id, order)].append(row)
    expected = sum(
        _asset_executed(record, phase)
        for record in records
        for phase in ("rest", "single", "sobol")
    )
    if len(rows) != expected:
        raise AutomationError(f"state record execution mismatch: {len(rows)} != {expected}")
    phase_key = {
        "rest": "rest",
        "single_joint_sweep": "single",
        "multi_joint_sobol": "sobol",
    }
    for asset_index, record in enumerate(records):
        key = (str(_first(record, "dataset_id", "asset_id", default="")), int(_first(record, "order", default=0)))
        states = grouped.get(key, [])
        declared = record.get("state_records_count")
        if declared is not None and int(declared) != len(states):
            raise AutomationError(f"asset state record count mismatch: {key}")
        expected_hash = record.get("state_records_sha256")
        if expected_hash:
            observed_hash = _canonical(states)
            if str(expected_hash) != observed_hash:
                raise AutomationError(f"asset state record hash mismatch: {key}")
        counts = {phase: 0 for phase in ("rest", "single", "sobol")}
        rest_indexes: set[int] = set()
        single_keys: set[tuple[str, int]] = set()
        single_by_joint: dict[str, set[int]] = defaultdict(set)
        sobol_indexes: set[int] = set()
        protocol = str(record.get("sampling_protocol") or SAMPLING_PROTOCOL_V1)
        for state_index, state in enumerate(states):
            raw_phase = str(state.get("phase", state.get("stage", "")))
            phase = phase_key[raw_phase]
            counts[phase] += 1
            if protocol != SAMPLING_PROTOCOL_V2:
                continue
            sample_index = _integer(
                state.get("sample_index"),
                f"asset {key} state {state_index} sample index",
            )
            assert sample_index is not None
            if phase == "rest":
                if sample_index != 0 or sample_index in rest_indexes:
                    raise AutomationError(f"asset {key} has invalid/duplicate rest state")
                if state.get("joint_name") not in {None, ""}:
                    raise AutomationError(f"asset {key} rest state names a joint")
                rest_indexes.add(sample_index)
            elif phase == "single":
                joint_name = state.get("joint_name")
                if not isinstance(joint_name, str) or not joint_name:
                    raise AutomationError(f"asset {key} single state has no joint name")
                if sample_index >= SINGLE_SAMPLES:
                    raise AutomationError(f"asset {key} single sample index is out of range")
                state_key = (joint_name, sample_index)
                if state_key in single_keys:
                    raise AutomationError(f"asset {key} has duplicate single state {state_key}")
                single_keys.add(state_key)
                single_by_joint[joint_name].add(sample_index)
            else:
                if state.get("joint_name") not in {None, ""}:
                    raise AutomationError(f"asset {key} Sobol state names a joint")
                if sample_index >= SOBOL_SAMPLES or sample_index in sobol_indexes:
                    raise AutomationError(f"asset {key} has invalid/duplicate Sobol state")
                sobol_indexes.add(sample_index)
        for phase in counts:
            declared_executed = _asset_executed(record, phase)
            if counts[phase] != declared_executed:
                raise AutomationError(
                    f"asset {key} {phase} state coverage mismatch: "
                    f"{counts[phase]} != {declared_executed}"
                )
        if protocol == SAMPLING_PROTOCOL_V2:
            sampling = _sampling_metadata(record, asset_index)
            independent = int(sampling["independent_dof"])
            range_independent = int(sampling["range_independent_dof"] or 0)
            if len(single_by_joint) > range_independent:
                raise AutomationError(
                    f"asset {key} samples more joints than its range-evaluable independent DoF"
                )
            single_expected = _asset_expected(record, "single", independent)
            if counts["single"] == single_expected and single_expected:
                if len(single_by_joint) != independent or any(
                    indexes != set(range(SINGLE_SAMPLES))
                    for indexes in single_by_joint.values()
                ):
                    raise AutomationError(f"asset {key} has incomplete v2 single-state coverage")
            sobol_expected = _asset_expected(record, "sobol", independent)
            if counts["sobol"] == sobol_expected and sobol_expected:
                if sobol_indexes != set(range(SOBOL_SAMPLES)):
                    raise AutomationError(f"asset {key} has incomplete v2 Sobol coverage")
    return {"present": True, "rows": len(rows), "sha256": _sha(path)}


def _verify_artifacts(output: Path) -> None:
    artifact = output / "artifact_manifest.json"
    if not artifact.is_file():
        raise AutomationError(f"missing artifact manifest: {output}")
    try:
        common.verify_artifacts(artifact)
    except Exception as exc:  # noqa: BLE001
        raise AutomationError(f"artifact closure failed: {exc}") from exc


def _binding_map(value: Any, label: str) -> list[Mapping[str, Any]]:
    if not isinstance(value, list):
        raise AutomationError(f"{label} source_bindings is missing or not a list")
    result: list[Mapping[str, Any]] = []
    names: set[str] = set()
    for index, binding in enumerate(value):
        if not isinstance(binding, Mapping):
            raise AutomationError(f"{label} source binding {index} is not an object")
        if not binding.get("name") or not binding.get("path"):
            raise AutomationError(f"{label} source binding {index} has no name/path")
        name = str(binding["name"])
        if name in names:
            raise AutomationError(f"{label} has duplicate source binding name: {name}")
        names.add(name)
        result.append(binding)
    return result


def _binding_named(bindings: Sequence[Mapping[str, Any]], name: str, label: str) -> Mapping[str, Any]:
    matches = [binding for binding in bindings if str(binding.get("name")) == name]
    if len(matches) != 1:
        raise AutomationError(f"{label} requires exactly one source binding: {name}")
    return matches[0]


def _verify_hashed_binding(binding: Mapping[str, Any], label: str, *, required: bool = False) -> Path:
    raw_path = binding.get("path")
    try:
        path = Path(str(raw_path)).resolve(strict=True)
    except OSError as exc:
        raise AutomationError(f"{label} source path is missing: {raw_path}") from exc
    declared = binding.get("sha256")
    if required and declared is None:
        raise AutomationError(f"{label} source binding has no SHA-256")
    if declared is not None:
        if not path.is_file():
            raise AutomationError(f"{label} hashed source is not a file: {path}")
        if not isinstance(declared, str) or len(declared) != 64:
            raise AutomationError(f"{label} source SHA-256 is malformed")
        if _sha(path) != declared:
            raise AutomationError(f"{label} source binding hash mismatch")
        if binding.get("bytes") is not None and int(binding["bytes"]) != path.stat().st_size:
            raise AutomationError(f"{label} source byte count mismatch")
    return path


def _verify_dataset_sources(
    item: Mapping[str, Any],
    manifest: Mapping[str, Any],
    summary: Mapping[str, Any],
    entry: Mapping[str, Any],
    roster: Mapping[str, Any],
    n_eval: int,
) -> dict[str, Any]:
    """Verify the two user-supplied source paths that need explicit closure."""

    slug = str(item["slug"])
    manifest_bindings = _binding_map(manifest.get("source_bindings"), f"{item['display']} run manifest")
    for binding in manifest_bindings:
        _verify_hashed_binding(binding, item["display"])
    summary_value = summary.get("source_bindings")
    if summary_value is not None:
        summary_bindings = _binding_map(summary_value, f"{item['display']} summary")
        if [dict(value) for value in summary_bindings] != [dict(value) for value in manifest_bindings]:
            raise AutomationError(f"{item['display']} summary source bindings do not match manifest")
    entry_value = entry.get("source_bindings")
    if entry_value is not None:
        entry_bindings = _binding_map(entry_value, f"{item['display']} receipt")
        if [dict(value) for value in entry_bindings] != [dict(value) for value in manifest_bindings]:
            raise AutomationError(f"{item['display']} receipt source bindings do not match manifest")

    if slug == "infinite":
        parts = _binding_named(manifest_bindings, "parts_zip", item["display"])
        parts_path = _verify_hashed_binding(parts, item["display"], required=True)
        if not parts_path.is_file():
            raise AutomationError(f"{item['display']} parts_zip is not a file")
        # The evaluated packages are the generated cohort; retain and check
        # its manifest binding as the immediate input identity.
        cohort = [
            binding
            for binding in manifest_bindings
            if "manifest" in str(binding.get("name", "")).lower()
            or "cohort" in str(binding.get("name", "")).lower()
        ]
        if cohort:
            _verify_hashed_binding(cohort[0], item["display"])
        return {"parts_zip": str(parts_path)}

    if slug == "infinigen":
        source = _binding_named(manifest_bindings, "source_root", item["display"])
        source_path = _verify_hashed_binding(source, item["display"])
        if not source_path.is_dir() or not (source_path / "urdf").is_dir():
            raise AutomationError(f"{item['display']} source_root has no urdf directory")
        archive = _binding_named(
            manifest_bindings,
            "infinigen_archive_validation_receipt",
            item["display"],
        )
        archive_path = _verify_hashed_binding(archive, item["display"], required=True)
        receipt = _json(archive_path)
        if not isinstance(receipt, Mapping):
            raise AutomationError(f"{item['display']} archive validation receipt is not an object")
        internal_hash = receipt.get("receipt_sha256", receipt.get("receipt_content_sha256"))
        if internal_hash:
            observed_internal = _canonical(_without(receipt, "receipt_sha256"))
            if receipt.get("receipt_content_sha256") is not None:
                observed_internal = _canonical(_without(receipt, "receipt_content_sha256"))
            if str(internal_hash) != observed_internal:
                raise AutomationError(f"{item['display']} archive validation receipt self-hash mismatch")
        archive_root = Path(str(receipt.get("archive_root", ""))).resolve()
        if archive_root != (source_path / "urdf").resolve():
            raise AutomationError(f"{item['display']} archive root is not bound to source_root/urdf")
        extracted_raw = receipt.get("extracted_root")
        extracted = Path(str(extracted_raw)).resolve() if extracted_raw else None
        if extracted is None or not extracted.is_dir():
            raise AutomationError(f"{item['display']} extracted_root is missing")
        if int(receipt.get("extracted_urdf_count", -1)) != n_eval:
            raise AutomationError(f"{item['display']} extracted URDF count mismatch")
        extracted_binding = _binding_named(manifest_bindings, "extracted_root", item["display"])
        if Path(str(extracted_binding.get("path"))).resolve() != extracted:
            raise AutomationError(f"{item['display']} extracted_root binding mismatch")
        rows = roster.get("rows", roster.get("assets", []))
        if not isinstance(rows, list) or len(rows) != n_eval:
            raise AutomationError(f"{item['display']} roster rows unavailable for source containment")
        for index, row in enumerate(rows):
            if not isinstance(row, Mapping):
                raise AutomationError(f"{item['display']} roster row {index} is not an object")
            raw_package = row.get("source_path") or row.get("package_root")
            raw_urdf = row.get("primary_urdf_path") or row.get("urdf_path")
            if not raw_package or not raw_urdf:
                raise AutomationError(f"{item['display']} roster row {index} has no source paths")
            package = Path(str(raw_package)).resolve()
            urdf = Path(str(raw_urdf)).resolve()
            try:
                package.relative_to(extracted)
                urdf.relative_to(package)
            except ValueError as exc:
                raise AutomationError(f"{item['display']} roster row {index} escapes extracted_root") from exc
        return {
            "source_root": str(source_path),
            "archive_receipt": str(archive_path),
            "extracted_root": str(extracted),
        }
    return {}


def _verify_dataset(root: Path, item: Mapping[str, Any], entry: Mapping[str, Any]) -> dict[str, Any]:
    evidence = entry.get("evidence") if isinstance(entry.get("evidence"), Mapping) else {}
    output_value = entry.get("output") or entry.get("output_root") or evidence.get("output")
    if output_value is None:
        summary_ref = evidence.get("summary")
        if summary_ref:
            output_value = str(Path(summary_ref).parent)
    output = _resolve(root, output_value, label=f"{item['display']} output")
    manifest_path = output / "manifest.json"
    if not manifest_path.is_file():
        raise AutomationError(f"missing run manifest: {item['display']}")
    manifest = _json(manifest_path)
    if not isinstance(manifest, Mapping):
        raise AutomationError(f"run manifest is not an object: {item['display']}")
    schema = str(manifest.get("schema_version", ""))
    if schema not in {"table4_full_release_run_v1", "table4_full_release_run_v2"}:
        raise AutomationError(f"{item['display']} run manifest schema mismatch: {schema}")
    manifest_protocol = str(
        manifest.get("sampling_protocol") or SAMPLING_PROTOCOL_V1
    )
    expected_schema = (
        "table4_full_release_run_v2"
        if manifest_protocol == SAMPLING_PROTOCOL_V2
        else "table4_full_release_run_v1"
    )
    if manifest_protocol not in {SAMPLING_PROTOCOL_V1, SAMPLING_PROTOCOL_V2}:
        raise AutomationError(
            f"{item['display']} manifest sampling protocol is unknown"
        )
    if schema != expected_schema:
        raise AutomationError(
            f"{item['display']} manifest schema/protocol mismatch"
        )
    _check_self_hash(manifest, f"{item['display']} run manifest", field="manifest_content_sha256", required=True)
    n_eval = _integer(manifest.get("N_eval", manifest.get("n_eval")), f"{item['display']} N_eval")
    j_eval = _integer(manifest.get("J_eval", manifest.get("j_eval")), f"{item['display']} J_eval")
    assert n_eval is not None and j_eval is not None
    if (n_eval, j_eval) != (item["n_eval"], item["j_eval"]):
        raise AutomationError(f"{item['display']} manifest N/J mismatch")
    roster, roster_path = _load_roster(manifest, output, root)
    rows = roster.get("rows", roster.get("assets"))
    assert isinstance(rows, list)
    if len(rows) != n_eval:
        raise AutomationError(f"{item['display']} roster count mismatch")
    roster_joints = sum(int(row.get("joint_count", len(row.get("non_fixed_joints", [])))) for row in rows if isinstance(row, Mapping))
    if roster_joints != j_eval:
        raise AutomationError(f"{item['display']} roster J_eval mismatch: {roster_joints} != {j_eval}")
    records_path = _find_records(output)
    records = _records(records_path)
    if len(records) != n_eval:
        raise AutomationError(f"{item['display']} record count mismatch")
    observed_runner_hashes: set[str] = set()
    observed_core_hashes: set[str] = set()
    for record in records:
        identity = record.get("runtime_identity")
        if isinstance(identity, Mapping):
            runner_hash = identity.get("runner_sha256")
            core_hash = identity.get("collision_core_sha256")
            if runner_hash:
                observed_runner_hashes.add(str(runner_hash))
            if core_hash:
                observed_core_hashes.add(str(core_hash))
    declared_runner_hash = manifest.get("runner_sha256")
    declared_core_hash = manifest.get("collision_core_sha256")
    # Resumable jobs may legitimately contain more than one runner hash when
    # a protocol-preserving bug fix was applied between waves.  Require the
    # manifest's declared hash to be represented in the records, and expose
    # the complete observed set for audit, without rewriting provenance.
    if observed_runner_hashes and declared_runner_hash and str(declared_runner_hash) not in observed_runner_hashes:
        raise AutomationError(f"{item['display']} manifest runner hash is absent from records")
    if observed_core_hashes and declared_core_hash and str(declared_core_hash) not in observed_core_hashes:
        raise AutomationError(f"{item['display']} manifest collision-core hash is absent from records")
    independent_total = 0
    mimic_total = 0
    fixed_root_total = 0
    for index, (record, row) in enumerate(zip(records, rows, strict=True)):
        if not isinstance(row, Mapping):
            raise AutomationError(f"{item['display']} roster row {index} is not an object")
        _verify_record_identity(record, row, index)
        row_dof = int(
            row.get("joint_count", len(row.get("non_fixed_joints", [])))
        )
        sampling = _sampling_metadata(
            record,
            index,
            expected_protocol=manifest_protocol,
            expected_declared_dof=row_dof,
        )
        independent_total += int(sampling["independent_dof"])
        mimic_total += int(sampling["mimic_joint_count"])
        fixed_root_total += int(sampling["fixed_root_joint_count"])
    if manifest_protocol == SAMPLING_PROTOCOL_V2:
        if int(manifest.get("independent_J_eval", -1)) != independent_total:
            raise AutomationError(
                f"{item['display']} manifest independent J_eval mismatch"
            )
        if int(manifest.get("mimic_joint_count", -1)) != mimic_total:
            raise AutomationError(
                f"{item['display']} manifest mimic joint count mismatch"
            )
        if "fixed_root_joint_count" in manifest and int(
            manifest["fixed_root_joint_count"]
        ) != fixed_root_total:
            raise AutomationError(
                f"{item['display']} manifest fixed root joint count mismatch"
            )
    state_info = _verify_state_records(
        output,
        records,
        expected_sampling_protocol=manifest_protocol,
    )
    summary_path = output / "summary.json"
    if not summary_path.is_file():
        raise AutomationError(f"missing summary: {item['display']}")
    summary = _json(summary_path)
    if not isinstance(summary, Mapping):
        raise AutomationError(f"summary is not an object: {item['display']}")
    # Bind the published summary and combined receipt to the live run rather
    # than trusting the receipt's duplicated headline fields.
    summary_n = _integer(summary.get("n_eval", summary.get("N_eval")), f"{item['display']} summary N_eval")
    summary_j = _integer(summary.get("j_eval", summary.get("J_eval")), f"{item['display']} summary J_eval")
    if (summary_n, summary_j) != (n_eval, j_eval):
        raise AutomationError(f"{item['display']} summary N/J mismatch")
    if entry.get("N_eval", entry.get("n_eval")) is not None and int(entry["N_eval"] if "N_eval" in entry else entry["n_eval"]) != n_eval:
        raise AutomationError(f"{item['display']} receipt N_eval mismatch")
    if entry.get("J_eval", entry.get("j_eval")) is not None and int(entry["J_eval"] if "J_eval" in entry else entry["j_eval"]) != j_eval:
        raise AutomationError(f"{item['display']} receipt J_eval mismatch")
    if entry.get("status") is not None and str(entry["status"]).upper() != str(summary.get("status", "")).upper():
        raise AutomationError(f"{item['display']} receipt status mismatch")
    if "summary_content_sha256" in summary:
        _check_self_hash(summary, f"{item['display']} summary", field="summary_content_sha256", required=True)
    if summary.get("records_sha256") and _sha(records_path) != summary["records_sha256"]:
        raise AutomationError(f"{item['display']} summary records hash mismatch")
    if summary.get("manifest_content_sha256") and summary.get("manifest_content_sha256") != manifest.get("manifest_content_sha256"):
        raise AutomationError(f"{item['display']} summary manifest binding mismatch")
    summary_protocol = str(
        summary.get("sampling_protocol") or SAMPLING_PROTOCOL_V1
    )
    if summary_protocol != manifest_protocol:
        raise AutomationError(f"{item['display']} summary sampling protocol mismatch")
    if entry.get("sampling_protocol") is not None and str(
        entry["sampling_protocol"]
    ) != manifest_protocol:
        raise AutomationError(f"{item['display']} receipt sampling protocol mismatch")
    if manifest_protocol == SAMPLING_PROTOCOL_V2:
        if int(summary.get("independent_j_eval", -1)) != independent_total:
            raise AutomationError(
                f"{item['display']} summary independent J_eval mismatch"
            )
        if int(summary.get("mimic_joint_count", -1)) != mimic_total:
            raise AutomationError(
                f"{item['display']} summary mimic joint count mismatch"
            )
        if "fixed_root_joint_count" in summary and int(
            summary["fixed_root_joint_count"]
        ) != fixed_root_total:
            raise AutomationError(
                f"{item['display']} summary fixed root joint count mismatch"
            )
    source_audit = _verify_dataset_sources(item, manifest, summary, entry, roster, n_eval)
    aggregate = aggregate_records(records, n_eval, j_eval)
    if str(summary.get("status", "")).upper() != str(aggregate.get("status", "")).upper():
        raise AutomationError(f"{item['display']} summary status mismatch")
    if summary.get("status_counts") and summary.get("status_counts") != aggregate["status_counts"]:
        raise AutomationError(f"{item['display']} summary status_counts mismatch")
    for field in ("expected_states", "executed_states"):
        if summary.get(field) != aggregate[field]:
            raise AutomationError(f"{item['display']} summary {field} mismatch")
    published = summary.get("metrics")
    if not isinstance(published, Mapping):
        raise AutomationError(f"{item['display']} summary metrics missing")
    for key, expected in aggregate["metrics"].items():
        observed = published.get(key)
        if not isinstance(observed, Mapping):
            raise AutomationError(f"{item['display']} summary metric missing: {key}")
        expected_pair = _metric_pair(expected)
        observed_pair = _metric_pair(observed)
        expected_ne = str(expected.get("status", "")).upper() in {"N/E", "NE"}
        observed_ne = str(observed.get("status", "")).upper() in {"N/E", "NE"}
        if expected_ne:
            if not observed_ne:
                raise AutomationError(f"{item['display']} {key} must remain N/E")
        elif expected_pair is not None and observed_pair != expected_pair:
            raise AutomationError(f"{item['display']} summary metric mismatch: {key}")
    receipt_metrics = entry.get("metrics")
    if isinstance(receipt_metrics, Mapping):
        for key, observed in published.items():
            declared = receipt_metrics.get(key)
            if not isinstance(declared, Mapping) or not isinstance(observed, Mapping):
                continue
            if str(declared.get("status", "")).upper() in {"N/E", "NE"} or str(observed.get("status", "")).upper() in {"N/E", "NE"}:
                if str(declared.get("status", "")).upper() != str(observed.get("status", "")).upper():
                    raise AutomationError(f"{item['display']} receipt metric status mismatch: {key}")
            else:
                declared_pair = _metric_pair(declared)
                observed_pair = _metric_pair(observed)
                if declared_pair is not None and observed_pair is not None and declared_pair != observed_pair:
                    raise AutomationError(f"{item['display']} receipt metric mismatch: {key}")
    checkpoint_path = output / "checkpoint.json"
    if not checkpoint_path.is_file():
        raise AutomationError(f"missing checkpoint: {item['display']}")
    checkpoint = _json(checkpoint_path)
    if "checkpoint_content_sha256" in checkpoint:
        _check_self_hash(checkpoint, f"{item['display']} checkpoint", field="checkpoint_content_sha256", required=True)
    state = str(checkpoint.get("state", checkpoint.get("status", ""))).lower()
    if state not in {"complete", "completed"}:
        raise AutomationError(f"{item['display']} checkpoint is not complete")
    if checkpoint.get("records") is not None and int(checkpoint["records"]) != n_eval:
        raise AutomationError(f"{item['display']} checkpoint record count mismatch")
    if checkpoint.get("records_sha256") and _sha(records_path) != checkpoint["records_sha256"]:
        raise AutomationError(f"{item['display']} checkpoint records hash mismatch")
    _verify_artifacts(output)
    return {
        "slug": item["slug"],
        "display": item["display"],
        "n_eval": n_eval,
        "j_eval": j_eval,
        "output": str(output),
        "manifest": str(manifest_path),
        "roster": str(roster_path),
        "records": str(records_path),
        "runtime_runner_hashes": sorted(observed_runner_hashes),
        "runtime_core_hashes": sorted(observed_core_hashes),
        "summary": str(summary_path),
        "aggregate": aggregate,
        "state_records": state_info,
        "source_audit": source_audit,
    }


def _receipt_entries(root: Path) -> dict[str, Mapping[str, Any]]:
    candidates = (root / "full_release_receipt.json", root / "receipt.json")
    path = next((candidate for candidate in candidates if candidate.is_file()), None)
    if path is None:
        raise AutomationError("missing full-release receipt")
    receipt = _json(path)
    if not isinstance(receipt, Mapping):
        raise AutomationError("combined receipt is not an object")
    if "receipt_content_sha256" in receipt:
        _check_self_hash(receipt, "combined receipt", field="receipt_content_sha256", required=True)
    declared_root = receipt.get("root")
    if declared_root and Path(str(declared_root)).resolve() != root.resolve():
        raise AutomationError("combined receipt root mismatch")
    raw = receipt.get("methods", receipt.get("datasets", receipt.get("entries")))
    if isinstance(raw, Mapping):
        raw = list(raw.values())
    if not isinstance(raw, list):
        raise AutomationError("combined receipt has no methods/datasets list")
    entries: dict[str, Mapping[str, Any]] = {}
    for entry in raw:
        if not isinstance(entry, Mapping):
            raise AutomationError("combined receipt entry is not an object")
        slug = str(entry.get("slug") or entry.get("dataset_id") or "").lower().replace(" ", "-")
        if slug not in DATASET_BY_SLUG or slug in entries:
            raise AutomationError(f"invalid or duplicate receipt slug: {slug}")
        entries[slug] = entry
    if set(entries) != set(DATASET_BY_SLUG):
        raise AutomationError("combined receipt does not contain exactly eight datasets")
    global_bindings = _binding_map(receipt.get("source_bindings"), "combined receipt")
    # The combined receipt must expose both user-provided upstream sources;
    # per-dataset checks below bind them to the corresponding run manifests.
    parts = _binding_named(global_bindings, "parts_zip", "combined receipt")
    _verify_hashed_binding(parts, "combined receipt", required=True)
    source = _binding_named(global_bindings, "source_root", "combined receipt")
    source_path = _verify_hashed_binding(source, "combined receipt")
    if not source_path.is_dir() or not (source_path / "urdf").is_dir():
        raise AutomationError("combined receipt source_root has no urdf directory")
    archive = _binding_named(
        global_bindings,
        "infinigen_archive_validation_receipt",
        "combined receipt",
    )
    archive_path = _verify_hashed_binding(archive, "combined receipt", required=True)
    archive_value = _json(archive_path)
    if not isinstance(archive_value, Mapping):
        raise AutomationError("combined receipt archive validation receipt is not an object")
    if Path(str(archive_value.get("archive_root", ""))).resolve() != (source_path / "urdf").resolve():
        raise AutomationError("combined receipt archive root is not bound to source_root/urdf")
    return entries


def _fraction(text: str) -> tuple[int, int] | None:
    match = re.search(r"(?<![\w=])([0-9][0-9,]*)\s*/\s*([0-9][0-9,]*)(?![\w])", text)
    if not match:
        return None
    return int(match.group(1).replace(",", "")), int(match.group(2).replace(",", ""))


def _markdown_section(text: str) -> str:
    match = re.search(r"^#{2,3}\s+Table 4\.\s+Collision and Mechanical Clearance.*$", text, re.MULTILINE | re.IGNORECASE)
    if not match:
        raise AutomationError("Markdown is missing Table 4 heading")
    tail = text[match.end():]
    end = re.search(r"^#{1,4}\s+", tail, re.MULTILINE)
    return text[match.start(): match.end() + (end.start() if end else len(tail))]


def _markdown_rows(section: str) -> dict[str, list[str]]:
    rows: dict[str, list[str]] = {}
    for line in section.splitlines():
        if not line.strip().startswith("|") or re.match(r"^\|\s*:?-{2,}", line.strip()):
            continue
        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if cells and cells[0] and not cells[0].lower().startswith("dataset"):
            rows[cells[0]] = cells
    return rows


def validate_markdown(markdown_path: Path, results: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    text = Path(markdown_path).read_text(encoding="utf-8")
    rows = _markdown_rows(_markdown_section(text))
    for item in DATASETS:
        label = item["display"]
        row = rows.get(label)
        if row is None:
            aliases = item.get("aliases", ())
            row = next((rows.get(alias) for alias in aliases if rows.get(alias) is not None), None)
        if row is None:
            raise AutomationError(f"Markdown Table 4 is missing comparison row: {label}")
        joined = " | ".join(row)
        if re.search(r"(?:\bn\s*=\s*800\b|\bj\s*=\s*800\b|/\s*800\b)", joined, re.IGNORECASE):
            raise AutomationError(f"comparison row contains historical N=800: {label}")
        if str(item["n_eval"]) not in joined.replace(",", "") or str(item["j_eval"]) not in joined.replace(",", ""):
            # A compact renderer may omit N/J; in that case retain the row
            # check, while a row that explicitly includes either denominator
            # must be correct.
            # N/E is a metric placeholder, not an explicit N_eval field.
            # Only enforce a denominator check when the row actually labels
            # an N/J value (``N_eval``, ``J_eval``, ``N=``, or ``J=``).
            if re.search(r"\bN_eval\b|\bJ_eval\b|\bN\s*=|\bJ\s*=", joined, re.IGNORECASE):
                raise AutomationError(f"Markdown N/J mismatch: {label}")
        metrics = results[item["slug"]].get("aggregate", {}).get("metrics", {})
        if not isinstance(metrics, Mapping):
            continue
        # Compare every published fraction that the renderer exposes.  N/E is
        # checked explicitly for blocked/no-geometry cohorts.
        for key in ("rest_all_pair_cf", "rest_non_adjacent_cf", "single_joint_sweep_cf", "multi_joint_sobol_cf", "collision_state_rate", "collision_free_range", "strict_collision_pass"):
            value = metrics.get(key)
            if not isinstance(value, Mapping):
                continue
            expected = _metric_pair(value)
            if str(value.get("status", "")).upper() in {"N/E", "NE"}:
                if not re.search(r"\bN\s*/?\s*E\b", joined, re.IGNORECASE):
                    raise AutomationError(f"Markdown must publish N/E for {label} {key}")
            elif expected is not None and expected[1] > 0:
                # A row can have columns in either historical or extended
                # order; requiring the fraction somewhere prevents stale
                # numbers without hard-coding column positions.
                needle = rf"{expected[0]:,}?\s*/\s*{expected[1]:,}?"
                if not re.search(needle, joined):
                    compact = f"{expected[0]}/{expected[1]}"
                    if compact not in joined.replace(",", ""):
                        raise AutomationError(f"Markdown metric mismatch: {label} {key}")
    if not any(label.startswith("Ours-500K") for label in rows):
        raise AutomationError("Markdown Table 4 is missing preserved Ours-500K row")
    return {"comparison_rows": len(DATASETS), "ours_row_present": True}


def run_checks(
    root: Path,
    markdown: Path | None = None,
    *,
    run_pytest: bool = False,
    repo_root: Path | None = None,
) -> dict[str, Any]:
    root = Path(root).resolve()
    errors: list[str] = []
    results: dict[str, dict[str, Any]] = {}
    try:
        entries = _receipt_entries(root)
    except AutomationError as exc:
        entries = {}
        errors.append(str(exc))
    for item in DATASETS:
        try:
            results[item["slug"]] = _verify_dataset(root, item, entries.get(item["slug"], {}))
        except AutomationError as exc:
            errors.append(str(exc))
    markdown_result: dict[str, Any] = {}
    if markdown is not None and not errors:
        try:
            markdown_result = validate_markdown(Path(markdown), results)
        except (AutomationError, OSError) as exc:
            errors.append(str(exc))
    pytest_result = None
    if run_pytest:
        repo = Path(repo_root or Path(__file__).resolve().parents[2]).resolve()
        env = dict(os.environ)
        env["PYTHONPATH"] = str(repo / "exp" / "scripts") + os.pathsep + env.get("PYTHONPATH", "")
        configured = os.environ.get("TABLE4_PYTHON")
        fallback = repo / "exp" / ".venv_low_medium" / "bin" / "python"
        python_exe = Path(configured).expanduser() if configured else (fallback if fallback.is_file() else Path(sys.executable))
        command = [
            str(python_exe), "-m", "pytest", "-q",
            "exp/tests/test_run_table4_full_release.py",
            "exp/tests/test_table4_full_release.py",
            "exp/tests/test_render_table4_full_release_results.py",
            "exp/tests/test_update_table4_full_release_markdown.py",
        ]
        proc = subprocess.run(command, cwd=repo, text=True, capture_output=True, env=env, check=False)
        pytest_result = {"returncode": proc.returncode, "passed": proc.returncode == 0, "stdout_tail": proc.stdout[-4000:], "stderr_tail": proc.stderr[-4000:]}
        if proc.returncode:
            errors.append("focused Table 4 contract pytest failed")
    return {
        "schema_version": "table4_full_release_automation_check_v1",
        "root": str(root),
        "markdown_path": str(Path(markdown).resolve()) if markdown else None,
        "dataset_count": len(results),
        "datasets": results,
        "markdown": markdown_result,
        "pytest": pytest_result,
        "errors": errors,
        "all_pass": not errors and len(results) == len(DATASETS),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--markdown", type=Path)
    parser.add_argument("--pytest", action="store_true")
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args(argv)
    report = run_checks(args.root, args.markdown, run_pytest=args.pytest)
    payload = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if report["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
