#!/usr/bin/env python3
"""Read-only paired/cohort audit for Ours-500K and the PV-A v1 Table 4 run.

The audit deliberately compares two different views:

* the 500 exact Ours-500K frozen items, paired to the current PV-A release by
  exact ``(raw_category, integerized source_asset_id)`` identity; and
* the complete current PV-A release, split into the 12 Ours-500K categories
  and every remaining category.

All inputs are verified before a result is emitted.  SQLite is opened through
an immutable ``mode=ro`` URI, roster row hashes and result identities are
replayed, and the database bytes are checked against the full-release receipt.
The emitted JSON is canonical and carries a content self-hash.
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
import sqlite3
import sys
import tempfile
from typing import Any, Iterable, Mapping
import xml.etree.ElementTree as ET
import zlib

SCRIPT = Path(__file__).resolve()
EXP = SCRIPT.parent.parent
if str(SCRIPT.parent) not in sys.path:
    sys.path.insert(0, str(SCRIPT.parent))

import run_urdf_table4_partnet_mobility as table4_core
DEFAULT_OURS_ARTIFACT = (
    EXP / "runtime/urdf_table4_ours_500k_table2_n500_20260819T104011Z"
)
DEFAULT_PVA_EVALUATION = (
    EXP / "runtime/pva_table1234_full_release_20260826/evaluation"
)
DEFAULT_OUTPUT = EXP / "runtime/ours500_pva_table4_cohort_audit_20260827.json"

AUDIT_SCHEMA = "ours500_pva_table4_label_matched_cohort_audit_v2"
OURS_MANIFEST_SCHEMA = "table4_ours_500k_frozen_manifest_v1"
PVA_MANIFEST_SCHEMA = "pva_table1234_full_release_run_v1"
PVA_RECEIPT_SCHEMA = "pva_table1234_full_release_receipt_v1"
PVA_DATABASE_SCHEMA = "pva_table1234_results_db_v1"
OURS_PROTOCOL_ID = "urdf_sim_ready_table4_ours_500k_table2_n500_v1"
PVA_PROTOCOL_ID = "urdf_sim_ready_table4_pva_full_release_v1"
SEED_PATTERN = re.compile(r"^seed_(\d+)$")
PVA_SUCCESSFUL_WORKER_STATUSES = frozenset({"completed", "recovered"})
PVA_RETAINED_FAILURE_WORKER_STATUSES = frozenset({"parent_error", "timeout"})

OURS_FROZEN_INPUT_FIELDS = (
    "protocol_id",
    "order",
    "dataset_id",
    "asset_id",
    "raw_category",
    "seed_name",
    "asset_root_relpath",
    "selection_rank",
    "category",
    "package_audit_success",
    "audit_issue",
    "primary_urdf_relpath",
    "urdf_sha256",
    "valid_tree",
    "movable_dof_count",
    "range_evaluable_dof_count",
    "joint_specs_sha256",
    "collision_mesh_inventory_sha256",
    "missing_collision_mesh_reference_count",
    "unsafe_collision_mesh_reference_count",
    "scale_derivation_sha256",
    "object_bbox_diagonal_m",
    "rest_state_expected",
    "single_state_expected",
    "sobol_state_expected",
)


class CohortAuditError(ValueError):
    """Raised when an input cannot support the requested comparison."""


def canonical_text(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    )


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_text(value).encode("utf-8")).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _regular_file(path: Path, label: str) -> Path:
    candidate = Path(path)
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as error:
        raise CohortAuditError(f"{label} is unavailable: {candidate}: {error}") from error
    if not resolved.is_file() or candidate.is_symlink():
        raise CohortAuditError(f"{label} must be a regular non-symlink file: {candidate}")
    return resolved


def _load_json(path: Path, label: str) -> tuple[dict[str, Any], str]:
    resolved = _regular_file(path, label)
    try:
        value = json.loads(resolved.read_text(encoding="utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise CohortAuditError(f"{label} is not valid JSON: {error}") from error
    if not isinstance(value, dict):
        raise CohortAuditError(f"{label} must contain a JSON object")
    return value, sha256_file(resolved)


def _verify_self_hash(value: Mapping[str, Any], field: str, label: str) -> str:
    declared = value.get(field)
    if not isinstance(declared, str) or not re.fullmatch(r"[0-9a-f]{64}", declared):
        raise CohortAuditError(f"{label} has no valid {field}")
    payload = dict(value)
    payload.pop(field, None)
    calculated = canonical_sha256(payload)
    if calculated != declared:
        raise CohortAuditError(
            f"{label} {field} mismatch: declared {declared}, calculated {calculated}"
        )
    return declared


def _required(mapping: Mapping[str, Any], path: str, label: str) -> Any:
    current: Any = mapping
    for component in path.split("."):
        if not isinstance(current, Mapping) or component not in current:
            raise CohortAuditError(f"{label} is missing {path}")
        current = current[component]
    return current


def _integer(value: Any, label: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise CohortAuditError(f"{label} must be an integer >= {minimum}")
    return value


def _nonempty_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise CohortAuditError(f"{label} must be a non-empty string")
    return value


def _strict_bool(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise CohortAuditError(f"{label} must be boolean")
    return value


def _fail_closed_strict_pass(result: Mapping[str, Any], label: str) -> bool:
    """Replay the formal aggregator's retained-failure convention."""

    value = result.get("strict_collision_pass")
    if isinstance(value, bool):
        return value
    if value is None and result.get("status") == "error" and result.get(
        "measurement_complete"
    ) is False:
        return False
    raise CohortAuditError(
        f"{label}.strict_collision_pass must be boolean, or null on a retained error"
    )


def _seed_number(value: Any, label: str) -> int:
    text = _nonempty_string(value, label)
    match = SEED_PATTERN.fullmatch(text)
    if match is None:
        raise CohortAuditError(f"{label} is not a seed_<integer> identity: {text}")
    return int(match.group(1))


def _rate(passed: int, denominator: int) -> float:
    if denominator <= 0:
        raise CohortAuditError("cannot calculate a rate with an empty denominator")
    return passed / denominator


def _metric(passed: int, denominator: int) -> dict[str, Any]:
    return {
        "denominator": denominator,
        "passed": passed,
        "rate": _rate(passed, denominator),
    }


def _finite_number(value: Any, label: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CohortAuditError(f"{label} must be a finite number")
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        raise CohortAuditError(f"{label} is outside its valid numeric domain")
    return result


def _ours_v1_schedule(urdf: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    # The frozen v1 parser predates the v2 ``mimic`` metadata field.  The
    # remaining parser and sampling functions are byte-for-byte compatible.
    joints = [
        {key: value for key, value in row.items() if key != "mimic"}
        for row in table4_core.parse_urdf_joints(urdf)
    ]
    rest = [0.0] * len(joints)
    schedule: list[dict[str, Any]] = [{
        "phase": "rest", "sample_index": 0, "joint_name": None,
        "joint_values_sha256": canonical_sha256(rest),
    }]
    for position, joint in enumerate(joints):
        if not joint["range_evaluable"]:
            continue
        for sample_index, value in enumerate(table4_core.single_joint_values(joint)):
            values = list(rest)
            values[position] = value
            schedule.append({
                "phase": "single_joint_sweep", "sample_index": sample_index,
                "joint_name": str(joint["name"]),
                "joint_values_sha256": canonical_sha256(values),
            })
    if joints and all(joint["range_evaluable"] for joint in joints):
        for sample_index, values in enumerate(table4_core.sobol_joint_values(joints)):
            schedule.append({
                "phase": "multi_joint_sobol", "sample_index": sample_index,
                "joint_name": None, "joint_values_sha256": canonical_sha256(values),
            })
    return joints, schedule


def _recompute_ours_record(
    record: Mapping[str, Any],
    item: Mapping[str, Any],
    threshold: float,
    reset_tolerance: float,
    schedule: list[dict[str, Any]],
    label: str,
) -> dict[str, Any]:
    load_success = _strict_bool(record.get("load_success"), f"{label}.load_success")
    child_timed_out = _strict_bool(
        record.get("child_timed_out"), f"{label}.child_timed_out"
    )
    child_returncode = record.get("child_returncode")
    if child_returncode is not None:
        child_returncode = _integer(
            child_returncode, f"{label}.child_returncode", minimum=0
        )
    issues = record.get("issues")
    if not isinstance(issues, list) or any(not isinstance(issue, str) for issue in issues):
        raise CohortAuditError(f"{label}.issues must be a list of strings")
    execution_success = bool(
        load_success
        and not child_timed_out
        and child_returncode in (None, 0)
        and not issues
    )
    if not execution_success:
        raise CohortAuditError(f"{label} has an unsuccessful load/child/worker execution")

    states = record.get("state_records")
    if not isinstance(states, list):
        raise CohortAuditError(f"{label}.state_records must be a list")
    declared_state_hash = record.get("state_records_sha256")
    if declared_state_hash != canonical_sha256(states):
        raise CohortAuditError(f"{label} state_records_sha256 mismatch")
    if len(states) != len(schedule):
        raise CohortAuditError(f"{label} independently generated schedule length mismatch")

    phase_rows: dict[str, list[Mapping[str, Any]]] = {
        "rest": [],
        "single_joint_sweep": [],
        "multi_joint_sobol": [],
    }
    collision_states = 0
    single_free_states = 0
    max_penetration = 0.0
    max_reset_error = 0.0
    for index, raw_state in enumerate(states):
        if not isinstance(raw_state, Mapping):
            raise CohortAuditError(f"{label} state {index} is not an object")
        state = raw_state
        for field, expected in schedule[index].items():
            if state.get(field) != expected:
                raise CohortAuditError(
                    f"{label} state {index} independently generated schedule mismatch: {field}"
                )
        for field, expected in (
            ("protocol_id", item["protocol_id"]),
            ("order", item["order"]),
            ("dataset_id", item["dataset_id"]),
            ("category", item["category"]),
            ("input_identity_sha256", item["input_identity_sha256"]),
        ):
            if state.get(field) != expected:
                raise CohortAuditError(f"{label} state {index} {field} binding mismatch")
        phase = state.get("phase")
        if phase not in phase_rows:
            raise CohortAuditError(f"{label} state {index} has invalid phase: {phase}")
        sample_index = _integer(
            state.get("sample_index"), f"{label} state {index}.sample_index"
        )
        nonadj_count = _integer(
            state.get("non_adjacent_illegal_penetration_count"),
            f"{label} state {index}.non_adjacent_illegal_penetration_count",
        )
        nonadj_max = _finite_number(
            state.get("non_adjacent_max_penetration_m"),
            f"{label} state {index}.non_adjacent_max_penetration_m",
            minimum=0.0,
        )
        all_pair_count = _integer(
            state.get("all_pair_illegal_penetration_count"),
            f"{label} state {index}.all_pair_illegal_penetration_count",
        )
        all_pair_max = _finite_number(
            state.get("all_pair_max_penetration_m"),
            f"{label} state {index}.all_pair_max_penetration_m",
            minimum=0.0,
        )
        nonadj_free = nonadj_max <= threshold
        all_pair_free = all_pair_max <= threshold
        if nonadj_free != (nonadj_count == 0):
            raise CohortAuditError(
                f"{label} state {index} non-adjacent threshold/count mismatch"
            )
        if all_pair_free != (all_pair_count == 0):
            raise CohortAuditError(
                f"{label} state {index} all-pair threshold/count mismatch"
            )
        metric_max = _finite_number(
            state.get("metric_max_penetration_m"),
            f"{label} state {index}.metric_max_penetration_m",
            minimum=0.0,
        )
        expected_metric_max = all_pair_max if phase == "rest" else nonadj_max
        if metric_max != expected_metric_max:
            raise CohortAuditError(
                f"{label} state {index} phase-selected metric penetration mismatch"
            )
        reset_error = _finite_number(
            state.get("reset_readback_max_abs_error"),
            f"{label} state {index}.reset_readback_max_abs_error",
            minimum=0.0,
        )
        if reset_error > reset_tolerance:
            raise CohortAuditError(
                f"{label} state {index} reset readback exceeds frozen tolerance"
            )
        phase_rows[str(phase)].append(state)
        collision_states += int(not nonadj_free)
        single_free_states += int(phase == "single_joint_sweep" and nonadj_free)
        max_penetration = max(max_penetration, metric_max)
        max_reset_error = max(max_reset_error, reset_error)
        if sample_index < 0:  # pragma: no cover - guarded by _integer
            raise CohortAuditError(f"{label} state {index} has a negative sample index")

    rest = phase_rows["rest"]
    single = phase_rows["single_joint_sweep"]
    sobol = phase_rows["multi_joint_sobol"]
    expected_rest = int(item["rest_state_expected"])
    expected_single = int(item["single_state_expected"])
    expected_sobol = int(item["sobol_state_expected"])
    if len(rest) != expected_rest or {
        int(row["sample_index"]) for row in rest
    } != set(range(expected_rest)):
        raise CohortAuditError(f"{label} rest-state phase closure mismatch")
    if any(row.get("joint_name") is not None for row in rest):
        raise CohortAuditError(f"{label} rest-state joint_name must be null")

    samples_per_joint: dict[str, set[int]] = defaultdict(set)
    for row in single:
        joint_name = _nonempty_string(row.get("joint_name"), f"{label} single joint")
        sample_index = int(row["sample_index"])
        if sample_index in samples_per_joint[joint_name]:
            raise CohortAuditError(f"{label} duplicate single-joint sample")
        samples_per_joint[joint_name].add(sample_index)
    if len(single) != expected_single:
        raise CohortAuditError(f"{label} single-state phase closure mismatch")
    if any(samples != set(range(21)) for samples in samples_per_joint.values()):
        raise CohortAuditError(f"{label} single-joint sample-index closure mismatch")
    if len(samples_per_joint) != int(item["range_evaluable_dof_count"]):
        raise CohortAuditError(f"{label} single-joint identity closure mismatch")

    if len(sobol) != expected_sobol or {
        int(row["sample_index"]) for row in sobol
    } != set(range(expected_sobol)):
        raise CohortAuditError(f"{label} Sobol-state phase closure mismatch")
    if any(row.get("joint_name") is not None for row in sobol):
        raise CohortAuditError(f"{label} Sobol joint_name must be null")

    rest_all_pair_cf = all(
        float(row["all_pair_max_penetration_m"]) <= threshold for row in rest
    )
    rest_nonadj_cf = all(
        float(row["non_adjacent_max_penetration_m"]) <= threshold for row in rest
    )
    single_cf = len(single) == expected_single and all(
        float(row["non_adjacent_max_penetration_m"]) <= threshold for row in single
    )
    sobol_cf = len(sobol) == expected_sobol and all(
        float(row["non_adjacent_max_penetration_m"]) <= threshold for row in sobol
    )
    measurement_complete = bool(
        execution_success
        and max_reset_error <= reset_tolerance
        and int(item["range_evaluable_dof_count"]) == int(item["movable_dof_count"])
        and len(rest) == expected_rest
        and len(single) == expected_single
        and len(sobol) == expected_sobol
    )
    strict = bool(measurement_complete and rest_nonadj_cf and single_cf and sobol_cf)
    joint_passed = sum(
        all(
            float(row["non_adjacent_max_penetration_m"]) <= threshold
            for row in single
            if row.get("joint_name") == joint_name
        )
        for joint_name in samples_per_joint
    )
    recomputed = {
        "rest_state_executed": len(rest),
        "single_state_executed": len(single),
        "sobol_state_executed": len(sobol),
        "rest_non_adjacent_free": int(rest_nonadj_cf),
        "single_non_adjacent_free": single_free_states,
        "sobol_non_adjacent_free": sum(
            float(row["non_adjacent_max_penetration_m"]) <= threshold for row in sobol
        ),
        "joint_single_sweep_cf_passed": joint_passed,
        "rest_all_pair_cf": rest_all_pair_cf,
        "rest_non_adjacent_cf": rest_nonadj_cf,
        "single_joint_sweep_cf": single_cf,
        "multi_joint_sobol_cf": sobol_cf,
        "measurement_complete": measurement_complete,
        "strict_collision_pass": strict,
        "collision_states": collision_states,
        "single_free_states": single_free_states,
        "expected_states": expected_rest + expected_single + expected_sobol,
        "executed_states": len(states),
        "max_penetration_m": max_penetration,
        "max_reset_readback_error": max_reset_error,
        "execution_success": execution_success,
        "reset_tolerance_satisfied": max_reset_error <= reset_tolerance,
    }
    for field in (
        "rest_state_executed",
        "single_state_executed",
        "sobol_state_executed",
        "rest_non_adjacent_free",
        "single_non_adjacent_free",
        "sobol_non_adjacent_free",
        "joint_single_sweep_cf_passed",
        "rest_all_pair_cf",
        "rest_non_adjacent_cf",
        "single_joint_sweep_cf",
        "multi_joint_sobol_cf",
        "measurement_complete",
        "strict_collision_pass",
        "max_penetration_m",
        "max_reset_readback_error",
    ):
        if record.get(field) != recomputed[field]:
            raise CohortAuditError(f"{label} published field {field} mismatch")
    scale = _finite_number(
        record.get("object_bbox_diagonal_m"), f"{label}.object_bbox_diagonal_m",
        minimum=0.0,
    )
    expected_normalized = max_penetration / scale if scale > 0 else None
    if record.get("max_penetration_normalized") != expected_normalized:
        raise CohortAuditError(f"{label} published normalized penetration mismatch")
    recomputed["max_penetration_normalized"] = expected_normalized
    return recomputed


def _read_ours(
    manifest_path: Path,
    records_path: Path,
) -> tuple[
    dict[str, Any],
    dict[tuple[str, int], dict[str, Any]],
    set[str],
    dict[str, Any],
]:
    manifest, manifest_file_sha = _load_json(
        manifest_path, "Ours-500K frozen manifest"
    )
    if manifest.get("schema_version") != OURS_MANIFEST_SCHEMA:
        raise CohortAuditError("unexpected Ours-500K frozen manifest schema")
    if manifest.get("protocol_id") != OURS_PROTOCOL_ID:
        raise CohortAuditError("unexpected Ours-500K protocol_id")
    manifest_content_sha = _verify_self_hash(
        manifest, "manifest_content_sha256", "Ours-500K frozen manifest"
    )
    items = manifest.get("items")
    if not isinstance(items, list) or not items:
        raise CohortAuditError("Ours-500K frozen manifest items are empty")

    by_order: dict[int, dict[str, Any]] = {}
    identities: list[dict[str, Any]] = []
    asset_ids: list[str] = []
    manifest_pairs: set[tuple[str, int]] = set()
    for index, raw_item in enumerate(items):
        if not isinstance(raw_item, dict):
            raise CohortAuditError(f"Ours-500K item {index} is not an object")
        item = raw_item
        order = _integer(item.get("order"), f"Ours-500K item {index}.order")
        if order != index or order in by_order:
            raise CohortAuditError(
                f"Ours-500K item order is not unique and contiguous at {index}"
            )
        missing = [field for field in OURS_FROZEN_INPUT_FIELDS if field not in item]
        if missing:
            raise CohortAuditError(
                f"Ours-500K item {index} is missing frozen fields: {', '.join(missing)}"
            )
        identity = {field: item[field] for field in OURS_FROZEN_INPUT_FIELDS}
        expected_identity = canonical_sha256(identity)
        if item.get("input_identity_sha256") != expected_identity:
            raise CohortAuditError(
                f"Ours-500K item {index} input_identity_sha256 mismatch"
            )
        category = _nonempty_string(
            item.get("raw_category"), f"Ours-500K item {index}.raw_category"
        )
        seed_name = _nonempty_string(
            item.get("seed_name"), f"Ours-500K item {index}.seed_name"
        )
        seed = _seed_number(seed_name, f"Ours-500K item {index}.seed_name")
        asset_id = _nonempty_string(
            item.get("asset_id"), f"Ours-500K item {index}.asset_id"
        )
        if item.get("dataset_id") != asset_id or item.get("category") != category:
            raise CohortAuditError(f"Ours-500K item {index} identity aliases disagree")
        if asset_id != f"{category}/{seed_name}":
            raise CohortAuditError(f"Ours-500K item {index} asset_id is non-canonical")
        pair = (category, seed)
        if pair in manifest_pairs:
            raise CohortAuditError(f"duplicate Ours-500K category/seed pair: {pair}")
        manifest_pairs.add(pair)
        by_order[order] = item
        identities.append(identity)
        asset_ids.append(asset_id)

    declared_n = _integer(
        _required(manifest, "source.n_eval", "Ours-500K frozen manifest"),
        "Ours-500K source.n_eval",
        minimum=1,
    )
    if declared_n != len(items) or _required(
        manifest, "source.n_release", "Ours-500K frozen manifest"
    ) != len(items):
        raise CohortAuditError("Ours-500K manifest item/release counts disagree")
    categories = {pair[0] for pair in manifest_pairs}
    if _required(manifest, "source.category_count", "Ours-500K frozen manifest") != len(
        categories
    ):
        raise CohortAuditError("Ours-500K manifest category_count disagrees")
    if _required(
        manifest, "selection.selected_asset_ids_sha256", "Ours-500K frozen manifest"
    ) != canonical_sha256(asset_ids):
        raise CohortAuditError("Ours-500K selected asset identity list hash mismatch")
    if _required(
        manifest, "selection.ordered_identities_sha256", "Ours-500K frozen manifest"
    ) != canonical_sha256(identities):
        raise CohortAuditError("Ours-500K ordered frozen identity hash mismatch")

    dataset_root_text = _nonempty_string(
        _required(manifest, "source.dataset_root", "Ours-500K frozen manifest"),
        "Ours-500K source.dataset_root",
    )
    dataset_root_input = Path(dataset_root_text)
    try:
        dataset_root = dataset_root_input.resolve(strict=True)
    except OSError as error:
        raise CohortAuditError(
            f"Ours-500K dataset_root is unavailable: {dataset_root_input}: {error}"
        ) from error
    if not dataset_root.is_dir() or dataset_root_input.is_symlink():
        raise CohortAuditError(
            "Ours-500K dataset_root must be a regular non-symlink directory"
        )

    mimic_joint_count = 0
    mimic_asset_count = 0
    schedules_by_order: dict[int, list[dict[str, Any]]] = {}
    for order in range(len(items)):
        item = by_order[order]
        relative_text = _nonempty_string(
            item.get("primary_urdf_relpath"),
            f"Ours-500K item {order}.primary_urdf_relpath",
        )
        relative = Path(relative_text)
        if relative.is_absolute() or ".." in relative.parts:
            raise CohortAuditError(
                f"Ours-500K item {order} primary URDF path is unsafe: {relative_text}"
            )
        candidate = dataset_root / relative
        urdf = _regular_file(candidate, f"Ours-500K item {order} primary URDF")
        try:
            urdf.relative_to(dataset_root)
        except ValueError as error:
            raise CohortAuditError(
                f"Ours-500K item {order} primary URDF escapes dataset_root"
            ) from error
        observed_sha = sha256_file(urdf)
        if observed_sha != item.get("urdf_sha256"):
            raise CohortAuditError(
                f"Ours-500K item {order} primary URDF SHA256 mismatch"
            )
        try:
            xml_root = ET.parse(urdf).getroot()
        except (ET.ParseError, OSError) as error:
            raise CohortAuditError(
                f"Ours-500K item {order} primary URDF XML parse failed: {error}"
            ) from error
        joints = xml_root.findall(".//joint")
        item_mimic_count = sum(joint.find("mimic") is not None for joint in joints)
        mimic_joint_count += item_mimic_count
        mimic_asset_count += int(item_mimic_count > 0)
        parsed_joints, schedule = _ours_v1_schedule(urdf)
        if canonical_sha256(parsed_joints) != item.get("joint_specs_sha256"):
            raise CohortAuditError(f"Ours-500K item {order} parsed joint specification mismatch")
        if len(parsed_joints) != item.get("movable_dof_count"):
            raise CohortAuditError(f"Ours-500K item {order} parsed joint count mismatch")
        schedules_by_order[order] = schedule

    resolved_records = _regular_file(records_path, "Ours-500K asset records")
    raw_digest = hashlib.sha256()
    records_by_pair: dict[tuple[str, int], dict[str, Any]] = {}
    states_by_order: dict[int, list[dict[str, Any]]] = {}
    aggregate = defaultdict(int)
    maximum_normalized = 0.0
    seen_orders: set[int] = set()
    line_count = 0
    threshold = _finite_number(
        _required(
            manifest,
            "evaluation.penetration_threshold_m",
            "Ours-500K frozen manifest",
        ),
        "Ours-500K penetration threshold",
        minimum=0.0,
    )
    reset_tolerance = _finite_number(
        _required(manifest, "evaluation.reset_tolerance", "Ours-500K frozen manifest"),
        "Ours-500K reset tolerance",
        minimum=0.0,
    )
    with resolved_records.open("rb") as stream:
        for line_count, raw_line in enumerate(stream, start=1):
            raw_digest.update(raw_line)
            if not raw_line.strip():
                raise CohortAuditError(
                    f"Ours-500K asset records row {line_count} is blank"
                )
            try:
                record = json.loads(raw_line)
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                raise CohortAuditError(
                    f"Ours-500K asset records row {line_count} is invalid JSON: {error}"
                ) from error
            if not isinstance(record, dict):
                raise CohortAuditError(
                    f"Ours-500K asset records row {line_count} is not an object"
                )
            order = _integer(
                record.get("order"), f"Ours-500K record {line_count}.order"
            )
            if order in seen_orders:
                raise CohortAuditError(f"duplicate Ours-500K record order: {order}")
            seen_orders.add(order)
            item = by_order.get(order)
            if item is None:
                raise CohortAuditError(f"unknown Ours-500K record order: {order}")
            for field, expected in (
                ("protocol_id", item["protocol_id"]),
                ("dataset_id", item["dataset_id"]),
                ("category", item["category"]),
                ("input_identity_sha256", item["input_identity_sha256"]),
                ("movable_dof_count", item["movable_dof_count"]),
                ("range_evaluable_dof_count", item["range_evaluable_dof_count"]),
                ("rest_state_expected", item["rest_state_expected"]),
                ("single_state_expected", item["single_state_expected"]),
                ("sobol_state_expected", item["sobol_state_expected"]),
            ):
                if record.get(field) != expected:
                    raise CohortAuditError(
                        f"Ours-500K record {line_count} field {field} is not bound to its frozen item"
                    )
            if record.get("collision_core_sha256") != _required(
                manifest, "evaluation.core_sha256", "Ours-500K frozen manifest"
            ):
                raise CohortAuditError(
                    f"Ours-500K record {line_count} collision core binding mismatch"
                )
            if record.get("runner_sha256") != _required(
                manifest, "evaluation.adapter_sha256", "Ours-500K frozen manifest"
            ):
                raise CohortAuditError(
                    f"Ours-500K record {line_count} runner binding mismatch"
                )
            replay = _recompute_ours_record(
                record,
                item,
                threshold,
                reset_tolerance,
                schedules_by_order[order],
                f"Ours-500K record {line_count}",
            )
            strict = bool(replay["strict_collision_pass"])
            states_by_order[order] = [dict(state) for state in record["state_records"]]
            for field in (
                "rest_all_pair_cf",
                "rest_non_adjacent_cf",
                "single_joint_sweep_cf",
                "multi_joint_sobol_cf",
                "strict_collision_pass",
                "measurement_complete",
                "collision_states",
                "single_free_states",
                "expected_states",
                "executed_states",
            ):
                aggregate[field] += int(replay[field])
            normalized = replay["max_penetration_normalized"]
            if normalized is not None:
                aggregate["observed_normalized"] += 1
                maximum_normalized = max(maximum_normalized, float(normalized))
            seed = _seed_number(
                item["seed_name"], f"Ours-500K item {order}.seed_name"
            )
            pair = (str(item["raw_category"]), seed)
            if pair in records_by_pair:
                raise CohortAuditError(f"duplicate Ours-500K result pair: {pair}")
            records_by_pair[pair] = {
                "passed": strict,
                "urdf_sha256": item["urdf_sha256"],
                "joint_count": item["movable_dof_count"],
            }

    if line_count != len(items) or seen_orders != set(by_order):
        missing = sorted(set(by_order) - seen_orders)
        raise CohortAuditError(
            f"Ours-500K result closure mismatch: rows={line_count}, expected={len(items)}, missing_orders={missing[:10]}"
        )

    artifact_root = Path(manifest_path).resolve().parent
    external_states = _regular_file(
        artifact_root / "state_records.jsonl", "Ours-500K external state records"
    )
    replayed_state_digest = hashlib.sha256()
    replayed_state_count = 0
    for order in range(len(items)):
        for state in states_by_order[order]:
            replayed_state_digest.update(
                (json.dumps(state, sort_keys=True, ensure_ascii=True) + "\n").encode(
                    "utf-8"
                )
            )
            replayed_state_count += 1
    external_state_sha = sha256_file(external_states)
    if external_state_sha != replayed_state_digest.hexdigest():
        raise CohortAuditError(
            "Ours-500K external state-record artifact does not match embedded states"
        )

    summary_path = artifact_root / "summary.json"
    verification_path = artifact_root / "verification.json"
    summary, summary_sha = _load_json(summary_path, "Ours-500K formal summary")
    verification, verification_sha = _load_json(
        verification_path, "Ours-500K formal verification"
    )
    if (
        summary.get("protocol_id") != OURS_PROTOCOL_ID
        or summary.get("manifest_content_sha256") != manifest_content_sha
        or summary.get("status") != "COMPLETE"
        or summary.get("n_eval") != len(items)
        or summary.get("n_release") != len(items)
    ):
        raise CohortAuditError("Ours-500K formal summary binding mismatch")
    summary_metrics = summary.get("metrics")
    if not isinstance(summary_metrics, Mapping):
        raise CohortAuditError("Ours-500K formal summary metrics are missing")
    for name, passed_field in (
        ("rest_all_pair_cf", "rest_all_pair_cf"),
        ("rest_non_adjacent_cf", "rest_non_adjacent_cf"),
        ("single_joint_sweep_cf", "single_joint_sweep_cf"),
        ("multi_joint_sobol_cf", "multi_joint_sobol_cf"),
        ("strict_collision_pass", "strict_collision_pass"),
    ):
        if summary_metrics.get(name) != _metric(int(aggregate[passed_field]), len(items)):
            raise CohortAuditError(f"Ours-500K formal summary {name} mismatch")
    collision_states = int(aggregate["collision_states"])
    expected_states = int(aggregate["expected_states"])
    executed_states = int(aggregate["executed_states"])
    unexecuted = expected_states - executed_states
    expected_collision_metric = {
        "collision_states": collision_states + unexecuted,
        "definition": (
            "fail-closed collision-or-unexecuted configurations / frozen expected configurations"
        ),
        "denominator": expected_states,
        "executed_states": executed_states,
        "observed_collision_rate_executed": collision_states / executed_states,
        "observed_collision_states": collision_states,
        "rate": (collision_states + unexecuted) / expected_states,
        "unexecuted_states": unexecuted,
    }
    if summary_metrics.get("collision_state_rate") != expected_collision_metric:
        raise CohortAuditError("Ours-500K formal summary collision-state rate mismatch")
    expected_single_states = sum(int(item["single_state_expected"]) for item in items)
    expected_range_metric = {
        "denominator": expected_single_states,
        "passed_states": int(aggregate["single_free_states"]),
        "rate": int(aggregate["single_free_states"]) / expected_single_states,
    }
    if summary_metrics.get("collision_free_range") != expected_range_metric:
        raise CohortAuditError("Ours-500K formal summary collision-free range mismatch")
    max_metric = summary_metrics.get("max_penetration")
    if not isinstance(max_metric, Mapping) or (
        max_metric.get("denominator") != len(items)
        or max_metric.get("fully_measured_assets") != aggregate["measurement_complete"]
        or max_metric.get("observed_assets") != aggregate["observed_normalized"]
        or max_metric.get("maximum_observed_normalized") != maximum_normalized
        or max_metric.get("status") != "COMPLETE"
    ):
        raise CohortAuditError("Ours-500K formal summary max-penetration mismatch")

    checks = verification.get("checks")
    if (
        verification.get("status") != "PASS"
        or not isinstance(checks, Mapping)
        or not checks
        or not all(value is True for value in checks.values())
        or verification.get("expected_states") != expected_states
        or verification.get("executed_states") != executed_states
    ):
        raise CohortAuditError("Ours-500K formal verification binding mismatch")

    closure_names = (
        "asset_records.jsonl",
        "checkpoint.json",
        "frozen_manifest.json",
        "protocol_document_at_freeze.md",
        "report.md",
        "state_records.jsonl",
        "summary.json",
        "verification.json",
    )
    closure_files = []
    for name in closure_names:
        path = _regular_file(artifact_root / name, f"Ours-500K artifact {name}")
        closure_files.append(
            {"path": name, "bytes": path.stat().st_size, "sha256": sha256_file(path)}
        )
    closure = {
        "provenance": (
            "retrospective read-only closure; the historical formal runner did not "
            "publish an artifact_manifest.json"
        ),
        "original_formal_artifact_manifest_present": False,
        "files": closure_files,
    }
    closure["closure_content_sha256"] = canonical_sha256(closure)

    input_info = {
        "frozen_manifest": {
            "path": str(Path(manifest_path).resolve()),
            "file_sha256": manifest_file_sha,
            "manifest_content_sha256": manifest_content_sha,
            "self_hash_verified": True,
        },
        "asset_records": {
            "path": str(resolved_records),
            "file_sha256": raw_digest.hexdigest(),
            "row_count": line_count,
            "frozen_row_identities_verified": True,
            "state_semantics_independently_replayed": True,
            "load_child_worker_success_verified": True,
            "reset_readback_tolerance": reset_tolerance,
            "reset_readback_tolerance_verified": True,
        },
        "formal_summary": {
            "path": str(summary_path.resolve()),
            "file_sha256": summary_sha,
            "metrics_recomputed": True,
        },
        "formal_verification": {
            "path": str(verification_path.resolve()),
            "file_sha256": verification_sha,
            "checks_verified": True,
        },
        "external_state_records": {
            "path": str(external_states),
            "file_sha256": external_state_sha,
            "row_count": replayed_state_count,
            "embedded_state_replay_sha256_verified": True,
        },
        "retrospective_artifact_closure": closure,
        "primary_urdf_payloads": {
            "dataset_root": str(dataset_root),
            "verified_count": len(items),
            "regular_non_symlink_verified": True,
            "dataset_root_containment_verified": True,
            "manifest_sha256_verified": True,
            "structured_xml_parsed_count": len(items),
            "mimic_asset_count": mimic_asset_count,
            "mimic_joint_count": mimic_joint_count,
        },
    }
    return manifest, records_by_pair, categories, input_info


def _safe_relative_file(root: Path, relative: Any, label: str) -> Path:
    text = _nonempty_string(relative, label)
    relpath = Path(text)
    if relpath.is_absolute() or ".." in relpath.parts:
        raise CohortAuditError(f"{label} is not a safe relative path: {text}")
    candidate = root / relpath
    resolved = _regular_file(candidate, label)
    try:
        resolved.relative_to(root.resolve(strict=True))
    except ValueError as error:
        raise CohortAuditError(f"{label} escapes its artifact root") from error
    return resolved


def _decode_meta(rows: Iterable[tuple[Any, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for raw_key, raw_value in rows:
        key = str(raw_key)
        if key in result:
            raise CohortAuditError(f"duplicate PV-A database meta key: {key}")
        try:
            result[key] = json.loads(str(raw_value))
        except json.JSONDecodeError as error:
            raise CohortAuditError(f"invalid PV-A database meta value for {key}") from error
    return result


def _connect_read_only(database: Path) -> sqlite3.Connection:
    # ``immutable=1`` prevents SQLite from consulting or creating sidecars;
    # ``mode=ro`` is retained explicitly as part of the audit contract.
    uri = f"{database.as_uri()}?mode=ro&immutable=1"
    connection = sqlite3.connect(uri, uri=True, timeout=120.0)
    connection.execute("PRAGMA query_only=ON")
    if int(connection.execute("PRAGMA query_only").fetchone()[0]) != 1:
        connection.close()
        raise CohortAuditError("PV-A SQLite connection did not enter query_only mode")
    return connection


def _database_stat(path: Path) -> tuple[int, int, int, int]:
    stat = path.stat()
    return (int(stat.st_dev), int(stat.st_ino), int(stat.st_size), int(stat.st_mtime_ns))


def _validate_database_schema(connection: sqlite3.Connection) -> None:
    expected = {
        "meta": {"key", "value"},
        "assets": {
            "ordinal",
            "asset_id",
            "category",
            "joint_count",
            "row_sha256",
            "row_json",
        },
        "results": {
            "ordinal",
            "asset_id",
            "table1_json",
            "table2_json",
            "table2sup_json",
            "table3_json",
            "table4_json",
            "table4_states_zlib",
            "table4_state_count",
            "worker_status",
            "worker_seconds",
            "completed_at_utc",
        },
    }
    observed_tables = {
        str(row[0])
        for row in connection.execute(
            "SELECT name FROM sqlite_master WHERE type='table'"
        )
    }
    if not set(expected).issubset(observed_tables):
        raise CohortAuditError("PV-A database is missing required tables")
    for table, columns in expected.items():
        observed = {
            str(row[1]) for row in connection.execute(f"PRAGMA table_info({table})")
        }
        if observed != columns:
            raise CohortAuditError(f"PV-A database {table} schema mismatch")


def _pva_input_identity(row: Mapping[str, Any], category: str, joint_count: int) -> str:
    return canonical_sha256(
        {
            "dataset": "pva",
            "dataset_id": row["asset_id"],
            "category": category,
            "urdf_path": row["primary_urdf_path"],
            "primary_urdf_relative_path": row["primary_urdf_relative_path"],
            "expected_primary_urdf_sha256": row["primary_urdf_sha256"],
            "expected_movable_joints": joint_count,
            "package_binding_sha256": row["package_binding_sha256"],
        }
    )


def _replay_pva_v1(
    record: Mapping[str, Any], blob: Any, declared_count: Any,
    worker_status: Any, threshold: float, reset_tolerance: float, label: str,
) -> bool:
    if worker_status not in PVA_SUCCESSFUL_WORKER_STATUSES | PVA_RETAINED_FAILURE_WORKER_STATUSES:
        raise CohortAuditError(f"{label} unknown worker_status")
    if not isinstance(blob, bytes):
        raise CohortAuditError(f"{label} states payload is not a BLOB")
    count = _integer(declared_count, f"{label}.table4_state_count")
    decoder = zlib.decompressobj()
    try:
        payload = decoder.decompress(blob) + decoder.flush()
    except zlib.error as error:
        raise CohortAuditError(f"{label} states payload is invalid zlib") from error
    if not decoder.eof or decoder.unused_data or decoder.unconsumed_tail:
        raise CohortAuditError(f"{label} states payload has invalid zlib framing")
    states = []
    for index, line in enumerate(payload.decode("utf-8").splitlines()):
        try:
            state = json.loads(line)
        except json.JSONDecodeError as error:
            raise CohortAuditError(f"{label} state {index} is invalid JSON") from error
        if not isinstance(state, dict) or canonical_text(state) != line:
            raise CohortAuditError(f"{label} state {index} is not canonical")
        states.append(state)
    if len(states) != count:
        raise CohortAuditError(f"{label} state count mismatch")
    if record.get("state_records_sha256") != canonical_sha256(states):
        raise CohortAuditError(f"{label} state hash mismatch")
    if worker_status in PVA_RETAINED_FAILURE_WORKER_STATUSES:
        if (
            states
            or record.get("status") != "error"
            or record.get("load_success") is not False
            or record.get("measurement_complete") is not False
            or any(record.get(field) not in (None, False) for field in (
                "rest_all_pair_cf", "rest_non_adjacent_cf",
                "single_joint_sweep_cf", "multi_joint_sobol_cf",
                "strict_collision_pass",
            ))
        ):
            raise CohortAuditError(f"{label} retained worker failure is not fail-closed")
        return False
    if record.get("state_records_count") != count:
        raise CohortAuditError(f"{label} record/database state count mismatch")
    phases = defaultdict(list)
    max_penetration = 0.0
    max_reset = 0.0
    for index, state in enumerate(states):
        phase = state.get("phase")
        if phase not in ("rest", "single_joint_sweep", "multi_joint_sobol"):
            raise CohortAuditError(f"{label} state {index} invalid phase")
        for field in ("order", "dataset_id", "category", "protocol_id", "input_identity_sha256"):
            if state.get(field) != record.get(field):
                raise CohortAuditError(f"{label} state {index} binding mismatch: {field}")
        nonadj = _finite_number(state.get("non_adjacent_max_penetration_m"), f"{label} state penetration", minimum=0.0)
        allpair = _finite_number(state.get("all_pair_max_penetration_m"), f"{label} state all-pair penetration", minimum=0.0)
        _integer(state.get("non_adjacent_illegal_penetration_count"), f"{label} state non-adjacent illegal count")
        _integer(state.get("all_pair_illegal_penetration_count"), f"{label} state all-pair illegal count")
        metric = _finite_number(state.get("metric_max_penetration_m"), f"{label} state metric penetration", minimum=0.0)
        if metric != (allpair if phase == "rest" else nonadj):
            raise CohortAuditError(f"{label} state {index} phase metric mismatch")
        reset = _finite_number(state.get("reset_readback_max_abs_error"), f"{label} state reset", minimum=0.0)
        if reset > reset_tolerance:
            raise CohortAuditError(f"{label} state {index} reset tolerance exceeded")
        phases[phase].append(state)
        max_penetration = max(max_penetration, metric)
        max_reset = max(max_reset, reset)
    expected = {"rest": int(record["rest_state_expected"]), "single_joint_sweep": int(record["single_state_expected"]), "multi_joint_sobol": int(record["sobol_state_expected"])}
    executed = {phase: len(phases[phase]) for phase in expected}
    free = {phase: sum(int(row["non_adjacent_illegal_penetration_count"]) == 0 for row in phases[phase]) for phase in expected}
    measurement = bool(record.get("load_success") is True and record.get("status") == "completed" and executed == expected)
    rest_nonadj = bool(expected["rest"] and free["rest"] == expected["rest"])
    single_cf = bool(executed["single_joint_sweep"] == expected["single_joint_sweep"] and free["single_joint_sweep"] == expected["single_joint_sweep"])
    sobol_cf = bool(expected["multi_joint_sobol"] > 0 and executed["multi_joint_sobol"] == expected["multi_joint_sobol"] and free["multi_joint_sobol"] == expected["multi_joint_sobol"])
    strict = bool(measurement and rest_nonadj and single_cf and sobol_cf)
    recomputed = {
        "rest_state_executed": executed["rest"], "single_state_executed": executed["single_joint_sweep"], "sobol_state_executed": executed["multi_joint_sobol"],
        "rest_non_adjacent_free": free["rest"], "single_non_adjacent_free": free["single_joint_sweep"], "sobol_non_adjacent_free": free["multi_joint_sobol"],
        "rest_all_pair_cf": bool(executed["rest"] == expected["rest"] and all(int(row["all_pair_illegal_penetration_count"]) == 0 for row in phases["rest"])),
        "rest_non_adjacent_cf": rest_nonadj, "single_joint_sweep_cf": single_cf, "multi_joint_sobol_cf": sobol_cf,
        "measurement_complete": measurement, "strict_collision_pass": strict,
    }
    for field, value in recomputed.items():
        if record.get(field) != value:
            raise CohortAuditError(f"{label} independently replayed field mismatch: {field}")
    for field, value in (
        ("max_penetration_m", max_penetration),
        ("max_reset_readback_error", max_reset),
    ):
        published = record.get(field)
        if published is not None and published != value:
            raise CohortAuditError(f"{label} independently replayed field mismatch: {field}")
    return strict


def _read_pva(
    evaluation_root: Path,
    ours_pairs: set[tuple[str, int]],
    ours_categories: set[str],
) -> tuple[
    dict[str, Any],
    dict[tuple[str, int], dict[str, Any]],
    dict[str, tuple[int, int]],
    dict[str, Any],
]:
    root = Path(evaluation_root).resolve(strict=True)
    if not root.is_dir():
        raise CohortAuditError(f"PV-A evaluation root is not a directory: {root}")
    execution_path = _regular_file(root / "manifest.json", "PV-A execution manifest")
    receipt_path = _regular_file(
        root / "full_release_receipt.json", "PV-A full-release receipt"
    )
    execution, execution_file_sha = _load_json(
        execution_path, "PV-A execution manifest"
    )
    receipt, receipt_file_sha = _load_json(
        receipt_path, "PV-A full-release receipt"
    )
    if execution.get("schema_version") != PVA_MANIFEST_SCHEMA:
        raise CohortAuditError("unexpected PV-A execution manifest schema")
    if receipt.get("schema_version") != PVA_RECEIPT_SCHEMA:
        raise CohortAuditError("unexpected PV-A full-release receipt schema")
    execution_content_sha = _verify_self_hash(
        execution, "manifest_content_sha256", "PV-A execution manifest"
    )
    receipt_content_sha = _verify_self_hash(
        receipt, "receipt_content_sha256", "PV-A full-release receipt"
    )
    if receipt.get("execution_manifest_sha256") != execution_file_sha:
        raise CohortAuditError("PV-A receipt does not bind the execution manifest file")
    if receipt.get("N_eval") != execution.get("N_eval"):
        raise CohortAuditError("PV-A receipt and execution manifest N_eval disagree")
    if receipt.get("eval_category_count") != execution.get("category_count"):
        raise CohortAuditError(
            "PV-A receipt and execution manifest category counts disagree"
        )
    for field in ("roster_manifest_sha256", "roster_manifest_content_sha256"):
        if receipt.get(field) != execution.get(field):
            raise CohortAuditError(f"PV-A receipt/execution {field} mismatch")

    database = _safe_relative_file(
        root, receipt.get("result_database"), "PV-A result database"
    )
    for suffix in ("-wal", "-journal"):
        sidecar = Path(str(database) + suffix)
        if sidecar.exists() and sidecar.stat().st_size:
            raise CohortAuditError(
                f"PV-A database has a non-empty mutable sidecar: {sidecar.name}"
            )
    stat_before = _database_stat(database)
    declared_bytes = _integer(
        receipt.get("result_database_bytes"), "PV-A receipt result_database_bytes"
    )
    if stat_before[2] != declared_bytes:
        raise CohortAuditError("PV-A result database byte size disagrees with receipt")

    paired: dict[tuple[str, int], dict[str, Any]] = {}
    category_counts: dict[str, list[int]] = defaultdict(lambda: [0, 0])
    all_pairs: set[tuple[str, int]] = set()
    full_n = 0
    full_passed = 0
    pva_threshold = _finite_number(
        _required(execution, "protocol.table4_penetration_threshold_m", "PV-A execution manifest"),
        "PV-A penetration threshold", minimum=0.0,
    )
    with _connect_read_only(database) as connection:
        _validate_database_schema(connection)
        meta = _decode_meta(connection.execute("SELECT key, value FROM meta"))
        if meta.get("schema_version") != PVA_DATABASE_SCHEMA:
            raise CohortAuditError("unexpected PV-A database schema_version meta value")
        if meta.get("asset_import_state") != "COMPLETE":
            raise CohortAuditError("PV-A database asset import is not complete")
        for field in ("roster_manifest_sha256", "roster_manifest_content_sha256"):
            if meta.get(field) != receipt.get(field):
                raise CohortAuditError(f"PV-A database meta {field} mismatch")

        query = """
            SELECT
                a.ordinal, a.asset_id, a.category, a.joint_count,
                a.row_sha256, a.row_json,
                r.asset_id, r.table4_json, r.table4_states_zlib,
                r.table4_state_count, r.worker_status
            FROM assets AS a
            LEFT JOIN results AS r ON r.ordinal = a.ordinal
            ORDER BY a.ordinal
        """
        for expected_ordinal, db_row in enumerate(connection.execute(query)):
            (
                ordinal,
                asset_id_raw,
                category_raw,
                joint_count_raw,
                row_sha,
                row_text,
                result_asset_id,
                table4_text,
                states_blob,
                state_count,
                worker_status,
            ) = db_row
            ordinal = _integer(ordinal, "PV-A asset ordinal")
            if ordinal != expected_ordinal:
                raise CohortAuditError(
                    f"PV-A database ordinal closure mismatch at {expected_ordinal}"
                )
            asset_id = _nonempty_string(asset_id_raw, f"PV-A asset {ordinal}.asset_id")
            category = _nonempty_string(category_raw, f"PV-A asset {ordinal}.category")
            joint_count = _integer(
                joint_count_raw, f"PV-A asset {ordinal}.joint_count"
            )
            if not isinstance(row_text, str) or hashlib.sha256(
                row_text.encode("utf-8")
            ).hexdigest() != row_sha:
                raise CohortAuditError(f"PV-A asset {ordinal} row_sha256 mismatch")
            try:
                row = json.loads(row_text)
            except json.JSONDecodeError as error:
                raise CohortAuditError(f"PV-A asset {ordinal} row_json is invalid") from error
            if not isinstance(row, dict) or canonical_text(row) != row_text:
                raise CohortAuditError(f"PV-A asset {ordinal} row_json is not canonical")
            if (
                row.get("ordinal") != ordinal
                or row.get("asset_id") != asset_id
                or row.get("raw_category", row.get("category")) != category
                or row.get("joint_count") != joint_count
            ):
                raise CohortAuditError(f"PV-A asset {ordinal} database/row identity mismatch")
            # ``row.seed`` is a generator parameter and is not guaranteed to
            # equal the package roster index.  Cross-release identity is the
            # source package name, which is also embedded in ``asset_id``.
            _integer(row.get("seed"), f"PV-A asset {ordinal}.seed")
            source_asset_id = _nonempty_string(
                row.get("source_asset_id"),
                f"PV-A asset {ordinal}.source_asset_id",
            )
            seed = _seed_number(
                source_asset_id, f"PV-A asset {ordinal}.source_asset_id"
            )
            if asset_id != f"PV-A/{category}/{source_asset_id}":
                raise CohortAuditError(f"PV-A asset {ordinal} asset_id is non-canonical")
            pair = (category, seed)
            if pair in all_pairs:
                raise CohortAuditError(f"duplicate PV-A category/seed pair: {pair}")
            all_pairs.add(pair)

            if result_asset_id is None or table4_text is None:
                raise CohortAuditError(f"PV-A asset {ordinal} has no Table 4 result")
            if result_asset_id != asset_id:
                raise CohortAuditError(f"PV-A asset {ordinal} result asset_id mismatch")
            try:
                result = json.loads(table4_text)
            except json.JSONDecodeError as error:
                raise CohortAuditError(
                    f"PV-A asset {ordinal} table4_json is invalid"
                ) from error
            if not isinstance(result, dict) or canonical_text(result) != table4_text:
                raise CohortAuditError(
                    f"PV-A asset {ordinal} table4_json is not canonical"
                )
            expected_identity = _pva_input_identity(row, category, joint_count)
            bindings = (
                ("protocol_id", PVA_PROTOCOL_ID),
                ("order", ordinal),
                ("dataset_id", asset_id),
                ("category", category),
                ("expected_primary_urdf_sha256", row.get("primary_urdf_sha256")),
                ("expected_movable_joints", joint_count),
                ("primary_urdf_relative_path", row.get("primary_urdf_relative_path")),
                ("package_binding_sha256", row.get("package_binding_sha256")),
                ("input_identity_sha256", expected_identity),
            )
            for field, expected in bindings:
                if result.get(field) != expected:
                    raise CohortAuditError(
                        f"PV-A asset {ordinal} Table 4 {field} binding mismatch"
                    )
            if result.get("package") != row.get("source_path") or result.get(
                "urdf_path"
            ) != row.get("primary_urdf_path"):
                raise CohortAuditError(
                    f"PV-A asset {ordinal} Table 4 source path binding mismatch"
                )
            strict = _replay_pva_v1(
                result, states_blob, state_count, worker_status,
                pva_threshold, math.inf, f"PV-A asset {ordinal}",
            )
            full_n += 1
            full_passed += int(strict)
            category_counts[category][0] += 1
            category_counts[category][1] += int(strict)
            if pair in ours_pairs:
                paired[pair] = {
                    "passed": strict,
                    "urdf_sha256": row.get("primary_urdf_sha256"),
                    "joint_count": joint_count,
                }

        result_count = int(connection.execute("SELECT COUNT(*) FROM results").fetchone()[0])
        if result_count != full_n:
            raise CohortAuditError("PV-A database has orphan or duplicate result rows")
        if meta.get("selected_n") != full_n:
            raise CohortAuditError("PV-A database selected_n meta value disagrees")
        if meta.get("selected_category_count") != len(category_counts):
            raise CohortAuditError(
                "PV-A database selected_category_count meta value disagrees"
            )

    missing_pairs = sorted(ours_pairs - set(paired))
    if missing_pairs:
        raise CohortAuditError(
            f"PV-A release is missing {len(missing_pairs)} Ours-500K pairs: {missing_pairs[:10]}"
        )
    if set(paired) != ours_pairs:
        raise CohortAuditError("PV-A paired result closure mismatch")
    if full_n != receipt.get("N_eval") or full_n != receipt.get("N_release"):
        raise CohortAuditError("PV-A database and receipt release counts disagree")
    if len(category_counts) != receipt.get("eval_category_count") or len(
        category_counts
    ) != receipt.get("release_category_count"):
        raise CohortAuditError("PV-A database and receipt category counts disagree")
    if not ours_categories.issubset(category_counts):
        raise CohortAuditError("one or more Ours-500K categories are absent from PV-A")

    database_sha = sha256_file(database)
    stat_after = _database_stat(database)
    if stat_after != stat_before:
        raise CohortAuditError("PV-A result database changed during the audit")
    if database_sha != receipt.get("result_database_sha256"):
        raise CohortAuditError("PV-A result database SHA256 disagrees with receipt")

    inputs = {
        "execution_manifest": {
            "path": str(execution_path),
            "file_sha256": execution_file_sha,
            "manifest_content_sha256": execution_content_sha,
            "self_hash_verified": True,
        },
        "full_release_receipt": {
            "path": str(receipt_path),
            "file_sha256": receipt_file_sha,
            "receipt_content_sha256": receipt_content_sha,
            "self_hash_verified": True,
        },
        "results_database": {
            "path": str(database),
            "file_sha256": database_sha,
            "size_bytes": stat_after[2],
            "receipt_sha256_verified": True,
            "sqlite_uri_mode": "ro",
            "sqlite_immutable": True,
            "sqlite_query_only": True,
            "roster_row_sha256_verified": True,
            "table4_input_identities_verified": True,
            "all_table4_states_independently_replayed": True,
            "all_worker_statuses_verified": True,
        },
    }
    frozen_counts = {
        category: (counts[0], counts[1])
        for category, counts in category_counts.items()
    }
    return execution, paired, frozen_counts, inputs


def _protocol_value(
    ours: Mapping[str, Any],
    ours_path: str,
    pva: Mapping[str, Any],
    pva_path: str,
) -> dict[str, Any]:
    ours_value = _required(ours, ours_path, "Ours-500K frozen manifest")
    pva_value = _required(pva, pva_path, "PV-A execution manifest")
    return {"ours": ours_value, "pva": pva_value, "aligned": ours_value == pva_value}


def _protocol_alignment(
    ours_manifest: Mapping[str, Any], pva_manifest: Mapping[str, Any]
) -> dict[str, Any]:
    ours_core = _required(
        ours_manifest, "evaluation.core_sha256", "Ours-500K frozen manifest"
    )
    if _required(
        ours_manifest,
        "evaluation.runtime_identity.collision_core_sha256",
        "Ours-500K frozen manifest",
    ) != ours_core:
        raise CohortAuditError("Ours-500K collision core bindings disagree internally")
    pva_core = _required(
        pva_manifest, "source_hashes.table4_core", "PV-A execution manifest"
    )
    if _required(
        pva_manifest,
        "runtime_identity.collision_core_sha256",
        "PV-A execution manifest",
    ) != pva_core:
        raise CohortAuditError("PV-A collision core bindings disagree internally")

    fields = {
        "collision_core_sha256": {
            "ours": ours_core,
            "pva": pva_core,
            "aligned": ours_core == pva_core,
        },
        "pybullet_module_sha256": _protocol_value(
            ours_manifest,
            "evaluation.runtime_identity.pybullet_module_sha256",
            pva_manifest,
            "runtime_identity.pybullet_module_sha256",
        ),
        "pybullet_api_version": _protocol_value(
            ours_manifest,
            "evaluation.runtime_identity.pybullet_api_version",
            pva_manifest,
            "runtime_identity.pybullet_api_version",
        ),
        "single_joint_samples_K": _protocol_value(
            ours_manifest,
            "evaluation.single_samples",
            pva_manifest,
            "protocol.table4_single_joint_samples",
        ),
        "sobol_samples_R": _protocol_value(
            ours_manifest,
            "evaluation.sobol_samples",
            pva_manifest,
            "protocol.table4_sobol_samples",
        ),
        "sobol_seed": _protocol_value(
            ours_manifest,
            "evaluation.sobol_seed",
            pva_manifest,
            "protocol.table4_sobol_seed",
        ),
        "penetration_threshold_m": _protocol_value(
            ours_manifest,
            "evaluation.penetration_threshold_m",
            pva_manifest,
            "protocol.table4_penetration_threshold_m",
        ),
    }
    all_aligned = all(bool(value["aligned"]) for value in fields.values())
    if not all_aligned:
        mismatched = [name for name, value in fields.items() if not value["aligned"]]
        raise CohortAuditError(
            "Table 4 protocol alignment failed: " + ", ".join(mismatched)
        )
    return {
        **fields,
        "all_aligned": True,
        "interpretation": (
            "Declared evaluator fields align, but this does not establish protocol "
            "parity because 499/500 label-matched primary URDF payloads differ."
        ),
        "causal_protocol_attribution_supported": False,
    }


def build_audit(
    ours_artifact: Path = DEFAULT_OURS_ARTIFACT,
    pva_evaluation: Path = DEFAULT_PVA_EVALUATION,
) -> dict[str, Any]:
    ours_root = Path(ours_artifact).resolve(strict=True)
    if not ours_root.is_dir():
        raise CohortAuditError(f"Ours-500K artifact root is not a directory: {ours_root}")
    ours_manifest, ours_pairs, ours_categories, ours_inputs = _read_ours(
        ours_root / "frozen_manifest.json",
        ours_root / "asset_records.jsonl",
    )
    pva_manifest, pva_pairs, category_counts, pva_inputs = _read_pva(
        Path(pva_evaluation), set(ours_pairs), ours_categories
    )
    protocol = _protocol_alignment(ours_manifest, pva_manifest)

    both_pass = 0
    ours_only = 0
    pva_only = 0
    both_fail = 0
    byte_identical = 0
    joint_count_identical = 0
    ours_passed = 0
    pva_paired_passed = 0
    for pair in sorted(ours_pairs):
        ours = ours_pairs[pair]
        pva = pva_pairs[pair]
        ours_pass = bool(ours["passed"])
        pva_pass = bool(pva["passed"])
        ours_passed += int(ours_pass)
        pva_paired_passed += int(pva_pass)
        both_pass += int(ours_pass and pva_pass)
        ours_only += int(ours_pass and not pva_pass)
        pva_only += int(not ours_pass and pva_pass)
        both_fail += int(not ours_pass and not pva_pass)
        byte_identical += int(ours["urdf_sha256"] == pva["urdf_sha256"])
        joint_count_identical += int(ours["joint_count"] == pva["joint_count"])

    full_n = sum(counts[0] for counts in category_counts.values())
    full_passed = sum(counts[1] for counts in category_counts.values())
    ours_category_n = sum(category_counts[category][0] for category in ours_categories)
    ours_category_passed = sum(
        category_counts[category][1] for category in ours_categories
    )
    remaining_categories = set(category_counts) - ours_categories
    remaining_n = sum(category_counts[category][0] for category in remaining_categories)
    remaining_passed = sum(
        category_counts[category][1] for category in remaining_categories
    )
    paired_n = len(ours_pairs)
    if both_pass + ours_only + pva_only + both_fail != paired_n:
        raise CohortAuditError("paired contingency table does not close")
    if ours_passed != both_pass + ours_only:
        raise CohortAuditError("Ours-500K paired pass aggregation does not close")
    if pva_paired_passed != both_pass + pva_only:
        raise CohortAuditError("PV-A paired pass aggregation does not close")
    if ours_category_n + remaining_n != full_n:
        raise CohortAuditError("PV-A category partition denominator does not close")
    if ours_category_passed + remaining_passed != full_passed:
        raise CohortAuditError("PV-A category partition pass count does not close")

    audit: dict[str, Any] = {
        "schema_version": AUDIT_SCHEMA,
        "audit_semantics": {
            "mode": "read_only_label_matched_cohort_output_comparison",
            "label_match_key": [
                "exact raw_category",
                "integerized package source_asset_id (seed_<digits>)",
            ],
            "strict_metric": "strict_collision_pass",
            "interpretation": (
                "This compares outputs for label-matched category/package-seed rows. "
                "It is a cohort/output comparison, not protocol parity: 499/500 "
                "primary URDF payloads differ, so differences cannot be causally "
                "attributed to the evaluation protocol."
            ),
            "causal_protocol_attribution_supported": False,
        },
        "inputs": {"ours_500k": ours_inputs, "current_pva_v1": pva_inputs},
        "protocol_alignment": protocol,
        "label_matched_category_seed_outputs": {
            "label_match_count": paired_n,
            "ours_500k_output": _metric(ours_passed, paired_n),
            "current_pva_v1_output": _metric(pva_paired_passed, paired_n),
            "contingency": {
                "both_pass": both_pass,
                "ours_only_pass": ours_only,
                "pva_only_pass": pva_only,
                "both_fail": both_fail,
            },
            "byte_identical_primary_urdf_count": byte_identical,
            "joint_count_identical_count": joint_count_identical,
        },
        "current_pva_v1_cohort_partition": {
            "ours_500k_categories": {
                "category_count": len(ours_categories),
                "categories": sorted(ours_categories),
                **_metric(ours_category_passed, ours_category_n),
            },
            "remaining_categories": {
                "category_count": len(remaining_categories),
                **_metric(remaining_passed, remaining_n),
            },
            "full_release": {
                "category_count": len(category_counts),
                **_metric(full_passed, full_n),
            },
        },
        "validation": {
            "ours_manifest_self_hash_verified": True,
            "ours_frozen_row_identities_verified": True,
            "ours_primary_urdf_payload_sha256_verified": True,
            "ours_primary_urdf_regular_non_symlink_verified": True,
            "ours_primary_urdf_xml_parsed": True,
            "ours_state_semantics_independently_replayed": True,
            "ours_load_child_worker_success_verified": True,
            "ours_reset_readback_tolerance_verified": True,
            "ours_formal_summary_recomputed": True,
            "ours_formal_verification_bound": True,
            "ours_external_state_artifact_bound": True,
            "ours_retrospective_artifact_closure_recorded": True,
            "ours_result_closure_verified": True,
            "pva_execution_manifest_self_hash_verified": True,
            "pva_receipt_self_hash_verified": True,
            "pva_database_receipt_sha256_verified": True,
            "pva_database_opened_mode_ro": True,
            "pva_roster_row_sha256_verified": True,
            "pva_table4_input_identities_verified": True,
            "label_match_identity_closure_verified": True,
            "declared_protocol_fields_aligned": True,
            "causal_protocol_attribution_supported": False,
            "category_partition_closure_verified": True,
        },
        "implementation": {
            "script": str(SCRIPT),
            "script_sha256": sha256_file(SCRIPT),
        },
    }
    audit["audit_content_sha256"] = canonical_sha256(audit)
    return audit


def write_canonical_json(path: Path, value: Mapping[str, Any]) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = (canonical_text(value) + "\n").encode("ascii")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", dir=str(output.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, output)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def verify_audit_self_hash(value: Mapping[str, Any]) -> bool:
    declared = value.get("audit_content_sha256")
    if not isinstance(declared, str):
        return False
    payload = dict(value)
    payload.pop("audit_content_sha256", None)
    return declared == canonical_sha256(payload)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ours-artifact", type=Path, default=DEFAULT_OURS_ARTIFACT)
    parser.add_argument("--pva-evaluation", type=Path, default=DEFAULT_PVA_EVALUATION)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        audit = build_audit(args.ours_artifact, args.pva_evaluation)
        if not verify_audit_self_hash(audit):
            raise CohortAuditError("internal audit self-hash verification failed")
        write_canonical_json(args.output, audit)
    except (CohortAuditError, OSError, sqlite3.Error) as error:
        print(f"ERROR: {error}", file=__import__("sys").stderr)
        return 1
    print(canonical_text(audit))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
