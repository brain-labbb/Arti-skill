#!/usr/bin/env python3
"""Read-only integrity checks for the PV-A Table 1/2/2-supp/3/4 run."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sqlite3
from statistics import mean, median
import subprocess
import sys
from typing import Any, Iterator, Mapping
import zlib


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import build_pva_full_release_roster as pva_roster
import check_table4_full_release as table4_checker
import run_pva_table1234_full_release as runner
import table123_full_release_common as common


CONTRACT_TEST_FILES = (
    "test_pva_table1234_full_release.py",
    "test_pva_roster_shard_evidence.py",
    "test_pva_runner_scaling_resume.py",
)


class AutomationError(ValueError):
    """Raised when a PV-A publication artifact is incomplete or has drifted."""


def _json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise AutomationError(f"invalid JSON {path}: {error}") from error
    if not isinstance(value, dict):
        raise AutomationError(f"expected JSON object: {path}")
    return value


def _without(value: Mapping[str, Any], field: str) -> dict[str, Any]:
    result = dict(value)
    result.pop(field, None)
    return result


def _check_self_hash(value: Mapping[str, Any], field: str, label: str) -> None:
    if value.get(field) != common.canonical_sha256(_without(value, field)):
        raise AutomationError(f"{label} self-hash mismatch")


def _resolve(root: Path, value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise AutomationError(f"{label} path is missing")
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = root / candidate
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(root)
    except (OSError, ValueError) as error:
        raise AutomationError(f"{label} path escapes or is missing: {candidate}") from error
    if not resolved.is_file() or resolved.is_symlink():
        raise AutomationError(f"{label} is not a regular file: {resolved}")
    return resolved


def _sha_and_lines(path: Path) -> tuple[str, int]:
    digest = hashlib.sha256()
    lines = 0
    try:
        with Path(path).open("rb") as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(block)
                lines += block.count(b"\n")
    except OSError as error:
        raise AutomationError(f"cannot read artifact {path}: {error}") from error
    return digest.hexdigest(), lines


def _records(path: Path) -> list[dict[str, Any]]:
    values: list[dict[str, Any]] = []
    try:
        stream = path.open(encoding="utf-8")
    except OSError as error:
        raise AutomationError(f"cannot read records {path}: {error}") from error
    with stream:
        for line_number, line in enumerate(stream, 1):
            try:
                value = json.loads(line)
            except json.JSONDecodeError as error:
                raise AutomationError(f"invalid record JSON: {path}:{line_number}") from error
            if not isinstance(value, dict):
                raise AutomationError(f"record is not an object: {path}:{line_number}")
            values.append(value)
    return values


def _distribution(values: list[int]) -> dict[str, Any]:
    if not values:
        return {"denominator": 0, "mean": None, "median": None, "p90_nearest_rank": None}
    ordered = sorted(values)
    return {
        "denominator": len(ordered),
        "mean": mean(ordered),
        "median": median(ordered),
        "p90_nearest_rank": ordered[math.ceil(0.9 * len(ordered)) - 1],
    }


def _rate(numerator: int, denominator: int) -> dict[str, Any]:
    return {
        "numerator": int(numerator),
        "denominator": int(denominator),
        "percent": round(100.0 * numerator / denominator, 2) if denominator else None,
    }


def _require_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise AutomationError(f"{label} reaggregation mismatch")


def _reaggregate_table1(
    records: list[dict[str, Any]],
    summary: Mapping[str, Any],
    *,
    n_eval: int,
    n_release: int,
    release_category_count: int,
) -> None:
    parsed = [record for record in records if record.get("parse_success")]
    links = [int(record["link_count"]) for record in parsed]
    movable = [int(record["non_fixed_joint_count"]) for record in parsed]
    topologies = [
        str(record["topology_hash"])
        for record in records
        if record.get("valid_tree") and record.get("topology_hash")
    ]
    fingerprints = [
        str(record["package_fingerprint"])
        for record in records
        if record.get("fingerprint_complete") and record.get("package_fingerprint")
    ]
    fingerprint_counts = Counter(fingerprints)
    duplicate_counts = [count for count in fingerprint_counts.values() if count > 1]
    joint_types: Counter[str] = Counter()
    for record in parsed:
        joint_types.update(record.get("joint_type_counts") or {})
    multi = sum(value >= 2 for value in movable)
    expected = {
        "cohort": {
            "N_release": n_release,
            "N_eval": n_eval,
            "N_parse": len(parsed),
            "release_raw_categories": release_category_count,
            "eval_raw_categories": len(
                {str(record.get("raw_category", record.get("category", ""))) for record in records}
            ),
            "cohort_type": "FULL_LOCAL_RELEASE",
        },
        "links_per_asset": _distribution(links),
        "movable_joints_per_asset": _distribution(movable),
        "declared_joint_type_counts": dict(sorted(joint_types.items())),
        "multi_joint_assets": {
            "numerator": multi,
            "denominator": n_eval,
            "rate": multi / n_eval if n_eval else None,
            "valid_only_denominator": len(parsed),
            "valid_only_rate": multi / len(parsed) if parsed else None,
        },
        "unique_topologies": {
            "unique": len(set(topologies)),
            "denominator": len(topologies),
            "rate": len(set(topologies)) / len(topologies) if topologies else None,
            "coverage_denominator": n_eval,
            "coverage_rate": len(topologies) / n_eval if n_eval else None,
        },
        "exact_duplicate_rate": {
            "duplicate_excess": len(fingerprints) - len(fingerprint_counts),
            "unique": len(fingerprint_counts),
            "denominator": len(fingerprints),
            "rate": (len(fingerprints) - len(fingerprint_counts)) / len(fingerprints)
            if fingerprints
            else None,
            "assets_in_duplicate_clusters": sum(duplicate_counts),
            "assets_in_duplicate_clusters_rate": sum(duplicate_counts) / len(fingerprints)
            if fingerprints
            else None,
            "duplicate_cluster_count": len(duplicate_counts),
            "max_cluster_size": max(duplicate_counts, default=1 if fingerprints else 0),
            "coverage_denominator": n_eval,
            "coverage_rate": len(fingerprints) / n_eval if n_eval else None,
        },
        "asset_failure_count": sum(
            str(record.get("status")) != "EVALUATED" for record in records
        ),
    }
    for key, value in expected.items():
        _require_equal(summary.get(key), value, f"table1 {key}")


def _reaggregate_table2(
    records: list[dict[str, Any]], summary: Mapping[str, Any], *, n_eval: int
) -> None:
    metric_names = (
        "parse_rate",
        "resource_resolution",
        "finite_fields",
        "valid_tree",
        "valid_joint_spec",
        "collision_coverage",
        "inertial_coverage",
        "inertia_validity",
        "strict_urdf_pass",
    )
    expected_metrics = {}
    for name in metric_names:
        passed = sum(
            (record.get("metrics") or {}).get(name, {}).get("pass") is True
            for record in records
        )
        expected_metrics[name] = {
            "passed": passed,
            "denominator": n_eval,
            "rate": passed / n_eval if n_eval else None,
        }
    _require_equal(summary.get("metrics"), expected_metrics, "table2 metrics")
    _require_equal(
        summary.get("status_counts"),
        dict(sorted(Counter(str(record.get("status")) for record in records).items())),
        "table2 status_counts",
    )
    _require_equal(
        summary.get("error_count"),
        sum(str(record.get("status")) in {"error", "timeout"} for record in records),
        "table2 error_count",
    )


def _reaggregate_table2_supplementary(
    records: list[dict[str, Any]],
    summary: Mapping[str, Any],
    *,
    n_eval: int,
    j_eval: int,
) -> None:
    visual_passed = visual_declared = visual_covered = extraction_complete = zero_visual = 0
    port_passed = port_extracted = dyn_covered = dyn_extracted = 0
    dynamic_links = complete_inertial = 0
    for record in records:
        table = record.get("table2_supplementary") or {}
        visual = table.get("visual_bearing_collision_coverage") or {}
        portability = table.get("joint_limit_portability") or {}
        dynamics = table.get("joint_dynamics_coverage") or {}
        placeholder = table.get("placeholder_mass_incidence") or {}
        visual_passed += int(bool(visual.get("asset_pass")))
        visual_declared += int(visual.get("visual_bearing_links_declared", 0) or 0)
        visual_covered += int(visual.get("covered_visual_bearing_links", 0) or 0)
        extraction_complete += int(bool(visual.get("link_extraction_complete")))
        zero_visual += int(
            record.get("status") == "completed"
            and int(visual.get("visual_bearing_links_declared", 0) or 0) == 0
        )
        port_passed += int(portability.get("joints_passed", 0) or 0)
        port_extracted += int(portability.get("joints_extracted", 0) or 0)
        dyn_covered += int(dynamics.get("joints_covered", 0) or 0)
        dyn_extracted += int(dynamics.get("joints_extracted", 0) or 0)
        dynamic_links += int(placeholder.get("dynamic_links", 0) or 0)
        complete_inertial += int(placeholder.get("complete_inertial_links", 0) or 0)
    expected = {
        "visual_bearing_collision_coverage": {
            "asset": _rate(visual_passed, n_eval),
            "link_micro": _rate(visual_covered, visual_declared),
            "link_extraction_complete_assets": extraction_complete,
            "zero_visual_bearing_assets_completed": zero_visual,
        },
        "joint_limit_portability": _rate(port_passed, j_eval)
        | {"joints_extracted": port_extracted},
        "joint_dynamics_coverage": _rate(dyn_covered, j_eval)
        | {"joints_extracted": dyn_extracted},
        "placeholder_mass_incidence": {
            "status": "N/E",
            "reason": "placeholder_registry_empty",
            "registry_ids": [],
            "complete_inertial_links": complete_inertial,
            "dynamic_links_measured": dynamic_links,
            "coverage": _rate(complete_inertial, dynamic_links),
        },
    }
    _require_equal(summary.get("metrics"), expected, "table2_supplementary metrics")
    _require_equal(
        summary.get("status_counts"),
        dict(Counter(str(record.get("status", "error")) for record in records)),
        "table2_supplementary status_counts",
    )


def _reaggregate_table3(
    records: list[dict[str, Any]],
    summary: Mapping[str, Any],
    *,
    n_eval: int,
    j_eval: int,
) -> None:
    joints: list[Mapping[str, Any]] = []
    for record in records:
        record_joints = record.get("joints") or []
        if len(record_joints) != int(record.get("declared_joint_count", -1)):
            raise AutomationError("table3 joint atom denominator mismatch")
        joints.extend(record_joints)
    if len(joints) != j_eval:
        raise AutomationError("table3 joint denominator mismatch")
    fields = {
        "valid_range": "valid_range_pass",
        "joint_sweep_success": "joint_sweep_success",
        "non_degenerate_motion": "non_degenerate_motion_pass",
        "subtree_consistency": "subtree_consistency_pass",
        "joint_level_pass": "joint_level_pass",
    }
    expected: dict[str, Any] = {}
    for metric, field in fields.items():
        passed = sum(bool(joint.get(field, False)) for joint in joints)
        expected[metric] = {
            "passed": passed,
            "denominator": j_eval,
            "rate": passed / j_eval if j_eval else None,
        }
    measured = [
        joint
        for joint in joints
        if joint.get("fk_roundtrip_max_translation") is not None
        and joint.get("fk_roundtrip_max_rotation_rad") is not None
    ]
    expected["fk_roundtrip_error"] = {
        "max_normalized_translation": max(
            (float(joint["fk_roundtrip_max_translation"]) for joint in measured),
            default=None,
        ),
        "max_rotation_rad": max(
            (float(joint["fk_roundtrip_max_rotation_rad"]) for joint in measured),
            default=None,
        ),
        "passed_threshold": sum(bool(joint.get("fk_roundtrip_pass")) for joint in joints),
        "measured_joint_count": len(measured),
        "denominator": j_eval,
        "status": "COMPLETE" if len(measured) == j_eval else "PARTIAL",
    }
    strict = sum(bool(record.get("strict_kinematic_pass", False)) for record in records)
    expected["strict_kinematic_pass"] = {
        "passed": strict,
        "denominator": n_eval,
        "rate": strict / n_eval if n_eval else None,
    }
    _require_equal(summary.get("metrics"), expected, "table3 metrics")
    _require_equal(
        summary.get("status_counts"),
        dict(sorted(Counter(str(record.get("status")) for record in records).items())),
        "table3 status_counts",
    )


def _metric_signature(value: Mapping[str, Any]) -> dict[str, Any]:
    numerator = None
    for key in ("numerator", "passed", "collision_states", "passed_states"):
        if key in value:
            numerator = value.get(key)
            break
    return {
        "status": value.get("status"),
        "numerator": numerator,
        "denominator": value.get("denominator"),
        "rate": value.get("rate"),
        "maximum_observed_normalized": value.get("maximum_observed_normalized"),
        "measured_assets": value.get("measured_assets"),
        "observed_assets": value.get("observed_assets"),
    }


def _reaggregate_table4(
    records: list[dict[str, Any]],
    summary: Mapping[str, Any],
    *,
    n_eval: int,
    j_eval: int,
) -> None:
    try:
        expected = table4_checker.aggregate_records(records, n_eval, j_eval)
    except table4_checker.AutomationError as error:
        raise AutomationError(f"table4 reaggregation mismatch: {error}") from error
    _require_equal(summary.get("status_counts"), expected.get("status_counts"), "table4 status_counts")
    for key in (
        "status",
        "expected_states",
        "executed_states",
        "collision_geometry_assets",
        "category_count",
    ):
        _require_equal(summary.get(key), expected.get(key), f"table4 {key}")
    _require_equal(
        summary.get("state_records_expected"),
        sum(int(value) for value in expected["expected_states"].values()),
        "table4 state_records_expected",
    )
    _require_equal(
        summary.get("state_records_executed"),
        sum(int(value) for value in expected["executed_states"].values()),
        "table4 state_records_executed",
    )
    for name, metric in expected.get("metrics", {}).items():
        actual = (summary.get("metrics") or {}).get(name)
        if not isinstance(actual, Mapping):
            raise AutomationError(f"table4 reaggregation mismatch: missing metric {name}")
        _require_equal(
            _metric_signature(actual),
            _metric_signature(metric),
            f"table4 metric {name}",
        )


def _state_number(state: Mapping[str, Any], field: str, row_number: int) -> float:
    try:
        value = float(state[field])
    except (KeyError, TypeError, ValueError) as error:
        raise AutomationError(
            f"table4 state row {row_number} has invalid {field}"
        ) from error
    if not math.isfinite(value) or value < 0:
        raise AutomationError(f"table4 state row {row_number} has invalid {field}")
    return value


def _state_count(state: Mapping[str, Any], field: str, row_number: int) -> int:
    value = state.get(field)
    if isinstance(value, bool):
        raise AutomationError(f"table4 state row {row_number} has invalid {field}")
    try:
        parsed = int(value)
    except (TypeError, ValueError) as error:
        raise AutomationError(
            f"table4 state row {row_number} has invalid {field}"
        ) from error
    if parsed < 0 or parsed != value:
        raise AutomationError(f"table4 state row {row_number} has invalid {field}")
    return parsed


def _require_state_metric(
    record: Mapping[str, Any], field: str, expected: Any, record_index: int
) -> None:
    if record.get(field) != expected:
        raise AutomationError(
            f"table4 state-derived metric mismatch for asset {record_index}: {field}"
        )


def _check_table4_state_metrics(
    record: Mapping[str, Any], states: list[dict[str, Any]], record_index: int
) -> None:
    by_phase = {
        phase: [state for state in states if state.get("phase") == phase]
        for phase in ("rest", "single_joint_sweep", "multi_joint_sobol")
    }
    phase_rank = {"rest": 0, "single_joint_sweep": 1, "multi_joint_sobol": 2}
    ranks = [phase_rank[str(state["phase"])] for state in states]
    if ranks != sorted(ranks):
        raise AutomationError(
            f"table4 state sample coverage mismatch for asset {record_index}: phase order"
        )

    rest = by_phase["rest"]
    if len(rest) > 1 or (
        rest
        and (
            int(rest[0].get("sample_index", -1)) != 0
            or rest[0].get("joint_name") not in {None, ""}
        )
    ):
        raise AutomationError(
            f"table4 state sample coverage mismatch for asset {record_index}: rest"
        )
    if states and not rest:
        raise AutomationError(
            f"table4 state sample coverage mismatch for asset {record_index}: missing rest"
        )

    single_groups: dict[str, list[dict[str, Any]]] = {}
    single_order: list[str] = []
    previous_joint: str | None = None
    closed_joints: set[str] = set()
    for state in by_phase["single_joint_sweep"]:
        joint = str(state.get("joint_name") or "")
        if not joint:
            raise AutomationError(
                f"table4 state sample coverage mismatch for asset {record_index}: joint name"
            )
        if joint != previous_joint:
            if joint in closed_joints:
                raise AutomationError(
                    f"table4 state sample coverage mismatch for asset {record_index}: joint order"
                )
            if previous_joint is not None:
                closed_joints.add(previous_joint)
            single_order.append(joint)
            previous_joint = joint
        single_groups.setdefault(joint, []).append(state)
    for group_index, joint in enumerate(single_order):
        group = single_groups[joint]
        indices = [int(state.get("sample_index", -1)) for state in group]
        if indices != list(range(len(group))) or len(group) > runner.table4.SINGLE_SAMPLES:
            raise AutomationError(
                f"table4 state sample coverage mismatch for asset {record_index}: {joint}"
            )
        if group_index < len(single_order) - 1 and len(group) != runner.table4.SINGLE_SAMPLES:
            raise AutomationError(
                f"table4 state sample coverage mismatch for asset {record_index}: partial joint"
            )

    sobol = by_phase["multi_joint_sobol"]
    sobol_indices = [int(state.get("sample_index", -1)) for state in sobol]
    if sobol_indices != list(range(len(sobol))) or any(
        state.get("joint_name") not in {None, ""} for state in sobol
    ):
        raise AutomationError(
            f"table4 state sample coverage mismatch for asset {record_index}: sobol"
        )

    non_adjacent_free: dict[str, int] = {
        "rest": 0,
        "single_joint_sweep": 0,
        "multi_joint_sobol": 0,
    }
    all_pair_free_rest = False
    penetration_values: list[float] = []
    reset_values: list[float] = []
    for state_number, state in enumerate(states, 1):
        non_adjacent_count = _state_count(
            state, "non_adjacent_illegal_penetration_count", state_number
        )
        all_pair_count = _state_count(
            state, "all_pair_illegal_penetration_count", state_number
        )
        non_adjacent_free[str(state["phase"])] += int(non_adjacent_count == 0)
        if state.get("phase") == "rest":
            all_pair_free_rest = all_pair_count == 0
        metric_penetration = _state_number(
            state, "metric_max_penetration_m", state_number
        )
        metric_source = (
            "all_pair_max_penetration_m"
            if state.get("phase") == "rest"
            else "non_adjacent_max_penetration_m"
        )
        if metric_penetration != _state_number(state, metric_source, state_number):
            raise AutomationError(
                f"table4 state-derived metric mismatch for asset {record_index}: state penetration"
            )
        penetration_values.append(metric_penetration)
        reset_values.append(
            _state_number(state, "reset_readback_max_abs_error", state_number)
        )

    _require_state_metric(
        record,
        "rest_non_adjacent_free",
        non_adjacent_free["rest"],
        record_index,
    )
    _require_state_metric(
        record,
        "single_non_adjacent_free",
        non_adjacent_free["single_joint_sweep"],
        record_index,
    )
    _require_state_metric(
        record,
        "sobol_non_adjacent_free",
        non_adjacent_free["multi_joint_sobol"],
        record_index,
    )
    joint_passes = sum(
        len(group) == runner.table4.SINGLE_SAMPLES
        and all(
            int(state["non_adjacent_illegal_penetration_count"]) == 0
            for state in group
        )
        for group in single_groups.values()
    )
    _require_state_metric(
        record, "joint_single_sweep_cf_passed", joint_passes, record_index
    )

    dof = int(record.get("expected_movable_joints", -1))
    range_dof = int(record.get("range_evaluable_dof_count", -1))
    if dof < 0 or not 0 <= range_dof <= dof:
        raise AutomationError(
            f"table4 state-derived metric mismatch for asset {record_index}: DoF inventory"
        )
    expected_total = {
        "rest": int(record.get("rest_state_expected", -1)),
        "single_joint_sweep": int(record.get("single_state_expected", -1)),
        "multi_joint_sobol": int(record.get("sobol_state_expected", -1)),
    }
    executed_total = {phase: len(values) for phase, values in by_phase.items()}
    measurement_complete = bool(
        range_dof == dof
        and executed_total == expected_total
    )
    observed_complete = record.get("measurement_complete")
    if observed_complete is True and not measurement_complete:
        raise AutomationError(
            f"table4 state-derived metric mismatch for asset {record_index}: "
            "measurement_complete"
        )
    if str(record.get("status")) == "completed" and observed_complete is not True:
        raise AutomationError(
            f"table4 state-derived metric mismatch for asset {record_index}: "
            "measurement_complete"
        )
    if measurement_complete and (
        len(single_groups) != range_dof
        or any(
            len(group) != runner.table4.SINGLE_SAMPLES
            for group in single_groups.values()
        )
    ):
        raise AutomationError(
            f"table4 state sample coverage mismatch for asset {record_index}: complete sweep"
        )

    rest_all_pair = all_pair_free_rest if rest else False
    rest_non_adjacent = bool(non_adjacent_free["rest"] == 1) if rest else False
    single_pass = bool(
        executed_total["single_joint_sweep"] == expected_total["single_joint_sweep"]
        and non_adjacent_free["single_joint_sweep"]
        == expected_total["single_joint_sweep"]
    )
    sobol_pass = bool(
        dof
        and measurement_complete
        and non_adjacent_free["multi_joint_sobol"]
        == expected_total["multi_joint_sobol"]
    )
    strict_pass = bool(
        dof and measurement_complete and rest_non_adjacent and single_pass and sobol_pass
    )
    completed = str(record.get("status")) == "completed"
    for field, expected in (
        ("rest_all_pair_cf", rest_all_pair),
        ("rest_non_adjacent_cf", rest_non_adjacent),
        ("single_joint_sweep_cf", single_pass),
        ("multi_joint_sobol_cf", sobol_pass),
        ("strict_collision_pass", strict_pass),
    ):
        observed = record.get(field)
        if completed or (rest and field in {"rest_all_pair_cf", "rest_non_adjacent_cf"}):
            _require_state_metric(record, field, expected, record_index)
        elif observed is True and not expected:
            raise AutomationError(
                f"table4 state-derived metric mismatch for asset {record_index}: {field}"
            )

    if penetration_values:
        maximum_penetration = max(penetration_values)
        maximum_reset = max(reset_values)
        for field, expected in (
            ("max_penetration_m", maximum_penetration),
            ("max_reset_readback_error", maximum_reset),
        ):
            observed = record.get(field)
            if completed or observed is not None:
                try:
                    matches = math.isclose(
                        float(observed), expected, rel_tol=0.0, abs_tol=1e-15
                    )
                except (TypeError, ValueError):
                    matches = False
                if not matches:
                    raise AutomationError(
                        f"table4 state-derived metric mismatch for asset {record_index}: {field}"
                    )
        scale = record.get("object_bbox_diagonal_m")
        normalized = record.get("max_penetration_normalized")
        if completed or normalized is not None:
            try:
                expected_normalized = maximum_penetration / float(scale)
                matches = math.isclose(
                    float(normalized), expected_normalized, rel_tol=0.0, abs_tol=1e-15
                )
            except (TypeError, ValueError, ZeroDivisionError):
                matches = False
            if not matches:
                raise AutomationError(
                    f"table4 state-derived metric mismatch for asset {record_index}: normalized penetration"
                )
    elif any(
        record.get(field) is not None
        for field in (
            "max_penetration_m",
            "max_penetration_normalized",
            "max_reset_readback_error",
        )
    ):
        raise AutomationError(
            f"table4 state-derived metric mismatch for asset {record_index}: empty maxima"
        )
    if "unexecuted_state_count" in record:
        _require_state_metric(
            record,
            "unexecuted_state_count",
            sum(expected_total.values()) - len(states),
            record_index,
        )


def _verify_table4_states_stream(
    path: Path,
    records: list[Mapping[str, Any]],
    *,
    database_path: Path | None = None,
) -> dict[str, Any]:
    digest = hashlib.sha256()
    total = 0
    database: sqlite3.Connection | None = None
    database_states: sqlite3.Cursor | None = None
    if database_path is not None:
        uri = f"file:{Path(database_path).resolve(strict=True).as_posix()}?mode=ro"
        database = sqlite3.connect(uri, uri=True)
        database_states = database.execute(
            "SELECT table4_states_zlib, table4_state_count "
            "FROM results ORDER BY ordinal"
        )
    try:
        stream = path.open("rb")
    except OSError as error:
        if database is not None:
            database.close()
        raise AutomationError(f"cannot read Table 4 state records: {error}") from error
    try:
        with stream:
            for record_index, record in enumerate(records):
                declared = int(record.get("state_records_count", 0) or 0)
                expected_by_phase = {
                    "rest": int(record.get("rest_state_executed", 0) or 0),
                    "single_joint_sweep": int(record.get("single_state_executed", 0) or 0),
                    "multi_joint_sobol": int(record.get("sobol_state_executed", 0) or 0),
                }
                if declared != sum(expected_by_phase.values()):
                    raise AutomationError(
                        f"table4 state count/phase mismatch for asset {record_index}"
                    )
                states: list[dict[str, Any]] = []
                state_payload: list[bytes] = []
                phase_counts: Counter[str] = Counter()
                state_keys: set[tuple[str, str, int]] = set()
                expected_id = str(record.get("dataset_id", ""))
                expected_order = int(record.get("order", record_index))
                expected_identity = record.get("input_identity_sha256")
                for _local_index in range(declared):
                    line = stream.readline()
                    if not line:
                        raise AutomationError(
                            f"table4 state records ended early for asset {record_index}"
                        )
                    digest.update(line)
                    state_payload.append(line)
                    total += 1
                    try:
                        state = json.loads(line)
                    except json.JSONDecodeError as error:
                        raise AutomationError(
                            f"invalid table4 state JSON at global row {total}"
                        ) from error
                    if not isinstance(state, dict):
                        raise AutomationError(f"table4 state row {total} is not an object")
                    phase = str(state.get("phase", ""))
                    if phase not in expected_by_phase:
                        raise AutomationError(f"table4 state row {total} has an invalid phase")
                    if (
                        str(state.get("dataset_id", "")) != expected_id
                        or int(state.get("order", -1)) != expected_order
                        or (
                            expected_identity is not None
                            and state.get("input_identity_sha256") != expected_identity
                        )
                    ):
                        raise AutomationError(f"table4 state row {total} identity mismatch")
                    try:
                        sample_index = int(state.get("sample_index", -1))
                    except (TypeError, ValueError) as error:
                        raise AutomationError(
                            f"table4 state row {total} sample index is invalid"
                        ) from error
                    state_key = (phase, str(state.get("joint_name") or ""), sample_index)
                    if state_key in state_keys:
                        raise AutomationError(
                            f"table4 state row {total} duplicates an asset state"
                        )
                    state_keys.add(state_key)
                    phase_counts[phase] += 1
                    states.append(state)
                if database_states is not None:
                    database_row = database_states.fetchone()
                    if database_row is None:
                        raise AutomationError(
                            "table4 database state export mismatch: database ended early"
                        )
                    try:
                        database_payload = zlib.decompress(database_row[0])
                    except (TypeError, zlib.error) as error:
                        raise AutomationError(
                            f"table4 database state export mismatch at asset {record_index}"
                        ) from error
                    if (
                        int(database_row[1]) != declared
                        or database_payload != b"".join(state_payload)
                    ):
                        raise AutomationError(
                            f"table4 database state export mismatch at asset {record_index}"
                        )
                if dict(phase_counts) != {
                    phase: count for phase, count in expected_by_phase.items() if count
                }:
                    raise AutomationError(
                        f"table4 state phase totals mismatch for asset {record_index}"
                    )
                if runner.table4.canonical_sha256(states) != record.get("state_records_sha256"):
                    raise AutomationError(
                        f"table4 state hash mismatch for asset {record_index}"
                    )
                _check_table4_state_metrics(record, states, record_index)
            trailing = stream.readline()
            if trailing:
                raise AutomationError("table4 state records contain trailing rows")
            if database_states is not None and database_states.fetchone() is not None:
                raise AutomationError(
                    "table4 database state export mismatch: database has extra rows"
                )
    finally:
        if database is not None:
            database.close()
    return {"rows": total, "sha256": digest.hexdigest()}


def _iter_record_ids(path: Path, table: str) -> Iterator[tuple[int, str]]:
    try:
        stream = path.open(encoding="utf-8")
    except OSError as error:
        raise AutomationError(f"cannot read records {path}: {error}") from error
    with stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                raise AutomationError(f"blank record row: {path}:{line_number}")
            try:
                value = json.loads(line)
            except json.JSONDecodeError as error:
                raise AutomationError(f"invalid record JSON: {path}:{line_number}") from error
            if not isinstance(value, dict):
                raise AutomationError(f"record is not an object: {path}:{line_number}")
            asset_id = value.get("dataset_id") if table == "table4" else value.get("asset_id")
            ordinal = value.get(
                "order" if table == "table4" else "selection_index" if table == "table2_supplementary" else "ordinal"
            )
            try:
                ordinal = int(ordinal)
            except (TypeError, ValueError) as error:
                raise AutomationError(f"record ordinal is invalid: {path}:{line_number}") from error
            yield ordinal, str(asset_id or "")


def _summary_n(summary: Mapping[str, Any]) -> int:
    cohort = summary.get("cohort")
    value = summary.get("n_eval", summary.get("N_eval"))
    if value is None and isinstance(cohort, Mapping):
        value = cohort.get("N_eval")
    try:
        return int(value)
    except (TypeError, ValueError) as error:
        raise AutomationError("summary N_eval is missing") from error


def _check_database_roster(
    database_path: Path,
    roster_manifest_path: Path,
    *,
    n_eval: int,
    j_eval: int,
    category_count: int,
) -> list[str]:
    uri = f"file:{Path(database_path).resolve(strict=True).as_posix()}?mode=ro"
    expected_rows = pva_roster.iter_roster_rows(roster_manifest_path)
    identities: list[str] = []
    observed_joints = 0
    categories: set[str] = set()
    with sqlite3.connect(uri, uri=True) as connection:
        if connection.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
            raise AutomationError("result database integrity check failed")
        asset_rows = connection.execute(
            "SELECT ordinal, asset_id, category, joint_count, row_sha256, row_json "
            "FROM assets ORDER BY ordinal"
        )
        result_rows = connection.execute(
            "SELECT ordinal, asset_id FROM results ORDER BY ordinal"
        )
        for ordinal in range(n_eval):
            try:
                expected = next(expected_rows)
            except StopIteration as error:
                raise AutomationError("database roster mismatch: frozen roster ended early") from error
            observed = asset_rows.fetchone()
            result_identity = result_rows.fetchone()
            if observed is None or result_identity is None:
                raise AutomationError("database roster mismatch: database ended early")
            row_text = json.dumps(
                expected,
                sort_keys=True,
                separators=(",", ":"),
                ensure_ascii=True,
                allow_nan=False,
            )
            expected_tuple = (
                ordinal,
                str(expected.get("asset_id", "")),
                str(expected.get("raw_category", expected.get("category", ""))),
                int(expected.get("joint_count", -1)),
                hashlib.sha256(row_text.encode("utf-8")).hexdigest(),
                row_text,
            )
            if observed != expected_tuple or result_identity != expected_tuple[:2]:
                raise AutomationError(f"database roster mismatch at ordinal {ordinal}")
            identities.append(expected_tuple[1])
            categories.add(expected_tuple[2])
            observed_joints += expected_tuple[3]
        if asset_rows.fetchone() is not None or result_rows.fetchone() is not None:
            raise AutomationError("database roster mismatch: database has extra rows")
    if observed_joints != j_eval or len(categories) != category_count:
        raise AutomationError("database roster mismatch: denominator drift")
    return identities


def _check_record_source_binding(
    table: str,
    record: Mapping[str, Any],
    roster_row: Mapping[str, Any],
    ordinal: int,
) -> None:
    expected_id = str(roster_row.get("asset_id", ""))
    expected_category = str(
        roster_row.get("raw_category", roster_row.get("category", ""))
    )
    expected_primary = roster_row.get("primary_urdf_sha256")
    expected_package = roster_row.get("package_binding_sha256")
    expected_joints = int(roster_row.get("joint_count", -1))
    observed_id = (
        record.get("dataset_id") if table == "table4" else record.get("asset_id")
    )
    ordinal_field = (
        "order"
        if table == "table4"
        else "selection_index"
        if table == "table2_supplementary"
        else "ordinal"
    )
    category_field = "raw_category" if table in {"table1", "table2"} else "category"
    common_bindings = {
        "asset_id": (str(observed_id or ""), expected_id),
        "ordinal": (record.get(ordinal_field), ordinal),
        "category": (str(record.get(category_field, "")), expected_category),
        "package": (record.get("package_binding_sha256"), expected_package),
    }
    for field, (observed, expected) in common_bindings.items():
        if observed != expected:
            raise AutomationError(
                f"{table} source binding mismatch at {ordinal}: {field}"
            )
    if not isinstance(record.get("package_binding_verified"), bool):
        raise AutomationError(
            f"{table} source binding mismatch at {ordinal}: package verification state"
        )

    if table in {"table1", "table2", "table3"}:
        observed_primary = record.get("primary_urdf_sha256")
    else:
        observed_primary = record.get("expected_primary_urdf_sha256")
    if observed_primary != expected_primary:
        raise AutomationError(
            f"{table} source binding mismatch at {ordinal}: primary URDF"
        )

    if table == "table1":
        observed_movable = record.get("non_fixed_joint_count")
        if int(record.get("roster_joint_count", -1)) != expected_joints or (
            record.get("parse_success") is True
            and int(observed_movable) != expected_joints
        ):
            raise AutomationError(
                f"{table} source binding mismatch at {ordinal}: joint count"
            )
    elif table == "table2_supplementary":
        if int(record.get("expected_movable_joints", -1)) != expected_joints:
            raise AutomationError(
                f"{table} source binding mismatch at {ordinal}: joint count"
            )
    elif table == "table3":
        joints = record.get("joints")
        if (
            int(record.get("roster_joint_count", -1)) != expected_joints
            or int(record.get("declared_joint_count", -1)) != expected_joints
            or not isinstance(joints, list)
            or len(joints) != expected_joints
            or int(record.get("sample_count_expected", -1)) != 21
        ):
            raise AutomationError(
                f"{table} source binding mismatch at {ordinal}: joint inventory"
            )
    elif table == "table4":
        expected_job = runner._table4_job(roster_row)
        if (
            int(record.get("expected_movable_joints", -1)) != expected_joints
            or int(record.get("rest_state_expected", -1)) != 1
            or int(record.get("single_state_expected", -1))
            != runner.table4.SINGLE_SAMPLES * expected_joints
            or int(record.get("sobol_state_expected", -1))
            != (runner.table4.SOBOL_SAMPLES if expected_joints else 0)
            or record.get("input_identity_sha256")
            != expected_job["input_identity_sha256"]
        ):
            raise AutomationError(
                f"{table} source binding mismatch at {ordinal}: evaluation input"
            )


def _check_database_records(
    database_path: Path,
    column: str,
    records: list[Mapping[str, Any]],
    *,
    table: str,
) -> None:
    if column not in set(runner.RESULT_COLUMNS.values()):
        raise AutomationError(f"unknown database result column: {column}")
    uri = f"file:{Path(database_path).resolve(strict=True).as_posix()}?mode=ro"
    with sqlite3.connect(uri, uri=True) as connection:
        cursor = connection.execute(
            f"SELECT r.{column}, a.row_json "
            "FROM results AS r JOIN assets AS a ON a.ordinal = r.ordinal "
            "ORDER BY r.ordinal"
        )
        for ordinal, record in enumerate(records):
            row = cursor.fetchone()
            if row is None:
                raise AutomationError(f"{table} result database ended early")
            try:
                database_record = json.loads(row[0])
                roster_row = json.loads(row[1])
            except json.JSONDecodeError as error:
                raise AutomationError(f"{table} result database JSON is invalid") from error
            if database_record != record or not isinstance(roster_row, dict):
                raise AutomationError(f"{table} result database record mismatch at {ordinal}")
            _check_record_source_binding(table, database_record, roster_row, ordinal)
        if cursor.fetchone() is not None:
            raise AutomationError(f"{table} result database has extra records")


def _check_checkpoint_binding(
    name: str,
    checkpoint: Mapping[str, Any],
    binding: Mapping[str, Any],
    *,
    n_eval: int,
    j_eval: int,
    records_hash: str,
    summary_hash: str,
    state_count: int | None = None,
    state_hash: str | None = None,
) -> None:
    if checkpoint.get("schema_version") != "pva_table_checkpoint_v1":
        raise AutomationError(f"{name} checkpoint schema mismatch")
    if checkpoint.get("state") != "complete":
        raise AutomationError(f"{name} checkpoint is incomplete")
    if int(checkpoint.get("records", -1)) != n_eval:
        raise AutomationError(f"{name} checkpoint record count mismatch")
    if int(checkpoint.get("N_eval", -1)) != n_eval:
        raise AutomationError(f"{name} checkpoint N_eval mismatch")
    if int(checkpoint.get("J_eval", -1)) != j_eval:
        raise AutomationError(f"{name} checkpoint J_eval mismatch")
    if (
        checkpoint.get("records_sha256") != records_hash
        or checkpoint.get("records_sha256") != binding.get("records_sha256")
    ):
        raise AutomationError(f"{name} checkpoint records hash mismatch")
    if (
        checkpoint.get("summary_sha256") != summary_hash
        or checkpoint.get("summary_sha256") != binding.get("summary_sha256")
    ):
        raise AutomationError(f"{name} checkpoint summary hash mismatch")
    if name == "table4":
        if state_count is None or state_hash is None:
            raise AutomationError("table4 checkpoint state binding is missing")
        if (
            int(checkpoint.get("state_records", -1)) != state_count
            or int(binding.get("state_records_count", -1)) != state_count
        ):
            raise AutomationError("table4 checkpoint state count mismatch")
        if (
            checkpoint.get("state_records_sha256") != state_hash
            or binding.get("state_records_sha256") != state_hash
        ):
            raise AutomationError("table4 checkpoint state hash mismatch")


def _check_artifact_manifest(
    root: Path,
    name: str,
    binding: Mapping[str, Any],
    observed_artifacts: Mapping[str, tuple[int, str]],
) -> None:
    manifest_path = _resolve(
        root, binding.get("artifact_manifest"), f"{name} artifact manifest"
    )
    if common.sha256_file(manifest_path) != binding.get("artifact_manifest_sha256"):
        raise AutomationError(f"{name} artifact manifest hash mismatch")
    manifest = _json(manifest_path)
    if manifest.get("schema_version") != common.ARTIFACT_SCHEMA_VERSION:
        raise AutomationError(f"{name} artifact manifest schema mismatch")
    _check_self_hash(
        manifest,
        "artifact_manifest_content_sha256",
        f"{name} artifact manifest",
    )
    if (
        manifest.get("artifact_manifest_content_sha256")
        != binding.get("artifact_manifest_content_sha256")
    ):
        raise AutomationError(f"{name} artifact manifest content binding mismatch")
    expected_names = {
        Path(str(binding["records"])).name,
        Path(str(binding["summary"])).name,
        Path(str(binding["checkpoint"])).name,
    }
    if name == "table4":
        expected_names.add(Path(str(binding["state_records"])).name)
    entries = manifest.get("artifacts")
    if not isinstance(entries, list):
        raise AutomationError(f"{name} artifact manifest entries are missing")
    observed_names: set[str] = set()
    for entry in entries:
        if not isinstance(entry, Mapping):
            raise AutomationError(f"{name} artifact manifest entry is invalid")
        artifact = _resolve(
            manifest_path.parent,
            entry.get("path"),
            f"{name} artifact manifest entry",
        )
        relative_name = artifact.relative_to(manifest_path.parent).as_posix()
        if relative_name in observed_names:
            raise AutomationError(f"{name} artifact manifest has duplicate entries")
        observed_names.add(relative_name)
        observed = observed_artifacts.get(relative_name)
        if observed is None:
            raise AutomationError(f"{name} artifact manifest has an unknown artifact")
        observed_size, observed_hash = observed
        if artifact.stat().st_size != observed_size or observed_size != int(
            entry.get("size", -1)
        ):
            raise AutomationError(f"{name} artifact manifest size mismatch")
        if observed_hash != entry.get("sha256"):
            raise AutomationError(f"{name} artifact manifest artifact hash mismatch")
    if observed_names != expected_names:
        raise AutomationError(f"{name} artifact manifest roster mismatch")


def _check_table(
    root: Path,
    name: str,
    binding: Mapping[str, Any],
    expected_ids: list[str],
    *,
    database_path: Path,
    n_release: int,
    release_category_count: int,
    n_eval: int,
    j_eval: int,
    category_count: int,
) -> dict[str, Any]:
    records = _resolve(root, binding.get("records"), f"{name} records")
    records_hash, records_lines = _sha_and_lines(records)
    if records_hash != binding.get("records_sha256"):
        raise AutomationError(f"{name} records hash mismatch")
    if records_lines != n_eval or int(binding.get("records_count", -1)) != n_eval:
        raise AutomationError(f"{name} records denominator mismatch")
    observed_ids: list[str] = []
    for expected_ordinal, (ordinal, asset_id) in enumerate(_iter_record_ids(records, name)):
        if ordinal != expected_ordinal:
            raise AutomationError(f"{name} record ordinal mismatch at {expected_ordinal}")
        observed_ids.append(asset_id)
    if observed_ids != expected_ids:
        raise AutomationError(f"{name} record identity/order mismatch")
    record_values = _records(records)
    _check_database_records(
        database_path,
        runner.RESULT_COLUMNS[name],
        record_values,
        table=name,
    )

    summary_path = _resolve(root, binding.get("summary"), f"{name} summary")
    if common.sha256_file(summary_path) != binding.get("summary_sha256"):
        raise AutomationError(f"{name} summary hash mismatch")
    summary = _json(summary_path)
    _check_self_hash(summary, "summary_content_sha256", f"{name} summary")
    if _summary_n(summary) != n_eval:
        raise AutomationError(f"{name} summary N_eval mismatch")
    if name != "table1" and int(summary.get("j_eval", j_eval)) != j_eval:
        raise AutomationError(f"{name} summary J_eval mismatch")

    if name == "table1":
        _reaggregate_table1(
            record_values,
            summary,
            n_eval=n_eval,
            n_release=n_release,
            release_category_count=release_category_count,
        )
    elif name == "table2":
        _reaggregate_table2(record_values, summary, n_eval=n_eval)
    elif name == "table2_supplementary":
        _reaggregate_table2_supplementary(
            record_values, summary, n_eval=n_eval, j_eval=j_eval
        )
    elif name == "table3":
        _reaggregate_table3(record_values, summary, n_eval=n_eval, j_eval=j_eval)
    elif name == "table4":
        _reaggregate_table4(
            record_values,
            summary,
            n_eval=n_eval,
            j_eval=j_eval,
        )

    result = {
        "records": n_eval,
        "records_sha256": records_hash,
        "summary_sha256": binding.get("summary_sha256"),
        "checkpoint_sha256": binding.get("checkpoint_sha256"),
    }
    state_count: int | None = None
    state_hash: str | None = None
    observed_artifacts = {
        records.name: (records.stat().st_size, records_hash),
        summary_path.name: (
            summary_path.stat().st_size,
            str(binding.get("summary_sha256")),
        ),
    }
    if name == "table4":
        states = _resolve(root, binding.get("state_records"), "table4 state records")
        state_info = _verify_table4_states_stream(
            states, record_values, database_path=database_path
        )
        states_hash = str(state_info["sha256"])
        states_lines = int(state_info["rows"])
        if states_hash != binding.get("state_records_sha256"):
            raise AutomationError("table4 state records hash mismatch")
        if states_lines != int(binding.get("state_records_count", -1)):
            raise AutomationError("table4 state record count mismatch")
        if states_lines != int(summary.get("state_records_executed", -1)):
            raise AutomationError("table4 summary state count mismatch")
        result["state_records"] = states_lines
        result["state_records_sha256"] = states_hash
        state_count = states_lines
        state_hash = states_hash
        observed_artifacts[states.name] = (states.stat().st_size, states_hash)

    checkpoint_path = _resolve(root, binding.get("checkpoint"), f"{name} checkpoint")
    if common.sha256_file(checkpoint_path) != binding.get("checkpoint_sha256"):
        raise AutomationError(f"{name} checkpoint hash mismatch")
    checkpoint = _json(checkpoint_path)
    _check_self_hash(checkpoint, "checkpoint_content_sha256", f"{name} checkpoint")
    _check_checkpoint_binding(
        name,
        checkpoint,
        binding,
        n_eval=n_eval,
        j_eval=j_eval,
        records_hash=records_hash,
        summary_hash=str(binding.get("summary_sha256")),
        state_count=state_count,
        state_hash=state_hash,
    )
    observed_artifacts[checkpoint_path.name] = (
        checkpoint_path.stat().st_size,
        str(binding.get("checkpoint_sha256")),
    )
    _check_artifact_manifest(root, name, binding, observed_artifacts)
    return result


def check_results(
    root: Path,
    *,
    expected_n: int | None = None,
    expected_categories: int | None = None,
) -> dict[str, Any]:
    root = Path(root).resolve(strict=True)
    receipt_path = root / "full_release_receipt.json"
    receipt = _json(receipt_path)
    if receipt.get("schema_version") != runner.RECEIPT_SCHEMA_VERSION:
        raise AutomationError("PV-A receipt schema mismatch")
    _check_self_hash(receipt, "receipt_content_sha256", "PV-A receipt")
    n_release = int(receipt.get("N_release", -1))
    n_eval = int(receipt.get("N_eval", -1))
    j_eval = int(receipt.get("J_eval", -1))
    category_count = int(receipt.get("eval_category_count", -1))
    if expected_n is not None and n_eval != expected_n:
        raise AutomationError(f"PV-A N_eval mismatch: {n_eval} != {expected_n}")
    if expected_categories is not None and category_count != expected_categories:
        raise AutomationError(
            f"PV-A category count mismatch: {category_count} != {expected_categories}"
        )
    roster_path = Path(str(receipt.get("roster_manifest", ""))).resolve(strict=True)
    if common.sha256_file(roster_path) != receipt.get("roster_manifest_sha256"):
        raise AutomationError("PV-A roster manifest hash mismatch")
    try:
        roster = pva_roster.load_roster_manifest(roster_path, verify_rows=False)
    except ValueError as error:
        raise AutomationError(str(error)) from error
    if roster.get("manifest_content_sha256") != receipt.get("roster_manifest_content_sha256"):
        raise AutomationError("PV-A roster content binding mismatch")
    if roster.get("builder_source_hashes") != pva_roster._builder_source_hashes():
        raise AutomationError("PV-A roster builder source hash drift")
    if n_release != int(roster.get("N_release", -1)):
        raise AutomationError("PV-A release denominator binding mismatch")
    if int(receipt.get("release_category_count", -1)) != int(
        roster.get("release_category_count", -1)
    ):
        raise AutomationError("PV-A release category binding mismatch")

    execution_path = _resolve(root, receipt.get("execution_manifest"), "execution manifest")
    if common.sha256_file(execution_path) != receipt.get("execution_manifest_sha256"):
        raise AutomationError("execution manifest hash mismatch")
    execution = _json(execution_path)
    _check_self_hash(execution, "manifest_content_sha256", "execution manifest")
    if int(execution.get("N_eval", -1)) != n_eval or int(execution.get("J_eval", -1)) != j_eval:
        raise AutomationError("execution manifest denominator mismatch")
    if int(execution.get("category_count", -1)) != category_count:
        raise AutomationError("execution manifest category denominator mismatch")
    if execution.get("classification") != receipt.get("classification"):
        raise AutomationError("execution/receipt classification mismatch")
    try:
        execution_roster = Path(str(execution.get("roster_manifest", ""))).resolve(
            strict=True
        )
    except OSError as error:
        raise AutomationError("execution roster binding is missing") from error
    if (
        execution_roster != roster_path
        or execution.get("roster_manifest_sha256")
        != receipt.get("roster_manifest_sha256")
        or execution.get("roster_manifest_content_sha256")
        != receipt.get("roster_manifest_content_sha256")
    ):
        raise AutomationError("execution/receipt roster binding mismatch")
    if execution.get("source_hashes") != runner._source_hashes():
        raise AutomationError("execution evaluator source hash drift")
    if execution.get("runtime_identity") != runner.table4.runtime_identity():
        raise AutomationError("execution runtime identity drift")
    protocol = execution.get("protocol")
    if not isinstance(protocol, Mapping):
        raise AutomationError("execution protocol binding is missing")
    protocol_snapshot = _resolve(root, protocol.get("snapshot"), "protocol snapshot")
    if common.sha256_file(protocol_snapshot) != protocol.get("snapshot_sha256"):
        raise AutomationError("protocol snapshot hash mismatch")

    looks_like_full_release = (
        n_release == pva_roster.EXPECTED_RELEASE_COUNT
        and n_eval == pva_roster.EXPECTED_RELEASE_COUNT
        and int(receipt.get("release_category_count", -1))
        == pva_roster.EXPECTED_CATEGORY_COUNT
    )
    if receipt.get("classification") == "FORMAL_FULL_RELEASE" or looks_like_full_release:
        if {
            receipt.get("classification"),
            execution.get("classification"),
            roster.get("classification"),
        } != {"FORMAL_FULL_RELEASE"}:
            raise AutomationError("full PV-A run is not consistently classified as formal")
        if not looks_like_full_release or category_count != pva_roster.EXPECTED_CATEGORY_COUNT:
            raise AutomationError("formal PV-A receipt does not cover all 302,440 assets/531 categories")
        if execution.get("limit") is not None or execution.get("run_standard_parser") is not True:
            raise AutomationError("formal PV-A execution must enable the standard parser without a limit")
        source_bindings = roster.get("source_bindings") or {}
        if source_bindings.get("source_manifest_sha256") != pva_roster.EXPECTED_MANIFEST_SHA256:
            raise AutomationError("formal PV-A source manifest identity mismatch")

    database_path = _resolve(root, receipt.get("result_database"), "result database")
    if database_path.stat().st_size != int(receipt.get("result_database_bytes", -1)):
        raise AutomationError("result database size mismatch")
    if common.sha256_file(database_path) != receipt.get("result_database_sha256"):
        raise AutomationError("result database hash mismatch")
    expected_ids = _check_database_roster(
        database_path,
        roster_path,
        n_eval=n_eval,
        j_eval=j_eval,
        category_count=category_count,
    )

    tables = receipt.get("tables")
    if not isinstance(tables, Mapping) or set(tables) != set(runner.TABLE_NAMES):
        raise AutomationError("PV-A receipt table roster mismatch")
    checked = {
        name: _check_table(
            root,
            name,
            tables[name],
            expected_ids,
            database_path=database_path,
            n_release=n_release,
            release_category_count=int(receipt.get("release_category_count", -1)),
            n_eval=n_eval,
            j_eval=j_eval,
            category_count=category_count,
        )
        for name in runner.TABLE_NAMES
    }
    return {
        "schema_version": "pva_table1234_automation_check_v1",
        "all_pass": True,
        "root": str(root),
        "N_eval": n_eval,
        "J_eval": j_eval,
        "category_count": category_count,
        "tables_checked": len(checked),
        "tables": checked,
        "receipt_sha256": common.sha256_file(receipt_path),
        "database_integrity": "ok",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--expected-n", type=int)
    parser.add_argument("--expected-categories", type=int)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--pytest", action="store_true")
    args = parser.parse_args(argv)
    try:
        report = check_results(
            args.root,
            expected_n=args.expected_n,
            expected_categories=args.expected_categories,
        )
        if args.pytest:
            tests_root = Path(__file__).resolve().parents[1] / "tests"
            command = [
                sys.executable,
                "-m",
                "pytest",
                "-q",
                "--basetemp",
                str(
                    Path(__file__).resolve().parents[1]
                    / "runtime"
                    / "pytest_tmp"
                    / "pva_checker_contract"
                ),
                *(str(tests_root / name) for name in CONTRACT_TEST_FILES),
            ]
            completed = subprocess.run(command, text=True, capture_output=True, check=False)
            report["pytest"] = {
                "returncode": completed.returncode,
                "stdout": completed.stdout,
                "stderr": completed.stderr,
            }
            if completed.returncode != 0:
                raise AutomationError("focused pytest contract failed")
        if args.json_out is not None:
            common._atomic_write_bytes(
                args.json_out, (_canonical(report) + "\n").encode("utf-8")
            )
    except Exception as error:  # noqa: BLE001
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps(report, sort_keys=True))
    return 0


def _canonical(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["AutomationError", "check_results"]
