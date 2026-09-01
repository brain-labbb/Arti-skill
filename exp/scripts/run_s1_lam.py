#!/usr/bin/env python3
"""Evaluate Supplementary Table S1 on the frozen LAM Table 3 cohort."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import time
from typing import Any, Mapping


SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from exp.scripts import lam_supplementary_static as static_atoms  # noqa: E402

TABLE3_RUN = (
    REPO
    / "exp/runtime/urdf_table3_lam_n800_seed20260813_20260814T022400Z_v3"
)
TABLE3_MANIFEST = TABLE3_RUN / "manifest.json"
TABLE3_RECORDS = TABLE3_RUN / "asset_records.jsonl"
RELEASE_ROOT = REPO / "exp/Articulated-Object-Code/released_outputs"
EXPECTED_TABLE3_MANIFEST_SHA256 = (
    "7e16683bfe4e4f37d7972082d8512713c1d8d1ae4ce142b75bf7dfb0509b9951"
)
EXPECTED_TABLE3_MANIFEST_CONTENT_SHA256 = (
    "f8f7fe4da5634d4f806e793c0da919689eab25be1ce0bbed7e2232f3453d15c2"
)
EXPECTED_TABLE3_RECORDS_SHA256 = (
    "7ef1c38d61bc780e41f62c7dd359e66f0bfeabe655c7453c93e2ea9830122d94"
)
EXPECTED_ORDERED_ASSET_KEYS_SHA256 = (
    "643aa5b76ac61f57dd943bee26444a3525c01201a8dff3443763a7fd8d8267d3"
)
EXPECTED_N_EVAL = 800
EXPECTED_J_EVAL = 2395
TABLE4_RUN = REPO / "exp/runtime/urdf_table4_lam_n800_20260814"
TABLE4_MANIFEST = TABLE4_RUN / "frozen_manifest.json"
TABLE4_ASSET_RECORDS = TABLE4_RUN / "asset_records.json"
TABLE4_STATE_RECORDS = TABLE4_RUN / "state_records.jsonl"
TABLE4_VERIFICATION = TABLE4_RUN / "verification.json"
EXPECTED_TABLE4_MANIFEST_SHA256 = (
    "8adc7d8698eaeab5ee5a62d881ed50d4e65c5dc80c9d1d8ae0f4a4a204474594"
)
EXPECTED_TABLE4_MANIFEST_CONTENT_SHA256 = (
    "9a46a1cb7668666cf3c485cc35086cdd79a113d23a8b00625ede012c8b039d2d"
)
EXPECTED_TABLE4_ASSET_RECORDS_SHA256 = (
    "15423f8646be26dd01fe9d1ca5c0a1b7b1f454349e77d1562827e719c0d1d014"
)
EXPECTED_TABLE4_STATE_RECORDS_SHA256 = (
    "ac62b73d71530982a63c1e8cf345cfda126608aa6e42ce9710383daace2af257"
)
EXPECTED_TABLE4_VERIFICATION_SHA256 = (
    "e74ed91dca984af8aba900cf3915b490fb1298e5c2bc539af7ade43570edbc51"
)
EXPECTED_TABLE4_PROTOCOL_ID = "urdf_sim_ready_table4_lam_n800_v1"
EXPECTED_FORMAL_STRICT_PASSED = 91
EXPECTED_FORMAL_ALLOWANCE_MEASURED_ASSETS = 770
EXPECTED_FORMAL_ALLOWANCE_ELIGIBLE_PAIRS = 17_939
SINGLE_JOINT_SAMPLES = 21
SOBOL_SAMPLES = 64
SOBOL_SEED = 20260813
ZERO_WIDTH_TOLERANCE = 1e-12
PENETRATION_THRESHOLD_M = 1e-6
RESET_TOLERANCE = 1e-9
DEFAULT_WORKERS = 4
PROTOCOL_ID = "s1_lam_table3cohort_n800_seed20260813_v1"
SCHEMA_VERSION = "supplementary-s1-lam/v1"
DATASET = "LAM released outputs"
PROTOCOL_DOCUMENT = REPO / "exp/URDF-Sim-Ready-Automatic-Evaluation.md"
EXPECTED_PROTOCOL_SNAPSHOT_SHA256 = (
    "7e4558dee779b39759a4e622c6ff412d9ebd0f6c94e4905d27b9dc4021fbf70b"
)
VERIFIER_PATH = SCRIPT.with_name("verify_s1_lam.py")
DEFAULT_OUTPUT_PARENT = REPO / "exp/runtime"

TABLE4_IDENTITY_FIELDS = (
    "protocol_id",
    "order",
    "dataset_id",
    "asset_key",
    "category",
    "input_identity_sha256",
    "selection_rank",
    "selection_hash",
    "tier",
    "rel_path",
    "object_release_id",
    "package_relpath",
    "model_urdf_sha256",
    "package_content_manifest_sha256",
    "source_record_sha256",
    "source_manifest_record_sha256",
)


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                row = json.loads(stripped)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSONL at {path}:{line_number}") from exc
            if not isinstance(row, dict):
                raise ValueError(f"JSONL row must be an object at {path}:{line_number}")
            rows.append(row)
    return rows


def package_binding(package: Path) -> dict[str, Any]:
    package_lexical = Path(os.path.abspath(str(package)))
    if package_lexical.is_symlink():
        raise ValueError(f"package root is a symlink: {package_lexical}")
    package_resolved = package_lexical.resolve(strict=True)
    if not package_resolved.is_dir():
        raise ValueError(f"package is not a directory: {package_resolved}")
    files: list[dict[str, Any]] = []
    for current_raw, directory_names, file_names in os.walk(
        package_resolved, followlinks=False
    ):
        current = Path(current_raw)
        directory_names.sort()
        file_names.sort()
        for name in directory_names:
            path = current / name
            if path.is_symlink():
                raise ValueError(
                    f"package contains directory symlink: {path.relative_to(package_resolved)}"
                )
        for name in file_names:
            path = current / name
            relative = path.relative_to(package_resolved).as_posix()
            if path.is_symlink():
                raise ValueError(f"package contains file symlink: {relative}")
            resolved = path.resolve(strict=True)
            try:
                resolved.relative_to(package_resolved)
            except ValueError as exc:
                raise ValueError(f"package file escapes package: {relative}") from exc
            if not resolved.is_file():
                raise ValueError(f"package entry is not a regular file: {relative}")
            files.append(
                {
                    "path": relative,
                    "bytes": resolved.stat().st_size,
                    "sha256": sha256_file(resolved),
                }
            )
    return {
        "file_count": len(files),
        "total_bytes": sum(int(row["bytes"]) for row in files),
        "files": files,
        "content_manifest_sha256": canonical_sha256(files),
    }


def resolve_release_package(release_root: Path, rel_path: str) -> tuple[Path, Path]:
    root_lexical = Path(os.path.abspath(str(release_root)))
    if root_lexical.is_symlink():
        raise ValueError(f"release root is a symlink: {root_lexical}")
    root_resolved = root_lexical.resolve(strict=True)
    relative = Path(rel_path)
    if relative.is_absolute() or not relative.parts or any(
        part in {"", ".", ".."} for part in relative.parts
    ):
        raise ValueError(f"invalid package relative path: {rel_path!r}")

    package_lexical = root_lexical
    for part in relative.parts:
        package_lexical /= part
        if package_lexical.is_symlink():
            raise ValueError(
                f"package path contains symlink: {package_lexical.relative_to(root_lexical)}"
            )
    package = package_lexical.resolve(strict=True)
    try:
        package.relative_to(root_resolved)
    except ValueError as exc:
        raise ValueError(f"package escapes release root: {rel_path}") from exc
    if not package.is_dir():
        raise ValueError(f"package is not a directory: {rel_path}")

    urdf_lexical = package_lexical / "generated.urdf"
    if urdf_lexical.is_symlink():
        raise ValueError(f"generated.urdf is a symlink: {rel_path}")
    urdf_path = urdf_lexical.resolve(strict=True)
    if urdf_path.parent != package or not urdf_path.is_file():
        raise ValueError(f"invalid generated.urdf: {rel_path}")
    return package, urdf_path


def bind_table3_rows(
    manifest_rows: list[Mapping[str, Any]],
    asset_rows: list[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    if len(manifest_rows) != len(asset_rows):
        raise ValueError("Table 3 manifest/asset record count mismatch")
    fields = (
        "asset_key",
        "category",
        "object_release_id",
        "rel_path",
        "selection_hash",
        "selection_rank",
        "tier",
        "urdf_sha256",
    )
    bound: list[dict[str, Any]] = []
    for index, (manifest_row, asset_row) in enumerate(
        zip(manifest_rows, asset_rows, strict=True)
    ):
        for field in fields:
            if asset_row.get(field) != manifest_row.get(field):
                raise ValueError(f"Table 3 asset record {field} mismatch at index {index}")
        hint = manifest_row.get("declared_joint_count_hint")
        declared = asset_row.get("declared_joint_count")
        if (
            isinstance(hint, bool)
            or not isinstance(hint, int)
            or hint < 0
            or declared != hint
        ):
            raise ValueError(f"Table 3 declared_joint_count mismatch at index {index}")
        bound.append(
            {
                **dict(manifest_row),
                "table3_manifest_record": dict(manifest_row),
                "table3_asset_record": dict(asset_row),
            }
        )
    return bound


def load_frozen_cohort(*, limit: int | None = None) -> dict[str, Any]:
    if limit is not None and (
        isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= EXPECTED_N_EVAL
    ):
        raise ValueError(f"limit must be within 1..{EXPECTED_N_EVAL}")
    if sha256_file(TABLE3_MANIFEST) != EXPECTED_TABLE3_MANIFEST_SHA256:
        raise RuntimeError("frozen Table 3 manifest SHA256 mismatch")
    if sha256_file(TABLE3_RECORDS) != EXPECTED_TABLE3_RECORDS_SHA256:
        raise RuntimeError("frozen Table 3 asset records SHA256 mismatch")

    manifest = json.loads(TABLE3_MANIFEST.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError("Table 3 manifest root must be an object")
    content = {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    if (
        manifest.get("manifest_content_sha256")
        != EXPECTED_TABLE3_MANIFEST_CONTENT_SHA256
        or canonical_sha256(content) != EXPECTED_TABLE3_MANIFEST_CONTENT_SHA256
    ):
        raise RuntimeError("frozen Table 3 manifest content SHA256 mismatch")
    manifest_rows = manifest.get("records")
    if not isinstance(manifest_rows, list) or len(manifest_rows) != EXPECTED_N_EVAL:
        raise ValueError(f"Table 3 manifest must contain {EXPECTED_N_EVAL} records")
    asset_rows = load_jsonl(TABLE3_RECORDS)
    if len(asset_rows) != EXPECTED_N_EVAL:
        raise ValueError(f"Table 3 asset records must contain {EXPECTED_N_EVAL} rows")
    asset_ranks = [row.get("selection_rank") for row in asset_rows]
    if sorted(asset_ranks) != list(range(1, EXPECTED_N_EVAL + 1)):
        raise ValueError("Table 3 asset records do not cover selection_rank 1..800 exactly")
    asset_rows.sort(key=lambda row: int(row["selection_rank"]))
    bound = bind_table3_rows(manifest_rows, asset_rows)

    ranks = [row.get("selection_rank") for row in bound]
    if ranks != list(range(1, EXPECTED_N_EVAL + 1)):
        raise ValueError("Table 3 cohort is not in selection_rank 1..800 order")
    ordered_keys = [str(row["asset_key"]) for row in bound]
    if len(set(ordered_keys)) != EXPECTED_N_EVAL:
        raise ValueError("Table 3 cohort asset keys are not unique")
    ordered_hash = canonical_sha256(ordered_keys)
    if ordered_hash != EXPECTED_ORDERED_ASSET_KEYS_SHA256:
        raise RuntimeError("Table 3 ordered asset keys SHA256 mismatch")
    if sum(int(row["declared_joint_count_hint"]) for row in bound) != EXPECTED_J_EVAL:
        raise RuntimeError("Table 3 frozen joint denominator mismatch")

    records: list[dict[str, Any]] = []
    for index, row in enumerate(bound):
        try:
            package, urdf_path = resolve_release_package(
                RELEASE_ROOT,
                str(row["rel_path"]),
            )
        except (OSError, ValueError) as exc:
            raise ValueError(f"invalid Table 3 package at index {index}: {exc}") from exc
        if str(row.get("urdf_path")) != str(urdf_path):
            raise ValueError(f"Table 3 urdf_path mismatch at index {index}")
        if sha256_file(urdf_path) != row.get("urdf_sha256"):
            raise ValueError(f"Table 3 generated.urdf SHA256 mismatch at index {index}")
        records.append(
            {
                **row,
                "selection_index": index,
                "package": package,
                "urdf_path": urdf_path,
            }
        )
    selected = records if limit is None else records[:limit]
    return {
        "records": selected,
        "full_size": len(records),
        "ordered_asset_keys_sha256": ordered_hash,
        "manifest_file_sha256": EXPECTED_TABLE3_MANIFEST_SHA256,
        "manifest_content_sha256": EXPECTED_TABLE3_MANIFEST_CONTENT_SHA256,
        "asset_records_file_sha256": EXPECTED_TABLE3_RECORDS_SHA256,
    }


def _nonnegative_int(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a non-negative integer")
    return value


def _joint_interval(row: Mapping[str, Any]) -> tuple[float, float]:
    if row.get("type") == "continuous":
        return -math.pi, math.pi
    lower = row.get("lower")
    upper = row.get("upper")
    if (
        isinstance(lower, bool)
        or isinstance(upper, bool)
        or not isinstance(lower, (int, float))
        or not isinstance(upper, (int, float))
    ):
        raise ValueError(f"joint {row.get('name', '<unnamed>')} has no finite range")
    lower_value = float(lower)
    upper_value = float(upper)
    if (
        not math.isfinite(lower_value)
        or not math.isfinite(upper_value)
        or upper_value - lower_value <= ZERO_WIDTH_TOLERANCE
    ):
        raise ValueError(f"joint {row.get('name', '<unnamed>')} has invalid range")
    return lower_value, upper_value


def expected_state_identity_rows(item: Mapping[str, Any]) -> list[dict[str, Any]]:
    joints_raw = item.get("joint_specs")
    if not isinstance(joints_raw, list) or any(
        not isinstance(row, Mapping) for row in joints_raw
    ):
        raise ValueError("Table 4 joint_specs are invalid")
    joints = [dict(row) for row in joints_raw]
    if canonical_sha256(joints) != item.get("joint_specs_sha256"):
        raise ValueError("Table 4 joint_specs hash mismatch")
    movable = _nonnegative_int(item.get("movable_dof_count"), "movable DoF count")
    range_evaluable = _nonnegative_int(
        item.get("range_evaluable_dof_count"), "range-evaluable DoF count"
    )
    names = [row.get("name") for row in joints]
    if (
        len(joints) != movable
        or sum(row.get("range_evaluable") is True for row in joints) != range_evaluable
        or len(set(names)) != len(names)
        or any(not isinstance(name, str) or not name for name in names)
    ):
        raise ValueError("Table 4 joint specification closure mismatch")
    expected_denominators = {
        "rest_state_expected": 1,
        "single_state_expected": SINGLE_JOINT_SAMPLES * movable,
        "sobol_state_expected": SOBOL_SAMPLES if movable > 0 else 0,
    }
    for field, expected in expected_denominators.items():
        if item.get(field) != expected:
            raise ValueError(f"Table 4 {field} protocol mismatch")

    rest_values = [0.0] * movable
    identities = [{
        "phase": "rest",
        "joint_name": None,
        "sample_index": 0,
        "joint_values_sha256": canonical_sha256(rest_values),
    }]
    for position, joint in enumerate(joints):
        if joint.get("range_evaluable") is not True:
            continue
        lower, upper = _joint_interval(joint)
        for sample_index in range(SINGLE_JOINT_SAMPLES):
            values = list(rest_values)
            values[position] = lower + sample_index * (upper - lower) / (
                SINGLE_JOINT_SAMPLES - 1
            )
            identities.append({
                "phase": "single_joint_sweep",
                "joint_name": str(joint["name"]),
                "sample_index": sample_index,
                "joint_values_sha256": canonical_sha256(values),
            })
    if joints and range_evaluable == movable:
        from scipy.stats import qmc

        intervals = [_joint_interval(joint) for joint in joints]
        unit = qmc.Sobol(
            d=movable,
            scramble=True,
            seed=SOBOL_SEED,
        ).random_base2(m=6)
        if len(unit) != SOBOL_SAMPLES:
            raise ValueError("Sobol generator returned an unexpected state count")
        for sample_index, vector in enumerate(unit):
            values = [
                float(lower + scalar * (upper - lower))
                for scalar, (lower, upper) in zip(vector, intervals)
            ]
            identities.append({
                "phase": "multi_joint_sobol",
                "joint_name": None,
                "sample_index": sample_index,
                "joint_values_sha256": canonical_sha256(values),
            })
    return identities


def _validate_state_observation(state: Mapping[str, Any]) -> None:
    observations: dict[str, tuple[int, int, float]] = {}
    for prefix in ("all_pair", "non_adjacent"):
        contacts = _nonnegative_int(state.get(f"{prefix}_contact_count"), "contact count")
        illegal = _nonnegative_int(
            state.get(f"{prefix}_illegal_penetration_count"),
            "illegal penetration count",
        )
        maximum_raw = state.get(f"{prefix}_max_penetration_m")
        if (
            isinstance(maximum_raw, bool)
            or not isinstance(maximum_raw, (int, float))
            or not math.isfinite(float(maximum_raw))
            or float(maximum_raw) < 0.0
            or illegal > contacts
        ):
            raise ValueError("collision observation counters are invalid")
        maximum = float(maximum_raw)
        if (illegal > 0) != (maximum > PENETRATION_THRESHOLD_M):
            raise ValueError("penetration threshold evidence is inconsistent")
        if maximum > 0.0 and contacts == 0:
            raise ValueError("penetration exists without a contact")
        observations[prefix] = (contacts, illegal, maximum)
    if (
        observations["non_adjacent"][0] > observations["all_pair"][0]
        or observations["non_adjacent"][1] > observations["all_pair"][1]
        or observations["non_adjacent"][2] > observations["all_pair"][2] + 1e-15
    ):
        raise ValueError("non-adjacent observation is not an all-pair subset")
    expected_metric = observations[
        "all_pair" if state.get("phase") == "rest" else "non_adjacent"
    ][2]
    metric = state.get("metric_max_penetration_m")
    if (
        isinstance(metric, bool)
        or not isinstance(metric, (int, float))
        or not math.isclose(float(metric), expected_metric, rel_tol=0.0, abs_tol=1e-15)
    ):
        raise ValueError("state metric penetration policy mismatch")
    readback = state.get("reset_readback_max_abs_error")
    if (
        isinstance(readback, bool)
        or not isinstance(readback, (int, float))
        or not math.isfinite(float(readback))
        or not 0.0 <= float(readback) <= RESET_TOLERANCE
    ):
        raise ValueError("reset readback evidence is invalid")


def _require_identity(
    expected: Mapping[str, Any],
    observed: Mapping[str, Any],
    *,
    label: str,
) -> None:
    for field in TABLE4_IDENTITY_FIELDS:
        if observed.get(field) != expected.get(field):
            raise ValueError(f"{label} {field} mismatch")


def reaggregate_table4_asset(
    item: Mapping[str, Any],
    asset: Mapping[str, Any],
    states: list[dict[str, Any]],
) -> dict[str, Any]:
    """Recompute one frozen Table 4 strict result from its raw state rows."""

    asset_key = str(item["asset_key"])
    _require_identity(item, asset, label=f"Table 4 asset {asset_key}")
    observed_states_sha256 = canonical_sha256(states)
    if observed_states_sha256 != asset.get("state_records_sha256"):
        raise ValueError(f"state records SHA256 mismatch: {asset_key}")
    expected_state_identities = expected_state_identity_rows(item)
    observed_state_identities = [
        {
            "phase": state.get("phase"),
            "joint_name": state.get("joint_name"),
            "sample_index": state.get("sample_index"),
            "joint_values_sha256": state.get("joint_values_sha256"),
        }
        for state in states
    ]
    identity_keys = [
        (row["phase"], row["joint_name"], row["sample_index"])
        for row in observed_state_identities
    ]
    if len(identity_keys) != len(set(identity_keys)):
        raise ValueError(f"duplicate state identity: {asset_key}")
    if observed_state_identities != expected_state_identities[: len(states)]:
        raise ValueError(
            f"state identity does not match frozen sampling sequence: {asset_key}"
        )

    expected_by_phase = {
        "rest": _nonnegative_int(item.get("rest_state_expected"), "rest expected"),
        "single_joint_sweep": _nonnegative_int(
            item.get("single_state_expected"), "single expected"
        ),
        "multi_joint_sobol": _nonnegative_int(
            item.get("sobol_state_expected"), "sobol expected"
        ),
    }
    rows_by_phase: dict[str, list[dict[str, Any]]] = {
        phase: [] for phase in expected_by_phase
    }
    for state in states:
        _require_identity(item, state, label=f"Table 4 state {asset_key}")
        phase = state.get("phase")
        if phase not in rows_by_phase:
            raise ValueError(f"unknown Table 4 state phase for {asset_key}: {phase!r}")
        _validate_state_observation(state)
        rows_by_phase[str(phase)].append(state)

    executed_by_phase = {
        phase: len(phase_rows) for phase, phase_rows in rows_by_phase.items()
    }
    free_by_phase = {
        phase: sum(
            row["non_adjacent_illegal_penetration_count"] == 0
            for row in phase_rows
        )
        for phase, phase_rows in rows_by_phase.items()
    }
    movable = _nonnegative_int(item.get("movable_dof_count"), "movable DoF count")
    range_evaluable = _nonnegative_int(
        item.get("range_evaluable_dof_count"), "range-evaluable DoF count"
    )
    load_success = asset.get("load_success") is True
    rest_all_pair_pass = bool(
        load_success
        and executed_by_phase["rest"] == expected_by_phase["rest"]
        and all(
            _nonnegative_int(
                row.get("all_pair_illegal_penetration_count"),
                f"all-pair illegal penetration count for {asset_key}",
            )
            == 0
            for row in rows_by_phase["rest"]
        )
    )
    rest_pass = bool(
        load_success
        and executed_by_phase["rest"] == expected_by_phase["rest"]
        and free_by_phase["rest"] == expected_by_phase["rest"]
    )
    single_pass = bool(
        load_success
        and executed_by_phase["single_joint_sweep"]
        == expected_by_phase["single_joint_sweep"]
        and free_by_phase["single_joint_sweep"]
        == expected_by_phase["single_joint_sweep"]
    )
    sobol_pass = bool(
        movable > 0
        and range_evaluable == movable
        and executed_by_phase["multi_joint_sobol"]
        == expected_by_phase["multi_joint_sobol"]
        and free_by_phase["multi_joint_sobol"]
        == expected_by_phase["multi_joint_sobol"]
    )
    measurement_complete = bool(
        load_success
        and range_evaluable == movable
        and sum(executed_by_phase.values()) == sum(expected_by_phase.values())
    )
    strict_pass = bool(
        measurement_complete and rest_pass and single_pass and sobol_pass
    )

    single_by_joint: dict[str, list[dict[str, Any]]] = {}
    for state in rows_by_phase["single_joint_sweep"]:
        joint_name = state.get("joint_name")
        if not isinstance(joint_name, str) or not joint_name:
            raise ValueError(f"single-joint state has invalid joint_name: {asset_key}")
        single_by_joint.setdefault(joint_name, []).append(state)
    joint_sweep_passed = sum(
        len(joint_states) == SINGLE_JOINT_SAMPLES
        and all(
            row["non_adjacent_illegal_penetration_count"] == 0
            for row in joint_states
        )
        for joint_states in single_by_joint.values()
    )

    recorded_fields = {
        "rest_state_executed": executed_by_phase["rest"],
        "rest_non_adjacent_free": free_by_phase["rest"],
        "rest_all_pair_cf": rest_all_pair_pass,
        "rest_non_adjacent_cf": rest_pass,
        "single_state_executed": executed_by_phase["single_joint_sweep"],
        "single_non_adjacent_free": free_by_phase["single_joint_sweep"],
        "joint_single_sweep_cf_passed": joint_sweep_passed,
        "single_joint_sweep_cf": single_pass,
        "sobol_state_executed": executed_by_phase["multi_joint_sobol"],
        "sobol_non_adjacent_free": free_by_phase["multi_joint_sobol"],
        "multi_joint_sobol_cf": sobol_pass,
        "measurement_complete": measurement_complete,
        "strict_collision_pass": strict_pass,
    }
    for field, expected in recorded_fields.items():
        if asset.get(field) != expected:
            raise ValueError(f"Table 4 asset {field} mismatch: {asset_key}")
    for phase, field in (
        ("rest", "rest_state_expected"),
        ("single_joint_sweep", "single_state_expected"),
        ("multi_joint_sobol", "sobol_state_expected"),
    ):
        if asset.get(field) != expected_by_phase[phase]:
            raise ValueError(f"Table 4 asset {field} mismatch: {asset_key}")

    return {
        "asset_key": asset_key,
        "dataset_id": item["dataset_id"],
        "order": item["order"],
        "strict_collision_pass": strict_pass,
        "measurement_complete": measurement_complete,
        "state_record_count": len(states),
        "state_records_sha256": observed_states_sha256,
        "table4_asset_record_sha256": canonical_sha256(asset),
    }


def load_table4_evidence(
    cohort_records: list[Mapping[str, Any]],
) -> dict[str, Any]:
    expected_files = (
        (TABLE4_MANIFEST, EXPECTED_TABLE4_MANIFEST_SHA256),
        (TABLE4_ASSET_RECORDS, EXPECTED_TABLE4_ASSET_RECORDS_SHA256),
        (TABLE4_STATE_RECORDS, EXPECTED_TABLE4_STATE_RECORDS_SHA256),
        (TABLE4_VERIFICATION, EXPECTED_TABLE4_VERIFICATION_SHA256),
    )
    for path, expected in expected_files:
        if sha256_file(path) != expected:
            raise RuntimeError(f"frozen Table 4 artifact SHA256 mismatch: {path}")

    manifest = json.loads(TABLE4_MANIFEST.read_text(encoding="utf-8"))
    assets = json.loads(TABLE4_ASSET_RECORDS.read_text(encoding="utf-8"))
    states = load_jsonl(TABLE4_STATE_RECORDS)
    verification = json.loads(TABLE4_VERIFICATION.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or not isinstance(verification, dict):
        raise ValueError("Table 4 manifest and verification roots must be objects")
    if not isinstance(assets, list) or len(assets) != EXPECTED_N_EVAL:
        raise ValueError(f"Table 4 asset records must contain {EXPECTED_N_EVAL} rows")
    items = manifest.get("items")
    if not isinstance(items, list) or len(items) != EXPECTED_N_EVAL:
        raise ValueError(f"Table 4 manifest must contain {EXPECTED_N_EVAL} items")
    content = {key: value for key, value in manifest.items() if key != "manifest_content_sha256"}
    if (
        manifest.get("protocol_id") != EXPECTED_TABLE4_PROTOCOL_ID
        or manifest.get("manifest_content_sha256")
        != EXPECTED_TABLE4_MANIFEST_CONTENT_SHA256
        or canonical_sha256(content) != EXPECTED_TABLE4_MANIFEST_CONTENT_SHA256
    ):
        raise RuntimeError("frozen Table 4 manifest identity mismatch")
    artifact_hashes = verification.get("artifact_sha256")
    expected_receipt_hashes = {
        "frozen_manifest.json": EXPECTED_TABLE4_MANIFEST_SHA256,
        "asset_records.json": EXPECTED_TABLE4_ASSET_RECORDS_SHA256,
        "state_records.jsonl": EXPECTED_TABLE4_STATE_RECORDS_SHA256,
    }
    if (
        verification.get("status") != "PASS"
        or verification.get("sample_size") != EXPECTED_N_EVAL
        or verification.get("evaluated_protocol_id") != EXPECTED_TABLE4_PROTOCOL_ID
        or verification.get("table3_asset_records_sha256")
        != EXPECTED_TABLE3_RECORDS_SHA256
        or verification.get("table3_manifest_sha256")
        != EXPECTED_TABLE3_MANIFEST_SHA256
        or verification.get("table3_manifest_content_sha256")
        != EXPECTED_TABLE3_MANIFEST_CONTENT_SHA256
        or not isinstance(artifact_hashes, dict)
        or any(artifact_hashes.get(name) != digest for name, digest in expected_receipt_hashes.items())
    ):
        raise RuntimeError("Table 4 verification receipt does not bind frozen evidence")

    states_by_dataset_id: dict[str, list[dict[str, Any]]] = {}
    for state in states:
        dataset_id = state.get("dataset_id")
        if not isinstance(dataset_id, str):
            raise ValueError("Table 4 state has invalid dataset_id")
        states_by_dataset_id.setdefault(dataset_id, []).append(state)

    table4_results: list[dict[str, Any]] = []
    known_dataset_ids: set[str] = set()
    for order, (item, asset) in enumerate(zip(items, assets, strict=True)):
        if not isinstance(item, dict) or not isinstance(asset, dict):
            raise ValueError(f"Table 4 item/asset must be objects at order {order}")
        if item.get("order") != order or asset.get("order") != order:
            raise ValueError(f"Table 4 order mismatch at order {order}")
        dataset_id = item.get("dataset_id")
        if not isinstance(dataset_id, str) or dataset_id in known_dataset_ids:
            raise ValueError(f"Table 4 dataset_id is invalid at order {order}")
        known_dataset_ids.add(dataset_id)
        table4_results.append(
            reaggregate_table4_asset(
                item,
                asset,
                states_by_dataset_id.get(dataset_id, []),
            )
        )
    unknown_dataset_ids = set(states_by_dataset_id) - known_dataset_ids
    if unknown_dataset_ids:
        raise ValueError("Table 4 states contain unknown dataset IDs")
    strict_passed = sum(row["strict_collision_pass"] for row in table4_results)
    if strict_passed != EXPECTED_FORMAL_STRICT_PASSED:
        raise RuntimeError("Table 4 strict result no longer matches frozen formal result")

    if not 1 <= len(cohort_records) <= EXPECTED_N_EVAL:
        raise ValueError("requested cohort is empty or exceeds the frozen cohort")
    selected: list[dict[str, Any]] = []
    for order, source in enumerate(cohort_records):
        item = items[order]
        asset = assets[order]
        result = table4_results[order]
        expected_source = {
            "order": source.get("selection_index"),
            "asset_key": source.get("asset_key"),
            "selection_rank": source.get("selection_rank"),
            "selection_hash": source.get("selection_hash"),
            "tier": source.get("tier"),
            "rel_path": source.get("rel_path"),
            "object_release_id": source.get("object_release_id"),
            "model_urdf_sha256": source.get("urdf_sha256"),
            "package_relpath": f"released_outputs/{source.get('rel_path')}",
            "source_record_sha256": canonical_sha256(source["table3_asset_record"]),
            "source_manifest_record_sha256": canonical_sha256(
                source["table3_manifest_record"]
            ),
        }
        for field, expected in expected_source.items():
            if item.get(field) != expected or asset.get(field) != expected:
                raise ValueError(f"Table 3/Table 4 {field} mismatch at order {order}")
        selected.append(result)

    return {
        "records": selected,
        "items": items[: len(cohort_records)],
        "verification_status": verification["status"],
        "manifest_file_sha256": EXPECTED_TABLE4_MANIFEST_SHA256,
        "manifest_content_sha256": EXPECTED_TABLE4_MANIFEST_CONTENT_SHA256,
        "asset_records_file_sha256": EXPECTED_TABLE4_ASSET_RECORDS_SHA256,
        "state_records_file_sha256": EXPECTED_TABLE4_STATE_RECORDS_SHA256,
        "verification_file_sha256": EXPECTED_TABLE4_VERIFICATION_SHA256,
        "formal_strict_passed": strict_passed,
    }


def evaluate_asset(
    source: Mapping[str, Any],
    table4_item: Mapping[str, Any],
    table4_result: Mapping[str, Any],
) -> dict[str, Any]:
    asset_key = str(source["asset_key"])
    package = Path(source["package"])
    urdf_path = Path(source["urdf_path"])
    binding_issues: list[str] = []
    try:
        observed_binding = package_binding(package)
        if observed_binding != table4_item.get("package_binding"):
            binding_issues.append("package_binding_mismatch")
        observed_urdf_sha256 = sha256_file(urdf_path)
        if observed_urdf_sha256 != source.get("urdf_sha256"):
            binding_issues.append("table3_urdf_sha256_mismatch")
        if observed_urdf_sha256 != table4_item.get("model_urdf_sha256"):
            binding_issues.append("table4_urdf_sha256_mismatch")
        if observed_binding.get("content_manifest_sha256") != table4_item.get(
            "package_content_manifest_sha256"
        ):
            binding_issues.append("package_content_manifest_sha256_mismatch")
    except Exception as exc:  # noqa: BLE001
        observed_binding = None
        observed_urdf_sha256 = None
        binding_issues.append(f"binding_error:{type(exc).__name__}:{exc}")

    binding_verified = not binding_issues
    if binding_verified:
        static_record = static_atoms.audit_lam_package(
            package,
            urdf_relative_path="generated.urdf",
            asset_id=asset_key,
            expected_movable_joints=int(source["declared_joint_count_hint"]),
            placeholder_registry=[],
        )
    else:
        static_record = {
            "status": "error",
            "resource_closure": {
                "status": "NOT_EVALUABLE",
                "complete": False,
                "sha256": None,
                "issues": list(binding_issues),
            },
            "s1_evidence": {"status": "NOT_EVALUABLE", "issues": list(binding_issues)},
            "issues": list(binding_issues),
        }

    evidence_raw = static_record.get("s1_evidence")
    evidence = dict(evidence_raw) if isinstance(evidence_raw, Mapping) else {}
    receipt = (
        dict(evidence.get("receipt"))
        if isinstance(evidence.get("receipt"), Mapping)
        else {"receipt_bound_asset": 0, "candidate_count": 0, "issues": ["receipt_audit_missing"]}
    )
    receipt_bound = bool(receipt.get("receipt_bound_asset"))
    evidence["receipt"] = receipt
    evidence["receipt_replay"] = {
        "eligible_receipt_count": int(receipt.get("valid_mechanical_receipt_count", 0)),
        "attempted": 0,
        "passed": False,
        "status": (
            "VALID_RECEIPT_NOT_REPLAYED" if receipt_bound else "NO_VALID_RECEIPT"
        ),
    }
    rebuild = (
        evidence.get("rebuild")
        if isinstance(evidence.get("rebuild"), Mapping)
        else {}
    )
    rebuild_eligible = bool(rebuild.get("eligible_asset"))
    allowance = (
        evidence.get("allowance")
        if isinstance(evidence.get("allowance"), Mapping)
        else {}
    )
    registered_pairs = (
        allowance.get("registered_excluded_pair_count")
        if allowance.get("status") == "COMPLETE"
        else None
    )
    strict_pass = bool(
        binding_verified and table4_result.get("strict_collision_pass") is True
    )
    registered_strict = registered_allowance_outcome(strict_pass, allowance)
    status = (
        "completed"
        if binding_verified and static_record.get("status") == "completed"
        else "audit_failed"
    )
    identity = {
        "selection_index": source.get("selection_index"),
        "asset_key": asset_key,
        "selection_rank": source.get("selection_rank"),
        "selection_hash": source.get("selection_hash"),
        "tier": source.get("tier"),
        "rel_path": source.get("rel_path"),
        "object_release_id": source.get("object_release_id"),
        "category": source.get("category"),
        "package": str(package.resolve()),
        "primary_urdf_relative_path": "generated.urdf",
        "model_urdf_sha256": observed_urdf_sha256,
        "package_content_manifest_sha256": (
            observed_binding.get("content_manifest_sha256")
            if isinstance(observed_binding, Mapping)
            else None
        ),
        "table4_input_identity_sha256": table4_item.get("input_identity_sha256"),
    }
    return {
        **identity,
        "s1_input_identity_sha256": canonical_sha256(identity),
        "status": status,
        "binding": {"verified": binding_verified, "issues": binding_issues},
        "resource_closure": static_record.get("resource_closure"),
        "s1_evidence": evidence,
        "deterministic_rebuild_match": None,
        "rebuild_replay_status": "ELIGIBLE_NOT_RUN" if rebuild_eligible else "N/E",
        "strict_pass_no_method_allowance": strict_pass,
        "registered_allowance_strict_pass": registered_strict,
        "table4_measurement_complete": bool(table4_result.get("measurement_complete")),
        "table4_state_record_count": table4_result.get("state_record_count"),
        "table4_state_records_sha256": table4_result.get("state_records_sha256"),
        "table4_asset_record_sha256": table4_result.get(
            "table4_asset_record_sha256"
        ),
    }


def registered_allowance_outcome(
    strict_pass: bool,
    allowance: Mapping[str, Any],
) -> bool | None:
    registered_pairs = allowance.get("registered_excluded_pair_count")
    no_named_candidate = allowance.get("candidate_file_count") == 0
    if registered_pairs == 0 or (registered_pairs is None and no_named_candidate):
        return strict_pass
    return None


def _ratio(passed: int, denominator: int) -> dict[str, int | float | None]:
    rate = passed / denominator if denominator else None
    return {
        "passed": passed,
        "denominator": denominator,
        "rate": rate,
        "percentage": None if rate is None else rate * 100.0,
    }


def aggregate_s1(
    records: list[Mapping[str, Any]],
    *,
    intended_assets: int,
) -> dict[str, Any]:
    if intended_assets < 0 or len(records) != intended_assets:
        raise ValueError("S1 records must exactly cover the intended asset denominator")

    receipt_bound = 0
    receipt_replayed = 0
    rebuild_eligible = 0
    rebuild_matched = 0
    rebuild_complete = True
    registered_pairs = 0
    eligible_pairs = 0
    allowance_complete = True
    allowance_measured_assets = 0
    strict_passed = 0
    registered_passed = 0
    registered_outcomes_complete = True

    for record in records:
        binding = record.get("binding")
        result_eligible = bool(
            record.get("status") == "completed"
            and isinstance(binding, Mapping)
            and binding.get("verified") is True
        )
        evidence = (
            record.get("s1_evidence")
            if isinstance(record.get("s1_evidence"), Mapping)
            else {}
        )
        receipt = (
            evidence.get("receipt")
            if isinstance(evidence.get("receipt"), Mapping)
            else {}
        )
        replay = (
            evidence.get("receipt_replay")
            if isinstance(evidence.get("receipt_replay"), Mapping)
            else {}
        )
        rebuild = (
            evidence.get("rebuild")
            if isinstance(evidence.get("rebuild"), Mapping)
            else {}
        )
        allowance = (
            evidence.get("allowance")
            if isinstance(evidence.get("allowance"), Mapping)
            else {}
        )
        receipt_bound += int(result_eligible and bool(receipt.get("receipt_bound_asset")))
        receipt_replayed += int(result_eligible and bool(replay.get("passed")))
        eligible = int(result_eligible and bool(rebuild.get("eligible_asset")))
        rebuild_eligible += eligible
        rebuild_matched += int(bool(record.get("deterministic_rebuild_match"))) if eligible else 0
        if eligible and record.get("rebuild_replay_status") != "COMPLETE":
            rebuild_complete = False

        if not result_eligible or allowance.get("status") != "COMPLETE":
            allowance_complete = False
        else:
            allowance_measured_assets += 1
            registered_pairs += _nonnegative_int(
                allowance.get("registered_excluded_pair_count"),
                "registered allowance pair count",
            )
            eligible_pairs += _nonnegative_int(
                allowance.get("eligible_nonadjacent_pair_count"),
                "eligible non-adjacent pair count",
            )
        strict_passed += int(
            result_eligible and bool(record.get("strict_pass_no_method_allowance"))
        )
        registered_outcome = record.get("registered_allowance_strict_pass")
        if result_eligible and isinstance(registered_outcome, bool):
            registered_passed += int(registered_outcome)
        else:
            registered_outcomes_complete = False

    rebuild_metric = {
        "status": (
            "N/E"
            if rebuild_eligible == 0
            else ("COMPLETE" if rebuild_complete else "NOT_EVALUABLE")
        ),
        "passed": None if rebuild_eligible == 0 else rebuild_matched,
        "denominator": rebuild_eligible,
        "rate": None if rebuild_eligible == 0 else rebuild_matched / rebuild_eligible,
        "percentage": (
            None
            if rebuild_eligible == 0
            else 100.0 * rebuild_matched / rebuild_eligible
        ),
        "eligible_assets": rebuild_eligible,
        "asset_denominator": intended_assets,
    }
    allowance_rate = (
        registered_pairs / eligible_pairs
        if allowance_complete and eligible_pairs > 0
        else None
    )
    allowance_metric = {
        "status": (
            "PARTIAL"
            if not allowance_complete
            else ("N/E" if eligible_pairs == 0 else "COMPLETE")
        ),
        "registered_pairs": registered_pairs,
        "eligible_pairs": eligible_pairs,
        "rate": allowance_rate,
        "percentage": None if allowance_rate is None else allowance_rate * 100.0,
        "measured_assets": allowance_measured_assets,
        "intended_assets": intended_assets,
    }
    strict_metric = _ratio(strict_passed, intended_assets)
    if registered_pairs == 0 and allowance_complete:
        gain_metric = {
            "status": "COMPLETE",
            "value": 0.0,
            "registered_passed": strict_passed,
            "no_allowance_passed": strict_passed,
            "denominator": intended_assets,
        }
    elif registered_outcomes_complete:
        gain_metric = {
            "status": "COMPLETE",
            "value": 100.0 * (registered_passed - strict_passed) / intended_assets,
            "registered_passed": registered_passed,
            "no_allowance_passed": strict_passed,
            "denominator": intended_assets,
        }
    else:
        gain_metric = {
            "status": "NOT_EVALUABLE",
            "value": None,
            "registered_passed": None,
            "no_allowance_passed": strict_passed,
            "denominator": intended_assets,
            "reason": "registered allowance exists but no frozen pair-specific replay is available",
        }
    return {
        "receipt_bound_assets": _ratio(receipt_bound, intended_assets),
        "receipt_replay_pass": _ratio(receipt_replayed, intended_assets),
        "deterministic_rebuild_match": rebuild_metric,
        "allowance_density": allowance_metric,
        "strict_pass_no_method_allowance": strict_metric,
        "registered_allowance_gain_pp": gain_metric,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("formal", "smoke"), required=True)
    parser.add_argument("--n", type=int, default=None)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--skip-verify", action="store_true")
    return parser.parse_args(argv)


def requested_n(args: argparse.Namespace) -> int:
    return EXPECTED_N_EVAL if args.mode == "formal" and args.n is None else int(args.n)


def validate_args(args: argparse.Namespace) -> None:
    if args.mode == "formal":
        if args.n not in (None, EXPECTED_N_EVAL):
            raise ValueError(f"formal mode requires n={EXPECTED_N_EVAL}")
        if args.workers != DEFAULT_WORKERS:
            raise ValueError(f"formal mode requires workers={DEFAULT_WORKERS}")
        if bool(getattr(args, "skip_verify", False)):
            raise ValueError("formal mode requires independent verification")
    elif args.n is None:
        raise ValueError("smoke mode requires --n")
    if args.n is not None and not 1 <= args.n <= EXPECTED_N_EVAL:
        raise ValueError(f"n must be within 1..{EXPECTED_N_EVAL}")
    if args.workers <= 0:
        raise ValueError("workers must be positive")


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def timestamp_tag() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def atomic_write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_write_json(path: Path, value: Any) -> None:
    atomic_write_text(
        path,
        json.dumps(
            value,
            indent=2,
            sort_keys=True,
            ensure_ascii=True,
            allow_nan=False,
        )
        + "\n",
    )


def render_summary(summary: Mapping[str, Any]) -> str:
    metrics = summary["metrics"]

    def fraction(metric: Mapping[str, Any]) -> str:
        percentage = metric.get("percentage")
        if percentage is None:
            return f"{metric.get('passed')} / {metric.get('denominator')} (N/E)"
        return (
            f"{metric.get('passed')} / {metric.get('denominator')} "
            f"({float(percentage):.2f}%)"
        )

    rebuild = metrics["deterministic_rebuild_match"]
    rebuild_text = (
        f"N/E ({rebuild['eligible_assets']} / {rebuild['asset_denominator']} eligible)"
        if rebuild["status"] == "N/E"
        else fraction(rebuild)
    )
    allowance = metrics["allowance_density"]
    if allowance["status"] == "PARTIAL":
        allowance_text = (
            f"{allowance['registered_pairs']} / {allowance['eligible_pairs']} "
            f"(PARTIAL; {allowance['measured_assets']} / "
            f"{allowance['intended_assets']} assets)"
        )
    elif allowance["percentage"] is None:
        allowance_text = (
            f"{allowance['registered_pairs']} / {allowance['eligible_pairs']} (N/E)"
        )
    else:
        allowance_text = (
            f"{allowance['registered_pairs']} / {allowance['eligible_pairs']} "
            f"({float(allowance['percentage']):.2f}%)"
        )
    gain = metrics["registered_allowance_gain_pp"]
    gain_text = "N/E" if gain["value"] is None else f"{float(gain['value']):.2f} pp"
    return "\n".join(
        [
            f"# Supplementary Table S1 - {summary['dataset']}",
            "",
            f"- Protocol: `{summary['protocol_id']}`",
            f"- Classification: `{summary['classification']}`",
            f"- N_eval: {summary['n_eval']}",
            f"- Status counts: {summary['status_counts']}",
            "",
            "| Metric | Result |",
            "|---|---:|",
            f"| Receipt-bound Assets | {fraction(metrics['receipt_bound_assets'])} |",
            f"| Receipt Replay Pass | {fraction(metrics['receipt_replay_pass'])} |",
            f"| Deterministic Rebuild Match | {rebuild_text} |",
            f"| Allowance Density | {allowance_text} |",
            "| Strict Pass (No Method-specific Allowance) | "
            f"{fraction(metrics['strict_pass_no_method_allowance'])} |",
            f"| Registered-allowance Gain | {gain_text} |",
            "",
        ]
    )


def _failed_asset_record(
    source: Mapping[str, Any],
    issue: str,
) -> dict[str, Any]:
    return {
        "selection_index": source.get("selection_index"),
        "asset_key": source.get("asset_key"),
        "selection_rank": source.get("selection_rank"),
        "selection_hash": source.get("selection_hash"),
        "tier": source.get("tier"),
        "rel_path": source.get("rel_path"),
        "object_release_id": source.get("object_release_id"),
        "category": source.get("category"),
        "package": str(source.get("package")),
        "status": "worker_error",
        "binding": {"verified": False, "issues": [issue]},
        "resource_closure": {
            "status": "NOT_EVALUABLE",
            "complete": False,
            "sha256": None,
            "issues": [issue],
        },
        "s1_evidence": {
            "receipt": {"receipt_bound_asset": 0, "candidate_count": 0},
            "receipt_replay": {"passed": False, "status": "WORKER_ERROR"},
            "rebuild": {"eligible_asset": 0, "status": "N/E"},
            "allowance": {
                "status": "NOT_EVALUABLE",
                "registered_excluded_pair_count": None,
                "eligible_nonadjacent_pair_count": None,
            },
        },
        "deterministic_rebuild_match": None,
        "rebuild_replay_status": "N/E",
        "strict_pass_no_method_allowance": False,
        "registered_allowance_strict_pass": None,
        "issues": [issue],
    }


def evaluate_asset_payload(
    payload: tuple[dict[str, Any], dict[str, Any], dict[str, Any]],
) -> dict[str, Any]:
    source, table4_item, table4_result = payload
    try:
        return evaluate_asset(source, table4_item, table4_result)
    except Exception as exc:  # noqa: BLE001
        return _failed_asset_record(
            source,
            f"worker_error:{type(exc).__name__}:{exc}",
        )


def _parallel_evaluate(
    payloads: list[tuple[dict[str, Any], dict[str, Any], dict[str, Any]]],
    *,
    workers: int,
) -> list[dict[str, Any]]:
    if workers == 1:
        return [evaluate_asset_payload(payload) for payload in payloads]
    context = multiprocessing.get_context("fork")
    with context.Pool(processes=workers) as pool:
        return pool.map(evaluate_asset_payload, payloads)


def _artifact_binding(path: Path) -> dict[str, Any]:
    return {"bytes": path.stat().st_size, "sha256": sha256_file(path)}


def _default_output(mode: str, n: int) -> Path:
    prefix = (
        "s1_lam_table3cohort_n800_seed20260813"
        if mode == "formal"
        else f"s1_lam_smoke_n{n}"
    )
    return DEFAULT_OUTPUT_PARENT / f"{prefix}_{timestamp_tag()}"


def run_evaluation(
    *,
    mode: str,
    n: int,
    workers: int,
    output: Path,
    run_verifier: bool,
) -> dict[str, Any]:
    args = argparse.Namespace(
        mode=mode,
        n=n,
        workers=workers,
        output=output,
        skip_verify=not run_verifier,
    )
    validate_args(args)
    output = Path(os.path.abspath(str(output)))
    if output.is_symlink():
        raise ValueError(f"output directory is a symlink: {output}")
    if output.exists():
        if not output.is_dir() or any(output.iterdir()):
            raise ValueError(f"output directory must be new or empty: {output}")
    else:
        output.mkdir(parents=True)

    started_at = utc_now()
    start_monotonic = time.monotonic()
    cohort = load_frozen_cohort(limit=n if mode == "smoke" else None)
    protocol_snapshot = output / "protocol_snapshot.md"
    source_protocol_sha256 = sha256_file(PROTOCOL_DOCUMENT)
    if mode == "formal" and source_protocol_sha256 != EXPECTED_PROTOCOL_SNAPSHOT_SHA256:
        raise RuntimeError("protocol document changed after the S1 snapshot was frozen")
    atomic_write_text(protocol_snapshot, PROTOCOL_DOCUMENT.read_text(encoding="utf-8"))
    if sha256_file(protocol_snapshot) != source_protocol_sha256:
        raise RuntimeError("protocol snapshot copy hash mismatch")
    verifier_sha256 = sha256_file(VERIFIER_PATH) if VERIFIER_PATH.is_file() else None
    frozen_config = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "classification": mode.upper(),
        "created_at": started_at,
        "n_eval": n,
        "workers": workers,
        "cohort": {
            "source_manifest": str(TABLE3_MANIFEST),
            "source_manifest_file_sha256": EXPECTED_TABLE3_MANIFEST_SHA256,
            "source_manifest_content_sha256": EXPECTED_TABLE3_MANIFEST_CONTENT_SHA256,
            "source_asset_records": str(TABLE3_RECORDS),
            "source_asset_records_file_sha256": EXPECTED_TABLE3_RECORDS_SHA256,
            "ordered_asset_keys_sha256": EXPECTED_ORDERED_ASSET_KEYS_SHA256,
            "selection": "first N records in frozen selection_rank 1..800 order",
        },
        "table4": {
            "protocol_id": EXPECTED_TABLE4_PROTOCOL_ID,
            "manifest_sha256": EXPECTED_TABLE4_MANIFEST_SHA256,
            "manifest_content_sha256": EXPECTED_TABLE4_MANIFEST_CONTENT_SHA256,
            "asset_records_sha256": EXPECTED_TABLE4_ASSET_RECORDS_SHA256,
            "state_records_sha256": EXPECTED_TABLE4_STATE_RECORDS_SHA256,
            "verification_sha256": EXPECTED_TABLE4_VERIFICATION_SHA256,
        },
        "pair_policy": {
            "eligible_pairs": "distinct source-URDF links with collision geometry",
            "shared_topology_exclusion": "exclude_direct_parent_child",
            "surface_contact_allowed": True,
            "illegal_penetration_threshold_m": 1e-6,
            "registered_method_specific_allowance_registry": [],
        },
        "privacy": {
            "generation_config": "hash metadata only; contents are not parsed or emitted",
            "released_code_execution": "disabled",
            "network_access": "disabled",
        },
        "code": {
            "runner": str(SCRIPT),
            "runner_sha256": sha256_file(SCRIPT),
            "verifier": str(VERIFIER_PATH),
            "verifier_sha256": verifier_sha256,
            "static_atoms": str(Path(static_atoms.__file__).resolve()),
            "static_atoms_sha256": sha256_file(Path(static_atoms.__file__).resolve()),
        },
        "protocol_snapshot_sha256": sha256_file(protocol_snapshot),
    }
    atomic_write_json(output / "frozen_config.json", frozen_config)
    environment = {
        "python": sys.version,
        "python_executable": sys.executable,
        "platform": platform.platform(),
        "workers": workers,
        "thread_environment": {
            name: os.environ.get(name)
            for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")
        },
    }
    atomic_write_json(output / "environment.json", environment)

    table4 = load_table4_evidence(cohort["records"])
    payloads = [
        (dict(source), dict(item), dict(table4_result))
        for source, item, table4_result in zip(
            cohort["records"], table4["items"], table4["records"], strict=True
        )
    ]
    records = _parallel_evaluate(payloads, workers=workers)
    if [row.get("selection_index") for row in records] != list(range(n)):
        raise RuntimeError("worker output order no longer matches frozen cohort")

    metrics = aggregate_s1(records, intended_assets=n)
    status_counts: dict[str, int] = {}
    for record in records:
        status = str(record.get("status", "missing"))
        status_counts[status] = status_counts.get(status, 0) + 1
    candidate_counts = {
        "receipt": sum(
            int(record.get("s1_evidence", {}).get("receipt", {}).get("candidate_count", 0))
            for record in records
        ),
        "rebuild_recipe": sum(
            int(
                record.get("s1_evidence", {})
                .get("rebuild", {})
                .get("candidate_recipe_count", 0)
            )
            for record in records
        ),
        "allowance": sum(
            int(
                record.get("s1_evidence", {})
                .get("allowance", {})
                .get("candidate_file_count", 0)
            )
            for record in records
        ),
    }
    if mode == "formal":
        formal_failures: list[str] = []
        if status_counts != {"completed": EXPECTED_N_EVAL}:
            formal_failures.append(f"status_counts={status_counts}")
        if candidate_counts != {
            "receipt": 0,
            "rebuild_recipe": 0,
            "allowance": 0,
        }:
            formal_failures.append(f"unexpected_candidates={candidate_counts}")
        if metrics["receipt_bound_assets"] != _ratio(0, EXPECTED_N_EVAL):
            formal_failures.append("receipt_bound_metric_mismatch")
        if metrics["receipt_replay_pass"] != _ratio(0, EXPECTED_N_EVAL):
            formal_failures.append("receipt_replay_metric_mismatch")
        rebuild_metric = metrics["deterministic_rebuild_match"]
        if (
            rebuild_metric.get("status") != "N/E"
            or rebuild_metric.get("eligible_assets") != 0
            or rebuild_metric.get("passed") is not None
        ):
            formal_failures.append("deterministic_rebuild_metric_mismatch")
        allowance_metric = metrics["allowance_density"]
        if (
            allowance_metric.get("status") != "PARTIAL"
            or allowance_metric.get("registered_pairs") != 0
            or allowance_metric.get("eligible_pairs")
            != EXPECTED_FORMAL_ALLOWANCE_ELIGIBLE_PAIRS
            or allowance_metric.get("measured_assets")
            != EXPECTED_FORMAL_ALLOWANCE_MEASURED_ASSETS
            or allowance_metric.get("intended_assets") != EXPECTED_N_EVAL
            or allowance_metric.get("rate") is not None
            or allowance_metric.get("percentage") is not None
        ):
            formal_failures.append("allowance_density_metric_mismatch")
        if metrics["strict_pass_no_method_allowance"]["passed"] != EXPECTED_FORMAL_STRICT_PASSED:
            formal_failures.append("formal_strict_pass_mismatch")
        gain_metric = metrics["registered_allowance_gain_pp"]
        if (
            gain_metric.get("status") != "COMPLETE"
            or gain_metric.get("value") != 0.0
            or gain_metric.get("registered_passed")
            != EXPECTED_FORMAL_STRICT_PASSED
            or gain_metric.get("no_allowance_passed")
            != EXPECTED_FORMAL_STRICT_PASSED
        ):
            formal_failures.append("registered_allowance_gain_metric_mismatch")
        if formal_failures:
            raise RuntimeError("formal S1 invariants failed: " + "; ".join(formal_failures))

    completed_at = utc_now()
    summary = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "classification": mode.upper(),
        "status": "completed",
        "started_at": started_at,
        "completed_at": completed_at,
        "wall_seconds": time.monotonic() - start_monotonic,
        "n_eval": n,
        "full_frozen_cohort_size": cohort["full_size"],
        "status_counts": dict(sorted(status_counts.items())),
        "evidence_candidate_counts": candidate_counts,
        "metrics": metrics,
    }
    records_text = "".join(
        json.dumps(
            record,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        )
        + "\n"
        for record in records
    )
    atomic_write_text(output / "asset_records.jsonl", records_text)
    atomic_write_json(output / "summary.json", summary)
    atomic_write_text(output / "summary.md", render_summary(summary))

    artifact_names = (
        "asset_records.jsonl",
        "environment.json",
        "frozen_config.json",
        "protocol_snapshot.md",
        "summary.json",
        "summary.md",
    )
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "protocol_id": PROTOCOL_ID,
        "dataset": DATASET,
        "classification": mode.upper(),
        "status": "completed",
        "created_at": completed_at,
        "n_eval": n,
        "artifacts": {
            name: _artifact_binding(output / name) for name in artifact_names
        },
    }
    manifest["manifest_content_sha256"] = canonical_sha256(manifest)
    atomic_write_json(output / "manifest.json", manifest)

    if run_verifier:
        if not VERIFIER_PATH.is_file():
            raise RuntimeError(f"independent verifier is unavailable: {VERIFIER_PATH}")
        command = [
            sys.executable,
            str(VERIFIER_PATH),
            "--run",
            str(output),
        ]
        if mode == "formal":
            command.append("--formal")
        completed = subprocess.run(
            command,
            cwd=REPO,
            check=False,
            text=True,
            capture_output=True,
        )
        if completed.returncode != 0:
            raise RuntimeError(
                "independent S1 verifier failed:\n"
                + completed.stdout
                + completed.stderr
            )
    return {"status": "completed", "output": output, "summary": summary}


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        validate_args(args)
        n = requested_n(args)
        output = args.output or _default_output(args.mode, n)
        result = run_evaluation(
            mode=args.mode,
            n=n,
            workers=args.workers,
            output=output,
            run_verifier=not args.skip_verify,
        )
    except (OSError, RuntimeError, TypeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(result["output"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
