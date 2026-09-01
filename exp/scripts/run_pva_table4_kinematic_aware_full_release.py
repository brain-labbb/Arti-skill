#!/usr/bin/env python3
"""Run the PV-A Table 4 v3 kinematic- and collision-aware protocol.

This is a separate adapter from the historical mimic-aware v2 runner.  It
combines reviewed, package-bound affine constraints with the numerical
zero-margin collision oracle.  Historical v2 jobs and receipts remain
unchanged.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
from typing import Any, Mapping
import xml.etree.ElementTree as ET
import zlib


SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import run_pva_table4_mimic_aware_full_release as v2


RUN_SCHEMA_VERSION = "pva_table4_kinematic_aware_full_release_run_v3"
RESULT_DB_SCHEMA_VERSION = "pva_table4_kinematic_aware_results_db_v1"
RECEIPT_SCHEMA_VERSION = "pva_table4_kinematic_aware_full_release_receipt_v1"
SUMMARY_SCHEMA_VERSION = "pva_table4_kinematic_aware_summary_v3"
CHECKPOINT_SCHEMA_VERSION = "pva_table4_kinematic_aware_checkpoint_v1"
ARTIFACT_SCHEMA_VERSION = "pva_table4_kinematic_aware_artifacts_v1"
PROGRESS_SCHEMA_VERSION = "pva_table4_kinematic_aware_progress_v1"
PROTOCOL_ID = "urdf_sim_ready_table4_pva_full_release_v3"
DEFAULT_OUTPUT = (
    SCRIPT.parents[1] / "runtime" / "pva_table4_kinematic_aware_full_release_v3"
)
KINEMATIC_CONSTRAINT_REGISTRY = (
    SCRIPT.parents[1] / "reference" / "pva_table4_kinematic_constraints_v1.json"
)
KINEMATIC_CONSTRAINT_REGISTRY_SCHEMA = (
    "pva_table4_kinematic_constraint_registry_v1"
)
KINEMATIC_CONSTRAINT_BINDING_SCHEMA = "pva_table4_kinematic_constraint_binding_v1"

table4 = v2.table4
pva_run = v2.pva_run
pva_roster = v2.pva_roster
SAMPLING_PROTOCOL = v2.SAMPLING_PROTOCOL
DEFAULT_SOURCE_EVALUATION = v2.DEFAULT_SOURCE_EVALUATION

_V2_MANIFEST = v2._manifest
_V2_VALIDATE_RESULT = v2._validate_result
_V2_FINALIZE = v2._finalize
_V2_ARTIFACT_MANIFEST = v2._artifact_manifest
_V2_WRITE_PROGRESS = v2._write_progress
_V2_RUN = v2.run_pva_table4_v2
_V3_HOOK_LOCK = threading.RLock()
_ACTIVE_EXECUTION_SOURCE_HASHES: dict[str, str] | None = None


@lru_cache(maxsize=1)
def _zero_margin_oracle_sha256() -> str:
    return v2._sha256_file(table4.ZERO_MARGIN_ORACLE_SCRIPT)


def _verify_zero_margin_oracle_sha256(expected: str) -> None:
    observed = v2._sha256_file(table4.ZERO_MARGIN_ORACLE_SCRIPT)
    if observed != expected:
        raise ValueError(
            "zero-margin oracle changed during the PV-A v3 run: "
            f"{observed} != {expected}"
        )


@lru_cache(maxsize=1)
def _kinematic_constraint_registry_snapshot() -> tuple[dict[str, Any], str]:
    path = KINEMATIC_CONSTRAINT_REGISTRY.resolve(strict=True)
    payload = path.read_bytes()
    try:
        registry = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("PV-A kinematic constraint registry is invalid JSON") from exc
    if not isinstance(registry, dict):
        raise ValueError("PV-A kinematic constraint registry is not an object")
    if registry.get("schema_version") != KINEMATIC_CONSTRAINT_REGISTRY_SCHEMA:
        raise ValueError("unsupported PV-A kinematic constraint registry schema")
    entries = registry.get("entries")
    if not isinstance(entries, list) or any(
        not isinstance(entry, Mapping) for entry in entries
    ):
        raise ValueError("PV-A kinematic constraint registry has invalid entries")
    identities = [str(entry.get("constraint_id") or "") for entry in entries]
    if any(not value for value in identities) or len(identities) != len(
        set(identities)
    ):
        raise ValueError("PV-A kinematic constraint IDs must be non-empty and unique")
    return registry, hashlib.sha256(payload).hexdigest()


def _kinematic_constraint_registry() -> dict[str, Any]:
    return _kinematic_constraint_registry_snapshot()[0]


def _kinematic_constraint_registry_sha256() -> str:
    return _kinematic_constraint_registry_snapshot()[1]


def _verify_kinematic_constraint_registry_sha256(expected: str) -> None:
    observed = v2._sha256_file(KINEMATIC_CONSTRAINT_REGISTRY)
    if observed != expected:
        raise ValueError(
            "kinematic constraint registry changed during the PV-A v3 run: "
            f"{observed} != {expected}"
        )


def _finite_joint_limit(row: Mapping[str, Any], field: str) -> float:
    value = row.get(field)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"joint {row.get('name')} has no finite {field} limit")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"joint {row.get('name')} has no finite {field} limit")
    return result


def _joint_axes(urdf: Path) -> dict[str, tuple[float, float, float]]:
    root = ET.parse(urdf).getroot()
    axes: dict[str, tuple[float, float, float]] = {}
    for joint in root.findall("joint"):
        if str(joint.get("type") or "") == "fixed":
            continue
        name = str(joint.get("name") or "")
        axis = joint.find("axis")
        raw = str(axis.get("xyz") or "") if axis is not None else ""
        try:
            values = tuple(float(value) for value in raw.split())
        except ValueError as exc:
            raise ValueError(f"joint {name!r} has a non-numeric axis") from exc
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise ValueError(f"joint {name!r} has no explicit finite 3-vector axis")
        axes[name] = values
    return axes


def _axis_matches(
    actual: tuple[float, float, float], expected: Any, tolerance: float
) -> bool:
    if not isinstance(expected, list) or len(expected) != 3:
        raise ValueError("kinematic registry axis must be a three-element list")
    try:
        expected_values = tuple(float(value) for value in expected)
    except (TypeError, ValueError) as exc:
        raise ValueError("kinematic registry axis is non-numeric") from exc
    return all(
        math.isfinite(value) and abs(observed - value) <= tolerance
        for observed, value in zip(actual, expected_values, strict=True)
    )


def _kinematic_constraint_binding(
    row: Mapping[str, Any],
    *,
    package: Path,
    urdf: Path,
) -> dict[str, Any] | None:
    """Resolve a reviewed constraint and bind it to one immutable package."""

    category = str(row.get("raw_category", row.get("category", "")))
    registry = _kinematic_constraint_registry()
    candidates = [
        entry
        for entry in registry["entries"]
        if str(entry.get("category") or "") == category
    ]
    if not candidates:
        return None
    if len(candidates) != 1:
        raise ValueError(f"ambiguous kinematic registry entries for {category!r}")
    entry = candidates[0]

    expected_urdf_sha256 = str(row.get("primary_urdf_sha256") or "")
    if not expected_urdf_sha256 or v2._sha256_file(urdf) != expected_urdf_sha256:
        raise ValueError("kinematic constraint URDF hash binding mismatch")
    physics_path = package / "physics.json"
    if not physics_path.is_file() or physics_path.is_symlink():
        raise ValueError("kinematic constraint requires a regular physics.json")
    physics = v2._load_json(physics_path)
    if physics.get("model_urdf_sha256") != expected_urdf_sha256:
        raise ValueError("physics.json does not bind the constrained URDF")
    template_sha256 = str(entry.get("template_sha256") or "")
    if physics.get("template_sha256") != template_sha256:
        raise ValueError("physics.json template hash is absent from the registry")
    if str(physics.get("slug") or "") != category:
        raise ValueError("physics.json slug does not match constrained category")

    raw_overrides = row.get("overrides_json")
    if not isinstance(raw_overrides, str):
        raise ValueError("constrained roster row has no overrides_json")
    try:
        overrides = json.loads(raw_overrides)
    except json.JSONDecodeError as exc:
        raise ValueError("constrained roster overrides_json is invalid") from exc
    if not isinstance(overrides, Mapping):
        raise ValueError("constrained roster overrides_json is not an object")
    pitch_spec = entry.get("pitch_radius")
    if not isinstance(pitch_spec, Mapping):
        raise ValueError("kinematic registry pitch-radius specification is invalid")
    teeth_field = str(pitch_spec.get("pinion_teeth_roster_field") or "")
    teeth = overrides.get(teeth_field)
    if isinstance(teeth, bool) or not isinstance(teeth, int) or teeth <= 0:
        raise ValueError("constrained roster has invalid pinion tooth count")
    tooth_pitch = pitch_spec.get("tooth_pitch_m")
    if isinstance(tooth_pitch, bool) or not isinstance(tooth_pitch, (int, float)):
        raise ValueError("kinematic registry tooth pitch is invalid")
    pitch_radius = float(tooth_pitch) * teeth / (2.0 * math.pi)
    if not math.isfinite(pitch_radius) or pitch_radius <= 0.0:
        raise ValueError("derived pinion pitch radius is invalid")

    joints = v2._collision_core().parse_urdf_joints(urdf)
    by_name = {str(joint["name"]): joint for joint in joints}
    axes = _joint_axes(urdf)
    driver_spec = entry.get("driver_joint")
    if not isinstance(driver_spec, Mapping):
        raise ValueError("kinematic registry driver specification is invalid")
    driver_name = str(driver_spec.get("name") or "")
    driver = by_name.get(driver_name)
    if driver is None or driver.get("type") != driver_spec.get("type"):
        raise ValueError("constrained rack driver joint identity/type mismatch")
    axis_tolerance = float(entry.get("axis_abs_tolerance", 0.0))
    if not math.isfinite(axis_tolerance) or axis_tolerance < 0.0:
        raise ValueError("kinematic registry axis tolerance is invalid")
    if driver_name not in axes or not _axis_matches(
        axes[driver_name], driver_spec.get("axis"), axis_tolerance
    ):
        raise ValueError("constrained rack driver axis mismatch")

    drive_skeleton = overrides.get("drive_skeleton")
    follower_map = entry.get("follower_joint_sets_by_drive_skeleton")
    if not isinstance(drive_skeleton, str) or not isinstance(follower_map, Mapping):
        raise ValueError("constrained rack drive skeleton is not registered")
    registered_followers = follower_map.get(drive_skeleton)
    if not isinstance(registered_followers, list) or any(
        not isinstance(name, str) or not name for name in registered_followers
    ):
        raise ValueError("constrained rack follower set is not registered")
    follower_names = [str(name) for name in registered_followers]
    if set(by_name) != {driver_name, *follower_names}:
        raise ValueError("constrained rack joint set disagrees with drive skeleton")
    follower_type = str(entry.get("follower_joint_type") or "")
    follower_axis = entry.get("follower_joint_axis")
    for name in follower_names:
        if by_name[name].get("type") != follower_type:
            raise ValueError("constrained pinion follower joint type mismatch")
        if name not in axes or not _axis_matches(
            axes[name], follower_axis, axis_tolerance
        ):
            raise ValueError("constrained pinion follower axis mismatch")
    if driver.get("mimic") is not None or any(
        by_name[name].get("mimic") is not None for name in follower_names
    ):
        raise ValueError("external rack constraint cannot replace native mimic")

    tolerance = float(entry.get("limit_relation_abs_tolerance", 0.0))
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("kinematic registry limit tolerance is invalid")
    driver_lower = _finite_joint_limit(driver, "lower")
    driver_upper = _finite_joint_limit(driver, "upper")
    if abs(driver_lower + driver_upper) > tolerance:
        raise ValueError("registered rack driver limits are not symmetric")
    follower_limit = entry.get("follower_limit")
    if not isinstance(follower_limit, Mapping):
        raise ValueError("kinematic registry follower-limit specification is invalid")
    minimum_abs = follower_limit.get("minimum_abs_rad")
    maximum_abs = follower_limit.get("maximum_abs_rad")
    if (
        isinstance(minimum_abs, bool)
        or not isinstance(minimum_abs, (int, float))
        or isinstance(maximum_abs, bool)
        or not isinstance(maximum_abs, (int, float))
    ):
        raise ValueError("kinematic registry follower-limit bounds are invalid")
    minimum_abs = float(minimum_abs)
    maximum_abs = float(maximum_abs)
    if (
        not math.isfinite(minimum_abs)
        or not math.isfinite(maximum_abs)
        or minimum_abs < 0.0
        or maximum_abs < minimum_abs
    ):
        raise ValueError("kinematic registry follower-limit bounds are invalid")
    driver_abs = max(abs(driver_lower), abs(driver_upper))
    expected_follower_abs = max(
        minimum_abs,
        min(driver_abs / pitch_radius, maximum_abs),
    )
    for name in follower_names:
        follower = by_name[name]
        if (
            abs(_finite_joint_limit(follower, "lower") + expected_follower_abs)
            > tolerance
            or abs(
                _finite_joint_limit(follower, "upper")
                - expected_follower_abs
            )
            > tolerance
        ):
            raise ValueError(
                f"registered rack/pinion ratio disagrees with limits for {name!r}"
            )

    offset = float(entry.get("phase_offset_rad", 0.0))
    if not math.isfinite(offset):
        raise ValueError("registered rack/pinion phase offset is non-finite")
    external = [
        {
            "constraint_id": f"{entry['constraint_id']}:{name}",
            "driver_joint": driver_name,
            "follower_joint": name,
            "multiplier": 1.0 / pitch_radius,
            "offset": offset,
        }
        for name in follower_names
    ]
    binding: dict[str, Any] = {
        "schema_version": KINEMATIC_CONSTRAINT_BINDING_SCHEMA,
        "registry_sha256": _kinematic_constraint_registry_sha256(),
        "constraint_id": entry["constraint_id"],
        "asset_id": str(row.get("asset_id") or ""),
        "urdf_sha256": expected_urdf_sha256,
        "physics_json_sha256": v2._sha256_file(physics_path),
        "template_sha256": template_sha256,
        "roster_overrides_sha256": v2._canonical_sha256(dict(overrides)),
        "drive_skeleton": drive_skeleton,
        "pinion_teeth": teeth,
        "tooth_pitch_m": float(tooth_pitch),
        "pitch_radius_m": pitch_radius,
        "phase_offset_rad": offset,
        "external_joint_constraints": external,
    }
    binding["binding_content_sha256"] = v2._self_hash(
        binding, "binding_content_sha256"
    )
    return binding


def _job_with_plan(
    row: Mapping[str, Any],
    plan: Mapping[str, Any],
    *,
    kinematic_constraint_binding: Mapping[str, Any] | None = None,
    package_root_binding: Mapping[str, Any] | None = None,
    execution_paths: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    plan_with_v3_counts = dict(plan)
    plan_with_v3_counts.setdefault("native_mimic_joint_count", 0)
    plan_with_v3_counts.setdefault("external_joint_constraint_count", 0)
    dof = int(row.get("joint_count", 0))
    package = Path(str(row.get("source_path", ""))).resolve()
    urdf = Path(str(row.get("primary_urdf_path", ""))).resolve()
    independent = int(plan_with_v3_counts["independent_dof_count"])
    execution_source_hashes = _source_hashes()
    job: dict[str, Any] = {
        "protocol_id": PROTOCOL_ID,
        "order": int(row["ordinal"]),
        "dataset": "pva",
        "dataset_id": str(row["asset_id"]),
        "asset_id": str(row["asset_id"]),
        "category": str(row.get("raw_category", row.get("category", ""))),
        "package": str(package),
        "urdf_path": str(urdf),
        "primary_urdf_path": str(urdf),
        "primary_urdf_relative_path": str(
            row.get("primary_urdf_relative_path", "model.urdf")
        ),
        "expected_primary_urdf_sha256": row.get("primary_urdf_sha256"),
        "expected_movable_joints": dof,
        "sampling_protocol": SAMPLING_PROTOCOL,
        **plan_with_v3_counts,
        "rest_state_expected": 1,
        "single_state_expected": table4.SINGLE_SAMPLES * independent,
        "sobol_state_expected": table4.SOBOL_SAMPLES if independent else 0,
        "package_binding_sha256": row.get("package_binding_sha256"),
        "roster_ordinal": int(row["ordinal"]),
        "collision_oracle": table4.COLLISION_ORACLE_ZERO_MARGIN,
        "zero_margin_oracle_sha256": execution_source_hashes[
            "zero_margin_oracle"
        ],
        "execution_source_hashes": execution_source_hashes,
        "execution_source_hashes_sha256": table4.canonical_sha256(
            execution_source_hashes
        ),
    }
    constraints = (
        [dict(value) for value in kinematic_constraint_binding["external_joint_constraints"]]
        if kinematic_constraint_binding is not None
        else []
    )
    job.update(
        {
            "external_joint_constraints": constraints,
            "kinematic_constraint_binding": (
                dict(kinematic_constraint_binding)
                if kinematic_constraint_binding is not None
                else None
            ),
            "kinematic_constraint_binding_sha256": (
                kinematic_constraint_binding["binding_content_sha256"]
                if kinematic_constraint_binding is not None
                else None
            ),
        }
    )
    identity_fields = (
        "dataset",
        "dataset_id",
        "category",
        "urdf_path",
        "primary_urdf_relative_path",
        "expected_primary_urdf_sha256",
        "expected_movable_joints",
        "package_binding_sha256",
        "sampling_protocol",
        "independent_dof_count",
        "range_evaluable_independent_dof_count",
        "mimic_joint_count",
        "native_mimic_joint_count",
        "external_joint_constraint_count",
        "external_joint_constraints",
        "kinematic_constraint_binding",
        "kinematic_constraint_binding_sha256",
        "joint_sampling_plan_sha256",
        "single_state_expected",
        "sobol_state_expected",
        "collision_oracle",
        "zero_margin_oracle_sha256",
        "execution_source_hashes",
        "execution_source_hashes_sha256",
    )
    job["input_identity_sha256"] = table4.canonical_sha256(
        {key: job[key] for key in identity_fields}
    )
    if package_root_binding is not None:
        job["package_root_binding_content_sha256"] = package_root_binding[
            "binding_content_sha256"
        ]
        try:
            mapped = dict(
                execution_paths
                if execution_paths is not None
                else v2._mapped_execution_paths(row, package_root_binding)
            )
        except (KeyError, TypeError, ValueError):
            job["execution_input_sha256"] = None
        else:
            job.update(mapped)
            job["execution_input_sha256"] = table4.canonical_sha256(
                {
                    "input_identity_sha256": job["input_identity_sha256"],
                    "package_root_binding_content_sha256": job[
                        "package_root_binding_content_sha256"
                    ],
                    "evaluation_package_relative_path": job[
                        "evaluation_package_relative_path"
                    ],
                    "evaluation_urdf_relative_path": job[
                        "evaluation_urdf_relative_path"
                    ],
                    "package_binding_sha256": job["package_binding_sha256"],
                    "expected_primary_urdf_sha256": job[
                        "expected_primary_urdf_sha256"
                    ],
                }
            )
    return job


def _build_job(
    row: Mapping[str, Any],
    package_root_binding: Mapping[str, Any] | None = None,
    execution_paths: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    if execution_paths is None and package_root_binding is not None:
        execution_paths = v2._verified_execution_paths(row, package_root_binding)
    urdf = Path(
        str(
            execution_paths["evaluation_urdf_path"]
            if execution_paths is not None
            else row.get("primary_urdf_path", "")
        )
    ).resolve()
    package = Path(
        str(
            execution_paths["evaluation_package_path"]
            if execution_paths is not None
            else row.get("source_path", "")
        )
    ).resolve()
    binding = _kinematic_constraint_binding(row, package=package, urdf=urdf)
    external = binding["external_joint_constraints"] if binding is not None else []
    plan = v2._collision_core().sampling_plan_metadata(
        urdf,
        declared_dof=int(row.get("joint_count", 0)),
        expected_sha256=row.get("primary_urdf_sha256"),
        external_joint_constraints=external,
    )
    if plan.get("sampling_plan_error") is not None:
        raise ValueError(f"kinematic-aware sampling plan failed: {plan['sampling_plan_error']}")
    return _job_with_plan(
        row,
        plan,
        kinematic_constraint_binding=binding,
        package_root_binding=package_root_binding,
        execution_paths=execution_paths,
    )


def _validate_result(
    result: Mapping[str, Any], job: Mapping[str, Any]
) -> dict[str, Any]:
    table4._validate_result_binding(result, job)
    validated = _V2_VALIDATE_RESULT(result, job)
    for field in (
        "native_mimic_joint_count",
        "external_joint_constraint_count",
        "kinematic_constraint_binding",
        "kinematic_constraint_binding_sha256",
        "collision_oracle",
        "zero_margin_oracle_sha256",
    ):
        if result.get(field) != job.get(field):
            raise ValueError(f"child result binding mismatch: {field}")
    expected_binding = job.get("kinematic_constraint_binding_sha256")
    states = result.get("state_records", [])
    if any(
        state.get("kinematic_constraint_binding_sha256") != expected_binding
        for state in states
    ):
        raise ValueError("child state kinematic constraint binding mismatch")
    if any(
        state.get("collision_oracle") != table4.COLLISION_ORACLE_ZERO_MARGIN
        for state in states
    ):
        raise ValueError("child state collision oracle binding mismatch")
    return validated


def _validated_source_row(
    values: tuple[Any, ...], expected: int
) -> dict[str, Any]:
    ordinal, asset_id, category, joint_count, row_hash, payload = values
    if int(ordinal) != expected:
        raise ValueError(f"source asset index is not contiguous at {expected}")
    row = json.loads(payload)
    if not isinstance(row, dict):
        raise ValueError(f"source roster row {expected} is not an object")
    if v2._canonical_sha256(row) != str(row_hash):
        raise ValueError(f"source roster row hash mismatch at {expected}")
    if (
        int(row.get("ordinal", -1)) != expected
        or str(row.get("asset_id")) != str(asset_id)
        or str(row.get("raw_category", row.get("category", "")))
        != str(category)
        or int(row.get("joint_count", -1)) != int(joint_count)
    ):
        raise ValueError(f"source roster row binding mismatch at {expected}")
    return row


def _stored_states(blob: bytes, declared_count: int) -> list[dict[str, Any]]:
    try:
        payload = zlib.decompress(blob)
    except zlib.error as exc:
        raise ValueError("result database state payload is not valid zlib") from exc
    if payload.count(b"\n") != declared_count or (
        payload and not payload.endswith(b"\n")
    ):
        raise ValueError("result database compressed state count mismatch")
    states: list[dict[str, Any]] = []
    for index, line in enumerate(payload.splitlines()):
        try:
            state = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(
                f"result database state {index} is invalid JSON"
            ) from exc
        if not isinstance(state, dict):
            raise ValueError(f"result database state {index} is not an object")
        states.append(state)
    return states


def _validated_result_prefix(result: Any, source: Any) -> int:
    """Validate every committed v3 row against its rebuilt frozen job."""

    database_rows = result.execute("PRAGMA database_list").fetchall()
    database_names = [
        Path(str(path)).resolve()
        for _, name, path in database_rows
        if name == "main" and path
    ]
    if len(database_names) != 1:
        raise ValueError("cannot resolve PV-A v3 result database path")
    manifest = v2._load_json(database_names[0].parent / "manifest.json")
    _manifest_bindings(manifest)
    package_root_binding = manifest.get("package_root_binding")
    if package_root_binding is not None and not isinstance(
        package_root_binding, Mapping
    ):
        raise ValueError("PV-A v3 package root binding is invalid")

    source_cursor = source.execute(
        "SELECT ordinal, asset_id, category, joint_count, row_sha256, row_json "
        "FROM assets ORDER BY ordinal"
    )
    result_cursor = result.execute(
        "SELECT ordinal, asset_id, record_json, states_zlib, state_count "
        "FROM results ORDER BY ordinal"
    )
    expected = 0
    for ordinal, asset_id, record_payload, states_blob, state_count in result_cursor:
        try:
            source_values = next(source_cursor)
        except StopIteration as exc:
            raise ValueError("result database extends beyond the frozen source") from exc
        row = _validated_source_row(source_values, expected)
        if int(ordinal) != expected or str(asset_id) != str(row["asset_id"]):
            raise ValueError(f"result/source identity mismatch at ordinal {expected}")
        execution_paths = (
            None
            if package_root_binding is None
            else v2._mapped_execution_paths(row, package_root_binding)
        )
        job = _build_job(
            row,
            package_root_binding=package_root_binding,
            execution_paths=execution_paths,
        )
        try:
            record = json.loads(record_payload)
        except json.JSONDecodeError as exc:
            raise ValueError(
                f"result database record {expected} is invalid JSON"
            ) from exc
        if not isinstance(record, dict):
            raise ValueError(f"result database record {expected} is not an object")
        states = _stored_states(bytes(states_blob), int(state_count))
        record["state_records"] = states
        if int(record.get("state_records_count", len(states))) != int(state_count):
            raise ValueError(f"result database state count mismatch at {expected}")
        _validate_result(record, job)
        expected += 1
    return expected


def _compute_source_hashes() -> dict[str, str]:
    return {
        "adapter": v2._sha256_file(SCRIPT),
        "base_adapter": v2._sha256_file(Path(v2.__file__)),
        "table4_runner": v2._sha256_file(Path(table4.__file__)),
        "table4_core": v2._sha256_file(Path(table4.CORE_SCRIPT)),
        "pva_roster_verifier": v2._sha256_file(Path(pva_roster.__file__)),
        "pva_table1234_runner": v2._sha256_file(Path(pva_run.__file__)),
        "table123_common": v2._sha256_file(Path(pva_run.common.__file__)),
        "kinematic_constraint_registry": v2._sha256_file(
            KINEMATIC_CONSTRAINT_REGISTRY
        ),
        "zero_margin_oracle": v2._sha256_file(
            table4.ZERO_MARGIN_ORACLE_SCRIPT
        ),
    }


def _source_hashes() -> dict[str, str]:
    if _ACTIVE_EXECUTION_SOURCE_HASHES is not None:
        return dict(_ACTIVE_EXECUTION_SOURCE_HASHES)
    return _compute_source_hashes()


def _verify_execution_source_hashes(expected: Mapping[str, str]) -> None:
    observed = _compute_source_hashes()
    if observed != dict(expected):
        changed = sorted(
            key
            for key in set(observed) | set(expected)
            if observed.get(key) != expected.get(key)
        )
        raise ValueError(
            "PV-A v3 execution sources changed during the run: "
            + ", ".join(changed)
        )


def _manifest(*args: Any, **kwargs: Any) -> dict[str, Any]:
    value = _V2_MANIFEST(*args, **kwargs)
    source_hashes = value.get("source_hashes")
    if not isinstance(source_hashes, Mapping):
        raise ValueError("PV-A v3 manifest source hashes are missing")
    value["protocol"]["external_kinematic_constraints"] = {
        "policy": "reviewed_hash_bound_affine_registry_v1",
        "registry": str(KINEMATIC_CONSTRAINT_REGISTRY.resolve()),
        "registry_sha256": _kinematic_constraint_registry_sha256(),
    }
    value["protocol"]["collision_oracle"] = (
        table4.COLLISION_ORACLE_ZERO_MARGIN
    )
    value["protocol"]["zero_margin_oracle_sha256"] = (
        source_hashes["zero_margin_oracle"]
    )
    value["execution_source_hashes_sha256"] = table4.canonical_sha256(
        source_hashes
    )
    value["manifest_content_sha256"] = v2._self_hash(
        value, "manifest_content_sha256"
    )
    return value


def _manifest_bindings(manifest: Mapping[str, Any]) -> tuple[str, str]:
    if manifest.get("schema_version") != RUN_SCHEMA_VERSION:
        raise ValueError("PV-A v3 manifest schema mismatch")
    if manifest.get("protocol_id") != PROTOCOL_ID:
        raise ValueError("PV-A v3 manifest protocol mismatch")
    if manifest.get("manifest_content_sha256") != v2._self_hash(
        manifest, "manifest_content_sha256"
    ):
        raise ValueError("PV-A v3 manifest self-hash mismatch")
    protocol = manifest.get("protocol")
    if not isinstance(protocol, Mapping):
        raise ValueError("PV-A v3 manifest protocol binding is invalid")
    collision_oracle = protocol.get("collision_oracle")
    if collision_oracle != table4.COLLISION_ORACLE_ZERO_MARGIN:
        raise ValueError("PV-A v3 manifest collision oracle mismatch")
    oracle_sha256 = protocol.get("zero_margin_oracle_sha256")
    constraints = protocol.get("external_kinematic_constraints")
    if not isinstance(constraints, Mapping):
        raise ValueError("PV-A v3 manifest kinematic registry binding is invalid")
    registry_sha256 = constraints.get("registry_sha256")
    if not isinstance(oracle_sha256, str) or len(oracle_sha256) != 64:
        raise ValueError("PV-A v3 manifest oracle hash is invalid")
    if not isinstance(registry_sha256, str) or len(registry_sha256) != 64:
        raise ValueError("PV-A v3 manifest registry hash is invalid")
    source_hashes = manifest.get("source_hashes")
    if not isinstance(source_hashes, Mapping):
        raise ValueError("PV-A v3 manifest source hashes are invalid")
    required_source_hashes = {
        "adapter",
        "base_adapter",
        "table4_runner",
        "table4_core",
        "pva_roster_verifier",
        "pva_table1234_runner",
        "table123_common",
        "kinematic_constraint_registry",
        "zero_margin_oracle",
    }
    if set(source_hashes) != required_source_hashes:
        raise ValueError("PV-A v3 manifest execution source set mismatch")
    table4._validate_v3_execution_source_binding(
        {
            "execution_source_hashes": source_hashes,
            "execution_source_hashes_sha256": manifest.get(
                "execution_source_hashes_sha256"
            ),
        },
        verify_current_files=False,
    )
    if source_hashes.get("zero_margin_oracle") != oracle_sha256:
        raise ValueError("PV-A v3 manifest oracle source hash mismatch")
    if source_hashes.get("kinematic_constraint_registry") != registry_sha256:
        raise ValueError("PV-A v3 manifest registry source hash mismatch")
    return oracle_sha256, registry_sha256


def _finalize(*args: Any, **kwargs: Any) -> dict[str, Any]:
    if len(args) >= 3:
        connection, output, manifest = args[:3]
    else:
        connection = kwargs["connection"]
        output = kwargs["output"]
        manifest = kwargs["manifest"]
    output = Path(output)
    oracle_sha256, registry_sha256 = _manifest_bindings(manifest)
    fatal = connection.execute(
        "SELECT ordinal, asset_id FROM results WHERE "
        "json_extract(record_json, '$.execution_source_integrity') = 'failed' "
        "OR instr(record_json, ?) > 0 ORDER BY ordinal LIMIT 1",
        (table4.EXECUTION_SOURCE_INTEGRITY_FATAL,),
    ).fetchone()
    if fatal is not None:
        raise RuntimeError(
            f"{table4.EXECUTION_SOURCE_INTEGRITY_FATAL}: "
            f"asset {fatal[1]} at ordinal {fatal[0]}"
        )
    _verify_execution_source_hashes(manifest["source_hashes"])
    summary = _V2_FINALIZE(*args, **kwargs)
    native_mimic, external_constraints = connection.execute(
        "SELECT "
        "COALESCE(SUM(CAST(json_extract(record_json, "
        "'$.native_mimic_joint_count') AS INTEGER)), 0), "
        "COALESCE(SUM(CAST(json_extract(record_json, "
        "'$.external_joint_constraint_count') AS INTEGER)), 0) "
        "FROM results"
    ).fetchone()
    summary.update(
        {
            "schema_version": SUMMARY_SCHEMA_VERSION,
            "protocol_id": PROTOCOL_ID,
            "collision_oracle": table4.COLLISION_ORACLE_ZERO_MARGIN,
            "zero_margin_oracle_sha256": oracle_sha256,
            "kinematic_constraint_registry_sha256": registry_sha256,
            "execution_source_hashes_sha256": manifest[
                "execution_source_hashes_sha256"
            ],
            "native_mimic_joint_count": int(native_mimic),
            "external_joint_constraint_count": int(external_constraints),
        }
    )
    summary["summary_content_sha256"] = v2._self_hash(
        summary, "summary_content_sha256"
    )
    v2._atomic_json(output / "summary.json", summary)
    pva_run.common._atomic_write_bytes(
        output / "summary.md",
        table4.report_text(summary, "pva").encode("utf-8"),
    )

    checkpoint_path = output / "checkpoint.json"
    checkpoint = v2._load_json(checkpoint_path)
    if checkpoint.get("checkpoint_content_sha256") != v2._self_hash(
        checkpoint, "checkpoint_content_sha256"
    ):
        raise ValueError("PV-A v3 checkpoint self-hash mismatch before finalization")
    checkpoint.update(
        {
            "schema_version": CHECKPOINT_SCHEMA_VERSION,
            "protocol_id": PROTOCOL_ID,
            "collision_oracle": table4.COLLISION_ORACLE_ZERO_MARGIN,
            "zero_margin_oracle_sha256": oracle_sha256,
            "kinematic_constraint_registry_sha256": registry_sha256,
            "execution_source_hashes_sha256": manifest[
                "execution_source_hashes_sha256"
            ],
            "native_mimic_joint_count": int(native_mimic),
            "external_joint_constraint_count": int(external_constraints),
            "summary_sha256": v2._sha256_file(output / "summary.json"),
        }
    )
    checkpoint["checkpoint_content_sha256"] = v2._self_hash(
        checkpoint, "checkpoint_content_sha256"
    )
    v2._atomic_json(checkpoint_path, checkpoint)
    return summary


def _write_progress(*args: Any, **kwargs: Any) -> None:
    _V2_WRITE_PROGRESS(*args, **kwargs)
    output = Path(args[0] if args else kwargs["output"])
    progress = v2._load_json(output / "progress.json")
    progress.update(
        {
            "schema_version": PROGRESS_SCHEMA_VERSION,
            "protocol_id": PROTOCOL_ID,
            "collision_oracle": table4.COLLISION_ORACLE_ZERO_MARGIN,
            "execution_source_hashes_sha256": table4.canonical_sha256(
                _source_hashes()
            ),
        }
    )
    v2._atomic_json(output / "progress.json", progress)


def _artifact_manifest(output: Path) -> dict[str, Any]:
    value = _V2_ARTIFACT_MANIFEST(output)
    value["schema_version"] = ARTIFACT_SCHEMA_VERSION
    value["artifact_manifest_content_sha256"] = v2._self_hash(
        value, "artifact_manifest_content_sha256"
    )
    return value


@contextmanager
def _frozen_execution_sources(source_hashes: Mapping[str, str]) -> Any:
    """Make one manifest snapshot authoritative for every job in a run."""

    global _ACTIVE_EXECUTION_SOURCE_HASHES
    with _V3_HOOK_LOCK:
        previous = _ACTIVE_EXECUTION_SOURCE_HASHES
        _ACTIVE_EXECUTION_SOURCE_HASHES = dict(source_hashes)
        try:
            yield
        finally:
            _ACTIVE_EXECUTION_SOURCE_HASHES = previous


@contextmanager
def _v3_hooks() -> Any:
    """Install v3 hooks only for one run, then restore the v2 module."""

    replacements = {
        "RUN_SCHEMA_VERSION": RUN_SCHEMA_VERSION,
        "RESULT_DB_SCHEMA_VERSION": RESULT_DB_SCHEMA_VERSION,
        "RECEIPT_SCHEMA_VERSION": RECEIPT_SCHEMA_VERSION,
        "PROTOCOL_ID": PROTOCOL_ID,
        "_job_with_plan": _job_with_plan,
        "_build_job": _build_job,
        "_validate_result": _validate_result,
        "_validated_result_prefix": _validated_result_prefix,
        "_source_hashes": _source_hashes,
        "_manifest": _manifest,
        "_finalize": _finalize,
        "_write_progress": _write_progress,
        "_artifact_manifest": _artifact_manifest,
    }
    with _V3_HOOK_LOCK:
        previous = {name: getattr(v2, name) for name in replacements}
        try:
            for name, value in replacements.items():
                setattr(v2, name, value)
            yield
        finally:
            for name, value in previous.items():
                setattr(v2, name, value)


def _seal_v3_receipt(output: Path) -> None:
    manifest = v2._load_json(Path(output) / "manifest.json")
    oracle_sha256, registry_sha256 = _manifest_bindings(manifest)
    path = Path(output) / "full_release_receipt.json"
    receipt = v2._load_json(path)
    if receipt.get("schema_version") != RECEIPT_SCHEMA_VERSION:
        raise ValueError("PV-A v3 receipt schema mismatch")
    if receipt.get("protocol_id") != PROTOCOL_ID:
        raise ValueError("PV-A v3 receipt protocol mismatch")
    if receipt.get("receipt_content_sha256") != v2._self_hash(
        receipt, "receipt_content_sha256"
    ):
        raise ValueError("PV-A v3 receipt self-hash mismatch before oracle binding")
    receipt.update(
        {
            "collision_oracle": table4.COLLISION_ORACLE_ZERO_MARGIN,
            "zero_margin_oracle_sha256": oracle_sha256,
            "kinematic_constraint_registry_sha256": registry_sha256,
            "execution_source_hashes_sha256": manifest[
                "execution_source_hashes_sha256"
            ],
        }
    )
    receipt["receipt_content_sha256"] = v2._self_hash(
        receipt, "receipt_content_sha256"
    )
    v2._atomic_json(path, receipt)


def run_pva_table4_v3(
    source_evaluation: Path,
    output: Path,
    *,
    workers: int = 32,
    timeout_seconds: float = 600.0,
    resume: bool = False,
    limit: int | None = None,
    batch_size: int | None = None,
    package_root_override: Path | None = None,
) -> Path:
    frozen_source_hashes = _compute_source_hashes()
    frozen_oracle_sha256 = frozen_source_hashes["zero_margin_oracle"]
    frozen_registry_sha256 = frozen_source_hashes[
        "kinematic_constraint_registry"
    ]
    _verify_zero_margin_oracle_sha256(frozen_oracle_sha256)
    _verify_kinematic_constraint_registry_sha256(frozen_registry_sha256)
    _verify_execution_source_hashes(frozen_source_hashes)
    with _frozen_execution_sources(frozen_source_hashes):
        with _v3_hooks():
            completed = _V2_RUN(
                source_evaluation,
                output,
                workers=workers,
                timeout_seconds=timeout_seconds,
                resume=resume,
                limit=limit,
                batch_size=batch_size,
                package_root_override=package_root_override,
            )
            manifest = v2._load_json(Path(completed) / "manifest.json")
            _manifest_bindings(manifest)
            if manifest.get("source_hashes") != frozen_source_hashes:
                raise ValueError("PV-A v3 manifest execution source hashes drifted")
            _verify_execution_source_hashes(frozen_source_hashes)
            _seal_v3_receipt(completed)
            return completed


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-evaluation", type=Path, default=DEFAULT_SOURCE_EVALUATION
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--workers", type=int, default=32)
    parser.add_argument("--timeout-seconds", type=float, default=600.0)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--limit", type=int)
    parser.add_argument("--batch-size", type=int)
    parser.add_argument("--package-root-override", type=Path)
    args = parser.parse_args(argv)
    try:
        output = run_pva_table4_v3(
            args.source_evaluation,
            args.output,
            workers=args.workers,
            timeout_seconds=args.timeout_seconds,
            resume=args.resume,
            limit=args.limit,
            batch_size=args.batch_size,
            package_root_override=args.package_root_override,
        )
    except Exception as error:  # noqa: BLE001
        print(f"ERROR: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(v2._canonical_text({"status": "COMPLETE", "output": str(output)}))
    return 0


def __getattr__(name: str) -> Any:
    return getattr(v2, name)


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "PROTOCOL_ID",
    "RECEIPT_SCHEMA_VERSION",
    "RESULT_DB_SCHEMA_VERSION",
    "RUN_SCHEMA_VERSION",
    "run_pva_table4_v3",
]
