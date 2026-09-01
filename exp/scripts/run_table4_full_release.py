#!/usr/bin/env python3
"""Automated full-release evaluator for primary URDF Sim-Ready Table 4.

The runner evaluates the eight immutable comparison rosters used by the
full-release Table 1--3 run.  Each asset is isolated in a fresh interpreter
and every expected state remains in the denominator when a package cannot be
loaded.  Collision-free claims are never made for an URDF without native
collision geometry: those assets are recorded as ``N/E``/blocked.

The implementation intentionally keeps the evaluator independent of the old
N=800 adapters.  It reuses only the frozen PyBullet sampling and contact
oracle helpers from ``run_urdf_table4_partnet_mobility.py``.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any, Mapping, Sequence
import xml.etree.ElementTree as ET


SCRIPT = Path(__file__).resolve()
EXP_ROOT = SCRIPT.parents[1]
REPO_ROOT = SCRIPT.parents[2]
ROSTER_ROOT = EXP_ROOT / "runtime" / "table123_full_release_20260825" / "rosters"
DEFAULT_OUTPUT_ROOT = EXP_ROOT / "runtime" / "table4_full_release_20260826"
DATASETS = ("articraft", "lam", "artiverse", "partnet", "physx", "sketch", "infinite", "infinigen")
DISPLAY_NAMES = {
    "articraft": "Articraft-10K",
    "lam": "LAM released outputs",
    "artiverse": "Artiverse",
    "partnet": "PartNet-Mobility",
    "physx": "PhysX-Mobility",
    "sketch": "SketchMobility",
    "infinite": "Infinite Mobility",
    "infinigen": "Infinigen-Sim",
}
ROSTER_SCHEMA = "table123_full_release_manifest_v1"
# Public aliases kept stable for the focused contract tests and downstream
# automation.  The values are semantic labels, not pass/fail shortcuts.
ROSTER_SCHEMA_VERSION = ROSTER_SCHEMA
SCHEMA_VERSION = "table4_full_release_run_v1"
SCHEMA_VERSION_V2 = "table4_full_release_run_v2"
SCHEMA_VERSION_V3 = "table4_full_release_run_v3"
SUMMARY_SCHEMA = "table4_full_release_summary_v1"
SUMMARY_SCHEMA_V2 = "table4_full_release_summary_v2"
SUMMARY_SCHEMA_V3 = "table4_full_release_summary_v3"
RECEIPT_SCHEMA = "table4_full_release_receipt_v1"
RECEIPT_SCHEMA_V2 = "table4_full_release_receipt_v2"
RECEIPT_SCHEMA_V3 = "table4_full_release_receipt_v3"
RECEIPT_SCHEMA_VERSION = RECEIPT_SCHEMA
BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT = "N/E"
MEASURED_COLLISION_GEOMETRY = "MEASURED"
SINGLE_SAMPLES = 21
SOBOL_SAMPLES = 64
SOBOL_SEED = 20260813
PENETRATION_THRESHOLD_M = 1e-6
RESET_TOLERANCE = 1e-9
CORE_SCRIPT = EXP_ROOT / "scripts" / "run_urdf_table4_partnet_mobility.py"
ZERO_MARGIN_ORACLE_SCRIPT = EXP_ROOT / "scripts" / "table4_zero_margin_oracle.py"
SAMPLING_PROTOCOL_V1 = "independent_sampling_v1"
SAMPLING_PROTOCOL_V2 = "mimic_aware_independent_sampling_v2"
COLLISION_ORACLE_LEGACY = "pybullet_urdf_raw_margin_v1"
COLLISION_ORACLE_ZERO_MARGIN = (
    "pybullet_per_collision_element_numerical_zero_margin_proxy_v1"
)
COLLISION_ORACLES = (COLLISION_ORACLE_LEGACY, COLLISION_ORACLE_ZERO_MARGIN)
EXECUTION_SOURCE_INTEGRITY_FATAL = "execution_source_integrity_fatal"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(value: Any) -> str:
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _generic_v3_execution_source_hashes() -> dict[str, str]:
    """Hash every source file executed by a generic Table 4 v3 child."""

    return {
        "table4_runner": sha256_file(SCRIPT),
        "table4_core": sha256_file(CORE_SCRIPT),
        "zero_margin_oracle": sha256_file(ZERO_MARGIN_ORACLE_SCRIPT),
    }


def _validate_v3_execution_source_binding(
    value: Mapping[str, Any],
    *,
    verify_current_files: bool,
) -> dict[str, str]:
    source_hashes = value.get("execution_source_hashes")
    if not isinstance(source_hashes, Mapping) or not source_hashes:
        raise ValueError("v3 execution source hashes are missing")
    normalized: dict[str, str] = {}
    for key, digest in source_hashes.items():
        if (
            not isinstance(key, str)
            or not key
            or not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise ValueError("v3 execution source hash map is invalid")
        normalized[key] = digest
    declared_digest = value.get("execution_source_hashes_sha256")
    observed_digest = canonical_sha256(normalized)
    if declared_digest != observed_digest:
        raise ValueError("v3 execution source hash map digest mismatch")

    known_paths = {
        "adapter": EXP_ROOT
        / "scripts"
        / "run_pva_table4_kinematic_aware_full_release.py",
        "base_adapter": EXP_ROOT
        / "scripts"
        / "run_pva_table4_mimic_aware_full_release.py",
        "table4_runner": SCRIPT,
        "table4_core": CORE_SCRIPT,
        "pva_roster_verifier": EXP_ROOT
        / "scripts"
        / "build_pva_full_release_roster.py",
        "pva_table1234_runner": EXP_ROOT
        / "scripts"
        / "run_pva_table1234_full_release.py",
        "table123_common": EXP_ROOT
        / "scripts"
        / "table123_full_release_common.py",
        "kinematic_constraint_registry": EXP_ROOT
        / "reference"
        / "pva_table4_kinematic_constraints_v1.json",
        "zero_margin_oracle": ZERO_MARGIN_ORACLE_SCRIPT,
    }
    for required in (
        "table4_runner",
        "table4_core",
        "zero_margin_oracle",
    ):
        if required not in normalized:
            raise ValueError(f"v3 execution source hash is missing: {required}")
    for key, expected in normalized.items():
        path = known_paths.get(key)
        if verify_current_files and path is None:
            raise ValueError(f"v3 execution source path is unknown: {key}")
        if verify_current_files and sha256_file(path) != expected:
            raise ValueError(f"v3 execution source changed: {key}")
    return normalized


def _without(value: Mapping[str, Any], field: str) -> dict[str, Any]:
    result = dict(value)
    result.pop(field, None)
    return result


def _self_hash(value: Mapping[str, Any], field: str) -> str:
    return canonical_sha256(_without(value, field))


def _atomic_write(path: Path, payload: str) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.is_symlink():
        raise ValueError(f"refusing to overwrite symlink: {path}")
    fd, name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    temporary = Path(name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path: Path, value: Any) -> None:
    _atomic_write(path, json.dumps(value, sort_keys=True, indent=2, ensure_ascii=True, allow_nan=False) + "\n")


def atomic_jsonl(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    text = "".join(json.dumps(dict(row), sort_keys=True, ensure_ascii=True, allow_nan=False) + "\n" for row in rows)
    _atomic_write(path, text)


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def _load_json(path: Path) -> Any:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _core() -> Any:
    """Load the frozen contact helpers without entering the legacy CLI."""

    import importlib.util

    spec = importlib.util.spec_from_file_location("table4_frozen_collision_core", CORE_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import collision core: {CORE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _zero_margin_oracle() -> Any:
    """Load the separately hashed v3 collision oracle on explicit request."""

    import importlib.util

    name = "table4_zero_margin_oracle"
    spec = importlib.util.spec_from_file_location(name, ZERO_MARGIN_ORACLE_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import zero-margin oracle: {ZERO_MARGIN_ORACLE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    # ``dataclasses`` resolves postponed annotations through ``sys.modules``.
    sys.modules[name] = module
    spec.loader.exec_module(module)
    if module.ORACLE_POLICY_ID != COLLISION_ORACLE_ZERO_MARGIN:
        raise RuntimeError("zero-margin oracle policy ID mismatch")
    return module


def runtime_identity() -> dict[str, Any]:
    import pybullet
    import scipy
    return {
        "python_executable": str(Path(sys.executable).resolve()),
        "python_version": sys.version,
        "pybullet_api_version": pybullet.getAPIVersion(),
        "pybullet_module": str(Path(pybullet.__file__).resolve()),
        "pybullet_module_sha256": sha256_file(Path(pybullet.__file__).resolve()),
        "scipy_version": scipy.__version__,
        "runner_sha256": sha256_file(SCRIPT),
        "collision_core_sha256": sha256_file(CORE_SCRIPT),
    }


def _row_package(row: Mapping[str, Any]) -> Path:
    value = row.get("package_path") or row.get("source_path") or row.get("package_root")
    if not isinstance(value, str) or not value:
        raise ValueError(f"row {row.get('asset_id')} has no source package")
    return Path(value).resolve(strict=True)


def _row_urdf(row: Mapping[str, Any]) -> Path:
    value = row.get("primary_urdf_path") or row.get("urdf_path")
    if not isinstance(value, str) or not value:
        raise ValueError(f"row {row.get('asset_id')} has no primary URDF")
    path = Path(value).resolve(strict=True)
    package = _row_package(row)
    # A source row is immutable; accepting only a URDF inside its package
    # prevents an accidental absolute-path substitution during a run.
    try:
        path.relative_to(package)
    except ValueError as exc:
        raise ValueError(f"primary URDF escapes package: {path}") from exc
    return path


def build_jobs(
    manifest: Mapping[str, Any],
    dataset: str,
    *,
    sampling_protocol: str = SAMPLING_PROTOCOL_V1,
    collision_oracle: str | None = None,
) -> list[dict[str, Any]]:
    if manifest.get("schema_version") != ROSTER_SCHEMA:
        raise ValueError("table123_full_release_manifest_v1 required")
    rows = manifest.get("rows")
    if not isinstance(rows, list):
        raise ValueError("roster manifest has no rows")
    if int(manifest.get("N_eval", -1)) != len(rows):
        raise ValueError("roster N_eval mismatch")
    declared_j = sum(int(row.get("joint_count", len(row.get("non_fixed_joints", [])))) for row in rows if isinstance(row, Mapping))
    if int(manifest.get("J_eval", -1)) != declared_j:
        raise ValueError("roster J_eval mismatch")
    if sampling_protocol not in {SAMPLING_PROTOCOL_V1, SAMPLING_PROTOCOL_V2}:
        raise ValueError(f"unknown sampling protocol: {sampling_protocol}")
    effective_collision_oracle = collision_oracle or COLLISION_ORACLE_LEGACY
    if effective_collision_oracle not in COLLISION_ORACLES:
        raise ValueError(f"unknown collision oracle: {effective_collision_oracle}")
    execution_source_hashes = (
        _generic_v3_execution_source_hashes()
        if effective_collision_oracle == COLLISION_ORACLE_ZERO_MARGIN
        else None
    )
    zero_margin_oracle_sha256 = (
        execution_source_hashes["zero_margin_oracle"]
        if execution_source_hashes is not None
        else None
    )
    core_module = _core() if sampling_protocol == SAMPLING_PROTOCOL_V2 else None
    jobs: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        if not isinstance(row, Mapping) or not isinstance(row.get("asset_id"), str):
            raise ValueError(f"invalid roster row {index}")
        package = _row_package(row)
        urdf = _row_urdf(row)
        dof = int(row.get("joint_count", len(row.get("non_fixed_joints", []))))
        if dof < 0:
            raise ValueError(f"negative joint count at {index}")
        version = (
            "v3"
            if effective_collision_oracle == COLLISION_ORACLE_ZERO_MARGIN
            else ("v2" if sampling_protocol == SAMPLING_PROTOCOL_V2 else "v1")
        )
        job = {
            "protocol_id": f"urdf_sim_ready_table4_{dataset}_full_release_{version}",
            "order": index,
            "dataset": dataset,
            "dataset_id": str(row["asset_id"]),
            "asset_id": str(row["asset_id"]),
            "category": str(row.get("category") or row.get("raw_category") or ""),
            "package": str(package),
            "urdf_path": str(urdf),
            "primary_urdf_path": str(urdf),
            "primary_urdf_relative_path": str(row.get("primary_urdf_relative_path") or urdf.relative_to(package)),
            "expected_primary_urdf_sha256": row.get("primary_urdf_sha256"),
            "expected_movable_joints": dof,
            "sampling_protocol": sampling_protocol,
            "independent_dof_count": dof,
            "range_evaluable_independent_dof_count": 0,
            "mimic_joint_count": 0,
            "fixed_root_joint_count": 0,
            "joint_sampling_plan_sha256": None,
            "sampling_plan_error": None,
            "rest_state_expected": 1,
            "single_state_expected": SINGLE_SAMPLES * dof,
            "sobol_state_expected": SOBOL_SAMPLES if dof > 0 else 0,
            "package_binding_sha256": row.get("package_binding_sha256"),
            "roster_ordinal": row.get("ordinal", index),
        }
        if collision_oracle is not None:
            job["collision_oracle"] = effective_collision_oracle
        if effective_collision_oracle == COLLISION_ORACLE_ZERO_MARGIN:
            job["zero_margin_oracle_sha256"] = zero_margin_oracle_sha256
            job["execution_source_hashes"] = dict(execution_source_hashes or {})
            job["execution_source_hashes_sha256"] = canonical_sha256(
                job["execution_source_hashes"]
            )
        if sampling_protocol == SAMPLING_PROTOCOL_V2:
            assert core_module is not None
            plan_meta = core_module.sampling_plan_metadata(
                urdf,
                declared_dof=dof,
                expected_sha256=row.get("primary_urdf_sha256"),
            )
            job.update(plan_meta)
            independent = int(plan_meta["independent_dof_count"])
            job["single_state_expected"] = SINGLE_SAMPLES * independent
            job["sobol_state_expected"] = SOBOL_SAMPLES if independent > 0 else 0
        input_identity = {key: job[key] for key in (
            "dataset", "dataset_id", "category", "urdf_path", "primary_urdf_relative_path",
            "expected_primary_urdf_sha256", "expected_movable_joints", "package_binding_sha256",
            "sampling_protocol", "independent_dof_count",
            "range_evaluable_independent_dof_count", "mimic_joint_count",
            "joint_sampling_plan_sha256", "single_state_expected",
            "sobol_state_expected",
        )}
        if collision_oracle is not None:
            input_identity["collision_oracle"] = effective_collision_oracle
        if effective_collision_oracle == COLLISION_ORACLE_ZERO_MARGIN:
            input_identity["zero_margin_oracle_sha256"] = job[
                "zero_margin_oracle_sha256"
            ]
            input_identity["execution_source_hashes"] = job[
                "execution_source_hashes"
            ]
            input_identity["execution_source_hashes_sha256"] = job[
                "execution_source_hashes_sha256"
            ]
        job["input_identity_sha256"] = canonical_sha256(input_identity)
        jobs.append(job)
    if len({job["dataset_id"] for job in jobs}) != len(jobs):
        raise ValueError("duplicate asset IDs in roster")
    return jobs


def load_roster(
    path: Path,
    *,
    dataset: str | None = None,
    sampling_protocol: str = SAMPLING_PROTOCOL_V1,
    collision_oracle: str | None = None,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Load and validate one full-release roster without touching payloads."""

    manifest = _load_json(Path(path).resolve(strict=True))
    if not isinstance(manifest, dict):
        raise ValueError("roster manifest must be an object")
    if manifest.get("schema_version") != ROSTER_SCHEMA:
        raise ValueError("table123_full_release_manifest_v1 required")
    declared = manifest.get("manifest_content_sha256")
    if declared is not None and declared != _self_hash(manifest, "manifest_content_sha256"):
        raise ValueError("roster manifest self-hash mismatch")
    effective_dataset = str(dataset or manifest.get("dataset") or "fixture").lower()
    # ``build_jobs`` only uses the dataset value as an identity component; a
    # fixture or private roster is intentionally accepted by this low-level
    # API.  The public all-datasets CLI still restricts its choices.
    return manifest, build_jobs(
        manifest,
        effective_dataset,
        sampling_protocol=sampling_protocol,
        collision_oracle=collision_oracle,
    )


def _resolve_mesh(package: Path, urdf: Path, filename: str) -> Path:
    raw = filename.replace("\\", "/").strip()
    if not raw:
        raise ValueError("empty collision mesh filename")
    if (
        raw.startswith("file://")
        or Path(raw).is_absolute()
        or (len(raw) >= 3 and raw[1] == ":" and raw[2] == "/")
    ):
        raise ValueError(f"absolute/file URI collision mesh reference is forbidden: {filename}")
    if raw.startswith("package://"):
        remainder = raw[len("package://"):]
        pieces = remainder.split("/", 1)
        candidate = package / (pieces[1] if len(pieces) == 2 else pieces[0])
        candidate = candidate.resolve(strict=False)
    else:
        candidate = (urdf.parent / raw).resolve(strict=False)
    try:
        candidate.relative_to(package)
    except ValueError as exc:
        raise ValueError(f"collision mesh escapes package: {filename}") from exc
    return candidate


def collision_inventory(package: Path, urdf: Path) -> dict[str, Any]:
    root = ET.parse(urdf).getroot()
    links = root.findall("link")
    collision_elements = 0
    mesh_refs: list[str] = []
    geometry_kinds: list[str] = []
    missing: list[str] = []
    for link in links:
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            if geometry is None or len(list(geometry)) == 0:
                continue
            collision_elements += 1
            for shape in list(geometry):
                geometry_kinds.append(str(shape.tag))
                if shape.tag == "mesh":
                    filename = str(shape.get("filename") or "")
                    mesh_refs.append(filename)
                    normalized = filename.replace("\\", "/").strip()
                    if (
                        normalized.startswith("file://")
                        or Path(normalized).is_absolute()
                        or (
                            len(normalized) >= 3
                            and normalized[1] == ":"
                            and normalized[2] == "/"
                        )
                    ):
                        raise ValueError(
                            "absolute/file URI collision mesh reference is forbidden: "
                            + filename
                        )
                    try:
                        path = _resolve_mesh(package, urdf, filename)
                    except ValueError:
                        missing.append(filename)
                    else:
                        if not path.is_file():
                            missing.append(filename)
    return {
        "link_count": len(links),
        "native_collision_elements": collision_elements,
        "collision_mesh_references": sorted(set(mesh_refs)),
        "missing_collision_mesh_references": sorted(set(missing)),
        "geometry_kinds": sorted(geometry_kinds),
    }


def _result_schema_for_job(job: Mapping[str, Any]) -> str:
    collision_oracle = str(
        job.get("collision_oracle", COLLISION_ORACLE_LEGACY)
    )
    if collision_oracle == COLLISION_ORACLE_ZERO_MARGIN:
        return SCHEMA_VERSION_V3
    if job.get("sampling_protocol") == SAMPLING_PROTOCOL_V2:
        return SCHEMA_VERSION_V2
    return SCHEMA_VERSION


def _state_schema_for_job(job: Mapping[str, Any]) -> str:
    collision_oracle = str(
        job.get("collision_oracle", COLLISION_ORACLE_LEGACY)
    )
    if collision_oracle == COLLISION_ORACLE_ZERO_MARGIN:
        return "table4_state_v3"
    if job.get("sampling_protocol") == SAMPLING_PROTOCOL_V2:
        return "table4_state_v2"
    return "table4_state_v1"


def _validate_result_binding(
    result: Mapping[str, Any],
    job: Mapping[str, Any],
) -> None:
    """Reject stale child/resume rows before they enter a frozen run."""

    if not isinstance(result, Mapping):
        raise ValueError("child result is not an object")
    required_bindings = {
        "schema_version": _result_schema_for_job(job),
        "protocol_id": job.get("protocol_id"),
        "dataset": job.get("dataset"),
        "dataset_id": str(job.get("dataset_id")),
        "order": int(job.get("order", 0)),
        "sampling_protocol": job.get(
            "sampling_protocol", SAMPLING_PROTOCOL_V1
        ),
        "input_identity_sha256": job.get("input_identity_sha256"),
        "urdf_path": job.get("urdf_path"),
        "primary_urdf_relative_path": job.get(
            "primary_urdf_relative_path"
        ),
        "expected_primary_urdf_sha256": job.get(
            "expected_primary_urdf_sha256"
        ),
        "joint_sampling_plan_sha256": job.get(
            "joint_sampling_plan_sha256"
        ),
    }
    for field, expected in required_bindings.items():
        if result.get(field) != expected:
            raise ValueError(
                f"child result binding mismatch for {field}: "
                f"{result.get(field)!r} != {expected!r}"
            )

    explicit_oracle = "collision_oracle" in job
    if explicit_oracle:
        if result.get("collision_oracle") != job.get("collision_oracle"):
            raise ValueError("child result collision oracle mismatch")
    elif "collision_oracle" in result:
        raise ValueError("legacy child result unexpectedly declares collision oracle")

    zero_margin = (
        job.get("collision_oracle") == COLLISION_ORACLE_ZERO_MARGIN
    )
    if zero_margin:
        _validate_v3_execution_source_binding(
            job, verify_current_files=False
        )
        if (
            result.get("execution_source_integrity") == "failed"
            or any(
                EXECUTION_SOURCE_INTEGRITY_FATAL in str(issue)
                for issue in (result.get("issues") or [])
            )
        ):
            raise ValueError(EXECUTION_SOURCE_INTEGRITY_FATAL)
        if result.get("zero_margin_oracle_sha256") != job.get(
            "zero_margin_oracle_sha256"
        ):
            raise ValueError("child result zero-margin oracle SHA-256 mismatch")
        for field in (
            "execution_source_hashes",
            "execution_source_hashes_sha256",
        ):
            if result.get(field) != job.get(field):
                raise ValueError(f"child result binding mismatch for {field}")
    elif "zero_margin_oracle_sha256" in result:
        raise ValueError("non-v3 child result declares zero-margin oracle SHA-256")

    states = result.get("state_records")
    if not isinstance(states, list):
        raise ValueError("child result state_records is not a list")
    expected_state_hash = canonical_sha256(states)
    if result.get("state_records_sha256") != expected_state_hash:
        raise ValueError("child result state_records SHA-256 mismatch")
    if int(result.get("state_records_count", len(states))) != len(states):
        raise ValueError("child result state_records count mismatch")

    state_bindings = {
        "schema_version": _state_schema_for_job(job),
        "protocol_id": job.get("protocol_id"),
        "dataset": job.get("dataset"),
        "dataset_id": str(job.get("dataset_id")),
        "order": int(job.get("order", 0)),
        "sampling_protocol": job.get(
            "sampling_protocol", SAMPLING_PROTOCOL_V1
        ),
        "input_identity_sha256": job.get("input_identity_sha256"),
        "joint_sampling_plan_sha256": job.get(
            "joint_sampling_plan_sha256"
        ),
    }
    for state_index, state in enumerate(states):
        if not isinstance(state, Mapping):
            raise ValueError(f"child state {state_index} is not an object")
        for field, expected in state_bindings.items():
            if state.get(field) != expected:
                raise ValueError(
                    f"child state {state_index} binding mismatch for {field}"
                )
        if explicit_oracle:
            if state.get("collision_oracle") != job.get("collision_oracle"):
                raise ValueError(
                    f"child state {state_index} collision oracle mismatch"
                )
        elif "collision_oracle" in state:
            raise ValueError(
                f"legacy child state {state_index} unexpectedly declares oracle"
            )
        if zero_margin:
            if state.get("zero_margin_oracle_sha256") != job.get(
                "zero_margin_oracle_sha256"
            ):
                raise ValueError(
                    f"child state {state_index} zero-margin oracle SHA-256 mismatch"
                )
            if state.get("execution_source_hashes_sha256") != job.get(
                "execution_source_hashes_sha256"
            ):
                raise ValueError(
                    f"child state {state_index} binding mismatch for "
                    "execution_source_hashes_sha256"
                )
        elif "zero_margin_oracle_sha256" in state:
            raise ValueError(
                f"non-v3 child state {state_index} declares oracle SHA-256"
            )


def _empty_record(job: Mapping[str, Any], issue: str, *, status: str = "error") -> dict[str, Any]:
    dof = int(job.get("expected_movable_joints", 0))
    independent = int(job.get("independent_dof_count", dof))
    collision_oracle = str(
        job.get("collision_oracle", COLLISION_ORACLE_LEGACY)
    )
    result = {
        "schema_version": _result_schema_for_job(job),
        "protocol_id": job.get("protocol_id"),
        "dataset": job.get("dataset"),
        "order": int(job.get("order", 0)),
        "dataset_id": str(job.get("dataset_id")),
        "category": job.get("category", ""),
        "package": job.get("package"),
        "urdf_path": job.get("urdf_path"),
        "primary_urdf_relative_path": job.get("primary_urdf_relative_path"),
        "expected_primary_urdf_sha256": job.get("expected_primary_urdf_sha256"),
        "expected_movable_joints": dof,
        "sampling_protocol": job.get("sampling_protocol", SAMPLING_PROTOCOL_V1),
        "independent_dof_count": independent,
        "range_evaluable_independent_dof_count": int(
            job.get("range_evaluable_independent_dof_count", 0)
        ),
        "mimic_joint_count": int(job.get("mimic_joint_count", 0)),
        "fixed_root_joint_count": int(job.get("fixed_root_joint_count", 0)),
        "joint_sampling_plan_sha256": job.get("joint_sampling_plan_sha256"),
        "sampling_plan_error": job.get("sampling_plan_error"),
        "input_identity_sha256": job.get("input_identity_sha256"),
        "status": status,
        "collision_metric_status": BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT,
        "native_collision_elements": 0,
        "load_success": False,
        "measurement_complete": False,
        "movable_dof_count": dof,
        "range_evaluable_dof_count": 0,
        "rest_state_expected": 1,
        "rest_state_executed": 0,
        "rest_all_pair_cf": None,
        "rest_non_adjacent_cf": None,
        "rest_non_adjacent_free": 0,
        "single_state_expected": int(
            job.get("single_state_expected", SINGLE_SAMPLES * independent)
        ),
        "single_state_executed": 0,
        "single_non_adjacent_free": 0,
        "single_joint_sweep_cf": None,
        "joint_single_sweep_cf_passed": 0,
        "sobol_state_expected": int(
            job.get("sobol_state_expected", SOBOL_SAMPLES if independent else 0)
        ),
        "sobol_state_executed": 0,
        "sobol_non_adjacent_free": 0,
        "multi_joint_sobol_cf": None,
        "strict_collision_pass": None,
        "max_penetration_m": None,
        "max_penetration_normalized": None,
        "max_reset_readback_error": None,
        "object_bbox_diagonal_m": None,
        "state_records": [],
        "state_records_sha256": canonical_sha256([]),
        "issues": [issue],
        "runtime_identity": None,
    }
    if "collision_oracle" in job:
        result["collision_oracle"] = collision_oracle
    if collision_oracle == COLLISION_ORACLE_ZERO_MARGIN:
        result["execution_source_integrity"] = (
            "failed"
            if EXECUTION_SOURCE_INTEGRITY_FATAL in issue
            else "not_observed"
        )
    for field in (
        "package_root_binding_content_sha256",
        "execution_input_sha256",
        "evaluation_package_relative_path",
        "evaluation_urdf_relative_path",
        "zero_margin_oracle_sha256",
        "execution_source_hashes",
        "execution_source_hashes_sha256",
    ):
        if field in job:
            result[field] = job.get(field)
    if "external_joint_constraints" in job:
        result["native_mimic_joint_count"] = int(
            job.get("native_mimic_joint_count", job.get("mimic_joint_count", 0))
        )
        result["external_joint_constraint_count"] = int(
            job.get("external_joint_constraint_count", 0)
        )
        result["kinematic_constraint_binding"] = job.get(
            "kinematic_constraint_binding"
        )
        result["kinematic_constraint_binding_sha256"] = job.get(
            "kinematic_constraint_binding_sha256"
        )
    return result


def _derive_scale(bullet: Any, body: int, client: int) -> float:
    bounds: list[tuple[Sequence[float], Sequence[float]]] = []
    for link_index in range(-1, bullet.getNumJoints(body, physicsClientId=client)):
        shapes = bullet.getCollisionShapeData(body, link_index, physicsClientId=client)
        if not shapes:
            continue
        lower, upper = bullet.getAABB(body, link_index, physicsClientId=client)
        if not all(math.isfinite(float(value)) for value in (*lower, *upper)):
            raise ValueError("collision AABB is non-finite")
        bounds.append((lower, upper))
    if not bounds:
        raise ValueError("loaded URDF has no collision shapes")
    low = [min(float(pair[0][axis]) for pair in bounds) for axis in range(3)]
    high = [max(float(pair[1][axis]) for pair in bounds) for axis in range(3)]
    diagonal = math.sqrt(sum((b - a) ** 2 for a, b in zip(low, high)))
    if not math.isfinite(diagonal) or diagonal <= 0:
        raise ValueError("collision AABB diagonal is not positive")
    return diagonal


def _validate_joint_state_identity(joints: Sequence[Mapping[str, Any]]) -> None:
    names = [str(joint.get("name", "")) for joint in joints]
    if any(not name for name in names) or len(set(names)) != len(names):
        raise ValueError("movable joint names must be non-empty and unique")


def evaluate_job(job: Mapping[str, Any]) -> dict[str, Any]:
    """Evaluate one asset; all exceptions become a terminal fail-closed row."""

    result = _empty_record(job, "evaluation_not_completed")
    result["runtime_identity"] = runtime_identity()
    urdf = Path(str(job.get("evaluation_urdf_path", job["urdf_path"])))
    package = Path(str(job.get("evaluation_package_path", job["package"])))
    expected_hash = job.get("expected_primary_urdf_sha256")
    states: list[dict[str, Any]] = []
    try:
        collision_oracle = str(
            job.get("collision_oracle", COLLISION_ORACLE_LEGACY)
        )
        if collision_oracle not in COLLISION_ORACLES:
            raise ValueError(f"unknown collision oracle: {collision_oracle}")
        if collision_oracle == COLLISION_ORACLE_ZERO_MARGIN:
            try:
                _validate_v3_execution_source_binding(
                    job, verify_current_files=True
                )
            except BaseException as exc:
                raise RuntimeError(
                    f"{EXECUTION_SOURCE_INTEGRITY_FATAL}: {exc}"
                ) from exc
            result["execution_source_integrity"] = "verified_start"
            expected_oracle_sha256 = job.get("zero_margin_oracle_sha256")
            observed_oracle_sha256 = sha256_file(ZERO_MARGIN_ORACLE_SCRIPT)
            if expected_oracle_sha256 != observed_oracle_sha256:
                raise ValueError(
                    "zero-margin oracle SHA-256 is missing or drifted: "
                    f"{expected_oracle_sha256} != {observed_oracle_sha256}"
                )
        if not urdf.is_file():
            raise FileNotFoundError(f"primary URDF missing: {urdf}")
        if expected_hash and sha256_file(urdf) != str(expected_hash):
            raise ValueError("primary URDF hash drift before evaluation")
        inventory = collision_inventory(package, urdf)
        result.update(inventory)
        if inventory["missing_collision_mesh_references"]:
            raise FileNotFoundError(
                "missing collision mesh references: "
                + str(len(inventory["missing_collision_mesh_references"]))
            )
        if int(inventory["native_collision_elements"]) == 0:
            result["status"] = "blocked"
            result["collision_metric_status"] = BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT
            result["issues"] = ["no_native_collision_geometry"]
            result["unexecuted_state_count"] = (
                int(result["rest_state_expected"])
                + int(result["single_state_expected"])
                + int(result["sobol_state_expected"])
            )
            result["state_records_sha256"] = canonical_sha256([])
            return result

        import pybullet as bullet
        core = _core()
        joints = core.parse_urdf_joints(urdf)
        _validate_joint_state_identity(joints)
        sampling_protocol = str(job.get("sampling_protocol", SAMPLING_PROTOCOL_V1))
        plan = None
        if sampling_protocol == SAMPLING_PROTOCOL_V2:
            if "external_joint_constraints" in job:
                binding = job.get("kinematic_constraint_binding")
                binding_hash = job.get("kinematic_constraint_binding_sha256")
                if binding is None:
                    if binding_hash is not None or job.get("external_joint_constraints"):
                        raise ValueError("external joint constraints have no binding")
                elif not isinstance(binding, Mapping):
                    raise ValueError("kinematic constraint binding is not an object")
                else:
                    payload = dict(binding)
                    declared_binding_hash = payload.pop(
                        "binding_content_sha256", None
                    )
                    computed_binding_hash = canonical_sha256(payload)
                    if (
                        declared_binding_hash != computed_binding_hash
                        or binding_hash != computed_binding_hash
                    ):
                        raise ValueError("kinematic constraint binding hash mismatch")
                    if binding.get("external_joint_constraints") != job.get(
                        "external_joint_constraints"
                    ):
                        raise ValueError(
                            "kinematic constraint binding payload mismatch"
                        )
            plan = core.compile_joint_sampling_plan(
                joints,
                external_joint_constraints=job.get(
                    "external_joint_constraints"
                ),
            )
            if job.get("joint_sampling_plan_sha256") not in {
                None,
                plan["plan_sha256"],
            }:
                raise ValueError("joint sampling plan hash mismatch")
            independent_rows = list(plan["independent_joints"])
            independent_dof = int(plan["independent_dof_count"])
            range_independent = int(
                plan["range_evaluable_independent_dof_count"]
            )
            fixed_roots = int(plan["fixed_root_joint_count"])
        elif sampling_protocol == SAMPLING_PROTOCOL_V1:
            independent_rows = joints
            independent_dof = len(joints)
            range_independent = sum(
                bool(row.get("range_evaluable")) for row in joints
            )
            fixed_roots = 0
        else:
            raise ValueError(f"unknown sampling protocol: {sampling_protocol}")
        result["movable_dof_count"] = len(joints)
        result["range_evaluable_dof_count"] = sum(bool(row.get("range_evaluable")) for row in joints)
        if len(joints) != int(job["expected_movable_joints"]):
            raise ValueError(f"roster joint count mismatch: {len(joints)} != {job['expected_movable_joints']}")
        result["sampling_protocol"] = sampling_protocol
        result["independent_dof_count"] = independent_dof
        result["range_evaluable_independent_dof_count"] = range_independent
        result["mimic_joint_count"] = int(plan["mimic_joint_count"]) if plan is not None else 0
        if "external_joint_constraints" in job:
            result["native_mimic_joint_count"] = (
                int(plan["native_mimic_joint_count"]) if plan is not None else 0
            )
            result["external_joint_constraint_count"] = (
                int(plan["external_joint_constraint_count"])
                if plan is not None
                else 0
            )
        result["fixed_root_joint_count"] = fixed_roots
        result["joint_sampling_plan_sha256"] = str(plan["plan_sha256"]) if plan is not None else None
        result["single_state_expected"] = int(
            job.get("single_state_expected", SINGLE_SAMPLES * independent_dof)
        )
        result["sobol_state_expected"] = int(
            job.get("sobol_state_expected", SOBOL_SAMPLES if independent_dof else 0)
        )
        client = bullet.connect(bullet.DIRECT)
        body: int | None = None
        margin_oracle_instance: Any | None = None
        try:
            flags = int(
                bullet.URDF_USE_INERTIA_FROM_FILE
                | bullet.URDF_USE_SELF_COLLISION
                | bullet.URDF_USE_SELF_COLLISION_INCLUDE_PARENT
                | bullet.URDF_IGNORE_VISUAL_SHAPES
            )
            body = bullet.loadURDF(str(urdf), useFixedBase=True, flags=flags, physicsClientId=client)
            result["load_success"] = True
            if "collision_oracle" in job:
                result["collision_oracle"] = collision_oracle
                result["collision_load_flags"] = flags
            if collision_oracle == COLLISION_ORACLE_ZERO_MARGIN:
                margin_module = _zero_margin_oracle()
                margin_oracle_instance = margin_module.ZeroMarginProxyOracle.build(
                    bullet,
                    body,
                    client,
                    urdf,
                    package,
                    _resolve_mesh,
                )
                result.update(margin_oracle_instance.receipt())
                result["zero_margin_oracle_sha256"] = sha256_file(
                    ZERO_MARGIN_ORACLE_SCRIPT
                )
            simulator_by_name: dict[str, int] = {}
            for index in range(bullet.getNumJoints(body, physicsClientId=client)):
                info = bullet.getJointInfo(body, index, physicsClientId=client)
                name = info[1].decode("utf-8") if isinstance(info[1], bytes) else str(info[1])
                simulator_by_name[name] = index
            if any(str(row["name"]) not in simulator_by_name for row in joints):
                missing = [str(row["name"]) for row in joints if str(row["name"]) not in simulator_by_name]
                raise ValueError(f"simulator joint mapping missing: {missing[:5]}")
            joint_indices = [simulator_by_name[str(row["name"])] for row in joints]
            for index in joint_indices:
                bullet.setJointMotorControl2(body, index, bullet.VELOCITY_CONTROL, targetVelocity=0.0, force=0.0, physicsClientId=client)
            direct_pairs = core._direct_parent_pairs(bullet, body, client)
            rest_values = (
                [0.0] * len(joints)
                if plan is None
                else core.expand_joint_values(plan, [0.0] * independent_dof)
            )
            if any(float(value) != 0.0 for value in rest_values):
                # The simulator loads joints at zero, while a mimic offset can
                # make the protocol's expanded rest state nonzero.
                if margin_oracle_instance is not None:
                    core._reset_and_readback(
                        bullet, body, client, joint_indices, rest_values
                    )
                else:
                    core._reset_and_observe(
                        bullet,
                        body,
                        client,
                        joint_indices,
                        rest_values,
                        direct_pairs,
                    )
            scale = (
                margin_oracle_instance.object_bbox_diagonal()
                if margin_oracle_instance is not None
                else _derive_scale(bullet, body, client)
            )
            result["object_bbox_diagonal_m"] = scale
            result["normalization_configuration"] = "expanded_rest"
            if margin_oracle_instance is not None:
                result["normalization_collision_geometry"] = (
                    "per_collision_element_numerical_zero_margin_proxy_union_aabb"
                )
            result["normalization_joint_values_sha256"] = core.canonical_sha256(
                rest_values
            )

            def observe(values: list[float], phase: str, sample_index: int, joint_name: str | None = None) -> dict[str, Any]:
                if margin_oracle_instance is not None:
                    readback = core._reset_and_readback(
                        bullet, body, client, joint_indices, values
                    )
                    # One oracle snapshot refreshes both the raw body and all
                    # calibrated proxies before either contact set is queried.
                    collision = margin_oracle_instance.observe(direct_pairs)
                else:
                    collision, readback = core._reset_and_observe(
                        bullet,
                        body,
                        client,
                        joint_indices,
                        values,
                        direct_pairs,
                    )
                metric_key = "all_pair_max_penetration_m" if phase == "rest" else "non_adjacent_max_penetration_m"
                state = {
                    "schema_version": _state_schema_for_job(job),
                    "dataset": job["dataset"], "dataset_id": job["dataset_id"], "order": job["order"],
                    "category": job.get("category", ""), "protocol_id": job["protocol_id"],
                    "input_identity_sha256": job["input_identity_sha256"],
                    "sampling_protocol": sampling_protocol,
                    "joint_sampling_plan_sha256": result.get(
                        "joint_sampling_plan_sha256"
                    ),
                    "phase": phase, "sample_index": sample_index, "joint_name": joint_name,
                    "joint_values_sha256": core.canonical_sha256(values),
                    "reset_readback_max_abs_error": float(readback),
                    "metric_max_penetration_m": float(collision[metric_key]),
                    **collision,
                }
                if "external_joint_constraints" in job:
                    state["kinematic_constraint_binding_sha256"] = job.get(
                        "kinematic_constraint_binding_sha256"
                    )
                if "collision_oracle" in job:
                    state["collision_oracle"] = collision_oracle
                if collision_oracle == COLLISION_ORACLE_ZERO_MARGIN:
                    state["zero_margin_oracle_sha256"] = job[
                        "zero_margin_oracle_sha256"
                    ]
                    state["execution_source_hashes_sha256"] = job[
                        "execution_source_hashes_sha256"
                    ]
                states.append(state)
                return state

            rest = observe(rest_values, "rest", 0)
            result["rest_state_executed"] = 1
            result["rest_all_pair_cf"] = int(rest["all_pair_illegal_penetration_count"]) == 0
            result["rest_non_adjacent_cf"] = int(rest["non_adjacent_illegal_penetration_count"]) == 0
            result["rest_non_adjacent_free"] = int(result["rest_non_adjacent_cf"])
            for position, joint in enumerate(independent_rows):
                if not bool(
                    joint.get("sampling_range_evaluable", joint.get("range_evaluable"))
                ):
                    continue
                joint_free = True
                for sample_index, value in enumerate(core.single_joint_values(joint)):
                    if plan is None:
                        values = [0.0] * len(joints)
                        values[position] = float(value)
                    else:
                        independent_values = [0.0] * independent_dof
                        independent_values[position] = float(value)
                        values = core.expand_joint_values(plan, independent_values)
                    state = observe(values, "single_joint_sweep", sample_index, str(joint["name"]))
                    result["single_state_executed"] += 1
                    free = int(state["non_adjacent_illegal_penetration_count"]) == 0
                    result["single_non_adjacent_free"] += int(free)
                    joint_free = joint_free and free
                result["joint_single_sweep_cf_passed"] += int(joint_free)
            if independent_rows and all(
                bool(
                    joint.get("sampling_range_evaluable", joint.get("range_evaluable"))
                )
                for joint in independent_rows
            ):
                for sample_index, sampled in enumerate(
                    core.sobol_joint_values(independent_rows, seed=SOBOL_SEED)
                ):
                    values = (
                        [float(value) for value in sampled]
                        if plan is None
                        else core.expand_joint_values(
                            plan, [float(value) for value in sampled]
                        )
                    )
                    state = observe(values, "multi_joint_sobol", sample_index)
                    result["sobol_state_executed"] += 1
                    result["sobol_non_adjacent_free"] += int(int(state["non_adjacent_illegal_penetration_count"]) == 0)
            result["measurement_complete"] = bool(
                result["range_evaluable_independent_dof_count"]
                == result["independent_dof_count"]
                and result["rest_state_executed"] == result["rest_state_expected"]
                and result["single_state_executed"] == result["single_state_expected"]
                and result["sobol_state_executed"] == result["sobol_state_expected"]
            )
            # A zero-DoF asset has no executable Sobol state.  The protocol
            # retains it in the asset-level Sobol/Strict denominators and
            # requires those results to fail closed; the empty single-joint
            # plan remains the historical vacuous single-sweep convention.
            if independent_dof == 0:
                result["single_joint_sweep_cf"] = bool(
                    result["single_state_executed"] == result["single_state_expected"]
                    and result["single_non_adjacent_free"] == result["single_state_expected"]
                )
                result["multi_joint_sobol_cf"] = False
                result["strict_collision_pass"] = False
                result["joint_single_sweep_cf_passed"] = 0
                result["zero_dof_fail_closed"] = True
            else:
                result["single_joint_sweep_cf"] = bool(result["single_state_executed"] == result["single_state_expected"] and result["single_non_adjacent_free"] == result["single_state_expected"])
                result["multi_joint_sobol_cf"] = bool(result["measurement_complete"] and result["sobol_non_adjacent_free"] == result["sobol_state_expected"])
                result["strict_collision_pass"] = bool(result["measurement_complete"] and result["rest_non_adjacent_cf"] and result["single_joint_sweep_cf"] and result["multi_joint_sobol_cf"])
            result["collision_metric_status"] = MEASURED_COLLISION_GEOMETRY
            result["status"] = "completed" if result["measurement_complete"] else "error"
            result["issues"] = [] if result["measurement_complete"] else ["measurement_incomplete"]
        finally:
            try:
                if margin_oracle_instance is not None:
                    margin_oracle_instance.close()
            finally:
                if body is not None:
                    bullet.removeBody(body, physicsClientId=client)
                bullet.disconnect(client)
        if states:
            result["max_penetration_m"] = max(float(state["metric_max_penetration_m"]) for state in states)
            result["max_penetration_normalized"] = result["max_penetration_m"] / float(result["object_bbox_diagonal_m"])
            result["max_reset_readback_error"] = max(float(state["reset_readback_max_abs_error"]) for state in states)
            if collision_oracle == COLLISION_ORACLE_ZERO_MARGIN:
                result["raw_max_penetration_m"] = max(
                    float(
                        state[
                            "raw_all_pair_max_penetration_m"
                            if state["phase"] == "rest"
                            else "raw_non_adjacent_max_penetration_m"
                        ]
                    )
                    for state in states
                )
    except BaseException as exc:  # child boundary is fail-closed
        result["status"] = "error"
        result["issues"] = [f"{type(exc).__name__}: {exc}"]
        if result.get("native_collision_elements", 0):
            result["collision_metric_status"] = "N/E" if not states else "partial"
        result["measurement_complete"] = False
        # Preserve a phase-level rest result when that state was already
        # observed successfully before a later sweep failed.  Unexecuted or
        # incomplete sweep phases remain fail-closed; this keeps the metric
        # definitions compositional without turning a partial asset into a
        # strict pass.
        if int(result.get("rest_state_executed", 0) or 0) < int(result.get("rest_state_expected", 1) or 1):
            result["rest_all_pair_cf"] = False
            result["rest_non_adjacent_cf"] = False
        result["single_joint_sweep_cf"] = False
        result["multi_joint_sobol_cf"] = False
        result["strict_collision_pass"] = False
    finally:
        if expected_hash and urdf.is_file() and sha256_file(urdf) != str(expected_hash):
            result["status"] = "error"
            result["issues"] = ["primary URDF hash drift after evaluation"]
        if (
            job.get("collision_oracle") == COLLISION_ORACLE_ZERO_MARGIN
        ):
            try:
                _validate_v3_execution_source_binding(
                    job, verify_current_files=True
                )
            except BaseException as exc:
                result["status"] = "error"
                result["measurement_complete"] = False
                result["execution_source_integrity"] = "failed"
                result["issues"] = [
                    f"{EXECUTION_SOURCE_INTEGRITY_FATAL}: "
                    f"{type(exc).__name__}: {exc}"
                ]
            else:
                result["execution_source_integrity"] = (
                    "failed"
                    if any(
                        EXECUTION_SOURCE_INTEGRITY_FATAL in str(issue)
                        for issue in (result.get("issues") or [])
                    )
                    else "verified_start_and_end"
                )
    result["state_records"] = states
    result["state_records_sha256"] = canonical_sha256(states)
    result["state_records_count"] = len(states)
    result.setdefault("unexecuted_state_count", max(
        0,
        int(result.get("rest_state_expected", 0))
        + int(result.get("single_state_expected", 0))
        + int(result.get("sobol_state_expected", 0))
        - len(states),
    ))
    return result


def _child_main(job_path: Path, result_path: Path) -> int:
    try:
        job = _load_json(job_path)
        result = evaluate_job(job)
    except BaseException as exc:
        try:
            job = _load_json(job_path)
            result = _empty_record(job, f"child_exception: {type(exc).__name__}: {exc}")
        except BaseException:
            return 1
    atomic_json(result_path, result)
    return 0


def _kill_group(process: subprocess.Popen[str]) -> None:
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except (OSError, AttributeError):
        process.terminate()
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except (OSError, AttributeError):
            process.kill()
        process.wait(timeout=5.0)


def _execute_child(job: Mapping[str, Any], output: Path, timeout_seconds: float) -> dict[str, Any]:
    index = int(job["order"])
    jobs_dir = output / "child_jobs"; children_dir = output / "children"; logs_dir = output / "child_logs"
    for directory in (jobs_dir, children_dir, logs_dir): directory.mkdir(parents=True, exist_ok=True)
    job_path = jobs_dir / f"{index:06d}.json"; result_path = children_dir / f"{index:06d}.json"; log_path = logs_dir / f"{index:06d}.log"
    atomic_json(job_path, dict(job))
    env = dict(os.environ)
    env.update({"OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "NUMEXPR_NUM_THREADS": "1", "PYTHONDONTWRITEBYTECODE": "1"})
    command = [sys.executable, str(SCRIPT), "--child", "--job", str(job_path), "--result", str(result_path)]
    result_path.unlink(missing_ok=True)
    with log_path.open("wb") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, env=env, start_new_session=True)
        try:
            code = process.wait(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            _kill_group(process); return _empty_record(job, f"asset_timeout_after_{timeout_seconds:g}_seconds", status="timeout")
    if code != 0 or not result_path.is_file():
        return _empty_record(job, f"child_exit_{code}")
    try:
        result = _load_json(result_path)
        _validate_result_binding(result, job)
        return result
    except BaseException as exc:
        return _empty_record(job, f"child_result_invalid: {type(exc).__name__}: {exc}")


def _artifact_manifest(output: Path) -> dict[str, Any]:
    entries = []
    for path in sorted(output.rglob("*")):
        if not path.is_file() or path.name == "artifact_manifest.json" or path.is_symlink():
            continue
        entries.append({"path": path.relative_to(output).as_posix(), "size": path.stat().st_size, "sha256": sha256_file(path)})
    result = {"schema_version": "table123_artifact_manifest_v1", "artifacts": entries}
    result["artifact_manifest_content_sha256"] = _self_hash(result, "artifact_manifest_content_sha256")
    return result


def _metric_fraction(passed: int, denominator: int) -> dict[str, Any]:
    return {"passed": int(passed), "denominator": int(denominator), "rate": (passed / denominator if denominator else None)}


def aggregate_records(records: Sequence[Mapping[str, Any]], n_eval: int, j_eval: int) -> dict[str, Any]:
    if len(records) != n_eval:
        raise ValueError(f"record count mismatch: {len(records)} != {n_eval}")
    status_counts: dict[str, int] = defaultdict(int)
    expected = {phase: 0 for phase in ("rest", "single", "sobol")}; executed = {phase: 0 for phase in expected}; free = {phase: 0 for phase in expected}
    pass_keys = ("rest_all_pair_cf", "rest_non_adjacent_cf", "single_joint_sweep_cf", "multi_joint_sobol_cf", "strict_collision_pass")
    passed = {key: 0 for key in pass_keys}; collision_assets = 0; max_values: list[float] = []; measured_assets = 0
    for index, record in enumerate(records):
        status_counts[str(record.get("status", "error"))] += 1
        dof = int(record.get("movable_dof_count", record.get("expected_movable_joints", 0)) or 0)
        for phase in expected:
            expected[phase] += int(record.get(f"{phase}_state_expected", 1 if phase == "rest" else SINGLE_SAMPLES * dof if phase == "single" else SOBOL_SAMPLES if dof else 0))
            executed[phase] += int(record.get(f"{phase}_state_executed", 0) or 0)
            free[phase] += int(record.get("rest_non_adjacent_free" if phase == "rest" else f"{phase}_non_adjacent_free", 0) or 0)
        native = int(record.get("native_collision_elements", 0) or 0) > 0 and str(record.get("collision_metric_status", "")).upper() not in {"N/E", "NE", "BLOCKED"}
        collision_assets += int(native)
        zero_dof_native = dof == 0 and native
        for key in pass_keys:
            value = bool(record.get(key, False))
            # Keep aggregate semantics fail-closed even when a legacy child
            # receipt predates the zero-DoF evaluator fix.
            if zero_dof_native and key in {"multi_joint_sobol_cf", "strict_collision_pass"}:
                value = False
            passed[key] += int(value)
        raw = record.get("max_penetration_normalized")
        if raw is not None:
            try:
                value = float(raw)
                if math.isfinite(value) and value >= 0: max_values.append(value)
            except (TypeError, ValueError):
                pass
        measured_assets += int(bool(record.get("measurement_complete")) and native)
    total_expected = sum(expected.values()); total_executed = sum(executed.values()); total_free = sum(free.values())
    metrics: dict[str, Any] = {}
    if collision_assets == 0:
        ne = {"status": "N/E", "reason": "no_native_collision_geometry", "numerator": None, "denominator": None}
        metrics.update({key: dict(ne) for key in pass_keys})
        metrics["collision_state_rate"] = dict(ne); metrics["collision_free_range"] = dict(ne)
        metrics["max_penetration"] = {"status": "N/E", "reason": "empty_collision_shape_union", "maximum_observed_normalized": None, "observed_assets": 0, "measured_assets": 0, "denominator": n_eval}
    else:
        metrics.update({key: _metric_fraction(passed[key], n_eval) for key in pass_keys})
        metrics["collision_state_rate"] = {"collision_states": total_expected - total_free, "denominator": total_expected, "rate": ((total_expected - total_free) / total_expected if total_expected else None), "executed_states": total_executed, "unexecuted_states": total_expected - total_executed}
        metrics["collision_free_range"] = {"passed_states": free["single"], "denominator": expected["single"], "rate": (free["single"] / expected["single"] if expected["single"] else None)}
        metrics["max_penetration"] = {"status": "N/E" if not max_values else "COMPLETE" if measured_assets == n_eval else "PARTIAL", "maximum_observed_normalized": max(max_values) if max_values else None, "observed_assets": len(max_values), "measured_assets": measured_assets, "denominator": n_eval}
    metrics["aor"] = {"status": "N/E", "reason": "exact_overlap_volume_backend_not_registered"}
    if collision_assets == 0 and records and all(str(row.get("status")) == "blocked" for row in records):
        overall_status = "BLOCKED"
    elif all(str(row.get("status")) == "completed" for row in records):
        overall_status = "COMPLETE"
    else:
        overall_status = "COMPLETE_WITH_RETAINED_FAILURES"
    return {
        "schema_version": SUMMARY_SCHEMA, "n_eval": n_eval, "j_eval": j_eval,
        "status": overall_status,
        "status_counts": dict(sorted(status_counts.items())), "collision_geometry_assets": collision_assets,
        "expected_states": expected, "executed_states": executed, "metrics": metrics,
        "state_records_expected": total_expected, "state_records_executed": total_executed,
        "claim_boundary": {"continuous_collision_detection": "not_run", "semantic_joint_correctness": "not_evaluated", "physical_dynamics_validity": "not_evaluated", "empty_contact_query_is_not_a_pass": True},
    }


def report_text(summary: Mapping[str, Any], dataset: str) -> str:
    m = summary["metrics"]
    def fmt(key: str) -> str:
        value = m.get(key, {})
        if str(value.get("status", "")).upper() == "N/E": return "N/E"
        if key == "max_penetration": return str(value.get("maximum_observed_normalized"))
        num = value.get("passed", value.get("collision_states", value.get("passed_states"))); den = value.get("denominator")
        return f"{num} / {den}" if num is not None and den is not None else "N/E"
    lines = [f"# {DISPLAY_NAMES.get(dataset, dataset)}: Table 4 full-release evaluation", "", f"Status: **{summary.get('status')}**", "", f"N_eval: {summary['n_eval']}  \\  J_eval: {summary['j_eval']}", "", "| Metric | Result |", "|---|---:|"]
    for key, label in (("rest_all_pair_cf", "Rest All-pair CF"), ("rest_non_adjacent_cf", "Rest Non-adjacent CF"), ("single_joint_sweep_cf", "Single-joint Sweep CF"), ("multi_joint_sobol_cf", "Multi-joint Sobol CF"), ("collision_state_rate", "Collision-state Rate"), ("aor", "AOR"), ("max_penetration", "Max Penetration"), ("collision_free_range", "Collision-free Range"), ("strict_collision_pass", "Strict Collision Pass")):
        lines.append(f"| {label} | {fmt(key)} |")
    lines += ["", "Collision-dependent metrics are N/E when native collision geometry is absent. Unexecuted states remain fail-closed in denominators; AOR is N/E because no exact overlap-volume backend is registered.", ""]
    return "\n".join(lines)


def _source_bindings_compatible(
    previous: Any,
    requested: Sequence[Mapping[str, Any]],
) -> bool:
    """Check resume inputs against a sealed source-binding superset.

    Source-binding utilities may add provenance after an evaluation has
    completed (for example an archive receipt or a cohort manifest).  Those
    additions must not invalidate a resume, while a changed path or declared
    digest must still fail closed.  Matching by path also accommodates a
    historical alias such as ``Infinigen-Sim`` versus ``extracted_root``.
    """

    if previous is None:
        return not requested
    if not isinstance(previous, list):
        return False
    sealed = [item for item in previous if isinstance(item, Mapping)]
    for current in requested:
        if not isinstance(current, Mapping) or not current.get("path"):
            return False
        current_name = str(current.get("name", ""))
        current_path = str(current.get("path", ""))
        same_name = [item for item in sealed if str(item.get("name", "")) == current_name]
        # A binding name is an identity when present: silently retargeting it
        # is unsafe even if another historical alias happens to share a path.
        if same_name and not any(str(item.get("path", "")) == current_path for item in same_name):
            return False
        candidates = [item for item in same_name if str(item.get("path", "")) == current_path]
        if not candidates:
            candidates = [item for item in sealed if str(item.get("path", "")) == current_path]
        if not candidates:
            return False
        sealed_item = candidates[0]
        for field in ("sha256", "bytes"):
            expected = current.get(field)
            observed = sealed_item.get(field)
            if expected is not None and observed is not None and str(expected) != str(observed):
                return False
    return True


def run_dataset(
    roster: Path,
    output: Path,
    *,
    workers: int = 32,
    timeout_seconds: float = 300.0,
    resume: bool = False,
    dataset: str | None = None,
    parts_zip: Path | None = None,
    source_root: Path | None = None,
    sampling_protocol: str = SAMPLING_PROTOCOL_V1,
    collision_oracle: str | None = None,
) -> Path:
    roster = Path(roster).resolve(strict=True); output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=True)
    if dataset is None: dataset = roster.parent.name
    # Low-level callers may provide a fixture/private roster; the orchestrator
    # validates the canonical eight slugs before invoking this function.
    manifest = _load_json(roster)
    jobs = build_jobs(
        manifest,
        dataset,
        sampling_protocol=sampling_protocol,
        collision_oracle=collision_oracle,
    )
    n_eval = len(jobs)
    j_eval = sum(int(job["expected_movable_joints"]) for job in jobs)
    display_name = DISPLAY_NAMES.get(dataset, str(manifest.get("dataset") or dataset))
    protocol_path = EXP_ROOT / "URDF-Sim-Ready-Automatic-Evaluation.md"
    # Carry the frozen roster's source map into the run receipt.  Explicit
    # command-line bindings below augment (and never replace) that map.
    source_bindings: list[dict[str, Any]] = [
        dict(binding)
        for binding in (manifest.get("source_bindings", []) or [])
        if isinstance(binding, Mapping)
    ]
    if parts_zip is not None:
        archive = Path(parts_zip).resolve(strict=True)
        if not archive.is_file():
            raise ValueError(f"parts_zip is not a file: {archive}")
        source_bindings.append({"name": "parts_zip", "path": str(archive), "sha256": sha256_file(archive), "bytes": archive.stat().st_size})
    if source_root is not None:
        source = Path(source_root).resolve(strict=True)
        if not source.is_dir():
            raise ValueError(f"source_root is not a directory: {source}")
        source_bindings.append({"name": "source_root", "path": str(source)})
    deduped_bindings: list[dict[str, Any]] = []
    seen_bindings: set[tuple[str, str]] = set()
    for binding in source_bindings:
        key = (str(binding.get("name", "")), str(binding.get("path", "")))
        if key in seen_bindings:
            continue
        seen_bindings.add(key)
        deduped_bindings.append(binding)
    source_bindings = deduped_bindings
    zero_margin = collision_oracle == COLLISION_ORACLE_ZERO_MARGIN
    frozen_zero_margin_oracle_sha256 = None
    frozen_execution_source_hashes: dict[str, str] | None = None
    frozen_execution_source_hashes_sha256 = None
    if zero_margin:
        frozen_hashes = {
            str(job.get("zero_margin_oracle_sha256")) for job in jobs
        }
        if len(frozen_hashes) != 1 or "None" in frozen_hashes:
            raise ValueError("v3 jobs do not share one zero-margin oracle SHA-256")
        frozen_zero_margin_oracle_sha256 = next(iter(frozen_hashes))
        source_maps = {
            canonical_sha256(job.get("execution_source_hashes"))
            for job in jobs
        }
        source_map_digests = {
            str(job.get("execution_source_hashes_sha256")) for job in jobs
        }
        if len(source_maps) != 1 or len(source_map_digests) != 1:
            raise ValueError("v3 jobs do not share one execution source hash map")
        frozen_execution_source_hashes = dict(
            jobs[0]["execution_source_hashes"]
        )
        frozen_execution_source_hashes_sha256 = next(
            iter(source_map_digests)
        )
        _validate_v3_execution_source_binding(
            jobs[0], verify_current_files=True
        )
    run_schema = (
        SCHEMA_VERSION_V3
        if zero_margin
        else (
            SCHEMA_VERSION_V2
            if sampling_protocol == SAMPLING_PROTOCOL_V2
            else SCHEMA_VERSION
        )
    )
    summary_schema = (
        SUMMARY_SCHEMA_V3
        if zero_margin
        else (
            SUMMARY_SCHEMA_V2
            if sampling_protocol == SAMPLING_PROTOCOL_V2
            else SUMMARY_SCHEMA
        )
    )
    checkpoint_schema = (
        "table4_checkpoint_v3"
        if zero_margin
        else (
            "table4_checkpoint_v2"
            if sampling_protocol == SAMPLING_PROTOCOL_V2
            else "table4_checkpoint_v1"
        )
    )
    run_manifest = {"schema_version": run_schema, "dataset": display_name, "dataset_slug": dataset, "roster": str(roster), "roster_sha256": sha256_file(roster), "N_eval": n_eval, "J_eval": j_eval, "jobs_sha256": canonical_sha256(jobs), "sampling_protocol": sampling_protocol, "independent_J_eval": sum(int(job["independent_dof_count"]) for job in jobs), "mimic_joint_count": sum(int(job["mimic_joint_count"]) for job in jobs), "fixed_root_joint_count": sum(int(job.get("fixed_root_joint_count", 0)) for job in jobs), "protocol_snapshot": str(protocol_path), "protocol_snapshot_sha256": sha256_file(protocol_path), "source_bindings": source_bindings, "execution": {"workers": workers, "timeout_seconds": timeout_seconds, "fresh_interpreter_per_asset": True}, "runner_sha256": sha256_file(SCRIPT), "collision_core_sha256": sha256_file(CORE_SCRIPT), "started_at": utc_now()}
    if collision_oracle is not None:
        run_manifest["collision_oracle"] = collision_oracle
    if zero_margin:
        run_manifest["zero_margin_oracle_sha256"] = (
            frozen_zero_margin_oracle_sha256
        )
        run_manifest["execution_source_hashes"] = (
            frozen_execution_source_hashes
        )
        run_manifest["execution_source_hashes_sha256"] = (
            frozen_execution_source_hashes_sha256
        )
    run_manifest["manifest_content_sha256"] = _self_hash(run_manifest, "manifest_content_sha256")
    manifest_path = output / "manifest.json"
    records_path = output / "records.jsonl"
    records: dict[str, dict[str, Any]] = {}
    if resume and manifest_path.is_file():
        old = _load_json(manifest_path)
        if old.get("manifest_content_sha256") != _self_hash(old, "manifest_content_sha256"): raise ValueError("run manifest self-hash mismatch")
        for key in (
            "roster_sha256",
            "jobs_sha256",
            "N_eval",
            "J_eval",
            "sampling_protocol",
            "independent_J_eval",
            "mimic_joint_count",
            "fixed_root_joint_count",
            "collision_oracle",
            "zero_margin_oracle_sha256",
            "execution_source_hashes",
            "execution_source_hashes_sha256",
        ):
            if old.get(key) != run_manifest.get(key): raise ValueError(f"resume configuration mismatch: {key}")
        if not _source_bindings_compatible(old.get("source_bindings"), source_bindings):
            raise ValueError("resume configuration mismatch: source_bindings")
        if old.get("protocol_snapshot_sha256") != run_manifest.get("protocol_snapshot_sha256"):
            # The published protocol document may be updated after a run is
            # sealed.  A complete run is still bound to its recorded protocol
            # hash and can be resumed/repacked without rerunning assets; an
            # interrupted run must fail closed rather than mixing protocols.
            checkpoint_path = output / "checkpoint.json"
            checkpoint_state = ""
            checkpoint_records = None
            if checkpoint_path.is_file():
                try:
                    checkpoint = _load_json(checkpoint_path)
                    checkpoint_state = str(checkpoint.get("state", checkpoint.get("status", ""))).lower()
                    checkpoint_records = int(checkpoint.get("records", -1))
                except (OSError, TypeError, ValueError, json.JSONDecodeError):
                    checkpoint_state = ""
            if checkpoint_state not in {"complete", "completed"} or checkpoint_records != n_eval:
                raise ValueError("resume configuration mismatch: protocol_snapshot_sha256")
            if not records_path.is_file():
                raise ValueError("cannot reuse complete run without records.jsonl")
        # Preserve the original manifest, including its frozen protocol hash,
        # when resuming a sealed run after the source document was edited.
        run_manifest = old
        duplicate_record_ids: set[str] = set()
        if records_path.is_file():
            for line in records_path.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    row = json.loads(line)
                    if not isinstance(row, dict) or "dataset_id" not in row:
                        continue
                    asset_id = str(row["dataset_id"])
                    if asset_id in records:
                        duplicate_record_ids.add(asset_id)
                    records[asset_id] = row
        # Reattach state payloads from the durable stream (or child receipts)
        # before rebuilding summaries.  This keeps resume lossless even though
        # the compact records JSONL intentionally omits repeated state rows.
        state_path = output / "state_records.jsonl"
        if state_path.is_file():
            by_state_id: dict[str, list[dict[str, Any]]] = defaultdict(list)
            for line in state_path.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    state = json.loads(line)
                    by_state_id.setdefault(str(state.get("dataset_id")), []).append(state)
            for asset_id, row in records.items():
                row["state_records"] = by_state_id.get(asset_id, [])
        for job in jobs:
            asset_id = job["dataset_id"]
            if asset_id in records and "state_records" not in records[asset_id]:
                child_path = output / "children" / f"{int(job['order']):06d}.json"
                if child_path.is_file():
                    try:
                        records[asset_id]["state_records"] = list(_load_json(child_path).get("state_records", []))
                    except Exception:
                        records[asset_id]["state_records"] = []
        frozen_jobs = {str(job["dataset_id"]): job for job in jobs}
        for asset_id in list(records):
            job = frozen_jobs.get(asset_id)
            if job is None or asset_id in duplicate_record_ids:
                records.pop(asset_id, None)
                continue
            try:
                _validate_result_binding(records[asset_id], job)
            except (TypeError, ValueError):
                records.pop(asset_id, None)
        # Recover child receipts produced before a parent interruption.  A
        # child file is accepted only when its identity matches the frozen job;
        # stale or malformed files remain pending and are safely rerun.
        children_dir = output / "children"
        if children_dir.is_dir():
            for job in jobs:
                asset_id = job["dataset_id"]
                if asset_id in records:
                    continue
                child_path = children_dir / f"{int(job['order']):06d}.json"
                if not child_path.is_file():
                    continue
                try:
                    child = _load_json(child_path)
                    _validate_result_binding(child, job)
                    records[asset_id] = child
                except (OSError, json.JSONDecodeError, TypeError, ValueError):
                    continue
    elif not resume and (manifest_path.exists() or records_path.exists()):
        raise ValueError("output already contains a run; use --resume")
    atomic_json(manifest_path, run_manifest)
    pending = [job for job in jobs if job["dataset_id"] not in records]
    started = time.monotonic()
    def persist_running() -> None:
        ordered_partial = [records[j["dataset_id"]] for j in jobs if j["dataset_id"] in records]
        compact_partial = [{k: v for k, v in row.items() if k != "state_records"} for row in ordered_partial]
        atomic_jsonl(records_path, compact_partial)
        partial_states = [state for row in ordered_partial for state in row.get("state_records", [])]
        atomic_jsonl(output / "state_records.jsonl", partial_states)
        checkpoint_partial = {
            "schema_version": checkpoint_schema,
            "state": "running",
            "records": len(ordered_partial),
            "records_sha256": sha256_file(records_path),
            "manifest_content_sha256": run_manifest["manifest_content_sha256"],
            "updated_at": utc_now(),
        }
        checkpoint_partial["checkpoint_content_sha256"] = _self_hash(checkpoint_partial, "checkpoint_content_sha256")
        atomic_json(output / "checkpoint.json", checkpoint_partial)
        atomic_json(output / "progress.json", {"status": "RUNNING", "completed_assets": len(records), "total_assets": n_eval, "state_records": len(partial_states), "elapsed_seconds": time.monotonic() - started})

    try:
        with ThreadPoolExecutor(max_workers=max(1, workers)) as pool:
            futures = {pool.submit(_execute_child, job, output, timeout_seconds): job for job in pending}
            for number, future in enumerate(as_completed(futures), 1):
                job = futures[future]
                try: row = future.result()
                except BaseException as exc: row = _empty_record(job, f"parent_executor_exception: {type(exc).__name__}: {exc}")
                try:
                    _validate_result_binding(row, job)
                except (TypeError, ValueError) as exc:
                    row = _empty_record(
                        job,
                        f"parent_child_binding_invalid: {type(exc).__name__}: {exc}",
                    )
                records[str(job["dataset_id"])] = row
                if number % 16 == 0 or number == len(pending):
                    persist_running()
    except BaseException:
        persist_running()
        raise
    ordered = [records[job["dataset_id"]] for job in jobs]
    for job, row in zip(jobs, ordered):
        _validate_result_binding(row, job)
    if zero_margin:
        sealed_oracle_sha256 = run_manifest.get("zero_margin_oracle_sha256")
        if sealed_oracle_sha256 != frozen_zero_margin_oracle_sha256:
            raise ValueError("run manifest zero-margin oracle SHA-256 drift")
        if sha256_file(ZERO_MARGIN_ORACLE_SCRIPT) != sealed_oracle_sha256:
            raise ValueError("zero-margin oracle changed during the run")
        if run_manifest.get("execution_source_hashes") != (
            frozen_execution_source_hashes
        ):
            raise ValueError("run manifest execution source hash map drift")
        if run_manifest.get("execution_source_hashes_sha256") != (
            frozen_execution_source_hashes_sha256
        ):
            raise ValueError("run manifest execution source hash digest drift")
        _validate_v3_execution_source_binding(
            run_manifest, verify_current_files=True
        )
    # Keep state payloads in one deterministic stream; asset records remain
    # compact so the receipt can be inspected without loading every state.
    compact = [{k: v for k, v in row.items() if k != "state_records"} for row in ordered]
    all_states = [state for row in ordered for state in row.get("state_records", [])]
    atomic_jsonl(records_path, compact); atomic_jsonl(output / "asset_records.jsonl", compact); atomic_jsonl(output / "state_records.jsonl", all_states)
    # Child receipts are useful provenance, but embedding every state in each
    # child would duplicate the canonical state stream.  Keep the digest and
    # counters in the child artifact and publish the states once above.
    children_dir = output / "children"
    if children_dir.is_dir():
        for child_path in children_dir.glob("*.json"):
            try:
                child = _load_json(child_path)
                if isinstance(child, dict) and "state_records" in child:
                    child.pop("state_records", None)
                    atomic_json(child_path, child)
            except (OSError, json.JSONDecodeError, ValueError):
                continue
    records_hash = sha256_file(records_path)
    summary = aggregate_records(ordered, n_eval, j_eval); summary.update({"schema_version": summary_schema, "dataset": display_name, "dataset_slug": dataset, "sampling_protocol": sampling_protocol, "independent_j_eval": sum(int(job["independent_dof_count"]) for job in jobs), "mimic_joint_count": sum(int(job["mimic_joint_count"]) for job in jobs), "fixed_root_joint_count": sum(int(job.get("fixed_root_joint_count", 0)) for job in jobs), "manifest_content_sha256": run_manifest["manifest_content_sha256"], "records_sha256": records_hash, "source_bindings": run_manifest.get("source_bindings", []), "completed_at": utc_now()})
    if collision_oracle is not None:
        summary["collision_oracle"] = collision_oracle
    if zero_margin:
        summary["zero_margin_oracle_sha256"] = run_manifest[
            "zero_margin_oracle_sha256"
        ]
        summary["execution_source_hashes"] = run_manifest[
            "execution_source_hashes"
        ]
        summary["execution_source_hashes_sha256"] = run_manifest[
            "execution_source_hashes_sha256"
        ]
    summary["summary_content_sha256"] = _self_hash(summary, "summary_content_sha256")
    atomic_json(output / "summary.json", summary); atomic_json(output / "asset_records.json", compact); _atomic_write(output / "summary.md", report_text(summary, dataset));
    checkpoint = {"schema_version": checkpoint_schema, "state": "complete", "records": n_eval, "records_sha256": records_hash, "summary_sha256": sha256_file(output / "summary.json"), "manifest_content_sha256": run_manifest["manifest_content_sha256"], "completed_at": utc_now()}; checkpoint["checkpoint_content_sha256"] = _self_hash(checkpoint, "checkpoint_content_sha256"); atomic_json(output / "checkpoint.json", checkpoint)
    atomic_json(output / "progress.json", {"status": "COMPLETE", "completed_assets": n_eval, "total_assets": n_eval, "state_records": len(all_states)})
    atomic_json(output / "artifact_manifest.json", _artifact_manifest(output))
    return output


def build_combined_receipt(outputs: Mapping[str, Path | str], root: Path) -> dict[str, Any]:
    entries = []
    sampling_protocols: set[str] = set()
    collision_oracles: set[str] = set()
    zero_margin_oracle_hashes: set[str] = set()
    explicit_collision_oracle = False
    ordered_slugs = [slug for slug in DATASETS if slug in outputs]
    ordered_slugs.extend(slug for slug in outputs if slug not in ordered_slugs)
    for slug in ordered_slugs:
        path = Path(outputs[slug]).resolve(); summary = _load_json(path / "summary.json")
        if summary.get("summary_content_sha256") != _self_hash(
            summary, "summary_content_sha256"
        ):
            raise ValueError(f"summary self-hash mismatch for {slug}")
        display = DISPLAY_NAMES.get(slug, str(summary.get("dataset") or slug))
        sampling_protocol = str(
            summary.get("sampling_protocol", SAMPLING_PROTOCOL_V1)
        )
        sampling_protocols.add(sampling_protocol)
        collision_oracle = str(
            summary.get("collision_oracle", COLLISION_ORACLE_LEGACY)
        )
        collision_oracles.add(collision_oracle)
        zero_margin_oracle_sha256 = summary.get(
            "zero_margin_oracle_sha256"
        )
        if collision_oracle == COLLISION_ORACLE_ZERO_MARGIN:
            if not isinstance(zero_margin_oracle_sha256, str) or len(
                zero_margin_oracle_sha256
            ) != 64:
                raise ValueError(
                    f"v3 summary has no frozen zero-margin oracle SHA-256: {slug}"
                )
            zero_margin_oracle_hashes.add(zero_margin_oracle_sha256)
        elif zero_margin_oracle_sha256 is not None:
            raise ValueError(
                f"non-v3 summary declares zero-margin oracle SHA-256: {slug}"
            )
        explicit_collision_oracle = explicit_collision_oracle or (
            "collision_oracle" in summary
        )
        try:
            evidence = str((path / "summary.json").relative_to(Path(root).resolve()))
        except ValueError:
            evidence = str(path / "summary.json")
        entry = {
            "slug": slug,
            "display": display,
            "dataset": display,
            "N_eval": summary["n_eval"],
            "J_eval": summary["j_eval"],
            "status": summary.get("status"),
            "sampling_protocol": sampling_protocol,
            "metrics": summary.get("metrics", {}),
            "source_bindings": summary.get("source_bindings", []),
            "output": str(path),
            "evidence": {"summary": evidence},
        }
        if "collision_oracle" in summary:
            entry["collision_oracle"] = collision_oracle
        if collision_oracle == COLLISION_ORACLE_ZERO_MARGIN:
            entry["zero_margin_oracle_sha256"] = zero_margin_oracle_sha256
        entries.append(entry)

    # Keep the user-specified external inputs visible at the combined level.
    # These are provenance bindings, not implicit evaluation inputs: the
    # Infinite cohort is already materialized in its frozen roster and the
    # Infinigen URDFs are the validated extraction of the archive set.
    root_path = Path(root).resolve()
    exp_root = root_path.parents[1] if len(root_path.parents) > 1 else EXP_ROOT
    global_bindings: list[dict[str, Any]] = []
    by_key: set[tuple[str, str]] = set()
    for entry in entries:
        for binding in entry.get("source_bindings", []):
            if not isinstance(binding, Mapping):
                continue
            key = (str(binding.get("name", "")), str(binding.get("path", "")))
            if key not in by_key:
                by_key.add(key)
                global_bindings.append(dict(binding))
    archive = exp_root / "parts.zip"
    if archive.is_file() and not any(item.get("name") == "parts_zip" for item in global_bindings):
        global_bindings.append({"name": "parts_zip", "path": str(archive), "sha256": sha256_file(archive), "bytes": archive.stat().st_size})
    source_root = exp_root / "Infinigen-Sim"
    if source_root.is_dir() and not any(item.get("name") == "source_root" for item in global_bindings):
        global_bindings.append({"name": "source_root", "path": str(source_root)})
    archive_receipt = exp_root / "runtime" / "table123_full_release_20260825" / "infinigen_archive_validation_receipt.json"
    if archive_receipt.is_file() and not any(item.get("name") == "infinigen_archive_validation_receipt" for item in global_bindings):
        global_bindings.append({"name": "infinigen_archive_validation_receipt", "path": str(archive_receipt), "sha256": sha256_file(archive_receipt)})
    if len(sampling_protocols) > 1:
        raise ValueError("combined receipt requires one homogeneous sampling protocol")
    if len(collision_oracles) > 1:
        raise ValueError("combined receipt requires one homogeneous collision oracle")
    if len(zero_margin_oracle_hashes) > 1:
        raise ValueError(
            "combined receipt requires one homogeneous zero-margin oracle SHA-256"
        )
    sampling_protocol = next(iter(sampling_protocols), SAMPLING_PROTOCOL_V1)
    collision_oracle = next(
        iter(collision_oracles), COLLISION_ORACLE_LEGACY
    )
    receipt_v3 = collision_oracle == COLLISION_ORACLE_ZERO_MARGIN
    receipt_v2 = sampling_protocol == SAMPLING_PROTOCOL_V2
    result = {
        "schema_version": (
            RECEIPT_SCHEMA_V3
            if receipt_v3
            else (RECEIPT_SCHEMA_V2 if receipt_v2 else RECEIPT_SCHEMA)
        ),
        "root": str(root_path),
        "protocol": {
            "id": (
                "urdf_sim_ready_table4_full_release_v3"
                if receipt_v3
                else (
                    "urdf_sim_ready_table4_full_release_v2"
                    if receipt_v2
                    else "urdf_sim_ready_table4_full_release_v1"
                )
            ),
            "sampling_protocol": sampling_protocol,
            "single_joint_samples": SINGLE_SAMPLES,
            "sobol_samples": SOBOL_SAMPLES,
            "sobol_seed": SOBOL_SEED,
            "penetration_threshold_m": PENETRATION_THRESHOLD_M,
            "pair_policy": "all-pair plus non-adjacent (direct parent-child excluded)",
            "empty_contact_query_is_not_a_pass": True,
        },
        "source_bindings": global_bindings,
        "methods": entries,
        "datasets": entries,
    }
    if explicit_collision_oracle:
        result["protocol"]["collision_oracle"] = collision_oracle
    if receipt_v3:
        if len(zero_margin_oracle_hashes) != 1:
            raise ValueError(
                "combined v3 receipt has no frozen zero-margin oracle SHA-256"
            )
        result["protocol"]["zero_margin_oracle_sha256"] = next(
            iter(zero_margin_oracle_hashes)
        )
    return result


def write_combined_receipt(outputs: Mapping[str, Path | str], root: Path) -> Path:
    receipt = build_combined_receipt(outputs, root); receipt["created_at"] = utc_now(); receipt["receipt_content_sha256"] = _self_hash(receipt, "receipt_content_sha256"); path = Path(root) / "full_release_receipt.json"; atomic_json(path, receipt); return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", choices=DATASETS); parser.add_argument("--roster", type=Path); parser.add_argument("--output", type=Path); parser.add_argument("--workers", type=int, default=32); parser.add_argument("--timeout-seconds", type=float, default=300.0); parser.add_argument("--resume", action="store_true"); parser.add_argument("--sampling-protocol", choices=(SAMPLING_PROTOCOL_V1, SAMPLING_PROTOCOL_V2), default=SAMPLING_PROTOCOL_V1); parser.add_argument("--collision-oracle", choices=COLLISION_ORACLES); parser.add_argument("--parts-zip", type=Path, help=argparse.SUPPRESS); parser.add_argument("--source-root", type=Path, help=argparse.SUPPRESS); parser.add_argument("--child", action="store_true", help=argparse.SUPPRESS); parser.add_argument("--job", type=Path, help=argparse.SUPPRESS); parser.add_argument("--result", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.child:
        if args.job is None or args.result is None: parser.error("--child requires --job and --result")
        return _child_main(args.job, args.result)
    if args.dataset is None or args.roster is None or args.output is None: parser.error("--dataset, --roster and --output are required")
    try: run_dataset(args.roster, args.output, workers=args.workers, timeout_seconds=args.timeout_seconds, resume=args.resume, dataset=args.dataset, parts_zip=args.parts_zip, source_root=args.source_root, sampling_protocol=args.sampling_protocol, collision_oracle=args.collision_oracle)
    except Exception as exc: print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr); return 1
    print(json.dumps({"status": "completed", "output": str(args.output.resolve())}, sort_keys=True)); return 0


if __name__ == "__main__":
    raise SystemExit(main())
