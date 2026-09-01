#!/usr/bin/env python3
"""Run PV-A Table 4 v4 with analytic FK and python-fcl, without a simulator."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import sqlite3
import stat
import subprocess
import sys
import tempfile
import threading
from typing import Any, Mapping


SCRIPT = Path(__file__).resolve()
SCRIPT_DIR = SCRIPT.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import run_pva_table4_kinematic_aware_full_release as v3  # noqa: E402
import run_pva_table4_mimic_aware_full_release as v2  # noqa: E402
import run_table4_simulator_free as simulator_free  # noqa: E402


RUN_SCHEMA_VERSION = "pva_table4_simulator_free_full_release_run_v4"
RESULT_DB_SCHEMA_VERSION = "pva_table4_simulator_free_results_db_v1"
RECEIPT_SCHEMA_VERSION = "pva_table4_simulator_free_full_release_receipt_v1"
SUMMARY_SCHEMA_VERSION = "pva_table4_simulator_free_summary_v4"
CHECKPOINT_SCHEMA_VERSION = "pva_table4_simulator_free_checkpoint_v1"
ARTIFACT_SCHEMA_VERSION = "pva_table4_simulator_free_artifacts_v1"
PROGRESS_SCHEMA_VERSION = "pva_table4_simulator_free_progress_v1"
PROVISIONAL_RECEIPT_NAME = ".full_release_receipt.provisional.json"
PROTOCOL_ID = "urdf_sim_ready_table4_pva_full_release_v4_simulator_free"
SAMPLING_PROTOCOL = simulator_free.SAMPLING_PROTOCOL
DEFAULT_SOURCE_EVALUATION = v2.DEFAULT_SOURCE_EVALUATION
DEFAULT_OUTPUT = (
    SCRIPT.parents[1] / "runtime" / "pva_table4_simulator_free_full_release_v4"
)
DEFAULT_CONTACT_REGISTRY = simulator_free.DEFAULT_CONTACT_REGISTRY
DEFAULT_CHILD_PYTHON = simulator_free.DEFAULT_CHILD_PYTHON

table4 = v2.table4
pva_run = v2.pva_run
pva_roster = v2.pva_roster

_BASE_RUN = v2.run_pva_table4_v2
_BASE_SOURCE_CONTEXT = v2._source_context
_BASE_ATOMIC_JSON = v2._atomic_json
_BASE_MANIFEST = v2._manifest
_BASE_VALIDATE_RESULT = v2._validate_result
_BASE_FINALIZE = v2._finalize
_BASE_WRITE_PROGRESS = v2._write_progress
_BASE_ARTIFACT_MANIFEST = v2._artifact_manifest
_HOOK_LOCK = threading.RLock()
_ACTIVE_SOURCE_HASHES: dict[str, str] | None = None
_ACTIVE_CHILD_SOURCE_HASHES: dict[str, str] | None = None
_ACTIVE_CONTACT_REGISTRY: Path | None = None
_ACTIVE_CONTACT_REGISTRY_SHA256: str | None = None
_ACTIVE_CHILD_PYTHON: Path | None = None
_ACTIVE_CHILD_RUNTIME: dict[str, Any] | None = None
_ACTIVE_SOURCE_DATABASE_ATTESTATION: dict[str, Any] | None = None
FORBIDDEN_SIMULATOR_MODULES = ("pybullet", "genesis", "mujoco")


def _orchestrator_runtime_attestation() -> dict[str, Any]:
    loaded = sorted(
        name
        for name in FORBIDDEN_SIMULATOR_MODULES
        if name in sys.modules
        or any(module.startswith(f"{name}.") for module in sys.modules)
    )
    return {
        "simulator_modules_loaded": loaded,
        "simulator_free_runtime_verified": not loaded,
    }


def _assert_orchestrator_simulator_free() -> None:
    attestation = _orchestrator_runtime_attestation()
    if not attestation["simulator_free_runtime_verified"]:
        raise RuntimeError(
            "simulator module loaded in v4 orchestrator: "
            + ", ".join(attestation["simulator_modules_loaded"])
        )


@contextmanager
def _output_lock(output: Path) -> Any:
    requested = Path(os.path.abspath(os.fspath(output)))
    if requested.name in {"", ".", ".."}:
        raise ValueError("v4 output must name a dedicated directory")
    requested.parent.mkdir(parents=True, exist_ok=True)
    parent = requested.parent.resolve(strict=True)
    normalized = parent / requested.name
    if normalized.is_symlink():
        raise ValueError("v4 output directory must not be a symlink")
    lock_path = parent / f".{normalized.name}.table4-v4.lock"
    flags = os.O_CREAT | os.O_RDWR
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(lock_path, flags, 0o600)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"another v4 run owns output lock: {lock_path}") from exc
        yield normalized
    finally:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)


def _compute_source_hashes(contact_registry: Path) -> dict[str, str]:
    return {
        "adapter": v2._sha256_file(SCRIPT),
        "base_adapter": v2._sha256_file(Path(v2.__file__)),
        "kinematic_adapter": v2._sha256_file(Path(v3.__file__)),
        "table4_host_runner": v2._sha256_file(Path(table4.__file__)),
        "simulator_free_runner": v2._sha256_file(Path(simulator_free.__file__)),
        "simulator_free_oracle": v2._sha256_file(simulator_free.ORACLE_SCRIPT),
        "sampling_core": v2._sha256_file(simulator_free.SAMPLING_CORE_SCRIPT),
        "intended_contact_policy": v2._sha256_file(
            simulator_free.CONTACT_POLICY_SCRIPT
        ),
        "intended_contact_registry": v2._sha256_file(contact_registry),
        "kinematic_constraint_registry": v2._sha256_file(
            v3.KINEMATIC_CONSTRAINT_REGISTRY
        ),
        "pva_roster_verifier": v2._sha256_file(Path(pva_roster.__file__)),
        "pva_table1234_runner": v2._sha256_file(Path(pva_run.__file__)),
        "table123_common": v2._sha256_file(Path(pva_run.common.__file__)),
    }


def _stable_file_identity(path: Path) -> dict[str, int]:
    metadata = path.stat()
    if not stat.S_ISREG(metadata.st_mode):
        raise ValueError(f"expected regular file: {path}")
    return {
        "device": int(metadata.st_dev),
        "inode": int(metadata.st_ino),
        "bytes": int(metadata.st_size),
        "mtime_ns": int(metadata.st_mtime_ns),
        "ctime_ns": int(metadata.st_ctime_ns),
    }


def _source_sidecar_state(database_path: Path) -> dict[str, dict[str, Any]]:
    state: dict[str, dict[str, Any]] = {}
    for suffix in ("-wal", "-journal"):
        path = Path(f"{database_path}{suffix}")
        try:
            metadata = path.lstat()
        except FileNotFoundError:
            state[suffix] = {"exists": False, "bytes": 0}
            continue
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
            raise ValueError(f"unsafe SQLite source sidecar: {path}")
        if metadata.st_size:
            raise ValueError(
                f"source database has an uncheckpointed SQLite sidecar: {path}"
            )
        state[suffix] = {
            "exists": True,
            **_stable_file_identity(path),
        }
    return state


def _assert_source_snapshot_matches(attestation: Mapping[str, Any]) -> None:
    receipt_path = Path(str(attestation["source_receipt"]))
    if receipt_path.is_symlink():
        raise ValueError("source receipt became a symlink")
    receipt_identity_before = _stable_file_identity(receipt_path)
    receipt_sha256 = v2._sha256_file(receipt_path)
    receipt_identity_after = _stable_file_identity(receipt_path)
    if receipt_identity_before != receipt_identity_after:
        raise RuntimeError("source receipt changed while it was being checked")
    if receipt_identity_after != attestation.get("source_receipt_file_identity"):
        raise RuntimeError("source receipt identity changed")
    if receipt_sha256 != attestation.get("source_receipt_sha256"):
        raise RuntimeError("source receipt content changed")
    database_path = Path(str(attestation["source_result_database"]))
    if database_path.is_symlink():
        raise ValueError("source result database became a symlink")
    if _stable_file_identity(database_path) != attestation.get(
        "source_result_database_file_identity"
    ):
        raise RuntimeError("source result database identity changed")
    if _source_sidecar_state(database_path) != attestation.get(
        "source_result_database_sidecars"
    ):
        raise RuntimeError("source result database sidecar state changed")


def _connect_verified_source(path: Path) -> sqlite3.Connection:
    attestation = _source_database_attestation()
    database_path = Path(path).resolve(strict=True)
    if str(database_path) != attestation["source_result_database"]:
        raise ValueError("SQLite source path does not match the frozen attestation")
    _assert_source_snapshot_matches(attestation)
    connection = sqlite3.connect(
        f"{database_path.as_uri()}?mode=ro&immutable=1",
        uri=True,
    )
    try:
        connection.execute("PRAGMA query_only=ON")
        _assert_source_snapshot_matches(attestation)
    except BaseException:
        connection.close()
        raise
    return connection


def _verify_source_database(source_evaluation: Path) -> dict[str, Any]:
    source_evaluation = Path(source_evaluation).resolve(strict=True)
    if not source_evaluation.is_dir():
        raise ValueError("source evaluation must be a directory")
    receipt_path = source_evaluation / "full_release_receipt.json"
    database_path = source_evaluation / "results.sqlite3"
    for path, label in (
        (receipt_path, "source receipt"),
        (database_path, "source result database"),
    ):
        if path.is_symlink():
            raise ValueError(f"{label} must not be a symlink: {path}")
        try:
            metadata = path.lstat()
        except FileNotFoundError as exc:
            raise ValueError(f"{label} is missing: {path}") from exc
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError(f"{label} must be a regular file: {path}")

    receipt_identity_before = _stable_file_identity(receipt_path)
    receipt_sha256 = v2._sha256_file(receipt_path)
    receipt = v2._load_json(receipt_path)
    receipt_identity_after = _stable_file_identity(receipt_path)
    if receipt_identity_before != receipt_identity_after:
        raise RuntimeError("source receipt changed while it was being verified")
    if receipt.get("schema_version") != pva_run.RECEIPT_SCHEMA_VERSION:
        raise ValueError("source evaluation is not a sealed PV-A Table 1--4 release")
    if receipt.get("receipt_content_sha256") != v2._self_hash(
        receipt, "receipt_content_sha256"
    ):
        raise ValueError("source PV-A receipt self-hash mismatch")
    if receipt.get("result_database") != "results.sqlite3":
        raise ValueError("source PV-A receipt result database path mismatch")
    declared_sha256 = receipt.get("result_database_sha256")
    if (
        not isinstance(declared_sha256, str)
        or len(declared_sha256) != 64
        or any(character not in "0123456789abcdef" for character in declared_sha256)
    ):
        raise ValueError("source PV-A receipt database digest is invalid")

    database_sidecars_before = _source_sidecar_state(database_path)
    database_identity_before = _stable_file_identity(database_path)
    observed_sha256 = v2._sha256_file(database_path)
    database_identity_after = _stable_file_identity(database_path)
    database_sidecars_after = _source_sidecar_state(database_path)
    if database_identity_before != database_identity_after:
        raise RuntimeError("source result database changed while it was being hashed")
    if database_sidecars_before != database_sidecars_after:
        raise RuntimeError(
            "source result database sidecars changed while it was being hashed"
        )
    if observed_sha256 != declared_sha256:
        raise ValueError("source PV-A result database SHA-256 mismatch")
    if receipt_path.is_symlink():
        raise ValueError("source receipt became a symlink")
    receipt_identity_final_before = _stable_file_identity(receipt_path)
    receipt_sha256_final = v2._sha256_file(receipt_path)
    receipt_identity_final_after = _stable_file_identity(receipt_path)
    if (
        receipt_identity_final_before != receipt_identity_final_after
        or receipt_identity_final_after != receipt_identity_after
        or receipt_sha256_final != receipt_sha256
    ):
        raise RuntimeError("source receipt changed during database verification")

    value = {
        "schema_version": "pva_table4_source_database_attestation_v1",
        "source_evaluation": str(source_evaluation),
        "source_receipt": str(receipt_path),
        "source_receipt_sha256": receipt_sha256,
        "source_receipt_file_identity": receipt_identity_final_after,
        "source_receipt_content_sha256": receipt["receipt_content_sha256"],
        "source_result_database": str(database_path),
        "source_result_database_bytes": database_identity_after["bytes"],
        "source_result_database_declared_sha256": declared_sha256,
        "source_result_database_observed_sha256": observed_sha256,
        "source_result_database_file_identity": database_identity_after,
        "source_result_database_sidecars": database_sidecars_after,
    }
    value["attestation_content_sha256"] = v2._self_hash(
        value, "attestation_content_sha256"
    )
    return value


def _source_database_attestation() -> dict[str, Any]:
    if _ACTIVE_SOURCE_DATABASE_ATTESTATION is None:
        raise RuntimeError("source database attestation is not frozen")
    return dict(_ACTIVE_SOURCE_DATABASE_ATTESTATION)


def _verified_source_context(
    source_evaluation: Path,
) -> tuple[dict[str, Any], Any]:
    attestation = _source_database_attestation()
    requested_source = str(Path(source_evaluation).resolve(strict=True))
    if requested_source != attestation["source_evaluation"]:
        raise ValueError("source evaluation does not match the frozen attestation")
    _assert_source_snapshot_matches(attestation)
    context, source = _BASE_SOURCE_CONTEXT(source_evaluation)
    try:
        expected = {
            "source_evaluation": attestation["source_evaluation"],
            "source_receipt": attestation["source_receipt"],
            "source_receipt_sha256": attestation["source_receipt_sha256"],
            "source_receipt_content_sha256": attestation[
                "source_receipt_content_sha256"
            ],
            "source_result_database": attestation["source_result_database"],
            "source_result_database_declared_sha256": attestation[
                "source_result_database_declared_sha256"
            ],
        }
        for field, value in expected.items():
            if context.get(field) != value:
                raise ValueError(f"source context/attestation mismatch: {field}")
        context.update(
            {
                "source_result_database_observed_sha256": attestation[
                    "source_result_database_observed_sha256"
                ],
                "source_result_database_bytes": attestation[
                    "source_result_database_bytes"
                ],
                "source_database_attestation": attestation,
                "source_database_attestation_sha256": attestation[
                    "attestation_content_sha256"
                ],
            }
        )
        _assert_source_snapshot_matches(attestation)
    except BaseException:
        source.close()
        raise
    return context, source


def _verify_source_database_unchanged() -> None:
    expected = _source_database_attestation()
    observed = _verify_source_database(Path(expected["source_evaluation"]))
    if observed != expected:
        raise RuntimeError("source PV-A database or receipt changed during the v4 run")


def _source_hashes() -> dict[str, str]:
    if _ACTIVE_SOURCE_HASHES is not None:
        return dict(_ACTIVE_SOURCE_HASHES)
    registry = (_ACTIVE_CONTACT_REGISTRY or DEFAULT_CONTACT_REGISTRY).resolve(
        strict=True
    )
    return _compute_source_hashes(registry)


def _child_source_hashes() -> dict[str, str]:
    if _ACTIVE_CHILD_SOURCE_HASHES is not None:
        return dict(_ACTIVE_CHILD_SOURCE_HASHES)
    return simulator_free._source_hashes()


def _contact_registry_path() -> Path:
    return (_ACTIVE_CONTACT_REGISTRY or DEFAULT_CONTACT_REGISTRY).resolve(strict=True)


def _contact_registry_sha256() -> str:
    return _ACTIVE_CONTACT_REGISTRY_SHA256 or v2._sha256_file(_contact_registry_path())


def _child_python() -> Path:
    launcher = Path(
        os.path.abspath(os.fspath(_ACTIVE_CHILD_PYTHON or DEFAULT_CHILD_PYTHON))
    )
    if not launcher.is_file() or not os.access(launcher, os.X_OK):
        raise ValueError(f"child Python launcher is not executable: {launcher}")
    return launcher


def _probe_child_runtime(python: Path) -> dict[str, Any]:
    completed = subprocess.run(
        [
            str(python),
            str(Path(simulator_free.__file__).resolve()),
            "--runtime-identity",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=180.0,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "simulator-free child runtime probe failed: " + completed.stderr[-2000:]
        )
    try:
        identity = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            "simulator-free child runtime probe returned invalid JSON"
        ) from exc
    if not isinstance(identity, dict) or not identity.get(
        "simulator_free_runtime_verified"
    ):
        raise RuntimeError("child runtime is not simulator-free")
    if identity.get("simulator_modules_loaded") != []:
        raise RuntimeError("child runtime loaded a simulator module")
    if identity.get("python_executable") != str(
        Path(os.path.abspath(os.fspath(python)))
    ):
        raise RuntimeError(
            "child runtime did not preserve the requested Python launcher"
        )
    module_hashes = identity.get("module_sha256")
    if not isinstance(module_hashes, Mapping) or not {
        "python_fcl_extension",
        "numpy_core_extension",
    } <= set(module_hashes):
        raise RuntimeError("child runtime did not attest required native extensions")
    return identity


def _runtime_identity() -> dict[str, Any]:
    if _ACTIVE_CHILD_RUNTIME is None:
        return _probe_child_runtime(_child_python())
    return dict(_ACTIVE_CHILD_RUNTIME)


def _child_runtime_identity_sha256() -> str:
    return simulator_free.canonical_sha256(_runtime_identity())


def _verify_child_runtime_unchanged() -> None:
    observed = _probe_child_runtime(_child_python())
    if simulator_free.canonical_sha256(observed) != _child_runtime_identity_sha256():
        raise RuntimeError("simulator-free child runtime changed during the v4 run")


def _verify_frozen_sources(
    expected: Mapping[str, str],
    *,
    contact_registry: Path | None = None,
) -> None:
    registry = (
        Path(contact_registry).resolve(strict=True)
        if contact_registry is not None
        else _contact_registry_path()
    )
    observed = _compute_source_hashes(registry)
    if dict(expected) != observed:
        changed = sorted(
            key
            for key in set(expected) | set(observed)
            if expected.get(key) != observed.get(key)
        )
        raise ValueError("PV-A v4 execution sources changed: " + ", ".join(changed))
    child_observed = simulator_free._source_hashes()
    if child_observed != _child_source_hashes():
        raise ValueError("PV-A v4 child execution sources changed")


def _job_with_plan(
    row: Mapping[str, Any],
    plan: Mapping[str, Any],
    *,
    kinematic_constraint_binding: Mapping[str, Any] | None = None,
    package_root_binding: Mapping[str, Any] | None = None,
    execution_paths: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    normalized_plan = dict(plan)
    normalized_plan.setdefault("native_mimic_joint_count", 0)
    normalized_plan.setdefault("external_joint_constraint_count", 0)
    declared_dof = int(row.get("joint_count", 0))
    independent_dof = int(normalized_plan["independent_dof_count"])
    package = Path(str(row.get("source_path", ""))).resolve()
    urdf = Path(str(row.get("primary_urdf_path", ""))).resolve()
    child_hashes = _child_source_hashes()
    constraints = (
        [
            dict(value)
            for value in kinematic_constraint_binding["external_joint_constraints"]
        ]
        if kinematic_constraint_binding is not None
        else []
    )
    job: dict[str, Any] = {
        "protocol_id": PROTOCOL_ID,
        "evaluator_protocol_id": simulator_free.PROTOCOL_ID,
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
        "expected_movable_joints": declared_dof,
        "sampling_protocol": SAMPLING_PROTOCOL,
        **normalized_plan,
        "rest_state_expected": 1,
        "single_state_expected": simulator_free.SINGLE_SAMPLES * independent_dof,
        "sobol_state_expected": (
            simulator_free.SOBOL_SAMPLES if independent_dof else 0
        ),
        "package_binding_sha256": row.get("package_binding_sha256"),
        "roster_ordinal": int(row["ordinal"]),
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
        "collision_oracle": simulator_free.geometry_oracle().ORACLE_POLICY_ID,
        "execution_source_hashes": child_hashes,
        "execution_source_hashes_sha256": simulator_free.canonical_sha256(child_hashes),
        "child_runtime_identity_sha256": _child_runtime_identity_sha256(),
        "intended_contact_registry_path": str(_contact_registry_path()),
        "intended_contact_registry_sha256": _contact_registry_sha256(),
    }
    identity_fields = (
        "dataset",
        "dataset_id",
        "category",
        "evaluator_protocol_id",
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
        "execution_source_hashes",
        "execution_source_hashes_sha256",
        "child_runtime_identity_sha256",
        "intended_contact_registry_path",
        "intended_contact_registry_sha256",
    )
    job["input_identity_sha256"] = simulator_free.canonical_sha256(
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
            job["execution_input_sha256"] = simulator_free.canonical_sha256(
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
                    "expected_primary_urdf_sha256": job["expected_primary_urdf_sha256"],
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
    binding = v3._kinematic_constraint_binding(row, package=package, urdf=urdf)
    external = binding["external_joint_constraints"] if binding is not None else []
    plan = v2._collision_core().sampling_plan_metadata(
        urdf,
        declared_dof=int(row.get("joint_count", 0)),
        expected_sha256=row.get("primary_urdf_sha256"),
        external_joint_constraints=external,
    )
    if plan.get("sampling_plan_error") is not None:
        raise ValueError(
            f"simulator-free sampling plan failed: {plan['sampling_plan_error']}"
        )
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
    simulator_free.validate_result_binding(result, job)
    validated = _BASE_VALIDATE_RESULT(result, job)
    for field in (
        "native_mimic_joint_count",
        "external_joint_constraint_count",
        "kinematic_constraint_binding",
        "kinematic_constraint_binding_sha256",
        "collision_oracle",
        "evaluator_protocol_id",
        "execution_source_hashes",
        "execution_source_hashes_sha256",
        "child_runtime_identity_sha256",
        "intended_contact_registry_path",
        "intended_contact_registry_sha256",
    ):
        if result.get(field) != job.get(field):
            raise ValueError(f"simulator-free child result binding mismatch: {field}")
    return validated


def _validated_result_prefix(result: Any, source: Any) -> int:
    database_rows = result.execute("PRAGMA database_list").fetchall()
    paths = [
        Path(str(path)).resolve()
        for _, name, path in database_rows
        if name == "main" and path
    ]
    if len(paths) != 1:
        raise ValueError("cannot resolve PV-A v4 result database path")
    manifest = v2._load_json(paths[0].parent / "manifest.json")
    _manifest_bindings(manifest)
    package_root_binding = manifest.get("package_root_binding")
    source_cursor = source.execute(
        "SELECT ordinal, asset_id, category, joint_count, row_sha256, row_json "
        "FROM assets ORDER BY ordinal"
    )
    result_cursor = result.execute(
        "SELECT ordinal, asset_id, record_json, states_zlib, state_count "
        "FROM results ORDER BY ordinal"
    )
    expected = 0
    for ordinal, asset_id, payload, states_blob, state_count in result_cursor:
        try:
            source_values = next(source_cursor)
        except StopIteration as exc:
            raise ValueError(
                "v4 result database extends beyond the source roster"
            ) from exc
        row = v3._validated_source_row(source_values, expected)
        if int(ordinal) != expected or str(asset_id) != str(row["asset_id"]):
            raise ValueError(f"v4 result/source identity mismatch at {expected}")
        record = json.loads(payload)
        if not isinstance(record, dict):
            raise ValueError(f"v4 result record {expected} is not an object")
        if record.get("joint_sampling_plan_sha256") is None:
            job = v2._fallback_job(
                row,
                "resume_reconstruction_of_retained_preflight_failure",
                package_root_binding,
            )
        else:
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
        states = v3._stored_states(bytes(states_blob), int(state_count))
        record["state_records"] = states
        _validate_result(record, job)
        expected += 1
    return expected


def _execute_child(
    job: Mapping[str, Any], output: Path, timeout_seconds: float
) -> dict[str, Any]:
    return simulator_free.execute_child(
        job,
        output,
        timeout_seconds,
        python=_child_python(),
    )


def _manifest(*args: Any, **kwargs: Any) -> dict[str, Any]:
    value = _BASE_MANIFEST(*args, **kwargs)
    source_attestation = _source_database_attestation()
    value["schema_version"] = RUN_SCHEMA_VERSION
    value["protocol_id"] = PROTOCOL_ID
    value["source_database_attestation_sha256"] = source_attestation[
        "attestation_content_sha256"
    ]
    value["runtime_identity"] = _runtime_identity()
    value["child_runtime_identity_sha256"] = _child_runtime_identity_sha256()
    value["orchestrator_runtime_attestation"] = _orchestrator_runtime_attestation()
    value["execution"]["child_python"] = str(_child_python())
    value["execution"]["simulator_import_forbidden"] = True
    value["execution"]["physics_engine_initialization"] = "not_run"
    value["protocol"].update(
        {
            "forward_kinematics": "analytic_urdf_tree_v1",
            "collision_oracle": simulator_free.geometry_oracle().ORACLE_POLICY_ID,
            "collision_backend": "python-fcl",
            "simulator_used": False,
            "contact_depth_semantics": ("fcl_contact_depth_proxy_not_global_mesh_mtd"),
            "exact_global_mesh_penetration_depth": "N/E",
            "raw_intersection_view": "all_fcl_contacts_including_zero_depth",
            "penetration_proxy_view": (
                "depth_proxy_gt_1e-6m; missing_depth_fails_closed"
            ),
            "pair_policy": (
                "same_link_excluded; raw_XML_direct_parent_excluded_only_from_"
                "non_adjacent_panel"
            ),
            "external_kinematic_constraints": {
                "policy": "reviewed_hash_bound_affine_registry_v1",
                "registry": str(v3.KINEMATIC_CONSTRAINT_REGISTRY.resolve()),
                "registry_sha256": v3._kinematic_constraint_registry_sha256(),
            },
            "intended_contact_policy": {
                "policy": "reviewed_exact_asset_local_contact_registry_v1",
                "registry": str(_contact_registry_path()),
                "registry_sha256": _contact_registry_sha256(),
                "raw_results_preserved": True,
                "unmatched_contacts_fail_closed": True,
            },
        }
    )
    value["child_execution_source_hashes"] = _child_source_hashes()
    value["child_execution_source_hashes_sha256"] = simulator_free.canonical_sha256(
        _child_source_hashes()
    )
    value["execution_source_hashes_sha256"] = simulator_free.canonical_sha256(
        value["source_hashes"]
    )
    value["manifest_content_sha256"] = v2._self_hash(value, "manifest_content_sha256")
    return value


def _manifest_bindings(manifest: Mapping[str, Any]) -> None:
    if manifest.get("schema_version") != RUN_SCHEMA_VERSION:
        raise ValueError("PV-A v4 manifest schema mismatch")
    if manifest.get("protocol_id") != PROTOCOL_ID:
        raise ValueError("PV-A v4 manifest protocol mismatch")
    if manifest.get("manifest_content_sha256") != v2._self_hash(
        manifest, "manifest_content_sha256"
    ):
        raise ValueError("PV-A v4 manifest self-hash mismatch")
    source = manifest.get("source")
    source_attestation = (
        source.get("source_database_attestation")
        if isinstance(source, Mapping)
        else None
    )
    if not isinstance(source_attestation, Mapping):
        raise ValueError("PV-A v4 source database attestation is missing")
    if source_attestation.get("attestation_content_sha256") != v2._self_hash(
        source_attestation, "attestation_content_sha256"
    ):
        raise ValueError("PV-A v4 source database attestation self-hash mismatch")
    if source_attestation.get(
        "source_result_database_declared_sha256"
    ) != source_attestation.get("source_result_database_observed_sha256"):
        raise ValueError("PV-A v4 source database observed digest mismatch")
    if manifest.get("source_database_attestation_sha256") != source_attestation.get(
        "attestation_content_sha256"
    ):
        raise ValueError("PV-A v4 source database attestation binding mismatch")
    if dict(source_attestation) != _source_database_attestation():
        raise ValueError("PV-A v4 source database attestation changed")
    if manifest.get("orchestrator_runtime_attestation") != {
        "simulator_modules_loaded": [],
        "simulator_free_runtime_verified": True,
    }:
        raise ValueError("PV-A v4 orchestrator runtime is not simulator-free")
    protocol = manifest.get("protocol")
    if not isinstance(protocol, Mapping) or protocol.get("simulator_used") is not False:
        raise ValueError("PV-A v4 manifest is not simulator-free")
    if (
        protocol.get("collision_oracle")
        != simulator_free.geometry_oracle().ORACLE_POLICY_ID
    ):
        raise ValueError("PV-A v4 manifest collision oracle mismatch")
    contact = protocol.get("intended_contact_policy")
    if (
        not isinstance(contact, Mapping)
        or contact.get("registry_sha256") != _contact_registry_sha256()
    ):
        raise ValueError("PV-A v4 contact registry binding mismatch")
    if manifest.get("child_execution_source_hashes") != _child_source_hashes():
        raise ValueError("PV-A v4 child source binding mismatch")
    if manifest.get("child_execution_source_hashes_sha256") != (
        simulator_free.canonical_sha256(manifest["child_execution_source_hashes"])
    ):
        raise ValueError("PV-A v4 child source digest mismatch")
    if manifest.get("source_hashes") != _source_hashes():
        raise ValueError("PV-A v4 source binding mismatch")
    if manifest.get("execution_source_hashes_sha256") != (
        simulator_free.canonical_sha256(manifest["source_hashes"])
    ):
        raise ValueError("PV-A v4 source digest mismatch")
    if manifest.get("child_runtime_identity_sha256") != (
        _child_runtime_identity_sha256()
    ):
        raise ValueError("PV-A v4 child runtime identity binding mismatch")
    if simulator_free.canonical_sha256(
        manifest.get("runtime_identity")
    ) != manifest.get("child_runtime_identity_sha256"):
        raise ValueError("PV-A v4 manifest runtime payload/digest mismatch")
    execution = manifest.get("execution")
    if not isinstance(execution, Mapping) or execution.get("child_python") != str(
        _child_python()
    ):
        raise ValueError("PV-A v4 child Python launcher binding mismatch")


def _finalize(*args: Any, **kwargs: Any) -> dict[str, Any]:
    if len(args) >= 3:
        connection, output, manifest = args[:3]
    else:
        connection = kwargs["connection"]
        output = kwargs["output"]
        manifest = kwargs["manifest"]
    output = Path(output)
    _assert_orchestrator_simulator_free()
    _verify_child_runtime_unchanged()
    _manifest_bindings(manifest)
    _verify_frozen_sources(manifest["source_hashes"])
    _verify_source_database_unchanged()
    fatal_runtime, fatal_source = connection.execute(
        "SELECT "
        "COALESCE(SUM(CAST(json_extract(record_json, "
        "'$.runtime_integrity_fatal') AS INTEGER)), 0), "
        "COALESCE(SUM(CAST(json_extract(record_json, "
        "'$.execution_source_integrity_fatal') AS INTEGER)), 0) "
        "FROM results"
    ).fetchone()
    if int(fatal_runtime):
        raise RuntimeError("refusing to publish: a v4 child loaded a simulator module")
    if int(fatal_source):
        raise RuntimeError("refusing to publish: v4 child execution sources drifted")
    summary = _BASE_FINALIZE(*args, **kwargs)
    native_mimic, external_constraints, bound_contacts = connection.execute(
        "SELECT "
        "COALESCE(SUM(CAST(json_extract(record_json, "
        "'$.native_mimic_joint_count') AS INTEGER)), 0), "
        "COALESCE(SUM(CAST(json_extract(record_json, "
        "'$.external_joint_constraint_count') AS INTEGER)), 0), "
        "COALESCE(SUM(CAST(json_extract(record_json, "
        "'$.bound_intended_contact_registration_count') AS INTEGER)), 0) "
        "FROM results"
    ).fetchone()
    summary.update(
        {
            "schema_version": SUMMARY_SCHEMA_VERSION,
            "protocol_id": PROTOCOL_ID,
            "collision_oracle": simulator_free.geometry_oracle().ORACLE_POLICY_ID,
            "simulator_used": False,
            "child_runtime": _runtime_identity(),
            "child_runtime_identity_sha256": _child_runtime_identity_sha256(),
            "execution_source_hashes_sha256": manifest[
                "execution_source_hashes_sha256"
            ],
            "child_execution_source_hashes_sha256": manifest[
                "child_execution_source_hashes_sha256"
            ],
            "source_database_attestation_sha256": manifest[
                "source_database_attestation_sha256"
            ],
            "kinematic_constraint_registry_sha256": (
                v3._kinematic_constraint_registry_sha256()
            ),
            "intended_contact_registry_sha256": _contact_registry_sha256(),
            "native_mimic_joint_count": int(native_mimic),
            "external_joint_constraint_count": int(external_constraints),
            "bound_intended_contact_registration_count": int(bound_contacts),
        }
    )
    summary["summary_content_sha256"] = v2._self_hash(summary, "summary_content_sha256")
    v2._atomic_json(output / "summary.json", summary)
    pva_run.common._atomic_write_bytes(
        output / "summary.md",
        simulator_free.report_text(summary, "Ours / PV-A").encode("utf-8"),
    )
    checkpoint_path = output / "checkpoint.json"
    checkpoint = v2._load_json(checkpoint_path)
    checkpoint.update(
        {
            "schema_version": CHECKPOINT_SCHEMA_VERSION,
            "protocol_id": PROTOCOL_ID,
            "collision_oracle": simulator_free.geometry_oracle().ORACLE_POLICY_ID,
            "simulator_used": False,
            "intended_contact_registry_sha256": _contact_registry_sha256(),
            "summary_sha256": v2._sha256_file(output / "summary.json"),
        }
    )
    checkpoint["checkpoint_content_sha256"] = v2._self_hash(
        checkpoint, "checkpoint_content_sha256"
    )
    v2._atomic_json(checkpoint_path, checkpoint)
    return summary


def _write_progress(*args: Any, **kwargs: Any) -> None:
    _BASE_WRITE_PROGRESS(*args, **kwargs)
    output = Path(args[0] if args else kwargs["output"])
    progress = v2._load_json(output / "progress.json")
    progress.update(
        {
            "schema_version": PROGRESS_SCHEMA_VERSION,
            "protocol_id": PROTOCOL_ID,
            "collision_oracle": simulator_free.geometry_oracle().ORACLE_POLICY_ID,
            "simulator_used": False,
            "intended_contact_registry_sha256": _contact_registry_sha256(),
        }
    )
    v2._atomic_json(output / "progress.json", progress)


def _artifact_manifest(output: Path) -> dict[str, Any]:
    value = _BASE_ARTIFACT_MANIFEST(output)
    value["schema_version"] = ARTIFACT_SCHEMA_VERSION
    value["artifact_manifest_content_sha256"] = v2._self_hash(
        value, "artifact_manifest_content_sha256"
    )
    return value


_RECEIPT_ARTIFACT_FIELDS = {
    "manifest": "manifest.json",
    "records": "records.jsonl",
    "state_records": "state_records.jsonl",
    "summary": "summary.json",
    "artifact_manifest": "artifact_manifest.json",
    "result_database": "results.sqlite3",
}
_ARTIFACT_MANIFEST_REQUIRED_PATHS = {
    "manifest.json",
    "protocol_snapshot.md",
    "records.jsonl",
    "asset_records.jsonl",
    "state_records.jsonl",
    "summary.json",
    "summary.md",
    "checkpoint.json",
    "results.sqlite3",
}


def _output_artifact_path(output: Path, name: Any) -> Path:
    if not isinstance(name, str):
        raise ValueError("receipt artifact path is not a string")
    relative = Path(name)
    if relative.is_absolute() or relative.name != name or name in {"", ".", ".."}:
        raise ValueError(f"unsafe receipt artifact path: {name!r}")
    path = output / relative
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"receipt artifact is not a regular file: {path}")
    return path


def _stable_hashed_output_file(path: Path) -> dict[str, Any]:
    before = _stable_file_identity(path)
    digest = v2._sha256_file(path)
    after = _stable_file_identity(path)
    if before != after:
        raise RuntimeError(f"output artifact changed while being hashed: {path}")
    return {"bytes": after["bytes"], "sha256": digest, "identity": after}


def _validate_receipt_artifacts(
    receipt: Mapping[str, Any], output: Path
) -> dict[str, dict[str, Any]]:
    output = Path(output)
    snapshots: dict[str, dict[str, Any]] = {}
    for field, expected_name in _RECEIPT_ARTIFACT_FIELDS.items():
        if receipt.get(field) != expected_name:
            raise ValueError(f"v4 receipt artifact path mismatch: {field}")
        path = _output_artifact_path(output, expected_name)
        snapshot = _stable_hashed_output_file(path)
        expected_hash = receipt.get(f"{field}_sha256")
        if snapshot["sha256"] != expected_hash:
            raise ValueError(f"v4 receipt artifact hash mismatch: {field}")
        snapshots[expected_name] = snapshot

    artifact_path = _output_artifact_path(output, receipt["artifact_manifest"])
    artifact = v2._load_json(artifact_path)
    if artifact.get("schema_version") != ARTIFACT_SCHEMA_VERSION:
        raise ValueError("v4 artifact manifest schema mismatch")
    if artifact.get("artifact_manifest_content_sha256") != v2._self_hash(
        artifact, "artifact_manifest_content_sha256"
    ):
        raise ValueError("v4 artifact manifest self-hash mismatch")
    entries = artifact.get("artifacts")
    if not isinstance(entries, list):
        raise ValueError("v4 artifact manifest entries are missing")
    entry_paths: list[str] = []
    for entry in entries:
        if not isinstance(entry, Mapping):
            raise ValueError("v4 artifact manifest entry is not an object")
        name = entry.get("path")
        if not isinstance(name, str) or name in entry_paths:
            raise ValueError("v4 artifact manifest has duplicate/invalid paths")
        entry_paths.append(name)
        if name not in _ARTIFACT_MANIFEST_REQUIRED_PATHS:
            raise ValueError(f"v4 artifact manifest has unexpected path: {name}")
        snapshot = snapshots.get(name)
        if snapshot is None:
            path = _output_artifact_path(output, name)
            snapshot = _stable_hashed_output_file(path)
            snapshots[name] = snapshot
        if (
            entry.get("bytes") != snapshot["bytes"]
            or entry.get("sha256") != snapshot["sha256"]
        ):
            raise ValueError(f"v4 artifact manifest entry mismatch: {name}")
    if set(entry_paths) != _ARTIFACT_MANIFEST_REQUIRED_PATHS:
        raise ValueError("v4 artifact manifest key set is incomplete")
    snapshots["__results_sqlite_sidecars__"] = {
        "state": _source_sidecar_state(output / "results.sqlite3")
    }
    return snapshots


def _atomic_json_exclusive(path: Path, value: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (v2._canonical_text(value) + "\n").encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=str(path.parent)
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary_path, path)
        directory_descriptor = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    finally:
        temporary_path.unlink(missing_ok=True)


def _provisional_receipt_atomic_json(path: Path, value: Any) -> None:
    path = Path(path)
    if path.name == "full_release_receipt.json":
        path = path.with_name(PROVISIONAL_RECEIPT_NAME)
    _BASE_ATOMIC_JSON(path, value)


@contextmanager
def _frozen_configuration(
    *,
    source_hashes: Mapping[str, str],
    child_source_hashes: Mapping[str, str],
    contact_registry: Path,
    contact_registry_sha256: str,
    child_python: Path,
    child_runtime: Mapping[str, Any],
    source_database_attestation: Mapping[str, Any],
) -> Any:
    global _ACTIVE_SOURCE_HASHES
    global _ACTIVE_CHILD_SOURCE_HASHES
    global _ACTIVE_CONTACT_REGISTRY
    global _ACTIVE_CONTACT_REGISTRY_SHA256
    global _ACTIVE_CHILD_PYTHON
    global _ACTIVE_CHILD_RUNTIME
    global _ACTIVE_SOURCE_DATABASE_ATTESTATION
    with _HOOK_LOCK:
        previous = (
            _ACTIVE_SOURCE_HASHES,
            _ACTIVE_CHILD_SOURCE_HASHES,
            _ACTIVE_CONTACT_REGISTRY,
            _ACTIVE_CONTACT_REGISTRY_SHA256,
            _ACTIVE_CHILD_PYTHON,
            _ACTIVE_CHILD_RUNTIME,
            _ACTIVE_SOURCE_DATABASE_ATTESTATION,
        )
        _ACTIVE_SOURCE_HASHES = dict(source_hashes)
        _ACTIVE_CHILD_SOURCE_HASHES = dict(child_source_hashes)
        _ACTIVE_CONTACT_REGISTRY = Path(contact_registry)
        _ACTIVE_CONTACT_REGISTRY_SHA256 = contact_registry_sha256
        _ACTIVE_CHILD_PYTHON = Path(child_python)
        _ACTIVE_CHILD_RUNTIME = dict(child_runtime)
        _ACTIVE_SOURCE_DATABASE_ATTESTATION = dict(source_database_attestation)
        try:
            yield
        finally:
            (
                _ACTIVE_SOURCE_HASHES,
                _ACTIVE_CHILD_SOURCE_HASHES,
                _ACTIVE_CONTACT_REGISTRY,
                _ACTIVE_CONTACT_REGISTRY_SHA256,
                _ACTIVE_CHILD_PYTHON,
                _ACTIVE_CHILD_RUNTIME,
                _ACTIVE_SOURCE_DATABASE_ATTESTATION,
            ) = previous


@contextmanager
def _v4_hooks() -> Any:
    v2_replacements = {
        "RUN_SCHEMA_VERSION": RUN_SCHEMA_VERSION,
        "RESULT_DB_SCHEMA_VERSION": RESULT_DB_SCHEMA_VERSION,
        "RECEIPT_SCHEMA_VERSION": RECEIPT_SCHEMA_VERSION,
        "PROTOCOL_ID": PROTOCOL_ID,
        "_job_with_plan": _job_with_plan,
        "_build_job": _build_job,
        "_validate_result": _validate_result,
        "_validated_result_prefix": _validated_result_prefix,
        "_source_hashes": _source_hashes,
        "_source_context": _verified_source_context,
        "_connect_source": _connect_verified_source,
        "_manifest": _manifest,
        "_finalize": _finalize,
        "_write_progress": _write_progress,
        "_artifact_manifest": _artifact_manifest,
        "_atomic_json": _provisional_receipt_atomic_json,
    }
    table4_replacements = {
        "SINGLE_SAMPLES": simulator_free.SINGLE_SAMPLES,
        "SOBOL_SAMPLES": simulator_free.SOBOL_SAMPLES,
        "SOBOL_SEED": simulator_free.SOBOL_SEED,
        "PENETRATION_THRESHOLD_M": simulator_free.PENETRATION_THRESHOLD_M,
        "runtime_identity": _runtime_identity,
        "_execute_child": _execute_child,
        "_empty_record": simulator_free._empty_record,
        "aggregate_records": simulator_free.aggregate_records,
        "report_text": simulator_free.report_text,
        "canonical_sha256": simulator_free.canonical_sha256,
    }
    with _HOOK_LOCK:
        previous_v2 = {name: getattr(v2, name) for name in v2_replacements}
        previous_table4 = {name: getattr(table4, name) for name in table4_replacements}
        try:
            for name, value in v2_replacements.items():
                setattr(v2, name, value)
            for name, value in table4_replacements.items():
                setattr(table4, name, value)
            yield
        finally:
            for name, value in previous_v2.items():
                setattr(v2, name, value)
            for name, value in previous_table4.items():
                setattr(table4, name, value)


def _publish_receipt(output: Path) -> None:
    output = Path(output)
    manifest = v2._load_json(output / "manifest.json")
    _manifest_bindings(manifest)
    provisional_path = output / PROVISIONAL_RECEIPT_NAME
    receipt_path = output / "full_release_receipt.json"
    if provisional_path.is_symlink() or receipt_path.is_symlink():
        raise ValueError("PV-A v4 receipt paths must not be symlinks")
    provisional_snapshot = _stable_hashed_output_file(provisional_path)
    receipt = v2._load_json(provisional_path)
    if receipt.get("schema_version") != RECEIPT_SCHEMA_VERSION:
        raise ValueError("PV-A v4 receipt schema mismatch")
    if receipt.get("protocol_id") != PROTOCOL_ID:
        raise ValueError("PV-A v4 receipt protocol mismatch")
    if receipt.get("receipt_content_sha256") != v2._self_hash(
        receipt, "receipt_content_sha256"
    ):
        raise ValueError("PV-A v4 receipt self-hash mismatch")
    if receipt.get("source") != manifest.get("source"):
        raise ValueError("v4 provisional receipt/source binding mismatch")
    if receipt.get("package_root_binding") != manifest.get("package_root_binding"):
        raise ValueError("v4 provisional receipt/package binding mismatch")
    for field in ("N_eval", "J_eval", "category_count", "sampling_protocol"):
        if receipt.get(field) != manifest.get(field):
            raise ValueError(f"v4 provisional receipt binding mismatch: {field}")
    _verify_frozen_sources(manifest["source_hashes"])
    _assert_orchestrator_simulator_free()
    _verify_child_runtime_unchanged()
    artifact_snapshot_before = _validate_receipt_artifacts(receipt, output)
    _verify_source_database_unchanged()
    artifact_snapshot_after = _validate_receipt_artifacts(receipt, output)
    if artifact_snapshot_before != artifact_snapshot_after:
        raise RuntimeError(
            "v4 output artifacts changed during final source verification"
        )
    _assert_source_snapshot_matches(_source_database_attestation())
    provisional_snapshot_after = _stable_hashed_output_file(provisional_path)
    if provisional_snapshot != provisional_snapshot_after:
        raise RuntimeError("v4 provisional receipt changed during final publication")
    summary = v2._load_json(output / "summary.json")
    if receipt.get("status") != summary.get("status") or receipt.get(
        "metrics"
    ) != summary.get("metrics"):
        raise ValueError("v4 provisional receipt/summary binding mismatch")
    receipt.update(
        {
            "collision_oracle": simulator_free.geometry_oracle().ORACLE_POLICY_ID,
            "simulator_used": False,
            "execution_source_hashes_sha256": manifest[
                "execution_source_hashes_sha256"
            ],
            "child_execution_source_hashes_sha256": manifest[
                "child_execution_source_hashes_sha256"
            ],
            "child_runtime_identity_sha256": manifest["child_runtime_identity_sha256"],
            "source_database_attestation_sha256": manifest[
                "source_database_attestation_sha256"
            ],
            "kinematic_constraint_registry_sha256": (
                v3._kinematic_constraint_registry_sha256()
            ),
            "intended_contact_registry_sha256": _contact_registry_sha256(),
        }
    )
    receipt["receipt_content_sha256"] = v2._self_hash(receipt, "receipt_content_sha256")
    _atomic_json_exclusive(receipt_path, receipt)
    provisional_path.unlink()


def _run_pva_table4_v4_locked(
    source_evaluation: Path,
    output: Path,
    *,
    workers: int = 32,
    timeout_seconds: float = 600.0,
    resume: bool = False,
    limit: int | None = None,
    batch_size: int | None = None,
    package_root_override: Path | None = None,
    contact_registry: Path = DEFAULT_CONTACT_REGISTRY,
    child_python: Path = DEFAULT_CHILD_PYTHON,
) -> Path:
    _assert_orchestrator_simulator_free()
    formal_receipt = Path(output) / "full_release_receipt.json"
    if formal_receipt.is_symlink() or formal_receipt.exists():
        raise ValueError(
            "v4 output already has a formal receipt; use a new output directory"
        )
    contact_registry = Path(contact_registry).resolve(strict=True)
    if not contact_registry.is_file() or contact_registry.is_symlink():
        raise ValueError("contact registry must be a regular file")
    normalized_registry = simulator_free.contact_policy().validate_registry(
        v2._load_json(contact_registry)
    )
    if (
        normalized_registry["schema_version"]
        != simulator_free.contact_policy().REGISTRY_SCHEMA
    ):
        raise ValueError("contact registry schema mismatch")
    child_python = Path(os.path.abspath(os.fspath(child_python)))
    if not child_python.is_file() or not os.access(child_python, os.X_OK):
        raise ValueError(f"child Python launcher is not executable: {child_python}")
    source_hashes = _compute_source_hashes(contact_registry)
    child_source_hashes = simulator_free._source_hashes()
    contact_hash = v2._sha256_file(contact_registry)
    child_runtime = _probe_child_runtime(child_python)
    source_database_attestation = _verify_source_database(source_evaluation)
    if (
        child_runtime.get("oracle_source_sha256")
        != child_source_hashes["simulator_free_oracle"]
    ):
        raise ValueError("child runtime oracle source hash mismatch")
    _verify_frozen_sources(source_hashes, contact_registry=contact_registry)
    with _frozen_configuration(
        source_hashes=source_hashes,
        child_source_hashes=child_source_hashes,
        contact_registry=contact_registry,
        contact_registry_sha256=contact_hash,
        child_python=child_python,
        child_runtime=child_runtime,
        source_database_attestation=source_database_attestation,
    ):
        with _v4_hooks():
            provisional_path = Path(output) / PROVISIONAL_RECEIPT_NAME
            if provisional_path.is_symlink():
                raise ValueError("PV-A v4 provisional receipt must not be a symlink")
            provisional_path.unlink(missing_ok=True)
            try:
                completed = _BASE_RUN(
                    source_evaluation,
                    output,
                    workers=workers,
                    timeout_seconds=timeout_seconds,
                    resume=resume,
                    limit=limit,
                    batch_size=batch_size,
                    package_root_override=package_root_override,
                )
                formal_receipt = Path(completed) / "full_release_receipt.json"
                if formal_receipt.is_symlink() or formal_receipt.exists():
                    raise RuntimeError(
                        "base runner unexpectedly created a formal v4 receipt"
                    )
                manifest = v2._load_json(Path(completed) / "manifest.json")
                _manifest_bindings(manifest)
                _verify_frozen_sources(source_hashes)
                _assert_orchestrator_simulator_free()
                _verify_child_runtime_unchanged()
                _manifest_bindings(manifest)
                _verify_frozen_sources(source_hashes)
                _assert_orchestrator_simulator_free()
                _verify_child_runtime_unchanged()
                _publish_receipt(completed)
                return completed
            except BaseException:
                provisional_path.unlink(missing_ok=True)
                raise


def run_pva_table4_v4(
    source_evaluation: Path,
    output: Path,
    *,
    workers: int = 32,
    timeout_seconds: float = 600.0,
    resume: bool = False,
    limit: int | None = None,
    batch_size: int | None = None,
    package_root_override: Path | None = None,
    contact_registry: Path = DEFAULT_CONTACT_REGISTRY,
    child_python: Path = DEFAULT_CHILD_PYTHON,
) -> Path:
    with _output_lock(output) as locked_output:
        return _run_pva_table4_v4_locked(
            source_evaluation,
            locked_output,
            workers=workers,
            timeout_seconds=timeout_seconds,
            resume=resume,
            limit=limit,
            batch_size=batch_size,
            package_root_override=package_root_override,
            contact_registry=contact_registry,
            child_python=child_python,
        )


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
    parser.add_argument(
        "--contact-registry", type=Path, default=DEFAULT_CONTACT_REGISTRY
    )
    parser.add_argument("--child-python", type=Path, default=DEFAULT_CHILD_PYTHON)
    args = parser.parse_args(argv)
    try:
        output = run_pva_table4_v4(
            args.source_evaluation,
            args.output,
            workers=args.workers,
            timeout_seconds=args.timeout_seconds,
            resume=args.resume,
            limit=args.limit,
            batch_size=args.batch_size,
            package_root_override=args.package_root_override,
            contact_registry=args.contact_registry,
            child_python=args.child_python,
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
    "run_pva_table4_v4",
]
