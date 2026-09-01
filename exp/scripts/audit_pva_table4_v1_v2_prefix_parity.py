#!/usr/bin/env python3
"""Audit a read-only PV-A Table 4 v1/v2 result prefix.

The v2 protocol intentionally changes the sampled configuration space for
URDF mimic joints.  This audit therefore applies two different contracts:

* assets without mimic joints must retain every pre-registered record and
  state semantic field; and
* assets with mimic joints must retain source/static identity, while their
  strict, rest, single-joint, and Sobol outcomes are reported as migrations.

Receipt-sealed inputs are opened with ``mode=ro&immutable=1`` only after their
main-file SHA256 and durable-sidecar closure are verified. Active v2 prefix
inputs retain ``mode=ro``. All connections enforce ``PRAGMA query_only=ON``.
The output is deterministic for a fixed prefix and carries a canonical
content self-hash.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from functools import lru_cache
import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3
import stat
import sys
import tempfile
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET
import zlib


SCRIPT = Path(__file__).resolve()
EXP = SCRIPT.parent.parent
if str(SCRIPT.parent) not in sys.path:
    sys.path.insert(0, str(SCRIPT.parent))

import run_table4_full_release as table4


DEFAULT_V1_DATABASE = (
    EXP / "runtime/pva_table1234_full_release_20260826/evaluation/results.sqlite3"
)
DEFAULT_V2_DATABASE = Path(
    "/tmp/pva_table4_mimic_aware_full_release_20260827/results.sqlite3"
)

AUDIT_SCHEMA = "pva_table4_v1_v2_prefix_parity_audit_v2"
V1_DATABASE_SCHEMA = "pva_table1234_results_db_v1"
V2_DATABASE_SCHEMA = "pva_table4_mimic_aware_results_db_v1"
V1_RECEIPT_SCHEMA = "pva_table1234_full_release_receipt_v1"
V2_RECEIPT_SCHEMA = "pva_table4_mimic_aware_full_release_receipt_v1"
V2_ARTIFACT_MANIFEST_SCHEMA = "pva_table4_mimic_aware_artifacts_v1"
V1_MANIFEST_SCHEMA = "pva_table1234_full_release_run_v1"
V2_MANIFEST_SCHEMA = "pva_table4_mimic_aware_full_release_run_v2"
V2_PROTOCOL_ID = "urdf_sim_ready_table4_pva_full_release_v2"
V2_SAMPLING_PROTOCOL = "mimic_aware_independent_sampling_v2"
PACKAGE_ROOT_BINDING_SCHEMA = "pva_package_root_override_v1"
PACKAGE_ROOT_MAPPING_POLICY = "relative_prefix_substitution_v1"
PACKAGE_VERIFICATION = "frozen_package_files_sha256_v1"
SUCCESSFUL_WORKER_STATUSES = frozenset({"completed", "recovered"})
V1_RETAINED_INFRASTRUCTURE_STATUSES = frozenset({"parent_error", "timeout"})

# These fields describe the same source package or static model property in
# both protocols.  They must remain identical even when mimic sampling changes.
COMMON_STATIC_RECORD_FIELDS = (
    "category",
    "collision_mesh_references",
    "dataset",
    "dataset_id",
    "expected_movable_joints",
    "expected_primary_urdf_sha256",
    "geometry_kinds",
    "link_count",
    "missing_collision_mesh_references",
    "movable_dof_count",
    "native_collision_elements",
    "object_bbox_diagonal_m",
    "order",
    "package",
    "package_binding_sha256",
    "package_binding_verified",
    "primary_urdf_relative_path",
    "range_evaluable_dof_count",
    "urdf_path",
)

# The complete pre-registered common record projection for non-mimic assets.
# It is intentionally explicit: adding or removing a field changes the audit
# contract and therefore requires a schema/version review.
NON_MIMIC_RECORD_SEMANTIC_FIELDS = (
    "category",
    "collision_mesh_references",
    "collision_metric_status",
    "dataset",
    "dataset_id",
    "expected_movable_joints",
    "expected_primary_urdf_sha256",
    "geometry_kinds",
    "issues",
    "joint_single_sweep_cf_passed",
    "link_count",
    "load_success",
    "max_penetration_m",
    "max_penetration_normalized",
    "max_reset_readback_error",
    "measurement_complete",
    "missing_collision_mesh_references",
    "movable_dof_count",
    "multi_joint_sobol_cf",
    "native_collision_elements",
    "object_bbox_diagonal_m",
    "order",
    "package",
    "package_binding_sha256",
    "package_binding_verified",
    "primary_urdf_relative_path",
    "range_evaluable_dof_count",
    "rest_all_pair_cf",
    "rest_non_adjacent_cf",
    "rest_non_adjacent_free",
    "rest_state_executed",
    "rest_state_expected",
    "single_joint_sweep_cf",
    "single_non_adjacent_free",
    "single_state_executed",
    "single_state_expected",
    "sobol_non_adjacent_free",
    "sobol_state_executed",
    "sobol_state_expected",
    "state_records_count",
    "status",
    "strict_collision_pass",
    "unexecuted_state_count",
    "urdf_path",
)

NON_MIMIC_STATE_SEMANTIC_FIELDS = (
    "all_pair_contact_count",
    "all_pair_illegal_penetration_count",
    "all_pair_max_penetration_m",
    "category",
    "dataset",
    "dataset_id",
    "joint_name",
    "joint_values_sha256",
    "metric_max_penetration_m",
    "non_adjacent_contact_count",
    "non_adjacent_illegal_penetration_count",
    "non_adjacent_max_penetration_m",
    "order",
    "phase",
    "reset_readback_max_abs_error",
    "sample_index",
)

# These common record fields are expected to differ solely because v2 binds a
# new schema/protocol, execution plan, and state serialization identity.
ALLOWED_COMMON_RECORD_DIFFERENCES = (
    "input_identity_sha256",
    "protocol_id",
    "runtime_identity",
    "schema_version",
    "state_records_sha256",
)

# These fields exist only in the v2 record and make the new execution plan
# explicit.  They are not silently ignored; the contract publishes them here.
V2_ONLY_RECORD_FIELDS = (
    "evaluation_package_relative_path",
    "evaluation_urdf_relative_path",
    "execution_input_sha256",
    "independent_dof_count",
    "joint_sampling_plan_sha256",
    "mimic_joint_count",
    "package_root_binding_content_sha256",
    "range_evaluable_independent_dof_count",
    "roster_ordinal",
    "sampling_plan_error",
    "sampling_protocol",
)

# On mimic assets only protocol-derived denominators and observations from the
# changed reachable states may differ.  Load/status/issues, package integrity,
# runtime bindings, rest coverage, and all static fields remain hard gates.
MIMIC_RECORD_FIELDS_ALLOWED_TO_CHANGE = (
    "joint_single_sweep_cf_passed",
    "max_penetration_m",
    "max_penetration_normalized",
    "max_reset_readback_error",
    "multi_joint_sobol_cf",
    "rest_all_pair_cf",
    "rest_non_adjacent_cf",
    "rest_non_adjacent_free",
    "single_joint_sweep_cf",
    "single_non_adjacent_free",
    "single_state_executed",
    "single_state_expected",
    "sobol_non_adjacent_free",
    "sobol_state_executed",
    "sobol_state_expected",
    "state_records_count",
    "strict_collision_pass",
    "unexecuted_state_count",
)
MIMIC_RECORD_FIELDS_REQUIRED_EQUAL = tuple(
    field
    for field in NON_MIMIC_RECORD_SEMANTIC_FIELDS
    if field not in MIMIC_RECORD_FIELDS_ALLOWED_TO_CHANGE
)

ALLOWED_COMMON_STATE_DIFFERENCES = (
    "input_identity_sha256",
    "protocol_id",
    "schema_version",
)
V2_ONLY_STATE_FIELDS = (
    "joint_sampling_plan_sha256",
    "sampling_protocol",
)

PHASE_FIELDS = {
    "strict": "strict_collision_pass",
    "rest": "rest_non_adjacent_cf",
    "single": "single_joint_sweep_cf",
    "sobol": "multi_joint_sobol_cf",
}
TRANSITION_KEYS = (
    "fail_to_fail",
    "fail_to_pass",
    "pass_to_fail",
    "pass_to_pass",
)


class ParityAuditError(ValueError):
    """Raised when an input cannot support a trustworthy prefix audit."""


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


def _json_value_equal(observed: Any, expected: Any) -> bool:
    """Compare JSON values without Python's bool/int equality aliasing."""
    return canonical_text(observed) == canonical_text(expected)


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
        raise ParityAuditError(f"{label} is unavailable: {candidate}: {error}") from error
    if candidate.is_symlink() or not resolved.is_file():
        raise ParityAuditError(f"{label} must be a regular non-symlink file: {candidate}")
    return resolved


def _load_json_file(path: Path, label: str) -> tuple[dict[str, Any], Path, str]:
    resolved = _regular_file(path, label)
    try:
        value = json.loads(resolved.read_text(encoding="utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ParityAuditError(f"{label} is not valid JSON: {error}") from error
    if not isinstance(value, dict):
        raise ParityAuditError(f"{label} must contain a JSON object")
    return value, resolved, sha256_file(resolved)


def _verify_self_hash(value: Mapping[str, Any], field: str, label: str) -> str:
    declared = value.get(field)
    if not isinstance(declared, str) or len(declared) != 64:
        raise ParityAuditError(f"{label} has no valid {field}")
    payload = dict(value)
    payload.pop(field, None)
    calculated = canonical_sha256(payload)
    if calculated != declared:
        raise ParityAuditError(
            f"{label} self-hash mismatch: declared {declared}, calculated {calculated}"
        )
    return declared


def _resolve_bound_file(root: Path, value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ParityAuditError(f"{label} path is missing")
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = root / candidate
    return _regular_file(candidate, label)


def _plain_absolute_directory(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ParityAuditError(f"{label} is missing")
    path = Path(value)
    if not path.is_absolute() or ".." in path.parts:
        raise ParityAuditError(f"{label} is not a safe absolute path")
    current = Path(path.anchor)
    for component in path.parts[1:]:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise ParityAuditError(f"{label} is unavailable: {path}") from error
        if stat.S_ISLNK(info.st_mode):
            raise ParityAuditError(f"{label} contains a symlink: {path}")
    if path.resolve(strict=True) != path or not path.is_dir():
        raise ParityAuditError(f"{label} is not a plain directory: {path}")
    return path


def _plain_descendant_file(path: Path, root: Path, label: str) -> Path:
    try:
        relative = path.relative_to(root)
    except ValueError as error:
        raise ParityAuditError(f"{label} escapes the physical mirror") from error
    if relative == Path(".") or ".." in relative.parts:
        raise ParityAuditError(f"{label} has an unsafe relative path")
    current = root
    for component in relative.parts:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise ParityAuditError(f"{label} is unavailable: {path}") from error
        if stat.S_ISLNK(info.st_mode):
            raise ParityAuditError(f"{label} contains a symlink: {path}")
    if path.resolve(strict=True) != path or not stat.S_ISREG(path.lstat().st_mode):
        raise ParityAuditError(f"{label} is not a plain regular file: {path}")
    return path


def _verify_v1_sidecars(database: Path) -> dict[str, Any]:
    """Reject durable SQLite sidecar content before trusting the sealed DB hash.

    A zero-byte ``-wal`` or ``-journal`` is accepted and reported.  A ``-shm``
    file is SQLite's volatile WAL index rather than database content; it is
    accepted only when the corresponding WAL exists and is zero bytes.  Every
    accepted sidecar must be a plain regular non-symlink file.
    """

    result: dict[str, Any] = {
        "policy": (
            "non-empty -wal/-journal rejected; zero-byte durable sidecars "
            "accepted; -shm accepted only with a zero-byte -wal"
        ),
        "observed": {},
    }
    observed: dict[str, Any] = result["observed"]
    wal = Path(str(database) + "-wal")
    journal = Path(str(database) + "-journal")
    shm = Path(str(database) + "-shm")
    for label, path in (("wal", wal), ("journal", journal)):
        if not path.exists():
            observed[label] = {"exists": False, "bytes": 0}
            continue
        if path.is_symlink() or not stat.S_ISREG(path.lstat().st_mode):
            raise ParityAuditError(f"v1 SQLite {label} sidecar is not a plain file")
        size = path.stat().st_size
        if size != 0:
            raise ParityAuditError(
                f"v1 SQLite {label} sidecar is non-empty ({size} bytes)"
            )
        observed[label] = {"exists": True, "bytes": 0, "sha256": sha256_file(path)}
    if shm.exists():
        if shm.is_symlink() or not stat.S_ISREG(shm.lstat().st_mode):
            raise ParityAuditError("v1 SQLite shm sidecar is not a plain file")
        if not wal.exists() or wal.stat().st_size != 0:
            raise ParityAuditError(
                "v1 SQLite shm sidecar is not paired with a zero-byte WAL"
            )
        observed["shm"] = {
            "exists": True,
            "bytes": shm.stat().st_size,
            "sha256": sha256_file(shm),
            "content_role": "volatile WAL index; not durable database content",
        }
    else:
        observed["shm"] = {"exists": False, "bytes": 0}
    return result


def _v2_sidecars(database: Path, *, sealed: bool) -> dict[str, Any]:
    result: dict[str, Any] = {
        "policy": (
            "sealed receipt requires empty/absent -wal and -journal; zero-byte WAL "
            "may retain a volatile -shm; active prefix snapshots may have sidecars"
        ),
        "sealed_policy_applied": sealed,
        "observed": {},
    }
    wal_size = 0
    wal_exists = False
    for suffix, label in (("-wal", "wal"), ("-journal", "journal")):
        path = Path(str(database) + suffix)
        entry: dict[str, Any] = {"exists": path.exists(), "bytes": 0}
        if path.exists():
            info = path.lstat()
            if not stat.S_ISREG(info.st_mode) or path.is_symlink():
                raise ParityAuditError(f"v2 SQLite {label} sidecar is not a plain file")
            entry["bytes"] = info.st_size
            entry["sha256"] = sha256_file(path)
            if sealed and info.st_size:
                raise ParityAuditError(f"sealed v2 SQLite {label} sidecar is non-empty")
        result["observed"][label] = entry
        if label == "wal":
            wal_exists, wal_size = path.exists(), int(entry["bytes"])
    shm = Path(str(database) + "-shm")
    shm_entry: dict[str, Any] = {"exists": shm.exists(), "bytes": 0}
    if shm.exists():
        info = shm.lstat()
        if not stat.S_ISREG(info.st_mode) or shm.is_symlink():
            raise ParityAuditError("v2 SQLite shm sidecar is not a plain file")
        if sealed and (not wal_exists or wal_size != 0):
            raise ParityAuditError("sealed v2 SQLite shm is not paired with a zero-byte WAL")
        shm_entry.update({
            "bytes": info.st_size,
            "sha256": sha256_file(shm),
            "content_role": "volatile WAL index; not durable database content",
        })
    result["observed"]["shm"] = shm_entry
    result["sealed_sidecar_policy_satisfied"] = bool(
        sealed and wal_size == 0 and int(result["observed"]["journal"]["bytes"]) == 0
    )
    return result


def _verify_v2_receipt(
    database: Path,
    manifest_path: Path,
    manifest: Mapping[str, Any],
    manifest_file_sha: str,
    manifest_content_sha: str,
    requested_receipt: Path | None,
    preverified_database_sha: str | None = None,
) -> dict[str, Any]:
    discovered = database.parent / "full_release_receipt.json"
    receipt_path = Path(requested_receipt) if requested_receipt is not None else discovered
    if requested_receipt is None and not receipt_path.exists():
        return {
            "v2_receipt_verified": False,
            "v2_receipt_path": None,
            "v2_receipt_auto_discovered": False,
            "v2_database_full_file_sha256": None,
            "v2_sidecars": _v2_sidecars(database, sealed=False),
        }
    receipt, receipt_path, receipt_file_sha = _load_json_file(
        receipt_path, "v2 full-release receipt"
    )
    if receipt.get("schema_version") != V2_RECEIPT_SCHEMA:
        raise ParityAuditError("unexpected v2 full-release receipt schema")
    receipt_content_sha = _verify_self_hash(
        receipt, "receipt_content_sha256", "v2 full-release receipt"
    )
    expected_paths = {
        "manifest": "manifest.json",
        "records": "records.jsonl",
        "state_records": "state_records.jsonl",
        "summary": "summary.json",
        "artifact_manifest": "artifact_manifest.json",
        "result_database": "results.sqlite3",
    }
    if any(receipt.get(field) != name for field, name in expected_paths.items()):
        raise ParityAuditError("v2 receipt artifact path semantics mismatch")
    sidecars = _v2_sidecars(database, sealed=True)
    if (
        receipt.get("protocol_id") != V2_PROTOCOL_ID
        or receipt.get("sampling_protocol") != V2_SAMPLING_PROTOCOL
        or not _json_value_equal(receipt.get("N_eval"), manifest.get("N_eval"))
        or not _json_value_equal(receipt.get("J_eval"), manifest.get("J_eval"))
        or not _json_value_equal(
            receipt.get("category_count"), manifest.get("category_count")
        )
        or receipt.get("classification") != manifest.get("classification")
        or not _json_value_equal(
            receipt.get("package_root_binding"), manifest.get("package_root_binding")
        )
        or not _json_value_equal(receipt.get("source"), manifest.get("source"))
    ):
        raise ParityAuditError("v2 receipt protocol/denominator/source binding mismatch")
    if receipt.get("status") not in {"COMPLETE", "COMPLETE_WITH_RETAINED_FAILURES"}:
        raise ParityAuditError("v2 receipt has an invalid full-release status")
    bound_manifest = _resolve_bound_file(
        receipt_path.parent, receipt.get("manifest"), "v2 receipt manifest"
    )
    if bound_manifest != manifest_path or receipt.get("manifest_sha256") != manifest_file_sha:
        raise ParityAuditError("v2 receipt manifest file binding mismatch")
    if manifest.get("manifest_content_sha256") != manifest_content_sha:
        raise ParityAuditError("v2 receipt manifest content chain mismatch")
    bound_database = _resolve_bound_file(
        receipt_path.parent, receipt.get("result_database"), "v2 receipt database"
    )
    if bound_database != database:
        raise ParityAuditError("v2 receipt database path does not bind the input")
    database_sha = preverified_database_sha or sha256_file(database)
    if receipt.get("result_database_sha256") != database_sha:
        raise ParityAuditError("v2 database SHA256 differs from the sealed receipt")

    artifact_path = _resolve_bound_file(
        receipt_path.parent, receipt.get("artifact_manifest"), "v2 artifact manifest"
    )
    artifact, _artifact_path, artifact_file_sha = _load_json_file(
        artifact_path, "v2 artifact manifest"
    )
    if artifact.get("schema_version") != V2_ARTIFACT_MANIFEST_SCHEMA:
        raise ParityAuditError("unexpected v2 artifact manifest schema")
    artifact_content_sha = _verify_self_hash(
        artifact, "artifact_manifest_content_sha256", "v2 artifact manifest"
    )
    if receipt.get("artifact_manifest_sha256") != artifact_file_sha:
        raise ParityAuditError("v2 receipt artifact-manifest SHA256 mismatch")
    rows = artifact.get("artifacts")
    if not isinstance(rows, list):
        raise ParityAuditError("v2 artifact manifest has no artifact list")
    entries: dict[str, Mapping[str, Any]] = {}
    for row in rows:
        if not isinstance(row, Mapping) or not isinstance(row.get("path"), str):
            raise ParityAuditError("v2 artifact manifest contains an invalid row")
        name = str(row["path"])
        if name in entries:
            raise ParityAuditError(f"v2 artifact manifest duplicates {name}")
        entries[name] = row
    required = {
        "manifest.json", "protocol_snapshot.md", "records.jsonl",
        "asset_records.jsonl", "state_records.jsonl", "summary.json",
        "summary.md", "checkpoint.json", "results.sqlite3",
    }
    receipt_bindings = {
        "manifest.json": "manifest_sha256",
        "records.jsonl": "records_sha256",
        "state_records.jsonl": "state_records_sha256",
        "summary.json": "summary_sha256",
        "results.sqlite3": "result_database_sha256",
    }
    if not required.issubset(entries):
        raise ParityAuditError("v2 artifact manifest is missing required sealed files")
    for name, row in entries.items():
        relative = Path(name)
        if relative.is_absolute() or ".." in relative.parts:
            raise ParityAuditError(f"v2 artifact manifest path is unsafe: {name}")
        path = _resolve_bound_file(
            receipt_path.parent, name, f"v2 sealed artifact {name}"
        )
        if (
            not _json_value_equal(row.get("bytes"), path.stat().st_size)
            or row.get("sha256") != sha256_file(path)
        ):
            raise ParityAuditError(f"v2 sealed artifact binding mismatch: {name}")
    receipt_paths = {
        "records.jsonl": "records",
        "state_records.jsonl": "state_records",
        "summary.json": "summary",
    }
    for name, receipt_path_field in receipt_paths.items():
        if _resolve_bound_file(
            receipt_path.parent,
            receipt.get(receipt_path_field),
            f"v2 receipt {receipt_path_field}",
        ) != receipt_path.parent / name:
            raise ParityAuditError(f"v2 receipt path binding mismatch: {receipt_path_field}")
    for name, receipt_field in receipt_bindings.items():
        row = entries[name]
        if row.get("sha256") != receipt.get(receipt_field):
            raise ParityAuditError(f"v2 sealed artifact binding mismatch: {name}")
    summary, _summary_path, _summary_sha = _load_json_file(
        receipt_path.parent / "summary.json", "v2 sealed summary"
    )
    if summary.get("status") != receipt.get("status"):
        raise ParityAuditError("v2 receipt/summary status binding mismatch")
    return {
        "v2_receipt_verified": True,
        "v2_receipt_path": receipt_path,
        "v2_receipt_file_sha256": receipt_file_sha,
        "v2_receipt_content_sha256": receipt_content_sha,
        "v2_receipt_auto_discovered": requested_receipt is None,
        "v2_database_full_file_sha256": database_sha,
        "v2_artifact_manifest_path": artifact_path,
        "v2_artifact_manifest_file_sha256": artifact_file_sha,
        "v2_artifact_manifest_content_sha256": artifact_content_sha,
        "v2_sidecars": sidecars,
    }


def _receipt_database_preflight(
    database: Path,
    receipt_path: Path,
    *,
    receipt_schema: str,
    label: str,
    v1: bool,
) -> str:
    """Verify a sealed main-file snapshot before an immutable SQLite open."""
    receipt, resolved_receipt, _receipt_sha = _load_json_file(
        receipt_path, f"{label} receipt"
    )
    if receipt.get("schema_version") != receipt_schema:
        raise ParityAuditError(f"unexpected {label} receipt schema")
    _verify_self_hash(receipt, "receipt_content_sha256", f"{label} receipt")
    bound = _resolve_bound_file(
        resolved_receipt.parent,
        receipt.get("result_database"),
        f"{label} receipt database",
    )
    if bound != database:
        raise ParityAuditError(f"{label} receipt database path does not bind the input")
    if v1:
        _verify_v1_sidecars(database)
    else:
        _v2_sidecars(database, sealed=True)
    observed_sha = sha256_file(database)
    if receipt.get("result_database_sha256") != observed_sha:
        raise ParityAuditError(f"{label} database SHA256 differs from the sealed receipt")
    return observed_sha


def connect_read_only(
    path: Path, label: str, *, immutable: bool = False
) -> tuple[sqlite3.Connection, Path]:
    resolved = _regular_file(path, label)
    query = "mode=ro&immutable=1" if immutable else "mode=ro"
    connection = sqlite3.connect(f"{resolved.as_uri()}?{query}", uri=True)
    try:
        connection.execute("PRAGMA query_only = ON")
        query_only = connection.execute("PRAGMA query_only").fetchone()
        if query_only is None or int(query_only[0]) != 1:
            raise ParityAuditError(f"failed to enforce query_only on {label}")
        connection.execute("BEGIN")
    except BaseException:
        connection.close()
        raise
    return connection, resolved


def _table_columns(connection: sqlite3.Connection, table: str) -> set[str]:
    rows = connection.execute(f"PRAGMA table_info({table})").fetchall()
    return {str(row[1]) for row in rows}


def _require_columns(
    connection: sqlite3.Connection,
    table: str,
    required: Iterable[str],
    label: str,
) -> None:
    observed = _table_columns(connection, table)
    missing = sorted(set(required) - observed)
    if missing:
        raise ParityAuditError(
            f"{label}.{table} is missing columns: {', '.join(missing)}"
        )


def _read_meta(connection: sqlite3.Connection, label: str) -> dict[str, Any]:
    _require_columns(connection, "meta", ("key", "value"), label)
    result: dict[str, Any] = {}
    for key, raw_value in connection.execute("SELECT key, value FROM meta ORDER BY key"):
        if str(key) in result:
            raise ParityAuditError(f"{label}.meta contains duplicate key {key!r}")
        try:
            result[str(key)] = json.loads(str(raw_value))
        except json.JSONDecodeError as error:
            raise ParityAuditError(
                f"{label}.meta[{key!r}] is not canonical JSON: {error}"
            ) from error
    return result


def _verify_evidence_chain(
    v1_path: Path,
    v2_path: Path,
    v1_meta: Mapping[str, Any],
    v2_meta: Mapping[str, Any],
    *,
    v1_receipt_path: Path | None,
    v2_manifest_path: Path | None,
    v2_receipt_path: Path | None,
    preverified_v1_database_sha: str | None = None,
    preverified_v2_database_sha: str | None = None,
) -> dict[str, Any]:
    receipt_path = (
        v1_path.parent / "full_release_receipt.json"
        if v1_receipt_path is None
        else Path(v1_receipt_path)
    )
    receipt, receipt_path, receipt_file_sha = _load_json_file(
        receipt_path, "v1 full-release receipt"
    )
    if receipt.get("schema_version") != V1_RECEIPT_SCHEMA:
        raise ParityAuditError("unexpected v1 full-release receipt schema")
    receipt_content_sha = _verify_self_hash(
        receipt, "receipt_content_sha256", "v1 full-release receipt"
    )
    bound_v1_database = _resolve_bound_file(
        receipt_path.parent, receipt.get("result_database"), "v1 receipt database"
    )
    if bound_v1_database != v1_path:
        raise ParityAuditError("v1 receipt database path does not bind the input")
    _verify_v1_sidecars(v1_path)
    v1_database_sha = preverified_v1_database_sha or sha256_file(v1_path)
    v1_sidecars = _verify_v1_sidecars(v1_path)
    if receipt.get("result_database_sha256") != v1_database_sha:
        raise ParityAuditError("v1 database SHA256 differs from the sealed receipt")
    if not _json_value_equal(
        receipt.get("result_database_bytes"), v1_path.stat().st_size
    ):
        raise ParityAuditError("v1 database byte count differs from the sealed receipt")

    v1_manifest_path = _resolve_bound_file(
        receipt_path.parent,
        receipt.get("execution_manifest"),
        "v1 execution manifest",
    )
    v1_manifest, _resolved, v1_manifest_file_sha = _load_json_file(
        v1_manifest_path, "v1 execution manifest"
    )
    if v1_manifest.get("schema_version") != V1_MANIFEST_SCHEMA:
        raise ParityAuditError("unexpected v1 execution manifest schema")
    v1_manifest_content_sha = _verify_self_hash(
        v1_manifest, "manifest_content_sha256", "v1 execution manifest"
    )
    if receipt.get("execution_manifest_sha256") != v1_manifest_file_sha:
        raise ParityAuditError("v1 receipt execution-manifest SHA256 mismatch")

    roster_path = _resolve_bound_file(
        receipt_path.parent, receipt.get("roster_manifest"), "v1 roster manifest"
    )
    roster, _resolved, roster_file_sha = _load_json_file(
        roster_path, "v1 roster manifest"
    )
    roster_content_sha = _verify_self_hash(
        roster, "manifest_content_sha256", "v1 roster manifest"
    )
    if (
        receipt.get("roster_manifest_sha256") != roster_file_sha
        or receipt.get("roster_manifest_content_sha256") != roster_content_sha
        or v1_meta.get("roster_manifest_sha256") != roster_file_sha
        or v1_meta.get("roster_manifest_content_sha256") != roster_content_sha
    ):
        raise ParityAuditError("v1 roster binding mismatch")

    for label, value in (
        ("N_eval", receipt.get("N_eval")),
        ("J_eval", receipt.get("J_eval")),
    ):
        v1_key = "selected_n" if label == "N_eval" else "selected_j"
        if (
            not _json_value_equal(value, v1_meta.get(v1_key))
            or not _json_value_equal(value, v1_manifest.get(label))
        ):
            raise ParityAuditError(f"v1 receipt/manifest/database mismatch: {label}")

    manifest_path = (
        v2_path.parent / "manifest.json"
        if v2_manifest_path is None
        else Path(v2_manifest_path)
    )
    v2_manifest, manifest_path, v2_manifest_file_sha = _load_json_file(
        manifest_path, "v2 execution manifest"
    )
    if v2_manifest.get("schema_version") != V2_MANIFEST_SCHEMA:
        raise ParityAuditError("unexpected v2 execution manifest schema")
    v2_manifest_content_sha = _verify_self_hash(
        v2_manifest, "manifest_content_sha256", "v2 execution manifest"
    )
    if (
        v2_manifest.get("protocol_id") != V2_PROTOCOL_ID
        or v2_manifest.get("sampling_protocol") != V2_SAMPLING_PROTOCOL
    ):
        raise ParityAuditError("v2 manifest protocol binding mismatch")
    protocol = v2_manifest.get("protocol")
    expected_protocol = {
        "single_joint_samples": table4.SINGLE_SAMPLES,
        "sobol_samples": table4.SOBOL_SAMPLES,
        "sobol_seed": table4.SOBOL_SEED,
        "penetration_threshold_m": table4.PENETRATION_THRESHOLD_M,
        "mimic_constraints": "affine_expansion_from_independent_roots",
        "contact_policy": "all_non_direct-parent_pairs",
    }
    if not isinstance(protocol, Mapping) or any(
        not _json_value_equal(protocol.get(key), expected)
        for key, expected in expected_protocol.items()
    ):
        raise ParityAuditError("v2 manifest frozen protocol constants mismatch")

    source = v2_manifest.get("source")
    if not isinstance(source, Mapping):
        raise ParityAuditError("v2 manifest source binding is missing")
    if any(
        not _json_value_equal(value.get(field), receipt.get(field))
        for value in (v2_manifest, source)
        for field in ("N_eval", "J_eval")
    ):
        raise ParityAuditError("v2 manifest/source N_eval or J_eval binding mismatch")
    if (
        _resolve_bound_file(
            manifest_path.parent,
            source.get("source_receipt"),
            "v2 source receipt",
        )
        != receipt_path
        or source.get("source_receipt_sha256") != receipt_file_sha
        or source.get("source_receipt_content_sha256") != receipt_content_sha
        or _resolve_bound_file(
            manifest_path.parent,
            source.get("source_result_database"),
            "v2 source database",
        )
        != v1_path
        or source.get("source_result_database_declared_sha256") != v1_database_sha
        or _resolve_bound_file(
            manifest_path.parent,
            source.get("roster_manifest"),
            "v2 source roster",
        )
        != roster_path
        or source.get("roster_manifest_sha256") != roster_file_sha
        or source.get("roster_manifest_content_sha256") != roster_content_sha
    ):
        raise ParityAuditError("v2 manifest sealed source binding mismatch")

    expected_meta = {
        "schema_version": V2_DATABASE_SCHEMA,
        "manifest_content_sha256": v2_manifest_content_sha,
        "source_receipt_content_sha256": receipt_content_sha,
        "source_result_database_declared_sha256": v1_database_sha,
        "roster_manifest_content_sha256": roster_content_sha,
        "N_eval": receipt["N_eval"],
        "J_eval": receipt["J_eval"],
        "limit": v2_manifest.get("limit"),
        "sampling_protocol": V2_SAMPLING_PROTOCOL,
        "protocol_id": V2_PROTOCOL_ID,
    }

    binding = v2_manifest.get("package_root_binding")
    if not isinstance(binding, Mapping):
        raise ParityAuditError("v2 manifest package-root binding is missing")
    expected_binding_fields = {
        "schema_version",
        "mapping_policy",
        "logical_root",
        "physical_root",
        "roster_manifest_content_sha256",
        "package_verification",
        "binding_content_sha256",
    }
    if set(binding) != expected_binding_fields:
        raise ParityAuditError("v2 package-root binding fields mismatch")
    if (
        binding.get("schema_version") != PACKAGE_ROOT_BINDING_SCHEMA
        or binding.get("mapping_policy") != PACKAGE_ROOT_MAPPING_POLICY
        or binding.get("package_verification") != PACKAGE_VERIFICATION
        or binding.get("roster_manifest_content_sha256") != roster_content_sha
    ):
        raise ParityAuditError("v2 package-root binding policy mismatch")
    binding_content_sha = _verify_self_hash(
        binding, "binding_content_sha256", "v2 package-root binding"
    )
    source_bindings = roster.get("source_bindings")
    if (
        not isinstance(source_bindings, Mapping)
        or binding.get("logical_root") != source_bindings.get("extracted_root")
    ):
        raise ParityAuditError("v2 logical mirror root differs from the sealed roster")
    logical_root = Path(str(binding["logical_root"]))
    if not logical_root.is_absolute() or ".." in logical_root.parts:
        raise ParityAuditError("v2 logical package root is unsafe")
    physical_root = _plain_absolute_directory(
        binding.get("physical_root"), "v2 physical package root"
    )
    expected_meta["package_root_binding_content_sha256"] = binding_content_sha
    for key, expected in expected_meta.items():
        if not _json_value_equal(v2_meta.get(key), expected):
            raise ParityAuditError(f"v2 manifest/database meta mismatch: {key}")

    source_hashes = v2_manifest.get("source_hashes")
    runtime = v2_manifest.get("runtime_identity")
    if not isinstance(source_hashes, Mapping) or not isinstance(runtime, Mapping):
        raise ParityAuditError("v2 source/runtime binding is missing")
    bound_sources = {
        "adapter": EXP / "scripts/run_pva_table4_mimic_aware_full_release.py",
        "pva_roster_verifier": EXP / "scripts/build_pva_full_release_roster.py",
        "table4_core": Path(table4.CORE_SCRIPT),
        "table4_runner": EXP / "scripts/run_table4_full_release.py",
    }
    observed_source_hashes = {
        key: sha256_file(_regular_file(path, f"v2 bound source {key}"))
        for key, path in bound_sources.items()
    }
    if any(source_hashes.get(key) != digest for key, digest in observed_source_hashes.items()):
        raise ParityAuditError("v2 frozen source hash binding mismatch")
    if (
        runtime.get("collision_core_sha256") != observed_source_hashes["table4_core"]
        or runtime.get("runner_sha256") != observed_source_hashes["table4_runner"]
    ):
        raise ParityAuditError("v2 runtime source binding mismatch")
    pybullet_path = _regular_file(
        Path(str(runtime.get("pybullet_module", ""))), "v2 PyBullet module"
    )
    pybullet_sha = sha256_file(pybullet_path)
    if runtime.get("pybullet_module_sha256") != pybullet_sha:
        raise ParityAuditError("v2 PyBullet runtime hash mismatch")
    import scipy

    if runtime.get("scipy_version") != scipy.__version__:
        raise ParityAuditError("v2 SciPy runtime version mismatch")

    v2_receipt_info = _verify_v2_receipt(
        v2_path,
        manifest_path,
        v2_manifest,
        v2_manifest_file_sha,
        v2_manifest_content_sha,
        v2_receipt_path,
        preverified_database_sha=preverified_v2_database_sha,
    )

    return {
        "v1_receipt": receipt,
        "v1_receipt_path": receipt_path,
        "v1_receipt_file_sha256": receipt_file_sha,
        "v1_manifest": v1_manifest,
        "v1_manifest_path": v1_manifest_path,
        "v1_manifest_file_sha256": v1_manifest_file_sha,
        "v1_manifest_content_sha256": v1_manifest_content_sha,
        "v1_database_sha256": v1_database_sha,
        "v1_sidecars": v1_sidecars,
        "roster_path": roster_path,
        "roster_file_sha256": roster_file_sha,
        "roster_content_sha256": roster_content_sha,
        "v2_manifest": v2_manifest,
        "v2_manifest_path": manifest_path,
        "v2_manifest_file_sha256": v2_manifest_file_sha,
        "v2_manifest_content_sha256": v2_manifest_content_sha,
        "binding": dict(binding),
        "binding_content_sha256": binding_content_sha,
        "logical_root": logical_root,
        "physical_root": physical_root,
        "v2_runtime_identity": dict(runtime),
        "v1_runtime_identity": dict(v1_manifest.get("runtime_identity", {})),
        "observed_source_hashes": observed_source_hashes,
        "pybullet_module_sha256": pybullet_sha,
        **v2_receipt_info,
    }


def _integer(value: Any, label: str, *, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ParityAuditError(f"{label} must be an integer >= {minimum}")
    return value


def _json_object(raw: Any, label: str) -> dict[str, Any]:
    try:
        value = json.loads(str(raw))
    except json.JSONDecodeError as error:
        raise ParityAuditError(f"{label} is not valid JSON: {error}") from error
    if not isinstance(value, dict):
        raise ParityAuditError(f"{label} must contain a JSON object")
    return value


def _project(
    value: Mapping[str, Any], fields: Sequence[str], label: str
) -> dict[str, Any]:
    missing = [field for field in fields if field not in value]
    if missing:
        raise ParityAuditError(f"{label} is missing fields: {', '.join(missing)}")
    return {field: value[field] for field in fields}


def _strict_field_differences(
    left: Mapping[str, Any], right: Mapping[str, Any], fields: Sequence[str]
) -> list[str]:
    return [
        field
        for field in fields
        if not _json_value_equal(left[field], right[field])
    ]


def _collision_core() -> Any:
    core = getattr(_collision_core, "_cached", None)
    if core is None:
        core = table4._core()
        setattr(_collision_core, "_cached", core)
    return core


def _mapped_urdf_and_plan(
    roster_row: Mapping[str, Any],
    evidence: Mapping[str, Any],
    ordinal: int,
) -> tuple[dict[str, Any], dict[str, Any]]:
    logical_root = Path(evidence["logical_root"])
    physical_root = Path(evidence["physical_root"])
    logical_package = Path(str(roster_row.get("source_path", "")))
    logical_urdf = Path(str(roster_row.get("primary_urdf_path", "")))
    if (
        not logical_package.is_absolute()
        or not logical_urdf.is_absolute()
        or ".." in logical_package.parts
        or ".." in logical_urdf.parts
    ):
        raise ParityAuditError(f"ordinal {ordinal} has unsafe frozen logical paths")
    try:
        package_relative = logical_package.relative_to(logical_root)
        urdf_relative = logical_urdf.relative_to(logical_package)
    except ValueError as error:
        raise ParityAuditError(
            f"ordinal {ordinal} escapes the frozen logical package root"
        ) from error
    if (
        package_relative == Path(".")
        or urdf_relative == Path(".")
        or ".." in package_relative.parts
        or ".." in urdf_relative.parts
    ):
        raise ParityAuditError(f"ordinal {ordinal} has an unsafe mirror mapping")
    physical_urdf = _plain_descendant_file(
        physical_root / package_relative / urdf_relative,
        physical_root,
        f"ordinal {ordinal} physical mirror URDF",
    )
    expected_urdf_sha = str(roster_row.get("primary_urdf_sha256", ""))
    observed_urdf_sha = sha256_file(physical_urdf)
    if observed_urdf_sha != expected_urdf_sha:
        raise ParityAuditError(f"ordinal {ordinal} physical mirror URDF hash drift")

    core = _collision_core()
    declared_dof = _integer(
        roster_row.get("joint_count"), f"roster ordinal {ordinal}.joint_count"
    )
    try:
        joints = core.parse_urdf_joints(physical_urdf)
        if len(joints) != declared_dof:
            raise ValueError("declared joint count mismatch")
        compiled = core.compile_joint_sampling_plan(joints)
        plan = {
            "independent_dof_count": int(compiled["independent_dof_count"]),
            "range_evaluable_independent_dof_count": int(
                compiled["range_evaluable_independent_dof_count"]
            ),
            "mimic_joint_count": int(compiled["mimic_joint_count"]),
            "joint_sampling_plan_sha256": str(compiled["plan_sha256"]),
            "sampling_plan_error": None,
            "_compiled_plan": compiled,
        }
    except (OSError, ET.ParseError, TypeError, ValueError):
        metadata = core.sampling_plan_metadata(
            physical_urdf,
            declared_dof=declared_dof,
            expected_sha256=expected_urdf_sha,
        )
        plan = {
            "independent_dof_count": int(metadata["independent_dof_count"]),
            "range_evaluable_independent_dof_count": int(
                metadata["range_evaluable_independent_dof_count"]
            ),
            "mimic_joint_count": int(metadata["mimic_joint_count"]),
            "joint_sampling_plan_sha256": str(
                metadata["joint_sampling_plan_sha256"]
            ),
            "sampling_plan_error": metadata["sampling_plan_error"],
            "_compiled_plan": None,
        }
    paths = {
        "logical_package": str(logical_package),
        "logical_urdf": str(logical_urdf),
        "evaluation_package_relative_path": package_relative.as_posix(),
        "evaluation_urdf_relative_path": urdf_relative.as_posix(),
        "physical_urdf_sha256": observed_urdf_sha,
    }
    return paths, plan


def _expected_v2_input_identity(
    roster_row: Mapping[str, Any], plan: Mapping[str, Any]
) -> str:
    independent = int(plan["independent_dof_count"])
    logical_urdf = Path(str(roster_row["primary_urdf_path"])).resolve()
    values = {
        "dataset": "pva",
        "dataset_id": str(roster_row["asset_id"]),
        "category": str(
            roster_row.get("raw_category", roster_row.get("category", ""))
        ),
        "urdf_path": str(logical_urdf),
        "primary_urdf_relative_path": str(
            roster_row.get("primary_urdf_relative_path", "model.urdf")
        ),
        "expected_primary_urdf_sha256": roster_row.get("primary_urdf_sha256"),
        "expected_movable_joints": int(roster_row.get("joint_count", 0)),
        "package_binding_sha256": roster_row.get("package_binding_sha256"),
        "sampling_protocol": V2_SAMPLING_PROTOCOL,
        "independent_dof_count": independent,
        "range_evaluable_independent_dof_count": int(
            plan["range_evaluable_independent_dof_count"]
        ),
        "mimic_joint_count": int(plan["mimic_joint_count"]),
        "joint_sampling_plan_sha256": plan["joint_sampling_plan_sha256"],
        "single_state_expected": table4.SINGLE_SAMPLES * independent,
        "sobol_state_expected": table4.SOBOL_SAMPLES if independent else 0,
    }
    return canonical_sha256(values)


@lru_cache(maxsize=None)
def _sobol_unit(dimension: int) -> tuple[tuple[float, ...], ...]:
    from scipy.stats import qmc

    rows = qmc.Sobol(
        d=dimension, scramble=True, seed=table4.SOBOL_SEED
    ).random_base2(m=6)
    return tuple(tuple(float(value) for value in row) for row in rows)


def _expected_v2_schedule(plan: Mapping[str, Any]) -> list[dict[str, Any]]:
    compiled = plan.get("_compiled_plan")
    if not isinstance(compiled, Mapping):
        return []
    core = _collision_core()
    independent_rows = list(compiled["independent_joints"])
    dimension = len(independent_rows)
    schedule: list[dict[str, Any]] = []

    def append(
        phase: str,
        sample_index: int,
        joint_name: str | None,
        independent_values: Sequence[float],
    ) -> None:
        expanded = core.expand_joint_values(
            compiled, [float(value) for value in independent_values]
        )
        schedule.append(
            {
                "phase": phase,
                "sample_index": sample_index,
                "joint_name": joint_name,
                "joint_values_sha256": canonical_sha256(expanded),
            }
        )

    append("rest", 0, None, [0.0] * dimension)
    for position, joint in enumerate(independent_rows):
        if not bool(joint.get("sampling_range_evaluable")):
            continue
        for sample_index, value in enumerate(core.single_joint_values(joint)):
            independent = [0.0] * dimension
            independent[position] = float(value)
            append(
                "single_joint_sweep",
                sample_index,
                str(joint["name"]),
                independent,
            )
    if independent_rows and all(
        bool(joint.get("sampling_range_evaluable")) for joint in independent_rows
    ):
        intervals = [
            (float(joint["sampling_lower"]), float(joint["sampling_upper"]))
            for joint in independent_rows
        ]
        for sample_index, unit in enumerate(_sobol_unit(dimension)):
            independent = [
                lower + scalar * (upper - lower)
                for scalar, (lower, upper) in zip(unit, intervals, strict=True)
            ]
            append("multi_joint_sobol", sample_index, None, independent)
    return schedule


def _validate_v2_schedule(
    record: Mapping[str, Any],
    states: Sequence[Mapping[str, Any]],
    plan: Mapping[str, Any],
    ordinal: int,
) -> None:
    schedule = _expected_v2_schedule(plan)
    if len(states) > len(schedule):
        raise ParityAuditError(
            f"v2 ordinal {ordinal} contains states beyond the recomputed schedule"
        )
    for state_index, state in enumerate(states):
        expected = schedule[state_index]
        for field, expected_value in expected.items():
            observed = state.get(field)
            if field == "sample_index" and (
                isinstance(observed, bool) or not isinstance(observed, int)
            ):
                raise ParityAuditError(
                    f"v2 ordinal {ordinal} state {state_index} sample_index is invalid"
                )
            if not _json_value_equal(observed, expected_value):
                raise ParityAuditError(
                    f"v2 ordinal {ordinal} state {state_index} schedule mismatch: {field}"
                )

    expected_counts = {
        "rest": _integer(
            record.get("rest_state_expected"),
            f"v2 ordinal {ordinal}.rest_state_expected",
        ),
        "single_joint_sweep": _integer(
            record.get("single_state_expected"),
            f"v2 ordinal {ordinal}.single_state_expected",
        ),
        "multi_joint_sobol": _integer(
            record.get("sobol_state_expected"),
            f"v2 ordinal {ordinal}.sobol_state_expected",
        ),
    }
    executed_fields = {
        "rest": "rest_state_executed",
        "single_joint_sweep": "single_state_executed",
        "multi_joint_sobol": "sobol_state_executed",
    }
    observed_counts = Counter(str(state.get("phase")) for state in states)
    for phase, field in executed_fields.items():
        executed = _integer(record.get(field), f"v2 ordinal {ordinal}.{field}")
        if executed != observed_counts.get(phase, 0):
            raise ParityAuditError(
                f"v2 ordinal {ordinal} state execution coverage mismatch: {phase}"
            )
    expected_total = sum(expected_counts.values())
    unexecuted = _integer(
        record.get("unexecuted_state_count"),
        f"v2 ordinal {ordinal}.unexecuted_state_count",
    )
    if unexecuted != expected_total - len(states):
        raise ParityAuditError(
            f"v2 ordinal {ordinal} unexecuted state accounting mismatch"
        )

    measurement_complete = record.get("measurement_complete")
    if not isinstance(measurement_complete, bool):
        raise ParityAuditError(
            f"v2 ordinal {ordinal} measurement_complete is not boolean"
        )
    if measurement_complete:
        if (
            plan.get("sampling_plan_error") is not None
            or record.get("status") != "completed"
            or record.get("load_success") is not True
            or len(states) != expected_total
            or len(schedule) != expected_total
            or unexecuted != 0
            or any(
                _integer(record.get(field), f"v2 ordinal {ordinal}.{field}")
                != expected_counts[phase]
                for phase, field in executed_fields.items()
            )
        ):
            raise ParityAuditError(
                f"v2 ordinal {ordinal} completed measurement does not close to its plan"
            )
        return

    if record.get("status") == "completed":
        raise ParityAuditError(
            f"v2 ordinal {ordinal} incomplete measurement publishes completed status"
        )
    for field in (
        "strict_collision_pass",
        "single_joint_sweep_cf",
        "multi_joint_sobol_cf",
    ):
        if record.get(field) is True:
            raise ParityAuditError(
                f"v2 ordinal {ordinal} retained error is not fail-closed: {field}"
            )
    if not states and (
        record.get("rest_all_pair_cf") is True
        or record.get("rest_non_adjacent_cf") is True
    ):
        raise ParityAuditError(
            f"v2 ordinal {ordinal} publishes a rest pass without a rest state"
        )
    if plan.get("sampling_plan_error") is not None and states:
        raise ParityAuditError(
            f"v2 ordinal {ordinal} invalid sampling plan has executed states"
        )


def _validate_v2_plan_bindings(
    record: Mapping[str, Any],
    states: Sequence[Mapping[str, Any]],
    roster_row: Mapping[str, Any],
    evidence: Mapping[str, Any],
    ordinal: int,
) -> int:
    paths, plan = _mapped_urdf_and_plan(roster_row, evidence, ordinal)
    for field in (
        "independent_dof_count",
        "range_evaluable_independent_dof_count",
        "mimic_joint_count",
        "joint_sampling_plan_sha256",
        "sampling_plan_error",
    ):
        if field not in record or not _json_value_equal(record[field], plan[field]):
            raise ParityAuditError(
                f"v2 ordinal {ordinal} recomputed sampling plan mismatch: {field}"
            )
    independent = int(plan["independent_dof_count"])
    expected_counts = {
        "rest_state_expected": 1,
        "single_state_expected": table4.SINGLE_SAMPLES * independent,
        "sobol_state_expected": table4.SOBOL_SAMPLES if independent else 0,
    }
    for field, expected in expected_counts.items():
        if field not in record or not _json_value_equal(record[field], expected):
            raise ParityAuditError(
                f"v2 ordinal {ordinal} plan-derived denominator mismatch: {field}"
            )
    if (
        record.get("protocol_id") != V2_PROTOCOL_ID
        or record.get("schema_version") != "table4_full_release_run_v2"
        or record.get("sampling_protocol") != V2_SAMPLING_PROTOCOL
        or not _json_value_equal(record.get("roster_ordinal"), ordinal)
    ):
        raise ParityAuditError(f"v2 ordinal {ordinal} protocol/roster binding mismatch")
    expected_input = _expected_v2_input_identity(roster_row, plan)
    if record.get("input_identity_sha256") != expected_input:
        raise ParityAuditError(f"v2 ordinal {ordinal} input identity mismatch")
    expected_execution = canonical_sha256(
        {
            "input_identity_sha256": expected_input,
            "package_root_binding_content_sha256": evidence[
                "binding_content_sha256"
            ],
            "evaluation_package_relative_path": paths[
                "evaluation_package_relative_path"
            ],
            "evaluation_urdf_relative_path": paths["evaluation_urdf_relative_path"],
            "package_binding_sha256": roster_row.get("package_binding_sha256"),
            "expected_primary_urdf_sha256": roster_row.get(
                "primary_urdf_sha256"
            ),
        }
    )
    expected_record_bindings = {
        "category": str(
            roster_row.get("category") or roster_row.get("raw_category") or ""
        ),
        "dataset": "pva",
        "dataset_id": str(roster_row.get("asset_id", "")),
        "expected_movable_joints": int(roster_row.get("joint_count", 0)),
        "movable_dof_count": int(roster_row.get("joint_count", 0)),
        "package": paths["logical_package"],
        "package_binding_sha256": roster_row.get("package_binding_sha256"),
        "primary_urdf_relative_path": str(
            roster_row.get("primary_urdf_relative_path", "model.urdf")
        ),
        "urdf_path": paths["logical_urdf"],
        "evaluation_package_relative_path": paths[
            "evaluation_package_relative_path"
        ],
        "evaluation_urdf_relative_path": paths["evaluation_urdf_relative_path"],
        "package_root_binding_content_sha256": evidence[
            "binding_content_sha256"
        ],
        "execution_input_sha256": expected_execution,
        "expected_primary_urdf_sha256": paths["physical_urdf_sha256"],
        "package_binding_verified": True,
        "runtime_identity": evidence["v2_runtime_identity"],
        "order": ordinal,
    }
    for field, expected in expected_record_bindings.items():
        if field not in record or not _json_value_equal(record[field], expected):
            raise ParityAuditError(
                f"v2 ordinal {ordinal} execution/runtime binding mismatch: {field}"
            )
    if "evaluation_package_path" in record or "evaluation_urdf_path" in record:
        raise ParityAuditError(f"v2 ordinal {ordinal} leaks a physical mirror path")

    phase_counts = Counter(str(state.get("phase")) for state in states)
    expected_executed = {
        "rest": record.get("rest_state_executed"),
        "single_joint_sweep": record.get("single_state_executed"),
        "multi_joint_sobol": record.get("sobol_state_executed"),
    }
    for phase, expected in expected_executed.items():
        if not _json_value_equal(expected, phase_counts.get(phase, 0)):
            raise ParityAuditError(
                f"v2 ordinal {ordinal} state execution coverage mismatch: {phase}"
            )
    for state_index, state in enumerate(states):
        expected_state_bindings = {
            "category": record.get("category"),
            "dataset": "pva",
            "schema_version": "table4_state_v2",
            "protocol_id": V2_PROTOCOL_ID,
            "sampling_protocol": V2_SAMPLING_PROTOCOL,
            "joint_sampling_plan_sha256": plan["joint_sampling_plan_sha256"],
            "input_identity_sha256": expected_input,
            "dataset_id": record.get("dataset_id"),
            "order": ordinal,
        }
        for field, expected in expected_state_bindings.items():
            if field not in state or not _json_value_equal(state[field], expected):
                raise ParityAuditError(
                    f"v2 ordinal {ordinal} state {state_index} binding mismatch: {field}"
                )
    _validate_v2_schedule(record, states, plan, ordinal)
    return int(plan["mimic_joint_count"])


def _framed_update(digest: Any, value: bytes) -> None:
    digest.update(len(value).to_bytes(8, "big"))
    digest.update(value)


def _input_row_update(
    digest: Any,
    ordinal: int,
    asset_id: str,
    record_json: str,
    states_blob: bytes,
    state_count: int,
) -> None:
    for value in (
        str(ordinal).encode("ascii"),
        asset_id.encode("utf-8"),
        record_json.encode("utf-8"),
        states_blob,
        str(state_count).encode("ascii"),
    ):
        _framed_update(digest, value)


def _decode_states(
    blob: Any,
    declared_count: Any,
    record: Mapping[str, Any],
    label: str,
    *,
    allow_legacy_missing_record_count: bool = False,
) -> list[dict[str, Any]]:
    if not isinstance(blob, bytes):
        raise ParityAuditError(f"{label} state payload is not a BLOB")
    count = _integer(declared_count, f"{label}.state_count")
    decompressor = zlib.decompressobj()
    try:
        payload = decompressor.decompress(blob) + decompressor.flush()
    except zlib.error as error:
        raise ParityAuditError(f"{label} state payload is invalid zlib: {error}") from error
    if not decompressor.eof or decompressor.unused_data or decompressor.unconsumed_tail:
        raise ParityAuditError(f"{label} state payload has an invalid zlib boundary")
    try:
        lines = payload.decode("utf-8").splitlines()
    except UnicodeDecodeError as error:
        raise ParityAuditError(f"{label} state payload is not UTF-8: {error}") from error
    if len(lines) != count:
        raise ParityAuditError(
            f"{label} state count mismatch: database={count}, decoded={len(lines)}"
        )
    states = [_json_object(line, f"{label}.state[{index}]") for index, line in enumerate(lines)]
    if "state_records_count" not in record:
        if not allow_legacy_missing_record_count or count != 0:
            raise ParityAuditError(f"{label} record/database state count mismatch")
    else:
        record_count = _integer(
            record.get("state_records_count"), f"{label}.state_records_count"
        )
        if record_count != count:
            raise ParityAuditError(f"{label} record/database state count mismatch")
    if record.get("state_records_sha256") != canonical_sha256(states):
        raise ParityAuditError(f"{label} state_records_sha256 mismatch")
    return states


def _expected_v1_input_identity(roster_row: Mapping[str, Any]) -> str:
    return canonical_sha256(
        {
            "dataset": "pva",
            "dataset_id": str(roster_row.get("asset_id", "")),
            "category": str(
                roster_row.get("category")
                or roster_row.get("raw_category")
                or ""
            ),
            "urdf_path": str(roster_row.get("primary_urdf_path", "")),
            "primary_urdf_relative_path": str(
                roster_row.get("primary_urdf_relative_path", "model.urdf")
            ),
            "expected_primary_urdf_sha256": roster_row.get(
                "primary_urdf_sha256"
            ),
            "expected_movable_joints": int(roster_row.get("joint_count", 0)),
            "package_binding_sha256": roster_row.get("package_binding_sha256"),
        }
    )


def _validate_v1_retained_infrastructure_record(
    record: Mapping[str, Any],
    states: Sequence[Mapping[str, Any]],
    roster_row: Mapping[str, Any],
    ordinal: int,
    worker_status: str,
    expected_timeout_seconds: Any,
) -> str:
    """Validate the strict no-child semantics emitted by the sealed v1 parent."""
    if worker_status not in V1_RETAINED_INFRASTRUCTURE_STATUSES:
        raise ParityAuditError(
            f"v1 ordinal {ordinal} is not a retained infrastructure status"
        )
    dof = int(roster_row.get("joint_count", 0))
    expected = {
        "schema_version": "table4_full_release_run_v1",
        "protocol_id": "urdf_sim_ready_table4_pva_full_release_v1",
        "dataset": "pva",
        "dataset_id": str(roster_row.get("asset_id", "")),
        "order": ordinal,
        "category": str(
            roster_row.get("category") or roster_row.get("raw_category") or ""
        ),
        "package": str(roster_row.get("source_path", "")),
        "urdf_path": str(roster_row.get("primary_urdf_path", "")),
        "primary_urdf_relative_path": str(
            roster_row.get("primary_urdf_relative_path", "model.urdf")
        ),
        "expected_primary_urdf_sha256": roster_row.get("primary_urdf_sha256"),
        "expected_movable_joints": dof,
        "movable_dof_count": dof,
        "package_binding_sha256": roster_row.get("package_binding_sha256"),
        "package_binding_verified": False,
        "input_identity_sha256": _expected_v1_input_identity(roster_row),
        "status": "error",
        "collision_metric_status": table4.BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT,
        "native_collision_elements": 0,
        "load_success": False,
        "measurement_complete": False,
        "range_evaluable_dof_count": 0,
        "rest_state_expected": 1,
        "rest_state_executed": 0,
        "rest_all_pair_cf": None,
        "rest_non_adjacent_cf": None,
        "rest_non_adjacent_free": 0,
        "single_state_expected": table4.SINGLE_SAMPLES * dof,
        "single_state_executed": 0,
        "single_non_adjacent_free": 0,
        "single_joint_sweep_cf": None,
        "joint_single_sweep_cf_passed": 0,
        "sobol_state_expected": table4.SOBOL_SAMPLES if dof else 0,
        "sobol_state_executed": 0,
        "sobol_non_adjacent_free": 0,
        "multi_joint_sobol_cf": None,
        "strict_collision_pass": None,
        "max_penetration_m": None,
        "max_penetration_normalized": None,
        "max_reset_readback_error": None,
        "object_bbox_diagonal_m": None,
        "runtime_identity": None,
        "state_records_sha256": canonical_sha256([]),
    }
    missing = [field for field in expected if field not in record]
    if missing:
        raise ParityAuditError(
            f"v1 ordinal {ordinal} retained infrastructure record is missing fields: "
            + ", ".join(missing)
        )
    mismatched = [
        field
        for field, value in expected.items()
        if not _json_value_equal(record[field], value)
    ]
    if mismatched:
        raise ParityAuditError(
            f"v1 ordinal {ordinal} retained infrastructure record mismatch: "
            + ", ".join(mismatched)
        )
    if states or "state_records_count" in record or "unexecuted_state_count" in record:
        raise ParityAuditError(
            f"v1 ordinal {ordinal} retained infrastructure record has child state accounting"
        )
    if worker_status == "parent_error" and (
        "state_records" not in record or record["state_records"] != []
    ):
        raise ParityAuditError(
            f"v1 ordinal {ordinal} parent_error state_records envelope mismatch"
        )
    if worker_status == "timeout" and "state_records" in record:
        raise ParityAuditError(
            f"v1 ordinal {ordinal} timeout state_records envelope mismatch"
        )
    issues = record.get("issues")
    if not isinstance(issues, list) or len(issues) != 1 or not isinstance(issues[0], str):
        raise ParityAuditError(
            f"v1 ordinal {ordinal} retained infrastructure issue is invalid"
        )
    issue = issues[0]
    if worker_status == "parent_error":
        prefix = "parent_executor_exception: "
        valid_issue = issue.startswith(prefix) and bool(issue[len(prefix) :].strip())
    else:
        manifest_timeout_valid = (
            not isinstance(expected_timeout_seconds, bool)
            and isinstance(expected_timeout_seconds, (int, float))
            and math.isfinite(float(expected_timeout_seconds))
            and float(expected_timeout_seconds) > 0.0
        )
        valid_issue = manifest_timeout_valid and issue == (
            f"asset_timeout_after_{float(expected_timeout_seconds):g}_seconds"
        )
    if not valid_issue:
        raise ParityAuditError(
            f"v1 ordinal {ordinal} retained infrastructure issue/status mismatch"
        )
    return issue


def _validate_v2_completed_reexecution(
    record: Mapping[str, Any], ordinal: int
) -> None:
    expected = {
        "collision_metric_status": table4.MEASURED_COLLISION_GEOMETRY,
        "issues": [],
        "load_success": True,
        "measurement_complete": True,
        "status": "completed",
    }
    mismatched = [
        field
        for field, value in expected.items()
        if field not in record or not _json_value_equal(record[field], value)
    ]
    if mismatched:
        raise ParityAuditError(
            f"v2 ordinal {ordinal} infrastructure reexecution is not complete: "
            + ", ".join(mismatched)
        )


def _finite_nonnegative(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ParityAuditError(f"{label} must be a finite non-negative number")
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ParityAuditError(f"{label} must be a finite non-negative number")
    return number


def _validate_mimic_state_semantics(
    record: Mapping[str, Any],
    states: Sequence[Mapping[str, Any]],
    ordinal: int,
    version: str,
) -> None:
    """Recompute every collision outcome published by a mimic record."""
    label = f"{version} mimic ordinal {ordinal}"
    phases: dict[str, list[Mapping[str, Any]]] = {
        "rest": [], "single_joint_sweep": [], "multi_joint_sobol": []
    }
    max_penetration: float | None = None
    max_reset: float | None = None
    for index, state in enumerate(states):
        phase = state.get("phase")
        if phase not in phases:
            raise ParityAuditError(f"{label} state {index} has invalid phase")
        maxima: dict[str, float] = {}
        for family in ("all_pair", "non_adjacent"):
            contact = _integer(state.get(f"{family}_contact_count"), f"{label} state {index}.{family}_contact_count")
            illegal = _integer(state.get(f"{family}_illegal_penetration_count"), f"{label} state {index}.{family}_illegal_penetration_count")
            if illegal > contact:
                raise ParityAuditError(f"{label} state {index} {family} illegal count exceeds contacts")
            maximum = _finite_nonnegative(state.get(f"{family}_max_penetration_m"), f"{label} state {index}.{family}_max_penetration_m")
            if (illegal == 0) != (maximum <= table4.PENETRATION_THRESHOLD_M):
                raise ParityAuditError(f"{label} state {index} {family} count/max threshold mismatch")
            maxima[family] = maximum
        metric = _finite_nonnegative(state.get("metric_max_penetration_m"), f"{label} state {index}.metric_max_penetration_m")
        expected_metric = maxima["all_pair"] if phase == "rest" else maxima["non_adjacent"]
        if metric != expected_metric:
            raise ParityAuditError(f"{label} state {index} metric selection mismatch")
        reset = _finite_nonnegative(state.get("reset_readback_max_abs_error"), f"{label} state {index}.reset_readback_max_abs_error")
        if reset > table4.RESET_TOLERANCE:
            raise ParityAuditError(f"{label} state {index} reset tolerance exceeded")
        phases[str(phase)].append(state)
        max_penetration = (
            metric if max_penetration is None else max(max_penetration, metric)
        )
        max_reset = reset if max_reset is None else max(max_reset, reset)

    rest, single, sobol = phases["rest"], phases["single_joint_sweep"], phases["multi_joint_sobol"]
    phase_counts = {
        "rest_state_executed": len(rest),
        "single_state_executed": len(single),
        "sobol_state_executed": len(sobol),
    }
    rest_all = len(rest) == 1 and int(rest[0]["all_pair_illegal_penetration_count"]) == 0
    rest_nonadj = len(rest) == 1 and int(rest[0]["non_adjacent_illegal_penetration_count"]) == 0
    single_free = sum(int(state["non_adjacent_illegal_penetration_count"]) == 0 for state in single)
    sobol_free = sum(int(state["non_adjacent_illegal_penetration_count"]) == 0 for state in sobol)
    joints: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for state in single:
        joint_name = state.get("joint_name")
        if not isinstance(joint_name, str) or not joint_name:
            raise ParityAuditError(f"{label} single state has invalid joint_name")
        joints[joint_name].append(state)
    joint_passed = sum(
        len(rows) == table4.SINGLE_SAMPLES
        and all(
            int(state["non_adjacent_illegal_penetration_count"]) == 0
            for state in rows
        )
        for rows in joints.values()
    )
    expected_total = sum(_integer(record.get(field), f"{label}.{field}") for field in ("rest_state_expected", "single_state_expected", "sobol_state_expected"))
    range_field = (
        "range_evaluable_independent_dof_count"
        if "range_evaluable_independent_dof_count" in record
        else "range_evaluable_dof_count"
    )
    dof_field = (
        "independent_dof_count"
        if "independent_dof_count" in record
        else "movable_dof_count"
    )
    range_count = _integer(record.get(range_field), f"{label}.{range_field}")
    dof_count = _integer(record.get(dof_field), f"{label}.{dof_field}")
    unexecuted = _integer(
        record.get("unexecuted_state_count"), f"{label}.unexecuted_state_count"
    )
    executed_expected_pairs = (
        ("rest_state_executed", "rest_state_expected"),
        ("single_state_executed", "single_state_expected"),
        ("sobol_state_executed", "sobol_state_expected"),
    )
    executed_counts = {
        field: _integer(record.get(field), f"{label}.{field}")
        for pair in executed_expected_pairs
        for field in pair
    }
    complete = bool(
        record.get("load_success") is True and record.get("status") == "completed"
        and record.get("issues") == [] and range_count == dof_count
        and len(states) == expected_total and unexecuted == 0
        and all(
            executed_counts[executed] == executed_counts[expected]
            for executed, expected in executed_expected_pairs
        )
    )
    single_cf = bool(
        len(single) == executed_counts["single_state_expected"]
        and single_free == len(single)
    )
    zero_dof = dof_count == 0
    sobol_cf = bool(
        not zero_dof
        and complete
        and len(sobol) == executed_counts["sobol_state_expected"]
        and sobol_free == len(sobol)
    )
    strict = bool(not zero_dof and complete and rest_nonadj and single_cf and sobol_cf)
    metric_status = record.get("collision_metric_status")
    partial_exception = False
    if metric_status == table4.MEASURED_COLLISION_GEOMETRY:
        if not states:
            raise ParityAuditError(f"{label} measured collision metric has no states")
        published_max_penetration = max_penetration
        published_max_reset = max_reset
    elif metric_status == "partial":
        partial_exception = True
        issues = record.get("issues")
        partial_maxima = (
            "max_penetration_m",
            "max_penetration_normalized",
            "max_reset_readback_error",
        )
        if (
            not 0 < len(states) < expected_total
            or unexecuted != expected_total - len(states)
            or record.get("status") != "error"
            or record.get("load_success") is not True
            or record.get("measurement_complete") is not False
            or not isinstance(issues, list)
            or len(issues) != 1
            or not isinstance(issues[0], str)
            or not issues[0]
            or any(field not in record for field in partial_maxima)
            or any(record[field] is not None for field in partial_maxima)
        ):
            raise ParityAuditError(
                f"{label} partial collision metric is not a sealed exception prefix"
            )
        # The runner preserves states observed before an exception, but its
        # aggregate maxima are finalized only after the full sampling loop.
        published_max_penetration = None
        published_max_reset = None
    elif metric_status == table4.BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT:
        if states:
            raise ParityAuditError(f"{label} N/E collision metric unexpectedly has states")
        published_max_penetration = None
        published_max_reset = None
    else:
        raise ParityAuditError(f"{label} has invalid collision_metric_status")

    scale_value = record.get("object_bbox_diagonal_m")
    if scale_value is None:
        scale = None
        if metric_status != table4.BLOCKED_NATIVE_COLLISION_GEOMETRY_ABSENT:
            raise ParityAuditError(f"{label}.object_bbox_diagonal_m is missing")
    else:
        scale = _finite_nonnegative(scale_value, f"{label}.object_bbox_diagonal_m")
        if scale <= 0.0:
            raise ParityAuditError(f"{label}.object_bbox_diagonal_m must be positive")
    recomputed = {
        **phase_counts,
        "rest_all_pair_cf": rest_all, "rest_non_adjacent_cf": rest_nonadj,
        "rest_non_adjacent_free": int(rest_nonadj), "single_non_adjacent_free": single_free,
        "sobol_non_adjacent_free": sobol_free, "joint_single_sweep_cf_passed": joint_passed,
        "single_joint_sweep_cf": False if partial_exception else single_cf,
        "multi_joint_sobol_cf": sobol_cf,
        "measurement_complete": complete, "strict_collision_pass": strict,
        "max_penetration_m": published_max_penetration,
        "max_penetration_normalized": (
            published_max_penetration / scale
            if published_max_penetration is not None and scale is not None
            else None
        ),
        "max_reset_readback_error": published_max_reset,
    }
    missing = [field for field in recomputed if field not in record]
    if missing:
        raise ParityAuditError(
            f"{label} record is missing recomputed fields: {', '.join(missing)}"
        )
    for field, expected in recomputed.items():
        if field not in record or not _json_value_equal(record[field], expected):
            raise ParityAuditError(f"{label} recomputed field mismatch: {field}")


def _phase_pass(value: Any, label: str) -> bool:
    if value is not None and not isinstance(value, bool):
        raise ParityAuditError(f"{label} must be boolean or null")
    return value is True


def _new_phase_accumulator() -> dict[str, Any]:
    return {
        phase: {
            "transitions": Counter(),
            "v1_null": 0,
            "v2_null": 0,
            "improved_ordinals": [],
            "regressed_ordinals": [],
        }
        for phase in PHASE_FIELDS
    }


def _observe_phases(
    accumulator: dict[str, Any],
    ordinal: int,
    v1_record: Mapping[str, Any],
    v2_record: Mapping[str, Any],
) -> None:
    for phase, field in PHASE_FIELDS.items():
        if field not in v1_record or field not in v2_record:
            raise ParityAuditError(f"ordinal {ordinal} is missing phase field {field}")
        v1_raw = v1_record[field]
        v2_raw = v2_record[field]
        v1_pass = _phase_pass(v1_raw, f"v1 ordinal {ordinal}.{field}")
        v2_pass = _phase_pass(v2_raw, f"v2 ordinal {ordinal}.{field}")
        transition = f"{'pass' if v1_pass else 'fail'}_to_{'pass' if v2_pass else 'fail'}"
        accumulator[phase]["transitions"][transition] += 1
        accumulator[phase]["v1_null"] += int(v1_raw is None)
        accumulator[phase]["v2_null"] += int(v2_raw is None)
        if not v1_pass and v2_pass:
            accumulator[phase]["improved_ordinals"].append(ordinal)
        elif v1_pass and not v2_pass:
            accumulator[phase]["regressed_ordinals"].append(ordinal)


def _finalize_phases(
    accumulator: Mapping[str, Any], *, include_ordinals: bool
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for phase in PHASE_FIELDS:
        raw = accumulator[phase]
        transitions = {key: int(raw["transitions"].get(key, 0)) for key in TRANSITION_KEYS}
        denominator = sum(transitions.values())
        v1_passed = transitions["pass_to_fail"] + transitions["pass_to_pass"]
        v2_passed = transitions["fail_to_pass"] + transitions["pass_to_pass"]
        phase_result: dict[str, Any] = {
            "denominator": denominator,
            "transitions": transitions,
            "v1_null_count": int(raw["v1_null"]),
            "v2_null_count": int(raw["v2_null"]),
            "v1_passed": v1_passed,
            "v2_passed": v2_passed,
            "v1_rate": v1_passed / denominator if denominator else None,
            "v2_rate": v2_passed / denominator if denominator else None,
            "passed_delta": v2_passed - v1_passed,
        }
        if include_ordinals:
            phase_result["improved_ordinals"] = list(raw["improved_ordinals"])
            phase_result["regressed_ordinals"] = list(raw["regressed_ordinals"])
        result[phase] = phase_result
    return result


def _new_reexecution_phase_accumulator() -> dict[str, Any]:
    return {
        phase: {
            "outcomes": Counter(),
            "passed_ordinals": [],
            "failed_ordinals": [],
        }
        for phase in PHASE_FIELDS
    }


def _observe_reexecution_phases(
    accumulator: dict[str, Any],
    ordinal: int,
    v1_record: Mapping[str, Any],
    v2_record: Mapping[str, Any],
) -> None:
    for phase, field in PHASE_FIELDS.items():
        if field not in v1_record or v1_record[field] is not None:
            raise ParityAuditError(
                f"v1 ordinal {ordinal}.{field} is not an unobserved recovery outcome"
            )
        if field not in v2_record or not isinstance(v2_record[field], bool):
            raise ParityAuditError(
                f"v2 ordinal {ordinal}.{field} is not a completed recovery outcome"
            )
        passed = v2_record[field] is True
        outcome = "unobserved_to_pass" if passed else "unobserved_to_fail"
        accumulator[phase]["outcomes"][outcome] += 1
        ordinal_field = "passed_ordinals" if passed else "failed_ordinals"
        accumulator[phase][ordinal_field].append(ordinal)


def _finalize_reexecution_phases(
    accumulator: Mapping[str, Any],
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for phase in PHASE_FIELDS:
        raw = accumulator[phase]
        outcomes = {
            key: int(raw["outcomes"].get(key, 0))
            for key in ("unobserved_to_fail", "unobserved_to_pass")
        }
        denominator = sum(outcomes.values())
        v2_passed = outcomes["unobserved_to_pass"]
        result[phase] = {
            "denominator": denominator,
            "outcomes": outcomes,
            "v1_unobserved_count": denominator,
            "v2_passed": v2_passed,
            "v2_rate": v2_passed / denominator if denominator else None,
            "passed_ordinals": list(raw["passed_ordinals"]),
            "failed_ordinals": list(raw["failed_ordinals"]),
        }
    return result


def _atomic_json(path: Path, value: Any) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", dir=str(destination.parent)
    )
    try:
        with os.fdopen(descriptor, "w", encoding="ascii") as stream:
            stream.write(json.dumps(value, indent=2, sort_keys=True, ensure_ascii=True))
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, destination)
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def audit_prefix(
    v1_database: Path,
    v2_database: Path,
    *,
    limit: int | None = None,
    v1_receipt: Path | None = None,
    v2_manifest: Path | None = None,
    v2_receipt: Path | None = None,
) -> dict[str, Any]:
    if limit is not None and (isinstance(limit, bool) or limit <= 0):
        raise ParityAuditError("limit must be a positive integer")

    v1_preflight_path = _regular_file(v1_database, "v1 database")
    v1_preflight_receipt = (
        v1_preflight_path.parent / "full_release_receipt.json"
        if v1_receipt is None
        else Path(v1_receipt)
    )
    v1_preflight_sha = _receipt_database_preflight(
        v1_preflight_path,
        v1_preflight_receipt,
        receipt_schema=V1_RECEIPT_SCHEMA,
        label="v1 full-release",
        v1=True,
    )

    v2_preflight_path = _regular_file(v2_database, "v2 database")
    discovered_v2_receipt = v2_preflight_path.parent / "full_release_receipt.json"
    selected_v2_receipt = (
        discovered_v2_receipt if v2_receipt is None else Path(v2_receipt)
    )
    if v2_receipt is not None or selected_v2_receipt.exists():
        v2_preflight_sha = _receipt_database_preflight(
            v2_preflight_path,
            selected_v2_receipt,
            receipt_schema=V2_RECEIPT_SCHEMA,
            label="v2 full-release",
            v1=False,
        )
        v2_immutable = True
    else:
        v2_preflight_sha = None
        v2_immutable = False

    v1, v1_path = connect_read_only(
        v1_preflight_path, "v1 database", immutable=True
    )
    try:
        v2, v2_path = connect_read_only(
            v2_preflight_path, "v2 database", immutable=v2_immutable
        )
    except BaseException:
        v1.close()
        raise
    try:
        _require_columns(
            v1,
            "assets",
            ("ordinal", "asset_id", "category", "row_sha256", "row_json"),
            "v1 database",
        )
        _require_columns(
            v1,
            "results",
            (
                "ordinal",
                "asset_id",
                "table4_json",
                "table4_states_zlib",
                "table4_state_count",
                "worker_status",
            ),
            "v1 database",
        )
        _require_columns(
            v2,
            "results",
            (
                "ordinal",
                "asset_id",
                "record_json",
                "states_zlib",
                "state_count",
                "worker_status",
            ),
            "v2 database",
        )
        v1_meta = _read_meta(v1, "v1 database")
        v2_meta = _read_meta(v2, "v2 database")
        if v1_meta.get("schema_version") != V1_DATABASE_SCHEMA:
            raise ParityAuditError("unexpected v1 database schema_version")
        if v2_meta.get("schema_version") != V2_DATABASE_SCHEMA:
            raise ParityAuditError("unexpected v2 database schema_version")
        for key in ("N_eval", "J_eval", "roster_manifest_content_sha256"):
            v1_key = {"N_eval": "selected_n", "J_eval": "selected_j"}.get(key, key)
            if not _json_value_equal(v1_meta.get(v1_key), v2_meta.get(key)):
                raise ParityAuditError(f"v1/v2 database meta mismatch: {key}")
        evidence = _verify_evidence_chain(
            v1_path,
            v2_path,
            v1_meta,
            v2_meta,
            v1_receipt_path=v1_receipt,
            v2_manifest_path=v2_manifest,
            v2_receipt_path=v2_receipt,
            preverified_v1_database_sha=v1_preflight_sha,
            preverified_v2_database_sha=v2_preflight_sha,
        )

        available_count, minimum, maximum = v2.execute(
            "SELECT COUNT(*), MIN(ordinal), MAX(ordinal) FROM results"
        ).fetchone()
        available = _integer(available_count, "v2 result count")
        if available <= 0:
            raise ParityAuditError("v2 database contains no completed result prefix")
        if int(minimum) != 0 or int(maximum) != available - 1:
            raise ParityAuditError("v2 results are not a contiguous zero-based prefix")
        audited = available if limit is None else limit
        if audited > available:
            raise ParityAuditError(
                f"requested limit {audited} exceeds v2 completed prefix {available}"
            )

        v1_cursor = v1.execute(
            "SELECT a.ordinal, a.asset_id, a.category, a.row_sha256, a.row_json, "
            "r.asset_id, r.table4_json, r.table4_states_zlib, "
            "r.table4_state_count, r.worker_status "
            "FROM assets AS a JOIN results AS r ON r.ordinal = a.ordinal "
            "WHERE a.ordinal < ? ORDER BY a.ordinal",
            (audited,),
        )
        v2_cursor = v2.execute(
            "SELECT ordinal, asset_id, record_json, states_zlib, state_count, "
            "worker_status FROM results WHERE ordinal < ? ORDER BY ordinal",
            (audited,),
        )

        v1_input_digest = hashlib.sha256()
        v2_input_digest = hashlib.sha256()
        identity_digest = hashlib.sha256()
        roster_digest = hashlib.sha256()
        v1_state_semantic_digest = hashlib.sha256()
        v2_state_semantic_digest = hashlib.sha256()
        worker_statuses = {"v1": Counter(), "v2": Counter()}
        mimic_counts: Counter[int] = Counter()
        static_mismatches: list[dict[str, Any]] = []
        record_mismatches: list[dict[str, Any]] = []
        mimic_integrity_mismatches: list[dict[str, Any]] = []
        state_mismatches: list[dict[str, Any]] = []
        retained_infrastructure_reexecutions: list[dict[str, Any]] = []
        all_phases = _new_phase_accumulator()
        non_mimic_phases = _new_phase_accumulator()
        comparable_non_mimic_phases = _new_phase_accumulator()
        retained_infrastructure_phases = _new_reexecution_phase_accumulator()
        mimic_phases = _new_phase_accumulator()
        strata_phases: dict[int, dict[str, Any]] = defaultdict(_new_phase_accumulator)
        non_mimic_state_count = 0

        for ordinal in range(audited):
            v1_row = v1_cursor.fetchone()
            v2_row = v2_cursor.fetchone()
            if v1_row is None or v2_row is None:
                raise ParityAuditError(f"database ended before ordinal {ordinal}")
            (
                v1_ordinal,
                v1_asset_id,
                v1_category,
                roster_row_sha,
                roster_row_json,
                v1_result_id,
                v1_record_json,
                v1_blob,
                v1_state_count,
                v1_worker_status,
            ) = v1_row
            (
                v2_ordinal,
                v2_asset_id,
                v2_record_json,
                v2_blob,
                v2_state_count,
                v2_worker_status,
            ) = v2_row
            if int(v1_ordinal) != ordinal or int(v2_ordinal) != ordinal:
                raise ParityAuditError(f"non-contiguous ordinal at {ordinal}")
            asset_id = str(v1_asset_id)
            if asset_id != str(v1_result_id) or asset_id != str(v2_asset_id):
                raise ParityAuditError(f"asset identity mismatch at ordinal {ordinal}")

            roster_row = _json_object(roster_row_json, f"v1 roster ordinal {ordinal}")
            if canonical_sha256(roster_row) != str(roster_row_sha):
                raise ParityAuditError(f"v1 roster row hash mismatch at ordinal {ordinal}")
            if (
                not _json_value_equal(roster_row.get("ordinal"), ordinal)
                or roster_row.get("asset_id") != asset_id
                or str(roster_row.get("raw_category", roster_row.get("category", "")))
                != str(v1_category)
            ):
                raise ParityAuditError(f"v1 roster row binding mismatch at ordinal {ordinal}")
            _framed_update(roster_digest, str(roster_row_sha).encode("ascii"))

            v1_record = _json_object(v1_record_json, f"v1 record ordinal {ordinal}")
            v2_record = _json_object(v2_record_json, f"v2 record ordinal {ordinal}")
            for version, record in (("v1", v1_record), ("v2", v2_record)):
                if (
                    not _json_value_equal(record.get("order"), ordinal)
                    or record.get("dataset_id") != asset_id
                ):
                    raise ParityAuditError(
                        f"{version} record identity mismatch at ordinal {ordinal}"
                    )
            if v1_record.get("category") != str(v1_category):
                raise ParityAuditError(f"v1 category mismatch at ordinal {ordinal}")
            v1_worker = str(v1_worker_status)
            v2_worker = str(v2_worker_status)
            v1_retained_infrastructure = (
                v1_worker in V1_RETAINED_INFRASTRUCTURE_STATUSES
            )
            if (
                v1_record.get("schema_version") != "table4_full_release_run_v1"
                or v1_record.get("protocol_id")
                != "urdf_sim_ready_table4_pva_full_release_v1"
                or (
                    not v1_retained_infrastructure
                    and not _json_value_equal(
                        v1_record.get("runtime_identity"),
                        evidence["v1_runtime_identity"],
                    )
                )
            ):
                raise ParityAuditError(
                    f"v1 ordinal {ordinal} protocol/runtime binding mismatch"
                )
            if (
                v1_worker not in SUCCESSFUL_WORKER_STATUSES
                and not v1_retained_infrastructure
            ):
                raise ParityAuditError(
                    f"v1 ordinal {ordinal} unsuccessful worker status: {v1_worker_status}"
                )
            if v2_worker not in SUCCESSFUL_WORKER_STATUSES:
                raise ParityAuditError(
                    f"v2 ordinal {ordinal} unsuccessful worker status: {v2_worker_status}"
                )
            worker_statuses["v1"][v1_worker] += 1
            worker_statuses["v2"][v2_worker] += 1
            _input_row_update(
                v1_input_digest,
                ordinal,
                asset_id,
                str(v1_record_json),
                bytes(v1_blob),
                int(v1_state_count),
            )
            _input_row_update(
                v2_input_digest,
                ordinal,
                asset_id,
                str(v2_record_json),
                bytes(v2_blob),
                int(v2_state_count),
            )
            if not v1_retained_infrastructure:
                static_v1 = _project(
                    v1_record,
                    COMMON_STATIC_RECORD_FIELDS,
                    f"v1 record ordinal {ordinal}",
                )
                static_v2 = _project(
                    v2_record,
                    COMMON_STATIC_RECORD_FIELDS,
                    f"v2 record ordinal {ordinal}",
                )
                static_difference_fields = _strict_field_differences(
                    static_v1, static_v2, COMMON_STATIC_RECORD_FIELDS
                )
                if static_difference_fields:
                    static_mismatches.append(
                        {
                            "asset_id": asset_id,
                            "fields": static_difference_fields,
                            "ordinal": ordinal,
                        }
                    )

            v1_states = _decode_states(
                bytes(v1_blob),
                int(v1_state_count),
                v1_record,
                f"v1 ordinal {ordinal}",
                allow_legacy_missing_record_count=v1_retained_infrastructure,
            )
            v2_states = _decode_states(
                bytes(v2_blob),
                int(v2_state_count),
                v2_record,
                f"v2 ordinal {ordinal}",
            )
            mimic_count = _validate_v2_plan_bindings(
                v2_record,
                v2_states,
                roster_row,
                evidence,
                ordinal,
            )
            mimic_counts[mimic_count] += 1
            _framed_update(
                identity_digest,
                canonical_text(
                    {
                        "asset_id": asset_id,
                        "mimic_joint_count": mimic_count,
                        "ordinal": ordinal,
                    }
                ).encode("ascii"),
            )

            _observe_phases(all_phases, ordinal, v1_record, v2_record)
            _observe_phases(strata_phases[mimic_count], ordinal, v1_record, v2_record)
            if v1_retained_infrastructure:
                if mimic_count != 0:
                    raise ParityAuditError(
                        f"v1 ordinal {ordinal} retained infrastructure row is a mimic asset"
                    )
                issue = _validate_v1_retained_infrastructure_record(
                    v1_record,
                    v1_states,
                    roster_row,
                    ordinal,
                    v1_worker,
                    evidence["v1_manifest"].get("asset_timeout_seconds"),
                )
                if (
                    "range_evaluable_dof_count" not in v2_record
                    or "range_evaluable_independent_dof_count" not in v2_record
                    or not _json_value_equal(
                        v2_record["range_evaluable_dof_count"],
                        v2_record["range_evaluable_independent_dof_count"],
                    )
                ):
                    raise ParityAuditError(
                        f"v2 ordinal {ordinal} non-mimic reexecution range count mismatch"
                    )
                _validate_v2_completed_reexecution(v2_record, ordinal)
                _validate_mimic_state_semantics(
                    v2_record,
                    v2_states,
                    ordinal,
                    "v2 infrastructure reexecution",
                )
                _observe_phases(
                    non_mimic_phases, ordinal, v1_record, v2_record
                )
                _observe_reexecution_phases(
                    retained_infrastructure_phases,
                    ordinal,
                    v1_record,
                    v2_record,
                )
                retained_infrastructure_reexecutions.append(
                    {
                        "asset_id": asset_id,
                        "mimic_joint_count": 0,
                        "ordinal": ordinal,
                        "v1_issue": issue,
                        "v1_worker_status": v1_worker,
                        "v2_measurement_complete": v2_record.get(
                            "measurement_complete"
                        ),
                        "v2_state_count": len(v2_states),
                        "v2_status": v2_record.get("status"),
                        "v2_strict_collision_pass": _phase_pass(
                            v2_record.get("strict_collision_pass"),
                            f"v2 ordinal {ordinal}.strict_collision_pass",
                        ),
                        "v2_worker_status": v2_worker,
                    }
                )
                continue
            if mimic_count:
                _validate_mimic_state_semantics(v1_record, v1_states, ordinal, "v1")
                _validate_mimic_state_semantics(v2_record, v2_states, ordinal, "v2")
                _observe_phases(mimic_phases, ordinal, v1_record, v2_record)
                integrity_v1 = _project(
                    v1_record,
                    MIMIC_RECORD_FIELDS_REQUIRED_EQUAL,
                    f"v1 mimic record ordinal {ordinal}",
                )
                integrity_v2 = _project(
                    v2_record,
                    MIMIC_RECORD_FIELDS_REQUIRED_EQUAL,
                    f"v2 mimic record ordinal {ordinal}",
                )
                integrity_difference_fields = _strict_field_differences(
                    integrity_v1,
                    integrity_v2,
                    MIMIC_RECORD_FIELDS_REQUIRED_EQUAL,
                )
                if integrity_difference_fields:
                    mimic_integrity_mismatches.append(
                        {
                            "asset_id": asset_id,
                            "fields": integrity_difference_fields,
                            "ordinal": ordinal,
                        }
                    )
                continue

            _observe_phases(non_mimic_phases, ordinal, v1_record, v2_record)
            _observe_phases(
                comparable_non_mimic_phases,
                ordinal,
                v1_record,
                v2_record,
            )
            semantic_v1 = _project(
                v1_record,
                NON_MIMIC_RECORD_SEMANTIC_FIELDS,
                f"v1 record ordinal {ordinal}",
            )
            semantic_v2 = _project(
                v2_record,
                NON_MIMIC_RECORD_SEMANTIC_FIELDS,
                f"v2 record ordinal {ordinal}",
            )
            semantic_difference_fields = _strict_field_differences(
                semantic_v1, semantic_v2, NON_MIMIC_RECORD_SEMANTIC_FIELDS
            )
            if semantic_difference_fields:
                record_mismatches.append(
                    {
                        "asset_id": asset_id,
                        "fields": semantic_difference_fields,
                        "ordinal": ordinal,
                    }
                )

            if len(v1_states) != len(v2_states):
                state_mismatches.append(
                    {
                        "asset_id": asset_id,
                        "ordinal": ordinal,
                        "reason": "state_count",
                        "v1": len(v1_states),
                        "v2": len(v2_states),
                    }
                )
                continue
            first_state_difference: dict[str, Any] | None = None
            for state_index, (v1_state, v2_state) in enumerate(zip(v1_states, v2_states)):
                projected_v1 = _project(
                    v1_state,
                    NON_MIMIC_STATE_SEMANTIC_FIELDS,
                    f"v1 ordinal {ordinal} state {state_index}",
                )
                projected_v2 = _project(
                    v2_state,
                    NON_MIMIC_STATE_SEMANTIC_FIELDS,
                    f"v2 ordinal {ordinal} state {state_index}",
                )
                encoded_v1 = canonical_text(projected_v1).encode("ascii")
                encoded_v2 = canonical_text(projected_v2).encode("ascii")
                _framed_update(v1_state_semantic_digest, encoded_v1)
                _framed_update(v2_state_semantic_digest, encoded_v2)
                non_mimic_state_count += 1
                state_difference_fields = _strict_field_differences(
                    projected_v1,
                    projected_v2,
                    NON_MIMIC_STATE_SEMANTIC_FIELDS,
                )
                if first_state_difference is None and state_difference_fields:
                    first_state_difference = {
                        "asset_id": asset_id,
                        "fields": state_difference_fields,
                        "ordinal": ordinal,
                        "state_index": state_index,
                    }
            if first_state_difference is not None:
                state_mismatches.append(first_state_difference)

        if v1_cursor.fetchone() is not None or v2_cursor.fetchone() is not None:
            raise ParityAuditError("prefix cursor returned more rows than requested")

        non_mimic_count = int(mimic_counts.get(0, 0))
        mimic_asset_count = audited - non_mimic_count
        state_hash_v1 = v1_state_semantic_digest.hexdigest()
        state_hash_v2 = v2_state_semantic_digest.hexdigest()
        all_pass = (
            not static_mismatches
            and not record_mismatches
            and not mimic_integrity_mismatches
            and not state_mismatches
        )
        report: dict[str, Any] = {
            "schema_version": AUDIT_SCHEMA,
            "all_pass": all_pass,
            "audited_prefix": {
                "first_ordinal": 0,
                "last_ordinal": audited - 1,
                "asset_count": audited,
                "identity_content_sha256": identity_digest.hexdigest(),
                "v1_roster_row_hashes_sha256": roster_digest.hexdigest(),
            },
            "inputs": {
                "v1_database": str(v1_path),
                "v2_database": str(v2_path),
                "v1_meta": v1_meta,
                "v2_meta": v2_meta,
                "v1_full_release_receipt": str(evidence["v1_receipt_path"]),
                "v1_full_release_receipt_file_sha256": evidence[
                    "v1_receipt_file_sha256"
                ],
                "v1_full_release_receipt_content_sha256": evidence["v1_receipt"][
                    "receipt_content_sha256"
                ],
                "v1_database_full_file_sha256": evidence["v1_database_sha256"],
                "v1_sqlite_sidecars": evidence["v1_sidecars"],
                "v1_execution_manifest": str(evidence["v1_manifest_path"]),
                "v1_execution_manifest_file_sha256": evidence[
                    "v1_manifest_file_sha256"
                ],
                "v1_execution_manifest_content_sha256": evidence[
                    "v1_manifest_content_sha256"
                ],
                "v1_roster_manifest": str(evidence["roster_path"]),
                "v1_roster_manifest_file_sha256": evidence["roster_file_sha256"],
                "v1_roster_manifest_content_sha256": evidence[
                    "roster_content_sha256"
                ],
                "v2_execution_manifest": str(evidence["v2_manifest_path"]),
                "v2_execution_manifest_file_sha256": evidence[
                    "v2_manifest_file_sha256"
                ],
                "v2_execution_manifest_content_sha256": evidence[
                    "v2_manifest_content_sha256"
                ],
                "v2_package_root_binding_content_sha256": evidence[
                    "binding_content_sha256"
                ],
                "v2_logical_package_root": str(evidence["logical_root"]),
                "v2_physical_package_root": str(evidence["physical_root"]),
                "v2_observed_frozen_source_hashes": evidence[
                    "observed_source_hashes"
                ],
                "v2_runtime_identity": evidence["v2_runtime_identity"],
                "v2_full_release_receipt_verified": evidence["v2_receipt_verified"],
                "v2_full_release_receipt": (
                    str(evidence["v2_receipt_path"])
                    if evidence["v2_receipt_path"] is not None else None
                ),
                "v2_full_release_receipt_file_sha256": evidence.get(
                    "v2_receipt_file_sha256"
                ),
                "v2_full_release_receipt_content_sha256": evidence.get(
                    "v2_receipt_content_sha256"
                ),
                "v2_receipt_auto_discovered": evidence["v2_receipt_auto_discovered"],
                "v2_database_full_file_sha256": evidence[
                    "v2_database_full_file_sha256"
                ],
                "v2_artifact_manifest": (
                    str(evidence["v2_artifact_manifest_path"])
                    if evidence.get("v2_artifact_manifest_path") is not None else None
                ),
                "v2_artifact_manifest_file_sha256": evidence.get(
                    "v2_artifact_manifest_file_sha256"
                ),
                "v2_artifact_manifest_content_sha256": evidence.get(
                    "v2_artifact_manifest_content_sha256"
                ),
                "v2_sqlite_sidecars": evidence["v2_sidecars"],
                "v1_prefix_raw_content_sha256": v1_input_digest.hexdigest(),
                "v2_prefix_raw_content_sha256": v2_input_digest.hexdigest(),
                "database_file_sha256_scope": (
                    "both full database files are receipt-sealed; the framed prefix "
                    "hash additionally binds the audited transaction"
                    if evidence["v2_receipt_verified"]
                    else "v1 full database bytes are sealed above; v2 has no verified "
                    "receipt, so its framed raw record/state prefix hash binds every "
                    "audited result in the fixed read transaction"
                ),
            },
            "comparison_contract": {
                "common_static_record_fields": list(COMMON_STATIC_RECORD_FIELDS),
                "non_mimic_record_semantic_fields": list(
                    NON_MIMIC_RECORD_SEMANTIC_FIELDS
                ),
                "non_mimic_state_semantic_fields": list(
                    NON_MIMIC_STATE_SEMANTIC_FIELDS
                ),
                "allowed_common_record_differences": list(
                    ALLOWED_COMMON_RECORD_DIFFERENCES
                ),
                "v2_only_record_fields": list(V2_ONLY_RECORD_FIELDS),
                "mimic_record_fields_allowed_to_change": list(
                    MIMIC_RECORD_FIELDS_ALLOWED_TO_CHANGE
                ),
                "mimic_record_fields_required_equal": list(
                    MIMIC_RECORD_FIELDS_REQUIRED_EQUAL
                ),
                "allowed_common_state_differences": list(
                    ALLOWED_COMMON_STATE_DIFFERENCES
                ),
                "v2_only_state_fields": list(V2_ONLY_STATE_FIELDS),
                "mimic_classification_source": (
                    "structured parse of the SHA256-bound physical mirror URDF and "
                    "frozen v2 compile_joint_sampling_plan; record mimic_joint_count "
                    "is never trusted"
                ),
                "worker_status_gate": {
                    "v1_executed": sorted(SUCCESSFUL_WORKER_STATUSES),
                    "v1_retained_no_child": sorted(
                        V1_RETAINED_INFRASTRUCTURE_STATUSES
                    ),
                    "v2_executed": sorted(SUCCESSFUL_WORKER_STATUSES),
                },
                "v1_retained_infrastructure_policy": (
                    "receipt-sealed v1 parent_error/timeout rows must match the strict "
                    "status-specific no-child semantic contract, have zero states, and "
                    "classify as non-mimic; their roster-bound, fully completed v2 "
                    "reexecutions are reported as unobserved outcomes and excluded from "
                    "non-mimic semantic equality hashes"
                ),
                "phase_accounting_policy": {
                    "release_denominator_reconciliation": [
                        "all_assets",
                        "non_mimic_assets",
                        "by_mimic_joint_count[0]",
                    ],
                    "release_reconciliation_null_policy": (
                        "historical null phase outcomes are fail-closed for exact published "
                        "summary numerator reconciliation and are not causal attribution"
                    ),
                    "mimic_protocol_attribution": "mimic_assets",
                    "non_mimic_parity_control": "non_mimic_comparable_assets",
                    "infrastructure_reexecution_attribution": (
                        "v1_retained_infrastructure_reexecutions uses unobserved_to_pass/"
                        "unobserved_to_fail, never fail_to_*"
                    ),
                },
                "v2_state_schedule_replay": {
                    "rest_states": 1,
                    "single_samples_per_independent_joint": table4.SINGLE_SAMPLES,
                    "sobol_samples": table4.SOBOL_SAMPLES,
                    "sobol_seed": table4.SOBOL_SEED,
                    "validated_fields": [
                        "phase",
                        "sample_index",
                        "joint_name",
                        "joint_values_sha256",
                        "joint_sampling_plan_sha256",
                        "input_identity_sha256",
                    ],
                    "partial_result_policy": (
                        "states must be an exact prefix of the recomputed schedule; "
                        "completed measurements require full closure, retained errors "
                        "require exact unexecuted accounting and fail-closed passes"
                    ),
                },
                "mimic_transitions_are_observational": True,
                "mimic_state_semantic_replay": {
                    "versions": ["v1", "v2"],
                    "collision_contract": "contact/illegal counts, finite maxima, threshold consistency, and phase-selected metric",
                    "record_fields_recomputed": [
                        "rest/single/sobol executed and free counts",
                        "rest/single/sobol/strict pass flags",
                        "joint_single_sweep_cf_passed",
                        "measurement_complete",
                        "max_penetration_m and normalized value",
                        "max_reset_readback_error",
                    ],
                    "reset_tolerance": table4.RESET_TOLERANCE,
                },
            },
            "stratification": {
                "non_mimic_asset_count": non_mimic_count,
                "non_mimic_comparable_asset_count": non_mimic_count
                - len(retained_infrastructure_reexecutions),
                "mimic_asset_count": mimic_asset_count,
                "by_mimic_joint_count": {
                    str(count): {
                        "asset_count": int(mimic_counts[count]),
                        "phase_migrations": _finalize_phases(
                            strata_phases[count], include_ordinals=False
                        ),
                    }
                    for count in sorted(mimic_counts)
                },
            },
            "phase_migrations": {
                "all_assets": _finalize_phases(all_phases, include_ordinals=False),
                "non_mimic_assets": _finalize_phases(
                    non_mimic_phases, include_ordinals=False
                ),
                "non_mimic_comparable_assets": _finalize_phases(
                    comparable_non_mimic_phases, include_ordinals=False
                ),
                "v1_retained_infrastructure_reexecutions": (
                    _finalize_reexecution_phases(retained_infrastructure_phases)
                ),
                "mimic_assets": _finalize_phases(
                    mimic_phases, include_ordinals=True
                ),
            },
            "parity": {
                "static_record_mismatch_count": len(static_mismatches),
                "static_record_mismatches": static_mismatches,
                "non_mimic_record_mismatch_count": len(record_mismatches),
                "non_mimic_record_mismatches": record_mismatches,
                "mimic_integrity_mismatch_count": len(
                    mimic_integrity_mismatches
                ),
                "mimic_integrity_mismatches": mimic_integrity_mismatches,
                "non_mimic_state_count": non_mimic_state_count,
                "non_mimic_state_mismatch_count": len(state_mismatches),
                "non_mimic_state_mismatches": state_mismatches,
                "v1_non_mimic_state_semantic_sha256": state_hash_v1,
                "v2_non_mimic_state_semantic_sha256": state_hash_v2,
                "non_mimic_state_semantic_hashes_match": state_hash_v1
                == state_hash_v2,
                "v1_retained_infrastructure_reexecution_count": len(
                    retained_infrastructure_reexecutions
                ),
                "v1_retained_infrastructure_reexecutions": (
                    retained_infrastructure_reexecutions
                ),
            },
            "worker_status_counts": {
                version: dict(sorted(counter.items()))
                for version, counter in worker_statuses.items()
            },
            "publication_status": {
                "interim_non_durable_active_snapshot": not evidence[
                    "v2_receipt_verified"
                ],
                "final_publication_eligible": bool(
                    all_pass
                    and evidence["v2_receipt_verified"]
                    and evidence["v2_sidecars"]["sealed_sidecar_policy_satisfied"]
                    and limit is None
                    and audited == int(v2_meta["N_eval"])
                    and evidence["v2_manifest"].get("limit") is None
                    and evidence["v2_manifest"].get("classification")
                    == "FORMAL_FULL_RELEASE"
                ),
                "replacement_policy": "replace with a sealed full-prefix audit after the formal run closes",
            },
            "implementation": {
                "script": str(SCRIPT),
                "script_sha256": sha256_file(SCRIPT),
                "sqlite_access": (
                    "receipt-and-main-file-SHA-verified sealed databases use "
                    "mode=ro&immutable=1; active v2 prefixes use mode=ro; all "
                    "connections enforce PRAGMA query_only=ON and an explicit "
                    "read transaction"
                ),
                "prefix_hash_framing": "uint64-be byte length followed by raw field bytes",
                "v2_plan_recomputation": (
                    "SHA256-bound physical mirror URDF parsed by the frozen Table 4 "
                    "core compile_joint_sampling_plan"
                ),
                "v2_schedule_replay": (
                    "implemented in this self-hashed audit script using the manifest-"
                    "bound Table 4 core and manifest-bound SciPy version/Sobol seed"
                ),
            },
        }
        report["audit_content_sha256"] = canonical_sha256(report)
        return report
    finally:
        try:
            v2.rollback()
        finally:
            v2.close()
        try:
            v1.rollback()
        finally:
            v1.close()


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "v1_database",
        nargs="?",
        type=Path,
        default=DEFAULT_V1_DATABASE,
        help="PV-A v1 source evaluation/results.sqlite3",
    )
    parser.add_argument(
        "v2_database",
        nargs="?",
        type=Path,
        default=DEFAULT_V2_DATABASE,
        help="PV-A mimic-aware v2 results.sqlite3",
    )
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument(
        "--v1-receipt",
        type=Path,
        default=None,
        help="defaults to full_release_receipt.json beside the v1 database",
    )
    parser.add_argument(
        "--v2-manifest",
        type=Path,
        default=None,
        help="defaults to manifest.json beside the v2 database",
    )
    parser.add_argument(
        "--v2-receipt",
        type=Path,
        default=None,
        help=(
            "defaults to full_release_receipt.json beside v2 when present; "
            "an absent receipt leaves the audit in prefix-only mode"
        ),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        report = audit_prefix(
            args.v1_database,
            args.v2_database,
            limit=args.limit,
            v1_receipt=args.v1_receipt,
            v2_manifest=args.v2_manifest,
            v2_receipt=args.v2_receipt,
        )
    except (ParityAuditError, sqlite3.Error, OSError) as error:
        print(f"ERROR: {error}", file=os.sys.stderr)
        return 2
    if args.json_out is not None:
        _atomic_json(args.json_out, report)
    print(json.dumps(report, indent=2, sort_keys=True, ensure_ascii=True))
    return 0 if report["all_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
